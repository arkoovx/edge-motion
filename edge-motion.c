#define _GNU_SOURCE
#include <errno.h>
#include <ctype.h>
#include <fcntl.h>
#include <getopt.h>
#include <libevdev/libevdev.h>
#include <math.h>
#include <linux/uinput.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/resource.h>
#include <time.h>
#include <libudev.h>
#include <unistd.h>

#define EDGE_MOTION_VERSION "1.3.1"

#define DEFAULT_EDGE_THRESHOLD 0.06
#define DEFAULT_EDGE_HYSTERESIS 0.015
#define DEFAULT_HOLD_MS 80
#define DEFAULT_PULSE_MS 10
#define DEFAULT_PULSE_STEP 1.5
#define DEFAULT_MAX_SPEED 3.0
#define TOUCHPAD_DISCONNECT_TIMEOUT_MS 200
#define TOUCHPAD_REOPEN_POLL_MS 250
#define UINPUT_SETTLE_MS 50
#define RESOURCE_CHECK_INTERVAL_MS 1000
#define DEFAULT_MAX_RSS_MB 256
#define DEFAULT_MAX_CPU_PERCENT 90.0
#define DEFAULT_RESOURCE_GRACE_CHECKS 5
#define DEFAULT_BUTTON_ZONE 0.14
#define DEFAULT_BUTTON_COOLDOWN_MS 180

static double edge_threshold = DEFAULT_EDGE_THRESHOLD;
static double edge_hysteresis = DEFAULT_EDGE_HYSTERESIS;
static int hold_ms = DEFAULT_HOLD_MS;
static int pulse_ms = DEFAULT_PULSE_MS;
static double pulse_step = DEFAULT_PULSE_STEP;
static double max_speed = DEFAULT_MAX_SPEED;
static int verbose = 0;
static int list_devices = 0;
static int use_grab = 0;
static char *forced_devnode = NULL;
static int diagonal_scroll = 0;
static int natural_scroll = 0;
static int two_finger_scroll = 0;
static double deadzone = 0.0;
static double threshold_left = -1.0;
static double threshold_right = -1.0;
static double threshold_top = -1.0;
static double threshold_bottom = -1.0;
static double accel_exponent = 1.0;
static double pressure_boost = 0.0;
static int daemon_mode = 0;
static int resource_guard_enabled = 1;
static int max_rss_mb = DEFAULT_MAX_RSS_MB;
static double max_cpu_percent = DEFAULT_MAX_CPU_PERCENT;
static int resource_grace_checks = DEFAULT_RESOURCE_GRACE_CHECKS;
static double button_zone = DEFAULT_BUTTON_ZONE;
static int button_cooldown_ms = DEFAULT_BUTTON_COOLDOWN_MS;
static char **ignored_devnodes = NULL;
static size_t ignored_devnode_count = 0;

enum em_mode {
    EM_MODE_MOTION = 0,
    EM_MODE_SCROLL = 1,
};

static enum em_mode mode = EM_MODE_MOTION;

enum scroll_priority {
    SCROLL_PRIORITY_DOMINANT = 0,
    SCROLL_PRIORITY_HORIZONTAL = 1,
    SCROLL_PRIORITY_VERTICAL = 2,
};

static enum scroll_priority scroll_priority = SCROLL_PRIORITY_DOMINANT;

static volatile sig_atomic_t running = 1;

struct em_state {
    pthread_mutex_t lock;
    pthread_cond_t cond;
    int edge_active;
    int dir_x;
    int dir_y;
    double speed_factor;
};

struct touchpad_resources {
    char *devnode;
    int input_fd;
    struct libevdev *dev;
};

struct touchpad_candidate {
    char *devnode;
    char *name;
    int integrated;
    int min_x;
    int max_x;
    int min_y;
    int max_y;
    long long area;
    int has_finger_tool;
    int has_btn_touch;
    int is_mouse_like;
};

struct resource_guard_state {
    double last_cpu_seconds;
    struct timespec last_ts;
    int initialized;
    int consecutive_over_limit;
};

static inline int64_t timespec_to_ms(const struct timespec *ts);

static struct em_state state = {
    .lock = PTHREAD_MUTEX_INITIALIZER,
    .edge_active = 0,
    .dir_x = 0,
    .dir_y = 0,
    .speed_factor = 0.0,
};

static int parse_mode(const char *value, enum em_mode *out)
{
    if (strcmp(value, "motion") == 0) {
        *out = EM_MODE_MOTION;
        return 0;
    }
    if (strcmp(value, "scroll") == 0) {
        *out = EM_MODE_SCROLL;
        return 0;
    }

    return -1;
}

static int parse_scroll_priority(const char *value, enum scroll_priority *out)
{
    if (strcmp(value, "dominant") == 0) {
        *out = SCROLL_PRIORITY_DOMINANT;
        return 0;
    }
    if (strcmp(value, "horizontal") == 0) {
        *out = SCROLL_PRIORITY_HORIZONTAL;
        return 0;
    }
    if (strcmp(value, "vertical") == 0) {
        *out = SCROLL_PRIORITY_VERTICAL;
        return 0;
    }

    return -1;
}

static int add_ignored_devnode(const char *value)
{
    if (!value || value[0] != '/')
        return -1;

    char *copy = strdup(value);
    if (!copy)
        return -1;

    char **tmp = realloc(ignored_devnodes, (ignored_devnode_count + 1) * sizeof(*ignored_devnodes));
    if (!tmp) {
        free(copy);
        return -1;
    }

    ignored_devnodes = tmp;
    ignored_devnodes[ignored_devnode_count++] = copy;
    return 0;
}

static int is_ignored_devnode(const char *devnode)
{
    for (size_t i = 0; i < ignored_devnode_count; i++) {
        if (strcmp(ignored_devnodes[i], devnode) == 0)
            return 1;
    }
    return 0;
}

static void free_ignored_devnodes(void)
{
    for (size_t i = 0; i < ignored_devnode_count; i++)
        free(ignored_devnodes[i]);
    free(ignored_devnodes);
    ignored_devnodes = NULL;
    ignored_devnode_count = 0;
}

static int parse_int_arg(const char *value, int *out)
{
    char *end = NULL;
    errno = 0;
    long parsed = strtol(value, &end, 10);
    if (errno != 0 || !end || *end != '\0' || parsed < INT32_MIN || parsed > INT32_MAX)
        return -1;
    *out = (int)parsed;
    return 0;
}

static int parse_double_arg(const char *value, double *out)
{
    char *end = NULL;
    errno = 0;
    double parsed = strtod(value, &end);
    if (errno != 0 || !end || *end != '\0' || !isfinite(parsed))
        return -1;
    *out = parsed;
    return 0;
}

static int parse_bool_arg(const char *value, int *out)
{
    if (!value)
        return -1;

    if (strcmp(value, "1") == 0 || strcasecmp(value, "true") == 0 ||
        strcasecmp(value, "yes") == 0 || strcasecmp(value, "on") == 0) {
        *out = 1;
        return 0;
    }

    if (strcmp(value, "0") == 0 || strcasecmp(value, "false") == 0 ||
        strcasecmp(value, "no") == 0 || strcasecmp(value, "off") == 0) {
        *out = 0;
        return 0;
    }

    return -1;
}

static int read_rss_kb(void)
{
    FILE *fp = fopen("/proc/self/statm", "r");
    if (!fp)
        return -1;

    unsigned long total_pages = 0;
    unsigned long rss_pages = 0;
    if (fscanf(fp, "%lu %lu", &total_pages, &rss_pages) != 2) {
        fclose(fp);
        return -1;
    }
    fclose(fp);
    (void)total_pages;

    long page_size = sysconf(_SC_PAGESIZE);
    if (page_size <= 0)
        return -1;

    unsigned long long rss_bytes = (unsigned long long)rss_pages * (unsigned long long)page_size;
    return (int)(rss_bytes / 1024ULL);
}

static double read_cpu_seconds(void)
{
    struct rusage usage;
    if (getrusage(RUSAGE_SELF, &usage) != 0)
        return -1.0;

    double user = (double)usage.ru_utime.tv_sec + (double)usage.ru_utime.tv_usec / 1000000.0;
    double sys = (double)usage.ru_stime.tv_sec + (double)usage.ru_stime.tv_usec / 1000000.0;
    return user + sys;
}

static void maybe_show_resource_error_dialog(const char *message)
{
    const char *display = getenv("DISPLAY");
    if (!display || !*display)
        return;

    pid_t pid = fork();
    if (pid != 0)
        return;

    execlp("zenity", "zenity", "--error", "--title=edge-motion", "--width=520", "--text", message,
           (char *)NULL);
    _exit(0);
}

static int check_resource_limits(struct resource_guard_state *guard)
{
    if (!resource_guard_enabled)
        return 0;

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    int64_t now_ms = timespec_to_ms(&now);
    if (guard->initialized) {
        int64_t elapsed_ms = now_ms - timespec_to_ms(&guard->last_ts);
        if (elapsed_ms < RESOURCE_CHECK_INTERVAL_MS)
            return 0;
    }

    int rss_kb = read_rss_kb();
    double cpu_seconds = read_cpu_seconds();

    if (!guard->initialized) {
        guard->last_ts = now;
        guard->last_cpu_seconds = cpu_seconds;
        guard->initialized = 1;
        guard->consecutive_over_limit = 0;
        return 0;
    }

    double elapsed_s = (double)(now_ms - timespec_to_ms(&guard->last_ts)) / 1000.0;
    if (elapsed_s < 0.001)
        return 0;

    double cpu_percent = 0.0;
    if (cpu_seconds >= 0.0 && guard->last_cpu_seconds >= 0.0)
        cpu_percent = (cpu_seconds - guard->last_cpu_seconds) / elapsed_s * 100.0;

    guard->last_ts = now;
    guard->last_cpu_seconds = cpu_seconds;

    int rss_limit_kb = max_rss_mb > 0 ? max_rss_mb * 1024 : 0;
    int rss_over = rss_limit_kb > 0 && rss_kb > 0 && rss_kb > rss_limit_kb;
    int cpu_over = max_cpu_percent > 0.0 && cpu_percent > max_cpu_percent;

    if (rss_over || cpu_over) {
        guard->consecutive_over_limit++;
        if (guard->consecutive_over_limit >= resource_grace_checks) {
            char msg[512];
            snprintf(msg,
                     sizeof(msg),
                     "edge-motion остановлен: повышенное потребление ресурсов.\n"
                     "CPU: %.1f%% (лимит %.1f%%), RSS: %.1f MB (лимит %d MB).",
                     cpu_percent,
                     max_cpu_percent,
                     rss_kb > 0 ? (double)rss_kb / 1024.0 : -1.0,
                     max_rss_mb);
            fprintf(stderr, "%s\n", msg);
            maybe_show_resource_error_dialog(msg);
            return -1;
        }
    } else {
        guard->consecutive_over_limit = 0;
    }

    return 0;
}


static int set_forced_devnode(const char *value)
{
    if (!value || value[0] != '/')
        return -1;

    char *copy = strdup(value);
    if (!copy)
        return -1;

    free(forced_devnode);
    forced_devnode = copy;
    return 0;
}

static int apply_config_option(const char *key, const char *value)
{
    if (strcmp(key, "threshold") == 0)
        return parse_double_arg(value, &edge_threshold);
    if (strcmp(key, "threshold-left") == 0)
        return parse_double_arg(value, &threshold_left);
    if (strcmp(key, "threshold-right") == 0)
        return parse_double_arg(value, &threshold_right);
    if (strcmp(key, "threshold-top") == 0)
        return parse_double_arg(value, &threshold_top);
    if (strcmp(key, "threshold-bottom") == 0)
        return parse_double_arg(value, &threshold_bottom);
    if (strcmp(key, "hysteresis") == 0)
        return parse_double_arg(value, &edge_hysteresis);
    if (strcmp(key, "hold-ms") == 0)
        return parse_int_arg(value, &hold_ms);
    if (strcmp(key, "pulse-ms") == 0)
        return parse_int_arg(value, &pulse_ms);
    if (strcmp(key, "pulse-step") == 0)
        return parse_double_arg(value, &pulse_step);
    if (strcmp(key, "max-speed") == 0)
        return parse_double_arg(value, &max_speed);
    if (strcmp(key, "mode") == 0)
        return parse_mode(value, &mode);
    if (strcmp(key, "natural-scroll") == 0) {
        return parse_bool_arg(value, &natural_scroll);
    }
    if (strcmp(key, "diagonal-scroll") == 0) {
        return parse_bool_arg(value, &diagonal_scroll);
    }
    if (strcmp(key, "two-finger-scroll") == 0) {
        return parse_bool_arg(value, &two_finger_scroll);
    }
    if (strcmp(key, "deadzone") == 0)
        return parse_double_arg(value, &deadzone);
    if (strcmp(key, "grab") == 0) {
        return parse_bool_arg(value, &use_grab);
    }
    if (strcmp(key, "device") == 0)
        return set_forced_devnode(value);
    if (strcmp(key, "ignore") == 0)
        return add_ignored_devnode(value);
    if (strcmp(key, "daemon") == 0) {
        return parse_bool_arg(value, &daemon_mode);
    }
    if (strcmp(key, "resource-guard") == 0)
        return parse_bool_arg(value, &resource_guard_enabled);
    if (strcmp(key, "max-rss-mb") == 0)
        return parse_int_arg(value, &max_rss_mb);
    if (strcmp(key, "max-cpu-percent") == 0)
        return parse_double_arg(value, &max_cpu_percent);
    if (strcmp(key, "resource-grace-checks") == 0)
        return parse_int_arg(value, &resource_grace_checks);
    if (strcmp(key, "scroll-axis-priority") == 0)
        return parse_scroll_priority(value, &scroll_priority);
    if (strcmp(key, "accel-exponent") == 0)
        return parse_double_arg(value, &accel_exponent);
    if (strcmp(key, "pressure-boost") == 0)
        return parse_double_arg(value, &pressure_boost);
    if (strcmp(key, "button-zone") == 0)
        return parse_double_arg(value, &button_zone);
    if (strcmp(key, "button-cooldown-ms") == 0)
        return parse_int_arg(value, &button_cooldown_ms);

    return -1;
}

static int load_config_file(const char *path)
{
    FILE *fp = fopen(path, "r");
    if (!fp)
        return -1;

    char line[512];
    int line_no = 0;
    while (fgets(line, sizeof(line), fp)) {
        line_no++;
        char *p = line;
        while (*p && isspace((unsigned char)*p))
            p++;
        if (*p == '\0' || *p == '\n' || *p == '#')
            continue;

        char *eq = strchr(p, '=');
        if (!eq)
            continue;
        *eq = '\0';
        char *key = p;
        char *value = eq + 1;

        char *kend = key + strlen(key);
        while (kend > key && isspace((unsigned char)kend[-1]))
            *--kend = '\0';
        while (*value && isspace((unsigned char)*value))
            value++;

        char *vend = value + strlen(value);
        while (vend > value && isspace((unsigned char)vend[-1]))
            *--vend = '\0';

        if (apply_config_option(key, value) < 0) {
            fprintf(stderr, "Invalid config option at %s:%d -> %s\n", path, line_no, key);
            fflush(stderr);
            fclose(fp);
            return -1;
        }
    }

    fclose(fp);
    return 0;
}

static void handle_signal(int sig)
{
    (void)sig;
    running = 0;
}

static inline int emit_event(int ufd, int type, int code, int val)
{
    struct input_event ev = {0};
    ev.type = type;
    ev.code = code;
    ev.value = val;
    size_t written = 0;

    while (written < sizeof(ev)) {
        ssize_t ret = write(ufd, (const char *)&ev + written, sizeof(ev) - written);
        if (ret > 0) {
            written += (size_t)ret;
            continue;
        }

        if (ret < 0 && errno == EINTR)
            continue;

        if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            struct timespec ts = {.tv_sec = 0, .tv_nsec = 1000000};
            nanosleep(&ts, NULL);
            continue;
        }

        return -1;
    }

    return 0;
}

static inline int emit_rel(int ufd, int code, int val)
{
    return emit_event(ufd, EV_REL, code, val);
}

static inline int emit_syn(int ufd)
{
    return emit_event(ufd, EV_SYN, SYN_REPORT, 0);
}

static inline int64_t timespec_to_ms(const struct timespec *ts)
{
    return ts->tv_sec * 1000LL + ts->tv_nsec / 1000000LL;
}

static inline int64_t monotonic_now_ms(void)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return timespec_to_ms(&now);
}

static int reset_multitouch_state(struct libevdev *dev, int **slot_x, int **slot_y,
                                  unsigned char **slot_active, int *slot_count)
{
    const struct input_absinfo *slot_info = libevdev_get_abs_info(dev, ABS_MT_SLOT);
    int new_slot_count = 1;

    if (slot_info && slot_info->maximum >= slot_info->minimum)
        new_slot_count = slot_info->maximum - slot_info->minimum + 1;

    int *new_slot_x = calloc((size_t)new_slot_count, sizeof(int));
    int *new_slot_y = calloc((size_t)new_slot_count, sizeof(int));
    unsigned char *new_slot_active = calloc((size_t)new_slot_count, sizeof(unsigned char));
    if (!new_slot_x || !new_slot_y || !new_slot_active) {
        free(new_slot_x);
        free(new_slot_y);
        free(new_slot_active);
        return -1;
    }

    for (int i = 0; i < new_slot_count; i++) {
        new_slot_x[i] = -1;
        new_slot_y[i] = -1;
    }

    free(*slot_x);
    free(*slot_y);
    free(*slot_active);

    *slot_x = new_slot_x;
    *slot_y = new_slot_y;
    *slot_active = new_slot_active;
    *slot_count = new_slot_count;

    return 0;
}

static void cleanup_touchpad_resources(struct touchpad_resources *tp)
{
    if (tp->devnode) {
        free(tp->devnode);
        tp->devnode = NULL;
    }

    if (tp->input_fd >= 0) {
        close(tp->input_fd);
        tp->input_fd = -1;
    }

    if (tp->dev) {
        libevdev_free(tp->dev);
        tp->dev = NULL;
    }
}

static int create_uinput_device(void)
{
    int fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0)
        fd = open("/dev/input/uinput", O_WRONLY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0)
        return -1;

    if (ioctl(fd, UI_SET_EVBIT, EV_REL) < 0 ||
        ioctl(fd, UI_SET_RELBIT, REL_X) < 0 || ioctl(fd, UI_SET_RELBIT, REL_Y) < 0 ||
        ioctl(fd, UI_SET_RELBIT, REL_WHEEL) < 0 || ioctl(fd, UI_SET_RELBIT, REL_HWHEEL) < 0) {
        close(fd);
        return -1;
    }

    struct uinput_setup uset = {0};
    snprintf(uset.name, UINPUT_MAX_NAME_SIZE, "edge-motion-virtual-mouse");
    uset.id.bustype = BUS_VIRTUAL;
    uset.id.vendor = 0x1234;
    uset.id.product = 0x5678;
    uset.id.version = 1;

    if (ioctl(fd, UI_DEV_SETUP, &uset) < 0 || ioctl(fd, UI_DEV_CREATE) < 0) {
        ioctl(fd, UI_DEV_DESTROY);
        close(fd);
        return -1;
    }

    // Kernel needs a short settle delay so the created uinput device becomes visible.
    usleep(UINPUT_SETTLE_MS * 1000);
    return fd;
}

static void free_touchpad_candidates(struct touchpad_candidate *items, size_t count)
{
    for (size_t i = 0; i < count; i++) {
        free(items[i].devnode);
        free(items[i].name);
    }
    free(items);
}

static int enumerate_touchpad_candidates(struct touchpad_candidate **out_items, size_t *out_count)
{
    struct udev *udev = udev_new();
    if (!udev)
        return -1;

    struct udev_enumerate *en = udev_enumerate_new(udev);
    if (!en) {
        udev_unref(udev);
        return -1;
    }

    udev_enumerate_add_match_subsystem(en, "input");
    udev_enumerate_add_match_property(en, "ID_INPUT_TOUCHPAD", "1");
    udev_enumerate_scan_devices(en);

    struct touchpad_candidate *items = NULL;
    size_t count = 0;
    struct udev_list_entry *entry;
    udev_list_entry_foreach(entry, udev_enumerate_get_list_entry(en)) {
        struct udev_device *dev =
            udev_device_new_from_syspath(udev, udev_list_entry_get_name(entry));
        if (!dev)
            continue;

        const char *devnode = udev_device_get_devnode(dev);
        if (devnode && strstr(devnode, "/event") && !is_ignored_devnode(devnode)) {
            int fd = open(devnode, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
            if (fd >= 0) {
                struct libevdev *evdev = NULL;
                if (libevdev_new_from_fd(fd, &evdev) == 0) {
                    const struct input_absinfo *absx =
                        libevdev_get_abs_info(evdev, ABS_MT_POSITION_X);
                    if (!absx)
                        absx = libevdev_get_abs_info(evdev, ABS_X);
                    const struct input_absinfo *absy =
                        libevdev_get_abs_info(evdev, ABS_MT_POSITION_Y);
                    if (!absy)
                        absy = libevdev_get_abs_info(evdev, ABS_Y);

                    if (absx && absy && absx->maximum >= absx->minimum &&
                        absy->maximum >= absy->minimum) {
                        int has_finger_tool = libevdev_has_event_code(evdev, EV_KEY, BTN_TOOL_FINGER);
                        int has_btn_touch = libevdev_has_event_code(evdev, EV_KEY, BTN_TOUCH);
                        if (!has_finger_tool && !has_btn_touch) {
                            libevdev_free(evdev);
                            close(fd);
                            udev_device_unref(dev);
                            continue;
                        }

                        const char *device_name = libevdev_get_name(evdev) ? libevdev_get_name(evdev)
                                                                          : "unknown";
                        char *devnode_copy = strdup(devnode);
                        char *name_copy = strdup(device_name);
                        if (devnode_copy && name_copy) {
                            struct touchpad_candidate *tmp =
                                realloc(items, (count + 1) * sizeof(*items));
                            if (tmp) {
                                items = tmp;
                                items[count].devnode = devnode_copy;
                                items[count].name = name_copy;
                                items[count].integrated =
                                    udev_device_get_property_value(dev,
                                                                   "ID_INPUT_TOUCHPAD_INTEGRATED") &&
                                            strcmp(udev_device_get_property_value(
                                                       dev, "ID_INPUT_TOUCHPAD_INTEGRATED"),
                                                   "1") == 0
                                        ? 1
                                        : 0;
                                items[count].min_x = absx->minimum;
                                items[count].max_x = absx->maximum;
                                items[count].min_y = absy->minimum;
                                items[count].max_y = absy->maximum;
                                long long range_x =
                                    (long long)absx->maximum - (long long)absx->minimum;
                                long long range_y =
                                    (long long)absy->maximum - (long long)absy->minimum;
                                items[count].area = range_x * range_y;
                                items[count].has_finger_tool = has_finger_tool ? 1 : 0;
                                items[count].has_btn_touch = has_btn_touch ? 1 : 0;
                                items[count].is_mouse_like =
                                    libevdev_has_event_code(evdev, EV_REL, REL_X) &&
                                            libevdev_has_event_code(evdev, EV_REL, REL_Y)
                                        ? 1
                                        : 0;
                                count++;
                            } else {
                                free(devnode_copy);
                                free(name_copy);
                                free_touchpad_candidates(items, count);
                                libevdev_free(evdev);
                                close(fd);
                                udev_device_unref(dev);
                                udev_enumerate_unref(en);
                                udev_unref(udev);
                                return -1;
                            }
                        } else {
                            free(devnode_copy);
                            free(name_copy);
                        }
                    }
                    libevdev_free(evdev);
                }
                close(fd);
            }
        }

        udev_device_unref(dev);
    }

    udev_enumerate_unref(en);
    udev_unref(udev);

    *out_items = items;
    *out_count = count;
    return count > 0 ? 0 : -1;
}

static int print_touchpad_devices(void)
{
    struct touchpad_candidate *items = NULL;
    size_t count = 0;
    if (enumerate_touchpad_candidates(&items, &count) < 0) {
        fprintf(stderr, "No suitable touchpad devices found.\n");
        return -1;
    }

    for (size_t i = 0; i < count; i++) {
        printf("%s\t%s\tintegrated=%s\tarea=%lld\trange=[%d..%d]x[%d..%d]\n",
               items[i].devnode,
               items[i].name,
               items[i].integrated ? "yes" : "no",
               items[i].area,
               items[i].min_x,
               items[i].max_x,
               items[i].min_y,
               items[i].max_y);
    }

    free_touchpad_candidates(items, count);
    return 0;
}

static char *find_touchpad_devnode(void)
{
    struct touchpad_candidate *items = NULL;
    size_t count = 0;
    if (enumerate_touchpad_candidates(&items, &count) < 0)
        return NULL;

    size_t best = 0;
    for (size_t i = 1; i < count; i++) {
        int better_integrated = items[i].integrated > items[best].integrated;
        int same_integrated = items[i].integrated == items[best].integrated;
        int better_tool = items[i].has_finger_tool > items[best].has_finger_tool;
        int same_tool = items[i].has_finger_tool == items[best].has_finger_tool;
        int less_mouse_like = items[i].is_mouse_like < items[best].is_mouse_like;
        int same_mouse_like = items[i].is_mouse_like == items[best].is_mouse_like;
        int better_area = items[i].area > items[best].area;
        if (better_integrated ||
            (same_integrated &&
             (better_tool || (same_tool && (less_mouse_like || (same_mouse_like && better_area))))))
            best = i;
    }

    char *result = items[best].devnode ? strdup(items[best].devnode) : NULL;
    free_touchpad_candidates(items, count);
    return result;
}

static int reopen_touchpad(struct touchpad_resources *tp,
                           int *min_x, int *max_x, int *min_y, int *max_y)
{
    cleanup_touchpad_resources(tp);

    if (forced_devnode) {
        if (is_ignored_devnode(forced_devnode))
            return -1;
        tp->devnode = strdup(forced_devnode);
        if (!tp->devnode)
            return -1;
    } else {
        tp->devnode = find_touchpad_devnode();
    }
    if (!tp->devnode)
        return -1;

    tp->input_fd = open(tp->devnode, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    if (tp->input_fd < 0) {
        cleanup_touchpad_resources(tp);
        return -1;
    }

    if (libevdev_new_from_fd(tp->input_fd, &tp->dev) < 0) {
        cleanup_touchpad_resources(tp);
        return -1;
    }

    if (use_grab) {
        int grc = 0;
        int attempts = 3;
        useconds_t delay_us = 10000;
        while (attempts-- > 0) {
            grc = libevdev_grab(tp->dev, LIBEVDEV_GRAB);
            if (grc == 0)
                break;
            if (attempts > 0) {
                usleep(delay_us);
                delay_us *= 2;
            }
        }
        if (grc < 0 && verbose)
            fprintf(stderr, "Failed to grab touchpad: %s\n", strerror(-grc));
    }

    const struct input_absinfo *absx = libevdev_get_abs_info(tp->dev, ABS_MT_POSITION_X);
    if (!absx)
        absx = libevdev_get_abs_info(tp->dev, ABS_X);
    const struct input_absinfo *absy = libevdev_get_abs_info(tp->dev, ABS_MT_POSITION_Y);
    if (!absy)
        absy = libevdev_get_abs_info(tp->dev, ABS_Y);

    if (!absx || !absy) {
        cleanup_touchpad_resources(tp);
        return -1;
    }

    *min_x = absx->minimum;
    *max_x = absx->maximum;
    *min_y = absy->minimum;
    *max_y = absy->maximum;

    return 0;
}

static void read_pressure_range(struct libevdev *dev, int *min_pressure, int *max_pressure)
{
    const struct input_absinfo *pressure = libevdev_get_abs_info(dev, ABS_MT_PRESSURE);
    if (!pressure)
        pressure = libevdev_get_abs_info(dev, ABS_PRESSURE);

    if (pressure && pressure->maximum > pressure->minimum) {
        *min_pressure = pressure->minimum;
        *max_pressure = pressure->maximum;
    } else {
        *min_pressure = 0;
        *max_pressure = 0;
    }
}

static void deactivate_edge_motion(void)
{
    pthread_mutex_lock(&state.lock);
    state.edge_active = 0;
    state.dir_x = 0;
    state.dir_y = 0;
    state.speed_factor = 0.0;
    pthread_cond_broadcast(&state.cond);
    pthread_mutex_unlock(&state.lock);
}

static void get_timeout_timespec(struct timespec *ts, int ms)
{
    if (ms < 0)
        ms = 0;

    clock_gettime(CLOCK_MONOTONIC, ts);
    int64_t sec_add = ms / 1000;
    int64_t msec_add = ms % 1000;
    int64_t nsec = (int64_t)ts->tv_nsec + msec_add * 1000000LL;
    ts->tv_sec += (time_t)sec_add + (time_t)(nsec / 1000000000LL);
    ts->tv_nsec = (long)(nsec % 1000000000LL);
}

static void *pulser_thread(void *arg)
{
    int ufd = (int)(intptr_t)arg;

    pthread_mutex_lock(&state.lock);
    while (running) {
        while (!state.edge_active && running)
            pthread_cond_wait(&state.cond, &state.lock);

        if (!running)
            break;

        bool edge_active = state.edge_active;
        int dx = state.dir_x;
        int dy = state.dir_y;
        double speed_factor = state.speed_factor;
        pthread_mutex_unlock(&state.lock);

        int err = 0;
        if (edge_active && (dx || dy)) {
            if (ufd < 0)
                ufd = create_uinput_device();
            if (ufd < 0) {
                err = -1;
                goto relock;
            }

            double len = hypot((double)dx, (double)dy);
            if (len < 1e-9)
                goto relock;
            int current_step =
                (int)lround(pulse_step * (1.0 + speed_factor * (max_speed - 1.0)));
            if (current_step < 1)
                current_step = 1;
            if (current_step > 100)
                current_step = 100;
            int step_x = (int)lround((double)dx / len * (double)current_step);
            int step_y = (int)lround((double)dy / len * (double)current_step);

            if (mode == EM_MODE_MOTION) {
                if (step_x)
                    err |= emit_rel(ufd, REL_X, step_x);
                if (step_y)
                    err |= emit_rel(ufd, REL_Y, step_y);
            } else {
                if (!diagonal_scroll) {
                    if (scroll_priority == SCROLL_PRIORITY_HORIZONTAL) {
                        step_y = 0;
                    } else if (scroll_priority == SCROLL_PRIORITY_VERTICAL) {
                        step_x = 0;
                    } else if (abs(step_x) >= abs(step_y)) {
                        step_y = 0;
                    } else {
                        step_x = 0;
                    }
                }

                if (step_x)
                    err |= emit_rel(ufd, REL_HWHEEL, step_x);
                if (step_y)
                    err |= emit_rel(ufd, REL_WHEEL, natural_scroll ? step_y : -step_y);
            }
            err |= emit_syn(ufd);
        }

relock:
        pthread_mutex_lock(&state.lock);
        if (!running)
            break;

        if (err < 0) {
            if (verbose)
                fprintf(stderr, "uinput write failed, disabling edge motion until recovery.\n");
            if (ufd >= 0) {
                ioctl(ufd, UI_DEV_DESTROY);
                close(ufd);
                ufd = -1;
            }
            state.edge_active = 0;
            state.dir_x = 0;
            state.dir_y = 0;
            state.speed_factor = 0.0;
            pthread_cond_broadcast(&state.cond);
        }

        if (running && state.edge_active) {
            struct timespec ts;
            get_timeout_timespec(&ts, pulse_ms);
            pthread_cond_timedwait(&state.cond, &state.lock, &ts);
        }
    }
    pthread_mutex_unlock(&state.lock);

    if (ufd >= 0) {
        ioctl(ufd, UI_DEV_DESTROY);
        close(ufd);
    }

    return NULL;
}

static void print_usage(const char *prog)
{
    printf("edge-motion - edge-triggered touchpad helper\n\n");
    printf("Usage: %s [OPTIONS]\n", prog);
    printf("Loads ~/.config/edge-motion.conf automatically if present.\n");
    printf("  --threshold <0.01-0.5>   Edge threshold for all sides (default %.2f)\n", DEFAULT_EDGE_THRESHOLD);
    printf("  --threshold-left <0.01-0.5>   Left edge threshold override\n");
    printf("  --threshold-right <0.01-0.5>  Right edge threshold override\n");
    printf("  --threshold-top <0.01-0.5>    Top edge threshold override\n");
    printf("  --threshold-bottom <0.01-0.5> Bottom edge threshold override\n");
    printf("  --hysteresis <0.0-0.2>   Edge hysteresis (default %.3f)\n", DEFAULT_EDGE_HYSTERESIS);
    printf("  --hold-ms <ms>           Hold delay before activation (default %d)\n", DEFAULT_HOLD_MS);
    printf("  --pulse-ms <ms>          Pulse interval (default %d)\n", DEFAULT_PULSE_MS);
    printf("  --pulse-step <n>         Base movement step (default %.1f)\n", DEFAULT_PULSE_STEP);
    printf("  --max-speed <n>          Max speed multiplier (default %.1f)\n", DEFAULT_MAX_SPEED);
    printf("  --mode <motion|scroll>   Cursor motion or wheel scrolling\n");
    printf("  --natural-scroll         Natural scroll direction\n");
    printf("  --reverse-scroll         Alias for --natural-scroll\n");
    printf("  --diagonal-scroll        Allow diagonal scrolling\n");
    printf("  --two-finger-scroll      Require two fingers in scroll mode\n");
    printf("  --deadzone <0.0-0.49>    Central non-activation zone\n");
    printf("  --scroll-axis-priority <dominant|horizontal|vertical>\n");
    printf("                           Scroll axis preference without diagonal mode\n");
    printf("  --accel-exponent <n>     Non-linear edge depth acceleration (default 1.0)\n");
    printf("  --pressure-boost <0-2>   Extra speed from touch pressure (default 0)\n");
    printf("  --button-zone <0-0.4>    Disable edge motion near bottom button area (default %.2f)\n",
           DEFAULT_BUTTON_ZONE);
    printf("  --button-cooldown-ms <ms> Suppress edge motion shortly after click (default %d)\n",
           DEFAULT_BUTTON_COOLDOWN_MS);
    printf("  --grab / --no-grab       Exclusive grab (can disable normal touchpad input) / shared mode\n");
    printf("  --device </dev/input/eventX>  Force touchpad device\n");
    printf("  --ignore </dev/input/eventX>  Ignore device (can be repeated)\n");
    printf("  --config <path>          Load config file with key=value lines\n");
    printf("  --daemon                 Run in daemon mode\n");
    printf("  --resource-guard / --no-resource-guard  Enable/disable self-protection\n");
    printf("  --max-rss-mb <n>         RSS memory limit in MB (default %d)\n", DEFAULT_MAX_RSS_MB);
    printf("  --max-cpu-percent <n>    CPU usage limit in %% (default %.1f)\n", DEFAULT_MAX_CPU_PERCENT);
    printf("  --resource-grace-checks <n> Consecutive checks above limits before stop (default %d)\n",
           DEFAULT_RESOURCE_GRACE_CHECKS);
    printf("  --list-devices           Show available touchpads and exit\n");
    printf("  --version                Show version and exit\n");
    printf("  --verbose                Verbose logging\n");
    printf("  --help                   Show this help\n");
}

enum {
    OPT_THRESHOLD_LEFT = 1000,
    OPT_THRESHOLD_RIGHT,
    OPT_THRESHOLD_TOP,
    OPT_THRESHOLD_BOTTOM,
    OPT_SCROLL_AXIS_PRIORITY,
    OPT_ACCEL_EXPONENT,
    OPT_PRESSURE_BOOST,
    OPT_BUTTON_ZONE,
    OPT_BUTTON_COOLDOWN_MS,
    OPT_IGNORE,
    OPT_DAEMON,
    OPT_CONFIG,
    OPT_RESOURCE_GUARD,
    OPT_NO_RESOURCE_GUARD,
    OPT_MAX_RSS_MB,
    OPT_MAX_CPU_PERCENT,
    OPT_RESOURCE_GRACE_CHECKS,
};

int main(int argc, char **argv)
{
    static struct option long_opts[] = {
        {"help", no_argument, NULL, 'h'},
        {"threshold", required_argument, NULL, 't'},
        {"threshold-left", required_argument, NULL, OPT_THRESHOLD_LEFT},
        {"threshold-right", required_argument, NULL, OPT_THRESHOLD_RIGHT},
        {"threshold-top", required_argument, NULL, OPT_THRESHOLD_TOP},
        {"threshold-bottom", required_argument, NULL, OPT_THRESHOLD_BOTTOM},
        {"hysteresis", required_argument, NULL, 'y'},
        {"hold-ms", required_argument, NULL, 'H'},
        {"pulse-ms", required_argument, NULL, 'p'},
        {"pulse-step", required_argument, NULL, 's'},
        {"max-speed", required_argument, NULL, 'm'},
        {"mode", required_argument, NULL, 'M'},
        {"natural-scroll", no_argument, NULL, 'n'},
        {"reverse-scroll", no_argument, NULL, 'r'},
        {"diagonal-scroll", no_argument, NULL, 'D'},
        {"two-finger-scroll", no_argument, NULL, '2'},
        {"deadzone", required_argument, NULL, 'z'},
        {"scroll-axis-priority", required_argument, NULL, OPT_SCROLL_AXIS_PRIORITY},
        {"accel-exponent", required_argument, NULL, OPT_ACCEL_EXPONENT},
        {"pressure-boost", required_argument, NULL, OPT_PRESSURE_BOOST},
        {"button-zone", required_argument, NULL, OPT_BUTTON_ZONE},
        {"button-cooldown-ms", required_argument, NULL, OPT_BUTTON_COOLDOWN_MS},
        {"grab", no_argument, NULL, 'g'},
        {"no-grab", no_argument, NULL, 'G'},
        {"device", required_argument, NULL, 'd'},
        {"ignore", required_argument, NULL, OPT_IGNORE},
        {"daemon", no_argument, NULL, OPT_DAEMON},
        {"config", required_argument, NULL, OPT_CONFIG},
        {"resource-guard", no_argument, NULL, OPT_RESOURCE_GUARD},
        {"no-resource-guard", no_argument, NULL, OPT_NO_RESOURCE_GUARD},
        {"max-rss-mb", required_argument, NULL, OPT_MAX_RSS_MB},
        {"max-cpu-percent", required_argument, NULL, OPT_MAX_CPU_PERCENT},
        {"resource-grace-checks", required_argument, NULL, OPT_RESOURCE_GRACE_CHECKS},
        {"list-devices", no_argument, NULL, 'l'},
        {"version", no_argument, NULL, 'V'},
        {"verbose", no_argument, NULL, 'v'},
        {0, 0, 0, 0},
    };

    char default_config[512];
    const char *home = getenv("HOME");
    if (home) {
        snprintf(default_config, sizeof(default_config), "%s/.config/edge-motion.conf", home);
        (void)load_config_file(default_config);
    }

    int opt;
    while ((opt = getopt_long(argc, argv, "", long_opts, NULL)) != -1) {
        switch (opt) {
        case 'h':
            print_usage(argv[0]);
            return 0;
        case 't':
            if (parse_double_arg(optarg, &edge_threshold) < 0) {
                fprintf(stderr, "Invalid threshold: %s\n", optarg);
                return 2;
            }
            break;
        case OPT_THRESHOLD_LEFT:
            if (parse_double_arg(optarg, &threshold_left) < 0) {
                fprintf(stderr, "Invalid threshold-left: %s\n", optarg);
                return 2;
            }
            break;
        case OPT_THRESHOLD_RIGHT:
            if (parse_double_arg(optarg, &threshold_right) < 0) {
                fprintf(stderr, "Invalid threshold-right: %s\n", optarg);
                return 2;
            }
            break;
        case OPT_THRESHOLD_TOP:
            if (parse_double_arg(optarg, &threshold_top) < 0) {
                fprintf(stderr, "Invalid threshold-top: %s\n", optarg);
                return 2;
            }
            break;
        case OPT_THRESHOLD_BOTTOM:
            if (parse_double_arg(optarg, &threshold_bottom) < 0) {
                fprintf(stderr, "Invalid threshold-bottom: %s\n", optarg);
                return 2;
            }
            break;
        case 'y':
            if (parse_double_arg(optarg, &edge_hysteresis) < 0) {
                fprintf(stderr, "Invalid hysteresis: %s\n", optarg);
                return 2;
            }
            break;
        case 'H':
            if (parse_int_arg(optarg, &hold_ms) < 0) {
                fprintf(stderr, "Invalid hold-ms: %s\n", optarg);
                return 2;
            }
            break;
        case 'p':
            if (parse_int_arg(optarg, &pulse_ms) < 0) {
                fprintf(stderr, "Invalid pulse-ms: %s\n", optarg);
                return 2;
            }
            break;
        case 's':
            if (parse_double_arg(optarg, &pulse_step) < 0) {
                fprintf(stderr, "Invalid pulse-step: %s\n", optarg);
                return 2;
            }
            break;
        case 'm':
            if (parse_double_arg(optarg, &max_speed) < 0) {
                fprintf(stderr, "Invalid max-speed: %s\n", optarg);
                return 2;
            }
            break;
        case 'M':
            if (parse_mode(optarg, &mode) < 0) {
                fprintf(stderr, "Invalid mode: %s\n", optarg);
                return 2;
            }
            break;
        case 'n':
        case 'r':
            natural_scroll = 1;
            break;
        case 'D':
            diagonal_scroll = 1;
            break;
        case '2':
            two_finger_scroll = 1;
            break;
        case 'z':
            if (parse_double_arg(optarg, &deadzone) < 0) {
                fprintf(stderr, "Invalid deadzone: %s\n", optarg);
                return 2;
            }
            break;
        case OPT_SCROLL_AXIS_PRIORITY:
            if (parse_scroll_priority(optarg, &scroll_priority) < 0) {
                fprintf(stderr, "Invalid scroll-axis-priority: %s\n", optarg);
                return 2;
            }
            break;
        case OPT_ACCEL_EXPONENT:
            if (parse_double_arg(optarg, &accel_exponent) < 0) {
                fprintf(stderr, "Invalid accel-exponent: %s\n", optarg);
                return 2;
            }
            break;
        case OPT_PRESSURE_BOOST:
            if (parse_double_arg(optarg, &pressure_boost) < 0) {
                fprintf(stderr, "Invalid pressure-boost: %s\n", optarg);
                return 2;
            }
            break;
        case OPT_BUTTON_ZONE:
            if (parse_double_arg(optarg, &button_zone) < 0) {
                fprintf(stderr, "Invalid button-zone: %s\n", optarg);
                return 2;
            }
            break;
        case OPT_BUTTON_COOLDOWN_MS:
            if (parse_int_arg(optarg, &button_cooldown_ms) < 0) {
                fprintf(stderr, "Invalid button-cooldown-ms: %s\n", optarg);
                return 2;
            }
            break;
        case 'g':
            use_grab = 1;
            break;
        case 'G':
            use_grab = 0;
            break;
        case 'd':
            if (set_forced_devnode(optarg) < 0) {
                fprintf(stderr, "Invalid device: %s\n", optarg);
                return 2;
            }
            break;
        case OPT_IGNORE:
            if (add_ignored_devnode(optarg) < 0) {
                fprintf(stderr, "Invalid ignore devnode: %s\n", optarg);
                return 2;
            }
            break;
        case OPT_DAEMON:
            daemon_mode = 1;
            break;
        case OPT_CONFIG:
            if (load_config_file(optarg) < 0)
                return 2;
            break;
        case OPT_RESOURCE_GUARD:
            resource_guard_enabled = 1;
            break;
        case OPT_NO_RESOURCE_GUARD:
            resource_guard_enabled = 0;
            break;
        case OPT_MAX_RSS_MB:
            if (parse_int_arg(optarg, &max_rss_mb) < 0 || max_rss_mb < 0) {
                fprintf(stderr, "Invalid max-rss-mb: %s\n", optarg);
                return 2;
            }
            break;
        case OPT_MAX_CPU_PERCENT:
            if (parse_double_arg(optarg, &max_cpu_percent) < 0 || max_cpu_percent < 0.0) {
                fprintf(stderr, "Invalid max-cpu-percent: %s\n", optarg);
                return 2;
            }
            break;
        case OPT_RESOURCE_GRACE_CHECKS:
            if (parse_int_arg(optarg, &resource_grace_checks) < 0 || resource_grace_checks < 1) {
                fprintf(stderr, "Invalid resource-grace-checks: %s\n", optarg);
                return 2;
            }
            break;
        case 'l':
            list_devices = 1;
            break;
        case 'V':
            printf("edge-motion %s\n", EDGE_MOTION_VERSION);
            return 0;
        case 'v':
            verbose = 1;
            break;
        default:
            print_usage(argv[0]);
            return 2;
        }
    }

    if (list_devices)
        return print_touchpad_devices() == 0 ? 0 : 1;

    if (threshold_left < 0.0)
        threshold_left = edge_threshold;
    if (threshold_right < 0.0)
        threshold_right = edge_threshold;
    if (threshold_top < 0.0)
        threshold_top = edge_threshold;
    if (threshold_bottom < 0.0)
        threshold_bottom = edge_threshold;

    if (edge_threshold < 0.01 || edge_threshold > 0.5 || edge_hysteresis < 0.0 || hold_ms < 0 ||
        pulse_ms <= 0 || pulse_step <= 0 || max_speed < 1.0 || deadzone < 0.0 || deadzone >= 0.5 ||
        threshold_left < 0.01 || threshold_left > 0.5 || threshold_right < 0.01 ||
        threshold_right > 0.5 || threshold_top < 0.01 || threshold_top > 0.5 ||
        threshold_bottom < 0.01 || threshold_bottom > 0.5 || accel_exponent < 0.0 ||
        pressure_boost < 0.0 || pressure_boost > 2.0 || button_zone < 0.0 || button_zone > 0.4 ||
        button_cooldown_ms < 0 || max_rss_mb < 0 || max_cpu_percent < 0.0 || resource_grace_checks < 1) {
        fprintf(stderr, "Invalid arguments. See --help.\n");
        return 2;
    }

    double max_threshold = fmax(fmax(threshold_left, threshold_right), fmax(threshold_top, threshold_bottom));
    if (edge_hysteresis >= max_threshold) {
        fprintf(stderr, "hysteresis must be lower than every active threshold\n");
        return 2;
    }
    if (deadzone + threshold_left > 0.5 || deadzone + threshold_right > 0.5 ||
        deadzone + threshold_top > 0.5 || deadzone + threshold_bottom > 0.5) {
        fprintf(stderr,
                "deadzone + threshold(side) must not exceed 0.5 for left/right/top/bottom\n");
        return 2;
    }

    struct sigaction sa = {.sa_handler = handle_signal, .sa_flags = 0};
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    if (daemon_mode && daemon(0, 0) < 0) {
        perror("daemon");
        return 1;
    }

    struct touchpad_resources tp = {
        .devnode = NULL,
        .input_fd = -1,
        .dev = NULL,
    };

    int min_x = 0, max_x = 0;
    int min_y = 0, max_y = 0;
    int *slot_x = NULL;
    int *slot_y = NULL;
    unsigned char *slot_active = NULL;

    if (reopen_touchpad(&tp, &min_x, &max_x, &min_y, &max_y) < 0) {
        fprintf(stderr, "Touchpad not found.\n");
        return 1;
    }

    int cond_initialized = 0;

    int ufd = create_uinput_device();
    if (ufd < 0) {
        fprintf(stderr, "Failed to create uinput (requires root/cap_sys_admin).\n");
        goto cleanup;
    }

    pthread_condattr_t cattr;
    if (pthread_condattr_init(&cattr) != 0 ||
        pthread_condattr_setclock(&cattr, CLOCK_MONOTONIC) != 0 ||
        pthread_cond_init(&state.cond, &cattr) != 0) {
        fprintf(stderr, "Failed to initialize condition variable.\n");
        pthread_condattr_destroy(&cattr);
        goto cleanup;
    }
    pthread_condattr_destroy(&cattr);
    cond_initialized = 1;

    pthread_t thr;
    int thread_started = 0;
    if (pthread_create(&thr, NULL, pulser_thread, (void *)(intptr_t)ufd) != 0) {
        fprintf(stderr, "Failed to create pulser thread.\n");
        goto cleanup;
    }
    thread_started = 1;

    int last_x = -1, last_y = -1;
    int current_slot = 0;
    int preferred_slot = -1;
    int was_in_edge = 0;
    int was_in_edge_x = 0;
    int was_in_edge_y = 0;
    int touchpad_available = 1;
    int64_t next_reopen_at_ms = INT64_MAX;
    struct resource_guard_state resource_guard = {0};
    int slot_count = 1;
    int active_fingers = 0;
    int pressure_min = 0, pressure_max = 0;
    int last_pressure = -1;
    int invalid_axes_logged = 0;
    int click_button_down = 0;
    int64_t edge_suppress_until_ms = 0;
    struct timespec edge_enter_time = {0};

    read_pressure_range(tp.dev, &pressure_min, &pressure_max);

    if (reset_multitouch_state(tp.dev, &slot_x, &slot_y, &slot_active, &slot_count) < 0) {
        fprintf(stderr, "Failed to allocate multitouch state memory.\n");
        goto cleanup;
    }

    struct pollfd pfd = {.fd = tp.input_fd, .events = POLLIN};
    int read_flags = LIBEVDEV_READ_FLAG_NORMAL;

    while (running) {
        if (check_resource_limits(&resource_guard) < 0) {
            running = 0;
            break;
        }

        int should_active = 0;
        int dx = 0, dy = 0;
        int64_t edge_diff_ms = 0;

        double speed_factor = 0.0;
        int two_finger_ok = !(mode == EM_MODE_SCROLL && two_finger_scroll) || active_fingers >= 2;
        int64_t now_ms_for_suppression = monotonic_now_ms();
        int in_button_cooldown = now_ms_for_suppression < edge_suppress_until_ms;
        if (max_x <= min_x || max_y <= min_y) {
            if (verbose && !invalid_axes_logged) {
                fprintf(stderr,
                        "Invalid touchpad axis range [%d..%d]x[%d..%d], waiting for recovery...\n",
                        min_x,
                        max_x,
                        min_y,
                        max_y);
                invalid_axes_logged = 1;
            }
            deactivate_edge_motion();
            last_x = -1;
            last_y = -1;
            (void)poll(NULL, 0, RESOURCE_CHECK_INTERVAL_MS);
            continue;
        }
        invalid_axes_logged = 0;

        if (last_x >= 0 && last_y >= 0 && two_finger_ok && !click_button_down && !in_button_cooldown) {
            double nx = (double)(last_x - min_x) / (double)(max_x - min_x);
            double ny = (double)(last_y - min_y) / (double)(max_y - min_y);
            int in_button_zone = ny >= (1.0 - button_zone);
            if (in_button_zone) {
                nx = 0.5;
                ny = 0.5;
            }
            if (nx > 0.5 - deadzone && nx < 0.5 + deadzone)
                nx = 0.5;
            if (ny > 0.5 - deadzone && ny < 0.5 + deadzone)
                ny = 0.5;
            double depth_x = 0.0;
            double depth_y = 0.0;

            double left_enter = threshold_left;
            double right_enter = threshold_right;
            double top_enter = threshold_top;
            double bottom_enter = threshold_bottom;
            double left_leave = left_enter - edge_hysteresis;
            double right_leave = right_enter - edge_hysteresis;
            double top_leave = top_enter - edge_hysteresis;
            double bottom_leave = bottom_enter - edge_hysteresis;

            if (was_in_edge_x) {
                if (nx >= 1.0 - right_leave)
                    dx = 1;
                else if (nx <= left_leave)
                    dx = -1;
            }

            if (!dx) {
                if (nx >= 1.0 - right_enter)
                    dx = 1;
                else if (nx <= left_enter)
                    dx = -1;
            }

            if (was_in_edge_y) {
                if (ny >= 1.0 - bottom_leave)
                    dy = 1;
                else if (ny <= top_leave)
                    dy = -1;
            }

            if (!dy) {
                if (ny >= 1.0 - bottom_enter)
                    dy = 1;
                else if (ny <= top_enter)
                    dy = -1;
            }

            if (nx >= 1.0 - right_enter)
                depth_x = (nx - (1.0 - right_enter)) / right_enter;
            else if (nx <= left_enter)
                depth_x = (left_enter - nx) / left_enter;

            if (ny >= 1.0 - bottom_enter)
                depth_y = (ny - (1.0 - bottom_enter)) / bottom_enter;
            else if (ny <= top_enter)
                depth_y = (top_enter - ny) / top_enter;

            if (depth_x > 1.0)
                depth_x = 1.0;
            if (depth_y > 1.0)
                depth_y = 1.0;

            speed_factor = fmax(depth_x, depth_y);
            if (accel_exponent != 1.0 && speed_factor > 0.0)
                speed_factor = pow(speed_factor, accel_exponent);
            if (pressure_boost > 0.0 && pressure_max > pressure_min && last_pressure >= pressure_min) {
                double p = (double)(last_pressure - pressure_min) / (double)(pressure_max - pressure_min);
                if (p < 0.0)
                    p = 0.0;
                if (p > 1.0)
                    p = 1.0;
                speed_factor *= 1.0 + p * pressure_boost;
                if (speed_factor > 1.0)
                    speed_factor = 1.0;
            }

            int currently_in_edge = (dx != 0 || dy != 0);
            if (currently_in_edge) {
                struct timespec now;
                clock_gettime(CLOCK_MONOTONIC, &now);
                if (!was_in_edge) {
                    edge_enter_time = now;
                    was_in_edge = 1;
                }
                edge_diff_ms = timespec_to_ms(&now) - timespec_to_ms(&edge_enter_time);
                should_active = edge_diff_ms >= hold_ms;
            } else {
                was_in_edge = 0;
            }

            was_in_edge_x = (dx != 0);
            was_in_edge_y = (dy != 0);
        } else {
            was_in_edge = 0;
            was_in_edge_x = 0;
            was_in_edge_y = 0;
        }

        pthread_mutex_lock(&state.lock);
        int changed = (state.edge_active != should_active || state.dir_x != dx || state.dir_y != dy ||
                       fabs(state.speed_factor - speed_factor) > 0.0001);
        state.edge_active = should_active;
        state.dir_x = dx;
        state.dir_y = dy;
        state.speed_factor = speed_factor;
        if (changed)
            pthread_cond_signal(&state.cond);
        pthread_mutex_unlock(&state.lock);

        int timeout_ms = -1;
        if (!should_active && (dx || dy)) {
            int remaining = hold_ms - (int)edge_diff_ms;
            timeout_ms = remaining > 0 ? remaining : 0;
        }

        if (!touchpad_available) {
            int64_t now_ms = monotonic_now_ms();
            int64_t remaining = next_reopen_at_ms - now_ms;
            timeout_ms = remaining > 0 ? (int)remaining : 0;
        }

        int ret = touchpad_available ? poll(&pfd, 1, timeout_ms) : poll(NULL, 0, timeout_ms);
        if (ret < 0) {
            if (errno == EINTR)
                continue;
            break;
        }

        if (ret > 0) {
            int rc = -EAGAIN;
            int sync_received = 0;

            if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
                rc = -ENODEV;
            } else if (pfd.revents & POLLIN) {
                struct input_event ev;

                while ((rc = libevdev_next_event(tp.dev, read_flags, &ev)) >= 0) {
                    if (rc == LIBEVDEV_READ_STATUS_SYNC)
                        read_flags = LIBEVDEV_READ_FLAG_SYNC;

                    if (ev.type == EV_SYN && ev.code == SYN_REPORT) {
                        sync_received = 1;
                        continue;
                    }

                    if (ev.type == EV_ABS) {
                        if (ev.code == ABS_MT_SLOT)
                            current_slot = ev.value;

                        int slot_valid = current_slot >= 0 && current_slot < slot_count;

                        if ((ev.code == ABS_MT_POSITION_X || ev.code == ABS_X) && slot_valid) {
                            slot_x[current_slot] = ev.value;
                            if (preferred_slot < 0 || preferred_slot == current_slot)
                                preferred_slot = current_slot;
                            if (preferred_slot == current_slot && slot_y[current_slot] >= 0)
                                last_x = ev.value;
                        }
                        if ((ev.code == ABS_MT_POSITION_Y || ev.code == ABS_Y) && slot_valid) {
                            slot_y[current_slot] = ev.value;
                            if (preferred_slot < 0 || preferred_slot == current_slot)
                                preferred_slot = current_slot;
                            if (preferred_slot == current_slot && slot_x[current_slot] >= 0)
                                last_y = ev.value;
                        }

                        if (ev.code == ABS_MT_PRESSURE || ev.code == ABS_PRESSURE)
                            last_pressure = ev.value;

                        if (ev.code == ABS_MT_TRACKING_ID && slot_valid) {
                            if (ev.value == -1) {
                                if (slot_active[current_slot] && active_fingers > 0)
                                    active_fingers--;
                                slot_active[current_slot] = 0;
                                slot_x[current_slot] = -1;
                                slot_y[current_slot] = -1;
                                if (preferred_slot == current_slot)
                                    preferred_slot = -1;
                            } else {
                                if (!slot_active[current_slot])
                                    active_fingers++;
                                slot_active[current_slot] = 1;
                                preferred_slot = current_slot;
                            }
                        }
                    } else if (ev.type == EV_KEY &&
                               (ev.code == BTN_LEFT || ev.code == BTN_RIGHT || ev.code == BTN_MIDDLE)) {
                        if (ev.value > 0) {
                            click_button_down = 1;
                        } else {
                            click_button_down = 0;
                        }
                        edge_suppress_until_ms = monotonic_now_ms() + button_cooldown_ms;
                    } else if (ev.type == EV_KEY &&
                               ((ev.code == BTN_TOUCH || ev.code == BTN_TOOL_FINGER ||
                                 ev.code == BTN_TOOL_PEN || ev.code == BTN_TOOL_MOUSE) &&
                                ev.value == 0)) {
                        last_x = -1;
                        last_y = -1;
                        last_pressure = -1;
                        was_in_edge = 0;
                        was_in_edge_x = 0;
                        was_in_edge_y = 0;
                        preferred_slot = -1;
                        active_fingers = 0;
                        for (int i = 0; i < slot_count; i++) {
                            slot_active[i] = 0;
                            slot_x[i] = -1;
                            slot_y[i] = -1;
                        }
                    }
                }

                if (rc == -EAGAIN && read_flags == LIBEVDEV_READ_FLAG_SYNC)
                    read_flags = LIBEVDEV_READ_FLAG_NORMAL;
            }

            if (sync_received) {
                int active_slot = -1;
                if (preferred_slot >= 0 && preferred_slot < slot_count && slot_active[preferred_slot] &&
                    slot_x[preferred_slot] >= 0 && slot_y[preferred_slot] >= 0)
                    active_slot = preferred_slot;
                else {
                    for (int i = 0; i < slot_count; i++) {
                        if (slot_active[i] && slot_x[i] >= 0 && slot_y[i] >= 0) {
                            active_slot = i;
                            break;
                        }
                    }
                }

                if (active_slot >= 0) {
                    last_x = slot_x[active_slot];
                    last_y = slot_y[active_slot];
                } else {
                    last_x = -1;
                    last_y = -1;
                }
            }
            if (rc < 0 && rc != -EAGAIN) {
                if (verbose)
                    fprintf(stderr, "Touchpad disconnected, reconnecting...\n");

                deactivate_edge_motion();

                last_x = -1;
                last_y = -1;
                was_in_edge = 0;
                was_in_edge_x = 0;
                was_in_edge_y = 0;
                touchpad_available = 0;
                active_fingers = 0;
                click_button_down = 0;
                cleanup_touchpad_resources(&tp);
                pfd.fd = -1;
                struct timespec now;
                clock_gettime(CLOCK_MONOTONIC, &now);
                next_reopen_at_ms = timespec_to_ms(&now) + TOUCHPAD_DISCONNECT_TIMEOUT_MS;
            }
        }

        if (!touchpad_available && running) {
            int64_t now_ms = monotonic_now_ms();
            if (now_ms >= next_reopen_at_ms) {
                if (reopen_touchpad(&tp, &min_x, &max_x, &min_y, &max_y) == 0) {
                    read_pressure_range(tp.dev, &pressure_min, &pressure_max);

                    if (reset_multitouch_state(tp.dev, &slot_x, &slot_y, &slot_active, &slot_count) < 0) {
                        fprintf(stderr, "Failed to refresh multitouch state after reconnect.\n");
                        running = 0;
                        break;
                    }

                    if (verbose)
                        fprintf(stderr, "Touchpad reconnected: %s\n", tp.devnode);

                    touchpad_available = 1;
                    pfd.fd = tp.input_fd;
                    pfd.events = POLLIN;
                    pfd.revents = 0;
                    read_flags = LIBEVDEV_READ_FLAG_NORMAL;
                    current_slot = 0;
                    preferred_slot = -1;
                    active_fingers = 0;
                    click_button_down = 0;
                    last_pressure = -1;
                    last_x = -1;
                    last_y = -1;
                    was_in_edge = 0;
                    was_in_edge_x = 0;
                    was_in_edge_y = 0;
                    edge_enter_time = (struct timespec){0};
                }
                next_reopen_at_ms = now_ms + TOUCHPAD_REOPEN_POLL_MS;
            }
        }
    }

cleanup:
    running = 0;

    if (cond_initialized) {
        pthread_mutex_lock(&state.lock);
        pthread_cond_broadcast(&state.cond);
        pthread_mutex_unlock(&state.lock);
    }

    if (thread_started)
        pthread_join(thr, NULL);

    if (!thread_started && ufd >= 0) {
        ioctl(ufd, UI_DEV_DESTROY);
        close(ufd);
    }

    cleanup_touchpad_resources(&tp);
    free(slot_x);
    free(slot_y);
    free(slot_active);
    free(forced_devnode);
    forced_devnode = NULL;
    free_ignored_devnodes();

    pthread_mutex_destroy(&state.lock);
    if (cond_initialized)
        pthread_cond_destroy(&state.cond);

    return 0;
}
