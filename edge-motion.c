#define _GNU_SOURCE
#include <errno.h>
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
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <libudev.h>
#include <unistd.h>

#define EDGE_MOTION_VERSION "1.1.0"

#define DEFAULT_EDGE_THRESHOLD 0.06
#define DEFAULT_EDGE_HYSTERESIS 0.015
#define DEFAULT_HOLD_MS 80
#define DEFAULT_PULSE_MS 10
#define DEFAULT_PULSE_STEP 3
#define DEFAULT_MAX_SPEED 3.0
#define TOUCHPAD_DISCONNECT_TIMEOUT_MS 200
#define TOUCHPAD_REOPEN_POLL_MS 250
#define UINPUT_SETTLE_MS 50

static double edge_threshold = DEFAULT_EDGE_THRESHOLD;
static double edge_hysteresis = DEFAULT_EDGE_HYSTERESIS;
static int hold_ms = DEFAULT_HOLD_MS;
static int pulse_ms = DEFAULT_PULSE_MS;
static int pulse_step = DEFAULT_PULSE_STEP;
static double max_speed = DEFAULT_MAX_SPEED;
static int verbose = 0;
static int list_devices = 0;
static int use_grab = 1;
static const char *forced_devnode = NULL;
static int diagonal_scroll = 0;
static int natural_scroll = 0;
static int two_finger_scroll = 0;
static double deadzone = 0.0;

enum em_mode {
    EM_MODE_MOTION = 0,
    EM_MODE_SCROLL = 1,
};

static enum em_mode mode = EM_MODE_MOTION;

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
};

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
        if (devnode && strstr(devnode, "/event")) {
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
                                count++;
                            } else {
                                free(devnode_copy);
                                free(name_copy);
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
        int better_area = items[i].area > items[best].area;
        if (better_integrated || (same_integrated && better_area))
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
        while (attempts-- > 0) {
            grc = libevdev_grab(tp->dev, LIBEVDEV_GRAB);
            if (grc == 0)
                break;
            if (attempts > 0)
                usleep(10000);
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
            double len = hypot((double)dx, (double)dy);
            if (len < 1e-9)
                goto relock;
            int current_step =
                (int)lround((double)pulse_step * (1.0 + speed_factor * (max_speed - 1.0)));
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
                if (!diagonal_scroll && abs(step_x) >= abs(step_y))
                    step_y = 0;
                else if (!diagonal_scroll)
                    step_x = 0;

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

    return NULL;
}

static void print_usage(const char *prog)
{
    printf("edge-motion - edge-triggered touchpad helper\n\n");
    printf("Usage: %s [OPTIONS]\n", prog);
    printf("  --threshold <0.01-0.5>   Edge threshold (default %.2f)\n", DEFAULT_EDGE_THRESHOLD);
    printf("  --hysteresis <0.0-0.2>   Edge hysteresis (default %.3f)\n", DEFAULT_EDGE_HYSTERESIS);
    printf("  --hold-ms <ms>           Hold delay before activation (default %d)\n", DEFAULT_HOLD_MS);
    printf("  --pulse-ms <ms>          Pulse interval (default %d)\n", DEFAULT_PULSE_MS);
    printf("  --pulse-step <n>         Base movement step (default %d)\n", DEFAULT_PULSE_STEP);
    printf("  --max-speed <n>          Max speed multiplier (default %.1f)\n", DEFAULT_MAX_SPEED);
    printf("  --mode <motion|scroll>   Cursor motion or wheel scrolling\n");
    printf("  --natural-scroll         Natural scroll direction\n");
    printf("  --reverse-scroll         Alias for --natural-scroll\n");
    printf("  --diagonal-scroll        Allow diagonal scrolling\n");
    printf("  --two-finger-scroll      Require two fingers in scroll mode\n");
    printf("  --deadzone <0.0-0.49>    Central non-activation zone\n");
    printf("  --grab / --no-grab       Grab / do not grab touchpad\n");
    printf("  --device </dev/input/eventX>  Force touchpad device\n");
    printf("  --list-devices           Show available touchpads and exit\n");
    printf("  --version                Show version and exit\n");
    printf("  --verbose                Verbose logging\n");
    printf("  --help                   Show this help\n");
}

int main(int argc, char **argv)
{
    static struct option long_opts[] = {
        {"help", no_argument, NULL, 'h'},
        {"threshold", required_argument, NULL, 't'},
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
        {"grab", no_argument, NULL, 'g'},
        {"no-grab", no_argument, NULL, 'G'},
        {"device", required_argument, NULL, 'd'},
        {"list-devices", no_argument, NULL, 'l'},
        {"version", no_argument, NULL, 'V'},
        {"verbose", no_argument, NULL, 'v'},
        {0, 0, 0, 0},
    };

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
            if (parse_int_arg(optarg, &pulse_step) < 0) {
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
        case 'g':
            use_grab = 1;
            break;
        case 'G':
            use_grab = 0;
            break;
        case 'd':
            forced_devnode = optarg;
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

    if (edge_threshold < 0.01 || edge_threshold > 0.5 || edge_hysteresis < 0.0 ||
        edge_hysteresis >= edge_threshold || hold_ms < 0 || pulse_ms <= 0 || pulse_step <= 0 ||
        max_speed < 1.0 || deadzone < 0.0 || deadzone >= 0.5) {
        fprintf(stderr, "Invalid arguments. See --help.\n");
        return 2;
    }
    if (deadzone + edge_threshold > 0.5) {
        fprintf(stderr, "deadzone + edge_threshold must not exceed 0.5\n");
        return 2;
    }

    struct sigaction sa = {.sa_handler = handle_signal, .sa_flags = 0};
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

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
    int slot_count = 1;
    int active_fingers = 0;
    struct timespec edge_enter_time = {0};

    if (reset_multitouch_state(tp.dev, &slot_x, &slot_y, &slot_active, &slot_count) < 0) {
        fprintf(stderr, "Failed to allocate multitouch state memory.\n");
        goto cleanup;
    }

    struct pollfd pfd = {.fd = tp.input_fd, .events = POLLIN};
    int read_flags = LIBEVDEV_READ_FLAG_NORMAL;

    while (running) {
        int should_active = 0;
        int dx = 0, dy = 0;
        int64_t edge_diff_ms = 0;

        double speed_factor = 0.0;
        int two_finger_ok = !(mode == EM_MODE_SCROLL && two_finger_scroll) || active_fingers >= 2;
        if (max_x <= min_x || max_y <= min_y) {
            last_x = -1;
            last_y = -1;
            continue;
        }

        if (last_x >= 0 && last_y >= 0 && two_finger_ok) {
            double nx = (double)(last_x - min_x) / (double)(max_x - min_x);
            double ny = (double)(last_y - min_y) / (double)(max_y - min_y);
            if (nx > 0.5 - deadzone && nx < 0.5 + deadzone)
                nx = 0.5;
            if (ny > 0.5 - deadzone && ny < 0.5 + deadzone)
                ny = 0.5;
            double depth_x = 0.0;
            double depth_y = 0.0;

            double enter_outer = edge_threshold;
            double leave_inner = edge_threshold - edge_hysteresis;

            if (was_in_edge_x) {
                if (nx >= 1.0 - leave_inner)
                    dx = 1;
                else if (nx <= leave_inner)
                    dx = -1;
            }

            if (!dx) {
                if (nx >= 1.0 - enter_outer)
                    dx = 1;
                else if (nx <= enter_outer)
                    dx = -1;
            }

            if (was_in_edge_y) {
                if (ny >= 1.0 - leave_inner)
                    dy = 1;
                else if (ny <= leave_inner)
                    dy = -1;
            }

            if (!dy) {
                if (ny >= 1.0 - enter_outer)
                    dy = 1;
                else if (ny <= enter_outer)
                    dy = -1;
            }

            if (nx >= 1.0 - edge_threshold)
                depth_x = (nx - (1.0 - edge_threshold)) / edge_threshold;
            else if (nx <= edge_threshold)
                depth_x = (edge_threshold - nx) / edge_threshold;

            if (ny >= 1.0 - edge_threshold)
                depth_y = (ny - (1.0 - edge_threshold)) / edge_threshold;
            else if (ny <= edge_threshold)
                depth_y = (edge_threshold - ny) / edge_threshold;

            if (depth_x > 1.0)
                depth_x = 1.0;
            if (depth_y > 1.0)
                depth_y = 1.0;

            speed_factor = fmax(depth_x, depth_y);

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
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            int64_t now_ms = now.tv_sec * 1000LL + now.tv_nsec / 1000000LL;
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

                        if (current_slot < 0 || current_slot >= slot_count)
                            continue;

                        if (ev.code == ABS_MT_POSITION_X || ev.code == ABS_X) {
                            slot_x[current_slot] = ev.value;
                            if (preferred_slot < 0 || preferred_slot == current_slot)
                                preferred_slot = current_slot;
                            if (preferred_slot == current_slot && slot_y[current_slot] >= 0)
                                last_x = ev.value;
                        }
                        if (ev.code == ABS_MT_POSITION_Y || ev.code == ABS_Y) {
                            slot_y[current_slot] = ev.value;
                            if (preferred_slot < 0 || preferred_slot == current_slot)
                                preferred_slot = current_slot;
                            if (preferred_slot == current_slot && slot_x[current_slot] >= 0)
                                last_y = ev.value;
                        }

                        if (ev.code == ABS_MT_TRACKING_ID) {
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
                               ((ev.code == BTN_TOUCH || ev.code == BTN_TOOL_FINGER ||
                                 ev.code == BTN_TOOL_PEN || ev.code == BTN_TOOL_MOUSE) &&
                                ev.value == 0)) {
                        last_x = -1;
                        last_y = -1;
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
                cleanup_touchpad_resources(&tp);
                pfd.fd = -1;
                struct timespec now;
                clock_gettime(CLOCK_MONOTONIC, &now);
                next_reopen_at_ms = timespec_to_ms(&now) + TOUCHPAD_DISCONNECT_TIMEOUT_MS;
            }
        }

        if (!touchpad_available && running) {
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            int64_t now_ms = now.tv_sec * 1000LL + now.tv_nsec / 1000000LL;
            if (now_ms >= next_reopen_at_ms) {
                if (reopen_touchpad(&tp, &min_x, &max_x, &min_y, &max_y) == 0) {
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

    if (ufd >= 0) {
        ioctl(ufd, UI_DEV_DESTROY);
        close(ufd);
    }

    cleanup_touchpad_resources(&tp);
    free(slot_x);
    free(slot_y);
    free(slot_active);

    pthread_mutex_destroy(&state.lock);
    if (cond_initialized)
        pthread_cond_destroy(&state.cond);

    return 0;
}
