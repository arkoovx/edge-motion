#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <libevdev/libevdev.h>
#include <linux/uinput.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <libudev.h>
#include <unistd.h>

#define DEFAULT_EDGE_THRESHOLD 0.06
#define DEFAULT_HOLD_MS 80
#define DEFAULT_PULSE_MS 10
#define DEFAULT_PULSE_STEP 3

static double edge_threshold = DEFAULT_EDGE_THRESHOLD;
static int hold_ms = DEFAULT_HOLD_MS;
static int pulse_ms = DEFAULT_PULSE_MS;
static int pulse_step = DEFAULT_PULSE_STEP;
static int verbose = 0;

static volatile sig_atomic_t running = 1;

struct em_state {
    pthread_mutex_t lock;
    pthread_cond_t cond;
    int edge_active;
    int dir_x;
    int dir_y;
};

static struct em_state state = {
    .lock = PTHREAD_MUTEX_INITIALIZER,
    .edge_active = 0,
    .dir_x = 0,
    .dir_y = 0,
};

static void handle_signal(int sig)
{
    (void)sig;
    running = 0;
}

static inline int emit_rel(int ufd, int code, int val)
{
    struct input_event ev = {0};
    ev.type = EV_REL;
    ev.code = code;
    ev.value = val;
    return write(ufd, &ev, sizeof(ev)) == (ssize_t)sizeof(ev) ? 0 : -1;
}

static inline int emit_syn(int ufd)
{
    struct input_event ev = {0};
    ev.type = EV_SYN;
    ev.code = SYN_REPORT;
    return write(ufd, &ev, sizeof(ev)) == (ssize_t)sizeof(ev) ? 0 : -1;
}

static int create_uinput_device(void)
{
    int fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
    if (fd < 0)
        return -1;

    if (ioctl(fd, UI_SET_EVBIT, EV_KEY) < 0 || ioctl(fd, UI_SET_KEYBIT, BTN_LEFT) < 0 ||
        ioctl(fd, UI_SET_KEYBIT, BTN_RIGHT) < 0 || ioctl(fd, UI_SET_EVBIT, EV_REL) < 0 ||
        ioctl(fd, UI_SET_RELBIT, REL_X) < 0 || ioctl(fd, UI_SET_RELBIT, REL_Y) < 0) {
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
        close(fd);
        return -1;
    }

    usleep(20000);
    return fd;
}

static char *find_touchpad_devnode(void)
{
    struct udev *udev = udev_new();
    if (!udev)
        return NULL;

    struct udev_enumerate *en = udev_enumerate_new(udev);
    if (!en) {
        udev_unref(udev);
        return NULL;
    }

    udev_enumerate_add_match_subsystem(en, "input");
    udev_enumerate_add_match_property(en, "ID_INPUT_TOUCHPAD", "1");
    udev_enumerate_scan_devices(en);

    char *result = NULL;
    struct udev_list_entry *entry;
    udev_list_entry_foreach(entry, udev_enumerate_get_list_entry(en)) {
        struct udev_device *dev =
            udev_device_new_from_syspath(udev, udev_list_entry_get_name(entry));
        if (!dev)
            continue;

        const char *devnode = udev_device_get_devnode(dev);
        if (devnode && strstr(devnode, "/event")) {
            result = strdup(devnode);
            udev_device_unref(dev);
            break;
        }

        udev_device_unref(dev);
    }

    udev_enumerate_unref(en);
    udev_unref(udev);
    return result;
}

static int reopen_touchpad(char **devnode_ptr, int *input_fd_ptr, struct libevdev **dev_ptr,
                           int *min_x, int *max_x, int *min_y, int *max_y, int *bounce_x,
                           int *bounce_y)
{
    if (*devnode_ptr) {
        free(*devnode_ptr);
        *devnode_ptr = NULL;
    }

    if (*input_fd_ptr >= 0) {
        close(*input_fd_ptr);
        *input_fd_ptr = -1;
    }

    if (*dev_ptr) {
        libevdev_free(*dev_ptr);
        *dev_ptr = NULL;
    }

    *devnode_ptr = find_touchpad_devnode();
    if (!*devnode_ptr)
        return -1;

    *input_fd_ptr = open(*devnode_ptr, O_RDONLY | O_NONBLOCK);
    if (*input_fd_ptr < 0) {
        free(*devnode_ptr);
        *devnode_ptr = NULL;
        return -1;
    }

    if (libevdev_new_from_fd(*input_fd_ptr, dev_ptr) < 0) {
        close(*input_fd_ptr);
        *input_fd_ptr = -1;
        free(*devnode_ptr);
        *devnode_ptr = NULL;
        return -1;
    }

    const struct input_absinfo *absx = libevdev_get_abs_info(*dev_ptr, ABS_MT_POSITION_X);
    if (!absx)
        absx = libevdev_get_abs_info(*dev_ptr, ABS_X);
    const struct input_absinfo *absy = libevdev_get_abs_info(*dev_ptr, ABS_MT_POSITION_Y);
    if (!absy)
        absy = libevdev_get_abs_info(*dev_ptr, ABS_Y);

    if (!absx || !absy) {
        libevdev_free(*dev_ptr);
        *dev_ptr = NULL;
        close(*input_fd_ptr);
        *input_fd_ptr = -1;
        free(*devnode_ptr);
        *devnode_ptr = NULL;
        return -1;
    }

    *min_x = absx->minimum;
    *max_x = absx->maximum;
    *min_y = absy->minimum;
    *max_y = absy->maximum;

    *bounce_x = (int)((*max_x - *min_x) * 0.002 + 0.5);
    if (*bounce_x < 2)
        *bounce_x = 2;

    *bounce_y = (int)((*max_y - *min_y) * 0.002 + 0.5);
    if (*bounce_y < 2)
        *bounce_y = 2;

    return 0;
}

static void get_timeout_timespec(struct timespec *ts, int ms)
{
    clock_gettime(CLOCK_MONOTONIC, ts);
    ts->tv_nsec += (ms % 1000) * 1000000L;
    ts->tv_sec += ms / 1000;
    if (ts->tv_nsec >= 1000000000L) {
        ts->tv_sec += 1;
        ts->tv_nsec -= 1000000000L;
    }
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

        int dx = state.dir_x;
        int dy = state.dir_y;
        pthread_mutex_unlock(&state.lock);

        if (dx || dy) {
            if (dx)
                emit_rel(ufd, REL_X, dx * pulse_step);
            if (dy)
                emit_rel(ufd, REL_Y, dy * pulse_step);
            emit_syn(ufd);
        }

        pthread_mutex_lock(&state.lock);
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
    printf("edge-motion v4.2 Ultimate — движение курсора у края тачпада\n\n");
    printf("Usage: %s [OPTIONS]\n", prog);
    printf("  --threshold <0.01-0.5>   Порог края (default %.2f)\n", DEFAULT_EDGE_THRESHOLD);
    printf("  --hold-ms <ms>           Задержка (default %d)\n", DEFAULT_HOLD_MS);
    printf("  --pulse-ms <ms>          Интервал импульсов (default %d)\n", DEFAULT_PULSE_MS);
    printf("  --pulse-step <n>         Скорость (default %d)\n", DEFAULT_PULSE_STEP);
    printf("  --verbose                Подробный вывод\n");
    printf("  --help                   Эта справка\n");
}

int main(int argc, char **argv)
{
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--help")) {
            print_usage(argv[0]);
            return 0;
        }

        if (!strcmp(argv[i], "--threshold") && i + 1 < argc)
            edge_threshold = atof(argv[++i]);
        else if (!strcmp(argv[i], "--hold-ms") && i + 1 < argc)
            hold_ms = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pulse-ms") && i + 1 < argc)
            pulse_ms = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--pulse-step") && i + 1 < argc)
            pulse_step = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--verbose"))
            verbose = 1;
    }

    struct sigaction sa = {.sa_handler = handle_signal, .sa_flags = SA_RESTART};
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    char *devnode = NULL;
    int input_fd = -1;
    struct libevdev *dev = NULL;

    int min_x = 0, max_x = 0;
    int min_y = 0, max_y = 0;
    int bounce_thr_x = 2, bounce_thr_y = 2;

    if (reopen_touchpad(&devnode, &input_fd, &dev, &min_x, &max_x, &min_y, &max_y, &bounce_thr_x,
                        &bounce_thr_y) < 0) {
        fprintf(stderr, "Тачпад не найден.\n");
        return 1;
    }

    int cond_initialized = 0;

    int ufd = create_uinput_device();
    if (ufd < 0) {
        fprintf(stderr, "Не удалось создать uinput (нужен root/cap_sys_admin).\n");
        goto cleanup;
    }

    pthread_condattr_t cattr;
    if (pthread_condattr_init(&cattr) != 0 ||
        pthread_condattr_setclock(&cattr, CLOCK_MONOTONIC) != 0 ||
        pthread_cond_init(&state.cond, &cattr) != 0) {
        fprintf(stderr, "Не удалось инициализировать condition variable.\n");
        pthread_condattr_destroy(&cattr);
        goto cleanup;
    }
    pthread_condattr_destroy(&cattr);
    cond_initialized = 1;

    pthread_t thr;
    int thread_started = 0;
    if (pthread_create(&thr, NULL, pulser_thread, (void *)(intptr_t)ufd) != 0) {
        fprintf(stderr, "Не удалось создать pulser thread.\n");
        goto cleanup;
    }
    thread_started = 1;

    int last_x = -1, last_y = -1;
    int prev_x = -1, prev_y = -1;
    int current_slot = 0;
    struct timespec last_motion;
    clock_gettime(CLOCK_MONOTONIC, &last_motion);

    struct pollfd pfd = {.fd = input_fd, .events = POLLIN};
    int read_flags = LIBEVDEV_READ_FLAG_NORMAL;

    while (running) {
        int should_active = 0;
        int dx = 0, dy = 0;
        long diff_ms = 0;

        if (last_x >= 0 && last_y >= 0 && max_x > min_x && max_y > min_y) {
            double nx = (double)(last_x - min_x) / (double)(max_x - min_x);
            double ny = (double)(last_y - min_y) / (double)(max_y - min_y);

            if (nx >= 1.0 - edge_threshold)
                dx = 1;
            else if (nx <= edge_threshold)
                dx = -1;

            if (ny >= 1.0 - edge_threshold)
                dy = 1;
            else if (ny <= edge_threshold)
                dy = -1;

            if (dx || dy) {
                struct timespec now;
                clock_gettime(CLOCK_MONOTONIC, &now);
                diff_ms = (now.tv_sec - last_motion.tv_sec) * 1000L +
                          (now.tv_nsec - last_motion.tv_nsec) / 1000000L;
                if (diff_ms >= hold_ms)
                    should_active = 1;
            }
        }

        pthread_mutex_lock(&state.lock);
        int changed = (state.edge_active != should_active || state.dir_x != dx || state.dir_y != dy);
        state.edge_active = should_active;
        state.dir_x = dx;
        state.dir_y = dy;
        if (changed)
            pthread_cond_signal(&state.cond);
        pthread_mutex_unlock(&state.lock);

        int timeout_ms = -1;
        if (!should_active && (dx || dy)) {
            int remaining = hold_ms - (int)diff_ms;
            timeout_ms = remaining > 0 ? remaining : 0;
        }

        int ret = poll(&pfd, 1, timeout_ms);
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

                while ((rc = libevdev_next_event(dev, read_flags, &ev)) >= 0) {
                    if (rc == LIBEVDEV_READ_STATUS_SYNC)
                        read_flags = LIBEVDEV_READ_FLAG_SYNC;

                    if (ev.type == EV_SYN && ev.code == SYN_REPORT) {
                        sync_received = 1;
                        continue;
                    }

                    if (ev.type == EV_ABS) {
                        if (ev.code == ABS_MT_SLOT)
                            current_slot = ev.value;

                        if (current_slot == 0) {
                            if (ev.code == ABS_MT_POSITION_X || ev.code == ABS_X)
                                last_x = ev.value;
                            if (ev.code == ABS_MT_POSITION_Y || ev.code == ABS_Y)
                                last_y = ev.value;
                        }

                        if (ev.code == ABS_MT_TRACKING_ID && current_slot == 0 && ev.value == -1) {
                            last_x = -1;
                            last_y = -1;
                            prev_x = -1;
                            prev_y = -1;
                        }
                    } else if (ev.type == EV_KEY && ev.code == BTN_TOUCH && ev.value == 0) {
                        last_x = -1;
                        last_y = -1;
                        prev_x = -1;
                        prev_y = -1;
                    }
                }

                if (rc == -EAGAIN && read_flags == LIBEVDEV_READ_FLAG_SYNC)
                    read_flags = LIBEVDEV_READ_FLAG_NORMAL;
            }

            if (sync_received && last_x >= 0 && last_y >= 0) {
                if (prev_x < 0 || prev_y < 0) {
                    clock_gettime(CLOCK_MONOTONIC, &last_motion);
                    prev_x = last_x;
                    prev_y = last_y;
                } else if (abs(last_x - prev_x) > bounce_thr_x || abs(last_y - prev_y) > bounce_thr_y) {
                    clock_gettime(CLOCK_MONOTONIC, &last_motion);
                    prev_x = last_x;
                    prev_y = last_y;
                }
            }

            if (rc < 0 && rc != -EAGAIN) {
                if (verbose)
                    fprintf(stderr, "Тачпад отключён, переподключаюсь...\n");

                pthread_mutex_lock(&state.lock);
                state.edge_active = 0;
                state.dir_x = 0;
                state.dir_y = 0;
                pthread_cond_signal(&state.cond);
                pthread_mutex_unlock(&state.lock);

                last_x = -1;
                last_y = -1;
                prev_x = -1;
                prev_y = -1;
                clock_gettime(CLOCK_MONOTONIC, &last_motion);

                while (running &&
                       reopen_touchpad(&devnode, &input_fd, &dev, &min_x, &max_x, &min_y, &max_y,
                                       &bounce_thr_x, &bounce_thr_y) < 0) {
                    usleep(200000);
                }
                if (!running)
                    break;

                pfd.fd = input_fd;
                pfd.events = POLLIN;
                pfd.revents = 0;
                read_flags = LIBEVDEV_READ_FLAG_NORMAL;
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

    if (dev)
        libevdev_free(dev);
    if (input_fd >= 0)
        close(input_fd);
    if (devnode)
        free(devnode);

    pthread_mutex_destroy(&state.lock);
    if (cond_initialized)
        pthread_cond_destroy(&state.cond);

    return 0;
}
