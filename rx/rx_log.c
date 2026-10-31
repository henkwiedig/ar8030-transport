#include "rx_log.h"

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

static const char *g_log_path;

static void log_banner(const char *what)
{
    char ts[32] = "?";
    time_t now = time(NULL);
    struct tm tm;
    if (localtime_r(&now, &tm))
        strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &tm);
    fprintf(stderr, "rx: log %s %s (pid %d)\n", what, ts, (int)getpid());
}

/* Points fds 1 and 2 at a fresh open of g_log_path. dup2() swaps each
 * descriptor atomically, so a line another thread writes meanwhile lands
 * whole in either the old file or the new one. */
static int reopen_log(void)
{
    int fd = open(g_log_path, O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0644);
    if (fd < 0)
        return -1;
    fflush(stdout);
    dup2(fd, STDOUT_FILENO);
    dup2(fd, STDERR_FILENO);
    close(fd);
    return 0;
}

static void shift_old_logs(void)
{
    char from[PATH_MAX], to[PATH_MAX];
    for (int i = RX_LOG_KEEP; i > 1; i--) {
        snprintf(from, sizeof(from), "%s.%d", g_log_path, i - 1);
        snprintf(to, sizeof(to), "%s.%d", g_log_path, i);
        rename(from, to); /* ENOENT until there have been i-1 rotations */
    }
    snprintf(to, sizeof(to), "%s.1", g_log_path);
    rename(g_log_path, to);
}

static void *rotate_thread_main(void *arg)
{
    (void)arg;
    for (;;) {
        sleep(1);
        struct stat st;
        if (fstat(STDERR_FILENO, &st) != 0 || st.st_size < RX_LOG_MAX_BYTES)
            continue;
        shift_old_logs();
        if (reopen_log() == 0) {
            log_banner("rotated");
        } else {
            /* Still writing to what is now <file>.1: cut it rather than
             * let it grow. */
            int err = errno;
            if (ftruncate(STDERR_FILENO, 0) != 0)
                continue;
            fprintf(stderr, "rx: log reopen of %s failed (%s), truncated instead\n", g_log_path,
                    strerror(err));
        }
    }
    return NULL;
}

int rx_log_open(const char *path)
{
    g_log_path = path;

    struct stat st;
    if (stat(path, &st) == 0 && st.st_size > 0)
        shift_old_logs();
    if (reopen_log() != 0) {
        fprintf(stderr, "rx: can't open log file %s: %s\n", path, strerror(errno));
        return -1;
    }
    /* rx itself never prints to stdout, the SDK client library may:
     * line-buffer it so such lines interleave with ours instead of
     * arriving in 4 KB blocks. */
    setvbuf(stdout, NULL, _IOLBF, 0);
    log_banner("opened");

    pthread_t thread;
    if (pthread_create(&thread, NULL, rotate_thread_main, NULL) == 0)
        pthread_detach(thread);
    else
        fprintf(stderr, "rx: log rotation thread failed to start, %s will not be capped\n", path);
    return 0;
}
