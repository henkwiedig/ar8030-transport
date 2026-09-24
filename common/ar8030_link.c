#include "ar8030_link.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>

int ar8030_link_connect(ar8030_link_t *link, const char *daemon_ip, int daemon_port)
{
    /* Field-by-field, not a blanket memset(link, 0, ...): a mid-session
     * reconnect (ar8030_link_reconnect_retry()) can run this while tx's
     * bitrate_ctl.c thread is concurrently reading link->dev, and that
     * field must go through an atomic store even for "reset to NULL"
     * on a failed attempt -- see this struct's own header comment. */
    __atomic_store_n(&link->dev, NULL, __ATOMIC_RELEASE);
    link->host = NULL;
    link->sockfd = -1;

    int ret = bb_host_connect(&link->host, daemon_ip, daemon_port);
    if (ret) {
        fprintf(stderr, "ar8030_link: bb_host_connect(%s:%d) failed (ret=%d) -- is ar8030d running?\n",
                daemon_ip, daemon_port, ret);
        return -1;
    }

    bb_dev_t **devs = NULL;
    int dev_cnt = bb_dev_getlist(link->host, &devs);
    if (dev_cnt <= 0) {
        fprintf(stderr, "ar8030_link: no AR8030 device known to the daemon\n");
        bb_host_disconnect(link->host);
        link->host = NULL;
        return -1;
    }

    bb_dev_handle_t *dev = bb_dev_open(devs[0]);
    bb_dev_freelist(devs);

    if (!dev) {
        fprintf(stderr, "ar8030_link: bb_dev_open failed\n");
        bb_host_disconnect(link->host);
        link->host = NULL;
        return -1;
    }

    __atomic_store_n(&link->dev, dev, __ATOMIC_RELEASE);
    return 0;
}

int ar8030_link_connect_retry(ar8030_link_t *link, const char *daemon_ip, int daemon_port,
                               int retry_ms, const volatile int *stop_flag)
{
    for (;;) {
        if (ar8030_link_connect(link, daemon_ip, daemon_port) == 0)
            return 0;

        if (stop_flag && *stop_flag)
            return -1;

        usleep((useconds_t)retry_ms * 1000);

        if (stop_flag && *stop_flag)
            return -1;
    }
}

int ar8030_link_open_socket(ar8030_link_t *link, bb_slot_e slot, uint32_t port, uint32_t flag,
                             bb_sock_opt_t *opt)
{
    bb_dev_handle_t *dev = __atomic_load_n(&link->dev, __ATOMIC_ACQUIRE);
    if (!dev)
        return -1;

    int fd = bb_socket_open(dev, slot, port, flag, opt);
    if (fd < 0) {
        fprintf(stderr, "ar8030_link: bb_socket_open(slot=%d, port=%u) failed (ret=%d)\n", slot,
                port, fd);
        return -1;
    }

    __atomic_store_n(&link->sockfd, fd, __ATOMIC_RELEASE); /* read by the IDR control threads */
    return 0;
}

int ar8030_link_force_close_socket(ar8030_link_t *link, bb_slot_e slot, uint32_t port)
{
    bb_dev_handle_t *dev = __atomic_load_n(&link->dev, __ATOMIC_ACQUIRE);
    if (!dev)
        return -1;

    bb_force_close_socket_t fc = { .slot = (uint8_t)slot, .port = (uint8_t)port };
    return bb_ioctl(dev, BB_FORCE_CLS_SOCKET, &fc, NULL);
}

int ar8030_link_force_close_all_sockets(ar8030_link_t *link)
{
    bb_dev_handle_t *dev = __atomic_load_n(&link->dev, __ATOMIC_ACQUIRE);
    if (!dev)
        return -1;

    return bb_ioctl(dev, BB_FORCE_CLS_SOCKET_ALL, NULL, NULL);
}

void ar8030_link_close(ar8030_link_t *link)
{
    if (!link)
        return;

    if (link->sockfd >= 0) {
        bb_socket_close(link->sockfd);
        __atomic_store_n(&link->sockfd, -1, __ATOMIC_RELEASE);
    }

    /* Atomic exchange, not a plain read-then-close: publishes NULL to any
     * concurrent reader (tx's bitrate_ctl.c thread) as one indivisible
     * step with grabbing the handle to close, rather than a check
     * followed by a separate close-and-clear that leaves a window where
     * another thread could still observe and use the about-to-be-freed
     * handle. This ordering is what closed the exact use-after-free race
     * the frame-shm ring reattach fix hit on its first live test (see
     * README.md) -- NULL is already a value that reader has to treat as
     * "skip this tick" regardless, so publishing it first costs nothing. */
    bb_dev_handle_t *dev = __atomic_exchange_n(&link->dev, NULL, __ATOMIC_ACQ_REL);
    if (dev)
        bb_dev_close(dev);

    if (link->host) {
        bb_host_disconnect(link->host);
        link->host = NULL;
    }
}

int ar8030_link_is_alive(ar8030_link_t *link)
{
    bb_dev_handle_t *dev = __atomic_load_n(&link->dev, __ATOMIC_ACQUIRE);
    if (!dev)
        return 0;

    bb_get_status_in_t in;
    bb_get_status_out_t out;
    memset(&in, 0, sizeof(in));
    memset(&out, 0, sizeof(out));
    in.user_bmp = 0; /* not concerned with physical-layer info -- see bb_api.h's own doc comment */

    return bb_ioctl(dev, BB_GET_STATUS, &in, &out) == 0;
}

int ar8030_link_reconnect_retry(ar8030_link_t *link, const char *daemon_ip, int daemon_port,
                                 int retry_ms, const volatile int *stop_flag)
{
    ar8030_link_close(link);
    return ar8030_link_connect_retry(link, daemon_ip, daemon_port, retry_ms, stop_flag);
}
