#include "ar8030_link.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>

int ar8030_link_connect(ar8030_link_t *link, const char *daemon_ip, int daemon_port)
{
    memset(link, 0, sizeof(*link));
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

    link->dev = bb_dev_open(devs[0]);
    bb_dev_freelist(devs);

    if (!link->dev) {
        fprintf(stderr, "ar8030_link: bb_dev_open failed\n");
        bb_host_disconnect(link->host);
        link->host = NULL;
        return -1;
    }

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
    if (!link->dev)
        return -1;

    int fd = bb_socket_open(link->dev, slot, port, flag, opt);
    if (fd < 0) {
        fprintf(stderr, "ar8030_link: bb_socket_open(slot=%d, port=%u) failed (ret=%d)\n", slot,
                port, fd);
        return -1;
    }

    link->sockfd = fd;
    return 0;
}

void ar8030_link_close(ar8030_link_t *link)
{
    if (!link)
        return;

    if (link->sockfd >= 0) {
        bb_socket_close(link->sockfd);
        link->sockfd = -1;
    }
    if (link->dev) {
        bb_dev_close(link->dev);
        link->dev = NULL;
    }
    if (link->host) {
        bb_host_disconnect(link->host);
        link->host = NULL;
    }
}
