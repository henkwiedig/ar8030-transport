#include "sidecar_sub.h"

#include "sidecar_wire.h"
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#define SIDECAR_SUB_REFRESH_MS 2000
#define SIDECAR_SUB_POLL_MS 500

static long long now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

void *sidecar_sub_thread_main(void *arg)
{
    sidecar_sub_cfg_t *cfg = arg;

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        fprintf(stderr, "tx: sidecar: socket failed: %s -- sidecar not forwarded\n", strerror(errno));
        return NULL;
    }
    /* connect() so only waybeam's own replies are received, and send()
     * needs no address. */
    struct sockaddr_in addr = {.sin_family = AF_INET, .sin_port = htons((uint16_t)cfg->port),
                               .sin_addr.s_addr = htonl(INADDR_LOOPBACK)};
    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        fprintf(stderr, "tx: sidecar: connect 127.0.0.1:%d failed: %s -- sidecar not forwarded\n",
                cfg->port, strerror(errno));
        close(fd);
        return NULL;
    }
    fprintf(stderr, "tx: forwarding waybeam sidecar from 127.0.0.1:%d\n", cfg->port);

    struct sidecar_hdr sub = {.magic = htonl(SIDECAR_MAGIC), .version = SIDECAR_VERSION,
                              .msg_type = SIDECAR_MSG_SUBSCRIBE};
    long long last_sub_ms = 0;
    tx_outq_item_t item;
    memset(&item, 0, sizeof(item));
    item.codec = AR8030_CHUNK_CODEC_SIDECAR;

    while (!*cfg->stop_flag) {
        long long now = now_ms();
        if (now - last_sub_ms >= SIDECAR_SUB_REFRESH_MS) {
            /* ECONNREFUSED while waybeam is down or restarting is expected;
             * the next refresh tries again. */
            send(fd, &sub, sizeof(sub), 0);
            last_sub_ms = now;
        }

        struct pollfd pfd = {.fd = fd, .events = POLLIN};
        if (poll(&pfd, 1, SIDECAR_SUB_POLL_MS) <= 0)
            continue;
        ssize_t n = recv(fd, item.data, sizeof(item.data), 0);
        if (n < (ssize_t)sizeof(struct sidecar_hdr))
            continue;
        const struct sidecar_hdr *h = (const struct sidecar_hdr *)item.data;
        if (ntohl(h->magic) != SIDECAR_MAGIC || h->version != SIDECAR_VERSION ||
            h->msg_type != SIDECAR_MSG_FRAME)
            continue;
        item.len = (uint16_t)n;
        tx_outq_push(cfg->outq, &item);
        cfg->forwarded++;
    }

    close(fd);
    return NULL;
}
