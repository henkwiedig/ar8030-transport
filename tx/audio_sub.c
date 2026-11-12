#include "audio_sub.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define AUDIO_SUB_POLL_MS 500
#define RTP_HEADER_MIN 12

void *audio_sub_thread_main(void *arg)
{
    audio_sub_cfg_t *cfg = arg;

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        fprintf(stderr, "tx: audio: socket failed: %s -- audio not forwarded\n", strerror(errno));
        return NULL;
    }
    struct sockaddr_in addr = {.sin_family = AF_INET, .sin_port = htons((uint16_t)cfg->port),
                               .sin_addr.s_addr = htonl(INADDR_LOOPBACK)};
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        fprintf(stderr, "tx: audio: bind 127.0.0.1:%d failed: %s -- audio not forwarded\n", cfg->port,
                strerror(errno));
        close(fd);
        return NULL;
    }
    fprintf(stderr, "tx: forwarding waybeam audio from 127.0.0.1:%d\n", cfg->port);

    tx_outq_item_t item;
    memset(&item, 0, sizeof(item));
    item.codec = AR8030_CHUNK_CODEC_OPUS;

    while (!*cfg->stop_flag) {
        struct pollfd pfd = {.fd = fd, .events = POLLIN};
        if (poll(&pfd, 1, AUDIO_SUB_POLL_MS) <= 0)
            continue;
        /* One byte more than fits: a datagram that fills the buffer was cut. */
        uint8_t buf[AR8030_SIDECAR_MAX_PAYLOAD + 1];
        ssize_t n = recv(fd, buf, sizeof(buf), 0);
        if (n < RTP_HEADER_MIN || (buf[0] >> 6) != 2)
            continue; /* not RTP */
        if ((size_t)n > AR8030_SIDECAR_MAX_PAYLOAD) {
            cfg->too_big++;
            continue;
        }
        memcpy(item.data, buf, (size_t)n);
        item.len = (uint16_t)n;
        tx_outq_push(cfg->outq, &item);
        cfg->forwarded++;
    }

    close(fd);
    return NULL;
}
