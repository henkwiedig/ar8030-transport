#include "idr_relay.h"

#include "ar8030_chunk.h"
#include "chunker.h"
#include <arpa/inet.h>
#include <ctype.h>
#include <errno.h>
#include <netinet/in.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define IDR_RELAY_WRITE_TIMEOUT_MS 100
#define IDR_RELAY_TOKEN_MAX        16

/* ar8030_chunk_send_fn: one small control chunk into the video socket's
 * reverse direction. The fd is reloaded per call because rx's main loop
 * reopens the socket after a daemon reconnect (see ar8030_link.h). */
static int ctrl_send(void *ctx, const uint8_t *buf, uint32_t len)
{
    ar8030_link_t *link = ctx;
    int sockfd = __atomic_load_n(&link->sockfd, __ATOMIC_ACQUIRE);
    if (sockfd < 0)
        return -1;
    uint32_t sent = 0;
    while (sent < len) {
        int wr = bb_socket_write(sockfd, buf + sent, len - sent, IDR_RELAY_WRITE_TIMEOUT_MS);
        if (wr <= 0)
            return -1;
        sent += (uint32_t)wr;
    }
    return 0;
}

void *idr_relay_thread_main(void *arg)
{
    idr_relay_cfg_t *cfg = arg;

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        fprintf(stderr, "rx: idr relay: socket failed: %s\n", strerror(errno));
        return NULL;
    }
    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    /* Any address, like alink_idr: PixelPilot sends to whichever address
     * our RTP came from, which is only loopback when it runs locally. */
    struct sockaddr_in addr = {.sin_family = AF_INET, .sin_port = htons((uint16_t)cfg->udp_port),
                               .sin_addr.s_addr = htonl(INADDR_ANY)};
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        fprintf(stderr, "rx: idr relay: bind UDP %d failed: %s -- keyframe requests disabled\n",
                cfg->udp_port, strerror(errno));
        close(fd);
        return NULL;
    }
    fprintf(stderr, "rx: idr relay listening on UDP %d\n", cfg->udp_port);

    uint8_t scratch[AR8030_CHUNK_HDR_SIZE + AR8030_CTRL_MAX_PAYLOAD];
    uint16_t ctrl_seq = 0;

    while (!*cfg->stop_flag) {
        struct pollfd pfd = {.fd = fd, .events = POLLIN};
        if (poll(&pfd, 1, 500) <= 0)
            continue;

        char buf[64];
        struct sockaddr_in from;
        socklen_t from_len = sizeof(from);
        ssize_t n = recvfrom(fd, buf, sizeof(buf) - 1, 0, (struct sockaddr *)&from, &from_len);
        if (n <= 0)
            continue;
        buf[n] = '\0';
        while (n > 0 && isspace((unsigned char)buf[n - 1]))
            buf[--n] = '\0';
        if (n == 0 || n > IDR_RELAY_TOKEN_MAX)
            continue;

        /* Same answer alink_idr gave, for any sender that looks for it. */
        char ack[IDR_RELAY_TOKEN_MAX + 8];
        int ack_len = snprintf(ack, sizeof(ack), "ACK:%s\n", buf);
        sendto(fd, ack, (size_t)ack_len, 0, (struct sockaddr *)&from, from_len);

        char payload[AR8030_CTRL_MAX_PAYLOAD];
        int payload_len = snprintf(payload, sizeof(payload), "IDR %s", buf);
        uint32_t chunks = 0;
        int sent = ar8030_chunk_frame(ctrl_seq++, 0, AR8030_CHUNK_CODEC_CTRL, 0, (const uint8_t *)payload,
                                      (uint32_t)payload_len, AR8030_CTRL_MAX_PAYLOAD, ctrl_send, cfg->link,
                                      &chunks, scratch, sizeof(scratch));
        if (cfg->verbose || sent != 1)
            fprintf(stderr, "rx: idr request token=%s -> air %s\n", buf, sent == 1 ? "sent" : "FAILED");
    }

    close(fd);
    return NULL;
}
