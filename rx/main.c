/*
 * ar8030-transport-rx -- ground-side bridge from the AR8030's non-IP
 * baseband data channel (bb_socket) to PixelPilot_rk.
 *
 * Reads ar8030_chunk_hdr-prefixed chunks (common/ar8030_chunk.h) off a
 * datagram-mode bb_socket, reassembles them into whole Annex-B H.265
 * access units, RTP/H.265-packetizes each (rx/rtp_h265.c) and sends the
 * packets over UDP to PixelPilot's existing RTP listener -- no
 * PixelPilot_rk source changes; point a stock `pixelpilot -p 5600` at
 * this tool's -H/-p target.
 *
 * Reassembly assumes chunk order is preserved by the transport (bb_socket
 * is a queued point-to-point channel, not a packet-switched network, so
 * this holds unlike plain UDP -- see README.md "Reassembly ordering").
 * A chunk that doesn't extend the frame currently being assembled (wrong
 * frame_seq, or an out-of-order chunk_idx) drops whatever was collected
 * so far and, if it looks like the start of a new frame, restarts from
 * it -- no waiting, no NACK, freshness over completeness.
 */

#include "ar8030_chunk.h"
#include "ar8030_link.h"
#include "reassembly.h"
#include "rtp_h265.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netdb.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define DEFAULT_DAEMON_IP "127.0.0.1"
#define DEFAULT_TARGET_HOST "127.0.0.1"
#define DEFAULT_TARGET_PORT 5600 /* PixelPilot_rk's own -p default */
#define DEFAULT_VIDEO_PORT 2     /* must match tx/main.c's -o, see its comment */
#define DEFAULT_MAX_RTP_PAYLOAD 1400
#define DEFAULT_REASSEMBLY_MAX (512 * 1024) /* venc_frame_ring.h: up to 512 KB slots (CV610) */
#define RETRY_MS 2000

static volatile int g_stop;

static void on_signal(int sig)
{
    (void)sig;
    g_stop = 1;
}

struct rx_args {
    const char *daemon_ip;
    int daemon_port;
    int port;
    const char *target_host;
    int target_port;
    uint16_t rtp_max_payload;
    uint32_t reassembly_max;
};

static void usage(const char *argv0)
{
    fprintf(stderr,
            "usage: %s [options]\n"
            "  -d <ip>        ar8030d daemon IP (default %s)\n"
            "  -o <port>      AR8030 bb_socket logical port, 0-3 (default %d)\n"
            "  -H <host>      PixelPilot UDP target host (default %s)\n"
            "  -p <port>      PixelPilot UDP target port (default %d)\n"
            "  -M <bytes>     max RTP payload before FU fragmentation (default %d)\n"
            "  -b <bytes>     max reassembled frame size (default %d)\n"
            "  -h             this help\n",
            argv0, DEFAULT_DAEMON_IP, DEFAULT_VIDEO_PORT, DEFAULT_TARGET_HOST, DEFAULT_TARGET_PORT,
            DEFAULT_MAX_RTP_PAYLOAD, DEFAULT_REASSEMBLY_MAX);
}

static int parse_args(int argc, char **argv, struct rx_args *a)
{
    a->daemon_ip = DEFAULT_DAEMON_IP;
    a->daemon_port = BB_PORT_DEFAULT;
    a->port = DEFAULT_VIDEO_PORT;
    a->target_host = DEFAULT_TARGET_HOST;
    a->target_port = DEFAULT_TARGET_PORT;
    a->rtp_max_payload = DEFAULT_MAX_RTP_PAYLOAD;
    a->reassembly_max = DEFAULT_REASSEMBLY_MAX;

    int opt;
    while ((opt = getopt(argc, argv, "d:o:H:p:M:b:h")) != -1) {
        switch (opt) {
        case 'd':
            a->daemon_ip = optarg;
            break;
        case 'o':
            a->port = atoi(optarg);
            break;
        case 'H':
            a->target_host = optarg;
            break;
        case 'p':
            a->target_port = atoi(optarg);
            break;
        case 'M':
            a->rtp_max_payload = (uint16_t)strtoul(optarg, NULL, 10);
            break;
        case 'b':
            a->reassembly_max = (uint32_t)strtoul(optarg, NULL, 10);
            break;
        case 'h':
        default:
            usage(argv[0]);
            return -1;
        }
    }
    return 0;
}

static int open_udp_target(const char *host, int port)
{
    char portbuf[8];
    snprintf(portbuf, sizeof(portbuf), "%d", port);

    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;

    struct addrinfo *res = NULL;
    if (getaddrinfo(host, portbuf, &hints, &res) != 0 || !res) {
        fprintf(stderr, "rx: getaddrinfo(%s:%d) failed\n", host, port);
        return -1;
    }

    int fd = -1;
    for (struct addrinfo *rp = res; rp; rp = rp->ai_next) {
        fd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
        if (fd < 0)
            continue;
        if (connect(fd, rp->ai_addr, rp->ai_addrlen) == 0)
            break;
        close(fd);
        fd = -1;
    }
    freeaddrinfo(res);

    if (fd < 0)
        fprintf(stderr, "rx: failed to open UDP socket to %s:%d\n", host, port);
    return fd;
}

int main(int argc, char **argv)
{
    struct rx_args args;
    if (parse_args(argc, argv, &args) != 0)
        return 1;

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    ar8030_link_t link;
    fprintf(stderr, "rx: connecting to ar8030d at %s:%d...\n", args.daemon_ip, args.daemon_port);
    if (ar8030_link_connect_retry(&link, args.daemon_ip, args.daemon_port, RETRY_MS, &g_stop) != 0)
        return 0;

    bb_sock_opt_t sock_opt;
    sock_opt.tx_buf_size = 1024;
    sock_opt.rx_buf_size = 64 * 1024;
    /* Ground is DEV role: bb_socket_open()'s slot parameter is ignored by
     * the SDK for a DEV and always addresses its one AP peer -- see
     * bb_api.h's bb_socket_open doc comment ("If DEV, target SLOT is
     * BB_SLOT_AP"). Passing BB_SLOT_AP explicitly documents that rather
     * than relying on the SDK's silent override. */
    if (ar8030_link_open_socket(&link, BB_SLOT_AP, (uint32_t)args.port,
                                 BB_SOCK_FLAG_RX | BB_SOCK_FLAG_DATAGRAM, &sock_opt) != 0) {
        ar8030_link_close(&link);
        return 1;
    }
    fprintf(stderr, "rx: bb_socket open (port=%d)\n", args.port);

    int udp_fd = open_udp_target(args.target_host, args.target_port);
    if (udp_fd < 0) {
        ar8030_link_close(&link);
        return 1;
    }
    fprintf(stderr, "rx: forwarding RTP/H.265 to %s:%d\n", args.target_host, args.target_port);

    rtp_h265_ctx_t rtp_ctx;
    rtp_h265_init(&rtp_ctx, udp_fd, args.rtp_max_payload);

    ar8030_reassembly_t reasm;
    uint8_t *reasm_buf = malloc(args.reassembly_max);
    if (!reasm_buf) {
        fprintf(stderr, "rx: OOM allocating %u-byte reassembly buffer\n", args.reassembly_max);
        close(udp_fd);
        ar8030_link_close(&link);
        return 1;
    }
    ar8030_reassembly_init(&reasm, reasm_buf, args.reassembly_max);

    uint8_t *recvbuf = malloc(AR8030_CHUNK_HDR_SIZE + 65536);
    if (!recvbuf) {
        fprintf(stderr, "rx: OOM allocating receive buffer\n");
        free(reasm.buf);
        close(udp_fd);
        ar8030_link_close(&link);
        return 1;
    }
    uint32_t recvbuf_cap = AR8030_CHUNK_HDR_SIZE + 65536;

    while (!g_stop) {
        int n = bb_socket_read(link.sockfd, recvbuf, recvbuf_cap, 200);
        if (n < 0)
            continue; /* timeout or transient error; loop back and re-check g_stop */
        if ((uint32_t)n < AR8030_CHUNK_HDR_SIZE)
            continue; /* short read, not even a full header -- discard */

        struct ar8030_chunk_hdr hdr;
        memcpy(&hdr, recvbuf, AR8030_CHUNK_HDR_SIZE);
        if (hdr.magic != AR8030_CHUNK_MAGIC)
            continue; /* not our framing, or the link handed us garbage -- discard */

        const uint8_t *payload = recvbuf + AR8030_CHUNK_HDR_SIZE;
        uint32_t payload_len = (uint32_t)n - AR8030_CHUNK_HDR_SIZE;

        if (!ar8030_reassembly_feed(&reasm, &hdr, payload, payload_len))
            continue;

        uint32_t rtp_ts = (uint32_t)(((uint64_t)reasm.frame_pts * 90ull) / 1000ull);
        rtp_h265_send_frame(&rtp_ctx, reasm.buf, reasm.write_off, rtp_ts);
        ar8030_reassembly_reset(&reasm);
    }

    free(recvbuf);
    free(reasm_buf);
    close(udp_fd);
    ar8030_link_close(&link);
    return 0;
}
