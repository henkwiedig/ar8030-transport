/*
 * ar8030-transport-audio-rx -- ground-side bridge from the AR8030's
 * non-IP baseband data channel (bb_socket) back to a UDP RTP/Opus stream,
 * on a separate logical port from rx/main.c's video traffic.
 *
 * Unlike rx/main.c, there is no re-packetization here: each reassembled
 * frame is already a complete RTP/Opus packet (PT=98) exactly as
 * waybeam's cv610_audio.c built it, because audio_tx/main.c sent it
 * across verbatim (see that file's own header comment). This just
 * reassembles the chunks and forwards the resulting bytes on with one
 * send(), unmodified.
 *
 * Point this at the SAME -H/-p target rx/main.c is already forwarding
 * RTP/H.265 (PT=97) to. Both processes then send to the same UDP
 * destination, differing only in RTP payload type inside each packet --
 * a receiver bound to that one port (or PixelPilot itself, if it
 * demuxes by payload type) sees exactly what the stock Majestic
 * combined video+audio stream looked like, without touching rx/main.c
 * or its own bb_socket/reassembly state at all. See audio_tx/main.c's
 * header comment for why this runs as its own process with its own
 * ar8030d connection rather than a second thread inside rx/main.c.
 */

#include "ar8030_chunk.h"
#include "ar8030_link.h"
#include "chunk_stream.h"
#include "reassembly.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netdb.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#define DEFAULT_DAEMON_IP "127.0.0.1"
#define DEFAULT_TARGET_HOST "127.0.0.1"
#define DEFAULT_TARGET_PORT 5600 /* match rx/main.c's own -p / -H so both land on one UDP stream */
#define DEFAULT_BB_PORT 3        /* must match audio_tx/main.c's -o */
#define RETRY_MS 2000
#define STREAM_READ_TIMEOUT_MS 200
#define STREAM_BUF_CHUNKS 4
/* Opus/RTP packets are tiny (well under 200 bytes) and always fit in one
 * chunk -- see audio_tx/main.c. This only needs to be large enough for
 * that plus headroom, nowhere near rx/main.c's 512 KB video default. */
#define DEFAULT_REASSEMBLY_MAX (16 * 1024)
#define STATS_INTERVAL_S 1.0
#define DAEMON_HEALTH_CHECK_INTERVAL_S 3.0
#define SOCKET_REOPEN_MAX_ATTEMPTS 5
#define SOCKET_REOPEN_RETRY_MS 500

static volatile int g_stop;

static void on_signal(int sig)
{
    (void)sig;
    g_stop = 1;
}

struct audio_rx_args {
    const char *daemon_ip;
    int daemon_port;
    int bb_port;
    const char *target_host;
    int target_port;
    uint32_t reassembly_max;
    int verbose;
};

static void usage(const char *argv0)
{
    fprintf(stderr,
            "usage: %s [options]\n"
            "  -d <ip>        ar8030d daemon IP (default %s)\n"
            "  -o <port>      AR8030 bb_socket logical port for audio, 0-3 (default %d;\n"
            "                 must match audio_tx/main.c's -o)\n"
            "  -H <host>      UDP target host (default %s -- match rx/main.c's own -H so\n"
            "                 video and audio land on the same destination)\n"
            "  -p <port>      UDP target port (default %d -- match rx/main.c's own -p)\n"
            "  -b <bytes>     max reassembled packet size (default %d)\n"
            "  -v             print periodic in/out stats to stderr\n"
            "  -h             this help\n",
            argv0, DEFAULT_DAEMON_IP, DEFAULT_BB_PORT, DEFAULT_TARGET_HOST, DEFAULT_TARGET_PORT,
            DEFAULT_REASSEMBLY_MAX);
}

static int parse_args(int argc, char **argv, struct audio_rx_args *a)
{
    a->daemon_ip = DEFAULT_DAEMON_IP;
    a->daemon_port = BB_PORT_DEFAULT;
    a->bb_port = DEFAULT_BB_PORT;
    a->target_host = DEFAULT_TARGET_HOST;
    a->target_port = DEFAULT_TARGET_PORT;
    a->reassembly_max = DEFAULT_REASSEMBLY_MAX;
    a->verbose = 0;

    int opt;
    while ((opt = getopt(argc, argv, "d:o:H:p:b:vh")) != -1) {
        switch (opt) {
        case 'd':
            a->daemon_ip = optarg;
            break;
        case 'o':
            a->bb_port = atoi(optarg);
            break;
        case 'H':
            a->target_host = optarg;
            break;
        case 'p':
            a->target_port = atoi(optarg);
            break;
        case 'b':
            a->reassembly_max = (uint32_t)strtoul(optarg, NULL, 10);
            break;
        case 'v':
            a->verbose = 1;
            break;
        case 'h':
        default:
            usage(argv[0]);
            return -1;
        }
    }
    return 0;
}

/* Same as rx/main.c's own open_udp_target() -- connect()-ed UDP socket so
 * the send loop below can use plain send() rather than carrying a
 * sockaddr around. Not shared code for the same reason resolve_connected_
 * slot() isn't in audio_tx/main.c: small, and tied up with this file's
 * own error reporting. */
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
        fprintf(stderr, "audio_rx: getaddrinfo(%s:%d) failed\n", host, port);
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
        fprintf(stderr, "audio_rx: failed to open UDP socket to %s:%d\n", host, port);
    return fd;
}

static int read_from_bb_socket(void *ctx, uint8_t *buf, uint32_t cap, int timeout_ms)
{
    int sockfd = *(int *)ctx;
    return bb_socket_read(sockfd, buf, cap, timeout_ms);
}

struct rx_stats_snapshot {
    uint64_t chunks;
    uint64_t bytes;
    uint64_t resync_dropped;
    uint64_t checksum_fails;
    uint64_t complete;
    uint64_t dropped;
    uint64_t udp_packets;
    uint64_t udp_bytes;
};

static struct rx_stats_snapshot stats_take(const ar8030_chunk_stream_t *stream,
                                            const ar8030_reassembly_t *reasm, uint64_t udp_packets,
                                            uint64_t udp_bytes)
{
    struct rx_stats_snapshot s;
    s.chunks = stream->chunks_read;
    s.bytes = stream->bytes_consumed;
    s.resync_dropped = stream->resync_dropped_bytes;
    s.checksum_fails = stream->checksum_fails;
    s.complete = reasm->completed_frames;
    s.dropped = reasm->dropped_frames;
    s.udp_packets = udp_packets;
    s.udp_bytes = udp_bytes;
    return s;
}

static void print_stats(const struct rx_stats_snapshot *cur, const struct rx_stats_snapshot *prev,
                         double dt_s)
{
    uint64_t d_chunks = cur->chunks - prev->chunks;
    uint64_t d_resync = cur->resync_dropped - prev->resync_dropped;
    uint64_t d_cksum = cur->checksum_fails - prev->checksum_fails;
    uint64_t d_complete = cur->complete - prev->complete;
    uint64_t d_dropped = cur->dropped - prev->dropped;
    uint64_t d_pkts = cur->udp_packets - prev->udp_packets;
    uint64_t d_bytes = cur->udp_bytes - prev->udp_bytes;

    fprintf(stderr,
            "audio_rx stats: chunks %.1f/s in, resync drops %.1f B/s, checksum fails %.1f/s | "
            "packets %.1f/s complete, %.1f/s dropped | UDP %.1f pkt/s out (%.2f kbit/s) | "
            "totals: complete=%llu dropped=%llu udp_pkts=%llu\n",
            d_chunks / dt_s, d_resync / dt_s, d_cksum / dt_s, d_complete / dt_s, d_dropped / dt_s,
            d_pkts / dt_s, (d_bytes * 8.0) / (dt_s * 1e3), (unsigned long long)cur->complete,
            (unsigned long long)cur->dropped, (unsigned long long)cur->udp_packets);
}

static double now_monotonic_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

int main(int argc, char **argv)
{
    struct audio_rx_args args;
    if (parse_args(argc, argv, &args) != 0)
        return 1;

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    ar8030_link_t link;
    fprintf(stderr, "audio_rx: connecting to ar8030d at %s:%d...\n", args.daemon_ip,
            args.daemon_port);
    if (ar8030_link_connect_retry(&link, args.daemon_ip, args.daemon_port, RETRY_MS, &g_stop) != 0)
        return 0;

    /* TX|RX, not RX-only, for the same reason as rx/main.c's video socket:
     * the chip doesn't count an RX-only socket's bytes on the ground
     * (BB_GET_SOCK_INFO total_size stays 0). Nothing is ever written on
     * the TX half. */
    bb_sock_opt_t sock_opt;
    sock_opt.tx_buf_size = 1024;
    sock_opt.rx_buf_size = 8 * 1024;
    const uint32_t sock_flags = BB_SOCK_FLAG_TX | BB_SOCK_FLAG_RX;
    /* Ground is DEV role: same BB_SLOT_AP fixed target as rx/main.c -- see
     * that file's own comment on why the slot parameter is ignored by the
     * SDK on this side.
     *
     * Scoped to this port only, never ar8030_link_force_close_all_sockets()
     * -- see audio_tx/main.c's header comment: video's rx/main.c owns a
     * concurrently-open bb_socket on a DIFFERENT port of the same device,
     * and BB_FORCE_CLS_SOCKET_ALL closes every port, not just this one --
     * confirmed live to stall the video stream the instant this process
     * started. */
    ar8030_link_force_close_socket(&link, BB_SLOT_AP, (uint32_t)args.bb_port);
    int open_ret = -1;
    for (int attempt = 0; attempt < SOCKET_REOPEN_MAX_ATTEMPTS && !g_stop; attempt++) {
        open_ret = ar8030_link_open_socket(&link, BB_SLOT_AP, (uint32_t)args.bb_port,
                                            sock_flags, &sock_opt);
        if (open_ret == 0)
            break;
        usleep(SOCKET_REOPEN_RETRY_MS * 1000);
    }
    if (open_ret != 0) {
        ar8030_link_close(&link);
        return 1;
    }
    fprintf(stderr, "audio_rx: bb_socket open (port=%d)\n", args.bb_port);

    int udp_fd = open_udp_target(args.target_host, args.target_port);
    if (udp_fd < 0) {
        ar8030_link_close(&link);
        return 1;
    }
    fprintf(stderr, "audio_rx: forwarding RTP/Opus to %s:%d\n", args.target_host, args.target_port);

    ar8030_reassembly_t reasm;
    uint8_t *reasm_buf = malloc(args.reassembly_max);
    if (!reasm_buf) {
        fprintf(stderr, "audio_rx: OOM allocating %u-byte reassembly buffer\n", args.reassembly_max);
        close(udp_fd);
        ar8030_link_close(&link);
        return 1;
    }
    ar8030_reassembly_init(&reasm, reasm_buf, args.reassembly_max);

    uint32_t stream_buf_cap = STREAM_BUF_CHUNKS * (AR8030_CHUNK_HDR_SIZE + AR8030_CHUNK_MAX_PAYLOAD);
    uint8_t *stream_buf = malloc(stream_buf_cap);
    uint8_t *payload_buf = malloc(AR8030_CHUNK_MAX_PAYLOAD);
    if (!stream_buf || !payload_buf) {
        fprintf(stderr, "audio_rx: OOM allocating stream buffers\n");
        free(stream_buf);
        free(payload_buf);
        free(reasm_buf);
        close(udp_fd);
        ar8030_link_close(&link);
        return 1;
    }
    ar8030_chunk_stream_t stream;
    ar8030_chunk_stream_init(&stream, read_from_bb_socket, &link.sockfd, stream_buf, stream_buf_cap,
                              STREAM_READ_TIMEOUT_MS, &g_stop);

    uint64_t udp_packets_sent = 0;
    uint64_t udp_bytes_sent = 0;
    struct rx_stats_snapshot stats_prev = stats_take(&stream, &reasm, 0, 0);
    double stats_last_print = now_monotonic_s();
    double last_health_check_s = now_monotonic_s();

    while (!g_stop) {
        if (args.verbose) {
            double now = now_monotonic_s();
            if (now - stats_last_print >= STATS_INTERVAL_S) {
                struct rx_stats_snapshot cur =
                    stats_take(&stream, &reasm, udp_packets_sent, udp_bytes_sent);
                print_stats(&cur, &stats_prev, now - stats_last_print);
                stats_prev = cur;
                stats_last_print = now;
            }
        }

        double now_health = now_monotonic_s();
        if (now_health - last_health_check_s >= DAEMON_HEALTH_CHECK_INTERVAL_S) {
            last_health_check_s = now_health;
            if (!ar8030_link_is_alive(&link)) {
                fprintf(stderr, "audio_rx: ar8030d connection lost, reconnecting...\n");
                if (ar8030_link_reconnect_retry(&link, args.daemon_ip, args.daemon_port, RETRY_MS,
                                                 &g_stop) != 0)
                    break;
                fprintf(stderr, "audio_rx: reconnected to ar8030d\n");

                /* Scoped, not "all" -- see the startup call's own comment above. */
                ar8030_link_force_close_socket(&link, BB_SLOT_AP, (uint32_t)args.bb_port);
                int attempt, ret2 = -1;
                for (attempt = 0; attempt < SOCKET_REOPEN_MAX_ATTEMPTS && !g_stop; attempt++) {
                    ret2 = ar8030_link_open_socket(&link, BB_SLOT_AP, (uint32_t)args.bb_port,
                                                    sock_flags, &sock_opt);
                    if (ret2 == 0)
                        break;
                    usleep(SOCKET_REOPEN_RETRY_MS * 1000);
                }
                if (ret2 != 0) {
                    fprintf(stderr,
                            "audio_rx: failed to reopen bb_socket after reconnect (%d attempts), "
                            "stopping\n",
                            attempt);
                    break;
                }
                fprintf(stderr, "audio_rx: bb_socket open (port=%d)\n", args.bb_port);
            }
        }

        struct ar8030_chunk_hdr hdr;
        uint32_t payload_len = 0;
        int ret = ar8030_chunk_stream_read(&stream, &hdr, payload_buf, AR8030_CHUNK_MAX_PAYLOAD,
                                            &payload_len);
        if (ret <= 0)
            continue;

        if (!ar8030_reassembly_feed(&reasm, &hdr, payload_buf, payload_len))
            continue;

        /* No RTP re-packetization: reasm.buf[0..write_off) is already a
         * complete RTP/Opus packet exactly as venc built it -- see this
         * file's header comment. */
        ssize_t sent = send(udp_fd, reasm.buf, reasm.write_off, 0);
        if (sent > 0) {
            udp_packets_sent++;
            udp_bytes_sent += (uint64_t)sent;
        }
        ar8030_reassembly_reset(&reasm);
    }

    free(stream_buf);
    free(payload_buf);
    free(reasm_buf);
    close(udp_fd);
    ar8030_link_close(&link);
    return 0;
}
