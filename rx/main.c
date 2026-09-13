/*
 * ar8030-transport-rx -- ground-side bridge from the AR8030's non-IP
 * baseband data channel (bb_socket) to PixelPilot_rk.
 *
 * Reads ar8030_chunk_hdr-prefixed chunks (common/ar8030_chunk.h) off a
 * stream-mode bb_socket (no BB_SOCK_FLAG_DATAGRAM -- see README.md
 * "Stream mode, not datagram", matching the stock vendor streamer's own
 * bb_socket_open() calls) via common/chunk_stream.c, reassembles them
 * into whole Annex-B H.265 access units, RTP/H.265-packetizes each
 * (rx/rtp_h265.c) and sends the packets over UDP to PixelPilot's
 * existing RTP listener -- no PixelPilot_rk source changes; point a
 * stock `pixelpilot -p 5600` at this tool's -H/-p target.
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
#include "chunk_stream.h"
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
#include <time.h>
#include <unistd.h>

#define DEFAULT_DAEMON_IP "127.0.0.1"
#define DEFAULT_TARGET_HOST "127.0.0.1"
#define DEFAULT_TARGET_PORT 5600 /* PixelPilot_rk's own -p default */
#define DEFAULT_VIDEO_PORT 2     /* must match tx/main.c's -o, see its comment */
#define DEFAULT_MAX_RTP_PAYLOAD 1400
#define DEFAULT_REASSEMBLY_MAX (512 * 1024) /* venc_frame_ring.h: up to 512 KB slots (CV610) */
#define RETRY_MS 2000
/* Per bb_socket_read() call inside the chunk stream reader. Independent
 * of tx's (much longer) write timeout -- this one only needs to be short
 * enough that the main loop keeps checking g_stop promptly; it does not
 * bound how long reassembling one whole chunk can take (chunk_stream.c
 * just calls bb_socket_read() again if a chunk isn't complete yet). */
#define STREAM_READ_TIMEOUT_MS 200
/* Internal chunk_stream buffer, sized as a small multiple of one whole
 * chunk so a burst that arrives as several chunks concatenated in one
 * bb_socket_read() doesn't need extra round trips to drain -- see
 * chunk_stream.h's init() comment for the hard minimum (one chunk). */
#define STREAM_BUF_CHUNKS 4
#define STATS_INTERVAL_S 1.0
/* How often the main loop asks the daemon directly whether it's still
 * there (ar8030_link_is_alive(), a single BB_GET_STATUS round trip) --
 * see that function's own comment (common/ar8030_link.h) for why this
 * can't just be inferred from bb_socket_read() timeouts, which look
 * identical whether the RF link is merely idle/saturated or the daemon
 * itself crashed and restarted. */
#define DAEMON_HEALTH_CHECK_INTERVAL_S 3.0
/* See tx/main.c's own (much longer) comment on this pair of constants --
 * confirmed live there that reopening the data socket immediately after
 * a fresh reconnect can fail on the first attempt alone even though the
 * daemon's own log shows the open succeeding then instantly unwinding
 * again, well under any real timeout. Applying the same short retry
 * loop here for the same reason: nothing else in this reconnect
 * sequence assumes its first attempt succeeds either. */
#define SOCKET_REOPEN_MAX_ATTEMPTS 5
#define SOCKET_REOPEN_RETRY_MS 500

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
    int verbose;
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
            "  -v             print periodic in/out stats to stderr (chunks, resyncs, frames, "
            "bytes, RTP output)\n"
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
    a->verbose = 0;

    int opt;
    while ((opt = getopt(argc, argv, "d:o:H:p:M:b:vh")) != -1) {
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

/* ar8030_chunk_read_fn for ar8030_chunk_stream_read(): the real
 * bb_socket_read()-backed reader. ctx is the bb_socket fd, boxed as a
 * pointer so common/chunk_stream.c stays free of any AR8030 SDK
 * dependency (see its header comment) -- the fake used by
 * test/chunk_stream_test.c is the only other implementation of this. */
static int read_from_bb_socket(void *ctx, uint8_t *buf, uint32_t cap, int timeout_ms)
{
    int sockfd = *(int *)ctx;
    return bb_socket_read(sockfd, buf, cap, timeout_ms);
}

/* Snapshot of everything -v reports. Most of it is just a copy of
 * lifetime counters that already live on the stream reader and
 * reassembler (chunk_stream.c's chunks_read/bytes_consumed/
 * resync_dropped_bytes, reassembly.c's completed_frames/dropped_frames)
 * -- rtp_packets/rtp_bytes are the only counters rx/main.c itself owns,
 * since neither of those modules knows what happens after reassembly. */
struct rx_stats_snapshot {
    uint64_t chunks;
    uint64_t bytes;
    uint64_t resync_dropped;
    uint64_t checksum_fails;
    uint64_t complete;
    uint64_t dropped;
    uint64_t rtp_packets;
    uint64_t rtp_bytes;
};

static struct rx_stats_snapshot rx_stats_snapshot_take(const ar8030_chunk_stream_t *stream,
                                                        const ar8030_reassembly_t *reasm,
                                                        uint64_t rtp_packets, uint64_t rtp_bytes)
{
    struct rx_stats_snapshot s;
    s.chunks = stream->chunks_read;
    s.bytes = stream->bytes_consumed;
    s.resync_dropped = stream->resync_dropped_bytes;
    s.checksum_fails = stream->checksum_fails;
    s.complete = reasm->completed_frames;
    s.dropped = reasm->dropped_frames;
    s.rtp_packets = rtp_packets;
    s.rtp_bytes = rtp_bytes;
    return s;
}

static void print_rx_stats(const struct rx_stats_snapshot *cur, const struct rx_stats_snapshot *prev,
                            double dt_s)
{
    uint64_t d_chunks = cur->chunks - prev->chunks;
    uint64_t d_bytes = cur->bytes - prev->bytes;
    uint64_t d_resync = cur->resync_dropped - prev->resync_dropped;
    uint64_t d_cksum = cur->checksum_fails - prev->checksum_fails;
    uint64_t d_complete = cur->complete - prev->complete;
    uint64_t d_dropped = cur->dropped - prev->dropped;
    uint64_t d_pkts = cur->rtp_packets - prev->rtp_packets;
    uint64_t d_rtp_bytes = cur->rtp_bytes - prev->rtp_bytes;

    fprintf(stderr,
            "rx stats: chunks %.1f/s in (%.2f Mbit/s), resync drops %.1f B/s, checksum fails %.1f/s "
            "| frames %.1f/s complete, %.1f/s dropped | RTP %.1f pkt/s out (%.2f Mbit/s) | totals: "
            "chunks=%llu resync_dropped=%llu checksum_fails=%llu complete=%llu dropped=%llu "
            "rtp_pkts=%llu\n",
            d_chunks / dt_s, (d_bytes * 8.0) / (dt_s * 1e6), d_resync / dt_s, d_cksum / dt_s,
            d_complete / dt_s, d_dropped / dt_s, d_pkts / dt_s, (d_rtp_bytes * 8.0) / (dt_s * 1e6),
            (unsigned long long)cur->chunks, (unsigned long long)cur->resync_dropped,
            (unsigned long long)cur->checksum_fails, (unsigned long long)cur->complete,
            (unsigned long long)cur->dropped, (unsigned long long)cur->rtp_packets);
}

static double now_monotonic_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
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
     * than relying on the SDK's silent override.
     *
     * Force-close + retry: same as the reconnect path below (see
     * SOCKET_REOPEN_MAX_ATTEMPTS's own, longer comment on tx/main.c's
     * side) -- confirmed live on tx that this exact failure isn't
     * specific to reconnecting; it also hit a genuinely fresh startup
     * once a prior run had left the daemon in this state. */
    ar8030_link_force_close_all_sockets(&link);
    int startup_open_ret = -1;
    int startup_open_attempt;
    for (startup_open_attempt = 0; startup_open_attempt < SOCKET_REOPEN_MAX_ATTEMPTS && !g_stop;
         startup_open_attempt++) {
        startup_open_ret =
            ar8030_link_open_socket(&link, BB_SLOT_AP, (uint32_t)args.port, BB_SOCK_FLAG_RX, &sock_opt);
        if (startup_open_ret == 0)
            break;
        usleep(SOCKET_REOPEN_RETRY_MS * 1000);
    }
    if (startup_open_ret != 0) {
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

    uint32_t stream_buf_cap = STREAM_BUF_CHUNKS * (AR8030_CHUNK_HDR_SIZE + AR8030_CHUNK_MAX_PAYLOAD);
    uint8_t *stream_buf = malloc(stream_buf_cap);
    uint8_t *payload_buf = malloc(AR8030_CHUNK_MAX_PAYLOAD);
    if (!stream_buf || !payload_buf) {
        fprintf(stderr, "rx: OOM allocating stream buffers\n");
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

    uint64_t rtp_packets_sent = 0;
    uint64_t rtp_bytes_sent = 0;
    struct rx_stats_snapshot stats_prev = rx_stats_snapshot_take(&stream, &reasm, 0, 0);
    double stats_last_print = now_monotonic_s();
    double last_health_check_s = now_monotonic_s();

    while (!g_stop) {
        if (args.verbose) {
            double now = now_monotonic_s();
            if (now - stats_last_print >= STATS_INTERVAL_S) {
                struct rx_stats_snapshot cur =
                    rx_stats_snapshot_take(&stream, &reasm, rtp_packets_sent, rtp_bytes_sent);
                print_rx_stats(&cur, &stats_prev, now - stats_last_print);
                stats_prev = cur;
                stats_last_print = now;
            }
        }

        /* Periodic daemon-liveness probe, independent of whatever the
         * data path below is doing -- see DAEMON_HEALTH_CHECK_INTERVAL_S's
         * own comment. Simpler than tx's own version of this: no second
         * thread sharing `link`, and the ground side always targets the
         * fixed BB_SLOT_AP (no slot to re-resolve). Reopening the socket
         * updates link.sockfd in place, which read_from_bb_socket() (see
         * its own comment) already reads through a pointer on every call
         * -- ar8030_chunk_stream_read() picks up the new fd with no
         * further plumbing needed. */
        double now_health = now_monotonic_s();
        if (now_health - last_health_check_s >= DAEMON_HEALTH_CHECK_INTERVAL_S) {
            last_health_check_s = now_health;
            if (!ar8030_link_is_alive(&link)) {
                fprintf(stderr, "rx: ar8030d connection lost, reconnecting...\n");
                if (ar8030_link_reconnect_retry(&link, args.daemon_ip, args.daemon_port, RETRY_MS,
                                                 &g_stop) != 0)
                    break; /* stopped while waiting for the daemon to come back */
                fprintf(stderr, "rx: reconnected to ar8030d\n");

                /* Confirmed live on the tx side: a daemon killed abruptly
                 * never runs its own bb_socket_close() teardown, so the
                 * fresh daemon instance's first open on this same
                 * slot/port fails with ret=-1 ("already opened", stale
                 * chip-side state) even though nothing is genuinely still
                 * using it. The single-socket ar8030_link_force_close_
                 * socket() was tried first on tx and confirmed live NOT
                 * to clear it (returned -2); ar8030_link_force_close_
                 * all_sockets() is the one that actually worked -- see
                 * its own comment. Ignoring its return value is
                 * deliberate: on the common path there is nothing to
                 * force-close and this is a harmless no-op; the retry
                 * loop below is the real, checked, fatal-on-failure
                 * step. */
                ar8030_link_force_close_all_sockets(&link);

                int open_ret = -1;
                int attempt;
                for (attempt = 0; attempt < SOCKET_REOPEN_MAX_ATTEMPTS && !g_stop; attempt++) {
                    open_ret = ar8030_link_open_socket(&link, BB_SLOT_AP, (uint32_t)args.port,
                                                        BB_SOCK_FLAG_RX, &sock_opt);
                    if (open_ret == 0)
                        break;
                    usleep(SOCKET_REOPEN_RETRY_MS * 1000);
                }
                if (open_ret != 0) {
                    fprintf(stderr, "rx: failed to reopen bb_socket after reconnect (%d attempts), stopping\n",
                            attempt);
                    g_stop = 1; /* no second thread to hang here (unlike tx's bitrate_ctl), but
                                 * set for consistency and in case that ever changes. */
                    break;
                }
                fprintf(stderr, "rx: bb_socket open (port=%d)\n", args.port);
            }
        }

        struct ar8030_chunk_hdr hdr;
        uint32_t payload_len = 0;
        int ret = ar8030_chunk_stream_read(&stream, &hdr, payload_buf, AR8030_CHUNK_MAX_PAYLOAD,
                                            &payload_len);
        if (ret <= 0)
            continue; /* timeout, stop requested, or (ret<0) a misconfigured buffer -- either way,
                        * loop back and re-check g_stop */

        if (!ar8030_reassembly_feed(&reasm, &hdr, payload_buf, payload_len))
            continue;

        uint32_t rtp_ts = (uint32_t)(((uint64_t)reasm.frame_pts * 90ull) / 1000ull);
        int rtp_pkts = rtp_h265_send_frame(&rtp_ctx, reasm.buf, reasm.write_off, rtp_ts);
        if (rtp_pkts > 0) {
            rtp_packets_sent += (uint64_t)rtp_pkts;
            rtp_bytes_sent += reasm.write_off;
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
