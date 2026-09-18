/*
 * ar8030-transport-audio-tx -- air-side bridge from waybeam's loopback
 * audio UDP output (venc's cv610_audio.c, "Waybeam Link loopback UDP side
 * channel" -- see waybeam_venc/documentation/AUDIO_UDP_OUTPUT_FEASIBILITY.md
 * and cv610_validation.c) to the AR8030's non-IP baseband data channel
 * (bb_socket), on a SEPARATE logical port from tx/main.c's video traffic.
 *
 * Unlike tx/main.c, there is no framing to understand here: waybeam
 * already emits complete RTP/Opus packets (PT=98, see cv610_audio.c) to
 * 127.0.0.1:<audioPort>. This process just binds that port, and forwards
 * each whole datagram over the air as one ar8030_chunk_hdr-prefixed chunk
 * (common/ar8030_chunk.h, codec=AR8030_CHUNK_CODEC_OPUS) -- see
 * audio_rx/main.c for the ground side, which forwards the reassembled
 * bytes on unmodified rather than re-packetizing them.
 *
 * Runs as its own process with its own ar8030d connection, deliberately
 * NOT threaded into tx/main.c: that file's reconnect/backoff state machine
 * is already carrying a lot of hard-won, hardware-verified subtlety (see
 * its own comments) for a single video bb_socket, and entangling a second
 * socket's lifecycle into it risks the video path to fix an audio path.
 * bb_socket_open() taking its own dev handle per port (see this project's
 * README "Stream mode, not datagram" -- the vendor's own streamer opens
 * port=3 video and port=2 audio the same way, concurrently, on one dev)
 * means two independent processes each with their own daemon connection is
 * just as valid as sharing one, and far simpler to get right.
 */

#include "ar8030_chunk.h"
#include "ar8030_link.h"
#include "chunker.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netdb.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#define DEFAULT_DAEMON_IP "127.0.0.1"
#define DEFAULT_BIND_HOST "127.0.0.1"
/* Matches waybeam's own outgoing.audioPort default (venc_config.c) and
 * cv610_audio.c's loopback destination for unix:// / frame-shm:// video
 * output -- this process is that loopback's listener. */
#define DEFAULT_AUDIO_UDP_PORT 5601
/* See tx/main.c's own DEFAULT_VIDEO_PORT comment: ports 0/1 are reserved
 * for the ar_net0 IP bridge on this project's devices, and tx/main.c's
 * video already defaults to port 2. Port 3 is the other free slot. */
#define DEFAULT_BB_PORT 3
#define DEFAULT_SLOT (-1) /* auto-resolve, same reasoning as tx/main.c */
#define RETRY_MS 2000
/* Opus/RTP datagrams are tiny (well under 200 bytes at 32 kbit/s, 20 ms
 * frames) and each fits in a single chunk however small -c is left at, so
 * there's no equivalent of tx/main.c's 1500-2500ms video write budget to
 * tune here -- a write that can't make progress within a fraction of one
 * audio frame period is already well past useful for a live link. */
#define DEFAULT_WRITE_TIMEOUT_MS 500
#define STATS_INTERVAL_S 1.0
#define DAEMON_HEALTH_CHECK_INTERVAL_S 3.0
#define SOCKET_REOPEN_MAX_ATTEMPTS 5
#define SOCKET_REOPEN_RETRY_MS 500
/* UDP receive timeout -- short enough to keep checking g_stop/health
 * promptly, same role as rx/main.c's STREAM_READ_TIMEOUT_MS. */
#define UDP_RECV_TIMEOUT_MS 200
/* Ceiling for one recvfrom(): generous headroom over a real Opus/RTP
 * packet so a misconfigured upstream (e.g. PCM instead of Opus) doesn't
 * silently truncate rather than being visibly oversized. Still comfortably
 * inside AR8030_CHUNK_MAX_PAYLOAD. */
#define MAX_UDP_PACKET 4096

static volatile int g_stop;

static void on_signal(int sig)
{
    (void)sig;
    g_stop = 1;
}

struct audio_tx_args {
    const char *daemon_ip;
    int daemon_port;
    int slot;
    int bb_port;
    const char *bind_host;
    int udp_port;
    uint32_t chunk_payload;
    int write_timeout_ms;
    int verbose;
};

static void usage(const char *argv0)
{
    fprintf(stderr,
            "usage: %s [options]\n"
            "  -d <ip>        ar8030d daemon IP (default %s)\n"
            "  -s <slot>      AR8030 slot (default: auto-detect the connected DEV peer's\n"
            "                 slot from BB_GET_STATUS at startup, same as tx/main.c)\n"
            "  -o <port>      AR8030 bb_socket logical port for audio, 0-3 (default %d;\n"
            "                 must differ from the video tx's -o and match audio_rx's -o)\n"
            "  -U <host>      local UDP bind host to receive waybeam's audio from (default %s)\n"
            "  -u <port>      local UDP port to receive waybeam's audio from (default %d,\n"
            "                 matches waybeam's outgoing.audioPort default)\n"
            "  -c <bytes>     max chunk payload (default %u)\n"
            "  -t <ms>        per-chunk bb_socket_write ack-wait timeout (default %d)\n"
            "  -v             print periodic in/out stats to stderr\n"
            "  -h             this help\n",
            argv0, DEFAULT_DAEMON_IP, DEFAULT_BB_PORT, DEFAULT_BIND_HOST, DEFAULT_AUDIO_UDP_PORT,
            AR8030_CHUNK_DEFAULT_PAYLOAD, DEFAULT_WRITE_TIMEOUT_MS);
}

static int parse_args(int argc, char **argv, struct audio_tx_args *a)
{
    a->daemon_ip = DEFAULT_DAEMON_IP;
    a->daemon_port = BB_PORT_DEFAULT;
    a->slot = DEFAULT_SLOT;
    a->bb_port = DEFAULT_BB_PORT;
    a->bind_host = DEFAULT_BIND_HOST;
    a->udp_port = DEFAULT_AUDIO_UDP_PORT;
    a->chunk_payload = AR8030_CHUNK_DEFAULT_PAYLOAD;
    a->write_timeout_ms = DEFAULT_WRITE_TIMEOUT_MS;
    a->verbose = 0;

    int opt;
    while ((opt = getopt(argc, argv, "d:s:o:U:u:c:t:vh")) != -1) {
        switch (opt) {
        case 'd':
            a->daemon_ip = optarg;
            break;
        case 's':
            a->slot = atoi(optarg);
            break;
        case 'o':
            a->bb_port = atoi(optarg);
            break;
        case 'U':
            a->bind_host = optarg;
            break;
        case 'u':
            a->udp_port = atoi(optarg);
            break;
        case 'c':
            a->chunk_payload = (uint32_t)strtoul(optarg, NULL, 10);
            break;
        case 't':
            a->write_timeout_ms = atoi(optarg);
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

struct audio_tx_stats {
    uint64_t packets_in;
    uint64_t frames_complete;
    uint64_t frames_incomplete;
    uint64_t chunks_sent;
    uint64_t chunks_failed;
    uint64_t bytes_sent;
};

struct audio_send_ctx {
    int sockfd;
    int write_timeout_ms;
    const volatile int *stop_flag;
    struct audio_tx_stats *stats;
};

/* Same shape as tx/main.c's chunk_send_to_socket() -- see that function's
 * own (much longer) comment for why a partial bb_socket_write() isn't a
 * failure, only a zero-progress one is. */
static int chunk_send_to_socket(void *ctx, const uint8_t *buf, uint32_t len)
{
    struct audio_send_ctx *sc = (struct audio_send_ctx *)ctx;
    uint32_t sent = 0;

    while (sent < len) {
        if (sc->stop_flag && *sc->stop_flag)
            return -1;

        int wr = bb_socket_write(sc->sockfd, buf + sent, len - sent, sc->write_timeout_ms);
        if (wr <= 0) {
            fprintf(stderr, "audio_tx: bb_socket_write made no progress (len=%u, sent=%u, ret=%d)\n",
                    len, sent, wr);
            sc->stats->chunks_failed++;
            return -1;
        }
        sent += (uint32_t)wr;
    }
    sc->stats->chunks_sent++;
    sc->stats->bytes_sent += len;
    return 0;
}

static void print_stats(const struct audio_tx_stats *cur, const struct audio_tx_stats *prev, double dt_s)
{
    uint64_t d_in = cur->packets_in - prev->packets_in;
    uint64_t d_ok = cur->frames_complete - prev->frames_complete;
    uint64_t d_bad = cur->frames_incomplete - prev->frames_incomplete;
    uint64_t d_bytes = cur->bytes_sent - prev->bytes_sent;

    fprintf(stderr,
            "audio_tx stats: %.1f pkt/s in, %.1f/s complete, %.1f/s incomplete | %.2f kbit/s | "
            "totals: in=%llu complete=%llu incomplete=%llu bytes=%llu\n",
            d_in / dt_s, d_ok / dt_s, d_bad / dt_s, (d_bytes * 8.0) / (dt_s * 1e3),
            (unsigned long long)cur->packets_in, (unsigned long long)cur->frames_complete,
            (unsigned long long)cur->frames_incomplete, (unsigned long long)cur->bytes_sent);
}

static double now_monotonic_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

/* Identical in spirit to tx/main.c's resolve_connected_slot() -- see its
 * own comment for why this has to block rather than fail outright (this
 * tool may start well before ar8030-pair's own retry loop finishes). Not
 * shared code: it is ~15 lines and tx/main.c's copy is bound up with that
 * file's own bb_dev_handle_t plumbing in a way not worth extracting a
 * common helper for two call sites. */
static int resolve_connected_slot(bb_dev_handle_t *dev, const volatile int *stop_flag)
{
    int logged = 0;
    while (!*stop_flag) {
        bb_get_status_in_t st_in = { .user_bmp = 0xffff };
        bb_get_status_out_t st_out;
        memset(&st_out, 0, sizeof(st_out));
        if (bb_ioctl(dev, BB_GET_STATUS, &st_in, &st_out) == 0) {
            for (int s = 0; s < BB_SLOT_MAX; s++) {
                if (st_out.link_status[s].state == BB_LINK_STATE_CONNECT)
                    return s;
            }
        }
        if (!logged) {
            fprintf(stderr, "audio_tx: waiting for a peer to reach CONNECT before opening the "
                             "data socket...\n");
            logged = 1;
        }
        usleep(RETRY_MS * 1000);
    }
    return -1;
}

static int open_udp_source(const char *host, int port)
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        fprintf(stderr, "audio_tx: socket(): %s\n", strerror(errno));
        return -1;
    }

    struct timeval tv = { .tv_sec = UDP_RECV_TIMEOUT_MS / 1000,
                           .tv_usec = (UDP_RECV_TIMEOUT_MS % 1000) * 1000 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons((uint16_t)port);
    if (inet_pton(AF_INET, host, &addr.sin_addr) != 1) {
        fprintf(stderr, "audio_tx: bad bind host '%s'\n", host);
        close(fd);
        return -1;
    }
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        fprintf(stderr, "audio_tx: bind(%s:%d): %s\n", host, port, strerror(errno));
        close(fd);
        return -1;
    }
    return fd;
}

int main(int argc, char **argv)
{
    struct audio_tx_args args;
    if (parse_args(argc, argv, &args) != 0)
        return 1;
    if (args.chunk_payload == 0 || args.chunk_payload > AR8030_CHUNK_MAX_PAYLOAD) {
        fprintf(stderr, "audio_tx: -c must be 1-%u\n", AR8030_CHUNK_MAX_PAYLOAD);
        return 1;
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    int udp_fd = open_udp_source(args.bind_host, args.udp_port);
    if (udp_fd < 0)
        return 1;
    fprintf(stderr, "audio_tx: listening for waybeam audio on %s:%d\n", args.bind_host,
            args.udp_port);

    ar8030_link_t link;
    fprintf(stderr, "audio_tx: connecting to ar8030d at %s:%d...\n", args.daemon_ip,
            args.daemon_port);
    if (ar8030_link_connect_retry(&link, args.daemon_ip, args.daemon_port, RETRY_MS, &g_stop) != 0) {
        close(udp_fd);
        return 0;
    }

    int slot_was_auto = (args.slot < 0);
    if (args.slot < 0) {
        args.slot = resolve_connected_slot(link.dev, &g_stop);
        if (args.slot < 0) {
            ar8030_link_close(&link);
            close(udp_fd);
            return 0;
        }
        fprintf(stderr, "audio_tx: resolved connected slot %d\n", args.slot);
    }

    bb_sock_opt_t sock_opt;
    sock_opt.tx_buf_size = 8 * 1024;
    sock_opt.rx_buf_size = 1024;
    /* Scoped to this port only -- see this file's own header comment on why
     * ar8030_link_force_close_all_sockets() must never be used here: video's
     * tx/main.c owns a concurrently-open bb_socket on a DIFFERENT port of
     * the SAME device, and the "all" variant (BB_FORCE_CLS_SOCKET_ALL) closes
     * every port, not just this one -- confirmed live to stall the video
     * stream the instant this process started. */
    ar8030_link_force_close_socket(&link, (bb_slot_e)args.slot, (uint32_t)args.bb_port);
    int open_ret = -1;
    for (int attempt = 0; attempt < SOCKET_REOPEN_MAX_ATTEMPTS && !g_stop; attempt++) {
        open_ret = ar8030_link_open_socket(&link, (bb_slot_e)args.slot, (uint32_t)args.bb_port,
                                            BB_SOCK_FLAG_TX, &sock_opt);
        if (open_ret == 0)
            break;
        usleep(SOCKET_REOPEN_RETRY_MS * 1000);
    }
    if (open_ret != 0) {
        ar8030_link_close(&link);
        close(udp_fd);
        return 1;
    }
    fprintf(stderr, "audio_tx: bb_socket open (slot=%d port=%d)\n", args.slot, args.bb_port);

    uint32_t scratch_size = AR8030_CHUNK_HDR_SIZE + args.chunk_payload;
    uint8_t *scratch = malloc(scratch_size);
    uint8_t *udp_buf = malloc(MAX_UDP_PACKET);
    if (!scratch || !udp_buf) {
        fprintf(stderr, "audio_tx: OOM allocating buffers\n");
        free(scratch);
        free(udp_buf);
        ar8030_link_close(&link);
        close(udp_fd);
        return 1;
    }

    struct audio_tx_stats stats;
    memset(&stats, 0, sizeof(stats));
    struct audio_tx_stats stats_prev = stats;
    double stats_last_print = now_monotonic_s();
    double last_health_check_s = now_monotonic_s();

    struct audio_send_ctx send_ctx = {
        .sockfd = link.sockfd, .write_timeout_ms = args.write_timeout_ms, .stop_flag = &g_stop,
        .stats = &stats
    };

    uint16_t frame_seq = 0;
    while (!g_stop) {
        if (args.verbose) {
            double now = now_monotonic_s();
            if (now - stats_last_print >= STATS_INTERVAL_S) {
                print_stats(&stats, &stats_prev, now - stats_last_print);
                stats_prev = stats;
                stats_last_print = now;
            }
        }

        double now_health = now_monotonic_s();
        if (now_health - last_health_check_s >= DAEMON_HEALTH_CHECK_INTERVAL_S) {
            last_health_check_s = now_health;
            if (!ar8030_link_is_alive(&link)) {
                fprintf(stderr, "audio_tx: ar8030d connection lost, reconnecting...\n");
                if (ar8030_link_reconnect_retry(&link, args.daemon_ip, args.daemon_port, RETRY_MS,
                                                 &g_stop) != 0)
                    break;
                fprintf(stderr, "audio_tx: reconnected to ar8030d\n");

                int slot = args.slot;
                if (slot_was_auto) {
                    bb_dev_handle_t *dev = __atomic_load_n(&link.dev, __ATOMIC_ACQUIRE);
                    slot = resolve_connected_slot(dev, &g_stop);
                    if (slot < 0)
                        break;
                    fprintf(stderr, "audio_tx: resolved connected slot %d\n", slot);
                }
                args.slot = slot;

                /* Scoped, not "all" -- see the startup call's own comment above. */
                ar8030_link_force_close_socket(&link, (bb_slot_e)slot, (uint32_t)args.bb_port);
                int attempt, ret2 = -1;
                for (attempt = 0; attempt < SOCKET_REOPEN_MAX_ATTEMPTS && !g_stop; attempt++) {
                    ret2 = ar8030_link_open_socket(&link, (bb_slot_e)slot, (uint32_t)args.bb_port,
                                                    BB_SOCK_FLAG_TX, &sock_opt);
                    if (ret2 == 0)
                        break;
                    usleep(SOCKET_REOPEN_RETRY_MS * 1000);
                }
                if (ret2 != 0) {
                    fprintf(stderr,
                            "audio_tx: failed to reopen bb_socket after reconnect (%d attempts), "
                            "stopping\n",
                            attempt);
                    break;
                }
                send_ctx.sockfd = link.sockfd;
                fprintf(stderr, "audio_tx: bb_socket open (slot=%d port=%d)\n", slot, args.bb_port);
            }
        }

        ssize_t n = recvfrom(udp_fd, udp_buf, MAX_UDP_PACKET, 0, NULL, NULL);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
                continue; /* recv timeout -- loop back to re-check g_stop/health */
            fprintf(stderr, "audio_tx: recvfrom: %s\n", strerror(errno));
            continue;
        }
        if (n == 0)
            continue;

        stats.packets_in++;
        uint32_t expected_chunks = 0;
        int sent = ar8030_chunk_frame(frame_seq, 0, AR8030_CHUNK_CODEC_OPUS, 0, udp_buf,
                                       (uint32_t)n, args.chunk_payload, chunk_send_to_socket,
                                       &send_ctx, &expected_chunks, scratch, scratch_size);
        if (sent >= 0 && (uint32_t)sent == expected_chunks)
            stats.frames_complete++;
        else
            stats.frames_incomplete++;
        frame_seq++;
    }

    free(scratch);
    free(udp_buf);
    ar8030_link_close(&link);
    close(udp_fd);
    return 0;
}
