/*
 * ar8030-transport-tx -- air-side bridge from waybeam's frame-shm ring to
 * the AR8030's non-IP baseband data channel (bb_socket).
 *
 * Reads whole encoded H.265 access units from waybeam's frame-shm ring
 * (see third_party/waybeam_frame_ring), splits each into fixed-size
 * chunks prefixed with an ar8030_chunk_hdr (common/ar8030_chunk.h), and
 * writes each chunk with bb_socket_write() on a stream-mode bb_socket
 * (no BB_SOCK_FLAG_DATAGRAM -- see README.md "Stream mode, not
 * datagram", matching the stock vendor streamer's own bb_socket_open()
 * calls). No FEC, no ACK: a lost chunk just makes its frame incomplete
 * on the ground side, which drops it and moves on (see rx/main.c).
 *
 * A second thread (tx/bitrate_ctl.c) watches the AR8030's own local TX
 * MCS/throughput and throttles waybeam's live bitrate over loopback HTTP
 * so the encoder doesn't outrun the radio.
 */

#include "ar8030_chunk.h"
#include "ar8030_link.h"
#include "bitrate_ctl.h"
#include "chunker.h"
#include "venc_frame_ring.h"

#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define DEFAULT_RING_NAME "venc_frames"
#define DEFAULT_DAEMON_IP "127.0.0.1"
#define DEFAULT_WAYBEAM_HOST "127.0.0.1"
#define DEFAULT_WAYBEAM_PORT 80
/* BB_CONFIG_MAX_TRANSPORT_PER_SLOT is 4 (ports 0..3). ar8030d reserves
 * ports 0 and 1 for the ar_net0 IP bridge on this project's devices --
 * confirmed on bench hardware that bb_socket_open() on either fails
 * (ret=-1) even with ar_net0 down/never started, so this tool defaults
 * to port 2. Override with -o if a deployment differs (check
 * `ar8030-status`'s BB_GET_SOCK_INFO output first: any port it lists is
 * already claimed by something). */
#define DEFAULT_VIDEO_PORT 2
/* -1, not BB_SLOT_0: air is AP role, and bb_socket_open()'s doc comment
 * ("Target SLOT. If DEV, target SLOT is BB_SLOT_AP") makes clear that on
 * the AP side the slot parameter targets a *specific connected DEV peer*,
 * not a fixed self-reference the way BB_SLOT_AP is for DEV (see rx/main.c's
 * own comment on that). A fixed default here silently opens the socket on
 * whatever slot number was guessed, which the actual peer may not occupy --
 * confirmed on real hardware pairing landing on slot 2 one boot and slot 0
 * another. -1 means "resolve from BB_GET_STATUS at startup" (see
 * resolve_connected_slot() in main()); -s still overrides it explicitly for
 * bench use without a live peer.
 */
#define DEFAULT_SLOT (-1)
#define RETRY_MS 2000
/* Per-write-attempt bb_socket_write() timeout. Used to match the stock
 * vendor streamer's own 1500ms value (reverse-engineered from
 * ar_ldyhs_sky's fpv_bb_video_stream_send) -- appropriate when the
 * vendor's own kernel-level write-wait gives up quickly too. Bumped past
 * that once 0012-sdio-write-wait-for-real-interrupt-not-100ms-poll.patch
 * made our own artosyn_sdio_write() wait far more patiently (up to
 * SDIO_WRITE_WAIT_MS, 2000ms) than it used to: with a *shorter* client
 * timeout than the kernel's own wait, this client gives up and moves on
 * before a write the kernel was still legitimately completing ever
 * finishes -- confirmed on real hardware via session_socket.c's own
 * "recv bad socket pack" log (a daemon reply arriving after this
 * client had already stopped waiting for it, opt=so_write, nobody left
 * in the wake_up list to consume it). Diagnostic/mitigation, not
 * necessarily the underlying fix: this doesn't explain *why*
 * artosyn_sdio_write() sometimes needs close to 2s in the first place,
 * just avoids this client giving up before it's done. Comfortably past
 * SDIO_WRITE_WAIT_MS so a normal-but-slow write always gets to finish
 * rather than getting orphaned this way; unlike an earlier datagram-mode
 * version of this tool, a write that only makes partial progress isn't a
 * failure either way -- it's just handed the remainder and retried (see
 * chunk_send_to_socket()) -- only a write that makes *zero* progress
 * within this timeout counts as a real failure. */
#define DEFAULT_WRITE_TIMEOUT_MS 2500
/* After this many back-to-back frames where not even the first chunk
 * got written (the strongest available signal that the link is
 * currently saturated, not just this one frame's bad luck), pause
 * briefly instead of immediately hammering it with the next frame's
 * chunks -- see the backoff comment in main()'s send loop. */
#define BACKOFF_AFTER_CONSECUTIVE_STALLS 5
#define BACKOFF_MS 50
#define STATS_INTERVAL_S 1.0

static volatile int g_stop;

static void on_signal(int sig)
{
    (void)sig;
    g_stop = 1;
}

struct tx_args {
    const char *ring_name;
    const char *daemon_ip;
    int daemon_port;
    int slot;
    int port;
    uint32_t chunk_payload;
    int write_timeout_ms;
    const char *waybeam_host;
    int waybeam_port;
    double margin;
    uint32_t min_kbps;
    uint32_t max_kbps;
    int verbose;
};

static void usage(const char *argv0)
{
    fprintf(stderr,
            "usage: %s [options]\n"
            "  -r <name>      frame-shm ring name (default %s)\n"
            "  -d <ip>        ar8030d daemon IP (default %s)\n"
            "  -s <slot>      AR8030 slot (default: auto-detect the connected DEV\n"
            "                 peer's slot from BB_GET_STATUS at startup)\n"
            "  -o <port>      AR8030 bb_socket logical port, 0-3 (default %d)\n"
            "  -c <bytes>     max chunk payload (default %u)\n"
            "  -t <ms>        per-chunk bb_socket_write ack-wait timeout (default %d)\n"
            "  -w <host>      waybeam HTTP host (default %s)\n"
            "  -P <port>      waybeam HTTP port (default %d)\n"
            "  -m <fraction>  bitrate margin applied to link throughput (default 0.70)\n"
            "  -n <kbps>      minimum bitrate floor (default 512)\n"
            "  -x <kbps>      maximum bitrate ceiling (default 20000)\n"
            "  -v             print periodic in/out stats to stderr (frames, chunks, bytes, "
            "failures, ring health)\n"
            "  -h             this help\n",
            argv0, DEFAULT_RING_NAME, DEFAULT_DAEMON_IP, DEFAULT_VIDEO_PORT,
            AR8030_CHUNK_DEFAULT_PAYLOAD, DEFAULT_WRITE_TIMEOUT_MS, DEFAULT_WAYBEAM_HOST,
            DEFAULT_WAYBEAM_PORT);
}

static int parse_args(int argc, char **argv, struct tx_args *a)
{
    a->ring_name = DEFAULT_RING_NAME;
    a->daemon_ip = DEFAULT_DAEMON_IP;
    a->daemon_port = BB_PORT_DEFAULT;
    a->slot = DEFAULT_SLOT;
    a->port = DEFAULT_VIDEO_PORT;
    a->chunk_payload = AR8030_CHUNK_DEFAULT_PAYLOAD;
    a->write_timeout_ms = DEFAULT_WRITE_TIMEOUT_MS;
    a->waybeam_host = DEFAULT_WAYBEAM_HOST;
    a->waybeam_port = DEFAULT_WAYBEAM_PORT;
    a->margin = 0.70;
    a->min_kbps = 512;
    a->max_kbps = 20000;
    a->verbose = 0;

    int opt;
    while ((opt = getopt(argc, argv, "r:d:s:o:c:t:w:P:m:n:x:vh")) != -1) {
        switch (opt) {
        case 'r':
            a->ring_name = optarg;
            break;
        case 'd':
            a->daemon_ip = optarg;
            break;
        case 's':
            a->slot = atoi(optarg);
            break;
        case 'o':
            a->port = atoi(optarg);
            break;
        case 'c':
            a->chunk_payload = (uint32_t)strtoul(optarg, NULL, 10);
            break;
        case 't':
            a->write_timeout_ms = atoi(optarg);
            break;
        case 'w':
            a->waybeam_host = optarg;
            break;
        case 'P':
            a->waybeam_port = atoi(optarg);
            break;
        case 'm':
            a->margin = strtod(optarg, NULL);
            break;
        case 'n':
            a->min_kbps = (uint32_t)strtoul(optarg, NULL, 10);
            break;
        case 'x':
            a->max_kbps = (uint32_t)strtoul(optarg, NULL, 10);
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

static void *bitrate_thread_main(void *arg)
{
    bitrate_ctl_run((const bitrate_ctl_cfg_t *)arg);
    return NULL;
}

/* -v stats, updated inline on the single send-loop thread (no locking
 * needed -- bitrate_ctl.c runs on its own thread but never touches
 * these). frames_in is every frame handed to ar8030_chunk_frame();
 * frames_complete/frames_incomplete classify what happened to it by
 * comparing the chunks actually sent against the chunks it needed (see
 * main()'s use of ar8030_chunk_frame()'s out_chunk_count). bytes_sent
 * counts wire bytes (header + payload) of chunks that were actually
 * written. */
struct tx_stats {
    uint64_t frames_in;
    uint64_t frames_complete;
    uint64_t frames_incomplete;
    uint64_t chunks_sent;
    uint64_t chunks_failed;
    uint64_t bytes_sent;
};

struct tx_send_ctx {
    ar8030_link_t *link;
    int write_timeout_ms;
    const volatile int *stop_flag;
    struct tx_stats *stats;
};

/* ar8030_chunk_send_fn for ar8030_chunk_frame(): writes one whole chunk,
 * retrying on partial progress rather than treating it as failure.
 *
 * In stream mode, bb_socket_write() returning less than the requested
 * length is not an error -- see session_socket.c's non-datagram path,
 * which lets a write make whatever progress the link currently has room
 * for and hands back that count. The stock vendor streamer's own send
 * loop (fpv_bb_video_stream_send, reverse-engineered from ar_ldyhs_sky)
 * does exactly this: keep calling bb_socket_write() with the remaining
 * bytes until the whole chunk is sent, and only treat a call that made
 * *zero* progress (ret <= 0) as a real failure. A half-sent frame is
 * still going to be dropped as incomplete on the ground side either way
 * once a chunk genuinely fails, so ar8030_chunk_frame() stopping early
 * there (rather than retrying the whole frame) remains the right call --
 * holding up the encode-thread-fed ring to fight a dead link is worse
 * than just moving on to the next frame. */
static int chunk_send_to_socket(void *ctx, const uint8_t *buf, uint32_t len)
{
    struct tx_send_ctx *sc = (struct tx_send_ctx *)ctx;
    uint32_t sent = 0;

    while (sent < len) {
        if (sc->stop_flag && *sc->stop_flag)
            return -1;

        int wr = bb_socket_write(sc->link->sockfd, buf + sent, len - sent, sc->write_timeout_ms);
        if (wr <= 0) {
            fprintf(stderr, "tx: bb_socket_write made no progress (len=%u, sent=%u, ret=%d)\n", len,
                    sent, wr);
            sc->stats->chunks_failed++;
            return -1;
        }
        sent += (uint32_t)wr;
    }
    sc->stats->chunks_sent++;
    sc->stats->bytes_sent += len;
    return 0;
}

static void print_tx_stats(const struct tx_stats *cur, const struct tx_stats *prev, double dt_s,
                            const venc_frame_ring_t *ring)
{
    uint64_t d_in = cur->frames_in - prev->frames_in;
    uint64_t d_ok = cur->frames_complete - prev->frames_complete;
    uint64_t d_bad = cur->frames_incomplete - prev->frames_incomplete;
    uint64_t d_chunks = cur->chunks_sent - prev->chunks_sent;
    uint64_t d_chunk_fail = cur->chunks_failed - prev->chunks_failed;
    uint64_t d_bytes = cur->bytes_sent - prev->bytes_sent;

    venc_frame_ring_fill_t fill;
    memset(&fill, 0, sizeof(fill));
    venc_frame_ring_get_fill(ring, &fill);
    /* fill.writes is venc_frame_ring_get_fill()'s r->stats.writes, a
     * per-attached-instance counter only a *producer* (venc_frame_ring_
     * begin_write()/commit_write()) increments -- this process only ever
     * calls venc_frame_ring_read_wait() as a consumer, so that field would
     * always read 0 here regardless of whether waybeam is producing
     * anything, which is actively misleading rather than merely useless.
     * ring->hdr->write_idx is the real, shared, producer-updated counter
     * (same field waybeam's own frame_shm_consumer_test.c reads for this
     * exact purpose) -- read it directly instead. */
    uint64_t producer_writes = __atomic_load_n(&ring->hdr->write_idx, __ATOMIC_ACQUIRE);

    fprintf(stderr,
            "tx stats: frames %.1f/s in, %.1f/s complete, %.1f/s incomplete | chunks %.1f/s sent, "
            "%.1f/s failed | %.2f Mbit/s | ring %u%% full (producer_writes=%llu our_reads=%llu "
            "full_drops=%llu other_drops=%llu) | totals: in=%llu complete=%llu incomplete=%llu "
            "bytes=%llu\n",
            d_in / dt_s, d_ok / dt_s, d_bad / dt_s, d_chunks / dt_s, d_chunk_fail / dt_s,
            (d_bytes * 8.0) / (dt_s * 1e6), fill.fill_pct, (unsigned long long)producer_writes,
            (unsigned long long)fill.reads, (unsigned long long)fill.full_drops,
            (unsigned long long)fill.other_drops, (unsigned long long)cur->frames_in,
            (unsigned long long)cur->frames_complete, (unsigned long long)cur->frames_incomplete,
            (unsigned long long)cur->bytes_sent);
}

static double now_monotonic_s(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

/* Scans BB_GET_STATUS for whichever slot is actually BB_LINK_STATE_CONNECT,
 * retrying every RETRY_MS until one is found or *stop_flag fires -- mirrors
 * ar8030-linkctl's resolve_connected_slot() and bb_pair's own
 * wait_for_paired_peer() slot-scan. Called before opening the data socket
 * (see DEFAULT_SLOT's comment on why a hardcoded slot is wrong here): this
 * tool otherwise starts writing within seconds of boot, often well before
 * ar8030-pair's own retry loop finishes -- confirmed on real hardware that
 * opening the socket on the wrong (or a not-yet-connected) slot leaves the
 * chip with nowhere to drain that data, which backs up the low-level SDIO
 * write queue until it times out permanently (needs a reboot to clear).
 * Blocks like the ring-attach/daemon-connect retry loops above it in
 * main() -- same reasoning: waybeam/ar8030d may come up before pairing
 * finishes, so this has to wait rather than fail outright. Returns the
 * connected slot, or -1 if *stop_flag fired first. */
static int resolve_connected_slot(bb_dev_handle_t *dev, const volatile int *stop_flag)
{
    int logged = 0;
    while (!*stop_flag) {
        bb_get_status_in_t st_in = { .user_bmp = 0xffff };
        bb_get_status_out_t st_out;
        memset(&st_out, 0, sizeof(st_out));
        if (bb_ioctl(dev, BB_GET_STATUS, &st_in, &st_out) == 0) {
            for (int s = 0; s < BB_SLOT_MAX; s++) {
                if (st_out.link_status[s].state == BB_LINK_STATE_CONNECT) {
                    return s;
                }
            }
        }
        if (!logged) {
            fprintf(stderr, "tx: waiting for a peer to reach CONNECT before opening the data socket...\n");
            logged = 1;
        }
        usleep(RETRY_MS * 1000);
    }
    return -1;
}

static uint8_t frame_flags_from_meta(const VencFrameMeta *meta)
{
    uint8_t flags = 0;
    if (meta->flags & VENC_FRAME_FLAG_IDR)
        flags |= AR8030_CHUNK_FLAG_IDR;
    if (meta->flags & VENC_FRAME_FLAG_GDR)
        flags |= AR8030_CHUNK_FLAG_GDR;
    if (meta->flags & VENC_FRAME_FLAG_ENHANCE)
        flags |= AR8030_CHUNK_FLAG_ENHANCE;
    return flags;
}

int main(int argc, char **argv)
{
    struct tx_args args;
    if (parse_args(argc, argv, &args) != 0)
        return 1;
    if (args.chunk_payload == 0 || args.chunk_payload > AR8030_CHUNK_MAX_PAYLOAD) {
        fprintf(stderr, "tx: -c must be 1-%u\n", AR8030_CHUNK_MAX_PAYLOAD);
        return 1;
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    fprintf(stderr, "tx: waiting for frame-shm ring '%s'...\n", args.ring_name);
    venc_frame_ring_t *ring = NULL;
    while (!g_stop && !(ring = venc_frame_ring_attach(args.ring_name)))
        usleep(RETRY_MS * 1000);
    if (!ring)
        return 0; /* stopped before waybeam ever came up */
    fprintf(stderr, "tx: attached to ring '%s'\n", args.ring_name);

    ar8030_link_t link;
    fprintf(stderr, "tx: connecting to ar8030d at %s:%d...\n", args.daemon_ip, args.daemon_port);
    if (ar8030_link_connect_retry(&link, args.daemon_ip, args.daemon_port, RETRY_MS, &g_stop) != 0) {
        venc_frame_ring_destroy(ring);
        return 0;
    }

    if (args.slot < 0) {
        args.slot = resolve_connected_slot(link.dev, &g_stop);
        if (args.slot < 0) {
            ar8030_link_close(&link);
            venc_frame_ring_destroy(ring);
            return 0; /* stopped before any peer ever connected */
        }
        fprintf(stderr, "tx: resolved connected slot %d\n", args.slot);
    }

    bb_sock_opt_t sock_opt;
    sock_opt.tx_buf_size = 64 * 1024;
    sock_opt.rx_buf_size = 1024;
    if (ar8030_link_open_socket(&link, (bb_slot_e)args.slot, (uint32_t)args.port, BB_SOCK_FLAG_TX,
                                 &sock_opt) != 0) {
        ar8030_link_close(&link);
        venc_frame_ring_destroy(ring);
        return 1;
    }
    fprintf(stderr, "tx: bb_socket open (slot=%d port=%d)\n", args.slot, args.port);

    bitrate_ctl_cfg_t bc_cfg;
    memset(&bc_cfg, 0, sizeof(bc_cfg));
    bc_cfg.link = &link;
    bc_cfg.slot = (bb_slot_e)args.slot;
    bc_cfg.waybeam_host = args.waybeam_host;
    bc_cfg.waybeam_port = args.waybeam_port;
    bc_cfg.margin = args.margin;
    bc_cfg.min_kbps = args.min_kbps;
    bc_cfg.max_kbps = args.max_kbps;
    bc_cfg.hysteresis = 0.05;
    bc_cfg.min_interval_ms = 1500;
    bc_cfg.poll_interval_ms = 2000;
    bc_cfg.ring = ring; /* already attached above; see bitrate_ctl.h's cfg->ring comment */
    bc_cfg.ring_backlog_high_slots = 2; /* venc_frame_ring.h: >=2 is standing backlog */
    bc_cfg.ring_backoff = 0.85;
    bc_cfg.stop_flag = &g_stop;

    pthread_t bc_thread;
    int bc_thread_ok = (pthread_create(&bc_thread, NULL, bitrate_thread_main, &bc_cfg) == 0);
    if (!bc_thread_ok)
        fprintf(stderr, "tx: failed to start bitrate control thread (continuing without it)\n");

    uint32_t ring_buf_size = ring->slot_data_size;
    uint8_t *ring_buf = malloc(ring_buf_size);
    if (!ring_buf) {
        fprintf(stderr, "tx: OOM allocating %u-byte ring read buffer\n", ring_buf_size);
        g_stop = 1;
    }

    /* One-time scratch buffer for ar8030_chunk_frame()'s header+payload
     * staging -- see chunker.h for why this moved out of an internal
     * fixed-size array (AR8030_CHUNK_MAX_PAYLOAD is now the wire
     * format's full 65535-byte ceiling, too big for a repeated on-stack
     * buffer on this RAM-constrained target). Sized to args.chunk_payload,
     * not the ceiling, so a smaller -c doesn't allocate more than it needs. */
    uint32_t chunk_scratch_size = AR8030_CHUNK_HDR_SIZE + args.chunk_payload;
    uint8_t *chunk_scratch = malloc(chunk_scratch_size);
    if (!chunk_scratch) {
        fprintf(stderr, "tx: OOM allocating %u-byte chunk scratch buffer\n", chunk_scratch_size);
        g_stop = 1;
    }

    struct tx_stats stats;
    memset(&stats, 0, sizeof(stats));
    struct tx_stats stats_prev = stats;
    double stats_last_print = now_monotonic_s();

    struct tx_send_ctx send_ctx = {
        .link = &link, .write_timeout_ms = args.write_timeout_ms, .stop_flag = &g_stop, .stats = &stats
    };
    /* Consecutive frames where not even the first chunk got written --
     * see BACKOFF_AFTER_CONSECUTIVE_STALLS. A frame that sent *some*
     * chunks before failing resets this: that is ordinary loss, not
     * sustained saturation. */
    int consecutive_stalls = 0;

    uint16_t frame_seq = 0;
    while (!g_stop) {
        if (args.verbose) {
            double now = now_monotonic_s();
            if (now - stats_last_print >= STATS_INTERVAL_S) {
                print_tx_stats(&stats, &stats_prev, now - stats_last_print, ring);
                stats_prev = stats;
                stats_last_print = now;
            }
        }

        uint32_t out_len = 0;
        int ret = venc_frame_ring_read_wait(ring, ring_buf, ring_buf_size, &out_len, 200);
        if (ret != 0)
            continue; /* timeout, loop back and re-check g_stop */
        if (out_len < VENC_FRAME_META_SIZE)
            continue; /* malformed slot; ring already accounts this via bad_slot_drops */

        VencFrameMeta meta;
        memcpy(&meta, ring_buf, VENC_FRAME_META_SIZE);
        const uint8_t *frame_data = ring_buf + VENC_FRAME_META_SIZE;
        uint32_t frame_len = out_len - VENC_FRAME_META_SIZE;

        stats.frames_in++;
        uint32_t expected_chunks = 0;
        int sent = ar8030_chunk_frame(frame_seq, frame_flags_from_meta(&meta), AR8030_CHUNK_CODEC_H265,
                                       meta.pts, frame_data, frame_len, args.chunk_payload,
                                       chunk_send_to_socket, &send_ctx, &expected_chunks, chunk_scratch,
                                       chunk_scratch_size);
        if (sent >= 0 && (uint32_t)sent == expected_chunks)
            stats.frames_complete++;
        else
            stats.frames_incomplete++;
        frame_seq++;

        if (sent == 0) {
            consecutive_stalls++;
            /* The link couldn't take even one chunk of several frames in
             * a row: it's saturated, not just unlucky on one write. Give
             * it a brief pause to drain instead of immediately queuing
             * another frame's worth of writes it also can't take --
             * confirmed on bench hardware that hammering a saturated
             * link this way just produces an unbroken run of timeouts
             * (see common/ar8030_chunk.h's default payload comment). */
            if (consecutive_stalls >= BACKOFF_AFTER_CONSECUTIVE_STALLS) {
                usleep(BACKOFF_MS * 1000);
                consecutive_stalls = 0;
            }
        } else {
            consecutive_stalls = 0;
        }
    }

    if (bc_thread_ok)
        pthread_join(bc_thread, NULL);

    free(chunk_scratch);
    free(ring_buf);
    ar8030_link_close(&link);
    venc_frame_ring_destroy(ring);
    return 0;
}
