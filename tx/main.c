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
#include "idr_ctrl.h"
#include "chunker.h"
#include "venc_frame_ring.h"

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#define DEFAULT_RING_NAME "venc_frames"
#define DEFAULT_DAEMON_IP "127.0.0.1"
#define DEFAULT_WAYBEAM_HOST "127.0.0.1"
#define DEFAULT_WAYBEAM_PORT 80
#define DEFAULT_IDR_COALESCE_MS 250 /* see idr_ctrl.h */
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
/* How often the main loop asks the daemon directly whether it's still
 * there (ar8030_link_is_alive(), a single BB_GET_STATUS round trip) --
 * see that function's own comment for why this can't just be inferred
 * from bb_socket_write() timeouts. A few seconds is frequent enough to
 * notice a daemon restart promptly without adding meaningful RPC load
 * next to the actual video traffic. */
#define DAEMON_HEALTH_CHECK_INTERVAL_S 3.0
/* Confirmed live: reopening the data socket -- at startup or after a
 * reconnect -- can still fail even with
 * ar8030_link_force_close_all_sockets() called first. The single-socket
 * ar8030_link_force_close_socket() was tried first and confirmed NOT to
 * clear the stuck state (returned -2); force-close-*all* does clear it,
 * but not always instantly -- the daemon's own log showed one open
 * succeeding and then tearing itself back down again within about a
 * millisecond, well under any real timeout, on an attempt sandwiched
 * between two others that worked fine. A short retry loop here matches
 * every other step already in this same connect/reconnect sequence
 * (ar8030_link_connect_retry(), resolve_connected_slot()) -- none of
 * them assume their first attempt succeeds either. */
#define SOCKET_REOPEN_MAX_ATTEMPTS 5
#define SOCKET_REOPEN_RETRY_MS 500

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
    /* Socket experiment knobs (stock's video socket is opened RX|TX with
     * options {5, 0x800} on port 3 -- see README/docs; these let that be
     * tried without a rebuild). Defaults keep the previous behaviour. */
    uint32_t sock_tx_buf;
    uint32_t sock_rx_buf;
    int sock_bidir;
    int idr_coalesce_ms;
    uint32_t ring_backlog_slots; /* URGENT trips at low_water_slots >= this (ring has 8 slots) */
    double ring_backoff;         /* bitrate multiplier per URGENT cut */
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
            "  -m <fraction>  bitrate margin applied to link throughput (default 0.545)\n"
            "  -n <kbps>      minimum bitrate floor (default 512)\n"
            "  -x <kbps>      maximum bitrate ceiling (default 35000)\n"
            "  -Q <slots>     ring backlog that triggers an URGENT bitrate cut (default 6; the\n"
            "                 ring has 8 slots, one frame each -- higher reacts later, tolerates bursts)\n"
            "  -K <fraction>  bitrate multiplier applied per URGENT cut (default 0.92)\n"
            "  -B <bytes>     socket tx_buf_size option (default 65536)\n"
            "  -R <bytes>     socket rx_buf_size option (default 1024)\n"
            "  -N             open the socket TX only (default is RX|TX like stock, which also\n"
            "                 makes the daemon count it for `ar8030-linkctl rate`; RX also carries\n"
            "                 the ground's keyframe requests -- -N disables those)\n"
            "  -i <ms>        coalesce keyframe requests within this window (default %d, 0 = honor\n"
            "                 every one; PixelPilot sends 3 per request, 100 ms apart)\n"
            "  -v             print periodic in/out stats to stderr (frames, chunks, bytes, "
            "failures, ring health)\n"
            "  -h             this help\n",
            argv0, DEFAULT_RING_NAME, DEFAULT_DAEMON_IP, DEFAULT_VIDEO_PORT,
            AR8030_CHUNK_DEFAULT_PAYLOAD, DEFAULT_WRITE_TIMEOUT_MS, DEFAULT_WAYBEAM_HOST,
            DEFAULT_WAYBEAM_PORT, DEFAULT_IDR_COALESCE_MS);
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
    a->margin = 0.545; /* 20 Mbit/s of the 36.7 Mbit/s frame-changed link: the real drain rate is ~21.5, stock holds 21.3 */
    a->min_kbps = 512;
    a->max_kbps = 35000;
    a->verbose = 0;
    a->sock_tx_buf = 64 * 1024;
    a->sock_rx_buf = 1024;
    a->sock_bidir = 1;
    a->idr_coalesce_ms = DEFAULT_IDR_COALESCE_MS;
    a->ring_backlog_slots = 6; /* of 8; measured: <=4 fires on ordinary keyframe bursts */
    a->ring_backoff = 0.92;

    int opt;
    while ((opt = getopt(argc, argv, "r:d:s:o:c:t:w:P:m:n:x:B:R:Q:K:i:XNvh")) != -1) {
        switch (opt) {
        case 'i':
            a->idr_coalesce_ms = atoi(optarg);
            break;
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
        case 'B':
            a->sock_tx_buf = (uint32_t)strtoul(optarg, NULL, 0);
            break;
        case 'R':
            a->sock_rx_buf = (uint32_t)strtoul(optarg, NULL, 0);
            break;
        case 'X':
            a->sock_bidir = 1;
            break;
        case 'N':
            a->sock_bidir = 0;
            break;
        case 'Q':
            a->ring_backlog_slots = (uint32_t)strtoul(optarg, NULL, 10);
            break;
        case 'K':
            a->ring_backoff = strtod(optarg, NULL);
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

/* Stats the *currently named* shm object, independent of (and without
 * touching) any mapping this process already has attached -- used to
 * notice waybeam has restarted out from under us.
 *
 * venc_frame_ring_create() (waybeam's own producer-side call) does
 * shm_unlink() then shm_open(O_CREAT|O_EXCL) on every single start --
 * "stale-ring guard" in its own comment -- so a restart (a config change
 * that requires one, or a crash) always creates a brand-new shm object
 * under the same name, never reopens the existing one. Unlinking a POSIX
 * shm object behaves like unlinking a regular file: it does not
 * invalidate an already-open fd or an already-mmap()'d region in another
 * process. So an already-attached consumer's mapping keeps working
 * exactly as before -- reading a now-orphaned copy of the ring that the
 * new producer will never write to again -- with no error, no signal, and
 * (at this project's normal fps) nothing but a read timeout that just
 * keeps recurring forever to tell you something's wrong.
 *
 * Comparing the inode of a *fresh* shm_open() of the same name against
 * the one we last attached to is the standard fix for exactly this shape
 * of problem (the same technique `tail -F` uses to notice a rotated log
 * file). Returns 0 and fills *out_dev and *out_ino on success, -1 if the name
 * doesn't currently resolve to anything (waybeam mid-restart, between its
 * own shm_unlink() and next shm_open(), or not running at all). */
static int stat_named_shm(const char *ring_name, dev_t *out_dev, ino_t *out_ino)
{
    char name[256];
    if (ring_name[0] == '/')
        snprintf(name, sizeof(name), "%s", ring_name);
    else
        snprintf(name, sizeof(name), "/%s", ring_name);

    int fd = shm_open(name, O_RDONLY, 0);
    if (fd < 0)
        return -1;
    struct stat st;
    int ret = fstat(fd, &st);
    close(fd);
    if (ret != 0)
        return -1;
    *out_dev = st.st_dev;
    *out_ino = st.st_ino;
    return 0;
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

    /* Baseline for stat_named_shm()'s own staleness check below -- if this
     * fails right after a successful attach (vanishingly unlikely, we just
     * opened the same name), leave both at 0 so the first read timeout's
     * check treats the current object as "unknown" and reattaches rather
     * than silently skipping detection forever. */
    dev_t ring_dev = 0;
    ino_t ring_ino = 0;
    stat_named_shm(args.ring_name, &ring_dev, &ring_ino);

    ar8030_link_t link;
    fprintf(stderr, "tx: connecting to ar8030d at %s:%d...\n", args.daemon_ip, args.daemon_port);
    if (ar8030_link_connect_retry(&link, args.daemon_ip, args.daemon_port, RETRY_MS, &g_stop) != 0) {
        venc_frame_ring_destroy(ring);
        return 0;
    }

    /* Captured before the very first resolve overwrites args.slot below --
     * the main loop's own reconnect path needs to know whether "-s auto"
     * was actually requested, since a peer's connected slot isn't
     * guaranteed to stay the same across a daemon restart (same reason
     * -s auto exists at all: see this option's own usage() text). */
    int slot_was_auto = (args.slot < 0);

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
    sock_opt.tx_buf_size = args.sock_tx_buf;
    sock_opt.rx_buf_size = args.sock_rx_buf;
    const uint32_t sock_flags = args.sock_bidir ? (BB_SOCK_FLAG_TX | BB_SOCK_FLAG_RX) : BB_SOCK_FLAG_TX;
    fprintf(stderr, "tx: socket flags=0x%x tx_buf=%u rx_buf=%u write_timeout=%dms port=%d\n", sock_flags,
            sock_opt.tx_buf_size, sock_opt.rx_buf_size, args.write_timeout_ms, args.port);
    fprintf(stderr, "tx: bitrate control margin=%.3f min=%u max=%u kbps backlog_slots=%u backoff=%.2f\n", args.margin,
            args.min_kbps, args.max_kbps, args.ring_backlog_slots, args.ring_backoff);
    /* Same force-close + retry as the reconnect path below (see its own,
     * longer comment) -- confirmed live that this exact failure isn't
     * specific to reconnecting: it also hit a genuinely fresh startup
     * once a prior run had left the daemon in this state (crashed
     * without a clean bb_socket_close(), device not rebooted since).
     *
     * Scoped to this port only, NOT ar8030_link_force_close_all_sockets():
     * audio_tx (see ../audio_tx/main.c) may already own a concurrently-open
     * bb_socket on a different port of this same device, and
     * BB_FORCE_CLS_SOCKET_ALL closes every port, not just this one --
     * confirmed live to take down a running audio stream the instant this
     * process (re)started. The narrower call was previously found not to
     * clear a stale post-crash state on its own (see this project's earlier
     * history), so this may reintroduce that specific failure mode; the
     * retry loop below is the mitigation until/unless that's confirmed to
     * still be a problem now that force-close is scoped. */
    ar8030_link_force_close_socket(&link, (bb_slot_e)args.slot, (uint32_t)args.port);
    int startup_open_ret = -1;
    int startup_open_attempt;
    for (startup_open_attempt = 0; startup_open_attempt < SOCKET_REOPEN_MAX_ATTEMPTS && !g_stop;
         startup_open_attempt++) {
        startup_open_ret = ar8030_link_open_socket(&link, (bb_slot_e)args.slot, (uint32_t)args.port,
                                                    sock_flags, &sock_opt);
        if (startup_open_ret == 0)
            break;
        usleep(SOCKET_REOPEN_RETRY_MS * 1000);
    }
    if (startup_open_ret != 0) {
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
    bc_cfg.ring_backlog_high_slots = args.ring_backlog_slots; /* venc_frame_ring.h: >=2 is standing backlog */
    bc_cfg.ring_backoff = args.ring_backoff;
    bc_cfg.ramp_step = 0.10;          /* +10% per apply toward the MCS-derived target */
    bc_cfg.ramp_settle_ms = 3000;     /* no increase until 3s of clear backlog */
    bc_cfg.probe_ceiling_frac = 0.95; /* after a backlog cut, stay <95% of the offending rate ... */
    bc_cfg.probe_hold_ms = 15000;     /* ... for 15s, then probe above it again */
    bc_cfg.roi_max_kbps = 3000;    /* last-resort measure: only below ~3Mbit/s (this project's own
                                     * "2-4Mbit/s" call, middle of the range) */
    bc_cfg.roi_recovery_ms = 5000; /* hold ROI on for 5s of clear backlog + recovered bitrate before
                                     * switching it back off */
    bc_cfg.ldpc_ratio_high = 0.10; /* see bitrate_ctl.h's own comment: >=10% LDPC blocks failing is
                                     * treated as the radio actively under repair pressure */
    bc_cfg.ldpc_backoff = 0.85;    /* same cut factor as ring_backoff above */
    bc_cfg.retx_event_backoff = 0.85; /* see bitrate_ctl.h's own comment: fires on every
                                        * BB_EVENT_RETX_TOO_MANY, the vendor's own real
                                        * retx-pressure signal (independently recovered via
                                        * Ghidra, not in the SDK's own bb_event_e) */
    bc_cfg.stop_flag = &g_stop;

    idr_ctrl_cfg_t idr_cfg = {
        .link = &link, .waybeam_host = args.waybeam_host, .waybeam_port = args.waybeam_port,
        .coalesce_ms = args.idr_coalesce_ms, .dedup_ms = 2000 /* alink_idr's --keep-ms default */,
        .verbose = args.verbose, .stop_flag = &g_stop
    };
    pthread_t idr_thread;
    int idr_thread_ok = args.sock_bidir && pthread_create(&idr_thread, NULL, idr_ctrl_thread_main, &idr_cfg) == 0;

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
    double last_health_check_s = now_monotonic_s();

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

        /* Periodic daemon-liveness probe, independent of whatever the
         * data path below is doing -- see ar8030_link_is_alive()'s own
         * comment for why a run of bb_socket_write() timeouts alone
         * can't tell "the RF link is saturated" (self-clearing) apart
         * from "ar8030d itself crashed or restarted" (never clears
         * without this). */
        double now_health = now_monotonic_s();
        if (now_health - last_health_check_s >= DAEMON_HEALTH_CHECK_INTERVAL_S) {
            last_health_check_s = now_health;
            if (!ar8030_link_is_alive(&link)) {
                fprintf(stderr, "tx: ar8030d connection lost, reconnecting...\n");
                if (ar8030_link_reconnect_retry(&link, args.daemon_ip, args.daemon_port, RETRY_MS,
                                                 &g_stop) != 0)
                    break; /* stopped while waiting for the daemon to come back */
                fprintf(stderr, "tx: reconnected to ar8030d\n");

                int slot = args.slot;
                if (slot_was_auto) {
                    bb_dev_handle_t *dev = __atomic_load_n(&link.dev, __ATOMIC_ACQUIRE);
                    slot = resolve_connected_slot(dev, &g_stop);
                    if (slot < 0)
                        break; /* stopped before any peer ever reconnected */
                    fprintf(stderr, "tx: resolved connected slot %d\n", slot);
                }
                args.slot = slot;
                /* bitrate_ctl's own thread reads cfg->slot concurrently
                 * (see bitrate_ctl.h's own comment on that field). */
                __atomic_store_n(&bc_cfg.slot, (bb_slot_e)slot, __ATOMIC_RELEASE);

                /* Confirmed live: a daemon killed abruptly (not a clean
                 * shutdown) never runs its own bb_socket_close() teardown,
                 * so the fresh daemon instance's first open on this same
                 * slot/port fails with ret=-1 ("already opened", stale
                 * chip-side state from the old session) even though
                 * nothing is genuinely still using it. The single-socket
                 * ar8030_link_force_close_socket() was tried here first
                 * and confirmed live NOT to clear it (returned -2); the
                 * broader ar8030_link_force_close_all_sockets() is the
                 * one that actually worked -- see its own comment.
                 * Ignore its return value: on the common path there is
                 * nothing to force-close and this is a harmless no-op;
                 * the retry loop below is the real, checked,
                 * fatal-on-failure step.
                 *
                 * Scoped, not "all" -- see the startup call's own comment
                 * above on why the broader ioctl can no longer be used
                 * here now that audio_tx may be running concurrently on a
                 * different port of this device. */
                ar8030_link_force_close_socket(&link, (bb_slot_e)slot, (uint32_t)args.port);

                int open_ret = -1;
                int attempt;
                for (attempt = 0; attempt < SOCKET_REOPEN_MAX_ATTEMPTS && !g_stop; attempt++) {
                    open_ret = ar8030_link_open_socket(&link, (bb_slot_e)slot, (uint32_t)args.port,
                                                        sock_flags, &sock_opt);
                    if (open_ret == 0)
                        break;
                    usleep(SOCKET_REOPEN_RETRY_MS * 1000);
                }
                if (open_ret != 0) {
                    fprintf(stderr, "tx: failed to reopen bb_socket after reconnect (%d attempts), stopping\n",
                            attempt);
                    g_stop = 1; /* bitrate_ctl's thread only exits on this -- see its own
                                 * while (!*cfg->stop_flag) loop -- so pthread_join() below
                                 * would otherwise block forever after this break. Confirmed
                                 * live: without this, the process never actually exited. */
                    break;
                }
                fprintf(stderr, "tx: bb_socket open (slot=%d port=%d)\n", slot, args.port);
            }
        }

        uint32_t out_len = 0;
        int ret = venc_frame_ring_read_wait(ring, ring_buf, ring_buf_size, &out_len, 200);
        if (ret != 0) {
            /* A read timeout is also the cheapest place to notice waybeam
             * restarted out from under us (see stat_named_shm()'s own
             * comment) -- at this project's normal fps a healthy ring
             * almost never goes 200ms without a fresh frame, so a timeout
             * here is already a meaningful signal, not routine polling
             * noise. The check itself (open+fstat+close by name) is cheap
             * enough to run on every timeout regardless. */
            dev_t cur_dev;
            ino_t cur_ino;
            int have_cur = stat_named_shm(args.ring_name, &cur_dev, &cur_ino) == 0;
            if (!have_cur || cur_dev != ring_dev || cur_ino != ring_ino) {
                fprintf(stderr, "tx: frame-shm ring '%s' %s -- waybeam restarted, reattaching\n",
                        args.ring_name, have_cur ? "changed" : "disappeared");

                /* Clear bitrate_ctl's view of the ring BEFORE freeing it.
                 * That thread reads cfg->ring concurrently, every tick,
                 * via an atomic load (see bitrate_ctl.c) -- destroying and
                 * freeing the ring here while that pointer still
                 * referenced it would be a genuine use-after-free race,
                 * not just a stale read, and was confirmed live to
                 * segfault the whole process (one thread crashing takes
                 * down all of it) the first time this path was exercised
                 * against a real waybeam restart. NULL is already a
                 * supported state for cfg->ring (see its own "may be NULL"
                 * comment), so the other thread just skips its backlog
                 * check for the duration of the reattach below. */
                __atomic_store_n(&bc_cfg.ring, NULL, __ATOMIC_RELEASE);

                venc_frame_ring_destroy(ring);
                ring = NULL;
                while (!g_stop && !(ring = venc_frame_ring_attach(args.ring_name)))
                    usleep(RETRY_MS * 1000);
                if (!ring)
                    break; /* stopped while waiting for waybeam to come back */
                fprintf(stderr, "tx: reattached to ring '%s'\n", args.ring_name);

                if (stat_named_shm(args.ring_name, &ring_dev, &ring_ino) != 0) {
                    ring_dev = 0;
                    ring_ino = 0;
                }

                if (ring->slot_data_size != ring_buf_size) {
                    uint8_t *new_buf = realloc(ring_buf, ring->slot_data_size);
                    if (new_buf) {
                        ring_buf = new_buf;
                        ring_buf_size = ring->slot_data_size;
                    } else {
                        fprintf(stderr, "tx: OOM growing ring read buffer to %u bytes, stopping\n",
                                ring->slot_data_size);
                        g_stop = 1;
                    }
                }

                /* bitrate_ctl's own thread reads cfg->ring concurrently
                 * (its own backlog check) -- publish the new pointer
                 * atomically rather than a plain store, matching how it
                 * loads it (see bitrate_ctl.c's own comment). */
                __atomic_store_n(&bc_cfg.ring, ring, __ATOMIC_RELEASE);
            }
            continue; /* timeout (or just reattached), loop back and re-check g_stop */
        }
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

    g_stop = 1; /* every path out of the loop above; both threads exit on it */
    if (bc_thread_ok)
        pthread_join(bc_thread, NULL);
    if (idr_thread_ok)
        pthread_join(idr_thread, NULL);

    free(chunk_scratch);
    free(ring_buf);
    ar8030_link_close(&link);
    venc_frame_ring_destroy(ring);
    return 0;
}
