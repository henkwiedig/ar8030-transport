/*
 * ar8030-transport-tx -- air-side bridge from waybeam's frame-shm ring to
 * the AR8030's non-IP baseband data channel (bb_socket).
 *
 * Reads whole encoded H.265 access units from waybeam's frame-shm ring
 * (see third_party/waybeam_frame_ring), splits each into fixed-size
 * chunks prefixed with an ar8030_chunk_hdr (common/ar8030_chunk.h), and
 * writes each chunk with bb_socket_write() on a datagram-mode bb_socket.
 * No FEC, no ACK: a lost chunk just makes its frame incomplete on the
 * ground side, which drops it and moves on (see rx/main.c).
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
#define DEFAULT_SLOT BB_SLOT_0
#define RETRY_MS 2000
/* Per-chunk bb_socket_write() ack-wait timeout. Bumped up from an
 * earlier 200ms default: on a real, contended RF link the daemon's own
 * write-completion ack can lag past 200ms under an IDR-frame burst
 * (confirmed on bench hardware -- see common/ar8030_chunk.h's default
 * payload comment), and bb_socket_write() aborts the whole call the
 * instant its single wait times out, so a short timeout was actively
 * making congestion worse rather than just tolerating it. */
#define DEFAULT_WRITE_TIMEOUT_MS 800
/* After this many back-to-back frames where not even the first chunk
 * got written (the strongest available signal that the link is
 * currently saturated, not just this one frame's bad luck), pause
 * briefly instead of immediately hammering it with the next frame's
 * chunks -- see the backoff comment in main()'s send loop. */
#define BACKOFF_AFTER_CONSECUTIVE_STALLS 5
#define BACKOFF_MS 50

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
};

static void usage(const char *argv0)
{
    fprintf(stderr,
            "usage: %s [options]\n"
            "  -r <name>      frame-shm ring name (default %s)\n"
            "  -d <ip>        ar8030d daemon IP (default %s)\n"
            "  -s <slot>      AR8030 slot (default %d)\n"
            "  -o <port>      AR8030 bb_socket logical port, 0-3 (default %d)\n"
            "  -c <bytes>     max chunk payload (default %u)\n"
            "  -t <ms>        per-chunk bb_socket_write ack-wait timeout (default %d)\n"
            "  -w <host>      waybeam HTTP host (default %s)\n"
            "  -P <port>      waybeam HTTP port (default %d)\n"
            "  -m <fraction>  bitrate margin applied to link throughput (default 0.70)\n"
            "  -n <kbps>      minimum bitrate floor (default 512)\n"
            "  -x <kbps>      maximum bitrate ceiling (default 20000)\n"
            "  -h             this help\n",
            argv0, DEFAULT_RING_NAME, DEFAULT_DAEMON_IP, DEFAULT_SLOT, DEFAULT_VIDEO_PORT,
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

    int opt;
    while ((opt = getopt(argc, argv, "r:d:s:o:c:t:w:P:m:n:x:h")) != -1) {
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

struct tx_send_ctx {
    ar8030_link_t *link;
    int write_timeout_ms;
};

/* ar8030_chunk_send_fn for ar8030_chunk_frame(): writes one chunk with a
 * bounded timeout. A half-sent frame is going to be dropped as incomplete
 * on the ground side either way, so ar8030_chunk_frame() stopping early
 * on a non-zero return (rather than this retrying) is the right call --
 * holding up the encode-thread-fed ring to fight a stalled radio link is
 * worse than just moving on to the next frame. */
static int chunk_send_to_socket(void *ctx, const uint8_t *buf, uint32_t len)
{
    struct tx_send_ctx *sc = (struct tx_send_ctx *)ctx;
    int wr = bb_socket_write(sc->link->sockfd, buf, len, sc->write_timeout_ms);
    if (wr < 0) {
        fprintf(stderr, "tx: bb_socket_write failed (len=%u, ret=%d)\n", len, wr);
        return -1;
    }
    return 0;
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

    bb_sock_opt_t sock_opt;
    sock_opt.tx_buf_size = 64 * 1024;
    sock_opt.rx_buf_size = 1024;
    if (ar8030_link_open_socket(&link, (bb_slot_e)args.slot, (uint32_t)args.port,
                                 BB_SOCK_FLAG_TX | BB_SOCK_FLAG_DATAGRAM, &sock_opt) != 0) {
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

    struct tx_send_ctx send_ctx = { .link = &link, .write_timeout_ms = args.write_timeout_ms };
    /* Consecutive frames where not even the first chunk got written --
     * see BACKOFF_AFTER_CONSECUTIVE_STALLS. A frame that sent *some*
     * chunks before failing resets this: that is ordinary loss, not
     * sustained saturation. */
    int consecutive_stalls = 0;

    uint16_t frame_seq = 0;
    while (!g_stop) {
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

        int sent = ar8030_chunk_frame(frame_seq, frame_flags_from_meta(&meta), AR8030_CHUNK_CODEC_H265,
                                       meta.pts, frame_data, frame_len, args.chunk_payload,
                                       chunk_send_to_socket, &send_ctx);
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

    free(ring_buf);
    ar8030_link_close(&link);
    venc_frame_ring_destroy(ring);
    return 0;
}
