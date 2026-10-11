/*
 * ar8030-still -- stands in for waybeam on the frame-shm ring with one
 * stored picture, repeated.
 *
 * Creates the ring waybeam would (venc_frame_ring_create(), same name, so
 * ar8030-transport-tx attaches to it like to a restarted waybeam) and
 * publishes one pre-encoded H.265 access unit -- a keyframe that carries
 * its own VPS/SPS/PPS, so a decoder can start on any copy -- a few times a
 * second, until SIGTERM/SIGINT removes the ring again.
 *
 * Meant for while waybeam is stopped, e.g. to cool down on the ground: the
 * goggles keep a picture (say "VTX OVERHEAT") instead of freezing on the
 * last live frame. Never run it next to waybeam: both own the same ring
 * name and each unlinks the other's on start/exit.
 *
 * Switching back: stop this first (its ring disappears, tx notices on its
 * next read timeout and retries the attach), then start waybeam.
 */

#include "venc_frame_ring.h"

#include <errno.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define DEFAULT_RING_NAME "venc_frames"
#define DEFAULT_FPS       5
#define MAX_FRAME_BYTES   (256 * 1024)
/* A handful of slots: tx reads every frame as it comes, and a full ring
 * (tx not attached yet) just drops this period's copy. */
#define RING_SLOTS        4

static volatile sig_atomic_t g_stop = 0;

static void on_signal(int sig)
{
    (void)sig;
    g_stop = 1;
}

static uint32_t now_monotonic_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u);
}

static void usage(const char* argv0)
{
    fprintf(stderr,
            "Usage: %s -f <frame.h265> [-r <ring>] [-F <fps>]\n"
            "  -f <file>   one Annex-B H.265 access unit, a keyframe with VPS/SPS/PPS\n"
            "  -r <name>   frame-shm ring name (default %s)\n"
            "  -F <fps>    copies per second, 1-30 (default %d)\n",
            argv0, DEFAULT_RING_NAME, DEFAULT_FPS);
}

static uint8_t* load_frame(const char* path, uint32_t* out_len)
{
    FILE* f = fopen(path, "rb");
    if (!f) {
        fprintf(stderr, "still: can't open %s: %s\n", path, strerror(errno));
        return NULL;
    }
    uint8_t* buf = malloc(MAX_FRAME_BYTES + 1);
    size_t   n   = buf ? fread(buf, 1, MAX_FRAME_BYTES + 1, f) : 0;
    fclose(f);
    if (!buf || n == 0 || n > MAX_FRAME_BYTES) {
        fprintf(stderr, "still: %s is empty or larger than %d bytes\n", path, MAX_FRAME_BYTES);
        free(buf);
        return NULL;
    }
    /* Annex B: starts with a 3- or 4-byte start code. */
    if (!(n > 4 && buf[0] == 0 && buf[1] == 0 && (buf[2] == 1 || (buf[2] == 0 && buf[3] == 1)))) {
        fprintf(stderr, "still: %s is not an Annex-B stream (no start code)\n", path);
        free(buf);
        return NULL;
    }
    *out_len = (uint32_t)n;
    return buf;
}

int main(int argc, char** argv)
{
    const char* file      = NULL;
    const char* ring_name = DEFAULT_RING_NAME;
    int         fps       = DEFAULT_FPS;
    int         opt;
    while ((opt = getopt(argc, argv, "f:r:F:h")) != -1) {
        switch (opt) {
        case 'f':
            file = optarg;
            break;
        case 'r':
            ring_name = optarg;
            break;
        case 'F':
            fps = atoi(optarg);
            break;
        default:
            usage(argv[0]);
            return opt == 'h' ? 0 : 1;
        }
    }
    if (!file || fps < 1 || fps > 30) {
        usage(argv[0]);
        return 1;
    }

    uint32_t frame_len;
    uint8_t* frame = load_frame(file, &frame_len);
    if (!frame) {
        return 1;
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    /* Slot = metadata + frame, rounded up to 4 KiB. */
    uint32_t           slot_size = (VENC_FRAME_META_SIZE + frame_len + 4095u) & ~4095u;
    venc_frame_ring_t* ring      = venc_frame_ring_create(ring_name, RING_SLOTS, slot_size);
    if (!ring) {
        fprintf(stderr, "still: can't create ring '%s'\n", ring_name);
        free(frame);
        return 1;
    }
    fprintf(stderr, "still: %s (%u bytes) on ring '%s' at %d fps\n", file, frame_len, ring_name, fps);

    VencFrameMeta meta;
    memset(&meta, 0, sizeof(meta));
    meta.codec = VENC_FRAME_CODEC_H265;
    meta.flags = VENC_FRAME_FLAG_IDR;

    useconds_t period_us = (useconds_t)(1000000 / fps);
    while (!g_stop) {
        meta.pts = now_monotonic_us();
        venc_frame_ring_write(ring, &meta, frame, frame_len); /* full: drop this copy */
        usleep(period_us);
    }

    venc_frame_ring_destroy(ring);
    free(frame);
    fprintf(stderr, "still: stopped\n");
    return 0;
}
