/* Host test for rx/ring_out.c: what a consumer of rx's frame-shm ring sees,
 * entry by entry, in the cases docs/ground-frame-ring.md defines.
 *   make -C test check */

#include "ring_out.h"

#include "ar8030_chunk.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static int failures;

#define CHECK(cond, ...)                                                                                \
    do {                                                                                                \
        if (!(cond)) {                                                                                  \
            fprintf(stderr, "FAIL %s:%d: ", __FILE__, __LINE__);                                        \
            fprintf(stderr, __VA_ARGS__);                                                               \
            fprintf(stderr, "\n");                                                                      \
            failures++;                                                                                 \
        }                                                                                               \
    } while (0)

static uint8_t entry[4096];

/* Reads the next entry: returns its flags, or -1 if the ring is empty;
 * the payload (meta stripped) lands in entry[], its length in *len. */
static int next_entry(venc_frame_ring_t *c, uint32_t *len)
{
    uint32_t n = 0;
    if (venc_frame_ring_read(c, entry, sizeof(entry), &n) != 0)
        return -1;
    VencFrameMeta m;
    memcpy(&m, entry, VENC_FRAME_META_SIZE);
    *len = n - VENC_FRAME_META_SIZE;
    memmove(entry, entry + VENC_FRAME_META_SIZE, *len);
    if (m.codec != VENC_FRAME_CODEC_H265)
        return -2;
    return m.flags;
}

static const uint8_t A[] = {0, 0, 0, 1, 0x40, 0x01};
static const uint8_t B[] = {0, 0, 1, 0x26, 0x01, 0xaf};

int main(void)
{
    char name[64];
    snprintf(name, sizeof(name), "/ring_out_test_%d", (int)getpid());

    rx_ring_out_t o;
    if (rx_ring_out_open(&o, name, 4, 256) != 0) {
        fprintf(stderr, "FAIL: cannot create the ring\n");
        return 1;
    }
    venc_frame_ring_t *c = venc_frame_ring_attach(name);
    if (!c) {
        fprintf(stderr, "FAIL: cannot attach to the ring\n");
        rx_ring_out_close(&o);
        return 1;
    }
    uint32_t len;
    int f;

    /* A whole frame, IDR: flags carried over, FRAME_END not. */
    CHECK(rx_ring_out_publish(&o, 10, 1000, AR8030_CHUNK_FLAG_IDR | AR8030_CHUNK_FLAG_FRAME_END, 0, A,
                              sizeof(A), NULL, 0) == 0, "publish frame 10");
    f = next_entry(c, &len);
    CHECK(f == VENC_FRAME_FLAG_IDR, "frame 10 flags 0x%x", f);
    CHECK(len == sizeof(A) && !memcmp(entry, A, sizeof(A)), "frame 10 payload");

    /* A streamed frame: two parts, then the SEI + the rest in one entry. */
    rx_ring_out_publish(&o, 11, 1016, AR8030_CHUNK_FLAG_GDR, 1, A, sizeof(A), NULL, 0);
    rx_ring_out_publish(&o, 11, 1016, AR8030_CHUNK_FLAG_GDR, 1, B, sizeof(B), NULL, 0);
    rx_ring_out_publish(&o, 11, 1016, AR8030_CHUNK_FLAG_GDR, 0, A, sizeof(A), B, sizeof(B));
    f = next_entry(c, &len);
    CHECK(f == (VENC_FRAME_FLAG_GDR | VENC_FRAME_FLAG_PARTIAL), "frame 11 part 1 flags 0x%x", f);
    f = next_entry(c, &len);
    CHECK(f == (VENC_FRAME_FLAG_GDR | VENC_FRAME_FLAG_PARTIAL), "frame 11 part 2 flags 0x%x", f);
    f = next_entry(c, &len);
    CHECK(f == VENC_FRAME_FLAG_GDR, "frame 11 last part flags 0x%x", f);
    CHECK(len == sizeof(A) + sizeof(B) && !memcmp(entry, A, sizeof(A)) && !memcmp(entry + sizeof(A), B, sizeof(B)),
          "frame 11 last part = a then b");

    /* Frame 12 lost: frame 13 says so. */
    rx_ring_out_publish(&o, 13, 1048, 0, 0, A, sizeof(A), NULL, 0);
    f = next_entry(c, &len);
    CHECK(f == VENC_FRAME_FLAG_DISCONT, "frame 13 after a gap: flags 0x%x", f);
    rx_ring_out_publish(&o, 14, 1064, 0, 0, A, sizeof(A), NULL, 0);
    f = next_entry(c, &len);
    CHECK(f == 0, "frame 14 in order: flags 0x%x", f);

    /* Frame 15 cut short after its first part: frame 16 says so. */
    rx_ring_out_publish(&o, 15, 1080, 0, 1, A, sizeof(A), NULL, 0);
    rx_ring_out_publish(&o, 16, 1096, 0, 0, B, sizeof(B), NULL, 0);
    f = next_entry(c, &len);
    CHECK(f == VENC_FRAME_FLAG_PARTIAL, "frame 15 part flags 0x%x", f);
    f = next_entry(c, &len);
    CHECK(f == VENC_FRAME_FLAG_DISCONT, "frame 16 after a cut picture: flags 0x%x", f);

    /* An empty last part only ends the access unit. */
    rx_ring_out_publish(&o, 17, 1112, 0, 1, A, sizeof(A), NULL, 0);
    rx_ring_out_publish(&o, 17, 1112, 0, 0, NULL, 0, NULL, 0);
    f = next_entry(c, &len);
    CHECK(f == VENC_FRAME_FLAG_PARTIAL, "frame 17 part flags 0x%x", f);
    f = next_entry(c, &len);
    CHECK(f == 0 && len == 0, "frame 17 empty end: flags 0x%x len %u", f, len);

    /* A full ring (4 slots, nobody reading): the 5th entry is dropped and
     * the next one published carries DISCONT. */
    for (uint16_t s = 18; s < 22; s++)
        CHECK(rx_ring_out_publish(&o, s, 0, 0, 0, A, sizeof(A), NULL, 0) == 0, "fill %u", s);
    CHECK(rx_ring_out_publish(&o, 22, 0, 0, 0, A, sizeof(A), NULL, 0) == -1, "frame 22 into a full ring");
    for (int i = 0; i < 4; i++) {
        f = next_entry(c, &len);
        CHECK(f == 0, "drain entry %d flags 0x%x", i, f);
    }
    CHECK(next_entry(c, &len) == -1, "ring empty after the drain");
    rx_ring_out_publish(&o, 23, 0, 0, 0, A, sizeof(A), NULL, 0);
    f = next_entry(c, &len);
    CHECK(f == VENC_FRAME_FLAG_DISCONT, "frame 23 after a full-ring drop: flags 0x%x", f);

    /* Bigger than a slot: dropped, the next entry carries DISCONT. */
    static uint8_t big[300];
    CHECK(rx_ring_out_publish(&o, 24, 0, 0, 0, big, sizeof(big), NULL, 0) == -1, "oversize frame 24");
    rx_ring_out_publish(&o, 25, 0, 0, 0, A, sizeof(A), NULL, 0);
    f = next_entry(c, &len);
    CHECK(f == VENC_FRAME_FLAG_DISCONT, "frame 25 after an oversize drop: flags 0x%x", f);
    CHECK(next_entry(c, &len) == -1, "nothing published for the oversize frame");

    /* Counter wrap-around is not a gap. */
    rx_ring_out_publish(&o, 0xffff, 0, 0, 0, A, sizeof(A), NULL, 0);
    rx_ring_out_publish(&o, 0, 0, 0, 0, A, sizeof(A), NULL, 0);
    f = next_entry(c, &len);
    CHECK(f == VENC_FRAME_FLAG_DISCONT, "0xffff after 25: flags 0x%x", f);
    f = next_entry(c, &len);
    CHECK(f == 0, "0 after 0xffff: flags 0x%x", f);

    CHECK(o.dropped == 2, "dropped %llu", (unsigned long long)o.dropped);
    CHECK(o.discont_entries == 5, "discont entries %llu", (unsigned long long)o.discont_entries);

    venc_frame_ring_destroy(c);
    rx_ring_out_close(&o);

    if (failures) {
        fprintf(stderr, "ring_out_test: %d failure(s)\n", failures);
        return 1;
    }
    printf("ring_out_test: all passed\n");
    return 0;
}
