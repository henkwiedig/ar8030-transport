/*
 * Host-only test of the per-frame timing path: the timing SEI
 * (common/timing_sei.c) and the SYNC/SYNCR clock mapping
 * (rx/clock_sync.c).
 */

#include "clock_sync.h"
#include "timing_sei.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

/* 00 00 00, 00 00 01 and 00 00 02 may not appear inside a NAL (00 00 03
 * is the emulation-prevention escape itself). */
static int has_start_code(const uint8_t *p, uint32_t len)
{
    for (uint32_t i = 0; i + 2 < len; i++)
        if (p[i] == 0 && p[i + 1] == 0 && p[i + 2] <= 2)
            return 1;
    return 0;
}

static void test_sei(void)
{
    /* Values chosen to be full of zero runs, so emulation prevention has
     * to kick in (e.g. 00 00 00 01 inside capture_ground_us). */
    struct timing_sei_v1 in = {
        .version = TIMING_SEI_VERSION,
        .flags = TIMING_SEI_FLAG_SYNC_VALID | TIMING_SEI_FLAG_ENCODE,
        .frame_seq = 0x0100,
        .capture_ground_us = 0x0000000100000000ull,
        .rx_done_ground_us = 0x0000000000000003ull,
        .sync_uncertainty_us = 0,
        .encode_us = 0x00000200,
        .frames_lost = 1,
    };
    uint8_t sei[TIMING_SEI_NAL_MAX];
    uint32_t sei_len = timing_sei_build(&in, sei);
    CHECK(sei_len > 4 + 2 + TIMING_SEI_PAYLOAD_SIZE, "SEI too short (%u), emulation prevention missing?", sei_len);
    CHECK(sei_len <= TIMING_SEI_NAL_MAX, "SEI longer than TIMING_SEI_NAL_MAX");
    CHECK(!has_start_code(sei + 4, sei_len - 4), "emulated start code inside the SEI NAL");
    CHECK(sei[4] == 0x4e && sei[5] == 0x01, "NAL header %02x %02x, want prefix SEI 4e 01", sei[4], sei[5]);

    /* VPS, SPS, PPS, two slices of one picture. */
    const uint8_t vps[] = {0, 0, 0, 1, 0x40, 0x01, 0xaa};
    const uint8_t sps[] = {0, 0, 0, 1, 0x42, 0x01, 0xbb};
    const uint8_t pps[] = {0, 0, 1, 0x44, 0x01, 0xcc};
    const uint8_t slice1[] = {0, 0, 0, 1, 0x26, 0x01, 0xaf, 0x11, 0x22};
    const uint8_t slice2[] = {0, 0, 1, 0x26, 0x01, 0x2f, 0x33};
    uint8_t au[256];
    uint32_t len = 0;
    memcpy(au + len, vps, sizeof(vps)), len += sizeof(vps);
    memcpy(au + len, sps, sizeof(sps)), len += sizeof(sps);
    memcpy(au + len, pps, sizeof(pps)), len += sizeof(pps);
    uint32_t slices_at = len;
    memcpy(au + len, slice1, sizeof(slice1)), len += sizeof(slice1);
    memcpy(au + len, slice2, sizeof(slice2)), len += sizeof(slice2);
    uint32_t orig_len = len;

    CHECK(timing_sei_insert(au, &len, sizeof(sei) - 1, sei, sei_len) == -1, "insert ignored the capacity");
    CHECK(timing_sei_insert(au, &len, sizeof(au), sei, sei_len) == 0, "insert failed");
    CHECK(len == orig_len + sei_len, "AU length %u, want %u", len, orig_len + sei_len);
    CHECK(memcmp(au + slices_at, sei, sei_len) == 0, "SEI is not right before the first slice");
    CHECK(memcmp(au + slices_at + sei_len, slice1, sizeof(slice1)) == 0, "slices moved wrongly");
    CHECK(memcmp(au, vps, sizeof(vps)) == 0, "parameter sets disturbed");

    struct timing_sei_v1 out;
    memset(&out, 0xff, sizeof(out));
    CHECK(timing_sei_parse(au, len, &out) == 1, "SEI not found again");
    CHECK(memcmp(&in, &out, sizeof(in)) == 0, "SEI fields changed in the round trip");

    const uint8_t no_slice[] = {0, 0, 0, 1, 0x40, 0x01, 0xaa};
    uint8_t buf[64];
    memcpy(buf, no_slice, sizeof(no_slice));
    uint32_t nlen = sizeof(no_slice);
    CHECK(timing_sei_insert(buf, &nlen, sizeof(buf), sei, sei_len) == -1 && nlen == sizeof(no_slice),
          "insert into an AU without a slice must leave it alone");
    CHECK(timing_sei_parse(slice1, sizeof(slice1), &out) == 0, "found an SEI that isn't there");
}

/* One simulated exchange: air clock = ground + offset, the SYNC takes
 * up_us, the air holds it for turn_us, the SYNCR takes down_us. */
static void exchange(clock_sync_t *cs, uint64_t t1, int64_t offset, uint64_t up_us, uint64_t turn_us,
                     uint64_t down_us)
{
    uint64_t t2 = (uint64_t)((int64_t)(t1 + up_us) + offset);
    uint64_t t3 = t2 + turn_us;
    uint64_t t4 = t1 + up_us + turn_us + down_us;
    clock_sync_on_reply(cs, t1, t2, t3, t4);
}

static void test_clock_sync(void)
{
    clock_sync_t cs;
    clock_sync_init(&cs);
    int64_t off;
    uint32_t unc;
    CHECK(clock_sync_get(&cs, &off, &unc) == 0, "estimate before any reply");

    const int64_t offset = 123456789; /* air booted ~2 min before the ground */
    uint64_t t = 1000000;
    /* Mostly lopsided round trips (a reply queued behind a video frame is
     * the air's turnaround, not asymmetry; a slow downlink is), plus one
     * clean, symmetric one. */
    exchange(&cs, t, offset, 2000, 30000, 9000), t += 100000;
    exchange(&cs, t, offset, 1500, 500, 1500), t += 100000;
    exchange(&cs, t, offset, 4000, 100, 12000), t += 100000;
    CHECK(clock_sync_get(&cs, &off, &unc) == 1, "no estimate after three replies");
    CHECK(off == offset, "offset %lld, want %lld (the symmetric sample must win)", (long long)off,
          (long long)offset);
    CHECK(unc == 1500, "uncertainty %u, want half the best rtt (1500)", unc);

    /* Asymmetry shows up as error, bounded by the reported uncertainty. */
    clock_sync_t cs2;
    clock_sync_init(&cs2);
    exchange(&cs2, t, offset, 1000, 200, 3000);
    clock_sync_get(&cs2, &off, &unc);
    long long err = (long long)(off - offset);
    CHECK((err < 0 ? -err : err) <= (long long)unc, "error %lld beyond the uncertainty %u", err, unc);

    /* The air unit reboots: its clock restarts near zero. The new offset
     * must take over at once, not after the old low-rtt samples age out. */
    const int64_t new_offset = -900000000;
    exchange(&cs, t, new_offset, 3000, 200, 3000);
    clock_sync_get(&cs, &off, &unc);
    CHECK(off == new_offset, "after an air reboot offset is %lld, want %lld", (long long)off,
          (long long)new_offset);

    uint32_t seq;
    uint64_t t1;
    clock_sync_t cs3;
    clock_sync_init(&cs3);
    CHECK(clock_sync_due(&cs3, 5000000, &seq, &t1) == 1 && seq == 0 && t1 == 5000000, "first probe not due");
    CHECK(clock_sync_due(&cs3, 5000000 + 50000, &seq, &t1) == 0, "probe due again after 50 ms");
    CHECK(clock_sync_due(&cs3, 5000000 + CLOCK_SYNC_FAST_PERIOD_MS * 1000, &seq, &t1) == 1 && seq == 1,
          "second probe not due after the fast period");
}

int main(void)
{
    test_sei();
    test_clock_sync();
    if (failures) {
        fprintf(stderr, "timing_test: %d failure(s)\n", failures);
        return 1;
    }
    printf("timing test: SEI emulation prevention, insert and parse round trip; clock sync picks the "
           "min-rtt sample, stays within its bound, and resets on an air reboot\nPASS\n");
    return 0;
}
