#ifndef AR8030_TRANSPORT_TIMING_SEI_H
#define AR8030_TRANSPORT_TIMING_SEI_H

/*
 * Per-frame timing carried inside the H.265 bitstream, from
 * ar8030-transport-rx to PixelPilot_rk.
 *
 * rx inserts one prefix SEI NAL (type 39) into every access unit, ahead of
 * its first slice, holding a user_data_unregistered message (payload type
 * 5): our 16-byte UUID followed by struct timing_sei_v1. Riding in the
 * bitstream means it survives RTP packetization and GStreamer's depayloader
 * untouched and stays attached to exactly the picture it describes, with
 * no side channel to keep in step. Decoders skip SEI they don't know, and a
 * raw DVR recording keeps it for offline analysis.
 *
 * Every time is the GROUND's CLOCK_MONOTONIC in microseconds: rx has
 * already mapped the air unit's capture time across with the SYNC/SYNCR
 * clock offset (rx/clock_sync.c), so PixelPilot can compare it directly
 * against its own clock -- e.g. at the page flip, for capture-to-scan-out
 * latency.
 *
 * PixelPilot_rk carries its own copy of the UUID and struct layout; keep
 * the two in step (bump version on any layout change, append-only).
 */

#include <stdint.h>

#define TIMING_SEI_VERSION 1

/* Random, fixed: identifies this message among user_data_unregistered SEI. */
#define TIMING_SEI_UUID                                                                                 \
    {0x8a, 0x3f, 0x52, 0xc1, 0x6e, 0x0b, 0x4d, 0x97, 0xb2, 0x14, 0x5a, 0xe3, 0x09, 0x7c, 0xd1, 0x46}

#define TIMING_SEI_FLAG_SYNC_VALID 0x01 /* capture_ground_us is on the ground clock */
#define TIMING_SEI_FLAG_ENCODE 0x02     /* encode_us is known (sidecar is flowing) */

#pragma pack(push, 1)
struct timing_sei_v1 { /* all fields little endian */
    uint8_t version;              /* TIMING_SEI_VERSION */
    uint8_t flags;                /* TIMING_SEI_FLAG_* */
    uint16_t frame_seq;           /* the air's video frame counter */
    uint64_t capture_ground_us;   /* sensor capture (encoder pts) on the ground clock */
    uint64_t rx_done_ground_us;   /* last chunk of this frame reassembled by rx */
    uint32_t sync_uncertainty_us; /* +- bound of the clock mapping: half the best round trip */
    uint32_t encode_us;           /* recent average capture -> encode done, from the sidecar */
    uint32_t frames_lost;         /* lifetime count of frames that never reached rx whole */
};
#pragma pack(pop)

#define TIMING_SEI_PAYLOAD_SIZE (16 + (int)sizeof(struct timing_sei_v1))
/* Worst case with an emulation-prevention byte after every second byte. */
#define TIMING_SEI_NAL_MAX (4 + 2 + 2 + TIMING_SEI_PAYLOAD_SIZE * 3 / 2 + 2)

/* Builds the complete NAL, 4-byte start code included, into out (at least
 * TIMING_SEI_NAL_MAX bytes). Returns its length. */
uint32_t timing_sei_build(const struct timing_sei_v1 *t, uint8_t *out);

/* Inserts the SEI NAL in front of the first VCL NAL of the Annex-B access
 * unit buf[0..*len) in place (parameter sets stay ahead of it). Returns 0,
 * or -1 if there is no slice in the AU or cap is too small -- the AU is
 * then left unchanged. */
int timing_sei_insert(uint8_t *buf, uint32_t *len, uint32_t cap, const uint8_t *sei, uint32_t sei_len);

/* Finds our SEI in an Annex-B access unit and decodes it. Returns 1 if
 * found, 0 otherwise. Reference implementation for PixelPilot's parser. */
int timing_sei_parse(const uint8_t *au, uint32_t len, struct timing_sei_v1 *out);

#endif
