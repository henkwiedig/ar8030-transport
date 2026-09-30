#ifndef AR8030_TRANSPORT_SIDECAR_WIRE_H
#define AR8030_TRANSPORT_SIDECAR_WIRE_H

/*
 * The subset of waybeam's RTP timing sidecar wire protocol this project
 * touches (waybeam_venc/include/rtp_sidecar.h is the canonical spec; the
 * values below must stay identical to it). All multi-byte fields are
 * network byte order.
 *
 * tx/sidecar_sub.c subscribes to waybeam's sidecar on the air unit and
 * forwards every MSG_FRAME verbatim as an AR8030_CHUNK_CODEC_SIDECAR chunk;
 * rx/sidecar_srv.c serves those datagrams to local ground subscribers with
 * the same subscribe/TTL rules waybeam uses. Nothing here parses trailers:
 * only the fixed MSG_FRAME head (capture_us, frame_ready_us) is read, for
 * the encode-time figure in the timing SEI.
 */

#include <stdint.h>

#define SIDECAR_MAGIC 0x52545053u /* "RTPS" */
#define SIDECAR_VERSION 1

#define SIDECAR_MSG_SUBSCRIBE 1
#define SIDECAR_MSG_FRAME 2
#define SIDECAR_MSG_SYNC_REQ 3
#define SIDECAR_MSG_SYNC_RESP 4

#define SIDECAR_SUB_TTL_US (5 * 1000000ULL)
#define SIDECAR_MAX_SUBS 4

#pragma pack(push, 1)
struct sidecar_hdr {
    uint32_t magic;
    uint8_t version;
    uint8_t msg_type;
    uint8_t b6; /* MSG_FRAME: stream_id, otherwise padding */
    uint8_t b7; /* MSG_FRAME: flags, otherwise padding */
};

struct sidecar_frame {
    struct sidecar_hdr hdr;
    uint32_t ssrc;
    uint32_t rtp_timestamp;
    uint64_t frame_id;
    uint64_t frame_ready_us; /* CLOCK_MONOTONIC_RAW at encode complete */
    uint16_t seq_first;
    uint16_t seq_count;
    uint64_t capture_us; /* encoder PTS as CLOCK_MONOTONIC, 0 = unknown */
    uint64_t last_pkt_send_us;
};

struct sidecar_sync_req {
    struct sidecar_hdr hdr;
    uint64_t t1_us;
};

struct sidecar_sync_resp {
    struct sidecar_hdr hdr;
    uint64_t t1_us;
    uint64_t t2_us;
    uint64_t t3_us;
};
#pragma pack(pop)

#ifdef __cplusplus
static_assert(sizeof(struct sidecar_frame) == 52, "sidecar MSG_FRAME head is 52 bytes");
#else
_Static_assert(sizeof(struct sidecar_frame) == 52, "sidecar MSG_FRAME head is 52 bytes");
_Static_assert(sizeof(struct sidecar_sync_req) == 16, "sidecar SYNC_REQ is 16 bytes");
_Static_assert(sizeof(struct sidecar_sync_resp) == 32, "sidecar SYNC_RESP is 32 bytes");
#endif

static inline uint64_t sidecar_be64(uint64_t v)
{
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    return __builtin_bswap64(v);
#else
    return v;
#endif
}

#endif /* AR8030_TRANSPORT_SIDECAR_WIRE_H */
