#include "timing_sei.h"

#include <string.h>

static const uint8_t k_uuid[16] = TIMING_SEI_UUID;

#define SEI_PREFIX_NAL_TYPE 39
#define SEI_USER_DATA_UNREGISTERED 5

uint32_t timing_sei_build(const struct timing_sei_v1 *t, uint8_t *out)
{
    /* RBSP: payload type, size, UUID, data, then rbsp_trailing_bits. Both
     * type and size fit one byte (< 255). */
    uint8_t rbsp[2 + TIMING_SEI_PAYLOAD_SIZE + 1];
    uint32_t r = 0;
    rbsp[r++] = SEI_USER_DATA_UNREGISTERED;
    rbsp[r++] = (uint8_t)TIMING_SEI_PAYLOAD_SIZE;
    memcpy(rbsp + r, k_uuid, sizeof(k_uuid));
    r += sizeof(k_uuid);
    memcpy(rbsp + r, t, sizeof(*t)); /* fields are little endian on both ends (ARM) */
    r += sizeof(*t);
    rbsp[r++] = 0x80;

    uint32_t o = 0;
    out[o++] = 0;
    out[o++] = 0;
    out[o++] = 0;
    out[o++] = 1;
    out[o++] = (uint8_t)(SEI_PREFIX_NAL_TYPE << 1); /* forbidden 0, type 39, layer id 0 */
    out[o++] = 1;                                   /* nuh_temporal_id_plus1 */
    /* Emulation prevention: two zeros followed by 0..3 inside a NAL would
     * read as a start code; the timestamps contain zero bytes. */
    int zeros = 0;
    for (uint32_t i = 0; i < r; i++) {
        if (zeros >= 2 && rbsp[i] <= 3) {
            out[o++] = 3;
            zeros = 0;
        }
        out[o++] = rbsp[i];
        zeros = rbsp[i] == 0 ? zeros + 1 : 0;
    }
    return o;
}

/* Offset of the start code of the next NAL at or after from, or len. */
static uint32_t find_start_code(const uint8_t *buf, uint32_t len, uint32_t from, uint32_t *sc_len)
{
    for (uint32_t i = from; i + 2 < len; i++) {
        if (buf[i] == 0 && buf[i + 1] == 0) {
            if (buf[i + 2] == 1) {
                *sc_len = 3;
                return i;
            }
            if (i + 3 < len && buf[i + 2] == 0 && buf[i + 3] == 1) {
                *sc_len = 4;
                return i;
            }
        }
    }
    *sc_len = 0;
    return len;
}

int timing_sei_insert(uint8_t *buf, uint32_t *len, uint32_t cap, const uint8_t *sei, uint32_t sei_len)
{
    if (*len + sei_len > cap)
        return -1;
    uint32_t sc_len;
    uint32_t pos = find_start_code(buf, *len, 0, &sc_len);
    while (pos < *len) {
        uint32_t hdr = pos + sc_len;
        if (hdr < *len && ((buf[hdr] >> 1) & 0x3f) <= 31) { /* VCL: a slice */
            memmove(buf + pos + sei_len, buf + pos, *len - pos);
            memcpy(buf + pos, sei, sei_len);
            *len += sei_len;
            return 0;
        }
        pos = find_start_code(buf, *len, hdr, &sc_len);
    }
    return -1;
}

int timing_sei_parse(const uint8_t *au, uint32_t len, struct timing_sei_v1 *out)
{
    uint32_t sc_len;
    uint32_t pos = find_start_code(au, len, 0, &sc_len);
    while (pos < len) {
        uint32_t nal = pos + sc_len;
        uint32_t next_sc;
        uint32_t next = find_start_code(au, len, nal, &next_sc);
        if (nal + 2 < next && ((au[nal] >> 1) & 0x3f) == SEI_PREFIX_NAL_TYPE) {
            /* Strip emulation prevention into an RBSP copy, only as much
             * as one of our messages can be. */
            uint8_t rbsp[2 + TIMING_SEI_PAYLOAD_SIZE];
            uint32_t r = 0;
            int zeros = 0;
            for (uint32_t i = nal + 2; i < next && r < sizeof(rbsp); i++) {
                if (zeros >= 2 && au[i] == 3) {
                    zeros = 0;
                    continue;
                }
                rbsp[r++] = au[i];
                zeros = au[i] == 0 ? zeros + 1 : 0;
            }
            if (r == sizeof(rbsp) && rbsp[0] == SEI_USER_DATA_UNREGISTERED &&
                rbsp[1] >= TIMING_SEI_PAYLOAD_SIZE && memcmp(rbsp + 2, k_uuid, sizeof(k_uuid)) == 0 &&
                rbsp[2 + 16] == TIMING_SEI_VERSION) {
                memcpy(out, rbsp + 2 + 16, sizeof(*out));
                return 1;
            }
        }
        pos = next;
        sc_len = next_sc;
    }
    return 0;
}
