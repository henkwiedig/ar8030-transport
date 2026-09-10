#ifndef AR8030_TRANSPORT_CRC16_H
#define AR8030_TRANSPORT_CRC16_H

/*
 * CRC16-CCITT (poly 0x1021, init 0xFFFF, no reflection) over a chunk's
 * payload bytes -- see common/ar8030_chunk.h's checksum field.
 *
 * This exists because tx/rx's own stats showed zero write failures, zero
 * resync events and zero dropped frames, yet video still visibly
 * corrupted ("green blocks", recovering after the next IDR) -- the
 * classic symptom of a decoder fed *corrupted*, not missing, frame data.
 * ar8030_chunk_hdr's magic only ever validated framing (where one chunk
 * ends and the next begins), never the payload bytes themselves, so bit
 * errors that survive whatever FEC/retx the AR8030 baseband itself does
 * would sail through completely undetected. Telling precedent: the stock
 * vendor streamer builds its own checksum into its own custom header
 * (see fpv_bb_video_stream_send, reverse-engineered from ar_ldyhs_sky) --
 * this link evidently needs one.
 *
 * Not required to match any external CRC16 standard or interoperate with
 * anything outside this project -- tx and rx only need to agree with
 * each other, which a fixed poly/init picked once and used on both ends
 * for both cases here trivially satisfies.
 */

#include <stdint.h>

uint16_t ar8030_crc16(const uint8_t *data, uint32_t len);

#endif /* AR8030_TRANSPORT_CRC16_H */
