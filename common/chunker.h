#ifndef AR8030_TRANSPORT_CHUNKER_H
#define AR8030_TRANSPORT_CHUNKER_H

/*
 * Splits one whole encoded frame into ar8030_chunk_hdr-prefixed chunks
 * (common/ar8030_chunk.h) and hands each to a caller-supplied send
 * function. Shared by tx/main.c (send == bb_socket_write) and
 * test/roundtrip_test.c (send == append to an in-memory queue), so the
 * exact chunking logic under test is the same logic that ships.
 */

#include <stdint.h>

/* Returns 0 on success, non-zero to abort chunking (ar8030_chunk_frame()
 * stops and returns the count sent so far without treating it as fatal --
 * see tx/main.c's send_frame() comment on why a stalled link just moves
 * on to the next frame rather than retrying). buf points at a
 * caller-owned scratch buffer of at least AR8030_CHUNK_HDR_SIZE +
 * chunk_payload bytes containing the header+payload to send; len is the
 * total bytes to send from it. */
typedef int (*ar8030_chunk_send_fn)(void *ctx, const uint8_t *buf, uint32_t len);

/* Splits data[0..data_len) into chunks of at most chunk_payload bytes
 * each and calls send(ctx, ...) once per chunk, building each chunk's
 * header+payload in the caller-owned scratch buffer (scratch/scratch_cap)
 * rather than an internal fixed-size array: with AR8030_CHUNK_MAX_PAYLOAD
 * now the wire format's full 65535-byte ceiling (see ar8030_chunk.h), a
 * function-local buffer that size on every call is a needlessly large,
 * fixed stack frame on a RAM-constrained target regardless of the
 * chunk_payload actually in use. The caller (tx/main.c mallocs one once
 * at startup, sized to its actual -c value; test/roundtrip_test.c uses a
 * static array) owns the allocation instead. scratch_cap must be at
 * least AR8030_CHUNK_HDR_SIZE + chunk_payload.
 *
 * frame_seq/flags/codec/frame_pts populate every chunk's header
 * identically except chunk_idx/chunk_count.
 *
 * A zero-length data still produces exactly one (header-only) chunk, so
 * a frame is never silently dropped just for being empty.
 *
 * out_chunk_count, if non-NULL, is set to the total number of chunks
 * this frame was split into -- comparing it against the return value
 * is how a caller tells a fully-sent frame from one that failed partway
 * (see tx/main.c's -v stats). Left unset if data_len needs more than
 * 65535 chunks (see below).
 *
 * Returns the number of chunks successfully sent (send() returned 0 for
 * each), or -1 if data_len would need more than 65535 chunks (chunk_idx/
 * chunk_count are uint16_t) or scratch_cap is too small for chunk_payload
 * -- in which case nothing is sent. */
int ar8030_chunk_frame(uint16_t frame_seq, uint8_t flags, uint8_t codec, uint32_t frame_pts,
                        const uint8_t *data, uint32_t data_len, uint32_t chunk_payload,
                        ar8030_chunk_send_fn send, void *send_ctx, uint32_t *out_chunk_count,
                        uint8_t *scratch, uint32_t scratch_cap);

#endif /* AR8030_TRANSPORT_CHUNKER_H */
