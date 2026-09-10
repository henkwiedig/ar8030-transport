#ifndef AR8030_TRANSPORT_CHUNK_STREAM_H
#define AR8030_TRANSPORT_CHUNK_STREAM_H

/*
 * Pulls one complete ar8030_chunk_hdr + payload at a time out of a
 * bb_socket running in stream mode (no BB_SOCK_FLAG_DATAGRAM -- see
 * README.md "Stream mode, not datagram"). Unlike datagram mode, a
 * bb_socket_read() here has no relationship to write() boundaries on the
 * sender: it can return less than one chunk, more than one concatenated,
 * or a chunk split across several read() calls. This buffers and
 * reassembles the byte stream into discrete chunks using
 * ar8030_chunk_hdr.payload_len, and resyncs (byte-at-a-time) if the
 * expected magic doesn't validate at the current position -- which
 * should only happen right after a corrupted read or a sender restart
 * mid-stream, not in steady state.
 *
 * The actual read is injected as a callback (matching chunker.h's
 * ar8030_chunk_send_fn) rather than calling bb_socket_read() directly, so
 * this module has no AR8030 SDK dependency and is host-testable like the
 * rest of common/ -- see rx/main.c for the real bb_socket_read()-backed
 * callback and test/roundtrip_test.c's fake_wire_read() for a fake one.
 */

#include "ar8030_chunk.h"

#include <stdint.h>

/* Returns >0 = that many bytes appended at buf[0..cap), 0 or <0 = no data
 * this call (a timeout, or a stop request -- indistinguishable to the
 * caller here, both just mean "try again"). Must not block longer than
 * timeout_ms. */
typedef int (*ar8030_chunk_read_fn)(void *ctx, uint8_t *buf, uint32_t cap, int timeout_ms);

typedef struct {
    ar8030_chunk_read_fn read_fn;
    void *read_ctx;
    int read_timeout_ms;
    const volatile int *stop_flag; /* checked between reads so a stalled
                                     * link can't wedge shutdown */
    uint8_t *buf;
    uint32_t cap;
    uint32_t len; /* valid bytes currently buffered at buf[0..len) */

    /* Lifetime counters for -v stats reporting (rx/main.c). chunks_read /
     * bytes_consumed count only chunks returned to the caller (well-formed
     * header AND a passing payload checksum). resync_dropped_bytes counts
     * bytes discarded while scanning past a bad magic to find the next
     * real header -- it should stay at (or very near) zero in steady
     * state; anything climbing steadily means the stream lost alignment,
     * e.g. a corrupted read or the sender having restarted mid-stream.
     * checksum_fails counts chunks with a well-formed header (valid magic,
     * in-range payload_len) whose payload's CRC16 didn't match -- i.e. the
     * bytes arrived with the framing intact but corrupted content, which
     * resyncing can't catch or fix (the header itself was fine, so there
     * is nothing to resync past); such a chunk is silently skipped, never
     * handed to the caller, since forwarding known-corrupt video data is
     * worse than treating it as lost. */
    uint64_t chunks_read;
    uint64_t bytes_consumed;
    uint64_t resync_dropped_bytes;
    uint64_t checksum_fails;
} ar8030_chunk_stream_t;

/* buf/cap must already be allocated by the caller and must be at least
 * AR8030_CHUNK_HDR_SIZE + AR8030_CHUNK_MAX_PAYLOAD bytes (a size that
 * holds only whole chunks with no slack will still work correctness-wise,
 * but a few chunks of headroom means fewer read_fn round trips per chunk
 * under a burst -- see rx/main.c for the size it actually allocates). */
void ar8030_chunk_stream_init(ar8030_chunk_stream_t *s, ar8030_chunk_read_fn read_fn, void *read_ctx,
                               uint8_t *buf, uint32_t cap, int read_timeout_ms,
                               const volatile int *stop_flag);

/* Returns 1 with *hdr and payload_buf[0..*out_payload_len) filled in when
 * a complete chunk was assembled. Returns 0 when nothing completed yet
 * (a read timed out, or *stop_flag fired) -- the caller should just call
 * again; any bytes already buffered are kept across calls. Returns -1 on
 * a hard error (payload_buf_cap too small for what a chunk header claims,
 * which should never happen with a correctly-sized buffer -- see the
 * init() comment). */
int ar8030_chunk_stream_read(ar8030_chunk_stream_t *s, struct ar8030_chunk_hdr *hdr,
                              uint8_t *payload_buf, uint32_t payload_buf_cap,
                              uint32_t *out_payload_len);

#endif /* AR8030_TRANSPORT_CHUNK_STREAM_H */
