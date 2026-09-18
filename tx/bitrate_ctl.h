#ifndef AR8030_TRANSPORT_TX_BITRATE_CTL_H
#define AR8030_TRANSPORT_TX_BITRATE_CTL_H

/*
 * Feeds the AR8030's own local link-budget signal into waybeam's live
 * bitrate control, so the encoder doesn't outrun what the radio can
 * currently carry.
 *
 * The air unit (AP role) can read its own current TX MCS and the SDK's
 * own theoretical-throughput figure for it (BB_GET_MCS ->
 * bb_get_mcs_out_t.throughput, already in kbps -- see bb_api.h) without
 * any round trip to the ground: the SDK exposes this locally per side,
 * it does not need the peer to report anything back. BB_EVENT_MCS_CHANGE
 * / BB_EVENT_LINK_STATE are used only as a "go check now" wake hint; the
 * poll loop is the source of truth so a missed/coalesced event callback
 * can never wedge the control loop.
 */

#include "ar8030_link.h"
#include "venc_frame_ring.h"

#include <stdint.h>

typedef struct {
    ar8030_link_t *link; /* already-connected; bitrate_ctl only calls bb_ioctl on it.
                           * The pointer itself never changes, but its ->dev field can
                           * (tx/main.c's read loop reconnects to the daemon after
                           * detecting it died -- ar8030_link_reconnect_retry()) --
                           * every access goes through __atomic_*(), see ar8030_link_t's
                           * own header comment. */
    bb_slot_e slot;       /* which slot's TX MCS to read (this side's own data slot).
                           * Accessed via __atomic_*(): tx/main.c updates this after a
                           * reconnect if resolve_connected_slot() finds the peer on a
                           * different slot than before. */

    const char *waybeam_host; /* default "127.0.0.1" */
    int waybeam_port;         /* default 80 */

    double margin;           /* applied to BB_GET_MCS throughput before clamping, e.g. 0.70 */
    uint32_t min_kbps;
    uint32_t max_kbps;
    double hysteresis;        /* fractional change required to re-apply, e.g. 0.05 */
    int min_interval_ms;      /* rate limit between /api/v1/set calls */
    int poll_interval_ms;     /* safety-net poll period regardless of events */

    /* Second, faster backoff signal alongside the MCS-derived target
     * above, mirroring the stock vendor streamer's own dual-signal
     * design (reverse-engineered from ar_ldyhs_sky's
     * fpv_video_buffer_cache_monitor / fpv_video_send_thread): MCS says
     * what the radio should currently be able to carry, but says nothing
     * about whether THIS side's own bb_socket_write() pipeline is
     * actually draining frames fast enough -- that's a local congestion
     * signal MCS can't see. waybeam's frame-shm ring already measures and
     * publishes exactly this (venc_frame_ring.h's low_water_slots: the
     * ring's lowest occupancy over each ~200ms window, published by the
     * producer specifically for a co-located rate controller to read) --
     * see its header comment for why this is low-water rather than a
     * naive high-water fill_pct check (short healthy bursts routinely
     * spike fill_pct with nothing wrong; low-water asks whether the ring
     * ever failed to drain, which is what actually distinguishes standing
     * backlog from a normal burst). ring may be NULL to disable this
     * entirely (falls back to MCS-only behavior).
     *
     * Owned by the caller (tx/main.c), not this file -- read via
     * __atomic_load_n() every tick rather than cached, since the caller's
     * own read loop replaces it with a freshly-reattached ring after
     * detecting waybeam restarted (see main.c's stat_named_shm()), and
     * this thread would otherwise keep reading a stale, orphaned mapping
     * forever after the first such restart. */
    venc_frame_ring_t *ring;
    uint32_t ring_backlog_high_slots; /* >= this many low-water slots is standing
                                        * backlog per venc_frame_ring.h's own doc
                                        * comment (<=1 is healthy); e.g. 2 */
    double ring_backoff;              /* multiplies the last-applied bitrate when
                                        * backlogged, e.g. 0.85 -- applied instead
                                        * of (never above) the MCS-derived target */

    /* Centre-priority ROI (waybeam's fpv.roiEnabled) is a last-resort
     * measure, not a response to every backlog blip -- intended for when
     * the link itself has already forced bitrate down near the floor and
     * we need to squeeze what little there is toward the part of frame a
     * pilot is actually looking at, not a routine reaction to a
     * keyframe-sized burst at an otherwise-comfortable bitrate (which
     * would just pay ROI's own ~1.4x overshoot risk for nothing -- see
     * bitrate_ctl.c's own comment at the call site). Gated on *both*
     * signals: engages only when a standing ring backlog (the same
     * condition that drives ring_backoff above) is observed while
     * last_applied_kbps is already below roi_max_kbps, e.g. 3000 (this
     * project's own "below 2-4Mbit/s" call); disengages once backlog has
     * been clear *and* last_applied_kbps has recovered back above
     * roi_max_kbps for roi_recovery_ms, e.g. 5000 -- hysteresis against
     * flapping on/off right at the boundary. */
    uint32_t roi_max_kbps;
    int roi_recovery_ms;

    /* Third, independent backoff signal alongside MCS throughput and ring
     * backlog above: BB_GET_USER_QUALITY's LDPC block-error ratio
     * (ldpc_err/ldpc_num) on this side's own physical user. This is the
     * closest signal already available to this project (no unverified/
     * unregistered opcode needed -- see baseband-firmware-analysis doc)
     * to the vendor streamer's own fpv_bb_is_send_retx_too_many() radio-
     * repair-pressure check, which stock uses to trigger exactly this
     * kind of bitrate cut. Bypasses hysteresis like the ring path (an
     * LDPC error burst is itself the kind of thing worth reacting to
     * immediately), with its own short rate limit -- see bitrate_ctl.c's
     * read site. 0 disables this path entirely (falls back to MCS+ring-
     * only behavior, matching how ring == NULL already disables that
     * path). */
    double ldpc_ratio_high; /* e.g. 0.10 -- ldpc_err/ldpc_num at/above this trips the backoff */
    double ldpc_backoff;    /* multiplies last_applied_kbps when tripped, e.g. 0.85 */

    /* Fourth backoff signal: BB_EVENT_RETX_TOO_MANY, the actual event the
     * vendor streamer subscribes to and polls via its own
     * fpv_bb_is_send_retx_too_many() before cutting bitrate -- this is
     * the real mechanism the ldpc_* fields above were originally only a
     * proxy for (LDPC ratio was the closest ALREADY-REGISTERED signal at
     * the time; this event id itself was missing from the SDK's own
     * bb_event_e entirely and had to be independently recovered via
     * Ghidra decompile, tracing fpv_bb_is_send_retx_too_many() back to
     * its actual trigger -- see BB_EVENT_RETX_TOO_MANY's own doc comment
     * in bb_api.h). Deliberately NOT parsed for payload: bb_event_callback's
     * own doc comment says `arg` is event-specific, but this event's
     * payload layout is only known second-hand (the vendor app's own
     * local re-derivation after its own internal IPC indirection, not a
     * directly-confirmed `arg` layout for THIS SDK's own callback
     * delivery) -- so the callback here only bumps a counter, exactly
     * like the existing BB_EVENT_MCS_CHANGE/BB_EVENT_LINK_STATE
     * subscriptions already do, and the main loop treats every firing as
     * the same conservative "radio is actively under repair pressure"
     * hint regardless of whatever gating condition the vendor's own
     * payload parsing might apply. Bypasses hysteresis like the ring/LDPC
     * paths, with its own rate limit. 0 disables this path entirely. */
    double retx_event_backoff; /* multiplies last_applied_kbps on every event firing, e.g. 0.85 */

    const volatile int *stop_flag;
} bitrate_ctl_cfg_t;

/* Runs the control loop until *cfg->stop_flag becomes non-zero. Meant to
 * be the body of its own pthread (blocks). Returns 0 on a clean stop,
 * -1 if the initial event subscription failed (the caller may still
 * choose to run without it -- see tx/main.c). */
int bitrate_ctl_run(const bitrate_ctl_cfg_t *cfg);

#endif /* AR8030_TRANSPORT_TX_BITRATE_CTL_H */
