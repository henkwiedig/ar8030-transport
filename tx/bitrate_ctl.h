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
    int min_interval_ms;      /* minimum time between two increases (decreases are never
                               * held back by this -- see the MCS path in bitrate_ctl.c) */
    int poll_interval_ms;     /* BB_GET_MCS poll period, on top of the event wake hint */

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

    /* Focus mode: centre-priority ROI (waybeam's fpv.roiEnabled) once the
     * link has forced the bitrate below roi_max_kbps, e.g. 3000 -- the
     * centre of the frame, where a pilot looks, keeps more detail at the
     * expense of the edges. 0 = never touch fpv.roiEnabled. Needs waybeam
     * 0.84.2 or later: earlier versions overran the bitrate target by
     * multiples with ROI on.
     *
     * PI mode: on as soon as the applied rate is below roi_max_kbps, off
     * once it has stayed at or above it for roi_recovery_ms, e.g. 5000.
     * Rules mode: on only while a standing ring backlog is observed below
     * roi_max_kbps; off once backlog has been clear and the rate back
     * above it for roi_recovery_ms. */
    uint32_t roi_max_kbps;
    int roi_recovery_ms;

    /* Frame-rate stages below the bitrate floor (waybeam's
     * video0.fpsDivisor, CV610, waybeam 0.85.0 or later). waybeam's floor
     * is per frame -- encoding every Nth frame it is floor / N -- so once
     * the PI wants less than the floor, the encoder takes every 2nd frame
     * and the floor halves, down to every fps_div_max-th frame: 1 (off), 2
     * or 4. A frame keeps the size it had at the floor, so the only cost
     * is the longer frame interval, paid only where the link cannot carry
     * the full frame rate's floor -- where the queue would otherwise grow
     * and frames be dropped. A stage up once the PI has wanted at least
     * 1.25 x that stage's floor for fps_recovery_ms. k's lower bound
     * becomes pi_k_min / fps_div_max, so the PI can ask for those rates.
     * PI mode only. */
    int fps_div_max;
    int fps_recovery_ms;

    /* Stepwise ramp-up: any increase toward the MCS-derived target is
     * capped at last_applied * (1 + ramp_step) per apply, instead of
     * jumping straight there (which overshot, backlogged, and got cut
     * again -- a bang-bang sawtooth). 0 disables (jump to target, the old
     * behaviour). Decreases are never stepped -- cutting is always
     * immediate. Exception: right after a fade (target >= 2x the current
     * rate) the first increase jumps to 60% of the target.
     *
     * ramp_settle_ms: no increase until the ring has shown no backlog,
     * and no cut from any path has happened, for this long.
     *
     * After an URGENT ring cut, increases are additionally held below
     * probe_ceiling_frac * (the bitrate that just backlogged) for
     * probe_hold_ms, then allowed to probe above it again (the link's
     * real ceiling drifts with conditions, so it is never learned
     * permanently). */
    double ramp_step;
    int ramp_settle_ms;

    /* While the link is unstable -- its throughput jumped up >= 1.5x and
     * fell back >= 1.5x within unstable_window_ms, less than
     * unstable_window_ms ago -- increases use unstable_ramp_step instead
     * of ramp_step and skip the recovery jump. A fade (falling, then
     * rising once) is not a swing, so recoveries stay fast. 0 disables. */
    int unstable_window_ms;
    double unstable_ramp_step;
    double probe_ceiling_frac;
    int probe_hold_ms;

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
     * in bb_api.h). The firmware raises it on every busy/conti edge,
     * recoveries included; the callback decodes `arg`
     * (common/ar8030_retx_event.h) and counts only the edges where
     * busy_on or conti_on rose. A payload that does not decode counts
     * every firing.
     * Bypasses hysteresis like the ring/LDPC paths, with its own rate
     * limit. 0 disables this path entirely. */
    double retx_event_backoff; /* multiplies last_applied_kbps on every counted event, e.g. 0.85 */

    /* Severity: a deep fade cuts harder than a marginal one. An LDPC
     * error ratio >= ldpc_severe_ratio, or >= retx_severe_events
     * RETX_TOO_MANY events within 1 s, multiply by severe_backoff instead
     * of ldpc_backoff / retx_event_backoff. 0 disables each trigger. */
    double ldpc_severe_ratio;  /* e.g. 0.5 */
    int retx_severe_events;    /* e.g. 3 */
    double severe_backoff;     /* e.g. 0.4 */

    /* BITRATE_CTL_MODE_RULES (default) or BITRATE_CTL_MODE_PI, see
     * bitrate_ctl.c. The PI mode uses margin, min/max_kbps,
     * poll_interval_ms, ring/ring_backlog_high_slots/ring_backoff,
     * ldpc_ratio_high, ldpc_severe_ratio, roi_max_kbps/roi_recovery_ms,
     * fps_div_max/fps_recovery_ms and the pi_* fields below; the
     * ramp/settle/unstable/backoff factors are rules-mode only. */
    int mode;

    /* Link-side queueing delay, written by tx/main.c's send loop
     * (__atomic): EWMA of (frames waiting in the ring x frame interval +
     * time the radio took to accept the frame) in us -- encoder latency
     * excluded -- and when the last frame finished (CLOCK_MONOTONIC ms).
     * PI mode only. */
    uint32_t delay_us;
    uint64_t last_tx_done_ms;

    /* Bumped by tx/main.c (__atomic) each time it reattaches to the ring
     * of a restarted waybeam. waybeam comes back with its configured
     * bitrate and fpv.roiEnabled and at the full frame rate, so PI mode
     * sends all three again. */
    uint32_t ring_generation;

    double pi_delay_set_ms;  /* queueing-delay setpoint, e.g. 10 */
    double pi_kp_up, pi_kp_down; /* proportional gain per ms of error */
    double pi_ki_up, pi_ki_down; /* integral gain per ms of error per s */
    double pi_k_min, pi_k_max;   /* bounds of k (rate = C * margin * k) */
    double pi_slew_up;           /* max relative increase per second, e.g. 1.5 */
    double pi_severe_k;          /* integrator clamp on a severe LDPC/RETX burst, e.g. 0.4 */
    int pi_retx_severe_events;   /* RETX_TOO_MANY within 1 s that count as severe in PI
                                  * mode (retx_severe_events is the rules mode's), e.g. 6 */
    double pi_mild_trim;         /* integrator trim on a mild one, e.g. 0.9 */
    double pi_recover_per_s;     /* min climb of k per s, 1 s after the last disturbance
                                  * with the queue below setpoint, e.g. 0.3; 0 = off */

    const volatile int *stop_flag;
} bitrate_ctl_cfg_t;

#define BITRATE_CTL_MODE_RULES 0
#define BITRATE_CTL_MODE_PI    1

/* Runs the control loop until *cfg->stop_flag becomes non-zero. Meant to
 * be the body of its own pthread (blocks). Returns 0 on a clean stop,
 * -1 if the initial event subscription failed (the caller may still
 * choose to run without it -- see tx/main.c). */
int bitrate_ctl_run(const bitrate_ctl_cfg_t *cfg);

/* Non-zero while the link is congested: within ~1.5 s of any bitrate cut
 * or of an observed ring backlog. idr_ctrl suppresses keyframe requests
 * then -- a keyframe is the largest burst the encoder makes, the GOP
 * brings one within a second anyway, and forcing extra ones into a link
 * that is already backlogged is what turned the 2026-10-02 fade into
 * seconds of frozen, jittering video. Callable from any thread. */
int bitrate_ctl_congested(void);

#endif /* AR8030_TRANSPORT_TX_BITRATE_CTL_H */
