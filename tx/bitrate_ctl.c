#include "bitrate_ctl.h"

#include "http_get.h"

#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/* Bumped by the MCS/link-state event callbacks purely as a "check sooner"
 * hint for the poll loop below -- see the header comment on why the poll
 * loop, not the callback, is the source of truth. Callbacks documented as
 * "synchronous locally" (bb_api.h) so this must stay signal-safe-cheap:
 * no I/O, no locking, just a counter bump. */
static volatile sig_atomic_t g_wake;

static void on_link_event(void *arg, void *user)
{
    (void)arg;
    (void)user;
    g_wake++;
}

/* Separate counter from g_wake above -- this one drives an immediate,
 * rate-limited backoff in the main loop (see the ldpc/ring paths' own
 * shape), not just a "check sooner" hint for the ordinary MCS-driven
 * poll. Deliberately does not touch `arg`: see bitrate_ctl.h's own
 * comment on retx_event_backoff for why this event's payload layout
 * isn't trusted. Same signal-safe-cheap constraint as on_link_event. */
static volatile sig_atomic_t g_retx_too_many;

static void on_retx_too_many_event(void *arg, void *user)
{
    (void)arg;
    (void)user;
    g_retx_too_many++;
}

static uint64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000u + (uint64_t)ts.tv_nsec / 1000000u;
}

/* BB_GET_MCS's struct doc-comments in bb_api.h call this "BB_GET_TX_MCS"
 * but the actual request macro and struct names are BB_GET_MCS /
 * bb_get_mcs_in_t / bb_get_mcs_out_t -- a naming inconsistency in the
 * vendor header, not ours; using the macro/struct names that actually
 * compile. */
static int read_tx_throughput_kbps(ar8030_link_t *link, bb_slot_e slot, uint32_t *out_kbps)
{
    /* Atomic load, not a plain link->dev -- tx/main.c's own read loop may
     * be mid-reconnect (ar8030_link_reconnect_retry(), after detecting
     * the daemon itself died) concurrently with this thread's tick. See
     * ar8030_link_t's own header comment for why every access to this
     * field goes through __atomic_*(). NULL is a normal, expected state
     * here for the duration of a reconnect, not an error. */
    bb_dev_handle_t *dev = __atomic_load_n(&link->dev, __ATOMIC_ACQUIRE);
    if (!dev)
        return -1;

    bb_get_mcs_in_t in;
    bb_get_mcs_out_t out;
    memset(&in, 0, sizeof(in));
    memset(&out, 0, sizeof(out));
    in.dir = BB_DIR_TX;
    in.slot = (uint8_t)slot;

    int ret = bb_ioctl(dev, BB_GET_MCS, &in, &out);
    if (ret) {
        fprintf(stderr, "bitrate_ctl: BB_GET_MCS failed (ret=%d)\n", ret);
        return -1;
    }

    *out_kbps = out.throughput;
    return 0;
}

/* Physical-user index hardcoded to 0, matching this project's own existing
 * single-user assumption elsewhere -- multi-user setups aren't this
 * project's target configuration. Returns -1 (leave *out_ratio untouched)
 * on an RPC failure or an invalid reading (ldpc_num == 0, matching
 * bb_quality_t's own "all zero means invalid" doc comment in bb_api.h) so
 * a momentarily-unpopulated reading is never misread as a perfect 0%
 * error ratio. */
static int read_ldpc_ratio(ar8030_link_t *link, double *out_ratio)
{
    bb_dev_handle_t *dev = __atomic_load_n(&link->dev, __ATOMIC_ACQUIRE);
    if (!dev)
        return -1;

    bb_get_user_quality_in_t in;
    bb_get_user_quality_out_t out;
    memset(&in, 0, sizeof(in));
    memset(&out, 0, sizeof(out));
    in.user_bmp = 0xffff;
    in.average = 0;

    if (bb_ioctl(dev, BB_GET_USER_QUALITY, &in, &out))
        return -1;

    bb_quality_t *q = &out.qualities[0];
    if (!q->ldpc_num)
        return -1;

    *out_ratio = (double)q->ldpc_err / (double)q->ldpc_num;
    return 0;
}

static uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

/* venc_frame_ring.h's low_water_slots is republished by the producer at
 * most once per ~200ms window (VENC_RING_LOW_WATER_WINDOW_US) -- reading
 * it more often than that just re-observes the same value, so a backlog
 * apply is still rate-limited, just on a much shorter leash than the
 * normal MCS path's min_interval_ms (that one is about not spamming
 * waybeam's HTTP API on ordinary MCS jitter; this one is a "the pipe is
 * backing up right now" alarm and should react close to as fast as the
 * signal itself updates). */
#define RING_BACKLOG_MIN_INTERVAL_MS 250

/* ROI safety net: if the ring is still backlogged this long after ROI was
 * switched on, ROI is making it worse -- switch it off and never back on
 * for the rest of the process. Bench walk 2026-10-02: ROI on at the
 * 1058 kbps floor made the encoder emit ~23 Mbit/s; the ring sat at 100%
 * with ~110 ms frame age and 804 dropped frames, and nothing could undo
 * it -- the ROI-off path waits for 5 s of clear backlog, which ROI itself
 * prevented, while cuts were pinned at the floor and increases held off by
 * the very same backlog. */
#define ROI_BACKLOG_ABORT_MS 2000

/* After an URGENT cut, ignore further backlog for this long. A cut only
 * takes effect once the encoder has emitted frames at the new rate and
 * low_water_slots (itself a ~200ms window) has drained the frames that
 * were already queued, so re-reading backlog every
 * RING_BACKLOG_MIN_INTERVAL_MS right after a cut sees the *old* burst
 * again and cuts a second, third, fourth time for one event -- confirmed
 * live: 25681 -> 21828 -> 18553 -> 15770 -> 13404 kbps within ~1s. One
 * burst, one cut. */
#define RING_CUT_HOLDOFF_MS 1500

/* Same rate-limit class as RING_BACKLOG_MIN_INTERVAL_MS above -- this is
 * an RPC (BB_GET_USER_QUALITY), not a shm read, but the same reasoning
 * applies: an LDPC error burst is an "actively under repair pressure"
 * alarm and should react close to as fast as the underlying baseband
 * counters actually update, not be gated behind the ordinary MCS path's
 * min_interval_ms (which exists to avoid spamming waybeam on routine MCS
 * jitter, not on this).
 *
 * This briefly ran at 1000ms as a workaround for a real bug: the
 * vendor SDK's own bb_quality_t was declared 8 bytes but the real
 * per-entry wire size is 16, so every call logged a client-library
 * "reply datalen... truncating" warning -- confirmed harmless to the
 * data actually read (qualities[0] was always correct), but loud at
 * high frequency. That's now fixed at the source
 * (0030-bb_api-fix-bb_quality_t-real-size.patch corrects bb_quality_t's
 * declared size to match, so libar8030_client.so no longer truncates
 * this reply at all), so this runs at the same fast cadence as the ring
 * path again -- see this project's own SDK patch series. */
#define LDPC_BACKOFF_MIN_INTERVAL_MS 250

/* Same rate-limit class as LDPC_BACKOFF_MIN_INTERVAL_MS above -- see
 * bitrate_ctl.h's own comment on retx_event_backoff. */
#define RETX_EVENT_BACKOFF_MIN_INTERVAL_MS 250

/* The multiplicative fast paths (RETX/LDPC) don't stack on top of a cut
 * that hasn't taken effect yet: the encoder needs a few frames at the new
 * rate before the radio's repair pressure can drop, so re-reading the
 * same pressure 250 ms later and cutting again just compounds one event
 * into several (flight 2026-10-02: 7.4 -> 6.3 -> 5.3 -> ... Mbit/s at
 * ~1 cut/s while the MCS-derived target that would have cut straight to
 * what the link carries was starved -- see the MCS path below). */
#define FAST_CUT_HOLDOFF_MS 500

/* A drop of the MCS-derived target is applied as a direct cut on this
 * short leash of its own -- not behind cfg->min_interval_ms (which now
 * only paces increases), and not behind the fast paths' last cut either:
 * a RETX storm cutting every FAST_CUT_HOLDOFF_MS would otherwise keep
 * re-arming that gate and starve this path again (seen in a host replay
 * of the 2026-10-02 fade: stepping down by 15% per 500 ms for 6 s
 * instead of one cut to what the link carries). */
#define MCS_CUT_MIN_INTERVAL_MS 250

/* "Congested" (see bitrate_ctl_congested()) lasts this long after any
 * cut or observed ring backlog. */
#define CONGESTED_HOLD_MS 1500

/* Recovery jump: when the MCS-derived target is at least this many times
 * the current rate (a fade just ended), the first increase goes straight
 * to RECOVERY_JUMP_FRAC of the target instead of one ramp_step -- from
 * ~1 Mbit/s, +25% steps alone would still need ~10 steps to reach a
 * 10 Mbit/s link. Not all the way: jumping straight to the full target
 * overshot and backlogged before (see bitrate_ctl.h on ramp_step). */
#define RECOVERY_JUMP_RATIO 2.0
#define RECOVERY_JUMP_FRAC  0.6

/* Link throughput change that counts as a jump for swing detection. */
#define LINK_SWING_FACTOR 1.5

/* RETX_TOO_MANY events counted over this window for the severe cut
 * (cfg->retx_severe_events), and how many timestamps are kept for it. */
#define RETX_SEVERE_WINDOW_MS 1000
#define RETX_EVENT_HISTORY    16

static uint64_t g_congested_until_ms; /* CLOCK_MONOTONIC ms, __atomic access */

/* waybeam refuses video0.bitrate below its own floor with HTTP 409 (a
 * deliberate, configurable safety net: below ~1 Mbit/s the encoder can't
 * hold a steady rate). cfg->min_kbps doesn't know that floor. A 409 for
 * `rejected` kbps proves the floor is above it, so the floor becomes
 * rejected + 1. Learned per process, so a changed waybeam config is
 * picked up on the next start.
 *
 * (The first version took the *currently applied* rate as the floor.
 * That was harmless with 15% cuts, which only ever asked for a little
 * below the real floor, but a severe 40% cut from 1594 kbps asked for
 * 637, got 409 and pinned the floor at 1594 for the rest of the run --
 * caught in a host replay of the 2026-10-03 10:45 fade.) */
static uint32_t learn_floor(uint32_t floor_kbps, int status, uint32_t rejected_kbps)
{
    if (status == 409 && rejected_kbps + 1 > floor_kbps)
        return rejected_kbps + 1;
    return floor_kbps;
}

/* Applies a cut to *kbps from current_kbps (which waybeam accepted). On a
 * 409 the floor is learned and the request retried halfway between the
 * refused value and current_kbps, a few times: bisects onto waybeam's
 * real floor instead of leaving the bitrate where it was. On success
 * *kbps holds what was actually applied. Returns the last HTTP status. */
static int apply_cut(const bitrate_ctl_cfg_t *cfg, uint32_t *kbps, uint32_t current_kbps, uint32_t *floor_kbps)
{
    int status = 0;
    for (int attempt = 0; attempt < 5; attempt++) {
        char path[128];
        snprintf(path, sizeof(path), "/api/v1/live/set?video0.bitrate=%u", *kbps);
        status = http_get_status(cfg->waybeam_host, cfg->waybeam_port, path, 1000);
        if (status != 409)
            return status;
        uint32_t floor_before = *floor_kbps;
        *floor_kbps = learn_floor(*floor_kbps, status, *kbps);
        if (*floor_kbps != floor_before)
            fprintf(stderr, "bitrate_ctl: waybeam refused %u kbps (409) -> floor >= %u kbps\n", *kbps,
                    *floor_kbps);
        if (*kbps >= current_kbps || current_kbps - *kbps <= 32)
            return status;
        *kbps += (current_kbps - *kbps) / 2;
    }
    return status;
}

static void mark_congested(uint64_t now)
{
    __atomic_store_n(&g_congested_until_ms, now + CONGESTED_HOLD_MS, __ATOMIC_RELEASE);
}

int bitrate_ctl_congested(void)
{
    return now_ms() < __atomic_load_n(&g_congested_until_ms, __ATOMIC_ACQUIRE);
}

static void subscribe_events(const bitrate_ctl_cfg_t *cfg)
{
    /* Loaded once here via the same atomic accessor the rest of this file
     * uses (see ar8030_link_t's own header comment) even though nothing
     * can be mid-reconnect this early in practice -- cheap, and removes
     * any doubt about a pthread_create() race against an implausibly
     * fast first daemon failure.
     *
     * Known, accepted gap: these two subscriptions are tied to *this*
     * dev handle and are never re-established after a mid-session
     * ar8030_link_reconnect_retry() (see tx/main.c) swaps it for a new
     * one -- this thread silently falls back to poll-only cadence
     * (bounded by poll_interval_ms, not a correctness issue) for the
     * remainder of the process after the first daemon restart. Not worth
     * the added coupling to fix given the fallback already exists and
     * degrades gracefully; revisit if poll-only turns out to be too slow
     * in practice post-reconnect. */
    bb_dev_handle_t *dev = __atomic_load_n(&cfg->link->dev, __ATOMIC_ACQUIRE);

    bb_set_event_callback_t sub_mcs;
    memset(&sub_mcs, 0, sizeof(sub_mcs));
    sub_mcs.event = BB_EVENT_MCS_CHANGE;
    sub_mcs.callback = on_link_event;
    sub_mcs.user = NULL;
    int sub_ret = dev ? bb_ioctl(dev, BB_SET_EVENT_SUBSCRIBE, &sub_mcs, NULL) : -1;
    if (sub_ret) {
        fprintf(stderr,
                "bitrate_ctl: BB_SET_EVENT_SUBSCRIBE(MCS_CHANGE) failed (ret=%d) -- "
                "falling back to poll-only\n",
                sub_ret);
    }

    if (dev) {
        bb_set_event_callback_t sub_link;
        memset(&sub_link, 0, sizeof(sub_link));
        sub_link.event = BB_EVENT_LINK_STATE;
        sub_link.callback = on_link_event;
        sub_link.user = NULL;
        bb_ioctl(dev, BB_SET_EVENT_SUBSCRIBE, &sub_link, NULL);
    }

    /* BB_EVENT_RETX_TOO_MANY -- see bitrate_ctl.h's own comment on
     * retx_event_backoff. Best-effort like the two subscriptions above:
     * a failure here (plausible on older firmware that never raises
     * this id) just means this backoff path never fires, not a reason
     * to abort the whole control loop. cfg->retx_event_backoff == 0
     * skips subscribing at all, matching how ring == NULL already skips
     * that path entirely. */
    if (dev && cfg->retx_event_backoff > 0.0) {
        bb_set_event_callback_t sub_retx;
        memset(&sub_retx, 0, sizeof(sub_retx));
        sub_retx.event = BB_EVENT_RETX_TOO_MANY;
        sub_retx.callback = on_retx_too_many_event;
        sub_retx.user = NULL;
        int retx_sub_ret = bb_ioctl(dev, BB_SET_EVENT_SUBSCRIBE, &sub_retx, NULL);
        if (retx_sub_ret) {
            fprintf(stderr,
                    "bitrate_ctl: BB_SET_EVENT_SUBSCRIBE(RETX_TOO_MANY) failed (ret=%d) -- "
                    "this backoff path will never fire\n",
                    retx_sub_ret);
        }
    }
}

/* The rule-based controller (cfg->mode == BITRATE_CTL_MODE_RULES): capacity
 * feedforward plus fixed multiplicative cuts and a stepped ramp. Kept as
 * the default until the PI controller below has flown. */
static int run_rules(const bitrate_ctl_cfg_t *cfg)
{
    uint32_t last_applied_kbps = 0;
    int have_applied = 0;
    uint64_t last_apply_ms = 0;
    sig_atomic_t last_seen_wake = 0;
    sig_atomic_t last_seen_retx_too_many = 0;
    int roi_enabled = 0;
    int roi_banned = 0;          /* set by the ROI safety net, see ROI_BACKLOG_ABORT_MS */
    uint64_t roi_enabled_ms = 0;
    uint64_t last_backlog_ms = 0;
    uint64_t last_urgent_cut_ms = 0;
    uint32_t probe_ceiling_kbps = 0; /* 0 = none */
    uint64_t probe_ceiling_until_ms = 0;
    uint64_t last_roi_disable_attempt_ms = 0;
    uint64_t last_ldpc_apply_ms = 0;
    uint64_t last_retx_event_apply_ms = 0;
    uint64_t last_cut_ms = 0;      /* any decrease, from any path */
    uint64_t last_mcs_cut_ms = 0;  /* MCS-derived decreases only */
    uint32_t prev_link_kbps = 0;   /* previous BB_GET_MCS throughput */
    uint64_t last_link_rise_ms = 0; /* link throughput jumped up (>= LINK_SWING_FACTOR) */
    uint64_t last_link_swing_ms = 0; /* ... and fell back within unstable_window_ms */
    uint32_t floor_kbps = cfg->min_kbps; /* raised by learn_floor()/apply_cut() */
    uint64_t retx_event_ms[RETX_EVENT_HISTORY] = {0}; /* ring of recent RETX_TOO_MANY times */
    unsigned retx_event_pos = 0;
    sig_atomic_t retx_counted = 0;
    uint64_t last_increase_ms = 0; /* paces increases (cfg->min_interval_ms) */
    uint64_t last_mcs_poll_ms = 0; /* paces BB_GET_MCS (cfg->poll_interval_ms) */

    while (!*cfg->stop_flag) {
        usleep(100 * 1000);

        uint64_t now = now_ms();


        /* Ring backlog check: a cheap, always-on shared-memory read (no
         * RPC, unlike BB_GET_MCS), so this runs every ~100ms tick
         * regardless of the MCS poll/event gating below -- a backlog can
         * develop well inside one poll_interval_ms window and this is the
         * only signal that would catch it in time. Bypasses hysteresis
         * entirely (any standing backlog is by definition worth reacting
         * to) but keeps its own short rate limit so a persistent backlog
         * doesn't re-issue the same HTTP call every single tick.
         *
         * cfg->ring is loaded atomically, fresh, every tick: tx/main.c's
         * own read loop swaps it to a newly-reattached ring after
         * detecting waybeam restarted out from under it (see main.c's
         * stat_named_shm() comment), and this thread must not keep
         * reading the old, now-orphaned mapping's low_water_slots forever
         * once that happens. */
        venc_frame_ring_t *ring = __atomic_load_n(&cfg->ring, __ATOMIC_ACQUIRE);
        if (ring && have_applied && (now - last_apply_ms) >= (uint64_t)RING_BACKLOG_MIN_INTERVAL_MS) {
            uint16_t backlog_slots = __atomic_load_n(&ring->hdr->low_water_slots, __ATOMIC_RELAXED);
            if (backlog_slots >= cfg->ring_backlog_high_slots) {
                last_backlog_ms = now;
                mark_congested(now);

                uint32_t backlog_target = clamp_u32((uint32_t)((double)last_applied_kbps * cfg->ring_backoff),
                                                      floor_kbps, cfg->max_kbps);
                int cut_bitrate = backlog_target < last_applied_kbps &&
                                  (last_urgent_cut_ms == 0 ||
                                   (now - last_urgent_cut_ms) >= (uint64_t)RING_CUT_HOLDOFF_MS);

                /* Centre-priority ROI (waybeam's fpv.roiEnabled) is a
                 * last-resort measure, not a routine reaction to this
                 * same backlog signal on its own -- see
                 * bitrate_ctl.h's own comment on roi_max_kbps for why
                 * it's additionally gated on already being squeezed down
                 * near the bitrate floor. A backlog blip at a comfortable
                 * bitrate (a keyframe-sized burst, say) skips ROI
                 * entirely, paying none of its documented ~1.4x
                 * bitrate-overshoot risk for nothing; only once
                 * last_applied_kbps is already below roi_max_kbps does
                 * standing backlog also mean "turn ROI on". Only ever
                 * toggled via /api/v1/live/set (never persisted) at
                 * waybeam's own shipped-default roiQp/roiSteps/roiCenter
                 * -- this codebase sets none of those itself, and that
                 * default is exactly what HTTP_API_CONTRACT.md documents
                 * as calibrated safe (~1.4x overshoot) against CV610's
                 * own default max_qp ceiling, not the 5.8-12x blowup
                 * measured once max_qp is also lowered elsewhere --
                 * something this codebase never does. Any residual
                 * overshoot from turning ROI on is just more backlog,
                 * handled by this same loop on its next tick like any
                 * other overshoot source; no separate compensation needed. */
                if (roi_enabled && (now - roi_enabled_ms) >= (uint64_t)ROI_BACKLOG_ABORT_MS) {
                    int st = http_get_status(cfg->waybeam_host, cfg->waybeam_port,
                                             "/api/v1/live/set?fpv.roiEnabled=false", 1000);
                    fprintf(stderr,
                            "bitrate_ctl: ring still backlogged %llu ms after ROI on -> fpv.roiEnabled=false "
                            "(status=%d), ROI disabled for this run\n",
                            (unsigned long long)(now - roi_enabled_ms), st);
                    if (st >= 200 && st < 300) {
                        roi_enabled = 0;
                        roi_banned = 1;
                    }
                }

                int want_roi = !roi_banned && last_applied_kbps < cfg->roi_max_kbps;

                if (cut_bitrate || (want_roi && !roi_enabled)) {
                    char path[160];
                    if (cut_bitrate)
                        snprintf(path, sizeof(path), "/api/v1/live/set?video0.bitrate=%u%s", backlog_target,
                                 (want_roi && !roi_enabled) ? "&fpv.roiEnabled=true" : "");
                    else
                        snprintf(path, sizeof(path), "/api/v1/live/set?fpv.roiEnabled=true");

                    int status = http_get_status(cfg->waybeam_host, cfg->waybeam_port, path, 1000);
                    if (status >= 200 && status < 300) {
                        fprintf(stderr, "bitrate_ctl: URGENT ring backlog=%u slots -> %s\n", backlog_slots,
                                path);
                        if (cut_bitrate) {
                            if (cfg->probe_ceiling_frac > 0.0) {
                                /* the rate that just backlogged is over the
                                 * link's real ceiling; stay under it for a
                                 * while (see bitrate_ctl.h) */
                                probe_ceiling_kbps =
                                    (uint32_t)((double)last_applied_kbps * cfg->probe_ceiling_frac);
                                probe_ceiling_until_ms = now + (uint64_t)cfg->probe_hold_ms;
                            }
                            last_applied_kbps = backlog_target;
                            last_apply_ms = now;
                            last_urgent_cut_ms = now;
                            last_cut_ms = now;
                        }
                        if (want_roi && !roi_enabled) {
                            roi_enabled = 1;
                            roi_enabled_ms = now;
                        }
                    } else {
                        fprintf(stderr,
                                "bitrate_ctl: waybeam %s returned status=%d (URGENT backlog=%u slots)\n",
                                path, status, backlog_slots);
                        if (cut_bitrate)
                            floor_kbps = learn_floor(floor_kbps, status, backlog_target);
                    }
                }
            } else if (roi_enabled && last_applied_kbps >= cfg->roi_max_kbps &&
                       (now - last_backlog_ms) >= (uint64_t)cfg->roi_recovery_ms &&
                       (now - last_roi_disable_attempt_ms) >= (uint64_t)RING_BACKLOG_MIN_INTERVAL_MS) {
                /* Backlog clear *and* bitrate recovered back above
                 * roi_max_kbps for roi_recovery_ms -- hand quality back
                 * from the centre band to the whole frame. */
                last_roi_disable_attempt_ms = now;
                int status = http_get_status(cfg->waybeam_host, cfg->waybeam_port,
                                              "/api/v1/live/set?fpv.roiEnabled=false", 1000);
                if (status >= 200 && status < 300) {
                    fprintf(stderr,
                            "bitrate_ctl: bitrate recovered to %u kbps, backlog clear for %dms -> "
                            "fpv.roiEnabled=false\n",
                            last_applied_kbps, cfg->roi_recovery_ms);
                    roi_enabled = 0;
                } else {
                    fprintf(stderr, "bitrate_ctl: waybeam fpv.roiEnabled=false returned status=%d\n", status);
                }
            }
        }

        /* LDPC error-ratio backoff: independent of, and checked before,
         * the ordinary MCS-driven path below -- see bitrate_ctl.h's own
         * comment on ldpc_ratio_high/ldpc_backoff for why this exists and
         * why it bypasses hysteresis the same way the ring-backlog path
         * above does. Only ever cuts (never raises) last_applied_kbps;
         * recovery back up happens through the normal MCS-driven path's
         * own next poll once the ratio drops back below threshold, same
         * as the ring path's cut side has no dedicated "undo" either. */
        if (cfg->ldpc_ratio_high > 0.0 && have_applied &&
            (now - last_ldpc_apply_ms) >= (uint64_t)LDPC_BACKOFF_MIN_INTERVAL_MS &&
            (now - last_cut_ms) >= (uint64_t)FAST_CUT_HOLDOFF_MS) {
            /* Set on every attempt, not just a tripped one -- this is what
             * actually enforces LDPC_BACKOFF_MIN_INTERVAL_MS. Setting it
             * only inside the tripped branch below left the healthy-link
             * case (ratio never crosses threshold, the common case)
             * completely unthrottled: confirmed live, this call's own
             * pre-existing "reply datalen... truncating" warning (see
             * read_ldpc_ratio()'s own comment) kept firing every ~100ms
             * tick regardless of this constant's value until fixed here. */
            last_ldpc_apply_ms = now;
            double ldpc_ratio;
            if (read_ldpc_ratio(cfg->link, &ldpc_ratio) == 0 && ldpc_ratio >= cfg->ldpc_ratio_high) {
                /* Severity: a mostly-failing link (flight 2026-10-03
                 * 10:44:57, ratio 100%) got the same 15% trim as a
                 * marginal one, and the fade finished filling the ring
                 * before the MCS path caught up. */
                double factor = (cfg->ldpc_severe_ratio > 0.0 && ldpc_ratio >= cfg->ldpc_severe_ratio)
                                    ? cfg->severe_backoff
                                    : cfg->ldpc_backoff;
                uint32_t ldpc_target = clamp_u32((uint32_t)((double)last_applied_kbps * factor),
                                                  floor_kbps, cfg->max_kbps);
                if (ldpc_target < last_applied_kbps) {
                    int status = apply_cut(cfg, &ldpc_target, last_applied_kbps, &floor_kbps);
                    if (status >= 200 && status < 300) {
                        fprintf(stderr, "bitrate_ctl: LDPC error ratio=%.1f%% -> video0.bitrate=%u kbps%s\n",
                                ldpc_ratio * 100.0, ldpc_target, factor == cfg->severe_backoff ? " (severe)" : "");
                        last_applied_kbps = ldpc_target;
                        last_apply_ms = now;
                        last_cut_ms = now;
                        mark_congested(now);
                    } else {
                        fprintf(stderr, "bitrate_ctl: waybeam video0.bitrate=%u returned status=%d (LDPC ratio=%.1f%%)\n",
                                ldpc_target, status, ldpc_ratio * 100.0);
                    }
                }
            }
        }

        /* BB_EVENT_RETX_TOO_MANY backoff: edge-triggered off g_retx_too_many
         * (set by on_retx_too_many_event(), see bitrate_ctl.h's own
         * comment on retx_event_backoff), not a threshold check like the
         * LDPC path above -- there is no ratio to compare here, just "did
         * this fire since last tick". Rate-limited the same way (own
         * short interval, independent of the ordinary MCS path's
         * min_interval_ms) so a burst of several events in quick
         * succession cuts once, not once per event. Bypasses hysteresis,
         * cut only -- same shape as every other fast-path signal in this
         * file. */
        /* Timestamp every RETX_TOO_MANY event as it is seen, independent
         * of the cut gating below, for the severity count. */
        {
            sig_atomic_t c = g_retx_too_many;
            while (retx_counted != c) {
                retx_counted++;
                retx_event_ms[retx_event_pos++ % RETX_EVENT_HISTORY] = now;
            }
        }
        if (cfg->retx_event_backoff > 0.0 && have_applied &&
            (now - last_retx_event_apply_ms) >= (uint64_t)RETX_EVENT_BACKOFF_MIN_INTERVAL_MS &&
            (now - last_cut_ms) >= (uint64_t)FAST_CUT_HOLDOFF_MS) {
            sig_atomic_t retx_too_many_now = g_retx_too_many;
            if (retx_too_many_now != last_seen_retx_too_many) {
                last_seen_retx_too_many = retx_too_many_now;
                last_retx_event_apply_ms = now;
                int recent = 0;
                for (int i = 0; i < RETX_EVENT_HISTORY; i++)
                    if (retx_event_ms[i] && now - retx_event_ms[i] < (uint64_t)RETX_SEVERE_WINDOW_MS)
                        recent++;
                double factor = (cfg->retx_severe_events > 0 && recent >= cfg->retx_severe_events)
                                    ? cfg->severe_backoff
                                    : cfg->retx_event_backoff;
                uint32_t retx_target = clamp_u32((uint32_t)((double)last_applied_kbps * factor),
                                                  floor_kbps, cfg->max_kbps);
                if (retx_target < last_applied_kbps) {
                    int status = apply_cut(cfg, &retx_target, last_applied_kbps, &floor_kbps);
                    if (status >= 200 && status < 300) {
                        fprintf(stderr, "bitrate_ctl: BB_EVENT_RETX_TOO_MANY -> video0.bitrate=%u kbps%s\n",
                                retx_target, factor == cfg->severe_backoff ? " (severe)" : "");
                        last_applied_kbps = retx_target;
                        last_apply_ms = now;
                        last_cut_ms = now;
                        mark_congested(now);
                    } else {
                        fprintf(stderr, "bitrate_ctl: waybeam video0.bitrate=%u returned status=%d (RETX_TOO_MANY)\n",
                                retx_target, status);
                    }
                }
            }
        }

        /* MCS-derived target: polled on its own timer (plus the event
         * wake hint), not "poll_interval_ms since the last apply" -- with
         * the fast paths above cutting every few hundred ms during a fade,
         * that gate never opened and the link-derived target (the one
         * signal that knows what the radio can carry right now) went
         * unread for 11 s (flight 2026-10-02, 11:53:51). */
        int woken = g_wake != last_seen_wake;
        last_seen_wake = g_wake;
        if (have_applied && !woken && (now - last_mcs_poll_ms) < (uint64_t)cfg->poll_interval_ms)
            continue;
        last_mcs_poll_ms = now;

        /* Atomic load: tx/main.c updates cfg->slot after a reconnect if
         * the peer's connected slot resolves differently than before
         * (see bitrate_ctl.h's own comment on this field). */
        bb_slot_e slot = __atomic_load_n(&cfg->slot, __ATOMIC_ACQUIRE);
        uint32_t link_kbps;
        if (read_tx_throughput_kbps(cfg->link, slot, &link_kbps) != 0)
            continue;

        /* Link swing detection for the cautious ramp: the link's own
         * throughput jumping up and falling back within
         * unstable_window_ms (a yo-yo, flight 2026-10-03 10:37:55-10:38:06:
         * MCS 2 -> 8 -> 2 within seconds). A fade -- falling, then rising
         * once -- is not a swing, so its recovery ramps at full speed. */
        if (prev_link_kbps && link_kbps >= prev_link_kbps * LINK_SWING_FACTOR) {
            last_link_rise_ms = now;
        } else if (prev_link_kbps && link_kbps * LINK_SWING_FACTOR <= prev_link_kbps && last_link_rise_ms &&
                   (now - last_link_rise_ms) < (uint64_t)cfg->unstable_window_ms) {
            last_link_swing_ms = now;
        }
        prev_link_kbps = link_kbps;

        uint32_t target_kbps = (uint32_t)((double)link_kbps * cfg->margin);
        target_kbps = clamp_u32(target_kbps, floor_kbps, cfg->max_kbps);

        if (have_applied) {
            double delta =
                (double)(target_kbps > last_applied_kbps ? target_kbps - last_applied_kbps
                                                           : last_applied_kbps - target_kbps) /
                (double)last_applied_kbps;
            if (delta < cfg->hysteresis)
                continue;
        }

        /* Decreases: straight to the link-derived target, on a short
         * leash of their own -- never behind min_interval_ms. */
        int ramped = 0;
        int cutting = have_applied && target_kbps < last_applied_kbps;
        if (cutting && (now - last_mcs_cut_ms) < (uint64_t)MCS_CUT_MIN_INTERVAL_MS)
            continue;

        /* Increases: paced by min_interval_ms, held off for
         * ramp_settle_ms after any backlog or cut, and stepped (see
         * bitrate_ctl.h) -- except a recovery jump right after a fade. */
        if (have_applied && !cutting) {
            if ((now - last_increase_ms) < (uint64_t)cfg->min_interval_ms)
                continue;
            if ((now - last_backlog_ms) < (uint64_t)cfg->ramp_settle_ms ||
                (now - last_cut_ms) < (uint64_t)cfg->ramp_settle_ms)
                continue;
            /* Unstable link: it swung (see above) within the last
             * unstable_window_ms -- step cautiously and skip the recovery
             * jump. Flight 2026-10-03 10:37:59: +25% steps rode a short
             * MCS 8 peak from 4.5 to 8.8 Mbit/s, the next drop hit a full
             * ring (93 ms frame age). */
            int unstable = last_link_swing_ms && (now - last_link_swing_ms) < (uint64_t)cfg->unstable_window_ms;
            double step = unstable && cfg->unstable_ramp_step > 0.0 ? cfg->unstable_ramp_step : cfg->ramp_step;
            if (step > 0.0) {
                uint32_t step_cap = (uint32_t)((double)last_applied_kbps * (1.0 + step));
                if (!unstable && (double)target_kbps >= (double)last_applied_kbps * RECOVERY_JUMP_RATIO) {
                    uint32_t jump = (uint32_t)((double)target_kbps * RECOVERY_JUMP_FRAC);
                    if (jump > step_cap)
                        step_cap = jump;
                }
                if (step_cap <= last_applied_kbps)
                    step_cap = last_applied_kbps + 1;
                if (target_kbps > step_cap) {
                    target_kbps = step_cap;
                    ramped = 1;
                }
            }
            if (probe_ceiling_kbps && now < probe_ceiling_until_ms && target_kbps > probe_ceiling_kbps) {
                target_kbps = probe_ceiling_kbps;
                ramped = 1;
            }
            if (target_kbps <= last_applied_kbps)
                continue;
        }

        /* /api/v1/live/set, not /api/v1/set: this fires routinely (every
         * MCS change plus the poll safety net) and waybeam's own
         * HTTP_API_CONTRACT.md documents /live/set as exactly the
         * endpoint built for this ("high-cadence automated writers...
         * persist-on-set would wear flash and boot into the last adaptive
         * transient") -- the persisting /api/v1/set this used to call
         * would otherwise write every single adaptive bitrate change to
         * /etc/waybeam.json and, worse, have a crash/reboot right after a
         * link-quality dip boot back up pinned at that low bitrate
         * instead of the configured default. */
        int status;
        if (cutting && have_applied) {
            status = apply_cut(cfg, &target_kbps, last_applied_kbps, &floor_kbps);
        } else {
            char path[128];
            snprintf(path, sizeof(path), "/api/v1/live/set?video0.bitrate=%u", target_kbps);
            status = http_get_status(cfg->waybeam_host, cfg->waybeam_port, path, 1000);
        }
        if (status < 200 || status >= 300) {
            fprintf(stderr, "bitrate_ctl: waybeam video0.bitrate=%u returned status=%d (link=%u kbps)\n",
                    target_kbps, status, link_kbps);
            continue; /* try again next tick rather than pinning last_applied to an unapplied value */
        }

        fprintf(stderr, "bitrate_ctl: link=%u kbps -> video0.bitrate=%u kbps%s\n", link_kbps, target_kbps,
                ramped ? " (ramp)" : "");
        if (cutting) {
            last_cut_ms = now;
            last_mcs_cut_ms = now;
            mark_congested(now);
        } else {
            last_increase_ms = now;
        }
        last_applied_kbps = target_kbps;
        have_applied = 1;
        last_apply_ms = now;
    }

    return 0;
}

/* ---------------------------------------------------------------------
 * PI controller (cfg->mode == BITRATE_CTL_MODE_PI)
 *
 * rate = clamp(C * margin * k, floor, max_kbps), every 100 ms tick:
 *   C  -- link capacity, BB_GET_MCS throughput (feedforward: a drop is
 *         applied at once, a rise is slew-limited);
 *   k  -- PI output on the air-side delay D (capture -> last chunk
 *         written, published by tx/main.c), error e = D_set - D in ms,
 *         asymmetric gains (back off fast, creep up slowly), no D term;
 *   disturbance feedforward: severe LDPC / RETX bursts clamp the
 *         integrator at once, milder ones trim it; ring backlog stays as
 *         a hard backstop.
 * Anti-windup: the integrator doesn't climb while the rate is held by
 * the ceiling or the slew limit, and doesn't sink while at waybeam's
 * floor.
 * ------------------------------------------------------------------- */

#define PI_TICK_MS            100
#define PI_DISTURB_HOLD_MS    500  /* mild LDPC/RETX trims at most this often */
#define PI_CONGESTED_FACTOR   2.0  /* D above this x D_set counts as congested */
#define PI_STALE_MS           150  /* no frame finished this long: D grows with it */
#define PI_MIN_UP_STEP        0.05 /* send increases of at least 5% ... */
#define PI_MIN_DOWN_STEP      0.03 /* ... and decreases of at least 3% */

static int run_pi(const bitrate_ctl_cfg_t *cfg)
{
    uint32_t cap_kbps = 0;           /* last good BB_GET_MCS throughput */
    uint64_t last_mcs_poll_ms = 0;
    sig_atomic_t last_seen_wake = 0;
    double k_i = 1.0;                /* integrator state */
    double k = 1.0;
    uint32_t applied_kbps = 0;       /* last rate waybeam accepted */
    uint64_t last_apply_ms = 0;
    uint32_t floor_kbps = cfg->min_kbps;
    uint64_t last_ldpc_ms = 0, last_disturb_ms = 0, last_severe_ms = 0, last_backlog_ms = 0;
    uint64_t retx_event_ms[RETX_EVENT_HISTORY] = {0};
    unsigned retx_event_pos = 0;
    sig_atomic_t retx_counted = 0, retx_seen = 0;
    uint64_t last_tick_ms = now_ms();

    while (!*cfg->stop_flag) {
        usleep(PI_TICK_MS * 1000);
        uint64_t now = now_ms();
        double dt = (double)(now - last_tick_ms) / 1000.0;
        last_tick_ms = now;

        /* Measured delay; if nothing finished for a while, the frame in
         * flight is at least that old. */
        double d_ms = (double)__atomic_load_n(&cfg->delay_us, __ATOMIC_ACQUIRE) / 1000.0;
        uint64_t done_ms = __atomic_load_n(&cfg->last_tx_done_ms, __ATOMIC_ACQUIRE);
        if (done_ms && now > done_ms + PI_STALE_MS && (double)(now - done_ms) > d_ms)
            d_ms = (double)(now - done_ms);

        /* Capacity (feedforward). */
        int woken = g_wake != last_seen_wake;
        last_seen_wake = g_wake;
        if (!cap_kbps || woken || now - last_mcs_poll_ms >= (uint64_t)cfg->poll_interval_ms) {
            last_mcs_poll_ms = now;
            uint32_t c;
            bb_slot_e slot = __atomic_load_n(&cfg->slot, __ATOMIC_ACQUIRE);
            if (read_tx_throughput_kbps(cfg->link, slot, &c) == 0 && c)
                cap_kbps = c;
        }
        if (!cap_kbps)
            continue;

        /* Disturbance feedforward: the radio says it is losing frames
         * before the delay can show it. */
        {
            sig_atomic_t c = g_retx_too_many;
            while (retx_counted != c) {
                retx_counted++;
                retx_event_ms[retx_event_pos++ % RETX_EVENT_HISTORY] = now;
            }
        }
        int recent_retx = 0;
        for (int i = 0; i < RETX_EVENT_HISTORY; i++)
            if (retx_event_ms[i] && now - retx_event_ms[i] < (uint64_t)RETX_SEVERE_WINDOW_MS)
                recent_retx++;
        int new_retx = retx_seen != retx_counted;
        retx_seen = retx_counted;
        double ldpc = 0.0;
        if (cfg->ldpc_ratio_high > 0.0 && now - last_ldpc_ms >= (uint64_t)LDPC_BACKOFF_MIN_INTERVAL_MS) {
            last_ldpc_ms = now;
            if (read_ldpc_ratio(cfg->link, &ldpc) != 0)
                ldpc = 0.0;
        }
        const char *why = NULL;
        if ((cfg->ldpc_severe_ratio > 0.0 && ldpc >= cfg->ldpc_severe_ratio) ||
            (cfg->retx_severe_events > 0 && recent_retx >= cfg->retx_severe_events)) {
            if (k_i > cfg->pi_severe_k) {
                k_i = cfg->pi_severe_k;
                why = "severe";
            }
            last_severe_ms = last_disturb_ms = now;
        } else if ((ldpc >= cfg->ldpc_ratio_high || new_retx) && now - last_disturb_ms >= PI_DISTURB_HOLD_MS) {
            k_i *= cfg->pi_mild_trim;
            last_disturb_ms = now;
            why = "trim";
        }

        /* Ring backlog: hard backstop, same signal as the rules mode. */
        venc_frame_ring_t *ring = __atomic_load_n(&cfg->ring, __ATOMIC_ACQUIRE);
        if (ring && __atomic_load_n(&ring->hdr->low_water_slots, __ATOMIC_RELAXED) >= cfg->ring_backlog_high_slots &&
            now - last_backlog_ms >= (uint64_t)RING_CUT_HOLDOFF_MS) {
            k_i = (k < k_i ? k : k_i) * cfg->ring_backoff;
            last_backlog_ms = now;
            why = "backlog";
        }

        /* PI on the delay. */
        double e = cfg->pi_delay_set_ms - d_ms;
        double kp = e < 0 ? cfg->pi_kp_down : cfg->pi_kp_up;
        double ki = e < 0 ? cfg->pi_ki_down : cfg->pi_ki_up;
        double base = (double)cap_kbps * cfg->margin;
        int at_floor = applied_kbps && applied_kbps <= floor_kbps;
        int held_up = applied_kbps && (double)applied_kbps < base * k * 0.9; /* ceiling or slew holding it */
        if ((e > 0 && !held_up && applied_kbps < cfg->max_kbps) || (e < 0 && !at_floor))
            k_i += ki * e * dt;
        if (k_i < cfg->pi_k_min)
            k_i = cfg->pi_k_min;
        if (k_i > cfg->pi_k_max)
            k_i = cfg->pi_k_max;
        k = k_i + kp * e;
        if (k < cfg->pi_k_min)
            k = cfg->pi_k_min;
        if (k > cfg->pi_k_max)
            k = cfg->pi_k_max;

        if (d_ms > cfg->pi_delay_set_ms * PI_CONGESTED_FACTOR || (last_severe_ms && now - last_severe_ms < 1500))
            mark_congested(now);

        /* Target: capacity feedforward x k, slew-limited upward. */
        double target = base * k;
        if (target < floor_kbps)
            target = floor_kbps;
        if (target > cfg->max_kbps)
            target = cfg->max_kbps;
        if (applied_kbps && target > applied_kbps) {
            double cap_up = (double)applied_kbps * (1.0 + cfg->pi_slew_up * dt);
            if (target > cap_up)
                target = cap_up;
        }
        uint32_t t = (uint32_t)target;
        if (applied_kbps) {
            double rel = ((double)t - applied_kbps) / applied_kbps;
            int due = now - last_apply_ms >= 1000;
            if (rel > 0 ? (rel < PI_MIN_UP_STEP && !due) : (-rel < PI_MIN_DOWN_STEP && !why))
                continue;
            if (t == applied_kbps)
                continue;
        }

        int status;
        if (applied_kbps && t < applied_kbps) {
            status = apply_cut(cfg, &t, applied_kbps, &floor_kbps);
        } else {
            char path[128];
            snprintf(path, sizeof(path), "/api/v1/live/set?video0.bitrate=%u", t);
            status = http_get_status(cfg->waybeam_host, cfg->waybeam_port, path, 1000);
        }
        if (status < 200 || status >= 300) {
            fprintf(stderr, "bitrate_ctl: pi waybeam video0.bitrate=%u returned status=%d\n", t, status);
            continue;
        }
        fprintf(stderr, "bitrate_ctl: pi C=%u D=%.1fms k=%.2f%s%s -> video0.bitrate=%u kbps\n", cap_kbps, d_ms, k,
                why ? " " : "", why ? why : "", t);
        applied_kbps = t;
        last_apply_ms = now;
    }
    return 0;
}

int bitrate_ctl_run(const bitrate_ctl_cfg_t *cfg)
{
    subscribe_events(cfg);
    return cfg->mode == BITRATE_CTL_MODE_PI ? run_pi(cfg) : run_rules(cfg);
}
