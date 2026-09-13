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
    bb_get_mcs_in_t in;
    bb_get_mcs_out_t out;
    memset(&in, 0, sizeof(in));
    memset(&out, 0, sizeof(out));
    in.dir = BB_DIR_TX;
    in.slot = (uint8_t)slot;

    int ret = bb_ioctl(link->dev, BB_GET_MCS, &in, &out);
    if (ret) {
        fprintf(stderr, "bitrate_ctl: BB_GET_MCS failed (ret=%d)\n", ret);
        return -1;
    }

    *out_kbps = out.throughput;
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

int bitrate_ctl_run(const bitrate_ctl_cfg_t *cfg)
{
    bb_set_event_callback_t sub_mcs;
    memset(&sub_mcs, 0, sizeof(sub_mcs));
    sub_mcs.event = BB_EVENT_MCS_CHANGE;
    sub_mcs.callback = on_link_event;
    sub_mcs.user = NULL;
    int sub_ret = bb_ioctl(cfg->link->dev, BB_SET_EVENT_SUBSCRIBE, &sub_mcs, NULL);
    if (sub_ret) {
        fprintf(stderr,
                "bitrate_ctl: BB_SET_EVENT_SUBSCRIBE(MCS_CHANGE) failed (ret=%d) -- "
                "falling back to poll-only\n",
                sub_ret);
    }

    bb_set_event_callback_t sub_link;
    memset(&sub_link, 0, sizeof(sub_link));
    sub_link.event = BB_EVENT_LINK_STATE;
    sub_link.callback = on_link_event;
    sub_link.user = NULL;
    bb_ioctl(cfg->link->dev, BB_SET_EVENT_SUBSCRIBE, &sub_link, NULL);

    uint32_t last_applied_kbps = 0;
    int have_applied = 0;
    uint64_t last_apply_ms = 0;
    sig_atomic_t last_seen_wake = 0;
    int roi_enabled = 0;
    uint64_t last_backlog_ms = 0;
    uint64_t last_roi_disable_attempt_ms = 0;

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
         * doesn't re-issue the same HTTP call every single tick. */
        if (cfg->ring && have_applied && (now - last_apply_ms) >= (uint64_t)RING_BACKLOG_MIN_INTERVAL_MS) {
            uint16_t backlog_slots = __atomic_load_n(&cfg->ring->hdr->low_water_slots, __ATOMIC_RELAXED);
            if (backlog_slots >= cfg->ring_backlog_high_slots) {
                last_backlog_ms = now;

                uint32_t backlog_target = clamp_u32((uint32_t)((double)last_applied_kbps * cfg->ring_backoff),
                                                      cfg->min_kbps, cfg->max_kbps);
                int cut_bitrate = backlog_target < last_applied_kbps;

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
                int want_roi = last_applied_kbps < cfg->roi_max_kbps;

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
                            last_applied_kbps = backlog_target;
                            last_apply_ms = now;
                        }
                        if (want_roi)
                            roi_enabled = 1;
                    } else {
                        fprintf(stderr,
                                "bitrate_ctl: waybeam %s returned status=%d (URGENT backlog=%u slots)\n",
                                path, status, backlog_slots);
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

        int poll_due = (now - last_apply_ms) >= (uint64_t)cfg->poll_interval_ms || !have_applied;
        int woken = g_wake != last_seen_wake;
        last_seen_wake = g_wake;

        if (!poll_due && !woken)
            continue;

        uint32_t link_kbps;
        if (read_tx_throughput_kbps(cfg->link, cfg->slot, &link_kbps) != 0)
            continue;

        uint32_t target_kbps = (uint32_t)((double)link_kbps * cfg->margin);
        target_kbps = clamp_u32(target_kbps, cfg->min_kbps, cfg->max_kbps);

        if (have_applied) {
            double delta =
                (double)(target_kbps > last_applied_kbps ? target_kbps - last_applied_kbps
                                                           : last_applied_kbps - target_kbps) /
                (double)last_applied_kbps;
            if (delta < cfg->hysteresis)
                continue;
        }

        if (have_applied && (now - last_apply_ms) < (uint64_t)cfg->min_interval_ms)
            continue;

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
        char path[128];
        snprintf(path, sizeof(path), "/api/v1/live/set?video0.bitrate=%u", target_kbps);
        int status = http_get_status(cfg->waybeam_host, cfg->waybeam_port, path, 1000);
        if (status < 200 || status >= 300) {
            fprintf(stderr, "bitrate_ctl: waybeam %s returned status=%d (link=%u kbps, target=%u kbps)\n",
                    path, status, link_kbps, target_kbps);
            continue; /* try again next tick rather than pinning last_applied to an unapplied value */
        }

        fprintf(stderr, "bitrate_ctl: link=%u kbps -> video0.bitrate=%u kbps\n", link_kbps, target_kbps);
        last_applied_kbps = target_kbps;
        have_applied = 1;
        last_apply_ms = now;
    }

    return 0;
}
