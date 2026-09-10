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

    while (!*cfg->stop_flag) {
        usleep(100 * 1000);

        uint64_t now = now_ms();
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

        char path[128];
        snprintf(path, sizeof(path), "/api/v1/set?video0.bitrate=%u", target_kbps);
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
