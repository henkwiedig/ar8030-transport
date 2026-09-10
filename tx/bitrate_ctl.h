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

#include <stdint.h>

typedef struct {
    ar8030_link_t *link; /* already-connected; bitrate_ctl only calls bb_ioctl on it */
    bb_slot_e slot;       /* which slot's TX MCS to read (this side's own data slot) */

    const char *waybeam_host; /* default "127.0.0.1" */
    int waybeam_port;         /* default 80 */

    double margin;           /* applied to BB_GET_MCS throughput before clamping, e.g. 0.70 */
    uint32_t min_kbps;
    uint32_t max_kbps;
    double hysteresis;        /* fractional change required to re-apply, e.g. 0.05 */
    int min_interval_ms;      /* rate limit between /api/v1/set calls */
    int poll_interval_ms;     /* safety-net poll period regardless of events */

    const volatile int *stop_flag;
} bitrate_ctl_cfg_t;

/* Runs the control loop until *cfg->stop_flag becomes non-zero. Meant to
 * be the body of its own pthread (blocks). Returns 0 on a clean stop,
 * -1 if the initial event subscription failed (the caller may still
 * choose to run without it -- see tx/main.c). */
int bitrate_ctl_run(const bitrate_ctl_cfg_t *cfg);

#endif /* AR8030_TRANSPORT_TX_BITRATE_CTL_H */
