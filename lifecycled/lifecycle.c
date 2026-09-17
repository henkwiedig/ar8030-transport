#include "lifecycle.h"
#include "ar8030.h"
#include "bb_api.h"
#include "lc_log.h"
#include "lifecycle_client.h"
#include "lifecycle_gpio.h"
#include "lifecycle_hooks.h"
#include "lifecycle_pair.h"
#include "lifecycle_tuning.h"
#include <pthread.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/*
 * State machine: INIT -> IDLE -> CONNECTED -> back to IDLE on drop.
 *
 * IDLE never triggers a BB_SET_PRJ_DISPATCH/PRJ_CMD_EVENT_PAIR pairing
 * dispatch itself -- see the long comment in the IDLE case below for why
 * (Ghidra evidence from stock's own ar_ldy_gnd/ar_ldyhs_sky streamers: an
 * already-paired chip's RTOS firmware reconnects to its configured peer
 * entirely on its own; the dispatch call is exclusively the bind button's).
 * This process just waits for that to happen.
 *
 * Link-state tracking is event-driven first (BB_EVENT_LINK_STATE, via
 * lc_client_subscribe_events()), with a permanent BB_GET_STATUS fallback
 * poll every LC_POLL_FALLBACK_S as a safety net -- not a temporary
 * bring-up crutch, kept forever, because a silent mismatch between what
 * the callback reports and what a poll finds is exactly the kind of bug
 * that used to surface as an unexplained hang. Both paths update the same
 * g_link_state under g_link_state_lock; the main loop only ever reads that
 * shared value, never calls bb_ioctl directly to check link state itself.
 */

#define LC_POLL_FALLBACK_S     5
#define LC_ESTABLISH_RETRY_S   3

/* Debounces link-state transitions in BOTH directions -- confirmed live
 * (air and ground) that the chip itself can report CONNECT and then flip
 * straight back to IDLE within single-digit *milliseconds*, with zero
 * host-side action (tuning, hooks) fast enough to have caused it: the
 * BB_EVENT_LINK_STATE callback for the drop fires within 1-5ms of the
 * one for the connect, sometimes before this process has even dispatched
 * a single hook for the connect yet. The chip's own RTOS firmware here
 * is unmodified stock, so this is exactly what decompiling the vendor's
 * own real streamer (ar_ldyhs_sky, ghidra-mcp) found *its* link-state
 * machine already assumes: it never commits to a peer off one positive
 * scan (several consecutive ones first), and never declares a drop
 * immediately either (only through a delayed timer callback). Neither
 * this daemon nor the shell watchdogs it replaces (S65ar8030-transport-tx/
 * S97ar8030's own former wait_while_connected(), which only debounced the
 * *drop* side, never the connect side) fully matched that -- committing
 * to CONNECTED (firing hooks, applying tuning) off a single reading, on a
 * link that hadn't actually stabilized yet, then declaring that same
 * transient blip a real drop and immediately re-pairing (repeating
 * forever) is the flapping loop this whole investigation kept hitting.
 *
 * Both debounces sample the shared g_link_state once per 1s main-loop
 * tick and require several *consecutive* ticks in agreement before
 * committing -- not a re-implementation of the chip's own ~20ms-scale
 * scan debounce (this process has no visibility at that resolution), but
 * enough that a link genuinely flapping every few ms has low odds of
 * accidentally producing several straight same-direction 1s samples,
 * while a link that has actually settled reads the same way every time. */
#define LC_CONNECT_CONFIRM_TICKS 3
#define LC_DROP_CONFIRM_TICKS    6

typedef enum {
    LC_STATE_INIT = 0,
    LC_STATE_IDLE,
    LC_STATE_CONNECTED,
} lc_state_e;

struct lifecycle_ctx {
    lc_config_t cfg;
    lc_client_t client;
    lc_state_e  state;

    /* Set whenever ctx->state transitions to LC_STATE_CONNECTED -- reused
     * both by the drop-detection hook dispatch (which slot dropped) and
     * the periodic tuning re-check below (which slot to apply to). */
    int connected_slot;

    /* Consecutive main-loop ticks (1s each) the link has read as not-
     * CONNECT while in LC_STATE_CONNECTED -- see LC_DROP_CONFIRM_TICKS.
     * Reset to 0 on any CONNECT reading. */
    int drop_miss_count;

    /* Consecutive main-loop ticks (1s each) the link has read as CONNECT
     * while in LC_STATE_IDLE -- every CONNECT this process ever observes
     * arrives this way now (an externally-triggered pair via the bind
     * button, or the chip's own firmware autonomously reconnecting to an
     * already-configured peer) -- see LC_CONNECT_CONFIRM_TICKS. */
    int connect_confirm_count;

    /* Last bandwidth (MHz) this process itself either applied or
     * persisted -- -1 means "not known yet", not "no tuning wanted".
     * Compared against a fresh BB_GET_STATUS reading every fallback-poll
     * tick while connected so a live `ar8030-linkctl bandwidth` change
     * gets captured without needing its own resident watcher. */
    int last_bandwidth;

    /* Fires the "idle" hook (this unit has never been paired) once per
     * process instead of once per second -- the branch that wants it is
     * otherwise checked on every 1s loop iteration. */
    int idle_hook_fired;
};

static volatile sig_atomic_t g_shutdown_requested = 0;

/* Shared link-state, written by both the event callback (client library's
 * own reader thread) and the fallback poll (lifecycle thread itself). */
static pthread_mutex_t g_link_state_lock  = PTHREAD_MUTEX_INITIALIZER;
static bb_link_state_e g_link_state       = BB_LINK_STATE_IDLE;
static int             g_link_state_valid = 0;

void lifecycle_request_shutdown(void)
{
    g_shutdown_requested = 1;
}

int lifecycle_shutdown_requested(void)
{
    return g_shutdown_requested;
}

static void lc_set_link_state(bb_link_state_e state)
{
    pthread_mutex_lock(&g_link_state_lock);
    g_link_state       = state;
    g_link_state_valid = 1;
    pthread_mutex_unlock(&g_link_state_lock);
}

static int lc_get_link_state(bb_link_state_e* out)
{
    int ok;
    pthread_mutex_lock(&g_link_state_lock);
    ok  = g_link_state_valid;
    *out = g_link_state;
    pthread_mutex_unlock(&g_link_state_lock);
    return ok;
}

void lc_on_link_state_event(void* arg, void* user)
{
    (void)user;
    bb_event_link_state_t* evt = (bb_event_link_state_t*)arg;
    if (!evt) {
        return;
    }
    lc_log("lifecycle: link state event slot=%d cur=%d prev=%d", evt->slot, evt->cur_state, evt->prev_state);
    lc_set_link_state((bb_link_state_e)evt->cur_state);
}

void lc_on_pair_result_event(void* arg, void* user)
{
    (void)user;
    bb_event_pair_result_t* evt = (bb_event_pair_result_t*)arg;
    lc_log("lifecycle: pair result event ret=%d", evt ? evt->ret : -1);
}

lifecycle_ctx* lifecycle_init(const lc_config_t* cfg)
{
    if (cfg->no_lifecycle) {
        lc_log("lifecycle: disabled via --no-lifecycle");
        return NULL;
    }

    lifecycle_ctx* ctx = (lifecycle_ctx*)calloc(1, sizeof(lifecycle_ctx));
    if (!ctx) {
        lc_log("lifecycle: calloc failed");
        return NULL;
    }
    ctx->cfg            = *cfg;
    ctx->state          = LC_STATE_INIT;
    ctx->connected_slot = -1;
    ctx->last_bandwidth = -1;

    lc_hooks_init();

    if (lifecycle_gpio_reset(&ctx->cfg) != 0) {
        lc_log("lifecycle: initial hardware reset failed, continuing anyway (chip may already be up)");
    }

    return ctx;
}

/* One BB_GET_STATUS round-trip, slot 0 only -- this project only ever uses
 * single-user/slot-0 mode (see ar8030-linkctl's own default). */
static int lc_poll_link_state(lifecycle_ctx* ctx, bb_link_state_e* out_state)
{
    bb_get_status_in_t  in = {0};
    bb_get_status_out_t out;
    memset(&out, 0, sizeof(out));

    if (bb_ioctl(ctx->client.handle, BB_GET_STATUS, &in, &out) != 0) {
        return -1;
    }
    *out_state = (bb_link_state_e)out.link_status[0].state;
    return 0;
}

/* Applies persisted (or, absent that, this process's --default-bandwidth)
 * tuning to a freshly-connected slot, and seeds ctx->last_bandwidth so the
 * periodic re-check below has a baseline to compare fresh readings
 * against instead of re-persisting its own just-applied value back to
 * itself on the very next tick. Best-effort throughout: a tuning failure
 * is logged, never treated as a reason to tear the connection back down. */
static void lc_apply_tuning_on_connect(lifecycle_ctx* ctx, int slot)
{
    int bandwidth = ctx->cfg.cfg_path[0] ? lc_tuning_load(ctx->cfg.cfg_path) : -1;
    if (bandwidth < 0) {
        bandwidth = ctx->cfg.default_bandwidth;
    }
    if (bandwidth > 0) {
        lc_tuning_apply(ctx->client.handle, slot, bandwidth);
    }
    ctx->last_bandwidth = bandwidth;
}

void* lifecycle_thread_main(void* arg)
{
    lifecycle_ctx* ctx = (lifecycle_ctx*)arg;

    /* Retry the whole connect+subscribe sequence from scratch on any
     * failure -- including a subscribe that timed out rather than cleanly
     * failed -- since a bad attempt during the chip's post-enumeration
     * settle window (see lc_client_subscribe_events()'s comment) leaves no
     * reliable partial state worth keeping. Runs until shutdown or success;
     * lc_client_connect() already retries the raw connect internally, this
     * is the outer loop for recovering from a connect that succeeded but
     * whose subscribe then failed. */
    while (!lifecycle_shutdown_requested()) {
        if (lc_client_connect(&ctx->client, ctx->cfg.rpc_port) != 0) {
            lc_log("lifecycle: could not establish loopback client connection, retrying");
            sleep(LC_ESTABLISH_RETRY_S);
            continue;
        }
        if (lc_client_subscribe_events(&ctx->client) != 0) {
            lc_log("lifecycle: event subscribe failed, reconnecting from scratch");
            lc_client_disconnect(&ctx->client);
            sleep(LC_ESTABLISH_RETRY_S);
            continue;
        }
        break;
    }
    if (lifecycle_shutdown_requested()) {
        return NULL;
    }

    /* Stock's own ar_ldy_gnd does exactly this, unconditionally, once at
     * every startup (fpv_gnd_read_factory_user_cfg() feeding
     * fpv_bb_set_ap_candidate_mac(), confirmed via decompile) -- the
     * chip's own candidate/ap_mac state lives in its volatile RAM and is
     * gone after every hardware reset, so an already-paired unit that
     * only ever pushed this once (at the original pair) comes back up
     * with a chip that has nothing to reconnect to. A no-op (logged,
     * non-fatal) if this unit has never been through a real pair yet. */
    if (ctx->cfg.cfg_path[0]) {
        lc_pair_apply_known_candidate(ctx->client.handle, ctx->cfg.cfg_path, ctx->cfg.role);
    }

    ctx->state = LC_STATE_IDLE;
    lc_log("lifecycle: thread running (role=%d)", (int)ctx->cfg.role);

    time_t last_fallback_poll = 0;

    while (!lifecycle_shutdown_requested()) {
        time_t now              = time(NULL);
        int    did_fallback_poll = 0;
        if (now - last_fallback_poll >= LC_POLL_FALLBACK_S) {
            bb_link_state_e polled;
            if (lc_poll_link_state(ctx, &polled) == 0) {
                lc_set_link_state(polled);
            } else {
                lc_log("lifecycle: BB_GET_STATUS fallback poll failed");
            }
            last_fallback_poll = now;
            did_fallback_poll  = 1;
        }

        bb_link_state_e state;
        int              have_state = lc_get_link_state(&state);

        switch (ctx->state) {
        case LC_STATE_IDLE:
            if (have_state && state == BB_LINK_STATE_CONNECT) {
                /* Reached CONNECT without this loop having triggered it
                 * itself -- either an externally-triggered pair (bind
                 * button) just succeeded, or the chip's own firmware just
                 * autonomously reconnected to an already-configured peer
                 * (see this file's top-of-file comment). Debounced either
                 * way, per that same comment: a single CONNECT reading
                 * here can be a millisecond-scale blip, not a real link. */
                ctx->connect_confirm_count++;
                if (ctx->connect_confirm_count < LC_CONNECT_CONFIRM_TICKS) {
                    break;
                }
                ctx->connect_confirm_count = 0;
                /* Don't re-persist; just follow the transition. */
                lc_log("lifecycle: observed CONNECT, following");
                int slot            = lc_tuning_resolve_connected_slot(ctx->client.handle);
                ctx->connected_slot = slot >= 0 ? slot : 0;
                ctx->state          = LC_STATE_CONNECTED;
                lc_hooks_dispatch(ctx->cfg.hook_dir, "connected", ctx->cfg.role, ctx->connected_slot, NULL);
                lc_apply_tuning_on_connect(ctx, ctx->connected_slot);
                break;
            }
            ctx->connect_confirm_count = 0;
            /* Stay passive regardless of lc_pair_has_been_paired() -- this
             * used to branch here into lc_attempt_pairing() (a
             * BB_SET_PRJ_DISPATCH/PRJ_CMD_EVENT_PAIR dispatch) whenever a
             * peer was already configured, on the theory that a dropped,
             * already-paired link still needed a host-triggered nudge to
             * come back. Ghidra on the stock ar_ldy_gnd binary disproves
             * that: its own init (fpv_bb_init, the counterpart of this
             * daemon's own startup) opens the device, subscribes to
             * BB_EVENT_LINK_STATE, and does nothing else -- no dispatch
             * call anywhere in its normal startup/reconnect path. The only
             * caller of stock's pairing-dispatch routine (fpv_bb_match) is
             * do_bb_match(), wired to the GUI's bind button alone. Stock's
             * own S99ar8030 boot script confirms the same story end to
             * end: enable_rf -> modprobe -> rmmod -> start daemon -> start
             * ar_ldy_gnd, no pairing tool anywhere in it. An already-paired
             * chip's RTOS firmware reconnects to its configured candidate
             * entirely on its own; our repeated dispatch calls were
             * forcing it back into pairing-scan mode on every drop, which
             * is almost certainly what was producing the observed
             * connect-then-drop-within-a-second flapping on ground. So:
             * once paired, this daemon's only job is the same as stock's
             * -- wait for the chip to reconnect by itself. A genuinely
             * fresh pair (no peer ever configured) still requires the bind
             * button; the daemon never auto-binds to "whatever AR8030
             * happens to be in range" either way (see lc_pair_persist()'s
             * own comment on the "accept anyone" candidate list). */
            if (!ctx->idle_hook_fired) {
                lc_hooks_dispatch(ctx->cfg.hook_dir, "idle", ctx->cfg.role, -1, NULL);
                ctx->idle_hook_fired = 1;
            }
            break;

        case LC_STATE_CONNECTED:
            if (have_state && state != BB_LINK_STATE_CONNECT) {
                ctx->drop_miss_count++;
                if (ctx->drop_miss_count < LC_DROP_CONFIRM_TICKS) {
                    break;
                }
                lc_log("lifecycle: link dropped (state=%d), re-pairing", (int)state);
                lc_hooks_dispatch(ctx->cfg.hook_dir, "dropped", ctx->cfg.role, ctx->connected_slot, NULL);
                ctx->state          = LC_STATE_IDLE;
                ctx->connected_slot = -1;
                ctx->last_bandwidth = -1;
                ctx->drop_miss_count = 0;
                break;
            }
            ctx->drop_miss_count = 0;
            /* Re-apply (not "detect and persist a live change"): confirmed
             * live, tonight, that a fresh CONNECT can report a narrow
             * bandwidth (e.g. 2.5M) for a while before the two sides
             * finish negotiating up to what was actually just requested --
             * this is exactly the "bandwidth-plateau" behavior
             * S65ar8030-transport-tx's own former apply_link_tuning()
             * fought by re-issuing its fixed BB_SET_BANDWIDTH every 30s
             * regardless of what the last reading showed. An earlier
             * version of this code tried to be cleverer -- read the
             * current value, and if it differed from what was last
             * applied, treat that as an operator's own live
             * `ar8030-linkctl bandwidth` change and persist it -- but a
             * mid-negotiation narrow reading is indistinguishable from a
             * genuine operator change by value alone, and got
             * self-inflicted proof of that live: it persisted a transient
             * 2MHz plateau reading over a real 20MHz default within one
             * second of connecting. Re-asserting the intended value
             * periodically (BB_SET_BANDWIDTH is a fast, idempotent
             * override, safe to reissue on an already-correct link) is
             * simpler and matches the one thing already proven to work. */
            if (did_fallback_poll && ctx->last_bandwidth > 0) {
                lc_tuning_apply(ctx->client.handle, ctx->connected_slot, ctx->last_bandwidth);
            }
            break;

        case LC_STATE_INIT:
            /* lifecycle_init() already moved past this synchronously;
             * unreachable here, kept only so the switch covers every
             * enumerator. */
            ctx->state = LC_STATE_IDLE;
            break;
        }

        sleep(1);
    }

    lc_client_disconnect(&ctx->client);
    lc_log("lifecycle: thread shutting down cleanly");
    return NULL;
}
