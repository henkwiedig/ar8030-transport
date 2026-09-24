#include "lifecycle.h"
#include "ar8030.h"
#include "bb_api.h"
#include "lc_log.h"
#include "lifecycle_client.h"
#include "lifecycle_gpio.h"
#include "lifecycle_hooks.h"
#include "lifecycle_pair.h"
#include "lifecycle_tuning.h"
#include "../common/ar8030_batt.h"
#include "../common/ar8030_rftemp.h"
#include <pthread.h>
#include <stdio.h>
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

/* Vendor's own shipped defaults for the windowed retransmission
 * controller (ar_ldyhs_sky's own fpv_bb_init, confirmed via this
 * project's Ghidra decompile -- see bb_retx_cfg_t's own doc comment in
 * bb_api.h). Applied at every connect whenever nothing has been
 * persisted via /api/v1/retx-tuning yet, so the controller is always
 * actively configured rather than left in its raw, never-set state --
 * unlike bandwidth's own unconfigured state (which just means "chip's
 * own default gear"), an unconfigured retx controller has no meaningful
 * signal to read at all, which matters once this feeds a future rate-
 * control input (see this project's own Phase C notes). Confirmed SAFE
 * by this project's own hardware sweep (this exact combination, plus
 * several more extreme ones, caused no adverse link effect) -- not
 * confirmed CORRECT, since the units/semantics of these thresholds are
 * still unverified; this is "the vendor's own choice, known not to
 * break anything," not "the right value for this hardware." */
#define LC_RETX_DEFAULT_WIN         10
#define LC_RETX_DEFAULT_BUSY        6
#define LC_RETX_DEFAULT_IDLE        4
#define LC_RETX_DEFAULT_CONTI_BUSY  2
#define LC_RETX_DEFAULT_CONTI_IDLE  0

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

    /* Guards state/connected_slot/last_bandwidth above against a
     * concurrent lifecycle_get_status() call from another thread (the
     * HTTP control API's own worker thread) -- see lifecycle.h's own
     * comment on why this is a separate lock from g_link_state_lock and
     * cmd_lock below. Only ever write-locked by the lifecycle thread
     * itself (already single-threaded with respect to its own state), so
     * this never blocks that thread on anything but a fast read. */
    pthread_mutex_t status_lock;

    /* One-slot mailbox for lifecycle_request_bandwidth() -- see that
     * function's own comment in lifecycle.h for why a bandwidth change
     * requested over HTTP is queued here instead of applied directly by
     * the requesting thread. -1 = nothing queued. A second request
     * overwriting an unconsumed first is fine (last-write-wins, same as
     * calling the HTTP endpoint twice in a row would already imply). */
    pthread_mutex_t cmd_lock;
    int             pending_bandwidth_mhz;

    /* Same one-slot-mailbox pattern, for lifecycle_request_retx() --
     * guarded by the same cmd_lock rather than a new one, since both
     * are just different requests the lifecycle thread drains on its
     * own next tick. pending_retx_valid is the "anything queued" flag
     * (unlike bandwidth, 0 is itself a legal value for every one of
     * these 5 fields, so there's no single sentinel int that means
     * "nothing queued"). */
    int pending_retx_valid;
    int pending_retx_win;
    int pending_retx_busy;
    int pending_retx_idle;
    int pending_retx_conti_busy;
    int pending_retx_conti_idle;

    /* Link distance in metres, -1 without a link/result (status_lock). */
    int distance_m;

    /* The chip's channel table, read once at startup (status_lock). */
    int      chan_table_n;
    uint32_t chan_table_khz[LC_MAX_CHANNELS];

    /* lifecycle_request_power()'s mailbox, same cmd_lock. */
    int pending_power_valid;
    int pending_power;

    /* Wanted output power (mW level, LC_POWER_AUTO or LC_POWER_NONE) and
     * the chip's last-read dBm target (-1 until read), status_lock. */
    int power;
    int power_dbm;

    /* lifecycle_request_channel()'s mailbox, same cmd_lock. */
    int pending_channel_valid;
    int pending_channel;

    /* Wanted channel (index, LC_CHANNEL_AUTO or LC_CHANNEL_NONE): the
     * persisted one, else cfg.default_channel -- AP only, always
     * LC_CHANNEL_NONE on the DEV (see lc_channel_track()). Written only by
     * the lifecycle thread, under status_lock (lifecycle_get_status()). */
    int channel;
    /* Last BB_GET_CHAN_INFO reading, -1 until read (status_lock). */
    int chan_auto;
    int work_chan;
    /* Whether the chip has matched ctx->channel since the last change we
     * made on this connection -- only then does a divergence mean the
     * peer moved the link (lc_channel_track()). */
    int chan_settled;

    /* RF-board temperature (lc_poll_rf_temp()), guarded by status_lock
     * like the rest of what lifecycle_get_status() exposes. */
    int rf_temp_armed;
    int rf_temp_rearm_wait; /* ticks until the next re-arm attempt */
    int rf_temp_valid;
    int rf_temp_c10;
    int rf_temp_mv;

    /* Supply voltage (lc_poll_batt()), same locking as rf-temp. */
    int batt_armed;
    int batt_rearm_wait;
    int batt_arm_ret; /* last arm result, to log only changes */
    int batt_adc_mv;  /* smoothed ADC reading, 0 = none */
};

static void lc_apply_power(lifecycle_ctx* ctx);

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
    ctx->cfg                  = *cfg;
    ctx->state                = LC_STATE_INIT;
    ctx->connected_slot       = -1;
    ctx->last_bandwidth       = -1;
    ctx->pending_bandwidth_mhz = -1;
    ctx->chan_auto            = -1;
    ctx->work_chan            = -1;
    ctx->power_dbm            = -1;
    ctx->distance_m           = -1;
    ctx->batt_arm_ret         = 1; /* not an ioctl result: logs the first arm */
    if (cfg->role != LC_ROLE_AP) {
        ctx->channel = LC_CHANNEL_NONE;
    } else if (!cfg->cfg_path[0] || lc_channel_load(cfg->cfg_path, &ctx->channel) != 0) {
        ctx->channel = cfg->default_channel;
    }
    int is_ap = cfg->role == LC_ROLE_AP;
    if (!cfg->cfg_path[0] || lc_power_load(cfg->cfg_path, &ctx->power) != 0 || !lc_power_valid(is_ap, ctx->power)) {
        ctx->power = cfg->default_power == LC_POWER_ROLE_DEFAULT ? lc_power_default(is_ap) : cfg->default_power;
    }
    if (ctx->power != LC_POWER_NONE && !lc_power_valid(is_ap, ctx->power)) {
        lc_log("lifecycle: power level %d is not valid for this role, using %d mW", ctx->power,
               lc_power_default(is_ap));
        ctx->power = lc_power_default(is_ap);
    }
    pthread_mutex_init(&ctx->status_lock, NULL);
    pthread_mutex_init(&ctx->cmd_lock, NULL);

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
    pthread_mutex_lock(&ctx->status_lock);
    ctx->last_bandwidth = bandwidth;
    pthread_mutex_unlock(&ctx->status_lock);

    /* Chip-wide, not tied to `slot` -- but only worth applying once a
     * link has actually formed (mirrors bandwidth's own "connect first,
     * tune second" ordering). Unlike bandwidth's own persisted-value-or-
     * chip-default fallback, there is no "chip default" worth falling
     * back to here: this project's own stack has simply never called
     * BB_SET_RETX_EVENT_STATUS before, so the controller's raw state is
     * all-zero, not a real default -- always apply *something* (LC_RETX_
     * DEFAULT_* above absent a persisted choice) so the controller is
     * always actively configured. This matters beyond "why not leave it
     * alone": an unconfigured controller has no meaningful state for a
     * future rate-control input to read, so treating "never touch it"
     * as the safe default would just make Phase C's retx-driven backoff
     * permanently unbuildable. */
    bb_retx_cfg_t retx;
    if (!ctx->cfg.cfg_path[0] || lc_retx_load(ctx->cfg.cfg_path, &retx) != 0) {
        memset(&retx, 0, sizeof(retx));
        retx.win        = LC_RETX_DEFAULT_WIN;
        retx.busy       = LC_RETX_DEFAULT_BUSY;
        retx.idle       = LC_RETX_DEFAULT_IDLE;
        retx.conti_busy = LC_RETX_DEFAULT_CONTI_BUSY;
        retx.conti_idle = LC_RETX_DEFAULT_CONTI_IDLE;
    }
    lc_retx_apply(ctx->client.handle, retx.win, retx.busy, retx.idle, retx.conti_busy, retx.conti_idle);

    /* Re-assert output power: cheap and idempotent, and nothing guarantees
     * the chip keeps it across a re-link. */
    lc_apply_power(ctx);

    /* AP side only -- it is the controlling side of the 1V1 link (stock
     * gets ~40 Mbit/s at MCS 12 vs 25.9 without this). Chip-wide, needs an
     * established link, and is lost on reboot/re-link, hence here. */
    if (ctx->cfg.role == LC_ROLE_AP && ctx->cfg.frame_change) {
        lc_frame_change_apply(ctx->client.handle, 1);
    }
}

/* Drains lifecycle_request_bandwidth()'s one-slot mailbox, if anything is
 * queued, and acts on it -- called once per main-loop tick regardless of
 * ctx->state (unlike lc_apply_tuning_on_connect() above, which only ever
 * runs exactly at the IDLE->CONNECTED transition). Always persists (so an
 * HTTP request made while idle still sticks for the next connect, same
 * persist-then-apply-on-connect path a normal --cfg-path-driven startup
 * already uses), and additionally applies live via lc_tuning_apply() only
 * if actually connected right now -- applying blind against a slot with
 * no real peer would just be a wasted ioctl (see lc_tuning_apply()'s own
 * BB_SET_BANDWIDTH target). */
static void lc_drain_bandwidth_request(lifecycle_ctx* ctx)
{
    int requested_mhz;
    pthread_mutex_lock(&ctx->cmd_lock);
    requested_mhz              = ctx->pending_bandwidth_mhz;
    ctx->pending_bandwidth_mhz = -1;
    pthread_mutex_unlock(&ctx->cmd_lock);

    if (requested_mhz <= 0) {
        return;
    }

    lc_log("lifecycle: http: bandwidth change requested: %d MHz", requested_mhz);
    if (ctx->cfg.cfg_path[0]) {
        lc_tuning_save(ctx->cfg.cfg_path, requested_mhz);
    }
    if (ctx->state == LC_STATE_CONNECTED) {
        lc_tuning_apply(ctx->client.handle, ctx->connected_slot, requested_mhz);
        pthread_mutex_lock(&ctx->status_lock);
        ctx->last_bandwidth = requested_mhz;
        pthread_mutex_unlock(&ctx->status_lock);
    }
}

/* Same drain pattern as lc_drain_bandwidth_request() above, for
 * lifecycle_request_retx()'s own mailbox -- always persists (sticks
 * across the next reconnect), and additionally applies live via
 * lc_retx_apply() since this project's own hardware sweep confirmed a
 * live SET after boot actually takes effect (unlike the vendor's own
 * app, which only ever sets this once, at init -- see bb_retx_cfg_t's
 * doc comment). Chip-wide: unlike bandwidth this doesn't need
 * ctx->connected_slot at all, so it applies regardless of ctx->state
 * rather than gating on CONNECTED -- confirmed live that a live SET
 * doesn't require an active link (real hardware sweep tested it while
 * CONNECTED throughout, but the ioctl itself carries no slot/link
 * precondition per bb_retx_cfg_t's own struct, which has no slot
 * field either). */
static void lc_drain_retx_request(lifecycle_ctx* ctx)
{
    int valid, win, busy, idle, conti_busy, conti_idle;
    pthread_mutex_lock(&ctx->cmd_lock);
    valid                     = ctx->pending_retx_valid;
    win                       = ctx->pending_retx_win;
    busy                      = ctx->pending_retx_busy;
    idle                      = ctx->pending_retx_idle;
    conti_busy                = ctx->pending_retx_conti_busy;
    conti_idle                = ctx->pending_retx_conti_idle;
    ctx->pending_retx_valid   = 0;
    pthread_mutex_unlock(&ctx->cmd_lock);

    if (!valid) {
        return;
    }

    lc_log("lifecycle: http: retx config change requested: win=%d,busy=%d,idle=%d,conti_busy=%d,conti_idle=%d",
           win, busy, idle, conti_busy, conti_idle);
    if (ctx->cfg.cfg_path[0]) {
        lc_retx_save(ctx->cfg.cfg_path, win, busy, idle, conti_busy, conti_idle);
    }
    lc_retx_apply(ctx->client.handle, win, busy, idle, conti_busy, conti_idle);
}

/* "auto", "none" or the index, for log lines. */
static const char* lc_channel_str(int chan, char* buf, size_t buf_sz)
{
    if (chan == LC_CHANNEL_AUTO) {
        return "auto";
    }
    if (chan == LC_CHANNEL_NONE) {
        return "none";
    }
    snprintf(buf, buf_sz, "%d", chan);
    return buf;
}

static void lc_set_channel(lifecycle_ctx* ctx, int chan)
{
    pthread_mutex_lock(&ctx->status_lock);
    ctx->channel = chan;
    pthread_mutex_unlock(&ctx->status_lock);
}

/* Drains lifecycle_request_channel()'s mailbox -- same shape as
 * lc_drain_bandwidth_request(). Applies to both ends if connected,
 * otherwise to this radio only (AP only: the DEV never gets here while
 * idle, see lifecycle_request_channel()). Persisted on the AP only; a
 * DEV-initiated change reaches the AP's sidecar through the AP's own
 * lc_channel_track(). */
static void lc_drain_channel_request(lifecycle_ctx* ctx)
{
    int valid, chan;
    pthread_mutex_lock(&ctx->cmd_lock);
    valid                      = ctx->pending_channel_valid;
    chan                       = ctx->pending_channel;
    ctx->pending_channel_valid = 0;
    pthread_mutex_unlock(&ctx->cmd_lock);

    if (!valid) {
        return;
    }
    char buf[16];
    lc_log("lifecycle: http: channel change requested: %s", lc_channel_str(chan, buf, sizeof(buf)));

    /* The HTTP side can only bound-check against BB_CONFIG_MAX_CHAN_NUM;
     * the chip's real table size is only readable from here. */
    int auto_mode, work_chan, chan_num;
    if (chan >= 0 && lc_channel_read(ctx->client.handle, &auto_mode, &work_chan, &chan_num) == 0 &&
        chan >= chan_num) {
        lc_log("lifecycle: channel %d is outside the chip's %d-entry table, ignoring", chan, chan_num);
        return;
    }

    if (ctx->cfg.role == LC_ROLE_AP) {
        if (ctx->cfg.cfg_path[0]) {
            lc_channel_save(ctx->cfg.cfg_path, chan);
        }
        lc_set_channel(ctx, chan);
        ctx->chan_settled = 0;
    }
    if (ctx->state == LC_STATE_CONNECTED) {
        lc_channel_apply_linked(ctx->client.handle, ctx->connected_slot, chan);
    } else {
        lc_channel_apply_local(ctx->client.handle, chan);
    }
}

/* Once per fallback poll on a live link: publishes the chip's channel
 * for lifecycle_get_status(), and on the AP picks up a change the other
 * end made (its own /api/v1/channel, or a raw `linkctl channel`
 * passthrough on either side) so it sticks across reboots.
 *
 * AP only, and only after the chip has matched ctx->channel at least
 * once on this connection. Confirmed on hardware that the DEV's reading
 * is useless for this: while idle the DEV chip hops through the whole
 * channel table looking for the AP (ar8030d log:
 * bb_link_node_br_idle_proc), so right after a reconnect it reports
 * wherever that search left it -- an earlier version adopted those
 * readings on the DEV and saved channels nobody asked for. The AP is the
 * one whose channel the DEV searches for, so it is the only side that
 * keeps one. No re-pushing on a mismatch either: the AP pushes once per
 * connect (lc_channel_on_connect()) and otherwise leaves the link alone. */
static void lc_channel_track(lifecycle_ctx* ctx)
{
    int auto_mode, work_chan, chan_num;
    if (lc_channel_read(ctx->client.handle, &auto_mode, &work_chan, &chan_num) != 0) {
        return;
    }
    pthread_mutex_lock(&ctx->status_lock);
    ctx->chan_auto = auto_mode;
    ctx->work_chan = work_chan;
    pthread_mutex_unlock(&ctx->status_lock);

    if (ctx->channel == LC_CHANNEL_NONE) {
        return;
    }
    if (lc_channel_matches(ctx->channel, auto_mode, work_chan)) {
        ctx->chan_settled = 1;
        return;
    }
    if (!ctx->chan_settled) {
        return;
    }

    int  observed = auto_mode ? LC_CHANNEL_AUTO : work_chan;
    char buf[16];
    lc_log("lifecycle: link moved to channel %s by the peer, saving", lc_channel_str(observed, buf, sizeof(buf)));
    if (ctx->cfg.cfg_path[0]) {
        lc_channel_save(ctx->cfg.cfg_path, observed);
    }
    lc_set_channel(ctx, observed);
}

/* IDLE -> CONNECTED, AP only: pushes ctx->channel once if the link came
 * up anywhere else (e.g. the pre-link lc_channel_apply_local() didn't
 * take). */
static void lc_channel_on_connect(lifecycle_ctx* ctx)
{
    ctx->chan_settled = 0;
    if (ctx->channel == LC_CHANNEL_NONE) {
        return;
    }
    int auto_mode, work_chan, chan_num = 0;
    if (lc_channel_read(ctx->client.handle, &auto_mode, &work_chan, &chan_num) == 0 &&
        lc_channel_matches(ctx->channel, auto_mode, work_chan)) {
        return;
    }
    if (ctx->channel >= 0 && chan_num > 0 && ctx->channel >= chan_num) {
        lc_log("lifecycle: channel %d is outside the chip's %d-entry table, not applying", ctx->channel, chan_num);
        return;
    }
    lc_channel_apply_linked(ctx->client.handle, ctx->connected_slot, ctx->channel);
}

static const char* lc_power_str(int level, char* buf, size_t buf_sz)
{
    if (level == LC_POWER_AUTO) {
        return "auto";
    }
    if (level == LC_POWER_NONE) {
        return "none";
    }
    snprintf(buf, buf_sz, "%d mW", level);
    return buf;
}

static void lc_apply_power(lifecycle_ctx* ctx)
{
    if (ctx->power == LC_POWER_NONE) {
        return;
    }
    lc_power_apply(ctx->client.handle, ctx->cfg.role == LC_ROLE_AP, ctx->power);
}

/* Refreshes lifecycle_get_status()'s power_dbm, once per fallback poll. */
static void lc_poll_power(lifecycle_ctx* ctx)
{
    int dbm = lc_power_read_dbm(ctx->client.handle, ctx->cfg.role == LC_ROLE_AP);
    pthread_mutex_lock(&ctx->status_lock);
    ctx->power_dbm = dbm;
    pthread_mutex_unlock(&ctx->status_lock);
}

/* Refreshes lifecycle_get_status()'s distance_m, every tick while
 * connected -- cheap, and an OSD wants it fresher than the 5 s fallback
 * poll. */
static void lc_poll_distance(lifecycle_ctx* ctx)
{
    int m = ctx->state == LC_STATE_CONNECTED ? lc_distance_read_m(ctx->client.handle) : -1;
    pthread_mutex_lock(&ctx->status_lock);
    ctx->distance_m = m;
    pthread_mutex_unlock(&ctx->status_lock);
}

/* Drains lifecycle_request_power()'s mailbox: persists, then applies right
 * away (chip-wide, no link needed). */
static void lc_drain_power_request(lifecycle_ctx* ctx)
{
    int valid, level;
    pthread_mutex_lock(&ctx->cmd_lock);
    valid                    = ctx->pending_power_valid;
    level                    = ctx->pending_power;
    ctx->pending_power_valid = 0;
    pthread_mutex_unlock(&ctx->cmd_lock);

    if (!valid) {
        return;
    }
    char buf[16];
    lc_log("lifecycle: http: power change requested: %s", lc_power_str(level, buf, sizeof(buf)));
    if (ctx->cfg.cfg_path[0]) {
        lc_power_save(ctx->cfg.cfg_path, level);
    }
    pthread_mutex_lock(&ctx->status_lock);
    ctx->power = level;
    pthread_mutex_unlock(&ctx->status_lock);
    lc_apply_power(ctx);
    lc_poll_power(ctx);
}

/* Writes whole degC to cfg.rf_temp_file via rename(), so a reader never
 * sees a half-written line. */
static void lc_write_rf_temp_file(const lifecycle_ctx* ctx, int c10)
{
    char tmp[272];
    snprintf(tmp, sizeof(tmp), "%s.tmp", ctx->cfg.rf_temp_file);
    FILE* f = fopen(tmp, "w");
    if (!f) {
        return;
    }
    fprintf(f, "%d\n", (c10 + 5) / 10);
    fclose(f);
    rename(tmp, ctx->cfg.rf_temp_file);
}

/* One RF-board temperature sample, once per main-loop tick -- stock polls
 * from its own temperature thread the same way. The ADC channel is armed
 * once up front (stock: fpv_bb_init) and re-armed whenever a read fails or
 * returns 0 mV (not armed, e.g. after a chip reset), at most every
 * LC_RFTEMP_REARM_TICKS. Smoothing matches stock's
 * fpv_bb_update_rf_board_temp(): new = 0.75*old + 0.25*sample. */
#define LC_RFTEMP_REARM_TICKS 5

static void lc_poll_rf_temp(lifecycle_ctx* ctx)
{
    int ch = ctx->cfg.rf_temp_adc;
    if (ch < 0) {
        return;
    }

    if (!ctx->rf_temp_armed) {
        if (ctx->rf_temp_rearm_wait > 0) {
            ctx->rf_temp_rearm_wait--;
            return;
        }
        int ret = ar8030_rftemp_arm(ctx->client.handle, ch);
        lc_log("lifecycle: rf-temp: armed ADC channel %d (ret=%d)", ch, ret);
        ctx->rf_temp_armed      = ret == 0;
        ctx->rf_temp_rearm_wait = LC_RFTEMP_REARM_TICKS;
        return; /* first measurement lands after the arm period */
    }

    int mv = 0;
    if (ar8030_rftemp_read_mv(ctx->client.handle, ch, &mv) != 0 || mv <= 0) {
        ctx->rf_temp_armed = 0;
        return;
    }

    int sample = ar8030_rftemp_mv_to_c10(mv);
    if (sample == AR8030_RFTEMP_NONE) {
        /* Out of the thermistor's range: no sensor on this channel (see
         * ar8030_rftemp.h). Report nothing rather than a made-up value. */
        pthread_mutex_lock(&ctx->status_lock);
        if (ctx->rf_temp_valid) {
            lc_log("lifecycle: rf-temp: %d mV on ADC %d is below the thermistor table, no sensor?", mv, ch);
        }
        ctx->rf_temp_valid = 0;
        ctx->rf_temp_mv    = mv;
        pthread_mutex_unlock(&ctx->status_lock);
        if (ctx->cfg.rf_temp_file[0]) {
            unlink(ctx->cfg.rf_temp_file);
        }
        return;
    }
    pthread_mutex_lock(&ctx->status_lock);
    ctx->rf_temp_c10   = ctx->rf_temp_valid ? (ctx->rf_temp_c10 * 3 + sample) / 4 : sample;
    ctx->rf_temp_mv    = mv;
    ctx->rf_temp_valid = 1;
    int c10            = ctx->rf_temp_c10;
    pthread_mutex_unlock(&ctx->status_lock);

    if (ctx->cfg.rf_temp_file[0]) {
        lc_write_rf_temp_file(ctx, c10);
    }
}

/* Writes the supply voltage to cfg.batt_file as volts with two decimals
 * ("11.98"), via rename() like the rf-temp file. */
static void lc_write_batt_file(const lifecycle_ctx* ctx, int mv)
{
    char tmp[272];
    snprintf(tmp, sizeof(tmp), "%s.tmp", ctx->cfg.batt_file);
    FILE* f = fopen(tmp, "w");
    if (!f) {
        return;
    }
    int cv = (mv + 5) / 10; /* centivolts, rounded */
    fprintf(f, "%d.%02d\n", cv / 100, cv % 100);
    fclose(f);
    rename(tmp, ctx->cfg.batt_file);
}

/* One supply-voltage sample per main-loop tick (stock: every 500 ms from
 * its temperature thread), mapped by ar8030_batt_mv(). Arming and re-arming
 * work like lc_poll_rf_temp(), except that 0 mV is also a real reading
 * here -- nothing on the power input, e.g. on USB power -- so a re-arm
 * that doesn't help just keeps reporting "no reading", logging only when
 * the arm result changes. Smoothing matches stock's
 * fpv_sys_update_batt_volt(): new = 0.75*old + 0.25*sample. */
static void lc_poll_batt(lifecycle_ctx* ctx)
{
    int ch = ctx->cfg.batt_adc;
    if (ch < 0) {
        return;
    }

    if (!ctx->batt_armed) {
        if (ctx->batt_rearm_wait > 0) {
            ctx->batt_rearm_wait--;
            return;
        }
        int ret = ar8030_rftemp_arm(ctx->client.handle, ch);
        if (ret != ctx->batt_arm_ret) {
            lc_log("lifecycle: batt: armed ADC channel %d (ret=%d)", ch, ret);
            ctx->batt_arm_ret = ret;
        }
        ctx->batt_armed      = ret == 0;
        ctx->batt_rearm_wait = LC_RFTEMP_REARM_TICKS;
        return;
    }

    int mv = 0;
    if (ar8030_rftemp_read_mv(ctx->client.handle, ch, &mv) != 0 || mv <= 0) {
        ctx->batt_armed = 0;
        pthread_mutex_lock(&ctx->status_lock);
        ctx->batt_adc_mv = 0;
        pthread_mutex_unlock(&ctx->status_lock);
        if (ctx->cfg.batt_file[0]) {
            unlink(ctx->cfg.batt_file);
        }
        return;
    }

    pthread_mutex_lock(&ctx->status_lock);
    ctx->batt_adc_mv = ctx->batt_adc_mv > 0 ? (ctx->batt_adc_mv * 3 + mv) / 4 : mv;
    int batt_mv      = ar8030_batt_mv(ctx->batt_adc_mv, ctx->cfg.batt_scale, ctx->cfg.batt_offset_mv);
    pthread_mutex_unlock(&ctx->status_lock);

    if (ctx->cfg.batt_file[0]) {
        lc_write_batt_file(ctx, batt_mv);
    }
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

    /* Fixed for the chip's lifetime (it comes from ar8030.json), so one
     * read is enough -- served by GET /api/v1/channel for channel pickers. */
    {
        uint32_t khz[LC_MAX_CHANNELS];
        int      n = lc_channel_read_table(ctx->client.handle, khz, LC_MAX_CHANNELS);
        if (n > 0) {
            pthread_mutex_lock(&ctx->status_lock);
            memcpy(ctx->chan_table_khz, khz, (size_t)n * sizeof(khz[0]));
            ctx->chan_table_n = n;
            pthread_mutex_unlock(&ctx->status_lock);
        }
        lc_log("lifecycle: channel table: %d entries", n);
    }

    /* The chip boots on whatever ar8030.json says (currently manual mode
     * on the table's first channel). Only the AP moves itself to its
     * persisted/default channel before any link exists -- the DEV finds
     * it there with its own idle channel search (see lc_channel_track()),
     * which pinning the DEV to a fixed channel would only get in the way
     * of. ctx->channel is always LC_CHANNEL_NONE on the DEV. */
    if (ctx->channel != LC_CHANNEL_NONE) {
        char buf[16];
        lc_log("lifecycle: startup channel %s", lc_channel_str(ctx->channel, buf, sizeof(buf)));
        lc_channel_apply_local(ctx->client.handle, ctx->channel);
    }

    /* Output power is chip-wide and needs no link: set it before the link
     * comes up (so the disconnected/search power is already right too),
     * and again on every connect -- see lc_apply_tuning_on_connect(). */
    if (ctx->power != LC_POWER_NONE) {
        char buf[16];
        lc_log("lifecycle: startup power %s", lc_power_str(ctx->power, buf, sizeof(buf)));
        lc_apply_power(ctx);
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
            lc_poll_power(ctx);
        }

        lc_drain_bandwidth_request(ctx);
        lc_drain_retx_request(ctx);
        lc_drain_channel_request(ctx);
        lc_drain_power_request(ctx);
        lc_poll_distance(ctx);
        lc_poll_rf_temp(ctx);
        lc_poll_batt(ctx);

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
                int slot = lc_tuning_resolve_connected_slot(ctx->client.handle);
                pthread_mutex_lock(&ctx->status_lock);
                ctx->connected_slot = slot >= 0 ? slot : 0;
                ctx->state          = LC_STATE_CONNECTED;
                pthread_mutex_unlock(&ctx->status_lock);
                lc_hooks_dispatch(ctx->cfg.hook_dir, "connected", ctx->cfg.role, ctx->connected_slot, NULL);
                lc_apply_tuning_on_connect(ctx, ctx->connected_slot);
                lc_channel_on_connect(ctx);
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
                pthread_mutex_lock(&ctx->status_lock);
                ctx->state          = LC_STATE_IDLE;
                ctx->connected_slot = -1;
                ctx->last_bandwidth = -1;
                ctx->chan_auto      = -1;
                ctx->work_chan      = -1;
                pthread_mutex_unlock(&ctx->status_lock);
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
                /* Bandwidth renegotiation may reset the frame structure;
                 * mode=1 is idempotent, so just re-assert it alongside. */
                if (ctx->cfg.role == LC_ROLE_AP && ctx->cfg.frame_change) {
                    lc_frame_change_apply(ctx->client.handle, 1);
                }
            }
            if (did_fallback_poll) {
                lc_channel_track(ctx);
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

void lifecycle_get_status(lifecycle_ctx* ctx, lc_status_t* out)
{
    memset(out, 0, sizeof(*out));
    out->role = ctx->cfg.role;

    pthread_mutex_lock(&ctx->status_lock);
    out->state          = ctx->state;
    out->connected_slot = ctx->connected_slot;
    out->bandwidth_mhz  = ctx->last_bandwidth;
    out->channel        = ctx->channel;
    out->chan_auto      = ctx->chan_auto;
    out->work_chan      = ctx->work_chan;
    out->power          = ctx->power;
    out->power_dbm      = ctx->power_dbm;
    out->distance_m     = ctx->distance_m;
    out->chan_table_n   = ctx->chan_table_n;
    memcpy(out->chan_table_khz, ctx->chan_table_khz, sizeof(out->chan_table_khz));
    out->rf_temp_valid  = ctx->rf_temp_valid;  /* rf_temp_mv is set either way */
    out->rf_temp_c10    = ctx->rf_temp_c10;
    out->rf_temp_mv     = ctx->rf_temp_mv;
    out->batt_adc_mv    = ctx->batt_adc_mv;
    out->batt_mv        = ar8030_batt_mv(ctx->batt_adc_mv, ctx->cfg.batt_scale, ctx->cfg.batt_offset_mv);
    pthread_mutex_unlock(&ctx->status_lock);

    out->paired = ctx->cfg.cfg_path[0] ? lc_pair_has_been_paired(ctx->cfg.cfg_path) : 0;
}

const lc_config_t* lifecycle_get_config(const lifecycle_ctx* ctx)
{
    return &ctx->cfg;
}

int lifecycle_request_bandwidth(lifecycle_ctx* ctx, int mhz)
{
    if (!lc_tuning_valid_mhz(mhz)) {
        return -1;
    }
    pthread_mutex_lock(&ctx->cmd_lock);
    ctx->pending_bandwidth_mhz = mhz;
    pthread_mutex_unlock(&ctx->cmd_lock);
    return 0;
}

int lifecycle_request_retx(lifecycle_ctx* ctx, int win, int busy, int idle, int conti_busy, int conti_idle)
{
    if (!lc_retx_valid(win, busy, idle, conti_busy, conti_idle)) {
        return -1;
    }
    pthread_mutex_lock(&ctx->cmd_lock);
    ctx->pending_retx_win         = win;
    ctx->pending_retx_busy        = busy;
    ctx->pending_retx_idle        = idle;
    ctx->pending_retx_conti_busy  = conti_busy;
    ctx->pending_retx_conti_idle  = conti_idle;
    ctx->pending_retx_valid       = 1;
    pthread_mutex_unlock(&ctx->cmd_lock);
    return 0;
}

int lifecycle_request_channel(lifecycle_ctx* ctx, int chan)
{
    if (chan != LC_CHANNEL_AUTO && (chan < 0 || chan >= BB_CONFIG_MAX_CHAN_NUM)) {
        return -1;
    }
    if (ctx->cfg.role == LC_ROLE_DEV) {
        pthread_mutex_lock(&ctx->status_lock);
        int connected = ctx->state == LC_STATE_CONNECTED;
        pthread_mutex_unlock(&ctx->status_lock);
        if (!connected) {
            return -2;
        }
    }
    pthread_mutex_lock(&ctx->cmd_lock);
    ctx->pending_channel       = chan;
    ctx->pending_channel_valid = 1;
    pthread_mutex_unlock(&ctx->cmd_lock);
    return 0;
}

int lifecycle_request_power(lifecycle_ctx* ctx, int level)
{
    if (!lc_power_valid(ctx->cfg.role == LC_ROLE_AP, level)) {
        return -1;
    }
    pthread_mutex_lock(&ctx->cmd_lock);
    ctx->pending_power       = level;
    ctx->pending_power_valid = 1;
    pthread_mutex_unlock(&ctx->cmd_lock);
    return 0;
}
