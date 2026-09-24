#ifndef __LIFECYCLE_H__
#define __LIFECYCLE_H__
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * Lifecycle supervisor: owns the AR8030 chip's whole life (hardware reset,
 * pairing, reconnect-on-drop, tuning, hook-script notification, and
 * optionally the physical bind button -- see lifecycle_bind.h) as one
 * dedicated process, replacing the pile of external shell-script
 * watchdogs (S65ar8030-transport-tx's autoreconnect/device_watchdog/
 * apply_link_tuning, S97ar8030's equivalents, ar8030-led-status.sh,
 * ar8030-bind-button.sh) that used to poll `ar8030-linkctl status` from
 * outside and guess, or hold no shared state with this process at all.
 *
 * Lives in this repo, not package/ar8030's patch stack against the
 * vendor SDK, because it is entirely original code with no vendor
 * lineage -- built the same way tx/ and linkctl/ already are, against
 * the ar8030 package's staged headers/libar8030_client.so
 * ($(STAGING_DIR)), not inside the vendor's own CMake tree.
 */

#define LC_MAX_RESET_GPIOS 8
#define LC_MAX_CHANNELS    64 /* >= the SDK's BB_CONFIG_MAX_CHAN_NUM (60) */

typedef enum {
    LC_RESET_METHOD_NONE = 0,
    LC_RESET_METHOD_DEVMEM,
    LC_RESET_METHOD_GPIO_SYSFS,
} lc_reset_method_e;

typedef enum {
    LC_ROLE_AP = 0,
    LC_ROLE_DEV,
} lc_role_e;

typedef struct {
    int               rpc_port; /* the daemon's own -p port, for the loopback client connection */
    lc_role_e         role;
    lc_reset_method_e reset_method;

    /* LC_RESET_METHOD_DEVMEM only */
    unsigned long reset_devmem_addr;

    /* LC_RESET_METHOD_GPIO_SYSFS only -- exported/pulsed in this order */
    int reset_gpios[LC_MAX_RESET_GPIOS];
    int reset_gpio_count;

    char hook_dir[256];
    char cfg_path[256];

    int default_bandwidth;
    /* BB_SET_FRAME_CHANGE(mode=1) on the AP after every connect, 0 = leave
     * the chip's own frame structure alone. See lc_frame_change_apply(). */
    int frame_change;
    /* AP only: channel used while nothing is persisted yet (no
     * ar8030.channel sidecar): an index, LC_CHANNEL_AUTO or LC_CHANNEL_NONE
     * (leave the chip's own startup channel alone) -- see
     * lifecycle_tuning.h. Ignored on the DEV, which finds the AP's channel
     * with its own idle search. */
    int default_channel;
    /* Output power while none is persisted yet (no ar8030.power sidecar):
     * a level in mW, LC_POWER_AUTO (ground only), LC_POWER_NONE (leave the
     * chip's config-file power alone) or LC_POWER_ROLE_DEFAULT -- see
     * lifecycle_tuning.h. */
    int default_power;

    int no_lifecycle; /* --no-lifecycle escape hatch: lifecycle_init() returns NULL */

    /* GPIO the physical bind button lives on, or -1 (the default) if this
     * board has none -- see lifecycle_bind.h. Pin export/direction is the
     * device overlay's job (matches how the reset/LED GPIOs are already
     * handled); this module only ever reads its value. */
    int bind_gpio;

    /* HTTP control API (lifecycle_http.h), opt-in: http_port == 0 (the
     * default) means disabled -- most boards have no reason to expose
     * this. http_bind defaults to "0.0.0.0" (every interface) once a
     * non-zero port is set, deliberately: the whole point is reaching
     * this from the *other* end of the link (see lifecycle_http.h's own
     * header comment), not just loopback. */
    char     http_bind[64];
    uint16_t http_port;

    /* RF-board temperature via the AR8030's own ADC (see
     * ../common/ar8030_rftemp.h), opt-in: rf_temp_adc < 0 (the default)
     * means disabled, since the ADC channel and thermistor curve are
     * board-specific (Caddx Ascent: channel 4). When enabled, the
     * smoothed reading is also written to rf_temp_file (whole degC, one
     * line) for consumers without an HTTP client, e.g. msposd. */
    int  rf_temp_adc;
    char rf_temp_file[256];
} lc_config_t;

typedef struct lifecycle_ctx lifecycle_ctx;

/* State machine driven by lifecycle_thread_main() (lifecycle.c) --
 * public so lc_status_t below can expose it directly instead of via a
 * parallel duplicate enum. */
typedef enum {
    LC_STATE_INIT = 0,
    LC_STATE_IDLE,
    LC_STATE_CONNECTED,
} lc_state_e;

/* Thread-safe snapshot of the lifecycle thread's own view of the link --
 * see lifecycle_get_status()'s own comment on what protects it. */
typedef struct {
    lc_role_e  role;
    lc_state_e state;
    int        connected_slot; /* -1 if not connected */
    int        bandwidth_mhz;  /* -1 if not yet known/applied */
    int        paired;         /* non-zero if lc_pair_has_been_paired() */
    int        channel;        /* wanted (AP): index, LC_CHANNEL_AUTO; LC_CHANNEL_NONE on the DEV */
    int        chan_auto;      /* chip-reported channel mode, -1 if not read yet */
    int        work_chan;      /* chip-reported working channel, -1 if not read yet */
    int        power;          /* wanted: mW level, LC_POWER_AUTO or LC_POWER_NONE */
    int        power_dbm;      /* chip-reported dBm target, -1 if not read yet */
    int        chan_table_n;   /* channel table size, 0 until read at startup */
    uint32_t   chan_table_khz[LC_MAX_CHANNELS];
    int        rf_temp_valid;  /* non-zero once a real RF-board reading has arrived */
    int        rf_temp_c10;    /* smoothed RF-board temperature, 0.1 degC */
    int        rf_temp_mv;     /* last raw ADC reading, mV */
} lc_status_t;

/*
 * Performs the synchronous hardware reset (once, before returning) and
 * prepares the lifecycle context. Call after rpc_init()/reg_8030_dev() have
 * already run in main.c, since the very first BB_GET_STATUS/pairing dispatch
 * needs a registered device to target and the loopback client connection
 * needs rpc_init()'s listener already bound.
 *
 * Returns NULL if cfg->no_lifecycle is set, or on a fatal init error -- in
 * either case main.c's own pre-existing loop keeps running unaffected.
 */
lifecycle_ctx* lifecycle_init(const lc_config_t* cfg);

/* pthread_create(&t, NULL, lifecycle_thread_main, ctx) -- runs until
 * lifecycle_request_shutdown() is called, then returns NULL. */
void* lifecycle_thread_main(void* arg);

/* Async-signal-safe (just sets a volatile sig_atomic_t flag): called from
 * the SIGTERM handler main.c installs, requests a clean shutdown of the
 * lifecycle thread and, if a hardware-reset pulse is mid-sequence, lets it
 * finish rather than leaving the chip held in reset. */
void lifecycle_request_shutdown(void);

/* Non-zero once lifecycle_request_shutdown() has been called. Polled by
 * lifecycle_gpio.c between reset-pulse phases and by the main loop in
 * lifecycle.c. */
int lifecycle_shutdown_requested(void);

/*
 * BB_EVENT_LINK_STATE / BB_EVENT_PAIR_RESULT callbacks (bb_event_callback
 * signature: void(*)(void* arg, void* user)) -- implemented in lifecycle.c,
 * registered by lifecycle_client_subscribe_events() in lifecycle_client.c.
 * Declared here (not static in lifecycle.c) purely so lifecycle_client.c
 * can take their address; nothing outside this module should call them
 * directly. Run on the client library's own reader thread, not the
 * lifecycle thread -- they only ever touch the shared link-state behind
 * its own lock, never block.
 */
void lc_on_link_state_event(void* arg, void* user);
void lc_on_pair_result_event(void* arg, void* user);

/* Thread-safe snapshot of role/state/connected_slot/bandwidth_mhz/paired
 * -- safe to call from any thread, in particular the HTTP control API's
 * own worker thread (lifecycle_http.c). Protected by ctx's own internal
 * status_lock (lifecycle.c), separate from the pre-existing
 * g_link_state_lock (raw bb_link_state_e only) and cmd_lock (the
 * bandwidth-request mailbox below) -- three distinct pieces of state,
 * three distinct locks, on purpose: conflating them would mean a slow
 * HTTP client blocking a status read out from under the lifecycle
 * thread's own main-loop tick, which must never wait on anything this
 * daemon doesn't otherwise already wait on. */
void lifecycle_get_status(lifecycle_ctx* ctx, lc_status_t* out);

/* Read-only accessor for ctx->cfg -- safe unsynchronized from any thread
 * since cfg is set once in lifecycle_init() and never mutated afterward
 * (mirrors lifecycle_bind_init()'s own by-value cfg copy for the same
 * reason). Used by the HTTP control API to build a pairing/tuning
 * request against the same config the lifecycle thread itself uses. */
const lc_config_t* lifecycle_get_config(const lifecycle_ctx* ctx);

/* Queues a bandwidth change (MHz: 1/2/5/10/20/40) for the lifecycle
 * thread to persist and apply on its own next 1s tick -- never applied
 * directly by the calling thread, since ctx->client.handle's bb_ioctl
 * calls are only ever safe to make from the lifecycle thread itself (see
 * main.c's own header comment on why ar8030-lifecycled is a whole
 * separate process from ar8030d in the first place; the same reasoning
 * rules out a second thread inside *this* process issuing concurrent
 * bb_ioctl calls on the same handle). Persists via lc_tuning_save()
 * regardless of current link state (so it sticks across the next
 * reconnect, exactly like a normal --cfg-path-driven startup would), and
 * additionally applies it live via lc_tuning_apply() if currently
 * connected. Returns 0 if mhz is a valid AR8030 bandwidth and the
 * request was queued, -1 otherwise (nothing queued, nothing changed). */
int lifecycle_request_bandwidth(lifecycle_ctx* ctx, int mhz);

/* Same mailbox pattern as lifecycle_request_bandwidth() above, for the
 * windowed retransmission controller's own tuning parameters (see
 * lifecycle_tuning.h's own comment on lc_retx_apply()/lc_retx_save()).
 * Chip-wide, not per-slot -- applied regardless of which slot is
 * connected. Returns 0 if all 5 values are in range (0-255) and the
 * request was queued, -1 otherwise. */
int lifecycle_request_retx(lifecycle_ctx* ctx, int win, int busy, int idle, int conti_busy, int conti_idle);

/* Same mailbox pattern again, for the channel (an index or
 * LC_CHANNEL_AUTO -- see lifecycle_tuning.h). While connected, applied
 * to both ends at once via lc_channel_apply_linked(), otherwise (AP
 * only) to this radio. The AP owns the channel and is the only side that
 * persists it; a DEV-side change reaches the AP's sidecar through the
 * AP's own lc_channel_track(). Returns 0 if queued, -1 for an invalid
 * value, -2 on the DEV side while no link is up (nothing to push to). */
int lifecycle_request_channel(lifecycle_ctx* ctx, int chan);

/* Same mailbox pattern, for the output power level (mW or LC_POWER_AUTO,
 * see lifecycle_tuning.h). Persisted and applied right away -- power is
 * chip-wide and needs no link. Returns 0 if queued, -1 if level is not
 * one of this role's levels. */
int lifecycle_request_power(lifecycle_ctx* ctx, int level);

#ifdef __cplusplus
}
#endif
#endif
