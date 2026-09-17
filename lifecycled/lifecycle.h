#ifndef __LIFECYCLE_H__
#define __LIFECYCLE_H__
#ifdef __cplusplus
extern "C" {
#endif

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
    int default_channel; /* -1 = unset, no channel override at connect time */

    int no_lifecycle; /* --no-lifecycle escape hatch: lifecycle_init() returns NULL */

    /* GPIO the physical bind button lives on, or -1 (the default) if this
     * board has none -- see lifecycle_bind.h. Pin export/direction is the
     * device overlay's job (matches how the reset/LED GPIOs are already
     * handled); this module only ever reads its value. */
    int bind_gpio;
} lc_config_t;

typedef struct lifecycle_ctx lifecycle_ctx;

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

#ifdef __cplusplus
}
#endif
#endif
