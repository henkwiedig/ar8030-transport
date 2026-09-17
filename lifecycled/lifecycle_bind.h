#ifndef LIFECYCLE_BIND_H
#define LIFECYCLE_BIND_H
#ifdef __cplusplus
extern "C" {
#endif

#include "lifecycle.h"

/*
 * Owns the physical bind-button GPIO directly inside ar8030-lifecycled --
 * see lifecycle_bind.c's own header comment for the full story of why
 * this replaced a separate shell script with no shared state with this
 * daemon (a real bug this project hit: a rebind to an already-connected
 * peer left the LED stuck on its "binding" blink forever, since nothing
 * told the external script's loop to stop).
 */

typedef struct lifecycle_bind_ctx lifecycle_bind_ctx;

/* Returns NULL if cfg->bind_gpio < 0 -- the default, meaning this board
 * has no physical bind button (--bind-gpio was never passed). Callers
 * should skip lifecycle_bind_thread_main() entirely in that case. */
lifecycle_bind_ctx* lifecycle_bind_init(const lc_config_t* cfg);

/* Polls the configured GPIO for a debounced press+release, runs the
 * SDK's own ar8030-pair binary on release, and drives
 * hooks.d/{pairing,connected,idle} directly from its result. Runs until
 * lifecycle_shutdown_requested(), then returns NULL. Same
 * void*(void*) signature as lifecycle_thread_main so either can be
 * pthread_create()'d, though main.c currently just calls this one
 * directly from its own thread -- see main.c's own header comment for
 * why that pairing is safe. */
void* lifecycle_bind_thread_main(void* arg);

#ifdef __cplusplus
}
#endif
#endif
