#include "lifecycle_bind.h"
#include "lc_log.h"
#include "lifecycle_gpio.h"
#include "lifecycle_pair.h"
#include <stdlib.h>
#include <unistd.h>

/*
 * Direct port of what used to be a separate shell script
 * (devices/hi3516cv6xx_fpv_caddx-ascent-lite/.../ar8030-bind-button.sh +
 * S66ar8030-bind-button in the builder repo), moved into this daemon so
 * it can drive hook dispatch directly instead of guessing.
 *
 * Confirmed live (debugging this exact move) why the split mattered: the
 * shell script called the SDK's own ar8030-pair binary directly, with no
 * shared state with ar8030-lifecycled at all. A rebind to an
 * already-connected peer produced no observable transition in
 * lifecycle_thread_main's own state machine (lifecycle.c) -- it was
 * already in LC_STATE_CONNECTED before the button press, stayed there
 * after, and "connected" only ever fires on the IDLE->CONNECTED
 * transition, never merely because the link is still up -- so nothing
 * ever told the script's own fast-blink "pairing" LED loop to stop. That
 * bug is architectural (whichever process owns the button needs to know
 * whether its own pairing attempt actually changed anything), not a
 * shell-scripting mistake, which is why the fix is this module driving
 * hooks.d/connected itself immediately from ar8030-pair's own result,
 * rather than waiting on lifecycle_thread_main to (maybe never) notice.
 *
 * Still invokes the SDK's own ar8030-pair binary (dev_helper/bb_pair,
 * package/ar8030) for the actual pairing handshake rather than
 * reimplementing BB_SET_PRJ_DISPATCH here -- deliberate: this module
 * owns *when* to pair (the physical button) and *what happens after*
 * (hook dispatch), not the handshake protocol itself. See
 * lifecycle_pair.c's own header comment for why a from-scratch
 * reimplementation of that dispatch was tried once already and removed.
 * The actual fork+exec+hook-dispatch sequence now lives in
 * lifecycle_pair.c's lc_pair_run(), factored out from this file so the
 * HTTP control API's own "pair now" endpoint can trigger the identical
 * sequence -- this file just owns deciding *when* (button press).
 *
 * Deliberately a second thread inside this process, not a third: unlike
 * the documented reason ar8030-lifecycled itself is a separate process
 * from ar8030d (a thread there made blocking client-library RPC calls
 * into the same process serving as the RPC server -- see main.c's own
 * header comment), this thread never touches lifecycle_thread_main's
 * RPC client at all -- only plain sysfs GPIO reads and forking the
 * separate ar8030-pair process -- so that failure mode doesn't apply
 * here. The two threads share only read-only config (hook_dir, cfg_path,
 * role) plus lc_hooks_dispatch(), which is already fork-per-call with no
 * shared mutable state, so both can call it concurrently safely.
 */

#define LC_BIND_POLL_MS     100
#define LC_BIND_DEBOUNCE_MS 50
#define LC_BIND_LOCKOUT_S   5

struct lifecycle_bind_ctx {
    lc_config_t cfg;
};

lifecycle_bind_ctx* lifecycle_bind_init(const lc_config_t* cfg)
{
    if (cfg->bind_gpio < 0) {
        return NULL;
    }
    lifecycle_bind_ctx* ctx = (lifecycle_bind_ctx*)calloc(1, sizeof(*ctx));
    if (!ctx) {
        return NULL;
    }
    ctx->cfg = *cfg;
    return ctx;
}

static int read_button(int gpio)
{
    int val = 1; /* idle=1, pressed=0 -- confirmed by hand on real hardware */
    if (lc_gpio_read(gpio, &val) != 0) {
        return 1;
    }
    return val;
}

void* lifecycle_bind_thread_main(void* arg)
{
    lifecycle_bind_ctx* ctx  = (lifecycle_bind_ctx*)arg;
    int                  gpio = ctx->cfg.bind_gpio;

    lc_log("lifecycle: bind: watching gpio%d for the physical bind button", gpio);

    while (!lifecycle_shutdown_requested()) {
        usleep(LC_BIND_POLL_MS * 1000);
        if (read_button(gpio) != 0) {
            continue;
        }

        /* Debounce: confirm it's still pressed after a short settle, not
         * switch bounce or a transient glitch. */
        usleep(LC_BIND_DEBOUNCE_MS * 1000);
        if (read_button(gpio) != 0) {
            continue;
        }

        lc_log("lifecycle: bind: button pressed, waiting for release");
        while (read_button(gpio) == 0 && !lifecycle_shutdown_requested()) {
            usleep(LC_BIND_POLL_MS * 1000);
        }
        if (lifecycle_shutdown_requested()) {
            break;
        }

        lc_log("lifecycle: bind: button released, pairing");
        int slot = -1;
        int rc   = lc_pair_run(&ctx->cfg, &slot, NULL);
        lc_log("lifecycle: bind: pair %s (slot=%d)", rc == 0 ? "succeeded" : "failed", slot);

        /* Ignore further presses for a bit rather than re-triggering on
         * any residual bounce right after we already acted. */
        sleep(LC_BIND_LOCKOUT_S);
    }

    return NULL;
}
