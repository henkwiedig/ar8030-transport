#include "lifecycle_bind.h"
#include "bb_api.h"
#include "lc_log.h"
#include "lifecycle_gpio.h"
#include "lifecycle_hooks.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
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

/* Direct fork+pipe+execlp, no shell -- matches lc_hooks_dispatch's own
 * no-shell convention. Captures ar8030-pair's stdout+stderr (2>&1,
 * matching the shell script this replaces) into buf. Returns its exit
 * code, or -1 on a fork/pipe/wait failure. */
static int run_pair_tool(const char* cfg_path, char* buf, size_t buf_sz)
{
    int pipefd[2];
    if (pipe(pipefd) != 0) {
        return -1;
    }

    pid_t pid = fork();
    if (pid < 0) {
        close(pipefd[0]);
        close(pipefd[1]);
        return -1;
    }
    if (pid == 0) {
        close(pipefd[0]);
        dup2(pipefd[1], STDOUT_FILENO);
        dup2(pipefd[1], STDERR_FILENO);
        close(pipefd[1]);
        execlp("ar8030-pair", "ar8030-pair", "-c", cfg_path, (char*)NULL);
        _exit(127);
    }

    close(pipefd[1]);
    size_t  total = 0;
    ssize_t n;
    while (total + 1 < buf_sz && (n = read(pipefd[0], buf + total, buf_sz - 1 - total)) > 0) {
        total += (size_t)n;
    }
    buf[total] = 0;
    close(pipefd[0]);

    int status = 0;
    waitpid(pid, &status, 0);
    if (!WIFEXITED(status)) {
        return -1;
    }
    return WEXITSTATUS(status);
}

/* Parses "pair: peer <8 hex chars> connected on slot <N>" out of
 * ar8030-pair's own captured stdout (dev_helper/bb_pair/txg_bb_pair.cpp's
 * own log line, confirmed live against real device output while
 * debugging this move) into a slot + 4-byte bb_mac_t (BB_MAC_LEN -- this
 * chip's MAC is not a real 6-byte Ethernet address). Returns 0 on a
 * match, -1 otherwise (caller falls back to slot=-1 and an unset mac,
 * matching lc_hooks_dispatch's own NULL-mac convention). */
static int parse_pair_output(const char* buf, int* out_slot, bb_mac_t* out_mac)
{
    const char* p = strstr(buf, "pair: peer ");
    if (!p) {
        return -1;
    }
    p += strlen("pair: peer ");

    char hex[9] = {0};
    int  slot   = -1;
    if (sscanf(p, "%8[0-9a-fA-F] connected on slot %d", hex, &slot) != 2 || strlen(hex) != 8) {
        return -1;
    }

    memset(out_mac, 0, sizeof(*out_mac));
    for (int i = 0; i < BB_MAC_LEN && i < 4; i++) {
        unsigned int byte = 0;
        sscanf(hex + i * 2, "%2x", &byte);
        out_mac->addr[i] = (uint8_t)byte;
    }
    *out_slot = slot;
    return 0;
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
        lc_hooks_dispatch(ctx->cfg.hook_dir, "pairing", ctx->cfg.role, -1, NULL);

        char out[1024];
        int  rc = run_pair_tool(ctx->cfg.cfg_path, out, sizeof(out));
        lc_log("lifecycle: bind: ar8030-pair exited %d, output: %s", rc, out);

        if (rc == 0) {
            int      slot = -1;
            bb_mac_t mac;
            memset(&mac, 0, sizeof(mac));
            if (parse_pair_output(out, &slot, &mac) == 0) {
                lc_log("lifecycle: bind: pair succeeded (slot=%d), driving connected hooks directly", slot);
                lc_hooks_dispatch(ctx->cfg.hook_dir, "connected", ctx->cfg.role, slot, &mac);
            } else {
                lc_log("lifecycle: bind: pair succeeded but couldn't parse its output, driving connected hooks with slot/mac unknown");
                lc_hooks_dispatch(ctx->cfg.hook_dir, "connected", ctx->cfg.role, -1, NULL);
            }
        } else {
            lc_log("lifecycle: bind: pair failed (rc=%d), leaving current link alone", rc);
            /* Nothing else resets the LED after a failed attempt --
             * lifecycle_thread_main's own "idle" hook only ever fires
             * once per process lifetime (its own idle_hook_fired latch),
             * so without this the board would be stuck on the fast
             * "binding mode" blink despite not actually binding anything
             * any more. hooks.d/idle/ itself decides red-vs-slow-blink
             * from the ".paired" marker file, so this correctly falls
             * back to "searching" rather than "never bound" when
             * re-binding an already-paired unit fails -- and if the old
             * link is actually still up, lifecycle_thread_main's own
             * "connected" hook corrects the LED again within a few
             * seconds regardless. */
            lc_hooks_dispatch(ctx->cfg.hook_dir, "idle", ctx->cfg.role, -1, NULL);
        }

        /* Ignore further presses for a bit rather than re-triggering on
         * any residual bounce right after we already acted. */
        sleep(LC_BIND_LOCKOUT_S);
    }

    return NULL;
}
