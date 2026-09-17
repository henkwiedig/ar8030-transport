#ifndef LIFECYCLE_HOOKS_H
#define LIFECYCLE_HOOKS_H
#ifdef __cplusplus
extern "C" {
#endif

#include "bb_api.h"
#include "lifecycle.h"

/*
 * run-parts-style hook dispatch, settled on over udev (no existing
 * infrastructure, would need new kernel-side sysfs/uevent plumbing and a
 * resident listener) and status-file-plus-watchers (needs a resident
 * poller per consumer -- recreates the exact "pile of watchdogs" problem
 * this whole daemon exists to fix). The daemon directly executes hook
 * scripts on each real state transition: zero steady-state cost, paid
 * only when something actually changes.
 */

/* Call once at process startup, before any dispatch: installs SIGCHLD as
 * SIG_IGN so forked hook scripts are auto-reaped without this process
 * ever having to wait() for them. */
void lc_hooks_init(void);

/* Runs every executable, regular file directly under <hook_dir>/<event>/
 * in lexical order (run-parts convention) -- e.g. "connected", "dropped",
 * "pairing", "idle". Each one is fork()+exec()'d with argv[1] = event and
 * environment variables AR8030_EVENT, AR8030_ROLE ("ap"/"dev"),
 * AR8030_SLOT, and AR8030_PEER_MAC (colon-hex, BB_MAC_LEN=4 bytes --
 * this chip's own MAC is not a real 6-byte Ethernet address -- or
 * "00:00:00:00" if peer_mac is NULL) set, so a one-line hook can use whichever's
 * convenient. Never blocks on the scripts themselves -- SIGCHLD is
 * ignored (see lc_hooks_init()), so a slow or hung hook can never stall
 * the lifecycle thread. Missing <hook_dir>/<event>/ is not an error:
 * hooks are entirely optional, most boards will have none at all. */
void lc_hooks_dispatch(const char* hook_dir, const char* event, lc_role_e role, int slot, const bb_mac_t* peer_mac);

#ifdef __cplusplus
}
#endif
#endif
