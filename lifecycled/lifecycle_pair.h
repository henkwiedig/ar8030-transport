#ifndef __LIFECYCLE_PAIR_H__
#define __LIFECYCLE_PAIR_H__
#ifdef __cplusplus
extern "C" {
#endif

#include "ar8030.h"
#include "bb_api.h"
#include "lifecycle.h"
#include <stdint.h>

/*
 * Persist/read/re-apply only -- the actual pairing handshake
 * (PRJ_CMD_EVENT_PAIR dispatch) is the SDK's own ar8030-pair binary's
 * job (dev_helper/bb_pair/txg_bb_pair.cpp, package/ar8030), invoked by
 * lifecycle_bind.c. Exact behavior matters most here (see
 * lifecycle_pair.c's own comments): an AP with an empty candidate list
 * means "accept anyone", so getting the paired/never-paired distinction
 * wrong silently auto-binds to a stranger.
 */

/* Writes the paired peer's MAC into cfg_path's baseband.basic.{ap.candidate|
 * dev.ap_mac} section (atomic .tmp+rename), then the <cfg_path>.paired
 * marker, then a best-effort live BB_SET_CANDIDATES/BB_SET_AP_MAC update so
 * this session doesn't itself need a reboot. Returns 0 on success. */
int lc_pair_persist(bb_dev_handle_t* handle, const char* cfg_path, int slot, uint8_t role, const bb_mac_t* mac);

/* Non-zero if cfg_path.paired exists -- this unit has been through a real
 * pair before (as opposed to a factory-default candidate list, which on
 * the AP side means "accept anyone"). */
int lc_pair_has_been_paired(const char* cfg_path);

/* Re-reads whatever peer this unit last persisted (the same
 * baseband.basic.{ap.candidate|dev.ap_mac} section lc_pair_persist()
 * itself writes) and pushes it to the chip via BB_SET_CANDIDATES/
 * BB_SET_AP_MAC -- no PRJ_CMD_EVENT_PAIR dispatch involved. See this
 * file's own comment on lc_pair_apply_known_candidate() for why this
 * needs to run on every startup, not just once at the original pair.
 * Returns 0 if a peer was found and pushed, -1 if there's nothing
 * persisted yet or on error -- both non-fatal to the caller. */
int lc_pair_apply_known_candidate(bb_dev_handle_t* handle, const char* cfg_path, lc_role_e role);

/* Runs the SDK's own ar8030-pair binary (dev_helper/bb_pair) for the
 * actual pairing handshake and drives hooks.d/{pairing,connected,idle}
 * from its result -- the exact sequence the physical bind button
 * (lifecycle_bind.c) already runs, factored out here so the HTTP control
 * API's own "pair now" endpoint can trigger the identical sequence.
 * NOT the same thing as the old, removed lc_pair_trigger() this file's
 * own header comment mentions (a from-scratch BB_SET_PRJ_DISPATCH
 * reimplementation) -- named lc_pair_run() specifically to avoid being
 * confused with that; this one still just shells out to ar8030-pair like
 * lifecycle_bind.c always has.
 *
 * Blocks until ar8030-pair exits (bounded by its own handshake timeout).
 * out_slot/out_mac (either may be NULL) receive the connected slot/peer
 * MAC on success, unset (-1 / zeroed) otherwise. Safe to call
 * concurrently with the bind-button thread and from multiple HTTP
 * requests at once -- see lifecycle_bind.c's own header comment on why
 * concurrent hook dispatch/ar8030-pair invocation is already a supported
 * pattern (fork-per-call, no shared mutable state). Returns 0 if
 * ar8030-pair exited successfully, -1 otherwise. */
int lc_pair_run(const lc_config_t* cfg, int* out_slot, bb_mac_t* out_mac);

#ifdef __cplusplus
}
#endif
#endif
