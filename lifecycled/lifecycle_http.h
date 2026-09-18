#ifndef LIFECYCLE_HTTP_H
#define LIFECYCLE_HTTP_H
#ifdef __cplusplus
extern "C" {
#endif

#include "lifecycle.h"
#include <stdint.h>

/*
 * Small REST + HTML control API for ar8030-lifecycled, opt-in via
 * --http-port (0 = disabled, the default -- see main.c's usage()). Exists
 * so an operator (or another tool -- e.g. a ground-side dashboard that
 * wants one place to check both ends of the link) can query link state
 * and drive pairing/tuning with plain `curl`, from either the air or the
 * ground unit, instead of shelling into the device -- see this project's
 * README, section "HTTP control API", for the full endpoint list and the
 * curl examples.
 *
 * Deliberately minimal: one worker thread (requests are handled one at a
 * time -- this is a low-traffic control plane, not a video path, so
 * simplicity wins over concurrency), HTTP/1.0, no request body parsing
 * (every endpoint takes its arguments as query parameters, curl-friendly:
 * `curl -X POST 'http://host:PORT/api/v1/bandwidth?mhz=20'`), no auth --
 * this is meant for a trusted local/link network the same way this
 * project's other loopback-style APIs already are (waybeam's own
 * /api/v1/live/set, ar8030d's own RPC port), not for exposure beyond
 * that. Binding 0.0.0.0 (the default once a port is set) rather than
 * loopback-only is deliberate: the whole point is reaching this from the
 * *other* end of the link, not just the box it runs on.
 *
 * Two families of endpoint:
 *
 *  - /api/v1/status and /api/v1/pair and /api/v1/bandwidth are owned by
 *    this daemon's own lifecycle state -- status is a snapshot of
 *    lifecycle.c's own state machine (lifecycle_get_status()), pair
 *    triggers the exact same fork+exec+hook-dispatch sequence the
 *    physical bind button already runs (lifecycle_pair.c's
 *    lc_pair_run()), and bandwidth is *queued* for the lifecycle thread
 *    to persist+apply on its own next tick (lifecycle_request_
 *    bandwidth()) rather than applied directly here -- see lifecycle.h's
 *    own comment on why ctx->client.handle's bb_ioctl calls must stay
 *    single-threaded (the lifecycle thread's own).
 *
 *  - /api/v1/linkctl is a generic, allow-listed passthrough to the
 *    standalone `ar8030-linkctl` binary (linkctl/main.c) -- every one of
 *    its own subcommands (status, channel, mcs, power, freq,
 *    force-close-*, ...), reachable the same way `curl` would invoke the
 *    CLI directly. Safe to add this way specifically because every
 *    linkctl invocation opens its own independent, one-shot connection
 *    to ar8030d and exits (see linkctl/main.c's own header comment) --
 *    it never touches this daemon's own ctx->client at all, so unlike
 *    bandwidth above, it needs no mailbox and can run directly on this
 *    module's own worker thread, concurrently with everything else this
 *    process is doing (matching lc_pair_run()'s own already-established
 *    "fork-per-call, no shared mutable state" safety argument).
 *    IMPORTANT: a raw `linkctl bandwidth` call through this passthrough
 *    is a ONE-SHOT override -- lifecycle.c's own periodic tuning
 *    re-assert (its "Re-apply, not detect-and-persist" logic) will
 *    silently overwrite it with whatever was last persisted/queued
 *    within LC_POLL_FALLBACK_S seconds while connected. Use
 *    /api/v1/bandwidth instead for anything meant to stick.
 */

typedef struct lifecycle_http_ctx lifecycle_http_ctx;

/* Binds and starts serving on bind_addr:port in a dedicated thread.
 * bind_addr may be NULL/empty for "0.0.0.0" (every interface). Returns
 * NULL on any setup failure (logged) -- non-fatal to the caller, matching
 * lifecycle_bind_init()'s own "board doesn't have this feature"
 * convention. */
lifecycle_http_ctx* lifecycle_http_start(lifecycle_ctx* lc, const char* bind_addr, uint16_t port);

/* Stops the worker thread and closes the listening socket. Safe to call
 * with NULL (no-op). Never frees http -- see this function's own comment
 * in lifecycle_http.c for why; called once, right before process exit. */
void lifecycle_http_stop(lifecycle_http_ctx* http);

#ifdef __cplusplus
}
#endif
#endif
