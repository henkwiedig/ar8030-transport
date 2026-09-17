#ifndef __LIFECYCLE_CLIENT_H__
#define __LIFECYCLE_CLIENT_H__
#ifdef __cplusplus
extern "C" {
#endif

#include "ar8030.h"
#include "bb_dev.h"

/*
 * The daemon becomes its own RPC client over loopback, exactly the same
 * way ar8030-linkctl/ar8030-pair/ar8030-status already do (bb_host_connect
 * -> bb_dev_getlist -> bb_dev_open), so the lifecycle module can reuse the
 * existing, proven client library (bb_ioctl, event subscription) instead
 * of reaching into the daemon's own internal RPC dispatch structures.
 */
typedef struct {
    bb_host_t*       host;
    bb_dev_handle_t* handle;
} lc_client_t;

/* Connects to 127.0.0.1:rpc_port, resolves the (single) registered AR8030
 * device, and opens a handle to it. Retries every 2s -- this is a pure
 * bring-up ordering concern (rpc_init()'s listener needs to already be
 * accepting), not something expected to fail in steady state, since both
 * ends are the same process. Returns 0 on success. */
int lc_client_connect(lc_client_t* client, int rpc_port);

void lc_client_disconnect(lc_client_t* client);

/* Subscribes to BB_EVENT_LINK_STATE and BB_EVENT_PAIR_RESULT via
 * BB_SET_EVENT_SUBSCRIBE (bb_ioctl already special-cases these two request
 * IDs to route through the client library's own dedicated callback
 * session -- see ar8030.c -- rather than the generic ioctl dispatch table,
 * so no daemon-side RPC changes are needed for this to work). The
 * callbacks run on the client library's own reader thread, not the
 * lifecycle thread -- they only update the shared state in lifecycle.c
 * behind its own lock, never block or call back into bb_ioctl themselves.
 * Returns 0 if both subscriptions succeeded. */
int lc_client_subscribe_events(lc_client_t* client);

#ifdef __cplusplus
}
#endif
#endif
