#ifndef AR8030_TRANSPORT_LINK_H
#define AR8030_TRANSPORT_LINK_H

/*
 * Thin wrapper over the ar8030 SDK's connect sequence
 * (bb_host_connect -> bb_dev_getlist -> bb_dev_open), shared by tx and rx.
 * Mirrors the exact pattern sbc-groundstations/package/ar8030's own
 * ar8030-status.c uses, since that is this project's house precedent for
 * a standalone tool talking to ar8030d.
 *
 * ar8030d (the daemon) is a separate process started by its own init
 * script (S60ar8030 on air, S97ar8030 on ground) and this tool has no
 * ordering guarantee relative to it, so every connect step here is meant
 * to be called from a retry loop -- see ar8030_link_connect_retry().
 */

#include <stdint.h>

#include "bb_api.h"

typedef struct {
    bb_host_t *host;
    /* Always read/written via __atomic_*() once a link may be
     * reconnected mid-session (ar8030_link_reconnect_retry() below) --
     * tx/bitrate_ctl.c's own thread reads this concurrently for its
     * BB_GET_MCS poll, and a plain access here would be the exact same
     * shape of use-after-free race the frame-shm ring reattach fix hit
     * on its first live test (see README.md). Every function in this
     * file already does this; a caller that dereferences link->dev
     * directly instead of going through ar8030_link_is_alive() or its
     * own bb_ioctl() call must load it atomically first if it runs
     * anywhere near a concurrent reconnect. */
    bb_dev_handle_t *dev;
    int sockfd; /* -1 until ar8030_link_open_socket() succeeds. Not shared
                 * with another thread anywhere in this codebase (only
                 * bitrate_ctl.c runs concurrently with a link owner, and
                 * it never touches sockfd), so no atomic access needed. */
} ar8030_link_t;

/* One-shot connect attempt: bb_host_connect -> bb_dev_getlist ->
 * bb_dev_open on the first device the daemon reports. Returns 0 on
 * success, -1 otherwise (link is left zeroed on failure, safe to retry). */
int ar8030_link_connect(ar8030_link_t *link, const char *daemon_ip, int daemon_port);

/* Retries ar8030_link_connect() every retry_ms until it succeeds or
 * *stop_flag becomes non-zero (checked between attempts, e.g. from a
 * signal handler). Returns 0 on success, -1 if stop_flag fired first. */
int ar8030_link_connect_retry(ar8030_link_t *link, const char *daemon_ip, int daemon_port,
                               int retry_ms, const volatile int *stop_flag);

/* Opens the data-plane bb_socket used for the actual video chunk traffic.
 * slot is BB_SLOT_AP on the DEV-role side (ground) addressing its AP peer,
 * or the assigned data slot on the AP-role side (air) -- see each app's
 * README for the slot each role uses. Stores the fd in link->sockfd.
 * Returns 0 on success, -1 otherwise. */
int ar8030_link_open_socket(ar8030_link_t *link, bb_slot_e slot, uint32_t port, uint32_t flag,
                             bb_sock_opt_t *opt);

/* Closes the data socket (if open) and disconnects from the daemon.
 * Safe to call on a partially-connected or zeroed link. */
void ar8030_link_close(ar8030_link_t *link);

/* Force-releases one slot/port's socket session state on the daemon side,
 * matching linkctl's own "force-close-socket" command
 * (BB_FORCE_CLS_SOCKET) -- see its own doc comment there for the general
 * caveat ("may desync the daemon's own socket accounting -- only use it
 * to unstick a port nothing else is actively using"). Meant specifically
 * for the moment right after ar8030_link_reconnect_retry() succeeds and
 * before the caller's own ar8030_link_open_socket(): confirmed live that
 * a daemon killed abruptly (not a clean shutdown, e.g. it crashed) never
 * gets the chance to run its own bb_socket_close() teardown, so the new
 * daemon instance's first bb_socket_open() on the same slot/port fails
 * with ret=-1 ("already opened", chip-side state left over from the old
 * session) even though nothing is genuinely still using it -- exactly
 * the situation this ioctl's own caveat describes as safe. Returns 0 on
 * success, -1 if link->dev is NULL or the ioctl itself failed. */
int ar8030_link_force_close_socket(ar8030_link_t *link, bb_slot_e slot, uint32_t port);

/* Same idea (BB_FORCE_CLS_SOCKET_ALL, matching linkctl's own
 * "force-close-all"), but every socket on every slot rather than one --
 * confirmed live as the one that actually works where
 * ar8030_link_force_close_socket() alone returned -2 (did not clear
 * whatever state was actually stuck) on a real daemon in this exact
 * reconnect scenario. Broader blast radius in general (README's own
 * caveat on linkctl's "force-close-all" applies here too), but for this
 * project's actual deployment -- one slot, one socket, ever -- it is not
 * meaningfully different from closing just the one this caller cares
 * about. Returns 0 on success, -1 if link->dev is NULL or the ioctl
 * itself failed. */
int ar8030_link_force_close_all_sockets(ar8030_link_t *link);

/* Cheap liveness probe for the daemon connection this link already has
 * open (a single BB_GET_STATUS round trip -- the same always-registered,
 * minimal-payload ioctl linkctl's own status dump and tx/main.c's
 * resolve_connected_slot() already rely on). Returns 1 if the daemon
 * answered, 0 if link->dev is NULL or the call itself failed.
 *
 * This exists as a *separate* check from data-path timeouts
 * deliberately: a bb_socket_write()/read() timeout looks identical
 * whether the RF link is merely saturated (daemon and chip both fine,
 * will clear on its own) or the daemon process itself crashed/restarted
 * (nothing will clear on its own -- every call on this link will keep
 * failing forever without a fresh ar8030_link_reconnect_retry()).
 * Inferring "daemon is dead" from a run of data-path failures conflates
 * these two very different situations; asking the daemon directly on its
 * own timer does not. */
int ar8030_link_is_alive(ar8030_link_t *link);

/* Full daemon reconnect: closes whatever's left of the old connection
 * (safe even if it's already partially broken) and retries
 * ar8030_link_connect() every retry_ms until it succeeds or *stop_flag
 * fires, same semantics as ar8030_link_connect_retry().
 *
 * Deliberately does NOT reopen the data socket -- slot/port/flags differ
 * by caller role (tx re-resolves wherever its peer's connected slot
 * lands now, which may not be where it was before the daemon restarted;
 * rx always targets the fixed BB_SLOT_AP) and are the caller's own
 * concern already, same as the original startup sequence in each of
 * tx/main.c and rx/main.c. */
int ar8030_link_reconnect_retry(ar8030_link_t *link, const char *daemon_ip, int daemon_port,
                                 int retry_ms, const volatile int *stop_flag);

#endif /* AR8030_TRANSPORT_LINK_H */
