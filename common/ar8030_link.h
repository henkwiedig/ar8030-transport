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
    bb_dev_handle_t *dev;
    int sockfd; /* -1 until ar8030_link_open_socket() succeeds */
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

#endif /* AR8030_TRANSPORT_LINK_H */
