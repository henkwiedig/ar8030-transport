#include "lifecycle_client.h"
#include "lc_log.h"
#include "lifecycle.h"
#include <string.h>
#include <unistd.h>

#define LC_CLIENT_CONNECT_RETRIES        200
#define LC_CLIENT_CONNECT_DELAY_START_S  2
#define LC_CLIENT_CONNECT_DELAY_CAP_S    60

/*
 * Exponential backoff (2s, 4s, 8s, ..., capped at 60s), not the flat 2s
 * this used to sleep. Confirmed live: bb_dev_getlist()'s own retry (up to
 * 4 attempts/150ms apart, needed so a one-shot CLI tool survives a single
 * missed reply window) stacked on top of a flat-2s outer loop sustains
 * roughly 1-1.3 new loopback connections/sec against the daemon. During an
 * extended outage (the daemon's SDIO transport to the chip itself down,
 * which can last from seconds to indefinitely) that sustained rate was
 * enough to permanently wedge the daemon's own connection handling --
 * observed live as a 300+ vs 60-ish start/close imbalance in its log and
 * 100+ accumulated threads, at which point the daemon's SDIO recovery
 * itself froze too and never came back without a manual restart. Backing
 * off caps how much load a prolonged outage can put on the daemon while
 * it's already trying to recover the physical link -- exactly the
 * mid-flight scenario this needs to not make worse, not just a rate limit
 * for its own sake.
 */
int lc_client_connect(lc_client_t* client, int rpc_port)
{
    memset(client, 0, sizeof(*client));

    int delay_s = LC_CLIENT_CONNECT_DELAY_START_S;
    for (int attempt = 0; attempt < LC_CLIENT_CONNECT_RETRIES; attempt++) {
        if (lifecycle_shutdown_requested()) {
            return -1;
        }

        if (bb_host_connect(&client->host, "127.0.0.1", rpc_port) == 0) {
            bb_dev_list_t* devs   = NULL;
            int            devcnt = bb_dev_getlist(client->host, &devs);
            if (devcnt > 0) {
                /* bb_dev_list_t is typedef'd to bb_dev_t* (see bb_api.h,
                 * non-SWIG branch) -- it's already a flat array of device
                 * pointers, index it directly rather than via a wrapper. */
                bb_dev_t* pdev = devs[0];
                client->handle = pdev ? bb_dev_open(pdev) : NULL;
                bb_dev_freelist(devs);
                if (client->handle) {
                    lc_log("lifecycle: loopback client connected (attempt %d)", attempt + 1);
                    return 0;
                }
                lc_log("lifecycle: bb_dev_open failed, retrying");
            } else {
                lc_log("lifecycle: no AR8030 device known to the daemon yet, retrying in %ds", delay_s);
            }
            bb_host_disconnect(client->host);
            client->host = NULL;
        } else {
            lc_log("lifecycle: loopback connect to 127.0.0.1:%d failed, retrying in %ds", rpc_port, delay_s);
        }

        sleep(delay_s);
        if (delay_s < LC_CLIENT_CONNECT_DELAY_CAP_S) {
            delay_s = delay_s * 2 < LC_CLIENT_CONNECT_DELAY_CAP_S ? delay_s * 2 : LC_CLIENT_CONNECT_DELAY_CAP_S;
        }
    }

    lc_log("lifecycle: giving up on loopback client connection after %d attempts", LC_CLIENT_CONNECT_RETRIES);
    return -1;
}

/*
 * Bounded, not bb_ioctl()'s default infinite timeout: confirmed live that
 * the daemon's SDIO transport to the physical chip can fault (and force-
 * close every in-flight session, this one included) within the first
 * ~10-15s after every fresh chip enumeration -- most likely the chip still
 * completing its own firmware boot over the same bus, per the "upgrade
 * spl/bb_cfg success" sequence in dmesg right before it. An infinite-
 * timeout subscribe landing in that window hangs this thread forever with
 * no way back; a bounded one surfaces as an ordinary failure the caller
 * can retry. Known residual cost: a timed-out attempt's dedicated
 * connection (bb_gethandle() opens a new loopback socket + reader thread
 * per subscribe call) is abandoned rather than cleanly torn down -- one
 * leaked thread/socket per retry, bounded in practice because retries are
 * infrequent and the fault window is narrow.
 */
#define LC_SUBSCRIBE_TIMEOUT_MS 5000

int lc_client_subscribe_events(lc_client_t* client)
{
    bb_set_event_callback_t cb;

    /* user left NULL: there is exactly one lifecycle instance per daemon
     * process (one chip, one supervisor), so the callbacks in lifecycle.c
     * just update their own module-level shared state directly rather
     * than threading a context pointer through the client library's own
     * callback plumbing for no real benefit. */
    memset(&cb, 0, sizeof(cb));
    cb.event    = BB_EVENT_LINK_STATE;
    cb.callback = lc_on_link_state_event;
    cb.user     = NULL;
    int ret1    = bb_ioctl_ex(client->handle, BB_SET_EVENT_SUBSCRIBE, &cb, NULL, LC_SUBSCRIBE_TIMEOUT_MS);

    memset(&cb, 0, sizeof(cb));
    cb.event    = BB_EVENT_PAIR_RESULT;
    cb.callback = lc_on_pair_result_event;
    cb.user     = NULL;
    int ret2    = bb_ioctl_ex(client->handle, BB_SET_EVENT_SUBSCRIBE, &cb, NULL, LC_SUBSCRIBE_TIMEOUT_MS);

    if (ret1 != 0 || ret2 != 0) {
        lc_log("lifecycle: event subscribe failed (link_state=%d, pair_result=%d)", ret1, ret2);
        return -1;
    }
    lc_log("lifecycle: subscribed to BB_EVENT_LINK_STATE and BB_EVENT_PAIR_RESULT");
    return 0;
}

void lc_client_disconnect(lc_client_t* client)
{
    if (client->handle) {
        bb_dev_close(client->handle);
        client->handle = NULL;
    }
    if (client->host) {
        bb_host_disconnect(client->host);
        client->host = NULL;
    }
}
