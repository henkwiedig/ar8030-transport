/*
 * ar8030-linkctl -- manual AR8030 baseband link control (bandwidth,
 * channel, MCS, frequency), plus a status dump.
 *
 * Grew out of a throwaway diagnostic tool during the bandwidth-plateau
 * investigation documented in README.md's "Bandwidth: the real
 * bottleneck" section: on real hardware, BB_GET_MCS reported an
 * excellent MCS (matching the peer's own rx_mcs) but a theoretical
 * throughput far below what that MCS should give -- because channel
 * *bandwidth* (a separate dimension from MCS in this chip: 1.25/2.5/5/
 * 10/20/40 MHz gears, see bb_bandwidth_e) was pinned at its narrowest
 * gear. The auto-widen mechanism (BB_CFG_SLOT_RX_MCS's bw_auto policy)
 * turned out to be unimplemented in this SDK build's client library
 * ("req 5 not found"), but BB_SET_BANDWIDTH -- a direct manual override
 * -- works and is registered. This tool is that fix made durable and
 * reusable, since bandwidth/channel control is exactly the kind of
 * thing this project expects to need again (different antennas, a
 * regulatory-domain change, a future auto-tuning daemon).
 *
 * Every command talks to the local ar8030d exactly like ar8030-status
 * does (bb_host_connect -> bb_dev_getlist -> bb_dev_open), issues one
 * bb_ioctl, and exits -- no persistent state of its own. Nothing this
 * tool sets survives a chip reboot/re-probe; call it again after
 * reconnecting (see -w/--wait-connect, and S60ar8030/S97ar8030's own
 * autoreconnect hook).
 */
#include "ar8030.h"
#include "bb_api.h"
#include "bb_dev.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static void usage(const char *argv0)
{
    fprintf(stderr,
            "usage: %s <command> [args]\n"
            "\n"
            "  status [-s slot]\n"
            "      Dump BB_GET_STATUS (per-user tx/rx mcs+bandwidth+freq, link\n"
            "      state) and BB_GET_MCS for the given slot (default 0).\n"
            "\n"
            "  bandwidth <mhz> [-d tx|rx] [-s slot] [-w seconds]\n"
            "      Manually set channel bandwidth. <mhz> is one of\n"
            "      1.25 2.5 5 10 20 40, or a raw bb_bandwidth_e index 0-5.\n"
            "\n"
            "  channel-mode auto|manual\n"
            "      Channel working mode by itself, with no channel change --\n"
            "      'channel' below already forces manual mode as part of its\n"
            "      own sequence, so you only need this on its own to revert to\n"
            "      'auto' afterwards.\n"
            "\n"
            "  channel <index> [-s slot|auto] [-w seconds]\n"
            "      Actually change channel: forces manual channel mode, sets\n"
            "      this side's RX channel to the given index into the\n"
            "      pre-configured channel table (ar8030.json's\n"
            "      baseband.basic.<role>.channel.freq[]), then pushes the same\n"
            "      channel to the connected peer over the still-live link\n"
            "      (BB_SET_REMOTE) so it retunes too -- matching the vendor's\n"
            "      own reference sequence, since changing only this side would\n"
            "      just desync the link. -s picks which connected peer to\n"
            "      notify (default 0, or 'auto' to resolve it live). Confirmed\n"
            "      on real hardware: this triggers a synchronized \"safe hop\"\n"
            "      (see ar8030d's own log: bb_phy_chan_hop_trigger /\n"
            "      bb_link_sync_tx_proc+bb_link_sync_rx_proc handshake /\n"
            "      \"safe hop ok!\") that retunes both radios together without\n"
            "      dropping CONNECT state -- but if the final BB_SET_REMOTE\n"
            "      push fails after the local change already applied (e.g. the\n"
            "      peer missed the handshake), the link IS left desynced with\n"
            "      no automatic recovery, and the peer needs its own manual\n"
            "      channel change to match.\n"
            "\n"
            "  mcs-mode auto|manual [-s slot]\n"
            "      MCS control mode -- manual mode is required before 'mcs'\n"
            "      below has any effect.\n"
            "\n"
            "  mcs <value> [-s slot] [-w seconds]\n"
            "      Manually set MCS gear (raw bb_phy_mcs_e index, matching\n"
            "      'status' output's own mcs= numbers).\n"
            "\n"
            "  force-close-socket <slot> <port>\n"
            "      Force the daemon to release one socket's session state.\n"
            "      For recovering a port stuck reporting \"already opened\"/\n"
            "      \"socket open error = 257\" after a client crashed or was\n"
            "      killed without a clean bb_socket_close() -- confirmed on\n"
            "      real hardware to happen (see README's throughput-benchmark\n"
            "      section). May desync the daemon's own socket accounting\n"
            "      (per BB_FORCE_CLS_SOCKET's own doc comment) -- only use it\n"
            "      to unstick a port nothing else is actively using.\n"
            "\n"
            "  force-close-all\n"
            "      Same, but every socket on every slot. Same caveat.\n"
            "\n"
            "  freq <khz> [-u user] [-d tx|rx|both] [-w seconds]\n"
            "      Set a physical user's TX and/or RX carrier frequency\n"
            "      directly, bypassing the channel table.\n"
            "\n"
            "Common flags:\n"
            "  -s slot     target slot, bb_slot_e (default 0), or 'auto' (bandwidth\n"
            "              and channel only) to use whichever slot is actually\n"
            "              CONNECTed -- which slot a peer lands on isn't fixed\n"
            "              across reboots\n"
            "  -d dir      tx, rx, or (freq only) both (default tx)\n"
            "  -u user     physical user index, bb_user_e (default 0)\n"
            "  -w seconds  wait up to this long for the link to reach CONNECT\n"
            "              before issuing the command (default: don't wait --\n"
            "              the command runs immediately and may no-op or fail\n"
            "              if the link isn't up yet)\n",
            argv0);
}

static bb_dev_handle_t *g_hbb;
static bb_host_t *g_phost;
static bb_dev_t **g_devs;

static int connect_daemon(void)
{
    if (bb_host_connect(&g_phost, "127.0.0.1", BB_PORT_DEFAULT)) {
        fprintf(stderr, "linkctl: connect to daemon failed -- is ar8030d running?\n");
        return -1;
    }
    int dev_cnt = bb_dev_getlist(g_phost, &g_devs);
    if (dev_cnt <= 0) {
        fprintf(stderr, "linkctl: no AR8030 device known to the daemon\n");
        return -1;
    }
    g_hbb = bb_dev_open(g_devs[0]);
    if (!g_hbb) {
        fprintf(stderr, "linkctl: bb_dev_open failed\n");
        return -1;
    }
    return 0;
}

static void disconnect_daemon(void)
{
    if (g_hbb)
        bb_dev_close(g_hbb);
    if (g_devs)
        bb_dev_freelist(g_devs);
    if (g_phost)
        bb_host_disconnect(g_phost);
}

/* Polls BB_GET_STATUS until link_status[slot].state == BB_LINK_STATE_CONNECT
 * or timeout_s elapses. Returns 0 if connected, -1 on timeout. Meant for
 * a boot-time init script that wants to apply link settings right after
 * the link comes up, without a fixed guessed sleep -- see S60ar8030's own
 * wait_flag_file() for the equivalent pattern used elsewhere in this
 * project. */
static int wait_for_connect(uint8_t slot, int timeout_s)
{
    for (int waited_ms = 0; waited_ms < timeout_s * 1000; waited_ms += 500) {
        bb_get_status_in_t st_in = { .user_bmp = 0xffff };
        bb_get_status_out_t st_out;
        memset(&st_out, 0, sizeof(st_out));
        if (bb_ioctl(g_hbb, BB_GET_STATUS, &st_in, &st_out) == 0 && slot < BB_SLOT_MAX &&
            st_out.link_status[slot].state == BB_LINK_STATE_CONNECT) {
            return 0;
        }
        usleep(500 * 1000);
    }
    return -1;
}

/* Scans every slot for BB_LINK_STATE_CONNECT instead of checking one fixed
 * slot -- for '-s auto', since which slot a peer actually lands on isn't
 * fixed across reboots (confirmed on real hardware: the same unit paired
 * on slot 2 one boot, slot 0 another). Mirrors bb_pair's own
 * wait_for_paired_peer() slot-scan. Polls at least once even if timeout_s
 * is 0, so '-s auto' without -w still resolves against the current state
 * instead of refusing outright. Returns the connected slot, or -1 if none
 * is up within timeout_s. */
static int resolve_connected_slot(int timeout_s)
{
    int waited_ms = 0;
    do {
        bb_get_status_in_t st_in = { .user_bmp = 0xffff };
        bb_get_status_out_t st_out;
        memset(&st_out, 0, sizeof(st_out));
        if (bb_ioctl(g_hbb, BB_GET_STATUS, &st_in, &st_out) == 0) {
            for (int s = 0; s < BB_SLOT_MAX; s++) {
                if (st_out.link_status[s].state == BB_LINK_STATE_CONNECT) {
                    return s;
                }
            }
        }
        if (waited_ms + 500 >= timeout_s * 1000) {
            break;
        }
        usleep(500 * 1000);
        waited_ms += 500;
    } while (1);
    return -1;
}

/* musl's getopt() -- this project's actual target libc -- does not
 * permute argv the way glibc's does: it stops parsing at the first
 * non-option token instead of scanning past it for later flags. Every
 * subcommand below is documented as "<value> [flags...]", so on real
 * hardware any flag placed after the positional value was being
 * silently dropped with no error (confirmed under qemu-arm with this
 * project's actual cross toolchain) -- including S65ar8030-transport-tx's
 * own production 'bandwidth 20 -d tx -s auto -w 30' invocation, where
 * '-s auto' and '-w 30' never took effect, quietly reintroducing the
 * hardcoded-slot-0 bug '-s auto' exists to fix. Reorders argv in place
 * so every "-f value" pair comes before the leftover positional
 * argument(s), matching what glibc's getopt would already have produced,
 * so the rest of this file can keep calling plain getopt() unmodified.
 * optstring must list every flag the subcommand accepts (all take a
 * value here); anything not in it is left as a positional token, same
 * as plain getopt()'s own handling of an unrecognized flag. */
static void permute_argv(int argc, char **argv, const char *optstring)
{
    char **out = malloc(sizeof(char *) * (size_t)argc);
    char **rest = malloc(sizeof(char *) * (size_t)argc);
    int n = 0, rest_n = 0, i;
    out[n++] = argv[0];
    for (i = 1; i < argc; i++) {
        if (argv[i][0] == '-' && argv[i][1] != '\0') {
            const char *p = strchr(optstring, argv[i][1]);
            out[n++] = argv[i];
            if (p && p[1] == ':' && i + 1 < argc)
                out[n++] = argv[++i];
        } else {
            rest[rest_n++] = argv[i];
        }
    }
    for (i = 0; i < rest_n; i++)
        out[n++] = rest[i];
    memcpy(argv, out, sizeof(char *) * (size_t)argc);
    free(out);
    free(rest);
}

static int parse_bandwidth(const char *s)
{
    if (!strcmp(s, "1.25"))
        return BB_BW_1_25M;
    if (!strcmp(s, "2.5"))
        return BB_BW_2_5M;
    if (!strcmp(s, "5"))
        return BB_BW_5M;
    if (!strcmp(s, "10"))
        return BB_BW_10M;
    if (!strcmp(s, "20"))
        return BB_BW_20M;
    if (!strcmp(s, "40"))
        return BB_BW_40M;
    char *end;
    long v = strtol(s, &end, 10);
    if (*end == '\0' && v >= 0 && v < BB_BW_MAX)
        return (int)v;
    return -1;
}

static int parse_dir(const char *s, int allow_both)
{
    if (!strcmp(s, "tx"))
        return BB_DIR_TX;
    if (!strcmp(s, "rx"))
        return BB_DIR_RX;
    if (allow_both && !strcmp(s, "both"))
        return -2; /* caller-recognized sentinel, not a real bb_dir_e value */
    return -1;
}

static const char *bandwidth_name(uint8_t bw)
{
    switch (bw) {
    case BB_BW_1_25M:
        return "1.25M";
    case BB_BW_2_5M:
        return "2.5M";
    case BB_BW_5M:
        return "5M";
    case BB_BW_10M:
        return "10M";
    case BB_BW_20M:
        return "20M";
    case BB_BW_40M:
        return "40M";
    default:
        return "?";
    }
}

static const char *link_state_name(uint8_t state)
{
    switch (state) {
    case BB_LINK_STATE_IDLE:
        return "IDLE";
    case BB_LINK_STATE_LOCK:
        return "LOCK";
    case BB_LINK_STATE_CONNECT:
        return "CONNECT";
    default:
        return "UNKNOWN";
    }
}

static const char *role_name(uint8_t role)
{
    switch (role) {
    case BB_ROLE_AP:
        return "AP";
    case BB_ROLE_DEV:
        return "DEV";
    default:
        return "unknown";
    }
}

static const char *bb_mode_name(uint8_t mode)
{
    switch (mode) {
    case BB_MODE_SINGLE_USER:
        return "single-user";
    case BB_MODE_MULTI_USER:
        return "multi-user";
    case BB_MODE_RELAY:
        return "relay";
    case BB_MODE_DIRECTOR:
        return "director";
    default:
        return "unknown";
    }
}

/* cfg_sbmp/rt_sbmp are bitmasks over bb_slot_e (bit N = slot N). Printed as
 * a plain slot list instead of raw hex so "which slots actually exist"
 * doesn't require the reader to decode a bitmap by hand. */
static void print_slot_list(const char *label, uint8_t bmp)
{
    printf("%s:", label);
    int any = 0;
    for (int s = 0; s < BB_SLOT_MAX; s++) {
        if (bmp & (1u << s)) {
            printf(" %d", s);
            any = 1;
        }
    }
    if (!any)
        printf(" none");
    printf("\n");
}

static int cmd_status(int argc, char **argv)
{
    int slot = 0;
    int opt;
    while ((opt = getopt(argc, argv, "s:")) != -1) {
        if (opt == 's')
            slot = atoi(optarg);
        else
            return 1;
    }

    bb_get_status_in_t st_in = { .user_bmp = 0xffff };
    bb_get_status_out_t st_out;
    memset(&st_out, 0, sizeof(st_out));
    if (bb_ioctl(g_hbb, BB_GET_STATUS, &st_in, &st_out)) {
        fprintf(stderr, "linkctl: BB_GET_STATUS failed\n");
        return 1;
    }

    printf("role=%s mode=%s\n", role_name(st_out.role), bb_mode_name(st_out.mode));

    /* Deliberately NOT calling BB_GET_CHAN_INFO here. Its output struct
     * (bb_get_chan_info_out_t) has fixed-size freq[]/power[] arrays sized
     * BB_CONFIG_MAX_CHAN_NUM (32), but this project's own ar8030.json
     * configures 42 channels -- confirmed on real hardware (a fully
     * self-consistent daemon+lib+linkctl build, single flash, no version
     * skew) that this overflows the reply into whatever's next on the
     * stack and segfaults, deterministically, every call. The
     * auto_mode/work_chan fields (before the arrays in the struct) come
     * back correct right up until the crash -- it's specifically the
     * fixed 32-slot arrays choking on 42 configured channels. There is
     * no way to safely call this ioctl on this device's config; if
     * channel-mode/working-channel display is wanted again, it needs a
     * fix on the daemon/SDK side (or a channel table trimmed to <=32
     * entries), not a client-side workaround. */

    print_slot_list("configured slots", st_out.cfg_sbmp);
    print_slot_list("active slots", st_out.rt_sbmp);
    for (int s = 0; s < BB_SLOT_MAX; s++) {
        /* Slots outside cfg_sbmp have no real backing state -- the
         * daemon's BB_GET_STATUS reply doesn't zero them, so without this
         * filter they print CONNECT with a leftover/garbage peer address
         * and rx_mcs (observed on real hardware: rx_mcs stuck at
         * BB_PHY_MCS_NEG_1, the reset default, on every such slot). */
        if (!(st_out.cfg_sbmp & (1u << s)))
            continue;
        bb_link_status_t *ls = &st_out.link_status[s];
        if (ls->state >= BB_LINK_STATE_MAX)
            continue;
        printf("slot %d: %-8s peer=%02x:%02x:%02x:%02x rx_mcs=%u\n", s, link_state_name(ls->state),
               ls->peer_mac.addr[0], ls->peer_mac.addr[1], ls->peer_mac.addr[2], ls->peer_mac.addr[3],
               ls->rx_mcs);
    }
    for (int u = 0; u < BB_DATA_USER_MAX; u++) {
        bb_phy_status_t *tx = &st_out.user_status[u].tx_status;
        bb_phy_status_t *rx = &st_out.user_status[u].rx_status;
        if (!tx->freq_khz && !rx->freq_khz)
            continue; /* unpopulated user entry */
        /* Observed on real hardware: every user entry has SOME freq_khz
         * populated (not a reliable "is this real" filter, hence still
         * printed below) -- this daemon build's BB_GET_STATUS just
         * doesn't fully zero unused user slots, matching ar8030-status.c's
         * own documented caveat about this same command's top-level
         * role/mode/mac fields. The genuinely active user is the one
         * whose tx.mcs/tx.bandwidth actually track real link changes
         * (compare against BB_GET_MCS below); the rest repeat one fixed
         * "default" reading (mcs=1, bw=2.5M) that never moves. */
        printf("user %d: tx{mcs=%u bandwidth=%s freq=%u kHz} rx{mcs=%u bandwidth=%s freq=%u kHz}\n", u, tx->mcs,
               bandwidth_name(tx->bandwidth), tx->freq_khz, rx->mcs, bandwidth_name(rx->bandwidth), rx->freq_khz);
    }

    bb_get_mcs_in_t mcs_in = { .dir = BB_DIR_TX, .slot = (uint8_t)slot };
    bb_get_mcs_out_t mcs_out;
    memset(&mcs_out, 0, sizeof(mcs_out));
    if (bb_ioctl(g_hbb, BB_GET_MCS, &mcs_in, &mcs_out) == 0)
        printf("BB_GET_MCS(dir=tx,slot=%d): mcs=%u throughput=%u kbps\n", slot, mcs_out.mcs, mcs_out.throughput);

    return 0;
}

static int cmd_bandwidth(int argc, char **argv)
{
    int slot = 0, dir = BB_DIR_TX, wait_s = 0;
    int slot_auto = 0;
    int opt;
    optind = 1;
    permute_argv(argc, argv, "d:s:w:");
    while ((opt = getopt(argc, argv, "d:s:w:")) != -1) {
        switch (opt) {
        case 'd':
            dir = parse_dir(optarg, 0);
            break;
        case 's':
            if (!strcmp(optarg, "auto")) {
                slot_auto = 1;
            } else {
                slot = atoi(optarg);
            }
            break;
        case 'w':
            wait_s = atoi(optarg);
            break;
        default:
            return 1;
        }
    }
    if (dir < 0) {
        fprintf(stderr, "linkctl: -d must be tx or rx\n");
        return 1;
    }
    if (optind >= argc) {
        fprintf(stderr, "linkctl: bandwidth needs a value (e.g. 20)\n");
        return 1;
    }
    int bw = parse_bandwidth(argv[optind]);
    if (bw < 0) {
        fprintf(stderr, "linkctl: bad bandwidth '%s' (want 1.25/2.5/5/10/20/40 or 0-5)\n", argv[optind]);
        return 1;
    }
    if (slot_auto) {
        /* Which slot a peer connects on isn't fixed across reboots
         * (confirmed on real hardware), so a hardcoded slot -- as this
         * used to be -- silently applies to the wrong slot and reports
         * back a benign-looking error instead of ever taking effect. */
        slot = resolve_connected_slot(wait_s);
        if (slot < 0) {
            fprintf(stderr, "linkctl: -s auto: no slot reached CONNECT within %ds\n", wait_s);
            return 1;
        }
    } else if (wait_s > 0 && wait_for_connect((uint8_t)slot, wait_s) != 0) {
        fprintf(stderr, "linkctl: link not CONNECT after %ds, applying anyway\n", wait_s);
    }

    bb_set_bandwidth_t sb = { .slot = (uint8_t)slot, .dir = (uint8_t)dir, .bandwidth = (uint8_t)bw };
    int ret = bb_ioctl(g_hbb, BB_SET_BANDWIDTH, &sb, NULL);
    printf("BB_SET_BANDWIDTH(slot=%d, dir=%s, bandwidth=%s) ret=%d\n", slot, dir == BB_DIR_TX ? "tx" : "rx",
           bandwidth_name((uint8_t)bw), ret);
    return ret ? 1 : 0;
}

static int cmd_channel_mode(int argc, char **argv)
{
    if (argc < 2 || (strcmp(argv[1], "auto") && strcmp(argv[1], "manual"))) {
        fprintf(stderr, "linkctl: channel-mode needs 'auto' or 'manual'\n");
        return 1;
    }
    bb_set_chan_mode_t m = { .auto_mode = (uint8_t)(!strcmp(argv[1], "auto")) };
    int ret = bb_ioctl(g_hbb, BB_SET_CHAN_MODE, &m, NULL);
    printf("BB_SET_CHAN_MODE(auto_mode=%u) ret=%d\n", m.auto_mode, ret);
    return ret ? 1 : 0;
}

static int cmd_channel(int argc, char **argv)
{
    int wait_s = 0, slot = 0;
    int slot_auto = 0;
    int opt;
    optind = 1;
    permute_argv(argc, argv, "w:s:");
    while ((opt = getopt(argc, argv, "w:s:")) != -1) {
        switch (opt) {
        case 'w':
            wait_s = atoi(optarg);
            break;
        case 's':
            if (!strcmp(optarg, "auto"))
                slot_auto = 1;
            else
                slot = atoi(optarg);
            break;
        default:
            return 1;
        }
    }
    if (optind >= argc) {
        fprintf(stderr, "linkctl: channel needs an index\n");
        return 1;
    }
    int index = atoi(argv[optind]);

    if (slot_auto) {
        slot = resolve_connected_slot(wait_s);
        if (slot < 0) {
            fprintf(stderr, "linkctl: -s auto: no slot reached CONNECT within %ds\n", wait_s);
            return 1;
        }
    } else if (wait_s > 0 && wait_for_connect((uint8_t)slot, wait_s) != 0) {
        fprintf(stderr, "linkctl: link not CONNECT after %ds, applying anyway\n", wait_s);
    }

    /* A real channel change is 3 ioctls, not 1 -- BB_SET_CHAN only ever
     * retunes this radio's own receiver, with no effect on the peer.
     * Matches the vendor's own reference sequence (bb_test.c): force
     * manual channel mode, set this side's RX channel, then push the
     * same target channel to the connected peer over the still-live
     * link via BB_SET_REMOTE so it retunes too. Direction is hardcoded
     * to RX: bb_set_chan_t's own doc comment says manual TX-direction
     * control is director-mode only (this project never runs director
     * mode -- see bb_mode_name()/ar8030.json), and RX is what the
     * vendor's reference tool always uses -- "when RX side channel
     * changes, it changes synchronously for all SLOTs" per that same
     * comment, which is also why BB_SET_CHAN itself takes no slot
     * parameter (unlike BB_SET_REMOTE below, which does: it has to name
     * which connected peer to notify).
     *
     * Confirmed on real hardware (ar8030d's own log): this drives a
     * synchronized "safe hop" -- bb_phy_chan_hop_trigger, then a
     * bb_link_sync_tx_proc/bb_link_sync_rx_proc handshake between the
     * two radios, then "safe hop ok!" -- that retunes both ends together
     * without ever dropping CONNECT state or touching MCS/throughput. If
     * the BB_SET_REMOTE push below fails after BB_SET_CHAN already
     * applied locally, there's no such handshake and the link is left
     * genuinely desynced with no automatic recovery. */
    bb_set_chan_mode_t cm = { .auto_mode = 0 };
    int ret = bb_ioctl(g_hbb, BB_SET_CHAN_MODE, &cm, NULL);
    printf("BB_SET_CHAN_MODE(auto_mode=0) ret=%d\n", ret);
    if (ret) {
        fprintf(stderr, "linkctl: failed to force manual channel mode, aborting before touching the link\n");
        return 1;
    }

    bb_set_chan_t sc = { .chan_dir = BB_DIR_RX, .chan_index = (uint8_t)index };
    ret = bb_ioctl(g_hbb, BB_SET_CHAN, &sc, NULL);
    printf("BB_SET_CHAN(dir=rx, index=%d) ret=%d\n", index, ret);
    if (ret) {
        fprintf(stderr, "linkctl: BB_SET_CHAN failed, not pushing to peer\n");
        return 1;
    }

    bb_set_remote_t sr = { .slot = (uint8_t)slot,
                            .type_bmp = (1u << BB_REMOTE_TYPE_CHAN_MODE) | (1u << BB_REMOTE_TYPE_TARGET_CHAN) };
    sr.setting.auto_chan = 0;
    sr.setting.target_chan = (uint8_t)index;
    ret = bb_ioctl(g_hbb, BB_SET_REMOTE, &sr, NULL);
    printf("BB_SET_REMOTE(slot=%d, auto_chan=0, target_chan=%d) ret=%d\n", slot, index, ret);
    if (ret) {
        fprintf(stderr,
                "linkctl: warning: local channel changed but pushing it to the peer failed -- link is likely "
                "desynced now; the peer needs its own manual channel change to match\n");
        return 1;
    }
    return 0;
}

static int cmd_mcs_mode(int argc, char **argv)
{
    int slot = 0, opt;
    optind = 1;
    permute_argv(argc, argv, "s:");
    while ((opt = getopt(argc, argv, "s:")) != -1) {
        if (opt == 's')
            slot = atoi(optarg);
        else
            return 1;
    }
    if (optind >= argc || (strcmp(argv[optind], "auto") && strcmp(argv[optind], "manual"))) {
        fprintf(stderr, "linkctl: mcs-mode needs 'auto' or 'manual'\n");
        return 1;
    }
    int auto_mode = !strcmp(argv[optind], "auto");

    bb_set_mcs_mode_t m = { .slot = (uint8_t)slot, .auto_mode = (uint8_t)auto_mode };
    int ret = bb_ioctl(g_hbb, BB_SET_MCS_MODE, &m, NULL);
    printf("BB_SET_MCS_MODE(slot=%d, auto_mode=%u) ret=%d\n", slot, m.auto_mode, ret);
    return ret ? 1 : 0;
}

static int cmd_mcs(int argc, char **argv)
{
    int slot = 0, wait_s = 0, opt;
    optind = 1;
    permute_argv(argc, argv, "s:w:");
    while ((opt = getopt(argc, argv, "s:w:")) != -1) {
        switch (opt) {
        case 's':
            slot = atoi(optarg);
            break;
        case 'w':
            wait_s = atoi(optarg);
            break;
        default:
            return 1;
        }
    }
    if (optind >= argc) {
        fprintf(stderr, "linkctl: mcs needs a value\n");
        return 1;
    }
    int mcs = atoi(argv[optind]);
    if (wait_s > 0 && wait_for_connect((uint8_t)slot, wait_s) != 0) {
        fprintf(stderr, "linkctl: link not CONNECT after %ds, applying anyway\n", wait_s);
    }

    bb_set_mcs_t sm = { .slot = (uint8_t)slot, .mcs = (uint8_t)mcs };
    int ret = bb_ioctl(g_hbb, BB_SET_MCS, &sm, NULL);
    printf("BB_SET_MCS(slot=%d, mcs=%d) ret=%d\n", slot, mcs, ret);
    return ret ? 1 : 0;
}

static int cmd_force_close_socket(int argc, char **argv)
{
    if (argc < 3) {
        fprintf(stderr, "linkctl: force-close-socket needs <slot> <port>\n");
        return 1;
    }
    bb_force_close_socket_t fc = { .slot = (uint8_t)atoi(argv[1]), .port = (uint8_t)atoi(argv[2]) };
    int ret = bb_ioctl(g_hbb, BB_FORCE_CLS_SOCKET, &fc, NULL);
    printf("BB_FORCE_CLS_SOCKET(slot=%u, port=%u) ret=%d\n", fc.slot, fc.port, ret);
    return ret ? 1 : 0;
}

static int cmd_force_close_all(void)
{
    int ret = bb_ioctl(g_hbb, BB_FORCE_CLS_SOCKET_ALL, NULL, NULL);
    printf("BB_FORCE_CLS_SOCKET_ALL ret=%d\n", ret);
    return ret ? 1 : 0;
}

static int cmd_freq(int argc, char **argv)
{
    int user = 0, wait_s = 0, slot = 0, opt;
    const char *dir_arg = "tx";
    optind = 1;
    permute_argv(argc, argv, "u:d:w:s:");
    while ((opt = getopt(argc, argv, "u:d:w:s:")) != -1) {
        switch (opt) {
        case 'u':
            user = atoi(optarg);
            break;
        case 'd':
            dir_arg = optarg;
            break;
        case 'w':
            wait_s = atoi(optarg);
            break;
        case 's':
            slot = atoi(optarg);
            break;
        default:
            return 1;
        }
    }
    int dir = parse_dir(dir_arg, 1);
    if (dir == -1) {
        fprintf(stderr, "linkctl: -d must be tx, rx, or both\n");
        return 1;
    }
    uint8_t dir_bmp = dir == -2 ? (uint8_t)((1u << BB_DIR_TX) | (1u << BB_DIR_RX)) : (uint8_t)(1u << dir);

    if (optind >= argc) {
        fprintf(stderr, "linkctl: freq needs a kHz value\n");
        return 1;
    }
    uint32_t khz = (uint32_t)strtoul(argv[optind], NULL, 10);

    if (wait_s > 0 && wait_for_connect((uint8_t)slot, wait_s) != 0) {
        fprintf(stderr, "linkctl: link not CONNECT after %ds, applying anyway\n", wait_s);
    }

    bb_set_freq_t sf = { .user = (uint8_t)user, .dir_bmp = dir_bmp, .freq_khz = khz };
    int ret = bb_ioctl(g_hbb, BB_SET_FREQ, &sf, NULL);
    printf("BB_SET_FREQ(user=%d, dir_bmp=0x%02x, freq_khz=%u) ret=%d\n", user, dir_bmp, khz, ret);
    return ret ? 1 : 0;
}

int main(int argc, char **argv)
{
    if (argc < 2 || !strcmp(argv[1], "-h") || !strcmp(argv[1], "--help")) {
        usage(argv[0]);
        return argc < 2 ? 1 : 0;
    }

    if (connect_daemon()) {
        disconnect_daemon();
        return 1;
    }

    const char *cmd = argv[1];
    int rc;
    if (!strcmp(cmd, "status"))
        rc = cmd_status(argc - 1, argv + 1);
    else if (!strcmp(cmd, "bandwidth"))
        rc = cmd_bandwidth(argc - 1, argv + 1);
    else if (!strcmp(cmd, "channel-mode"))
        rc = cmd_channel_mode(argc - 1, argv + 1);
    else if (!strcmp(cmd, "channel"))
        rc = cmd_channel(argc - 1, argv + 1);
    else if (!strcmp(cmd, "mcs-mode"))
        rc = cmd_mcs_mode(argc - 1, argv + 1);
    else if (!strcmp(cmd, "mcs"))
        rc = cmd_mcs(argc - 1, argv + 1);
    else if (!strcmp(cmd, "freq"))
        rc = cmd_freq(argc - 1, argv + 1);
    else if (!strcmp(cmd, "force-close-socket"))
        rc = cmd_force_close_socket(argc - 1, argv + 1);
    else if (!strcmp(cmd, "force-close-all"))
        rc = cmd_force_close_all();
    else {
        fprintf(stderr, "linkctl: unknown command '%s'\n", cmd);
        usage(argv[0]);
        rc = 1;
    }

    disconnect_daemon();
    return rc;
}
