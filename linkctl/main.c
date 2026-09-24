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
#include "../common/ar8030_batt.h"
#include "../common/ar8030_rftemp.h"

#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

static void usage(const char *argv0)
{
    fprintf(stderr,
            "usage: %s <command> [args]\n"
            "\n"
            "  status [-s slot]\n"
            "      Dump BB_GET_STATUS (per-user tx/rx mcs+bandwidth+freq, link\n"
            "      state), BB_GET_MCS, BB_GET_1V1_INFO, BB_GET_USER_QUALITY/\n"
            "      BB_GET_PEER_QUALITY (per-user/peer snr + LDPC block-error\n"
            "      ratio + antenna gain -- the closest available proxy to how\n"
            "      much repair work the baseband's FEC/retx layer is doing),\n"
            "      and BB_GET_SOCK_INFO (per-port tx/rx byte counters and\n"
            "      overflow counts, only for ports actually in use) for the\n"
            "      given slot (default 0).\n"
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
            "  mcs-range <min> <max> [-s slot]\n"
            "      BB_SET_MCS_RANGE -- constrains which MCS levels are usable.\n"
            "      Pass the literal word 'max' for either value to mean\n"
            "      BB_PHY_MCS_MAX (\"no limit\"), per that field's own doc comment.\n"
            "\n"
            "  mcs-table-air\n"
            "      Push stock AIR's own 3-entry table (mcs 1,2,5) exactly as\n"
            "      ar_ldyhs_sky's fpv_ap_reload_mcs_tab() does (note byte 10 = 2).\n"
            "\n"
            "  rate [-s slot] [seconds]\n"
            "      Measured (not theoretical) throughput: samples BB_GET_SOCK_INFO's\n"
            "      cumulative total_size counters for every open port twice, <seconds>\n"
            "      apart (default 5), and prints Mbit/s per port and direction plus the\n"
            "      socket buffer fill. Note: the daemon may only track these counters\n"
            "      for some socket types -- all-zero means this port isn't counted, not\n"
            "      that it is idle. Compare BB_GET_MCS's throughput (theoretical).\n"
            "\n"
            "  frame-change <0|1>\n"
            "      BB_SET_FRAME_CHANGE (1V1 only): 1 = exchange frame structure,\n"
            "      0 = restore the original. Compare BB_GET_MCS throughput after.\n"
            "\n"
            "  cfg-dump [-m mode] [file]\n"
            "      Read the baseband configuration file via BB_GET_CFG (mode\n"
            "      0 auto, 1 memory, 2 flash). Prints total length + crc16 and,\n"
            "      with [file], writes the raw bytes there (else hex dump) --\n"
            "      for diffing stock vs ours.\n"
            "\n"
            "  distance [-n count] [-i ms]\n"
            "      Reads BB_GET_DISTC_RESULT for every slot, count times (default\n"
            "      10) every ms milliseconds (default 500). Metres (-1 = no\n"
            "      ranging result): the chip subtracts ar8030.json's\n"
            "      dist_calc.offset and clamps at 0, and stock displays the value\n"
            "      unconverted as \"<n>m\".\n"
            "\n"
            "  rf-temp [-c channel] [-a]\n"
            "      RF-board temperature from the AR8030's own ADC (default channel 4,\n"
            "      the Caddx Ascent's thermistor): BB_GET_PRJ_DISPATCH 0x89, mapped\n"
            "      through stock ar_ldyhs_sky's thermistor table. Arms the channel\n"
            "      (0x8a, as stock's init does) first if it isn't yet, or always\n"
            "      with -a. Unsmoothed -- ar8030-lifecycled --rf-temp-adc keeps a\n"
            "      smoothed value (HTTP /api/v1/rf-temp).\n"
            "\n"
            "  batt [-c channel] [-k scale] [-o offset_mv] [-n count] [-a]\n"
            "      Supply voltage from the AR8030's own ADC (default channel 0,\n"
            "      scale 16, offset 1200 mV -- measured on the Ascent Lite, see\n"
            "      common/ar8030_batt.h): supply = adc * scale + offset. Averages\n"
            "      count reads (default 10, 250 ms apart) and prints the raw ADC\n"
            "      value too, for recalibrating. Arms the ADC first if it reads\n"
            "      0 mV, or always with -a. 0 mV = nothing on the power input.\n"
            "\n"
            "  prj-cmd <cmd> [byte ...]\n"
            "      Raw BB_SET_PRJ_DISPATCH: cmd id (dec/0x hex) + up to 252\n"
            "      payload bytes. Stock init uses 0x8c <0|1> (rfo_kikp), 0x8a\n"
            "      <ch> <u32> (adc_meas), 0xcb 0 0 0x01 0x03 ... (rf_path_b).\n"
            "\n"
            "  mcs-table <0|1|2> [mcs]\n"
            "      Push a whole MCS policy table (7 entries covering mcs 1,2,5,\n"
            "      7,8,10,12) via BB_SET_MCS_ITEM, all on slot 0. Reverse-\n"
            "      engineered from stock's ar_ldy_gnd binary (Ghidra): its own\n"
            "      \"reload mcs tab %%d\" routine is driven by fpv_config.json's\n"
            "      \"video_strategy\" setting, and 0/1 select one table while 2\n"
            "      selects a second, distinct one (different SNR thresholds and\n"
            "      LDPC error-count gates per entry -- see linkctl/main.c's own\n"
            "      mcs_tables[] for the exact transcribed values). BB_SET_MCS_ITEM\n"
            "      does not exist in this SDK build's own header/dispatch table at\n"
            "      all -- added here (bb_api.h/ioctl_tab.c) from the same decompile,\n"
            "      cross-checked against bb_mcs_para_t's own field set. No -w/-s:\n"
            "      always slot 0, matching every observed call site.\n"
            "\n"
            "      With [mcs], pushes only that one entry instead of all 7 (to\n"
            "      retry/isolate a single failure from a full run).\n"
            "\n"
            "      Confirmed live: all 7 entries succeed on ground every time.\n"
            "      On air, only mcs 1,2,5,8 succeed -- 7,10,12 consistently fail\n"
            "      with an undocumented ret=6 (isolated single-entry pushes fail\n"
            "      identically, so it is not a batch-ordering issue, and widening\n"
            "      BB_SET_MCS_RANGE first has no effect either). This is a real\n"
            "      board/firmware-side limit, not a bug in this reconstruction --\n"
            "      decompiling air's own production app (ar_ldyhs_sky) shows its\n"
            "      own fpv_ap_reload_mcs_tab() only ever writes 3 entries (mcs\n"
            "      1,2,5), never touching 7/8/10/12 either. Since stock's own\n"
            "      code never overrides those higher entries on air, whatever\n"
            "      gives stock its own higher throughput at e.g. mcs=12 is not\n"
            "      coming from this mechanism -- likely a factory-calibrated\n"
            "      default baked into air's chip/firmware that neither stock nor\n"
            "      this tool ever writes.\n"
            "\n"
            "  retx [--set <win> <busy> <idle> <conti_busy> <conti_idle>]\n"
            "      Dump (and optionally tune) the windowed retransmission\n"
            "      controller's own event/status record via\n"
            "      BB_GET_RETX_EVENT_STATUS/BB_SET_RETX_EVENT_STATUS -- added\n"
            "      to this SDK build from an external firmware analysis,\n"
            "      independently confirmed via Ghidra (see linkctl/main.c's own\n"
            "      cmd_retx comment). Only the first 5 of 136 bytes are decoded;\n"
            "      the rest print as raw hex (confirmed leaked ar8030d-internal\n"
            "      memory, not real data). --set is confirmed to take effect live\n"
            "      after boot (unlike the vendor's own app, which only ever sets\n"
            "      this once, at its own init) -- still always read back.\n"
            "\n"
            "  retx-watch\n"
            "      Live-monitor BB_EVENT_RETX_TOO_MANY -- the actual event the\n"
            "      vendor streamer subscribes to and polls before cutting\n"
            "      bitrate (independently recovered via Ghidra decompile of\n"
            "      fpv_bb_is_send_retx_too_many(), NOT the same thing as the\n"
            "      `retx` command above; see BB_EVENT_RETX_TOO_MANY's own doc\n"
            "      comment in bb_api.h). Prints one line per firing plus a 10s\n"
            "      heartbeat, until Ctrl-C. Unlike every other command here,\n"
            "      this one does not exit after one bb_ioctl -- it's meant for\n"
            "      watching this fire live while forcing real link stress (e.g.\n"
            "      moving the peer out of range).\n"
            "\n"
            "  power-mode [auto|manual]\n"
            "      With no argument, reads back the current mode (BB_GET_POWER_MODE).\n"
            "      With an argument, sets it: transmit power open/closed loop\n"
            "      mode -- 'manual' (open loop) is required before 'power' below\n"
            "      has any effect; 'auto' (closed loop) has the chip manage its\n"
            "      own transmit power and ignore manual writes. Chip-wide, no\n"
            "      per-user/slot parameter (matching BB_SET_POWER_MODE's own\n"
            "      struct, which carries none).\n"
            "\n"
            "  power <dbm> [-u user] [-w seconds]\n"
            "      Manually set transmit power for one physical user, in dBm,\n"
            "      range [0-31] per BB_SET_POWER's own doc comment. Also prints\n"
            "      the equivalent in mW (mW = 10^(dBm/10)).\n"
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

/* Naming here matches cmd_power_mode's own auto/manual mapping, not
 * bb_phy_pwr_mode_e's literal OPENLOOP/CLOSELOOP names -- see that
 * function's own comment on why the two are inverted. */
static const char *pwr_mode_name(uint8_t mode)
{
    switch (mode) {
    case BB_PHY_PWR_OPENLOOP:
        return "manual (open loop)";
    case BB_PHY_PWR_CLOSELOOP:
        return "auto (closed loop)";
    default:
        return "unknown";
    }
}

static double dbm_to_mw(uint8_t dbm)
{
    return pow(10.0, (double)dbm / 10.0);
}

/* bb_quality_t's own doc comment in bb_api.h: "conversion to db formula:
 * 10log(snr/36)" -- same raw snr units BB_GET_1V1_INFO's self/peer.snr
 * already use unconverted above; this converts for BB_GET_USER_QUALITY/
 * BB_GET_PEER_QUALITY's own printing below. */
static double snr_to_db(uint16_t snr)
{
    return 10.0 * log10((double)snr / 36.0);
}

/* cfg_sbmp/rt_sbmp are bitmasks over bb_slot_e (bit N = slot N). Printed as
 * a plain slot list instead of raw hex so "which slots actually exist"
 * doesn't require the reader to decode a bitmap by hand. */
/* Generic "which bits are set" printer -- shared by cfg_sbmp/rt_sbmp
 * (slots, count = BB_SLOT_MAX) below and BB_GET_SOCK_INFO's port_bmp
 * (ports, count = BB_SOCK_INFO_NUM) further down. Both happen to be 8 on
 * this SDK, but that's coincidence, not a reason to hardcode one bound
 * for both meanings -- count is a parameter, not BB_SLOT_MAX baked in. */
static void print_bit_list(const char *label, uint8_t bmp, int count)
{
    printf("%s:", label);
    int any = 0;
    for (int i = 0; i < count; i++) {
        if (bmp & (1u << i)) {
            printf(" %d", i);
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

    /* BB_GET_CHAN_INFO used to be skipped here entirely: its output struct
     * (bb_get_chan_info_out_t) has fixed-size freq[]/power[] arrays sized
     * BB_CONFIG_MAX_CHAN_NUM, and this project's own ar8030.json configures
     * 42 channels -- with the old cap of 32 that overflowed the reply into
     * whatever was next on the stack and segfaulted, deterministically,
     * every call. Fixed two ways together, neither of them a client-side
     * workaround: session_ioctl.c's io_rpc_cb() now clamps any oversized RPC
     * reply to the caller's actual buffer size instead of trusting the wire
     * (patch 0020/0027), and BB_CONFIG_MAX_CHAN_NUM itself was raised from
     * 32 to 60 (patch 0022/0029) so a 42-channel reply now fits without that
     * clamp ever needing to fire for this call. Confirmed live before this
     * patch that the chip itself was never limited to 32 -- channel indices
     * above it (33, 40) already retuned successfully via
     * BB_SET_CHAN/BB_SET_REMOTE -- only this host-side struct was.
     *
     * Only a compact one-line summary here (chan_num/auto_mode/work_chan +
     * the working channel's own frequency looked up from freq[]), not the
     * full freq[]/power[] table: this runs on every `status` call, which the
     * HTTP control API's /api/v1/status polls every 2s (lifecycle_http.c),
     * so keep the added RPC payload/parsing small rather than dumping up to
     * 60 frequency/power pairs on every poll. work_chan is bounds-checked
     * against both chan_num and the array size before indexing freq[] --
     * chan_num is chip-reported and not otherwise validated here. */
    bb_get_chan_info_out_t chan_out;
    memset(&chan_out, 0, sizeof(chan_out));
    if (bb_ioctl(g_hbb, BB_GET_CHAN_INFO, NULL, &chan_out) == 0) {
        uint32_t cur_freq = (chan_out.work_chan < chan_out.chan_num && chan_out.work_chan < BB_CONFIG_MAX_CHAN_NUM)
                                 ? chan_out.freq[chan_out.work_chan]
                                 : 0;
        printf("BB_GET_CHAN_INFO: chan_num=%u auto_mode=%s work_chan=%u acs_chan=%u freq=%u kHz\n", chan_out.chan_num,
               chan_out.auto_mode ? "auto" : "manual", chan_out.work_chan, chan_out.acs_chan, cur_freq);
    }

    print_bit_list("configured slots", st_out.cfg_sbmp, BB_SLOT_MAX);
    print_bit_list("active slots", st_out.rt_sbmp, BB_SLOT_MAX);
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

    /* rf_1tx (1=single-antenna TX, 0=dual/MIMO TX) directly affects PHY
     * throughput at a given MCS index -- a link that fell back to
     * single-TX reports a much lower throughput than one running dual-TX
     * at the identical reported mcs+bandwidth, which is otherwise
     * invisible in the tx/rx mcs+bandwidth+freq dump above. Also useful
     * for spotting a real SNR/gain asymmetry between this side and the
     * peer, not just this side's own view. Previously crashed this tool
     * outright on air (SIGILL) -- root-caused to session_ioctl.c's
     * io_rpc_cb() copying the daemon's reply datalen into this call's
     * fixed-size stack buffer with no bounds check; fixed there (clamped
     * to get_bb_ioctl_cmdoutlen()) rather than worked around here. */
    bb_get_1v1_info_in_t info_in = { .frame_num = 0 };
    bb_get_1v1_info_out_t info_out;
    memset(&info_out, 0, sizeof(info_out));
    if (bb_ioctl(g_hbb, BB_GET_1V1_INFO, &info_in, &info_out) == 0) {
        printf("BB_GET_1V1_INFO: self{snr=%u gain=[%u,%u] tx_mcs=%u tx_chan=%u tx_power=%u tx=%s} "
               "peer{snr=%u gain=[%u,%u] tx_mcs=%u tx_chan=%u tx_power=%u tx=%s}\n",
               info_out.self.snr, info_out.self.gain_a, info_out.self.gain_b, info_out.self.tx_mcs,
               info_out.self.tx_chan, info_out.self.tx_power, info_out.self.rf_1tx ? "single" : "dual",
               info_out.peer.snr, info_out.peer.gain_a, info_out.peer.gain_b, info_out.peer.tx_mcs,
               info_out.peer.tx_chan, info_out.peer.tx_power, info_out.peer.rf_1tx ? "single" : "dual");
    }

    /* LDPC block-error ratio -- a real error-rate proxy for how much
     * repair work the baseband's own FEC/retx layer is doing that neither
     * BB_GET_STATUS nor BB_GET_1V1_INFO above carries (both give
     * snr/gain/mcs but not ldpc_err/ldpc_num). BB_GET_USER_QUALITY is this
     * side's own per-physical-user reading; BB_GET_PEER_QUALITY is the
     * connected peer's, keyed by slot instead of user index -- both are
     * documented as bb_quality_t{snr,ldpc_err,ldpc_num,gain_a,gain_b}
     * (8 bytes) arrays.
     *
     * CONFIRMED ON REAL HARDWARE (both air and ground, 2026-09):
     * both replies come back roughly 2x the size sizeof() the SDK header's
     * own struct predicts (BB_GET_USER_QUALITY: 160 bytes on the wire vs.
     * 80 expected for BB_DATA_USER_MAX=10 entries; BB_GET_PEER_QUALITY:
     * 128 vs. 64 for BB_SLOT_MAX=8) -- logged by session_ioctl.c's own
     * io_rpc_cb() truncation guard ("reply datalen=... exceeds expected
     * ..., truncating"), the same class of header/wire mismatch already
     * hit and fixed for BB_GET_CHAN_INFO. Unlike that single-struct case,
     * an array's element STRIDE being wrong corrupts every index beyond
     * the first, not just trailing bytes: only qualities[0] is guaranteed
     * to start at the wire's own byte 0 regardless of the real per-entry
     * size, so it's the only index safe to trust until the actual stride
     * is confirmed (a small follow-up RE task, same class as the
     * BB_GET_RETX_EVENT_STATUS work but much smaller). Cross-validated
     * live: the ground unit's own peer-slot-0 reading (its view of the
     * air AP) matched the air unit's own user-0 reading (snr~10.2-10.3dB,
     * gain=[20,103]) almost exactly -- two independent daemons agreeing on
     * the same physical link is strong evidence index 0 itself is
     * correctly aligned. bitrate_ctl.c's read_ldpc_ratio() already only
     * reads index 0 for exactly this reason. Do not loop over further
     * indices here until the real stride is confirmed. */
    bb_get_user_quality_in_t uq_in = { .user_bmp = 0xffff, .average = 0 };
    bb_get_user_quality_out_t uq_out;
    memset(&uq_out, 0, sizeof(uq_out));
    if (bb_ioctl(g_hbb, BB_GET_USER_QUALITY, &uq_in, &uq_out) == 0) {
        bb_quality_t *q = &uq_out.qualities[0];
        if (q->snr || q->ldpc_num || q->gain_a || q->gain_b) {
            double ratio = q->ldpc_num ? (double)q->ldpc_err / (double)q->ldpc_num : 0.0;
            printf("user 0 quality: snr=%.1fdB ldpc=%u/%u (%.1f%%) gain=[%u,%u]\n", snr_to_db(q->snr),
                   q->ldpc_err, q->ldpc_num, ratio * 100.0, q->gain_a, q->gain_b);
        }
    }

    bb_get_peer_quality_in_t pq_in = { .slot_bmp = st_out.cfg_sbmp, .arverage = 0 };
    bb_get_peer_quality_out_t pq_out;
    memset(&pq_out, 0, sizeof(pq_out));
    if (bb_ioctl(g_hbb, BB_GET_PEER_QUALITY, &pq_in, &pq_out) == 0) {
        bb_quality_t *q = &pq_out.qualities[0];
        if ((st_out.cfg_sbmp & 1u) && (q->snr || q->ldpc_num || q->gain_a || q->gain_b)) {
            double ratio = q->ldpc_num ? (double)q->ldpc_err / (double)q->ldpc_num : 0.0;
            printf("peer slot 0 quality: snr=%.1fdB ldpc=%u/%u (%.1f%%) gain=[%u,%u]\n", snr_to_db(q->snr),
                   q->ldpc_err, q->ldpc_num, ratio * 100.0, q->gain_a, q->gain_b);
        }
    }

    /* Ranging result for every configured slot (same cfg_sbmp filter as the
     * link_status loop above). Configured only by ar8030.json's dist_calc
     * at chip load -- BB_CFG_DISTC at runtime is rejected (ret=-2), and the
     * stock streamers never send it either. The value is metres: the chip
     * subtracts dist_calc.offset (the vendor's calibration) and clamps at 0,
     * and stock shows it unconverted -- ar_ldy_gnd forwards distance[0] to
     * GlassesUI, which renders QString("%1m").arg(distance). A bench link
     * reads 0 m. `ar8030-linkctl distance` samples it repeatedly. */
    bb_get_distc_result_in_t dist_in = { .slot_bmp = st_out.cfg_sbmp };
    bb_get_distc_result_out_t dist_out;
    memset(&dist_out, 0, sizeof(dist_out));
    if (bb_ioctl(g_hbb, BB_GET_DISTC_RESULT, &dist_in, &dist_out) == 0) {
        for (int s = 0; s < BB_SLOT_MAX; s++) {
            if (!(st_out.cfg_sbmp & (1u << s)))
                continue;
            if (dist_out.distance[s] < 0)
                printf("BB_GET_DISTC_RESULT(slot=%d): no ranging result\n", s);
            else
                printf("BB_GET_DISTC_RESULT(slot=%d): distance=%d m\n", s, dist_out.distance[s]);
        }
    }

    bb_get_pwr_mode_out_t pwr_mode_out;
    memset(&pwr_mode_out, 0, sizeof(pwr_mode_out));
    if (bb_ioctl(g_hbb, BB_GET_POWER_MODE, NULL, &pwr_mode_out) == 0)
        printf("BB_GET_POWER_MODE: %s\n", pwr_mode_name(pwr_mode_out.pwr_mode));

    /* The user whose power this side actually transmits with, the same one
     * ar8030-lifecycled sets (lifecycle_tuning.c's lc_power_apply(), after
     * the stock streamers): BR/CS on the AP, user 0 on the DEV. Confirmed
     * on hardware that the AP's user 0 keeps reporting its config-file
     * value no matter what the AP really transmits. No mW conversion: this
     * is the chip's dBm scale, not antenna output (stock's own "500 mW" is
     * 26 here on the Lite air unit, 24 on the ground). */
    uint8_t pwr_usr = st_out.role == BB_ROLE_AP ? BB_USER_BR_CS : BB_USER_0;
    bb_get_cur_pwr_in_t pwr_in = { .usr = pwr_usr };
    bb_get_cur_pwr_out_t pwr_out;
    memset(&pwr_out, 0, sizeof(pwr_out));
    if (bb_ioctl(g_hbb, BB_GET_CUR_POWER, &pwr_in, &pwr_out) == 0)
        printf("BB_GET_CUR_POWER(usr=%u%s): pwr=%u (chip dBm)\n", pwr_usr, pwr_usr == BB_USER_BR_CS ? " BR/CS" : "",
               pwr_out.pwr);

    /* Per-port bb_socket usage for this same slot. Genuinely useful
     * beyond a nice-to-have: this session's own ar8030d-reconnect work
     * hit a real, live "port stuck reporting already opened" failure
     * more than once (see force-close-socket's own doc comment above and
     * README's "ar8030d connection: surviving a daemon restart") with no
     * way to actually SEE which port the daemon thought was still in
     * use -- this is that visibility.
     *
     * Deliberately keyed on port_bmp, not each uni_info's own
     * `available` flag (which sbc-groundstations' ar8030-status.c uses
     * instead) -- confirmed live on a real, actively-streaming socket
     * that the two disagree: port_bmp correctly reported this project's
     * own video port (2, stream-mode TX-only) as open, while that same
     * port's tx/rx `available` both read 0 with every byte counter
     * zeroed. Likely because the daemon's detailed per-port byte/
     * overflow counters are only tracked for the datagram-style ar_net0
     * ports (0/1) this ioctl was presumably designed around, not this
     * project's own stream-mode socket type. Whatever the reason,
     * `available` would have hidden exactly the port most worth seeing
     * for the stuck-port scenario this exists to diagnose -- port_bmp is
     * the bitmap BB_SET_CANDIDATES/force-close-all's own callers already
     * treat as authoritative elsewhere in this SDK (cfg_sbmp/rt_sbmp
     * above), so trust it here too. */
    bb_get_sock_info_in_t sock_in = { .slot = (uint8_t)slot, .port = -1 };
    bb_get_sock_info_out_t sock_out;
    memset(&sock_out, 0, sizeof(sock_out));
    if (bb_ioctl(g_hbb, BB_GET_SOCK_INFO, &sock_in, &sock_out) == 0) {
        print_bit_list("open ports", sock_out.port_bmp, BB_SOCK_INFO_NUM);
        for (int p = 0; p < BB_SOCK_INFO_NUM; p++) {
            if (!(sock_out.port_bmp & (1u << p)))
                continue;
            bb_sock_uni_t *tx_u = &sock_out.sock_info[p].uni_info[BB_DIR_TX];
            bb_sock_uni_t *rx_u = &sock_out.sock_info[p].uni_info[BB_DIR_RX];
            printf("port %d: tx_bytes=%llu (overflow=%u) rx_bytes=%llu (overflow=%u)\n", p,
                   (unsigned long long)tx_u->total_size, tx_u->overflow_cnt,
                   (unsigned long long)rx_u->total_size, rx_u->overflow_cnt);
        }
    }

    return 0;
}

/* BB_GET_RETX_EVENT_STATUS / BB_SET_RETX_EVENT_STATUS -- the windowed
 * retransmission controller's own tuning parameters (how aggressively
 * the baseband declares this link "in trouble" and re-sends), NOT the
 * same thing as BB_SET_RETX_COUNT (a separate, unrelated per-slot max-
 * retries cap this tool doesn't touch). Both opcodes were missing from
 * this SDK build entirely (like BB_SET_MCS_ITEM, see mcs-table's own
 * comment) and were added here from an external firmware analysis,
 * independently confirmed by this project via Ghidra decompile of the
 * vendor streamer binary -- see bb_retx_cfg_t's own doc comment in
 * bb_api.h for exactly what is and isn't verified. Only the first 5 of
 * the struct's 136 bytes are meaningful; the rest print as raw hex for
 * completeness only.
 *
 * CONFIRMED DEAD END on real hardware (2026-09, live-monitoring a real
 * link): those remaining 131 bytes are not a stable per-opcode record
 * at all -- they're leaked, reused ar8030d-internal buffer memory. One
 * captured sample decoded byte-for-byte as the daemon's own literal log
 * string, `[ 0][ERR] bb_ioctl: req 0x0100006B failed -1\r\n`, i.e. a
 * completely unrelated BB_GET_1V1_INFO failure's own error message,
 * left over in whatever buffer this opcode's reply happened to reuse.
 * Other samples showed pointer-looking values in the same byte range.
 * Never build a control loop (retx-pressure signal, etc.) on anything
 * past byte 4 of this struct -- there is nothing stable there to read. */
static void print_retx_cfg(const bb_retx_cfg_t *cfg)
{
    printf("win=%u busy=%u idle=%u conti_busy=%u conti_idle=%u\n", cfg->win, cfg->busy, cfg->idle,
           cfg->conti_busy, cfg->conti_idle);
    printf("reserved[131] (leaked ar8030d-internal memory, not real protocol data -- see this "
           "function's own comment):");
    for (size_t i = 0; i < sizeof(cfg->reserved); i++)
        printf(" %02x", cfg->reserved[i]);
    printf("\n");
}

static int cmd_retx(int argc, char **argv)
{
    bb_retx_cfg_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    if (bb_ioctl(g_hbb, BB_GET_RETX_EVENT_STATUS, NULL, &cfg)) {
        fprintf(stderr, "linkctl: BB_GET_RETX_EVENT_STATUS failed\n");
        return 1;
    }
    printf("current: ");
    print_retx_cfg(&cfg);

    if (argc < 2 || strcmp(argv[1], "--set") != 0)
        return 0;

    if (argc != 7) {
        fprintf(stderr, "linkctl: retx --set needs exactly 5 values: win busy idle conti_busy conti_idle\n");
        return 1;
    }
    fprintf(stderr,
            "linkctl: WARNING -- live SET after boot is UNVERIFIED on real hardware (the vendor\n"
            "app itself only ever calls this once, at its own init). Test from a clean boot,\n"
            "expect the link may need to be re-established, and read back below to see\n"
            "whether anything actually changed.\n");
    bb_retx_cfg_t set_cfg;
    memset(&set_cfg, 0, sizeof(set_cfg));
    set_cfg.win = (uint8_t)strtoul(argv[2], NULL, 10);
    set_cfg.busy = (uint8_t)strtoul(argv[3], NULL, 10);
    set_cfg.idle = (uint8_t)strtoul(argv[4], NULL, 10);
    set_cfg.conti_busy = (uint8_t)strtoul(argv[5], NULL, 10);
    set_cfg.conti_idle = (uint8_t)strtoul(argv[6], NULL, 10);

    int ret = bb_ioctl(g_hbb, BB_SET_RETX_EVENT_STATUS, &set_cfg, NULL);
    printf("BB_SET_RETX_EVENT_STATUS(");
    print_retx_cfg(&set_cfg);
    printf("): ret=%d\n", ret);
    if (ret)
        return 1;

    memset(&cfg, 0, sizeof(cfg));
    if (bb_ioctl(g_hbb, BB_GET_RETX_EVENT_STATUS, NULL, &cfg) == 0) {
        printf("read back: ");
        print_retx_cfg(&cfg);
    }
    return 0;
}

/* Bumped by on_retx_too_many_event() -- see that callback's own comment
 * for why `arg` is never dereferenced. Signal-safe-cheap per
 * bb_event_callback's own "synchronous locally" doc comment in
 * bb_api.h, same constraint tx/bitrate_ctl.c's own callbacks already
 * follow. */
static volatile sig_atomic_t g_retx_too_many;
static volatile sig_atomic_t g_stop_watch;

static void on_retx_too_many_event(void *arg, void *user)
{
    (void)arg;
    (void)user;
    g_retx_too_many++;
}

static void on_sigint(int sig)
{
    (void)sig;
    g_stop_watch = 1;
}

static uint64_t watch_now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000u + (uint64_t)ts.tv_nsec / 1000000u;
}

/* Deliberately NOT one-shot, unlike every other command in this file
 * (see this file's own top-of-file comment on that general design) --
 * a live event monitor is the natural way to actually verify
 * BB_EVENT_RETX_TOO_MANY fires on real link stress (e.g. moving the
 * peer out of range) without digging through ar8030d's own text log
 * (the daemon_log directory). Runs until Ctrl-C, printing one line per firing
 * plus a periodic heartbeat so it's clear the subscription itself is
 * still alive even during a long quiet stretch. See bb_api.h's own
 * BB_EVENT_RETX_TOO_MANY doc comment for what is and isn't confirmed
 * about this event (notably: `arg`'s payload layout is not, so this
 * only ever counts firings, never inspects one). */
static int cmd_retx_watch(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    bb_set_event_callback_t sub;
    memset(&sub, 0, sizeof(sub));
    sub.event = BB_EVENT_RETX_TOO_MANY;
    sub.callback = on_retx_too_many_event;
    sub.user = NULL;
    int ret = bb_ioctl(g_hbb, BB_SET_EVENT_SUBSCRIBE, &sub, NULL);
    if (ret) {
        fprintf(stderr, "linkctl: BB_SET_EVENT_SUBSCRIBE(RETX_TOO_MANY) failed (ret=%d)\n", ret);
        return 1;
    }
    printf("watching for BB_EVENT_RETX_TOO_MANY -- Ctrl-C to stop\n");

    signal(SIGINT, on_sigint);

    sig_atomic_t last_seen = 0;
    uint64_t last_heartbeat_ms = watch_now_ms();
    while (!g_stop_watch) {
        usleep(100 * 1000);
        sig_atomic_t now_count = g_retx_too_many;
        uint64_t now = watch_now_ms();
        if (now_count != last_seen) {
            printf("[%llu.%03llus] BB_EVENT_RETX_TOO_MANY fired (count=%d)\n", (unsigned long long)(now / 1000),
                   (unsigned long long)(now % 1000), (int)now_count);
            last_seen = now_count;
            last_heartbeat_ms = now;
        } else if (now - last_heartbeat_ms >= 10000) {
            printf("[%llu.%03llus] still watching (count=%d)\n", (unsigned long long)(now / 1000),
                   (unsigned long long)(now % 1000), (int)now_count);
            last_heartbeat_ms = now;
        }
    }
    printf("stopped -- total firings: %d\n", (int)g_retx_too_many);

    bb_set_event_callback_t unsub;
    memset(&unsub, 0, sizeof(unsub));
    unsub.event = BB_EVENT_RETX_TOO_MANY;
    bb_ioctl(g_hbb, BB_SET_EVENT_UNSUBSCRIBE, &unsub, NULL);
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

static int parse_mcs_or_max(const char *s)
{
    if (!strcmp(s, "max")) {
        return BB_PHY_MCS_MAX;
    }
    return atoi(s);
}

static int cmd_mcs_range(int argc, char **argv)
{
    int slot = 0, opt;
    optind = 1;
    permute_argv(argc, argv, "s:");
    while ((opt = getopt(argc, argv, "s:")) != -1) {
        if (opt == 's') {
            slot = atoi(optarg);
        } else {
            return 1;
        }
    }
    if (optind + 1 >= argc) {
        fprintf(stderr, "linkctl: mcs-range needs <min> <max>\n");
        return 1;
    }
    int mcs_min = parse_mcs_or_max(argv[optind]);
    int mcs_max = parse_mcs_or_max(argv[optind + 1]);

    bb_set_mcs_range_in_t mr = { .slot = (uint8_t)slot, .mcs_min = (uint8_t)mcs_min, .mcs_max = (uint8_t)mcs_max };
    int ret = bb_ioctl(g_hbb, BB_SET_MCS_RANGE, &mr, NULL);
    printf("BB_SET_MCS_RANGE(slot=%d, mcs_min=%d, mcs_max=%d) ret=%d\n", slot, mcs_min, mcs_max, ret);
    return ret ? 1 : 0;
}

/* Transcribed byte-for-byte from Ghidra's decompile of stock ar_ldy_gnd's
 * mcs-table-reload routine (FUN_001a1c68 in that binary) -- param_1==0
 * and ==1 both hit the same branch (table index 0 below); ==2 hits a
 * second, distinct branch (table index 1 below). Field order here
 * matches bb_set_mcs_item_t exactly (mcs, ldpc_up_num, snr_up, snr_dw,
 * ldpc_dw_num, up_keep_time, dw_keep_time) -- NOT bb_mcs_para_t's own
 * declared order, which this wire struct does not reuse. */
struct mcs_table_entry {
    uint8_t  mcs;
    uint8_t  ldpc_up_num;
    uint16_t snr_up;
    uint16_t snr_dw;
    uint8_t  ldpc_dw_num;
    uint16_t up_keep_time;
    uint16_t dw_keep_time;
};

static const struct mcs_table_entry mcs_tables[2][7] = {
    /* [0]: stock's "else" branch (video_strategy 0 or 1) */
    {
        { 1, 2, 0x24, 0x1d, 4, 1000, 15 },
        { 2, 2, 0x5c, 0x41, 4, 0x5dc, 15 },
        { 5, 2, 0xa8, 0x77, 4, 0x5dc, 15 },
        { 7, 2, 0x12f, 0xf1, 4, 800, 15 },
        { 8, 2, 0x256, 0x1db, 3, 800, 15 },
        { 10, 2, 0x4a8, 0x3b3, 4, 1000, 12 },
        { 12, 2, 0x736, 0x5ba, 2, 1000, 1 },
    },
    /* [1]: stock's video_strategy==2 branch */
    {
        { 1, 3, 0x24, 0x1d, 5, 1000, 0 },
        { 2, 4, 0x41, 0x34, 6, 0x5dc, 100 },
        { 5, 3, 0x72, 0x5a, 5, 0x5dc, 100 },
        { 7, 3, 300, 0xd6, 5, 700, 0x19 },
        { 8, 3, 0x214, 0x1a6, 5, 800, 15 },
        { 10, 3, 0x426, 0x34b, 5, 1000, 0 },
        { 12, 3, 0x736, 0x5bb, 5, 1000, 0 },
    },
};

static int cmd_mcs_table(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "linkctl: mcs-table needs a value (0, 1, or 2)\n");
        return 1;
    }
    int variant = atoi(argv[1]);
    if (variant < 0 || variant > 2) {
        fprintf(stderr, "linkctl: mcs-table must be 0, 1, or 2\n");
        return 1;
    }
    const struct mcs_table_entry *table = mcs_tables[variant == 2 ? 1 : 0];

    /* Optional 3rd arg: push only the single entry matching this mcs
     * value, instead of all 7 -- for retrying/isolating one that failed
     * in a full run. */
    int only_mcs = argc >= 3 ? atoi(argv[2]) : -1;

    int fail = 0;
    for (int i = 0; i < 7; i++) {
        if (only_mcs >= 0 && table[i].mcs != only_mcs) {
            continue;
        }
        bb_set_mcs_item_t item;
        memset(&item, 0, sizeof(item));
        item.mcs           = table[i].mcs;
        item.ldpc_up_num   = table[i].ldpc_up_num;
        item.snr_up        = table[i].snr_up;
        item.snr_dw        = table[i].snr_dw;
        item.ldpc_dw_num   = table[i].ldpc_dw_num;
        item.up_keep_time  = table[i].up_keep_time;
        item.dw_keep_time  = table[i].dw_keep_time;

        int ret = bb_ioctl(g_hbb, BB_SET_MCS_ITEM, &item, NULL);
        printf("BB_SET_MCS_ITEM(mcs=%u snr_up=%u snr_dw=%u ldpc_up=%u ldpc_dw=%u "
               "up_keep=%ums dw_keep=%ums) ret=%d\n",
               item.mcs, item.snr_up, item.snr_dw, item.ldpc_up_num, item.ldpc_dw_num,
               item.up_keep_time, item.dw_keep_time, ret);
        if (ret) {
            fail = 1;
        }
    }
    return fail;
}

/* Stock AIR's table, from ar_ldyhs_sky fpv_ap_reload_mcs_tab @ 0x000685c0
 * (3 entries only). Byte 10 of the wire struct (rsv2 here) is 2 in every
 * entry -- unlike the ground table -- so it is set explicitly. */
static int cmd_mcs_table_air(void)
{
    static const struct { uint8_t mcs, up, dwn; uint16_t snr_up, snr_dw, upk, dwk; } air[3] = {
        { 1, 2, 4, 0x42, 0x2f, 1000, 500 },
        { 2, 2, 3, 0x83, 0x5d, 500, 10 },
        { 5, 2, 4, 0xee, 0xa9, 500, 30 },
    };
    int fail = 0;
    for (int i = 0; i < 3; i++) {
        bb_set_mcs_item_t item;
        memset(&item, 0, sizeof(item));
        item.mcs          = air[i].mcs;
        item.ldpc_up_num  = air[i].up;
        item.snr_up       = air[i].snr_up;
        item.snr_dw       = air[i].snr_dw;
        item.rsv2         = 2;
        item.ldpc_dw_num  = air[i].dwn;
        item.up_keep_time = air[i].upk;
        item.dw_keep_time = air[i].dwk;
        int ret = bb_ioctl(g_hbb, BB_SET_MCS_ITEM, &item, NULL);
        printf("BB_SET_MCS_ITEM(air mcs=%u snr_up=%u snr_dw=%u ldpc_up=%u ldpc_dw=%u rsv2=2 up=%ums dw=%ums) ret=%d\n",
               item.mcs, item.snr_up, item.snr_dw, item.ldpc_up_num, item.ldpc_dw_num,
               item.up_keep_time, item.dw_keep_time, ret);
        if (ret) {
            fail = 1;
        }
    }
    return fail;
}

/* Raw BB_SET_PRJ_DISPATCH, same 256-byte buffer layout the vendor uses
 * (byte 0 = cmd id, bytes 1-3 padding, bytes 4.. payload). */
static int read_sock_info(int slot, bb_get_sock_info_out_t *out)
{
    bb_get_sock_info_in_t in = { .slot = (uint8_t)slot, .port = -1 };
    memset(out, 0, sizeof(*out));
    return bb_ioctl(g_hbb, BB_GET_SOCK_INFO, &in, out);
}

static uint64_t mono_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000u + (uint64_t)ts.tv_nsec / 1000000u;
}

static int cmd_rate(int argc, char **argv)
{
    int slot = 0, opt;
    optind = 1;
    permute_argv(argc, argv, "s:");
    while ((opt = getopt(argc, argv, "s:")) != -1) {
        if (opt == 's') {
            slot = atoi(optarg);
        } else {
            return 1;
        }
    }
    int seconds = optind < argc ? atoi(argv[optind]) : 5;
    if (seconds < 1) {
        seconds = 1;
    }

    bb_get_sock_info_out_t a, b;
    if (read_sock_info(slot, &a) != 0) {
        fprintf(stderr, "linkctl: BB_GET_SOCK_INFO failed\n");
        return 1;
    }
    uint64_t t0 = mono_ms();
    sleep((unsigned)seconds);
    if (read_sock_info(slot, &b) != 0) {
        fprintf(stderr, "linkctl: BB_GET_SOCK_INFO failed\n");
        return 1;
    }
    double dt = (double)(mono_ms() - t0) / 1000.0;

    bb_get_mcs_in_t mcs_in = { .dir = BB_DIR_TX, .slot = (uint8_t)slot };
    bb_get_mcs_out_t mcs_out;
    memset(&mcs_out, 0, sizeof(mcs_out));
    if (bb_ioctl(g_hbb, BB_GET_MCS, &mcs_in, &mcs_out) == 0) {
        printf("theoretical (BB_GET_MCS): mcs=%u throughput=%.1f Mbit/s\n", mcs_out.mcs,
               mcs_out.throughput / 1000.0);
    }
    printf("measured over %.1fs, slot %d:\n", dt, slot);
    print_bit_list("open ports", b.port_bmp, BB_SOCK_INFO_NUM);
    int any_counted = 0;
    for (int p = 0; p < BB_SOCK_INFO_NUM; p++) {
        if (!(b.port_bmp & (1u << p))) {
            continue;
        }
        for (int d = 0; d < BB_DIR_MAX; d++) {
            bb_sock_uni_t *u0 = &a.sock_info[p].uni_info[d];
            bb_sock_uni_t *u1 = &b.sock_info[p].uni_info[d];
            double mbit = (double)(u1->total_size - u0->total_size) * 8.0 / dt / 1e6;
            if (u1->total_size)
                any_counted = 1;
            printf("  port %d %s: %7.2f Mbit/s  (total=%llu bytes, buf %u/%u, overflow=%u, available=%u)\n", p,
                   d == BB_DIR_TX ? "tx" : "rx", mbit, (unsigned long long)u1->total_size, u1->data_size,
                   u1->buf_size, u1->overflow_cnt, u1->available);
        }
    }
    if (!any_counted) {
        printf("  (all counters zero: the daemon isn't counting these sockets -- see help)\n");
    }
    return 0;
}

static int cmd_frame_change(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "linkctl: frame-change needs 0 or 1\n");
        return 1;
    }
    bb_set_frame_change_t fc;
    memset(&fc, 0, sizeof(fc));
    fc.mode = (uint8_t)(atoi(argv[1]) ? 1 : 0);
    int ret = bb_ioctl(g_hbb, BB_SET_FRAME_CHANGE, &fc, NULL);
    printf("BB_SET_FRAME_CHANGE(mode=%u) ret=%d\n", fc.mode, ret);
    return ret ? 1 : 0;
}

static int cmd_cfg_dump(int argc, char **argv)
{
    int mode = 0, opt;
    optind = 1;
    permute_argv(argc, argv, "m:");
    while ((opt = getopt(argc, argv, "m:")) != -1) {
        if (opt == 'm') {
            mode = atoi(optarg);
        } else {
            return 1;
        }
    }
    const char *path = optind < argc ? argv[optind] : NULL;
    FILE *fp = NULL;
    if (path && !(fp = fopen(path, "wb"))) {
        perror("linkctl: cfg-dump: fopen");
        return 1;
    }

    unsigned offset = 0, total = 0, crc = 0;
    uint16_t seq = 1;
    int rc = 0;
    do {
        bb_get_cfg_in_t in;
        bb_get_cfg_out_t out;
        memset(&in, 0, sizeof(in));
        memset(&out, 0, sizeof(out));
        in.seq = seq++;
        in.mode = (uint8_t)mode;
        in.offset = (uint16_t)offset;
        in.length = BB_CFG_PAGE_SIZE - 12;
        int ret = bb_ioctl(g_hbb, BB_GET_CFG, &in, &out);
        if (ret) {
            fprintf(stderr, "linkctl: BB_GET_CFG(offset=%u) failed, ret=%d\n", offset, ret);
            rc = 1;
            break;
        }
        if (offset == 0) {
            total = out.total_length;
            crc = out.total_crc16;
            printf("cfg: mode=%d total_length=%u crc16=0x%04x\n", mode, total, crc);
        }
        if (out.length == 0 || out.length > sizeof(out.data)) {
            break;
        }
        if (fp) {
            fwrite(out.data, 1, out.length, fp);
        } else {
            for (unsigned i = 0; i < out.length; i++) {
                if (i % 16 == 0) {
                    printf("%s%04x:", i ? "\n" : "", offset + i);
                }
                printf(" %02x", out.data[i]);
            }
            printf("\n");
        }
        offset += out.length;
    } while (offset < total);

    if (fp) {
        fclose(fp);
        printf("cfg: wrote %u bytes to %s\n", offset, path);
    }
    return rc;
}

static int cmd_prj_cmd(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "linkctl: prj-cmd needs <cmd> [byte ...]\n");
        return 1;
    }
    uint8_t buf[256];
    memset(buf, 0, sizeof(buf));
    buf[0] = (uint8_t)strtoul(argv[1], NULL, 0);
    if (argc - 2 > 252) {
        fprintf(stderr, "linkctl: prj-cmd payload too long (max 252)\n");
        return 1;
    }
    for (int i = 2; i < argc; i++) {
        buf[4 + i - 2] = (uint8_t)strtoul(argv[i], NULL, 0);
    }
    int ret = bb_ioctl(g_hbb, BB_SET_PRJ_DISPATCH, buf, NULL);
    printf("BB_SET_PRJ_DISPATCH(cmd=0x%02x, %d payload bytes) ret=%d\n", buf[0], argc - 2, ret);
    return ret ? 1 : 0;
}

static int cmd_distance(int argc, char **argv)
{
    int count = 10, interval_ms = 500, opt;
    optind = 1;
    while ((opt = getopt(argc, argv, "n:i:")) != -1) {
        if (opt == 'n')
            count = atoi(optarg);
        else if (opt == 'i')
            interval_ms = atoi(optarg);
        else
            return 1;
    }
    for (int i = 0; i < count; i++) {
        bb_get_distc_result_in_t  in = { .slot_bmp = 0xff };
        bb_get_distc_result_out_t out;
        memset(&out, 0xa5, sizeof(out)); /* spot slots the chip never writes */
        int ret = bb_ioctl(g_hbb, BB_GET_DISTC_RESULT, &in, &out);
        printf("ret=%d", ret);
        for (int s = 0; s < BB_SLOT_MAX; s++)
            printf(out.distance[s] >= 0 ? " s%d=%dm" : " s%d=%d", s, out.distance[s]);
        printf("\n");
        fflush(stdout);
        if (i + 1 < count)
            usleep((useconds_t)interval_ms * 1000);
    }
    return 0;
}

static int cmd_rf_temp(int argc, char **argv)
{
    int channel = AR8030_RFTEMP_DEFAULT_ADC, force_arm = 0, opt;
    optind = 1;
    permute_argv(argc, argv, "c:a");
    while ((opt = getopt(argc, argv, "c:a")) != -1) {
        if (opt == 'c') {
            channel = atoi(optarg);
        } else if (opt == 'a') {
            force_arm = 1;
        } else {
            return 1;
        }
    }

    int mv = 0, ret = force_arm ? -1 : ar8030_rftemp_read_mv(g_hbb, channel, &mv);
    if (ret != 0 || mv <= 0) {
        /* Not armed (fresh chip, or nothing else armed it) -- arm and
         * wait out one measurement period before reading again. */
        ret = ar8030_rftemp_arm(g_hbb, channel);
        if (ret != 0) {
            fprintf(stderr, "linkctl: arming ADC channel %d failed (ret=%d)\n", channel, ret);
            return 1;
        }
        usleep((AR8030_RFTEMP_ARM_PERIOD + 200) * 1000);
        ret = ar8030_rftemp_read_mv(g_hbb, channel, &mv);
    }
    if (ret != 0) {
        fprintf(stderr, "linkctl: BB_GET_PRJ_DISPATCH(0x89) failed (ret=%d)\n", ret);
        return 1;
    }

    int c10 = ar8030_rftemp_mv_to_c10(mv);
    if (c10 == AR8030_RFTEMP_NONE) {
        printf("rf_temp: channel=%d adc=%d mV temp=n/a (below the thermistor table -- no sensor on this "
               "channel?)\n", channel, mv);
        return 1;
    }
    printf("rf_temp: channel=%d adc=%d mV temp=%d.%d C\n", channel, mv, c10 / 10, c10 % 10);
    return 0;
}

static int cmd_batt(int argc, char **argv)
{
    int channel = AR8030_BATT_DEFAULT_ADC, scale = AR8030_BATT_DEFAULT_SCALE;
    int offset = AR8030_BATT_DEFAULT_OFFSET_MV, count = 10, force_arm = 0, opt;
    optind = 1;
    permute_argv(argc, argv, "c:k:o:n:a");
    while ((opt = getopt(argc, argv, "c:k:o:n:a")) != -1) {
        if (opt == 'c') {
            channel = atoi(optarg);
        } else if (opt == 'k') {
            scale = atoi(optarg);
        } else if (opt == 'o') {
            offset = atoi(optarg);
        } else if (opt == 'n') {
            count = atoi(optarg);
        } else if (opt == 'a') {
            force_arm = 1;
        } else {
            return 1;
        }
    }
    if (count < 1)
        count = 1;

    int mv = 0, ret = force_arm ? -1 : ar8030_rftemp_read_mv(g_hbb, channel, &mv);
    if (ret != 0 || mv <= 0) {
        /* Not armed yet (fresh chip) -- or really 0 V, which re-arming
         * can't tell apart; the read below settles it either way. */
        ret = ar8030_rftemp_arm(g_hbb, channel);
        if (ret != 0) {
            fprintf(stderr, "linkctl: arming ADC channel %d failed (ret=%d)\n", channel, ret);
            return 1;
        }
        usleep((AR8030_RFTEMP_ARM_PERIOD + 200) * 1000);
    }

    int sum = 0, n = 0, lo = 0, hi = 0;
    for (int i = 0; i < count; i++) {
        if (i)
            usleep(250 * 1000);
        ret = ar8030_rftemp_read_mv(g_hbb, channel, &mv);
        if (ret != 0) {
            fprintf(stderr, "linkctl: BB_GET_PRJ_DISPATCH(0x89) failed (ret=%d)\n", ret);
            return 1;
        }
        if (!n || mv < lo)
            lo = mv;
        if (!n || mv > hi)
            hi = mv;
        sum += mv;
        n++;
    }
    int avg = sum / n, batt = ar8030_batt_mv(avg, scale, offset);
    if (batt < 0) {
        printf("batt: channel=%d adc=%d mV supply=n/a (nothing on the power input?)\n", channel, avg);
        return 1;
    }
    printf("batt: channel=%d adc=%d mV (%d..%d, n=%d) supply=%d.%02d V (x%d %+d mV)\n", channel, avg, lo, hi, n,
           batt / 1000, (batt % 1000) / 10, scale, offset);
    return 0;
}

static int cmd_power_mode(int argc, char **argv)
{
    if (argc < 2) {
        bb_get_pwr_mode_out_t m;
        memset(&m, 0, sizeof(m));
        int ret = bb_ioctl(g_hbb, BB_GET_POWER_MODE, NULL, &m);
        if (ret) {
            fprintf(stderr, "linkctl: BB_GET_POWER_MODE failed, ret=%d\n", ret);
            return 1;
        }
        printf("%s\n", pwr_mode_name(m.pwr_mode));
        return 0;
    }
    if (strcmp(argv[1], "auto") && strcmp(argv[1], "manual")) {
        fprintf(stderr, "linkctl: power-mode needs 'auto' or 'manual'\n");
        return 1;
    }
    /* Naming is inverted from mcs-mode/channel-mode's own auto=0/manual=1
     * convention -- bb_phy_pwr_mode_e itself defines OPENLOOP=0 (manual: a
     * fixed value from BB_SET_POWER) and CLOSELOOP=1 (auto: the chip's own
     * feedback loop manages it) -- so map explicitly rather than reusing
     * those other commands' !strcmp(...,"auto") pattern verbatim. */
    bb_set_pwr_mode_in_t m = { .pwr_mode = (uint8_t)(!strcmp(argv[1], "auto") ? BB_PHY_PWR_CLOSELOOP
                                                                               : BB_PHY_PWR_OPENLOOP) };
    int ret = bb_ioctl(g_hbb, BB_SET_POWER_MODE, &m, NULL);
    printf("BB_SET_POWER_MODE(pwr_mode=%u) ret=%d\n", m.pwr_mode, ret);
    return ret ? 1 : 0;
}

static int cmd_power(int argc, char **argv)
{
    int user = 0, wait_s = 0, opt;
    optind = 1;
    permute_argv(argc, argv, "u:w:");
    while ((opt = getopt(argc, argv, "u:w:")) != -1) {
        switch (opt) {
        case 'u':
            user = atoi(optarg);
            break;
        case 'w':
            wait_s = atoi(optarg);
            break;
        default:
            return 1;
        }
    }
    if (optind >= argc) {
        fprintf(stderr, "linkctl: power needs a dBm value (0-31)\n");
        return 1;
    }
    int dbm = atoi(argv[optind]);
    if (dbm < 0 || dbm > 31) {
        fprintf(stderr, "linkctl: power must be 0-31 dBm, got %d\n", dbm);
        return 1;
    }
    if (wait_s > 0 && wait_for_connect(0, wait_s) != 0) {
        fprintf(stderr, "linkctl: link not CONNECT after %ds, applying anyway\n", wait_s);
    }

    bb_set_pwr_in_t sp = { .usr = (uint8_t)user, .pwr = (uint8_t)dbm };
    int ret = bb_ioctl(g_hbb, BB_SET_POWER, &sp, NULL);
    printf("BB_SET_POWER(usr=%d, pwr=%ddBm / %.1fmW) ret=%d\n", user, dbm, dbm_to_mw((uint8_t)dbm), ret);
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
    else if (!strcmp(cmd, "mcs-range"))
        rc = cmd_mcs_range(argc - 1, argv + 1);
    else if (!strcmp(cmd, "mcs-table"))
        rc = cmd_mcs_table(argc - 1, argv + 1);
    else if (!strcmp(cmd, "mcs-table-air"))
        rc = cmd_mcs_table_air();
    else if (!strcmp(cmd, "rate"))
        rc = cmd_rate(argc - 1, argv + 1);
    else if (!strcmp(cmd, "frame-change"))
        rc = cmd_frame_change(argc - 1, argv + 1);
    else if (!strcmp(cmd, "cfg-dump"))
        rc = cmd_cfg_dump(argc - 1, argv + 1);
    else if (!strcmp(cmd, "distance"))
        rc = cmd_distance(argc - 1, argv + 1);
    else if (!strcmp(cmd, "rf-temp"))
        rc = cmd_rf_temp(argc - 1, argv + 1);
    else if (!strcmp(cmd, "batt"))
        rc = cmd_batt(argc - 1, argv + 1);
    else if (!strcmp(cmd, "prj-cmd"))
        rc = cmd_prj_cmd(argc - 1, argv + 1);
    else if (!strcmp(cmd, "retx"))
        rc = cmd_retx(argc - 1, argv + 1);
    else if (!strcmp(cmd, "retx-watch"))
        rc = cmd_retx_watch(argc - 1, argv + 1);
    else if (!strcmp(cmd, "power-mode"))
        rc = cmd_power_mode(argc - 1, argv + 1);
    else if (!strcmp(cmd, "power"))
        rc = cmd_power(argc - 1, argv + 1);
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
