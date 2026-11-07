/*
 * ar8030-xdata -- write to the air unit's chip UART2 from the ground host.
 *
 * In work mode 3 the air chip bridges its own UART2 to radio socket port 1,
 * a port the firmware keeps for itself (on the Caddx Ascent Lite+ UART2 is
 * the pair of extra UART pads, the head tracking gimbal port, 115200 8N1 from
 * cfg uart2_config). The ground chip routes its host data channel
 * (BB_SET_PRJ_DISPATCH2 sub-command 6) to radio port 1, so bytes written
 * here leave the air unit's UART2 pads with no software on the air host
 * involved. Verified on a VRX Pro + Ascent Lite+.
 *
 * The VRX Pro's ground image (bb_demo_cx485_2PA) cuts that channel into CRSF
 * frames -- sync 0xC8, length byte (type + payload + CRC), type, payload,
 * CRC-8 poly 0xD5 over type + payload -- and drops every byte outside a valid
 * frame (xstats "gap_bytes"). The air chip forwards the frame unchanged, so
 * the UART2 pads see the whole CRSF frame: 3 bytes in front of the payload,
 * 1 behind. A receiver that hunts for its own sync bytes (the Walksnail/Caddx
 * gimbal head tracking frame starts with A5 5A) skips them. -r sends the
 * payload as it is, for a ground chip that does not frame this channel.
 *
 * The ground chip drops a new radio-port-1 packet while more than 32 bytes
 * still wait in that port's tx buffer, and nothing is sent while the link is
 * down. Keep frames small and the rate at about one frame per radio cycle or
 * less; this is a state channel, not a stream.
 *
 *   ar8030-xdata send [-t type] [-r] <byte> ...   one frame, then exit
 *   ar8030-xdata relay [-l host:port] [-t type] [-r] [-a] [-v]
 *                                                 one frame per UDP datagram
 */
#include "ar8030.h"
#include "bb_api.h"
#include "bb_dev.h"
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#define DISPATCH2_HDR    4              /* sub, 0, u16 LE length */
#define SUB_XDATA        6
#define CRSF_SYNC        0xc8
#define CRSF_MAX_PAYLOAD 60             /* length byte 2..62 */
#define RAW_MAX_PAYLOAD  ((int)sizeof(bb_set_prj_dispatch2_in_t) - DISPATCH2_HDR)

static bb_dev_handle_t *g_hbb;
static bb_host_t *g_phost;
static bb_dev_t **g_devs;
static volatile sig_atomic_t g_stop;
static int g_raw;
static uint8_t g_type = 0x7f;

static void on_signal(int sig)
{
    (void)sig;
    g_stop = 1;
}

static void usage(void)
{
    fprintf(stderr,
            "usage: ar8030-xdata send [-t type] [-r] <byte> ...\n"
            "       ar8030-xdata relay [-l host:port] [-t type] [-r] [-a] [-v]\n"
            "\n"
            "Writes to the air unit's chip UART2 through the ground chip's host data\n"
            "channel (BB_SET_PRJ_DISPATCH2 sub 6 -> radio port 1 -> air UART2).\n"
            "\n"
            "  send     one frame with the given bytes (decimal or 0x hex), then exit\n"
            "  relay    one frame per UDP datagram received on -l (default\n"
            "           127.0.0.1:5770); reconnects to ar8030d when it goes away\n"
            "  -t type  CRSF frame type byte (default 0x7f); the ground chip only\n"
            "           forwards CRSF frames, which leave the air UART2 whole:\n"
            "           C8 <len> <type> <payload> <crc8>\n"
            "  -r       send the payload as it is, no CRSF frame (for a ground chip\n"
            "           that does not frame this channel)\n"
            "  -a       relay every datagram; default: only the newest one that is\n"
            "           waiting, which is what a state stream (head tracking) wants\n"
            "  -v       print a counter line every 5 s\n"
            "\n"
            "Payload at most %d bytes (CRSF) or %d (raw); keep frames small, the\n"
            "ground chip drops new packets on that port while more than 32 bytes\n"
            "are still queued.\n",
            CRSF_MAX_PAYLOAD, RAW_MAX_PAYLOAD);
}

static int max_payload(void)
{
    return g_raw ? RAW_MAX_PAYLOAD : CRSF_MAX_PAYLOAD;
}

static int connect_daemon(int quiet)
{
    if (bb_host_connect(&g_phost, "127.0.0.1", BB_PORT_DEFAULT)) {
        if (!quiet)
            fprintf(stderr, "ar8030-xdata: connect to ar8030d failed\n");
        g_phost = NULL;
        return -1;
    }
    if (bb_dev_getlist(g_phost, &g_devs) <= 0) {
        if (!quiet)
            fprintf(stderr, "ar8030-xdata: no AR8030 device known to ar8030d\n");
        bb_host_disconnect(g_phost);
        g_phost = NULL;
        g_devs = NULL;
        return -1;
    }
    g_hbb = bb_dev_open(g_devs[0]);
    if (!g_hbb) {
        if (!quiet)
            fprintf(stderr, "ar8030-xdata: bb_dev_open failed\n");
        bb_dev_freelist(g_devs);
        bb_host_disconnect(g_phost);
        g_devs = NULL;
        g_phost = NULL;
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
    g_hbb = NULL;
    g_devs = NULL;
    g_phost = NULL;
}

static uint8_t crc8_d5(const uint8_t *p, int n)
{
    uint8_t c = 0;

    while (n--) {
        c ^= *p++;
        for (int b = 0; b < 8; b++)
            c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0xd5) : (uint8_t)(c << 1);
    }
    return c;
}

/* one payload into the ground chip's host data channel; bb_ioctl's result */
static int send_frame(const uint8_t *p, int n)
{
    static bb_set_prj_dispatch2_in_t in;
    uint8_t *f = in.data + DISPATCH2_HDR;
    int len = n;

    memset(&in, 0, sizeof(in));
    in.data[0] = SUB_XDATA;
    if (g_raw) {
        memcpy(f, p, (size_t)n);
    } else {
        f[0] = CRSF_SYNC;
        f[1] = (uint8_t)(n + 2);
        f[2] = g_type;
        memcpy(f + 3, p, (size_t)n);
        f[3 + n] = crc8_d5(f + 2, n + 1);
        len = n + 4;
    }
    in.data[2] = (uint8_t)len;
    in.data[3] = (uint8_t)(len >> 8);
    return bb_ioctl(g_hbb, BB_SET_PRJ_DISPATCH2, &in, NULL);
}

static int parse_common(int opt)
{
    switch (opt) {
    case 't': g_type = (uint8_t)strtoul(optarg, NULL, 0); return 0;
    case 'r': g_raw = 1; return 0;
    default: return -1;
    }
}

static int cmd_send(int argc, char **argv)
{
    uint8_t buf[RAW_MAX_PAYLOAD];
    int opt, n = 0;

    while ((opt = getopt(argc, argv, "t:r")) != -1) {
        if (parse_common(opt)) {
            usage();
            return 2;
        }
    }
    if (optind >= argc || argc - optind > max_payload()) {
        usage();
        return 2;
    }
    for (int i = optind; i < argc; i++)
        buf[n++] = (uint8_t)strtoul(argv[i], NULL, 0);
    if (connect_daemon(0))
        return 1;
    int ret = send_frame(buf, n);
    printf("BB_SET_PRJ_DISPATCH2(sub 6, %d payload bytes, %s) ret=%d\n", n, g_raw ? "raw" : "CRSF", ret);
    disconnect_daemon();
    return ret ? 1 : 0;
}

static int open_udp(const char *spec)
{
    char host[64];
    const char *colon = strrchr(spec, ':');
    struct sockaddr_in sa;
    int fd;

    memset(&sa, 0, sizeof(sa));
    sa.sin_family = AF_INET;
    if (!colon || (size_t)(colon - spec) >= sizeof(host)) {
        fprintf(stderr, "ar8030-xdata: -l wants host:port\n");
        return -1;
    }
    memcpy(host, spec, (size_t)(colon - spec));
    host[colon - spec] = 0;
    sa.sin_port = htons((uint16_t)atoi(colon + 1));
    if (inet_pton(AF_INET, host, &sa.sin_addr) != 1) {
        fprintf(stderr, "ar8030-xdata: bad address %s\n", host);
        return -1;
    }
    if ((fd = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0)) < 0 ||
        bind(fd, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
        perror("ar8030-xdata: udp");
        if (fd >= 0)
            close(fd);
        return -1;
    }
    return fd;
}

static double now_s(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + ts.tv_nsec / 1e9;
}

static int cmd_relay(int argc, char **argv)
{
    const char *listen_on = "127.0.0.1:5770";
    uint8_t buf[2048];
    int opt, all = 0, verbose = 0, fd;
    unsigned long rx = 0, sent = 0, skipped = 0, failed = 0, big = 0;
    double next_report, next_connect = 0;

    while ((opt = getopt(argc, argv, "l:t:rav")) != -1) {
        if (opt == 'l')
            listen_on = optarg;
        else if (opt == 'a')
            all = 1;
        else if (opt == 'v')
            verbose = 1;
        else if (parse_common(opt)) {
            usage();
            return 2;
        }
    }
    if ((fd = open_udp(listen_on)) < 0)
        return 1;
    fprintf(stderr, "ar8030-xdata: relaying UDP %s to the air UART2 (%s)\n", listen_on,
            g_raw ? "raw" : "CRSF frames");
    next_report = now_s() + 5;
    while (!g_stop) {
        ssize_t n = recv(fd, buf, sizeof(buf), 0);
        if (n < 0) {
            if (errno == EINTR)
                continue;
            perror("ar8030-xdata: recv");
            break;
        }
        rx++;
        if (!all) {                     /* keep only the newest datagram already waiting */
            ssize_t m;
            while ((m = recv(fd, buf, sizeof(buf), MSG_DONTWAIT)) >= 0) {
                n = m;
                rx++;
                skipped++;
            }
        }
        if (n == 0 || n > max_payload()) {
            big++;
            continue;
        }
        if (!g_hbb) {
            double t = now_s();
            if (t < next_connect || connect_daemon(1)) {
                if (t >= next_connect)
                    next_connect = t + 1.0;
                failed++;
                continue;
            }
            fprintf(stderr, "ar8030-xdata: connected to ar8030d\n");
        }
        if (send_frame(buf, (int)n)) {
            failed++;
            fprintf(stderr, "ar8030-xdata: BB_SET_PRJ_DISPATCH2 failed, reconnecting\n");
            disconnect_daemon();
            next_connect = now_s() + 1.0;
        } else {
            sent++;
        }
        if (verbose && now_s() >= next_report) {
            next_report += 5;
            fprintf(stderr, "ar8030-xdata: rx %lu sent %lu skipped %lu failed %lu bad_size %lu\n",
                    rx, sent, skipped, failed, big);
        }
    }
    close(fd);
    disconnect_daemon();
    return 0;
}

int main(int argc, char **argv)
{
    struct sigaction sa;

    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_signal;          /* no SA_RESTART: recv() returns EINTR */
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    signal(SIGPIPE, SIG_IGN);
    if (argc < 2) {
        usage();
        return 2;
    }
    if (!strcmp(argv[1], "send"))
        return cmd_send(argc - 1, argv + 1);
    if (!strcmp(argv[1], "relay"))
        return cmd_relay(argc - 1, argv + 1);
    usage();
    return 2;
}
