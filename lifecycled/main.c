/*
 * ar8030-lifecycled -- owns the AR8030 chip's whole life (hardware reset,
 * pairing, reconnect-on-drop, tuning, hook-script notification on state
 * change) in one place, replacing the pile of external shell-script
 * watchdogs (autoreconnect/device_watchdog/apply_link_tuning in
 * S65ar8030-transport-tx and S97ar8030, ar8030-led-status.sh) that used to
 * poll `ar8030-linkctl status` from outside and guess.
 *
 * Deliberately a SEPARATE PROCESS from ar8030d, not a thread inside it --
 * confirmed live that running this logic as a pthread inside the daemon
 * process itself, calling back into the client library's own blocking
 * network I/O against the *same* process that's also the RPC server, can
 * wedge (a standalone test process doing the identical connect/subscribe/
 * poll sequence ran clean for 40s with zero thread growth on the daemon
 * side; the same code as an in-process thread produced 100+ stuck threads
 * within a minute). A genuinely separate process talking to ar8030d over
 * loopback -- exactly like ar8030-linkctl/ar8030-pair/ar8030-status
 * already do -- has none of that risk.
 *
 * Two threads of its own, though: lifecycle_thread_main() (the passive
 * reconnect-follower) always runs as a real pthread now, and -- only
 * when --bind-gpio is given -- lifecycle_bind_thread_main() (the
 * physical bind-button watcher) runs in this thread. That pairing is
 * safe in a way the paragraph above is specifically warning against:
 * lifecycle_bind_thread_main() never touches ctx->client or makes any
 * blocking SDK RPC call itself -- only plain sysfs GPIO reads and
 * forking the separate ar8030-pair process -- so it doesn't share the
 * failure mode a second RPC-calling thread inside ar8030d would have.
 * See lifecycle_bind.c's own header comment.
 */
#include "lc_log.h"
#include "lifecycle.h"
#include "lifecycle_bind.h"
#include "lifecycle_client.h"
#include "lifecycle_http.h"
#include "lifecycle_tuning.h"
#include "../common/ar8030_batt.h"
#include <getopt.h>
#include <pthread.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>

static void print_help(const char* argv0)
{
    printf("usage: %s [options]\n", argv0);
    printf("  -p, --port <n>            ar8030d RPC port to connect to (default %d)\n", BB_PORT_DEFAULT);
    printf("  --role ap|dev             device role (default ap)\n");
    printf("  --reset-method devmem|gpio-sysfs|none\n");
    printf("                            hardware-reset backend (default none)\n");
    printf("  --reset-devmem-addr <hex> register address, devmem method only\n");
    printf("  --reset-gpio <list>       comma-separated GPIO numbers, gpio-sysfs method only\n");
    printf("  --hook-dir <path>         hook script root (default /etc/ar8030/hooks.d)\n");
    printf("  --cfg-path <path>         baseband JSON to persist pairing/tuning against\n");
    printf("  --default-bandwidth <n>   fallback bandwidth if no tuning state yet (default 20)\n");
    printf("  --default-channel <n|auto|none>\n");
    printf("                            AP only: channel while none is persisted yet: a channel-table\n");
    printf("                            index, auto (chip's channel adaptation) or none (leave\n");
    printf("                            the chip's own startup channel alone) (default 25 = 5805 MHz)\n");
    printf("  --default-power <mw|auto|none>\n");
    printf("                            output power while none is persisted yet: air 400/\n");
    printf("                            200/100/25 mW, ground auto/500/200/100/25 mW, or none\n");
    printf("                            (leave the chip's config-file power alone)\n");
    printf("                            (default: air 400 mW = 26 dBm, ground 500 mW, or the\n");
    printf("                            highest level within --max-power)\n");
    printf("  --max-power <mW>          highest level this board may use: higher ones are\n");
    printf("                            not offered (GET /api/v1/power), refused and not\n");
    printf("                            taken from the persisted setting; ground auto stays\n");
    printf("                            (BetaFPV P1: 200, its PA's calibrated 22 dBm)\n");
    printf("                            (default: none)\n");
    printf("  --bind-gpio <n>           watch this GPIO for the physical bind button\n");
    printf("                            (press: pair; hold 5 s: hooks.d/longpress)\n");
    printf("                            (default: none -- no physical button on this board)\n");
    printf("  --http-port <n>           start the HTTP control API on this port\n");
    printf("                            (default: 0 -- disabled; see README's \"HTTP\n");
    printf("                            control API\" section)\n");
    printf("  --http-bind <addr>        address to bind the HTTP control API to\n");
    printf("                            (default: 0.0.0.0, every interface)\n");
    printf("  --frame-change 0|1        AP only: apply BB_SET_FRAME_CHANGE(1) after every connect\n");
    printf("                            (default 1; 0 leaves the chip's frame structure alone)\n");
    printf("  --rf-temp-adc <ch>        poll RF-board temperature on this AR8030 ADC channel\n");
    printf("                            (default: none -- board-specific; Caddx Ascent: 4)\n");
    printf("  --rf-temp-file <path>     also write it there in whole degC, \"\" = don't\n");
    printf("                            (default /tmp/rf_temperature.msg)\n");
    printf("  --batt-adc <ch>           poll the supply voltage on this AR8030 ADC channel\n");
    printf("                            (default: none -- board-specific; Ascent Lite: 0)\n");
    printf("  --batt-scale <n>          supply mV = ADC mV * n + offset (default 16)\n");
    printf("  --batt-offset-mv <mv>     ... + this (default 1200; both measured on the Lite)\n");
    printf("  --batt-file <path>        also write it there as volts (\"11.98\"), \"\" = don't\n");
    printf("  --rf-path-b-off           single-PA board: switch RF path B (TX and RX) off at\n");
    printf("                            startup, as stock does on the Ascent Lite\n");
    printf("                            (default: \"\")\n");
    printf("  --link-policy <0|1|2>     stock's runtime MCS policy for this video_strategy\n");
    printf("                            (Ascent Lite ships 2): air uplink table + MCS range +\n");
    printf("                            rfo_kikp at startup, ground downlink table on every\n");
    printf("                            connect (default: none -- ar8030.json's tables)\n");
    printf("  --fem-ctrl                air, Ascent Lite: stock's power extras -- power-auto\n");
    printf("                            mode 3 and the FEM bit at >= 24 dBm\n");
    printf("  --standby-dbm <dBm>       air: stock's low-power mode -- this chip dBm target\n");
    printf("                            while the FC reports disarmed (POST /api/v1/armed)\n");
    printf("                            or the video is paused for heat, entered with a link\n");
    printf("                            up, left on arm (Ascent Lite stock: 10) (default: off)\n");
    printf("  --soc-temp-cmd <cmd>      air: command printing the SoC temperature in degC,\n");
    printf("                            run every 2 s (default: none)\n");
    printf("  --overheat-c <C>          air: pause the video at or above this SoC\n");
    printf("                            temperature, armed or not -- hooks.d/overheat, plus\n");
    printf("                            --standby-dbm (needs --soc-temp-cmd) (default: off)\n");
    printf("  --overheat-resume-c <C>   air: hooks.d/resume once cooled to this (default:\n");
    printf("                            --overheat-c - %d)\n", LC_OVERHEAT_HYST_C);
    printf("  --daemon-log <level>      ar8030d's log level once connected: debug, info,\n");
    printf("                            warn, error or keep (default: info). ar8030d's own\n");
    printf("                            debug logs 3-4 lines per socket WRITE frame, which\n");
    printf("                            on the air costs ~1 ms per video frame\n");
    printf("  -h, --help                this help\n");
}

static void parse_reset_gpio_list(lc_config_t* cfg, const char* arg)
{
    char buf[128];
    strncpy(buf, arg, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = 0;

    cfg->reset_gpio_count = 0;
    char* tok = strtok(buf, ",");
    while (tok && cfg->reset_gpio_count < LC_MAX_RESET_GPIOS) {
        cfg->reset_gpios[cfg->reset_gpio_count++] = (int)strtol(tok, NULL, 10);
        tok = strtok(NULL, ",");
    }
}

enum {
    OPT_ROLE = 1000,
    OPT_RESET_METHOD,
    OPT_RESET_DEVMEM_ADDR,
    OPT_RESET_GPIO,
    OPT_HOOK_DIR,
    OPT_CFG_PATH,
    OPT_DEFAULT_BANDWIDTH,
    OPT_DEFAULT_CHANNEL,
    OPT_DEFAULT_POWER,
    OPT_MAX_POWER,
    OPT_BIND_GPIO,
    OPT_HTTP_PORT,
    OPT_HTTP_BIND,
    OPT_FRAME_CHANGE,
    OPT_RF_TEMP_ADC,
    OPT_RF_TEMP_FILE,
    OPT_LINK_POLICY,
    OPT_FEM_CTRL,
    OPT_BATT_ADC,
    OPT_BATT_SCALE,
    OPT_BATT_OFFSET_MV,
    OPT_BATT_FILE,
    OPT_RF_PATH_B_OFF,
    OPT_STANDBY_DBM,
    OPT_SOC_TEMP_CMD,
    OPT_OVERHEAT_C,
    OPT_OVERHEAT_RESUME_C,
    OPT_DAEMON_LOG,
};

static void handle_sigterm(int sig)
{
    (void)sig;
    lifecycle_request_shutdown();
}

int main(int argc, char** argv)
{
    lc_config_t cfg = {
        .rpc_port          = BB_PORT_DEFAULT,
        .role              = LC_ROLE_AP,
        .reset_method      = LC_RESET_METHOD_NONE,
        .reset_devmem_addr = 0,
        .reset_gpio_count  = 0,
        .hook_dir          = "/etc/ar8030/hooks.d",
        .cfg_path          = "",
        .default_bandwidth = 20,
        .frame_change      = 1,
        .default_channel   = 25, /* 5805 MHz in ar8030.json's channel table */
        .default_power     = LC_POWER_ROLE_DEFAULT,
        .max_power         = 0,
        .no_lifecycle      = 0,
        .bind_gpio         = -1,
        .http_bind         = "0.0.0.0",
        .http_port         = 0,
        .rf_temp_adc       = -1,
        .rf_temp_file      = "/tmp/rf_temperature.msg",
        .batt_adc          = -1,
        .batt_scale        = AR8030_BATT_DEFAULT_SCALE,
        .batt_offset_mv    = AR8030_BATT_DEFAULT_OFFSET_MV,
        .batt_file         = "",
        .link_policy       = -1,
        .standby_dbm       = -1,
        .soc_temp_cmd      = "",
        .overheat_c        = -1,
        .overheat_resume_c = -1,
        .daemon_log        = LC_DAEMON_LOG_INFO,
    };

    static struct option long_options[] = {
        {"port",              required_argument, 0, 'p'                  },
        {"role",              required_argument, 0, OPT_ROLE             },
        {"reset-method",      required_argument, 0, OPT_RESET_METHOD     },
        {"reset-devmem-addr", required_argument, 0, OPT_RESET_DEVMEM_ADDR},
        {"reset-gpio",        required_argument, 0, OPT_RESET_GPIO       },
        {"hook-dir",          required_argument, 0, OPT_HOOK_DIR         },
        {"cfg-path",          required_argument, 0, OPT_CFG_PATH         },
        {"default-bandwidth", required_argument, 0, OPT_DEFAULT_BANDWIDTH},
        {"default-channel",   required_argument, 0, OPT_DEFAULT_CHANNEL  },
        {"default-power",     required_argument, 0, OPT_DEFAULT_POWER    },
        {"max-power",         required_argument, 0, OPT_MAX_POWER        },
        {"bind-gpio",         required_argument, 0, OPT_BIND_GPIO        },
        {"http-port",         required_argument, 0, OPT_HTTP_PORT        },
        {"http-bind",         required_argument, 0, OPT_HTTP_BIND        },
        {"frame-change",      required_argument, 0, OPT_FRAME_CHANGE     },
        {"rf-temp-adc",       required_argument, 0, OPT_RF_TEMP_ADC      },
        {"rf-temp-file",      required_argument, 0, OPT_RF_TEMP_FILE     },
        {"batt-adc",          required_argument, 0, OPT_BATT_ADC         },
        {"batt-scale",        required_argument, 0, OPT_BATT_SCALE       },
        {"batt-offset-mv",    required_argument, 0, OPT_BATT_OFFSET_MV   },
        {"batt-file",         required_argument, 0, OPT_BATT_FILE        },
        {"rf-path-b-off",     no_argument,       0, OPT_RF_PATH_B_OFF    },
        {"link-policy",       required_argument, 0, OPT_LINK_POLICY      },
        {"fem-ctrl",          no_argument,       0, OPT_FEM_CTRL         },
        {"standby-dbm",       required_argument, 0, OPT_STANDBY_DBM      },
        {"soc-temp-cmd",      required_argument, 0, OPT_SOC_TEMP_CMD     },
        {"overheat-c",        required_argument, 0, OPT_OVERHEAT_C       },
        {"overheat-resume-c", required_argument, 0, OPT_OVERHEAT_RESUME_C},
        {"daemon-log",        required_argument, 0, OPT_DAEMON_LOG       },
        {"help",              no_argument,       0, 'h'                  },
        {0,                   0,                 0, 0                    },
    };

    int c;
    while ((c = getopt_long(argc, argv, "p:h", long_options, NULL)) != -1) {
        switch (c) {
        case 'p':
            cfg.rpc_port = (int)strtoul(optarg, NULL, 10);
            break;
        case OPT_ROLE:
            cfg.role = (strcmp(optarg, "dev") == 0) ? LC_ROLE_DEV : LC_ROLE_AP;
            break;
        case OPT_RESET_METHOD:
            if (strcmp(optarg, "devmem") == 0) {
                cfg.reset_method = LC_RESET_METHOD_DEVMEM;
            } else if (strcmp(optarg, "gpio-sysfs") == 0) {
                cfg.reset_method = LC_RESET_METHOD_GPIO_SYSFS;
            } else {
                cfg.reset_method = LC_RESET_METHOD_NONE;
            }
            break;
        case OPT_RESET_DEVMEM_ADDR:
            cfg.reset_devmem_addr = strtoul(optarg, NULL, 0);
            break;
        case OPT_RESET_GPIO:
            parse_reset_gpio_list(&cfg, optarg);
            break;
        case OPT_HOOK_DIR:
            strncpy(cfg.hook_dir, optarg, sizeof(cfg.hook_dir) - 1);
            break;
        case OPT_CFG_PATH:
            strncpy(cfg.cfg_path, optarg, sizeof(cfg.cfg_path) - 1);
            break;
        case OPT_DEFAULT_BANDWIDTH:
            cfg.default_bandwidth = (int)strtoul(optarg, NULL, 10);
            break;
        case OPT_DEFAULT_CHANNEL:
            if (lc_channel_parse(optarg, &cfg.default_channel) != 0) {
                fprintf(stderr, "invalid --default-channel '%s' (index, auto or none)\n", optarg);
                return 1;
            }
            break;
        case OPT_DEFAULT_POWER:
            if (lc_power_parse(optarg, &cfg.default_power) != 0) {
                fprintf(stderr, "invalid --default-power '%s' (mW level, auto or none)\n", optarg);
                return 1;
            }
            break;
        case OPT_MAX_POWER: {
            char* end;
            cfg.max_power = (int)strtol(optarg, &end, 10);
            if (end == optarg || (*end && strcmp(end, "mw") != 0 && strcmp(end, "mW") != 0) || cfg.max_power <= 0) {
                fprintf(stderr, "invalid --max-power '%s' (mW)\n", optarg);
                return 1;
            }
            break;
        }
        case OPT_BIND_GPIO:
            cfg.bind_gpio = (int)strtol(optarg, NULL, 10);
            break;
        case OPT_HTTP_PORT:
            cfg.http_port = (uint16_t)strtoul(optarg, NULL, 10);
            break;
        case OPT_FRAME_CHANGE:
            cfg.frame_change = atoi(optarg) ? 1 : 0;
            break;
        case OPT_HTTP_BIND:
            strncpy(cfg.http_bind, optarg, sizeof(cfg.http_bind) - 1);
            break;
        case OPT_RF_TEMP_ADC:
            cfg.rf_temp_adc = (int)strtol(optarg, NULL, 10);
            break;
        case OPT_RF_TEMP_FILE:
            snprintf(cfg.rf_temp_file, sizeof(cfg.rf_temp_file), "%s", optarg);
            break;
        case OPT_BATT_ADC:
            cfg.batt_adc = (int)strtol(optarg, NULL, 10);
            break;
        case OPT_BATT_SCALE:
            cfg.batt_scale = (int)strtol(optarg, NULL, 10);
            break;
        case OPT_BATT_OFFSET_MV:
            cfg.batt_offset_mv = (int)strtol(optarg, NULL, 10);
            break;
        case OPT_BATT_FILE:
            snprintf(cfg.batt_file, sizeof(cfg.batt_file), "%s", optarg);
            break;
        case OPT_RF_PATH_B_OFF:
            cfg.rf_path_b_off = 1;
            break;
        case OPT_LINK_POLICY:
            cfg.link_policy = (int)strtol(optarg, NULL, 10);
            if (cfg.link_policy < 0 || cfg.link_policy > 2) {
                fprintf(stderr, "invalid --link-policy '%s' (0, 1 or 2)\n", optarg);
                return 1;
            }
            break;
        case OPT_FEM_CTRL:
            cfg.fem_ctrl = 1;
            break;
        case OPT_STANDBY_DBM: {
            char* end;
            cfg.standby_dbm = (int)strtol(optarg, &end, 10);
            if (end == optarg || *end || cfg.standby_dbm < 0 || cfg.standby_dbm > 30) {
                fprintf(stderr, "invalid --standby-dbm '%s' (0..30)\n", optarg);
                return 1;
            }
            break;
        }
        case OPT_SOC_TEMP_CMD:
            snprintf(cfg.soc_temp_cmd, sizeof(cfg.soc_temp_cmd), "%s", optarg);
            break;
        case OPT_OVERHEAT_C: {
            char* end;
            cfg.overheat_c = (int)strtol(optarg, &end, 10);
            if (end == optarg || *end || cfg.overheat_c < 50 || cfg.overheat_c > 150) {
                fprintf(stderr, "invalid --overheat-c '%s' (50..150)\n", optarg);
                return 1;
            }
            break;
        }
        case OPT_OVERHEAT_RESUME_C: {
            char* end;
            cfg.overheat_resume_c = (int)strtol(optarg, &end, 10);
            if (end == optarg || *end || cfg.overheat_resume_c < 40 || cfg.overheat_resume_c > 150) {
                fprintf(stderr, "invalid --overheat-resume-c '%s' (40..150)\n", optarg);
                return 1;
            }
            break;
        }
        case OPT_DAEMON_LOG:
            cfg.daemon_log = lc_daemon_log_parse(optarg);
            if (cfg.daemon_log == -2) {
                fprintf(stderr, "invalid --daemon-log '%s' (debug, info, warn, error or keep)\n", optarg);
                return 1;
            }
            break;
        case 'h':
            print_help(argv[0]);
            return 0;
        default:
            break;
        }
    }

    if (cfg.overheat_c >= 0) {
        if (cfg.overheat_resume_c < 0) {
            cfg.overheat_resume_c = cfg.overheat_c - LC_OVERHEAT_HYST_C;
        } else if (cfg.overheat_resume_c >= cfg.overheat_c) {
            fprintf(stderr, "--overheat-resume-c %d must be below --overheat-c %d\n", cfg.overheat_resume_c,
                    cfg.overheat_c);
            return 1;
        }
    }

    if (cfg.max_power > 0 && lc_power_default(cfg.role == LC_ROLE_AP, cfg.max_power) < 0) {
        fprintf(stderr, "--max-power %d leaves no power level for this role (lowest: 25 mW)\n", cfg.max_power);
        return 1;
    }

    openlog("ar8030-lifecycled", LOG_PID, LOG_USER);

    if (cfg.overheat_c >= 0 && !cfg.soc_temp_cmd[0]) {
        lc_log("ar8030-lifecycled: --overheat-c without --soc-temp-cmd never triggers");
    }

    signal(SIGTERM, handle_sigterm);
    signal(SIGINT, handle_sigterm);
    /* A send() on a connection ar8030d has closed (it drops its clients when
     * it loses the SDIO device) must fail, not kill the process: the SDK
     * client library sends without MSG_NOSIGNAL. Seen 2026-10-18, lifecycled
     * gone mid-subscribe without a log line. */
    signal(SIGPIPE, SIG_IGN);

    lifecycle_ctx* ctx = lifecycle_init(&cfg);
    if (!ctx) {
        lc_log("ar8030-lifecycled: init failed or disabled, exiting");
        return 1;
    }

    pthread_t lc_thread;
    if (pthread_create(&lc_thread, NULL, lifecycle_thread_main, ctx) != 0) {
        lc_log("ar8030-lifecycled: failed to start the lifecycle thread, exiting");
        return 1;
    }

    /* Only when --http-port was given (non-zero) -- disabled by default,
     * see lifecycle_http.h's own header comment for why this is opt-in
     * and what it exposes. Non-fatal if it fails to start (logged) --
     * this daemon's core job (reconnect-following, tuning, hooks) doesn't
     * depend on it. */
    lifecycle_http_ctx* http_ctx = NULL;
    if (cfg.http_port != 0) {
        http_ctx = lifecycle_http_start(ctx, cfg.http_bind, cfg.http_port);
    }

    /* Only when --bind-gpio was given -- lifecycle_bind_init() returns
     * NULL otherwise (see its own header comment), and this thread (the
     * only other thing in this binary competing for the main thread) is
     * simply skipped, matching the previous single-thread behavior for
     * every board without a physical bind button. */
    lifecycle_bind_ctx* bind_ctx = lifecycle_bind_init(&cfg);
    if (bind_ctx) {
        lifecycle_bind_thread_main(bind_ctx);
    }

    pthread_join(lc_thread, NULL);
    lifecycle_http_stop(http_ctx);

    if (lifecycle_restart_requested()) {
        /* Nothing open may survive into the new image (the HTTP port would
         * stay bound): mark every inherited descriptor close-on-exec. */
        lc_log("ar8030-lifecycled: restarting");
        for (int fd = 3; fd < 1024; fd++) {
            fcntl(fd, F_SETFD, FD_CLOEXEC);
        }
        execv("/proc/self/exe", argv);
        lc_log("ar8030-lifecycled: re-exec failed: %s", strerror(errno));
        return 1;
    }
    return 0;
}
