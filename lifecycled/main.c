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
#include <getopt.h>
#include <pthread.h>
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
    printf("  --default-channel <n>     fallback channel if no tuning state yet (default: none)\n");
    printf("  --bind-gpio <n>           watch this GPIO for the physical bind button\n");
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
    OPT_BIND_GPIO,
    OPT_HTTP_PORT,
    OPT_HTTP_BIND,
    OPT_FRAME_CHANGE,
    OPT_RF_TEMP_ADC,
    OPT_RF_TEMP_FILE,
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
        .default_channel   = -1,
        .no_lifecycle      = 0,
        .bind_gpio         = -1,
        .http_bind         = "0.0.0.0",
        .http_port         = 0,
        .rf_temp_adc       = -1,
        .rf_temp_file      = "/tmp/rf_temperature.msg",
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
        {"bind-gpio",         required_argument, 0, OPT_BIND_GPIO        },
        {"http-port",         required_argument, 0, OPT_HTTP_PORT        },
        {"http-bind",         required_argument, 0, OPT_HTTP_BIND        },
        {"frame-change",      required_argument, 0, OPT_FRAME_CHANGE     },
        {"rf-temp-adc",       required_argument, 0, OPT_RF_TEMP_ADC      },
        {"rf-temp-file",      required_argument, 0, OPT_RF_TEMP_FILE     },
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
            cfg.default_channel = (int)strtoul(optarg, NULL, 10);
            break;
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
        case 'h':
            print_help(argv[0]);
            return 0;
        default:
            break;
        }
    }

    openlog("ar8030-lifecycled", LOG_PID, LOG_USER);

    signal(SIGTERM, handle_sigterm);
    signal(SIGINT, handle_sigterm);

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

    return 0;
}
