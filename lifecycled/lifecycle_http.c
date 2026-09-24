#include "lifecycle_http.h"
#include "lc_log.h"
#include "lifecycle_pair.h"
#include "lifecycle_tuning.h"
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

#define HTTP_MAX_REQ   4096
#define HTTP_MAX_QUERY 512
#define HTTP_MAX_ARGV  24

struct lifecycle_http_ctx {
    lifecycle_ctx* lc;
    int            listen_fd;
    pthread_t      thread;
    int            running; /* __atomic-accessed */
};

/* ── tiny query-string helpers ───────────────────────────────────────── */

static int hex_nibble(int c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/* Copies key's value out of a raw (still encoded) query string, decoding
 * '+' -> space and %XX along the way -- standard application/
 * x-www-form-urlencoded, same as any HTML form's GET submission would
 * produce, so `?args=20+-s+auto` and `?args=20%20-s%20auto` both work.
 * Returns 0 and fills out on a match, -1 if key isn't present. */
static int query_param(const char* query, const char* key, char* out, size_t out_sz)
{
    if (!query || !key || !out || out_sz == 0) {
        return -1;
    }
    size_t      klen = strlen(key);
    const char* q    = query;
    while (q && *q) {
        if (*q == '&') {
            q++;
            continue;
        }
        int match = strncmp(q, key, klen) == 0 && (q[klen] == '=' || q[klen] == '&' || q[klen] == '\0');
        if (match) {
            const char* v = (q[klen] == '=') ? q + klen + 1 : q + klen;
            size_t      i = 0;
            while (*v && *v != '&' && i < out_sz - 1) {
                if (*v == '+') {
                    out[i++] = ' ';
                    v++;
                } else if (*v == '%' && v[1] && v[2]) {
                    int hi = hex_nibble((unsigned char)v[1]);
                    int lo = hex_nibble((unsigned char)v[2]);
                    if (hi >= 0 && lo >= 0) {
                        out[i++] = (char)((hi << 4) | lo);
                        v += 3;
                    } else {
                        out[i++] = *v++;
                    }
                } else {
                    out[i++] = *v++;
                }
            }
            out[i] = '\0';
            return 0;
        }
        const char* amp = strchr(q, '&');
        if (!amp) {
            break;
        }
        q = amp + 1;
    }
    return -1;
}

static int query_param_int(const char* query, const char* key, int* out)
{
    char buf[32];
    if (query_param(query, key, buf, sizeof(buf)) != 0) {
        return -1;
    }
    char* end;
    long  v = strtol(buf, &end, 10);
    if (end == buf) {
        return -1;
    }
    *out = (int)v;
    return 0;
}

/* Escapes a chunk of arbitrary CLI output (ar8030-linkctl's captured
 * stdout+stderr) for embedding as one JSON string value. Control bytes
 * other than \n/\r/\t are dropped rather than \u-escaped -- this only
 * ever wraps plain ASCII command output, not user-facing text that needs
 * to round-trip exactly. */
static void json_escape(const char* in, char* out, size_t out_sz)
{
    size_t i = 0;
    for (const unsigned char* p = (const unsigned char*)in; *p && i + 2 < out_sz; p++) {
        switch (*p) {
        case '"':
            out[i++] = '\\';
            out[i++] = '"';
            break;
        case '\\':
            out[i++] = '\\';
            out[i++] = '\\';
            break;
        case '\n':
            out[i++] = '\\';
            out[i++] = 'n';
            break;
        case '\r':
            out[i++] = '\\';
            out[i++] = 'r';
            break;
        case '\t':
            out[i++] = '\\';
            out[i++] = 't';
            break;
        default:
            if (*p >= 0x20) {
                out[i++] = (char)*p;
            }
            break;
        }
    }
    out[i] = '\0';
}

/* ── response helpers ────────────────────────────────────────────────── */

static int write_all(int fd, const char* buf, size_t len)
{
    size_t total = 0;
    while (total < len) {
        ssize_t n = send(fd, buf + total, len - total, MSG_NOSIGNAL);
        if (n < 0 && errno == EINTR) {
            continue;
        }
        if (n <= 0) {
            return -1;
        }
        total += (size_t)n;
    }
    return 0;
}

static void send_response(int fd, int status, const char* status_text, const char* content_type, const char* body)
{
    size_t body_len = strlen(body);
    char   header[256];
    int    hlen = snprintf(header, sizeof(header),
                            "HTTP/1.0 %d %s\r\n"
                               "Content-Type: %s\r\n"
                               "Content-Length: %zu\r\n"
                               "Connection: close\r\n"
                               "\r\n",
                            status, status_text, content_type, body_len);
    if (hlen < 0) {
        return;
    }
    if (write_all(fd, header, (size_t)hlen) != 0) {
        return;
    }
    write_all(fd, body, body_len);
}

static void send_json(int fd, int status, const char* status_text, const char* json)
{
    send_response(fd, status, status_text, "application/json", json);
}

static void send_html(int fd, const char* html)
{
    send_response(fd, 200, "OK", "text/html; charset=utf-8", html);
}

/* ── linkctl passthrough ─────────────────────────────────────────────── */

/* Every subcommand ar8030-linkctl itself accepts (linkctl/main.c's own
 * main()) -- validated against this list before ever forking, so an
 * unrecognized cmd gets a clear 400 instead of an opaque "unknown
 * command" buried in linkctl's own captured stderr. */
static const char* const LINKCTL_COMMANDS[] = {
    "status", "bandwidth", "channel-mode", "channel", "mcs-mode", "mcs", "mcs-range", "mcs-table",
    "power-mode", "power", "freq", "force-close-socket", "force-close-all", "rf-temp", NULL,
};

static int linkctl_command_valid(const char* cmd)
{
    for (int i = 0; LINKCTL_COMMANDS[i]; i++) {
        if (strcmp(cmd, LINKCTL_COMMANDS[i]) == 0) {
            return 1;
        }
    }
    return 0;
}

/* Runs "ar8030-linkctl <argv...>" (argv NULL-terminated, no shell -- same
 * execvp-based, no-shell convention as lifecycle_pair.c's run_pair_tool())
 * and captures its combined stdout+stderr into out. Returns its exit
 * code, or -1 on a fork/pipe/wait failure. Every linkctl invocation opens
 * its own independent connection to ar8030d and exits (linkctl/main.c's
 * own header comment) -- see lifecycle_http.h's own comment on why that
 * makes this safe to call from this thread with no coordination against
 * the lifecycle thread's own ctx->client at all. */
static int run_linkctl(char* const argv[], char* out, size_t out_sz)
{
    int pipefd[2];
    if (pipe(pipefd) != 0) {
        return -1;
    }

    pid_t pid = fork();
    if (pid < 0) {
        close(pipefd[0]);
        close(pipefd[1]);
        return -1;
    }
    if (pid == 0) {
        close(pipefd[0]);
        dup2(pipefd[1], STDOUT_FILENO);
        dup2(pipefd[1], STDERR_FILENO);
        close(pipefd[1]);
        execvp("ar8030-linkctl", argv);
        _exit(127);
    }

    close(pipefd[1]);
    size_t  total = 0;
    ssize_t n;
    while (total + 1 < out_sz && (n = read(pipefd[0], out + total, out_sz - 1 - total)) > 0) {
        total += (size_t)n;
    }
    out[total] = '\0';
    close(pipefd[0]);

    int status = 0;
    waitpid(pid, &status, 0);
    if (!WIFEXITED(status)) {
        return -1;
    }
    return WEXITSTATUS(status);
}

static void handle_linkctl(int fd, const char* query)
{
    char cmd[32];
    if (query_param(query, "cmd", cmd, sizeof(cmd)) != 0 || !linkctl_command_valid(cmd)) {
        send_json(fd, 400, "Bad Request",
                  "{\"ok\":false,\"error\":\"missing or unknown 'cmd' -- see the README's HTTP control API "
                  "section for the full list\"}");
        return;
    }

    /* "args" is a single, space-separated string of whatever positional
     * value(s)/flags that one linkctl subcommand takes (see linkctl -h),
     * e.g. args=20+-d+tx+-s+auto+-w+5 for bandwidth. No shell involved --
     * this is tokenized on plain whitespace straight into argv[] below,
     * so there is no quoting support and no injection surface beyond
     * "which argv entries does ar8030-linkctl itself get", exactly as if
     * each token had been typed as a separate CLI argument. */
    char args[HTTP_MAX_QUERY];
    args[0] = '\0';
    query_param(query, "args", args, sizeof(args));

    char* argv[HTTP_MAX_ARGV];
    int   argc      = 0;
    argv[argc++]    = "ar8030-linkctl";
    argv[argc++]    = cmd;
    char* saveptr   = NULL;
    char* tok       = strtok_r(args, " \t", &saveptr);
    while (tok && argc < HTTP_MAX_ARGV - 1) {
        argv[argc++] = tok;
        tok          = strtok_r(NULL, " \t", &saveptr);
    }
    argv[argc] = NULL;

    char out[2048];
    int  rc = run_linkctl(argv, out, sizeof(out));

    char escaped[4096];
    json_escape(out, escaped, sizeof(escaped));

    char json[4200];
    snprintf(json, sizeof(json), "{\"ok\":%s,\"exit_code\":%d,\"output\":\"%s\"}", rc == 0 ? "true" : "false", rc,
              escaped);
    send_json(fd, rc == 0 ? 200 : 502, rc == 0 ? "OK" : "Bad Gateway", json);
}

/* ── lifecycle-owned endpoints ───────────────────────────────────────── */

/* Besides this daemon's own lightweight state (role/state/connected_slot/
 * bandwidth_mhz/paired -- all cheap, always available even if the
 * ar8030-linkctl invocation below fails for some reason), also runs
 * `ar8030-linkctl status` and embeds its full text dump (peer MAC,
 * per-user tx/rx mcs/bandwidth/freq, SNR/gain, ranging, power, per-port
 * byte counters -- see linkctl/main.c's own cmd_status()) as
 * "linkctl_status", so a single GET here gives the same detail `curl`
 * would get running the CLI directly, not just the compact summary. Safe
 * to run on every poll (the web page's own refresh() calls this every
 * 2s) -- see run_linkctl()'s own comment on why a linkctl invocation
 * never touches this daemon's own ctx->client. */
/* JSON number with one decimal ("47.5"), or "null" without a reading. */
static void format_rf_temp(const lc_status_t* st, char* out, size_t out_sz)
{
    if (!st->rf_temp_valid) {
        snprintf(out, out_sz, "null");
        return;
    }
    snprintf(out, out_sz, "%d.%d", st->rf_temp_c10 / 10, st->rf_temp_c10 % 10);
}

/* JSON value for a wanted channel: "auto", null (LC_CHANNEL_NONE) or the
 * index. */
static void format_channel(int chan, char* out, size_t out_sz)
{
    if (chan == LC_CHANNEL_AUTO) {
        snprintf(out, out_sz, "\"auto\"");
    } else if (chan == LC_CHANNEL_NONE) {
        snprintf(out, out_sz, "null");
    } else {
        snprintf(out, out_sz, "%d", chan);
    }
}

/* "channel" (wanted/persisted), "chan_mode"/"work_chan" (what the chip
 * last reported on a live link, null otherwise). */
static void format_channel_fields(const lc_status_t* st, char* out, size_t out_sz)
{
    char chan[16], work[16];
    format_channel(st->channel, chan, sizeof(chan));
    if (st->work_chan >= 0) {
        snprintf(work, sizeof(work), "%d", st->work_chan);
    } else {
        snprintf(work, sizeof(work), "null");
    }
    snprintf(out, out_sz, "\"channel\":%s,\"chan_mode\":%s,\"work_chan\":%s", chan,
             st->chan_auto < 0 ? "null" : st->chan_auto ? "\"auto\"" : "\"manual\"", work);
}

/* "power" (wanted: mW, "auto" or null for LC_POWER_NONE), "power_dbm"
 * (chip's dBm target, null until read) and "power_levels" (this role's
 * choices, highest first). */
static void format_power_fields(const lifecycle_http_ctx* http, const lc_status_t* st, char* out, size_t out_sz)
{
    char level[16], dbm[16], levels[96];
    if (st->power == LC_POWER_AUTO) {
        snprintf(level, sizeof(level), "\"auto\"");
    } else if (st->power == LC_POWER_NONE) {
        snprintf(level, sizeof(level), "null");
    } else {
        snprintf(level, sizeof(level), "%d", st->power);
    }
    if (st->power_dbm >= 0) {
        snprintf(dbm, sizeof(dbm), "%d", st->power_dbm);
    } else {
        snprintf(dbm, sizeof(dbm), "null");
    }
    lc_power_levels_json(lifecycle_get_config(http->lc)->role == LC_ROLE_AP, levels, sizeof(levels));
    snprintf(out, out_sz, "\"power\":%s,\"power_dbm\":%s,\"power_levels\":[%s]", level, dbm, levels);
}

static void handle_status(lifecycle_http_ctx* http, int fd)
{
    lc_status_t st;
    lifecycle_get_status(http->lc, &st);

    const char* role_str  = st.role == LC_ROLE_AP ? "ap" : "dev";
    const char* state_str = st.state == LC_STATE_CONNECTED ? "connected" : st.state == LC_STATE_IDLE ? "idle" : "init";

    char* argv[] = {"ar8030-linkctl", "status", NULL};
    char  out[4096];
    int   rc = run_linkctl(argv, out, sizeof(out));

    char escaped[8192];
    json_escape(out, escaped, sizeof(escaped));

    char rf_temp[48];
    format_rf_temp(&st, rf_temp, sizeof(rf_temp));

    char chan[96];
    format_channel_fields(&st, chan, sizeof(chan));
    char power[192];
    format_power_fields(http, &st, power, sizeof(power));

    char json[8900];
    snprintf(json, sizeof(json),
             "{\"ok\":true,\"role\":\"%s\",\"state\":\"%s\",\"connected_slot\":%d,\"bandwidth_mhz\":%d,%s,%s,"
              "\"paired\":%s,\"rf_temp_c\":%s,\"linkctl_exit_code\":%d,\"linkctl_status\":\"%s\"}",
             role_str, state_str, st.connected_slot, st.bandwidth_mhz, chan, power, st.paired ? "true" : "false",
             rf_temp, rc, escaped);
    send_json(fd, 200, "OK", json);
}

/* GET /api/v1/rf-temp -- just the RF-board temperature (lifecycle.c's
 * lc_poll_rf_temp(), enabled via --rf-temp-adc). Cheap, unlike
 * /api/v1/status, which forks ar8030-linkctl on every call: meant for
 * frequent polling (OSD, dashboards). rf_temp_c is null until the first
 * reading arrives, or always when polling is disabled. */
static void handle_rf_temp(lifecycle_http_ctx* http, int fd)
{
    lc_status_t st;
    lifecycle_get_status(http->lc, &st);
    const lc_config_t* cfg = lifecycle_get_config(http->lc);

    char rf_temp[48];
    format_rf_temp(&st, rf_temp, sizeof(rf_temp));

    char json[160];
    snprintf(json, sizeof(json), "{\"ok\":true,\"enabled\":%s,\"adc_channel\":%d,\"rf_temp_c\":%s,\"adc_mv\":%d}",
             cfg->rf_temp_adc >= 0 ? "true" : "false", cfg->rf_temp_adc, rf_temp, st.rf_temp_mv);
    send_json(fd, 200, "OK", json);
}

static void handle_pair(lifecycle_http_ctx* http, int fd)
{
    const lc_config_t* cfg = lifecycle_get_config(http->lc);
    int                 slot = -1;
    int                 rc   = lc_pair_run(cfg, &slot, NULL);

    char json[128];
    if (rc == 0) {
        snprintf(json, sizeof(json), "{\"ok\":true,\"slot\":%d}", slot);
        send_json(fd, 200, "OK", json);
    } else {
        send_json(fd, 502, "Bad Gateway", "{\"ok\":false,\"error\":\"pairing failed, see syslog for ar8030-pair's own output\"}");
    }
}

static void handle_bandwidth(lifecycle_http_ctx* http, int fd, const char* query)
{
    int mhz;
    if (query_param_int(query, "mhz", &mhz) != 0) {
        send_json(fd, 400, "Bad Request", "{\"ok\":false,\"error\":\"missing 'mhz' query parameter\"}");
        return;
    }
    if (lifecycle_request_bandwidth(http->lc, mhz) != 0) {
        send_json(fd, 400, "Bad Request",
                  "{\"ok\":false,\"error\":\"invalid bandwidth, must be one of 1/2/5/10/20/40\"}");
        return;
    }
    char json[96];
    snprintf(json, sizeof(json), "{\"ok\":true,\"queued_mhz\":%d}", mhz);
    send_json(fd, 202, "Accepted", json);
}

/* GET /api/v1/channel -- wanted vs. chip-reported channel, cheap (no
 * linkctl fork). POST /api/v1/channel?chan=<index|auto> -- the persisted
 * way to change it, queued like /api/v1/bandwidth (see
 * lifecycle_request_channel() for the DEV-while-idle refusal). */
static void handle_channel(lifecycle_http_ctx* http, int fd, const char* method, const char* query)
{
    if (strcmp(method, "POST") == 0) {
        char arg[16];
        int  chan;
        if (query_param(query, "chan", arg, sizeof(arg)) != 0 || lc_channel_parse(arg, &chan) != 0 ||
            chan == LC_CHANNEL_NONE) {
            send_json(fd, 400, "Bad Request",
                      "{\"ok\":false,\"error\":\"'chan' must be a channel index or 'auto'\"}");
            return;
        }
        int rc = lifecycle_request_channel(http->lc, chan);
        if (rc == -2) {
            send_json(fd, 409, "Conflict",
                      "{\"ok\":false,\"error\":\"no link: on the dev side a channel change needs a connected "
                      "peer to retune with (the ap owns the channel on reconnect)\"}");
            return;
        }
        if (rc != 0) {
            send_json(fd, 400, "Bad Request", "{\"ok\":false,\"error\":\"invalid channel\"}");
            return;
        }
        char queued[16], json[64];
        format_channel(chan, queued, sizeof(queued));
        snprintf(json, sizeof(json), "{\"ok\":true,\"queued\":%s}", queued);
        send_json(fd, 202, "Accepted", json);
        return;
    }

    lc_status_t st;
    lifecycle_get_status(http->lc, &st);
    char fields[96];
    format_channel_fields(&st, fields, sizeof(fields));
    /* table_mhz: the chip's channel table, index -> MHz, for pickers */
    char   table[LC_MAX_CHANNELS * 6 + 1];
    size_t len = 0;
    table[0]   = '\0';
    for (int i = 0; i < st.chan_table_n && len < sizeof(table); i++) {
        len += (size_t)snprintf(table + len, sizeof(table) - len, "%s%u", i ? "," : "",
                                (unsigned)(st.chan_table_khz[i] / 1000));
    }
    char json[sizeof(fields) + sizeof(table) + 64];
    snprintf(json, sizeof(json), "{\"ok\":true,%s,\"table_mhz\":[%s]}", fields, table);
    send_json(fd, 200, "OK", json);
}

/* GET /api/v1/power -- wanted level, chip dBm target and this role's
 * levels (cheap, no linkctl fork). POST /api/v1/power?level=<mW|auto> --
 * persisted and applied on the lifecycle thread's next tick, like
 * /api/v1/bandwidth. */
static void handle_power(lifecycle_http_ctx* http, int fd, const char* method, const char* query)
{
    if (strcmp(method, "POST") == 0) {
        char arg[16];
        int  level;
        if (query_param(query, "level", arg, sizeof(arg)) != 0 || lc_power_parse(arg, &level) != 0 ||
            lifecycle_request_power(http->lc, level) != 0) {
            char levels[96], json[200];
            lc_power_levels_json(lifecycle_get_config(http->lc)->role == LC_ROLE_AP, levels, sizeof(levels));
            snprintf(json, sizeof(json), "{\"ok\":false,\"error\":\"'level' must be one of\",\"power_levels\":[%s]}",
                     levels);
            send_json(fd, 400, "Bad Request", json);
            return;
        }
        char json[64];
        if (level == LC_POWER_AUTO) {
            snprintf(json, sizeof(json), "{\"ok\":true,\"queued\":\"auto\"}");
        } else {
            snprintf(json, sizeof(json), "{\"ok\":true,\"queued\":%d}", level);
        }
        send_json(fd, 202, "Accepted", json);
        return;
    }

    lc_status_t st;
    lifecycle_get_status(http->lc, &st);
    char fields[192], json[220];
    format_power_fields(http, &st, fields, sizeof(fields));
    snprintf(json, sizeof(json), "{\"ok\":true,%s}", fields);
    send_json(fd, 200, "OK", json);
}

/* POST /api/v1/retx-tuning?win=&busy=&idle=&conti_busy=&conti_idle= --
 * same mailbox pattern as handle_bandwidth() above, for the windowed
 * retransmission controller's own tuning parameters (see
 * lifecycle_tuning.h's own doc comment on lc_retx_apply() for what
 * these mean and what's still unverified about them: units are
 * unconfirmed, only the range 0-255 is enforced here). All 5 query
 * parameters are required -- there's no "change just one, leave the
 * rest as last-applied" partial-update support, since this process
 * doesn't cache the chip's own currently-applied values anywhere it
 * could fill in the gaps from (unlike bandwidth, which reads back via
 * BB_GET_STATUS elsewhere); a caller wanting to tweak one field should
 * read /api/v1/status's own linkctl_status text (ar8030-linkctl retx)
 * first to get the current 5 values before posting all 5 back. */
static void handle_retx_tuning(lifecycle_http_ctx* http, int fd, const char* query)
{
    int win, busy, idle, conti_busy, conti_idle;
    if (query_param_int(query, "win", &win) != 0 || query_param_int(query, "busy", &busy) != 0 ||
        query_param_int(query, "idle", &idle) != 0 || query_param_int(query, "conti_busy", &conti_busy) != 0 ||
        query_param_int(query, "conti_idle", &conti_idle) != 0) {
        send_json(fd, 400, "Bad Request",
                  "{\"ok\":false,\"error\":\"missing one of win/busy/idle/conti_busy/conti_idle query "
                  "parameters (all 5 required)\"}");
        return;
    }
    if (lifecycle_request_retx(http->lc, win, busy, idle, conti_busy, conti_idle) != 0) {
        send_json(fd, 400, "Bad Request", "{\"ok\":false,\"error\":\"all 5 values must be in range 0-255\"}");
        return;
    }
    char json[160];
    snprintf(json, sizeof(json),
             "{\"ok\":true,\"queued\":{\"win\":%d,\"busy\":%d,\"idle\":%d,\"conti_busy\":%d,\"conti_idle\":%d}}",
             win, busy, idle, conti_busy, conti_idle);
    send_json(fd, 202, "Accepted", json);
}

/* ── control panel (static, no server-side templating) ───────────────── */

static const char INDEX_HTML[] =
    "<!doctype html>\n"
    "<html><head><meta charset='utf-8'>\n"
    "<meta name='viewport' content='width=device-width, initial-scale=1'>\n"
    "<title>ar8030-lifecycled</title>\n"
    "<style>\n"
    "body{font-family:sans-serif;max-width:720px;margin:2em auto;padding:0 1em;background:#111;color:#eee}\n"
    "h1{font-size:1.2em}h2{font-size:1em;margin-top:1.6em;color:#9ab;border-bottom:1px solid #333;"
    "padding-bottom:0.2em}\n"
    ".row{margin:0.8em 0}\n"
    ".ctl{display:flex;flex-wrap:wrap;gap:0.4em;align-items:center;margin:0.4em 0}\n"
    ".ctl span{min-width:11em;color:#aac}\n"
    "button,select,input{font-size:0.95em;padding:0.35em 0.5em;background:#222;color:#eee;border:1px solid #444;"
    "border-radius:4px}\n"
    "button{cursor:pointer}button:hover{background:#333}\n"
    "input.n{width:4.5em}input.s{width:4.5em}\n"
    "#status,#lc_out{white-space:pre-wrap;background:#1a1a1a;padding:0.8em;border-radius:4px;"
    "font-family:monospace;font-size:0.85em;max-height:20em;overflow:auto}\n"
    ".ok{color:#4caf50}.bad{color:#f44336}\n"
    ".temp{font-size:1.4em;font-weight:bold}.warm{color:#ffb300}.hot{color:#f44336}\n"
    "</style></head>\n"
    "<body>\n"
    "<h1>ar8030-lifecycled</h1>\n"
    "<div class='row'>RF board: <span id='rftemp' class='temp'>--</span></div>\n"
    "<div class='row' id='status'>loading...</div>\n"
    "\n"
    "<h2>Pairing</h2>\n"
    "<div class='row'><button onclick='doPair()'>Pair now</button></div>\n"
    "\n"
    "<h2>Bandwidth (persisted, survives reconnects)</h2>\n"
    "<div class='row'>\n"
    "<select id='bw'>\n"
    "<option value='1'>1.25 MHz</option><option value='2'>2.5 MHz</option><option value='5'>5 MHz</option>\n"
    "<option value='10'>10 MHz</option><option value='20' selected>20 MHz</option><option value='40'>40 MHz</option>\n"
    "</select>\n"
    "<button onclick='setBandwidth()'>Apply</button>\n"
    "</div>\n"
    "\n"
    "<h2>Channel (persisted, survives reboots)</h2>\n"
    "<div class='row'>\n"
    "<input id='chan' class='n' value='32' title='channel-table index, or auto'>\n"
    "<button onclick='setChannel()'>Apply</button>\n"
    "<button onclick=\"document.getElementById('chan').value='auto';setChannel()\">Auto</button>\n"
    "</div>\n"
    "\n"
    "<h2>Output power (persisted, survives reboots)</h2>\n"
    "<div class='row'>\n"
    "<select id='pwr'></select>\n"
    "<button onclick='setPower()'>Apply</button>\n"
    "<span id='pwr_cur'></span>\n"
    "</div>\n"
    "\n"
    "<h2>Advanced (ar8030-linkctl, one-shot -- not persisted)</h2>\n"
    "\n"
    "<div class='ctl'><span>Channel mode</span>\n"
    "<select id='cm_mode'><option>auto</option><option>manual</option></select>\n"
    "<button onclick='ctlChannelMode()'>Set</button></div>\n"
    "\n"
    "<div class='ctl'><span>Channel</span>\n"
    "index <input id='ch_index' class='n' type='number' value='0'>\n"
    "slot <input id='ch_slot' class='s' value='auto'>\n"
    "wait(s) <input id='ch_wait' class='n' type='number' value='5'>\n"
    "<button onclick='ctlChannel()'>Set channel</button></div>\n"
    "\n"
    "<div class='ctl'><span>MCS mode</span>\n"
    "<select id='mm_mode'><option>auto</option><option>manual</option></select>\n"
    "slot <input id='mm_slot' class='s' type='number' value='0'>\n"
    "<button onclick='ctlMcsMode()'>Set</button></div>\n"
    "\n"
    "<div class='ctl'><span>MCS</span>\n"
    "value <input id='mc_value' class='n' type='number' value='12'>\n"
    "slot <input id='mc_slot' class='s' type='number' value='0'>\n"
    "wait(s) <input id='mc_wait' class='n' type='number' value='5'>\n"
    "<button onclick='ctlMcs()'>Set MCS</button></div>\n"
    "\n"
    "<div class='ctl'><span>MCS range</span>\n"
    "min <input id='mr_min' class='n' value='0'>\n"
    "max <input id='mr_max' class='n' value='max'>\n"
    "slot <input id='mr_slot' class='s' type='number' value='0'>\n"
    "<button onclick='ctlMcsRange()'>Set range</button></div>\n"
    "\n"
    "<div class='ctl'><span>MCS table</span>\n"
    "<select id='mt_variant'><option value='0'>0</option><option value='1'>1</option><option value='2'>2</option>"
    "</select>\n"
    "only mcs <input id='mt_mcs' class='n' type='number' placeholder='all'>\n"
    "<button onclick='ctlMcsTable()'>Push table</button></div>\n"
    "\n"
    "<div class='ctl'><span>Power mode</span>\n"
    "<select id='pm_mode'><option value=''>(read)</option><option value='auto'>auto</option>"
    "<option value='manual'>manual</option></select>\n"
    "<button onclick='ctlPowerMode()'>Go</button></div>\n"
    "\n"
    "<div class='ctl'><span>TX power</span>\n"
    "dBm(0-31) <input id='pw_dbm' class='n' type='number' min='0' max='31' value='15'>\n"
    "user <input id='pw_user' class='s' type='number' value='0'>\n"
    "wait(s) <input id='pw_wait' class='n' type='number' value='0'>\n"
    "<button onclick='ctlPower()'>Set power</button></div>\n"
    "\n"
    "<div class='ctl'><span>Frequency</span>\n"
    "kHz <input id='fr_khz' type='number' style='width:7em' value='2100000'>\n"
    "user <input id='fr_user' class='s' type='number' value='0'>\n"
    "dir <select id='fr_dir'><option>tx</option><option>rx</option><option>both</option></select>\n"
    "slot <input id='fr_slot' class='s' type='number' value='0'>\n"
    "wait(s) <input id='fr_wait' class='n' type='number' value='0'>\n"
    "<button onclick='ctlFreq()'>Set freq</button></div>\n"
    "\n"
    "<div class='ctl'><span>Force-close socket</span>\n"
    "slot <input id='fc_slot' class='s' type='number' value='0'>\n"
    "port <input id='fc_port' class='s' type='number' value='0'>\n"
    "<button onclick='ctlForceCloseSocket()'>Close</button></div>\n"
    "\n"
    "<div class='ctl'><span>Force-close ALL sockets</span>\n"
    "<button onclick='ctlForceCloseAll()'>Close all</button></div>\n"
    "\n"
    "<div class='ctl'><span>Bandwidth (one-shot)</span>\n"
    "<select id='bwo_mhz'><option value='1'>1.25</option><option value='2'>2.5</option><option value='5'>5</option>"
    "<option value='10'>10</option><option value='20' selected>20</option><option value='40'>40</option></select>\n"
    "dir <select id='bwo_dir'><option>tx</option><option>rx</option></select>\n"
    "slot <input id='bwo_slot' class='s' value='auto'>\n"
    "wait(s) <input id='bwo_wait' class='n' type='number' value='5'>\n"
    "<button onclick='ctlBandwidthOnce()'>Apply once</button></div>\n"
    "\n"
    "<div class='ctl'><span>RF-board temp (raw ADC)</span>\n"
    "channel <input id='rt_ch' class='n' value='4'>\n"
    "<button onclick='ctlRfTemp()'>Read</button></div>\n"
    "\n"
    "<h2>Command output</h2>\n"
    "<div class='row' id='lc_out'>(nothing run yet)</div>\n"
    "\n"
    "<div class='row' id='msg'></div>\n"
    "<script>\n"
    "function v(id){ return document.getElementById(id).value; }\n"
    "function refresh(){\n"
    "  fetch('/api/v1/status').then(r=>r.json()).then(d=>{\n"
    "    var summary = 'role='+d.role+' state='+d.state+' connected_slot='+d.connected_slot+\n"
    "      ' bandwidth_mhz='+d.bandwidth_mhz+' channel='+d.channel+' (chip: '+d.chan_mode+' '+d.work_chan+')'+\n"
    "      ' paired='+d.paired+' (linkctl exit '+d.linkctl_exit_code+')';\n"
    "    document.getElementById('status').textContent = summary+'\\n\\n'+(d.linkctl_status||'');\n"
    "    var t = document.getElementById('rftemp');\n"
    "    if (d.rf_temp_c === null || d.rf_temp_c === undefined) { t.textContent = 'n/a'; t.className = 'temp'; }\n"
    "    else { t.textContent = d.rf_temp_c.toFixed(1)+' \\u00b0C';\n"
    "      t.className = 'temp'+(d.rf_temp_c >= 90 ? ' hot' : d.rf_temp_c >= 70 ? ' warm' : ''); }\n"
    "  }).catch(e=>{ document.getElementById('status').textContent = 'unreachable: '+e; });\n"
    "}\n"
    "function msg(text, ok){\n"
    "  var el = document.getElementById('msg');\n"
    "  el.textContent = text;\n"
    "  el.className = ok===false ? 'bad' : ok===true ? 'ok' : '';\n"
    "}\n"
    "function doPair(){\n"
    "  msg('pairing...');\n"
    "  fetch('/api/v1/pair', {method:'POST'}).then(r=>r.json()).then(d=>{\n"
    "    msg(d.ok ? 'paired (slot '+d.slot+')' : 'pair failed: '+d.error, d.ok);\n"
    "    refresh();\n"
    "  }).catch(e=>msg('error: '+e, false));\n"
    "}\n"
    "function setBandwidth(){\n"
    "  var mhz = v('bw');\n"
    "  fetch('/api/v1/bandwidth?mhz='+encodeURIComponent(mhz), {method:'POST'}).then(r=>r.json()).then(d=>{\n"
    "    msg(d.ok ? 'bandwidth queued: '+d.queued_mhz+' MHz' : 'failed: '+d.error, d.ok);\n"
    "  }).catch(e=>msg('error: '+e, false));\n"
    "}\n"
    "function loadPower(){\n"
    "  fetch('/api/v1/power').then(r=>r.json()).then(d=>{\n"
    "    var s = document.getElementById('pwr'); s.innerHTML = '';\n"
    "    d.power_levels.forEach(function(l){\n"
    "      var o = document.createElement('option'); o.value = l;\n"
    "      o.textContent = l === 'auto' ? 'Auto' : l+' mW'; if (l === d.power) o.selected = true;\n"
    "      s.appendChild(o);\n"
    "    });\n"
    "    document.getElementById('pwr_cur').textContent = d.power_dbm === null ? '' : 'chip target '+d.power_dbm+' dBm';\n"
    "  });\n"
    "}\n"
    "function setPower(){\n"
    "  fetch('/api/v1/power?level='+encodeURIComponent(v('pwr')), {method:'POST'}).then(r=>r.json()).then(d=>{\n"
    "    msg(d.ok ? 'power queued: '+d.queued : 'failed: '+d.error, d.ok);\n"
    "    setTimeout(loadPower, 1500);\n"
    "  }).catch(e=>msg('error: '+e, false));\n"
    "}\n"
    "function setChannel(){\n"
    "  var ch = v('chan');\n"
    "  fetch('/api/v1/channel?chan='+encodeURIComponent(ch), {method:'POST'}).then(r=>r.json()).then(d=>{\n"
    "    msg(d.ok ? 'channel queued: '+d.queued : 'failed: '+d.error, d.ok);\n"
    "  }).catch(e=>msg('error: '+e, false));\n"
    "}\n"
    "function runLinkctl(cmd, args){\n"
    "  var out = document.getElementById('lc_out');\n"
    "  var shown = cmd + (args ? ' ' + args : '');\n"
    "  out.textContent = '$ '+shown+'\\nrunning...';\n"
    "  var url = '/api/v1/linkctl?cmd='+encodeURIComponent(cmd)+'&args='+encodeURIComponent(args||'');\n"
    "  fetch(url, {method:'POST'}).then(r=>r.json()).then(d=>{\n"
    "    out.textContent = '$ '+shown+'  (exit '+d.exit_code+')\\n'+d.output;\n"
    "    refresh();\n"
    "  }).catch(e=>{ out.textContent = 'error: '+e; });\n"
    "}\n"
    "function ctlChannelMode(){ runLinkctl('channel-mode', v('cm_mode')); }\n"
    "function ctlChannel(){ runLinkctl('channel', v('ch_index')+' -s '+v('ch_slot')+' -w '+v('ch_wait')); }\n"
    "function ctlMcsMode(){ runLinkctl('mcs-mode', v('mm_mode')+' -s '+v('mm_slot')); }\n"
    "function ctlMcs(){ runLinkctl('mcs', v('mc_value')+' -s '+v('mc_slot')+' -w '+v('mc_wait')); }\n"
    "function ctlMcsRange(){ runLinkctl('mcs-range', v('mr_min')+' '+v('mr_max')+' -s '+v('mr_slot')); }\n"
    "function ctlMcsTable(){\n"
    "  var mcs = v('mt_mcs');\n"
    "  runLinkctl('mcs-table', v('mt_variant')+(mcs ? ' '+mcs : ''));\n"
    "}\n"
    "function ctlRfTemp(){ runLinkctl('rf-temp', '-c '+v('rt_ch')); }\n"
    "function ctlPowerMode(){ runLinkctl('power-mode', v('pm_mode')); }\n"
    "function ctlPower(){ runLinkctl('power', v('pw_dbm')+' -u '+v('pw_user')+' -w '+v('pw_wait')); }\n"
    "function ctlFreq(){\n"
    "  runLinkctl('freq', v('fr_khz')+' -u '+v('fr_user')+' -d '+v('fr_dir')+' -s '+v('fr_slot')+' -w '+v('fr_wait'));\n"
    "}\n"
    "function ctlForceCloseSocket(){\n"
    "  if(!confirm('Force-close one socket may desync the daemon internal socket accounting. Continue?')) return;\n"
    "  runLinkctl('force-close-socket', v('fc_slot')+' '+v('fc_port'));\n"
    "}\n"
    "function ctlForceCloseAll(){\n"
    "  if(!confirm('Force-close ALL sockets on every slot. Continue?')) return;\n"
    "  runLinkctl('force-close-all', '');\n"
    "}\n"
    "function ctlBandwidthOnce(){\n"
    "  runLinkctl('bandwidth', v('bwo_mhz')+' -d '+v('bwo_dir')+' -s '+v('bwo_slot')+' -w '+v('bwo_wait'));\n"
    "}\n"
    "refresh();\n"
    "loadPower();\n"
    "setInterval(refresh, 2000);\n"
    "</script>\n"
    "</body></html>\n";

/* ── request parsing + dispatch ──────────────────────────────────────── */

/* Reads and parses only the request line ("METHOD /path?query HTTP/1.x")
 * -- headers and body are read into the same buffer (so a client isn't
 * left hanging with unread bytes in flight) but never inspected: none of
 * this module's endpoints need a body or a header, everything comes in
 * as query parameters (see lifecycle_http.h's own header comment). */
static int read_request_line(int fd, char* method, size_t method_sz, char* path, size_t path_sz, char* query,
                              size_t query_sz)
{
    char buf[HTTP_MAX_REQ];
    int  total = 0;
    while (total < (int)sizeof(buf) - 1) {
        ssize_t n = read(fd, buf + total, sizeof(buf) - 1 - (size_t)total);
        if (n < 0 && errno == EINTR) {
            continue;
        }
        if (n <= 0) {
            break;
        }
        total += (int)n;
        if (memchr(buf, '\n', (size_t)total)) {
            break;
        }
    }
    if (total <= 0) {
        return -1;
    }
    buf[total] = '\0';

    char* line_end = strpbrk(buf, "\r\n");
    if (line_end) {
        *line_end = '\0';
    }

    char* sp1 = strchr(buf, ' ');
    if (!sp1) {
        return -1;
    }
    *sp1 = '\0';
    size_t mlen = (size_t)(sp1 - buf);
    if (mlen >= method_sz) {
        mlen = method_sz - 1;
    }
    memcpy(method, buf, mlen);
    method[mlen] = '\0';

    char* uri = sp1 + 1;
    char* sp2 = strchr(uri, ' ');
    if (sp2) {
        *sp2 = '\0';
    }

    char* qmark = strchr(uri, '?');
    if (qmark) {
        *qmark = '\0';
        snprintf(query, query_sz, "%s", qmark + 1);
    } else {
        query[0] = '\0';
    }
    snprintf(path, path_sz, "%s", uri);
    return 0;
}

static void dispatch(lifecycle_http_ctx* http, int fd, const char* method, const char* path, const char* query)
{
    if (strcmp(path, "/") == 0) {
        send_html(fd, INDEX_HTML);
    } else if (strcmp(path, "/api/v1/status") == 0) {
        handle_status(http, fd);
    } else if (strcmp(path, "/api/v1/pair") == 0 && strcmp(method, "POST") == 0) {
        handle_pair(http, fd);
    } else if (strcmp(path, "/api/v1/bandwidth") == 0 && strcmp(method, "POST") == 0) {
        handle_bandwidth(http, fd, query);
    } else if (strcmp(path, "/api/v1/retx-tuning") == 0 && strcmp(method, "POST") == 0) {
        handle_retx_tuning(http, fd, query);
    } else if (strcmp(path, "/api/v1/power") == 0) {
        handle_power(http, fd, method, query);
    } else if (strcmp(path, "/api/v1/channel") == 0) {
        handle_channel(http, fd, method, query);
    } else if (strcmp(path, "/api/v1/rf-temp") == 0) {
        handle_rf_temp(http, fd);
    } else if (strcmp(path, "/api/v1/linkctl") == 0) {
        handle_linkctl(fd, query);
    } else {
        send_json(fd, 404, "Not Found", "{\"ok\":false,\"error\":\"no such endpoint\"}");
    }
}

/* ── server thread ───────────────────────────────────────────────────── */

static void* http_thread_main(void* arg)
{
    lifecycle_http_ctx* http = (lifecycle_http_ctx*)arg;

    while (__atomic_load_n(&http->running, __ATOMIC_ACQUIRE)) {
        struct sockaddr_in peer;
        socklen_t          peer_len = sizeof(peer);
        int                fd       = accept(http->listen_fd, (struct sockaddr*)&peer, &peer_len);
        if (fd < 0) {
            if (__atomic_load_n(&http->running, __ATOMIC_ACQUIRE) && errno != EINTR) {
                lc_log("lifecycle: http: accept error: %s", strerror(errno));
            }
            continue;
        }

        struct timeval tv = {.tv_sec = 2, .tv_usec = 0};
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        char method[8] = {0}, path[256] = {0}, query[HTTP_MAX_QUERY] = {0};
        if (read_request_line(fd, method, sizeof(method), path, sizeof(path), query, sizeof(query)) == 0) {
            dispatch(http, fd, method, path, query);
        }
        close(fd);
    }
    return NULL;
}

lifecycle_http_ctx* lifecycle_http_start(lifecycle_ctx* lc, const char* bind_addr, uint16_t port)
{
    lifecycle_http_ctx* http = (lifecycle_http_ctx*)calloc(1, sizeof(*http));
    if (!http) {
        return NULL;
    }
    http->lc = lc;

    http->listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (http->listen_fd < 0) {
        lc_log("lifecycle: http: socket() failed: %s", strerror(errno));
        free(http);
        return NULL;
    }
    int opt = 1;
    setsockopt(http->listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(port);
    if (!bind_addr || !bind_addr[0] || strcmp(bind_addr, "0.0.0.0") == 0) {
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
    } else if (inet_pton(AF_INET, bind_addr, &addr.sin_addr) != 1) {
        lc_log("lifecycle: http: invalid --http-bind address '%s'", bind_addr);
        close(http->listen_fd);
        free(http);
        return NULL;
    }

    if (bind(http->listen_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        lc_log("lifecycle: http: bind %s:%u failed: %s", bind_addr && bind_addr[0] ? bind_addr : "0.0.0.0", port,
               strerror(errno));
        close(http->listen_fd);
        free(http);
        return NULL;
    }
    if (listen(http->listen_fd, 4) < 0) {
        lc_log("lifecycle: http: listen failed: %s", strerror(errno));
        close(http->listen_fd);
        free(http);
        return NULL;
    }

    http->running = 1;
    if (pthread_create(&http->thread, NULL, http_thread_main, http) != 0) {
        lc_log("lifecycle: http: pthread_create failed: %s", strerror(errno));
        close(http->listen_fd);
        free(http);
        return NULL;
    }

    lc_log("lifecycle: http: control API listening on %s:%u", bind_addr && bind_addr[0] ? bind_addr : "0.0.0.0",
           port);
    return http;
}

void lifecycle_http_stop(lifecycle_http_ctx* http)
{
    if (!http) {
        return;
    }
    __atomic_store_n(&http->running, 0, __ATOMIC_RELEASE);
    shutdown(http->listen_fd, SHUT_RDWR);
    close(http->listen_fd);
    /* Detach, don't join -- see venc_httpd.c's own precedent for this
     * exact tradeoff (waybeam_venc/src/venc_httpd.c): closing the listen
     * fd doesn't reliably unblock accept() on every embedded kernel this
     * project targets. Deliberately never freed either -- this is called
     * exactly once, right before main() returns and the whole process
     * exits, matching main.c's own convention of never freeing
     * lifecycle_ctx either; freeing it here would race a detached thread
     * that might still be inside dispatch() when this runs. */
    pthread_detach(http->thread);
}
