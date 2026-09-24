#include "lifecycle_pair.h"
#include "lc_log.h"
#include "lifecycle_hooks.h"
#include "lifecycle_tuning.h"
#include <cjson/cJSON.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

/*
 * Ported from dev_helper/bb_pair/txg_bb_pair.cpp's persist_paired_peer()
 * -- the on-disk JSON logic is unchanged; the connection-lifecycle
 * wrapper around it is adapted since the daemon holds one persistent
 * handle (lc_client_t) instead of the standalone CLI tool opening a
 * fresh one per invocation.
 *
 * The actual pairing handshake (PRJ_CMD_EVENT_PAIR dispatch,
 * BB_SET_PRJ_DISPATCH) is NOT ported here -- this module only ever
 * persists/reads/re-applies a result, it never triggers one. That stays
 * the SDK's own ar8030-pair binary's job (dev_helper/bb_pair,
 * package/ar8030), invoked by lifecycle_bind.c on a physical button
 * press. An earlier version of this file did port lc_pair_trigger()/
 * lc_pair_wait_for_peer() (a from-scratch reimplementation of that same
 * dispatch) for a since-removed automatic background re-pair path --
 * see lifecycle.c's own header comment for why that path was removed
 * (Ghidra evidence that stock firmware never auto-repairs either); those
 * two functions had no remaining caller once it was, so they were
 * dropped rather than carried along as dead code.
 */

int lc_pair_persist(bb_dev_handle_t* handle, const char* cfg_path, int slot, uint8_t role, const bb_mac_t* mac)
{
    char mac_hex[2 * BB_MAC_LEN + 1];
    for (int i = 0; i < BB_MAC_LEN; i++) {
        sprintf(mac_hex + i * 2, "%02x", mac->addr[i]);
    }

    FILE* f = fopen(cfg_path, "rb");
    if (!f) {
        lc_log("lifecycle: persist: can't open %s for reading: %s", cfg_path, strerror(errno));
        return -1;
    }
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);
    char* buf = (char*)malloc(len + 1);
    if (!buf || fread(buf, 1, len, f) != (size_t)len) {
        lc_log("lifecycle: persist: read of %s failed", cfg_path);
        fclose(f);
        free(buf);
        return -1;
    }
    buf[len] = 0;
    fclose(f);

    cJSON* root = cJSON_Parse(buf);
    free(buf);
    if (!root) {
        lc_log("lifecycle: persist: %s is not valid JSON", cfg_path);
        return -1;
    }

    int    ret      = -1;
    cJSON* baseband = cJSON_GetObjectItem(root, "baseband");
    cJSON* basic    = baseband ? cJSON_GetObjectItem(baseband, "basic") : NULL;
    cJSON* section  = basic ? cJSON_GetObjectItem(basic, role == BB_ROLE_AP ? "ap" : "dev") : NULL;
    if (!section) {
        lc_log("lifecycle: persist: %s has no baseband.basic.%s section", cfg_path, role == BB_ROLE_AP ? "ap" : "dev");
        cJSON_Delete(root);
        return -1;
    }

    if (role == BB_ROLE_AP) {
        cJSON* candidate = cJSON_GetObjectItem(section, "candidate");
        if (!candidate) {
            candidate = cJSON_AddObjectToObject(section, "candidate");
        }
        /* cJSON_ReplaceItemViaPointer only splices the linked list -- it
         * does NOT copy the object key onto the replacement, so it has to
         * be cJSON_ReplaceItemInObject (which strdup()s the key itself)
         * instead, or the entry silently loses its "slot"/"slotN" key on
         * the very first pair. */
        char slot_key[8];
        snprintf(slot_key, sizeof(slot_key), "slot%d", slot);
        const char* key = cJSON_GetObjectItem(candidate, "slot") ? "slot" : slot_key;

        cJSON* new_arr = cJSON_CreateArray();
        cJSON_AddItemToArray(new_arr, cJSON_CreateString(mac_hex));
        if (cJSON_GetObjectItem(candidate, key)) {
            cJSON_ReplaceItemInObject(candidate, key, new_arr);
        } else {
            cJSON_AddItemToObject(candidate, key, new_arr);
        }
    } else {
        cJSON* apmac_str = cJSON_CreateString(mac_hex);
        if (cJSON_GetObjectItem(section, "ap_mac")) {
            cJSON_ReplaceItemInObject(section, "ap_mac", apmac_str);
        } else {
            cJSON_AddItemToObject(section, "ap_mac", apmac_str);
        }
    }

    char* text = cJSON_Print(root);
    if (!text) {
        lc_log("lifecycle: persist: failed to serialize updated config");
        cJSON_Delete(root);
        return -1;
    }
    char tmp_path[512];
    snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", cfg_path);
    FILE* wf = fopen(tmp_path, "wb");
    if (!wf) {
        lc_log("lifecycle: persist: can't open %s for writing: %s", tmp_path, strerror(errno));
        free(text);
        cJSON_Delete(root);
        return -1;
    }
    fwrite(text, 1, strlen(text), wf);
    /* fsync the .tmp file's actual data before the rename -- otherwise
     * the rename can land durably (survive a power cut) while the
     * content it points at is still sitting in page cache, unflushed.
     * Confirmed live: powering off the air unit immediately after a
     * successful bind left ar8030.json empty on the next boot -- this
     * is exactly that race. */
    fflush(wf);
    fsync(fileno(wf));
    fclose(wf);
    free(text);
    if (rename(tmp_path, cfg_path) == 0) {
        ret = 0;
    } else {
        lc_log("lifecycle: persist: rename %s -> %s failed: %s", tmp_path, cfg_path, strerror(errno));
    }

    /* Marker for lc_pair_has_been_paired(): its whole point is to skip
     * auto-pairing on a factory-default unit that has never been through
     * a deliberate pair -- an empty AP candidate list means "accept
     * anyone" (see the config's own "slot match all" help text), so
     * blindly auto-pairing before this file exists would silently bind to
     * any nearby AR8030 in range. Existence alone is enough; content is
     * just for a human reading it over ssh. Best-effort -- a failure here
     * shouldn't undo an otherwise-successful persist. */
    if (ret == 0) {
        char marker_path[520];
        snprintf(marker_path, sizeof(marker_path), "%s.paired", cfg_path);
        FILE* mf = fopen(marker_path, "w");
        if (mf) {
            fprintf(mf, "%s\n", mac_hex);
            fflush(mf);
            fsync(fileno(mf));
            fclose(mf);
        }
    }

    /* Flushes the rename()s themselves (the directory entries), not just
     * the file content the fsync() calls above already covered -- a
     * rename can itself still be sitting unflushed in the containing
     * directory's metadata at power-loss time otherwise. Matches stock's
     * own ar_ldy_gnd exactly: its user_cfg.json persist function
     * (confirmed via decompile) ends with a bare sync() call too. This
     * whole function only runs once per actual pair, so the cost of a
     * full sync() (as opposed to fsync()-ing just the parent directory
     * fd) is a non-issue. */
    if (ret == 0) {
        sync();
    }

    cJSON_Delete(root);

    /* Best-effort live update too, so this session doesn't itself need a
     * reboot to lock onto the new peer -- not fatal if the running
     * firmware doesn't act on it, since the file above is what actually
     * matters after a reboot. */
    if (role == BB_ROLE_AP) {
        bb_set_candidate_t candi;
        memset(&candi, 0, sizeof(candi));
        candi.slot    = slot;
        candi.mac_num = 1;
        memcpy(&candi.mac_tab[0], mac, sizeof(bb_mac_t));
        bb_ioctl(handle, BB_SET_CANDIDATES, &candi, NULL);
    } else {
        bb_set_ap_mac_t ap_mac;
        memcpy(&ap_mac.mac, mac, sizeof(bb_mac_t));
        bb_ioctl(handle, BB_SET_AP_MAC, &ap_mac, NULL);
    }

    return ret;
}

int lc_pair_has_been_paired(const char* cfg_path)
{
    char marker_path[520];
    snprintf(marker_path, sizeof(marker_path), "%s.paired", cfg_path);
    return access(marker_path, F_OK) == 0;
}

/*
 * Ghidra RE of the stock ar_ldy_gnd binary (its fpv_gnd_read_factory_user_cfg
 * + fpv_bb_set_ap_candidate_mac, called unconditionally once at every
 * startup from its own main init, and again every 5s via a timer callback
 * armed after any successful pair) found the piece this daemon was missing:
 * the chip's own BB_SET_CANDIDATES/BB_SET_AP_MAC state lives in the chip's
 * volatile RAM, not anywhere persistent -- it does NOT survive a hardware
 * reset (power cycle, or this daemon's own devmem/gpio-sysfs reset at
 * startup). lc_pair_persist() already pushes this same ioctl live at the
 * moment a NEW pair succeeds, but that alone only helps the session that
 * was open at the time; every subsequent boot needs it re-pushed too, or
 * an already-paired unit's chip comes up with an empty candidate list and
 * has nothing to autonomously reconnect to -- no amount of passively
 * waiting fixes that, since nothing else ever tells the chip who to look
 * for. Confirmed via decompile that stock does this with a plain
 * BB_SET_CANDIDATES/BB_SET_AP_MAC ioctl, never PRJ_CMD_EVENT_PAIR --
 * exactly like lc_pair_persist()'s own live-update section already does.
 */
int lc_pair_apply_known_candidate(bb_dev_handle_t* handle, const char* cfg_path, lc_role_e role)
{
    FILE* f = fopen(cfg_path, "rb");
    if (!f) {
        lc_log("lifecycle: apply-candidate: can't open %s: %s", cfg_path, strerror(errno));
        return -1;
    }
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);
    char* buf = (char*)malloc(len + 1);
    if (!buf || fread(buf, 1, len, f) != (size_t)len) {
        lc_log("lifecycle: apply-candidate: read of %s failed", cfg_path);
        fclose(f);
        free(buf);
        return -1;
    }
    buf[len] = 0;
    fclose(f);

    cJSON* root = cJSON_Parse(buf);
    free(buf);
    if (!root) {
        lc_log("lifecycle: apply-candidate: %s is not valid JSON", cfg_path);
        return -1;
    }

    cJSON* baseband = cJSON_GetObjectItem(root, "baseband");
    cJSON* basic    = baseband ? cJSON_GetObjectItem(baseband, "basic") : NULL;
    cJSON* section  = basic ? cJSON_GetObjectItem(basic, role == LC_ROLE_AP ? "ap" : "dev") : NULL;

    const char* mac_hex = NULL;
    if (section) {
        if (role == LC_ROLE_AP) {
            cJSON* candidate = cJSON_GetObjectItem(section, "candidate");
            cJSON* arr       = candidate ? cJSON_GetObjectItem(candidate, "slot") : NULL;
            cJSON* first     = (arr && cJSON_IsArray(arr)) ? cJSON_GetArrayItem(arr, 0) : NULL;
            if (first && cJSON_IsString(first) && first->valuestring[0]) {
                mac_hex = first->valuestring;
            }
        } else {
            cJSON* apmac = cJSON_GetObjectItem(section, "ap_mac");
            if (apmac && cJSON_IsString(apmac) && apmac->valuestring[0]) {
                mac_hex = apmac->valuestring;
            }
        }
    }

    int ret = -1;
    if (mac_hex && strlen(mac_hex) >= 2 * BB_MAC_LEN) {
        bb_mac_t mac;
        memset(&mac, 0, sizeof(mac));
        for (int i = 0; i < BB_MAC_LEN; i++) {
            unsigned int byte = 0;
            sscanf(mac_hex + i * 2, "%2x", &byte);
            mac.addr[i] = (uint8_t)byte;
        }

        if (role == LC_ROLE_AP) {
            bb_set_candidate_t candi;
            memset(&candi, 0, sizeof(candi));
            candi.slot    = 0;
            candi.mac_num = 1;
            memcpy(&candi.mac_tab[0], &mac, sizeof(bb_mac_t));
            ret = bb_ioctl(handle, BB_SET_CANDIDATES, &candi, NULL);
        } else {
            bb_set_ap_mac_t ap_mac;
            memcpy(&ap_mac.mac, &mac, sizeof(bb_mac_t));
            ret = bb_ioctl(handle, BB_SET_AP_MAC, &ap_mac, NULL);
            /* Plus every earlier air unit, for multi-bind (lc_peers_push()). */
            lc_peers_push(handle, cfg_path);
        }

        if (ret == 0) {
            lc_log("lifecycle: pushed known candidate %s to chip", mac_hex);
        } else {
            lc_log("lifecycle: BB_SET_CANDIDATES/BB_SET_AP_MAC failed for %s (ret=%d)", mac_hex, ret);
        }
    }

    cJSON_Delete(root);
    return ret;
}

/* Direct fork+pipe+execlp, no shell -- matches lc_hooks_dispatch's own
 * no-shell convention. Captures ar8030-pair's stdout+stderr (2>&1) into
 * buf. Returns its exit code, or -1 on a fork/pipe/wait failure. Moved
 * here from lifecycle_bind.c unchanged (byte-for-byte) when lc_pair_run()
 * was factored out to be callable from the HTTP control API too, not
 * just the physical bind button. */
static int run_pair_tool(const char* cfg_path, char* buf, size_t buf_sz)
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
        execlp("ar8030-pair", "ar8030-pair", "-c", cfg_path, (char*)NULL);
        _exit(127);
    }

    close(pipefd[1]);
    size_t  total = 0;
    ssize_t n;
    while (total + 1 < buf_sz && (n = read(pipefd[0], buf + total, buf_sz - 1 - total)) > 0) {
        total += (size_t)n;
    }
    buf[total] = 0;
    close(pipefd[0]);

    int status = 0;
    waitpid(pid, &status, 0);
    if (!WIFEXITED(status)) {
        return -1;
    }
    return WEXITSTATUS(status);
}

/* Parses "pair: peer <8 hex chars> connected on slot <N>" out of
 * ar8030-pair's own captured stdout (dev_helper/bb_pair/txg_bb_pair.cpp's
 * own log line, confirmed live against real device output) into a slot +
 * 4-byte bb_mac_t (BB_MAC_LEN -- this chip's MAC is not a real 6-byte
 * Ethernet address). Returns 0 on a match, -1 otherwise (caller falls
 * back to slot=-1 and an unset mac, matching lc_hooks_dispatch's own
 * NULL-mac convention). Moved here from lifecycle_bind.c along with
 * run_pair_tool() above -- see lc_pair_run()'s own comment. */
static int parse_pair_output(const char* buf, int* out_slot, bb_mac_t* out_mac)
{
    const char* p = strstr(buf, "pair: peer ");
    if (!p) {
        return -1;
    }
    p += strlen("pair: peer ");

    char hex[9] = {0};
    int  slot   = -1;
    if (sscanf(p, "%8[0-9a-fA-F] connected on slot %d", hex, &slot) != 2 || strlen(hex) != 8) {
        return -1;
    }

    memset(out_mac, 0, sizeof(*out_mac));
    for (int i = 0; i < BB_MAC_LEN && i < 4; i++) {
        unsigned int byte = 0;
        sscanf(hex + i * 2, "%2x", &byte);
        out_mac->addr[i] = (uint8_t)byte;
    }
    *out_slot = slot;
    return 0;
}

int lc_pair_run(const lc_config_t* cfg, int* out_slot, bb_mac_t* out_mac)
{
    lc_hooks_dispatch(cfg->hook_dir, "pairing", cfg->role, -1, NULL);

    char out[1024];
    int  rc = run_pair_tool(cfg->cfg_path, out, sizeof(out));
    lc_log("lifecycle: pair: ar8030-pair exited %d, output: %s", rc, out);

    int      slot = -1;
    bb_mac_t mac;
    memset(&mac, 0, sizeof(mac));

    if (rc == 0) {
        if (parse_pair_output(out, &slot, &mac) == 0) {
            lc_log("lifecycle: pair: pair succeeded (slot=%d), driving connected hooks directly", slot);
            lc_hooks_dispatch(cfg->hook_dir, "connected", cfg->role, slot, &mac);
        } else {
            lc_log("lifecycle: pair: pair succeeded but couldn't parse its output, driving connected hooks with slot/mac unknown");
            lc_hooks_dispatch(cfg->hook_dir, "connected", cfg->role, -1, NULL);
        }
    } else {
        lc_log("lifecycle: pair: pair failed (rc=%d), leaving current link alone", rc);
        /* Matches lifecycle_bind.c's own former reasoning for firing this:
         * without it, a failed re-pair attempt would leave a board's own
         * hooks.d LED state stuck on whatever "pairing" already set above. */
        lc_hooks_dispatch(cfg->hook_dir, "idle", cfg->role, -1, NULL);
    }

    if (out_slot) {
        *out_slot = slot;
    }
    if (out_mac) {
        *out_mac = mac;
    }
    return rc == 0 ? 0 : -1;
}

/*
 * Multi-bind on the DEV (ground) side.
 *
 * The DEV links to its ap_mac OR to any MAC in its candidate list
 * (BB_SET_CANDIDATES, slot 0) -- confirmed live: with ap_mac pointed at a
 * nonexistent AP it relinks within ~2 s as soon as the real air unit's MAC
 * is in the list, and stays down with only unknown MACs in it or with an
 * empty list (no "accept anyone" fallback). Stock ar_ldy_gnd keeps every
 * air unit it ever paired in a 100-entry ring (/factory/user_cfg.json
 * bb_mac_addr_N) and pushes it this same way at boot and after each pair.
 *
 * Here: ap_mac in the baseband JSON stays the most recently paired air
 * unit (ar8030-pair rewrites it on every bind), and the sidecar
 * ar8030.peers holds every earlier one, newest first, one 8-hex-digit MAC
 * per line. lc_peers_note_ap_mac() records a freshly paired ap_mac on the
 * next CONNECT -- the connected MAC itself is not readable on the DEV,
 * BB_GET_STATUS reports the configured ap_mac as the peer -- and
 * lc_peers_push() hands the union to the chip. The air side stays single
 * bind: its pair replaces its one candidate.
 */

#define LC_PEERS_FILE "ar8030.peers"

static int mac_parse(const char* hex, bb_mac_t* mac)
{
    unsigned int b[BB_MAC_LEN];
    if (strlen(hex) != 2 * BB_MAC_LEN || sscanf(hex, "%2x%2x%2x%2x", &b[0], &b[1], &b[2], &b[3]) != BB_MAC_LEN) {
        return -1;
    }
    for (int i = 0; i < BB_MAC_LEN; i++) {
        mac->addr[i] = (uint8_t)b[i];
    }
    return 0;
}

static void mac_hex(const bb_mac_t* mac, char out[2 * BB_MAC_LEN + 1])
{
    for (int i = 0; i < BB_MAC_LEN; i++) {
        sprintf(out + i * 2, "%02x", mac->addr[i]);
    }
}

static int mac_in(const bb_mac_t* mac, const bb_mac_t* tab, int n)
{
    for (int i = 0; i < n; i++) {
        if (memcmp(mac, &tab[i], sizeof(*mac)) == 0) {
            return 1;
        }
    }
    return 0;
}

int lc_pair_read_ap_mac(const char* cfg_path, bb_mac_t* out)
{
    FILE* f = fopen(cfg_path, "rb");
    if (!f) {
        return -1;
    }
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);
    char* buf = (char*)malloc(len + 1);
    if (!buf || fread(buf, 1, len, f) != (size_t)len) {
        fclose(f);
        free(buf);
        return -1;
    }
    buf[len] = 0;
    fclose(f);

    cJSON* root = cJSON_Parse(buf);
    free(buf);
    if (!root) {
        return -1;
    }
    cJSON* baseband = cJSON_GetObjectItem(root, "baseband");
    cJSON* basic    = baseband ? cJSON_GetObjectItem(baseband, "basic") : NULL;
    cJSON* dev      = basic ? cJSON_GetObjectItem(basic, "dev") : NULL;
    cJSON* apmac    = dev ? cJSON_GetObjectItem(dev, "ap_mac") : NULL;
    int    ret      = -1;
    if (apmac && cJSON_IsString(apmac) && mac_parse(apmac->valuestring, out) == 0) {
        ret = 0;
    }
    cJSON_Delete(root);
    return ret;
}

int lc_peers_load(const char* cfg_path, bb_mac_t* out, int max)
{
    char path[512];
    lc_sidecar_path(cfg_path, LC_PEERS_FILE, path, sizeof(path));
    FILE* f = fopen(path, "r");
    if (!f) {
        return 0;
    }
    int  n = 0;
    char line[32];
    while (n < max && fgets(line, sizeof(line), f)) {
        line[strcspn(line, "\r\n \t")] = '\0';
        bb_mac_t mac;
        if (mac_parse(line, &mac) == 0 && !mac_in(&mac, out, n)) {
            out[n++] = mac;
        }
    }
    fclose(f);
    return n;
}

static int peers_save(const char* cfg_path, const bb_mac_t* tab, int n)
{
    char path[512], tmp_path[520];
    lc_sidecar_path(cfg_path, LC_PEERS_FILE, path, sizeof(path));
    snprintf(tmp_path, sizeof(tmp_path), "%s.tmp", path);
    FILE* f = fopen(tmp_path, "w");
    if (!f) {
        lc_log("lifecycle: peers: can't open %s for writing: %s", tmp_path, strerror(errno));
        return -1;
    }
    for (int i = 0; i < n; i++) {
        char hex[2 * BB_MAC_LEN + 1];
        mac_hex(&tab[i], hex);
        fprintf(f, "%s\n", hex);
    }
    fflush(f);
    fsync(fileno(f));
    fclose(f);
    if (rename(tmp_path, path) != 0) {
        lc_log("lifecycle: peers: rename %s -> %s failed: %s", tmp_path, path, strerror(errno));
        return -1;
    }
    sync();
    return 0;
}

/* ap_mac first, then the remembered ones, deduplicated. */
static int peers_effective(const char* cfg_path, bb_mac_t* out, int max)
{
    int n = 0;
    if (max > 0 && lc_pair_read_ap_mac(cfg_path, &out[0]) == 0) {
        n = 1;
    }
    bb_mac_t known[BB_CONFIG_MAX_SLOT_CANDIDATE];
    int      k = lc_peers_load(cfg_path, known, BB_CONFIG_MAX_SLOT_CANDIDATE);
    for (int i = 0; i < k && n < max; i++) {
        if (!mac_in(&known[i], out, n)) {
            out[n++] = known[i];
        }
    }
    return n;
}

int lc_peers_push(bb_dev_handle_t* handle, const char* cfg_path)
{
    bb_set_candidate_t candi;
    memset(&candi, 0, sizeof(candi));
    candi.slot    = 0;
    candi.mac_num = (uint8_t)peers_effective(cfg_path, candi.mac_tab, BB_CONFIG_MAX_SLOT_CANDIDATE);
    if (candi.mac_num == 0) {
        return 0;
    }
    int ret = bb_ioctl(handle, BB_SET_CANDIDATES, &candi, NULL);
    lc_log("lifecycle: peers: pushed %d known air unit(s) as candidates (ret=%d)", candi.mac_num, ret);
    return ret;
}

int lc_peers_note_ap_mac(bb_dev_handle_t* handle, const char* cfg_path)
{
    bb_mac_t ap;
    if (lc_pair_read_ap_mac(cfg_path, &ap) != 0) {
        return -1;
    }
    bb_mac_t known[BB_CONFIG_MAX_SLOT_CANDIDATE];
    int      n = lc_peers_load(cfg_path, known, BB_CONFIG_MAX_SLOT_CANDIDATE);
    if (mac_in(&ap, known, n)) {
        return 0;
    }
    /* Newest first; the oldest falls off a full list. */
    if (n == BB_CONFIG_MAX_SLOT_CANDIDATE) {
        n--;
    }
    memmove(&known[1], &known[0], (size_t)n * sizeof(known[0]));
    known[0] = ap;
    n++;
    char hex[2 * BB_MAC_LEN + 1];
    mac_hex(&ap, hex);
    lc_log("lifecycle: peers: remembering air unit %s (%d known)", hex, n);
    if (peers_save(cfg_path, known, n) != 0) {
        return -1;
    }
    return lc_peers_push(handle, cfg_path);
}

int lc_peers_forget(const char* cfg_path, const char* mac_hex_in)
{
    bb_mac_t ap, target;
    int      have_ap = lc_pair_read_ap_mac(cfg_path, &ap) == 0;
    if (mac_hex_in && mac_parse(mac_hex_in, &target) != 0) {
        return -1;
    }
    if (mac_hex_in && have_ap && memcmp(&target, &ap, sizeof(ap)) == 0) {
        return -2;
    }
    bb_mac_t known[BB_CONFIG_MAX_SLOT_CANDIDATE];
    int      n = lc_peers_load(cfg_path, known, BB_CONFIG_MAX_SLOT_CANDIDATE);
    int      kept = 0;
    for (int i = 0; i < n; i++) {
        int is_ap = have_ap && memcmp(&known[i], &ap, sizeof(ap)) == 0;
        int drop  = mac_hex_in ? memcmp(&known[i], &target, sizeof(target)) == 0 : !is_ap;
        if (!drop) {
            known[kept++] = known[i];
        }
    }
    if (kept == n) {
        return mac_hex_in ? -3 : 0;
    }
    lc_log("lifecycle: peers: forgot %d air unit(s), %d left", n - kept, kept);
    return peers_save(cfg_path, known, kept) == 0 ? n - kept : -4;
}

int lc_peers_json(const char* cfg_path, char* out, size_t out_sz)
{
    bb_mac_t ap;
    int      have_ap = lc_pair_read_ap_mac(cfg_path, &ap) == 0;
    bb_mac_t tab[BB_CONFIG_MAX_SLOT_CANDIDATE];
    int      n = peers_effective(cfg_path, tab, BB_CONFIG_MAX_SLOT_CANDIDATE);

    char   hex[2 * BB_MAC_LEN + 1];
    size_t len = 0;
    if (have_ap) {
        mac_hex(&ap, hex);
    }
    len += (size_t)snprintf(out + len, out_sz - len, "{\"ok\":true,\"current\":%s%s%s,\"max\":%d,\"peers\":[",
                            have_ap ? "\"" : "", have_ap ? hex : "null", have_ap ? "\"" : "",
                            BB_CONFIG_MAX_SLOT_CANDIDATE);
    for (int i = 0; i < n && len < out_sz; i++) {
        mac_hex(&tab[i], hex);
        len += (size_t)snprintf(out + len, out_sz - len, "%s\"%s\"", i ? "," : "", hex);
    }
    if (len < out_sz) {
        snprintf(out + len, out_sz - len, "]}");
    }
    return len < out_sz - 2 ? 0 : -1;
}
