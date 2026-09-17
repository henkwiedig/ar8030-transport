#include "lifecycle_pair.h"
#include "lc_log.h"
#include <cjson/cJSON.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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
