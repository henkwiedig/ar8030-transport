#include "lifecycle_hooks.h"
#include "lc_log.h"
#include <dirent.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

void lc_hooks_init(void)
{
    /* Fire-and-forget: a forked hook script is reaped by the kernel the
     * moment it exits, with no waitpid() call ever needed on this side --
     * exactly what "never block the lifecycle thread on a hook" requires. */
    signal(SIGCHLD, SIG_IGN);
}

static int name_cmp(const void* a, const void* b)
{
    return strcmp(*(const char* const*)a, *(const char* const*)b);
}

static void format_mac(const bb_mac_t* mac, char* out, size_t out_sz)
{
    if (!mac) {
        snprintf(out, out_sz, "00:00:00:00");
        return;
    }
    snprintf(out, out_sz, "%02x:%02x:%02x:%02x", mac->addr[0], mac->addr[1], mac->addr[2], mac->addr[3]);
}

void lc_hooks_dispatch(const char* hook_dir, const char* event, lc_role_e role, int slot, const bb_mac_t* peer_mac)
{
    char dir_path[512];
    snprintf(dir_path, sizeof(dir_path), "%s/%s", hook_dir, event);

    DIR* d = opendir(dir_path);
    if (!d) {
        /* No hooks configured for this event -- not an error, most
         * boards will have none at all. */
        return;
    }

    /* Collect names first, then sort, so scripts run in a predictable
     * (lexical) order regardless of what readdir()'s own order happens
     * to be -- matches run-parts convention. */
    char* names[256];
    int   count = 0;
    struct dirent* ent;
    while (count < 256 && (ent = readdir(d)) != NULL) {
        if (ent->d_name[0] == '.') {
            continue;
        }
        names[count++] = strdup(ent->d_name);
    }
    closedir(d);
    qsort(names, count, sizeof(names[0]), name_cmp);

    char slot_str[16];
    snprintf(slot_str, sizeof(slot_str), "%d", slot);
    char mac_str[16];
    format_mac(peer_mac, mac_str, sizeof(mac_str));
    const char* role_str = role == LC_ROLE_AP ? "ap" : "dev";

    for (int i = 0; i < count; i++) {
        char script_path[768];
        snprintf(script_path, sizeof(script_path), "%s/%s", dir_path, names[i]);

        struct stat st;
        if (stat(script_path, &st) != 0 || !S_ISREG(st.st_mode) || !(st.st_mode & S_IXUSR)) {
            free(names[i]);
            continue;
        }

        pid_t pid = fork();
        if (pid == 0) {
            setenv("AR8030_EVENT", event, 1);
            setenv("AR8030_ROLE", role_str, 1);
            setenv("AR8030_SLOT", slot_str, 1);
            setenv("AR8030_PEER_MAC", mac_str, 1);
            execl(script_path, script_path, event, (char*)NULL);
            _exit(127);
        } else if (pid > 0) {
            lc_log("lifecycle: hooks: ran %s (pid %d)", script_path, (int)pid);
        } else {
            lc_log("lifecycle: hooks: fork failed for %s", script_path);
        }
        free(names[i]);
    }
}
