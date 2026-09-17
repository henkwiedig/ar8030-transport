#ifndef LC_LOG_H
#define LC_LOG_H
#ifdef __cplusplus
extern "C" {
#endif

#include <syslog.h>

/*
 * Thin syslog wrapper replacing the vendor SDK's com_log()/com_log_init(),
 * dropped when this module moved out of package/ar8030's patch stack
 * into this project's own repo: com_log's own `com` library was never
 * staged to $(STAGING_DIR) (only libar8030_client.so and the public
 * headers tx/ and linkctl/ already build against are), so keeping it
 * would have meant staying inside the vendor SDK's own CMake build
 * forever. Confirmed live (debugging this exact move, `logread | grep -i
 * lifecycle`) that ar8030-lifecycled's com_log output already reached
 * syslog somehow even before this change, so switching fully to
 * syslog() preserves that workflow -- and drops com_log_init()'s
 * separate per-run file under a relative lifecycle_log/ dir in $RUNDIR
 * entirely (that file needed its own growth cap upstream; syslog's own
 * ring buffer doesn't have that problem in the first place).
 *
 * Call openlog("ar8030-lifecycled", LOG_PID, LOG_USER) once at process
 * startup (see main.c) before using this. Message text keeps the
 * original "lifecycle: ..." prefix throughout this module rather than
 * relying on the syslog tag alone, so existing `logread`/`grep`
 * workflows built around that text keep working unchanged.
 */
#define lc_log(...) syslog(LOG_NOTICE, __VA_ARGS__)

#ifdef __cplusplus
}
#endif
#endif
