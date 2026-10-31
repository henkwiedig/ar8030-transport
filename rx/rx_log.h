/*
 * rx_log -- -L <file>: stdout and stderr into a size-capped log file.
 *
 * The ground unit runs for hours (often left on at the bench), and -v
 * alone writes ~0.5 KB/s, so a plain shell redirect like tx's would grow
 * a RAM-backed /tmp without bound. rx_log_open() points fds 1 and 2 at
 * <file> (append) and starts a thread that rotates it once per second
 * when it passes RX_LOG_MAX_BYTES: <file> -> <file>.1 -> ... ->
 * <file>.RX_LOG_KEEP, oldest dropped. A thread, not a check in the main
 * loop, because that loop can sit in ar8030_link_connect_retry() for as
 * long as ar8030d is gone, still printing a line per retry.
 *
 * A non-empty <file> left by a previous run is rotated away at open, so
 * a restart keeps the run before it as <file>.1.
 */
#ifndef RX_LOG_H
#define RX_LOG_H

#define RX_LOG_MAX_BYTES (1024 * 1024)
#define RX_LOG_KEEP 2

/* 0 on success; -1 (error printed, stdout/stderr left as they were) if
 * <path> can't be opened. */
int rx_log_open(const char *path);

#endif
