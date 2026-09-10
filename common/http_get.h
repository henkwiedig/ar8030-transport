#ifndef AR8030_TRANSPORT_HTTP_GET_H
#define AR8030_TRANSPORT_HTTP_GET_H

/*
 * Minimal blocking HTTP/1.0 GET client, just enough to hit waybeam's
 * loopback control API (documentation/HTTP_API_CONTRACT.md: "HTTP/1.0,
 * all methods use GET (compatible with BusyBox wget)"). Not a general
 * HTTP client -- no redirects, no chunked transfer-encoding, no TLS.
 * Written instead of shelling out to wget/curl so bitrate_ctl.c's control
 * loop has no subprocess-spawn latency and no dependency on either tool
 * being present on the target.
 */

/* Issues "GET <path> HTTP/1.0" to host:port and waits (bounded by
 * timeout_ms) for the response. Returns the HTTP status code (e.g. 200)
 * on success, or -1 on any connection/timeout/parse failure. Does not
 * return the response body -- callers only need to know the request
 * landed; waybeam logs rejections (e.g. out-of-range bitrate) on its own
 * side. */
int http_get_status(const char *host, int port, const char *path, int timeout_ms);

#endif /* AR8030_TRANSPORT_HTTP_GET_H */
