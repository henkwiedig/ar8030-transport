#include "http_get.h"

#include <arpa/inet.h>
#include <errno.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

static int set_timeout(int fd, int timeout_ms)
{
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    if (setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)) < 0)
        return -1;
    if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
        return -1;
    return 0;
}

int http_get_status(const char *host, int port, const char *path, int timeout_ms)
{
    char portbuf[8];
    snprintf(portbuf, sizeof(portbuf), "%d", port);

    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    struct addrinfo *res = NULL;
    if (getaddrinfo(host, portbuf, &hints, &res) != 0 || !res)
        return -1;

    int fd = -1;
    int connected = -1;
    for (struct addrinfo *rp = res; rp; rp = rp->ai_next) {
        fd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
        if (fd < 0)
            continue;
        set_timeout(fd, timeout_ms);
        if (connect(fd, rp->ai_addr, rp->ai_addrlen) == 0) {
            connected = 0;
            break;
        }
        close(fd);
        fd = -1;
    }
    freeaddrinfo(res);

    if (connected != 0)
        return -1;

    char req[512];
    int req_len = snprintf(req, sizeof(req),
                            "GET %s HTTP/1.0\r\n"
                            "Host: %s\r\n"
                            "Connection: close\r\n"
                            "\r\n",
                            path, host);
    if (req_len < 0 || (size_t)req_len >= sizeof(req)) {
        close(fd);
        return -1;
    }

    ssize_t sent = 0;
    while (sent < req_len) {
        ssize_t n = write(fd, req + sent, (size_t)(req_len - sent));
        if (n <= 0) {
            close(fd);
            return -1;
        }
        sent += n;
    }

    /* Only need the status line ("HTTP/1.0 200 ...") -- read into a small
     * buffer and stop as soon as we have it or the peer closes. */
    char resp[64];
    size_t have = 0;
    while (have + 1 < sizeof(resp)) {
        ssize_t n = read(fd, resp + have, sizeof(resp) - 1 - have);
        if (n <= 0)
            break;
        have += (size_t)n;
        if (memchr(resp, '\n', have))
            break;
    }
    resp[have] = '\0';
    close(fd);

    int status = -1;
    const char *sp = strchr(resp, ' ');
    if (sp && sscanf(sp + 1, "%d", &status) != 1)
        status = -1;

    return status;
}
