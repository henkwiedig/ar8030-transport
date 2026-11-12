/*
 * ar8030-ringdump: a bench reader for rx's frame-shm ring (rx -R).
 *
 *   ar8030-ringdump [-r ring] [-u port] [-t seconds]
 *
 * Attaches to the ring (waiting for it), drops what is queued, then prints
 * once a second: entries and access units per second, partial entries, the
 * largest entry and access unit, IDR/GDR/DISCONT counts, and how long an
 * access unit sat between rx finishing it and this reader taking it (the
 * timing SEI's rx_done_ground_us against our clock -- both CLOCK_MONOTONIC on
 * this host).
 *
 * With -u <port> it also takes rx's RTP on that UDP port (rx -R -P -p
 * <port>), depacketizes it into access units at the marker bit and compares
 * each one's NAL units with the ring's access unit of the same frame (matched
 * by the timing SEI's frame_seq, so only once the clock sync holds).
 * See docs/ground-frame-ring.md.
 */

#include "timing_sei.h"
#include "venc_frame_ring.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#define AU_MAX (1024 * 1024)

static volatile sig_atomic_t g_stop;
static void on_signal(int sig) { (void)sig; g_stop = 1; }

static uint64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
}

/* rx/rtp_h265.c's NAL splitter, so both sides cut the same NAL units. */
static int next_nal(const uint8_t *data, uint32_t size, uint32_t *from, const uint8_t **nal, uint32_t *nal_len)
{
    uint32_t i = *from, start = 0;
    int found = 0;
    while (i + 2 < size) {
        if (data[i] == 0 && data[i + 1] == 0) {
            if (data[i + 2] == 1) { start = i + 3; found = 1; break; }
            if (i + 3 < size && data[i + 2] == 0 && data[i + 3] == 1) { start = i + 4; found = 1; break; }
        }
        i++;
    }
    if (!found)
        return 0;
    uint32_t j = start, end = size;
    while (j + 2 < size) {
        if (data[j] == 0 && data[j + 1] == 0 && (data[j + 2] == 1 || (j + 3 < size && data[j + 2] == 0 && data[j + 3] == 1))) {
            end = j;
            break;
        }
        j++;
    }
    *nal = data + start;
    *nal_len = end - start;
    *from = end;
    return 1;
}

/* FNV-1a over the NAL units with their lengths: start-code-agnostic. */
static uint64_t fnv(uint64_t h, const uint8_t *p, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++)
        h = (h ^ p[i]) * 0x100000001b3ull;
    return h;
}
static uint64_t hash_nal(uint64_t h, const uint8_t *nal, uint32_t len)
{
    uint8_t l[4] = {(uint8_t)(len >> 24), (uint8_t)(len >> 16), (uint8_t)(len >> 8), (uint8_t)len};
    return fnv(fnv(h, l, 4), nal, len);
}
static uint64_t hash_annexb(const uint8_t *au, uint32_t len, uint32_t *nals)
{
    uint64_t h = 0xcbf29ce484222325ull;
    uint32_t cur = 0, n = 0, nl;
    const uint8_t *nal;
    while (next_nal(au, len, &cur, &nal, &nl)) {
        h = hash_nal(h, nal, nl);
        n++;
    }
    *nals = n;
    return h;
}

/* Access units of both sources by frame_seq, until the other side's copy
 * arrives. */
#define SLOTS 256
struct seen { int have; uint16_t seq; uint64_t hash; uint32_t nals; };
static struct seen ring_seen[SLOTS], rtp_seen[SLOTS];
static uint64_t cmp_match, cmp_mismatch;

static void seen_add(struct seen *mine, struct seen *other, uint16_t seq, uint64_t hash, uint32_t nals)
{
    struct seen *o = &other[seq % SLOTS];
    if (o->have && o->seq == seq) {
        if (o->hash == hash && o->nals == nals)
            cmp_match++;
        else {
            cmp_mismatch++;
            fprintf(stderr, "ringdump: frame %u differs: ring/rtp %u vs %u NAL units\n", seq,
                    mine == ring_seen ? nals : o->nals, mine == ring_seen ? o->nals : nals);
        }
        o->have = 0;
        return;
    }
    mine[seq % SLOTS] = (struct seen){1, seq, hash, nals};
}

/* RTP/H.265 depacketizer (single NAL units, FU, AP) into Annex-B. */
struct rtp_au { uint8_t *buf; uint32_t len; int bad; int have_seq; uint16_t last_seq; uint64_t aus, gaps; };

static void rtp_put(struct rtp_au *a, const uint8_t *p, uint32_t n)
{
    if (a->len + n > AU_MAX) { a->bad = 1; return; }
    memcpy(a->buf + a->len, p, n);
    a->len += n;
}

static void rtp_packet(struct rtp_au *a, const uint8_t *pkt, uint32_t n)
{
    static const uint8_t sc[4] = {0, 0, 0, 1};
    if (n < 14 || (pkt[0] >> 6) != 2)
        return;
    uint16_t seq = (uint16_t)(pkt[2] << 8 | pkt[3]);
    if (a->have_seq && seq != (uint16_t)(a->last_seq + 1)) {
        a->bad = 1;
        a->gaps++;
    }
    a->have_seq = 1;
    a->last_seq = seq;
    uint32_t off = 12 + 4u * (pkt[0] & 0x0f);
    if (pkt[0] & 0x10) {
        if (off + 4 > n) return;
        off += 4 + 4u * (uint32_t)(pkt[off + 2] << 8 | pkt[off + 3]);
    }
    if (off + 2 > n) return;
    const uint8_t *pl = pkt + off;
    uint32_t pn = n - off;
    uint8_t type = (pl[0] >> 1) & 0x3f;
    if (type == 49 && pn > 3) { /* FU */
        if (pl[2] & 0x80) {
            uint8_t hdr[2] = {(uint8_t)((pl[0] & 0x81) | ((pl[2] & 0x3f) << 1)), pl[1]};
            rtp_put(a, sc, 4);
            rtp_put(a, hdr, 2);
        }
        rtp_put(a, pl + 3, pn - 3);
    } else if (type == 48) { /* AP */
        uint32_t i = 2;
        while (i + 2 <= pn) {
            uint32_t l = (uint32_t)(pl[i] << 8 | pl[i + 1]);
            i += 2;
            if (i + l > pn) break;
            rtp_put(a, sc, 4);
            rtp_put(a, pl + i, l);
            i += l;
        }
    } else {
        rtp_put(a, sc, 4);
        rtp_put(a, pl, pn);
    }
    if (pkt[1] & 0x80) { /* marker: the access unit is complete */
        struct timing_sei_v1 t;
        if (!a->bad && timing_sei_parse(a->buf, a->len, &t)) {
            uint32_t nals;
            uint64_t h = hash_annexb(a->buf, a->len, &nals);
            seen_add(rtp_seen, ring_seen, t.frame_seq, h, nals);
        }
        a->aus++;
        a->len = 0;
        a->bad = 0;
    }
}

int main(int argc, char **argv)
{
    const char *name = "gnd_frames";
    int udp_port = 0;
    int seconds = 0;
    int opt;
    while ((opt = getopt(argc, argv, "r:u:t:h")) != -1) {
        switch (opt) {
        case 'r': name = optarg; break;
        case 'u': udp_port = atoi(optarg); break;
        case 't': seconds = atoi(optarg); break;
        default:
            fprintf(stderr, "usage: %s [-r ring (default gnd_frames)] [-u rtp port to compare] [-t seconds]\n",
                    argv[0]);
            return 2;
        }
    }
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    venc_frame_ring_t *ring = NULL;
    while (!g_stop && !(ring = venc_frame_ring_attach(name)))
        usleep(200000);
    if (!ring)
        return 1;
    /* Whatever is queued is old: start with what comes next. */
    __atomic_store_n(&ring->hdr->read_idx, __atomic_load_n(&ring->hdr->write_idx, __ATOMIC_ACQUIRE),
                     __ATOMIC_RELEASE);
    fprintf(stderr, "ringdump: attached to '%s', %u slots x %u bytes\n", name, ring->hdr->slot_count,
            ring->slot_data_size);

    int udp = -1;
    struct rtp_au rtp = {0};
    static uint8_t pkt[65536];
    if (udp_port) {
        udp = socket(AF_INET, SOCK_DGRAM, 0);
        struct sockaddr_in sa = {.sin_family = AF_INET, .sin_port = htons((uint16_t)udp_port),
                                 .sin_addr.s_addr = htonl(INADDR_LOOPBACK)};
        int rcv = 2 * 1024 * 1024;
        setsockopt(udp, SOL_SOCKET, SO_RCVBUF, &rcv, sizeof(rcv));
        if (bind(udp, (struct sockaddr *)&sa, sizeof(sa)) != 0) {
            perror("ringdump: bind");
            return 1;
        }
        rtp.buf = malloc(AU_MAX);
    }

    uint8_t *slot = malloc(ring->slot_data_size);
    uint8_t *au = malloc(AU_MAX);
    uint32_t au_len = 0;
    int au_skip = 1; /* until the first entry after an access unit's end */
    uint64_t t_start = now_us(), t_print = t_start;
    uint64_t n_entries = 0, n_partial = 0, n_aus = 0, n_bytes = 0, n_idr = 0, n_gdr = 0, n_discont = 0;
    uint32_t max_entry = 0, max_au = 0;
    double hop_sum = 0, hop_max = 0;
    uint64_t hop_n = 0, tot_discont = 0;

    while (!g_stop) {
        uint32_t len;
        int got = venc_frame_ring_read_wait(ring, slot, ring->slot_data_size, &len, udp >= 0 ? 2 : 100) == 0;
        uint64_t t_read = now_us();
        if (got && len >= VENC_FRAME_META_SIZE) {
            VencFrameMeta m;
            memcpy(&m, slot, VENC_FRAME_META_SIZE);
            uint32_t dlen = len - VENC_FRAME_META_SIZE;
            n_entries++;
            n_bytes += dlen;
            if (dlen > max_entry) max_entry = dlen;
            if (m.flags & VENC_FRAME_FLAG_PARTIAL) n_partial++;
            if (m.flags & VENC_FRAME_FLAG_DISCONT) { n_discont++; tot_discont++; }
            if (!au_skip) {
                if (au_len + dlen <= AU_MAX) {
                    memcpy(au + au_len, slot + VENC_FRAME_META_SIZE, dlen);
                    au_len += dlen;
                }
                if (!(m.flags & VENC_FRAME_FLAG_PARTIAL)) {
                    n_aus++;
                    if (au_len > max_au) max_au = au_len;
                    if (m.flags & VENC_FRAME_FLAG_IDR) n_idr++;
                    if (m.flags & VENC_FRAME_FLAG_GDR) n_gdr++;
                    struct timing_sei_v1 t;
                    if (timing_sei_parse(au, au_len, &t)) {
                        double hop = ((double)t_read - (double)t.rx_done_ground_us) / 1000.0;
                        hop_sum += hop;
                        if (hop > hop_max) hop_max = hop;
                        hop_n++;
                        if (udp >= 0) {
                            uint32_t nals;
                            uint64_t h = hash_annexb(au, au_len, &nals);
                            seen_add(ring_seen, rtp_seen, t.frame_seq, h, nals);
                        }
                    }
                    au_len = 0;
                }
            } else if (!(m.flags & VENC_FRAME_FLAG_PARTIAL)) {
                au_skip = 0;
            }
        }
        if (udp >= 0) {
            struct pollfd p = {.fd = udp, .events = POLLIN};
            while (poll(&p, 1, 0) > 0) {
                ssize_t n = recv(udp, pkt, sizeof(pkt), 0);
                if (n <= 0) break;
                rtp_packet(&rtp, pkt, (uint32_t)n);
            }
        }
        uint64_t now = now_us();
        if (now - t_print >= 1000000u) {
            double dt = (now - t_print) / 1e6;
            printf("ring: %.1f entries/s (%.1f partial), %.1f AU/s, %.2f Mbit/s, max entry %u, max AU %u | "
                   "IDR %llu GDR %llu DISCONT %llu (total %llu) | rx_done->read %.2f ms avg %.2f max",
                   n_entries / dt, n_partial / dt, n_aus / dt, n_bytes * 8 / dt / 1e6, max_entry, max_au,
                   (unsigned long long)n_idr, (unsigned long long)n_gdr, (unsigned long long)n_discont,
                   (unsigned long long)tot_discont, hop_n ? hop_sum / hop_n : 0.0, hop_max);
            if (udp >= 0)
                printf(" | rtp %llu AU, %llu seq gaps | compared %llu equal, %llu differ",
                       (unsigned long long)rtp.aus, (unsigned long long)rtp.gaps, (unsigned long long)cmp_match,
                       (unsigned long long)cmp_mismatch);
            printf("\n");
            fflush(stdout);
            n_entries = n_partial = n_aus = n_bytes = n_idr = n_gdr = n_discont = 0;
            max_entry = max_au = 0;
            hop_sum = hop_max = 0;
            hop_n = 0;
            t_print = now;
            if (seconds && now - t_start >= (uint64_t)seconds * 1000000u)
                break;
        }
    }
    venc_frame_ring_destroy(ring);
    return cmp_mismatch ? 1 : 0;
}
