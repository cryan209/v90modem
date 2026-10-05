/*
 * v76_test.c -- V.76 multiplex function, two ends back to back over a bit
 * pipe.
 *
 * The FCS is checked against an independent implementation written straight
 * from 5.1.6 (polynomial division of x^k * ones and of x^w * M, complemented,
 * high-order coefficient first), not against the reflected-register shortcut
 * the module uses.  Frames are also hand-built and fed bit by bit, so what the
 * receiver accepts is judged by the Recommendation and not by our own
 * transmitter.
 */

#include "v76.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures;
static int checks;

#define CHECK(cond, ...) \
    do { \
        checks++; \
        if (!(cond)) { \
            printf("FAIL: "); printf(__VA_ARGS__); printf("  [%s:%d]\n", __FILE__, __LINE__); \
            failures++; \
        } \
    } while (0)

static uint32_t rng_state = 0x1234567u;
static uint32_t rnd(void)
{
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return rng_state;
}

/* ---------------------------------------------------------------------- *
 * Independent FCS, from 5.1.6
 * ---------------------------------------------------------------------- */

/* Polynomial remainder.  `bits` are coefficients, highest power first. */
static uint32_t poly_rem(const uint8_t *bits, int nbits, int w, uint32_t g_low)
{
    /* g = x^w + g_low.  Long division, MSB first. */
    uint32_t r = 0;
    uint32_t top = w == 32 ? 0x80000000u : (1u << (w - 1));
    uint32_t mask = w == 32 ? 0xFFFFFFFFu : ((1u << w) - 1);
    int i;

    for (i = 0; i < nbits; i++) {
        uint32_t fb = (r & top) ? 1u : 0u;

        r = ((r << 1) | bits[i]) & mask;
        if (fb)
            r ^= g_low;
    }
    return r;
}

static void spec_fcs(int fcs_len, const uint8_t *d, int n, uint8_t *out)
{
    int w = fcs_len * 8;
    uint32_t g_low = w == 8 ? 0x07u : w == 16 ? 0x1021u : 0x04C11DB7u;
    uint32_t mask = w == 32 ? 0xFFFFFFFFu : ((1u << w) - 1);
    int k = n * 8, i, b;
    uint8_t *m = malloc((size_t)k + (size_t)w);
    uint8_t *ones = malloc((size_t)k + (size_t)w);
    uint32_t r1, r2, fcs;

    /* Message polynomial in transmission order: bit 1 of octet 1 is the
     * highest power (5.2.1.1, 5.1.6). */
    for (i = 0; i < n; i++)
        for (b = 0; b < 8; b++)
            m[i * 8 + b] = (d[i] >> b) & 1;
    memset(m + k, 0, (size_t)w);                    /* x^w * M */
    r2 = poly_rem(m, k + w, w, g_low);

    /* x^k * (x^(w-1) + ... + 1): w ones followed by k zeros. */
    memset(ones, 0, (size_t)k + (size_t)w);
    memset(ones, 1, (size_t)w);
    r1 = poly_rem(ones, k + w, w, g_low);

    fcs = ~(r1 ^ r2) & mask;
    /* On the wire the x^(w-1) coefficient is bit 1 of the first octet. */
    memset(out, 0, (size_t)fcs_len);
    for (i = 0; i < w; i++) {
        int coeff = (fcs >> (w - 1 - i)) & 1;

        if (coeff)
            out[i / 8] |= (uint8_t)(1u << (i % 8));
    }
    free(m);
    free(ones);
}

static void test_fcs(void)
{
    static const int lens[3] = { 1, 2, 4 };
    int li, t;

    for (li = 0; li < 3; li++) {
        for (t = 0; t < 200; t++) {
            uint8_t d[64], want[4], have[4], tmp[70];
            int n = 1 + (int)(rnd() % 60), i;
            uint32_t f;

            for (i = 0; i < n; i++)
                d[i] = (uint8_t)rnd();
            spec_fcs(lens[li], d, n, want);
            f = v76_fcs(lens[li], d, n);
            for (i = 0; i < lens[li]; i++)
                have[i] = (uint8_t)(f >> (8 * i));
            CHECK(memcmp(want, have, (size_t)lens[li]) == 0,
                  "FCS-%d differs from the 5.1.6 definition (n=%d)", lens[li] * 8, n);
            memcpy(tmp, d, (size_t)n);
            memcpy(tmp + n, have, (size_t)lens[li]);
            CHECK(v76_fcs_residue(lens[li], tmp, n + lens[li]) ==
                  v76_fcs_expected_residue(lens[li]),
                  "FCS-%d residue is not the constant 5.1.6 gives", lens[li] * 8);
        }
    }
    {
        const uint8_t chk[9] = "123456789";

        CHECK(v76_fcs(2, chk, 9) == 0x906E, "CRC-16/X.25 check value");
        CHECK(v76_fcs(4, chk, 9) == 0xCBF43926u, "CRC-32 check value");
    }
}

/* ---------------------------------------------------------------------- *
 * Hand-built wire
 * ---------------------------------------------------------------------- */

typedef struct {
    uint8_t bits[1 << 18];
    int n;
} bitbuf_t;

static void bb_raw(bitbuf_t *b, uint32_t v, int n)
{
    int i;

    for (i = 0; i < n; i++)
        b->bits[b->n++] = (v >> i) & 1;
}

static void bb_flag(bitbuf_t *b) { bb_raw(b, 0x7E, 8); }

static void bb_frame(bitbuf_t *b, const uint8_t *f, int n)
{
    int ones = 0, i, k;

    for (i = 0; i < n; i++)
        for (k = 0; k < 8; k++) {
            int bit = (f[i] >> k) & 1;

            b->bits[b->n++] = (uint8_t)bit;
            if (bit) {
                if (++ones == 5) {
                    b->bits[b->n++] = 0;
                    ones = 0;
                }
            } else {
                ones = 0;
            }
        }
}

static void feed(v76_t *mf, const bitbuf_t *b)
{
    int i;

    for (i = 0; i < b->n; i++)
        v76_rx_put_bit(mf, b->bits[i]);
}

/* ---------------------------------------------------------------------- *
 * Two ends
 * ---------------------------------------------------------------------- */

#define MAXS 8
typedef struct {
    int dlci;
    uint8_t *buf;
    size_t len, cap;
} stream_t;

typedef struct {
    v76_t *mf;
    stream_t st[MAXS];
    int nst;
    bool auto_accept;
    v76_dlc_params_t accept;
    uint8_t ack_ud[16];
    int ack_len;
    /* observations */
    int est_ind, est_conf, rel_ind, setparm_ind, setparm_conf, test_ind, fcs_err;
    int last_rel_dlci;
    v76_release_reason_t last_rel_why;
    uint8_t last_est_ud[64];
    int last_est_ud_len;
    uint8_t last_conf_ud[64];
    int last_conf_len;
    int ui_count;
    int ui_hdr_only;
    uint8_t last_ui[256];
    int last_ui_len;
    bool last_ui_resp, last_ui_pf;
    uint8_t last_xid[64];
    int last_xid_len;
    /* voice-latency instrumentation */
    long enq_bits[100000];
    long lat_max, lat_sum;
    int lat_n, voice_bad, voice_next, voice_dlci;
    long *clock;
} end_t;

static stream_t *get_stream(end_t *e, int dlci)
{
    int i;

    for (i = 0; i < e->nst; i++)
        if (e->st[i].dlci == dlci)
            return &e->st[i];
    if (e->nst == MAXS)
        return NULL;
    e->st[e->nst].dlci = dlci;
    return &e->st[e->nst++];
}

static void cb_est_ind(void *c, int dlci, const uint8_t *ud, int len)
{
    end_t *e = c;

    e->est_ind++;
    e->last_est_ud_len = len > 64 ? 64 : len;
    memcpy(e->last_est_ud, ud, (size_t)e->last_est_ud_len);
    if (e->auto_accept)
        v76_establish_rsp(e->mf, dlci, &e->accept, e->ack_ud, e->ack_len);
    else
        v76_establish_reject(e->mf, dlci, NULL, 0);
}

static void cb_est_conf(void *c, int dlci, const uint8_t *ud, int len)
{
    end_t *e = c;

    (void)dlci;
    e->est_conf++;
    e->last_conf_len = len > 64 ? 64 : len;
    if (len > 0)
        memcpy(e->last_conf_ud, ud, (size_t)e->last_conf_len);
}

static void cb_rel_ind(void *c, int dlci, const uint8_t *ud, int len, v76_release_reason_t why)
{
    end_t *e = c;

    (void)ud;
    (void)len;
    e->rel_ind++;
    e->last_rel_dlci = dlci;
    e->last_rel_why = why;
}

static void cb_data(void *c, int dlci, const uint8_t *d, int len)
{
    end_t *e = c;
    stream_t *s = get_stream(e, dlci);

    if (!s)
        return;
    if (s->len + (size_t)len > s->cap) {
        s->cap = (s->len + (size_t)len) * 2 + 1024;
        s->buf = realloc(s->buf, s->cap);
    }
    memcpy(s->buf + s->len, d, (size_t)len);
    s->len += (size_t)len;
}

static void cb_unit(void *c, int dlci, const uint8_t *d, int len, bool resp, bool pf, bool ho)
{
    end_t *e = c;

    e->ui_count++;
    if (ho)
        e->ui_hdr_only++;
    e->last_ui_len = len > 256 ? 256 : len;
    memcpy(e->last_ui, d, (size_t)e->last_ui_len);
    e->last_ui_resp = resp;
    e->last_ui_pf = pf;
    if (e->voice_dlci >= 0 && dlci == e->voice_dlci && len >= 4) {
        uint32_t seq = (uint32_t)d[0] | (uint32_t)d[1] << 8 | (uint32_t)d[2] << 16 |
                       (uint32_t)d[3] << 24;
        long lat = *e->clock - e->enq_bits[seq];

        if ((int)seq != e->voice_next)
            e->voice_bad++;
        e->voice_next = (int)seq + 1;
        if (lat > e->lat_max)
            e->lat_max = lat;
        e->lat_sum += lat;
        e->lat_n++;
    }
}

static void cb_setparm_ind(void *c, int dlci, const uint8_t *ud, int len)
{
    end_t *e = c;

    e->setparm_ind++;
    e->last_xid_len = len > 64 ? 64 : len;
    memcpy(e->last_xid, ud, (size_t)e->last_xid_len);
    v76_setparm_rsp(e->mf, dlci, ud, len);
}

static void cb_setparm_conf(void *c, int dlci, const uint8_t *ud, int len)
{
    end_t *e = c;

    (void)dlci;
    e->setparm_conf++;
    e->last_xid_len = len > 64 ? 64 : len;
    memcpy(e->last_xid, ud, (size_t)e->last_xid_len);
}

static void cb_test(void *c, int dlci, const uint8_t *d, int len)
{
    end_t *e = c;

    (void)dlci;
    (void)d;
    (void)len;
    e->test_ind++;
}

static void cb_fcs(void *c, int dlci)
{
    end_t *e = c;

    (void)dlci;
    e->fcs_err++;
}

static void end_init(end_t *e, bool initiator, const v76_config_t *cfg_in, long *clock)
{
    v76_config_t cfg;
    v76_su_t su;

    memset(e, 0, sizeof(*e));
    if (cfg_in)
        cfg = *cfg_in;
    else
        v76_config_default(&cfg, initiator);
    cfg.initiator = initiator;
    cfg.line_bit_rate = cfg.line_bit_rate ? cfg.line_bit_rate : 28800;
    memset(&su, 0, sizeof(su));
    su.ctx = e;
    su.establish_ind = cb_est_ind;
    su.establish_conf = cb_est_conf;
    su.release_ind = cb_rel_ind;
    su.data_ind = cb_data;
    su.unitdata_ind = cb_unit;
    su.setparm_ind = cb_setparm_ind;
    su.setparm_conf = cb_setparm_conf;
    su.test_ind = cb_test;
    su.fcs_error = cb_fcs;
    e->mf = v76_create(&cfg, &su);
    e->auto_accept = true;
    v76_dlc_params_default(&e->accept);
    e->voice_dlci = -1;
    e->clock = clock;
}

static void end_free(end_t *e)
{
    int i;

    for (i = 0; i < e->nst; i++)
        free(e->st[i].buf);
    v76_destroy(e->mf);
}

typedef struct {
    end_t a, b;
    long clock;
    uint32_t ber_ab, ber_ba;            /* bit-error threshold, /2^32 */
    long black_from, black_to;          /* both directions silent (all ones) */
    uint64_t flips;
} pair_t;

static void pair_run(pair_t *p, long bits)
{
    long i;

    for (i = 0; i < bits; i++) {
        int ba = v76_tx_get_bit(p->a.mf);
        int bb = v76_tx_get_bit(p->b.mf);

        if (p->ber_ab && rnd() < p->ber_ab) { ba ^= 1; p->flips++; }
        if (p->ber_ba && rnd() < p->ber_ba) { bb ^= 1; p->flips++; }
        if (p->black_to && p->clock >= p->black_from && p->clock < p->black_to) {
            ba = 1;
            bb = 1;
        }
        v76_rx_put_bit(p->b.mf, ba);
        v76_rx_put_bit(p->a.mf, bb);
        p->clock++;
    }
}

static void pair_init(pair_t *p, const v76_config_t *ca, const v76_config_t *cb)
{
    memset(p, 0, sizeof(*p));
    end_init(&p->a, true, ca, &p->clock);
    end_init(&p->b, false, cb, &p->clock);
}

static void pair_free(pair_t *p)
{
    end_free(&p->a);
    end_free(&p->b);
}

/* ---------------------------------------------------------------------- */

/* Independent decoder for what a transmitter put on the line: remove the
 * 0 after five 1s, find the first non-empty run between two flags, return it
 * as octets (FCS included). */
static int decode_first_frame(const bitbuf_t *cap, uint8_t *out, int max)
{
    static uint8_t raw[1 << 16];
    int n = 0, ones = 0, i;
    int f0 = -1;

    for (i = 0; i < cap->n; i++) {
        int bit = cap->bits[i];

        if (!bit && ones == 5) {
            ones = 0;
            continue;
        }
        ones = bit ? ones + 1 : 0;
        raw[n++] = (uint8_t)bit;
    }
    for (i = 0; i + 8 <= n; i++) {
        static const uint8_t flag[8] = { 0, 1, 1, 1, 1, 1, 1, 0 };

        if (memcmp(raw + i, flag, 8) == 0) {
            if (f0 < 0) {
                f0 = i + 8;
            } else if (i > f0) {
                int bits = i - f0, k, o = 0;

                if (bits % 8)
                    return -1;
                memset(out, 0, (size_t)max);
                for (k = 0; k < bits && o < max; k++) {
                    if (raw[f0 + k])
                        out[k / 8] |= (uint8_t)(1u << (k % 8));
                    o = (k + 1) / 8;
                }
                return bits / 8;
            } else {
                f0 = i + 8;
            }
            i += 7;
        }
    }
    return 0;
}

static void test_handbuilt(void)
{
    /* Responder receives a hand-built SABME on DLCI 0.  Table 2: a command
     * from the initiator carries C/R = 1, so the address octet is
     * 0 0 0 0 0 0 1 1 = 0x03; SABME with P=1 is 0x7F. */
    pair_t p;
    bitbuf_t bb;
    uint8_t f[32];
    uint8_t ctl = 0x7F;
    const uint8_t ud[3] = { 0xA1, 0xB2, 0xC3 };
    int n;

    pair_init(&p, NULL, NULL);
    n = v76_build_frame(f, 1, 0, true, &ctl, 1, ud, 3, 2, 0);
    CHECK(f[0] == 0x03 && f[1] == 0x7F, "SABME address/control octets: %02x %02x", f[0], f[1]);
    /* Independent FCS over the 5 octets. */
    {
        uint8_t want[2];

        spec_fcs(2, f, 5, want);
        CHECK(f[5] == want[0] && f[6] == want[1], "SABME FCS bytes");
    }
    memset(&bb, 0, sizeof(bb));
    bb_flag(&bb); bb_flag(&bb);
    bb_frame(&bb, f, n);
    bb_flag(&bb);
    feed(p.b.mf, &bb);
    CHECK(p.b.est_ind == 1, "hand-built SABME produced an establish indication");
    CHECK(p.b.last_est_ud_len == 3 && memcmp(p.b.last_est_ud, ud, 3) == 0, "SABME user data");

    /* The UA that comes back.  A response from the responder carries C/R = 1
     * (Table 2), so the address octet is 0x03 again, and with F=1 the
     * control octet is 0x73. */
    {
        bitbuf_t cap;
        uint8_t out[32];
        int i, got;

        memset(&cap, 0, sizeof(cap));
        for (i = 0; i < 400; i++)
            cap.bits[cap.n++] = (uint8_t)v76_tx_get_bit(p.b.mf);
        got = decode_first_frame(&cap, out, sizeof(out));
        CHECK(got >= 5, "responder sent a frame back (%d octets)", got);
        if (got >= 5) {
            CHECK(out[0] == 0x03, "UA address octet 0x03: %02x", out[0]);
            CHECK(out[1] == 0x73, "UA control octet with F=1 is 0x73: %02x", out[1]);
        }
    }
    pair_free(&p);
}

static void test_invalid_and_abort(void)
{
    pair_t p;
    bitbuf_t bb;
    uint8_t f[64], ctl = 0x7F;
    int n;

    pair_init(&p, NULL, NULL);
    /* Bad FCS -> dropped. */
    n = v76_build_frame(f, 1, 0, true, &ctl, 1, NULL, 0, 2, 0);
    f[n - 1] ^= 0x10;
    memset(&bb, 0, sizeof(bb));
    bb_flag(&bb);
    bb_frame(&bb, f, n);
    bb_flag(&bb);
    feed(p.b.mf, &bb);
    CHECK(p.b.est_ind == 0, "SABME with a bad FCS is ignored");
    CHECK(v76_stats(p.b.mf)->rx_fcs_errors == 1 && p.b.fcs_err == 1, "FCS error reported");

    /* Too short (5.3 b): 16-bit FCS needs at least 5 octets for I/S, 4 for U. */
    memset(&bb, 0, sizeof(bb));
    bb_flag(&bb);
    {
        uint8_t s[3] = { 0x03, 0x7F, 0x00 };

        bb_frame(&bb, s, 3);
    }
    bb_flag(&bb);
    feed(p.b.mf, &bb);
    CHECK(p.b.est_ind == 0, "3-octet frame is invalid");

    /* Not a whole number of octets. */
    memset(&bb, 0, sizeof(bb));
    bb_flag(&bb);
    n = v76_build_frame(f, 1, 0, true, &ctl, 1, NULL, 0, 2, 0);
    bb_frame(&bb, f, n);
    bb.bits[bb.n++] = 1;
    bb.bits[bb.n++] = 0;
    bb.bits[bb.n++] = 1;
    bb_flag(&bb);
    feed(p.b.mf, &bb);
    CHECK(p.b.est_ind == 0, "frame of a non-integral number of octets is invalid");

    /* Abort: seven or more ones in the middle of a frame. */
    memset(&bb, 0, sizeof(bb));
    bb_flag(&bb);
    n = v76_build_frame(f, 1, 0, true, &ctl, 1, NULL, 0, 2, 0);
    {
        bitbuf_t half;

        memset(&half, 0, sizeof(half));
        bb_frame(&half, f, n);
        memcpy(bb.bits + bb.n, half.bits, (size_t)half.n / 2);
        bb.n += half.n / 2;
        bb_raw(&bb, 0xFF, 8);                           /* abort */
        bb_raw(&bb, 0xFF, 8);
        memcpy(bb.bits + bb.n, half.bits + half.n / 2, (size_t)(half.n - half.n / 2));
        bb.n += half.n - half.n / 2;
    }
    bb_flag(&bb);
    feed(p.b.mf, &bb);
    CHECK(p.b.est_ind == 0, "an aborted frame is discarded");
    CHECK(v76_stats(p.b.mf)->rx_aborts >= 1, "abort counted");

    /* ...and the line recovers: a good frame after the abort is accepted. */
    memset(&bb, 0, sizeof(bb));
    bb_flag(&bb);
    n = v76_build_frame(f, 1, 0, true, &ctl, 1, NULL, 0, 2, 0);
    bb_frame(&bb, f, n);
    bb_flag(&bb);
    feed(p.b.mf, &bb);
    CHECK(p.b.est_ind == 1, "a good frame after an abort is accepted");
    pair_free(&p);
}

static void test_zero_insertion(void)
{
    /* A payload full of 0xFF and 0x7E exercises 5.1.2 in both directions. */
    pair_t p;
    uint8_t d[128];
    int i, dlci;

    pair_init(&p, NULL, NULL);
    dlci = v76_establish_req(p.a.mf, NULL, NULL, 0);
    pair_run(&p, 20000);
    CHECK(p.a.est_conf == 1 && dlci == 0, "established on DLCI %d", dlci);
    for (i = 0; i < 128; i++)
        d[i] = (uint8_t)(i % 3 == 0 ? 0xFF : i % 3 == 1 ? 0x7E : 0x7F);
    for (i = 0; i < 20; i++)
        CHECK(v76_data_req(p.a.mf, dlci, d, 128), "queue SDU");
    pair_run(&p, 200000);
    {
        stream_t *s = get_stream(&p.b, dlci);

        CHECK(s && s->len == 20 * 128, "all %d octets delivered", 20 * 128);
        if (s)
            for (i = 0; i < 20; i++)
                CHECK(memcmp(s->buf + (size_t)i * 128, d, 128) == 0, "payload %d intact", i);
    }
    pair_free(&p);
}

/* Send `count` random SDUs each way over DLC `dlci_a` (opened by A) and
 * `dlci_b` (opened by B), then compare. */
static void bulk(pair_t *p, int dlci_a, int dlci_b, int count, long run_bits,
                 uint8_t **sent_a, size_t *len_a, uint8_t **sent_b, size_t *len_b)
{
    int i;
    size_t la = 0, lb = 0;
    uint8_t *sa = malloc((size_t)count * 130), *sb = malloc((size_t)count * 130);

    for (i = 0; i < count; i++) {
        uint8_t d[130];
        int n = 1 + (int)(rnd() % 120), k;

        for (k = 0; k < n; k++)
            d[k] = (uint8_t)rnd();
        while (!v76_data_req(p->a.mf, dlci_a, d, n))
            pair_run(p, 600);
        memcpy(sa + la, d, (size_t)n);
        la += (size_t)n;
        n = 1 + (int)(rnd() % 120);
        for (k = 0; k < n; k++)
            d[k] = (uint8_t)rnd();
        while (!v76_data_req(p->b.mf, dlci_b, d, n))
            pair_run(p, 600);
        memcpy(sb + lb, d, (size_t)n);
        lb += (size_t)n;
    }
    pair_run(p, run_bits);
    *sent_a = sa;
    *sent_b = sb;
    *len_a = la;
    *len_b = lb;
}

static void check_bulk(pair_t *p, int dlci_a, int dlci_b, uint8_t *sa, size_t la,
                       uint8_t *sb, size_t lb, const char *what)
{
    stream_t *rb = get_stream(&p->b, dlci_a);
    stream_t *ra = get_stream(&p->a, dlci_b);

    CHECK(rb && rb->len == la && memcmp(rb->buf, sa, la) == 0,
          "%s: A->B stream exact (%zu of %zu octets)", what, rb ? rb->len : 0, la);
    CHECK(ra && ra->len == lb && memcmp(ra->buf, sb, lb) == 0,
          "%s: B->A stream exact (%zu of %zu octets)", what, ra ? ra->len : 0, lb);
}

static void test_establish_and_bulk(void)
{
    pair_t p;
    int da, db;
    uint8_t *sa, *sb;
    size_t la, lb;
    const uint8_t olc[5] = { 'O', 'L', 'C', '-', 'A' };
    const uint8_t ack[3] = { 'A', 'C', 'K' };

    pair_init(&p, NULL, NULL);
    p.b.ack_len = 3;
    memcpy(p.b.ack_ud, ack, 3);
    da = v76_establish_req(p.a.mf, NULL, olc, 5);
    pair_run(&p, 30000);
    CHECK(da == 0, "initiator's first DLCI is 0 (6.1.1): got %d", da);
    CHECK(p.a.est_conf == 1 && p.b.est_ind == 1, "establishment completed");
    CHECK(p.b.last_est_ud_len == 5 && memcmp(p.b.last_est_ud, olc, 5) == 0, "SABME user data carried");
    CHECK(p.a.last_conf_len == 3 && memcmp(p.a.last_conf_ud, ack, 3) == 0, "UA user data carried");
    CHECK(v76_dlc_state(p.a.mf, da) == V76_DLC_CONNECTED, "A connected");

    db = v76_establish_req(p.b.mf, NULL, NULL, 0);
    pair_run(&p, 30000);
    CHECK(db == 63, "responder's first DLCI is 63 (6.1.1): got %d", db);
    CHECK(p.b.est_conf == 1 && p.a.est_ind == 1, "B-opened DLC established");

    bulk(&p, da, db, 300, 600000, &sa, &la, &sb, &lb);
    check_bulk(&p, da, db, sa, la, sb, lb, "clean line");
    CHECK(v76_stats(p.a.mf)->rej_sent == 0 && v76_stats(p.b.mf)->rej_sent == 0,
          "no REJ on a clean line");
    CHECK(v76_stats(p.a.mf)->t401_expiries == 0, "no T401 expiry on a clean line");
    free(sa); free(sb);

    /* Orderly release (7.3), and the freed DLCI is reused first (Cor.1). */
    CHECK(v76_release_req(p.a.mf, da, NULL, 0) == 0, "release request");
    pair_run(&p, 30000);
    CHECK(v76_dlc_state(p.a.mf, da) == V76_DLC_DISCONNECTED &&
          v76_dlc_state(p.b.mf, da) == V76_DLC_DISCONNECTED, "both ends disconnected");
    CHECK(p.b.rel_ind == 1 && p.b.last_rel_why == V76_REL_DISC_RECEIVED, "peer saw DISC");
    CHECK(v76_establish_req(p.a.mf, NULL, NULL, 0) == 0, "freed DLCI 0 is reused");
    pair_free(&p);
}

static void test_bit_errors(void)
{
    int sel;

    for (sel = 0; sel < 2; sel++) {
        pair_t p;
        int da, db;
        uint8_t *sa, *sb;
        size_t la, lb;
        v76_dlc_params_t dp;
        char what[64];

        rng_state = 0xC0FFEE + (uint32_t)sel;
        pair_init(&p, NULL, NULL);
        v76_dlc_params_default(&dp);
        dp.recovery = sel ? V76_REC_SREJ : V76_REC_REJ;
        p.b.accept = dp;
        p.a.accept = dp;
        da = v76_establish_req(p.a.mf, &dp, NULL, 0);
        db = v76_establish_req(p.b.mf, &dp, NULL, 0);
        pair_run(&p, 60000);
        CHECK(p.a.est_conf == 1 && p.b.est_conf == 1, "both DLCs up");
        p.ber_ab = p.ber_ba = 0xFFFFFFFFu / 3000;     /* ~3e-4 */
        bulk(&p, da, db, 400, 6000000, &sa, &la, &sb, &lb);
        snprintf(what, sizeof(what), "BER 3e-4, %s", sel ? "s-SREJ" : "REJ");
        check_bulk(&p, da, db, sa, la, sb, lb, what);
        printf("  %s: %llu bit errors injected, REJ sent %llu/%llu, SREJ sent %llu/%llu, "
               "retransmitted I %llu/%llu, T401 %llu/%llu\n", what,
               (unsigned long long)p.flips,
               (unsigned long long)v76_stats(p.a.mf)->rej_sent,
               (unsigned long long)v76_stats(p.b.mf)->rej_sent,
               (unsigned long long)v76_stats(p.a.mf)->srej_sent,
               (unsigned long long)v76_stats(p.b.mf)->srej_sent,
               (unsigned long long)v76_stats(p.a.mf)->tx_retransmitted_i,
               (unsigned long long)v76_stats(p.b.mf)->tx_retransmitted_i,
               (unsigned long long)v76_stats(p.a.mf)->t401_expiries,
               (unsigned long long)v76_stats(p.b.mf)->t401_expiries);
        CHECK(p.flips > 100, "errors were actually injected");
        CHECK(sel ? (v76_stats(p.a.mf)->srej_sent + v76_stats(p.b.mf)->srej_sent > 0)
                  : (v76_stats(p.a.mf)->rej_sent + v76_stats(p.b.mf)->rej_sent > 0),
              "the recovery procedure under test actually ran");
        CHECK(p.a.rel_ind == 0 && p.b.rel_ind == 0, "no DLC dropped");
        free(sa); free(sb);
        pair_free(&p);
    }
}

static void test_blackout(void)
{
    /* Everything vanishes for a while: only timer recovery (8.1.8) can
     * bring this back, with the last frames unacknowledged. */
    pair_t p;
    int da, db;
    uint8_t *sa, *sb;
    size_t la, lb;

    rng_state = 99;
    pair_init(&p, NULL, NULL);
    da = v76_establish_req(p.a.mf, NULL, NULL, 0);
    db = v76_establish_req(p.b.mf, NULL, NULL, 0);
    pair_run(&p, 60000);
    p.black_from = p.clock + 30000;
    p.black_to = p.black_from + 40000;                /* ~1.4 s dead line */
    bulk(&p, da, db, 120, 3000000, &sa, &la, &sb, &lb);
    check_bulk(&p, da, db, sa, la, sb, lb, "1.4 s blackout");
    CHECK(v76_stats(p.a.mf)->t401_expiries > 0, "T401 expired during the blackout");
    CHECK(p.a.rel_ind == 0 && p.b.rel_ind == 0, "link survived (N400 not reached)");
    free(sa); free(sb);
    pair_free(&p);
}

static void test_n400(void)
{
    pair_t p;
    int da;
    v76_config_t cfg;

    /* Peer silent from the start: SABME retried N400 times (7.1.2.2). */
    v76_config_default(&cfg, true);
    cfg.t401_ms = 100;
    cfg.n400 = 4;
    pair_init(&p, &cfg, NULL);
    p.black_from = 0;
    p.black_to = 100000000;
    da = v76_establish_req(p.a.mf, NULL, NULL, 0);
    pair_run(&p, 28800 / 2);                          /* 500 ms */
    CHECK(da == 0 && p.a.rel_ind == 1 && p.a.last_rel_why == V76_REL_N400,
          "establishment fails with N400 after the retries (rel_ind=%d)", p.a.rel_ind);
    CHECK(v76_dlc_state(p.a.mf, da) == V76_DLC_DISCONNECTED, "DLC freed");
    pair_free(&p);

    /* Established link, then the line dies: data cannot be acked -> N400. */
    pair_init(&p, &cfg, NULL);
    p.b.mf = p.b.mf;
    da = v76_establish_req(p.a.mf, NULL, NULL, 0);
    pair_run(&p, 20000);
    CHECK(p.a.est_conf == 1, "up before the failure");
    p.black_from = p.clock;
    p.black_to = p.clock + 100000000;
    {
        uint8_t d[10] = {1};

        v76_data_req(p.a.mf, da, d, 10);
    }
    pair_run(&p, 28800 * 2);
    CHECK(p.a.rel_ind == 1 && p.a.last_rel_why == V76_REL_N400,
          "unacknowledged data on a dead line ends in N400 (rel_ind=%d why=%d)",
          p.a.rel_ind, (int)p.a.last_rel_why);
    pair_free(&p);
}

static void test_refusal_and_collision(void)
{
    pair_t p;
    int da;

    pair_init(&p, NULL, NULL);
    p.b.auto_accept = false;
    da = v76_establish_req(p.a.mf, NULL, NULL, 0);
    pair_run(&p, 30000);
    CHECK(p.a.est_conf == 0 && p.a.rel_ind == 1 && p.a.last_rel_why == V76_REL_DM_RECEIVED,
          "refused DLC: DM -> release indication (7.1.2.1)");
    CHECK(v76_dlc_state(p.a.mf, da) == V76_DLC_DISCONNECTED, "refused DLC freed");
    pair_free(&p);

    /* The responder opens a DLC, then a SABME for that very DLCI arrives from
     * the initiator: 6.1.1 says the responder backs off. */
    {
        bitbuf_t bb;
        uint8_t f[32], ctl = 0x7F;
        int n, db;

        pair_init(&p, NULL, NULL);
        db = v76_establish_req(p.b.mf, NULL, NULL, 0);
        CHECK(db == 63, "responder picked 63");
        n = v76_build_frame(f, 1, 63, true, &ctl, 1, NULL, 0, 2, 0);
        memset(&bb, 0, sizeof(bb));
        bb_flag(&bb);
        bb_frame(&bb, f, n);
        bb_flag(&bb);
        feed(p.b.mf, &bb);
        CHECK(p.b.rel_ind == 1 && p.b.last_rel_why == V76_REL_BACKOFF,
              "responder gave way in the collision");
        CHECK(p.b.est_ind == 1, "and then took the initiator's establishment");
        pair_free(&p);
    }
}

static void test_window(void)
{
    pair_t p;
    v76_dlc_params_t dp;
    int da, i, maxun = 0;
    uint8_t d[100];

    pair_init(&p, NULL, NULL);
    v76_dlc_params_default(&dp);
    dp.k = 3;
    p.b.accept = dp;
    da = v76_establish_req(p.a.mf, &dp, NULL, 0);
    pair_run(&p, 30000);
    memset(d, 7, sizeof(d));
    for (i = 0; i < 60; i++)
        while (!v76_data_req(p.a.mf, da, d, 100))
            pair_run(&p, 100);
    for (i = 0; i < 4000; i++) {
        int u;

        pair_run(&p, 50);
        u = v76_unacked_frames(p.a.mf, da);
        if (u > maxun)
            maxun = u;
    }
    CHECK(maxun <= 3, "never more than k=3 unacknowledged I frames (saw %d)", maxun);
    CHECK(maxun == 3, "and the window was actually filled (saw %d)", maxun);
    {
        stream_t *s = get_stream(&p.b, da);

        CHECK(s && s->len == 6000, "all delivered with a small window");
    }
    pair_free(&p);
}

static void test_busy(void)
{
    pair_t p;
    int da, i;
    uint8_t d[50];

    pair_init(&p, NULL, NULL);
    da = v76_establish_req(p.a.mf, NULL, NULL, 0);
    pair_run(&p, 30000);
    memset(d, 3, sizeof(d));
    v76_set_busy(p.b.mf, da, true);
    for (i = 0; i < 40; i++)
        while (!v76_data_req(p.a.mf, da, d, 50))
            pair_run(&p, 100);
    pair_run(&p, 200000);
    {
        stream_t *s = get_stream(&p.b, da);

        CHECK(!s || s->len == 0, "nothing delivered while the receiver is busy (8.1.7)");
    }
    CHECK(v76_data_backlog(p.a.mf, da) + v76_unacked_frames(p.a.mf, da) > 0,
          "sender is holding data back (RNR)");
    CHECK(p.a.rel_ind == 0, "RNR polling kept the link up");
    v76_set_busy(p.b.mf, da, false);
    pair_run(&p, 600000);
    {
        stream_t *s = get_stream(&p.b, da);

        CHECK(s && s->len == 2000, "everything arrives once the busy condition clears (%zu)",
              s ? s->len : 0);
    }
    pair_free(&p);
}

static void test_fcs_variants_and_addr(void)
{
    pair_t p;
    v76_config_t ca, cb;
    int fl[3] = { V76_FCS_8, V76_FCS_16, V76_FCS_32 };
    int dl[3], i;
    uint8_t *sa, *sb;
    size_t la, lb;

    v76_config_default(&ca, true);
    v76_config_default(&cb, false);
    ca.fcs_support = cb.fcs_support = V76_FCS_MASK_8 | V76_FCS_MASK_16 | V76_FCS_MASK_32;
    pair_init(&p, &ca, &cb);
    for (i = 0; i < 3; i++) {
        v76_dlc_params_t dp;

        v76_dlc_params_default(&dp);
        dp.fcs_len = fl[i];
        dl[i] = v76_establish_req(p.a.mf, &dp, NULL, 0);
        pair_run(&p, 20000);
        CHECK(p.a.est_conf == i + 1, "DLC with a %d-bit FCS established", fl[i] * 8);
    }
    for (i = 0; i < 3; i++) {
        v76_dlc_params_t q;

        CHECK(v76_dlc_params(p.b.mf, dl[i], &q) && q.fcs_len == fl[i],
              "acceptor learned the %d-bit FCS from the SABME", fl[i] * 8);
    }
    /* Data on each. */
    for (i = 0; i < 3; i++) {
        uint8_t d[80];
        int k;

        for (k = 0; k < 80; k++)
            d[k] = (uint8_t)(rnd());
        v76_data_req(p.a.mf, dl[i], d, 80);
        pair_run(&p, 20000);
        {
            stream_t *s = get_stream(&p.b, dl[i]);

            CHECK(s && s->len == 80 && memcmp(s->buf, d, 80) == 0,
                  "data over a %d-bit FCS DLC", fl[i] * 8);
        }
    }
    pair_free(&p);

    /* Two-octet addresses: DLCI space 0..8191, responder allocates downward. */
    {
        v76_dlc_params_t dp;
        int da, db;

        v76_config_default(&ca, true);
        v76_config_default(&cb, false);
        ca.addr_octets = cb.addr_octets = 2;
        pair_init(&p, &ca, &cb);
        v76_dlc_params_default(&dp);
        dp.addr_octets = 2;
        da = v76_establish_req(p.a.mf, &dp, NULL, 0);
        db = v76_establish_req(p.b.mf, &dp, NULL, 0);
        pair_run(&p, 40000);
        CHECK(da == 0 && db == 8191, "2-octet DLCIs: initiator %d, responder %d", da, db);
        CHECK(p.a.est_conf == 1 && p.b.est_conf == 1, "2-octet-address DLCs established");
        bulk(&p, da, db, 50, 300000, &sa, &la, &sb, &lb);
        check_bulk(&p, da, db, sa, la, sb, lb, "2-octet address");
        free(sa); free(sb);
        pair_free(&p);
    }
}

static void test_unerm_and_uih(void)
{
    pair_t p;
    v76_dlc_params_t dp;
    int da, i;

    pair_init(&p, NULL, NULL);
    v76_dlc_params_default(&dp);
    dp.mode = V76_UNERM;
    p.b.accept = dp;
    da = v76_establish_req(p.a.mf, &dp, NULL, 0);
    pair_run(&p, 30000);
    CHECK(p.a.est_conf == 1, "UNERM DLC up");
    for (i = 0; i < 50; i++) {
        uint8_t d[20];

        memset(d, i, sizeof(d));
        CHECK(v76_unitdata_req(p.a.mf, da, d, 20), "UI queued");
    }
    CHECK(!v76_data_req(p.a.mf, da, (const uint8_t *)"x", 1), "L-DATA is refused on UNERM");
    pair_run(&p, 100000);
    CHECK(p.b.ui_count == 50, "all 50 UI frames delivered (%d)", p.b.ui_count);
    CHECK(!p.b.last_ui_resp && p.b.last_ui[0] == 49, "UI is a command with the right data");
    {
        /* On a lossy line UI is not recovered: what arrives is intact but
         * incomplete. */
        int before = p.b.ui_count, bad = 0;

        p.ber_ab = 0xFFFFFFFFu / 400;
        for (i = 0; i < 200; i++) {
            uint8_t d[20];

            memset(d, 0x55, sizeof(d));
            v76_unitdata_req(p.a.mf, da, d, 20);
            pair_run(&p, 800);
            if (p.b.last_ui_len == 20) {
                int k;

                for (k = 0; k < 20; k++)
                    if (p.b.last_ui[k] != 0x55 && p.b.last_ui[k] != 49)
                        bad++;
            }
        }
        pair_run(&p, 20000);
        CHECK(bad == 0, "no corrupted UI payload ever delivered");
        CHECK(p.b.ui_count - before < 200 && p.b.ui_count - before > 100,
              "UI frames were lost, not retransmitted (%d of 200 arrived)", p.b.ui_count - before);
    }
    pair_free(&p);

    /* UIH (App. II): FCS over the first four octets only, so damage after
     * them is delivered, flagged as header-only. */
    {
        bitbuf_t bb;
        uint8_t f[80], ctl = 0xEF;
        uint8_t info[40];
        int n;

        pair_init(&p, NULL, NULL);
        v76_dlc_params_default(&dp);
        dp.mode = V76_UNERM;
        dp.uih = true;
        dp.uih_protect = 4;
        p.b.accept = dp;
        da = v76_establish_req(p.a.mf, &dp, NULL, 0);
        pair_run(&p, 30000);
        CHECK(p.a.est_conf == 1, "UIH DLC up");
        memset(info, 0xA5, sizeof(info));
        n = v76_build_frame(f, 1, da, true, &ctl, 1, info, 40, 2, 4);
        f[20] ^= 0x01;                                  /* damage beyond the protected 4 */
        memset(&bb, 0, sizeof(bb));
        bb_flag(&bb);
        bb_frame(&bb, f, n);
        bb_flag(&bb);
        feed(p.b.mf, &bb);
        CHECK(p.b.ui_count == 1 && p.b.ui_hdr_only == 1,
              "UIH with damage outside the protected octets is delivered, flagged");
        f[2] ^= 0x01;                                   /* now inside the header */
        memset(&bb, 0, sizeof(bb));
        bb_flag(&bb);
        bb_frame(&bb, f, n);
        bb_flag(&bb);
        feed(p.b.mf, &bb);
        CHECK(p.b.ui_count == 1, "UIH with damage inside the header is dropped");
        pair_free(&p);
    }
}

static void test_xid_test(void)
{
    pair_t p;
    int da;
    const uint8_t x[6] = { 1, 2, 3, 4, 5, 6 };

    pair_init(&p, NULL, NULL);
    da = v76_establish_req(p.a.mf, NULL, NULL, 0);
    pair_run(&p, 30000);
    CHECK(v76_setparm_req(p.a.mf, da, x, 6) == 0, "XID request");
    pair_run(&p, 20000);
    CHECK(p.b.setparm_ind == 1 && p.a.setparm_conf == 1, "XID command and response exchanged");
    CHECK(p.a.last_xid_len == 6 && memcmp(p.a.last_xid, x, 6) == 0, "XID user data echoed back");
    CHECK(v76_test_req(p.a.mf, da, x, 6) == 0, "TEST request");
    pair_run(&p, 20000);
    CHECK(p.b.test_ind == 1, "TEST indication");
    pair_free(&p);
}

/* ---------------------------------------------------------------------- *
 * Suspend / resume
 * ---------------------------------------------------------------------- */

typedef struct {
    long lat_max_bits;
    int voice_got, voice_sent, voice_bad;
    bool data_ok;
    uint64_t suspends, resumes, violations;
} sr_result_t;

static sr_result_t run_voice_and_data(bool sr, bool with_addr, int n401_rt, long ber_div)
{
    pair_t p;
    v76_config_t ca, cb;
    v76_dlc_params_t dd, dv;
    sr_result_t r;
    int dd_id, dv_id, i, seq = 0;
    uint8_t *sent;
    size_t sent_len = 0;
    long frame_period = 288;                          /* 10 ms at 28.8 kbit/s */

    memset(&r, 0, sizeof(r));
    v76_config_default(&ca, true);
    v76_config_default(&cb, false);
    ca.suspend_resume = cb.suspend_resume = sr;
    ca.sr_with_address = cb.sr_with_address = with_addr;
    ca.fcs_support = cb.fcs_support = V76_FCS_MASK_8 | V76_FCS_MASK_16;
    pair_init(&p, &ca, &cb);

    v76_dlc_params_default(&dd);
    dd.n401_tx = dd.n401_rx = 1000;
    v76_dlc_params_default(&dv);
    dv.mode = V76_UNERM;
    dv.realtime = sr;
    dv.fcs_len = V76_FCS_8;
    dv.n401_tx = dv.n401_rx = n401_rt;
    dv.n401_rt = n401_rt;
    p.b.accept = dd;
    dd_id = v76_establish_req(p.a.mf, &dd, NULL, 0);
    pair_run(&p, 20000);
    p.b.accept = dv;
    dv_id = v76_establish_req(p.a.mf, &dv, NULL, 0);
    pair_run(&p, 20000);
    if (p.a.est_conf != 2) {
        r.voice_bad = -1;
        pair_free(&p);
        return r;
    }
    p.b.voice_dlci = dv_id;
    p.b.clock = &p.clock;
    p.b.voice_next = 0;
    sent = malloc(100 * 1000);
    p.ber_ab = ber_div ? 0xFFFFFFFFu / (uint32_t)ber_div : 0;

    for (i = 0; i < 100; i++) {
        long t_end = p.clock + frame_period * 3;
        int k;

        /* Keep the data queue full of long frames. */
        while (v76_data_backlog(p.a.mf, dd_id) < 4 && sent_len + 1000 < 100 * 1000) {
            uint8_t d[1000];

            for (k = 0; k < 1000; k++)
                d[k] = (uint8_t)rnd();
            if (!v76_data_req(p.a.mf, dd_id, d, 1000))
                break;
            memcpy(sent + sent_len, d, 1000);
            sent_len += 1000;
        }
        while (p.clock < t_end) {
            if (p.clock % frame_period == 0 && seq < 300) {
                uint8_t v[12];
                int q;

                v[0] = (uint8_t)seq; v[1] = (uint8_t)(seq >> 8);
                v[2] = (uint8_t)(seq >> 16); v[3] = (uint8_t)(seq >> 24);
                for (q = 4; q < 12; q++)
                    v[q] = (uint8_t)(seq * 7 + q);
                p.b.enq_bits[seq] = p.clock;
                v76_unitdata_req(p.a.mf, dv_id, v, 12);
                seq++;
            }
            pair_run(&p, 1);
        }
        if (seq >= 300)
            break;
    }
    pair_run(&p, 400000);
    r.voice_sent = seq;
    r.voice_got = p.b.lat_n;
    r.voice_bad = p.b.voice_bad;
    r.lat_max_bits = p.b.lat_max;
    {
        stream_t *s = get_stream(&p.b, dd_id);

        r.data_ok = s && s->len <= sent_len && memcmp(s->buf, sent, s->len) == 0 &&
                    s->len > 20000;
        if (s)
            printf("  S/R %s addr=%d: data delivered %zu of %zu octets\n", sr ? "on " : "off",
                   with_addr, s->len, sent_len);
    }
    r.suspends = v76_stats(p.a.mf)->sr_suspends;
    r.resumes = v76_stats(p.b.mf)->sr_resumes;
    r.violations = v76_stats(p.b.mf)->sr_violations + v76_stats(p.a.mf)->sr_violations;
    free(sent);
    pair_free(&p);
    return r;
}

static void test_suspend_resume(void)
{
    sr_result_t off, on, on_addr, on_max, on_err;

    rng_state = 4242;
    off = run_voice_and_data(false, false, 12, 0);
    rng_state = 4242;
    on = run_voice_and_data(true, false, 12, 0);
    rng_state = 4242;
    on_addr = run_voice_and_data(true, true, 12, 0);
    rng_state = 4242;
    on_max = run_voice_and_data(true, false, 12, 0);    /* RT frames of exactly N401RT */
    rng_state = 4242;
    on_err = run_voice_and_data(true, true, 12, 20000);

    printf("  worst voice-frame latency: S/R off %ld bits (%.1f ms), on %ld bits (%.1f ms), "
           "on+addr %ld bits\n", off.lat_max_bits, off.lat_max_bits / 28.8,
           on.lat_max_bits, on.lat_max_bits / 28.8, on_addr.lat_max_bits);
    CHECK(off.voice_got == off.voice_sent && off.voice_bad == 0,
          "S/R off: all %d voice frames, in order (%d)", off.voice_sent, off.voice_got);
    CHECK(on.voice_got == on.voice_sent && on.voice_bad == 0,
          "S/R on: all %d voice frames, in order (%d got, %d out of order)",
          on.voice_sent, on.voice_got, on.voice_bad);
    CHECK(on_addr.voice_got == on_addr.voice_sent && on_addr.voice_bad == 0,
          "S/R on with address: all voice frames in order (%d of %d)",
          on_addr.voice_got, on_addr.voice_sent);
    CHECK(off.data_ok && on.data_ok && on_addr.data_ok,
          "data delivered intact next to voice, with and without S/R");
    CHECK(on.suspends > 20 && on.resumes > 20, "S/R really interrupted data frames (%llu suspends)",
          (unsigned long long)on.suspends);
    CHECK(on.violations == 0 && on_addr.violations == 0, "no S/R protocol violations");
    CHECK(off.lat_max_bits > 3000, "without S/R voice waits behind long data frames (%ld bits)",
          off.lat_max_bits);
    CHECK(on.lat_max_bits < off.lat_max_bits / 4 && on.lat_max_bits < 1500,
          "S/R cuts the worst voice latency from %ld to %ld bits", off.lat_max_bits, on.lat_max_bits);
    CHECK(on_max.voice_got == on_max.voice_sent && on_max.voice_bad == 0 && on_max.data_ok,
          "max-length RT frames (no resume flag) still correct");
    CHECK(on_err.voice_bad == 0 && on_err.data_ok && on_err.voice_got > on_err.voice_sent / 2,
          "S/R with bit errors: data intact, voice partly lost but never reordered "
          "(%d of %d, bad=%d)", on_err.voice_got, on_err.voice_sent, on_err.voice_bad);
}

int main(void)
{
    test_fcs();
    test_handbuilt();
    test_invalid_and_abort();
    test_zero_insertion();
    test_establish_and_bulk();
    test_bit_errors();
    test_blackout();
    test_n400();
    test_refusal_and_collision();
    test_window();
    test_busy();
    test_fcs_variants_and_addr();
    test_unerm_and_uih();
    test_xid_test();
    test_suspend_resume();

    printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
