/*
 * v8bis_test.c -- tests for the V.8bis stage 1 modules: framing (v8bis_msg),
 * the information field codec (v8bis_ie) and the tone signals (v8bis_tones).
 *
 * The framing is graded against SpanDSP's HDLC in both directions and against
 * a bit-serial reference written here; the codec against hand-assembled octets
 * built from Tables 5-6 of the Recommendation; the tones against a signal
 * generator and interferers that are local to this file, so that the module
 * under test never grades itself.
 */
#include <math.h>
#include <spandsp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v8bis_ie.h"
#include "v8bis_msg.h"
#include "v8bis_tones.h"

static int g_fail, g_checks;
static int g_quick;              /* V8BIS_TEST_QUICK=1: fewer iterations, for sanitizer builds */

#define CHECK(c)                                                                         \
    do {                                                                                 \
        g_checks++;                                                                      \
        if (!(c)) {                                                                      \
            if (++g_fail <= 12)                                                          \
                printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);                    \
        }                                                                                \
    } while (0)

static uint32_t g_rng = 0x2545f491u;
static uint32_t rnd(void)
{
    g_rng ^= g_rng << 13;
    g_rng ^= g_rng >> 17;
    g_rng ^= g_rng << 5;
    return g_rng;
}
static double urand(void) { return (rnd() >> 8) / 16777216.0; }
static double gauss(void)
{
    double s = 0;
    for (int i = 0; i < 12; i++)
        s += urand();
    return s - 6.0;
}

/* ====================================================================== */
/* framing                                                                */
/* ====================================================================== */

static unsigned bitrev16(unsigned v)
{
    unsigned r = 0;
    for (int i = 0; i < 16; i++)
        if (v & (1u << i))
            r |= 1u << (15 - i);
    return r;
}

/* Reference residue check, the reflected way round: the receiver's register
 * after the information bits AND the FCS is 0001110100001111 (7.2.7). */
static unsigned ref_residue(const uint8_t *info, size_t len, uint16_t fcs)
{
    unsigned reg = 0xffff;                       /* x^15 in bit 15 */
    for (size_t i = 0; i < len + 2; i++) {
        for (int b = 0; b < 8; b++) {
            unsigned bit;
            if (i < len)
                bit = (info[i] >> b) & 1u;
            else
                bit = (fcs >> (15 - (int)((i - len) * 8 + b))) & 1u;  /* MSB first */
            bit ^= (reg >> 15) & 1u;
            reg = (reg << 1) & 0xffff;
            if (bit)
                reg ^= 0x1021;
        }
    }
    return reg;
}

typedef struct {
    int ok, bad;
    uint8_t last[80];
    size_t last_len;
    v8bis_frame_status_t last_bad;
    int by_status[5];
} rxlog_t;

static void our_cb(void *user, const uint8_t *info, size_t len, v8bis_frame_status_t st)
{
    rxlog_t *l = user;
    l->by_status[st]++;
    if (st == V8BIS_FRAME_OK) {
        l->ok++;
        l->last_len = len;
        memcpy(l->last, info, len);
    } else {
        l->bad++;
        l->last_bad = st;
    }
}

static void feed_bits(v8bis_frame_rx_t *rx, const uint8_t *bits, size_t n)
{
    for (size_t i = 0; i < n; i++)
        v8bis_frame_rx_bit(rx, bits[i]);
}

typedef struct {
    int ok;
    uint8_t pkt[80];
    int len;
} sp_rx_t;

static void sp_cb(void *user, const uint8_t *pkt, int len, int ok)
{
    sp_rx_t *s = user;
    if (ok && len > 0 && len <= 80) {
        memcpy(s->pkt, pkt, (size_t)len);
        s->len = len;
        s->ok++;
    }
}

/* Raw framing with no FCS, for building invalid frames. */
static size_t raw_frame(const uint8_t *bits_in, size_t nbits, uint8_t *out)
{
    static const uint8_t F[8] = {0, 1, 1, 1, 1, 1, 1, 0};
    size_t n = 0;
    unsigned ones = 0;
    for (int k = 0; k < 2; k++)
        for (int i = 0; i < 8; i++)
            out[n++] = F[i];
    for (size_t i = 0; i < nbits; i++) {
        out[n++] = bits_in[i];
        if (bits_in[i]) {
            if (++ones == 5) {
                out[n++] = 0;
                ones = 0;
            }
        } else {
            ones = 0;
        }
    }
    for (int i = 0; i < 8; i++)
        out[n++] = F[i];
    return n;
}

static void test_framing(void)
{
    static const uint8_t KAT[] = "123456789";
    uint8_t bits[V8BIS_MAX_FRAME_BITS + 400];
    uint8_t info[V8BIS_MAX_INFO_OCTETS];
    rxlog_t log;
    v8bis_frame_rx_t rx;

    printf("framing\n");
    /* CRC-16/X-25 check value is 0x906E for "123456789" with the register
     * reflected; ours has x^15 in bit 15, so it is the bit reversal. */
    CHECK(v8bis_fcs(KAT, 9) == bitrev16(0x906e));
    {
        uint8_t m[20];
        for (int t = 0; t < 200; t++) {
            size_t n = 1 + rnd() % 20;
            for (size_t i = 0; i < n; i++)
                m[i] = (uint8_t)rnd();
            CHECK(ref_residue(m, n, v8bis_fcs(m, n)) == 0x1d0f);        /* 7.2.7 */
        }
        /* SpanDSP's register, same polynomial, reflected: agree on every input */
        int agree = 0;
        for (int t = 0; t < 200; t++) {
            size_t n = 1 + rnd() % 20;
            uint16_t sp;
            for (size_t i = 0; i < n; i++)
                m[i] = (uint8_t)rnd();
            sp = crc_itu16_calc(m, (int)n, 0xffff);
            /* crc_itu16_calc returns the register before complement, reflected */
            if (v8bis_fcs(m, n) == (uint16_t)bitrev16((uint16_t)~sp))
                agree++;
        }
        CHECK(agree == 200);
    }

    /* ours -> SpanDSP's receiver */
    for (int t = 0; t < 300; t++) {
        size_t n = 1 + rnd() % V8BIS_MAX_INFO_OCTETS;
        size_t nb;
        hdlc_rx_state_t *sprx;
        sp_rx_t s = {0, {0}, 0};
        for (size_t i = 0; i < n; i++) {
            unsigned c = rnd() % 4;
            info[i] = c == 0 ? 0x7e : c == 1 ? 0xff : c == 2 ? 0x00 : (uint8_t)rnd();
        }
        nb = v8bis_frame_encode(info, n, V8BIS_PREAMBLE_BITS, 2 + rnd() % 4, 1 + rnd() % 3,
                                bits, sizeof(bits));
        CHECK(nb > 0);
        sprx = hdlc_rx_init(NULL, false, false, 1, sp_cb, &s);
        for (size_t i = 0; i < nb; i++)
            hdlc_rx_put_bit(sprx, bits[i]);
        hdlc_rx_free(sprx);
        CHECK(s.ok == 1 && (size_t)s.len == n && !memcmp(s.pkt, info, n));
    }

    /* SpanDSP's transmitter -> ours */
    for (int t = 0; t < 300; t++) {
        size_t n = 3 + rnd() % 60;
        hdlc_tx_state_t *sptx;
        for (size_t i = 0; i < n; i++)
            info[i] = (uint8_t)rnd();
        info[0] = t % 3 ? info[0] : 0xff;
        sptx = hdlc_tx_init(NULL, false, 2, false, NULL, NULL);
        hdlc_tx_flags(sptx, 8);
        hdlc_tx_frame(sptx, info, n);
        memset(&log, 0, sizeof(log));
        v8bis_frame_rx_init(&rx, our_cb, &log);
        for (int i = 0; i < 120 + (int)n * 12; i++)
            v8bis_frame_rx_bit(&rx, hdlc_tx_get_bit(sptx));
        hdlc_tx_free(sptx);
        CHECK(log.ok == 1 && log.bad == 0 && log.last_len == n && !memcmp(log.last, info, n));
    }

    /* ours -> ours, flag counts at the limits, and ABORT of marking before a frame */
    for (unsigned of = 2; of <= 5; of++)
        for (unsigned cf = 1; cf <= 3; cf++) {
            size_t nb;
            memset(&log, 0, sizeof(log));
            v8bis_frame_rx_init(&rx, our_cb, &log);
            for (int i = 0; i < 5; i++)
                info[i] = (uint8_t)(0x11 * (i + 1));
            nb = v8bis_frame_encode(info, 5, V8BIS_PREAMBLE_BITS, of, cf, bits, sizeof(bits));
            feed_bits(&rx, bits, nb);
            CHECK(log.ok == 1 && log.bad == 0 && log.last_len == 5 && !memcmp(log.last, info, 5));
        }
    CHECK(v8bis_frame_encode(info, 5, 0, 1, 1, bits, sizeof(bits)) == 0);   /* 7.2.5: 2-5 */
    CHECK(v8bis_frame_encode(info, 5, 0, 6, 1, bits, sizeof(bits)) == 0);
    CHECK(v8bis_frame_encode(info, 5, 0, 2, 0, bits, sizeof(bits)) == 0);   /* and 1-3 */
    CHECK(v8bis_frame_encode(info, 5, 0, 2, 4, bits, sizeof(bits)) == 0);
    CHECK(v8bis_frame_encode(info, 0, 0, 2, 1, bits, sizeof(bits)) == 0);
    CHECK(v8bis_frame_encode(info, 65, 0, 2, 1, bits, sizeof(bits)) == 0);  /* 8.6: 64 */
    CHECK(v8bis_frame_encode(info, 5, 0, 2, 1, bits, 20) == 0);

    /* a single flag shared between back to back frames, and a shared zero */
    {
        uint8_t a[3] = {0x14, 0x01, 0x02}, b[3] = {0x18, 0x03, 0x04}, f1[600], f2[600];
        size_t n1 = v8bis_frame_encode(a, 3, 0, 2, 1, f1, sizeof(f1));
        size_t n2 = v8bis_frame_encode(b, 3, 0, 2, 1, f2, sizeof(f2));
        memset(&log, 0, sizeof(log));
        v8bis_frame_rx_init(&rx, our_cb, &log);
        feed_bits(&rx, f1, n1 - 0);
        feed_bits(&rx, f2 + 8, n2 - 8);          /* the second frame's first flag is dropped */
        CHECK(log.ok == 2 && log.bad == 0);
        memset(&log, 0, sizeof(log));
        v8bis_frame_rx_init(&rx, our_cb, &log);
        feed_bits(&rx, f1, n1);
        feed_bits(&rx, f2 + 1, n2 - 1);          /* the closing flag's zero doubles as the opening one */
        CHECK(log.ok == 2 && log.bad == 0);
    }

    /* every single bit error in a frame is caught (never an OK) */
    {
        size_t nb, body_start, body_end;
        int accepted = 0;
        for (int i = 0; i < 20; i++)
            info[i] = (uint8_t)rnd();
        nb = v8bis_frame_encode(info, 20, 0, 2, 1, bits, sizeof(bits));
        body_start = 16;
        body_end = nb - 8;
        for (size_t k = body_start; k < body_end; k++) {
            uint8_t cp[V8BIS_MAX_FRAME_BITS];
            memcpy(cp, bits, nb);
            cp[k] ^= 1;
            memset(&log, 0, sizeof(log));
            v8bis_frame_rx_init(&rx, our_cb, &log);
            feed_bits(&rx, cp, nb);
            accepted += log.ok;
        }
        CHECK(accepted == 0);
    }

    /* 7.2.9: short, misaligned, FCS error, abort */
    {
        uint8_t raw[400], fr[600];
        size_t n;

        for (int nbits = 0; nbits <= 40; nbits++) {
            for (int i = 0; i < nbits; i++)
                raw[i] = (uint8_t)(rnd() & 1);
            n = raw_frame(raw, (size_t)nbits, fr);
            memset(&log, 0, sizeof(log));
            v8bis_frame_rx_init(&rx, our_cb, &log);
            feed_bits(&rx, fr, n);
            if (nbits == 0)
                CHECK(log.ok == 0 && log.bad == 0);
            else if (nbits % 8)
                CHECK(log.by_status[V8BIS_FRAME_ALIGN] == 1 && log.ok == 0);
            else if (nbits / 8 < 3)
                CHECK(log.by_status[V8BIS_FRAME_SHORT] == 1 && log.ok == 0);
            else
                CHECK(log.by_status[V8BIS_FRAME_FCS] == 1 && log.ok == 0);
        }
        /* abort: seven ones inside a frame */
        n = 0;
        memset(raw, 1, 7);
        for (int i = 0; i < 8; i++)
            raw[7 + i] = (uint8_t)(rnd() & 1);
        n = raw_frame(raw, 15, fr);       /* the stuffing would break up the ones; */
        (void)n;
        {
            static const uint8_t F[8] = {0, 1, 1, 1, 1, 1, 1, 0};
            size_t m = 0;
            for (int i = 0; i < 8; i++) fr[m++] = F[i];
            for (int i = 0; i < 12; i++) fr[m++] = (uint8_t)(i & 1);
            for (int i = 0; i < 8; i++) fr[m++] = 1;          /* abort */
            memset(&log, 0, sizeof(log));
            v8bis_frame_rx_init(&rx, our_cb, &log);
            feed_bits(&rx, fr, m);
            CHECK(log.by_status[V8BIS_FRAME_ABORT] == 1 && log.ok == 0);
        }
        /* marking before the preamble flags and a frame afterwards still decodes */
        memset(&log, 0, sizeof(log));
        v8bis_frame_rx_init(&rx, our_cb, &log);
        for (int i = 0; i < 100; i++)
            v8bis_frame_rx_bit(&rx, 1);
        info[0] = 0x14;
        n = v8bis_frame_encode(info, 1, V8BIS_PREAMBLE_BITS, 3, 2, bits, sizeof(bits));
        feed_bits(&rx, bits, n);
        CHECK(log.ok == 1 && log.last_len == 1 && log.last[0] == 0x14);
    }

    /* random bit noise never yields an OK frame and never crashes */
    {
        int ok = 0;
        memset(&log, 0, sizeof(log));
        v8bis_frame_rx_init(&rx, our_cb, &log);
        for (int i = 0; i < (g_quick ? 200000 : 4000000); i++)
            v8bis_frame_rx_bit(&rx, (int)(rnd() & 1));
        ok = log.ok;
        printf("  random bits: %d frames passed the FCS (chance ~ 2^-16 per candidate)\n", ok);
        CHECK(ok <= 3);
    }
}

/* ====================================================================== */
/* information field                                                      */
/* ====================================================================== */

static void expect_bytes(const v8bis_msg_t *m, const uint8_t *want, size_t n, const char *what)
{
    uint8_t out[V8BIS_MAX_INFO_OCTETS];
    int r = v8bis_msg_encode(m, out, sizeof(out));
    int same = r == (int)n && !memcmp(out, want, n);
    g_checks++;
    if (!same) {
        g_fail++;
        printf("  FAIL golden %s: got %d octets:", what, r);
        for (int i = 0; i < r; i++)
            printf(" %02x", out[i]);
        printf("\n");
    }
}

static bool same_fields(const v8bis_msg_t *a, const v8bis_msg_t *b)
{
    if (a->type != b->type || a->revision != b->revision || a->id_npar1 != b->id_npar1
        || a->network_type != b->network_type || a->network_npar2 != b->network_npar2
        || a->s_npar1 != b->s_npar1 || a->s_spar1 != b->s_spar1
        || memcmp(a->data, b->data, 3) || memcmp(a->svd, b->svd, 3)
        || a->h324_npar2 != b->h324_npar2 || a->h324_spar2 != b->h324_spar2
        || a->h324_data != b->h324_data || a->v18 != b->v18 || a->analogue_tel != b->analogue_tel
        || a->t101 != b->t101 || a->ns_count != b->ns_count)
        return false;
    for (unsigned i = 0; i < a->ns_count; i++) {
        const v8bis_ns_block_t *x = &a->ns[i], *y = &b->ns[i];
        if (x->country != y->country || x->provider_len != y->provider_len
            || x->data_len != y->data_len || memcmp(x->provider, y->provider, x->provider_len)
            || memcmp(x->data, y->data, x->data_len))
            return false;
    }
    return true;
}

static void random_msg(v8bis_msg_t *m)
{
    static const unsigned types[] = {V8BIS_MT_MS, V8BIS_MT_CL, V8BIS_MT_CLR};

    v8bis_msg_init(m, types[rnd() % 3]);
    m->id_npar1 = (uint8_t)(rnd() & (V8BIS_ID_V8 | V8BIS_ID_SHORT_V8 | V8BIS_ID_MORE_INFO | V8BIS_ID_TX_ACK1));
    m->network_type = rnd() & 1;
    if (m->network_type)
        m->network_npar2 = (uint8_t)(rnd() & 0x23);
    m->s_spar1 = (uint8_t)(rnd() & 0x6f);
    if (m->s_spar1 & V8BIS_S_DATA) {
        m->data[0] = (uint8_t)(rnd() & 0x3f);
        m->data[1] = (uint8_t)(rnd() & 0x37);
        m->data[2] = (uint8_t)(rnd() & 0x0f);
    }
    if (m->s_spar1 & V8BIS_S_SVD) {
        m->svd[0] = (uint8_t)(rnd() & 0x3b);
        m->svd[1] = (uint8_t)(rnd() & 0x3f);
        m->svd[2] = (uint8_t)(rnd() & 0x03);
    }
    if (m->s_spar1 & V8BIS_S_H324) {
        m->h324_npar2 = (uint8_t)(rnd() & 0x27);
        if (rnd() & 1) {
            m->h324_spar2 = V8BIS_H324_SPAR2_DATA;
            m->h324_data = (uint8_t)(rnd() & 0x3f);
        }
    }
    if (m->s_spar1 & V8BIS_S_V18) m->v18 = (uint8_t)(rnd() & 0x23);
    if (m->s_spar1 & V8BIS_S_ANALOGUE_TEL) m->analogue_tel = (uint8_t)(rnd() & 0x27);
    if (m->s_spar1 & V8BIS_S_T101) m->t101 = (uint8_t)(rnd() & 0x27);
    if (rnd() % 4 == 0) {
        m->id_npar1 |= V8BIS_ID_NON_STANDARD;
        m->ns_count = 1 + rnd() % 2;
        for (unsigned i = 0; i < m->ns_count; i++) {
            m->ns[i].country = (uint8_t)rnd();
            m->ns[i].provider_len = (uint8_t)(rnd() % 4);
            m->ns[i].data_len = (uint8_t)(rnd() % 6);
            for (int k = 0; k < 8; k++) m->ns[i].provider[k] = (uint8_t)rnd();
            for (int k = 0; k < 10; k++) m->ns[i].data[k] = (uint8_t)rnd();
            if (m->ns[i].provider_len > 4) m->ns[i].provider_len = 4;
        }
    }
}

static bool type_check_clean(const v8bis_msg_t *d)
{
    return !d->reserved_bits && !d->delimiter_anomaly && !d->extra_octets && !d->ignored_blocks
           && !d->ns_dropped;
}

static void test_ie(void)
{
    v8bis_msg_t m, d;
    uint8_t buf[V8BIS_MAX_INFO_OCTETS], buf2[V8BIS_MAX_INFO_OCTETS];

    printf("information field\n");

    /* Assembled by hand from Tables 5-1, 6-2 and 6-3 and Figure 7. */
    v8bis_msg_init(&m, V8BIS_MT_MS);
    m.id_npar1 = V8BIS_ID_V8 | V8BIS_ID_TX_ACK1;
    m.s_spar1 = V8BIS_S_DATA;
    m.data[1] = V8BIS_DATA2_V34;
    {
        const uint8_t want[] = {0x11, 0x89, 0x80, 0x80, 0x81, 0x00, 0xd0};
        expect_bytes(&m, want, sizeof(want), "MS data V.34");
    }

    v8bis_msg_init(&m, V8BIS_MT_CL);
    m.id_npar1 = V8BIS_ID_V8;
    m.s_spar1 = V8BIS_S_DATA;
    m.data[0] = V8BIS_DATA_TRANSPARENT | V8BIS_DATA_V42 | V8BIS_DATA_V42BIS;
    m.data[1] = V8BIS_DATA2_V34 | V8BIS_DATA2_V32BIS;
    m.data[2] = V8BIS_DATA3_V32 | V8BIS_DATA3_V22BIS | V8BIS_DATA3_V22 | V8BIS_DATA3_V21;
    {
        const uint8_t want[] = {0x12, 0x81, 0x80, 0x80, 0x81, 0x07, 0x30, 0xcf};
        expect_bytes(&m, want, sizeof(want), "CL data");
    }

    v8bis_msg_init(&m, V8BIS_MT_MS);                       /* H.324 with a Data SPar(2) */
    m.s_spar1 = V8BIS_S_H324;
    m.h324_npar2 = V8BIS_H324_VIDEO | V8BIS_H324_AUDIO;
    m.h324_spar2 = V8BIS_H324_SPAR2_DATA;
    m.h324_data = V8BIS_H324D_V42 | V8BIS_H324D_PPP;
    {
        const uint8_t want[] = {0x11, 0x80, 0x80, 0x80, 0x84, 0x43, 0x41, 0xc5};
        expect_bytes(&m, want, sizeof(want), "MS H.324 data");
    }

    v8bis_msg_init(&m, V8BIS_MT_CLR);                      /* network type block */
    m.network_type = true;
    m.network_npar2 = V8BIS_NET_CELLULAR;
    m.s_spar1 = V8BIS_S_ANALOGUE_TEL | V8BIS_S_V18;
    m.analogue_tel = V8BIS_TEL_VOICE;
    m.v18 = V8BIS_V18_V21;
    {
        const uint8_t want[] = {0x13, 0x80, 0x81, 0xc1, 0x80, 0xa8, 0xc1, 0xc1};
        expect_bytes(&m, want, sizeof(want), "CLR network + V.18 + telephony");
    }

    v8bis_msg_init(&m, V8BIS_MT_CL);                       /* NS block, Figure 10 */
    m.id_npar1 = V8BIS_ID_NON_STANDARD;
    m.s_spar1 = V8BIS_S_DATA;
    m.data[0] = V8BIS_DATA_TRANSPARENT;
    m.ns_count = 1;
    m.ns[0].country = 0xb5;                                /* T.35 US */
    m.ns[0].provider_len = 2;
    m.ns[0].provider[0] = 0x00;
    m.ns[0].provider[1] = 0x2a;
    m.ns[0].data_len = 3;
    m.ns[0].data[0] = 1;
    m.ns[0].data[1] = 2;
    m.ns[0].data[2] = 3;
    {
        const uint8_t want[] = {0x12, 0xc0, 0x80, 0x80, 0x81, 0xc1, 0x07, 0xb5, 0x02, 0x00, 0x2a, 0x01, 0x02, 0x03};
        expect_bytes(&m, want, sizeof(want), "CL with NS");
    }

    /* ACK and NAK: one octet, matching the K56flex frames already on a real peer */
    v8bis_msg_init(&m, V8BIS_MT_ACK1);
    {
        const uint8_t want[] = {0x14};
        expect_bytes(&m, want, 1, "ACK(1)");
    }
    v8bis_msg_init(&m, V8BIS_MT_NAK1);
    {
        const uint8_t want[] = {0x18};
        expect_bytes(&m, want, 1, "NAK(1)");
    }

    /* round trips, and the decoder's own idempotence */
    for (int t = 0; t < 4000; t++) {
        int n, n2;
        random_msg(&m);
        n = v8bis_msg_encode(&m, buf, sizeof(buf));
        CHECK(n > 0);
        if (n <= 0)
            continue;
        CHECK(v8bis_msg_decode(buf, (size_t)n, &d) == 0);
        CHECK(same_fields(&m, &d));
        CHECK(!d.reserved_bits && !d.delimiter_anomaly && !d.extra_octets && !d.ignored_blocks);
        n2 = v8bis_msg_encode(&d, buf2, sizeof(buf2));
        CHECK(n2 == n && !memcmp(buf, buf2, (size_t)n));
    }

    /* tolerance: what a receiver must walk past (8.2.3) */
    {
        /* reserved NPar(1) bits, a reserved SPar(1) bit with its block, an extra
         * NPar(2) octet in the data block, and a trailing octet */
        const uint8_t in2[] = {0x11, 0xb9, 0x80, 0x80, 0x91,
                               0x01, 0x02, 0x03, 0xc4,        /* data: four octets, one more than Table 6-3 */
                               0xc2,                          /* reserved SPar(1) bit 0x10's block */
                               0x55};                         /* trailing octet */
        CHECK(v8bis_msg_decode(in2, sizeof(in2), &d) == 0);
        CHECK(d.known_type && d.type == V8BIS_MT_MS);
        CHECK(d.id_npar1 == 0x39 && d.reserved_bits);
        CHECK(d.s_spar1 == V8BIS_S_DATA && d.ignored_blocks == 1);
        CHECK(d.data[0] == 1 && d.data[1] == 2 && d.data[2] == 3 && d.extra_octets);
    }
    {
        /* a revision other than 1 is read, not refused (V.92's note) */
        const uint8_t in[] = {0x51, 0x80, 0x80, 0x80, 0x81, 0xc1};
        CHECK(v8bis_msg_decode(in, sizeof(in), &d) == 0 && d.revision == 5 && d.data[0] == 1);
    }
    {
        /* V.92 shares type 1011 with NAK(4): returned unparsed, and re-encodes verbatim */
        const uint8_t in[] = {0x1b, 0x12, 0x34, 0xf0};
        CHECK(v8bis_msg_decode(in, sizeof(in), &d) == 0 && d.type == 0xb && !d.known_type);
        CHECK(d.payload_len == 3 && d.payload[0] == 0x12 && d.payload[2] == 0xf0);
        CHECK(v8bis_msg_encode(&d, buf, sizeof(buf)) == 4 && !memcmp(buf, in, 4));
    }
    {
        const uint8_t in[] = {0x14, 0x00};                  /* ACK with a stray octet */
        CHECK(v8bis_msg_decode(in, 2, &d) == 0 && d.known_type && d.extra_octets);
        CHECK(v8bis_msg_decode(in, 0, &d) == V8BIS_IE_EMPTY);
    }

    /* errors */
    {
        uint8_t full[V8BIS_MAX_INFO_OCTETS];
        int n;
        random_msg(&m);
        m.s_spar1 = V8BIS_S_DATA | V8BIS_S_H324;
        m.h324_spar2 = V8BIS_H324_SPAR2_DATA;
        m.id_npar1 |= V8BIS_ID_NON_STANDARD;
        m.ns_count = 2;
        m.ns[0].provider_len = 3;
        m.ns[0].data_len = 5;
        m.ns[1].provider_len = 2;
        m.ns[1].data_len = 4;
        n = v8bis_msg_encode(&m, full, sizeof(full));
        CHECK(n > 0);
        /* every proper prefix either fails cleanly or is itself a whole message */
        for (int k = 1; k < n; k++) {
            int r = v8bis_msg_decode(full, (size_t)k, &d);
            CHECK(r == 0 || r == V8BIS_IE_TRUNCATED || r == V8BIS_IE_BAD_NS);
        }
        CHECK(v8bis_msg_decode(full, 3, &d) == V8BIS_IE_TRUNCATED);
    }
    {
        v8bis_msg_init(&m, V8BIS_MT_CL);
        m.id_npar1 = V8BIS_ID_NON_STANDARD;
        m.ns_count = V8BIS_NS_BLOCKS_MAX;
        for (int i = 0; i < V8BIS_NS_BLOCKS_MAX; i++) {
            m.ns[i].provider_len = V8BIS_NS_PROVIDER_MAX;
            m.ns[i].data_len = V8BIS_NS_DATA_MAX;
        }
        CHECK(v8bis_msg_encode(&m, buf, sizeof(buf)) == V8BIS_IE_TOO_LONG);  /* 8.6: 64 octets */
        v8bis_msg_init(&m, V8BIS_MT_MS);
        m.data[0] = 0x40;                                   /* a delimiter bit is not a parameter */
        m.s_spar1 = V8BIS_S_DATA;
        CHECK(v8bis_msg_encode(&m, buf, sizeof(buf)) == V8BIS_IE_BAD_ARG);
        v8bis_msg_init(&m, V8BIS_MT_MS);
        m.ns_count = 1;                                     /* NS without the identification bit */
        CHECK(v8bis_msg_encode(&m, buf, sizeof(buf)) == V8BIS_IE_BAD_ARG);
        v8bis_msg_init(&m, V8BIS_MT_MS);
        CHECK(v8bis_msg_encode(&m, buf, 3) < 0);
    }
    {
        /* NS length lies */
        const uint8_t bad1[] = {0x12, 0xc0, 0x80, 0x80, 0x80, 0x09, 0xb5, 0x01};
        const uint8_t bad2[] = {0x12, 0xc0, 0x80, 0x80, 0x80, 0x03, 0xb5, 0x05, 0x00};
        const uint8_t bad3[] = {0x12, 0xc0, 0x80, 0x80, 0x80, 0x00};
        CHECK(v8bis_msg_decode(bad1, sizeof(bad1), &d) == V8BIS_IE_BAD_NS);
        CHECK(v8bis_msg_decode(bad2, sizeof(bad2), &d) == V8BIS_IE_BAD_NS);
        CHECK(v8bis_msg_decode(bad3, sizeof(bad3), &d) == V8BIS_IE_BAD_NS);
    }

    /* fuzz: arbitrary octets never crash, and whatever decodes cleanly
     * survives a re-encode */
    {
        int decoded = 0, stable = 0;
        for (int t = 0; t < (g_quick ? 20000 : 200000); t++) {
            size_t n = 1 + rnd() % 64;
            int r;
            for (size_t i = 0; i < n; i++)
                buf[i] = (uint8_t)rnd();
            if (t % 3 == 0)
                buf[0] = (uint8_t)((buf[0] & 0xf0) | (1 + rnd() % 3));
            if (t % 5 == 0 && n > 2)
                buf[1] |= 0x80, buf[2] |= 0x80;            /* make short blocks likelier */
            r = v8bis_msg_decode(buf, n, &d);
            if (r == 0 && d.known_type && type_check_clean(&d)) {
                v8bis_msg_t e;
                int n2 = v8bis_msg_encode(&d, buf2, sizeof(buf2));
                decoded++;
                if (n2 > 0 && v8bis_msg_decode(buf2, (size_t)n2, &e) == 0 && same_fields(&d, &e))
                    stable++;
            }
        }
        printf("  fuzz: %d cleanly decoded, %d re-encode stable\n", decoded, stable);
        CHECK(decoded > (g_quick ? 10 : 100) && stable == decoded);
    }
}

/* ====================================================================== */
/* tones                                                                  */
/* ====================================================================== */

#define TWO_PI 6.283185307179586

static double dbm0_peak(double dbm0) { return 32767.0 * pow(10.0, (dbm0 - 3.14) / 20.0); }

/* An independent generator: a dual tone then a single tone, durations and
 * frequency error given, summed into out (double). */
static void synth_signal(double *out, size_t at, double f1, double f2, double f3, double level_dbm0,
                         unsigned seg1, unsigned seg2, double ppm)
{
    double a = dbm0_peak(level_dbm0 - 3.0103), b = dbm0_peak(level_dbm0);
    double k = 1.0 + ppm * 1e-6;
    for (unsigned i = 0; i < seg1; i++)
        out[at + i] += a * (sin(TWO_PI * f1 * k * i / 8000.0) + sin(TWO_PI * f2 * k * i / 8000.0));
    for (unsigned i = 0; i < seg2; i++)
        out[at + seg1 + i] += b * sin(TWO_PI * f3 * k * i / 8000.0);
}

static size_t run_detector(const double *x, size_t n, size_t chunk, v8bis_tone_event_t *evs, size_t max_evs)
{
    v8bis_tone_rx_t rx;
    int16_t blk[1024];
    size_t pos = 0, ne = 0;
    v8bis_tone_event_t ev;

    v8bis_tone_rx_init(&rx);
    while (pos < n) {
        size_t c = chunk < n - pos ? chunk : n - pos;
        for (size_t i = 0; i < c; i++) {
            double v = x[pos + i];
            blk[i] = (int16_t)(v > 32767 ? 32767 : v < -32768 ? -32768 : lrint(v));
        }
        v8bis_tone_rx(&rx, blk, (int)c);
        while (v8bis_tone_rx_event(&rx, &ev))
            if (ne < max_evs)
                evs[ne++] = ev;
        pos += c;
    }
    return ne;
}

static void add_noise(double *x, size_t n, double rms)
{
    for (size_t i = 0; i < n; i++)
        x[i] += gauss() * rms;
}

/* Something voice-like: a 130-190 Hz pitch with harmonics to 3.4 kHz,
 * 1/h amplitude, 4 Hz syllabic modulation and vibrato. */
static void add_voice(double *x, size_t n, double rms)
{
    double ph = 0.0, energy = 0.0;
    double *v = calloc(n, sizeof(double));
    for (size_t i = 0; i < n; i++) {
        double t = i / 8000.0, f0 = 160.0 + 30.0 * sin(TWO_PI * 1.7 * t);
        double env = 0.5 + 0.5 * sin(TWO_PI * 4.0 * t), s = 0.0;
        ph += TWO_PI * f0 / 8000.0;
        for (int h = 1; h * f0 < 3400.0; h++)
            s += sin(h * ph) / h;
        v[i] = s * env;
        energy += v[i] * v[i];
    }
    {
        double g = rms / sqrt(energy / n);
        for (size_t i = 0; i < n; i++)
            x[i] += v[i] * g;
    }
    free(v);
}

static void test_tone_generator(void)
{
    v8bis_tone_tx_t tx;
    int16_t buf[4096];
    int total = 0, n;
    double ms1 = 0;

    printf("tone generator\n");
    for (int sig = 0; sig < V8BIS_SIG_COUNT; sig++)
        for (int set = 0; set < 2; set++) {
            double level = v8bis_signal_default_level_dbm0((v8bis_signal_t)sig, -10.0);
            if (!v8bis_signal_in_toneset((v8bis_signal_t)sig, set))
                continue;
            v8bis_tone_tx_start(&tx, (v8bis_signal_t)sig, set, level, 0.0);
            total = 0;
            ms1 = 0;
            while ((n = v8bis_tone_tx(&tx, buf + total, 4096 - total)) > 0)
                total += n;
            CHECK(total == V8BIS_SEG1_SAMPLES + V8BIS_SEG2_SAMPLES);   /* 400 + 100 ms */
            CHECK(v8bis_tone_tx_done(&tx));
            /* segment 1 level: the pair together is `level` */
            for (int i = 0; i < V8BIS_SEG1_SAMPLES; i++)
                ms1 += (double)buf[i] * buf[i];
            ms1 /= V8BIS_SEG1_SAMPLES;
            {
                double ref = pow(32767.0 / sqrt(2.0), 2.0);          /* the +3.14 dBm0 sine */
                double got_dbm0 = 3.14 + 10.0 * log10(ms1 / ref);
                CHECK(fabs(got_dbm0 - level) < 0.15);
            }
        }
    CHECK(v8bis_signal_default_level_dbm0(V8BIS_SIG_CRE, -10) == -10 - 13.5);   /* 7.1.4 */
    CHECK(v8bis_signal_default_level_dbm0(V8BIS_SIG_MRE, -10) == -10 - 13.5);
    CHECK(v8bis_signal_default_level_dbm0(V8BIS_SIG_CRD, -10) == -10);
    /* seg1-only for an ES signal that is followed by a message */
    v8bis_tone_tx_start(&tx, V8BIS_SIG_ESI, false, -10, 0);
    v8bis_tone_tx_seg1_only(&tx);
    total = 0;
    while ((n = v8bis_tone_tx(&tx, buf + total, 4096 - total)) > 0)
        total += n;
    CHECK(total == V8BIS_SEG1_SAMPLES);
}

static void expect_detect(const char *what, const double *x, size_t n, size_t chunk, v8bis_signal_t sig,
                          bool set, size_t seg1_at, int *fail)
{
    v8bis_tone_event_t ev[4];
    size_t ne = run_detector(x, n, chunk, ev, 4);
    bool ok = ne == 1 && ev[0].sig == sig && ev[0].responding_set == set;
    if (ok) {
        double err_ms = fabs((double)((int64_t)ev[0].seg1_start - (int64_t)seg1_at)) / 8.0;
        double lat_ms = ((double)ev[0].detect_sample - (double)(seg1_at + V8BIS_SEG1_SAMPLES)) / 8.0;
        ok = err_ms <= 45.0 && lat_ms > 20.0 && lat_ms < 140.0;
    }
    if (!ok && fail) {
        (*fail)++;
        if (*fail <= 3)
            printf("    miss: %s (%zu events%s)\n", what, ne, ne ? "" : "");
    }
}

static void test_tone_detector(void)
{
    static const struct { v8bis_signal_t sig; bool set; } ALL[] = {
        {V8BIS_SIG_MRE, false}, {V8BIS_SIG_MRD, false}, {V8BIS_SIG_CRE, false},
        {V8BIS_SIG_CRD, false}, {V8BIS_SIG_ESI, false}, {V8BIS_SIG_MRD, true},
        {V8BIS_SIG_CRD, true},  {V8BIS_SIG_ESR, true}};
    const size_t N = 8000 * 3;
    double *x = calloc(N, sizeof(double));
    int fails;
    static const size_t chunks[] = {1, 7, 80, 160, 333, 1000};

    printf("tone detector\n");

    /* clean, every signal in both sets, odd offsets and chunk sizes, tolerances at their limits */
    fails = 0;
    for (size_t k = 0; k < sizeof(ALL) / sizeof(ALL[0]); k++)
        for (int rep = 0; rep < 12; rep++) {
            size_t at = 2000 + rnd() % 3000;
            double ppm = rep % 3 == 0 ? 250.0 : rep % 3 == 1 ? -250.0 : 0.0;
            unsigned s1 = rep & 4 ? 3264 : 3136, s2 = rep & 8 ? 816 : 784;    /* +/-2% */
            double f1, f2;
            double level = -45.0 + 3.0 * (rnd() % 12);                           /* -45 .. -12 dBm0 */
            memset(x, 0, N * sizeof(double));
            v8bis_toneset_hz(ALL[k].set, &f1, &f2);
            synth_signal(x, at, f1, f2, v8bis_signal_seg2_hz(ALL[k].sig), level, s1, s2, ppm);
            expect_detect(v8bis_signal_name(ALL[k].sig), x, N, chunks[rep % 6], ALL[k].sig, ALL[k].set, at, &fails);
        }
    printf("  clean, +/-250 ppm, +/-2%% duration, -45..-12 dBm0: %d of %zu missed\n", fails,
           (size_t)(sizeof(ALL) / sizeof(ALL[0]) * 12));
    CHECK(fails == 0);

    /* noise: SNR measured against the whole signal */
    for (int snr = 20; snr >= 0; snr -= 5) {
        int miss = 0, wrong_total = 0, trials = 0;
        for (size_t k = 0; k < sizeof(ALL) / sizeof(ALL[0]); k++)
            for (int rep = 0; rep < 25; rep++) {
                size_t at = 3000 + rnd() % 2000;
                double f1, f2, level = -25.0, sigrms;
                v8bis_tone_event_t ev[4];
                size_t ne;
                memset(x, 0, N * sizeof(double));
                v8bis_toneset_hz(ALL[k].set, &f1, &f2);
                synth_signal(x, at, f1, f2, v8bis_signal_seg2_hz(ALL[k].sig), level, 3200, 800, 0);
                sigrms = dbm0_peak(level) / sqrt(2.0);
                add_noise(x, N, sigrms / pow(10.0, snr / 20.0));
                ne = run_detector(x, N, 160, ev, 4);
                trials++;
                if (ne == 0)
                    miss++;
                else if (ne != 1 || ev[0].sig != ALL[k].sig || ev[0].responding_set != ALL[k].set)
                    wrong_total++;
            }
        printf("  white noise %2d dB SNR: %3d/%d missed, %d misidentified\n", snr, miss, trials, wrong_total);
        if (snr >= 10)
            CHECK(miss == 0 && wrong_total == 0);
        CHECK(wrong_total == 0);       /* never the wrong signal, whatever the noise */
    }

    /* voice-like interference over the signal ("detectable in the presence of interfering voice") */
    for (int snr = 10; snr >= -5; snr -= 5) {
        int miss = 0, wrong_total = 0, trials = 0;
        for (size_t k = 0; k < sizeof(ALL) / sizeof(ALL[0]); k++)
            for (int rep = 0; rep < 10; rep++) {
                size_t at = 3000 + rnd() % 2000;
                double f1, f2, level = -25.0, sigrms;
                v8bis_tone_event_t ev[4];
                size_t ne;
                memset(x, 0, N * sizeof(double));
                v8bis_toneset_hz(ALL[k].set, &f1, &f2);
                synth_signal(x, at, f1, f2, v8bis_signal_seg2_hz(ALL[k].sig), level, 3200, 800, 0);
                sigrms = dbm0_peak(level) / sqrt(2.0);
                add_voice(x, N, sigrms / pow(10.0, snr / 20.0));
                ne = run_detector(x, N, 160, ev, 4);
                trials++;
                if (ne == 0)
                    miss++;
                else if (ne != 1 || ev[0].sig != ALL[k].sig || ev[0].responding_set != ALL[k].set)
                    wrong_total++;
            }
        printf("  voice-like interference %3d dB SNR: %3d/%d missed, %d misidentified\n", snr, miss,
               trials, wrong_total);
        if (snr >= 10)
            CHECK(miss == 0);
        if (snr == 5)
            CHECK(miss * 20 <= trials);              /* measured 1 in 80 */
        CHECK(wrong_total == 0);
    }

    /* things that must NOT be detected */
    {
        int events = 0, tests = 0;
        v8bis_tone_event_t ev[4];
        double f1, f2;

        /* silence and noise */
        memset(x, 0, N * sizeof(double));
        events += (int)run_detector(x, N, 160, ev, 4);
        add_noise(x, N, 3000.0);
        events += (int)run_detector(x, N, 160, ev, 4);
        tests += 2;
        /* loud voice-like audio alone */
        for (int r = 0; r < 20; r++) {
            memset(x, 0, N * sizeof(double));
            add_voice(x, N, 4000.0);
            events += (int)run_detector(x, N, 160, ev, 4);
            tests++;
        }
        /* ANS, ANSam (15 Hz AM, phase reversals every 450 ms), CNG, V.21 FSK, DTMF */
        {
            double ph = 0.0;
            memset(x, 0, N * sizeof(double));
            for (size_t i = 0; i < N; i++) {
                double t = i / 8000.0, am = 1.0 + 0.2 * sin(TWO_PI * 15.0 * t);
                if (i % 3600 == 0 && i)
                    ph += 3.14159265;
                x[i] = 6000.0 * am * sin(TWO_PI * 2100.0 * i / 8000.0 + ph);
            }
            events += (int)run_detector(x, N, 160, ev, 4);
            tests++;
            for (size_t i = 0; i < N; i++)
                x[i] = (i % 24000 < 4000) ? 6000.0 * sin(TWO_PI * 1100.0 * i / 8000.0) : 0.0;
            events += (int)run_detector(x, N, 160, ev, 4);
            tests++;
        }
        for (int chan = 0; chan < 2; chan++) {
            double ph = 0.0;
            memset(x, 0, N * sizeof(double));
            for (size_t i = 0; i < N; i++) {
                static int bit;
                double f;
                if (i % 27 == 0)
                    bit = rnd() & 1;
                f = chan ? (bit ? 1650.0 : 1850.0) : (bit ? 980.0 : 1180.0);
                ph += TWO_PI * f / 8000.0;
                x[i] = 6000.0 * sin(ph);
            }
            events += (int)run_detector(x, N, 160, ev, 4);
            tests++;
        }
        {
            static const double row[4] = {697, 770, 852, 941}, col[4] = {1209, 1336, 1477, 1633};
            for (int r = 0; r < 4; r++)
                for (int c = 0; c < 4; c++) {
                    memset(x, 0, N * sizeof(double));
                    for (size_t i = 0; i < 6000; i++)
                        x[2000 + i] = 3000.0 * (sin(TWO_PI * row[r] * i / 8000.0) + sin(TWO_PI * col[c] * i / 8000.0));
                    events += (int)run_detector(x, N, 160, ev, 4);
                    tests++;
                }
        }
        /* each tone of either pair alone, a pair with no segment 2, segment 2 with no pair,
         * and a pair followed by the other set's identifying tone */
        {
            static const double singles[] = {1375, 2002, 1529, 2225, 650, 1150, 400, 1900, 980, 1650};
            for (size_t i = 0; i < sizeof(singles) / sizeof(singles[0]); i++) {
                memset(x, 0, N * sizeof(double));
                for (size_t j = 0; j < 12000; j++)
                    x[2000 + j] = 5000.0 * sin(TWO_PI * singles[i] * j / 8000.0);
                events += (int)run_detector(x, N, 160, ev, 4);
                tests++;
            }
            for (int set = 0; set < 2; set++) {
                v8bis_toneset_hz(set, &f1, &f2);
                memset(x, 0, N * sizeof(double));
                synth_signal(x, 3000, f1, f2, 1000.0, -20.0, 3200, 0, 0);          /* no segment 2 */
                events += (int)run_detector(x, N, 160, ev, 4);
                memset(x, 0, N * sizeof(double));
                synth_signal(x, 3000, 1000.0, 1000.0, 1900.0, -20.0, 0, 800, 0);   /* no segment 1 */
                events += (int)run_detector(x, N, 160, ev, 4);
                tests += 2;
            }
            memset(x, 0, N * sizeof(double));
            v8bis_toneset_hz(false, &f1, &f2);
            synth_signal(x, 3000, f1, f2, 1650.0, -20.0, 3200, 800, 0);            /* ESr after the initiating pair */
            events += (int)run_detector(x, N, 160, ev, 4);
            memset(x, 0, N * sizeof(double));
            v8bis_toneset_hz(true, &f1, &f2);
            synth_signal(x, 3000, f1, f2, 650.0, -20.0, 3200, 800, 0);             /* MRe after the responding pair */
            synth_signal(x, 12000, f1, f2, 400.0, -20.0, 3200, 800, 0);            /* CRe likewise */
            synth_signal(x, 12000 + 4500, f1, f2, 980.0, -20.0, 3200, 800, 0);     /* ESi likewise */
            events += (int)run_detector(x, N, 160, ev, 4);
            tests += 2;
        }
        printf("  %d signals that are not V.8bis: %d spurious detections\n", tests, events);
        CHECK(events == 0);
    }

    /* two initiating signals less than 0.5 s apart both come out, in order (10.2.1) */
    {
        double *y = calloc(8000 * 4, sizeof(double));
        v8bis_tone_event_t ev[4];
        double f1, f2;
        size_t ne;

        v8bis_toneset_hz(false, &f1, &f2);
        synth_signal(y, 1000, f1, f2, 400.0, -20.0, 3200, 800, 0);                 /* CRe */
        synth_signal(y, 1000 + 4000 + 2400, f1, f2, 400.0, -20.0, 3200, 800, 0);   /* 300 ms later */
        ne = run_detector(y, 8000 * 4, 160, ev, 4);
        CHECK(ne == 2 && ev[0].sig == V8BIS_SIG_CRE && ev[1].sig == V8BIS_SIG_CRE);
        CHECK(ne == 2 && ev[1].seg1_start > ev[0].detect_sample);
        /* initiating signal answered at once by a responding one */
        memset(y, 0, 8000 * 4 * sizeof(double));
        synth_signal(y, 1000, f1, f2, 1900.0, -20.0, 3200, 800, 0);                /* CRd */
        v8bis_toneset_hz(true, &f1, &f2);
        synth_signal(y, 1000 + 4000 + 400, f1, f2, 1650.0, -20.0, 3200, 800, 0);   /* ESr */
        ne = run_detector(y, 8000 * 4, 160, ev, 4);
        CHECK(ne == 2 && ev[0].sig == V8BIS_SIG_CRD && !ev[0].responding_set
              && ev[1].sig == V8BIS_SIG_ESR && ev[1].responding_set);
        free(y);
    }

    /* the module's own generator through the module's own detector, every combination */
    {
        v8bis_tone_tx_t tx;
        int missed = 0, trials = 0;
        for (size_t k = 0; k < sizeof(ALL) / sizeof(ALL[0]); k++) {
            int16_t buf[8000 * 2];
            size_t at = 1234;
            double *y = calloc(8000 * 2, sizeof(double));
            int got = 0, n;
            v8bis_tone_event_t ev[2];

            v8bis_tone_tx_start(&tx, ALL[k].sig, ALL[k].set, -20.0, 0);
            while ((n = v8bis_tone_tx(&tx, buf + got, 8000 - got)) > 0)
                got += n;
            for (int i = 0; i < got; i++)
                y[at + i] = buf[i];
            trials++;
            if (run_detector(y, 8000 * 2, 160, ev, 2) != 1 || ev[0].sig != ALL[k].sig
                || ev[0].responding_set != ALL[k].set)
                missed++;
            free(y);
        }
        CHECK(missed == 0 && trials == 8);
    }
    free(x);
}

int main(void)
{
    g_quick = getenv("V8BIS_TEST_QUICK") != NULL;
    test_framing();
    test_ie();
    test_tone_generator();
    test_tone_detector();
    printf("%d checks, %d failed\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
