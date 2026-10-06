/*
 * v8bis_modem_test.c -- the V.8bis sample-level modem, end to end.
 *
 * Two modems are joined by a simulated line (delay, noise, echo, G.711) and run
 * every Table 7 transaction as audio.  What each station sent is read back from
 * what the other one received, so the expected sequences are the Recommendation's
 * and not the modem's own bookkeeping.  The transmit waveform is also graded
 * directly, with measurements that do not go through the receiver: tone
 * frequencies and durations, the preamble that doubles as ES segment 2, the
 * silences the clauses prescribe, and the absence of any gap inside CL-MS.
 */
#include <math.h>
#include <spandsp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v8bis_modem.h"

static int g_fail, g_checks;

#define CHECK(c)                                                                         \
    do {                                                                                 \
        g_checks++;                                                                      \
        if (!(c)) {                                                                      \
            if (++g_fail <= 20)                                                          \
                printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);                    \
        }                                                                                \
    } while (0)

static uint32_t g_rng = 0x9e3779b9u;
static uint32_t rnd(void)
{
    g_rng ^= g_rng << 13;
    g_rng ^= g_rng >> 17;
    g_rng ^= g_rng << 5;
    return g_rng;
}
static double gauss(void)
{
    double s = 0;
    for (int i = 0; i < 12; i++)
        s += (rnd() >> 8) / 16777216.0;
    return s - 6.0;
}

#define BLOCK 160

typedef enum { LAW_NONE, LAW_ULAW, LAW_ALAW } law_t;

typedef struct {
    unsigned delay;                  /* samples one way */
    double noise_rms;
    law_t law;
    double echo_gain;                /* own tx leaking into own rx */
    unsigned echo_delay;
    int drop_from, drop_len;         /* mute the A->B line over [from, from+len) samples, -1 = none */
    int drop_dir;                    /* 0: A->B, 1: B->A */
} line_t;

typedef struct {
    v8bis_modem_t *m[2];
    line_t line;
    int16_t *hist[2];                /* each station's transmit history, for delay and echo */
    uint64_t t;                      /* samples run */
    int16_t *txlog[2];               /* what each station transmitted, kept for measurement */
    size_t log_cap;
    /* events */
    v8bis_modem_event_t mode[2];
    bool got_mode[2], got_initial[2], got_no_peer[2];
    v8bis_why_t why[2];
    unsigned nak[2];
    char rx_trace[2][512];           /* tokens heard by station i, in order */
    uint64_t rx_time[2][64];
    unsigned rx_n[2];
    struct { char tok[16]; uint64_t at; int from; } heard[64];
    unsigned heard_n;
    unsigned bad_frames[2];
    unsigned signals[2];
    uint64_t done_at;
} bench_t;

static int16_t apply_law(int16_t s, law_t law)
{
    if (law == LAW_ULAW)
        return ulaw_to_linear(linear_to_ulaw(s));
    if (law == LAW_ALAW)
        return alaw_to_linear(linear_to_alaw(s));
    return s;
}

static void bench_init(bench_t *b, const v8bis_modem_cfg_t *ca, const v8bis_modem_cfg_t *cb, line_t line)
{
    memset(b, 0, sizeof(*b));
    b->m[0] = v8bis_modem_new(ca);
    b->m[1] = v8bis_modem_new(cb);
    b->line = line;
    b->log_cap = 8000 * 40;
    for (int i = 0; i < 2; i++) {
        b->hist[i] = calloc(8000 * 40, sizeof(int16_t));
        b->txlog[i] = calloc(b->log_cap, sizeof(int16_t));
    }
}

static void bench_free(bench_t *b)
{
    for (int i = 0; i < 2; i++) {
        v8bis_modem_free(b->m[i]);
        free(b->hist[i]);
        free(b->txlog[i]);
    }
}

static void drain(bench_t *b, int i)
{
    v8bis_modem_event_t e;

    while (v8bis_modem_event(b->m[i], &e)) {
        switch (e.type) {
        case V8BIS_MEV_MODE:
            b->got_mode[i] = true;
            b->mode[i] = e;
            if (b->done_at == 0 && b->got_mode[0] && b->got_mode[1])
                b->done_at = b->t;
            break;
        case V8BIS_MEV_INITIAL:
            b->got_initial[i] = true;
            b->why[i] = e.why;
            b->nak[i] = e.nak;
            break;
        case V8BIS_MEV_NO_PEER:
            b->got_no_peer[i] = true;
            break;
        case V8BIS_MEV_RX_SIGNAL:
            b->signals[i]++;
            if (b->heard_n < 64) {
                snprintf(b->heard[b->heard_n].tok, 16, "%s", v8bis_signal_name(e.sig));
                b->heard[b->heard_n].at = e.at_sample;
                b->heard[b->heard_n++].from = !i;
            }
            break;
        case V8BIS_MEV_RX_MESSAGE:
            if (b->heard_n < 64) {
                snprintf(b->heard[b->heard_n].tok, 16, "%s", v8bis_msg_type_name(e.msg_type));
                b->heard[b->heard_n].at = e.at_sample;
                b->heard[b->heard_n++].from = !i;
            }
            break;
        case V8BIS_MEV_RX_BAD_FRAME:
            b->bad_frames[i]++;
            break;
        }
    }
}

/* One 20 ms step of the whole line. */
static void step(bench_t *b)
{
    int16_t tx[2][BLOCK], rx[2][BLOCK];

    for (int i = 0; i < 2; i++) {
        v8bis_modem_tx(b->m[i], tx[i], BLOCK);
        if (b->t + BLOCK <= b->log_cap)
            memcpy(b->txlog[i] + b->t, tx[i], BLOCK * sizeof(int16_t));
        if (b->t + BLOCK <= 8000 * 40)
            memcpy(b->hist[i] + b->t, tx[i], BLOCK * sizeof(int16_t));
    }
    for (int i = 0; i < 2; i++) {          /* receiver i hears transmitter !i */
        int from = !i;

        for (int k = 0; k < BLOCK; k++) {
            int64_t src = (int64_t)(b->t + (uint64_t)k) - (int64_t)b->line.delay;
            double v = 0.0;

            if (src >= 0 && src < 8000 * 40)
                v = b->hist[from][src];
            if (b->line.drop_from >= 0 && from == b->line.drop_dir && (int64_t)(b->t + (uint64_t)k) >= b->line.drop_from
                && (int64_t)(b->t + (uint64_t)k) < b->line.drop_from + b->line.drop_len)
                v = 0.0;
            if (b->line.echo_gain > 0.0) {
                int64_t es = (int64_t)(b->t + (uint64_t)k) - (int64_t)b->line.echo_delay;

                if (es >= 0)
                    v += b->line.echo_gain * b->hist[i][es];
            }
            v += gauss() * b->line.noise_rms;
            if (v > 32767.0) v = 32767.0;
            if (v < -32768.0) v = -32768.0;
            rx[i][k] = apply_law((int16_t)lrint(v), b->line.law);
        }
    }
    for (int i = 0; i < 2; i++)
        v8bis_modem_rx(b->m[i], rx[i], BLOCK);
    b->t += BLOCK;
    for (int i = 0; i < 2; i++)
        drain(b, i);
    /* 9.7: an MS receiver that sent no ACK(1) starts ANS/ANSam; its peer hears that */
    for (int i = 0; i < 2; i++)
        if (b->got_mode[i] && b->mode[i].mode.start_signal_next && !b->mode[i].mode.ack_sent
            && !b->got_mode[!i] && v8bis_modem_fsm(b->m[!i])->state == V8BIS_S_SENT_MS)
            v8bis_modem_startup_signal(b->m[!i]);
}

static void run_for(bench_t *b, unsigned ms)
{
    unsigned blocks = ms * 8 / BLOCK;

    for (unsigned i = 0; i < blocks; i++)
        step(b);
}

/* Run until both stations have an outcome or the time runs out. */
static void run_until_done(bench_t *b, unsigned max_ms)
{
    unsigned blocks = max_ms * 8 / BLOCK;

    for (unsigned i = 0; i < blocks; i++) {
        step(b);
        if (b->got_mode[0] && b->got_mode[1]) {
            run_for(b, 200);                       /* let any trailing audio finish */
            return;
        }
        if ((b->got_initial[0] || b->got_no_peer[0]) && (b->got_initial[1] || b->got_no_peer[1])
            && !v8bis_modem_tx_busy(b->m[0]) && !v8bis_modem_tx_busy(b->m[1]))
            return;
    }
}

/* ---- configurations --------------------------------------------------------- */

static void caps_full(v8bis_msg_t *c)
{
    v8bis_msg_init(c, V8BIS_MT_CL);
    c->id_npar1 = V8BIS_ID_V8 | V8BIS_ID_SHORT_V8;
    c->s_spar1 = V8BIS_S_DATA | V8BIS_S_ANALOGUE_TEL;
    c->data[0] = V8BIS_DATA_TRANSPARENT | V8BIS_DATA_V42 | V8BIS_DATA_V42BIS;
    c->data[1] = V8BIS_DATA2_V34 | V8BIS_DATA2_V32BIS;
    c->data[2] = V8BIS_DATA3_V32 | V8BIS_DATA3_V22BIS | V8BIS_DATA3_V22 | V8BIS_DATA3_V21;
    c->analogue_tel = V8BIS_TEL_VOICE;
}

static void mcfg(v8bis_modem_cfg_t *c, bool auto_call, bool answering)
{
    v8bis_modem_cfg_default(c);
    caps_full(&c->fsm.caps);
    c->fsm.auto_answer_call = auto_call;
    c->fsm.answering_station = answering;
}

static line_t clean_line(void)
{
    line_t l;

    memset(&l, 0, sizeof(l));
    l.drop_from = -1;
    return l;
}

/* The sequence of what was heard, as the Recommendation writes it: "I:MRe R:ESr R:MS I:ACK(1)". */
static void build_trace(const bench_t *b, char *out, size_t cap)
{
    size_t n = 0;
    int last_from = -1;
    char last[16] = "";

    out[0] = 0;
    for (unsigned i = 0; i < b->heard_n; i++) {
        const char *t = b->heard[i].tok;
        char tok[24];

        /* CL immediately followed by MS from the same sender is one CL-MS */
        if (!strcmp(t, "MS") && !strcmp(last, "CL") && last_from == b->heard[i].from) {
            size_t len = strlen(out);

            /* rewrite the previous CL as CL-MS */
            if (len >= 2 && !strcmp(out + len - 2, "CL"))
                snprintf(out + len, cap - len, "-MS");
            snprintf(last, sizeof(last), "%s", "CL-MS");
            continue;
        }
        snprintf(tok, sizeof(tok), "%c:%s", b->heard[i].from ? 'R' : 'I', t);
        n = strlen(out);
        snprintf(out + n, cap - n, "%s%s", n ? " " : "", tok);
        snprintf(last, sizeof(last), "%s", t);
        last_from = b->heard[i].from;
    }
}

/* ---- Table 7 as audio ---------------------------------------------------------- */

typedef struct {
    int number;
    bool auto_call;
    v8bis_init_t how;
    v8bis_mr_reply_t mr_reply;
    v8bis_cr_reply_t cr_reply;
    bool i_ask_caps, i_crd_clr, r_crd_clr;
    const char *trace;
    int ms_from;
} txn_t;

static const txn_t TXNS[] = {
    {1, true, V8BIS_INIT_MR, V8BIS_MRR_MS, V8BIS_CRR_CL, 0, 0, 0, "I:MRe R:ESr R:MS I:ACK(1)", 1},
    {2, true, V8BIS_INIT_CR, V8BIS_MRR_MS, V8BIS_CRR_CL, 0, 0, 0, "I:CRe R:ESr R:CL I:MS R:ACK(1)", 0},
    {3, true, V8BIS_INIT_CR, V8BIS_MRR_MS, V8BIS_CRR_CLR, 0, 0, 0, "I:CRe R:ESr R:CLR I:CL-MS R:ACK(1)", 0},
    {4, true, V8BIS_INIT_MS, V8BIS_MRR_MS, V8BIS_CRR_CL, 0, 0, 0, "I:ESi I:MS R:ACK(1)", 0},
    {5, true, V8BIS_INIT_CL, V8BIS_MRR_MS, V8BIS_CRR_CL, 0, 0, 0, "I:ESi I:CL R:MS I:ACK(1)", 1},
    {6, true, V8BIS_INIT_CLR, V8BIS_MRR_MS, V8BIS_CRR_CL, 0, 0, 0, "I:ESi I:CLR R:CL I:MS R:ACK(1)", 0},
    {7, true, V8BIS_INIT_MR, V8BIS_MRR_MRD, V8BIS_CRR_CL, 0, 0, 0, "I:MRe R:MRd I:MS R:ACK(1)", 0},
    {8, true, V8BIS_INIT_MR, V8BIS_MRR_MRD, V8BIS_CRR_CL, 1, 0, 0, "I:MRe R:MRd I:CRd R:CL I:MS R:ACK(1)", 0},
    {9, true, V8BIS_INIT_MR, V8BIS_MRR_MRD, V8BIS_CRR_CL, 1, 0, 1, "I:MRe R:MRd I:CRd R:CLR I:CL-MS R:ACK(1)", 0},
    {10, true, V8BIS_INIT_MR, V8BIS_MRR_CRD, V8BIS_CRR_CL, 0, 0, 0, "I:MRe R:CRd I:CL R:MS I:ACK(1)", 1},
    {11, true, V8BIS_INIT_MR, V8BIS_MRR_CRD, V8BIS_CRR_CL, 0, 1, 0, "I:MRe R:CRd I:CLR R:CL-MS I:ACK(1)", 1},
    {12, true, V8BIS_INIT_CR, V8BIS_MRR_MS, V8BIS_CRR_CRD, 0, 0, 0, "I:CRe R:CRd I:CL R:MS I:ACK(1)", 1},
    {13, true, V8BIS_INIT_CR, V8BIS_MRR_MS, V8BIS_CRR_CRD, 0, 1, 0, "I:CRe R:CRd I:CLR R:CL-MS I:ACK(1)", 1},
    {1, false, V8BIS_INIT_MR, V8BIS_MRR_MS, V8BIS_CRR_CL, 0, 0, 0, "I:MRd R:MS I:ACK(1)", 1},
    {2, false, V8BIS_INIT_CR, V8BIS_MRR_MS, V8BIS_CRR_CL, 0, 0, 0, "I:CRd R:CL I:MS R:ACK(1)", 0},
    {3, false, V8BIS_INIT_CR, V8BIS_MRR_MS, V8BIS_CRR_CLR, 0, 0, 0, "I:CRd R:CLR I:CL-MS R:ACK(1)", 0},
    {4, false, V8BIS_INIT_MS, V8BIS_MRR_MS, V8BIS_CRR_CL, 0, 0, 0, "I:ESi I:MS R:ACK(1)", 0},
    {5, false, V8BIS_INIT_CL, V8BIS_MRR_MS, V8BIS_CRR_CL, 0, 0, 0, "I:ESi I:CL R:MS I:ACK(1)", 1},
    {6, false, V8BIS_INIT_CLR, V8BIS_MRR_MS, V8BIS_CRR_CL, 0, 0, 0, "I:ESi I:CLR R:CL I:MS R:ACK(1)", 0},
};
#define NTXN ((int)(sizeof(TXNS) / sizeof(TXNS[0])))

static void setup_txn(const txn_t *t, v8bis_modem_cfg_t *ci, v8bis_modem_cfg_t *cr)
{
    mcfg(ci, t->auto_call, true);
    mcfg(cr, t->auto_call, false);
    ci->fsm.on_mrd_ask_caps = t->i_ask_caps;
    ci->fsm.on_crd_send_clr = t->i_crd_clr;
    cr->fsm.mr_reply = t->mr_reply;
    cr->fsm.cr_reply = t->cr_reply;
    cr->fsm.on_crd_send_clr = t->r_crd_clr;
    if (t->how == V8BIS_INIT_MS) {
        ci->fsm.have_preset_ms = true;
        v8bis_default_select_ms(&ci->fsm.caps, NULL, true, &ci->fsm.preset_ms);
    }
}

static bool run_txn(const txn_t *t, line_t line, bench_t *b, char *trace, size_t cap)
{
    v8bis_modem_cfg_t ci, cr;

    setup_txn(t, &ci, &cr);
    bench_init(b, &ci, &cr, line);
    v8bis_modem_initiate(b->m[0], t->how);
    run_until_done(b, 30000);
    build_trace(b, trace, cap);
    return b->got_mode[0] && b->got_mode[1];
}

static void test_table7_audio(void)
{
    printf("Table 7 as audio\n");
    for (int i = 0; i < NTXN; i++) {
        const txn_t *t = &TXNS[i];
        bench_t b;
        char trace[512], what[48];
        bool ok;

        ok = run_txn(t, clean_line(), &b, trace, sizeof(trace));
        if (getenv("V8BIS_DEBUG") && !ok)
            printf("    txn %d: bad=%u/%u mode=%d/%d initial=%d/%d nopeer=%d/%d\n", t->number, b.bad_frames[0], b.bad_frames[1], b.got_mode[0], b.got_mode[1], b.got_initial[0], b.got_initial[1], b.got_no_peer[0], b.got_no_peer[1]);
        snprintf(what, sizeof(what), "transaction %d%s", t->number, t->auto_call ? " (auto)" : "");
        g_checks++;
        if (strcmp(trace, t->trace)) {
            if (++g_fail <= 20)
                printf("  FAIL %s:\n    want %s\n    got  %s\n", what, t->trace, trace);
        }
        CHECK(ok);
        if (ok) {
            int sender = b.mode[0].mode.we_sent_ms ? 0 : 1;

            CHECK(b.mode[0].mode.we_sent_ms != b.mode[1].mode.we_sent_ms);
            CHECK(sender == t->ms_from);
            CHECK(b.mode[0].mode.startup == V8BIS_STARTUP_SHORT_V8);
            CHECK(!memcmp(b.mode[0].mode.ms.data, b.mode[1].mode.ms.data, 3));
            CHECK(b.bad_frames[0] == 0 && b.bad_frames[1] == 0);
            if (i == 0 || i == 3 || i == 12)
                printf("  transaction %d%s completed in %.2f s\n", t->number, t->auto_call ? " (auto)" : "",
                       (double)b.done_at / 8000.0);
        }
        bench_free(&b);
    }
}

/* ---- the waveform, measured directly -------------------------------------------------- */

static double tone_frac(const int16_t *x, size_t n, double f)
{
    double w = 2.0 * M_PI * f / 8000.0, c = 2.0 * cos(w), s1 = 0, s2 = 0, tot = 0, p;

    for (size_t i = 0; i < n; i++) {
        double s0 = x[i] + c * s1 - s2;
        s2 = s1;
        s1 = s0;
        tot += (double)x[i] * x[i];
    }
    if (tot <= 0)
        return 0;
    p = s1 * s1 + s2 * s2 - c * s1 * s2;
    return 2.0 * p / ((double)n * tot);
}

static size_t first_nonzero(const int16_t *x, size_t n)
{
    for (size_t i = 0; i < n; i++)
        if (x[i])
            return i;
    return n;
}

/* longest run of exact zeros starting at or after `from` and ending before the last non-zero sample */
static size_t longest_zero_run(const int16_t *x, size_t from, size_t to)
{
    size_t best = 0, run = 0;

    for (size_t i = from; i < to; i++) {
        if (x[i] == 0) {
            if (++run > best)
                best = run;
        } else {
            run = 0;
        }
    }
    return best;
}

static size_t last_nonzero(const int16_t *x, size_t n)
{
    while (n && x[n - 1] == 0)
        n--;
    return n;
}

static void test_waveform(void)
{
    printf("transmit waveform\n");

    /* MRe: 400 ms of silence, then 400 ms of 1375+2002 and 100 ms of 650, 12-15 dB under nominal */
    {
        v8bis_modem_cfg_t ca, cb;
        bench_t b;
        size_t at;
        const int16_t *x;

        mcfg(&ca, true, true);
        mcfg(&cb, true, false);
        ca.fsm.auto_answer_call = true;
        bench_init(&b, &ca, &cb, clean_line());
        v8bis_modem_initiate(b.m[0], V8BIS_INIT_MR);
        run_for(&b, 1200);
        x = b.txlog[0];
        at = first_nonzero(x, 8000);
        CHECK(at >= 3200 && at <= 3210);                         /* 10.2.2: 400 ms of silence first */
        CHECK(tone_frac(x + at + 200, 2400, 1375.0) + tone_frac(x + at + 200, 2400, 2002.0) > 0.9);
        CHECK(tone_frac(x + at + 3200 + 80, 640, 650.0) > 0.95); /* segment 2 */
        {
            /* level: nominal is -13 dBm0, MRe is 13.5 dB under that */
            double ms = 0;
            for (size_t i = at + 400; i < at + 3000; i++)
                ms += (double)x[i] * x[i];
            ms /= 2600.0;
            CHECK(fabs(3.14 + 10.0 * log10(ms / (32767.0 * 32767.0 / 2.0)) - (-13.0 - 13.5)) < 0.3);
        }
        CHECK(last_nonzero(x, 8000) - at >= 3990 && last_nonzero(x, 8000) - at <= 4010);   /* 500 ms */
        bench_free(&b);
    }

    /* ESi + MS (transaction 4, no echo suppressor): 400 ms pair, then the preamble IS segment 2 */
    {
        v8bis_modem_cfg_t ca, cb;
        bench_t b;
        const int16_t *x;
        size_t end;
        int nbits_expected;

        mcfg(&ca, false, true);
        mcfg(&cb, false, false);
        ca.fsm.have_preset_ms = true;
        v8bis_default_select_ms(&ca.fsm.caps, NULL, true, &ca.fsm.preset_ms);
        bench_init(&b, &ca, &cb, clean_line());
        v8bis_modem_initiate(b.m[0], V8BIS_INIT_MS);
        run_for(&b, 4000);
        x = b.txlog[0];
        CHECK(first_nonzero(x, 8000) < 4);
        CHECK(tone_frac(x + 100, 2400, 1375.0) + tone_frac(x + 100, 2400, 2002.0) > 0.9);
        /* V.21(L) mark is 980 Hz: straight after the pair, for 100 ms (30 bits) */
        CHECK(tone_frac(x + 3200 + 160, 480, 980.0) > 0.9);
        /* no gap between the pair and the preamble */
        CHECK(longest_zero_run(x, 0, last_nonzero(x, 32000)) < 4);
        end = last_nonzero(x, 32000);
        {
            uint8_t info[64], bits[1300];
            v8bis_msg_t ms = ca.fsm.preset_ms;
            int n;

            ms.type = V8BIS_MT_MS;
            n = v8bis_msg_encode(&ms, info, sizeof(info));
            nbits_expected = (int)v8bis_frame_encode(info, (size_t)n, V8BIS_PREAMBLE_BITS, 2, 1, bits, sizeof(bits));
            /* the message lasts exactly nbits at 300 bit/s after the 400 ms pair (to within a bit) */
            CHECK(fabs((double)(end - 3200) - nbits_expected * 8000.0 / 300.0) < 8000.0 / 300.0 * 0.6);
        }
        bench_free(&b);
    }

    /* the 9.4 gap: ES pair, 100 ms of mark as segment 2, 1.5 s of silence, then the message with its own preamble */
    {
        v8bis_modem_cfg_t ca, cb;
        bench_t b;
        const int16_t *x;
        size_t gap_start, resume;

        mcfg(&ca, false, true);
        mcfg(&cb, false, false);
        ca.fsm.echo_suppressor = true;
        ca.fsm.have_preset_ms = true;
        v8bis_default_select_ms(&ca.fsm.caps, NULL, true, &ca.fsm.preset_ms);
        bench_init(&b, &ca, &cb, clean_line());
        v8bis_modem_initiate(b.m[0], V8BIS_INIT_MS);
        run_for(&b, 6000);
        x = b.txlog[0];
        CHECK(tone_frac(x + 3200 + 100, 560, 980.0) > 0.9);              /* the 100 ms segment 2 */
        /* the silence is the first run of 1000 zeros after the pair; its far end is the new preamble */
        for (gap_start = 3200; gap_start < 40000; gap_start++)
            if (longest_zero_run(x, gap_start, gap_start + 1000) == 1000)
                break;
        CHECK(gap_start >= 3995 && gap_start <= 4005);                   /* seg 2 ends 100 ms after the pair */
        for (resume = gap_start; resume < 40000 && x[resume] == 0; resume++)
            ;
        if (getenv("V8BIS_DEBUG"))
            printf("    gap_start %zu resume %zu\n", gap_start, resume);
        CHECK(resume - gap_start >= 11990 && resume - gap_start <= 12010);   /* 1.5 s */
        CHECK(tone_frac(x + resume + 100, 480, 980.0) > 0.9);            /* the message's own preamble */
        bench_free(&b);
    }

    /* CL immediately followed by MS: no silent interval anywhere in the pair (9.1 note 1) */
    {
        const txn_t *t = &TXNS[2];                                       /* transaction 3: I sends CL-MS */
        v8bis_modem_cfg_t ci, cr;
        bench_t b;
        size_t from, to;
        unsigned ms_start;
        int16_t *x;

        setup_txn(t, &ci, &cr);
        bench_init(&b, &ci, &cr, clean_line());
        v8bis_modem_initiate(b.m[0], t->how);
        run_until_done(&b, 30000);
        CHECK(b.got_mode[0] && b.got_mode[1]);
        x = b.txlog[0];
        /* I's transmission is: [silence] CRe, [silence while R answers], CL-MS, [silence]; find the last burst */
        to = last_nonzero(x, b.t);
        from = to;
        while (from > 0 && longest_zero_run(x, from - 1, to) < 30)
            from--;
        (void)ms_start;
        CHECK(to - from > 4000);                                         /* two messages of audio */
        CHECK(longest_zero_run(x, from + 8, to) < 30);
        bench_free(&b);
    }
}

/* ---- the line is not clean ------------------------------------------------------------------ */

static void sweep(const char *name, line_t line, int expect_min_pct)
{
    int ok = 0, total = 0;

    for (int rep = 0; rep < 2; rep++)
        for (int i = 0; i < NTXN; i++) {
            bench_t b;
            char trace[512];

            total++;
            if (run_txn(&TXNS[i], line, &b, trace, sizeof(trace)) && !strcmp(trace, TXNS[i].trace))
                ok++;
            else if (getenv("V8BIS_DEBUG") && total < 40)
                printf("    miss %s%d: got [%s] bad=%u/%u mode=%d/%d\n", TXNS[i].auto_call ? "auto " : "", TXNS[i].number,
                       trace, b.bad_frames[0], b.bad_frames[1], b.got_mode[0], b.got_mode[1]);
            bench_free(&b);
        }
    printf("  %-34s %3d of %d\n", name, ok, total);
    CHECK(ok * 100 >= expect_min_pct * total);
}

static void test_line_conditions(void)
{
    line_t l;

    printf("line conditions (all 19 transaction cases, twice)\n");
    l = clean_line();
    l.delay = 37;
    sweep("37 samples one way", l, 100);
    l.delay = 480;
    sweep("60 ms one way", l, 100);
    l.delay = 2400;
    sweep("300 ms one way", l, 100);
    l = clean_line();
    l.law = LAW_ULAW;
    sweep("mu-law", l, 100);
    l.law = LAW_ALAW;
    sweep("A-law", l, 100);
    l = clean_line();
    l.echo_gain = 0.3;
    l.echo_delay = 24;
    sweep("own echo at -10 dB, 3 ms", l, 100);
    l.echo_delay = 400;
    sweep("own echo at -10 dB, 50 ms", l, 100);
    l = clean_line();
    l.law = LAW_ULAW;
    l.delay = 160;
    l.noise_rms = 100.0;                  /* signals are near 2000-4500 rms: ~30 dB */
    sweep("30 dB noise, mu-law", l, 100);
    l.noise_rms = 400.0;
    sweep("~18 dB noise", l, 90);
    l.noise_rms = 800.0;
    sweep("~12 dB noise (reported)", l, 0);
    l.noise_rms = 1500.0;
    sweep("~6 dB noise (reported)", l, 0);
}

/* ---- what the receiver must not believe ---------------------------------------------------------- */

static void test_bad_input(void)
{
    printf("hostile input\n");

    /* noise, voice-like audio and unframed V.21 on an idle modem: no events, no transmission */
    {
        v8bis_modem_cfg_t c;
        v8bis_modem_t *m;
        int16_t blk[BLOCK], out[BLOCK];
        v8bis_modem_event_t e;
        int events = 0, tx_nonzero = 0;
        double ph = 0;

        mcfg(&c, false, false);
        m = v8bis_modem_new(&c);
        for (int i = 0; i < 8000 * 20 / BLOCK; i++) {
            for (int k = 0; k < BLOCK; k++) {
                double v = gauss() * 2000.0;
                int bit = ((i * BLOCK + k) / 27) & 1;

                if (i > 400)                         /* then random V.21(L) with no preamble or flags */
                    v = 4000.0 * sin(ph += 2.0 * M_PI * (bit ? 980.0 : 1180.0) / 8000.0);
                blk[k] = (int16_t)v;
            }
            v8bis_modem_rx(m, blk, BLOCK);
            v8bis_modem_tx(m, out, BLOCK);
            for (int k = 0; k < BLOCK; k++)
                tx_nonzero += out[k] != 0;
            while (v8bis_modem_event(m, &e))
                events++;
        }
        CHECK(events == 0);
        CHECK(tx_nonzero == 0);
        v8bis_modem_free(m);
    }

    /* a frame cut in the middle: FCS error, NAK(1), both back to Initial */
    {
        const txn_t *t = &TXNS[13 + 1];                  /* telephony transaction 2: CRd R:CL I:MS R:ACK */
        v8bis_modem_cfg_t ci, cr;
        bench_t b;
        line_t l = clean_line();

        l.drop_dir = 1;                                   /* R's CL */
        setup_txn(t, &ci, &cr);
        bench_init(&b, &ci, &cr, l);
        /* find when R starts transmitting its CL in a clean run, then cut a bit out of it */
        {
            bench_t probe;
            char tr[512];
            size_t start;

            run_txn(t, clean_line(), &probe, tr, sizeof(tr));
            start = first_nonzero(probe.txlog[1], probe.log_cap);
            bench_free(&probe);
            bench_free(&b);
            l.drop_from = (int)start + 1500;
            l.drop_len = 400;                             /* 50 ms inside the message */
        }
        setup_txn(t, &ci, &cr);
        bench_init(&b, &ci, &cr, l);
        v8bis_modem_initiate(b.m[0], t->how);
        run_until_done(&b, 20000);
        if (getenv("V8BIS_DEBUG"))
            { char tr[512]; build_trace(&b, tr, sizeof(tr)); printf("    cut at %d len %d dir %d: bad=%u/%u mode=%d/%d trace [%s] R tx first %zu\n", l.drop_from, l.drop_len, l.drop_dir, b.bad_frames[0], b.bad_frames[1], b.got_mode[0], b.got_mode[1], tr, first_nonzero(b.txlog[1], b.log_cap)); }
        CHECK(b.bad_frames[0] == 1);
        CHECK(!b.got_mode[0] && !b.got_mode[1]);
        CHECK(b.got_initial[0] || b.got_no_peer[0]);
        bench_free(&b);
    }
}

/* ---- nobody there ----------------------------------------------------------------------------------- */

static void test_no_peer(void)
{
    v8bis_modem_cfg_t c;
    bench_t b;
    v8bis_modem_cfg_t idle;

    printf("no peer\n");

    /* 10.2.2: MRe at 400 ms, again 3 s after each one ends, twice, then give up */
    mcfg(&c, true, true);
    mcfg(&idle, true, false);
    bench_init(&b, &c, &idle, clean_line());
    b.line.noise_rms = 0;
    v8bis_modem_initiate(b.m[0], V8BIS_INIT_MR);
    /* the second station is not reached: break the line both ways */
    b.line.drop_from = 0;
    b.line.drop_len = 8000 * 40;
    b.line.drop_dir = 0;
    run_for(&b, 14000);
    CHECK(b.got_no_peer[0]);
    {
        /* bursts of audio separated by silences of at least a quarter second */
        int bursts = 0;
        size_t start[8] = {0}, i = 0, end_of_last = 0;

        while (i < b.t) {
            if (b.txlog[0][i] == 0) {
                i++;
                continue;
            }
            if (bursts == 0 || i - end_of_last > 2000) {
                if (bursts < 8)
                    start[bursts] = i;
                bursts++;
            }
            end_of_last = i;
            i++;
        }
        if (getenv("V8BIS_DEBUG"))
            printf("    bursts %d starts %zu %zu %zu\n", bursts, start[0], start[1], start[2]);
        CHECK(bursts == 3);                                            /* the signal and two retries */
        CHECK(start[0] >= 3200 && start[0] <= 3210);                   /* after the 400 ms silence */
        /* each repeat 3 s after the previous signal ended: 4000 samples of signal + 24000 of waiting */
        CHECK(start[1] - start[0] >= 28000 - 4 && start[1] - start[0] <= 28000 + 340);
        CHECK(start[2] - start[1] >= 28000 - 4 && start[2] - start[1] <= 28000 + 340);
    }
    bench_free(&b);

    /* a message initiation with nobody listening: the 9.8 rule, reported as no peer */
    mcfg(&c, false, true);
    mcfg(&idle, false, false);
    c.fsm.have_preset_ms = true;
    v8bis_default_select_ms(&c.fsm.caps, NULL, true, &c.fsm.preset_ms);
    bench_init(&b, &c, &idle, clean_line());
    b.line.drop_from = 0;
    b.line.drop_len = 8000 * 40;
    b.line.drop_dir = 0;
    v8bis_modem_initiate(b.m[0], V8BIS_INIT_MS);
    run_for(&b, 9000);
    CHECK(b.got_no_peer[0] && !b.got_mode[0]);
    bench_free(&b);

    /* both ends start at once (glare): nobody gets into MS mode with the wrong idea of the other */
    {
        v8bis_modem_cfg_t a, bb;
        bench_t g;

        mcfg(&a, false, false);
        mcfg(&bb, false, false);
        bench_init(&g, &a, &bb, clean_line());
        v8bis_modem_initiate(g.m[0], V8BIS_INIT_CR);
        v8bis_modem_initiate(g.m[1], V8BIS_INIT_CR);
        run_for(&g, 20000);
        CHECK(!g.got_mode[0] && !g.got_mode[1]);
        CHECK(g.got_no_peer[0] && g.got_no_peer[1]);
        bench_free(&g);
    }
}

/* ---- random configurations and lines: whatever happens, the two ends agree ----------------------- */

static void random_mcfg(v8bis_modem_cfg_t *c, bool auto_call, bool answering)
{
    mcfg(c, auto_call, answering);
    c->fsm.caps.id_npar1 = (uint8_t)(rnd() & 3);
    c->fsm.caps.data[0] = (uint8_t)(rnd() & 7);
    c->fsm.caps.data[1] = (uint8_t)(rnd() & 0x30);
    c->fsm.caps.data[2] = (uint8_t)(rnd() & 0x0f);
    c->fsm.tx_ack1 = rnd() % 4 != 0;
    c->fsm.mode_available = rnd() % 6 != 0;
    c->fsm.max_info_octets = rnd() % 3 == 0 ? 10 + rnd() % 10 : 0;
    c->fsm.mr_reply = (v8bis_mr_reply_t)(rnd() % 3);
    c->fsm.cr_reply = (v8bis_cr_reply_t)(rnd() % 3);
    c->fsm.on_mrd_ask_caps = rnd() & 1;
    c->fsm.on_crd_send_clr = rnd() & 1;
    c->fsm.echo_suppressor = rnd() % 4 == 0;
    c->retries = 0;
}

static void test_random_audio(void)
{
    int both = 0, none = 0, disagree = 0, runs = 150;

    printf("random pairs over random lines\n");
    for (int r = 0; r < runs; r++) {
        v8bis_modem_cfg_t ci, cr;
        bench_t b;
        line_t l = clean_line();
        bool auto_call = rnd() & 1;
        static const v8bis_init_t HOW[] = {V8BIS_INIT_MR, V8BIS_INIT_CR, V8BIS_INIT_MS, V8BIS_INIT_CL, V8BIS_INIT_CLR};
        v8bis_init_t how = HOW[rnd() % 5];

        random_mcfg(&ci, auto_call, true);
        random_mcfg(&cr, auto_call, false);
        if (how == V8BIS_INIT_MS) {
            ci.fsm.have_preset_ms = v8bis_default_select_ms(&ci.fsm.caps, NULL, ci.fsm.tx_ack1, &ci.fsm.preset_ms);
        }
        l.delay = rnd() % 1200;
        l.law = (law_t)(rnd() % 3);
        l.noise_rms = rnd() % 3 == 0 ? 60.0 + rnd() % 300 : 0.0;
        l.echo_gain = rnd() % 3 == 0 ? 0.2 : 0.0;
        l.echo_delay = rnd() % 200;
        bench_init(&b, &ci, &cr, l);
        v8bis_modem_initiate(b.m[0], how);
        run_until_done(&b, 25000);
        if (b.got_mode[0] && b.got_mode[1]) {
            both++;
            if (b.mode[0].mode.we_sent_ms == b.mode[1].mode.we_sent_ms
                || memcmp(b.mode[0].mode.ms.data, b.mode[1].mode.ms.data, 3)
                || b.mode[0].mode.ms.id_npar1 != b.mode[1].mode.ms.id_npar1)
                disagree++;
        } else if (!b.got_mode[0] && !b.got_mode[1]) {
            none++;
        } else if (!b.mode[0].mode.start_signal_next && !b.mode[1].mode.start_signal_next
                   && (b.got_mode[0] || b.got_mode[1])) {
            /* one end reached MS mode alone: only a lost final ACK(1) (or noise) can do that */
            if (b.line.noise_rms == 0.0 && b.line.law == LAW_NONE)
                disagree++;
        }
        bench_free(&b);
    }
    printf("  %d runs: %d reached MS mode on both, %d on neither, %d disagreed\n", runs, both, none, disagree);
    CHECK(disagree == 0);
    CHECK(both > runs / 5);
}

/* ---- arbitrary audio never breaks it ---------------------------------------------------------------- */

static void test_audio_fuzz(void)
{
    v8bis_modem_cfg_t c;
    v8bis_modem_t *m;
    int16_t blk[BLOCK], out[BLOCK];
    v8bis_modem_event_t e;
    long events = 0, sigs = 0, msgs = 0, bad = 0;

    printf("audio fuzz\n");
    for (int run = 0; run < 6; run++) {
        mcfg(&c, run & 1, run & 2);
        m = v8bis_modem_new(&c);
        if (run & 4)
            v8bis_modem_initiate(m, V8BIS_INIT_CR);
        for (int i = 0; i < 8000 * 120 / BLOCK; i++) {
            double f1 = 300.0 + (rnd() % 2000), amp = (rnd() % 3) ? 3000.0 : 12000.0;
            bool burst = (i / 40) % 3 == 0;

            for (int k = 0; k < BLOCK; k++) {
                double v = gauss() * 300.0;

                if (burst)
                    v += amp * sin(2.0 * M_PI * f1 * (i * BLOCK + k) / 8000.0);
                blk[k] = (int16_t)(v > 32767 ? 32767 : v < -32768 ? -32768 : v);
            }
            v8bis_modem_rx(m, blk, BLOCK);
            v8bis_modem_tx(m, out, BLOCK);
            while (v8bis_modem_event(m, &e)) {
                events++;
                sigs += e.type == V8BIS_MEV_RX_SIGNAL;
                msgs += e.type == V8BIS_MEV_RX_MESSAGE;
                bad += e.type == V8BIS_MEV_RX_BAD_FRAME;
            }
        }
        v8bis_modem_free(m);
    }
    printf("  12 minutes of random audio: %ld events (%ld signals, %ld messages, %ld bad frames)\n", events, sigs, msgs, bad);
    CHECK(sigs == 0 && msgs == 0);
}

int main(void)
{
    test_table7_audio();
    test_waveform();
    test_line_conditions();
    test_bad_input();
    test_no_peer();
    test_random_audio();
    test_audio_fuzz();
    printf("%d checks, %d failed\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
