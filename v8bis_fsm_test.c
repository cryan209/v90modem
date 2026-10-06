/*
 * v8bis_fsm_test.c -- tests for the V.8bis transaction state machine.
 *
 * Two machines are wired to each other.  Every message between them goes
 * through the stage 1 chain -- encode, HDLC frame, bit-serial receive, FCS,
 * decode -- so a transaction here exercises exactly what a line would carry
 * above the V.21 modem.  The expected sequences are Table 7 of V.8bis, written
 * out by hand; the safety properties (agreement, no deadlock) are checked over
 * random configurations with injected frame corruption.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v8bis_fsm.h"
#include "v8bis_msg.h"

static int g_fail, g_checks;

#define CHECK(c)                                                                         \
    do {                                                                                 \
        g_checks++;                                                                      \
        if (!(c)) {                                                                      \
            if (++g_fail <= 15)                                                          \
                printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #c);                    \
        }                                                                                \
    } while (0)

static uint32_t g_rng = 0x1234abcdu;
static uint32_t rnd(void)
{
    g_rng ^= g_rng << 13;
    g_rng ^= g_rng >> 17;
    g_rng ^= g_rng << 5;
    return g_rng;
}

/* ---- the wire ----------------------------------------------------------- */

typedef struct {
    v8bis_fsm_t f;
    bool in_mode;
    v8bis_mode_t mode;
    unsigned initial_events;
    v8bis_why_t why;
    unsigned nak;
} station_t;

typedef struct {
    station_t s[2];                  /* 0 = "I", the tester that initiates; 1 = "R" */
    char trace[640];
    int corrupt_msg;                 /* corrupt the Nth message sent (1-based), 0 = none */
    int msg_count;
} wire_t;

typedef struct {
    int ok, bad;
    uint8_t info[V8BIS_MAX_INFO_OCTETS + 4];
    size_t len;
} rx_sink_t;

static void sink_cb(void *user, const uint8_t *info, size_t len, v8bis_frame_status_t st)
{
    rx_sink_t *s = user;

    if (st == V8BIS_FRAME_OK) {
        s->ok++;
        s->len = len;
        memcpy(s->info, info, len);
    } else {
        s->bad++;
    }
}

static void trace_add(wire_t *w, int from, const char *tok)
{
    size_t n = strlen(w->trace);

    snprintf(w->trace + n, sizeof(w->trace) - n, "%s%c:%s", n ? " " : "", from ? 'R' : 'I', tok);
}

static void station_init(station_t *s, const v8bis_fsm_cfg_t *cfg)
{
    memset(s, 0, sizeof(*s));
    v8bis_fsm_init(&s->f, cfg);
}

/* One message across encode -> frame -> bit-serial receive -> decode. */
static void carry(wire_t *w, int from, const v8bis_msg_t *m)
{
    uint8_t info[V8BIS_MAX_INFO_OCTETS], bits[V8BIS_MAX_FRAME_BITS + 200];
    int n = v8bis_msg_encode(m, info, sizeof(info));
    size_t nb;
    station_t *to = &w->s[!from];
    rx_sink_t sink;
    v8bis_frame_rx_t fr;
    v8bis_msg_t d;

    CHECK(n > 0);
    if (n <= 0)
        return;
    nb = v8bis_frame_encode(info, (size_t)n, V8BIS_PREAMBLE_BITS, 2, 1, bits, sizeof(bits));
    CHECK(nb > 0);
    w->msg_count++;
    if (w->corrupt_msg == w->msg_count)
        bits[V8BIS_PREAMBLE_BITS + 16 + 3] ^= 1;             /* inside the first information octet */
    memset(&sink, 0, sizeof(sink));
    v8bis_frame_rx_init(&fr, sink_cb, &sink);
    for (size_t i = 0; i < nb; i++)
        v8bis_frame_rx_bit(&fr, bits[i]);
    if (sink.ok != 1) {
        v8bis_fsm_invalid_frame(&to->f);
        return;
    }
    CHECK(v8bis_msg_decode(sink.info, sink.len, &d) == 0);
    v8bis_fsm_message(&to->f, &d);
}

static void run(wire_t *w)
{
    for (int guard = 0; guard < 200; guard++) {
        bool any = false;

        for (int from = 0; from < 2; from++) {
            station_t *s = &w->s[from];
            v8bis_action_t a;

            while (v8bis_fsm_next_action(&s->f, &a)) {
                any = true;
                switch (a.type) {
                case V8BIS_ACT_SIGNAL:
                    trace_add(w, from, v8bis_signal_name(a.sig));
                    v8bis_fsm_signal(&w->s[!from].f, a.sig, a.responding_set);
                    break;
                case V8BIS_ACT_MESSAGES:
                    if (a.es != V8BIS_ES_NONE) {
                        v8bis_signal_t es = a.es == V8BIS_ES_ESI ? V8BIS_SIG_ESI : V8BIS_SIG_ESR;
                        trace_add(w, from, v8bis_signal_name(es));
                        v8bis_fsm_signal(&w->s[!from].f, es, a.es == V8BIS_ES_ESR);
                    }
                    if (a.n_msg == 2) {
                        trace_add(w, from, "CL-MS");
                    } else {
                        trace_add(w, from, v8bis_msg_type_name(a.msg[0].type));
                    }
                    for (unsigned i = 0; i < a.n_msg; i++)
                        carry(w, from, &a.msg[i]);
                    break;
                case V8BIS_ACT_MS_MODE:
                    s->in_mode = true;
                    s->mode = a.mode;
                    break;
                case V8BIS_ACT_INITIAL:
                    s->initial_events++;
                    s->why = a.why;
                    s->nak = a.nak;
                    break;
                }
            }
        }
        if (!any)
            return;
    }
    CHECK(!"wire did not settle");
}

/* ---- configurations ------------------------------------------------------ */

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

static void cfg_for(v8bis_fsm_cfg_t *c, bool auto_call, bool answering)
{
    v8bis_fsm_cfg_default(c);
    caps_full(&c->caps);
    c->auto_answer_call = auto_call;
    c->answering_station = answering;
}

static void pair(wire_t *w, const v8bis_fsm_cfg_t *ci, const v8bis_fsm_cfg_t *cr)
{
    memset(w, 0, sizeof(*w));
    station_init(&w->s[0], ci);
    station_init(&w->s[1], cr);
}

static bool both_in_mode(const wire_t *w)
{
    return w->s[0].f.state == V8BIS_S_MS_MODE && w->s[1].f.state == V8BIS_S_MS_MODE;
}

static bool both_initial(const wire_t *w)
{
    return w->s[0].f.state == V8BIS_S_INITIAL && w->s[1].f.state == V8BIS_S_INITIAL;
}

static void expect_trace(const wire_t *w, const char *want, const char *what)
{
    g_checks++;
    if (strcmp(w->trace, want)) {
        if (++g_fail <= 15)
            printf("  FAIL %s:\n    want %s\n    got  %s\n", what, want, w->trace);
    }
}

/* ---- Table 7 -------------------------------------------------------------- */

typedef struct {
    int number;
    bool auto_call;
    v8bis_init_t how;
    v8bis_mr_reply_t mr_reply;           /* the responder's choice */
    v8bis_cr_reply_t cr_reply;
    bool i_on_mrd_ask_caps, on_crd_clr;  /* initiator's choices */
    bool r_on_crd_clr;
    const char *trace;
    int ms_from;                         /* which station sent the MS: 0 = I, 1 = R */
} txn_t;

static const txn_t TXNS[] = {
    /* answering-station context first (e signals, ESr) */
    {1, true, V8BIS_INIT_MR, V8BIS_MRR_MS, V8BIS_CRR_CL, 0, 0, 0,
     "I:MRe R:ESr R:MS I:ACK(1)", 1},
    {2, true, V8BIS_INIT_CR, V8BIS_MRR_MS, V8BIS_CRR_CL, 0, 0, 0,
     "I:CRe R:ESr R:CL I:MS R:ACK(1)", 0},
    {3, true, V8BIS_INIT_CR, V8BIS_MRR_MS, V8BIS_CRR_CLR, 0, 0, 0,
     "I:CRe R:ESr R:CLR I:CL-MS R:ACK(1)", 0},
    {4, true, V8BIS_INIT_MS, V8BIS_MRR_MS, V8BIS_CRR_CL, 0, 0, 0,
     "I:ESi I:MS R:ACK(1)", 0},
    {5, true, V8BIS_INIT_CL, V8BIS_MRR_MS, V8BIS_CRR_CL, 0, 0, 0,
     "I:ESi I:CL R:MS I:ACK(1)", 1},
    {6, true, V8BIS_INIT_CLR, V8BIS_MRR_MS, V8BIS_CRR_CL, 0, 0, 0,
     "I:ESi I:CLR R:CL I:MS R:ACK(1)", 0},
    {7, true, V8BIS_INIT_MR, V8BIS_MRR_MRD, V8BIS_CRR_CL, 0, 0, 0,
     "I:MRe R:MRd I:MS R:ACK(1)", 0},
    {8, true, V8BIS_INIT_MR, V8BIS_MRR_MRD, V8BIS_CRR_CL, 1, 0, 0,
     "I:MRe R:MRd I:CRd R:CL I:MS R:ACK(1)", 0},
    {9, true, V8BIS_INIT_MR, V8BIS_MRR_MRD, V8BIS_CRR_CL, 1, 0, 1,
     "I:MRe R:MRd I:CRd R:CLR I:CL-MS R:ACK(1)", 0},
    {10, true, V8BIS_INIT_MR, V8BIS_MRR_CRD, V8BIS_CRR_CL, 0, 0, 0,
     "I:MRe R:CRd I:CL R:MS I:ACK(1)", 1},
    {11, true, V8BIS_INIT_MR, V8BIS_MRR_CRD, V8BIS_CRR_CL, 0, 1, 0,
     "I:MRe R:CRd I:CLR R:CL-MS I:ACK(1)", 1},
    {12, true, V8BIS_INIT_CR, V8BIS_MRR_MS, V8BIS_CRR_CRD, 0, 0, 0,
     "I:CRe R:CRd I:CL R:MS I:ACK(1)", 1},
    {13, true, V8BIS_INIT_CR, V8BIS_MRR_MS, V8BIS_CRR_CRD, 0, 1, 0,
     "I:CRe R:CRd I:CLR R:CL-MS I:ACK(1)", 1},
    /* telephony context: d signals, no ES on a response, no dotted transitions */
    {1, false, V8BIS_INIT_MR, V8BIS_MRR_MS, V8BIS_CRR_CL, 0, 0, 0,
     "I:MRd R:MS I:ACK(1)", 1},
    {2, false, V8BIS_INIT_CR, V8BIS_MRR_MS, V8BIS_CRR_CL, 0, 0, 0,
     "I:CRd R:CL I:MS R:ACK(1)", 0},
    {3, false, V8BIS_INIT_CR, V8BIS_MRR_MS, V8BIS_CRR_CLR, 0, 0, 0,
     "I:CRd R:CLR I:CL-MS R:ACK(1)", 0},
    {4, false, V8BIS_INIT_MS, V8BIS_MRR_MS, V8BIS_CRR_CL, 0, 0, 0,
     "I:ESi I:MS R:ACK(1)", 0},
    {5, false, V8BIS_INIT_CL, V8BIS_MRR_MS, V8BIS_CRR_CL, 0, 0, 0,
     "I:ESi I:CL R:MS I:ACK(1)", 1},
    {6, false, V8BIS_INIT_CLR, V8BIS_MRR_MS, V8BIS_CRR_CL, 0, 0, 0,
     "I:ESi I:CLR R:CL I:MS R:ACK(1)", 0},
};

static void test_table7(void)
{
    printf("Table 7 transactions\n");
    for (unsigned i = 0; i < sizeof(TXNS) / sizeof(TXNS[0]); i++) {
        const txn_t *t = &TXNS[i];
        v8bis_fsm_cfg_t ci, cr;
        wire_t w;
        char what[48];
        int sender;

        cfg_for(&ci, t->auto_call, true);
        cfg_for(&cr, t->auto_call, false);
        ci.on_mrd_ask_caps = t->i_on_mrd_ask_caps;
        ci.on_crd_send_clr = t->on_crd_clr;
        cr.mr_reply = t->mr_reply;
        cr.cr_reply = t->cr_reply;
        cr.on_crd_send_clr = t->r_on_crd_clr;
        if (t->how == V8BIS_INIT_MS) {
            ci.have_preset_ms = true;
            v8bis_default_select_ms(&ci.caps, NULL, true, &ci.preset_ms);
        }
        pair(&w, &ci, &cr);
        snprintf(what, sizeof(what), "transaction %d%s", t->number, t->auto_call ? " (auto)" : "");
        CHECK(v8bis_fsm_initiate(&w.s[0].f, t->how));
        run(&w);
        expect_trace(&w, t->trace, what);
        CHECK(both_in_mode(&w));
        sender = w.s[0].mode.we_sent_ms ? 0 : 1;
        CHECK(w.s[0].mode.we_sent_ms != w.s[1].mode.we_sent_ms);
        CHECK(sender == t->ms_from);
        /* 9.9: the MS receiver is the answer modem, whichever end called */
        CHECK(w.s[sender].mode.answer_modem == false && w.s[!sender].mode.answer_modem == true);
        CHECK(w.s[0].mode.startup == V8BIS_STARTUP_SHORT_V8);       /* both support it, V.34 chosen */
        CHECK(w.s[0].mode.ms.data[1] == V8BIS_DATA2_V34);
        CHECK(w.s[0].mode.ms.data[0] == (V8BIS_DATA_V42 | V8BIS_DATA_V42BIS));
        CHECK(!memcmp(&w.s[0].mode.ms.data, &w.s[1].mode.ms.data, 3));
        CHECK(w.s[!sender].mode.ack_sent && w.s[sender].mode.ack_expected);
    }
}

/* ---- mode selection and refusal --------------------------------------------- */

static void test_selection(void)
{
    v8bis_fsm_cfg_t ci, cr;
    wire_t w;
    v8bis_msg_t ms;

    printf("mode selection\n");

    /* the best modulation both ends have, and the start-up their codepoints allow */
    cfg_for(&ci, false, false);
    cfg_for(&cr, false, false);
    ci.caps.data[1] = V8BIS_DATA2_V34 | V8BIS_DATA2_V32BIS;
    cr.caps.data[1] = V8BIS_DATA2_V32BIS;                   /* no V.34 */
    pair(&w, &ci, &cr);
    v8bis_fsm_initiate(&w.s[0].f, V8BIS_INIT_CR);
    run(&w);
    CHECK(both_in_mode(&w));
    CHECK(w.s[0].mode.ms.data[1] == V8BIS_DATA2_V32BIS && w.s[0].mode.ms.data[2] == 0);
    CHECK(w.s[0].mode.startup == V8BIS_STARTUP_V8);          /* short V.8 is recommended for V.34 only */

    cfg_for(&ci, false, false);
    cfg_for(&cr, false, false);
    ci.caps.id_npar1 = 0;                                   /* neither V.8 codepoint: V.25 (9.9.3) */
    cr.caps.id_npar1 = 0;
    pair(&w, &ci, &cr);
    v8bis_fsm_initiate(&w.s[0].f, V8BIS_INIT_CR);
    run(&w);
    CHECK(both_in_mode(&w) && w.s[0].mode.startup == V8BIS_STARTUP_V25);

    cfg_for(&ci, false, false);
    cfg_for(&cr, false, false);
    ci.caps.id_npar1 = V8BIS_ID_V8;                         /* only plain V.8 in common */
    pair(&w, &ci, &cr);
    v8bis_fsm_initiate(&w.s[0].f, V8BIS_INIT_CR);
    run(&w);
    CHECK(both_in_mode(&w) && w.s[0].mode.startup == V8BIS_STARTUP_V8);

    /* nothing in common: the station that must pick gives up, both end in Initial */
    cfg_for(&ci, false, false);
    cfg_for(&cr, false, false);
    ci.caps.data[0] = ci.caps.data[1] = 0;
    ci.caps.data[2] = V8BIS_DATA3_V21;
    cr.caps.data[2] = V8BIS_DATA3_V22;
    ci.caps.s_spar1 = cr.caps.s_spar1 = V8BIS_S_DATA;
    pair(&w, &ci, &cr);
    v8bis_fsm_initiate(&w.s[0].f, V8BIS_INIT_CR);
    run(&w);
    CHECK(w.s[0].f.state == V8BIS_S_INITIAL && w.s[0].why == V8BIS_WHY_NO_COMMON_MODE);

    /* analogue telephony selected: no modem start-up */
    cfg_for(&ci, false, false);
    cfg_for(&cr, false, false);
    ci.caps.s_spar1 = cr.caps.s_spar1 = V8BIS_S_ANALOGUE_TEL;
    pair(&w, &ci, &cr);
    v8bis_fsm_initiate(&w.s[0].f, V8BIS_INIT_CR);
    run(&w);
    CHECK(both_in_mode(&w) && w.s[0].mode.startup == V8BIS_STARTUP_TELEPHONY);

    /* the MS carries the minimum: modulation and protocols, nothing else (Table 7 note 2) */
    cfg_for(&ci, false, false);
    v8bis_default_select_ms(&ci.caps, &ci.caps, true, &ms);
    CHECK(ms.s_spar1 == V8BIS_S_DATA && ms.data[1] == V8BIS_DATA2_V34 && ms.data[2] == 0
          && ms.analogue_tel == 0 && !ms.network_type);
}

static v8bis_accept_t reject_all(void *u, const v8bis_msg_t *ms)
{
    (void)u;
    (void)ms;
    return V8BIS_ACCEPT_NAK3;
}

static void test_refusal(void)
{
    v8bis_fsm_cfg_t ci, cr;
    wire_t w;

    printf("NAK, ACK suppression, timeout, invalid frames\n");

    /* NAK(2): temporarily unable (9.5) -> both back to Initial, the sender is told which */
    cfg_for(&ci, false, false);
    cfg_for(&cr, false, false);
    cr.mode_available = false;
    pair(&w, &ci, &cr);
    v8bis_fsm_initiate(&w.s[0].f, V8BIS_INIT_CR);
    run(&w);
    expect_trace(&w, "I:CRd R:CL I:MS R:NAK(2)", "NAK(2)");
    CHECK(both_initial(&w));
    CHECK(w.s[0].why == V8BIS_WHY_NAK_RECEIVED && w.s[0].nak == 2);
    CHECK(w.s[1].why == V8BIS_WHY_NAK_SENT && w.s[1].nak == 2);

    /* NAK(3): not supported */
    cfg_for(&ci, false, false);
    cfg_for(&cr, false, false);
    cr.accept = reject_all;
    pair(&w, &ci, &cr);
    v8bis_fsm_initiate(&w.s[0].f, V8BIS_INIT_CR);
    run(&w);
    expect_trace(&w, "I:CRd R:CL I:MS R:NAK(3)", "NAK(3)");
    CHECK(both_initial(&w) && w.s[0].nak == 3);

    /* an MS asking for what the receiver lacks gets NAK(3) from the default test */
    cfg_for(&ci, false, false);
    cfg_for(&cr, false, false);
    cr.caps.data[1] = V8BIS_DATA2_V32BIS;                    /* no V.34 */
    ci.have_preset_ms = true;
    v8bis_default_select_ms(&ci.caps, NULL, true, &ci.preset_ms);   /* asks for V.34 */
    pair(&w, &ci, &cr);
    v8bis_fsm_initiate(&w.s[0].f, V8BIS_INIT_MS);
    run(&w);
    expect_trace(&w, "I:ESi I:MS R:NAK(3)", "unsupported MS");
    CHECK(both_initial(&w));

    /* 9.7: transmit ACK(1) = 0, no ACK; the sender proceeds when the start-up signal comes */
    cfg_for(&ci, false, false);
    cfg_for(&cr, false, false);
    ci.tx_ack1 = false;
    ci.have_preset_ms = true;
    v8bis_default_select_ms(&ci.caps, NULL, false, &ci.preset_ms);
    pair(&w, &ci, &cr);
    v8bis_fsm_initiate(&w.s[0].f, V8BIS_INIT_MS);
    run(&w);
    expect_trace(&w, "I:ESi I:MS", "ACK suppressed");
    CHECK(w.s[1].f.state == V8BIS_S_MS_MODE && !w.s[1].mode.ack_sent && w.s[1].mode.start_signal_next);
    CHECK(w.s[0].f.state == V8BIS_S_SENT_MS);                /* waiting to hear ANS/ANSam */
    v8bis_fsm_startup_signal(&w.s[0].f);
    run(&w);
    CHECK(w.s[0].f.state == V8BIS_S_MS_MODE && w.s[0].mode.we_sent_ms && !w.s[0].mode.ack_expected);
    /* ... but a NAK is sent whatever the bit says */
    cfg_for(&ci, false, false);
    cfg_for(&cr, false, false);
    cr.mode_available = false;
    ci.tx_ack1 = false;
    pair(&w, &ci, &cr);
    v8bis_fsm_initiate(&w.s[0].f, V8BIS_INIT_CR);
    run(&w);
    expect_trace(&w, "I:CRd R:CL I:MS R:NAK(2)", "NAK despite no ACK request");
    /* a startup signal with no suppressed MS outstanding does nothing */
    cfg_for(&ci, false, false);
    cfg_for(&cr, false, false);
    pair(&w, &ci, &cr);
    v8bis_fsm_initiate(&w.s[0].f, V8BIS_INIT_CR);
    v8bis_fsm_startup_signal(&w.s[0].f);
    CHECK(w.s[0].f.state == V8BIS_S_SENT_CR);

    /* 9.8: invalid frame -> NAK(1), back to Initial, and the other end follows */
    for (int victim = 1; victim <= 3; victim++) {
        cfg_for(&ci, false, false);
        cfg_for(&cr, false, false);
        pair(&w, &ci, &cr);
        w.corrupt_msg = victim;                 /* 1: R's CL, 2: I's MS, 3: R's ACK(1) */
        v8bis_fsm_initiate(&w.s[0].f, V8BIS_INIT_CR);
        run(&w);
        CHECK(strstr(w.trace, "NAK(1)") != NULL);
        CHECK(w.s[0].f.state == V8BIS_S_INITIAL);
        if (victim < 3) {
            CHECK(both_initial(&w));
        } else {
            /* the ACK(1) was lost: R has already gone to MS mode (start-up begins right
             * after ACK(1), 9.6) and the failure surfaces in the start-up procedure */
            CHECK(w.s[1].f.state == V8BIS_S_MS_MODE);
        }
    }

    /* 9.8: five seconds out of the Initial State and the station gives up */
    cfg_for(&ci, false, false);
    pair(&w, &ci, &ci);
    v8bis_fsm_initiate(&w.s[0].f, V8BIS_INIT_CR);
    run(&w);                                                  /* nobody answers: R sees the CR and replies... */
    {
        v8bis_fsm_t f;
        v8bis_action_t a;

        v8bis_fsm_init(&f, &ci);
        v8bis_fsm_initiate(&f, V8BIS_INIT_CR);
        while (v8bis_fsm_next_action(&f, &a))
            ;
        v8bis_fsm_tick(&f, 4999);
        CHECK(f.state == V8BIS_S_SENT_CR);
        v8bis_fsm_tick(&f, 1);
        CHECK(f.state == V8BIS_S_INITIAL);
        CHECK(v8bis_fsm_next_action(&f, &a) && a.type == V8BIS_ACT_INITIAL && a.why == V8BIS_WHY_TIMEOUT);
        v8bis_fsm_tick(&f, 100000);                           /* the Initial State has no timer */
        CHECK(!v8bis_fsm_next_action(&f, &a));
        /* every transition restarts the clock */
        v8bis_fsm_initiate(&f, V8BIS_INIT_CR);
        while (v8bis_fsm_next_action(&f, &a))
            ;
        v8bis_fsm_tick(&f, 4000);
        {
            v8bis_msg_t cl;

            cl = ci.caps;
            cl.type = V8BIS_MT_CL;
            cl.known_type = true;
            cl.id_npar1 |= V8BIS_ID_MORE_INFO;               /* CL/ACK(2): a loop in the same state */
            v8bis_fsm_message(&f, &cl);
        }
        v8bis_fsm_tick(&f, 4000);
        CHECK(f.state == V8BIS_S_SENT_CR);
    }
}

/* ---- segmentation (9.10) ------------------------------------------------------ */

static bool caps_equal_for_merge(const v8bis_msg_t *a, const v8bis_msg_t *b)
{
    return a->s_spar1 == b->s_spar1 && !memcmp(a->data, b->data, 3) && !memcmp(a->svd, b->svd, 3)
           && a->h324_npar2 == b->h324_npar2 && a->h324_spar2 == b->h324_spar2
           && a->h324_data == b->h324_data && a->v18 == b->v18 && a->analogue_tel == b->analogue_tel
           && a->t101 == b->t101 && a->ns_count == b->ns_count
           && (a->id_npar1 & (V8BIS_ID_V8 | V8BIS_ID_SHORT_V8))
                  == (b->id_npar1 & (V8BIS_ID_V8 | V8BIS_ID_SHORT_V8));
}

static void rich_caps(v8bis_msg_t *c)
{
    caps_full(c);
    c->s_spar1 |= V8BIS_S_SVD | V8BIS_S_H324 | V8BIS_S_V18 | V8BIS_S_T101;
    c->svd[0] = V8BIS_SVD_V70 | V8BIS_SVD_V34;
    c->svd[1] = V8BIS_DATA_TRANSPARENT | V8BIS_DATA_V42;
    c->h324_npar2 = V8BIS_H324_VIDEO | V8BIS_H324_AUDIO;
    c->h324_spar2 = V8BIS_H324_SPAR2_DATA;
    c->h324_data = V8BIS_H324D_V42 | V8BIS_H324D_PPP;
    c->v18 = V8BIS_V18_V21;
    c->t101 = V8BIS_T101_DUPLEX | V8BIS_T101_V27TER;
    c->id_npar1 |= V8BIS_ID_NON_STANDARD;
    c->ns_count = 2;
    c->ns[0].country = 0xb5;
    c->ns[0].provider_len = 2;
    c->ns[0].provider[0] = 0;
    c->ns[0].provider[1] = 9;
    c->ns[0].data_len = 6;
    memcpy(c->ns[0].data, "abcdef", 6);
    c->ns[1].country = 0x3c;
    c->ns[1].provider_len = 1;
    c->ns[1].provider[0] = 7;
    c->ns[1].data_len = 4;
    memcpy(c->ns[1].data, "wxyz", 4);
}

static void test_segmentation(void)
{
    v8bis_msg_t caps, segs[8], merged;
    unsigned n;

    printf("segmentation\n");
    rich_caps(&caps);

    /* the splitter: every segment fits, MORE_INFO on all but the last, and nothing is lost */
    for (unsigned lim = 10; lim <= 64; lim += 3) {
        uint8_t buf[V8BIS_MAX_INFO_OCTETS + 1];

        n = v8bis_caps_split(&caps, lim, segs, 8);
        CHECK(n >= 1);
        v8bis_msg_init(&merged, V8BIS_MT_CL);
        for (unsigned i = 0; i < n; i++) {
            int len = v8bis_msg_encode(&segs[i], buf, sizeof(buf));
            CHECK(len > 0);
            CHECK(len <= (int)lim || (segs[i].ns_count + __builtin_popcount(segs[i].s_spar1)) == 1);
            CHECK(((segs[i].id_npar1 & V8BIS_ID_MORE_INFO) != 0) == (i + 1 < n));
            v8bis_caps_merge(&merged, &segs[i]);
        }
        CHECK(caps_equal_for_merge(&merged, &caps));
        if (lim >= 64)
            CHECK(n == 1);
    }
    CHECK(v8bis_caps_split(&caps, 10, segs, 2) == 0);          /* does not fit in two */

    /* over the wire: the CL goes out in pieces and the far end ends up with all of it */
    {
        static const struct { v8bis_init_t how; const char *trace; } CASES[] = {
            {V8BIS_INIT_CR, NULL}, {V8BIS_INIT_CL, NULL}, {V8BIS_INIT_CLR, NULL}};
        for (unsigned c = 0; c < 3; c++) {
            v8bis_fsm_cfg_t ci, cr;
            wire_t w;
            int acks = 0;

            cfg_for(&ci, false, false);
            cfg_for(&cr, false, false);
            rich_caps(&ci.caps);
            rich_caps(&cr.caps);
            ci.max_info_octets = cr.max_info_octets = 14;
            if (CASES[c].how == V8BIS_INIT_CR)
                cr.cr_reply = V8BIS_CRR_CLR;                    /* CL-MS with segments, transaction 3 */
            pair(&w, &ci, &cr);
            v8bis_fsm_initiate(&w.s[0].f, CASES[c].how);
            run(&w);
            for (const char *p = w.trace; (p = strstr(p, "ACK(2)")); p++)
                acks++;
            CHECK(both_in_mode(&w));
            CHECK(acks > 0);
            /* each end learned the other's full list from the pieces */
            /* CR: R sends CLR and I answers CL-MS, so both lists cross; CL: only I's list is sent;
             * CLR: I's CLR and R's CL cross */
            CHECK(w.s[1].f.peer_caps_known && caps_equal_for_merge(&w.s[1].f.peer_caps, &ci.caps));
            if (CASES[c].how != V8BIS_INIT_CL)
                CHECK(w.s[0].f.peer_caps_known && caps_equal_for_merge(&w.s[0].f.peer_caps, &cr.caps));
            else
                CHECK(!w.s[0].f.peer_caps_known);
            printf("  %s in pieces: %s\n", c == 0 ? "transaction 3" : c == 1 ? "transaction 5" : "transaction 6", w.trace);
        }
    }

    /* an end that does not want the rest ignores MORE_INFO and never sends ACK(2) */
    {
        v8bis_fsm_cfg_t ci, cr;
        wire_t w;

        cfg_for(&ci, false, false);
        cfg_for(&cr, false, false);
        rich_caps(&ci.caps);
        ci.max_info_octets = 14;
        cr.want_more_info = false;
        pair(&w, &ci, &cr);
        v8bis_fsm_initiate(&w.s[0].f, V8BIS_INIT_CL);
        run(&w);
        CHECK(both_in_mode(&w) && !strstr(w.trace, "ACK(2)"));
    }
}

/* ---- Figure 14/15 details ---------------------------------------------------------- */

static void test_details(void)
{
    v8bis_fsm_cfg_t ci, cr;
    wire_t w;
    v8bis_fsm_t f;
    v8bis_action_t a;

    printf("details\n");

    /* e/d: only the answering station's first signal is subscript e */
    cfg_for(&ci, true, true);
    v8bis_fsm_init(&f, &ci);
    v8bis_fsm_initiate(&f, V8BIS_INIT_CR);
    CHECK(v8bis_fsm_next_action(&f, &a) && a.type == V8BIS_ACT_SIGNAL && a.sig == V8BIS_SIG_CRE
          && !a.responding_set);
    cfg_for(&ci, true, false);                               /* the caller initiating */
    v8bis_fsm_init(&f, &ci);
    v8bis_fsm_initiate(&f, V8BIS_INIT_MR);
    CHECK(v8bis_fsm_next_action(&f, &a) && a.sig == V8BIS_SIG_MRD);

    /* responses to MRe/CRe use the responding tone set, an initiator's the initiating one */
    cfg_for(&cr, true, false);
    cr.mr_reply = V8BIS_MRR_CRD;
    v8bis_fsm_init(&f, &cr);
    v8bis_fsm_signal(&f, V8BIS_SIG_MRE, false);
    CHECK(v8bis_fsm_next_action(&f, &a) && a.sig == V8BIS_SIG_CRD && a.responding_set);

    /* the dotted transitions need automatic answering: the same peer signal gets the plain reply */
    cfg_for(&cr, false, false);
    cr.mr_reply = V8BIS_MRR_CRD;
    v8bis_fsm_init(&f, &cr);
    v8bis_fsm_signal(&f, V8BIS_SIG_MRE, false);
    CHECK(v8bis_fsm_next_action(&f, &a) && a.type == V8BIS_ACT_MESSAGES && a.msg[0].type == V8BIS_MT_MS
          && a.es == V8BIS_ES_NONE);
    cfg_for(&cr, true, false);
    cr.mr_reply = V8BIS_MRR_CRD;
    v8bis_fsm_init(&f, &cr);
    v8bis_fsm_signal(&f, V8BIS_SIG_MRD, true);               /* MRd, not MRe: no dotted transition */
    CHECK(v8bis_fsm_next_action(&f, &a) && a.type == V8BIS_ACT_MESSAGES && a.es == V8BIS_ES_ESR);

    /* ES signals alone change nothing; 9.4's echo suppressor gap rides on the action */
    cfg_for(&ci, false, false);
    ci.echo_suppressor = true;
    v8bis_fsm_init(&f, &ci);
    v8bis_fsm_signal(&f, V8BIS_SIG_ESI, false);
    CHECK(f.state == V8BIS_S_INITIAL && !v8bis_fsm_next_action(&f, &a));
    v8bis_fsm_initiate(&f, V8BIS_INIT_CL);
    CHECK(v8bis_fsm_next_action(&f, &a) && a.es == V8BIS_ES_ESI && a.es_gap);

    /* unexpected events are ignored, not fatal: an MS in the middle of Sent CR, ACK in Initial */
    cfg_for(&ci, false, false);
    cfg_for(&cr, false, false);
    pair(&w, &ci, &cr);
    {
        v8bis_msg_t m;

        v8bis_msg_init(&m, V8BIS_MT_ACK1);
        m.known_type = true;
        v8bis_fsm_message(&w.s[0].f, &m);
        CHECK(w.s[0].f.state == V8BIS_S_INITIAL && !v8bis_fsm_next_action(&w.s[0].f, &a));
        v8bis_fsm_initiate(&w.s[0].f, V8BIS_INIT_CR);
        v8bis_fsm_next_action(&w.s[0].f, &a);
        v8bis_msg_init(&m, V8BIS_MT_MS);
        m.known_type = true;
        v8bis_fsm_message(&w.s[0].f, &m);
        CHECK(w.s[0].f.state == V8BIS_S_SENT_CR && !v8bis_fsm_next_action(&w.s[0].f, &a));
    }

    /* V.92's type 1011 and anything not in Table 3 are not ours to act on */
    {
        v8bis_msg_t m;

        v8bis_msg_init(&m, V8BIS_MT_NAK4);
        m.known_type = false;
        v8bis_fsm_init(&f, &ci);
        v8bis_fsm_initiate(&f, V8BIS_INIT_CR);
        v8bis_fsm_next_action(&f, &a);
        v8bis_fsm_message(&f, &m);
        CHECK(f.state == V8BIS_S_SENT_CR);
    }

    /* MS mode is final: nothing moves it, even an invalid frame */
    cfg_for(&ci, false, false);
    cfg_for(&cr, false, false);
    pair(&w, &ci, &cr);
    v8bis_fsm_initiate(&w.s[0].f, V8BIS_INIT_CR);
    run(&w);
    CHECK(both_in_mode(&w));
    v8bis_fsm_invalid_frame(&w.s[0].f);
    v8bis_fsm_tick(&w.s[0].f, 60000);
    CHECK(w.s[0].f.state == V8BIS_S_MS_MODE && !v8bis_fsm_next_action(&w.s[0].f, &a));
}

/* ---- random pairs: agreement and liveness ------------------------------------------------ */

static void random_cfg(v8bis_fsm_cfg_t *c, bool auto_call, bool answering)
{
    cfg_for(c, auto_call, answering);
    c->caps.id_npar1 = (uint8_t)(rnd() & 3);
    c->caps.data[0] = (uint8_t)(rnd() & 7);
    c->caps.data[1] = (uint8_t)(rnd() & 0x30);
    c->caps.data[2] = (uint8_t)(rnd() & 0x0f);
    if (rnd() % 3 == 0)
        c->caps.s_spar1 &= ~V8BIS_S_DATA;
    if (rnd() % 8 == 0)
        c->caps.s_spar1 = V8BIS_S_ANALOGUE_TEL;
    c->tx_ack1 = rnd() % 4 != 0;
    c->mode_available = rnd() % 6 != 0;
    c->want_more_info = rnd() % 5 != 0;
    c->max_info_octets = rnd() % 3 == 0 ? 8 + rnd() % 10 : 0;
    if (rnd() % 3 == 0)
        rich_caps(&c->caps), c->caps.data[1] = (uint8_t)(rnd() & 0x30);
    c->mr_reply = (v8bis_mr_reply_t)(rnd() % 3);
    c->cr_reply = (v8bis_cr_reply_t)(rnd() % 3);
    c->on_mrd_ask_caps = rnd() & 1;
    c->on_crd_send_clr = rnd() & 1;
    c->echo_suppressor = rnd() & 1;
}

static void test_random_pairs(void)
{
    int both_mode = 0, both_init = 0, runs = 20000, stuck = 0, disagree = 0;

    printf("random pairs\n");
    for (int r = 0; r < runs; r++) {
        v8bis_fsm_cfg_t ci, cr;
        wire_t w;
        bool auto_call = rnd() & 1;
        static const v8bis_init_t HOW[] = {V8BIS_INIT_MR, V8BIS_INIT_CR, V8BIS_INIT_MS,
                                           V8BIS_INIT_CL, V8BIS_INIT_CLR};
        v8bis_init_t how = HOW[rnd() % 5];

        random_cfg(&ci, auto_call, true);
        random_cfg(&cr, auto_call, false);
        if (how == V8BIS_INIT_MS) {
            ci.have_preset_ms = true;
            if (!v8bis_default_select_ms(&ci.caps, NULL, ci.tx_ack1, &ci.preset_ms))
                ci.have_preset_ms = false;
        }
        pair(&w, &ci, &cr);
        if (rnd() % 5 == 0)
            w.corrupt_msg = 1 + (int)(rnd() % 4);
        if (!v8bis_fsm_initiate(&w.s[0].f, how))
            continue;
        run(&w);
        /* an MS sent without an ACK request waits for the start-up signal the receiver would send */
        for (int side = 0; side < 2; side++)
            if (w.s[!side].f.state == V8BIS_S_MS_MODE && w.s[!side].mode.start_signal_next
                && !w.s[!side].mode.ack_sent && w.s[side].f.state == V8BIS_S_SENT_MS)
                v8bis_fsm_startup_signal(&w.s[side].f);
        run(&w);
        if (!both_in_mode(&w) && !both_initial(&w)) {
            /* somebody is waiting on a peer that has gone quiet: the 9.8 rule must free it */
            v8bis_fsm_tick(&w.s[0].f, 5000);
            v8bis_fsm_tick(&w.s[1].f, 5000);
            run(&w);
            if (w.s[0].f.state != V8BIS_S_MS_MODE && w.s[0].f.state != V8BIS_S_INITIAL)
                stuck++;
            if (w.s[1].f.state != V8BIS_S_MS_MODE && w.s[1].f.state != V8BIS_S_INITIAL)
                stuck++;
        }
        if (both_in_mode(&w)) {
            both_mode++;
            if (w.s[0].mode.we_sent_ms == w.s[1].mode.we_sent_ms
                || memcmp(w.s[0].mode.ms.data, w.s[1].mode.ms.data, 3)
                || w.s[0].mode.ms.id_npar1 != w.s[1].mode.ms.id_npar1
                || w.s[0].mode.startup != w.s[1].mode.startup)
                disagree++;
        } else if (both_initial(&w)) {
            both_init++;
        } else if ((w.s[0].f.state == V8BIS_S_MS_MODE) != (w.s[1].f.state == V8BIS_S_MS_MODE)) {
            /* one end in MS mode and the other not is only legal when the other was timed out */
            if (w.s[0].f.state != V8BIS_S_INITIAL && w.s[1].f.state != V8BIS_S_INITIAL)
                disagree++;
        }
    }
    printf("  %d runs: %d reached MS mode on both, %d ended in Initial on both, %d stuck, %d disagreed\n",
           runs, both_mode, both_init, stuck, disagree);
    CHECK(stuck == 0);
    CHECK(disagree == 0);
    CHECK(both_mode > runs / 5);
}

/* ---- arbitrary events never break it ------------------------------------------------------- */

static void test_event_fuzz(void)
{
    static const v8bis_msg_type_t TYPES[] = {V8BIS_MT_MS, V8BIS_MT_CL, V8BIS_MT_CLR, V8BIS_MT_ACK1,
                                             V8BIS_MT_ACK2, V8BIS_MT_NAK1, V8BIS_MT_NAK2,
                                             V8BIS_MT_NAK3, V8BIS_MT_NAK4};
    long events = 0, mode_entries = 0;

    printf("event fuzz\n");
    for (int run_i = 0; run_i < 400; run_i++) {
        v8bis_fsm_cfg_t c;
        v8bis_fsm_t f;
        v8bis_action_t a;

        random_cfg(&c, rnd() & 1, rnd() & 1);
        v8bis_fsm_init(&f, &c);
        for (int i = 0; i < 2000; i++) {
            unsigned k = rnd() % 10;

            events++;
            if (k < 2) {
                v8bis_fsm_signal(&f, (v8bis_signal_t)(rnd() % V8BIS_SIG_COUNT), rnd() & 1);
            } else if (k < 6) {
                v8bis_msg_t m;

                v8bis_msg_init(&m, TYPES[rnd() % 9]);
                m.known_type = m.type != V8BIS_MT_NAK4;
                if (m.type <= V8BIS_MT_CLR) {
                    m.id_npar1 = (uint8_t)(rnd() & 0x0f);
                    m.s_spar1 = (uint8_t)(rnd() & 0x6f);
                    m.data[1] = (uint8_t)(rnd() & 0x37);
                    m.data[2] = (uint8_t)(rnd() & 0x0f);
                }
                v8bis_fsm_message(&f, &m);
            } else if (k == 6) {
                v8bis_fsm_initiate(&f, (v8bis_init_t)(rnd() % 5));
            } else if (k == 7) {
                v8bis_fsm_invalid_frame(&f);
            } else if (k == 8) {
                v8bis_fsm_startup_signal(&f);
            } else {
                v8bis_fsm_tick(&f, rnd() % 3000);
            }
            while (v8bis_fsm_next_action(&f, &a)) {
                CHECK(a.type == V8BIS_ACT_SIGNAL || a.type == V8BIS_ACT_MESSAGES
                      || a.type == V8BIS_ACT_MS_MODE || a.type == V8BIS_ACT_INITIAL);
                if (a.type == V8BIS_ACT_MESSAGES) {
                    uint8_t buf[V8BIS_MAX_INFO_OCTETS + 1];

                    CHECK(a.n_msg == 1 || a.n_msg == 2);
                    for (unsigned j = 0; j < a.n_msg; j++)
                        CHECK(v8bis_msg_encode(&a.msg[j], buf, sizeof(buf)) > 0);
                }
                if (a.type == V8BIS_ACT_MS_MODE)
                    mode_entries++;
            }
            CHECK(f.state >= V8BIS_S_INITIAL && f.state <= V8BIS_S_MS_MODE);
            if (f.state == V8BIS_S_MS_MODE && rnd() % 50 == 0) {   /* start a fresh machine now and then */
                v8bis_fsm_init(&f, &c);
            }
        }
    }
    printf("  %ld events, %ld mode entries, every action encodable\n", events, mode_entries);
}

int main(void)
{
    test_table7();
    test_selection();
    test_refusal();
    test_segmentation();
    test_details();
    test_random_pairs();
    test_event_fuzz();
    printf("%d checks, %d failed\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
