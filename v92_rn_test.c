/*
 * v92_rn_test.c — V.92 9.8 rate renegotiation, 9.9 fast parameter exchange
 * and 9.11 cleardown, digital and analogue controllers against each other.
 *
 * The line carries whole units (R, bar, TRN chunk, SUV, CP, E, FB1, B1) with
 * a one-way delay; R is detected 64T after it starts, its bar at the end of
 * the bar, a frame when it ends.  Each scenario is a figure or a clause, and
 * is graded on the sequence each modem actually put on the line, not only on
 * where the controllers ended up.
 */
#include "v92_rn.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

#define STEP 6
#define DELAY 320                     /* 40 ms one way */
#define MAXEV 4096
#define MAXLOG 4096

typedef struct { long t; v92_rn_event_t ev; } qev_t;

typedef struct {
    v92_rn_t rn;
    v92_rn_unit_t cur;
    long cur_start, cur_left;
    qev_t q[MAXEV];                   /* events for the OTHER side */
    int qh, qt;
    v92_rn_unit_t log[MAXLOG];
    long log_t[MAXLOG];
    int nlog;
    v92_rn_action_t acts[32];
    long act_t[32];
    int nact;
    int drop_cp;                      /* drop this many of our CPs on the line */
} side_t;

static long now;
static side_t D, A;

static void push(side_t *s, long t, v92_rn_event_t ev)
{
    assert(s->qt < MAXEV);
    s->q[s->qt].t = t;
    s->q[s->qt].ev = ev;
    s->qt++;
}

static void start_unit(side_t *s)
{
    v92_rn_next(&s->rn, &s->cur);
    s->cur_start = now;
    s->cur_left = s->cur.kind == V92_RN_TX_DATA ? STEP : s->cur.symbols;
    if (s->cur.kind != V92_RN_TX_DATA) {
        assert(s->nlog < MAXLOG);
        s->log[s->nlog] = s->cur;
        s->log_t[s->nlog++] = now;
    }
    if (s->cur.kind == V92_RN_TX_R) {
        v92_rn_event_t e = { V92_RN_EV_R, s->cur.r, false, false, 0 };
        push(s, now + 64 + DELAY, e);
    }
}

static void end_unit(side_t *s)
{
    v92_rn_event_t e;
    long t = now + DELAY;

    memset(&e, 0, sizeof(e));
    switch (s->cur.kind) {
    case V92_RN_TX_RBAR: e.kind = V92_RN_EV_RBAR; e.r = s->cur.r; break;
    case V92_RN_TX_SUV:  e.kind = V92_RN_EV_SUV; e.ack = s->cur.ack; e.silence = s->cur.silence; break;
    case V92_RN_TX_CP:
        if (s->drop_cp > 0) { s->drop_cp--; return; }
        e.kind = V92_RN_EV_CP; e.ack = s->cur.ack; e.drn = s->cur.drn; break;
    case V92_RN_TX_E:    e.kind = V92_RN_EV_E; break;
    case V92_RN_TX_FB1:  e.kind = V92_RN_EV_FB1; break;
    case V92_RN_TX_B1:   e.kind = V92_RN_EV_B1; break;
    default: return;
    }
    push(s, t, e);
}

static void deliver(side_t *from, side_t *to)
{
    while (from->qh < from->qt && from->q[from->qh].t <= now) {
        v92_rn_rx(&to->rn, &from->q[from->qh].ev);
        from->qh++;
    }
}

static void collect(side_t *s)
{
    v92_rn_action_t a;

    while ((a = v92_rn_take_action(&s->rn)) != V92_RN_ACT_NONE) {
        assert(s->nact < 32);
        s->acts[s->nact] = a;
        s->act_t[s->nact++] = now;
    }
}

static void step(void)
{
    side_t *s[2] = { &D, &A };

    deliver(&D, &A);
    deliver(&A, &D);
    for (int i = 0; i < 2; i++) {
        if (s[i]->cur_left <= 0) {
            end_unit(s[i]);
            start_unit(s[i]);
        }
        s[i]->cur_left -= STEP;
        v92_rn_tick(&s[i]->rn, STEP);
        collect(s[i]);
    }
    now += STEP;
}

static void setup(void)
{
    memset(&D, 0, sizeof(D));
    memset(&A, 0, sizeof(A));
    now = 0;
    v92_rn_init(&D.rn, V92_RN_DIGITAL, 2 * DELAY);
    v92_rn_init(&A.rn, V92_RN_ANALOGUE, 2 * DELAY);
    D.rn.drn = 19;
    A.rn.drn = 13;
    for (int i = 0; i < 20; i++)
        step();                       /* data mode */
}

static void run(long symbols)
{
    for (long end = now + symbols; now < end; ) {
        step();
        if (D.rn.state == V92_RN_DONE && A.rn.state == V92_RN_DONE)
            break;
    }
}

static int count(const side_t *s, v92_rn_tx_kind_t k)
{
    int n = 0;
    for (int i = 0; i < s->nlog; i++)
        n += s->log[i].kind == k;
    return n;
}

static int sum(const side_t *s, v92_rn_tx_kind_t k)
{
    int n = 0;
    for (int i = 0; i < s->nlog; i++)
        if (s->log[i].kind == k)
            n += s->log[i].symbols;
    return n;
}

static bool has(const side_t *s, v92_rn_action_t a)
{
    for (int i = 0; i < s->nact; i++)
        if (s->acts[i] == a)
            return true;
    return false;
}

/* The unit kinds in order, runs collapsed, as a string for the figure. */
static const char *seq(const side_t *s)
{
    static char buf[2][512];
    static int which;
    char *b = buf[which ^= 1];
    static const char *nm[] = { "DATA", "R", "Rbar", "TRN", "SIL", "SUV", "CP", "E", "FB1", "B1" };
    static const char *rn[] = { "", "d", "u", "f", "M", "t" };
    int last = -1, lastack = -1;

    b[0] = 0;
    for (int i = 0; i < s->nlog; i++) {
        const v92_rn_unit_t *u = &s->log[i];
        int key = (int)u->kind * 10 + (int)u->r;

        if (key == last && u->ack == lastack)
            continue;
        last = key;
        lastack = u->ack;
        snprintf(b + strlen(b), 512 - strlen(b), "%s%s%s%s ",
                 nm[u->kind], (u->kind == V92_RN_TX_R || u->kind == V92_RN_TX_RBAR) ? rn[u->r] : "",
                 (u->kind == V92_RN_TX_SUV || u->kind == V92_RN_TX_CP) && u->ack ? "'" : "",
                 (u->kind == V92_RN_TX_SUV && u->silence) ? "(s)" : "");
    }
    return b;
}

static void ok(bool c, const char *label)
{
    if (!c) {
        printf("FAIL: %s\n  D [%s] %s\n  A [%s] %s\n", label,
               v92_rn_state_name(D.rn.state), seq(&D), v92_rn_state_name(A.rn.state), seq(&A));
        assert(0);
    }
    printf("PASS: %s\n      D: %s\n      A: %s\n", label, seq(&D), seq(&A));
}

static bool both_data(void)
{
    return has(&D, V92_RN_ACT_TX_DATA) && has(&D, V92_RN_ACT_RX_DATA)
        && has(&A, V92_RN_ACT_TX_DATA) && has(&A, V92_RN_ACT_RX_DATA)
        && !has(&D, V92_RN_ACT_DISCONNECT) && !has(&A, V92_RN_ACT_DISCONNECT);
}

int main(void)
{
    /* Figure 15: digital-initiated, no silence. */
    setup();
    assert(v92_rn_initiate(&D.rn, V92_RN_RENEG, false, false));
    run(80000);
    ok(both_data() && count(&D, V92_RN_TX_SILENCE) == 0 && count(&D, V92_RN_TX_CP) == 1
       && count(&A, V92_RN_TX_CP) == 1 && sum(&A, V92_RN_TX_TRN) >= 2400
       && D.rn.peer_drn == 13 && A.rn.peer_drn == 19,
       "Figure 15: rate renegotiation initiated by the digital modem, no silence");

    /* The same initiated by the analogue modem (9.8.2.1). */
    setup();
    assert(v92_rn_initiate(&A.rn, V92_RN_RENEG, false, false));
    run(80000);
    ok(both_data() && count(&A, V92_RN_TX_SILENCE) == 0,
       "9.8.2.1: rate renegotiation initiated by the analogue modem");

    /* Figure 16: silence requested by the digital modem, held to its length. */
    setup();
    D.rn.silence_symbols = 4008;
    assert(v92_rn_initiate(&D.rn, V92_RN_RENEG, true, false));
    run(120000);
    ok(both_data() && sum(&D, V92_RN_TX_SILENCE) >= 4008 && count(&D, V92_RN_TX_E) == 2
       && count(&A, V92_RN_TX_E) == 2 && sum(&A, V92_RN_TX_TRN) > 2400 + 4008,
       "Figure 16: silence requested by the digital modem, maximum length");

    /* Figure 17: the same, ended early by the digital modem's Rt. */
    setup();
    D.rn.silence_symbols = 1200;
    assert(v92_rn_initiate(&D.rn, V92_RN_RENEG, true, false));
    run(120000);
    {
        int a_trn2 = 0, seen_e = 0;
        for (int i = 0; i < A.nlog; i++) {
            if (A.log[i].kind == V92_RN_TX_E) seen_e = 1;
            else if (seen_e && A.log[i].kind == V92_RN_TX_TRN) a_trn2 += A.log[i].symbols;
        }
        ok(both_data() && sum(&D, V92_RN_TX_SILENCE) < 2400 && a_trn2 < 8004,
           "Figure 17: digital's silence ended early; analogue's TRN2u stops on Rt (9.8.2.1.6)");
    }

    /* Figure 18: digital initiates, the analogue modem asks for silence. */
    setup();
    A.rn.respond_silence = true;
    assert(v92_rn_initiate(&D.rn, V92_RN_RENEG, false, false));
    run(120000);
    ok(both_data() && count(&D, V92_RN_TX_E) == 2 && count(&D, V92_RN_TX_R) == 2,
       "Figure 18: silence requested by the analogue modem, digital waits for SUVu bit 32 clear");

    /* Figure 19: fast parameter exchange initiated by the analogue modem. */
    setup();
    assert(v92_rn_initiate(&A.rn, V92_RN_FPE, false, false));
    run(80000);
    ok(both_data() && count(&A, V92_RN_TX_FB1) == 1 && count(&A, V92_RN_TX_TRN) == 0
       && count(&D, V92_RN_TX_TRN) == 0 && has(&D, V92_RN_ACT_REINIT_SCRAMBLER)
       && has(&A, V92_RN_ACT_REINIT_SCRAMBLER),
       "Figure 19: fast parameter exchange initiated by the analogue modem (FB1u, no TRN)");

    setup();
    assert(v92_rn_initiate(&D.rn, V92_RN_FPE, false, false));
    run(80000);
    ok(both_data() && count(&A, V92_RN_TX_FB1) == 1,
       "9.9.1.1: fast parameter exchange initiated by the digital modem");

    /* 9.11 cleardown, via each procedure and from each side. */
    setup();
    assert(v92_rn_initiate(&D.rn, V92_RN_RENEG, false, true));
    run(80000);
    ok(has(&D, V92_RN_ACT_DISCONNECT) && has(&A, V92_RN_ACT_DISCONNECT)
       && count(&D, V92_RN_TX_B1) == 0 && count(&A, V92_RN_TX_B1) == 0
       && A.rn.peer_cleardown,
       "9.11: cleardown by the digital modem via rate renegotiation");
    {
        /* Amd.1 9.11 waits: the responder >= 100 ms + RTD/2 after its
         * acknowledged CP; the initiator no later than 100 ms + RTD after
         * its own CP. */
        long a_cp_end = -1, d_cp_end = -1, a_disc = -1, d_disc = -1;
        for (int i = 0; i < A.nlog; i++)
            if (A.log[i].kind == V92_RN_TX_CP && A.log[i].ack)
                a_cp_end = A.log_t[i] + A.log[i].symbols;
        for (int i = 0; i < D.nlog; i++)
            if (D.log[i].kind == V92_RN_TX_CP)
                d_cp_end = D.log_t[i] + D.log[i].symbols;
        /* act_t is stamped before the step's clock advances; the
         * controller acted on the clock after it. */
        for (int i = 0; i < A.nact; i++) if (A.acts[i] == V92_RN_ACT_DISCONNECT) a_disc = A.act_t[i] + STEP;
        for (int i = 0; i < D.nact; i++) if (D.acts[i] == V92_RN_ACT_DISCONNECT) d_disc = D.act_t[i] + STEP;
        ok(a_disc - a_cp_end >= 800 + DELAY && a_disc - a_cp_end <= 800 + DELAY + STEP
           && d_disc - d_cp_end <= 800 + 2 * DELAY + STEP,
           "9.11 (Amd.1): responder waits 100 ms + RTD/2 after its acknowledged CP, initiator at most 100 ms + RTD");
        printf("      responder %ld T after its CP', initiator %ld T after its CP\n",
               a_disc - a_cp_end, d_disc - d_cp_end);
    }

    setup();
    assert(v92_rn_initiate(&A.rn, V92_RN_FPE, false, true));
    run(80000);
    ok(has(&D, V92_RN_ACT_DISCONNECT) && has(&A, V92_RN_ACT_DISCONNECT)
       && count(&D, V92_RN_TX_B1) == 0 && D.rn.peer_cleardown,
       "9.11: cleardown by the analogue modem via fast parameter exchange");

    /* 9.11 forbids silence to the cleardown initiator; 9.9 has none. */
    setup();
    assert(!v92_rn_initiate(&D.rn, V92_RN_RENEG, true, true));
    assert(!v92_rn_initiate(&D.rn, V92_RN_FPE, true, false));
    puts("PASS: silence refused for a cleardown and for a fast parameter exchange");

    /* 9.6.x.1.3: the first CPd is lost; it is repeated after 100 ms + RTD. */
    setup();
    D.drop_cp = 1;
    assert(v92_rn_initiate(&D.rn, V92_RN_RENEG, false, false));
    run(80000);
    ok(both_data() && count(&D, V92_RN_TX_CP) >= 2,
       "9.6.1.1.3: unacknowledged CPd is repeated and the exchange completes");

    /* A fast parameter exchange that meets the peer's renegotiation follows
     * it (9.9.1.1.2). */
    setup();
    assert(v92_rn_initiate(&D.rn, V92_RN_FPE, false, false));
    assert(v92_rn_initiate(&A.rn, V92_RN_RENEG, false, false));
    run(80000);
    {
        bool rd = false;
        for (int i = 0; i < D.nlog; i++)
            rd |= D.log[i].kind == V92_RN_TX_R && D.log[i].r == V92_R_RD;
        ok(both_data() && rd && sum(&D, V92_RN_TX_TRN) >= 2040,
           "9.9.1.1.2: FPE initiator meeting the peer's Ru answers it with Rd and TRN2d");
    }

    puts("v92_rn_test: all passed");
    return 0;
}
