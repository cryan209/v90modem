/*
 * v92_mh_line_test.c — V.92 modem-on-hold at the waveform level.
 *
 * Two controllers, each behind a v92_mh_line, over a G.711 mu-law line with
 * a 60 ms one-way delay, each side also hearing its own signal back at
 * -20 dB (a hybrid).  Audio moves in 160-sample frames, as the SIP media
 * path delivers it.  One side has Tone A as its retrain tone and the other
 * Tone B (8.9.1).  Every outcome is decided by what the detectors read off
 * the line: Figures 20-24, a retrain reversal told apart from MH (Cor.1
 * 9.7.1.2 NOTE), and an initiator abandoned by a silent peer.
 */
#include "v92_mh_line.h"

#include <spandsp.h>

#include <assert.h>
#include <stdio.h>
#include <string.h>

#define FRAME 160
#define DELAY 480                     /* samples, 60 ms */
#define LINE (1 << 18)

typedef struct {
    v92_mh_ctrl_t c;
    v92_mh_line_t l;
    v92_mh_action_t acts[16];
    int act_ms[16];
    int nact;
    int16_t sent[LINE];               /* what this side put on the line */
    bool mute;                        /* transmit nothing (peer that is not there) */
    bool tone_only;                   /* transmit raw Tone RT: a retraining peer */
    int reverse_at;                   /* sample at which tone_only reverses (once) */
    double tphase, tsign;
} side_t;

static int now;                       /* samples */

static int16_t g711(int16_t x)
{
    return ulaw_to_linear(linear_to_ulaw(x));
}

static void side_init(side_t *s, bool tone_a)
{
    memset(s, 0, sizeof(*s));
    v92_mh_ctrl_init(&s->c, 2 * DELAY / 8);
    v92_mh_line_init(&s->l, tone_a, -12.0);
    s->tsign = 1.0;
    s->reverse_at = -1;
}

static void side_tx(side_t *s, int16_t *out)
{
    if (s->mute) {
        memset(out, 0, FRAME * sizeof(*out));
    } else if (s->tone_only) {
        for (int i = 0; i < FRAME; i++) {
            if (now + i == s->reverse_at)
                s->tsign = -s->tsign;
            out[i] = (int16_t)(s->tsign * 8000.0 * cos(2.0 * M_PI * s->tphase));
            s->tphase += s->l.own_hz / 8000.0;
            if (s->tphase >= 1.0) s->tphase -= 1.0;
        }
    } else if (!v92_mh_line_tx(&s->l, &s->c, out, FRAME)) {
        /* data mode: a busy wideband stand-in */
        static uint32_t seed = 1;
        for (int i = 0; i < FRAME; i++) {
            seed = seed * 1103515245u + 12345u;
            out[i] = (int16_t)((int)((seed >> 16) & 0x1FFF) - 0x1000);
        }
    }
    for (int i = 0; i < FRAME; i++) {
        out[i] = g711(out[i]);
        s->sent[now + i] = out[i];
    }
}

static void side_rx(side_t *s, const side_t *o)
{
    int16_t in[FRAME];
    v92_mh_action_t a;

    if (s->mute || s->tone_only)
        return;
    for (int i = 0; i < FRAME; i++) {
        int t = now + i;
        int far = t >= DELAY ? o->sent[t - DELAY] : 0;
        int own = s->sent[t] / 10;                 /* -20 dB hybrid */
        in[i] = g711((int16_t)(far + own));
    }
    v92_mh_line_rx(&s->l, &s->c, in, FRAME);
    while ((a = v92_mh_ctrl_take_action(&s->c)) != V92_MH_ACT_NONE) {
        assert(s->nact < 16);
        s->acts[s->nact] = a;
        s->act_ms[s->nact++] = now / 8;
    }
}

static void run(side_t *a, side_t *b, int ms)
{
    int16_t fa[FRAME], fb[FRAME];

    for (int end = now + ms * 8; now < end; now += FRAME) {
        assert(now + FRAME < LINE);
        side_tx(a, fa);
        side_tx(b, fb);
        side_rx(a, b);
        side_rx(b, a);
    }
}

static bool has(const side_t *s, v92_mh_action_t a, int *at)
{
    for (int i = 0; i < s->nact; i++)
        if (s->acts[i] == a) {
            if (at) *at = s->act_ms[i];
            return true;
        }
    return false;
}

static side_t A, B;                   /* A: Tone A (analogue), B: Tone B (digital) */

static void start(void)
{
    now = 0;
    side_init(&A, true);
    side_init(&B, false);
    run(&A, &B, 200);                 /* data mode */
}

static void dump(const char *what)
{
    printf("  %s: A %s tx=%d nact=%d bits=%u frames=%u | B %s tx=%d nact=%d bits=%u frames=%u\n",
           what, v92_mh_state_name(A.c.state), A.c.tx, A.nact, A.l.bits, A.c.rx.frames,
           v92_mh_state_name(B.c.state), B.c.tx, B.nact, B.l.bits, B.c.rx.frames);
}

static void check(bool ok, const char *label)
{
    if (!ok) {
        dump(label);
        printf("FAIL: %s\n", label);
        assert(0);
    }
    printf("PASS: %s\n", label);
}

int main(void)
{
    int t1, t2;

    /* Figure 20, both directions of initiative. */
    for (int dir = 0; dir < 2; dir++) {
        side_t *ini = dir ? &B : &A, *rsp = dir ? &A : &B;
        start();
        rsp->c.t1_code = 0x1;          /* 10 s */
        assert(v92_mh_ctrl_initiate(&ini->c, V92_MH_REQ, 0));
        run(&A, &B, 3000);
        check(has(ini, V92_MH_ACT_ON_HOLD, NULL) && has(rsp, V92_MH_ACT_ON_HOLD, NULL)
              && rsp->c.state == V92_MH_ST_ON_HOLD && rsp->c.tx == V92_MH_TX_ANSAM,
              dir ? "Figure 20 over the line, Tone B side initiates"
                  : "Figure 20 over the line, Tone A side initiates");
        run(&A, &B, 9000);
        check(has(rsp, V92_MH_ACT_DISCONNECT, &t1) && t1 < 12500,
              "  on-hold T1 (10 s) expires to disconnect");
    }

    start();
    B.c.grant = false;
    A.c.after_nack_reconnect = false;
    assert(v92_mh_ctrl_initiate(&A.c, V92_MH_REQ, 0));
    run(&A, &B, 3000);
    check(has(&A, V92_MH_ACT_DISCONNECT, NULL) && has(&B, V92_MH_ACT_DISCONNECT, NULL),
          "Figure 21 over the line: MHnack then MHcda, both disconnect");

    start();
    B.c.grant = false;
    assert(v92_mh_ctrl_initiate(&A.c, V92_MH_REQ, 0));
    run(&A, &B, 4000);
    check(has(&B, V92_MH_ACT_PHASE1_ANSWER, &t1) && has(&A, V92_MH_ACT_PHASE1_CALL, &t2)
          && t2 - t1 >= 1000,
          "Figure 22 over the line: MHnack, MHfrr, ANSam held 1 s, Phase 1");

    start();
    assert(v92_mh_ctrl_initiate(&B.c, V92_MH_CLRD, V92_MH_CLRD_OTHER));
    run(&A, &B, 3000);
    check(has(&A, V92_MH_ACT_DISCONNECT, NULL) && has(&B, V92_MH_ACT_DISCONNECT, NULL),
          "Figure 23 over the line: MHclrd/MHcda, both disconnect");

    start();
    assert(v92_mh_ctrl_initiate(&A.c, V92_MH_FRR, 0));
    run(&A, &B, 3000);
    check(has(&B, V92_MH_ACT_PHASE1_ANSWER, NULL) && has(&A, V92_MH_ACT_PHASE1_CALL, NULL),
          "Figure 24 over the line: MHfrr, ANSam, Phase 1");

    /* A retraining peer: Tone A, then one reversal.  B must take it as a
     * retrain, not as MH. */
    start();
    A.tone_only = true;
    A.reverse_at = now + 8 * 400;
    run(&A, &B, 1000);
    check(has(&B, V92_MH_ACT_RETRAIN, &t1) && B.c.rx.frames == 0,
          "retrain reversal is not mistaken for MH (Cor.1 9.7.1.2 NOTE)");

    /* The same tone with no reversal and no MH: B times out to a retrain. */
    start();
    A.tone_only = true;
    run(&A, &B, 4000);
    check(has(&B, V92_MH_ACT_RETRAIN, NULL), "steady peer RT with no MH ends in a retrain");

    /* A silent peer: the initiator gives up. */
    start();
    B.mute = true;
    assert(v92_mh_ctrl_initiate(&A.c, V92_MH_REQ, 0));
    run(&A, &B, 4000);
    check(has(&A, V92_MH_ACT_RETRAIN, NULL), "no response: initiator retrains (9.10.1.1)");

    /* Data mode alone must not start anything. */
    start();
    run(&A, &B, 5000);
    check(A.nact == 0 && B.nact == 0 && A.c.rx.frames == 0 && B.c.rx.frames == 0,
          "5 s of data mode: no false MH, RT or reversal");

    puts("v92_mh_line_test: all passed");
    return 0;
}
