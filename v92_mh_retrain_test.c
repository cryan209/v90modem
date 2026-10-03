/*
 * v92_mh_retrain_test.c — a retrain told apart from modem-on-hold, handed to
 * V.90 Phase 2 without losing the reversal that told them apart.
 *
 * Two SpanDSP V.34 instances in V.90 mode -- the analogue modem (Tone A) and
 * the digital modem (Tone B) -- run Phase 2 to Phase 3 over a G.711 line
 * with a one-way delay.  Then the analogue modem retrains (V.90 9.5.2), and
 * the digital modem answers through the V.92 modem-on-hold layer, as the
 * engine does when ME_V92_MH=1: its Tone RT goes out, the analogue modem's
 * first Tone A reversal comes back, and the controller calls it a retrain
 * (9.10.1.1).  The retrain must then complete: the analogue modem receives a
 * fresh INFO1d, the digital modem a fresh INFO1a, and Phase 3 starts.
 *
 *   fixed   : the MH layer answers the reversal itself (11.2.1.1.3, Tone B
 *             40 ms after it, 10 ms reversed) and hands V.34 the following
 *             silence with the reversal already counted;
 *   control : the hand-over before this fix -- v34_v90_start_retrain_response(),
 *             70 ms of silence and Tone B, waiting for a reversal that has
 *             already gone by.
 */
#include "v92_mh_line.h"

#include <spandsp.h>
#include <spandsp/private/logging.h>
#include <spandsp/private/bitstream.h>
#include <spandsp/private/power_meter.h>
#include <spandsp/private/v34.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define CHUNK 160
static int DELAY_CHUNKS = 2;           /* one way, in 20 ms chunks */
#define RING 8

static int dummy_get_bit(void *u) { (void)u; return 1; }
static void dummy_put_bit(void *u, int b) { (void)u; (void)b; }

static void line(int16_t *x, int n, bool alaw)
{
    for (int i = 0; i < n; i++)
        x[i] = alaw ? alaw_to_linear(linear_to_alaw(x[i]))
                    : ulaw_to_linear(linear_to_ulaw(x[i]));
}

typedef struct {
    int16_t buf[RING][CHUNK];
    int head;
} delay_t;

static void delay_io(delay_t *d, const int16_t *in, int16_t *out)
{
    int rd;

    rd = (d->head + RING - DELAY_CHUNKS) % RING;

    memcpy(out, d->buf[rd], sizeof(d->buf[rd]));
    memcpy(d->buf[d->head], in, sizeof(d->buf[d->head]));
    d->head = (d->head + 1) % RING;
}

/* Phase 2 complete, judged by stage TRANSITIONS seen from now on: events
 * and flags from the first startup are still set after a retrain, and a
 * check that reads them passes a retrain that never ran. */
typedef struct {
    bool caller_l1l2, caller_info1c, answerer_info1a, phase3;
    bool startup;                     /* first startup: events are fresh */
} progress_t;

static void note(progress_t *p, v34_state_t *caller, v34_state_t *answerer)
{
    int crx = v34_get_rx_stage(caller), arx = v34_get_rx_stage(answerer);

    p->caller_l1l2 |= crx == V34_RX_STAGE_L1_L2;
    p->caller_info1c |= p->caller_l1l2 && crx == V34_RX_STAGE_INFO1C;
    p->answerer_info1a |= arx == V34_RX_STAGE_INFO1A;
    p->phase3 |= p->caller_info1c && p->answerer_info1a
                 && (crx >= V34_RX_STAGE_PHASE3_TRAINING
                     || arx >= V34_RX_STAGE_PHASE3_TRAINING);
}

static bool done(const progress_t *p)
{
    return p->caller_l1l2 && p->caller_info1c && p->answerer_info1a && p->phase3;
}

static double rms(const int16_t *x)
{
    double e = 0;
    for (int i = 0; i < CHUNK; i++) e += (double)x[i] * x[i];
    return sqrt(e / CHUNK);
}

static bool run(bool alaw, bool fixed, int *reply_gap_ms, int *complete_ms)
{
    v34_state_t *caller, *answerer;
    v92_mh_ctrl_t mh;
    v92_mh_line_t ml;
    delay_t c2a, a2c;
    progress_t p;
    enum { STARTUP, MH, REPLY, V34 } mode = STARTUP;
    int chunk, retrain_at = -1;
    bool ok = false;

    memset(&c2a, 0, sizeof(c2a));
    memset(&a2c, 0, sizeof(a2c));
    caller = v34_init(NULL, 3200, 21600, true, true, dummy_get_bit, NULL, dummy_put_bit, NULL);
    answerer = v34_init(NULL, 3200, 21600, false, true, dummy_get_bit, NULL, dummy_put_bit, NULL);
    for (chunk = 0; chunk < 4; chunk++) {        /* V.8 CJ silence */
        int16_t a[CHUNK], b[CHUNK];
        v34_tx(caller, a, CHUNK);
        v34_tx(answerer, b, CHUNK);
        memset(a, 0, sizeof(a));
        v34_rx(caller, a, CHUNK);
        v34_rx(answerer, a, CHUNK);
    }
    v34_set_v90_mode(caller, alaw);
    v34_set_v90_mode(answerer, alaw);
    v34_set_v90_u_info(caller, 1);
    memset(&p, 0, sizeof(p));
    if (reply_gap_ms)
        *reply_gap_ms = -1;
    if (complete_ms)
        *complete_ms = -1;

    for (chunk = 0; chunk < 3000; chunk++) {     /* 60 s */
        int16_t ctx[CHUNK], atx[CHUNK], crx[CHUNK], arx[CHUNK];
        v92_mh_action_t act;

        v34_tx(caller, ctx, CHUNK);
        if (mode == STARTUP || mode == V34) {
            v34_tx(answerer, atx, CHUNK);
        } else if (mode == MH) {
            if (!v92_mh_line_tx(&ml, &mh, atx, CHUNK))
                memset(atx, 0, sizeof(atx));     /* data mode stopped */
        } else {
            int n = v92_mh_line_retrain_reply_fill(&ml, atx, CHUNK);

            if (n < CHUNK) {
                if (reply_gap_ms)
                    *reply_gap_ms = (int)((ml.rx_samples - ml.rev_sample + (uint64_t)n) / 8);
                if (getenv("V92_MH_RETRAIN_DEBUG"))
                    fprintf(stderr, "  reply done: rx_samples=%llu rev_sample=%llu n=%d\n", (unsigned long long)ml.rx_samples, (unsigned long long)ml.rev_sample, n);
                v34_v90_retrain_first_b_silence(answerer);
                v34_tx(answerer, atx + n, CHUNK - n);
                mode = V34;
            }
        }
        line(ctx, CHUNK, alaw);
        line(atx, CHUNK, alaw);
        delay_io(&c2a, ctx, arx);
        delay_io(&a2c, atx, crx);
        v34_rx(caller, crx, CHUNK);

        if (mode == MH) {
            v92_mh_line_rx(&ml, &mh, arx, CHUNK);
            while ((act = v92_mh_ctrl_take_action(&mh)) != V92_MH_ACT_NONE) {
                if (act != V92_MH_ACT_RETRAIN)
                    continue;
                v34_restart(answerer, 3200, 21600, true);
                v34_set_v90_mode(answerer, alaw);
                if (fixed && mh.retrain_by_reversal) {
                    v34_v90_retrain_after_reversal(answerer);
                    v92_mh_line_retrain_reply(&ml, (int)(ml.rx_samples - ml.rev_sample));
                    mode = REPLY;
                } else {
                    v34_v90_start_retrain_response(answerer);
                    mode = V34;
                }
            }
        } else {
            v34_rx(answerer, arx, CHUNK);
        }

        if (getenv("V92_MH_RETRAIN_STAGES")) {
            static int last[4] = {-1, -1, -1, -1};
            int now4[4] = { v34_get_tx_stage(caller), v34_get_rx_stage(caller),
                            v34_get_tx_stage(answerer), v34_get_rx_stage(answerer) };
            if (memcmp(now4, last, sizeof(now4)) != 0)
                fprintf(stderr, "    %.3f mode=%d mh=%s caller tx=%d rx=%d | answerer tx=%d rx=%d\n",
                        chunk / 50.0, mode, v92_mh_state_name(mh.state),
                        now4[0], now4[1], now4[2], now4[3]);
            memcpy(last, now4, sizeof(now4));
        }
        if (getenv("V92_MH_RETRAIN_TRACE") && chunk % 25 == 0)
            fprintf(stderr, "    t=%.1f mode=%d caller tx=%d rx=%d ev=%d | answerer tx=%d rx=%d ev=%d rms c=%.0f a=%.0f\n", chunk / 50.0, mode,
                    v34_get_tx_stage(caller), v34_get_rx_stage(caller), v34_get_rx_event(caller),
                    v34_get_tx_stage(answerer), v34_get_rx_stage(answerer), v34_get_rx_event(answerer),
                    rms(crx), rms(arx));
        if (mode == STARTUP) {
            note(&p, caller, answerer);
            if (done(&p)) {
                /* The analogue modem retrains (V.90 9.5.2); the digital
                 * side is now the V.92 modem-on-hold layer. */
                /* As restart_v90_analogue_phase2_locked() does. */
                v34_restart(caller, 3200, 21600, true);
                v34_set_v90_mode(caller, alaw);
                v34_set_v90_u_info(caller, 1);
                v34_v90_start_analogue_retrain(caller);
                v92_mh_ctrl_init(&mh, 2 * DELAY_CHUNKS * 20);
                v92_mh_line_init(&ml, false, -12.0);
                memset(&p, 0, sizeof(p));
                mode = MH;
                if (getenv("V92_MH_BASELINE")) {
                    v34_restart(answerer, 3200, 21600, true);
                    v34_set_v90_mode(answerer, alaw);
                    v34_v90_start_retrain_response(answerer);
                    mode = V34;
                }
                retrain_at = chunk;
            }
        } else if (mode == V34) {
            note(&p, caller, answerer);
            if (done(&p)) {
                ok = true;
                if (complete_ms)
                    *complete_ms = (chunk - retrain_at) * 20;
                break;
            }
        }
    }
    if (getenv("V92_MH_RETRAIN_DEBUG"))
        fprintf(stderr, "  %s %s: retrain at %d s, mode %d, caller tx=%d rx=%d answerer tx=%d rx=%d, %s\n",
                alaw ? "A-law" : "u-law", fixed ? "fixed" : "control",
                retrain_at / 50, mode,
                v34_get_tx_stage(caller), v34_get_rx_stage(caller),
                v34_get_tx_stage(answerer), v34_get_rx_stage(answerer),
                ok ? "completed" : "stalled");
    v34_free(caller);
    v34_free(answerer);
    return ok && retrain_at >= 0;
}

int main(void)
{
    int fails = 0;

    for (DELAY_CHUNKS = 1; DELAY_CHUNKS <= 3; DELAY_CHUNKS++)
    for (int alaw = 0; alaw < 2; alaw++) {
        int gap, t_fixed, t_control;
        bool fixed = run(alaw, true, &gap, &t_fixed);
        bool control = run(alaw, false, NULL, &t_control);
        /* gap runs to the end of the 10 ms reversed segment */
        bool timing = gap - 10 >= 38 && gap - 10 <= 46;

        printf("%s: %d ms one way, %s retrain answered through the MH layer completes Phase 2 in %d ms; "
               "our Tone B reversal %d ms after the peer's (11.2.1.1.3: 40)\n",
               fixed && timing ? "PASS" : "FAIL", DELAY_CHUNKS * 20, alaw ? "A-law" : "u-law",
               t_fixed, gap - 10);
        printf("      control (old hand-over, silence + fresh Tone B): %s in %d ms\n",
               control ? "completes" : "stalls", t_control);
        if (!fixed || !timing)
            fails++;
    }
    if (fails) {
        printf("v92_mh_retrain_test: %d FAILURES\n", fails);
        return 1;
    }
    puts("v92_mh_retrain_test: OK");
    return 0;
}
