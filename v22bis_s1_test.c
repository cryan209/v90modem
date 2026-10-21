/*
 * v22bis_s1_test.c -- the V.22bis answer modem must find the caller's S1.
 *
 * V.22bis 6.3.1.1.2 b): an answer modem that receives the unscrambled
 * repetitive double dibit 00 and 11 (S1) answers with S1 and goes on to
 * 2400 bit/s; one that receives only scrambled binary 1 for 270 ms goes to
 * 1200 (6.3.1.2.2 b)).  S1 is 100 ms, and SpanDSP's answerer spends the
 * first 66 ms of it acquiring symbol timing, so it has about 25 symbols to
 * find it in, before its equalizer has trained.
 *
 * A live CX93001 (AT+MS=V22B) sent S1 on every call, but on 3 of 4 calls
 * (docs/carrier_loss.md, 2026-10-21) its two S1 points arrived about 70
 * degrees apart and both inside one slicer region.  The answerer read S1 as
 * 01 01 01, committed to 1200, and the CX (which went to 2400) never
 * connected.  This checks, on the datapump alone:
 *
 *   1. each of those four calls' received signal (12.9-14.9 s of the
 *      receive tap, reference-data/artifacts/cx93001-v22bis-s1) trains a
 *      fresh 2400 bit/s answerer at 2400;
 *   2. a SpanDSP V.22bis caller into the answerer over a short dispersive
 *      channel trains at 2400 for every one-way delay from 0 to 14 samples
 *      (each sample turns the 1200 Hz carrier 54 degrees against the symbol
 *      timing, so the sweep puts S1 at every angle to the slicer).
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <spandsp.h>

#define FRAME   160
#define FIXTURE_DIR "reference-data/artifacts/cx93001-v22bis-s1"

static int failures;

static void check(int ok, const char *what)
{
    printf("  %s %s\n", ok ? "ok  " : "FAIL", what);
    if (!ok)
        failures++;
}

static int ones_bit(void *user)
{
    (void) user;
    return 1;
}

static void drop_bit(void *user, int bit)
{
    (void) user;
    (void) bit;
}

/* The rate the datapump reports when it says it has trained, 0 if not. */
typedef struct {
    v22bis_state_t *m;
    int trained_rate;
} watch_t;

static void status(void *user, int st)
{
    watch_t *w = user;

    if (st == SIG_STATUS_TRAINING_SUCCEEDED && w->trained_rate == 0)
        w->trained_rate = v22bis_get_current_bit_rate(w->m);
}

static v22bis_state_t *make_modem(bool caller, watch_t *w)
{
    v22bis_state_t *m = v22bis_init(NULL, 2400, V22BIS_GUARD_TONE_NONE, caller,
                                    ones_bit, NULL, drop_bit, NULL);

    if (!m) {
        fprintf(stderr, "v22bis_init failed\n");
        exit(2);
    }
    w->m = m;
    w->trained_rate = 0;
    v22bis_set_modem_status_handler(m, status, w);
    return m;
}

static int fixture_rate(const char *name)
{
    char path[512];
    uint8_t ulaw[FRAME];
    int16_t rx[FRAME];
    int16_t tx[FRAME];
    watch_t w;
    v22bis_state_t *m;
    FILE *f;
    size_t n;

    snprintf(path, sizeof(path), FIXTURE_DIR "/%s", name);
    if (!(f = fopen(path, "rb"))) {
        perror(path);
        exit(2);
    }
    m = make_modem(false, &w);
    while ((n = fread(ulaw, 1, FRAME, f)) > 0) {
        for (size_t i = 0; i < n; i++)
            rx[i] = ulaw_to_linear(ulaw[i]);
        v22bis_rx(m, rx, (int) n);
        v22bis_tx(m, tx, (int) n);
    }
    fclose(f);
    v22bis_free(m);
    return w.trained_rate;
}

/* Caller into answerer over h[] with `delay` samples of one-way delay each
   way.  Returns the answerer's trained rate in *ans and the caller's in
   *call. */
static void pair_rates(int delay, int *call, int *ans)
{
    static const float h[] = {1.0f, 0.45f, -0.2f, 0.1f};
    enum { HL = sizeof(h)/sizeof(h[0]), SECONDS = 4, DMAX = 64 };
    watch_t wc;
    watch_t wa;
    v22bis_state_t *c = make_modem(true, &wc);
    v22bis_state_t *a = make_modem(false, &wa);
    float c_hist[DMAX + HL] = {0};
    float a_hist[DMAX + HL] = {0};
    int16_t ct[FRAME];
    int16_t at[FRAME];
    int16_t cr[FRAME];
    int16_t ar[FRAME];

    for (int t = 0; t < SECONDS*8000/FRAME; t++) {
        v22bis_tx(c, ct, FRAME);
        v22bis_tx(a, at, FRAME);
        for (int i = 0; i < FRAME; i++) {
            float yc = 0.0f;
            float ya = 0.0f;

            memmove(c_hist + 1, c_hist, sizeof(c_hist) - sizeof(c_hist[0]));
            memmove(a_hist + 1, a_hist, sizeof(a_hist) - sizeof(a_hist[0]));
            c_hist[0] = ct[i];
            a_hist[0] = at[i];
            for (int k = 0; k < HL; k++) {
                yc += h[k]*c_hist[delay + k];
                ya += h[k]*a_hist[delay + k];
            }
            ar[i] = (int16_t) (yc*0.7f);
            cr[i] = (int16_t) (ya*0.7f);
        }
        v22bis_rx(a, ar, FRAME);
        v22bis_rx(c, cr, FRAME);
    }
    *call = wc.trained_rate;
    *ans = wa.trained_rate;
    v22bis_free(c);
    v22bis_free(a);
}

int main(void)
{
    static const char *calls[] = {"call1-rx.ulaw", "call2-rx.ulaw", "call3-rx.ulaw", "call4-rx.ulaw"};
    char what[128];

    printf("CX93001 S1, live receive taps (V.22bis 6.3.1.1.2 b)):\n");
    for (int i = 0; i < 4; i++) {
        int rate = fixture_rate(calls[i]);

        snprintf(what, sizeof(what), "%s: answerer trained at %d (want 2400)", calls[i], rate);
        check(rate == 2400, what);
    }

    printf("caller -> answerer, dispersive channel, delay sweep:\n");
    for (int d = 0; d <= 14; d++) {
        int call;
        int ans;

        pair_rates(d, &call, &ans);
        snprintf(what, sizeof(what), "delay %2d: caller %d, answerer %d (want 2400, 2400)", d, call, ans);
        check(call == 2400 && ans == 2400, what);
    }

    if (failures) {
        printf("v22bis_s1_test: %d FAILED\n", failures);
        return 1;
    }
    printf("v22bis_s1_test: all passed\n");
    return 0;
}
