/*
 * v90_analogue_sd_test.c — the scale-free Sd acquirer, and the same acquirer
 * fed through the T/2 equaliser from a 16 kHz stream.
 *
 * The point of the end-to-end case is that it is the situation the codeword
 * slicer cannot reach: a channel, an arbitrary sampling phase, and NO
 * calibration of any kind, which is exactly what a live analogue call presents
 * before Sd has been found.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "v90_analogue_sd.h"
#include "v90_analogue_fse.h"

static int failures;

static void check(bool ok, const char *what)
{
    printf("%s: %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok)
        failures++;
}

/* §8.4.4: +W, +0, +W, -W, -0, -W. */
static double sd_symbol(int i, double w)
{
    static const double pat[6] = { +1.0, 0.0, +1.0, -1.0, 0.0, -1.0 };

    return pat[i%6]*w;
}

static uint32_t rng = 12345;

static double urand(void)
{
    rng = rng*1103515245u + 12345u;
    return ((double) ((rng >> 8) & 0xFFFFFF))/16777216.0 - 0.5;
}

static void test_clean(double w, int start_slot)
{
    v90a_sd_t *s = v90a_sd_init(0);
    bool got = false;
    char msg[128];
    int i;

    for (i = 0; i < 6*64  &&  !got; i++)
        got = v90a_sd_put(s, sd_symbol(start_slot + i, w));
    snprintf(msg, sizeof msg, "clean Sd acquires at W=%.0f start slot %d",
             w, start_slot);
    check(got, msg);
    if (got) {
        /* The slot the NEXT symbol lands on is where the generator is. */
        /* §8.4.4's pattern is antisymmetric over three slots -- pat[i+3] is
         * -pat[i] -- so slot p and slot p+3 differ only by an overall sign,
         * which nothing here (or on a line that may invert) can recover.  The
         * grid is what matters; §9.3.2.4's Sd-to-S-bar-d boundary is found by
         * the CHANGE of sign, which is unambiguous. */
        int want = (start_slot + i)%3;

        snprintf(msg, sizeof msg, "  slot %d (want %d mod 3), level %.3f, score %.3f",
                 v90a_sd_next_slot(s), want, v90a_sd_level(s), v90a_sd_score(s));
        check(v90a_sd_next_slot(s)%3 == want, msg);
        check(fabs(v90a_sd_level(s) - w) < 0.02*w, "  level within 2%");
    }
    v90a_sd_free(s);
}

static void test_rejects(void)
{
    v90a_sd_t *s;
    int i;

    /* §8.4.5's TRN1d: scrambled ones on ONE level, so every slot is |L|. */
    s = v90a_sd_init(0);
    for (i = 0; i < 6*200; i++)
        (void) v90a_sd_put(s, (urand() > 0.0) ? 1.0 : -1.0);
    check(!v90a_sd_acquired(s), "TRN1d (one level, random signs) is rejected");
    v90a_sd_free(s);

    /* Silence. */
    s = v90a_sd_init(0);
    for (i = 0; i < 6*200; i++)
        (void) v90a_sd_put(s, 0.0);
    check(!v90a_sd_acquired(s), "silence is rejected");
    v90a_sd_free(s);

    /* Noise alone. */
    s = v90a_sd_init(0);
    for (i = 0; i < 6*200; i++)
        (void) v90a_sd_put(s, urand());
    check(!v90a_sd_acquired(s), "noise is rejected");
    v90a_sd_free(s);

    /* A six-periodic signal with the RIGHT zeros and the WRONG signs: this is
     * what says the sign term is doing work rather than the depth carrying the
     * whole score. */
    s = v90a_sd_init(0);
    for (i = 0; i < 6*200; i++) {
        static const double pat[6] = { +1.0, 0.0, -1.0, +1.0, 0.0, -1.0 };
        (void) v90a_sd_put(s, pat[i%6]);
    }
    check(!v90a_sd_acquired(s), "six-periodic with wrong signs is rejected");
    v90a_sd_free(s);
}

static void test_noisy(double snr_db)
{
    v90a_sd_t *s = v90a_sd_init(0);
    double n = pow(10.0, -snr_db/20.0)/0.29;   /* urand() has sd ~0.29 */
    bool got = false;
    char msg[96];
    int i;

    for (i = 0; i < 6*200  &&  !got; i++)
        got = v90a_sd_put(s, sd_symbol(i, 1.0) + n*urand());
    snprintf(msg, sizeof msg, "Sd at %.0f dB SNR acquires (score %.3f)",
             snr_db, v90a_sd_score(s));
    check(got, msg);
    v90a_sd_free(s);
}

/*
 * The end-to-end case.  Sd at 8 kHz -> zero-order hold to 16 kHz -> a channel
 * with real ISI -> an arbitrary sampling phase -> the T/2 equaliser -> the
 * detector.  Nothing is calibrated anywhere in that chain.
 */
static void test_through_fse(double phase, double gain, double snr_db)
{
    /* A short, deliberately dispersive channel at 16 kHz: a main tap with
     * neighbours of the size measured on this line (±0.14 against 0.80). */
    static const double ch[5] = { 0.05, 0.18, 0.80, 0.18, 0.05 };
    v90a_fse_t *fse = v90a_fse_init(0, 0.0);
    v90a_sd_t *sd = v90a_sd_init(0);
    int16_t buf[2048];
    double h[V90A_SD_MAX_TAPS], score = 0.0, level = 0.0;
    int taps = v90a_fse_tap_count(fse), parity = 0, nbuf = 0;
    bool fitted = false;
    bool got = false;
    char msg[160];
    int i;

    for (i = 0; i < 8000  &&  !got; i++) {
        double x = 0.0;
        int16_t amp[2];
        double sym[2];
        int n, k;

        /* Two 16 kHz samples per DS0 interval, held. */
        for (k = 0; k < 2; k++) {
            int j;
            double acc = 0.0;

            for (j = 0; j < 5; j++) {
                /* The held sample 'j' taps back, with the fractional phase
                 * mixing two neighbouring symbols exactly as a sampling
                 * instant off the centre does. */
                int t = 2*i + k - j;
                int si = (int) floor((t - 2.0*phase)/2.0);
                double frac = (t - 2.0*phase)/2.0 - si;
                double a = (si >= 0) ? sd_symbol(si, gain) : 0.0;
                double b = (si + 1 >= 0) ? sd_symbol(si + 1, gain) : 0.0;

                acc += ch[j]*((1.0 - frac)*a + frac*b);
            }
            x = acc;
            if (snr_db > 0.0)
                x += gain*pow(10.0, -snr_db/20.0)*urand()/0.29;
            amp[k] = (int16_t) ((x > 32000.0) ? 32000.0
                                              : (x < -32000.0) ? -32000.0 : x);
        }
        if (!fitted) {
            /* Collect a window, fit the taps to §8.4.4's reference, install
             * them.  Nothing is calibrated before this point. */
            if (nbuf + 2 <= (int) (sizeof buf/sizeof buf[0])) {
                buf[nbuf++] = amp[0];
                buf[nbuf++] = amp[1];
            }
            if (nbuf == (int) (sizeof buf/sizeof buf[0])) {
                fitted = v90a_sd_fit(buf, nbuf, taps, &parity, h,
                                     &score, &level);
                if (!fitted)
                    nbuf = 0;
                else {
                    (void) v90a_fse_set_taps(fse, h, taps, parity);
                    /* CMA would walk straight off the fit -- Sd is not
                     * constant modulus, which is why the fit exists. */
                    v90a_fse_set_mode(fse, V90A_FSE_FROZEN);
                }
            }
            continue;
        }
        n = v90a_fse_put(fse, amp, 2, sym, 2);
        for (k = 0; k < n  &&  !got; k++)
            got = v90a_sd_put(sd, sym[k]);
    }
    snprintf(msg, sizeof msg,
             "fit + equaliser at phase %.2f, gain %.0f, SNR %.0f dB: fit %.3f "
             "parity %d, structure %.3f level %.3f",
             phase, gain, snr_db, score, parity,
             v90a_sd_score(sd), v90a_sd_level(sd));
    check(fitted && got, msg);
    v90a_sd_free(sd);
    v90a_fse_free(fse);
}

/* The fit must not accept what is not Sd, or the live hunt would install taps
 * fitted to noise and call it acquisition.  Held-out scoring is what makes this
 * possible; in sample, 32 free taps explain all three of these. */
static void test_fit_rejects(void)
{
    int16_t buf[2048];
    double h[V90A_SD_MAX_TAPS], score = 0.0;
    int parity = 0, i;
    char msg[96];

    for (i = 0; i < (int) (sizeof buf/sizeof buf[0]); i++)
        buf[i] = (int16_t) (3000.0*((urand() > 0.0) ? 1.0 : -1.0));
    snprintf(msg, sizeof msg, "fit rejects TRN1d-like input (score %.3f)", score);
    check(!v90a_sd_fit(buf, (int) (sizeof buf/sizeof buf[0]), 32, &parity, h,
                       &score, NULL), msg);
    printf("      TRN1d-like fit score %.3f\n", score);

    for (i = 0; i < (int) (sizeof buf/sizeof buf[0]); i++)
        buf[i] = (int16_t) (6000.0*urand());
    check(!v90a_sd_fit(buf, (int) (sizeof buf/sizeof buf[0]), 32, &parity, h,
                       &score, NULL), "fit rejects noise");
    printf("      noise fit score %.3f\n", score);

    memset(buf, 0, sizeof buf);
    check(!v90a_sd_fit(buf, (int) (sizeof buf/sizeof buf[0]), 32, &parity, h,
                       &score, NULL), "fit rejects silence");
}

/*
 * The case a live call found and an endless-Sd test cannot: §8.4.4's Sd is only
 * 64 six-symbol repetitions -- 384 DS0 intervals, 48 ms -- and S-bar-d follows
 * it immediately.  A fit window longer than that can never hold Sd alone, and
 * the fit then correctly refuses a mixture.  Live, at a 128 ms window, eight
 * consecutive windows scored below 0.014 on a call where the digital modem
 * transmitted Sd exactly as it should.
 *
 * So this drives the sliding window the engine drives, over a stream that is
 * TRN1d, then exactly 384 symbols of Sd, then TRN1d again.
 */
#define FIT_WIN   512
#define FIT_SLIDE (FIT_WIN/4)

static void test_finite_sd(double phase)
{
    static const double ch[5] = { 0.05, 0.18, 0.80, 0.18, 0.05 };
    const int lead = 900, sd_len = 384;
    int16_t win[FIT_WIN];
    double h[V90A_SD_MAX_TAPS], score = 0.0, best = -1e30, level = 0.0;
    int fill = 0, parity = 0, hits = 0;
    char msg[128];
    int i;

    for (i = 0; i < 4000; i++) {
        double acc = 0.0;
        int k;

        for (k = 0; k < 2; k++) {
            int j;

            acc = 0.0;
            for (j = 0; j < 5; j++) {
                int t = 2*i + k - j;
                double u = (t - 2.0*phase)/2.0;
                int si = (int) floor(u);
                double frac = u - si;
                double a, b;
                int m;

                /* TRN1d outside the Sd interval: one level, random signs. */
                m = si;
                a = (m < 0) ? 0.0
                  : (m >= lead && m < lead + sd_len) ? sd_symbol(m - lead, 3000.0)
                  : ((((unsigned) m*2654435761u) & 0x10000u) ? 1400.0 : -1400.0);
                m = si + 1;
                b = (m < 0) ? 0.0
                  : (m >= lead && m < lead + sd_len) ? sd_symbol(m - lead, 3000.0)
                  : ((((unsigned) m*2654435761u) & 0x10000u) ? 1400.0 : -1400.0);
                acc += ch[j]*((1.0 - frac)*a + frac*b);
            }
            win[fill++] = (int16_t) ((acc > 32000.0) ? 32000.0
                                     : (acc < -32000.0) ? -32000.0 : acc);
            if (fill == FIT_WIN) {
                if (v90a_sd_fit(win, FIT_WIN, 32, &parity, h, &score, &level))
                    hits++;
                if (score > best)
                    best = score;
                memmove(win, win + FIT_SLIDE,
                        (size_t) (FIT_WIN - FIT_SLIDE)*sizeof(win[0]));
                fill = FIT_WIN - FIT_SLIDE;
            }
        }
    }
    snprintf(msg, sizeof msg,
             "384-symbol Sd inside TRN1d at phase %.2f: %d window(s) hit, "
             "best score %.3f", phase, hits, best);
    check(hits > 0, msg);
}

int main(void)
{
    int slot;

    for (slot = 0; slot < 6; slot++)
        test_clean(1.0, slot);
    test_clean(4000.0, 0);
    test_clean(0.01, 0);
    test_rejects();
    test_fit_rejects();
    test_noisy(20.0);
    test_noisy(12.0);
    test_through_fse(0.00, 3000.0, 0.0);
    test_through_fse(0.25, 3000.0, 0.0);
    test_through_fse(0.50, 3000.0, 0.0);
    test_through_fse(0.75, 3000.0, 0.0);
    test_through_fse(0.50, 300.0, 0.0);
    test_through_fse(0.25, 3000.0, 25.0);
    test_through_fse(0.50, 3000.0, 18.0);
    test_finite_sd(0.00);
    test_finite_sd(0.25);
    test_finite_sd(0.50);
    test_finite_sd(0.75);

    if (failures) {
        printf("v90_analogue_sd_test: %d FAILURES\n", failures);
        return 1;
    }
    printf("v90_analogue_sd_test: OK\n");
    return 0;
}
