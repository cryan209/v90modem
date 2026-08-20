/*
 * v34_gardner_test.c - the V.90 upstream timing loop, against a signal
 *                      whose timing is known exactly.
 *
 * The receiver this serves runs at three samples per symbol and, until the
 * loop existed, advanced its symbol instant by exactly three samples for the
 * life of a call.  Live that decodes correctly for about fifteen seconds and
 * then walks off.  The previous attempt at a detector was judged only by
 * live calls, and shipped a default that asked for thirty corrections a
 * second -- so this one is judged here first, where the true sampling
 * instant is a number we chose.
 *
 * The signal is a raised-cosine pulse train evaluated analytically, so a
 * fractional delay or a clock offset in ppm costs nothing to impose and
 * needs no resampler of its own (which would bring its own timing error to
 * the party).
 *
 * Checks, in order of what they would catch:
 *   1. S-curve sign and shape: late sampling gives a positive error, early
 *      gives negative, and the zero is at the ideal instant.  A sign error
 *      here is the difference between a loop that locks and one that runs
 *      away, and it is invisible in a "does the call work" test.
 *   2. Acquisition: a static half-sample offset is pulled in.
 *   3. Tracking: a 50 ppm clock offset is followed, with the sampling error
 *      staying bounded and the slip count close to what the offset implies.
 *   4. Quiescence: with perfect timing the loop asks for almost nothing --
 *      the failure mode of the detector this replaced.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "spandsp-master/src/v34_gardner.h"

#define SAMPLES_PER_SYMBOL  3
#define ROLLOFF             0.25
#define PULSE_SPAN          12          /* symbols either side */

typedef struct {
    int8_t *sym_re;
    int8_t *sym_im;
    int nsym;
} pulse_train_t;

/* Raised cosine in the time domain, t in symbols. */
static double raised_cosine(double t)
{
    double denom;

    if (fabs(t) < 1e-9)
        return 1.0;
    denom = 1.0 - (2.0*ROLLOFF*t)*(2.0*ROLLOFF*t);
    if (fabs(denom) < 1e-9)
    {
        /* Removable singularity at t = +/- 1/(2*beta). */
        return (M_PI/4.0)*(sin(M_PI*t)/(M_PI*t));
    }
    /*endif*/
    return (sin(M_PI*t)/(M_PI*t))*cos(M_PI*ROLLOFF*t)/denom;
}
/*- End of function --------------------------------------------------------*/

/* The transmitted waveform at an arbitrary time, t in symbols. */
static void signal_at(const pulse_train_t *p, double t,
                      float *re, float *im)
{
    int centre = (int) floor(t);
    double acc_re = 0.0;
    double acc_im = 0.0;

    for (int k = centre - PULSE_SPAN;  k <= centre + PULSE_SPAN;  k++)
    {
        double h;

        if (k < 0 || k >= p->nsym)
            continue;
        /*endif*/
        h = raised_cosine(t - k);
        acc_re += p->sym_re[k]*h;
        acc_im += p->sym_im[k]*h;
    }
    /*endfor*/
    *re = (float) acc_re;
    *im = (float) acc_im;
}
/*- End of function --------------------------------------------------------*/

static uint32_t rng_state = 0x12345678u;

static int rng_bit(void)
{
    rng_state = rng_state*1103515245u + 12345u;
    return (int) ((rng_state >> 16) & 1);
}
/*- End of function --------------------------------------------------------*/

static void make_train(pulse_train_t *p, int nsym)
{
    p->nsym = nsym;
    p->sym_re = malloc(nsym);
    p->sym_im = malloc(nsym);
    for (int k = 0;  k < nsym;  k++)
    {
        p->sym_re[k] = rng_bit() ? 1 : -1;
        p->sym_im[k] = rng_bit() ? 1 : -1;
    }
    /*endfor*/
}
/*- End of function --------------------------------------------------------*/

static void free_train(pulse_train_t *p)
{
    free(p->sym_re);
    free(p->sym_im);
}
/*- End of function --------------------------------------------------------*/

/* Mean Gardner error with the sampling instant held at a fixed offset, in
   symbols, from the ideal.  This is the detector's S-curve. */
static double s_curve(const pulse_train_t *p, double offset)
{
    double total = 0.0;
    int count = 0;

    for (int k = PULSE_SPAN + 1;  k < p->nsym - PULSE_SPAN;  k++)
    {
        float now_re, now_im, prev_re, prev_im, mid_re, mid_im;
        float e;
        float power;

        signal_at(p, k + offset, &now_re, &now_im);
        signal_at(p, k - 1 + offset, &prev_re, &prev_im);
        signal_at(p, k - 0.5 + offset, &mid_re, &mid_im);
        e = v34_gardner_error(now_re, now_im, prev_re, prev_im,
                              mid_re, mid_im);
        power = now_re*now_re + now_im*now_im
              + prev_re*prev_re + prev_im*prev_im + 1e-6f;
        total += e/power;
        count++;
    }
    /*endfor*/
    return count ? total/count : 0.0;
}
/*- End of function --------------------------------------------------------*/

static int test_s_curve(void)
{
    pulse_train_t p;
    double late;
    double early;
    double centred;
    int failed = 0;

    make_train(&p, 4000);
    late = s_curve(&p, +0.2);
    early = s_curve(&p, -0.2);
    centred = s_curve(&p, 0.0);
    printf("  S-curve: early(-0.2)=%+.4f centre=%+.4f late(+0.2)=%+.4f\n",
           early, centred, late);
    if (!(late > 0.02))
    {
        printf("  FAIL: late sampling must give a positive error\n");
        failed = 1;
    }
    /*endif*/
    if (!(early < -0.02))
    {
        printf("  FAIL: early sampling must give a negative error\n");
        failed = 1;
    }
    /*endif*/
    if (fabs(centred) > 0.01)
    {
        printf("  FAIL: the zero of the S-curve must be the ideal instant\n");
        failed = 1;
    }
    /*endif*/
    free_train(&p);
    return failed;
}
/*- End of function --------------------------------------------------------*/

/* Run the closed loop over a train whose sample clock is offset by ppm.
   The actuator is modelled the way the receiver applies it: whole samples
   move the instant, and the loop's leftover fraction is applied as an
   interpolation weight, so what is measured here is the same thing the
   receiver gets.  Returns the RMS sampling error in symbols over the second
   half; also reports net and total whole-sample corrections. */
static double run_loop(double ppm, double initial_offset, int nsym,
                       int *slips_out, int *total_out)
{
    pulse_train_t p;
    v34_gardner_state_t g;
    /* The receiver's own idea of where it is, in samples of its own clock. */
    int64_t instant;
    double sq = 0.0;
    int count = 0;
    int total = 0;

    make_train(&p, nsym);
    v34_gardner_init(&g, V34_GARDNER_DEFAULT_MU, V34_GARDNER_DEFAULT_BETA);
    instant = (int64_t) ((PULSE_SPAN + 2)*SAMPLES_PER_SYMBOL);
    for (int k = 0;  k < nsym - 2*PULSE_SPAN - 8;  k++)
    {
        float now_re, now_im, mid_re, mid_im;
        double sample_now;
        double t_now;
        double t_mid;
        double ideal;
        int correction;

        /* Where the receiver samples, in its own sample units: the integer
           instant plus the fraction the loop is holding. */
        sample_now = (double) instant + g.acc;
        t_now = (sample_now/(double) SAMPLES_PER_SYMBOL)*(1.0 + ppm*1e-6)
              + initial_offset;
        t_mid = ((sample_now - 1.5)/(double) SAMPLES_PER_SYMBOL)
                    *(1.0 + ppm*1e-6)
              + initial_offset;
        if (t_now >= p.nsym - PULSE_SPAN - 1)
            break;
        /*endif*/
        signal_at(&p, t_now, &now_re, &now_im);
        signal_at(&p, t_mid, &mid_re, &mid_im);
        /* How far this instant sits from the nearest symbol centre.  Taken
           before the update, so it measures where we actually sampled. */
        ideal = t_now - floor(t_now + 0.5);
        correction = v34_gardner_update(&g, now_re, now_im, mid_re, mid_im, 1);
        if (correction)
            total++;
        /*endif*/
        if (k > (nsym - 2*PULSE_SPAN - 8)/2)
        {
            sq += ideal*ideal;
            count++;
        }
        /*endif*/
        instant += SAMPLES_PER_SYMBOL + correction;
    }
    /*endfor*/
    *slips_out = g.slips;
    *total_out = total;
    free_train(&p);
    return count ? sqrt(sq/count) : 1.0;
}
/*- End of function --------------------------------------------------------*/

static int test_acquire(void)
{
    int slips;
    int total;
    double rms = run_loop(0.0, 0.30, 40000, &slips, &total);

    printf("  acquire: static 0.30 symbol offset -> rms %.3f symbols, "
           "%d corrections (net %d)\n", rms, total, slips);
    /* With a continuous actuator the residual is not quantised, so a static
       offset should come down to a small fraction of a symbol. */
    if (rms > 0.05)
    {
        printf("  FAIL: the loop did not pull in a static offset\n");
        return 1;
    }
    /*endif*/
    return 0;
}
/*- End of function --------------------------------------------------------*/

static int test_track(void)
{
    int slips;
    int total;
    double rms;
    int expected;

    /* 50 ppm at three samples per symbol over 20000 symbols is 20000*3*50e-6
       = 3 samples of drift.  A positive ppm means our sample clock walks
       through the peer's symbols slightly fast, so the instant arrives ever
       later and the loop must take AWAY samples: the net correction is
       negative.  Getting that sign wrong is the difference between a loop
       that tracks and one that runs away twice as fast as the drift. */
    rms = run_loop(50.0, 0.0, 40000, &slips, &total);
    expected = -6;
    printf("  track: 50 ppm -> rms %.3f symbols, net %d corrections "
           "(expected about %d)\n", rms, slips, expected);
    if (rms > 0.05)
    {
        printf("  FAIL: sampling error is not bounded under a clock offset\n");
        return 1;
    }
    /*endif*/
    if (abs(slips - expected) > 2)
    {
        printf("  FAIL: correction count does not match the offset\n");
        return 1;
    }
    /*endif*/
    return 0;
}
/*- End of function --------------------------------------------------------*/

static int test_quiescent(void)
{
    int slips;
    int total;
    double rms = run_loop(0.0, 0.0, 20000, &slips, &total);

    printf("  quiescent: perfect timing -> rms %.3f symbols, "
           "%d corrections\n", rms, total);
    /* The detector this replaced asked for about thirty corrections a
       second on a clock that was nearly right. */
    if (total > 2)
    {
        printf("  FAIL: the loop is chasing noise on a perfect clock\n");
        return 1;
    }
    /*endif*/
    if (rms > 0.02)
    {
        printf("  FAIL: the loop wandered off a perfect clock\n");
        return 1;
    }
    /*endif*/
    return 0;
}
/*- End of function --------------------------------------------------------*/

int main(void)
{
    int failed = 0;

    printf("v34_gardner_test: V.90 upstream timing loop\n");
    failed |= test_s_curve();
    failed |= test_acquire();
    failed |= test_track();
    failed |= test_quiescent();
    if (failed)
    {
        printf("v34_gardner_test: FAILED\n");
        return 1;
    }
    /*endif*/
    printf("v34_gardner_test: OK\n");
    return 0;
}
/*- End of function --------------------------------------------------------*/
