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

/* levels is the number of odd-integer amplitudes per axis either side of
   zero: 1 gives the +/-1 four-point constellation the loop was designed
   against, 8 gives +/-1..+/-15, which is the density V.34 is carrying by
   28800 bit/s.  That difference is the whole point -- Gardner's error is
   formed from (y[k] - y[k-1]), so its data self-noise grows with the
   constellation, and the loop was only ever tested at four points. */
/* One odd-integer amplitude, uniformly over the constellation's levels. */
static int rng_level(int levels)
{
    int mag;

    if (levels <= 1)
        return rng_bit() ? 1 : -1;
    /*endif*/
    rng_state = rng_state*1103515245u + 12345u;
    mag = 2*(int) ((rng_state >> 16) % (unsigned) levels) + 1;
    return rng_bit() ? mag : -mag;
}
/*- End of function --------------------------------------------------------*/

static void make_train(pulse_train_t *p, int nsym, int levels)
{
    p->nsym = nsym;
    p->sym_re = malloc(nsym);
    p->sym_im = malloc(nsym);
    for (int k = 0;  k < nsym;  k++)
    {
        p->sym_re[k] = (int8_t) (rng_level(levels));
        p->sym_im[k] = (int8_t) (rng_level(levels));
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

/* V.34 puts every constellation point on odd integers, which is what makes
   the decision available to the loop without a shell decoder. */
static float slice(float y)
{
    return 2.0f*floorf(y/2.0f) + 1.0f;
}
/*- End of function --------------------------------------------------------*/

/* Mean error and its spread with the sampling instant held at a fixed
   offset, in symbols, from the ideal.  The mean over offset is the
   detector's S-curve; the spread AT ZERO offset is its data self-noise,
   which is what the loop filter has to be slow enough to reject. */
static double s_curve_det(const pulse_train_t *p, double offset, int det,
                          double *rms_out)
{
    double total = 0.0;
    double sq = 0.0;
    int count = 0;

    for (int k = PULSE_SPAN + 1;  k < p->nsym - PULSE_SPAN;  k++)
    {
        float now_re, now_im, prev_re, prev_im, mid_re, mid_im;
        float e;
        float power;

        signal_at(p, k + offset, &now_re, &now_im);
        signal_at(p, k - 1 + offset, &prev_re, &prev_im);
        signal_at(p, k - 0.5 + offset, &mid_re, &mid_im);
        switch (det)
        {
        case V34_GARDNER_DET_DD:
            e = v34_gardner_error_dd(slice(now_re), slice(now_im),
                                     slice(prev_re), slice(prev_im),
                                     mid_re, mid_im);
            break;
        case V34_GARDNER_DET_MM:
            e = v34_gardner_error_mm(now_re, now_im, prev_re, prev_im,
                                     slice(now_re), slice(now_im),
                                     slice(prev_re), slice(prev_im));
            break;
        default:
            e = v34_gardner_error(now_re, now_im, prev_re, prev_im,
                                  mid_re, mid_im);
            break;
        }
        /*endswitch*/
        power = now_re*now_re + now_im*now_im
              + prev_re*prev_re + prev_im*prev_im + 1e-6f;
        total += e/power;
        sq += ((double) e/power)*((double) e/power);
        count++;
    }
    /*endfor*/
    if (rms_out)
        *rms_out = count ? sqrt(sq/count) : 0.0;
    /*endif*/
    return count ? total/count : 0.0;
}
/*- End of function --------------------------------------------------------*/

static double s_curve(const pulse_train_t *p, double offset)
{
    return s_curve_det(p, offset, V34_GARDNER_DET_GARDNER, NULL);
}
/*- End of function --------------------------------------------------------*/

static int test_s_curve(void)
{
    pulse_train_t p;
    double late;
    double early;
    double centred;
    int failed = 0;

    make_train(&p, 4000, 1);
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
                       int levels, int det, int *slips_out, int *total_out)
{
    pulse_train_t p;
    v34_gardner_state_t g;
    /* The receiver's own idea of where it is, in samples of its own clock. */
    int64_t instant;
    double sq = 0.0;
    int count = 0;
    int total = 0;

    make_train(&p, nsym, levels);
    v34_gardner_init(&g, V34_GARDNER_DEFAULT_MU, V34_GARDNER_DEFAULT_BETA);
    g.detector = det;
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
        correction = v34_gardner_update(&g, now_re, now_im, mid_re, mid_im,
                                        slice(now_re), slice(now_im),
                                        0.0f, 1);
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
    double rms = run_loop(0.0, 0.30, 40000, 1, V34_GARDNER_DET_GARDNER, &slips, &total);

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
    rms = run_loop(50.0, 0.0, 40000, 1, V34_GARDNER_DET_GARDNER, &slips, &total);
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
    double rms = run_loop(0.0, 0.0, 20000, 1, V34_GARDNER_DET_GARDNER, &slips, &total);

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

/* The test the loop never had.  Every case above runs the four-point
   constellation of V.34's training sequences, and the loop is fine on that;
   the V.90 upstream is carrying a shaped constellation of hundreds of points
   by 28800 bit/s, and there the detector is a different instrument.

   Gardner is non-data-aided: its difference term is (y[k] - y[k-1]), so what
   it reports is dominated by how far apart two RANDOM constellation points
   happened to fall, and that grows with the constellation while the true
   timing error does not.  What matters to a loop in lock is the ratio --
   self-noise per unit of S-curve slope, since that is what the loop filter
   has to reject -- and it is measured at a SMALL offset, because that is the
   regime a tracking loop lives in.  Both decision-aided detectors remove the
   noisy difference: V.34 puts every constellation point on odd integers, so
   the decision is free once the eye is open. */
static void detector_quality(int levels, int det, double offset,
                             double *ratio_out, double *slope_out)
{
    pulse_train_t p;
    double noise;
    double late;
    double early;
    double slope;

    make_train(&p, 8000, levels);
    s_curve_det(&p, 0.0, det, &noise);
    late = s_curve_det(&p, +offset, det, NULL);
    early = s_curve_det(&p, -offset, det, NULL);
    slope = (late - early)/(2.0*offset);
    free_train(&p);
    if (ratio_out)
        *ratio_out = (slope > 1e-9) ? noise/slope : 999.0;
    /*endif*/
    if (slope_out)
        *slope_out = slope;
    /*endif*/
    printf("    %-8s slope=%+.4f/symbol  self-noise=%.4f  ratio=%6.2f\n",
           (det == V34_GARDNER_DET_DD) ? "dd"
             : (det == V34_GARDNER_DET_MM) ? "mm" : "gardner",
           slope, noise, (slope > 1e-9) ? noise/slope : 999.0);
}
/*- End of function --------------------------------------------------------*/

static int test_dense_constellation(void)
{
    double g4;
    double g256;
    double dd256;
    double mm256;
    double mm_slope_lock;
    double mm_slope_far;
    int failed = 0;

    printf("  in lock (+/-0.05 symbol), four points:\n");
    detector_quality(1, V34_GARDNER_DET_GARDNER, 0.05, &g4, NULL);
    printf("  in lock (+/-0.05 symbol), 16 levels per axis -- V.34 carrying "
           "data:\n");
    detector_quality(8, V34_GARDNER_DET_GARDNER, 0.05, &g256, NULL);
    detector_quality(8, V34_GARDNER_DET_DD, 0.05, &dd256, NULL);
    detector_quality(8, V34_GARDNER_DET_MM, 0.05, &mm256, &mm_slope_lock);
    if (!(g256 > 2.0*g4))
    {
        printf("  FAIL: Gardner's noise-to-slope ratio must grow sharply "
               "with the constellation -- that is the defect described\n");
        failed = 1;
    }
    /*endif*/
    if (!(dd256 < g256))
    {
        printf("  FAIL: substituting the decisions into the difference must "
               "quieten the detector on a dense constellation\n");
        failed = 1;
    }
    /*endif*/
    if (!(mm256 < 0.01))
    {
        printf("  FAIL: Mueller and Muller has no data-dependent term at "
               "all; its self-noise here should be nil\n");
        failed = 1;
    }
    /*endif*/
    /* And the reason it is not simply the better detector everywhere.  Its
       error is built from the decisions alone, and on a dense constellation
       a sampling instant far enough out droops the amplitude past the
       slicer's boundaries -- so the decisions go wrong, and the S-curve it
       reports flattens exactly when a loop would need it to pull in.
       Gardner keeps its slope there because it never asks what the symbol
       was.  Whichever detector this loop is built from, it is a TRACKING
       instrument on a dense constellation, and the fractionally spaced
       equalizer is what owns acquisition. */
    printf("  far from lock (+/-0.15 symbol), 16 levels per axis:\n");
    detector_quality(8, V34_GARDNER_DET_MM, 0.15, NULL, &mm_slope_far);
    if (!(mm_slope_far < 0.25*mm_slope_lock))
    {
        printf("  NOTE: Mueller and Muller kept its slope far from lock; "
               "the decision-error limit described here has moved\n");
    }
    /*endif*/
    return failed;
}
/*- End of function --------------------------------------------------------*/

int main(void)
{
    int failed = 0;

    printf("v34_gardner_test: V.90 upstream timing loop\n");
    failed |= test_s_curve();
    failed |= test_dense_constellation();
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
