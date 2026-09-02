/*
 * v90_analogue_fse.c — see v90_analogue_fse.h.
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "v90_analogue_fse.h"

#define DEFAULT_TAPS    32
#define DEFAULT_MU      0.010
/* The dispersion constant.  For a two-level ±A signal E[a^4]/E[a^2] = A^2, so
 * driving the output to unit modulus is what R2 = 1 asks for, and it fixes the
 * output scale to §8.4.5's TRN1d level. */
#define R2              1.0
#define DISP_WIN        512
/* Input AGC.  The taps are initialised around a unit main tap, so the input has
 * to arrive at about unit RMS or the first outputs are nowhere near the modulus
 * CMA is driving them to. */
#define AGC_ALPHA       (1.0/2048.0)
#define AGC_MIN         1.0
/*
 * The AGC has to be right BEFORE the first adaptation, not merely eventually.
 * CMA's error is y(1 - y^2), cubic in the output, so an input arriving at a few
 * thousand counts against taps initialised for unit modulus produces an error
 * of order 1e9 and the tap set is gone on the first symbol -- which is exactly
 * what happened, NaN by the end of the run at every sampling phase.  So the
 * first samples set the level directly rather than being averaged into it.
 */
#define AGC_PRIME       256
/* And a belt: CMA's cubic error is unbounded even after that, at a seam where
 * the line level jumps (§8.4.4's Sd is 6.6 dB above §8.4.5's TRN1d). */
#define ERR_MAX         4.0

struct v90a_fse_s {
    int      taps;
    double   mu;
    double  *h;
    double  *x;            /* T/2 delay line, x[0] most recent */
    double   dc;
    double   agc;
    int      agc_primed;   /* samples of fast-attack priming still to do */
    int      half;         /* 0/1: which half-symbol the next input is */
    int      filled;

    v90a_fse_mode_t mode;
    double   last_y;
    bool     have_y;

    double   disp_acc;
    int      disp_n;
    double   disp;
    int      symbols;
};

v90a_fse_t *v90a_fse_init(int taps, double mu)
{
    v90a_fse_t *s;

    if (taps <= 0)
        taps = DEFAULT_TAPS;
    if (taps & 1)
        taps++;
    if (taps < 4  ||  taps > 512)
        return NULL;
    if ((s = calloc(1, sizeof(*s))) == NULL)
        return NULL;
    s->taps = taps;
    s->mu = (mu > 0.0) ? mu : DEFAULT_MU;
    s->h = calloc((size_t) taps, sizeof(double));
    s->x = calloc((size_t) taps, sizeof(double));
    if (s->h == NULL  ||  s->x == NULL) {
        v90a_fse_free(s);
        return NULL;
    }
    /*
     * A unit main tap in the middle.  Any starting point converges on a channel
     * this benign, but a centred spike leaves the useful part of the response
     * reachable in both directions, which matters because what the equaliser
     * has to absorb here is a sampling phase — an arbitrary shift of up to a
     * whole symbol either way.
     */
    s->h[taps/2] = 1.0;
    /*
     * ...and a deliberate asymmetry on the next T/2 tap.
     *
     * A pure centre spike is symmetric about the half-symbol, so at a sampling
     * phase of exactly 0.5 the two neighbouring symbols contribute equally, the
     * CMA gradient is symmetric, and the tap set sits on a saddle: measured, it
     * stalls at dispersion 0.53 and recovers 75% of TRN1d's signs where every
     * other phase reaches 100%.  Tilting the start breaks the tie and costs the
     * other phases nothing, since CMA washes it out within a few hundred
     * symbols.
     */
    s->h[taps/2 + 1] = 0.1;
    s->agc = AGC_MIN;
    s->agc_primed = AGC_PRIME;
    s->disp = 1.0;
    return s;
}

void v90a_fse_free(v90a_fse_t *s)
{
    if (s) {
        free(s->h);
        free(s->x);
        free(s);
    }
}

void v90a_fse_set_mode(v90a_fse_t *s, v90a_fse_mode_t mode)
{
    if (s)
        s->mode = mode;
}

v90a_fse_mode_t v90a_fse_mode(const v90a_fse_t *s)
{
    return s ? s->mode : V90A_FSE_FROZEN;
}

double v90a_fse_dispersion(const v90a_fse_t *s)
{
    return s ? s->disp : 1.0;
}

double v90a_fse_level(const v90a_fse_t *s)
{
    return s ? s->agc : 0.0;
}

int v90a_fse_symbols(const v90a_fse_t *s)
{
    return s ? s->symbols : 0;
}

double v90a_fse_centre(const v90a_fse_t *s)
{
    double num = 0.0;
    double den = 0.0;

    if (s == NULL)
        return 0.0;
    for (int j = 0; j < s->taps; j++) {
        double e = s->h[j]*s->h[j];

        num += e*j;
        den += e;
    }
    return (den > 0.0) ? (num/den)/2.0 : 0.0;
}

/* NLMS: normalising by the delay line's own energy keeps the step meaningful
 * whatever the line level is doing, which matters at the seams between §8.4's
 * signals — Sd is 6.6 dB above TRN1d. */
static void adapt(v90a_fse_t *s, double err)
{
    double energy = 1e-6;

    if (err > ERR_MAX)
        err = ERR_MAX;
    else if (err < -ERR_MAX)
        err = -ERR_MAX;

    for (int j = 0; j < s->taps; j++)
        energy += s->x[j]*s->x[j];
    for (int j = 0; j < s->taps; j++)
        s->h[j] += s->mu*err*s->x[j]/energy;
}

void v90a_fse_decide(v90a_fse_t *s, double decision)
{
    if (s == NULL  ||  !s->have_y  ||  s->mode != V90A_FSE_DD)
        return;
    adapt(s, decision - s->last_y);
    s->have_y = false;
}

int v90a_fse_put(v90a_fse_t *s, const int16_t *amp, int len,
                 double *out, int max)
{
    int n = 0;

    if (s == NULL  ||  amp == NULL  ||  out == NULL)
        return 0;
    for (int i = 0; i < len  &&  n < max; i++) {
        double y = 0.0;

        /* Track the line level before the filter, not after: CMA is driving
         * the output to a fixed modulus, so an AGC downstream of it would be
         * measuring the algorithm rather than the line. */
        /* The HSF codec's receive stream carries a standing DC offset of about
         * 900 counts.  A linear filter can null it and CMA would eventually
         * ask for that, but spending taps on it costs the span they are here
         * for, so it comes off first. */
        s->dc += ((double) amp[i] - s->dc)/8192.0;
        if (s->agc_primed > 0) {
            double a = fabs((double) amp[i] - s->dc);

            s->agc = (s->agc_primed == AGC_PRIME) ? a
                   : (s->agc + (a - s->agc)*0.05);
            s->agc_primed--;
        } else {
            s->agc += (fabs((double) amp[i] - s->dc) - s->agc)*AGC_ALPHA;
        }
        /*endif*/
        if (s->agc < AGC_MIN)
            s->agc = AGC_MIN;
        memmove(s->x + 1, s->x, sizeof(double)*(size_t) (s->taps - 1));
        s->x[0] = ((double) amp[i] - s->dc)/s->agc;
        if (s->filled < s->taps)
            s->filled++;

        s->half ^= 1;
        if (s->half != 0  ||  s->filled < s->taps  ||  s->agc_primed > 0)
            continue;
        for (int j = 0; j < s->taps; j++)
            y += s->h[j]*s->x[j];

        if (s->mode == V90A_FSE_CMA) {
            /* Gradient of (y^2 - R2)^2, sign folded in: the update drives the
             * modulus to R2 and says nothing about the sign, which is the whole
             * reason this works on a scrambler output whose signs are unknown. */
            adapt(s, s->mu > 0.0 ? y*(R2 - y*y) : 0.0);
        } else if (s->mode == V90A_FSE_DD) {
            s->last_y = y;
            s->have_y = true;
        }
        /*endif*/

        s->disp_acc += (y*y - R2)*(y*y - R2);
        if (++s->disp_n >= DISP_WIN) {
            s->disp = s->disp_acc/s->disp_n;
            s->disp_acc = 0.0;
            s->disp_n = 0;
        }
        s->symbols++;
        out[n++] = y;
    }
    return n;
}
