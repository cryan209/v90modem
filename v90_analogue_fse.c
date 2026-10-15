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
/*
 * How far into its own decision region a sample may fall and still be adapted
 * on.
 *
 * A decision-directed loop is a ratchet: adapt on wrong decisions and the taps
 * move to make those decisions look right, which produces more wrong ones.
 * This project has the scar -- in the V.34 data mode one disturbance became a
 * permanent collapse until the loop was gated on the receiver still being
 * healthy (docs/v34_data_mode_rates.md).
 *
 * The scale for that gate is the decision region, which the caller supplies,
 * and NOT the level: the G.711 ladder's steps span a factor of a hundred, so a
 * gate proportional to the level accepts errors on the loud points most readily
 * and pulls the gain up after them.  That was measured, and it took a run whose
 * frozen taps recovered 14% of codewords down to 0.7%.
 */
#define DD_TRUST        0.5
/* Output magnitude no G.711 level reaches, in TRN1d units (see put()). */
#define FB_RUNAWAY      16.0

struct v90a_fse_s {
    int      taps;
    double   mu;
    double  *h;
    double  *x;            /* T/2 delay line, x[0] most recent */
    double   dc;
    double   agc;
    int      agc_primed;   /* samples of fast-attack priming still to do */
    bool     agc_frozen;
    int      half;         /* 0/1: which half-symbol the next input is */
    int      filled;

    v90a_fse_mode_t mode;
    double   last_y;
    bool     have_y;

    /* Decision feedback (v90a_fse_set_feedback()): b[] on the last nb
     * decisions, d[0] the most recent. */
    int      nb;
    double  *b;
    double  *d;

    /* v90a_fse_train_ls(): normal equations over [x ; -d]. */
    double  *ls_A, *ls_y;
    int      ls_n, ls_target, ls_solved, ls_passes;
    v90a_fse_mode_t ls_then;
    int      fb_resets;

    int      dd_used;
    int      dd_rejected;
    int      dd_total;

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
        free(s->b);
        free(s->d);
        free(s->ls_A);
        free(s->ls_y);
        free(s);
    }
}

int v90a_fse_set_feedback(v90a_fse_t *s, int nb)
{
    if (s == NULL  ||  nb < 0  ||  nb > 256)
        return 0;
    free(s->b);
    free(s->d);
    s->b = s->d = NULL;
    s->nb = 0;
    if (nb == 0)
        return 1;
    s->b = calloc((size_t) nb, sizeof(double));
    s->d = calloc((size_t) nb, sizeof(double));
    if (s->b == NULL  ||  s->d == NULL) {
        free(s->b);
        free(s->d);
        s->b = s->d = NULL;
        return 0;
    }
    s->nb = nb;
    return 1;
}

int v90a_fse_feedback(const v90a_fse_t *s)
{
    return s ? s->nb : 0;
}

int v90a_fse_train_ls(v90a_fse_t *s, int symbols, int passes, v90a_fse_mode_t then)
{
    int n;

    if (s == NULL  ||  symbols <= 0  ||  passes <= 0
        ||  (then != V90A_FSE_DD  &&  then != V90A_FSE_FROZEN))
        return 0;
    s->ls_passes = passes;
    s->ls_then = then;
    n = s->taps + s->nb;
    free(s->ls_A);
    free(s->ls_y);
    s->ls_A = calloc((size_t) n*n, sizeof(double));
    s->ls_y = calloc((size_t) n, sizeof(double));
    if (s->ls_A == NULL  ||  s->ls_y == NULL)
        return 0;
    s->ls_n = 0;
    s->ls_target = symbols;
    v90a_fse_set_mode(s, V90A_FSE_LS);
    return 1;
}

int v90a_fse_ls_solved(const v90a_fse_t *s)
{
    return s ? s->ls_solved : 0;
}

int v90a_fse_fb_resets(const v90a_fse_t *s)
{
    return s ? s->fb_resets : 0;
}

static void ls_accumulate(v90a_fse_t *s, double target)
{
    int n = s->taps + s->nb;
    double u[512 + 256];

    for (int j = 0; j < s->taps; j++)
        u[j] = s->x[j];
    for (int j = 0; j < s->nb; j++)
        u[s->taps + j] = -s->d[j];
    for (int i = 0; i < n; i++) {
        double *row = s->ls_A + (size_t) i*n;

        for (int j = 0; j <= i; j++)
            row[j] += u[i]*u[j];
        s->ls_y[i] += u[i]*target;
    }
    s->ls_n++;
}

/* Cholesky on the accumulated (lower-triangle) system, lightly loaded. */
static int ls_solve(v90a_fse_t *s)
{
    int n = s->taps + s->nb;
    double *A = s->ls_A, *w;

    w = calloc((size_t) n, sizeof(double));
    if (w == NULL)
        return 0;
    /* Load each diagonal element relative to ITSELF.  The forward regressors
     * are input samples -- raw counts, thousands, once a fit has frozen the
     * AGC at 1 -- and the feedback ones are decisions near unity, so loading
     * relative to the mean diagonal was ~7x the feedback block's own energy
     * on the Apple modem and pinned every feedback tap to zero: the trained
     * equaliser read 12.6 dB, exactly a forward-only one. */
    for (int i = 0; i < n; i++)
        A[(size_t) i*n + i] *= 1.0 + 1e-6;
    for (int i = 0; i < n; i++)
        A[(size_t) i*n + i] += 1e-12;
    for (int i = 0; i < n; i++) {
        for (int j = 0; j <= i; j++) {
            double v = A[(size_t) i*n + j];

            for (int k = 0; k < j; k++)
                v -= A[(size_t) i*n + k]*A[(size_t) j*n + k];
            if (i == j) {
                if (v <= 0.0) {
                    free(w);
                    return 0;
                }
                A[(size_t) i*n + i] = sqrt(v);
            } else {
                A[(size_t) i*n + j] = v/A[(size_t) j*n + j];
            }
        }
    }
    for (int i = 0; i < n; i++) {
        double v = s->ls_y[i];

        for (int k = 0; k < i; k++)
            v -= A[(size_t) i*n + k]*w[k];
        w[i] = v/A[(size_t) i*n + i];
    }
    for (int i = n - 1; i >= 0; i--) {
        double v = w[i];

        for (int k = i + 1; k < n; k++)
            v -= A[(size_t) k*n + i]*w[k];
        w[i] = v/A[(size_t) i*n + i];
    }
    for (int j = 0; j < s->taps; j++)
        s->h[j] = w[j];
    for (int j = 0; j < s->nb; j++)
        s->b[j] = w[s->taps + j];
    free(w);
    return 1;
}

/* The decision for the symbol just emitted joins the feedback history. */
static void push_decision(v90a_fse_t *s, double decision)
{
    if (s->nb == 0)
        return;
    memmove(s->d + 1, s->d, sizeof(double)*(size_t) (s->nb - 1));
    s->d[0] = decision;
}

int v90a_fse_tap_count(const v90a_fse_t *s)
{
    return (s != NULL) ? s->taps : 0;
}

int v90a_fse_set_taps(v90a_fse_t *s, const double *h, int taps, int parity)
{
    int i;

    if (s == NULL  ||  h == NULL  ||  taps != s->taps)
        return 0;
    /* put() toggles `half` and emits when it lands on 0, so the sample that
     * produces a symbol is the one AFTER the toggle takes half to 1.  Setting
     * half = parity therefore makes the next input sample the one the fit
     * treated as offset `parity` -- and getting this backwards is not a
     * subtle failure: the output is then the other eye, six-periodic with the
     * zero slots still in place but the sign grouping shuffled, which reads
     * like a detector bug rather than an alignment one. */
    s->half = (parity & 1);
    for (i = 0; i < taps; i++)
        s->h[i] = h[i];
    /* The fit was made on the raw samples, so its taps already contain the
     * line's gain.  Leaving the AGC free to keep adapting would multiply that
     * in a second time and walk the output off the unit modulus the fit put it
     * at, which is the scale everything downstream is calibrated against. */
    s->agc = 1.0;
    s->agc_primed = 0;
    s->agc_frozen = true;
    return taps;
}

void v90a_fse_set_mode(v90a_fse_t *s, v90a_fse_mode_t mode)
{
    if (s == NULL)
        return;
    /*
     * Leaving CMA freezes the input AGC, and it has to.
     *
     * The AGC is there to bring the input into the range the taps are
     * initialised for; while the target is a constant modulus that is all it is.
     * On a level ladder it is actively destructive -- it divides the input by
     * its own running mean, so a constellation point 1.8x the training level
     * arrives at the slicer as 0.5x and the absolute scale, which IS the
     * information in §8.4.1's DIL and §8.6's Phase 4, is gone.  Measured before
     * this: the tail of a real downstream sits 1.785x above TRN1d and came out
     * at 0.89x, a mean Ucode of 38 against the 54 transmitted -- almost exactly
     * one G.711 chord, which is the self-similarity trap this project has been
     * caught by before.
     */
    if (mode != V90A_FSE_CMA)
        s->agc_frozen = true;
    s->mode = mode;
}

void v90a_fse_set_mu(v90a_fse_t *s, double mu)
{
    if (s != NULL  &&  mu > 0.0)
        s->mu = mu;
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
    for (int j = 0; j < s->nb; j++)
        energy += s->d[j]*s->d[j];
    for (int j = 0; j < s->taps; j++)
        s->h[j] += s->mu*err*s->x[j]/energy;
    /* y = h.x - b.d, so raising y by the error lowers b along d. */
    for (int j = 0; j < s->nb; j++)
        s->b[j] -= s->mu*err*s->d[j]/energy;
}

void v90a_fse_decide(v90a_fse_t *s, double decision, double tolerance)
{
    double err;

    if (s == NULL  ||  !s->have_y)
        return;
    s->have_y = false;
    /* Adapt first: the regressor that made last_y is the delay line AND the
     * feedback history as they stood, before this decision joins it. */
    if (s->mode == V90A_FSE_LS) {
        /* Every decision, ungated: on TRN1d they are all but certain, and a
         * least-squares fit is not a ratchet the way a gradient loop is. */
        ls_accumulate(s, decision);
        if (s->ls_n >= s->ls_target) {
            int n = s->taps + s->nb;

            if (ls_solve(s))
                s->ls_solved++;
            if (--s->ls_passes > 0) {
                /* Next pass: decide with the taps just installed. */
                memset(s->ls_A, 0, sizeof(double)*(size_t) n*n);
                memset(s->ls_y, 0, sizeof(double)*(size_t) n);
                s->ls_n = 0;
            } else {
                free(s->ls_A);
                free(s->ls_y);
                s->ls_A = s->ls_y = NULL;
                s->mode = s->ls_then;
            }
        }
    } else if (s->mode == V90A_FSE_DD) {
        err = decision - s->last_y;
        s->dd_total++;
        if (tolerance > 0.0  &&  fabs(err) > DD_TRUST*tolerance) {
            s->dd_rejected++;
        } else {
            s->dd_used++;
            adapt(s, err);
        }
    }
    /* In every mode: feedback needs every decision, adapting or not. */
    push_decision(s, decision);
}

int v90a_fse_dd_used(const v90a_fse_t *s)
{
    return s ? s->dd_used : 0;
}

int v90a_fse_dd_rejected(const v90a_fse_t *s)
{
    return s ? s->dd_rejected : 0;
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
        if (!s->agc_frozen)
            s->dc += ((double) amp[i] - s->dc)/8192.0;
        if (s->agc_primed > 0) {
            double a = fabs((double) amp[i] - s->dc);

            s->agc = (s->agc_primed == AGC_PRIME) ? a
                   : (s->agc + (a - s->agc)*0.05);
            s->agc_primed--;
        } else if (!s->agc_frozen) {
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
        /* A symbol the caller never decided feeds back as itself. */
        if (s->have_y) {
            push_decision(s, s->last_y);
            s->have_y = false;
        }
        for (int j = 0; j < s->taps; j++)
            y += s->h[j]*s->x[j];
        for (int j = 0; j < s->nb; j++)
            y -= s->b[j]*s->d[j];
        /*
         * A feedback loop fed decisions that merely track its output is a
         * linear IIR inverse of the channel, and inverting a 4000 Hz null is
         * unstable: measured, a fine ladder near zero grew a Nyquist
         * oscillation x1.35 a symbol until it pinned at the top of the G.711
         * range and stayed there.  No G.711 level is anywhere near
         * FB_RUNAWAY in these units (the ladder's top is ~8.5x TRN1d's), so
         * an output beyond it is the loop, not the line: clear the history
         * that is driving it and start again from the forward taps.
         */
        if (s->nb > 0  &&  fabs(y) > FB_RUNAWAY) {
            for (int j = 0; j < s->nb; j++)
                y += s->b[j]*s->d[j];
            memset(s->d, 0, sizeof(double)*(size_t) s->nb);
            s->fb_resets++;
        }

        if (s->mode == V90A_FSE_CMA) {
            /* Gradient of (y^2 - R2)^2, sign folded in: the update drives the
             * modulus to R2 and says nothing about the sign, which is the whole
             * reason this works on a scrambler output whose signs are unknown. */
            adapt(s, s->mu > 0.0 ? y*(R2 - y*y) : 0.0);
        }
        /*endif*/
        /* Every mode keeps the symbol for v90a_fse_decide(): DD adapts on it,
         * and feedback needs its decision whatever the mode. */
        if (s->mode != V90A_FSE_CMA  ||  s->nb > 0) {
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
