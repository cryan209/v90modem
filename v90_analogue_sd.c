/*
 * v90_analogue_sd.c — see v90_analogue_sd.h.
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "v90_analogue_sd.h"

/* The reference §8.4.4 supplies, at unit scale. */
static const double SD_REF[6] = { +1.0, 0.0, +1.0, -1.0, 0.0, -1.0 };

/*
 * Ridge on the normal equations.  The input is a held staircase through a
 * low-pass channel, so neighbouring T/2 samples are strongly correlated and R
 * is close to singular; without this the solve returns enormous taps that fit
 * the training half and explain nothing out of sample.  Scaled by the trace so
 * it is independent of the line level.
 */
#define FIT_RIDGE       1e-3
/*
 * The held-out score a fit must reach.  Sd through a dispersive channel at any
 * sampling phase reaches 0.97+; TRN1d, noise and silence cannot be mapped to a
 * periodic reference by a fixed filter at all and score below zero.  0.80
 * sits in the empty middle.
 */
#define FIT_SCORE_MIN   0.80

/* Solve (R + ridge)h = p in place by Cholesky.  Returns false if R is not
 * positive definite, which on this data means the window was silent. */
static bool solve_spd(double *R, double *p, int n)
{
    int i, j, k;

    for (i = 0; i < n; i++) {
        for (j = 0; j <= i; j++) {
            double sum = R[i*n + j];

            for (k = 0; k < j; k++)
                sum -= R[i*n + k]*R[j*n + k];
            if (i == j) {
                if (sum <= 0.0)
                    return false;
                R[i*n + i] = sqrt(sum);
            } else {
                R[i*n + j] = sum/R[j*n + j];
            }
        }
    }
    for (i = 0; i < n; i++) {
        double sum = p[i];

        for (k = 0; k < i; k++)
            sum -= R[i*n + k]*p[k];
        p[i] = sum/R[i*n + i];
    }
    for (i = n - 1; i >= 0; i--) {
        double sum = p[i];

        for (k = i + 1; k < n; k++)
            sum -= R[k*n + i]*p[k];
        p[i] = sum/R[i*n + i];
    }
    return true;
}

static bool fit_one(const int16_t *amp, int n, int taps, int off, double dc,
                    double *h_out, double *score_out, double *level_out);

bool v90a_sd_fit(const int16_t *amp, int n, int taps, int *parity_out,
                 double *h_out, double *score_out, double *level_out)
{
    double best_h[V90A_SD_MAX_TAPS];
    double best_score = -1e30, best_level = 0.0, dc = 0.0;
    int best_off = -1, off, i;

    if (amp == NULL  ||  h_out == NULL  ||  taps <= 0  ||  (taps & 1)
        ||  taps > V90A_SD_MAX_TAPS)
        return false;
    for (i = 0; i < n; i++)
        dc += amp[i];
    dc /= (n > 0) ? n : 1;
    /*
     * The two T/2 parities are the "two eyes" and they are NOT
     * interchangeable: a filter fitted to be evaluated on even samples,
     * evaluated on odd ones, is a filter shifted by half a symbol, which is
     * not a delay the reference tolerates.  Everything else -- the slot phase,
     * the whole-symbol delay, the sign, the channel and the sampling instant --
     * the fit absorbs; this one thing it has to be told, so try both.
     */
    for (off = 0; off < 2; off++) {
        double h[V90A_SD_MAX_TAPS], score = 0.0, level = 0.0;

        if (!fit_one(amp, n, taps, off, dc, h, &score, &level))
            continue;
        if (score > best_score) {
            best_score = score;
            best_level = level;
            best_off = off;
            memcpy(best_h, h, (size_t) taps*sizeof(double));
        }
    }
    if (best_off < 0)
        return false;
    memcpy(h_out, best_h, (size_t) taps*sizeof(double));
    if (parity_out)
        *parity_out = best_off;
    if (score_out)
        *score_out = best_score;
    if (level_out)
        *level_out = best_level;
    return best_score >= FIT_SCORE_MIN;
}

static bool fit_one(const int16_t *amp, int n, int taps, int off, double dc,
                    double *h_out, double *score_out, double *level_out)
{
    int syms, train, i, j, k;
    double *R, *p, ref_energy = 0.0, res = 0.0, level = 0.0;
    double trace = 0.0, score;
    bool ok;

    /* Symbols the window can produce, allowing for the filter's span. */
    syms = (n - taps - off)/2;
    if (syms < 6*32)
        return false;
    syms -= syms%6;
    train = syms/2;
    train -= train%6;
    if (train < 6*8)
        return false;

    if ((R = calloc((size_t) taps*taps, sizeof(double))) == NULL)
        return false;
    if ((p = calloc((size_t) taps, sizeof(double))) == NULL) {
        free(R);
        return false;
    }
    /* Symbol k reads x[2k + taps - 1 - j] for tap j, so tap 0 is the most
     * recent sample -- the same convention v90a_fse_put() uses. */
    for (k = 0; k < train; k++) {
        double r = SD_REF[k%6];

        for (i = 0; i < taps; i++) {
            double xi = amp[2*k + off + taps - 1 - i] - dc;

            for (j = 0; j <= i; j++)
                R[i*taps + j] += xi*(amp[2*k + off + taps - 1 - j] - dc);
            p[i] += xi*r;
        }
    }
    for (i = 0; i < taps; i++) {
        for (j = i + 1; j < taps; j++)
            R[i*taps + j] = R[j*taps + i];
        trace += R[i*taps + i];
    }
    if (trace <= 0.0) {
        free(R);
        free(p);
        return false;
    }
    for (i = 0; i < taps; i++)
        R[i*taps + i] += FIT_RIDGE*trace/taps;
    ok = solve_spd(R, p, taps);
    free(R);
    if (!ok) {
        free(p);
        return false;
    }
    /* Held out: the second half of the window, which the fit never saw. */
    for (k = train; k < syms; k++) {
        double y = 0.0;
        double r = SD_REF[k%6];

        for (i = 0; i < taps; i++)
            y += p[i]*(amp[2*k + off + taps - 1 - i] - dc);
        res += (y - r)*(y - r);
        ref_energy += r*r;
        if (r != 0.0)
            level += fabs(y);
    }
    score = (ref_energy > 0.0) ? 1.0 - res/ref_energy : 0.0;
    memcpy(h_out, p, (size_t) taps*sizeof(double));
    free(p);
    if (score_out)
        *score_out = score;
    /* The fit normalises W to 1 by construction, so the level is reported in
     * the equaliser's own output units and is 1 on a good fit -- it is a
     * health reading, not a measurement of the line. */
    if (level_out)
        *level_out = (syms > train) ? level/((syms - train)*2/3) : 0.0;
    return true;
}

#define DEFAULT_REPS    16
#define MAX_REPS        64
/*
 * The score two windows must reach before the grid is believed.
 *
 * The score is (w - z)/w times the sign agreement, both in [0, 1] on a clean
 * Sd and both near zero on anything else, so their product is a single number
 * with a real floor: on the WRONG phase of a true Sd the zero slots land on
 * non-zero ones and (w - z)/w collapses, and on a signal that is not Sd at all
 * the sign agreement does.  0.55 is deliberately well under what a clean
 * capture gives (0.9+) because the whole point of this detector is to work
 * where the codeword slicer cannot -- through a channel, at an arbitrary
 * sampling phase, on an equaliser that is still converging.
 */
#define SCORE_MIN       0.55
/*
 * And two consecutive windows agreeing on the same phase, because one window
 * of a six-way choice on a noisy stream is a one-in-six guess when the signal
 * is absent.  TRN1d is scrambled ones on ONE level, so its zero slots are never
 * zero and it cannot pass (w - z)/w; Jd is differentially encoded signs on one
 * level and cannot either.  The realistic false positive is silence, which is
 * why a floor on w itself is also required.
 */
#define CONFIRM         2

struct v90a_sd_s {
    int      reps;
    int      window;              /* reps*6 symbols */
    double  *ring;
    int      len;                 /* symbols in the ring, capped at window */
    int      pos;                 /* next write index */
    int64_t  count;               /* symbols ever fed */

    bool     acquired;
    int      phase;               /* slot index of ring position 0 */
    double   level;
    double   score;

    int      cand_phase;
    int      cand_runs;
};

v90a_sd_t *v90a_sd_init(int reps)
{
    v90a_sd_t *s;

    if (reps <= 0)
        reps = DEFAULT_REPS;
    if (reps > MAX_REPS)
        reps = MAX_REPS;
    if ((s = calloc(1, sizeof(*s))) == NULL)
        return NULL;
    s->reps = reps;
    s->window = reps*6;
    if ((s->ring = calloc((size_t) s->window, sizeof(double))) == NULL) {
        free(s);
        return NULL;
    }
    s->phase = -1;
    s->cand_phase = -1;
    return s;
}

void v90a_sd_free(v90a_sd_t *s)
{
    if (s) {
        free(s->ring);
        free(s);
    }
}

void v90a_sd_reset(v90a_sd_t *s)
{
    if (s == NULL)
        return;
    memset(s->ring, 0, (size_t) s->window*sizeof(double));
    s->len = 0;
    s->pos = 0;
    s->count = 0;
    s->acquired = false;
    s->phase = -1;
    s->level = 0.0;
    s->score = 0.0;
    s->cand_phase = -1;
    s->cand_runs = 0;
}

/*
 * Score one candidate phase over the ring.
 *
 * p is the slot index of the OLDEST symbol in the ring.  §8.4.4's pattern is
 * +W, +0, +W, -W, -0, -W, so slots 1 and 4 are the zero ones, slots 0 and 2
 * carry one sign and slots 3 and 5 the other.
 *
 * Two independent things have to hold, and multiplying them is what stops
 * either alone from carrying a false lock:
 *
 *   depth = (w - z)/w   -- the zero slots really are the quiet ones
 *   sign  = |m+ - m-| / (2w) capped at 1 -- the four non-zero slots really do
 *                                           split + + - -
 *
 * Both are ratios, so the whole thing is invariant to the equaliser's scale,
 * which is the property that makes this usable before anything is calibrated.
 */
static double score_phase(const v90a_sd_t *s, int p, double *level_out)
{
    static const int is_zero[6] = { 0, 1, 0, 0, 1, 0 };
    static const int sign_of[6] = { +1, 0, +1, -1, 0, -1 };
    double w = 0.0, z = 0.0, mp = 0.0, mn = 0.0;
    int nw = 0, nz = 0, np = 0, nn = 0;
    double depth, sign, mag;
    int i;

    for (i = 0; i < s->len; i++) {
        /* ring[(pos - len + i) mod window] is the i'th oldest. */
        int idx = (s->pos - s->len + i + 2*s->window) % s->window;
        double v = s->ring[idx];
        int slot = (p + i)%6;

        if (is_zero[slot]) {
            z += fabs(v);
            nz++;
        } else {
            w += fabs(v);
            nw++;
            if (sign_of[slot] > 0) {
                mp += v;
                np++;
            } else {
                mn += v;
                nn++;
            }
        }
    }
    if (nw == 0  ||  nz == 0  ||  np == 0  ||  nn == 0)
        return 0.0;
    w /= nw;
    z /= nz;
    mp /= np;
    mn /= nn;
    if (level_out)
        *level_out = w;
    if (w <= 0.0)
        return 0.0;
    depth = (w - z)/w;
    if (depth < 0.0)
        depth = 0.0;
    mag = fabs(mp - mn)/(2.0*w);
    sign = (mag > 1.0) ? 1.0 : mag;
    return depth*sign;
}

bool v90a_sd_put(v90a_sd_t *s, double sym)
{
    double best_score = 0.0, best_level = 0.0;
    int best_phase = -1;
    int p;

    if (s == NULL)
        return false;
    s->ring[s->pos] = sym;
    s->pos = (s->pos + 1)%s->window;
    if (s->len < s->window)
        s->len++;
    s->count++;
    if (s->acquired  ||  s->len < s->window)
        return false;
    /* One decision per window rather than per symbol: the windows a sliding
     * decision would take are 5/6 the same data, so it costs six times the work
     * to produce correlated answers, and CONFIRM would then be measuring the
     * overlap rather than a repeat. */
    if (s->count%s->window != 0)
        return false;

    for (p = 0; p < 6; p++) {
        double level = 0.0;
        double sc = score_phase(s, p, &level);

        if (sc > best_score) {
            best_score = sc;
            best_level = level;
            best_phase = p;
        }
    }
    if (best_phase < 0  ||  best_score < SCORE_MIN  ||  best_level <= 0.0) {
        s->cand_phase = -1;
        s->cand_runs = 0;
        return false;
    }
    /* The candidate is the slot of the ring's OLDEST symbol; the caller wants
     * the slot the NEXT symbol will land on, and the ring holds a whole number
     * of six-symbol periods, so those are the same value. */
    if (best_phase == s->cand_phase) {
        s->cand_runs++;
    } else {
        s->cand_phase = best_phase;
        s->cand_runs = 1;
    }
    if (s->cand_runs < CONFIRM)
        return false;
    s->acquired = true;
    s->phase = best_phase;
    s->level = best_level;
    s->score = best_score;
    return true;
}

bool v90a_sd_acquired(const v90a_sd_t *s)
{
    return s != NULL  &&  s->acquired;
}

int v90a_sd_next_slot(const v90a_sd_t *s)
{
    return (s != NULL  &&  s->acquired) ? s->phase : -1;
}

double v90a_sd_level(const v90a_sd_t *s)
{
    return (s != NULL) ? s->level : 0.0;
}

double v90a_sd_score(const v90a_sd_t *s)
{
    return (s != NULL) ? s->score : 0.0;
}
