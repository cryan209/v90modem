/*
 * v34_pp_fit.c — see v34_pp_fit.h.
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "v34_pp_fit.h"

/*
 * Ridge on the normal equations, scaled by the trace so it is independent of
 * the line level.  The received signal is band-limited, so R is close to
 * singular and an unregularised solve returns enormous taps that fit the
 * training half and explain nothing out of sample.
 */
#define FIT_RIDGE       1e-3
/*
 * The held-out score a fit must reach.  PP through a dispersive channel scores
 * above 0.9; TRN, noise and silence cannot be mapped to PP by a fixed filter
 * and score at or below zero.  0.60 sits in the empty middle -- lower than the
 * Sd fit's 0.80 because PP is a 288-symbol wideband sequence rather than a
 * six-symbol pattern, so a real channel leaves more residual in it.
 */
#define FIT_SCORE_MIN   0.60

void v34_pp_reference(int i, double *re, double *im)
{
    /* i = 4k + I, k = 0..71, I = 0..3;
     *   PP(i) = exp(j*pi*(k*I + 4)/6)  if k mod 3 == 1
     *         = exp(j*pi*k*I/6)        otherwise                    (10-1) */
    int k = (i/4)%72;
    int I = i%4;
    double ang = ((k%3) == 1) ? M_PI*(k*I + 4)/6.0 : M_PI*k*I/6.0;

    if (re)
        *re = cos(ang);
    if (im)
        *im = sin(ang);
}

/*
 * Hermitian positive-definite solve, R h = p, by Cholesky.  R is stored as two
 * real matrices (real and imaginary parts) because the rest of this tree is C89
 * plus a little, and complex.h buys nothing at this size.
 */
static bool solve_hpd(double *Rr, double *Ri, double *pr, double *pi, int n)
{
    int i, j, k;

    for (i = 0; i < n; i++) {
        for (j = 0; j <= i; j++) {
            double sr = Rr[i*n + j], si = Ri[i*n + j];

            for (k = 0; k < j; k++) {
                /* sum -= L[i][k] * conj(L[j][k]) */
                double ar = Rr[i*n + k], ai = Ri[i*n + k];
                double br = Rr[j*n + k], bi = -Ri[j*n + k];

                sr -= ar*br - ai*bi;
                si -= ar*bi + ai*br;
            }
            if (i == j) {
                if (sr <= 0.0)
                    return false;
                Rr[i*n + i] = sqrt(sr);
                Ri[i*n + i] = 0.0;
            } else {
                double d = Rr[j*n + j];

                Rr[i*n + j] = sr/d;
                Ri[i*n + j] = si/d;
            }
        }
    }
    /* forward: L y = p */
    for (i = 0; i < n; i++) {
        double sr = pr[i], si = pi[i];

        for (k = 0; k < i; k++) {
            double ar = Rr[i*n + k], ai = Ri[i*n + k];

            sr -= ar*pr[k] - ai*pi[k];
            si -= ar*pi[k] + ai*pr[k];
        }
        pr[i] = sr/Rr[i*n + i];
        pi[i] = si/Rr[i*n + i];
    }
    /* back: L^H h = y */
    for (i = n - 1; i >= 0; i--) {
        double sr = pr[i], si = pi[i];

        for (k = i + 1; k < n; k++) {
            double ar = Rr[k*n + i], ai = -Ri[k*n + i];

            sr -= ar*pr[k] - ai*pi[k];
            si -= ar*pi[k] + ai*pr[k];
        }
        pr[i] = sr/Rr[i*n + i];
        pi[i] = si/Rr[i*n + i];
    }
    return true;
}

/* Which sub-sample phase symbol k lands on, and the sample it anchors to. */
static int symbol_anchor(int k, double step, int branches, int *branch)
{
    double pos = k*step;
    int n = (int) floor(pos);
    double frac = pos - n;
    int b = (int) floor(frac*branches + 0.5)%branches;

    if (branch)
        *branch = b;
    return n;
}

static int count_branches(double step)
{
    int b;

    /* The distinct fractional parts of k*step repeat with the denominator of
     * step as a rational.  8000/3200 = 5/2 gives two; 16000/3200 = 5 gives
     * one.  Anything not resolving inside the cap is clamped, which costs
     * accuracy and not correctness -- the residual simply rises. */
    for (b = 1; b <= V34_PP_MAX_BRANCH; b++) {
        double v = step*b;

        if (fabs(v - floor(v + 0.5)) < 1e-9)
            return b;
    }
    return V34_PP_MAX_BRANCH;
}

bool v34_pp_fit(const int16_t *amp, int n, double fs, double baud, double fc,
                int taps, v34_pp_fit_t *out)
{
    double step = fs/baud;
    int branches = count_branches(step);
    int syms, train, b, i, j, k;
    double dc = 0.0, ref_energy = 0.0, res = 0.0;
    bool ok = true;

    if (amp == NULL  ||  out == NULL  ||  taps <= 0  ||  taps > V34_PP_MAX_TAPS)
        return false;
    memset(out, 0, sizeof(*out));
    out->taps = taps;
    out->branches = branches;

    syms = (int) ((n - taps - 4)/step);
    if (syms > V34_PP_SYMBOLS)
        syms = V34_PP_SYMBOLS;
    if (syms < 96)
        return false;
    out->symbols = syms;
    train = syms/2;

    for (i = 0; i < n; i++)
        dc += amp[i];
    dc /= n;

    for (b = 0; b < branches  &&  ok; b++) {
        double *Rr = calloc((size_t) taps*taps, sizeof(double));
        double *Ri = calloc((size_t) taps*taps, sizeof(double));
        double *pr = calloc((size_t) taps, sizeof(double));
        double *pi = calloc((size_t) taps, sizeof(double));
        double trace = 0.0;

        if (!Rr || !Ri || !pr || !pi) {
            free(Rr); free(Ri); free(pr); free(pi);
            return false;
        }
        for (k = 0; k < train; k++) {
            int bk, anchor = symbol_anchor(k, step, branches, &bk);
            double rr, ri, ph, cr, ci;

            if (bk != b)
                continue;
            v34_pp_reference(k%V34_PP_SYMBOLS, &rr, &ri);
            /* The carrier the filter cannot supply goes into the reference. */
            ph = 2.0*M_PI*fc*k/baud;
            cr = cos(ph);
            ci = sin(ph);
            {
                double er = rr*cr - ri*ci;
                double ei = rr*ci + ri*cr;

                for (i = 0; i < taps; i++) {
                    double xi = amp[anchor + taps - 1 - i] - dc;

                    for (j = 0; j <= i; j++)
                        Rr[i*taps + j] += xi*(amp[anchor + taps - 1 - j] - dc);
                    /* p = sum x* .ref, and x is real. */
                    pr[i] += xi*er;
                    pi[i] += xi*ei;
                }
            }
        }
        for (i = 0; i < taps; i++) {
            for (j = i + 1; j < taps; j++) {
                Rr[i*taps + j] = Rr[j*taps + i];
                Ri[i*taps + j] = -Ri[j*taps + i];
            }
            trace += Rr[i*taps + i];
        }
        if (trace <= 0.0) {
            ok = false;
        } else {
            for (i = 0; i < taps; i++)
                Rr[i*taps + i] += FIT_RIDGE*trace/taps;
            ok = solve_hpd(Rr, Ri, pr, pi, taps);
            if (ok) {
                memcpy(out->h_re[b], pr, (size_t) taps*sizeof(double));
                memcpy(out->h_im[b], pi, (size_t) taps*sizeof(double));
            }
        }
        free(Rr); free(Ri); free(pr); free(pi);
    }
    if (!ok)
        return false;

    for (k = train; k < syms; k++) {
        double yr, yi, rr, ri, ph, cr, ci, er, ei;

        v34_pp_fit_apply(out, amp, n, fs, baud, k, &yr, &yi);
        v34_pp_reference(k%V34_PP_SYMBOLS, &rr, &ri);
        ph = 2.0*M_PI*fc*k/baud;
        cr = cos(ph);
        ci = sin(ph);
        er = rr*cr - ri*ci;
        ei = rr*ci + ri*cr;
        res += (yr - er)*(yr - er) + (yi - ei)*(yi - ei);
        ref_energy += er*er + ei*ei;
    }
    out->score = (ref_energy > 0.0) ? 1.0 - res/ref_energy : 0.0;
    return out->score >= FIT_SCORE_MIN;
}

bool v34_pp_fit_search(const int16_t *amp, int n, double fs, double baud,
                       double fc, int taps, int symbols,
                       v34_pp_fit_t *out, int *offset_out)
{
    double step = fs/baud;
    int period = (int) (V34_PP_PERIOD*step + 0.5);
    int need = (int) (symbols*step) + taps + 8;
    v34_pp_fit_t best;
    int best_off = -1, off;

    if (amp == NULL  ||  out == NULL  ||  n < need + period)
        return false;
    memset(&best, 0, sizeof(best));
    best.score = -1e30;
    for (off = 0; off < period; off++) {
        v34_pp_fit_t f;

        /* v34_pp_fit() reports its score whether or not it passes, so the
         * sweep ranks on the score and the threshold is applied once at the
         * end -- otherwise a sweep would stop at the first acceptable offset
         * rather than the best one. */
        (void) v34_pp_fit(amp + off, need, fs, baud, fc, taps, &f);
        if (f.score > best.score) {
            best = f;
            best_off = off;
        }
    }
    if (best_off < 0)
        return false;
    *out = best;
    if (offset_out)
        *offset_out = best_off;
    return best.score >= FIT_SCORE_MIN;
}

void v34_pp_fit_apply(const v34_pp_fit_t *f, const int16_t *amp, int n,
                      double fs, double baud, int k, double *re, double *im)
{
    double step = fs/baud;
    double dc = 0.0, yr = 0.0, yi = 0.0;
    int b, anchor, i;

    if (f == NULL || amp == NULL) {
        if (re) *re = 0.0;
        if (im) *im = 0.0;
        return;
    }
    for (i = 0; i < n; i++)
        dc += amp[i];
    dc /= n;
    anchor = symbol_anchor(k, step, f->branches, &b);
    for (i = 0; i < f->taps; i++) {
        int idx = anchor + f->taps - 1 - i;
        double xi = (idx >= 0 && idx < n) ? (amp[idx] - dc) : 0.0;

        yr += f->h_re[b][i]*xi;
        yi += f->h_im[b][i]*xi;
    }
    if (re)
        *re = yr;
    if (im)
        *im = yi;
}
