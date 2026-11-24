/*
 * v92_prefilter.c -- V.92 6.4.2 prefilter design (see v92_prefilter.h).
 *
 * The design: with H the convolution by the channel, choose w to make
 * (Hw)[d] = 1 and minimise
 *
 *     J(w) = sum over k outside {d .. d+lb} of (Hw)[k]^2  +  s^2 |w|^2,
 *
 * where s^2 = noise2 is the line's noise.  Normalised to unit energy the
 * prefilter's main tap is g = 1/|w|, so the main tap's SNR is
 * g^2/(s^2 + J_isi g^2) = 1/J(w): the minimum of J is the best SNR the
 * delay allows.  That is a quadratic with one linear constraint,
 * w = A^-1 h_d / (h_d' A^-1 h_d), A = sum_k h_k h_k' + s^2 I over the
 * penalised rows k, h_k the k-th row of H.
 */
#include "v92_prefilter.h"

#include <math.h>
#include <string.h>

/* Cholesky solve of the symmetric positive definite n x n system a x = b
 * (a destroyed); false if a is not positive definite. */
static bool solve_spd(double *a, double *x, const double *b, int n)
{
    for (int j = 0; j < n; j++) {
        double s = a[j*n + j];

        for (int k = 0; k < j; k++)
            s -= a[j*n + k]*a[j*n + k];
        if (s <= 0.0)
            return false;
        s = sqrt(s);
        a[j*n + j] = s;
        for (int i = j + 1; i < n; i++) {
            double t = a[i*n + j];

            for (int k = 0; k < j; k++)
                t -= a[i*n + k]*a[j*n + k];
            a[i*n + j] = t/s;
        }
    }
    for (int i = 0; i < n; i++) {
        double t = b[i];

        for (int k = 0; k < i; k++)
            t -= a[i*n + k]*x[k];
        x[i] = t/a[i*n + i];
    }
    for (int i = n - 1; i >= 0; i--) {
        double t = x[i];

        for (int k = i + 1; k < n; k++)
            t -= a[k*n + i]*x[k];
        x[i] = t/a[i*n + i];
    }
    return true;
}

/* The smaller of the two steps at linear value |r|. */
static double step_at(const double *levels, double r)
{
    double a = fabs(r);
    int lo = 0, hi = 127;

    while (lo < hi) {
        int m = (lo + hi)/2;

        if (levels[m] < a)
            lo = m + 1;
        else
            hi = m;
    }
    double up = lo < 127 ? levels[lo + 1] - levels[lo] : levels[lo] - levels[lo - 1];
    double down = lo > 0 ? levels[lo] - levels[lo - 1] : up;

    return up < down ? up : down;
}

bool v92_pf_fit_channel(v92_pf_channel_t *ch, const double *r, const double *t,
                        int n, const double *levels)
{
    static double a[V92_PF_CH*V92_PF_CH];
    double v[V92_PF_CH];
    double e2 = 0.0, q2 = 0.0, trace = 0.0;

    if (!ch || !r || !t || n < 4*V92_PF_CH)
        return false;
    memset(ch, 0, sizeof(*ch));
    memset(a, 0, sizeof(a));
    memset(v, 0, sizeof(v));
    for (int k = 0; k < n; k++) {
        /* row: t[k + CH - 1 - j] is the symbol V92_PF_CH_PRE - j after
         * the one r[k] is centred on. */
        const double *p = t + k;

        for (int i = 0; i < V92_PF_CH; i++) {
            double pi = p[V92_PF_CH - 1 - i];

            v[i] += pi*r[k];
            for (int j = 0; j <= i; j++)
                a[i*V92_PF_CH + j] += pi*p[V92_PF_CH - 1 - j];
        }
    }
    for (int i = 0; i < V92_PF_CH; i++) {
        for (int j = i + 1; j < V92_PF_CH; j++)
            a[i*V92_PF_CH + j] = a[j*V92_PF_CH + i];
        trace += a[i*V92_PF_CH + i];
    }
    for (int i = 0; i < V92_PF_CH; i++)
        a[i*V92_PF_CH + i] += 1e-9*trace/V92_PF_CH;
    if (!solve_spd(a, ch->h, v, V92_PF_CH))
        return false;
    for (int k = 0; k < n; k++) {
        const double *p = t + k;
        double y = 0.0;

        for (int j = 0; j < V92_PF_CH; j++)
            y += ch->h[j]*p[V92_PF_CH - 1 - j];
        e2 += (r[k] - y)*(r[k] - y);
        if (levels) {
            double st = step_at(levels, r[k]);

            q2 += st*st/12.0;
        }
    }
    ch->rows = n;
    ch->fit_err2 = e2/n;
    ch->quant2 = q2/n;
    ch->noise2 = fmax(ch->fit_err2 - ch->quant2, 1e-6*ch->fit_err2);
    for (int j = 0; j < V92_PF_CH; j++)
        ch->power += ch->h[j]*ch->h[j];
    return true;
}

bool v92_pf_design(v92_pf_design_t *out, const v92_pf_channel_t *ch,
                   int lw, int lb)
{
    enum { KMAX = V92_PF_CH + V92_PF_MAX_W - 1 };
    static double base[V92_PF_MAX_W*V92_PF_MAX_W];
    static double a[V92_PF_MAX_W*V92_PF_MAX_W];
    double hd[V92_PF_MAX_W], w[V92_PF_MAX_W];
    double best_j = -1.0;
    const int nk = V92_PF_CH + lw - 1;
    double hmax = 0.0;
    int main_k = 0;

    if (!out || !ch || lw < 1 || lw > V92_PF_MAX_W || lb < 0 || lb > V92_PF_MAX_B)
        return false;
    memset(out, 0, sizeof(*out));
    /* base = H'H + s^2 I (Toeplitz in the channel's autocorrelation). */
    for (int i = 0; i < lw; i++)
        for (int j = 0; j < lw; j++) {
            int l = i > j ? i - j : j - i;
            double s = 0.0;

            for (int k = 0; k + l < V92_PF_CH; k++)
                s += ch->h[k]*ch->h[k + l];
            base[i*lw + j] = s + (i == j ? ch->noise2 : 0.0);
        }
    for (int j = 0; j < V92_PF_CH; j++)
        if (fabs(ch->h[j]) > hmax) {
            hmax = fabs(ch->h[j]);
            main_k = j;
        }
    /* No prefilter: the main tap against everything else and the noise. */
    {
        double isi = ch->power - hmax*hmax;

        out->identity_snr_db = 10.0*log10(hmax*hmax/(ch->noise2 + isi));
    }
    for (int kd = 0; kd + lb < nk; kd++) {
        double jmin, hah = 0.0, ww = 0.0, isi = 0.0;

        memcpy(a, base, (size_t)lw*lw*sizeof(double));
        /* The free rows k = kd .. kd+lb come out of H'H. */
        for (int k = kd; k <= kd + lb; k++)
            for (int i = 0; i < lw; i++) {
                double hi = k - i >= 0 && k - i < V92_PF_CH ? ch->h[k - i] : 0.0;

                if (hi == 0.0)
                    continue;
                for (int j = 0; j < lw; j++)
                    if (k - j >= 0 && k - j < V92_PF_CH)
                        a[i*lw + j] -= hi*ch->h[k - j];
            }
        for (int i = 0; i < lw; i++)
            hd[i] = kd - i >= 0 && kd - i < V92_PF_CH ? ch->h[kd - i] : 0.0;
        if (!solve_spd(a, w, hd, lw))
            continue;
        for (int i = 0; i < lw; i++)
            hah += hd[i]*w[i];
        if (hah <= 0.0)
            continue;
        /* w/hah meets (Hw)[kd] = 1; J = 1/hah there. */
        jmin = 1.0/hah;
        if (best_j >= 0.0 && jmin >= best_j)
            continue;
        best_j = jmin;
        for (int i = 0; i < lw; i++) {
            w[i] /= hah;
            ww += w[i]*w[i];
        }
        out->lw = lw;
        out->lb = lb;
        out->delay = kd;
        out->gain = 1.0/sqrt(ww);
        for (int i = 0; i < lw; i++)
            out->w[i] = w[i]*out->gain;
        for (int k = 0; k < nk; k++) {
            double c = 0.0;

            for (int i = 0; i < lw; i++)
                if (k - i >= 0 && k - i < V92_PF_CH)
                    c += ch->h[k - i]*w[i];
            if (k > kd && k <= kd + lb)
                out->b[k - kd - 1] = c;
            else if (k != kd)
                isi += c*c;
        }
        out->interference = isi;
        out->snr_db = 10.0*log10(1.0/jmin);
    }
    (void)main_k;
    return best_j > 0.0;
}
