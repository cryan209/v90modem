/*
 * v92_p3_eq.c -- equaliser and symbol timing for the V.92 Phase 3 upstream;
 * see v92_p3_eq.h and docs/v92_p3_rx_line_plan.md step 4.
 */
#include "v92_p3_eq.h"

#include <math.h>
#include <string.h>

#define HMASK (V92_P3_EQ_HISTORY - 1)

void v92_p3_eq_default_config(v92_p3_eq_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->ntaps = 31;
    cfg->nfb = 8;
    cfg->seed_symbols = 256;
    cfg->trn_symbols = 2040;
    cfg->mu = 0.01;
    cfg->mu_dd = 0.002;
    cfg->detector = V92_P3_EQ_DET_GRADIENT;
    cfg->timing_kp = 0.01;
    cfg->timing_ki = 0.00001;
    cfg->timing_avg = 1.0/16;
    cfg->max_ppm = 500.0;
    cfg->timing = true;
}

bool v92_p3_eq_init(v92_p3_eq_t *eq, const v92_p3_eq_config_t *cfg)
{
    const int h = V92_P3_EQ_INTERP_HALF;
    v92_p3_eq_config_t c;

    if (!eq)
        return false;
    if (cfg)
        c = *cfg;
    else
        v92_p3_eq_default_config(&c);
    if (c.ntaps < 1 || c.ntaps > V92_P3_EQ_MAX_TAPS || !(c.ntaps & 1)
        || c.nfb < 0 || c.nfb > V92_P3_EQ_MAX_FB
        || c.seed_symbols < c.ntaps + c.nfb || c.seed_symbols > V92_P3_EQ_MAX_SEED
        || c.trn_symbols < c.seed_symbols || c.max_ppm < 0.0)
        return false;
    memset(eq, 0, sizeof(*eq));
    eq->cfg = c;
    for (int k = 0; k < 2*h; k++)
        eq->window[k] = 0.42 - 0.5*cos(2.0*M_PI*(k + 0.5)/(2*h))
                      + 0.08*cos(4.0*M_PI*(k + 0.5)/(2*h));
    return true;
}

void v92_p3_eq_push(v92_p3_eq_t *eq, double sample)
{
    eq->x[eq->written & HMASK] = sample;
    eq->written++;
}

bool v92_p3_eq_start(v92_p3_eq_t *eq, int64_t start)
{
    int64_t earliest = start - eq->cfg.ntaps/2 - V92_P3_EQ_INTERP_HALF;

    if (earliest < 0 || earliest < eq->written - V92_P3_EQ_HISTORY + 1024)
        return false;
    eq->started = true;
    eq->start = start;
    eq->ref_from = 0;
    eq->ref_until = eq->cfg.trn_symbols;
    return true;
}

/* The input at absolute time t (samples).  Exact at integer t. */
static double interpolate(const v92_p3_eq_t *eq, double t)
{
    const int h = V92_P3_EQ_INTERP_HALF;
    int64_t base = (int64_t)floor(t);
    double frac = t - (double)base;
    double y = 0.0;

    if (frac == 0.0)
        return eq->x[base & HMASK];
    for (int k = -h + 1; k <= h; k++) {
        double u = k - frac;
        y += eq->x[(base + k) & HMASK]*sin(M_PI*u)/(M_PI*u)
           * eq->window[k + h - 1];
    }
    return y;
}

/* The input's slope at time t, per sample: a central difference on the
 * interpolator, whose half-step costs 2.5% of the slope at Nyquist. */
#define SLOPE_STEP 0.125
static double slope(const v92_p3_eq_t *eq, double t)
{
    return (interpolate(eq, t + SLOPE_STEP) - interpolate(eq, t - SLOPE_STEP))
         / (2*SLOPE_STEP);
}

/* Whether input around time t has arrived. */
static bool available(const v92_p3_eq_t *eq, double t)
{
    return (int64_t)floor(t + SLOPE_STEP) + V92_P3_EQ_INTERP_HALF < eq->written;
}

/* 8.5.7's reference: GPA (6.3, delay taps 5 and 23) zero-initialised and
 * fed ones, output 0 -> +L_U. */
static int reference_next(uint32_t *reg)
{
    int b = 1 ^ (int)((*reg >> 4) & 1) ^ (int)((*reg >> 22) & 1);

    *reg = (*reg << 1) | (uint32_t)b;
    return b ? -1 : 1;
}

#define LS_MAX (V92_P3_EQ_MAX_TAPS + V92_P3_EQ_MAX_FB)

/* Solve (A'A + lambda I) w = A'r by Gaussian elimination.  Row k of A is
 * the ntaps-long window of z starting at k followed by the nfb reference
 * symbols before k (zero before TRN1u: not known, and not TRN1u). */
static bool least_squares(const double *z, const int8_t *ref, int rows,
                          int n, int nfb, double fb_scale, double *w)
{
    static double m[LS_MAX][LS_MAX + 1];
    const int dim = n + nfb;
    double a[LS_MAX];
    double trace = 0.0;

    memset(m, 0, sizeof(m));
    for (int k = 0; k < rows; k++) {
        for (int i = 0; i < n; i++)
            a[i] = z[k + i];
        for (int j = 0; j < nfb; j++)
            a[n + j] = k - 1 - j >= 0 ? ref[k - 1 - j]*fb_scale : 0.0;
        for (int i = 0; i < dim; i++) {
            m[i][dim] += a[i]*ref[k];
            for (int j = i; j < dim; j++)
                m[i][j] += a[i]*a[j];
        }
    }
    for (int i = 0; i < dim; i++) {
        for (int j = 0; j < i; j++)
            m[i][j] = m[j][i];
        trace += m[i][i];
    }
    /* A whisker of ridge: the loop's band edge leaves near-null
     * directions the solve would otherwise fill with noise gain. */
    for (int i = 0; i < dim; i++)
        m[i][i] += 1e-6*trace/dim;
    for (int c = 0; c < dim; c++) {
        int p = c;

        for (int r = c + 1; r < dim; r++)
            if (fabs(m[r][c]) > fabs(m[p][c]))
                p = r;
        if (fabs(m[p][c]) < 1e-12)
            return false;
        if (p != c)
            for (int j = 0; j <= dim; j++) {
                double t = m[c][j];

                m[c][j] = m[p][j];
                m[p][j] = t;
            }
        for (int r = 0; r < dim; r++) {
            double f;

            if (r == c)
                continue;
            f = m[r][c]/m[c][c];
            for (int j = c; j <= dim; j++)
                m[r][j] -= f*m[c][j];
        }
    }
    for (int i = 0; i < dim; i++)
        w[i] = m[i][dim]/m[i][i];
    return true;
}

/* Energy centroid of the taps, in tap positions. */
static double centroid(const v92_p3_eq_t *eq)
{
    double num = 0.0;
    double den = 1e-30;

    for (int j = 0; j < eq->cfg.ntaps; j++) {
        double p = eq->taps[j]*eq->taps[j];

        num += j*p;
        den += p;
    }
    return num/den;
}

static bool seed(v92_p3_eq_t *eq)
{
    const int n = eq->cfg.ntaps;
    const int half = n/2;
    const int rows = eq->cfg.seed_symbols;
    int8_t ref[V92_P3_EQ_MAX_SEED];
    uint32_t reg = 0;

    /* All seed inputs at the nominal instants (tau = 0). */
    if (!available(eq, (double)(eq->start + rows - 1 + half)))
        return false;
    for (int m = 0; m < rows + n - 1; m++) {
        eq->zseed[m] = eq->x[(eq->start - half + m) & HMASK];
        eq->dzseed[m] = slope(eq, (double)(eq->start - half + m));
    }
    for (int k = 0; k < rows; k++)
        ref[k] = (int8_t)reference_next(&reg);
    {
        double w[LS_MAX];

        double power = 0.0;

        for (int m = 0; m < rows + n - 1; m++)
            power += eq->zseed[m]*eq->zseed[m];
        eq->fb_scale = sqrt(power/(rows + n - 1)) + 1e-9;
        if (!least_squares(eq->zseed, ref, rows, n, eq->cfg.nfb, eq->fb_scale, w))
            return false;
        memcpy(eq->taps, w, (size_t)n*sizeof(double));
        memcpy(eq->fb, w + n, (size_t)eq->cfg.nfb*sizeof(double));
    }
    eq->zcount = rows + n - 1;
    eq->centroid_ref = centroid(eq);
    eq->main_tap = 0;
    for (int j = 1; j < n; j++)
        if (fabs(eq->taps[j]) > fabs(eq->taps[eq->main_tap]))
            eq->main_tap = j;
    eq->seeded = true;
    return true;
}

static void record(v92_p3_eq_t *eq, double y, int d, int ref)
{
    eq->y = y;
    eq->decision = y >= 0.0 ? 1 : -1;
    eq->reference = ref;
    if (ref) {
        int agree = eq->decision == ref;

        if (eq->agree_fill == V92_P3_EQ_AGREE_WINDOW)
            eq->agree_sum -= eq->agree_ring[eq->agree_pos];
        else
            eq->agree_fill++;
        eq->agree_ring[eq->agree_pos] = (uint8_t)agree;
        eq->agree_sum += agree;
        eq->agree_pos = (eq->agree_pos + 1) % V92_P3_EQ_AGREE_WINDOW;
        if (eq->k >= eq->cfg.seed_symbols) {
            eq->post_seed_symbols++;
            eq->post_seed_agree += agree;
            eq->post_seed_err2 += (ref - y)*(ref - y);
        }
    }
    (void)d;
}

bool v92_p3_eq_step(v92_p3_eq_t *eq)
{
    const int n = eq->cfg.ntaps;
    const int half = n/2;
    const double *u;
    const double *du;
    double y = 0.0;
    int ref;
    double d;

    if (!eq || !eq->started)
        return false;
    if (!eq->seeded && !seed(eq))
        return false;

    if (eq->k < eq->cfg.seed_symbols) {
        /* The seed window itself, through the solved taps. */
        u = eq->zseed + eq->k;
        du = eq->dzseed + eq->k;
        if (eq->k == eq->cfg.seed_symbols - 1) {
            memcpy(eq->z, eq->zseed + eq->k + 1, (size_t)(n - 1)*sizeof(double));
            memcpy(eq->dz, eq->dzseed + eq->k + 1, (size_t)(n - 1)*sizeof(double));
        }
    } else {
        double t = (double)(eq->start - half + eq->zcount) + eq->tau;

        if (!available(eq, t))
            return false;
        eq->z[n - 1] = interpolate(eq, t);
        eq->dz[n - 1] = slope(eq, t);
        eq->zcount++;
        u = eq->z;
        du = eq->dz;
    }

    for (int j = 0; j < n; j++)
        y += eq->taps[j]*u[j];
    for (int j = 0; j < eq->cfg.nfb; j++)
        y += eq->fb[j]*eq->dhist[j];
    ref = eq->k >= eq->ref_from && eq->k < eq->ref_until
        ? reference_next(&eq->gpa) : 0;
    if (ref)
        d = ref;
    else if (eq->pam4) {
        double a = fabs(y)*sqrt(5.0);

        d = (a >= 2.0 ? 3.0 : 1.0)/sqrt(5.0);
        if (y < 0.0)
            d = -d;
    } else
        d = y >= 0.0 ? 1 : -1;
    record(eq, y, (int)(d >= 0.0 ? 1 : -1), ref);

    if (eq->k >= eq->cfg.seed_symbols && eq->hold)
        eq->tau += eq->freq;
    else if (eq->k >= eq->cfg.seed_symbols) {
        double energy = 1e-9;
        double e = d - y;
        double mu = ref ? eq->cfg.mu : eq->cfg.mu_dd;

        for (int j = 0; j < n; j++)
            energy += u[j]*u[j];
        for (int j = 0; j < eq->cfg.nfb; j++)
            energy += eq->dhist[j]*eq->dhist[j];
        for (int j = 0; j < n; j++)
            eq->taps[j] += mu*e*u[j]/energy;
        for (int j = 0; j < eq->cfg.nfb; j++)
            eq->fb[j] += mu*e*eq->dhist[j]/energy;

        /* Second-order timing loop on a slow average of the detector (for
         * M&M only the slow mean is clock error; see v92_trn2u.c). */
        if (eq->cfg.timing && eq->have_prev) {
            /* Positive when the input has moved later than the taps were
             * seeded for: the taps index later samples as j grows. */
            double err;

            if (eq->cfg.detector == V92_P3_EQ_DET_GRADIENT) {
                /* d(e^2)/d(tau) = -2 e dy/dtau: positive err means a later
                 * instant lowers the error. */
                double dy = 0.0;

                for (int j = 0; j < n; j++)
                    dy += eq->taps[j]*du[j];
                /* Normalised by the slope power, so err estimates the
                 * timing offset in samples whatever the channel: a loop
                 * with little ISI leaves far more slope at Nyquist than
                 * one that low-passes it, and an unnormalised gain that
                 * suits one oscillates on the other. */
                if (eq->dy2_avg == 0.0)
                    eq->dy2_avg = dy*dy;
                eq->dy2_avg += (dy*dy - eq->dy2_avg)/64.0;
                err = e*dy/(eq->dy2_avg + 1e-9);
            } else if (eq->cfg.detector == V92_P3_EQ_DET_MM)
                err = eq->d_prev*y - d*eq->y_prev;
            else
                err = centroid(eq) - eq->centroid_ref;
            double bound = eq->cfg.max_ppm*1e-6;

            eq->mm_avg += eq->cfg.timing_avg*(err - eq->mm_avg);
            eq->freq += eq->cfg.timing_ki*eq->mm_avg;
            if (eq->freq > bound)
                eq->freq = bound;
            if (eq->freq < -bound)
                eq->freq = -bound;
            eq->tau += eq->freq + eq->cfg.timing_kp*eq->mm_avg;
        }
        else
            eq->tau += eq->freq;   /* an open loop holds the set frequency */
    }
    if (eq->k >= eq->cfg.seed_symbols) {
        memmove(eq->z, eq->z + 1, (size_t)(n - 1)*sizeof(double));
        memmove(eq->dz, eq->dz + 1, (size_t)(n - 1)*sizeof(double));
    }
    /* The symbol just decided becomes the newest feedback input. */
    if (eq->cfg.nfb > 0) {
        memmove(eq->dhist + 1, eq->dhist, (size_t)(eq->cfg.nfb - 1)*sizeof(double));
        eq->dhist[0] = d*eq->fb_scale;
    }
    eq->y_prev = y;
    eq->d_prev = d;
    eq->have_prev = true;
    eq->k++;
    return true;
}

void v92_p3_eq_hold(v92_p3_eq_t *eq, bool hold)
{
    eq->hold = hold;
}

void v92_p3_eq_set_pam4(v92_p3_eq_t *eq, bool pam4)
{
    eq->pam4 = pam4;
}

void v92_p3_eq_train_from(v92_p3_eq_t *eq, int64_t k0, int n)
{
    eq->gpa = 0;
    eq->ref_from = k0;
    eq->ref_until = k0 + n;
    for (int64_t k = k0; k < eq->k && k < k0 + n; k++)
        (void)reference_next(&eq->gpa);
    if (eq->ref_from < eq->k)
        eq->ref_from = eq->k;
    eq->agree_fill = eq->agree_pos = eq->agree_sum = 0;
    eq->hold = false;
}

void v92_p3_eq_reference(int8_t *ref, int n)
{
    uint32_t reg = 0;

    for (int k = 0; k < n; k++)
        ref[k] = (int8_t)reference_next(&reg);
}

int v92_p3_eq_agree_x10(const v92_p3_eq_t *eq)
{
    return eq->agree_fill ? eq->agree_sum*1000/eq->agree_fill : 0;
}

double v92_p3_eq_snr_db(const v92_p3_eq_t *eq)
{
    if (eq->post_seed_symbols == 0 || eq->post_seed_err2 <= 0.0)
        return 0.0;
    return 10.0*log10((double)eq->post_seed_symbols/eq->post_seed_err2);
}

double v92_p3_eq_post_seed_agreement(const v92_p3_eq_t *eq)
{
    return eq->post_seed_symbols
         ? (double)eq->post_seed_agree/(double)eq->post_seed_symbols : 0.0;
}

int v92_p3_eq_main_tap(const v92_p3_eq_t *eq)
{
    return eq->main_tap;
}

double v92_p3_eq_ppm(const v92_p3_eq_t *eq)
{
    return eq->freq*1e6;
}
