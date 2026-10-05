#include "k56flex_rxfe.h"
#include "k56flex_probe.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define RING 16384u
#define XR 4096u
#define SINC_HALF 64
#define HIST 64u                  /* symbols kept for delayed adaptation */
#define CORR_LEN 320u
#define CORR_SPAN 160
#define ID_SAMPLES 288            /* ID_A (22 pairs) + ID_B (2 pairs) before P1 */
#define N K56FLEX_EQ_TAPS
#define LOOK K56FLEX_EQ_LOOKAHEAD

typedef enum { FE_ENERGY, FE_CORR, FE_RUN } fe_state_t;

struct k56flex_rxfe {
    k56flex_law_t law;
    k56flex_rxfe_sink_fn sink;
    void *user;
    fe_state_t state;
    float ring[RING];
    uint64_t nin;                 /* samples received */
    /* acquisition */
    double blk_energy;
    unsigned blk_n;
    double floor_rms;
    unsigned blocks_seen;
    uint64_t onset, searched_from;
    float ref[CORR_LEN];
    float gain;
    uint64_t locked_at;
    /* resampler */
    double pos;                   /* next input position (fractional) taken as a symbol */
    double ppm;                   /* learned clock offset */
    uint64_t tnext;               /* first symbol of the timing block being collected */
    float refring[XR];            /* references by symbol index */
    double prev_hr[1024], prev_hi[1024];   /* previous block's channel response (TFFT/2 bins) */
    unsigned char prev_ok[1024];
    int have_prev;
    double ehist[8];              /* block-to-block timing slides since the last rate update */
    unsigned nehist;
    uint64_t m;                   /* symbols resampled */
    float xr[XR];
    /* equalizer */
    double w[N];
    double R[N][N];               /* weighted input autocorrelation while training */
    double pv[N];                 /* weighted input/reference cross-correlation */
    double w0[N];                 /* prior: the acquisition-time centre-tap solution */
    double load;                  /* diagonal loading */
    unsigned ls_count;
    float dv[HIST][N];
    float ycap[HIST];
    unsigned hist_n;              /* symbols delivered so far */
    uint64_t nout;
    float mu;
    double err_pow, ref_pow;
};

k56flex_rxfe_t *k56flex_rxfe_new(k56flex_law_t law, k56flex_rxfe_sink_fn sink, void *user)
{
    k56flex_rxfe_t *fe = calloc(1, sizeof(*fe));
    k56flex_probe_t p;
    int16_t blk[6];
    unsigned i = 0;
    if (!fe) return NULL;
    fe->law = law;
    fe->sink = sink;
    fe->user = user;
    /* the known P1 waveform: server state is cleared and the source seeded with FFFF */
    if (k56flex_probe_init(&p, K56FLEX_PROBE_1, law, 0) < 0) { free(fe); return NULL; }
    k56flex_probe_seed(&p, 0xffff);
    while (i < CORR_LEN) {
        unsigned n = k56flex_probe_block(&p, blk), j;
        for (j = 0; j < n && i < CORR_LEN; ++j) fe->ref[i++] = blk[j];
    }
    return fe;
}

void k56flex_rxfe_free(k56flex_rxfe_t *fe) { free(fe); }

int k56flex_rxfe_locked(const k56flex_rxfe_t *fe) { return fe->state == FE_RUN; }
float k56flex_rxfe_gain(const k56flex_rxfe_t *fe) { return fe->gain; }
float k56flex_rxfe_ppm(const k56flex_rxfe_t *fe) { return (float)fe->ppm; }
unsigned k56flex_rxfe_locked_at(const k56flex_rxfe_t *fe) { return (unsigned)fe->locked_at; }
float k56flex_rxfe_snr_db(const k56flex_rxfe_t *fe)
{
    return fe->err_pow > 0 && fe->ref_pow > 0 ? (float)(10.0 * log10(fe->ref_pow / fe->err_pow)) : 0.0f;
}

static float in_at(const k56flex_rxfe_t *fe, uint64_t i) { return fe->ring[i % RING]; }

static void try_acquire(k56flex_rxfe_t *fe)
{
    uint64_t base, p, best_p = 0;
    int64_t lo, hi, cand;
    double best = -1, mean, rr = 0, cm1 = 0, cp1 = 0, c0 = 0, frac = 0;
    unsigned i;
    if (fe->nin < fe->onset + ID_SAMPLES + CORR_SPAN + CORR_LEN + 8) return;
    base = fe->onset + ID_SAMPLES;
    lo = (int64_t)base - CORR_SPAN;
    hi = (int64_t)base + CORR_SPAN;
    if (lo < 1) lo = 1;
    for (i = 0; i < CORR_LEN; ++i) rr += (double)fe->ref[i] * fe->ref[i];
    for (cand = lo - 1; cand <= hi + 1; ++cand) {
        double sx = 0, sxx = 0, sxr = 0, c;
        for (i = 0; i < CORR_LEN; ++i) sx += in_at(fe, (uint64_t)cand + i);
        mean = sx / CORR_LEN;
        for (i = 0; i < CORR_LEN; ++i) {
            double x = in_at(fe, (uint64_t)cand + i) - mean;
            sxx += x * x;
            sxr += x * fe->ref[i];
        }
        c = sxx > 0 ? sxr / sqrt(sxx * rr) : 0;
        if (cand == lo - 1) cm1 = c;
        if (cand >= lo && cand <= hi && c > best) { best = c; best_p = (uint64_t)cand; }
    }
    if (best < 0.5) {                        /* not the P1 probe: look for the next onset */
        fe->state = FE_ENERGY;
        fe->searched_from = fe->nin;
        return;
    }
    p = best_p;
    {
        double sx = 0, sxx = 0, sxr = 0, c, g;
        for (cand = (int64_t)p - 1; cand <= (int64_t)p + 1; cand += 1) {
            sx = sxx = sxr = 0;
            for (i = 0; i < CORR_LEN; ++i) sx += in_at(fe, (uint64_t)cand + i);
            mean = sx / CORR_LEN;
            for (i = 0; i < CORR_LEN; ++i) {
                double x = in_at(fe, (uint64_t)cand + i) - mean;
                sxx += x * x;
                sxr += x * fe->ref[i];
            }
            c = sxx > 0 ? sxr / sqrt(sxx * rr) : 0;
            if (cand == (int64_t)p - 1) cm1 = c; else if (cand == (int64_t)p) { c0 = c; g = sxr / rr; fe->gain = (float)g; } else cp1 = c;
        }
        {
            double den = cm1 - 2 * c0 + cp1;
            frac = den != 0 ? 0.5 * (cm1 - cp1) / den : 0;
            if (frac > 0.5) frac = 0.5;
            if (frac < -0.5) frac = -0.5;
        }
    }
    if (fe->gain < 1e-3f) fe->gain = 1e-3f;
    fe->pos = (double)p;
    (void)frac;
    fe->m = 0;
    fe->nout = 0;
    fe->hist_n = 0;
    fe->ppm = 0;
    fe->tnext = 0;
    fe->have_prev = 0;
    fe->nehist = 0;
    fe->mu = 0.4f;
    memset(fe->w, 0, sizeof(fe->w));
    fe->w[LOOK] = 1.0 / fe->gain;
    memcpy(fe->w0, fe->w, sizeof(fe->w0));
    memset(fe->R, 0, sizeof(fe->R));
    memset(fe->pv, 0, sizeof(fe->pv));
    fe->ls_count = 0;
    fe->locked_at = p;
    fe->state = FE_RUN;
}

static void detect_energy(k56flex_rxfe_t *fe, float x)
{
    fe->blk_energy += (double)x * x;
    if (++fe->blk_n < 40) return;
    {
        double rms = sqrt(fe->blk_energy / 40.0);
        uint64_t start = fe->nin - 40;
        fe->blk_energy = 0;
        fe->blk_n = 0;
        if (fe->blocks_seen < 4) {
            fe->floor_rms = (fe->floor_rms * fe->blocks_seen + rms) / (fe->blocks_seen + 1);
            ++fe->blocks_seen;
            return;
        }
        if (start < fe->searched_from) return;
        if (rms > (fe->floor_rms * 6 > 60.0 ? fe->floor_rms * 6 : 60.0)) {
            /* refine the onset inside this block and the one before it */
            float peak = 0;
            uint64_t i, lo = start >= 40 ? start - 40 : 0;
            for (i = start; i < start + 40; ++i) if (fabsf(in_at(fe, i)) > peak) peak = fabsf(in_at(fe, i));
            fe->onset = start;
            for (i = lo; i < start + 40; ++i)
                if (fabsf(in_at(fe, i)) > 0.25f * peak && fabsf(in_at(fe, i)) > fe->floor_rms * 4) { fe->onset = i; break; }
            fe->state = FE_CORR;
        }
    }
}

/* Band-limited interpolation at a fractional input position. */
static float interp(const k56flex_rxfe_t *fe, double pos)
{
    int64_t i0 = (int64_t)floor(pos);
    double f = pos - (double)i0, sn = sin(M_PI * f), acc = 0;
    int j;
    for (j = -SINC_HALF + 1; j <= SINC_HALF; ++j) {
        double u = f - j, w, sinc;
        sinc = fabs(u) < 1e-9 ? 1.0 : (j & 1 ? -sn : sn) / (M_PI * u);
        w = 0.35875 + 0.48829 * cos(M_PI * u / SINC_HALF) + 0.14128 * cos(2 * M_PI * u / SINC_HALF)
            + 0.01168 * cos(3 * M_PI * u / SINC_HALF);
        acc += in_at(fe, (uint64_t)(i0 + j)) * sinc * w;
    }
    return (float)acc;
}

static void emit_symbol(k56flex_rxfe_t *fe, float y)
{
    fe->ycap[(unsigned)(fe->hist_n % HIST)] = y;
    ++fe->hist_n;
    if (fe->sink) fe->sink(fe->user, y);
}

static void run_symbols(k56flex_rxfe_t *fe)
{
    while (fe->state == FE_RUN && fe->nin > (uint64_t)floor(fe->pos) + SINC_HALF + 1) {
        unsigned k, slot = (unsigned)(fe->m % XR);
        double y = 0;
        fe->xr[slot] = interp(fe, fe->pos);
        ++fe->m;
        fe->pos += 1.0 + fe->ppm * 1e-6;
        if (fe->m <= LOOK) continue;
        {
            unsigned h = (unsigned)(fe->hist_n % HIST);
            for (k = 0; k < N; ++k) {
                float d = fe->xr[(unsigned)((fe->m - 1 + XR * 4 - k) % XR)];
                fe->dv[h][k] = d;
                y += fe->w[k] * d;
            }
        }
        ++fe->nout;
        emit_symbol(fe, (float)y);
    }
}

void k56flex_rxfe_push(k56flex_rxfe_t *fe, int16_t sample)
{
    fe->ring[fe->nin % RING] = (float)sample;
    ++fe->nin;
    switch (fe->state) {
    case FE_ENERGY: detect_energy(fe, (float)sample); break;
    case FE_CORR: try_acquire(fe); break;
    case FE_RUN: break;
    }
    if (fe->state == FE_CORR) try_acquire(fe);
    if (fe->state == FE_RUN) run_symbols(fe);
}

/* Clock-offset tracking.  Per block of TBLK symbols the channel's frequency response is
 * estimated by spectral deconvolution, H(f) = X(f) conj(R(f)) / (|R(f)|^2 + eps), where X is the
 * resampled input and R the reference (the expected stream, or the client's own decisions once
 * the stream is data).  Dividing by R removes the reference's own colour, which is what made
 * correlation peaks hop between lobes.  The channel's group delay is read from the phase slope
 * of H across frequency; a drifting clock slides it linearly from block to block, so the
 * block-to-block change sets the resampling rate.  Nothing here depends on the equalizer, so the
 * loop is not coupled to its adaptation. */
#define TBLK 2048u
#define TFFT 2048
#define TK 48                          /* bin separation of the phase-difference pairs */
#define TIMING_FIT 3u
#define TIMING_GAIN 0.7
#define TIMING_OUTLIER 0.45            /* samples per block: more than this is a bad block */

static void fft(double *re, double *im, int n)
{
    int i, j, k, m;
    for (i = 1, j = 0; i < n; ++i) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) { double t = re[i]; re[i] = re[j]; re[j] = t; t = im[i]; im[i] = im[j]; im[j] = t; }
    }
    for (m = 2; m <= n; m <<= 1) {
        double ang = -2.0 * M_PI / m, wr = cos(ang), wi = sin(ang);
        for (i = 0; i < n; i += m) {
            double cr = 1, ci = 0;
            for (k = 0; k < m / 2; ++k) {
                int a = i + k, b = i + k + m / 2;
                double tr = re[b] * cr - im[b] * ci, ti = re[b] * ci + im[b] * cr;
                re[b] = re[a] - tr; im[b] = im[a] - ti;
                re[a] += tr; im[a] += ti;
                {
                    double nr = cr * wr - ci * wi;
                    ci = cr * wi + ci * wr;
                    cr = nr;
                }
            }
        }
    }
}

/* Channel response over symbols [first, first + TFFT), by spectral deconvolution.  Bins where
 * the reference or the estimate is too weak to trust are flagged invalid. */
static int block_response(const k56flex_rxfe_t *fe, uint64_t first, double *hr, double *hi, unsigned char *ok)
{
    static double xr_[TFFT], xi_[TFFT], rr[TFFT], ri[TFFT];
    double rmax = 0, hmax = 0;
    int i;
    for (i = 0; i < TFFT; ++i) {
        double w = 0.5 - 0.5 * cos(2.0 * M_PI * (i + 0.5) / TFFT);
        xr_[i] = fe->xr[(first + (uint64_t)i) % XR] * w; xi_[i] = 0;
        rr[i] = fe->refring[(first + (uint64_t)i) % XR] * w; ri[i] = 0;
    }
    fft(xr_, xi_, TFFT);
    fft(rr, ri, TFFT);
    for (i = 0; i < TFFT / 2; ++i) { double p = rr[i] * rr[i] + ri[i] * ri[i]; if (p > rmax) rmax = p; }
    if (rmax <= 0) return 0;
    for (i = 0; i < TFFT / 2; ++i) {
        double p = rr[i] * rr[i] + ri[i] * ri[i], d = p + 1e-4 * rmax, m;
        hr[i] = (xr_[i] * rr[i] + xi_[i] * ri[i]) / d;                 /* X conj(R) / |R|^2 */
        hi[i] = (xi_[i] * rr[i] - xr_[i] * ri[i]) / d;
        m = hr[i] * hr[i] + hi[i] * hi[i];
        ok[i] = p > 1e-2 * rmax;
        if (ok[i] && m > hmax) hmax = m;
    }
    for (i = 0; i < TFFT / 2; ++i)
        if (ok[i] && hr[i] * hr[i] + hi[i] * hi[i] < 1e-2 * hmax) ok[i] = 0;
    return 1;
}

/* Timing slide since the previous block, in samples.  Each bin's phase is compared with the
 * same bin one block earlier, so the channel's own (frequency-dependent) group delay cancels and
 * only the common sliding of a drifting clock remains: delta-phi = -2 pi f * delta-tau. */
static int block_slide(const k56flex_rxfe_t *fe, const double *hr, const double *hi, const unsigned char *ok, double *dtau)
{
    double num = 0, den = 0;
    int i;
    for (i = (int)(0.02 * TFFT); i < (int)(0.46 * TFFT); ++i) {
        double f = (double)i / TFFT, re, im, w;
        if (!ok[i] || !fe->prev_ok[i]) continue;
        re = hr[i] * fe->prev_hr[i] + hi[i] * fe->prev_hi[i];             /* H conj(Hprev) */
        im = hi[i] * fe->prev_hr[i] - hr[i] * fe->prev_hi[i];
        w = sqrt((hr[i] * hr[i] + hi[i] * hi[i]) * (fe->prev_hr[i] * fe->prev_hr[i] + fe->prev_hi[i] * fe->prev_hi[i]));
        num += w * f * atan2(im, re);
        den += w * f * f;
    }
    if (den <= 0) return 0;
    *dtau = -num / (2.0 * M_PI * den);
    return 1;
}

static void timing_block(k56flex_rxfe_t *fe, uint64_t first)
{
    static double hr[TFFT / 2], hi[TFFT / 2];
    static unsigned char ok[TFFT / 2];
    double d;
    if (!block_response(fe, first, hr, hi, ok)) { fe->have_prev = 0; return; }
    if (fe->have_prev && block_slide(fe, hr, hi, ok, &d) && fabs(d) < TIMING_OUTLIER) {
        fe->ehist[fe->nehist++] = d;                  /* samples per block */
        if (fe->nehist >= TIMING_FIT) {
            double mean = 0;
            unsigned i;
            for (i = 0; i < fe->nehist; ++i) mean += fe->ehist[i];
            mean /= fe->nehist;
            fe->ppm += TIMING_GAIN * 1e6 * mean / TBLK;       /* pulse later each block: step faster */
            if (fe->ppm > 500) fe->ppm = 500;
            if (fe->ppm < -500) fe->ppm = -500;
            fe->nehist = 0;
        }
    }
    memcpy(fe->prev_hr, hr, sizeof(hr));
    memcpy(fe->prev_hi, hi, sizeof(hi));
    memcpy(fe->prev_ok, ok, sizeof(ok));
    fe->have_prev = 1;
}

#define LS_SYMBOLS 45000u
#define LS_LAMBDA 0.9998
#define LS_EVERY 128u
#define LS_LOAD 2e-2                 /* diagonal loading, relative to the mean input power */

static void ls_accumulate(k56flex_rxfe_t *fe, const float *d, double ref)
{
    unsigned i, j;
    for (i = 0; i < N; ++i) {
        double xi = d[i];
        fe->pv[i] = LS_LAMBDA * fe->pv[i] + xi * ref;
        for (j = i; j < N; ++j) fe->R[i][j] = LS_LAMBDA * fe->R[i][j] + xi * d[j];
    }
}

/* Solve (R + load I) w = p by Cholesky; the weights are left alone if R is not positive. */
static void ls_solve(k56flex_rxfe_t *fe)
{
    static double L[N][N];
    double y[N], tr = 0, delta;
    unsigned i, j, k;
    for (i = 0; i < N; ++i) tr += fe->R[i][i];
    delta = LS_LOAD * tr / N + 1e-3;
    for (i = 0; i < N; ++i)
        for (j = 0; j <= i; ++j) {
            double sum = fe->R[j][i] + (i == j ? delta : 0.0);
            for (k = 0; k < j; ++k) sum -= L[i][k] * L[j][k];
            if (i == j) {
                if (sum <= 0) return;
                L[i][i] = sqrt(sum);
            } else {
                L[i][j] = sum / L[j][j];
            }
        }
    for (i = 0; i < N; ++i) {
        double sum = fe->pv[i] + delta * fe->w0[i];     /* regularise toward the prior, not toward zero */
        for (k = 0; k < i; ++k) sum -= L[i][k] * y[k];
        y[i] = sum / L[i][i];
    }
    for (i = N; i-- > 0;) {
        double sum = y[i];
        for (k = i + 1; k < N; ++k) sum -= L[k][i] * fe->w[k];
        fe->w[i] = sum / L[i][i];
    }
}

uint64_t k56flex_rxfe_symbols(const k56flex_rxfe_t *fe) { return fe->hist_n; }

void k56flex_rxfe_block(k56flex_rxfe_t *fe, uint64_t first_symbol, const int16_t *reference, unsigned n, int known)
{
    unsigned j;
    if (fe->state != FE_RUN || n == 0 || n > HIST || first_symbol + n > fe->hist_n || fe->hist_n - first_symbol > HIST) return;
    if (first_symbol == 0) fe->tnext = 0;
    for (j = 0; j < n; ++j) {
        unsigned idx = (unsigned)((first_symbol + j) % HIST), k;
        double ref = (double)reference[j];
        uint64_t sym = first_symbol + j;
        const float *d = fe->dv[idx];
        fe->refring[sym % XR] = (float)ref;
        if (known >= 0) {
            /* Least squares throughout: on the known stream against the expected levels,
             * afterwards decision-directed against the client's own slicer output.  A negative
             * `known` holds the equalizer (the sparse parameter stream is not worth adapting on)
             * but timing keeps running on the client's decisions. */
            ls_accumulate(fe, d, ref);
            if (++fe->ls_count % LS_EVERY == 0) ls_solve(fe);
            if (known) {
                double y = 0, e;
                for (k = 0; k < N; ++k) y += fe->w[k] * d[k];
                e = ref - y;
                fe->err_pow = 0.998 * fe->err_pow + 0.002 * e * e;
                fe->ref_pow = 0.998 * fe->ref_pow + 0.002 * ref * ref;
            }
        }
        if (sym + 1 >= fe->tnext + TBLK && sym + 1 - fe->tnext == TBLK) {
            timing_block(fe, fe->tnext);
            fe->tnext += TBLK;
        }
    }
}
