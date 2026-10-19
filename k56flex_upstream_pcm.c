/* See k56flex_upstream_pcm.h. This is an offline, fixed-rate acquisition
 * experiment; it does not replace or configure the live V.34 receiver. */
#include "k56flex_upstream_pcm.h"
#include "p3_demod.h"

#include <math.h>
#include <string.h>

/* Nine complex T/2 observations, solved as two real-output FIRs. Allowing
 * both I and Q as independent columns also handles conjugate orientation. */
enum { TAPS = 9, COLS = 2 * TAPS, TRN = 512 };

typedef struct { double re, im; } point_t;

static void reference(point_t ref[TRN], unsigned tap)
{
    static const double re[4] = {1, 1, -1, -1};
    static const double im[4] = {1, -1, -1, 1};
    uint32_t history = 0;
    for (int n = 0; n < TRN; ++n) {
        unsigned dibit = 0;
        for (unsigned i = 0; i < 2; ++i) {
            unsigned y = 1 ^ ((history >> (tap - 1)) & 1)
                           ^ ((history >> 22) & 1);
            history = (history << 1) | y;
            dibit |= y << i;
        }
        ref[n].re = re[dibit] * 0.7071067811865475;
        ref[n].im = im[dibit] * 0.7071067811865475;
    }
}

static void row(const p3_result_t *r, int at, double x[COLS])
{
    for (int j = 0; j < TAPS; ++j) {
        x[2*j] = r->symbols[at + j - TAPS/2].re;
        x[2*j+1] = r->symbols[at + j - TAPS/2].im;
    }
}

static point_t equalize(const p3_result_t *r, int at, const double w[COLS][2])
{
    double x[COLS];
    point_t y = {0, 0};
    row(r, at, x);
    for (int j = 0; j < COLS; ++j) {
        y.re += x[j] * w[j][0];
        y.im += x[j] * w[j][1];
    }
    return y;
}

static int train(const p3_result_t *r, int start, const point_t ref[TRN],
                 double w[COLS][2])
{
    double a[COLS][COLS+2] = {{0}}, trace = 0;
    /* Do not train on the PP/TRN edge, nor on the held-out tail. */
    for (int n = 16; n < 272; ++n) {
        double x[COLS];
        row(r, start + 2*n, x);
        for (int j = 0; j < COLS; ++j) {
            for (int k = 0; k < COLS; ++k) a[j][k] += x[j]*x[k];
            a[j][COLS] += x[j]*ref[n].re;
            a[j][COLS+1] += x[j]*ref[n].im;
        }
    }
    for (int j = 0; j < COLS; ++j) trace += a[j][j];
    if (trace < 1e-12) return 0;
    /* Local regularization, not an ITU constant. */
    for (int j = 0; j < COLS; ++j) a[j][j] += trace * 1e-6 / COLS;
    for (int j = 0; j < COLS; ++j) {
        int pivot = j;
        for (int k = j+1; k < COLS; ++k)
            if (fabs(a[k][j]) > fabs(a[pivot][j])) pivot = k;
        if (fabs(a[pivot][j]) < trace * 1e-12) return 0;
        for (int k = j; k < COLS+2; ++k) {
            double tmp = a[j][k]; a[j][k] = a[pivot][k]; a[pivot][k] = tmp;
        }
        double scale = a[j][j];
        for (int k = j; k < COLS+2; ++k) a[j][k] /= scale;
        for (int i = 0; i < COLS; ++i) if (i != j) {
            scale = a[i][j];
            for (int k = j; k < COLS+2; ++k) a[i][k] -= scale*a[j][k];
        }
    }
    for (int j = 0; j < COLS; ++j) {
        w[j][0] = a[j][COLS]; w[j][1] = a[j][COLS+1];
    }
    return 1;
}

static double error(point_t a, point_t b)
{
    return (a.re-b.re)*(a.re-b.re) + (a.im-b.im)*(a.im-b.im);
}

int k56flex_upstream_pcm_decode(const int16_t *pcm, int count, unsigned tap,
                               k56flex_upstream_pcm_result_t *out)
{
    p3_demod_t d;
    p3_result_t *r;
    point_t ref[TRN];
    double best = 0, w[COLS][2];
    int start = -1, events = 0;
    if (!out) return -1;
    memset(out, 0, sizeof(*out));
    out->trn_sample = out->report_sample = out->end_sample = -1;
    if (!pcm || count <= 0 || count > 80000 || (tap != 5 && tap != 18)) return -1;
    r = p3_result_alloc(count + 100, 0);
    if (!r) return -1;
    p3_demod_init(&d, P3_BAUD_3200, P3_CARRIER_LOW, 8000);
    /* Use only p3's vendored matched-filter bank and carrier mixer. The
     * observation stream must not be steered by its blind CMA or AGC.
     * Fixed-phase T/2 observations retain both timing parities for the FIR.
     * Disable the Godard correction explicitly, independent of env knobs. */
    d.use_rrc_frontend = true;
    d.emit_half_baud = true;
    d.bypass_equalizer = true;
    d.cma_freeze_after_sample = 0;
    d.rrc_agc_gain = 0.0001f;
    d.ted_low_coeff[2] = d.ted_high_coeff[2] = d.ted_mixed_coeff = 0;
    p3_demod_process(&d, pcm, count, 0, r);
    reference(ref, tap);
    /* Find zero-initialized TRN without knowing the signal's onset. Search
     * a non-edge 128-symbol window; test both complex orientations. */
    for (int at = 2*(288+16+64); at + 2*TRN + TAPS < r->symbol_count; ++at) {
        double cr = 0, ci = 0, dr = 0, di = 0, power = 0;
        for (int n = 32; n < 160; ++n) {
            const p3_symbol_t *s = &r->symbols[at+2*n];
            cr += s->re*ref[n].re + s->im*ref[n].im;
            ci += s->im*ref[n].re - s->re*ref[n].im;
            dr += s->re*ref[n].re - s->im*ref[n].im;
            di += s->im*ref[n].re + s->re*ref[n].im;
            power += s->re*s->re + s->im*s->im;
        }
        double corr = power > 1e-12 ? sqrt(fmax(cr*cr+ci*ci, dr*dr+di*di)/(128*power)) : 0;
        if (corr > best) { best = corr; start = at; }
    }
    out->correlation = best;
    /* These are conservative experimental confidence floors, not spec
     * thresholds. Held-out training and the preceding preamble must agree
     * before report bits are ever examined. */
    if (start < 0 || best < 0.65 || !train(r, start, ref, w)) goto done;
    double e = 0;
    for (int n = 288; n < 480; ++n)
        e += error(equalize(r, start+2*n, w), ref[n]);
    out->trn_rms = sqrt(e/192);
    if (out->trn_rms > 0.25) goto done;
    e = 0;
    for (int n = 16; n < 272; ++n) {
        /* V.34 10.1.3.6, 48-symbol PP, repeated six times. */
        int k = (n%48)/4, i = n%4;
        double angle = 3.14159265358979323846 * (k*i + (k%3 == 1 ? 4 : 0))/6;
        point_t pp = {cos(angle), sin(angle)};
        e += error(equalize(r, start - 2*288 + 2*n, w), pp);
    }
    out->pp_rms = sqrt(e/256);
    e = 0;
    for (int n = -64; n < 16; ++n) {
        /* S alternates (++,-+); S-bar alternates (--,+-). */
        point_t s = {(n&1) ? -0.7071067811865475 : 0.7071067811865475,
                     0.7071067811865475};
        if (n >= 0) { s.re = -s.re; s.im = -s.im; }
        e += error(equalize(r, start - 2*(288+16) + 2*n, w), s);
    }
    out->s_rms = sqrt(e/80);
    if (out->pp_rms > 0.3 || out->s_rms > 0.3) goto done;
    out->acquired = 1;
    /* p3's timestamps subtract 32T for its full equalizer. Here we use
     * only its matched-filter output: restore that bookkeeping offset. */
    out->trn_sample = r->symbols[start].sample_index + 80;
    k56flex_upstream_t rx;
    k56flex_upstream_init(&rx, tap);
    for (int at = start; at + TAPS/2 < r->symbol_count; at += 2) {
        point_t y = equalize(r, at, w);
        int ev = k56flex_upstream_symbol(&rx, (float)y.re, (float)y.im);
        if (ev < 0) continue;
        if (ev & K56U_REPORT) {
            out->ext = rx.report.ext; out->report = rx.report.report;
            out->report_sample = r->symbols[at].sample_index + 80;
        }
        if (ev & K56U_REPORT_END) out->end_sample = r->symbols[at].sample_index + 80;
        events |= ev;
        if (rx.ended) break;
    }
done:
    p3_result_free(r);
    return events;
}
