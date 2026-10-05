#include "k56flex_channel.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define HALF 64                         /* windowed-sinc half length */
#define HIST 256

typedef struct { double b0, b1, b2, a1, a2, z1, z2; } biquad_t;

struct k56flex_channel {
    k56flex_channel_cfg_t cfg;
    biquad_t lp[2];
    double hp_a, hp_x, hp_y;
    double hist[HIST];                  /* filtered network-rate samples */
    uint64_t n;
    double t;                           /* next client sample time, in network samples */
    unsigned rng;
};

static double gauss(k56flex_channel_t *c)
{
    double u1, u2;
    c->rng = c->rng * 1664525u + 1013904223u; u1 = (c->rng >> 8) / 16777216.0 + 1e-9;
    c->rng = c->rng * 1664525u + 1013904223u; u2 = (c->rng >> 8) / 16777216.0;
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2);
}

/* One RC section, matched-pole discretisation: y[n] = a y[n-1] + (1 - a) x[n].  Unlike a
 * bilinear Butterworth it has no zero at Nyquist, so the band edge is attenuated, not erased:
 * the T-spaced symbol samples of a real analogue loop keep some energy there. */
static void design_lp(biquad_t *q, double fc, double unused)
{
    double a = exp(-2.0 * M_PI * fc / 8000.0);
    (void)unused;
    q->b0 = 1.0 - a; q->b1 = 0; q->b2 = 0;
    q->a1 = -a; q->a2 = 0;
}

k56flex_channel_t *k56flex_channel_new(const k56flex_channel_cfg_t *cfg)
{
    k56flex_channel_t *c = calloc(1, sizeof(*c));
    if (!c) return NULL;
    c->cfg = *cfg;
    c->rng = cfg->seed ? cfg->seed : 1;
    if (cfg->lowpass_hz > 0) {              /* two RC sections */
        design_lp(&c->lp[0], cfg->lowpass_hz, 0);
        design_lp(&c->lp[1], cfg->lowpass_hz, 0);
    }
    if (cfg->highpass_hz > 0) c->hp_a = exp(-2.0 * M_PI * cfg->highpass_hz / 8000.0);
    /* client sample 0 lands `delay` network samples after network sample HALF */
    c->t = (double)HALF + 1.0 - (cfg->delay - floor(cfg->delay)) + 1.0;
    return c;
}

void k56flex_channel_free(k56flex_channel_t *ch) { free(ch); }

static double biq(biquad_t *q, double x)
{
    double o = q->b0 * x + q->z1;
    q->z1 = q->b1 * x - q->a1 * o + q->z2;
    q->z2 = q->b2 * x - q->a2 * o;
    return o;
}

static double hist_at(const k56flex_channel_t *c, int64_t idx)
{
    return idx < 0 ? 0.0 : c->hist[(unsigned)(idx % HIST)];
}

/* Band-limited interpolation at an exact fractional time. */
static double sinc_at(const k56flex_channel_t *c, double t)
{
    int64_t i0 = (int64_t)floor(t);
    double f = t - (double)i0, s = sin(M_PI * f), acc = 0;
    int j;
    for (j = -HALF + 1; j <= HALF; ++j) {
        double u = f - j, w, sinc;
        if (fabs(u) < 1e-9) { sinc = 1.0; }
        else sinc = (j & 1 ? -s : s) / (M_PI * u);
        w = 0.35875 + 0.48829 * cos(M_PI * u / HALF) + 0.14128 * cos(2 * M_PI * u / HALF)
            + 0.01168 * cos(3 * M_PI * u / HALF);
        acc += hist_at(c, i0 + j) * sinc * w;
    }
    return acc;
}

unsigned k56flex_channel_push(k56flex_channel_t *c, double in, int16_t out[2])
{
    unsigned produced = 0;
    double y = in * c->cfg.gain;
    if (c->cfg.lowpass_hz > 0) { y = biq(&c->lp[0], y); y = biq(&c->lp[1], y); }
    if (c->cfg.highpass_hz > 0) {
        double o = c->hp_a * (c->hp_y + y - c->hp_x);
        c->hp_x = y;
        c->hp_y = o;
        y = o;
    }
    c->hist[(unsigned)(c->n % HIST)] = y;
    ++c->n;
    {
        double step = 1.0 / (1.0 + c->cfg.clock_ppm * 1e-6);
        while (c->t + HALF <= (double)c->n - 1.0 && produced < 2) {
            double v = sinc_at(c, c->t) + c->cfg.dc + c->cfg.noise_rms * gauss(c);
            if (v > 32767) v = 32767;
            if (v < -32768) v = -32768;
            out[produced++] = (int16_t)lrint(v);
            c->t += step;
        }
    }
    return produced;
}
