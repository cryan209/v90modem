/*
 * v92_line_channel.c -- test model of the analogue upstream path; see
 * v92_line_channel.h.
 */
#include "v92_line_channel.h"
#include "v92_line_channel_r4.h"

#include <math.h>
#include <string.h>

const double v92_line_channel_r4_taps[] = { V92_LINE_CHANNEL_R4_TAPS };
const int v92_line_channel_r4_ntaps =
    (int)(sizeof(v92_line_channel_r4_taps)/sizeof(v92_line_channel_r4_taps[0]));

static double gaussian(v92_line_channel_t *c)
{
    double u1;
    double u2;
    double r;

    if (c->have_spare) {
        c->have_spare = false;
        return c->spare;
    }
    do {
        c->rng ^= c->rng << 13;
        c->rng ^= c->rng >> 17;
        c->rng ^= c->rng << 5;
        u1 = (c->rng + 1.0)/4294967297.0;
        c->rng ^= c->rng << 13;
        c->rng ^= c->rng >> 17;
        c->rng ^= c->rng << 5;
        u2 = (c->rng + 1.0)/4294967297.0;
    } while (u1 <= 0.0);
    r = sqrt(-2.0*log(u1));
    c->spare = r*sin(2.0*M_PI*u2);
    c->have_spare = true;
    return r*cos(2.0*M_PI*u2);
}

bool v92_line_channel_init(v92_line_channel_t *c,
                           const v92_line_channel_config_t *cfg)
{
    const int h = V92_LINE_CHANNEL_INTERP_HALF;

    if (!c || !cfg || cfg->ntaps < 0 || cfg->ntaps > 256
        || (cfg->ntaps > 0 && !cfg->taps)
        || cfg->phase < 0.0 || cfg->phase >= 1.0
        || fabs(cfg->ppm) > 5000.0 || cfg->noise_rms < 0.0)
        return false;
    memset(c, 0, sizeof(*c));
    c->cfg = *cfg;
    c->ideal = (cfg->ntaps == 0 || (cfg->ntaps == 1 && cfg->taps[0] == 1.0))
            && cfg->phase == 0.0 && cfg->ppm == 0.0 && !cfg->antialias;
    if (cfg->antialias) {
        const int n = V92_LINE_CHANNEL_AA_TAPS;
        const double fc = 3700.0/16000.0;
        double sum = 0.0;

        for (int k = 0; k < n; k++) {
            double m = k - (n - 1)/2.0;
            double s = m == 0.0 ? 2.0*fc : sin(2.0*M_PI*fc*m)/(M_PI*m);
            double w = 0.42 - 0.5*cos(2.0*M_PI*k/(n - 1))
                     + 0.08*cos(4.0*M_PI*k/(n - 1));

            c->aa_taps[k] = s*w;
            sum += s*w;
        }
        for (int k = 0; k < n; k++)
            c->aa_taps[k] /= sum;
    }
    /* One 8 kHz A/D sample is two 16 kHz samples; a fast A/D (ppm > 0)
     * takes its samples closer together in the modem's time. */
    c->step = 2.0/(1.0 + cfg->ppm*1.0e-6);
    c->t = 2.0*cfg->phase;
    for (int k = 0; k < 2*h; k++)
        c->window[k] = 0.42 - 0.5*cos(2.0*M_PI*(k + 0.5)/(2*h))
                     + 0.08*cos(4.0*M_PI*(k + 0.5)/(2*h));
    c->rng = cfg->seed ? cfg->seed : 0x9e3779b9u;
    return true;
}

static double antialias(v92_line_channel_t *c, double x)
{
    double y = 0.0;

    if (!c->cfg.antialias)
        return x;
    c->aa_pos = (c->aa_pos + 255) & 255;
    c->aa_hist[c->aa_pos] = x;
    for (int k = 0; k < V92_LINE_CHANNEL_AA_TAPS; k++)
        y += c->aa_taps[k]*c->aa_hist[(c->aa_pos + k) & 255];
    return y;
}

static double fir(v92_line_channel_t *c, double x)
{
    double y = 0.0;
    int n = c->cfg.ntaps;

    if (n == 0)
        return x;
    c->fir_pos = (c->fir_pos + 255) & 255;
    c->fir_hist[c->fir_pos] = x;
    for (int k = 0; k < n; k++)
        y += c->cfg.taps[k]*c->fir_hist[(c->fir_pos + k) & 255];
    return y;
}

/* Band-limited value of the filtered 16 kHz stream at time t (16 kHz
 * sample units).  Blackman-windowed sinc; the stream is already confined
 * below 4 kHz by the modem's own reconstruction, so its cut-off at 8 kHz
 * passes it untouched. */
static double interpolate(const v92_line_channel_t *c, double t)
{
    const int h = V92_LINE_CHANNEL_INTERP_HALF;
    long base = (long)floor(t);
    double frac = t - base;
    double y = 0.0;

    if (frac == 0.0)
        return c->ring[(uint64_t)base % V92_LINE_CHANNEL_HISTORY];
    for (int k = -h + 1; k <= h; k++) {
        double x = k - frac;
        double s = sin(M_PI*x)/(M_PI*x);
        y += c->ring[(uint64_t)(base + k) % V92_LINE_CHANNEL_HISTORY]
           * s*c->window[k + h - 1];
    }
    return y;
}

int v92_line_channel_put(v92_line_channel_t *c, const double *in, int n,
                         double *out, int max)
{
    const int h = V92_LINE_CHANNEL_INTERP_HALF;
    int produced = 0;

    for (int i = 0; i < n; i++) {
        c->ring[c->written % V92_LINE_CHANNEL_HISTORY] = antialias(c, fir(c, in[i]));
        c->written++;
        /* Emit every A/D instant whose interpolation window is complete.
         * The integer instants of an ideal channel need no lookahead. */
        while (produced < max) {
            double need = c->ideal ? c->t : c->t + h;

            if (need > (double)(c->written - 1))
                break;
            out[produced] = c->ideal
                ? c->ring[(uint64_t)c->t % V92_LINE_CHANNEL_HISTORY]
                : interpolate(c, c->t);
            if (c->cfg.noise_rms > 0.0)
                out[produced] += c->cfg.noise_rms*gaussian(c);
            produced++;
            c->t += c->step;
        }
    }
    return produced;
}
