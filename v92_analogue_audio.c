#include "v92_analogue_audio.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "v90_analogue_fse.h"
#include "v90_analogue_sd.h"
#include "v91.h"

/* A streaming interpolator retains raw audio, not G.711. The read cursor
 * advances continuously; changing its increment never inserts/drops symbols. */
#define AUDIO_RING 256
#define AUDIO_RADIUS 16
#define SD_WINDOW 512
#define SD_SLIDE 128

typedef struct {
    double samples[AUDIO_RING];
    int64_t count;
} audio_history_t;

struct v92a_audio_s {
    v92a_t *core;
    unsigned rate;
    audio_history_t tx, rx;
    double tx_next, rx_next;
    v90a_fse_t *eq;
    int16_t acquisition[SD_WINDOW];
    int acquisition_count;
    bool acquired, training, timing;
    unsigned training_symbols, tracking_symbols;
    double sd_level, trn_level, input_trn;
    double centre_reference, centre_average, frequency, integral;
    uint64_t clipped, rx_symbols;
};

static double sinc(double x)
{
    const double pi = 3.14159265358979323846;
    return fabs(x) < 1e-12 ? 1.0 : sin(pi*x)/(pi*x);
}

bool v92_pcm_interpolator_init(v92_pcm_interpolator_t *s, int factor)
{
    if (!s || (factor < 1 || factor > 6)) return false;
    memset(s, 0, sizeof(*s));
    s->factor = factor;
    for (int p = 0; p < factor; p++) {
        double sum = 0;
        for (int j = 0; j < V92_AUDIO_INTERPOLATOR_TAPS; j++) {
            double t = j - 8.0 + (double)p/factor;
            double c = fabs(t) < 8 ? sinc(t)*sinc(t/8) : 0;
            /* Preserve the input lattice exactly, including digital zero. */
            if (!p) c = j == 8 ? 1 : 0;
            s->coefficients[p][j] = c;
            sum += c;
        }
        for (int j = 0; j < V92_AUDIO_INTERPOLATOR_TAPS; j++)
            s->coefficients[p][j] /= sum * V92_AUDIO_LINEAR_SCALE;
    }
    return true;
}

static int16_t quantize(double value, uint64_t *clipped)
{
    if (value > 32767) { (*clipped)++; return 32767; }
    if (value < -32768) { (*clipped)++; return -32768; }
    return (int16_t)lround(value);
}

int v92_pcm_interpolator_put(v92_pcm_interpolator_t *s, int16_t input,
                             int16_t *output)
{
    if (!s || !output || (s->factor < 1 || s->factor > 6)) return 0;
    memmove(s->history+1, s->history, sizeof(s->history)-sizeof(s->history[0]));
    s->history[0] = input;
    for (int p = 0; p < s->factor; p++) {
        double value = 0;
        for (int j = 0; j < V92_AUDIO_INTERPOLATOR_TAPS; j++)
            value += s->history[j]*s->coefficients[p][j];
        output[p] = quantize(value, &s->clipped);
    }
    return s->factor;
}

static void history_put(audio_history_t *s, double sample)
{
    s->samples[s->count++ % AUDIO_RING] = sample;
}

static double history_at(const audio_history_t *s, int64_t index)
{
    if (index < 0 || index >= s->count || index < s->count-AUDIO_RING) return 0;
    return s->samples[index % AUDIO_RING];
}

static double interpolate(const audio_history_t *s, double position, double cutoff)
{
    int64_t base = (int64_t)floor(position);
    double value = 0, weight = 0;
    for (int j = -AUDIO_RADIUS+1; j <= AUDIO_RADIUS; j++) {
        double t = position-(base+j);
        double c = sinc(t*cutoff)*sinc(t/AUDIO_RADIUS)*cutoff;
        value += history_at(s, base+j)*c;
        weight += c;
    }
    return value/weight;
}

v92a_audio_t *v92a_audio_init_rate(const v92a_config_t *cfg, unsigned rate)
{
    if (rate < 8000 || rate > 48000) return NULL;
    v92a_audio_t *s = calloc(1, sizeof(*s));
    if (!s) return NULL;
    s->core = v92a_init_line(cfg);
    s->eq = v90a_fse_init(32, V90A_FSE_MU_CMA);
    if (!s->core || !s->eq) { v92a_audio_free(s); return NULL; }
    s->rate = rate;
    s->tx_next = -AUDIO_RADIUS;
    v90a_fse_set_mode(s->eq, V90A_FSE_FROZEN);
    uint8_t trn = v91_ucode_to_codeword((v91_law_t)cfg->law, cfg->u_info, true);
    uint8_t sd = v91_ucode_to_codeword((v91_law_t)cfg->law, cfg->u_info+16, true);
    s->trn_level = fabs(v91_codeword_to_linear((v91_law_t)cfg->law, trn));
    s->sd_level = fabs(v91_codeword_to_linear((v91_law_t)cfg->law, sd));
    return s;
}

v92a_audio_t *v92a_audio_init(const v92a_config_t *cfg, unsigned rx_phase)
{
    if (rx_phase >= V92_AUDIO_PER_SYMBOL) return NULL;
    /* Kept for source compatibility. Acquisition now absorbs input phase. */
    return v92a_audio_init_rate(cfg, V92_AUDIO_RATE);
}

void v92a_audio_free(v92a_audio_t *s)
{
    if (!s) return;
    v92a_free(s->core);
    v90a_fse_free(s->eq);
    free(s);
}

v92a_t *v92a_audio_core(v92a_audio_t *s) { return s ? s->core : NULL; }

int v92a_audio_tx(v92a_audio_t *s, int16_t *samples, int count)
{
    if (!s || !samples || count <= 0) return 0;
    for (int i = 0; i < count; i++) {
        /* V.92 §6.2: the upstream clock follows the recovered downstream
         * network clock, rather than a second free-running oscillator. */
        double step = 16000.0/s->rate/(1+s->frequency);
        int64_t need = (int64_t)floor(s->tx_next)+AUDIO_RADIUS;
        while (s->tx.count <= need) {
            int16_t value;
            v92a_tx(s->core, &value, 1);
            history_put(&s->tx, value);
        }
        double value = interpolate(&s->tx, s->tx_next, 1);
        samples[i] = quantize(value/V92_AUDIO_LINEAR_SCALE, &s->clipped);
        s->tx_next += step;
    }
    return count;
}

static void equalized(v92a_audio_t *s, int16_t input)
{
    double y;
    if (v90a_fse_put(s->eq, &input, 1, &y, 1) != 1) return;
    int mode = v92a_rx_training(s->core);
    if (mode == 1 && !s->training) {
        s->training = true;
        /* Sd constrains only its harmonics. Its minimum-norm fit is not a
         * broadband inverse; start TRN1d's §9.5.2.1.4 blind fit from a
         * localized impulse instead of keeping that periodic matched filter. */
        double h[32] = {0};
        h[16] = 1/s->input_trn;
        h[17] = 0.1/s->input_trn;
        v90a_fse_set_taps(s->eq, h, 32, 1);
        v90a_fse_set_mode(s->eq, V90A_FSE_CMA);
        v90a_fse_set_mu(s->eq, V90A_FSE_MU_CMA);
    }
    if (mode == 1) {
        s->training_symbols++;
        if (s->training_symbols == 1500) {
            v90a_fse_set_mode(s->eq, V90A_FSE_DD);
            v90a_fse_set_mu(s->eq, 0.15);
            s->centre_reference = s->centre_average = v90a_fse_centre(s->eq);
            s->integral = s->frequency;
            s->timing = false;
        }
    } else if (mode == 2) {
        v90a_fse_set_mode(s->eq, V90A_FSE_DD);
        v90a_fse_set_mu(s->eq, V90A_FSE_MU_TRACK);
    }
    double scale = s->training ? s->trn_level : s->sd_level;
    int16_t sample = quantize(y*scale, &s->clipped);
    fprintf(stderr, "EQRAW %llu %d %d %.8f\n", (unsigned long long)s->rx_symbols, mode, sample, y);
    v92a_rx(s->core, &sample, 1);
    s->rx_symbols++;
    if (v90a_fse_mode(s->eq) == V90A_FSE_DD) {
        double target = mode == 1 ? (y >= 0 ? 1 : -1)
                                  : v92a_rx_decision(s->core)/scale;
        double tolerance = mode == 1 ? 1 : v92a_rx_tolerance(s->core)/scale;
        v90a_fse_decide(s->eq, target, tolerance);
    }
    /* The adaptive filter absorbs fractional phase. Its delay centroid is
     * the phase detector for the slow clock servo, including on zero-excess-
     * bandwidth PCM. Keep that delay near its trained value so a ppm error
     * cannot walk the impulse response out of the tap window. No sample
     * duplication/deletion and no changes to the digital DS0 are involved. */
    if (s->timing) {
        s->centre_average += (v90a_fse_centre(s->eq)-s->centre_average)/256;
        if (++s->tracking_symbols % 256 == 0) {
            double error = s->centre_average-s->centre_reference;
            s->integral = fmax(-0.001, fmin(0.001, s->integral-0.000002*error));
            s->frequency = fmax(-0.002, fmin(0.002, s->integral-0.0002*error));
        }
    }
}

/* Sd is known to repeat every six DS0 symbols (V.90 §8.4.4, inherited
 * by V.92 §8.6). Fit its fundamental and third harmonic jointly with DC;
 * frequency comes from samples, never packet arrival time or peer state. */
static double tone_residual(const int16_t *x, int n, double ratio)
{
    double matrix[5][6] = {{0}}, power = 0;
    const double w = 2*3.14159265358979323846/12*ratio;
    for (int i = 0; i < n; i++) {
        double b[5] = {cos(w*i), sin(w*i), cos(3*w*i), sin(3*w*i), 1};
        power += (double)x[i]*x[i];
        for (int j = 0; j < 5; j++) {
            matrix[j][5] += b[j]*x[i];
            for (int k = 0; k < 5; k++) matrix[j][k] += b[j]*b[k];
        }
    }
    double rhs[5];
    for (int j = 0; j < 5; j++) rhs[j] = matrix[j][5];
    for (int j = 0; j < 5; j++) {
        double pivot = matrix[j][j];
        if (fabs(pivot) < 1e-12) return 1;
        for (int k = j; k < 6; k++) matrix[j][k] /= pivot;
        for (int r = 0; r < 5; r++) if (r != j) {
            double factor = matrix[r][j];
            for (int k = j; k < 6; k++) matrix[r][k] -= factor*matrix[j][k];
        }
    }
    double residual = power;
    for (int j = 0; j < 5; j++) residual -= rhs[j]*matrix[j][5];
    return power > 1 ? fmax(0, residual/power) : 1;
}

static void acquire_clock(v92a_audio_t *s)
{
    const int16_t *x = s->acquisition+SD_WINDOW/2;
    double lo = .998, hi = 1.002;
    for (int i = 0; i < 40; i++) {
        double a = lo+(hi-lo)/3, b = hi-(hi-lo)/3;
        if (tone_residual(x, SD_WINDOW/2, a) < tone_residual(x, SD_WINDOW/2, b)) hi = b;
        else lo = a;
    }
    double ratio = (lo+hi)/2;
    if (tone_residual(x, SD_WINDOW/2, ratio) < .001)
        s->frequency = 1/ratio-1;
}

static void receive_half_symbol(v92a_audio_t *s, int16_t sample)
{
    if (s->acquired) { equalized(s, sample); return; }
    s->acquisition[s->acquisition_count++] = sample;
    if (s->acquisition_count < SD_WINDOW) return;
    double taps[V90A_SD_MAX_TAPS], score, level;
    int parity;
    if (v90a_sd_fit(s->acquisition, SD_WINDOW, 32, &parity, taps, &score, &level)) {
        v90a_fse_set_taps(s->eq, taps, 32, parity);
        double power = 0;
        for (int i = SD_WINDOW/2; i < SD_WINDOW; i++) power += (double)s->acquisition[i]*s->acquisition[i];
        s->input_trn = sqrt(power/(SD_WINDOW/2))*s->trn_level/s->sd_level/sqrt(2.0/3);
        acquire_clock(s);
        s->acquired = true;
        /* V.90 §8.4.4 only promises 384 Sd symbols; replay what the fit
         * buffered so acquisition does not consume the receiver's preamble. */
        for (int i = 0; i < SD_WINDOW; i++) equalized(s, s->acquisition[i]);
    } else {
        memmove(s->acquisition, s->acquisition+SD_SLIDE,
                (SD_WINDOW-SD_SLIDE)*sizeof(s->acquisition[0]));
        s->acquisition_count = SD_WINDOW-SD_SLIDE;
    }
}

void v92a_audio_rx(v92a_audio_t *s, const int16_t *samples, int count)
{
    if (!s || !samples || count <= 0) return;
    for (int i = 0; i < count; i++) {
        history_put(&s->rx, samples[i]);
        while ((int64_t)floor(s->rx_next)+AUDIO_RADIUS < s->rx.count) {
            double value = interpolate(&s->rx, s->rx_next, fmin(1, 16000.0/s->rate));
            receive_half_symbol(s, quantize(value, &s->clipped));
            s->rx_next += s->rate/16000.0*(1+s->frequency);
        }
    }
}

uint64_t v92a_audio_clipped(const v92a_audio_t *s) { return s ? s->clipped : 0; }
bool v92a_audio_acquired(const v92a_audio_t *s) { return s && s->acquired; }
double v92a_audio_clock_ppm(const v92a_audio_t *s) { return s ? s->frequency*1e6 : 0; }
double v92a_audio_eq_error(const v92a_audio_t *s) { return s ? v90a_fse_dispersion(s->eq) : 1; }
