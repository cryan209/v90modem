/*
 * v21_fsk_demod.c — Standalone V.21 FSK demodulator (no spandsp)
 *
 * Implements both block-mode (brute-force phase sweep for offline decode)
 * and streaming (PLL-based timing recovery) V.21 FSK demodulation.
 *
 * V.21 CH1: mark=980 Hz, space=1180 Hz (calling modem)
 * V.21 CH2: mark=1650 Hz, space=1850 Hz (answering modem)
 * Baud rate: 300 symbols/sec, 8000 Hz sample rate → ~26.67 samples/symbol
 */

#include "v21_fsk_demod.h"

#include <math.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* Channel frequency definitions                                       */
/* ------------------------------------------------------------------ */

static void v21_get_freqs(int channel, double *mark_hz, double *space_hz)
{
    if (channel == V21_CH2) {
        *mark_hz = 1650.0;
        *space_hz = 1850.0;
    } else {
        *mark_hz = 980.0;
        *space_hz = 1180.0;
    }
}

/* ------------------------------------------------------------------ */
/* Goertzel tone analysis                                              */
/* ------------------------------------------------------------------ */

goertzel_result_t goertzel_analyze(const int16_t *samples,
                                   int len,
                                   int sample_rate,
                                   double freq_hz)
{
    goertzel_result_t r = {0.0, 0.0, 0.0};
    double w, cos_w, sin_w;
    double osc_re = 1.0, osc_im = 0.0;

    if (!samples || len <= 0 || sample_rate <= 0 || freq_hz <= 0.0)
        return r;

    w = 2.0 * M_PI * freq_hz / (double)sample_rate;
    cos_w = cos(w);
    sin_w = sin(w);

    for (int i = 0; i < len; i++) {
        double s = (double)samples[i];
        double next_re = osc_re * cos_w - osc_im * sin_w;
        double next_im = osc_im * cos_w + osc_re * sin_w;

        r.re += s * osc_re;
        r.im -= s * osc_im;
        osc_re = next_re;
        osc_im = next_im;
    }

    r.energy = r.re * r.re + r.im * r.im;
    return r;
}

/* ------------------------------------------------------------------ */
/* Single-symbol decision                                              */
/* ------------------------------------------------------------------ */

int v21_fsk_symbol_decision(const int16_t *samples,
                            int total_samples,
                            int sample_rate,
                            int channel,
                            int offset,
                            int symbol_len,
                            double *confidence_out)
{
    double mark_hz, space_hz;
    goertzel_result_t mark_r, space_r;
    double total_e;
    double mark_ratio, space_ratio;

    if (!samples || offset < 0 || offset + symbol_len > total_samples || symbol_len <= 0)
        return -1;

    v21_get_freqs(channel, &mark_hz, &space_hz);
    mark_r = goertzel_analyze(samples + offset, symbol_len, sample_rate, mark_hz);
    space_r = goertzel_analyze(samples + offset, symbol_len, sample_rate, space_hz);

    total_e = mark_r.energy + space_r.energy;
    if (total_e <= 0.0)
        return -1;

    mark_ratio = mark_r.energy / total_e;
    space_ratio = space_r.energy / total_e;

    if (confidence_out)
        *confidence_out = fabs(mark_ratio - space_ratio);

    return (mark_ratio >= space_ratio) ? 1 : 0;
}

/* ------------------------------------------------------------------ */
/* Block-mode demodulator with brute-force phase sweep                 */
/* ------------------------------------------------------------------ */

int v21_fsk_demod_block(const int16_t *samples,
                        int total_samples,
                        int sample_rate,
                        int channel,
                        bool invert,
                        v21_bit_cb_t put_bit,
                        void *put_bit_ctx)
{
    /*
     * Sweep baud rates 294-306 and phase offsets to find the alignment
     * that maximizes aggregate confidence. Then deliver bits from the
     * best alignment.
     *
     * This mirrors the approach in v8_targeted_v21_decode in vpcm_decode.c
     * but is generalized as a reusable module.
     */
    enum {
        BAUD_MIN_X10 = 2940,
        BAUD_MAX_X10 = 3060,
        BAUD_STEP_X10 = 20,
        PHASE_STEPS = 8,
        MAX_BITS = 16384
    };
    double mark_hz, space_hz;
    double best_score = -1.0;
    int best_baud_x10 = 3000;
    int best_phase = 0;
    int best_bit_count = 0;
    int best_bits_buf[MAX_BITS];
    int best_offsets[MAX_BITS];

    if (!samples || total_samples <= 0 || sample_rate <= 0 || !put_bit)
        return 0;

    v21_get_freqs(channel, &mark_hz, &space_hz);

    /* Phase 1: sweep baud rate and phase to find best alignment */
    for (int baud_x10 = BAUD_MIN_X10; baud_x10 <= BAUD_MAX_X10; baud_x10 += BAUD_STEP_X10) {
        double sym_samples = (double)sample_rate * 10.0 / (double)baud_x10;
        int sym_len = (int)(sym_samples + 0.5);

        if (sym_len <= 2)
            continue;

        for (int phase = 0; phase < PHASE_STEPS; phase++) {
            int start_offset = (int)(phase * sym_samples / PHASE_STEPS);
            double score = 0.0;
            int bit_count = 0;
            int bits[MAX_BITS];
            int offsets[MAX_BITS];

            for (int pos = start_offset; pos + sym_len <= total_samples && bit_count < MAX_BITS; pos += sym_len) {
                goertzel_result_t mr = goertzel_analyze(samples + pos, sym_len, sample_rate, mark_hz);
                goertzel_result_t sr = goertzel_analyze(samples + pos, sym_len, sample_rate, space_hz);
                double total_e = mr.energy + sr.energy;
                double mark_r, space_r, conf;
                int bit;

                if (total_e <= 0.0) {
                    bits[bit_count] = 1; /* idle = mark */
                    offsets[bit_count] = pos;
                    bit_count++;
                    continue;
                }

                mark_r = mr.energy / total_e;
                space_r = sr.energy / total_e;
                conf = fabs(mark_r - space_r);
                bit = (mark_r >= space_r) ? 1 : 0;
                if (invert)
                    bit = !bit;

                bits[bit_count] = bit;
                offsets[bit_count] = pos;
                bit_count++;
                score += conf;
            }

            if (bit_count > 0 && score > best_score) {
                best_score = score;
                best_baud_x10 = baud_x10;
                best_phase = phase;
                best_bit_count = bit_count;
                if (bit_count <= MAX_BITS) {
                    memcpy(best_bits_buf, bits, (size_t)bit_count * sizeof(int));
                    memcpy(best_offsets, offsets, (size_t)bit_count * sizeof(int));
                }
            }
        }
    }

    (void)best_baud_x10;
    (void)best_phase;

    /* Phase 2: deliver best-alignment bits */
    for (int i = 0; i < best_bit_count; i++)
        put_bit(put_bit_ctx, best_bits_buf[i], best_offsets[i]);

    return best_bit_count;
}

/* ------------------------------------------------------------------ */
/* Streaming demodulator with simple PLL timing recovery               */
/* ------------------------------------------------------------------ */

void v21_fsk_stream_init(v21_fsk_stream_t *d,
                          int channel,
                          int sample_rate,
                          bool invert,
                          v21_bit_cb_t put_bit,
                          void *put_bit_ctx)
{
    double w;

    if (!d)
        return;

    memset(d, 0, sizeof(*d));
    d->channel = channel;
    d->sample_rate = sample_rate;
    d->invert = invert;
    d->put_bit = put_bit;
    d->put_bit_ctx = put_bit_ctx;

    v21_get_freqs(channel, &d->mark_hz, &d->space_hz);

    d->symbol_samples = (double)sample_rate / 300.0;
    d->phase_accum = 0.0;
    d->window_count = 0;
    d->window_energy = 0.0;
    d->total_samples_fed = 0;

    /* Precompute twiddle factors */
    w = 2.0 * M_PI * d->mark_hz / (double)sample_rate;
    d->mark_cos = cos(w);
    d->mark_sin = sin(w);
    d->mark_osc_re = 1.0;
    d->mark_osc_im = 0.0;

    w = 2.0 * M_PI * d->space_hz / (double)sample_rate;
    d->space_cos = cos(w);
    d->space_sin = sin(w);
    d->space_osc_re = 1.0;
    d->space_osc_im = 0.0;

    d->mark_re = 0.0;
    d->mark_im = 0.0;
    d->space_re = 0.0;
    d->space_im = 0.0;
    d->prev_decision_val = 0.0;
}

void v21_fsk_stream_reset(v21_fsk_stream_t *d)
{
    if (!d)
        return;
    d->mark_re = 0.0;
    d->mark_im = 0.0;
    d->space_re = 0.0;
    d->space_im = 0.0;
    d->mark_osc_re = 1.0;
    d->mark_osc_im = 0.0;
    d->space_osc_re = 1.0;
    d->space_osc_im = 0.0;
    d->window_count = 0;
    d->window_energy = 0.0;
    d->phase_accum = 0.0;
    d->prev_decision_val = 0.0;
}

static void v21_fsk_stream_emit_symbol(v21_fsk_stream_t *d)
{
    double mark_e, space_e, decision_val;
    int bit;

    if (!d)
        return;

    mark_e = d->mark_re * d->mark_re + d->mark_im * d->mark_im;
    space_e = d->space_re * d->space_re + d->space_im * d->space_im;
    decision_val = mark_e - space_e;
    bit = (decision_val >= 0.0) ? 1 : 0;
    if (d->invert)
        bit = !bit;

    /* Simple early/late PLL nudge based on decision value transition */
    if (d->prev_decision_val != 0.0) {
        double transition = fabs(decision_val) - fabs(d->prev_decision_val);

        /* If the decision got weaker, we're drifting — nudge timing */
        if (transition < -0.1 * fabs(d->prev_decision_val))
            d->phase_accum -= 0.3;
        else if (transition > 0.1 * fabs(d->prev_decision_val))
            d->phase_accum += 0.15;
    }
    d->prev_decision_val = decision_val;

    if (d->put_bit)
        d->put_bit(d->put_bit_ctx, bit, d->total_samples_fed - d->window_count);

    /* Reset accumulators for next symbol */
    d->mark_re = 0.0;
    d->mark_im = 0.0;
    d->space_re = 0.0;
    d->space_im = 0.0;
    d->mark_osc_re = 1.0;
    d->mark_osc_im = 0.0;
    d->space_osc_re = 1.0;
    d->space_osc_im = 0.0;
    d->window_count = 0;
    d->window_energy = 0.0;
}

void v21_fsk_stream_rx(v21_fsk_stream_t *d,
                        const int16_t *samples,
                        int count)
{
    if (!d || !samples || count <= 0)
        return;

    for (int i = 0; i < count; i++) {
        double s = (double)samples[i];
        double next_re, next_im;

        /* Accumulate mark Goertzel */
        next_re = d->mark_osc_re * d->mark_cos - d->mark_osc_im * d->mark_sin;
        next_im = d->mark_osc_im * d->mark_cos + d->mark_osc_re * d->mark_sin;
        d->mark_re += s * d->mark_osc_re;
        d->mark_im -= s * d->mark_osc_im;
        d->mark_osc_re = next_re;
        d->mark_osc_im = next_im;

        /* Accumulate space Goertzel */
        next_re = d->space_osc_re * d->space_cos - d->space_osc_im * d->space_sin;
        next_im = d->space_osc_im * d->space_cos + d->space_osc_re * d->space_sin;
        d->space_re += s * d->space_osc_re;
        d->space_im -= s * d->space_osc_im;
        d->space_osc_re = next_re;
        d->space_osc_im = next_im;

        d->window_energy += s * s;
        d->window_count++;
        d->total_samples_fed++;
        d->phase_accum += 1.0;

        /* Emit symbol when we've accumulated enough samples */
        if (d->phase_accum >= d->symbol_samples) {
            v21_fsk_stream_emit_symbol(d);
            d->phase_accum -= d->symbol_samples;
        }
    }
}
