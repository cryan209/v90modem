/*
 * p3_demod.c — Lightweight Phase 3 demodulator for V.34/V.90/V.92
 *
 * Carrier recovery, symbol timing, differential PSK demodulation,
 * descrambling, and pattern detection for offline modem analysis.
 */

#include "p3_demod.h"

#include <float.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Use SpanDSP's generated, validated 192-phase V.34 receive filters. Keep
 * the explicit path: same-named files at the repository root are stubs. */
#include "spandsp-master/src/v34_rx_2400_low_carrier_rrc.h"
#include "spandsp-master/src/v34_rx_2400_high_carrier_rrc.h"
#include "spandsp-master/src/v34_rx_2743_low_carrier_rrc.h"
#include "spandsp-master/src/v34_rx_2743_high_carrier_rrc.h"
#include "spandsp-master/src/v34_rx_2800_low_carrier_rrc.h"
#include "spandsp-master/src/v34_rx_2800_high_carrier_rrc.h"
#include "spandsp-master/src/v34_rx_3000_low_carrier_rrc.h"
#include "spandsp-master/src/v34_rx_3000_high_carrier_rrc.h"
#include "spandsp-master/src/v34_rx_3200_low_carrier_rrc.h"
#include "spandsp-master/src/v34_rx_3200_high_carrier_rrc.h"
#include "spandsp-master/src/v34_rx_3429_rrc.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ------------------------------------------------------------------ */
/*  Baud rate table (from V.34 Table 1)                               */
/*                                                                    */
/*  Carrier frequency = baud_rate * d / e                             */
/*    low_high[0] = {d, e} for low carrier                            */
/*    low_high[1] = {d, e} for high carrier                           */
/* ------------------------------------------------------------------ */

typedef struct {
    int baud_rate;
    int samples_num;
    int samples_den;
    int low_d, low_e;
    int high_d, high_e;
} baud_entry_t;

static const baud_entry_t baud_table[P3_BAUD_COUNT] = {
    /* baud   num den  low_d low_e  high_d high_e */
    { 2400,   10,  3,   2,    3,     3,     4  },   /* 2400: low=1600, high=1800 */
    { 2743,   35, 12,   3,    5,     2,     3  },   /* 2743: low=1646, high=1829 */
    { 2800,   20,  7,   3,    5,     2,     3  },   /* 2800: low=1680, high=1867 */
    { 3000,    8,  3,   3,    5,     2,     3  },   /* 3000: low=1800, high=2000 */
    { 3200,    5,  2,   4,    7,     3,     5  },   /* 3200: low=1829, high=1920 */
    { 3429,    7,  3,   4,    7,     4,     7  },   /* 3429: low=1959, high=1959 */
};

#define P3_RRC_PHASES 192
#define P3_RRC_TAPS   27

static const float (*const p3_rrc_re[P3_BAUD_COUNT][2])[P3_RRC_TAPS] = {
    {rx_pulseshaper_2400_low_carrier_re, rx_pulseshaper_2400_high_carrier_re},
    {rx_pulseshaper_2743_low_carrier_re, rx_pulseshaper_2743_high_carrier_re},
    {rx_pulseshaper_2800_low_carrier_re, rx_pulseshaper_2800_high_carrier_re},
    {rx_pulseshaper_3000_low_carrier_re, rx_pulseshaper_3000_high_carrier_re},
    {rx_pulseshaper_3200_low_carrier_re, rx_pulseshaper_3200_high_carrier_re},
    {rx_pulseshaper_3429_re, rx_pulseshaper_3429_re}
};

static const float (*const p3_rrc_im[P3_BAUD_COUNT][2])[P3_RRC_TAPS] = {
    {rx_pulseshaper_2400_low_carrier_im, rx_pulseshaper_2400_high_carrier_im},
    {rx_pulseshaper_2743_low_carrier_im, rx_pulseshaper_2743_high_carrier_im},
    {rx_pulseshaper_2800_low_carrier_im, rx_pulseshaper_2800_high_carrier_im},
    {rx_pulseshaper_3000_low_carrier_im, rx_pulseshaper_3000_high_carrier_im},
    {rx_pulseshaper_3200_low_carrier_im, rx_pulseshaper_3200_high_carrier_im},
    {rx_pulseshaper_3429_im, rx_pulseshaper_3429_im}
};

/* Literal steps_per_baud[] values from SpanDSP primary_channel_rx(). */
static const int p3_rrc_steps_per_baud[P3_BAUD_COUNT] = {
    640, 560, 540, 512, 480, 448
};

static void create_godard_coeffs(p3_demod_t *d, float alpha)
{
    float low_edge = (float)(2.0 * M_PI)
                   * (d->carrier_hz - d->baud_rate / 2.0f) / (float)d->sample_rate;
    float high_edge = (float)(2.0 * M_PI)
                    * (d->carrier_hz + d->baud_rate / 2.0f) / (float)d->sample_rate;

    d->ted_low_coeff[0] = 2.0f * alpha * cosf(low_edge);
    d->ted_high_coeff[0] = 2.0f * alpha * cosf(high_edge);
    d->ted_low_coeff[1] = d->ted_high_coeff[1] = -alpha * alpha;
    d->ted_low_coeff[2] = -alpha * sinf(low_edge);
    d->ted_high_coeff[2] = -alpha * sinf(high_edge);
    d->ted_mixed_coeff = -alpha * alpha
                       * (sinf(high_edge) * cosf(low_edge)
                          - sinf(low_edge) * cosf(high_edge));
}

/* ------------------------------------------------------------------ */
/*  NCO helpers                                                       */
/* ------------------------------------------------------------------ */

/* Phase accumulator uses full 32-bit range = 2*pi */
#define NCO_SCALE  4294967296.0

static uint32_t hz_to_phase_inc(float hz, int sample_rate)
{
    return (uint32_t)(hz * NCO_SCALE / (float)sample_rate + 0.5f);
}

/* Fast NCO using 1024-entry lookup table */
#define NCO_TABLE_SIZE 1024
#define NCO_TABLE_SHIFT 22  /* 32 - 10 = top 10 bits index the table */

static float nco_cos_table[NCO_TABLE_SIZE];
static float nco_sin_table[NCO_TABLE_SIZE];
static bool nco_table_initialized = false;

static void nco_init_tables(void)
{
    if (nco_table_initialized)
        return;
    for (int i = 0; i < NCO_TABLE_SIZE; i++) {
        float angle = (float)i * (float)(2.0 * M_PI / NCO_TABLE_SIZE);
        nco_cos_table[i] = cosf(angle);
        nco_sin_table[i] = sinf(angle);
    }
    nco_table_initialized = true;
}

static void nco_get(uint32_t phase, float *cos_out, float *sin_out)
{
    unsigned idx = phase >> NCO_TABLE_SHIFT;
    *cos_out = nco_cos_table[idx];
    *sin_out = nco_sin_table[idx];
}

/* ------------------------------------------------------------------ */
/*  Public: baud parameters                                           */
/* ------------------------------------------------------------------ */

bool p3_get_baud_params(int baud_code, p3_baud_params_t *out)
{
    const baud_entry_t *e;

    if (baud_code < 0 || baud_code >= P3_BAUD_COUNT || !out)
        return false;
    e = &baud_table[baud_code];
    out->baud_rate = e->baud_rate;
    out->samples_num = e->samples_num;
    out->samples_den = e->samples_den;
    out->carrier_low_hz = (float)e->baud_rate * (float)e->low_d / (float)e->low_e;
    out->carrier_high_hz = (float)e->baud_rate * (float)e->high_d / (float)e->high_e;
    return true;
}

/* ------------------------------------------------------------------ */
/*  Demodulator init / reset                                          */
/* ------------------------------------------------------------------ */

void p3_demod_init(p3_demod_t *d, int baud_code, int carrier_sel, int sample_rate)
{
    p3_baud_params_t bp;

    if (!d)
        return;
    memset(d, 0, sizeof(*d));
    nco_init_tables();

    if (!p3_get_baud_params(baud_code, &bp))
        return;

    d->baud_code = baud_code;
    d->carrier_sel = carrier_sel;
    d->sample_rate = sample_rate;
    d->baud_rate = (float)bp.baud_rate;
    d->carrier_hz = (carrier_sel == P3_CARRIER_HIGH) ? bp.carrier_high_hz : bp.carrier_low_hz;
    d->samples_per_symbol = (float)bp.samples_num / (float)bp.samples_den;

    /* NCO */
    d->nco_phase = 0;
    d->nco_phase_inc = hz_to_phase_inc(d->carrier_hz, sample_rate);

    /* PLL — moderately aggressive for offline use */
    d->pll_alpha = 0.008f;
    d->pll_beta = 0.00005f;
    d->pll_freq_err = 0.0f;

    /* Symbol timing */
    d->baud_phase = 0.0f;
    d->ted_alpha = 0.02f;

    /* Matched-filter state */
    memset(d->mf_hist_re, 0, sizeof(d->mf_hist_re));
    memset(d->mf_hist_im, 0, sizeof(d->mf_hist_im));
    d->mf_hist_pos = 0;
    d->mf_prev_re = 0.0f;
    d->mf_prev_im = 0.0f;
    d->mf_prev_valid = false;

    /* RRC T/2 resampler and Godard timing loop */
    d->rrc_re = p3_rrc_re[baud_code][carrier_sel ? 1 : 0];
    d->rrc_im = p3_rrc_im[baud_code][carrier_sel ? 1 : 0];
    d->rrc_steps_per_baud = p3_rrc_steps_per_baud[baud_code];
    d->rrc_step = 0;
    d->rrc_hist_pos = 0;
    d->rrc_half_baud = 0;
    create_godard_coeffs(d, 0.99f);
    d->use_rrc_frontend = getenv("P3_LEGACY_FRONTEND") == NULL;
    d->rrc_agc_gain = 1.0f / 8000.0f;
    d->eq_coeff_re[63] = 1.0f;
    d->eq_delta = 0.21f / 127.0f;
    d->s_previous_dibit = -1;

    /* AGC */
    d->agc_gain = 1.0f / 8000.0f;  /* Initial conservative gain */
    d->agc_target = 1.0f;

    /* Differential decode */
    d->prev_re = 0.0f;
    d->prev_im = 0.0f;
    d->prev_valid = false;

    /* Descrambler */
    d->descrambler_sr = 0;

    /* Statistics */
    d->total_symbols = 0;
    d->magnitude_sum = 0.0f;
    d->magnitude_count = 0;
}

void p3_demod_reset(p3_demod_t *d)
{
    if (!d)
        return;
    d->nco_phase = 0;
    d->pll_freq_err = 0.0f;
    d->baud_phase = 0.0f;
    d->idum_re = 0.0f;
    d->idum_im = 0.0f;
    d->idum_count = 0;
    memset(d->mf_hist_re, 0, sizeof(d->mf_hist_re));
    memset(d->mf_hist_im, 0, sizeof(d->mf_hist_im));
    d->mf_hist_pos = 0;
    d->mf_prev_re = 0.0f;
    d->mf_prev_im = 0.0f;
    d->mf_prev_valid = false;
    memset(d->rrc_hist, 0, sizeof(d->rrc_hist));
    d->rrc_hist_pos = 0;
    d->rrc_step = 0;
    d->rrc_half_baud = 0;
    memset(d->ted_low, 0, sizeof(d->ted_low));
    memset(d->ted_high, 0, sizeof(d->ted_high));
    memset(d->ted_dc, 0, sizeof(d->ted_dc));
    d->ted_phase = 0.0f;
    d->timing_correction = 0;
    d->rrc_agc_gain = 1.0f / 8000.0f;
    memset(d->eq_buf_re, 0, sizeof(d->eq_buf_re));
    memset(d->eq_buf_im, 0, sizeof(d->eq_buf_im));
    memset(d->eq_coeff_re, 0, sizeof(d->eq_coeff_re));
    memset(d->eq_coeff_im, 0, sizeof(d->eq_coeff_im));
    d->eq_coeff_re[63] = 1.0f;
    d->eq_buf_pos = 0;
    d->eq_delta = 0.21f / 127.0f;
    d->cma_freeze_symbols = 0;
    d->s_alternating_run = 0;
    d->s_previous_dibit = -1;
    d->prev_re = 0.0f;
    d->prev_im = 0.0f;
    d->prev_valid = false;
    d->descrambler_sr = 0;
    d->total_symbols = 0;
    d->magnitude_sum = 0.0f;
    d->magnitude_count = 0;
}

/* ------------------------------------------------------------------ */
/*  Result allocation                                                 */
/* ------------------------------------------------------------------ */

p3_result_t *p3_result_alloc(int max_symbols, int max_segments)
{
    p3_result_t *r;

    r = calloc(1, sizeof(*r));
    if (!r)
        return NULL;
    if (max_symbols > 0) {
        r->symbols = calloc((size_t)max_symbols, sizeof(p3_symbol_t));
        if (!r->symbols) {
            free(r);
            return NULL;
        }
        r->symbol_capacity = max_symbols;
    }
    if (max_segments > 0) {
        r->segments = calloc((size_t)max_segments, sizeof(p3_segment_t));
        if (!r->segments) {
            free(r->symbols);
            free(r);
            return NULL;
        }
        r->segment_capacity = max_segments;
    }
    return r;
}

void p3_result_free(p3_result_t *r)
{
    if (!r)
        return;
    free(r->symbols);
    free(r->segments);
    free(r);
}

/* ------------------------------------------------------------------ */
/*  Differential decode: phase difference -> dibit                    */
/* ------------------------------------------------------------------ */

static int phase_to_dibit(float re, float im, float prev_re, float prev_im)
{
    /* Compute conjugate product: z * conj(prev) */
    float dr = re * prev_re + im * prev_im;
    float di = im * prev_re - re * prev_im;
    float angle;

    /* Map angle to quadrant (0-3) */
    angle = atan2f(di, dr);
    if (angle < 0.0f)
        angle += (float)(2.0 * M_PI);

    /* Quadrant boundaries at 45, 135, 225, 315 degrees */
    if (angle < (float)(M_PI / 4.0))
        return 0;
    if (angle < (float)(3.0 * M_PI / 4.0))
        return 1;
    if (angle < (float)(5.0 * M_PI / 4.0))
        return 2;
    if (angle < (float)(7.0 * M_PI / 4.0))
        return 3;
    return 0;
}

/* ------------------------------------------------------------------ */
/*  Descrambler (x^23 + x^5 + 1)                                     */
/*                                                                    */
/*  For descrambling: out = in ^ sr[22] ^ sr[4]                       */
/*  Then shift in the received (scrambled) bit.                       */
/* ------------------------------------------------------------------ */

static int descramble_bit(uint32_t *sr, int in_bit)
{
    int fb = (int)((*sr >> 22) ^ (*sr >> 4)) & 1;
    int out = in_bit ^ fb;
    *sr = ((*sr << 1) | (uint32_t)(in_bit & 1)) & 0x7FFFFF;
    return out;
}

int p3_descramble_bits(const uint8_t *scrambled, int count,
                       uint8_t *descrambled, uint32_t *sr)
{
    int errors = 0;
    uint32_t local_sr;

    if (!scrambled || count <= 0 || !sr)
        return -1;
    local_sr = *sr;
    for (int i = 0; i < count; i++) {
        int out = descramble_bit(&local_sr, scrambled[i] & 1);
        if (descrambled)
            descrambled[i] = (uint8_t)out;
        if (out != 1)
            errors++;
    }
    *sr = local_sr;
    return errors;
}

/* ------------------------------------------------------------------ */
/*  Core demodulation loop                                            */
/* ------------------------------------------------------------------ */

static void emit_symbol(p3_demod_t *d, float bb_re, float bb_im,
                        int sample_idx, bool pre_scaled, p3_result_t *result)
{
    p3_symbol_t *sym;
    float mag;
    float scaled_re, scaled_im;
    int dibit;
    int bit0, bit1;

    if (!result || result->symbol_count >= result->symbol_capacity)
        return;

    /* AGC. The RRC path normalizes before its adaptive equalizer. */
    if (pre_scaled) {
        scaled_re = bb_re;
        scaled_im = bb_im;
    } else {
        mag = sqrtf(bb_re * bb_re + bb_im * bb_im);
        if (mag > 1e-10f) {
            float err = d->agc_target - mag * d->agc_gain;
            d->agc_gain += 0.005f * err;
            if (d->agc_gain < 1e-10f)
                d->agc_gain = 1e-10f;
        }
        scaled_re = bb_re * d->agc_gain;
        scaled_im = bb_im * d->agc_gain;
    }
    mag = sqrtf(scaled_re * scaled_re + scaled_im * scaled_im);

    /* Differential decode */
    if (d->prev_valid) {
        dibit = phase_to_dibit(scaled_re, scaled_im, d->prev_re, d->prev_im);

        /* Carrier PLL: decision-directed phase error */
        /* For 4-point: snap to nearest 90-degree, compute residual */
        {
            float conj_re = scaled_re * d->prev_re + scaled_im * d->prev_im;
            float conj_im = scaled_im * d->prev_re - scaled_re * d->prev_im;
            float ref_angle = (float)dibit * (float)(M_PI / 2.0);
            float ref_re = cosf(ref_angle);
            float ref_im = sinf(ref_angle);
            /* Phase error = angle between measured and reference */
            float err = conj_im * ref_re - conj_re * ref_im;
            float conj_mag = sqrtf(conj_re * conj_re + conj_im * conj_im);
            if (conj_mag > 1e-10f)
                err /= conj_mag;

            d->pll_freq_err += d->pll_beta * err;
            d->nco_phase_inc = hz_to_phase_inc(d->carrier_hz + d->pll_freq_err,
                                                d->sample_rate);
        }
    } else {
        dibit = 0;
    }

    /* Descramble: dibit -> 2 bits -> descrambler */
    bit0 = descramble_bit(&d->descrambler_sr, dibit & 1);
    bit1 = descramble_bit(&d->descrambler_sr, (dibit >> 1) & 1);

    /* S is an exact 1/3 differential-dibit alternation for 128T. Once a
     * strong run is present, freeze blind CMA through S-bar and the first
     * 512T of TRN so a 16-point constellation keeps its amplitude levels. */
    if ((dibit == P3_DIBIT_1 || dibit == P3_DIBIT_3)
        && (d->s_previous_dibit == P3_DIBIT_1
            || d->s_previous_dibit == P3_DIBIT_3)
        && dibit != d->s_previous_dibit) {
        d->s_alternating_run++;
    } else {
        d->s_alternating_run = 0;
    }
    d->s_previous_dibit = dibit;
    if (d->s_alternating_run >= 96)
        d->cma_freeze_symbols = 16 + 512 + 64;

    /* Store symbol */
    sym = &result->symbols[result->symbol_count++];
    sym->sample_index = sample_idx;
    sym->re = scaled_re;
    sym->im = scaled_im;
    sym->magnitude = mag;
    sym->phase = atan2f(scaled_im, scaled_re);
    sym->quadrant = phase_to_dibit(scaled_re, scaled_im, 1.0f, 0.0f);
    sym->dibit = dibit;
    sym->bit0 = bit0;
    sym->bit1 = bit1;

    /* Update previous symbol for next differential decode */
    d->prev_re = scaled_re;
    d->prev_im = scaled_im;
    d->prev_valid = true;

    /* Statistics */
    d->total_symbols++;
    d->magnitude_sum += mag;
    d->magnitude_count++;
}

static float rrc_circular_dot(const float hist[P3_RRC_TAPS],
                              int pos,
                              const float coeff[P3_RRC_TAPS])
{
    float sum = 0.0f;

    for (int i = 0; i < P3_RRC_TAPS; i++) {
        int idx = pos + i;
        if (idx >= P3_RRC_TAPS)
            idx -= P3_RRC_TAPS;
        sum += hist[idx] * coeff[i];
    }
    return sum;
}

static void godard_filter_update(p3_demod_t *d, float sample_re)
{
    float v;

    v = d->ted_low[0] * d->ted_low_coeff[0]
      + d->ted_low[1] * d->ted_low_coeff[1] + sample_re;
    d->ted_low[1] = d->ted_low[0];
    d->ted_low[0] = v;

    v = d->ted_high[0] * d->ted_high_coeff[0]
      + d->ted_high[1] * d->ted_high_coeff[1] + sample_re;
    d->ted_high[1] = d->ted_high[0];
    d->ted_high[0] = v;
}

static void godard_symbol_sync(p3_demod_t *d)
{
    const float error_clip = 50.0f;
    const float fine_trigger = 100.0f;
    const float coarse_trigger = 200.0f;
    float v;
    float p;

    v = d->ted_low[1] * d->ted_high[0] * d->ted_low_coeff[2]
      - d->ted_low[0] * d->ted_high[1] * d->ted_high_coeff[2]
      + d->ted_low[1] * d->ted_high[1] * d->ted_mixed_coeff;
    p = v - d->ted_dc[1];
    if (!isfinite(p))
        p = 0.0f;
    else if (p > error_clip)
        p = error_clip;
    else if (p < -error_clip)
        p = -error_clip;
    d->ted_dc[1] = d->ted_dc[0];
    d->ted_dc[0] = v;
    d->ted_phase -= p;
    if (!isfinite(d->ted_phase))
        d->ted_phase = 0.0f;
    else if (d->ted_phase > 500.0f)
        d->ted_phase = 500.0f;
    else if (d->ted_phase < -500.0f)
        d->ted_phase = -500.0f;

    v = fabsf(d->ted_phase);
    if (v > fine_trigger) {
        int correction = (v > coarse_trigger) ? 2 : 1;
        if (d->ted_phase < 0.0f)
            correction = -correction;
        d->rrc_step += correction;
        d->timing_correction += correction;
        d->ted_phase -= (float)correction * fine_trigger;
    }
}

static void equalizer_push(p3_demod_t *d, float re, float im)
{
    d->eq_buf_re[d->eq_buf_pos] = re;
    d->eq_buf_im[d->eq_buf_pos] = im;
    d->eq_buf_pos = (d->eq_buf_pos + 1) & 127;
}

static void equalizer_get(const p3_demod_t *d, float *re_out, float *im_out)
{
    float re = 0.0f;
    float im = 0.0f;
    int p = d->eq_buf_pos - 1;

    for (int i = 0; i < 127; i++) {
        float xr;
        float xi;
        float cr = d->eq_coeff_re[i];
        float ci = d->eq_coeff_im[i];

        p = (p - 1) & 127;
        xr = d->eq_buf_re[p];
        xi = d->eq_buf_im[p];
        re += cr * xr - ci * xi;
        im += cr * xi + ci * xr;
    }
    *re_out = re;
    *im_out = im;
}

static void equalizer_tune_cma(p3_demod_t *d, float out_re, float out_im)
{
    float mag2 = out_re * out_re + out_im * out_im;
    float error;
    float norm;
    float grad_re;
    float grad_im;
    int p;

    if (!isfinite(mag2) || mag2 < 0.001f || mag2 > 100.0f)
        return;
    error = 1.0f - mag2;
    if (error < -4.0f)
        error = -4.0f;
    else if (error > 4.0f)
        error = 4.0f;
    norm = (mag2 > 0.001f) ? mag2 : 0.001f;
    grad_re = 0.1f * d->eq_delta * error * out_re / norm;
    grad_im = 0.1f * d->eq_delta * error * out_im / norm;

    p = d->eq_buf_pos - 1;
    for (int i = 0; i < 127; i++) {
        float xr;
        float xi;

        p = (p - 1) & 127;
        xr = d->eq_buf_re[p];
        xi = d->eq_buf_im[p];
        d->eq_coeff_re[i] += grad_re * xr + grad_im * xi;
        d->eq_coeff_im[i] += grad_im * xr - grad_re * xi;
    }
}

static int p3_rrc_demod_process(p3_demod_t *d,
                                const int16_t *samples,
                                int sample_count,
                                int sample_offset,
                                p3_result_t *result)
{
    int start_count;

    if (!d || !samples || sample_count <= 0 || !result || !d->rrc_re || !d->rrc_im)
        return 0;

    start_count = result->symbol_count;
    for (int i = 0; i < sample_count; i++) {
        float ii;
        int phase;

        d->rrc_hist[d->rrc_hist_pos] = (float)samples[i];
        if (++d->rrc_hist_pos >= P3_RRC_TAPS)
            d->rrc_hist_pos = 0;

        d->rrc_step -= P3_RRC_PHASES;
        phase = -d->rrc_step;
        if (phase >= P3_RRC_PHASES)
            phase = P3_RRC_PHASES - 1;
        while (phase < 0)
            phase += P3_RRC_PHASES;

        ii = rrc_circular_dot(d->rrc_hist, d->rrc_hist_pos, d->rrc_re[phase]);
        godard_filter_update(d, ii * d->rrc_agc_gain);

        if (d->rrc_step <= 0) {
            float qq;
            float cos_val;
            float sin_val;
            float sym_re;
            float sym_im;
            float raw_mag;
            float eq_re;
            float eq_im;

            d->rrc_step += d->rrc_steps_per_baud / 2;
            qq = rrc_circular_dot(d->rrc_hist, d->rrc_hist_pos, d->rrc_im[phase]);
            nco_get(d->nco_phase, &cos_val, &sin_val);
            sym_re = ii * cos_val - qq * sin_val;
            sym_im = -ii * sin_val - qq * cos_val;
            raw_mag = hypotf(sym_re, sym_im);
            if (raw_mag > 1.0f && isfinite(raw_mag)) {
                float target_gain = 1.0f / raw_mag;
                d->rrc_agc_gain = 0.995f * d->rrc_agc_gain + 0.005f * target_gain;
            }
            sym_re *= d->rrc_agc_gain;
            sym_im *= d->rrc_agc_gain;
            equalizer_push(d, sym_re, sym_im);

            if (d->rrc_half_baud) {
                d->rrc_half_baud = 0;
                godard_symbol_sync(d);
                equalizer_get(d, &eq_re, &eq_im);
                if (d->cma_freeze_symbols > 0)
                    d->cma_freeze_symbols--;
                else
                    equalizer_tune_cma(d, eq_re, eq_im);
                emit_symbol(d,
                            eq_re,
                            eq_im,
                            sample_offset + i - (int)(32.0f * d->samples_per_symbol + 0.5f),
                            true,
                            result);
            } else {
                d->rrc_half_baud = 1;
            }
        }
        d->nco_phase += d->nco_phase_inc;
    }

    return result->symbol_count - start_count;
}

static int p3_legacy_demod_process(p3_demod_t *d,
                                   const int16_t *samples,
                                   int sample_count,
                                   int sample_offset,
                                   p3_result_t *result)
{
    static const float mf_taps[5] = {0.40f, 0.25f, 0.18f, 0.11f, 0.06f};
    int start_count = result->symbol_count;
    float spb = d->samples_per_symbol;

    for (int i = 0; i < sample_count; i++) {
        float sample = (float)samples[i];
        float cos_val;
        float sin_val;
        float bb_re;
        float bb_im;
        float filt_re = 0.0f;
        float filt_im = 0.0f;
        float phase_prev;

        nco_get(d->nco_phase, &cos_val, &sin_val);
        d->nco_phase += d->nco_phase_inc;
        bb_re = sample * cos_val;
        bb_im = sample * (-sin_val);

        d->mf_hist_re[d->mf_hist_pos] = bb_re;
        d->mf_hist_im[d->mf_hist_pos] = bb_im;
        d->mf_hist_pos = (d->mf_hist_pos + 1) % 5;
        for (int k = 0; k < 5; k++) {
            int idx = d->mf_hist_pos - 1 - k;
            if (idx < 0)
                idx += 5;
            filt_re += mf_taps[k] * d->mf_hist_re[idx];
            filt_im += mf_taps[k] * d->mf_hist_im[idx];
        }

        phase_prev = d->baud_phase;
        d->baud_phase += 1.0f;
        if (d->baud_phase >= spb) {
            float frac = spb - phase_prev;
            float sym_re = filt_re;
            float sym_im = filt_im;

            if (frac < 0.0f)
                frac = 0.0f;
            if (frac > 1.0f)
                frac = 1.0f;
            if (d->mf_prev_valid) {
                sym_re = d->mf_prev_re * (1.0f - frac) + filt_re * frac;
                sym_im = d->mf_prev_im * (1.0f - frac) + filt_im * frac;
            }
            emit_symbol(d, sym_re, sym_im, sample_offset + i, false, result);
            d->baud_phase -= spb;
        }
        d->mf_prev_re = filt_re;
        d->mf_prev_im = filt_im;
        d->mf_prev_valid = true;
    }
    return result->symbol_count - start_count;
}

int p3_demod_process(p3_demod_t *d,
                     const int16_t *samples,
                     int sample_count,
                     int sample_offset,
                     p3_result_t *result)
{
    if (!d || !samples || sample_count <= 0 || !result)
        return 0;
    if (d->use_rrc_frontend)
        return p3_rrc_demod_process(d, samples, sample_count, sample_offset, result);
    return p3_legacy_demod_process(d, samples, sample_count, sample_offset, result);
}

/* ------------------------------------------------------------------ */
/*  Signal segmentation                                               */
/* ------------------------------------------------------------------ */

/* Check if a run of dibits matches 6-symbol S pattern.
 * V.34 Phase 3 S pattern (from Table A.3):
 *   S = {A, B, A, B*, A*, B*} repeating
 * In differential encoding this appears as a fixed 6-dibit sequence.
 * We detect any consistent 6-symbol repeating pattern as S-like. */
static bool is_s_pattern(const p3_symbol_t *syms, int start, int len,
                         bool *is_complement)
{
    int matches = 0;
    int checks = 0;
    int period = 6;
    int dibit_mask = 0;
    int unique_dibits = 0;

    if (len < 12)
        return false;

    /* Reject trivial 2-point repeats; S/S-bar should exercise multiple dibits. */
    for (int i = 0; i < period; i++) {
        int d = syms[start + i].dibit & 3;
        dibit_mask |= (1 << d);
    }
    for (int d = 0; d < 4; d++) {
        if (dibit_mask & (1 << d))
            unique_dibits++;
    }
    if (unique_dibits < 3)
        return false;

    /* Check consistency of 6-symbol repeat */
    for (int i = period; i < len; i++) {
        checks++;
        if (syms[start + i].dibit == syms[start + i % period].dibit)
            matches++;
    }
    if (checks <= 0)
        return false;
    if (matches * 5 < checks * 4)  /* Need >=80% periodic match */
        return false;

    /* Check if dibits have the S-bar complement relationship.
     * S-bar has the opposite sign pattern: dibit XOR 2 */
    if (is_complement) {
        /* Default: check if first half-period has inverted dibits */
        int inv_count = 0;
        for (int i = 0; i < period / 2 && i + period / 2 < len; i++) {
            int d1 = syms[start + i].dibit;
            int d2 = syms[start + i + period / 2].dibit;
            if ((d1 ^ d2) == 2)
                inv_count++;
        }
        *is_complement = (inv_count >= period / 2 - 1);
    }
    return true;
}

/* PP reference from V.34 §10.1.3.6.
 * One PP block is a 48-symbol sequence repeated six times (288T). */
#define P3_PP_PERIOD_SYMBOLS 48

static void pp_reference_symbol(int idx, float *re_out, float *im_out)
{
    int i = idx % P3_PP_PERIOD_SYMBOLS;
    int k = i / 4;
    int I = i & 3;
    int num = ((k % 3) == 1) ? (k * I + 4) : (k * I);
    float angle = (float)M_PI * (float)num / 6.0f;

    if (re_out)
        *re_out = cosf(angle);
    if (im_out)
        *im_out = sinf(angle);
}

/* Coherent correlation against one 48-symbol PP block.
 * Use magnitude only (non-coherent w.r.t. unknown constant phase),
 * and also test conjugate polarity to absorb I/Q inversion ambiguity. */
static float pp_block_correlation(const p3_symbol_t *syms, int start_sym, int phase_offset)
{
    float sum1_re = 0.0f, sum1_im = 0.0f;
    float sum2_re = 0.0f, sum2_im = 0.0f;
    int valid = 0;

    for (int i = 0; i < P3_PP_PERIOD_SYMBOLS; i++) {
        float ref_re, ref_im;
        float zr, zi, mag;

        pp_reference_symbol((phase_offset + i) % P3_PP_PERIOD_SYMBOLS, &ref_re, &ref_im);
        zr = syms[start_sym + i].re;
        zi = syms[start_sym + i].im;
        mag = sqrtf(zr * zr + zi * zi);
        if (mag < 1.0e-6f)
            continue;
        zr /= mag;
        zi /= mag;

        /* z * conj(ref) */
        sum1_re += zr * ref_re + zi * ref_im;
        sum1_im += zi * ref_re - zr * ref_im;

        /* z * ref (conjugate polarity hypothesis) */
        sum2_re += zr * ref_re - zi * ref_im;
        sum2_im += zi * ref_re + zr * ref_im;
        valid++;
    }

    if (valid < (P3_PP_PERIOD_SYMBOLS * 3) / 4)
        return 0.0f;
    {
        float c1 = hypotf(sum1_re, sum1_im) / (float)valid;
        float c2 = hypotf(sum2_re, sum2_im) / (float)valid;
        return (c1 > c2) ? c1 : c2;
    }
}

static bool is_pp_pattern(const p3_symbol_t *syms, int start, int len,
                          int *phase_out, int *match_len_out, float *conf_out)
{
    float best_first = 0.0f;
    int best_phase = 0;
    int max_blocks;
    int blocks = 0;
    float score_sum = 0.0f;

    if (!syms || len < 2 * P3_PP_PERIOD_SYMBOLS)
        return false;

    for (int phase = 0; phase < P3_PP_PERIOD_SYMBOLS; phase++) {
        float c = pp_block_correlation(syms, start, phase);
        if (c > best_first) {
            best_first = c;
            best_phase = phase;
        }
    }
    if (best_first < 0.58f)
        return false;

    max_blocks = len / P3_PP_PERIOD_SYMBOLS;
    for (int b = 0; b < max_blocks; b++) {
        float c = pp_block_correlation(syms,
                                       start + b * P3_PP_PERIOD_SYMBOLS,
                                       best_phase);
        if (b == 0) {
            if (c < 0.58f)
                break;
        } else {
            if (c < 0.52f)
                break;
        }
        score_sum += c;
        blocks++;
    }

    if (blocks < 2)
        return false;
    if ((score_sum / (float)blocks) < 0.56f)
        return false;

    if (phase_out)
        *phase_out = best_phase;
    if (match_len_out)
        *match_len_out = blocks * P3_PP_PERIOD_SYMBOLS;
    if (conf_out)
        *conf_out = score_sum / (float)blocks;
    return true;
}

static int symbol_trn_scrambled_bit(const p3_symbol_t *syms,
                                    int start,
                                    int bit_pos,
                                    int transform)
{
    int sym_idx = bit_pos / 2;
    int bit_sel = bit_pos & 1;
    int quadrant = syms[start + sym_idx].quadrant & 3;

    /* Unknown carrier phase gives four rotations; an I/Q conjugation reverses
     * the quadrant direction. V.34 TRN maps I directly to this absolute
     * quadrant, unlike the differentially encoded J sequence. */
    if (transform & 4)
        quadrant = (-quadrant) & 3;
    quadrant = (quadrant + (transform & 3)) & 3;

    return bit_sel ? ((quadrant >> 1) & 1) : (quadrant & 1);
}

static int symbol_descrambled_bit(const p3_symbol_t *syms, int start, int bit_pos)
{
    int sym_idx = bit_pos / 2;
    int bit_sel = bit_pos & 1;

    return bit_sel ? syms[start + sym_idx].bit1 : syms[start + sym_idx].bit0;
}

/* For TRN, the scrambled bitstream follows:
 * in[n] = 1 ^ in[n-23] ^ in[n-5]
 * This recurrence is independent of descrambler start state once we have
 * enough history, so it gives a robust pattern test on mid-stream captures. */
static int trn_recurrence_errors(const p3_symbol_t *syms,
                                 int start,
                                 int len,
                                 int *checks_out,
                                 int *transform_out)
{
    int total_bits;
    int checks;
    int best_errors = INT_MAX;
    int best_transform = 0;

    if (!syms || len <= 0) {
        if (checks_out)
            *checks_out = 0;
        return 0;
    }

    total_bits = len * 2;
    checks = total_bits - 23;
    for (int transform = 0; transform < 8; transform++) {
        int errors = 0;
        for (int b = 23; b < total_bits; b++) {
            int prev23 = symbol_trn_scrambled_bit(syms, start, b - 23, transform);
            int prev5 = symbol_trn_scrambled_bit(syms, start, b - 5, transform);
            int expect = 1 ^ prev23 ^ prev5;
            int got = symbol_trn_scrambled_bit(syms, start, b, transform);
            if (got != expect)
                errors++;
        }
        if (errors < best_errors) {
            best_errors = errors;
            best_transform = transform;
        }
    }
    if (checks_out)
        *checks_out = checks;
    if (transform_out)
        *transform_out = best_transform;
    return best_errors;
}

static int trn_descrambled_errors(const p3_symbol_t *syms,
                                  int start,
                                  int len,
                                  int transform,
                                  int *bits_out)
{
    uint32_t sr = 0;
    int errors = 0;
    int bits = len * 2;

    for (int b = 0; b < bits; b++) {
        int in = symbol_trn_scrambled_bit(syms, start, b, transform);
        if (descramble_bit(&sr, in) != 1)
            errors++;
    }
    if (bits_out)
        *bits_out = bits;
    return errors;
}

/* Check if a run of symbols looks like TRN (scrambled ones) */
static bool is_trn_pattern(const p3_symbol_t *syms, int start, int len)
{
    int checks;
    int errors;
    int transform;
    int descr_errors;
    int descr_bits;

    if (len < 24)
        return false;

    errors = trn_recurrence_errors(syms, start, len, &checks, &transform);
    if (checks >= 24) {
        /* Recurrence match >= 78% */
        if ((checks - errors) * 100 >= checks * 78)
            return true;
    }

    /* Fallback when recurrence window is short/noisy. TRN resets its
     * scrambler, so descramble locally at the candidate boundary. */
    descr_errors = trn_descrambled_errors(syms, start, len, transform, &descr_bits);
    return ((descr_bits - descr_errors) * 100 >= descr_bits * 80);
}

/* Check for Ru/uR 2-point pattern (V.92 §3.8):
 * Ru  = {+LU, +LU, +LU, -LU, -LU, -LU} repeating (6-symbol)
 * uR  = {-LU, -LU, -LU, +LU, +LU, +LU} repeating (complement) */
static bool is_ru_pattern(const p3_symbol_t *syms, int start, int len,
                          bool *positive_first)
{
    int period = 6;
    int match_normal = 0;
    int match_complement = 0;

    if (len < 18)
        return false;

    for (int i = 0; i < len; i++) {
        /* For 2-point: dibit is either 0 (same phase) or 2 (opposite) */
        int d = syms[start + i].dibit;
        int pos_in_period = i % period;
        bool first_half = (pos_in_period < 3);

        /* Normal Ru: same phase in first half, opposite in second */
        if (first_half) {
            if (d == 0)
                match_normal++;
            if (d == 2)
                match_complement++;
        } else {
            if (d == 2)
                match_normal++;
            if (d == 0)
                match_complement++;
        }
    }

    if (match_normal * 5 >= len * 4) {
        if (positive_first) *positive_first = true;
        return true;
    }
    if (match_complement * 5 >= len * 4) {
        if (positive_first) *positive_first = false;
        return true;
    }
    return false;
}

/* Detect J frame: 16-bit repeating pattern in descrambled bits */
static bool detect_j_pattern(const p3_symbol_t *syms, int start, int len,
                             uint16_t *trn16, int *hypothesis,
                             int *match_pct_out)
{
    uint16_t best_pat = 0;
    int best_phase = 0;
    int best_matches = 0;
    int best_checks = 0;
    int total_bits;

    if (len < 16)
        return false;

    total_bits = len * 2;

    /* Need enough repeats to avoid random periodic false positives. */
    if (total_bits < 64)
        return false;

    for (int phase = 0; phase < 16; phase++) {
        uint16_t pat = 0;
        int matches = 0;
        int checks = 0;

        if (phase + 16 > total_bits)
            break;
        for (int b = 0; b < 16; b++) {
            int bit = symbol_descrambled_bit(syms, start, phase + b);
            pat |= (uint16_t) (bit & 1) << b;
        }

        for (int b = phase; b < total_bits; b++) {
            int bit = symbol_descrambled_bit(syms, start, b);
            int expected = (int) ((pat >> ((b - phase) % 16)) & 1);
            checks++;
            if (bit == expected)
                matches++;
        }
        if (checks > 0 && (matches > best_matches
                           || (matches == best_matches && checks > best_checks))) {
            best_pat = pat;
            best_phase = phase;
            best_matches = matches;
            best_checks = checks;
        }
    }

    if (best_checks == 0)
        return false;
    /* Require strong periodicity for J detection. */
    if (best_matches * 100 < best_checks * 82)
        return false;

    /* Reject degenerate low-entropy patterns that arise from false locks. */
    {
        int ones = 0;
        for (int b = 0; b < 16; b++)
            ones += (best_pat >> b) & 1;
        if (ones < 3 || ones > 13)
            return false;
    }
    for (int p = 1; p <= 8; p <<= 1) {
        bool repeats = true;
        for (int b = p; b < 16; b++) {
            int bit = (best_pat >> b) & 1;
            int prev = (best_pat >> (b - p)) & 1;
            if (bit != prev) {
                repeats = false;
                break;
            }
        }
        if (repeats)
            return false;
    }

    if (trn16) *trn16 = best_pat;
    if (hypothesis) *hypothesis = best_phase;
    if (match_pct_out)
        *match_pct_out = (best_matches * 100 + best_checks / 2) / best_checks;
    return true;
}

/* Table 18 / Table 19 canonical bit patterns (left-most bit is first in time). */
static const uint8_t j_table18_4pt_bits[16]  = {0,0,0,0,1,0,0,1,1,0,0,1,0,0,0,1};
static const uint8_t j_table18_16pt_bits[16] = {0,0,0,0,1,1,0,1,1,0,0,1,0,0,0,1};
static const uint8_t j_table19_prime_bits[16] = {1,1,1,1,1,0,0,1,1,0,0,1,0,0,0,1};

static int j_transformed_bit(const p3_symbol_t *syms, int start, int bit_pos, int transform)
{
    int sym_idx = bit_pos / 2;
    int bsel = bit_pos & 1;
    int b0 = symbol_descrambled_bit(syms, start, sym_idx * 2);
    int b1 = symbol_descrambled_bit(syms, start, sym_idx * 2 + 1);
    int bit;

    if (transform & 2)
        bit = bsel ? b0 : b1; /* swap I1/I2 */
    else
        bit = bsel ? b1 : b0;
    if (transform & 1)
        bit ^= 1; /* invert */
    return bit;
}

static int j_match_periodic_pattern_pct(const p3_symbol_t *syms,
                                        int start,
                                        int len_symbols,
                                        const uint8_t pattern[16],
                                        int phase,
                                        int transform)
{
    int bit_count = len_symbols * 2;
    int matches = 0;

    if (!syms || len_symbols <= 0)
        return 0;
    for (int b = 0; b < bit_count; b++) {
        int got = j_transformed_bit(syms, start, b, transform);
        int exp = pattern[(phase + b) & 15];
        if (got == exp)
            matches++;
    }
    return (matches * 100 + bit_count / 2) / bit_count;
}

static int j_match_single_block_pct(const p3_symbol_t *syms,
                                    int start,
                                    const uint8_t pattern[16],
                                    int phase,
                                    int transform)
{
    int matches = 0;

    if (!syms)
        return 0;
    for (int b = 0; b < 16; b++) {
        int got = j_transformed_bit(syms, start, b, transform);
        int exp = pattern[(phase + b) & 15];
        if (got == exp)
            matches++;
    }
    return (matches * 100 + 8) / 16;
}

static void classify_j_table18(const p3_symbol_t *syms,
                               int start,
                               int len_symbols,
                               int *j_bits_out,
                               int *phase_out,
                               int *transform_out,
                               int *match_pct_out)
{
    int best_bits = 0;
    int best_phase = 0;
    int best_xform = 0;
    int best_pct = -1;
    int best_pref = -1;

    if (!j_bits_out || !phase_out || !transform_out || !match_pct_out)
        return;
    *j_bits_out = 0;
    *phase_out = 0;
    *transform_out = 0;
    *match_pct_out = 0;
    if (!syms || len_symbols < 32)
        return;

    for (int pat_idx = 0; pat_idx < 2; pat_idx++) {
        const uint8_t *pat = (pat_idx == 0) ? j_table18_4pt_bits : j_table18_16pt_bits;
        int pat_bits = (pat_idx == 0) ? 4 : 16;
        for (int xform = 0; xform < 4; xform++) {
            int pref = (xform == 0) ? 2 : (xform == 1 ? 1 : 0);
            for (int phase = 0; phase < 16; phase++) {
                int pct = j_match_periodic_pattern_pct(syms, start, len_symbols, pat, phase, xform);
                if (pct > best_pct
                    || (pct == best_pct && pref > best_pref)
                    || (pct == best_pct && pref == best_pref && pat_bits == 16 && best_bits == 4)) {
                    best_pct = pct;
                    best_bits = pat_bits;
                    best_phase = phase;
                    best_xform = xform;
                    best_pref = pref;
                }
            }
        }
    }

    if (best_pct >= 70) {
        *j_bits_out = best_bits;
        *phase_out = best_phase;
        *transform_out = best_xform;
        *match_pct_out = best_pct;
    } else {
        *j_bits_out = 0;
        *phase_out = best_phase;
        *transform_out = best_xform;
        *match_pct_out = (best_pct < 0) ? 0 : best_pct;
    }
}

static bool detect_j_prime_after_j(const p3_symbol_t *syms,
                                   int start,
                                   int max_len_symbols,
                                   int force_transform,
                                   int *start_offset_out,
                                   int *phase_out,
                                   int *match_pct_out)
{
    int best_off = 0;
    int best_phase = 0;
    int best_pct = -1;
    int xform_start = 0;
    int xform_end = 4;
    int max_off;

    if (!syms || max_len_symbols < 8)
        return false;
    if (force_transform >= 0 && force_transform < 4) {
        xform_start = force_transform;
        xform_end = force_transform + 1;
    }
    max_off = max_len_symbols - 8;
    if (max_off > 24)
        max_off = 24;

    for (int off = 0; off <= max_off; off++) {
        for (int xform = xform_start; xform < xform_end; xform++) {
            for (int phase = 0; phase < 16; phase++) {
                int pct = j_match_single_block_pct(syms,
                                                   start + off,
                                                   j_table19_prime_bits,
                                                   phase,
                                                   xform);
                if (pct > best_pct) {
                    best_pct = pct;
                    best_phase = phase;
                    best_off = off;
                }
            }
        }
    }
    if (start_offset_out)
        *start_offset_out = best_off;
    if (phase_out)
        *phase_out = best_phase;
    if (match_pct_out)
        *match_pct_out = (best_pct < 0) ? 0 : best_pct;
    return best_pct >= 75;
}

/* Check for silence/very low energy */
static bool is_silence(const p3_symbol_t *syms, int start, int len,
                       float threshold)
{
    float sum = 0.0f;

    for (int i = 0; i < len; i++)
        sum += syms[start + i].magnitude;
    return (sum / (float)len) < threshold;
}

/* Add a segment to the result */
static void add_segment(p3_result_t *result, p3_signal_type_t type,
                        int start_sym, int length,
                        const p3_symbol_t *syms)
{
    p3_segment_t *seg;
    float mag_sum = 0.0f;

    if (!result || result->segment_count >= result->segment_capacity)
        return;

    seg = &result->segments[result->segment_count++];
    memset(seg, 0, sizeof(*seg));
    seg->type = type;
    seg->start_symbol = start_sym;
    seg->length = length;
    seg->start_sample = syms[start_sym].sample_index;
    if (start_sym + length - 1 < result->symbol_count)
        seg->end_sample = syms[start_sym + length - 1].sample_index;

    for (int i = 0; i < length; i++)
        mag_sum += syms[start_sym + i].magnitude;
    seg->avg_magnitude = (length > 0) ? mag_sum / (float)length : 0.0f;
}

int p3_segment_symbols(p3_result_t *result)
{
    int n;
    const p3_symbol_t *syms;
    int pos;
    float silence_threshold;
    float total_mag;

    if (!result || result->symbol_count == 0)
        return 0;

    n = result->symbol_count;
    syms = result->symbols;
    result->segment_count = 0;

    /* Compute silence threshold as 10% of average magnitude */
    total_mag = 0.0f;
    for (int i = 0; i < n; i++)
        total_mag += syms[i].magnitude;
    silence_threshold = total_mag / (float)n * 0.10f;

    pos = 0;
    while (pos < n) {
        int remaining = n - pos;
        int seg_len;
        bool s_complement = false;
        bool ru_positive = false;
        int pp_phase = 0;
        int pp_match_len = 0;
        float pp_conf = 0.0f;
        uint16_t j_trn16 = 0;
        int j_hyp = 0;
        int j_periodic_match_pct = 0;


        /* Skip silence */
        if (is_silence(syms, pos, (remaining < 6) ? remaining : 6,
                       silence_threshold)) {
            int end = pos;
            while (end < n && syms[end].magnitude < silence_threshold)
                end++;
            /* Guarantee forward progress: if window-average was silence but no
             * individual sample was below threshold, skip at least 1 symbol. */
            if (end <= pos)
                end = pos + 1;
            add_segment(result, P3_SIGNAL_SILENCE, pos, end - pos, syms);
            pos = end;
            continue;
        }

        /* Try to identify the signal type.
         * Test patterns from most specific to least specific. */

        /* Check for Ru/uR (V.92, 6-symbol 2-point pattern). */
        if (remaining >= 18 && is_ru_pattern(syms, pos, 18, &ru_positive)) {
            /* Extend incrementally: check new 6 symbols match the period-6 2-point pattern */
            seg_len = 18;
            while (pos + seg_len + 6 <= n) {
                int ext_match = 0;
                for (int ei = 0; ei < 6; ei++) {
                    int d = syms[pos + seg_len + ei].dibit;
                    int pos_in_period = (seg_len + ei) % 6;
                    bool first_half = (pos_in_period < 3);
                    int expected = (first_half == ru_positive) ? 0 : 2;
                    if (d == expected)
                        ext_match++;
                }
                if (ext_match < 4)  /* Need 4/6 match to continue */
                    break;
                seg_len += 6;
            }
            {
                p3_segment_t *seg;
                add_segment(result,
                            ru_positive ? P3_SIGNAL_RU : P3_SIGNAL_UR,
                            pos, seg_len, syms);
                seg = &result->segments[result->segment_count - 1];
                seg->ru_positive_first = ru_positive;
                seg->confidence = 0.9f;
            }
            pos += seg_len;
            continue;
        }

        /* Check for S / S-bar (6-symbol repeating) */
        if (remaining >= 12 && is_s_pattern(syms, pos, 12, &s_complement)) {
            /* Extend incrementally: check 6-symbol consistency of new chunk */
            seg_len = 12;
            while (pos + seg_len + 6 <= n) {
                int ext_match = 0;
                for (int ei = 0; ei < 6; ei++) {
                    int d = syms[pos + seg_len + ei].dibit;
                    int ref = syms[pos + (seg_len + ei) % 6].dibit;
                    if (d == ref)
                        ext_match++;
                }
                /* Allow one symbol error per 6-symbol extension chunk. */
                if (ext_match < 5)
                    break;
                seg_len += 6;
            }
            add_segment(result,
                        s_complement ? P3_SIGNAL_S_BAR : P3_SIGNAL_S,
                        pos, seg_len, syms);
            result->segments[result->segment_count - 1].confidence = 0.85f;
            pos += seg_len;
            continue;
        }

        /* Check for PP (48-symbol block repeated >=2 times). */
        if (remaining >= 2 * P3_PP_PERIOD_SYMBOLS
            && is_pp_pattern(syms, pos, remaining, &pp_phase, &pp_match_len, &pp_conf)) {
            p3_segment_t *seg;

            add_segment(result, P3_SIGNAL_PP, pos, pp_match_len, syms);
            seg = &result->segments[result->segment_count - 1];
            seg->pp_phase = pp_phase;
            seg->pp_blocks = pp_match_len / P3_PP_PERIOD_SYMBOLS;
            seg->confidence = pp_conf;
            pos += pp_match_len;
            continue;
        }

        /* Check for TRN (scrambled ones) */
        if (remaining >= 24 && is_trn_pattern(syms, pos, 24)) {
            /* Extend incrementally: check scrambler recurrence on new symbols */
            seg_len = 24;
            while (pos + seg_len + 12 <= n) {
                int ext_errors;
                int ext_checks = 0;
                ext_errors = trn_recurrence_errors(syms,
                                                   pos,
                                                   seg_len + 12,
                                                   &ext_checks,
                                                   NULL);
                if (ext_checks > 0 && (ext_checks - ext_errors) * 100 < ext_checks * 70)
                    break;
                seg_len += 12;
            }
            {
                int checks = 0;
                int transform;
                int error_bits = trn_recurrence_errors(syms,
                                                       pos,
                                                       seg_len,
                                                       &checks,
                                                       &transform);
                int descr_bits;
                int descr_errors = trn_descrambled_errors(syms,
                                                          pos,
                                                          seg_len,
                                                          transform,
                                                          &descr_bits);
                float conf;
                add_segment(result, P3_SIGNAL_TRN, pos, seg_len, syms);
                result->segments[result->segment_count - 1].trn_errors = error_bits;
                result->segments[result->segment_count - 1].trn_recurrence_checks = checks;
                result->segments[result->segment_count - 1].trn_recurrence_match_pct =
                    (checks > 0) ? ((checks - error_bits) * 100 + checks / 2) / checks : 0;
                result->segments[result->segment_count - 1].trn_descrambled_errors = descr_errors;
                result->segments[result->segment_count - 1].trn_descrambled_bits = descr_bits;
                result->segments[result->segment_count - 1].trn_descrambled_ber_pct =
                    (descr_bits > 0) ? (descr_errors * 100 + descr_bits / 2) / descr_bits : 0;
                if (checks > 0)
                    conf = 1.0f - (float) error_bits / (float) checks;
                else
                    conf = 0.5f;
                result->segments[result->segment_count - 1].confidence = conf;
            }
            pos += seg_len;
            continue;
        }

        /* Check for J frame (16-bit repeating descrambled pattern) */
        if (remaining >= 48
            && detect_j_pattern(syms, pos, 48, &j_trn16, &j_hyp,
                                &j_periodic_match_pct)) {
            int j_table_bits = 0;
            int j_table_phase = 0;
            int j_table_transform = 0;
            int j_table_match_pct = 0;
            int jprime_offset = 0;
            int jprime_phase = 0;
            int jprime_pct = 0;
            bool have_jprime = false;

            /* Extend incrementally: verify new bits match the 16-bit pattern */
            seg_len = 48;
            while (pos + seg_len + 8 <= n) {
                int ext_matches = 0;
                int ext_total = 16;
                for (int ei = 0; ei < 8; ei++) {
                    int b0 = symbol_descrambled_bit(syms, pos, (seg_len + ei) * 2);
                    int b1 = symbol_descrambled_bit(syms, pos, (seg_len + ei) * 2 + 1);
                    int e0 = (int)((j_trn16 >> (((seg_len + ei) * 2 - j_hyp) % 16)) & 1);
                    int e1 = (int)((j_trn16 >> (((seg_len + ei) * 2 + 1 - j_hyp) % 16)) & 1);
                    if (b0 == e0) ext_matches++;
                    if (b1 == e1) ext_matches++;
                }
                if (ext_matches * 100 < ext_total * 70)
                    break;
                seg_len += 8;
            }
            {
                p3_segment_t *seg;
                add_segment(result, P3_SIGNAL_J, pos, seg_len, syms);
                seg = &result->segments[result->segment_count - 1];
                seg->j_trn16 = j_trn16;
                seg->j_hypothesis = j_hyp;
                seg->j_periodic_match_pct = j_periodic_match_pct;
                classify_j_table18(syms,
                                   pos,
                                   seg_len,
                                   &j_table_bits,
                                   &j_table_phase,
                                   &j_table_transform,
                                   &j_table_match_pct);
                seg->j_table_bits = j_table_bits;
                seg->j_table_phase = j_table_phase;
                seg->j_table_transform = j_table_transform;
                seg->j_table_match_pct = j_table_match_pct;
                seg->confidence = 0.78f;

                if (pos + seg_len + 8 <= n
                    && detect_j_prime_after_j(syms,
                                              pos + seg_len,
                                              n - (pos + seg_len),
                                              j_table_transform,
                                              &jprime_offset,
                                              &jprime_phase,
                                              &jprime_pct)) {
                    p3_segment_t *jp;
                    add_segment(result, P3_SIGNAL_J_PRIME, pos + seg_len + jprime_offset, 8, syms);
                    jp = &result->segments[result->segment_count - 1];
                    jp->j_table_phase = jprime_phase;
                    jp->j_table_transform = j_table_transform;
                    jp->jprime_match_pct = jprime_pct;
                    jp->confidence = (float) jprime_pct / 100.0f;
                    have_jprime = true;
                    seg->jprime_match_pct = jprime_pct;
                }
            }
            pos += seg_len + (have_jprime ? (jprime_offset + 8) : 0);
            continue;
        }

        /* Unknown: advance by a small window */
        seg_len = (remaining < 6) ? remaining : 6;
        add_segment(result, P3_SIGNAL_UNKNOWN, pos, seg_len, syms);
        pos += seg_len;
    }

    return result->segment_count;
}

static int rounded_pct(int matches, int checks)
{
    return (checks > 0) ? (matches * 100 + checks / 2) / checks : 0;
}

static void phase4_trn_quality(const p3_symbol_t *syms,
                               int start,
                               int len,
                               bool source_calling_party,
                               int *recurrence_pct_out,
                               int *dber_pct_out)
{
    static const uint8_t map_table[24][4] = {
        {0,1,2,3}, {1,0,3,2}, {2,3,0,1}, {3,2,1,0},
        {0,2,1,3}, {2,0,3,1}, {1,3,0,2}, {3,1,2,0},
        {0,3,2,1}, {1,2,3,0}, {2,1,0,3}, {3,0,1,2},
        {0,1,3,2}, {1,0,2,3}, {2,3,1,0}, {3,2,0,1},
        {0,2,3,1}, {1,3,2,0}, {2,0,1,3}, {3,1,0,2},
        {0,3,1,2}, {1,2,0,3}, {2,1,3,0}, {3,0,2,1}
    };
    int best_ones = -1;
    int total_bits = len * 2;
    int tap = source_calling_party ? 17 : 4;

    /* Mirror SpanDSP's robust Phase 4 TRN search: absolute/differential
     * domains, all 24 invertible dibit maps, and both bit serialization
     * orders. The exact timing boundary supplies the scrambler reset. */
    for (int domain = 0; domain < 2; domain++) {
        for (int order = 0; order < 2; order++) {
            for (int h = 0; h < 24; h++) {
                uint32_t reg = 0;
                int ones = 0;

                for (int sym = 0; sym < len; sym++) {
                    int raw = domain ? syms[start + sym].quadrant
                                     : syms[start + sym].dibit;
                    int mapped = map_table[h][raw & 3];
                    int first = order ? ((mapped >> 1) & 1) : (mapped & 1);
                    int second = order ? (mapped & 1) : ((mapped >> 1) & 1);
                    int d0 = (first ^ (reg >> tap) ^ (reg >> 22)) & 1;

                    reg = ((reg << 1) | (uint32_t)first) & 0x7FFFFF;
                    {
                        int d1 = (second ^ (reg >> tap) ^ (reg >> 22)) & 1;
                        reg = ((reg << 1) | (uint32_t)second) & 0x7FFFFF;
                        ones += d0 + d1;
                    }
                }
                if (ones > best_ones)
                    best_ones = ones;
            }
            }
        }
    if (recurrence_pct_out)
        *recurrence_pct_out = rounded_pct(best_ones, total_bits);
    if (dber_pct_out)
        *dber_pct_out = rounded_pct(total_bits - best_ones, total_bits);
}

static void phase4_s_sbar_quality(const p3_symbol_t *syms,
                                  int start,
                                  int *s_pct_out,
                                  int *sbar_pct_out)
{
    int best_s = 0;
    int best_sbar = 0;
    int best_total = -1;

    /* V.34 10.1.3.7: S alternates 0,-90 degrees for 128T; S-bar
     * alternates 180,+90 degrees for 16T. Test both spectral polarities,
     * since an I/Q conjugation swaps differential dibits 1 and 3. */
    for (int polarity = 0; polarity < 2; polarity++) {
        int neg90 = polarity ? P3_DIBIT_1 : P3_DIBIT_3;
        int pos90 = polarity ? P3_DIBIT_3 : P3_DIBIT_1;
        int s_matches = 0;
        int sbar_matches = 0;

        for (int i = 1; i < 128; i++) {
            int expected = (i & 1) ? neg90 : pos90;
            if (syms[start + i].dibit == expected)
                s_matches++;
        }
        /* Include the S-to-S-bar transition as the first S-bar check.
         * S ends at -90 degrees and S-bar starts at 180 degrees, another
         * -90-degree differential step. */
        for (int i = 128; i < 144; i++) {
            int local = i - 128;
            int expected = (local == 0 || (local & 1)) ? neg90 : pos90;
            if (syms[start + i].dibit == expected)
                sbar_matches++;
        }
        if (s_matches + sbar_matches > best_total) {
            best_total = s_matches + sbar_matches;
            best_s = s_matches;
            best_sbar = sbar_matches;
        }
    }
    if (s_pct_out)
        *s_pct_out = rounded_pct(best_s, 127);
    if (sbar_pct_out)
        *sbar_pct_out = rounded_pct(best_sbar, 16);
}

static int phase4_jprime_quality(const p3_symbol_t *syms, int start, int transform)
{
    int matches = 0;

    if (transform < 0 || transform > 3)
        return 0;
    for (int b = 0; b < 16; b++) {
        int got = j_transformed_bit(syms, start, b, transform);
        if (got == j_table19_prime_bits[b])
            matches++;
    }
    return rounded_pct(matches, 16);
}

bool p3_find_phase4_timing(const p3_result_t *result,
                           int j_start_symbol,
                           int j_length,
                           int j_transform,
                           bool source_calling_party,
                           p3_phase4_timing_quality_t *out)
{
    const p3_symbol_t *syms;
    int j_end;
    int lo;
    int hi;
    int best_score = -1;

    if (!out)
        return false;
    memset(out, 0, sizeof(*out));
    out->source_calling_party = source_calling_party;
    if (!result || !result->symbols || j_start_symbol < 0 || j_length < 48)
        return false;

    syms = result->symbols;
    j_end = j_start_symbol + j_length;
    lo = j_end - 24;
    if (lo < j_start_symbol + 32)
        lo = j_start_symbol + 32;

    if (source_calling_party) {
        /* Call modem: J is terminated by exactly one 16-bit (8T) J'
         * sequence, immediately followed by TRN. Search only across the
         * demodulator's possible J end-boundary error. */
        hi = j_end + 32;
        for (int start = lo; start <= hi; start++) {
            int jprime_pct;
            int recurrence_pct;
            int dber_pct;
            int score;

            if (start + 8 + 512 > result->symbol_count)
                break;
            jprime_pct = phase4_jprime_quality(syms, start, j_transform);
            phase4_trn_quality(syms, start + 8, 512,
                               source_calling_party,
                               &recurrence_pct, &dber_pct);
            score = jprime_pct * 2 + recurrence_pct - dber_pct;
            if (score > best_score) {
                best_score = score;
                out->start_symbol = start;
                out->trn_start_symbol = start + 8;
                out->jprime_match_pct = jprime_pct;
                out->trn_recurrence_match_pct = recurrence_pct;
                out->trn_descrambled_ber_pct = dber_pct;
            }
        }
    } else {
        int max_wait_symbols = (int)(result->baud_rate_estimate * 0.5f + 0.5f);

        /* Answer modem: it may wait up to 500 ms after J, then sends
         * S(128T), S-bar(16T), and TRN with no intervening gap. */
        if (max_wait_symbols <= 0)
            max_wait_symbols = 1715;
        hi = j_end + max_wait_symbols + 24;
        for (int start = lo; start <= hi; start++) {
            int s_pct;
            int sbar_pct;
            int recurrence_pct;
            int dber_pct;
            int score;

            if (start + 144 + 512 > result->symbol_count)
                break;
            phase4_s_sbar_quality(syms, start, &s_pct, &sbar_pct);
            phase4_trn_quality(syms, start + 144, 512,
                               source_calling_party,
                               &recurrence_pct, &dber_pct);
            score = s_pct * 2 + sbar_pct * 2 + recurrence_pct - dber_pct;
            if (score > best_score) {
                best_score = score;
                out->start_symbol = start;
                out->trn_start_symbol = start + 144;
                out->s_match_pct = s_pct;
                out->sbar_match_pct = sbar_pct;
                out->trn_recurrence_match_pct = recurrence_pct;
                out->trn_descrambled_ber_pct = dber_pct;
            }
        }
    }

    if (best_score < 0)
        return false;
    out->found = true;
    out->start_sample = syms[out->start_symbol].sample_index;
    out->trn_start_sample = syms[out->trn_start_symbol].sample_index;
    return true;
}

bool p3_find_answer_phase4_timing(const p3_result_t *result,
                                  p3_phase4_timing_quality_t *out)
{
    int best_score = INT_MIN;

    if (!out)
        return false;
    memset(out, 0, sizeof(*out));
    if (!result || !result->symbols || result->symbol_count < 144 + 512)
        return false;

    out->source_calling_party = false;
    for (int start = 1; start + 144 + 512 <= result->symbol_count; start++) {
        int s_pct;
        int sbar_pct;
        int trn_pct;
        int dber_pct;
        int score;

        phase4_s_sbar_quality(result->symbols, start, &s_pct, &sbar_pct);
        /* Avoid the expensive 24-hypothesis TRN evaluation unless the long
         * normative waveform already has meaningful evidence. */
        if (s_pct < 65 || sbar_pct < 63)
            continue;
        phase4_trn_quality(result->symbols,
                           start + 144,
                           512,
                           false,
                           &trn_pct,
                           &dber_pct);
        score = s_pct * 3 + sbar_pct * 4 + trn_pct - dber_pct;
        if (score > best_score) {
            best_score = score;
            out->start_symbol = start;
            out->start_sample = result->symbols[start].sample_index;
            out->trn_start_symbol = start + 144;
            out->trn_start_sample = result->symbols[start + 144].sample_index;
            out->s_match_pct = s_pct;
            out->sbar_match_pct = sbar_pct;
            out->trn_recurrence_match_pct = trn_pct;
            out->trn_descrambled_ber_pct = dber_pct;
        }
    }
    if (best_score == INT_MIN)
        return false;
    out->found = true;
    return true;
}

typedef struct {
    p3_phase4_timing_quality_t timing;
    int j_table_match_pct;
} p3_call_phase4_candidate_t;

bool p3_find_stereo_phase4_timing(const p3_result_t *call_source,
                                  const p3_result_t *answer_source,
                                  int min_start_sample,
                                  p3_phase4_timing_quality_t *call_out,
                                  p3_phase4_timing_quality_t *answer_out,
                                  int *score_out)
{
    p3_call_phase4_candidate_t *calls;
    int call_count = 0;
    int j_segments = 0;
    int j48_segments = 0;
    int best_jprime_pct = 0;
    int best_call_trn_pct = 0;
    int fallback_answer_count = 0;
    int fallback_jprime_count = 0;
    int fallback_prefix_count = 0;
    int answer_pattern_count = 0;
    int nearby_pattern_count = 0;
    int best_score = INT_MIN;
    p3_phase4_timing_quality_t best_call;
    p3_phase4_timing_quality_t best_answer;

    if (!call_source || !answer_source || !call_out || !answer_out
        || !call_source->symbols || !answer_source->symbols
        || call_source->segment_count <= 0
        || answer_source->symbol_count < 144 + 512) {
        return false;
    }
    calls = calloc((size_t) call_source->symbol_count, sizeof(*calls));
    if (!calls)
        return false;

    /* J' is only 16 bits, so it is not sufficient by itself.  Retain J
     * candidates only when the immediately following 512T also has useful
     * TRN recurrence; the opposite channel supplies the final discriminator. */
    for (int i = 0; i < call_source->segment_count; i++) {
        const p3_segment_t *seg = &call_source->segments[i];
        p3_phase4_timing_quality_t timing;

        if (seg->type != P3_SIGNAL_J)
            continue;
        j_segments++;
        if (seg->length < 48)
            continue;
        j48_segments++;
        if (!p3_find_phase4_timing(call_source,
                                   seg->start_symbol,
                                   seg->length,
                                   seg->j_table_transform,
                                   true,
                                   &timing)) {
            continue;
        }
        if (timing.jprime_match_pct > best_jprime_pct)
            best_jprime_pct = timing.jprime_match_pct;
        if (timing.trn_recurrence_match_pct > best_call_trn_pct)
            best_call_trn_pct = timing.trn_recurrence_match_pct;
        if (getenv("P3_MATCH_STATS")) {
            fprintf(stderr,
                    "[P3STEREO-J] sample=%d table=%d%% Jprime=%d%% TRN=%d%% DBER=%d%% min=%d\n",
                    timing.start_sample,
                    seg->j_table_match_pct,
                    timing.jprime_match_pct,
                    timing.trn_recurrence_match_pct,
                    timing.trn_descrambled_ber_pct,
                    min_start_sample);
        }
        if (timing.jprime_match_pct < 63
            || timing.trn_recurrence_match_pct < 65
            || timing.trn_descrambled_ber_pct > 35
            || timing.start_sample < min_start_sample) {
            continue;
        }
        calls[call_count].timing = timing;
        calls[call_count].j_table_match_pct = seg->j_table_match_pct;
        call_count++;
    }

    /* If the loose segmenter found no usable late J, search only in the
     * standard response window around plausible answer-side S/S-bar
     * boundaries.  J' alone is short, so require both a 64T TRN prefix and
     * the full 512T recurrence before retaining a boundary. */
    if (call_count == 0) {
        int last_added_sample = -1;

        for (int astart = 1;
             astart + 144 + 512 <= answer_source->symbol_count;
             astart++) {
            int s_pct;
            int sbar_pct;
            int answer_sample;
            int answer_trn_pct;
            int answer_dber_pct;

            phase4_s_sbar_quality(answer_source->symbols,
                                  astart,
                                  &s_pct,
                                  &sbar_pct);
            if (s_pct < 60 || sbar_pct < 75)
                continue;
            answer_sample = answer_source->symbols[astart].sample_index;
            if (answer_sample < min_start_sample)
                continue;
            phase4_trn_quality(answer_source->symbols,
                               astart + 144,
                               512,
                               false,
                               &answer_trn_pct,
                               &answer_dber_pct);
            if (answer_trn_pct < 50)
                continue;
            fallback_answer_count++;

            for (int cstart = 1; cstart + 8 + 512 <= call_source->symbol_count; cstart++) {
                int call_sample = call_source->symbols[cstart].sample_index;
                int best_jprime = 0;
                int j_bits;
                int j_phase;
                int j_transform;
                int j_table_pct;
                int prefix_pct;
                int prefix_dber;
                int trn_pct;
                int dber_pct;

                if (call_sample < answer_sample - 4400)
                    continue;
                if (call_sample > answer_sample + 400)
                    break;
                if (call_sample < min_start_sample || call_sample == last_added_sample
                    || cstart < 64)
                    continue;
                for (int transform = 0; transform < 4; transform++) {
                    int pct = phase4_jprime_quality(call_source->symbols,
                                                    cstart,
                                                    transform);
                    if (pct > best_jprime)
                        best_jprime = pct;
                }
                if (best_jprime < 75)
                    continue;
                fallback_jprime_count++;
                phase4_trn_quality(call_source->symbols,
                                   cstart + 8,
                                   64,
                                   true,
                                   &prefix_pct,
                                   &prefix_dber);
                if (prefix_pct < 60 || prefix_dber > 40)
                    continue;
                fallback_prefix_count++;
                classify_j_table18(call_source->symbols,
                                   cstart - 64,
                                   64,
                                   &j_bits,
                                   &j_phase,
                                   &j_transform,
                                   &j_table_pct);
                best_jprime = phase4_jprime_quality(call_source->symbols,
                                                    cstart,
                                                    j_transform);
                if (j_table_pct < 60 || best_jprime < 63)
                    continue;
                phase4_trn_quality(call_source->symbols,
                                   cstart + 8,
                                   512,
                                   true,
                                   &trn_pct,
                                   &dber_pct);
                if (trn_pct < 50)
                    continue;
                memset(&calls[call_count], 0, sizeof(calls[call_count]));
                calls[call_count].timing.found = true;
                calls[call_count].timing.source_calling_party = true;
                calls[call_count].timing.start_symbol = cstart;
                calls[call_count].timing.start_sample = call_sample;
                calls[call_count].timing.trn_start_symbol = cstart + 8;
                calls[call_count].timing.trn_start_sample =
                    call_source->symbols[cstart + 8].sample_index;
                calls[call_count].timing.jprime_match_pct = best_jprime;
                calls[call_count].timing.trn_recurrence_match_pct = trn_pct;
                calls[call_count].timing.trn_descrambled_ber_pct = dber_pct;
                calls[call_count].j_table_match_pct = j_table_pct;
                last_added_sample = call_sample;
                call_count++;
                if (call_count >= call_source->symbol_count)
                    break;
            }
            if (call_count >= call_source->symbol_count)
                break;
        }
    }

    memset(&best_call, 0, sizeof(best_call));
    memset(&best_answer, 0, sizeof(best_answer));
    for (int start = 1;
         call_count > 0 && start + 144 + 512 <= answer_source->symbol_count;
         start++) {
        int s_pct;
        int sbar_pct;
        int answer_sample;
        bool has_nearby_call = false;

        phase4_s_sbar_quality(answer_source->symbols, start, &s_pct, &sbar_pct);
        if (s_pct < 60 || sbar_pct < 75)
            continue;
        answer_pattern_count++;
        answer_sample = answer_source->symbols[start].sample_index;
        if (answer_sample < min_start_sample)
            continue;
        for (int c = 0; c < call_count; c++) {
            int delta = answer_sample - calls[c].timing.start_sample;

            /* Allow a small negative skew for independent matched-filter
             * delays, then the V.34 answerer's specified <=500 ms J wait. */
            if (delta >= -400 && delta <= 4400) {
                has_nearby_call = true;
                break;
            }
        }
        if (has_nearby_call) {
            int trn_pct;
            int dber_pct;

            nearby_pattern_count++;

            phase4_trn_quality(answer_source->symbols,
                               start + 144,
                               512,
                               false,
                               &trn_pct,
                               &dber_pct);
            if (trn_pct < 50)
                continue;
            for (int c = 0; c < call_count; c++) {
                int delta = answer_sample - calls[c].timing.start_sample;
                int proximity;
                int score;

                if (delta < -400 || delta > 4400)
                    continue;
                proximity = abs(delta) / 40; /* 5 ms per point */
                score = calls[c].j_table_match_pct
                      + 2 * calls[c].timing.jprime_match_pct
                      + 2 * calls[c].timing.trn_recurrence_match_pct
                      - 2 * calls[c].timing.trn_descrambled_ber_pct
                      + 3 * s_pct + 4 * sbar_pct
                      + trn_pct - dber_pct
                      - proximity;
                if (score > best_score) {
                    best_score = score;
                    best_call = calls[c].timing;
                    memset(&best_answer, 0, sizeof(best_answer));
                    best_answer.found = true;
                    best_answer.source_calling_party = false;
                    best_answer.start_symbol = start;
                    best_answer.start_sample = answer_sample;
                    best_answer.trn_start_symbol = start + 144;
                    best_answer.trn_start_sample =
                        answer_source->symbols[start + 144].sample_index;
                    best_answer.s_match_pct = s_pct;
                    best_answer.sbar_match_pct = sbar_pct;
                    best_answer.trn_recurrence_match_pct = trn_pct;
                    best_answer.trn_descrambled_ber_pct = dber_pct;
                }
            }
        }
    }
    free(calls);
    if (getenv("P3_MATCH_STATS")) {
        fprintf(stderr,
                "[P3STEREO] J=%d J48=%d best_Jprime=%d%% best_call_TRN=%d%% fallback_answer=%d fallback_Jprime=%d fallback_prefix=%d call_candidates=%d answer_patterns=%d nearby_patterns=%d found=%s score=%d\n",
                j_segments,
                j48_segments,
                best_jprime_pct,
                best_call_trn_pct,
                fallback_answer_count,
                fallback_jprime_count,
                fallback_prefix_count,
                call_count,
                answer_pattern_count,
                nearby_pattern_count,
                best_score == INT_MIN ? "no" : "yes",
                best_score);
    }
    if (best_score == INT_MIN)
        return false;
    *call_out = best_call;
    *answer_out = best_answer;
    if (score_out)
        *score_out = best_score;
    return true;
}

static float coarse_s6_repeat_score(const p3_symbol_t *syms, int n)
{
    int checks = 0;
    int matches = 0;

    if (!syms || n < 12)
        return 0.0f;
    for (int i = 6; i < n; i++) {
        checks++;
        if (syms[i].dibit == syms[i - 6].dibit)
            matches++;
    }
    return (checks > 0) ? ((float) matches / (float) checks) : 0.0f;
}

static float coarse_ru_repeat_score(const p3_symbol_t *syms, int n)
{
    int best_match = 0;
    int best_checks = 0;

    if (!syms || n < 18)
        return 0.0f;

    for (int phase = 0; phase < 6; phase++) {
        int checks = 0;
        int match_normal = 0;
        int match_complement = 0;
        for (int i = phase; i < n; i++) {
            int d = syms[i].dibit;
            int pos = (i - phase) % 6;
            bool first_half = (pos < 3);
            checks++;
            if (first_half) {
                if (d == 0) match_normal++;
                if (d == 2) match_complement++;
            } else {
                if (d == 2) match_normal++;
                if (d == 0) match_complement++;
            }
        }
        if (checks > 0) {
            int match = (match_normal > match_complement) ? match_normal : match_complement;
            if (match > best_match) {
                best_match = match;
                best_checks = checks;
            }
        }
    }
    return (best_checks > 0) ? ((float) best_match / (float) best_checks) : 0.0f;
}

static float coarse_trn_recurrence_score(const p3_symbol_t *syms, int n)
{
    int checks = 0;
    int errors;

    if (!syms || n < 24)
        return 0.0f;
    errors = trn_recurrence_errors(syms, 0, n, &checks, NULL);
    return (checks > 0) ? ((float) (checks - errors) / (float) checks) : 0.0f;
}

static float coarse_j16_periodicity_score(const p3_symbol_t *syms, int n)
{
    int total_bits;
    int best_matches = 0;
    int best_checks = 0;

    if (!syms || n < 16)
        return 0.0f;
    total_bits = n * 2;
    if (total_bits < 64)
        return 0.0f;

    for (int phase = 0; phase < 16; phase++) {
        uint16_t pat = 0;
        int checks = 0;
        int matches = 0;
        if (phase + 16 > total_bits)
            break;
        for (int b = 0; b < 16; b++) {
            int bit = symbol_descrambled_bit(syms, 0, phase + b);
            pat |= (uint16_t) (bit & 1) << b;
        }
        for (int b = phase; b < total_bits; b++) {
            int bit = symbol_descrambled_bit(syms, 0, b);
            int expect = (int) ((pat >> ((b - phase) % 16)) & 1);
            checks++;
            if (bit == expect)
                matches++;
        }
        if (matches > best_matches || (matches == best_matches && checks > best_checks)) {
            best_matches = matches;
            best_checks = checks;
        }
    }
    return (best_checks > 0) ? ((float) best_matches / (float) best_checks) : 0.0f;
}

static float score_result(const p3_result_t *result);

/* ------------------------------------------------------------------ */
/*  Convenience: run full demodulation on a sample range              */
/* ------------------------------------------------------------------ */

static p3_result_t *p3_demod_run_internal(const int16_t *samples,
                                          int sample_count,
                                          int sample_offset,
                                          int baud_code,
                                          int carrier_sel,
                                          int sample_rate,
                                          int preferred_phase4_role,
                                          int trials_override)
{
    p3_result_t *best_result = NULL;
    float best_score = -FLT_MAX;
    int best_timing_score = INT_MIN;
    int trials;
    int est_symbols;

    if (!samples || sample_count <= 0)
        return NULL;

    /* Estimate max symbols: samples / (samples_per_symbol) + margin */
    est_symbols = (int)((float)sample_count * 2.0f) + 100;

    /* Reduce trials on long inputs to avoid excessive runtime, but never
     * drop to a single trial: this demodulator has no closed-loop symbol
     * timing recovery (baud_phase advances open-loop from the initial
     * strobe offset), so the phase-strobe sweep is what finds a workable
     * lock at all, not just a refinement for short windows. Dropping to
     * trials=1 above 40000 samples used to make the one-shot full-length
     * demod in promote_v34_phases_from_p3() a coin flip on the correct
     * phase (verified: 3429-baud Phase 4 J/TRN on conexant-rh56sp locks
     * at trials=2 and is silently lost at trials=1, with an identical
     * start sample and hypothesis both times). */
    if (trials_override > 0)
        trials = trials_override;
    else if (sample_count > 16000)
        trials = 2;
    else
        trials = 4;

    for (int t = 0; t < trials; t++) {
        p3_demod_t demod;
        p3_result_t *result;
        float s;
        int timing_score = INT_MIN;

        result = p3_result_alloc(est_symbols, est_symbols / 4 + 16);
        if (!result)
            continue;

        p3_demod_init(&demod, baud_code, carrier_sel, sample_rate);
        /* Sweep initial strobe phase to reduce aliasing to a bad symbol cut. */
        demod.baud_phase = ((float) t / (float) trials) * demod.samples_per_symbol;
        demod.rrc_step = (t * P3_RRC_PHASES) / trials;
        p3_demod_process(&demod, samples, sample_count, sample_offset, result);
        p3_segment_symbols(result);

        result->carrier_freq_estimate = demod.carrier_hz + demod.pll_freq_err;
        result->baud_rate_estimate = demod.baud_rate;
        result->locked = (demod.magnitude_count > 24);
        if (demod.magnitude_count > 0 && demod.magnitude_sum > 0.0f) {
            float avg_mag = demod.magnitude_sum / (float)demod.magnitude_count;
            /* Very rough SNR estimate from magnitude variance */
            result->snr_estimate_db = 20.0f * log10f(avg_mag + 1e-10f);
        }

        s = score_result(result);
        if (preferred_phase4_role == 1) {
            p3_phase4_timing_quality_t timing;

            if (p3_find_answer_phase4_timing(result, &timing)) {
                timing_score = timing.s_match_pct * 3
                             + timing.sbar_match_pct * 4
                             + timing.trn_recurrence_match_pct
                             - timing.trn_descrambled_ber_pct;
            }
        } else if (preferred_phase4_role == 2) {
            for (int i = 0; i < result->segment_count; i++) {
                const p3_segment_t *seg = &result->segments[i];
                p3_phase4_timing_quality_t timing;
                int candidate_score;

                if (seg->type != P3_SIGNAL_J || seg->length < 48)
                    continue;
                if (!p3_find_phase4_timing(result,
                                           seg->start_symbol,
                                           seg->length,
                                           seg->j_table_transform,
                                           true,
                                           &timing)) {
                    continue;
                }
                candidate_score = seg->j_table_match_pct
                                + 2 * timing.jprime_match_pct
                                + 2 * timing.trn_recurrence_match_pct
                                - 2 * timing.trn_descrambled_ber_pct;
                if (candidate_score > timing_score)
                    timing_score = candidate_score;
            }
        }
        if (!best_result
            || timing_score > best_timing_score
            || (timing_score == best_timing_score && s > best_score)) {
            if (best_result)
                p3_result_free(best_result);
            best_result = result;
            best_score = s;
            best_timing_score = timing_score;
        } else {
            p3_result_free(result);
        }
    }

    return best_result;
}

p3_result_t *p3_demod_run(const int16_t *samples,
                          int sample_count,
                          int sample_offset,
                          int baud_code,
                          int carrier_sel,
                          int sample_rate)
{
    return p3_demod_run_internal(samples,
                                 sample_count,
                                 sample_offset,
                                 baud_code,
                                 carrier_sel,
                                 sample_rate,
                                 0,
                                 0);
}

p3_result_t *p3_demod_run_answer_phase4(const int16_t *samples,
                                        int sample_count,
                                        int sample_offset,
                                        int baud_code,
                                        int carrier_sel,
                                        int sample_rate)
{
    return p3_demod_run_internal(samples,
                                 sample_count,
                                 sample_offset,
                                 baud_code,
                                 carrier_sel,
                                 sample_rate,
                                 1,
                                 0);
}

p3_result_t *p3_demod_run_call_phase4(const int16_t *samples,
                                      int sample_count,
                                      int sample_offset,
                                      int baud_code,
                                      int carrier_sel,
                                      int sample_rate)
{
    return p3_demod_run_internal(samples,
                                 sample_count,
                                 sample_offset,
                                 baud_code,
                                 carrier_sel,
                                 sample_rate,
                                 2,
                                 0);
}

p3_result_t *p3_demod_run_phase4_trials(const int16_t *samples,
                                        int sample_count,
                                        int sample_offset,
                                        int baud_code,
                                        int carrier_sel,
                                        int sample_rate,
                                        bool source_calling_party,
                                        int timing_trials)
{
    return p3_demod_run_internal(samples,
                                 sample_count,
                                 sample_offset,
                                 baud_code,
                                 carrier_sel,
                                 sample_rate,
                                 source_calling_party ? 2 : 1,
                                 timing_trials);
}

/* ------------------------------------------------------------------ */
/*  Multi-hypothesis scan                                             */
/* ------------------------------------------------------------------ */

static float score_result(const p3_result_t *result)
{
    float score = 0.0f;
    int unknown_symbols = 0;

    if (!result)
        return -1000.0f;

    for (int i = 0; i < result->segment_count; i++) {
        const p3_segment_t *seg = &result->segments[i];
        float seg_score = seg->confidence * (float)seg->length;

        switch (seg->type) {
        case P3_SIGNAL_S:
        case P3_SIGNAL_S_BAR:
            if (seg->length >= 18)
                score += seg_score * 1.7f;
            break;
        case P3_SIGNAL_TRN:
            if (seg->length >= 24)
                score += seg_score * 1.5f;
            break;
        case P3_SIGNAL_J:
            if (seg->length >= 64)
                score += seg_score * 3.0f;
            break;
        case P3_SIGNAL_RU:
        case P3_SIGNAL_UR:
            if (seg->length >= 18)
                score += seg_score * 2.0f;
            break;
        case P3_SIGNAL_PP:
            if (seg->length >= 2 * P3_PP_PERIOD_SYMBOLS)
                score += seg_score * 1.2f;
            break;
        case P3_SIGNAL_SILENCE:
            break;
        case P3_SIGNAL_UNKNOWN:
            unknown_symbols += seg->length;
            break;
        case P3_SIGNAL_J_PRIME:
            break;
        }
    }
    score -= 0.20f * (float) result->segment_count;
    score -= 0.03f * (float) unknown_symbols;
    return score;
}

/* Find the start and end of the main energy region, skipping silence.
 * Returns false if no significant energy found. */
static bool find_active_region(const int16_t *samples, int sample_count,
                               int *start_out, int *end_out)
{
    int window = 800;  /* 100 ms at 8000 Hz */
    float threshold;
    float peak_energy = 0.0f;
    int active_start = -1;
    int active_end = -1;

    if (!samples || sample_count < window)
        return false;

    /* Find peak energy window */
    for (int i = 0; i + window <= sample_count; i += window) {
        float energy = 0.0f;
        for (int j = 0; j < window; j++) {
            float s = (float)samples[i + j];
            energy += s * s;
        }
        energy /= (float)window;
        if (energy > peak_energy)
            peak_energy = energy;
    }

    /* Threshold at 1% of peak (about -20 dB below peak) */
    threshold = peak_energy * 0.01f;
    if (threshold < 100.0f)
        threshold = 100.0f;

    /* Find first and last active window */
    for (int i = 0; i + window <= sample_count; i += window) {
        float energy = 0.0f;
        for (int j = 0; j < window; j++) {
            float s = (float)samples[i + j];
            energy += s * s;
        }
        energy /= (float)window;
        if (energy >= threshold) {
            if (active_start < 0)
                active_start = i;
            active_end = i + window;
        }
    }

    if (active_start < 0)
        return false;

    *start_out = active_start;
    *end_out = (active_end > sample_count) ? sample_count : active_end;
    return true;
}

int p3_scan_all_hypotheses(const int16_t *samples,
                           int sample_count,
                           int sample_offset,
                           int sample_rate,
                           p3_hypothesis_t *hypotheses,
                           int max_hypotheses)
{
    int count = 0;
    int scan_start = 0;
    int scan_len = sample_count;

    if (!samples || sample_count <= 0 || !hypotheses || max_hypotheses <= 0)
        return 0;

    /* Narrow to active energy region on long scans to avoid demodulating
     * large silence spans. For already-targeted Phase 3 windows, keep the
     * full window so ordering/pattern context is preserved. */
    if (sample_count > 12000) {
        int active_start, active_end;
        if (find_active_region(samples, sample_count, &active_start, &active_end)) {
            scan_start = active_start;
            scan_len = active_end - active_start;
        }
    }

    /* Cap the window used for full per-hypothesis demodulation. Real
     * captures show Phase 3 (S/!S/MD/S/!S/PP/TRN/J) commonly running past
     * 5 seconds after INFO1 (e.g. banksia-wavesp336 R lands its S/TRN/J
     * sequence at 4566ms post-INFO1, right at the old cap); 12 seconds keeps
     * runtime bounded while covering the observed range.
     *
     * The candidate-ranking pre-score below stays anchored to the original
     * 5-second sub-window regardless of this cap: widening the full-demod
     * cap must not shift which baud/carrier hypotheses get ranked highest,
     * or short captures that previously matched inside 5s can lose their
     * hypothesis to a worse-ranked candidate. */
    if (scan_len > 12 * sample_rate)
        scan_len = 12 * sample_rate;

    /* Quick pre-score: correlate a short window with each carrier.
     * For short Phase 3 windows we evaluate all hypotheses; for long
     * fallback scans we cap the number of full demod runs. */
    {
        float pre_scores[P3_BAUD_COUNT * 2];
        int order[P3_BAUD_COUNT * 2];
        int n_hyp = 0;
        int candidates_to_run;
        int prescore_cap = 5 * sample_rate;
        int prescore_len = (scan_len < prescore_cap) ? scan_len : prescore_cap;
        /* Use a 2000-sample window from the middle of the (unwidened)
         * pre-score region */
        int pre_len = (prescore_len < 2000) ? prescore_len : 2000;
        int pre_start = scan_start + (prescore_len - pre_len) / 2;
        const int16_t *pre_samples = samples + pre_start;

        nco_init_tables();
        for (int baud = 0; baud < P3_BAUD_COUNT; baud++) {
            for (int carrier = 0; carrier <= 1; carrier++) {
                p3_baud_params_t bp;
                float carrier_hz;
                uint32_t phase = 0;
                uint32_t phase_inc;
                float power = 0.0f;

                if (!p3_get_baud_params(baud, &bp)) {
                    pre_scores[n_hyp++] = 0.0f;
                    continue;
                }
                carrier_hz = (carrier == P3_CARRIER_HIGH) ? bp.carrier_high_hz : bp.carrier_low_hz;
                phase_inc = hz_to_phase_inc(carrier_hz, sample_rate);

                /* Correlate: measure power at this carrier frequency */
                {
                    float acc_re = 0.0f, acc_im = 0.0f;
                    for (int i = 0; i < pre_len; i++) {
                        float s = (float)pre_samples[i];
                        float c, sn;
                        nco_get(phase, &c, &sn);
                        phase += phase_inc;
                        acc_re += s * c;
                        acc_im += s * (-sn);
                    }
                    power = (acc_re * acc_re + acc_im * acc_im) / ((float)pre_len * (float)pre_len);
                }
                pre_scores[n_hyp++] = power;
            }
        }

        for (int i = 0; i < n_hyp; i++)
            order[i] = i;

        /* Sort candidate indices by descending pre-score (small N=12). */
        for (int i = 0; i < n_hyp; i++) {
            for (int j = i + 1; j < n_hyp; j++) {
                if (pre_scores[order[j]] > pre_scores[order[i]]) {
                    int tmp = order[i];
                    order[i] = order[j];
                    order[j] = tmp;
                }
            }
        }

        if (scan_len <= 12000)
            candidates_to_run = n_hyp;          /* targeted phase-3 window */
        else if (scan_len <= 30000)
            candidates_to_run = (n_hyp < 8) ? n_hyp : 8;
        else
            candidates_to_run = (n_hyp < 4) ? n_hyp : 4;

        /* Fully demodulate ranked candidates. */
        for (int ti = 0; ti < candidates_to_run && count < max_hypotheses; ti++) {
            p3_result_t *result;
            p3_hypothesis_t *h;
            int idx;
            int baud;
            int carrier;

            idx = order[ti];
            baud = idx / 2;
            carrier = idx % 2;

            result = p3_demod_run(samples + scan_start, scan_len,
                                  sample_offset + scan_start,
                                  baud, carrier, sample_rate);
            if (!result)
                continue;

            h = &hypotheses[count++];
            memset(h, 0, sizeof(*h));
            h->baud_code = baud;
            h->carrier_sel = carrier;
            h->carrier_hz = result->carrier_freq_estimate;
            h->baud_rate = result->baud_rate_estimate;
            h->symbol_count = result->symbol_count;
            h->segment_count = result->segment_count;
            h->score = score_result(result);

            {
                float s6_score = coarse_s6_repeat_score(result->symbols, result->symbol_count);
                float trn_score = coarse_trn_recurrence_score(result->symbols, result->symbol_count);
                float ru_score = coarse_ru_repeat_score(result->symbols, result->symbol_count);
                float j16_score = coarse_j16_periodicity_score(result->symbols, result->symbol_count);

                if (s6_score >= 0.72f) h->has_s = true;
                if (trn_score >= 0.74f) h->has_trn = true;
                if (j16_score >= 0.78f) h->has_j = true;
                if (ru_score >= 0.72f) h->has_ru = true;

                /* Add weak-evidence scoring so we can still surface
                 * candidate hypotheses when strict segmentation fails. */
                h->score += 20.0f*s6_score + 22.0f*trn_score + 18.0f*ru_score + 24.0f*j16_score;
            }

            for (int i = 0; i < result->segment_count; i++) {
                switch (result->segments[i].type) {
                case P3_SIGNAL_S:
                case P3_SIGNAL_S_BAR:
                    h->has_s = true;
                    break;
                case P3_SIGNAL_TRN:
                    h->has_trn = true;
                    break;
                case P3_SIGNAL_J:
                    h->has_j = true;
                    break;
                case P3_SIGNAL_RU:
                case P3_SIGNAL_UR:
                    h->has_ru = true;
                    break;
                default:
                    break;
                }
            }

            p3_result_free(result);
        }
    }

    return count;
}

/* ------------------------------------------------------------------ */
/*  Utilities                                                         */
/* ------------------------------------------------------------------ */

const char *p3_signal_type_name(p3_signal_type_t type)
{
    switch (type) {
    case P3_SIGNAL_UNKNOWN: return "unknown";
    case P3_SIGNAL_SILENCE: return "silence";
    case P3_SIGNAL_S:       return "S";
    case P3_SIGNAL_S_BAR:   return "S-bar";
    case P3_SIGNAL_PP:      return "PP";
    case P3_SIGNAL_TRN:     return "TRN";
    case P3_SIGNAL_J:       return "J";
    case P3_SIGNAL_J_PRIME: return "J'";
    case P3_SIGNAL_RU:      return "Ru";
    case P3_SIGNAL_UR:      return "uR";
    }
    return "?";
}
