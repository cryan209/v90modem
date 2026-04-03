/*
 * p3_demod.c — Lightweight Phase 3 demodulator for V.34/V.90/V.92
 *
 * Carrier recovery, symbol timing, differential PSK demodulation,
 * descrambling, and pattern detection for offline modem analysis.
 */

#include "p3_demod.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

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
                        int sample_idx, p3_result_t *result)
{
    p3_symbol_t *sym;
    float mag;
    float scaled_re, scaled_im;
    int dibit;
    int bit0, bit1;

    if (!result || result->symbol_count >= result->symbol_capacity)
        return;

    /* AGC */
    mag = sqrtf(bb_re * bb_re + bb_im * bb_im);
    if (mag > 1e-10f) {
        float err = d->agc_target - mag * d->agc_gain;
        d->agc_gain += 0.005f * err;
        if (d->agc_gain < 1e-10f)
            d->agc_gain = 1e-10f;
    }
    scaled_re = bb_re * d->agc_gain;
    scaled_im = bb_im * d->agc_gain;
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

    /* Store symbol */
    sym = &result->symbols[result->symbol_count++];
    sym->sample_index = sample_idx;
    sym->re = scaled_re;
    sym->im = scaled_im;
    sym->magnitude = mag;
    sym->phase = atan2f(scaled_im, scaled_re);
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

int p3_demod_process(p3_demod_t *d,
                     const int16_t *samples,
                     int sample_count,
                     int sample_offset,
                     p3_result_t *result)
{
    int start_count;
    float spb;

    if (!d || !samples || sample_count <= 0 || !result)
        return 0;

    start_count = result->symbol_count;
    spb = d->samples_per_symbol;

    for (int i = 0; i < sample_count; i++) {
        float sample = (float)samples[i];
        float cos_val, sin_val;

        /* Mix down to baseband */
        nco_get(d->nco_phase, &cos_val, &sin_val);
        d->nco_phase += d->nco_phase_inc;

        d->idum_re += sample * cos_val;
        d->idum_im += sample * (-sin_val);
        d->idum_count++;

        /* Check for baud strobe */
        d->baud_phase += 1.0f;
        if (d->baud_phase >= spb) {
            d->baud_phase -= spb;

            /* Normalize integrate-and-dump */
            if (d->idum_count > 0) {
                float norm = 1.0f / (float)d->idum_count;
                emit_symbol(d,
                            d->idum_re * norm,
                            d->idum_im * norm,
                            sample_offset + i,
                            result);
            }

            /* Reset accumulator */
            d->idum_re = 0.0f;
            d->idum_im = 0.0f;
            d->idum_count = 0;
        }
    }

    return result->symbol_count - start_count;
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
    int period = 6;

    if (len < 12)
        return false;

    /* Check consistency of 6-symbol repeat */
    for (int i = period; i < len; i++) {
        if (syms[start + i].dibit == syms[start + i % period].dibit)
            matches++;
    }
    if (matches * 5 < (len - period) * 4)  /* Need 80% match */
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

static int symbol_scrambled_bit(const p3_symbol_t *syms, int start, int bit_pos)
{
    int sym_idx = bit_pos / 2;
    int bit_sel = bit_pos & 1;
    int dibit = syms[start + sym_idx].dibit;

    return bit_sel ? ((dibit >> 1) & 1) : (dibit & 1);
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
static int trn_recurrence_errors(const p3_symbol_t *syms, int start, int len, int *checks_out)
{
    int total_bits;
    int checks = 0;
    int errors = 0;

    if (!syms || len <= 0) {
        if (checks_out)
            *checks_out = 0;
        return 0;
    }

    total_bits = len * 2;
    for (int b = 23; b < total_bits; b++) {
        int prev23 = symbol_scrambled_bit(syms, start, b - 23);
        int prev5 = symbol_scrambled_bit(syms, start, b - 5);
        int expect = 1 ^ prev23 ^ prev5;
        int got = symbol_scrambled_bit(syms, start, b);
        checks++;
        if (got != expect)
            errors++;
    }
    if (checks_out)
        *checks_out = checks;
    return errors;
}

/* Check if a run of symbols looks like TRN (scrambled ones) */
static bool is_trn_pattern(const p3_symbol_t *syms, int start, int len)
{
    int checks;
    int errors;
    int descr_ones = 0;
    int descr_bits = 0;

    if (len < 24)
        return false;

    errors = trn_recurrence_errors(syms, start, len, &checks);
    if (checks >= 24) {
        /* Recurrence match >= 78% */
        if ((checks - errors) * 100 >= checks * 78)
            return true;
    }

    /* Fallback when recurrence window is short/noisy */
    for (int i = 0; i < len; i++) {
        if (syms[start + i].bit0 == 1)
            descr_ones++;
        if (syms[start + i].bit1 == 1)
            descr_ones++;
        descr_bits += 2;
    }
    return (descr_ones * 100 >= descr_bits * 80);
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
                             uint16_t *trn16, int *hypothesis)
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

    if (trn16) *trn16 = best_pat;
    if (hypothesis) *hypothesis = best_phase;
    return true;
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
        uint16_t j_trn16 = 0;
        int j_hyp = 0;

        /* Skip silence */
        if (is_silence(syms, pos, (remaining < 6) ? remaining : 6,
                       silence_threshold)) {
            int end = pos;
            while (end < n && syms[end].magnitude < silence_threshold)
                end++;
            add_segment(result, P3_SIGNAL_SILENCE, pos, end - pos, syms);
            pos = end;
            continue;
        }

        /* Try to identify the signal type.
         * Test patterns from most specific to least specific. */

        /* Check for Ru/uR (V.92, 6-symbol 2-point pattern) */
        if (remaining >= 18) {
            int end = pos;
            while (end < n && is_ru_pattern(syms, pos, end - pos + 6,
                                            &ru_positive)
                   && end + 6 <= n)
                end += 6;
            if (end - pos >= 18) {
                p3_segment_t *seg;
                add_segment(result,
                            ru_positive ? P3_SIGNAL_RU : P3_SIGNAL_UR,
                            pos, end - pos, syms);
                seg = &result->segments[result->segment_count - 1];
                seg->ru_positive_first = ru_positive;
                seg->confidence = 0.9f;
                pos = end;
                continue;
            }
        }

        /* Check for S / S-bar (6-symbol repeating) */
        if (remaining >= 12 && is_s_pattern(syms, pos, remaining,
                                            &s_complement)) {
            /* Find extent of S pattern */
            seg_len = 12;
            while (pos + seg_len + 6 <= n
                   && is_s_pattern(syms, pos, seg_len + 6, NULL))
                seg_len += 6;
            add_segment(result,
                        s_complement ? P3_SIGNAL_S_BAR : P3_SIGNAL_S,
                        pos, seg_len, syms);
            result->segments[result->segment_count - 1].confidence = 0.85f;
            pos += seg_len;
            continue;
        }

        /* Check for TRN (scrambled ones) */
        if (remaining >= 24 && is_trn_pattern(syms, pos, remaining)) {
            seg_len = remaining;
            for (int try_len = 24; try_len + 12 <= remaining; try_len += 12) {
                if (!is_trn_pattern(syms, pos, try_len + 12))
                    break;
                seg_len = try_len + 12;
            }
            {
                int checks = 0;
                int error_bits = trn_recurrence_errors(syms, pos, seg_len, &checks);
                float conf;
                add_segment(result, P3_SIGNAL_TRN, pos, seg_len, syms);
                result->segments[result->segment_count - 1].trn_errors = error_bits;
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
            && detect_j_pattern(syms, pos, remaining, &j_trn16, &j_hyp)) {
            seg_len = remaining;
            /* Trim to where the J pattern fades */
            for (int try_len = 48; try_len + 8 <= remaining; try_len += 8) {
                if (!detect_j_pattern(syms, pos, try_len + 8, NULL, NULL))
                    break;
                seg_len = try_len + 8;
            }
            {
                p3_segment_t *seg;
                add_segment(result, P3_SIGNAL_J, pos, seg_len, syms);
                seg = &result->segments[result->segment_count - 1];
                seg->j_trn16 = j_trn16;
                seg->j_hypothesis = j_hyp;
                seg->confidence = 0.78f;
            }
            pos += seg_len;
            continue;
        }

        /* Unknown: advance by a small window */
        seg_len = (remaining < 6) ? remaining : 6;
        add_segment(result, P3_SIGNAL_UNKNOWN, pos, seg_len, syms);
        pos += seg_len;
    }

    return result->segment_count;
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
    errors = trn_recurrence_errors(syms, 0, n, &checks);
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

/* ------------------------------------------------------------------ */
/*  Convenience: run full demodulation on a sample range              */
/* ------------------------------------------------------------------ */

p3_result_t *p3_demod_run(const int16_t *samples,
                          int sample_count,
                          int sample_offset,
                          int baud_code,
                          int carrier_sel,
                          int sample_rate)
{
    p3_demod_t demod;
    p3_result_t *result;
    int est_symbols;

    if (!samples || sample_count <= 0)
        return NULL;

    /* Estimate max symbols: samples / (samples_per_symbol) + margin */
    est_symbols = (int)((float)sample_count * 2.0f) + 100;
    result = p3_result_alloc(est_symbols, est_symbols / 4 + 16);
    if (!result)
        return NULL;

    p3_demod_init(&demod, baud_code, carrier_sel, sample_rate);
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
    return result;
}

/* ------------------------------------------------------------------ */
/*  Multi-hypothesis scan                                             */
/* ------------------------------------------------------------------ */

static float score_result(const p3_result_t *result)
{
    float score = 0.0f;

    if (!result)
        return -1000.0f;

    for (int i = 0; i < result->segment_count; i++) {
        const p3_segment_t *seg = &result->segments[i];
        float seg_score = seg->confidence * (float)seg->length;

        switch (seg->type) {
        case P3_SIGNAL_S:
        case P3_SIGNAL_S_BAR:
            score += seg_score * 2.0f;
            break;
        case P3_SIGNAL_TRN:
            score += seg_score * 1.5f;
            break;
        case P3_SIGNAL_J:
            score += seg_score * 3.0f;
            break;
        case P3_SIGNAL_RU:
        case P3_SIGNAL_UR:
            score += seg_score * 2.5f;
            break;
        case P3_SIGNAL_SILENCE:
        case P3_SIGNAL_UNKNOWN:
        case P3_SIGNAL_PP:
        case P3_SIGNAL_J_PRIME:
            break;
        }
    }
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

    /* Quick pre-score: correlate a short window with each carrier.
     * For short Phase 3 windows we evaluate all hypotheses; for long
     * fallback scans we cap the number of full demod runs. */
    {
        float pre_scores[P3_BAUD_COUNT * 2];
        int order[P3_BAUD_COUNT * 2];
        int n_hyp = 0;
        int candidates_to_run;
        /* Use a 2000-sample window from the middle of the active region */
        int pre_len = (scan_len < 2000) ? scan_len : 2000;
        int pre_start = scan_start + (scan_len - pre_len) / 2;
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
