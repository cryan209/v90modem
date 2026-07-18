/* Strict live V.90 Table-14 CP receiver over a buffered PCM waveform. */

#include "v90_cp_live.h"

#include "p3_demod.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define V90_CP_LIVE_SAMPLE_RATE 8000
#define V90_CP_LIVE_BAUD_CODE P3_BAUD_2400
#define V90_CP_LIVE_TIMINGS 16
#define V90_CP_LIVE_MIN_BITS 292

static int v90_cp_live_descramble(uint32_t *reg, int input)
{
    int output = (input ^ (*reg >> 4) ^ (*reg >> 22)) & 1;

    *reg = ((*reg << 1) | (uint32_t)(input & 1)) & 0x7FFFFFU;
    return output;
}

static bool v90_cp_live_frame_sane(const vpcm_cp_diag_t *diag,
                                   int expected_compatibility,
                                   bool expected_alaw)
{
    return diag
        && diag->valid
        && (diag->frame.v90_compatibility ? 1 : 0)
             == expected_compatibility
        && diag->frame.codec_alaw == expected_alaw
        && diag->frame.drn >= 1
        && diag->frame.drn <= 22
        && diag->frame.upstream_rate_mask != 0;
}

static bool v90_cp_live_decode_bits(const uint8_t *bits,
                                    int bit_count,
                                    int expected_compatibility,
                                    bool expected_alaw,
                                    vpcm_cp_diag_t *best,
                                    int *best_offset)
{
    bool found = false;
    bool best_ack = false;

    if (!bits || !best || !best_offset || bit_count < V90_CP_LIVE_MIN_BITS)
        return false;

    for (int offset = 24; offset + V90_CP_LIVE_MIN_BITS <= bit_count;
         offset++) {
        bool sync = bits[offset + 17] == 0;

        for (int bit = 0; bit <= 16 && sync; bit++)
            sync = bits[offset + bit] != 0;
        if (!sync)
            continue;

        for (int mask_blocks = 1;
             mask_blocks <= VPCM_CP_MAX_MASK_BLOCKS; mask_blocks++) {
            int period = 156 + 136 * mask_blocks;
            vpcm_cp_diag_t candidate;

            if (offset + period > bit_count)
                break;
            if (!vpcm_cp_decode_diag(bits + offset, period, &candidate)
                || !v90_cp_live_frame_sane(&candidate,
                                            expected_compatibility,
                                            expected_alaw)) {
                continue;
            }

            /* CP' supersedes CP.  Within the same kind retain the newest
             * copy so later stages get the closest observed boundary. */
            if (!found
                || (candidate.frame.acknowledge && !best_ack)
                || (candidate.frame.acknowledge == best_ack
                    && offset > *best_offset)) {
                *best = candidate;
                *best_offset = offset;
                best_ack = candidate.frame.acknowledge;
                found = true;
            }
        }
    }
    return found;
}

static bool v90_cp_live_slice_trial(const p3_result_t *detail,
                                    int first_symbol,
                                    int end_symbol,
                                    int expected_compatibility,
                                    bool expected_alaw,
                                    vpcm_cp_diag_t *out,
                                    int *frame_symbol,
                                    int *last_symbol,
                                    int *carrier_step_out,
                                    float *pll_gain_out,
                                    bool *conjugate_out)
{
    static const uint8_t cp_map[4] = {0, 3, 2, 1};
    static const float pll_gains[] = {0.01f, 0.0f, 0.05f};
    uint8_t *bits;
    int bit_capacity;
    bool found = false;
    bool found_ack = false;
    int found_offset = -1;

    if (!detail || !detail->symbols || !out || first_symbol < 1
        || end_symbol <= first_symbol || end_symbol > detail->symbol_count) {
        return false;
    }
    bit_capacity = 2 * (end_symbol - first_symbol);
    if (bit_capacity < V90_CP_LIVE_MIN_BITS)
        return false;
    bits = malloc((size_t)bit_capacity);
    if (!bits)
        return false;

    /* The offline capture's strict winner is -80.  Search a wider residual
     * carrier range so clock offsets do not pin acquisition to a boundary. */
    for (int conjugate = 0; conjugate < 2; conjugate++) {
        for (int carrier_step = -256; carrier_step <= 256;
             carrier_step += 2) {
            float carrier_delta = carrier_step * (float)(M_PI / 2048.0);

            for (size_t pll_index = 0;
                 pll_index < sizeof(pll_gains) / sizeof(pll_gains[0]);
                 pll_index++) {
                uint32_t reg = 0;
                float tracked_delta = carrier_delta;
                int bit_count = 0;
                vpcm_cp_diag_t candidate;
                int candidate_offset = -1;

                for (int sym = first_symbol; sym < end_symbol; sym++) {
                    float re = detail->symbols[sym].re;
                    float im = detail->symbols[sym].im;
                    float prev_re = detail->symbols[sym - 1].re;
                    float prev_im = detail->symbols[sym - 1].im;
                    float angle = atan2f(im * prev_re - re * prev_im,
                                         re * prev_re + im * prev_im);
                    float corrected;
                    float ideal;
                    float error;
                    int quadrant;
                    int mapped;

                    if (conjugate)
                        angle = -angle;
                    corrected = angle - tracked_delta;
                    quadrant = (int)floorf(
                        corrected / (float)(M_PI / 2.0) + 0.5f) & 3;
                    ideal = quadrant * (float)(M_PI / 2.0);
                    error = remainderf(corrected - ideal,
                                       (float)(2.0 * M_PI));
                    tracked_delta += pll_gains[pll_index] * error;
                    mapped = cp_map[quadrant];
                    bits[bit_count++] = (uint8_t)v90_cp_live_descramble(
                        &reg, mapped & 1);
                    bits[bit_count++] = (uint8_t)v90_cp_live_descramble(
                        &reg, (mapped >> 1) & 1);
                }

                memset(&candidate, 0, sizeof(candidate));
                if (v90_cp_live_decode_bits(bits,
                                            bit_count,
                                            expected_compatibility,
                                            expected_alaw,
                                            &candidate,
                                            &candidate_offset)) {
                    bool candidate_ack = candidate.frame.acknowledge;

                    if (!found
                        || (candidate_ack && !found_ack)
                        || (candidate_ack == found_ack
                            && candidate_offset > found_offset)) {
                        *out = candidate;
                        found = true;
                        found_ack = candidate_ack;
                        found_offset = candidate_offset;
                        if (frame_symbol)
                            *frame_symbol = first_symbol
                                          + candidate_offset / 2;
                        if (last_symbol)
                            *last_symbol = first_symbol
                                         + (candidate_offset
                                            + candidate.nbits - 1) / 2;
                        if (carrier_step_out)
                            *carrier_step_out = carrier_step;
                        if (pll_gain_out)
                            *pll_gain_out = pll_gains[pll_index];
                        if (conjugate_out)
                            *conjugate_out = conjugate != 0;
                    }
                }
            }
        }
    }
    free(bits);
    return found;
}

bool v90_cp_live_recover(const int16_t *samples,
                         int sample_count,
                         int phase4_hint_sample,
                         int expected_compatibility,
                         bool expected_alaw,
                         vpcm_cp_diag_t *out,
                         v90_cp_live_meta_t *meta)
{
    int capture_start;
    int search_start;
    int search_end;

    if (!samples || !out || sample_count <= 0
        || phase4_hint_sample < 0
        || phase4_hint_sample >= sample_count
        || (expected_compatibility != 0 && expected_compatibility != 1)) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    if (meta)
        memset(meta, 0, sizeof(*meta));

    /* Preserve enough Phase-3 history to train the RRC/CMA front end while
     * bounding a retry's CPU cost.  CPt on the SmartLink path begins close
     * to Ri; data CP/CP' can occur several seconds later. */
    capture_start = phase4_hint_sample - 14 * V90_CP_LIVE_SAMPLE_RATE;
    if (capture_start < 0)
        capture_start = 0;
    search_start = phase4_hint_sample - 4000;
    if (search_start < capture_start)
        search_start = capture_start;
    search_end = sample_count;

    for (int carrier_pass = 0; carrier_pass < 2; carrier_pass++) {
        int carrier = carrier_pass == 0 ? P3_CARRIER_LOW : P3_CARRIER_HIGH;

        for (int timing = 0; timing < V90_CP_LIVE_TIMINGS; timing++) {
            p3_result_t *detail = p3_demod_run_phase4_data_at_timing(
                samples + capture_start,
                sample_count - capture_start,
                capture_start,
                V90_CP_LIVE_BAUD_CODE,
                carrier,
                V90_CP_LIVE_SAMPLE_RATE,
                true,
                timing,
                V90_CP_LIVE_TIMINGS,
                phase4_hint_sample - 4000);
            int first_symbol = 1;
            int end_symbol;
            int frame_symbol = -1;
            int last_symbol = -1;
            int carrier_step = 0;
            float pll_gain = 0.0f;
            bool conjugate = false;
            bool found;

            if (!detail)
                continue;
            while (first_symbol < detail->symbol_count
                   && detail->symbols[first_symbol].sample_index
                        < search_start) {
                first_symbol++;
            }
            end_symbol = first_symbol;
            while (end_symbol < detail->symbol_count
                   && detail->symbols[end_symbol].sample_index < search_end) {
                end_symbol++;
            }
            found = v90_cp_live_slice_trial(detail,
                                             first_symbol,
                                             end_symbol,
                                             expected_compatibility,
                                             expected_alaw,
                                             out,
                                             &frame_symbol,
                                             &last_symbol,
                                             &carrier_step,
                                             &pll_gain,
                                             &conjugate);
            if (found) {
                if (meta) {
                    meta->frame_sample = detail->symbols[frame_symbol]
                                             .sample_index;
                    meta->last_sample = detail->symbols[last_symbol]
                                            .sample_index;
                    meta->carrier_sel = carrier;
                    meta->timing_index = timing;
                    meta->carrier_step = carrier_step;
                    meta->pll_gain = pll_gain;
                    meta->conjugate = conjugate;
                }
                p3_result_free(detail);
                return true;
            }
            p3_result_free(detail);
        }
    }
    return false;
}
