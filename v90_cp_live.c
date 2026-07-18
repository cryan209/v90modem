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

static int v90_cp_live_find_carrier_rise(const int16_t *samples,
                                         int search_start,
                                         int search_end)
{
    const int window = V90_CP_LIVE_SAMPLE_RATE / 10; /* 100 ms */
    int best = -1;

    if (!samples || search_end - search_start < 8 * window)
        return -1;
    if (search_start < 5 * window)
        search_start = 5 * window;
    for (int start = search_start;
         start + 5 * window <= search_end;
         start += window / 4) {
        double before = 0.0;
        double after = 0.0;

        for (int i = start - 5 * window; i < start; i++) {
            double sample = samples[i];
            before += sample * sample;
        }
        for (int i = start; i < start + 5 * window; i++) {
            double sample = samples[i];
            after += sample * sample;
        }
        before = sqrt(before / (5.0 * window));
        after = sqrt(after / (5.0 * window));
        double ratio = after / (before + 1.0);

        if (after > 200.0 && after - before > 250.0 && ratio >= 2.0) {
            /* Phase 3 contains an earlier J/primary-channel rise.  CPt is
             * the latest sustained rise near the local Ri marker. */
            best = start;
        }
    }
    return best;
}

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

/* CP/CP' is repeated until the other modem acknowledges it.  The GPA
 * scrambler is self-synchronising, so after its 23-bit memory the decoded
 * copies are periodic even when capture starts mid-stream.  Majority-vote a
 * bounded odd number of copies, then require a complete, semantically sane
 * Table-14 decode with its CRC intact.  The agreement gate makes an
 * accidental CRC hit under a wrong carrier/timing hypothesis vanishingly
 * unlikely. */
static bool v90_cp_live_decode_repeated(const uint8_t *bits,
                                        int bit_count,
                                        int expected_compatibility,
                                        bool expected_alaw,
                                        vpcm_cp_diag_t *best,
                                        int *best_offset,
                                        int *best_frames,
                                        int *best_agreement)
{
    uint8_t voted[VPCM_CP_MAX_BITS];
    bool found = false;
    bool found_ack = false;

    if (!bits || !best || !best_offset || !best_frames || !best_agreement)
        return false;
    for (int mask_blocks = 1;
         mask_blocks <= VPCM_CP_MAX_MASK_BLOCKS; mask_blocks++) {
        int period = 156 + 136 * mask_blocks;

        if (5 * period > bit_count)
            break;
        for (int phase = 0;
             phase < period && phase + 5 * period <= bit_count; phase++) {
            int available = (bit_count - phase) / period;

            for (int skip = 0; skip <= 4 && skip + 5 <= available; skip++) {
                int frames = available - skip;
                int sync_errors = 0;
                int disagreements = 0;
                int agreement;
                vpcm_cp_diag_t candidate;

                if (frames > 9)
                    frames = 9;
                if ((frames & 1) == 0)
                    frames--;
                for (int bit = 0; bit <= 17; bit++) {
                    int ones = 0;

                    for (int frame = 0; frame < frames; frame++) {
                        ones += bits[phase
                                  + (skip + frame) * period + bit] != 0;
                    }
                    if (bit <= 16)
                        sync_errors += 2 * ones < frames;
                    else
                        sync_errors += 2 * ones >= frames;
                }
                if (sync_errors != 0)
                    continue;
                {
                    int ones = 0;

                    for (int frame = 0; frame < frames; frame++) {
                        ones += bits[phase
                                  + (skip + frame) * period + 19] != 0;
                    }
                    if ((2 * ones >= frames ? 1 : 0)
                        != expected_compatibility) {
                        continue;
                    }
                }
                for (int bit = 0; bit < period; bit++) {
                    int ones = 0;

                    for (int frame = 0; frame < frames; frame++) {
                        ones += bits[phase
                                  + (skip + frame) * period + bit] != 0;
                    }
                    voted[bit] = (uint8_t)(2 * ones >= frames);
                    disagreements += ones < frames - ones
                                   ? ones : frames - ones;
                }
                agreement = 100
                          - (100 * disagreements + frames * period / 2)
                            / (frames * period);
                if (agreement < 75)
                    continue;
                memset(&candidate, 0, sizeof(candidate));
                if (!vpcm_cp_decode_diag(voted, period, &candidate)
                    || !v90_cp_live_frame_sane(&candidate,
                                                expected_compatibility,
                                                expected_alaw)) {
                    continue;
                }
                if (!found
                    || (candidate.frame.acknowledge && !found_ack)
                    || (candidate.frame.acknowledge == found_ack
                        && agreement > *best_agreement)) {
                    *best = candidate;
                    *best_offset = phase + skip * period;
                    *best_frames = frames;
                    *best_agreement = agreement;
                    found_ack = candidate.frame.acknowledge;
                    found = true;
                }
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
                                    bool *conjugate_out,
                                    int *map_index_out,
                                    int *bit_order_out,
                                    int *voted_frames_out,
                                    int *agreement_pct_out)
{
    static const uint8_t map_table[24][4] = {
        {0,1,2,3}, {1,0,3,2}, {2,3,0,1}, {3,2,1,0},
        {0,2,1,3}, {2,0,3,1}, {1,3,0,2}, {3,1,2,0},
        {0,3,2,1}, {1,2,3,0}, {2,1,0,3}, {3,0,1,2},
        {0,1,3,2}, {1,0,2,3}, {2,3,1,0}, {3,2,0,1},
        {0,2,3,1}, {1,3,2,0}, {2,0,1,3}, {3,1,0,2},
        {0,3,1,2}, {1,2,0,3}, {2,1,3,0}, {3,0,2,1}
    };
    static const float pll_gains[] = {0.01f, 0.0f, 0.05f};
    uint8_t *bits;
    int bit_capacity;
    bool found = false;
    bool found_ack = false;
    bool found_voted = false;
    int found_offset = -1;
    int map_begin = 8;
    int map_end = 8;
    int order_begin = 0;
    int order_end = 0;
    int carrier_step_begin = -256;
    int carrier_step_end = 256;

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
    if (getenv("ME_V90_CP_BROAD_MAP")) {
        map_begin = 0;
        map_end = 23;
        order_end = 1;
    }
    if (getenv("ME_V90_CP_MAP")) {
        map_begin = atoi(getenv("ME_V90_CP_MAP"));
        if (map_begin < 0)
            map_begin = 0;
        if (map_begin > 23)
            map_begin = 23;
        map_end = map_begin;
    }
    if (getenv("ME_V90_CP_ORDER")) {
        order_begin = atoi(getenv("ME_V90_CP_ORDER")) ? 1 : 0;
        order_end = order_begin;
    }
    if (getenv("ME_V90_CP_CARRIER_STEP")) {
        carrier_step_begin = atoi(getenv("ME_V90_CP_CARRIER_STEP"));
        carrier_step_end = carrier_step_begin;
    }

    /* The offline capture's strict winner is -80.  Search a wider residual
     * carrier range so clock offsets do not pin acquisition to a boundary. */
    for (int conjugate = 0; conjugate < 2; conjugate++) {
        for (int carrier_step = carrier_step_begin;
             carrier_step <= carrier_step_end;
             carrier_step += 2) {
            float carrier_delta = carrier_step * (float)(M_PI / 2048.0);

            for (size_t pll_index = 0;
                 pll_index < sizeof(pll_gains) / sizeof(pll_gains[0]);
                 pll_index++) {
              for (int map = map_begin; map <= map_end; map++) {
               for (int order = order_begin; order <= order_end; order++) {
                uint32_t reg = 0;
                float tracked_delta = carrier_delta;
                int bit_count = 0;
                vpcm_cp_diag_t candidate;
                int candidate_offset = -1;
                int candidate_frames = 0;
                int candidate_agreement = 100;
                bool candidate_voted = false;

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
                    mapped = map_table[map][quadrant];
                    bits[bit_count++] = (uint8_t)v90_cp_live_descramble(
                        &reg, order ? (mapped >> 1) & 1 : mapped & 1);
                    bits[bit_count++] = (uint8_t)v90_cp_live_descramble(
                        &reg, order ? mapped & 1 : (mapped >> 1) & 1);
                }

                memset(&candidate, 0, sizeof(candidate));
                if (!v90_cp_live_decode_bits(bits,
                                             bit_count,
                                             expected_compatibility,
                                             expected_alaw,
                                             &candidate,
                                             &candidate_offset)) {
                    candidate_voted = v90_cp_live_decode_repeated(
                        bits,
                        bit_count,
                        expected_compatibility,
                        expected_alaw,
                        &candidate,
                        &candidate_offset,
                        &candidate_frames,
                        &candidate_agreement);
                }
                if (candidate.valid) {
                    bool candidate_ack = candidate.frame.acknowledge;

                    if (!found
                        || (found_voted && !candidate_voted)
                        || (candidate_ack && !found_ack)
                        || (candidate_voted == found_voted
                            && candidate_ack == found_ack
                            && candidate_offset > found_offset)) {
                        *out = candidate;
                        found = true;
                        found_ack = candidate_ack;
                        found_voted = candidate_voted;
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
                        if (map_index_out)
                            *map_index_out = map;
                        if (bit_order_out)
                            *bit_order_out = order;
                        if (voted_frames_out)
                            *voted_frames_out = candidate_frames;
                        if (agreement_pct_out)
                            *agreement_pct_out = candidate_agreement;
                    }
                    if (!expected_compatibility)
                        goto done;
                }
               }
              }
            }
        }
    }
done:
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
    int equalizer_freeze_sample;
    int carrier_begin = P3_CARRIER_LOW;
    int carrier_end = P3_CARRIER_HIGH;
    int timing_begin = 0;
    int timing_end = V90_CP_LIVE_TIMINGS;
    int carrier_onset = -1;
    int baud_code = V90_CP_LIVE_BAUD_CODE;

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
    /* The two Phase-2 directions can finish several seconds apart after an
     * INFO retry.  SmartLink may therefore begin repeating CPt before our
     * downstream DIL->Ri marker.  Keep the marker as the training/freezing
     * anchor, but search the preceding eight seconds as well. */
    search_start = phase4_hint_sample - 8 * V90_CP_LIVE_SAMPLE_RATE;
    if (search_start < capture_start)
        search_start = capture_start;
    search_end = sample_count;
    equalizer_freeze_sample = phase4_hint_sample - 4000;
    if (!expected_compatibility) {
        int rise_end = phase4_hint_sample + V90_CP_LIVE_SAMPLE_RATE;
        if (rise_end > sample_count)
            rise_end = sample_count;
        carrier_onset = v90_cp_live_find_carrier_rise(samples,
                                                      search_start,
                                                      rise_end);
        if (carrier_onset >= 0) {
            /* SmartLink may start CPt several seconds before our Ri when the
             * two Phase-2 directions finish at different times.  Freezing
             * the CMA relative to the late local marker then freezes it in
             * the middle of CPt.  Anchor it just before the caller's own
             * sustained carrier rise instead. */
            search_start = carrier_onset - 800;
            if (search_start < capture_start)
                search_start = capture_start;
            /* Some SmartLink captures need several seconds of repeated CPt
             * before the equalizer yields an untouched frame.  Keep a
             * bounded eight-second train; live retries naturally provide a
             * shorter prefix until that much waveform exists. */
            search_end = carrier_onset + 8 * V90_CP_LIVE_SAMPLE_RATE;
            if (search_end > sample_count)
                search_end = sample_count;
        }
    }
    if (equalizer_freeze_sample < capture_start)
        equalizer_freeze_sample = capture_start;

    /* Diagnostic overrides keep offline waveform sweeps bounded.  Production
     * uses both carriers and all timing phases unless explicitly requested. */
    if (getenv("ME_V90_CP_FREEZE_SAMPLE"))
        equalizer_freeze_sample = atoi(getenv("ME_V90_CP_FREEZE_SAMPLE"));
    if (getenv("ME_V90_CP_CARRIER")) {
        carrier_begin = atoi(getenv("ME_V90_CP_CARRIER"))
                      ? P3_CARRIER_HIGH : P3_CARRIER_LOW;
        carrier_end = carrier_begin;
    }
    if (getenv("ME_V90_CP_TIMING")) {
        timing_begin = atoi(getenv("ME_V90_CP_TIMING"));
        if (timing_begin < 0)
            timing_begin = 0;
        if (timing_begin >= V90_CP_LIVE_TIMINGS)
            timing_begin = V90_CP_LIVE_TIMINGS - 1;
        timing_end = timing_begin + 1;
    }
    if (getenv("ME_V90_CP_BAUD_CODE")) {
        baud_code = atoi(getenv("ME_V90_CP_BAUD_CODE"));
        if (baud_code < P3_BAUD_2400 || baud_code >= P3_BAUD_COUNT)
            baud_code = V90_CP_LIVE_BAUD_CODE;
    }
    if (getenv("ME_V90_CP_DIAG")) {
        fprintf(stderr,
                "v90 CP live window capture=%d search=%d..%d hint=%d freeze=%d carrier=%d..%d timing=%d..%d\n",
                capture_start, search_start, search_end,
                phase4_hint_sample, equalizer_freeze_sample,
                carrier_begin, carrier_end, timing_begin, timing_end - 1);
    }

    for (int carrier = carrier_begin; carrier <= carrier_end; carrier++) {

        for (int timing = timing_begin; timing < timing_end; timing++) {
            p3_result_t *detail = p3_demod_run_phase4_data_at_timing(
                samples + capture_start,
                sample_count - capture_start,
                capture_start,
                baud_code,
                carrier,
                V90_CP_LIVE_SAMPLE_RATE,
                true,
                timing,
                V90_CP_LIVE_TIMINGS,
                equalizer_freeze_sample);
            int first_symbol = 1;
            int end_symbol;
            int frame_symbol = -1;
            int last_symbol = -1;
            int carrier_step = 0;
            float pll_gain = 0.0f;
            bool conjugate = false;
            int map_index = 8;
            int bit_order = 0;
            int voted_frames = 0;
            int agreement_pct = 100;
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
                                             &conjugate,
                                             &map_index,
                                             &bit_order,
                                             &voted_frames,
                                             &agreement_pct);
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
                    meta->map_index = map_index;
                    meta->bit_order = bit_order;
                    meta->voted_frames = voted_frames;
                    meta->agreement_pct = agreement_pct;
                }
                p3_result_free(detail);
                return true;
            }
            p3_result_free(detail);
        }
    }
    return false;
}
