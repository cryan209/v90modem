/* Strict live V.90 Table-14 CP receiver over a buffered PCM waveform. */

#include "v90_cp_live.h"

#include "p3_demod.h"

#include <float.h>
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
#define V90_CP_DIRECT_RRC_HALF 27
#define V90_CP_DIRECT_RRC_ALPHA 0.10f

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
    /* The analogue modem reports the codec law it inferred during DIL.  A
     * SmartLink SL8200 occasionally reports A-law on an otherwise verified
     * PCMU call; keep the CRC-valid frame and let the engine reconcile that
     * field with the negotiated SIP law before configuring its mapper. */
    (void)expected_alaw;
    return diag
        && diag->valid
        && (diag->frame.v90_compatibility ? 1 : 0)
             == expected_compatibility
        && diag->frame.drn >= 1
        && diag->frame.drn <= 22
        && diag->frame.upstream_rate_mask != 0;
}

/* A first CPt is sometimes the only copy available when the analogue modem
 * immediately requests retrain.  Correct only bits whose values are fixed by
 * Table 14, then allow the CRC to identify one unique remaining uncertain
 * information/CRC bit.  Every accepted result still passes the complete
 * frame CRC, structural checks, and negotiated-field sanity checks. */
static bool v90_cp_live_repair_first_cpt(const uint8_t *observed,
                                         int period,
                                         int mask_blocks,
                                         int expected_compatibility,
                                         bool expected_alaw,
                                         vpcm_cp_diag_t *out,
                                         int *fixed_corrections,
                                         int *first_bit_correction,
                                         int *second_bit_correction)
{
    static const int fixed_zero_bits[] = {
        17, 18, 25, 26, 27, 28, 29, 30, 34, 51, 68,
        85, 102, 119, 129, 130, 131, 132, 133, 134, 135
    };
    uint8_t repaired[VPCM_CP_MAX_BITS];
    int raw_bits = period - 3;
    int corrections = 0;
    int solutions = 0;
    int solution_bit = -1;
    vpcm_cp_diag_t solution;

    if (fixed_corrections)
        *fixed_corrections = 0;
    if (first_bit_correction)
        *first_bit_correction = -1;
    if (second_bit_correction)
        *second_bit_correction = -1;
    if (!observed || !out || period < V90_CP_LIVE_MIN_BITS
        || period > VPCM_CP_MAX_BITS || mask_blocks < 1
        || mask_blocks > VPCM_CP_MAX_MASK_BLOCKS) {
        return false;
    }
    memcpy(repaired, observed, (size_t)period);
    for (int bit = 0; bit <= 16; bit++) {
        corrections += repaired[bit] == 0;
        repaired[bit] = 1;
    }
    for (size_t i = 0;
         i < sizeof(fixed_zero_bits) / sizeof(fixed_zero_bits[0]); i++) {
        int bit = fixed_zero_bits[i];

        corrections += repaired[bit] != 0;
        repaired[bit] = 0;
    }
    for (int block = 0; block < mask_blocks; block++) {
        int base = 136 + 136 * block;

        for (int chord = 0; chord < 8; chord++) {
            int bit = base + 17 * chord;

            corrections += repaired[bit] != 0;
            repaired[bit] = 0;
        }
    }
    corrections += repaired[136 + 136 * mask_blocks] != 0;
    repaired[136 + 136 * mask_blocks] = 0;
    for (int bit = raw_bits; bit < period; bit++) {
        corrections += repaired[bit] != 0;
        repaired[bit] = 0;
    }
    if (fixed_corrections)
        *fixed_corrections = corrections;
    if (corrections > 4)
        return false;

    if (vpcm_cp_decode_diag(repaired, period, out)
        && v90_cp_live_frame_sane(out,
                                  expected_compatibility,
                                  expected_alaw)) {
        return true;
    }
    for (int bit = 18; bit < raw_bits; bit++) {
        bool start_bit = bit == 34 || bit == 51 || bit == 68
                      || bit == 85 || bit == 102 || bit == 119
                      || (bit >= 136 && ((bit - 136) % 17) == 0);
        vpcm_cp_diag_t trial;

        if (start_bit)
            continue;
        repaired[bit] ^= 1;
        memset(&trial, 0, sizeof(trial));
        if (vpcm_cp_decode_diag(repaired, period, &trial)
            && v90_cp_live_frame_sane(&trial,
                                      expected_compatibility,
                                      expected_alaw)) {
            solution = trial;
            solution_bit = bit;
            solutions++;
        }
        repaired[bit] ^= 1;
    }
    if (solutions != 1)
    {
        /* A 428-bit first CPt from the SmartLink path can retain two
         * uncertain payload/CRC decisions under simultaneous Ri echo.  Use
         * CRC syndrome linearity to enumerate only pairs which can possibly
         * close the received CRC, then retain a result only when the complete
         * Table-14 decode has one unique semantic solution. */
        if (solutions == 0 && period <= 428) {
            int candidate_bits[VPCM_CP_MAX_BITS];
            uint16_t effects[VPCM_CP_MAX_BITS];
            int candidate_count = 0;
            vpcm_cp_diag_t baseline;
            int pair_first = -1;
            int pair_second = -1;

            memset(&baseline, 0, sizeof(baseline));
            (void)vpcm_cp_decode_diag(repaired, period, &baseline);
            if (baseline.fill_ok) {
                for (int bit = 18; bit < raw_bits; bit++) {
                    bool start_bit = bit == 34 || bit == 51 || bit == 68
                                  || bit == 85 || bit == 102 || bit == 119
                                  || (bit >= 136
                                      && ((bit - 136) % 17) == 0);
                    vpcm_cp_diag_t trial;

                    if (start_bit)
                        continue;
                    repaired[bit] ^= 1;
                    memset(&trial, 0, sizeof(trial));
                    (void)vpcm_cp_decode_diag(repaired, period, &trial);
                    repaired[bit] ^= 1;
                    if (!trial.fill_ok)
                        continue;
                    candidate_bits[candidate_count] = bit;
                    effects[candidate_count] =
                        (uint16_t)(trial.crc_remainder
                                   ^ baseline.crc_remainder);
                    candidate_count++;
                }
                for (int first = 0; first < candidate_count; first++) {
                    for (int second = first + 1;
                         second < candidate_count; second++) {
                        int bit1;
                        int bit2;
                        vpcm_cp_diag_t trial;

                        if ((uint16_t)(effects[first] ^ effects[second])
                            != baseline.crc_remainder) {
                            continue;
                        }
                        bit1 = candidate_bits[first];
                        bit2 = candidate_bits[second];
                        repaired[bit1] ^= 1;
                        repaired[bit2] ^= 1;
                        memset(&trial, 0, sizeof(trial));
                        if (vpcm_cp_decode_diag(repaired, period, &trial)
                            && v90_cp_live_frame_sane(
                                &trial,
                                expected_compatibility,
                                expected_alaw)) {
                            solution = trial;
                            pair_first = bit1;
                            pair_second = bit2;
                            solutions++;
                        }
                        repaired[bit1] ^= 1;
                        repaired[bit2] ^= 1;
                    }
                }
            }
            if (solutions == 1) {
                solution_bit = pair_first;
                if (second_bit_correction)
                    *second_bit_correction = pair_second;
            }
        }
        if (solutions != 1)
            return false;
    }
    *out = solution;
    if (first_bit_correction)
        *first_bit_correction = solution_bit;
    return true;
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

/* Section 8.5.2 resets both the GPA scrambler and the differential encoder
 * immediately before the first CPt.  A receiver which starts before that
 * boundary cannot recover the first 17-bit frame sync through the usual
 * self-synchronising descrambler: its 23-bit memory has not settled before
 * the sync has already passed.  Search possible first-symbol boundaries and
 * reconstruct the first dibit from the specified all-zero initial state.
 * The first two scrambled bits are both one because CP bit 0 and bit 1 are
 * ones and the GPA register is initially zero. */
static bool v90_cp_live_decode_first_cpt(const uint8_t *quadrants,
                                         int symbol_count,
                                         int expected_compatibility,
                                         bool expected_alaw,
                                         vpcm_cp_diag_t *best,
                                         int *best_offset,
                                         int *best_map,
                                         int *best_order,
                                         int *best_repair_cost,
                                         int *sync_candidates)
{
    static const uint8_t map_table[24][4] = {
        {0,1,2,3}, {1,0,3,2}, {2,3,0,1}, {3,2,1,0},
        {0,2,1,3}, {2,0,3,1}, {1,3,0,2}, {3,1,2,0},
        {0,3,2,1}, {1,2,3,0}, {2,1,0,3}, {3,0,1,2},
        {0,1,3,2}, {1,0,2,3}, {2,3,1,0}, {3,2,0,1},
        {0,2,3,1}, {1,3,2,0}, {2,0,1,3}, {3,1,0,2},
        {0,3,1,2}, {1,2,0,3}, {2,1,3,0}, {3,0,2,1}
    };
    uint8_t bits[VPCM_CP_MAX_BITS];
    int map_begin = 8;
    int map_end = 20;
    int best_sync_errors = 19;
    int best_sync_map = -1;
    int best_sync_order = -1;
    int best_sync_symbol = -1;

    if (sync_candidates)
        *sync_candidates = 0;
    if (!quadrants || !best || !best_offset || !best_map || !best_order
        || !best_repair_cost
        || expected_compatibility != 0
        || 2 * symbol_count < V90_CP_LIVE_MIN_BITS) {
        return false;
    }
    if (getenv("ME_V90_CP_BROAD_MAP")) {
        map_begin = 0;
        map_end = 23;
    }

    for (int map = map_begin; map <= map_end; map++) {
      for (int order = 0; order < 2; order++) {
        /* Without the explicit broad-search override, only the two physical
         * direct/conjugated DQPSK mappings are meaningful. */
        if (!getenv("ME_V90_CP_BROAD_MAP")
            && !((map == 8 && order == 0)
                 || (map == 20 && order == 1))) {
            continue;
        }
        for (int symbol_offset = 0;
             symbol_offset + V90_CP_LIVE_MIN_BITS / 2 <= symbol_count;
             symbol_offset++) {
            uint32_t descrambler = 0;
            int available_symbols = symbol_count - symbol_offset;
            int available;
            bool sync = true;
            int sync_errors = 0;

            if (available_symbols > VPCM_CP_MAX_BITS / 2)
                available_symbols = VPCM_CP_MAX_BITS / 2;
            available = 2 * available_symbols;
            bits[0] = (uint8_t)v90_cp_live_descramble(&descrambler, 1);
            bits[1] = (uint8_t)v90_cp_live_descramble(&descrambler, 1);
            for (int symbol = 1; symbol < available_symbols; symbol++) {
                int mapped = map_table[map]
                                      [quadrants[symbol_offset + symbol] & 3];
                int first = order ? (mapped >> 1) & 1 : mapped & 1;
                int second = order ? mapped & 1 : (mapped >> 1) & 1;
                int bit = 2 * symbol;

                bits[bit] = (uint8_t)v90_cp_live_descramble(
                    &descrambler, first);
                bits[bit + 1] = (uint8_t)v90_cp_live_descramble(
                    &descrambler, second);
            }
            for (int bit = 0; bit <= 16; bit++) {
                sync = sync && bits[bit] != 0;
                sync_errors += bits[bit] == 0;
            }
            sync = sync && bits[17] == 0;
            sync_errors += bits[17] != 0;
            if (sync_errors < best_sync_errors) {
                best_sync_errors = sync_errors;
                best_sync_map = map;
                best_sync_order = order;
                best_sync_symbol = symbol_offset;
            }
            if (!sync)
                continue;
            if (sync_candidates)
                (*sync_candidates)++;
            if (getenv("ME_V90_CP_DIAG")
                && map == 8 && order == 0 && available >= 428) {
                vpcm_cp_diag_t probe;
                char reason[96];
                bool decoded;

                memset(&probe, 0, sizeof(probe));
                decoded = vpcm_cp_decode_diag(bits, 428, &probe);
                (void)vpcm_cp_validate(&probe.frame,
                                       reason, sizeof(reason));
                fprintf(stderr,
                        "v90 CP first-frame probe symbol=%d decoded=%d "
                        "kind=%s drn=%u mask=0x%04x count=%u codec-diff=%d "
                        "dfi=%u,%u,%u,%u,%u,%u sync=%d start=%d reserved=%d "
                        "compat=%d fill=%d crc=0x%04x reason=%s\n",
                        symbol_offset, decoded ? 1 : 0,
                        probe.frame.v90_compatibility ? "CP" : "CPt",
                        (unsigned)probe.frame.drn,
                        probe.frame.upstream_rate_mask,
                        (unsigned)probe.frame.constellation_count,
                        probe.frame.codec_constellations_differ ? 1 : 0,
                        (unsigned)probe.frame.dfi[0],
                        (unsigned)probe.frame.dfi[1],
                        (unsigned)probe.frame.dfi[2],
                        (unsigned)probe.frame.dfi[3],
                        (unsigned)probe.frame.dfi[4],
                        (unsigned)probe.frame.dfi[5],
                        probe.frame_sync_ok ? 1 : 0,
                        probe.start_bits_ok ? 1 : 0,
                        probe.reserved_bits_ok ? 1 : 0,
                        probe.v90_compat_ok ? 1 : 0,
                        probe.fill_ok ? 1 : 0,
                        (unsigned)probe.crc_remainder,
                        reason[0] ? reason : "ok");
            }

            for (int mask_blocks = 1;
                 mask_blocks <= VPCM_CP_MAX_MASK_BLOCKS; mask_blocks++) {
                int period = 156 + 136 * mask_blocks;
                vpcm_cp_diag_t candidate;
                int fixed_corrections = 0;
                int first_bit_correction = -1;
                int second_bit_correction = -1;
                bool decoded;

                if (period > available)
                    break;
                memset(&candidate, 0, sizeof(candidate));
                decoded = vpcm_cp_decode_diag(bits, period, &candidate)
                       && v90_cp_live_frame_sane(&candidate,
                                                 expected_compatibility,
                                                 expected_alaw);
                if (!decoded) {
                    decoded = v90_cp_live_repair_first_cpt(
                        bits,
                        period,
                        mask_blocks,
                        expected_compatibility,
                        expected_alaw,
                        &candidate,
                        &fixed_corrections,
                        &first_bit_correction,
                        &second_bit_correction);
                }
                if (!decoded) {
                    if (getenv("ME_V90_CP_DIAG")
                        && period == 428 && fixed_corrections > 0
                        && map == 8 && order == 0) {
                        fprintf(stderr,
                                "v90 CP first-frame repair miss symbol=%d "
                                "fixed=%d bits=%d\n",
                                symbol_offset, fixed_corrections, period);
                    }
                    continue;
                }
                if (getenv("ME_V90_CP_DIAG")
                    && (fixed_corrections > 0
                        || first_bit_correction >= 0)) {
                    fprintf(stderr,
                            "v90 CP first-frame strict repair symbol=%d "
                            "fixed=%d uncertain=%d,%d bits=%d\n",
                            symbol_offset, fixed_corrections,
                            first_bit_correction,
                            second_bit_correction, period);
                }
                *best = candidate;
                *best_offset = 2 * symbol_offset;
                *best_map = map;
                *best_order = order;
                *best_repair_cost = fixed_corrections
                                  + (first_bit_correction >= 0 ? 1 : 0)
                                  + (second_bit_correction >= 0 ? 1 : 0);
                return true;
            }
        }
      }
    }
    if (getenv("ME_V90_CP_DIAG") && best_sync_symbol >= 0) {
        fprintf(stderr,
                "v90 CP first-frame best sync errors=%d map=%d order=%d symbol=%d\n",
                best_sync_errors, best_sync_map, best_sync_order,
                best_sync_symbol);
    }
    return false;
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

/* SmartLink's long-startup CPt is ordinary 2400-baud, 1800-Hz differential
 * QPSK with a 0.1-rolloff pulse shape.  The general Phase-3 front end below
 * is useful when the preceding training can seed its adaptive equalizer, but
 * it can freeze on the pre-CP Phase-4 signal and miss a subsequently perfect
 * CPt stream.  This small fixed matched-filter path has no adaptive state and
 * therefore provides a deterministic first attempt over a buffered capture. */
static float v90_cp_direct_rrc(float t)
{
    const float alpha = V90_CP_DIRECT_RRC_ALPHA;
    float denominator;

    if (fabsf(t) < 1.0e-6f)
        return 1.0f + alpha * (4.0f / (float)M_PI - 1.0f);
    if (fabsf(fabsf(t) - 1.0f / (4.0f * alpha)) < 1.0e-5f) {
        return alpha / sqrtf(2.0f)
             * ((1.0f + 2.0f / (float)M_PI)
                    * sinf((float)M_PI / (4.0f * alpha))
                + (1.0f - 2.0f / (float)M_PI)
                    * cosf((float)M_PI / (4.0f * alpha)));
    }
    denominator = (float)M_PI * t
                * (1.0f - 16.0f * alpha * alpha * t * t);
    return (sinf((float)M_PI * t * (1.0f - alpha))
            + 4.0f * alpha * t
                * cosf((float)M_PI * t * (1.0f + alpha)))
         / denominator;
}

static bool v90_cp_live_direct_recover(const int16_t *samples,
                                       int segment_start,
                                       int segment_end,
                                       float carrier_hz,
                                       int expected_compatibility,
                                       bool expected_alaw,
                                       vpcm_cp_diag_t *out,
                                       v90_cp_live_meta_t *meta,
                                       int carrier_sel)
{
    static const uint8_t quadrant_map[4] = {0, 3, 2, 1};
    const float samples_per_symbol = 10.0f / 3.0f;
    const int half = V90_CP_DIRECT_RRC_HALF;
    int direct_carrier_step = getenv("ME_V90_CP_DIRECT_CARRIER_STEP")
                            ? atoi(getenv("ME_V90_CP_DIRECT_CARRIER_STEP"))
                            : 0;
    float direct_carrier_delta = direct_carrier_step
                               * (float)(M_PI / 2048.0);
    int sample_count;
    int max_symbols;
    float taps[2 * V90_CP_DIRECT_RRC_HALF + 1];
    float *mixed_re = NULL;
    float *mixed_im = NULL;
    float *filtered_re = NULL;
    float *filtered_im = NULL;
    float *decision_errors = NULL;
    uint8_t *quadrants = NULL;
    uint8_t *bits = NULL;
    int *bit_samples = NULL;
    bool found = false;
    double best_quality = DBL_MAX;
    vpcm_cp_diag_t best_candidate;
    v90_cp_live_meta_t best_meta;

    if (!samples || !out || segment_start < 0
        || segment_end <= segment_start + 4 * half) {
        return false;
    }
    sample_count = segment_end - segment_start;
    max_symbols = (int)((sample_count - 2 * half - 2)
                        / samples_per_symbol) + 1;
    if (max_symbols < V90_CP_LIVE_MIN_BITS / 2 + 2)
        return false;

    mixed_re = calloc((size_t)sample_count, sizeof(*mixed_re));
    mixed_im = calloc((size_t)sample_count, sizeof(*mixed_im));
    filtered_re = calloc((size_t)sample_count, sizeof(*filtered_re));
    filtered_im = calloc((size_t)sample_count, sizeof(*filtered_im));
    decision_errors = malloc((size_t)max_symbols
                             * sizeof(*decision_errors));
    quadrants = malloc((size_t)max_symbols);
    bits = malloc((size_t)(2 * max_symbols));
    bit_samples = malloc((size_t)(2 * max_symbols) * sizeof(*bit_samples));
    if (!mixed_re || !mixed_im || !filtered_re || !filtered_im
        || !decision_errors
        || !quadrants || !bits || !bit_samples) {
        goto done;
    }

    {
        float norm = 0.0f;

        for (int tap = -half; tap <= half; tap++) {
            float t = tap / samples_per_symbol;
            float value = v90_cp_direct_rrc(t);

            taps[tap + half] = value;
            norm += value * value;
        }
        norm = sqrtf(norm);
        if (norm > 0.0f) {
            for (int tap = 0; tap <= 2 * half; tap++)
                taps[tap] /= norm;
        }
    }

    {
        double phase = 2.0 * M_PI * carrier_hz
                     * segment_start / V90_CP_LIVE_SAMPLE_RATE;
        double step = 2.0 * M_PI * carrier_hz
                    / V90_CP_LIVE_SAMPLE_RATE;
        float osc_re = (float)cos(phase);
        float osc_im = (float)-sin(phase);
        float step_re = (float)cos(step);
        float step_im = (float)-sin(step);

        for (int i = 0; i < sample_count; i++) {
            float sample = samples[segment_start + i];
            float next_re;

            mixed_re[i] = sample * osc_re;
            mixed_im[i] = sample * osc_im;
            next_re = osc_re * step_re - osc_im * step_im;
            osc_im = osc_re * step_im + osc_im * step_re;
            osc_re = next_re;
            if ((i & 1023) == 1023) {
                float magnitude = hypotf(osc_re, osc_im);

                if (magnitude > 0.0f) {
                    osc_re /= magnitude;
                    osc_im /= magnitude;
                }
            }
        }
    }

    for (int i = half; i < sample_count - half; i++) {
        float re = 0.0f;
        float im = 0.0f;

        for (int tap = -half; tap <= half; tap++) {
            re += taps[tap + half] * mixed_re[i + tap];
            im += taps[tap + half] * mixed_im[i + tap];
        }
        filtered_re[i] = re;
        filtered_im[i] = im;
    }

    for (int timing = 0; timing < V90_CP_LIVE_TIMINGS; timing++) {
        float position = half
                       + timing * samples_per_symbol / V90_CP_LIVE_TIMINGS;
        float previous_re = 0.0f;
        float previous_im = 0.0f;
        bool have_previous = false;
        uint32_t descrambler = 0;
        int bit_count = 0;
        vpcm_cp_diag_t candidate;
        int candidate_offset = -1;
        int candidate_frames = 0;
        int candidate_agreement = 100;
        bool candidate_voted = false;
        int candidate_map = 8;
        int candidate_order = 0;
        int candidate_repair_cost = 0;

        while (position + 1.0f < sample_count - half) {
            int index = (int)floorf(position);
            float fraction = position - index;
            float re = filtered_re[index]
                     + fraction * (filtered_re[index + 1]
                                   - filtered_re[index]);
            float im = filtered_im[index]
                     + fraction * (filtered_im[index + 1]
                                   - filtered_im[index]);

            if (have_previous) {
                float angle = atan2f(im * previous_re - re * previous_im,
                                     re * previous_re + im * previous_im);
                int quadrant = (int)floorf(
                    (angle - direct_carrier_delta)
                    / (float)(M_PI / 2.0) + 0.5f) & 3;
                float ideal = quadrant * (float)(M_PI / 2.0);
                float decision_error = fabsf(remainderf(
                    angle - direct_carrier_delta - ideal,
                    (float)(2.0 * M_PI)));
                int mapped = quadrant_map[quadrant];
                int sample_index = segment_start + (int)(position + 0.5f);

                quadrants[bit_count / 2] = (uint8_t)quadrant;
                decision_errors[bit_count / 2] = decision_error;
                bits[bit_count] = (uint8_t)v90_cp_live_descramble(
                    &descrambler, mapped & 1);
                bit_samples[bit_count++] = sample_index;
                bits[bit_count] = (uint8_t)v90_cp_live_descramble(
                    &descrambler, (mapped >> 1) & 1);
                bit_samples[bit_count++] = sample_index;
            }
            previous_re = re;
            previous_im = im;
            have_previous = true;
            position += samples_per_symbol;
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
        if (!candidate.valid && expected_compatibility == 0) {
            int reset_sync_candidates = 0;

            if (v90_cp_live_decode_first_cpt(quadrants,
                                              bit_count / 2,
                                              expected_compatibility,
                                              expected_alaw,
                                              &candidate,
                                              &candidate_offset,
                                              &candidate_map,
                                              &candidate_order,
                                              &candidate_repair_cost,
                                              &reset_sync_candidates)) {
                candidate_agreement = 100;
            } else if (getenv("ME_V90_CP_DIAG")
                       && reset_sync_candidates > 0) {
                fprintf(stderr,
                        "v90 CP first-frame reset timing=%d sync-candidates=%d\n",
                        timing, reset_sync_candidates);
            }
        }
        if (candidate.valid) {
            int last_offset = candidate_offset + candidate.nbits - 1;
            int first_symbol = candidate_offset / 2;
            int last_symbol = last_offset / 2;
            double phase_error = 0.0;
            int phase_error_count = 0;
            double quality;

            for (int symbol = first_symbol + 1;
                 symbol <= last_symbol && symbol < bit_count / 2; symbol++) {
                phase_error += decision_errors[symbol];
                phase_error_count++;
            }
            if (phase_error_count > 0)
                phase_error /= phase_error_count;
            quality = 10.0 * candidate_repair_cost + phase_error;
            if (!found || quality < best_quality) {
                best_candidate = candidate;
                memset(&best_meta, 0, sizeof(best_meta));
                best_meta.frame_sample = bit_samples[candidate_offset];
                best_meta.last_sample = bit_samples[last_offset];
                best_meta.carrier_sel = carrier_sel;
                best_meta.timing_index = timing;
                best_meta.carrier_step = direct_carrier_step;
                best_meta.pll_gain = 0.0f;
                best_meta.conjugate = false;
                best_meta.map_index = candidate_map;
                best_meta.bit_order = candidate_order;
                best_meta.voted_frames = candidate_voted
                                       ? candidate_frames : 0;
                best_meta.agreement_pct = candidate_agreement;
                best_quality = quality;
                found = true;
            }
        }
    }
    if (found) {
        *out = best_candidate;
        if (meta)
            *meta = best_meta;
        if (getenv("ME_V90_CP_DIAG")) {
            fprintf(stderr,
                    "v90 CP direct selected timing=%d quality=%.6f "
                    "frame=%d bits=%d drn=%u\n",
                    best_meta.timing_index, best_quality,
                    best_meta.frame_sample, best_candidate.nbits,
                    (unsigned)best_candidate.frame.drn);
        }
    }

done:
    free(bit_samples);
    free(bits);
    free(quadrants);
    free(decision_errors);
    free(filtered_im);
    free(filtered_re);
    free(mixed_im);
    free(mixed_re);
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
    if (getenv("ME_V90_CP_SEARCH_START")) {
        search_start = atoi(getenv("ME_V90_CP_SEARCH_START"));
        if (search_start < capture_start)
            search_start = capture_start;
        if (search_start >= sample_count)
            search_start = sample_count - 1;
    }
    if (getenv("ME_V90_CP_SEARCH_END")) {
        search_end = atoi(getenv("ME_V90_CP_SEARCH_END"));
        if (search_end > sample_count)
            search_end = sample_count;
        if (search_end <= search_start)
            search_end = search_start + 1;
    }
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

    if (!getenv("ME_V90_CP_DISABLE_DIRECT")
        && baud_code == P3_BAUD_2400) {
        p3_baud_params_t bp;

        if (p3_get_baud_params(baud_code, &bp)) {
            /* The SL8200 calling modem uses the high (1800-Hz) carrier.  Try
             * it first, then retain the low carrier for other V.90 peers. */
            for (int pass = 0; pass < 2; pass++) {
                int carrier = pass == 0 ? P3_CARRIER_HIGH : P3_CARRIER_LOW;
                float carrier_hz;

                if (carrier < carrier_begin || carrier > carrier_end)
                    continue;
                carrier_hz = carrier == P3_CARRIER_HIGH
                           ? bp.carrier_high_hz : bp.carrier_low_hz;
                if (v90_cp_live_direct_recover(samples,
                                               search_start,
                                               search_end,
                                               carrier_hz,
                                               expected_compatibility,
                                               expected_alaw,
                                               out,
                                               meta,
                                               carrier)) {
                    if (getenv("ME_V90_CP_DIAG")) {
                        fprintf(stderr,
                                "v90 CP direct matched-filter hit carrier=%.0f timing=%d frame=%d\n",
                                carrier_hz,
                                meta ? meta->timing_index : -1,
                                meta ? meta->frame_sample : -1);
                    }
                    return true;
                }
            }
        }
    }

    /* A live worker retries this fixed receiver on each fresh 500-ms PCM
     * block.  Do not let an early, incomplete snapshot fall into the much
     * slower adaptive hypothesis sweep and monopolize that worker while the
     * peer times out.  The adaptive path remains opt-in for offline analysis
     * and unusual channels. */
    if (!getenv("ME_V90_CP_ENABLE_ADAPTIVE_FALLBACK"))
        return false;

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
