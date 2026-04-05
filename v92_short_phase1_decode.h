/*
 * v92_short_phase1_decode.h — V.92 short Phase 1 helper decoder
 *
 * Extracted from vpcm_decode.c to keep V.8 decode flow simpler.
 */

#ifndef V92_SHORT_PHASE1_DECODE_H
#define V92_SHORT_PHASE1_DECODE_H

#include "v91.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool ok;
    const char *name;
    bool digital_modem;
    bool qca;
    bool lapm;
    int aux_value;
    int uqts_ucode;
    uint16_t frame1_bits;
    uint16_t frame2_bits;
    uint16_t decoded_frame_bits;
    int decoded_frame_index;
    bool repeat_seen;
    bool repeat_match;
    bool soft_match;
    int bit_error_count;
    int soft_score;
} v92_short_phase1_candidate_t;

typedef struct {
    bool seen;
    int start_sample;
    int qts_reps;
    int qts_bar_reps;
    int alignment_phase;
    int symbol_count;
    int score;
} v92_qts_hit_t;

typedef struct {
    bool seen;
    int start_sample;
    int duration_symbols;
    int level;
    int score;
    int avg_abs_error;
} v92_anspcm_hit_t;

typedef int (*v92_codeword_to_ucode_fn)(v91_law_t law, uint8_t codeword);

bool v92_decode_short_phase1_candidate(const uint8_t *bits,
                                       int bit_len,
                                       bool use_ch2,
                                       v92_short_phase1_candidate_t *out);

bool v92_decode_short_phase1_candidate_soft(const uint8_t *bits,
                                            const double *confidence,
                                            int bit_len,
                                            bool use_ch2,
                                            v92_short_phase1_candidate_t *out);

const char *v92_anspcm_level_to_str(int level);

bool v92_detect_qts_sequence(const uint8_t *codewords,
                             int total,
                             v91_law_t law,
                             int uqts_ucode,
                             int search_start,
                             int search_end,
                             v92_codeword_to_ucode_fn codeword_to_ucode,
                             v92_qts_hit_t *out);

bool v92_detect_anspcm_sequence(const uint8_t *codewords,
                                int total,
                                v91_law_t law,
                                int lm_level,
                                int alignment_phase,
                                int search_start,
                                int search_end,
                                v92_anspcm_hit_t *out);

#endif /* V92_SHORT_PHASE1_DECODE_H */
