#ifndef V34_PHASE2_DECODE_H
#define V34_PHASE2_DECODE_H

#include "v90.h"
#include "v91.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * Phase 2 / early Phase 3 offline decode result.
 *
 * This is the current shared handshake result structure used by vpcm_decode.
 * The implementation is still SpanDSP-backed today, but callers should depend
 * on this interface rather than on the raw SpanDSP modem loop.
 */
typedef struct {
    bool info0_seen;
    bool info1_seen;
    bool info0_is_d;
    bool info1_is_d;
    bool info0_ok_event_seen;
    bool info0_bad_event_seen;
    bool info1_ok_event_seen;
    bool info1_bad_event_seen;
    bool info0_from_rescue;
    bool info1_from_rescue;
    bool phase3_seen;
    bool phase4_ready_seen;
    bool phase4_seen;
    bool training_failed;
    bool u_info_from_info1a;
    bool ja_bits_known;
    bool ja_bits_from_local_tx;
    bool ja_bits_estimated;
    bool mp_seen;
    bool mp_remote_ack_seen;
    bool mp_from_rx_frame;
    bool mp_from_rx_recovery;
    bool mp_from_info_sequence;
    bool mp_crc_valid;
    bool mp_fill_valid;
    bool mp_candidate_seen;
    bool mp_candidate_crc_valid;
    bool mp_candidate_fill_valid;
    bool mp_aux_channel_supported;
    bool mp_use_non_linear_encoder;
    bool mp_expanded_shaping;
    bool mp_asymmetric_rates_allowed;
    bool mp_local_ack_bit;
    bool mp_rates_valid;
    int info0_sample;
    int info1_sample;
    int info0_ok_event_sample;
    int info0_bad_event_sample;
    int info1_ok_event_sample;
    int info1_bad_event_sample;
    int phase3_sample;
    int tx_first_s_sample;
    int tx_first_not_s_sample;
    int tx_md_sample;
    int tx_second_s_sample;
    int tx_second_not_s_sample;
    int tx_pp_sample;
    int tx_pp_end_sample;
    int rx_pp_detect_sample;
    int rx_pp_complete_sample;
    int rx_pp_phase;
    int rx_pp_phase_score;
    int rx_pp_acquire_hits;
    bool rx_pp_started;
    int tx_trn_sample;
    int phase3_trn_lock_sample;
    int phase3_trn_strong_sample;
    int phase3_trn_lock_score;
    int tx_ja_sample;
    int tx_jdashed_sample;
    int ja_trn16;
    int ja_detector_bits;
    int ja_observed_bits;
    char ja_bits[129];
    int ja_aux_bit_len;
    char ja_aux_bits[8193];
    int ja_aux_hyp_bit_len[8];
    char ja_aux_hyp_bits[8][8193];
    int ja_aux_hyp_raw_bit_len[8];
    char ja_aux_hyp_raw_bits[8][8193];
    int rx_s_event_sample;
    int rx_tone_a_sample;
    int rx_tone_b_sample;
    int rx_tone_a_reversal_sample;
    int rx_tone_b_reversal_sample;
    int tx_tone_a_sample;
    int tx_tone_b_sample;
    int tx_tone_a_reversal_sample;
    int tx_tone_b_reversal_sample;
    int tx_info1_sample;
    int rx_phase4_s_sample;
    int rx_phase4_sbar_sample;
    int rx_phase4_trn_sample;
    int ru_window_len;
    uint8_t ru_window_symbols[32];
    float ru_window_mags[32];
    bool ru_window_captured;
    int ru_window_score;
    float trn1u_mag_mean;
    int trn1u_mag_count;
    int phase4_ready_sample;
    int phase4_sample;
    int failure_sample;
    int final_rx_stage;
    int final_tx_stage;
    int final_rx_event;
    int mp_type;
    int mp_trellis_size;
    int mp_signalling_rate_mask;
    int mp_frame_bits_captured;
    int mp_start_error_count;
    int mp_candidate_type;
    int mp_candidate_trellis_size;
    int mp_candidate_signalling_rate_mask;
    int mp_candidate_frame_bits_captured;
    int mp_candidate_start_error_count;
    int mp_candidate_votes;
    int mp_candidate_rate_a_to_c_bps;
    int mp_candidate_rate_c_to_a_bps;
    int mp_rate_a_to_c_bps;
    int mp_rate_c_to_a_bps;
    int u_info;
    v34_v90_info0a_t info0_raw;
    v90_info0a_t info0a;
    v34_v90_info1a_t info1a_raw;
    v90_info1a_t info1a;
    v34_v90_info1d_t info1d;
} decode_v34_result_t;

int v34_phase2_result_spec_score(const decode_v34_result_t *result);

void v34_phase2_decode_pair(const int16_t *samples,
                            int total_samples,
                            v91_law_t law,
                            bool allow_info_rate_infer,
                            decode_v34_result_t *answerer,
                            bool *have_answerer,
                            decode_v34_result_t *caller,
                            bool *have_caller);

void v34_phase2_decode_pair_cached(const int16_t *samples,
                                   int total_samples,
                                   v91_law_t law,
                                   bool allow_info_rate_infer,
                                   decode_v34_result_t *answerer,
                                   bool *have_answerer,
                                   decode_v34_result_t *caller,
                                   bool *have_caller);

#endif
