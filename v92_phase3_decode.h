/*
 * v92_phase3_decode.h — V.92 Phase 3 analyzer scaffold
 *
 * Starts at Ru detection (analog modem Phase 3 entry) and tracks a
 * best-effort progression into Phase 4 using already-collected markers.
 */

#ifndef V92_PHASE3_DECODE_H
#define V92_PHASE3_DECODE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    V92_PHASE3_ROLE_UNKNOWN = 0,
    V92_PHASE3_ROLE_ANALOGUE,
    V92_PHASE3_ROLE_DIGITAL
} v92_phase3_role_t;

typedef enum {
    V92_PHASE3_RU_SOURCE_NONE = 0,
    V92_PHASE3_RU_SOURCE_LOCAL_TX_PHASE3_MARKER,
    V92_PHASE3_RU_SOURCE_REMOTE_S_EVENT,
    V92_PHASE3_RU_SOURCE_PHASE3_GATE
} v92_phase3_ru_source_t;

typedef struct {
    bool info0_seen;
    bool info0_is_d;
    bool short_phase2_requested;
    bool v92_capable;
    bool phase3_seen;
    bool phase4_seen;
    bool training_failed;
    int info0_sample;
    int phase3_sample;
    int phase4_sample;
    int tx_first_s_sample;
    int tx_first_not_s_sample;
    int tx_md_sample;
    int tx_second_s_sample;
    int tx_second_not_s_sample;
    int tx_pp_sample;
    int tx_trn_sample;
    int tx_ja_sample;
    int rx_s_event_sample;
    int ru_window_len;
    int ru_window_score;
    uint8_t ru_window_symbols[32];
    float ru_window_mags[32];
    float trn1u_mag_mean;
    int trn1u_mag_count;
} v92_phase3_observation_t;

typedef struct {
    bool valid;
    v92_phase3_role_t local_role;
    v92_phase3_ru_source_t ru_source;
    bool short_phase2_requested;
    bool v92_capable;
    int ru_sample;
    int phase3_sample;
    int phase4_sample;
    bool ru_seen;
    bool sequence_started;
    bool sequence_complete;
    bool phase4_started;
    bool ru_transmitter_is_analogue;
    bool digital_quiet_before_sd;
    bool first_digital_signal_sd_during_ja;
    int ru_expected_t;
    int ur_expected_t;
    int ru_ur_cycle_t;
    bool ru_ur_repeats_after_md_expected;
    const char *ru_ur_repeats_after_md_status;
    bool ru_precoder_bypass_expected;
    bool ru_prefilter_bypass_expected;
    bool ru_trn1u_structure_expected;
    const char *ru_pattern_primary;
    const char *ru_pattern_complement;
    bool ru_pattern_decoded;
    bool ru_pattern_match;
    int ru_symbol_a;
    int ru_symbol_b;
    int ru_match_percent;
    const char *lu_definition;
    const char *lu_absolute_level;
    const char *lu_reference;
    const char *ru_lu_consistency_with_trn1u;
    float ru_lu_ratio;
    const char *phase4_status;
    const char *status;
} v92_phase3_result_t;

bool v92_phase3_analyze(const v92_phase3_observation_t *obs,
                        v92_phase3_result_t *out);

const char *v92_phase3_role_id(v92_phase3_role_t role);
const char *v92_phase3_ru_source_id(v92_phase3_ru_source_t source);

#ifdef __cplusplus
}
#endif

#endif /* V92_PHASE3_DECODE_H */
