/*
 * v92_short_phase2_decode.h — V.92 short Phase 2 (Figure 9) analyzer
 */

#ifndef V92_SHORT_PHASE2_DECODE_H
#define V92_SHORT_PHASE2_DECODE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    V92_SHORT_PHASE2_SEQUENCE_UNKNOWN = 0,
    V92_SHORT_PHASE2_SEQUENCE_A,
    V92_SHORT_PHASE2_SEQUENCE_B
} v92_short_phase2_sequence_t;

typedef struct {
    bool info0_seen;
    bool info0_is_d;
    bool short_phase2_requested;
    bool v92_capable;
    bool info0_ack;
    bool info1_seen;
    bool phase3_seen;
    bool training_failed;
    int info0_sample;
    int info1_sample;
    int tx_tone_a_sample;
    int tx_tone_b_sample;
    int tx_tone_a_reversal_sample;
    int tx_tone_b_reversal_sample;
    int rx_tone_a_sample;
    int rx_tone_b_sample;
    int rx_tone_a_reversal_sample;
    int rx_tone_b_reversal_sample;
    int tx_info1_sample;
} v92_short_phase2_observation_t;

typedef struct {
    bool valid;
    v92_short_phase2_sequence_t sequence;
    bool v92_capable;
    int tx_tone_sample;
    int tx_reversal_sample;
    int rx_tone_sample;
    int rx_reversal_sample;
    int tx_info1_sample;
    bool complete;
    bool c94111;
    bool c94112;
    bool c94113;
    bool c94114;
    bool c94115;
    bool c94121;
    bool c94122;
    bool c94123;
    bool c94211;
    bool c94212;
    bool c94213;
    bool c94214;
    bool c94221;
    bool c94222;
    const char *status;
} v92_short_phase2_result_t;

bool v92_short_phase2_req_from_info0_bits(bool info0_is_d, uint8_t raw_26_27);
bool v92_short_phase2_v92_cap_from_info0_bits(bool info0_is_d, uint8_t raw_26_27);

bool v92_short_phase2_analyze(const v92_short_phase2_observation_t *obs,
                              v92_short_phase2_result_t *out);

const char *v92_short_phase2_sequence_id(v92_short_phase2_sequence_t sequence);
const char *v92_short_phase2_local_modem(v92_short_phase2_sequence_t sequence);

#endif /* V92_SHORT_PHASE2_DECODE_H */
