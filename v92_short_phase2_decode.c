/*
 * v92_short_phase2_decode.c — V.92 short Phase 2 (Figure 9) analyzer
 */

#include "v92_short_phase2_decode.h"

#include <string.h>

bool v92_short_phase2_req_from_info0_bits(bool info0_is_d, uint8_t raw_26_27)
{
    if (info0_is_d)
        return (raw_26_27 & 0x01U) != 0; /* INFO0d bit 26 */
    return (raw_26_27 & 0x02U) != 0;     /* INFO0a bit 27 */
}

bool v92_short_phase2_v92_cap_from_info0_bits(bool info0_is_d, uint8_t raw_26_27)
{
    if (info0_is_d)
        return (raw_26_27 & 0x02U) != 0; /* INFO0d bit 27 */
    return (raw_26_27 & 0x01U) != 0;     /* INFO0a bit 26 */
}

const char *v92_short_phase2_sequence_id(v92_short_phase2_sequence_t sequence)
{
    switch (sequence) {
    case V92_SHORT_PHASE2_SEQUENCE_A: return "A";
    case V92_SHORT_PHASE2_SEQUENCE_B: return "B";
    default: return "?";
    }
}

const char *v92_short_phase2_local_modem(v92_short_phase2_sequence_t sequence)
{
    switch (sequence) {
    case V92_SHORT_PHASE2_SEQUENCE_A: return "analogue";
    case V92_SHORT_PHASE2_SEQUENCE_B: return "digital";
    default: return "unknown";
    }
}

bool v92_short_phase2_analyze(const v92_short_phase2_observation_t *obs,
                              v92_short_phase2_result_t *out)
{
    int phase2_start;

    if (!obs || !out)
        return false;

    memset(out, 0, sizeof(*out));
    if (!obs->info0_seen || !obs->short_phase2_requested)
        return false;

    phase2_start = obs->info0_sample;
    out->sequence = obs->info0_is_d ? V92_SHORT_PHASE2_SEQUENCE_A : V92_SHORT_PHASE2_SEQUENCE_B;
    out->v92_capable = obs->v92_capable;

    if (out->sequence == V92_SHORT_PHASE2_SEQUENCE_A) {
        out->tx_tone_sample = obs->tx_tone_a_sample;
        out->tx_reversal_sample = obs->tx_tone_a_reversal_sample;
        out->rx_tone_sample = obs->rx_tone_b_sample;
        out->rx_reversal_sample = obs->rx_tone_b_reversal_sample;
    } else {
        out->tx_tone_sample = obs->tx_tone_b_sample;
        out->tx_reversal_sample = obs->tx_tone_b_reversal_sample;
        out->rx_tone_sample = obs->rx_tone_a_sample;
        out->rx_reversal_sample = obs->rx_tone_a_reversal_sample;
    }
    out->tx_info1_sample = obs->tx_info1_sample;

    if (out->tx_tone_sample >= 0 && out->tx_tone_sample < phase2_start)
        out->tx_tone_sample = -1;
    if (out->tx_reversal_sample >= 0 && out->tx_reversal_sample < phase2_start)
        out->tx_reversal_sample = -1;
    if (out->rx_tone_sample >= 0 && out->rx_tone_sample < phase2_start)
        out->rx_tone_sample = -1;
    if (out->rx_reversal_sample >= 0 && out->rx_reversal_sample < phase2_start)
        out->rx_reversal_sample = -1;
    if (out->tx_info1_sample >= 0 && out->tx_info1_sample < phase2_start)
        out->tx_info1_sample = -1;

    if (out->sequence == V92_SHORT_PHASE2_SEQUENCE_A) {
        out->c94211 = obs->info0_seen && obs->info0_is_d;
        out->c94212 = out->c94211 && out->rx_tone_sample >= 0;
        out->c94213 = out->c94212 && out->rx_reversal_sample >= 0 && out->tx_reversal_sample >= 0;
        out->c94214 = out->c94213 && out->tx_info1_sample >= 0;
        out->complete = out->c94214;

        /* Recovery hints for 9.4.2.2.x */
        out->c94221 = obs->info0_ack;
        out->c94222 = !out->c94213 && obs->training_failed;
    } else {
        out->c94111 = obs->info0_seen && !obs->info0_is_d;
        out->c94112 = out->c94111 && out->rx_tone_sample >= 0;
        out->c94113 = out->c94112 && out->tx_reversal_sample >= 0;
        out->c94114 = out->c94113 && out->rx_reversal_sample >= 0;
        out->c94115 = out->c94114 && obs->info1_seen;
        out->complete = out->c94115;

        /* Recovery hints for 9.4.1.2.x */
        out->c94121 = obs->info0_ack;
        out->c94122 = !out->c94114 && obs->phase3_seen;
        out->c94123 = !out->c94115 && obs->phase3_seen;
    }

    if (out->complete)
        out->status = "complete";
    else if (out->c94121 || out->c94122 || out->c94123 || out->c94221 || out->c94222)
        out->status = "recovery";
    else
        out->status = "in_progress";

    out->valid = true;
    return true;
}
