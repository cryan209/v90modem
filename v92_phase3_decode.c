/*
 * v92_phase3_decode.c — V.92 Phase 3 analyzer scaffold
 */

#include "v92_phase3_decode.h"

#include "v92_phase3_ru.h"
#include "v92_phase4_decode.h"

#include <math.h>
#include <string.h>

static void decode_ru_window(v92_phase3_result_t *out, const v92_phase3_observation_t *obs)
{
    int counts[4] = {0, 0, 0, 0};
    int top0 = -1;
    int top1 = -1;
    int best_pct = -1;
    int best_a = -1;
    int best_b = -1;
    int n = 0;
    int i;

    if (!out || !obs || obs->ru_window_len <= 0)
        return;

    n = obs->ru_window_len;
    if (n > 32)
        n = 32;
    for (i = 0; i < n; i++) {
        int s = obs->ru_window_symbols[i] & 0x3;
        counts[s]++;
    }
    for (i = 0; i < 4; i++) {
        if (top0 < 0 || counts[i] > counts[top0]) {
            top1 = top0;
            top0 = i;
        } else if (top1 < 0 || counts[i] > counts[top1]) {
            top1 = i;
        }
    }
    if (top0 < 0 || top1 < 0 || counts[top0] <= 0 || counts[top1] <= 0)
        return;

    for (int offset = 0; offset < 6; offset++) {
        for (int pol = 0; pol < 2; pol++) {
            int a = pol == 0 ? top0 : top1;
            int b = pol == 0 ? top1 : top0;
            int valid = 0;
            int match = 0;
            int pct;

            for (i = 0; i < n; i++) {
                int s = obs->ru_window_symbols[i] & 0x3;
                int expect = (((i + offset) % 6) < 3) ? a : b;

                if (s != a && s != b)
                    continue;
                valid++;
                if (s == expect)
                    match++;
            }
            if (valid < 8)
                continue;
            pct = (100 * match + valid / 2) / valid;
            if (pct > best_pct) {
                best_pct = pct;
                best_a = a;
                best_b = b;
            }
        }
    }

    if (best_pct < 0)
        return;
    out->ru_pattern_decoded = true;
    out->ru_symbol_a = best_a;
    out->ru_symbol_b = best_b;
    out->ru_match_percent = best_pct;
    out->ru_pattern_match = (best_pct >= 75);
}

static void evaluate_lu_trn1u_consistency(v92_phase3_result_t *out,
                                          const v92_phase3_observation_t *obs)
{
    float sum_a = 0.0f;
    float sum_b = 0.0f;
    int n_a = 0;
    int n_b = 0;
    float mean_a;
    float mean_b;
    float ru_lu;
    float ratio;
    int n;

    if (!out || !obs)
        return;
    if (!out->ru_pattern_decoded)
        return;
    if (!out->ru_pattern_match) {
        out->ru_lu_consistency_with_trn1u = "fail_measured_pattern";
        return;
    }
    if (obs->trn1u_mag_count < 8 || obs->trn1u_mag_mean <= 0.0f)
        return;

    n = obs->ru_window_len;
    if (n > 32)
        n = 32;
    for (int i = 0; i < n; i++) {
        int s = obs->ru_window_symbols[i] & 0x3;
        float m = obs->ru_window_mags[i];

        if (!isfinite(m) || m <= 0.0f)
            continue;
        if (s == out->ru_symbol_a) {
            sum_a += m;
            n_a++;
        } else if (s == out->ru_symbol_b) {
            sum_b += m;
            n_b++;
        }
    }
    if (n_a < 3 || n_b < 3)
        return;

    mean_a = sum_a / (float) n_a;
    mean_b = sum_b / (float) n_b;
    ru_lu = 0.5f * (mean_a + mean_b);
    ratio = ru_lu / obs->trn1u_mag_mean;
    out->ru_lu_ratio = ratio;
    out->ru_lu_consistency_with_trn1u =
        (ratio >= 0.70f && ratio <= 1.43f) ? "pass_measured" : "fail_measured";
}

const char *v92_phase3_role_id(v92_phase3_role_t role)
{
    switch (role) {
    case V92_PHASE3_ROLE_ANALOGUE: return "analogue";
    case V92_PHASE3_ROLE_DIGITAL: return "digital";
    default: return "unknown";
    }
}

const char *v92_phase3_ru_source_id(v92_phase3_ru_source_t source)
{
    switch (source) {
    case V92_PHASE3_RU_SOURCE_LOCAL_TX_PHASE3_MARKER: return "local_phase3_tx";
    case V92_PHASE3_RU_SOURCE_REMOTE_S_EVENT: return "remote_s";
    case V92_PHASE3_RU_SOURCE_PHASE3_GATE: return "phase3_gate";
    default: return "none";
    }
}

bool v92_phase3_analyze(const v92_phase3_observation_t *obs,
                        v92_phase3_result_t *out)
{
    v92_phase4_observation_t p4_obs;
    v92_phase4_result_t p4;

    if (!obs || !out)
        return false;

    memset(out, 0, sizeof(*out));
    if (!obs->info0_seen)
        return false;

    out->local_role = obs->info0_is_d ? V92_PHASE3_ROLE_ANALOGUE : V92_PHASE3_ROLE_DIGITAL;
    out->short_phase2_requested = obs->short_phase2_requested;
    out->v92_capable = obs->v92_capable;
    out->ru_sample = v92_phase3_find_ru_sample(obs, &out->ru_source);
    out->phase3_sample = obs->phase3_sample;
    out->phase4_sample = obs->phase4_sample;
    out->ru_seen = out->ru_sample >= 0;
    out->ru_transmitter_is_analogue = true;
    out->digital_quiet_before_sd = true;
    out->first_digital_signal_sd_during_ja = true;
    out->ru_expected_t = 384;
    out->ur_expected_t = 24;
    out->ru_ur_cycle_t = 408;
    out->ru_ur_repeats_after_md_expected = true;
    out->ru_ur_repeats_after_md_status = "unknown";
    /* V.92 8.5.5: Ru/uR are 6-symbol 2-point sequences with opposite polarity. */
    out->ru_pattern_primary = "+LU,+LU,+LU,-LU,-LU,-LU";
    out->ru_pattern_complement = "-LU,-LU,-LU,+LU,+LU,+LU";
    out->ru_precoder_bypass_expected = true;
    out->ru_prefilter_bypass_expected = true;
    out->ru_trn1u_structure_expected = true;
    out->lu_definition = "trn1u_power_calibrated";
    out->lu_absolute_level = "derived_from_trn1u";
    out->lu_reference = "V92_3.8";
    out->ru_lu_consistency_with_trn1u = "inconclusive";
    decode_ru_window(out, obs);
    evaluate_lu_trn1u_consistency(out, obs);

    memset(&p4_obs, 0, sizeof(p4_obs));
    p4_obs.phase4_seen = obs->phase4_seen;
    p4_obs.training_failed = obs->training_failed;
    p4_obs.phase4_sample = obs->phase4_sample;
    if (v92_phase4_analyze(&p4_obs, &p4)) {
        out->phase4_started = p4.started;
        out->phase4_status = p4.status;
    }

    out->sequence_started = out->ru_seen
                            || obs->tx_pp_sample >= 0
                            || obs->tx_trn_sample >= 0
                            || obs->tx_ja_sample >= 0;
    out->sequence_complete = out->sequence_started
                             && obs->phase4_seen
                             && obs->phase4_sample >= 0
                             && (out->ru_sample < 0 || obs->phase4_sample >= out->ru_sample);

    if (obs->tx_md_sample >= 0 && obs->tx_second_s_sample >= 0)
        out->ru_ur_repeats_after_md_status = "observed";
    else if (obs->tx_md_sample >= 0)
        out->ru_ur_repeats_after_md_status = "inconclusive";

    if (!out->ru_seen)
        out->status = "waiting_ru";
    else if (out->sequence_complete)
        out->status = "complete";
    else if (obs->training_failed)
        out->status = "failed";
    else if (obs->tx_ja_sample >= 0)
        out->status = "ja";
    else if (obs->tx_trn_sample >= 0)
        out->status = "trn";
    else if (obs->tx_pp_sample >= 0)
        out->status = "pp";
    else
        out->status = "started";

    out->valid = true;
    return true;
}
