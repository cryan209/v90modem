/*
 * v92_phase3_decode.c — V.92 Phase 3 analyzer scaffold
 */

#include "v92_phase3_decode.h"

#include "v92_phase3_ru.h"
#include "v92_phase4_decode.h"

#include <string.h>

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
    case V92_PHASE3_RU_SOURCE_LOCAL_S: return "local_s";
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
    /* V.92 8.5.5: Ru/uR are 6-symbol 2-point sequences with opposite polarity. */
    out->ru_pattern_primary = "+LU,+LU,+LU,-LU,-LU,-LU";
    out->ru_pattern_complement = "-LU,-LU,-LU,+LU,+LU,+LU";
    out->ru_precoder_bypass_expected = true;
    out->ru_prefilter_bypass_expected = true;
    out->ru_trn1u_structure_expected = true;

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
