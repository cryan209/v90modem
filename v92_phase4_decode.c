/*
 * v92_phase4_decode.c — V.92 Phase 4 analyzer scaffold
 */

#include "v92_phase4_decode.h"

#include <string.h>

bool v92_phase4_analyze(const v92_phase4_observation_t *obs,
                        v92_phase4_result_t *out)
{
    if (!obs || !out)
        return false;

    memset(out, 0, sizeof(*out));
    out->phase4_sample = obs->phase4_sample;
    out->started = obs->phase4_seen && obs->phase4_sample >= 0;

    if (!out->started) {
        out->status = "waiting_phase4";
    } else if (obs->training_failed) {
        out->status = "failed";
    } else {
        out->status = "started";
    }

    out->complete = false;
    out->valid = true;
    return true;
}
