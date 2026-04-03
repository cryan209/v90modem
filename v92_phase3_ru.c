/*
 * v92_phase3_ru.c — Ru start detection helpers
 */

#include "v92_phase3_ru.h"

int v92_phase3_find_ru_sample(const v92_phase3_observation_t *obs,
                              v92_phase3_ru_source_t *source)
{
    int ru = -1;

    if (source)
        *source = V92_PHASE3_RU_SOURCE_NONE;
    if (!obs)
        return -1;

    if (obs->info0_is_d) {
        if (obs->phase3_sample >= 0) {
            ru = obs->phase3_sample;
            if (source)
                *source = V92_PHASE3_RU_SOURCE_PHASE3_GATE;
            return ru;
        }
        if (obs->tx_first_s_sample >= 0) {
            ru = obs->tx_first_s_sample;
            if (source)
                *source = V92_PHASE3_RU_SOURCE_LOCAL_TX_PHASE3_MARKER;
            return ru;
        }
    } else {
        if (obs->rx_s_event_sample >= 0) {
            ru = obs->rx_s_event_sample;
            if (source)
                *source = V92_PHASE3_RU_SOURCE_REMOTE_S_EVENT;
            return ru;
        }
        if (obs->phase3_sample >= 0) {
            ru = obs->phase3_sample;
            if (source)
                *source = V92_PHASE3_RU_SOURCE_PHASE3_GATE;
            return ru;
        }
    }

    return -1;
}
