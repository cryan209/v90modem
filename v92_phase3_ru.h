/*
 * v92_phase3_ru.h — helpers for locating V.92 Phase 3 Ru start
 */

#ifndef V92_PHASE3_RU_H
#define V92_PHASE3_RU_H

#include "v92_phase3_decode.h"

#ifdef __cplusplus
extern "C" {
#endif

int v92_phase3_find_ru_sample(const v92_phase3_observation_t *obs,
                              v92_phase3_ru_source_t *source);

#ifdef __cplusplus
}
#endif

#endif /* V92_PHASE3_RU_H */
