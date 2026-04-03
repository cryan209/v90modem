/*
 * v92_phase4_decode.h — V.92 Phase 4 analyzer scaffold
 */

#ifndef V92_PHASE4_DECODE_H
#define V92_PHASE4_DECODE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool phase4_seen;
    bool training_failed;
    int phase4_sample;
} v92_phase4_observation_t;

typedef struct {
    bool valid;
    bool started;
    bool complete;
    int phase4_sample;
    const char *status;
} v92_phase4_result_t;

bool v92_phase4_analyze(const v92_phase4_observation_t *obs,
                        v92_phase4_result_t *out);

#ifdef __cplusplus
}
#endif

#endif /* V92_PHASE4_DECODE_H */
