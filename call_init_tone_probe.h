#ifndef CALL_INIT_TONE_PROBE_H
#define CALL_INIT_TONE_PROBE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool ok;
    int ans_sample;
    int ans_tone;
    int ct_sample;
    int cng_sample;
} call_init_tone_probe_t;

bool call_init_collect_tones(const int16_t *samples,
                             int total_samples,
                             int max_sample,
                             call_init_tone_probe_t *out);

#endif
