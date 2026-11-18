/* Offline scan of a recorded call for the first ANS/ANSam (V.25, V.8 7.2),
 * calling tone CT (V.25) and fax CNG (T.30), using SpanDSP's
 * modem_connect_tones detectors in 20 ms chunks.  Reports the sample index
 * of each first detection.  Used by phase12_decode to locate Phase 1; not
 * on the live call path. */
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
