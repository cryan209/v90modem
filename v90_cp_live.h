/*
 * Strict batch fallback for V.90 upstream CP/CP' acquisition.
 *
 * The live SpanDSP V.34 receiver normally supplies these bits.  This helper
 * keeps an independent path over the received PCM and returns a frame only
 * when the untouched Table 14 CRC and every fixed field validate.
 */
#ifndef V90_CP_LIVE_H
#define V90_CP_LIVE_H

#include "vpcm_cp.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int frame_sample;
    int last_sample;
    int carrier_sel;
    int timing_index;
    int carrier_step;
    float pll_gain;
    bool conjugate;
} v90_cp_live_meta_t;

/*
 * Recover the newest strict frame of the requested kind from a captured
 * 8 kHz upstream waveform.  phase4_hint_sample is the local downstream
 * DIL->Ri boundary in the same sample coordinate system.  Set
 * expected_compatibility to 0 for CPt and 1 for data-mode CP/CP'.
 */
bool v90_cp_live_recover(const int16_t *samples,
                         int sample_count,
                         int phase4_hint_sample,
                         int expected_compatibility,
                         bool expected_alaw,
                         vpcm_cp_diag_t *out,
                         v90_cp_live_meta_t *meta);

#endif /* V90_CP_LIVE_H */
