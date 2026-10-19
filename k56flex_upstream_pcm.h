/* Offline K56flex upstream acquisition, Draft 0.23 clause 11.
 * V.34 10.1.3.6-8 supplies the S/S-bar/PP waveform definitions; resident
 * 588D/581B supplies the known 18/23 TRN sequence (not overlay 8E).
 * Research profile: 3200 baud, low carrier, 128T S, 16T S-bar, 288T PP,
 * 512T four-point TRN. These lengths are fixture hypotheses, not a claim
 * that every K56flex client uses this profile. No live engine gates.
 */
#ifndef K56FLEX_UPSTREAM_PCM_H
#define K56FLEX_UPSTREAM_PCM_H

#include "k56flex_upstream.h"

typedef struct {
    int acquired;
    int trn_sample;      /* observation index, includes filter delay */
    int report_sample;
    int end_sample;
    double correlation;
    double trn_rms;      /* held-out error; reference has unit RMS */
    double pp_rms;
    double s_rms;
    uint8_t ext;
    uint16_t report;
} k56flex_upstream_pcm_result_t;

/* Batch-only linear 8 kHz input. Caller expands G.711 without altering its
 * sample accounting. No supplied stage boundary, report value or I/Q points.
 * Returns K56U_* bits, 0 if no accepted report, -1 for bad args/allocation.
 * Results reset on every call. Fixed-rate timing search and a TRN-trained
 * T/2 FIR, not a clock/carrier tracking loop: offsets/drift are not qualified.
 * Maximum input: 80000 samples (10 seconds). tap must be 5 or 18. */
int k56flex_upstream_pcm_decode(const int16_t *pcm, int count, unsigned tap,
                               k56flex_upstream_pcm_result_t *out);
#endif
