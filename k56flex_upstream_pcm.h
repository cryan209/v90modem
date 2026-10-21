/* K56flex upstream preamble/report acquisition, Draft 0.23 clause 11.
 * V.34 10.1.3.6-8 supplies the S/S-bar/PP waveform definitions; resident
 * 588D/581B supplies the known 18/23 TRN sequence (not overlay 8E).
 * Research profile: 3200 baud, low carrier, 128T S, 16T S-bar, 288T PP,
 * 512T four-point TRN. These lengths are fixture hypotheses, not a claim
 * that every K56flex client uses this profile. Streaming callers supply the
 * baud/carrier actually selected by INFO1a. Only audio produces events.
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
    int sbar_sample;
    int mp_sample, mp_ack_sample;
    unsigned mp_type, mp_up, mp_down, mp_mask, mp_trellis, mp_nonlinear, mp_shaping;
} k56flex_upstream_pcm_result_t;

/* Batch-only linear 8 kHz input. Caller expands G.711 without altering its
 * sample accounting. No supplied stage boundary, report value or I/Q points.
 * Returns K56U_* bits, 0 if no accepted report, -1 for bad args/allocation.
 * Results reset on every call. Fixed-rate timing search and a TRN-trained
 * T/2 FIR, not a clock/carrier tracking loop: offsets/drift are not qualified.
 * Maximum input: 80000 samples (10 seconds). tap must be 5 or 18. */
int k56flex_upstream_pcm_decode(const int16_t *pcm, int count, unsigned tap,
                               k56flex_upstream_pcm_result_t *out);
/* Streaming audio-only acquisition of the preamble/report and following MP.
 * MP is V.34 10.1.3.9 framing with K56's downstream rate field. baud_code
 * is V.34 Table 1's 0..5 symbol-rate index; carrier comes from INFO1a.
 * Per-call state, at most 80000 samples before acquisition; the acquired
 * FIR retains only its history/lookahead between chunks. Each event is
 * emitted once. No gate is released by a timer or caller-supplied report. */
typedef struct k56flex_upstream_pcm_s k56flex_upstream_pcm_t;
#define K56U_ACQUIRED 4  /* one-shot streaming S/S-bar/PP/TRN validation */
#define K56U_SBAR 8      /* earlier, directly observed S/S-bar reversal */
#define K56U_MP 16       /* two identical CRC/start/fill-valid MP records */
#define K56U_MP_ACK 32   /* one CRC-valid MP' acknowledging the accepted MP */
k56flex_upstream_pcm_t *k56flex_upstream_pcm_new(int baud_code, int high_carrier, unsigned tap);
void k56flex_upstream_pcm_free(k56flex_upstream_pcm_t *s);
int k56flex_upstream_pcm_rx(k56flex_upstream_pcm_t *s, const int16_t *pcm, int count,
                           k56flex_upstream_pcm_result_t *out);
#endif
