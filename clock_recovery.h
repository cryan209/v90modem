/*
 * clock_recovery.h — DPLL clock recovery for RTP audio streams
 *
 * V.90 and V.22bis are sensitive to symbol timing. This module implements a
 * digital phase-locked loop (DPLL) that tracks RTP timestamp progression and
 * generates slip signals (insert/delete one sample) to keep the local sample
 * clock aligned with the remote end's clock.
 */

#ifndef CLOCK_RECOVERY_H
#define CLOCK_RECOVERY_H

#include <stdint.h>

typedef struct {
    int    sample_rate;     /* Nominal sample rate (8000 Hz) */
    int    initialized;     /* First packet seen flag */

    /* DPLL state */
    double phase_acc;       /* Phase accumulator (samples) */
    double freq_offset;     /* Frequency offset estimate (samples/frame) */
    double phase_err_int;   /* Integral error term */

    /* RTP tracking */
    uint32_t last_rtp_ts;   /* Last observed RTP timestamp */
    int64_t  last_local_ns; /* Wall-clock time of last packet (nanoseconds) */

    /* PI controller gains */
    double Kp;   /* Proportional gain */
    double Ki;   /* Integral gain */

    /* Diagnostics */
    float  phase_error_samples; /* Current phase error in samples */
} cr_state_t;

/* Initialise the clock recovery state for the given sample rate. */
void cr_init(cr_state_t *s, int sample_rate);

/* Reset (e.g. on call hangup). */
void cr_reset(cr_state_t *s);

/*
 * Feed an incoming RTP timestamp and current wall-clock time.
 * rtp_ts     : RTP timestamp from the received packet header.
 * local_ns   : local wall-clock time in nanoseconds (e.g. from clock_gettime).
 */
void cr_update(cr_state_t *s, uint32_t rtp_ts, int64_t local_ns);

/*
 * Call once per audio frame (after cr_update).
 * Returns:
 *   0  — process this frame normally
 *  +1  — insert one extra silence sample (remote is slow)
 *  -1  — drop one sample from this frame (remote is fast)
 */
int cr_get_adjustment(cr_state_t *s);

/* Current phase error in samples (diagnostic). */
float cr_get_phase_error(cr_state_t *s);

#endif /* CLOCK_RECOVERY_H */
