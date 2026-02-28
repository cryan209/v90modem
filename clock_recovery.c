/*
 * clock_recovery.c — DPLL clock recovery for RTP jitter compensation
 *
 * Tracks the rate at which RTP timestamps advance relative to the local wall
 * clock, and emits slip signals (+1 insert, -1 drop) to keep the modem's
 * symbol clock locked to the remote end's 8 kHz PCM clock.
 *
 * PI controller gains are conservative: modems tolerate very few slips per
 * second, so we react slowly and only act when phase error exceeds ±0.5
 * samples.
 */

#include "clock_recovery.h"
#include <string.h>
#include <time.h>

/* ------------------------------------------------------------------ */

void cr_init(cr_state_t *s, int sample_rate)
{
    memset(s, 0, sizeof(*s));
    s->sample_rate = sample_rate;
    /* PI gains: Kp makes one slip per ~50 ms of phase error; Ki damps drift */
    s->Kp = 0.01;
    s->Ki = 0.001;
}

void cr_reset(cr_state_t *s)
{
    int sr = s->sample_rate;
    memset(s, 0, sizeof(*s));
    s->sample_rate = sr;
    s->Kp = 0.01;
    s->Ki = 0.001;
}

/* ------------------------------------------------------------------ */

void cr_update(cr_state_t *s, uint32_t rtp_ts, int64_t local_ns)
{
    if (!s->initialized) {
        s->last_rtp_ts  = rtp_ts;
        s->last_local_ns = local_ns;
        s->initialized  = 1;
        return;
    }

    /* Elapsed RTP samples (handle 32-bit wrap) */
    int32_t rtp_delta = (int32_t)(rtp_ts - s->last_rtp_ts);

    /* Elapsed local samples */
    int64_t wall_ns_delta = local_ns - s->last_local_ns;
    double  wall_samples  = (double)wall_ns_delta * s->sample_rate / 1e9;

    /* Phase error: positive = remote is ahead (we should insert a sample) */
    double err = (double)rtp_delta - wall_samples;

    /* Clamp to avoid wild swings on packet loss */
    if (err >  160.0) err =  160.0;
    if (err < -160.0) err = -160.0;

    /* PI controller */
    s->phase_err_int += err * s->Ki;
    double correction = err * s->Kp + s->phase_err_int;

    s->phase_acc += correction;
    s->phase_error_samples = (float)s->phase_acc;

    s->last_rtp_ts   = rtp_ts;
    s->last_local_ns = local_ns;
}

/* ------------------------------------------------------------------ */

int cr_get_adjustment(cr_state_t *s)
{
    /* Trigger a slip when the accumulated phase error exceeds ±0.5 samples */
    if (s->phase_acc >= 0.5) {
        s->phase_acc -= 1.0;
        return +1;   /* insert one silence sample */
    }
    if (s->phase_acc <= -0.5) {
        s->phase_acc += 1.0;
        return -1;   /* drop one sample */
    }
    return 0;
}

float cr_get_phase_error(cr_state_t *s)
{
    return s->phase_error_samples;
}
