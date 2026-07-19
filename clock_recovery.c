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
 *
 * A single packet-to-packet arrival delta is a bad error signal: ordinary
 * network/OS scheduling jitter is commonly 1-5 ms (8-40 samples at 8 kHz),
 * while a genuine oscillator mismatch between two independent 8 kHz clocks
 * is typically well under 1 sample/sec. Feeding raw per-packet deltas into
 * the PI loop lets jitter completely dominate the "drift" signal, producing
 * a slip on nearly every frame instead of the rare, gentle corrections this
 * module is meant to make. To fix that, cr_update() only runs the PI step
 * once per CR_WINDOW_NS of accumulated wall-clock time, using the RTP-vs-wall
 * delta over that whole window. Jitter's contribution to a ~1 s baseline is
 * a small fraction of a sample; genuine drift accumulates linearly with the
 * window, so the signal-to-noise ratio improves with window length.
 */

#include "clock_recovery.h"
#include <string.h>
#include <time.h>

#define CR_WINDOW_NS (1000000000LL)  /* Batch drift measurement over ~1 s */

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
    int64_t window_ns;
    int32_t rtp_delta;
    double  wall_samples;
    double  err;
    double  correction;

    if (!s->initialized) {
        s->last_rtp_ts   = rtp_ts;
        s->last_local_ns = local_ns;
        s->window_start_rtp_ts   = rtp_ts;
        s->window_start_local_ns = local_ns;
        s->initialized   = 1;
        return;
    }

    s->last_rtp_ts   = rtp_ts;
    s->last_local_ns = local_ns;

    window_ns = local_ns - s->window_start_local_ns;
    if (window_ns < CR_WINDOW_NS)
        return;  /* Still accumulating this window's baseline. */

    /* Elapsed RTP samples over the whole window (handles 32-bit wrap) */
    rtp_delta = (int32_t)(rtp_ts - s->window_start_rtp_ts);

    /* Elapsed local samples over the same window */
    wall_samples = (double) window_ns * s->sample_rate / 1e9;

    /* Phase error: positive = remote is ahead (we should insert a sample) */
    err = (double) rtp_delta - wall_samples;

    /* Clamp to avoid wild swings on packet loss/reordering within a window;
       a window this far off is not plausible as genuine clock drift. */
    if (err >  400.0) err =  400.0;
    if (err < -400.0) err = -400.0;

    /* PI controller */
    s->phase_err_int += err * s->Ki;
    correction = err * s->Kp + s->phase_err_int;

    s->phase_acc += correction;
    s->phase_error_samples = (float) s->phase_acc;

    s->window_start_rtp_ts   = rtp_ts;
    s->window_start_local_ns = local_ns;
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
