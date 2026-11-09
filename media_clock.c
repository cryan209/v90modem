/*
 * media_clock.c -- loop-timed media clock; see media_clock.h for why.
 *
 * Fill error -> tick rate, a type-2 PI loop.  The plant: a correction of
 * 1 ppm changes the fill by sample_rate * 1e-6 = 0.008 samples/s.  With
 * kp = 12.5 ppm/sample and ki = 0.3125 ppm/(sample s) the loop's natural
 * frequency is 0.05 rad/s, critically damped: a 62 ppm step settles in about
 * 80 s with under 4 samples of peak fill error, far inside a 40 ms (320
 * sample) jitter buffer and far below anything a gateway's playout reacts to.
 * The loop's own noise only moves packet send instants by microseconds; the
 * samples themselves never change.
 *
 * Fill is sampled at every packet arrival and the window keeps the maximum:
 * network jitter only ever delays a packet, so the least-delayed packet of a
 * 1 s window is the clean measurement.
 */
#include "media_clock.h"

#include <string.h>

void media_clock_init(media_clock_t *mc, int sample_rate, int samples_per_frame)
{
    memset(mc, 0, sizeof(*mc));
    mc->sample_rate = sample_rate > 0 ? sample_rate : 8000;
    mc->samples_per_frame = samples_per_frame > 0 ? samples_per_frame : 160;
    mc->max_ppm = 500.0;
    mc->kp = 12.5;
    mc->ki = 0.3125;
    mc->window_ns = 1000000000LL;
    media_clock_reset(mc);
}

void media_clock_reset(media_clock_t *mc)
{
    mc->kp = 12.5;      /* acquisition gains; media_clock_seed() narrows them */
    mc->ki = 0.3125;
    mc->have_rx = 0;
    mc->have_ref = 0;
    mc->integ = 0.0;
    mc->ppm = 0.0;
    mc->windows = 0;
    mc->win_count = 0;
    mc->last_err = 0.0;
    mc->period_ns = 1e9 * mc->samples_per_frame / mc->sample_rate;
}

double media_clock_period_ns(const media_clock_t *mc)
{
    return mc->period_ns;
}

double media_clock_ppm(const media_clock_t *mc)
{
    return mc->ppm;
}

void media_clock_seed(media_clock_t *mc, double ppm)
{
    if (ppm > mc->max_ppm)
        ppm = mc->max_ppm;
    if (ppm < -mc->max_ppm)
        ppm = -mc->max_ppm;
    mc->integ = ppm;
    mc->ppm = ppm;
    /* The frequency is already known, so steer phase at a tenth of the
       acquisition bandwidth (0.005 rad/s, still critically damped):
       kp 1.25 ppm/sample, ki 0.003125 ppm/(sample s).  At the acquisition
       gains a few samples of startup fill error (jitter in the first
       window that sets the reference) became +60 ppm for 15 s -- +128 ppm
       against the Cisco's +68 on the Eicon rig -- and the gateway deleted
       a sample 95 ms into our Ri (V.90 9.4.1.1).  A gateway on fixed
       playout does not care where its fill sits; it slips on rate. */
    mc->kp = 1.25;
    mc->ki = 0.003125;
    mc->period_ns = 1e9 * mc->samples_per_frame / mc->sample_rate / (1.0 + ppm * 1e-6);
}

double media_clock_freq_ppm(const media_clock_t *mc)
{
    return mc->integ;
}

void media_clock_tick(media_clock_t *mc, int64_t now_ns)
{
    mc->ticks++;
    mc->last_tick_ns = now_ns;
}

/* Samples consumed since the anchor, interpolated between ticks. */
static double consumed(const media_clock_t *mc, int64_t now_ns)
{
    double frac = 0.0;

    if (mc->last_tick_ns && now_ns > mc->last_tick_ns) {
        frac = (double) (now_ns - mc->last_tick_ns) / mc->period_ns;
        if (frac > 1.0)
            frac = 1.0;
    }
    return ((double) (mc->ticks - mc->tick_anchor) + frac) * mc->samples_per_frame;
}

static void close_window(media_clock_t *mc)
{
    double nominal = 1e9 * mc->samples_per_frame / mc->sample_rate;
    double err, ppm;

    if (!mc->have_ref) {
        mc->ref_fill = mc->win_max;
        mc->have_ref = 1;
    }
    err = mc->win_max - mc->ref_fill;   /* + : far end ahead of us */
    mc->integ += mc->ki * err * ((double) mc->window_ns / 1e9);
    if (mc->integ > mc->max_ppm)
        mc->integ = mc->max_ppm;
    if (mc->integ < -mc->max_ppm)
        mc->integ = -mc->max_ppm;
    ppm = mc->integ + mc->kp * err;
    if (ppm > mc->max_ppm)
        ppm = mc->max_ppm;
    if (ppm < -mc->max_ppm)
        ppm = -mc->max_ppm;
    mc->ppm = ppm;
    mc->last_err = err;
    mc->period_ns = nominal / (1.0 + ppm * 1e-6);
    mc->windows++;
}

void media_clock_rx(media_clock_t *mc, uint32_t rtp_ts, int64_t now_ns)
{
    int32_t delta;
    double fill;

    if (!mc->have_rx) {
        mc->have_rx = 1;
        mc->last_ts = rtp_ts;
        mc->rx_samples = 0;
        mc->tick_anchor = mc->ticks;
        mc->win_start_ns = now_ns;
        mc->win_count = 0;
        mc->have_ref = 0;     /* phase re-anchored; frequency (integ) kept */
        return;
    }
    delta = (int32_t) (rtp_ts - mc->last_ts);
    if (delta <= 0)
        return;               /* duplicate or reordered: nothing new */
    if (delta > 5 * mc->sample_rate) {
        /* Timestamp jump (new stream, long hold): re-anchor the phase. */
        mc->have_rx = 0;
        media_clock_rx(mc, rtp_ts, now_ns);
        return;
    }
    mc->last_ts = rtp_ts;
    mc->rx_samples += delta;

    /* Received through the end of this packet, minus consumed. */
    fill = (double) (mc->rx_samples + mc->samples_per_frame) - consumed(mc, now_ns);
    if (mc->win_count == 0 || fill > mc->win_max)
        mc->win_max = fill;
    mc->win_count++;
    if (now_ns - mc->win_start_ns >= mc->window_ns) {
        close_window(mc);
        mc->win_start_ns = now_ns;
        mc->win_count = 0;
    }
}
