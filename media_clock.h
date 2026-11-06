/*
 * media_clock.h -- loop-timed media clock: send at the far end's sample rate
 *
 * Over SIP we are the codec.  On a real line the network-timed codec samples
 * the analogue modem's signal, so the PCM stream reaching the digital modem is
 * always at the network's 8000 Hz (V.90 5.2: "timing from the digital network
 * interface"; G.711).  We instead generated our transmit stream from this
 * host's oscillator, and the gateway's playout buffer absorbed the difference
 * until it had to insert or delete a block: measured on the Eicon rig the
 * Cisco runs +62.3 ppm against CT117 on every call (25 calls, 47-1846 s), and
 * the Diva retrained our V.90 upstream every ~150-270 s -- 80 samples at
 * 0.5 samples/s is 160 s.
 *
 * The engine already makes exactly one transmit sample per received sample,
 * so loop timing needs no resampling: only the wall-clock instants at which
 * frames are pulled and sent change.  This module decides those instants.  It
 * watches how far the received RTP stream runs ahead of what we have consumed
 * (the receive jitter buffer's fill, measured at packet arrival), and steers
 * the tick period with a PI loop so that fill stays where it started.  The
 * transmit stream then runs at the far end's rate, as RFC 3550 5.1 expects a
 * sender's timestamps to advance with its sampling clock.
 *
 * Pure logic: no threads, no pjsip.  The caller supplies CLOCK_MONOTONIC
 * nanoseconds and serialises access.
 */
#ifndef MEDIA_CLOCK_H
#define MEDIA_CLOCK_H

#include <stdint.h>

typedef struct {
    /* configuration */
    int     samples_per_frame;   /* 160 at 20 ms */
    int     sample_rate;         /* 8000 */
    double  max_ppm;             /* correction clamp */
    double  kp;                  /* ppm per sample of fill error */
    double  ki;                  /* ppm per sample-second */
    int64_t window_ns;           /* fill measurement window */

    /* ticks we issued */
    int64_t ticks;
    int64_t last_tick_ns;
    double  period_ns;           /* current period */

    /* received stream, unwrapped RTP samples since the anchor */
    int     have_rx;
    uint32_t last_ts;
    int64_t rx_samples;          /* newest packet's position (start of payload) */
    int64_t tick_anchor;         /* ticks at the anchor */

    /* PI loop */
    int64_t win_start_ns;
    int     win_count;
    double  win_max;             /* least-delayed fill in this window */
    int     have_ref;
    double  ref_fill;
    double  integ;
    double  ppm;                 /* + = we tick faster than nominal */
    int     windows;
    double  last_err;
} media_clock_t;

void    media_clock_init(media_clock_t *mc, int sample_rate, int samples_per_frame);
/* Forget the far end (call ended / new stream); keeps configuration. */
void    media_clock_reset(media_clock_t *mc);
/* One frame was pulled and sent at now_ns. */
void    media_clock_tick(media_clock_t *mc, int64_t now_ns);
/* An RTP packet with this timestamp arrived at now_ns. */
void    media_clock_rx(media_clock_t *mc, uint32_t rtp_ts, int64_t now_ns);
/* Period until the next tick, in ns. */
double  media_clock_period_ns(const media_clock_t *mc);
double  media_clock_ppm(const media_clock_t *mc);

#endif
