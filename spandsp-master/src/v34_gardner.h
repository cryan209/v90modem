/*
 * SPDX-License-Identifier: LGPL-2.1-only
 *
 * v34_gardner.h - Gardner timing-error detector for the V.90 upstream
 *                 T/3 receiver.
 *
 * The V.90 digital-side upstream receiver runs at three samples per symbol
 * (9 kHz for 3000 baud, 9.6 kHz for 3200) and used to advance its symbol
 * instant by exactly three samples for the life of a call, with nothing
 * correcting it.  Measured against slmodemd that is good for about fifteen
 * seconds: the descrambled idle stream decodes at 100% ones and then walks
 * off into noise while the peer's DTE is still idle.  A few ppm between the
 * peer's symbol clock and our 8 kHz bearer, or a single sample inserted or
 * dropped anywhere in the RTP path -- a third of a symbol, instantly --
 * accumulates without limit.
 *
 * Gardner's detector (F.M. Gardner, "A BPSK/QPSK Timing-Error Detector for
 * Sampled Receivers", IEEE Trans. Comm. 1986) is the right one here: it
 * needs no decisions, no carrier phase, and only two samples per symbol --
 * the symbol instant and the point halfway to the previous one.
 *
 *     e[k] = Re{ conj(y_mid) * (y[k] - y[k-1]) }
 *
 * It must be fed the matched-filtered signal -- and in this receiver that
 * is the EQUALIZER OUTPUT, not the fixed RRC ahead of it.  The supervised
 * filter is fitted by least squares onto B1, so it is the matched filter
 * here; the RRC is only a pre-filter for acquisition scoring, and its own
 * best sampling instant sits wherever the fitted delay left it.  Measured on
 * the clean loopback, a detector reading the RRC stream reported an error of
 * -0.40 with the true timing error at zero and drove the loop into inserting
 * symbols on a channel that had no drift.  Taking the half-symbol point
 * through the same filter costs one extra evaluation per symbol.
 *
 * An earlier attempt used the ENERGY of the equalizer output a third of a
 * symbol either side of the instant, which is not a timing discriminant at
 * all: live it asked for about thirty corrections a second where a
 * ppm-level offset needs one every few seconds.
 *
 * Sign convention, derived for a +1 -> -1 transition sampled late: the mid
 * sample has passed the zero crossing and carries the sign of the new
 * symbol, while (y[k] - y[k-1]) carries the same sign, so e > 0 means the
 * instant is LATE and should move earlier.  v34_gardner_update() returns
 * that as a correction in samples, so a caller can simply add it.
 *
 * Kept header-only and free of spandsp types so that v34_gardner_test can
 * exercise it directly against a synthetic pulse train with a known offset,
 * rather than only ever being judged by whether a live call survives.
 */

#if !defined(_SPANDSP_V34_GARDNER_H_)
#define _SPANDSP_V34_GARDNER_H_

/* Default loop gains.  They are deliberately slow.  Gardner's error carries
   data self-noise on a dense QAM constellation -- measured on the clean
   loopback it swings about +/-0.2 from symbol to symbol with the true timing
   error at zero -- and this loop is not being asked to acquire phase.  The
   fractionally spaced equalizer owns phase; all this has to do is catch a
   clock offset of a few ppm before it accumulates into a symbol.  Fast gains
   here just turn self-noise into jitter and, with the integrator, into drift
   the signal never had. */
#define V34_GARDNER_DEFAULT_MU              0.005f
#define V34_GARDNER_DEFAULT_BETA            0.000005f
/* Symbols the sampling position must stay beyond half a sample before the
   instant is actually moved.  At 3200 baud this is about 60 ms. */
#define V34_GARDNER_SLIP_HOLD               200

/*! Gardner timing loop state.  One per receive direction. */
typedef struct
{
    /*! Proportional gain, applied to the normalised timing error. */
    float mu;
    /*! Integral gain.  This is what lets a constant clock offset be tracked
        with no standing error; without it the loop only ever chases. */
    float beta;
    /*! Integrator: the per-symbol drift the loop has settled on, in samples
        per symbol. */
    float freq;
    /*! Fractional part of the accumulated correction, in samples, always in
        [-0.5, 0.5).  Whole samples are handed back to the caller; this is
        what the caller interpolates by, and having it is what stops the loop
        limit-cycling against a third-of-a-symbol actuator quantum. */
    float acc;
    /*! Previous symbol instant's equalizer output. */
    float prev_re;
    float prev_im;
    int prev_valid;
    /*! Net whole-sample corrections applied, for logging: a healthy loop on
        a few-ppm offset slips seconds apart, not tens of times a second. */
    int slips;
    /*! Consecutive symbols the position has spent beyond half a sample,
        signed by direction.  A whole-sample step needs this to persist. */
    int hold;
    /*! Most recent normalised error, for tests and diagnostics. */
    float last_error;
} v34_gardner_state_t;

static __inline__ void v34_gardner_init(v34_gardner_state_t *g,
                                        float mu,
                                        float beta)
{
    g->mu = mu;
    g->beta = beta;
    g->freq = 0.0f;
    g->acc = 0.0f;
    g->hold = 0;
    g->prev_re = 0.0f;
    g->prev_im = 0.0f;
    g->prev_valid = 0;
    g->slips = 0;
    g->last_error = 0.0f;
}
/*- End of function --------------------------------------------------------*/

/*! Raw Gardner error for one symbol, unnormalised.
    \param now_re,now_im   Equalizer output at this symbol instant.
    \param prev_re,prev_im Equalizer output at the previous instant.
    \param mid_re,mid_im   Equalizer output halfway between the two.
    \return e, positive when the instant is late. */
static __inline__ float v34_gardner_error(float now_re, float now_im,
                                          float prev_re, float prev_im,
                                          float mid_re, float mid_im)
{
    float d_re = now_re - prev_re;
    float d_im = now_im - prev_im;

    /* Re{ conj(mid) * d } */
    return mid_re*d_re + mid_im*d_im;
}
/*- End of function --------------------------------------------------------*/

/*! Feed one symbol to the loop.

    The actuator is continuous: whole samples come back as the return value,
    and the leftover fraction is left in g->acc for the caller to apply as an
    interpolation weight when it reads the sample stream.  That matters more
    than it sounds.  With a whole-sample-only actuator the quantum is a third
    of a symbol, the steady-state error is up to a sixth of one, and the
    integrator sees that residual forever: measured against a synthetic
    signal with perfect timing, such a loop asked for 984 corrections over
    6000 symbols and wound its integrator up until it thrashed.  A fractional
    actuator has no such floor.

    \param track Zero to hold the loop still because the decisions feeding
           it cannot be trusted; the detector's history is still kept so it
           can resume without a transient.
    \return -1, 0 or +1 whole samples to move the next symbol instant. */
static __inline__ int v34_gardner_update(v34_gardner_state_t *g,
                                         float now_re, float now_im,
                                         float mid_re, float mid_im,
                                         int track)
{
    /* A real clock offset is a few hundred ppm at worst; at three samples
       per symbol that is well under a thousandth of a sample per symbol.
       Clamping the integrator there keeps a persistent error from winding it
       up into a drift the signal never had. */
    static const float freq_limit = 0.002f;
    float e;
    float power;
    float err;
    int correction = 0;

    if (!g->prev_valid)
    {
        g->prev_re = now_re;
        g->prev_im = now_im;
        g->prev_valid = 1;
        return 0;
    }
    /*endif*/
    if (!track)
    {
        /* The caller says the decisions behind the error are not to be
           trusted -- typically the frame phase is lost and the decoder is
           emitting garbage.  Gardner then reports bias, not noise, and an
           integrator fed on it winds straight to its clamp: measured live,
           freq pinned at -0.002 and 225 corrections requested in forty
           seconds on a call whose symbols were never on the lattice.  Hold
           everything, including the position, and keep only the history the
           detector needs to resume. */
        g->prev_re = now_re;
        g->prev_im = now_im;
        return 0;
    }
    /*endif*/
    e = v34_gardner_error(now_re, now_im, g->prev_re, g->prev_im,
                          mid_re, mid_im);
    /* Normalise so the loop gain does not depend on the signal level, which
       varies with the negotiated constellation. */
    power = now_re*now_re + now_im*now_im
          + g->prev_re*g->prev_re + g->prev_im*g->prev_im
          + 1e-6f;
    g->last_error = e/power;
    /* e > 0 means late, so the correction is the other way. */
    err = -g->last_error;
    g->freq += g->beta*err;
    if (g->freq > freq_limit)
        g->freq = freq_limit;
    else if (g->freq < -freq_limit)
        g->freq = -freq_limit;
    /*endif*/
    /* A whole-sample step needs a SUSTAINED excursion, not an instantaneous
       one.  Gardner's error carries data self-noise, and once a decode goes
       wrong the error is not noise but bias -- it reports whatever the
       garbage looks like.  Measured live: a call sat at 100% ones with freq
       at noise level and zero slips for forty seconds, and when the peer's
       DTE began sending, the slip count went 7 -> 12 in a few seconds and
       the decode was lost, with the symbol error unchanged at 0.10.  The
       signal was never the problem.  Three slips in one direction move the
       symbol clock a whole symbol against the transmitter, and the frame
       phase goes with it -- which the frame-phase sweep cannot recover,
       because it searches mapping frames, not single symbols.

       So the position may sit past half a sample for a while; only if it
       stays there does the instant actually move. */
    g->acc += g->mu*err + g->freq;
    if (g->acc > 1.5f)
        g->acc = 1.5f;
    else if (g->acc < -1.5f)
        g->acc = -1.5f;
    /*endif*/
    if (g->acc >= 0.5f)
    {
        g->hold = (g->hold < 0) ? 1 : g->hold + 1;
        if (g->hold >= V34_GARDNER_SLIP_HOLD)
        {
            correction++;
            g->acc -= 1.0f;
            g->slips++;
            g->hold = 0;
        }
        /*endif*/
    }
    else if (g->acc <= -0.5f)
    {
        g->hold = (g->hold > 0) ? -1 : g->hold - 1;
        if (g->hold <= -V34_GARDNER_SLIP_HOLD)
        {
            correction--;
            g->acc += 1.0f;
            g->slips--;
            g->hold = 0;
        }
        /*endif*/
    }
    else
    {
        g->hold = 0;
    }
    /*endif*/
    g->prev_re = now_re;
    g->prev_im = now_im;
    return correction;
}
/*- End of function --------------------------------------------------------*/

#endif
/*- End of file ------------------------------------------------------------*/
