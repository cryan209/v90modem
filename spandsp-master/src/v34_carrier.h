/*
 * SPDX-License-Identifier: LGPL-2.1-only
 *
 * v34_carrier.h - carrier recovery for the V.90 upstream T/3 receiver.
 *
 * This receiver had no carrier loop of its own.  B1 acquisition sets a
 * derotation once, and after that the only thing correcting carrier was the
 * ordinary V.34 decision-directed tracker in process_primary_symbol() --
 * which ME_V34_DATA_CARRIER_TRACK=0 switches off.  Measured against
 * slmodemd, the difference that makes is the whole story of the upstream:
 *
 *   tracker off   the symbols go from 0.09 to 0.66 -- the mean square
 *                 distance of points bearing no relation to the lattice --
 *                 within seconds, and STAY there for the remaining four
 *                 minutes of a five-minute call
 *   tracker on    the same collapse happens and then reverses: 0.66 at
 *                 t=4 s, 0.415 at t=34, 0.103 at t=42, over and over
 *
 * Nothing but a rotating constellation explains that pair.  A gain error
 * cannot: a 33-point sweep from 0.40x to 2.00x found no scale that puts the
 * symbols back.  A stale equalizer cannot: restoring a known-good one 142
 * times in one call changed nothing.
 *
 * So the upstream needs its own carrier recovery, and it needs two modes,
 * because the two jobs are different:
 *
 *   - Holding a lock is decision-directed.  With the symbols on the
 *     constellation, the angle between a point and its nearest lattice
 *     neighbour is the phase error, and a PI loop keeps it at zero.
 *
 *   - REGAINING a lock cannot be, because the decisions are meaningless
 *     when the constellation is spinning -- that is exactly the state we
 *     are trying to leave.  A square QAM constellation is invariant under
 *     90 degrees, so raising each symbol to the fourth power removes the
 *     modulation and leaves four times the carrier phase; how much that
 *     angle advances between two blocks of symbols measures the frequency
 *     offset without needing a single correct decision.
 *
 * Which mode runs is chosen by the caller from its own estimate of how far
 * the symbols sit from the lattice, so neither mode is ever fed the input
 * the other one is for.
 */

#if !defined(_SPANDSP_V34_CARRIER_H_)
#define _SPANDSP_V34_CARRIER_H_

/*! Symbols per block for the non-data-aided frequency estimate.  Long enough
    that the fourth-power sum has a clear angle, short enough that a real
    offset does not wrap 4*pi within one block. */
#define V34_CARRIER_NDA_BLOCK               256

/*! Decision-directed loop gains, per symbol. */
#define V34_CARRIER_DD_KP                   0.010f
#define V34_CARRIER_DD_KI                   0.00002f

/*! Fraction of a non-data-aided frequency estimate to apply at once.  The
    estimate is noisy; taking a part of it each block converges in a few
    blocks without overshooting into a spin the other way. */
#define V34_CARRIER_NDA_TRUST               0.25f

/*! Ceiling on the tracked offset, in radians per symbol.  At 3200 baud this
    is about +/-50 Hz, far more than any real residual and small enough that
    a runaway cannot hide in it. */
#define V34_CARRIER_FREQ_LIMIT              0.1f

typedef struct
{
    /*! Current derotation angle and its rate, radians and radians/symbol. */
    float phase;
    float freq;
    /*! Fourth-power accumulator for the non-data-aided estimate. */
    float nda_re;
    float nda_im;
    int nda_count;
    /*! Angle of the previous block's accumulator, and whether there was one. */
    float nda_prev_angle;
    int nda_prev_valid;
    /*! Counters, for logs: how often each mode has acted. */
    int dd_updates;
    int nda_updates;
} v34_carrier_state_t;

static __inline__ void v34_carrier_init(v34_carrier_state_t *c)
{
    c->phase = 0.0f;
    c->freq = 0.0f;
    c->nda_re = 0.0f;
    c->nda_im = 0.0f;
    c->nda_count = 0;
    c->nda_prev_angle = 0.0f;
    c->nda_prev_valid = 0;
    c->dd_updates = 0;
    c->nda_updates = 0;
}
/*- End of function --------------------------------------------------------*/

static __inline__ float v34_carrier_wrap(float a)
{
    while (a > 3.14159265f)
        a -= 6.28318531f;
    /*endwhile*/
    while (a < -3.14159265f)
        a += 6.28318531f;
    /*endwhile*/
    return a;
}
/*- End of function --------------------------------------------------------*/

/*! Derotate one equalizer output by the loop's current estimate. */
static __inline__ void v34_carrier_derotate(const v34_carrier_state_t *c,
                                            float in_re, float in_im,
                                            float *out_re, float *out_im)
{
    float cs = cosf(c->phase);
    float sn = sinf(c->phase);

    *out_re = in_re*cs + in_im*sn;
    *out_im = in_im*cs - in_re*sn;
}
/*- End of function --------------------------------------------------------*/

/*! Advance the loop by one symbol.
    \param locked Non-zero when the symbols are close enough to the
           constellation for their nearest neighbours to mean something.  The
           decision-directed loop runs then; the fourth-power estimator runs
           when they are not. */
static __inline__ void v34_carrier_update(v34_carrier_state_t *c,
                                          float re, float im,
                                          int locked)
{
    if (locked)
    {
        /* Phase error against the nearest lattice point.  Normalising by the
           magnitude makes it sin(delta phi) whatever ring the point is on. */
        float t_re = 2.0f*floorf(re/2.0f) + 1.0f;
        float t_im = 2.0f*floorf(im/2.0f) + 1.0f;
        float mag = t_re*t_re + t_im*t_im + 1e-6f;
        float err = (im*t_re - re*t_im)/mag;

        if (err > 0.5f)
            err = 0.5f;
        else if (err < -0.5f)
            err = -0.5f;
        /*endif*/
        c->freq += V34_CARRIER_DD_KI*err;
        c->phase = v34_carrier_wrap(c->phase
                                    + V34_CARRIER_DD_KP*err + c->freq);
        c->dd_updates++;
        /* A held lock makes the previous block's angle stale. */
        c->nda_count = 0;
        c->nda_re = 0.0f;
        c->nda_im = 0.0f;
        c->nda_prev_valid = 0;
    }
    else
    {
        /* Fourth power: (a + jb)^2 then squared again, on the unit circle so
           that big points do not dominate the sum. */
        float mag = sqrtf(re*re + im*im) + 1e-6f;
        float ur = re/mag;
        float ui = im/mag;
        float s_re = ur*ur - ui*ui;
        float s_im = 2.0f*ur*ui;
        float q_re = s_re*s_re - s_im*s_im;
        float q_im = 2.0f*s_re*s_im;

        c->nda_re += q_re;
        c->nda_im += q_im;
        c->phase = v34_carrier_wrap(c->phase + c->freq);
        if (++c->nda_count >= V34_CARRIER_NDA_BLOCK)
        {
            float angle = atan2f(c->nda_im, c->nda_re);

            if (c->nda_prev_valid)
            {
                /* Four times the phase advanced over one block. */
                float d = v34_carrier_wrap(angle - c->nda_prev_angle);
                float est = d/(4.0f*V34_CARRIER_NDA_BLOCK);

                c->freq += V34_CARRIER_NDA_TRUST*est;
                if (c->freq > V34_CARRIER_FREQ_LIMIT)
                    c->freq = V34_CARRIER_FREQ_LIMIT;
                else if (c->freq < -V34_CARRIER_FREQ_LIMIT)
                    c->freq = -V34_CARRIER_FREQ_LIMIT;
                /*endif*/
                c->nda_updates++;
            }
            /*endif*/
            c->nda_prev_angle = angle;
            c->nda_prev_valid = 1;
            c->nda_re = 0.0f;
            c->nda_im = 0.0f;
            c->nda_count = 0;
        }
        /*endif*/
    }
    /*endif*/
}
/*- End of function --------------------------------------------------------*/

#endif
/*- End of file ------------------------------------------------------------*/
