/*
 * SpanDSP - a series of DSP components for telephony
 *
 * v32bis.c - ITU V.32bis modem
 *
 * Written by Steve Underwood <steveu@coppice.org>
 *
 * Copyright (C) 2008 Steve Underwood
 *
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License version 2.1,
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/*! \file */

/* V.32bis SUPPORT IS A WORK IN PROGRESS - NOT YET FUNCTIONAL! */

#if defined(HAVE_CONFIG_H)
#include "config.h"
#endif

#include <stdlib.h>
#include <inttypes.h>
#include <string.h>
#include <stdio.h>
#if defined(HAVE_TGMATH_H)
#include <tgmath.h>
#endif
#if defined(HAVE_MATH_H)
#include <math.h>
#endif
#if defined(HAVE_STDBOOL_H)
#include <stdbool.h>
#else
#include "spandsp/stdbool.h"
#endif
#include "floating_fudge.h"

#include "spandsp/telephony.h"
#include "spandsp/alloc.h"
#include "spandsp/logging.h"
#include "spandsp/complex.h"
#include "spandsp/vector_float.h"
#include "spandsp/complex_vector_float.h"
#include "spandsp/async.h"
#include "spandsp/power_meter.h"
#include "spandsp/arctan2.h"
#include "spandsp/dds.h"
#include "spandsp/complex_filters.h"
#include "spandsp/godard.h"

#include "spandsp/modem_echo.h"
#include "spandsp/v29rx.h"
#include "spandsp/v17tx.h"
#include "spandsp/v17rx.h"
#include "spandsp/v32bis.h"

#include "spandsp/v17tx.h"

#include "spandsp/private/logging.h"
#include "spandsp/private/power_meter.h"
#include "spandsp/private/godard.h"
#include "spandsp/private/v17tx.h"
#include "spandsp/private/v17rx.h"
#include "spandsp/private/v32bis.h"

#if defined(SPANDSP_USE_FIXED_POINTx)
#define FP_SCALE(x)     ((int16_t) x)
#else
#define FP_SCALE(x)     (x)
#endif

#define FP_CONSTELLATION_SCALE(x)       FP_SCALE(x)

#include "v17_v32bis_tx_constellation_maps.h"
#include "v17_v32bis_rx_constellation_maps.h"
#include "v17_v32bis_tx_rrc.h"
#include "v17_v32bis_rx_rrc.h"

#if defined(SPANDSP_USE_FIXED_POINT)
SPAN_DECLARE(int) v32bis_equalizer_state(v32bis_state_t *s, complexi16_t **coeffs)
#else
SPAN_DECLARE(int) v32bis_equalizer_state(v32bis_state_t *s, complexf_t **coeffs)
#endif
{
    return v17_rx_equalizer_state(&s->rx, coeffs);
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(float) v32bis_rx_carrier_frequency(v32bis_state_t *s)
{
    return v17_rx_carrier_frequency(&s->rx);
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(float) v32bis_rx_symbol_timing_correction(v32bis_state_t *s)
{
    return v17_rx_symbol_timing_correction(&s->rx);
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(float) v32bis_rx_signal_power(v32bis_state_t *s)
{
    return v17_rx_signal_power(&s->rx);
}
/*- End of function --------------------------------------------------------*/

/*! Is the echo canceller in the sample path?  V.32bis is full duplex on a
    2-wire line, so the near end hybrid returns our own transmit into our own
    receiver; the canceller is what the Recommendation assumes is dealing with
    it.  Default on -- but read v32bis_echo_can_adapting() before moving it,
    because a canceller with nothing to cancel is a known way to make a clean
    bearer worse rather than better. */
/*! -log2 of the echo canceller's step while the far end is transmitting.

    1/65536, which sounds absurdly slow and is not.  A V.32bis modem is in
    double talk for the whole call -- that is what full duplex on one pair
    means -- so the adaption's error term is dominated by the far end signal,
    which is uncorrelated with the reference and is typically well ABOVE the
    echo.  An LMS loop injects misadjustment noise of roughly mu/2 of that
    interference, so the step sets a noise floor the receiver has to live
    with: at mu = 1/16 the canceller measurably removed 11.6 dB of the
    received power on a bearer whose echo was 37 dB down, i.e. it was adding
    far more than it took away, and every hybrid row failed with it in and
    passed with it out.

    Measured over return loss x channel delay in v32bis_duplex_test, as rows
    passing with the canceller in against out:

        mu_shift  31 dB return loss   37 dB
        12        0/3 vs 1/3          0/3 vs 3/3
        16        3/3 vs 1/3          3/3 vs 3/3
        20        2/3 vs 1/3          3/3 vs 3/3

    A peak rather than a plateau, which is what says the adaption is doing
    the work and the 31 dB result is not an acquisition coin flip. */
#define V32BIS_ECHO_MU_SHIFT 16

/*! -log2 of the step while clause 6 Note 3's training sequence is on the
    line.  The far end is silent there by construction, so the error term is
    the echo and nothing else and the loop can run as fast as stability
    allows -- which is the whole reason the Recommendation offers the
    sequence. */
#define V32BIS_ECHO_MU_FAST_SHIFT 3

/*! The received-to-reference power ratio above which the far end is taken
    to be transmitting, whatever the script believes.  -6 dB: no 2-wire
    hybrid returns more than that, so anything louder is not our echo. */
#define V32BIS_ECHO_DT_RATIO 0.25

/*! ITU-T V.32bis 6, Note 3: "The duration of this signal must not exceed
    8192 symbol intervals." */
#define V32BIS_EC_TRAIN_MAX_SYMBOLS 8192

/*! How much of that to use.  Note 4 warns that a G.165 network echo
    canceller needs 650 ms of training, which at 2400 baud is 1560 symbol
    intervals, so the default clears that with room to spare. */
#define V32BIS_EC_TRAIN_SYMBOLS 2048

/*! How many symbols of it to build at a time.  The sequence can be four
    times as long as the whole of the 5.2 conditioning signal, so it is
    generated a buffer at a time rather than sized for. */
#define V32BIS_EC_TRAIN_CHUNK 256

static int v32bis_echo_mu_slow(void)
{
    const char *e = getenv("V32BIS_ECHO_MU");

    return (e != NULL) ? atoi(e) : V32BIS_ECHO_MU_SHIFT;
}
/*- End of function --------------------------------------------------------*/

static int v32bis_echo_mu_fast(void)
{
    const char *e = getenv("V32BIS_ECHO_MU_FAST");

    return (e != NULL) ? atoi(e) : V32BIS_ECHO_MU_FAST_SHIFT;
}
/*- End of function --------------------------------------------------------*/

/*! How many symbol intervals of Note 3's training sequence to transmit, 0
    for none.  It is optional, so 0 is a conformant modem. */
static int v32bis_ec_train_symbols(void)
{
    const char *e = getenv("V32BIS_EC_TRAIN");
    int n = (e != NULL) ? atoi(e) : V32BIS_EC_TRAIN_SYMBOLS;

    if (n < 0)
        n = 0;
    /*endif*/
    if (n > V32BIS_EC_TRAIN_MAX_SYMBOLS)
        n = V32BIS_EC_TRAIN_MAX_SYMBOLS;
    /*endif*/
    return n;
}
/*- End of function --------------------------------------------------------*/

static bool v32bis_echo_can(void)
{
    const char *e = getenv("V32BIS_ECHO_CAN");

    return (e == NULL  ||  atoi(e) != 0);
}
/*- End of function --------------------------------------------------------*/

/*! May the canceller adapt on what we are transmitting right now?

    modem_echo.c's own documentation is explicit that LMS adaption "can go
    seriously wrong" on a highly correlative transmit signal, because a
    repetitive signal has many tap sets that cancel it and nothing chooses
    between them.  Clause 6's tone phases are the worst case there is: state
    A repeated is a pure 1800 Hz tone, and alternating A and C is a pair of
    pure tones.  So the canceller runs in the sample path throughout -- its
    estimate still has to be subtracted -- but it only adapts once there is a
    conditioning signal or data on the line, which is the same point clause 6
    Note 3 places its optional echo canceller training sequence. */
static bool v32bis_echo_can_adapting(v32bis_state_t *s)
{
    return !s->tone_phase_active;
}
/*- End of function --------------------------------------------------------*/

/*! Record what we just put on the line, so the receive side can subtract its
    echo from what comes back.  The two streams are consumed in lock step,
    one transmit sample per received sample; if the caller has not produced a
    transmit block yet the reference is silence, which is what the line
    carries at that point anyway. */
static void v32bis_echo_ref_put(v32bis_state_t *s, const int16_t amp[], int len)
{
    int i;

    for (i = 0;  i < len;  i++)
    {
        s->echo_ref[s->echo_ref_in] = amp[i];
        s->echo_ref_quiet[s->echo_ref_in] = s->tx_far_end_quiet;
        s->echo_ref_in = (s->echo_ref_in + 1) & (V32BIS_ECHO_REF_LEN - 1);
        if (s->echo_ref_count < V32BIS_ECHO_REF_LEN)
            s->echo_ref_count++;
        else
            s->echo_ref_out = s->echo_ref_in;
        /*endif*/
    }
    /*endfor*/
}
/*- End of function --------------------------------------------------------*/

static int16_t v32bis_echo_ref_get(v32bis_state_t *s, bool *quiet)
{
    int16_t amp;

    if (s->echo_ref_count <= 0)
    {
        *quiet = false;
        return 0;
    }
    /*endif*/
    amp = s->echo_ref[s->echo_ref_out];
    *quiet = (s->echo_ref_quiet[s->echo_ref_out] != 0);
    s->echo_ref_out = (s->echo_ref_out + 1) & (V32BIS_ECHO_REF_LEN - 1);
    s->echo_ref_count--;
    return amp;
}
/*- End of function --------------------------------------------------------*/

/*! Run one received block through the canceller in place. */
static void v32bis_echo_cancel(v32bis_state_t *s, int16_t amp[], int len)
{
    int i;
    int16_t tx;
    int16_t clean;

    bool quiet;

    if (s->ec == NULL  ||  !s->echo_can_enabled)
    {
        /* Keep the two streams in step even when we are not cancelling, so
           turning the canceller on mid-call cannot start it misaligned. */
        for (i = 0;  i < len;  i++)
            v32bis_echo_ref_get(s, &quiet);
        /*endfor*/
        return;
    }
    /*endif*/
    modem_echo_can_adaption_mode(s->ec, v32bis_echo_can_adapting(s));
    for (i = 0;  i < len;  i++)
    {
        tx = v32bis_echo_ref_get(s, &quiet);
        /* The step follows the reference sample, not the wall clock: what
           matters is whether the far end was silent when the sample that is
           echoing now went out, and the tag travels with it down the FIFO,
           so the whole of the echo's delay spread is covered without
           guessing at the round trip.

           The tag alone is not enough, and taking it at face value is what
           makes this dangerous: it says "we believe clause 6 has the far
           end silent here", and if that belief is wrong the fast step is
           being applied to an error term that is mostly far end signal,
           which diverges rather than converges.  So it is confirmed against
           the line.  A hybrid cannot return more than a few dB below what
           was sent, so received power well above the reference power means
           the far end is transmitting whatever the script says. */
        s->echo_ref_pow += ((double) tx*tx - s->echo_ref_pow)*(1.0/256.0);
        s->echo_rx_pow += ((double) amp[i]*amp[i] - s->echo_rx_pow)*(1.0/256.0);
        quiet = quiet  &&  (s->echo_rx_pow < V32BIS_ECHO_DT_RATIO*s->echo_ref_pow);
        if (quiet != s->echo_fast_adapt)
        {
            s->echo_fast_adapt = quiet;
            modem_echo_can_step_size(s->ec,
                                     quiet ? v32bis_echo_mu_fast()
                                           : v32bis_echo_mu_slow());
        }
        /*endif*/
        clean = modem_echo_can_update(s->ec, tx, amp[i]);
        s->echo_in_power += (double) amp[i]*amp[i];
        s->echo_out_power += (double) (amp[i] - clean)*(amp[i] - clean);
        s->echo_samples++;
        amp[i] = clean;
    }
    /*endfor*/
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v32bis_tx(v32bis_state_t *s, int16_t amp[], int len)
{
    int ret;

    s->tx_line_samples += len;
    if (s->reneg_cleared)
    {
        memset(amp, 0, len*sizeof(*amp));
        return 0;
    }
    ret = v17_tx(&s->tx, amp, len);
    /* v17_tx() pads the tail of a short block with silence, and that silence
       is what the line carries, so the whole block is the reference. */
    v32bis_echo_ref_put(s, amp, len);
    return ret;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v32bis_set_echo_canceller(v32bis_state_t *s, bool enabled)
{
    s->echo_can_enabled = enabled;
    return 0;
}
/*- End of function --------------------------------------------------------*/

/*! How much of the received signal the canceller is removing, in dB relative
    to the received signal.

    This is deliberately NOT an echo return loss enhancement.  ERLE is the
    received power before cancellation over the power after it, and on a full
    duplex modem that number is nearly meaningless: the received signal is
    dominated by the far end, so removing a 12 dB echo completely moves the
    total received power by about a quarter of a dB.  Measured that way this
    canceller read 0.0 dB while working perfectly.  What can honestly be
    measured from inside the receiver is the size of the estimate being
    subtracted, and whether the call then carries data is the real grade. */
SPAN_DECLARE(float) v32bis_echo_estimate_level(v32bis_state_t *s)
{
    if (s->echo_samples <= 0  ||  s->echo_in_power <= 0.0  ||  s->echo_out_power <= 0.0)
        return 0.0f;
    /*endif*/
    return (float) (10.0*log10(s->echo_out_power/s->echo_in_power));
}
/*- End of function --------------------------------------------------------*/

static void v32bis_tone_rx(v32bis_state_t *s, const int16_t amp[], int len);
static int v32bis_rx_clean(v32bis_state_t *s, const int16_t amp[], int len);

SPAN_DECLARE(int) v32bis_rx(v32bis_state_t *s, const int16_t amp_in[], int len)
{
    int this_len;
    int16_t amp[V32BIS_ECHO_BLOCK];
    int off;

    /* Cancel the near end echo of our own transmit before anything else sees
       the block -- the tone detectors and the V.17 receiver alike.  The input
       is const, so this is done a chunk at a time into a local buffer. */
    for (off = 0;  off < len;  off += this_len)
    {
        this_len = len - off;
        if (this_len > V32BIS_ECHO_BLOCK)
            this_len = V32BIS_ECHO_BLOCK;
        /*endif*/
        memcpy(amp, &amp_in[off], this_len*sizeof(amp[0]));
        v32bis_echo_cancel(s, amp, this_len);
        v32bis_rx_clean(s, amp, this_len);
        s->rx_line_samples += this_len;
    }
    /*endfor*/
    return 0;
}
/*- End of function --------------------------------------------------------*/

static int v32bis_reneg_watch(v32bis_state_t *s, const int16_t amp[], int len);
static int v32bis_retrain_watch(v32bis_state_t *s, const int16_t amp[], int len);
static void v32bis_begin_retrain(v32bis_state_t *s, int64_t rx_line_sample, bool local);
static void startup_b1_reset_vote(v32bis_state_t *s);
static bool v32bis_rx_carrying_data(v32bis_state_t *s);
static bool v32bis_rx_in_far_preamble(v32bis_state_t *s);

static int v32bis_rx_clean(v32bis_state_t *s, const int16_t amp[], int len)
{
    int used;

    if (s->reneg_cleared)
        return 0;
    if (s->startup_complete  &&  s->reactive_startup
        &&  v32bis_rx_carrying_data(s))
    {
        /* 8.1/8.2 clamp at detection, not at the start of the containing
           callback.  The watcher feeds the prefix as data before installing
           the sink; feed only the remaining samples through that sink. */
        used = v32bis_reneg_watch(s, amp, len);
        if (used >= 0)
            return v17_rx(&s->rx, &amp[used], len - used);
    }
    /*endif*/
    if (s->startup_complete  &&  s->reactive_startup
        &&  v32bis_rx_in_far_preamble(s))
    {
        /* 7.1/7.2: the far end's AA or AC may be a renegotiation preamble
           (8: 56T, then the 180 degree reversal) or a retrain (more than
           128T, no reversal).  The first 40T cannot tell them apart, so the
           renegotiation watch has already clamped circuit 104; the tone is
           followed on until it either breaks or outlasts any preamble. */
        used = v32bis_retrain_watch(s, amp, len);
        if (used >= 0)
        {
            v32bis_begin_retrain(s, s->rx_line_samples + used, false);
            return v32bis_rx_clean(s, &amp[used], len - used);
        }
        /*endif*/
    }
    /*endif*/
    if (s->tone_phase_active)
    {
        /* The clause 6 tone phases are read straight off the line, and the
           V.17 receiver is held off until there is a conditioning signal for
           it to train on.  The tone machine may end the phase part way
           through a block, so the rest of the block goes on to V.17. */
        used = 0;
        while (used < len  &&  s->tone_phase_active)
        {
            v32bis_tone_rx(s, &amp[used], 1);
            used++;
        }
        /*endwhile*/
        if (used >= len)
            return 0;
        /*endif*/
        return v17_rx(&s->rx, &amp[used], len - used);
    }
    /*endif*/
    return v17_rx(&s->rx, amp, len);
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v32bis_rx_fillin(v32bis_state_t *s, int len)
{
    return v17_rx_fillin(&s->rx, len);
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v32bis_rx_set_signal_cutoff(v32bis_state_t *s, float cutoff)
{
    v17_rx_set_signal_cutoff(&s->rx, cutoff);
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v32bis_tx_power(v32bis_state_t *s, float power)
{
    v17_tx_power(&s->tx, power);
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v32bis_set_get_bit(v32bis_state_t *s, span_get_bit_func_t get_bit, void *user_data)
{
    v17_tx_set_get_bit(&s->tx, get_bit, user_data);
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v32bis_set_put_bit(v32bis_state_t *s, span_put_bit_func_t put_bit, void *user_data)
{
    v17_rx_set_put_bit(&s->rx, put_bit, user_data);
}
/*- End of function --------------------------------------------------------*/

#define V32BIS_VALID_RATE_MASK   (V32BIS_RATE_14400 | V32BIS_RATE_12000 \
                                | V32BIS_RATE_9600 | V32BIS_RATE_7200 \
                                | V32BIS_RATE_4800)
/* Table 5: B0-B3 = 0, B4 = B8 = 1 (V.32bis rather than V.32), and B7, B11 and
   B15 = 1.  5.3.1 detects a rate signal on B0-B3, B7, B11 and B15.  B11 and
   B15 used to be sent and expected as 0, so a conformant peer's rate signal
   (slmodemd's 0x9ff0) failed our sync test and ours should fail its. */
#define V32BIS_RATE_FIXED_BITS   0x8990
#define V32BIS_RATE_SYNC_MASK    0x888F
#define V32BIS_RATE_SYNC_VALUE   0x8880
#define V32BIS_E_FIXED_BITS      0x899F
/* Table 6 detects E on B0-B3 = 1 and B7, B11, B15 = 1, the same positions
   5.3.1 uses for the rate signal.  B4 and B8 are Table 5's V.32bis flags --
   slmodemd's 4800 bit/s E is 0x88bf, B8 = 0, Note 1's V.32 interworking --
   and B13/B14 "shall be ... ignored during the reception" (Note 2), so none
   of them may be part of the match.  They used to be, and the 4800 bit/s E
   was rejected. */
#define V32BIS_E_SYNC_MASK       0x888F
#define V32BIS_E_SYNC_VALUE      0x888F
#define V32BIS_S_SYMBOLS         256
#define V32BIS_S_BAR_SYMBOLS     16
#define V32BIS_SCRAMBLER_MASK    0x7FFFFF
/*! The V.17 equalizer LMS step, fast and annealed. Kept in step with
    EQUALIZER_FAST_ADAPTION_DELTA in v17rx.c. */
#define V32BIS_EQ_DELTA_FAST_BASE (0.21f/33)
#define V32BIS_EQ_DELTA_FAST     (v32bis_eq_delta_fast())
#define V32BIS_EQ_DELTA_SLOW     (v32bis_eq_delta_slow())
/*! TRN symbols spent at the fast step before annealing. */
#define V32BIS_TRN_FAST_SYMBOLS  (v32bis_trn_fast_symbols())

/*! Energy-normalized LMS. Default OFF, and its premise is now refuted. It was
    written for a residual that looked like gradient noise; that residual was
    the unlatched AGC below, and with the AGC latched the plain step recovers
    every rate/law row without error while this costs 7952. Swept over
    V32BIS_EQ_FAST it never bettered the plain step at any setting, and above
    about 14x the equalizer diverges. V32BIS_NLMS=1 enables it. */
static bool v32bis_use_nlms(void)
{
    static int cached = -1;
    const char *e;

    if (cached < 0)
    {
        e = getenv("V32BIS_NLMS");
        cached = (e != NULL  &&  atoi(e) != 0);
    }
    /*endif*/
    return (bool) cached;
}
/*- End of function --------------------------------------------------------*/

/*! Multiplier on the fast LMS step.  The plain step inherits V.17's tuning;
    the energy-normalized one needs its own, because normalizing divides by the
    equalizer buffer energy and so changes the effective step by that factor. */
static float v32bis_eq_delta_fast(void)
{
    static float cached = -1.0f;
    const char *e;

    if (cached < 0.0f)
    {
        e = getenv("V32BIS_EQ_FAST");
        cached = (e != NULL) ? (float) atof(e)*V32BIS_EQ_DELTA_FAST_BASE
                             : V32BIS_EQ_DELTA_FAST_BASE;
    }
    /*endif*/
    return cached;
}
/*- End of function --------------------------------------------------------*/

/*! Decision-directed equalizer adaption through the data phase. */
static bool v32bis_data_eq(void)
{
    const char *e = getenv("V32BIS_DATA_EQ");

    return (e == NULL  ||  atoi(e) != 0);
}
/*- End of function --------------------------------------------------------*/

static float v32bis_eq_delta_slow(void)
{
    static float cached = -1.0f;
    const char *e;

    if (cached < 0.0f)
    {
        e = getenv("V32BIS_EQ_SLOW");
        cached = (e != NULL) ? (float) atof(e)*V32BIS_EQ_DELTA_FAST
                             : 0.1f*V32BIS_EQ_DELTA_FAST;
    }
    /*endif*/
    return cached;
}
/*- End of function --------------------------------------------------------*/

static int v32bis_trn_fast_symbols(void)
{
    static int cached = -1;
    const char *e;
    int n;

    if (cached >= 0)
        return cached;
    /*endif*/
    e = getenv("V32BIS_TRN_FAST");
    if (e != NULL  &&  (n = atoi(e)) >= 0)
        return (cached = n);
    /* Swept over both laws at all five rates: 160/320/640 give 4539/5144/5415
       total bit errors at the 0.1 anneal.  Since the AGC latch below, the
       anneal is no longer load-bearing: every combination of TRN_FAST in
       {0, 80, 160, 320, 640, 1280} and EQ_SLOW in {0.1, 0.3, 1.0} recovers all
       ten rate/law rows without error.  It is kept because it still helps at
       the smallest steps, where 0.05 costs up to 743 bit errors. */
    return (cached = 160);
}
/*- End of function --------------------------------------------------------*/
/*! ITU-T V.32bis 6.  B1 is the marks segment between E and data. */
#define V32BIS_B1_SYMBOLS        (v32bis_b1_symbols())

/*! 5.2.3: TRN is "at least 1280 and not exceed 8192 symbol intervals".  This
    modem sends the minimum; V32BIS_TRN_SYMBOLS sends more, so that a receiver
    can be tested against a far end that does (slmodemd's runs to ~8000). */
static int v32bis_trn_symbols(void)
{
    static int cached = -1;
    const char *e;

    if (cached < 0)
    {
        cached = 1280;
        if ((e = getenv("V32BIS_TRN_SYMBOLS")) != NULL
            &&  atoi(e) >= 1280  &&  atoi(e) <= 8192)
            cached = atoi(e);
        /*endif*/
    }
    /*endif*/
    return cached;
}
/*- End of function --------------------------------------------------------*/

static int v32bis_b1_symbols(void)
{
    const char *e = getenv("V32BIS_B1_SYMBOLS");
    int n;

    if (e != NULL  &&  (n = atoi(e)) > 0)
        return n;
    return 128;
}
/*- End of function --------------------------------------------------------*/


static bool valid_rate_mask(int rates)
{
    return (rates & V32BIS_VALID_RATE_MASK) != 0
        && (rates & ~V32BIS_VALID_RATE_MASK) == 0;
}
/*- End of function --------------------------------------------------------*/

static int rate_to_mask(int bit_rate)
{
    switch (bit_rate)
    {
    case 4800:
        return V32BIS_RATE_4800;
    case 7200:
        return V32BIS_RATE_7200;
    case 9600:
        return V32BIS_RATE_9600;
    case 12000:
        return V32BIS_RATE_12000;
    case 14400:
        return V32BIS_RATE_14400;
    default:
        return 0;
    }
    /*endswitch*/
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v32bis_build_rate_signal(int rates, uint16_t *word)
{
    if (word == NULL  ||  !valid_rate_mask(rates))
        return -1;
    /* ITU-T V.32bis Table 5.  Bits 4 and 8 advertise V.32 operation at
       4800 and 9600 bit/s; this implementation supports both. */
    *word = (uint16_t) (V32BIS_RATE_FIXED_BITS | rates);
    return 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v32bis_decode_rate_signal(uint16_t word, int *rates)
{
    int decoded;

    if (rates == NULL  ||  (word & V32BIS_RATE_SYNC_MASK) != V32BIS_RATE_SYNC_VALUE)
        return -1;
    decoded = word & V32BIS_VALID_RATE_MASK;
    if (!valid_rate_mask(decoded))
        return -1;
    *rates = decoded;
    return 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v32bis_build_e_signal(int bit_rate, uint16_t *word)
{
    int rate;

    if (word == NULL  ||  (rate = rate_to_mask(bit_rate)) == 0)
        return -1;
    /* ITU-T V.32bis Table 6 requires exactly one selected rate. */
    *word = (uint16_t) (V32BIS_E_FIXED_BITS | rate);
    return 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v32bis_decode_e_signal(uint16_t word, int *bit_rate)
{
    int rates;

    if (bit_rate == NULL  ||  (word & V32BIS_E_SYNC_MASK) != V32BIS_E_SYNC_VALUE)
        return -1;
    rates = word & V32BIS_VALID_RATE_MASK;
    switch (rates)
    {
    case V32BIS_RATE_4800:
        *bit_rate = 4800;
        break;
    case V32BIS_RATE_7200:
        *bit_rate = 7200;
        break;
    case V32BIS_RATE_9600:
        *bit_rate = 9600;
        break;
    case V32BIS_RATE_12000:
        *bit_rate = 12000;
        break;
    case V32BIS_RATE_14400:
        *bit_rate = 14400;
        break;
    default:
        return -1;
    }
    /*endswitch*/
    return 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v32bis_select_common_rate(int local_rates, int remote_rates)
{
    int common;

    if (!valid_rate_mask(local_rates)  ||  !valid_rate_mask(remote_rates))
        return 0;
    common = local_rates & remote_rates;
    if (common & V32BIS_RATE_14400)
        return 14400;
    if (common & V32BIS_RATE_12000)
        return 12000;
    if (common & V32BIS_RATE_9600)
        return 9600;
    if (common & V32BIS_RATE_7200)
        return 7200;
    if (common & V32BIS_RATE_4800)
        return 4800;
    return 0;
}
/*- End of function --------------------------------------------------------*/

static int startup_scramble_bit(uint32_t *reg, int tap, int input)
{
    int output;

    output = (input ^ (*reg >> tap) ^ (*reg >> 22)) & 1;
    *reg = ((*reg << 1) | (uint32_t) output) & V32BIS_SCRAMBLER_MASK;
    return output;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v32bis_build_conditioning(bool calling_party,
                                             int trn_symbols,
                                             uint8_t states[],
                                             uint32_t *scrambler_register,
                                             int *diff_state)
{
    uint32_t reg;
    int tap;
    int i;
    int b0;
    int b1;
    int state;

    if (states == NULL  ||  scrambler_register == NULL  ||  diff_state == NULL
        ||  trn_symbols < 256)
        return -1;
    for (i = 0;  i < V32BIS_S_SYMBOLS;  i++)
        states[i] = (i & 1) ? V32BIS_STARTUP_B : V32BIS_STARTUP_A;
    for (i = 0;  i < V32BIS_S_BAR_SYMBOLS;  i++)
        states[V32BIS_S_SYMBOLS + i] = (i & 1) ? V32BIS_STARTUP_D : V32BIS_STARTUP_C;

    /* ITU-T V.32bis section 5.2.3 initializes TRN's scrambler to zero. */
    reg = 0;
    tap = calling_party ? 17 : 4;
    state = V32BIS_STARTUP_A;
    for (i = 0;  i < trn_symbols;  i++)
    {
        b0 = startup_scramble_bit(&reg, tap, 1);
        b1 = startup_scramble_bit(&reg, tap, 1);
        if (i < 256)
            state = b0 ? V32BIS_STARTUP_C : V32BIS_STARTUP_A;
        else
            state = b0 | (b1 << 1);
        states[V32BIS_S_SYMBOLS + V32BIS_S_BAR_SYMBOLS + i] = (uint8_t) state;
    }
    /* Section 6.1 derives the differential state from the last TRN state.
       The normal-startup scrambler carry is the explicit project policy in
       docs/v32bis_compliance_plan.md; renegotiation resets it instead. */
    *scrambler_register = reg;
    *diff_state = state;
    return V32BIS_S_SYMBOLS + V32BIS_S_BAR_SYMBOLS + trn_symbols;
}
/*- End of function --------------------------------------------------------*/

/*! ITU-T V.32bis 6, Note 3: the optional echo canceller training sequence.
    The Recommendation deliberately does not define it -- it is "a sequence
    which can be used specially for training the echo canceller, but which
    need not be defined in detail" -- but it does constrain it three ways,
    and all three are met here.

    It must keep energy on the line, to hold network echo control devices
    disabled; this is a continuously transmitted signal at the modem's normal
    power.

    It must not be confusable with Segments 1 or 2 of the 5.2 receiver
    conditioning signal, and the Recommendation says exactly what that means:
    the power in the three 200 Hz bands centred at 600, 1800 and 3000 Hz,
    summed, must be at least 1 dB below the power in the rest of the
    bandwidth, averaged over any 6 ms interval.  Those three frequencies are
    the lines S and S-bar put on the line (1800 +/- 1200 Hz) and the carrier
    itself, so what is being asked for is a signal with no spectral lines.
    Scrambled data on the 4 point training constellation has none: it is the
    same construction as the TRN segment, which Note 3's own first sentence
    says is suitable for the job.

    It must not exceed 8192 symbol intervals, which the caller enforces.

    The scrambler runs on independently of TRN's, so this sequence cannot
    disturb the state 5.2 hands to the rate signals. */
static int v32bis_build_ec_training(bool calling_party,
                                    int symbols,
                                    uint8_t states[],
                                    uint32_t *scrambler_register)
{
    uint32_t reg;
    int tap;
    int i;
    int b0;
    int b1;

    if (states == NULL  ||  scrambler_register == NULL  ||  symbols < 0)
        return -1;
    reg = *scrambler_register;
    tap = calling_party ? 17 : 4;
    for (i = 0;  i < symbols;  i++)
    {
        b0 = startup_scramble_bit(&reg, tap, 1);
        b1 = startup_scramble_bit(&reg, tap, 1);
        states[i] = (uint8_t) (b0 | (b1 << 1));
    }
    /*endfor*/
    *scrambler_register = reg;
    return symbols;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v32bis_encode_startup_word(bool calling_party,
                                             uint16_t word,
                                             uint32_t *scrambler_register,
                                             int *diff_state,
                                             uint8_t states[8])
{
    static const uint8_t differential_encoder[4][4] =
    {
        {2, 3, 0, 1},
        {0, 2, 1, 3},
        {3, 1, 2, 0},
        {1, 0, 3, 2}
    };
    uint32_t reg;
    int tap;
    int state;
    int i;
    int b0;
    int b1;

    if (scrambler_register == NULL  ||  diff_state == NULL  ||  states == NULL
        ||  *diff_state < 0  ||  *diff_state > 3)
        return -1;
    reg = *scrambler_register & V32BIS_SCRAMBLER_MASK;
    tap = calling_party ? 17 : 4;
    state = *diff_state;
    for (i = 0;  i < 8;  i++)
    {
        b0 = startup_scramble_bit(&reg, tap, (word >> (2*i)) & 1);
        b1 = startup_scramble_bit(&reg, tap, (word >> (2*i + 1)) & 1);
        state = differential_encoder[state][b0 | (b1 << 1)];
        states[i] = (uint8_t) state;
    }
    *scrambler_register = reg;
    *diff_state = state;
    return 8;
}
/*- End of function --------------------------------------------------------*/

static int startup_descramble_bit(uint32_t *reg, int tap, int input)
{
    int output;

    output = (input ^ (*reg >> tap) ^ (*reg >> 22)) & 1;
    *reg = ((*reg << 1) | (uint32_t) (input & 1)) & V32BIS_SCRAMBLER_MASK;
    return output;
}
/*- End of function --------------------------------------------------------*/

/* Table 2's 4800 bit/s differential transform is self-inverse when each row is
   indexed by the received state. */
static const uint8_t startup_differential_decoder[4][4] =
{
    {2, 3, 0, 1},
    {0, 2, 1, 3},
    {3, 1, 2, 0},
    {1, 0, 3, 2}
};

SPAN_DECLARE(int) v32bis_decode_startup_word(bool calling_party,
                                             const uint8_t states[8],
                                             uint32_t *descrambler_register,
                                             int *diff_state,
                                             uint16_t *word)
{
    uint32_t reg;
    uint16_t decoded;
    int tap;
    int previous;
    int dibit;
    int i;
    int b0;
    int b1;

    if (states == NULL  ||  descrambler_register == NULL  ||  diff_state == NULL
        ||  word == NULL  ||  *diff_state < 0  ||  *diff_state > 3)
        return -1;
    reg = *descrambler_register & V32BIS_SCRAMBLER_MASK;
    tap = calling_party ? 17 : 4;
    previous = *diff_state;
    decoded = 0;
    for (i = 0;  i < 8;  i++)
    {
        if (states[i] > 3)
            return -1;
        dibit = startup_differential_decoder[previous][states[i]];
        previous = states[i];
        b0 = startup_descramble_bit(&reg, tap, dibit & 1);
        b1 = startup_descramble_bit(&reg, tap, (dibit >> 1) & 1);
        decoded |= (uint16_t) (b0 << (2*i));
        decoded |= (uint16_t) (b1 << (2*i + 1));
    }
    *descrambler_register = reg;
    *diff_state = previous;
    *word = decoded;
    return 8;
}
/*- End of function --------------------------------------------------------*/

/*! ITU-T V.32bis 6 transmit phases.  Figure 3 is a half-duplex dialogue, so
    each segment is generated only once the event that releases it has been
    received, rather than queued in advance. */
enum
{
    V32BIS_TX_PHASE_IDLE = 0,
    /*! 6.1: the call modem "shall repetively transmit carrier state A" -- a
        pure 1800 Hz tone. */
    V32BIS_TX_PHASE_TONE_A,
    /*! 6.1: state C, 64 symbol intervals after the first reversal arrives. */
    V32BIS_TX_PHASE_TONE_C,
    /*! 6.2: "alternate carrier states A and C". */
    V32BIS_TX_PHASE_TONE_AC,
    /*! 6.2: "alternate carrier states C and A". */
    V32BIS_TX_PHASE_TONE_CA,
    /*! 6.2: the second run of alternate A and C, after the scheduled CA to AC
        transition. */
    V32BIS_TX_PHASE_TONE_AC2,
    /*! Transmitting nothing.  The call modem does this from the second phase
        reversal until it detects R1; the answer modem from detecting the
        incoming S until it detects R2. */
    V32BIS_TX_PHASE_SILENT,
    /*! 6.1: "the modem shall transmit an S sequence for a period NT already
        estimated by the counter/timer". */
    V32BIS_TX_PHASE_S_NT,
    /*! 6, Note 3: the optional echo canceller training sequence, sent
        immediately before a receiver conditioning signal at the two points
        in clause 6 where the far end is known to be silent. */
    V32BIS_TX_PHASE_EC_TRAIN,
    /*! The 5.2 receiver conditioning signal: S, S-bar, TRN. */
    V32BIS_TX_PHASE_COND,
    /*! A Table 5 rate signal, repeated until the far end answers. */
    V32BIS_TX_PHASE_R,
    /*! One Table 6 E word. */
    V32BIS_TX_PHASE_E,
    /*! 8: the rate renegotiation preamble.  56T of AA then 8T of CC for the
        call mode modem, 56T of AC then 8T of CA for the answer mode modem. */
    V32BIS_TX_PHASE_RENEG_PREAMBLE,
    /*! 8: R4 from the initiating modem, R5 from the responding one. */
    V32BIS_TX_PHASE_RENEG_R,
    /*! 8: the single E word naming the rate common to R4 and R5. */
    V32BIS_TX_PHASE_RENEG_E,
    V32BIS_TX_PHASE_RENEG_CLEAR,
    /*! B1 and then data, generated by the V.17 encoder rather than here. */
    V32BIS_TX_PHASE_DATA
};

static bool v32bis_tx_refill(v32bis_state_t *s);
static void startup_rx_reset(v32bis_state_t *s);

/*! Clause 6 dialogue trace. Both sides print into one stream, so the ordering
    of the two scripts against each other is what it shows. */
static bool v32bis_trace(void)
{
    static int cached = -1;

    if (cached < 0)
        cached = (getenv("V32BIS_TRACE") != NULL);
    /*endif*/
    return (bool) cached;
}

#if defined(SPANDSP_USE_FIXED_POINT)
static int v32bis_startup_symbol_source(void *user_data, complexi16_t *symbol)
#else
static int v32bis_startup_symbol_source(void *user_data, complexf_t *symbol)
#endif
{
    v32bis_state_t *s;
    int state;

    s = (v32bis_state_t *) user_data;
    s->tx_symbol_index++;
    if (s->startup_tx_symbol_count <= 0
        || s->startup_tx_symbol_pos >= s->startup_tx_symbol_count)
    {
        if (!s->reactive_startup  ||  !v32bis_tx_refill(s))
        {
            s->tx.symbol_source = NULL;
            s->tx.symbol_source_user_data = NULL;
            return -1;
        }
        /*endif*/
        if (s->startup_tx_symbol_count == 0)
        {
            /* Silence.  The far end's amplitude-drop and S detectors are what
               this phase exists to drive. */
#if defined(SPANDSP_USE_FIXED_POINT)
            symbol->re = 0;
            symbol->im = 0;
#else
            symbol->re = 0.0f;
            symbol->im = 0.0f;
#endif
            return 0;
        }
        /*endif*/
    }
    /*endif*/
    state = s->startup_tx_symbols[s->startup_tx_symbol_pos++];
#if defined(SPANDSP_USE_FIXED_POINT)
    /* v17_v32bis_tx_constellation_maps.h is generated under
       SPANDSP_USE_FIXED_POINTx, so its tables are float whatever the library is
       built as, while symbol_source takes complexi16_t under --enable-fixed-point.
       Convert at the seam rather than silently mismatching the types. */
    symbol->re = (int16_t) v17_v32bis_4800_constellation[state & 3].re;
    symbol->im = (int16_t) v17_v32bis_4800_constellation[state & 3].im;
#else
    *symbol = v17_v32bis_4800_constellation[state & 3];
#endif
    return 0;
}
/*- End of function --------------------------------------------------------*/

enum
{
    V32BIS_RX_SEARCH_S = 0,
    V32BIS_RX_S_BAR,
    V32BIS_RX_TRN,
    V32BIS_RX_R_FIRST,
    V32BIS_RX_R_SECOND,
    V32BIS_RX_E,
    V32BIS_RX_B1,
    V32BIS_RX_DATA,
    /*! 8: framing the far end's R4 or R5 after its preamble. */
    V32BIS_RX_RENEG_R,
    /*! 8: waiting for the far end's single E word. */
    V32BIS_RX_RENEG_E
};


/* ITU-T V.32bis 8: rate renegotiation.
   ==================================================================

   Clause 8 changes the data signalling rate "without retraining", which is
   the whole point of it and the whole of the difficulty: everything the
   receiver learned at start-up -- the equalizer, the carrier loop, the
   timing loop -- is the solution for a channel that has not changed, and
   must survive the procedure untouched.  The only things that change are
   the constellation and the scrambler/differential state handed to B1.

   The procedure is a preamble, a rate signal, one E word and 24T of B1.
   The preamble is the call modem's repeated state A (a pure 1800 Hz tone)
   or the answer modem's alternating A and C (600 and 3000 Hz), for 56
   symbol intervals, followed by 8 symbol intervals of the same thing turned
   through 180 degrees.  So it is detectable with the clause 6 tone
   detectors, on the samples, with no help at all from the equalizer or the
   carrier loop -- which matters, because 8.2 requires it to be detected
   "at any time while receiving data", when the symbol path is busy carrying
   data and is not available for it. */

/*! 8: the preamble is 56T of one carrier state pattern then 8T of its 180
    degree rotation. */
#define V32BIS_RENEG_PREAMBLE_HEAD  56
#define V32BIS_RENEG_PREAMBLE_TAIL  8

/*! 8.1/8.2: "when R4 has been transmitted for a minimum of 64T", "after R5
    has been transmitted for a period of 64T". */
#define V32BIS_RENEG_R_MIN_SYMBOLS  64

/*! 8.1/8.2: "transmit scrambled binary ones at this data signalling rate for
    24T", and "after a delay of 24T, shall unclamp circuit 104". */
#define V32BIS_RENEG_B1_SYMBOLS     24

/*! How much of the preamble to see before believing it.  Well short of the
    56T head, so the receiver is conditioned for the rate signal before it
    starts, and long enough that data cannot masquerade as it. */
#define V32BIS_RENEG_DETECT_SYMBOLS 40

/*! How far the coherent tone measurement has to stand above the received
    signal's own rms before it is a tone rather than data, as a fraction of
    what an ideal preamble gives.  A W sample coherent sum of a tone of
    amplitude A is W*A/2, and N such tones have an rms of A*sqrt(N/2), so the
    summed measurement of a clean preamble stands at W*sqrt(N/2) times the
    rms: 14.1 for the call modem's AA (one line at 1800 Hz), 20 for the
    answer modem's AC (two, at 600 and 3000 Hz).  Data is not 3 times the rms
    per line as once assumed: the mean of a Rayleigh magnitude is
    sqrt(pi*W/4) = 4.0 times it.  The ratio used to be a flat 6, which the
    call modem's two-line sum of data exceeds on average (7.3, above 6 for 62%
    of samples, measured on the engine pair's data), so it found 8.2's
    preamble inside ordinary data on one engine pair call in five to eight,
    depending on the bits, and started a renegotiation the far end had not
    asked for.  At 0.7 of the ideal the longest run on data seen over 56 s of
    it was 40 samples on either side against the 133 that
    V32BIS_RENEG_DETECT_SYMBOLS needs, while a real preamble keeps 3 dB of
    margin for echo and noise. */
#define V32BIS_RENEG_TONE_FRACTION  0.7f

/*! 7.1/7.2: a retrain tone is one held "for more than 128 symbol
    intervals".  The run is counted from when the 2 x 20 sample detection
    window first holds the tone, about 12T after it starts, so a run of this
    length is a tone some 140T old -- more than twice the 56T head of a
    clause 8 preamble, the only other place the same tone appears. */
#define V32BIS_RETRAIN_TONE_SYMBOLS 128

/*! 8.1: R4 "shall indicate the desired rate in the initiating modem and all
    lower data signalling rates at which the initiating modem is enabled to
    operate".  8.2 says the same of R5 in the responding modem, "irrespective
    of the rates indicated in R4" -- so unlike 6.1's R2 this is NOT masked by
    what the far end asked for. */
static int v32bis_reneg_rate_mask(v32bis_state_t *s, int bit_rate)
{
    static const int ladder[5] =
    {
        V32BIS_RATE_4800,
        V32BIS_RATE_7200,
        V32BIS_RATE_9600,
        V32BIS_RATE_12000,
        V32BIS_RATE_14400
    };
    static const int rates[5] = {4800, 7200, 9600, 12000, 14400};
    int local_rates;
    int mask;
    int i;

    if (v32bis_decode_rate_signal(s->permitted_rates_signal, &local_rates) != 0)
        return 0;
    mask = 0;
    for (i = 0;  i < 5;  i++)
    {
        if (rates[i] <= bit_rate)
            mask |= ladder[i];
        /*endif*/
    }
    /*endfor*/
    return mask & local_rates;
}
/*- End of function --------------------------------------------------------*/

/*! 8: build this side's preamble.  The states are the ones the symbol source
    already emits through the 4 point constellation, so "AA" is state A
    repeated and "AC" is A and C alternating. */
static int v32bis_build_reneg_preamble(bool calling_party, uint8_t states[])
{
    int i;

    for (i = 0;  i < V32BIS_RENEG_PREAMBLE_HEAD;  i++)
    {
        states[i] = calling_party
                  ? V32BIS_STARTUP_A
                  : ((i & 1) ? V32BIS_STARTUP_C : V32BIS_STARTUP_A);
    }
    /*endfor*/
    for (i = 0;  i < V32BIS_RENEG_PREAMBLE_TAIL;  i++)
    {
        states[V32BIS_RENEG_PREAMBLE_HEAD + i] = calling_party
                  ? V32BIS_STARTUP_C
                  : ((i & 1) ? V32BIS_STARTUP_A : V32BIS_STARTUP_C);
    }
    /*endfor*/
    return V32BIS_RENEG_PREAMBLE_HEAD + V32BIS_RENEG_PREAMBLE_TAIL;
}
/*- End of function --------------------------------------------------------*/

/*! 5.3.2: in the rate renegotiation procedure "the differential encoder shall
    be initialized using the final symbol of the transmitted preamble and the
    scrambler shall be initialized to all zeros". */
static int v32bis_reneg_preamble_last_state(bool calling_party)
{
    return calling_party ? V32BIS_STARTUP_C : V32BIS_STARTUP_A;
}
/*- End of function --------------------------------------------------------*/

static void v32bis_reneg_start_tx(v32bis_state_t *s);
static int v32bis_rx_set_rate(v32bis_state_t *s, int bit_rate);

/*! Is the receiver in data mode, rather than part way through a start-up or
    a renegotiation?  That is the condition 8.2 attaches the preamble watch
    to: "at any time while receiving data". */
static bool v32bis_rx_carrying_data(v32bis_state_t *s)
{
    return (s->startup_rx_stage == V32BIS_RX_DATA);
}
/*- End of function --------------------------------------------------------*/

/*! Has the renegotiation watch fired on a far end tone that has not yet
    broken into a preamble's reversal? */
static bool v32bis_rx_in_far_preamble(v32bis_state_t *s)
{
    return s->startup_rx_stage == V32BIS_RX_RENEG_R
        && s->reneg_far_preamble
        && !s->reneg_pre_done;
}
/*- End of function --------------------------------------------------------*/

/*! Frame and decode the far end's rate renegotiation words.

    There is no S sequence here to pin the word boundary to.  What pins it is
    the preamble's own shape: 8 sends 56T of one carrier state pattern
    followed by 8T of the same pattern turned through 180 degrees, so the
    reversal is visible in the symbols as the one place the pattern breaks,
    and 5.3.2 then starts the rate signal's scrambler at all zeros with its
    differential encoder seeded from "the final symbol of the transmitted
    preamble" -- which is 8 symbol intervals after that break.

    Framing it any other way does not work, and the way that looks as though
    it does is the trap: two identical consecutive 16-bit words establish the
    PERIOD of a repeating rate signal but say nothing about its PHASE, and
    several of the 16 cyclic rotations of a Table 5 word pass Table 5's own
    sync test.  A request for 4800 and 7200 came out, perfectly stably, as
    7200, 9600 and 12000 -- the same word rotated by two. */
static void v32bis_reneg_rx_symbol(v32bis_state_t *s, int state)
{
    uint16_t word;
    int expected;
    int rates;
    int rate;

    state &= 3;
    if (!s->reneg_pre_done)
    {
        /* The far end's preamble: repeated state A if it is the call modem,
           alternating A and C if it is the answer modem.  Either way the
           break in that pattern is the 180 degree reversal at 56T. */
        if (s->reneg_pre_have)
        {
            expected = (!s->calling_party)
                     ? s->reneg_pre_last
                     : (s->reneg_pre_last ^ 3);
            if (state != expected)
                s->reneg_pre_tail = 1;
            else if (s->reneg_pre_tail > 0)
                s->reneg_pre_tail++;
            /*endif*/
        }
        /*endif*/
        s->reneg_pre_last = state;
        s->reneg_pre_have = true;
        if (s->reneg_pre_tail < V32BIS_RENEG_PREAMBLE_TAIL)
            return;
        /*endif*/
        /* 5.3.2: scrambler to all zeros, differential encoder to the final
           symbol of the preamble -- taken off the wire rather than from the
           far end's role, so a 180 degree carrier phase ambiguity is
           absorbed exactly as the differential encoding intends. */
        s->reneg_pre_done = true;
        s->reneg_rx_reg = 0;
        s->reneg_rx_diff = state;
        s->reneg_word_pos = 0;
        return;
    }
    /*endif*/
    s->reneg_word_states[s->reneg_word_pos++] = (uint8_t) state;
    if (s->reneg_word_pos < 8)
        return;
    /*endif*/
    s->reneg_word_pos = 0;
    if (v32bis_decode_startup_word(!s->calling_party,
                                   s->reneg_word_states,
                                   &s->reneg_rx_reg,
                                   &s->reneg_rx_diff,
                                   &word) != 8)
        return;
    /*endif*/
    if ((word & (V32BIS_RATE_SYNC_MASK | V32BIS_VALID_RATE_MASK | 0x10))
            == V32BIS_RATE_SYNC_VALUE
        || v32bis_decode_rate_signal(word, &rates) == 0)
    {
        if ((word & (V32BIS_VALID_RATE_MASK | 0x10)) == 0)
            rates = 0;  /* Table 5 Note 3: GSTN cleardown. */
        s->reneg_remote_rates = rates;
        if (!s->reneg_r_seen)
        {
            s->reneg_r_seen = true;
            if (v32bis_trace())
            {
                fprintf(stderr,
                        "[V32BIS %s] 8: %s 0x%04x rates 0x%04x at rx symbol %d\n",
                        s->calling_party ? "call  " : "answer",
                        s->reneg_initiator ? "R5" : "R4",
                        word,
                        rates,
                        s->startup_rx_symbol_count);
            }
            /*endif*/
            if (!s->reneg_initiator)
            {
                /* 8.2: "On detection of R4, the responding modem shall turn
                   circuit 106 OFF and shall transmit the appropriate
                   preamble." */
                v32bis_reneg_start_tx(s);
            }
            /*endif*/
        }
        /*endif*/
        return;
    }
    /*endif*/
    if (v32bis_decode_e_signal(word, &rate) != 0  ||  rate == 0)
        return;
    /*endif*/
    if (v32bis_trace())
    {
        fprintf(stderr,
                "[V32BIS %s] 8: E word %d bit/s at rx symbol %d\n",
                s->calling_party ? "call  " : "answer",
                rate,
                s->startup_rx_symbol_count);
    }
    /*endif*/
    /* 8.1/8.2: "condition itself to receive data at the highest data
       signalling rate common to both R4 and R5 and, after a delay of 24T,
       unclamp circuit 104" -- the 24T being the B1 that follows the E. */
    /* 8.1/8.2: E must select the highest common R4/R5 rate, not an
       arbitrary valid constellation named by a corrupted E word. */
    if (!s->reneg_r_seen
        || rate != v32bis_select_common_rate(s->reneg_local_rates,
                                             s->reneg_remote_rates)
        || v32bis_rx_set_rate(s, rate) != 0)
        return;
    /*endif*/
    s->reneg_selected_rate = rate;
    s->startup_rx_b1_pos = 0;
    s->startup_rx_b1_reg = s->reneg_rx_reg;
    s->startup_rx_b1_diff = s->reneg_rx_diff;
    s->startup_rx_b1_convolution = 0;
    startup_b1_reset_vote(s);
    s->rx_b1_target = V32BIS_RENEG_B1_SYMBOLS;
    s->startup_rx_stage = V32BIS_RX_B1;
    s->rx.symbol_sink_uses_data_constellation = true;
}
/*- End of function --------------------------------------------------------*/

#if defined(SPANDSP_USE_FIXED_POINTx)
static int v32bis_startup_symbol_sink(void *user_data, const complexi16_t *symbol);
#else
static int v32bis_startup_symbol_sink(void *user_data, const complexf_t *symbol);
#endif

static float startup_power(const complexf_t *z)
{
    return z->re*z->re + z->im*z->im;
}
/*- End of function --------------------------------------------------------*/

static complexf_t startup_gain_observation(const complexf_t *z, int state)
{
    const complexf_t *p;
    complexf_t gain;
    float power;

    p = &v17_v32bis_4800_constellation[state & 3];
    power = startup_power(p);
    gain.re = (z->re*p->re + z->im*p->im)/power;
    gain.im = (z->im*p->re - z->re*p->im)/power;
    return gain;
}
/*- End of function --------------------------------------------------------*/

static bool startup_try_acquire_s(v32bis_state_t *s)
{
    complexf_t gains[2];
    complexf_t predicted;
    complexf_t error;
    const complexf_t *p;
    float input_power;
    float reference_power;
    float error_power[2];
    float metric;
    float best_metric;
    int best;
    int parity;
    int i;

    if (s->startup_rx_acq_count < 64)
        return false;
    best = -1;
    best_metric = 1.0e30f;
    for (parity = 0;  parity < 2;  parity++)
    {
        gains[parity].re = 0.0f;
        gains[parity].im = 0.0f;
        reference_power = 0.0f;
        for (i = 0;  i < 64;  i++)
        {
            /* 5.2.1: S alternates states A and B. */
            p = &v17_v32bis_4800_constellation[((i + parity) & 1)
                                               ? V32BIS_STARTUP_B : V32BIS_STARTUP_A];
            gains[parity].re += s->startup_rx_acq[i].re*p->re
                              + s->startup_rx_acq[i].im*p->im;
            gains[parity].im += s->startup_rx_acq[i].im*p->re
                              - s->startup_rx_acq[i].re*p->im;
            reference_power += startup_power(p);
        }
        gains[parity].re /= reference_power;
        gains[parity].im /= reference_power;
        error_power[parity] = 0.0f;
        input_power = 0.0f;
        for (i = 0;  i < 64;  i++)
        {
            p = &v17_v32bis_4800_constellation[((i + parity) & 1)
                                               ? V32BIS_STARTUP_B : V32BIS_STARTUP_A];
            predicted.re = gains[parity].re*p->re - gains[parity].im*p->im;
            predicted.im = gains[parity].re*p->im + gains[parity].im*p->re;
            error.re = s->startup_rx_acq[i].re - predicted.re;
            error.im = s->startup_rx_acq[i].im - predicted.im;
            error_power[parity] += startup_power(&error);
            input_power += startup_power(&s->startup_rx_acq[i]);
        }
        metric = error_power[parity]/input_power;
        if (metric < best_metric)
        {
            best_metric = metric;
            best = parity;
        }
    }
    if (best < 0  ||  best_metric > 0.08f)
        return false;
    s->startup_rx_gain = gains[best];
#if !defined(SPANDSP_USE_FIXED_POINTx)
    /* Put the S-derived complex gain into the shared FSE immediately.  TRN
       can then train against canonical 4-point targets instead of asking LMS
       to remove an arbitrary carrier rotation and the channel together. */
    reference_power = startup_power(&s->startup_rx_gain);
    if (reference_power > 1.0e-6f)
    {
        complexf_t inverse;
        complexf_t tap;

        inverse.re = s->startup_rx_gain.re/reference_power;
        inverse.im = -s->startup_rx_gain.im/reference_power;
        for (i = 0;  i < V17_EQUALIZER_LEN;  i++)
        {
            tap = s->rx.eq_coeff[i];
            s->rx.eq_coeff[i].re = tap.re*inverse.re - tap.im*inverse.im;
            s->rx.eq_coeff[i].im = tap.re*inverse.im + tap.im*inverse.re;
        }
        s->startup_rx_gain.re = 1.0f;
        s->startup_rx_gain.im = 0.0f;
    }
#endif
    s->startup_rx_sbar_run = 0;
    return true;
}
/*- End of function --------------------------------------------------------*/

static int startup_nearest_state(const v32bis_state_t *s, const complexf_t *z)
{
    complexf_t predicted;
    float distance;
    float best_distance;
    int best;
    int state;

    best = 0;
    best_distance = 1.0e30f;
    for (state = 0;  state < 4;  state++)
    {
        predicted.re = s->startup_rx_gain.re*v17_v32bis_4800_constellation[state].re
                     - s->startup_rx_gain.im*v17_v32bis_4800_constellation[state].im;
        predicted.im = s->startup_rx_gain.re*v17_v32bis_4800_constellation[state].im
                     + s->startup_rx_gain.im*v17_v32bis_4800_constellation[state].re;
        predicted.re = z->re - predicted.re;
        predicted.im = z->im - predicted.im;
        distance = startup_power(&predicted);
        if (distance < best_distance)
        {
            best_distance = distance;
            best = state;
        }
    }
    return best;
}
/*- End of function --------------------------------------------------------*/

static void startup_track_gain(v32bis_state_t *s, const complexf_t *z, int state)
{
    complexf_t observation;

    observation = startup_gain_observation(z, state);
    s->startup_rx_gain.re = 0.99f*s->startup_rx_gain.re + 0.01f*observation.re;
    s->startup_rx_gain.im = 0.99f*s->startup_rx_gain.im + 0.01f*observation.im;
}
/*- End of function --------------------------------------------------------*/

/*! B1 is "binary ones scrambled and encoded as for the subsequent transmission
    of data" (6.1, 6.2), and the receiver trains on it as a known sequence.  The
    scrambler runs on from E and the convolutional encoder's delay elements
    "shall be set to zero"; the Recommendation does not say where the trellis
    path's differential encoder starts -- V.32's own Figure 2 draws it as a
    block separate from the convolutional encoder, and only the latter is
    zeroed.  This modem continues it from E's final symbol.  slmodemd starts
    it from Y1Y2 = 00: its B1, demodulated off the line, matches that reading
    in 128 of 128 symbols and this modem's in none, and a receiver that
    trained 128 symbols of equalizer on the wrong reading entered data mode
    white and never recovered.  Both readings are generated, the first
    V32BIS_B1_VOTE_SYMBOLS are decided on whichever lies nearer, and the
    reading that was nearer more often is kept. */
#define V32BIS_B1_VOTE_SYMBOLS  16

static void startup_b1_reset_vote(v32bis_state_t *s)
{
    s->startup_rx_b1_diff_alt = 0;
    s->startup_rx_b1_convolution_alt = 0;
    s->startup_rx_b1_votes = 0;
    s->startup_rx_b1_voted = 0;
}
/*- End of function --------------------------------------------------------*/

static int startup_b1_symbol(v32bis_state_t *s, const complexf_t *z)
{
    static const uint8_t differential_4800[4][4] =
    {
        {2, 3, 0, 1}, {0, 2, 1, 3}, {3, 1, 2, 0}, {1, 0, 3, 2}
    };
    static const uint8_t differential_coded[4][4] =
    {
        {0, 1, 2, 3}, {1, 2, 3, 0}, {2, 3, 0, 1}, {3, 0, 1, 2}
    };
    static const uint8_t convolutional[8][4] =
    {
        {0, 2, 3, 1}, {4, 7, 5, 6}, {1, 3, 2, 0}, {7, 4, 6, 5},
        {2, 0, 1, 3}, {6, 5, 7, 4}, {3, 1, 0, 2}, {5, 6, 4, 7}
    };
    int bits;
    int bit;
    int i;
    int tap;
    int primary;
    int alt;
    bool alt_nearer;

    tap = (!s->calling_party) ? 17 : 4;
    bits = 0;
    for (i = 0;  i < s->rx.bits_per_symbol;  i++)
    {
        bit = startup_scramble_bit(&s->startup_rx_b1_reg, tap, 1);
        bits |= bit << i;
    }
    if (s->rx.bits_per_symbol == 2)
    {
        s->startup_rx_b1_diff = differential_4800[s->startup_rx_b1_diff][bits & 3];
        return s->startup_rx_b1_diff;
    }
    s->startup_rx_b1_diff = differential_coded[s->startup_rx_b1_diff][bits & 3];
    s->startup_rx_b1_convolution =
        convolutional[s->startup_rx_b1_convolution][s->startup_rx_b1_diff];
    primary = ((bits << 1) & 0x78)
            | (s->startup_rx_b1_diff << 1)
            | ((s->startup_rx_b1_convolution >> 2) & 1);
    if (s->startup_rx_b1_voted >= V32BIS_B1_VOTE_SYMBOLS)
        return primary;
    /*endif*/
    s->startup_rx_b1_diff_alt = differential_coded[s->startup_rx_b1_diff_alt][bits & 3];
    s->startup_rx_b1_convolution_alt =
        convolutional[s->startup_rx_b1_convolution_alt][s->startup_rx_b1_diff_alt];
    alt = ((bits << 1) & 0x78)
        | (s->startup_rx_b1_diff_alt << 1)
        | ((s->startup_rx_b1_convolution_alt >> 2) & 1);
    {
        float pr = z->re - s->rx.constellation[primary].re;
        float pi = z->im - s->rx.constellation[primary].im;
        float ar = z->re - s->rx.constellation[alt].re;
        float ai = z->im - s->rx.constellation[alt].im;

        alt_nearer = (ar*ar + ai*ai < pr*pr + pi*pi);
    }
    s->startup_rx_b1_votes += alt_nearer  ?  -1  :  1;
    if (++s->startup_rx_b1_voted >= V32BIS_B1_VOTE_SYMBOLS
        &&  s->startup_rx_b1_votes < 0)
    {
        /* The other reading won: carry on from its state. */
        s->startup_rx_b1_diff = s->startup_rx_b1_diff_alt;
        s->startup_rx_b1_convolution = s->startup_rx_b1_convolution_alt;
        if (v32bis_trace())
        {
            fprintf(stderr,
                    "[V32BIS %s] B1's trellis differential state starts at 00 (%d votes)\n",
                    s->calling_party ? "call  " : "answer",
                    s->startup_rx_b1_votes);
        }
        /*endif*/
    }
    /*endif*/
    return alt_nearer  ?  alt  :  primary;
}
/*- End of function --------------------------------------------------------*/

/*! Point the V.17 receiver at a V.32bis rate's constellation, and nothing
    else.  Clause 8 changes the data signalling rate without retraining, so
    the equalizer, carrier loop and timing loop are the trained solution for
    a channel that has not changed and must be left exactly as they are. */
static int v32bis_rx_set_rate(v32bis_state_t *s, int bit_rate)
{
    switch (bit_rate)
    {
    case 14400:
        s->rx.constellation = v17_v32bis_14400_constellation;
        s->rx.space_map = 0;
        s->rx.bits_per_symbol = 6;
        break;
    case 12000:
        s->rx.constellation = v17_v32bis_12000_constellation;
        s->rx.space_map = 1;
        s->rx.bits_per_symbol = 5;
        break;
    case 9600:
        s->rx.constellation = v17_v32bis_9600_constellation;
        s->rx.space_map = 2;
        s->rx.bits_per_symbol = 4;
        break;
    case 7200:
        s->rx.constellation = v17_v32bis_7200_constellation;
        s->rx.space_map = 3;
        s->rx.bits_per_symbol = 3;
        break;
    case 4800:
        s->rx.constellation = v17_v32bis_4800_constellation;
        s->rx.space_map = 0;
        s->rx.bits_per_symbol = 2;
        break;
    default:
        return -1;
    }
    /*endswitch*/
    s->rx.bit_rate = bit_rate;
    return 0;
}
/*- End of function --------------------------------------------------------*/

static int startup_enter_data_rx(v32bis_state_t *s,
                                 int bit_rate,
                                 uint32_t descrambler_register,
                                 int diff_state)
{
    complexf_t inverse;
    complexf_t tap;
    float gain_power;
    int i;

    if (v32bis_rx_set_rate(s, bit_rate) != 0)
        return -1;
    /* The startup detector estimates the one-tap complex channel while the
       V.17 FSE remains in its neutral state.  Transfer that estimate into the
       FSE before handing dense data decisions back to V.17. */
    gain_power = startup_power(&s->startup_rx_gain);
    if (gain_power <= 1.0e-6f)
        return -1;
    inverse.re = s->startup_rx_gain.re/gain_power;
    inverse.im = -s->startup_rx_gain.im/gain_power;
#if defined(SPANDSP_USE_FIXED_POINTx)
    (void) tap;
    (void) inverse;
#else
    for (i = 0;  i < V17_EQUALIZER_LEN;  i++)
    {
        tap = s->rx.eq_coeff[i];
        s->rx.eq_coeff[i].re = tap.re*inverse.re - tap.im*inverse.im;
        s->rx.eq_coeff[i].im = tap.re*inverse.im + tap.im*inverse.re;
    }
#endif
    s->rx.diff = diff_state;
    s->rx.scramble_reg = descrambler_register;
    s->rx.training_stage = 0;  /* TRAINING_STAGE_NORMAL_OPERATION */
    s->rx.training_count = 0;
    for (i = 0;  i < 8;  i++)
        s->rx.distances[i] = (i == 0) ? 0.0f : 99.0f;
    memset(s->rx.full_path_to_past_state_locations, 0,
           sizeof(s->rx.full_path_to_past_state_locations));
    memset(s->rx.past_state_locations, 0,
           sizeof(s->rx.past_state_locations));
    s->rx.trellis_ptr = 14;
    s->rx.symbol_sink = NULL;
    s->rx.symbol_sink_user_data = NULL;
    return 0;
}
/*- End of function --------------------------------------------------------*/

/*! 6.2: "On detection of an incoming S sequence, the modem shall cease
    transmitting", then wait MT and train on the S that persists or reappears.
    The hold is what keeps the answer modem's receiver off 6.1's NT-length S,
    which is the round-trip probe rather than the conditioning signal. */
static void v32bis_rx_event_s(v32bis_state_t *s)
{
    if (!s->reactive_startup  ||  s->rx_s_event_sent)
        return;
    /*endif*/
    if (v32bis_trace())
    {
        fprintf(stderr, "[V32BIS %s] incoming S at rx symbol %d\n",
                s->calling_party ? "call  " : "answer",
                s->startup_rx_symbol_count);
    }
    /*endif*/
    s->rx_s_event_sent = true;
    if (s->calling_party  ||  s->tx_phase != V32BIS_TX_PHASE_R  ||  s->tx_step > 1)
        return;
    /*endif*/
    s->tx_released = true;
    if (s->mt_symbols > 0)
    {
        /* Drop this acquisition and take the next one, after MT. */
        s->rx_hold_symbols = s->mt_symbols;
        s->startup_rx_stage = V32BIS_RX_SEARCH_S;
        s->startup_rx_acq_count = 0;
    }
    /*endif*/
}
/*- End of function --------------------------------------------------------*/

/*! A validated Table 5 rate signal from the far end releases whichever phase
    of the script was waiting on it. */
static void v32bis_rx_event_rate(v32bis_state_t *s, int rates)
{
    if (v32bis_trace())
    {
        fprintf(stderr, "[V32BIS %s] rate signal 0x%03x at rx symbol %d\n",
                s->calling_party ? "call  " : "answer",
                rates, s->startup_rx_symbol_count);
    }
    /*endif*/
    s->startup_remote_rates = rates;
    /* A rate signal naming exactly one rate is R3, which is what 6.1 has the
       call modem echo in its E word. */
    switch (rates)
    {
    case V32BIS_RATE_4800:
        s->startup_selected_rate = 4800;
        break;
    case V32BIS_RATE_7200:
        s->startup_selected_rate = 7200;
        break;
    case V32BIS_RATE_9600:
        s->startup_selected_rate = 9600;
        break;
    case V32BIS_RATE_12000:
        s->startup_selected_rate = 12000;
        break;
    case V32BIS_RATE_14400:
        s->startup_selected_rate = 14400;
        break;
    }
    /*endswitch*/
    if (!s->reactive_startup)
        return;
    /*endif*/
    if (s->tx_phase == V32BIS_TX_PHASE_SILENT  ||  s->tx_phase == V32BIS_TX_PHASE_R)
        s->tx_released = true;
    /*endif*/
}
/*- End of function --------------------------------------------------------*/

static void startup_finish_word(v32bis_state_t *s)
{
    uint32_t reg;
    uint16_t word;
    int diff;
    int rates;
    int rate;
    bool remote_calling_party;

    reg = s->startup_rx_trn_reg;
    diff = s->startup_rx_trn_diff;
    remote_calling_party = !s->calling_party;
    if (v32bis_decode_startup_word(remote_calling_party,
                                   s->startup_rx_word_states,
                                   &reg,
                                   &diff,
                                   &word) != 8)
    {
        s->startup_rx_stage = V32BIS_RX_SEARCH_S;
        return;
    }
    /* 5.3: one scrambled, differentially encoded stream from the end of TRN
       through every repeated sequence and E to B1, so the descrambler and the
       differential decoder carry on from word to word. */
    s->startup_rx_trn_reg = reg;
    s->startup_rx_trn_diff = diff;
    if (v32bis_trace())
    {
        fprintf(stderr, "[V32BIS %s] rx word 0x%04x in stage %d at rx symbol %d\n",
                s->calling_party ? "call  " : "answer",
                word, s->startup_rx_stage, s->startup_rx_symbol_count);
    }
    /*endif*/
    if (s->startup_rx_stage == V32BIS_RX_R_FIRST)
    {
        if (v32bis_decode_rate_signal(word, &rates) != 0)
        {
            s->startup_rx_stage = V32BIS_RX_SEARCH_S;
            return;
        }
        s->startup_rx_first_r = word;
        s->startup_remote_rates = rates;
        s->startup_rx_stage = V32BIS_RX_R_SECOND;
    }
    else if (s->startup_rx_stage == V32BIS_RX_R_SECOND)
    {
        if (word != s->startup_rx_first_r
            || v32bis_decode_rate_signal(word, &rates) != 0)
        {
            s->startup_rx_stage = V32BIS_RX_SEARCH_S;
            return;
        }
        /* 6.1/6.2 both ask for "at least two consecutive identical" rate
           sequences before the rate signal is believed. */
        s->rx_repeat_word = word;
        v32bis_rx_event_rate(s, rates);
        s->startup_rx_stage = V32BIS_RX_E;
    }
    else if (s->startup_rx_stage == V32BIS_RX_E)
    {
        if (word == s->rx_repeat_word)
        {
            /* The far end is still repeating the rate signal while it waits
               for our answer.  An E word cannot be mistaken for one: Table 6's
               fixed bits fail Table 5's sync test. */
            s->startup_rx_word_pos = 0;
            return;
        }
        /*endif*/
        if (v32bis_decode_rate_signal(word, &rates) == 0)
        {
            /* A different rate signal.  For the call modem this is R3
               arriving after R1. */
            s->rx_repeat_word = word;
            v32bis_rx_event_rate(s, rates);
            s->startup_rx_word_pos = 0;
            return;
        }
        /*endif*/
        if (v32bis_decode_e_signal(word, &rate) != 0
            || (rate_to_mask(rate) & s->startup_remote_rates) == 0)
        {
            s->startup_rx_stage = V32BIS_RX_SEARCH_S;
            return;
        }
        if (startup_enter_data_rx(s, rate, reg, diff) != 0)
        {
            s->startup_rx_stage = V32BIS_RX_SEARCH_S;
            return;
        }
        if (v32bis_trace())
        {
            fprintf(stderr, "[V32BIS %s] E word %d bit/s at rx symbol %d\n",
                    s->calling_party ? "call  " : "answer",
                    rate, s->startup_rx_symbol_count);
        }
        /*endif*/
        s->startup_selected_rate = rate;
        s->bit_rate = rate;
        /* 6.2: the answer modem answers the incoming E with its own, after
           completing the rate sequence it is in the middle of. */
        if (s->reactive_startup  &&  s->tx_phase == V32BIS_TX_PHASE_R)
            s->tx_released = true;
        /*endif*/
        s->startup_rx_b1_pos = 0;
        s->startup_rx_b1_reg = reg;
        s->startup_rx_b1_diff = diff;
        s->startup_rx_b1_convolution = 0;
        startup_b1_reset_vote(s);
        s->rx_b1_target = V32BIS_B1_SYMBOLS;
        s->startup_rx_stage = V32BIS_RX_B1;
        s->rx.symbol_sink = v32bis_startup_symbol_sink;
        s->rx.symbol_sink_user_data = s;
        s->rx.symbol_sink_uses_data_constellation = true;
    }
    s->startup_rx_word_pos = 0;
}
/*- End of function --------------------------------------------------------*/

#if defined(SPANDSP_USE_FIXED_POINTx)
static int v32bis_startup_symbol_sink(void *user_data, const complexi16_t *symbol)
#else
static int v32bis_startup_symbol_sink(void *user_data, const complexf_t *symbol)
#endif
{
    v32bis_state_t *s;
    complexf_t z;
    int remote_tap;
    int expected;
    int state;

    s = (v32bis_state_t *) user_data;
#if defined(SPANDSP_USE_FIXED_POINTx)
    z.re = symbol->re;
    z.im = symbol->im;
#else
    z = *symbol;
#endif
    s->startup_rx_symbol_count++;
    switch (s->startup_rx_stage)
    {
    case V32BIS_RX_SEARCH_S:
        if (s->rx_hold_symbols > 0)
        {
            s->rx_hold_symbols--;
            return -1;
        }
        /*endif*/
        if (startup_power(&z) < 10.0f)
            return -1;
        if (s->startup_rx_acq_count < 64)
            s->startup_rx_acq[s->startup_rx_acq_count++] = z;
        else
        {
            memmove(&s->startup_rx_acq[0],
                    &s->startup_rx_acq[1],
                    63*sizeof(s->startup_rx_acq[0]));
            s->startup_rx_acq[63] = z;
        }
        if (startup_try_acquire_s(s))
        {
            s->startup_rx_stage = V32BIS_RX_S_BAR;
            v32bis_rx_event_s(s);
        }
        /*endif*/
        return -1;
    case V32BIS_RX_S_BAR:
        state = startup_nearest_state(s, &z);
        if (s->startup_rx_sbar_run == 0)
        {
            if (state == V32BIS_STARTUP_C)
                s->startup_rx_sbar_run = 1;
        }
        else
        {
            expected = (s->startup_rx_sbar_run & 1)
                     ? V32BIS_STARTUP_D : V32BIS_STARTUP_C;
            if (state == expected)
                s->startup_rx_sbar_run++;
            else
                s->startup_rx_sbar_run = (state == V32BIS_STARTUP_C) ? 1 : 0;
        }
        if (s->startup_rx_sbar_run >= 8)
        {
            s->startup_rx_sbar_remaining = 8;
            s->startup_rx_stage = V32BIS_RX_TRN;
            s->startup_rx_trn_pos = -8;
            s->startup_rx_trn_reg = 0;
            s->startup_rx_trn_diff = V32BIS_STARTUP_A;
        }
        return -1;
    case V32BIS_RX_TRN:
        if (s->startup_rx_trn_pos < 0)
        {
            s->startup_rx_trn_pos++;
            return -1;
        }
        remote_tap = (!s->calling_party) ? 17 : 4;
        expected = startup_scramble_bit(&s->startup_rx_trn_reg, remote_tap, 1);
        state = startup_scramble_bit(&s->startup_rx_trn_reg, remote_tap, 1);
        if (s->startup_rx_trn_pos < 256)
            expected = expected ? V32BIS_STARTUP_C : V32BIS_STARTUP_A;
        else
            expected |= state << 1;
        startup_track_gain(s, &z, expected);
        s->startup_rx_trn_diff = expected;
        /* The V.17 LMS step is unnormalized and fixed.  Held at the fast value
           for the whole of TRN it converges in a couple of hundred symbols and
           then adds gradient noise for the remaining thousand: measured over
           TRN the equalized error RISES from 0.23 to 0.55 of a unit while
           carrier phase stays inside 0.1 degree and gain at 1.00, so the
           residual is dispersive, not a loop that has lost lock.  A 4 point
           decision does not care; a 128 point one cannot survive it.  Converge
           fast, then anneal. */
        /* v17_rx_restart() clears this, and it runs after init, so it has to be
           (re)asserted from inside the startup path rather than at init. */
        /* V.17 latches the AGC as it leaves its own training stages.  The
           V.32bis path takes over the symbol stream before those stages run,
           so nothing ever latched it and the AGC kept re-deriving its scaling
           from the instantaneous power meter on every T/2 sample -- a white,
           amplitude-proportional gain error of about 10% for the whole call.
           S has run for 256 symbols by here, so the level is settled. */
        if (s->rx.agc_scaling_save == 0.0f)
            s->rx.agc_scaling_save = s->rx.agc_scaling;
        /*endif*/
        s->rx.eq_normalized_lms = v32bis_use_nlms();
        s->rx.v32bis_eye_log = (getenv("V32BIS_DATA_EYE") != NULL);
        s->rx.v32bis_data_eq = v32bis_data_eq();
        s->rx.v32bis_timing_hold = (getenv("V32BIS_TIMING_HOLD") != NULL);
        s->rx.v32bis_carrier_hold = (getenv("V32BIS_CARRIER_HOLD") != NULL);
        s->rx.eq_delta = (s->startup_rx_trn_pos < V32BIS_TRN_FAST_SYMBOLS)
                       ? V32BIS_EQ_DELTA_FAST
                       : V32BIS_EQ_DELTA_SLOW;
        if (++s->startup_rx_trn_pos >= 1280)
        {
            /* 5.2.3: TRN is "at least 1280 and not exceed 8192 symbol
               intervals", so this is only where the rate signal MAY begin.
               It used to be taken as where it does begin -- our own
               transmitter's length -- and the 8 symbols after it framed as a
               word; against slmodemd, whose TRN ran 5500 to 8200 symbols,
               that "word" was TRN, failed Table 5's sync test, and the
               receiver went back to hunting for S for the rest of the call. */
            s->startup_rx_stage = V32BIS_RX_R_FIRST;
            s->startup_rx_word_pos = 0;
            s->startup_rx_slide_bits = 0;
            s->startup_rx_slide_count = 0;
        }
        return expected;
    case V32BIS_RX_R_FIRST:
        if (startup_power(&z) < 10.0f)
        {
            s->startup_rx_stage = V32BIS_RX_SEARCH_S;
            s->startup_rx_acq_count = 0;
            s->startup_rx_word_pos = 0;
            return -1;
        }
        /*endif*/
        state = startup_nearest_state(s, &z);
        startup_track_gain(s, &z, state);
        /* Decode every symbol as rate signal -- differentially, then through
           the self-synchronising descrambler -- whether or not TRN has ended,
           and look for 6.1/6.2's "two consecutive identical" Table 5
           sequences ending on this symbol.  While TRN is still running this
           decodes noise into the descrambler; 23 bits of rate signal flush
           it, so a TRN longer than the minimum costs one extra sequence and
           nothing else.  The alignment found here frames every word after it. */
        {
            int dibit = startup_differential_decoder[s->startup_rx_trn_diff][state];
            int remote = (!s->calling_party) ? 17 : 4;
            uint32_t b0;
            uint32_t b1;
            uint16_t older;
            uint16_t newer;

            s->startup_rx_trn_diff = state;
            b0 = (uint32_t) startup_descramble_bit(&s->startup_rx_trn_reg, remote, dibit & 1);
            b1 = (uint32_t) startup_descramble_bit(&s->startup_rx_trn_reg, remote, (dibit >> 1) & 1);
            s->startup_rx_slide_bits = (s->startup_rx_slide_bits >> 2) | (b0 << 30) | (b1 << 31);
            if (++s->startup_rx_slide_count >= 16)
            {
                older = (uint16_t) (s->startup_rx_slide_bits & 0xFFFF);
                newer = (uint16_t) (s->startup_rx_slide_bits >> 16);
                if (older == newer
                    &&  v32bis_decode_rate_signal(newer, &expected) == 0)
                {
                    if (v32bis_trace())
                    {
                        fprintf(stderr,
                                "[V32BIS %s] rate signal framed after %d symbols of TRN\n",
                                s->calling_party ? "call  " : "answer",
                                1280 + s->startup_rx_slide_count - 16);
                    }
                    /*endif*/
                    s->startup_rx_first_r = newer;
                    s->startup_remote_rates = expected;
                    s->rx_repeat_word = newer;
                    v32bis_rx_event_rate(s, expected);
                    s->startup_rx_stage = V32BIS_RX_E;
                    s->startup_rx_word_pos = 0;
                    return -1;
                }
                /*endif*/
            }
            /*endif*/
            /* Past the longest TRN 5.2.3 allows, with room for a far end
               whose count runs a little over (slmodemd's has been seen at
               8208 by this receiver's reckoning). */
            if (s->startup_rx_slide_count > 8192 - 1280 + 512)
            {
                s->startup_rx_stage = V32BIS_RX_SEARCH_S;
                s->startup_rx_acq_count = 0;
                return -1;
            }
            /*endif*/
        }
        /* Nothing is handed back for the loops to train on: the equalizer
           leaves TRN trained and is not adapted again until B1.  Training it
           decision-directed on the 4 point decisions here instead was tried
           against slmodemd's ~6900 extra symbols of TRN and measured: the
           data mode after it read 0.68 from the 9600 constellation and went
           white, against 0.13 and every line intact without it. */
        return -1;
    case V32BIS_RX_R_SECOND:
    case V32BIS_RX_E:
        if (startup_power(&z) < 10.0f)
        {
            /* The far end has stopped transmitting.  Go back to looking for
               the next conditioning signal rather than framing silence. */
            s->startup_rx_stage = V32BIS_RX_SEARCH_S;
            s->startup_rx_acq_count = 0;
            s->startup_rx_word_pos = 0;
            return -1;
        }
        /*endif*/
        state = startup_nearest_state(s, &z);
        /* Keep the one-tap channel estimate current.  In the clause 6 dialogue
           a rate signal can repeat for well over a thousand symbols while the
           far end works through its own script, and startup_enter_data_rx()
           hands this estimate to the FSE at the E handoff: measured on the
           duplex harness, an estimate left at its TRN value costs the answer
           modem its whole data phase at 12000 and 14400. */
        startup_track_gain(s, &z, state);
        s->startup_rx_word_states[s->startup_rx_word_pos++] = (uint8_t) state;
        if (s->startup_rx_word_pos >= 8)
            startup_finish_word(s);
        return -1;
    case V32BIS_RX_RENEG_R:
    case V32BIS_RX_RENEG_E:
        /* 8: the rate signal and E are 4 point signals at 4800 bit/s, but
           the equalizer and carrier loop are the trained solution for the
           data mode this came out of and are left alone -- clause 8's whole
           promise is that no retraining happens.  Only the one tap estimate
           the 4 point slicer uses is kept current. */
        state = startup_nearest_state(s, &z);
        startup_track_gain(s, &z, state);
        v32bis_reneg_rx_symbol(s, state);
        /* The current symbol is still the final 4-point E symbol; the
           newly selected data constellation applies starting with B1. */
        if (s->startup_rx_stage == V32BIS_RX_B1)
            return -1;
        /* Hand the 4 point decision back, so v17rx.c keeps the carrier loop
           and the equalizer tracking on it.  Returning -1 here leaves both
           coasting for the ~250 symbol intervals a renegotiation takes, and
           the carrier phase that accumulates in that time is enough on its
           own to make the data mode on the far side of it white. */
        return state;
    case V32BIS_RX_B1:
        state = startup_b1_symbol(s, &z);
        if (++s->startup_rx_b1_pos >= s->rx_b1_target)
        {
            s->rx.scramble_reg = s->startup_rx_b1_reg;
            s->rx.diff = s->startup_rx_b1_diff;
            for (expected = 0;  expected < 8;  expected++)
                s->rx.distances[expected] = 99.0f;
            s->rx.distances[s->startup_rx_b1_convolution] = 0.0f;
            memset(s->rx.full_path_to_past_state_locations, 0,
                   sizeof(s->rx.full_path_to_past_state_locations));
            memset(s->rx.past_state_locations, 0,
                   sizeof(s->rx.past_state_locations));
            s->rx.trellis_ptr = 14;
            /* The trellis emits the symbol from t - (V17_TRELLIS_LOOKBACK_DEPTH - 1),
               so its first 15 symbols of output are traceback fill rather than
               B1.  Those bits must be neither delivered to circuit 104 nor
               shifted into the descrambler: scramble_reg is seeded above with
               the last 23 line bits of B1, which is exactly what the first real
               data bit needs, and letting the fill in destroys it.  The uncoded
               4800 bit/s mode has no trellis and so no fill to hide. */
            s->rx.v32bis_data_bits_suppress =
                (s->rx.bits_per_symbol == 2)
              ? 0
              : (V17_TRELLIS_LOOKBACK_DEPTH - 1)*s->rx.bits_per_symbol;
            s->startup_complete = true;
            if (s->reneg_active)
            {
                s->reneg_active = false;
                s->reneg_count++;
                s->reneg_initiator = false;
                s->reneg_far_preamble = false;
                s->reneg_r_seen = false;
                s->reneg_local_rates = 0;
                s->reneg_remote_rates = 0;
                s->reneg_preamble_run = 0;
                if (v32bis_trace())
                {
                    fprintf(stderr,
                            "[V32BIS %s] 8: renegotiation %d complete at %d bit/s\n",
                            s->calling_party ? "call  " : "answer",
                            s->reneg_count,
                            s->bit_rate);
                }
                /*endif*/
            }
            /*endif*/
            s->startup_rx_stage = V32BIS_RX_DATA;
            s->rx.symbol_sink = NULL;
            s->rx.symbol_sink_user_data = NULL;
            /* Keep the data target for this final B1 callback.  The sink
               is detached, so the flag is irrelevant to following data. */
        }
        return state;
    case V32BIS_RX_DATA:
    default:
        return -1;
    }
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v32bis_prepare_startup_tx(v32bis_state_t *s, int remote_rates)
{
    uint8_t word_states[8];
    uint16_t word;
    uint32_t trn_reg;
    uint32_t word_reg;
    int trn_diff;
    int word_diff;
    int local_rates;
    int offered_rates;
    int selected_rate;
    int count;
    int i;

    if (s == NULL
        || v32bis_decode_rate_signal(s->permitted_rates_signal, &local_rates) != 0
        || !valid_rate_mask(remote_rates))
        return -1;
    offered_rates = local_rates & remote_rates;
    selected_rate = v32bis_select_common_rate(local_rates, remote_rates);
    if (selected_rate == 0)
        return -1;
    /* Select the post-E data constellation before the burst starts.  The
       external startup source bypasses it until E is complete, so this reset
       cannot introduce a waveform seam. */
    if (v17_tx_restart(&s->tx, selected_rate, false, false) != 0)
        return -1;

    count = v32bis_build_conditioning(s->calling_party,
                                      1280,
                                      s->startup_tx_symbols,
                                      &trn_reg,
                                      &trn_diff);
    if (count < 0  ||  v32bis_build_rate_signal(offered_rates, &word) != 0)
        return -1;
    /* Section 6 requires at least two identical consecutive R words.  5.3
       scrambles and differentially encodes the rate signal as one stream, so
       the second is encoded on from the first, and E on from that (see
       v32bis_tx_fill_word()). */
    word_reg = trn_reg;
    word_diff = trn_diff;
    for (i = 0;  i < 2;  i++)
    {
        if (v32bis_encode_startup_word(s->calling_party,
                                       word,
                                       &word_reg,
                                       &word_diff,
                                       word_states) != 8)
            return -1;
        memcpy(&s->startup_tx_symbols[count], word_states, sizeof(word_states));
        count += (int) sizeof(word_states);
    }
    /*endfor*/

    if (v32bis_build_e_signal(selected_rate, &word) != 0)
        return -1;
    if (v32bis_encode_startup_word(s->calling_party,
                                   word,
                                   &word_reg,
                                   &word_diff,
                                   word_states) != 8)
        return -1;
    memcpy(&s->startup_tx_symbols[count], word_states, sizeof(word_states));
    count += (int) sizeof(word_states);

    s->startup_tx_symbol_count = count;
    s->startup_tx_symbol_pos = 0;
    s->bit_rate = selected_rate;
    /* Section 6.1: E hands its scrambler/differential state to B1/data and
       initializes the convolutional encoder to zero. */
    s->tx.scramble_reg = word_reg;
    s->tx.diff = word_diff;
    s->tx.convolution = 0;
    s->tx.v32bis_b1_bits_remaining = V32BIS_B1_SYMBOLS*s->tx.bits_per_symbol;
    s->tx.in_training = false;
    s->tx.current_get_bit = s->tx.get_bit;
    s->tx.symbol_source = v32bis_startup_symbol_source;
    s->tx.symbol_source_user_data = s;
    return count;
}
/*- End of function --------------------------------------------------------*/

/* ITU-T V.32bis 6.1/6.2 tone phases.

   Figure 2-5's carrier states are four points 90 degrees apart, so a modem
   repeating state A puts a pure 1800 Hz tone on the line, and one alternating
   A and C puts a suppressed-carrier pair at 1800 -/+ 1200 Hz, which is the
   600 Hz and 3000 Hz the call modem is told to look for.  State C is state A
   turned through 180 degrees, so every transition clause 6 calls a "phase
   reversal" -- AA to CC, AC to CA and CA back to AC -- is a sign change of
   the whole waveform, and shows in whichever tone is being tracked.

   Timing matters here: 6.1 and 6.2 both require the scheduled transition to
   appear at the line terminals 64 +/- 2 symbol intervals after the reversal
   that caused it arrives there, and the difference between the two modems'
   counters is the round-trip delay they then use as NT and MT.  So the
   detector has to place the reversal in time, not merely notice it. */

/*! The transmit pulse shaper's group delay, in symbol intervals.  A symbol
    handed to v17_tx() appears at the line this much later, so it is what
    converts 6.1's and 6.2's "at the line terminals" into a transmit symbol
    index.  It is MEASURED, not derived: the 9 symbol-spaced taps suggest 4,
    and the interpolating structure -- which indexes the coefficient sets as
    TX_PULSESHAPER_COEFF_SETS - 1 - baud_phase -- makes it 3.  The duplex
    harness reads the delay off the two ends' transmit symbol indices, where
    the shaper delay does not cancel, and 3 is the value that puts it exactly
    on 64 + the one-way delay at 0, 80 and 240 samples, and NT exactly on
    128 + the round trip.  4 puts every row one symbol low, 5 two. */
#define V32BIS_TX_SHAPER_DELAY_SYMBOLS  3
/*! 10/3 samples per symbol at 2400 baud and 8000 samples/s. */
#define V32BIS_SAMPLES_PER_SYMBOL       (10.0f/3.0f)
/*! 6.1/6.2: the scheduled transition delay. */
#define V32BIS_REVERSAL_DELAY_SYMBOLS   64
/*! 6.2: "an incoming tone has been detected at 1800 +/- 7 Hz for 64 symbol
    periods". */
#define V32BIS_TONE_PRESENT_SYMBOLS     64
/*! 6.2: the answer modem acts on an amplitude drop in the incoming tone. */
#define V32BIS_TONE_DROP_SYMBOLS        16

static void v32bis_tone_det_init(v32bis_tone_det_t *d, float freq)
{
    memset(d, 0, sizeof(*d));
    d->dphase = 2.0f*3.1415926535f*freq/8000.0f;
    d->hold_until = -1;
}
/*- End of function --------------------------------------------------------*/

/*! Returns true on the sample at which a reversal is declared.  The instant
    of the reversal itself is left in d->reversal_sample: the leading window's
    magnitude dips to a minimum when the reversal sits in the middle of it, so
    that minimum, half a window back, is an unbiased estimate of it. */
static bool v32bis_tone_det_rx(v32bis_tone_det_t *d, float x, int32_t now)
{
    complexf_t m;
    complexf_t leaving;
    complexf_t oldest;
    float dot;
    float mag2;
    float best;
    int best_i;
    int i;
    int idx;

    m.re = x*cosf(d->phase);
    m.im = -x*sinf(d->phase);
    d->phase += d->dphase;
    if (d->phase > 2.0f*3.1415926535f)
        d->phase -= 2.0f*3.1415926535f;
    /*endif*/
    idx = (int) (d->pos%(2*V32BIS_TONE_WINDOW));
    leaving = d->ring[(idx + 2*V32BIS_TONE_WINDOW - V32BIS_TONE_WINDOW)%(2*V32BIS_TONE_WINDOW)];
    oldest = d->ring[idx];
    d->ring[idx] = m;
    d->s1.re += m.re - leaving.re;
    d->s1.im += m.im - leaving.im;
    d->s2.re += leaving.re - oldest.re;
    d->s2.im += leaving.im - oldest.im;
    d->mag = sqrtf(d->s1.re*d->s1.re + d->s1.im*d->s1.im);
    d->mag_hist[idx] = d->mag;
    if (d->mag > d->peak)
        d->peak = d->mag;
    else
        d->peak *= 0.99995f;
    /*endif*/
    d->pos++;
    d->reversal = false;
    if (d->pos < 2*V32BIS_TONE_WINDOW  ||  now < d->hold_until)
        return false;
    /*endif*/
    mag2 = sqrtf(d->s2.re*d->s2.re + d->s2.im*d->s2.im);
    if (d->mag < 0.4f*d->peak  ||  mag2 < 0.4f*d->peak  ||  d->peak < 100.0f)
        return false;
    /*endif*/
    dot = d->s1.re*d->s2.re + d->s1.im*d->s2.im;
    if (dot > -0.5f*d->mag*mag2)
        return false;
    /*endif*/
    /* Where did the leading window dip? */
    best = -1.0f;
    best_i = 0;
    for (i = 0;  i < 2*V32BIS_TONE_WINDOW;  i++)
    {
        int j = (int) ((d->pos - 1 - i)%(2*V32BIS_TONE_WINDOW));

        if (best < 0.0f  ||  d->mag_hist[j] < best)
        {
            best = d->mag_hist[j];
            best_i = i;
        }
        /*endif*/
    }
    /*endfor*/
    d->reversal = true;
    d->reversal_sample = now - best_i - V32BIS_TONE_WINDOW/2;
    d->hold_until = now + 2*V32BIS_TONE_WINDOW;
    return true;
}
/*- End of function --------------------------------------------------------*/

/*! Convert an instant at the line terminals into the transmit symbol index
    whose own arrival at the line is 64 symbol intervals after it. */
static int32_t v32bis_schedule_reversal(int32_t line_sample)
{
    float target;

    target = (float) line_sample
           + V32BIS_REVERSAL_DELAY_SYMBOLS*V32BIS_SAMPLES_PER_SYMBOL;
    return (int32_t) lrintf(target/V32BIS_SAMPLES_PER_SYMBOL)
         - V32BIS_TX_SHAPER_DELAY_SYMBOLS;
}
/*- End of function --------------------------------------------------------*/

/* ITU-T V.32bis 6.  The reactive start-up machine.

   Figure 3 is a dialogue, and the two roles read as one script each:

     call   SILENT -> (R1) S(NT) COND R2... -> (R3) E -> data
     answer COND R1... -> (S) SILENT -> (R2) COND R3... -> (E) E -> data

   Everything after an arrow is generated only once the bracketed event has
   arrived from the receiver, and a phase that repeats is left at a whole
   16-bit word boundary, as 6.1 and 6.2 both require ("after completing its
   current 16-bit rate sequence"). */

static void v32bis_tx_enter_phase(v32bis_state_t *s, int phase);
static void v32bis_tx_tone_symbol(v32bis_state_t *s);

/*! Build the R word this side should be sending at this point in the script.
    6.1: R2 "shall exclude rates not appearing in the previously received rate
    signal R1".  6.2: the rate selected by R3 "shall be within those indicated
    by R2". */
static int v32bis_tx_build_word(v32bis_state_t *s, uint16_t *word)
{
    int local_rates;
    int rates;
    int rate;

    if (v32bis_decode_rate_signal(s->permitted_rates_signal, &local_rates) != 0)
        return -1;
    if (s->calling_party)
    {
        /* R2. */
        rates = local_rates & s->startup_remote_rates;
        if (rates == 0)
            return -1;
        return v32bis_build_rate_signal(rates, word);
    }
    /*endif*/
    if (s->tx_step <= 1)
    {
        /* R1 advertises what the answer modem and its DTE can do. */
        return v32bis_build_rate_signal(local_rates, word);
    }
    /*endif*/
    /* R3 names the one rate both sides will use. */
    rate = v32bis_select_common_rate(local_rates, s->startup_remote_rates);
    if (rate == 0)
        return -1;
    s->startup_selected_rate = rate;
    return v32bis_build_rate_signal(rate_to_mask(rate), word);
}
/*- End of function --------------------------------------------------------*/

static int v32bis_tx_fill_word(v32bis_state_t *s, uint16_t word)
{
    uint32_t reg;
    int diff;

    /* 5.3: "The rate signal consists of a whole number of repeated 16-bit
       binary sequences ... scrambled and transmitted at 4800 bit/s with
       dibits differentially encoded", the differential encoder initialised
       ONCE, from the final TRN symbol (start-up, retrain) or the final
       preamble symbol with the scrambler zeroed (clause 8).  So the scrambler
       and the differential encoder run on through the repeated sequences and
       into E, and B1 takes over from there.  tx_trn_reg/tx_trn_diff are that
       running state; v32bis_build_conditioning() and the clause 8 RENEG_R
       phase seed it.  The start-up path used to reseed every word from the
       end of TRN, as an inferred policy, so that repeated words came out as
       identical symbols.  Against a foreign modem that is not conformant and
       does not interoperate: slmodemd's R1, decoded with one continuous
       descrambler, reads as 1573 identical valid Table 5 words, and a
       continuous descrambler fed our reseeded words recovers a stable pattern
       that is not a Table 5 word at all, so it never answered our R1. */
    reg = s->tx_trn_reg;
    diff = s->tx_trn_diff;
    if (v32bis_encode_startup_word(s->calling_party,
                                   word,
                                   &reg,
                                   &diff,
                                   s->startup_tx_symbols) != 8)
        return -1;
    s->tx_trn_reg = reg;
    s->tx_trn_diff = diff;
    s->startup_tx_symbol_count = 8;
    s->startup_tx_symbol_pos = 0;
    return 0;
}
/*- End of function --------------------------------------------------------*/

/*! Move the transmitter onto the negotiated data constellation without
    disturbing the waveform.  Everything else v17_tx_restart() would touch is
    either carried across the E handoff by 6.1/6.2 or set by the caller. */
static int v32bis_tx_set_rate(v32bis_state_t *s, int bit_rate)
{
    switch (bit_rate)
    {
    case 14400:
        s->tx.bits_per_symbol = 6;
        s->tx.constellation = v17_v32bis_14400_constellation;
        break;
    case 12000:
        s->tx.bits_per_symbol = 5;
        s->tx.constellation = v17_v32bis_12000_constellation;
        break;
    case 9600:
        s->tx.bits_per_symbol = 4;
        s->tx.constellation = v17_v32bis_9600_constellation;
        break;
    case 7200:
        s->tx.bits_per_symbol = 3;
        s->tx.constellation = v17_v32bis_7200_constellation;
        break;
    case 4800:
        s->tx.bits_per_symbol = 2;
        s->tx.constellation = v17_v32bis_4800_constellation;
        break;
    default:
        return -1;
    }
    /*endswitch*/
    s->tx.bit_rate = bit_rate;
    return 0;
}
/*- End of function --------------------------------------------------------*/

/*! 6.1/6.2: E hands its scrambler and differential state to B1 and the data
    that follows, and zeroes the convolutional encoder. */
static int v32bis_tx_enter_data(v32bis_state_t *s)
{
    uint32_t reg;
    int diff;
    uint16_t word;

    int rate;
    int b1_symbols;

    /* 8.1/8.2: the E word names "the highest data signalling rate common to
       R4 and R5", and B1 is 24T rather than start-up's length. */
    if (s->reneg_active)
    {
        rate = v32bis_select_common_rate(s->reneg_local_rates,
                                         s->reneg_remote_rates);
        b1_symbols = V32BIS_RENEG_B1_SYMBOLS;
    }
    else
    {
        rate = s->startup_selected_rate;
        b1_symbols = V32BIS_B1_SYMBOLS;
    }
    /*endif*/
    /* NOT v17_tx_restart(): it zeroes the pulse shaper history and resets the
       carrier and baud phases, which is harmless before a burst starts and a
       hole in the middle of one.  E sits between the rate signals and B1 with
       no gap, so only the constellation may change here. */
    if (rate == 0
        || v32bis_tx_set_rate(s, rate) != 0
        || v32bis_build_e_signal(rate, &word) != 0)
        return -1;
    reg = s->tx_trn_reg;
    diff = s->tx_trn_diff;
    if (v32bis_encode_startup_word(s->calling_party,
                                   word,
                                   &reg,
                                   &diff,
                                   s->startup_tx_symbols) != 8)
        return -1;
    s->bit_rate = rate;
    s->reneg_selected_rate = rate;
    s->tx.scramble_reg = reg;
    s->tx.diff = diff;
    s->tx.convolution = 0;
    s->tx.v32bis_b1_bits_remaining = b1_symbols*s->tx.bits_per_symbol;
    s->tx.in_training = false;
    s->tx.current_get_bit = s->tx.get_bit;
    return 0;
}
/*- End of function --------------------------------------------------------*/

/*! Build the next chunk of clause 6 Note 3's training sequence.  It may run
    to 8192 symbol intervals, four times the whole 5.2 conditioning signal,
    so it is generated a buffer at a time rather than sized for. */
static bool v32bis_tx_fill_ec_training(v32bis_state_t *s)
{
    int count;

    count = s->ec_train_remaining;
    if (count > V32BIS_EC_TRAIN_CHUNK)
        count = V32BIS_EC_TRAIN_CHUNK;
    /*endif*/
    if (count <= 0)
        return false;
    /*endif*/
    if (v32bis_build_ec_training(s->calling_party,
                                 count,
                                 s->startup_tx_symbols,
                                 &s->ec_train_reg) != count)
        return false;
    /*endif*/
    s->ec_train_remaining -= count;
    s->startup_tx_symbol_pos = 0;
    s->startup_tx_symbol_count = count;
    return true;
}
/*- End of function --------------------------------------------------------*/

/*! Emit one symbol of whichever tone phase is running, after honouring any
    transition scheduled for this symbol.  6.2 requires the alternating
    segments to be an even number of symbol intervals long, and requires the
    CA segment to end on a state A, which the parity of the switch instant is
    what enforces. */
static void v32bis_tx_tone_symbol(v32bis_state_t *s)
{
    int32_t offset;
    int state;

    if (s->tx_transition_at >= 0  &&  s->tx_symbol_index >= s->tx_transition_at)
    {
        s->tx_transition_at = -1;
        s->tone_transition_symbol = s->tx_symbol_index;
        switch (s->tx_phase)
        {
        case V32BIS_TX_PHASE_TONE_A:
            v32bis_tx_enter_phase(s, V32BIS_TX_PHASE_TONE_C);
            return;
        case V32BIS_TX_PHASE_TONE_CA:
            v32bis_tx_enter_phase(s, V32BIS_TX_PHASE_TONE_AC2);
            return;
        default:
            break;
        }
        /*endswitch*/
    }
    /*endif*/
    offset = s->tx_symbol_index - s->tx_phase_start_symbol;
    switch (s->tx_phase)
    {
    case V32BIS_TX_PHASE_TONE_A:
        state = V32BIS_STARTUP_A;
        break;
    case V32BIS_TX_PHASE_TONE_C:
        state = V32BIS_STARTUP_C;
        break;
    case V32BIS_TX_PHASE_TONE_CA:
        state = (offset & 1) ? V32BIS_STARTUP_A : V32BIS_STARTUP_C;
        break;
    default:
        state = (offset & 1) ? V32BIS_STARTUP_C : V32BIS_STARTUP_A;
        break;
    }
    /*endswitch*/
    s->startup_tx_symbols[0] = (uint8_t) state;
    s->startup_tx_symbol_count = 1;
    s->startup_tx_symbol_pos = 0;
}
/*- End of function --------------------------------------------------------*/

static void v32bis_tx_enter_phase(v32bis_state_t *s, int phase)
{
    int count;

    if (v32bis_trace())
    {
        fprintf(stderr, "[V32BIS %s] tx phase %d step %d at rx symbol %d\n",
                s->calling_party ? "call  " : "answer",
                phase, s->tx_step, s->startup_rx_symbol_count);
    }
    /*endif*/
    s->tx_phase = phase;
    s->tx_released = false;
    s->tx_phase_start_symbol = s->tx_symbol_index;
    s->startup_tx_symbol_pos = 0;
    s->startup_tx_symbol_count = 0;
    switch (phase)
    {
    case V32BIS_TX_PHASE_TONE_A:
    case V32BIS_TX_PHASE_TONE_C:
    case V32BIS_TX_PHASE_TONE_AC:
    case V32BIS_TX_PHASE_TONE_CA:
    case V32BIS_TX_PHASE_TONE_AC2:
        v32bis_tx_tone_symbol(s);
        break;
    case V32BIS_TX_PHASE_SILENT:
        break;
    case V32BIS_TX_PHASE_S_NT:
        /* 6.1's NT S sequence.  It is the round-trip estimate, so it is zero
           until the clause 6 tone phases run; the conditioning S that follows
           is what the far end actually trains on. */
        for (count = 0;  count < s->nt_symbols;  count++)
        {
            s->startup_tx_symbols[count] = (count & 1) ? V32BIS_STARTUP_B
                                                       : V32BIS_STARTUP_A;
        }
        /*endfor*/
        s->startup_tx_symbol_count = count;
        break;
    case V32BIS_TX_PHASE_EC_TRAIN:
        /* The far end is silent for the whole of this phase, by
           construction: 6.1's site follows the NT S sequence, which is what
           made the answer modem cease transmitting, and 6.2's follows the
           16 symbol intervals of silence, during which the call modem has
           been silent since its second phase reversal. */
        s->tx_far_end_quiet = true;
        s->ec_train_remaining = s->ec_train_symbols;
        if (!v32bis_tx_fill_ec_training(s))
        {
            s->tx_far_end_quiet = false;
            v32bis_tx_enter_phase(s, V32BIS_TX_PHASE_COND);
            return;
        }
        /*endif*/
        break;
    case V32BIS_TX_PHASE_COND:
        s->tx_far_end_quiet = false;
        count = v32bis_build_conditioning(s->calling_party,
                                          v32bis_trn_symbols(),
                                          s->startup_tx_symbols,
                                          &s->tx_trn_reg,
                                          &s->tx_trn_diff);
        s->startup_tx_symbol_count = (count > 0) ? count : 0;
        break;
    case V32BIS_TX_PHASE_R:
        if (v32bis_tx_build_word(s, &s->tx_word) != 0
            || v32bis_tx_fill_word(s, s->tx_word) != 0)
            s->tx_phase = V32BIS_TX_PHASE_IDLE;
        /*endif*/
        break;
    case V32BIS_TX_PHASE_E:
        if (v32bis_tx_enter_data(s) != 0)
            s->tx_phase = V32BIS_TX_PHASE_IDLE;
        else
            s->startup_tx_symbol_count = 8;
        /*endif*/
        break;
    case V32BIS_TX_PHASE_RENEG_PREAMBLE:
        s->startup_tx_symbol_count =
            v32bis_build_reneg_preamble(s->calling_party, s->startup_tx_symbols);
        break;
    case V32BIS_TX_PHASE_RENEG_R:
        /* 5.3.2: in the rate renegotiation procedure the scrambler is
           initialised to all zeros and the differential encoder to the final
           symbol of the transmitted preamble. */
        s->tx_trn_reg = 0;
        s->tx_trn_diff = v32bis_reneg_preamble_last_state(s->calling_party);
        s->reneg_r_start_symbol = s->tx_symbol_index;
        if (v32bis_tx_fill_word(s, s->reneg_tx_word) != 0)
            s->tx_phase = V32BIS_TX_PHASE_IDLE;
        /*endif*/
        break;
    case V32BIS_TX_PHASE_RENEG_CLEAR:
        /* Clause 8 Note 2: no common rate requires repeated E for at
           least 64T before clearing, rather than falling back to data. */
        s->reneg_clear_start_symbol = s->tx_symbol_index;
        v32bis_tx_fill_word(s, V32BIS_E_FIXED_BITS);
        break;
    case V32BIS_TX_PHASE_RENEG_E:
        if (v32bis_tx_enter_data(s) != 0)
            s->tx_phase = V32BIS_TX_PHASE_IDLE;
        else
            s->startup_tx_symbol_count = 8;
        /*endif*/
        break;
    case V32BIS_TX_PHASE_DATA:
    default:
        break;
    }
    /*endswitch*/
}
/*- End of function --------------------------------------------------------*/


/*! 8.2: "A modem shall be conditioned to detect an incoming preamble at any
    time while receiving data."  The preamble is one pure tone (the call
    modem's repeated state A, 1800 Hz) or a pair of them (the answer modem's
    alternating A and C, 600 and 3000 Hz), so it is found the same way the
    clause 6 tones are, on the samples, with nothing borrowed from the
    equalizer or the carrier loop -- which is what lets it run while the
    symbol path is busy carrying data.

    The discriminator against data is the coherent measurement's height above
    the received signal's own rms, not its absolute level: at this point in
    the call the far end has been transmitting data at the same power, so an
    absolute threshold cannot separate them at all. */
/*! One received sample through the far end's tone detectors: is the far
    end's AA (the call modem's, 1800 Hz) or AC (the answer modem's, 600 and
    3000 Hz) on the line?  Shared by the clause 8 preamble watch and the
    clause 7 retrain watch, which see the same tone and must not both run the
    detectors over one sample. */
static bool v32bis_far_tone_present(v32bis_state_t *s, int16_t x)
{
    float mag;
    float rms;
    /* The call modem watches for AC's two lines, the answer modem for AA's
       one. */
    const float threshold = V32BIS_RENEG_TONE_FRACTION*V32BIS_TONE_WINDOW
                          *sqrtf((s->calling_party  ?  2.0f  :  1.0f)/2.0f);

    s->reneg_watch_pow += ((float) x*x - s->reneg_watch_pow)*(1.0f/64.0f);
    if (s->calling_party)
    {
        /* The answer modem's preamble is alternating A and C, which is a
           suppressed carrier pair at 1800 -/+ 1200 Hz, so each line
           carries half the power and the two are added. */
        v32bis_tone_det_rx(&s->tone[0], (float) x, s->rx_sample_count);
        v32bis_tone_det_rx(&s->tone[1], (float) x, s->rx_sample_count);
        mag = s->tone[0].mag + s->tone[1].mag;
    }
    else
    {
        v32bis_tone_det_rx(&s->tone[2], (float) x, s->rx_sample_count);
        mag = s->tone[2].mag;
    }
    /*endif*/
    s->rx_sample_count++;
    rms = sqrtf(s->reneg_watch_pow);
    if (rms > 1.0f  &&  mag > threshold*rms)
    {
        s->retrain_tone_run++;
        return true;
    }
    /*endif*/
    s->retrain_tone_run = 0;
    return false;
}
/*- End of function --------------------------------------------------------*/

/*! ITU-T V.32bis 7.1/7.2: "detection of one of two tones at frequencies
    600 +/- 7 Hz and 3000 +/- 7 Hz for more than 128 symbol intervals" (call
    modem), "a tone of frequency 1800 +/- 7 Hz for more than 128 symbol
    intervals" (answer modem).  Those are the far end's AC and AA, which 8
    also uses for its 56T preamble head -- so a retrain is told apart from a
    renegotiation by duration alone.  A preamble's tone breaks at 56T, where
    its 180 degree reversal sits in the detection window and the coherent
    measurement dips, and then gives way to R4/R5, so it never approaches
    128T.  Returns the index of the sample on which a retrain is declared, or
    -1. */
static int v32bis_retrain_watch(v32bis_state_t *s, const int16_t amp[], int len)
{
    int i;

    for (i = 0;  i < len;  i++)
    {
        v32bis_far_tone_present(s, amp[i]);
        if (s->retrain_tone_run
                > (int) (V32BIS_RETRAIN_TONE_SYMBOLS*V32BIS_SAMPLES_PER_SYMBOL))
            return i + 1;
        /*endif*/
    }
    /*endfor*/
    return -1;
}
/*- End of function --------------------------------------------------------*/

static int v32bis_reneg_watch(v32bis_state_t *s, const int16_t amp[], int len)
{
    int i;

    for (i = 0;  i < len;  i++)
    {
        if (v32bis_far_tone_present(s, amp[i]))
        {
            /* The block is scanned here before v17_rx() sees any of it, so
               this is the frequency from before the data decoder was fed the
               first sample of the run. */
            if (s->reneg_preamble_run++ == 0)
                s->reneg_rate_snap = s->rx.carrier_phase_rate;
            /*endif*/
        }
        else
            s->reneg_preamble_run = 0;
        /*endif*/
        if (s->reneg_preamble_run
                < (int) (V32BIS_RENEG_DETECT_SYMBOLS*V32BIS_SAMPLES_PER_SYMBOL))
            continue;
        /*endif*/
        /* Preserve every sample before the clamp under the old data
           decoder, even when a callback straddles the detection instant. */
        if (i > 0)
            v17_rx(&s->rx, amp, i);
        /*endif*/
        /* By now the data decoder has run on V32BIS_RENEG_DETECT_SYMBOLS
           (plus the window) of preamble, and AA or AC is not data: its
           decisions are wrong, and track_carrier()'s integrator has walked
           the carrier frequency on them.  Nothing after this retrains it --
           8 promises no retraining, and the rate signal and E are decided
           against the one tap estimate below -- so the frequency comes back
           to what it was before the run began.  Measured on
           v32bis_duplex_test --reneg-only: with detection taken honestly ~46T
           into the preamble, the call modem responding to the answer modem's
           second renegotiation (call then answer, 12000 bit/s, both laws)
           came back white without this; restoring the equalizer instead
           changed nothing. */
        s->rx.carrier_phase_rate = s->reneg_rate_snap;
        s->rx_sample_count += len - i - 1;
        s->reneg_preamble_run = 0;
        s->reneg_far_preamble = true;
        if (v32bis_trace())
        {
            fprintf(stderr,
                    "[V32BIS %s] 8: far preamble at sample %d\n",
                    s->calling_party ? "call  " : "answer",
                    s->rx_sample_count);
        }
        /*endif*/
        /* 8.1/8.2 both clamp circuit 104 here and condition the receiver for
           the rate signal.  Attaching the symbol sink does both at once:
           v17rx.c hands the equalized symbol over and skips decode_baud()
           entirely, so no data reaches circuit 104 for as long as it is
           attached. */
        s->reneg_active = true;
        s->reneg_rx_reg = 0;
        s->reneg_rx_diff = v32bis_reneg_preamble_last_state(!s->calling_party);
        s->reneg_pre_done = false;
        s->reneg_pre_have = false;
        s->reneg_pre_tail = 0;
        s->reneg_word_pos = 0;
        s->reneg_r_seen = false;
        /* startup_rx_gain is the one tap channel estimate the 4 point slicer
           uses, and it was TRANSFERRED INTO THE EQUALIZER at the end of
           start-up -- so the equalizer now removes exactly that factor and
           the estimate itself has to start again at unity here, not at the
           value it had while it still described the channel.  Left stale,
           the slicer compares an already equalized symbol against a second
           copy of the channel and decides nothing. */
        s->startup_rx_gain.re = 1.0f;
        s->startup_rx_gain.im = 0.0f;
        s->startup_rx_stage = V32BIS_RX_RENEG_R;
        s->rx.symbol_sink = v32bis_startup_symbol_sink;
        s->rx.symbol_sink_user_data = s;
        s->rx.symbol_sink_uses_data_constellation = false;
        return i;
    }
    /*endfor*/
    return -1;
}
/*- End of function --------------------------------------------------------*/

/*! Start this side's half of the transmit sequence: preamble, then the rate
    signal.  8.1 reaches this when the DTE asks for a new rate; 8.2 reaches
    it on detecting the far end's R4. */
static void v32bis_reneg_start_tx(v32bis_state_t *s)
{
    if (s->reneg_local_rates == 0)
        s->reneg_local_rates = v32bis_reneg_rate_mask(s, s->bit_rate);
    /*endif*/
    if (s->reneg_local_rates == 0
        || v32bis_build_rate_signal(s->reneg_local_rates, &s->reneg_tx_word) != 0)
    {
        s->reneg_active = false;
        return;
    }
    /*endif*/
    s->tx.symbol_source = v32bis_startup_symbol_source;
    s->tx.symbol_source_user_data = s;
    v32bis_tx_enter_phase(s, V32BIS_TX_PHASE_RENEG_PREAMBLE);
}
/*- End of function --------------------------------------------------------*/

/*! Called by the symbol source when the current segment runs out.  Returns
    false only when the V.17 data encoder should take the baud stream over. */
static bool v32bis_tx_refill(v32bis_state_t *s)
{
    switch (s->tx_phase)
    {
    case V32BIS_TX_PHASE_TONE_A:
    case V32BIS_TX_PHASE_TONE_C:
    case V32BIS_TX_PHASE_TONE_AC:
    case V32BIS_TX_PHASE_TONE_CA:
    case V32BIS_TX_PHASE_TONE_AC2:
        v32bis_tx_tone_symbol(s);
        return true;
    case V32BIS_TX_PHASE_SILENT:
        if (s->tx_transition_at >= 0)
        {
            /* 6.2's 16 symbol intervals of silence before the conditioning
               signal. */
            if (s->tx_symbol_index < s->tx_transition_at)
                return true;
            /*endif*/
            /* This is 6.2's 16 symbol intervals before the FIRST conditioning
               signal, so the script has not advanced a step: the answer
               modem's tone phases occupy step 0, exactly as its opening
               conditioning signal does when the tones are skipped. */
            s->tx_transition_at = -1;
            /* 6.2: "cease transmitting for a period of 16 symbol intervals
               and then (see Note 3 below) transmit the receiver
               conditioning signal". */
            v32bis_tx_enter_phase(s, V32BIS_TX_PHASE_EC_TRAIN);
            return true;
        }
        /*endif*/
        if (!s->tx_released)
            return true;
        /*endif*/
        s->tx_step++;
        v32bis_tx_enter_phase(s,
                              s->calling_party ? V32BIS_TX_PHASE_S_NT
                                               : V32BIS_TX_PHASE_COND);
        return true;
    case V32BIS_TX_PHASE_S_NT:
        s->tx_step++;
        /* 6.1: "After this period has expired (see Note 3 below), the modem
           shall transmit the receiver conditioning signal". */
        v32bis_tx_enter_phase(s, V32BIS_TX_PHASE_EC_TRAIN);
        return true;
    case V32BIS_TX_PHASE_EC_TRAIN:
        if (v32bis_tx_fill_ec_training(s))
            return true;
        /*endif*/
        s->tx_far_end_quiet = false;
        v32bis_tx_enter_phase(s, V32BIS_TX_PHASE_COND);
        return true;
    case V32BIS_TX_PHASE_COND:
        s->tx_step++;
        v32bis_tx_enter_phase(s, V32BIS_TX_PHASE_R);
        return true;
    case V32BIS_TX_PHASE_R:
        if (!s->tx_released)
        {
            /* Keep repeating the same 16-bit rate sequence. */
            v32bis_tx_fill_word(s, s->tx_word);
            return true;
        }
        /*endif*/
        s->tx_step++;
        if (s->calling_party)
        {
            /* R2 is answered by R3, and 6.1 goes straight to E. */
            v32bis_tx_enter_phase(s, V32BIS_TX_PHASE_E);
        }
        else if (s->tx_step <= 2)
        {
            /* 6.2: "On detection of an incoming S sequence, the modem shall
               cease transmitting." */
            v32bis_tx_enter_phase(s, V32BIS_TX_PHASE_SILENT);
        }
        else
        {
            v32bis_tx_enter_phase(s, V32BIS_TX_PHASE_E);
        }
        /*endif*/
        return true;
    case V32BIS_TX_PHASE_E:
        s->tx_step++;
        v32bis_tx_enter_phase(s, V32BIS_TX_PHASE_DATA);
        return false;
    case V32BIS_TX_PHASE_RENEG_PREAMBLE:
        v32bis_tx_enter_phase(s, V32BIS_TX_PHASE_RENEG_R);
        return true;
    case V32BIS_TX_PHASE_RENEG_R:
        /* 8.1: "when R4 has been transmitted for a minimum of 64T, it shall
           complete the current 16-bit rate signal R4 and transmit sequence
           E".  8.2 is the same for R5, except that the responding modem has
           already seen R4 by the time it starts sending.  Either way the
           rate signal repeats until both conditions hold, so a far end that
           is slower than the minimum is simply waited for. */
        if (s->tx_symbol_index - s->reneg_r_start_symbol
                < V32BIS_RENEG_R_MIN_SYMBOLS
            ||
            !s->reneg_r_seen)
        {
            v32bis_tx_fill_word(s, s->reneg_tx_word);
            return true;
        }
        /*endif*/
        v32bis_tx_enter_phase(s,
            v32bis_select_common_rate(s->reneg_local_rates, s->reneg_remote_rates)
                ? V32BIS_TX_PHASE_RENEG_E : V32BIS_TX_PHASE_RENEG_CLEAR);
        return true;
    case V32BIS_TX_PHASE_RENEG_CLEAR:
        if (s->tx_symbol_index - s->reneg_clear_start_symbol < V32BIS_RENEG_R_MIN_SYMBOLS)
            v32bis_tx_fill_word(s, V32BIS_E_FIXED_BITS);
        else
        {
            s->reneg_cleared = true;
            s->startup_complete = false;
            s->bit_rate = 0;
            s->startup_tx_symbol_count = 0;
        }
        return true;
    case V32BIS_TX_PHASE_RENEG_E:
        v32bis_tx_enter_phase(s, V32BIS_TX_PHASE_DATA);
        return false;
    default:
        return false;
    }
    /*endswitch*/
}
/*- End of function --------------------------------------------------------*/

/*! Round a transmit symbol index up to the next one at which the alternating
    segment has run for an even number of symbol intervals, which is what 6.2
    requires of both the AC and the CA segments, and what makes the CA segment
    end on a state A. */
static int32_t v32bis_even_boundary(v32bis_state_t *s, int32_t at)
{
    if (at < s->tx_symbol_index + 1)
        at = s->tx_symbol_index + 1;
    /*endif*/
    if (((at - s->tx_phase_start_symbol) & 1) != 0)
        at++;
    /*endif*/
    return at;
}
/*- End of function --------------------------------------------------------*/

static int v32bis_symbols_between(int32_t a, int32_t b)
{
    return (int) lrintf((float) (b - a)/V32BIS_SAMPLES_PER_SYMBOL);
}
/*- End of function --------------------------------------------------------*/

/*! The clause 6 tone dialogue, run on the raw received samples.  The V.17
    receiver is not fed while this is going on: there is no conditioning
    signal to train on yet, and 6.1 and 6.2 both have the modem condition its
    receiver only once the tones are done. */
static void v32bis_tone_rx(v32bis_state_t *s, const int16_t amp[], int len)
{
    int i;
    int32_t now;
    int32_t at;

    for (i = 0;  i < len;  i++)
    {
        now = s->rx_sample_count++;
        if (s->calling_party)
        {
            bool rev0;
            bool rev1;

            rev0 = v32bis_tone_det_rx(&s->tone[0], (float) amp[i], now);
            rev1 = v32bis_tone_det_rx(&s->tone[1], (float) amp[i], now);
            s->tone_watch_pow += ((float) amp[i]*amp[i] - s->tone_watch_pow)*(1.0f/64.0f);
            if (s->tone_which < 0)
            {
                /* 6.1: "conditioned to detect ... one of two incoming tones at
                   frequencies 600 +/- 7 Hz and 3000 +/- 7 Hz", and only
                   "subsequently to detect a phase reversal in that tone" --
                   so the tone has to be established before the reversal
                   detector is armed, exactly as 6.2 spells out for the
                   answer modem's 1800 Hz tone.  Without the dwell this
                   modem's own state A leaks enough into the 600 and 3000 Hz
                   detectors, over a hybrid, to be taken for the far end's
                   tone and then for a reversal in it, before the far end's
                   tone has even arrived.  The answer modem sends at least
                   128 symbol intervals of alternating A and C before its
                   first reversal, so 64 of dwell cannot miss a real one. */
                if (s->tone[0].peak > 100.0f  ||  s->tone[1].peak > 100.0f)
                {
                    int which = (s->tone[0].peak >= s->tone[1].peak) ? 0 : 1;

                    /* And it has to be THAT tone, not something leaking into
                       a 20 sample window whose main lobe is 800 Hz wide.  A
                       call modem answering plain V.25 ANS with AA (A.2.1.3)
                       is listening here while the answer modem may still be
                       sending ANS, and 2100 Hz reaches the 3000 Hz detector
                       at about 1.4 times the received rms, where one line of
                       AC stands at W/2 = 10 times it -- and ANS's 180 degree
                       reversals then read as reversals in "the tone".  The
                       relative test alone passed it (slmodemd's ANS, then AC:
                       "reversal 1" during ANS and a second 40 samples later,
                       NT = 12 symbols).  Half the ideal is the bar. */
                    if (s->tone[which].mag > 0.4f*s->tone[which].peak
                        &&  s->tone[which].mag
                            > 0.25f*V32BIS_TONE_WINDOW*sqrtf(s->tone_watch_pow))
                        s->tone_present_run++;
                    else
                        s->tone_present_run = 0;
                    /*endif*/
                    if (s->tone_present_run
                            >= (int) (V32BIS_TONE_PRESENT_SYMBOLS*V32BIS_SAMPLES_PER_SYMBOL))
                    {
                        s->tone_which = which;
                    }
                    /*endif*/
                }
                else
                {
                    s->tone_present_run = 0;
                }
                /*endif*/
                continue;
            }
            /*endif*/
            if (!(s->tone_which == 0 ? rev0 : rev1))
                continue;
            /*endif*/
            at = s->tone[s->tone_which].reversal_sample;
            if (s->reversals_seen == 0)
            {
                /* 6.1: start the counter and, 64 symbol intervals after this
                   reversal reaches the line, change from AA to CC. */
                s->reversals_seen = 1;
                s->tone_counter_start = at;
                s->tx_transition_at = v32bis_schedule_reversal(at);
                if (v32bis_trace())
                {
                    fprintf(stderr,
                            "[V32BIS call  ] reversal 1 in the %d Hz tone at sample %d,"
                            " CC scheduled for tx symbol %d\n",
                            (s->tone_which == 0) ? 600 : 3000,
                            at,
                            s->tx_transition_at);
                }
                /*endif*/
            }
            else if (s->reversals_seen == 1)
            {
                /* 6.1: stop the counter and cease transmitting. */
                s->reversals_seen = 2;
                s->nt_symbols = v32bis_symbols_between(s->tone_counter_start, at);
                s->tone_phase_active = false;
                s->tx_transition_at = -1;
                v32bis_tx_enter_phase(s, V32BIS_TX_PHASE_SILENT);
                if (v32bis_trace())
                {
                    fprintf(stderr,
                            "[V32BIS call  ] reversal 2 at sample %d, NT = %d symbols\n",
                            at, s->nt_symbols);
                }
                /*endif*/
            }
            /*endif*/
            continue;
        }
        /*endif*/
        /* Answer mode. */
        if (v32bis_tone_det_rx(&s->tone[2], (float) amp[i], now)
            &&  s->reversals_seen == 1)
        {
            /* 6.2: stop the counter and, 64 symbol intervals after this
               reversal reaches the line, revert from CA to AC. */
            at = s->tone[2].reversal_sample;
            s->reversals_seen = 2;
            s->mt_symbols = v32bis_symbols_between(s->tone_counter_start, at);
            s->tx_transition_at = v32bis_even_boundary(s, v32bis_schedule_reversal(at));
            if (v32bis_trace())
            {
                fprintf(stderr,
                        "[V32BIS answer] reversal at sample %d, MT = %d symbols,"
                        " AC scheduled for tx symbol %d\n",
                        at, s->mt_symbols, s->tx_transition_at);
            }
            /*endif*/
            continue;
        }
        /*endif*/
        if (s->reversals_seen == 0)
        {
            /* 6.2: alternate A and C for an even number of symbol intervals
               of at least 128, and detect the incoming 1800 Hz tone for 64
               symbol periods, before starting the counter and switching to
               alternate C and A. */
            if (s->tone[2].mag > 0.4f*s->tone[2].peak  &&  s->tone[2].peak > 100.0f)
                s->tone_present_run++;
            else
                s->tone_present_run = 0;
            /*endif*/
            if (s->tone_present_run
                    >= (int) (V32BIS_TONE_PRESENT_SYMBOLS*V32BIS_SAMPLES_PER_SYMBOL)
                &&
                s->tx_symbol_index - s->tx_phase_start_symbol >= 128)
            {
                s->reversals_seen = 1;
                /* 6.2 measures the reversal delay at the line terminals, so
                   the counter starts when this modem's own AC to CA
                   transition reaches the line, not when the decision was
                   taken: the transmitter has already generated the current
                   block, so the two differ by up to a block. */
                s->tone_counter_start =
                    (int32_t) lrintf((s->tx_symbol_index + 1
                                      + V32BIS_TX_SHAPER_DELAY_SYMBOLS)
                                     *V32BIS_SAMPLES_PER_SYMBOL);
                s->tx_transition_at = -1;
                v32bis_tx_enter_phase(s, V32BIS_TX_PHASE_TONE_CA);
                if (v32bis_trace())
                {
                    fprintf(stderr,
                            "[V32BIS answer] 1800 Hz held for 64T; counter started at"
                            " sample %d, CA from tx symbol %d\n",
                            now, s->tx_symbol_index);
                }
                /*endif*/
            }
            /*endif*/
            continue;
        }
        /*endif*/
        if (s->reversals_seen == 2  &&  s->tx_transition_at < 0)
        {
            /* 6.2: "When an amplitude drop is detected in the incoming tone,
               the modem shall cease transmitting for a period of 16 symbol
               intervals and then transmit the receiver conditioning signal." */
            if (s->tone[2].mag < 0.25f*s->tone[2].peak)
                s->tone_drop_run++;
            else
                s->tone_drop_run = 0;
            /*endif*/
            if (s->tone_drop_run
                    >= (int) (V32BIS_TONE_DROP_SYMBOLS*V32BIS_SAMPLES_PER_SYMBOL))
            {
                s->tone_phase_active = false;
                v32bis_tx_enter_phase(s, V32BIS_TX_PHASE_SILENT);
                s->tx_transition_at = s->tx_symbol_index + V32BIS_TONE_DROP_SYMBOLS;
                if (v32bis_trace())
                {
                    fprintf(stderr,
                            "[V32BIS answer] amplitude drop at sample %d; conditioning"
                            " from tx symbol %d\n",
                            now, s->tx_transition_at);
                }
                /*endif*/
            }
            /*endif*/
        }
        /*endif*/
    }
    /*endfor*/
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v32bis_start_tones(v32bis_state_t *s)
{
    if (s == NULL  ||  v32bis_start_startup(s) != 0)
        return -1;
    /*endif*/
    s->tone_phase_active = true;
    s->tone_which = -1;
    s->tone_present_run = 0;
    s->tone_watch_pow = 0.0f;
    s->tone_drop_run = 0;
    s->reversals_seen = 0;
    s->tone_transition_symbol = -1;
    s->nt_symbols = 0;
    s->mt_symbols = 0;
    v32bis_tx_enter_phase(s,
                          s->calling_party ? V32BIS_TX_PHASE_TONE_A
                                           : V32BIS_TX_PHASE_TONE_AC);
    return 0;
}
/*- End of function --------------------------------------------------------*/

/*! ITU-T V.32bis clause 7.  Both roles re-enter clause 6 at its third
    paragraph -- 7.1 "repetively transmit carrier state A ... then proceed in
    accordance with 6.1 beginning with the third paragraph", 7.2 "transmit
    alternate carrier states A and C for an even number of symbol intervals
    not less than 128 ... then proceed in accordance with 6.2 beginning with
    the third paragraph" -- which is exactly where v32bis_start_tones() starts
    each of them, the V.25 answer sequence being the engine's.  The receiver
    is retrained from scratch: a retrain exists because reception became
    unsatisfactory, so nothing it had learned is kept.  The transmitter is NOT
    restarted (v17_tx_restart() would put a hole in the line signal); its
    symbol source simply takes the baud stream back from the data encoder,
    which also stops it drawing bits from circuit 103 -- circuit 106 OFF.

    The tone phases schedule transmit symbols off received sample instants
    (6.1/6.2's 64 +/- 2 symbol reversal delay, and NT and MT), so both clocks
    are re-based here on the samples that have actually crossed the line in
    each direction.  The transmit symbol clock advances exactly 3 symbols per
    10 samples from v32bis_restart(), so rebasing on a multiple of 10 samples
    keeps it exact. */
static void v32bis_begin_retrain(v32bis_state_t *s, int64_t rx_line_sample, bool local)
{
    int64_t base;
    int64_t tx_line;

    tx_line = s->tx_line_samples;
    base = (rx_line_sample < tx_line)  ?  rx_line_sample  :  tx_line;
    base -= base%10;
    if (v32bis_trace())
    {
        fprintf(stderr,
                "[V32BIS %s] 7: retrain %s at rx sample %lld (tx sample %lld)\n",
                s->calling_party ? "call  " : "answer",
                local ? "initiated" : "detected (far end tone > 128T)",
                (long long) rx_line_sample,
                (long long) tx_line);
    }
    /*endif*/
    span_log(&s->logging, SPAN_LOG_FLOW, "V.32bis retrain %s\n",
             local ? "initiated" : "requested by the far end");
    v17_rx_restart(&s->rx, s->bit_rate, false);
    s->retrain_tone_run = 0;
    s->retrain_count++;
    v32bis_start_tones(s);
    s->rx_sample_count = (int32_t) (rx_line_sample - base);
    s->tx_symbol_index = (int32_t) (((tx_line - base)*3)/10);
    s->tx_phase_start_symbol = s->tx_symbol_index;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v32bis_start_retrain(v32bis_state_t *s)
{
    if (s == NULL  ||  !s->reactive_startup)
        return -1;
    /*endif*/
    /* "A retrain may be initiated during data transmission": once start-up
       has handed over to data, including while a clause 8 exchange is in
       progress, since a renegotiation that cannot be completed is one of the
       ways reception turns out to be unsatisfactory. */
    if (!s->startup_complete  &&  !s->reneg_active)
        return -1;
    /*endif*/
    v32bis_begin_retrain(s, s->rx_line_samples, true);
    return 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v32bis_retrain_count(v32bis_state_t *s)
{
    return (s != NULL)  ?  s->retrain_count  :  0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v32bis_round_trip_symbols(v32bis_state_t *s, int *nt, int *mt)
{
    if (s == NULL)
        return -1;
    if (nt != NULL)
        *nt = s->nt_symbols;
    /*endif*/
    if (mt != NULL)
        *mt = s->mt_symbols;
    /*endif*/
    return 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v32bis_tone_transition_symbol(v32bis_state_t *s)
{
    return (s != NULL) ? s->tone_transition_symbol : -1;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v32bis_start_startup(v32bis_state_t *s)
{
    if (s == NULL)
        return -1;
    startup_rx_reset(s);
    s->reactive_startup = true;
    s->tx_step = 0;
    s->tx_released = false;
    s->rx_s_event_sent = false;
    s->rx_hold_symbols = 0;
    s->rx_repeat_word = 0;
    s->startup_selected_rate = 0;
    /* 8.2 watches during data even when a harness skips clause 6 tones. */
    v32bis_tone_det_init(&s->tone[0], 600.0f);
    v32bis_tone_det_init(&s->tone[1], 3000.0f);
    v32bis_tone_det_init(&s->tone[2], 1800.0f);
    s->tone_phase_active = false;
    s->tx_transition_at = -1;
    s->tx_symbol_index = 0;
    s->tx_phase_start_symbol = 0;
    s->rx_sample_count = 0;
    s->tx.symbol_source = v32bis_startup_symbol_source;
    s->tx.symbol_source_user_data = s;
    s->rx.symbol_sink = v32bis_startup_symbol_sink;
    s->rx.symbol_sink_user_data = s;
    s->rx.symbol_sink_uses_data_constellation = false;
    /* 6.1 leaves the call modem silent from the second phase reversal until it
       detects R1; 6.2 has the answer modem open with a conditioning signal. */
    v32bis_tx_enter_phase(s,
                          s->calling_party ? V32BIS_TX_PHASE_SILENT
                                           : V32BIS_TX_PHASE_COND);
    return 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v32bis_set_round_trip_symbols(v32bis_state_t *s, int nt, int mt)
{
    if (s == NULL  ||  nt < 0  ||  mt < 0  ||  nt > 512  ||  mt > 512)
        return -1;
    s->nt_symbols = nt;
    s->mt_symbols = mt;
    return 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v32bis_startup_tx_symbols_sent(v32bis_state_t *s)
{
    return (s != NULL) ? s->startup_tx_symbol_pos : 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v32bis_startup_rx_symbols_seen(v32bis_state_t *s)
{
    return (s != NULL) ? s->startup_rx_symbol_count : 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(bool) v32bis_startup_complete(v32bis_state_t *s)
{
    return s != NULL  &&  s->startup_complete;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v32bis_startup_remote_rates(v32bis_state_t *s)
{
    return (s != NULL) ? s->startup_remote_rates : 0;
}
/*- End of function --------------------------------------------------------*/

static void startup_rx_reset(v32bis_state_t *s)
{
    s->startup_rx_symbol_count = 0;
    s->startup_rx_stage = V32BIS_RX_SEARCH_S;
    s->startup_rx_acq_count = 0;
    s->startup_rx_sbar_run = 0;
    s->startup_rx_sbar_remaining = 0;
    s->startup_rx_trn_reg = 0;
    s->startup_rx_trn_pos = 0;
    s->startup_rx_trn_diff = V32BIS_STARTUP_A;
    s->startup_rx_word_pos = 0;
    s->startup_rx_first_r = 0;
    s->startup_rx_b1_pos = 0;
    s->startup_rx_b1_reg = 0;
    s->startup_rx_b1_diff = V32BIS_STARTUP_A;
    s->startup_rx_b1_convolution = 0;
    startup_b1_reset_vote(s);
    s->startup_remote_rates = 0;
    s->startup_selected_rate = 0;
    s->startup_complete = false;
    s->reneg_cleared = false;
    s->reneg_active = false;
    s->reneg_initiator = false;
    s->reneg_count = 0;
    s->reneg_local_rates = 0;
    s->reneg_remote_rates = 0;
    s->reneg_preamble_run = 0;
    s->reneg_watch_pow = 0.0f;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v32bis_set_supported_bit_rates(v32bis_state_t *s, int rates)
{
    uint16_t word;

    if (v32bis_build_rate_signal(rates, &word) != 0)
        return -1;
    s->permitted_rates_signal = word;
    return 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v32bis_current_bit_rate(v32bis_state_t *s)
{
    return s->bit_rate;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(logging_state_t *) v32bis_get_logging_state(v32bis_state_t *s)
{
    return &s->logging;
}
/*- End of function --------------------------------------------------------*/

static bool valid_bit_rate(int bit_rate)
{
    return bit_rate == 4800
        || bit_rate == 7200
        || bit_rate == 9600
        || bit_rate == 12000
        || bit_rate == 14400;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v32bis_rx_restart(v32bis_state_t *s, int bit_rate)
{
    if (!valid_bit_rate(bit_rate))
        return -1;
    if (v17_rx_restart(&s->rx, bit_rate, false) != 0)
        return -1;
    startup_rx_reset(s);
    s->rx.symbol_sink = v32bis_startup_symbol_sink;
    s->rx.symbol_sink_user_data = s;

    s->bit_rate = bit_rate;
    return 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v32bis_restart(v32bis_state_t *s, int bit_rate)
{
    if (!valid_bit_rate(bit_rate))
        return -1;
    span_log(&s->rx.logging, SPAN_LOG_FLOW, "Restarting V.32bis, %dbps\n", bit_rate);
    if (v17_tx_restart(&s->tx, bit_rate, false, false) != 0)
        return -1;
    if (v17_rx_restart(&s->rx, bit_rate, false) != 0)
        return -1;
    s->startup_tx_symbol_count = 0;
    s->startup_tx_symbol_pos = 0;
    s->tx_line_samples = 0;
    s->rx_line_samples = 0;
    s->retrain_tone_run = 0;
    s->retrain_count = 0;
    startup_rx_reset(s);
    s->tx.symbol_source = NULL;
    s->tx.symbol_source_user_data = NULL;
    s->rx.symbol_sink = v32bis_startup_symbol_sink;
    s->rx.symbol_sink_user_data = s;

    s->bit_rate = bit_rate;
    return 0;
}
/*- End of function --------------------------------------------------------*/

/*! Is the modem transmitting clause 6 Note 3's echo canceller training
    sequence right now?  Note 3 constrains that signal's spectrum, so a test
    has to be able to window the transmit audio to exactly it. */
/*! Set how many symbol intervals of clause 6 Note 3's echo canceller
    training sequence to transmit.  Note 3 makes the sequence optional, so 0
    is a conformant modem, and the far end has to cope either way. */
SPAN_DECLARE(int) v32bis_set_ec_training_symbols(v32bis_state_t *s, int symbols)
{
    if (symbols < 0  ||  symbols > V32BIS_EC_TRAIN_MAX_SYMBOLS)
        return -1;
    s->ec_train_symbols = symbols;
    return 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(bool) v32bis_tx_in_ec_training(v32bis_state_t *s)
{
    return (s->tx_phase == V32BIS_TX_PHASE_EC_TRAIN);
}
/*- End of function --------------------------------------------------------*/


/*! ITU-T V.32bis 8.1: "Rate renegotiation may be initiated at any time
    during data transmission." */
SPAN_DECLARE(int) v32bis_start_rate_renegotiation(v32bis_state_t *s, int bit_rate)
{
    if (s == NULL  ||  !s->startup_complete  ||  s->reneg_active)
        return -1;
    /*endif*/
    if (!s->reactive_startup)
        return -1;
    /*endif*/
    if (rate_to_mask(bit_rate) == 0
        || !(s->permitted_rates_signal & rate_to_mask(bit_rate)))
        return -1;
    /*endif*/
    s->reneg_local_rates = v32bis_reneg_rate_mask(s, bit_rate);
    if (s->reneg_local_rates == 0)
        return -1;
    /*endif*/
    s->reneg_active = true;
    s->reneg_initiator = true;
    s->reneg_r_seen = false;
    s->reneg_far_preamble = false;
    s->reneg_remote_rates = 0;
    s->reneg_preamble_run = 0;
    if (v32bis_trace())
    {
        fprintf(stderr,
                "[V32BIS %s] 8: initiating a renegotiation to %d bit/s\n",
                s->calling_party ? "call  " : "answer",
                bit_rate);
    }
    /*endif*/
    /* 8.1: turn circuit 106 OFF and transmit the preamble followed by R4.
       Circuit 104 is not clamped yet -- that waits on the far end's
       preamble, so data keeps arriving until the far end stops sending it. */
    v32bis_reneg_start_tx(s);
    return s->reneg_active ? 0 : -1;
}
/*- End of function --------------------------------------------------------*/

/*! How many rate renegotiations have completed on this connection. */
SPAN_DECLARE(int) v32bis_rate_renegotiation_count(v32bis_state_t *s)
{
    return s->reneg_count;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(v32bis_state_t *) v32bis_init(v32bis_state_t *s,
                                           int bit_rate,
                                           bool calling_party,
                                           span_get_bit_func_t get_bit,
                                           void *get_bit_user_data,
                                           span_put_bit_func_t put_bit,
                                           void *put_bit_user_data)
{
    if (!valid_bit_rate(bit_rate))
        return NULL;
    if (s == NULL)
    {
        if ((s = (v32bis_state_t *) span_alloc(sizeof(*s))) == NULL)
            return NULL;
    }
    memset(s, 0, sizeof(*s));
    span_log_init(&s->logging, SPAN_LOG_NONE, NULL);
    span_log_set_protocol(&s->logging, "V.32bis");
    s->bit_rate = bit_rate;
    s->calling_party = calling_party;

    /* V.32bis never uses TEP */
    v17_tx_init(&s->tx, bit_rate, false, get_bit, get_bit_user_data);
    v17_rx_init(&s->rx, bit_rate, put_bit, put_bit_user_data);
    /* 256 samples is 32 ms, which covers a near end hybrid's echo path with
       room to spare; the far end's echo is the far end's problem. */
    s->ec = modem_echo_can_segment_init(256);
    s->echo_can_enabled = v32bis_echo_can();
    s->ec_train_symbols = v32bis_ec_train_symbols();
    {
        const char *e = getenv("V32BIS_ECHO_MU");

        (void) e;
        modem_echo_can_step_size(s->ec, v32bis_echo_mu_slow());
    }

    {
        const char *d;

        char path[512];

        if (calling_party  &&  (d = getenv("V32BIS_SYM_DUMP_TX")) != NULL)
        {
            snprintf(path, sizeof(path), "%s-%d", d, bit_rate);
            s->tx.v32bis_sym_dump = fopen(path, "w");
        }
        /*endif*/
        if (!calling_party  &&  (d = getenv("V32BIS_SYM_DUMP_RX")) != NULL)
        {
            snprintf(path, sizeof(path), "%s-%d", d, bit_rate);
            s->rx.v32bis_sym_dump = fopen(path, "w");
        }
        /*endif*/
        if (!calling_party  &&  (d = getenv("V32BIS_T2_DUMP")) != NULL)
        {
            snprintf(path, sizeof(path), "%s-%d", d, bit_rate);
            s->rx.v32bis_t2_dump = fopen(path, "w");
        }
        /*endif*/
    }
    /* Initialise things which are not quite like V.17 */
    if (s->calling_party)
    {
        s->tx.scrambler_tap = 17;
        s->rx.scrambler_tap = 4;
    }
    else
    {
        s->tx.scrambler_tap = 4;
        s->rx.scrambler_tap = 17;
    }
    v32bis_set_supported_bit_rates(s,
                                   V32BIS_RATE_14400
                                 | V32BIS_RATE_12000
                                 | V32BIS_RATE_9600
                                 | V32BIS_RATE_7200
                                 | V32BIS_RATE_4800);
    v32bis_restart(s, bit_rate);
    return s;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v32bis_release(v32bis_state_t *s)
{
    if (s->tx.v32bis_sym_dump != NULL)
    {
        fclose(s->tx.v32bis_sym_dump);
        s->tx.v32bis_sym_dump = NULL;
    }
    /*endif*/
    if (s->rx.v32bis_sym_dump != NULL)
    {
        fclose(s->rx.v32bis_sym_dump);
        s->rx.v32bis_sym_dump = NULL;
    }
    /*endif*/
    if (s->rx.v32bis_t2_dump != NULL)
    {
        fclose(s->rx.v32bis_t2_dump);
        s->rx.v32bis_t2_dump = NULL;
    }
    /*endif*/
    if (s->rx.v32bis_eye_log  &&  s->rx.v32bis_eye_count > 0)
    {
        fprintf(stderr,
                "V32BIS data eye: rate=%d symbols=%d rms=%.4f\n",
                s->bit_rate,
                s->rx.v32bis_eye_count,
                sqrt(s->rx.v32bis_eye_sum/s->rx.v32bis_eye_count));
    }
    /*endif*/
    if (s->ec != NULL)
    {
        modem_echo_can_segment_free(s->ec);
        s->ec = NULL;
    }
    return 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v32bis_free(v32bis_state_t *s)
{
    v32bis_release(s);
    span_free(s);
    return 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v32bis_set_qam_report_handler(v32bis_state_t *s, qam_report_handler_t handler, void *user_data)
{
    s->rx.qam_report = handler;
    s->rx.qam_user_data = user_data;
}
/*- End of function --------------------------------------------------------*/
/*- End of file ------------------------------------------------------------*/
