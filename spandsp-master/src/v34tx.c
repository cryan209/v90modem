/*
 * SpanDSP - a series of DSP components for telephony
 *
 * v34tx.c - ITU V.34 modem, transmit part
 *
 * Written by Steve Underwood <steveu@coppice.org>
 *
 * Copyright (C) 2009 Steve Underwood
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

/* THIS IS A WORK IN PROGRESS - NOT YET FUNCTIONAL! */

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
#include <stddef.h>

#include "spandsp/telephony.h"
#include "spandsp/alloc.h"
#include "spandsp/fast_convert.h"
#include "spandsp/logging.h"
#include "spandsp/bit_operations.h"
#include "spandsp/bitstream.h"
#include "spandsp/complex.h"
#include "spandsp/vector_float.h"
#include "spandsp/complex_vector_float.h"
#include "spandsp/vector_int.h"
#include "spandsp/complex_vector_int.h"
#include "spandsp/modem_echo.h"
#include "spandsp/async.h"
#include "spandsp/power_meter.h"
#include "spandsp/arctan2.h"
#include "spandsp/dds.h"
#include "spandsp/crc.h"
#include "spandsp/complex_filters.h"
#include "spandsp/g711.h"

#include "spandsp/v29rx.h"
#include "spandsp/v34.h"

#include "spandsp/private/bitstream.h"
#include "spandsp/private/logging.h"
#include "spandsp/private/power_meter.h"
#include "spandsp/private/modem_echo.h"
#include "spandsp/private/v34.h"

#include "v22bis_tx_rrc.h"

#include "v34_tx_2400_rrc.h"
#include "v34_tx_2743_rrc.h"
#include "v34_tx_2800_rrc.h"
#include "v34_tx_3000_rrc.h"
#include "v34_tx_3200_rrc.h"
#include "v34_tx_3429_rrc.h"

#include "v34_local.h"
#include "v34_tables.h"
#include "v34_superconstellation_map.h"
#include "v34_convolutional_coders.h"
#include "v34_probe_signals.h"
#include "v34_shell_map.h"
#include "v34_tx_pre_emphasis_filters.h"

#if defined(SPANDSP_USE_FIXED_POINT)
#define complex_sig_set(re,im) complex_seti16(re,im)
#define complex_sig_t complexi16_t
#else
#define complex_sig_set(re,im) complex_setf(re,im)
#define complex_sig_t complexf_t
#endif

#define FP_Q9_7_TO_F(x)                 ((float) x/128.0f)

#define EQUALIZER_DELTA                 0.21f
#define EQUALIZER_SLOW_ADAPT_RATIO      0.1f

#define V34_TRAINING_SEG_1              0
#define V34_TRAINING_SEG_4              0
#define V34_TRAINING_END                0
#define V34_TRAINING_SHUTDOWN_END       0

#define INFO_FILL_AND_SYNC_BITS         0x4EF

#if defined(SPANDSP_USE_FIXED_POINT)
#define TRAINING_SCALE(x)               ((int16_t) (32767.0f*x + ((x >= 0.0)  ?  0.5  :  -0.5)))
#else
#define TRAINING_SCALE(x)               (x)
#endif

/* v34_tx_power() turns a dBm0 request into tx.gain on the assumption that the
   symbols reaching the RRC filter have this modulus RMS. Anything handed to
   either modulator has to be scaled to it, or the wire level misses the
   requested dBm0 by exactly the ratio. */
#define V34_NOMINAL_SYMBOL_RMS          4.4843f

/* Constant-modulus amplitude of the training and tone signals - the Phase 2
   tones and INFO carriers on the CC modulator, and S/!S, PP and TRN on the
   primary channel.

   This was 10.0f, i.e. +6.97 dB over V34_NOMINAL_SYMBOL_RMS. The CC path
   carried a separate empirical trim of 0.438f which cancelled almost exactly
   that error (10*0.438 = 4.38, within 0.2 dB of 4.4843) - the wire calibration
   of 2026-07-19 was, without knowing it, measuring this mismatch. The Phase 3/4
   primary-channel modulator had no equivalent trim, which is why it transmitted
   6.7 dB hot. Correcting the amplitude at source fixes both paths, so the CC
   trim is gone; adding one back now would double-correct. */
#define TRAINING_AMP                    V34_NOMINAL_SYMBOL_RMS

/* Empirical TX level trim for the L1/L2 probe, calibrated against the wire
   (live G.711 taps, 2026-07-19): with v34_tx_power(-10 dBm0) the probe measured
   L2 at -3.6 dBm0, leaving L1 hard clipped at 100% of full scale (3.8% of
   samples) - an analog V.90 peer judges the downstream PCM path from these
   exact signals and falls back to V.34 at minimum baud when they arrive hot and
   clipped. The probe scales its own tables through line_probe_scaling rather
   than going through TRAINING_AMP, so it keeps its own trim. */
#define V34_LINE_PROBE_LEVEL_TRIM       0.478f

enum
{
    TRAINING_TX_STAGE_NORMAL_OPERATION_V34 = 0,
    TRAINING_TX_STAGE_NORMAL_OPERATION_CC = 1,
    TRAINING_TX_STAGE_PARKED
};

static const char *v34_tx_stage_to_str(int stage)
{
    switch (stage)
    {
    case V34_TX_STAGE_INITIAL_PREAMBLE: return "INITIAL_PREAMBLE";
    case V34_TX_STAGE_INFO0: return "INFO0";
    case V34_TX_STAGE_INITIAL_A: return "INITIAL_A";
    case V34_TX_STAGE_FIRST_A: return "FIRST_A";
    case V34_TX_STAGE_FIRST_NOT_A: return "FIRST_NOT_A";
    case V34_TX_STAGE_FIRST_NOT_A_REVERSAL_SEEN: return "FIRST_NOT_A_REVERSAL_SEEN";
    case V34_TX_STAGE_SECOND_A: return "SECOND_A";
    case V34_TX_STAGE_L1: return "L1";
    case V34_TX_STAGE_L2: return "L2";
    case V34_TX_STAGE_POST_INFO0_RESUME_A: return "POST_INFO0_RESUME_A";
    case V34_TX_STAGE_POST_L2_WAIT_TONE_B: return "POST_L2_WAIT_TONE_B";
    case V34_TX_STAGE_POST_L2_A: return "POST_L2_A";
    case V34_TX_STAGE_POST_L2_NOT_A: return "POST_L2_NOT_A";
    case V34_TX_STAGE_A_SILENCE: return "A_SILENCE";
    case V34_TX_STAGE_PRE_INFO1_A: return "PRE_INFO1_A";
    case V34_TX_STAGE_INFOMARKSA: return "INFOMARKSA";
    case V34_TX_STAGE_V90_WAIT_TONE_A: return "V90_WAIT_TONE_A";
    case V34_TX_STAGE_V90_WAIT_INFO1A: return "V90_WAIT_INFO1A";
    case V34_TX_STAGE_V90_WAIT_RX_L2: return "V90_WAIT_RX_L2";
    case V34_TX_STAGE_V90_WAIT_TONE_A_REV: return "V90_WAIT_TONE_A_REV";
    case V34_TX_STAGE_V90_B_REV_DELAY: return "V90_B_REV_DELAY";
    case V34_TX_STAGE_V90_B_REV_10MS: return "V90_B_REV_10MS";
    case V34_TX_STAGE_V90_PHASE2_B: return "V90_PHASE2_B";
    case V34_TX_STAGE_V90_PHASE2_B_INFO0_SEEN: return "V90_PHASE2_B_INFO0_SEEN";
    case V34_TX_STAGE_V90_RETRAIN_SILENCE: return "V90_RETRAIN_SILENCE";
    case V34_TX_STAGE_V34_FALLBACK_WAIT_J: return "V34_FALLBACK_WAIT_J";
    case V34_TX_STAGE_INFO1: return "INFO1";
    case V34_TX_STAGE_FIRST_B: return "FIRST_B";
    case V34_TX_STAGE_FIRST_B_INFO_SEEN: return "FIRST_B_INFO_SEEN";
    case V34_TX_STAGE_FIRST_NOT_B_WAIT: return "FIRST_NOT_B_WAIT";
    case V34_TX_STAGE_FIRST_NOT_B: return "FIRST_NOT_B";
    case V34_TX_STAGE_FIRST_B_SILENCE: return "FIRST_B_SILENCE";
    case V34_TX_STAGE_FIRST_B_POST_REVERSAL_SILENCE: return "FIRST_B_POST_REVERSAL_SILENCE";
    case V34_TX_STAGE_SECOND_B: return "SECOND_B";
    case V34_TX_STAGE_SECOND_B_WAIT: return "SECOND_B_WAIT";
    case V34_TX_STAGE_SECOND_NOT_B: return "SECOND_NOT_B";
    case V34_TX_STAGE_INFO0_RETRY: return "INFO0_RETRY";
    case V34_TX_STAGE_FIRST_S: return "FIRST_S";
    case V34_TX_STAGE_FIRST_NOT_S: return "FIRST_NOT_S";
    case V34_TX_STAGE_MD: return "MD";
    case V34_TX_STAGE_SECOND_S: return "SECOND_S";
    case V34_TX_STAGE_SECOND_NOT_S: return "SECOND_NOT_S";
    case V34_TX_STAGE_TRN: return "TRN";
    case V34_TX_STAGE_J: return "J";
    case V34_TX_STAGE_J_DASHED: return "J_DASHED";
    case V34_TX_STAGE_PHASE4_WAIT: return "PHASE4_WAIT";
    case V34_TX_STAGE_PHASE4_S: return "PHASE4_S";
    case V34_TX_STAGE_PHASE4_NOT_S: return "PHASE4_NOT_S";
    case V34_TX_STAGE_PHASE4_TRN: return "PHASE4_TRN";
    case V34_TX_STAGE_MP: return "MP";
    case V34_TX_STAGE_HDX_INITIAL_A: return "HDX_INITIAL_A";
    case V34_TX_STAGE_HDX_FIRST_A: return "HDX_FIRST_A";
    case V34_TX_STAGE_HDX_FIRST_NOT_A: return "HDX_FIRST_NOT_A";
    case V34_TX_STAGE_HDX_FIRST_A_SILENCE: return "HDX_FIRST_A_SILENCE";
    case V34_TX_STAGE_HDX_SECOND_A: return "HDX_SECOND_A";
    case V34_TX_STAGE_HDX_SECOND_A_WAIT: return "HDX_SECOND_A_WAIT";
    case V34_TX_STAGE_HDX_FIRST_B: return "HDX_FIRST_B";
    case V34_TX_STAGE_HDX_FIRST_B_INFO_SEEN: return "HDX_FIRST_B_INFO_SEEN";
    case V34_TX_STAGE_HDX_FIRST_NOT_B_WAIT: return "HDX_FIRST_NOT_B_WAIT";
    case V34_TX_STAGE_HDX_FIRST_NOT_B: return "HDX_FIRST_NOT_B";
    case V34_TX_STAGE_HDX_POST_L2_B: return "HDX_POST_L2_B";
    case V34_TX_STAGE_HDX_POST_L2_SILENCE: return "HDX_POST_L2_SILENCE";
    case V34_TX_STAGE_HDX_SH: return "HDX_SH";
    case V34_TX_STAGE_HDX_FIRST_ALT: return "HDX_FIRST_ALT";
    case V34_TX_STAGE_HDX_PPH: return "HDX_PPH";
    case V34_TX_STAGE_HDX_SECOND_ALT: return "HDX_SECOND_ALT";
    case V34_TX_STAGE_HDX_MPH: return "HDX_MPH";
    case V34_TX_STAGE_HDX_E: return "HDX_E";
    default: return "UNKNOWN";
    }
}

static const char *v34_modulation_to_str(int mod)
{
    switch (mod)
    {
    case V34_MODULATION_V34: return "V34";
    case V34_MODULATION_CC: return "CC";
    case V34_MODULATION_TONES: return "TONES";
    case V34_MODULATION_L1_L2: return "L1_L2";
    case V34_MODULATION_PCM_L1_L2: return "PCM_L1_L2";
    case V34_MODULATION_SILENCE: return "SILENCE";
    default: return "UNKNOWN";
    }
}

/* V.90 9.2.1.2.4 defines the third-reversal recovery deadline as 900 ms
   plus RTD from the second received Tone A reversal.  Keep that absolute
   deadline as a recovery guard; normal progress is driven by the durable
   reversal transaction below.  The old default was an empirically shortened
   333 ms stage timer, so every call that lost the mailbox event appeared to
   make progress while actually waiting on a local clock. */
static bool v90_tone_a_reversal_recovery_due(const v34_state_t *s)
{
    span_sample_timer_t deadline_samples;
    span_sample_timer_t elapsed_samples;
    int rtd_samples;
    int rtd_bauds;
    int fallback_bauds;

    rtd_samples = (s->rx.round_trip_delay_estimate > 0)
                  ? s->rx.round_trip_delay_estimate
                  : 0;
    deadline_samples = (SAMPLE_RATE*900 + 500)/1000 + rtd_samples;
    if (s->rx.tone_ab_hop_time > 0
        &&  s->tx.sample_time >= s->rx.tone_ab_hop_time)
    {
        elapsed_samples = s->tx.sample_time - s->rx.tone_ab_hop_time;
        return elapsed_samples >= deadline_samples;
    }
    /*endif*/

    /* If the second reversal edge itself was missed, retain a bounded
       standards-sized recovery measured from this stage's entry. */
    rtd_bauds = (rtd_samples*600 + SAMPLE_RATE/2)/SAMPLE_RATE;
    fallback_bauds = (600*900 + 500)/1000 + rtd_bauds;
    return s->tx.tone_duration >= fallback_bauds;
}
/*- End of function --------------------------------------------------------*/

/* Phase 2 receive progress is transactional, not a "latest event" value.

   received_event is still useful for diagnostics and for V.34 compatibility,
   but it is a lossy mailbox: an INFO repeat, tone indication, or TX-side clear
   can replace an unconsumed reversal/L2 event.  RX therefore owns monotonic
   counters and TX consumes one completed transaction at a time.  Consuming
   only one count is important when a large media block contains two adjacent
   milestones; the next TX stage must still observe the second one. */
static bool v90_phase2_reversal_pending(const v34_state_t *s)
{
    return s->rx.phase2_reversal_count > s->tx.v90_phase2_reversals_consumed;
}
/*- End of function --------------------------------------------------------*/

static void v90_phase2_consume_reversal(v34_state_t *s)
{
    if (v90_phase2_reversal_pending(s))
        s->tx.v90_phase2_reversals_consumed++;
    /*endif*/
}
/*- End of function --------------------------------------------------------*/

static bool v90_phase2_l2_pending(const v34_state_t *s)
{
    return s->rx.phase2_l2_count > s->tx.v90_phase2_l2_consumed;
}
/*- End of function --------------------------------------------------------*/

static void v90_phase2_consume_l2(v34_state_t *s)
{
    if (v90_phase2_l2_pending(s))
        s->tx.v90_phase2_l2_consumed++;
    /*endif*/
}
/*- End of function --------------------------------------------------------*/

static void v90_phase2_reset_transactions(v34_state_t *s)
{
    s->rx.phase2_reversal_count = 0;
    s->rx.phase2_l2_count = 0;
    s->tx.v90_phase2_reversals_consumed = 0;
    s->tx.v90_phase2_l2_consumed = 0;
}
/*- End of function --------------------------------------------------------*/

/* Split a sample count into whole milliseconds plus a tenths digit, so stage
   timings can be logged at sub-millisecond resolution without pulling floating
   point into the log path. Done as two divisions rather than samples*10/8 so
   the intermediate cannot overflow on a long-running state. */
static void samples_to_ms_tenths(span_sample_timer_t samples, int *ms, int *tenths)
{
    if (samples < 0)
        samples = 0;
    /*endif*/
    *ms = samples/(SAMPLE_RATE/1000);
    *tenths = ((samples%(SAMPLE_RATE/1000))*10)/(SAMPLE_RATE/1000);
}
/*- End of function --------------------------------------------------------*/

/* Log every TX stage transition with the time actually spent in the stage being
   left, plus the elapsed time since Phase 2 began. Phase 2 is a chain of
   "exit on event, else exit on timeout" waits, so the interesting question on a
   slow handshake is which stages ran to their full timeout. Reading that off the
   raw stage log means hand-converting baud counts, so report it directly.

   sample_offset lets a caller inside a modulator report the position within the
   block it is currently filling, since s->tx.sample_time is only advanced once
   the whole block has been generated. */
static void v34_tx_log_state_change_at(v34_state_t *s, int sample_offset)
{
    span_sample_timer_t now;
    span_sample_timer_t dwell;
    const char *role;
    int dwell_ms;
    int dwell_tenths;
    int total_ms;
    int total_tenths;

    if (s->tx.last_logged_stage == s->tx.stage
        &&  s->tx.last_logged_modulator == s->tx.current_modulator)
    {
        return;
    }
    /*endif*/
    now = s->tx.sample_time + sample_offset;
    /* Name the side in the message rather than relying on the log tag: this
       logging module deliberately prints tag/protocol as pointers rather than
       dereferencing them, so a tag would not be readable. Coupled tests run
       both modems into one log stream and need them told apart. */
    role = s->tx.calling_party ? "analogue" : "digital";

    /* Phase 2 starts at the first INFO0 (or the preamble ahead of it) and ends
       when we hand off to the Phase 3 S/!S sequence. A retrain drops back to
       INFO0, which re-arms the window for a fresh measurement. */
    if (s->tx.phase2_entry_sample_time < 0
        &&
        (s->tx.stage == V34_TX_STAGE_INITIAL_PREAMBLE
         ||
         s->tx.stage == V34_TX_STAGE_INFO0))
    {
        s->tx.phase2_entry_sample_time = now;
    }
    /*endif*/

    /* Time the (stage, modulator) pair rather than the stage alone. The L1/L2
       probe swaps the modulator without touching the stage, so timing stage
       changes only would silently fold the whole ~560 ms probe into whichever
       stage happened to launch it. The modulator is also what is actually
       audible, which is the thing being diagnosed. */
    if (s->tx.stage_entry_sample_time >= 0)
    {
        dwell = now - s->tx.stage_entry_sample_time;
        samples_to_ms_tenths(dwell, &dwell_ms, &dwell_tenths);
        if (s->tx.phase2_entry_sample_time >= 0)
        {
            samples_to_ms_tenths(now - s->tx.phase2_entry_sample_time,
                                 &total_ms,
                                 &total_tenths);
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Tx - timing [%s]: %s/%s took %d.%01d ms (phase 2 t=%d.%01d ms)\n",
                     role,
                     v34_tx_stage_to_str(s->tx.last_logged_stage),
                     v34_modulation_to_str(s->tx.last_logged_modulator),
                     dwell_ms, dwell_tenths,
                     total_ms, total_tenths);
        }
        else
        {
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Tx - timing [%s]: %s/%s took %d.%01d ms\n",
                     role,
                     v34_tx_stage_to_str(s->tx.last_logged_stage),
                     v34_modulation_to_str(s->tx.last_logged_modulator),
                     dwell_ms, dwell_tenths);
        }
        /*endif*/
    }
    /*endif*/

    span_log(&s->logging, SPAN_LOG_FLOW,
             "Tx - [%s] stage=%s (%d) mod=%s (%d)\n",
             role,
             v34_tx_stage_to_str(s->tx.stage), s->tx.stage,
             v34_modulation_to_str(s->tx.current_modulator), s->tx.current_modulator);

    if (s->tx.stage == V34_TX_STAGE_FIRST_S
        &&
        s->tx.phase2_entry_sample_time >= 0)
    {
        samples_to_ms_tenths(now - s->tx.phase2_entry_sample_time,
                             &total_ms,
                             &total_tenths);
        span_log(&s->logging, SPAN_LOG_FLOW,
                 "Tx - timing [%s]: Phase 2 complete in %d.%01d ms, handing off to Phase 3\n",
                 role, total_ms, total_tenths);
        s->tx.phase2_entry_sample_time = -1;
    }
    /*endif*/

    s->tx.stage_entry_sample_time = now;
    s->tx.last_logged_stage = s->tx.stage;
    s->tx.last_logged_modulator = s->tx.current_modulator;
}
/*- End of function --------------------------------------------------------*/

static void v34_tx_log_state_change(v34_state_t *s)
{
    v34_tx_log_state_change_at(s, 0);
}

#if defined(SPANDSP_USE_FIXED_POINT)
typedef int16_t tx_shaper_t[V34_TX_FILTER_STEPS];
#else
typedef float tx_shaper_t[V34_TX_FILTER_STEPS];
#endif

static const tx_shaper_t *v34_tx_shapers[] =
{
    tx_pulseshaper_2400,
    tx_pulseshaper_2743,
    tx_pulseshaper_2800,
    tx_pulseshaper_3000,
    tx_pulseshaper_3200,
    tx_pulseshaper_3429
};

static const complex_sig_t zero = {TRAINING_SCALE(0.0f), TRAINING_SCALE(0.0f)};

static const complex_sig_t training_constellation_4[4] =
{
    {TRAINING_SCALE(-0.7071068f*TRAINING_AMP), TRAINING_SCALE(-0.7071068f*TRAINING_AMP)},   /* 225 degrees */
    {TRAINING_SCALE(-0.7071068f*TRAINING_AMP), TRAINING_SCALE( 0.7071068f*TRAINING_AMP)},   /* 135 degrees */
    {TRAINING_SCALE( 0.7071068f*TRAINING_AMP), TRAINING_SCALE( 0.7071068f*TRAINING_AMP)},   /*  45 degrees */
    {TRAINING_SCALE( 0.7071068f*TRAINING_AMP), TRAINING_SCALE(-0.7071068f*TRAINING_AMP)}    /* 315 degrees */
};

static const complex_sig_t training_constellation_16[16] =
{
    {TRAINING_SCALE(-1.0f*TRAINING_AMP), TRAINING_SCALE(-1.0f*TRAINING_AMP)},
    {TRAINING_SCALE(-1.0f*TRAINING_AMP), TRAINING_SCALE( 1.0f*TRAINING_AMP)},
    {TRAINING_SCALE( 1.0f*TRAINING_AMP), TRAINING_SCALE( 1.0f*TRAINING_AMP)},
    {TRAINING_SCALE( 1.0f*TRAINING_AMP), TRAINING_SCALE(-1.0f*TRAINING_AMP)},

    {TRAINING_SCALE( 3.0f*TRAINING_AMP), TRAINING_SCALE(-1.0f*TRAINING_AMP)},
    {TRAINING_SCALE(-1.0f*TRAINING_AMP), TRAINING_SCALE(-3.0f*TRAINING_AMP)},
    {TRAINING_SCALE(-3.0f*TRAINING_AMP), TRAINING_SCALE( 1.0f*TRAINING_AMP)},
    {TRAINING_SCALE( 1.0f*TRAINING_AMP), TRAINING_SCALE( 3.0f*TRAINING_AMP)},

    {TRAINING_SCALE(-1.0f*TRAINING_AMP), TRAINING_SCALE( 3.0f*TRAINING_AMP)},
    {TRAINING_SCALE( 3.0f*TRAINING_AMP), TRAINING_SCALE( 1.0f*TRAINING_AMP)},
    {TRAINING_SCALE( 1.0f*TRAINING_AMP), TRAINING_SCALE(-3.0f*TRAINING_AMP)},
    {TRAINING_SCALE(-3.0f*TRAINING_AMP), TRAINING_SCALE(-1.0f*TRAINING_AMP)},

    {TRAINING_SCALE( 3.0f*TRAINING_AMP), TRAINING_SCALE( 3.0f*TRAINING_AMP)},
    {TRAINING_SCALE( 3.0f*TRAINING_AMP), TRAINING_SCALE(-3.0f*TRAINING_AMP)},
    {TRAINING_SCALE(-3.0f*TRAINING_AMP), TRAINING_SCALE(-3.0f*TRAINING_AMP)},
    {TRAINING_SCALE(-3.0f*TRAINING_AMP), TRAINING_SCALE( 3.0f*TRAINING_AMP)}
};

/*
DUPLEX OPERATION
----------------

Duplex caller
-------------
V.8 sequence   | INFO0c |   B   |!B|                   |   B   |!B|L1|   L2   | INFO1c |                                        |S|!S|  MD  |S|!S| PP | TRN |  J  |J'|  TRN |MP |MP |MP'|MP'|E| B1 | Data
---------------|XXXXXXXX|XXXXXXX|XX|-------------------|XXXXXXX|XX|XX|XXXXXXXX|XXXXXXXX|----------------------------------------|X|XX|XXXXXX|X|XX|XXXX|XXXXX|XXXXX|XX|XXXXXX|XXX|XXX|XXX|XXX|X|XXXX|XXXXXXXXXXXXX

Duplex answerer
---------------
V.8 sequence    | INFO0a  | A |  !A  | A |L1|   L2   | A |!A|               |     A    | INFO1a |  | S |!S| MD | S |!S| PP | TRN | J |                       | S |!S|  TRN |MP |MP |MP'|MP'|E| B1 |   Data
----------------|XXXXXXXXX|XXX|XXXXXX|XXX|XX|XXXXXXXX|XXX|XX|---------------|XXXXXXXXXX|XXXXXXXX|--|XXX|XX|XXXX|XXX|XX|XXXX|XXXXX|XXX|-----------------------|XXX|XX|XXXXXX|XXX|XXX|XXX|XXX|X|XXXX|XXXXXXXXXXXXXXX

J       Repetitions of 0x8990 for a 4 point constellation, or 0x89B0 for a 16 point constellation.
J'      A single transmission of 0x899F.
MD      Manufacturer specific training sequence
PP      Preliminary training sequence. 8 symbols, repeated 4 times.
S       90 degree alternations
!S      180 degree shift from S
TRN     Training sequence 4 or 16 symbols scrambled by the scrambler
MP      Modulation parameter sequence
MP'     Modulation parameter sequence with the acknowledgement bit set
A       2400Hz
!A      Phase reversed 2400Hz
B       1200Hz
!B      Phase reversed 1200Hz
ALT
E
*/

/*
HALF-DUPLEX OPERATION
---------------------

Half duplex caller, when caller is source
-----------------------------------------
V.8 sequence  silence  INFO0c  B  !B  L1  L2  B  silence  S  !S  PP  TRN  silence

Half duplex answerer, when caller is source
-------------------------------------------
V.8 sequence silence INFO0a  A  !A  silence  A  INFOh  silence

Half duplex caller, when answerer is source
-------------------------------------------
V.8 sequence  silence  INFO0c  B  !B  silence  B INFOh  silence

Half duplex answerer, when answerer is source
---------------------------------------------
V.8 sequence silence INFO0a  A  !A  L1  L2  A  silence  S  !S  PP  TRN  silence


High speed training sequences:

Caller
------
S  !S  MD  S  !S  PP  TRN  J  J'  TRN  MP  MP'  E  B1  Data

Answer
------
S  !S  MD  S  !S  PP  TRN  J  [wait]  S  !S  TRN  MP  MP'  E  B1  Data

Control channel training sequences:

PPh  ALT  MPh  MPh  E  Data

or

Sh  !Sh  ALT  PPh  ALT  MPh  MPh  E  Data


J       Repetitions of 0x8990 for a 4 point constellation, or 0x89B0 for a 16 point constellation.
J'      A single transmission of 0x899F.
MD      Manufacturer specific training sequence
PP      Preliminary training sequence. 8 symbols, repeated 4 times.
S       90 degree alternations
!S      180 degree shift from S
Sh      90 degree alternations
!Sh     180 degree shift from S
TRN     Training sequence 4 or 16 symbols scrambled by the scrambler
MP      Modulation parameter sequence
MP'     Modulation parameter sequence with the acknowledgement bit set
MPh     Modulation parameter sequence
A       2400Hz
!A      Phase reversed 2400Hz
B       1200Hz
!B      Phase reversed 1200Hz
ALT
E
*/

/*
Power is -16.328760dBm0 V.8

Power is -16.416633dBm0 info0
Power is -16.359079dBm0 tone
Power is -11.280243dBm0 L1
Power is -17.304279dBm0 L2
Power is -16.403152dBm0 tone


Power is -18.137333dBm0 S


Power is -17.194252dBm0 CC

Phase 1 is all nominal

A is 2400Hz 1dB below nominal + guard at nominal

B is 1200Hz at nominal

INFO is 2400Hz at 1dB below nominal + guard 7dB below nominal

INFO is 1200Hz at nominal

L1 is 6dB above nominal

L2 is nominal

CC is 2400Hz at 1dB below nominal + guard 7dB below nominal

CC is 1200Hz at nominal
*/

/*
    Framing terminology:
        2 symbols makes a 4D symbol (k = 0, 1)
        4 4D symbols makes a mapping frame (j = 0, 1, 2, 3)
        P mapping frames makes a data frame (35 or 40ms) (P = 12, 14, 15 or 16)
        J data frames makes a super frame (280ms) (J = 7 or 8)
*/

static void tx_silence_init(v34_state_t *s, int duration);
static void transmission_preamble_init(v34_state_t *s);
static void info0_baud_init(v34_state_t *s);
static void initial_ab_not_ab_baud_init(v34_state_t *s);
static complex_sig_t get_initial_fdx_a_not_a_baud(v34_state_t *s);
static void v90_arm_tone_a_detection(v34_state_t *s, const char *reason);
static void v90_wait_rx_l2_init(v34_state_t *s, const char *reason);
static void l1_l2_signal_init(v34_state_t *s);
static int tx_pcm_l1_l2(v34_state_t *s, int16_t amp[], int max_len);
static void second_a_baud_init(v34_state_t *s);
static void post_l2_wait_tone_b_init(v34_state_t *s);
static void answer_resume_probe(v34_state_t *s, const char *reason);
static int post_info0_resume_bauds(void);
static int answer_info0_retry_policy(void);
static int post_l2_tone_b_wait_bauds(v34_state_t *s);
static void pre_info1_a_init(v34_state_t *s);
static void second_b_baud_init(v34_state_t *s);
static void v90_wait_tone_a_init(v34_state_t *s, bool preserve_tone_a_event);
static void v90_wait_info1a_init(v34_state_t *s);
static void v90_v34_fallback_wait_init(v34_state_t *s);
static complex_sig_t get_phase4_baud(v34_state_t *s);
static void info1_baud_init(v34_state_t *s);
static int info1a_repeats(v34_state_t *s);
static void infomarksa_baud_init(v34_state_t *s);
static int info1c_wait_bauds(v34_state_t *s);
static void infoh_baud_init(v34_state_t *s);
static complex_sig_t get_s_not_s_baud(v34_state_t *s);
static void s_not_s_baud_init(v34_state_t *s);
static void pp_baud_init(v34_state_t *s);
static void trn_baud_init(v34_state_t *s);
static void phase4_wait_init(v34_state_t *s);
static complex_sig_t get_v34_answer_phase3_wait_j_baud(v34_state_t *s);
static void v34_answer_phase3_wait_j_init(v34_state_t *s);
static void phase4_rx_conditioning_init(v34_state_t *s, int initial_stage, const char *reason);
static int mp_rate_n_is_valid(int rate_n);
static void v34_tx_get_mp_rates(v34_state_t *s, int *bit_rate_a_to_c, int *bit_rate_c_to_a);
static void mp_or_mph_baud_init(v34_state_t *s);
static void e_baud_init(v34_state_t *s);
static void data_baud_init(v34_state_t *s);

/* Control channel startup routines */
static void pph_baud_init(v34_state_t *s);
static void first_alt_baud_init(v34_state_t *s);
static void second_alt_baud_init(v34_state_t *s);
static void sh_baud_init(v34_state_t *s);

static void reset_primary_rx_frontend_for_phase3(v34_state_t *s)
{
    s->rx.agc_scaling = 0.0017f/V34_RX_PULSESHAPER_GAIN;
    s->rx.carrier_track_i = 5000.0f;
    s->rx.carrier_track_p = 40000.0f;
    s->rx.total_baud_timing_correction = 0;
    s->rx.carrier_phase = 0;
    s->rx.baud_half = 0;
    s->rx.rrc_filter_step = 0;
    memset(s->rx.rrc_filter, 0, sizeof(s->rx.rrc_filter));

    cvec_zerof(s->rx.eq_coeff, V34_EQUALIZER_PRE_LEN + 1 + V34_EQUALIZER_POST_LEN);
    s->rx.eq_coeff[V34_EQUALIZER_PRE_LEN] = complex_sig_set(TRAINING_SCALE(1.0f), TRAINING_SCALE(0.0f));
    cvec_zerof(s->rx.eq_buf, V34_EQUALIZER_MASK + 1);
    s->rx.eq_step = 0;
    s->rx.eq_put_step = V34_RX_PULSESHAPER_COEFF_SETS*10/(3*2) - 1;
    s->rx.eq_delta = EQUALIZER_DELTA/(V34_EQUALIZER_PRE_LEN + 1 + V34_EQUALIZER_POST_LEN);
    s->rx.eq_target_mag = 0.0f;

#if defined(SPANDSP_USE_FIXED_POINT)
    s->rx.pri_ted.symbol_sync_low[0] = s->rx.pri_ted.symbol_sync_low[1] = 0;
    s->rx.pri_ted.symbol_sync_high[0] = s->rx.pri_ted.symbol_sync_high[1] = 0;
    s->rx.pri_ted.symbol_sync_dc_filter[0] = s->rx.pri_ted.symbol_sync_dc_filter[1] = 0;
    s->rx.pri_ted.baud_phase = 0;
#else
    s->rx.pri_ted.symbol_sync_low[0] = s->rx.pri_ted.symbol_sync_low[1] = 0.0f;
    s->rx.pri_ted.symbol_sync_high[0] = s->rx.pri_ted.symbol_sync_high[1] = 0.0f;
    s->rx.pri_ted.symbol_sync_dc_filter[0] = s->rx.pri_ted.symbol_sync_dc_filter[1] = 0.0f;
    s->rx.pri_ted.baud_phase = 0.0f;
#endif
}

static __inline__ int scramble(v34_tx_state_t *s, int in_bit)
{
    int out_bit;

    /* One of the scrambler taps is a variable, so it can be adjusted for caller or answerer operation. */
    out_bit = (in_bit ^ (s->scramble_reg >> s->scrambler_tap) ^ (s->scramble_reg >> (23 - 1))) & 1;
    s->scramble_reg = (s->scramble_reg << 1) | out_bit;
    return out_bit;
}
/*- End of function --------------------------------------------------------*/

static uint16_t crc_bit_block(const uint8_t buf[], int first_bit, int last_bit, uint16_t crc)
{
    int pre;
    int post;

    /* Calculate the CRC between first_bit and last_bit, inclusive, of buf */
    last_bit++;
    pre = first_bit & 0x7;
    first_bit >>= 3;
    if (pre)
    {
        crc = crc_itu16_bits(buf[first_bit] >> pre, (8 - pre), crc);
        first_bit++;
    }
    /*endif*/
    post = last_bit & 0x7;
    last_bit >>= 3;
    if ((last_bit - first_bit) != 0)
        crc = crc_itu16_calc(buf + first_bit, last_bit - first_bit, crc);
    /*endif*/
    if (post)
        crc = crc_itu16_bits(buf[last_bit], post, crc);
    /*endif*/
    return crc;
}
/*- End of function --------------------------------------------------------*/

static logging_state_t *tx_log_state(v34_tx_state_t *s)
{
    v34_state_t *owner;

    owner = (v34_state_t *) ((char *) s - offsetof(v34_state_t, tx));
    return &owner->logging;
}
/*- End of function --------------------------------------------------------*/

static int info0_sequence_tx(v34_tx_state_t *s)
{
    uint8_t *t;
    uint16_t crc;
    bitstream_state_t bs;

    if (s->v90_mode && !s->calling_party)
    {
        /* V.90 INFO0d frame (ITU-T V.90 Table 7) — 62 bits total.
           Transmitted by the digital modem (answerer) at 1200 Hz carrier. */
        span_log(tx_log_state(s), SPAN_LOG_FLOW, "Tx INFO0d (V.90):\n");
        bitstream_init(&bs, true);
        t = s->txbuf;
        /* 0:3      Fill bits: 1111. */
        /* 4:11     Frame sync: 01110010 */
        bitstream_put(&bs, &t, INFO_FILL_AND_SYNC_BITS, 12);
        /* 12       Symbol rate 2743 supported in V.34 mode */
        bitstream_put(&bs, &t, (v34_capabilities.support_baud_rate_low_carrier[V34_BAUD_RATE_2743])  ?  1  :  0, 1);
        /* 13       Symbol rate 2800 supported in V.34 mode */
        bitstream_put(&bs, &t, (v34_capabilities.support_baud_rate_low_carrier[V34_BAUD_RATE_2800])  ?  1  :  0, 1);
        /* 14       Symbol rate 3429 supported in V.34 mode */
        bitstream_put(&bs, &t, (v34_capabilities.support_baud_rate_low_carrier[V34_BAUD_RATE_3429])  ?  1  :  0, 1);
        /* 15       Ability to transmit at low carrier frequency with symbol rate 3000 */
        bitstream_put(&bs, &t, (v34_capabilities.support_baud_rate_low_carrier[V34_BAUD_RATE_3000])  ?  1  :  0, 1);
        /* 16       Ability to transmit at high carrier frequency with symbol rate 3000 */
        bitstream_put(&bs, &t, (v34_capabilities.support_baud_rate_high_carrier[V34_BAUD_RATE_3000])  ?  1  :  0, 1);
        /* 17       Ability to transmit at low carrier frequency with symbol rate 3200 */
        bitstream_put(&bs, &t, (v34_capabilities.support_baud_rate_low_carrier[V34_BAUD_RATE_3200])  ?  1  :  0, 1);
        /* 18       Ability to transmit at high carrier frequency with symbol rate 3200 */
        bitstream_put(&bs, &t, (v34_capabilities.support_baud_rate_high_carrier[V34_BAUD_RATE_3200])  ?  1  :  0, 1);
        /* 19       Set to 0 indicates transmission with symbol rate 3429 is disallowed */
        bitstream_put(&bs, &t, (v34_capabilities.rate_3429_allowed)  ?  1  :  0, 1);
        /* 20       Ability to reduce transmit power */
        bitstream_put(&bs, &t, (v34_capabilities.support_power_reduction)  ?  1  :  0, 1);
        /* 21:23    Max allowed difference in symbol rates */
        bitstream_put(&bs, &t, v34_capabilities.max_baud_rate_difference, 3);
        /* 24       CME modem */
        bitstream_put(&bs, &t, v34_capabilities.from_cme_modem, 1);
        /* 25       1664-point signal constellations supported */
        bitstream_put(&bs, &t, (v34_capabilities.support_1664_point_constellation)  ?  1  :  0, 1);
        /* 26:27    V.92 Table 15 extensions.  Bit 26 asks for short Phase 2;
           bit 27 advertises V.92 capability.  Keep both clear by default so
           a V.90 application cannot accidentally select a procedure it has
           not enabled explicitly. */
        span_log(tx_log_state(s), SPAN_LOG_FLOW,
                 "Tx INFO0d V.92 flags: short-phase2(bit26)=%d, capability(bit27)=%d\n",
                 s->v92_short_phase2_requested ? 1 : 0,
                 s->v92_info0_capable ? 1 : 0);
        bitstream_put(&bs, &t,
                      (s->v92_short_phase2_requested ? 1 : 0)
                    | (s->v92_info0_capable ? 2 : 0),
                      2);
        /* 28       Acknowledge correct reception of INFO0a during error recovery */
        bitstream_put(&bs, &t, s->info0_acknowledgement, 1);
        /* 29:32    Digital modem nominal transmit power for Phase 2.
                    Represented in -1 dBm0 steps: 0 = -6 dBm0, 15 = -21 dBm0.
                    V.90 Table 7 requires the transmitted value, not the level
                    requested from the linear modulator.  The byte-exact G.711
                    output measures -13 dBm0 for L2 and the INFO carrier, so
                    advertise that wire level rather than the old -10 dBm0. */
        bitstream_put(&bs, &t, 7, 4);
        /* 33:37    Maximum digital modem transmit power.
                    Represented in -0.5 dBm0 steps: 0 = -0.5 dBm0, 31 = -16 dBm0.
                    -13 dBm0 -> code = (13 - 0.5) / 0.5 = 25. */
        bitstream_put(&bs, &t, 25, 5);
        /* 38       Power measurement at codec output (1) or modem terminals (0).
                    SIP/RTP = codec output */
        bitstream_put(&bs, &t, 1, 1);
        /* 39       PCM coding in use: 0 = u-law, 1 = A-law */
        bitstream_put(&bs, &t, s->v90_pcm_law, 1);
        /* 40       Set to 1 = ability to operate V.90 with upstream symbol rate 3429 */
        bitstream_put(&bs, &t, 0, 1);
        /* 41       Reserved — set to 0 */
        bitstream_put(&bs, &t, 0, 1);
        bitstream_emit(&bs, &t);
        crc = crc_bit_block(s->txbuf, 12, 41, 0xFFFF);
        /* 42:57    CRC */
        bitstream_put(&bs, &t, crc, 16);
        /* 58:61    Fill bits: 1111 */
        bitstream_put(&bs, &t, 0xF, 4);
        /* Extra postamble for byte alignment */
        bitstream_put(&bs, &t, 0, 8);
        bitstream_flush(&bs, &t);
        span_log(tx_log_state(s), SPAN_LOG_FLOW,
                 "  PCM law: %s, nominal power: -13 dBm0, max power: -13 dBm0\n",
                 s->v90_pcm_law ? "A-law" : "u-law");
        return 62;
    }

    log_info0(tx_log_state(s), true, &v34_capabilities, s->info0_acknowledgement);
    bitstream_init(&bs, true);
    t = s->txbuf;
    /* 0:3      Fill bits: 1111. */
    /* 4:11     Frame sync: 01110010, where the left-most bit is first in time. */
    bitstream_put(&bs, &t, INFO_FILL_AND_SYNC_BITS, 12);
    /* 12       Set to 1 indicates symbol rate 2743 is supported. */
    bitstream_put(&bs, &t, (v34_capabilities.support_baud_rate_low_carrier[V34_BAUD_RATE_2743])  ?  1  :  0, 1);
    /* 13       Set to 1 indicates symbol rate 2800 is supported. */
    bitstream_put(&bs, &t, (v34_capabilities.support_baud_rate_low_carrier[V34_BAUD_RATE_2800])  ?  1  :  0, 1);
    /* 14       Set to 1 indicates symbol rate 3429 is supported. */
    bitstream_put(&bs, &t, (v34_capabilities.support_baud_rate_low_carrier[V34_BAUD_RATE_3429])  ?  1  :  0, 1);
    /* 15       Set to 1 indicates the ability to transmit at the low carrier frequency with a symbol rate of 3000. */
    bitstream_put(&bs, &t, (v34_capabilities.support_baud_rate_low_carrier[V34_BAUD_RATE_3000])  ?  1  :  0, 1);
    /* 16       Set to 1 indicates the ability to transmit at the high carrier frequency with a symbol rate of 3000. */
    bitstream_put(&bs, &t, (v34_capabilities.support_baud_rate_high_carrier[V34_BAUD_RATE_3000])  ?  1  :  0, 1);
    /* 17       Set to 1 indicates the ability to transmit at the low carrier frequency with a symbol rate of 3200. */
    bitstream_put(&bs, &t, (v34_capabilities.support_baud_rate_low_carrier[V34_BAUD_RATE_3200])  ?  1  :  0, 1);
    /* 18       Set to 1 indicates the ability to transmit at the high carrier frequency with a symbol rate of 3200. */
    bitstream_put(&bs, &t, (v34_capabilities.support_baud_rate_high_carrier[V34_BAUD_RATE_3200])  ?  1  :  0, 1);
    /* 19       Set to 0 indicates that transmission with a symbol rate of 3429 is disallowed. */
    bitstream_put(&bs, &t, (v34_capabilities.rate_3429_allowed)  ?  1  :  0, 1);
    /* 20       Set to 1 indicates the ability to reduce transmit power to a value lower than the nominal setting. */
    bitstream_put(&bs, &t, (v34_capabilities.support_power_reduction)  ?  1  :  0, 1);
    /* 21:23    Maximum allowed difference in symbol rates in the transmit and receive directions. With the symbol rates
                labelled in increasing order, where 0 represents 2400 and 5 represents 3429, an integer between 0 and 5
                indicates the difference allowed in number of symbol rate steps. */
    bitstream_put(&bs, &t, v34_capabilities.max_baud_rate_difference, 3);
    /* 24       Set to 1 in an INFO0 sequence transmitted from a CME modem. */
    bitstream_put(&bs, &t, v34_capabilities.from_cme_modem, 1);
    /* 25       Set to 1 indicates the ability to support up to 1664-point signal constellations. */
    bitstream_put(&bs, &t, (v34_capabilities.support_1664_point_constellation)  ?  1  :  0, 1);
    /* 26:27    Transmit clock source: 0 = internal; 1 = synchronized to receive timing; 2 = external; 3 = reserved for ITU-T. */
    bitstream_put(&bs, &t, v34_capabilities.tx_clock_source, 2);
    /* 28       Set to 1 to acknowledge correct reception of an INFO0 frame during error recovery. */
    bitstream_put(&bs, &t, s->info0_acknowledgement, 1);
    bitstream_emit(&bs, &t);
    crc = crc_bit_block(s->txbuf, 12, 28, 0xFFFF);
    /* 29:44    CRC. */
    bitstream_put(&bs, &t, crc, 16);
    /* 45:48    Fill bits: 1111. */
    bitstream_put(&bs, &t, 0xF, 4);
    /* Add some extra postamble, so we have a whole number of bytes to work with. */
    bitstream_put(&bs, &t, 0, 8);
    bitstream_flush(&bs, &t);
    return 49;
}
/*- End of function --------------------------------------------------------*/

/* Turn the received L2 spectrum into one Table 17/V.92 probing-result block.
   V.34 leaves the receiver's projection algorithm implementation-specific;
   the transmitted fields themselves, however, are measurements, not a copy
   of the locally configured transmitter profile (V.34 10.1.2.3.4, V.92
   Table 17).  Select the carrier and one of Tables 3/4's pre-emphasis shapes
   by minimizing the ripple of channel-response × filter-response over the
   occupied band, then constrain the projected rate by measured in-band SNR. */
static int v34_l2_probe_result(v34_state_t *s,
                               int baud_idx,
                               int *high_carrier,
                               int *pre_emphasis,
                               int configured_max_n)
{
    static const int tone_present[25] =
    {
        1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1,
        1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
    };
    float baud;
    float noise;
    float best_score;
    float best_min_gain;
    int best_carrier;
    int best_filter;
    int carrier;
    int filter;
    int i;

    if (s->rx.l1_l2_gain_count <= 0)
        return 0;
    /*endif*/
    baud = 2400.0f*baud_rate_parameters[baud_idx].a/baud_rate_parameters[baud_idx].c;
    noise = (s->rx.l1_l2_noise_count > 0)
          ? s->rx.l1_l2_noise_sum/s->rx.l1_l2_noise_count
          : 1.0f;
    if (noise < 1.0f)
        noise = 1.0f;
    /*endif*/
    best_score = 1.0e30f;
    best_min_gain = 0.0f;
    best_carrier = 0;
    best_filter = 0;
    for (carrier = 0;  carrier < 2;  carrier++)
    {
        float centre;
        bool supported;

        supported = carrier
                  ? s->rx.far_capabilities.support_baud_rate_high_carrier[baud_idx]
                  : s->rx.far_capabilities.support_baud_rate_low_carrier[baud_idx];
        if (!supported)
            continue;
        /*endif*/
        centre = baud*baud_rate_parameters[baud_idx].low_high[carrier].d
                      /baud_rate_parameters[baud_idx].low_high[carrier].e;
        for (filter = 0;  filter <= 10;  filter++)
        {
            float sum;
            float sum2;
            float min_gain;
            int n;

            sum = 0.0f;
            sum2 = 0.0f;
            min_gain = 1.0e30f;
            n = 0;
            for (i = 0;  i < 25;  i++)
            {
                float f;
                float channel_gain;
                float filter_gain;
                float response_db;

                if (!tone_present[i])
                    continue;
                f = 150.0f*(i + 1);
                if (f < centre - 0.48f*baud  ||  f > centre + 0.48f*baud)
                    continue;
                channel_gain = s->rx.l1_l2_gain_sum[i]/s->rx.l1_l2_gain_count;
                if (channel_gain <= 0.0f)
                    continue;
                filter_gain = 1.0f;
                if (filter > 0)
                {
                    float re;
                    float im;
                    int tap;

                    re = 0.0f;
                    im = 0.0f;
                    for (tap = 0;  tap < 16;  tap++)
                    {
                        float angle;

                        angle = 2.0f*3.14159265f*f*tap/8000.0f;
                        re += v34_tx_pre_emphasis_filters[baud_idx][carrier][filter - 1][tap]*cosf(angle);
                        im -= v34_tx_pre_emphasis_filters[baud_idx][carrier][filter - 1][tap]*sinf(angle);
                    }
                    filter_gain = sqrtf(re*re + im*im);
                }
                if (filter_gain < 1.0e-6f)
                    filter_gain = 1.0e-6f;
                response_db = 20.0f*log10f(channel_gain*filter_gain);
                sum += response_db;
                sum2 += response_db*response_db;
                if (channel_gain < min_gain)
                    min_gain = channel_gain;
                n++;
            }
            /*endfor*/
            if (n >= 4)
            {
                float score;

                score = sum2/n - (sum/n)*(sum/n);
                if (score < best_score)
                {
                    best_score = score;
                    best_min_gain = min_gain;
                    best_carrier = carrier;
                    best_filter = filter;
                }
            }
            /*endif*/
        }
        /*endfor*/
    }
    /*endfor*/
    if (best_score >= 1.0e29f  ||  best_min_gain <= 0.0f)
        return 0;
    /*endif*/
    *high_carrier = best_carrier;
    *pre_emphasis = best_filter;
    {
        float snr_db;
        int snr_max_n;

        snr_db = 20.0f*log10f(best_min_gain/noise);
        snr_max_n = (int) floorf((snr_db - 4.0f)/2.0f);
        if (snr_max_n < 1)
            return 0;
        if (snr_max_n > 14)
            snr_max_n = 14;
        span_log(tx_log_state(&s->tx), SPAN_LOG_FLOW,
                 "Tx INFO1d measured probe: baud=%d carrier=%s pre=%d ripple=%.2f snr=%.1f dB rate=%d\n",
                 baud_rate_parameters[baud_idx].baud_rate,
                 best_carrier ? "high" : "low", best_filter,
                 sqrtf(best_score), snr_db,
                 (snr_max_n < configured_max_n ? snr_max_n : configured_max_n)*2400);
        return (snr_max_n < configured_max_n) ? snr_max_n : configured_max_n;
    }
}
/*- End of function --------------------------------------------------------*/

static __inline__ float carrier_frequency(int symbol_rate_code, int low_high);

static int v34_probe_frequency_offset(const v34_state_t *s)
{
    float offset_hz;
    int offset_code;

    if (s->rx.l1_l2_1050_phase_step_count <= 0)
        return -512;
    /* V.34 10.1.2.3.4/.5: units are 0.02 Hz and -512 means that the
       required 0.25 Hz measurement accuracy was unavailable. */
    offset_hz = (s->rx.l1_l2_1050_phase_step_sum
               / s->rx.l1_l2_1050_phase_step_count)
              / (2.0f*3.14159265f*0.020f);
    offset_code = (int) lroundf(offset_hz/0.02f);
    return (offset_code >= -511 && offset_code <= 511) ? offset_code : -512;
}
/*- End of function --------------------------------------------------------*/

static int v34_configured_rate_n(const v34_state_t *s, int baud_idx)
{
    int configured_max;
    int rate_cap;

    configured_max = (s->tx.parms.max_bit_rate_code >> 1) + 1;
    rate_cap = (baud_rate_parameters[baud_idx].max_bit_rate_code >> 1) + 1;
    if (configured_max > rate_cap)
        configured_max = rate_cap;
    /* Tables 15 and 16 forbid projected rates above 28800 unless the
       remote INFO0 advertises the 1664-point constellation. */
    if (!s->rx.far_capabilities.support_1664_point_constellation
        && configured_max > 12)
        configured_max = 12;
    return configured_max;
}
/*- End of function --------------------------------------------------------*/

static bool v34_local_tx_carrier_supported(int baud_idx, int high_carrier)
{
    if (baud_idx == V34_BAUD_RATE_3429 && !v34_capabilities.rate_3429_allowed)
        return false;
    return high_carrier
         ? v34_capabilities.support_baud_rate_high_carrier[baud_idx]
         : v34_capabilities.support_baud_rate_low_carrier[baud_idx];
}
/*- End of function --------------------------------------------------------*/

static void prepare_info1c(v34_state_t *s)
{
    int i;
    bool v92_info1d;

    s->tx.info1c.power_reduction = 0;
    s->tx.info1c.additional_power_reduction = 0;
    s->tx.info1c.md = 0;
    s->tx.info1c.freq_offset = 0;
    /* V.92 §9.3 switches INFO1d to Table 17 only after the two INFO0
       capability bits agree.  Table 17 keeps the V.34 probing fields, but
       repurposes bit 70 as PCM-upstream support.  That bit is what actually
       decides V.92 vs V.90: an analogue peer reading a zero has no data-pump
       feature left to gain and answers V.90 (observed live -- slmodemd logs
       "V92 capabilities: local=1 , remote=1 , selected=90" in the same
       instant it consumes our INFO1d).  Default off because the upstream
       data path here is still V.34; see v34_set_v92_pcm_upstream_capability. */
    v92_info1d = s->tx.v90_mode
              && s->tx.v92_info0_capable
              && (s->rx.info0_raw_26_27 & 0x01U) != 0
              && (s->rx.info0_raw_26_27 & 0x02U) == 0;

    for (i = 0;  i <= V34_BAUD_RATE_3429;  i++)
    {
        int configured_max;
        int measured_carrier;
        int measured_pre_emphasis;

        /* V.34 10.1.2.3.4/Table 15: every enabled INFO1c row is the
           receiver's L1/L2 result, not a restatement of the startup profile.
           The same Table-15 fields are reused by V.90 INFO1d. */
        configured_max = (s->tx.baud_rate >= i) ? v34_configured_rate_n(s, i) : 0;
        measured_carrier = 0;
        measured_pre_emphasis = 0;
        if (i == V34_BAUD_RATE_3429
            && (!s->rx.far_capabilities.rate_3429_allowed
                || !v34_capabilities.rate_3429_allowed))
            configured_max = 0;
        s->tx.info1c.rate_data[i].max_bit_rate =
            configured_max > 0
            ? v34_l2_probe_result(s, i,
                                  &measured_carrier,
                                  &measured_pre_emphasis,
                                  configured_max)
            : 0;
        s->tx.info1c.rate_data[i].use_high_carrier = measured_carrier != 0;
        s->tx.info1c.rate_data[i].pre_emphasis = measured_pre_emphasis;
        /* V.90 §8.2.3.2 Tables 9/10: INFO1a selects a row but does not
           repeat that row's carrier bit.  Retain exactly what this digital
           modem is about to transmit so its upstream RX can apply it. */
        s->rx.local_info1c_high_carrier[i] =
            s->tx.info1c.rate_data[i].use_high_carrier;
    }
    s->tx.info1c.freq_offset = v34_probe_frequency_offset(s);
    if (v92_info1d)
    {
        /* The 3429-HI carrier selector that occupied bit 70 in V.34/V.90
           becomes the PCM-upstream capability bit in V.92 Table 17. */
        s->tx.info1c.rate_data[V34_BAUD_RATE_3429].use_high_carrier = false;
        span_log(tx_log_state(&s->tx), SPAN_LOG_FLOW,
                 "Tx INFO1d V.92 Table 17 selected (bit70 PCM-upstream=%d)\n",
                 s->tx.v92_pcm_upstream_capable ? 1 : 0);
    }
    s->tx.v92_info1d_mode = v92_info1d;
}
/*- End of function --------------------------------------------------------*/

static void prepare_info1a(v34_state_t *s)
{
    int rx_carrier[6];
    int rx_pre_emphasis[6];
    int rx_max_rate[6];
    int allowed_difference;
    int answer_to_call;
    int call_to_answer;
    int best_score;
    int a;
    int c;

    s->tx.info1a.power_reduction = 0;
    s->tx.info1a.additional_power_reduction = 0;
    s->tx.info1a.md = 0;
    s->tx.info1a.freq_offset = v34_probe_frequency_offset(s);

    memset(rx_carrier, 0, sizeof(rx_carrier));
    memset(rx_pre_emphasis, 0, sizeof(rx_pre_emphasis));
    memset(rx_max_rate, 0, sizeof(rx_max_rate));
    for (c = V34_BAUD_RATE_2400; c <= V34_BAUD_RATE_3429; c++)
    {
        int configured_max;

        configured_max = (s->tx.baud_rate >= c) ? v34_configured_rate_n(s, c) : 0;
        if (c == V34_BAUD_RATE_3429
            && (!s->rx.far_capabilities.rate_3429_allowed
                || !v34_capabilities.rate_3429_allowed))
            configured_max = 0;
        if (configured_max > 0)
            rx_max_rate[c] = v34_l2_probe_result(s, c,
                                                 &rx_carrier[c],
                                                 &rx_pre_emphasis[c],
                                                 configured_max);
    }

    /* V.34 10.1.2.3.5/Table 16: INFO1a jointly selects answer->call from
       the caller's INFO1c rows and call->answer from this receiver's L1/L2
       result.  The two INFO0 values bound their symbol-rate difference. */
    allowed_difference = v34_capabilities.max_baud_rate_difference;
    if (allowed_difference > s->rx.far_capabilities.max_baud_rate_difference)
        allowed_difference = s->rx.far_capabilities.max_baud_rate_difference;
    answer_to_call = -1;
    call_to_answer = -1;
    best_score = -1;
    if (s->rx.info1c_received)
    {
        for (a = V34_BAUD_RATE_2400; a <= V34_BAUD_RATE_3429; a++)
        {
            int answer_rate;
            int answer_carrier;

            answer_rate = s->rx.info1c.rate_data[a].max_bit_rate;
            answer_carrier = s->rx.info1c.rate_data[a].use_high_carrier;
            if (answer_rate <= 0
                || !v34_local_tx_carrier_supported(a, answer_carrier))
                continue;
            if (answer_rate > v34_configured_rate_n(s, a))
                answer_rate = v34_configured_rate_n(s, a);
            for (c = V34_BAUD_RATE_2400; c <= V34_BAUD_RATE_3429; c++)
            {
                int score;
                int lower_rate;

                if (rx_max_rate[c] <= 0 || abs(a - c) > allowed_difference)
                    continue;
                lower_rate = (answer_rate < rx_max_rate[c])
                           ? answer_rate : rx_max_rate[c];
                /* Prefer the strongest duplex floor, then aggregate rate,
                   then the higher symbol rates.  Selection policy is local;
                   legality comes from INFO0/INFO1 and is enforced above. */
                score = lower_rate*10000 + (answer_rate + rx_max_rate[c])*100
                      + a*10 + c;
                if (score > best_score)
                {
                    best_score = score;
                    answer_to_call = a;
                    call_to_answer = c;
                }
            }
        }
    }

    if (best_score < 0)
    {
        /* §11.2.1.2.9 reaches INFO1a only after a valid INFO1c and probe.
           Do not manufacture a usable static profile if those prerequisites
           are absent; the Phase-2 recovery/retrain path must handle it. */
        answer_to_call = s->tx.baud_rate;
        call_to_answer = s->rx.baud_rate;
        s->tx.info1a.use_high_carrier = false;
        s->tx.info1a.preemphasis_filter = 0;
        s->tx.info1a.max_data_rate = 0;
        span_log(&s->logging, SPAN_LOG_WARNING,
                 "Tx INFO1a: no legal measured duplex symbol-rate pair\n");
    }
    else
    {
        s->tx.info1a.use_high_carrier = rx_carrier[call_to_answer] != 0;
        s->tx.info1a.preemphasis_filter = rx_pre_emphasis[call_to_answer];
        s->tx.info1a.max_data_rate = rx_max_rate[call_to_answer];

        /* This endpoint is the answer modem.  Configure its two primary
           channels from the independently selected Table-16 directions. */
        s->tx.baud_rate = answer_to_call;
        s->tx.high_carrier =
            s->rx.info1c.rate_data[answer_to_call].use_high_carrier;
        s->tx.parms.samples_per_symbol_numerator =
            baud_rate_parameters[answer_to_call].samples_per_symbol_numerator;
        s->tx.parms.samples_per_symbol_denominator =
            baud_rate_parameters[answer_to_call].samples_per_symbol_denominator;
        s->tx.v34_carrier_phase_rate =
            dds_phase_ratef(carrier_frequency(answer_to_call, s->tx.high_carrier));
        v34_rx_set_primary_channel(s, call_to_answer,
                                   s->tx.info1a.use_high_carrier);
    }

    s->tx.info1a.baud_rate_a_to_c = answer_to_call;
    s->tx.info1a.baud_rate_c_to_a = call_to_answer;
    span_log(&s->logging, SPAN_LOG_FLOW,
             "Tx INFO1a measured selection: answer->call=%d baud/%s, "
             "call->answer=%d baud/%s, projected=%d bps\n",
             baud_rate_parameters[answer_to_call].baud_rate,
             s->tx.high_carrier ? "high" : "low",
             baud_rate_parameters[call_to_answer].baud_rate,
             s->tx.info1a.use_high_carrier ? "high" : "low",
             s->tx.info1a.max_data_rate*2400);
}
/*- End of function --------------------------------------------------------*/

/* V.90 Table 15's maximum single-point magnitudes for INFO0d's maximum
   digital-modem transmit-power codes, from -0.5 through -16 dBm0.  Table 10
   requires U_INFO's training point not to exceed that announced maximum. */
static const int v90_max_point_level[32] =
{
    15124, 14276, 13480, 12724, 12012, 11340, 10708, 10108,
     9544,  9008,  8504,  8028,  7580,  7156,  6756,  6380,
     6020,  5684,  5368,  5068,  4784,  4516,  4264,  4024,
     3800,  3588,  3388,  3196,  3020,  2852,  2692,  2540
};

/* Positive G.711 reconstruction level for a universal code. */
static int v90_u_info_level(int pcm_law, int ucode)
{
    int mantissa;
    int segment;

    mantissa = ucode & 15;
    segment = (ucode >> 4) & 7;
    if (pcm_law) {
        if (segment == 0)
            return 8 + 16*mantissa;
        return (264 + 16*mantissa) << (segment - 1);
    }
    return (((mantissa << 3) + 132) << segment) - 132;
}

static int v90_select_info1a_baud(const v34_state_t *s)
{
    /* §6.2 makes 3200 mandatory.  SpanDSP also implements the optional 3000
       and 3429 rates; use one only when INFO1d says its receiver supports it. */
    static const int preference[] = {
        V34_BAUD_RATE_3200, V34_BAUD_RATE_3000, V34_BAUD_RATE_3429
    };

    if (s->rx.info1c_received) {
        for (unsigned i = 0; i < sizeof(preference)/sizeof(preference[0]); i++) {
            int baud = preference[i];

            if (s->rx.info1c.rate_data[baud].max_bit_rate > 0)
                return baud;
        }
    }
    /* No valid INFO1d should reach §9.2.2.1.9.  Retain the mandatory rate as
       a defensive fallback rather than put a reserved value in Table 10. */
    return V34_BAUD_RATE_3200;
}

static void prepare_v90_info1a(v34_state_t *s)
{
    int requested_u_info;

    /* V.90 §8.2.3.2 Table 10: INFO1a from the analog (calling) modem.
       Different field layout from standard V.34 INFO1a — carries U_INFO
       (Ucode for 2-point train) and upstream/downstream rate codes. */
    s->tx.info1a.power_reduction = 0;
    s->tx.info1a.additional_power_reduction = 0;
    s->tx.info1a.md = 0;
    if (s->rx.l1_l2_1050_phase_step_count > 0) {
        float offset_hz;
        int offset_code;

        offset_hz = (s->rx.l1_l2_1050_phase_step_sum
                   / s->rx.l1_l2_1050_phase_step_count)
                  / (2.0f*3.14159265f*0.020f);
        offset_code = (int) lroundf(offset_hz/0.02f);
        s->tx.info1a.freq_offset = (offset_code >= -511 && offset_code <= 511)
                                ? offset_code : -512;
    } else {
        /* Table 10: -512 says the required 0.25 Hz accuracy was unavailable. */
        s->tx.info1a.freq_offset = -512;
    }

    /* U_INFO is strictly greater than 66 and its point power may not exceed
       INFO0d's maximum.  Clamp a diagnostic preference rather than emit a
       non-conformant INFO1a; the effective value is exposed to the PCM RX. */
    requested_u_info = (s->tx.v90_u_info >= 67 && s->tx.v90_u_info < 128)
                     ? s->tx.v90_u_info : 78;
    if (s->rx.info0d_extensions_valid) {
        int limit = v90_max_point_level[s->rx.info0d_max_power_code & 31];

        while (requested_u_info > 67
               && v90_u_info_level(s->tx.v90_pcm_law, requested_u_info) > limit) {
            requested_u_info--;
        }
    }
    s->tx.v90_u_info = requested_u_info;
    s->tx.info1a.max_data_rate = requested_u_info;
    s->tx.info1a.use_high_carrier = false;
    s->tx.info1a.preemphasis_filter = 0;

    /* Table 10 bits 34:36 must select a rate INFO1d enabled. */
    s->tx.baud_rate = v90_select_info1a_baud(s);
    s->tx.info1a.baud_rate_a_to_c = s->tx.baud_rate;
    /* Downstream: 8000 PCM sampling = code 6. */
    s->tx.info1a.baud_rate_c_to_a = 6;
}
/*- End of function --------------------------------------------------------*/

static void prepare_infoh(v34_state_t *s)
{
    s->tx.infoh.power_reduction = 0;
    s->tx.infoh.length_of_trn = 30;
    s->tx.infoh.use_high_carrier = 0;
    s->tx.infoh.preemphasis_filter = 0;
    s->tx.infoh.baud_rate = 14;
    s->tx.infoh.trn16 = 0;
}
/*- End of function --------------------------------------------------------*/

static int info1c_sequence_tx(v34_tx_state_t *s, info1c_t *info1c)
{
    uint8_t *t;
    uint16_t crc;
    bitstream_state_t bs;
    int i;
    bool v92_info1d;

    log_info1c(tx_log_state(s), true, info1c);
    v92_info1d = s->v92_info1d_mode;
    bitstream_init(&bs, true);
    t = s->txbuf;
    /* 0:3      Fill bits: 1111. */
    /* 4:11     Frame sync: 01110010, where the left-most bit is first in time. */
    bitstream_put(&bs, &t, INFO_FILL_AND_SYNC_BITS, 12);
    /* 12:14    Minimum power reduction to be implemented by the answer modem transmitter. An integer between 0 and 7
                gives the recommended power reduction in dB. These bits shall indicate 0 if INFO0a indicated that the answer
                modem transmitter cannot reduce its power. */
    bitstream_put(&bs, &t, info1c->power_reduction, 3);
    /* 15:17    Additional power reduction, below that indicated by bits 12-14, which can be tolerated by the call modem
                receiver. An integer between 0 and 7 gives the additional power reduction in dB. These bits shall indicate 0 if
                INFO0a indicated that the answer modem transmitter cannot reduce its power. */
    bitstream_put(&bs, &t, info1c->additional_power_reduction, 3);
    /* 18:24    Length of MD to be transmitted by the call modem during Phase 3. An integer between 0 and 127 gives the
                length of this sequence in 35 ms increments. */
    bitstream_put(&bs, &t, info1c->md, 7);
    /* 25       Set to 1 indicates that the high carrier frequency is to be used in transmitting from the answer modem to the call
                modem for a symbol rate of 2400. */
    /* 26:29    Pre-emphasis filter to be used in transmitting from the answer modem to the call modem for a symbol
                rate of 2400. These bits form an integer between 0 and 10 which represents the pre-emphasis filter index
                (see Tables 3 and 4). */
    /* 30:33    Projected maximum data rate for a symbol rate of 2400. These bits form an integer between 0 and 14 which
                gives the projected data rate as a multiple of 2400 bits/s. A 0 indicates the symbol rate cannot be used. */

    /* 34:42    Probing results pertaining to a final symbol rate selection of 2743 symbols per second. The coding of these
                9 bits is identical to that for bits 25-33. */

    /* 43:51    Probing results pertaining to a final symbol rate selection of 2800 symbols per second. The coding of these
                9 bits is identical to that for bits 25-33. */

    /* 52:60    Probing results pertaining to a final symbol rate selection of 3000 symbols per second. The coding of these
                9 bits is identical to that for bits 25-33. Information in this field shall be consistent with the answer modem
                capabilities indicated in INFO0a. */

    /* 61:69    Probing results pertaining to a final symbol rate selection of 3200 symbols per second. The coding of these
                9 bits is identical to that for bits 25-33. Information in this field shall be consistent with the answer modem
                capabilities indicated in INFO0a. */

    /* 70:78 are the normal 3429 probing result in V.34/V.90.  In V.92
       Table 17, bit 70 instead advertises PCM upstream, followed by the
       eight 3429 pre-emphasis/rate bits. */
    for (i = 0;  i < (v92_info1d ? 5 : 6);  i++)
    {
        bitstream_put(&bs, &t, info1c->rate_data[i].use_high_carrier, 1);
        bitstream_put(&bs, &t, info1c->rate_data[i].pre_emphasis, 4);
        bitstream_put(&bs, &t, info1c->rate_data[i].max_bit_rate, 4);
    }
    if (v92_info1d)
    {
        /* bit 70: PCM upstream capability (see v34_set_v92_pcm_upstream_capability) */
        bitstream_put(&bs, &t, s->v92_pcm_upstream_capable ? 1 : 0, 1);
        bitstream_put(&bs, &t,
                      info1c->rate_data[V34_BAUD_RATE_3429].pre_emphasis,
                      4);
        bitstream_put(&bs, &t,
                      info1c->rate_data[V34_BAUD_RATE_3429].max_bit_rate,
                      4);
    }
    /*endfor*/
    /* 79:88    Frequency offset of the probing tones as measured by the call modem receiver. The frequency offset number
                shall be the difference between the nominal 1050 Hz line probing signal tone received and the 1050 Hz tone
                transmitted, f(received) and f(transmitted). A two's complement signed integer between -511 and 511 gives the
                measured offset in 0.02 Hz increments. Bit 88 is the sign bit of this integer. The frequency offset measurement
                shall be accurate to 0.25 Hz. Under conditions where this accuracy cannot be achieved, the integer shall be set
                to -512 indicating that this field is to be ignored. */
    bitstream_put(&bs, &t, info1c->freq_offset, 10);
    bitstream_emit(&bs, &t);
    crc = crc_bit_block(s->txbuf, 12, 88, 0xFFFF);
    /* 89:104   CRC. */
    bitstream_put(&bs, &t, crc, 16);
    /* 105:108  Fill bits: 1111. */
    bitstream_put(&bs, &t, 0xF, 4);
    /* Add some extra postamble, so we have a whole number of bytes to work with. */
    bitstream_put(&bs, &t, 0, 8);
    bitstream_flush(&bs, &t);
    return 109;
}
/*- End of function --------------------------------------------------------*/

static int info1a_sequence_tx(v34_tx_state_t *s, info1a_t *info1a)
{
    uint8_t *t;
    uint16_t crc;
    bitstream_state_t bs;

    log_info1a(tx_log_state(s), true, info1a);
    bitstream_init(&bs, true);
    t = s->txbuf;
    /* 0:3      Fill bits: 1111. */
    /* 4:11     Frame sync: 01110010, where the left-most bit is first in time. */
    bitstream_put(&bs, &t, INFO_FILL_AND_SYNC_BITS, 12);
    /* 12:14    Minimum power reduction to be implemented by the call modem transmitter. An integer between 0 and 7 gives
                the recommended power reduction in dB. These bits shall indicate 0 if INFO0c indicated that the call modem
                transmitter cannot reduce its power. */
    bitstream_put(&bs, &t, info1a->power_reduction, 3);
    /* 15:17    Additional power reduction, below that indicated by bits 12:14, which can be tolerated by the answer modem
                receiver. An integer between 0 and 7 gives the additional power reduction in dB. These bits shall indicate 0 if
                INFO0c indicated that the call modem transmitter cannot reduce its power. */
    bitstream_put(&bs, &t, info1a->additional_power_reduction, 3);
    /* 18:24    Length of MD to be transmitted by the answer modem during Phase 3. An integer between 0 and 127 gives the
                length of this sequence in 35 ms increments. */
    bitstream_put(&bs, &t, info1a->md, 7);
    /* 25       Set to 1 indicates that the high carrier frequency is to be used in transmitting from the call modem to the answer
                modem. This shall be consistent with the capabilities of the call modem indicated in INFO0c. */
    bitstream_put(&bs, &t, info1a->use_high_carrier, 1);
    /* 26:29    Pre-emphasis filter to be used in transmitting from the call modem to the answer modem. These bits form an
                integer between 0 and 10 which represents the pre-emphasis filter index (see Tables 3 and 4). */
    bitstream_put(&bs, &t, info1a->preemphasis_filter, 4);
    /* 30:33    Projected maximum data rate for the selected symbol rate from the call modem to the answer modem. These bits
                form an integer between 0 and 14 which gives the projected data rate as a multiple of 2400 bits/s. */
    bitstream_put(&bs, &t, info1a->max_data_rate, 4);
    /* 34:36    Symbol rate to be used in transmitting from the answer modem to the call modem. An integer between 0 and 5
                gives the symbol rate, where 0 represents 2400 and a 5 represents 3429. The symbol rate selected shall be
                consistent with information in INFO1c and consistent with the symbol rate asymmetry allowed as indicated in
                INFO0a and INFO0c. The carrier frequency and pre-emphasis filter to be used are those already indicated for
                this symbol rate in INFO1c. */
    bitstream_put(&bs, &t, info1a->baud_rate_a_to_c, 3);
    /* 37:39    Symbol rate to be used in transmitting from the call modem to the answer modem. An integer between 0 and 5
                gives the symbol rate, where 0 represents 2400 and a 5 represents 3429. The symbol rate selected shall be
                consistent with the capabilities indicated in INFO0a and consistent with the symbol rate asymmetry allowed as
                indicated in INFO0a and INFO0c. */
    bitstream_put(&bs, &t, info1a->baud_rate_c_to_a, 3);
    /* 40:49    Frequency offset of the probing tones as measured by the answer modem receiver. The frequency offset number
                shall be the difference between the nominal 1050 Hz line probing signal tone received and the 1050 Hz tone
                transmitted, f(received) and f(transmitted). A two's complement signed integer between -511 and 511 gives the
                measured offset in 0.02 Hz increments. Bit 49 is the sign bit of this integer. The frequency offset measurement
                shall be accurate to 0.25 Hz. Under conditions where this accuracy cannot be achieved, the integer shall be set
                to -512 indicating that this field is to be ignored. */
    bitstream_put(&bs, &t, info1a->freq_offset, 10);
    bitstream_emit(&bs, &t);
    crc = crc_bit_block(s->txbuf, 12, 49, 0xFFFF);
    /* 50:65    CRC. */
    bitstream_put(&bs, &t, crc, 16);
    /* 66:69    Fill bits: 1111. */
    bitstream_put(&bs, &t, 0xF, 4);
    /* Add some extra postamble, so we have a whole number of bytes to work with. */
    bitstream_put(&bs, &t, 0, 8);
    bitstream_flush(&bs, &t);
    return 70;
}
/*- End of function --------------------------------------------------------*/

static int v90_info1a_sequence_tx(v34_tx_state_t *s, info1a_t *info1a)
{
    uint8_t *t;
    uint16_t crc;
    bitstream_state_t bs;

    span_log(tx_log_state(s), SPAN_LOG_FLOW, "Tx INFO1a (V.90 Table 10):\n");
    span_log(tx_log_state(s), SPAN_LOG_FLOW, "  MD = %dms, U_INFO = %d, upstream rate code = %d, downstream rate code = %d\n",
             info1a->md*35, info1a->max_data_rate, info1a->baud_rate_a_to_c, info1a->baud_rate_c_to_a);
    bitstream_init(&bs, true);
    t = s->txbuf;
    /* 0:3      Fill bits: 1111. */
    /* 4:11     Frame sync: 01110010 */
    bitstream_put(&bs, &t, INFO_FILL_AND_SYNC_BITS, 12);
    /* 12:17    Reserved for ITU (set to 0 by analog modem) */
    bitstream_put(&bs, &t, 0, 6);
    /* 18:24    Length of MD to be transmitted by the analog modem during Phase 3. */
    bitstream_put(&bs, &t, info1a->md, 7);
    /* 25:31    U_INFO: Ucode of the PCM codeword to be used for the 2-point training
                signal by the digital modem (7 bits). */
    bitstream_put(&bs, &t, info1a->max_data_rate, 7);
    /* 32:33    Reserved for ITU (set to 0) */
    bitstream_put(&bs, &t, 0, 2);
    /* 34:36    Symbol rate for upstream (analog→digital). 3=3000, 4=3200, 5=3429 */
    bitstream_put(&bs, &t, info1a->baud_rate_a_to_c, 3);
    /* 37:39    Symbol rate code 6 = 8000 PCM downstream rate */
    bitstream_put(&bs, &t, info1a->baud_rate_c_to_a, 3);
    /* 40:49    Frequency offset (same as V.34) */
    bitstream_put(&bs, &t, info1a->freq_offset, 10);
    bitstream_emit(&bs, &t);
    crc = crc_bit_block(s->txbuf, 12, 49, 0xFFFF);
    /* 50:65    CRC. */
    bitstream_put(&bs, &t, crc, 16);
    /* 66:69    Fill bits: 1111. */
    bitstream_put(&bs, &t, 0xF, 4);
    /* Add some extra postamble, so we have a whole number of bytes to work with. */
    bitstream_put(&bs, &t, 0, 8);
    bitstream_flush(&bs, &t);
    return 70;
}
/*- End of function --------------------------------------------------------*/

static int infoh_sequence_tx(v34_tx_state_t *s, infoh_t *infoh)
{
    uint8_t *t;
    uint16_t crc;
    bitstream_state_t bs;

    log_infoh(tx_log_state(s), true, infoh);
    bitstream_init(&bs, true);
    t = s->txbuf;
    /* 0:3      Fill bits: 1111. */
    /* 4:11     Frame sync: 01110010, where the left-most bit is first in time. */
    bitstream_put(&bs, &t, INFO_FILL_AND_SYNC_BITS, 12);
    /* 12:14    Power reduction requested by the recipient modem receiver. An integer between 0 and 7
                gives the requested power reduction in dB. These bits shall indicate 0 if the source
                modem's INFO0 indicated that the source modem transmitter cannot reduce its power. */
    bitstream_put(&bs, &t, infoh->power_reduction, 3);
    /* 15:21    Length of TRN to be transmitted by the source modem during Phase 3. An integer between
                0 and 127 gives the length of this sequence in 35 ms increments. */
    bitstream_put(&bs, &t, infoh->length_of_trn, 7);
    /* 22       Set to 1 indicates the high carrier frequency is to be used in data mode transmission. This
                must be consistent with the capabilities indicated in the source modem's INFO0. */
    bitstream_put(&bs, &t, infoh->use_high_carrier, 1);
    /* 23:26    Pre-emphasis filter to be used in transmitting from the source modem to the recipient modem.
                These bits form an integer between 0 and 10 which represents the pre-emphasis filter index
                (see Tables 3 and 4). */
    bitstream_put(&bs, &t, infoh->preemphasis_filter, 4);
    /* 27:29    Symbol rate to be used for data transmission. An integer between 0 and 5 gives the symbol rate, where 0
                represents 2400 and a 5 represents 3429. */
    bitstream_put(&bs, &t, infoh->baud_rate, 3);
    /* 30       Set to 1 indicates TRN uses a 16-point constellation, 0 indicates TRN uses a 4-point constellation. */
    bitstream_put(&bs, &t, infoh->trn16, 1);
    bitstream_emit(&bs, &t);
    crc = crc_bit_block(s->txbuf, 12, 30, 0xFFFF);
    /* 31:46    Code CRC. */
    bitstream_put(&bs, &t, crc, 16);
    /* 47:50    Fill bits: 1111. */
    bitstream_put(&bs, &t, 0xF, 4);
    /* Add some extra postamble, so we have a whole number of bytes to work with. */
    bitstream_put(&bs, &t, 0, 8);
    bitstream_flush(&bs, &t);
    return 51;
}
/*- End of function --------------------------------------------------------*/

static int mp_sequence_tx(v34_tx_state_t *s, mp_t *mp)
{
    int i;
    int len;
    uint8_t *t;
    uint16_t crc;
    bitstream_state_t bs;

    bitstream_init(&bs, true);
    t = s->txbuf;
    /* 0:16     Frame sync: 11111111111111111. */
    /* 17       Start bit: 0. */
    bitstream_put(&bs, &t, 0x1FFFF, 18);
    /* 18       Type: 0 or 1. */
    bitstream_put(&bs, &t, mp->type, 1);
    /* 19       Reserved for ITU-T: This bit is set to 0 by the transmitting modem and is not
                interpreted by the receiving modem. */
    bitstream_put(&bs, &t, 0, 1);
    /* 20:23    Maximum call modem to answer modem data signalling rate: Data rate = N * 2400
                where N is a four-bit integer between 1 and 14. */
    bitstream_put(&bs, &t, mp->bit_rate_c_to_a, 4);
    /* 24:27    Maximum answer modem to call modem data signalling rate: Data rate = N * 2400
                where N is a four-bit integer between 1 and 14. */
    bitstream_put(&bs, &t, mp->bit_rate_a_to_c, 4);
    /* 28       Auxiliary channel select bit. Set to 1 if modem is capable of supporting and
                enables auxiliary channel. Auxiliary channel is used only if both modems set
                this bit to 1. */
    bitstream_put(&bs, &t, mp->aux_channel_supported, 1);
    /* 29:30    Trellis encoder select bits:
                0 = 16 state; 1 = 32 state; 2 = 64 state; 3 = Reserved for ITU-T.
                Receiver requires remote-end transmitter to use selected trellis encoder. */
    bitstream_put(&bs, &t, mp->trellis_size, 2);
    /* 31       Non-linear encoder parameter select bit for the remote-end transmitter.
                0: Q = 0, 1: Q = 0.3125. */
    bitstream_put(&bs, &t, mp->use_non_linear_encoder, 1);
    /* 32       Constellation shaping select bit for the remote-end transmitter.
                0: minimum, 1: expanded (see Table 10). */
    bitstream_put(&bs, &t, mp->expanded_shaping, 1);
    /* 33       Acknowledge bit. 0 = modem has not received MP from far end. 1 = received MP from far end. */
    bitstream_put(&bs, &t, mp->mp_acknowledged, 1);
    /* 34       Start bit: 0. */
    bitstream_put(&bs, &t, 0, 1);
    /* 35:49    Data signalling rate capability mask.
                Bit 35:2400; bit 36:4800; bit 37:7200;...; bit 46:28 800; bit 47:31 200; bit 48:33 600;
                bit 49: Reserved for ITU-T. (This bit is set to 0 by the transmitting modem and is not
                interpreted by the receiving modem.) Bits set to 1 indicate data signalling rates supported
                and enabled in both transmitter and receiver of modem. */
    bitstream_put(&bs, &t, mp->signalling_rate_mask, 15);
    /* 50       Asymmetric data signalling rate enable. Set to 1 indicates modem capable of asymmetric
                data signalling rates. */
    bitstream_put(&bs, &t, mp->asymmetric_rates_allowed, 1);
    if (mp->type == 1)
    {
        /* 51       Start bit: 0. */
        /* 52:67    Precoding coefficient h(1) real. */
        /* 68       Start bit: 0. */
        /* 69:84    Precoding coefficient h(1) imaginary. */
        /* 85       Start bit: 0. */
        /* 86:101   Precoding coefficient h(2) real. */
        /* 102      Start bit: 0. */
        /* 103:118  Precoding coefficient h(2) imaginary. */
        /* 119      Start bit: 0. */
        /* 120:135  Precoding coefficient h(3) real. */
        /* 136      Start bit: 0. */
        /* 137:152  Precoding coefficient h(3) imaginary. */
        for (i = 0;  i < 3;  i++)
        {
            bitstream_put(&bs, &t, 0, 1);
            bitstream_put(&bs, &t, mp->precoder_coeffs[i].re, 16);
            bitstream_put(&bs, &t, 0, 1);
            bitstream_put(&bs, &t, mp->precoder_coeffs[i].im, 16);
        }
        /*endfor*/
    }
    /*endif*/
    /* 51/153           Start bit: 0. */
    bitstream_put(&bs, &t, 0, 1);
    /* 52:67/154:169    Reserved for ITU-T: These bits are set to 0 by the transmitting modem and are
                        not interpreted by the receiving modem. */
    bitstream_put(&bs, &t, 0, 16);
    /* 68/170           Start bit: 0. */
    bitstream_put(&bs, &t, 0, 1);
    bitstream_emit(&bs, &t);
    crc = 0xFFFF;
    len = (mp->type == 1)  ?  170  :  68;
    for (i = 17;  i < len;  i += 17)
        /* Per V.34 10.1.2.3.2, CRC excludes frame sync/start/fill bits.
           Each 17-bit block is start + 16 information bits, so CRC the
           16 information bits only. */
        crc = crc_bit_block(s->txbuf, i + 1, i + 16, crc);
    /*endfor*/
    /* 69:84/171:186    CRC. */
    bitstream_put(&bs, &t, crc, 16);
    /* 85:87 Fill bits: 000.    187 Fill bit: 0. */
    if (mp->type == 1)
        bitstream_put(&bs, &t, 0, 1);
    else
        bitstream_put(&bs, &t, 0, 3);
    /*endif*/
    /* Add some extra postamble, so we have a whole number of bytes to work with. */
    bitstream_put(&bs, &t, 0, 8);
    bitstream_flush(&bs, &t);

    /* TX diagnostic: dump the exact serialized MP frame bits (LSB-first stream)
       to compare directly against RX Phase 4 MP frame reconstruction. */
    {
        bitstream_state_t dbg_bs;
        const uint8_t *dbg_p;
        int dbg_len;
        char dbg_bits[188 + 1];

        dbg_len = (mp->type == 1) ? 188 : 88;
        bitstream_init(&dbg_bs, true);
        dbg_p = s->txbuf;
        for (i = 0;  i < dbg_len;  i++)
            dbg_bits[i] = (char) ('0' + (int) bitstream_get(&dbg_bs, &dbg_p, 1));
        /*endfor*/
        dbg_bits[dbg_len] = '\0';
        span_log(tx_log_state(s), SPAN_LOG_FLOW,
                 "Tx - MP%d bits[0..%d]: %s\n",
                 mp->type, dbg_len - 1, dbg_bits);
    }

    return (mp->type == 1)  ?  188  :  88;
}
/*- End of function --------------------------------------------------------*/

static int mph_sequence_tx(v34_tx_state_t *s, mph_t *mph)
{
    int i;
    int len;
    uint8_t *t;
    uint16_t crc;
    bitstream_state_t bs;

    log_mph(tx_log_state(s), true, mph);
    bitstream_init(&bs, true);
    t = s->txbuf;
    /* 0:16     Frame sync: 11111111111111111. */
    /* 17       Start bit: 0. */
    bitstream_put(&bs, &t, 0x1FFFF, 18);
    /* 18       Type: */
    bitstream_put(&bs, &t, mph->type, 1);
    /* 19       Reserved for ITU-T: This bit is set to 0 by the transmitting modem and is not
                interpreted by the receiving modem. */
    bitstream_put(&bs, &t, 0, 1);
    /* 20:23    Maximum data signalling rate:
                Data rate = N * 2400 where N is a four-bit integer between 1 and 14. */
    bitstream_put(&bs, &t, mph->max_data_rate, 4);
    /* 24:26    Reserved for ITU-T: These bits are set to 0 by the transmitting modem and are
                not interpreted by the receiving modem. */
    bitstream_put(&bs, &t, 0, 3);
    /* 27       Control channel data signalling rate selected for remote transmitter.
                0 = 1200 bit/s, 1 = 2400 bit/s (see bit 50 below). */
    bitstream_put(&bs, &t, mph->control_channel_2400, 1);
    /* 28       Reserved for ITU-T: This bit is set to 0 by the transmitting modem and is not
                interpreted by the receiving modem. */
    bitstream_put(&bs, &t, 0, 1);
    /* 29:30    Trellis encoder select bits:
                0 = 16 state; 1 = 32 state; 2 = 64 state; 3 = Reserved for ITU-T.
                Receiver requires remote-end transmitter to use selected trellis encoder. */
    bitstream_put(&bs, &t, mph->trellis_size, 2);
    /* 31       Non-linear encoder parameter select bit for the remote-end transmitter.
                0: Q = 0, 1: Q = 0.3125. */
    bitstream_put(&bs, &t, mph->use_non_linear_encoder, 1);
    /* 32       Constellation shaping select bit for the remote-end transmitter.
                0: minimum, 1: expanded (see Table 10). */
    bitstream_put(&bs, &t, mph->expanded_shaping, 1);
    /* 33       Reserved for ITU-T: This bit is set to 0 by the transmitting modem and is not
                interpreted by the receiving modem. */
    bitstream_put(&bs, &t, 0, 1);
    /* 34       Start bit: 0. */
    bitstream_put(&bs, &t, 0, 1);
    /* 35:49    Data signalling rate capability mask.
                Bit 35:2400; bit 36:4800; bit 37:7200;...; bit 46:28 800; bit 47:31 200; bit 48:33 600;
                bit 49: Reserved for ITU-T. (This bit is set to 0 by the transmitting modem and is not
                interpreted by the receiving modem.) Bits set to 1 indicate data signalling rates supported
                and enabled in both transmitter and receiver of modem. */
    bitstream_put(&bs, &t, mph->signalling_rate_mask, 15);
    /* 50       Enables asymmetric control channel data rates:
                0 = Asymmetric mode not allowed; 1 = Asymmetric mode allowed.
                    Asymmetric mode shall be used only when both modems set bit 50 to 1. If different data
                rates are selected in symmetric mode, both modems shall transmit at the lower rate. */
    bitstream_put(&bs, &t, mph->asymmetric_rates_allowed, 1);
    if (mph->type == 1)
    {
        /* 51       Start bit: 0. */
        /* 52:67    Precoding coefficient h(1) real. */
        /* 68       Start bit: 0. */
        /* 69:84    Precoding coefficient h(1) imaginary. */
        /* 85       Start bit: 0. */
        /* 86:101   Precoding coefficient h(2) real. */
        /* 102      Start bit: 0. */
        /* 103:118  Precoding coefficient h(2) imaginary. */
        /* 119      Start bit: 0. */
        /* 120:135  Precoding coefficient h(3) real. */
        /* 136      Start bit: 0. */
        /* 137:152  Precoding coefficient h(3) imaginary. */
        for (i = 0;  i < 3;  i++)
        {
            bitstream_put(&bs, &t, 0, 1);
            bitstream_put(&bs, &t, mph->precoder_coeffs[i].re, 16);
            bitstream_put(&bs, &t, 0, 1);
            bitstream_put(&bs, &t, mph->precoder_coeffs[i].im, 16);
        }
        /*endfor*/
    }
    /*endif*/
    /* 51/153           Start bit: 0. */
    bitstream_put(&bs, &t, 0, 1);
    /* 52:67/154:169    Reserved for ITU-T: These bits are set to 0 by the transmitting modem and are not
                        interpreted by the receiving modem. */
    bitstream_put(&bs, &t, 0, 16);
    /* 68/170           Start bit: 0. */
    bitstream_put(&bs, &t, 0, 1);
    bitstream_emit(&bs, &t);
    crc = 0xFFFF;
    len = (mph->type == 1)  ?  170  :  68;
    for (i = 17;  i < len;  i += 17)
        /* Per V.34 10.1.2.3.2, CRC excludes frame sync/start/fill bits.
           Each 17-bit block is start + 16 information bits, so CRC the
           16 information bits only. */
        crc = crc_bit_block(s->txbuf, i + 1, i + 16, crc);
    /*endfor*/
    /* 69:84/171:186    CRC. */
    bitstream_put(&bs, &t, crc, 16);
    /* 85:87 Fill bits: 000.    187 Fill bit: 0. */
    if (mph->type == 1)
        bitstream_put(&bs, &t, 0, 1);
    else
        bitstream_put(&bs, &t, 0, 3);
    /*endif*/
    /* Add some extra postamble, so we have a whole number of bytes to work with. */
    bitstream_put(&bs, &t, 0, 8);
    bitstream_flush(&bs, &t);
    return (mph->type == 1)  ?  188  :  88;
}
/*- End of function --------------------------------------------------------*/

static int fake_get_bit(void *user_data)
{
    return 1;
}
/*- End of function --------------------------------------------------------*/

static void parse_primary_channel_bitstream(v34_tx_state_t *s)
{
    uint8_t *u;
    const uint8_t *t;
    int i;
    int n;
    int bit;
    int bb;
    int kk;

    /* Parse a series of input data bits into a set of S bits, Q bits, and I bits which we can
       feed into the modulation process. */
    bitstream_init(&s->bs, true);
    u = s->txbuf;
    bb = s->parms.b;
    kk = s->parms.k;
    /* If there are S bits we switch between high mapping frames and low mapping frames based
       on the SWP pattern. We derive SWP algorithmically.  Note that high/low mapping is only
       relevant when b >= 12. */
    s->s_bit_cnt += s->parms.r;
    if (s->s_bit_cnt >= s->parms.p)
    {
        /* This is a high mapping frame */
        s->s_bit_cnt -= s->parms.p;
    }
    else
    {
        if (bb > 12)
        {
            /* We need one less bit in a low mapping frame */
            bb--;
            kk--;
        }
        /*endif*/
    }
    /*endif*/
    i = 0;
    /* The first of the I bits might be auxiliary data */
    s->aux_bit_cnt += s->parms.w;
    if (s->aux_bit_cnt >= s->parms.p)
    {
        s->aux_bit_cnt -= s->parms.p;
        /* Insert an auxiliary data bit after the K bits, where it will appear as
           the first of the I bits. */
        for (  ;  i < kk;  i++)
        {
            if ((bit = s->current_get_bit(s->get_bit_user_data)) == SIG_STATUS_END_OF_DATA)
            {
                /* TODO: Need to handle things properly here. SIG_STATUS_END_OF_DATA may not
                         mean shut down the modem. It may mean shut down the current mode, when
                         we are working half-duplex. */
                s->current_get_bit = fake_get_bit;
            }
            /*endif*/
            bitstream_put(&s->bs, &u, scramble(s, bit), 1);
        }
        /*endfor*/
        /* Auxiliary data bits are not scrambled (V.34/7) */
        bit = (s->get_aux_bit)  ?  s->get_aux_bit(s->get_aux_bit_user_data)  :  0;
        bitstream_put(&s->bs, &u, bit, 1);
        i++;
    }
    for (  ;  i < bb;  i++)
    {
        if ((bit = s->current_get_bit(s->get_bit_user_data)) == SIG_STATUS_END_OF_DATA)
        {
            /* TODO: Need to handle things properly here. SIG_STATUS_END_OF_DATA may not
                     mean shut down the modem. It may mean shut down the current mode, when
                     we are working half-duplex. */
            s->current_get_bit = fake_get_bit;
        }
        /*endif*/
        bitstream_put(&s->bs, &u, scramble(s, bit), 1);
    }
    /*endfor*/
    bitstream_flush(&s->bs, &u);

    bitstream_init(&s->bs, true);
    t = s->txbuf;
    if (s->parms.k)
    {
        /* V.34/9.3.1 */
        /* K is always < 32, so we always get the entire K bits from a single word */
        s->r0 = bitstream_get(&s->bs, &t, kk);
        for (i = 0;  i < 4;  i++)
        {
            /* Some I bits. These are always present, and always 3 bits each. */
            s->ibits[i] = bitstream_get(&s->bs, &t, 3);
            /* Maybe some uncoded Q bits. */
            if (s->parms.q)
            {
                s->qbits[2*i] = bitstream_get(&s->bs, &t, s->parms.q);
                s->qbits[2*i + 1] = bitstream_get(&s->bs, &t, s->parms.q);
            }
            else
            {
                s->qbits[2*i] = 0;
                s->qbits[2*i + 1] = 0;
            }
            /*endif*/
        }
        /*endfor*/
    }
    else
    {
        /* V.34/9.3.2 */
        /* If K is zero (i.e. b = 8, 9, 11, or 12), things need slightly special treatment */
        /* Some I bits. These are always present, and may be 2 or 3 bits each. */
        /* Need to treat 8, 9, 11, and 12 individually */
        s->r0 = 0;
        n = bb - 8;
        for (i = 0;  i < n;  i++)
            s->ibits[i] = bitstream_get(&s->bs, &t, 3);
        /*endfor*/
        for (  ;  i < 4;  i++)
            s->ibits[i] = bitstream_get(&s->bs, &t, 2);
        /*endfor*/
        /* No uncoded Q bits */
        for (i = 0;  i < 8;  i++)
            s->qbits[i] = 0;
        /*endfor*/
    }
    /*endif*/
    /* Offline interoperability diagnostic: preserve the shell/Q bits while
       testing the six possible I1/I2/I3 serialization conventions against a
       captured reset-state B1 frame.  Zero (and an unset variable) is the
       normative I1,I2,I3 order. */
    {
        const char *perm_env = getenv("SPANDSP_V34_DIAG_I_PERM");
        static const uint8_t permutation[6][3] =
        {
            {0, 1, 2}, {0, 2, 1}, {1, 0, 2},
            {1, 2, 0}, {2, 0, 1}, {2, 1, 0}
        };
        int perm = perm_env ? atoi(perm_env) : 0;

        if (perm > 0  &&  perm < 6)
        {
            for (i = 0;  i < 4;  i++)
            {
                int original = s->ibits[i];

                s->ibits[i] = ((original >> permutation[perm][0]) & 1)
                            | (((original >> permutation[perm][1]) & 1) << 1)
                            | (((original >> permutation[perm][2]) & 1) << 2);
            }
            /*endfor*/
        }
        /*endif*/
    }
    span_log(tx_log_state(s),
             SPAN_LOG_FLOW,
             "Tx - Parsed %p %8X - %X %X %X %X - %2X %2X %2X %2X %2X %2X %2X %2X\n",
             s,
             s->r0,
             s->ibits[0],
             s->ibits[1],
             s->ibits[2],
             s->ibits[3],
             s->qbits[0],
             s->qbits[1],
             s->qbits[2],
             s->qbits[3],
             s->qbits[4],
             s->qbits[5],
             s->qbits[6],
             s->qbits[7]);
}
/*- End of function --------------------------------------------------------*/

static void shell_map(v34_tx_state_t *s)
{
    int32_t a;
    int32_t b;
    int32_t c;
    int32_t d;
    int32_t e;
    int32_t f;
    int32_t g;
    int32_t h;
    int32_t r[6];
    int32_t t1;
    int32_t t2;
    const uint32_t *g2;
    const uint32_t *g4;
    const uint32_t *z8;

    if (s->parms.m == 0)
    {
        s->mjk[0] = 0;
        s->mjk[1] = 0;
        s->mjk[2] = 0;
        s->mjk[3] = 0;
        s->mjk[4] = 0;
        s->mjk[5] = 0;
        s->mjk[6] = 0;
        s->mjk[7] = 0;
        return;
    }
    /*endif*/
    g2 = g2s[s->parms.m];
    g4 = g4s[s->parms.m];
    z8 = z8s[s->parms.m];

    /* TODO: This code comes directly from the equations in V.34. Can it be made faster? */

    for (a = 1;  z8[a] <= s->r0;  a++)
        /*loop*/;
    /*endfor*/
    /* We are now at a ring which is too big, so step back one */
    a--;

    /* V.34/9-8 */
    t2 = s->r0 - z8[a];
    for (b = -1;  t2 >= 0;  )
    {
        b++;
        t1 = g4[b]*g4[a - b];
        t2 -= t1;
    }
    /*endfor*/
    r[1] = t2 + t1;

    /* V.34/9-9 */
    r[2] = r[1]%g4[b];

    /* V.34/9-10 */
    r[3] = (r[1] - r[2])/g4[b];

    /* V.34/9-11 */
    t2 = r[2];
    for (c = -1;  t2 >= 0;  )
    {
        c++;
        t1 = g2[c]*g2[b - c];
        t2 -= t1;
    }
    /*endfor*/
    r[4] = t2 + t1;

    /* V.34/9-12 */
    t2 = r[3];
    for (d = -1;  t2 >= 0;  )
    {
        d++;
        t1 = g2[d]*g2[a - b - d];
        t2 -= t1;
    }
    /*endfor*/
    r[5] = t2 + t1;

    /* V.34/9-13 */
    e = r[4]%g2[c];
    /* V.34/9-14 */
    f = (r[4] - e)/g2[c];

    /* V.34/9-15 */
    g = r[5]%g2[d];
    /* V.34/9-16 */

    h = (r[5] - g)/g2[d];

    if (c < s->parms.m)
    {
        /* V.34/9-17 */
        s->mjk[0] = e;
        s->mjk[1] = c - s->mjk[0];
    }
    else
    {
        /* V.34/9-18 */
        s->mjk[1] = s->parms.m - 1 - e;
        s->mjk[0] = c - s->mjk[1];
    }
    /*endif*/

    if (b - c < s->parms.m)
    {
        /* V.34/9-19 */
        s->mjk[2] = f;
        s->mjk[3] = b - c - s->mjk[2];
    }
    else
    {
        /* V.34/9-20 */
        s->mjk[3] = s->parms.m - 1 - f;
        s->mjk[2] = b - c - s->mjk[3];
    }
    /*endif*/

    if (d < s->parms.m)
    {
        /* V.34/9-21 */
        s->mjk[4] = g;
        s->mjk[5] = d - s->mjk[4];
    }
    else
    {
        /* V.34/9-22 */
        s->mjk[5] = s->parms.m - 1 - g;
        s->mjk[4] = d - s->mjk[5];
    }
    /*endif*/

    if (a - b - d < s->parms.m)
    {
        /* V.34/9-23 */
        s->mjk[6] = h;
        s->mjk[7] = a - b - d - s->mjk[6];
    }
    else
    {
        /* V.34/9-24 */
        s->mjk[7] = s->parms.m - 1 - h;
        s->mjk[6] = a - b - d - s->mjk[7];
    }
    /*endif*/
}
/*- End of function --------------------------------------------------------*/

static complexi16_t v34_non_linear_encoder(complexi16_t *pre)
{
    int32_t zeta;
    int32_t x;
    complexi16_t post;

    /* V.34/9.7 for the 0.3125 case */
    /* 341/2048 is 1/6 */
    zeta = ((((int32_t) pre->re*(int32_t) pre->re + (int32_t) pre->im*(int32_t) pre->im + 0x800) >> 12)*341 + 0x800) >> 12;
    /* 15127/16384 is 0.92328 */
    /* 19661/65536 is 6*6/120 */
    x = (zeta*zeta + 0x2000) >> 14;
    x = (zeta + ((x*19661) >> 16)*15127 + 0x4000) >> 14;
    post.re = (int16_t) ((int32_t) pre->re*x >> 14);
    post.im = (int16_t) ((int32_t) pre->im*x >> 14);
    return post;
}
/*- End of function --------------------------------------------------------*/

static complexi16_t rotate90_clockwise(complexi16_t *x, int quads)
{
    complexi16_t y;

    /* Rotate a point clockwise by "quads" 90 degree steps */
    /* These are simple negate and swap operations */
    switch (quads & 3)
    {
    case 0:
        y.re = x->re;
        y.im = x->im;
        break;
    case 1:
        y.re = x->im;
        y.im = -x->re;
        break;
    case 2:
        y.re = -x->re;
        y.im = -x->im;
        break;
    case 3:
        y.re = -x->im;
        y.im = x->re;
        break;
    }
    /*endswitch*/
    return y;
}
/*- End of function --------------------------------------------------------*/

/* Determine the 3 bits subset label for a particular constellation point */
static int16_t get_binary_subset_label(complexi16_t *pos)
{
    int16_t x;
    int16_t xored;
    int16_t subset;

    /* See V.34/9.6.3.1 */
    xored = pos->re ^ pos->im;
    x = xored & 2;
    subset = ((xored & 4) ^ (x << 1)) | (pos->re & 2) | (x >> 1);
    //printf("XXX Pre subset %d,%d => %d\n", pos->re, pos->im, subset);
    return subset;
}
/*- End of function --------------------------------------------------------*/

static complexi16_t quantize_tx(v34_tx_state_t *s, complexi16_t *x)
{
    complexi16_t y;

    /* Value is stored in Q9.7 format. */
    y.re = abs(x->re);
    y.im = abs(x->im);
    if (s->parms.b >= 56)
    {
        /* 2w is 4 */
        /* Output integer values. i.e. 16:0 */
        /* We must mask out the 1st and 2nd bits, because we are rounding to the 3rd bit.
           All numbers coming out of this routine should be multiples of 4. */
        y.re = (y.re + 0x0FF) >> 7;
        y.re &= ~0x03;
        y.im = (y.im + 0x0FF) >> 7;
        y.im &= ~0x03;
    }
    else
    {
        /* 2w is 2 */
        /* Output integer values. i.e. Q16.0 */
        /* We must mask out the 1st bit because we are rounding to the 2nd bit
           All numbers coming out of this routine should be multiples of 2 (i.e. even). */
        y.re = (y.re + 0x07F) >> 7;
        y.re &= ~0x01;
        y.im = (y.im + 0x07F) >> 7;
        y.im &= ~0x01;
    }
    /*endif*/
    if (x->re < 0)
        y.re = -y.re;
    /*endif*/
    if (x->im < 0)
        y.im = -y.im;
    /*endif*/
    return y;
}
/*- End of function --------------------------------------------------------*/

static complexi16_t precoder_tx_filter(v34_tx_state_t *s)
{
    int i;
    int j;
    complexi32_t sum;
    complexi16_t p;

#if 1
#elif 0
s->precoder_coeffs[0].re = 0x1aee;
s->precoder_coeffs[0].re = 0x0308;
s->precoder_coeffs[1].re = 0xf995;
s->precoder_coeffs[1].re = 0x0065;
s->precoder_coeffs[2].re = 0x0de0;
s->precoder_coeffs[2].re = 0xfe37;
#else
s->precoder_coeffs[0].re = 0x1adb;
s->precoder_coeffs[0].re = 0x0230;
s->precoder_coeffs[1].re = 0xf95b;
s->precoder_coeffs[1].re = 0x0069;
s->precoder_coeffs[2].re = 0x0def;
s->precoder_coeffs[2].re = 0xfe34;
#endif
    /* 9.6.2/V.34 */
    /* h's are stored in Q2.14
       x's are stored in Q9.7
       not sure about x's
       so product is in Q11.21 */
    sum = complex_seti32(0, 0);
    for (i = 0;  i < 3;  i++)
    {
        j = V34_XOFF + s->step_2d - i;
        sum.re += (s->x[j].re*s->precoder_coeffs[i].re - s->x[j].im*s->precoder_coeffs[i].im);
        sum.im += (s->x[j].re*s->precoder_coeffs[i].im + s->x[j].im*s->precoder_coeffs[i].re);
    }
    /*endfor*/
    /* 9.6.2/V.34 item 2 - Round Q11.21 number format to Q9.7 */
    p.re = (abs(sum.re) + 0x01FFFL) >> 14;
    if (sum.re < 0)
        p.re = -p.re;
    /*endif*/
    p.im = (abs(sum.im) + 0x01FFFL) >> 14;
    if (sum.im < 0)
        p.im = -p.im;
    /*endif*/
    return p;
}
/*- End of function --------------------------------------------------------*/

static void qam_mod(v34_tx_state_t *s)
{
//    printf("QAM %p [%6d, %6d] [%8.3f, %8.3f] [%8.3f, %8.3f]\n",
//           s,
//           s->x[V34_XOFF + s->step_2d].re,
//           s->x[V34_XOFF + s->step_2d].im,
//           FP_Q9_7_TO_F(s->x[V34_XOFF + s->step_2d].re),
//           FP_Q9_7_TO_F(s->x[V34_XOFF + s->step_2d].im),
//           35.77*s->x[V34_XOFF + s->step_2d].re,
//           35.77*s->x[V34_XOFF + s->step_2d].im);
//    fflush(stdout);
}
/*- End of function --------------------------------------------------------*/

/* Keep this global until the modem is VERY well tested */
SPAN_DECLARE(int) v34_get_mapping_frame(v34_tx_state_t *s, int16_t bits[16])
{
    int len;
    int y4321;
    int c0;
    int u0;
    int v0;
    int rot;
    int32_t sum1;
    int32_t sum2;
    complexi16_t u;
    complexi16_t v;
    complexi16_t y;
    complexi16_t c_prev;
    int subsets[2];
    int mapping_index;

    /* This gets the four 4D symbols (eight 2D symbols) of a mapping frame */
    parse_primary_channel_bitstream(s);
    shell_map(s);

    u0 = 0;
    for (s->step_2d = 0;  s->step_2d < 8;  s->step_2d++)
    {
        /* Steps to map, precode and trellis code a 4D symbol (2 x 2D symbols)
           Step    Inputs                              Operation               Outputs
            1      Z(m), v(2m)                         9.6.1                   u(2m)
            2      u(2m), c(2m), p(2m)                 9.6.2, item 4           y(2m), x(2m)
            3      x(2m)                               9.6.2, items 1 to 3     c(2m + 1), p(2m + 1)
            4      c(2m), c(2m + 1)                    9.6.3.3                 C0(m)
            5      C0(m), Y0(m), V0(m)                 9.6.3                   U0(m)
            6      Z(m), U0(m), v(2m + 1)              9.6.1                   u(2m + 1)
            7      u(2m + 1), c(2m + 1), p(2m + 1)     9.6.2, item 4           y(2m + 1), x(2m + 1)
            8      x(2m + 1)                           9.6.2, items 1 to 3     c(2m + 2), p(2m + 2)
            9      y(2m), y(2m + 1)                    9.6.3.1, 9.6.3.2        Y0(m + 1) */
        /* 9.6.1/V.34 - Get the initial unrotated constellation point from the table. */
        mapping_index = ((s->mjk[s->step_2d] << s->parms.q) + s->qbits[s->step_2d]);
        v.re = v34_superconstellation[mapping_index][0];
        v.im = v34_superconstellation[mapping_index][1];
//printf("W %d %d %d %d\n", s->step_2d, s->mjk[s->step_2d], s->parms.q, s->qbits[s->step_2d]);
//printf("V %d - %d %d\n", mapping_index, v.re, v.im);
        if ((s->step_2d & 1) == 0)
        {
            /* Figure 6/V.34, 9.5/V.34 - Differential encoder */
            s->z = (s->z + (s->ibits[s->step_2d >> 1] >> 1)) & 3;
            /* Table 11/V.34 step 1, 9.6.1/V.34 - Rotation factor */
            rot = s->z;
        }
        else
        {
            /* Table 11/V.34 step 6, 9.6.1/V.34 - Compute rotation factor */
            rot = (s->z + ((s->ibits[s->step_2d >> 1] & 1) << 1) + u0) & 3;
        }
        /*endif*/
        u = rotate90_clockwise(&v, rot);
//printf("QMA %p [%6d, %6d] [%6d, %6d] (%d)\n", s, v.re, v.im, u.re, u.im, rot);

        /* Table 11/V.34 step 2/7, 9.6.2/V.34 item 4 - Compute the channel output signal y(n), and the precoded signal x(n) */
        y.re = u.re + s->c.re;
        y.im = u.im + s->c.im;
        s->x[V34_XOFF + s->step_2d].re = (y.re << 7) - s->p.re;
        s->x[V34_XOFF + s->step_2d].im = (y.im << 7) - s->p.im;

        subsets[s->step_2d & 1] = get_binary_subset_label(&y);
        qam_mod(s);
        bits[2*s->step_2d] = s->x[V34_XOFF + s->step_2d].re;
        bits[2*s->step_2d + 1] = s->x[V34_XOFF + s->step_2d].im;

        /* Table 11/V.34 step 3/8, 9.6.2/V.34 items 1 and 2 */
        s->p = precoder_tx_filter(s);
        if (s->use_non_linear_encoder)
            s->p = v34_non_linear_encoder(&s->p);
        /*endif*/
        c_prev = s->c;
        /* Table 11/V.34 step 3/8, 9.6.2/V.34 item 3 */
        s->c = quantize_tx(s, &s->p);

        if ((s->step_2d & 1) == 0)
        {
            /* Table 11/V.34 step 4, 9.6.3.3/V.34 */
            sum1 = (c_prev.re + c_prev.im) >> 1;
            sum2 = (s->c.re + s->c.im) >> 1;
            c0 = (sum1 ^ sum2) & 1;
            /* Superframe synchronisation bit inversion indicator */
            /* From Table 12/V.34. If J is 7, then 14 bits of this are used. If J is 8,
               all 16 bits are used. */
            /* Inversions are applied to the first 4D symbol in each half data frame. If P
               is 12, 14 or 16, the inversion will be in the first 4D symbol of a mapping frame.
               If P is 15, the inversions will alternate between being in the first and third 4D
               symbols of a mapping frame. */
            if ((s->data_frame*8 + s->step_2d)%(4*s->parms.p) == 0)
                v0 = (0x5FEE >> s->v0_pattern++) & 1;
            else
                v0 = 0;
            /*endif*/
            /* Table 11/V.34 step 5, 9.6.3/V.34 */
            u0 = (s->y0 ^ c0 ^ v0) & 1;
        }
        else
        {
            y4321 = conv_encode_input[subsets[0]][subsets[1]];
            {
                const char *perm_env =
                    getenv("SPANDSP_V34_DIAG_Y_PERM");
                int perm = perm_env ? atoi(perm_env) : 0;

                if (perm > 0  &&  perm < 24)
                {
                    int source[4] = {0, 1, 2, 3};
                    int source_count = 4;
                    int original = y4321;
                    int remapped = 0;

                    for (int dst = 0;  dst < 4;  dst++)
                    {
                        int choice = perm % source_count;
                        int src = source[choice];

                        perm /= source_count;
                        remapped |= ((original >> src) & 1) << dst;
                        for (int move = choice;
                             move + 1 < source_count;
                             move++)
                            source[move] = source[move + 1];
                        /*endfor*/
                        source_count--;
                    }
                    /*endfor*/
                    y4321 = remapped;
                }
                /*endif*/
            }
            /* Table 11/V.34 step 9, 9.6.3.1/V.34 and 9.6.3.2/V.34 */
            /* The convolutional encoder has one 4D-symbol interval of
               inherent delay (V.34 9.6.3.2).  The state transition consumes
               the subset labels from the pair just transmitted; the output
               cached for the next pair must therefore come from the updated
               state.  Caching the old state's output here adds a second
               interval of delay and rotates only the odd 2D point in every
               subsequent pair. */
            s->state = s->conv_encode_table[s->state][y4321];
            s->y0 = s->state & 1;
//printf("Y4321 %d %d - %d %d %d\n", subsets[0], subsets[1], y4321, s->y0, s->state);
//printf("WWW 0x%x 0x%x -> 0x%x\n", v0, y4321, s->state);
        }
        /*endif*/
    }
    /*endfor*/
    /* At the end of the eight 2D symbols of a mapping frame. We need to reset some buffers.
       These values are remembered from one mapping frame to the next. */
    s->x[V34_XOFF - 3] = s->x[V34_XOFF + 5];
    s->x[V34_XOFF - 2] = s->x[V34_XOFF + 6];
    s->x[V34_XOFF - 1] = s->x[V34_XOFF + 7];

    if (++s->data_frame >= s->parms.p)
    {
        s->data_frame = 0;

        if (++s->super_frame >= s->parms.j)
        {
            s->super_frame = 0;
            s->v0_pattern = 0;
        }
        /*endif*/
    }
    /*endif*/
//printf("QAZ data frame %d, super frame %d\n", s->data_frame, s->super_frame);

    len = 2*8;
    return len;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_get_mapping_frame_state(v34_state_t *s,
                                              int16_t bits[16])
{
    if (!s || !bits)
        return -1;
    return v34_get_mapping_frame(&s->tx, bits);
}
/*- End of function --------------------------------------------------------*/

static __inline__ float exact_baud_rate(int symbol_rate_code)
{
    float a;
    float c;

    a = baud_rate_parameters[symbol_rate_code].a;
    c = baud_rate_parameters[symbol_rate_code].c;
    return 2400.0f*a/c;
}
/*- End of function --------------------------------------------------------*/

static __inline__ float carrier_frequency(int symbol_rate_code, int low_high)
{
    float d;
    float e;

    d = baud_rate_parameters[symbol_rate_code].low_high[low_high].d;
    e = baud_rate_parameters[symbol_rate_code].low_high[low_high].e;
    return exact_baud_rate(symbol_rate_code)*d/e;
}
/*- End of function --------------------------------------------------------*/

static complex_sig_t get_silence_baud(v34_state_t *s)
{
    (void) s;
    return zero;
}
/*- End of function --------------------------------------------------------*/

static int get_data_bit(v34_tx_state_t *s)
{
    int bit;

    if (s->txptr >= s->txbits)
        return -1;
    /*endif*/
    bit = (s->txbuf[s->txptr >> 3] >> (s->txptr & 7)) & 1;
    s->txptr++;
    return bit;
}
/*- End of function --------------------------------------------------------*/

static complex_sig_t get_transmission_preamble_baud(v34_state_t *s)
{
    if (++s->tx.txptr >= s->tx.txbits)
        info0_baud_init(s);
    /*endif*/
    return s->tx.lastbit;
}
/*- End of function --------------------------------------------------------*/

static void transmission_preamble_init(v34_state_t *s)
{
    /* Send some bits as the modulator starts up, to allow things to stabilise before the
       important data goes out. */
    span_log(&s->logging, SPAN_LOG_FLOW, "Tx - transmission_preamble_init()\n");
    s->tx.txbits = 16;
    s->tx.txptr = 0;
    s->tx.lastbit = complex_sig_set(TRAINING_SCALE(TRAINING_AMP), TRAINING_SCALE(0.0f));
    s->tx.current_modulator = V34_MODULATION_CC;
    s->tx.current_getbaud = get_transmission_preamble_baud;
    s->tx.stage = V34_TX_STAGE_INITIAL_PREAMBLE;
}
/*- End of function --------------------------------------------------------*/

static complex_sig_t get_info0_baud(v34_state_t *s)
{
    enum
    {
        V90_INFO0_ACK_GUARD_RETRIES = 4,
        /* Absolute ceiling on INFO0d resends before we give up on the INFO0
           handshake and follow the peer into the Tone B/reversal transaction,
           regardless of the acknowledgement/INFO0a state the normal guard
           needs.  When the peer has already moved on to Tone A reversals but
           info0_received has gone stale, neither the resume branch (needs
           info0_received) nor the ack guard (needs ack && info0) fires, so
           without this the retry loop ran unbounded -- observed storming to 73
           resends, which the peer answered with a Link Error / NO CARRIER.
           Normal calls escape via the ack guard by ~7; 16 is clear of that. */
        V90_INFO0_RETRY_HARD_CAP = 16
    };
    int bit;

    bit = get_data_bit(&s->tx);
    if (s->tx.txptr >= s->tx.txbits)
    {
        if (s->tx.v90_mode
            && !s->tx.calling_party
            && s->tx.duplex)
        {
            if (s->rx.info0_received)
            {
                if (!s->tx.info0_acknowledgement
                    &&  s->tx.stage == V34_TX_STAGE_INFO0)
                {
                    /* V.90 9.2.1.1.1-9.2.1.1.2 error-free path: INFO0d is
                       transmitted once, followed by Tone B.  The ack'd
                       INFO0d repeat belongs to the 9.2.1.2.1 recovery
                       procedure (Tone A before INFO0a, or repeated INFO0a)
                       only.  Repeating INFO0d after a clean first exchange
                       makes spec-following analogue modems treat the repeat
                       itself as a recovery trigger, restart Phase 2, and
                       deadlock the INFO exchange. */
                    span_log(&s->logging, SPAN_LOG_FLOW,
                             "Tx - V.90: INFO0a received OK, proceeding to Tone B (9.2.1.1.2)\n");
                    v90_arm_tone_a_detection(s, "INFO0a received error-free");
                    s->rx.received_event = V34_EVENT_NONE;
                    initial_ab_not_ab_baud_init(s);
                    s->tx.stage = V34_TX_STAGE_V90_PHASE2_B_INFO0_SEEN;
                    if (bit)
                        s->tx.lastbit.re = -s->tx.lastbit.re;
                    /*endif*/
                    return s->tx.lastbit;
                }
                /*endif*/
                if (!s->tx.info0_acknowledgement)
                {
                    span_log(&s->logging, SPAN_LOG_FLOW,
                             "Tx - V.90: INFO0a received OK, setting INFO0d acknowledgement bit and repeating INFO0d\n");
                    s->tx.info0_acknowledgement = true;
                }
                else
                {
                    span_log(&s->logging, SPAN_LOG_FLOW,
                             "Tx - V.90: INFO0a received again (ack=%d), checking exit condition\n",
                             s->rx.info0_acknowledgement);
                }
                v90_arm_tone_a_detection(s, "INFO0a received during recovery");
                s->rx.received_event = V34_EVENT_NONE;
            }

            if (s->tx.info0_acknowledgement
                && (s->rx.info0_acknowledgement
                    || (s->rx.info0_received
                        && ((s->rx.stage == V34_RX_STAGE_TONE_A
                             && s->rx.received_event == V34_EVENT_TONE_SEEN)
                            || (s->rx.stage == V34_RX_STAGE_TONE_A
                                && s->rx.received_event == V34_EVENT_REVERSAL_1)))))
            {
                const char *reason;

                if (s->rx.info0_acknowledgement)
                    reason = "peer acknowledged INFO0d";
                else if (s->rx.received_event == V34_EVENT_REVERSAL_1)
                    reason = "Tone A reversal detected after INFO0a";
                else
                    reason = "Tone A detected after INFO0a";
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - V.90: %s; completing INFO0d and resuming the Tone B/reversal transaction\n",
                         reason);
                initial_ab_not_ab_baud_init(s);
                s->tx.stage = V34_TX_STAGE_V90_PHASE2_B_INFO0_SEEN;
            }
            else if (s->tx.info0_acknowledgement
                     || s->tx.stage == V34_TX_STAGE_INFO0_RETRY)
            {
                s->tx.info0_retry_count++;
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - V.90: INFO0d retry %d (rx_event=%d, rx_stage=%d, rx_ack=%d, rx_info0=%d, pers2=%d)\n",
                         s->tx.info0_retry_count,
                         s->rx.received_event,
                         s->rx.stage,
                         s->rx.info0_acknowledgement,
                         s->rx.info0_received,
                         s->rx.persistence2);
                if ((s->tx.info0_retry_count >= V90_INFO0_ACK_GUARD_RETRIES
                     && s->tx.info0_acknowledgement
                     && s->rx.info0_received)
                    || s->tx.info0_retry_count >= V90_INFO0_RETRY_HARD_CAP)
                {
                    /* Some peers keep repeating valid INFO0a but never reflect the
                       acknowledgement bit back to us. Once we've resent an
                       acknowledged INFO0d a few times and still have live INFO0a
                       from the far end, stop burning more retries and move back to
                       the Tone A/L1/L2 path.  The hard-cap arm also breaks the
                       storm where the peer has advanced to Tone A reversals but
                       info0_received went stale, so the ack-guard condition can
                       never be satisfied -- follow the peer into the reversal
                       transaction instead of resending INFO0d until Link Error. */
                    span_log(&s->logging, SPAN_LOG_FLOW,
                             "Tx - V.90: INFO0d retry guard reached (retry=%d, ack=%d, info0=%d); resuming the Tone B/reversal transaction\n",
                             s->tx.info0_retry_count,
                             s->tx.info0_acknowledgement,
                             s->rx.info0_received);
                    initial_ab_not_ab_baud_init(s);
                    s->tx.stage = V34_TX_STAGE_V90_PHASE2_B_INFO0_SEEN;
                }
                else
                {
                    info0_baud_init(s);
                }
            }
            else
            {
                initial_ab_not_ab_baud_init(s);
            }
        }
        /* Are we at the initial stage, where A or B comes next, or at the retry
           stage, where we keep repeating INFO0 */
        else if (s->tx.stage == V34_TX_STAGE_INFO0)
        {
            initial_ab_not_ab_baud_init(s);
        }
        else if (s->tx.stage == V34_TX_STAGE_INFO0_RETRY
                 &&  !s->tx.calling_party
                 &&  !s->tx.v90_mode
                 &&  s->tx.duplex
                 &&  s->tx.phase2_probe_sent)
        {
            /* The acknowledged INFO0a is complete.  Do not wait here for
               another INFO0c and do not repeat INFO0a while waiting: this
               call modem takes the acknowledgement and moves on by itself,
               and every further INFO0 it sees restarts its recovery. */
            answer_resume_probe(s, "INFO0 acknowledgement sent");
        }
        else if (s->tx.stage == V34_TX_STAGE_INFO0_RETRY
                 && s->rx.received_event == V34_EVENT_INFO0_OK)
        {
            /* A valid INFO0c arrived while we were retrying — go straight to Tone A */
            span_log(&s->logging, SPAN_LOG_FLOW, "Tx - INFO0_RETRY: INFO0c received OK, switching to Tone A\n");
            initial_ab_not_ab_baud_init(s);
        }
        else
        {
            info0_baud_init(s);
        }
        /*endif*/
    }
    /*endif*/
    if (bit)
        s->tx.lastbit.re = -s->tx.lastbit.re;
    /*endif*/
    return s->tx.lastbit;
}
/*- End of function --------------------------------------------------------*/

static void info0_baud_init(v34_state_t *s)
{
    span_log(&s->logging, SPAN_LOG_FLOW, "Tx - info0_baud_init()\n");
    s->tx.txbits = info0_sequence_tx(&s->tx);
    s->tx.txptr = 0;
    s->tx.lastbit = complex_sig_set(TRAINING_SCALE(TRAINING_AMP), TRAINING_SCALE(0.0f));
    s->tx.current_modulator = V34_MODULATION_CC;
    s->tx.stage = (s->tx.stage >= V34_TX_STAGE_INFO0)  ?  V34_TX_STAGE_INFO0_RETRY  :  V34_TX_STAGE_INFO0;
    s->tx.current_getbaud = get_info0_baud;
}
/*- End of function --------------------------------------------------------*/

static void v90_arm_tone_a_detection(v34_state_t *s, const char *reason)
{
    if (s->rx.stage != V34_RX_STAGE_TONE_A)
    {
        span_log(&s->logging, SPAN_LOG_FLOW,
                 "Tx - V.90: %s, conditioning RX for Tone A detection\n",
                 reason);
        s->rx.stage = V34_RX_STAGE_TONE_A;
        s->rx.persistence1 = 0;
        s->rx.persistence2 = 0;
        s->rx.received_event = V34_EVENT_NONE;
    }
}
/*- End of function --------------------------------------------------------*/

static void v90_wait_rx_l2_init(v34_state_t *s, const char *reason)
{
    span_log(&s->logging, SPAN_LOG_FLOW,
             "Tx - V.90: %s, completing INFO0d recovery and sending Tone B\n",
             reason);
    span_log(&s->logging, SPAN_LOG_FLOW,
             "Tx - V.90: arming RX for analog L1/L2 analysis during Tone B\n");
    s->rx.dft_ptr = 0;
    s->rx.base_phase = 42.0f;
    s->rx.l1_l2_duration = 0;
    s->rx.current_demodulator = V34_MODULATION_L1_L2;
    s->rx.stage = V34_RX_STAGE_L1_L2;
    s->rx.bit_count = 0;
    s->rx.bitstream = 0;
    s->rx.persistence1 = 0;
    s->rx.persistence2 = 0;
    s->rx.received_event = V34_EVENT_NONE;
    s->rx.last_logged_stage = -1;
    s->rx.last_logged_event = -1;
    s->rx.last_logged_demodulator = -1;
    s->tx.lastbit = complex_sig_set(TRAINING_SCALE(TRAINING_AMP), TRAINING_SCALE(0.0f));
    s->tx.current_modulator = V34_MODULATION_CC;
    s->tx.current_getbaud = get_initial_fdx_a_not_a_baud;
    s->tx.tone_duration = 0;
    s->tx.stage = V34_TX_STAGE_V90_WAIT_RX_L2;
}
/*- End of function --------------------------------------------------------*/

static int v90_note_phase2_info0_recovery(v34_state_t *s)
{
    s->tx.v90_phase2_info0_recovery_loops++;
    return s->tx.v90_phase2_info0_recovery_loops;
}
/*- End of function --------------------------------------------------------*/

static void v90_reset_phase2_rx_frontend(v34_state_t *s)
{
    power_meter_init(&s->rx.power, 4);
    s->rx.signal_present = false;
    s->rx.carrier_phase = 0;
    s->rx.agc_scaling = 0.01f;
    s->rx.bit_count = 0;
    s->rx.bitstream = 0;
    s->rx.duration = 0;
    s->rx.blip_duration = 0;
    s->rx.last_angles[0] = 0;
    s->rx.last_angles[1] = 0;
    s->rx.rrc_filter_step = 0;
    memset(s->rx.rrc_filter, 0, sizeof(s->rx.rrc_filter));
}
/*- End of function --------------------------------------------------------*/

static void v90_prime_info0a_tone_a_rx(v34_state_t *s, const char *reason)
{
    int preserve_active_search;

    preserve_active_search = (s->rx.current_demodulator == V34_MODULATION_TONES
                              && (s->rx.stage == V34_RX_STAGE_INFO0
                                  || s->rx.stage == V34_RX_STAGE_TONE_A
                                  || s->rx.stage == V34_RX_STAGE_TONE_B
                                  || s->rx.signal_present
                                  || s->rx.bit_count > 0
                                  || s->rx.persistence1 > 0
                                  || s->rx.persistence2 > 0));
    span_log(&s->logging, SPAN_LOG_FLOW,
             "Tx - V.90: %s, conditioning RX for INFO0a and Tone A detection%s\n",
             reason,
             preserve_active_search ? " (preserving active Phase 2 acquisition)" : "");
    if (!preserve_active_search)
        v90_reset_phase2_rx_frontend(s);
    else
    {
        /* Keep the live Phase 2 tone frontend warm across the INFO0d -> Tone A
           crossover. Resetting the power meter / RRC history here can miss a
           peer that starts INFO0a or Tone A immediately after our INFO0d.
           Critically, if an INFO frame is mid-accumulation (sync already
           matched, bit_count > 0), keep the accumulator, bitstream and CRC
           intact: this crossover fires while the peer is part-way through an
           INFO0a repetition, and zeroing bit_count here discards a
           nearly-complete frame, desynchronising the whole Phase 2 exchange
           (peer sees our INFO0d continue, enters error recovery, and we end
           up framing its INFO0a resends as INFO1a). */
    }
    /*endif*/
    s->rx.current_demodulator = V34_MODULATION_TONES;
    s->rx.stage = V34_RX_STAGE_TONE_A;
    if (s->rx.bit_count == 0)
        s->rx.target_bits = (s->rx.duplex)  ?  (49 - (4 + 8 + 4))  :  (51 - (4 + 8 + 4));
    if (!preserve_active_search)
    {
        s->rx.received_event = V34_EVENT_NONE;
        s->rx.persistence1 = 0;
        s->rx.persistence2 = 0;
    }
    s->rx.last_logged_stage = -1;
    s->rx.last_logged_event = -1;
    s->rx.last_logged_demodulator = -1;
}
/*- End of function --------------------------------------------------------*/

static complex_sig_t get_initial_fdx_a_not_a_baud(v34_state_t *s)
{
    /* Answering side */
    switch (s->tx.stage)
    {
    case V34_TX_STAGE_INITIAL_A:
        /* Send pure tone for at least 50ms (V.34/11.2.1.2.1) */
        if (++s->tx.tone_duration == 30)
        {
            /* 50ms minimum A period has passed - accept an incoming INFO0c */
            if (s->rx.received_event == V34_EVENT_REVERSAL_1
                &&  s->rx.info0_received)
            {
                /* INFO0 was already received and a reversal arrived during
                   INITIAL_A — skip FIRST_A and go straight to the 40ms
                   delay before sending our reversal back. */
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - INITIAL_A: INFO0 + reversal already seen, skipping FIRST_A\n");
                s->tx.tone_duration = 0;
                s->tx.stage = V34_TX_STAGE_FIRST_NOT_A_REVERSAL_SEEN;
            }
            else
            {
                s->tx.stage = V34_TX_STAGE_FIRST_A;
            }
        }
        /*endif*/
        break;
    case V34_TX_STAGE_FIRST_A:
        /* Continue sending pure tone until the peer's INFO0 has been received
           *and* its tone has actually been detected (V.34/11.2.1.2.3,
           V.90/9.2.2.1.3 -- identical wording: "After Tone B is detected and
           Tone A has been transmitted for at least 50 ms").  INITIAL_A served
           the 50 ms. */
        if (s->tx.phase2_reranging)
        {
            /* Re-ranging out of the §11.2.2.1.1 INFO0 recovery.  The call
               modem accepted our acknowledged INFO0a, stopped repeating
               INFO0c and is now transmitting Tone B waiting for our reversal,
               so waiting for another INFO0c here would deadlock both sides.
               §11.2.1.2.3 gates this on Tone B being detected and Tone A
               having run for 50 ms, both of which already hold: INITIAL_A
               served the 50 ms and the recovery handshake established that
               the peer is in Tone B. */
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Tx - FIRST_A: re-ranging after INFO0 recovery, sending !A without waiting for INFO0c\n");
            s->tx.phase2_reranging = false;
            s->tx.lastbit.re = -s->tx.lastbit.re;
            s->tx.tone_duration = 1;
            s->tx.stage = V34_TX_STAGE_FIRST_NOT_A;
        }
        else if (s->tx.v90_mode
                 &&
                 s->rx.info0_received
                 &&
                 s->rx.received_event == V34_EVENT_TONE_SEEN)
        {
            /* Both halves of the §9.2.2.1.3 condition now hold ("After Tone B
               is detected and Tone A has been transmitted for at least 50 ms";
               INITIAL_A served the 50 ms).  This used to fire on INFO0_OK
               alone, which is only the same thing when the peer's tone is
               already up by the time its INFO0 decodes.  In V.90 it never is:
               §9.2.1.1.1-9.2.1.1.2 have the digital modem start Tone B *after*
               it receives INFO0a, so our INFO0d decode precedes its Tone B by a
               whole INFO0a length.  Reversing there put the reversal in front
               of the digital modem's tone detector before it had 20 bauds of
               steady Tone A to measure against, so the reversal was swallowed
               and both sides waited on each other -- the analogue in
               FIRST_NOT_A, the digital in V90_PHASE2_B_INFO0_SEEN.

               V.90 only.  §11.2.1.2.3 says the same thing, but the V.34 answer
               modem cannot test it: process_rx_info0() parks its receiver in
               V34_RX_STAGE_TONE_B, whose detector never publishes TONE_SEEN
               (the assignment in v34rx.c is commented out), so requiring it
               there would deadlock.  V.34 keeps the INFO0_OK shortcut below,
               which is sound for it because the call modem's Tone B is already
               running by then. */
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Tx - FIRST_A: INFO0d received and Tone B detected, sending !A (9.2.2.1.3)\n");
            /* Transmit our first phase reversal */
            s->tx.lastbit.re = -s->tx.lastbit.re;
            s->tx.tone_duration = 1;
            s->tx.stage = V34_TX_STAGE_FIRST_NOT_A;
        }
        else if (!s->tx.v90_mode  &&  s->rx.received_event == V34_EVENT_INFO0_OK)
        {
            span_log(&s->logging, SPAN_LOG_FLOW, "Tx - FIRST_A: INFO0c received OK, sending !A (11.2.1.2.3)\n");
            /* First reversal seen - send a phase reversal back */
            s->tx.lastbit.re = -s->tx.lastbit.re;
            s->tx.tone_duration = 1;
            s->tx.stage = V34_TX_STAGE_FIRST_NOT_A;
        }
        else if (s->rx.received_event == V34_EVENT_REVERSAL_1
                 &&  s->rx.info0_received)
        {
            /* Reversal arrived after INFO0 was received but before we could
               check for INFO0_OK — the event was overwritten. */
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Tx - FIRST_A: reversal seen (INFO0 already received), sending !A\n");
            s->tx.lastbit.re = -s->tx.lastbit.re;
            s->tx.tone_duration = 0;
            s->tx.stage = V34_TX_STAGE_FIRST_NOT_A_REVERSAL_SEEN;
        }
        else if (s->rx.received_event == V34_EVENT_INFO0_BAD
                 ||
                 (s->rx.received_event == V34_EVENT_TONE_SEEN
                  &&  !s->rx.info0_received))
        {
            /* §11.2.2.2.1 / §9.2.2.2.1: the peer's tone detected *before* its
               INFO0 was correctly received.  Repeat INFO0a.  Tone detection
               after a good INFO0 is not this case -- it is the §9.2.2.1.3
               trigger above. */
            span_log(&s->logging, SPAN_LOG_FLOW, "Tx - FIRST_A: bad event %d, retrying INFO0a\n",
                     s->rx.received_event);
            /* Go back to sending INFO0a until we get a clean INFO0c */
            info0_baud_init(s);
        }
        /*endif*/
        break;
    case V34_TX_STAGE_FIRST_NOT_A:
        /* Send phase reversed pure tone until we see another phase reversal */
        if (s->rx.received_event == V34_EVENT_REVERSAL_1
            ||
            s->rx.tone_b_ended)
        {
            /* Second reversal seen - wait 40+=1ms.
               Tone B *ending* counts as well: 11.2.1.1.3 has the call modem
               answer our reversal with its own, hold Tone B 10 ms more and
               then transmit silence, so the carrier dropping is the same
               event seen from the other side.  On a call where the reversal
               itself was missed this is the only evidence of it, and without
               it the answer modem holds !A forever against a peer that has
               already gone quiet and is waiting for the probe -- measured
               against a SmartLink call modem after the 11.2.2.2.1 INFO0
               recovery. */
            if (s->rx.tone_b_ended  &&  s->rx.received_event != V34_EVENT_REVERSAL_1)
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - FIRST_NOT_A: Tone B ended without a detected "
                         "reversal; treating it as 11.2.1.1.3\n");
            /*endif*/
            s->rx.tone_b_ended = false;
            s->tx.tone_duration = 0;
            s->tx.stage = V34_TX_STAGE_FIRST_NOT_A_REVERSAL_SEEN;
        }
        /*endif*/
        break;
    case V34_TX_STAGE_POST_INFO0_RESUME_A:
        /* Tone A, held before resuming 11.2.1.2.5.  The call modem needs this
           time: it answers the acknowledged INFO0a by working through its own
           recovery, and only then reaches the state where it receives L1/L2.
           Measured against a SmartLink call modem, it got there about 80 ms
           after taking our INFO0a and then waited 2 s before retraining, so a
           short hold puts the probe inside that window; reversing immediately,
           as this used to, put the probe in front of it. */
        if (s->rx.tone_b_ended
            ||
            ++s->tx.tone_duration >= post_info0_resume_bauds())
        {
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Tx - Tone B %s after %d bauds of Tone A following the INFO0 "
                     "acknowledgement; sending the 11.2.1.2.3 Tone A reversal\n",
                     s->rx.tone_b_ended ? "ended" : "still up (timeout)",
                     s->tx.tone_duration);
            s->tx.tone_duration = 1;
            s->tx.lastbit.re = -s->tx.lastbit.re;
            s->tx.stage = V34_TX_STAGE_FIRST_NOT_A;
        }
        /*endif*/
        break;
    case V34_TX_STAGE_FIRST_NOT_A_REVERSAL_SEEN:
        /* Continue sending phase reversed pure tone for 40+-1ms */
        if (++s->tx.tone_duration == 24)
        {
            /* 40ms has passed - send another reversal back */
            s->tx.lastbit.re = -s->tx.lastbit.re;
            s->tx.tone_duration = 0;
            s->tx.stage = V34_TX_STAGE_SECOND_A;
        }
        /*endif*/
        break;
    case V34_TX_STAGE_SECOND_A:
        /* Send phase reversed pure tone for 10ms */
        if (++s->tx.tone_duration == 6)
        {
            if (s->tx.v90_mode  &&  !s->tx.calling_party)
            {
                /* V.90 §9.2.1.1.5: digital modem does NOT send L1/L2 here.
                   Instead, send Tone B and wait to RECEIVE analog's L1/L2.
                   The analogue modem takes the L1/L2 branch below: §9.2.2.1.5
                   has it transmit the probe right off its own 10 ms of Tone A,
                   exactly as the V.34 answer modem does in §11.2.1.2.5. */
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - V.90: sending Tone B, waiting for analog L1/L2\n");
                s->tx.tone_duration = 0;
                s->tx.stage = V34_TX_STAGE_V90_WAIT_RX_L2;
            }
            else
            {
                /* V.34: 10ms has passed - move on to sending L1/L2 */
                l1_l2_signal_init(s);
            }
        }
        /*endif*/
        break;
    case V34_TX_STAGE_V90_WAIT_RX_L2:
        /* V.90 §9.2.1.1.5: send Tone B while receiving analog's L1/L2.
           L1 lasts 160 ms and L2 may be received for at most another
           500 ms, so 400 bauds is a rounded 667 ms recovery guard. */
        if (v90_phase2_l2_pending(s)
            ||
            ++s->tx.tone_duration >= 400)
        {
            bool l2_received;

            l2_received = v90_phase2_l2_pending(s);
            if (l2_received)
                v90_phase2_consume_l2(s);
            /*endif*/
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Tx - V.90: analog L1/L2 %s after %d bauds, waiting for Tone A reversal\n",
                     l2_received ? "received" : "timeout",
                     s->tx.tone_duration);
            s->tx.tone_duration = 0;
            s->tx.stage = V34_TX_STAGE_V90_WAIT_TONE_A_REV;
            /* Clear stale L2_SEEN so Tone A detection can set TONE_SEEN/REVERSAL_1 */
            s->rx.received_event = V34_EVENT_NONE;
            s->rx.persistence1 = 0;
            s->rx.persistence2 = 0;
        }
        /*endif*/
        break;
    case V34_TX_STAGE_V90_WAIT_TONE_A_REV:
        /* V.90 §9.2.1.1.6: send Tone B, wait for Tone A phase REVERSAL from analog.
           Analog sends Tone A 50ms + reversal + 10ms + silence (§9.2.2.1.6).
           TONE_SEEN means Tone A is present (not a reversal) — must wait for REVERSAL_1. */
        if (!v90_phase2_reversal_pending(s)
            &&
            (s->rx.v90_repeated_info0a_pending
             ||  s->rx.received_event == V34_EVENT_INFO0_OK))
        {
            int recoveries;

            recoveries = v90_note_phase2_info0_recovery(s);
            if (recoveries <= 2)
            {
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - V.90: repeated INFO0a while waiting for Tone A reversal after analog L1/L2; treating it as stale and staying on Tone B (recovery %d)\n",
                         recoveries);
                s->rx.v90_repeated_info0a_pending = false;
                s->rx.received_event = V34_EVENT_NONE;
                s->rx.persistence1 = 0;
                s->rx.persistence2 = 0;
            }
            else
            {
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - V.90: repeated INFO0a persists while waiting for Tone A reversal after %d stale repeats; forcing B reversal path instead of restarting INFO0d\n",
                         recoveries);
                s->rx.v90_repeated_info0a_pending = false;
                s->rx.received_event = V34_EVENT_NONE;
                s->rx.persistence1 = 0;
                s->rx.persistence2 = 0;
                s->tx.tone_duration = 0;
                s->tx.stage = V34_TX_STAGE_V90_B_REV_DELAY;
            }
            break;
        }
        /*endif*/
        if (s->rx.received_event == V34_EVENT_INFO0_BAD)
        {
            /* Some peers keep leaking INFO0a decodes into the later Tone A
               window. At this point the L1/L2 exchange is already complete, so
               treat them as stale and continue waiting for the Tone A reversal. */
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Tx - V.90: ignoring stale %s while waiting for Tone A reversal\n",
                     "bad INFO0a");
            s->rx.received_event = V34_EVENT_NONE;
        }
        /*endif*/
        ++s->tx.tone_duration;
        if (v90_phase2_reversal_pending(s)
            ||
            v90_tone_a_reversal_recovery_due(s))
        {
            bool reversal_received;

            reversal_received = v90_phase2_reversal_pending(s);
            if (reversal_received)
                v90_phase2_consume_reversal(s);
            /*endif*/
            /* V.90 9.2.1.1.6: the B reversal must be timed from the RECEIVED
               Tone A reversal edge.  The analogue modem sends only 50 ms of
               Tone A before reversing (9.2.2.1.6), and it arms its own Tone B
               reversal detector only after its reversal completes and it goes
               silent.  Committing early on mere tone presence used to place
               our B reversal inside the peer's Tone A window, where its
               detector was not yet listening - it then retrained instead of
               sending INFO1a.  Tone presence alone must never trigger this
               transition; the absolute 900 ms + RTD deadline is the
               9.2.1.2.4 recovery. */
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Tx - V.90: Tone A %s (event=%d) after %d bauds, delaying 40ms for B reversal\n",
                     reversal_received ? "reversal transaction completed"
                     : "reversal timeout (9.2.1.2.4 recovery)",
                     s->rx.received_event, s->tx.tone_duration);
            s->rx.received_event = V34_EVENT_NONE;
            s->tx.tone_duration = 0;
            s->tx.stage = V34_TX_STAGE_V90_B_REV_DELAY;
        }
        else if (s->rx.received_event == V34_EVENT_TONE_SEEN)
        {
            /* Tone A present - the reversal is still to come; keep waiting */
            s->rx.received_event = V34_EVENT_NONE;
        }
        /*endif*/
        break;
    case V34_TX_STAGE_V90_B_REV_DELAY:
        /* V.90 §9.2.1.1.6: delay 40ms before Tone B reversal */
        if (++s->tx.tone_duration == 24)
        {
            /* Send Tone B phase reversal */
            s->tx.lastbit.re = -s->tx.lastbit.re;
            s->tx.tone_duration = 0;
            s->tx.stage = V34_TX_STAGE_V90_B_REV_10MS;
        }
        /*endif*/
        break;
    case V34_TX_STAGE_V90_B_REV_10MS:
        /* V.90 §9.2.1.1.6: send Tone B for 10ms after reversal, then L1/L2 */
        if (++s->tx.tone_duration == 6)
        {
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Tx - V.90: sending digital L1/L2\n");
            l1_l2_signal_init(s);
        }
        /*endif*/
        break;
    }
    /*endswitch*/
    return s->tx.lastbit;
}
/*- End of function --------------------------------------------------------*/

static complex_sig_t get_initial_fdx_b_not_b_baud(v34_state_t *s)
{
    /* Calling side */
    switch (s->tx.stage)
    {
    case V34_TX_STAGE_V90_PHASE2_B:
        /* V.90 digital answerer: Tone B window after INFO0d (§9.2.1.1.1). */
        if (++s->tx.tone_duration % 600 == 0)
        {
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Tx - V.90 PHASE2_B: baud=%d rx_event=%d rx_stage=%d sig=%d pers1=%d pers2=%d demod=%d\n",
                     s->tx.tone_duration,
                     s->rx.received_event, s->rx.stage,
                     s->rx.signal_present, s->rx.persistence1,
                     s->rx.persistence2, s->rx.current_demodulator);
        }
        if (s->rx.info0_received)
        {
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Tx - V.90: INFO0a received, continuing Tone B and waiting for Tone A reversal\n");
            s->tx.stage = V34_TX_STAGE_V90_PHASE2_B_INFO0_SEEN;
            s->rx.received_event = V34_EVENT_NONE;
        }
        else if (v90_phase2_reversal_pending(s))
        {
            /* Some peers assert Tone A and its first reversal before we manage
               to decode a clean INFO0a frame. Treat the observed reversal as
               sufficient Phase 2 progress and continue instead of looping on
               INFO0d retries forever. */
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Tx - V.90: Tone A reversal arrived before clean INFO0a, continuing Phase 2 using reversal fallback\n");
            v90_phase2_consume_reversal(s);
            s->tx.tone_duration = 1;
            s->tx.stage = V34_TX_STAGE_FIRST_NOT_B_WAIT;
        }
        else if (s->rx.received_event == V34_EVENT_TONE_SEEN)
        {
            /* A number of peers assert Tone A early; keep Tone B running long
               enough for INFO0a or the first reversal to settle instead of
               immediately dropping back into INFO0d retries. */
            if (s->tx.tone_duration == 1 || s->tx.tone_duration % 120 == 0)
            {
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - V.90: Tone A detected before clean INFO0a, holding Tone B and waiting for INFO0a or reversal\n");
            }
        }
        else if (s->rx.received_event == V34_EVENT_INFO0_BAD)
        {
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Tx - V.90: bad INFO0a during initial Tone B, repeating INFO0d\n");
            info0_baud_init(s);
        }
        break;
    case V34_TX_STAGE_FIRST_B:
        /* Send pure tone (V.34 answerer side). */
        if (s->rx.received_event == V34_EVENT_INFO0_OK)
        {
            s->tx.stage = V34_TX_STAGE_FIRST_B_INFO_SEEN;
        }
        else if (s->rx.received_event == V34_EVENT_INFO0_BAD
                 ||
                 s->rx.received_event == V34_EVENT_TONE_SEEN)
        {
            /* Go back to sending INFO0c until we get a clean INFO0a */
            info0_baud_init(s);
        }
        /*endif*/
        break;
    case V34_TX_STAGE_V90_PHASE2_B_INFO0_SEEN:
        /* A completed Tone A reversal commits the peer to 9.2.2.1.3.  Consume
           that transaction before considering a concurrently decoded INFO0a
           repeat; otherwise the repeat regresses both modems to INFO0 recovery
           after ranging has already advanced (the source of the multi-second
           live Phase 2 loop). */
        if (v90_phase2_reversal_pending(s))
        {
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Tx - V.90: first Tone A reversal transaction completed; scheduling Tone B reversal\n");
            v90_phase2_consume_reversal(s);
            s->rx.v90_repeated_info0a_pending = false;
            s->rx.received_event = V34_EVENT_NONE;
            s->tx.tone_duration = 1;
            s->tx.stage = V34_TX_STAGE_FIRST_NOT_B_WAIT;
        }
        else if (s->rx.received_event == V34_EVENT_INFO0_OK
                 || s->rx.v90_repeated_info0a_pending)
        {
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Tx - V.90: repeated INFO0a during Tone B, repeating INFO0d with acknowledgement\n");
            s->tx.info0_acknowledgement = true;
            info0_baud_init(s);
            s->rx.received_event = V34_EVENT_NONE;
            s->rx.v90_repeated_info0a_pending = false;
        }
        /*endif*/
        break;
    case V34_TX_STAGE_FIRST_B_INFO_SEEN:
        if (s->rx.received_event == V34_EVENT_REVERSAL_1)
        {
            /* First reversal seen - continue sending pure tone for 40+-1ms */
            s->tx.tone_duration = 1;
            s->tx.stage = V34_TX_STAGE_FIRST_NOT_B_WAIT;
        }
        /*endif*/
        break;
    case V34_TX_STAGE_FIRST_NOT_B_WAIT:
        /* Continue sending pure tone for 40+-1ms (V.34/11.2.1.1.3) */
        if (++s->tx.tone_duration == 24)
        {
            /* 40ms has passed - send a phase reversal back */
            s->tx.lastbit.re = -s->tx.lastbit.re;
            s->tx.tone_duration = 1;
            s->tx.stage = V34_TX_STAGE_FIRST_NOT_B;
        }
        /*endif*/
        break;
    case V34_TX_STAGE_FIRST_NOT_B:
        /* Send phase reversed pure tone for 10ms (V.34/11.2.1.1.3) */
        if (++s->tx.tone_duration == 6)
        {
            /* 10ms has passed */
            /* Move on to sending silence */
            s->tx.tone_duration = 0;
            s->tx.stage = V34_TX_STAGE_FIRST_B_SILENCE;
            if (s->tx.v90_mode)
            {
                /* V.90: clear stale REVERSAL_1 from first exchange so we wait
                   for the actual second Tone A reversal. Keep persistence intact
                   so we don't miss a reversal that arrives before re-detecting Tone A. */
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - V.90: entering FIRST_B_SILENCE, clearing event (was %d) for second A reversal\n",
                         s->rx.received_event);
                s->rx.v90_repeated_info0a_pending = false;
                s->rx.received_event = V34_EVENT_NONE;
            }
        }
        /*endif*/
        break;
    case V34_TX_STAGE_FIRST_B_SILENCE:
        /* Send silence, as we wait for reversal (V.34/11.2.1.1.4) */
        if (s->tx.v90_mode && v90_phase2_reversal_pending(s))
        {
            /* The second Tone A reversal transaction may be reported as any
               event ordinal, or the event may already have been overwritten
               by the immediately following L1.  The durable counter is the
               authority. */
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Tx - V.90: second Tone A reversal transaction completed; receiving analogue L1/L2\n");
            v90_phase2_consume_reversal(s);
            s->rx.received_event = V34_EVENT_NONE;
            s->tx.tone_duration = 1;
            s->tx.stage = V34_TX_STAGE_FIRST_B_POST_REVERSAL_SILENCE;
        }
        else if (s->rx.received_event == V34_EVENT_REVERSAL_1)
        {
            /* Second reversal seen. We now have the round trip timed */
            s->tx.tone_duration = 1;
            s->tx.stage = V34_TX_STAGE_FIRST_B_POST_REVERSAL_SILENCE;
        }
        else if (s->tx.v90_mode
                 && s->rx.v90_repeated_info0a_pending)
        {
            int recoveries;

            recoveries = v90_note_phase2_info0_recovery(s);
            if (recoveries <= 2)
            {
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - V.90: repeated INFO0a while waiting for second Tone A reversal; re-sending acknowledged INFO0d to recover Phase 2 (recovery %d)\n",
                         recoveries);
                s->tx.info0_acknowledgement = true;
                s->tx.info0_retry_count = 0;
                s->tx.tone_duration = 0;
                s->rx.v90_repeated_info0a_pending = false;
                s->rx.received_event = V34_EVENT_NONE;
                v90_phase2_reset_transactions(s);
                info0_baud_init(s);
            }
            else
            {
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - V.90: repeated INFO0a persists while waiting for second Tone A reversal after %d recoveries; forcing Tone B/L1/L2 path instead of another INFO0d loop\n",
                         recoveries);
                s->rx.v90_repeated_info0a_pending = false;
                s->rx.received_event = V34_EVENT_NONE;
                s->rx.persistence1 = 0;
                s->rx.persistence2 = 0;
                v90_wait_rx_l2_init(s, "repeated INFO0a recovery cap reached");
            }
        }
        else if (s->tx.v90_mode
                 &&
                 (s->rx.received_event == V34_EVENT_INFO0_OK
                  ||
                  s->rx.received_event == V34_EVENT_INFO0_BAD))
        {
            /* V.90: the analog modem's Tone A carries INFO0a data.  The info_rx
               demodulator may decode another INFO0a frame and overwrite the event
               before the reversal detector can set REVERSAL_1.  Consume the stale
               INFO0 event so the next reversal can be recorded. */
            s->rx.received_event = V34_EVENT_NONE;
        }
        else if (s->tx.v90_mode
                 &&
                 (v90_phase2_l2_pending(s)
                  ||
                  s->rx.received_event == V34_EVENT_L2_SEEN
                  ||
                  ++s->tx.tone_duration >= 240))
        {
            /* V.90 §9.2.2.1.5: the analogue modem's second Tone A reversal is
               10 ms of tone and then L1 begins immediately.  The Phase 2
               reversal counter numbers reversals across the whole phase, so
               here the second reversal arrives as REVERSAL_2, not the
               REVERSAL_1 the branch above tests -- or the 10 ms sliver is
               missed entirely and the first evidence is L2_SEEN.  On the
               initial call the repeated-INFO0a recovery above masks this and
               eventually forces the same landing state, but a §9.5-initiated
               retrain carries no INFO0a at all, so without these exits the
               state dead-ends while the peer waits out L2 for our Tone B and
               drops the link (observed live 2026-07-22, SmartLink Link Error
               0.96 s into L1).  §9.2.1.1.4/.5 want us receiving L1/L2 and
               answering with Tone B; v90_wait_rx_l2_init() is exactly that.
               The 400 ms cap (240 bauds) matters most on the §9.5 retrain
               path, where no INFO0a traffic conditions the receiver for
               L1/L2, the peer sends no second reversal at all, and its
               post-L1 Tone B wait (DET_AB) aborts ~1.25 s in — a fresh
               Tone B onset must appear within that window.  The proven
               initial-call FIRST_B_SILENCE dwell is ~353 ms. */
            const char *reason;

            if (v90_phase2_l2_pending(s))
            {
                v90_phase2_consume_l2(s);
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - V.90: analogue L1/L2 transaction completed after a missed second reversal; transmitting Tone B and waiting for the next Tone A reversal\n");
                s->tx.lastbit = complex_sig_set(TRAINING_SCALE(TRAINING_AMP), TRAINING_SCALE(0.0f));
                s->tx.tone_duration = 0;
                /* V90_WAIT_TONE_A_REV is a case in get_initial_fdx_a_not_a_baud,
                   not in this function.  Setting the stage without also moving
                   the getbaud left the digital modem falling off the end of this
                   switch every baud, holding Tone B for the rest of the call. */
                s->tx.current_getbaud = get_initial_fdx_a_not_a_baud;
                s->tx.stage = V34_TX_STAGE_V90_WAIT_TONE_A_REV;
                s->rx.received_event = V34_EVENT_NONE;
                s->rx.persistence1 = 0;
                s->rx.persistence2 = 0;
                break;
            }
            else if (s->rx.received_event == V34_EVENT_L2_SEEN)
                reason = "L1/L2 arriving while waiting for the second Tone A reversal";
            else
                reason = "second Tone A reversal timeout";
            v90_wait_rx_l2_init(s, reason);
        }
        else if (s->tx.tone_duration == (1200 - 30))
        {
            /* Timeout, as we have not received a round trip time indication after 2s */
        }
        /*endif*/
        return zero;
    case V34_TX_STAGE_FIRST_B_POST_REVERSAL_SILENCE:
        /* Send silence, as we wait for L2 (V.34/11.2.1.1.4).  Only the Tone B
           transmitter reaches here -- the V.34 call modem and the V.90 digital
           modem.  The V.90 analogue modem used to be routed through this whole
           B-family chain and needed a special case here to get its §9.2.2.1.5
           probe out; it now runs the A family, where L1/L2 follows its own
           second reversal directly. */
        if ((s->tx.v90_mode && v90_phase2_l2_pending(s))
            ||
            s->rx.received_event == V34_EVENT_L2_SEEN
            ||
            ++s->tx.tone_duration >= 400)
        {
            if (s->tx.v90_mode && v90_phase2_l2_pending(s))
                v90_phase2_consume_l2(s);
            /*endif*/
            /* L2 recognised */
            s->tx.lastbit.re = -s->tx.lastbit.re;
            s->tx.tone_duration = 1;
            s->tx.stage = V34_TX_STAGE_SECOND_B;
            if (s->tx.v90_mode)
            {
                /* V.90 answerer: clear stale L2_SEEN so Tone A reversal detection starts fresh */
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - V.90: entering SECOND_B, clearing L2_SEEN for fresh Tone A reversal detection\n");
                s->rx.received_event = V34_EVENT_NONE;
                s->rx.persistence1 = 0;
                s->rx.persistence2 = 0;
                s->rx.current_demodulator = V34_MODULATION_TONES;
                s->rx.stage = V34_RX_STAGE_TONE_A;
            }
        }
        /*endif*/
        return zero;
    case V34_TX_STAGE_SECOND_B:
        /* Send pure tone (V.34/11.2.1.1.5, V.90/9.2.1.1.6) */
        ++s->tx.tone_duration;
        if (s->tx.v90_mode)
        {
            /* V.90 §9.2.1.1.6: wait for Tone A phase reversal, then delay 40ms.
               Normal progress is the next durable reversal transaction. */
            if (v90_phase2_reversal_pending(s))
            {
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - V.90: Tone A reversal transaction completed at SECOND_B after %d bauds (event=%d), delaying 40ms\n",
                         s->tx.tone_duration, s->rx.received_event);
                v90_phase2_consume_reversal(s);
                s->tx.tone_duration = 1;
                s->tx.stage = V34_TX_STAGE_SECOND_B_WAIT;
            }
            else if (v90_tone_a_reversal_recovery_due(s))
            {
                /* V.90 §9.2.1.2.4 recovery: the deadline is absolute from
                   the second received Tone A reversal, not a seven-second
                   local stage timer. */
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - V.90: Tone A reversal recovery deadline reached at SECOND_B after %d bauds\n",
                         s->tx.tone_duration);
                s->tx.tone_duration = 1;
                s->tx.stage = V34_TX_STAGE_SECOND_B_WAIT;
            }
        }
        else if (s->tx.tone_duration >= 100)
        {
            /* V.34: fixed timing */
            s->tx.tone_duration = 1;
            s->tx.stage = V34_TX_STAGE_SECOND_B_WAIT;
        }
        /*endif*/
        break;
    case V34_TX_STAGE_SECOND_B_WAIT:
        /* Continue sending pure tone for 40+-1ms (V.34/11.2.1.1.6) */
        if (++s->tx.tone_duration == 24)
        {
            /* 40ms has passed - send a phase reversal back */
            s->tx.lastbit.re = -s->tx.lastbit.re;
            s->tx.tone_duration = 1;
            s->tx.stage = V34_TX_STAGE_SECOND_NOT_B;
        }
        /*endif*/
        break;
    case V34_TX_STAGE_SECOND_NOT_B:
        /* Send phase reversed pure tone for 10ms (V.34/11.2.1.1.6) */
        if (++s->tx.tone_duration == 6)
        {
            /* 10ms has passed - move on to sending L1/L2 */
            s->tx.tone_duration = 0;
            l1_l2_signal_init(s);
        }
        /*endif*/
        break;
    }
    /*endswitch*/
    return s->tx.lastbit;
}
/*- End of function --------------------------------------------------------*/

static complex_sig_t get_initial_hdx_a_not_a_baud(v34_state_t *s)
{
    /* Answering side */
    switch (s->tx.stage)
    {
    case V34_TX_STAGE_HDX_INITIAL_A:
        /* Send pure tone (V.34/12.2.1.2.1) */
        if (++s->tx.tone_duration == 30)
        {
            /* 50ms minimum A period has passed - accept an incoming INFO0c */
            s->tx.stage = V34_TX_STAGE_HDX_FIRST_A;
        }
        /*endif*/
        break;
    case V34_TX_STAGE_HDX_FIRST_A:
        /* Continue sending pure tone until we see an INFO0c message (V.34/12.2.1.2.3) */
        if (s->rx.received_event == V34_EVENT_INFO0_OK)
        {
            /* First reversal seen - send a phase reversal back */
            s->tx.lastbit.re = -s->tx.lastbit.re;
            s->tx.tone_duration = 1;
            s->tx.stage = V34_TX_STAGE_HDX_FIRST_NOT_A;
        }
        else if (s->rx.received_event == V34_EVENT_INFO0_BAD
                 ||
                 s->rx.received_event == V34_EVENT_TONE_SEEN)
        {
            /* Go back to sending INFO0a until we get a clean INFO0c */
            info0_baud_init(s);
        }
        /*endif*/
        break;
    case V34_TX_STAGE_HDX_FIRST_NOT_A:
        /* Send phase reversed pure tone for 10ms (V.34/12.2.1.2.3) */
        if (++s->tx.tone_duration == 6)
        {
            /* 10ms has passed - send silence */
            s->tx.tone_duration = 0;
            s->tx.stage = V34_TX_STAGE_HDX_FIRST_A_SILENCE;
        }
        /*endif*/
        break;
    case V34_TX_STAGE_HDX_FIRST_A_SILENCE:
        /* Send silence, as we wait for L2 (V.34/12.2.1.2.3) */
        if (s->rx.received_event == V34_EVENT_L2_SEEN
            ||
            ++s->tx.tone_duration >= 400)
        {
            /* L2 recognised */
            s->tx.lastbit.re = -s->tx.lastbit.re;
            s->tx.tone_duration = 1;
            s->tx.stage = V34_TX_STAGE_HDX_SECOND_A;
        }
        /*endif*/
        return zero;
    case V34_TX_STAGE_HDX_SECOND_A:
        /* Send pure tone (V.34/12.2.1.2.5) */
        if (++s->tx.tone_duration >= 100)
        //if (s->rx.received_event == V34_EVENT_REVERSAL_2)
        {
            /* Second reversal seen - continue sending pure tone for 25ms */
            s->tx.lastbit.re = -s->tx.lastbit.re;
            s->tx.tone_duration = 1;
            s->tx.stage = V34_TX_STAGE_HDX_SECOND_A_WAIT;
        }
        /*endif*/
        break;
    case V34_TX_STAGE_HDX_SECOND_A_WAIT:
        /* Continue sending pure tone for 25ms (V.34/12.2.1.2.6) */
        if (++s->tx.tone_duration == 15)
        {
            /* 25ms has passed - send INFOh */
            s->tx.tone_duration = 0;
            infoh_baud_init(s);
        }
        /*endif*/
        break;
    }
    /*endswitch*/
    return s->tx.lastbit;
}
/*- End of function --------------------------------------------------------*/

static complex_sig_t get_initial_hdx_b_not_b_baud(v34_state_t *s)
{
    /* Calling side */
    switch (s->tx.stage)
    {
    case V34_TX_STAGE_HDX_FIRST_B:
        /* Send pure tone (V.34/12.2.1.1.1) */
        if (s->rx.received_event == V34_EVENT_INFO0_OK)
        {
            s->tx.stage = V34_TX_STAGE_HDX_FIRST_B_INFO_SEEN;
        }
        else if (s->rx.received_event == V34_EVENT_INFO0_BAD
                 ||
                 s->rx.received_event == V34_EVENT_TONE_SEEN)
        {
            /* Go back to sending INFO0c until we get a clean INFO0a */
            info0_baud_init(s);
        }
        /*endif*/
        break;
    case V34_TX_STAGE_HDX_FIRST_B_INFO_SEEN:
        /* Continue sending pure tone (V.34/12.2.1.1.1) */
        if (s->rx.received_event == V34_EVENT_REVERSAL_1)
        {
            /* First reversal seen - continue sending pure tone for 40+-1ms */
            s->tx.tone_duration = 1;
            s->tx.stage = V34_TX_STAGE_HDX_FIRST_NOT_B_WAIT;
        }
        /*endif*/
        break;
    case V34_TX_STAGE_HDX_FIRST_NOT_B_WAIT:
        /* Continue sending pure tone for 40+-10ms (V.34/12.2.1.1.3) */
        if (++s->tx.tone_duration == 24)
        {
            /* 40ms has passed - send a phase reversal back */
            s->tx.lastbit.re = -s->tx.lastbit.re;
            s->tx.tone_duration = 1;
            s->tx.stage = V34_TX_STAGE_HDX_FIRST_NOT_B;
        }
        /*endif*/
        break;
    case V34_TX_STAGE_HDX_FIRST_NOT_B:
        /* Send phase reversed pure tone for 10ms (V.34/12.2.1.1.3) */
        if (++s->tx.tone_duration == 6)
        {
            /* 10ms has passed */
            /* Move on to sending L1/L2 */
            s->tx.tone_duration = 0;
            l1_l2_signal_init(s);
        }
        /*endif*/
        break;
    }
    /*endswitch*/
    return s->tx.lastbit;
}
/*- End of function --------------------------------------------------------*/

static void initial_ab_not_ab_baud_init(v34_state_t *s)
{
    span_log(&s->logging, SPAN_LOG_FLOW, "Tx - initial_ab_not_ab_baud_init() [calling=%d duplex=%d sample_time=%d]\n",
             s->tx.calling_party, s->tx.duplex, s->tx.sample_time);
    s->tx.tone_duration = 0;
    s->tx.current_modulator = V34_MODULATION_CC;
    s->tx.lastbit = complex_sig_set(TRAINING_SCALE(TRAINING_AMP), TRAINING_SCALE(0.0f));
    if (s->tx.duplex)
    {
        /* Which tone this modem transmits follows the role, not the call
           direction.  V.34 gives Tone A to the answer modem (§11.2.1.2) and
           Tone B to the call modem (§11.2.1.1).  V.90 keeps both tones and
           both timetables but hands them to the other end of the call: the
           analogue modem is the *calling* party and runs §9.2.2.1, which is
           §11.2.1.2 word for word with INFO0d/INFO1d in place of
           INFO0c/INFO1c; the digital modem answers and runs §9.2.1.1, which
           is §11.2.1.1.  So the Tone A transmitter is the side where
           calling_party and v90_mode agree. */
        if (s->tx.calling_party  &&  !s->tx.v90_mode)
        {
            /* V.34 call modem (§11.2.1.1): Tone B. */
            s->tx.current_getbaud = get_initial_fdx_b_not_b_baud;
            s->tx.stage = V34_TX_STAGE_FIRST_B;
        }
        else if (!s->tx.calling_party  &&  s->tx.v90_mode)
        {
            /* V.90 9.2.1.1.1: the digital modem follows INFO0d with Tone B,
               not the V.34 answerer A/!A sequence. It must also condition
               its receiver to receive INFO0a and detect Tone A in this
               window, so use the Tone A RX stage while keeping INFO0
               target_bits active. */
            s->tx.current_getbaud = get_initial_fdx_b_not_b_baud;
            s->tx.stage = V34_TX_STAGE_V90_PHASE2_B;
            s->tx.tone_duration = 0;
            v90_prime_info0a_tone_a_rx(s, "after INFO0d");
        }
        else
        {
            /* Tone A: the V.34 answer modem (§11.2.1.2.1) and the V.90
               analogue modem (§9.2.2.1.1), which run the same timetable. */
            s->tx.current_getbaud = get_initial_fdx_a_not_a_baud;
            s->tx.stage = V34_TX_STAGE_INITIAL_A;
            if (s->rx.stage == V34_RX_STAGE_INFO1C  &&  !s->tx.v90_mode
                &&
                s->tx.phase2_probe_sent)
            {
                /* 11.2.2.2.1 sends the answer modem back to 11.2.1.2.3, which
                   waits for Tone B and then reverses Tone A.  A call modem
                   that has already completed 11.2.1.1.3 does not raise Tone B
                   again -- it has transmitted its Tone B reversal, gone
                   silent, and is waiting for the second Tone A reversal and
                   the L1/L2 probe of 11.2.1.2.5.  Measured against a
                   SmartLink call modem: after acknowledging the repeated
                   INFO0c the peer sat in its receive-probe state, transmitting
                   nothing, while we held Tone A for a Tone B reversal that was
                   never coming, until it retrained and cleared the call.  The
                   same peer's own logs show this recovery is meant to be
                   survivable -- in the answer role it runs the identical
                   "errorrecovery is initialized in TX_PHASE2" branch and then
                   goes straight on to transmit L1/L2.  So resume at 11.2.1.2.5
                   instead: 40 ms of Tone A, the phase reversal, 10 ms more,
                   and the probe again.  Before the probe has ever gone out
                   there is nothing to resume and the 11.2.1.2.3 path below
                   still applies. */
                answer_resume_probe(s, "re-ranging after INFO0 recovery");
                return;
            }
            /*endif*/
            if (s->rx.stage == V34_RX_STAGE_INFO1C  &&  !s->tx.v90_mode)
            {
                /* Re-entering the ranging sequence from the §11.2.2.1.1
                   INFO0 recovery.  The receiver is still conditioned for
                   INFO1c, so it would never report the Tone B reversal
                   that FIRST_NOT_A waits on and we would hold Tone A
                   forever.  §11.2.1.2.2: condition the receiver to detect
                   Tone B and receive INFO0c.  Restricted to the recovery
                   case so the ordinary first pass, where the receiver is
                   already conditioned coming out of INFO0, is untouched.
                   The V.90 analogue modem parks its receiver in INFO1C for
                   INFO1d as an ordinary part of §9.2.2.1.8, so this guard
                   would misfire there; its own re-ranging comes through
                   §9.2.2.2.x instead. */
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - re-ranging after INFO0 recovery; conditioning RX for Tone B + INFO0 (11.2.1.2.2)\n");
                s->rx.stage = V34_RX_STAGE_TONE_B;
                s->rx.target_bits = (s->rx.duplex)  ?  (49 - (4 + 8 + 4))  :  (51 - (4 + 8 + 4));
                s->rx.bit_count = 0;
                s->rx.persistence1 = 0;
                s->rx.persistence2 = 0;
                s->rx.received_event = V34_EVENT_NONE;
                s->tx.phase2_reranging = true;
            }
            /*endif*/
        }
        /*endif*/
    }
    else
    {
        if (s->tx.calling_party)
        {
            s->tx.current_getbaud = get_initial_hdx_b_not_b_baud;
            s->tx.stage = V34_TX_STAGE_HDX_FIRST_B;
        }
        else
        {
            s->tx.current_getbaud = get_initial_hdx_a_not_a_baud;
            s->tx.stage = V34_TX_STAGE_HDX_INITIAL_A;
        }
        /*endif*/
    }
    /*endif*/
    s->tx.persistence2 = 0;
}
/*- End of function --------------------------------------------------------*/

static int tx_l1_l2(v34_state_t *s, int16_t amp[], int max_len)
{
    int sample;

    /* This signal repeats every 160 samples, so we have the appropriate
       pattern stored, and we just scale and repeat it. We start 6dB above nominal
       power (L1) and then drop the amplitude to nominal power after the first 160ms
       (8 cycles) (L2). L2 should not last longer than 550ms + a round trip time. */
    /* This can occur between:
            !B and INFO1c for a FDX caller
            !B and B for a HDX caller
            A and A for a FDX answerer
            !A and A for a HDX answerer
     */
    for (sample = 0;  sample < max_len;  sample++)
    {
        amp[sample] = (int16_t) lfastrintf(line_probe_samples[s->tx.line_probe_step]*s->tx.line_probe_scaling);
        if (++s->tx.line_probe_step >= LINE_PROBE_SAMPLES)
        {
            s->tx.line_probe_step = 0;
            if (++s->tx.line_probe_cycles == 8)
            {
                /* Move to the L2 stage, by dropping 6dB. No stage is recorded:
                   s->tx.state is the trellis encoder state, not a pipeline
                   stage -- see the note in l1_l2_signal_init(). */
                s->tx.line_probe_scaling *= 0.5f;
            }
            else if (s->tx.line_probe_cycles == (8 + 20))
            {
                /* End of line probe sequence */
                if (s->tx.duplex)
                {
                    if (s->tx.calling_party  &&  s->tx.v90_mode)
                    {
                        /* V.90 §9.2.2.1.6: the *analogue* modem is the calling
                           party, and its own probe is only the first of the
                           two rounds.  Off the end of L2 it owes the digital
                           modem 50 ms of Tone A, a reversal, 10 ms more and
                           then silence, so that §9.2.1.1.6 can time its Tone B
                           reversal against it; only after that reversal does it
                           receive the digital modem's L1/L2 (§9.2.2.1.7) and
                           finally hold Tone A for INFO1d (§9.2.2.1.8).  That is
                           second_a_baud_init()'s POST_L2_A -> A_SILENCE ->
                           PRE_INFO1_A chain, which the V.34 answer modem
                           already walks for §11.2.1.2.6-8.  Jumping straight to
                           PRE_INFO1_A here skipped the whole second round: the
                           digital modem sat in SECOND_B waiting for a Tone A
                           reversal that never came, and never sent INFO1d.
                           Note INFO1a must not go out before INFO1d either
                           (Table 11: bit 25 selects the carrier and bits 26:29
                           the pre-emphasis for the digital-to-analogue
                           direction, both answers to what INFO1d offered), so
                           the V.34 call-modem branch below is wrong twice
                           over. */
                        span_log(&s->logging, SPAN_LOG_FLOW,
                                 "Tx - V.90 analogue modem: L2 done, Tone A + reversal + silence (9.2.2.1.6)\n");
                        second_a_baud_init(s);
                    }
                    else if (s->tx.calling_party)
                        info1_baud_init(s);
                    else if (s->tx.v90_mode)
                    {
                        /* V.90 §9.2.1.1.7: Tone A normally arrives while the
                           digital modem is still transmitting L1/L2.  Preserve
                           that indication and start INFO1d immediately at the
                           L2 boundary.  Inserting a fresh Tone A guard here
                           creates a non-standard carrier gap which causes
                           strict analogue modems to lose the INFO1d sync. */
                        if (s->rx.received_event == V34_EVENT_TONE_SEEN
                            || s->rx.received_event == V34_EVENT_REVERSAL_1
                            || s->rx.received_event == V34_EVENT_REVERSAL_3
                            || s->rx.signal_present)
                        {
                            span_log(&s->logging, SPAN_LOG_FLOW,
                                     "Tx - V.90: Tone A already present at end of L2; sending INFO1d without a carrier gap\n");
                            info1_baud_init(s);
                        }
                        else
                        {
                            v90_wait_tone_a_init(s, false);
                        }
                    }
                    else
                    {
                        /* Plain V.34 answer modem, 11.2.1.2.6: the Tone A
                           phase reversal owed here is conditional on Tone B
                           having been detected, and the call modem raises
                           Tone B only after it has received this L1/L2
                           (11.2.1.1.5).  Hold Tone A until then. */
                        post_l2_wait_tone_b_init(s);
                    }
                    /*endif*/
                }
                else
                {
                    if (s->tx.calling_party)
                        second_b_baud_init(s);
                    else
                        second_a_baud_init(s);
                    /*endif*/
                }
                /*endif*/
                break;
            }
            /*endif*/
        }
        /*endif*/
    }
    /*endfor*/
    return sample;
}
/*- End of function --------------------------------------------------------*/

static __inline__ int16_t pcm_phase2_quantise(v34_state_t *s, int16_t linear)
{
    /* V.90/V.91/V.92 downstream signalling must ultimately be conveyed as exact
       G.711 codewords.  Quantise the generated Phase 2 waveform through the
       selected PCM law now so this modulator has a dedicated PCM-oriented
       transmit identity instead of borrowing the generic V.34 analogue path. */
    if (s->tx.v90_pcm_law)
        return alaw_to_linear(linear_to_alaw(linear));
    return ulaw_to_linear(linear_to_ulaw(linear));
}
/*- End of function --------------------------------------------------------*/

static int tx_pcm_l1_l2(v34_state_t *s, int16_t amp[], int max_len)
{
    int sample;

    /* Dedicated PCM-modem Phase 2 downstream path.
       For now this reuses the existing L1/L2 probe envelope, but forces it
       through exact PCM codewords so the transmit chain is law-aware and
       separated from the generic V.34 analogue modulator. This is the shared
       foundation for V.90/V.91/V.92 downstream TX work. */
    for (sample = 0;  sample < max_len;  sample++)
    {
        int16_t raw;

        raw = (int16_t) lfastrintf(line_probe_samples[s->tx.line_probe_step]*s->tx.line_probe_scaling);
        amp[sample] = pcm_phase2_quantise(s, raw);
        if (++s->tx.line_probe_step >= LINE_PROBE_SAMPLES)
        {
            s->tx.line_probe_step = 0;
            if (++s->tx.line_probe_cycles == 8)
            {
                /* Drop 6dB for L2. No stage is recorded: s->tx.state is the
                   trellis encoder state, not a pipeline stage -- see the note
                   in l1_l2_signal_init(). */
                s->tx.line_probe_scaling *= 0.5f;
            }
            else if (s->tx.line_probe_cycles == (8 + 20))
            {
                if (s->tx.duplex)
                {
                    if (s->tx.calling_party)
                        info1_baud_init(s);
                    else if (s->tx.v90_mode)
                    {
                        /* §9.2.1.1.7 sends INFO1d only "after the digital modem
                           detects Tone A and has received the local echo of L2".
                           s->rx.signal_present is not that test -- it is true for
                           any energy at all, including the peer's own L1/L2 probe
                           still running, so it fired while the far end was still
                           working through the §9.2.1.1.5/§9.2.1.1.6 exchange and
                           sent INFO1d into the middle of it.  Measured over 16
                           calls: every call that reached INFO1a saw 4 L1/L2
                           events and sent 2 INFO1d frames carrying an identical
                           probe field; every call that failed saw 2 events, one
                           probe set, and a payload that varied call to call.  In
                           two of those the peer was still transmitting 11.4 s
                           later, i.e. it had not gone quiet -- we had jumped
                           ahead.
                           Require real Tone A evidence.  The guard-tone ratio is
                           the strongest such evidence available (V.34 10.1.2.1:
                           about +1 dB under Tone A against about -6 dB under an
                           INFO sequence) and does not depend on an event flag
                           that v34tx.c clears in dozens of places. */
                        bool tone_a_evidence;

                        tone_a_evidence = (s->rx.received_event == V34_EVENT_TONE_SEEN
                                        || s->rx.received_event == V34_EVENT_REVERSAL_1
                                        || s->rx.received_event == V34_EVENT_REVERSAL_3);
                        /* ...but not while the peer is still *holding* Tone A.
                           Measured over 8 calls, the guard/carrier ratio at this
                           decision separates the outcomes completely:

                             INFO1a received   -5.9 -5.0 -4.2 -5.9 -4.6 -5.9
                                               -6.0 -4.0 -5.1   (all negative)
                             INFO1a never came +0.2 +7.0 +4.4   (all positive)

                           Negative is the INFO configuration (10.1.2.3), i.e.
                           the peer has finished with Tone A and is ready to
                           exchange INFO; positive is Tone A itself (10.1.2.1),
                           i.e. it is still mid-§9.2.1.1.6 and INFO1d sent now
                           lands too early and is ignored.  Every reading here
                           carried event=REVERSAL_3, so the event flags cannot
                           tell these two apart -- only the ratio can. */
                        if (tone_a_evidence
                            &&  s->rx.guard_carrier_valid
                            &&  s->rx.guard_carrier_db > -2.5f)
                        {
                            span_log(&s->logging, SPAN_LOG_FLOW,
                                     "Tx - V.90: end of PCM L2 but peer still holds Tone A (guard %+.1f dB); waiting rather than sending INFO1d early\n",
                                     s->rx.guard_carrier_db);
                            tone_a_evidence = false;
                        }
                        /*endif*/
                        if (tone_a_evidence)
                        {
                            /* The expected third Tone A reversal can complete
                               during our PCM L2 and switches the RX directly to
                               INFO1A.  REVERSAL_3 is therefore positive Tone A
                               evidence, not a stale event; dropping it here
                               created a 650 ms silence before INFO1d on V.92. */
                            span_log(&s->logging, SPAN_LOG_FLOW,
                                     "Tx - V.90: Tone A at end of PCM L2 (event=%d guard=%+.1f dB valid=%d); sending INFO1d without a carrier gap\n",
                                     s->rx.received_event,
                                     s->rx.guard_carrier_db,
                                     s->rx.guard_carrier_valid);
                            info1_baud_init(s);
                        }
                        else
                        {
                            v90_wait_tone_a_init(s, false);
                        }
                    }
                    else
                        second_a_baud_init(s);
                }
                else
                {
                    if (s->tx.calling_party)
                        second_b_baud_init(s);
                    else
                        second_a_baud_init(s);
                }
                break;
            }
        }
    }
    return sample;
}
/*- End of function --------------------------------------------------------*/

static void l1_l2_signal_init(v34_state_t *s)
{
    span_log(&s->logging, SPAN_LOG_FLOW, "Tx - l2_l2_signal_init()\n");
    s->tx.line_probe_step = 0;
    s->tx.line_probe_cycles = 0;
    s->tx.line_probe_scaling = 0.0008f*V34_LINE_PROBE_LEVEL_TRIM*s->tx.gain;
    s->tx.current_modulator = (s->tx.v90_mode && !s->tx.calling_party) ? V34_MODULATION_PCM_L1_L2 : V34_MODULATION_L1_L2;
    /* 11.2.1.2.5 conditions the receiver to detect Tone B as L1 begins, so
       any Tone B seen earlier in the call (the 11.2.1.1.1 burst that carried
       INFO0c) is stale by here. */
    s->rx.tone_b_present = false;
    if (s->tx.duplex  &&  !s->tx.calling_party  &&  !s->tx.v90_mode)
        s->tx.phase2_probe_sent = true;
    /*endif*/
    /* Deliberately does NOT record an L1 stage. s->tx.state is the trellis
       encoder state (see qam_mod(): state = conv_encode_table[state][y4321]),
       not a pipeline stage -- the two fields are one character apart. Writing
       V34_TX_STAGE_L1 here used to corrupt the encoder for the rest of the
       call. The probe is identified by current_modulator, which is what the
       stage-change logging reports. */
}
/*- End of function --------------------------------------------------------*/

static int info1c_wait_bauds(v34_state_t *s)
{
    if (s->tx.v90_mode  &&  s->tx.calling_party)
    {
        /* §9.2.2.2.4's 2000 ms + two round trips is a floor on how long the
           analogue modem waits for INFO1d before recovering, not a ceiling on
           how long Tone A may be held -- and the recovery it offers,
           INFOMARKSa, stops the Tone A that §9.2.1.1.7 has the digital modem
           waiting to detect.  An Eicon Diva Server reaches the state where it
           looks for Tone A around 10 s into the call, by which time the
           default recovery has already replaced it.  ME_V90_ANALOGUE_INFO1D_WAIT_MS
           holds Tone A for longer so that peer can be measured. */
        const char *env = getenv("ME_V90_ANALOGUE_INFO1D_WAIT_MS");

        if (env  &&  *env)
        {
            long ms = strtol(env, NULL, 10);

            if (ms > 0)
                return (int) ((600*ms + 500)/1000);
            /*endif*/
        }
        /*endif*/
    }
    /*endif*/
    int rtd_bauds;
    int timeout_bauds;

    /* V.34/11.2.2.2.4 allows 2000 ms plus two round trip delays.  The
       deadline is specified from the Tone B detection of 11.2.1.2.6, which is
       a little before this stage begins; counting from stage entry is
       deliberately on the generous side.  This getbaud runs at the 600 baud
       CC rate, so express the budget in 600ths of a second (same convention
       as the V.90 INFO1a wait). */
    rtd_bauds = (s->rx.round_trip_delay_estimate > 0)
                ? (s->rx.round_trip_delay_estimate*600 + 4000)/8000
                : 0;
    timeout_bauds = (600*2000 + 500)/1000 + 2*rtd_bauds;
    if (timeout_bauds < 1)
        timeout_bauds = 1;
    /*endif*/
    return timeout_bauds;
}
/*- End of function --------------------------------------------------------*/

static complex_sig_t get_infomarksa_baud(v34_state_t *s)
{
    /* V.34/10.1.2.3.6: INFOMARKSa is binary ones applied to the DPSK
       modulator, i.e. a phase reversal every baud. */
    if (s->rx.received_event == V34_EVENT_INFO1_OK)
    {
        span_log(&s->logging, SPAN_LOG_FLOW,
                 "Tx - INFO1c received after %d bauds of INFOMARKSa, sending INFO1a (11.2.2.2.4)\n",
                 s->tx.tone_duration);
        s->rx.received_event = V34_EVENT_NONE;
        s->tx.tone_duration = 0;
        info1_baud_init(s);
        return s->tx.lastbit;
    }
    /*endif*/
    if (s->rx.received_event == V34_EVENT_INFO0_OK)
    {
        /* Same §11.2.2.1.1 recovery as in PRE_INFO1_A - the call modem is
           repeating INFO0c and needs an acknowledged INFO0a to move on. */
        s->rx.received_event = V34_EVENT_NONE;
        s->tx.tone_duration = 0;
        if (s->tx.phase2_probe_sent)
        {
            answer_resume_probe(s, "repeated INFO0c during INFOMARKSa");
            return s->tx.lastbit;
        }
        /*endif*/
        span_log(&s->logging, SPAN_LOG_FLOW,
                 "Tx - repeated INFO0c during INFOMARKSa; acknowledging with INFO0a bit 28 (11.2.2.1.1)\n");
        s->tx.info0_acknowledgement = true;
        info0_baud_init(s);
        return s->tx.lastbit;
    }
    /*endif*/
    if (s->rx.received_event == V34_EVENT_INFO1_BAD)
        s->rx.received_event = V34_EVENT_NONE;
    /*endif*/
    s->tx.tone_duration++;
    s->tx.lastbit.re = -s->tx.lastbit.re;
    return s->tx.lastbit;
}
/*- End of function --------------------------------------------------------*/

static void infomarksa_baud_init(v34_state_t *s)
{
    span_log(&s->logging, SPAN_LOG_FLOW, "Tx - infomarksa_baud_init()\n");
    s->tx.tone_duration = 0;
    s->tx.current_modulator = V34_MODULATION_CC;
    s->tx.lastbit = complex_sig_set(TRAINING_SCALE(TRAINING_AMP), TRAINING_SCALE(0.0f));
    s->tx.stage = V34_TX_STAGE_INFOMARKSA;
    s->tx.current_getbaud = get_infomarksa_baud;
}
/*- End of function --------------------------------------------------------*/

/* V.34/11.2.2.2.3 bounds the wait for Tone B at 600 ms plus a round trip
   delay from the beginning of L2.  This stage begins at the *end* of L2, so
   counting the whole budget from here is deliberately generous -- the point
   of the bound is to stop a silent peer wedging the call, not to be tight. */
static int post_l2_tone_b_wait_bauds(v34_state_t *s)
{
    int rtd_bauds;

    rtd_bauds = (s->rx.round_trip_delay_estimate > 0)
                ? (s->rx.round_trip_delay_estimate*600 + 4000)/8000
                : 0;
    return (600*600 + 500)/1000 + rtd_bauds;
}
/*- End of function --------------------------------------------------------*/

static complex_sig_t get_second_a_baud(v34_state_t *s)
{
    switch (s->tx.stage)
    {
    case V34_TX_STAGE_POST_L2_WAIT_TONE_B:
        /* V.34/11.2.1.2.6: the answer modem transmits the post-L2 Tone A
           phase reversal only "when Tone B is detected".  11.2.1.1.3 has the
           call modem transmit *silence* from its first Tone B reversal until
           it has received L1 and L2, and 11.2.1.1.5 has it raise Tone B only
           after that -- so the call modem is not listening for Tone A when
           our L2 ends.  Reversing off the end of our own L2, as this used to,
           puts the reversal on the line before the peer starts looking for
           it: measured against a SmartLink call modem the peer raised Tone B
           145 ms after our L2 ended, having missed a reversal sent 65 ms
           before that, and fell into the 11.2.2.1.1 INFO0c recovery from
           which the call never returned.  Hold Tone A until Tone B is seen.

           On timeout this falls through to the old unconditional behaviour
           rather than 11.2.2.2.3's return to 11.2.1.2.3, so the change can
           only add the wait -- never remove a path that used to work. */
        if (s->rx.tone_b_present
            ||
            ++s->tx.tone_duration >= post_l2_tone_b_wait_bauds(s))
        {
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Tx - Tone B %s after %d bauds of post-L2 Tone A; sending the "
                     "11.2.1.2.6 Tone A phase reversal\n",
                     s->rx.tone_b_present ? "detected" : "timeout",
                     s->tx.tone_duration);
            s->tx.tone_duration = 0;
            s->tx.stage = V34_TX_STAGE_POST_L2_A;
        }
        /*endif*/
        break;
    case V34_TX_STAGE_POST_L2_A:
        /* Send pure tone for 50ms (V.34/11.2.1.2.6) */
        if (++s->tx.tone_duration == 30)
        {
            /* 50ms has passed - reverse */
            s->tx.lastbit.re = -s->tx.lastbit.re;
            s->tx.tone_duration = 0;
            s->tx.stage = V34_TX_STAGE_POST_L2_NOT_A;
        }
        /*endif*/
        break;
    case V34_TX_STAGE_POST_L2_NOT_A:
        /* Send phase reversed pure tone for 10ms (V.34/11.2.1.2.6) */
        if (++s->tx.tone_duration == 6)
        {
            /* 10ms has passed - change to silence */
            s->tx.tone_duration = 0;
            s->tx.stage = V34_TX_STAGE_A_SILENCE;
        }
        /*endif*/
        break;
    case V34_TX_STAGE_A_SILENCE:
        /* Send silence, as we wait for L2 (V.34/11.2.1.2.6) */
        if (s->rx.received_event == V34_EVENT_L2_SEEN
            ||
            ++s->tx.tone_duration >= 390)
        {
            /* 650ms has passed - wait for INFO1c message */
            s->tx.lastbit.re = -s->tx.lastbit.re;
            s->tx.tone_duration = 0;
            s->tx.stage = V34_TX_STAGE_PRE_INFO1_A;
        }
        /*endif*/
        return zero;
    case V34_TX_STAGE_PRE_INFO1_A:
        /* V.34/11.2.1.2.8-9: transmit Tone A and condition the receiver to
           receive INFO1c.  Only *after* receiving INFO1c may we send INFO1a
           and proceed to Phase 3.  This used to fire off a fixed 180 baud
           (300 ms) timer with the INFO1c check commented out, so we sent
           INFO1a while the call modem was still in the INFO exchange, then
           left for Phase 3 without ever reading its line-probe results. */
        if (s->rx.received_event == V34_EVENT_INFO1_OK)
        {
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Tx - %s received after %d bauds of Tone A, sending INFO1a (%s)\n",
                     (s->tx.v90_mode  &&  s->tx.calling_party) ? "INFO1d" : "INFO1c",
                     s->tx.tone_duration,
                     (s->tx.v90_mode  &&  s->tx.calling_party) ? "9.2.2.1.9" : "11.2.1.2.9");
            s->rx.received_event = V34_EVENT_NONE;
            s->tx.tone_duration = 0;
            info1_baud_init(s);
        }
        else if (s->rx.received_event == V34_EVENT_INFO0_OK)
        {
            /* The call modem is repeating INFO0c instead of sending INFO1c,
               i.e. it is in the §11.2.2.1.1 recovery loop.  It leaves that
               loop on receiving INFO0a with bit 28 set, so acknowledge and
               resend.  info0_baud_init() parks us in INFO0_RETRY, whose
               existing handler returns to Tone A when the next INFO0c
               arrives - §11.2.2.1.1's "proceed according to 11.2.1.1.3". */
            s->rx.received_event = V34_EVENT_NONE;
            s->tx.tone_duration = 0;
            if (s->tx.phase2_probe_sent
                &&
                answer_info0_retry_policy() != 0
                &&
                !s->tx.phase2_info0_repeated)
            {
                s->tx.phase2_info0_repeated = true;
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - repeated INFO0c during INFO1c wait; repeating INFO0a "
                         "with bit 28 %s (11.2.2.2.1, ME_V34_INFO0_RETRY)\n",
                         (answer_info0_retry_policy() == 1) ? "set" : "clear");
                s->tx.info0_acknowledgement = (answer_info0_retry_policy() == 1);
                info0_baud_init(s);
            }
            else if (s->tx.phase2_probe_sent  &&  s->tx.phase2_info0_repeated)
            {
                /* Already answered this recovery.  Hold Tone A rather than
                   probing again: measured against a SmartLink call modem, an
                   answer modem that re-ran the 11.2.1.2.5 probe on every
                   repeated INFO0c drove it round its recovery 20 times in one
                   call, where holding Tone A let it pass and reach its
                   receive-probe state.  Its INFO detector is fed the wideband
                   probe whenever it is not in that state, which is the most
                   likely source of the INFO0s it kept declaring. */
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - repeated INFO0c after the recovery was answered; "
                         "holding Tone A (11.2.2.2.1)\n");
            }
            else if (s->tx.phase2_probe_sent)
            {
                /* 11.2.2.2.1 gives the answer modem two ways out of the INFO0
                   recovery, and only the first of them transmits anything
                   new: on INFO0c with bit 28 set, acknowledge; *or*, "if the
                   answer modem detects Tone B and has received INFO0c, it
                   shall complete the current INFO0a, and transmit Tone A" --
                   no fresh INFO0a at all.  Past 11.2.1.2.5 there is no
                   current INFO0a to complete, so that branch is just Tone A
                   and the probe.
                   Take it, because sending the INFO0a is what loses the call
                   here.  Measured against a SmartLink call modem: it enters
                   this recovery by itself on reaching its transmit-probe
                   state, and the acknowledged INFO0a it asks for is the very
                   thing it cannot survive -- 60 ms after accepting the frame
                   it re-reads its own info0-received flag as a repeat,
                   re-enters recovery, leaves it again on detecting our
                   Tone A, sets the flag from the same cached frame, and
                   loops until it retrains and clears the call (7 rounds
                   observed, 0.4-2.6 s apart, always the identical cached
                   octets).  With no INFO0a it leaves recovery on Tone A,
                   finds nothing flagged, and drops into its receive-probe
                   state -- which is what the resumed L1/L2 is for. */
                answer_resume_probe(s, "repeated INFO0c during the INFO1c wait");
            }
            else
            {
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - repeated INFO0c during INFO1c wait; acknowledging with INFO0a bit 28 (11.2.2.1.1)\n");
                s->tx.info0_acknowledgement = true;
                info0_baud_init(s);
            }
            /*endif*/
        }
        else
        {
            if (s->rx.received_event == V34_EVENT_INFO1_BAD)
            {
                /* A corrupt INFO1c is not a handshake failure - the call
                   modem repeats it.  Drop the event so it cannot be mistaken
                   for a later one, and keep waiting. */
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - INFO1c received with bad CRC at baud %d; continuing to wait\n",
                         s->tx.tone_duration);
                s->rx.received_event = V34_EVENT_NONE;
            }
            /*endif*/
            s->tx.tone_duration++;
            if (s->tx.phase2_info0_repeated
                &&
                s->tx.phase2_resume_count < 6
                &&
                s->tx.tone_duration >= (600*1500 + 500)/1000)
            {
                /* Post-recovery, and no INFO1c yet.  The call modem takes
                   several seconds of its own recovery to reach the state where
                   it receives L1/L2, and waits there only about 2 s before
                   retraining -- measured, it got there 600 ms after our single
                   resumed probe had finished.  Offer the probe again rather
                   than sitting on Tone A until the INFO1c deadline. */
                s->tx.tone_duration = 0;
                answer_resume_probe(s, "no INFO1c since the INFO0 recovery");
                break;
            }
            /*endif*/
            if (s->tx.tone_duration >= info1c_wait_bauds(s))
            {
                /* V.34/11.2.2.2.4: no INFO1c within 2000 ms + two round trip
                   delays.  Of the two permitted responses (retrain per
                   11.5.2.1, or INFOMARKSa) take INFOMARKSa, which keeps the
                   handshake alive and lets the call modem resend INFO1c. */
                const char *recovery = NULL;

                if (s->tx.v90_mode  &&  s->tx.calling_party)
                    recovery = getenv("ME_V90_ANALOGUE_INFO1D_TIMEOUT");
                /*endif*/
                if (recovery  &&  strcmp(recovery, "info1a") == 0)
                {
                    /* Not one of §9.2.2.2.4's two responses, and opt-in for
                       that reason.  A digital modem that never sends INFO1d
                       leaves the conformant pair unreachable: INFOMARKSa
                       replaces the Tone A the peer is waiting for, and a
                       retrain returns to the same deadlock.  An Eicon Diva
                       Server is such a peer -- it cycles its INFO receive
                       states waiting for INFO1a, whose bits 37:39 are what
                       its firmware reads to choose the V.90 page. */
                    span_log(&s->logging, SPAN_LOG_FLOW,
                             "Tx - no INFO1d within %d bauds; sending INFO1a anyway "
                             "(ME_V90_ANALOGUE_INFO1D_TIMEOUT=info1a)\n",
                             s->tx.tone_duration);
                    s->tx.tone_duration = 0;
                    info1_baud_init(s);
                    break;
                }
                /*endif*/
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - no %s within %d bauds; sending INFOMARKSa (%s)\n",
                         (s->tx.v90_mode  &&  s->tx.calling_party) ? "INFO1d" : "INFO1c",
                         s->tx.tone_duration,
                         (s->tx.v90_mode  &&  s->tx.calling_party) ? "9.2.2.2.4" : "11.2.2.2.4");
                infomarksa_baud_init(s);
            }
            /*endif*/
        }
        /*endif*/
        break;
    }
    /*endswitch*/
    return s->tx.lastbit;
}
/*- End of function --------------------------------------------------------*/

static void second_a_baud_init(v34_state_t *s)
{
    span_log(&s->logging, SPAN_LOG_FLOW, "Tx - second_a_baud_init()\n");
    s->tx.tone_duration = 0;
    s->tx.current_modulator = V34_MODULATION_CC;
    s->tx.lastbit = complex_sig_set(TRAINING_SCALE(TRAINING_AMP), TRAINING_SCALE(0.0f));
    s->tx.stage = V34_TX_STAGE_POST_L2_A;
    s->tx.current_getbaud = get_second_a_baud;
}
/*- End of function --------------------------------------------------------*/

/* Resume the V.34 answer modem at 11.2.1.2.5 -- the Tone A phase reversal,
   then L1 and L2 -- after the 11.2.2.2.1 INFO0 recovery.
   11.2.2.2.1 sends the answer modem back to 11.2.1.2.3, which waits for
   Tone B and then reverses Tone A.  A call modem that has already completed
   11.2.1.1.3 does not raise Tone B again: it has sent its Tone B reversal,
   gone silent, and is waiting for the second Tone A reversal and the probe.
   Measured against a SmartLink call modem, after acknowledging the repeated
   INFO0c the peer sat in its receive-probe state transmitting nothing while
   we held Tone A for a Tone B reversal that was never coming, until it
   retrained and cleared the call.  The same peer's own logs show the
   recovery is meant to be survivable -- in the answer role it runs the
   identical "errorrecovery is initialized in TX_PHASE2" branch and then goes
   straight on to transmit L1/L2. */
/* Which of 11.2.2.2.1's answers to a repeated INFO0c the answer modem gives
   once it is past 11.2.1.2.5.  The clause offers two, and only one of them
   transmits anything: acknowledge with bit 28, or -- "if the answer modem
   detects Tone B and has received INFO0c" -- complete the current INFO0a and
   transmit Tone A, which past 11.2.1.2.5 means no INFO0a at all.  This
   selects between them so a peer can be measured against both; "noack" is a
   third, non-conformant setting kept only for that measurement. */
static int answer_info0_retry_policy(void)
{
    static int initialized = 0;
    static int policy = 1;      /* 0 = none, 1 = acknowledged, 2 = unacknowledged */

    if (!initialized)
    {
        const char *value = getenv("ME_V34_INFO0_RETRY");

        if (value  &&  strcmp(value, "none") == 0)
            policy = 0;
        else if (value  &&  strcmp(value, "noack") == 0)
            policy = 2;
        /*endif*/
        initialized = 1;
    }
    /*endif*/
    return policy;
}
/*- End of function --------------------------------------------------------*/

/* Backstop on the Tone A hold after the INFO0 acknowledgement, in 600ths of a
   second.  The normal exit is the call modem dropping Tone B, which 11.2.1.1.3
   has it do exactly when it is ready to receive L1/L2; this only bounds the
   wait if that never happens.  ME_V34_RESUME_DELAY_MS. */
static int post_info0_resume_bauds(void)
{
    static int initialized = 0;
    static int bauds = (600*800 + 500)/1000;

    if (!initialized)
    {
        const char *value = getenv("ME_V34_RESUME_DELAY_MS");

        if (value  &&  *value)
        {
            long ms = strtol(value, NULL, 10);

            if (ms > 0  &&  ms < 5000)
                bauds = (int) ((600*ms + 500)/1000);
            /*endif*/
        }
        /*endif*/
        initialized = 1;
    }
    /*endif*/
    return bauds;
}
/*- End of function --------------------------------------------------------*/

static void answer_resume_probe(v34_state_t *s, const char *reason)
{
    s->tx.phase2_resume_count++;
    span_log(&s->logging, SPAN_LOG_FLOW,
             "Tx - %s with the probe already sent; resuming at the "
             "11.2.1.2.5 Tone A reversal and L1/L2\n",
             reason);
    s->tx.current_getbaud = get_initial_fdx_a_not_a_baud;
    s->tx.stage = V34_TX_STAGE_POST_INFO0_RESUME_A;
    s->tx.tone_duration = 0;
    s->tx.persistence2 = 0;
    s->tx.current_modulator = V34_MODULATION_CC;
    s->tx.lastbit = complex_sig_set(TRAINING_SCALE(TRAINING_AMP), TRAINING_SCALE(0.0f));
    /* Put the receiver back where it stood at the same point of the first
       pass: watching for Tone B, with reversal 1 behind it, so the call
       modem's next Tone B reversal reads as the 11.2.1.2.7 one that arms
       L1/L2 reception.  Leaving it in INFO1C would drop the peer's probe, and
       leaving received_event at REVERSAL_2 would make that reversal the
       third, which arms nothing. */
    s->rx.stage = V34_RX_STAGE_TONE_B;
    s->rx.target_bits = (s->rx.duplex)  ?  (109 - (4 + 8 + 4))  :  (111 - (4 + 8 + 4));
    s->rx.persistence1 = 0;
    s->rx.bit_count = 0;
    s->rx.persistence1 = 0;
    s->rx.persistence2 = 0;
    /* Leave the reversal ordinal at the start of the sequence, not part way
       through it: the resume re-runs 11.2.1.2.3, so the call modem's next
       Tone B reversal is the one FIRST_NOT_A waits on and the one after it is
       11.2.1.2.7's, which arms L1/L2 reception -- exactly the first pass. */
    s->rx.received_event = V34_EVENT_NONE;
    s->rx.tone_b_present = false;
    s->rx.tone_b_ended = false;
}
/*- End of function --------------------------------------------------------*/

static void post_l2_wait_tone_b_init(v34_state_t *s)
{
    span_log(&s->logging, SPAN_LOG_FLOW, "Tx - post_l2_wait_tone_b_init()\n");
    s->tx.tone_duration = 0;
    s->tx.current_modulator = V34_MODULATION_CC;
    s->tx.lastbit = complex_sig_set(TRAINING_SCALE(TRAINING_AMP), TRAINING_SCALE(0.0f));
    s->tx.stage = V34_TX_STAGE_POST_L2_WAIT_TONE_B;
    s->tx.current_getbaud = get_second_a_baud;
}
/*- End of function --------------------------------------------------------*/

static void pre_info1_a_init(v34_state_t *s)
{
    /* Tone A, held until the peer's INFO1 arrives.  V.34 reaches this as the
       answer modem (§11.2.1.2.8); the V.90 analogue modem reaches it as the
       *calling* party (§9.2.2.1.8), which is the same signal for the same
       reason -- the peer is waiting to see Tone A before it sends its INFO1. */
    span_log(&s->logging, SPAN_LOG_FLOW, "Tx - pre_info1_a_init()\n");
    s->tx.tone_duration = 0;
    s->tx.current_modulator = V34_MODULATION_CC;
    s->tx.lastbit = complex_sig_set(TRAINING_SCALE(TRAINING_AMP), TRAINING_SCALE(0.0f));
    s->tx.stage = V34_TX_STAGE_PRE_INFO1_A;
    s->tx.current_getbaud = get_second_a_baud;
}
/*- End of function --------------------------------------------------------*/

static complex_sig_t get_v90_wait_tone_a_baud(v34_state_t *s)
{
    int rtd_bauds;
    int timeout_bauds;

    /* V.90 §9.2.1.1.7: After L2, the digital modem waits for Tone A from the
       analogue modem, then sends INFO1d.  §9.2.1.2.5 does not declare Tone A
       missing until 650 ms plus RTD from the end of the local L2 echo.  This
       getbaud runs at the 600-baud control-channel rate.  The old hardcoded
       200-baud timeout was only 333 ms and sent INFO1d before a CX93001 raised
       Tone A; that peer then selected Table 11/V.34 despite mutual V.92.
       Only trigger early on actual Tone A detection -- L2_SEEN fires before
       the analogue modem has received our L1/L2 and started Tone A. */
    rtd_bauds = (s->rx.round_trip_delay_estimate > 0)
                ? (s->rx.round_trip_delay_estimate*600 + SAMPLE_RATE/2)/SAMPLE_RATE
                : 0;
    timeout_bauds = (600*650 + 500)/1000 + rtd_bauds;
    if (s->rx.received_event == V34_EVENT_INFO1_OK)
    {
        span_log(&s->logging, SPAN_LOG_FLOW,
                 "Tx - V.90: INFO1a received while waiting for Tone A, proceeding directly to Phase 3 handoff\n");
        s->tx.v90_info1a_fast_retries = 0;
        s->tx.v90_info1a_total_retries = 0;
        s_not_s_baud_init(s);
        s->rx.received_event = V34_EVENT_NONE;
        return zero;
    }
    /*endif*/

    if (s->rx.received_event == V34_EVENT_INFO0_OK)
    {
        span_log(&s->logging, SPAN_LOG_FLOW,
                 "Tx - V.90: ignoring stale repeated INFO0a while waiting for Tone A before INFO1d\n");
        s->rx.received_event = V34_EVENT_NONE;
        s->rx.persistence1 = 0;
        s->rx.persistence2 = 0;
        return zero;
    }
    /*endif*/

    if (s->rx.received_event == V34_EVENT_INFO0_BAD)
    {
        span_log(&s->logging, SPAN_LOG_FLOW,
                 "Tx - V.90: ignoring stale bad INFO0a while waiting for Tone A before INFO1d\n");
        s->rx.received_event = V34_EVENT_NONE;
        s->rx.persistence1 = 0;
        s->rx.persistence2 = 0;
        return zero;
    }
    /*endif*/

    ++s->tx.tone_duration;
    /* Same rule as the end-of-L2 gate above: while the peer is still holding
       Tone A (guard/carrier positive) it is not ready for INFO1d, so keep
       waiting.  The timeout below is the backstop, so a peer that never moves
       to the INFO configuration behaves exactly as before this check. */
    if (s->rx.guard_carrier_valid
        &&  s->rx.guard_carrier_db > -2.5f
        &&  s->tx.tone_duration < timeout_bauds)
    {
        if ((s->tx.tone_duration % 120) == 0)
        {
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Tx - V.90: holding INFO1d, peer still in Tone A (guard %+.1f dB) at %d bauds\n",
                     s->rx.guard_carrier_db,
                     s->tx.tone_duration);
        }
        /*endif*/
        s->rx.received_event = V34_EVENT_NONE;
        return zero;
    }
    /*endif*/
    if (s->rx.received_event == V34_EVENT_TONE_SEEN
        || s->rx.received_event == V34_EVENT_REVERSAL_1
        || (s->rx.signal_present && s->tx.tone_duration >= 30)
        || s->tx.tone_duration >= timeout_bauds)
    {
        if (s->tx.tone_duration < 30
            && (s->rx.received_event == V34_EVENT_TONE_SEEN
                || s->rx.received_event == V34_EVENT_REVERSAL_1))
        {
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Tx - V.90: ignoring early Tone A indication before INFO1d (event=%d) at %d bauds\n",
                     s->rx.received_event,
                     s->tx.tone_duration);
            s->rx.received_event = V34_EVENT_NONE;
            s->rx.persistence1 = 0;
            s->rx.persistence2 = 0;
            return zero;
        }
        /*endif*/
        /* Tone A detected (or timeout) — proceed to INFO1d */
        span_log(&s->logging, SPAN_LOG_FLOW,
                 "Tx - V.90: %s (event=%d signal=%d) after %d bauds, sending INFO1d\n",
                 (s->tx.tone_duration >= timeout_bauds) ? "Timeout"
                 : (((s->rx.received_event == V34_EVENT_TONE_SEEN)
                     || (s->rx.received_event == V34_EVENT_REVERSAL_1))
                    ? "RX event"
                    : "Tone A present (edge missed)"),
                 s->rx.received_event,
                 s->rx.signal_present,
                 s->tx.tone_duration);
        info1_baud_init(s);
    }
    /*endif*/
    return zero;
}
/*- End of function --------------------------------------------------------*/

static complex_sig_t get_v90_wait_info1a_baud(v34_state_t *s)
{
    enum
    {
        V90_INFO1A_TONE_GUARD_BAUDS = 120,
        V90_INFO1A_FAST_RETRY_BAUDS = 120,
        V90_INFO1A_MAX_FAST_RETRIES = 3,
        V90_INFO1A_MAX_TOTAL_RETRIES = 6,
        V90_INFO1A_INTERNAL_CLOCK_MAX_FAST_RETRIES = 1,
        V90_INFO1A_INTERNAL_CLOCK_MAX_TOTAL_RETRIES = 2,
        V90_INFO1A_MAX_RETRAIN_RESPONSES = 2,
        /* Total INFO0d re-send recoveries allowed per Phase 2 attempt while
           waiting for INFO1a.  Shares v90_phase2_info0_recovery_loops with the
           FIRST_B_SILENCE recovery, so this bounds the combined budget: a peer
           still repeating INFO0a after this many INFO0d resends will not be
           un-stuck by another one, and resending unbounded both wiped reversal
           progress and produced INFO0d retry storms that ended in Link Error. */
        V90_INFO1A_MAX_INFO0_RECOVERIES = 3
    };
    int baud_rate;
    int rtd_bauds;
    int timeout_bauds;
    int max_fast_retries;
    int max_total_retries;

    if (s->tx.stage == V34_TX_STAGE_V90_RETRAIN_SILENCE)
    {
        /* V.90 §9.5.1.2/§9.5.2: after 70 ± 5 ms of silence, transmit the
           role's tone and resume Phase 2 after the omitted INFO0 exchange. */
        if (++s->tx.tone_duration >= 42)
        {
            if (s->calling_party)
            {
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - V.90 analogue: retrain silence complete; transmitting Tone A and awaiting Tone B\n");
                s->tx.tone_duration = 0;
                s->rx.received_event = V34_EVENT_NONE;
                s->rx.persistence1 = 0;
                s->rx.persistence2 = 0;
                s->rx.info0_received = true; /* §9.5 omits INFO0. */
                s->rx.current_demodulator = V34_MODULATION_TONES;
                s->rx.stage = V34_RX_STAGE_TONE_B;
                initial_ab_not_ab_baud_init(s);
                return zero;
            }
            /*endif*/
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Tx - V.90: retrain-response silence complete; transmitting Tone B and awaiting Tone A reversal\n");
            /* V.90 §9.2.1.1.8: "Any subsequent retrains shall use Phase 2 of
               V.90 regardless of the analogue modem's choice of operating
               mode" -- drop any V.34-fallback role flip and restore the
               answerer scrambler taps for the fresh Phase 2/3. */
            if (s->tx.v90_v34_fallback  ||  s->rx.v90_v34_fallback)
            {
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - V.90: clearing V.34-fallback state for retrain\n");
                s->tx.v90_v34_fallback = false;
                s->rx.v90_v34_fallback = false;
                s->tx.scrambler_tap = 4;
                s->rx.scrambler_tap = 17;
            }
            /*endif*/
            s->tx.tone_duration = 0;
            s->tx.lastbit = complex_sig_set(TRAINING_SCALE(TRAINING_AMP), TRAINING_SCALE(0.0f));
            s->tx.current_getbaud = get_initial_fdx_b_not_b_baud;
            s->tx.stage = V34_TX_STAGE_V90_PHASE2_B_INFO0_SEEN;
            s->rx.received_event = V34_EVENT_NONE;
            s->rx.persistence1 = 0;
            s->rx.persistence2 = 0;
            s->rx.current_demodulator = V34_MODULATION_TONES;
            s->rx.stage = V34_RX_STAGE_TONE_A;
        }
        /*endif*/
        return zero;
    }
    /*endif*/
    if (s->tx.stage != V34_TX_STAGE_V90_WAIT_INFO1A)
        return zero;
    /*endif*/

    if (s->tx.baud_rate >= 0  &&  s->tx.baud_rate <= 5)
        baud_rate = baud_rate_parameters[s->tx.baud_rate].baud_rate;
    else
        baud_rate = 3200;
    /*endif*/
    (void) baud_rate;
    /* This getbaud runs at the 600 baud CC rate, so the §9.2.1.2.6 deadline
       of 700 ms + RTD is measured in 600ths of a second, not in symbol-rate
       bauds (the previous use of the 3200-ish symbol rate stretched the
       "700 ms" deadline to ~3.7 s of wall clock). */
    rtd_bauds = (s->rx.round_trip_delay_estimate > 0)
                ? (s->rx.round_trip_delay_estimate*600 + 4000)/8000
                : 0;
    timeout_bauds = (600*700 + 500)/1000 + rtd_bauds;
    if (timeout_bauds < 1)
        timeout_bauds = 1;
    /*endif*/
    /* far_capabilities.tx_clock_source was being used here as an "is the
       peer's TX clock free-running" signal to cut the retry budget from
       3/6 down to 1/2. But this project always runs as the V.90 digital
       modem/answerer (calling_party is always false -- see v34_init() in
       modem_engine.c), so process_rx_info0() never takes the branch that
       gives this field a real meaning (that branch requires v90_mode &&
       calling_party, populating it from INFO0d's genuine PCM-law bit).
       On the branch we always take, it's populated straight from INFO0a
       bits 26:27, which V.90 Table 8 defines as "Reserved for the ITU...
       not interpreted by the digital modem" -- i.e. spec-undefined for our
       role. Live interop showed this mattering: a USR Courier that was
       still actively cycling Tone A/reversal (working, just needed more
       time) got cut off after ~2 total retries (~1.4s) because its INFO0a
       happened to carry 0 in those reserved bits, while a different
       analogue modem that carried a nonzero value there got the full
       budget for no more legitimate reason. Always use the full budget. */
    max_fast_retries = V90_INFO1A_MAX_FAST_RETRIES;
    max_total_retries = V90_INFO1A_MAX_TOTAL_RETRIES;

    if (s->rx.received_event == V34_EVENT_INFO1_OK)
    {
        span_log(&s->logging, SPAN_LOG_FLOW,
                 "Tx - V.90: INFO1a received after %d bauds of wait, proceeding to Phase 3 handoff\n",
                 s->tx.tone_duration);
        s->tx.v90_phase2_info0_recovery_loops = 0;
        s->tx.v90_info1a_fast_retries = 0;
        s->tx.v90_info1a_total_retries = 0;
        s->tx.v90_info1a_retrain_responses = 0;
        if (s->rx.v90_v34_fallback)
        {
            /* V.90 §9.2.1.1.8: INFO1a bits 37:39 selected V.34; proceed per
               11.3.1.1/V.34 as a CALL modem -- silent until the analogue
               modem (answer role) completes its Phase 3 lead. */
            v90_v34_fallback_wait_init(s);
        }
        else
        {
            s_not_s_baud_init(s);
        }
        /*endif*/
        s->rx.received_event = V34_EVENT_NONE;
        return zero;
    }
    /*endif*/

    if (s->rx.received_event == V34_EVENT_INFO1_BAD)
    {
        span_log(&s->logging, SPAN_LOG_FLOW,
                 "Tx - V.90: INFO1a candidate failed CRC at %d bauds, continuing to wait\n",
                 s->tx.tone_duration);
        s->rx.received_event = V34_EVENT_NONE;
        goto wait_timeout_check;
    }
    /*endif*/

    if (s->rx.v90_repeated_info0a_pending || s->rx.received_event == V34_EVENT_INFO0_OK)
    {
        int recoveries = v90_note_phase2_info0_recovery(s);

        if (recoveries <= V90_INFO1A_MAX_INFO0_RECOVERIES)
        {
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Tx - V.90: repeated INFO0a arrived while waiting for INFO1a; re-sending acknowledged INFO0d to recover Phase 2 (recovery %d)\n",
                     recoveries);
            s->tx.info0_acknowledgement = true;
            s->tx.info0_retry_count = 0;
            s->tx.tone_duration = 0;
            s->rx.v90_repeated_info0a_pending = false;
            s->rx.v90_info1d_sent = false;
            s->rx.received_event = V34_EVENT_NONE;
            v90_phase2_reset_transactions(s);
            info0_baud_init(s);
            return zero;
        }
        /*endif*/

        /* Recovery cap reached: a peer that keeps repeating INFO0a after we
           have already acknowledged INFO0d and sent INFO1d this many times is
           not going to be un-stuck by another INFO0d.  Stop the resend loop --
           each resend threw away the counted Tone A reversals and, unbounded,
           stormed INFO0d until the peer answered with Link Error.  Hold for
           INFO1a instead; the §9.2.1.2.6 timeout path below then drives the
           Tone A / INFOMARKSa recovery, keeping the reversal progress intact. */
        span_log(&s->logging, SPAN_LOG_FLOW,
                 "Tx - V.90: repeated INFO0a persists after %d INFO0d recoveries while waiting for INFO1a; holding for INFO1a/timeout instead of another INFO0d loop\n",
                 recoveries);
        s->rx.v90_repeated_info0a_pending = false;
        s->rx.received_event = V34_EVENT_NONE;
        goto wait_timeout_check;
    }
    /*endif*/

    /* §9.2.1.2.6, the INFOMARKSa half: "Upon detection of INFOMARKSa, the
       digital modem shall either initiate a retrain according to 9.5.1.1 or
       send INFO1d and proceed in accordance with 9.2.1.1.8."  We take the
       send-INFO1d option, which is what the peer is asking for by marking.
       This is the trigger the re-send belongs to; before the INFOMARKSa
       detector existed the timeout below drove it off nothing in particular,
       and the Tone A branch -- whose spec response is a retrain, not a
       re-send -- was unreachable. */
    if (s->rx.received_event == V34_EVENT_INFOMARKSA_SEEN)
    {
        s->rx.received_event = V34_EVENT_NONE;
        if (s->tx.v90_info1a_total_retries < max_total_retries)
        {
            s->tx.v90_info1a_total_retries++;
            s->tx.v90_info1a_fast_retries = 0;
            s->tx.tone_duration = 0;
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Tx - V.90: INFOMARKSa while waiting for INFO1a; re-sending INFO1d per 9.2.1.2.6 (total=%d)\n",
                     s->tx.v90_info1a_total_retries);
            info1_baud_init(s);
            return zero;
        }
        /*endif*/
        span_log(&s->logging, SPAN_LOG_FLOW,
                 "Tx - V.90: INFOMARKSa after %d INFO1d re-sends; leaving it to the 9.2.1.2.6 deadline\n",
                 s->tx.v90_info1a_total_retries);
    }
    /*endif*/

    if (s->rx.received_event == V34_EVENT_TONE_SEEN
        ||  s->rx.received_event == V34_EVENT_REVERSAL_1)
    {
        /* +1 because wait_timeout_check below pre-increments tone_duration
           before comparing it against the same deadline.  Testing the
           un-incremented value here made this branch unreachable: on the baud
           where the counter would have reached timeout_bauds, this test saw
           timeout_bauds-1 and fell through, and the timeout then fired,
           re-sent INFO1d and reset the counter to 0.  Measured live against
           the Courier (2026-08-13): tone_duration topped out at 414 against a
           420-baud deadline in every failing call, v90_info1a_retrain_responses
           was never once incremented, and Phase 2 failed ~40% of the time by
           exhausting six INFO1d re-sends and falling back to V.22bis while the
           peer sat there transmitting Tone A. */
        if (s->tx.tone_duration + 1 >= timeout_bauds
            &&  s->tx.v90_info1a_retrain_responses < V90_INFO1A_MAX_RETRAIN_RESPONSES)
        {
            /* §9.2.1.2.6: Tone A after the INFO1a deadline is the analog
               modem initiating a retrain; respond per §9.5.1.2 (70 ms
               silence, then Tone B and the §9.2.1.1.3 ranging exchange)
               instead of ignoring it until the peer gives up.  Note the spec
               assigns the *other* action -- re-sending INFO1d -- to
               INFOMARKSa, not to Tone A. */
            s->tx.v90_info1a_retrain_responses++;
            v90_phase2_reset_transactions(s);
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Tx - V.90: Tone A after INFO1a deadline (event=%d at %d bauds); responding to retrain per 9.5.1.2 (response %d)\n",
                     s->rx.received_event,
                     s->tx.tone_duration,
                     s->tx.v90_info1a_retrain_responses);
            s->tx.tone_duration = 0;
            s->tx.v90_info1a_fast_retries = 0;
            s->tx.stage = V34_TX_STAGE_V90_RETRAIN_SILENCE;
            s->rx.v90_info1d_sent = false;
            s->rx.received_event = V34_EVENT_NONE;
            s->rx.persistence1 = 0;
            s->rx.persistence2 = 0;
            return zero;
        }
        /*endif*/
        span_log(&s->logging, SPAN_LOG_FLOW,
                 "Tx - V.90: %signoring Tone A/reversal while waiting for INFO1a (event=%d) at %d bauds; continuing to wait for INFO1a until timeout\n",
                 (s->tx.tone_duration < V90_INFO1A_TONE_GUARD_BAUDS) ? "early " : "",
                 s->rx.received_event,
                 s->tx.tone_duration);
        s->rx.received_event = V34_EVENT_NONE;
        s->rx.persistence1 = 0;
        s->rx.persistence2 = 0;
        goto wait_timeout_check;
    }
    /*endif*/

    if (!s->rx.signal_present
        && s->tx.tone_duration >= V90_INFO1A_FAST_RETRY_BAUDS
        && s->tx.v90_info1a_fast_retries < max_fast_retries)
    {
        s->tx.v90_info1a_fast_retries++;
        s->tx.v90_info1a_total_retries++;
        if (s->tx.v90_info1a_total_retries >= max_total_retries)
        {
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Tx - V.90: aborting after %d INFO1a retries with repeated carrier loss and no valid INFO1a\n",
                     s->tx.v90_info1a_total_retries);
            s->rx.training_failed_reported = false;
            s->rx.received_event = V34_EVENT_TRAINING_FAILED;
            tx_silence_init(s, 30000);
            s->tx.stage = 0;
            return zero;
        }
        /*endif*/
        span_log(&s->logging, SPAN_LOG_FLOW,
                 "Tx - V.90: carrier absent while waiting for INFO1a after %d bauds; fast retrying INFO1d (fast retries=%d total=%d)\n",
                 s->tx.tone_duration,
                 s->tx.v90_info1a_fast_retries,
                 s->tx.v90_info1a_total_retries);
        s->rx.received_event = V34_EVENT_NONE;
        s->rx.persistence1 = 0;
        s->rx.persistence2 = 0;
        info1_baud_init(s);
        return zero;
    }
    /*endif*/

    if (!s->rx.signal_present
        && s->tx.tone_duration >= V90_INFO1A_FAST_RETRY_BAUDS
        && s->tx.v90_info1a_fast_retries >= max_fast_retries)
    {
        s->tx.v90_info1a_total_retries++;
        if (s->tx.v90_info1a_total_retries >= max_total_retries)
        {
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Tx - V.90: aborting after %d INFO1a recovery attempts with repeated carrier loss and no valid INFO1a\n",
                     s->tx.v90_info1a_total_retries);
            s->rx.training_failed_reported = false;
            s->rx.received_event = V34_EVENT_TRAINING_FAILED;
            tx_silence_init(s, 30000);
            s->tx.stage = 0;
            return zero;
        }
        /*endif*/
        span_log(&s->logging, SPAN_LOG_FLOW,
                 "Tx - V.90: exhausted fast INFO1d retries with carrier still absent; restarting Phase 2 from acknowledged INFO0d (total=%d)\n",
                 s->tx.v90_info1a_total_retries);
        s->tx.v90_info1a_fast_retries = 0;
        s->tx.info0_acknowledgement = true;
        s->tx.info0_retry_count = 0;
        s->tx.tone_duration = 0;
        s->rx.v90_repeated_info0a_pending = false;
        s->rx.received_event = V34_EVENT_NONE;
        s->rx.persistence1 = 0;
        s->rx.persistence2 = 0;
        v90_phase2_reset_transactions(s);
        info0_baud_init(s);
        return zero;
    }
    /*endif*/

wait_timeout_check:
    if (++s->tx.tone_duration >= timeout_bauds)
    {
        /* §9.2.1.2.6 branches on what the receiver detects at the deadline,
           so decide from what is *present* rather than from received_event.
           The event is published once, at the baud persistence2 reaches 20,
           and v34tx.c clears it in dozens of places, so at the deadline it is
           almost always NONE -- which is why the Tone A branch above never
           fired even once it was made reachable, and every failing call took
           the re-send path regardless of what the peer was doing.
           persistence2 >= 20 is the same "sustained zeros on the DPSK stream"
           condition the Tone A detector itself uses. */
        /* Is the peer holding Tone A, or transmitting an INFO sequence?
           V.34 10.1.2.1/10.1.2.3 fix the 1800 Hz guard tone's level in each
           state, so the guard/carrier ratio answers it directly: about +1 dB
           under Tone A, about -6 dB under INFO.  -2.5 dB is the midpoint.
           persistence2 >= 20 was the previous test and never once fired --
           measured live, the deadline is reached with persistence2 below it
           even while the peer sits in Tone A. */
        {
            bool tone_a_present;

            if (s->rx.guard_carrier_valid)
                tone_a_present = (s->rx.guard_carrier_db > -2.5f);
            else
                tone_a_present = (s->rx.persistence2 >= 20);
            /*endif*/
            if (s->rx.signal_present
                &&  tone_a_present
                &&  s->tx.v90_info1a_retrain_responses < V90_INFO1A_MAX_RETRAIN_RESPONSES)
        {
            s->tx.v90_info1a_retrain_responses++;
            v90_phase2_reset_transactions(s);
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Tx - V.90: INFO1a deadline, guard/carrier %+.1f dB (valid=%d) => Tone A; responding to retrain per 9.5.1.2 (response %d)\n",
                     s->rx.guard_carrier_db,
                     s->rx.guard_carrier_valid,
                     s->tx.v90_info1a_retrain_responses);
            s->tx.tone_duration = 0;
            s->tx.v90_info1a_fast_retries = 0;
            s->tx.stage = V34_TX_STAGE_V90_RETRAIN_SILENCE;
            s->rx.v90_info1d_sent = false;
            s->rx.received_event = V34_EVENT_NONE;
            s->rx.persistence1 = 0;
            s->rx.persistence2 = 0;
            return zero;
        }
            /*endif*/
        }
        /*endif*/
        s->tx.v90_info1a_total_retries++;
        s->tx.v90_info1a_fast_retries = 0;
        if (s->tx.v90_info1a_total_retries >= max_total_retries)
        {
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Tx - V.90: aborting after %d INFO1a timeouts without valid INFO1a; signalling training failure\n",
                     s->tx.v90_info1a_total_retries);
            s->rx.training_failed_reported = false;
            s->rx.received_event = V34_EVENT_TRAINING_FAILED;
            tx_silence_init(s, 30000);
            s->tx.stage = 0;
            return zero;
        }
        /*endif*/
        span_log(&s->logging, SPAN_LOG_FLOW,
                 "Tx - V.90: INFO1a wait timeout after %d bauds (~700ms + RTD), re-sending INFO1d (timeout retries=%d)\n",
                 s->tx.tone_duration,
                 s->tx.v90_info1a_total_retries);
        info1_baud_init(s);
    }
    /*endif*/
    return zero;
}
/*- End of function --------------------------------------------------------*/

static void v90_wait_tone_a_init(v34_state_t *s, bool preserve_tone_a_event)
{
    span_log(&s->logging, SPAN_LOG_FLOW,
             "Tx - v90_wait_tone_a_init(): waiting for Tone A before INFO1d%s\n",
             preserve_tone_a_event ? " (preserving prior Tone A indication)" : "");
    s->tx.v90_phase2_info0_recovery_loops = 0;
    s->tx.tone_duration = 0;
    s->rx.v90_info1d_sent = false;
    /* Use CC modulation so getbaud is called each baud — outputs silence via zero return */
    s->tx.current_modulator = V34_MODULATION_CC;
    s->tx.stage = V34_TX_STAGE_V90_WAIT_TONE_A;
    s->tx.current_getbaud = get_v90_wait_tone_a_baud;
    /* Clear stale RX events so we wait for a fresh Tone A detection, but preserve
       a legitimate Tone A indication some analog modems emit before we enter this
       state (e.g. reversal 1 during the L1/L2 to INFO1d crossover). */
    if (!preserve_tone_a_event)
    {
        s->rx.received_event = V34_EVENT_NONE;
        s->rx.persistence1 = 0;
        s->rx.persistence2 = 0;
    }
    /*endif*/
    /* Set RX to detect Tone A from analog modem.
       Must also switch demodulator to TONES — if L1/L2 analysis is still running,
       the tone detector won't fire. */
    s->rx.current_demodulator = V34_MODULATION_TONES;
    s->rx.stage = V34_RX_STAGE_TONE_A;
}
/*- End of function --------------------------------------------------------*/

static void v90_wait_info1a_init(v34_state_t *s)
{
    span_log(&s->logging, SPAN_LOG_FLOW,
             "Tx - V.90: INFO1d complete, sending silence, waiting for INFO1a with Tone A recovery armed\n");
    s->tx.tone_duration = 0;
    /* Use CC modulation so we get a per-baud callback while transmitting silence. */
    s->tx.current_modulator = V34_MODULATION_CC;
    s->tx.stage = V34_TX_STAGE_V90_WAIT_INFO1A;
    s->tx.current_getbaud = get_v90_wait_info1a_baud;
    /* In V.90, after the second reversal exchange the analog modem sends
       Tone A carrying INFO1a directly. Keep the RX in TONE_A while changing
       the target length to INFO1a so we can still observe fresh Tone A /
       reversal events from peers that reassert the tone before the INFO1a
       sync word settles, while info_rx() continues searching for the 70-bit
       INFO1a payload. */
    s->rx.current_demodulator = V34_MODULATION_TONES;
    s->rx.target_bits = 70 - (4 + 8 + 4);
    if (s->rx.stage == V34_RX_STAGE_INFO1A  &&  s->rx.bit_count > 0)
    {
        /* The receiver is armed from the start of the INFO1d burst
           (info1_baud_init) and the peer's INFO1a may already be
           mid-accumulation when the last repeat finishes.  Zeroing the
           accumulator or yanking the stage back to TONE_A here would
           destroy that in-flight frame. */
        span_log(&s->logging, SPAN_LOG_FLOW,
                 "Tx - V.90: INFO1a already accumulating (%d bits) at end of INFO1d; preserving it\n",
                 s->rx.bit_count);
    }
    else
    {
        s->rx.bit_count = 0;
        s->rx.bitstream = 0;
        s->rx.stage = V34_RX_STAGE_TONE_A;
    }
    /*endif*/
    s->rx.v90_repeated_info0a_pending = false;
    s->rx.v90_info1d_sent = true;
    s->rx.received_event = V34_EVENT_NONE;
    s->rx.persistence1 = 0;
    s->rx.persistence2 = 0;
}
/*- End of function --------------------------------------------------------*/

static complex_sig_t get_v34_fallback_wait_baud(v34_state_t *s)
{
    /* V.34 §11.3.1.1.1-11.3.1.1.3 (call-modem Phase 3): stay silent while the
       answer-role analogue modem sends S/S-bar/PP/TRN/J; after receiving J,
       respond with our own S within 500 ms.  This getbaud runs at the 600
       baud CC rate.  Live CX93001 timing: its full lead takes ~1 s, so 8 s
       covers slow peers plus repeats before the interop escape hatch. */
    enum
    {
        V34_FALLBACK_WAIT_MAX_BAUDS = 600*8
    };

    if (s->tx.stage != V34_TX_STAGE_V34_FALLBACK_WAIT_J)
        return zero;
    /*endif*/

    if (s->rx.received_event == V34_EVENT_J
        ||  s->rx.phase3_j_trn16 >= 0)
    {
        span_log(&s->logging, SPAN_LOG_FLOW,
                 "Tx - V.34 fallback: far-end Phase 3 J %s after %d bauds of silence; responding with S/S-bar\n",
                 (s->rx.received_event == V34_EVENT_J) ? "event" : "decode",
                 s->tx.tone_duration);
        s->rx.received_event = V34_EVENT_NONE;
        s_not_s_baud_init(s);
        return zero;
    }
    /*endif*/
    if (++s->tx.tone_duration >= V34_FALLBACK_WAIT_MAX_BAUDS)
    {
        /* Interop escape hatch: if the peer's J never decodes, respond anyway
           rather than dying silent -- the peer's own recovery (INFOMARKSa) can
           still resynchronise on our Phase 3. */
        span_log(&s->logging, SPAN_LOG_FLOW,
                 "Tx - V.34 fallback: no far-end J after %d bauds; starting our Phase 3 anyway\n",
                 s->tx.tone_duration);
        s_not_s_baud_init(s);
    }
    /*endif*/
    return zero;
}
/*- End of function --------------------------------------------------------*/

static void v90_v34_fallback_wait_init(v34_state_t *s)
{
    span_log(&s->logging, SPAN_LOG_FLOW,
             "Tx - V.90 declined -> V.34 fallback (9.2.1.1.8): call-modem role; "
             "silence while far end leads Phase 3 (S/S-bar/PP/TRN/J)\n");
    s->tx.v90_v34_fallback = true;
    s->tx.tone_duration = 0;
    /* Use CC modulation so we get a per-baud callback while transmitting silence. */
    s->tx.current_modulator = V34_MODULATION_CC;
    s->tx.stage = V34_TX_STAGE_V34_FALLBACK_WAIT_J;
    s->tx.current_getbaud = get_v34_fallback_wait_baud;
}
/*- End of function --------------------------------------------------------*/

static complex_sig_t get_second_b_baud(v34_state_t *s)
{
    switch (s->tx.stage)
    {
    case V34_TX_STAGE_HDX_POST_L2_B:
        /* Send pure tone until we receive INFOh (V.34/12.2.1.1.4) */
        if (s->rx.received_event == V34_EVENT_INFOH_OK)
        {
            s->tx.tone_duration = 0;
            s->tx.stage = V34_TX_STAGE_HDX_POST_L2_SILENCE;
        }
        else if (s->rx.received_event == V34_EVENT_INFO0_BAD
                 ||
                 s->rx.received_event == V34_EVENT_TONE_SEEN)
        {
        }
        else if (++s->tx.tone_duration == 1200)
        {
            /* Timeout, as we have not received INFOh after 2s */
        }
        /*endif*/
        break;
    case V34_TX_STAGE_HDX_POST_L2_SILENCE:
        /* Send silence for 75ms (V.34/12.3.1.1) */
        if (++s->tx.tone_duration == 45)
        {
            s->tx.tone_duration = 0;
        }
        /*endif*/
        return zero;
    }
    /*endswitch*/
    return s->tx.lastbit;
}
/*- End of function --------------------------------------------------------*/

static void second_b_baud_init(v34_state_t *s)
{
    /* This is for half-duplex */
    span_log(&s->logging, SPAN_LOG_FLOW, "Tx - second_b_baud_init()\n");
    s->tx.tone_duration = 0;
    s->tx.current_modulator = V34_MODULATION_CC;
    s->tx.lastbit = complex_sig_set(TRAINING_SCALE(TRAINING_AMP), TRAINING_SCALE(0.0f));
    s->tx.stage = V34_TX_STAGE_HDX_POST_L2_B;
    s->tx.current_getbaud = get_second_b_baud;
}
/*- End of function --------------------------------------------------------*/

static complex_sig_t get_infoh_baud(v34_state_t *s)
{
    int bit;

    bit = get_data_bit(&s->tx);
    if (s->tx.txptr >= s->tx.txbits)
    {
        if (s->tx.calling_party)
            tx_silence_init(s, 30000);
        else
            s_not_s_baud_init(s);
        /*endif*/
    }
    /*endif*/
    if (bit)
        s->tx.lastbit.re = -s->tx.lastbit.re;
    /*endif*/
    return s->tx.lastbit;
}
/*- End of function --------------------------------------------------------*/

static void infoh_baud_init(v34_state_t *s)
{
    span_log(&s->logging, SPAN_LOG_FLOW, "Tx - infoh_baud_init()\n");
    prepare_infoh(s);
    s->tx.txbits = infoh_sequence_tx(&s->tx, &s->tx.infoh);
    s->tx.txbits += 8;
    s->tx.txptr = 0;
#if 0
#if defined(SPANDSP_USE_FIXED_POINT)
    cvec_zeroi16(s->tx.rrc_filter, sizeof(s->tx.rrc_filter)/sizeof(s->tx.rrc_filter[0]));
#else
    cvec_zerof(s->tx.rrc_filter, sizeof(s->tx.rrc_filter)/sizeof(s->tx.rrc_filter[0]));
#endif
    s->tx.lastbit = complex_sig_set(TRAINING_SCALE(0.0f), TRAINING_SCALE(0.0f));
    s->tx.rrc_filter_step = 0;
    s->tx.baud_phase = 0;
#endif

    s->tx.lastbit = complex_sig_set(TRAINING_SCALE(TRAINING_AMP), TRAINING_SCALE(0.0f));
    /* Round up to a whole number of bytes */
    s->tx.txbits = (s->tx.txbits + 7) & ~7;
    s->tx.current_modulator = V34_MODULATION_CC;
    s->tx.current_getbaud = get_infoh_baud;
}
/*- End of function --------------------------------------------------------*/

static complex_sig_t get_v34_call_phase3_wait_baud(v34_state_t *s)
{
    /* V.34 11.3.1.1.1-.3: after INFO1c the call modem is silent while
       receiving the answerer's S/S-bar, PP, first 512T of TRN and J. */
    if (s->rx.received_event == V34_EVENT_J)
    {
        span_log(&s->logging, SPAN_LOG_FLOW,
                 "Tx - V.34 caller: far-end J received, starting local Phase 3 S/S-bar\n");
        s->rx.received_event = V34_EVENT_NONE;
        s->tx.phase3_call_wait_j = false;
        s->tx.current_getbaud = get_s_not_s_baud;
    }
    return zero;
}
/*- End of function --------------------------------------------------------*/

static complex_sig_t get_v34_call_info1a_wait_baud(v34_state_t *s)
{
    /* V.34 11.2.1.1.9 and 11.3.1.1.1: INFO1a still arrives on the answer
       modem's control channel after INFO1c transmission completes.  Do not
       switch this receiver to the primary channel until INFO1a has selected
       its answer-to-call baud/carrier parameters. */
    if (s->rx.received_event == V34_EVENT_INFO1_OK)
    {
        span_log(&s->logging, SPAN_LOG_FLOW,
                 "Tx - V.34 caller: INFO1a received; conditioning for answerer Phase 3\n");
        s->rx.received_event = V34_EVENT_NONE;
        s_not_s_baud_init(s);
        s->tx.phase3_call_wait_j = true;
        s->tx.current_getbaud = get_v34_call_phase3_wait_baud;
    }
    return zero;
}
/*- End of function --------------------------------------------------------*/

static void v34_call_phase3_wait_init(v34_state_t *s)
{
    s->tx.phase3_call_wait_j = false;
    s->tx.current_getbaud = get_v34_call_info1a_wait_baud;
    span_log(&s->logging, SPAN_LOG_FLOW,
             "Tx - V.34 caller: INFO1c complete; silent while receiving INFO1a\n");
}
/*- End of function --------------------------------------------------------*/

static complex_sig_t get_info1_baud(v34_state_t *s)
{
    int bit;

    /* V.90 §8.2.3.1: precede the INFO sequence with one point at an
       arbitrary carrier phase. */
    if (s->tx.v90_mode && s->tx.tone_duration < 1)
    {
        s->tx.tone_duration++;
        return s->tx.lastbit;
    }
    /*endif*/

    bit = get_data_bit(&s->tx);
    if (s->tx.txptr >= s->tx.txbits)
    {
        if (s->tx.calling_party && s->tx.v90_mode)
        {
            /* §8.2.3.1 permits multiple INFO sequences as a group, and the
               digital modem's INFO1d is already repeated four times here for
               exactly that reason: a receiver that switches detectors on the
               first boundary needs a subsequent sync word to acquire on.  The
               analogue modem's INFO1a used to go out once, and a peer that
               starts its INFO receive late never completed a frame -- an Eicon
               Diva Server logs INFO_RX event without complete, over and over,
               against a single INFO1a. */
            if (s->tx.tone_duration < info1a_repeats(s))
            {
                s->tx.tone_duration++;
                s->tx.txptr = 0;
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - V.90: repeating INFO1a contiguously (%d/%d)\n",
                         s->tx.tone_duration, info1a_repeats(s));
            }
            else
            {
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - V.90: INFO1a complete, entering Phase 3 S/!S handoff\n");
                s->tx.tone_duration = 0;
                s_not_s_baud_init(s);
            }
            /*endif*/
        }
        else if (s->tx.calling_party)
        {
            v34_call_phase3_wait_init(s);
        }
        else if (s->tx.v90_mode)
        {
            /* V.90 §9.2.1.1.8: after sending INFO1d, the digital modem
               shall transmit silence and condition its receiver to receive
               INFO1a.  Do NOT call s_not_s_baud_init() which would start
               Phase 3 S/S̄ and overwrite RX state. */
            if (s->tx.tone_duration < 4)
            {
                /* V.90 §8.2.3.1 explicitly permits multiple INFO sequences
                   as a group, with only the first preceded by an arbitrary
                   point.  Repeating INFO1d contiguously gives a receiver that
                   switches from its L2 detector on the first boundary a full
                   subsequent sync word to acquire. */
                s->tx.tone_duration++;
                s->tx.txptr = 0;
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - V.90: repeating INFO1d contiguously (%d/4)\n",
                         s->tx.tone_duration);
            }
            else if (s->tx.stage != V34_TX_STAGE_V90_WAIT_INFO1A)
            {
                s->tx.tone_duration = 0;
                v90_wait_info1a_init(s);
            }
        }
        else
        {
            s_not_s_baud_init(s);
        }
        /*endif*/
    }
    /*endif*/
    if (bit > 0)
        s->tx.lastbit.re = -s->tx.lastbit.re;
    /*endif*/
    return s->tx.lastbit;
}
/*- End of function --------------------------------------------------------*/

/* How many times the analogue modem's INFO1a is repeated as one §8.2.3.1
   group.  Four matches what the digital side already sends for INFO1d;
   ME_V90_ANALOGUE_INFO1A_REPEATS raises it for a peer that needs longer to
   arrive at its INFO receive state. */
static int info1a_repeats(v34_state_t *s)
{
    const char *env;

    (void) s;
    if ((env = getenv("ME_V90_ANALOGUE_INFO1A_REPEATS")) != NULL  &&  *env)
    {
        long n = strtol(env, NULL, 10);

        if (n >= 1  &&  n <= 200)
            return (int) n;
        /*endif*/
    }
    /*endif*/
    return 4;
}
/*- End of function --------------------------------------------------------*/

static void info1_baud_init(v34_state_t *s)
{
    span_log(&s->logging, SPAN_LOG_FLOW, "Tx - info1_baud_init()\n");
    if (s->tx.v90_mode && s->tx.calling_party)
    {
        /* V.90 §8.2.3.2 Table 10: analog (calling) modem sends INFO1a
           with V.90-specific field layout (70 bits with U_INFO). */
        prepare_v90_info1a(s);
        s->tx.txbits = v90_info1a_sequence_tx(&s->tx, &s->tx.info1a);
    }
    else if (s->tx.calling_party || s->tx.v90_mode)
    {
        /* V.34 caller sends INFO1c (109 bits with probing results).
           V.90 §8.2.3.2 Table 9: digital modem (answerer) sends INFO1d
           which is identical to V.34 INFO1c. */
        if (s->tx.v90_mode)
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Tx INFO1d (%s):\n",
                     s->tx.v92_info1d_mode
                         ? "V.92 Table 17" : "V.90 Table 9");
        prepare_info1c(s);
        s->tx.txbits = info1c_sequence_tx(&s->tx, &s->tx.info1c);
        s->tx.txbits += 8;
    }
    else
    {
        prepare_info1a(s);
        s->tx.txbits = info1a_sequence_tx(&s->tx, &s->tx.info1a);
    }
    /*endif*/
    /* Round up to a whole number of bytes */
    s->tx.txbits = (s->tx.txbits + 7) & ~7;
    s->tx.txptr = 0;
    if (s->tx.v90_mode)
    {
        int nbytes = (s->tx.txbits + 7) >> 3;
        char hexbuf[256];
        int pos = 0;
        for (int di = 0;  di < nbytes && pos < (int)sizeof(hexbuf) - 4;  di++)
            pos += snprintf(hexbuf + pos, sizeof(hexbuf) - pos, " %02X", s->tx.txbuf[di]);
        span_log(&s->logging, SPAN_LOG_FLOW, "Tx INFO1d raw frame (%d bits, %d bytes):%s\n", s->tx.txbits, nbytes, hexbuf);
    }
#if 0
#if defined(SPANDSP_USE_FIXED_POINT)
    cvec_zeroi16(s->tx.rrc_filter, sizeof(s->tx.rrc_filter)/sizeof(s->tx.rrc_filter[0]));
#else
    cvec_zerof(s->tx.rrc_filter, sizeof(s->tx.rrc_filter)/sizeof(s->tx.rrc_filter[0]));
#endif
    s->tx.lastbit = complex_sig_set(TRAINING_SCALE(0.0f), TRAINING_SCALE(0.0f));
    s->tx.rrc_filter_step = 0;
    s->tx.baud_phase = 0;
#endif

    s->tx.lastbit = complex_sig_set(TRAINING_SCALE(TRAINING_AMP), TRAINING_SCALE(0.0f));
    s->tx.current_modulator = V34_MODULATION_CC;
    s->tx.stage = V34_TX_STAGE_INFO1;
    s->tx.current_getbaud = get_info1_baud;
    if (s->tx.v90_mode)
    {
        s->tx.tone_duration = 0;
        span_log(&s->logging, SPAN_LOG_FLOW, "Tx - V.90: INFO1d will start with one arbitrary-phase point\n");
    }
    /*endif*/
    if (s->tx.v90_mode  &&  !s->tx.calling_party)
    {
        /* SmartLink answers the *first* decoded INFO1d repetition with a
           single ~140 ms INFO1a, which can arrive while we are still
           transmitting the remaining contiguous repeats.  The INFO1a frame
           accumulator used to be armed only by v90_wait_info1a_init() after
           the last repeat: v90_info1d_sent gates both the cross-Tone-state
           sync search and the target_bits bias (and the bias also needs
           info0_received, which a §9.5 retrain skips), so an early INFO1a
           hit the sync hunter with stale INFO0-era framing and was
           structurally lost.  Observed live 2026-07-22: the retrained flow
           deadlocked whenever the peer's INFO1a landed ~100 ms inside our
           burst (call 9), and only meshed when it happened to land after it
           (call 8).  Arm the receiver at burst start; a mid-burst decode
           runs v90_enter_phase3_from_info1a() synchronously and abandons
           the rest of the burst. */
        s->rx.v90_info1d_sent = true;
        s->rx.target_bits = 70 - (4 + 8 + 4);
        if (s->rx.stage != V34_RX_STAGE_INFO1A  ||  s->rx.bit_count == 0)
        {
            /* Don't clobber an INFO1a that is already mid-accumulation --
               this init also runs on INFO1d retries. */
            s->rx.bit_count = 0;
            s->rx.bitstream = 0;
        }
        /*endif*/
    }
}
/*- End of function --------------------------------------------------------*/

/* Rotate a training point by 180 degrees.  Both components, which is what
   makes it a rotation rather than a reflection about the imaginary axis. */
static void v34_rotate_180(complex_sig_t *p)
{
    p->re = -p->re;
    p->im = -p->im;
}
/*- End of function --------------------------------------------------------*/

static complex_sig_t get_s_not_s_baud(v34_state_t *s)
{
#if defined(SPANDSP_USE_FIXED_POINT)
    int16_t x;
#else
    float x;
#endif
    int silence_bauds;
    int md_bauds;

    /* V.34 11.3.1.2.1 (answer modem): 70 +/- 5 ms silence after INFO1a
       before first S(128T)+S-bar(16T). Caller path keeps legacy timing. */
    silence_bauds = 0;
    if (!s->tx.calling_party)
        silence_bauds = (baud_rate_parameters[s->tx.baud_rate].baud_rate*70 + 500)/1000;
    /*endif*/
    if (silence_bauds < 0)
        silence_bauds = 0;
    /*endif*/

    /* MD is optional and disabled in this implementation.
       Always skip MD waveform transmission. */
    md_bauds = 0;

    switch (s->tx.stage)
    {
    case V34_TX_STAGE_FIRST_S:
        if (++s->tx.tone_duration <= silence_bauds)
            return zero;
        /*endif*/
        if (s->tx.tone_duration == (128 + silence_bauds))
        {
            /* 10.1.3.7: S-bar is S rotated by 180 degrees.  Negating only the
               real part is that rotation *only* where the imaginary part is
               zero, and the alternation puts a zero real part here every time,
               so this was a no-op and S-bar went out identical to S.  The call
               modem's whole Phase 3 hangs off that edge: 11.3.1.1.2 has it
               begin training its equalizer on PP after detecting the
               S-to-S-bar transition, so with no transition on the wire it
               started PP at the wrong offset and its equalizer error pegged at
               full scale from the first reading, on every call. */
            v34_rotate_180(&s->tx.lastbit);
            s->tx.stage = V34_TX_STAGE_FIRST_NOT_S;
            s->tx.tone_duration = 0;
        }
        /*endif*/
        break;
    case V34_TX_STAGE_FIRST_NOT_S:
        if (++s->tx.tone_duration == 16)
        {
            v34_rotate_180(&s->tx.lastbit);
            if (s->tx.duplex  &&  md_bauds > 0)
                s->tx.stage = V34_TX_STAGE_MD;
            else
                pp_baud_init(s);
            /*endif*/
            s->tx.tone_duration = 0;
        }
        /*endif*/
        break;
    case V34_TX_STAGE_MD:
        /* MD waveform is implementation-specific. Use silence placeholder
           with correct duration so Phase 3 timing and sequencing remain
           standards-aligned when MD is non-zero. */
        if (md_bauds <= 0  ||  ++s->tx.tone_duration >= md_bauds)
        {
            s->tx.stage = V34_TX_STAGE_SECOND_S;
            s->tx.tone_duration = 0;
        }
        /*endif*/
        return zero;
    case V34_TX_STAGE_SECOND_S:
        if (++s->tx.tone_duration == 128)
        {
            v34_rotate_180(&s->tx.lastbit);
            s->tx.stage = V34_TX_STAGE_SECOND_NOT_S;
            s->tx.tone_duration = 0;
        }
        /*endif*/
        break;
    case V34_TX_STAGE_SECOND_NOT_S:
        if (++s->tx.tone_duration == 16)
            pp_baud_init(s);
        /*endif*/
        break;
    }
    /*endswitch*/
    x = s->tx.lastbit.re;
    s->tx.lastbit.re = s->tx.lastbit.im;
    s->tx.lastbit.im = x;
    return s->tx.lastbit;
}
/*- End of function --------------------------------------------------------*/

/*! Scale a pre-emphasis coefficient set to unity average power gain.

    V.34/5.4 defines the pre-emphasis filters as *shape* templates: Tables 3 and 4
    give only the tilt across the band (index 0 is flat, index 5 is a 10 dB ramp,
    indices 6 to 10 add beta at the low end rising to beta+gamma at the high end),
    and 10.1.3 then lists "symbol rate, carrier frequency, pre-emphasis filter and
    power level" as four separate attributes of the transmitted signal. Selecting a
    filter therefore must not move the average transmitted power - pre-emphasis
    redistributes energy across the band, it does not add any. The same clause's
    NOTE states the general principle, that the transmitter compensates for
    processing gain so the average signal power is maintained.

    The generated sets in v34_tx_pre_emphasis_filters.h implement the templates
    literally, and because every template has gain >= 1 everywhere in band, each
    one is a pure boost: measured against a flat in-band input, index 5 adds about
    6.5 dB of power and index 10 about 3.2 dB. Left uncompensated that lands on the
    wire on top of the level v34_tx_power() was asked for, and index 5 clipped.

    Normalise over exactly the band 5.4.1 defines the template on,
    S*(d/e - 0.45) to S*(d/e + 0.45), assuming a flat input across it - which is
    what the RRC-shaped V.34 signal approximates. The tables themselves are left
    alone: they are the correct *shape*, and the shape is what the peer asked for. */
static void pre_emphasis_normalise(float out[16],
                                   const float in[16],
                                   int baud_idx,
                                   int carrier_idx)
{
#define PRE_EMPHASIS_NORM_STEPS     256
    double baud;
    double d_over_e;
    double lo;
    double hi;
    double f;
    double w;
    double re;
    double im;
    double sum;
    double scale;
    int i;
    int k;

    baud = baud_rate_parameters[baud_idx].baud_rate;
    d_over_e = (double) baud_rate_parameters[baud_idx].low_high[carrier_idx].d
             / (double) baud_rate_parameters[baud_idx].low_high[carrier_idx].e;
    lo = baud*(d_over_e - 0.45);
    hi = baud*(d_over_e + 0.45);
    sum = 0.0;
    for (i = 0;  i <= PRE_EMPHASIS_NORM_STEPS;  i++)
    {
        f = lo + (hi - lo)*i/PRE_EMPHASIS_NORM_STEPS;
        re = 0.0;
        im = 0.0;
        for (k = 0;  k < 16;  k++)
        {
            w = 2.0*M_PI*f*k/SAMPLE_RATE;
            re += in[k]*cos(w);
            im -= in[k]*sin(w);
        }
        /*endfor*/
        sum += re*re + im*im;
    }
    /*endfor*/
    sum /= (PRE_EMPHASIS_NORM_STEPS + 1);
    /* sum is the mean power gain across the band. A degenerate set would leave it
       at zero; fall back to the unscaled coefficients rather than divide by it. */
    scale = (sum > 0.0)  ?  1.0/sqrt(sum)  :  1.0;
    for (k = 0;  k < 16;  k++)
        out[k] = (float) (in[k]*scale);
    /*endfor*/
#undef PRE_EMPHASIS_NORM_STEPS
}
/*- End of function --------------------------------------------------------*/

static void s_not_s_baud_init(v34_state_t *s)
{
    const char *info1_source;
    int power_reduction;
    int preemp_idx;
    int baud_idx;
    int carrier_idx;

    span_log(&s->logging, SPAN_LOG_FLOW, "Tx - s_not_s_baud_init()\n");
    if (s->tx.v90_v34_fallback  &&  s->rx.info1a_received)
    {
        /* V.90 §9.2.1.1.8 V.34 fallback, call-modem role.  Table 11 dictates
           our transmit configuration: bits 37:39 the digital->analogue symbol
           rate, bit 25 the carrier, bits 26:29 the pre-emphasis, and bits
           12:17 the power reduction (applied below via info1_source). */
        if (s->rx.info1a.baud_rate_c_to_a >= 0  &&  s->rx.info1a.baud_rate_c_to_a <= 5)
        {
            s->tx.baud_rate = s->rx.info1a.baud_rate_c_to_a;
            /* Refresh only the modulator-facing rate parameters; the full
               v34_parameters_t is refilled at data-mode entry. */
            s->tx.parms.samples_per_symbol_numerator = baud_rate_parameters[s->tx.baud_rate].samples_per_symbol_numerator;
            s->tx.parms.samples_per_symbol_denominator = baud_rate_parameters[s->tx.baud_rate].samples_per_symbol_denominator;
            s->tx.parms.max_bit_rate_code = baud_rate_parameters[s->tx.baud_rate].max_bit_rate_code;
        }
        /*endif*/
        s->tx.high_carrier = s->rx.info1a.use_high_carrier;
        s->tx.v34_carrier_phase_rate = dds_phase_ratef(carrier_frequency(s->tx.baud_rate, s->tx.high_carrier));
        span_log(&s->logging, SPAN_LOG_FLOW,
                 "Tx - Phase 3 (V.34 fallback, call role): S/!S at %d baud, %s carrier\n",
                 baud_rate_parameters[s->tx.baud_rate].baud_rate,
                 s->tx.high_carrier ? "high" : "low");
    }
    else if (s->tx.calling_party && !s->tx.v90_mode && s->rx.info1a_received)
    {
        /* V.34 10.1.2.3.5/Table 16 bits 37:39 select call->answer,
           this call modem's transmit direction. */
        if (s->rx.info1a.baud_rate_c_to_a >= V34_BAUD_RATE_2400
            && s->rx.info1a.baud_rate_c_to_a <= V34_BAUD_RATE_3429)
        {
            s->tx.baud_rate = s->rx.info1a.baud_rate_c_to_a;
            s->tx.high_carrier = s->rx.info1a.use_high_carrier;
            s->tx.parms.samples_per_symbol_numerator =
                baud_rate_parameters[s->tx.baud_rate].samples_per_symbol_numerator;
            s->tx.parms.samples_per_symbol_denominator =
                baud_rate_parameters[s->tx.baud_rate].samples_per_symbol_denominator;
            s->tx.v34_carrier_phase_rate =
                dds_phase_ratef(carrier_frequency(s->tx.baud_rate, s->tx.high_carrier));
        }
    }
    else if (!s->tx.calling_party)
    {
        int silence_bauds = (baud_rate_parameters[s->tx.baud_rate].baud_rate*70 + 500)/1000;
        span_log(&s->logging, SPAN_LOG_FLOW,
                 "Tx - Phase 3 (answerer): INFO1a -> silence 70ms (~%d bauds) -> S/!S\n",
                 silence_bauds);
    }
    /*endif*/

    /* Phase 3 transmit power and pre-emphasis are dictated by the *remote* modem,
       in whichever INFO1 message describes this side's transmitter:

         - We are the answerer -> INFO1c (Table 15/V.34, §10.1.2.3.4).  Bits 12:14
           are the "minimum power reduction to be implemented by the answer modem
           transmitter"; the per-symbol-rate 9-bit probing blocks carry the
           pre-emphasis index for the answer->call direction (bits 26:29 for 2400,
           and identically coded fields for the other symbol rates).  §11.2.1.2.9
           is the matching procedure step.

         - We are the caller -> INFO1a (Table 16/V.34, §10.1.2.3.5).  Bits 12:14
           are the "minimum power reduction to be implemented by the call modem
           transmitter" and bits 26:29 the pre-emphasis index for the call->answer
           direction.  §11.2.1.1.8 is the matching procedure step.  Note INFO1a
           carries a single pre-emphasis index, for the already-selected symbol
           rate, rather than one per candidate rate as INFO1c does.

       V.90 is excluded on the calling side: Table 10/V.90 reserves INFO1a bits
       12:17 and 26:29, and process_rx_info1a() parses them as zero, so there is
       nothing to honour and we keep the conservative default instead of reading a
       0 dB reduction out of reserved bits.

       Gate on the *_received flag, not on the value: 0 dB is a legitimate request
       (bits 12:14 are an integer 0-7), and treating it as "no data" transmitted
       3 dB below what the peer asked for on every call where the INFO1 actually
       decoded.  Both flags are only set on a CRC-valid frame — including via the
       boundary/local-slip recovery paths, which re-check the CRC of the recovered
       bits before accepting them. */
    baud_idx = s->tx.baud_rate;
    carrier_idx = s->tx.high_carrier ? 1 : 0;
    if (s->tx.v90_v34_fallback  &&  s->rx.info1a_received)
    {
        /* V.90 Table 11 (V.34-selected INFO1a): bits 12:14 are the minimum
           power reduction for the DIGITAL modem transmitter and 15:17 the
           additional reduction the analogue receiver tolerates.  V.34
           §11.2.1.1.8 lets the call modem apply anywhere in [min, min+add];
           apply the full amount -- the analogue modem asked for it (the
           live CX93001 requests 7+1 dB). */
        info1_source = "INFO1a (V.90->V.34 fallback)";
        power_reduction = s->rx.info1a.power_reduction + s->rx.info1a.additional_power_reduction;
        if (power_reduction > 14)
            power_reduction = 14;
        /*endif*/
        preemp_idx = s->rx.info1a.preemphasis_filter;
    }
    else if (s->tx.calling_party  &&  s->tx.v90_mode)
    {
        /* V.90 analogue role.  §8.2.3.2 Table 9 makes INFO1d identical to
           INFO1c, and V.34 §10.1.2.3.4 has INFO1c describe the *answer*
           modem's transmitter -- which in V.90 is this one, because
           §9.2.2.1.9 gives the analogue modem the V.34 answer-modem role.
           So INFO1d governs our Phase 3 transmit: power reduction, and the
           per-symbol-rate block's pre-emphasis index and carrier.

           This used to fall through to the "no INFO1 received" default even
           though INFO1d had decoded, so every analogue-role Phase 3 went out
           3 dB down, unemphasised, and -- worse -- on whichever carrier
           Phase 2 happened to leave set.  An Eicon Diva Server asks for the
           *low* carrier at 3200 baud; transmitting the high one puts the
           upstream 91 Hz off (1920 Hz against the 1829 Hz its receiver is
           tuned to), which no carrier recovery will pull in.  Loopback
           cannot catch it: our own receiver is configured from the same
           variable, so it demodulates a wrong carrier perfectly. */
        info1_source = (s->rx.info1c_received)  ?  "INFO1d"  :  NULL;
        if (info1_source)
        {
            power_reduction = s->rx.info1c.power_reduction;
            preemp_idx = s->rx.info1c.rate_data[baud_idx].pre_emphasis;
            if (s->tx.high_carrier != s->rx.info1c.rate_data[baud_idx].use_high_carrier)
            {
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - Phase 3: INFO1d selects the %s carrier at %d baud "
                         "(was %s); retuning to %.0f Hz\n",
                         s->rx.info1c.rate_data[baud_idx].use_high_carrier ? "high" : "low",
                         baud_rate_parameters[baud_idx].baud_rate,
                         s->tx.high_carrier ? "high" : "low",
                         carrier_frequency(s->tx.baud_rate,
                                           s->rx.info1c.rate_data[baud_idx].use_high_carrier));
                s->tx.high_carrier = s->rx.info1c.rate_data[baud_idx].use_high_carrier;
                carrier_idx = s->tx.high_carrier ? 1 : 0;
                s->tx.v34_carrier_phase_rate =
                    dds_phase_ratef(carrier_frequency(s->tx.baud_rate, s->tx.high_carrier));
            }
            /*endif*/
        }
        /*endif*/
    }
    else if (s->tx.calling_party)
    {
        info1_source = (s->rx.info1a_received)  ?  "INFO1a"  :  NULL;
        if (info1_source)
        {
            power_reduction = s->rx.info1a.power_reduction;
            preemp_idx = s->rx.info1a.preemphasis_filter;
        }
        /*endif*/
    }
    else
    {
        info1_source = (s->rx.info1c_received)  ?  "INFO1c"  :  NULL;
        if (info1_source)
        {
            power_reduction = s->rx.info1c.power_reduction;
            preemp_idx = s->rx.info1c.rate_data[baud_idx].pre_emphasis;
        }
        /*endif*/
    }
    /*endif*/
    if (info1_source == NULL)
    {
        /* Safe default when the governing INFO1 was not received: back off 3 dB
           rather than risk overdriving the remote receiver, and do not pre-emphasise. */
        power_reduction = 3;
        preemp_idx = 0;
    }
    /*endif*/
    {
        /* Diagnostic override for the Phase 3 transmit shaping, so a peer whose
           equalizer will not converge can be measured against a flat spectrum
           without inventing an INFO1.  Not a policy knob: unset, the value the
           governing INFO1 asked for is used, which is what the spec requires. */
        const char *env = getenv("ME_V34_TX_PREEMP");

        if (env  &&  *env)
        {
            int forced = atoi(env);

            if (forced >= 0  &&  forced <= 10)
            {
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - Phase 3: ME_V34_TX_PREEMP overrides pre-emphasis %d -> %d\n",
                         preemp_idx, forced);
                preemp_idx = forced;
            }
            /*endif*/
        }
        /*endif*/
    }
    v34_tx_power(s, -14.0f - (float)power_reduction);
    span_log(&s->logging, SPAN_LOG_FLOW,
             "Tx - Phase 3: applying %d dB power reduction (%.1f dBm0) [from %s]\n",
             power_reduction, -14.0f - (float)power_reduction,
             (info1_source)  ?  info1_source  :  "default, no INFO1 received");

    /* Pre-emphasis filter (V.34/5.4). Index 0 = no pre-emphasis, 1-10 = filter. */
    s->tx.pre_emphasis_coeffs = NULL;
    memset(s->tx.pre_emphasis_buf, 0, sizeof(s->tx.pre_emphasis_buf));
    s->tx.pre_emphasis_idx = 0;
    if (preemp_idx >= 1  &&  preemp_idx <= 10)
    {
        pre_emphasis_normalise(s->tx.pre_emphasis_norm_coeffs,
                               v34_tx_pre_emphasis_filters[baud_idx][carrier_idx][preemp_idx - 1],
                               baud_idx,
                               carrier_idx);
        s->tx.pre_emphasis_coeffs = s->tx.pre_emphasis_norm_coeffs;
        span_log(&s->logging, SPAN_LOG_FLOW,
                 "Tx - Phase 3: applying pre-emphasis filter %d (baud %d, %s carrier), "
                 "normalised to unity band power\n",
                 preemp_idx, baud_rate_parameters[baud_idx].baud_rate,
                 carrier_idx ? "high" : "low");
    }
    else
    {
        span_log(&s->logging, SPAN_LOG_FLOW,
                 "Tx - Phase 3: no pre-emphasis (index %d)\n", preemp_idx);
    }
    /*endif*/

    span_log(&s->logging, SPAN_LOG_FLOW,
             "Tx - Phase 3: TX carrier=%s (%.0f Hz), RX carrier=%s (%.0f Hz)\n",
             s->tx.high_carrier ? "high" : "low",
             carrier_frequency(s->tx.baud_rate, s->tx.high_carrier),
             s->rx.high_carrier ? "high" : "low",
             carrier_frequency(s->rx.baud_rate, s->rx.high_carrier));

    /* 10.1.3.7 requires S to "end with the transmission of point 0 rotated
       counterclockwise by 90 degrees", which this seed already delivers: the
       swap in get_s_not_s_baud() runs after the stage logic, so the symbols
       returned alternate 90, 0, 90, ... and the last one before the S-bar
       rotation is a 90 degree point. */
    s->tx.lastbit = complex_sig_set(TRAINING_SCALE(TRAINING_AMP), TRAINING_SCALE(0.0f));
    s->tx.tone_duration = 0;
    s->tx.current_modulator = V34_MODULATION_V34;
    s->tx.stage = V34_TX_STAGE_FIRST_S;
    s->tx.current_getbaud = get_s_not_s_baud;
    /* Reset baud phase and RRC filter for V.34 modulator. The previous CC modulator
       used different num/den (40/3 for 600 baud), so baud_phase could be invalid.
       Also flush the RRC filter buffer to avoid stale CC modulator data. */
    s->tx.baud_phase = 0;
    s->tx.rrc_filter_step = 0;
    memset(s->tx.rrc_filter_re, 0, sizeof(s->tx.rrc_filter_re));
    memset(s->tx.rrc_filter_im, 0, sizeof(s->tx.rrc_filter_im));

    /* Switch RX to primary channel demodulator for Phase 3 reception.
       Acquire the far-end PP immediately, then refine on the first 512T of TRN.
       J detection is re-armed later when local TRN completes.

       V.90 exception (§9.2.1.1.8): the digital modem must receive INFO1a
       from the analog modem before proceeding to Phase 3.  Keep the CC
       demodulator active and the RX stage at INFO1A so that info_rx()
       can decode INFO1a.  The transition to PHASE3_TRAINING happens later
       in process_rx_info1a() once INFO1a is received. */
    if (s->tx.v90_mode && s->tx.calling_party)
    {
        /* V.90 caller has already finished INFO1a transmission at this point.
           Unlike the digital answerer, it should now switch RX straight to the
           primary channel for Phase 3 training rather than continuing to wait
           on the control-channel INFO path. */
        s->rx.current_demodulator = V34_MODULATION_V34;
        s->rx.stage = V34_RX_STAGE_PHASE3_TRAINING;
    }
    else if (s->tx.v90_mode)
    {
        if (s->rx.stage >= V34_RX_STAGE_PHASE3_TRAINING
            || s->rx.stage == V34_RX_STAGE_INFO1A
            || s->rx.info1a.max_data_rate > 0)
        {
            /* RX already decoded INFO1a, or has already moved into primary-
               channel training. Preserve that live V.90 Phase 3 receive
               context instead of regressing back to Tone B while TX enters
               S/!S. */
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Tx - V.90: INFO1a already decoded, preserving primary-channel RX across S/!S handoff\n");
            s->rx.current_demodulator = V34_MODULATION_V34;
            if (s->rx.stage < V34_RX_STAGE_PHASE3_TRAINING)
                s->rx.stage = V34_RX_STAGE_PHASE3_TRAINING;
        }
        else
        {
            s->rx.current_demodulator = V34_MODULATION_TONES;
            /* RX hasn't reached INFO1a yet — stay on TONE_B and wait
               for reversals to trigger the natural transition. */
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Tx - V.90: keeping RX on CC demod (TONE_B) for INFO1a reception\n");
            s->rx.stage = V34_RX_STAGE_TONE_B;
            s->rx.received_event = V34_EVENT_REVERSAL_1;
            s->rx.persistence1 = 0;
            s->rx.persistence2 = 0;
        }
    }
    else
    {
        s->rx.current_demodulator = V34_MODULATION_V34;
        s->rx.stage = V34_RX_STAGE_PHASE3_TRAINING;
    }
    s->rx.duration = 0;
    s->rx.bit_count = 0;
    s->rx.s_detect_count = 0;
    s->rx.s_window = 0;
    s->rx.phase3_s_alt_window = 0;
    s->rx.phase3_s_alt_count = 0;
    s->rx.phase3_s_stable_windows = 0;
    s->rx.phase3_s_guard_samples = 4000;
    s->rx.phase3_s_hits = 0;
    memset(s->rx.phase3_s_ring, 0, sizeof(s->rx.phase3_s_ring));
    memset(s->rx.phase3_s_counts, 0, sizeof(s->rx.phase3_s_counts));
    s->rx.phase3_s_pos = 0;
    memset(s->rx.phase3_pp_lag8, 0, sizeof(s->rx.phase3_pp_lag8));
    s->rx.phase3_pp_obs = 0;
    s->rx.phase3_pp_match = 0;
    memset(s->rx.phase3_pp_error, 0, sizeof(s->rx.phase3_pp_error));
    memset(s->rx.phase3_pp_corr, 0, sizeof(s->rx.phase3_pp_corr));
    s->rx.phase3_pp_corr_energy = 0.0f;
    s->rx.phase3_pp_corr_weight = 0.0f;
    s->rx.phase3_pp_rotation.re = 1.0f;
    s->rx.phase3_pp_rotation.im = 0.0f;
    s->rx.phase3_pp_phase = -1;
    s->rx.phase3_pp_phase_score = -1;
    s->rx.phase3_pp_acquire_hits = 0;
    s->rx.phase3_pp_started = 0;
    memset(s->rx.phase3_j_scramble, 0, sizeof(s->rx.phase3_j_scramble));
    memset(s->rx.phase3_j_stream, 0, sizeof(s->rx.phase3_j_stream));
    memset(s->rx.phase3_j_prev_z, 0, sizeof(s->rx.phase3_j_prev_z));
    memset(s->rx.phase3_j_prev_valid, 0, sizeof(s->rx.phase3_j_prev_valid));
    memset(s->rx.phase3_j_win, 0, sizeof(s->rx.phase3_j_win));
    s->rx.phase3_j_bits = 0;
    s->rx.phase3_j_lock_hyp = -1;
    s->rx.phase3_j_trn16 = -1;
    s->rx.phase3_j_candidate_hyp = -1;
    s->rx.phase3_j_candidate_phase = -1;
    s->rx.phase3_j_candidate_pat = -1;
    s->rx.phase3_j_candidate_count = 0;
    s->rx.phase3_j_candidate_last_bits = 0;
    memset(s->rx.phase3_trn_scramble, 0, sizeof(s->rx.phase3_trn_scramble));
    memset(s->rx.phase3_trn_one_count, 0, sizeof(s->rx.phase3_trn_one_count));
    s->rx.phase3_trn_bits = 0;
    s->rx.phase3_trn_lock_hyp = -1;
    s->rx.phase3_trn_lock_score = -1;
    s->rx.phase4_j_seen = 0;
    s->rx.phase4_j_lock_hyp = -1;
    s->rx.phase4_trn_after_j = 0;
    s->rx.phase4_j_bits = 0;
    memset(s->rx.phase4_j_scramble_tap, 0, sizeof(s->rx.phase4_j_scramble_tap));
    memset(s->rx.phase4_j_stream_tap, 0, sizeof(s->rx.phase4_j_stream_tap));
    memset(s->rx.phase4_j_prev_z_tap, 0, sizeof(s->rx.phase4_j_prev_z_tap));
    memset(s->rx.phase4_j_prev_valid_tap, 0, sizeof(s->rx.phase4_j_prev_valid_tap));
    memset(s->rx.phase4_j_win_tap, 0, sizeof(s->rx.phase4_j_win_tap));
    memset(s->rx.phase4_trn_scramble_tap, 0, sizeof(s->rx.phase4_trn_scramble_tap));
    memset(s->rx.phase4_trn_one_count_tap, 0, sizeof(s->rx.phase4_trn_one_count_tap));
    memset(s->rx.phase4_trn_scramble, 0, sizeof(s->rx.phase4_trn_scramble));
    memset(s->rx.phase4_trn_prev_z, 0, sizeof(s->rx.phase4_trn_prev_z));
    memset(s->rx.phase4_trn_prev_valid, 0, sizeof(s->rx.phase4_trn_prev_valid));
    memset(s->rx.phase4_trn_one_count, 0, sizeof(s->rx.phase4_trn_one_count));
    s->rx.phase4_trn_lock_hyp = -1;
    s->rx.phase4_trn_lock_score = -1;
    s->rx.phase4_trn_lock_tap = -1;
    s->rx.phase4_trn_lock_order = -1;
    s->rx.phase4_trn_lock_domain = -1;
    s->rx.phase4_trn_current_hyp = -1;
    s->rx.phase4_trn_current_score = -1;
    s->rx.phase4_trn_current_tap = -1;
    s->rx.phase4_trn_current_order = -1;
    s->rx.phase4_trn_current_domain = -1;
    s->rx.mp_phase4_bit_order = 0;
    s->rx.mp_phase4_default_bit_order = 0;
    s->rx.mp_phase4_alt_order_active = 0;
    s->rx.mp_phase4_retry_mode = 0;
    s->rx.received_event = V34_EVENT_NONE;
    reset_primary_rx_frontend_for_phase3(s);
    span_log(&s->logging, SPAN_LOG_FLOW,
             "Rx - Phase 3: primary demod active; acquiring far-end PP for equalizer conditioning\n");
}
/*- End of function --------------------------------------------------------*/

static complex_sig_t get_pp_baud(v34_state_t *s)
{
    complex_sig_t x;
    int i;

    /* The 48 symbol PP signal, which is repeated 6 times, to make a 288 symbol sequence */
    /* See V.34/10.1.3.6 */
    i = s->tx.tone_duration%PP_PERIOD_SYMBOLS;
    if (++s->tx.tone_duration == 1)
    {
        span_log(&s->logging, SPAN_LOG_FLOW,
                 "Tx - Phase 3: PP transmission started (%d symbols)\n",
                 PP_TOTAL_SYMBOLS);
    }
    if (s->tx.tone_duration == PP_TOTAL_SYMBOLS)
    {
        span_log(&s->logging, SPAN_LOG_FLOW,
                 "Tx - Phase 3: PP transmission complete (%d symbols), starting TRN\n",
                 s->tx.tone_duration);
        trn_baud_init(s);
    }
    /*endif*/
    x = pp_symbols[i];
    x.re *= TRAINING_SCALE(TRAINING_AMP);
    x.im *= TRAINING_SCALE(TRAINING_AMP);
    return x;
}
/*- End of function --------------------------------------------------------*/

static void pp_baud_init(v34_state_t *s)
{
    span_log(&s->logging, SPAN_LOG_FLOW,
             "Tx - pp_baud_init() [Phase 3 PP: %d-symbol sequence]\n",
             PP_TOTAL_SYMBOLS);
    s->tx.tone_duration = 0;
    s->tx.current_getbaud = get_pp_baud;
}
/*- End of function --------------------------------------------------------*/

static complex_sig_t get_trn_baud(v34_state_t *s)
{
    /* J pattern per V.34 §10.1.3.3, Table 18.
       4-point:  "0000100110010001" (left-most bit first in time)
       16-point: "0000110110010001" (left-most bit first in time)
       Since persistence2 is shifted out LSB-first, the left-most (first transmitted)
       bit is stored at the LSB.  Reading "0000100110010001" into a uint16_t with
       position 0 at bit 0 gives 0x8990. */
    static const uint16_t j_pattern[2] =
    {
        0x8990, /* 4 point constellation */
        0x89B0  /* 16 point constellation */
    };
    /* J' pattern per V.34 §10.1.3.4, Table 19.
       "1111100110010001" (left-most bit first in time) = 0x899F.
       J' is the same pattern for all constellation sizes. */
    static const uint16_t j_dashed_pattern[2] =
    {
        0x899F, /* 4 point constellation */
        0x899F  /* 16 point constellation (same pattern per Table 19) */
    };
    /* Interop guard for weak/noisy Phase 3 signaling.
       Spec allows "up to 500 ms" after receiving J, but in the field we can
       miss/late-detect J at this boundary. Use a larger hold-off before
       fallback to avoid transitioning too early. */
    static const int baud_rate_hz[6] = {2400, 2743, 2800, 3000, 3200, 3429};
    int j_wait_max_bauds[6];
    int bit;
    int trn_i;
    int trn_q;
    int trn_sym;
    int j_wait_ms;
    int i;

    /* V.34 11.3.1.2.6: the answer modem stays in J until it detects the call
       modem's Phase 3 transition.  This bound is only an interop escape for a
       peer whose transition we never detect, so it must be longer than any
       legitimate call-modem Phase 3 (S, S-bar, PP, 2048T of TRN and J) plus
       the time that modem needs to detect our own J.  A 1000 ms bound was
       shorter than that at every symbol rate, so it fired on the normal path
       and raced the answerer into Phase 4 while the caller was still in TRN. */
    j_wait_ms = getenv("V34_J_WAIT_MAX_MS") ? atoi(getenv("V34_J_WAIT_MAX_MS")) : 4000;
    for (i = 0;  i < 6;  i++)
        j_wait_max_bauds[i] = (baud_rate_hz[i]*j_wait_ms + 999)/1000;
    /*endfor*/
    int j_pat_idx;

    /* See V.34/10.1.3.8 */
    bit = 0;
    trn_i = 0;
    trn_q = 0;
    trn_sym = 0;
    /* V.34 §10.1.3.3 Table 18: J pattern depends on TRN constellation size. */
    j_pat_idx = s->tx.infoh.trn16 ? 1 : 0;
    switch (s->tx.stage)
    {
    case V34_TX_STAGE_TRN:
        /* Send the TRN signal (V.34 §10.1.3.8).
           TRN uses direct mapping (no differential encoding).
           4-point TRN:
             I_n = 2*I2_n + I1_n, transmitted point = point 0 rotated by I_n*90°.
           16-point TRN:
             Q_n = 2*Q2_n + Q1_n selects quarter-superconstellation point,
             then rotate by I_n*90°, where I_n = 2*I2_n + I1_n. */
        trn_i = scramble(&s->tx, 1);
        trn_i = (scramble(&s->tx, 1) << 1) | trn_i;
        if (s->tx.infoh.trn16)
        {
            trn_q = scramble(&s->tx, 1);
            trn_q = (scramble(&s->tx, 1) << 1) | trn_q;
            trn_sym = (trn_q << 2) | trn_i;
        }
        else
        {
            trn_sym = trn_i;
        }
        /*endif*/
        /* In half-duplex modem the length of the training comes from the INFOh message, in 35ms increments.
           In full-duplex, send enough TRN for the remote equalizer to converge before
           the J pattern starts. 2048 bauds (~597ms at 3429 baud) is standard. */
        if ((!s->tx.duplex  &&  ++s->tx.tone_duration >= s->rx.infoh.length_of_trn*35*s->rx.infoh.baud_rate/1000)
            ||
            (s->tx.duplex  &&  ++s->tx.tone_duration >= 2048))
        {
            span_log(&s->logging, SPAN_LOG_FLOW, "Tx - TRN complete (%d bauds), starting J\n",
                     s->tx.tone_duration);
            s->tx.stage = V34_TX_STAGE_J;
            s->tx.persistence2 = j_pattern[j_pat_idx];
            s->tx.tone_duration = 0;
            /* V.34 §10.1.3.3: "The differential encoder shall be initialized
               using the final symbol of the transmitted TRN sequence." */
            s->tx.diff = trn_i;
            /* Clear any stale S detection event (e.g. from timeout during TRN).
               The caller can't send S until it detects our J, so any event
               from before J is spurious. Reset RX to wait for the real S.
               Must also reset stage to PHASE3_WAIT_S so the S detection code
               runs during J — otherwise it can stay at PHASE3_TRAINING and
               miss the early J/S crossover.

               For native V.90 caller/answerer continuation we still want this
               arming step once local TRN is complete, but only if RX has not
               already advanced beyond the Phase 3 conditioning stage. */
            if (!s->tx.v90_mode
                || s->rx.stage < V34_RX_STAGE_PHASE3_WAIT_S)
            {
                s->rx.received_event = V34_EVENT_NONE;
                s->rx.stage = V34_RX_STAGE_PHASE3_WAIT_S;
                s->rx.duration = 0;
                s->rx.bit_count = 0;
                s->rx.s_detect_count = 0;
                s->rx.s_window = 0;
                s->rx.phase3_s_alt_window = 0;
                s->rx.phase3_s_alt_count = 0;
                s->rx.phase3_s_stable_windows = 0;
                s->rx.phase3_s_present = false;
                s->rx.phase3_s_detect_armed = true;
                s->rx.phase3_s_dom_windows = 0;
                s->rx.phase3_s_dom_symbol = -1;
                s->rx.phase3_s_fired_symbol = -1;
                s->rx.phase3_s_guard_samples = 4000;
                s->rx.phase3_s_hits = 0;
                memset(s->rx.phase3_s_ring, 0, sizeof(s->rx.phase3_s_ring));
                memset(s->rx.phase3_s_counts, 0, sizeof(s->rx.phase3_s_counts));
                s->rx.phase3_s_pos = 0;
                memset(s->rx.phase3_pp_lag8, 0, sizeof(s->rx.phase3_pp_lag8));
                s->rx.phase3_pp_obs = 0;
                s->rx.phase3_pp_match = 0;
                memset(s->rx.phase3_pp_error, 0, sizeof(s->rx.phase3_pp_error));
                memset(s->rx.phase3_pp_corr, 0, sizeof(s->rx.phase3_pp_corr));
                s->rx.phase3_pp_corr_energy = 0.0f;
                s->rx.phase3_pp_corr_weight = 0.0f;
                s->rx.phase3_pp_rotation.re = 1.0f;
                s->rx.phase3_pp_rotation.im = 0.0f;
                s->rx.phase3_pp_phase = -1;
                s->rx.phase3_pp_phase_score = -1;
                s->rx.phase3_pp_acquire_hits = 0;
                s->rx.phase3_pp_started = 0;
                memset(s->rx.phase3_j_scramble, 0, sizeof(s->rx.phase3_j_scramble));
                memset(s->rx.phase3_j_stream, 0, sizeof(s->rx.phase3_j_stream));
                memset(s->rx.phase3_j_prev_z, 0, sizeof(s->rx.phase3_j_prev_z));
                memset(s->rx.phase3_j_prev_valid, 0, sizeof(s->rx.phase3_j_prev_valid));
                memset(s->rx.phase3_j_win, 0, sizeof(s->rx.phase3_j_win));
                s->rx.phase3_j_bits = 0;
                if (!s->tx.calling_party)
                {
                    s->rx.phase3_j_lock_hyp = -1;
                    s->rx.phase3_j_trn16 = -1;
                }
                /* The call modem already decoded the answerer's J in
                   11.3.1.1.3.  Preserve that TRN mode across its own
                   PP/TRN/J transmission so 11.3.1.1.7 can send J'. */
                s->rx.phase3_j_candidate_hyp = -1;
                s->rx.phase3_j_candidate_phase = -1;
                s->rx.phase3_j_candidate_pat = -1;
                s->rx.phase3_j_candidate_count = 0;
                s->rx.phase3_j_candidate_last_bits = 0;
                memset(s->rx.phase3_trn_scramble, 0, sizeof(s->rx.phase3_trn_scramble));
                memset(s->rx.phase3_trn_one_count, 0, sizeof(s->rx.phase3_trn_one_count));
                s->rx.phase3_trn_bits = 0;
                s->rx.phase3_trn_lock_hyp = -1;
                s->rx.phase3_trn_lock_score = -1;
                s->rx.phase4_j_seen = 0;
                s->rx.phase4_j_lock_hyp = -1;
                s->rx.phase4_trn_after_j = 0;
                s->rx.phase4_j_bits = 0;
                memset(s->rx.phase4_j_scramble_tap, 0, sizeof(s->rx.phase4_j_scramble_tap));
                memset(s->rx.phase4_j_stream_tap, 0, sizeof(s->rx.phase4_j_stream_tap));
                memset(s->rx.phase4_j_prev_z_tap, 0, sizeof(s->rx.phase4_j_prev_z_tap));
                memset(s->rx.phase4_j_prev_valid_tap, 0, sizeof(s->rx.phase4_j_prev_valid_tap));
                memset(s->rx.phase4_j_win_tap, 0, sizeof(s->rx.phase4_j_win_tap));
                memset(s->rx.phase4_trn_scramble_tap, 0, sizeof(s->rx.phase4_trn_scramble_tap));
                memset(s->rx.phase4_trn_one_count_tap, 0, sizeof(s->rx.phase4_trn_one_count_tap));
                memset(s->rx.phase4_trn_scramble, 0, sizeof(s->rx.phase4_trn_scramble));
                memset(s->rx.phase4_trn_prev_z, 0, sizeof(s->rx.phase4_trn_prev_z));
                memset(s->rx.phase4_trn_prev_valid, 0, sizeof(s->rx.phase4_trn_prev_valid));
                memset(s->rx.phase4_trn_one_count, 0, sizeof(s->rx.phase4_trn_one_count));
                s->rx.phase4_trn_lock_hyp = -1;
                s->rx.phase4_trn_lock_score = -1;
                s->rx.phase4_trn_lock_tap = -1;
                s->rx.phase4_trn_lock_order = -1;
                s->rx.phase4_trn_lock_domain = -1;
                s->rx.phase4_trn_current_hyp = -1;
                s->rx.phase4_trn_current_score = -1;
                s->rx.phase4_trn_current_tap = -1;
                s->rx.phase4_trn_current_order = -1;
                s->rx.phase4_trn_current_domain = -1;
                s->rx.mp_phase4_bit_order = 0;
                s->rx.mp_phase4_default_bit_order = 0;
                s->rx.mp_phase4_alt_order_active = 0;
                s->rx.mp_phase4_retry_mode = 0;
            }
            /*endif*/
        }
        /*endif*/
        return s->tx.infoh.trn16 ? training_constellation_16[trn_sym]
                                 : training_constellation_4[trn_sym];
    case V34_TX_STAGE_J:
        /* Send the J signal (V.34 §10.1.3.3).
           J uses DIFFERENTIAL encoding unlike TRN:
           I_n = 2*I2_n + I1_n, Z_n = (I_n + Z_{n-1}) mod 4,
           transmitted point = point 0 rotated by Z_n*90°. */
        bit = scramble(&s->tx, (s->tx.persistence2 & 1));
        s->tx.persistence2 >>= 1;
        bit = (scramble(&s->tx, (s->tx.persistence2 & 1)) << 1) | bit;
        s->tx.persistence2 >>= 1;
        /* Apply differential encoding per V.34 §10.1.3.3 */
        s->tx.diff = (s->tx.diff + bit) & 3;
        bit = s->tx.diff;
        /* Reload J pattern when all 16 bits are consumed (every 8 bauds for 4-point,
           every 4 bauds for 16-point). */
        if (s->tx.persistence2 == 0)
            s->tx.persistence2 = j_pattern[j_pat_idx];
        /*endif*/
        if (++s->tx.tone_duration >= 16
            &&
            (s->tx.tone_duration % 16) == 0)
        {
            if (s->tx.duplex)
            {
                if ((s->tx.calling_party  ||  s->tx.v90_v34_fallback)
                    &&
                    s->rx.received_event == V34_EVENT_S)
                {
                    if (s->rx.phase3_j_trn16 >= 0)
                    {
                        /* Caller: terminate J only after far-end J is decoded so
                           MP type (4-point/16-point) is known from J per spec. */
                        span_log(&s->logging, SPAN_LOG_FLOW,
                                 "Tx - far-end S detected and J decoded (trn=%s), switching J -> J'\n",
                                 s->rx.phase3_j_trn16 ? "16-point" : "4-point");
                        s->tx.stage = V34_TX_STAGE_J_DASHED;
                        s->tx.persistence2 = j_dashed_pattern[0];
                        s->tx.tone_duration = 0;
                    }
                    else if ((s->tx.tone_duration % 64) == 0)
                    {
                        span_log(&s->logging, SPAN_LOG_FLOW,
                                 "Tx - far-end S detected, waiting for explicit J decode before J' (J bits=%d)\n",
                                 s->rx.phase3_j_bits);
                    }
                    /*endif*/
                }
                else if (!s->tx.calling_party
                         &&
                         !s->tx.v90_v34_fallback
                         &&
                         (s->rx.received_event == V34_EVENT_J
                          ||  s->rx.received_event == V34_EVENT_J_DASHED
                          ||  s->rx.received_event == V34_EVENT_S))
                {
                    int md_units;
                    int md_wait_samples;

                    md_units = s->rx.info1c.md;
                    md_wait_samples = (md_units*35*8000 + 500)/1000;

                    /* Answerer Phase 3 (11.3.1.2.4): after first detected
                       S-transition, send silence and (if MD is indicated)
                       wait MD duration while conditioning for the next
                       S-transition. */
                    if (!s->tx.v90_mode
                        && s->rx.received_event == V34_EVENT_S
                        && md_units == 0)
                    {
                        /* V.34 11.3.1.2.4-.6: after the caller's S/S-bar,
                           stop J, train the receive equalizer on PP/TRN and
                           remain silent until the caller's J is received. */
                        v34_answer_phase3_wait_j_init(s);
                        break;
                    }
                    else if (!s->tx.v90_mode
                        &&
                        s->rx.received_event == V34_EVENT_S
                        &&
                        md_units > 0
                        &&
                        s->rx.phase3_s_hits == 0)
                    {
                        s->rx.phase3_s_hits = 1;
                        s->rx.received_event = V34_EVENT_NONE;
                        s->rx.stage = V34_RX_STAGE_PHASE3_WAIT_S;
                        s->rx.duration = 0;
                        s->rx.bit_count = 0;
                        s->rx.phase3_s_guard_samples = md_wait_samples;
                        s->tx.persistence2 = j_pattern[j_pat_idx];
                        s->tx.tone_duration = 0;
                        memset(s->rx.phase3_j_scramble, 0, sizeof(s->rx.phase3_j_scramble));
                        memset(s->rx.phase3_j_stream, 0, sizeof(s->rx.phase3_j_stream));
                        memset(s->rx.phase3_j_prev_z, 0, sizeof(s->rx.phase3_j_prev_z));
                        memset(s->rx.phase3_j_prev_valid, 0, sizeof(s->rx.phase3_j_prev_valid));
                        memset(s->rx.phase3_j_win, 0, sizeof(s->rx.phase3_j_win));
                        s->rx.phase3_j_bits = 0;
                        s->rx.phase3_j_lock_hyp = -1;
                        s->rx.phase3_j_trn16 = -1;
                        s->rx.phase3_j_candidate_hyp = -1;
                        s->rx.phase3_j_candidate_phase = -1;
                        s->rx.phase3_j_candidate_pat = -1;
                        s->rx.phase3_j_candidate_count = 0;
                        s->rx.phase3_j_candidate_last_bits = 0;
                        memset(s->rx.phase3_trn_scramble, 0, sizeof(s->rx.phase3_trn_scramble));
                        memset(s->rx.phase3_trn_one_count, 0, sizeof(s->rx.phase3_trn_one_count));
                        s->rx.phase3_trn_bits = 0;
                        s->rx.phase3_trn_lock_hyp = -1;
                        s->rx.phase3_trn_lock_score = -1;
                        s->rx.phase4_j_seen = 0;
                        s->rx.phase4_j_lock_hyp = -1;
                        s->rx.phase4_trn_after_j = 0;
                        s->rx.phase4_j_bits = 0;
                        memset(s->rx.phase4_j_scramble_tap, 0, sizeof(s->rx.phase4_j_scramble_tap));
                        memset(s->rx.phase4_j_stream_tap, 0, sizeof(s->rx.phase4_j_stream_tap));
                        memset(s->rx.phase4_j_prev_z_tap, 0, sizeof(s->rx.phase4_j_prev_z_tap));
                        memset(s->rx.phase4_j_prev_valid_tap, 0, sizeof(s->rx.phase4_j_prev_valid_tap));
                        memset(s->rx.phase4_j_win_tap, 0, sizeof(s->rx.phase4_j_win_tap));
                        memset(s->rx.phase4_trn_scramble_tap, 0, sizeof(s->rx.phase4_trn_scramble_tap));
                        memset(s->rx.phase4_trn_one_count_tap, 0, sizeof(s->rx.phase4_trn_one_count_tap));
                        memset(s->rx.phase4_trn_scramble, 0, sizeof(s->rx.phase4_trn_scramble));
                        memset(s->rx.phase4_trn_prev_z, 0, sizeof(s->rx.phase4_trn_prev_z));
                        memset(s->rx.phase4_trn_prev_valid, 0, sizeof(s->rx.phase4_trn_prev_valid));
                        memset(s->rx.phase4_trn_one_count, 0, sizeof(s->rx.phase4_trn_one_count));
                        s->rx.phase4_trn_lock_hyp = -1;
                        s->rx.phase4_trn_lock_score = -1;
                        s->rx.phase4_trn_lock_tap = -1;
                        s->rx.phase4_trn_lock_order = -1;
                        s->rx.phase4_trn_lock_domain = -1;
                        s->rx.phase4_trn_current_hyp = -1;
                        s->rx.phase4_trn_current_score = -1;
                        s->rx.phase4_trn_current_tap = -1;
                        s->rx.phase4_trn_current_order = -1;
                        s->rx.phase4_trn_current_domain = -1;
                        s->rx.mp_phase4_bit_order = 0;
                        s->rx.mp_phase4_default_bit_order = 0;
                        s->rx.mp_phase4_alt_order_active = 0;
                        s->rx.mp_phase4_retry_mode = 0;
                        span_log(&s->logging, SPAN_LOG_FLOW,
                                 "Tx - Phase 3: first S transition seen, MD indicated (%d x35ms); "
                                 "waiting %d samples for next S transition\n",
                                 md_units, md_wait_samples);
                        break;
                    }
                    /*endif*/

                    span_log(&s->logging, SPAN_LOG_FLOW,
                             "Tx - far-end %s detected, starting Phase 4 wait\n",
                             (s->rx.received_event == V34_EVENT_J_DASHED)
                                ? "J'"
                                : ((s->rx.received_event == V34_EVENT_J) ? "J" : "S(MD)"));
                    phase4_wait_init(s);
                }
                else if (!s->tx.calling_party
                         &&
                         !s->tx.v90_mode
                         &&
                         s->tx.baud_rate >= 0
                         &&
                         s->tx.baud_rate <= 5
                         &&
                         s->tx.tone_duration >= j_wait_max_bauds[s->tx.baud_rate])
                {
                    /* Interop fallback: if the peer's Phase 3 transition is not
                       detected reliably, move to Phase 4 after the allowed wait. */
                    span_log(&s->logging, SPAN_LOG_FLOW,
                             "Tx - Phase 3: J wait timeout (%d bauds), starting Phase 4 wait\n",
                             s->tx.tone_duration);
                    phase4_wait_init(s);
                }
                else
                {
                    /* Continue with repeats of J */
                    s->tx.persistence2 = j_pattern[j_pat_idx];
                }
                /*endif*/
            }
            else
            {
                mp_or_mph_baud_init(s);
            }
            /*endif*/
        }
        /*endif*/
        break;
    case V34_TX_STAGE_J_DASHED:
        /* Send J' (V.34 §10.1.3.4) — same differential encoding as J */
        bit = scramble(&s->tx, (s->tx.persistence2 & 1));
        s->tx.persistence2 >>= 1;
        bit = (scramble(&s->tx, (s->tx.persistence2 & 1)) << 1) | bit;
        s->tx.persistence2 >>= 1;
        /* Apply differential encoding per V.34 §10.1.3.3 */
        s->tx.diff = (s->tx.diff + bit) & 3;
        bit = s->tx.diff;
        if (++s->tx.tone_duration >= 16)
        {
            if (s->tx.calling_party  ||  s->tx.v90_v34_fallback)
            {
                /* Caller Phase 4 RX conditioning: once local J' is complete the
                   far-end answerer begins S/S-bar/TRN before MP. Re-arm RX for
                   explicit Phase 4 S detection here; otherwise the caller can
                   enter MP TX while RX is still stuck in Phase 3 WAIT_S. */
                phase4_rx_conditioning_init(s, V34_RX_STAGE_PHASE4_S, "S/S-bar/TRN then MP");
            }
            /*endif*/
            if (s->tx.calling_party || s->tx.v90_v34_fallback)
            {
                /* V.34 11.4.1.1.1: every call modem transmits one J' and
                   then TRN before MP.  This was accidentally restricted to
                   the V.90 fallback path, so plain V.34 put MP where the
                   answer modem was still conditioning on TRN. */
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - V.34 caller: J' complete, transmitting Phase 4 TRN before MP\n");
                s->tx.scramble_reg = 0;
                s->tx.stage = V34_TX_STAGE_PHASE4_TRN;
                s->tx.tone_duration = 0;
                s->tx.current_getbaud = get_phase4_baud;
            }
            else
            {
                mp_or_mph_baud_init(s);
            }
            /*endif*/
        }
        /*endif*/
        break;
    }
    /*endswitch*/
    return training_constellation_4[bit];
}
/*- End of function --------------------------------------------------------*/

static void trn_baud_init(v34_state_t *s)
{
    span_log(&s->logging, SPAN_LOG_FLOW, "Tx - trn_baud_init()\n");
    span_log(&s->logging, SPAN_LOG_FLOW,
             "Tx - Phase 3 TRN mode: %s-point (infoh.trn16=%d)\n",
             s->tx.infoh.trn16 ? "16" : "4", s->tx.infoh.trn16);
    s->tx.tone_duration = 0;
    s->tx.stage = V34_TX_STAGE_TRN;
    s->tx.current_getbaud = get_trn_baud;
}
/*- End of function --------------------------------------------------------*/

/* Phase 4 answer modem timing (V.34 §11.4.1.2):
   - S for 128T
   - S-bar for 16T
   - TRN for >=512T, then MP
   Keep wait at 0 so S starts immediately after J handling. */
#define PHASE4_WAIT_BAUDS 0
#define PHASE4_S_BAUDS 128
#define PHASE4_TRN_BAUDS 512
static int phase4_trn_max_bauds(const v34_state_t *s)
{
    int baud_rate;
    int max_bauds;

    if (s->tx.baud_rate < 0 || s->tx.baud_rate > 5)
        return 6858;
    /*endif*/
    baud_rate = baud_rate_parameters[s->tx.baud_rate].baud_rate;
    max_bauds = (baud_rate*2000 + 500)/1000;
    if (max_bauds < PHASE4_TRN_BAUDS)
        max_bauds = PHASE4_TRN_BAUDS;
    /*endif*/
    return max_bauds;
}
/*- End of function --------------------------------------------------------*/

static complex_sig_t get_phase4_baud(v34_state_t *s)
{
    int phase4_trn_guard_bauds;

    phase4_trn_guard_bauds = phase4_trn_max_bauds(s);
    switch (s->tx.stage)
    {
    case V34_TX_STAGE_PHASE4_WAIT:
        /* Transmit silence while waiting for the caller to complete Phase 3.
           V.34 §11.3.1.2.4: "After detecting the S-to-S-bar transition, the
           modem shall transmit silence." */
        if (++s->tx.tone_duration >= PHASE4_WAIT_BAUDS)
        {
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Tx - Phase 4: wait complete, starting S signal\n");
            /* V.34 10.1.3.7: S alternates two points separated by 90°.
               Use the same absolute-point generator as Phase 3 S rather than
               the old 180° differential sequence, which the caller cannot
               identify as S. */
            s->tx.lastbit = complex_sig_set(TRAINING_SCALE(TRAINING_AMP),
                                             TRAINING_SCALE(0.0f));
            s->tx.stage = V34_TX_STAGE_PHASE4_S;
            s->tx.tone_duration = 0;
        }
        /*endif*/
        return zero;

    case V34_TX_STAGE_PHASE4_S:
        /* V.34 10.1.3.7 and 11.4.1.2.1: alternate points separated by 90°
           for 128T, exactly as the Phase 3 S generator does. */
        if (++s->tx.tone_duration == PHASE4_S_BAUDS)
        {
            /* Same defect as the Phase 3 generator had: negating only the real
               part is a 180 degree rotation only where the imaginary part is
               zero, and the alternation guarantees it is not, so S-bar went
               out identical to S. */
            v34_rotate_180(&s->tx.lastbit);
            s->tx.stage = V34_TX_STAGE_PHASE4_NOT_S;
            s->tx.tone_duration = 0;
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Tx - Phase 4: S complete (%d bauds), starting S-bar (16T)\n",
                     PHASE4_S_BAUDS);
        }
        /*endif*/
        {
            complex_sig_t z = s->tx.lastbit;
            s->tx.lastbit.re = z.im;
            s->tx.lastbit.im = z.re;
        }
        return s->tx.lastbit;

    case V34_TX_STAGE_PHASE4_NOT_S:
        /* The 180 degree rotation above is the normative S-to-S-bar
           transition; continue the 90-degree-separated alternation for 16T. */
        if (++s->tx.tone_duration == 16)
        {
            v34_rotate_180(&s->tx.lastbit);
            /* V.34 10.1.3.8 initializes the TRN scrambler to zero. */
            s->tx.scramble_reg = 0;
            s->tx.stage = V34_TX_STAGE_PHASE4_TRN;
            s->tx.tone_duration = 0;
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Tx - Phase 4: S-bar complete, starting TRN (>=512T, mode=%s-point)\n",
                     s->tx.infoh.trn16 ? "16" : "4");
        }
        /*endif*/
        {
            complex_sig_t z = s->tx.lastbit;
            s->tx.lastbit.re = z.im;
            s->tx.lastbit.im = z.re;
        }
        return s->tx.lastbit;

    case V34_TX_STAGE_PHASE4_TRN:
        /* Phase 4 TRN: scrambled training before MP.
           V.34 §11.4.1.2.2: "the answer modem shall transmit TRN for at least
           512T but no longer than 2000 ms plus a round trip delay". */
        {
            int i_sym;
            int q_sym;
            int trn_sym;

            i_sym = scramble(&s->tx, 1);
            i_sym = (scramble(&s->tx, 1) << 1) | i_sym;
            if (s->tx.infoh.trn16)
            {
                q_sym = scramble(&s->tx, 1);
                q_sym = (scramble(&s->tx, 1) << 1) | q_sym;
                trn_sym = (q_sym << 2) | i_sym;
            }
            else
            {
                trn_sym = i_sym;
            }
            /*endif*/
            s->tx.tone_duration++;
            {
                static const char *p4trn_path = NULL;

                if (p4trn_path == NULL)
                    p4trn_path = getenv("V34_P4TRN_TX_DUMP")
                               ?  getenv("V34_P4TRN_TX_DUMP")  :  "";
                /*endif*/
                if (p4trn_path[0])
                {
                    FILE *f = fopen(p4trn_path, "a");

                    if (f)
                    {
                        fprintf(f, "%s %d %d\n",
                                s->tx.calling_party ? "caller" : "answer",
                                s->tx.tone_duration, trn_sym);
                        fclose(f);
                    }
                    /*endif*/
                }
                /*endif*/
            }
            if (s->rx.received_event == V34_EVENT_TRAINING_FAILED)
            {
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - Phase 4: TRN training failed after %d bauds, dropping call\n",
                         s->tx.tone_duration);
                s->tx.current_getbaud = NULL;
                return zero;
            }
            /*endif*/
            if (s->tx.v90_v34_fallback
                && s->tx.tone_duration >= PHASE4_TRN_BAUDS)
            {
                /* V.34 fallback call-modem role: our TRN follows our own J'
                   (§11.4.1.1.1); there is no far-end J' to wait for -- the
                   answerer moves to MP off our TRN.  512T minimum then MP. */
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - Phase 4 (V.34 fallback): TRN complete (%d bauds), starting MP\n",
                         s->tx.tone_duration);
                mp_or_mph_baud_init(s);
            }
            else if (s->tx.tone_duration >= PHASE4_TRN_BAUDS
                && s->rx.received_event == V34_EVENT_PHASE4_TRN_READY)
            {
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - Phase 4: TRN complete (%d bauds) and far-end J'/TRN confirmed, starting MP\n",
                         s->tx.tone_duration);
                mp_or_mph_baud_init(s);
            }
            else if (s->tx.tone_duration == PHASE4_TRN_BAUDS)
            {
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - Phase 4: local TRN minimum reached (%d bauds), waiting for far-end J'/TRN confirmation\n",
                         s->tx.tone_duration);
            }
            else if (s->tx.tone_duration >= phase4_trn_guard_bauds)
            {
                if (s->tx.tone_duration == phase4_trn_guard_bauds)
                {
                    /* Do not force MP locally. If RX has not confirmed far-end
                       J'/TRN yet, switching to MP only pollutes the peer's TRN
                       detector and creates a false late handoff. Hold TRN
                       until explicit PHASE4_TRN_READY arrives. */
                    span_log(&s->logging, SPAN_LOG_FLOW,
                             "Tx - Phase 4: TRN guard reached (%d bauds ~= 2000 ms) without far-end J'/TRN confirmation; continuing TRN\n",
                             s->tx.tone_duration);
                }
                else if ((s->tx.tone_duration % 512) == 0)
                {
                    span_log(&s->logging, SPAN_LOG_FLOW,
                             "Tx - Phase 4: TRN guard exceeded (%d bauds) without far-end J'/TRN confirmation; continuing TRN\n",
                             s->tx.tone_duration);
                }
                /*endif*/
            }
            /*endif*/
            return s->tx.infoh.trn16 ? training_constellation_16[trn_sym]
                                     : training_constellation_4[trn_sym];
        }

    default:
        return zero;
    }
    /*endswitch*/
}
/*- End of function --------------------------------------------------------*/

static void phase4_rx_conditioning_init(v34_state_t *s, int initial_stage, const char *reason)
{
    /* The S-to-S-bar junction detector starts unarmed; a zeroed struct would
       otherwise read as "junction already reached". */
    s->rx.phase4_s_bar_left = -1;
    s->rx.phase4_s_last_step = -1;
    const char *retain_env;
    bool retain_phase3_frontend;

    retain_env = getenv("ME_V34_RETAIN_PHASE3_FRONTEND");
    retain_phase3_frontend = s->rx.v90_mode
                          && retain_env
                          && atoi(retain_env) != 0;

    s->primary_channel_active = true;
    s->rx.current_demodulator = V34_MODULATION_V34;
    s->rx.stage = initial_stage;
    s->rx.duration = 0;
    s->rx.s_detect_count = 0;
    s->rx.s_window = 0;
    s->rx.bitstream = 0;
    s->rx.mp_seen = 0;
    s->rx.mp_remote_ack_seen = 0;
    s->rx.mp_count = -1;
    s->rx.mp_early_rejects = 0;
    s->rx.mp_hypothesis = -1;
    s->rx.received_event = V34_EVENT_NONE;
    memset(s->rx.phase3_j_scramble, 0, sizeof(s->rx.phase3_j_scramble));
    memset(s->rx.phase3_j_stream, 0, sizeof(s->rx.phase3_j_stream));
    memset(s->rx.phase3_j_prev_z, 0, sizeof(s->rx.phase3_j_prev_z));
    memset(s->rx.phase3_j_prev_valid, 0, sizeof(s->rx.phase3_j_prev_valid));
    memset(s->rx.phase3_j_win, 0, sizeof(s->rx.phase3_j_win));
    s->rx.phase3_j_bits = 0;
    s->rx.phase3_j_candidate_hyp = -1;
    s->rx.phase3_j_candidate_phase = -1;
    s->rx.phase3_j_candidate_pat = -1;
    s->rx.phase3_j_candidate_count = 0;
    s->rx.phase3_j_candidate_last_bits = 0;
    memset(s->rx.phase3_trn_scramble, 0, sizeof(s->rx.phase3_trn_scramble));
    memset(s->rx.phase3_trn_one_count, 0, sizeof(s->rx.phase3_trn_one_count));
    s->rx.phase3_trn_bits = 0;
    s->rx.phase3_trn_lock_hyp = -1;
    s->rx.phase3_trn_lock_score = -1;
    s->rx.phase4_j_seen = 0;
    s->rx.phase4_j_lock_hyp = -1;
    s->rx.phase4_trn_after_j = 0;
    s->rx.phase4_j_bits = 0;
    memset(s->rx.phase4_j_scramble_tap, 0, sizeof(s->rx.phase4_j_scramble_tap));
    memset(s->rx.phase4_j_stream_tap, 0, sizeof(s->rx.phase4_j_stream_tap));
    memset(s->rx.phase4_j_prev_z_tap, 0, sizeof(s->rx.phase4_j_prev_z_tap));
    memset(s->rx.phase4_j_prev_valid_tap, 0, sizeof(s->rx.phase4_j_prev_valid_tap));
    memset(s->rx.phase4_j_win_tap, 0, sizeof(s->rx.phase4_j_win_tap));
    memset(s->rx.phase4_trn_scramble_tap, 0, sizeof(s->rx.phase4_trn_scramble_tap));
    memset(s->rx.phase4_trn_one_count_tap, 0, sizeof(s->rx.phase4_trn_one_count_tap));
    memset(s->rx.phase4_trn_scramble, 0, sizeof(s->rx.phase4_trn_scramble));
    memset(s->rx.phase4_trn_prev_z, 0, sizeof(s->rx.phase4_trn_prev_z));
    memset(s->rx.phase4_trn_prev_valid, 0, sizeof(s->rx.phase4_trn_prev_valid));
    memset(s->rx.phase4_trn_one_count, 0, sizeof(s->rx.phase4_trn_one_count));
    s->rx.phase4_trn_lock_hyp = -1;
    s->rx.phase4_trn_lock_score = -1;
    s->rx.phase4_trn_lock_tap = -1;
    s->rx.phase4_trn_lock_order = -1;
    s->rx.phase4_trn_lock_domain = -1;
    s->rx.phase4_trn_current_hyp = -1;
    s->rx.phase4_trn_current_score = -1;
    s->rx.phase4_trn_current_tap = -1;
    s->rx.phase4_trn_current_order = -1;
    s->rx.phase4_trn_current_domain = -1;
    s->rx.mp_phase4_bit_order = 0;
    s->rx.mp_phase4_default_bit_order = 0;
    s->rx.mp_phase4_alt_order_active = 0;
    s->rx.mp_phase4_retry_mode = 0;
    memset(s->rx.mp_hyp_scramble, 0, sizeof(s->rx.mp_hyp_scramble));
    memset(s->rx.mp_hyp_bitstream, 0, sizeof(s->rx.mp_hyp_bitstream));

    /* The normal path carries the Phase 3 coefficients but re-seeds the
       surrounding timing/filter state for a clean Phase 4 acquisition.  On
       an analogue V.90 line, optionally retain the complete trained frontend
       instead: the equalizer history, carrier phase, RRC history, and TED
       state are all part of the coherent solution established by Phase 3. */
    if (!retain_phase3_frontend)
    {
        s->rx.eq_target_mag = 0.0f;
        cvec_zerof(s->rx.eq_buf, V34_EQUALIZER_MASK + 1);
        s->rx.eq_step = V34_EQUALIZER_PRE_LEN;
        s->rx.baud_half = 0;
        s->rx.v34_carrier_phase_rate = dds_phase_ratef(carrier_frequency(s->rx.baud_rate, s->rx.high_carrier));
        s->rx.carrier_phase = 0;
        memset(s->rx.rrc_filter, 0, sizeof(s->rx.rrc_filter));
        s->rx.rrc_filter_step = 0;
        s->rx.pri_ted.baud_phase = 0.0f;
        s->rx.pri_ted.symbol_sync_low[0] = 0.0f;
        s->rx.pri_ted.symbol_sync_low[1] = 0.0f;
        s->rx.pri_ted.symbol_sync_high[0] = 0.0f;
        s->rx.pri_ted.symbol_sync_high[1] = 0.0f;
        s->rx.pri_ted.symbol_sync_dc_filter[0] = 0.0f;
        s->rx.pri_ted.symbol_sync_dc_filter[1] = 0.0f;
        s->rx.total_baud_timing_correction = 0;
    }

    span_log(&s->logging, SPAN_LOG_FLOW,
             "Rx - Phase 4: conditioned for %s (stage=%d, baud_rate=%d, high_carrier=%d, carrier=%.1f Hz, frontend=%s)\n",
             reason ? reason : "Phase 4 startup",
             s->rx.stage,
             s->rx.baud_rate,
             s->rx.high_carrier,
             carrier_frequency(s->rx.baud_rate, s->rx.high_carrier),
             retain_phase3_frontend ? "retained" : "reset");
}
/*- End of function --------------------------------------------------------*/

static int mp_rate_n_is_valid(int rate_n)
{
    return (rate_n >= 1 && rate_n <= 14);
}
/*- End of function --------------------------------------------------------*/

static int mp_highest_masked_rate(int maximum, int mask)
{
    int rate;

    if (maximum > 14)
        maximum = 14;
    for (rate = maximum; rate >= 1; rate--)
    {
        if (mask & (1 << (rate - 1)))
            return rate;
    }
    return 0;
}
/*- End of function --------------------------------------------------------*/

static int v34_rx_current_trellis_code(const v34_rx_state_t *s)
{
    if (s->viterbi.encode_table == v34_conv64_encode_table)
        return V34_TRELLIS_64;
    if (s->viterbi.encode_table == v34_conv32_encode_table)
        return V34_TRELLIS_32;
    return V34_TRELLIS_16;
}
/*- End of function --------------------------------------------------------*/

static void v34_tx_get_mp_rates(v34_state_t *s, int *bit_rate_a_to_c, int *bit_rate_c_to_a)
{
    int a_to_c;
    int c_to_a;

    if (!bit_rate_a_to_c || !bit_rate_c_to_a)
        return;
    if (s->tx.mp_rate_policy_valid
        && mp_rate_n_is_valid(s->tx.mp_rate_a_to_c)
        && mp_rate_n_is_valid(s->tx.mp_rate_c_to_a))
    {
        *bit_rate_a_to_c = s->tx.mp_rate_a_to_c;
        *bit_rate_c_to_a = s->tx.mp_rate_c_to_a;
        return;
    }

    if (s->tx.v90_mode && !s->tx.v90_v34_fallback)
    {
        /* V.90 repurposes INFO1a's max-rate field as U_INFO and its PCM
           direction is negotiated by CP, not Table 16/V.34.  Preserve the
           existing V.90 ceiling until that path supplies an explicit policy. */
        a_to_c = (s->tx.parms.max_bit_rate_code >> 1) + 1;
        c_to_a = a_to_c;
        if (s->calling_party
            && mp_rate_n_is_valid(s->rx.info1c.rate_data[s->tx.baud_rate].max_bit_rate))
            a_to_c = s->rx.info1c.rate_data[s->tx.baud_rate].max_bit_rate;
        *bit_rate_a_to_c = a_to_c;
        *bit_rate_c_to_a = c_to_a;
        return;
    }

    /* V.34 10.1.2.3.4/.5 and 10.1.3.9: carry the directional Phase-2
       projections into MP.  Do not recreate them from the selected baud's
       theoretical maximum; the line probe may have selected a lower rate. */
    if (s->calling_party)
    {
        a_to_c = s->tx.info1c.rate_data[s->rx.baud_rate].max_bit_rate;
        c_to_a = s->rx.info1a.max_data_rate;
    }
    else
    {
        a_to_c = s->rx.info1c.rate_data[s->tx.baud_rate].max_bit_rate;
        c_to_a = s->tx.info1a.max_data_rate;
    }
    if (!mp_rate_n_is_valid(a_to_c))
        a_to_c = 1;
    if (!mp_rate_n_is_valid(c_to_a))
        c_to_a = 1;
    *bit_rate_a_to_c = a_to_c;
    *bit_rate_c_to_a = c_to_a;
}
/*- End of function --------------------------------------------------------*/

static complex_sig_t get_v34_answer_phase3_wait_j_baud(v34_state_t *s)
{
    /* V.34 11.3.1.2.6 permits up to 500 ms after J; start immediately once
       the waveform receiver has made J authoritative. */
    if (s->rx.received_event == V34_EVENT_J
        || s->rx.received_event == V34_EVENT_J_DASHED)
    {
        span_log(&s->logging, SPAN_LOG_FLOW,
                 "Tx - V.34 answerer: caller J received, starting Phase 4 S\n");
        s->rx.received_event = V34_EVENT_NONE;
        phase4_wait_init(s);
    }
    return zero;
}
/*- End of function --------------------------------------------------------*/

static void v34_answer_phase3_wait_j_init(v34_state_t *s)
{
    /* V.34 11.3.1.2.4-.6: silence follows the caller's S/S-bar while PP and
       TRN condition the primary receiver.  PHASE3_TRAINING promotes itself to
       the J scanner after PP acquisition and the initial TRN interval. */
    s->rx.received_event = V34_EVENT_NONE;
    s->rx.current_demodulator = V34_MODULATION_V34;
    s->rx.stage = V34_RX_STAGE_PHASE3_TRAINING;
    s->rx.duration = 0;
    s->rx.phase3_pp_started = 0;
    s->rx.phase3_pp_acquire_hits = 0;
    s->rx.phase3_pp_phase = -1;
    s->rx.phase3_pp_phase_score = -1;
    memset(s->rx.phase3_pp_error, 0, sizeof(s->rx.phase3_pp_error));
    memset(s->rx.phase3_pp_corr, 0, sizeof(s->rx.phase3_pp_corr));
    s->rx.phase3_pp_corr_energy = 0.0f;
    s->rx.phase3_pp_corr_weight = 0.0f;
    s->tx.tone_duration = 0;
    s->tx.current_getbaud = get_v34_answer_phase3_wait_j_baud;
    span_log(&s->logging, SPAN_LOG_FLOW,
             "Tx - V.34 answerer: caller S received; silent while acquiring PP/TRN/J\n");
}
/*- End of function --------------------------------------------------------*/

static void phase4_wait_init(v34_state_t *s)
{
    span_log(&s->logging, SPAN_LOG_FLOW, "Tx - phase4_wait_init()\n");
    s->primary_channel_active = true;
    s->tx.current_modulator = V34_MODULATION_V34;
    s->tx.stage = V34_TX_STAGE_PHASE4_WAIT;
    s->tx.tone_duration = 0;
    s->tx.diff = 0;
    s->tx.current_getbaud = get_phase4_baud;
    /* External V.90 Phase 3 may hand us back TX while the native V.34 side is
       still parked on silence/control-channel state. Re-prime the V.34 TX
       pulse-shaping path so Phase 4 S/S-bar/TRN starts on the actual primary
       channel instead of remaining in the old SILENCE modulator. */
    s->tx.lastbit = complex_sig_set(TRAINING_SCALE(TRAINING_AMP), TRAINING_SCALE(0.0f));
    s->tx.baud_phase = 0;
    s->tx.rrc_filter_step = 0;
    memset(s->tx.rrc_filter_re, 0, sizeof(s->tx.rrc_filter_re));
    memset(s->tx.rrc_filter_im, 0, sizeof(s->tx.rrc_filter_im));

    /* Phase 4 answerer RX conditioning (V.34 11.4.1.2.1/11.4.1.2.2):
       while sending S, condition receiver to detect caller J' followed by TRN,
       then receive MP. We therefore enter TRN/descrambler conditioning directly
       instead of waiting on a far-end S detector. */
    phase4_rx_conditioning_init(s, V34_RX_STAGE_PHASE4_TRN, "J'/TRN then MP");
}
/*- End of function --------------------------------------------------------*/

/* Number of bauds to transmit silence at the start of MP TX.
   This allows the caller's MP to arrive without echo interference
   from our own TX.  Set to 0 to disable (normal V.34 operation). */
#define MP_TX_SILENCE_BAUDS 0

static complex_sig_t get_mp_or_mph_baud(v34_state_t *s)
{
    int bit;

    /* Transmit silence for the first MP_TX_SILENCE_BAUDS to test
       whether echo of our own TX is corrupting MP reception. */
    if (MP_TX_SILENCE_BAUDS > 0)
    {
        s->tx.tone_duration++;
        if (s->tx.tone_duration <= MP_TX_SILENCE_BAUDS)
        {
            if (s->tx.tone_duration == 1)
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - MP: transmitting silence for %d bauds (echo test)\n",
                         MP_TX_SILENCE_BAUDS);
            if (s->tx.tone_duration == MP_TX_SILENCE_BAUDS)
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - MP: silence period complete, starting MP TX\n");
            return zero;
        }
    }

    bit = scramble(&s->tx, get_data_bit(&s->tx));
    bit = (scramble(&s->tx, get_data_bit(&s->tx)) << 1) | bit;
    if (s->tx.txptr >= s->tx.txbits)
    {
        if (s->tx.duplex)
        {
            /* Check if we've received the far-end's MP (mp_seen >= 1).
               If so, set the acknowledge bit to turn MP into MP'.
               V.34 §11.4.2: "When the modem has received its first valid MP
               sequence from the far end, it shall begin transmitting MP' with
               the acknowledge bit set to 1." */
            if (s->rx.mp_seen >= 1  &&  !s->tx.mp.mp_acknowledged)
            {
                s->tx.mp.mp_acknowledged = 1;
                s->tx.txbits = mp_sequence_tx(&s->tx, &s->tx.mp);
                s->tx.txptr = 0;
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - far-end MP received, switching to MP'\n");
                /* V.34 11.4.1.1.3/11.4.1.2.4 requires a complete MP'
                   sequence before E.  Do not observe the already-received
                   remote acknowledgement in this same end-of-MP iteration
                   and skip transmission of our newly built MP' entirely. */
            }
            else if (s->tx.mp.mp_acknowledged  &&  s->rx.mp_remote_ack_seen)
            {
                /* A complete local MP' has now been sent and remote MP' was
                   received, so the next sequence is E. */
                e_baud_init(s);
            }
            else
            {
                s->tx.txptr = 0;
            }
            /*endif*/
        }
        else
        {
            /* Restart the message */
            s->tx.txptr = 0;
        }
        /*endif*/
    }
    /*endif*/
    s->tx.diff = (s->tx.diff + bit) & 3;
    return training_constellation_4[s->tx.diff];
}
/*- End of function --------------------------------------------------------*/

static void mp_or_mph_baud_init(v34_state_t *s)
{
    span_log(&s->logging, SPAN_LOG_FLOW, "Tx - mp_baud_init()\n");
    s->tx.current_modulator = V34_MODULATION_V34;

    if (s->tx.duplex
        && (s->tx.calling_party  ||  s->tx.v90_v34_fallback)
        && s->tx.v90_mode
        && s->rx.stage < V34_RX_STAGE_PHASE4_S)
    {
        /* Caller-side safety net: native V.90 can reach MP TX through a few
           different Phase 3 exit seams. If RX has not already been advanced
           into Phase 4 by the time MP starts, force the caller onto explicit
           S/S-bar/TRN receive conditioning here rather than carrying a stale
           Phase 3 WAIT_S state into MP. */
        phase4_rx_conditioning_init(s, V34_RX_STAGE_PHASE4_S, "S/S-bar/TRN then MP");
    }
    /*endif*/

    if (s->tx.duplex)
    {
        int bit_rate_a_to_c;
        int bit_rate_c_to_a;
        int i;
        int mask;

        /* Populate MP parameters from negotiated settings.
           V.34/Table 12 defines the MP sequence fields. */
        s->tx.mp.type = 0;  /* MP0 — no precoder coefficients initially */
        v34_tx_get_mp_rates(s, &bit_rate_a_to_c, &bit_rate_c_to_a);
        s->tx.mp.bit_rate_a_to_c = bit_rate_a_to_c;
        s->tx.mp.bit_rate_c_to_a = bit_rate_c_to_a;

        /* Table 20 bit 35:49 is capability in both the transmitter and
           receiver of this modem.  With asymmetric INFO1 symbol rates it is
           the intersection of both mapping tables, not the TX table alone. */
        mask = 0;
        for (i = 0;  i < 14;  i++)
        {
            if (baud_rate_parameters[s->tx.baud_rate].mappings[i * 2].b > 0
                && baud_rate_parameters[s->rx.baud_rate].mappings[i * 2].b > 0)
                mask |= (1 << i);
        }
        /*endfor*/
        s->tx.mp.signalling_rate_mask = mask;
        s->tx.mp.bit_rate_a_to_c =
            mp_highest_masked_rate(s->tx.mp.bit_rate_a_to_c, mask);
        s->tx.mp.bit_rate_c_to_a =
            mp_highest_masked_rate(s->tx.mp.bit_rate_c_to_a, mask);

        /* V.34 10.1.3.9/Table 20: these encoder fields select the
           remote-end transmitter, so advertise this receiver's requested
           modes.  The peer's MP configures our transmitter on receipt. */
        s->tx.mp.trellis_size = v34_rx_current_trellis_code(&s->rx);
        s->tx.mp.use_non_linear_encoder = s->rx.use_non_linear_encoder;
        s->tx.mp.expanded_shaping = s->rx.parms.expanded_shaping;
        s->tx.mp.aux_channel_supported = false;
        s->tx.mp.asymmetric_rates_allowed = s->tx.v90_mode
                                          ? (bit_rate_a_to_c != bit_rate_c_to_a)
                                          : true;
        s->tx.mp.mp_acknowledged = false;
        s->tx.negotiated_rates_valid = false;
        s->tx.negotiated_rate_a_to_c = 0;
        s->tx.negotiated_rate_c_to_a = 0;
        /* Before the first MP, 10.1.3.9 initializes precoding coefficients
           to zero.  A later MP0 leaves them unaffected; process_rx_mp only
           replaces them when a valid MP1 arrives. */
        if (!s->rx.last_rx_mp_valid)
        {
            memset(s->tx.precoder_coeffs, 0, sizeof(s->tx.precoder_coeffs));
            memset(s->rx.h, 0, sizeof(s->rx.h));
        }

        log_mp(s->tx.logging, true, &s->tx.mp);
        s->tx.txbits = mp_sequence_tx(&s->tx, &s->tx.mp);
        if (getenv("V34_MP_TX_BITS"))
        {
            /* Diagnostic only: the transmitted MP0 frame, so a receiver's
               decoded frame can be diffed against the truth.  Crosses no
               protocol seam -- nothing reads this but a human. */
            char buf[200];
            int bi;

            for (bi = 0;  bi < s->tx.txbits  &&  bi < 88;  bi++)
                buf[bi] = (char) ('0' + ((s->tx.txbuf[bi >> 3] >> (bi & 7)) & 1));
            /*endfor*/
            buf[bi] = '\0';
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Tx - MP0 frame bits[0..%d]: %s\n", bi - 1, buf);
        }
        /*endif*/
        s->tx.stage = V34_TX_STAGE_MP;
    }
    else
    {
        s->tx.txbits = mph_sequence_tx(&s->tx, &s->tx.mph);
        s->tx.stage = V34_TX_STAGE_HDX_MPH;
    }
    /*endif*/
    s->tx.txptr = 0;
    s->tx.tone_duration = 0;  /* Reset for MP_TX_SILENCE_BAUDS counter */
    s->tx.current_getbaud = get_mp_or_mph_baud;

    /* Phase 4 MP exchange is DUPLEX on the PRIMARY channel, not CC.
       V.34 §11.4: Both sides transmit MP using the same 4-point constellation
       at the negotiated baud rate.
       DO NOT touch RX state here — the RX progresses independently
       through TRN/J' conditioning into MP decode.
       The RX stage was set in phase4_wait_init(). */
    span_log(&s->logging, SPAN_LOG_FLOW, "Tx - MP transmission starting (RX conditioned for J'/TRN/MP)\n");
}
/*- End of function --------------------------------------------------------*/

static complex_sig_t get_e_baud(v34_state_t *s)
{
    int bit;

    /* V.34 E detection in RX is based on 20 consecutive post-descrambler 1 bits.
       Therefore transmit continuous scrambled 1s on the primary channel so a
       synchronized far-end descrambler yields an all-ones run. */
    bit = scramble(&s->tx, 1);
    bit = (scramble(&s->tx, 1) << 1) | bit;
    s->tx.diff = (s->tx.diff + bit) & 3;
    if (++s->tx.tone_duration == 10)
    {
        span_log(&s->logging, SPAN_LOG_FLOW,
                 "Tx - E minimum reached (>=20 bits)\n");
    }
    /*endif*/
    /* V.34 10.1.3.2 and 11.4.1 require a single 20-bit E sequence.
       Extending it to 40 bits leaves ten QPSK symbols after a conforming
       receiver has entered B1 and destroys the eight-symbol mapping-frame
       boundary. */
    if (s->tx.tone_duration >= 10)
    {
        data_baud_init(s);
    }
    /*endif*/
    return training_constellation_4[s->tx.diff];
}
/*- End of function --------------------------------------------------------*/

static void e_baud_init(v34_state_t *s)
{
    span_log(&s->logging, SPAN_LOG_FLOW, "Tx - e_baud_init()\n");
    s->tx.tone_duration = 0;
    s->tx.stage = V34_TX_STAGE_HDX_E;
    s->tx.current_getbaud = get_e_baud;
}
/*- End of function --------------------------------------------------------*/

static complex_sig_t get_data_baud(v34_state_t *s)
{
    complex_sig_t v;

    if (s->tx.tx_mapping_frame_step == 0)
    {
        /* Need a new mapping frame */
        if (s->tx.b1_frames_sent < s->tx.parms.p)
        {
            /* §10.1.3.1/V.34: B1 is a complete data frame of P mapping
               frames, all ones.  One mapping frame is only 2.5 ms at 3200
               baud; treating that as B1 starts payload inside the receiver's
               remaining B1 interval and permanently displaces framing. */
            span_get_bit_func_t saved_get_bit = s->tx.current_get_bit;
            s->tx.current_get_bit = fake_get_bit;
            v34_get_mapping_frame(&s->tx, s->tx.tx_mapping_frame_buf);
            s->tx.current_get_bit = saved_get_bit;
            if (++s->tx.b1_frames_sent == s->tx.parms.p)
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - B1 complete (%d all-ones mapping frames)\n",
                         s->tx.parms.p);
        }
        else
        {
            v34_get_mapping_frame(&s->tx, s->tx.tx_mapping_frame_buf);
        }
        /*endif*/
    }
    /*endif*/

    /* Return one 2D symbol from the current mapping frame.
       tx_mapping_frame_buf holds the precoded signal x(n) in Q9.7, as written by
       v34_get_mapping_frame() ("(y.re << 7) - p.re"). It has to be scaled back by
       128 before it reaches the modulator - the plain cast that used to be here
       handed the modulator symbols 128x too large. v34_get_mapping_frame() keeps
       its Q9.7 output contract, since v34_get_mapping_frame_state() callers and
       the spandsp v34 tests read those raw values. */
    v.re = FP_Q9_7_TO_F(s->tx.tx_mapping_frame_buf[2*s->tx.tx_mapping_frame_step])
         * s->tx.data_symbol_scale;
    v.im = FP_Q9_7_TO_F(s->tx.tx_mapping_frame_buf[2*s->tx.tx_mapping_frame_step + 1])
         * s->tx.data_symbol_scale;
    if (++s->tx.tx_mapping_frame_step >= 8)
        s->tx.tx_mapping_frame_step = 0;
    /*endif*/
    return v;
}
/*- End of function --------------------------------------------------------*/

static void data_baud_init(v34_state_t *s)
{
    /* Update TX parms from the MP-negotiated rate for our transmit direction */
    {
        int tx_rate_n;
        const mp_t *remote_mp;

        tx_rate_n = s->calling_party
                  ? s->tx.negotiated_rate_c_to_a
                  : s->tx.negotiated_rate_a_to_c;
        if (!s->tx.negotiated_rates_valid || !mp_rate_n_is_valid(tx_rate_n))
        {
            /* Defensive only: §11.4 does not permit E until a mutually valid
               MP/MP-prime exchange has selected both rates. */
            tx_rate_n = s->calling_party
                      ? s->tx.mp.bit_rate_c_to_a
                      : s->tx.mp.bit_rate_a_to_c;
        }
        remote_mp = s->rx.last_rx_mp_valid ? &s->rx.last_rx_mp : &s->tx.mp;
        s->tx.bit_rate = (tx_rate_n - 1) * 2;
        v34_set_working_parameters(&s->tx.parms, s->tx.baud_rate, s->tx.bit_rate,
                                   remote_mp->expanded_shaping);
        s->tx.use_non_linear_encoder = remote_mp->use_non_linear_encoder;
        span_log(&s->logging, SPAN_LOG_FLOW,
                 "Tx - data_baud_init(): rate=%d bps (N=%d code=%d) "
                 "b=%d k=%d q=%d m=%d p=%d j=%d l=%d\n",
                 tx_rate_n * 2400, tx_rate_n, s->tx.bit_rate,
                 s->tx.parms.b, s->tx.parms.k, s->tx.parms.q, s->tx.parms.m,
                 s->tx.parms.p, s->tx.parms.j, s->tx.parms.l);
    }
    s->tx.tx_mapping_frame_step = 0;
    s->tx.b1_frames_sent = 0;
    s->tx.data_frame = 0;
    /* V.34 10.1.3.1 inserts B1's V0 inversions as though B1 were the
       final data frame of a superframe.  Start there; after P mapping frames
       the normal frame advance wraps to superframe zero for payload. */
    if (s->tx.parms.j > 0)
    {
        s->tx.super_frame = s->tx.parms.j - 1;
        s->tx.v0_pattern = (uint16_t)(2*(s->tx.parms.j - 1));
    }
    else
    {
        s->tx.super_frame = 0;
        s->tx.v0_pattern = 0;
    }
    s->tx.s_bit_cnt = 0;
    s->tx.aux_bit_cnt = 0;
    /* Initialize scrambler, precoder and trellis state for B1.  V.34
       10.1.3.1 resets all of them; MP/E use the same scrambler register but
       are a separate sequence and must not leak state into B1. */
    s->tx.scramble_reg = 0;
    s->tx.c.re = 0;
    s->tx.c.im = 0;
    s->tx.p.re = 0;
    s->tx.p.im = 0;
    s->tx.z = 0;
    span_log(&s->logging, SPAN_LOG_FLOW,
             "Tx - data_baud_init(): trellis state on entry state=%d y0=%d "
             "(V.34/9.6 requires 0; forcing)\n",
             s->tx.state, s->tx.y0);
    s->tx.y0 = 0;
    s->tx.state = 0;
    s->tx.data_symbol_scale = 1.0f;
    {
        int trellis = (s->tx.conv_encode_table == v34_conv64_encode_table)
                    ? V34_TRELLIS_64
                    : ((s->tx.conv_encode_table == v34_conv32_encode_table)
                       ? V34_TRELLIS_32 : V34_TRELLIS_16);
        int16_t precoder[6];
        v34_state_t *probe;

        for (int i = 0; i < 3; i++)
        {
            precoder[2*i] = s->tx.precoder_coeffs[i].re;
            precoder[2*i + 1] = s->tx.precoder_coeffs[i].im;
        }
        probe = v34_init(NULL,
                         baud_rate_parameters[s->tx.baud_rate].baud_rate,
                         (s->tx.bit_rate/2 + 1)*2400,
                         s->tx.calling_party, true,
                         fake_get_bit, NULL, NULL, NULL);
        if (probe
            && v34_seed_tx_data(probe, s->tx.bit_rate/2 + 1, trellis,
                                s->tx.use_non_linear_encoder,
                                s->tx.parms.expanded_shaping, precoder) == 0)
        {
            double energy = 0.0;
            int symbols = 0;
            int16_t frame[16];
            int frames = probe->tx.parms.p*probe->tx.parms.j;

            probe->tx.scrambler_tap = s->tx.scrambler_tap;
            probe->tx.super_frame = probe->tx.parms.j - 1;
            probe->tx.v0_pattern = (uint16_t)(2*(probe->tx.parms.j - 1));
            /* The note in 10.1.3 requires compensation for modulation
               factors so PP/TRN power is maintained in B1 and DATA.  Measure
               a reset-state superframe through the negotiated mapper. */
            for (int m = 0; m < frames; m++)
            {
                if (v34_get_mapping_frame(&probe->tx, frame) != 16)
                    break;
                for (int i = 0; i < 16; i++)
                {
                    double x = frame[i]/128.0;
                    energy += x*x;
                }
                symbols += 8;
            }
            if (symbols > 0 && energy > 0.0)
            {
                float rms = (float) sqrt(energy/symbols);
                s->tx.data_symbol_scale = V34_NOMINAL_SYMBOL_RMS/rms;
                span_log(&s->logging, SPAN_LOG_FLOW,
                         "Tx - data modulation normalization: mapper_rms=%.4f "
                         "target=%.4f scale=%.5f (V.34 10.1.3)\n",
                         rms, V34_NOMINAL_SYMBOL_RMS,
                         s->tx.data_symbol_scale);
            }
        }
        if (probe)
            v34_free(probe);
    }
    s->tx.current_modulator = V34_MODULATION_V34;
    s->tx.tx_data_mode = true;
    s->tx.current_getbaud = get_data_baud;
    span_log(&s->logging, SPAN_LOG_FLOW, "Tx - switching to DATA mode\n");
}
/*- End of function --------------------------------------------------------*/

static complex_sig_t get_pph_baud(v34_state_t *s)
{
    int i;

    /* This is the beginning of half-duplex control channel restart */
    /* The 8 symbol PPh signal, which is repeated 4 times, to make a 32 symbol sequence */
    /* See V.34/10.2.4.5 */
    i = s->tx.tone_duration & 0x7;
    if (++s->tx.tone_duration == PPH_SYMBOLS*PPH_REPEATS)
        second_alt_baud_init(s);
    /*endif*/
    return pph_symbols[i];
}
/*- End of function --------------------------------------------------------*/

static void pph_baud_init(v34_state_t *s)
{
    span_log(&s->logging, SPAN_LOG_FLOW, "Tx - pph_baud_init()\n");
    s->tx.tone_duration = 0;
    s->tx.current_modulator = V34_MODULATION_CC;
    s->tx.stage = V34_TX_STAGE_HDX_PPH;
    s->tx.current_getbaud = get_pph_baud;
}
/*- End of function --------------------------------------------------------*/

static complex_sig_t get_second_alt_baud(v34_state_t *s)
{
    int bit;

    /* Signal ALT is transmitted using the control channel modulation with the differential
       encoder enabled and consists of scrambled alternations of binary 0 and 1 at 1200 bit/s.
       The initial state of the scrambler shall be all zeroes. */
    /* See V.34/10.2.4.2 */
    bit = scramble(&s->tx, 0);
    bit = (scramble(&s->tx, 1) << 1) | bit;
    s->tx.diff = (s->tx.diff + bit) & 3;
    if (++s->tx.tone_duration >= 16)
    {
        /* We have reached the absolute minimum allowed for the duration of ALT */
        if (s->tx.tone_duration >= 120)
        {
            /* TODO: Should allow for early termination. */
            if (1)
            {
                /* Control channel training */
                mp_or_mph_baud_init(s);
            }
            else
            {
                /* Control channel resynchronisation */
                e_baud_init(s);
            }
            /*endif*/
        }
        /*endif*/
    }
    /*endif*/
    return training_constellation_4[s->tx.diff];
}
/*- End of function --------------------------------------------------------*/

static void second_alt_baud_init(v34_state_t *s)
{
    span_log(&s->logging, SPAN_LOG_FLOW, "Tx - second_alt_baud_init()\n");
    s->tx.tone_duration = 0;
    s->tx.current_modulator = V34_MODULATION_V34;
    s->tx.scramble_reg = 0;
    s->tx.diff = 0;
    s->tx.stage = V34_TX_STAGE_HDX_SECOND_ALT;
    s->tx.current_getbaud = get_second_alt_baud;
}
/*- End of function --------------------------------------------------------*/

static complex_sig_t get_first_alt_baud(v34_state_t *s)
{
    int bit;

    /* Signal ALT is transmitted using the control channel modulation with the differential
       encoder enabled and consists of scrambled alternations of binary 0 and 1 at 1200 bit/s.
       The initial state of the scrambler shall be all zeroes. */
    /* See V.34/10.2.4.2 */
    bit = scramble(&s->tx, 0);
    bit = (scramble(&s->tx, 1) << 1) | bit;
    s->tx.diff = (s->tx.diff + bit) & 3;
    if (++s->tx.tone_duration >= 16)
    {
        /* We have reached the absolute minimum allowed for the duration of ALT */
        if (s->tx.tone_duration >= 120)
        {
            /* TODO: Should allow for early termination. */
            /* Control channel training */
            pph_baud_init(s);
        }
        /*endif*/
    }
    /*endif*/
    return training_constellation_4[s->tx.diff];
}
/*- End of function --------------------------------------------------------*/

static void first_alt_baud_init(v34_state_t *s)
{
    span_log(&s->logging, SPAN_LOG_FLOW, "Tx - first_alt_baud_init()\n");
    s->tx.tone_duration = 0;
    s->tx.current_modulator = V34_MODULATION_V34;
    s->tx.scramble_reg = 0;
    s->tx.diff = 0;
    s->tx.stage = V34_TX_STAGE_HDX_FIRST_ALT;
    s->tx.current_getbaud = get_first_alt_baud;
}
/*- End of function --------------------------------------------------------*/

static complex_sig_t get_sh_baud(v34_state_t *s)
{
#define SH_PLUS_NO_SH_SYMBOLS       32
    static const uint8_t sh_plus_not_sh[SH_PLUS_NO_SH_SYMBOLS] =
    {
        2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1,     /* Sh */
        0, 3, 0, 3, 0, 3, 0, 3                                                      /* !Sh */
    };
    int i;

    /* See V.34/10.2.3.3 */
    i = s->tx.tone_duration;
    if (++s->tx.tone_duration == SH_PLUS_NO_SH_SYMBOLS)
    {
        /* The Sh and !Sh have finished */
        first_alt_baud_init(s);
    }
    /*endif*/
    return training_constellation_4[sh_plus_not_sh[i]];
}
/*- End of function --------------------------------------------------------*/

static void sh_baud_init(v34_state_t *s)
{
    /* This is the beginning of half-duplex control channel startup */
    span_log(&s->logging, SPAN_LOG_FLOW, "Tx - sh_baud_init()\n");
    s->tx.lastbit = complex_sig_set(TRAINING_SCALE(TRAINING_AMP), TRAINING_SCALE(0.0f));
    s->tx.tone_duration = 0;
    s->tx.current_modulator = V34_MODULATION_V34;
    s->tx.stage = V34_TX_STAGE_HDX_SH;
    s->tx.current_getbaud = get_sh_baud;
}
/*- End of function --------------------------------------------------------*/

#if defined(SPANDSP_USE_FIXED_POINT)
static __inline__ uint32_t dist_sq(const complexi_t *x, const complexi_t *y)
{
    return (x->re - y->re)*(x->re - y->re) + (x->im - y->im)*(x->im - y->im);
}
/*- End of function --------------------------------------------------------*/
#else
static __inline__ float dist_sq(const complexf_t *x, const complexf_t *y)
{
    return (x->re - y->re)*(x->re - y->re) + (x->im - y->im)*(x->im - y->im);
}
/*- End of function --------------------------------------------------------*/
#endif

static __inline__ complex_sig_t training_get(v34_tx_state_t *s)
{
    return zero;
}
/*- End of function --------------------------------------------------------*/

static __inline__ complex_sig_t connect_sequence_get(v34_tx_state_t *s)
{
    return zero;
}
/*- End of function --------------------------------------------------------*/

static int tx_v34_modulation(v34_state_t *s, int16_t amp[], int max_len)
{
#if defined(SPANDSP_USE_FIXED_POINT)
    complexi16_t v;
    complexi32_t x;
    complexi_t z;
#else
    complexf_t v;
    complexf_t x;
    complexf_t z;
#endif
    const tx_shaper_t *shaper;
    int num;
    int den;
    int i;
    int sample;

    /* The V.34 modulator. */
    num = s->tx.parms.samples_per_symbol_numerator;
    den = s->tx.parms.samples_per_symbol_denominator;
    shaper = v34_tx_shapers[s->tx.baud_rate];
    for (sample = 0;  sample < max_len;  sample++)
    {
        if ((s->tx.baud_phase += den) >= num)
        {
            s->tx.baud_phase -= num;
            if (s->tx.current_getbaud == NULL)
            {
                if (!s->tx.getbaud_null_logged)
                {
                    span_log(&s->logging, SPAN_LOG_ERROR,
                             "Tx - NULL current_getbaud in V34 modulator (stage=%d mod=%d); silencing further\n",
                             s->tx.stage, s->tx.current_modulator);
                    s->tx.getbaud_null_logged = true;
                }
                /*endif*/
                v = zero;
            }
            else
            {
                v = s->tx.current_getbaud(s);
                /* Same per-baud stage-change check as the CC modulator, so the
                   Phase 3/4 stages are timed on the same basis. */
                v34_tx_log_state_change_at(s, sample);
            }
            /*endif*/
            s->tx.rrc_filter_re[s->tx.rrc_filter_step] = v.re;
            s->tx.rrc_filter_im[s->tx.rrc_filter_step] = v.im;
            if (++s->tx.rrc_filter_step >= V34_TX_FILTER_STEPS)
                s->tx.rrc_filter_step = 0;
            /*endif*/
        }
        /*endif*/
        /* Root raised cosine pulse shaping at baseband */
#if defined(SPANDSP_USE_FIXED_POINT)
        x = complex_seti32(0, 0);
        for (i = 0;  i < V34_TX_FILTER_STEPS;  i++)
        {
            x.re += (int32_t) shaper[num - 1 - s->tx.baud_phase][i]*(int32_t) s->tx.rrc_filter[i + s->tx.rrc_filter_step].re;
            x.im += (int32_t) shaper[num - 1 - s->tx.baud_phase][i]*(int32_t) s->tx.rrc_filter[i + s->tx.rrc_filter_step].im;
        }
        /*endfor*/
        /* Now create and modulate the carrier */
        x.re >>= 4;
        x.im >>= 4;
        z = dds_complexi(&(s->tx.carrier_phase), s->tx.v34_carrier_phase_rate);
        /* Don't bother saturating. We should never clip. */
        i = (x.re*z.re - x.im*z.im) >> 15;
        amp[sample] = (int16_t) ((i*s->tx.gain) >> 15);
#else
        x.re = vec_circular_dot_prodf(s->tx.rrc_filter_re, shaper[num - 1 - s->tx.baud_phase], V34_TX_FILTER_STEPS, s->tx.rrc_filter_step);
        x.im = vec_circular_dot_prodf(s->tx.rrc_filter_im, shaper[num - 1 - s->tx.baud_phase], V34_TX_FILTER_STEPS, s->tx.rrc_filter_step);
        /* Now create and modulate the carrier */
        z = dds_complexf(&(s->tx.carrier_phase), s->tx.v34_carrier_phase_rate);
        /* Don't bother saturating. We should never clip. */
        {
            float sample_f;

            sample_f = (x.re*z.re - x.im*z.im)*s->tx.gain;
            /* Apply pre-emphasis filter if active (V.34/5.4).
               This shapes the TX spectrum as requested by the remote modem's INFO1c. */
            if (s->tx.pre_emphasis_coeffs)
            {
                float filtered;
                int j;
                int idx;

                s->tx.pre_emphasis_buf[s->tx.pre_emphasis_idx] = sample_f;
                filtered = 0.0f;
                for (j = 0;  j < 16;  j++)
                {
                    idx = s->tx.pre_emphasis_idx - j;
                    if (idx < 0)
                        idx += 16;
                    /*endif*/
                    filtered += s->tx.pre_emphasis_coeffs[j]*s->tx.pre_emphasis_buf[idx];
                }
                /*endfor*/
                if (++s->tx.pre_emphasis_idx >= 16)
                    s->tx.pre_emphasis_idx = 0;
                /*endif*/
                amp[sample] = (int16_t) lfastrintf(filtered);
            }
            else
            {
                amp[sample] = (int16_t) lfastrintf(sample_f);
            }
            /*endif*/
        }
#endif
    }
    /*endfor*/
    return sample;
}
/*- End of function --------------------------------------------------------*/

static int tx_cc_modulation(v34_state_t *s, int16_t amp[], int max_len)
{
#if defined(SPANDSP_USE_FIXED_POINT)
    complexi16_t v;
    complexi32_t x;
    complexi_t z;
    int16_t iamp;
#else
    complexf_t v;
    complexf_t x;
    complexf_t z;
    float famp;
#endif
    int sample;

    /* The V.22bis like split band modulator for configuration data and the
       half-duplex control channel. */
    for (sample = 0;  sample < max_len;  sample++)
    {
        if ((s->tx.baud_phase += 3) >= 40)
        {
            s->tx.baud_phase -= 40;
            if (s->tx.current_getbaud == NULL)
            {
                span_log(&s->logging, SPAN_LOG_ERROR,
                         "Tx - NULL current_getbaud in CC modulator (stage=%d mod=%d)\n",
                         s->tx.stage, s->tx.current_modulator);
                v = zero;
            }
            else
            {
                v = s->tx.current_getbaud(s);
                /* The Phase 2 getbaud handlers change stage at baud boundaries,
                   and several of those stages (the 10 ms !B windows) are shorter
                   than one v34_tx() block. Sampling for a stage change only at
                   the top of v34_tx() would drop them from the timeline, so
                   check here too, at the exact baud where the change happened. */
                v34_tx_log_state_change_at(s, sample);
            }
            /*endif*/
            s->tx.rrc_filter_re[s->tx.rrc_filter_step] = v.re;
            s->tx.rrc_filter_im[s->tx.rrc_filter_step] = v.im;
            if (++s->tx.rrc_filter_step >= V34_INFO_TX_FILTER_STEPS)
                s->tx.rrc_filter_step = 0;
            /*endif*/
        }
        /*endif*/
        /* Root raised cosine pulse shaping at baseband */
#if defined(SPANDSP_USE_FIXED_POINT)
        x.re = vec_circular_dot_prodi16(s->tx.rrc_filter_re, tx_pulseshaper[TX_PULSESHAPER_COEFF_SETS - 1 - s->tx.baud_phase], V34_INFO_TX_FILTER_STEPS, s->tx.rrc_filter_step) >> 4;
        x.im = vec_circular_dot_prodi16(s->tx.rrc_filter_im, tx_pulseshaper[TX_PULSESHAPER_COEFF_SETS - 1 - s->tx.baud_phase], V34_INFO_TX_FILTER_STEPS, s->tx.rrc_filter_step) >> 4;
        /* Now create and modulate the carrier */
        z = dds_complexi(&s->tx.carrier_phase, s->tx.cc_carrier_phase_rate);
        /* Don't bother saturating. We should never clip. */
        iamp = (x.re*z.re - x.im*z.im) >> 15;
        if (s->tx.guard_phase_rate  &&  (s->tx.rrc_filter[s->tx.rrc_filter_step].re != 0  ||  s->tx.rrc_filter[s->tx.rrc_filter_step].im != 0))
        {
            /* Add the guard tone */
            iamp += dds_mod(&s->tx.guard_phase, s->tx.guard_phase_rate, s->tx.cjo, 0);
        }
        /*endif*/
        amp[sample] = (int16_t) (((int32_t) iamp*s->tx.gain) >> 15);
#else
        x.re = vec_circular_dot_prodf(s->tx.rrc_filter_re, tx_pulseshaper[TX_PULSESHAPER_COEFF_SETS - 1 - s->tx.baud_phase], V34_INFO_TX_FILTER_STEPS, s->tx.rrc_filter_step);
        x.im = vec_circular_dot_prodf(s->tx.rrc_filter_im, tx_pulseshaper[TX_PULSESHAPER_COEFF_SETS - 1 - s->tx.baud_phase], V34_INFO_TX_FILTER_STEPS, s->tx.rrc_filter_step);
        /* Now create and modulate the carrier */
        z = dds_complexf(&s->tx.carrier_phase, s->tx.cc_carrier_phase_rate);
        famp = x.re*z.re - x.im*z.im;
        if (s->tx.guard_phase_rate  &&  (s->tx.rrc_filter_re[s->tx.rrc_filter_step] != 0.0f  ||  s->tx.rrc_filter_im[s->tx.rrc_filter_step] != 0.0f))
        {
            /* Add the guard tone */
            famp += dds_modf(&s->tx.guard_phase, s->tx.guard_phase_rate, s->tx.guard_level, 0);
        }
        /*endif*/
        /* Don't bother saturating. We should never clip. */
        amp[sample] = (int16_t) lfastrintf(famp*s->tx.gain);
#endif
    }
    return sample;
}
/*- End of function --------------------------------------------------------*/

static int tx_silence(v34_state_t *s, int16_t amp[], int max_len)
{
    if (s->tx.tone_duration <= max_len)
    {
        max_len = s->tx.tone_duration;
        s->tx.tone_duration = 0;
        if (s->tx.training_stage == 0x100)
        {
            s->tx.training_stage = 0x101;
            transmission_preamble_init(s);
        }
        /*endif*/
    }
    else
    {
        s->tx.tone_duration -= max_len;
    }
    /*endif*/
    vec_zeroi16(amp, max_len);
    return max_len;
}
/*- End of function --------------------------------------------------------*/

static void tx_silence_init(v34_state_t *s, int duration)
{
    s->tx.tone_duration = milliseconds_to_samples(duration);
    s->tx.current_modulator = V34_MODULATION_SILENCE;
    s->tx.current_getbaud = get_silence_baud;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_set_v90_u_info(v34_state_t *s, int u_info)
{
    if (s == NULL  ||  u_info < 0  ||  u_info > 127)
        return;
    /*endif*/
    s->tx.v90_u_info = u_info;
    span_log(&s->logging, SPAN_LOG_FLOW, "Tx - V.90 analogue role: U_INFO = %d\n", u_info);
}
/*- End of function --------------------------------------------------------*/

static complex_sig_t get_external_baud(v34_state_t *s)
{
    float re;
    float im;

    re = 0.0f;
    im = 0.0f;
    if (s->tx.external_symbol_func)
        s->tx.external_symbol_func(s->tx.external_symbol_user_data, &re, &im);
    /*endif*/
    /* The caller works in constellation steps; nominal symbol RMS belongs to
       the modulator, exactly as it does for the built-in training signals. */
    return complex_sig_set(TRAINING_SCALE(re*TRAINING_AMP), TRAINING_SCALE(im*TRAINING_AMP));
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_tx_start_external_symbols(v34_state_t *s,
                                                int baud_rate,
                                                int high_carrier,
                                                v34_tx_external_symbol_func_t fn,
                                                void *user_data)
{
    if (fn == NULL  ||  baud_rate < V34_BAUD_RATE_2400  ||  baud_rate > V34_BAUD_RATE_3429)
        return -1;
    /*endif*/
    s->tx.baud_rate = baud_rate;
    s->tx.high_carrier = high_carrier;
    s->tx.v34_carrier_phase_rate = dds_phase_ratef(carrier_frequency(s->tx.baud_rate, s->tx.high_carrier));
    /* Only the sample-rate-to-symbol-rate ratio is wanted here.  The bit rate
       is irrelevant to a symbol source that maps its own points, so ask for
       the lowest rate this symbol rate supports and take the timing from it. */
    v34_set_working_parameters(&s->tx.parms, s->tx.baud_rate, 2400, true);

#if defined(SPANDSP_USE_FIXED_POINT)
    vec_zeroi16(s->tx.rrc_filter_re, sizeof(s->tx.rrc_filter_re)/sizeof(s->tx.rrc_filter_re[0]));
    vec_zeroi16(s->tx.rrc_filter_im, sizeof(s->tx.rrc_filter_im)/sizeof(s->tx.rrc_filter_im[0]));
#else
    vec_zerof(s->tx.rrc_filter_re, sizeof(s->tx.rrc_filter_re)/sizeof(s->tx.rrc_filter_re[0]));
    vec_zerof(s->tx.rrc_filter_im, sizeof(s->tx.rrc_filter_im)/sizeof(s->tx.rrc_filter_im[0]));
#endif
    s->tx.rrc_filter_step = 0;
    s->tx.baud_phase = 0;
    s->tx.carrier_phase = 0;
    s->tx.external_symbol_func = fn;
    s->tx.external_symbol_user_data = user_data;
    s->tx.current_getbaud = get_external_baud;
    s->tx.current_modulator = V34_MODULATION_V34;
    s->tx.getbaud_null_logged = false;
    span_log(&s->logging, SPAN_LOG_FLOW,
             "Tx - external symbols: inherited gain %.4f (pre-emphasis idx %d)\n",
             (double)s->tx.gain, s->tx.pre_emphasis_idx);
    span_log(&s->logging, SPAN_LOG_FLOW,
             "Tx - external symbol source started (%d baud, %s carrier)\n",
             baud_rate_parameters[s->tx.baud_rate].baud_rate,
             high_carrier  ?  "high"  :  "low");
    return 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_v90_resume_external_symbols(v34_state_t *s,
                                                  v34_tx_external_symbol_func_t fn,
                                                  void *user_data)
{
    if (s == NULL  ||  fn == NULL)
        return -1;
    /* V.90 §9.6 requires downstream data-frame alignment to survive rate
       renegotiation.  Unlike v34_tx_start_external_symbols(), this seam must
       not zero carrier phase, baud phase, or the RRC history. */
    s->tx.external_symbol_func = fn;
    s->tx.external_symbol_user_data = user_data;
    s->tx.current_getbaud = get_external_baud;
    s->tx.current_modulator = V34_MODULATION_V34;
    s->tx.tx_data_mode = false;
    span_log(&s->logging, SPAN_LOG_FLOW,
             "Tx - V.90 rate renegotiation: preserving modulator phase for S/S-bar/CP\n");
    return 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_tx_stop_external_symbols(v34_state_t *s)
{
    s->tx.external_symbol_func = NULL;
    s->tx.external_symbol_user_data = NULL;
    if (s->tx.current_getbaud == get_external_baud)
    {
        /* Stay in the V.34 modulator putting out zero symbols rather than
           calling tx_silence_init(), which would restart the transmission
           preamble and drag the Phase 2 state machine back in. */
        s->tx.current_getbaud = get_silence_baud;
    }
    /*endif*/
    span_log(&s->logging, SPAN_LOG_FLOW, "Tx - external symbol source stopped\n");
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_tx(v34_state_t *s, int16_t amp[], int max_len)
{
    int len;
    int lenx;

    v34_tx_log_state_change(s);
    len = 0;
    lenx = -1;
    do
    {
        switch (s->tx.current_modulator)
        {
        case V34_MODULATION_V34:
            lenx = tx_v34_modulation(s, &amp[len], max_len - len);
            break;
        case V34_MODULATION_CC:
            lenx = tx_cc_modulation(s, &amp[len], max_len - len);
            break;
        case V34_MODULATION_L1_L2:
            lenx = tx_l1_l2(s, &amp[len], max_len - len);
            break;
        case V34_MODULATION_PCM_L1_L2:
            lenx = tx_pcm_l1_l2(s, &amp[len], max_len - len);
            break;
        case V34_MODULATION_SILENCE:
            lenx = tx_silence(s, &amp[len], max_len - len);
            break;
        }
        /*endswitch*/
        len += lenx;
        /* Add step by step, so each segment is seen up to date */
        s->tx.sample_time += lenx;
    }
    while (lenx > 0  &&  len < max_len);
    /* If the transmission is short, this should be the end of operation of the modem,
       so we don't really need to worry about the residue and keeping the sample time
       current. */
    return len;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_tx_power(v34_state_t *s, float power)
{
    /* The constellation design seems to keep the average power the same, regardless
       of which bit rate is in use.
       The leading factor is 1/V34_NOMINAL_SYMBOL_RMS: it only yields the requested
       dBm0 if the symbols reaching the RRC filter really do have that modulus RMS.
       It was previously the bare literal 0.223f, which let the training amplitude
       drift 6.97 dB away from it unnoticed - keep the two expressed in terms of
       each other. */
#if defined(SPANDSP_USE_FIXED_POINT)
    s->tx.gain = (1.0f/V34_NOMINAL_SYMBOL_RMS)*db_to_amplitude_ratio(power - DBM0_MAX_SINE_POWER)*16.0f*(32767.0f/30672.52f)*32768.0f/TX_PULSESHAPER_GAIN;
#else
    s->tx.gain = (1.0f/V34_NOMINAL_SYMBOL_RMS)*db_to_amplitude_ratio(power - DBM0_MAX_SINE_POWER)*32768.0f/TX_PULSESHAPER_GAIN;
#endif
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_set_get_bit(v34_state_t *s, span_get_bit_func_t get_bit, void *user_data)
{
    if (s->tx.get_bit == s->tx.current_get_bit)
        s->tx.current_get_bit = get_bit;
    /*endif*/
    s->tx.get_bit = get_bit;
    s->tx.get_bit_user_data = user_data;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_set_get_aux_bit(v34_state_t *s, span_get_bit_func_t get_bit, void *user_data)
{
    s->tx.get_aux_bit = get_bit;
    s->tx.get_aux_bit_user_data = user_data;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(logging_state_t *) v34_get_logging_state(v34_state_t *s)
{
    return &s->logging;
}
/*- End of function --------------------------------------------------------*/

void v34_set_working_parameters(v34_parameters_t *s, int baud_rate, int bit_rate, int expanded)
{
    /* This should be one of the normal V.34 modes. Not a control channel mode. */
    s->bit_rate = ((bit_rate >> 1) + 1)*2400 + (bit_rate & 1)*200;
    s->expanded_shaping = (expanded != 0);

    s->b = baud_rate_parameters[baud_rate].mappings[bit_rate].b;
    /* V.34/9.2 */
    if (s->b <= 12)
    {
        /* There are so few bits per mapping frame, that there are only I bits */
        s->k = 0;
        s->q = 0;
    }
    else
    {
        /* We have some K bits and maybe some Q bits */
        /* The baseline for K is the total bits less the I bits */
        s->k = s->b - 12;
        s->q = 0;
        /* If there are too many k bits, we need to trade some of them for
           uncoded Q bits, until the number of K bits is in range. We add
           Q bits in groups of 8, as the rule is each of the Q bit chunks
           in the 8 2D symbols of a mapping frame is the same size. */
        while (s->k >= 32)
        {
            s->k -= 8;
            s->q++;
        }
        /*endwhile*/
    }
    /*endif*/
    s->q_mask = ((1 << s->q) - 1);

    /* Calculating m, as described in V.34/9.2, doesn't always match the values in
       V.34/Table 10, so we use a table, to ensure an exact match. */
    s->m = baud_rate_parameters[baud_rate].mappings[bit_rate].m[(expanded)  ?  1  :  0];

    /* l is easy to calculate from m. We don't need to get it from a table, as
       shown in V.34/Table 10. */
    s->l = 4*s->m*(1 << s->q);
    s->j = baud_rate_parameters[baud_rate].j;
    s->p = baud_rate_parameters[baud_rate].p;
    /* We don't need to use a table entry for w. It is trivial to calculate it from j */
    s->w = (bit_rate & 1)  ?  (15 - s->j)  :  0;
    /* V.34/8.2 */
    s->r = (s->bit_rate*28)/(s->j*100) - (s->b - 1)*s->p;
    
    s->max_bit_rate_code = baud_rate_parameters[baud_rate].max_bit_rate_code;
    /* The numerator of the number of samples per symbol ratio. */
    s->samples_per_symbol_numerator = baud_rate_parameters[baud_rate].samples_per_symbol_numerator;
    /* The denominator of the number of samples per symbol ratio. */
    s->samples_per_symbol_denominator = baud_rate_parameters[baud_rate].samples_per_symbol_denominator;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_seed_tx_data(v34_state_t *s,
                                   int bit_rate_n,
                                   int trellis_size,
                                   int use_non_linear_encoder,
                                   int expanded_shaping,
                                   const int16_t precoder_coeffs[6])
{
    const uint8_t (*table)[16];

    if (!s || bit_rate_n < 1 || bit_rate_n > 14)
        return -1;
    switch (trellis_size)
    {
    case V34_TRELLIS_16:
        table = v34_conv16_encode_table;
        break;
    case V34_TRELLIS_32:
        table = v34_conv32_encode_table;
        break;
    case V34_TRELLIS_64:
        table = v34_conv64_encode_table;
        break;
    default:
        return -1;
    }
    s->tx.bit_rate = (bit_rate_n - 1)*2;
    v34_set_working_parameters(&s->tx.parms,
                               s->tx.baud_rate,
                               s->tx.bit_rate,
                               expanded_shaping != 0);
    s->tx.conv_encode_table = table;
    s->tx.use_non_linear_encoder = (use_non_linear_encoder != 0);
    s->tx.scramble_reg = 0;
    s->tx.diff = 0;
    s->tx.r0 = 0;
    memset(s->tx.qbits, 0, sizeof(s->tx.qbits));
    memset(s->tx.ibits, 0, sizeof(s->tx.ibits));
    memset(s->tx.mjk, 0, sizeof(s->tx.mjk));
    memset(s->tx.x, 0, sizeof(s->tx.x));
    memset(s->tx.precoder_coeffs, 0, sizeof(s->tx.precoder_coeffs));
    if (precoder_coeffs)
    {
        for (int i = 0;  i < 3;  i++)
        {
            s->tx.precoder_coeffs[i].re = precoder_coeffs[2*i];
            s->tx.precoder_coeffs[i].im = precoder_coeffs[2*i + 1];
        }
    }
    s->tx.c.re = 0;
    s->tx.c.im = 0;
    s->tx.p.re = 0;
    s->tx.p.im = 0;
    s->tx.z = 0;
    s->tx.y0 = 0;
    s->tx.state = 0;
    s->tx.step_2d = 0;
    /* The first seeded frame is B1. V.34 10.1.3.1 assigns B1 the inversion
       state of the final data frame of a superframe. */
    s->tx.super_frame = s->tx.parms.j > 0 ? s->tx.parms.j - 1 : 0;
    s->tx.data_frame = 0;
    s->tx.s_bit_cnt = 0;
    s->tx.aux_bit_cnt = 0;
    s->tx.v0_pattern = s->tx.parms.j > 0
                     ? (uint16_t)(2*(s->tx.parms.j - 1)) : 0;
    s->tx.data_symbol_scale = 1.0f;
    s->tx.current_get_bit = s->tx.get_bit;
    return 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_v90_begin_tx_data(v34_state_t *s,
                                        int bit_rate_n,
                                        int trellis_size,
                                        int use_non_linear_encoder,
                                        int expanded_shaping,
                                        const int16_t precoder_coeffs[6])
{
    /* V.90 §8.5.1: B1 and upstream data use the analogue-modem GPA
       scrambler (1 + x^-5 + x^-23), i.e. zero-based tap 4.  Do not inherit
       the answer-modem/downstream GPC tap from an earlier role. */
    s->tx.scrambler_tap = 4;
    if (v34_seed_tx_data(s, bit_rate_n, trellis_size,
                         use_non_linear_encoder, expanded_shaping,
                         precoder_coeffs) != 0)
        return -1;
    /* V.90 §8.5.1/§9.4.2.5 uses V.34's B1: the first data frame is scrambled
       ones with every data-mode state reset.  get_data_baud() already does
       exactly that before returning to the normal get-bit callback.  Preserve
       carrier phase and pulse-shaper history from E; only replace the external
       symbol source and mapper. */
    s->tx.external_symbol_func = NULL;
    s->tx.external_symbol_user_data = NULL;
    s->tx.tx_mapping_frame_step = 0;
    s->tx.b1_frames_sent = 0;
    /* §10.1.3.1/V.34: B1 carries the superframe inversion state of the
       final data frame.  v34_begin_rx_data() enters with this same state. */
    if (s->tx.parms.j > 0)
    {
        s->tx.super_frame = s->tx.parms.j - 1;
        s->tx.v0_pattern = (uint16_t)(2*(s->tx.parms.j - 1));
    }
    s->tx.current_modulator = V34_MODULATION_V34;
    s->tx.current_getbaud = get_data_baud;
    s->tx.tx_data_mode = true;
    s->primary_channel_active = true;
    span_log(&s->logging, SPAN_LOG_FLOW,
             "Tx - V.90 analogue handover to B1/data: N=%d (%d bps), "
             "trellis=%d nonlinear=%d expanded=%d\n",
             bit_rate_n, bit_rate_n*2400, trellis_size,
             use_non_linear_encoder != 0, expanded_shaping != 0);
    return 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_get_current_bit_rate(v34_state_t *s)
{
    return s->bit_rate;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(bool) v34_get_primary_channel_active(v34_state_t *s)
{
    return s->primary_channel_active;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_get_rx_stage(v34_state_t *s)
{
    return s->rx.stage;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(float) v34_get_guard_carrier_db(v34_state_t *s, int *valid)
{
    /* Level of the peer's 1800 Hz guard tone relative to its 2400 Hz carrier.
       V.34 10.1.2.1/10.1.2.3 fix both, so this says which state the peer is in:
       about +1 dB while it holds Tone A, about -6 dB while it transmits an INFO
       sequence.  Exposed so the engine can log it continuously -- it was
       previously only reported when the 9.2.1.2.6 deadline branch fired, and a
       whole 16-call series passed without any call reaching that deadline, so
       the measurement went unvalidated. */
    if (valid)
        *valid = s ? s->rx.guard_carrier_valid : 0;
    /*endif*/
    return s ? s->rx.guard_carrier_db : 0.0f;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_get_rx_baud_rate(v34_state_t *s)
{
    if (!s)
        return -1;
    return s->rx.baud_rate;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_get_rx_high_carrier(v34_state_t *s)
{
    return s ? (s->rx.high_carrier ? 1 : 0) : -1;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_get_tx_baud_rate(v34_state_t *s)
{
    return s ? s->tx.baud_rate : -1;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_get_tx_high_carrier(v34_state_t *s)
{
    return s ? s->tx.high_carrier : -1;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_get_round_trip_delay_samples(v34_state_t *s)
{
    return (s && s->rx.round_trip_delay_estimate > 0)
         ? s->rx.round_trip_delay_estimate : 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_get_v90_tx_u_info(v34_state_t *s)
{
    return (s && s->tx.v90_mode && s->calling_party) ? s->tx.v90_u_info : 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_get_tx_stage(v34_state_t *s)
{
    return s->tx.stage;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_get_phase3_j_bits(v34_state_t *s)
{
    return s->rx.phase3_j_bits;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_get_phase3_j_trn16(v34_state_t *s)
{
    return s->rx.phase3_j_trn16;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_get_phase3_trn_lock_score(v34_state_t *s)
{
    return s->rx.phase3_trn_lock_score;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_get_tx_data_mode(v34_state_t *s)
{
    return s->tx.tx_data_mode ? 1 : 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_set_mp_rate_policy(v34_state_t *s, int bit_rate_a_to_c, int bit_rate_c_to_a)
{
    if (!s)
        return;
    /*endif*/
    if (!mp_rate_n_is_valid(bit_rate_a_to_c) || !mp_rate_n_is_valid(bit_rate_c_to_a))
        return;
    /*endif*/
    s->tx.mp_rate_policy_valid = true;
    s->tx.mp_rate_a_to_c = bit_rate_a_to_c;
    s->tx.mp_rate_c_to_a = bit_rate_c_to_a;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_clear_mp_rate_policy(v34_state_t *s)
{
    if (!s)
        return;
    /*endif*/
    s->tx.mp_rate_policy_valid = false;
    s->tx.mp_rate_a_to_c = 0;
    s->tx.mp_rate_c_to_a = 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_get_v90_u_info(v34_state_t *s)
{
    return s->rx.info1a.max_data_rate;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_get_v90_received_info0a(v34_state_t *s, v34_v90_info0a_t *info)
{
    if (!s || !info || !s->rx.v90_mode)
        return 0;

    info->support_2743 = s->rx.far_capabilities.support_baud_rate_low_carrier[V34_BAUD_RATE_2743];
    info->support_2800 = s->rx.far_capabilities.support_baud_rate_low_carrier[V34_BAUD_RATE_2800];
    info->support_3429 = s->rx.far_capabilities.support_baud_rate_low_carrier[V34_BAUD_RATE_3429];
    info->support_3000_low = s->rx.far_capabilities.support_baud_rate_low_carrier[V34_BAUD_RATE_3000];
    info->support_3000_high = s->rx.far_capabilities.support_baud_rate_high_carrier[V34_BAUD_RATE_3000];
    info->support_3200_low = s->rx.far_capabilities.support_baud_rate_low_carrier[V34_BAUD_RATE_3200];
    info->support_3200_high = s->rx.far_capabilities.support_baud_rate_high_carrier[V34_BAUD_RATE_3200];
    info->rate_3429_allowed = s->rx.far_capabilities.rate_3429_allowed;
    info->support_power_reduction = s->rx.far_capabilities.support_power_reduction;
    info->max_baud_rate_difference = s->rx.far_capabilities.max_baud_rate_difference;
    info->from_cme_modem = s->rx.far_capabilities.from_cme_modem;
    info->support_1664_point_constellation = s->rx.far_capabilities.support_1664_point_constellation;
    info->tx_clock_source = s->rx.far_capabilities.tx_clock_source;
    info->acknowledge_info0d = s->rx.info0_acknowledgement;
    info->raw_26_27 = s->rx.info0_raw_26_27;
    info->info0d_nominal_power_code = s->rx.info0d_nominal_power_code;
    info->info0d_max_power_code = s->rx.info0d_max_power_code;
    info->info0d_power_measured_at_codec_output = s->rx.info0d_power_measured_at_codec_output;
    info->info0d_pcm_alaw = s->rx.info0d_pcm_alaw;
    info->info0d_upstream_3429_support = s->rx.info0d_upstream_3429_support;
    info->info0d_reserved_41 = s->rx.info0d_reserved_41;
    info->info0d_extensions_valid = s->rx.info0d_extensions_valid;
    return s->rx.info0_received ? 1 : 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_get_v90_received_info1a(v34_state_t *s, v34_v90_info1a_t *info)
{
    if (!s || !info || !s->rx.v90_mode)
        return 0;

    info->md = s->rx.info1a.md;
    info->u_info = s->rx.info1a.max_data_rate;
    info->upstream_symbol_rate_code = s->rx.info1a.baud_rate_a_to_c;
    info->downstream_rate_code = s->rx.info1a.baud_rate_c_to_a;
    info->freq_offset = s->rx.info1a.freq_offset;
    info->raw_12_17 = s->rx.info1a_raw_12_17;
    info->raw_32_33 = s->rx.info1a_raw_32_33;
    info->raw_40_49 = s->rx.info1a_raw_40_49;
    return s->rx.info1a_received ? 1 : 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_get_v90_received_info1d(v34_state_t *s, v34_v90_info1d_t *info)
{
    int i;

    if (!s || !info || !s->rx.v90_mode)
        return 0;

    info->power_reduction = s->rx.info1c.power_reduction;
    info->additional_power_reduction = s->rx.info1c.additional_power_reduction;
    info->md = s->rx.info1c.md;
    info->freq_offset = s->rx.info1c.freq_offset;
    for (i = 0; i < 6; i++) {
        info->rate_data[i].use_high_carrier = s->rx.info1c.rate_data[i].use_high_carrier;
        info->rate_data[i].pre_emphasis = s->rx.info1c.rate_data[i].pre_emphasis;
        info->rate_data[i].max_bit_rate = s->rx.info1c.rate_data[i].max_bit_rate;
    }
    return s->rx.info1c_received ? 1 : 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_force_phase4(v34_state_t *s)
{
    if (!s)
        return;
    if (s->tx.current_getbaud == get_phase4_baud
        || s->tx.stage >= V34_TX_STAGE_PHASE4_WAIT)
    {
        span_log(&s->logging, SPAN_LOG_FLOW,
                 "Tx - v34_force_phase4(): already in Phase 4 path (stage=%d)\n",
                 s->tx.stage);
        return;
    }
    span_log(&s->logging, SPAN_LOG_FLOW,
             "Tx - v34_force_phase4(): external V.90 Phase 3 complete, handing TX/RX to native Phase 4\n");
    phase4_wait_init(s);
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_v90_start_analogue_retrain(v34_state_t *s)
{
    if (!s || !s->tx.v90_mode || !s->calling_party)
        return;
    /* V.90 §9.5.2.1/.2: both analogue procedures start with 70 ms silence
       followed by Tone A.  The ordinary initial preamble is deliberately
       skipped because §9.5 resumes at §9.2.2.1.3, after INFO0. */
    span_log(&s->logging, SPAN_LOG_FLOW,
             "Tx - V.90 analogue retrain armed; 70 ms silence then Tone A (9.5.2)\n");
    v90_phase2_reset_transactions(s);
    s->tx.current_modulator = V34_MODULATION_CC;
    s->tx.current_getbaud = get_v90_wait_info1a_baud;
    s->tx.tone_duration = 0;
    s->tx.stage = V34_TX_STAGE_V90_RETRAIN_SILENCE;
    s->rx.received_event = V34_EVENT_NONE;
    s->rx.persistence1 = 0;
    s->rx.persistence2 = 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_v90_start_retrain_response(v34_state_t *s)
{
    if (!s  ||  !s->tx.v90_mode  ||  s->calling_party)
        return;
    /*endif*/
    /* V.90 9.5.1.2: answer a peer-initiated retrain with 70 +/- 5 ms of
       silence and then Tone B, receiver conditioned for the Tone A phase
       reversal -- the INFO0 exchange is skipped on a retrain (9.5.2.1 sends
       the analogue modem straight back to the 9.2.2.1.3 tone ranging, and
       SmartLink then searches directly for INFO1d after L1/L2).  Restarting
       from INITIAL_PREAMBLE/INFO0d instead leaves the peer's Tone B detector
       facing a modulated INFO0d carrier during its L2 window, which it does
       not reliably accept (observed live 2026-07-22: post-Phase-4 retrains
       aborted in DET_AB 1.3 s in).  This enters the same
       V90_RETRAIN_SILENCE -> Tone B -> PHASE2_B_INFO0_SEEN path the INFO1a
       deadline handler uses; repeated INFO0a from peers that do re-run INFO0
       still triggers the acknowledged-INFO0d recovery from that state. */
    span_log(&s->logging, SPAN_LOG_FLOW,
             "Tx - V.90: peer retrain response armed; 70 ms silence then Tone B (9.5.1.2)\n");
    v90_phase2_reset_transactions(s);
    s->tx.current_modulator = V34_MODULATION_CC;
    s->tx.current_getbaud = get_v90_wait_info1a_baud;
    s->tx.tone_duration = 0;
    s->tx.stage = V34_TX_STAGE_V90_RETRAIN_SILENCE;
    s->rx.received_event = V34_EVENT_NONE;
    s->rx.persistence1 = 0;
    s->rx.persistence2 = 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_force_phase3_rx(v34_state_t *s)
{
    if (!s)
        return;
    s_not_s_baud_init(s);
    /* Offline replay has already decoded INFO1 outside this context.  The
       normal V.90 answerer branch otherwise remains on the 600-baud control
       channel because rx.info1a is intentionally empty here. */
    s->primary_channel_active = true;
    s->rx.current_demodulator = V34_MODULATION_V34;
    s->rx.stage = V34_RX_STAGE_PHASE3_TRAINING;
    s->rx.duration = 0;
    s->rx.received_event = V34_EVENT_NONE;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_set_v90_mode(v34_state_t *s, int pcm_law)
{
    s->tx.v90_mode = true;
    s->tx.v90_pcm_law = pcm_law;
    s->tx.v90_l2_count = 0;
    s->tx.v90_info1a_retrain_responses = 0;
    s->rx.v90_mode = true;
    v90_phase2_reset_transactions(s);

    if (s->calling_party)
    {
        /* V.90 §8.2.3.1: analog (calling) modem TX CC at 2400 Hz, RX CC at 1200 Hz.
           This is different from standard V.34 where both sides use 1200 Hz.
           The analog modem transmits INFO0a and Phase 2 tones at 2400 Hz.
           It receives INFO0d from the digital answerer at 1200 Hz. */
        s->tx.cc_carrier_phase_rate = dds_phase_ratef(2400.0f);
        s->rx.cc_carrier_phase_rate = dds_phase_ratef(1200.0f);

        /* Re-prime caller RX for INFO0d detection (62 bits instead of 49). */
        if (s->tx.stage == 0
            || s->tx.stage == V34_TX_STAGE_INITIAL_PREAMBLE
            || s->tx.stage == V34_TX_STAGE_INFO0
            || s->tx.stage == V34_TX_STAGE_INFO0_RETRY)
        {
            s->rx.stage = V34_RX_STAGE_INFO0;
            s->rx.current_demodulator = V34_MODULATION_TONES;
            /* INFO0d is 62 bits: 12 fill/sync + 30 data + 16 CRC + 4 fill.
               target_bits counts data+CRC = 62 - 12 - 4 = 46. */
            s->rx.target_bits = 62 - (4 + 8 + 4);
            s->rx.bit_count = 0;
            s->rx.bitstream = 0;
            s->rx.info0_received = false;
            s->rx.info0_acknowledgement = false;
            s->rx.received_event = V34_EVENT_NONE;
            s->rx.persistence1 = 0;
            s->rx.persistence2 = 0;
            s->rx.last_logged_stage = -1;
            s->rx.last_logged_event = -1;
            s->rx.last_logged_demodulator = -1;
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "V.90 caller mode: re-primed RX for INFO0d detection (62 bits)\n");
        }
        /*endif*/

        /* V.8 already enforces the mandatory 75 ms post-CJ silence before
           handing control to Phase 2. Skip the generic V.34 startup silence. */
        if (s->tx.training_stage == 0x100
            && s->tx.current_modulator == V34_MODULATION_SILENCE)
        {
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "V.90 caller mode: skipping duplicate Phase 2 startup silence\n");
            s->tx.tone_duration = 0;
            s->tx.training_stage = 0x101;
            transmission_preamble_init(s);
        }
        /*endif*/
    }
    else
    {
        /* V.90 §8.2.3.1: digital modem TX CC at 1200 Hz, RX CC at 2400 Hz.
           Must update carrier phase rates here since v34_init/restart already ran
           with v90_mode=false and set standard V.34 carriers. */
        s->tx.cc_carrier_phase_rate = dds_phase_ratef(1200.0f);
        /* V.90 §8.2.3.1: digital modem transmits at 1200 Hz at nominal power.
           NO guard tone — the 1800 Hz guard tone is only for the analog modem
           (which transmits at 2400 Hz). Disable any guard tone set by V.34 init. */
        s->tx.guard_phase_rate = 0;
        s->tx.guard_level = 0;
        s->rx.cc_carrier_phase_rate = dds_phase_ratef(2400.0f);

        /* We enable V.90 after a plain V.34 init/restart. Re-prime the early
           answerer RX path here so INFO0a detection starts from a known state
           even if the startup state was left stale during the V.34->V.90 handoff. */
        if (s->tx.stage == 0
            || s->tx.stage == V34_TX_STAGE_INITIAL_PREAMBLE
            || s->tx.stage == V34_TX_STAGE_INFO0
            || s->tx.stage == V34_TX_STAGE_INFO0_RETRY)
        {
            s->rx.stage = V34_RX_STAGE_INFO0;
            s->rx.current_demodulator = V34_MODULATION_TONES;
            s->rx.target_bits = (s->rx.duplex)  ?  (49 - (4 + 8 + 4))  :  (51 - (4 + 8 + 4));
            s->rx.bit_count = 0;
            s->rx.bitstream = 0;
            s->rx.info0_received = false;
            s->rx.info0_acknowledgement = false;
            s->rx.received_event = V34_EVENT_NONE;
            s->rx.persistence1 = 0;
            s->rx.persistence2 = 0;
            s->rx.last_logged_stage = -1;
            s->rx.last_logged_event = -1;
            s->rx.last_logged_demodulator = -1;
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "V.90 mode: re-primed answerer RX for INFO0 detection\n");
        }
        /*endif*/

        /* V.8 already enforces the mandatory 75 ms post-CJ silence before
           handing control to Phase 2. Skip the generic V.34 startup silence
           here so INFO0d is not delayed by an extra 75 ms on V.90 answerer
           handoff. */
        if (s->tx.training_stage == 0x100
            && s->tx.current_modulator == V34_MODULATION_SILENCE)
        {
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "V.90 mode: skipping duplicate Phase 2 startup silence; V.8 already supplied 75 ms\n");
            s->tx.tone_duration = 0;
            s->tx.training_stage = 0x101;
            transmission_preamble_init(s);
        }
        /*endif*/
    }

    span_log(&s->logging, SPAN_LOG_FLOW,
             "V.90 mode enabled (%s, PCM law: %s)\n",
             s->calling_party ? "caller/analog" : "answerer/digital",
             pcm_law ? "A-law" : "u-law");
}

SPAN_DECLARE(void) v34_set_v92_info0_capabilities(v34_state_t *s,
                                                   int v92_capable,
                                                   int short_phase2_requested)
{
    if (!s)
        return;
    s->tx.v92_info0_capable = v92_capable != 0;
    s->tx.v92_short_phase2_requested = short_phase2_requested != 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_set_v92_pcm_upstream_capability(v34_state_t *s,
                                                        int pcm_upstream_capable)
{
    if (!s)
        return;
    s->tx.v92_pcm_upstream_capable = pcm_upstream_capable != 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_half_duplex_change_mode(v34_state_t *s, int mode)
{
    switch (mode)
    {
    case V34_HALF_DUPLEX_SOURCE:
    case V34_HALF_DUPLEX_RECIPIENT:
        s->rx.half_duplex_source =
        s->tx.half_duplex_source =
        s->half_duplex_source = mode;
        break;
    case V34_HALF_DUPLEX_CONTROL_CHANNEL:
        s->rx.half_duplex_state =
        s->tx.half_duplex_state =
        s->half_duplex_state = mode;
        break;
    case V34_HALF_DUPLEX_PRIMARY_CHANNEL:
        s->rx.half_duplex_state =
        s->tx.half_duplex_state =
        s->half_duplex_state = mode;
        break;
    case V34_HALF_DUPLEX_SILENCE:
        s->rx.half_duplex_state =
        s->tx.half_duplex_state =
        s->half_duplex_state = mode;
        break;
    }
    /*endswitch*/

    return 0;
}
/*- End of function --------------------------------------------------------*/

static int v34_tx_restart(v34_state_t *s, int baud_rate, int bit_rate, int high_carrier)
{
    s->tx.bit_rate = bit_rate;
    s->tx.baud_rate = baud_rate;
    s->tx.high_carrier = high_carrier;
    s->tx.info0_acknowledgement = false;
    s->tx.info0_retry_count = 0;
    s->tx.v90_info1a_fast_retries = 0;
    s->tx.v90_info1a_total_retries = 0;
    s->tx.v90_phase2_info0_recovery_loops = 0;
    s->tx.phase3_call_wait_j = false;
    /* -1, not 0, so stage-change logging can tell "not started yet" from
       "started at sample 0" and does not report a Phase 2 elapsed time before
       Phase 2 has actually begun. */
    s->tx.stage_entry_sample_time = -1;
    s->tx.phase2_entry_sample_time = -1;

    s->tx.v34_carrier_phase_rate = dds_phase_ratef(carrier_frequency(s->tx.baud_rate, s->tx.high_carrier));
    if (s->calling_party && s->tx.v90_mode)
    {
        /* V.90 §8.2.3.1: analog (calling) modem transmits INFO at 2400 Hz */
        s->tx.cc_carrier_phase_rate = dds_phase_ratef(2400.0f);
        s->tx.guard_phase_rate = 0;
        s->tx.guard_level = 0.0f;
    }
    else if (s->calling_party)
    {
        s->tx.cc_carrier_phase_rate = dds_phase_ratef(1200.0f);
        s->tx.guard_phase_rate = 0;
        s->tx.guard_level = 0.0f;
    }
    else if (s->tx.v90_mode)
    {
        /* V.90 §8.2.3.1: digital modem (answerer) transmits INFO at 1200 Hz */
        s->tx.cc_carrier_phase_rate = dds_phase_ratef(1200.0f);
        s->tx.guard_phase_rate = 0;
        s->tx.guard_level = 0.0f;
    }
    else
    {
        s->tx.cc_carrier_phase_rate = dds_phase_ratef(2400.0f);
        s->tx.guard_phase_rate = 0; //dds_phase_ratef(1800.0f);
        s->tx.guard_level = 4.0f;
    }
    /*endif*/
    v34_set_working_parameters(&s->tx.parms, s->tx.baud_rate, s->tx.bit_rate, true);
    /* The constructor/restart rate is the local capability ceiling, not just
       an initial mapper choice.  10.1.2.3.4 may project lower rates from L2,
       but must not advertise above the configured ceiling. */
    s->tx.parms.max_bit_rate_code = bit_rate;

#if defined(SPANDSP_USE_FIXED_POINT)
    vec_zeroi16(s->tx.rrc_filter_re, sizeof(s->tx.rrc_filter_re)/sizeof(s->tx.rrc_filter_re[0]));
    vec_zeroi16(s->tx.rrc_filter_im, sizeof(s->tx.rrc_filter_im)/sizeof(s->tx.rrc_filter_im[0]));
#else
    vec_zerof(s->tx.rrc_filter_re, sizeof(s->tx.rrc_filter_re)/sizeof(s->tx.rrc_filter_re[0]));
    vec_zerof(s->tx.rrc_filter_im, sizeof(s->tx.rrc_filter_im)/sizeof(s->tx.rrc_filter_im[0]));
#endif
    s->tx.lastbit = complex_sig_set(TRAINING_SCALE(0.0f), TRAINING_SCALE(0.0f));
    s->tx.rrc_filter_step = 0;
    s->tx.convolution = 0;
    s->tx.scramble_reg = 0;
    s->tx.carrier_phase = 0;

    s->tx.txbits = 0;
    s->tx.txptr = 0;
    s->tx.diff = 0;

    s->tx.line_probe_step = 0;
    s->tx.line_probe_cycles = 0;
    s->tx.line_probe_scaling = 0.0008f*V34_LINE_PROBE_LEVEL_TRIM*s->tx.gain;

    s->tx.training_stage = 0x100;
    tx_silence_init(s, 75);

    s->tx.v0_pattern = 0;
    s->tx.super_frame = 0;
    s->tx.data_frame = 0;
    s->tx.s_bit_cnt = 0;
    s->tx.aux_bit_cnt = 0;

    s->tx.conv_encode_table = v34_conv16_encode_table;

    s->tx.current_get_bit = s->tx.get_bit;
    s->tx.last_logged_stage = -1;
    s->tx.last_logged_modulator = -1;
    return 0;
}
/*- End of function --------------------------------------------------------*/

static int bit_rate_to_code(int bit_rate)
{
    int code;
    int rate;

    /* Translate between the bit rate as an integer and an internal code that
       represents the N*2400 bps and the possible extra 200 bps for auxilliary data. */
    if (bit_rate > 36800)
        return -1;
    /*endif*/
    code = bit_rate/2400;
    rate = code*2400;
    code = (code - 1) << 1;
    if (rate == bit_rate)
        return code;
    /*endif*/
    if ((rate + 200) == bit_rate)
        return (code | 1);
    /*endif*/
    return -1;
}
/*- End of function --------------------------------------------------------*/

static int baud_rate_to_code(int baud_rate)
{
    int i;

    /* Translate between the baud rate, as the integer nearest approaximation to the
       actual baud rate, and a 0-5 code used internally */
    for (i = 0;  i < 6;  i++)
    {
        if (baud_rate_parameters[i].baud_rate == baud_rate)
            return i;
        /*endif*/
    }
    /*endfor*/
    return -1;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_restart(v34_state_t *s, int baud_rate, int bit_rate, bool duplex)
{
    int bit_rate_code;
    int baud_rate_code;
    int high_carrier;

    span_log(&s->logging, SPAN_LOG_FLOW, "Tx - Restarting V.34, %d baud, %dbps\n", baud_rate, bit_rate);
    if ((bit_rate_code = bit_rate_to_code(bit_rate)) < 0)
        return -1;
    /*endif*/
    if ((baud_rate_code = baud_rate_to_code(baud_rate)) < 0)
        return -1;
    /*endif*/
    /* Check the bit rate and baud rate combination is valid */
    if (baud_rate_parameters[baud_rate_code].mappings[bit_rate_code].b == 0)
        return -1;
    /*endif*/
    s->duplex =
    s->rx.duplex =
    s->tx.duplex = duplex;

    /* Select the default half-duplex configuration */
    s->rx.half_duplex_source =
    s->tx.half_duplex_source =
    s->half_duplex_source = (s->calling_party)  ?  V34_HALF_DUPLEX_SOURCE  :  V34_HALF_DUPLEX_RECIPIENT;

    /* V.34 §3.2: In duplex mode, the calling modem uses the higher frequency
       carrier for transmission and the answering modem uses the lower frequency.
       TX and RX must use DIFFERENT carriers so full-duplex signals don't collide.
         Caller:   TX=high, RX=low  (receives answerer's low-carrier TX)
         Answerer: TX=low,  RX=high (receives caller's high-carrier TX)
       In half-duplex, both TX and RX use the same carrier. */
    if (duplex)
    {
        int tx_high = s->calling_party ? true : false;
        int rx_high = s->calling_party ? false : true;
        span_log(&s->logging, SPAN_LOG_FLOW,
                 "Tx - Carrier assignment: %s, TX=%s (%.0f Hz), RX=%s (%.0f Hz)\n",
                 s->calling_party ? "caller" : "answerer",
                 tx_high ? "high" : "low",
                 carrier_frequency(baud_rate_code, tx_high),
                 rx_high ? "high" : "low",
                 carrier_frequency(baud_rate_code, rx_high));
        v34_tx_restart(s, baud_rate_code, bit_rate_code, tx_high);
        v34_rx_restart(s, baud_rate_code, bit_rate_code, rx_high);
    }
    else
    {
        high_carrier = true;
        v34_tx_restart(s, baud_rate_code, bit_rate_code, high_carrier);
        v34_rx_restart(s, baud_rate_code, bit_rate_code, high_carrier);
    }

    return 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(v34_state_t *) v34_init(v34_state_t *s,
                                     int baud_rate,
                                     int bit_rate,
                                     bool calling_party,
                                     bool duplex,
                                     span_get_bit_func_t get_bit,
                                     void *get_bit_user_data,
                                     span_put_bit_func_t put_bit,
                                     void *put_bit_user_data)
{
    int bit_rate_code;
    int baud_rate_code;

    if ((baud_rate_code = baud_rate_to_code(baud_rate)) < 0)
        return NULL;
    /*endif*/
    if ((bit_rate_code = bit_rate_to_code(bit_rate)) < 0)
        return NULL;
    /*endif*/
    /* Check the bit rate and baud rate combination is valid */
    if (baud_rate_parameters[baud_rate_code].mappings[bit_rate_code].b == 0)
        return NULL;
    /*endif*/
    if (s == NULL)
    {
        if ((s = (v34_state_t *) span_alloc(sizeof(*s))) == NULL)
            return NULL;
        /*endif*/
    }
    /*endif*/
    memset(s, 0, sizeof(*s));
    span_log_init(&s->logging, SPAN_LOG_NONE, NULL);
    span_log_set_protocol(&s->logging, "V.34");
    s->rx.logging = &s->logging;
    s->tx.logging = &s->logging;
    s->bit_rate = bit_rate;
    s->calling_party =
    s->rx.calling_party =
    s->tx.calling_party = calling_party;

    s->rx.stage = V34_RX_STAGE_INFO0;

    s->tx.get_bit = get_bit;
    s->tx.get_bit_user_data = get_bit_user_data;
    v34_tx_power(s, -14.0f);
    v34_restart(s, baud_rate, bit_rate, duplex);

    s->rx.put_bit = put_bit;
    s->rx.put_bit_user_data = put_bit_user_data;
    v34_rx_set_signal_cutoff(s, -45.5f);
    s->rx.agc_scaling = 0.0017f/V34_RX_PULSESHAPER_GAIN;
    s->rx.agc_scaling_save = 0.0f;
    s->rx.carrier_phase_rate_save = 0;

    if (calling_party)
    {
        s->tx.scrambler_tap = 17;
        s->rx.scrambler_tap = 4;
    }
    else
    {
        s->tx.scrambler_tap = 4;
        s->rx.scrambler_tap = 17;
    }
    /*endif*/
    return s;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_release(v34_state_t *s)
{
    return 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_free(v34_state_t *s)
{
    /* The upstream symbol dump, if ME_V90_UPSTREAM_SYM_DUMP opened one.
       v34_free() accepted NULL before this was added, and callers rely on
       that. */
    if (s == NULL)
        return 0;
    /*endif*/
    if (s->rx.v90_t3_sym_dump)
    {
        fclose(s->rx.v90_t3_sym_dump);
        s->rx.v90_t3_sym_dump = NULL;
    }
    /*endif*/
    span_free(s);
    return 0;
}
/*- End of function --------------------------------------------------------*/
/*- End of file ------------------------------------------------------------*/
