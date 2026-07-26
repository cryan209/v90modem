/*
 * SpanDSP - a series of DSP components for telephony
 *
 * v34rx.c - ITU V.34 modem, receive part
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
#include <stddef.h>
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

#include "spandsp/v29rx.h"
#include "spandsp/v34.h"

#include "spandsp/private/bitstream.h"
#include "spandsp/private/logging.h"
#include "spandsp/private/modem_echo.h"
#include "spandsp/private/power_meter.h"
#include "spandsp/private/v34.h"

#ifndef V34_TRACE_DIAGNOSTICS
#define V34_TRACE_DIAGNOSTICS 0
#endif

/* Phase 3 RX audio capture for offline analysis */
static FILE *phase3_rx_dump_fp = NULL;
static int phase3_rx_dump_count = 0;
#define PHASE3_RX_DUMP_SAMPLES 24000  /* 3 seconds */

#include "v22bis_rx_1200_rrc.h"
#include "v22bis_rx_2400_rrc.h"

#include "v34_rx_2400_low_carrier_rrc.h"
#include "v34_rx_2400_high_carrier_rrc.h"
#include "v34_rx_2743_low_carrier_rrc.h"
#include "v34_rx_2743_high_carrier_rrc.h"
#include "v34_rx_2800_low_carrier_rrc.h"
#include "v34_rx_2800_high_carrier_rrc.h"
#include "v34_rx_3000_low_carrier_rrc.h"
#include "v34_rx_3000_high_carrier_rrc.h"
#include "v34_rx_3200_low_carrier_rrc.h"
#include "v34_rx_3200_high_carrier_rrc.h"
#include "v34_rx_3429_rrc.h"

#include "v34_local.h"
#include "v34_tables.h"
#include "v34_superconstellation_map.h"
#include "v34_convolutional_coders.h"
#include "v34_shell_map.h"
#include "v34_probe_signals.h"

#if !defined(M_PI)
/* C99 systems may not define M_PI */
#define M_PI 3.14159265358979323846264338327
#endif

#if defined(SPANDSP_USE_FIXED_POINT)
#define FP_FACTOR                       4096
#define FP_SHIFT_FACTOR                 12
#endif

#define FP_Q9_7_TO_F(x)                 ((float) x/128.0f)

#define CARRIER_NOMINAL_FREQ            1800.0f
#define EQUALIZER_DELTA                 0.21f
#define EQUALIZER_SLOW_ADAPT_RATIO      0.1f

#define V34_TRAINING_SEG_1              0
#define V34_TRAINING_SEG_4              0
#define V34_TRAINING_END                0
#define V34_TRAINING_SHUTDOWN_END       0
#define MP_LOCK_SCORE_MIN               15
#define MP_PREAMBLE_SCORE_MIN           18
#define MP_HINT_LOCK_SCORE_MIN          14
#define MP_PREAMBLE_WAIT_BITS           800
#define MP_PRELOCK_PREAMBLE_WAIT_BITS   160
#define PHASE4_MP_TIMEOUT_BAUDS         20000
/* V.90 §9.4.1 lets the digital modem wait 15 s plus round-trip delays for
   B1 after INFO1a.  Before CPt is accepted, silence is therefore not enough
   evidence that the analogue modem has retrained: it may still be between
   repeated CPt attempts.  Keep the legacy silence heuristic out of that
   acquisition window; the independent Tone A detector below still handles a
   genuine §9.5.2.1 retrain immediately. */
#define PHASE4_CP_ACQUISITION_WAIT_SECONDS 15
#define MP_TRN_PRELOCK_SCORE_MIN        70
#define PHASE4_TRN_SCORE_START_BAUD     145
#define PHASE4_TRN_LOCK_MIN_BITS        64
#define PHASE4_TRN_READY_MIN_SCORE      65
#define PHASE4_TRN_READY_MIN_BAUD       4800    /* ~1.5s at 3200 baud; per V.34 §11.4 train on TRN ~2000ms before scanning MP */
#define PHASE4_TRN_READY_MAX_BAUD       9600    /* ~3s at 3200 baud; V.34 TRN max is 2s */
#define PHASE4_TRN_RECENT_WINDOW_BAUDS  256
#define PHASE4_TRN_FREEZE_SCORE         80
#define MP_HYPOTHESIS_COUNT             24
#define MP_EARLY_START_ERR_MAX          2
#define MP_EARLY_START_ERR_FRAME_LIMIT  85
#define MP1_START_ERR_ACCEPT_MAX        3
#define MP_BOUNDARY_BRUTEFORCE_MAX_CHANGES 4
#define MP_HINT_STRICT_REJECTS          2
#define MP_HINT_MAX_NOLOCKS             3
/* Minimum equalizer-output |y|^2 required before attempting a new MP
   preamble lock (squared magnitude, so no sqrt needed at the call site).
   Prevents locking onto near-zero-energy/silent samples: real in-lock MP
   symbols measured live against d-modem/slmodemd sit around |y|~0.03-2.9
   (median ~1.1), while true silence reads as an exact 0.0 (bit-identical,
   not just small). 0.01^2 sits two orders of magnitude below the weakest
   observed real symbol while still comfortably clearing float noise on
   true silence. NOTE: this and MP_LOCK_SETTLE_BAUDS below do not fix the
   live MP CRC failure against d-modem/slmodemd -- that failure's actual
   root cause turned out to be d-modem discarding its Phase 3/4 state and
   restarting the whole handshake (Phase 1/2 tones) to retry V.90 after
   its own constellation designer rejects our CPt (see "Phase 4 MP frame
   CRC failure" in rig/README.md), so the "signal" these gates wait for
   is d-modem's handshake-retrain tone, not a real MP0 frame. Kept anyway
   as correct, generally-useful defensive behavior (don't lock onto
   silence or a still-settling equalizer), not because it resolves this
   bug. */
#define MP_LOCK_MIN_SIGNAL_MAG2         0.0001f
/* Consecutive above-threshold bauds required before a preamble lock is
   attempted, once real signal has appeared. Live measurement showed the
   CMA equalizer is still actively re-adapting its gain well past 100
   bauds after the initial silence->signal transition (coefmag kept
   climbing through baud 5537-5600 with no clear plateau); 200 bauds
   still let a lock land mid-readaptation. 400 (~167ms at 2400 baud) is
   a conservative middle ground -- see the note on MP_LOCK_MIN_SIGNAL_MAG2
   above for why this does not actually fix the live CRC failure. */
#define MP_LOCK_SETTLE_BAUDS            400
#define PHASE3_PP_TRAIN_BAUDS           232
#define PHASE3_TRN_REFINE_BAUDS         256
#define PHASE3_PP_ACQUIRE_MIN_BAUDS     48
#define PHASE3_PP_ACQUIRE_HOLD_BAUDS    12
#define PHASE3_PP_ACQUIRE_SCORE_MIN     650
#define PHASE3_PP_ACQUIRE_DECAY         0.98f
#define V34_AGC_POWER_MIN               100000
#define V34_AGC_SCALING_MIN             0.00001f
#define V34_AGC_SCALING_MAX             0.01f
#define PHASE3_PP_MAG_SANITY_MAX        20.0f
#define PHASE3_J_PROGRESS_LOG_INTERVAL  32
#define PHASE4_J_PROGRESS_LOG_INTERVAL  32
#define V34_DEBUG_IQ_LOG                0
#define V34_DEBUG_INFO_RX_DIAG          1
#define V34_DEBUG_MP_DIBIT_DIST         0
#define PHASE4_MP_NOLOCK_LOG_INTERVAL   800
#define PHASE4_MP_BAUD_LOG_INTERVAL     400
#define PHASE4_MP_DIBIT_LOG_INTERVAL    1600
#define PHASE4_MP_REJECT_DETAIL_LOG_INTERVAL 3200
#define PHASE3_S_BAUD_LOG_INTERVAL      1000
#define PHASE3_S_ALTERNATING_MIN        24
#define PHASE3_S_STABLE_WINDOWS         64
/* Sustained-rotation S detection (see private/v34.h): a +/-90 degrees/symbol
   rotation shows as one dominant differential dibit filling the 32-baud window.
   DOMINANT_MIN sits well above scrambled Ja (~11/32) and below a real S
   (~30/32); DOMINANT_STABLE (bauds held) sits above Ja's longest run (~10)
   and below the 128T (~128 baud) S signal, so it is Ja-safe with margin. */
#define PHASE3_S_DOMINANT_MIN           24
#define PHASE3_S_DOMINANT_STABLE        48
#define PHASE3_PP_ACQUIRE_LOG_INTERVAL  256
#define PHASE3_PP_BAUD_LOG_INTERVAL     192

static int v90_phase3_j_lookahead_bits(void)
{
    static int initialized = 0;
    static int lookahead_bits = 0;

    if (!initialized)
    {
        const char *value;
        char *end;
        long parsed;

        value = getenv("ME_V90_J_LOOKAHEAD_BITS");
        if (value  &&  value[0] != '\0')
        {
            end = NULL;
            parsed = strtol(value, &end, 10);
            if (end != value  &&  end  &&  *end == '\0'
                && parsed >= 256  &&  parsed <= 20000)
            {
                lookahead_bits = (int) parsed;
            }
            /*endif*/
        }
        /*endif*/
        initialized = 1;
    }
    /*endif*/
    return lookahead_bits;
}

enum
{
    TRAINING_TX_STAGE_NORMAL_OPERATION_V34 = 0,
    TRAINING_TX_STAGE_NORMAL_OPERATION_CC = 1,
    TRAINING_TX_STAGE_PARKED
};

static const char *v34_rx_stage_to_str(int stage)
{
    switch (stage)
    {
    case V34_RX_STAGE_INFO0: return "INFO0";
    case V34_RX_STAGE_INFOH: return "INFOH";
    case V34_RX_STAGE_INFO1C: return "INFO1C";
    case V34_RX_STAGE_INFO1A: return "INFO1A";
    case V34_RX_STAGE_TONE_A: return "TONE_A";
    case V34_RX_STAGE_TONE_B: return "TONE_B";
    case V34_RX_STAGE_L1_L2: return "L1_L2";
    case V34_RX_STAGE_CC: return "CC";
    case V34_RX_STAGE_PRIMARY_CHANNEL: return "PRIMARY_CHANNEL";
    case V34_RX_STAGE_PHASE3_WAIT_S: return "PHASE3_WAIT_S";
    case V34_RX_STAGE_PHASE3_TRAINING: return "PHASE3_TRAINING";
    case V34_RX_STAGE_PHASE3_DONE: return "PHASE3_DONE";
    case V34_RX_STAGE_PHASE4_S: return "PHASE4_S";
    case V34_RX_STAGE_PHASE4_S_BAR: return "PHASE4_S_BAR";
    case V34_RX_STAGE_PHASE4_TRN: return "PHASE4_TRN";
    case V34_RX_STAGE_PHASE4_MP: return "PHASE4_MP";
    case V34_RX_STAGE_DATA: return "DATA";
    case V34_RX_STAGE_V90_CP: return "V90_CP";
    default: return "UNKNOWN";
    }
}

static bool v34_rx_stage_is_phase4_frame(int stage)
{
    return stage == V34_RX_STAGE_PHASE4_MP
        || stage == V34_RX_STAGE_V90_CP;
}

static bool v34_rx_stage_is_primary_training(int stage)
{
    return (stage >= V34_RX_STAGE_PHASE3_WAIT_S
            && stage <= V34_RX_STAGE_PHASE4_MP)
        || stage == V34_RX_STAGE_V90_CP;
}

static const char *v34_event_to_str(int event)
{
    switch (event)
    {
    case V34_EVENT_NONE: return "NONE";
    case V34_EVENT_TONE_SEEN: return "TONE_SEEN";
    case V34_EVENT_REVERSAL_1: return "REVERSAL_1";
    case V34_EVENT_REVERSAL_2: return "REVERSAL_2";
    case V34_EVENT_REVERSAL_3: return "REVERSAL_3";
    case V34_EVENT_INFO0_OK: return "INFO0_OK";
    case V34_EVENT_INFO0_BAD: return "INFO0_BAD";
    case V34_EVENT_INFO1_OK: return "INFO1_OK";
    case V34_EVENT_INFO1_BAD: return "INFO1_BAD";
    case V34_EVENT_INFOH_OK: return "INFOH_OK";
    case V34_EVENT_INFOH_BAD: return "INFOH_BAD";
    case V34_EVENT_L2_SEEN: return "L2_SEEN";
    case V34_EVENT_S: return "S";
    case V34_EVENT_J: return "J";
    case V34_EVENT_J_DASHED: return "J_DASHED";
    case V34_EVENT_PHASE4_TRN_READY: return "PHASE4_TRN_READY";
    default: return "UNKNOWN";
    }
}

static const char *v34_demodulator_to_str(int mod)
{
    switch (mod)
    {
    case V34_MODULATION_V34: return "V34";
    case V34_MODULATION_CC: return "CC";
    case V34_MODULATION_TONES: return "TONES";
    case V34_MODULATION_L1_L2: return "L1_L2";
    case V34_MODULATION_SILENCE: return "SILENCE";
    default: return "UNKNOWN";
    }
}

static void v34_rx_log_state_change(v34_rx_state_t *s)
{
    if (s->last_logged_stage != s->stage
        || s->last_logged_demodulator != s->current_demodulator)
    {
        span_log(s->logging, SPAN_LOG_FLOW,
                 "Rx - stage=%s (%d) demod=%s (%d)\n",
                 v34_rx_stage_to_str(s->stage), s->stage,
                 v34_demodulator_to_str(s->current_demodulator), s->current_demodulator);
        s->last_logged_stage = s->stage;
        s->last_logged_demodulator = s->current_demodulator;
    }
    if (s->last_logged_event != s->received_event)
    {
        span_log(s->logging, SPAN_LOG_FLOW,
                 "Rx - event=%s (%d)\n",
                 v34_event_to_str(s->received_event), s->received_event);
        s->last_logged_event = s->received_event;
    }
}

enum
{
    V34_MP_DIAG_STATE_NONE = -1,
    V34_MP_DIAG_STATE_DET_SYNC = 0,
    V34_MP_DIAG_STATE_DET_INFO = 1,
    V34_MP_DIAG_STATE_COMPLETE = 2
};

static const char *v34_mp_diag_state_to_str(int state)
{
    switch (state)
    {
    case V34_MP_DIAG_STATE_DET_SYNC: return "DET_SYNC";
    case V34_MP_DIAG_STATE_DET_INFO: return "DET_INFO";
    case V34_MP_DIAG_STATE_COMPLETE: return "COMPLETE";
    default: return "NONE";
    }
}

static void v34_rx_log_mp_diag_state(v34_rx_state_t *s, int state, const char *reason)
{
    if (s->last_logged_mp_diag_state == state)
        return;
    /*endif*/
    if (reason && reason[0])
    {
        span_log(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4 MP microstate=%s (%s)\n",
                 v34_mp_diag_state_to_str(state), reason);
    }
    else
    {
        span_log(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4 MP microstate=%s\n",
                 v34_mp_diag_state_to_str(state));
    }
    /*endif*/
    s->last_logged_mp_diag_state = state;
}

static const v34_rx_shaper_t *v34_rx_shapers_re[6][2] =
{
    {&rx_pulseshaper_2400_low_carrier_re, &rx_pulseshaper_2400_high_carrier_re},
    {&rx_pulseshaper_2743_low_carrier_re, &rx_pulseshaper_2743_high_carrier_re},
    {&rx_pulseshaper_2800_low_carrier_re, &rx_pulseshaper_2800_high_carrier_re},
    {&rx_pulseshaper_3000_low_carrier_re, &rx_pulseshaper_3000_high_carrier_re},
    {&rx_pulseshaper_3200_low_carrier_re, &rx_pulseshaper_3200_high_carrier_re},
    {&rx_pulseshaper_3429_re, &rx_pulseshaper_3429_re}
};

static const v34_rx_shaper_t *v34_rx_shapers_im[6][2] =
{
    {&rx_pulseshaper_2400_low_carrier_im, &rx_pulseshaper_2400_high_carrier_im},
    {&rx_pulseshaper_2743_low_carrier_im, &rx_pulseshaper_2743_high_carrier_im},
    {&rx_pulseshaper_2800_low_carrier_im, &rx_pulseshaper_2800_high_carrier_im},
    {&rx_pulseshaper_3000_low_carrier_im, &rx_pulseshaper_3000_high_carrier_im},
    {&rx_pulseshaper_3200_low_carrier_im, &rx_pulseshaper_3200_high_carrier_im},
    {&rx_pulseshaper_3429_im, &rx_pulseshaper_3429_im}
};

#if defined(SPANDSP_USE_FIXED_POINT)
#define complex_sig_set(re,im) complex_seti16(re,im)
#define complex_sig_t complexi16_t
#else
#define complex_sig_set(re,im) complex_setf(re,im)
#define complex_sig_t complexf_t
#endif

#if defined(SPANDSP_USE_FIXED_POINT)
#define TRAINING_SCALE(x)       ((int16_t) (32767.0f*x + ((x >= 0.0)  ?  0.5  :  -0.5)))
#else
#define TRAINING_SCALE(x)       (x)
#endif

static const complex_sig_t zero = {TRAINING_SCALE(0.0f), TRAINING_SCALE(0.0f)};

static void process_cc_half_baud(v34_rx_state_t *s, const complexf_t *sample);
static void process_primary_half_baud(v34_rx_state_t *s, const complexf_t *sample);
static void l1_l2_analysis_init(v34_rx_state_t *s);
static void equalizer_reset(v34_rx_state_t *s);
static complexf_t equalizer_get(v34_rx_state_t *s);
static void tune_equalizer(v34_rx_state_t *s, const complexf_t *z, const complexf_t *target);
static void create_godard_coeffs(ted_t *coeffs, float carrier, float baud_rate, float alpha);
SPAN_DECLARE(void) v34_put_mapping_frame(v34_rx_state_t *s, int16_t bits[16]);

static int descramble(v34_rx_state_t *s, int in_bit)
{
    int out_bit;

    /* One of the scrambler taps is a variable, so it can be adjusted for caller or answerer operation. */
    out_bit = (in_bit ^ (s->scramble_reg >> s->scrambler_tap) ^ (s->scramble_reg >> (23 - 1))) & 1;
    s->scramble_reg = (s->scramble_reg << 1) | in_bit;
    return out_bit;
}
/*- End of function --------------------------------------------------------*/

/* Whether the receiver keeps adapting through Phase 3 once TRN is locked.
   Default on; ME_V34_TRACK_PHASE3=0 restores the old frozen behaviour for A/B. */
static int phase3_tracking_enabled(void)
{
    static int initialized = 0;
    static int enabled = 1;

    if (!initialized)
    {
        const char *value = getenv("ME_V34_TRACK_PHASE3");

        if (value  &&  value[0] != '\0'  &&  strcmp(value, "0") == 0)
            enabled = 0;
        /*endif*/
        initialized = 1;
    }
    /*endif*/
    return enabled;
}
/*- End of function --------------------------------------------------------*/

static int descramble_reg(uint32_t *reg, int scrambler_tap, int in_bit)
{
    int out_bit;

    out_bit = (in_bit ^ (*reg >> scrambler_tap) ^ (*reg >> (23 - 1))) & 1;
    *reg = (*reg << 1) | in_bit;
    return out_bit;
}
/*- End of function --------------------------------------------------------*/

static int map_phase4_raw_bits(int dibit, int hypothesis)
{
    static const uint8_t map_table[MP_HYPOTHESIS_COUNT][4] =
    {
        /* Legacy hyp8 ordering first for Phase 3 lock-hint compatibility */
        {0, 1, 2, 3}, /*  0: identity */
        {1, 0, 3, 2}, /*  1: xor 1 */
        {2, 3, 0, 1}, /*  2: xor 2 */
        {3, 2, 1, 0}, /*  3: xor 3 */
        {0, 2, 1, 3}, /*  4: swap */
        {2, 0, 3, 1}, /*  5: swap + xor 1 */
        {1, 3, 0, 2}, /*  6: swap + xor 2 */
        {3, 1, 2, 0}, /*  7: swap + xor 3 */

        /* Additional invertible affine transforms on dibits */
        {0, 3, 2, 1}, /*  8 */
        {1, 2, 3, 0}, /*  9 */
        {2, 1, 0, 3}, /* 10 */
        {3, 0, 1, 2}, /* 11 */
        {0, 1, 3, 2}, /* 12 */
        {1, 0, 2, 3}, /* 13 */
        {2, 3, 1, 0}, /* 14 */
        {3, 2, 0, 1}, /* 15 */
        {0, 2, 3, 1}, /* 16 */
        {1, 3, 2, 0}, /* 17 */
        {2, 0, 1, 3}, /* 18 */
        {3, 1, 0, 2}, /* 19 */
        {0, 3, 1, 2}, /* 20 */
        {1, 2, 0, 3}, /* 21 */
        {2, 1, 3, 0}, /* 22 */
        {3, 0, 2, 1}  /* 23 */
    };

    int hidx;

    hidx = hypothesis % MP_HYPOTHESIS_COUNT;
    if (hidx < 0)
        hidx += MP_HYPOTHESIS_COUNT;
    /*endif*/
    return map_table[hidx][dibit & 0x3];
}
/*- End of function --------------------------------------------------------*/

static void bits32_to_str(uint32_t v, char out[33])
{
    int j;

    for (j = 31;  j >= 0;  j--)
        out[31 - j] = ((v >> j) & 1) ? '1' : '0';
    /*endfor*/
    out[32] = '\0';
}
/*- End of function --------------------------------------------------------*/

static void frame_bits_to_str(const uint8_t bits[], int start, int count, char *out)
{
    int i;

    for (i = 0;  i < count;  i++)
        out[i] = bits[start + i] ? '1' : '0';
    /*endfor*/
    out[count] = '\0';
}
/*- End of function --------------------------------------------------------*/

static void bits16_to_str(uint16_t v, char out[17])
{
    int j;

    /* Left-most output bit is first-in-time.
       For our shift-left stream register, that is bit 15 (oldest). */
    for (j = 15;  j >= 0;  j--)
        out[15 - j] = ((v >> j) & 1) ? '1' : '0';
    /*endfor*/
    out[16] = '\0';
}
/*- End of function --------------------------------------------------------*/

static uint16_t j_ordered16(uint16_t rx_recent16, int total_bits, int phase)
{
    uint16_t ordered;
    int t;

    ordered = 0;
    for (t = 0;  t < 16;  t++)
    {
        int seq_idx;
        int b;

        /* t runs from oldest -> newest bit in time. */
        seq_idx = (total_bits - 16 + t + phase) & 0xF;
        b = (rx_recent16 >> (15 - t)) & 1;
        ordered |= (uint16_t) (b << seq_idx);
    }
    /*endfor*/
    return ordered;
}
/*- End of function --------------------------------------------------------*/

static int mp_preamble_score(uint32_t bitstream)
{
    int k;
    int score;

    /* Score fixed MP preamble bits currently in bitstream[18:1]:
       17x'1' followed by start '0'. bitstream[0] is the type bit. */
    score = 0;
    for (k = 1;  k <= 18;  k++)
    {
        int got;
        int want;

        got = (bitstream >> k) & 1;
        want = (k == 1)  ?  0  :  1;
        if (got == want)
            score++;
        /*endif*/
    }
    /*endfor*/
    return score;
}
/*- End of function --------------------------------------------------------*/

static bool mp_preamble_has_start_zero(uint32_t bitstream)
{
    /* Preamble layout in bitstream[18:1]:
       17x'1' followed by start '0' at bit 1. */
    return ((bitstream >> 1) & 1) == 0;
}
/*- End of function --------------------------------------------------------*/

static bool mp_preamble_has_sync_ones(uint32_t bitstream)
{
    int k;
    int ones;

    /* Preamble layout in bitstream[18:1]:
       17x'1' in bits [18:2], start '0' in bit [1], type in bit [0]. */
    ones = 0;
    for (k = 2;  k <= 18;  k++)
    {
        ones += ((bitstream >> k) & 1);
    }
    /*endfor*/
    /* Allow up to two sync-bit errors in the preamble gate.
       CRC/fill checks still provide the final validity filter. */
    return (ones >= 15);
}
/*- End of function --------------------------------------------------------*/

static void mp_seed_frame_prefix(uint8_t bits[], uint32_t preamble_stream)
{
    int idx;

    /* frame_idx 0..16 are sync bits, 17 is start, 18 is type. */
    for (idx = 0;  idx <= 18;  idx++)
        bits[idx] = (preamble_stream >> (18 - idx)) & 1;
    /*endfor*/
}
/*- End of function --------------------------------------------------------*/

static void mp_reset_hypothesis_search(v34_rx_state_t *s)
{
    s->mp_hypothesis = -1;
    s->mp_count = -1;
    /* Do NOT zero mp_hyp_scramble or mp_hyp_bitstream here — the GPC
       descrambler shift registers must track every input baud continuously.
       Zeroing them destroys synchronization and makes all subsequent
       descrambled data garbage even if preamble re-locks.  The registers
       are properly initialized at MP entry from the TRN scramble state
       and updated every baud in the hypothesis scan loop. */
}
/*- End of function --------------------------------------------------------*/

static void phase3_pp_reset(v34_rx_state_t *s)
{
    memset(s->phase3_pp_lag8, 0, sizeof(s->phase3_pp_lag8));
    s->phase3_pp_obs = 0;
    s->phase3_pp_match = 0;
    memset(s->phase3_pp_error, 0, sizeof(s->phase3_pp_error));
    memset(s->phase3_pp_corr, 0, sizeof(s->phase3_pp_corr));
    s->phase3_pp_corr_energy = 0.0f;
    s->phase3_pp_corr_weight = 0.0f;
    s->phase3_pp_rotation.re = 1.0f;
    s->phase3_pp_rotation.im = 0.0f;
    s->phase3_pp_phase = -1;
    s->phase3_pp_phase_score = -1;
    s->phase3_pp_acquire_hits = 0;
    s->phase3_pp_started = 0;
}
/*- End of function --------------------------------------------------------*/

static void phase4_trn_hyp_reset(v34_rx_state_t *s)
{
    memset(s->phase4_trn_scramble_tap, 0, sizeof(s->phase4_trn_scramble_tap));
    memset(s->phase4_trn_one_count_tap, 0, sizeof(s->phase4_trn_one_count_tap));
    memset(s->phase4_trn_scramble, 0, sizeof(s->phase4_trn_scramble));
    memset(s->phase4_trn_prev_z, 0, sizeof(s->phase4_trn_prev_z));
    memset(s->phase4_trn_prev_valid, 0, sizeof(s->phase4_trn_prev_valid));
    memset(s->phase4_trn_one_count, 0, sizeof(s->phase4_trn_one_count));
    s->phase4_trn_lock_hyp = -1;
    s->phase4_trn_lock_score = -1;
    s->phase4_trn_lock_tap = -1;
    s->phase4_trn_lock_order = -1;
    s->phase4_trn_lock_domain = -1;
    s->phase4_trn_current_hyp = -1;
    s->phase4_trn_current_score = -1;
    s->phase4_trn_current_tap = -1;
    s->phase4_trn_current_order = -1;
    s->phase4_trn_current_domain = -1;
    s->phase4_trn_recent_scramble = 0;
    s->phase4_trn_recent_window_bits = 0;
    s->phase4_trn_recent_window_ones = 0;
    s->phase4_trn_recent_window_fill = 0;
    s->phase4_trn_recent_score = -1;
    memset(s->phase4_trn_recent_symbol_ones, 0, sizeof(s->phase4_trn_recent_symbol_ones));
    s->phase4_trn_recent_active = 0;
}
/*- End of function --------------------------------------------------------*/

static void phase4_trn_recent_reset(v34_rx_state_t *s)
{
    s->phase4_trn_recent_scramble = 0;
    s->phase4_trn_recent_window_bits = 0;
    s->phase4_trn_recent_window_ones = 0;
    s->phase4_trn_recent_window_fill = 0;
    s->phase4_trn_recent_score = -1;
    memset(s->phase4_trn_recent_symbol_ones, 0, sizeof(s->phase4_trn_recent_symbol_ones));
    s->phase4_trn_recent_active = 0;
}
/*- End of function --------------------------------------------------------*/

static void phase4_trn_recent_seed(v34_rx_state_t *s)
{
    if (s->phase4_trn_lock_hyp < 0
        || s->phase4_trn_lock_domain < 0
        || s->phase4_trn_lock_tap < 0
        || s->phase4_trn_lock_order < 0)
    {
        phase4_trn_recent_reset(s);
        return;
    }
    /*endif*/
    s->phase4_trn_recent_scramble =
        s->phase4_trn_scramble_tap[s->phase4_trn_lock_domain][s->phase4_trn_lock_tap][s->phase4_trn_lock_order][s->phase4_trn_lock_hyp];
    s->phase4_trn_recent_window_bits = 0;
    s->phase4_trn_recent_window_ones = 0;
    s->phase4_trn_recent_window_fill = 0;
    s->phase4_trn_recent_score = -1;
    memset(s->phase4_trn_recent_symbol_ones, 0, sizeof(s->phase4_trn_recent_symbol_ones));
    s->phase4_trn_recent_active = 1;
}
/*- End of function --------------------------------------------------------*/

static void phase4_trn_recent_update(v34_rx_state_t *s, int raw_sym)
{
    int d0;
    int d1;
    int ones;
    int pos;
    int tap;

    if (!s->phase4_trn_recent_active)
        return;
    /*endif*/
    if (s->phase4_trn_lock_hyp < 0)
        return;
    /*endif*/
    if (s->phase4_trn_after_j < PHASE4_TRN_SCORE_START_BAUD)
        return;
    /*endif*/

    tap = (s->phase4_trn_lock_tap == 0) ? 17 : 4;
    if (s->phase4_trn_lock_order == 0)
    {
        d0 = descramble_reg(&s->phase4_trn_recent_scramble, tap, raw_sym & 1);
        d1 = descramble_reg(&s->phase4_trn_recent_scramble, tap, (raw_sym >> 1) & 1);
    }
    else
    {
        d1 = descramble_reg(&s->phase4_trn_recent_scramble, tap, (raw_sym >> 1) & 1);
        d0 = descramble_reg(&s->phase4_trn_recent_scramble, tap, raw_sym & 1);
    }
    /*endif*/
    ones = d0 + d1;
    pos = (s->phase4_trn_after_j - PHASE4_TRN_SCORE_START_BAUD) & (PHASE4_TRN_RECENT_WINDOW_BAUDS - 1);

    if (s->phase4_trn_recent_window_fill >= PHASE4_TRN_RECENT_WINDOW_BAUDS)
        s->phase4_trn_recent_window_ones -= s->phase4_trn_recent_symbol_ones[pos];
    else
        s->phase4_trn_recent_window_fill++;
    /*endif*/
    s->phase4_trn_recent_symbol_ones[pos] = (uint8_t) ones;
    s->phase4_trn_recent_window_ones += (uint16_t) ones;
    s->phase4_trn_recent_window_bits = 2*s->phase4_trn_recent_window_fill;
    if (s->phase4_trn_recent_window_bits > 0)
    {
        s->phase4_trn_recent_score =
            (100*s->phase4_trn_recent_window_ones + (s->phase4_trn_recent_window_bits/2))/s->phase4_trn_recent_window_bits;
    }
    /*endif*/
}
/*- End of function --------------------------------------------------------*/

static int phase4_trn_should_freeze_tracking(const v34_rx_state_t *s)
{
    /* Also freeze carrier tracking during MP — the carrier phase was locked
       during TRN and CMA-based QPSK slicer decisions during MP are too noisy
       to drive carrier tracking without corrupting the locked phase. */
    if (v34_rx_stage_is_phase4_frame(s->stage))
        return 1;
    return (s->stage == V34_RX_STAGE_PHASE4_TRN
            && s->phase4_j_seen
            && s->phase4_trn_lock_hyp >= 0
            && s->phase4_trn_lock_score >= PHASE4_TRN_FREEZE_SCORE
            && s->phase4_trn_current_hyp == s->phase4_trn_lock_hyp
            && s->phase4_trn_current_tap == s->phase4_trn_lock_tap
            && s->phase4_trn_current_order == s->phase4_trn_lock_order
            && s->phase4_trn_current_domain == s->phase4_trn_lock_domain
            && s->phase4_trn_current_score >= PHASE4_TRN_FREEZE_SCORE);
}
/*- End of function --------------------------------------------------------*/

static void phase3_trn_hyp_reset(v34_rx_state_t *s)
{
    memset(s->phase3_trn_scramble, 0, sizeof(s->phase3_trn_scramble));
    memset(s->phase3_trn_one_count, 0, sizeof(s->phase3_trn_one_count));
    s->phase3_trn_bits = 0;
    s->phase3_trn_lock_hyp = -1;
    s->phase3_trn_lock_score = -1;
    s->phase3_trn_rescore_bits = 0;
    s->phase3_tracking_armed = false;
}
/*- End of function --------------------------------------------------------*/

static void phase3_ja_capture_append(v34_rx_state_t *s, int bit0, int bit1)
{
    if (!s)
        return;
    /*endif*/
    if (s->phase3_ja_capture_len + 2 > (int) sizeof(s->phase3_ja_capture))
        return;
    /*endif*/
    s->phase3_ja_capture[s->phase3_ja_capture_len++] = (uint8_t) (bit0 & 1);
    s->phase3_ja_capture[s->phase3_ja_capture_len++] = (uint8_t) (bit1 & 1);
    s->phase3_ja_bits += 2;
    if (s->phase3_ja_bits == 2 || (s->phase3_ja_bits % 256) == 0)
    {
        span_log(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 3 Ja capture: emitted %d bits using hyp=%d\n",
                 s->phase3_ja_bits, s->phase3_ja_hyp);
    }
    /*endif*/
}
/*- End of function --------------------------------------------------------*/

static int mp_alternate_scrambler_tap(int tap)
{
    /* V.34 uses the two complementary scrambler taps (x^-5 and x^-18),
       represented here as zero-based indices 4 and 17. */
    return (tap == 17) ? 4 : 17;
}
/*- End of function --------------------------------------------------------*/

static int phase4_trn_tap_value(int tap_idx)
{
    static const int taps[2] = {17, 4};

    if (tap_idx < 0 || tap_idx > 1)
        return taps[0];
    /*endif*/
    return taps[tap_idx];
}
/*- End of function --------------------------------------------------------*/

static const char *phase4_trn_order_name(int order_idx)
{
    return (order_idx == 1) ? "b1,b0" : "b0,b1";
}
/*- End of function --------------------------------------------------------*/

static const char *phase4_trn_domain_name(int domain_idx)
{
    return (domain_idx == 1) ? "abs" : "diff";
}
/*- End of function --------------------------------------------------------*/

static void phase4_unpack_ordered_bits(int raw_bits, int order_idx, int *first_bit, int *second_bit)
{
    if (order_idx == 1)
    {
        *first_bit = (raw_bits >> 1) & 1;
        *second_bit = raw_bits & 1;
    }
    else
    {
        *first_bit = raw_bits & 1;
        *second_bit = (raw_bits >> 1) & 1;
    }
    /*endif*/
}
/*- End of function --------------------------------------------------------*/

static int mp_hist_min4(const int hist[4])
{
    int i;
    int min_v;

    min_v = hist[0];
    for (i = 1;  i < 4;  i++)
    {
        if (hist[i] < min_v)
            min_v = hist[i];
        /*endif*/
    }
    /*endfor*/
    return min_v;
}
/*- End of function --------------------------------------------------------*/

static int mp_hist_max4(const int hist[4])
{
    int i;
    int max_v;

    max_v = hist[0];
    for (i = 1;  i < 4;  i++)
    {
        if (hist[i] > max_v)
            max_v = hist[i];
        /*endif*/
    }
    /*endfor*/
    return max_v;
}
/*- End of function --------------------------------------------------------*/

static int mp_hist_top2_sum(const int hist[4])
{
    int i;
    int top1;
    int top2;

    top1 = -1;
    top2 = -1;
    for (i = 0;  i < 4;  i++)
    {
        int v;

        v = hist[i];
        if (v > top1)
        {
            top2 = top1;
            top1 = v;
        }
        else if (v > top2)
        {
            top2 = v;
        }
        /*endif*/
    }
    /*endfor*/
    return top1 + top2;
}
/*- End of function --------------------------------------------------------*/

static int mp_hist_bottom2_sum(const int hist[4])
{
    int i;
    int low1;
    int low2;

    low1 = 0x7FFFFFFF;
    low2 = 0x7FFFFFFF;
    for (i = 0;  i < 4;  i++)
    {
        int v;

        v = hist[i];
        if (v < low1)
        {
            low2 = low1;
            low1 = v;
        }
        else if (v < low2)
        {
            low2 = v;
        }
        /*endif*/
    }
    /*endfor*/
    return low1 + low2;
}
/*- End of function --------------------------------------------------------*/

static int mp_phase4_diff_hist_collapsed(const int diff_hist[4])
{
    int top2_sum;
    int bot2_sum;

    top2_sum = mp_hist_top2_sum(diff_hist);
    bot2_sum = mp_hist_bottom2_sum(diff_hist);
    return (top2_sum >= 360 && bot2_sum <= 8);
}
/*- End of function --------------------------------------------------------*/

static int mp_phase4_abs_hist_healthy(const int abs_hist[4])
{
    int min_v;
    int max_v;

    min_v = mp_hist_min4(abs_hist);
    max_v = mp_hist_max4(abs_hist);
    return (min_v >= 60 && max_v <= 140);
}
/*- End of function --------------------------------------------------------*/

static void mp_phase4_update_auto_domain(v34_rx_state_t *s, const int diff_hist[4], const int abs_hist[4])
{
    int diff_collapsed;
    int abs_healthy;

    /* Once a Phase 4 hypothesis has locked, its bit stream must remain in the
       same dibit domain for the complete CP/MP frame.  Switching from the
       absolute fallback back to differential dibits while the frame is being
       collected preserves the preamble but corrupts the DFI/DRN/CRC fields.
       Leave the selected domain latched until the hypothesis is explicitly
       reset after a rejected frame. */
    if (s->mp_hypothesis >= 0)
        return;

    diff_collapsed = mp_phase4_diff_hist_collapsed(diff_hist);
    abs_healthy = mp_phase4_abs_hist_healthy(abs_hist);
    if (diff_collapsed && abs_healthy)
    {
        s->mp_phase4_diff_collapse_streak++;
        s->mp_phase4_diff_recover_streak = 0;
    }
    else
    {
        s->mp_phase4_diff_collapse_streak = 0;
        if (!diff_collapsed)
            s->mp_phase4_diff_recover_streak++;
        else
            s->mp_phase4_diff_recover_streak = 0;
        /*endif*/
    }
    /*endif*/

    if (!s->mp_phase4_force_abs_active
        && s->mp_phase4_diff_collapse_streak >= 1)
    {
        s->mp_phase4_force_abs_active = 1;
        span_log(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4: auto-domain fallback enabled (diff dibits collapsed); forcing abs decode\n");
    }
    else if (s->mp_phase4_force_abs_active
             && s->mp_phase4_diff_recover_streak >= 2)
    {
        s->mp_phase4_force_abs_active = 0;
        span_log(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4: auto-domain fallback cleared (diff dibits recovered); restoring configured domain=%s\n",
                 phase4_trn_domain_name(s->mp_phase4_domain));
    }
    /*endif*/
}
/*- End of function --------------------------------------------------------*/

static void phase4_j_detector_reset(v34_rx_state_t *s)
{
    s->phase4_j_bits = 0;
    memset(s->phase4_j_scramble_tap, 0, sizeof(s->phase4_j_scramble_tap));
    memset(s->phase4_j_stream_tap, 0, sizeof(s->phase4_j_stream_tap));
    memset(s->phase4_j_prev_z_tap, 0, sizeof(s->phase4_j_prev_z_tap));
    memset(s->phase4_j_prev_valid_tap, 0, sizeof(s->phase4_j_prev_valid_tap));
    memset(s->phase4_j_win_tap, 0, sizeof(s->phase4_j_win_tap));
}
/*- End of function --------------------------------------------------------*/

static void mp_phase4_apply_retry_mode(v34_rx_state_t *s, int retry_mode)
{
    int use_alt_domain;
    int use_alt_order;
    int use_alt_tap;
    int tap;
    int order;
    int domain;

    use_alt_domain = ((retry_mode & 0x4) != 0);
    use_alt_order = ((retry_mode & 0x1) != 0);
    use_alt_tap = ((retry_mode & 0x2) != 0);
    tap = s->mp_phase4_default_scrambler_tap;
    order = s->mp_phase4_default_bit_order;
    domain = s->mp_phase4_default_domain;
    if (use_alt_tap)
        tap = mp_alternate_scrambler_tap(tap);
    /*endif*/
    if (use_alt_order)
        order ^= 1;
    /*endif*/
    if (use_alt_domain)
        domain ^= 1;
    /*endif*/
    s->scrambler_tap = tap;
    s->mp_phase4_bit_order = order;
    s->mp_phase4_domain = domain;
    s->mp_phase4_alt_tap_active = use_alt_tap;
    s->mp_phase4_alt_order_active = use_alt_order;
    s->mp_phase4_alt_domain_active = use_alt_domain;
}
/*- End of function --------------------------------------------------------*/

static void mp_vote_reset(v34_rx_state_t *s);

static void mp_v90_cp_reset_at_carrier_gap(v34_rx_state_t *s)
{
    /* V.90 §8.5.2 initializes the analogue modem's scrambler and
       differential encoder to zero before the first CPt.  The forced V.90
       receiver can be armed while the preceding Phase-3 signal is still
       present, so hypotheses accumulated before the carrier gap do not
       belong to CPt.  Reset only the CP framing/descrambling search at that
       unambiguous boundary; retain the trained equalizer, carrier recovery
       and sample timing. */
    s->mp_phase4_retry_mode = 0;
    mp_phase4_apply_retry_mode(s, 0);
    s->mp_phase4_reject_streak = 0;
    s->mp_phase4_nolock_count = 0;
    s->mp_phase4_force_abs_active = 0;
    s->mp_phase4_diff_collapse_streak = 0;
    s->mp_phase4_diff_recover_streak = 0;
    s->phase4_da_active = 0;
    s->phase4_da_seeded = 0;
    s->phase4_da_expected_ang = 0;
    s->phase4_da_derot = 0;
    s->mp_frame_pos = 0;
    s->mp_frame_target = 0;
    s->mp_early_rejects = 0;
    s->bitstream = 0;
    s->bit_count = 0;
    memset(s->mp_hyp_scramble, 0, sizeof(s->mp_hyp_scramble));
    memset(s->mp_hyp_bitstream, 0, sizeof(s->mp_hyp_bitstream));
    mp_reset_hypothesis_search(s);
    mp_vote_reset(s);
}
/*- End of function --------------------------------------------------------*/

static int mp_phase4_has_pinned_trn_lock(const v34_rx_state_t *s)
{
    return (s->phase4_trn_lock_hyp >= 0
            && s->phase4_trn_lock_hyp < MP_HYPOTHESIS_COUNT
            && s->phase4_trn_lock_score >= PHASE4_TRN_READY_MIN_SCORE);
}
/*- End of function --------------------------------------------------------*/

static void mp_phase4_rotate_retry_mode(v34_rx_state_t *s, const char *reason)
{
    if (mp_phase4_has_pinned_trn_lock(s))
    {
        if (s->mp_phase4_reject_streak < 3)
        {
            s->mp_phase4_nolock_count++;
            /* After 3 no-lock rotations (~1200 bauds), broaden search beyond
               TRN hint hypothesis — the TRN-locked hyp may not match MP. */
            if (s->mp_phase4_nolock_count >= 3)
            {
                s->mp_phase4_reject_streak = 3;
                span_log(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 4: %s; TRN hint hyp=%d failed %d times, broadening MP search\n",
                         reason, s->phase4_trn_lock_hyp, s->mp_phase4_nolock_count);
            }
            else
            {
                mp_reset_hypothesis_search(s);
                span_log(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 4: %s; keeping TRN-locked MP settings (hyp=%d, dom=%s, tap=%d, ord=%s)\n",
                         reason, s->phase4_trn_lock_hyp,
                         phase4_trn_domain_name(s->mp_phase4_domain), s->scrambler_tap,
                         phase4_trn_order_name(s->mp_phase4_bit_order));
                return;
            }
        }
        /*endif*/
    }
    /*endif*/
    s->mp_phase4_retry_mode = (s->mp_phase4_retry_mode + 1) & 0x7;
    mp_phase4_apply_retry_mode(s, s->mp_phase4_retry_mode);
    mp_reset_hypothesis_search(s);
    if (s->mp_phase4_retry_mode == 0)
    {
        span_log(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4: %s; restoring MP descrambler defaults (dom=%s, tap=%d, ord=%s)\n",
                 reason, phase4_trn_domain_name(s->mp_phase4_domain), s->scrambler_tap,
                 phase4_trn_order_name(s->mp_phase4_bit_order));
    }
    else
    {
        span_log(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4: %s; switching MP descrambler retry mode=%d (dom=%s, tap=%d, ord=%s)\n",
                 reason, s->mp_phase4_retry_mode,
                 phase4_trn_domain_name(s->mp_phase4_domain), s->scrambler_tap,
                 phase4_trn_order_name(s->mp_phase4_bit_order));
    }
    /*endif*/
}
/*- End of function --------------------------------------------------------*/

static void mp_unlock_after_reject(v34_rx_state_t *s, bool count_tap_reject)
{
    const int tap_switch_rejects = 3;

    span_log(s->logging, SPAN_LOG_FLOW,
             "Rx - Phase 4: unlock MP hypothesis=%d after rejected frame\n",
             s->mp_hypothesis);
    s->mp_early_rejects = 0;
    mp_reset_hypothesis_search(s);
    if (mp_phase4_has_pinned_trn_lock(s))
    {
        if (count_tap_reject)
            s->mp_phase4_reject_streak++;
        /*endif*/
        if (s->mp_phase4_reject_streak >= tap_switch_rejects)
        {
            s->mp_phase4_retry_mode = (s->mp_phase4_retry_mode + 1) & 0x7;
            mp_phase4_apply_retry_mode(s, s->mp_phase4_retry_mode);
            span_log(s->logging, SPAN_LOG_FLOW,
                     "Rx - Phase 4: pinned TRN lock with %d rejects, switching MP retry mode=%d (dom=%s, tap=%d, ord=%s)\n",
                     s->mp_phase4_reject_streak, s->mp_phase4_retry_mode,
                     phase4_trn_domain_name(s->mp_phase4_domain), s->scrambler_tap,
                     phase4_trn_order_name(s->mp_phase4_bit_order));
            s->mp_phase4_reject_streak = 0;
        }
        /*endif*/
        s->mp_frame_pos = 0;
        s->mp_frame_target = 0;
        span_log(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4: keeping TRN-locked MP hypothesis/settings after reject (hyp=%d, streak=%d, dom=%s, tap=%d, ord=%s)\n",
                 s->phase4_trn_lock_hyp, s->mp_phase4_reject_streak,
                 phase4_trn_domain_name(s->mp_phase4_domain), s->scrambler_tap,
                 phase4_trn_order_name(s->mp_phase4_bit_order));
        return;
    }
    /*endif*/
    if (count_tap_reject
        && ++s->mp_phase4_reject_streak >= tap_switch_rejects)
    {
        s->mp_phase4_retry_mode = (s->mp_phase4_retry_mode + 1) & 0x7;
        mp_phase4_apply_retry_mode(s, s->mp_phase4_retry_mode);
        span_log(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4: after %d rejects, MP retry mode=%d (dom=%s, tap=%d, ord=%s)\n",
                 s->mp_phase4_reject_streak, s->mp_phase4_retry_mode,
                 phase4_trn_domain_name(s->mp_phase4_domain), s->scrambler_tap,
                 phase4_trn_order_name(s->mp_phase4_bit_order));
        s->mp_phase4_reject_streak = 0;
    }
    /*endif*/
    if (!count_tap_reject)
        s->mp_phase4_reject_streak = 0;
    /*endif*/
    s->mp_frame_pos = 0;
    s->mp_frame_target = 0;
}
/*- End of function --------------------------------------------------------*/

static void mp_vote_reset(v34_rx_state_t *s)
{
    memset(s->mp0_vote_counts, 0, sizeof(s->mp0_vote_counts));
    s->mp0_vote_frames = 0;
    s->mp0_vote_hyp = -1;
    memset(s->mp1_vote_counts, 0, sizeof(s->mp1_vote_counts));
    s->mp1_vote_frames = 0;
    s->mp1_vote_hyp = -1;
}
/*- End of function --------------------------------------------------------*/

static int phase3_j_pattern_bit(int pat_type, int bit_idx)
{
    /* LSB-first pattern bits, per V.34 Table 18/19 representation used by TX:
       J (4-point)  = 0x8990
       J (16-point) = 0x89B0
       J'           = 0x899F */
    static const uint16_t pats[3] =
    {
        0x8990,
        0x89B0,
        0x899F
    };

    return (pats[pat_type] >> (bit_idx & 15)) & 1;
}
/*- End of function --------------------------------------------------------*/

static uint16_t mp_crc_bits(const uint8_t bits[], int type)
{
    int i;
    int len;
    uint16_t crc;

    crc = 0xFFFF;
    len = (type == 1)  ?  170  :  68;
    for (i = 17;  i < len;  i += 17)
    {
        int j;

        /* Each 17-bit block is: start bit + 16 payload bits.
           CRC is over the 16 payload bits only (exclude start bit). */
        for (j = 0;  j < 16;  j++)
            crc = crc_itu16_bits(bits[i + 1 + j], 1, crc);
        /*endfor*/
    }
    /*endfor*/
    return crc;
}
/*- End of function --------------------------------------------------------*/

static bool mp_crc_ok(const uint8_t bits[], int type, uint16_t *rx_crc_out, uint16_t *residual_out)
{
    int i;
    int crc_start;
    uint16_t crc;
    uint16_t rx_crc;

    crc = mp_crc_bits(bits, type);
    crc_start = (type == 1)  ?  171  :  69;
    rx_crc = 0;
    for (i = 0;  i < 16;  i++)
    {
        rx_crc |= bits[crc_start + i] << i;
        crc = crc_itu16_bits(bits[crc_start + i], 1, crc);
    }
    /*endfor*/
    if (rx_crc_out)
        *rx_crc_out = rx_crc;
    /*endif*/
    if (residual_out)
        *residual_out = crc;
    /*endif*/
    return (crc == 0);
}
/*- End of function --------------------------------------------------------*/

static bool mp_fill_ok(const uint8_t bits[], int type)
{
    if (type == 1)
        return bits[187] == 0;
    return (bits[85] | bits[86] | bits[87]) == 0;
}
/*- End of function --------------------------------------------------------*/

static bool mp_start_bit_ok(int type, int bit_index, int bit_value)
{
    if (bit_index == 17 || bit_index == 34 || bit_index == 51 || bit_index == 68)
        return bit_value == 0;
    if (type == 1
        &&
        (bit_index == 85
         || bit_index == 102
         || bit_index == 119
         || bit_index == 136
         || bit_index == 153
         || bit_index == 170))
    {
        return bit_value == 0;
    }
    /*endif*/
    return true;
}
/*- End of function --------------------------------------------------------*/

static int mp_start_error_count(const uint8_t bits[], int type, int target)
{
    int i;
    int errs;

    errs = 0;
    for (i = 17;  i < target;  i++)
    {
        if (!mp_start_bit_ok(type, i, bits[i]))
            errs++;
        /*endif*/
    }
    /*endfor*/
    return errs;
}
/*- End of function --------------------------------------------------------*/

static bool mp_try_slip_recovery(uint8_t bits[188], int type, int target, int *slip_out)
{
    static const int slips[] = {-2, -1, 1, 2};
    uint8_t trial[188];
    int sidx;

    for (sidx = 0;  sidx < (int) (sizeof(slips)/sizeof(slips[0]));  sidx++)
    {
        int slip;
        int i;
        uint16_t rx_crc;
        uint16_t residual_crc;
        bool crc_ok;
        bool fill_ok;

        slip = slips[sidx];
        memcpy(trial, bits, sizeof(trial));
        /* Start at 17 (the first start bit, right after the 17-bit sync
           run), not 19: a lock that is off by one right at the sync/start17
           boundary -- confirmed live against d-modem/slmodemd, where the
           preamble scorer accepted a 17/18 match with bit 16 (last sync bit)
           actually holding what should be bit 17's value -- was structurally
           unreachable by every recovery path here, since all four started
           their earliest correctable boundary at 19, leaving positions
           17-18 (start bit, type bit) permanently excluded from correction.
           Bits 0-16 are a uniform run of 1s, so there is nothing to gain
           shifting from earlier than 17. */
        for (i = 17;  i < target;  i++)
        {
            int src;

            src = i + slip;
            trial[i] = (src >= 17 && src < target)  ?  bits[src]  :  0;
        }
        /*endfor*/
        crc_ok = mp_crc_ok(trial, type, &rx_crc, &residual_crc);
        fill_ok = mp_fill_ok(trial, type);
        if (crc_ok  &&  fill_ok)
        {
            memcpy(bits, trial, sizeof(trial));
            if (slip_out)
                *slip_out = slip;
            /*endif*/
            return true;
        }
        /*endif*/
    }
    /*endfor*/
    return false;
}
/*- End of function --------------------------------------------------------*/

static bool mp_try_boundary_slip_recovery(uint8_t bits[188], int type, int target, int *boundary_out, int *slip_out)
{
    static const int starts_mp0[] = {34, 51, 68};
    static const int starts_mp1[] = {34, 51, 68, 85, 102, 119, 136, 153, 170};
    static const int slips[] = {-2, -1, 1, 2};
    uint8_t trial[188];
    const int *starts;
    int nstarts;
    int si;

    starts = (type == 1) ? starts_mp1 : starts_mp0;
    nstarts = (type == 1) ? (int) (sizeof(starts_mp1)/sizeof(starts_mp1[0]))
                          : (int) (sizeof(starts_mp0)/sizeof(starts_mp0[0]));

    for (si = 0;  si < nstarts;  si++)
    {
        int boundary;
        int k;

        boundary = starts[si];
        if (boundary < 19 || boundary >= target)
            continue;
        /*endif*/
        for (k = 0;  k < (int) (sizeof(slips)/sizeof(slips[0]));  k++)
        {
            int slip;
            int i;
            uint16_t rx_crc;
            uint16_t residual_crc;
            bool crc_ok;
            bool fill_ok;

            slip = slips[k];
            memcpy(trial, bits, sizeof(trial));
            for (i = boundary;  i < target;  i++)
            {
                int src;

                src = i + slip;
                trial[i] = (src >= boundary && src < target) ? bits[src] : 0;
            }
            /*endfor*/
            crc_ok = mp_crc_ok(trial, type, &rx_crc, &residual_crc);
            fill_ok = mp_fill_ok(trial, type);
            if (crc_ok  &&  fill_ok)
            {
                memcpy(bits, trial, sizeof(trial));
                if (boundary_out)
                    *boundary_out = boundary;
                /*endif*/
                if (slip_out)
                    *slip_out = slip;
                /*endif*/
                return true;
            }
            /*endif*/
        }
        /*endfor*/
    }
    /*endfor*/
    return false;
}
/*- End of function --------------------------------------------------------*/

static void mp_apply_boundary_slip(uint8_t bits[188], int boundary, int target, int slip)
{
    uint8_t src[188];
    int i;

    memcpy(src, bits, sizeof(src));
    for (i = boundary;  i < target;  i++)
    {
        int from;

        from = i + slip;
        bits[i] = (from >= boundary && from < target) ? src[from] : 0;
    }
    /*endfor*/
}
/*- End of function --------------------------------------------------------*/

static bool mp_try_boundary_double_slip_recovery(uint8_t bits[188], int type, int target, int *b1_out, int *s1_out, int *b2_out, int *s2_out)
{
    static const int starts_mp0[] = {34, 51, 68};
    static const int starts_mp1[] = {34, 51, 68, 85, 102, 119, 136, 153, 170};
    static const int slips[] = {-2, -1, 1, 2};
    const int *starts;
    int nstarts;
    int i;
    int j;
    int k1;
    int k2;

    starts = (type == 1) ? starts_mp1 : starts_mp0;
    nstarts = (type == 1) ? (int) (sizeof(starts_mp1)/sizeof(starts_mp1[0]))
                          : (int) (sizeof(starts_mp0)/sizeof(starts_mp0[0]));

    for (i = 0;  i < nstarts;  i++)
    {
        int b1;

        b1 = starts[i];
        if (b1 < 19 || b1 >= target)
            continue;
        /*endif*/
        for (k1 = 0;  k1 < (int) (sizeof(slips)/sizeof(slips[0]));  k1++)
        {
            uint8_t trial1[188];
            uint16_t rx_crc;
            uint16_t residual_crc;
            bool crc_ok;
            bool fill_ok;

            memcpy(trial1, bits, sizeof(trial1));
            mp_apply_boundary_slip(trial1, b1, target, slips[k1]);
            crc_ok = mp_crc_ok(trial1, type, &rx_crc, &residual_crc);
            fill_ok = mp_fill_ok(trial1, type);
            if (crc_ok  &&  fill_ok)
            {
                memcpy(bits, trial1, sizeof(trial1));
                if (b1_out) *b1_out = b1;
                if (s1_out) *s1_out = slips[k1];
                if (b2_out) *b2_out = -1;
                if (s2_out) *s2_out = 0;
                return true;
            }
            /*endif*/

            for (j = i + 1;  j < nstarts;  j++)
            {
                int b2;

                b2 = starts[j];
                if (b2 < 19 || b2 >= target)
                    continue;
                /*endif*/
                for (k2 = 0;  k2 < (int) (sizeof(slips)/sizeof(slips[0]));  k2++)
                {
                    uint8_t trial2[188];

                    memcpy(trial2, trial1, sizeof(trial2));
                    mp_apply_boundary_slip(trial2, b2, target, slips[k2]);
                    crc_ok = mp_crc_ok(trial2, type, &rx_crc, &residual_crc);
                    fill_ok = mp_fill_ok(trial2, type);
                    if (crc_ok  &&  fill_ok)
                    {
                        memcpy(bits, trial2, sizeof(trial2));
                        if (b1_out) *b1_out = b1;
                        if (s1_out) *s1_out = slips[k1];
                        if (b2_out) *b2_out = b2;
                        if (s2_out) *s2_out = slips[k2];
                        return true;
                    }
                    /*endif*/
                }
                /*endfor*/
            }
            /*endfor*/
        }
        /*endfor*/
    }
    /*endfor*/
    return false;
}

static bool mp_try_boundary_bruteforce_recovery(uint8_t bits[188], int type, int target, int *changes_out)
{
    static const int base_slips[] = {-2, -1, 0, 1, 2};
    int boundaries[16];
    int boundary_count;
    int b;
    int bs;
    int total_states;

    boundary_count = 0;
    boundaries[boundary_count++] = 34;
    boundaries[boundary_count++] = 51;
    boundaries[boundary_count++] = 68;
    if (type == 1)
    {
        boundaries[boundary_count++] = 85;
        boundaries[boundary_count++] = 102;
        boundaries[boundary_count++] = 119;
        boundaries[boundary_count++] = 136;
        boundaries[boundary_count++] = 153;
        boundaries[boundary_count++] = 170;
    }
    /*endif*/
    for (b = 0;  b < boundary_count;  b++)
    {
        if (boundaries[b] >= target)
            break;
        /*endif*/
    }
    /*endfor*/
    boundary_count = b;
    if (boundary_count <= 0)
        return false;
    /*endif*/

    total_states = 1;
    for (b = 0;  b < boundary_count;  b++)
        total_states *= 3;
    /*endfor*/

    for (bs = 0;  bs < (int) (sizeof(base_slips)/sizeof(base_slips[0]));  bs++)
    {
        uint8_t base_trial[188];
        int base_slip;
        int state;

        base_slip = base_slips[bs];
        memcpy(base_trial, bits, sizeof(base_trial));
        if (base_slip != 0)
        {
            int i;

            /* See mp_try_slip_recovery(): start at 17, not 19, so a slip
               right at the sync/start17 boundary is reachable here too. */
            for (i = 17;  i < target;  i++)
            {
                int src;

                src = i + base_slip;
                base_trial[i] = (src >= 17  &&  src < target)  ?  bits[src]  :  0;
            }
            /*endfor*/
        }

        for (state = 0;  state < total_states;  state++)
        {
            uint8_t trial[188];
            int code;
            int i;
            int change_count;
            bool crc_ok;
            bool fill_ok;
            uint16_t rx_crc;
            uint16_t residual_crc;

            memcpy(trial, base_trial, sizeof(trial));
            code = state;
            change_count = (base_slip != 0) ? 1 : 0;
            for (i = 0;  i < boundary_count;  i++)
            {
                int trit;
                int slip;

                trit = code % 3;
                code /= 3;
                slip = trit - 1;   /* 0->-1, 1->0, 2->+1 */
                if (slip == 0)
                    continue;
                /*endif*/
                change_count++;
                mp_apply_boundary_slip(trial, boundaries[i], target, slip);
            }
            /*endfor*/
            if (change_count <= 0  ||  change_count > MP_BOUNDARY_BRUTEFORCE_MAX_CHANGES)
                continue;
            /*endif*/
            crc_ok = mp_crc_ok(trial, type, &rx_crc, &residual_crc);
            fill_ok = mp_fill_ok(trial, type);
            if (crc_ok  &&  fill_ok)
            {
                memcpy(bits, trial, sizeof(trial));
                if (changes_out)
                    *changes_out = change_count;
                /*endif*/
                return true;
            }
            /*endif*/
        }
        /*endfor*/
    }
    /*endfor*/
    return false;
}
/*- End of function --------------------------------------------------------*/

static int mp_data_bit_index(int type, int frame_idx)
{
    /* Return 1-based data-bit index within MP body (excluding inserted start bits).
       For inserted start-bit locations return -1. frame_idx is the absolute MP bit
       index where 0..16 are sync '1's, 17=start, 18=type. */
    int idx;
    int data_count;

    if (frame_idx < 19)
        return -1;
    /*endif*/
    if (mp_start_bit_ok(type, frame_idx, 1))
    {
        /* Non-start-bit location. Count non-start bits from frame_idx 19..frame_idx. */
        data_count = 0;
        for (idx = 19;  idx <= frame_idx;  idx++)
        {
            if (mp_start_bit_ok(type, idx, 1))
                data_count++;
            /*endif*/
        }
        /*endfor*/
        return data_count;
    }
    /*endif*/
    return -1;
}
/*- End of function --------------------------------------------------------*/

static void mp_pack_for_parser(uint8_t out[25], const uint8_t bits[], int type)
{
    int i;
    int limit;
    uint8_t *t;
    bitstream_state_t bs;

    memset(out, 0, 25);
    bitstream_init(&bs, true);
    t = out;
    limit = (type == 1)  ?  188  :  88;
    for (i = 18;  i < limit;  i++)
        bitstream_put(&bs, &t, bits[i], 1);
    /*endfor*/
    bitstream_flush(&bs, &t);
}
/*- End of function --------------------------------------------------------*/

static void log_mp_frame_diag(v34_rx_state_t *s, const uint8_t bits[], int type, bool crc_ok, uint16_t rx_crc, uint16_t residual_crc, bool fill_ok)
{
    if (type == 0)
    {
        span_log(s->logging, SPAN_LOG_FLOW,
                 "Rx - MP0 diag: sync[0..16]=all1 start17=%d type18=%d reserved19=%d "
                 "start34=%d start51=%d start68=%d crc_rx=0x%04X crc_res=0x%04X "
                 "fill85..87=%d%d%d crc_ok=%d fill_ok=%d\n",
                 bits[17], bits[18], bits[19],
                 bits[34], bits[51], bits[68],
                 rx_crc, residual_crc,
                 bits[85], bits[86], bits[87],
                 crc_ok, fill_ok);
    }
    else
    {
        span_log(s->logging, SPAN_LOG_FLOW,
                 "Rx - MP1 diag: sync[0..16]=all1 start17=%d type18=%d reserved19=%d "
                 "starts34/51/68/85/102/119/136/153/170=%d%d%d%d%d%d%d%d%d "
                 "crc_rx=0x%04X crc_res=0x%04X fill187=%d crc_ok=%d fill_ok=%d\n",
                 bits[17], bits[18], bits[19],
                 bits[34], bits[51], bits[68], bits[85], bits[102],
                 bits[119], bits[136], bits[153], bits[170],
                 rx_crc, residual_crc, bits[187],
                 crc_ok, fill_ok);
    }
    /*endif*/
}

/*- End of function --------------------------------------------------------*/

static void log_mp_lock_seed(v34_rx_state_t *s,
                             int hyp,
                             int type_bit,
                             int score,
                             int bit_pos,
                             uint32_t preamble_stream,
                             int pending_valid,
                             int pending_bit)
{
    char tail[33];
    char seed_bits[25];

    if (score < (MP_PREAMBLE_SCORE_MIN - 1))
        return;
    /*endif*/

    bits32_to_str(preamble_stream, tail);
    frame_bits_to_str(s->mp_frame_bits, 0, 24, seed_bits);
    span_log(s->logging, SPAN_LOG_FLOW,
             "Rx - Phase 4: MP lock seed hyp=%d type=%d score=%d/18 bit%d pending=%s%d "
             "frame_pos=%d target=%d preamble=0b%s seeded[0..23]=%s\n",
             hyp, type_bit, score, bit_pos,
             pending_valid ? "" : "none/",
             pending_valid ? pending_bit : 0,
             s->mp_frame_pos, s->mp_frame_target,
             tail, seed_bits);
}
/*- End of function --------------------------------------------------------*/

static bool mp_semantic_ok_phase4(v34_rx_state_t *s, const mp_t *mp, int type, const uint8_t bits[])
{
    int bit_idx;

    if (mp->type != type)
    {
        span_log(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4: MP%d semantic reject (type mismatch frame=%d parsed=%d)\n",
                 type, type, mp->type);
        return false;
    }
    /*endif*/
    if (bits[19] != 0)
    {
        span_log(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4: MP%d reserved19=%d (expected 0), tolerating\n",
                 type, bits[19]);
    }
    /*endif*/
    if (mp->bit_rate_a_to_c < 1  ||  mp->bit_rate_a_to_c > 14
        ||  mp->bit_rate_c_to_a < 1  ||  mp->bit_rate_c_to_a > 14)
    {
        span_log(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4: MP%d semantic reject (rate fields out of range a_to_c=%d c_to_a=%d)\n",
                 type, mp->bit_rate_a_to_c, mp->bit_rate_c_to_a);
        return false;
    }
    /*endif*/
    if (mp->trellis_size < V34_TRELLIS_16  ||  mp->trellis_size > V34_TRELLIS_64)
    {
        span_log(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4: MP%d semantic reject (invalid trellis code=%d)\n",
                 type, mp->trellis_size);
        return false;
    }
    /*endif*/
    if ((mp->signalling_rate_mask & 0x3FFF) == 0)
    {
        span_log(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4: MP%d semantic reject (empty signalling_rate_mask=0x%04X)\n",
                 type, mp->signalling_rate_mask & 0x7FFF);
        return false;
    }
    /*endif*/
    if (mp->signalling_rate_mask & 0x4000)
    {
        span_log(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4: MP%d reserved rate-mask bit set (0x%04X), tolerating for analog interop\n",
                 type, mp->signalling_rate_mask & 0x7FFF);
    }
    /*endif*/
    bit_idx = mp->bit_rate_a_to_c - 1;
    if (!(mp->signalling_rate_mask & (1 << bit_idx)))
    {
        span_log(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4: MP%d a_to_c rate %d missing from mask 0x%04X, tolerating for analog interop\n",
                 type, mp->bit_rate_a_to_c, mp->signalling_rate_mask & 0x7FFF);
    }
    /*endif*/
    bit_idx = mp->bit_rate_c_to_a - 1;
    if (!(mp->signalling_rate_mask & (1 << bit_idx)))
    {
        span_log(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4: MP%d c_to_a rate %d missing from mask 0x%04X, tolerating for analog interop\n",
                 type, mp->bit_rate_c_to_a, mp->signalling_rate_mask & 0x7FFF);
    }
    /*endif*/
    s->last_rx_mp = *mp;
    s->last_rx_mp_valid = true;
    return true;
}
/*- End of function --------------------------------------------------------*/

static void pack_output_bitstream(v34_rx_state_t *s)
{
    uint8_t *t;
    const uint8_t *u;
    int i;
    int n;
    int bit;
    int bb;
    int kk;

    span_log(s->logging,
             SPAN_LOG_FLOW,
             "Rx - Packed %p %8X - %X %X %X %X - %2X %2X %2X %2X %2X %2X %2X %2X\n",
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
    bitstream_init(&s->bs, true);
    t = s->rxbuf;
    bb = s->parms.b;
    kk = s->parms.k;
    /* If there are S bits, we switch between high mapping frames and low mapping frames based
       on the SWP pattern. We derive SWP algorithmically. Note that high/low mapping is only
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
    if (s->parms.k)
    {
        /* k is always < 32, so we always put the entire k bits into a single word */
        bitstream_put(&s->bs, &t, s->r0, kk);
        /* We can rely on this calculation always producing a value for chunk with no
           fractional part? */
        for (i = 0;  i < 4;  i++)
        {
            /* Some I bits */
            bitstream_put(&s->bs, &t, s->ibits[i], 3);
            if (s->parms.q)
            {
                /* Some Q bits */
                bitstream_put(&s->bs, &t, s->qbits[2*i], s->parms.q);
                bitstream_put(&s->bs, &t, s->qbits[2*i + 1], s->parms.q);
            }
            /*endif*/
        }
        /*endfor*/
    }
    else
    {
        /* If K is zero (i.e. b = 8, 9, 11, or 12), things need slightly special treatment */
        /* Pack 4 'i' fields */
        /* Need to treat 8, 9, 11, and 12 individually */
        n = bb - 8;
        for (i = 0;  i < n;  i++)
            bitstream_put(&s->bs, &t, s->ibits[i], 3);
        /*endfor*/
        for (  ;  i < 4;  i++)
            bitstream_put(&s->bs, &t, s->ibits[i], 2);
        /*endfor*/
    }
    /*endif*/
    bitstream_flush(&s->bs, &t);
#if 0
    printf("Block ");
    for (i = 0;  i < (s->b + 7)/8;  i++)
        printf("%02X ", s->rxbuf[i]);
    /*endfor*/
    printf("\n");
#endif

    bitstream_init(&s->bs, true);
    u = s->rxbuf;
    /* The first of the I bits might be auxiliary data */
    i = 0;
    s->aux_bit_cnt += s->parms.w;
    if (s->aux_bit_cnt >= s->parms.p)
    {
        s->aux_bit_cnt -= s->parms.p;
        for (  ;  i < kk;  i++)
        {
            bit = bitstream_get(&s->bs, &u, 1);
            s->put_bit(s->put_bit_user_data, descramble(s, bit));
        }
        /*endfor*/
        /* Auxiliary data bits are not scrambled (V.34/7) */
        bit = bitstream_get(&s->bs, &u, 1);
        if (s->put_aux_bit)
            s->put_aux_bit(s->put_bit_user_data, bit);
        /*endif*/
        i++;
    }
    for (  ;  i < bb;  i++)
    {
        bit = bitstream_get(&s->bs, &u, 1);
        s->put_bit(s->put_bit_user_data, descramble(s, bit));
    }
    /*endfor*/
}
/*- End of function --------------------------------------------------------*/

static void shell_unmap(v34_rx_state_t *s)
{
    int n21;
    int n22;
    int n23;
    int n24;
    int n41;
    int n42;
    int32_t n8;
    int k;
    int w41;
    int w42;
    int w2;
    int w8;
    const uint32_t *g2;
    const uint32_t *g4;
    const uint32_t *z8;

    g2 = g2s[s->parms.m];
    g4 = g4s[s->parms.m];
    z8 = z8s[s->parms.m];

    /* TODO: This code comes directly from the equations in V.34. Can it be improved? */
    n21 = (s->mjk[6] < s->parms.m - s->mjk[7])  ?  s->mjk[6]  :  (s->parms.m - 1 - s->mjk[7]);
    n22 = (s->mjk[4] < s->parms.m - s->mjk[5])  ?  s->mjk[4]  :  (s->parms.m - 1 - s->mjk[5]);
    n23 = (s->mjk[2] < s->parms.m - s->mjk[3])  ?  s->mjk[2]  :  (s->parms.m - 1 - s->mjk[3]);
    n24 = (s->mjk[0] < s->parms.m - s->mjk[1])  ?  s->mjk[0]  :  (s->parms.m - 1 - s->mjk[1]);

    w2 = s->mjk[4] + s->mjk[5];
    w41 = w2 + s->mjk[6] + s->mjk[7];
    n41 = 0;
    for (k = 0;  k < w2;  k++)
        n41 += g2[k]*g2[w41 - k];
    /*endfor*/
    n41 += n21*g2[w2];
    n41 += n22;

    w2 = s->mjk[0] + s->mjk[1];
    w42 = w2 + s->mjk[2] + s->mjk[3];
    n42 = 0;
    for (k = 0;  k < w2;  k++)
        n42 += g2[k]*g2[w42 - k];
    /*endfor*/
    n42 += n23*g2[w2];
    n42 += n24;

    w8 = w41 + w42;
    n8 = 0;
    for (k = 0;  k < w42;  k++)
        n8 += g4[k]*g4[w8 - k];
    /*endfor*/
    n8 += n41*g4[w42];
    n8 += n42;

    s->r0 = z8[w8] + n8;
}
/*- End of function --------------------------------------------------------*/

static int get_inverse_constellation_point(complexi16_t *point)
{
    int x;
    int y;

    x = point->re + 1;
    x = (x + 43)/4;
    if (x < 0)
        x = 0;
    else if (x > 22)
        x = 22;
    /*endif*/
    y = point->im + 1;
    y = (y + 43)/4;
    if (y < 0)
        y = 0;
    else if (y > 22)
        y = 22;
    /*endif*/
    return v34_inverse_superconstellation[y][x];
}
/*- End of function --------------------------------------------------------*/

static complexi16_t rotate90_counterclockwise(complexi16_t *x, int quads)
{
    complexi16_t y;

    /* Rotate a point counter-clockwise by quads 90 degree steps */
    switch (quads & 3)
    {
    case 0:
        y.re = x->re;
        y.im = x->im;
        break;
    case 1:
        y.re = -x->im;
        y.im = x->re;
        break;
    case 2:
        y.re = -x->re;
        y.im = -x->im;
        break;
    case 3:
        y.re = x->im;
        y.im = -x->re;
        break;
    }
    /*endswitch*/
    return y;
}
/*- End of function --------------------------------------------------------*/

/* Determine the 3 bits subset label for a particular constellation point */
static int16_t get_binary_subset_label(complexi16_t *pos)
{
    int x;
    int xored;
    int16_t subset;

    /* See V.34/9.6.3.1 */
    xored = pos->re ^ pos->im;
    x = xored & 2;
    subset = ((xored & 4) ^ (x << 1)) | (pos->re & 2) | (x >> 1);
    //printf("XXX Pre subset %d,%d => %d\n", pos->re, pos->im, subset);
    return subset;
}
/*- End of function --------------------------------------------------------*/

static int16_t get_binary_subset_label_q9_7(const complexi16_t *pos)
{
    complexi16_t integer;

    integer.re = pos->re >> 7;
    integer.im = pos->im >> 7;
    return get_binary_subset_label(&integer);
}
/*- End of function --------------------------------------------------------*/

static complexi16_t quantize_rx(v34_rx_state_t *s, complexi16_t *x)
{
    complexi16_t y;

    /* Value is stored in Q9.7 format. */
    /* Output integer values. i.e. Q16.0 */
    y.re = abs(x->re);
    y.im = abs(x->im);
    if (s->parms.b >= 56)
    {
        /* 2w is 4 */
        /* We must mask out the 1st and 2nd bits, because we are rounding to the 3rd bit.
           All numbers coming out of this routine should be a multiple of 4. */
        y.re = (y.re + 0x0FF) >> 7;
        y.re &= ~0x03;
        y.im = (y.im + 0x0FF) >> 7;
        y.im &= ~0x03;
    }
    else
    {
        /* 2w is 2 */
        /* We must mask out the 1st bit, because we are rounding to the 2nd bit.
           All numbers coming out of this routine should be even. */
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

static complexi16_t precoder_rx_filter(v34_rx_state_t *s)
{
    /* h's are stored in Q2.14
       x's are stored in Q9.7
       not sure about x's
       so product is stored in Q11.21 */
    int i;
    complexi32_t sum;
    complexi16_t p;

    sum.re = 0;
    sum.im = 0;
    for (i = 0;  i < 3;  i++)
    {
        sum.re += ((int32_t) s->x[i].re*s->h[i].re - (int32_t) s->x[i].im*s->h[i].im);
        sum.im += ((int32_t) s->x[i].re*s->h[i].im + (int32_t) s->x[i].im*s->h[i].re);
    }
    /*endfor*/
    /* Round Q11.21 number format to Q9.7 */
    p.re = (abs(sum.re) + 0x01FFFL) >> 14;
    if (sum.re < 0)
        p.re = -p.re;
    /*endif*/
    p.im = (abs(sum.im) + 0x01FFFL) >> 14;
    if (sum.im < 0)
        p.im = -p.im;
    /*endif*/
    for (i = 2;  i > 0;  i--)
        s->x[i] = s->x[i - 1];
    /*endfor*/
    return p;
}
/*- End of function --------------------------------------------------------*/

static complexi16_t prediction_error_filter(v34_rx_state_t *s)
{
    int i;
    complexi32_t sum;
    complexi16_t yt;

    sum.re = (int32_t) s->xt[0].re*16384;
    sum.im = (int32_t) s->xt[0].im*16384;
    for (i = 0;  i < 3;  i++)
    {
        sum.re += ((int32_t) s->xt[i + 1].re*s->h[i].re - (int32_t) s->xt[i + 1].im*s->h[i].im);
        sum.im += ((int32_t) s->xt[i + 1].im*s->h[i].re + (int32_t) s->xt[i + 1].re*s->h[i].im);
    }
    /*endfor*/
    for (i = 3;  i > 0;  i--)
        s->xt[i] = s->xt[i - 1];
    /*endfor*/
    /* Round Q11.21 number format to Q9.7 */
    yt.re = (abs(sum.re) + 0x01FFFL) >> 14;
    if (sum.re < 0)
        yt.re = -yt.re;
    /*endif*/
    yt.im = (abs(sum.im) + 0x01FFFL) >> 14;
    if (sum.im < 0)
        yt.im = -yt.im;
    /*endif*/
    return yt;
}
/*- End of function --------------------------------------------------------*/

static void quantize_n_ways(complexi16_t xy[], complexi16_t *yt)
{
    int16_t q;

    /* Quantize the current x,y point to points in the 4 2D subsets */
    /* TODO: This suits the 16 way convolutional code. The 32 and 64 way codes need 8 way quantization here */

    /* We want to quantize to a -7, -3, 1, 5, 9 grid, but -8, -4, 0, 4, 8 is easier to deal with.
       We subtract 1, quantize to the nearest multiple of 4, and add the 1 back. */
    /* Note that this works in Q9.7 format. */

    /* Offset by one */
    xy[0].re = yt->re - FP_Q9_7(1);
    xy[0].im = yt->im - FP_Q9_7(1);
    /* Round to the nearest multiple of 4 towards zero */
    q = xy[0].re;
    xy[0].re = (abs(xy[0].re) + FP_Q9_7(2)) & ~(FP_Q9_7(4) - 1);
    if (q < 0)
        xy[0].re = -xy[0].re;
    /*endif*/
    q = xy[0].im;
    xy[0].im = (abs(xy[0].im) + FP_Q9_7(2)) & ~(FP_Q9_7(4) - 1);
    if (q < 0)
        xy[0].im = -xy[0].im;
    /*endif*/
    /* Restore the offset of one */
    xy[0].re += FP_Q9_7(1);
    xy[0].im += FP_Q9_7(1);

    /* Subset 0 done. Figure out the rest as offsets from subset 0 */
    xy[1].re = xy[0].re;
    if (yt->re < xy[0].re)
    {
        xy[2].re = xy[0].re - FP_Q9_7(2);
        xy[3].re = xy[0].re - FP_Q9_7(2);
    }
    else
    {
        xy[2].re = xy[0].re + FP_Q9_7(2);
        xy[3].re = xy[0].re + FP_Q9_7(2);
    }
    /*endif*/
    if (yt->im < xy[0].im)
    {
        xy[1].im = xy[0].im - FP_Q9_7(2);
        xy[2].im = xy[0].im - FP_Q9_7(2);
    }
    else
    {
        xy[1].im = xy[0].im + FP_Q9_7(2);
        xy[2].im = xy[0].im + FP_Q9_7(2);
    }
    /*endif*/
    xy[3].im = xy[0].im;
}
/*- End of function --------------------------------------------------------*/

static void viterbi_calculate_candidate_errors(int16_t error[4], complexi16_t xy[4], complexi16_t *yt)
{
    int i;
    complexi32_t diff;
    int32_t err;

    /* Calculate the errors between yt and the four 2D candidates. Errors are stored as 6:10 */
//printf("CIC");
    for (i = 0;  i < 4;  i++)
    {
        diff.re = (int32_t) xy[i].re - yt->re;
        diff.im = (int32_t) xy[i].im - yt->im;
        err = diff.re*diff.re + diff.im*diff.im;
        error[i] = err >> 4;
//printf(" %3d", error[i]);
    }
    /*endfor*/
//printf("\n");
}
/*- End of function --------------------------------------------------------*/

static void viterbi_calculate_branch_errors(viterbi_t *s, complexi16_t xy[2][4], int invert)
{
    static const uint8_t kk[8][4] =
    {
        {0, 0, 2, 2},
        {0, 1, 2, 3},
        {0, 2, 2, 0},
        {0, 3, 2, 1},
        {1, 1, 3, 3},
        {1, 2, 3, 0},
        {1, 3, 3, 1},
        {1, 0, 3, 2}
    };
    int br;
    int n;
    int inv;
    int error0;
    int error1;
    int smaller;
    int k0;
    int k1;

    inv = (invert)  ?  4  :  0;
    for (br = 0;  br < 8;  br++)
    {
        n = br ^ inv;
        error0 = s->error[0][kk[n][0]] + s->error[1][kk[n][1]];
        error1 = s->error[0][kk[n][2]] + s->error[1][kk[n][3]];
        if (error0 < error1)
        {
            smaller = error0;
            k0 = kk[n][0];
            k1 = kk[n][1];
        }
        else
        {
            smaller = error1;
            k0 = kk[n][2];
            k1 = kk[n][3];
        }
        /*endif*/
        s->branch_error[br] = smaller;
        s->vit[s->ptr].branch_error_x[br] = smaller;
        s->vit[s->ptr].bb[0][br] = xy[0][k0];
        s->vit[s->ptr].bb[1][br] = xy[1][k1];
    }
    /*endfor*/
}
/*- End of function --------------------------------------------------------*/

static void viterbi_update_path_metrics(viterbi_t *s,
                                        complexi16_t xy[2][4],
                                        int invert)
{
    static const int8_t geometric_branch[4][4] =
    {
        {0, 1, 2, 3},
        {7, 4, 5, 6},
        {2, 3, 0, 1},
        {5, 6, 7, 4}
    };
    int16_t state;
    int16_t k0;
    int16_t k1;
    uint32_t curr_min_metric;
    uint32_t min_metric;
    uint32_t metric;
    int prev_ptr;

    if (getenv("SPANDSP_V34_DIAG_VITERBI"))
    {
        static int diag_state = 0;
        static int diag_count = 0;
        int dk0 = -1;
        int dk1 = -1;

        for (k0 = 0; k0 < 4; k0++)
        {
            if (s->error[0][k0] == 0)
                dk0 = k0;
            if (s->error[1][k0] == 0)
                dk1 = k0;
        }
        if (dk0 >= 0 && dk1 >= 0)
        {
            int subset0 = get_binary_subset_label_q9_7(&xy[0][dk0]) & 7;
            int subset1 = get_binary_subset_label_q9_7(&xy[1][dk1]) & 7;
            int input = conv_encode_input[subset0][subset1];
            int expected = (((input & 3) << 1) | (diag_state & 1))
                         ^ (invert ? 1 : 0);

            fprintf(stderr,
                    "V34VIT n=%d state=%d y0=%d input=%x k=%d,%d geom=%d expected=%d invert=%d next=%d\n",
                    diag_count,
                    diag_state,
                    diag_state & 1,
                    input,
                    dk0,
                    dk1,
                    geometric_branch[dk0][dk1],
                    expected,
                    invert,
                    s->encode_table[diag_state][input]);
            diag_state = s->encode_table[diag_state][input];
            diag_count++;
        }
    }

    curr_min_metric = UINT32_MAX;
    /* Build the full 4D candidate set.  The old dormant decoder collapsed
       these to eight branches and then tried to invert a generated table;
       that loses the actual Table-13 subset pair and is not a valid trellis
       metric. */
    for (k0 = 0; k0 < 4; k0++)
    {
        for (k1 = 0; k1 < 4; k1++)
        {
            int branch = 4*k0 + k1;
            s->vit[s->ptr].bb[0][branch] = xy[0][k0];
            s->vit[s->ptr].bb[1][branch] = xy[1][k1];
            s->vit[s->ptr].branch_error_x[branch] =
                s->error[0][k0] + s->error[1][k1];
        }
        /*endfor*/
    }
    /*endfor*/
    prev_ptr = (s->ptr - 1) & 0xF;
    for (state = 0; state < s->state_count; state++)
    {
        s->vit[s->ptr].cumulative_path_metric[state] = UINT32_MAX;
        s->vit[s->ptr].previous_path_ptr[state] = 0;
        s->vit[s->ptr].pts[state] = 0;
    }
    /*endfor*/
    for (state = 0; state < s->state_count; state++)
    {
        for (k0 = 0; k0 < 4; k0++)
        {
            int subset0 = get_binary_subset_label_q9_7(&xy[0][k0]) & 7;

            for (k1 = 0; k1 < 4; k1++)
            {
                int subset1 = get_binary_subset_label_q9_7(&xy[1][k1]) & 7;
                int input = conv_encode_input[subset0][subset1];
                int next_state = s->encode_table[state][input];
                int branch = 4*k0 + k1;

                /* For the 16-state rate-2/3 encoder, Y1/Y2 and the old
                   state's Y0 select one of the eight 4D branches.  The
                   previous full-candidate update used Y1/Y2 only for the
                   state transition and allowed geometrically impossible Y0
                   branches, making the decoder collapse under tiny noise.
                   V0 flips the Y0 branch bit at the two half-frame
                   boundaries. */
                if (s->state_count == 16
                    && geometric_branch[k0][k1]
                       != ((((input & 3) << 1) | (state & 1))
                           ^ (invert ? 1 : 0))) {
                    continue;
                }

                metric = s->vit[prev_ptr].cumulative_path_metric[state]
                       + s->vit[s->ptr].branch_error_x[branch];
                if (metric < s->vit[s->ptr].cumulative_path_metric[next_state])
                {
                    s->vit[s->ptr].cumulative_path_metric[next_state] = metric;
                    s->vit[s->ptr].previous_path_ptr[next_state] = state;
                    s->vit[s->ptr].pts[next_state] = branch;
                }
                /*endif*/
            }
            /*endfor*/
        }
        /*endfor*/
    }
    /*endfor*/
    for (state = 0; state < s->state_count; state++)
    {
        metric = s->vit[s->ptr].cumulative_path_metric[state];
        if (metric < curr_min_metric)
        {
            curr_min_metric = metric;
            s->curr_min_state = state;
        }
        /*endif*/
    }
    /*endfor*/
//printf("GGG %p min metric %d, state %d\n", s, curr_min_metric, s->curr_min_state);
//printf("JJJ %p ", s);
    for (state = 0; state < s->state_count; state++)
    {
        s->vit[s->ptr].cumulative_path_metric[state] -= curr_min_metric;
//printf("%4d ", s->cumulative_path_metric[s->ptr][i]);
    }
    /*endfor*/
//printf("\n");
}
/*- End of function --------------------------------------------------------*/

static void viterbi_trace_back(viterbi_t *s, complexi16_t y[2])
{
    int branch;
    int next_state;
    int last_baud;
    int i;

    next_state = s->curr_min_state;
    last_baud = (s->ptr - 15) & 0xF;
//printf("FFF %p %2d", s, next_state);
    for (i = s->ptr;  i != last_baud;  i = (i - 1) & 0xF)
    {
        next_state = s->vit[i].previous_path_ptr[next_state];
//printf(" %2d", next_state);
    }
    /*endfor*/
    for (i = 0;  i < 8;  i++)
    {
        if (s->vit[last_baud].branch_error_x[i] == 0)
        {
            branch = i;
            break;
        }
    }
    /*endfor*/
    branch = s->vit[last_baud].pts[next_state];
//printf(" (%d)\n", branch);

    y[0] = s->vit[last_baud].bb[0][branch];
    y[1] = s->vit[last_baud].bb[1][branch];
}
/*- End of function --------------------------------------------------------*/

static int viterbi_set_trellis(viterbi_t *s,
                               const uint8_t (*encode_table)[16],
                               int state_count)
{
    int state;
    int input;
    int previous;
    int found;

    if (!s || !encode_table
        || (state_count != 16 && state_count != 32 && state_count != 64))
        return -1;
    /* In Figures 10-12/V.34 each destination state has one predecessor for
       each of the four coded input pairs.  Build the inverse mapping from the
       authoritative encoder tables so the receiver supports every negotiated
       trellis without maintaining another set of generated tables. */
    for (state = 0;  state < state_count;  state++)
    {
        for (input = 0;  input < 4;  input++)
        {
            found = -1;
            for (previous = 0;  previous < state_count;  previous++)
            {
                if (encode_table[previous][input] == state)
                    found = previous;
                /*endif*/
            }
            /*endfor*/
            if (found < 0)
                return -1;
            s->previous_state[state][input] = (uint8_t) found;
            s->branch[state][input] = (uint8_t) ((input << 1) | (found & 1));
        }
        /*endfor*/
    }
    /*endfor*/
    s->state_count = state_count;
    s->encode_table = encode_table;
    memset(s->vit, 0, sizeof(s->vit));
    s->ptr = 0;
    s->windup = 15;
    s->curr_min_state = 0;
    return 0;
}
/*- End of function --------------------------------------------------------*/

static int set_trellis_mode(v34_state_t *s, int trellis_size)
{
    const uint8_t (*table)[16];
    int states;

    switch (trellis_size)
    {
    case V34_TRELLIS_16:
        table = v34_conv16_encode_table;
        states = 16;
        break;
    case V34_TRELLIS_32:
        table = v34_conv32_encode_table;
        states = 32;
        break;
    case V34_TRELLIS_64:
        table = v34_conv64_encode_table;
        states = 64;
        break;
    default:
        return -1;
    }
    /*endswitch*/
    s->tx.conv_encode_table = table;
    return viterbi_set_trellis(&s->rx.viterbi, table, states);
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

static int process_rx_info0(v34_rx_state_t *s, uint8_t buf[])
{
    bitstream_state_t bs;
    const uint8_t *t;
    uint8_t raw_26_27;

    memset(&s->far_capabilities, 0, sizeof(s->far_capabilities));
    s->info0d_extensions_valid = false;
    s->info0_raw_26_27 = 0;
    s->info0d_nominal_power_code = 0;
    s->info0d_max_power_code = 0;
    s->info0d_power_measured_at_codec_output = false;
    s->info0d_pcm_alaw = false;
    s->info0d_upstream_3429_support = false;
    s->info0d_reserved_41 = 0;
    bitstream_init(&bs, true);
    t = buf;
    s->far_capabilities.support_baud_rate_low_carrier[V34_BAUD_RATE_2400] =
    s->far_capabilities.support_baud_rate_high_carrier[V34_BAUD_RATE_2400] = true;
    s->far_capabilities.support_baud_rate_low_carrier[V34_BAUD_RATE_2743] =
    s->far_capabilities.support_baud_rate_high_carrier[V34_BAUD_RATE_2743] = bitstream_get(&bs, &t, 1);
    s->far_capabilities.support_baud_rate_low_carrier[V34_BAUD_RATE_2800] =
    s->far_capabilities.support_baud_rate_high_carrier[V34_BAUD_RATE_2800] = bitstream_get(&bs, &t, 1);
    s->far_capabilities.support_baud_rate_low_carrier[V34_BAUD_RATE_3429] =
    s->far_capabilities.support_baud_rate_high_carrier[V34_BAUD_RATE_3429] = bitstream_get(&bs, &t, 1);
    s->far_capabilities.support_baud_rate_low_carrier[V34_BAUD_RATE_3000] = bitstream_get(&bs, &t, 1);
    s->far_capabilities.support_baud_rate_high_carrier[V34_BAUD_RATE_3000] = bitstream_get(&bs, &t, 1);
    s->far_capabilities.support_baud_rate_low_carrier[V34_BAUD_RATE_3200] = bitstream_get(&bs, &t, 1);
    s->far_capabilities.support_baud_rate_high_carrier[V34_BAUD_RATE_3200] = bitstream_get(&bs, &t, 1);
    s->far_capabilities.rate_3429_allowed = bitstream_get(&bs, &t, 1);
    s->far_capabilities.support_power_reduction = bitstream_get(&bs, &t, 1);
    s->far_capabilities.max_baud_rate_difference = bitstream_get(&bs, &t, 3);
    s->far_capabilities.from_cme_modem = bitstream_get(&bs, &t, 1);
    s->far_capabilities.support_1664_point_constellation = bitstream_get(&bs, &t, 1);
    if (s->v90_mode && s->calling_party)
    {
        /* V.90 INFO0d (62 bits): bits 26-27 are reserved (not tx_clock_source),
           bit 28 is acknowledgement, then additional digital modem fields follow. */
        raw_26_27 = bitstream_get(&bs, &t, 2);     /* 26:27 reserved (V.90), V.92 flags */
        s->info0_raw_26_27 = raw_26_27;
        s->info0_acknowledgement = bitstream_get(&bs, &t, 1);  /* 28 */
        /* 29:32    Digital modem nominal TX power (-1 dBm0 steps, 0=-6 dBm0) */
        s->info0d_nominal_power_code = bitstream_get(&bs, &t, 4);
        /* 33:37    Maximum digital modem TX power (-0.5 dBm0 steps) */
        s->info0d_max_power_code = bitstream_get(&bs, &t, 5);
        /* 38       Power measurement location (1=codec output) */
        s->info0d_power_measured_at_codec_output = bitstream_get(&bs, &t, 1);
        /* 39       PCM coding: 0=µ-law, 1=A-law */
        s->far_capabilities.tx_clock_source = bitstream_get(&bs, &t, 1);  /* reuse field for pcm_law */
        s->info0d_pcm_alaw = (s->far_capabilities.tx_clock_source != 0);
        /* 40       V.90 upstream symbol rate 3429 support */
        s->info0d_upstream_3429_support = bitstream_get(&bs, &t, 1);
        /* 41       Reserved */
        s->info0d_reserved_41 = bitstream_get(&bs, &t, 1);
        s->info0d_extensions_valid = true;
        span_log(s->logging, SPAN_LOG_FLOW, "Rx INFO0d (V.90): PCM law=%s, ack=%d\n",
                 s->far_capabilities.tx_clock_source ? "A-law" : "u-law",
                 s->info0_acknowledgement);
    }
    else
    {
        raw_26_27 = bitstream_get(&bs, &t, 2);
        s->info0_raw_26_27 = raw_26_27;
        s->far_capabilities.tx_clock_source = raw_26_27;
        s->info0_acknowledgement = bitstream_get(&bs, &t, 1);
        if (s->v90_mode)
        {
            span_log(s->logging, SPAN_LOG_FLOW,
                     "Rx INFO0a V.92 flags: capability(bit26)=%d, short-phase2(bit27)=%d\n",
                     (raw_26_27 & 0x01U) != 0,
                     (raw_26_27 & 0x02U) != 0);
        }
    }
    /*endif*/

    log_info0(s->logging, false, &s->far_capabilities, s->info0_acknowledgement);
    return 0;
}
/*- End of function --------------------------------------------------------*/

static int process_rx_info1c(v34_rx_state_t *s, info1c_t *info1c, uint8_t buf[])
{
    bitstream_state_t bs;
    const uint8_t *t;
    int i;

    bitstream_init(&bs, true);
    t = buf;
    /* 12:14    Minimum power reduction to be implemented by the answer modem transmitter. An integer between 0 and 7
                gives the recommended power reduction in dB. These bits shall indicate 0 if INFO0a indicated that the answer
                modem transmitter cannot reduce its power. */
    info1c->power_reduction = bitstream_get(&bs, &t, 3);
    /* 15:17    Additional power reduction, below that indicated by bits 12-14, which can be tolerated by the call modem
                receiver. An integer between 0 and 7 gives the additional power reduction in dB. These bits shall indicate 0 if
                INFO0a indicated that the answer modem transmitter cannot reduce its power. */
    info1c->additional_power_reduction = bitstream_get(&bs, &t, 3);
    /* 18:24    Length of MD to be transmitted by the call modem during Phase 3. An integer between 0 and 127 gives the
                length of this sequence in 35 ms increments. */
    info1c->md = bitstream_get(&bs, &t, 7);
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

    /* 70:78    Probing results pertaining to a final symbol rate selection of 3429 symbols per second. The coding of these
                9 bits is identical to that for bits 25-33. Information in this field shall be consistent with the answer modem
                capabilities indicated in INFO0a. */
    for (i = 0;  i <= 5;  i++)
    {
        info1c->rate_data[i].use_high_carrier = bitstream_get(&bs, &t, 1);
        info1c->rate_data[i].pre_emphasis = bitstream_get(&bs, &t, 4);
        info1c->rate_data[i].max_bit_rate = bitstream_get(&bs, &t, 4);
    }
    /*endfor*/
    /* 79:88    Frequency offset of the probing tones as measured by the call modem receiver. The frequency offset number
                shall be the difference between the nominal 1050 Hz line probing signal tone received and the 1050 Hz tone
                transmitted, f(received) and f(transmitted). A two's complement signed integer between -511 and 511 gives the
                measured offset in 0.02 Hz increments. Bit 88 is the sign bit of this integer. The frequency offset measurement
                shall be accurate to 0.25 Hz. Under conditions where this accuracy cannot be achieved, the integer shall be set
                to -512 indicating that this field is to be ignored. */
    info1c->freq_offset = bitstream_get(&bs, &t, 10);
    if ((info1c->freq_offset & 0x200))
        info1c->freq_offset = -(info1c->freq_offset ^ 0x3FF) - 1;
    /*endif*/

    log_info1c(s->logging, false, info1c);
    s->info1c_received = true;
    return 0;
}
/*- End of function --------------------------------------------------------*/

static int process_rx_info1a(v34_rx_state_t *s, info1a_t *info1a, uint8_t buf[])
{
    bitstream_state_t bs;
    const uint8_t *t;
    uint16_t raw_freq;

    bitstream_init(&bs, true);
    t = buf;

    if (s->v90_mode)
    {
        /* V.90 §8.2.3.2 Table 10: INFO1a from analog modem when V.90 is selected.
           Different field layout from V.34 INFO1a. */
        /* 12:17    Reserved for ITU (set to 0 by analog modem) */
        s->info1a_raw_12_17 = bitstream_get(&bs, &t, 6);
        info1a->power_reduction = 0;
        info1a->additional_power_reduction = 0;
        /* 18:24    Length of MD to be transmitted by the analog modem during Phase 3 */
        info1a->md = bitstream_get(&bs, &t, 7);
        /* 25:31    U_INFO: Ucode of the PCM codeword for the 2-point train.
                    Stored in use_high_carrier (1 bit) + preemphasis_filter (4 bits) + 2 extra bits.
                    We pack the 7-bit value into max_data_rate for the caller to retrieve. */
        info1a->max_data_rate = bitstream_get(&bs, &t, 7);  /* U_INFO */
        info1a->use_high_carrier = false;
        info1a->preemphasis_filter = 0;
        /* V.90 Table 10 reserves 32:33.  V.92 Table 19 retains bit 32 as
           reserved and uses bit 33 to select the high upstream carrier. */
        s->info1a_raw_32_33 = bitstream_get(&bs, &t, 2);
        /* 34:36    Symbol rate for analog→digital (upstream). 3=3000, 4=3200, 5=3429 */
        info1a->baud_rate_a_to_c = bitstream_get(&bs, &t, 3);
        /* 37:39    Symbol rate of 8000 (the integer 6) — V.90 PCM downstream rate */
        info1a->baud_rate_c_to_a = bitstream_get(&bs, &t, 3);
        /* 40:49    Frequency offset (same as V.34) */
        raw_freq = (uint16_t) bitstream_get(&bs, &t, 10);
        s->info1a_raw_40_49 = raw_freq;
        info1a->freq_offset = (int) raw_freq;
        if ((info1a->freq_offset & 0x200))
            info1a->freq_offset = -(info1a->freq_offset ^ 0x3FF) - 1;
        /*endif*/

        if (info1a->baud_rate_c_to_a >= 0  &&  info1a->baud_rate_c_to_a <= 5)
        {
            /* V.90 §8.2.3.2 Table 11 / §9.2.1.1.8: bits 37:39 in 0..5 means
               the analogue modem selected V.34.  The frame then carries the
               standard V.34 INFO1a fields (Table 11 is bit-identical to
               Table 16/V.34), NOT the Table 10 U_INFO layout parsed above.
               Re-parse from the top with the V.34 field layout.  Live ground
               truth (CX93001, 2026-07-23): raw 0f c0 34 89 c0 ba 66 decodes
               to power reduction 7+1 dB, low carrier, pre-emphasis 3,
               projected max 31200, 3200/3200 baud -- all coherent, where the
               Table 10 reading gave nonzero "reserved" bits. */
            bitstream_init(&bs, true);
            t = buf;
            info1a->power_reduction = bitstream_get(&bs, &t, 3);
            info1a->additional_power_reduction = bitstream_get(&bs, &t, 3);
            info1a->md = bitstream_get(&bs, &t, 7);
            info1a->use_high_carrier = bitstream_get(&bs, &t, 1);
            info1a->preemphasis_filter = bitstream_get(&bs, &t, 4);
            info1a->max_data_rate = bitstream_get(&bs, &t, 4);
            info1a->baud_rate_a_to_c = bitstream_get(&bs, &t, 3);
            info1a->baud_rate_c_to_a = bitstream_get(&bs, &t, 3);
            raw_freq = (uint16_t) bitstream_get(&bs, &t, 10);
            s->info1a_raw_40_49 = raw_freq;
            info1a->freq_offset = (int) raw_freq;
            if ((info1a->freq_offset & 0x200))
                info1a->freq_offset = -(info1a->freq_offset ^ 0x3FF) - 1;
            /*endif*/
            s->v90_v34_fallback = true;

            span_log(s->logging, SPAN_LOG_FLOW, "Rx INFO1a (V.90 Table 11 - V.34 selected):\n");
            span_log(s->logging, SPAN_LOG_FLOW, "  Power reduction = %d dB + %d dB additional\n",
                     info1a->power_reduction, info1a->additional_power_reduction);
            span_log(s->logging, SPAN_LOG_FLOW, "  Length of MD = %dms\n", info1a->md*35);
            span_log(s->logging, SPAN_LOG_FLOW, "  High carrier (digital->analogue) = %d\n", info1a->use_high_carrier);
            span_log(s->logging, SPAN_LOG_FLOW, "  Pre-emphasis filter = %d\n", info1a->preemphasis_filter);
            span_log(s->logging, SPAN_LOG_FLOW, "  Projected max rate = %d (%d bps)\n",
                     info1a->max_data_rate, info1a->max_data_rate*2400);
            span_log(s->logging, SPAN_LOG_FLOW, "  Symbol rate analogue->digital = %d\n", info1a->baud_rate_a_to_c);
            span_log(s->logging, SPAN_LOG_FLOW, "  Symbol rate digital->analogue = %d\n", info1a->baud_rate_c_to_a);
            if (info1a->freq_offset == -512)
                span_log(s->logging, SPAN_LOG_FLOW, "  Frequency offset not available\n");
            else
                span_log(s->logging, SPAN_LOG_FLOW, "  Frequency offset = %fHz\n", info1a->freq_offset*0.02f);

            /* The analogue modem transmits Phase 3 at the a->c symbol rate.
               Table 11: "The carrier frequency ... to be used are those
               already indicated for this symbol rate in INFO1d" -- our
               INFO1d advertises the high carrier for every upstream rate
               (see prepare_info1c()). */
            if (info1a->baud_rate_a_to_c >= 0  &&  info1a->baud_rate_a_to_c <= 5)
            {
                s->baud_rate = info1a->baud_rate_a_to_c;
                s->high_carrier = true;
                s->v34_carrier_phase_rate = dds_phase_ratef(carrier_frequency(s->baud_rate, s->high_carrier));
                create_godard_coeffs(&s->pri_ted,
                                     carrier_frequency(s->baud_rate, s->high_carrier),
                                     baud_rate_parameters[s->baud_rate].baud_rate,
                                     0.99f);
            }
            /*endif*/
        }
        else
        {
            span_log(s->logging, SPAN_LOG_FLOW, "Rx INFO1a (V.90 Table 10):\n");
            span_log(s->logging, SPAN_LOG_FLOW, "  Length of MD = %dms\n", info1a->md*35);
            span_log(s->logging, SPAN_LOG_FLOW, "  U_INFO = %d\n", info1a->max_data_rate);
            span_log(s->logging, SPAN_LOG_FLOW, "  Upstream symbol rate code = %d\n", info1a->baud_rate_a_to_c);
            span_log(s->logging, SPAN_LOG_FLOW, "  Downstream rate code = %d (8000 PCM)\n", info1a->baud_rate_c_to_a);
            if (info1a->freq_offset == -512)
                span_log(s->logging, SPAN_LOG_FLOW, "  Frequency offset not available\n");
            else
                span_log(s->logging, SPAN_LOG_FLOW, "  Frequency offset = %fHz\n", info1a->freq_offset*0.02f);

            /* In V.90, the upstream (analog→digital) uses V.34 modulation at the selected baud rate.
               Use the upstream baud rate for the primary channel RX configuration. */
            if (info1a->baud_rate_a_to_c >= 0  &&  info1a->baud_rate_a_to_c <= 5)
            {
                s->baud_rate = info1a->baud_rate_a_to_c;
                if (s->info1a_raw_32_33 & 0x2)
                    s->high_carrier = true;
                s->v34_carrier_phase_rate = dds_phase_ratef(carrier_frequency(s->baud_rate, s->high_carrier));
                create_godard_coeffs(&s->pri_ted,
                                     carrier_frequency(s->baud_rate, s->high_carrier),
                                     baud_rate_parameters[s->baud_rate].baud_rate,
                                     0.99f);
            }
            /*endif*/
        }
        /*endif*/
    }
    else
    {
        s->info1a_raw_12_17 = 0;
        s->info1a_raw_32_33 = 0;
        s->info1a_raw_40_49 = 0;
        /* Standard V.34 INFO1a parsing */
        /* 12:14    Minimum power reduction */
        info1a->power_reduction = bitstream_get(&bs, &t, 3);
        /* 15:17    Additional power reduction */
        info1a->additional_power_reduction = bitstream_get(&bs, &t, 3);
        /* 18:24    Length of MD */
        info1a->md = bitstream_get(&bs, &t, 7);
        /* 25       High carrier frequency */
        info1a->use_high_carrier = bitstream_get(&bs, &t, 1);
        /* 26:29    Pre-emphasis filter index */
        info1a->preemphasis_filter = bitstream_get(&bs, &t, 4);
        /* 30:33    Projected maximum data rate */
        info1a->max_data_rate = bitstream_get(&bs, &t, 4);
        /* 34:36    Symbol rate answer→call */
        info1a->baud_rate_a_to_c = bitstream_get(&bs, &t, 3);
        /* 37:39    Symbol rate call→answer */
        info1a->baud_rate_c_to_a = bitstream_get(&bs, &t, 3);
        /* 40:49    Frequency offset */
        info1a->freq_offset = bitstream_get(&bs, &t, 10);
        if ((info1a->freq_offset & 0x200))
            info1a->freq_offset = -(info1a->freq_offset ^ 0x3FF) - 1;
        /*endif*/
        s->baud_rate = info1a->baud_rate_c_to_a;
        s->v34_carrier_phase_rate = dds_phase_ratef(carrier_frequency(s->baud_rate, s->high_carrier));
        create_godard_coeffs(&s->pri_ted,
                             carrier_frequency(s->baud_rate, s->high_carrier),
                             baud_rate_parameters[s->baud_rate].baud_rate,
                             0.99f);

        log_info1a(s->logging, false, info1a);
    }
    /*endif*/
    s->info1a_received = true;

#if defined(SPANDSP_USE_FIXED_POINT)
    s->pri_ted.symbol_sync_low[0] = s->pri_ted.symbol_sync_low[1] = 0;
    s->pri_ted.symbol_sync_high[0] = s->pri_ted.symbol_sync_high[1] = 0;
    s->pri_ted.symbol_sync_dc_filter[0] = s->pri_ted.symbol_sync_dc_filter[1] = 0;
    s->pri_ted.baud_phase = 0;
#else
    s->pri_ted.symbol_sync_low[0] = s->pri_ted.symbol_sync_low[1] = 0.0f;
    s->pri_ted.symbol_sync_high[0] = s->pri_ted.symbol_sync_high[1] = 0.0f;
    s->pri_ted.symbol_sync_dc_filter[0] = s->pri_ted.symbol_sync_dc_filter[1] = 0.0f;
    s->pri_ted.baud_phase = 0.0f;
#endif

    return 0;
}
/*- End of function --------------------------------------------------------*/

static void v90_enter_phase3_from_info1a(v34_rx_state_t *s)
{
    v34_state_t *owner;

    /* INFO1a can finish part-way through an RTP media frame.  Merely changing
       current_demodulator here lets the uninitialised primary-channel frontend
       consume the residue of that frame before the TX state machine gets its
       next callback and performs the normal Phase 3 reset.  That shifted the
       live PP acquisition by 24 bauds relative to replay and destroyed the TRN
       lock.  Enter Phase 3 synchronously so the first primary-channel sample is
       processed with the same clean frontend used by offline replay. */
    owner = (v34_state_t *) ((char *) s - offsetof(v34_state_t, rx));

    /* Table 10 bits 37:39 confirm which protocol the analogue modem actually
       committed to: the integer 6 means genuine V.90 PCM downstream; 0-5
       means it declined V.90 and is falling back to plain V.34 at that
       symbol rate (V.90 9.2.1.1.8). The two cases need different upstream
       scrambler polynomials. Forcing the V.90 tap onto a real V.34-fallback
       signal corrupts every descrambled bit from here on — live interop
       with a CX93001 that declined V.90 showed exactly this: an MP
       hypothesis would lock, then every subsequent frame failed CRC and got
       rejected, forever (2026-07-19). */
    if (s->info1a.baud_rate_c_to_a == 6)
    {
        /* V.90 §9 uses the analog-modem upstream scrambler 1 + x^-5 + x^-23.
           SpanDSP's ordinary V.34 answerer initialisation selects tap 17 for
           the far-end caller, which makes SmartLink TRN look random (~52%
           ones). The captured upstream resolves at 96-99% with the V.90 tap
           value 4, so select it before Phase 3 resets its hypothesis banks. */
        s->scrambler_tap = 4;
        s->mp_phase4_default_scrambler_tap = 4;
    }
    else
    {
        /* V.90 §9.2.1.1.8: the digital modem proceeds per 11.3.1.1/V.34
           "assuming the role of a call modem", which makes the analogue
           modem the V.34 answer modem.  The answer modem scrambles with
           GPA (1 + x^-5 + x^-23), so descramble its TRN/J with tap 4 and
           scramble our own TX with GPC (tap 17).  The earlier tap-17 RX
           choice here dated from before the role mapping was pinned down
           (2026-07-19, when the whole fallback Phase 3 was desynced). */
        span_log(s->logging, SPAN_LOG_FLOW,
                 "Rx - V.90: INFO1a declined PCM (downstream code=%d, not 6); "
                 "V.34 fallback, we take the call-modem role (RX descrambler GPA/tap 4, TX GPC/tap 17)\n",
                 s->info1a.baud_rate_c_to_a);
        s->scrambler_tap = 4;
        s->mp_phase4_default_scrambler_tap = 4;
        owner->tx.scrambler_tap = 17;
        owner->tx.v90_v34_fallback = true;
    }
    /*endif*/
    v34_force_phase3_rx(owner);

    if (!phase3_rx_dump_fp)
    {
        phase3_rx_dump_fp = fopen("/tmp/v90_phase3_rx.raw", "wb");
        phase3_rx_dump_count = 0;
        if (phase3_rx_dump_fp)
            fprintf(stderr, "[V34 RX] Phase 3 RX audio dump started -> /tmp/v90_phase3_rx.raw\n");
        /*endif*/
    }
    /*endif*/
}
/*- End of function --------------------------------------------------------*/

static int process_rx_infoh(v34_rx_state_t *s, infoh_t *infoh, uint8_t buf[])
{
    bitstream_state_t bs;
    const uint8_t *t;

    memset(infoh, 0, sizeof(*infoh));
    bitstream_init(&bs, true);
    t = buf;
    /* 12:14    Power reduction requested by the recipient modem receiver. An integer between 0 and 7
                gives the requested power reduction in dB. These bits shall indicate 0 if the source
                modem's INFO0 indicated that the source modem transmitter cannot reduce its power. */
    infoh->power_reduction = bitstream_get(&bs, &t, 3);
    /* 15:21    Length of TRN to be transmitted by the source modem during Phase 3. An integer between
                0 and 127 gives the length of this sequence in 35 ms increments. */
    infoh->length_of_trn = bitstream_get(&bs, &t, 7);
    /* 22       Set to 1 indicates the high carrier frequency is to be used in data mode transmission. This
                must be consistent with the capabilities indicated in the source modem's INFO0. */
    infoh->use_high_carrier = bitstream_get(&bs, &t, 1);
    /* 23:26    Pre-emphasis filter to be used in transmitting from the source modem to the recipient modem.
                These bits form an integer between 0 and 10 which represents the pre-emphasis filter index
                (see Tables 3 and 4). */
    infoh->preemphasis_filter = bitstream_get(&bs, &t, 4);
    /* 27:29    Symbol rate to be used for data transmission. An integer between 0 and 5 gives the symbol rate, where 0
                represents 2400 and a 5 represents 3429. */
    infoh->baud_rate = bitstream_get(&bs, &t, 3);
    /* 30       Set to 1 indicates TRN uses a 16-point constellation, 0 indicates TRN uses a 4-point constellation. */
    infoh->trn16 = bitstream_get(&bs, &t, 1);

    log_infoh(s->logging, false, infoh);
    return 0;
}
/*- End of function --------------------------------------------------------*/

static int process_rx_mp(v34_rx_state_t *s, mp_t *mp, uint8_t buf[])
{
    int i;
    const uint8_t *t;
    bitstream_state_t bs;

    bitstream_init(&bs, true);
    t = buf;
    /* 18       Type */
    mp->type = bitstream_get(&bs, &t, 1);
    /* 19       Reserved by the ITU */
    bitstream_get(&bs, &t, 1);
    /* 20:23    Maximum call modem to answer modem data signalling rate: Data rate = N * 2400
                where N is a four-bit integer between 1 and 14. */
    mp->bit_rate_c_to_a = bitstream_get(&bs, &t, 4);
    /* 24:27    Maximum answer modem to call modem data signalling rate: Data rate = N * 2400
                where N is a four-bit integer between 1 and 14. */
    mp->bit_rate_a_to_c = bitstream_get(&bs, &t, 4);
    /* 28       Auxiliary channel select bit. Set to 1 if modem is capable of supporting and
                enables auxiliary channel. Auxiliary channel is used only if both modems set
                this bit to 1. */
    mp->aux_channel_supported = bitstream_get(&bs, &t, 1);
    /* 29:30    Trellis encoder select bits:
                0 = 16 state; 1 = 32 state; 2 = 64 state; 3 = Reserved for ITU-T.
                Receiver requires remote-end transmitter to use selected trellis encoder. */
    mp->trellis_size = bitstream_get(&bs, &t, 2);
    /* 31       Non-linear encoder parameter select bit for the remote-end transmitter.
                0: Q = 0, 1: Q = 0.3125. */
    mp->use_non_linear_encoder = bitstream_get(&bs, &t, 1);
    /* 32       Constellation shaping select bit for the remote-end transmitter. 0: minimum,
                1: expanded (see Table 10). */
    mp->expanded_shaping = bitstream_get(&bs, &t, 1);
    /* 33       Acknowledge bit. 0 = modem has not received MP from far end. 1 = received MP from far end. */
    mp->mp_acknowledged = bitstream_get(&bs, &t, 1);
    /* 34       Start bit: 0. */
    bitstream_get(&bs, &t, 1);
    /* 35:49    Data signalling rate capability mask.
                Bit 35:2400; bit 36:4800; bit 37:7200;...; bit 46:28800; bit 47:31200; bit 48:33600;
                bit 49: Reserved for ITU-T. (This bit is set to 0 by the transmitting modem and is not
                interpreted by the receiving modem.) Bits set to 1 indicate data signalling rates supported
                and enabled in both transmitter and receiver of modem. */
    mp->signalling_rate_mask = bitstream_get(&bs, &t, 15);
    /* 50       Asymmetric data signalling rate enable. 1 indicates a modem capable of
                asymmetric data signalling rates. */
    mp->asymmetric_rates_allowed = bitstream_get(&bs, &t, 1);
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
            bitstream_get(&bs, &t, 1);
            mp->precoder_coeffs[i].re = bitstream_get(&bs, &t, 16);
            bitstream_get(&bs, &t, 1);
            mp->precoder_coeffs[i].im = bitstream_get(&bs, &t, 16);
        }
        /*endfor*/
    }
    else
    {
        /* The following are not included in an MP0 message */
        for (i = 0;  i < 3;  i++)
        {
            mp->precoder_coeffs[i].re = 0;
            mp->precoder_coeffs[i].im = 0;
        }
        /*endfor*/
    }
    /*endif*/
    /* We can ignore the remaining bits. They are not used. */

    log_mp(s->logging, false, mp);
    return 0;
}
/*- End of function --------------------------------------------------------*/

static int process_rx_mph(v34_rx_state_t *s, mph_t *mph, uint8_t buf[])
{
    int i;
    const uint8_t *t;
    bitstream_state_t bs;

    bitstream_init(&bs, true);
    t = buf;
    /* 18       Type */
    mph->type = bitstream_get(&bs, &t, 1);
    /* 19       Reserved by the ITU */
    bitstream_get(&bs, &t, 1);
    /* 20:23    Maximum data signalling rate:
                Data rate = N * 2400 where N is a 4-bit integer between 1 and 14. */
    mph->max_data_rate = bitstream_get(&bs, &t, 4);
    /* 24:26    Reserved for ITU-T: These bits are set to 0 by the transmitting modem and are
                not interpreted by the receiving modem. */
    bitstream_get(&bs, &t, 3);
    /* 27       Control channel data signalling rate selected for remote transmitter.
                0 = 1200 bit/s, 1 = 2400 bit/s (see bit 50 below). */
    mph->control_channel_2400 = bitstream_get(&bs, &t, 1);
    /* 28       Reserved for ITU-T: This bit is set to 0 by the transmitting modem and is not
                interpreted by the receiving modem. */
    bitstream_get(&bs, &t, 1);
    /* 29:30    Trellis encoder select bits:
                0 = 16 state; 1 = 32 state; 2 = 64 state; 3 = Reserved for ITU-T.
                Receiver requires remote-end transmitter to use selected trellis encoder. */
    mph->trellis_size = bitstream_get(&bs, &t, 2);
    /* 31       Non-linear encoder parameter select bit for the remote-end transmitter.
                0: Q = 0, 1: Q = 0.3125. */
    mph->use_non_linear_encoder = bitstream_get(&bs, &t, 1);
    /* 32       Constellation shaping select bit for the remote-end transmitter.
                0: minimum, 1: expanded (see Table 10). */
    mph->expanded_shaping = bitstream_get(&bs, &t, 1);
    /* 33       Reserved for ITU-T: This bit is set to 0 by the transmitting modem and is not
                interpreted by the receiving modem. */
    /* 34       Start bit: 0. */
    bitstream_get(&bs, &t, 2);
    /* 35:49    Data signalling rate capability mask.
                Bit 35:2400; bit 36:4800; bit 37:7200;...; bit 46:28800; bit 47:31200; bit 48:33600;
                bit 49: Reserved for ITU-T. (This bit is set to 0 by the transmitting modem and is not
                interpreted by the receiving modem.) Bits set to 1 indicate data signalling rates supported
                and enabled in both transmitter and receiver of modem. */
    mph->signalling_rate_mask = bitstream_get(&bs, &t, 15);
    /* 50       Enables asymmetric control channel data rates:
                0 = Asymmetric mode not allowed; 1 = Asymmetric mode allowed.
                Asymmetric mode shall be used only when both modems set bit 50 to 1. If different data rates are selected
                in symmetric mode, both modems shall transmit at the lower rate. */
    mph->asymmetric_rates_allowed = bitstream_get(&bs, &t, 1);
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
            bitstream_get(&bs, &t, 1);
            mph->precoder_coeffs[i].re = bitstream_get(&bs, &t, 16);
            bitstream_get(&bs, &t, 1);
            mph->precoder_coeffs[i].im = bitstream_get(&bs, &t, 16);
        }
        /*endfor*/
    }
    else
    {
        for (i = 0;  i < 3;  i++)
        {
            mph->precoder_coeffs[i].re = 0;
            mph->precoder_coeffs[i].im = 0;
        }
        /*endfor*/
    }
    /*endif*/
    /* We can ignore the remaining bits. They are not used. */
    log_mph(s->logging, false, mph);
    return 0;
}
/*- End of function --------------------------------------------------------*/

static void info_unpack_bits(uint8_t bits[], int nbits, const uint8_t buf[])
{
    int i;

    for (i = 0;  i < nbits;  i++)
    {
        uint8_t octet;

        octet = bit_reverse8(buf[i >> 3]);
        bits[i] = (octet >> (7 - (i & 7))) & 1;
    }
    /*endfor*/
}
/*- End of function --------------------------------------------------------*/

static void info_pack_bits(uint8_t buf[25], const uint8_t bits[], int nbits)
{
    int i;

    memset(buf, 0, 25);
    for (i = 0;  i < nbits;  i++)
    {
        if (bits[i])
            buf[i >> 3] |= (1 << (i & 7));
        /*endif*/
    }
    /*endfor*/
}
/*- End of function --------------------------------------------------------*/

static uint16_t info_crc_from_bits(const uint8_t bits[], int nbits)
{
    uint16_t crc;
    int i;

    crc = 0xFFFF;
    for (i = 0;  i < nbits;  i++)
        crc = crc_itu16_bits(bits[i], 1, crc);
    /*endfor*/
    return crc;
}
/*- End of function --------------------------------------------------------*/

static void info_bits_to_str(char out[], size_t out_len, const uint8_t bits[], int nbits)
{
    int i;
    size_t pos;

    if (out_len == 0)
        return;
    /*endif*/
    pos = 0;
    for (i = 0;  i < nbits  &&  pos + 1 < out_len;  i++)
        out[pos++] = bits[i] ? '1' : '0';
    /*endfor*/
    out[pos] = '\0';
}
/*- End of function --------------------------------------------------------*/

static void info_log_candidate_diag(v34_rx_state_t *s, const uint8_t in[25], int nbits, uint16_t crc)
{
    uint8_t bits[80];
    char prefix[33];
    char suffix[33];
    char full_bits[81];
    uint16_t shift_crc[5];
    int shift;
    int i;

    if (nbits > (int) (sizeof(bits)/sizeof(bits[0])))
        return;
    /*endif*/

    info_unpack_bits(bits, nbits, in);
    memset(shift_crc, 0, sizeof(shift_crc));
    for (shift = -2;  shift <= 2;  shift++)
    {
        uint8_t trial[80];

        if (shift == 0)
        {
            shift_crc[shift + 2] = crc;
            continue;
        }
        /*endif*/
        for (i = 0;  i < nbits;  i++)
        {
            int src;

            src = i + shift;
            trial[i] = (src >= 0  &&  src < nbits)  ?  bits[src]  :  0;
        }
        /*endfor*/
        shift_crc[shift + 2] = info_crc_from_bits(trial, nbits);
    }
    /*endfor*/

    frame_bits_to_str(bits, 0, (nbits < 32) ? nbits : 32, prefix);
    if (nbits > 32)
        frame_bits_to_str(bits, nbits - 32, 32, suffix);
    else
        suffix[0] = '\0';
    /*endif*/
    info_bits_to_str(full_bits, sizeof(full_bits), bits, nbits);

    span_log(s->logging, SPAN_LOG_FLOW,
             "Rx - info candidate diag: stage=%s bits=%d crc=0x%04x sig=%d pwr=%" PRId32 " peak=%" PRId32 " bit_count=%d duration=%d\n",
             v34_rx_stage_to_str(s->stage),
             nbits,
             crc,
             s->signal_present,
             s->last_info_rx_power,
             s->last_info_rx_power_peak,
             s->bit_count,
             s->duration);
    if (nbits <= 64)
    {
        span_log(s->logging, SPAN_LOG_FLOW,
                 "Rx - info candidate bits=%s\n",
                 full_bits);
    }
    else
    {
        span_log(s->logging, SPAN_LOG_FLOW,
                 "Rx - info candidate prefix=%s suffix=%s\n",
                 prefix,
                 suffix);
    }
    /*endif*/
    span_log(s->logging, SPAN_LOG_FLOW,
             "Rx - info candidate shift CRCs: -2=0x%04x -1=0x%04x 0=0x%04x +1=0x%04x +2=0x%04x\n",
             shift_crc[0], shift_crc[1], shift_crc[2], shift_crc[3], shift_crc[4]);
}
/*- End of function --------------------------------------------------------*/

static bool try_info_boundary_recovery(uint8_t out[25], const uint8_t in[25], int nbits, int *shift_out, uint16_t *crc_out)
{
    uint8_t bits[80];
    uint8_t trial[80];
    int shift;
    int i;

    if (nbits > (int) (sizeof(bits)/sizeof(bits[0])))
        return false;
    /*endif*/
    info_unpack_bits(bits, nbits, in);
    for (shift = -2;  shift <= 2;  shift++)
    {
        uint16_t crc;

        if (shift == 0)
            continue;
        /*endif*/
        for (i = 0;  i < nbits;  i++)
        {
            int src;

            src = i + shift;
            trial[i] = (src >= 0  &&  src < nbits)  ?  bits[src]  :  0;
        }
        /*endfor*/
        crc = info_crc_from_bits(trial, nbits);
        if (crc == 0)
        {
            info_pack_bits(out, trial, nbits);
            if (shift_out)
                *shift_out = shift;
            /*endif*/
            if (crc_out)
                *crc_out = crc;
            /*endif*/
            return true;
        }
        /*endif*/
    }
    /*endfor*/
    return false;
}
/*- End of function --------------------------------------------------------*/

static bool try_info_local_slip_recovery(uint8_t out[25], const uint8_t in[25], int nbits, int *pivot_out, int *shift_out)
{
    uint8_t bits[80];
    uint8_t trial[80];
    int pivot;
    int shift;
    int i;

    if (nbits > (int) (sizeof(bits)/sizeof(bits[0])))
        return false;
    /*endif*/
    info_unpack_bits(bits, nbits, in);
    for (pivot = 4;  pivot < nbits - 4;  pivot++)
    {
        for (shift = -1;  shift <= 1;  shift += 2)
        {
            uint16_t crc;

            for (i = 0;  i < nbits;  i++)
            {
                int src;

                if (i < pivot)
                {
                    src = i;
                }
                else
                {
                    src = i + shift;
                }
                /*endif*/
                trial[i] = (src >= 0  &&  src < nbits)  ?  bits[src]  :  0;
            }
            /*endfor*/
            crc = info_crc_from_bits(trial, nbits);
            if (crc == 0)
            {
                info_pack_bits(out, trial, nbits);
                if (pivot_out)
                    *pivot_out = pivot;
                /*endif*/
                if (shift_out)
                    *shift_out = shift;
                /*endif*/
                return true;
            }
            /*endif*/
        }
        /*endfor*/
    }
    /*endfor*/
    return false;
}
/*- End of function --------------------------------------------------------*/

static bool info_has_valid_prefix_crc(const uint8_t in[25], int total_bits, int prefix_bits, uint16_t *prefix_crc_out)
{
    uint8_t bits[80];
    uint16_t prefix_crc;

    if (total_bits > (int) (sizeof(bits)/sizeof(bits[0])) || prefix_bits > total_bits)
        return false;
    /*endif*/
    info_unpack_bits(bits, total_bits, in);
    prefix_crc = info_crc_from_bits(bits, prefix_bits);
    if (prefix_crc_out)
        *prefix_crc_out = prefix_crc;
    /*endif*/
    return prefix_crc == 0;
}
/*- End of function --------------------------------------------------------*/

static int put_info_bit_count = 0;

/* V34_RX_STAGE_TONE_A's bit-persistence counters (below) drive Tone A /
   reversal detection purely off bit polarity, gated only by the generic
   signal_present hysteresis — which is tuned to catch weak-but-real
   carriers and so also passes ordinary line noise. On a real analog line
   with an old/noisy modem, background noise sits a consistent order of
   magnitude below a genuine Tone A carrier (observed ~4-5M vs ~14-25M in
   this codebase's own power units on a noisy line), but is still loud
   enough to occasionally produce a run of same-polarity bit decisions by
   chance, falsely tripping "Tone A detected"/reversal events. This
   threshold requires real carrier-level power, not just "louder than the
   off/on hysteresis", before letting persistence2 accumulate at all. */
/* Absolute override for the Tone A carrier gate.  Returns 0 when unset, which
   selects the adaptive SNR gate in tone_a_carrier_present() below. */
static int32_t tone_a_min_power_override(void)
{
    static int initialized = 0;
    static int32_t threshold = 0;

    if (!initialized)
    {
        const char *value = getenv("V34_TONE_A_MIN_POWER");
        if (value  &&  value[0] != '\0')
        {
            char *end = NULL;
            long parsed = strtol(value, &end, 10);
            if (end != value  &&  end  &&  *end == '\0'  &&  parsed > 0)
                threshold = (int32_t) parsed;
            /*endif*/
        }
        /*endif*/
        initialized = 1;
    }
    /*endif*/
    return threshold;
}
/*- End of function --------------------------------------------------------*/

/* How far below the measured carrier reference still counts as a real
   carrier.  The original absolute gate sat at 13000000 against a peer whose
   real carrier measured ~185000000 -- a ratio of ~14 -- so 8 reproduces that
   intent while being slightly more permissive. */
static int32_t tone_a_carrier_divisor(void)
{
    static int initialized = 0;
    static int32_t divisor = 8;         /* ~9 dB below the carrier reference */

    if (!initialized)
    {
        const char *value = getenv("V34_TONE_A_CARRIER_DIVISOR");
        if (value  &&  value[0] != '\0')
        {
            char *end = NULL;
            long parsed = strtol(value, &end, 10);
            if (end != value  &&  end  &&  *end == '\0'  &&  parsed > 1)
                divisor = (int32_t) parsed;
            /*endif*/
        }
        /*endif*/
        initialized = 1;
    }
    /*endif*/
    return divisor;
}
/*- End of function --------------------------------------------------------*/

/* Is info_rx() currently looking at a real carrier rather than line noise?
 *
 * This used to be an absolute power threshold (13000000), calibrated against
 * one hardware modem.  That cannot generalise: the received level depends on
 * the peer's transmit power, the FXS gateway, and every digital pad in the
 * SIP path, none of which we control.  Worse, it was calibrated to within
 * about 1 dB of the signal it had to pass, so it was never a threshold so
 * much as a coin toss.  Measured on the d-modem rig, peak received power over
 * a 0.5 s window was 16411569 on 2026-07-17 (clears 13000000 by 1.0 dB) and
 * 12664417 on 2026-07-20 (misses it by 0.1 dB) -- ordinary run-to-run
 * variation of about 2 dB, no protocol change at all.
 *
 * On the losing side of that coin toss the failure is total, not degraded:
 * persistence2 is reset on every sample, Tone A is never declared, the third
 * reversal never arrives, and the receiver never advances to
 * V34_RX_STAGE_INFO1A.  The peer sends INFO1a perfectly well; we are
 * structurally unable to go and listen for it, and every call dies at
 * "aborting after 6 INFO1a timeouts".
 *
 * Gate relative to a measured carrier reference instead, so the same relative
 * discrimination applies at any absolute level.  V34_TONE_A_MIN_POWER still
 * forces the old absolute behaviour if a specific peer ever needs it. */
static int tone_a_carrier_present(v34_rx_state_t *s)
{
    int32_t absolute;
    int32_t reference;

    absolute = tone_a_min_power_override();
    if (absolute > 0)
        return s->last_info_rx_power >= absolute;
    /*endif*/
    reference = s->info_rx_carrier_ref;
    if (reference <= 0)
    {
        /* Nothing measured yet (first window of the call).  Fail open -- the
           coarse signal_present hysteresis still applies, and refusing
           everything here is exactly the failure mode being fixed. */
        return true;
    }
    /*endif*/
    return s->last_info_rx_power >= reference/tone_a_carrier_divisor();
}
/*- End of function --------------------------------------------------------*/

static int info0_target_bits(v34_rx_state_t *s)
{
    /* Payload+CRC length of an INFO0 sequence, excluding the sync code, the
       fill bits and the postamble.  Matches the value v34_rx_restart() picks
       when the receiver is first conditioned for INFO0. */
    return (s->duplex)  ?  (49 - (4 + 8 + 4))  :  (51 - (4 + 8 + 4));
}
/*- End of function --------------------------------------------------------*/

static void put_info_bit(v34_rx_state_t *s, int bit, int time_offset)
{
    int info_search_enabled;

    /* Put info0, info1, tone A or tone B bits */
    s->bitstream = (s->bitstream << 1) | bit;
    if (++put_info_bit_count % 600 == 0)
        span_log(s->logging, SPAN_LOG_FLOW, "Rx - info_rx bits=%d bitstream=0x%03x stage=%d\n",
                 put_info_bit_count, (int)(s->bitstream & 0xFFF), s->stage);
    /* Log only sync code matches and CRC results (verbose bit logging removed) */
    switch (s->stage)
    {
    case V34_RX_STAGE_TONE_A:
        /* Calling side */
        if (++s->persistence1 < 10)
            break;
        /*endif*/
        if (!tone_a_carrier_present(s))
        {
            /* Not a real carrier — just line noise clearing the coarser
               signal_present hysteresis. Don't let it accumulate toward a
               false Tone A / reversal declaration. */
            s->persistence2 = 0;
            break;
        }
        /*endif*/
        if (bit == 0)
        {
            if (++s->persistence2 == 20)
            {
                span_log(s->logging, SPAN_LOG_FLOW, "Rx - Tone A detected\n");
                /* Only set TONE_SEEN if we haven't already seen a reversal —
                   otherwise we'd overwrite REVERSAL_1 and the next reversal
                   would be misidentified as the first instead of the second. */
                if (s->received_event != V34_EVENT_REVERSAL_1
                    && s->received_event != V34_EVENT_REVERSAL_2
                    && s->received_event != V34_EVENT_L2_SEEN)
                {
                    s->received_event = V34_EVENT_TONE_SEEN;
                }
            }
            /*endif*/
            break;
        }
        /*endif*/
        if (!s->signal_present)
            s->persistence2 = 0;
        /*endif*/
        /* We have a reversal, but we should only recognise it if it has been
           a little while since the last one.

           Progress is counted in phase2_reversal_count, which only this
           detector writes. It used to be inferred from received_event, but
           v34tx.c clears that field in 39 places as it consumes events, so a
           reversal sequence could silently reset between reversals: live
           captures show runs with 10 "reversal 1" and 2 "reversal 2" that
           never reach the third and therefore never advance to INFO1A, which
           is the "aborting after 6 INFO1a timeouts" failure. received_event is
           still published for consumers -- it is just no longer the memory. */
        if (s->persistence2 > 20)
        {
            /* L2 having been seen means Phase 2 is already past the first two
               reversals, whatever the counter says. */
            if (s->received_event == V34_EVENT_L2_SEEN  &&  s->phase2_reversal_count < 2)
                s->phase2_reversal_count = 2;
            /*endif*/
            if (s->phase2_reversal_count < 3)
                s->phase2_reversal_count++;
            /*endif*/
            s->tone_ab_hop_time = s->sample_time + time_offset;
            switch (s->phase2_reversal_count)
            {
            case 1:
                span_log(s->logging, SPAN_LOG_FLOW, "Rx - reversal 1 in tone A\n");
                s->received_event = V34_EVENT_REVERSAL_1;
                break;
            case 2:
                span_log(s->logging, SPAN_LOG_FLOW, "Rx - reversal 2 in tone A\n");
                s->received_event = V34_EVENT_REVERSAL_2;
                l1_l2_analysis_init(s);
                break;
            default:
                span_log(s->logging, SPAN_LOG_FLOW, "Rx - reversal 3 in tone A\n");
                s->received_event = V34_EVENT_REVERSAL_3;
                if (s->v90_mode && s->calling_party)
                {
                    /* V.90 caller expects INFO1d (109 bits, same format as INFO1c)
                       from the digital answerer. */
                    s->target_bits = 109 - (4 + 8 + 4);
                    s->stage = V34_RX_STAGE_INFO1C;
                    span_log(s->logging, SPAN_LOG_FLOW,
                             "Rx - V.90 caller: expecting INFO1d (109 bits) from digital answerer\n");
                }
                else
                {
                    /* Standard V.34: next info message will be INFO1a */
                    s->target_bits = 70 - (4 + 8 + 4);
                    s->stage = V34_RX_STAGE_INFO1A;
                }
                /*endif*/
                break;
            }
            /*endswitch*/
            s->persistence1 = 0;
        }
        /*endif*/
        s->persistence2 = 0;
        break;
    case V34_RX_STAGE_TONE_B:
        /* Answering side */
        if (++s->persistence1 < 10)
            break;
        /*endif*/
        if (bit == 0)
        {
            if (++s->persistence2 == 20)
            {
                //s->received_event = V34_EVENT_TONE_SEEN;
            }
            /*endif*/
            break;
        }
        /*endif*/
        if (!s->signal_present)
            s->persistence2 = 0;
        /*endif*/
        /* We have a reversal, but we should only recognise it if it has been
           a little while since the last one */
        if (s->persistence2 > 20)
        {
            switch (s->received_event)
            {
            case V34_EVENT_REVERSAL_2:
                span_log(s->logging, SPAN_LOG_FLOW, "Rx - reversal 3 in tone B\n");
                s->tone_ab_hop_time = s->sample_time + time_offset;
                s->received_event = V34_EVENT_REVERSAL_3;
                break;
            case V34_EVENT_REVERSAL_1:
                /* TODO: Need to avoid getting here falsely, just because the tone has resumed */
                span_log(s->logging, SPAN_LOG_FLOW, "Rx - reversal 2 in tone B\n");
                s->tone_ab_hop_time = s->sample_time + time_offset;
                s->received_event = V34_EVENT_REVERSAL_2;
                if (s->v90_mode)
                {
                    /* V.90 §8.2.3.2 Table 10: analog modem sends INFO1a (70 bits),
                       not INFO1c (109 bits) as in standard V.34 */
                    span_log(s->logging, SPAN_LOG_FLOW,
                             "Rx - V.90 mode: expecting INFO1a (70 bits) from analog modem\n");
                    s->target_bits = 70 - (4 + 8 + 4);
                    s->stage = V34_RX_STAGE_INFO1A;
                }
                else
                {
                    /* The next info message will be INFO1c */
                    s->target_bits = 109 - (4 + 8 + 4);
                }
                l1_l2_analysis_init(s);
                break;
            default:
                span_log(s->logging, SPAN_LOG_FLOW, "Rx - reversal 1 in tone B\n");
                s->tone_ab_hop_time = s->sample_time + time_offset;
                s->received_event = V34_EVENT_REVERSAL_1;
                break;
            }
            /*endswitch*/
            s->persistence1 = 0;
        }
        /*endif*/
        s->persistence2 = 0;
        break;
    }
    info_search_enabled = (s->stage == V34_RX_STAGE_INFO0
                           ||
                           s->stage == V34_RX_STAGE_INFOH
                           ||
                           s->stage == V34_RX_STAGE_INFO1A
                           ||
                           s->stage == V34_RX_STAGE_INFO1C
                           ||
                           /* V.90: analog modem may start sending INFO1a during L1/L2
                              analysis — keep searching for sync codes so we don't miss it */
                           (s->v90_mode
                            &&
                            s->stage == V34_RX_STAGE_L1_L2)
                           ||
                           /* V.90 answerer: once INFO1d has been sent, keep INFO1a
                              sync search active across all Tone A/B event states so
                              we can lock immediately after the 10 ms A-bar period. */
                           (s->v90_mode
                            && !s->calling_party
                            && s->v90_info1d_sent
                            && (s->stage == V34_RX_STAGE_TONE_A
                                || s->stage == V34_RX_STAGE_TONE_B
                                || s->stage == V34_RX_STAGE_L1_L2
                                || s->stage == V34_RX_STAGE_CC))
                           ||
                           ((s->stage == V34_RX_STAGE_TONE_A
                             ||
                             s->stage == V34_RX_STAGE_TONE_B)
                            &&
                            (s->received_event == V34_EVENT_NONE
                             ||
                             s->received_event == V34_EVENT_INFO0_BAD
                             ||
                             s->received_event == V34_EVENT_TONE_SEEN)));
    if (s->v90_mode
        && !s->calling_party
        && s->v90_info1d_sent
        && s->info0_received
        && info_search_enabled
        && s->target_bits != (70 - (4 + 8 + 4)))
    {
        /* Bias framing toward INFO1a in the post-INFO1d window, even when
           we are still traversing Tone A/B transitions. */
        s->target_bits = 70 - (4 + 8 + 4);
    }
    /* Search for INFO0, INFOh, INFO1a or INFO1c messages. */
    if (!info_search_enabled)
    {
        s->bit_count = 0;
    }
    else if (s->bit_count == 0)
    {
        /* Look for info message sync code */
        if ((s->bitstream & 0x3FF) == 0x372)
        {
            span_log(s->logging, SPAN_LOG_FLOW, "Rx - info sync code detected\n");
            s->crc = 0xFFFF;
            s->bit_count = 1;
        }
        /*endif*/
    }
    else
    {
        /* Every 8 bits save the resulting byte */
        if ((s->bit_count & 0x07) == 0)
            s->info_buf[(s->bit_count >> 3) - 1] = bit_reverse8(s->bitstream & 0xFF);
        /*endif*/
        s->crc = crc_itu16_bits(bit, 1, s->crc);
        /* V.34/11.2.2.1.1: while we sit in the INFO1c wait the call modem may
           instead be repeatedly sending INFO0c, which is shorter than INFO1c.
           Without this the repeats are read as malformed 93 bit frames (the
           first bytes match the genuine INFO0c, then we run off the end and
           the CRC always fails), the recovery is never noticed, and both
           sides wait for each other until the peer's train timeout. Test the
           CRC as we pass the INFO0 length so a repeat is recognised. */
        if (!s->v90_mode
            &&  !s->calling_party
            &&  s->stage == V34_RX_STAGE_INFO1C
            &&  s->bit_count == info0_target_bits(s)
            &&  s->crc == 0)
        {
            int tail = s->bit_count & 0x07;

            if (tail != 0)
                s->info_buf[(s->bit_count >> 3)] = bit_reverse8(s->bitstream & 0xFF);
            /*endif*/
            span_log(s->logging, SPAN_LOG_FLOW,
                     "Rx - repeated INFO0c during INFO1c wait (11.2.2.1.1 recovery)\n");
            process_rx_info0(s, s->info_buf);
            s->received_event = V34_EVENT_INFO0_OK;
            s->bit_count = 0;
            return;
        }
        /*endif*/
        if (s->bit_count++ == s->target_bits)
        {
            /* Flush any remaining bits in the last partial byte into info_buf.
               The byte-write at line 3248 only fires when bit_count is a multiple
               of 8, so the tail bits (target_bits % 8 != 0) are lost. Write the
               partial byte now so info_buf and CRC are consistent. */
            {
                int tail = s->target_bits & 0x07;
                if (tail != 0)
                    s->info_buf[(s->target_bits >> 3)] = bit_reverse8(s->bitstream & 0xFF);
            }
            span_log(s->logging, SPAN_LOG_FLOW, "Rx - info CRC result 0x%x (target_bits=%d)\n", s->crc, s->target_bits);
            {
                int nbytes = (s->target_bits + 7) / 8;
                if (nbytes > 25) nbytes = 25;
                char hexbuf[80];
                int hoff = 0;
                for (int hh = 0; hh < nbytes; hh++)
                    hoff += snprintf(hexbuf + hoff, sizeof(hexbuf) - hoff, " %02x", s->info_buf[hh]);
                span_log(s->logging, SPAN_LOG_FLOW, "Rx - info raw bytes:%s\n", hexbuf);
            }
            if (s->crc == 0)
            {
                int v90_info1a_search;

                v90_info1a_search = (s->v90_mode
                                     && !s->calling_party
                                     && s->target_bits == (70 - (4 + 8 + 4))
                                     && (s->stage == V34_RX_STAGE_TONE_A
                                         || s->stage == V34_RX_STAGE_TONE_B
                                         || s->stage == V34_RX_STAGE_L1_L2
                                         || s->stage == V34_RX_STAGE_CC
                                         || s->stage == V34_RX_STAGE_INFO1A));
                switch (s->stage)
                {
                case V34_RX_STAGE_TONE_A:
                case V34_RX_STAGE_TONE_B:
                case V34_RX_STAGE_L1_L2:
                case V34_RX_STAGE_CC:
                case V34_RX_STAGE_INFO0:
                    if (v90_info1a_search)
                    {
                        process_rx_info1a(s, &s->info1a, s->info_buf);
                        if (s->v90_mode)
                        {
                            span_log(s->logging, SPAN_LOG_FLOW,
                                     "Rx - V.90: INFO1a received, switching to Phase 3 primary channel RX\n");
                            v90_enter_phase3_from_info1a(s);
                        }
                        /* v34_force_phase3_rx() has already consumed the
                           INFO1a transition on the TX side.  Do not leave a
                           stale INFO1_OK event blocking Phase 3 J detection. */
                        s->received_event = V34_EVENT_NONE;
                    }
                    else
                    {
                        int preserve_tone_a_recovery;
                        int first_info0;

                        preserve_tone_a_recovery = (s->v90_mode
                                                    && !s->calling_party
                                                    && s->stage == V34_RX_STAGE_TONE_A);
                        first_info0 = !s->info0_received;
                        process_rx_info0(s, s->info_buf);
                        if (preserve_tone_a_recovery)
                        {
                            /* V.90 answerer recovery: after receiving INFO0a and
                               conditioning RX for Tone A, keep looking for Tone A
                               instead of falling back to Tone B on every repeated
                               INFO0a.  A repeated INFO0a is not a new Phase 2
                               transaction: do not erase a Tone A reversal that
                               may have arrived in the same media block. */
                            s->stage = V34_RX_STAGE_TONE_A;
                            if (first_info0)
                                s->phase2_reversal_count = 0;
                            /*endif*/
                            s->persistence1 = 0;
                            s->persistence2 = 0;
                        }
                        else
                        {
                            s->stage = (s->calling_party)  ?   V34_RX_STAGE_TONE_A  :  V34_RX_STAGE_TONE_B;
                        }
                        /* Only set INFO0_OK on the first reception. Repeated
                           INFO0a decodes during V.90 FIRST_B_SILENCE would overwrite
                           REVERSAL_1 events from the tone handler, preventing the
                           TX from detecting the second Tone A reversal. Track them
                           via a sticky flag instead so the TX state machine can
                           choose whether to recover Phase 2. */
                        if (!s->info0_received)
                            s->received_event = V34_EVENT_INFO0_OK;
                        else if (s->v90_mode
                                 && !s->calling_party
                                 && (s->stage == V34_RX_STAGE_TONE_A
                                     || s->stage == V34_RX_STAGE_TONE_B
                                     || s->stage == V34_RX_STAGE_INFO0))
                            s->v90_repeated_info0a_pending = true;
                        s->info0_received = true;
                    }
                    break;
                case V34_RX_STAGE_INFOH:
                    process_rx_infoh(s, &s->infoh, s->info_buf);
                    s->received_event = V34_EVENT_INFO1_OK;
                    break;
                case V34_RX_STAGE_INFO1C:
                    process_rx_info1c(s, &s->info1c, s->info_buf);
                    s->received_event = V34_EVENT_INFO1_OK;
                    break;
                case V34_RX_STAGE_INFO1A:
                    process_rx_info1a(s, &s->info1a, s->info_buf);
                    if (s->v90_mode)
                    {
                        /* V.90 §9.2.1.1.8: INFO1a received — now proceed to Phase 3.
                           Switch RX from CC demodulator to primary channel for
                           upstream V.34 reception. */
                        span_log(s->logging, SPAN_LOG_FLOW,
                                 "Rx - V.90: INFO1a received, switching to Phase 3 primary channel RX\n");
                        v90_enter_phase3_from_info1a(s);
                        s->received_event = V34_EVENT_NONE;
                    }
                    else
                    {
                        s->received_event = V34_EVENT_INFO1_OK;
                    }
                    break;
                }
                /*endswitch*/
            }
            else
            {
                int v90_info1a_search;
                uint8_t recovered_info[25];
                int recovery_shift;
                int recovery_pivot;
                uint16_t prefix_crc;

                v90_info1a_search = (s->v90_mode
                                     && !s->calling_party
                                     && s->target_bits == (70 - (4 + 8 + 4))
                                     && (s->stage == V34_RX_STAGE_TONE_A
                                         || s->stage == V34_RX_STAGE_TONE_B
                                         || s->stage == V34_RX_STAGE_L1_L2
                                         || s->stage == V34_RX_STAGE_CC
                                         || s->stage == V34_RX_STAGE_INFO1A));
                if (v90_info1a_search)
                    info_log_candidate_diag(s, s->info_buf, s->target_bits, s->crc);
                /*endif*/
                /* Try INFO1a boundary/slip recovery before the INFO0a prefix check.
                   Once INFO1d has been sent, the normal path is INFO1a, so
                   prioritise INFO1a decoding.  A peer in Phase 2 error recovery,
                   however, deliberately resumes INFO0a at this point to request an
                   acknowledged INFO0d.  Keep the valid-prefix fallback active after
                   INFO1d as well; otherwise the shorter INFO0a is padded with Tone A
                   bits to the INFO1a target length and reported forever as INFO1_BAD. */
                if (v90_info1a_search
                    && try_info_boundary_recovery(recovered_info, s->info_buf, s->target_bits, &recovery_shift, NULL))
                {
                    span_log(s->logging, SPAN_LOG_FLOW,
                             "Rx - INFO1a boundary recovery succeeded with %d-bit shift\n",
                             recovery_shift);
                    process_rx_info1a(s, &s->info1a, recovered_info);
                    span_log(s->logging, SPAN_LOG_FLOW,
                             "Rx - V.90: recovered INFO1a, switching to Phase 3 primary channel RX\n");
                    v90_enter_phase3_from_info1a(s);
                    s->received_event = V34_EVENT_NONE;
                    s->bit_count = 0;
                    return;
                }
                /*endif*/
                if (v90_info1a_search
                    && try_info_local_slip_recovery(recovered_info, s->info_buf, s->target_bits, &recovery_pivot, &recovery_shift))
                {
                    span_log(s->logging, SPAN_LOG_FLOW,
                             "Rx - INFO1a local-slip recovery succeeded at bit %d with suffix shift %d\n",
                             recovery_pivot,
                             recovery_shift);
                    process_rx_info1a(s, &s->info1a, recovered_info);
                    span_log(s->logging, SPAN_LOG_FLOW,
                             "Rx - V.90: recovered INFO1a via local-slip recovery, switching to Phase 3 primary channel RX\n");
                    v90_enter_phase3_from_info1a(s);
                    s->received_event = V34_EVENT_NONE;
                    s->bit_count = 0;
                    return;
                }
                /*endif*/
                if (v90_info1a_search
                    && info_has_valid_prefix_crc(s->info_buf, s->target_bits, 33, &prefix_crc))
                {
                    span_log(s->logging, SPAN_LOG_FLOW,
                             "Rx - INFO1a candidate contains a valid 33-bit INFO0a prefix (crc=0x%04x); treating it as repeated INFO0a, not INFO1a\n",
                             prefix_crc);
                    process_rx_info0(s, s->info_buf);
                    s->v90_repeated_info0a_pending = true;
                    s->received_event = V34_EVENT_INFO0_OK;
                    s->bit_count = 0;
                    return;
                }
                /*endif*/
                /* INFO0 single-bit-error recovery: if the CRC failed with only
                   1 bit wrong, try flipping each bit and recomputing.  This is
                   cheap (max 46 iterations for INFO0d) and dramatically improves
                   INFO0 decode success on recordings with minor DPSK bit errors. */
                if (s->stage == V34_RX_STAGE_INFO0
                    && !v90_info1a_search
                    && !s->info0_received)
                {
                    int nb = s->target_bits;
                    int nbytes_r = (nb + 7) / 8;
                    int recovered_info0 = 0;

                    for (int flip = 0;  flip < nb;  flip++)
                    {
                        uint16_t test_crc = 0xFFFF;
                        int byte_idx = flip >> 3;
                        int bit_idx = flip & 0x07;
                        uint8_t saved = s->info_buf[byte_idx];

                        s->info_buf[byte_idx] ^= (1 << bit_idx);
                        for (int bi = 0;  bi < nbytes_r;  bi++)
                        {
                            int bits_left = nb - bi * 8;
                            if (bits_left > 8)
                                bits_left = 8;
                            test_crc = crc_itu16_bits(s->info_buf[bi], bits_left, test_crc);
                        }
                        if (test_crc == 0)
                        {
                            span_log(s->logging, SPAN_LOG_FLOW,
                                     "Rx - INFO0 single-bit recovery: flipped bit %d, CRC now 0\n",
                                     flip);
                            process_rx_info0(s, s->info_buf);
                            if (!s->info0_received)
                                s->received_event = V34_EVENT_INFO0_OK;
                            s->info0_received = true;
                            recovered_info0 = 1;
                            break;
                        }
                        s->info_buf[byte_idx] = saved;
                    }
                    if (recovered_info0)
                    {
                        s->bit_count = 0;
                        return;
                    }
                }
                /*endif*/
                switch (s->stage)
                {
                case V34_RX_STAGE_TONE_A:
                case V34_RX_STAGE_TONE_B:
                case V34_RX_STAGE_L1_L2:
                case V34_RX_STAGE_CC:
                case V34_RX_STAGE_INFO0:
                    s->received_event = v90_info1a_search ? V34_EVENT_INFO1_BAD : V34_EVENT_INFO0_BAD;
                case V34_RX_STAGE_INFOH:
                    break;
                case V34_RX_STAGE_INFO1C:
                case V34_RX_STAGE_INFO1A:
                    s->received_event = V34_EVENT_INFO1_BAD;
                    break;
                }
                /*endswitch*/
            }
            /*endif*/
            s->bit_count = 0;
        }
        /*endif*/
    }
    /*endif*/
}
/*- End of function --------------------------------------------------------*/

static int info_rx(v34_rx_state_t *s, const int16_t amp[], int len)
{
    int i;
    int step;
    complexf_t z;
    complexf_t zz;
    complexf_t sample;
    float ii;
    float qq;
    uint32_t angle;
    int32_t phase_delta;
    int32_t power;

    s->agc_scaling = 0.01f;
    step = 6;
    for (i = 0;  i < len;  i++)
    {
        power = power_meter_update(&s->power, amp[i]);
        s->last_info_rx_power = power;
        if (s->sample_time - s->last_info_rx_power_peak_reset >= 800)
        {
            s->last_info_rx_power_peak = power;
            s->last_info_rx_power_peak_reset = s->sample_time;
        }
        else if (power > s->last_info_rx_power_peak)
        {
            s->last_info_rx_power_peak = power;
        }
        /*endif*/
        /* Carrier-level reference feeding the Tone A gate: instant attack,
           slow decay (~4096-sample time constant, so roughly -17 dB/s with no
           signal).  Deliberately not windowed against s->sample_time -- the
           V.90 Phase 2 flow reconditions this receiver repeatedly, resetting
           that clock, so a 2 s window never completed and the reference stayed
           at 0.  A minimum tracker is wrong here too: Phase 2 contains genuine
           silent gaps, so a minimum collapses to ~0 and the gate degenerates
           to always-open. */
        if (power > s->info_rx_carrier_ref)
            s->info_rx_carrier_ref = power;
        else
            s->info_rx_carrier_ref -= s->info_rx_carrier_ref >> 12;
        /*endif*/
        if (s->v90_mode
            && !s->calling_party
            && !s->signal_present
            && (s->stage == V34_RX_STAGE_INFO0 || s->stage == V34_RX_STAGE_TONE_A || s->stage == V34_RX_STAGE_TONE_B)
            && (s->duration == 400 || s->duration == 800 || s->duration == 1600 || s->duration == 3200))
        {
            span_log(s->logging, SPAN_LOG_FLOW,
                     "Rx - V.90 Phase 2 waiting for carrier: stage=%d power=%" PRId32 " on=%" PRId32 " off=%" PRId32 "\n",
                     s->stage, power, s->carrier_on_power, s->carrier_off_power);
        }
        if (s->signal_present)
        {
            if (power < s->carrier_off_power)
            {
span_log(s->logging, SPAN_LOG_FLOW, "Signal down\n");
                s->signal_present = false;
                s->persistence2 = 0;
            }
            /*endif*/
        }
        else
        {
            if (power > s->carrier_on_power)
            {
span_log(s->logging, SPAN_LOG_FLOW, "Signal up\n");
                s->signal_present = true;
                s->persistence2 = 0;
                /* Reset phase tracking so stale last_angles don't
                   cause spurious reversals when CC resumes */
                s->last_angles[0] = 0;
                s->last_angles[1] = 0;
                s->blip_duration = 0;
            }
            /*endif*/
        }
        /*endif*/
        s->rrc_filter[s->rrc_filter_step] = amp[i];
        if (++s->rrc_filter_step >= V34_RX_FILTER_STEPS)
            s->rrc_filter_step = 0;
        /*endif*/
        /* Standard V.34: caller RX at 2400 Hz, answerer RX at 1200 Hz.
           V.90 §8.2.3.1: carriers swapped — answerer RX at 2400 Hz, caller RX at 1200 Hz. */
        if (s->calling_party != s->v90_mode)
        {
#if defined(SPANDSP_USE_FIXED_POINT)
            ii = vec_circular_dot_prodi16(s->rrc_filter, rx_pulseshaper_2400_re[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
            qq = vec_circular_dot_prodi16(s->rrc_filter, rx_pulseshaper_2400_im[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
#else
            ii = vec_circular_dot_prodf(s->rrc_filter, rx_pulseshaper_2400_re[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
            qq = vec_circular_dot_prodf(s->rrc_filter, rx_pulseshaper_2400_im[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
#endif
        }
        else
        {
#if defined(SPANDSP_USE_FIXED_POINT)
            ii = vec_circular_dot_prodi16(s->rrc_filter, rx_pulseshaper_1200_re[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
            qq = vec_circular_dot_prodi16(s->rrc_filter, rx_pulseshaper_1200_im[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
#else
            ii = vec_circular_dot_prodf(s->rrc_filter, rx_pulseshaper_1200_re[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
            qq = vec_circular_dot_prodf(s->rrc_filter, rx_pulseshaper_1200_im[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
#endif
        }
        /*endif*/
        sample.re = ii*s->agc_scaling;
        sample.im = qq*s->agc_scaling;
        /* Shift to baseband - since this is done in full complex form, the result is clean. */
        z = dds_lookup_complexf(s->carrier_phase);
        zz.re = sample.re*z.re - sample.im*z.im;
        zz.im = -sample.re*z.im - sample.im*z.re;
        angle = arctan2(zz.im, zz.re);
        phase_delta = (int32_t) (angle - s->last_angles[1]);
        /* V34_DUMP_INFO_RX (diagnostic, 2026-07-19): dumps the info_rx
           demod chain -- pre-AGC matched-filter output, post-mix baseband,
           angle, and phase_delta -- to find why put_info_bit() never sees
           a reversal (bitstream stuck at 0x000) against a real remote
           modem despite signal_present staying true and the raw carrier
           measuring correct in an offline capture. */
        if (getenv("V34_DUMP_INFO_RX")
            && s->stage == V34_RX_STAGE_INFO0 && (s->duration % 400) == 0)
        {
            span_log(s->logging, SPAN_LOG_FLOW,
                     "Rx - info_rx sample: t=%d ii=%.4f qq=%.4f agc=%.5f zz=(%.4f,%.4f) angdeg=%.2f pdeltadeg=%.2f blip=%d\n",
                     s->duration, (double) ii, (double) qq, (double) s->agc_scaling,
                     (double) zz.re, (double) zz.im,
                     (double) (angle * 360.0 / 4294967296.0),
                     (double) (phase_delta * 360.0 / 4294967296.0),
                     s->blip_duration);
        }
        if ((phase_delta > (int32_t) DDS_PHASE(90.0f)
             || phase_delta < -(int32_t) DDS_PHASE(90.0f))
            &&
            s->blip_duration > 3)
        {
            /* Reversal event logging removed (was verbose) */
            put_info_bit(s, 1, i);
            s->duration = 0;
            s->blip_duration = 0;
        }
        else
        {
            if (s->blip_duration > 60)
            {
                /* We are getting rather late for a transition. This must be a zero bit. */
                put_info_bit(s, 0, i);
                /* Step on by one bit time. */
                s->blip_duration -= 40;
            }
            /*endif*/
        }
        /*endif*/
        s->last_angles[1] = s->last_angles[0];
        s->last_angles[0] = angle;
        s->duration++;
        s->blip_duration += 3;
        dds_advancef(&s->carrier_phase, s->cc_carrier_phase_rate);
    }
    /*endfor*/
    /* Periodic diagnostic: log every 8000 samples (~1s) */
    if (V34_DEBUG_INFO_RX_DIAG
        && s->duration % 8000 < (unsigned)len)
    {
        span_log(s->logging, SPAN_LOG_FLOW,
                 "Rx info_rx diag: ref=%d gate=%d carrier=%d "
                 "stage=%d sig=%d pwr=%d bits=%d\n",
                 s->info_rx_carrier_ref,
                 s->info_rx_carrier_ref/tone_a_carrier_divisor(),
                 tone_a_carrier_present(s),
                 s->stage, s->signal_present, power, s->bit_count);
    }
    return 0;
}
/*- End of function --------------------------------------------------------*/

static __inline__ void cc_symbol_sync(v34_rx_state_t *s)
{
    int i;
#if defined(SPANDSP_USE_FIXED_POINT)
    int32_t v;
    int32_t p;
#else
    float v;
    float p;
    const float ted_error_clip = 50.0f;
    const float ted_fine_trigger = 100.0f;
    const float ted_coarse_trigger = 200.0f;
    const float ted_phase_clip = 500.0f;
#endif

    /* This routine adapts the position of the half baud samples entering the equalizer. */

    /* This symbol sync scheme is based on the technique first described by Dominique Godard in
        Passband Timing Recovery in an All-Digital Modem Receiver
        IEEE TRANSACTIONS ON COMMUNICATIONS, VOL. COM-26, NO. 5, MAY 1978 */

    /* This is slightly rearranged from figure 3b of the Godard paper, as this saves a couple of
       maths operations */
#if defined(SPANDSP_USE_FIXED_POINT)
    /* TODO: The scalings used here need more thorough evaluation, to see if overflows are possible. */
    /* Cross correlate */
    v = (((s->cc_ted.symbol_sync_low[1] >> 5)*(s->cc_ted.symbol_sync_high[0] >> 4)) >> 15)*s->cc_ted.low_band_edge_coeff[2]
      - (((s->cc_ted.symbol_sync_low[0] >> 5)*(s->cc_ted.symbol_sync_high[1] >> 4)) >> 15)*s->cc_ted.high_band_edge_coeff[2]
      + (((s->cc_ted.symbol_sync_low[1] >> 5)*(s->cc_ted.symbol_sync_high[1] >> 4)) >> 15)*s->cc_ted.mixed_edges_coeff_3;
    /* Filter away any DC component */
    p = v - s->cc_ted.symbol_sync_dc_filter[1];
    s->cc_ted.symbol_sync_dc_filter[1] = s->cc_ted.symbol_sync_dc_filter[0];
    s->cc_ted.symbol_sync_dc_filter[0] = v;
    /* A little integration will now filter away much of the HF noise */
    s->cc_ted.baud_phase -= p;
    v = abs(s->cc_ted.baud_phase);
    if (v > 100*FP_FACTOR)
    {
        i = (v > 1000*FP_FACTOR)  ?  15  :  1;
        if (s->cc_ted.baud_phase < 0)
            i = -i;
        /*endif*/
        //printf("v = %10.5f %5d - %f %f %d %d\n", v, i, p, s->cc_ted.baud_phase, s->total_baud_timing_correction);
        s->eq_put_step += i;
        s->total_baud_timing_correction += i;
    }
    /*endif*/
#else
    /* Cross correlate */
    v = s->cc_ted.symbol_sync_low[1]*s->cc_ted.symbol_sync_high[0]*s->cc_ted.low_band_edge_coeff[2]
      - s->cc_ted.symbol_sync_low[0]*s->cc_ted.symbol_sync_high[1]*s->cc_ted.high_band_edge_coeff[2]
      + s->cc_ted.symbol_sync_low[1]*s->cc_ted.symbol_sync_high[1]*s->cc_ted.mixed_edges_coeff_3;
    /* Filter away any DC component  */
    p = v - s->cc_ted.symbol_sync_dc_filter[1];
    if (!isfinite(p))
        p = 0.0f;
    else if (p > ted_error_clip)
        p = ted_error_clip;
    else if (p < -ted_error_clip)
        p = -ted_error_clip;
    s->cc_ted.symbol_sync_dc_filter[1] = s->cc_ted.symbol_sync_dc_filter[0];
    s->cc_ted.symbol_sync_dc_filter[0] = v;
    /* A little integration will now filter away much of the HF noise */
    s->cc_ted.baud_phase -= p;
    if (!isfinite(s->cc_ted.baud_phase))
        s->cc_ted.baud_phase = 0.0f;
    else if (s->cc_ted.baud_phase > ted_phase_clip)
        s->cc_ted.baud_phase = ted_phase_clip;
    else if (s->cc_ted.baud_phase < -ted_phase_clip)
        s->cc_ted.baud_phase = -ted_phase_clip;
    v = fabsf(s->cc_ted.baud_phase);
    if (v > ted_fine_trigger)
    {
        i = (v > ted_coarse_trigger)  ?  2  :  1;
        if (s->cc_ted.baud_phase < 0.0f)
            i = -i;
        /*endif*/
        //printf("v = %10.5f %5d - %f %f %d\n", v, i, p, s->cc_ted.baud_phase, s->total_baud_timing_correction);
        s->eq_put_step += i;
        s->total_baud_timing_correction += i;
        /* Anti-windup: consume phase error when applying a timing step. */
        s->cc_ted.baud_phase -= i*ted_fine_trigger;
    }
    /*endif*/
#endif
}
/*- End of function --------------------------------------------------------*/

static __inline__ void pri_symbol_sync(v34_rx_state_t *s)
{
    int i;
#if defined(SPANDSP_USE_FIXED_POINT)
    int32_t v;
    int32_t p;
#else
    float v;
    float p;
    const float ted_error_clip = 50.0f;
    const float ted_fine_trigger = 100.0f;
    const float ted_coarse_trigger = 200.0f;
    const float ted_phase_clip = 500.0f;
#endif

    /* This routine adapts the position of the half baud samples entering the equalizer. */

    /* This symbol sync scheme is based on the technique first described by Dominique Godard in
        Passband Timing Recovery in an All-Digital Modem Receiver
        IEEE TRANSACTIONS ON COMMUNICATIONS, VOL. COM-26, NO. 5, MAY 1978 */

    /* This is slightly rearranged from figure 3b of the Godard paper, as this saves a couple of
       maths operations */
#if defined(SPANDSP_USE_FIXED_POINT)
    /* TODO: The scalings used here need more thorough evaluation, to see if overflows are possible. */
    /* Cross correlate */
    v = (((s->pri_ted.symbol_sync_low[1] >> 5)*(s->pri_ted.symbol_sync_high[0] >> 4)) >> 15)*s->pri_ted.low_band_edge_coeff[2]
      - (((s->pri_ted.symbol_sync_low[0] >> 5)*(s->pri_ted.symbol_sync_high[1] >> 4)) >> 15)*s->pri_ted.high_band_edge_coeff[2]
      + (((s->pri_ted.symbol_sync_low[1] >> 5)*(s->pri_ted.symbol_sync_high[1] >> 4)) >> 15)*s->pri_ted.mixed_edges_coeff_3;
    /* Filter away any DC component */
    p = v - s->pri_ted.symbol_sync_dc_filter[1];
    s->pri_ted.symbol_sync_dc_filter[1] = s->pri_ted.symbol_sync_dc_filter[0];
    s->pri_ted.symbol_sync_dc_filter[0] = v;
    /* A little integration will now filter away much of the HF noise */
    s->pri_ted.baud_phase -= p;
    v = abs(s->pri_ted.baud_phase);
    if (v > 100*FP_FACTOR
        && !(getenv("ME_V34_FREEZE_TIMING_DURING_MP") && phase4_trn_should_freeze_tracking(s)))
    {
        i = (v > 1000*FP_FACTOR)  ?  15  :  1;
        if (s->pri_ted.baud_phase < 0)
            i = -i;
        /*endif*/
        //printf("v = %10.5f %5d - %f %f %d %d\n", v, i, p, s->pri_ted.baud_phase, s->total_baud_timing_correction);
        s->eq_put_step += i;
        s->total_baud_timing_correction += i;
    }
    /*endif*/
#else
    /* Cross correlate */
    v = s->pri_ted.symbol_sync_low[1]*s->pri_ted.symbol_sync_high[0]*s->pri_ted.low_band_edge_coeff[2]
      - s->pri_ted.symbol_sync_low[0]*s->pri_ted.symbol_sync_high[1]*s->pri_ted.high_band_edge_coeff[2]
      + s->pri_ted.symbol_sync_low[1]*s->pri_ted.symbol_sync_high[1]*s->pri_ted.mixed_edges_coeff_3;
    /* Filter away any DC component  */
    p = v - s->pri_ted.symbol_sync_dc_filter[1];
    if (!isfinite(p))
        p = 0.0f;
    else if (p > ted_error_clip)
        p = ted_error_clip;
    else if (p < -ted_error_clip)
        p = -ted_error_clip;
    s->pri_ted.symbol_sync_dc_filter[1] = s->pri_ted.symbol_sync_dc_filter[0];
    s->pri_ted.symbol_sync_dc_filter[0] = v;
    /* A little integration will now filter away much of the HF noise */
    s->pri_ted.baud_phase -= p;
    if (!isfinite(s->pri_ted.baud_phase))
        s->pri_ted.baud_phase = 0.0f;
    else if (s->pri_ted.baud_phase > ted_phase_clip)
        s->pri_ted.baud_phase = ted_phase_clip;
    else if (s->pri_ted.baud_phase < -ted_phase_clip)
        s->pri_ted.baud_phase = -ted_phase_clip;
    v = fabsf(s->pri_ted.baud_phase);
    if (v > ted_fine_trigger
        && !(getenv("ME_V34_FREEZE_TIMING_DURING_MP") && phase4_trn_should_freeze_tracking(s)))
    {
        i = (v > ted_coarse_trigger)  ?  2  :  1;
        if (s->pri_ted.baud_phase < 0.0f)
            i = -i;
        /*endif*/
        /* ME_V34_FREEZE_TIMING_DURING_MP (experimental, 2026-07-19): live
           interop showed the Godard symbol-timing loop, which is NOT frozen
           during Phase 4 MP the way carrier tracking already is
           (phase4_trn_should_freeze_tracking()), holding baud_phase steady
           at exactly 0.0 for the entire preceding TRN period, then firing
           one large, sustained eq_put_step correction right at the TRN->MP
           boundary and settling into a new equilibrium it never recovers
           from. Looked like a plausible cause of MP frames failing CRC with
           errors that get worse later in the frame -- but verified live
           with this flag set (confirmed via V34_TRACE_DIAGNOSTICS that
           eq_put_step genuinely stayed frozen through the transition) that
           the decoded MP frame bits come out byte-for-byte identical to the
           unfrozen case anyway. Ruled out as the cause; left available
           (default off) since it's a real, harmless option and the negative
           result is worth being able to reproduce rather than silently losing
           the finding. See rig/README.md for the fuller elimination list. */
        //printf("v = %10.5f %5d - %f %f %d\n", v, i, p, s->pri_ted.baud_phase, s->total_baud_timing_correction);
        s->eq_put_step += i;
        s->total_baud_timing_correction += i;
        /* Anti-windup: consume phase error when applying a timing step. */
        s->pri_ted.baud_phase -= i*ted_fine_trigger;
    }
    /*endif*/
#endif
    /* Periodic TED + carrier tracking diagnostic (every 256 bauds) */
    if (V34_TRACE_DIAGNOSTICS
        && (s->duration & 0xFF) == 0
        && s->stage >= V34_RX_STAGE_PHASE3_WAIT_S)
    {
        fprintf(stderr, "[V34 RX] baud=%d ted_phase=%.1f ted_corr=%d carrier=%.2fHz eq_step=%d\n",
                s->duration, (double)s->pri_ted.baud_phase, s->total_baud_timing_correction,
                dds_frequencyf(s->v34_carrier_phase_rate), s->eq_put_step);
    }
}
/*- End of function --------------------------------------------------------*/

static void create_godard_coeffs(ted_t *coeffs, float carrier, float baud_rate, float alpha)
{
    float low_edge;
    float high_edge;

    /* Create the coefficient set for an arbitrary Godard TED/symbol sync filter */
    low_edge = 2.0*M_PI*(carrier - baud_rate/2.0)/SAMPLE_RATE;
    high_edge = 2.0*M_PI*(carrier + baud_rate/2.0)/SAMPLE_RATE;

#if defined(SPANDSP_USE_FIXED_POINT)
    coeffs->low_band_edge_coeff[0] = ((int32_t)(FP_FACTOR*(2.0*alpha*cos(low_edge))));
    coeffs->high_band_edge_coeff[0] = ((int32_t)(FP_FACTOR*(2.0*alpha*cos(high_edge))));
    coeffs->low_band_edge_coeff[1] =
    coeffs->high_band_edge_coeff[1] = ((int32_t)(FP_FACTOR*(-alpha*alpha)));
    coeffs->low_band_edge_coeff[2] = ((int32_t)(FP_FACTOR*(-alpha*sin(low_edge))));
    coeffs->high_band_edge_coeff[2] = ((int32_t)(FP_FACTOR*(-alpha*sin(high_edge))));
    coeffs->mixed_edges_coeff_3 = ((int32_t)(FP_FACTOR*(-alpha*alpha*(sin(high_edge)*cos(low_edge) - sin(low_edge)*cos(high_edge)))));
#else
    coeffs->low_band_edge_coeff[0] = 2.0*alpha*cos(low_edge);
    coeffs->high_band_edge_coeff[0] = 2.0*alpha*cos(high_edge);
    coeffs->low_band_edge_coeff[1] =
    coeffs->high_band_edge_coeff[1] = -alpha*alpha;
    coeffs->low_band_edge_coeff[2] = -alpha*sin(low_edge);
    coeffs->high_band_edge_coeff[2] = -alpha*sin(high_edge);
    coeffs->mixed_edges_coeff_3 = -alpha*alpha*(sin(high_edge)*cos(low_edge) - sin(low_edge)*cos(high_edge));
#endif
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(float) v34_rx_carrier_frequency(v34_state_t *s)
{
    return dds_frequency(s->rx.v34_carrier_phase_rate);
}
/*- End of function --------------------------------------------------------*/

static void report_status_change(v34_rx_state_t *s, int status)
{
    if (s->put_bit)
        s->put_bit(s->put_bit_user_data, status);
    /*endif*/
}
/*- End of function --------------------------------------------------------*/

#if 0

SPAN_DECLARE(float) v34_rx_symbol_timing_correction(v34_state_t *s)
{
    return (float) s->rx.total_baud_timing_correction/((float) V34_RX_PULSESHAPER_COEFF_SETS*10.0f/3.0f);
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(float) v34_rx_signal_power(v34_state_t *s)
{
    return power_meter_current_dbm0(&s->rx.power);
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_equalizer_state(v34_state_t *s, complexf_t **coeffs)
{
    *coeffs = s->rx.eq_coeff;
    return V34_EQUALIZER_PRE_LEN + 1 + V34_EQUALIZER_POST_LEN;
}
/*- End of function --------------------------------------------------------*/

static void report_status_change(v34_rx_state_t *s, int status)
{
    if (s->status_handler)
        s->status_handler(s->status_user_data, status);
    else if (s->put_bit)
        s->put_bit(s->put_bit_user_data, status);
    /*endif*/
}
/*- End of function --------------------------------------------------------*/
#endif  /* #if 0 - disabled API functions above */

static void equalizer_save(v34_rx_state_t *s)
{
    cvec_copyf(s->eq_coeff_save, s->eq_coeff, V34_EQUALIZER_PRE_LEN + 1 + V34_EQUALIZER_POST_LEN);
}
/*- End of function --------------------------------------------------------*/

static void equalizer_restore(v34_rx_state_t *s)
{
    cvec_copyf(s->eq_coeff, s->eq_coeff_save, V34_EQUALIZER_PRE_LEN + 1 + V34_EQUALIZER_POST_LEN);
    cvec_zerof(s->eq_buf, V34_EQUALIZER_MASK);

    s->eq_put_step = V34_RX_PULSESHAPER_COEFF_SETS*10/(3*2) - 1;
    s->eq_step = 0;
    s->eq_delta = EQUALIZER_SLOW_ADAPT_RATIO*EQUALIZER_DELTA/(V34_EQUALIZER_PRE_LEN + 1 + V34_EQUALIZER_POST_LEN);
}
/*- End of function --------------------------------------------------------*/

static void equalizer_reset(v34_rx_state_t *s)
{
    /* Start with an equalizer based on everything being perfect */
    cvec_zerof(s->eq_coeff, V34_EQUALIZER_PRE_LEN + 1 + V34_EQUALIZER_POST_LEN);
    s->eq_coeff[V34_EQUALIZER_PRE_LEN] = complex_sig_set(TRAINING_SCALE(1.0f), TRAINING_SCALE(0.0f));
    cvec_zerof(s->eq_buf, V34_EQUALIZER_MASK);

    s->eq_put_step = V34_RX_PULSESHAPER_COEFF_SETS*10/(3*2) - 1;
    s->eq_step = 0;
    s->eq_delta = EQUALIZER_DELTA/(V34_EQUALIZER_PRE_LEN + 1 + V34_EQUALIZER_POST_LEN);
    s->eq_target_mag = 0.0f;  /* Will be initialized from first equalizer output */
}
/*- End of function --------------------------------------------------------*/

static complexf_t equalizer_get(v34_rx_state_t *s)
{
    int i;
    int p;
    complexf_t z;
    complexf_t z1;

    /* Get the next equalized value. */
    z = zero;
    p = s->eq_step - 1;
    for (i = 0;  i < V34_EQUALIZER_PRE_LEN + 1 + V34_EQUALIZER_POST_LEN;  i++)
    {
        p = (p - 1) & V34_EQUALIZER_MASK;
        z1 = complex_mulf(&s->eq_coeff[i], &s->eq_buf[p]);
        z = complex_addf(&z, &z1);
    }
    /*endfor*/
    /* Guard against NaN/Inf from coefficient divergence — reset to
       center tap if the equalizer has blown up. */
    if (!isfinite(z.re) || !isfinite(z.im))
    {
        if (getenv("ME_V34_DUMP_MP_DIBITS"))
        {
            span_log(s->logging, SPAN_LOG_FLOW,
                     "Rx - EQUALIZER DIVERGED (NaN/Inf) at duration=%d stage=%d; resetting to identity coeffs\n",
                     s->duration, s->stage);
        }
        cvec_zerof(s->eq_coeff, V34_EQUALIZER_PRE_LEN + 1 + V34_EQUALIZER_POST_LEN);
        s->eq_coeff[V34_EQUALIZER_PRE_LEN] = complex_sig_set(TRAINING_SCALE(1.0f), TRAINING_SCALE(0.0f));
        z.re = 0.0f;
        z.im = 0.0f;
    }
    /*endif*/
    return z;
}
/*- End of function --------------------------------------------------------*/

static void tune_equalizer(v34_rx_state_t *s, const complexf_t *z, const complexf_t *target)
{
    int i;
    int p;
    complexf_t ez;
    complexf_t z1;

    /* Find the x and y mismatch from the exact constellation position. */
    ez = complex_subf(target, z);
    if (!isfinite(z->re) || !isfinite(z->im)
        || !isfinite(target->re) || !isfinite(target->im)
        || !isfinite(ez.re) || !isfinite(ez.im))
    {
        return;
    }
    /*endif*/
    /* Log equalizer error magnitude periodically */
    if ((s->duration & 0xFF) == 0)
    {
        float emag = sqrtf(ez.re*ez.re + ez.im*ez.im);
        float zmag = sqrtf(z->re*z->re + z->im*z->im);
        fprintf(stderr, "[EQ] baud=%d err=%.4f mag=%.4f target_mag=%.4f delta=%.6f\n",
                s->duration, emag, zmag, s->eq_target_mag, s->eq_delta);
    }
    ez.re *= s->eq_delta;
    ez.im *= s->eq_delta;

    p = s->eq_step - 1;
    for (i = 0;  i < V34_EQUALIZER_PRE_LEN + 1 + V34_EQUALIZER_POST_LEN;  i++)
    {
        p = (p - 1) & V34_EQUALIZER_MASK;
        z1 = complex_conjf(&s->eq_buf[p]);
        z1 = complex_mulf(&ez, &z1);
        s->eq_coeff[i] = complex_addf(&s->eq_coeff[i], &z1);
        /* Leak disabled — was causing coefficient decay faster than LMS convergence
           (0.9999^2400 = 0.787/sec), making equalizer actively harmful */
    }
    /*endfor*/
}
/*- End of function --------------------------------------------------------*/

static void tune_equalizer_cma(v34_rx_state_t *s, const complexf_t *z)
{
    int i;
    int p;
    complexf_t gz;
    complexf_t z1;
    float y_mag2;
    float R2;
    float error;
    float cma_delta;

    /* CMA (Constant Modulus Algorithm) — blind equalizer for constant-envelope
       signals like DQPSK.  Minimizes E[(R² - |y|²)²] without needing to know
       which symbol was transmitted.  Immune to the decision-directed convergence
       failure that occurs at 25% BER. */
    y_mag2 = z->re*z->re + z->im*z->im;
    R2 = 1.0f;  /* Fixed unit radius for QPSK — R²=1.0 */
    error = R2 - y_mag2;

    /* On a real analogue line, the Phase 3 solution is usually useful at
       the start of Phase 4.  Keep CMA available for slow level correction,
       but avoid letting its blind phase-insensitive gradient pull the taps
       away from that trained solution while MP/CP is being acquired.  This
       is deliberately opt-in for live A/B testing. */
    cma_delta = s->eq_delta;
    if (v34_rx_stage_is_phase4_frame(s->stage)
        && getenv("ME_V34_SLOW_CMA_DURING_MP"))
    {
        cma_delta *= EQUALIZER_SLOW_ADAPT_RATIO;
    }

    /* Log CMA error periodically */
    if (V34_TRACE_DIAGNOSTICS && ((s->duration & 0xFF) == 0))
    {
        fprintf(stderr, "[CMA] baud=%d err=%.4f mag=%.4f R=1.0000 delta=%.6f\n",
                s->duration, error, sqrtf(y_mag2), s->eq_delta);
    }

    /* Normalized CMA gradient: error * y / |y|² — normalizing by |y|²
       prevents the positive feedback loop where large output → large gradient
       → even larger output.  This is equivalent to NCMA and makes convergence
       independent of the output magnitude.  Floor y_mag2 to avoid division
       by zero when the equalizer output is near zero. */
    if (error < -4.0f * R2)
        error = -4.0f * R2;
    else if (error > 4.0f * R2)
        error = 4.0f * R2;
    /*endif*/
    {
        float norm = (y_mag2 > 0.001f) ? y_mag2 : 0.001f;
        gz.re = 0.1f * cma_delta * error * z->re / norm;
        gz.im = 0.1f * cma_delta * error * z->im / norm;
    }

    p = s->eq_step - 1;
    for (i = 0;  i < V34_EQUALIZER_PRE_LEN + 1 + V34_EQUALIZER_POST_LEN;  i++)
    {
        p = (p - 1) & V34_EQUALIZER_MASK;
        z1 = complex_conjf(&s->eq_buf[p]);
        z1 = complex_mulf(&gz, &z1);
        s->eq_coeff[i] = complex_addf(&s->eq_coeff[i], &z1);
    }
    /*endfor*/
}
/*- End of function --------------------------------------------------------*/

#if 0  /* Disabled functions - track_carrier reimplemented inline, others unused */
static void track_carrier(v34_rx_state_t *s, const complexf_t *z, const complexf_t *target)
{
    float error;

    /* For small errors the imaginary part of the difference between the actual and the target
       positions is proportional to the phase error, for any particular target. However, the
       different amplitudes of the various target positions scale things. */
    error = z->im*target->re - z->re*target->im;

    s->v34_carrier_phase_rate += (int32_t) (s->carrier_track_i*error);
    s->carrier_phase += (int32_t) (s->carrier_track_p*error);
    //span_log(s->logging, SPAN_LOG_FLOW, "Rx - Im = %15.5f   f = %15.5f\n", error, dds_frequencyf(s->v34_carrier_phase_rate));
    //printf("XXX Im = %15.5f   f = %15.5f   %f %f %f %f (%f %f)\n", error, dds_frequencyf(s->v34_carrier_phase_rate), target->re, target->im, z->re, z->im, s->carrier_track_i, s->carrier_track_p);
}
/*- End of function --------------------------------------------------------*/

static __inline__ void put_bit(v34_rx_state_t *s, int bit)
{
    int out_bit;

    /* We need to strip the last part of the training - the test period of all 1s -
       before we let data go to the application. */
    if (s->training_stage == TRAINING_TX_STAGE_NORMAL_OPERATION_V34)
    {
        out_bit = descramble(s, bit);
        s->put_bit(s->put_bit_user_data, out_bit);
    }
    else if (s->training_stage == TRAINING_STAGE_TEST_ONES)
    {
        /* The bits during the final stage of training should be all ones. However,
           buggy modems mean you cannot rely on this. Therefore we don't bother
           testing for ones, but just rely on a constellation mismatch measurement. */
        out_bit = descramble(s, bit);
        //span_log(s->logging, SPAN_LOG_FLOW, "Rx - A 1 is really %d\n", out_bit);
    }
    /*endif*/
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

#if defined(SPANDSP_USE_FIXED_POINT)
#else
static void straight_line_fit(float *slope, float *intercept, const float x[], const float y[], int data_points)
{
    float sum_x;
    float sum_y;
    float sum_xy;
    float sum_x2;
    float slopex;
    int i;

    sum_x = 0.0f;
    sum_y = 0.0f;
    sum_xy = 0.0f;
    sum_x2 = 0.0f;
    for (i = 0;  i < data_points;  i++)
    {
        sum_x += x[i];
        sum_y += y[i];
        sum_xy += x[i]*y[i];
        sum_x2 += x[i]*x[i];
    }
    /*endfor*/
    slopex = (sum_xy - sum_x*sum_y/data_points)/(sum_x2 - sum_x*sum_x/data_points);
    if (slope)
        *slope = slopex;
    /*endif*/
    if (intercept)
        *intercept = (sum_y - slopex*sum_x)/data_points;
    /*endif*/
}
/*- End of function --------------------------------------------------------*/
#endif

static void slow_dft(complexf_t data[], int len)
{
    int i;
    int bin;
    float arg;
    complexf_t buf[len];

    for (i = 0;  i < len;  i++)
    {
        buf[i].re = data[i].re;
        buf[i].im = data[i].im;
    }
    /*endfor*/

    for (bin = 0;  bin <= len/2;  bin++)
    {
        data[bin].re =
        data[bin].im = 0.0;
        for (i = 0;  i < len;  i++)
        {
            arg = bin*2.0f*3.1415926535f*i/(float) len;
            data[bin].re -= buf[i].re*sinf(arg);
            data[bin].im += buf[i].re*cosf(arg);
        }
        /*endfor*/
    }
    /*endfor*/
}
/*- End of function --------------------------------------------------------*/

static int perform_l1_l2_analysis(v34_rx_state_t *s)
{
    /* Phase adjustments to compensate for the tones which are sent phase inverted */
    static const float adjust[25] =
    {
        0.0f,           /**/
        3.14159265f,    /* 300 */
        0.0f,           /**/
        0.0f,           /**/
        0.0f,           /**/
        42.0f,          /* Tone not sent */
        0.0f,           /* 1050 nominal line probe frequency */
        42.0f,          /* Tone not sent */
        0.0f,           /**/
        0.0f,           /**/
        3.14159265f,    /* 1650 */
        42.0f,          /* Tone not sent */
        0.0f,           /**/
        0.0f,           /**/
        3.14159265f,    /* 2250 */
        42.0f,          /* Tone not sent */
        0.0f,           /**/
        3.14159265f,    /* 2700 */
        0.0f,           /**/
        3.14159265f,    /* 3000 */
        3.14159265f,    /* 3150 */
        3.14159265f,    /* 3300 */
        3.14159265f,    /* 3450 */
        0.0f,           /**/
        0.0f            /**/
    };
    int i;
    int j;

    slow_dft(s->dft_buffer, LINE_PROBE_SAMPLES);
    /* Now resolve the analysis into gain and phase values for the bins which contain the tones */ 
    /* Base things around what happens at 1050Hz the first time through. */
    if (s->l1_l2_duration == 0)
        s->base_phase = atan2f(s->dft_buffer[21].im, s->dft_buffer[21].re);
    /*endif*/
    for (i = 0;  i < 25;  i++)
    {
        if (adjust[i] < 7.0f)
        {
            /* This tone should be present in the transmitted signal. */
            j = 3*(i + 1);
            s->l1_l2_gains[i] = sqrtf(s->dft_buffer[j].re*s->dft_buffer[j].re
                                    + s->dft_buffer[j].im*s->dft_buffer[j].im);
            s->l1_l2_phases[i] = fmodf(atan2f(s->dft_buffer[j].im, s->dft_buffer[j].re) - s->base_phase + adjust[i],
                                       3.14159265f);
        }
        else
        {
            /* This tone should not be present in the transmitted signal. */
            s->l1_l2_gains[i] = 0.0f;
            s->l1_l2_phases[i] = 0.0f;
        }
        /*endif*/
    }
    /*endfor*/
    for (i = 0;  i < 25;  i++)
    {
        span_log(s->logging, SPAN_LOG_DEBUG, "DFT %4d, %12.5f, %12.5f, %12.5f\n",
                 i,
                 (i + 1)*150.0f,
                 s->l1_l2_gains[i],
                 s->l1_l2_phases[i]);
    }
    /*endfor*/
    //straight_line_fit(&slope, &intercept, x, y, data_points);
    return 0;
}
/*- End of function --------------------------------------------------------*/

static void l1_l2_analysis_init(v34_rx_state_t *s)
{
    span_log(s->logging, SPAN_LOG_FLOW, "Rx - Expect L1/L2\n");
    s->dft_ptr = 0;
    s->base_phase = 42.0;
    s->l1_l2_duration = 0;
    s->current_demodulator = V34_MODULATION_L1_L2;
    s->stage = V34_RX_STAGE_L1_L2;
}
/*- End of function --------------------------------------------------------*/

static int l1_l2_analysis(v34_rx_state_t *s, const int16_t amp[], int len)
{
    int i;

    /* We need to work over whole cycles of the L1/L2 pattern, to avoid windowing and
       all its ills. One cycle takes 160/3 samples at 8000 samples/second, so we will
       process groups of 3 cycles, and run a Fourier transform every 160 samples (20ms).
       Since this is not a suitable length for an FFT we have to run a slow DFT. However,
       we don't do this for much of the time, so its not that big a deal. */
    for (i = 0;  i < len;  i++)
    {
        s->dft_buffer[s->dft_ptr].re = amp[i];
        s->dft_buffer[s->dft_ptr].im = 0.0f;
        if (++s->dft_ptr >= LINE_PROBE_SAMPLES)
        {
            /* We now have 160 samples, so process the 3 cycles we should have in the buffer. */
            perform_l1_l2_analysis(s);
            s->dft_ptr = 0;
            span_log(s->logging, SPAN_LOG_DEBUG, "L1/L2 analysis x %d\n", s->l1_l2_duration);
            if (++s->l1_l2_duration > 20)
            {
                span_log(s->logging, SPAN_LOG_FLOW, "L1/L2 analysis done\n");
                s->phase2_l2_count++;
                s->received_event = V34_EVENT_L2_SEEN;
                s->current_demodulator = V34_MODULATION_TONES;
                if (s->calling_party)
                {
                    s->stage = V34_RX_STAGE_TONE_A;
                    /* L2 has been seen, so Phase 2 is demonstrably past the
                       first two reversals. Seed the durable counter with that
                       rather than zeroing it: received_event carries the same
                       fact today, but v34tx.c clears it as it consumes events,
                       which is exactly how this progress used to get lost. */
                    s->phase2_reversal_count = 2;
                }
                else if (s->v90_mode)
                {
                    s->stage = V34_RX_STAGE_TONE_A;
                    s->phase2_reversal_count = 2;   /* see above: L2 seen */
                    s->persistence1 = 0;
                    s->persistence2 = 0;
                }
                else
                    s->stage = V34_RX_STAGE_INFO1C;
            }
            /*endif*/
        }
        /*endif*/
    }
    /*endfor*/
    /* Also run this signal through the info analysis, so we pick up A or B tones */
    info_rx(s, amp, len);

    return 0;
}
/*- End of function --------------------------------------------------------*/

static void process_cc_half_baud(v34_rx_state_t *s, const complexf_t *sample)
{
    int i;
    int data_bits;
    mp_t mp;
    mph_t mph;
    uint32_t ang1;
    uint32_t ang2;
    uint32_t ang3;
    int bits[4];
    v34_state_t *t;

    /* This routine processes every half a baud, as we put things into the equalizer
       at the T/2 rate. This routine adapts the position of the half baud samples,
       which the caller takes. */
#if 0
    /* Add a sample to the equalizer's circular buffer, but don't calculate anything at this time. */
    s->eq_buf[s->eq_step] = *sample;
    s->eq_step = (s->eq_step + 1) & V34_EQUALIZER_MASK;
#endif

    /* On alternate insertions we have a whole baud and must process it. */
    if ((s->baud_half ^= 1))
        return;
    /*endif*/
    cc_symbol_sync(s);

    /* Slice the phase difference, to get a pair of data bits */
    ang1 = arctan2(sample->im, sample->re);
    ang2 = arctan2(s->last_sample.im, s->last_sample.re);
    ang3 = ang1 - ang2 + DDS_PHASE(45.0f);
    data_bits = (ang3 >> 30) & 0x3;

    /* Descramble the data bits. */
    for (i = 0;  i < 2;  i++)
    {
        bits[i] = descramble(s, data_bits & 1);
        data_bits >>= 1;
    }
    /*endfor*/

    /* Scan for MP/MPh and HDLC messages. */
    for (i = 0;  i < 2;  i++)
    {
        s->bitstream = (s->bitstream << 1) | bits[i];
        if (s->mp_seen >= 2)
        {
            /* Real control channel data */
            s->put_bit(s->put_bit_user_data, bits[i]);
            continue;
        }
        /*endif*/
        if (s->mp_seen == 1  &&  (s->bitstream & 0xFFFFF) == 0xFFFFF)
        {
            /* E is 20 consecutive ones, which signals the end of the MPh messages,
               and the start of actual user data */
            span_log(s->logging, SPAN_LOG_FLOW,
                     "Rx - CC: E signal detected, MP exchange complete\n");
            s->mp_seen = 2;
            if (s->duplex)
            {
                report_status_change(s, SIG_STATUS_TRAINING_SUCCEEDED);
            }
            /*endif*/
        }
        else if ((s->bitstream & 0x7FFFE) == 0x7FFFC)
        {
            s->crc = 0xFFFF;
            s->bit_count = 0;
            s->mp_count = 17;
            /* Check the type bit, and set the expected length accordingly. */
            if (bits[i])
            {
                s->mp_len = 186 + 1;
                s->mp_and_fill_len = 186 + 1 + 1;
            }
            else
            {
                s->mp_len = 84 + 1;
                s->mp_and_fill_len = 84 + 3 + 1;
            }
            /*endif*/
        }
        /*endif*/
        if (s->mp_count >= 0)
        {
            s->mp_count++;
            /* Don't include the start bits in the CRC calculation. These occur every 16 bits of
               real data - i.e. every 17 bits, including the start bits themselves. */
            if (s->mp_count%17 != 0)
                s->crc = crc_itu16_bits(bits[i], 1, s->crc);
            /*endif*/
            s->bit_count++;
            if ((s->bit_count & 0x07) == 0)
                s->info_buf[(s->bit_count >> 3) - 1] = bit_reverse8(s->bitstream & 0xFF);
            /*endif*/
            if (s->mp_count >= s->mp_len)
            {
                if (s->mp_count == s->mp_len)
                {
                    /* This should be the end of the MPh message */
                    if (s->crc == 0)
                    {
                        if (s->duplex)
                        {
                            process_rx_mp(s, &mp, s->info_buf);
                            if (mp.mp_acknowledged)
                                s->mp_remote_ack_seen = 1;
                            /*endif*/
                            t = ((v34_state_t *) ((char *)(s) - offsetof(v34_state_t, rx)));
                            if (mp.type == 1)
                            {
                                /* Set the precoder coefficients we are to use */
                                memcpy(&t->tx.precoder_coeffs, mp.precoder_coeffs, sizeof(t->tx.precoder_coeffs));
                            }
                            /*endif*/
                            if (set_trellis_mode(t, mp.trellis_size))
                                span_log(&t->logging, SPAN_LOG_FLOW, "Rx - Unexpected trellis size code %d\n", mp.trellis_size);
                        }
                        else
                        {
                            process_rx_mph(s, &mph, s->info_buf);
                            t = ((v34_state_t *) ((char *)(s) - offsetof(v34_state_t, rx)));
                            if (mph.type == 1)
                            {
                                /* Set the precoder coefficients we are to use */
                                memcpy(&t->tx.precoder_coeffs, mph.precoder_coeffs, sizeof(t->tx.precoder_coeffs));
                            }
                            /*endif*/
                            if (set_trellis_mode(t, mph.trellis_size))
                                span_log(&t->logging, SPAN_LOG_FLOW, "Rx - Unexpected trellis size code %d\n", mph.trellis_size);
                        }
                        /*endif*/
                        s->mp_seen = 1;
                        s->mp_accepted_baud = s->duration;
                    }
                    /*endif*/
                }
                /*endif*/
                /* Allow for the fill bits before ending the MP message */
                if (s->mp_count == s->mp_and_fill_len)
                    s->mp_count = -1;
                /*endif*/
            }
            /*endif*/
        }
        /*endif*/
    }
    /*endfor*/

    s->last_sample = *sample;
}
/*- End of function --------------------------------------------------------*/

static int cc_rx(v34_rx_state_t *s, const int16_t amp[], int len)
{
    int i;
    int step;
#if defined(SPANDSP_USE_FIXED_POINT)
    complexi16_t z;
    complexi16_t zz;
    complexi16_t sample;
#else
    complexf_t z;
    complexf_t zz;
    complexf_t sample;
#endif
    float ii;
    float qq;
    float v;

    step = 6;
    for (i = 0;  i < len;  i++)
    {
        s->rrc_filter[s->rrc_filter_step] = amp[i];
        if (++s->rrc_filter_step >= V34_RX_FILTER_STEPS)
            s->rrc_filter_step = 0;
        /*endif*/

        s->eq_put_step -= RX_PULSESHAPER_2400_COEFF_SETS;
        step = -s->eq_put_step;
        if (step > RX_PULSESHAPER_2400_COEFF_SETS - 1)
            step = RX_PULSESHAPER_2400_COEFF_SETS - 1;
        /*endif*/
        while (step < 0)
            step += RX_PULSESHAPER_2400_COEFF_SETS;
        /*endwhile*/
        /* Standard V.34: caller RX 2400, answerer RX 1200.
           V.90: swapped — caller RX 1200, answerer RX 2400. */
        if (s->calling_party != s->v90_mode)
        {
#if defined(SPANDSP_USE_FIXED_POINT)
            ii = vec_circular_dot_prodi16(s->rrc_filter, rx_pulseshaper_2400_re[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
#else
            ii = vec_circular_dot_prodf(s->rrc_filter, rx_pulseshaper_2400_re[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
#endif
        }
        else
        {
#if defined(SPANDSP_USE_FIXED_POINT)
            ii = vec_circular_dot_prodi16(s->rrc_filter, rx_pulseshaper_1200_re[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
#else
            ii = vec_circular_dot_prodf(s->rrc_filter, rx_pulseshaper_1200_re[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
#endif
        }
        /*endif*/
#if defined(SPANDSP_USE_FIXED_POINT)
        //sample.re = (ii*(int32_t) s->agc_scaling) >> 15;
        sample.re = ii*s->agc_scaling;
#else
        sample.re = ii*s->agc_scaling;
#endif
        /* Symbol timing synchronisation band edge filters */
        /* Low Nyquist band edge filter */
        v = s->cc_ted.symbol_sync_low[0]*s->cc_ted.low_band_edge_coeff[0] + s->cc_ted.symbol_sync_low[1]*s->cc_ted.low_band_edge_coeff[1] + sample.re;
        s->cc_ted.symbol_sync_low[1] = s->cc_ted.symbol_sync_low[0];
        s->cc_ted.symbol_sync_low[0] = v;
        /* High Nyquist band edge filter */
        v = s->cc_ted.symbol_sync_high[0]*s->cc_ted.high_band_edge_coeff[0] + s->cc_ted.symbol_sync_high[1]*s->cc_ted.high_band_edge_coeff[1] + sample.re;
        s->cc_ted.symbol_sync_high[1] = s->cc_ted.symbol_sync_high[0];
        s->cc_ted.symbol_sync_high[0] = v;

        /* Put things into the equalization buffer at T/2 rate. The symbol synchcronisation
           will fiddle the step to align this with the symbols. */
        if (s->eq_put_step <= 0)
        {
            /* CC channel AGC not needed — Phase 2 works with fixed scaling.
               Primary channel AGC is re-enabled in primary_channel_rx(). */
            s->eq_put_step += RX_PULSESHAPER_2400_COEFF_SETS*40/(3*2);
            /* Same carrier logic as above */
            if (s->calling_party != s->v90_mode)
            {
#if defined(SPANDSP_USE_FIXED_POINT)
                qq = vec_circular_dot_prodi16(s->rrc_filter, rx_pulseshaper_2400_im[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
#else
                qq = vec_circular_dot_prodf(s->rrc_filter, rx_pulseshaper_2400_im[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
#endif
            }
            else
            {
#if defined(SPANDSP_USE_FIXED_POINT)
                qq = vec_circular_dot_prodi16(s->rrc_filter, rx_pulseshaper_1200_im[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
#else
                qq = vec_circular_dot_prodf(s->rrc_filter, rx_pulseshaper_1200_im[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
#endif
            }
            /*endif*/
#if defined(SPANDSP_USE_FIXED_POINT)
            //sample.im = (qq*(int32_t) s->agc_scaling) >> 15;
            sample.im = qq*s->agc_scaling;
            z = dds_lookup_complexi16(s->carrier_phase);
#else
            sample.im = qq*s->agc_scaling;
            z = dds_lookup_complexf(s->carrier_phase);
#endif
            zz.re = sample.re*z.re - sample.im*z.im;
            zz.im = -sample.re*z.im - sample.im*z.re;
            process_cc_half_baud(s, &zz);

        }
        /*endif*/
        /* Use CC carrier phase rate, not V34 primary channel rate.
           For answerer RX: caller CC at 1200 Hz.
           For caller RX: answerer CC at 2400 Hz. */
#if defined(SPANDSP_USE_FIXED_POINT)
        dds_advance(&s->carrier_phase, s->cc_carrier_phase_rate);
#else
        dds_advancef(&s->carrier_phase, s->cc_carrier_phase_rate);
#endif
    }
    /*endfor*/
    return 0;
}
/*- End of function --------------------------------------------------------*/

static void process_primary_half_baud(v34_rx_state_t *s, const complexf_t *sample)
{
    float energy;
    uint32_t ang1;
    uint32_t ang2;
    uint32_t ang3;
    int data_bits;
    int phase3_abs_bits;
    int bits[4];
    int i;
    mp_t mp;
    v34_state_t *t;
    complexf_t eq_sample;
    complexf_t eq_target;
    const complexf_t *sym;

    /* This routine processes every half a baud, as we put things into the equalizer at the T/2 rate.
       This routine adapts the position of the half baud samples, which the caller takes. */

    /* Feed the T/2-rate primary channel samples into the equalizer buffer. */
    s->eq_buf[s->eq_step] = *sample;
    s->eq_step = (s->eq_step + 1) & V34_EQUALIZER_MASK;

    /* On alternate insertions we have a whole baud and must process it. */
    if ((s->baud_half ^= 1))
        return;
    /*endif*/
    pri_symbol_sync(s);
    eq_sample = equalizer_get(s);
    sym = &eq_sample;
    if (s->qam_report)
        s->qam_report(s->qam_user_data, sym, NULL, s->qam_sample_time);

    if (V34_TRACE_DIAGNOSTICS
        && (s->stage == V34_RX_STAGE_PHASE3_TRAINING || s->stage == V34_RX_STAGE_PHASE3_WAIT_S)
        && (s->duration == 0 || (s->duration % 256) == 0))
    {
        float sym_mag = sqrtf(sym->re*sym->re + sym->im*sym->im);
        fprintf(stderr,
                "[V34 RAW] primary_half_baud: stage=%d dur=%d sym=(%.4f,%.4f) mag=%.4f agc=%.5f\n",
                s->stage, s->duration, (double)sym->re, (double)sym->im,
                (double)sym_mag, (double)s->agc_scaling);
    }
    /*endif*/

    /* Phase 3 S signal detection state machine */
    switch (s->stage)
    {
    case V34_RX_STAGE_PHASE3_WAIT_S:
        /* Phase 3 S detection using demodulated differential phase.
           Use a dominant-symbol detector over a 32-baud window.
           In real channels, phase/mapping ambiguity can move S away from a
           fixed symbol index, so don't hardcode data_bits==2. */
        {
            float mag_now;
            float mag_prev;
            float dot;
            int idx;
            int dominant_symbol;
            int dominant_count;
            int previous_symbol;
            int prior2_symbol;
            int old_alt;
            int new_alt;
            int old_rev;
            int new_rev;

        ang1 = arctan2(sym->im, sym->re);
        ang2 = arctan2(s->last_sample.im, s->last_sample.re);
        ang3 = ang1 - ang2 + DDS_PHASE(45.0f);
        data_bits = (ang3 >> 30) & 0x3;
        phase3_abs_bits = (int) ((ang1 + DDS_PHASE(45.0f)) >> 30) & 0x3;
        s->duration++;

            if (V34_TRACE_DIAGNOSTICS
                && (s->duration == 1 || (s->duration % 256) == 0))
            {
                fprintf(stderr,
                        "[V34 RAW] WAIT_S case: s=%p stage=%d demod=%d duration=%d event=%d j_bits=%d trn_bits=%d hint=%d/%d%%\n",
                        (void *) s, s->stage, s->current_demodulator, s->duration,
                        s->received_event, s->phase3_j_bits, s->phase3_trn_bits,
                        s->phase3_trn_lock_hyp, s->phase3_trn_lock_score);
            }
            /*endif*/

            if (s->duration == 1 || (s->duration % 256) == 0)
            {
                span_log(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 3 WAIT_S heartbeat: dur=%d demod=%d event=%d j_bits=%d trn_bits=%d trn_hint=%d/%d%%\n",
                         s->duration, s->current_demodulator, s->received_event,
                         s->phase3_j_bits, s->phase3_trn_bits,
                         s->phase3_trn_lock_hyp, s->phase3_trn_lock_score);
            }
            /*endif*/

            /* Explicit Phase 3 J/J' detector:
               - apply all dibit mapping hypotheses
               - map the phase-difference slicer output I_n
               - descramble
               - correlate against J/J' 16-bit templates

               Both roles need the far-end J decode:
               - answerer uses it to leave Phase 3 J and enter Phase 4 wait
               - caller uses it to learn the far-end TRN/J mode before it can
                 legally terminate its own J with J' once S is later seen */
            {
                int h;
                int best_score;
                int best_h;
                int best_p;
                int cap_bit0[MP_HYPOTHESIS_COUNT];
                int cap_bit1[MP_HYPOTHESIS_COUNT];
                uint8_t cap_valid[MP_HYPOTHESIS_COUNT];

                best_score = 0;
                best_h = -1;
                best_p = 0;
                memset(cap_bit0, 0, sizeof(cap_bit0));
                memset(cap_bit1, 0, sizeof(cap_bit1));
                memset(cap_valid, 0, sizeof(cap_valid));
                for (h = 0;  h < MP_HYPOTHESIS_COUNT;  h++)
                {
                    int raw_sym;

                    raw_sym = map_phase4_raw_bits(data_bits, h);
                    if (s->phase3_j_prev_valid[h])
                    {
                        int in_sym;
                        uint32_t reg;
                        int dbit[2];
                        int b;

                        in_sym = raw_sym;
                        reg = s->phase3_j_scramble[h];
                        dbit[0] = descramble_reg(&reg, s->scrambler_tap, in_sym & 1);
                        dbit[1] = descramble_reg(&reg, s->scrambler_tap, (in_sym >> 1) & 1);
                        s->phase3_j_scramble[h] = reg;
                        cap_bit0[h] = dbit[0];
                        cap_bit1[h] = dbit[1];
                        cap_valid[h] = 1;
                        if (s->phase3_ja_capture_hyp_len[h] + 2 <= (int) sizeof(s->phase3_ja_capture_hyp[h]))
                        {
                            s->phase3_ja_capture_hyp[h][s->phase3_ja_capture_hyp_len[h]++] = (uint8_t) (dbit[0] & 1);
                            s->phase3_ja_capture_hyp[h][s->phase3_ja_capture_hyp_len[h]++] = (uint8_t) (dbit[1] & 1);
                        }
                        /*endif*/
                        if (s->phase3_ja_capture_hyp_raw_len[h] + 2 <= (int) sizeof(s->phase3_ja_capture_hyp_raw[h]))
                        {
                            s->phase3_ja_capture_hyp_raw[h][s->phase3_ja_capture_hyp_raw_len[h]++] = (uint8_t) (in_sym & 1);
                            s->phase3_ja_capture_hyp_raw[h][s->phase3_ja_capture_hyp_raw_len[h]++] = (uint8_t) ((in_sym >> 1) & 1);
                        }
                        /*endif*/
                        s->phase3_j_stream[h] = ((s->phase3_j_stream[h] << 1) | (uint32_t) dbit[0]) & 0xFFFFFFFFU;
                        s->phase3_j_stream[h] = ((s->phase3_j_stream[h] << 1) | (uint32_t) dbit[1]) & 0xFFFFFFFFU;

                        for (b = 0;  b < 2;  b++)
                        {
                            int t;
                            int bit_pos;

                            bit_pos = s->phase3_j_bits + b;
                            for (t = 0;  t < 3;  t++)
                            {
                                int p;

                                for (p = 0;  p < 16;  p++)
                                {
                                    uint32_t w;
                                    int match;
                                    int score;

                                    match = (dbit[b] == phase3_j_pattern_bit(t, bit_pos + p)) ? 1 : 0;
                                    w = s->phase3_j_win[h][t][p];
                                    w = (w << 1) | (uint32_t) match;
                                    s->phase3_j_win[h][t][p] = w;
                                    score = __builtin_popcount(w);
                                    if (score > best_score)
                                    {
                                        best_score = score;
                                        best_h = h;
                                        best_p = p;
                                    }
                                    /*endif*/
                                }
                                /*endfor*/
                            }
                            /*endfor*/
                        }
                        /*endfor*/
                    }
                    /*endif*/
                    s->phase3_j_prev_z[h] = (uint8_t) raw_sym;
                    s->phase3_j_prev_valid[h] = 1;
                }
                /*endfor*/
                s->phase3_j_bits += 2;
                if (!s->calling_party
                    && s->v90_mode
                    && s->phase3_j_trn16 < 0
                    && v90_phase3_j_lookahead_bits() > 0
                    && s->phase3_j_bits >= v90_phase3_j_lookahead_bits()
                    && (s->received_event == V34_EVENT_NONE
                        || s->received_event == V34_EVENT_S))
                {
                    /* The SmartLink test rig has roughly one media frame more
                       receive latency than its downstream-Sd wait permits.  An
                       explicitly configured recovered-bit threshold starts the
                       digital side early enough to compensate.  SmartLink uses
                       the 4-point TRN path in this rig; pin it so the later S
                       transition remains detectable after this synthetic J. */
                    s->phase3_j_trn16 = 0;
                    s->phase3_s_detect_armed = true;
                    s->received_event = V34_EVENT_J;
                    span_log(s->logging, SPAN_LOG_FLOW,
                             "Rx - Phase 3: configured V.90 J look-ahead fired at bits=%d (ME_V90_J_LOOKAHEAD_BITS=%d, trn=4-point)\n",
                             s->phase3_j_bits, v90_phase3_j_lookahead_bits());
                }
                /*endif*/
                {
                    int capture_h;

                    capture_h = -1;
                    if (s->phase3_j_lock_hyp >= 0
                        && s->phase3_j_lock_hyp < MP_HYPOTHESIS_COUNT
                        && cap_valid[s->phase3_j_lock_hyp])
                    {
                        capture_h = s->phase3_j_lock_hyp;
                    }
                    else if (s->phase3_ja_hyp >= 0
                             && s->phase3_ja_hyp < MP_HYPOTHESIS_COUNT
                             && cap_valid[s->phase3_ja_hyp])
                    {
                        capture_h = s->phase3_ja_hyp;
                    }
                    else if (s->phase3_trn_lock_hyp >= 0
                             && s->phase3_trn_lock_hyp < MP_HYPOTHESIS_COUNT
                             && cap_valid[s->phase3_trn_lock_hyp])
                    {
                        capture_h = s->phase3_trn_lock_hyp;
                    }
                    else if (best_h >= 0
                             && best_h < MP_HYPOTHESIS_COUNT
                             && cap_valid[best_h])
                    {
                        capture_h = best_h;
                    }
                    /*endif*/
                    if (capture_h >= 0)
                    {
                        s->phase3_ja_hyp = capture_h;
                        phase3_ja_capture_append(s, cap_bit0[capture_h], cap_bit1[capture_h]);
                    }
                    /*endif*/
                }
                if (s->phase3_j_bits <= 8
                    || (s->phase3_j_bits % 64) == 0)
                {
                    span_log(s->logging, SPAN_LOG_FLOW,
                             "Rx - Phase 3 J progress: bits=%d best_score=%d/32 hyp=%d trn_hint=%d/%d%%\n",
                             s->phase3_j_bits, best_score, best_h,
                             s->phase3_trn_lock_hyp, s->phase3_trn_lock_score);
                }
                /*endif*/
                if (s->phase3_j_bits >= 16
                    &&
                    (s->phase3_j_bits == 16
                        ||
                        (s->phase3_j_bits % PHASE3_J_PROGRESS_LOG_INTERVAL) == 0))
                {
                    span_log(s->logging, SPAN_LOG_FLOW,
                             "Rx - Phase 3 J detector: bits=%d best_score=%d/32 hyp=%d\n",
                             s->phase3_j_bits, best_score, best_h);
                }
                /*endif*/
                if (s->phase3_j_bits >= 32
                    &&
                    best_score >= 24
                    &&
                    (s->received_event == V34_EVENT_NONE || s->received_event == V34_EVENT_S))
                {
                    int d4;
                    int d16;
                    int djd;
                    int dmin;
                    int pat;
                    uint16_t rx_recent16;
                    uint16_t rx_ordered16;
                    char rx_recent_bits[17];
                    char rx_ordered_bits[17];
                    const char *j_validity;
                    int canonical_ok;
                    int emit_diag;

                    rx_recent16 = (uint16_t) (s->phase3_j_stream[best_h] & 0xFFFFU);
                    rx_ordered16 = j_ordered16(rx_recent16, s->phase3_j_bits, best_p);
                    bits16_to_str(rx_recent16, rx_recent_bits);
                    bits16_to_str(rx_ordered16, rx_ordered_bits);
                    d4 = __builtin_popcount((unsigned) (rx_ordered16 ^ 0x8990U));
                    d16 = __builtin_popcount((unsigned) (rx_ordered16 ^ 0x89B0U));
                    djd = __builtin_popcount((unsigned) (rx_ordered16 ^ 0x899FU));
                    dmin = d4;
                    pat = 0;
                    if (d16 < dmin)
                    {
                        dmin = d16;
                        pat = 1;
                    }
                    /*endif*/
                    if (djd < dmin)
                    {
                        dmin = djd;
                        pat = 2;
                    }
                    /*endif*/
                    if (pat == 1  &&  (d16 + 1) >= d4)
                    {
                        /* Prefer 4-point when J(4)/J(16) are nearly tied.
                           A weak 1-bit advantage for 16-point is not stable
                           enough and can mis-classify TRN mode. */
                        pat = 0;
                        dmin = d4;
                    }
                    /*endif*/
                    canonical_ok = (dmin <= 3);
                    if (pat == 1)
                        j_validity = canonical_ok ? "valid J(16-point)" : "near/non-canonical";
                    else if (pat == 0)
                        j_validity = canonical_ok ? "valid J(4-point)" : "near/non-canonical";
                    else
                        j_validity = canonical_ok ? "valid J'" : "near/non-canonical";
                    emit_diag = canonical_ok || ((s->phase3_j_bits % 16) == 0);
                    if (emit_diag)
                    {
                        span_log(s->logging, SPAN_LOG_FLOW,
                                 "Rx - Phase 3 J bits: recent16=%s ordered16=%s phase=%d (%s, d4=%d d16=%d dj'=%d)\n",
                                 rx_recent_bits, rx_ordered_bits, best_p, j_validity, d4, d16, djd);
                    }
                    /*endif*/
                    if (canonical_ok)
                    {
                        if (pat == 2)
                        {
                            /* Phase 3 transition should be driven by J (Table 18).
                               Treat J' hits here as diagnostics only to avoid
                               premature Phase 4 transitions with unknown TRN size. */
                            if ((s->phase3_j_bits % 32) == 0)
                            {
                                span_log(s->logging, SPAN_LOG_FLOW,
                                         "Rx - Phase 3: canonical J' candidate ignored (hyp=%d phase=%d score=%d/32 bits=%d)\n",
                                         best_h, best_p, best_score, s->phase3_j_bits);
                            }
                            /*endif*/
                        }
                        else
                        {
                            /* Require a sustained canonical sequence near the
                               complete Ja signature on the normal path.  Test
                               rigs that need latency compensation use the
                               explicit ME_V90_J_LOOKAHEAD_BITS path above.
                               The sustained path additionally demands
                               near-perfect windows (dmin <= 1, score >= 28):
                               scrambled TRN payload transiently matches the J
                               pattern within 2-3 bit errors across the many
                               hypothesis/phase alignments searched, and eight
                               such weak hits in a row happen in practice -
                               live captures show a false "confirmed J" firing
                               mid-TRN (score 25/32, d4=2..3) which launched Sd
                               ~900 ms before the analogue modem was listening. */
                            if (s->phase3_j_bits < 6000
                                ||  dmin > 1
                                ||  best_score < 28)
                            {
                                s->phase3_j_candidate_count = 0;
                            }
                            else if (s->phase3_j_candidate_hyp == best_h
                                && s->phase3_j_candidate_phase == best_p
                                && s->phase3_j_candidate_pat == pat
                                && s->phase3_j_bits > s->phase3_j_candidate_last_bits
                                && s->phase3_j_bits - s->phase3_j_candidate_last_bits <= 4)
                            {
                                s->phase3_j_candidate_count++;
                                s->phase3_j_candidate_last_bits = s->phase3_j_bits;
                            }
                            else if (s->phase3_j_candidate_hyp != best_h
                                     || s->phase3_j_candidate_phase != best_p
                                     || s->phase3_j_candidate_pat != pat
                                     || s->phase3_j_bits - s->phase3_j_candidate_last_bits > 4)
                            {
                                s->phase3_j_candidate_hyp = best_h;
                                s->phase3_j_candidate_phase = best_p;
                                s->phase3_j_candidate_pat = pat;
                                s->phase3_j_candidate_count = 1;
                                s->phase3_j_candidate_last_bits = s->phase3_j_bits;
                            }
                            if (s->phase3_j_candidate_count >= 8)
                            {
                                int v90_ja_already_consumed;

                                /* The V.90 digital answerer consumes Ja before
                                   the analogue modem later transmits S.  The
                                   canonical detector continues observing the
                                   buffered Ja symbols after the application has
                                   cleared received_event; do not publish that
                                   same Ja a second time and let the TX state
                                   machine mistake it for a Phase 4 transition. */
                                v90_ja_already_consumed = (!s->calling_party
                                                          && s->v90_mode
                                                          && s->phase3_j_trn16 >= 0);
                                s->phase3_j_lock_hyp = best_h;
                                s->phase3_j_trn16 = pat;
                                s->phase3_s_detect_armed = true;
                                if (!s->calling_party && !v90_ja_already_consumed)
                                {
                                    s->received_event = V34_EVENT_J;
                                    span_log(s->logging, SPAN_LOG_FLOW,
                                             "Rx - Phase 3: confirmed repeating J/Ja (hyp=%d phase=%d hits=%d bits=%d, trn=%s)\n",
                                             best_h, best_p, s->phase3_j_candidate_count,
                                             s->phase3_j_bits, pat ? "16-point" : "4-point");
                                }
                                else if (s->calling_party)
                                {
                                    span_log(s->logging, SPAN_LOG_FLOW,
                                             "Rx - Phase 3: confirmed far-end J for caller (hyp=%d phase=%d hits=%d bits=%d, trn=%s)\n",
                                             best_h, best_p, s->phase3_j_candidate_count,
                                             s->phase3_j_bits, pat ? "16-point" : "4-point");
                                }
                            }
                            else
                            {
                                span_log(s->logging, SPAN_LOG_FLOW,
                                         "Rx - Phase 3: canonical J/Ja candidate %d/8 (hyp=%d phase=%d score=%d/32 bits=%d)\n",
                                         s->phase3_j_candidate_count, best_h, best_p,
                                         best_score, s->phase3_j_bits);
                            }
                        }
                    }
                    else
                    {
                        if (emit_diag)
                        {
                            span_log(s->logging, SPAN_LOG_FLOW,
                                     "Rx - Phase 3: J candidate rejected (non-canonical 16-bit pattern)\n");
                        }
                        /*endif*/
                    }
                }
                /*endif*/

                /* Phase 3 TRN ones-based lock hint (4-point path):
                   score hypotheses by descrambled ones ratio. TRN descrambles
                   to all-ones, so the correct hypothesis stands out.

                   This used to be gated on phase3_j_trn16 < 0, which made the
                   whole TRN lock hostage to a shared field: the canonical J
                   matcher writes its matched pattern there, and v34tx.c clears
                   it to -1 in three more places. The moment anything wrote it,
                   TRN scoring stopped dead. That is why the lock was a lottery
                   -- some calls reached 93%, others produced no lock line at
                   all -- and everything downstream (S-detector constellation,
                   Ja capture anchoring, the rescore) inherited that.

                   The latch below is already self-protecting: it only accepts
                   a hypothesis at >=70% and only ever improves on its best, so
                   letting this run on is harmless. Once TRN gives way to Ja the
                   ones ratio falls to ~50% and simply stops improving the
                   latch, rather than corrupting it. */
                if (s->duration >= 256)
                {
                    int h;
                    int best_trn_h;
                    int best_trn_score;

                    best_trn_h = -1;
                    best_trn_score = -1;
                    for (h = 0;  h < MP_HYPOTHESIS_COUNT;  h++)
                    {
                        int raw_sym;
                        uint32_t reg;
                        int d0;
                        int d1;

                        /* TRN is mapped directly onto the constellation; only
                           J/Ja passes through the differential encoder. */
                        raw_sym = map_phase4_raw_bits(phase3_abs_bits, h);
                        reg = s->phase3_trn_scramble[h];
                        d0 = descramble_reg(&reg, s->scrambler_tap, raw_sym & 1);
                        d1 = descramble_reg(&reg, s->scrambler_tap, (raw_sym >> 1) & 1);
                        s->phase3_trn_scramble[h] = reg;
                        s->phase3_trn_one_count[h] += (uint16_t) (d0 + d1);
                        if (s->phase3_trn_one_count[h] > best_trn_score)
                        {
                            best_trn_h = h;
                            best_trn_score = s->phase3_trn_one_count[h];
                        }
                        /*endif*/
                    }
                    /*endfor*/
                    s->phase3_trn_bits += 2;
                    /* Re-score once the Phase 3 tracking loops have settled.
                     *
                     * phase3_trn_one_count[] is cumulative over the whole TRN
                     * and the lock is a monotonic "best ever" latch, so a score
                     * measured across the tracking transient averages two
                     * different regimes.  Enabling tracking moved this peer's
                     * lock to the correct hypothesis but dropped its reported
                     * confidence 93% -> 72%, purely because the correct
                     * hypothesis carries the pre-convergence stretch where it
                     * was still wrong.  Clear the counters and the latch once,
                     * a little after tracking engages, so the lock is retaken
                     * on post-convergence data only. */
                    if (phase3_tracking_enabled()
                        &&
                        s->phase3_tracking_armed
                        &&
                        s->phase3_trn_rescore_bits == 0
                        &&
                        s->phase3_trn_bits >= 512)
                    {
                        span_log(s->logging, SPAN_LOG_FLOW,
                                 "Rx - Phase 3 TRN: rescoring after tracking settled "
                                 "(was hyp=%d %d%% over %d bits)\n",
                                 s->phase3_trn_lock_hyp, s->phase3_trn_lock_score,
                                 s->phase3_trn_bits);
                        memset(s->phase3_trn_one_count, 0, sizeof(s->phase3_trn_one_count));
                        s->phase3_trn_rescore_bits = s->phase3_trn_bits;
                        s->phase3_trn_bits = 0;
                        s->phase3_trn_lock_hyp = -1;
                        s->phase3_trn_lock_score = -1;
                    }
                    /*endif*/
                    {
                        float trn_mag = sqrtf(sym->re*sym->re + sym->im*sym->im);
                        if (isfinite(trn_mag) && trn_mag > 0.0f) {
                            s->phase3_trn_mag_sum += trn_mag;
                            s->phase3_trn_mag_count++;
                        }
                    }
                    if (s->phase3_trn_bits >= 256  &&  best_trn_h >= 0)
                    {
                        int score_pct;

                        score_pct = (100*best_trn_score + (s->phase3_trn_bits/2))/s->phase3_trn_bits;
                        if (score_pct >= 70
                            &&
                            (s->phase3_trn_lock_hyp < 0  ||  score_pct > s->phase3_trn_lock_score))
                        {
                            s->phase3_trn_lock_hyp = best_trn_h;
                            s->phase3_trn_lock_score = score_pct;
                            s->phase3_j_lock_hyp = best_trn_h;
                            s->phase3_tracking_armed = true;
                            span_log(s->logging, SPAN_LOG_FLOW,
                                     "Rx - Phase 3 TRN: lock hint hyp=%d ones=%d/%d (%d%%)\n",
                                     best_trn_h, best_trn_score, s->phase3_trn_bits, score_pct);
                        }
                        else if ((s->phase3_trn_bits % 512) == 0)
                        {
                            span_log(s->logging, SPAN_LOG_FLOW,
                                     "Rx - Phase 3 TRN: best hyp=%d ones=%d/%d (%d%%)\n",
                                     best_trn_h, best_trn_score, s->phase3_trn_bits, score_pct);
                        }
                        /*endif*/
                    }
                    /*endif*/
                }
                /*endif*/
            }

            idx = (s->duration - 1) & 31;
            int old_sym = s->phase3_s_ring[idx] & 3;
            previous_symbol = (s->duration > 1) ? s->phase3_s_ring[(idx + 31) & 31] & 3 : -1;
            prior2_symbol = (s->duration > 2) ? s->phase3_s_ring[(idx + 30) & 31] & 3 : -1;

            if (s->duration > 32)
                s->phase3_s_counts[old_sym]--;
            /*endif*/
            s->phase3_s_ring[idx] = (uint8_t) (data_bits & 3);
            s->phase3_s_mag_ring[idx] = sqrtf(sym->re*sym->re + sym->im*sym->im);
            s->phase3_s_counts[data_bits & 3]++;
            dominant_symbol = 0;
            dominant_count = s->phase3_s_counts[0];
            if (s->phase3_s_counts[1] > dominant_count)
            {
                dominant_count = s->phase3_s_counts[1];
                dominant_symbol = 1;
            }
            if (s->phase3_s_counts[2] > dominant_count)
            {
                dominant_count = s->phase3_s_counts[2];
                dominant_symbol = 2;
            }
            if (s->phase3_s_counts[3] > dominant_count)
            {
                dominant_count = s->phase3_s_counts[3];
                dominant_symbol = 3;
            }

            /* V.34 section 10.1.3.7 defines S as alternating between point 0
               and that point rotated by 90 degrees.  Some peers render that as
               an A,B,A,B alternation (caught below by phase3_s_alt_count);
               others (SmartLink d-modem, confirmed live) render S/S-bar as a
               steady +/-90 degrees/symbol rotation, i.e. one dominant
               differential dibit held for the whole 128T signal.  A *bare*
               dominant-symbol detector would false-fire on the short dominant
               runs inside scrambled Ja, but those never exceed ~10 bauds,
               whereas S holds >=72; a *sustained* dominant ±90 dibit
               (phase3_s_dom_windows below) is therefore an unambiguous S. */
            old_alt = (s->duration > 32) ? ((s->phase3_s_alt_window >> idx) & 1) : 0;
            new_alt = (prior2_symbol >= 0
                       && data_bits != previous_symbol
                       && data_bits == prior2_symbol) ? 1 : 0;
            if (new_alt)
                s->phase3_s_alt_window |= (1u << idx);
            else
                s->phase3_s_alt_window &= ~(1u << idx);
            s->phase3_s_alt_count += new_alt - old_alt;
            s->s_detect_count = s->phase3_s_alt_count;

            if (s->duration >= 32  &&  s->phase3_s_alt_count >= PHASE3_S_ALTERNATING_MIN)
                s->phase3_s_stable_windows++;
            else
                s->phase3_s_stable_windows = 0;

            /* Sustained-rotation tracking: count how long one +/-90 dibit
               (dibit 1 or 3) dominates the 32-baud window.  A change of the
               dominant dibit (the S-to-S-bar rotation flip) or loss of
               dominance restarts the run.

               NOTE (2026-07-24): this rotation detector false-fires on any
               sustained single-frequency tone, because a pure tone differen-
               tially demodulates to a constant dibit.  Live against a USR
               Courier it fired on our own ~1325 Hz DIL echo (RX RMS ~132) and
               on the peer's 2400 Hz Tone A retrain (RMS ~3976) -- neither is a
               real far-end S.  Raw input power does NOT cleanly separate them
               (echo-tone S ~61k..275k overlaps real-signal power), so an energy
               gate is not a sufficient fix; the far-end-S log below now carries
               power/carrier_ref to support a better discriminator. */
            if (s->duration >= 32
                &&  dominant_count >= PHASE3_S_DOMINANT_MIN
                &&  (dominant_symbol == 1  ||  dominant_symbol == 3))
            {
                if (dominant_symbol == s->phase3_s_dom_symbol)
                    s->phase3_s_dom_windows++;
                else
                {
                    s->phase3_s_dom_symbol = dominant_symbol;
                    s->phase3_s_dom_windows = 1;
                }
                /*endif*/
            }
            else
            {
                s->phase3_s_dom_windows = 0;
                s->phase3_s_dom_symbol = -1;
            }
            /*endif*/

            /* Independent reversal detector:
               count bauds where current symbol is close to 180° from previous. */
            mag_now = sqrtf(sym->re * sym->re + sym->im * sym->im);
            mag_prev = sqrtf(s->last_sample.re * s->last_sample.re
                             + s->last_sample.im * s->last_sample.im);
            dot = sym->re*s->last_sample.re + sym->im*s->last_sample.im;
            old_rev = (s->duration > 32) ? ((s->s_window >> idx) & 1) : 0;
            new_rev = (mag_now > 0.2f  &&  mag_prev > 0.2f  &&  dot < -0.15f*mag_now*mag_prev) ? 1 : 0;
            if (new_rev)
                s->s_window |= (1u << idx);
            else
                s->s_window &= ~(1u << idx);
            s->bit_count += new_rev - old_rev;  /* reuse bit_count as reversal window count */

        if (s->duration <= 4 || (s->duration % PHASE3_S_BAUD_LOG_INTERVAL) == 0)
        {
            span_log(s->logging, SPAN_LOG_FLOW,
                     "Rx - Phase 3 S baud %d: mag=%.3f data_bits=%d alt=%d/32 stable=%d dom=%d:%d/32 rev=%d/32 counts=%d,%d,%d,%d\n",
                     s->duration, mag_now, data_bits,
                     s->phase3_s_alt_count, s->phase3_s_stable_windows,
                     dominant_symbol, dominant_count, s->bit_count,
                     s->phase3_s_counts[0], s->phase3_s_counts[1],
                     s->phase3_s_counts[2], s->phase3_s_counts[3]);
        }

        if ((s->calling_party || (s->v90_mode && !s->calling_party))
            && s->phase3_s_detect_armed
            && !s->phase3_s_present
            && s->duration >= 64
            && ((s->phase3_s_alt_count >= PHASE3_S_ALTERNATING_MIN
                 && s->phase3_s_stable_windows >= PHASE3_S_STABLE_WINDOWS)
                || s->phase3_s_dom_windows >= PHASE3_S_DOMINANT_STABLE))
        {
            bool by_rotation = (s->phase3_s_dom_windows >= PHASE3_S_DOMINANT_STABLE);

            s->phase3_s_present = true;
            s->phase3_s_event_count++;
            s->phase3_s_fired_symbol = by_rotation ? s->phase3_s_dom_symbol : dominant_symbol;
            s->received_event = V34_EVENT_S;
            span_log(s->logging, SPAN_LOG_FLOW,
                     "Rx - Phase 3: far-end S detected after J decode (count=%d via=%s role=%s alt=%d/32 stable=%d dom=%d:%d/32 domwin=%d rev=%d/32 bits=%d trn=%s power=%ld carrier_ref=%ld)\n",
                     s->phase3_s_event_count,
                     by_rotation ? "rotation" : "alternation",
                     s->calling_party ? "caller" : "V.90 digital answerer",
                     s->phase3_s_alt_count, s->phase3_s_stable_windows,
                     dominant_symbol, dominant_count, s->phase3_s_dom_windows,
                     s->bit_count, s->phase3_j_bits,
                     s->phase3_j_trn16 ? "16-point" : "4-point",
                     (long) power_meter_current(&s->power),
                     (long) s->info_rx_carrier_ref);
        }

        /* Rearm only after the current S has genuinely ended, so V.90 can
           count the later DIL-termination S / the S-to-S-bar transition as a
           fresh event without double-counting one long S signal:
             - alternation form: the A,B pattern faded and no rotation took
               over;
             - rotation form: the dominant +/-90 dibit flipped (S -> S-bar) to
               the opposite rotation. */
        if (s->phase3_s_present && s->duration >= 96
            && ((s->phase3_s_alt_count <= 12 && s->phase3_s_dom_windows == 0)
                || (s->phase3_s_fired_symbol >= 0
                    && s->phase3_s_dom_symbol >= 0
                    && s->phase3_s_dom_symbol != s->phase3_s_fired_symbol)))
        {
            s->phase3_s_present = false;
            s->phase3_s_stable_windows = 0;
            s->phase3_s_fired_symbol = -1;
            span_log(s->logging, SPAN_LOG_FLOW,
                     "Rx - Phase 3: S detector rearmed after transition %d\n",
                     s->phase3_s_event_count);
        }

        /* Keep the decoded Ja/J state while renewing only the S observation
           window.  S is defined as alternating points separated by 90 degrees,
           so its differential dibit is dominant; scrambled Ja has no such
           dominant dibit even when it happens to contain many 180-degree
           reversals. */
        if (s->duration >= 6000)
        {
            /* Don't force a false S event; keep searching for a real pattern. */
            span_log(s->logging, SPAN_LOG_FLOW,
                     "Rx - Phase 3: S detect timeout (%d bauds, alt=%d/32 rev=%d/32, j_bits=%d trn_bits=%d trn_hint=%d/%d%%), continuing search\n",
                     s->duration, s->phase3_s_alt_count, s->bit_count,
                     s->phase3_j_bits, s->phase3_trn_bits,
                     s->phase3_trn_lock_hyp, s->phase3_trn_lock_score);
            s->duration = 0;
            s->s_detect_count = 0;
            s->bit_count = 0;
            s->s_window = 0;
            s->phase3_s_alt_window = 0;
            s->phase3_s_alt_count = 0;
            s->phase3_s_stable_windows = 0;
            s->phase3_s_dom_windows = 0;
            s->phase3_s_dom_symbol = -1;
            memset(s->phase3_s_ring, 0, sizeof(s->phase3_s_ring));
            memset(s->phase3_s_mag_ring, 0, sizeof(s->phase3_s_mag_ring));
            memset(s->phase3_s_counts, 0, sizeof(s->phase3_s_counts));
            s->phase3_s_pos = 0;
        }
        }
        break;

    case V34_RX_STAGE_PHASE3_TRAINING:
        /* Phase 3 receiver conditioning:
           - acquire the PP start/phase against the known 48-symbol PP sequence
           - adapt equalizer directly against the aligned PP sequence
           - optionally refine for only the first 512T of TRN
           - then hold the equalizer steady until J/S handling takes over */
        {
        v34_state_t *t;

        t = ((v34_state_t *) ((char *)(s) - offsetof(v34_state_t, rx)));
        /* Differential symbols must be measured in one consistent domain.
           last_sample is the previous equalizer output, so using the newest
           raw T/2 input here compared unrelated points and made TRN/Ja bits
           random even when the equalizer itself was usable. */
        ang1 = arctan2(sym->im, sym->re);
        ang2 = arctan2(s->last_sample.im, s->last_sample.re);
        ang3 = ang1 - ang2 + DDS_PHASE(45.0f);
        data_bits = (ang3 >> 30) & 0x3;
        phase3_abs_bits = (int) ((ang1 + DDS_PHASE(45.0f)) >> 30) & 0x3;
        s->duration++;
        if (!s->phase3_pp_started)
        {
            int phase;
            int best_phase;
            float best_score;
            float next_score;
            float mag;

            mag = sqrtf(sym->re * sym->re + sym->im * sym->im);
            s->phase3_pp_corr_energy =
                PHASE3_PP_ACQUIRE_DECAY*s->phase3_pp_corr_energy + mag*mag;
            s->phase3_pp_corr_weight =
                PHASE3_PP_ACQUIRE_DECAY*s->phase3_pp_corr_weight + 1.0f;
            best_phase = 0;
            best_score = 0.0f;
            next_score = 0.0f;
            for (phase = 0;  phase < PP_PERIOD_SYMBOLS;  phase++)
            {
                complexf_t cand;
                float corr_mag;
                float denom;
                float score;

                cand = pp_symbols[(s->duration - 1 + phase)%PP_PERIOD_SYMBOLS];
                /* Correlate y with the conjugate of the known unit-magnitude
                   PP reference.  |correlation| is invariant to the arbitrary
                   carrier phase present when the primary demodulator starts. */
                s->phase3_pp_corr[phase].re =
                    PHASE3_PP_ACQUIRE_DECAY*s->phase3_pp_corr[phase].re
                    + sym->re*cand.re + sym->im*cand.im;
                s->phase3_pp_corr[phase].im =
                    PHASE3_PP_ACQUIRE_DECAY*s->phase3_pp_corr[phase].im
                    + sym->im*cand.re - sym->re*cand.im;
                corr_mag = sqrtf(s->phase3_pp_corr[phase].re*s->phase3_pp_corr[phase].re
                               + s->phase3_pp_corr[phase].im*s->phase3_pp_corr[phase].im);
                denom = sqrtf(s->phase3_pp_corr_energy*s->phase3_pp_corr_weight);
                score = (denom > 0.0001f) ? corr_mag/denom : 0.0f;
                s->phase3_pp_error[phase] = score;
                if (phase == 0 || score > best_score)
                {
                    next_score = best_score;
                    best_score = score;
                    best_phase = phase;
                }
                else if (score > next_score)
                {
                    next_score = score;
                }
                /*endif*/
            }
            /*endfor*/

            if (s->phase3_pp_phase == best_phase)
                s->phase3_pp_acquire_hits++;
            else
                s->phase3_pp_acquire_hits = 1;
            /*endif*/
            s->phase3_pp_phase = best_phase;
            s->phase3_pp_phase_score = (int) lrintf(1000.0f*best_score);

            if (s->duration <= 4 || (s->duration % PHASE3_PP_ACQUIRE_LOG_INTERVAL) == 0)
            {
                span_log(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 3 PP acquire baud %d: mag=%.3f data_bits=%d phase=%d score=%d hold=%d/%d\n",
                         s->duration, mag, data_bits,
                         s->phase3_pp_phase, s->phase3_pp_phase_score,
                         s->phase3_pp_acquire_hits, PHASE3_PP_ACQUIRE_HOLD_BAUDS);
            }
            /*endif*/

            if (s->duration >= PHASE3_PP_ACQUIRE_MIN_BAUDS
                && s->phase3_pp_phase_score >= PHASE3_PP_ACQUIRE_SCORE_MIN
                && s->phase3_pp_acquire_hits >= PHASE3_PP_ACQUIRE_HOLD_BAUDS)
            {
                int acquire_bauds;
                /* Do not arm PP conditioning while local Phase 3 TX is still in S.
                   In practice this avoids false PP locks on the S/!S interval.
                   However, in the external V.90 downstream path the local Phase 3
                   symbols are no longer driven by SpanDSP TX stage progression,
                   so this internal TX-stage guard can deadlock RX in
                   PHASE3_TRAINING forever. In V.90 mode, trust the Phase 3 RX
                   evidence and allow PP acquisition to proceed. */
                if (!s->v90_mode && t->tx.stage < V34_TX_STAGE_FIRST_NOT_S)
                {
                    goto phase3_training_done;
                }
                /*endif*/

                acquire_bauds = s->duration;
                s->phase3_pp_started = 1;
                {
                    float corr_mag;

                    corr_mag = sqrtf(s->phase3_pp_corr[s->phase3_pp_phase].re
                                   * s->phase3_pp_corr[s->phase3_pp_phase].re
                                   + s->phase3_pp_corr[s->phase3_pp_phase].im
                                   * s->phase3_pp_corr[s->phase3_pp_phase].im);
                    if (corr_mag > 0.0001f)
                    {
                        s->phase3_pp_rotation.re =
                            s->phase3_pp_corr[s->phase3_pp_phase].re/corr_mag;
                        s->phase3_pp_rotation.im =
                            s->phase3_pp_corr[s->phase3_pp_phase].im/corr_mag;
                    }
                    /*endif*/
                }
                /* Absorb the acquisition baud count into the phase offset so that
                   when duration resets to 0 the PP target index remains continuous:
                   during acquisition baud N, target = pp_symbols[(N-1+phase)%48].
                   After reset, conditioning baud 1 must use the same index as
                   acquisition baud (N+1), i.e. pp_symbols[(N+phase)%48].
                   With new_phase = (phase + acquire_bauds) % 48 and dur=1:
                   pp_symbols[(1-1+new_phase)%48] = pp_symbols[(acquire_bauds+phase)%48]. ✓ */
                s->phase3_pp_phase = (s->phase3_pp_phase + acquire_bauds) % PP_PERIOD_SYMBOLS;
                if (getenv("VPCM_V90_PP_PHASE"))
                {
                    int forced_phase = atoi(getenv("VPCM_V90_PP_PHASE"));

                    forced_phase %= PP_PERIOD_SYMBOLS;
                    if (forced_phase < 0)
                        forced_phase += PP_PERIOD_SYMBOLS;
                    s->phase3_pp_phase = forced_phase;
                }
                /*endif*/
                s->duration = 0;
                memset(s->phase3_pp_lag8, 0, sizeof(s->phase3_pp_lag8));
                s->phase3_pp_obs = 0;
                s->phase3_pp_match = 0;
                /* Freeze AGC — the equalizer will now be conditioned with known
                   PP symbols at the current magnitude.  Freezing prevents AGC from
                   fighting the LMS equalizer adaptation. */
                s->agc_scaling_save = s->agc_scaling;
                span_log(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 3: PP start detected (phase=%d score=%d after %d bauds), starting supervised PP conditioning (agc frozen at %.6f)\n",
                         s->phase3_pp_phase, s->phase3_pp_phase_score, acquire_bauds, s->agc_scaling);
            }
            /*endif*/
        }
        else if (s->duration <= PHASE3_PP_TRAIN_BAUDS)
        {
            complexf_t pp_target;
            int idx;
            int prev;
            int pp_baud;

            pp_baud = s->duration;
            idx = (pp_baud - 1) & 7;
            prev = s->phase3_pp_lag8[idx];
            if (pp_baud > 8)
            {
                s->phase3_pp_obs++;
                if (data_bits == prev)
                    s->phase3_pp_match++;
                /*endif*/
            }
            /*endif*/
            s->phase3_pp_lag8[idx] = (uint8_t) data_bits;

            /* Scale PP target to match the actual equalizer output magnitude.
               The AGC + center-tap equalizer produces output at some magnitude
               that depends on signal power at AGC freeze time.  Track it with
               an EMA (stored in eq_target_mag) and scale the unit-magnitude PP
               reference to match, so LMS drives direction without fighting magnitude. */
            {
                float sym_mag = sqrtf(sym->re*sym->re + sym->im*sym->im);
                float scale;

                if (!isfinite(sym_mag) || sym_mag <= 0.0f || sym_mag > PHASE3_PP_MAG_SANITY_MAX)
                {
                    span_log(s->logging, SPAN_LOG_FLOW,
                             "Rx - Phase 3: PP conditioning symbol magnitude out of range (mag=%.3f), clamping target scale and resetting equalizer\n",
                             sym_mag);
                    sym_mag = 1.0f;
                    equalizer_reset(s);
                }
                /*endif*/
                if (pp_baud <= 1)
                    s->eq_target_mag = sym_mag;
                else
                    s->eq_target_mag = 0.95f*s->eq_target_mag + 0.05f*sym_mag;
                /*endif*/
                scale = (s->eq_target_mag > 0.01f) ? s->eq_target_mag : 1.0f;
                pp_target = pp_symbols[(pp_baud - 1 + s->phase3_pp_phase)%PP_PERIOD_SYMBOLS];
                {
                    float re;

                    re = pp_target.re*s->phase3_pp_rotation.re
                       - pp_target.im*s->phase3_pp_rotation.im;
                    pp_target.im = pp_target.re*s->phase3_pp_rotation.im
                                 + pp_target.im*s->phase3_pp_rotation.re;
                    pp_target.re = re;
                }
                pp_target.re *= scale;
                pp_target.im *= scale;
                tune_equalizer(s, sym, &pp_target);
            }

            if (pp_baud == 1)
            {
                span_log(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 3: conditioning on aligned PP sequence (%d bauds)\n",
                         PHASE3_PP_TRAIN_BAUDS);
            }
            /*endif*/
            if (pp_baud <= 4 || (pp_baud % PHASE3_PP_BAUD_LOG_INTERVAL) == 0)
            {
                float mag = sqrtf(sym->re * sym->re + sym->im * sym->im);
                float pct = (s->phase3_pp_obs > 0)
                            ? (100.0f*s->phase3_pp_match/(float) s->phase3_pp_obs)
                            : 0.0f;
                span_log(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 3 PP baud %d: mag=%.3f data_bits=%d lag8=%d/%d (%.1f%%) phase=%d score=%d\n",
                         pp_baud, mag, data_bits,
                         s->phase3_pp_match, s->phase3_pp_obs, pct,
                         s->phase3_pp_phase, s->phase3_pp_phase_score);
            }
            /*endif*/
            if (pp_baud == PHASE3_PP_TRAIN_BAUDS)
            {
                float pct = (s->phase3_pp_obs > 0)
                            ? (100.0f*s->phase3_pp_match/(float) s->phase3_pp_obs)
                            : 0.0f;
                span_log(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 3: PP conditioning complete (lag8=%d/%d, %.1f%%, phase=%d, score=%d), refining with first %dT of TRN\n",
                         s->phase3_pp_match, s->phase3_pp_obs, pct,
                         s->phase3_pp_phase, s->phase3_pp_phase_score, PHASE3_TRN_REFINE_BAUDS);
            }
            /*endif*/
        }
        else if (s->duration <= (PHASE3_PP_TRAIN_BAUDS + PHASE3_TRN_REFINE_BAUDS))
        {
            int h;
            int best_trn_h;
            int best_trn_score;
            int trn_refine_baud;

            best_trn_h = -1;
            best_trn_score = -1;
            trn_refine_baud = s->duration - PHASE3_PP_TRAIN_BAUDS;
            for (h = 0;  h < MP_HYPOTHESIS_COUNT;  h++)
            {
                int raw_sym;
                uint32_t reg;
                int d0;
                int d1;

                /* V.34 10.1.3.8 TRN uses direct (absolute) mapping. */
                raw_sym = map_phase4_raw_bits(phase3_abs_bits, h);
                reg = s->phase3_trn_scramble[h];
                d0 = descramble_reg(&reg, s->scrambler_tap, raw_sym & 1);
                d1 = descramble_reg(&reg, s->scrambler_tap, (raw_sym >> 1) & 1);
                s->phase3_trn_scramble[h] = reg;
                s->phase3_trn_one_count[h] += (uint16_t) (d0 + d1);
                if (s->phase3_trn_one_count[h] > best_trn_score)
                {
                    best_trn_h = h;
                    best_trn_score = s->phase3_trn_one_count[h];
                }
                /*endif*/
            }
            /*endfor*/
            s->phase3_trn_bits += 2;
            {
                float trn_mag = sqrtf(sym->re*sym->re + sym->im*sym->im);
                if (isfinite(trn_mag) && trn_mag > 0.0f) {
                    s->phase3_trn_mag_sum += trn_mag;
                    s->phase3_trn_mag_count++;
                }
            }
            if (trn_refine_baud == 1)
            {
                span_log(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 3: PP complete, using first %dT of TRN for equalizer refinement\n",
                         PHASE3_TRN_REFINE_BAUDS);
            }
            /*endif*/
            if (s->phase3_trn_bits >= 256  &&  best_trn_h >= 0)
            {
                int score_pct;

                score_pct = (100*best_trn_score + (s->phase3_trn_bits/2))/s->phase3_trn_bits;
                if (score_pct >= 70
                    &&
                    (s->phase3_trn_lock_hyp < 0  ||  score_pct > s->phase3_trn_lock_score))
                {
                    s->phase3_trn_lock_hyp = best_trn_h;
                    s->phase3_trn_lock_score = score_pct;
                    s->phase3_j_lock_hyp = best_trn_h;
                    span_log(s->logging, SPAN_LOG_FLOW,
                             "Rx - Phase 3 TRN refine: lock hint hyp=%d ones=%d/%d (%d%%)\n",
                             best_trn_h, best_trn_score, s->phase3_trn_bits, score_pct);
                }
                else if ((s->phase3_trn_bits % 512) == 0)
                {
                    span_log(s->logging, SPAN_LOG_FLOW,
                             "Rx - Phase 3 TRN refine: best hyp=%d ones=%d/%d (%d%%)\n",
                             best_trn_h, best_trn_score, s->phase3_trn_bits, score_pct);
                }
                /*endif*/
            }
            /*endif*/
            if (trn_refine_baud == PHASE3_TRN_REFINE_BAUDS)
            {
                span_log(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 3: first %dT of TRN processed; equalizer frozen, waiting for J-handling stage\n",
                         PHASE3_TRN_REFINE_BAUDS);
                s->stage = V34_RX_STAGE_PHASE3_WAIT_S;
                s->duration = 0;
                s->s_detect_count = 0;
                s->bit_count = 0;
                s->s_window = 0;
                s->phase3_s_alt_window = 0;
                s->phase3_s_alt_count = 0;
                s->phase3_s_stable_windows = 0;
                s->phase3_s_dom_windows = 0;
                s->phase3_s_dom_symbol = -1;
                s->phase3_s_fired_symbol = -1;
                s->phase3_s_detect_armed = false;
                memset(s->phase3_s_ring, 0, sizeof(s->phase3_s_ring));
                memset(s->phase3_s_mag_ring, 0, sizeof(s->phase3_s_mag_ring));
                memset(s->phase3_s_counts, 0, sizeof(s->phase3_s_counts));
                s->phase3_s_pos = 0;
                memset(s->phase3_j_scramble, 0, sizeof(s->phase3_j_scramble));
                memset(s->phase3_j_stream, 0, sizeof(s->phase3_j_stream));
                memset(s->phase3_j_prev_z, 0, sizeof(s->phase3_j_prev_z));
                memset(s->phase3_j_prev_valid, 0, sizeof(s->phase3_j_prev_valid));
                memset(s->phase3_j_win, 0, sizeof(s->phase3_j_win));
                s->phase3_j_bits = 0;
                s->phase3_j_lock_hyp = -1;
                s->phase3_j_trn16 = -1;
                s->phase3_j_candidate_hyp = -1;
                s->phase3_j_candidate_phase = -1;
                s->phase3_j_candidate_pat = -1;
                s->phase3_j_candidate_count = 0;
                s->phase3_j_candidate_last_bits = 0;
                memset(s->phase3_ja_scramble, 0, sizeof(s->phase3_ja_scramble));
                memset(s->phase3_ja_prev_z, 0, sizeof(s->phase3_ja_prev_z));
                memset(s->phase3_ja_prev_valid, 0, sizeof(s->phase3_ja_prev_valid));
                s->phase3_ja_bits = 0;
                s->phase3_ja_hyp = -1;
                memset(s->phase3_ja_capture, 0, sizeof(s->phase3_ja_capture));
                s->phase3_ja_capture_len = 0;
                memset(s->phase3_ja_capture_hyp, 0, sizeof(s->phase3_ja_capture_hyp));
                memset(s->phase3_ja_capture_hyp_len, 0, sizeof(s->phase3_ja_capture_hyp_len));
                memset(s->phase3_ja_capture_hyp_raw, 0, sizeof(s->phase3_ja_capture_hyp_raw));
                memset(s->phase3_ja_capture_hyp_raw_len, 0, sizeof(s->phase3_ja_capture_hyp_raw_len));
                phase3_trn_hyp_reset(s);
                s->phase3_trn_mag_sum = 0.0f;
                s->phase3_trn_mag_count = 0;
            }
            /*endif*/
        }
        else
        {
            int h;
            int raw_sym;
            int lock_h = (s->phase3_trn_lock_hyp >= 0
                          && s->phase3_trn_lock_hyp < MP_HYPOTHESIS_COUNT)
                         ? s->phase3_trn_lock_hyp
                         : s->phase3_j_lock_hyp;

            if (lock_h < 0 || lock_h >= MP_HYPOTHESIS_COUNT)
                goto phase3_training_done;

            h = lock_h;
            raw_sym = map_phase4_raw_bits(data_bits, h);
            if (s->phase3_ja_prev_valid[h])
            {
                int in_sym;
                uint32_t reg;
                int b0;
                int b1;

                in_sym = raw_sym;
                reg = s->phase3_ja_scramble[h];
                b0 = descramble_reg(&reg, s->scrambler_tap, in_sym & 1);
                b1 = descramble_reg(&reg, s->scrambler_tap, (in_sym >> 1) & 1);
                s->phase3_ja_scramble[h] = reg;
                if (s->put_aux_bit)
                {
                    s->put_aux_bit(s->put_aux_bit_user_data, b0);
                    s->put_aux_bit(s->put_aux_bit_user_data, b1);
                }
                /* Persist Ja bits for offline analyzers even when no aux callback is armed. */
                if (s->phase3_ja_capture_len + 2 <= (int) sizeof(s->phase3_ja_capture))
                {
                    s->phase3_ja_capture[s->phase3_ja_capture_len++] = (uint8_t) (b0 & 1);
                    s->phase3_ja_capture[s->phase3_ja_capture_len++] = (uint8_t) (b1 & 1);
                }
                s->phase3_ja_bits += 2;
                s->phase3_ja_hyp = h;
                if (s->phase3_ja_bits == 2 || (s->phase3_ja_bits % 256) == 0)
                {
                    span_log(s->logging, SPAN_LOG_FLOW,
                             "Rx - Phase 3 Ja capture: emitted %d bits using hyp=%d\n",
                             s->phase3_ja_bits, h);
                }
                /*endif*/
            }
            /*endif*/
            s->phase3_ja_prev_z[h] = (uint8_t) raw_sym;
            s->phase3_ja_prev_valid[h] = 1;
        }
        phase3_training_done:
        ;
        }
        break;

    case V34_RX_STAGE_PHASE3_DONE:
        break;

    case V34_RX_STAGE_PHASE4_S:
        /* Phase 4: Detect the caller's S signal (constant 180° phase reversals).
           S is data_bits=2 for every baud.  Due to imperfect carrier recovery,
           we may see errors (~1 in 3 bauds).  Use a window-based detector:
           count data_bits=2 in last 32 bauds.  S detected when count >= 20/32.
           After S is confirmed, watch for a sustained drop (S→S-bar transition). */
        ang1 = arctan2(sym->im, sym->re);
        ang2 = arctan2(s->last_sample.im, s->last_sample.re);
        ang3 = ang1 - ang2 + DDS_PHASE(45.0f);
        data_bits = (ang3 >> 30) & 0x3;
        s->duration++;

        /* Sliding window: shift in new bit, shift out old */
        {
            int idx = (s->duration - 1) & 31;  /* circular index 0-31 */
            int old_was_2 = (s->duration > 32) ? ((s->s_window >> idx) & 1) : 0;
            int new_is_2 = (data_bits == 2) ? 1 : 0;

            if (new_is_2)
                s->s_window |= (1u << idx);
            else
                s->s_window &= ~(1u << idx);

            s->s_detect_count += new_is_2 - old_was_2;
        }

        if (s->duration <= 10 || (s->duration % 500) == 0)
        {
            float mag = sqrtf(sym->re * sym->re + sym->im * sym->im);
            span_log(s->logging, SPAN_LOG_FLOW,
                     "Rx - Phase 4 S baud %d: mag=%.3f data_bits=%d win=%d/32\n",
                     s->duration, mag, data_bits, s->s_detect_count);
        }
        /* Dump raw I/Q constellation for first 32 bauds to diagnose signal quality */
        #if V34_DEBUG_IQ_LOG
        if (s->duration <= 32)
        {
            uint32_t ang_abs = arctan2(sym->im, sym->re);
            float deg = (float)ang_abs / (4294967296.0f / 360.0f);
            float deg_diff = (float)ang3 / (4294967296.0f / 360.0f);
            fprintf(stderr, "[IQ] baud=%d re=%.4f im=%.4f ang=%.1f diff=%.1f data=%d\n",
                    s->duration, sym->re, sym->im, deg, deg_diff, data_bits);
        }
        #endif

        if (s->duration >= 128 && s->s_detect_count >= 20)
        {
            /* S signal confirmed.  Now transition to S-bar detection.
               We skip explicit S-bar detection since we can't reliably
               distinguish S-bar from S with this demodulator quality.
               Instead, wait a fixed time for the caller's S-bar(16T) + TRN(≥512T)
               to pass, then go straight to MP detection. */
            span_log(s->logging, SPAN_LOG_FLOW,
                     "Rx - Phase 4: S signal confirmed (baud %d, win=%d/32), "
                     "waiting for S-bar + TRN\n",
                     s->duration, s->s_detect_count);
            s->stage = V34_RX_STAGE_PHASE4_TRN;
            s->duration = 0;
            s->scramble_reg = 0;
            phase4_j_detector_reset(s);
            phase4_trn_hyp_reset(s);
            if (s->calling_party)
            {
                /* Caller-side Phase 4 does not wait for a far-end J':
                   after detecting the answerer's S/S-bar handoff, the far end
                   is already in TRN. Mark the J' gate as satisfied so Phase 4
                   TRN scoring can begin immediately instead of timing out while
                   waiting for a symbol pattern that the answerer never sends. */
                s->phase4_j_seen = 1;
                s->phase4_trn_after_j = 0;
                s->phase4_j_lock_hyp = s->phase3_j_lock_hyp;
                span_log(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 4: caller detected answerer S handoff; starting direct TRN scoring (phase3 hyp=%d)\n",
                         s->phase3_j_lock_hyp);
            }
            /*endif*/
        }
        else if (s->duration >= 2048)
        {
            /* If S wasn't confidently detected within a long guard interval,
               don't stall forever in Phase 4 S detection. Advance to TRN/MP
               search so the handshake can continue. */
            span_log(s->logging, SPAN_LOG_FLOW,
                     "Rx - Phase 4: S detect timeout (%d bauds, win=%d/32), "
                     "forcing TRN/MP search\n",
                     s->duration, s->s_detect_count);
            s->stage = V34_RX_STAGE_PHASE4_TRN;
            s->duration = 0;
            s->scramble_reg = 0;
            phase4_j_detector_reset(s);
            phase4_trn_hyp_reset(s);
            if (s->calling_party)
            {
                s->phase4_j_seen = 1;
                s->phase4_trn_after_j = 0;
                s->phase4_j_lock_hyp = s->phase3_j_lock_hyp;
                span_log(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 4: caller forcing direct TRN scoring after S timeout (phase3 hyp=%d)\n",
                         s->phase3_j_lock_hyp);
            }
            /*endif*/
        }
        break;

    case V34_RX_STAGE_PHASE4_S_BAR:
        /* Phase 4: S-bar is 16T. After S-bar, TRN begins. */
        ang1 = arctan2(sym->im, sym->re);
        ang2 = arctan2(s->last_sample.im, s->last_sample.re);
        ang3 = ang1 - ang2 + DDS_PHASE(45.0f);
        data_bits = (ang3 >> 30) & 0x3;
        s->duration++;

        if (s->duration >= 16)
        {
            span_log(s->logging, SPAN_LOG_FLOW,
                     "Rx - Phase 4: S-bar complete (%d bauds), starting TRN detection\n",
                     s->duration);
            s->stage = V34_RX_STAGE_PHASE4_TRN;
            s->duration = 0;
            s->scramble_reg = 0;
            phase4_j_detector_reset(s);
            phase4_trn_hyp_reset(s);
            if (s->calling_party)
            {
                s->phase4_j_seen = 1;
                s->phase4_trn_after_j = 0;
                s->phase4_j_lock_hyp = s->phase3_j_lock_hyp;
                span_log(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 4: caller finished S-bar; starting direct TRN scoring (phase3 hyp=%d)\n",
                         s->phase3_j_lock_hyp);
            }
            /*endif*/
        }
        break;

    case V34_RX_STAGE_PHASE4_TRN:
        /* Phase 4: gate MP entry on explicit far-end J' followed by >=512T TRN. */
        {
        int abs_bits;

        ang1 = arctan2(sym->im, sym->re);
        ang2 = arctan2(s->last_sample.im, s->last_sample.re);
        ang3 = ang1 - ang2 + DDS_PHASE(45.0f);
        data_bits = (ang3 >> 30) & 0x3;
        abs_bits = (int) ((ang1 + DDS_PHASE(45.0f)) >> 30) & 0x3;
        s->duration++;

        /* I/Q constellation diagnostic: log first 64 bauds of S+Sbar and first 64 bauds of TRN scoring */
        #if V34_DEBUG_IQ_LOG
        if (s->phase4_j_seen
            && (s->phase4_trn_after_j < 64
                || (s->phase4_trn_after_j >= PHASE4_TRN_SCORE_START_BAUD
                    && s->phase4_trn_after_j < PHASE4_TRN_SCORE_START_BAUD + 64)))
        {
            float deg_abs = (float)ang1 / (4294967296.0f / 360.0f);
            float deg_diff = (float)ang3 / (4294967296.0f / 360.0f);
            float mag_now = sqrtf(sym->re*sym->re + sym->im*sym->im);
            float deg_prev = (float)ang2 / (4294967296.0f / 360.0f);
            fprintf(stderr, "[IQ] baud=%d re=%.4f im=%.4f mag=%.3f abs=%.1f prev=%.1f diff=%.1f data=%d abs_bits=%d\n",
                    s->phase4_trn_after_j, sym->re, sym->im, mag_now, deg_abs, deg_prev, deg_diff, data_bits, abs_bits);
        }
        #endif

        /* Descramble to let the descrambler self-sync (needs ~23 bits) */
        {
            int raw_bits = data_bits;
            for (i = 0;  i < 2;  i++)
            {
                descramble(s, raw_bits & 1);
                raw_bits >>= 1;
            }
        }

        if (!s->phase4_j_seen)
        {
            int domain_idx;
            int tap_idx;
            int order_idx;
            int h;
            int best_h;
            int best_p;
            int best_score;
            int best_domain;
            int best_tap;
            int best_order;

            best_h = -1;
            best_p = 0;
            best_score = -1;
            best_domain = 0;
            best_tap = 0;
            best_order = 0;
            for (domain_idx = 0;  domain_idx < 2;  domain_idx++)
            {
                for (tap_idx = 0;  tap_idx < 2;  tap_idx++)
                {
                    int tap;

                    tap = phase4_trn_tap_value(tap_idx);
                    for (order_idx = 0;  order_idx < 2;  order_idx++)
                    {
                        for (h = 0;  h < 8;  h++)
                        {
                            int raw_sym;
                            int in_sym;
                            uint32_t reg;
                            int dbit[2];
                            int b;

                            raw_sym = map_phase4_raw_bits(domain_idx ? abs_bits : data_bits, h);
                            if (s->phase4_j_prev_valid_tap[domain_idx][tap_idx][order_idx][h])
                            {
                                in_sym = (raw_sym - s->phase4_j_prev_z_tap[domain_idx][tap_idx][order_idx][h]) & 0x3;
                                reg = s->phase4_j_scramble_tap[domain_idx][tap_idx][order_idx][h];
                                if (order_idx == 0)
                                {
                                    dbit[0] = descramble_reg(&reg, tap, in_sym & 1);
                                    dbit[1] = descramble_reg(&reg, tap, (in_sym >> 1) & 1);
                                }
                                else
                                {
                                    dbit[0] = descramble_reg(&reg, tap, (in_sym >> 1) & 1);
                                    dbit[1] = descramble_reg(&reg, tap, in_sym & 1);
                                }
                                /*endif*/
                                s->phase4_j_scramble_tap[domain_idx][tap_idx][order_idx][h] = reg;
                                s->phase4_j_stream_tap[domain_idx][tap_idx][order_idx][h] =
                                    ((s->phase4_j_stream_tap[domain_idx][tap_idx][order_idx][h] << 1) | (uint32_t) dbit[0]) & 0xFFFFFFFFU;
                                s->phase4_j_stream_tap[domain_idx][tap_idx][order_idx][h] =
                                    ((s->phase4_j_stream_tap[domain_idx][tap_idx][order_idx][h] << 1) | (uint32_t) dbit[1]) & 0xFFFFFFFFU;

                                for (b = 0;  b < 2;  b++)
                                {
                                    int bit_pos;
                                    int p;

                                    bit_pos = s->phase4_j_bits + b;
                                    for (p = 0;  p < 16;  p++)
                                    {
                                        int match;
                                        uint32_t w;
                                        int score;

                                        match = (dbit[b] == phase3_j_pattern_bit(2, bit_pos + p)) ? 1 : 0;
                                        w = s->phase4_j_win_tap[domain_idx][tap_idx][order_idx][h][p];
                                        w = ((w << 1) | match) & 0xFFFFFFFFU;
                                        s->phase4_j_win_tap[domain_idx][tap_idx][order_idx][h][p] = w;
                                        score = __builtin_popcount(w);
                                        if (score > best_score)
                                        {
                                            best_score = score;
                                            best_h = h;
                                            best_p = p;
                                            best_domain = domain_idx;
                                            best_tap = tap_idx;
                                            best_order = order_idx;
                                        }
                                        /*endif*/
                                    }
                                    /*endfor*/
                                }
                                /*endfor*/
                            }
                            /*endif*/
                            s->phase4_j_prev_z_tap[domain_idx][tap_idx][order_idx][h] = (uint8_t) raw_sym;
                            s->phase4_j_prev_valid_tap[domain_idx][tap_idx][order_idx][h] = 1;
                        }
                        /*endfor*/
                    }
                    /*endfor*/
                }
                /*endfor*/
            }
            /*endfor*/

            s->phase4_j_bits += 2;
            if (s->phase4_j_bits >= 16
                &&
                (s->phase4_j_bits == 16
                    ||
                    (s->phase4_j_bits % PHASE4_J_PROGRESS_LOG_INTERVAL) == 0))
            {
                span_log(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 4 J' detector: bits=%d best_score=%d/32 hyp=%d dom=%s tap=%d ord=%s\n",
                         s->phase4_j_bits, best_score, best_h,
                         phase4_trn_domain_name(best_domain),
                         phase4_trn_tap_value(best_tap),
                         phase4_trn_order_name(best_order));
            }
            /*endif*/

            if (s->phase4_j_bits >= 32
                && best_h >= 0
                && best_score >= 24)
            {
                int d4;
                int d16;
                int djd;
                int dmin;
                uint16_t rx_recent16;
                uint16_t rx_ordered16;
                char rx_recent_bits[17];
                char rx_ordered_bits[17];
                const char *j_validity;
                int canonical_ok;
                int emit_diag;

                rx_recent16 = (uint16_t) (s->phase4_j_stream_tap[best_domain][best_tap][best_order][best_h] & 0xFFFFU);
                rx_ordered16 = j_ordered16(rx_recent16, s->phase4_j_bits, best_p);
                bits16_to_str(rx_recent16, rx_recent_bits);
                bits16_to_str(rx_ordered16, rx_ordered_bits);
                d4 = __builtin_popcount((unsigned) (rx_ordered16 ^ 0x8990U));
                d16 = __builtin_popcount((unsigned) (rx_ordered16 ^ 0x89B0U));
                djd = __builtin_popcount((unsigned) (rx_ordered16 ^ 0x899FU));
                dmin = djd;
                if (d4 < dmin)
                    dmin = d4;
                if (d16 < dmin)
                    dmin = d16;
                canonical_ok = (djd <= 3);
                j_validity = canonical_ok ? "valid J'" : ((dmin <= 3) ? "valid non-J'" : "near/non-canonical");
                emit_diag = canonical_ok || ((s->phase4_j_bits % 16) == 0);
                if (emit_diag)
                {
                    span_log(s->logging, SPAN_LOG_FLOW,
                             "Rx - Phase 4 J bits: recent16=%s ordered16=%s phase=%d (%s, d4=%d d16=%d dj'=%d)\n",
                             rx_recent_bits, rx_ordered_bits, best_p, j_validity, d4, d16, djd);
                }
                /*endif*/
                if (canonical_ok)
                {
                    s->phase4_j_seen = 1;
                    s->phase4_trn_after_j = 0;
                    phase4_trn_hyp_reset(s);
                    s->phase4_j_lock_hyp = best_h;
                    /* Use role-based tap, not TRN auto-detected tap (see §7) */
                    s->scrambler_tap = s->calling_party ? 4 : 17;
                    s->received_event = V34_EVENT_J_DASHED;
                    span_log(s->logging, SPAN_LOG_FLOW,
                             "Rx - Phase 4: explicit J' detected (hyp=%d phase=%d score=%d/32 bits=%d dom=%s tap=%d ord=%s)\n",
                             best_h, best_p, best_score, s->phase4_j_bits,
                             phase4_trn_domain_name(best_domain),
                             phase4_trn_tap_value(best_tap),
                             phase4_trn_order_name(best_order));
                }
                /*endif*/
            }
            /*endif*/
        }
        else
        {
            int h;
            int best_h;
            int best_score;
            int best_tap;
            int best_order;
            int best_domain;
            int lock_raw_sym;

            s->phase4_trn_after_j++;
            best_h = -1;
            best_score = -1;
            best_tap = -1;
            best_order = -1;
            best_domain = -1;
            for (h = 0;  h < MP_HYPOTHESIS_COUNT;  h++)
            {
                /* Phase 4 after J' starts with S (128T) + S-bar (16T).
                   Do not pollute TRN ones-scoring with those non-TRN symbols. */
                if (s->phase4_trn_after_j >= PHASE4_TRN_SCORE_START_BAUD
                    && s->phase4_trn_prev_valid[h])
                {
                    int domain_idx;
                    int tap_idx;

                    /* TRN is not differentially encoded; score descrambled dibits directly.
                       Evaluate both domains (diff/abs), both complementary taps,
                       and both bit serialization orders in parallel. */
                    for (domain_idx = 0;  domain_idx < 2;  domain_idx++)
                    {
                        int raw_sym;

                        raw_sym = map_phase4_raw_bits(domain_idx ? abs_bits : data_bits, h);
                        for (tap_idx = 0;  tap_idx < 2;  tap_idx++)
                        {
                            int order_idx;

                            for (order_idx = 0;  order_idx < 2;  order_idx++)
                            {
                                uint32_t reg;
                                int d0;
                                int d1;
                                int tap;

                                tap = phase4_trn_tap_value(tap_idx);
                                reg = s->phase4_trn_scramble_tap[domain_idx][tap_idx][order_idx][h];
                                if (order_idx == 0)
                                {
                                    d0 = descramble_reg(&reg, tap, raw_sym & 1);
                                    d1 = descramble_reg(&reg, tap, (raw_sym >> 1) & 1);
                                }
                                else
                                {
                                    d1 = descramble_reg(&reg, tap, (raw_sym >> 1) & 1);
                                    d0 = descramble_reg(&reg, tap, raw_sym & 1);
                                }
                                /*endif*/
                                s->phase4_trn_scramble_tap[domain_idx][tap_idx][order_idx][h] = reg;
                                s->phase4_trn_one_count_tap[domain_idx][tap_idx][order_idx][h] += (uint16_t) (d0 + d1);
                                if (s->phase4_trn_one_count_tap[domain_idx][tap_idx][order_idx][h] > best_score)
                                {
                                    best_h = h;
                                    best_score = s->phase4_trn_one_count_tap[domain_idx][tap_idx][order_idx][h];
                                    best_tap = tap_idx;
                                    best_order = order_idx;
                                    best_domain = domain_idx;
                                }
                                /*endif*/
                            }
                            /*endfor*/
                        }
                        /*endif*/
                    }
                    /*endfor*/
                }
                /*endif*/
                s->phase4_trn_prev_valid[h] = 1;
            }
            /*endfor*/
            if (s->phase4_trn_lock_hyp >= 0
                && s->phase4_trn_lock_domain >= 0
                && s->phase4_trn_lock_tap >= 0
                && s->phase4_trn_lock_order >= 0)
            {
                lock_raw_sym = map_phase4_raw_bits(s->phase4_trn_lock_domain ? abs_bits : data_bits,
                                                   s->phase4_trn_lock_hyp);
                phase4_trn_recent_update(s, lock_raw_sym);
            }
            /*endif*/
            if (s->phase4_trn_after_j >= PHASE4_TRN_SCORE_START_BAUD)
            {
                int bits_observed;
                int score_pct;
                int scored_symbols;

                scored_symbols = s->phase4_trn_after_j - PHASE4_TRN_SCORE_START_BAUD + 1;
                bits_observed = 2*scored_symbols;
                if (bits_observed > 0  &&  best_h >= 0)
                {
                    int old_lock_hyp;
                    int old_lock_score;
                    int old_lock_tap;
                    int old_lock_order;
                    int old_lock_domain;
                    int lock_changed;
                    int lock_identity_changed;

                    score_pct = (100*best_score + (bits_observed/2))/bits_observed;
                    s->phase4_trn_current_hyp = best_h;
                    s->phase4_trn_current_score = score_pct;
                    s->phase4_trn_current_tap = best_tap;
                    s->phase4_trn_current_order = best_order;
                    s->phase4_trn_current_domain = best_domain;
                    old_lock_hyp = s->phase4_trn_lock_hyp;
                    old_lock_score = s->phase4_trn_lock_score;
                    old_lock_tap = s->phase4_trn_lock_tap;
                    old_lock_order = s->phase4_trn_lock_order;
                    old_lock_domain = s->phase4_trn_lock_domain;
                    lock_changed = 0;
                    lock_identity_changed = 0;

                    /* Keep the strongest sustained TRN candidate (with minimum
                       evidence), so later noisy segments do not overwrite it. */
                    if (bits_observed >= PHASE4_TRN_LOCK_MIN_BITS
                        && (s->phase4_trn_lock_hyp < 0
                            || score_pct > s->phase4_trn_lock_score))
                    {
                        s->phase4_trn_lock_hyp = best_h;
                        s->phase4_trn_lock_score = score_pct;
                        s->phase4_trn_lock_tap = best_tap;
                        s->phase4_trn_lock_order = best_order;
                        s->phase4_trn_lock_domain = best_domain;
                    }
                    /*endif*/
                    lock_changed = (s->phase4_trn_lock_hyp != old_lock_hyp
                                    || s->phase4_trn_lock_score != old_lock_score);
                    lock_identity_changed = (s->phase4_trn_lock_hyp != old_lock_hyp
                                             || s->phase4_trn_lock_tap != old_lock_tap
                                             || s->phase4_trn_lock_order != old_lock_order
                                             || s->phase4_trn_lock_domain != old_lock_domain);
                    if (lock_identity_changed)
                        phase4_trn_recent_seed(s);
                    /*endif*/
                    if (s->phase4_trn_recent_active
                        && s->phase4_trn_recent_window_bits > 0
                        && s->phase4_trn_lock_hyp >= 0)
                    {
                        s->phase4_trn_current_hyp = s->phase4_trn_lock_hyp;
                        s->phase4_trn_current_score = s->phase4_trn_recent_score;
                        s->phase4_trn_current_tap = s->phase4_trn_lock_tap;
                        s->phase4_trn_current_order = s->phase4_trn_lock_order;
                        s->phase4_trn_current_domain = s->phase4_trn_lock_domain;

                        /* Cumulative TRN score can be "poisoned" by early noisy
                           symbols and never recover above readiness threshold.
                           Promote a stable full-window recent score for the same
                           locked candidate to avoid stalling in TRN forever. */
                        if (s->phase4_trn_recent_window_fill >= PHASE4_TRN_RECENT_WINDOW_BAUDS
                            && s->phase4_trn_recent_score > s->phase4_trn_lock_score)
                        {
                            s->phase4_trn_lock_score = s->phase4_trn_recent_score;
                            lock_changed = 1;
                        }
                        /*endif*/
                    }
                    /*endif*/
                    if (lock_changed && s->phase4_trn_lock_score >= 70)
                    {
                        span_log(s->logging, SPAN_LOG_FLOW,
                                 "Rx - Phase 4 TRN: lock hint hyp=%d dom=%s tap=%d ord=%s ones=%d/%d (%d%%, recent=%d%%)\n",
                                 s->phase4_trn_lock_hyp,
                                 phase4_trn_domain_name(s->phase4_trn_lock_domain),
                                 phase4_trn_tap_value(s->phase4_trn_lock_tap),
                                 phase4_trn_order_name(s->phase4_trn_lock_order),
                                 best_score, bits_observed, s->phase4_trn_lock_score,
                                 s->phase4_trn_recent_score);
                    }
                    else if ((s->phase4_trn_after_j % 256) == 0)
                    {
                        /* Find best score for each domain separately */
                        int best_diff_score = 0;
                        int best_abs_score = 0;
                        int dh, dt, do2;
                        for (dt = 0; dt < 2; dt++)
                            for (do2 = 0; do2 < 2; do2++)
                                for (dh = 0; dh < MP_HYPOTHESIS_COUNT; dh++)
                                {
                                    if (s->phase4_trn_one_count_tap[0][dt][do2][dh] > best_diff_score)
                                        best_diff_score = s->phase4_trn_one_count_tap[0][dt][do2][dh];
                                    if (s->phase4_trn_one_count_tap[1][dt][do2][dh] > best_abs_score)
                                        best_abs_score = s->phase4_trn_one_count_tap[1][dt][do2][dh];
                                }
                        span_log(s->logging, SPAN_LOG_FLOW,
                                 "Rx - Phase 4 TRN: best hyp=%d dom=%s tap=%d ord=%s ones=%d/%d (%d%%, recent=%d%%) [diff_best=%d%% abs_best=%d%%]\n",
                                 best_h, phase4_trn_domain_name(best_domain),
                                 phase4_trn_tap_value(best_tap), phase4_trn_order_name(best_order),
                                 best_score, bits_observed, score_pct, s->phase4_trn_recent_score,
                                 (100*best_diff_score + (bits_observed/2))/bits_observed,
                                 (100*best_abs_score + (bits_observed/2))/bits_observed);
                    }
                    /*endif*/
                }
                /*endif*/
            }
            /*endif*/
        }
        /*endif*/

        /* Transition to MP scan after explicit J' and sufficient TRN training.
           Per V.34 §11.4, TRN is at least 512T and up to 2000ms+RTD.
           We train on TRN for ~1.5-2s to lock the equalizer and scrambler
           before scanning for MP. */
        if (s->phase4_j_seen
            && s->phase4_trn_after_j >= PHASE4_TRN_READY_MIN_BAUD
            && s->phase4_trn_lock_score >= PHASE4_TRN_READY_MIN_SCORE)
        {
            int h;
            int domain_idx;
            int tap_idx;
            int order_idx;
            int trn_bits_observed;
            int trn_best_h;
            int trn_best_ones;
            int trn_best_score_pct;
            int trn_best_domain;
            int trn_best_tap;
            int trn_best_order;

            if (s->phase4_trn_after_j >= PHASE4_TRN_SCORE_START_BAUD)
                trn_bits_observed = 2*(s->phase4_trn_after_j - PHASE4_TRN_SCORE_START_BAUD + 1);
            else
                trn_bits_observed = 0;
            /*endif*/
            trn_best_h = -1;
            trn_best_ones = -1;
            trn_best_score_pct = 0;
            trn_best_domain = -1;
            trn_best_tap = -1;
            trn_best_order = -1;
            if (trn_bits_observed > 0)
            {
                for (domain_idx = 0;  domain_idx < 2;  domain_idx++)
                {
                    for (tap_idx = 0;  tap_idx < 2;  tap_idx++)
                    {
                        for (order_idx = 0;  order_idx < 2;  order_idx++)
                        {
                            for (h = 0;  h < MP_HYPOTHESIS_COUNT;  h++)
                            {
                                if (s->phase4_trn_one_count_tap[domain_idx][tap_idx][order_idx][h] > trn_best_ones)
                                {
                                    trn_best_ones = s->phase4_trn_one_count_tap[domain_idx][tap_idx][order_idx][h];
                                    trn_best_h = h;
                                    trn_best_domain = domain_idx;
                                    trn_best_tap = tap_idx;
                                    trn_best_order = order_idx;
                                }
                                /*endif*/
                            }
                            /*endif*/
                        }
                        /*endif*/
                    }
                    /*endif*/
                }
                /*endfor*/
                if (trn_best_h >= 0)
                    trn_best_score_pct = (100*trn_best_ones + (trn_bits_observed/2))/trn_bits_observed;
                /*endif*/
            }
            /*endif*/
            s->received_event = V34_EVENT_PHASE4_TRN_READY;
            if (s->phase4_trn_lock_hyp >= 0
                && s->phase4_trn_lock_hyp < MP_HYPOTHESIS_COUNT
                && s->phase4_trn_lock_domain >= 0 && s->phase4_trn_lock_domain < 2
                && s->phase4_trn_lock_tap >= 0 && s->phase4_trn_lock_tap < 2
                && s->phase4_trn_lock_order >= 0 && s->phase4_trn_lock_order < 2)
            {
                memcpy(s->phase4_trn_scramble,
                       s->phase4_trn_scramble_tap[s->phase4_trn_lock_domain][s->phase4_trn_lock_tap][s->phase4_trn_lock_order],
                       sizeof(s->phase4_trn_scramble));
                memcpy(s->phase4_trn_one_count,
                       s->phase4_trn_one_count_tap[s->phase4_trn_lock_domain][s->phase4_trn_lock_tap][s->phase4_trn_lock_order],
                       sizeof(s->phase4_trn_one_count));
                s->scrambler_tap = phase4_trn_tap_value(s->phase4_trn_lock_tap);
                s->phase3_j_lock_hyp = s->phase4_trn_lock_hyp;
            }
            else if (trn_best_h >= 0  &&  trn_best_h < MP_HYPOTHESIS_COUNT)
            {
                /* Use final TRN best hypothesis at MP handoff. */
                s->phase4_trn_lock_hyp = trn_best_h;
                s->phase4_trn_lock_score = trn_best_score_pct;
                s->phase4_trn_lock_domain = trn_best_domain;
                s->phase4_trn_lock_tap = trn_best_tap;
                s->phase4_trn_lock_order = trn_best_order;
                s->phase3_j_lock_hyp = trn_best_h;
                if (trn_best_domain >= 0 && trn_best_domain < 2
                    && trn_best_tap >= 0 && trn_best_tap < 2
                    && trn_best_order >= 0 && trn_best_order < 2)
                {
                    memcpy(s->phase4_trn_scramble,
                           s->phase4_trn_scramble_tap[trn_best_domain][trn_best_tap][trn_best_order],
                           sizeof(s->phase4_trn_scramble));
                    memcpy(s->phase4_trn_one_count,
                           s->phase4_trn_one_count_tap[trn_best_domain][trn_best_tap][trn_best_order],
                           sizeof(s->phase4_trn_one_count));
                    s->scrambler_tap = phase4_trn_tap_value(trn_best_tap);
                }
                /*endif*/
            }
            /*endif*/
            /* V.34 §7: call modem uses GPC (tap=17), answer modem uses GPA (tap=4).
               We are receiving from the far end, so use the far end's scrambler:
               - If we are the answerer, far end is caller → tap=17 (GPC)
               - If we are the caller, far end is answerer → tap=4 (GPA)
               V.90 reverses this assignment for its asymmetric upstream/downstream.
               TRN (all-ones) cannot distinguish between the two polynomials because
               a self-synchronizing descrambler produces all-ones output for either tap
               once it converges.  Force the correct tap here. */
            {
                int correct_tap;

                correct_tap = s->v90_mode
                            ? (s->calling_party ? 17 : 4)
                            : (s->calling_party ? 4 : 17);
                if (s->scrambler_tap != correct_tap)
                {
                    span_log(s->logging, SPAN_LOG_FLOW,
                             "Rx - Phase 4: TRN selected tap=%d but role requires tap=%d (%s receives from %s); correcting\n",
                             s->scrambler_tap, correct_tap,
                             s->calling_party ? "caller" : "answerer",
                             s->calling_party ? "answerer/GPA" : "caller/GPC");
                    s->scrambler_tap = correct_tap;
                }
                /*endif*/
            }
            span_log(s->logging, SPAN_LOG_FLOW,
                     "Rx - Phase 4: far-end J' + TRN confirmed (J'->TRN=%d bauds, best_ones=%d%%), scanning for MP (decoder=hyp24-v2, gated=900)\n",
                     s->phase4_trn_after_j, s->phase4_trn_lock_score);
            s->stage = V34_RX_STAGE_PHASE4_MP;
            s->duration = 0;
            s->bitstream = 0;
            s->bit_count = 0;
            s->mp_seen = 0;
            /* Do NOT clear received_event here — TX needs to see PHASE4_TRN_READY
               to transition from TRN to MP transmission.  The MP timeout at line 5014
               will overwrite with TRAINING_FAILED if MP decoding fails. */
            s->eq_target_mag = 0.0f;  /* Reset so CMA re-seeds with minimum clamp (1.0) */
            s->mp_remote_ack_seen = 0;
            s->mp_signal_settle_bauds = 0;
            s->mp_count = -1;
            s->mp_frame_pos = 0;
            s->mp_frame_target = 0;
            s->mp_early_rejects = 0;
            s->mp_phase4_default_scrambler_tap = s->scrambler_tap;
            s->mp_phase4_default_bit_order = s->mp_phase4_bit_order;
            s->mp_phase4_default_domain = (s->phase4_trn_lock_domain == 1) ? 1 : 0;
            s->mp_phase4_reject_streak = 0;
            s->mp_phase4_nolock_count = 0;
            s->mp_phase4_alt_tap_active = 0;
            s->mp_phase4_alt_order_active = 0;
            s->mp_phase4_alt_domain_active = 0;
            s->mp_phase4_retry_mode = 0;
            s->mp_phase4_domain = s->mp_phase4_default_domain;
            s->mp_phase4_bit_order = (s->phase4_trn_lock_order == 1) ? 1 : 0;
            s->mp_phase4_default_bit_order = s->mp_phase4_bit_order;
            s->mp_phase4_force_abs_active = 0;
            s->mp_phase4_diff_collapse_streak = 0;
            s->mp_phase4_diff_recover_streak = 0;
            mp_vote_reset(s);
            if (s->phase4_trn_lock_hyp >= 0  &&  s->phase4_trn_lock_hyp < MP_HYPOTHESIS_COUNT)
            {
                span_log(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 4: TRN final best available (hyp=%d, dom=%s, tap=%d, ord=%s, ones=%d%%), starting MP hypothesis search\n",
                         s->phase4_trn_lock_hyp,
                         phase4_trn_domain_name(s->phase4_trn_lock_domain),
                         phase4_trn_tap_value(s->phase4_trn_lock_tap),
                         phase4_trn_order_name(s->phase4_trn_lock_order),
                         s->phase4_trn_lock_score);
            }
            else if (s->phase3_j_lock_hyp >= 0  &&  s->phase3_j_lock_hyp < MP_HYPOTHESIS_COUNT)
            {
                span_log(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 4: Phase 3 J lock hint available (hyp=%d, trn=%s), starting MP hypothesis search\n",
                         s->phase3_j_lock_hyp,
                         (s->phase3_j_trn16 < 0) ? "unknown" : (s->phase3_j_trn16 ? "16-point" : "4-point"));
            }
            /*endif*/
            mp_reset_hypothesis_search(s);
            memcpy(s->mp_hyp_scramble, s->phase4_trn_scramble, sizeof(s->mp_hyp_scramble));
            if (s->phase4_trn_lock_hyp >= 0
                && s->phase4_trn_lock_hyp < MP_HYPOTHESIS_COUNT)
            {
                /* A strong TRN ones-lock is useful as a hypothesis/search hint,
                   but it is not sufficient to assert that the current symbols
                   already belong to MP. In practice this pre-lock path tends to
                   consume late-TRN symbols as if they were MP, producing fake
                   all-ones preambles and poisoning frame alignment immediately.
                   Keep the TRN-selected domain/tap/order as the initial search
                   mode, but require an actually observed MP preamble before
                   locking a hypothesis. */
                span_log(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 4: TRN hint available but direct MP pre-lock disabled "
                         "(hyp=%d, dom=%s, tap=%d, ord=%s, ones=%d%%); waiting for observed MP preamble\n",
                         s->phase4_trn_lock_hyp,
                         phase4_trn_domain_name(s->phase4_trn_lock_domain),
                         phase4_trn_tap_value(s->phase4_trn_lock_tap),
                         phase4_trn_order_name(s->phase4_trn_lock_order),
                         s->phase4_trn_lock_score);
            }
            /*endif*/
        }
        /*endif*/
        else if (s->phase4_j_seen
                 && s->phase4_trn_after_j >= 512
                 && (s->phase4_trn_after_j % 256) == 0)
        {
            span_log(s->logging, SPAN_LOG_FLOW,
                     "Rx - Phase 4: waiting for TRN ones-lock before MP (after_j=%d, lock=%d%% current=%d%%, min=%d); staying in TRN\n",
                     s->phase4_trn_after_j, s->phase4_trn_lock_score, s->phase4_trn_current_score, PHASE4_TRN_READY_MIN_SCORE);
        }
        /*endif*/
        /* Timeout: if TRN lock never achieved within max allowed time, signal failure (once) */
        if (s->phase4_j_seen
            && s->phase4_trn_after_j >= PHASE4_TRN_READY_MAX_BAUD
            && s->phase4_trn_lock_score < PHASE4_TRN_READY_MIN_SCORE
            && s->received_event != V34_EVENT_TRAINING_FAILED)
        {
            span_log(s->logging, SPAN_LOG_FLOW,
                     "Rx - Phase 4: TRN timeout (%d bauds after J', lock=%d%% < %d%%); signalling failure\n",
                     s->phase4_trn_after_j, s->phase4_trn_lock_score, PHASE4_TRN_READY_MIN_SCORE);
            s->received_event = V34_EVENT_TRAINING_FAILED;
        }
        /*endif*/
        else if (s->duration >= 5200  &&  (s->duration % 512) == 0)
        {
            /* Do not force MP without explicit J' + TRN confirmation. */
            span_log(s->logging, SPAN_LOG_FLOW,
                     "Rx - Phase 4: still waiting for far-end J'/TRN confirmation (%d bauds)\n",
                     s->duration);
        }
        }
        break;

    case V34_RX_STAGE_V90_CP:
    case V34_RX_STAGE_PHASE4_MP:
        /* These stages share only the V.34 primary-channel DQPSK front end
           and hypothesis acquisition. V34_RX_STAGE_V90_CP hands the recovered
           bitstream to the project-owned V.90 Table-14 framer; the ordinary
           MP stage retains V.34 frame lengths, acknowledgements, timeouts and
           the E transition. Keeping the protocol states distinct is important:
           V.90 9.4.1.1 requires CPt reception while the digital modem sends Ri,
           without the preceding V.34 J'/TRN-to-MP sequencing. */
        {
            int locked_this_symbol;
            int h;
            int expected_mp_type;
            int abs_bits;
            int mp_decode_domain;
            int v90_cp_rx;

        v90_cp_rx = (s->stage == V34_RX_STAGE_V90_CP);
        ang1 = arctan2(sym->im, sym->re);
        ang2 = arctan2(s->last_sample.im, s->last_sample.re);
        ang3 = ang1 - ang2 + DDS_PHASE(45.0f);
        data_bits = (ang3 >> 30) & 0x3;
        abs_bits = (int) ((ang1 + DDS_PHASE(45.0f)) >> 30) & 0x3;
        mp_decode_domain = s->mp_phase4_force_abs_active ? 1 : s->mp_phase4_domain;

        /* Decision-aided carrier acquisition through the CP/MP stretch.
           The receiver reaches Phase 4 with no coherent lock (the TRN/MP
           "locks" are bit-domain hypothesis latches; CP/MP decode survives
           on differential dibits alone).  Data mode needs true coherence,
           and this window is the last chance to acquire it: the analogue
           modem's differential encoder is initialized to zero at the first
           CPt (8.5.2/V.90), the 4-point constellation sits on the 45-degree
           family, and once the hypothesis machinery locks, the differential
           dibit decisions here are the same ones that go on to CRC-validate
           whole CP frames -- reliable known data.  Integrate those quadrant
           increments into an expected absolute angle (seeded by snapping the
           received angle onto the 45-degree family, leaving only the
           90-degree ambiguity that V.34's differential quadrant bits absorb
           in data mode) and drive the existing carrier loop with the angle
           error, so the constellation is pulled onto the true grid before
           E/B1 hands over to the DATA slicer.  ME_V34_PHASE4_DA_TRACK=0
           disables for A/B. */
        {
            static int da_enabled = -1;

            if (da_enabled < 0)
            {
                const char *value = getenv("ME_V34_PHASE4_DA_TRACK");

                da_enabled = (value == NULL || atoi(value) != 0);
            }
            float da_sym_mag2 = sym->re*sym->re + sym->im*sym->im;

            if (!da_enabled || s->mp_hypothesis < 0)
            {
                s->phase4_da_active = 0;
            }
            else if (da_sym_mag2 < 0.04f)
            {
                /* The analogue modem goes quiet for seconds between CPt and
                   the data-mode CP (9.4.2.2/V.90 makes SCR optional).  A
                   zero symbol has arctan2(0,0) = constant angle, so the
                   tracker "locks" perfectly on nothing and the stale seed
                   survives to the signal return.  Suspend; V34_DA_HOLD_AFTER_
                   SILENCE=1 additionally freezes the CPt-era solution through
                   the (16-point-suspect) second CP stretch instead of
                   re-acquiring on it -- the channel is static, so the CPt
                   solution should carry to DATA. */
                s->phase4_da_active = 0;
                if (s->phase4_da_seeded && getenv("V34_DA_HOLD_AFTER_SILENCE"))
                    s->phase4_da_seeded = 2;    /* 2 = hold: no more updates */
            }
            else if (s->phase4_da_seeded == 2)
            {
                /* Held: CPt-era derot + equalizer frozen through to DATA. */
            }
            else if (!s->phase4_da_active)
            {
                /* Seed the derotator so the corrected angle lands on the
                   45-degree constellation family (90-degree family ambiguity
                   is absorbed by the data differential quadrant bits), and
                   start the expected-angle integrator there. */
                uint32_t ang1c = ang1 - s->phase4_da_derot;
                uint32_t tmp = ang1c - DDS_PHASE(45.0f);
                uint32_t quadrant = (tmp + 0x20000000U) >> 30;
                uint32_t snapped = (quadrant << 30) + DDS_PHASE(45.0f);

                s->phase4_da_derot += (uint32_t) ((int32_t) (ang1c - snapped));
                s->phase4_da_expected_ang = snapped;
                s->phase4_da_active = 1;
                s->phase4_da_seeded = 1;
            }
            else
            {
                int32_t err_i;

                s->phase4_da_expected_ang += ((uint32_t) data_bits) << 30;
                err_i = (int32_t) ((ang1 - s->phase4_da_derot)
                                   - s->phase4_da_expected_ang);
                /* Zero-delay loop: quarter-deadbeat is stable and follows
                   the CMA random walk with a ~4-baud time constant. */
                s->phase4_da_derot += err_i/4;
                /* Data-aided LMS: with the transmitted symbol known, train
                   the equalizer to the true zero-ISI solution.  CMA only
                   restores the modulus; its residual ISI is invisible on
                   constant-envelope signals but is exactly what smears the
                   multi-amplitude data constellation into ringless mush.
                   Target = the known 4-point symbol at CMA's unit radius,
                   rotated back into the equalizer-output domain. */
                {
                    float tang = (float) (uint32_t) (s->phase4_da_expected_ang
                                                     + s->phase4_da_derot)
                               * (float) (3.14159265358979/2147483648.0);
                    complexf_t da_target;

                    da_target.re = cosf(tang);
                    da_target.im = sinf(tang);
                    tune_equalizer(s, sym, &da_target);
                }
                if (getenv("V34_DA_TRACK_LOG") && (s->duration % 256) == 0)
                {
                    fprintf(stderr,
                            "[DA] baud=%d err=%.1fdeg derot=%.1fdeg timing_corr=%d baud_phase=%.1f\n",
                            s->duration,
                            (float) err_i*(360.0f/4294967296.0f),
                            (float) s->phase4_da_derot*(360.0f/4294967296.0f),
                            s->total_baud_timing_correction,
                            (float) s->pri_ted.baud_phase);
                }
            }
        }

        /* ME_V34_DUMP_MP_DIBITS (diagnostic, 2026-07-19): dumps the raw
           diff/abs dibit per baud for offline replay -- e.g. brute-forcing
           every (hypothesis, domain, tap, order) combination against the
           actual captured stream with a properly continuous self-
           synchronizing scrambler, independent of this file's own
           hypothesis-search state machine. Used to rule out a demapping/
           descrambling logic bug for a live MP CRC failure against
           d-modem/slmodemd: none of the 24*2*2*2=192 combinations, at any
           frame-start offset, produced a valid frame from the captured
           dibits, which points upstream at symbol demodulation (equalizer/
           carrier/timing feeding the angle computation below) rather than
           this file's bit-level decode logic. See rig/README.md. */
        if (getenv("ME_V34_DUMP_MP_DIBITS") && s->mp_seen == 0 && s->duration < 6000)
        {
            /* coefmag/rawmag (diagnostic, 2026-07-19): distinguishes a
               zeroed-coefficient equalizer from a genuinely-silent input —
               coefmag is the summed tap magnitude of s->eq_coeff, rawmag is
               the magnitude of the most recently inserted raw eq_buf sample
               (same index tune_equalizer_cma() reads as its newest tap). */
            float coefmag = 0.0f;
            int ci;
            int rawp;
            for (ci = 0;  ci < V34_EQUALIZER_PRE_LEN + 1 + V34_EQUALIZER_POST_LEN;  ci++)
                coefmag += sqrtf(s->eq_coeff[ci].re*s->eq_coeff[ci].re + s->eq_coeff[ci].im*s->eq_coeff[ci].im);
            rawp = (s->eq_step - 1) & V34_EQUALIZER_MASK;
            span_log(s->logging, SPAN_LOG_FLOW,
                     "Rx - MP raw dibit dump: baud=%d diff=%d abs=%d re=%.4f im=%.4f mag=%.4f ang1deg=%.2f coefmag=%.4f rawmag=%.4f\n",
                     s->duration, data_bits, abs_bits,
                     (double) sym->re, (double) sym->im,
                     (double) sqrtf(sym->re*sym->re + sym->im*sym->im),
                     (double) (ang1 * 360.0f / 4294967296.0f),
                     (double) coefmag,
                     (double) sqrtf(s->eq_buf[rawp].re*s->eq_buf[rawp].re + s->eq_buf[rawp].im*s->eq_buf[rawp].im));
        }

        /* Dibit distribution diagnostic */
        {
            static int dibit_hist[4] = {0,0,0,0};
            static int abs_hist[4] = {0,0,0,0};
            static int diag_count = 0;
            static int diag_baud = 0;
            dibit_hist[data_bits]++;
            abs_hist[abs_bits]++;
            if (s->mp_seen == 0)
            {
                diag_count++;
                diag_baud++;
                if (diag_count == 400)
                {
                    if (V34_DEBUG_MP_DIBIT_DIST
                        && (diag_baud % PHASE4_MP_DIBIT_LOG_INTERVAL) == 0)
                    {
                        float mag = sqrtf(sym->re*sym->re + sym->im*sym->im);
                        float ang_deg = atan2f(sym->im, sym->re) * 180.0f / 3.14159265f;
                        span_log(s->logging, SPAN_LOG_FLOW,
                                 "Rx - Phase 4 MP dibit dist (400 bauds): diff=[%d,%d,%d,%d] abs=[%d,%d,%d,%d] mag=%.3f ang=%.1f\n",
                                 dibit_hist[0], dibit_hist[1], dibit_hist[2], dibit_hist[3],
                                 abs_hist[0], abs_hist[1], abs_hist[2], abs_hist[3],
                                 mag, ang_deg);
                    }
                    mp_phase4_update_auto_domain(s, dibit_hist, abs_hist);
                    dibit_hist[0] = dibit_hist[1] = dibit_hist[2] = dibit_hist[3] = 0;
                    abs_hist[0] = abs_hist[1] = abs_hist[2] = abs_hist[3] = 0;
                    diag_count = 0;
                }
            }
        }

        /* During initial Phase 4 MP acquisition, lock only MP0 (type 0).
           In observed failures, early MP1 locks are almost always false and
           consume the Phase 4 budget. */
        expected_mp_type = (s->mp_seen == 0) ? 0 : -1;

            locked_this_symbol = 0;

            if (s->mp_hypothesis >= 0)
            {
                int in0;
                int in1;
                int raw_bits;

                raw_bits = map_phase4_raw_bits((mp_decode_domain == 1) ? abs_bits : data_bits,
                                               s->mp_hypothesis);
                phase4_unpack_ordered_bits(raw_bits, s->mp_phase4_bit_order, &in0, &in1);
                bits[0] = descramble(s, in0);
                bits[1] = descramble(s, in1);
            }
            else if ((sym->re*sym->re + sym->im*sym->im)
                         < MP_LOCK_MIN_SIGNAL_MAG2
                     || (v90_cp_rx
                         && power_meter_current(&s->power)
                              < s->carrier_off_power))
            {
                /* No real signal yet -- don't attempt a preamble lock
                   against near-zero-energy samples. Measured live: after
                   "far-end J' + TRN confirmed" our RX enters MP search
                   immediately, but d-modem/slmodemd doesn't actually start
                   transmitting real MP data until ~5500 bauds (~2.3s)
                   later, ramping from exactly 0.0 to full amplitude over
                   ~20 bauds. Without this gate, the hypothesis search
                   deterministically locks onto a frame straddling that
                   ramp -- structurally plausible (correct start bits) but
                   with corrupted payload bits from the still-settling
                   samples, failing CRC every time. See rig/README.md.

                   The equalizer can ring above MP_LOCK_MIN_SIGNAL_MAG2
                   through the real pre-CPt carrier gap. For the forced V.90
                   CP receiver, also use the primary-channel carrier-off
                   threshold; unlike an arbitrary constellation threshold,
                   that meter is already level-calibrated and hysteretic. */
                if (v90_cp_rx && s->mp_signal_settle_bauds > 0)
                {
                    mp_v90_cp_reset_at_carrier_gap(s);
                    span_log(s->logging, SPAN_LOG_FLOW,
                             "Rx - V.90 Phase 4: CP carrier gap; reset Table-14 search to first-CPt initial state\n");
                }
                /*endif*/
                s->mp_signal_settle_bauds = 0;
                bits[0] = bits[1] = 0;
            }
            else if (!v90_cp_rx
                     && s->mp_signal_settle_bauds++ < MP_LOCK_SETTLE_BAUDS)
            {
                /* Signal is present but the CMA equalizer is still
                   re-adapting its gain from the silence-era near-zero
                   coefficients to the real signal's amplitude -- locking
                   here still produces structurally-plausible but corrupt
                   frames (start-bit/CRC failures). Wait for a sustained
                   settle window. See rig/README.md.

                   Do not apply that V.34 MP guard to the V.90 digital
                   answerer's project-owned CP receiver. The primary-channel
                   front end was already conditioned before
                   v34_force_v90_phase4_cp_rx(), and V.90 9.4.1.1 requires
                   the digital modem to condition its receiver to receive
                   CPt while sending Ri. A short analogue modem can finish
                   its first CPt before this 400-baud guard expires. The
                   external Table-14 receiver still requires exact framing,
                   CRC and semantic validity before accepting anything. */
                bits[0] = bits[1] = 0;
            }
            else
            {
                int chosen_hyp;
                int chosen_type_bit;
                int chosen_score;
                uint32_t chosen_reg;
                uint32_t chosen_bstream;
                int chosen_bit_pos;
                int chosen_pending_valid;
                int chosen_pending_bit;
                uint32_t chosen_preamble_stream;
                int hint_h;
                int hint_only;
                int strict_mp0_lock;
                int hint_found;
                int hint_score;
                int hint_type_bit;
                uint32_t hint_reg;
                uint32_t hint_bstream;
                int hint_bit_pos;
                int hint_pending_valid;
                int hint_pending_bit;
                uint32_t hint_preamble_stream;

                if (v90_cp_rx)
                    s->mp_signal_settle_bauds++;
                /*endif*/
                chosen_hyp = -1;
                chosen_type_bit = 0;
                chosen_score = -1;
                chosen_reg = 0;
                chosen_bstream = 0;
                chosen_bit_pos = -1;
                chosen_pending_valid = 0;
                chosen_pending_bit = 0;
                chosen_preamble_stream = 0;
                hint_h = mp_phase4_has_pinned_trn_lock(s) ? s->phase4_trn_lock_hyp : s->phase3_j_lock_hyp;
                strict_mp0_lock = (s->mp_seen == 0 && expected_mp_type == 0);
                /* Constrain early MP lock to the TRN/J hint until we have
                   accumulated a couple of failed frame attempts. Using absolute
                   phase4 duration is ineffective because MP starts late in phase4. */
                hint_only = (hint_h >= 0
                             && s->mp_phase4_reject_streak < MP_HINT_STRICT_REJECTS
                             && s->mp_phase4_nolock_count < MP_HINT_MAX_NOLOCKS
                             && hint_h < MP_HYPOTHESIS_COUNT
                             && (mp_phase4_has_pinned_trn_lock(s)
                                 || s->phase3_j_lock_hyp >= 0));
                hint_found = 0;
                hint_score = -1;
                hint_type_bit = 0;
                hint_reg = 0;
                hint_bstream = 0;
                hint_bit_pos = -1;
                hint_pending_valid = 0;
                hint_pending_bit = 0;
                hint_preamble_stream = 0;
                for (h = 0;  h < MP_HYPOTHESIS_COUNT;  h++)
                {
                    uint32_t reg;
                    uint32_t bstream;
                    uint32_t pre0;
                    int raw_bits;
                    int d0;
                    int d1;
                    int in0;
                    int in1;
                    int sc;
                    int sc0;

                    reg = s->mp_hyp_scramble[h];
                    bstream = s->mp_hyp_bitstream[h];
                    raw_bits = map_phase4_raw_bits((mp_decode_domain == 1) ? abs_bits : data_bits, h);
                    phase4_unpack_ordered_bits(raw_bits, s->mp_phase4_bit_order, &in0, &in1);
                    d0 = descramble_reg(&reg, s->scrambler_tap, in0);
                    bstream = (bstream << 1) | d0;
                    sc0 = mp_preamble_score(bstream);
                    pre0 = bstream;
                    d1 = descramble_reg(&reg, s->scrambler_tap, in1);
                    bstream = (bstream << 1) | d1;
                    if (sc0 >= ((hint_only && h == hint_h) ? MP_HINT_LOCK_SCORE_MIN : MP_LOCK_SCORE_MIN)
                        && (!hint_only || h == hint_h)
                        && (expected_mp_type < 0 || d0 == expected_mp_type)
                        && (d1 == 0
                            || v90_cp_rx)
                        && mp_preamble_has_start_zero(pre0)
                        && mp_preamble_has_sync_ones(pre0)
                        && sc0 > chosen_score)
                    {
                        /* Lock boundary at bit0: current bit is MP type bit.
                           The second bit of this dibit belongs to frame_idx=19. */
                        chosen_hyp = h;
                        chosen_type_bit = d0;
                        chosen_score = sc0;
                        chosen_reg = reg;
                        chosen_bstream = bstream;
                        chosen_bit_pos = 0;
                        chosen_pending_valid = 1;
                        chosen_pending_bit = d1;
                        chosen_preamble_stream = pre0;
                    }
                    /*endif*/
                    if (h == hint_h
                        && sc0 >= MP_HINT_LOCK_SCORE_MIN
                        && (expected_mp_type < 0 || d0 == expected_mp_type)
                        && (d1 == 0
                            || v90_cp_rx)
                        && mp_preamble_has_start_zero(pre0)
                        && mp_preamble_has_sync_ones(pre0)
                        && sc0 > hint_score)
                    {
                        hint_found = 1;
                        hint_score = sc0;
                        hint_type_bit = d0;
                        hint_reg = reg;
                        hint_bstream = bstream;
                        hint_bit_pos = 0;
                        hint_pending_valid = 1;
                        hint_pending_bit = d1;
                        hint_preamble_stream = pre0;
                    }
                    /*endif*/
                    sc = mp_preamble_score(bstream);
                    if ((!strict_mp0_lock || expected_mp_type != 0
                         || v90_cp_rx)
                        && sc >= ((hint_only && h == hint_h) ? MP_HINT_LOCK_SCORE_MIN : MP_LOCK_SCORE_MIN)
                        && (!hint_only || h == hint_h)
                        && (!strict_mp0_lock || sc >= MP_PREAMBLE_SCORE_MIN)
                        && d0 == 0
                        && (expected_mp_type < 0 || d1 == expected_mp_type)
                        && mp_preamble_has_start_zero(bstream)
                        && mp_preamble_has_sync_ones(bstream)
                        && sc > chosen_score)
                    {
                        chosen_hyp = h;
                        chosen_type_bit = d1;
                        chosen_score = sc;
                        chosen_reg = reg;
                        chosen_bstream = bstream;
                        chosen_bit_pos = 1;
                        chosen_pending_valid = 0;
                        chosen_pending_bit = 0;
                        chosen_preamble_stream = bstream;
                    }
                    /*endif*/
                    if ((!strict_mp0_lock || expected_mp_type != 0
                         || v90_cp_rx)
                        && h == hint_h
                        && sc >= MP_HINT_LOCK_SCORE_MIN
                        && (!strict_mp0_lock || sc >= MP_PREAMBLE_SCORE_MIN)
                        && d0 == 0
                        && (expected_mp_type < 0 || d1 == expected_mp_type)
                        && mp_preamble_has_start_zero(bstream)
                        && mp_preamble_has_sync_ones(bstream)
                        && sc > hint_score)
                    {
                        hint_found = 1;
                        hint_score = sc;
                        hint_type_bit = d1;
                        hint_reg = reg;
                        hint_bstream = bstream;
                        hint_bit_pos = 1;
                        hint_pending_valid = 0;
                        hint_pending_bit = 0;
                        hint_preamble_stream = bstream;
                    }
                    s->mp_hyp_scramble[h] = reg;
                    s->mp_hyp_bitstream[h] = bstream;
                }
                /*endfor*/
                if (hint_found
                    && hint_h >= 0
                    && hint_h < MP_HYPOTHESIS_COUNT
                    /* Do not let a weaker TRN/J hint override an equally or more
                       convincing live preamble candidate. The hint is only meant
                       to break ties when the observed MP preamble is just as good. */
                    && (chosen_hyp < 0 || hint_score >= chosen_score))
                {
                    chosen_hyp = hint_h;
                    chosen_type_bit = hint_type_bit;
                    chosen_score = hint_score;
                    chosen_reg = hint_reg;
                    chosen_bstream = hint_bstream;
                    chosen_bit_pos = hint_bit_pos;
                    chosen_pending_valid = hint_pending_valid;
                    chosen_pending_bit = hint_pending_bit;
                    chosen_preamble_stream = hint_preamble_stream;
                }
                /*endif*/
                /* Expose one live hypothesis stream for diagnostics while unlocked. */
                s->bitstream = s->mp_hyp_bitstream[0];

                if (chosen_hyp >= 0)
                {
                    s->mp_phase4_nolock_count = 0;
                    s->mp_hypothesis = chosen_hyp;
                    s->scramble_reg = chosen_reg;
                    s->bitstream = chosen_bstream;
                    s->crc = 0xFFFF;
                    s->bit_count = 0;
                    s->mp_count = 0;
                    s->mp_len = 84 + 1;
                    s->mp_and_fill_len = 186 + 1 + 1;
                    s->mp_early_rejects = 0;
                    /* Lock criterion is the MP preamble itself, so start frame
                       collection immediately at this boundary. */
                    {
                        int type;

                        mp_seed_frame_prefix(s->mp_frame_bits, chosen_preamble_stream);
                        type = s->mp_frame_bits[18];
                        s->mp_frame_target = (type == 1)  ?  188  :  88;
                        s->mp_frame_pos = 19;
                        if (chosen_pending_valid)
                        {
                            s->mp_frame_bits[s->mp_frame_pos++] = chosen_pending_bit;
                            s->bit_count = 1;
                        }
                        else
                        {
                            s->bit_count = 0;
                        }
                        /*endif*/
                        if (v90_cp_rx)
                        {
                            int replay_len;

                            replay_len = s->mp_frame_pos;
                            for (int replay = 0; replay < replay_len; replay++)
                            {
                                int replay_bit;

                                /* The hypothesis lock permits up to two errors
                                   in the fixed 17-one/start-zero preamble.  The
                                   project CP framer quite properly requires an
                                   exact sync word, so repair only those fixed
                                   acquisition bits after the hypothesis has
                                   passed the preamble gate.  Type and all
                                   variable/CRC-protected bits remain observed
                                   data and are never corrected here. */
                                replay_bit = (replay < 17)
                                           ? 1
                                           : ((replay == 17)
                                              ? 0
                                              : (s->mp_frame_bits[replay] & 1));
                                s->put_phase4_bit(s->put_phase4_bit_user_data,
                                                  replay_bit);
                            }
                            /* The project-owned Table 14 framer owns CP length,
                               CRC, fill and semantics from this boundary on. */
                            s->mp_frame_pos = 0;
                            s->mp_frame_target = 0;
                        }
                        log_mp_lock_seed(s,
                                         chosen_hyp,
                                         chosen_type_bit,
                                         chosen_score,
                                         chosen_bit_pos,
                                         chosen_preamble_stream,
                                         chosen_pending_valid,
                                         chosen_pending_bit);
                    }
                    locked_this_symbol = 1;
                    span_log(s->logging, SPAN_LOG_FLOW,
                             "Rx - Phase 4: locked MP hypothesis=%d, type_bit=%d preamble_score=%d/18 at bit%d (baud %d), frame start armed%s\n",
                             chosen_hyp, chosen_type_bit, chosen_score, chosen_bit_pos,
                             s->duration + 1, chosen_pending_valid ? " (+1 pending bit)" : "");
                }
                /*endif*/
                bits[0] = bits[1] = 0;
            }
            /*endif*/

        s->duration++;
        if (v90_cp_rx)
        {
            if (s->mp_hypothesis >= 0 && !locked_this_symbol)
            {
                s->put_phase4_bit(s->put_phase4_bit_user_data, bits[0] & 1);
                s->put_phase4_bit(s->put_phase4_bit_user_data, bits[1] & 1);
            }
            /* CP is variable length and is validated by the V.90 layer. Do not
               apply V.34 MP lengths, semantic recovery or forced E timeouts. */
            break;
        }
        /* MP stage timeout: if we haven't successfully decoded an MP frame
           within ~2s (4800 bauds at 2400 baud rate), signal training failure
           so the call can retrain or drop.  The far-end won't wait forever. */
        if (s->mp_seen == 0
            && s->duration >= PHASE4_MP_TIMEOUT_BAUDS
            && s->received_event != V34_EVENT_TRAINING_FAILED)
        {
            span_log(s->logging, SPAN_LOG_FLOW,
                     "Rx - Phase 4: MP timeout (%d bauds without successful MP frame); signalling failure\n",
                     s->duration);
            s->received_event = V34_EVENT_TRAINING_FAILED;
        }
        /*endif*/
        if (s->mp_hypothesis < 0
            && s->mp_seen == 0
            && (s->duration % 400) == 0)
        {
            /* A pinned TRN lock is the best initial MP hint, but it is not
               infallible. In A-law especially we can end up with the wrong
               MP domain/order/tap while never locking any MP hypothesis at
               all. Hold the pinned settings for a few no-lock windows first,
               then rotate retry modes so we do not sit in PHASE4_MP until
               timeout on a bad decode mode forever. */
            if (mp_phase4_has_pinned_trn_lock(s))
            {
                s->mp_phase4_nolock_count++;
                if (s->mp_phase4_nolock_count >= MP_HINT_MAX_NOLOCKS
                    && s->mp_phase4_reject_streak < MP_HINT_STRICT_REJECTS)
                {
                    s->mp_phase4_reject_streak = MP_HINT_STRICT_REJECTS;
                    span_log(s->logging, SPAN_LOG_FLOW,
                             "Rx - Phase 4: no MP lock at baud %d with pinned hint hyp=%d for %d windows; broadening MP search beyond hint-only\n",
                             s->duration, s->phase4_trn_lock_hyp, s->mp_phase4_nolock_count);
                }
                /*endif*/
                /* Log best preamble score for the hint hypothesis to diagnose MP lock failures */
                {
                    int best_sc = 0;
                    for (h = 0;  h < MP_HYPOTHESIS_COUNT;  h++)
                    {
                        int sc_diag = mp_preamble_score(s->mp_hyp_bitstream[h]);
                        if (sc_diag > best_sc)
                            best_sc = sc_diag;
                    }
                    if ((s->duration % PHASE4_MP_NOLOCK_LOG_INTERVAL) == 0
                        || best_sc >= (MP_PREAMBLE_SCORE_MIN - 1))
                    {
                        span_log(s->logging, SPAN_LOG_FLOW,
                                 "Rx - Phase 4: no MP lock at baud %d; best_score=%d/%d hint_hyp=%d hint_bs=0x%08X (dom=%s%s, tap=%d, ord=%s)\n",
                                 s->duration, best_sc, MP_PREAMBLE_SCORE_MIN,
                                 s->phase4_trn_lock_hyp,
                                 (unsigned)s->mp_hyp_bitstream[s->phase4_trn_lock_hyp],
                                 phase4_trn_domain_name(s->mp_phase4_domain),
                                 s->mp_phase4_force_abs_active ? "/auto-abs" : "",
                                 s->scrambler_tap,
                                 phase4_trn_order_name(s->mp_phase4_bit_order));
                    }
                    /*endif*/
                }
                if (s->mp_phase4_nolock_count >= MP_HINT_MAX_NOLOCKS)
                {
                    mp_phase4_rotate_retry_mode(s, "no MP hypothesis lock");
                }
                else
                {
                    mp_reset_hypothesis_search(s);
                }
                /*endif*/
            }
            else
            {
                mp_phase4_rotate_retry_mode(s, "no MP hypothesis lock");
            }
        }
        /*endif*/
        /* Per-baud MP diagnostics are most useful once a hypothesis is locked.
           Suppress unlocked chatter (hyp=-1), which tends to dominate logs without
           adding actionable information. */
        if (s->mp_hypothesis >= 0
            && s->mp_seen == 0
            && (s->duration <= 6 || (s->duration % PHASE4_MP_BAUD_LOG_INTERVAL) == 0))
        {
            float mag = sqrtf(sym->re * sym->re + sym->im * sym->im);
            span_log(s->logging, SPAN_LOG_FLOW,
                     "Rx - Phase 4 MP baud %d: mag=%.3f data_bits=%d descr=%d,%d "
                     "bitstream=0x%08X hyp=%d\n",
                     s->duration, mag, data_bits, bits[0], bits[1],
                     (unsigned)s->bitstream, s->mp_hypothesis);
        }

        /* Scan for MP frames on the primary channel */
        if (s->mp_hypothesis >= 0  &&  !locked_this_symbol)
        {
            for (i = 0;  i < 2;  i++)
            {
                s->bitstream = (s->bitstream << 1) | bits[i];
                if (s->mp_hypothesis >= 0  &&  s->mp_frame_pos == 0  &&  s->mp_seen == 0)
                {
                    int preamble_wait_limit;
                    const char *hold_env;
                    bool hold_mp_hypothesis;

                    v34_rx_log_mp_diag_state(s, V34_MP_DIAG_STATE_DET_SYNC, "awaiting MP preamble");

                    hold_env = getenv("ME_V34_HOLD_MP_HYPOTHESIS");
                    hold_mp_hypothesis = hold_env && atoi(hold_env) != 0;

                    /* If we started from a TRN direct pre-lock, fail fast and
                       fall back to global hypothesis search rather than waiting
                       the full window on a possibly phase-offset hypothesis. */
                    preamble_wait_limit = (s->mp_phase4_reject_streak == 0)
                                          ? MP_PRELOCK_PREAMBLE_WAIT_BITS
                                          : MP_PREAMBLE_WAIT_BITS;
                    if (++s->mp_count > preamble_wait_limit)
                    {
                        if (hold_mp_hypothesis)
                        {
                            /* A strong preamble can be followed by a short
                               equalizer/timing excursion before the peer's
                               repeated MP arrives. Keep the same hypothesis
                               and restart only the local wait window. The
                               normal CRC and semantic checks still decide
                               whether this hypothesis is valid. */
                            s->mp_count = 0;
                            span_log(s->logging, SPAN_LOG_FLOW,
                                     "Rx - Phase 4: holding MP hypothesis=%d after preamble timeout; restarting local wait\n",
                                     s->mp_hypothesis);
                            break;
                        }
                        span_log(s->logging, SPAN_LOG_FLOW,
                                 "Rx - Phase 4: unlock MP hypothesis=%d (no preamble within %d bits)\n",
                                 s->mp_hypothesis, s->mp_count);
                        if (mp_phase4_has_pinned_trn_lock(s))
                        {
                            /* Keep TRN-locked descrambler settings, just reset
                               hypothesis search to re-scan for preamble */
                            mp_reset_hypothesis_search(s);
                            span_log(s->logging, SPAN_LOG_FLOW,
                                     "Rx - Phase 4: preamble timeout; keeping TRN-locked settings, resetting preamble search\n");
                        }
                        else
                        {
                            mp_phase4_rotate_retry_mode(s, "unlock MP hypothesis (preamble timeout)");
                        }
                        break;
                    }
                    /*endif*/
                }
                /*endif*/
                if (s->mp_seen >= 2)
                {
                    /* Should not reach here — we transition to V34_RX_STAGE_DATA on E */
                    continue;
                }
                /*endif*/
                if (s->mp_seen == 1  &&  (s->bitstream & 0xFFFFF) == 0xFFFFF)
                {
                    /* E is 20 consecutive ones — end of MP exchange.
                       Transition to data mode: the far end will send B1 (one mapping
                       frame of ones) then switch to the negotiated data constellation. */
                    v34_rx_log_mp_diag_state(s, V34_MP_DIAG_STATE_COMPLETE, "E detected");
                    span_log(s->logging, SPAN_LOG_FLOW,
                             "Rx - Phase 4: E signal detected, MP exchange complete — transitioning to DATA mode\n");
                    s->mp_seen = 2;
                    s->received_event = V34_EVENT_E;
                    /* Initialize data mode state */
                    s->step_2d = 0;
                    s->data_frame = 0;
                    s->super_frame = 0;
                    s->v0_pattern = 0;
                    s->mapping_frame_count = 0;
                    s->s_bit_cnt = 0;
                    s->aux_bit_cnt = 0;
                    memset(s->xt, 0, sizeof(s->xt));
                    memset(s->x, 0, sizeof(s->x));
                    memset(s->ww, 0, sizeof(s->ww));
                    s->viterbi.ptr = 0;
                    s->viterbi.windup = 15;
                    span_log(s->logging, SPAN_LOG_FLOW,
                             "Rx - DATA mode: parms b=%d k=%d q=%d m=%d p=%d j=%d l=%d r=%d w=%d\n",
                             s->parms.b, s->parms.k, s->parms.q, s->parms.m,
                             s->parms.p, s->parms.j, s->parms.l, s->parms.r, s->parms.w);
                    s->stage = V34_RX_STAGE_DATA;
                    if (s->duplex)
                        report_status_change(s, SIG_STATUS_TRAINING_SUCCEEDED);
                    /*endif*/
                    break;  /* Exit the bit loop; next baud will enter DATA stage */
                }
                /*endif*/
                if (s->mp_seen == 1  &&  s->mp_accepted_baud > 0
                    &&  (s->duration - s->mp_accepted_baud) > 500)
                {
                    /* Timeout: E not detected within 500 bauds of MP acceptance.
                       The far end likely already sent E and moved to data mode
                       while we were still majority-voting MP frames.  Force
                       transition to DATA mode. */
                    v34_rx_log_mp_diag_state(s, V34_MP_DIAG_STATE_COMPLETE, "E timeout — forcing DATA transition");
                    span_log(s->logging, SPAN_LOG_FLOW,
                             "Rx - Phase 4: E detection timeout (%d bauds since MP accept) — forcing DATA mode\n",
                             s->duration - s->mp_accepted_baud);
                    s->mp_seen = 2;
                    s->step_2d = 0;
                    s->data_frame = 0;
                    s->super_frame = 0;
                    s->v0_pattern = 0;
                    s->mapping_frame_count = 0;
                    s->s_bit_cnt = 0;
                    s->aux_bit_cnt = 0;
                    memset(s->xt, 0, sizeof(s->xt));
                    memset(s->x, 0, sizeof(s->x));
                    memset(s->ww, 0, sizeof(s->ww));
                    s->viterbi.ptr = 0;
                    s->viterbi.windup = 15;
                    span_log(s->logging, SPAN_LOG_FLOW,
                             "Rx - DATA mode: parms b=%d k=%d q=%d m=%d p=%d j=%d l=%d r=%d w=%d\n",
                             s->parms.b, s->parms.k, s->parms.q, s->parms.m,
                             s->parms.p, s->parms.j, s->parms.l, s->parms.r, s->parms.w);
                    s->stage = V34_RX_STAGE_DATA;
                    if (s->duplex)
                        report_status_change(s, SIG_STATUS_TRAINING_SUCCEEDED);
                    /*endif*/
                    break;
                }
                /*endif*/
                if (s->mp_seen == 1  &&  s->mp_remote_ack_seen)
                {
                    /* We've received MP' (with ack bit); now just wait for E.
                       Avoid relocking churn that can mask E. */
                    continue;
                }
                /*endif*/

                /* Detect 17x'1' + start '0' + type bit only when not already
                   collecting an MP frame. Otherwise we can retrigger on long runs
                   of ones and keep resetting frame alignment. */
                if (s->mp_frame_pos == 0)
                {
                    int preamble_score;

                    preamble_score = mp_preamble_score(s->bitstream);
                    if (preamble_score >= MP_PREAMBLE_SCORE_MIN
                        && mp_preamble_has_start_zero(s->bitstream)
                        && mp_preamble_has_sync_ones(s->bitstream))
                    {
                        int type;
                        char tail[33];

                        mp_seed_frame_prefix(s->mp_frame_bits, s->bitstream);
                        type = s->mp_frame_bits[18];
                        if (expected_mp_type >= 0  &&  type != expected_mp_type)
                            continue;
                        /*endif*/
                        if (type == 0 && s->mp_frame_bits[19] != 0)
                            continue;
                        /*endif*/
                        s->mp_frame_target = (type == 1)  ?  188  :  88;
                        s->mp_frame_pos = 19;
                        s->mp_count = 0;
                        s->bit_count = 0;
                        s->mp_early_rejects = 0;
                        v34_rx_log_mp_diag_state(s, V34_MP_DIAG_STATE_DET_INFO, "MP preamble found; collecting frame");

                        if (s->mp_seen == 0)
                        {
                            bits32_to_str(s->bitstream, tail);
                            span_log(s->logging, SPAN_LOG_FLOW,
                                     "Rx - Phase 4: MP preamble detected (baud %d): "
                                     "score=%d/18 17x'1'+start(0)+type(%d), target=%d bits, "
                                     "frame body starts at frame_idx=19 (includes inserted start bits), tail=0b%s\n",
                                     s->duration, preamble_score, type, s->mp_frame_target, tail);
                        }
                        continue;
                    }
                    /*endif*/
                }
                /*endif*/

                if (s->mp_frame_pos > 0  &&  s->mp_frame_pos < s->mp_frame_target)
                {
                    bool crc_good;
                    bool fill_good;
                    bool starts_good;
                    int start_err_count;
                    int type;
                    uint16_t rx_crc;
                    uint16_t residual_crc;

                    s->mp_frame_bits[s->mp_frame_pos++] = bits[i];
                    s->bit_count = s->mp_frame_pos - 19;
                    {
                        int type_now;
                        int idx;

                        idx = s->mp_frame_pos - 1;
                        type_now = s->mp_frame_bits[18];
                        if (!mp_start_bit_ok(type_now, idx, bits[i]))
                        {
                            int data_idx;
                            int start_err_accept_max;

                            data_idx = mp_data_bit_index(type_now, idx);
                            s->mp_early_rejects++;
                            /* MP0 often suffers a transient inserted-start mismatch
                               from one-bit alignment wobble. Allow a small number
                               before dropping lock so CRC/slip recovery can run. */
                            start_err_accept_max = (type_now == 1)
                                                   ? MP1_START_ERR_ACCEPT_MAX
                                                   : MP_EARLY_START_ERR_MAX;
                            if (s->mp_early_rejects <= 3  ||  (s->mp_early_rejects % 8) == 0)
                            {
                                span_log(s->logging, SPAN_LOG_FLOW,
                                         "Rx - Phase 4: MP%d start-bit mismatch at frame_idx=%d body_idx=%d data_idx=%d value=%d "
                                         "(expected 0), start_err_count=%d max=%d%s\n",
                                         type_now, idx, idx - 19 + 1, data_idx, bits[i],
                                         s->mp_early_rejects, start_err_accept_max,
                                         (s->mp_early_rejects > start_err_accept_max) ? " (rejecting lock immediately)" : " (continuing until CRC)");
                            }
                            /*endif*/
                            if (s->mp_early_rejects > start_err_accept_max)
                            {
                                mp_unlock_after_reject(s, true);
                                v34_rx_log_mp_diag_state(s, V34_MP_DIAG_STATE_DET_SYNC, "start-bit mismatch limit exceeded; dropping lock");
                                break;
                            }
                            /*endif*/
                        }
                        /*endif*/
                    }
                    if (s->mp_hypothesis < 0)
                        break;
                    /*endif*/
                    {
                        int frame_idx;
                        int type_now;
                        bool is_inserted_start;
                        bool log_body_bit;

                        frame_idx = s->mp_frame_pos - 1;
                        type_now = s->mp_frame_bits[18];
                        if (frame_idx == 19  &&  bits[i] != 0)
                        {
                            int early_reject_max;

                            early_reject_max = (type_now == 1)
                                               ? MP1_START_ERR_ACCEPT_MAX
                                               : MP_EARLY_START_ERR_MAX;
                            s->mp_early_rejects++;
                            if (s->mp_early_rejects <= 3  ||  (s->mp_early_rejects % 8) == 0)
                            {
                                span_log(s->logging, SPAN_LOG_FLOW,
                                         "Rx - Phase 4: MP%d reserved bit mismatch at frame_idx=19 value=%d "
                                         "(expected 0), early_reject_count=%d max=%d%s\n",
                                         type_now, bits[i], s->mp_early_rejects, early_reject_max,
                                         (s->mp_early_rejects > early_reject_max) ? " (rejecting lock immediately)" : " (continuing until CRC)");
                            }
                            /*endif*/
                            if (s->mp_early_rejects > early_reject_max)
                            {
                                mp_unlock_after_reject(s, true);
                                v34_rx_log_mp_diag_state(s, V34_MP_DIAG_STATE_DET_SYNC, "reserved bit mismatch; dropping lock");
                                break;
                            }
                            /*endif*/
                        }
                        /*endif*/
                        is_inserted_start = (frame_idx == 34 || frame_idx == 51 || frame_idx == 68)
                                            || (type_now == 1
                                                && (frame_idx == 85
                                                    || frame_idx == 102
                                                    || frame_idx == 119
                                                    || frame_idx == 136
                                                    || frame_idx == 153
                                                    || frame_idx == 170));
                        /* Keep MP body logs lightweight:
                           - MP0: early window + periodic checkpoints + inserted starts
                           - MP1: slightly denser early window + periodic checkpoints + inserted starts */
                        log_body_bit = (s->mp_frame_target <= 88)
                                       ? (s->bit_count <= 4
                                          || (s->bit_count % 16) == 0
                                          || is_inserted_start)
                                       : (s->bit_count <= 16
                                       || (s->bit_count % 16) == 0
                                       || is_inserted_start);
                        if (log_body_bit  &&  s->mp_seen == 0)
                        {
                            int data_idx;

                            data_idx = mp_data_bit_index(type_now, frame_idx);
                            span_log(s->logging, SPAN_LOG_FLOW,
                                     "Rx - Phase 4 MP body_idx=%d frame_idx=%d data_idx=%d value=%d frame_pos=%d/%d\n",
                                     s->bit_count, frame_idx, data_idx, bits[i],
                                     s->mp_frame_pos, s->mp_frame_target);
                        }
                        /*endif*/
                    }

                    if (s->mp_frame_pos < s->mp_frame_target)
                        continue;
                    /*endif*/

                    type = s->mp_frame_bits[18];
                    if (!mp_start_bit_ok(type, 17, s->mp_frame_bits[17]))
                    {
                        s->mp_early_rejects++;
                        span_log(s->logging, SPAN_LOG_FLOW,
                                 "Rx - Phase 4: MP%d start-bit mismatch at frame_idx=17 body_idx=0 data_idx=-1 value=%d "
                                 "(expected 0), start_err_count=%d (continuing until CRC)\n",
                                 type, s->mp_frame_bits[17], s->mp_early_rejects);
                    }
                    /*endif*/
                    crc_good = mp_crc_ok(s->mp_frame_bits, type, &rx_crc, &residual_crc);
                    fill_good = mp_fill_ok(s->mp_frame_bits, type);
                    if (!(crc_good  &&  fill_good))
                    {
                        int recovered_slip;
                        int recovered_boundary;
                        int recovered_boundary_2;
                        int recovered_slip_2;
                        int recovered_changes;

                        recovered_slip = 0;
                        recovered_boundary = 0;
                        recovered_boundary_2 = 0;
                        recovered_slip_2 = 0;
                        recovered_changes = 0;
                        if (mp_try_slip_recovery(s->mp_frame_bits, type, s->mp_frame_target, &recovered_slip))
                        {
                            crc_good = mp_crc_ok(s->mp_frame_bits, type, &rx_crc, &residual_crc);
                            fill_good = mp_fill_ok(s->mp_frame_bits, type);
                            span_log(s->logging, SPAN_LOG_FLOW,
                                     "Rx - Phase 4: MP%d recovered via bit-slip=%d before CRC/fill check\n",
                                     type, recovered_slip);
                        }
                        else if (mp_try_boundary_slip_recovery(s->mp_frame_bits, type, s->mp_frame_target, &recovered_boundary, &recovered_slip))
                        {
                            crc_good = mp_crc_ok(s->mp_frame_bits, type, &rx_crc, &residual_crc);
                            fill_good = mp_fill_ok(s->mp_frame_bits, type);
                            span_log(s->logging, SPAN_LOG_FLOW,
                                     "Rx - Phase 4: MP%d recovered via boundary-slip at frame_idx=%d slip=%d before CRC/fill check\n",
                                     type, recovered_boundary, recovered_slip);
                        }
                        else if (mp_try_boundary_double_slip_recovery(s->mp_frame_bits,
                                                                       type,
                                                                       s->mp_frame_target,
                                                                       &recovered_boundary,
                                                                       &recovered_slip,
                                                                       &recovered_boundary_2,
                                                                       &recovered_slip_2))
                        {
                            crc_good = mp_crc_ok(s->mp_frame_bits, type, &rx_crc, &residual_crc);
                            fill_good = mp_fill_ok(s->mp_frame_bits, type);
                            span_log(s->logging, SPAN_LOG_FLOW,
                                     "Rx - Phase 4: MP%d recovered via double-boundary-slip (%d,%d) and (%d,%d) before CRC/fill check\n",
                                     type,
                                     recovered_boundary,
                                     recovered_slip,
                                     recovered_boundary_2,
                                     recovered_slip_2);
                        }
                        else if (mp_try_boundary_bruteforce_recovery(s->mp_frame_bits,
                                                                      type,
                                                                      s->mp_frame_target,
                                                                      &recovered_changes))
                        {
                            crc_good = mp_crc_ok(s->mp_frame_bits, type, &rx_crc, &residual_crc);
                            fill_good = mp_fill_ok(s->mp_frame_bits, type);
                            span_log(s->logging, SPAN_LOG_FLOW,
                                     "Rx - Phase 4: MP%d recovered via boundary-bruteforce (%d boundary adjustments) before CRC/fill check\n",
                                     type, recovered_changes);
                        }
                        /*endif*/
                    }
                    /*endif*/
                    start_err_count = mp_start_error_count(s->mp_frame_bits, type, s->mp_frame_target);
                    starts_good = (start_err_count == 0);
                    if (s->mp_seen == 0  ||  !crc_good  ||  !fill_good  ||  !starts_good)
                    {
                        bool log_reject_detail;

                        log_reject_detail = (s->duration <= PHASE4_MP_REJECT_DETAIL_LOG_INTERVAL
                                             || (s->duration % PHASE4_MP_REJECT_DETAIL_LOG_INTERVAL) == 0
                                             || !fill_good
                                             || !starts_good
                                             || (crc_good && fill_good));
                        if (log_reject_detail)
                        {
                            log_mp_frame_diag(s, s->mp_frame_bits, type, crc_good, rx_crc, residual_crc, fill_good);
                        }
                    }
                    {
                        bool frame_accepted;
                        bool first_mp_accept;
                        int start_err_accept_max;
                        bool starts_acceptable;

                        frame_accepted = false;
                        first_mp_accept = (s->mp_seen == 0);
                        start_err_accept_max = (type == 1)
                                               ? MP1_START_ERR_ACCEPT_MAX
                                               : MP_EARLY_START_ERR_MAX;
                        starts_acceptable = (start_err_count <= start_err_accept_max);
                        if (crc_good  &&  fill_good)
                        {
                            bool semantic_good;

                            semantic_good = true;
                            if (s->duplex)
                            {
                                mp_pack_for_parser(s->info_buf, s->mp_frame_bits, type);
                                if (first_mp_accept  ||  type == 1  ||  !s->mp_remote_ack_seen)
                                {
                                    process_rx_mp(s, &mp, s->info_buf);
                                    semantic_good = mp_semantic_ok_phase4(s, &mp, type, s->mp_frame_bits);
                                    if (semantic_good)
                                    {
                                        if (mp.mp_acknowledged)
                                            s->mp_remote_ack_seen = 1;
                                        /*endif*/
                                        t = ((v34_state_t *) ((char *)(s) - offsetof(v34_state_t, rx)));
                                        if (mp.type == 1)
                                            memcpy(&t->tx.precoder_coeffs, mp.precoder_coeffs, sizeof(t->tx.precoder_coeffs));
                                        /*endif*/
                                        if (set_trellis_mode(t, mp.trellis_size))
                                        {
                                            span_log(&t->logging, SPAN_LOG_FLOW,
                                                     "Rx - Unexpected trellis size code %d\n", mp.trellis_size);
                                            semantic_good = false;
                                        }
                                        /* Update RX data mode parameters from MP-negotiated bit rate.
                                           We receive from the far end:
                                           - If we are answerer: receive at bit_rate_c_to_a
                                           - If we are caller: receive at bit_rate_a_to_c
                                           MP bit_rate field N means N*2400 bps; convert to
                                           internal bit_rate_code = (N-1)*2 */
                                        {
                                            int rx_rate_n;

                                            rx_rate_n = s->calling_party
                                                      ? mp.bit_rate_a_to_c
                                                      : mp.bit_rate_c_to_a;
                                            s->bit_rate = (rx_rate_n - 1) * 2;
                                            v34_set_working_parameters(&s->parms, s->baud_rate, s->bit_rate,
                                                                       mp.expanded_shaping);
                                            span_log(s->logging, SPAN_LOG_FLOW,
                                                     "Rx - Phase 4: updated parms from MP: rate=%d bps (N=%d code=%d) "
                                                     "b=%d k=%d q=%d m=%d p=%d j=%d l=%d\n",
                                                     rx_rate_n * 2400, rx_rate_n, s->bit_rate,
                                                     s->parms.b, s->parms.k, s->parms.q, s->parms.m,
                                                     s->parms.p, s->parms.j, s->parms.l);
                                        }
                                        /* Rate negotiation: adopt far-end's proposed rates
                                           (take min of our rate and their rate for each direction) */
                                        {
                                            int neg_a2c;
                                            int neg_c2a;

                                            neg_a2c = mp.bit_rate_a_to_c;
                                            neg_c2a = mp.bit_rate_c_to_a;
                                            if (t->tx.mp.bit_rate_a_to_c > neg_a2c)
                                                t->tx.mp.bit_rate_a_to_c = neg_a2c;
                                            if (t->tx.mp.bit_rate_c_to_a > neg_c2a)
                                                t->tx.mp.bit_rate_c_to_a = neg_c2a;
                                            span_log(s->logging, SPAN_LOG_FLOW,
                                                     "Rx - Phase 4: rate negotiation: a2c=%d bps c2a=%d bps\n",
                                                     t->tx.mp.bit_rate_a_to_c * 2400,
                                                     t->tx.mp.bit_rate_c_to_a * 2400);
                                        }
                                    }
                                    /*endif*/
                                }
                                /*endif*/
                            }
                            /*endif*/
                            if (semantic_good)
                            {
                                if (!starts_good)
                                {
                                    span_log(s->logging, SPAN_LOG_FLOW,
                                             "Rx - Phase 4: MP%d accepting frame with %d start-bit mismatches because CRC/fill are valid\n",
                                             type, start_err_count);
                                }
                                /*endif*/
                                if (first_mp_accept)
                                {
                                    span_log(s->logging, SPAN_LOG_FLOW,
                                             "Rx - Phase 4: MP%d frame accepted (%d bits)\n",
                                             type, s->mp_frame_target);
                                }
                                s->mp_seen = 1;
                                if (s->mp_accepted_baud == 0)
                                    s->mp_accepted_baud = s->duration;
                                s->mp_early_rejects = 0;
                                if (first_mp_accept)
                                    v34_rx_log_mp_diag_state(s, V34_MP_DIAG_STATE_DET_INFO, "MP frame accepted; awaiting E");
                                else if (s->mp_remote_ack_seen)
                                    v34_rx_log_mp_diag_state(s, V34_MP_DIAG_STATE_DET_INFO, "MP' (ack) accepted; awaiting E");
                                frame_accepted = true;
                            }
                            else
                            {
                                /* CRC/fill/starts passed but semantic parse failed.
                                   Prefer a local re-acquire on the same hypothesis
                                   before dropping back to global search. */
                                s->mp_early_rejects = 0;
                                s->mp_count = 0;
                                span_log(s->logging, SPAN_LOG_FLOW,
                                         "Rx - Phase 4: MP%d rejected (semantic checks failed)\n",
                                         type);
                                span_log(s->logging, SPAN_LOG_FLOW,
                                         "Rx - Phase 4: keeping MP hypothesis=%d after semantic reject; retrying local preamble reacquire\n",
                                         s->mp_hypothesis);
                                v34_rx_log_mp_diag_state(s, V34_MP_DIAG_STATE_DET_SYNC, "semantic reject; reacquiring preamble");
                            }
                            /*endif*/
                        }
                        else
                        {
                            bool keep_hypothesis;

                            /* If all inserted start bits were consistent, this is often
                               a boundary/timing wobble rather than a bad phase hypothesis.
                               Keep the current hypothesis and re-acquire the next preamble
                               locally instead of immediately jumping to global search. */
                            keep_hypothesis = (s->mp_hypothesis >= 0
                                               && starts_acceptable);
                            span_log(s->logging, SPAN_LOG_FLOW,
                                     "Rx - Phase 4: MP%d rejected (crc_ok=%d fill_ok=%d starts_ok=%d starts_acceptable=%d start_err_count=%d max=%d)\n",
                                     type, crc_good, fill_good, starts_good, starts_acceptable, start_err_count, start_err_accept_max);
                            /* Dump first 70 frame bits for diagnosis */
                            {
                                char dump[200];
                                int dlen = (s->mp_frame_target < 90) ? s->mp_frame_target : 90;
                                int d;
                                for (d = 0; d < dlen && d < (int)sizeof(dump) - 1; d++)
                                    dump[d] = '0' + (s->mp_frame_bits[d] & 1);
                                dump[d] = '\0';
                                span_log(s->logging, SPAN_LOG_FLOW,
                                         "Rx - Phase 4: MP frame bits[0..%d]: %s\n",
                                         dlen - 1, dump);
                            }
                            /* Majority-vote accumulator for MP0 frames */
                            if (type == 0  &&  s->mp_frame_target == 88)
                            {
                                int vi;

                                /* Reset accumulator if hypothesis changed */
                                if (s->mp_hypothesis != s->mp0_vote_hyp)
                                {
                                    memset(s->mp0_vote_counts, 0, sizeof(s->mp0_vote_counts));
                                    s->mp0_vote_frames = 0;
                                    s->mp0_vote_hyp = s->mp_hypothesis;
                                }
                                /* Accumulate: +1 for '1', -1 for '0' */
                                for (vi = 0;  vi < 88;  vi++)
                                    s->mp0_vote_counts[vi] += (s->mp_frame_bits[vi] & 1) ? 1 : -1;
                                s->mp0_vote_frames++;

                                if (s->mp0_vote_frames <= 2 || (s->mp0_vote_frames % 4) == 0)
                                {
                                    span_log(s->logging, SPAN_LOG_FLOW,
                                             "Rx - Phase 4: MP0 vote accumulator: %d frames (hyp=%d)\n",
                                             s->mp0_vote_frames, s->mp0_vote_hyp);
                                }

                                /* Try majority-vote after every 3+ frames */
                                if (s->mp0_vote_frames >= 3)
                                {
                                    uint8_t voted_bits[88];
                                    uint16_t vote_rx_crc;
                                    uint16_t vote_res_crc;
                                    bool vote_crc_ok;
                                    bool vote_fill_ok;

                                    for (vi = 0;  vi < 88;  vi++)
                                        voted_bits[vi] = (s->mp0_vote_counts[vi] > 0) ? 1 : 0;
                                    /* Force known structural bits */
                                    voted_bits[17] = 0;  /* start bit */
                                    voted_bits[18] = 0;  /* type = MP0 */
                                    voted_bits[19] = 0;  /* reserved */
                                    voted_bits[34] = 0;  /* start bit */
                                    voted_bits[51] = 0;  /* start bit */
                                    voted_bits[68] = 0;  /* start bit */

                                    vote_crc_ok = mp_crc_ok(voted_bits, 0, &vote_rx_crc, &vote_res_crc);
                                    vote_fill_ok = mp_fill_ok(voted_bits, 0);

                                    span_log(s->logging, SPAN_LOG_FLOW,
                                             "Rx - Phase 4: MP0 majority-vote result: crc_ok=%d fill_ok=%d crc=0x%04X res=0x%04X (%d frames)\n",
                                             vote_crc_ok, vote_fill_ok, vote_rx_crc, vote_res_crc, s->mp0_vote_frames);

                                    if (vote_crc_ok  &&  vote_fill_ok)
                                    {
                                        int accepted_vote_frames;

                                        accepted_vote_frames = s->mp0_vote_frames;
                                        /* Replace frame bits with voted version and accept */
                                        memcpy(s->mp_frame_bits, voted_bits, 88);
                                        crc_good = true;
                                        fill_good = true;
                                        keep_hypothesis = true;
                                        s->mp0_vote_frames = 0;
                                        memset(s->mp0_vote_counts, 0, sizeof(s->mp0_vote_counts));

                                        /* Process the accepted frame */
                                        if (s->duplex)
                                        {
                                            mp_pack_for_parser(s->info_buf, s->mp_frame_bits, 0);
                                            process_rx_mp(s, &mp, s->info_buf);
                                            if (mp_semantic_ok_phase4(s, &mp, 0, s->mp_frame_bits))
                                            {
                                                if (mp.mp_acknowledged)
                                                    s->mp_remote_ack_seen = 1;
                                                t = ((v34_state_t *) ((char *)(s) - offsetof(v34_state_t, rx)));
                                                (void) set_trellis_mode(t, mp.trellis_size);
                                                span_log(s->logging, SPAN_LOG_FLOW,
                                                         "Rx - Phase 4: MP0 ACCEPTED via majority vote (%d frames)\n",
                                                         accepted_vote_frames);
                                                s->mp_seen = 1;
                                                if (s->mp_accepted_baud == 0)
                                                    s->mp_accepted_baud = s->duration;
                                                s->mp_early_rejects = 0;
                                                v34_rx_log_mp_diag_state(s, V34_MP_DIAG_STATE_DET_INFO, "MP frame accepted via vote; awaiting E");
                                                frame_accepted = true;
                                            }
                                        }
                                    }
                                }
                            }
                            /*endif*/
                            /* Majority-vote accumulator for MP1 frames */
                            else if (type == 1  &&  s->mp_frame_target == 188)
                            {
                                int vi;

                                if (s->mp_hypothesis != s->mp1_vote_hyp)
                                {
                                    memset(s->mp1_vote_counts, 0, sizeof(s->mp1_vote_counts));
                                    s->mp1_vote_frames = 0;
                                    s->mp1_vote_hyp = s->mp_hypothesis;
                                }

                                for (vi = 0;  vi < 188;  vi++)
                                    s->mp1_vote_counts[vi] += (s->mp_frame_bits[vi] & 1) ? 1 : -1;
                                s->mp1_vote_frames++;

                                span_log(s->logging, SPAN_LOG_FLOW,
                                         "Rx - Phase 4: MP1 vote accumulator: %d frames (hyp=%d)\n",
                                         s->mp1_vote_frames, s->mp1_vote_hyp);

                                if (s->mp1_vote_frames >= 3)
                                {
                                    uint8_t voted_bits[188];
                                    uint16_t vote_rx_crc;
                                    uint16_t vote_res_crc;
                                    bool vote_crc_ok;
                                    bool vote_fill_ok;

                                    for (vi = 0;  vi < 188;  vi++)
                                        voted_bits[vi] = (s->mp1_vote_counts[vi] > 0) ? 1 : 0;
                                    /* Force structural bits */
                                    voted_bits[17] = 0;
                                    voted_bits[18] = 1;
                                    voted_bits[19] = 0;
                                    voted_bits[34] = 0;
                                    voted_bits[51] = 0;
                                    voted_bits[68] = 0;
                                    voted_bits[85] = 0;
                                    voted_bits[102] = 0;
                                    voted_bits[119] = 0;
                                    voted_bits[136] = 0;
                                    voted_bits[153] = 0;
                                    voted_bits[170] = 0;

                                    vote_crc_ok = mp_crc_ok(voted_bits, 1, &vote_rx_crc, &vote_res_crc);
                                    vote_fill_ok = mp_fill_ok(voted_bits, 1);

                                    span_log(s->logging, SPAN_LOG_FLOW,
                                             "Rx - Phase 4: MP1 majority-vote result: crc_ok=%d fill_ok=%d crc=0x%04X res=0x%04X (%d frames)\n",
                                             vote_crc_ok, vote_fill_ok, vote_rx_crc, vote_res_crc, s->mp1_vote_frames);

                                    if (vote_crc_ok  &&  vote_fill_ok)
                                    {
                                        memcpy(s->mp_frame_bits, voted_bits, 188);
                                        crc_good = true;
                                        fill_good = true;
                                        keep_hypothesis = true;
                                        s->mp1_vote_frames = 0;
                                        memset(s->mp1_vote_counts, 0, sizeof(s->mp1_vote_counts));

                                        if (s->duplex)
                                        {
                                            mp_pack_for_parser(s->info_buf, s->mp_frame_bits, 1);
                                            process_rx_mp(s, &mp, s->info_buf);
                                            if (mp_semantic_ok_phase4(s, &mp, 1, s->mp_frame_bits))
                                            {
                                                if (mp.mp_acknowledged)
                                                    s->mp_remote_ack_seen = 1;
                                                t = ((v34_state_t *) ((char *)(s) - offsetof(v34_state_t, rx)));
                                                memcpy(&t->tx.precoder_coeffs, mp.precoder_coeffs, sizeof(t->tx.precoder_coeffs));
                                                (void) set_trellis_mode(t, mp.trellis_size);
                                                span_log(s->logging, SPAN_LOG_FLOW,
                                                         "Rx - Phase 4: MP1 ACCEPTED via majority vote\n");
                                                s->mp_seen = 1;
                                                if (s->mp_accepted_baud == 0)
                                                    s->mp_accepted_baud = s->duration;
                                                s->mp_early_rejects = 0;
                                                v34_rx_log_mp_diag_state(s, V34_MP_DIAG_STATE_DET_INFO, "MP1 accepted via vote; awaiting E");
                                                frame_accepted = true;
                                            }
                                        }
                                    }
                                }
                            }
                            /*endif*/
                            if (!frame_accepted  &&  keep_hypothesis)
                            {
                                s->mp_early_rejects = 0;
                                s->mp_count = 0;
                                span_log(s->logging, SPAN_LOG_FLOW,
                                         "Rx - Phase 4: keeping MP hypothesis=%d after CRC-only reject; retrying local preamble reacquire\n",
                                         s->mp_hypothesis);
                                v34_rx_log_mp_diag_state(s, V34_MP_DIAG_STATE_DET_SYNC, "CRC-only reject; reacquiring preamble");
                            }
                            else if (!frame_accepted)
                            {
                                /* Bad lock: drop hypothesis and resume global search. */
                                mp_unlock_after_reject(s, true);
                                v34_rx_log_mp_diag_state(s, V34_MP_DIAG_STATE_DET_SYNC, "bad lock; resuming global search");
                            }
                            /*endif*/
                        }
                        /*endif*/
                        if (frame_accepted)
                            s->mp_phase4_reject_streak = 0;
                        /*endif*/
                    }
                    if (s->mp_hypothesis >= 0)
                    {
                        s->mp_frame_pos = 0;
                        s->mp_frame_target = 0;
                    }
                    /*endif*/
                }
                /*endif*/
            }
            /*endfor*/
        }
        /*endif*/
        }
        break;

    case V34_RX_STAGE_DATA:
        /* V.34 data mode: collect equalized symbols into mapping frames (8 x 2D symbols)
           and run the full decode pipeline.
           The CMA equalizer (frozen from training) normalizes to unit magnitude.
           Training DQPSK is constant modulus, so the frozen equalizer divides by
           whatever the training amplitude was - the AGC below has already taken
           the absolute level out, so no TX-side amplitude constant applies here.  The live equalizer output requires
           a data-mode gain calibration before slicing; 70 is the measured bridge
           from its unit-radius training normalization to the Q9.7 constellation
           used by the mapper on the reference 31.2 kbit/s calls.
           (odd integers 1..43) expected by quantize_n_ways() in Q9.7 format. */
        {
            float re;
            float im;
            float transformed_re;
            float transformed_im;

            /* Take out the decision-aided derotator acquired over the CP/MP
               stretch: this is what makes the symbols coherent.  CMA (the
               wander source) is frozen in DATA, so from here only genuine
               carrier drift remains, held by the DD tracker below. */
            {
                float c = cosf((float) s->phase4_da_derot
                               *(float) (3.14159265358979/2147483648.0));
                float sn = sinf((float) s->phase4_da_derot
                                *(float) (3.14159265358979/2147483648.0));
                float dr = sym->re*c + sym->im*sn;
                float di = sym->im*c - sym->re*sn;

                re = dr;
                im = s->data_symbol_conjugate ? -di : di;
            }
            switch (s->data_symbol_rotation & 3)
            {
            case 1:
                transformed_re = -im;
                transformed_im = re;
                break;
            case 2:
                transformed_re = -re;
                transformed_im = -im;
                break;
            case 3:
                transformed_re = im;
                transformed_im = -re;
                break;
            default:
                transformed_re = re;
                transformed_im = im;
                break;
            }
            /*endswitch*/
            s->mapping_frame_buf[s->mapping_frame_count++] =
                (int16_t)(transformed_re * 128.0f * s->data_symbol_scale);
            s->mapping_frame_buf[s->mapping_frame_count++] =
                (int16_t)(transformed_im * 128.0f * s->data_symbol_scale);

            /* Decision-directed carrier tracking against the DATA
               constellation.  Root cause (2026-07-23, offline 4th-power
               analysis of the winner capture): the V.90 answerer's Phase 3/4
               receive path is never phase-locked -- CP/MP decode succeeds
               because those sequences are DIFFERENTIALLY encoded (8.5.2/V.90)
               and the 24-hypothesis machinery absorbs the unknown rotation,
               while carrier tracking is deliberately frozen through PHASE4_MP
               (phase4_trn_should_freeze_tracking).  Data mode is the first
               thing in the call that needs true coherence, so it must pull in
               and hold its own lock.  The phase detector is the same
               Im(sym x conj(target)) form as the training loop, with the
               target taken from the nearest odd-integer constellation point
               and normalized so the error is sin(delta-phi) regardless of
               ring radius.  The 90-degree acquisition ambiguity is absorbed
               by the differential quadrant bits (V.34 data framing).
               ME_V34_DATA_CARRIER_TRACK=0 disables for A/B. */
            {
                static int dd_enabled = -1;

                if (dd_enabled < 0)
                {
                    const char *value = getenv("ME_V34_DATA_CARRIER_TRACK");

                    dd_enabled = (value == NULL || atoi(value) != 0);
                }
                if (dd_enabled)
                {
                    float g_re = transformed_re * s->data_symbol_scale;
                    float g_im = transformed_im * s->data_symbol_scale;
                    float t_re = 2.0f*floorf((g_re - 1.0f)/2.0f + 0.5f) + 1.0f;
                    float t_im = 2.0f*floorf((g_im - 1.0f)/2.0f + 0.5f) + 1.0f;
                    float sym_mag;
                    float tgt_mag;

                    if (t_re > 43.0f) t_re = 43.0f;
                    else if (t_re < -43.0f) t_re = -43.0f;
                    if (t_im > 43.0f) t_im = 43.0f;
                    else if (t_im < -43.0f) t_im = -43.0f;
                    sym_mag = sqrtf(g_re*g_re + g_im*g_im);
                    tgt_mag = sqrtf(t_re*t_re + t_im*t_im);
                    if (sym_mag > 0.5f && tgt_mag > 0.5f)
                    {
                        /* Phase error in the transformed (grid) domain equals
                           the error in the equalizer domain: the transform is
                           a fixed rotation/conjugation/scale, and the
                           conjugate flips the error sign, which we undo.
                           Update the zero-delay derotator (positive error =
                           received leads target = derotator must remove
                           more), gently -- data decisions are less reliable
                           than the CP/MP decision-aided ones. */
                        float error = (g_im*t_re - g_re*t_im)
                                    / (sym_mag*tgt_mag);

                        if (s->data_symbol_conjugate)
                            error = -error;
                        s->phase4_da_derot +=
                            (int32_t) (error*(1.0f/32.0f)*2147483648.0f/3.14159265f);
                    }
                }
            }
        }
        s->duration++;
        if (s->mapping_frame_count >= 16)
        {
            /* Per-frame RMS diagnostic.  MUST stay off by default on the live
               media path: at 3200 baud this is ~400 stderr lines/s, and disk
               I/O on the media clock is a proven call-killer (the buffered-tap
               lesson).  V34_DATA_FRAME_RMS_LOG=1 enables it for offline work. */
            {
                static int rms_log_enabled = -1;

                if (rms_log_enabled < 0)
                    rms_log_enabled = (getenv("V34_DATA_FRAME_RMS_LOG") != NULL);
                if (rms_log_enabled)
                {
                    float rms_sum = 0;
                    int ii;
                    for (ii = 0; ii < 16; ii++)
                        rms_sum += (float)s->mapping_frame_buf[ii] * s->mapping_frame_buf[ii];
                    fprintf(stderr, "[DATA] baud=%d frame_rms=%.1f (%.3f)\n",
                            s->duration, sqrtf(rms_sum / 16.0f),
                            sqrtf(rms_sum / 16.0f) / 128.0f);
                }
            }
            /* Dump the EXACT Q9.7 values entering the mapping-frame decoder.
               Offline diagnosis only (raw int16 LE pairs, 16 per frame). */
            {
                static int dump_initialized = 0;
                static FILE *dump_fp = NULL;

                if (!dump_initialized)
                {
                    const char *path = getenv("V34_DATA_FRAME_DUMP");

                    dump_initialized = 1;
                    if (path && *path)
                        dump_fp = fopen(path, "wb");
                }
                if (dump_fp)
                {
                    fwrite(s->mapping_frame_buf, sizeof(int16_t), 16, dump_fp);
                    fflush(dump_fp);
                }
            }
            v34_put_mapping_frame(s, s->mapping_frame_buf);
            s->mapping_frame_count = 0;
        }
        /*endif*/
        s->last_sample = *sym;
        break;

    default:
        break;
    }
    /*endswitch*/

    /* Decision-directed carrier tracking and equalizer training for DQPSK during
       Phase 3/4 training.  Snap to the nearest QPSK constellation point using a
       FIXED target magnitude (EMA of equalizer output) so the equalizer corrects
       both phase AND amplitude distortion (ISI).  Using the received magnitude as
       the target (as before) gives the equalizer zero amplitude-correction incentive
       and leaves ISI uncorrected. */
    if (v34_rx_stage_is_primary_training(s->stage))
    {
        float mag;

        mag = sqrtf(sym->re*sym->re + sym->im*sym->im);
        if (mag > 0.001f  &&  isfinite(mag)  &&  mag < 100.0f)
        {
            float error;
            float target_mag;

            /* Fixed CMA target radius: QPSK constellation at unit radius.
               Adaptive EMA tracking was causing CMA divergence when it seeded
               from weak first samples (R=0.15), making the equalizer oscillate.
               A fixed target of 1.0 normalizes the equalizer output to a known
               level regardless of input power. */
            target_mag = 1.0f;

            /* Snap to nearest of the 4 QPSK points at the FIXED target radius.
               QPSK points at (±1,±1)/√2 scaled by target_mag. */
            float s2 = target_mag * 0.7071068f;  /* target_mag/√2 */
            eq_target.re = (sym->re >= 0.0f)  ?  s2  :  -s2;
            eq_target.im = (sym->im >= 0.0f)  ?  s2  :  -s2;

            /* Keep the receiver adapting through Phase 3 once TRN is locked.
             *
             * V34_RX_STAGE_PHASE3_WAIT_S spans the peer's whole Phase 3 -- its
             * TRN *and* its Ja -- and both adaptation loops used to be off for
             * the entire stage.  That is sample-and-hold: freeze the receiver,
             * accumulate bits, and batch-search them afterwards.  It is good
             * enough to spot a periodic TRN (measured live, TRN locks at 93%)
             * but not to decode Ja, which is data -- every symbol has to be
             * right, and nothing is correcting residual carrier/gain error
             * over the ~1.5 s of TRN that precedes it.  Live captures show
             * exactly that split: real structure through TRN, then ~50% ones
             * on all 24 hypotheses once Ja starts.
             *
             * Gated on phase3_trn_lock_hyp >= 0 so this only switches on once
             * TRN is locked, i.e. well past the S detector -- S is carried as
             * phase reversals and must not be tracked out.  Set
             * ME_V34_TRACK_PHASE3=0 to restore the frozen behaviour. */
            if (s->stage == V34_RX_STAGE_PHASE4_TRN
                || v34_rx_stage_is_phase4_frame(s->stage)
                || (s->stage == V34_RX_STAGE_PHASE3_WAIT_S
                    && s->phase3_tracking_armed
                    && phase3_tracking_enabled()))
            {
                /* CMA (blind) equalizer — during Phase 3 TRN refinement and
                   Phase 4.  Phase 4 needs CMA to adapt equalizer gain for
                   variable signal levels on analog/SIP channels.
                   Stop CMA once TX enters data mode — echo from the high-power
                   data constellation would cause CMA to diverge.

                   ME_V34_FREEZE_CMA_DURING_MP (experimental, 2026-07-19): live
                   interop showed MP0 frames consistently correct for their
                   first ~35-50 bits, then close to 50% wrong on every
                   should-be-0 structural bit afterward -- the signature of a
                   clean signal whose equalizer has drifted away from its
                   TRN-converged state, not of noise or a framing bug (carrier
                   tracking is already frozen during MP for the same class of
                   risk; CMA is not). Gated behind an env var rather than
                   flipped outright since the existing comment states MP-time
                   CMA adaptation was a deliberate, tested choice for gain
                   variability -- freezing it needs live A/B verification
                   before it can safely become the default. */
                v34_state_t *t_cma = ((v34_state_t *) ((char *)(s) - offsetof(v34_state_t, rx)));
                /* V34_RX_STAGE_V90_CP must behave like the stateless
                   matched-filter fallback (v90_cp_live_direct_recover):
                   the CPt/CP payload is up to 428 bits / 214 symbols, and
                   CMA is phase-blind on the QPSK-only CP signal.  Letting it
                   re-adapt through the frame rotates the constellation
                   away from its TRN-converged state -- the synchronous path
                   repeatedly achieves a perfect 18/18 preamble lock then
                   fails every payload structural bit (live 2026-07-26:
                   crc=0 structure=N, source=spandsp accepted 0 frames).
                   Freeze CMA unconditionally for the dedicated V.90 CP
                   stage; ordinary V.34 MP keeps its env-gated behaviour so
                   this cannot contaminate normal MP acquisition (verified
                   by test_v90_v34_rx_stage_isolation). */
                bool freeze_mp_cma = (s->stage == V34_RX_STAGE_V90_CP)
                                  || (v34_rx_stage_is_phase4_frame(s->stage)
                                      && getenv("ME_V34_FREEZE_CMA_DURING_MP"));
                /* Once the decision-aided Phase 4 tracker owns the taps
                   (data-aided LMS above), CMA must stand down or the two
                   fight: CMA's phase-blind gradient re-randomizes the phase
                   the DA loop just fixed. */
                bool da_owns_eq = v34_rx_stage_is_phase4_frame(s->stage)
                               && s->phase4_da_seeded;
                if (!t_cma->tx.tx_data_mode && !freeze_mp_cma && !da_owns_eq)
                    tune_equalizer_cma(s, sym);
            }
            /*endif*/

            /* Re-enabled carrier tracking — test 4 showed MP detection worked
               better with carrier tracking on.  CMA equalization now provides
               more stable magnitude for eq_target, improving tracking quality. */
            if ((s->stage != V34_RX_STAGE_PHASE3_WAIT_S
                 || (s->phase3_tracking_armed && phase3_tracking_enabled()))
                && !phase4_trn_should_freeze_tracking(s))
            {
                error = sym->im*eq_target.re - sym->re*eq_target.im;
                s->v34_carrier_phase_rate += (int32_t)(s->carrier_track_i*error);
                s->carrier_phase += (int32_t)(s->carrier_track_p*error);
            }
            }
        /*endif*/
    }
    /*endif*/

    s->last_sample = *sym;
}
/*- End of function --------------------------------------------------------*/

static int primary_channel_rx(v34_rx_state_t *s, const int16_t amp[], int len)
{
    int i;
    int step;
#if defined(SPANDSP_USE_FIXED_POINT)
    complexi16_t z;
    complexi16_t zz;
    complexi16_t sample;
#else
    complexf_t z;
    complexf_t zz;
    complexf_t sample;
#endif
    float ii;
    float qq;
    float v;
    int32_t power;
    /* The following lead to integer values for the rx increments per symbol, for each of the 6 baud rates */
    static const int steps_per_baud[6] =
    {
        192*8000/2400,
        192*8000*7/(2400*8),
        189*8000*6/(2400*7),
        192*8000*4/(2400*5),
        192*8000*3/(2400*4),
        192*8000*7/(2400*10)
    };

    /* Use the negotiated baud rate and carrier assignment.
       baud_rate is set from INFO1a (process_rx_info1a) or v34_rx_restart.
       high_carrier is set from v34_rx_restart based on calling_party flag. */
    if (s->baud_rate < 0 || s->baud_rate > 5)
    {
        span_log(s->logging, SPAN_LOG_FLOW,
                 "Rx - ERROR: baud_rate=%d out of range (expected 0-5), forcing to 4\n",
                 s->baud_rate);
        s->baud_rate = 4;  /* V34_BAUD_RATE_3200 */
    }
    s->shaper_re = v34_rx_shapers_re[s->baud_rate][s->high_carrier];
    s->shaper_im = v34_rx_shapers_im[s->baud_rate][s->high_carrier];
    s->shaper_sets = steps_per_baud[s->baud_rate];
    /* Periodic diagnostic: log primary channel RX config on first entry and every 8000 samples */
    if (s->stage >= V34_RX_STAGE_PHASE3_TRAINING && (s->sample_time % 8000) < (unsigned)len)
    {
        span_log(s->logging, SPAN_LOG_FLOW,
                 "Rx - primary_channel_rx: baud_rate=%d high_carrier=%d carrier=%.1fHz agc=%.5f power=%ld shaper_sets=%d\n",
                 s->baud_rate, s->high_carrier, dds_frequencyf(s->v34_carrier_phase_rate),
                 (double)s->agc_scaling, (long)power_meter_current(&s->power), s->shaper_sets);
    }
    /* Follow the far end when it abandons Phase 3/4 and restarts.
     *
     * A V.90 analogue modem that gives up (constellation design failure, an
     * unanswered DIL request, a Phase 3 timeout) goes SILENCERETRAIN ->
     * TONE_AB -> Phase 1.  Measured on the d-modem rig, that silence is about
     * 80 ms.  Without noticing it we keep transmitting Phase 3/4 at a peer
     * that is no longer listening, and our own receiver stays parked in a
     * Phase 3/4 stage decoding the peer's Phase 1/2 tones as though they were
     * Phase 4 frames -- which is what the long MP-CRC investigation was
     * actually chasing (see rig/README.md).  Report it so the application can
     * follow the peer back to Phase 2 and get another attempt, rather than
     * hanging until the call dies.
     *
     * Deliberately keyed on sustained near-silence rather than on tone
     * detection: the tone detectors live in info_rx(), which is not the
     * demodulator running during Phase 3/4, and silence is the unambiguous
     * first half of every retrain the peer can start. */
    /* Do not run this detector in PHASE3_TRAINING.  V.90 §9.3.2.2 requires
       the analogue modem to begin Phase 3 with 70 +/- 5 ms of silence, so the
       old 60 ms threshold deterministically classified the legitimate startup
       gap as SILENCERETRAIN.  Once the digital transmitter reaches Jd the
       dedicated expect-silence/energy detector below owns abandonment; this
       detector remains useful in the later Phase 4 stages.

       Likewise, do not arm it during the initial CPt acquisition window in
       V90_CP.  The project-owned V.90 transmitter hands the receiver
       directly from DIL to CPt acquisition, and the analogue modem may send
       CPt, SCR, or silence while it searches for a usable upstream path.
       V.90 gives this exchange a 15 s plus round-trip-delay budget.  A short
       silence inside that interval must not stop our unbarred Ri and restart
       Phase 2.  The independent Tone A detector below remains armed for an
       explicit peer retrain request. */
    int phase4_cp_wait_bauds = 0;
    if (s->baud_rate >= 0 && s->baud_rate < 6)
        phase4_cp_wait_bauds = PHASE4_CP_ACQUISITION_WAIT_SECONDS
                               * baud_rate_parameters[s->baud_rate].baud_rate;

    if (v34_rx_stage_is_primary_training(s->stage)
        && s->stage > V34_RX_STAGE_PHASE3_TRAINING
        && !(s->stage == V34_RX_STAGE_V90_CP
             && s->duration < phase4_cp_wait_bauds))
    {
        int32_t retrain_floor = (s->info_rx_carrier_ref > 0)
                                ? s->info_rx_carrier_ref/64
                                : 0;

        for (i = 0;  i < len;  i++)
        {
            int32_t mag = (int32_t) amp[i]*amp[i];

            if (retrain_floor > 0  &&  mag < retrain_floor)
            {
                s->phase34_silence_samples++;
            }
            else
            {
                s->phase34_silence_samples = 0;
                /* Energy is back.  Re-arm so a later retrain is reported too. */
                s->phase34_retrain_reported = false;
            }
            /*endif*/
            /* 480 samples = 60 ms, comfortably inside the ~80 ms the peer
               spends in SILENCERETRAIN but far longer than any gap inside a
               live Phase 3/4 signal. */
            if (s->phase34_silence_samples >= 480  &&  !s->phase34_retrain_reported)
            {
                s->phase34_retrain_reported = true;
                s->received_event = V34_EVENT_PEER_RETRAIN;
                span_log(s->logging, SPAN_LOG_FLOW,
                         "Rx - far end abandoned Phase 3/4 (%d ms silence in stage %s); "
                         "reporting peer retrain\n",
                         s->phase34_silence_samples/8, v34_rx_stage_to_str(s->stage));
            }
            /*endif*/
        }
        /*endfor*/
    }
    else
    {
        s->phase34_silence_samples = 0;
        s->phase34_retrain_reported = false;
    }
    /*endif*/
    /* The spec-mandated form of the same check: V.90 9.3.1 and 9.4.1 both
     * require "If Tone A is detected during Phase 3/4, the digital modem
     * shall respond to retrain according to 9.5.1.2".  A peer initiating a
     * retrain (9.5.2.1) sends 70 +/- 5 ms of silence and then holds 2400 Hz
     * Tone A until it hears our Tone B -- the SmartLink peer gives up and
     * drops the call about 3.1 s in.  The silence detector above can miss
     * the gap when transport filtering rings into it (the interop rig's
     * 257-tap polyphase resampler shaves the observed 80 ms gap below the
     * 60 ms threshold), so detect the tone itself: a Goertzel bin at
     * 2400 Hz against total block energy.  A modulated primary channel
     * (CPt/SCR/CP on the 1800 Hz carrier) spreads its power across the
     * band, and the periodic Phase 3 line spectra (S at 600/3000 Hz, J/Ja
     * harmonics) never put most of it in one bin, so require a dominant,
     * sustained single-bin ratio. */
    if (s->v90_mode
        && v34_rx_stage_is_primary_training(s->stage))
    {
        /* 2*cos(2*pi*2400/8000) */
        static const float tone_a_coeff = -0.6180339887f;
        /* 160 samples = 20 ms per block; bin 48 lands exactly on 2400 Hz. */
        static const int tone_a_block = 160;

        for (i = 0;  i < len;  i++)
        {
            float x = (float) amp[i];
            float g0 = x + tone_a_coeff*s->phase34_tone_a_g1 - s->phase34_tone_a_g2;

            s->phase34_tone_a_g2 = s->phase34_tone_a_g1;
            s->phase34_tone_a_g1 = g0;
            s->phase34_tone_a_energy += x*x;
            if (++s->phase34_tone_a_samples >= tone_a_block)
            {
                float g1 = s->phase34_tone_a_g1;
                float g2 = s->phase34_tone_a_g2;
                float tone_power = g1*g1 + g2*g2 - tone_a_coeff*g1*g2;
                /* For a full-block sine, tone_power ~= energy*N/2, so this
                   ratio approaches 1.0 for a pure tone. */
                float denom = s->phase34_tone_a_energy*(float) tone_a_block*0.5f;
                bool tonal = false;

                /* Energy floor: mean square > 100^2 keeps line noise and the
                   Jd-wait silence from ever qualifying. */
                if (s->phase34_tone_a_energy > 10000.0f*(float) tone_a_block
                    &&  denom > 0.0f
                    &&  tone_power > 0.70f*denom)
                {
                    tonal = true;
                }
                /*endif*/
                if (tonal)
                    s->phase34_tone_a_blocks++;
                else
                    s->phase34_tone_a_blocks = 0;
                /*endif*/
                s->phase34_tone_a_g1 = 0.0f;
                s->phase34_tone_a_g2 = 0.0f;
                s->phase34_tone_a_energy = 0.0f;
                s->phase34_tone_a_samples = 0;
                /* 4 blocks = 80 ms, satisfying the "more than 50 ms" of
                   9.5.1.2 with margin against a chance tonal block. */
                if (s->phase34_tone_a_blocks >= 4  &&  !s->phase34_tone_a_reported)
                {
                    s->phase34_tone_a_reported = true;
                    s->received_event = V34_EVENT_PEER_RETRAIN;
                    span_log(s->logging, SPAN_LOG_FLOW,
                             "Rx - Tone A detected in stage %s (%d ms); peer initiated "
                             "a V.90 retrain (9.5.2.1), reporting peer retrain per 9.4.1/9.5.1.2\n",
                             v34_rx_stage_to_str(s->stage),
                             s->phase34_tone_a_blocks*tone_a_block/8);
                }
                /*endif*/
            }
            /*endif*/
        }
        /*endfor*/
    }
    else
    {
        s->phase34_tone_a_g1 = 0.0f;
        s->phase34_tone_a_g2 = 0.0f;
        s->phase34_tone_a_energy = 0.0f;
        s->phase34_tone_a_samples = 0;
        s->phase34_tone_a_blocks = 0;
        s->phase34_tone_a_reported = false;
    }
    /*endif*/
    /* The mirror image of the check above, for the Jd wait.
     *
     * PHASE3_WAIT_S sits *below* PHASE3_TRAINING in the stage enum, so it is
     * deliberately outside the silence check -- and rightly so: §9.3.2.4 makes
     * the analogue modem silent for this whole window, so silence here is the
     * normal case and carries no information.  What does carry information is
     * energy: until §9.3.2.7 starts S the peer must not be transmitting at all,
     * so sustained energy that never resolves into S means it has abandoned
     * V.90.  Verified on the d-modem rig: its V90AutoDigitalImpDetector raises
     * "drop to V34 requested", goes JaTXMIT -> SILENCERETRAIN, and comes back
     * with Phase 1 tones while we still had ~2 s of Jd budget left -- which we
     * spent transmitting Phase 3 over its fresh Phase 1, corrupting exactly the
     * bulk-delay estimate its next attempt depends on. */
    if (s->stage == V34_RX_STAGE_PHASE3_WAIT_S
        &&
        s->phase3_expect_silence
        &&
        !s->phase3_s_present)
    {
        int32_t energy_floor = (s->info_rx_carrier_ref > 0)
                               ? s->info_rx_carrier_ref/64
                               : 0;

        for (i = 0;  i < len;  i++)
        {
            int32_t mag = (int32_t) amp[i]*amp[i];

            if (energy_floor > 0  &&  mag >= energy_floor)
                s->phase3_energy_samples++;
            else
                s->phase3_energy_samples = 0;
            /*endif*/
            /* 1600 samples = 200 ms.  Long enough that a burst of line noise
               or the leading edge of a real S cannot trip it (S is confirmed
               well inside that, and phase3_s_present gates this off), short
               enough to stop us long before the Jd budget runs out. */
            if (s->phase3_energy_samples >= 1600  &&  !s->phase3_energy_retrain_reported)
            {
                s->phase3_energy_retrain_reported = true;
                s->received_event = V34_EVENT_PEER_RETRAIN;
                span_log(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 3: %d ms of energy during the Jd wait with no S; "
                         "far end has left V.90, reporting peer retrain\n",
                         s->phase3_energy_samples/8);
            }
            /*endif*/
        }
        /*endfor*/
    }
    else
    {
        s->phase3_energy_samples = 0;
        s->phase3_energy_retrain_reported = false;
    }
    /*endif*/
    /* Dump raw Phase 3 RX audio for offline analysis */
    if (phase3_rx_dump_fp && phase3_rx_dump_count < PHASE3_RX_DUMP_SAMPLES)
    {
        int to_write = len;
        if (phase3_rx_dump_count + to_write > PHASE3_RX_DUMP_SAMPLES)
            to_write = PHASE3_RX_DUMP_SAMPLES - phase3_rx_dump_count;
        fwrite(amp, sizeof(int16_t), to_write, phase3_rx_dump_fp);
        phase3_rx_dump_count += to_write;
        if (phase3_rx_dump_count >= PHASE3_RX_DUMP_SAMPLES)
        {
            fclose(phase3_rx_dump_fp);
            phase3_rx_dump_fp = NULL;
            fprintf(stderr, "[V34 RX] Phase 3 RX audio dump complete (%d samples)\n", phase3_rx_dump_count);
        }
    }
    for (i = 0;  i < len;  i++)
    {
        s->qam_sample_time++;
        s->rrc_filter[s->rrc_filter_step] = amp[i];
        if (++s->rrc_filter_step >= V34_RX_FILTER_STEPS)
            s->rrc_filter_step = 0;
        /*endif*/

        power = power_meter_update(&s->power, amp[i]);
        s->eq_put_step -= V34_RX_PULSESHAPER_COEFF_SETS;
        step = -s->eq_put_step;
        if (step > V34_RX_PULSESHAPER_COEFF_SETS - 1)
            step = V34_RX_PULSESHAPER_COEFF_SETS - 1;
        /*endif*/
        while (step < 0)
            step += V34_RX_PULSESHAPER_COEFF_SETS;
        /*endwhile*/
#if defined(SPANDSP_USE_FIXED_POINT)
        ii = vec_circular_dot_prodi16(s->rrc_filter, (*s->shaper_re)[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
#else
        ii = vec_circular_dot_prodf(s->rrc_filter, (*s->shaper_re)[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
#endif
#if defined(SPANDSP_USE_FIXED_POINT)
        //sample.re = (ii*(int32_t) s->agc_scaling) >> 15;
        sample.re = ii*s->agc_scaling;
#else
        sample.re = ii*s->agc_scaling;
#endif
        /* Symbol timing synchronisation band edge filters */
        /* Low Nyquist band edge filter */
        v = s->pri_ted.symbol_sync_low[0]*s->pri_ted.low_band_edge_coeff[0] + s->pri_ted.symbol_sync_low[1]*s->pri_ted.low_band_edge_coeff[1] + sample.re;
        s->pri_ted.symbol_sync_low[1] = s->pri_ted.symbol_sync_low[0];
        s->pri_ted.symbol_sync_low[0] = v;
        /* High Nyquist band edge filter */
        v = s->pri_ted.symbol_sync_high[0]*s->pri_ted.high_band_edge_coeff[0] + s->pri_ted.symbol_sync_high[1]*s->pri_ted.high_band_edge_coeff[1] + sample.re;
        s->pri_ted.symbol_sync_high[1] = s->pri_ted.symbol_sync_high[0];
        s->pri_ted.symbol_sync_high[0] = v;

        /* Put things into the equalization buffer at T/2 rate. The symbol synchcronisation
           will fiddle the step to align this with the symbols. */
        if (s->eq_put_step <= 0)
        {
            /* AGC: adapt scaling until locked down.
               This normalizes equalizer input from the received power, so the
               constant-modulus PP symbols and CMA operate at the expected
               magnitude whatever level the peer transmitted at. */
            if (s->agc_scaling_save == 0.0f  &&  power > 10)
            {
                if (power > V34_AGC_POWER_MIN)
                {
                    float new_scaling;

                    new_scaling = 2.17f / sqrtf((float) power);
                    if (!isfinite(new_scaling) || new_scaling < V34_AGC_SCALING_MIN)
                        new_scaling = V34_AGC_SCALING_MIN;
                    else if (new_scaling > V34_AGC_SCALING_MAX)
                        new_scaling = V34_AGC_SCALING_MAX;
                    /*endif*/
                    s->agc_scaling = new_scaling;
                }
                /*endif*/
            }
            /*endif*/
            s->eq_put_step += s->shaper_sets/2;
#if defined(SPANDSP_USE_FIXED_POINT)
            qq = vec_circular_dot_prodi16(s->rrc_filter, (*s->shaper_im)[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
#else
            qq = vec_circular_dot_prodf(s->rrc_filter, (*s->shaper_im)[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
#endif
#if defined(SPANDSP_USE_FIXED_POINT)
            //sample.im = (qq*(int32_t) s->agc_scaling) >> 15;
            sample.im = qq*s->agc_scaling;
            z = dds_lookup_complexi16(s->carrier_phase);
#else
            sample.im = qq*s->agc_scaling;
            z = dds_lookup_complexf(s->carrier_phase);
#endif
            zz.re = sample.re*z.re - sample.im*z.im;
            zz.im = -sample.re*z.im - sample.im*z.re;
            process_primary_half_baud(s, &zz);

        }
        /*endif*/
#if defined(SPANDSP_USE_FIXED_POINT)
        dds_advance(&s->carrier_phase, s->v34_carrier_phase_rate);
#else
        dds_advancef(&s->carrier_phase, s->v34_carrier_phase_rate);
#endif
    }
    /*endfor*/
    return 0;
}
/*- End of function --------------------------------------------------------*/

/* Keep this global until the modem is VERY well tested */
SPAN_DECLARE(void) v34_put_mapping_frame(v34_rx_state_t *s, int16_t bits[16])
{
    int i;
    int j;
    int constel;
    int invert;
    complexi16_t c;
    complexi16_t p;
    complexi16_t u;
    complexi16_t v;
    complexi16_t y[2];

    /* Put the four 4D symbols (eight 2D symbols) of a mapping frame */
//#define BYPASS_VITERBI
    for (i = 0;  i < 8;  i++)
    {
        s->xt[0].re = bits[2*i];
        s->xt[0].im = bits[2*i + 1];
//printf("AMZ %p [%6d, %6d] [%8.3f, %8.3f]\n", s, s->xt[0].re, s->xt[0].im, FP_Q9_7_TO_F(s->xt[0].re), FP_Q9_7_TO_F(s->xt[0].im));
        s->yt = prediction_error_filter(s);
        quantize_n_ways(s->xy[i & 1], &s->yt);
//printf("CCC %p [%8.3f, %8.3f] [%8.3f, %8.3f] [%8.3f, %8.3f] [%8.3f, %8.3f]\n",
//       s,
//       FP_Q9_7_TO_F(s->xy[i & 1][0].re),
//       FP_Q9_7_TO_F(s->xy[i & 1][0].im),
//       FP_Q9_7_TO_F(s->xy[i & 1][1].re),
//       FP_Q9_7_TO_F(s->xy[i & 1][1].im),
//       FP_Q9_7_TO_F(s->xy[i & 1][2].re),
//       FP_Q9_7_TO_F(s->xy[i & 1][2].im),
//       FP_Q9_7_TO_F(s->xy[i & 1][3].re),
//       FP_Q9_7_TO_F(s->xy[i & 1][3].im));
        viterbi_calculate_candidate_errors(s->viterbi.error[i & 1], s->xy[i & 1], &s->yt);
#if defined(BYPASS_VITERBI)
        y[i & 1].re = s->xt[0].re;
        y[i & 1].im = s->xt[0].im;
//printf("CCD %p [%8.3f, %8.3f]\n", s, FP_Q9_7_TO_F(y[i & 1].re), FP_Q9_7_TO_F(y[i & 1].im));
#endif
        if ((i & 1))
        {
            /* Deal with super-frame sync inversion at the time the 4D pair
               enters the Viterbi decoder.  step_2d is the delayed output
               position and remains zero during windup, so using it here
               repeatedly consumed V0 bits at the start of B1. */
            if (s->parms.p > 0 && s->parms.j > 0
                && s->input_4d % (2*s->parms.p) == 0) {
                int pattern = (s->input_4d / (2*s->parms.p))
                            % (2*s->parms.j);

                invert = (0x5FEE >> pattern) & 1;
            }
            else
                invert = false;
            /*endif*/
            viterbi_update_path_metrics(&s->viterbi, s->xy, invert);
            if (++s->input_4d >= 4*s->parms.p*s->parms.j)
                s->input_4d = 0;
//printf("EEE %p %4d %4d %4d %4d %4d %4d %4d %4d (%d)\n",
//       s,
//       s->viterbi.branch_error[0],
//       s->viterbi.branch_error[1],
//       s->viterbi.branch_error[2],
//       s->viterbi.branch_error[3],
//       s->viterbi.branch_error[4],
//       s->viterbi.branch_error[5],
//       s->viterbi.branch_error[6],
//       s->viterbi.branch_error[7],
//       s->viterbi.windup);
#if defined(BYPASS_VITERBI)
            {
#else
            if (s->viterbi.windup)
            {
                /* Wait for the Viterbi buffer to fill with symbols. */
                s->viterbi.windup--;
            }
            else
            {
                viterbi_trace_back(&s->viterbi, y);
                /* The public QAM report callback normally observes the
                   equalizer input.  A non-NULL target reports the delayed
                   lattice points selected by the Viterbi traceback as well.
                   Offline receivers can use these decisions for a bounded
                   decision-directed refinement without duplicating the
                   trellis implementation. */
                if (s->qam_report)
                {
                    complexf_t decided;

                    for (j = 0;  j < 2;  j++)
                    {
                        decided.re = FP_Q9_7_TO_F(y[j].re);
                        decided.im = FP_Q9_7_TO_F(y[j].im);
                        s->qam_report(s->qam_user_data,
                                      NULL,
                                      &decided,
                                      s->qam_sample_time);
                    }
                    /*endfor*/
                }
                /*endif*/
#endif
                /* We now have two points in y to be decoded. They are in Q9.7 format. */
//printf("AAA %p [%8.3f, %8.3f] [%8.3f, %8.3f]\n",
//       s,
//       FP_Q9_7_TO_F(y[0].re),
//       FP_Q9_7_TO_F(y[0].im),
//       FP_Q9_7_TO_F(y[1].re),
//       FP_Q9_7_TO_F(y[1].im));
                for (j = 0;  j < 2;  j++)
                {
                    p = precoder_rx_filter(s);

                    c = quantize_rx(s, &p);
                    s->x[0].re = y[j].re - p.re;
                    s->x[0].im = y[j].im - p.im;
                    u.re = (y[j].re >> 7) - c.re;
                    u.im = (y[j].im >> 7) - c.im;

                    s->ww[j + 1] = get_binary_subset_label(&u);
                    v = rotate90_counterclockwise(&u, s->ww[j + 1]);
                    constel = get_inverse_constellation_point(&v);
//printf("AMQ %p %d [%d, %d] [%d, %d] %d\n", s, constel, v.re, v.im, u.re, u.im, s->ww[j + 1]);
//printf("AMQ %p [%6d, %6d] (%d) [%6d, %6d] [%8.3f, %8.3f]\n", s, v.re, v.im, s->ww[j + 1], u.re, u.im, FP_Q9_7_TO_F(y[j].re), FP_Q9_7_TO_F(y[j].im));
                    s->qbits[s->step_2d + j] = constel & s->parms.q_mask;
                    s->mjk[s->step_2d + j] = constel >> s->parms.q;
                }
                /*endfor*/
                /* Compute the I bits */
                s->ibits[s->step_2d >> 1] = (((s->ww[1] - s->ww[0]) & 3) << 1)
                                          | (((s->ww[2] - s->ww[1]) >> 1) & 1);
                s->ww[0] = s->ww[1];
                s->step_2d += 2;
                if (s->step_2d == 8)
                {
                    shell_unmap(s);
                    pack_output_bitstream(s);
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
//printf("ZAQ data frame %d, super frame %d\n", s->data_frame, s->super_frame);
                    s->step_2d = 0;
                }
                /*endif*/
            }
            /*endif*/
            s->viterbi.ptr = (s->viterbi.ptr + 1) & 0xF;
        }
        /*endif*/
    }
    /*endfor*/
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_rx_fillin(v34_state_t *s, int len)
{
    int i;

    /* We want to sustain the current state (i.e carrier on<->carrier off), and
       try to sustain the carrier phase. We should probably push the filters, as well */
    span_log(&s->logging, SPAN_LOG_FLOW, "Rx - Fill-in %d samples\n", len);
    for (i = 0;  i < len;  i++)
    {
#if defined(SPANDSP_USE_FIXED_POINT)
        dds_advance(&s->rx.carrier_phase, s->rx.v34_carrier_phase_rate);
#else
        dds_advancef(&s->rx.carrier_phase, s->rx.v34_carrier_phase_rate);
#endif
    }
    /*endfor*/
    return 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_rx(v34_state_t *s, const int16_t amp[], int len)
{
    int leny;
    int lenx;

    v34_rx_log_state_change(&s->rx);
    leny = 0;
    lenx = -1;
    do
    {
        switch (s->rx.current_demodulator)
        {
        case V34_MODULATION_V34:
            lenx = primary_channel_rx(&s->rx, &amp[leny], len - leny);
            break;
        case V34_MODULATION_CC:
            lenx = cc_rx(&s->rx, &amp[leny], len - leny);
            break;
        case V34_MODULATION_L1_L2:
            lenx = l1_l2_analysis(&s->rx, &amp[leny], len - leny);
            break;
        case V34_MODULATION_TONES:
            lenx = info_rx(&s->rx, &amp[leny], len - leny);
            break;
        }
        /*endswitch*/
        leny += lenx;
        /* Add step by step, so each segment is seen up to date */
        s->rx.sample_time += lenx;
    }
    while (lenx > 0  &&  leny < len);

    if (s->rx.received_event == V34_EVENT_TRAINING_FAILED
        && !s->rx.training_failed_reported)
    {
        span_log(&s->logging, SPAN_LOG_FLOW,
                 "Rx - signalling training failure to host\n");
        report_status_change(&s->rx, SIG_STATUS_TRAINING_FAILED);
        s->rx.training_failed_reported = true;
    }
    /*endif*/
    /* If there is any residue, this should be the end of operation of the modem,
       so we don't really need to add that residue to the sample time. */
    return leny;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_rx_set_signal_cutoff(v34_state_t *s, float cutoff)
{
    /* The 0.4 factor allows for the gain of the DC blocker */
    s->rx.carrier_on_power = (int32_t) (power_meter_level_dbm0(cutoff + 2.5f)*0.4f);
    s->rx.carrier_off_power = (int32_t) (power_meter_level_dbm0(cutoff - 2.5f)*0.4f);
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_set_put_bit(v34_state_t *s, span_put_bit_func_t put_bit, void *user_data)
{
    s->rx.put_bit = put_bit;
    s->rx.put_bit_user_data = user_data;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_set_put_aux_bit(v34_state_t *s, span_put_bit_func_t put_bit, void *user_data)
{
    s->rx.put_aux_bit = put_bit;
    s->rx.put_aux_bit_user_data = user_data;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_set_put_phase4_bit(v34_state_t *s,
                                          span_put_bit_func_t put_bit,
                                          void *user_data)
{
    s->rx.put_phase4_bit = put_bit;
    s->rx.put_phase4_bit_user_data = user_data;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_force_v90_phase4_cp_rx(v34_state_t *s)
{
    if (!s || !s->rx.v90_mode || s->rx.calling_party || !s->rx.put_phase4_bit)
        return;

    /* A V.90 analogue modem begins repeated CPt immediately after DIL.  The
       ordinary V.34 Phase 4 receiver waits for J'/TRN before scanning for MP;
       applying that gate here consumes several complete CPt repetitions.  The
       external downstream implementation has already established the phase
       boundary, so retain the freshly conditioned primary-channel frontend
       and begin the CP preamble/hypothesis search immediately. */
    s->primary_channel_active = true;
    s->rx.current_demodulator = V34_MODULATION_V34;
    s->rx.stage = V34_RX_STAGE_V90_CP;
    s->rx.duration = 0;
    s->rx.received_event = V34_EVENT_NONE;
    s->rx.bitstream = 0;
    s->rx.bit_count = 0;
    s->rx.mp_seen = 0;
    s->rx.mp_remote_ack_seen = 0;
    s->rx.mp_count = -1;
    s->rx.mp_frame_pos = 0;
    s->rx.mp_frame_target = 0;
    s->rx.mp_early_rejects = 0;
    s->rx.phase3_j_lock_hyp = -1;
    s->rx.phase4_trn_lock_hyp = -1;
    s->rx.phase4_trn_lock_score = -1;

    /* V.90 upstream uses the analogue-modem scrambler polynomial (tap 4 in
       SpanDSP's zero-based representation).  Retry rotation still explores
       alternate order/domain/tap choices after rejected CP hypotheses. */
    s->rx.scrambler_tap = 4;
    s->rx.mp_phase4_default_scrambler_tap = 4;
    s->rx.mp_phase4_default_bit_order = 0;
    s->rx.mp_phase4_default_domain = 0;
    s->rx.mp_phase4_reject_streak = 0;
    s->rx.mp_phase4_nolock_count = 0;
    s->rx.mp_phase4_alt_tap_active = 0;
    s->rx.mp_phase4_alt_order_active = 0;
    s->rx.mp_phase4_alt_domain_active = 0;
    s->rx.mp_phase4_retry_mode = 0;
    s->rx.mp_phase4_bit_order = 0;
    s->rx.mp_phase4_domain = 0;
    s->rx.mp_phase4_force_abs_active = 0;
    s->rx.mp_phase4_diff_collapse_streak = 0;
    s->rx.mp_phase4_diff_recover_streak = 0;
    s->rx.phase4_da_active = 0;
    s->rx.phase4_da_seeded = 0;
    s->rx.phase4_da_derot = 0;
    mp_reset_hypothesis_search(&s->rx);
    mp_vote_reset(&s->rx);

    span_log(&s->logging, SPAN_LOG_FLOW,
             "Rx - V.90 Phase 4: immediate CPt acquisition armed (tap=4, domain=diff, order=b0,b1)\n");
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_reject_v90_phase4_hypothesis(v34_state_t *s)
{
    const char *hold_env;

    if (!s || !s->rx.v90_mode || s->rx.calling_party
        || s->rx.stage != V34_RX_STAGE_V90_CP)
        return;
    hold_env = getenv("ME_V34_HOLD_MP_HYPOTHESIS");
    if (hold_env && atoi(hold_env) != 0 && s->rx.mp_hypothesis >= 0)
    {
        /* The V.90 CP framer is the owner of frame validity, so a failed CP
           CRC does not by itself prove that SpanDSP's symbol hypothesis is
           wrong. Preserve that hypothesis and let the next repeated CP
           preamble try again with the same coherent bit stream. */
        s->rx.mp_early_rejects = 0;
        s->rx.mp_count = 0;
        s->rx.mp_frame_pos = 0;
        s->rx.mp_frame_target = 0;
        span_log(&s->logging, SPAN_LOG_FLOW,
                 "Rx - V.90 Phase 4: strict CP reject; holding MP hypothesis=%d for repeated CP\n",
                 s->rx.mp_hypothesis);
        return;
    }
    /* A strict Table 14 CRC rejection invalidates more than the current phase
       seed.  Advance the existing domain/tap/bit-order retry state as well;
       merely clearing mp_hypothesis retries the same decode mode forever. */
    mp_unlock_after_reject(&s->rx, true);
    span_log(&s->logging, SPAN_LOG_FLOW,
             "Rx - V.90 Phase 4: strict CP framer rejected hypothesis; resuming preamble search\n");
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_get_rx_event(v34_state_t *s)
{
    return s->rx.received_event;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_v90_clear_peer_retrain_event(v34_state_t *s)
{
    if (s && s->rx.received_event == V34_EVENT_PEER_RETRAIN)
        s->rx.received_event = V34_EVENT_NONE;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_get_phase3_s_event_count(v34_state_t *s)
{
    return s ? s->rx.phase3_s_event_count : 0;
}

SPAN_DECLARE(void) v34_v90_set_phase3_expect_silence(v34_state_t *s, int expect)
{
    if (!s)
        return;
    if (s->rx.phase3_expect_silence == (bool) expect)
        return;
    /*endif*/
    s->rx.phase3_expect_silence = (bool) expect;
    s->rx.phase3_energy_samples = 0;
    s->rx.phase3_energy_retrain_reported = false;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_v90_arm_phase3_s_detector(v34_state_t *s)
{
    if (!s)
        return;

    /* Ja has already been delivered to the external V.90 digital-side state
       machine.  Clear the shared V.34 event before v34_tx() can interpret it
       as its own far-end J and call phase4_wait_init().  Preserve the decoded
       Ja constellation choice, but start the S/S-bar detector with a clean
       32-baud window so the later analogue S transition is unambiguous. */
    /* Pin the TRN constellation from the TRN lock rather than trusting
     * phase3_j_trn16 as left by the canonical J matcher.
     *
     * That matcher sets trn16 from whichever 16-bit pattern it last matched,
     * and it gets it wrong: measured live against a peer whose own log says
     * TRNSEG4 (4-point), the S detector was being armed "16-point", and the
     * J look-ahead then pinned 4-point a moment later -- the two disagreeing
     * about the same signal. Arming the S detector for the wrong constellation
     * is why the analogue S was never seen, which left Jd to expire on its
     * interop timeout and dragged the rest of Phase 3 onto timers.
     *
     * phase3_trn_lock_score is scored through a 4-point mapping, so a high
     * score is positive evidence the far end really is 4-point -- far better
     * evidence than the pattern matcher's guess. */
    /* Any latched TRN lock is 4-point evidence: the latch only ever accepts a
     * hypothesis scoring >=70% through a 4-point mapping, so there is no need
     * for a second, higher threshold here (an earlier 80% cut simply missed
     * legitimate 75% locks). */
    if (s->rx.phase3_trn_lock_hyp >= 0)
    {
        if (s->rx.phase3_j_trn16)
        {
            span_log(&s->logging, SPAN_LOG_FLOW,
                     "Rx - V.90: TRN lock (hyp=%d %d%%, 4-point mapping) overrides "
                     "canonical-J trn16=1; arming S detector as 4-point\n",
                     s->rx.phase3_trn_lock_hyp, s->rx.phase3_trn_lock_score);
        }
        /*endif*/
        s->rx.phase3_j_trn16 = 0;
    }
    /*endif*/
    s->rx.received_event = V34_EVENT_NONE;
    s->rx.stage = V34_RX_STAGE_PHASE3_WAIT_S;
    s->rx.duration = 0;
    s->rx.bit_count = 0;
    s->rx.s_detect_count = 0;
    s->rx.s_window = 0;
    s->rx.phase3_s_present = false;
    s->rx.phase3_s_alt_window = 0;
    s->rx.phase3_s_alt_count = 0;
    s->rx.phase3_s_stable_windows = 0;
    s->rx.phase3_s_dom_windows = 0;
    s->rx.phase3_s_dom_symbol = -1;
    s->rx.phase3_s_fired_symbol = -1;
    memset(s->rx.phase3_s_counts, 0, sizeof(s->rx.phase3_s_counts));
    memset(s->rx.phase3_s_ring, 0, sizeof(s->rx.phase3_s_ring));
    memset(s->rx.phase3_s_mag_ring, 0, sizeof(s->rx.phase3_s_mag_ring));
    /* Arm unconditionally: this call *is* the statement that Ja has been
       consumed and the analogue S is the next thing to expect.  It used to arm
       only as a side effect of phase3_j_trn16 >= 0, so a call that reached Ja
       by the energy-gap heuristic or by v90_note_ja_confirmed_by_descriptor()
       -- neither of which runs the canonical J matcher -- left the detector off
       whenever there was also no TRN lock.  The constellation hint below is
       informational; the S detector does not use it. */
    s->rx.phase3_s_detect_armed = true;

    span_log(&s->logging, SPAN_LOG_FLOW,
             "Rx - V.90: analogue Ja consumed; armed clean Phase 3 S detector (trn=%s)\n",
             (s->rx.phase3_j_trn16 < 0)
                ? "unknown"
                : (s->rx.phase3_j_trn16 ? "16-point" : "4-point"));
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_v90_copy_phase3_ja_bits(v34_state_t *s,
                                               int hypothesis,
                                               uint8_t bits[],
                                               int max_bits)
{
    int len;

    if (!s || !bits || max_bits <= 0
        || hypothesis < 0 || hypothesis >= MP_HYPOTHESIS_COUNT)
        return 0;
    len = s->rx.phase3_ja_capture_hyp_len[hypothesis];
    if (len > max_bits)
        len = max_bits;
    memcpy(bits, s->rx.phase3_ja_capture_hyp[hypothesis], (size_t) len);
    return len;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_v90_copy_phase3_ja_raw_bits(v34_state_t *s,
                                                   int hypothesis,
                                                   uint8_t bits[],
                                                   int max_bits)
{
    int len;

    if (!s || !bits || max_bits <= 0
        || hypothesis < 0 || hypothesis >= MP_HYPOTHESIS_COUNT)
        return 0;
    len = s->rx.phase3_ja_capture_hyp_raw_len[hypothesis];
    if (len > max_bits)
        len = max_bits;
    memcpy(bits, s->rx.phase3_ja_capture_hyp_raw[hypothesis], (size_t) len);
    return len;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_seed_rx_mp(v34_state_t *s,
                                 int bit_rate_n,
                                 int trellis_size,
                                 int use_non_linear_encoder,
                                 int expanded_shaping,
                                 const int16_t precoder_coeffs[6])
{
    int i;

    if (!s || bit_rate_n < 1 || bit_rate_n > 14
        || set_trellis_mode(s, trellis_size))
        return -1;
    s->rx.bit_rate = (bit_rate_n - 1)*2;
    v34_set_working_parameters(&s->rx.parms,
                               s->rx.baud_rate,
                               s->rx.bit_rate,
                               expanded_shaping != 0);
    s->rx.use_non_linear_encoder = (use_non_linear_encoder != 0);
    for (i = 0;  i < 3;  i++)
    {
        s->rx.h[i].re = precoder_coeffs ? precoder_coeffs[2*i] : 0;
        s->rx.h[i].im = precoder_coeffs ? precoder_coeffs[2*i + 1] : 0;
    }
    /*endfor*/
    /* The external detector has supplied the receive parameters which our MP
       requested of the far-end transmitter.  Mark MP established so the
       existing Phase 4 path can detect E and enter DATA without guessing a
       frame boundary. */
    s->rx.mp_seen = 1;
    s->rx.mp_accepted_baud = s->rx.duration;
    if (s->rx.v90_mode && !s->calling_party)
    {
        /* In V.90 the externally decoded CPt replaces the ordinary V.34
           J'/TRN/MP receive handoff.  Do not leave the native receiver parked
           in PHASE4_TRN after the project layer has supplied the accepted MP
           parameters; enter MP/E search while preserving the trained primary
           equalizer and carrier/timing state. */
        s->rx.stage = V34_RX_STAGE_V90_CP;
        s->rx.duration = 0;
        s->rx.mp_accepted_baud = 0;
        s->rx.bitstream = 0;
        s->rx.bit_count = 0;
        s->rx.mp_count = -1;
        s->rx.mp_frame_pos = 0;
        s->rx.mp_frame_target = 0;
        s->rx.mp_early_rejects = 0;
        s->rx.mp_hypothesis = -1;
        s->rx.mp_phase4_default_scrambler_tap = 4;
        s->rx.scrambler_tap = 4;
        mp_reset_hypothesis_search(&s->rx);
    }
    /*endif*/
    return 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_set_rx_data_transform(v34_state_t *s,
                                            float scale,
                                            int rotation,
                                            int conjugate)
{
    if (!s || !isfinite(scale) || scale <= 0.0f
        || rotation < 0 || rotation > 3)
        return -1;
    s->rx.data_symbol_scale = scale;
    s->rx.data_symbol_rotation = rotation;
    s->rx.data_symbol_conjugate = (conjugate != 0);
    return 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_v90_prepare_upstream_data(v34_state_t *s,
                                                int bit_rate,
                                                int trellis_size)
{
    int i;
    int bit_rate_n;
    int data_baud;
    const char *baud_env;

    if (!s || bit_rate < 2400 || bit_rate > 33600 || (bit_rate % 2400) != 0)
        return -1;
    bit_rate_n = bit_rate/2400;
    if (set_trellis_mode(s, trellis_size))
        return -1;
    /* V.90 §6.2 mandates the V.34 analogue modem support the 3200-baud
       symbol rate; the upstream V.34 data direction uses it.  The dedicated
       V90_CP stage acquires CPt/CP at the 2400-baud control-channel rate
       (see v90_cp_live.c V90_CP_LIVE_BAUD_CODE), so by the time CP' is
       accepted s->rx.baud_rate is still the CP value and v34_set_working_
       parameters() below would size the data mapping frame / viterbi /
       superframe for the wrong baud -- which is exactly the dead-upstream-
       RX signature (live 2026-07-26: DATA entered at baud_rate=0/1800 Hz,
       zero mapping frames, while the downstream TX was healthy).  V.90
       skips the ordinary V.34 MP exchange that would otherwise reconfigure
       the receiver for data, so do it here, at the V90_CP seam: restore the
       negotiated data baud and recompute the carrier/shaper/parm state
       without touching the trained equalizer, carrier-tracking or
       decision-aided derotator (those carry CPt's solution into DATA per
       the §9.4.2.2 channel-static assumption).  high_carrier is preserved
       from the Phase-3/INFO1a assignment. */
    data_baud = V34_BAUD_RATE_3200;
    baud_env = getenv("ME_V90_UPSTREAM_BAUD");
    if (baud_env && *baud_env)
    {
        int b = atoi(baud_env);
        if (b >= V34_BAUD_RATE_2400 && b <= V34_BAUD_RATE_3429)
            data_baud = b;
    }
    s->rx.baud_rate = data_baud;
    s->rx.v34_carrier_phase_rate =
        dds_phase_ratef(carrier_frequency(s->rx.baud_rate, s->rx.high_carrier));
    s->rx.shaper_re = v34_rx_shapers_re[s->rx.baud_rate][s->rx.high_carrier];
    s->rx.shaper_im = v34_rx_shapers_im[s->rx.baud_rate][s->rx.high_carrier];
    create_godard_coeffs(&s->rx.pri_ted,
                         carrier_frequency(s->rx.baud_rate, s->rx.high_carrier),
                         baud_rate_parameters[s->rx.baud_rate].baud_rate,
                         0.99f);
    s->rx.bit_rate = (bit_rate_n - 1)*2;
    v34_set_working_parameters(&s->rx.parms,
                               s->rx.baud_rate,
                               s->rx.bit_rate,
                               false);
    s->rx.use_non_linear_encoder = false;
    for (i = 0;  i < 3;  i++)
    {
        s->rx.h[i].re = 0;
        s->rx.h[i].im = 0;
    }
    /*endfor*/
    return 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v34_begin_rx_data(v34_state_t *s)
{
    int prior;

    if (!s)
        return -1;
    s->rx.step_2d = 0;
    s->rx.data_frame = 0;
    s->rx.mapping_frame_count = 0;
    s->rx.s_bit_cnt = 0;
    s->rx.aux_bit_cnt = 0;
    memset(s->rx.xt, 0, sizeof(s->rx.xt));
    memset(s->rx.x, 0, sizeof(s->rx.x));
    memset(s->rx.ww, 0, sizeof(s->rx.ww));
    s->rx.viterbi.ptr = 0;
    s->rx.viterbi.windup = 15;
    /* 10.1.3.1/V.34: B1 is a reset-state data frame that carries the
       superframe synchronization inversions of the FINAL data frame in a
       superframe, and the convolutional encoder is reset to state zero.
       Initializing to an ordinary frame zero leaves the V0 inversion
       pattern out of phase for the whole connection. */
    if (s->rx.parms.j > 0)
    {
        s->rx.super_frame = s->rx.parms.j - 1;
        s->rx.v0_pattern = (uint16_t)(2*(s->rx.parms.j - 1));
        s->rx.input_4d = (s->rx.parms.j - 1)*4*s->rx.parms.p;
    }
    else
    {
        s->rx.super_frame = 0;
        s->rx.v0_pattern = 0;
        s->rx.input_4d = 0;
    }
    /*endif*/
    prior = (s->rx.viterbi.ptr - 1) & 0xF;
    for (int state = 0;  state < s->rx.viterbi.state_count;  state++)
    {
        s->rx.viterbi.vit[prior].cumulative_path_metric[state] =
            (state == 0)  ?  0U  :  0x3FFFFFFFU;
    }
    /*endfor*/
    s->rx.received_event = V34_EVENT_E;
    s->rx.mp_seen = 2;
    s->rx.stage = V34_RX_STAGE_DATA;
    return 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_put_mapping_frame_state(v34_state_t *s,
                                               int16_t bits[16])
{
    if (s && bits)
        v34_put_mapping_frame(&s->rx, bits);
    /*endif*/
}
/*- End of function --------------------------------------------------------*/

int v34_rx_restart(v34_state_t *s, int baud_rate, int bit_rate, int high_carrier)
{
    int i;

    s->rx.baud_rate = baud_rate;
    s->rx.bit_rate = bit_rate;
    s->rx.high_carrier = high_carrier;
    s->rx.training_failed_reported = false;
    /* Phase 3/4 peer-retrain detectors: a restart returns to Phase 2, where
       primary_channel_rx() -- and therefore these detectors' out-of-stage
       reset branches -- never runs, so one-shot flags latched during the
       previous attempt would otherwise mute the detectors for the whole next
       attempt.  Observed live 2026-07-22: the peer's second retrain (1.9 s of
       Tone A in armed stages) went unreported because phase34_tone_a_reported
       survived the first retrain's restart. */
    s->rx.phase34_silence_samples = 0;
    s->rx.phase34_retrain_reported = false;
    s->rx.phase34_tone_a_g1 = 0.0f;
    s->rx.phase34_tone_a_g2 = 0.0f;
    s->rx.phase34_tone_a_energy = 0.0f;
    s->rx.phase34_tone_a_samples = 0;
    s->rx.phase34_tone_a_blocks = 0;
    s->rx.phase34_tone_a_reported = false;
    s->rx.phase3_energy_samples = 0;
    s->rx.phase3_energy_retrain_reported = false;

    s->rx.v34_carrier_phase_rate = dds_phase_ratef(carrier_frequency(s->rx.baud_rate, s->rx.high_carrier));
    /* Phase 2 INFO exchange: answerer RX at 1200 Hz (tone B), caller RX at 2400 Hz (tone A).
       V.90 §8.2.3.1: in V.90 mode carriers are swapped — digital modem (answerer)
       RX listens at 2400 Hz (analog modem's CC carrier).
       This gets updated to Phase 4 CC frequencies in mp_or_mph_baud_init(). */
    /* V.90 §8.2.3.1: digital modem (answerer) listens at 2400 Hz (analog modem's CC),
       analog modem (caller) listens at 1200 Hz (digital modem's CC).
       Standard V.34: caller RX at 2400 Hz, answerer RX at 1200 Hz. */
    if (s->rx.v90_mode && !s->calling_party)
        s->rx.cc_carrier_phase_rate = dds_phase_ratef(2400.0f);
    else if (s->rx.v90_mode && s->calling_party)
        s->rx.cc_carrier_phase_rate = dds_phase_ratef(1200.0f);
    else
        s->rx.cc_carrier_phase_rate = dds_phase_ratef((s->calling_party)  ?  2400.0f  :  1200.0f);
    v34_set_working_parameters(&s->rx.parms, s->rx.baud_rate, s->rx.bit_rate, true);

    s->rx.high_sample = 0;
    s->rx.low_samples = 0;
    s->rx.carrier_drop_pending = false;

    power_meter_init(&s->rx.power, 4);

    s->rx.carrier_phase = 0;
    s->rx.agc_scaling_save = 0.0f;
    s->rx.agc_scaling = 0.0017f/V34_RX_PULSESHAPER_GAIN;
    equalizer_reset(&s->rx);
    s->rx.carrier_track_i = 5000.0f;
    s->rx.carrier_track_p = 40000.0f;

    /* Create a default symbol sync filter */
    create_godard_coeffs(&s->rx.pri_ted,
                         carrier_frequency(s->rx.baud_rate, s->rx.high_carrier),
                         baud_rate_parameters[s->rx.baud_rate].baud_rate,
                         0.99f);
    create_godard_coeffs(&s->rx.cc_ted,
                         (s->rx.v90_mode && !s->calling_party)  ?  2400.0f  :  ((s->calling_party)  ?  2400.0f  :  1200.0f),
                         600,
                         0.99f);
    /* Initialise the working data for symbol timing synchronisation */
#if defined(SPANDSP_USE_FIXED_POINT)
    for (i = 0;  i < 2;  i++)
    {
        s->rx.pri_ted.symbol_sync_low[i] = 0;
        s->rx.pri_ted.symbol_sync_high[i] = 0;
        s->rx.pri_ted.symbol_sync_dc_filter[i] = 0;
    }
    /*endfor*/
    s->rx.pri_ted.baud_phase = 0;
    for (i = 0;  i < 2;  i++)
    {
        s->rx.cc_ted.symbol_sync_low[i] = 0;
        s->rx.cc_ted.symbol_sync_high[i] = 0;
        s->rx.cc_ted.symbol_sync_dc_filter[i] = 0;
    }
    /*endfor*/
    s->rx.cc_ted.baud_phase = 0;
#else
    for (i = 0;  i < 2;  i++)
    {
        s->rx.pri_ted.symbol_sync_low[i] = 0.0f;
        s->rx.pri_ted.symbol_sync_high[i] = 0.0f;
        s->rx.pri_ted.symbol_sync_dc_filter[i] = 0.0f;
    }
    /*endfor*/
    s->rx.pri_ted.baud_phase = 0.0f;
    for (i = 0;  i < 2;  i++)
    {
        s->rx.cc_ted.symbol_sync_low[i] = 0.0f;
        s->rx.cc_ted.symbol_sync_high[i] = 0.0f;
        s->rx.cc_ted.symbol_sync_dc_filter[i] = 0.0f;
    }
    /*endfor*/
    s->rx.cc_ted.baud_phase = 0.0f;
#endif
    s->rx.baud_half = 0;

    s->rx.bitstream = 0;
    s->rx.bit_count = 0;
    s->rx.duration = 0;
    s->rx.blip_duration = 0;
    s->rx.last_angles[0] = 0;
    s->rx.last_angles[1] = 0;
    s->rx.total_baud_timing_correction = 0;
    s->rx.phase3_s_guard_samples = 4000;
    s->rx.phase3_s_hits = 0;
    s->rx.phase3_s_event_count = 0;
    s->rx.phase3_s_present = false;
    s->rx.phase3_s_alt_window = 0;
    s->rx.phase3_s_alt_count = 0;
    s->rx.phase3_s_stable_windows = 0;
    s->rx.phase3_s_detect_armed = false;
    memset(s->rx.phase3_s_ring, 0, sizeof(s->rx.phase3_s_ring));
    memset(s->rx.phase3_s_mag_ring, 0, sizeof(s->rx.phase3_s_mag_ring));
    memset(s->rx.phase3_s_counts, 0, sizeof(s->rx.phase3_s_counts));
    s->rx.phase3_s_pos = 0;
    phase3_pp_reset(&s->rx);
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
    memset(s->rx.phase3_ja_scramble, 0, sizeof(s->rx.phase3_ja_scramble));
    memset(s->rx.phase3_ja_prev_z, 0, sizeof(s->rx.phase3_ja_prev_z));
    memset(s->rx.phase3_ja_prev_valid, 0, sizeof(s->rx.phase3_ja_prev_valid));
    s->rx.phase3_ja_bits = 0;
    s->rx.phase3_ja_hyp = -1;
    memset(s->rx.phase3_ja_capture, 0, sizeof(s->rx.phase3_ja_capture));
    s->rx.phase3_ja_capture_len = 0;
    memset(s->rx.phase3_ja_capture_hyp, 0, sizeof(s->rx.phase3_ja_capture_hyp));
    memset(s->rx.phase3_ja_capture_hyp_len, 0, sizeof(s->rx.phase3_ja_capture_hyp_len));
    memset(s->rx.phase3_ja_capture_hyp_raw, 0, sizeof(s->rx.phase3_ja_capture_hyp_raw));
    memset(s->rx.phase3_ja_capture_hyp_raw_len, 0, sizeof(s->rx.phase3_ja_capture_hyp_raw_len));
    phase3_trn_hyp_reset(&s->rx);
    s->rx.phase2_reversal_count = 0;
    s->rx.phase2_l2_count = 0;
    s->rx.phase3_trn_mag_sum = 0.0f;
    s->rx.phase3_trn_mag_count = 0;
    s->rx.phase4_j_seen = 0;
    s->rx.phase4_j_lock_hyp = -1;
    s->rx.phase4_trn_after_j = 0;
    phase4_j_detector_reset(&s->rx);
    phase4_trn_hyp_reset(&s->rx);

    s->rx.info0_received = false;
    s->rx.info1a_received = false;
    s->rx.info1c_received = false;
    s->rx.info1a_raw_12_17 = 0;
    s->rx.info1a_raw_32_33 = 0;
    s->rx.info1a_raw_40_49 = 0;
    s->rx.info0_raw_26_27 = 0;
    s->rx.info0d_nominal_power_code = 0;
    s->rx.info0d_max_power_code = 0;
    s->rx.info0d_power_measured_at_codec_output = false;
    s->rx.info0d_pcm_alaw = false;
    s->rx.info0d_upstream_3429_support = false;
    s->rx.info0d_reserved_41 = 0;
    s->rx.info0d_extensions_valid = false;
    s->rx.v90_repeated_info0a_pending = false;
    s->rx.v90_info1d_sent = false;
    s->rx.stage = V34_RX_STAGE_INFO0;
    /* The next info message will be INFO0 or INFOH, depending whether we are in half or full duplex mode. */
    s->rx.target_bits = (s->rx.duplex)  ?  (49 - (4 + 8 + 4))  :  (51 - (4 + 8 + 4));

    s->rx.mp_count = -1;
    s->rx.mp_len = 0;
    s->rx.mp_seen = -1;
    s->rx.mp_remote_ack_seen = 0;
    s->rx.mp_frame_pos = 0;
    s->rx.mp_frame_target = 0;
    s->rx.mp_early_rejects = 0;
    s->rx.mp_phase4_default_scrambler_tap = s->rx.scrambler_tap;
    s->rx.mp_phase4_default_bit_order = 0;
    s->rx.mp_phase4_default_domain = 0;
    s->rx.mp_phase4_reject_streak = 0;
    s->rx.mp_phase4_nolock_count = 0;
    s->rx.mp_phase4_alt_tap_active = 0;
    s->rx.mp_phase4_alt_order_active = 0;
    s->rx.mp_phase4_alt_domain_active = 0;
    s->rx.last_rx_mp_valid = false;
    memset(&s->rx.last_rx_mp, 0, sizeof(s->rx.last_rx_mp));
    s->rx.mp_phase4_retry_mode = 0;
    s->rx.mp_phase4_bit_order = 0;
    s->rx.mp_phase4_domain = 0;
    s->rx.mp_phase4_force_abs_active = 0;
    s->rx.mp_phase4_diff_collapse_streak = 0;
    s->rx.mp_phase4_diff_recover_streak = 0;
    s->rx.phase4_da_active = 0;
    s->rx.phase4_da_seeded = 0;
    s->rx.phase4_da_derot = 0;
    mp_reset_hypothesis_search(&s->rx);
    mp_vote_reset(&s->rx);
    s->rx.last_logged_mp_diag_state = V34_MP_DIAG_STATE_NONE;
    s->rx.last_logged_stage = -1;
    s->rx.last_logged_event = -1;
    s->rx.last_logged_demodulator = -1;

    s->rx.viterbi.ptr = 0;
    s->rx.viterbi.windup = 15;

    s->rx.eq_put_step = RX_PULSESHAPER_2400_COEFF_SETS*40/(3*2) - 1;
    s->rx.eq_step = 0;
    s->rx.scramble_reg = 0;

    s->rx.current_demodulator = V34_MODULATION_TONES;
    (void) set_trellis_mode(s, V34_TRELLIS_16);

    s->rx.v0_pattern = 0;
    s->rx.super_frame = 0;
    s->rx.data_frame = 0;
    s->rx.s_bit_cnt = 0;
    s->rx.aux_bit_cnt = 0;
    s->rx.mapping_frame_count = 0;
    s->rx.data_symbol_scale = 70.0f;
    s->rx.data_symbol_rotation = 0;
    s->rx.data_symbol_conjugate = false;

    return 0;
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(void) v34_set_qam_report_handler(v34_state_t *s, qam_report_handler_t handler, void *user_data)
{
    s->rx.qam_report = handler;
    s->rx.qam_user_data = user_data;
}
/*- End of function --------------------------------------------------------*/
/*- End of file ------------------------------------------------------------*/
