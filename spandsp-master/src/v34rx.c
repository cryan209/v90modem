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
#define TRAINING_AMP                    10.0f
#define EQUALIZER_DELTA                 0.21f
#define EQUALIZER_SLOW_ADAPT_RATIO      0.1f

#define V34_TRAINING_SEG_1              0
#define V34_TRAINING_SEG_4              0
#define V34_TRAINING_END                0
#define V34_TRAINING_SHUTDOWN_END       0
#define MP_LOCK_SCORE_MIN               18
#define MP_PREAMBLE_SCORE_MIN           18
#define MP_HINT_LOCK_SCORE_MIN          18
#define MP_PREAMBLE_WAIT_BITS           800
#define MP_PRELOCK_PREAMBLE_WAIT_BITS   160
#define MP_TRN_PRELOCK_SCORE_MIN        70
#define PHASE4_TRN_SCORE_START_BAUD     145
#define PHASE4_TRN_LOCK_MIN_BITS        64
#define PHASE4_TRN_READY_MIN_SCORE      65
#define PHASE4_TRN_READY_MIN_BAUD       160
#define PHASE4_TRN_READY_MAX_BAUD       9600    /* ~3s at 3200 baud; V.34 TRN max is 2s */
#define PHASE4_TRN_RECENT_WINDOW_BAUDS  256
#define PHASE4_TRN_FREEZE_SCORE         80
#define MP_HYPOTHESIS_COUNT             24
#define MP_EARLY_START_ERR_MAX          2
#define MP_EARLY_START_ERR_FRAME_LIMIT  85
#define MP1_START_ERR_ACCEPT_MAX        3
#define MP_BOUNDARY_BRUTEFORCE_MAX_CHANGES 4
#define MP_HINT_STRICT_REJECTS          2
#define PHASE3_PP_TRAIN_BAUDS           PP_TOTAL_SYMBOLS
#define PHASE3_TRN_REFINE_BAUDS         512
#define PHASE3_PP_ACQUIRE_MIN_BAUDS     160
#define PHASE3_PP_ACQUIRE_HOLD_BAUDS    24
#define PHASE3_PP_ACQUIRE_SCORE_MIN     28
#define PHASE3_PP_ACQUIRE_DECAY         0.98f

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
    default: return "UNKNOWN";
    }
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

static int descramble(v34_rx_state_t *s, int in_bit)
{
    int out_bit;

    /* One of the scrambler taps is a variable, so it can be adjusted for caller or answerer operation. */
    out_bit = (in_bit ^ (s->scramble_reg >> s->scrambler_tap) ^ (s->scramble_reg >> (23 - 1))) & 1;
    s->scramble_reg = (s->scramble_reg << 1) | in_bit;
    return out_bit;
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
        for (i = 19;  i < target;  i++)
        {
            int src;

            src = i + slip;
            trial[i] = (src >= 19 && src < target)  ?  bits[src]  :  0;
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

            for (i = 19;  i < target;  i++)
            {
                int src;

                src = i + base_slip;
                base_trial[i] = (src >= 19  &&  src < target)  ?  bits[src]  :  0;
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

static void log_mp_tx_rx_compare(v34_rx_state_t *s, const uint8_t rx_bits[], int type)
{
    v34_state_t *t;
    bitstream_state_t bs;
    const uint8_t *p;
    int target;
    int first_diff;
    int i;
    int max_diffs;
    int diff_count;

    t = span_container_of(s, v34_state_t, rx);
    if (t->tx.mp.type != type)
        return;
    /*endif*/
    target = (type == 1) ? 188 : 88;
    bitstream_init(&bs, true);
    p = t->tx.txbuf;
    first_diff = -1;
    max_diffs = 8;
    diff_count = 0;

    for (i = 0;  i < target;  i++)
    {
        int tx_bit;

        tx_bit = bitstream_get(&bs, &p, 1);
        if (tx_bit != (rx_bits[i] & 1))
        {
            if (first_diff < 0)
                first_diff = i;
            /*endif*/
            if (diff_count < max_diffs)
            {
                span_log(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 4: MP%d TX/RX diff[%d] frame_idx=%d tx=%d rx=%d (dom=%s tap=%d ord=%s hyp=%d)\n",
                         type, diff_count, i, tx_bit, rx_bits[i] & 1,
                         phase4_trn_domain_name(s->mp_phase4_domain),
                         s->scrambler_tap,
                         phase4_trn_order_name(s->mp_phase4_bit_order),
                         s->mp_hypothesis);
            }
            /*endif*/
            diff_count++;
        }
        /*endif*/
    }
    /*endfor*/

    if (first_diff >= 0)
    {
        span_log(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4: MP%d first TX/RX divergence at frame_idx=%d (dom=%s tap=%d ord=%s hyp=%d total_diffs=%d)\n",
                 type, first_diff,
                 phase4_trn_domain_name(s->mp_phase4_domain),
                 s->scrambler_tap,
                 phase4_trn_order_name(s->mp_phase4_bit_order),
                 s->mp_hypothesis,
                 diff_count);
    }
    else
    {
        span_log(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4: MP%d TX/RX frame bits match exactly (%d bits)\n",
                 type, target);
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
                 "Rx - Phase 4: MP%d semantic reject (reserved rate-mask bit set: 0x%04X)\n",
                 type, mp->signalling_rate_mask & 0x7FFF);
        return false;
    }
    /*endif*/
    bit_idx = mp->bit_rate_a_to_c - 1;
    if (!(mp->signalling_rate_mask & (1 << bit_idx)))
    {
        span_log(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4: MP%d semantic reject (a_to_c rate %d missing from mask 0x%04X)\n",
                 type, mp->bit_rate_a_to_c, mp->signalling_rate_mask & 0x7FFF);
        return false;
    }
    /*endif*/
    bit_idx = mp->bit_rate_c_to_a - 1;
    if (!(mp->signalling_rate_mask & (1 << bit_idx)))
    {
        span_log(s->logging, SPAN_LOG_FLOW,
                 "Rx - Phase 4: MP%d semantic reject (c_to_a rate %d missing from mask 0x%04X)\n",
                 type, mp->bit_rate_c_to_a, mp->signalling_rate_mask & 0x7FFF);
        return false;
    }
    /*endif*/
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

static void viterbi_update_path_metrics(viterbi_t *s)
{
    int16_t i;
    int16_t j;
    int16_t prev_state;
    int16_t branch;
    uint32_t curr_min_metric;
    uint32_t min_metric;
    uint32_t metric;
    uint16_t min_state;
    uint16_t min_branch;
    int prev_ptr;

    curr_min_metric = UINT32_MAX;
    /* Loop through each state */
    prev_ptr = (s->ptr - 1) & 0xF;
    for (i = 0;  i < 16;  i++)
    {
        min_metric = UINT32_MAX;
        min_state = 0;
        min_branch = 0;
        /* Loop through each possible branch from the previous state */
        for (j = 0;  j < 4;  j++)
        {
            prev_state = (*s->conv_decode_table)[i][j] >> 3;
            branch = (*s->conv_decode_table)[i][j] & 0x7;
            metric = s->vit[prev_ptr].cumulative_path_metric[prev_state] + s->branch_error[branch];

//if (metric == 0)
//    printf("HHH %p metric is zero - %2d %2d %2d %2d %2d\n", s, prev_ptr, i, j, prev_state, branch);
///*endif*/
//if (s->branch_error[branch] == 0)
//    printf("HHX %p metric is zero - %2d %2d %2d %2d %2d\n", s, prev_ptr, i, j, prev_state, branch);
///*endif*/
            if (metric < min_metric)
            {
                min_metric = metric;
                min_state = prev_state;
                min_branch = branch;
            }
            /*endif*/
        }
        /*endfor*/
        s->vit[s->ptr].cumulative_path_metric[i] = min_metric;
        s->vit[s->ptr].previous_path_ptr[i] = min_state;
        s->vit[s->ptr].pts[i] = min_branch;
        if (min_metric < curr_min_metric)
        {
            curr_min_metric = min_metric;
            s->curr_min_state = i;
        }
        /*endif*/
    }
    /*endfor*/
//printf("GGG %p min metric %d, state %d\n", s, curr_min_metric, s->curr_min_state);
//printf("JJJ %p ", s);
    for (i = 0;  i < 16;  i++)
    {
        s->vit[s->ptr].cumulative_path_metric[i] -= curr_min_metric;
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

    memset(&s->far_capabilities, 0, sizeof(s->far_capabilities));
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
    s->far_capabilities.tx_clock_source = bitstream_get(&bs, &t, 2);
    s->info0_acknowledgement = bitstream_get(&bs, &t, 1);

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
    return 0;
}
/*- End of function --------------------------------------------------------*/

static int process_rx_info1a(v34_rx_state_t *s, info1a_t *info1a, uint8_t buf[])
{
    bitstream_state_t bs;
    const uint8_t *t;

    bitstream_init(&bs, true);
    t = buf;
    /* 12:14    Minimum power reduction to be implemented by the call modem transmitter. An integer between 0 and 7 gives
                the recommended power reduction in dB. These bits shall indicate 0 if INFO0c indicated that the call modem
                transmitter cannot reduce its power. */
    info1a->power_reduction = bitstream_get(&bs, &t, 3);
    /* 15:17    Additional power reduction, below that indicated by bits 12:14, which can be tolerated by the answer modem
                receiver. An integer between 0 and 7 gives the additional power reduction in dB. These bits shall indicate 0 if
                INFO0c indicated that the call modem transmitter cannot reduce its power. */
    info1a->additional_power_reduction = bitstream_get(&bs, &t, 3);
    /* 18:24    Length of MD to be transmitted by the answer modem during Phase 3. An integer between 0 and 127 gives the
                length of this sequence in 35 ms increments. */
    info1a->md = bitstream_get(&bs, &t, 7);
    /* 25       Set to 1 indicates that the high carrier frequency is to be used in transmitting from the call modem to the answer
                modem. This shall be consistent with the capabilities of the call modem indicated in INFO0c. */
    info1a->use_high_carrier = bitstream_get(&bs, &t, 1);
    /* 26:29    Pre-emphasis filter to be used in transmitting from the call modem to the answer modem. These bits form an
                integer between 0 and 10 which represents the pre-emphasis filter index (see Tables 3 and 4). */
    info1a->preemphasis_filter = bitstream_get(&bs, &t, 4);
    /* 30:33    Projected maximum data rate for the selected symbol rate from the call modem to the answer modem. These bits
                form an integer between 0 and 14 which gives the projected data rate as a multiple of 2400 bits/s. */
    info1a->max_data_rate = bitstream_get(&bs, &t, 4);
    /* 34:36    Symbol rate to be used in transmitting from the answer modem to the call modem. An integer between 0 and 5
                gives the symbol rate, where 0 represents 2400 and a 5 represents 3429. The symbol rate selected shall be
                consistent with information in INFO1c and consistent with the symbol rate asymmetry allowed as indicated in
                INFO0a and INFO0c. The carrier frequency and pre-emphasis filter to be used are those already indicated for
                this symbol rate in info1c. */
    info1a->baud_rate_a_to_c = bitstream_get(&bs, &t, 3);
    /* 37:39    Symbol rate to be used in transmitting from the call modem to the answer modem. An integer between 0 and 5
                gives the symbol rate, where 0 represents 2400 and a 5 represents 3429. The symbol rate selected shall be
                consistent with the capabilities indicated in INFO0a and consistent with the symbol rate asymmetry allowed as
                indicated in INFO0a and INFO0c. */
    info1a->baud_rate_c_to_a = bitstream_get(&bs, &t, 3);
    /* 40:49    Frequency offset of the probing tones as measured by the answer modem receiver. The frequency offset number
                shall be the difference between the nominal 1050 Hz line probing signal tone received and the 1050 Hz tone
                transmitted, f(received) and f(transmitted). A two's complement signed integer between -511 and 511 gives the
                measured offset in 0.02 Hz increments. Bit 49 is the sign bit of this integer. The frequency offset measurement
                shall be accurate to 0.25 Hz. Under conditions where this accuracy cannot be achieved, the integer shall be set
                to -512 indicating that this field is to be ignored. */
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

    log_info1a(s->logging, false, info1a);
    return 0;
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

static void put_info_bit(v34_rx_state_t *s, int bit, int time_offset)
{
    /* Put info0, info1, tone A or tone B bits */
    s->bitstream = (s->bitstream << 1) | bit;
    switch (s->stage)
    {
    case V34_RX_STAGE_TONE_A:
        /* Calling side */
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
            case V34_EVENT_REVERSAL_1:
                span_log(s->logging, SPAN_LOG_FLOW, "Rx - reversal 2 in tone A\n");
                s->tone_ab_hop_time = s->sample_time + time_offset;
                s->received_event = V34_EVENT_REVERSAL_2;
                l1_l2_analysis_init(s);
                break;
            case V34_EVENT_REVERSAL_2:
            case V34_EVENT_L2_SEEN:
                span_log(s->logging, SPAN_LOG_FLOW, "Rx - reversal 3 in tone A\n");
                s->tone_ab_hop_time = s->sample_time + time_offset;
                s->received_event = V34_EVENT_REVERSAL_3;
                /* The next info message will be INFO1a */
                s->target_bits = 70 - (4 + 8 + 4);
                s->stage = V34_RX_STAGE_INFO1A;
                break;
            default:
                span_log(s->logging, SPAN_LOG_FLOW, "Rx - reversal 1 in tone A\n");
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
                /* The next info message will be INFO1c */
                s->target_bits = 109 - (4 + 8 + 4);
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
    /* Search for INFO0, INFOh, INFO1a or INFO1c messages. */
    if (s->bit_count == 0)
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
        if (s->bit_count++ == s->target_bits)
        {
            span_log(s->logging, SPAN_LOG_FLOW, "Rx - info CRC result 0x%x\n", s->crc);
            if (s->crc == 0)
            {
                switch (s->stage)
                {
                case V34_RX_STAGE_TONE_A:
                case V34_RX_STAGE_TONE_B:
                case V34_RX_STAGE_INFO0:
                    process_rx_info0(s, s->info_buf);
                    s->stage = (s->calling_party)  ?   V34_RX_STAGE_TONE_A  :  V34_RX_STAGE_TONE_B;
                    s->received_event = V34_EVENT_INFO0_OK;
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
                    s->received_event = V34_EVENT_INFO1_OK;
                    break;
                }
                /*endswitch*/
            }
            else
            {
                switch (s->stage)
                {
                case V34_RX_STAGE_TONE_A:
                case V34_RX_STAGE_TONE_B:
                case V34_RX_STAGE_INFO0:
                    s->received_event = V34_EVENT_INFO0_BAD;
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
            }
            /*endif*/
        }
        /*endif*/
        s->rrc_filter[s->rrc_filter_step] = amp[i];
        if (++s->rrc_filter_step >= V34_RX_FILTER_STEPS)
            s->rrc_filter_step = 0;
        /*endif*/
        if (s->calling_party)
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
        if ((phase_delta > (int32_t) DDS_PHASE(90.0f)
             || phase_delta < -(int32_t) DDS_PHASE(90.0f))
            &&
            s->blip_duration > 3)
        {
            /* Log reversal events during first 3 seconds */
            if (s->sample_time < 24000)
            {
                span_log(s->logging, SPAN_LOG_FLOW,
                         "Rx info_rx REV: t=%u blip=%d angle=0x%08X last=0x%08X diff=0x%08X ii=%.1f qq=%.1f\n",
                         (unsigned) (s->sample_time + i), s->blip_duration, angle, s->last_angles[1],
                         (uint32_t)(angle - s->last_angles[1]), ii, qq);
            }
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
    /* Periodic diagnostic: log every 2000 samples (~250ms) */
    if (s->duration % 2000 < (unsigned)len)
    {
        span_log(s->logging, SPAN_LOG_FLOW,
                 "Rx info_rx diag: t=%u dur=%d bits=%d stream=0x%08X "
                 "stage=%d sig=%d blip=%d pwr=%d angle=0x%08X ii=%.2f qq=%.2f\n",
                 (unsigned) s->sample_time, s->duration, s->bit_count, s->bitstream,
                 s->stage, s->signal_present, s->blip_duration, power,
                 angle, ii, qq);
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
    s->cc_ted.symbol_sync_dc_filter[1] = s->cc_ted.symbol_sync_dc_filter[0];
    s->cc_ted.symbol_sync_dc_filter[0] = v;
    /* A little integration will now filter away much of the HF noise */
    s->cc_ted.baud_phase -= p;
    v = fabsf(s->cc_ted.baud_phase);
    if (v > 100.0f)
    {
        i = (v > 200.0f)  ?  2  :  1;
        if (s->cc_ted.baud_phase < 0.0f)
            i = -i;
        /*endif*/
        //printf("v = %10.5f %5d - %f %f %d\n", v, i, p, s->cc_ted.baud_phase, s->total_baud_timing_correction);
        s->eq_put_step += i;
        s->total_baud_timing_correction += i;
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
    if (v > 100*FP_FACTOR)
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
    s->pri_ted.symbol_sync_dc_filter[1] = s->pri_ted.symbol_sync_dc_filter[0];
    s->pri_ted.symbol_sync_dc_filter[0] = v;
    /* A little integration will now filter away much of the HF noise */
    s->pri_ted.baud_phase -= p;
    v = fabsf(s->pri_ted.baud_phase);
    if (v > 100.0f)
    {
        i = (v > 200.0f)  ?  2  :  1;
        if (s->pri_ted.baud_phase < 0.0f)
            i = -i;
        /*endif*/
        //printf("v = %10.5f %5d - %f %f %d\n", v, i, p, s->pri_ted.baud_phase, s->total_baud_timing_correction);
        s->eq_put_step += i;
        s->total_baud_timing_correction += i;
    }
    /*endif*/
#endif
    /* Periodic TED + carrier tracking diagnostic (every 256 bauds) */
    if ((s->duration & 0xFF) == 0  &&  s->stage >= V34_RX_STAGE_PHASE3_WAIT_S)
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

    /* CMA (Constant Modulus Algorithm) — blind equalizer for constant-envelope
       signals like DQPSK.  Minimizes E[(R² - |y|²)²] without needing to know
       which symbol was transmitted.  Immune to the decision-directed convergence
       failure that occurs at 25% BER. */
    y_mag2 = z->re*z->re + z->im*z->im;
    R2 = 1.0f;  /* Fixed unit radius for QPSK — R²=1.0 */
    error = R2 - y_mag2;

    /* Log CMA error periodically */
    if ((s->duration & 0xFF) == 0)
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
        gz.re = 0.1f * s->eq_delta * error * z->re / norm;
        gz.im = 0.1f * s->eq_delta * error * z->im / norm;
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

static int phase3_equalizer_refine_active(const v34_rx_state_t *s)
{
    return (s->stage == V34_RX_STAGE_PHASE3_TRAINING
            && s->duration > PHASE3_PP_TRAIN_BAUDS
            && s->duration <= (PHASE3_PP_TRAIN_BAUDS + PHASE3_TRN_REFINE_BAUDS));
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
                s->received_event = V34_EVENT_L2_SEEN;
                s->current_demodulator = V34_MODULATION_TONES;
                s->stage = (s->calling_party)  ?  V34_RX_STAGE_TONE_A  :  V34_RX_STAGE_INFO1C;
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
                            t = span_container_of(s, v34_state_t, rx);
                            if (mp.type == 1)
                            {
                                /* Set the precoder coefficients we are to use */
                                memcpy(&t->tx.precoder_coeffs, mp.precoder_coeffs, sizeof(t->tx.precoder_coeffs));
                            }
                            /*endif*/
                            switch (mp.trellis_size)
                            {
                            case V34_TRELLIS_16:
                                t->tx.conv_encode_table = v34_conv16_encode_table;
                                break;
                            case V34_TRELLIS_32:
                                t->tx.conv_encode_table = v34_conv32_encode_table;
                                break;
                            case V34_TRELLIS_64:
                                t->tx.conv_encode_table = v34_conv64_encode_table;
                                break;
                            default:
                                span_log(&t->logging, SPAN_LOG_FLOW, "Rx - Unexpected trellis size code %d\n", mp.trellis_size);
                                break;
                            }
                            /*endswitch*/
                        }
                        else
                        {
                            process_rx_mph(s, &mph, s->info_buf);
                            t = span_container_of(s, v34_state_t, rx);
                            if (mph.type == 1)
                            {
                                /* Set the precoder coefficients we are to use */
                                memcpy(&t->tx.precoder_coeffs, mph.precoder_coeffs, sizeof(t->tx.precoder_coeffs));
                            }
                            /*endif*/
                            switch (mph.trellis_size)
                            {
                            case V34_TRELLIS_16:
                                t->tx.conv_encode_table = v34_conv16_encode_table;
                                break;
                            case V34_TRELLIS_32:
                                t->tx.conv_encode_table = v34_conv32_encode_table;
                                break;
                            case V34_TRELLIS_64:
                                t->tx.conv_encode_table = v34_conv64_encode_table;
                                break;
                            default:
                                span_log(&t->logging, SPAN_LOG_FLOW, "Rx - Unexpected trellis size code %d\n", mph.trellis_size);
                                break;
                            }
                            /*endswitch*/
                        }
                        /*endif*/
                        s->mp_seen = 1;
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
        /* CC carrier assignments (same for Phase 2 and Phase 4):
           Caller TX at 1200 Hz, Answerer TX at 2400 Hz.
           So: Caller RX = 2400 Hz, Answerer RX = 1200 Hz. */
        if (s->calling_party)
        {
            /* We are caller: receive answerer's CC at 2400 Hz */
#if defined(SPANDSP_USE_FIXED_POINT)
            ii = vec_circular_dot_prodi16(s->rrc_filter, rx_pulseshaper_2400_re[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
#else
            ii = vec_circular_dot_prodf(s->rrc_filter, rx_pulseshaper_2400_re[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
#endif
        }
        else
        {
            /* We are answerer: receive caller's CC at 1200 Hz */
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
            /* Only AGC until we have locked down the setting. */
            //if (s->agc_scaling_save == 0.0f)
#if defined(SPANDSP_USE_FIXED_POINT)
            //s->agc_scaling = saturate16(((int32_t) (1024.0f*FP_SCALE(2.17f)))/fixed_sqrt32(power));
#else
            //s->agc_scaling = (FP_SCALE(2.17f)/RX_PULSESHAPER_GAIN)/fixed_sqrt32(power);
#endif
            s->eq_put_step += RX_PULSESHAPER_2400_COEFF_SETS*40/(3*2);
            /* Same shaper as the real part above:
               Caller RX at 2400 Hz (answerer TX), Answerer RX at 1200 Hz (caller TX). */
            if (s->calling_party)
            {
                /* We are caller: receive answerer's CC at 2400 Hz */
#if defined(SPANDSP_USE_FIXED_POINT)
                qq = vec_circular_dot_prodi16(s->rrc_filter, rx_pulseshaper_2400_im[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
#else
                qq = vec_circular_dot_prodf(s->rrc_filter, rx_pulseshaper_2400_im[step], V34_RX_FILTER_STEPS, s->rrc_filter_step);
#endif
            }
            else
            {
                /* We are answerer: receive caller's CC at 1200 Hz */
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
            int old_rev;
            int new_rev;

        ang1 = arctan2(sym->im, sym->re);
        ang2 = arctan2(s->last_sample.im, s->last_sample.re);
        ang3 = ang1 - ang2 + DDS_PHASE(45.0f);
        data_bits = (ang3 >> 30) & 0x3;
        s->duration++;

            /* Explicit Phase 3 J/J' detector (answerer side):
               - apply all dibit mapping hypotheses
               - undo differential encoding (Z_n -> I_n)
               - descramble
               - correlate against J/J' 16-bit templates */
            if (!s->calling_party)
            {
                int h;
                int best_score;
                int best_h;
                int best_p;

                best_score = 0;
                best_h = -1;
                best_p = 0;
                for (h = 0;  h < 8;  h++)
                {
                    int raw_sym;

                    raw_sym = map_phase4_raw_bits(data_bits, h);
                    if (s->phase3_j_prev_valid[h])
                    {
                        int in_sym;
                        uint32_t reg;
                        int dbit[2];
                        int b;

                        in_sym = (raw_sym - s->phase3_j_prev_z[h]) & 0x3;
                        reg = s->phase3_j_scramble[h];
                        dbit[0] = descramble_reg(&reg, s->scrambler_tap, in_sym & 1);
                        dbit[1] = descramble_reg(&reg, s->scrambler_tap, (in_sym >> 1) & 1);
                        s->phase3_j_scramble[h] = reg;
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
                if ((s->duration <= 20 || (s->duration % 64) == 0) && s->phase3_j_bits >= 16)
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
                    span_log(s->logging, SPAN_LOG_FLOW,
                             "Rx - Phase 3 J bits: recent16=%s ordered16=%s phase=%d (%s, d4=%d d16=%d dj'=%d)\n",
                             rx_recent_bits, rx_ordered_bits, best_p, j_validity, d4, d16, djd);
                    if (canonical_ok)
                    {
                        if (pat == 2)
                        {
                            /* Phase 3 transition should be driven by J (Table 18).
                               Treat J' hits here as diagnostics only to avoid
                               premature Phase 4 transitions with unknown TRN size. */
                            span_log(s->logging, SPAN_LOG_FLOW,
                                     "Rx - Phase 3: canonical J' candidate ignored (hyp=%d phase=%d score=%d/32 bits=%d)\n",
                                     best_h, best_p, best_score, s->phase3_j_bits);
                        }
                        else
                        {
                            s->received_event = V34_EVENT_J;
                            s->phase3_j_lock_hyp = best_h;
                            s->phase3_j_trn16 = pat;
                            span_log(s->logging, SPAN_LOG_FLOW,
                                     "Rx - Phase 3: explicit J detected (hyp=%d phase=%d score=%d/32 bits=%d, trn=%s)\n",
                                     best_h, best_p, best_score, s->phase3_j_bits,
                                     pat ? "16-point" : "4-point");
                        }
                    }
                    else
                    {
                        span_log(s->logging, SPAN_LOG_FLOW,
                                 "Rx - Phase 3: J candidate rejected (non-canonical 16-bit pattern)\n");
                    }
                }
                /*endif*/

                /* Phase 3 TRN ones-based lock hint (4-point path):
                   score hypotheses by descrambled ones ratio. This runs before
                   explicit J is decoded and helps stabilize later phase search. */
                if (s->phase3_j_trn16 < 0  &&  s->duration >= 256)
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

                        raw_sym = map_phase4_raw_bits(data_bits, h);
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
            /*endif*/

            idx = (s->duration - 1) & 31;
            int old_sym = s->phase3_s_ring[idx] & 3;

            if (s->duration > 32)
                s->phase3_s_counts[old_sym]--;
            /*endif*/
            s->phase3_s_ring[idx] = (uint8_t) (data_bits & 3);
            s->phase3_s_counts[data_bits & 3]++;
            s->s_detect_count = s->phase3_s_counts[0];
            if (s->phase3_s_counts[1] > s->s_detect_count)
                s->s_detect_count = s->phase3_s_counts[1];
            if (s->phase3_s_counts[2] > s->s_detect_count)
                s->s_detect_count = s->phase3_s_counts[2];
            if (s->phase3_s_counts[3] > s->s_detect_count)
                s->s_detect_count = s->phase3_s_counts[3];

            /* Independent reversal detector:
               count bauds where current symbol is close to 180° from previous. */
            mag_now = sqrtf(sample->re * sample->re + sample->im * sample->im);
            mag_prev = sqrtf(s->last_sample.re * s->last_sample.re
                             + s->last_sample.im * s->last_sample.im);
            dot = sample->re*s->last_sample.re + sample->im*s->last_sample.im;
            old_rev = (s->duration > 32) ? ((s->s_window >> idx) & 1) : 0;
            new_rev = (mag_now > 0.2f  &&  mag_prev > 0.2f  &&  dot < -0.15f*mag_now*mag_prev) ? 1 : 0;
            if (new_rev)
                s->s_window |= (1u << idx);
            else
                s->s_window &= ~(1u << idx);
            s->bit_count += new_rev - old_rev;  /* reuse bit_count as reversal window count */

        if (s->duration <= 10 || (s->duration % 500) == 0)
        {
            span_log(s->logging, SPAN_LOG_FLOW,
                     "Rx - Phase 3 S baud %d: mag=%.3f data_bits=%d dom=%d/32 rev=%d/32 counts=%d,%d,%d,%d\n",
                     s->duration, mag_now, data_bits, s->s_detect_count, s->bit_count,
                     s->phase3_s_counts[0], s->phase3_s_counts[1],
                     s->phase3_s_counts[2], s->phase3_s_counts[3]);
        }

        /* In this path detection is armed only after local J starts (Phase 3),
           so we can use a less strict threshold than earlier TRN-safe values.
           Reversal count is the most robust indicator across phase ambiguity. */
        if (s->duration >= 6000)
        {
            /* Don't force a false S event; keep searching for a real pattern. */
            span_log(s->logging, SPAN_LOG_FLOW,
                     "Rx - Phase 3: S detect timeout (%d bauds, dom=%d/32 rev=%d/32), continuing search\n",
                     s->duration, s->s_detect_count, s->bit_count);
            s->duration = 0;
            s->s_detect_count = 0;
            s->bit_count = 0;
            s->s_window = 0;
            memset(s->phase3_s_ring, 0, sizeof(s->phase3_s_ring));
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
            phase3_trn_hyp_reset(s);
            s->phase4_j_seen = 0;
            s->phase4_j_lock_hyp = -1;
            s->phase4_trn_after_j = 0;
            phase4_trn_hyp_reset(s);
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

        t = span_container_of(s, v34_state_t, rx);
        ang1 = arctan2(sample->im, sample->re);
        ang2 = arctan2(s->last_sample.im, s->last_sample.re);
        ang3 = ang1 - ang2 + DDS_PHASE(45.0f);
        data_bits = (ang3 >> 30) & 0x3;
        s->duration++;
        if (!s->phase3_pp_started)
        {
            int phase;
            int best_phase;
            float best_err;
            float next_err;
            float mag;

            mag = sqrtf(sym->re * sym->re + sym->im * sym->im);
            best_phase = 0;
            best_err = 0.0f;
            next_err = 0.0f;
            for (phase = 0;  phase < PP_PERIOD_SYMBOLS;  phase++)
            {
                complexf_t cand;
                float err;

                cand = pp_symbols[(s->duration - 1 + phase)%PP_PERIOD_SYMBOLS];
                cand.re *= TRAINING_AMP;
                cand.im *= TRAINING_AMP;
                err = (sym->re - cand.re)*(sym->re - cand.re)
                    + (sym->im - cand.im)*(sym->im - cand.im);
                s->phase3_pp_error[phase] = PHASE3_PP_ACQUIRE_DECAY*s->phase3_pp_error[phase] + err;
                if (phase == 0 || s->phase3_pp_error[phase] < best_err)
                {
                    next_err = best_err;
                    best_err = s->phase3_pp_error[phase];
                    best_phase = phase;
                }
                else if (next_err == 0.0f || s->phase3_pp_error[phase] < next_err)
                {
                    next_err = s->phase3_pp_error[phase];
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
            if (next_err > best_err)
                s->phase3_pp_phase_score = (int) lrintf(next_err - best_err);
            else
                s->phase3_pp_phase_score = 0;
            /*endif*/

            if (s->duration <= 10 || (s->duration % 64) == 0)
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
                   In practice this avoids false PP locks on the S/!S interval. */
                if (t->tx.stage < V34_TX_STAGE_FIRST_NOT_S)
                {
                    goto phase3_training_done;
                }
                /*endif*/

                acquire_bauds = s->duration;
                s->phase3_pp_started = 1;
                s->duration = 0;
                memset(s->phase3_pp_lag8, 0, sizeof(s->phase3_pp_lag8));
                s->phase3_pp_obs = 0;
                s->phase3_pp_match = 0;
                span_log(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 3: PP start detected (phase=%d score=%d after %d bauds), starting supervised PP conditioning\n",
                         s->phase3_pp_phase, s->phase3_pp_phase_score, acquire_bauds);
            }
            /*endif*/
        }
        else if (s->duration <= PHASE3_PP_TRAIN_BAUDS)
        {
            complexf_t pp_target;
            float target_mag;
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

            pp_target = pp_symbols[(pp_baud - 1 + s->phase3_pp_phase)%PP_PERIOD_SYMBOLS];
            pp_target.re *= TRAINING_AMP;
            pp_target.im *= TRAINING_AMP;
            target_mag = sqrtf(pp_target.re*pp_target.re + pp_target.im*pp_target.im);
            s->eq_target_mag = target_mag;
            tune_equalizer(s, sym, &pp_target);

            if (pp_baud == 1)
            {
                span_log(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 3: conditioning on aligned PP sequence (%d bauds)\n",
                         PHASE3_PP_TRAIN_BAUDS);
            }
            /*endif*/
            if (pp_baud <= 10 || (pp_baud % 96) == 0)
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

                raw_sym = map_phase4_raw_bits(data_bits, h);
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
                else if ((s->phase3_trn_bits % 256) == 0)
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
            }
            /*endif*/
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
        if (s->duration <= 32)
        {
            uint32_t ang_abs = arctan2(sym->im, sym->re);
            float deg = (float)ang_abs / (4294967296.0f / 360.0f);
            float deg_diff = (float)ang3 / (4294967296.0f / 360.0f);
            fprintf(stderr, "[IQ] baud=%d re=%.4f im=%.4f ang=%.1f diff=%.1f data=%d\n",
                    s->duration, sym->re, sym->im, deg, deg_diff, data_bits);
        }

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
            phase4_trn_hyp_reset(s);
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
            phase4_trn_hyp_reset(s);
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
            phase4_trn_hyp_reset(s);
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
            int h;
            int best_h;
            int best_p;
            int best_score;

            best_h = -1;
            best_p = 0;
            best_score = -1;
            for (h = 0;  h < 8;  h++)
            {
                int raw_sym;
                int in_sym;
                uint32_t reg;
                int dbit[2];
                int b;

                raw_sym = map_phase4_raw_bits(data_bits, h);
                if (s->phase3_j_prev_valid[h])
                {
                    in_sym = (raw_sym - s->phase3_j_prev_z[h]) & 0x3;
                    reg = s->phase3_j_scramble[h];
                    dbit[0] = descramble_reg(&reg, s->scrambler_tap, in_sym & 1);
                    dbit[1] = descramble_reg(&reg, s->scrambler_tap, (in_sym >> 1) & 1);
                    s->phase3_j_scramble[h] = reg;
                    s->phase3_j_stream[h] = ((s->phase3_j_stream[h] << 1) | (uint32_t) dbit[0]) & 0xFFFFFFFFU;
                    s->phase3_j_stream[h] = ((s->phase3_j_stream[h] << 1) | (uint32_t) dbit[1]) & 0xFFFFFFFFU;

                    for (b = 0;  b < 2;  b++)
                    {
                        int bit_pos;
                        int p;

                        bit_pos = s->phase3_j_bits + b;
                        for (p = 0;  p < 16;  p++)
                        {
                            int match;
                            uint32_t w;
                            int score;

                            match = (dbit[b] == phase3_j_pattern_bit(2, bit_pos + p)) ? 1 : 0;
                            w = s->phase3_j_win[h][2][p];
                            w = ((w << 1) | match) & 0xFFFFFFFFU;
                            s->phase3_j_win[h][2][p] = w;
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
                /*endif*/
                s->phase3_j_prev_z[h] = (uint8_t) raw_sym;
                s->phase3_j_prev_valid[h] = 1;
            }
            /*endfor*/

            s->phase3_j_bits += 2;
            if ((s->duration <= 20 || (s->duration % 64) == 0) && s->phase3_j_bits >= 16)
            {
                span_log(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 4 J' detector: bits=%d best_score=%d/32 hyp=%d\n",
                         s->phase3_j_bits, best_score, best_h);
            }
            /*endif*/

            if (s->phase3_j_bits >= 32
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

                rx_recent16 = (uint16_t) (s->phase3_j_stream[best_h] & 0xFFFFU);
                rx_ordered16 = j_ordered16(rx_recent16, s->phase3_j_bits, best_p);
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
                span_log(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 4 J bits: recent16=%s ordered16=%s phase=%d (%s, d4=%d d16=%d dj'=%d)\n",
                         rx_recent_bits, rx_ordered_bits, best_p, j_validity, d4, d16, djd);
                if (canonical_ok)
                {
                    s->phase4_j_seen = 1;
                    s->phase4_trn_after_j = 0;
                    phase4_trn_hyp_reset(s);
                    s->phase4_j_lock_hyp = best_h;
                    s->received_event = V34_EVENT_J_DASHED;
                    span_log(s->logging, SPAN_LOG_FLOW,
                             "Rx - Phase 4: explicit J' detected (hyp=%d phase=%d score=%d/32 bits=%d)\n",
                             best_h, best_p, best_score, s->phase3_j_bits);
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

        /* Transition to MP scan after explicit J' and TRN ones-lock.
           The far-end TRN may be very short (as few as ~100 bauds), so we
           transition as soon as we have a confident lock — we do NOT require
           the current sliding-window score to still be high, because the
           far-end may have already moved past TRN to MP by the time we check. */
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
            span_log(s->logging, SPAN_LOG_FLOW,
                     "Rx - Phase 4: far-end J' + TRN confirmed (J'->TRN=%d bauds, best_ones=%d%%), scanning for MP (decoder=hyp24-v2, gated=900)\n",
                     s->phase4_trn_after_j, s->phase4_trn_lock_score);
            s->stage = V34_RX_STAGE_PHASE4_MP;
            s->duration = 0;
            s->bitstream = 0;
            s->bit_count = 0;
            s->mp_seen = 0;
            s->received_event = 0;  /* Clear any TRN-phase timeout so MP timeout can fire */
            s->eq_target_mag = 0.0f;  /* Reset so CMA re-seeds with minimum clamp (1.0) */
            s->mp_remote_ack_seen = 0;
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

    case V34_RX_STAGE_PHASE4_MP:
        /* Phase 4 MP detection on primary channel using DQPSK demodulation.
           Same approach as CC channel MP detection: differential phase decoding,
           descrambling, then scanning for the MP preamble and frame. */
        {
            int locked_this_symbol;
            int h;
            int expected_mp_type;
            int abs_bits;

        ang1 = arctan2(sym->im, sym->re);
        ang2 = arctan2(s->last_sample.im, s->last_sample.re);
        ang3 = ang1 - ang2 + DDS_PHASE(45.0f);
        data_bits = (ang3 >> 30) & 0x3;
        abs_bits = (int) ((ang1 + DDS_PHASE(45.0f)) >> 30) & 0x3;
        /* During initial Phase 4 MP acquisition, prefer MP0 (type 0) to avoid
           false MP1 preamble locks. After a couple of rejects, relax back to
           either type for interoperability with non-standard peers. */
        expected_mp_type = (s->mp_seen == 0 && s->mp_phase4_reject_streak < 2) ? 0 : -1;

            locked_this_symbol = 0;

            if (s->mp_hypothesis >= 0)
            {
                int in0;
                int in1;
                int raw_bits;

                raw_bits = map_phase4_raw_bits((s->mp_phase4_domain == 1) ? abs_bits : data_bits,
                                               s->mp_hypothesis);
                phase4_unpack_ordered_bits(raw_bits, s->mp_phase4_bit_order, &in0, &in1);
                bits[0] = descramble(s, in0);
                bits[1] = descramble(s, in1);
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
                int hint_found;
                int hint_score;
                int hint_type_bit;
                uint32_t hint_reg;
                uint32_t hint_bstream;
                int hint_bit_pos;
                int hint_pending_valid;
                int hint_pending_bit;
                uint32_t hint_preamble_stream;

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
                /* Constrain early MP lock to the TRN/J hint until we have
                   accumulated a couple of failed frame attempts. Using absolute
                   phase4 duration is ineffective because MP starts late in phase4. */
                hint_only = (hint_h >= 0
                             && hint_h < MP_HYPOTHESIS_COUNT
                             && (mp_phase4_has_pinned_trn_lock(s)
                                 || s->mp_phase4_reject_streak < MP_HINT_STRICT_REJECTS));
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
                    raw_bits = map_phase4_raw_bits((s->mp_phase4_domain == 1) ? abs_bits : data_bits, h);
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
                        && d1 == 0
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
                        && d1 == 0
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
                    if (sc >= ((hint_only && h == hint_h) ? MP_HINT_LOCK_SCORE_MIN : MP_LOCK_SCORE_MIN)
                        && (!hint_only || h == hint_h)
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
                    if (h == hint_h
                        && sc >= MP_HINT_LOCK_SCORE_MIN
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
        /* MP stage timeout: if we haven't successfully decoded an MP frame
           within ~2s (4800 bauds at 2400 baud rate), signal training failure
           so the call can retrain or drop.  The far-end won't wait forever. */
        if (s->mp_seen == 0
            && s->duration >= 4800
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
            /* If TRN locked a hypothesis, keep those descrambler settings —
               do NOT rotate through modes.  The TRN lock determined the correct
               domain/tap/order; changing them will prevent MP detection. */
            if (mp_phase4_has_pinned_trn_lock(s))
            {
                mp_reset_hypothesis_search(s);
                span_log(s->logging, SPAN_LOG_FLOW,
                         "Rx - Phase 4: no MP lock at baud %d; keeping TRN-locked settings (hyp=%d, dom=%s, tap=%d, ord=%s)\n",
                         s->duration, s->phase4_trn_lock_hyp,
                         phase4_trn_domain_name(s->mp_phase4_domain), s->scrambler_tap,
                         phase4_trn_order_name(s->mp_phase4_bit_order));
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
            && (s->duration <= 30 || (s->duration % 200) == 0))
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
                if (s->mp_hypothesis >= 0  &&  s->mp_frame_pos == 0)
                {
                    int preamble_wait_limit;

                    v34_rx_log_mp_diag_state(s, V34_MP_DIAG_STATE_DET_SYNC, "awaiting MP preamble");

                    /* If we started from a TRN direct pre-lock, fail fast and
                       fall back to global hypothesis search rather than waiting
                       the full window on a possibly phase-offset hypothesis. */
                    preamble_wait_limit = (s->mp_phase4_reject_streak == 0)
                                          ? MP_PRELOCK_PREAMBLE_WAIT_BITS
                                          : MP_PREAMBLE_WAIT_BITS;
                    if (++s->mp_count > preamble_wait_limit)
                    {
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
                    /* Post-MP data */
                    s->put_bit(s->put_bit_user_data, bits[i]);
                    continue;
                }
                /*endif*/
                if (s->mp_seen == 1  &&  (s->bitstream & 0xFFFFF) == 0xFFFFF)
                {
                    /* E is 20 consecutive ones — end of MP exchange */
                    v34_rx_log_mp_diag_state(s, V34_MP_DIAG_STATE_COMPLETE, "E detected");
                    span_log(s->logging, SPAN_LOG_FLOW,
                             "Rx - Phase 4: E signal detected, MP exchange complete\n");
                    s->mp_seen = 2;
                    if (s->duplex)
                        report_status_change(s, SIG_STATUS_TRAINING_SUCCEEDED);
                    /*endif*/
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
                        s->mp_frame_target = (type == 1)  ?  188  :  88;
                        s->mp_frame_pos = 19;
                        s->mp_count = 0;
                        s->bit_count = 0;
                        s->mp_early_rejects = 0;
                        v34_rx_log_mp_diag_state(s, V34_MP_DIAG_STATE_DET_INFO, "MP preamble found; collecting frame");

                        bits32_to_str(s->bitstream, tail);
                        span_log(s->logging, SPAN_LOG_FLOW,
                                 "Rx - Phase 4: MP preamble detected (baud %d): "
                                 "score=%d/18 17x'1'+start(0)+type(%d), target=%d bits, "
                                 "frame body starts at frame_idx=19 (includes inserted start bits), tail=0b%s\n",
                                 s->duration, preamble_score, type, s->mp_frame_target, tail);
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

                            data_idx = mp_data_bit_index(type_now, idx);
                            s->mp_early_rejects++;
                            if (s->mp_early_rejects <= 3  ||  (s->mp_early_rejects % 8) == 0)
                            {
                                span_log(s->logging, SPAN_LOG_FLOW,
                                         "Rx - Phase 4: MP%d start-bit mismatch at frame_idx=%d body_idx=%d data_idx=%d value=%d "
                                         "(expected 0), start_err_count=%d (continuing until CRC)\n",
                                         type_now, idx, idx - 19 + 1, data_idx, bits[i], s->mp_early_rejects);
                            }
                            /*endif*/
                            /* Do not fast-reject on first inserted start mismatch.
                               Keep collecting through CRC so slip/boundary recovery
                               can salvage near-correct locks. */
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
                            s->mp_early_rejects++;
                            span_log(s->logging, SPAN_LOG_FLOW,
                                     "Rx - Phase 4: MP%d reserved bit mismatch at frame_idx=19 value=%d "
                                     "(expected 0), early_reject_count=%d (continuing until CRC)\n",
                                     type_now, bits[i], s->mp_early_rejects);
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
                        /* For MP0 debug, log every body bit so bit drift is visible.
                           For MP1, keep denser periodic logs plus all inserted starts. */
                        log_body_bit = (s->mp_frame_target <= 88)
                                       || s->bit_count <= 40
                                       || (s->bit_count % 8) == 0
                                       || is_inserted_start;
                        if (log_body_bit)
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
                    log_mp_frame_diag(s, s->mp_frame_bits, type, crc_good, rx_crc, residual_crc, fill_good);
                    log_mp_tx_rx_compare(s, s->mp_frame_bits, type);
                    {
                        bool frame_accepted;
                        int start_err_accept_max;
                        bool starts_acceptable;

                        frame_accepted = false;
                        start_err_accept_max = (type == 1) ? MP1_START_ERR_ACCEPT_MAX : 0;
                        starts_acceptable = (start_err_count <= start_err_accept_max);
                        if (crc_good  &&  fill_good)
                        {
                            bool semantic_good;

                            semantic_good = true;
                            if (s->duplex)
                            {
                                mp_pack_for_parser(s->info_buf, s->mp_frame_bits, type);
                                process_rx_mp(s, &mp, s->info_buf);
                                semantic_good = mp_semantic_ok_phase4(s, &mp, type, s->mp_frame_bits);
                                if (semantic_good)
                                {
                                    if (mp.mp_acknowledged)
                                        s->mp_remote_ack_seen = 1;
                                    /*endif*/
                                    t = span_container_of(s, v34_state_t, rx);
                                    if (mp.type == 1)
                                        memcpy(&t->tx.precoder_coeffs, mp.precoder_coeffs, sizeof(t->tx.precoder_coeffs));
                                    /*endif*/
                                    switch (mp.trellis_size)
                                    {
                                    case V34_TRELLIS_16:
                                        t->tx.conv_encode_table = v34_conv16_encode_table;
                                        break;
                                    case V34_TRELLIS_32:
                                        t->tx.conv_encode_table = v34_conv32_encode_table;
                                        break;
                                    case V34_TRELLIS_64:
                                        t->tx.conv_encode_table = v34_conv64_encode_table;
                                        break;
                                    default:
                                        span_log(&t->logging, SPAN_LOG_FLOW,
                                                 "Rx - Unexpected trellis size code %d\n", mp.trellis_size);
                                        semantic_good = false;
                                        break;
                                    }
                                    /*endswitch*/
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
                                span_log(s->logging, SPAN_LOG_FLOW,
                                         "Rx - Phase 4: MP%d frame accepted (%d bits)\n",
                                         type, s->mp_frame_target);
                                s->mp_seen = 1;
                                s->mp_early_rejects = 0;
                                v34_rx_log_mp_diag_state(s, V34_MP_DIAG_STATE_DET_INFO, "MP frame accepted; awaiting E");
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
                                               && start_err_count == 0);
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
                            if (keep_hypothesis)
                            {
                                s->mp_early_rejects = 0;
                                s->mp_count = 0;
                                span_log(s->logging, SPAN_LOG_FLOW,
                                         "Rx - Phase 4: keeping MP hypothesis=%d after CRC-only reject; retrying local preamble reacquire\n",
                                         s->mp_hypothesis);
                                v34_rx_log_mp_diag_state(s, V34_MP_DIAG_STATE_DET_SYNC, "CRC-only reject; reacquiring preamble");
                            }
                            else
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

    default:
        /* Normal data mode operation - not yet implemented */
        break;
    }
    /*endswitch*/

    /* Decision-directed carrier tracking and equalizer training for DQPSK during
       Phase 3/4 training.  Snap to the nearest QPSK constellation point using a
       FIXED target magnitude (EMA of equalizer output) so the equalizer corrects
       both phase AND amplitude distortion (ISI).  Using the received magnitude as
       the target (as before) gives the equalizer zero amplitude-correction incentive
       and leaves ISI uncorrected. */
    if (s->stage >= V34_RX_STAGE_PHASE3_WAIT_S
    &&  s->stage <= V34_RX_STAGE_PHASE4_MP)
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

            if (phase3_equalizer_refine_active(s)
                || s->stage == V34_RX_STAGE_PHASE4_TRN
                || s->stage == V34_RX_STAGE_PHASE4_MP)
            {
                /* CMA (blind) equalizer — during Phase 3 TRN refinement and
                   Phase 4.  Phase 4 needs CMA to adapt equalizer gain for
                   variable signal levels on analog/SIP channels. */
                tune_equalizer_cma(s, sym);
            }
            /*endif*/

            /* Re-enabled carrier tracking — test 4 showed MP detection worked
               better with carrier tracking on.  CMA equalization now provides
               more stable magnitude for eq_target, improving tracking quality. */
            if (!phase4_trn_should_freeze_tracking(s))
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
    for (i = 0;  i < len;  i++)
    {
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
            /* Only AGC until we have locked down the setting. */
#if defined(SPANDSP_USE_FIXED_POINT)
            //if (s->agc_scaling_save == 0)
            //    s->agc_scaling = saturate16(((int32_t) (1024.0f*FP_SCALE(2.17f)))/fixed_sqrt32(power));
            ///*endif*/
#else
            //if (s->agc_scaling_save == 0.0f)
            //    s->agc_scaling = (FP_SCALE(2.17f)/RX_PULSESHAPER_GAIN)/fixed_sqrt32(power);
            ///*endif*/
#endif
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
#define BYPASS_VITERBI
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
            /* Deal with super-frame sync inversion */
            if ((s->data_frame*8 + s->step_2d)%(4*s->parms.p) == 0)
                invert = (0x5FEE >> s->v0_pattern++) & 1;
            else
                invert = false;
            /*endif*/
            viterbi_calculate_branch_errors(&s->viterbi, s->xy, invert);
            viterbi_update_path_metrics(&s->viterbi);
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

int v34_rx_restart(v34_state_t *s, int baud_rate, int bit_rate, int high_carrier)
{
    int i;

    s->rx.baud_rate = baud_rate;
    s->rx.bit_rate = bit_rate;
    s->rx.high_carrier = high_carrier;

    s->rx.v34_carrier_phase_rate = dds_phase_ratef(carrier_frequency(s->rx.baud_rate, s->rx.high_carrier));
    /* Phase 2 INFO exchange: answerer RX at 1200 Hz (tone B), caller RX at 2400 Hz (tone A).
       This gets updated to Phase 4 CC frequencies in mp_or_mph_baud_init(). */
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
                         (s->calling_party)  ?  2400.0f  :  1200.0f,
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
    memset(s->rx.phase3_s_ring, 0, sizeof(s->rx.phase3_s_ring));
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
    phase3_trn_hyp_reset(&s->rx);
    s->rx.phase4_j_seen = 0;
    s->rx.phase4_j_lock_hyp = -1;
    s->rx.phase4_trn_after_j = 0;
    phase4_trn_hyp_reset(&s->rx);

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
    s->rx.mp_phase4_retry_mode = 0;
    s->rx.mp_phase4_bit_order = 0;
    s->rx.mp_phase4_domain = 0;
    mp_reset_hypothesis_search(&s->rx);
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
    s->rx.viterbi.conv_decode_table = v34_conv16_decode_table;

    s->rx.v0_pattern = 0;
    s->rx.super_frame = 0;
    s->rx.data_frame = 0;
    s->rx.s_bit_cnt = 0;
    s->rx.aux_bit_cnt = 0;

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
