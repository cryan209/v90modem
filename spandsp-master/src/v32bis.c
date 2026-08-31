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

SPAN_DECLARE(int) v32bis_tx(v32bis_state_t *s, int16_t amp[], int len)
{
    return v17_tx(&s->tx, amp, len);
}
/*- End of function --------------------------------------------------------*/

SPAN_DECLARE(int) v32bis_rx(v32bis_state_t *s, const int16_t amp[], int len)
{
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
#define V32BIS_RATE_FIXED_BITS   0x0190
#define V32BIS_RATE_SYNC_MASK    0x888F
#define V32BIS_RATE_SYNC_VALUE   0x0080
#define V32BIS_E_FIXED_BITS      0x899F
#define V32BIS_E_SYNC_MASK       0xE99F
#define V32BIS_S_SYMBOLS         256
#define V32BIS_S_BAR_SYMBOLS     16
#define V32BIS_SCRAMBLER_MASK    0x7FFFFF
/*! The V.17 equalizer LMS step, fast and annealed. Kept in step with
    EQUALIZER_FAST_ADAPTION_DELTA in v17rx.c. */
#define V32BIS_EQ_DELTA_FAST     (0.21f/33)
#define V32BIS_EQ_DELTA_SLOW     (v32bis_eq_delta_slow())
/*! TRN symbols spent at the fast step before annealing. */
#define V32BIS_TRN_FAST_SYMBOLS  (v32bis_trn_fast_symbols())

/*! Energy-normalized LMS. Default OFF: measured over the five rate rows in
    both laws it is worse than the plain step at the step size the plain step
    was tuned for (9433 bit errors against 5144), so it needs its own eq_delta
    retune before it can be adopted. V32BIS_NLMS=1 enables it. */
static bool v32bis_use_nlms(void)
{
    const char *e = getenv("V32BIS_NLMS");

    return (e != NULL  &&  atoi(e) != 0);
}
/*- End of function --------------------------------------------------------*/

static float v32bis_eq_delta_slow(void)
{
    const char *e = getenv("V32BIS_EQ_SLOW");

    return (e != NULL) ? (float) atof(e)*V32BIS_EQ_DELTA_FAST
                       : 0.1f*V32BIS_EQ_DELTA_FAST;
}
/*- End of function --------------------------------------------------------*/

static int v32bis_trn_fast_symbols(void)
{
    const char *e = getenv("V32BIS_TRN_FAST");
    int n;

    if (e != NULL  &&  (n = atoi(e)) >= 0)
        return n;
    /* Swept over both laws at all five rates: 160/320/640 give 4539/5144/5415
       total bit errors at the 0.1 anneal. */
    return 160;
}
/*- End of function --------------------------------------------------------*/
/*! ITU-T V.32bis 6.  B1 is the marks segment between E and data. */
#define V32BIS_B1_SYMBOLS        (v32bis_b1_symbols())

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

    if (bit_rate == NULL  ||  (word & V32BIS_E_SYNC_MASK) != V32BIS_E_FIXED_BITS)
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

SPAN_DECLARE(int) v32bis_decode_startup_word(bool calling_party,
                                             const uint8_t states[8],
                                             uint32_t *descrambler_register,
                                             int *diff_state,
                                             uint16_t *word)
{
    /* Table 1's 4800 bit/s differential transform is self-inverse when each
       row is indexed by the received state. */
    static const uint8_t differential_decoder[4][4] =
    {
        {2, 3, 0, 1},
        {0, 2, 1, 3},
        {3, 1, 2, 0},
        {1, 0, 3, 2}
    };
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
        dibit = differential_decoder[previous][states[i]];
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

#if defined(SPANDSP_USE_FIXED_POINT)
static int v32bis_startup_symbol_source(void *user_data, complexi16_t *symbol)
#else
static int v32bis_startup_symbol_source(void *user_data, complexf_t *symbol)
#endif
{
    v32bis_state_t *s;
    int state;

    s = (v32bis_state_t *) user_data;
    if (s->startup_tx_symbol_count <= 0
        || s->startup_tx_symbol_pos >= s->startup_tx_symbol_count)
    {
        s->tx.symbol_source = NULL;
        s->tx.symbol_source_user_data = NULL;
        return -1;
    }
    state = s->startup_tx_symbols[s->startup_tx_symbol_pos++];
    *symbol = v17_v32bis_4800_constellation[state & 3];
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
    V32BIS_RX_DATA
};

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
            p = &v17_v32bis_4800_constellation[(i + parity) & 1];
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
            p = &v17_v32bis_4800_constellation[(i + parity) & 1];
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

static int startup_b1_symbol(v32bis_state_t *s)
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
    return ((bits << 1) & 0x78)
         | (s->startup_rx_b1_diff << 1)
         | ((s->startup_rx_b1_convolution >> 2) & 1);
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
    s->rx.bit_rate = bit_rate;
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
        s->startup_rx_stage = V32BIS_RX_E;
    }
    else if (s->startup_rx_stage == V32BIS_RX_E)
    {
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
        s->startup_selected_rate = rate;
        s->bit_rate = rate;
        s->startup_rx_b1_pos = 0;
        s->startup_rx_b1_reg = reg;
        s->startup_rx_b1_diff = diff;
        s->startup_rx_b1_convolution = 0;
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
            s->startup_rx_stage = V32BIS_RX_S_BAR;
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
        s->rx.eq_normalized_lms = v32bis_use_nlms();
        s->rx.eq_delta = (s->startup_rx_trn_pos < V32BIS_TRN_FAST_SYMBOLS)
                       ? V32BIS_EQ_DELTA_FAST
                       : V32BIS_EQ_DELTA_SLOW;
        if (++s->startup_rx_trn_pos >= 1280)
        {
            s->startup_rx_stage = V32BIS_RX_R_FIRST;
            s->startup_rx_word_pos = 0;
        }
        return expected;
    case V32BIS_RX_R_FIRST:
    case V32BIS_RX_R_SECOND:
    case V32BIS_RX_E:
        state = startup_nearest_state(s, &z);
        s->startup_rx_word_states[s->startup_rx_word_pos++] = (uint8_t) state;
        if (s->startup_rx_word_pos >= 8)
            startup_finish_word(s);
        return -1;
    case V32BIS_RX_B1:
        state = startup_b1_symbol(s);
        if (++s->startup_rx_b1_pos >= V32BIS_B1_SYMBOLS)
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
            s->startup_rx_stage = V32BIS_RX_DATA;
            s->rx.symbol_sink = NULL;
            s->rx.symbol_sink_user_data = NULL;
            s->rx.symbol_sink_uses_data_constellation = false;
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
    word_reg = trn_reg;
    word_diff = trn_diff;
    if (v32bis_encode_startup_word(s->calling_party,
                                   word,
                                   &word_reg,
                                   &word_diff,
                                   word_states) != 8)
        return -1;
    /* Section 6 requires at least two identical consecutive R words. */
    memcpy(&s->startup_tx_symbols[count], word_states, sizeof(word_states));
    count += (int) sizeof(word_states);
    memcpy(&s->startup_tx_symbols[count], word_states, sizeof(word_states));
    count += (int) sizeof(word_states);

    if (v32bis_build_e_signal(selected_rate, &word) != 0)
        return -1;
    word_reg = trn_reg;
    word_diff = trn_diff;
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
    s->startup_remote_rates = 0;
    s->startup_selected_rate = 0;
    s->startup_complete = false;
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
    startup_rx_reset(s);
    s->tx.symbol_source = NULL;
    s->tx.symbol_source_user_data = NULL;
    s->rx.symbol_sink = v32bis_startup_symbol_sink;
    s->rx.symbol_sink_user_data = s;

    s->bit_rate = bit_rate;
    return 0;
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
    s->ec = modem_echo_can_segment_init(256);

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
