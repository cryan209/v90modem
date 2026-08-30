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
    if (s->startup_tx_symbol_count <= 0)
        return -1;
    if (s->startup_tx_symbol_pos < s->startup_tx_symbol_count)
        state = s->startup_tx_symbols[s->startup_tx_symbol_pos++];
    else
        state = s->startup_tx_symbols[s->startup_tx_symbol_count - 1];
    *symbol = v17_v32bis_4800_constellation[state & 3];
    return 0;
}
/*- End of function --------------------------------------------------------*/

#if defined(SPANDSP_USE_FIXED_POINTx)
static void v32bis_startup_symbol_sink(void *user_data, const complexi16_t *symbol)
#else
static void v32bis_startup_symbol_sink(void *user_data, const complexf_t *symbol)
#endif
{
    v32bis_state_t *s;

    (void) symbol;
    s = (v32bis_state_t *) user_data;
    s->startup_rx_symbol_count++;
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
    s->startup_rx_symbol_count = 0;
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
    s->startup_rx_symbol_count = 0;
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
