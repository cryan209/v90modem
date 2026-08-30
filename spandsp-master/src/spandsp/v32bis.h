/*
 * SpanDSP - a series of DSP components for telephony
 *
 * v32bis.h - ITU V.32bis modem
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

/* V.32bis SUPPORT IS A WORK IN PROGRESS - NOT YET FUNCTIONAL! */

/*! \file */

/*! \page v32bis_page The V.32bis modem
\section v32bis_page_sec_1 What does it do?

\section v32bis__page_sec_2 How does it work?
*/

#if !defined(_SPANDSP_V32BIS_H_)
#define _SPANDSP_V32BIS_H_

#if defined(SPANDSP_USE_FIXED_POINT)
#define V32BIS_CONSTELLATION_SCALING_FACTOR     4096.0
#else
#define V32BIS_CONSTELLATION_SCALING_FACTOR     1.0
#endif

enum
{
    V32BIS_RATE_14400 = 0x1000,
    V32BIS_RATE_12000 = 0x0400,
    V32BIS_RATE_9600 = 0x0200,
    V32BIS_RATE_7200 = 0x0040,
    V32BIS_RATE_4800 = 0x0020
};

/* State labels used by the V.32bis section 5.2 conditioning signal.  Their
   numeric values are also the Table 1 differential states used for startup
   words. */
enum
{
    V32BIS_STARTUP_A = 0,
    V32BIS_STARTUP_B = 1,
    V32BIS_STARTUP_D = 2,
    V32BIS_STARTUP_C = 3
};

/*!
    V.32bis modem descriptor. This defines the working state for a single instance
    of a V.32bis modem.
*/
typedef struct v32bis_state_s v32bis_state_t;

#if defined(__cplusplus)
extern "C"
{
#endif

/*! Reinitialise an existing V.32bis modem receive context.
    \brief Reinitialise an existing V.32bis modem receive context.
    \param s The modem context.
    \param bit_rate The bit rate of the modem. Valid values are 4800, 7200, 9600, 12000 and 14400.
    \return 0 for OK, -1 for bad parameter */
SPAN_DECLARE(int) v32bis_rx_restart(v32bis_state_t *s, int bit_rate);

/*! Process a block of received V.32bis modem audio samples.
    \brief Process a block of received V.32bis modem audio samples.
    \param s The modem context.
    \param amp The audio sample buffer.
    \param len The number of samples in the buffer.
    \return The number of samples unprocessed. */
SPAN_DECLARE(int) v32bis_rx(v32bis_state_t *s, const int16_t amp[], int len);

/*! Fake processing of a missing block of received V.32bis modem audio samples.
    (e.g due to packet loss).
    \brief Fake processing of a missing block of received V.32bis modem audio samples.
    \param s The modem context.
    \param len The number of samples to fake.
    \return The number of samples unprocessed. */
SPAN_DECLARE(int) v32bis_rx_fillin(v32bis_state_t *s, int len);

/*! Get a snapshot of the current equalizer coefficients.
    \brief Get a snapshot of the current equalizer coefficients.
    \param coeffs The vector of complex coefficients.
    \return The number of coefficients in the vector. */
#if defined(SPANDSP_USE_FIXED_POINT)
SPAN_DECLARE(int) v32bis_equalizer_state(v32bis_state_t *s, complexi16_t **coeffs);
#else
SPAN_DECLARE(int) v32bis_equalizer_state(v32bis_state_t *s, complexf_t **coeffs);
#endif

/*! Get the current received carrier frequency.
    \param s The modem context.
    \return The frequency, in Hertz. */
SPAN_DECLARE(float) v32bis_rx_carrier_frequency(v32bis_state_t *s);

/*! Get the current symbol timing correction since startup.
    \param s The modem context.
    \return The correction. */
SPAN_DECLARE(float) v32bis_rx_symbol_timing_correction(v32bis_state_t *s);

/*! Get a current received signal power.
    \param s The modem context.
    \return The signal power, in dBm0. */
SPAN_DECLARE(float) v32bis_rx_signal_power(v32bis_state_t *s);

/*! Set the power level at which the carrier detection will cut in
    \param s The modem context.
    \param cutoff The signal cutoff power, in dBm0. */
SPAN_DECLARE(void) v32bis_rx_set_signal_cutoff(v32bis_state_t *s, float cutoff);

/*! Set a handler routine to process QAM status reports
    \param s The modem context.
    \param handler The handler routine.
    \param user_data An opaque pointer passed to the handler routine. */
SPAN_DECLARE(void) v32bis_set_qam_report_handler(v32bis_state_t *s, qam_report_handler_t handler, void *user_data);

/*! Generate a block of V.32bis modem audio samples.
    \brief Generate a block of V.32bis modem audio samples.
    \param s The modem context.
    \param amp The audio sample buffer.
    \param len The number of samples to be generated.
    \return The number of samples actually generated. */
SPAN_DECLARE(int) v32bis_tx(v32bis_state_t *s, int16_t amp[], int len);

/*! Adjust a V.32bis modem transmit context's power output.
    \brief Adjust a V.32bis modem transmit context's output power.
    \param s The modem context.
    \param power The power level, in dBm0 */
SPAN_DECLARE(void) v32bis_tx_power(v32bis_state_t *s, float power);

/*! Set the supported bit rates for a V.32bis modem context.
    \brief Set the supported bit rates for a V.32bis modem context.
    \param s The modem context.
    \param rates The bit rate mask
    \return 0 for OK, -1 for bad parameter. */
SPAN_DECLARE(int) v32bis_set_supported_bit_rates(v32bis_state_t *s, int rates);

/*! Build or decode the Table 5 rate signal. */
SPAN_DECLARE(int) v32bis_build_rate_signal(int rates, uint16_t *word);
SPAN_DECLARE(int) v32bis_decode_rate_signal(uint16_t word, int *rates);

/*! Build or decode the Table 6 E signal. */
SPAN_DECLARE(int) v32bis_build_e_signal(int bit_rate, uint16_t *word);
SPAN_DECLARE(int) v32bis_decode_e_signal(uint16_t word, int *bit_rate);

/*! Select the highest rate present in both Table 5 masks, or zero when the
    modems have no common rate. */
SPAN_DECLARE(int) v32bis_select_common_rate(int local_rates, int remote_rates);

/*! Generate section 5.2's S, S-bar and TRN conditioning states.  `states`
    must hold 272 + trn_symbols entries.  The returned scrambler and
    differential state are the state at the TRN-to-rate-word handoff. */
SPAN_DECLARE(int) v32bis_build_conditioning(bool calling_party,
                                             int trn_symbols,
                                             uint8_t states[],
                                             uint32_t *scrambler_register,
                                             int *diff_state);

/*! Scramble and Table-1 encode one 16-bit startup word, least-significant
    bit first.  `scrambler_register` and `diff_state` are updated in place. */
SPAN_DECLARE(int) v32bis_encode_startup_word(bool calling_party,
                                             uint16_t word,
                                             uint32_t *scrambler_register,
                                             int *diff_state,
                                             uint8_t states[8]);

/*! Invert `v32bis_encode_startup_word` for a received R or E word.  The
    calling-party argument describes the remote transmitter. */
SPAN_DECLARE(int) v32bis_decode_startup_word(bool calling_party,
                                             const uint8_t states[8],
                                             uint32_t *descrambler_register,
                                             int *diff_state,
                                             uint16_t *word);

/*! Queue one native startup burst (conditioning, two identical Table 5 words,
    and Table 6 E) into the shared V.17 pulse shaper.  The remote mask is the
    already-received R word for this phase. */
SPAN_DECLARE(int) v32bis_prepare_startup_tx(v32bis_state_t *s, int remote_rates);

/*! Number of startup symbols supplied to the TX pulse shaper or delivered by
    the RX carrier/timing/equalizer front end. */
SPAN_DECLARE(int) v32bis_startup_tx_symbols_sent(v32bis_state_t *s);
SPAN_DECLARE(int) v32bis_startup_rx_symbols_seen(v32bis_state_t *s);

/*! Report the current operating bit rate of a V.32bis modem context.
    \brief Report the current operating bit rate of a V.22bis modem context
    \param s The modem context. */
SPAN_DECLARE(int) v32bis_current_bit_rate(v32bis_state_t *s);

/*! Reinitialise an existing V.32bis modem context, so it may be reused.
    \brief Reinitialise an existing V.32bis modem context.
    \param s The modem context.
    \param bit_rate The bit rate of the modem. Valid values are 4800, 7200, 9600, 12000 and 14400.
    \return 0 for OK, -1 for bad parameter. */
SPAN_DECLARE(int) v32bis_restart(v32bis_state_t *s, int bit_rate);

/*! Initialise a V.32bis modem context. This must be called before the first
    use of the context, to initialise its contents.
    \brief Initialise a V.32bis modem context.
    \param s The modem context.
    \param bit_rate The bit rate of the modem. Valid values are 4800, 7200, 9600, 12000 and 14400.
    \param calling_party True if this is the calling modem.
    \param get_bit The callback routine used to get the data to be transmitted.
    \param get_bit_user_data An opaque pointer, passed in calls to the get routine.
    \param put_bit The callback routine used to get the data to be transmitted.
    \param put_bit_user_data An opaque pointer, passed in calls to the put routine.
    \return A pointer to the modem context, or NULL if there was a problem.
    \param get_bit_user_data An opaque pointer, passed in calls to the put routine. */
SPAN_DECLARE(v32bis_state_t *) v32bis_init(v32bis_state_t *s,
                                           int bit_rate,
                                           bool calling_party,
                                           get_bit_func_t get_bit,
                                           void *get_bit_user_data,
                                           put_bit_func_t put_bit,
                                           void *put_bit_user_data);

/*! Release a V.32bis modem receive context.
    \brief Release a V.32bis modem receive context.
    \param s The modem context.
    \return 0 for OK */
SPAN_DECLARE(int) v32bis_release(v32bis_state_t *s);

/*! Free a V.32bis modem receive context.
    \brief Free a V.32bis modem receive context.
    \param s The modem context.
    \return 0 for OK */
SPAN_DECLARE(int) v32bis_free(v32bis_state_t *s);

/*! Get the logging context associated with a V.32bis modem context.
    \brief Get the logging context associated with a V.32bis modem context.
    \param s The modem context.
    \return A pointer to the logging context */
SPAN_DECLARE(logging_state_t *) v32bis_get_logging_state(v32bis_state_t *s);

/*! Change the get_bit function associated with a V.32bis modem context.
    \brief Change the get_bit function associated with a V.32bis modem context.
    \param s The modem context.
    \param get_bit The callback routine used to get the data to be transmitted.
    \param user_data An opaque pointer. */
SPAN_DECLARE(void) v32bis_set_get_bit(v32bis_state_t *s, get_bit_func_t get_bit, void *user_data);

/*! Change the get_bit function associated with a V.32bis modem context.
    \brief Change the put_bit function associated with a V.32bis modem context.
    \param s The modem context.
    \param put_bit The callback routine used to process the data received.
    \param user_data An opaque pointer. */
SPAN_DECLARE(void) v32bis_set_put_bit(v32bis_state_t *s, put_bit_func_t put_bit, void *user_data);

#if defined(__cplusplus)
}
#endif

#endif
/*- End of file ------------------------------------------------------------*/
