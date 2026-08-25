/*
 * SpanDSP - a series of DSP components for telephony
 *
 * v34.h - ITU V.34 modem
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

/*! \page v34_page The V.34 modem
\section v34_page_sec_1 What does it do?

\section v34__page_sec_2 How does it work?
*/

#if !defined(_SPANDSP_V34_H_)
#define _SPANDSP_V34_H_

#if defined(SPANDSP_USE_FIXED_POINT)
#define V34_CONSTELLATION_SCALING_FACTOR        512.0
#else
#define V34_CONSTELLATION_SCALING_FACTOR        1.0
#endif

enum v34_supported_bit_rates_e
{
    V34_SUPPORT_2400  = 0x0001,
    V34_SUPPORT_4800  = 0x0002,
    V34_SUPPORT_7200  = 0x0004,
    V34_SUPPORT_9600  = 0x0008,
    V34_SUPPORT_12000 = 0x0010,
    V34_SUPPORT_14400 = 0x0020,
    V34_SUPPORT_16800 = 0x0040,
    V34_SUPPORT_19200 = 0x0080,
    V34_SUPPORT_21600 = 0x0100,
    V34_SUPPORT_24000 = 0x0200,
    V34_SUPPORT_26400 = 0x0400,
    V34_SUPPORT_28800 = 0x0800,
    V34_SUPPORT_31200 = 0x1000,
    V34_SUPPORT_33600 = 0x2000
};

enum v34_half_duplex_modes_e
{
    /* Make this the source side modem in the half-duplex exchange */
    V34_HALF_DUPLEX_SOURCE,
    /* Make this the recipient side modem in the half-duplex exchange */
    V34_HALF_DUPLEX_RECIPIENT,
    /* Start control channel operation */
    V34_HALF_DUPLEX_CONTROL_CHANNEL,
    /* Start primary channel operation in the current source/recipient mode */
    V34_HALF_DUPLEX_PRIMARY_CHANNEL,
    /* Stop transmission */
    V34_HALF_DUPLEX_SILENCE
};

/*!
    V.34 modem descriptor. This defines the working state for a single instance
    of a V.34 modem.
*/
typedef struct v34_state_s v34_state_t;

typedef struct
{
    bool support_2743;
    bool support_2800;
    bool support_3429;
    bool support_3000_low;
    bool support_3000_high;
    bool support_3200_low;
    bool support_3200_high;
    bool rate_3429_allowed;
    bool support_power_reduction;
    uint8_t max_baud_rate_difference;
    bool from_cme_modem;
    bool support_1664_point_constellation;
    uint8_t tx_clock_source;
    bool acknowledge_info0d;
    uint8_t raw_26_27;
    uint8_t info0d_nominal_power_code;
    uint8_t info0d_max_power_code;
    bool info0d_power_measured_at_codec_output;
    bool info0d_pcm_alaw;
    bool info0d_upstream_3429_support;
    uint8_t info0d_reserved_41;
    bool info0d_extensions_valid;
} v34_v90_info0a_t;

typedef struct
{
    int md;
    int u_info;
    int upstream_symbol_rate_code;
    int downstream_rate_code;
    int freq_offset;
    uint8_t raw_12_17;
    uint8_t raw_32_33;
    uint16_t raw_40_49;
} v34_v90_info1a_t;

typedef struct
{
    bool use_high_carrier;
    int pre_emphasis;
    int max_bit_rate;
} v34_v90_info1d_baud_rate_parms_t;

typedef struct
{
    int power_reduction;
    int additional_power_reduction;
    int md;
    int freq_offset;
    v34_v90_info1d_baud_rate_parms_t rate_data[6];
} v34_v90_info1d_t;

/*! V.34 Table 20 directional rate offer.  Rates are N*2400 bit/s with
    N in 1..14; mask bit N-1 enables that rate. */
typedef struct
{
    int max_rate_a_to_c;
    int max_rate_c_to_a;
    uint16_t signalling_rate_mask;
    bool asymmetric_rates_allowed;
} v34_mp_rate_offer_t;

#if defined(__cplusplus)
extern "C"
{
#endif

/*! Process a block of received V.34 modem audio samples.
    \brief Process a block of received V.34 modem audio samples.
    \param s The modem context.
    \param amp The audio sample buffer.
    \param len The number of samples in the buffer.
    \return The number of samples unprocessed. */
SPAN_DECLARE(int) v34_rx(v34_state_t *s, const int16_t amp[], int len);

/*! Fake processing of a missing block of received V.34 modem audio samples.
    (e.g due to packet loss).
    \brief Fake processing of a missing block of received V.34 modem audio samples.
    \param s The modem context.
    \param len The number of samples to fake.
    \return The number of samples unprocessed. */
SPAN_DECLARE(int) v34_rx_fillin(v34_state_t *s, int len);

/*! Get a snapshot of the current equalizer coefficients.
    \brief Get a snapshot of the current equalizer coefficients.
    \param coeffs The vector of complex coefficients.
    \return The number of coefficients in the vector. */
#if defined(SPANDSP_USE_FIXED_POINT)
SPAN_DECLARE(int) v34_equalizer_state(v34_state_t *s, complexi16_t **coeffs);
#else
SPAN_DECLARE(int) v34_equalizer_state(v34_state_t *s, complexf_t **coeffs);
#endif

/*! Get the current received carrier frequency.
    \param s The modem context.
    \return The frequency, in Hertz. */
SPAN_DECLARE(float) v34_rx_carrier_frequency(v34_state_t *s);

/*! Get the current symbol timing correction since startup.
    \param s The modem context.
    \return The correction. */
SPAN_DECLARE(float) v34_rx_symbol_timing_correction(v34_state_t *s);

/*! Get a current received signal power.
    \param s The modem context.
    \return The signal power, in dBm0. */
SPAN_DECLARE(float) v34_rx_signal_power(v34_state_t *s);

/*! Set the power level at which the carrier detection will cut in
    \param s The modem context.
    \param cutoff The signal cutoff power, in dBm0. */
SPAN_DECLARE(void) v34_rx_set_signal_cutoff(v34_state_t *s, float cutoff);

/*! Set a handler routine to process QAM status reports
    \param s The modem context.
    \param handler The handler routine.
    \param user_data An opaque pointer passed to the handler routine. */
SPAN_DECLARE(void) v34_set_qam_report_handler(v34_state_t *s, qam_report_handler_t handler, void *user_data);

/*! Generate a block of V.34 modem audio samples.
    \brief Generate a block of V.34 modem audio samples.
    \param s The modem context.
    \param amp The audio sample buffer.
    \param len The number of samples to be generated.
    \return The number of samples actually generated. */
SPAN_DECLARE(int) v34_tx(v34_state_t *s, int16_t amp[], int len);

/*! Adjust a V.34 modem transmit context's power output.
    \brief Adjust a V.34 modem transmit context's output power.
    \param s The modem context.
    \param power The power level, in dBm0 */
SPAN_DECLARE(void) v34_tx_power(v34_state_t *s, float power);

/*! Report the current operating bit rate of a V.34 modem context.
    \brief Report the current operating bit rate of a V.34 modem context
    \param s The modem context.
    \return ??? */
SPAN_DECLARE(int) v34_get_current_bit_rate(v34_state_t *s);

/*! Change the operating mode of a V.34 half-duplex modem.
    \brief Change the operating mode of a V.34 half-duplex modem.
    \param s The modem context.
    \param mode The new mode to be selected.
    \return ??? */
SPAN_DECLARE(int) v34_half_duplex_change_mode(v34_state_t *s, int mode);

/*! Reinitialise an existing V.34 modem context, so it may be reused.
    \brief Reinitialise an existing V.34 modem context.
    \param s The modem context.
    \param baud_rate The baud rate of the modem. Valid values are 2400, 2743, 2800, 3000, 3200 and 3429
    \param bit_rate The bit rate of the modem. Valid values are 4800, 7200, 9600, 12000 and 14400.
    \param duplex True if this is a full duplex mode modem. Otherwise this is a half-duplex modem.
    \return 0 for OK, -1 for bad parameter */
SPAN_DECLARE(int) v34_restart(v34_state_t *s, int baud_rate, int bit_rate, bool duplex);

/*! Initialise a V.34 modem context. This must be called before the first
    use of the context, to initialise its contents.
    \brief Initialise a V.34 modem context.
    \param s The modem context.
    \param baud_rate The baud rate of the modem. Valid values are 2400, 2743, 2800, 3000, 3200 and 3429
    \param bit_rate The bit rate of the modem. Valid values are 4800, 7200, 9600, 12000 and 14400.
    \param calling_party True if this is the calling modem.
    \param duplex True if this is a full duplex mode modem. Otherwise this is a half-duplex modem.
    \param get_bit The callback routine used to get the data to be transmitted.
    \param get_bit_user_data An opaque pointer, passed in calls to the gett routine.
    \param put_bit The callback routine used to get the data to be transmitted.
    \param put_bit_user_data An opaque pointer, passed in calls to the put routine.
    \return A pointer to the modem context, or NULL if there was a problem. */
SPAN_DECLARE(v34_state_t *) v34_init(v34_state_t *s,
                                     int baud_rate,
                                     int bit_rate,
                                     bool calling_party,
                                     bool duplex,
                                     span_get_bit_func_t get_bit,
                                     void *get_bit_user_data,
                                     span_put_bit_func_t put_bit,
                                     void *put_bit_user_data);

/*! Release a V.34 modem receive context.
    \brief Release a V.34 modem receive context.
    \param s The modem context.
    \return 0 for OK */
SPAN_DECLARE(int) v34_release(v34_state_t *s);

/*! Free a V.34 modem receive context.
    \brief Free a V.34 modem receive context.
    \param s The modem context.
    \return 0 for OK */
SPAN_DECLARE(int) v34_free(v34_state_t *s);

/*! Get the logging context associated with a V.34 modem context.
    \brief Get the logging context associated with a V.34 modem context.
    \param s The modem context.
    \return A pointer to the logging context */
SPAN_DECLARE(logging_state_t *) v34_get_logging_state(v34_state_t *s);

/*! Change the get_bit function associated with a V.34 modem context.
    \brief Change the get_bit function associated with a V.34 modem context.
    \param s The modem context.
    \param get_bit The callback routine used to get the data to be transmitted.
    \param user_data An opaque pointer. */
SPAN_DECLARE(void) v34_set_get_bit(v34_state_t *s, span_get_bit_func_t get_bit, void *user_data);

/*! Change the get_aux_bit function associated with a V.34 modem context.
    \brief Change the get_aux_bit function associated with a V.34 modem context.
    \param s The modem context.
    \param get_bit The callback routine used to get the aux. data to be transmitted.
    \param user_data An opaque pointer. */
SPAN_DECLARE(void) v34_set_get_aux_bit(v34_state_t *s, span_get_bit_func_t get_bit, void *user_data);

/*! Change the put_bit function associated with a V.34 modem context.
    \brief Change the put_bit function associated with a V.34 modem context.
    \param s The modem context.
    \param put_bit The callback routine used to process the data received.
    \param user_data An opaque pointer. */
SPAN_DECLARE(void) v34_set_put_bit(v34_state_t *s, span_put_bit_func_t put_bit, void *user_data);

/*! Change the put_aux_bit function associated with a V.34 modem context.
    \brief Change the put_aux_bit function associated with a V.34 modem context.
    \param s The modem context.
    \param put_bit The callback routine used to process the aux data received.
    \param user_data An opaque pointer. */
SPAN_DECLARE(void) v34_set_put_aux_bit(v34_state_t *s, span_put_bit_func_t put_bit, void *user_data);

/*! Set a callback for descrambled V.90 analogue-side Phase 4 CP bits.
    This callback is active only for a V.90 digital answerer. */
SPAN_DECLARE(void) v34_set_put_phase4_bit(v34_state_t *s,
                                          span_put_bit_func_t put_bit,
                                          void *user_data);

/*! Reject only the current V.90 Phase 4 CP decode hypothesis and resume
    preamble acquisition without restarting carrier/equalizer training. */
SPAN_DECLARE(void) v34_reject_v90_phase4_hypothesis(v34_state_t *s);

/*! Check if the V.34 modem has entered the primary channel phase (Phase 3/4
    or data mode).  During Phase 2, both sides share the same 1200 Hz carrier,
    so an external echo canceller would cancel the far-end signal.  After this
    returns true, TX and RX use separated carriers and echo cancellation is safe.
    \param s The modem context.
    \return true when the primary channel is active. */
SPAN_DECLARE(bool) v34_get_primary_channel_active(v34_state_t *s);

/*! Get the current RX stage of the V.34 state machine.
    \param s The modem context.
    \return The current RX stage (v34_rx_stages_e value). */
SPAN_DECLARE(int) v34_get_rx_stage(v34_state_t *s);

/*! Level of the far end's 1800 Hz guard tone relative to its 2400 Hz carrier,
    in dB (V.34 10.1.2.1, 10.1.2.3).  About +1 dB while the peer holds Tone A,
    about -6 dB while it transmits an INFO sequence, so the ratio distinguishes
    the two states.  \param valid set to 0 when neither bin has usable signal.
*/
SPAN_DECLARE(float) v34_get_guard_carrier_db(v34_state_t *s, int *valid);

/*! Get the current RX symbol-rate code (v34_baud_rate_e, 0=2400..5=3429).
    After v34_v90_prepare_upstream_data() this is the negotiated V.90
    upstream data baud (3000 or 3200 per V.90 §6.2), not the CP
    control-channel rate.  Primarily a regression-test hook.
    \param s The modem context.
    \return The RX baud-rate code, or -1 on error. */
SPAN_DECLARE(int) v34_get_rx_baud_rate(v34_state_t *s);

/*! Get the current RX carrier selection.
    \param s The modem context.
    \return Non-zero for the high carrier, zero for low, or -1 on error. */
SPAN_DECLARE(int) v34_get_rx_high_carrier(v34_state_t *s);

/*! Get the current TX symbol-rate code (v34_baud_rate_e, 0=2400..5=3429).
    \param s The modem context.
    \return The TX baud-rate code, or -1 on error. */
SPAN_DECLARE(int) v34_get_tx_baud_rate(v34_state_t *s);

/*! Get the current TX carrier selection.
    \param s The modem context.
    \return Non-zero for the high carrier, zero for low, or -1 on error. */
SPAN_DECLARE(int) v34_get_tx_high_carrier(v34_state_t *s);

/*! Get the Phase 2 round-trip delay estimate.
    \param s The modem context.
    \return The estimate in 8000 Hz sample times, or 0 if unavailable. */
SPAN_DECLARE(int) v34_get_round_trip_delay_samples(v34_state_t *s);

/*! Get the effective U_INFO selected for a locally transmitted V.90 INFO1a.
    This can be lower than the configured preference when INFO0d's maximum
    digital transmit power requires it.
    \param s The modem context.
    \return U_INFO (67..127), or 0 when not in the analogue V.90 role. */
SPAN_DECLARE(int) v34_get_v90_tx_u_info(v34_state_t *s);

/*! Get the current TX stage of the V.34 state machine.
    \param s The modem context.
    \return The current TX stage (v34_tx_stages_e value). */
SPAN_DECLARE(int) v34_get_tx_stage(v34_state_t *s);

/*! Get the U_INFO value from the received INFO1a (V.90 mode).
    \param s The modem context.
    \return The U_INFO Ucode (0-127), or 0 if not yet received. */
SPAN_DECLARE(int) v34_get_v90_u_info(v34_state_t *s);

/*! Get the most recently decoded analogue-side INFO0a fields in V.90 mode.
    \param s The modem context.
    \param info Output frame fields.
    \return 1 if a valid INFO0a has been received, otherwise 0. */
SPAN_DECLARE(int) v34_get_v90_received_info0a(v34_state_t *s, v34_v90_info0a_t *info);

/*! Get the most recently decoded analogue-side INFO1a fields in V.90 mode.
    \param s The modem context.
    \param info Output frame fields.
    \return 1 if a valid INFO1a has been received, otherwise 0. */
SPAN_DECLARE(int) v34_get_v90_received_info1a(v34_state_t *s, v34_v90_info1a_t *info);

/*! Get the most recently decoded digital-side INFO1d fields in V.90 mode.
    \param s The modem context.
    \param info Output frame fields.
    \return 1 if a valid INFO1d has been received, otherwise 0. */
SPAN_DECLARE(int) v34_get_v90_received_info1d(v34_state_t *s, v34_v90_info1d_t *info);

/*! Get the most recent V.34/V.90 RX event code.
    \param s The modem context.
    \return The current rx.received_event value (v34_events_e). */
SPAN_DECLARE(int) v34_get_rx_event(v34_state_t *s);

/*! Clear a reported V.90 peer-retrain event after the external digital-side
    state machine has consumed or rejected it.  Other RX events are left
    untouched.
    \param s The modem context. */
SPAN_DECLARE(void) v34_v90_clear_peer_retrain_event(v34_state_t *s);

/*! Answer a peer-initiated V.90 retrain per §9.5.1.2: 70 ± 5 ms of silence,
    then Tone B with the receiver conditioned for the Tone A phase reversal
    (§9.2.1.1.3).  Call after v34_restart()/v34_set_v90_mode() so the retrained
    Phase 2 skips the INFO0 exchange, which §9.5 omits.  No-op unless the
    context is a V.90 digital answerer.
    \param s The modem context. */
SPAN_DECLARE(void) v34_v90_start_retrain_response(v34_state_t *s);

/*! \brief Start a V.34 11.5 retrain: 70 ms of silence, then this role's tone
           (Tone A for the answer modem, Tone B for the call modem), with the
           INFO0 exchange omitted.  Use INSTEAD of restarting into Phase 2 at
           INFO0, which puts a modulated carrier in front of a peer that is
           waiting for a tone.
    \param s The V.34 context. */
SPAN_DECLARE(void) v34_start_retrain(v34_state_t *s);

/*! \brief Has the plain V.34 data mode stopped decoding?  V.34 11.5/11.6
           give a receiver in that state a retrain and a rate renegotiation;
           this is what says it is in it.
    \param s The V.34 context.
    \return non-zero if the received symbols have been off the constellation
            for long enough to be a failure rather than a burst. */
SPAN_DECLARE(int) v34_data_carrier_lost(v34_state_t *s);

/*! \brief Clear a reported peer retrain, role-independently.  The event is
           application-owned; left set it suppresses the ordinary handshake
           events for the rest of the call.
    \param s The V.34 context. */
SPAN_DECLARE(void) v34_clear_peer_retrain_event(v34_state_t *s);

/*! \brief Clear a reported peer rate renegotiation (V.90 9.6.2 / V.34 11.6)
           and re-arm the detector.
    \param s The V.34 context. */
SPAN_DECLARE(void) v34_clear_peer_reneg_s_event(v34_state_t *s);

/*! \brief Start a V.34 11.6 rate renegotiation from data mode.  The spec
           offers this as the way to "resynchronize the receiver without going
           through a complete retrain": it re-runs Phase 4's S, S-bar, TRN, MP,
           MP', E and B1 without returning to Phase 2.  Both roles run the same
           sequence; 11.6.1.1 and 11.6.1.2 differ only in who sends S first,
           and a responder calls this having already detected the initiator's.
    \param s The V.34 context.
    \return 0 on success, -1 if the context is not in data mode. */
SPAN_DECLARE(int) v34_start_rate_renegotiation(v34_state_t *s);

/*! \brief Is a V.34 11.6 rate renegotiation in progress?
    \param s The V.34 context.
    \return non-zero while one is running. */
SPAN_DECLARE(int) v34_rate_renegotiation_active(v34_state_t *s);

/*! \brief Note that a recovery has been started for the current loss, so the
           same one does not start another.
    \param s The V.34 context. */
SPAN_DECLARE(void) v34_clear_data_carrier_lost(v34_state_t *s);

/*! Start the analogue-modem side of V.90 §9.5.2 after v34_restart(): 70 ± 5 ms
    silence, Tone A, Tone B detection/reversals, then §9.2.2.1.4.  This is used
    for both initiating (§9.5.2.1) and responding (§9.5.2.2), since the latter
    has already observed the required 50 ms of Tone B.
    \param s The V.90 analogue/calling modem context. */
SPAN_DECLARE(void) v34_v90_start_analogue_retrain(v34_state_t *s);

/*! Get the number of distinct, strictly detected Phase 3 S transitions.
    \param s The modem context.
    \return A monotonically increasing count for the current training attempt. */
SPAN_DECLARE(int) v34_get_phase3_s_event_count(v34_state_t *s);

/* Keep the V.34 upstream receiver in the V.90 Phase 3 S-detection path after
   the digital-side application has consumed analogue Ja.  The V.90 digital
   transmitter is external to SpanDSP, so the ordinary V.34 transmitter must
   not consume Ja by advancing both halves of the modem into Phase 4. */
SPAN_DECLARE(void) v34_v90_arm_phase3_s_detector(v34_state_t *s);

/*! Tell the receiver whether the analogue modem is currently required to be
    silent (true while we transmit Jd, per V.90 §9.3.2.4/§9.3.2.7; false during
    DIL, where §9.3.2.9 permits SCR).  While set, sustained energy that does not
    resolve into S is reported as a peer retrain.
    \param s The V.34 context.
    \param expect Non-zero while the far end must be silent. */
SPAN_DECLARE(void) v34_v90_set_phase3_expect_silence(v34_state_t *s, int expect);

/*! Copy one continuously decoded Phase 3 Ja hypothesis as unpacked bits. */
SPAN_DECLARE(int) v34_v90_copy_phase3_ja_bits(v34_state_t *s,
                                               int hypothesis,
                                               uint8_t bits[],
                                               int max_bits);

/*! Copy one Phase 3 Ja hypothesis before descrambling, as unpacked bits. */
SPAN_DECLARE(int) v34_v90_copy_phase3_ja_raw_bits(v34_state_t *s,
                                                   int hypothesis,
                                                   uint8_t bits[],
                                                   int max_bits);

/*! Get the number of Phase 3 J-detector bits accumulated so far.
    \param s The modem context.
    \return The current rx.phase3_j_bits counter. */
SPAN_DECLARE(int) v34_get_phase3_j_bits(v34_state_t *s);

/*! Get the current Phase 3 far-end J TRN mode hint.
    \param s The modem context.
    \return -1 if unknown, 0 for 4-point, 1 for 16-point. */
SPAN_DECLARE(int) v34_get_phase3_j_trn16(v34_state_t *s);

/*! Get the current Phase 3 TRN lock score hint.
    \param s The modem context.
    \return The current rx.phase3_trn_lock_score percentage, or -1 if unknown. */
SPAN_DECLARE(int) v34_get_phase3_trn_lock_score(v34_state_t *s);

/*! Get whether the transmitter has switched into data mode.
    \param s The modem context.
    \return Non-zero once tx_data_mode is active. */
SPAN_DECLARE(int) v34_get_tx_data_mode(v34_state_t *s);

/*! Override the per-direction MP signalling-rate advertisement used during
    Phase 4. Rates use the V.34 MP N coding, where rate = N * 2400 bps and
    N must be between 1 and 14 inclusive.
    \param s The modem context.
    \param bit_rate_a_to_c Maximum answerer-to-caller signalling rate N.
    \param bit_rate_c_to_a Maximum caller-to-answerer signalling rate N. */
SPAN_DECLARE(void) v34_set_mp_rate_policy(v34_state_t *s, int bit_rate_a_to_c, int bit_rate_c_to_a);

/*! Clear any explicit MP signalling-rate override so Phase 4 falls back to the
    modem's locally-derived defaults.
    \param s The modem context. */
SPAN_DECLARE(void) v34_clear_mp_rate_policy(v34_state_t *s);

/*! Resolve two V.34 MP rate offers according to 11.4.1.1.4/.2.4 and
    Table 20 bit 50.
    \return 0 on success, or -1 when no mutually enabled rate exists. */
SPAN_DECLARE(int) v34_negotiate_mp_rates(const v34_mp_rate_offer_t *local,
                                         const v34_mp_rate_offer_t *remote,
                                         int *rate_a_to_c,
                                         int *rate_c_to_a);

/*! Force the transmitter/receiver pair into Phase 4 startup. Intended for
    external V.90 downstream implementations which handle Phase 3 themselves
    and then hand control back to SpanDSP for native V.34/V.90 Phase 4. */
SPAN_DECLARE(void) v34_force_phase4(v34_state_t *s);

/*! Start immediate V.90 CPt acquisition on the primary-channel receiver.
    This is the digital-answerer companion to v34_force_phase4() when an
    external V.90 implementation owns Ri and the project-owned CP framer owns
    the variable-length Table 14 message. The receiver enters its dedicated
    V90_CP stage; ordinary V.34 MP frame lengths, timeouts and E handling do
    not run there. */
SPAN_DECLARE(void) v34_force_v90_phase4_cp_rx(v34_state_t *s);

/*! Force the primary-channel receiver into Phase 3 PP/TRN conditioning.
    Intended for offline V.90 replay after INFO1 has already been decoded;
    the trained equalizer is preserved by a later v34_force_phase4(). */
SPAN_DECLARE(void) v34_force_phase3_rx(v34_state_t *s);

/*! Seed the receive-side Phase 4 MP parameters when an external/offline
    detector has already validated the answer modem's MP frame.  The rate is
    the V.34 MP N value (rate = N * 2400 bit/s), and precoder_coeffs contains
    h(1..3) as interleaved Q2.14 real/imaginary values.  Passing NULL selects
    zero coefficients (MP0).
    \return 0 on success, or -1 for invalid parameters. */
SPAN_DECLARE(int) v34_seed_rx_mp(v34_state_t *s,
                                 int bit_rate_n,
                                 int trellis_size,
                                 int use_non_linear_encoder,
                                 int expanded_shaping,
                                 const int16_t precoder_coeffs[6]);

/*! Seed a transmit data mapper at the reset state used for B1.  This is the
    transmit-side counterpart of v34_seed_rx_mp() and is primarily useful to
    build deterministic B1 reference mapping frames for offline acquisition. */
SPAN_DECLARE(int) v34_seed_tx_data(v34_state_t *s,
                                   int bit_rate_n,
                                   int trellis_size,
                                   int use_non_linear_encoder,
                                   int expanded_shaping,
                                   const int16_t precoder_coeffs[6]);

/*! Hand an externally sequenced V.90 analogue transmitter from E to B1/data.
    Seeds the V.34 data mapper from the validated V.90 MP, emits B1 as the
    first all-ones data frame (V.90 §8.5.1/§9.4.2.5), then takes bits from the
    modem's normal get-bit callback.  The existing carrier/modulator phase is
    preserved across the handover.
    \return 0 on success, or -1 for invalid modulation parameters. */
SPAN_DECLARE(int) v34_v90_begin_tx_data(v34_state_t *s,
                                        int bit_rate_n,
                                        int trellis_size,
                                        int use_non_linear_encoder,
                                        int expanded_shaping,
                                        const int16_t precoder_coeffs[6]);

/*! Produce one complete V.34 mapping frame (eight Q9.7 complex symbols). */
SPAN_DECLARE(int) v34_get_mapping_frame_state(v34_state_t *s,
                                              int16_t bits[16]);

/*! Set the final complex transform between the trained primary-channel
    equalizer and the V.34 data constellation slicer.  Rotation is 0..3 in
    counter-clockwise quarter turns. */
SPAN_DECLARE(int) v34_set_rx_data_transform(v34_state_t *s,
                                            float scale,
                                            int rotation,
                                            int conjugate);

/*! Enter receive DATA mode after externally validated E/B1 timing. */
SPAN_DECLARE(int) v34_begin_rx_data(v34_state_t *s);

/*! Configure the receive data parameters (rate/trellis/parms) for the V.90
    answerer's upstream V.34 data channel WITHOUT touching the Phase 4
    stage, hypothesis lock or scrambler state.  Unlike v34_seed_rx_mp(),
    this is safe to call while the live CP bit tap is still running: the
    engine detects the analogue modem's E itself and then calls
    v34_begin_rx_data().  baud_rate is the negotiated V.34 symbol-rate code;
    V.90 §6.2 requires digital-modem support for codes 3 (3000) and 4 (3200).
    high_carrier is the corresponding INFO1d row's carrier selection. bit_rate
    is in bps and must be legal for that symbol rate. */
SPAN_DECLARE(int) v34_v90_prepare_upstream_data(v34_state_t *s,
                                                int baud_rate,
                                                int high_carrier,
                                                int bit_rate,
                                                int trellis_size);

/*! Report whether the V.90 upstream receiver has acquired its T/3
    timing/equalizer solution from the known B1 frame (9 kHz at 3000 baud,
    9.6 kHz at 3200 baud). */
SPAN_DECLARE(int) v34_v90_upstream_rx_acquired(v34_state_t *s);

/*! Has the V.90 upstream lost carrier -- the symbols at the white level for
    long enough that only a re-acquisition will bring them back?  V.90 9.6
    rate renegotiation is that re-acquisition: it ends in a fresh B1, which is
    what this receiver acquires against.
    \param s The V.34 context.
    \return non-zero if the upstream has lost carrier. */
SPAN_DECLARE(int) v34_v90_upstream_carrier_lost(v34_state_t *s);

/*! Note that recovery has been started for the current loss, so it is not
    started twice.
    \param s The V.34 context. */
SPAN_DECLARE(void) v34_v90_upstream_clear_carrier_lost(v34_state_t *s);

/*! Read internal V.90 upstream resampler accounting.  Both counts are DSP-side
    diagnostics; the external bearer remains an 8 kHz stream. */
SPAN_DECLARE(void) v34_v90_upstream_sample_counts(v34_state_t *s,
                                                  int64_t *input_8k,
                                                  int64_t *output_t3);

/*! Set U_INFO for a V.90 analogue-role INFO1a — Table 11 bits 25:31, the Ucode
    the digital modem uses for Sd and TRN1d (§8.4.4, §8.4.5).  The analogue
    modem chooses it, so its receiver must be told the same value.  Call before
    Phase 2.  0 restores the default.
    \param s The modem context.
    \param u_info Ucode, 0 to 127. */
SPAN_DECLARE(void) v34_set_v90_u_info(v34_state_t *s, int u_info);

/*! An external per-baud symbol source for the V.34 modulator.  The symbol is
    returned in normalised units: 1.0 is one constellation step, so the 4-point
    training constellation is (±0.7071068, ±0.7071068).  Nominal symbol RMS,
    pulse shaping, carrier and gain are applied by the modulator. */
typedef void (*v34_tx_external_symbol_func_t)(void *user_data, float *re, float *im);

/*! Drive the V.34 modulator from an external symbol source, bypassing the
    Phase 2/3/4 transmit state machine entirely.

    This exists for the V.90 analogue role.  The analogue modem's Phase 3
    signals (S, S̄, PP, TRN, Ja, SCR) are V.34-modulated, but their sequencing
    is governed by events in the *PCM* downstream — the Sd-to-S̄d transition,
    Jd, J'd, DIL (§9.3.2) — which this module never sees, because that
    direction is not V.34 at all.  So only the modulator is reused here; the
    state machine stays with the caller, and the receiver is untouched.

    \param s The modem context.
    \param baud_rate Symbol rate as a code from 0 (2400) to 5 (3429).
    \param high_carrier True to use the higher of the two carrier options.
    \param fn Symbol source, called once per baud.
    \param user_data Opaque pointer passed to fn.
    \return 0 on success, -1 on a bad baud rate or a NULL source. */
SPAN_DECLARE(int) v34_tx_start_external_symbols(v34_state_t *s,
                                                int baud_rate,
                                                int high_carrier,
                                                v34_tx_external_symbol_func_t fn,
                                                void *user_data);

/*! Replace the active data mapper with an external symbol source without
    resetting carrier phase, baud timing, or pulse-shaper history.  Used at
    V.90 §9.6's data-frame-aligned rate-renegotiation seam. */
SPAN_DECLARE(int) v34_v90_resume_external_symbols(v34_state_t *s,
                                                  v34_tx_external_symbol_func_t fn,
                                                  void *user_data);

/*! Stop an external symbol source and leave the transmitter silent.
    \param s The modem context. */
SPAN_DECLARE(void) v34_tx_stop_external_symbols(v34_state_t *s);

/*! Decode one complete V.34 mapping frame (eight Q9.7 complex symbols). */
SPAN_DECLARE(void) v34_put_mapping_frame_state(v34_state_t *s,
                                               int16_t bits[16]);

/*! Enable V.90 mode on a V.34 modem context.  When enabled, the digital
    modem sends INFO0d (V.90 Table 7) instead of standard V.34 INFO0 during
    Phase 2, and expects INFO0a (V.90 Table 8) from the analog modem.
    Must be called after v34_init() and before training begins.
    \param s The modem context.
    \param pcm_law PCM coding: 0 = µ-law, 1 = A-law. */
SPAN_DECLARE(void) v34_set_v90_mode(v34_state_t *s, int pcm_law);

/*! Configure the V.92 capability bits carried in a V.90-format INFO0d.
    V.92 Table 15 assigns bit 26 to short-Phase-2 request and bit 27 to
    V.92 capability.  This only changes the transmitted INFO0d; callers must
    still confirm the peer's INFO0a capability before selecting V.92. */
SPAN_DECLARE(void) v34_set_v92_info0_capabilities(v34_state_t *s,
                                                   int v92_capable,
                                                   int short_phase2_requested);

/*! Advertise PCM upstream support in V.92 Table 17 INFO1d bit 70.
    PCM upstream is V.92's only data-pump feature over V.90, so an analogue
    peer that sees a zero here selects V.90 even when both INFO0 capability
    bits agree.  Defaults to off: the upstream data path is still V.34, so
    setting this claims a data-mode receiver that does not exist.  Enable it
    to drive a peer into the V.92 Phase 3/4 upstream procedures. */
SPAN_DECLARE(void) v34_set_v92_pcm_upstream_capability(v34_state_t *s,
                                                        int pcm_upstream_capable);

#if defined(__cplusplus)
}
#endif

#endif
/*- End of file ------------------------------------------------------------*/
