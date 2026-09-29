/*
 * SpanDSP - a series of DSP components for telephony
 *
 * private/v8.h - V.8 modem negotiation processing.
 *
 * Written by Steve Underwood <steveu@coppice.org>
 *
 * Copyright (C) 2004 Steve Underwood
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

#if !defined(_SPANDSP_PRIVATE_V8_H_)
#define _SPANDSP_PRIVATE_V8_H_

struct v8_state_s
{
    /*! \brief True if we are the calling party */
    bool calling_party;

    /*! \brief A handler to process the V.8 signals */
    v8_result_handler_t result_handler;
    /*! \brief An opaque pointer passed to result_handler */
    void *result_handler_user_data;

    /*! \brief The current state of the V.8 protocol */
    int state;
    bool fsk_tx_on;
    /*! \brief Silence held before the answerer starts ANSam, so a leading
               V.25 calling tone (1300 Hz, 0.5-0.7s ON) from the calling
               modem has room to finish before our own tone starts
               overlapping it (no echo cancellation on analogue-modem
               calls means our echoed ANSam can otherwise pollute the
               calling tone's detection window on both ends). */
    span_sample_timer_t ansam_start_delay_timer;
    span_sample_timer_t modem_connect_tone_tx_timer;
    span_sample_timer_t negotiation_timer;
    span_sample_timer_t ci_timer;
    int ci_repetition_count;
    /* Calling role: samples since CM (re)started, and the silent gap before
       a CM restart (see V8_CM_ON). */
    int cm_on_samples;
    int cm_gap_samples;
    bool proceed;
    fsk_tx_state_t v21tx;
    /* Transmit level for the V.21 CI/CM/JM/CJ signalling, dBm0.  Both
       preset_fsk_specs V.21 channels declare -14; V.2 permits more, and this
       tree already boosts V.34 Phase 2 from -14 to -10 because a peer was not
       detecting it (modem_engine.c).  Settable so the same can be measured for
       V.8, whose CM a marginal answering detector may simply not hear. */
    float v21_tx_power;
    fsk_rx_state_t v21rx;
    queue_state_t *tx_queue;
    modem_connect_tones_tx_state_t ansam_tx;
    modem_connect_tones_rx_state_t ansam_rx;
    modem_connect_tones_rx_state_t calling_tone_rx;
    modem_connect_tones_rx_state_t cng_tone_rx;

    v8_parms_t parms;
    v8_parms_t result;

    /*! \brief The number of modulation bytes to use when sending. */
    int modulation_bytes;

    /* V.8 data parsing */
    uint32_t bit_stream;
    int bit_cnt;
    /* Indicates the type of message coming up */
    int preamble_type;
    uint8_t rx_data[64];
    int rx_data_ptr;

    /*! \brief a reference copy of the last CM or JM message, used when
               testing for matches. */
    uint8_t cm_jm_data[64];
    int cm_jm_len;
    bool got_ci;
    bool got_cm_jm;
    bool got_cj;
    int zero_byte_count;
    /*! \brief Error and flow logging control */
    logging_state_t logging;
};

#endif
/*- End of file ------------------------------------------------------------*/
