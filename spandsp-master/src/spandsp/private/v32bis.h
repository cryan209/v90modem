/*
 * SpanDSP - a series of DSP components for telephony
 *
 * private/v32bis.h - ITU V.32bis modem
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

#if !defined(_SPANDSP_PRIVATE_V32BIS_H_)
#define _SPANDSP_PRIVATE_V32BIS_H_

extern const complexf_t v32bis_constellation[16];

/*! The tone detector window, in samples.  20 samples is 6 symbol intervals at
    2400 baud and 8000 samples/s, which is short enough to time a reversal to
    within a symbol or two and long enough to be coherent. */
#define V32BIS_TONE_WINDOW  20

/*!
    V.32bis modem descriptor. This defines the working state for a single instance
    of a V.32bis modem.
*/
/*!
    One coherent sliding-window tone detector.  Two disjoint windows of the
    mixed signal are compared, so a 180 degree phase reversal shows as their
    dot product going negative, and the instant of the reversal is recovered
    from where the leading window's magnitude dipped.
*/
typedef struct
{
    float dphase;
    float phase;
    complexf_t ring[2*V32BIS_TONE_WINDOW];
    float mag_hist[2*V32BIS_TONE_WINDOW];
    int32_t pos;
    complexf_t s1;
    complexf_t s2;
    float mag;
    float peak;
    int32_t hold_until;
    bool reversal;
    int32_t reversal_sample;
} v32bis_tone_det_t;

struct v32bis_state_s
{
    /*! \brief The bit rate of the modem. Valid values are 1200 and 2400. */
    int bit_rate;
    /*! \brief True is this is the calling side modem. */
    bool calling_party;

    v17_rx_state_t rx;
    v17_tx_state_t tx;
    modem_echo_can_segment_state_t *ec;

    uint16_t permitted_rates_signal;

    /* One §5.2 conditioning sequence, two identical Table 5 words and one
       Table 6 word.  Later reactive phases refill this buffer rather than
       duplicating the V.17 pulse shaper. */
    uint8_t startup_tx_symbols[256 + 16 + 1280 + 16 + 8];
    int startup_tx_symbol_count;
    int startup_tx_symbol_pos;
    int startup_rx_symbol_count;

    int startup_rx_stage;
    complexf_t startup_rx_acq[64];
    int startup_rx_acq_count;
    complexf_t startup_rx_gain;
    int startup_rx_sbar_run;
    int startup_rx_sbar_remaining;
    uint32_t startup_rx_trn_reg;
    int startup_rx_trn_pos;
    int startup_rx_trn_diff;
    uint8_t startup_rx_word_states[8];
    int startup_rx_word_pos;
    uint16_t startup_rx_first_r;
    int startup_rx_b1_pos;
    uint32_t startup_rx_b1_reg;
    int startup_rx_b1_diff;
    int startup_rx_b1_convolution;
    /* ITU-T V.32bis 6.  The reactive start-up machine.  v32bis_prepare_startup_tx()
       still queues one self-contained burst for the offline harnesses; when
       v32bis_start_startup() is used instead, these drive the clause 6
       call/answer exchange, and each transmit phase is generated only when the
       events it waits on have arrived. */
    /* ITU-T V.32bis 6.1/6.2 tone phases.  One sliding-window coherent
       detector per tone the role has to watch: 600 and 3000 Hz for the call
       modem, 1800 Hz for the answer modem. */
    v32bis_tone_det_t tone[3];
    bool tone_phase_active;
    int tone_which;
    int tone_present_run;
    int tone_drop_run;
    int reversals_seen;
    int32_t tx_symbol_index;
    int32_t tx_transition_at;
    int32_t tx_phase_start_symbol;
    int32_t tone_counter_start;
    int32_t tone_transition_symbol;
    int32_t rx_sample_count;

    bool reactive_startup;
    int tx_phase;
    int tx_step;
    uint16_t tx_word;
    uint32_t tx_trn_reg;
    int tx_trn_diff;
    int nt_symbols;
    int mt_symbols;
    int rx_hold_symbols;
    uint16_t rx_repeat_word;
    bool rx_s_event_sent;
    bool tx_released;

    int startup_remote_rates;
    int startup_selected_rate;
    bool startup_complete;

    /*! \brief Error and flow logging control */
    logging_state_t logging;
};

#endif
/*- End of file ------------------------------------------------------------*/
