/*
 * SpanDSP - a series of DSP components for telephony
 *
 * private/v34.h - ITU V.34 modem
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

#if !defined(_SPANDSP_PRIVATE_V34_H_)
#define _SPANDSP_PRIVATE_V34_H_

/* Self-contained (plain floats, no spandsp types), so including it here
   costs nothing to the other users of this header. */
#include "v34_gardner.h"

/*! The number of taps in the info data transmit pulse shaping filter */
#define V34_INFO_TX_FILTER_STEPS            9
#define V34_TX_FILTER_STEPS                 9

#define V34_RX_FILTER_STEPS                 27
#define V34_RX_PULSESHAPER_COEFF_SETS       192
#define V34_RX_CC_PULSESHAPER_COEFF_SETS    12

#define V34_EQUALIZER_PRE_LEN               63
#define V34_EQUALIZER_POST_LEN              63
#define V34_EQUALIZER_MASK                  127

/* V.90 upstream DATA receiver.  The bearer remains 8 kHz; only this private
   receive branch runs at 9.6 kHz so 3200 baud is exactly three samples/T. */
#define V34_V90_T3_HILBERT_TAPS             63
#define V34_V90_T3_RESAMPLE_TAPS            33
#define V34_V90_T3_RRC_TAPS                 97
/* T/3-spaced fractionally spaced equalizer.  Seven taps (a bit over two
   symbols) left the B1 template's (39,5) arriving as (33.9,4.3) -- errors of
   four to five units against a constellation spacing of two, which is
   hopeless for 31200 and is why the upstream decoded to white.  A 98.6%
   *energy* fit sounds close and is not: 1.4% of a symbol power of 700 is
   about 3.2 rms.  21 taps spans seven symbols and is still well determined
   by B1's 128 symbols (42 unknowns, 256 equations). */
#define V34_V90_T3_FSE_TAPS                 21
/* Big enough to hold the whole E->B1 era plus the lag of the CP-bitstream E
   detector that anchors the search (about 3.4 s at 9.6 kHz).  B1 is only
   90 ms long and had been passing before the capture even started. */
#define V34_V90_T3_RAW_SIZE                 131072
#define V34_V90_T3_RAW_MASK                 (V34_V90_T3_RAW_SIZE - 1)
#define V34_V90_T3_B1_MAX_SYMBOLS           256
#define V34_V90_T3_GAIN_TRIALS              33
/* The widest superframe (j) any V.34 symbol rate uses, so the upstream
   phase search knows how many candidates it can ever have to try. */
#define V34_MAX_SUPER_FRAME_PHASES          8
/* Mean square distance to the constellation above which the upstream timing
   loop holds still: its detector needs symbols that mean something, and 2/3
   is what symbols unrelated to the lattice give. */
#define V34_V90_T3_TIMING_TRACK_ERR         0.35f

/*! The offset between x index values, and what they mean in terms of the V.34
    spec numbering */
#define V34_XOFF                            3

#define V34_RX_PULSESHAPER_GAIN             1.000000f

extern const complexf_t v34_constellation[16];

#if defined(SPANDSP_USE_FIXED_POINT)
typedef int16_t v34_rx_shaper_t[V34_RX_PULSESHAPER_COEFF_SETS][V34_RX_FILTER_STEPS];
typedef int16_t cc_rx_shaper_t[V34_RX_CC_PULSESHAPER_COEFF_SETS][V34_RX_FILTER_STEPS];
#else
typedef float v34_rx_shaper_t[V34_RX_PULSESHAPER_COEFF_SETS][V34_RX_FILTER_STEPS];
typedef float cc_rx_shaper_t[V34_RX_CC_PULSESHAPER_COEFF_SETS][V34_RX_FILTER_STEPS];
#endif

/* Generated V.34 tables currently vary in first/second dimension sizes
 * (e.g. 16/32/64 encode states and 4 decode entries), so keep these
 * typedefs dimension-flexible for assignment compatibility. */
typedef const uint8_t conv_encode_table_t[][16];
typedef const uint8_t conv_decode_table_t[][4];

enum
{
    V34_MODULATION_V34 = 0,
    V34_MODULATION_CC,
    V34_MODULATION_TONES,
    V34_MODULATION_L1_L2,
    V34_MODULATION_PCM_L1_L2,
    V34_MODULATION_SILENCE
};

enum v34_rx_stages_e
{
    V34_RX_STAGE_INFO0 = 1,
    V34_RX_STAGE_INFOH,
    V34_RX_STAGE_INFO1C,
    V34_RX_STAGE_INFO1A,
    V34_RX_STAGE_TONE_A,
    V34_RX_STAGE_TONE_B,
    V34_RX_STAGE_L1_L2,
    V34_RX_STAGE_CC,
    V34_RX_STAGE_PRIMARY_CHANNEL,
    V34_RX_STAGE_PHASE3_WAIT_S,
    V34_RX_STAGE_PHASE3_TRAINING,
    V34_RX_STAGE_PHASE3_DONE,
    /*! \brief Phase 4: detecting far-end S signal (180° phase reversals) */
    V34_RX_STAGE_PHASE4_S,
    /*! \brief Phase 4: detected S, waiting for S-bar transition (90° offset) */
    V34_RX_STAGE_PHASE4_S_BAR,
    /*! \brief Phase 4: TRN synchronization after S-bar */
    V34_RX_STAGE_PHASE4_TRN,
    /*! \brief Phase 4: detecting MP on primary channel (DQPSK demod) */
    V34_RX_STAGE_PHASE4_MP,
    /*! \brief Data mode: full constellation decode (Viterbi, shell unmap, etc.) */
    V34_RX_STAGE_DATA,
    /*! \brief V.90 Phase 4: receiving analogue-modem CP on the V.34 upstream.
               Kept after DATA so the pre-existing stage values remain stable. */
    V34_RX_STAGE_V90_CP = 18
};

enum v34_tx_stages_e
{
    /*! \brief An initial bit of extra preamble ahead of the first INFO0, to ensure
               bit synchronisation is OK by the first bit of INFO0 */
    V34_TX_STAGE_INITIAL_PREAMBLE = 1,
    /*! \brief INFO0 is being transmitted the first time */
    V34_TX_STAGE_INFO0,
    /*! \brief Transmitting A while waiting for 50ms timeout */
    V34_TX_STAGE_INITIAL_A,
    /*! \brief Transmitting A while waiting for received INFO0c */
    V34_TX_STAGE_FIRST_A,
    V34_TX_STAGE_FIRST_NOT_A,
    V34_TX_STAGE_FIRST_NOT_A_REVERSAL_SEEN,
    V34_TX_STAGE_SECOND_A,
    /*! \brief L1 is being transmitted */
    V34_TX_STAGE_L1,
    /*! \brief L2 is being transmitted */
    V34_TX_STAGE_L2,
    V34_TX_STAGE_POST_L2_A,
    V34_TX_STAGE_POST_L2_NOT_A,
    V34_TX_STAGE_A_SILENCE,
    V34_TX_STAGE_PRE_INFO1_A,
    /*! \brief V.34 §11.2.2.2.4: answer modem sending INFOMARKSa after the
               INFO1c wait expired, until INFO1c arrives or Tone B is seen */
    V34_TX_STAGE_INFOMARKSA,
    /*! \brief V.90: waiting for Tone A after L2 before sending INFO1d */
    V34_TX_STAGE_V90_WAIT_TONE_A,
    /*! \brief V.90: waiting for INFO1a after sending INFO1d */
    V34_TX_STAGE_V90_WAIT_INFO1A,
    /*! \brief V.90 §9.2.1.1.5: send Tone B while receiving analog's L1/L2 */
    V34_TX_STAGE_V90_WAIT_RX_L2,
    /*! \brief V.90 §9.2.1.1.6: detected Tone A, waiting for Tone A reversal */
    V34_TX_STAGE_V90_WAIT_TONE_A_REV,
    /*! \brief V.90 §9.2.1.1.6: 40ms delay before Tone B reversal */
    V34_TX_STAGE_V90_B_REV_DELAY,
    /*! \brief V.90 §9.2.1.1.6: send Tone B reversal for 10ms then L1/L2 */
    V34_TX_STAGE_V90_B_REV_10MS,
    /*! \brief V.90 §9.2.1.1.1: digital answerer sending initial Tone B after INFO0d */
    V34_TX_STAGE_V90_PHASE2_B,
    /*! \brief V.90 §9.2.1.1.2: digital answerer has received INFO0a and is waiting for Tone A reversal */
    V34_TX_STAGE_V90_PHASE2_B_INFO0_SEEN,
    /*! \brief V.90 §9.5.1.2: 70 ms silence before Tone B in response to an
               analog-modem retrain request.  This is a Phase 2 state and MUST
               sit below V34_TX_STAGE_FIRST_S: it previously lived at the end
               of the enum, where the application's "tx_stage >= FIRST_S with
               no INFO1a means the peer declined V.90" demotion guard read it
               as late-training and dropped the whole retrained attempt to
               plain V.34 (observed live 2026-07-22).  The application mirrors
               this enum value-for-value; keep both copies in sync. */
    V34_TX_STAGE_V90_RETRAIN_SILENCE,
    /*! \brief INFO1 is being trasnmitted */
    V34_TX_STAGE_INFO1,

    V34_TX_STAGE_FIRST_B,
    V34_TX_STAGE_FIRST_B_INFO_SEEN,
    V34_TX_STAGE_FIRST_NOT_B_WAIT,
    V34_TX_STAGE_FIRST_NOT_B,
    V34_TX_STAGE_FIRST_B_SILENCE,
    V34_TX_STAGE_FIRST_B_POST_REVERSAL_SILENCE,
    V34_TX_STAGE_SECOND_B,
    V34_TX_STAGE_SECOND_B_WAIT,
    V34_TX_STAGE_SECOND_NOT_B,
    /*! \brief INFO0 is being resent on a bad startup */
    V34_TX_STAGE_INFO0_RETRY,

    V34_TX_STAGE_FIRST_S,
    V34_TX_STAGE_FIRST_NOT_S,
    /*! \brief The optional MD is being transmitted */
    V34_TX_STAGE_MD,
    V34_TX_STAGE_SECOND_S,
    V34_TX_STAGE_SECOND_NOT_S,
    /*! \brief TRN is being transmitted */
    V34_TX_STAGE_TRN,
    /*! \brief J is being transmitted */
    V34_TX_STAGE_J,
    /*! \brief J' is being transmitted */
    V34_TX_STAGE_J_DASHED,
    /*! \brief Phase 4: silence while waiting for caller to complete Phase 3 */
    V34_TX_STAGE_PHASE4_WAIT,
    /*! \brief Phase 4: S signal (128T) before TRN and MP */
    V34_TX_STAGE_PHASE4_S,
    /*! \brief Phase 4: S-bar signal (16T) before TRN and MP */
    V34_TX_STAGE_PHASE4_NOT_S,
    /*! \brief Phase 4: TRN signal (>=512T) before MP */
    V34_TX_STAGE_PHASE4_TRN,
    /*! \brief MP is being transmitted */
    V34_TX_STAGE_MP,

    /*! \brief Half-duplex initial stages */
    V34_TX_STAGE_HDX_INITIAL_A,
    V34_TX_STAGE_HDX_FIRST_A,
    V34_TX_STAGE_HDX_FIRST_NOT_A,
    V34_TX_STAGE_HDX_FIRST_A_SILENCE,
    V34_TX_STAGE_HDX_SECOND_A,
    V34_TX_STAGE_HDX_SECOND_A_WAIT,

    V34_TX_STAGE_HDX_FIRST_B,
    V34_TX_STAGE_HDX_FIRST_B_INFO_SEEN,
    V34_TX_STAGE_HDX_FIRST_NOT_B_WAIT,
    V34_TX_STAGE_HDX_FIRST_NOT_B,
    V34_TX_STAGE_HDX_POST_L2_B,
    V34_TX_STAGE_HDX_POST_L2_SILENCE,

    /*! \brief Half-duplex control channel stages */
    /*! \brief Sh and !Sh are being transmitted */
    V34_TX_STAGE_HDX_SH,
    /*! \brief The first ALT is being transmitted */
    V34_TX_STAGE_HDX_FIRST_ALT,
    /*! \brief The PPh is being transmitted */
    V34_TX_STAGE_HDX_PPH,
    /*! \brief The second ALT is being transmitted */
    V34_TX_STAGE_HDX_SECOND_ALT,
    /*! \brief MPh is being transmitted */
    V34_TX_STAGE_HDX_MPH,
    /*! \brief E is being transmitted */
    V34_TX_STAGE_HDX_E,

    /*! \brief V.90 §9.2.1.1.8 V.34 fallback: digital modem in the call-modem
        role, transmitting silence while the analogue modem (answer role)
        leads Phase 3 with S/S-bar/PP/TRN/J (V.34 §11.3.1.1.1-11.3.1.1.3).
        Appended at the end of the enum so every earlier value stays stable;
        the application mirrors this enum value-for-value. */
    V34_TX_STAGE_V34_FALLBACK_WAIT_J
};

enum v34_events_e
{
    V34_EVENT_NONE = 0,
    V34_EVENT_TONE_SEEN,
    V34_EVENT_REVERSAL_1,
    V34_EVENT_REVERSAL_2,
    V34_EVENT_REVERSAL_3,
    V34_EVENT_INFO0_OK,
    V34_EVENT_INFO0_BAD,
    V34_EVENT_INFO1_OK,
    V34_EVENT_INFO1_BAD,
    V34_EVENT_INFOH_OK,
    V34_EVENT_INFOH_BAD,
    V34_EVENT_L2_SEEN,
    V34_EVENT_S,
    V34_EVENT_J,
    V34_EVENT_J_DASHED,
    V34_EVENT_PHASE4_TRN_READY,
    V34_EVENT_TRAINING_FAILED,
    V34_EVENT_E,
    /*! The far end abandoned Phase 3/4 and restarted its handshake (V.90
        SILENCERETRAIN -> TONE_AB -> Phase 1).  Reported so the application can
        follow it back to Phase 2 instead of continuing to transmit Phase 3/4
        signals over a peer that is no longer listening for them. */
    V34_EVENT_PEER_RETRAIN,
    /*! A sustained run of binary ones on the DPSK stream, i.e. INFOMARKSa
        (V.34 10.1.2.3.6).  V.90 9.2.1.2.6 has the digital modem condition its
        receiver for "either Tone A or INFOMARKSa" once the INFO1a deadline
        passes, and gives each a different response: re-send INFO1d for
        INFOMARKSa, respond to a retrain for Tone A.  Without this event only
        Tone A could be detected, so the INFO1d re-send was being driven by the
        Tone A trigger -- the wrong branch of that clause. */
    V34_EVENT_INFOMARKSA_SEEN
};

typedef struct
{
    bool support_baud_rate_low_carrier[6];
    bool support_baud_rate_high_carrier[6];
    bool support_power_reduction;
    uint8_t max_baud_rate_difference;
    bool support_1664_point_constellation;
    uint8_t tx_clock_source;
    bool from_cme_modem;
    bool rate_3429_allowed;
} v34_capabilities_t;

typedef struct
{
    bool use_high_carrier;
    int pre_emphasis;
    int max_bit_rate;
} info1c_baud_rate_parms_t;

typedef struct
{
    int power_reduction;
    int additional_power_reduction;
    int md;
    int freq_offset;
    info1c_baud_rate_parms_t rate_data[6];
} info1c_t;

typedef struct
{
    int power_reduction;
    int additional_power_reduction;
    int md;
    int freq_offset;
    bool use_high_carrier;
    int preemphasis_filter;
    int max_data_rate;
    int baud_rate_a_to_c;
    int baud_rate_c_to_a;
} info1a_t;

typedef struct
{
    int power_reduction;
    int length_of_trn;
    bool use_high_carrier;
    int preemphasis_filter;
    int baud_rate;
    bool trn16;
} infoh_t;

typedef struct
{
    int type;
    int bit_rate_a_to_c;
    int bit_rate_c_to_a;
    int aux_channel_supported;
    int trellis_size;
    bool use_non_linear_encoder;
    bool expanded_shaping;
    bool mp_acknowledged;
    int signalling_rate_mask;
    bool asymmetric_rates_allowed;
    /*! \brief Only in an MP1 message */
    complexi16_t precoder_coeffs[3];
} mp_t;

typedef struct
{
    int type;
    int max_data_rate;
    int control_channel_2400;
    int trellis_size;
    bool use_non_linear_encoder;
    bool expanded_shaping;
    int signalling_rate_mask;
    bool asymmetric_rates_allowed;
    /*! \brief Only in an MPH1 message */
    complexi16_t precoder_coeffs[3];
} mph_t;

/*! The set of working parameters, which defines operation at the current settings */
typedef struct
{
    /*! \brief The code (0-16) for the maximum bit rate */
    int max_bit_rate_code;
    /*! \brief Whether expanded shaping is enabled for this working mode. */
    bool expanded_shaping;
    /*! \brief Parameters for the current bit rate and baud rate */
    int bit_rate;
    /*! \brief Bits per high mapping frame. A low mapping frame is one bit less. */
    int b;
    int j;
    /*! \brief The number of shell mapped bits */
    int k;
    int l;
    int m;
    int p;
    /*! \brief The number of uncoded Q bits per 2D symbol */
    int q;
    int q_mask;
    /*! \brief Mapping frame switching parameter */
    int r;
    int w;
    /*! The numerator of the number of samples per symbol ratio. */
    int samples_per_symbol_numerator;
    /*! The denominator of the number of samples per symbol ratio. */
    int samples_per_symbol_denominator;
} v34_parameters_t;

typedef struct
{
    /*! \brief True if this is the calling side modem. */
    bool calling_party;
    /*! \brief True if this is a full duplex modem. */
    bool duplex;
    /*! The current source end when in half-duplex mode */
    bool half_duplex_source;
    /*! The current operating state when in half-duplex mode */
    bool half_duplex_state;
    /*! \brief */
    int bit_rate;
    /*! \brief The callback function used to get the next bit to be transmitted. */
    span_get_bit_func_t get_bit;
    /*! \brief A user specified opaque pointer passed to the get_bit function. */
    void *get_bit_user_data;

    /*! \brief The callback function used to get the next aux channel bit to be transmitted. */
    span_get_bit_func_t get_aux_bit;
    /*! \brief A user specified opaque pointer passed to the get_aux_bit function. */
    void *get_aux_bit_user_data;

    /*! \brief The current baud rate selection, as a value from 0 to 5. */
    int baud_rate;
    /*! \brief True if using the higher of the two carrier frequency options. */
    bool high_carrier;

    /*! \brief The register for the data scrambler. */
    uint32_t scramble_reg;
    /*! \brief The scrambler tap which selects between the caller and answerer scramblers */
    int scrambler_tap;

    bool use_non_linear_encoder;

#if defined(SPANDSP_USE_FIXED_POINT)
    complexi16_t (*current_getbaud)(v34_state_t *s);
#else
    complexf_t (*current_getbaud)(v34_state_t *s);
#endif

    bool getbaud_null_logged;

    /*! \brief U_INFO for a V.90 analogue-role INFO1a (Table 11 bits 25:31):
               the Ucode the digital modem trains on.  Zero uses the default. */
    int v90_u_info;

    /*! \brief External per-baud symbol source.  When set, the V.34 modulator
               (RRC shaping, carrier, gain) is driven by symbols supplied from
               outside this state machine — see v34_tx_start_external_symbols().
               Used by the V.90 analogue role, whose Phase 3 sequencing is
               governed by PCM-domain events this module never sees. */
    void (*external_symbol_func)(void *user_data, float *re, float *im);
    /*! \brief A user specified opaque pointer passed to external_symbol_func. */
    void *external_symbol_user_data;

    /*! \brief Mapping frame parsed input */
    uint32_t r0;
    uint16_t qbits[8];
    uint16_t ibits[4];

    /*! \brief (x0,y0) (x1,y1)... */
    int mjk[8];

    int step_2d;

    bitstream_state_t bs;
    uint32_t bitstream;

    int i;

    /*! \brief Parameters for the current bit rate and baud rate */
    v34_parameters_t parms;

    /*! \brief We need to remember some old x values
               in the C code:  x[0]  x[1]  x[2]  x[3] x[4] x[5] x[6] x[7] x[8] x[9] x[10]
               in V.34:        x[-3] x[-2] x[-1] x[0] x[1] x[2] x[3] x[4] x[5] x[6] x[7] */
    complexi16_t x[8 + V34_XOFF];
    /*! \brief Precoder coefficients */
    complexi16_t precoder_coeffs[3];

    complexi16_t c;
    complexi16_t p;
    int z;
    int y0;
    int state;

#if defined(SPANDSP_USE_FIXED_POINT)
    int16_t gain;
#else
    float gain;
#endif

#if defined(SPANDSP_USE_FIXED_POINT)
    /*! \brief The root raised cosine (RRC) pulse shaping filter buffer. */
    int16_t rrc_filter_re[V34_INFO_TX_FILTER_STEPS];
    int16_t rrc_filter_im[V34_INFO_TX_FILTER_STEPS];
    complexi16_t lastbit;
#else
    /*! \brief The root raised cosine (RRC) pulse shaping filter buffer. */
    float rrc_filter_re[V34_INFO_TX_FILTER_STEPS];
    float rrc_filter_im[V34_INFO_TX_FILTER_STEPS];
    complexf_t lastbit;
#endif
    /*! \brief Current offset into the RRC pulse shaping filter buffer. */
    int rrc_filter_step;

    /*! \brief The current phase of the carrier (i.e. the DDS parameter). */
    uint32_t carrier_phase;
    /*! \brief The update rate for the phase of the control channel carrier (i.e. the DDS increment). */
    int32_t cc_carrier_phase_rate;
    /*! \brief The update rate for the phase of the V.34 carrier (i.e. the DDS increment). */
    int32_t v34_carrier_phase_rate;

    /*! \brief The current phase of the guard tone (i.e. the DDS parameter). */
    uint32_t guard_phase;
    /*! \brief The update rate for the phase of the guard tone (i.e. the DDS increment). */
    int32_t guard_phase_rate;
    /*! \brief Guard tone signal level. */
    float guard_level;
    /*! \brief The current fractional phase of the baud timing. */
    int baud_phase;

    int stage;
    int convolution;
    int training_stage;
    int current_modulator;
    int diff;

    int line_probe_cycles;
    int line_probe_step;
    float line_probe_scaling;
    int tone_duration;

    int super_frame;
    int data_frame;
    int s_bit_cnt;
    int aux_bit_cnt;

    uint16_t v0_pattern;

    /*! \brief Data mode: buffer holding the current mapping frame's 8 x 2D symbols (Q9.7 re,im pairs) */
    int16_t tx_mapping_frame_buf[16];
    /*! \brief Data mode: which 2D symbol (0-7) we're on within the current mapping frame */
    int tx_mapping_frame_step;
    /*! \brief Number of all-ones B1 mapping frames emitted.  Per
        §10.1.3.1/V.34, B1 is one complete data frame (P mapping frames). */
    int b1_frames_sent;
    /*! \brief Clause 10.1.3 modulation-factor power normalization. */
    float data_symbol_scale;
    /*! \brief True once TX has entered data mode (used by RX to freeze equalizer) */
    bool tx_data_mode;

    uint8_t txbuf[50];
    int txbits;
    int txptr;
    const uint8_t (*conv_encode_table)[16];

    bool info0_acknowledgement;
    int info0_retry_count;
    /*! \brief Set when the ranging sequence is re-entered from the V.34
        §11.2.2.1.1 INFO0 recovery.  The call modem has left its recovery loop
        and is transmitting Tone B, so FIRST_A must not wait for another
        INFO0c before sending the Tone A phase reversal. */
    bool phase2_reranging;

    /*! \brief V.90 mode: when true, INFO0 uses V.90 INFO0d format (62 bits)
        instead of standard V.34 INFO0 (49 bits).  Set by external v90 module. */
    bool v90_mode;
    /*! \brief V.90 §9.2.1.1.8: the analogue modem's INFO1a selected V.34
        (bits 37:39 in 0..5), so the digital modem proceeds per 11.3.1.1/V.34
        "assuming the role of a call modem".  Cleared on retrain, which
        returns to V.90 Phase 2 regardless of the analogue modem's choice. */
    bool v90_v34_fallback;
    /*! \brief V.90 PCM law: 0 = µ-law, 1 = A-law */
    int v90_pcm_law;
    /*! \brief V.92 INFO0d extensions.  Table 15 bit 27 advertises V.92
        capability; bit 26 requests the short Phase 2 procedure.  The
        application currently implements the long procedure only. */
    bool v92_info0_capable;
    bool v92_short_phase2_requested;
    bool v92_info1d_mode;
    /*! \brief V.92 INFO1d Table 17 bit 70: PCM upstream supported by this
        digital endpoint.  Off by default -- the upstream data path is still
        V.34/V.22bis, so claiming this commits to a receiver we do not have.
        Advertising it is what makes an analogue peer select V.92 rather than
        V.90, so it exists to exercise the Phase 3/4 upstream receivers. */
    bool v92_pcm_upstream_capable;
    /*! \brief V.90: count of L1/L2 rounds sent (need 2 before INFO1d) */
    int v90_l2_count;
    /*! \brief V.90 answerer: consecutive fast Tone-A recoveries while waiting
               for INFO1a. Used to abort repeated INFO1d loops earlier. */
    int v90_info1a_fast_retries;
    /*! \brief V.90 answerer: total Tone-A recovery loops while waiting for
               INFO1a. Used to abort peers that never produce a valid INFO1a
               but keep bouncing between Tone A and INFO1d. */
    int v90_info1a_total_retries;
    /*! \brief V.90 answerer: total Phase 2 INFO0d recovery loops caused by
               repeated stale INFO0a before INFO1a. Used to stop peers from
               trapping us in endless INFO0d/Tone A recovery cycles. */
    int v90_phase2_info0_recovery_loops;
    /*! \brief V.90 answerer: number of §9.5.1.2 retrain responses performed
               after the peer asserted Tone A instead of INFO1a. Bounded so a
               peer that retrains forever cannot trap the start-up. */
    int v90_info1a_retrain_responses;
    /*! \brief Number of durable Phase 2 Tone A reversal transactions already
               consumed by the TX state machine.  RX owns the corresponding
               monotonic count; TX advances this cursor one transaction at a
               time so a later INFO/tone event cannot erase peer progress. */
    int v90_phase2_reversals_consumed;
    /*! \brief Number of durable Phase 2 L2-complete transactions already
               consumed by the TX state machine. */
    int v90_phase2_l2_consumed;
    union
    {
        info1a_t info1a;
        info1c_t info1c;
        infoh_t infoh;
    };
    union
    {
        mp_t mp;
        mph_t mph;
    };
    /*! \brief Optional explicit MP direction-rate policy. When false, MP rate
               advertisement falls back to locally-derived defaults. */
    bool mp_rate_policy_valid;
    /*! \brief Explicit MP maximum answerer-to-caller signalling rate (N*2400). */
    int mp_rate_a_to_c;
    /*! \brief Explicit MP maximum caller-to-answerer signalling rate (N*2400). */
    int mp_rate_c_to_a;
    /*! \brief Final rates after intersecting both MP rate fields and masks.
        These are separate from mp: MP-prime differs from MP only in its
        acknowledge bit (V.34 10.1.3.9), so negotiation must not rewrite the
        locally transmitted MP body. */
    bool negotiated_rates_valid;
    int negotiated_rate_a_to_c;
    int negotiated_rate_c_to_a;

    int persistence2;

    /*! \brief The get_bit function in use at any instant. */
    span_get_bit_func_t current_get_bit;

    /*! \brief Used to align the transmit and receive positions, to ensure things like
               round trip delay are properly handled. */
    span_sample_timer_t sample_time;

    /*! \brief Pre-emphasis filter state for Phase 3+ TX */
    float pre_emphasis_buf[16];
    int pre_emphasis_idx;
    const float *pre_emphasis_coeffs;
    /*! \brief The selected pre-emphasis set, scaled to unity band power gain. */
    float pre_emphasis_norm_coeffs[16];

    /*! \brief Plain V.34 call modem is silent per 11.3.1.1.1-.3 while its
        primary receiver acquires the answerer's PP/TRN/J. */
    bool phase3_call_wait_j;

    /*! \brief Last values emitted by debug stage-change logging. */
    int last_logged_stage;
    int last_logged_modulator;
    /*! \brief sample_time at which the currently logged stage was entered, so
               stage-change logging can report how long each stage actually ran.
               Negative means no stage has been timed yet. */
    span_sample_timer_t stage_entry_sample_time;
    /*! \brief sample_time at which Phase 2 (the first INFO0) began, so the
               stage timeline can be read as an elapsed-time budget. Negative
               means Phase 2 has not started (or has already been summarised). */
    span_sample_timer_t phase2_entry_sample_time;

    logging_state_t *logging;
} v34_tx_state_t;

typedef struct
{
#if defined(SPANDSP_USE_FIXED_POINT)
    /*! \brief Low band edge filter for symbol sync. */
    int32_t symbol_sync_low[2];
    /*! \brief High band edge filter for symbol sync. */
    int32_t symbol_sync_high[2];
    /*! \brief DC filter for symbol sync. */
    int32_t symbol_sync_dc_filter[2];
    /*! \brief Baud phase for symbol sync. */
    int32_t baud_phase;
    
    /*! \brief Low band edge filter coefficients for symbol sync. */
    int32_t low_band_edge_coeff[3];
    /*! \brief High band edge filter coefficients for symbol sync. */
    int32_t high_band_edge_coeff[3];
    /*! \brief A coefficient common to the low and high band edges */
    int32_t mixed_edges_coeff_3;
#else
    /*! \brief Low band edge filter for symbol sync. */
    float symbol_sync_low[2];
    /*! \brief High band edge filter for symbol sync. */
    float symbol_sync_high[2];
    /*! \brief DC filter for symbol sync. */
    float symbol_sync_dc_filter[2];
    /*! \brief Baud phase for symbol sync. */
    float baud_phase;

    /*! \brief Low band edge filter coefficients for symbol sync. */
    float low_band_edge_coeff[3];
    /*! \brief High band edge filter coefficients for symbol sync. */
    float high_band_edge_coeff[3];
    /*! \brief A coefficient common to the low and high band edges */
    float mixed_edges_coeff_3;
#endif
} ted_t;

typedef struct
{
    /*! \brief Viterbi trellis state table
               16 4D symbols deep, with 16 states each
               Each state has 4 entries: cumulative path metric, and prev. path pointer, x, y
               circularly addressed */
    struct
    {
        /*! \brief Cumulative path metric */
        uint32_t cumulative_path_metric[64];
        /*! \brief Previous path pointer */
        uint16_t previous_path_ptr[64];
        uint16_t pts[64];
        uint16_t branch_error_x[16];
        /*! \brief Branches of the x and y coords of the points in the eight 4D subsets
                   to which a sequence of 2D points has been sliced.
                   indexed from 0 to 15 --> 8 points for 16 past 4D symbols */
        complexi16_t bb[2][16];
    } vit[16];
    /*! \brief Latest viterbi table slot. */
    int ptr;
    /*! \brief Countdown to the first data being available from the viterbi pipeline */
    int windup;
    int16_t curr_min_state;

    int16_t error[2][4];

    /*! \brief Eight 4D squared branch errors for each of 8 4D subsets.
               Indexed array for indexing from viterbi lookup table */
    uint16_t branch_error[8];

    /*! Number of states in the negotiated trellis (16, 32, or 64). */
    int state_count;
    /*! Predecessor state and branch for each state/input-bit pair.  These are
        kept separately because a packed byte cannot represent a 64-state
        predecessor plus the three-bit branch label. */
    uint8_t previous_state[64][4];
    uint8_t branch[64][4];
    const uint8_t (*encode_table)[16];
} viterbi_t;

typedef struct
{
    /*! \brief True if this is the calling side modem. */
    bool calling_party;
    /*! \brief True if this is a full duplex modem. */
    bool duplex;
    /*! The current source end when in half-duplex mode */
    bool half_duplex_source;
    /*! The current operating state when in half-duplex mode */
    bool half_duplex_state;
    /*! \brief */
    int bit_rate;
    /*! \brief The callback function used to put each bit received. */
    span_put_bit_func_t put_bit;
    /*! \brief A user specified opaque pointer passed to the put_bit routine. */
    void *put_bit_user_data;

    /*! \brief The callback function used to put each aux bit received. */
    span_put_bit_func_t put_aux_bit;
    /*! \brief A user specified opaque pointer passed to the put_aux_bit routine. */
    void *put_aux_bit_user_data;

    /*! V.90 digital-side callback for strictly descrambled Phase 4 CP bits. */
    span_put_bit_func_t put_phase4_bit;
    void *put_phase4_bit_user_data;

    /*! \brief A callback function which may be enabled to report every symbol's
               constellation position. */
    qam_report_handler_t qam_report;
    /*! \brief A user specified opaque pointer passed to the qam_report callback
               routine. */
    void *qam_user_data;
    /*! \brief Absolute sample index of the latest primary-channel symbol. */
    int qam_sample_time;

    /*! \brief The current baud rate selection, as a value from 0 to 5. */
    int baud_rate;
    /*! \brief True if using the higher of the two carrier frequency options. */
    bool high_carrier;

    int stage;
    int received_event;

    /*! \brief The register for the data scrambler. */
    uint32_t scramble_reg;
    /*! \brief The scrambler tap which selects between the caller and answerer scramblers */
    int scrambler_tap;
    /*! \brief Distance-to-grid over the B1 era, for comparison with data. */
    float v90_t3_b1_err;
    int v90_t3_b1_err_count;
    /*! \brief Distance-to-grid under a swept gain, to tell a scaling fault
        from symbols that are simply not on the constellation. */
    float v90_t3_gain_err[V34_V90_T3_GAIN_TRIALS];
    /*! \brief Decision-directed NLMS step for the upstream equalizer. */
    float v90_t3_dd_mu;
    /*! \brief Running mean square distance from the constellation, used to
        decide whether the timing detector is being fed anything real. */
    float v90_t3_sym_err_ema;
    /*! \brief Gardner timing recovery on the upstream symbol instant. */
    bool v90_t3_timing_enabled;
    v34_gardner_state_t v90_t3_gardner;
    /*! \brief First published upstream data bits, for structure checks. */
    uint32_t v90_t3_first_word;
    int v90_t3_first_bits;
    /*! \brief Shadow descrambler for the upstream polynomial check. */
    uint32_t v90_t3_alt_scramble;
    int v90_t3_ones;
    int v90_t3_alt_ones;
    int v90_t3_bit_count;
    /*! \brief Running decision-error statistics for the V.90 upstream. */
    float v90_t3_decision_err;
    float v90_t3_decision_pow;
    int v90_t3_decision_count;
    /*! \brief Capturing into the T/3 ring alongside the normal receiver,
        before the DATA handover. */
    bool v90_t3_capture_only;
    /*! \brief Raw descrambled upstream bit dump (ME_V90_UPSTREAM_BIT_DUMP). */
    FILE *v90_t3_bit_dump;
    bool v90_t3_bit_dump_tried;
    uint8_t v90_t3_dump_byte;
    int v90_t3_dump_bits;
    /*! \brief Superframe-phase search over the post-B1 data stream.  The
        peer's superframe counter at B1 is not something B1 tells us, so
        without this the phase is right about one call in j -- measured, one
        in six across fourteen calls.  v90_t3_sf_force is the next phase to
        try, applied by the decoder at its own data-frame boundary so the
        search never disturbs alignment; -1 means nothing pending. */
    int64_t v90_t3_data_symbols;
    int v90_t3_sf_force;
    int v90_t3_df_force;
    int v90_t3_relocks;
    bool v90_t3_sf_locked;
    int v90_t3_sf_tries;
    /*! \brief Where B1 starts in the raw ring, and the running per-frame
        error against the B1 template, used to find where B1 actually ends
        rather than assuming it is one data frame long. */
    int64_t v90_t3_b1_start;
    float v90_t3_b1_frame_err;
    bool v90_t3_in_b1;
    /*! \brief Raw-sample index of the E handover, the anchor the B1 search
        works outwards from.  Negative until E is seen. */
    int64_t v90_t3_e_anchor;
    /*! \brief The far-end scrambler tap as measured on Phase 4 TRN (0 if not
        measured decisively).  Survives the upstream-data handover, which
        otherwise re-imposes the spec's role default. */
    int v90_far_tap_measured;

    /*! \brief Whether the far-end transmitter uses V.34 nonlinear precoding. */
    bool use_non_linear_encoder;

    uint16_t v0_pattern;

    /*! \brief A power meter, to measure the HPF'ed signal power in the channel. */
    power_meter_t power;
    /*! \brief The power meter level at which carrier on is declared. */
    int32_t carrier_on_power;
    /*! \brief The power meter level at which carrier off is declared. */
    int32_t carrier_off_power;
    bool signal_present;

    bitstream_state_t bs;
    uint32_t bitstream;

    /*! \brief Mapping frame output */
    uint32_t r0;
    uint16_t qbits[8];
    uint16_t ibits[4];

    /*! \brief (x0,y0) (x1,y1)... */
    int mjk[8];

    int step_2d;
    /*! Absolute incoming 4D-symbol position within the superframe.  This is
        separate from step_2d, which advances only when delayed Viterbi
        decisions emerge. */
    int input_4d;

    /*! \brief Buffer to accumulate 8 equalized 2D symbols (as Q9.7 re,im pairs)
               before calling v34_put_mapping_frame() */
    int16_t mapping_frame_buf[16];
    int mapping_frame_count;
    float data_symbol_scale;
    int data_symbol_rotation;
    bool data_symbol_conjugate;

    /*! \brief V.90 upstream-only 8 kHz -> 9.6 kHz, T/3 receive path.
        It is armed at the E/B1 seam and never touches the G.711 bearer or the
        downstream PCM path.  B1 supplies the timing phase and supervised FSE
        solution; raw/FIR/FSE state then continues unchanged into DATA. */
    bool v90_t3_prepared;
    int v90_t3_trellis_size;
    /*! \brief Three-times-symbol-rate clock: 9000 for 3000 baud, 9600 for 3200. */
    int v90_t3_internal_rate;
    bool v90_t3_active;
    bool v90_t3_acquisition_attempted;
    bool v90_t3_acquired;
    int64_t v90_t3_input_count;
    int64_t v90_t3_next_output;
    int64_t v90_t3_output_count;
    float v90_t3_hilbert[V34_V90_T3_HILBERT_TAPS];
    int v90_t3_hilbert_pos;
    complexf_t v90_t3_input[V34_V90_T3_RESAMPLE_TAPS + 8];
    float v90_t3_rrc_coeff[V34_V90_T3_RRC_TAPS];
    complexf_t v90_t3_rrc[V34_V90_T3_RRC_TAPS];
    int v90_t3_rrc_pos;
    complexf_t v90_t3_raw[V34_V90_T3_RAW_SIZE];
    complexf_t v90_t3_matched[V34_V90_T3_RAW_SIZE];
    int64_t v90_t3_raw_count;
    complexf_t v90_t3_fse[V34_V90_T3_FSE_TAPS];
    bool v90_t3_fse_conjugate;
    int64_t v90_t3_next_symbol;
    int64_t v90_t3_publish_symbol;
    bool v90_t3_suppress_output;
    int v90_t3_b1_symbols;
    complexf_t v90_t3_b1[V34_V90_T3_B1_MAX_SYMBOLS];
    float v90_t3_training_match;

    /*! Known-sequence B1 acquisition for ordinary V.34 (10.1.3.1). */
    bool b1_acquisition_active;
    int b1_observed_symbols;
    complexf_t b1_observed[V34_V90_T3_B1_MAX_SYMBOLS];

    /*! \brief Parameters for the current bit rate and baud rate */
    v34_parameters_t parms;

    /*! \brief yt's are the noise corrupted points fed to the viterbi decoder.
               Assumed to have format 9:7 (7 fractional bits) */
    complexi16_t yt;
    complexi16_t xt[4];
  
    complexi16_t x[3];
    complexi16_t h[3];

    /*! \brief These are quantized points in the respective 2D coset (0,1,2,3) */
    complexi16_t xy[2][4];

    viterbi_t viterbi;

    /*! \brief ww contains old z, current z and current w */
    int16_t ww[3];

    /*! \brief The current phase of the carrier (i.e. the DDS parameter). */
    uint32_t carrier_phase;
    /*! \brief The carrier update rate saved for reuse when using short training. */
    int32_t carrier_phase_rate_save;

    /*! \brief The update rate for the phase of the control channel carrier (i.e. the DDS increment). */
    int32_t cc_carrier_phase_rate;
    /*! \brief The update rate for the phase of the V.34 carrier (i.e. the DDS increment). */
    int32_t v34_carrier_phase_rate;

    /*! \brief The root raised cosine (RRC) pulse shaping filter buffer. */
#if defined(SPANDSP_USE_FIXED_POINT)
    int16_t rrc_filter[V34_RX_FILTER_STEPS];
#else
    float rrc_filter[V34_RX_FILTER_STEPS];
#endif
    /*! \brief Adaptive equalizer coefficients and sample history for the primary channel. */
    complexf_t eq_coeff[V34_EQUALIZER_PRE_LEN + 1 + V34_EQUALIZER_POST_LEN];
    complexf_t eq_coeff_save[V34_EQUALIZER_PRE_LEN + 1 + V34_EQUALIZER_POST_LEN];
    complexf_t eq_buf[V34_EQUALIZER_MASK + 1];
    float eq_delta;
    /*! \brief Exponential moving average of equalizer output magnitude for fixed-radius QPSK target. */
    float eq_target_mag;
    /*! \brief Current offset into the RRC pulse shaping filter buffer. */
    int rrc_filter_step;
    /*! \brief Current read offset into the equalizer buffer. */
    int eq_step;
    /*! \brief Current write offset into the equalizer buffer. */
    int eq_put_step;
    int shaper_sets;

#if defined(SPANDSP_USE_FIXED_POINT)
    /*! \brief The scaling factor assessed by the AGC algorithm. */
    int16_t agc_scaling;
    /*! \brief The previous value of agc_scaling, needed to reuse old training. */
    int16_t agc_scaling_save;
#else
    /*! \brief The scaling factor assessed by the AGC algorithm. */
    float agc_scaling;
    /*! \brief The previous value of agc_scaling, needed to reuse old training. */
    float agc_scaling_save;
#endif
    ted_t pri_ted;
    ted_t cc_ted;

#if defined(SPANDSP_USE_FIXED_POINT)
    /*! \brief The proportional part of the carrier tracking filter. */
    float carrier_track_p;
    /*! \brief The integral part of the carrier tracking filter. */
    float carrier_track_i;
#else
    /*! \brief The proportional part of the carrier tracking filter. */
    float carrier_track_p;
    /*! \brief The integral part of the carrier tracking filter. */
    float carrier_track_i;
#endif

    const v34_rx_shaper_t *shaper_re;
    const v34_rx_shaper_t *shaper_im;

    /*! \brief The total symbol timing correction since the carrier came up.
               This is only for performance analysis purposes. */
    int total_baud_timing_correction;

    /*! \brief The current half of the baud. */
    int baud_half;
    /*! \brief The measured round trip delay estimate, in sample times */
    int round_trip_delay_estimate;

    int duration;
    int bit_count;
    int target_bits;
    uint16_t crc;
    uint32_t last_angles[2];

    /*! \brief V.90 mode: when true, INFO0 RX expects V.90 INFO0a format (49 bits)
        instead of standard V.34 INFO0.  Set by external v90 module. */
    bool v90_mode;
    /*! \brief V.90 §9.2.1.1.8: the received INFO1a selected V.34 (Table 11
        layout, bits 37:39 in 0..5).  The frame was re-parsed with the V.34
        INFO1a field layout and the receiver expects the analogue modem to
        run Phase 3 in the V.34 answer-modem role. */
    bool v90_v34_fallback;

    /*! \brief Buffer for receiving info frames. */
    uint8_t info_buf[25];

    int super_frame;
    int data_frame;
    int s_bit_cnt;
    int aux_bit_cnt;

    uint8_t rxbuf[50];
    int rxbits;
    int rxptr;

    int blip_duration;

    v34_capabilities_t far_capabilities;

    /*! \brief Whether or not a carrier drop was detected and the signal delivery is pending. */
    int carrier_drop_pending;
    /*! \brief A count of the current consecutive samples below the carrier off threshold. */
    int low_samples;
    /*! \brief A highest magnitude sample seen. */
    int16_t high_sample;
    /*! \brief Most recent power sample seen by info_rx(), for candidate diagnostics. */
    int32_t last_info_rx_power;
    /*! \brief Peak recent power sample seen by info_rx(), for candidate diagnostics. */
    int32_t last_info_rx_power_peak;
    /*! \brief Sample time of the last info_rx() power-peak reset. */
    span_sample_timer_t last_info_rx_power_peak_reset;
    /*! \brief Fast-attack/slow-decay peak tracker for the level of a real
               carrier, so the Tone A gate scales with the actual line level
               instead of an absolute constant that only suits one
               peer/gateway combination. */
    int32_t info_rx_carrier_ref;
    /*! \brief Consecutive near-silent baseband samples seen while in a
               Phase 3/4 stage, used to spot a peer that has given up and
               restarted its handshake. */
    int phase34_silence_samples;
    /*! \brief Set once a peer retrain has been reported, so it is announced
               only once per Phase 3/4 attempt. */
    bool phase34_retrain_reported;
    /*! \brief Goertzel accumulators (delay line and running block energy) for
               the V.90 Phase 3/4 Tone A retrain detector (9.3.1/9.4.1/V.90). */
    float phase34_tone_a_g1;
    float phase34_tone_a_g2;
    float phase34_tone_a_energy;
    /*! \brief Sample index within the current Tone A Goertzel block. */
    int phase34_tone_a_samples;
    /*! \brief Consecutive Tone-A-dominant blocks seen in a Phase 3/4 stage. */
    int phase34_tone_a_blocks;
    /*! \brief Set once a Tone A retrain has been reported, so it is announced
               only once per Phase 3/4 attempt. */
    bool phase34_tone_a_reported;

    bool info0_acknowledgement;
    uint8_t info0_raw_26_27;
    uint8_t info0d_nominal_power_code;
    uint8_t info0d_max_power_code;
    bool info0d_power_measured_at_codec_output;
    bool info0d_pcm_alaw;
    bool info0d_upstream_3429_support;
    uint8_t info0d_reserved_41;
    bool info0d_extensions_valid;

    /*! \brief Set true once a valid INFO0 has been received (survives event overwrites). */
    bool info0_received;
    bool info1a_received;
    bool info1c_received;
    uint8_t info1a_raw_12_17;
    uint8_t info1a_raw_32_33;
    uint16_t info1a_raw_40_49;
    /*! \brief Per-rate carrier choices transmitted by this digital modem in
        INFO1d.  Table 10 reserves bits 32:33, so the later INFO1a cannot carry
        this choice back; the receiver must retain its own Table 9 rows. */
    /*! Carrier choices sent in local INFO1c/INFO1d, retained because the
        peer's INFO1a selects a row without repeating its carrier bit. */
    bool local_info1c_high_carrier[6];
    /*! \brief Sticky flag for V.90 answerer recovery when stale INFO0a is seen
               during the INFO1a wait window. */
    bool v90_repeated_info0a_pending;
    /*! \brief Set by TX after INFO1d has been sent; tells RX to prioritise
               INFO1a decoding over the INFO0a prefix check. */
    bool v90_info1d_sent;

    union
    {
        info1a_t info1a;
        info1c_t info1c;
        infoh_t infoh;
    };

    int step;
    int persistence1;
    int persistence2;
    /*! Run length of consecutive binary ones on the DPSK info stream, used to
        tell INFOMARKSa (ones, a phase reversal every baud) from Tone A
        (unmodulated, so zeros) while waiting for INFO1a.  See
        V34_EVENT_INFOMARKSA_SEEN. */
    int v90_infomarksa_run;
    /*! Goertzel state for the 1800 Hz guard tone and the 2400 Hz carrier, and
        the ratio between them in dB.  V.34 10.1.2.1 and 10.1.2.3 fix both
        levels, so the ratio says which state the peer is in:
          Tone A / A-bar   Tone A -1 dB, guard 0 dB  ->  about +1 dB
          INFO sequences   carrier -1 dB, guard -7 dB -> about -6 dB
        Measured live at -5.8 dB when INFO1a decoded and +2.9 dB when it did
        not, i.e. the peer was still in Tone A and had not sent INFO1a at all.
        That is the detection 9.2.1.2.6 needs and persistence2 never gave. */
    float guard_g1;
    float guard_g2;
    float carrier_g1;
    float carrier_g2;
    int guard_block_len;
    float guard_carrier_db;
    int guard_carrier_valid;
    int phase3_s_guard_samples;
    int phase3_s_hits;
    int phase3_s_event_count;
    /* Dedicated enable for the Phase 3 S detector.  This used to be inferred
       from phase3_j_trn16 >= 0, which is a constellation hint that doubles as
       a "Ja already consumed" latch and is cleared to -1 from several places.
       Any such clear silently switched the S detector off for the rest of the
       call, and the analogue S is what terminates Jd (9.3.1.5) and DIL
       (9.3.1.6) -- so losing it drags all of Phase 3 onto timers.  Same defect
       class as the TRN lock (2bd09c2) and the Tone A reversal count (f90f030):
       do not keep sequencing progress in a shared slot. */
    bool phase3_s_detect_armed;
    /* Set while we are transmitting Jd and the analogue modem is required to
       be silent (§9.3.2.4 "terminate Ja and transmit silence", until §9.3.2.7
       starts S).  Energy arriving in that window is the far end having left
       V.90 altogether -- measured on the d-modem rig, a
       "V90AutoDigitalImpDetector -> drop to V34 requested" abort shows up as
       SILENCERETRAIN then Phase 1 tones, ~2 s before our Jd budget expires.
       Not set during DIL: §9.3.2.9 lets the peer send SCR there. */
    bool phase3_expect_silence;
    int phase3_energy_samples;
    bool phase3_energy_retrain_reported;
    bool phase3_s_present;
    uint32_t phase3_s_alt_window;
    int phase3_s_alt_count;
    int phase3_s_stable_windows;
    uint8_t phase3_s_ring[32];
    float phase3_s_mag_ring[32];
    int phase3_s_counts[4];
    /* Sustained-rotation S detection: against real peers (SmartLink d-modem)
       the far-end S / S-bar present as a steady +/-90 degrees-per-symbol
       rotation (a single dominant differential dibit held for the whole
       128T signal), not the A,B,A,B alternation the alt detector looks for.
       Scrambled Ja never holds one dibit beyond ~10 bauds, so a long-run
       dominant ±90 dibit is an unambiguous S. */
    int phase3_s_dom_windows;   /* consecutive bauds the 32-window is dominated by a +/-90 dibit */
    int phase3_s_dom_symbol;    /* the currently dominant differential dibit (1 or 3), else -1 */
    int phase3_s_fired_symbol;  /* dibit whose rotation set the current phase3_s_present, else -1 */
    int phase3_s_pos;
    uint8_t phase3_pp_lag8[8];
    int phase3_pp_obs;
    int phase3_pp_match;
    float phase3_pp_error[48];
    complexf_t phase3_pp_corr[48];
    float phase3_pp_corr_energy;
    float phase3_pp_corr_weight;
    complexf_t phase3_pp_rotation;
    int phase3_pp_phase;
    int phase3_pp_phase_score;
    /* Mean |equalized symbol - known PP reference| / |reference| over the PP
       conditioning window: a descrambler-independent front-end health check. */
    float phase3_pp_resid_sum;
    int phase3_pp_resid_count;
    /* Mean distance from the nearest 4-point constellation point over the TRN
       refine window, normalised by the point radius.  Small means the symbols
       are clean and any TRN failure is a mapping/descrambler choice; large
       means the equalizer is not holding once PP stops driving it. */
    float phase3_trn_resid_sum;
    int phase3_trn_resid_count;
    /* V.90 9.4 CP is differentially encoded (10.1.3.3).  A differential
       decode has no carrier-phase ambiguity, so the dibit transform is fixed
       by the encoder convention rather than searched.  -1 = not pinned. */
    int v90_cp_diff_hypothesis;
    int phase3_pp_acquire_hits;
    int phase3_pp_started;
    uint32_t phase3_j_scramble[24];
    uint32_t phase3_j_stream[24];
    uint8_t phase3_j_prev_z[24];
    uint8_t phase3_j_prev_valid[24];
    uint32_t phase3_j_win[24][3][16];
    int phase3_j_bits;
    int phase3_j_lock_hyp;
    int phase3_j_trn16;
    int phase3_j_candidate_hyp;
    int phase3_j_candidate_phase;
    int phase3_j_candidate_pat;
    int phase3_j_candidate_count;
    int phase3_j_candidate_last_bits;
    uint32_t phase3_trn_scramble[24];
    uint16_t phase3_trn_one_count[24];
    int phase3_trn_bits;
    /*! \brief Bit count at which the TRN hypothesis scores were last rescored,
               so the lock can be retaken on post-convergence data once the
               Phase 3 tracking loops have settled. 0 = not yet rescored. */
    int phase3_trn_rescore_bits;
    /*! \brief Latched once TRN first locks: keeps the Phase 3 tracking loops
               engaged even while the lock itself is being retaken, so a
               rescore does not disengage tracking mid-convergence. */
    bool phase3_tracking_armed;
    int phase3_trn_lock_hyp;
    int phase3_trn_lock_score;
    float phase3_trn_mag_sum;
    int phase3_trn_mag_count;
    uint32_t phase3_ja_scramble[24];
    uint8_t phase3_ja_prev_z[24];
    uint8_t phase3_ja_prev_valid[24];
    int phase3_ja_bits;
    int phase3_ja_hyp;
    uint8_t phase3_ja_capture[65536];
    int phase3_ja_capture_len;
    uint8_t phase3_ja_capture_hyp[24][65536];
    int phase3_ja_capture_hyp_len[24];
    uint8_t phase3_ja_capture_hyp_raw[24][65536];
    int phase3_ja_capture_hyp_raw_len[24];
    int phase4_j_seen;
    int phase4_j_lock_hyp;
    int phase4_j_bits;
    uint32_t phase4_j_scramble_tap[2][2][2][8];
    uint32_t phase4_j_stream_tap[2][2][2][8];
    uint8_t phase4_j_prev_z_tap[2][2][2][8];
    uint8_t phase4_j_prev_valid_tap[2][2][2][8];
    uint32_t phase4_j_win_tap[2][2][2][8][16];
    int phase4_trn_after_j;
    uint32_t phase4_trn_scramble_tap[2][2][2][24];
    uint16_t phase4_trn_one_count_tap[2][2][2][24];
    uint32_t phase4_trn_scramble[24];
    uint8_t phase4_trn_prev_z[24];
    uint8_t phase4_trn_prev_valid[24];
    uint16_t phase4_trn_one_count[24];
    int phase4_trn_lock_hyp;
    int phase4_trn_lock_score;
    int phase4_trn_lock_tap;
    int phase4_trn_lock_order;
    int phase4_trn_lock_domain;
    int phase4_trn_current_hyp;
    int phase4_trn_current_score;
    int phase4_trn_current_tap;
    int phase4_trn_current_order;
    int phase4_trn_current_domain;
    uint32_t phase4_trn_recent_scramble;
    uint16_t phase4_trn_recent_window_bits;
    uint16_t phase4_trn_recent_window_ones;
    uint16_t phase4_trn_recent_window_fill;
    int phase4_trn_recent_score;
    uint8_t phase4_trn_recent_symbol_ones[256];
    uint8_t phase4_trn_recent_active;

    /* MP or MPh receive tracking data */
    int mp_count;
    int mp_len;
    int mp_and_fill_len;
    int mp_seen;
    bool last_rx_mp_valid;
    mp_t last_rx_mp;
    /*! \brief Baud count when mp_seen was first set to 1 (for E-detect timeout) */
    int mp_accepted_baud;
    /*! \brief Consecutive bauds of above-threshold equalizer output seen
        since entering Phase 4 MP search, used to require the CMA
        equalizer time to re-adapt after a post-TRN silence/ramp-up gap
        before a preamble lock is attempted (see MP_LOCK_MIN_SIGNAL_MAG2
        and MP_LOCK_SETTLE_BAUDS in v34rx.c). Reset to 0 whenever the
        signal drops back below threshold. */
    int mp_signal_settle_bauds;
    int mp_remote_ack_seen;
    int mp_hypothesis;
    uint32_t mp_hyp_scramble[24];
    uint32_t mp_hyp_bitstream[24];
    uint8_t mp_frame_bits[188];
    int mp_frame_pos;
    int mp_frame_target;
    int mp_early_rejects;
    int16_t mp0_vote_counts[88];
    int mp0_vote_frames;
    int mp0_vote_hyp;
    int16_t mp1_vote_counts[188];
    int mp1_vote_frames;
    int mp1_vote_hyp;
    int mp_phase4_default_scrambler_tap;
    int mp_phase4_default_bit_order;
    int mp_phase4_default_domain;
    int mp_phase4_reject_streak;
    int mp_phase4_nolock_count;
    int mp_phase4_alt_tap_active;
    int mp_phase4_alt_order_active;
    int mp_phase4_alt_domain_active;
    int mp_phase4_retry_mode;
    int mp_phase4_bit_order;
    int mp_phase4_domain;
    int mp_phase4_force_abs_active;
    int mp_phase4_diff_collapse_streak;
    int mp_phase4_diff_recover_streak;
    /*! \brief Decision-aided Phase 4 carrier acquisition: expected absolute
        symbol angle (DDS phase units), integrated from the hypothesis-locked
        differential dibit decisions.  Seeded by snapping the received angle
        to the 45-degree constellation family at lock, so the loop pulls the
        constellation onto the true grid up to a 90-degree ambiguity (which
        V.34's differential quadrant bits absorb in data mode). */
    uint32_t phase4_da_expected_ang;
    /*! \brief True once the decision-aided tracker has been seeded. */
    int phase4_da_active;
    /*! \brief Sticky: the tracker has seeded at least once this Phase 4.
        Keeps CMA muted through the CPt-to-CP silence so it cannot
        re-randomize the data-aided equalizer solution on line noise. */
    int phase4_da_seeded;
    /*! \brief Post-equalizer derotator (DDS phase units) owned by the
        decision-aided tracker.  The phase wander it corrects comes from the
        CMA equalizer's phase-blind tap rotation, which sits downstream of
        the carrier NCO -- correcting there fights the equalizer's group
        delay, so the rotation is taken out at the equalizer output where the
        loop delay is zero and a deadbeat update is stable. */
    uint32_t phase4_da_derot;
    int last_logged_mp_diag_state;

    int dft_ptr;
#if defined(SPANDSP_USE_FIXED_POINT)
    int16_t dft_buffer[160];
    int32_t l1_l2_gains[25];
    int32_t l1_l2_phases[25];
    int32_t base_phase;
    complexf_t last_sample;
    #else
    complexf_t dft_buffer[160];
    float l1_l2_gains[25];
    float l1_l2_phases[25];
    float base_phase;
    complexf_t last_sample;
#endif
    int l1_l2_duration;
    /*! L2-only accumulation used to derive the V.34/V.90/V.92 INFO1
        probing fields.  V.92 Table 17 requires probing results, rather than
        configured transmitter defaults. */
    float l1_l2_gain_sum[25];
    int l1_l2_gain_count;
    float l1_l2_noise_sum;
    int l1_l2_noise_count;
    float l1_l2_prev_1050_phase;
    float l1_l2_1050_phase_step_sum;
    int l1_l2_1050_phase_step_count;
    int l1_l2_have_prev_1050_phase;

    int current_demodulator;

    /*! \brief Phase 4 S signal detection: count of data_bits=2 in sliding window */
    int s_detect_count;
    /*! \brief How many Tone A phase reversals have been recognised so far.
               Owned solely by the Tone A detector. This progress used to live
               in received_event, which the TX state machine clears in 39
               places, so a reversal sequence could be reset to zero between
               reversals and never reach the third. */
    int phase2_reversal_count;
    /*! \brief How many Phase 2 L1/L2 probes have completed.  Unlike
               received_event, this is not cleared when TX consumes an
               unrelated INFO or tone event. */
    int phase2_l2_count;
    /*! \brief Phase 4 S signal detection: 32-bit circular window of data_bits==2 flags */
    uint32_t s_window;

    /*! \brief Last values emitted by debug stage-change logging. */
    int last_logged_stage;
    int last_logged_event;
    int last_logged_demodulator;
    bool training_failed_reported;

    /*! \brief Used to align the transmit and receive positions, to ensure things like
               round trip delay are properly handled. */
    span_sample_timer_t sample_time;

    span_sample_timer_t tone_ab_hop_time;

    logging_state_t *logging;
} v34_rx_state_t;

/*!
    V.34 modem descriptor. This defines the working state for a single instance
    of a V.34 modem.
*/
struct v34_state_s
{
    /*! \brief True if this is the calling side modem. */
    bool calling_party;
    /*! \brief True if this is a full duplex modem. */
    bool duplex;
    /*! The current source end when in half-duplex mode */
    bool half_duplex_source;
    /*! The current operating state when in half-duplex mode */
    bool half_duplex_state;
    /*! \brief The bit rate of the modem. */
    int bit_rate;

    v34_tx_state_t tx;
    v34_rx_state_t rx;
    modem_echo_can_state_t *ec;

    /*! \brief True when using the primary channel (Phase 3/4 and data mode).
        During Phase 2, both sides use 1200 Hz DPSK; during Phase 3+, TX and RX
        use separated carriers (e.g. 1829/1920 Hz at 3200 baud).  External echo
        cancellers should only operate when this flag is true. */
    bool primary_channel_active;

    /*! \brief Error and flow logging control */
    logging_state_t logging;
};

#endif
/*- End of file ------------------------------------------------------------*/
