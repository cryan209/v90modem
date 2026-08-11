/*
 * v92_upstream_data.h — V.92 PCM-upstream data-frame core
 *
 * Implements the analogue-modem data transmitter and deterministic receiver:
 * GPA (§6.3), the twelve-interval modulus encoder (§6.4.1), precoder and
 * prefilter (§6.4.2), inverse map (§6.4.3), and the initial 16-state V.34
 * convolutional-code/Viterbi path (§6.4.4).
 */

#ifndef V92_UPSTREAM_DATA_H
#define V92_UPSTREAM_DATA_H

#include <stdbool.h>
#include <stdint.h>

#include "v92_phase4_decode.h"

#ifdef __cplusplus
extern "C" {
#endif

#define V92_UPSTREAM_INTERVALS 12
#define V92_UPSTREAM_MAX_FRAME_BITS 72

typedef struct {
    uint8_t drn;                       /* Table 30: 1..19 during data mode */
    uint8_t moduli[V92_UPSTREAM_INTERVALS];
} v92_upstream_profile_t;

typedef struct {
    uint32_t scramble_reg;
    int previous_differential_sign;    /* d(f-1), initialized to zero */
} v92_upstream_tx_state_t;

typedef struct {
    uint32_t descramble_reg;
    int previous_differential_sign;    /* d(f-1), initialized to zero */
} v92_upstream_rx_state_t;

typedef struct {
    v92_upstream_tx_state_t data;
    uint8_t convolutional_state;
    double u_history[V92_CPD_MAX_TAPS];
    double x_history[V92_CPD_MAX_TAPS];
    double v_history[V92_CPD_MAX_TAPS];
} v92_upstream_wave_tx_t;

typedef struct {
    v92_upstream_rx_state_t data;
    uint8_t convolutional_state;
    double u_history[V92_CPD_MAX_TAPS];
    double x_history[V92_CPD_MAX_TAPS];
    double v_history[V92_CPD_MAX_TAPS];
    uint64_t symbols;
    uint64_t slicing_errors;
} v92_upstream_wave_rx_t;

/* V.92 Table 30 and §6.1: rate=(drn+17)*8000/6, hence a twelve-symbol
 * data frame carries K=2*(drn+17) bits.  Zero is the cleardown code. */
int v92_upstream_bits_per_frame(uint8_t drn);

/* Validate the §6.4.1 requirement product(Mi) >= 2^K. */
bool v92_upstream_profile_validate(const v92_upstream_profile_t *profile);

void v92_upstream_tx_init(v92_upstream_tx_state_t *state);
void v92_upstream_rx_init(v92_upstream_rx_state_t *state);

/* Apply §6.3 GPA scrambling and §6.4.1 modulus encoding.  Bits are one per
 * byte in transmission order (b0 first); ki_out contains K0..K11. */
bool v92_upstream_encode_frame(v92_upstream_tx_state_t *state,
                               const v92_upstream_profile_t *profile,
                               const uint8_t *bits,
                               int bit_count,
                               uint8_t ki_out[V92_UPSTREAM_INTERVALS]);

/* Inverse of v92_upstream_encode_frame.  The Ki values are what the later
 * constellation/trellis receiver recovers. */
bool v92_upstream_decode_frame(v92_upstream_rx_state_t *state,
                               const v92_upstream_profile_t *profile,
                               const uint8_t ki[V92_UPSTREAM_INTERVALS],
                               uint8_t *bits_out,
                               int bits_max);

/* Reference §6.4.2-.4 waveform path.  It currently implements the 16-state
 * convolutional encoder selected by CPd trellis code 0.  Samples are linear
 * values after the CPd gain G; the live G.711 slicer/equalizer sits outside
 * this deterministic core. */
void v92_upstream_wave_tx_init(v92_upstream_wave_tx_t *state);
void v92_upstream_wave_rx_init(v92_upstream_wave_rx_t *state);
bool v92_upstream_wave_profile_validate(const v92_cpd_frame_t *cpd);
bool v92_upstream_wave_encode_frame(v92_upstream_wave_tx_t *state,
                                    const v92_cpd_frame_t *cpd,
                                    const uint8_t *bits,
                                    int bit_count,
                                    double samples[V92_UPSTREAM_INTERVALS]);
bool v92_upstream_wave_decode_frame(v92_upstream_wave_rx_t *state,
                                    const v92_cpd_frame_t *cpd,
                                    const double samples[V92_UPSTREAM_INTERVALS],
                                    uint8_t *bits_out,
                                    int bits_max);

/* Three-transition, 16-state Viterbi decoder for the initial unfiltered CPd
 * profile.  It keeps the two nearest points per symbol and uses §6.4.4's Y0
 * parity/trellis constraint to correct hard-slicer errors. */
bool v92_upstream_wave_decode_viterbi_frame(
    v92_upstream_wave_rx_t *state,
    const v92_cpd_frame_t *cpd,
    const double samples[V92_UPSTREAM_INTERVALS],
    uint8_t *bits_out,
    int bits_max);

#ifdef __cplusplus
}
#endif

#endif /* V92_UPSTREAM_DATA_H */
