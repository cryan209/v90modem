/*
 * v92_trn2u.h — V.92 TRN2u PAM modem (upstream Phase 4 control channel)
 *
 * The analogue modem transmits TRN2u, SUVu, CPu, and E2u during Phase 4
 * using a 4- or 8-point PAM constellation (Tables 28/29 of V.92) at one
 * symbol per G.711 codeword.  All bits pass through the upstream GPA
 * scrambler (x^23 + x^18 + 1, the same polynomial the Phase 3 TRN1u
 * receiver in v92_p3_rx.c uses); the sign bit (constellation MSB) is
 * additionally differentially encoded.  Sign convention follows V.92
 * §8.5.7: PCM MSB = 1 is positive; a set constellation MSB selects the
 * negative level.
 *
 * Bit order within a symbol: the first scrambler-output bit is the
 * constellation MSB (sign), followed by the magnitude bits MSB-first.
 *
 * The demodulator slices raw G.711 codewords back to constellation labels,
 * differentially decodes the sign, descrambles, and feeds the recovered
 * bits straight into a v92_cp_rx_t so SUVu/CPu/CPus frames are delivered
 * from the live G.711 stream.  The modulator is the analogue-side
 * simulation used by loopback tests.
 */

#ifndef V92_TRN2U_H
#define V92_TRN2U_H

#include <stdbool.h>
#include <stdint.h>

#include "v92_cp_rx.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int constellation_points;   /* 2 for Phase-3 CPt, 4 or 8 for TRN2u */
    double lu;                  /* L_U reference amplitude (linear) */
    bool alaw;                  /* G.711 law of the transport */
    uint32_t scramble_reg;
    int prev_sign;              /* differential encoder memory */
} v92_trn2u_tx_t;

typedef struct {
    int constellation_points;   /* 2 for Phase-3 CPt, 4 or 8 for TRN2u */
    double lu;
    bool alaw;
    uint32_t descramble_reg;
    int prev_sign;
    bool prev_sign_valid;       /* first symbol only seeds the decoder */
    v92_cp_rx_t *sink;          /* receives every descrambled bit */
    uint32_t symbols;
    uint32_t frames_accepted;   /* sink frames that passed validation */
    uint32_t descrambled_one_run;
    uint32_t longest_descrambled_one_run; /* TRN2u lock-quality diagnostic */
    int bit_permutation[3];    /* diagnostic wire hypothesis; default 0,1,2 */
    int sign_mode;             /* v92_trn2u_sign_mode_t */
    int descrambler_mode;      /* v92_trn2u_descrambler_mode_t */
} v92_trn2u_demod_t;

typedef enum {
    V92_TRN2U_SIGN_DIFFERENTIAL = 0,
    V92_TRN2U_SIGN_DIFFERENTIAL_INVERTED,
    V92_TRN2U_SIGN_ABSOLUTE,
    V92_TRN2U_SIGN_ABSOLUTE_INVERTED
} v92_trn2u_sign_mode_t;

/* GPA = 1 + x^-5 + x^-23 (V.92 §6.3, the analogue modem's polynomial —
 * the correct default), GPC = 1 + x^-18 + x^-23.  LEFT/RIGHT are the two
 * shift-register reflections.  Until 2026-07-23 the GPA/GPC prefixes were
 * swapped relative to the taps they selected; results recorded against the
 * old labels mean the other polynomial. */
typedef enum {
    V92_TRN2U_DESCRAMBLER_GPA_LEFT = 0,
    V92_TRN2U_DESCRAMBLER_GPA_RIGHT,
    V92_TRN2U_DESCRAMBLER_GPC_LEFT,
    V92_TRN2U_DESCRAMBLER_GPC_RIGHT
} v92_trn2u_descrambler_mode_t;

void v92_trn2u_tx_init(v92_trn2u_tx_t *tx,
                       int constellation_points,
                       double lu,
                       bool alaw);

/* §8.7.6: reset GPA at TRN2u entry while carrying the final E1u sign. */
void v92_trn2u_tx_start(v92_trn2u_tx_t *tx, int preceding_e1u_sign);

/* Bits per PAM symbol: 1 for Phase-3 2-point, 2 for 4-point, 3 for 8-point. */
int v92_trn2u_bits_per_symbol(int constellation_points);

/*
 * Modulate nbits (a multiple of bits-per-symbol) into G.711 codewords.
 * Returns the number of codewords written, or 0 on error.
 */
int v92_trn2u_tx_bits(v92_trn2u_tx_t *tx,
                      const uint8_t *bits,
                      int nbits,
                      uint8_t *codewords,
                      int codewords_max);

/* Transmit TRN2u proper: scrambled binary ones for nsymbols symbols. */
int v92_trn2u_tx_ones(v92_trn2u_tx_t *tx,
                      uint8_t *codewords,
                      int nsymbols);

void v92_trn2u_demod_init(v92_trn2u_demod_t *demod,
                          int constellation_points,
                          double lu,
                          bool alaw,
                          v92_cp_rx_t *sink);

/* Diagnostic-only wire hypotheses. Defaults implement Tables 28/29 and GPA. */
bool v92_trn2u_demod_set_hypothesis(
    v92_trn2u_demod_t *demod,
    const int bit_permutation[3],
    v92_trn2u_sign_mode_t sign_mode,
    v92_trn2u_descrambler_mode_t descrambler_mode);

/*
 * Feed raw G.711 codewords; recovered bits go to the v92_cp_rx sink.
 * Returns the number of frames the sink accepted during this call.
 */
int v92_trn2u_demod_feed(v92_trn2u_demod_t *demod,
                         const uint8_t *codewords,
                         int count);

#ifdef __cplusplus
}
#endif

#endif /* V92_TRN2U_H */
