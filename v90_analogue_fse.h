/*
 * v90_analogue_fse.h — a T/2 fractionally-spaced equaliser for the analogue
 * modem's PCM downstream.
 *
 * Why this exists, and why it is fed at 16 kHz rather than 8:
 *
 * The downstream is one G.711 level per 125 µs DS0 interval, so its symbol rate
 * is 8 kHz and the line passes at most about 3.6 kHz.  That is ZERO excess
 * bandwidth, and it has two consequences that between them decide the whole
 * shape of this receiver (docs/hsf_analogue_v90_coupler.md):
 *
 *  - The received process is stationary, not cyclostationary, so no
 *    non-data-aided timing detector can find the symbol instant.  Measured on a
 *    real downstream through a 3600 Hz low-pass, mean square varies by 0.04%
 *    across an entire symbol.
 *  - The channel's first neighbours carry ±0.14 against a main tap of 0.80, and
 *    the µ-law ladder's steps near the top are finer than that, so the levels
 *    cannot be sliced however well the instant is chosen.
 *
 * A fractionally-spaced equaliser answers both at once, which a symbol-spaced
 * one cannot: sampled at T/2 the channel and the sampling phase are one linear
 * filter, so inverting the filter absorbs the phase.  There is then no timing
 * loop to acquire — "works at any sampling phase" is a property of the
 * structure, not of a loop that has to converge to something.  That is why the
 * coupler must hand over its 16 kHz stream before it decimates.
 *
 * Training is blind, on §8.4.5's TRN1d: scrambled ones on a single Ucode, so
 * ±U, constant modulus, and 30000T of it — 3.75 s at the length this project
 * transmits.  The constant-modulus algorithm needs no knowledge of the signs,
 * which is what makes it usable on a scrambler output.  The output is
 * normalised to ±1 by the CMA's own dispersion constant, so the level ambiguity
 * the raw slicer had (v90_analogue_linear.h) is settled at the same time: the
 * unit is TRN1d's level, and §8.4.4's Sd sits 6.6 dB above it.
 *
 * Decision-directed adaptation is available for after acquisition, but the
 * decisions come from the caller — this module does not know the constellation,
 * only that it is trying to make one.
 */
#ifndef V90_ANALOGUE_FSE_H
#define V90_ANALOGUE_FSE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    V90A_FSE_CMA = 0,   /* blind, constant modulus (TRN1d) */
    V90A_FSE_DD,        /* decision-directed; the caller supplies decisions */
    V90A_FSE_FROZEN,
} v90a_fse_mode_t;

typedef struct v90a_fse_s v90a_fse_t;

/*
 * taps is the number of T/2 taps — an even number, spanning taps/2 symbols.
 * 32 (16 symbols) covers everything measured on this channel with room over.
 * mu is the NLMS step; 0 takes the default.
 */
v90a_fse_t *v90a_fse_init(int taps, double mu);
void v90a_fse_free(v90a_fse_t *s);

/*
 * Feed 16 kHz samples; emit one equalised symbol per DS0 interval.
 *
 * Returns how many symbols were written to out[], which is about len/2.  out[]
 * must have room for len/2 + 1.
 */
int v90a_fse_put(v90a_fse_t *s, const int16_t *amp, int len,
                 double *out, int max);

void v90a_fse_set_mode(v90a_fse_t *s, v90a_fse_mode_t mode);
v90a_fse_mode_t v90a_fse_mode(const v90a_fse_t *s);

/*
 * The decision for the symbol most recently emitted, for V90A_FSE_DD.  Call it
 * once per symbol, before the next v90a_fse_put(); a symbol left without one is
 * simply not adapted on.
 */
void v90a_fse_decide(v90a_fse_t *s, double decision);

/*
 * Diagnostics.
 *
 * dispersion is the mean of (y^2 - 1)^2 over the last window — the quantity CMA
 * minimises, so it says whether the equaliser has opened the eye at all, and on
 * a two-level signal it is directly the residual.  Near 1.0 is an unconverged
 * equaliser; the level a converged one reaches on this channel is measured in
 * v90_analogue_rx_test.
 */
double v90a_fse_dispersion(const v90a_fse_t *s);
double v90a_fse_level(const v90a_fse_t *s);      /* input AGC level */
int v90a_fse_symbols(const v90a_fse_t *s);
/* Where the energy of the tap set sits, in symbols from the first tap: a
 * converged equaliser's centre of mass is what the sampling phase moved. */
double v90a_fse_centre(const v90a_fse_t *s);

#endif
