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
    V90A_FSE_LS,        /* least-squares training on decisions (v90a_fse_train_ls) */
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
/*
 * Install taps from outside — v90_analogue_sd.h's supervised fit to §8.4.4's
 * Sd, which is the only thing on the line before TRN1d that a blind algorithm
 * cannot use (Sd is not constant modulus, so CMA erases its zero slots).
 * Returns how many taps were taken, 0 if the count does not match.  The input
 * AGC is neutralised at the same time, because a fit already carries the line's
 * gain in its taps and applying a second one would scale the output away from
 * the unit modulus the fit produced.
 */
int v90a_fse_set_taps(v90a_fse_t *s, const double *h, int taps, int parity);
int v90a_fse_tap_count(const v90a_fse_t *s);
/* The NLMS step.  Acquisition and tracking want different ones: the step that
 * pulls the taps out of a saddle is far too large to sit at, since a gradient
 * loop's residual error is proportional to it. */
void v90a_fse_set_mu(v90a_fse_t *s, double mu);
/*
 * The two steps a call uses, and they differ by more than an order of
 * magnitude because they are doing different jobs.
 *
 * MU_TRAIN is for decision-directed adaptation on §8.4.5's TRN1d, where the
 * constellation is two points a long way apart and every decision is therefore
 * right: an NLMS step near unity is a projection, not a gamble, and the whole
 * point is to be converged before the signal stops being that easy.  It is
 * worth a lot -- at the hardest sampling phase, exact codeword recovery on the
 * multilevel stream that follows goes 52% -> 92% between 0.01 and this.
 *
 * MU_TRACK is for everything after, where decisions are ordinary and a gradient
 * loop's residual error is proportional to its step.
 */
#define V90A_FSE_MU_TRAIN   0.5
#define V90A_FSE_MU_TRACK   0.02
/*
 * MU_CMA is for the BLIND loop on TRN1d, and it is a third job again.  CMA's
 * error is y*(R2 - y*y) -- cubic in the output, and not the normalised
 * projection MU_TRAIN describes -- so the step that is right for a
 * decision-directed loop on two far-apart points is wildly wrong here.  Swept
 * against §8.4.5's own confirmation on reference-data/artifacts/hsf-v90/call-085428Z, which
 * reads 49.2% at MU_TRAIN:
 *
 *   0.5 (MU_TRAIN)  49.2%      0.01   70.7%
 *   0.05           100.0%      0.002  62.9%
 *
 * At 0.05 both in-tree analogue recordings reach 100.0%, Table 13's CRC finds
 * Jd unaided at TRN1d 20075T -- where the peer's 20004T of TRN1d ends -- and
 * 74 and 75 Jd frames decode into the DIL.
 */
#define V90A_FSE_MU_CMA     0.05
v90a_fse_mode_t v90a_fse_mode(const v90a_fse_t *s);

/*
 * The decision for the symbol most recently emitted -- V90A_FSE_DD adapts on
 * it, and decision feedback (below) needs it in every mode -- and the
 * half-width of the region that decision was taken inside
 * (v90a_linear_last_tolerance()).  Call it once per symbol, before the next
 * v90a_fse_put(); a symbol left without one is simply not adapted on, and one
 * whose sample fell in the outer half of its region is refused as unreliable.
 */
void v90a_fse_decide(v90a_fse_t *s, double decision, double tolerance);

/*
 * Decision feedback: nb symbol-spaced taps on the caller's past decisions,
 * subtracted from the output (y = h.x - b.d).  0 (the default) is the plain
 * FSE.
 *
 * Why: a two-wire loop has a null at 4000 Hz, and a linear equaliser inverting
 * a null amplifies whatever noise is there -- measured on the Apple modem's
 * Phase 4 (reference-data/artifacts/apple-pve-20261015/v90-lt6, echo cancelled), the best
 * linear T/2 equaliser reaches 13.8 dB at 64 taps and 16.9 at 256, while
 * adding 16 feedback taps reaches 45 dB.  Feedback cancels the postcursor ISI
 * from decisions instead of inverting it.  The decisions must be given for
 * every symbol (v90a_fse_decide(), in every mode -- only DD adapts on them);
 * a symbol left undecided feeds back as its own output.  The feedback taps
 * adapt with the forward ones, in DD only.
 */
int v90a_fse_set_feedback(v90a_fse_t *s, int nb);
int v90a_fse_feedback(const v90a_fse_t *s);

/*
 * Train forward and feedback taps together by least squares: `passes` passes
 * of `symbols` decisions each, solving and installing after every pass, then
 * continue in `then` (V90A_FSE_DD at the current step, or V90A_FSE_FROZEN).
 *
 * Why not just DD: the 4000 Hz null that makes feedback necessary also makes
 * the problem badly conditioned, and a gradient loop crawls along exactly the
 * directions the feedback is for -- measured on the Apple modem's TRN1d
 * (v90-lt6), NLMS-DD from the CMA solution stayed at the CMA's ~8 dB for
 * 16000 symbols.  Why passes: the first pass learns from the CMA's decisions,
 * 2.8% of which are wrong, and the errors it is taught cap it at 22.5 dB; the
 * second pass decides with the first's taps (34.1 dB) and the third reaches
 * 48.9 dB, where least squares on the TRUE symbols reaches 49.7.  The input
 * AGC is frozen for the duration, as for any non-CMA mode.
 */
int v90a_fse_train_ls(v90a_fse_t *s, int symbols, int passes, v90a_fse_mode_t then);
/* Least-squares solves installed so far. */
int v90a_fse_ls_solved(const v90a_fse_t *s);
/* Times the feedback history was cleared because the output ran away. */
int v90a_fse_fb_resets(const v90a_fse_t *s);

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
/* Decisions adapted on, and decisions refused as untrustworthy.  A run where
 * the second is climbing is a receiver losing the constellation, not a channel
 * being tracked. */
int v90a_fse_dd_used(const v90a_fse_t *s);
int v90a_fse_dd_rejected(const v90a_fse_t *s);
double v90a_fse_level(const v90a_fse_t *s);      /* input AGC level */
int v90a_fse_symbols(const v90a_fse_t *s);
/* Where the energy of the tap set sits, in symbols from the first tap: a
 * converged equaliser's centre of mass is what the sampling phase moved. */
double v90a_fse_centre(const v90a_fse_t *s);

#endif
