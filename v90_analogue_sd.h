/*
 * v90_analogue_sd.h — scale-free acquisition of §8.4.4's Sd on the analogue
 * modem's EQUALISED symbol stream.
 *
 * Why this exists.
 *
 * v90_analogue_rx.c acquires Sd from CODEWORDS, which is right on a digital
 * bearer and circular on an analogue one: the slicer that produces those
 * codewords (v90_analogue_linear.h) has to be told what level a Ucode is, and
 * the only thing on the line that says so is Sd itself.  Measured live against
 * an HT802 with a real 2-wire path, the digital modem transmitted Sd (64 reps),
 * S-bar-d and TRN1d -- its own log says so, and a 1333 Hz component (8000/6,
 * the line Sd's six-symbol pattern puts in the band) is present in the analogue
 * end's own recording at 38% of the power -- while this end reported
 * "hunting Sd (Sd 0 reps ...)" for the whole attempt and never left the hunt.
 *
 * What breaks the circle is that Sd's structure carries no scale.  §8.4.4 makes
 * it the six-symbol pattern
 *
 *      +W, +0, +W, -W, -0, -W
 *
 * so on ANY scale: four slots at one magnitude with signs + + - -, and two
 * slots at zero, repeating with period six.  A detector that measures the ratio
 * of the zero slots to the non-zero ones, and the sign pattern, needs no
 * calibrated ladder, no U_INFO, and no absolute level -- and the equaliser's
 * output is exactly where that measurement can be made, because the FSE has
 * already absorbed the channel and the sampling phase.
 *
 * It also SUPPLIES the calibration the slicer was missing: once the grid is
 * locked, the mean magnitude of the four non-zero slots is W's level in the
 * equaliser's own units, which is what v90a_linear_set_reference() wants.
 *
 * Note this is acquisition only.  Which Ucode W actually is remains a separate
 * question -- a digital modem need not honour the U_INFO we asked for -- and
 * §8.4.4/§8.4.5's 6.6 dB Sd-to-TRN1d ratio is still measured off the wire at
 * the S-bar-d seam by the codeword receiver, exactly as before.
 */
#ifndef V90_ANALOGUE_SD_H
#define V90_ANALOGUE_SD_H

#include <stdbool.h>
#include <stdint.h>

typedef struct v90a_sd_s v90a_sd_t;

/*
 * Acquire the equaliser itself, on Sd, by a supervised fit.
 *
 * The streaming detector below scores the STRUCTURE of an already-equalised
 * stream.  It cannot be reached from a live call on its own, because the only
 * equaliser this receiver has is blind: §8.4.5's TRN1d is constant modulus and
 * CMA is right for it, but Sd is 4 slots at W and 2 at zero, and driving that
 * to a constant modulus erases the zeros -- measured, CMA fed real Sd through a
 * dispersive channel produces +/-1 on ALL SIX slots, which is the pattern's
 * sign and none of its structure.
 *
 * Sd needs no blind algorithm, because §8.4.4 makes it a KNOWN sequence.  So
 * fit the T/2 taps to it directly: least squares against the reference
 * +1, 0, +1, -1, 0, -1, solved once over a window.
 *
 * The slot phase and the T/2 sub-sample offset are NOT hypotheses that have to
 * be searched.  A fractionally-spaced filter of this length can supply any
 * delay, so every (slot, offset) pair fits equally well and differs only by
 * which tap carries the energy -- the fit absorbs them, exactly as it absorbs
 * the channel and the sampling phase, and the output comes out aligned to slot
 * 0 of the reference.  The sign is likewise free: fitting -r returns -h with an
 * identical residual, so this cannot tell Sd from §9.3.2.4's S-bar-d, and does
 * not need to -- what marks that boundary is the CHANGE, which is unambiguous.
 *
 * The score is HELD OUT: the taps are fitted on the first half of the window
 * and the residual measured on the second.  With 32 free taps a least-squares
 * fit will explain almost anything in-sample, so an in-sample score would
 * accept TRN1d and noise alike.
 *
 * amp is 16 kHz, two samples per DS0 interval.  h_out receives `taps` T/2 taps
 * (h_out[0] the most recent sample), ready for v90a_fse_set_taps().  Returns
 * true when the held-out score reaches the threshold.
 */
#define V90A_SD_MAX_TAPS 64
bool v90a_sd_fit(const int16_t *amp, int n, int taps, int *parity_out,
                 double *h_out, double *score_out, double *level_out);

/*
 * reps is how many six-symbol repetitions each decision is taken over.  §8.4.4
 * sends Sd for at least 64 reps and this project's own transmitter sends
 * exactly that, so the window has to be a good deal shorter than the signal:
 * 16 reps (96 symbols, 12 ms) leaves room for the hunt to start late and still
 * see several windows.  0 takes the default.
 */
v90a_sd_t *v90a_sd_init(int reps);
void v90a_sd_free(v90a_sd_t *s);
void v90a_sd_reset(v90a_sd_t *s);

/*
 * Feed one equalised symbol.  Returns true on the sample that completes an
 * acquisition; the phase and level are then valid and stay valid until reset.
 */
bool v90a_sd_put(v90a_sd_t *s, double sym);

bool v90a_sd_acquired(const v90a_sd_t *s);
/*
 * The slot index, 0..5, that the NEXT symbol fed will occupy — 0 being the
 * first +W of §8.4.4's pattern.
 */
int v90a_sd_next_slot(const v90a_sd_t *s);
/* W's magnitude in the equaliser's own output units. */
double v90a_sd_level(const v90a_sd_t *s);
/* The score the acquisition was taken on, in [0, 1]; see the .c for the form. */
double v90a_sd_score(const v90a_sd_t *s);

#endif
