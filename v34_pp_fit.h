/*
 * v34_pp_fit.h — train a fractionally-spaced equaliser on §10.1.3.6's PP,
 * which is what V.34 and V.90 both say PP is for.
 *
 * V.34 §10.1.3.6: "Signal PP consists of six periods of a 48-symbol sequence
 * and is used by the remote modem for TRAINING ITS EQUALIZER."
 * V.90 §9.3.1.2: "After detecting signal S and the S-to-S-bar transition, the
 * digital modem shall condition its receiver to BEGIN TRAINING ITS EQUALISER
 * USING SIGNAL PP.  After receiving signal PP, the digital modem may further
 * refine its equaliser using the first 512T of signal TRN."
 *
 * So the spec's own ordering is detect S, train on PP, refine on TRN -- and
 * the reason this module exists is that on a real 2-wire bearer the
 * constellation-domain conditioning we have cannot do the middle step.
 * Measured on a live HSF-to-digital call, both training signals arrive
 * essentially undamaged -- S at a three-bin fraction of 0.618 and PP at a
 * periodicity of 0.890 against 0.902 transmitted -- and the receiver still
 * publishes no S, PP, TRN or J event and hands the Ja parser noise.
 *
 * PP is the right signal to fit and S is not, which is worth stating because
 * the obvious move is to fit S:
 *
 *   - §10.1.3.7's S is three spectral lines (fc and fc +/- baud/2).  Three
 *     complex constraints cannot determine a 32-tap equaliser however cleanly
 *     S is received.  S is a DETECTOR's signal, not a trainer's.
 *   - PP is 288 symbols of a wideband sequence, 90 ms at 3200 baud, and it is
 *     PERIODIC at 48 symbols, so it also supplies its own alignment.
 *
 * The fit is the same discipline as v90_analogue_sd.h's: least squares against
 * the known sequence, solved once, scored on data the fit never saw.  Two
 * differences follow from PP being a passband complex-valued signal rather than
 * Sd's real PCM ladder:
 *
 *   - the taps are COMPLEX, and they operate on the REAL received samples, so
 *     the filter selects the analytic signal and equalises it in one step.  A
 *     time-invariant filter cannot shift frequency, so the known carrier goes
 *     into the REFERENCE (ref[k] = PP(k) * exp(j*2*pi*fc*k/baud)) rather than
 *     into a mixer.  The carrier PHASE is then a constant complex factor and
 *     the fit absorbs it, along with the channel and the sampling instant.
 *   - the sample rate need not be a multiple of the symbol rate.  At the
 *     8 kHz a G.711 bearer gives and 3200 baud it is 2.5 samples per symbol,
 *     so symbols land on two distinct sub-sample phases; each gets its own tap
 *     set, which is a two-branch polyphase equaliser and not an approximation.
 */
#ifndef V34_PP_FIT_H
#define V34_PP_FIT_H

#include <stdbool.h>
#include <stdint.h>

#define V34_PP_PERIOD       48
#define V34_PP_SYMBOLS      (6*V34_PP_PERIOD)
#define V34_PP_MAX_TAPS     64
#define V34_PP_MAX_BRANCH   8

/* §10.1.3.6 equation 10-1, i = 0 .. 287.  PP(0) is transmitted first. */
void v34_pp_reference(int i, double *re, double *im);

typedef struct {
    int     taps;                   /* taps per branch */
    int     branches;               /* sub-sample phases, 1 or more */
    double  h_re[V34_PP_MAX_BRANCH][V34_PP_MAX_TAPS];
    double  h_im[V34_PP_MAX_BRANCH][V34_PP_MAX_TAPS];
    double  score;                  /* held out, 1.0 is perfect */
    int     symbols;                /* symbols the window supplied */
} v34_pp_fit_t;

/*
 * Fit over `n` real samples at `fs` Hz, for `baud` symbols/s and carrier `fc`.
 *
 * The window must hold at least 96 symbols; 288 (one whole PP) is what §9.3.1.2
 * offers and is what the caller should give it.  Returns true when the held-out
 * score reaches the threshold.
 */
bool v34_pp_fit(const int16_t *amp, int n, double fs, double baud, double fc,
                int taps, v34_pp_fit_t *out);

/*
 * The same fit, with the alignment searched.
 *
 * Unlike v90_analogue_sd.h's fit, the alignment here is NOT absorbed by the
 * filter.  PP repeats every 48 symbols -- 15 ms at 3200 baud -- and the filter
 * spans a few milliseconds, so a start offset a whole symbol out presents the
 * reference rotated and the residual explodes.  Measured on a live call, the
 * score goes 0.994 at the right instant and -2.0 half a millisecond away.
 *
 * So the offset is swept, over one period of the sequence, at one input sample.
 * `amp` must therefore hold the window plus one period.  `offset_out` receives
 * the winning start, in samples from `amp`.
 */
bool v34_pp_fit_search(const int16_t *amp, int n, double fs, double baud,
                       double fc, int taps, int symbols,
                       v34_pp_fit_t *out, int *offset_out);

/*
 * Apply a fitted equaliser: symbol `k` of the same window, as a complex value.
 * On a good fit this reproduces ref[k], so dividing out the carrier gives PP.
 */
void v34_pp_fit_apply(const v34_pp_fit_t *f, const int16_t *amp, int n,
                      double fs, double baud, int k, double *re, double *im);

#endif
