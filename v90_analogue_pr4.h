/*
 * v90_analogue_pr4.h — partial-response (1 - D^2) detection of the V.90
 * downstream over a two-wire analogue loop.
 *
 * Why: the loop's response is flat from 300 to 3400 Hz with a null at DC
 * (-33 dB) and another at 4000 Hz (-61 dB) -- measured on the Apple modem
 * (artifacts/apple-pve-20261015/v90-dfe2).  Equalising to the symbols means
 * inverting both nulls.  A linear equaliser cannot (14 dB), and decision
 * feedback can only by regenerating the lost energy from past decisions: its
 * dominant tap is ~1.9 at lag 2, so a single wrong decision cascades
 * (v90_analogue_fse.h).
 *
 * But "flat with nulls at DC and Nyquist" is the shape of the partial-response
 * class-IV target 1 - D^2, so equalise to THAT instead -- z[k] = s[k] - s[k-2]
 * -- which needs no null inverted (37 dB with 64 forward-only taps, 41.6 with
 * 128, against 15 to the symbols themselves), and detect with a Viterbi over
 * the known memory.  1 - D^2 is two interleaved 1 - D channels, so each
 * interleave is a trellis whose state is just the previous symbol on that
 * interleave.  Offline on v90-dfe2, trained on TRN1d and run over 8000 symbols
 * of TRN2d/MP: no symbol errors at a traceback of 16 per interleave, where
 * the DFE cascaded after 1319.  A 1 - D trellis has near-invisible constant-
 * offset error events, which is why shorter tracebacks are not enough (25
 * errors at 8 per interleave).
 *
 * The symbols' candidate sets come from the caller per symbol (the far end
 * maps each frame interval through its own constellation, §5.4.3), and must
 * be supersets of what can really arrive.
 */
#ifndef V90_ANALOGUE_PR4_H
#define V90_ANALOGUE_PR4_H

#include <stdbool.h>
#include <stdint.h>

typedef struct v90a_pr4_s v90a_pr4_t;

/*
 * Candidate set for absolute symbol index `index`: signed levels in the
 * equaliser's units and the codeword each stands for.  Return the count
 * (at most `max`), 0 if nothing is known (the step is then skipped).
 */
typedef int (*v90a_pr4_set_fn)(void *user, int64_t index,
                               double *levels, uint8_t *codewords, int max);

/* taps: T/2 forward taps.  dmax: decision delays 0..dmax symbols tried when
 * training.  depth: traceback per interleave. */
v90a_pr4_t *v90a_pr4_init(int taps, int dmax, int depth);
void v90a_pr4_free(v90a_pr4_t *s);

/* Every T/2 sample the main equaliser takes, in order. */
void v90a_pr4_sample(v90a_pr4_t *s, int16_t sample);

/*
 * Once per symbol the main equaliser emits, after its decision: `decision` is
 * that symbol's decided level (equaliser units).  Symbol indices count these
 * calls from 0.  While training, decisions are the reference (they must be
 * right: §8.4.5's TRN1d is); once trained, the decisions are ignored.
 */
void v90a_pr4_symbol(v90a_pr4_t *s, double decision);

/* Accumulate the next `symbols` calls, then solve for the taps and the best
 * decision delay.  Returns false on bad arguments. */
bool v90a_pr4_train(v90a_pr4_t *s, int symbols);
/* Symbol calls so far: the index the next v90a_pr4_symbol() call is. */
int64_t v90a_pr4_symbols(const v90a_pr4_t *s);
bool v90a_pr4_trained(const v90a_pr4_t *s);
int v90a_pr4_delay(const v90a_pr4_t *s);          /* chosen decision delay */
double v90a_pr4_train_snr_db(const v90a_pr4_t *s);  /* fit quality on the training */

/*
 * Start detecting: from symbol index `first` on, decided codewords are
 * produced, in order, through v90a_pr4_take().  Needs v90a_pr4_trained().
 */
bool v90a_pr4_start(v90a_pr4_t *s, int64_t first,
                    v90a_pr4_set_fn set_fn, void *user);
bool v90a_pr4_running(const v90a_pr4_t *s);
/* Re-decide every symbol not yet decided with freshly queried sets: call it
 * whenever what the set callback would answer has changed. */
void v90a_pr4_resync(v90a_pr4_t *s);
/* Decided codewords, in symbol order.  Returns how many were written. */
int v90a_pr4_take(v90a_pr4_t *s, uint8_t *codewords, int max);
int64_t v90a_pr4_next_index(const v90a_pr4_t *s);   /* index of the next codeword out */

#endif
