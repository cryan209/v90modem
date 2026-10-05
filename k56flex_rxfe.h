/* K56flex client receive front end over linear audio.
 *
 * A real client sees the line through its own codec: 8 kHz linear samples of the network
 * D/A output after the analogue loop, with an unknown gain, delay, band-limiting, noise and
 * a sample clock that is not quite the network's.  This front end turns that into the
 * symbol-rate equalized levels the decision-level client slices and consumes:
 *
 *   energy onset -> correlation against the known P1 probe (timing, fractional phase, gain)
 *   -> cubic resampler with a timing loop (clock offset) -> T-spaced NLMS equalizer trained
 *   on the known training stream, then decision-directed -> nearest-level slicer.
 *
 * Everything is our own design, informed by the spec's receive stages (resampler, FIR,
 * timing and carrier loops of clauses 7.15-7.23, gain control open).  It has met only the
 * simulated channel of k56flex_client_test, never a real loop.
 */
#ifndef K56FLEX_RXFE_H
#define K56FLEX_RXFE_H

#include "k56flex.h"

#include <stdint.h>

#define K56FLEX_EQ_TAPS 96
#define K56FLEX_EQ_LOOKAHEAD 32

typedef void (*k56flex_rxfe_sink_fn)(void *user, float equalized);

typedef struct k56flex_rxfe k56flex_rxfe_t;

k56flex_rxfe_t *k56flex_rxfe_new(k56flex_law_t law, k56flex_rxfe_sink_fn sink, void *user);
void k56flex_rxfe_free(k56flex_rxfe_t *fe);
/* One linear input sample; symbols are delivered through the sink as they become available
 * (none until acquisition, several at once when it completes). */
void k56flex_rxfe_push(k56flex_rxfe_t *fe, int16_t sample);
/* Set once acquisition has succeeded; the client then skips its timeline to P1. */
int k56flex_rxfe_locked(const k56flex_rxfe_t *fe);
/* Called after each block the client has processed: the reference for `n` delivered symbols
 * starting at absolute index first_symbol (the client may be holding some not yet used) (the expected levels on a known stage, otherwise the client's own decisions).
 * known = 1: reference is the expected stream (also drives timing); 0: decision-directed;
 * -1: hold the equalizer (the stages after training are too sparse to adapt on). */
void k56flex_rxfe_block(k56flex_rxfe_t *fe, uint64_t first_symbol, const int16_t *reference, unsigned n, int known);
/* Number of symbols delivered so far; the symbol being delivered has index count - 1. */
uint64_t k56flex_rxfe_symbols(const k56flex_rxfe_t *fe);
/* Statistics for tests and logs. */
float k56flex_rxfe_gain(const k56flex_rxfe_t *fe);
float k56flex_rxfe_snr_db(const k56flex_rxfe_t *fe);      /* equalizer output vs reference, recent */
float k56flex_rxfe_ppm(const k56flex_rxfe_t *fe);
unsigned k56flex_rxfe_locked_at(const k56flex_rxfe_t *fe);

#endif
