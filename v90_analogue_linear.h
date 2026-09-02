/*
 * v90_analogue_linear.h — a linear-sample front end for the analogue modem's
 * Phase 3/4 receiver (§8.4, §8.6).
 *
 * v90_analogue_rx.c and v90_analogue_phase4.c are codeword state machines,
 * because on a digital bearer the received stream *is* the DS0 the far D/A
 * converter is driven from and the codewords arrive byte-exact.  On a real
 * two-wire analogue line they do not: what arrives is a level, scaled by the
 * line's loss, sitting on whatever DC the codec adds.  The signals themselves
 * are unchanged -- §8.4's Sd, S̄d, TRN1d, Jd are still one G.711 level per DS0
 * interval -- so the state machine above is still the right one; what is
 * missing is the slicer that turns a level back into a Ucode.
 *
 * That is this module: DC removal, a level estimate, and nearest-Ucode
 * quantisation, producing exactly the codeword stream the digital-bearer path
 * hands v90_analogue_phase3_rx().  Both bearers then run the same receiver.
 *
 * Two things it deliberately does NOT do, and they bound what it can deliver:
 *
 *  - The absolute scale is not recoverable, and is not needed for acquisition.
 *    §8.4.4's W is learned off the wire (see sd_hunt_slot() in
 *    v90_analogue_rx.c) precisely because a digital modem need not honour the
 *    U_INFO we asked for, so a consistently scaled ladder acquires Sd, S̄d,
 *    TRN1d and Jd exactly as an unscaled one does.  The gain is therefore
 *    pinned so the measured line level maps onto a reference Ucode, and then
 *    FROZEN (v90a_linear_lock()) -- Sd and TRN1d are 6.6 dB apart by §8.4.4/
 *    §8.4.5 and a gain that keeps chasing the peak would erase that ratio,
 *    which is the one thing the S̄d→TRN1d boundary is found by.
 *    Consequence: a §9.3.2.9 DIL measurement taken through this front end is
 *    relative to that reference, not to the far end's dBm0.
 *
 *  - There is no equaliser and no symbol timing recovery.  Each input sample
 *    is taken to be one DS0 interval, sampled where the caller sampled it.
 */
#ifndef V90_ANALOGUE_LINEAR_H
#define V90_ANALOGUE_LINEAR_H

#include <stdbool.h>
#include <stdint.h>

#include "v90.h"

typedef struct v90a_linear_s v90a_linear_t;

v90a_linear_t *v90a_linear_init(v90_law_t law);
void v90a_linear_free(v90a_linear_t *s);

/*
 * Slice linear samples into G.711 codewords.
 *
 * Returns how many codewords were written to out[] (at most max).  The
 * conversion runs a block behind the input -- the level estimate for a block
 * is taken from that block -- so a call can return fewer codewords than it was
 * given samples, and the balance comes out of the next call.
 */
int v90a_linear_put(v90a_linear_t *s,
                    const int16_t *amp, int len,
                    uint8_t *out, int max);

/* Freeze the gain.  Call once the receiver has acquired Sd: everything after
 * that point is identified by its level *relative* to Sd. */
void v90a_linear_lock(v90a_linear_t *s);
bool v90a_linear_locked(const v90a_linear_t *s);

/* Diagnostics: the gain in use, and the line level it was derived from. */
double v90a_linear_gain(const v90a_linear_t *s);
double v90a_linear_level(const v90a_linear_t *s);

#endif
