/*
 * v90_sounder.h — a channel sounder of our own, for the band V.34's probe
 * cannot reach.
 *
 * V.34's line probing signal ends at 3750 Hz (11.2.3), and measured on the HSF
 * coupler's path that is 8.4 dB down and still climbing
 * (docs/hsf_analogue_v90_coupler.md).  What happens in the last 250 Hz decides
 * two things that nothing else can settle: whether the received signal is
 * band-limited strictly inside the 4 kHz Nyquist of the 8 kHz DS0 symbol rate
 * -- in which case there is no symbol-rate timing tone for any non-data-aided
 * detector, and no equaliser can produce an ISI-free response -- and how much
 * intersymbol interference the analogue receiver has to undo.
 *
 * Our own transmit is not constrained to V.34's 150 Hz grid, so this puts
 * tones on a 50 Hz grid to 3950 Hz.  Everything here is representable in the
 * DS0 stream by construction: the block is 160 samples at 8 kHz, so every tone
 * is an exact multiple of the block rate and nothing leaks between bins.
 *
 * It is measured as a two-port, not by assuming the transmitted spectrum: the
 * codewords this produces are what the far end actually put on the line, so a
 * capture of them (ME_G711_CAPTURE) divides out of the received spectrum, and
 * the µ-law quantisation the transmit path applies is inside the reference
 * rather than inside the answer.
 */
#ifndef V90_SOUNDER_H
#define V90_SOUNDER_H

#include <stdbool.h>
#include <stdint.h>

#include "v90.h"

/* Samples in one repetition.  20 ms at 8 kHz, so the tone grid is 50 Hz --
 * the same period V.34's probe uses, which keeps one analysis window able to
 * measure either. */
#define V90_SOUNDER_BLOCK   160

/*
 * The tones, ascending, in Hz.  Returns the count.
 *
 * 150 Hz to 3750 Hz on V.34's own grid, so a sounding overlaps the probe and
 * the two can be compared directly, then 3800, 3850, 3900 and 3950 -- which is
 * the point of the exercise.  V.34's four deliberate gaps (900, 1200, 1800,
 * 2400) are left empty here too: they are the noise reference, and a sounder
 * with no empty bins cannot tell a weak tone from the floor.
 */
const int *v90_sounder_tones(int *count);
/* The frequencies deliberately left empty, for that floor. */
const int *v90_sounder_empty_bins(int *count);

/*
 * Fill count codewords with the sounder, continuing from `phase` (the sample
 * position within the block, updated on return).  A caller that keeps `phase`
 * across calls emits a continuous signal whatever the frame size.
 */
void v90_sounder_fill(v90_law_t law, uint8_t *codewords, int count,
                      int *phase);

/* One block as linear samples, for analysis and for the self-check. */
void v90_sounder_block_linear(int16_t *out);

#endif
