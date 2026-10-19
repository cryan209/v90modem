/* K56flex upstream report control receiver, Draft 0.23 clause 11.
 * Resident MICA 5903/5961/1D5A, PM 0294/02B0; NOT overlay 8E.
 * Input is carrier-corrected, equalized four-point I/Q, one symbol at a time.
 * This is NOT a PCM receiver: S/S-bar/PP acquisition, timing, equalization,
 * MP/E and upstream data are outside this module. No engine gates are set.
 */
#ifndef K56FLEX_UPSTREAM_H
#define K56FLEX_UPSTREAM_H

#include "k56flex.h"

#define K56U_REPORT 1
#define K56U_REPORT_END 2

typedef struct {
    k56flex_report_rx_t report;
    unsigned previous_raw;
    uint64_t symbols;
    uint64_t end_window;
    unsigned end_bits;
    int ended;
} k56flex_upstream_t;

/* The live rig uses tap 18 (18/23); tap 5 is supported for diagnostic controls.
 * Returns -1 without modifying rx for an unsupported tap. Resets per call. */
int k56flex_upstream_init(k56flex_upstream_t *rx, unsigned tap);
/* Returns a one-shot K56U_* event, 0, or -1 for an invalid point (no state
 * change). Amplitude is not used; callers own gain and decision confidence.
 * Any fixed quarter-turn phase ambiguity cancels in differential decoding.
 * REPORT exposes report.ext/report.report; k56flex_report_field() converts it
 * to the downstream mapper field. END requires three repeated 899F words
 * AFTER a report, a conservative receiver policy, not a recovered threshold.
 * MP/E must be handed to a different decoder after END. */
int k56flex_upstream_symbol(k56flex_upstream_t *rx, float real, float imag);

#endif
