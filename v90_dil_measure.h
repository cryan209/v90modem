/*
 * v90_dil_measure.h — what the analogue modem actually does with DIL (§9.3.2.9).
 *
 * DIL is Digital Impairment Learning, and the analogue modem is the one that
 * asked for it: §8.4.1 has the DIL descriptor travel *to* the digital modem in
 * Ja, and §9.3.2.9 says the analogue modem "shall receive the DIL sequence it
 * requested".  The descriptor is therefore an input on this side, never
 * something to be recovered — the receiver already knows every codeword that
 * was meant to arrive.  Its job is to measure what *did*.
 *
 * That is a different problem from the one v90_dil_rx.c solves.  That module
 * recovers a descriptor nobody told us, which is what offline analysis of
 * somebody else's capture needs; it is not a step in a V.90 start-up.
 *
 * It also has a much weaker input requirement.  §9.3.2.10 lets the analogue
 * modem cut DIL short as soon as it "has received enough of the DIL sequence",
 * so a partial pass is the normal case, not a degraded one.  Every DIL-segment
 * independently measures one training Ucode, so half a cycle measures half the
 * Ucodes, and nothing here needs a whole one.
 *
 * What comes out feeds constellation selection: the Ucodes that arrive
 * distinguishable from their neighbours are the ones worth offering in CP
 * (docs/v90_mi_negotiation.md).
 */
#ifndef V90_DIL_MEASURE_H
#define V90_DIL_MEASURE_H

#include "v90.h"

#include <stdbool.h>
#include <stdint.h>

#define V90_DIL_UCODES 128

typedef struct {
    int      tx_count;          /* training symbols sent at this Ucode */
    int      rx_ucode;          /* most frequently received Ucode */
    int      rx_agree;          /* symbols that arrived as rx_ucode */
    int      rx_distinct;       /* distinct received Ucodes */
    int      tx_level;          /* transmitted |level|, linear */
    int      rx_level;          /* mean received |level|, linear */
    uint8_t  slot_disagree;     /* bitmask of DS0 phases (mod 6) that vary */
} v90_dil_ucode_obs_t;

typedef struct {
    v90_dil_ucode_obs_t u[V90_DIL_UCODES];

    int    align_offset;        /* stream index where the DIL starts */
    double align_score;         /* normalised correlation at that offset */
    int    symbols_used;        /* received symbols folded into the result */
    int    cycle_len;           /* symbols in one full descriptor cycle */
    double coverage;            /* symbols_used / cycle_len */

    int    ucodes_measured;     /* Ucodes with at least one training symbol */
    double gain_db;             /* mean rx/tx level ratio, in dB (pad shows here) */
    uint8_t rbs_slot_mask;      /* DS0 phases whose codewords are being altered */

    bool   usable[V90_DIL_UCODES];  /* arrived distinguishable from neighbours */
    int    usable_count;
} v90_dil_measurement_t;

/*
 * Locate the requested DIL inside a received stream.
 *
 * Correlates on levels, not codewords, because a digital pad changes every
 * codeword while leaving the shape intact — matching codewords would fail
 * precisely on the impairment this exists to find.  Searches [from, from+span)
 * of `rx`; pass span <= 0 to search to the end.
 */
bool v90_dil_measure_align(const uint8_t *rx, int rx_len, v90_law_t law,
                           const v90_dil_desc_t *desc,
                           int from, int span,
                           int *offset_out, double *score_out);

/*
 * Fold a received DIL into per-Ucode observations.  `offset` is where the DIL
 * starts (from v90_dil_measure_align, or known from the Phase 3 state
 * machine).  Consumes as much as is present: a partial pass is expected.
 */
bool v90_dil_measure(const uint8_t *rx, int rx_len, v90_law_t law,
                     const v90_dil_desc_t *desc, int offset,
                     v90_dil_measurement_t *out);

/*
 * The measured Ucodes that a receiver can still tell apart, ascending.
 * Returns how many were written.
 */
int v90_dil_measure_usable_ucodes(const v90_dil_measurement_t *m,
                                  uint8_t *out, int max_out);

#endif
