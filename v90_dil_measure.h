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
#include "vpcm_cp.h"

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

    /*
     * Per data-frame interval, because that is the resolution the answer is
     * needed at.  Robbed-bit signalling alters one DS0 phase in six, so a
     * Ucode can arrive clean in five intervals and collapse onto its
     * neighbour in the sixth -- which is exactly why §5.4.3 makes Mi
     * per-interval rather than one number for the frame.
     */
    int      rx_ucode_slot[6];  /* dominant received Ucode, -1 if unseen */
    int      rx_level_slot[6];  /* mean received |level| in that interval */
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

/*
 * The constellation and rate the measured channel will carry.
 *
 * Ci is built per data-frame interval from what actually arrived there, Mi is
 * its population, and §5.4.3's gate -- the modulus encoder needs prod(Mi) >=
 * 2^K -- picks the largest drn the line supports.  Rate follows from drn
 * (§5.4.1: D = drn + 20 bits per 6-symbol frame, so bps = D * 8000/6).
 *
 * `level_margin` is the smallest difference in received level, in linear
 * units, at which two Ucodes still count as distinguishable.  Zero means
 * "distinct codes are enough", which is the optimistic bound rather than a
 * noise-aware answer.
 *
 * Not applied here: §8.5.2's constraint on average constellation power, and
 * any noise estimate beyond `level_margin`.  The masks are a starting point
 * for a CP frame, not a finished one.
 */
typedef struct {
    uint8_t mask[6][VPCM_CP_MASK_BYTES];  /* Ci per data-frame interval */
    int     mi[6];                        /* |Ci| */
    double  bits_available;               /* sum of log2(Mi) */
    uint8_t drn;
    double  bps;
    bool    robbed_bit_limited;           /* some interval carries fewer points */

    /*
     * Data-frame intervals the DIL never probed, as a bitmask.
     *
     * A descriptor can be self-consistent and still be useless in an interval:
     * segment lengths are multiples of 6 and TP restarts every segment, so if
     * TP happens to hold no training symbol at any position congruent to
     * interval i, that interval only ever carries REFc and nothing is learned
     * about it.  §8.4.1 does not forbid it, but §5.4.3 needs an Mi for all
     * six.  Reported rather than quietly turned into "Mi = 1", because the
     * fix is a different DIL descriptor, not a lower rate.
     */
    uint8_t intervals_unprobed;
} v90_dil_rate_plan_t;

bool v90_dil_measure_plan_rate(const v90_dil_measurement_t *m,
                               int level_margin,
                               v90_dil_rate_plan_t *out);

#endif
