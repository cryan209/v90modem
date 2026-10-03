/*
 * v92_rsig.h — the R signals of V.92 rate renegotiation and fast parameter
 * exchange, on the PCM side.
 *
 *   Rd, Rt   V.90 8.6.4 (V.92 8.8.4): period 6, signs + + + - - -
 *   Ru       V.92 8.5.5: period 6 on +/-LU, upstream
 *   Rf       V.92 8.8.4: period 4, signs + + - -, each symbol the highest
 *            power codeword of its data frame interval's data-mode
 *            constellation as passed in CPu; R-bar-f is - - + +, 24T
 *   RM, RM'  V.92 8.7.4: data-mode Ki patterns, see v92_upstream_data.h
 *
 * Every bar is its signal shifted by half a period, so the detector here
 * locks a sign pattern of period 6 or 4, reports which, and reports the bar
 * as a sustained half-period shift of that lock -- which is the event
 * v92_rn.h's V92_RN_EV_RBAR needs.  It works on signs alone, so it is blind
 * to level and to which Ucodes carry them, and therefore cannot tell Rd
 * from Rt from Ru: the procedure state says which one is expected.
 */
#ifndef V92_RSIG_H
#define V92_RSIG_H

#include <stdbool.h>
#include <stdint.h>

#include "vpcm_cp.h"

/* 8.8.4: the six Ucodes Rf uses, one per data frame interval -- the highest
 * Ucode in that interval's constellation (masks[dfi[i]]) of the CPu. */
bool v92_rf_ucodes(const vpcm_cp_frame_t *cpu, uint8_t ucodes[6]);

/* Rf (bar = false) or R-bar-f symbols, starting at symbol `pos` of the
 * signal (which begins on a data frame boundary, so pos % 6 is the data
 * frame interval).  law: 0 = mu-law, 1 = A-law (v91_law_t). */
void v92_rf_codewords(int law, const uint8_t ucodes[6], bool bar,
                      int pos, uint8_t *out, int n);

typedef enum { V92_RSIG_NONE, V92_RSIG_P6, V92_RSIG_P4 } v92_rsig_kind_t;
typedef enum { V92_RSIG_EV_NONE, V92_RSIG_EV_R, V92_RSIG_EV_BAR } v92_rsig_event_t;

typedef struct {
    int threshold;                    /* |sample| below this has no sign */
    int lock_symbols;                 /* consecutive matches to report R */
    int bar_symbols;                  /* consecutive shifted matches for the bar */
    unsigned run4[4], run6[6];        /* consecutive matches per phase */
    v92_rsig_kind_t kind;             /* locked pattern, NONE until R */
    int phase;                        /* its phase */
    bool bar_seen;
    uint64_t symbols;
    uint64_t r_at, bar_at;            /* symbol index of each event */
} v92_rsig_rx_t;

void v92_rsig_rx_init(v92_rsig_rx_t *r, int threshold);
/* One received linear sample per symbol. */
v92_rsig_event_t v92_rsig_rx_put(v92_rsig_rx_t *r, int sample);

#endif
