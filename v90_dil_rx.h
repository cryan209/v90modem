/*
 * v90_dil_rx.h — Offline V.90 DIL waveform decoder (§8.4.1).
 *
 * Recovers a DIL descriptor from a raw G.711 codeword stream captured on
 * the DS0 side. The decoder is self-syncing: it needs no Phase 3 anchor.
 * It finds the repeating DIL cycle, segments it, and reconstructs a
 * descriptor whose §8.4.1 expansion reproduces the observed codewords.
 *
 * The recovered descriptor is canonical: N is the minimal segment cycle
 * and SP/TP are solved at their minimal period, then expanded to the
 * longest observed segment (their full observable extent) when that fits
 * in 128 bits. It may still differ textually from the descriptor the
 * analogue modem requested while remaining waveform-equivalent (the
 * `exact` flag asserts byte equality of the §8.4.1 expansion against
 * the capture).
 */
#ifndef V90_DIL_RX_H
#define V90_DIL_RX_H

#include "v90.h"

#include <stdbool.h>
#include <stdint.h>

/* Largest DIL cycle the decoder searches for, in symbols. Covers the
 * common real-modem shapes (e.g. 125 segments x 126 symbols = 15750);
 * the theoretical Table 12 maximum (255 x 774) is far past anything a
 * 15-second Phase 3 could carry twice. */
#define V90_DIL_RX_MAX_CYCLE 30000

typedef struct {
    v90_dil_desc_t desc;    /* recovered descriptor (canonical-minimal) */
    int run_start;          /* first stream index covered by the DIL run */
    int run_len;            /* symbols in the periodic DIL run */
    int first_segment_at;   /* stream index of the first whole DIL-segment */
    int cycle_len;          /* symbols per full descriptor cycle */
    int cycles_seen;        /* whole cycles observed in the run */
    int mismatches;         /* symbols disagreeing with the periodic model */
    bool exact;             /* re-encoded cycle matches the capture exactly */
} v90_dil_rx_result_t;

/*
 * Decode a window that contains only DIL (leading/trailing non-DIL symbols
 * are tolerated by the internal run trimming). Returns true when a
 * descriptor was recovered and its re-encoding matches the observed cycle.
 */
bool v90_dil_rx_decode(const uint8_t *codewords, int len, v90_law_t law,
                       v90_dil_rx_result_t *out);

/*
 * Search an arbitrary codeword stream (e.g. a whole-session DS0 capture)
 * for the longest DIL run and decode it. Returns true on success.
 */
bool v90_dil_rx_scan(const uint8_t *codewords, int len, v90_law_t law,
                     v90_dil_rx_result_t *out);

#endif
