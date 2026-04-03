/*
 * v92_ja_decode.h — V.92 Phase 3 Ja / DIL descriptor decoder
 *
 * Provides a modular interface for searching the V.90 PCM codeword stream
 * for the Ja DIL (Downstream Initialisation List) descriptor.  The search
 * logic is decoupled from vpcm_decode.c so it can be called from the
 * real-time modem engine as well as the offline decoder.
 *
 * ITU-T V.92 §8.5.1.3, V.90 §5.4 (DIL descriptor structure).
 */

#ifndef V92_JA_DECODE_H
#define V92_JA_DECODE_H

#include <stdbool.h>
#include <stdint.h>

#include "v90.h"   /* v90_dil_desc_t, v90_dil_analysis_t */

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ja_dil_search_params_t — caller-supplied context for a DIL search.
 *
 * The caller is responsible for computing the search window from whatever
 * timing information it has (INFO1a, Jd stage result, phase3/phase4 markers,
 * etc.).  Separating this from the search loop lets the same function be used
 * by vpcm_decode, modem_engine, or any future offline analyser.
 */
typedef struct {
    /* Codeword index range to scan (inclusive on both ends). */
    int search_start;

    /*
     * Last codeword index to attempt as a DIL start.  The search requires
     * at least 206 bits of lookahead, so the caller must ensure
     * search_end <= total_codewords - 206.
     */
    int search_end;

    /*
     * Known or estimated sample index of the Ja signal onset.  When >= 0
     * the scoring function penalises candidates that are far from this
     * anchor.  Set to -1 when the Ja anchor is unknown.
     */
    int tx_ja_sample;

    /*
     * U_INFO value from INFO1a (number of PCM levels, range 10–127).
     * Used to annotate the result; pass 0 when not known.
     */
    int u_info;

    /* Which side of the connection we are decoding. */
    bool calling_party;
} ja_dil_search_params_t;

/*
 * ja_dil_decode_t — result of a successful DIL search.
 *
 * Previously defined as a private type inside vpcm_decode.c; now public so
 * any module can inspect the decoded descriptor without coupling to the
 * offline-decode internals.
 */
typedef struct {
    /* True when a plausible DIL descriptor was found. */
    bool ok;

    bool calling_party;

    /* U_INFO from the search params (copied for caller convenience). */
    int u_info;

    /*
     * Codeword index at which the DIL descriptor starts (i.e. the index
     * whose sign bit is the first bit of the descriptor after descrambling).
     */
    int start_sample;

    /*
     * True when the best candidate required polarity inversion relative to
     * the raw codeword sign bits.
     */
    bool invert_sign;

    /* Parsed descriptor and quality analysis. */
    v90_dil_desc_t     desc;
    v90_dil_analysis_t analysis;
} ja_dil_decode_t;

/*
 * v92_ja_dil_search() — search a V.90 PCM codeword stream for the Ja DIL
 * descriptor.
 *
 * For each candidate start position in [params->search_start,
 * params->search_end] and for both sign polarities, the function:
 *   1. Extracts bits from the codeword stream using the V.90 differential
 *      sign decoder and the x^23+x^5+1 descrambler.
 *   2. Attempts to parse a DIL descriptor (v90_parse_dil_descriptor).
 *   3. Analyses descriptor quality (v90_analyse_dil_descriptor).
 *   4. Scores the candidate; keeps the best.
 *
 * Scoring (higher is better):
 *   +100 × unique_train_u   (diverse training U-chord set)
 *   + 50 × used_uchords     (number of distinct U-chords actually used)
 *   - 10 × impairment_score (penalty for out-of-spec entries)
 *   -  5 × non_default_h   (penalty for non-default H vector)
 *   -  distance / 8         (penalty for distance from tx_ja_sample anchor)
 *
 * Returns true and fills *out on success; returns false when no candidate
 * passes the parse/analysis checks.
 */
bool v92_ja_dil_search(const uint8_t *codewords,
                       int total_codewords,
                       const ja_dil_search_params_t *params,
                       ja_dil_decode_t *out);

#ifdef __cplusplus
}
#endif

#endif /* V92_JA_DECODE_H */
