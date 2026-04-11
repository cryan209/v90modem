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
    /* True when a strict parse succeeded. */
    bool ok;
    /* True when the strict parse matched V.92 Table 20 instead of V.90 Table 12. */
    bool is_v92;
    /* Exact parsed descriptor length in bits from the descriptor start. */
    int bit_len;
    /* V.92 downstream rate-capability masks. Zero for V.90 parses. */
    uint16_t rate_mask_lo;
    uint16_t rate_mask_hi;
} v92_ja_parse_meta_t;

typedef struct {
    /* True when a plausible DIL descriptor was found. */
    bool ok;
    /* True when no full descriptor parsed, but best near-lock is available. */
    bool soft_lock;

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
    bool parsed_v92;
    int  descriptor_bits;
    int  soft_score;
    int  soft_sync_hd;
    int  soft_frame17_viol;
    int  soft_zero_viol;
    int  soft_crc_hd;

    /* Parsed descriptor and quality analysis. */
    v90_dil_desc_t     desc;
    v90_dil_analysis_t analysis;
} ja_dil_decode_t;

/*
 * v92_parse_ja_descriptor_strict() — strictly parse a Ja DIL descriptor that
 * begins at bit 0 of a packed bitstream.
 *
 * The function accepts:
 *   - V.90 Table 12 descriptors with exact CRC
 *   - V.92 Table 20 descriptors with exact CRC and zero-fill to the next
 *     multiple of 12 bits
 *
 * It does not perform search, polarity selection, or soft recovery scoring.
 */
bool v92_parse_ja_descriptor_strict(v90_dil_desc_t *out_desc,
                                    const uint8_t *bits,
                                    int bit_count,
                                    v92_ja_parse_meta_t *meta);

/*
 * v92_ja_dil_search() — search a V.90 PCM codeword stream for the Ja DIL
 * descriptor.
 *
 * For each candidate start position in [params->search_start,
 * params->search_end] and for both sign polarities, the function:
 *   1. Extracts bits from the codeword stream using the V.90 differential
 *      sign decoder and the x^23+x^5+1 descrambler.
 *   2. Attempts to parse a DIL descriptor with strict V.90/V.92 rules.
 *   3. Analyses descriptor quality (v90_analyse_dil_descriptor).
 *   4. Scores the candidate; keeps the best.
 *
 * Scoring (higher is better):
 *   +100 × unique_train_u   (diverse training U-chord set)
 *   + 50 × used_uchords     (number of distinct U-chords actually used)
 *   - 10 × impairment_score (penalty for out-of-spec entries)
 *   -  5 × non_default_h   (penalty for non-default H vector)
 *   -  6 × frame17_violations (expected 17-bit framing start-zero rhythm)
 *   +  4 × frame17_best_run   (longest contiguous start-zero run)
 *   - 12 × sync_hamming18     (distance from 17 ones + start-zero sync)
 *   + 30 - 2*|fs12_pos-4| when 12-bit FS appears near expected placement
 *   - 15 when no FS12 pattern is found
 *   -  5 × reserved_zero_violations
 *   -  4 × crc_nearness_hd
 *   -  distance / 8         (penalty for distance from tx_ja_sample anchor)
 *
 * Returns true and fills *out on hard success (ok=true) or on soft near-lock
 * (ok=false, soft_lock=true). Returns false only when no viable candidate
 * exists in the window.
 */
bool v92_ja_dil_search(const uint8_t *codewords,
                       int total_codewords,
                       const ja_dil_search_params_t *params,
                       ja_dil_decode_t *out);

#ifdef __cplusplus
}
#endif

#endif /* V92_JA_DECODE_H */
