/*
 * v92_anspcm_decode.c — Table-based ANSpcm reverse decoder (V.92 §8.3.1)
 *
 * Reference: ITU-T V.92 (11/2000), §8.3.1 and Tables 6–10.
 *
 * The 301-symbol ANSpcm period is generated from:
 *   x[k] = floor( scl × √2 × cos(2π × k × 79/301 + θ) + 0.5 )
 *   then G.711-encoded per the law (µ or A).
 *
 * θ = 0.25π/301, scl per Table 6.  A phase reversal (XOR 0x80 in codeword
 * space) is applied every 3612 symbols (12 periods).
 */

#include "v92_anspcm_decode.h"

#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>

/* ------------------------------------------------------------------ */
/* Reference table generation                                          */
/* ------------------------------------------------------------------ */

/*
 * Table 6/V.92 — scl values per law and level.
 * Columns: [µ-law scl], [A-law scl].
 */
static const int anspcm_scl[V92_ANSPCM_NUM_LEVELS][2] = {
    { 1334, 667  },   /* level 0: −9.5 dBm0 */
    { 1000, 500  },   /* level 1: −12  dBm0 */
    {  708, 354  },   /* level 2: −15  dBm0 */
    {  500, 250  },   /* level 3: −18  dBm0 */
};

static const double ANSPCM_THETA = 0.25 * M_PI / (double)V92_ANSPCM_PERIOD;

/*
 * Precomputed spec tables: ref[law][level][k], law 0=µ 1=A.
 * Built once on first call via anspcm_build_ref().
 */
static uint8_t   ref_table[2][V92_ANSPCM_NUM_LEVELS][V92_ANSPCM_PERIOD];
static bool      ref_ready = false;

/*
 * Inverted index for fast pre-filtering.
 *
 * For each of the 256 possible byte values, we record every (law, level,
 * phase) triple where that byte appears in a reference table.  This
 * allows the outer scan to skip any offset whose first byte does not
 * appear anywhere in the combined reference space.
 *
 * An entry also stores the LSB-toggled variant so robbed-bit hits get
 * the same pre-filter benefit.
 */
#define MAX_REFS_PER_BYTE  96   /* generous upper bound: 8 tables × 12 avg */

typedef struct {
    uint8_t law;
    uint8_t level;
    uint16_t phase;
    bool lsb_toggled;  /* true if this entry came from byte ^ 1 */
} anspcm_ref_entry_t;

static anspcm_ref_entry_t byte_refs[256][MAX_REFS_PER_BYTE];
static int                byte_ref_count[256];

static void anspcm_build_ref(void)
{
    if (ref_ready)
        return;

    memset(byte_refs,      0, sizeof(byte_refs));
    memset(byte_ref_count, 0, sizeof(byte_ref_count));

    for (int lv = 0; lv < V92_ANSPCM_NUM_LEVELS; lv++) {
        for (int k = 0; k < V92_ANSPCM_PERIOD; k++) {
            double angle = 2.0 * M_PI * (double)k * 79.0
                           / (double)V92_ANSPCM_PERIOD + ANSPCM_THETA;

            for (int law = 0; law < 2; law++) {
                int    scl = anspcm_scl[lv][law];
                double x   = (double)scl * M_SQRT2 * cos(angle);
                int    lin = (int)floor(x + 0.5);

                /* Clamp to 16-bit signed range */
                if (lin >  32767) lin =  32767;
                if (lin < -32768) lin = -32768;

                uint8_t cw = v91_linear_to_codeword((v91_law_t)law, (int16_t)lin);
                ref_table[law][lv][k] = cw;

                /* Insert into inverted index for exact match */
                int ci = byte_ref_count[cw];
                if (ci < MAX_REFS_PER_BYTE) {
                    byte_refs[cw][ci].law         = (uint8_t)law;
                    byte_refs[cw][ci].level        = (uint8_t)lv;
                    byte_refs[cw][ci].phase        = (uint16_t)k;
                    byte_refs[cw][ci].lsb_toggled  = false;
                    byte_ref_count[cw]++;
                }

                /* Insert LSB-toggled variant (robbed-bit) */
                uint8_t cw_rob = cw ^ 1;
                int ci2 = byte_ref_count[cw_rob];
                if (ci2 < MAX_REFS_PER_BYTE) {
                    byte_refs[cw_rob][ci2].law        = (uint8_t)law;
                    byte_refs[cw_rob][ci2].level       = (uint8_t)lv;
                    byte_refs[cw_rob][ci2].phase       = (uint16_t)k;
                    byte_refs[cw_rob][ci2].lsb_toggled = true;
                    byte_ref_count[cw_rob]++;
                }
            }
        }
    }

    ref_ready = true;
}

/* ------------------------------------------------------------------ */
/* Scoring                                                             */
/* ------------------------------------------------------------------ */

/*
 * Score points per symbol outcome (tuned for sensitivity vs. specificity).
 */
#define SCORE_EXACT    20   /* codeword matches perfectly */
#define SCORE_ROBBED    8   /* matches with LSB flipped (T1 robbed bit) */
#define SCORE_MISMATCH (-12)

/*
 * Quick scan: check the first QUICK_LEN symbols of a candidate.
 * Returns approximate score; used to skip poor candidates before
 * committing to a full evaluation.
 */
#define QUICK_LEN  18
#define QUICK_PASS 12   /* must score QUICK_PASS × SCORE_EXACT equivalent to proceed */

static int quick_score(const uint8_t *codewords, int total,
                       int start, int period_phase,
                       int law, int level,
                       int reversal_offset)
{
    const uint8_t *ref = ref_table[law][level];
    int score = 0;
    int len = QUICK_LEN;
    if (start + len > total)
        len = total - start;

    for (int i = 0; i < len; i++) {
        uint8_t actual = codewords[start + i];
        int k = (period_phase + i) % V92_ANSPCM_PERIOD;
        /* Phase reversal: every V92_ANSPCM_REVERSAL_LEN symbols from the
         * absolute ANSpcm start.  reversal_offset = how far into the
         * stream our start position is. */
        int abs_i = reversal_offset + i;
        uint8_t expected = ref[k];
        if ((abs_i / V92_ANSPCM_REVERSAL_LEN) & 1)
            expected ^= 0x80;

        if (actual == expected)
            score += SCORE_EXACT;
        else if ((actual ^ 1) == expected || actual == (expected ^ 1))
            score += SCORE_ROBBED;
        else
            score += SCORE_MISMATCH;
    }
    return score;
}

/*
 * Full evaluation: score eval_len symbols and collect per-category counts.
 * Also discovers the duration of the run (how long the match sustains).
 */
static int full_score(const uint8_t *codewords, int total,
                      int start, int period_phase,
                      int law, int level,
                      int reversal_offset,
                      int eval_len,
                      int *exact_out, int *robbed_out, int *mismatch_out,
                      int *duration_out, int *reversals_out)
{
    const uint8_t *ref = ref_table[law][level];
    int score    = 0;
    int exact    = 0;
    int robbed   = 0;
    int mismatch = 0;
    int rev_count = 0;
    int last_good = -1;

    if (start + eval_len > total)
        eval_len = total - start;

    int prev_reversal_block = -1;

    for (int i = 0; i < eval_len; i++) {
        uint8_t actual = codewords[start + i];
        int k = (period_phase + i) % V92_ANSPCM_PERIOD;
        int abs_i = reversal_offset + i;
        int reversal_block = abs_i / V92_ANSPCM_REVERSAL_LEN;
        uint8_t expected = ref[k];
        if (reversal_block & 1)
            expected ^= 0x80;

        if (reversal_block != prev_reversal_block) {
            if (prev_reversal_block >= 0)
                rev_count++;
            prev_reversal_block = reversal_block;
        }

        int sym_score;
        if (actual == expected) {
            sym_score = SCORE_EXACT;
            exact++;
            last_good = i;
        } else if ((actual ^ 1) == expected || actual == (expected ^ 1)) {
            sym_score = SCORE_ROBBED;
            robbed++;
            last_good = i;
        } else {
            sym_score = SCORE_MISMATCH;
            mismatch++;
        }
        score += sym_score;

        /* Early termination: if the per-symbol average drops below a
         * threshold, this is probably not ANSpcm any more. */
        if (i >= V92_ANSPCM_PERIOD - 1 && i % V92_ANSPCM_PERIOD == (V92_ANSPCM_PERIOD - 1)) {
            int period_score = score;    /* accumulated so far */
            int threshold    = i * 2;   /* expect at least 2 pts/symbol average */
            if (period_score < threshold)
                break;
        }
    }

    if (exact_out)     *exact_out     = exact;
    if (robbed_out)    *robbed_out    = robbed;
    if (mismatch_out)  *mismatch_out  = mismatch;
    if (duration_out)  *duration_out  = (last_good >= 0) ? last_good + 1 : 0;
    if (reversals_out) *reversals_out = rev_count;
    return score;
}

/* ------------------------------------------------------------------ */
/* Main decoder                                                         */
/* ------------------------------------------------------------------ */

bool v92_anspcm_table_decode(const uint8_t   *codewords,
                             int              total,
                             v91_law_t        assumed_law,
                             int              search_start,
                             int              search_end,
                             v92_anspcm_table_hit_t *out)
{
    anspcm_build_ref();

    if (!codewords || total <= 0 || !out)
        return false;

    if (search_start < 0)
        search_start = 0;
    if (search_end <= 0 || search_end > total)
        search_end = total;

    int min_symbols = V92_ANSPCM_PERIOD * 2;   /* need 2+ full periods */
    int eval_len    = V92_ANSPCM_PERIOD * 6;   /* score up to 6 periods */

    if (search_end - search_start < min_symbols)
        return false;

    v92_anspcm_table_hit_t best;
    memset(&best, 0, sizeof(best));
    int best_score = INT_MIN;

    for (int start = search_start; start + min_symbols <= search_end; start++) {
        uint8_t first_byte = codewords[start];
        int n_refs = byte_ref_count[first_byte];
        if (n_refs == 0)
            continue;

        for (int ri = 0; ri < n_refs; ri++) {
            const anspcm_ref_entry_t *e = &byte_refs[first_byte][ri];
            int law   = e->law;
            int level = e->level;
            int pp    = e->phase;    /* period_phase: 0-300 */

            /*
             * If the first byte matched via the robbed-bit index entry
             * (lsb_toggled), we test two hypotheses:
             *   a) reversal is NOT in effect at this point (expected = ref[pp])
             *   b) reversal IS in effect (expected = ref[pp] ^ 0x80)
             * Both are tested; only the hypothesis consistent with the
             * expected codeword being first_byte (or first_byte^1) is kept.
             */
            bool ok_no_rev  = (ref_table[law][level][pp] == first_byte)
                              || ((ref_table[law][level][pp] ^ 1) == first_byte);
            bool ok_rev     = ((ref_table[law][level][pp] ^ 0x80) == first_byte)
                              || (((ref_table[law][level][pp] ^ 0x80) ^ 1) == first_byte);

            /* Number of reversal-offset hypotheses to try */
            int n_hyp = 0;
            int hyp_rev_offset[2];
            if (ok_no_rev) hyp_rev_offset[n_hyp++] = pp;
            /*
             * Second hypothesis: we're in the first reversed block at this
             * symbol.  reversal_offset = V92_ANSPCM_REVERSAL_LEN - (remaining
             * in block) but since we don't know exact position just try both
             * halves of the first reversal boundary.
             */
            if (ok_rev)    hyp_rev_offset[n_hyp++] = pp + V92_ANSPCM_REVERSAL_LEN;

            for (int h = 0; h < n_hyp; h++) {
                int rev_off = hyp_rev_offset[h];

                /* Quick filter */
                int qs = quick_score(codewords, total, start, pp,
                                     law, level, rev_off);
                if (qs < QUICK_PASS * SCORE_EXACT / 2)
                    continue;

                /* Full evaluation */
                int exact, robbed, mismatch, duration, reversals;
                int sc = full_score(codewords, total, start, pp,
                                    law, level, rev_off, eval_len,
                                    &exact, &robbed, &mismatch,
                                    &duration, &reversals);

                if (duration < min_symbols)
                    continue;

                /* Penalise heavy mismatch rate */
                int total_scored = exact + robbed + mismatch;
                if (total_scored > 0) {
                    int miss_pct = mismatch * 100 / total_scored;
                    if (miss_pct > 40)
                        continue;
                }

                if (sc > best_score) {
                    best_score             = sc;
                    best.seen              = true;
                    best.start_sample      = start;
                    best.duration_symbols  = duration;
                    best.period_phase      = pp;
                    best.law               = (v91_law_t)law;
                    best.wrong_law         = ((v91_law_t)law != assumed_law);
                    best.level             = level;
                    best.exact_matches     = exact;
                    best.robbed_bit_count  = robbed;
                    best.mismatches        = mismatch;
                    best.phase_reversals   = reversals;
                    best.score             = sc;
                }
            }
        }
    }

    if (!best.seen)
        return false;

    *out = best;
    return true;
}

/* ------------------------------------------------------------------ */
/* Utilities                                                           */
/* ------------------------------------------------------------------ */

const char *v92_anspcm_table_level_str(int level)
{
    switch (level) {
    case 0: return "-9.5 dBm0";
    case 1: return "-12 dBm0";
    case 2: return "-15 dBm0";
    case 3: return "-18 dBm0";
    default: return "unknown";
    }
}
