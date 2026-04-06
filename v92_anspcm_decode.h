/*
 * v92_anspcm_decode.h — Table-based ANSpcm reverse decoder (V.92 §8.3.1)
 *
 * Matches a raw G.711 codeword stream against the spec-defined ANSpcm
 * reference tables (Tables 7–10 / V.92), with tolerance for:
 *
 *   Wrong G.711 law   — tries both µ-law and A-law reference tables; the
 *                        one that matches better reports wrong_law=true if
 *                        it differs from the caller's assumed law.
 *
 *   Robbed-bit lines  — T1 superframe bit-stealing flips the LSB of every
 *                        sixth PCM byte.  A symbol that matches only with
 *                        its LSB toggled is counted as a robbed-bit hit
 *                        and scored lower than an exact match but far better
 *                        than a genuine mismatch.
 *
 *   Phase offset      — ANSpcm repeats every 301 symbols; the receiver may
 *                        enter the stream at any point.  The decoder discovers
 *                        the starting phase within the 301-symbol period.
 *
 *   Phase reversal    — Every 3612 symbols (12 × 301) the reference values
 *                        are negated (sign bit XOR 0x80 in G.711 codeword
 *                        space for both µ-law and A-law).
 */

#ifndef V92_ANSPCM_DECODE_H
#define V92_ANSPCM_DECODE_H

#include "v91.h"

#include <stdbool.h>
#include <stdint.h>

#define V92_ANSPCM_NUM_LEVELS    4
#define V92_ANSPCM_PERIOD       301   /* symbols per period */
#define V92_ANSPCM_REVERSAL_LEN 3612  /* symbols between phase reversals */

/* Result of one ANSpcm table-decode attempt. */
typedef struct {
    bool      seen;

    int       start_sample;       /* index of first matched codeword in input */
    int       duration_symbols;   /* matched run length (symbols) */
    int       period_phase;       /* phase within the 301-symbol period (0–300) */

    v91_law_t law;                /* G.711 law whose table matched best */
    bool      wrong_law;          /* true: matched law differs from assumed_law */
    int       level;              /* 0–3, matching Table 6/V.92 (-9.5/–12/–15/–18 dBm0) */

    int       exact_matches;      /* symbols matching the reference table exactly */
    int       robbed_bit_count;   /* symbols matching only with LSB toggled */
    int       mismatches;         /* symbols not matching even with LSB tolerance */
    int       phase_reversals;    /* number of 3612-symbol reversal blocks detected */

    int       score;              /* composite match quality (higher = better) */
} v92_anspcm_table_hit_t;

/*
 * v92_anspcm_table_decode() — scan [search_start, search_end) for ANSpcm.
 *
 * assumed_law  — the G.711 law the caller expects; used only to set
 *                wrong_law in the result.  Pass V91_LAW_ULAW or
 *                V91_LAW_ALAW; both are always tried.
 *
 * Returns true and fills *out with the best hit found.
 */
bool v92_anspcm_table_decode(const uint8_t   *codewords,
                             int              total,
                             v91_law_t        assumed_law,
                             int              search_start,
                             int              search_end,
                             v92_anspcm_table_hit_t *out);

/* Human-readable level string ("-9.5 dBm0" etc.) */
const char *v92_anspcm_table_level_str(int level);

#endif /* V92_ANSPCM_DECODE_H */
