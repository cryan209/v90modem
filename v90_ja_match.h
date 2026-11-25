/* Exact Hamming budget for the byte-per-bit Ja capture (V.90 9.3.1.3).
 * Inputs must contain only 0 or 1, as produced by v34_ja_window_get().
 * XOR then popcount compares eight such bits at once, independent of byte
 * order. memcpy permits unaligned sliding windows without aliasing UB.
 * A rejected window may return any count above budget; accepted counts
 * are exact. The caller retains its existing threshold and search order.
 */
#ifndef V90_JA_MATCH_H
#define V90_JA_MATCH_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

static inline int v90_ja_mismatches(const uint8_t *a, const uint8_t *b,
                                    size_t count, int budget)
{
    size_t k = 0;
    int errors = 0;

    for (; k + sizeof(uint64_t) <= count; k += sizeof(uint64_t)) {
        uint64_t x, y;

        memcpy(&x, a + k, sizeof(x));
        memcpy(&y, b + k, sizeof(y));
        errors += __builtin_popcountll(x ^ y);
        if (errors > budget)
            return errors;
    }
    for (; k < count; k++) {
        errors += a[k] != b[k];
        if (errors > budget)
            return errors;
    }
    return errors;
}

/* Does a window aligned at Table 12/V.90 bit 18 carry a well-formed header
 * that names a DIFFERENT descriptor from the template's?
 *
 * Table 12: N at bits 18:25, zeros at 26:33, start bit 34, Lsp-1 at 35:41,
 * start bit 42, Ltp-1 at 43:49, start bit 50.  The frame body after the
 * header (SP/TP patterns, h, ref, the first training Ucodes) is near-common
 * to every vendor: the HSF's Ja agrees with the CX93001's at 77-78% over
 * frame bits 146:529, which is past the 70% a template match needs, while
 * its N and Lsp (147/126 against 119/124) differ in 5 of 22 field bits.  So
 * a match whose 11 fixed-zero bits all read zero (a clean header) and whose
 * N/Lsp/Ltp fields differ from the template's in at least
 * V90_JA_HEADER_MIN_FIELD_DIFF bits is another peer's Ja, not the
 * template's.  A noisy header (any of its zero bits set) decides nothing.
 * 'w' and 'tpl' both start at frame bit 18 and hold at least 33 bits. */
#define V90_JA_HEADER_BITS            33
#define V90_JA_HEADER_MIN_FIELD_DIFF  3

static inline int v90_ja_header_names_other_descriptor(const uint8_t *w,
                                                       const uint8_t *tpl)
{
    int field_diff = 0;

    for (int k = 0; k < V90_JA_HEADER_BITS; k++) {
        int frame_bit = 18 + k;
        int fixed_zero = (frame_bit >= 26 && frame_bit <= 34)
                         || frame_bit == 42 || frame_bit == 50;

        if (fixed_zero) {
            if (w[k])
                return 0;
        } else if (w[k] != tpl[k]) {
            field_diff++;
        }
    }
    return field_diff >= V90_JA_HEADER_MIN_FIELD_DIFF;
}

#endif
