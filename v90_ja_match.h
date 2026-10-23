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

#endif
