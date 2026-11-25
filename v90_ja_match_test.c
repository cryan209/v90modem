/* The Ja detector's existing 384-bit/115-error engineering budget must be
 * unchanged by its faster comparison (V.90 9.3.1.3 receive-side timing).
 * Check the acceptance boundary, every error count, all alignments and
 * word tails. No modem samples or ITU signal constants are changed here.
 */
#include "v90_ja_match.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    uint8_t storage[384 + 8];
    uint8_t reference[384];
    const size_t lengths[] = {1, 7, 8, 9, 17, 383, 384};

    for (size_t row = 0; row < sizeof(lengths)/sizeof(lengths[0]); row++) {
        size_t n = lengths[row];
        int budget = n < 115 ? (int)n/2 : 115;

        for (int alignment = 0; alignment < 8; alignment++) {
            uint8_t *bits = storage + alignment;

            for (size_t i = 0; i < n; i++)
                bits[i] = reference[i] = (uint8_t)((17*i + i/13) & 1);
            for (size_t errors = 0; errors <= n; errors++) {
                int got = v90_ja_mismatches(bits, reference, n, budget);

                assert((got <= budget) == (errors <= (size_t)budget));
                if (errors <= (size_t)budget)
                    assert(got == (int)errors);
                /* Spread errors from both ends across word boundaries. */
                if (errors < n)
                    bits[(errors & 1) ? n - 1 - errors/2 : errors/2] ^= 1;
            }
        }
    }
    /* Table 12 header check, frame bits 18:50.  CX93001: N=119 Lsp=124
     * Ltp=126; HSF (D-Modem rig, 2026-11-25): N=147 Lsp=126 Ltp=126, bits
     * as received on hypothesis 8. */
    {
        static const char *cx  = "111011100000000001101111010111110";
        static const char *hsf = "110010010000000001011111010111110";
        uint8_t c[33], h[33];

        for (int k = 0; k < 33; k++) {
            c[k] = (uint8_t)(cx[k] - '0');
            h[k] = (uint8_t)(hsf[k] - '0');
        }
        assert(!v90_ja_header_names_other_descriptor(c, c));
        assert(v90_ja_header_names_other_descriptor(h, c));
        /* Two field-bit errors on the template's own header still match. */
        c[0] ^= 1; c[20] ^= 1;
        {
            uint8_t t[33];

            for (int k = 0; k < 33; k++)
                t[k] = (uint8_t)(cx[k] - '0');
            assert(!v90_ja_header_names_other_descriptor(c, t));
            /* A set fixed-zero bit (frame bit 30) makes the header noisy:
             * no verdict, even with HSF's fields. */
            h[12] = 1;
            assert(!v90_ja_header_names_other_descriptor(h, t));
        }
    }
    puts("v90_ja_match_test: PASSED");
    return 0;
}
