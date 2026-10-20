/* See k56flex_upstream.h: equalized-symbol control decoder, not PCM RX. */
#include "k56flex_upstream.h"

#include <math.h>
#include <string.h>

int k56flex_upstream_init(k56flex_upstream_t *rx, unsigned tap)
{
    if (tap != 5 && tap != 18) return -1;
    memset(rx, 0, sizeof(*rx));
    k56flex_report_rx_init(&rx->report, tap);
    return 0;
}

int k56flex_upstream_symbol(k56flex_upstream_t *rx, float real, float imag)
{
    /* Resident PM 0294 labels (++,-+,+-,--) as (0,3,1,2). PM 02B0
     * maps (previous,current) to (current - previous) mod 4. Overlay
     * 8E's labels/differential table are different (Draft 0.23 clause 11). */
    static const unsigned labels[4] = {0, 3, 1, 2};
    unsigned raw, dibit, i;
    int events = 0;
    if (!isfinite(real) || !isfinite(imag) || real == 0 || imag == 0)
        return -1;
    if (rx->ended) return 0;
    raw = labels[(real < 0) + 2 * (imag < 0)];
    dibit = (raw - rx->previous_raw) & 3;
    rx->previous_raw = raw;
    ++rx->symbols;
    for (i = 0; i < 2; ++i) {
        unsigned y = (dibit >> i) & 1;
        unsigned x = y ^ ((rx->report.hist >> (rx->report.tap - 1)) & 1)
                       ^ ((rx->report.hist >> 22) & 1);
        int accepted = rx->report.accepted;
        k56flex_report_rx_bit(&rx->report, (int)y);
        if (!accepted && rx->report.accepted) {
            events |= K56U_REPORT;
            /* Report acquisition rotates [report16, extension8] to
             * [extension8, report16]. Recover that bit phase so an isolated
             * 899F inside other data cannot terminate the exchange. */
            unsigned win = sizeof(rx->report.window);
            uint32_t want = ((uint32_t)rx->report.report << 8) | rx->report.ext;
            for (unsigned k = 0; k < 24; k++) {
                uint32_t word = 0;
                for (unsigned b = 0; b < 24; b++)
                    word |= (uint32_t)rx->report.window[(rx->report.pos + win - 24 + (k+b)%24)%win] << b;
                if (word == want) { rx->end_phase = k; break; }
            }
        }
        if (!accepted) continue;
        /* Draft 0.23 11 / resident 153D/1676/1DDC: J'a follows Ja.
         * Original HSF 120342c0 emits just eight dibits (one 899F), then
         * resets for TRN. Keep continuous descrambler/differential history
         * through that word; require its recovered packet boundary. */
        rx->end_window = (rx->end_window >> 1) | ((uint64_t)x << 15);
        ++rx->end_bits;
        if (rx->end_bits >= 16 && rx->end_bits%24 == rx->end_phase
            && rx->end_window == UINT64_C(0x899f)) {
            rx->ended = 1;
            events |= K56U_REPORT_END;
        }
    }
    return events;
}
