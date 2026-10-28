/* Resident K56flex report RX at the equalized-symbol boundary. No PCM claim. */
#include "k56flex_upstream.h"
#include "k56flex_upstream_vectors.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static unsigned failures;
#define CHECK(c, ...) do { if (!(c)) { ++failures; printf("FAIL: "); \
    printf(__VA_ARGS__); printf("\n"); } } while (0)

static int feed(k56flex_upstream_t *rx, unsigned signs, unsigned rotation)
{
    float re = (signs & 1) ? -9159 : 9159;
    float im = (signs & 2) ? -9159 : 9159;
    while (rotation--) { float old = re; re = -im; im = old; }
    return k56flex_upstream_symbol(rx, re, im);
}

static void vectors(void)
{
    unsigned tap, rotation, start;
    for (tap = 5; tap <= 18; tap += 13)
        for (rotation = 0; rotation < 4; ++rotation)
            for (start = 0; start < 12; ++start) {
                const uint8_t *v = tap == 5 ? k56u_vector_5 : k56u_vector_18;
                k56flex_upstream_t rx;
                unsigned n, reports = 0, ends = 0;
                CHECK(!k56flex_upstream_init(&rx, tap), "init tap %u", tap);
                /* Either observe TRN (which is NOT differential) or join the
                 * report mid-record, with unknown scrambler/previous point. */
                for (n = start ? 512 + start : 0; n < K56U_VECTOR_SYMBOLS; ++n) {
                    int events = feed(&rx, (v[n / 4] >> (2 * (n % 4))) & 3, rotation);
                    CHECK(events >= 0, "valid fixture point rejected");
                    if (events & K56U_REPORT) {
                        ++reports;
                        CHECK(n >= 512 && n < 896, "report outside report stage at %u", n);
                        CHECK(rx.report.ext == 0x63 && rx.report.report == 0x8990,
                              "tap %u rotation %u start %u: report %02x/%04x",
                              tap, rotation, start, rx.report.ext, rx.report.report);
                        CHECK(k56flex_report_field(rx.report.ext, rx.report.report) == 0x763,
                              "mapper field conversion");
                    }
                    if (events & K56U_REPORT_END) {
                        ++ends;
                        CHECK(n >= 896 + 7 && n < 896 + 9,
                              "terminator at %u (starts at 896)", n);
                    }
                }
                CHECK(reports == 1 && ends == 1, "tap %u rotation %u start %u: %u reports/%u ends",
                      tap, rotation, start, reports, ends);
                CHECK(!feed(&rx, 0, 0), "events must latch after end");
            }
}

/* Independent bit-to-point source for boundary/negative tests. Retained
 * MICA vectors above, not this inverse, establish the quadrant convention. */
typedef struct {
    uint32_t hist;
    unsigned raw, pending, bits, reports, ends;
} source_t;

static void source_bit(source_t *tx, k56flex_upstream_t *rx, unsigned bit)
{
    static const unsigned signs[4] = {0, 2, 3, 1};
    unsigned y = bit ^ ((tx->hist >> (rx->report.tap - 1)) & 1)
                     ^ ((tx->hist >> 22) & 1);
    tx->hist = (tx->hist << 1) | y;
    tx->pending |= y << tx->bits++;
    if (tx->bits == 2) {
        int events;
        tx->raw = (tx->raw + tx->pending) & 3;
        events = feed(rx, signs[tx->raw], 0);
        CHECK(events >= 0, "synthetic point rejected");
        tx->reports += !!(events & K56U_REPORT);
        tx->ends += !!(events & K56U_REPORT_END);
        tx->bits = tx->pending = 0;
    }
}

static void source_word(source_t *tx, k56flex_upstream_t *rx,
                        uint32_t word, unsigned bits)
{
    for (unsigned i = 0; i < bits; ++i)
        source_bit(tx, rx, (word >> i) & 1);
}

static void boundaries(void)
{
    for (unsigned tap = 5; tap <= 18; tap += 13)
    for (unsigned offset = 0; offset < 24; ++offset) {
        k56flex_upstream_t rx, fresh;
        source_t tx = {0};
        k56flex_upstream_init(&rx, tap);
        fresh = rx;
        /* Terminators before a report cannot arm END, even after the
         * self-synchronizing descrambler has settled. */
        for (unsigned i = 0; i < 8; ++i)
            source_word(&tx, &rx, 0x899f, 16);
        for (unsigned i = 0; i < offset; ++i) source_bit(&tx, &rx, 1);
        CHECK(!tx.reports && !tx.ends, "terminator alone acquired report");
        for (unsigned i = 0; i < 8; ++i)
            source_word(&tx, &rx, 0x638990, 24);
        CHECK(tx.reports == 1 && !tx.ends, "report event at bit offset %u", offset);
        CHECK(rx.report.ext == 0x63 && rx.report.report == 0x8990,
              "report content at bit offset %u", offset);
        /* A complete unaligned marker and an aligned corrupt marker must
         * not end the report. Each negative occupies a whole 24-bit record. */
        source_bit(&tx, &rx, 0);
        source_word(&tx, &rx, 0x899f, 16);
        source_word(&tx, &rx, 0, 7);
        CHECK(!tx.ends, "unaligned terminator accepted");
        source_word(&tx, &rx, 0x899e, 16);
        source_word(&tx, &rx, 0, 8);
        CHECK(!tx.ends, "corrupt terminator accepted");
        source_word(&tx, &rx, 0x899f, 15);
        CHECK(!tx.ends, "short terminator accepted");
        if (offset == 0) {
            /* One symbol (two bits) off the boundary, as the MT5634MU
             * sends it: accepted, since it breaks the report's period. */
            k56flex_upstream_t sym = rx;
            source_t stx = tx;
            /* Pad so the marker's last bit lands two bits past a boundary. */
            unsigned pad = (2u + 48u + sym.end_phase - 16u - (sym.end_bits + stx.bits)%24u) % 24u;
            source_word(&stx, &sym, 0, pad);
            source_word(&stx, &sym, 0x899f, 16);
            CHECK(stx.ends == 1, "terminator one symbol off the boundary");
        }
        source_bit(&tx, &rx, 1); /* bit 15 of 899F */
        if (tx.bits) source_bit(&tx, &rx, 0);
        CHECK(tx.reports == 1 && tx.ends == 1, "complete terminator at offset %u", offset);
        for (unsigned i = 0; i < 8; ++i) source_word(&tx, &rx, 0x899f, 16);
        CHECK(tx.reports == 1 && tx.ends == 1, "events must be one-shot");
        k56flex_upstream_init(&rx, tap);
        CHECK(!memcmp(&rx, &fresh, sizeof(rx)), "reset after completed call");
    }
}

static void rejects_and_reset(void)
{
    k56flex_upstream_t rx, before;
    unsigned n, seed = 1234567, events = 0;
    CHECK(!k56flex_upstream_init(&rx, 18), "init");
    before = rx;
    CHECK(k56flex_upstream_init(&rx, 0) < 0 && !memcmp(&rx, &before, sizeof(rx)),
          "invalid tap leaves state untouched");
    CHECK(k56flex_upstream_init(&rx, 32) < 0, "invalid shift rejected");
    CHECK(k56flex_upstream_symbol(&rx, NAN, 1) < 0, "NaN rejected");
    CHECK(k56flex_upstream_symbol(&rx, 1, INFINITY) < 0, "infinity rejected");
    CHECK(k56flex_upstream_symbol(&rx, 0, 1) < 0, "axis point rejected");
    CHECK(!memcmp(&rx, &before, sizeof(rx)), "invalid points preserve state");
    for (n = 0; n < 20000; ++n) events |= (unsigned)feed(&rx, 0, 0);
    CHECK(!events, "constant carrier must not acquire report");
    k56flex_upstream_init(&rx, 18);
    for (n = 0; n < 20000; ++n) {
        seed = seed * 1664525u + 1013904223u;
        events |= (unsigned)feed(&rx, seed >> 30, 0);
    }
    CHECK(!events, "random quadrants must not acquire report");
    k56flex_upstream_init(&rx, 18);
    CHECK(!memcmp(&rx, &before, sizeof(rx)), "per-call reset clears every field");
    /* The wrong scrambler tap is a negative control. */
    for (n = 512; n < 896; ++n)
        events |= (unsigned)feed(&rx, (k56u_vector_5[n/4] >> (2*(n%4))) & 3, 0);
    CHECK(!events, "wrong tap must not acquire report");
}

int main(void)
{
    vectors();
    boundaries();
    rejects_and_reset();
    printf(failures ? "k56flex_upstream_test: %u FAILURES\n" : "k56flex_upstream_test: all passed\n", failures);
    return failures != 0;
}
