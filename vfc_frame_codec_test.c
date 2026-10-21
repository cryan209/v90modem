/* Native c88a mapping and its inverse, including arbitrary 128-bit ring wraps.
 * RX expectations come from native TX vectors, not our own encoder output. */
#include "vfc_frame.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static unsigned nibble(char c)
{
    assert((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'));
    return c <= '9' ? c - '0' : c - 'a' + 10;
}

static void unpack(const char *text, uint8_t *out, unsigned n)
{
    assert(strlen(text) == 2*n);
    for (unsigned i = 0; i < n; i++) out[i] = 16*nibble(text[2*i]) + nibble(text[2*i+1]);
}

int main(void)
{
    FILE *f = fopen("reference-data/vfc/frames.csv", "r");
    assert(f);
    char line[512];
    unsigned cases = 0;
    vfc_frame_plan_t tx, rx;
    while (fgets(line, sizeof(line), f)) {
        if (line[0] == '#') continue;
        unsigned baud, rate, extra, number, cursor, end, bits;
        char raw_hex[33], direct_hex[17], ring_hex[17];
        assert(sscanf(line, "%u,%u,%u,%u,%u,%u,%u,%32[0-9a-f],%16[0-9a-f],%16[0-9a-f]",
                      &baud, &rate, &extra, &number, &cursor, &end, &bits,
                      raw_hex, direct_hex, ring_hex) == 10);
        uint8_t raw[16];
        vfc_symbol_frame_t expected = {.bits = bits, .next_cursor = end}, actual;
        unpack(raw_hex, raw, 16);
        unpack(direct_hex, expected.direct, 8);
        unpack(ring_hex, expected.ring, 8);
        if (!number) {
            assert(vfc_frame_plan(&tx, baud, rate, extra));
            rx = tx;
        }
        assert(vfc_frame_map(&tx, raw, cursor, &actual));
        if (memcmp(expected.direct, actual.direct, 8) || memcmp(expected.ring, actual.ring, 8)
            || actual.bits != bits || actual.next_cursor != end) {
            fprintf(stderr, "native V.FC frame mismatch: %s", line);
            fprintf(stderr, "actual rings:");
            for (unsigned i = 0; i < 8; i++) fprintf(stderr, " %u", actual.ring[i]);
            fputc('\n', stderr);
            return 1;
        }
        uint8_t output[16];
        memset(output, 0xa5, sizeof(output));
        assert(vfc_frame_unmap(&rx, &expected, cursor, output));
        for (unsigned bit = 0; bit < 128; bit++) {
            unsigned wanted = ((bit + 128 - cursor) & 127) < bits
                            ? (raw[bit >> 3] >> (bit & 7)) & 1
                            : (0xa5 >> (bit & 7)) & 1;
            assert(((output[bit >> 3] >> (bit & 7)) & 1) == wanted);
        }
        assert(rx.schedule == tx.schedule);
        cases++;
    }
    assert(!ferror(f));
    fclose(f);
    assert(cases == 312);
    assert(vfc_frame_plan(&tx, 5, 28800, false));
    rx = tx;
    uint8_t raw[16] = {0}, output[16], saved[16];
    vfc_symbol_frame_t bad;
    assert(vfc_frame_map(&tx, raw, 0, &bad));
    bad.ring[0] = rx.max_ring + 1;
    memset(output, 0xa5, sizeof(output));
    memcpy(saved, output, sizeof(saved));
    unsigned phase = rx.schedule;
    assert(!vfc_frame_unmap(&rx, &bad, 0, output));
    assert(rx.schedule == phase && !memcmp(output, saved, sizeof(saved)));
    printf("PASS: V.FC frame TX and RX match %u independent native frames\n", cases);
    return 0;
}
