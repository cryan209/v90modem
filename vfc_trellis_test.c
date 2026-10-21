/* Continuous original-firmware symbols, independently generated. The input
 * is the native mapper's lattice coordinates, not vfc_mapper_put's output. */
#include "vfc_trellis.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

typedef struct { unsigned index, dibit; float i, q; } native_t;
static native_t streams[2][4096];

static void check(unsigned code, const native_t *stream, float noise)
{
    vfc_trellis_t rx;
    assert(vfc_trellis_init(&rx, code, 96, 0));
    unsigned delivered = 0;
    for (unsigned k = 0; k < 4096; k++) {
        vfc_decision_t out;
        /* Bounded perturbation stays well below the lattice decision margin.
         * This checks metric handling, not a modem/channel SNR claim. */
        float di = noise * ((int)((k*17)%11)-5) / 5;
        float dq = noise * ((int)((k*23)%13)-6) / 6;
        int event = vfc_trellis_put(&rx, stream[k].i+di, stream[k].q+dq, &out);
        assert(event >= 0);
        if (event) {
            if (out.index != stream[delivered].index || out.dibit != stream[delivered].dibit) {
                fprintf(stderr, "V.FC %u-state mismatch at symbol %u: %u/%u expected %u/%u\n",
                        code, delivered, out.index, out.dibit,
                        stream[delivered].index, stream[delivered].dibit);
                assert(0);
            }
            delivered++;
        }
    }
    vfc_decision_t tail[63];
    unsigned count = vfc_trellis_finish(&rx, tail);
    assert(count == 63);
    for (unsigned k = 0; k < count; k++, delivered++)
        assert(tail[k].index == stream[delivered].index && tail[k].dibit == stream[delivered].dibit);
    assert(delivered == 4096);
    assert(vfc_trellis_finish(&rx, tail) == 0);
    assert(vfc_trellis_put(&rx, 0, 0, tail) == -1);

    assert(vfc_trellis_init(&rx, code, 96, 0));
    assert(!rx.received && !rx.decoded);
    assert(vfc_trellis_put(&rx, NAN, 0, tail) == -1);
    assert(vfc_trellis_put(&rx, 0, INFINITY, tail) == -1);
    assert(!rx.received);
    /* A short stream drains only real observations; no synthetic padding. */
    for (unsigned k = 0; k < 8; k++)
        assert(vfc_trellis_put(&rx, stream[k].i, stream[k].q, tail) == 0);
    assert(vfc_trellis_finish(&rx, tail) == 8);
    for (unsigned k = 0; k < 8; k++)
        assert(tail[k].index == stream[k].index && tail[k].dibit == stream[k].dibit);
    assert(vfc_trellis_init(&rx, code, 96, 3));
    for (unsigned k = 0; k < 8; k++)
        assert(vfc_trellis_put(&rx, stream[k].i, stream[k].q, tail) == 0);
    assert(vfc_trellis_finish(&rx, tail) == 8);
    assert(tail[0].dibit == ((stream[0].dibit + 1) & 3));
    for (unsigned k = 1; k < 8; k++) assert(tail[k].dibit == stream[k].dibit);
}

int main(void)
{
    FILE *f = fopen("reference-data/vfc/trellis-stream.csv", "r");
    assert(f);
    char line[128];
    unsigned counts[2] = {0};
    while (fgets(line, sizeof(line), f)) {
        if (line[0] == '#') continue;
        unsigned code;
        native_t row;
        assert(sscanf(line, "%u,%u,%u,%f,%f", &code, &row.index, &row.dibit, &row.i, &row.q) == 5);
        assert(code == 8 || code == 32);
        unsigned arm = code == 32;
        assert(counts[arm] < 4096);
        streams[arm][counts[arm]++] = row;
    }
    assert(!ferror(f));
    fclose(f);
    assert(counts[0] == 4096 && counts[1] == 4096);
    for (unsigned arm = 0; arm < 2; arm++) {
        check(arm ? 32 : 8, streams[arm], 0);
        check(arm ? 32 : 8, streams[arm], 0.1f);
    }
    puts("PASS: V.FC 8/32-state receiver recovers 8192 native symbols, exact and perturbed");
    return 0;
}
