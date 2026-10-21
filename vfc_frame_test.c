/* Independent c1bf/c88a vectors: every documented symbol rate and bit rate,
 * both negotiated ring options, including sixteen native frame boundaries. */
#include "vfc_frame.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

int main(void)
{
    FILE *f = fopen("reference-data/vfc/frame.csv", "r");
    assert(f);
    char line[512];
    unsigned cases = 0, valid = 0;
    while (fgets(line, sizeof(line), f)) {
        if (line[0] == '#') continue;
        int v[25];
        char *p = line, *end;
        for (unsigned k = 0; k < 25; k++) {
            v[k] = strtol(p, &end, 10);
            assert(end != p);
            assert(*end == (k == 24 ? '\n' : ','));
            p = end + 1;
        }
        vfc_frame_plan_t plan;
        bool fit = v[6] >= 0 && v[6] <= 7 && ((v[7]+1) << v[6]) <= 160;
        assert(vfc_frame_plan(&plan, v[0], v[1], v[2]) == fit);
        if (fit) {
            assert(plan.schedule_mask == v[3] && plan.schedule == v[4]);
            assert(plan.high_bits == (unsigned)v[5] && plan.uncoded_bits == (unsigned)v[6]);
            assert(plan.max_ring == (unsigned)v[7] && plan.shell_extra == (unsigned)v[8]);
            for (unsigned k = 0; k < 16; k++)
                assert(vfc_frame_next_bits(&plan) == (unsigned)v[9+k]);
            valid++;
        }
        cases++;
    }
    assert(!ferror(f));
    fclose(f);
    assert(cases == 108 && valid);
    vfc_frame_plan_t plan;
    assert(vfc_frame_plan(&plan, 5, 28800, false));
    unsigned bits = 0;
    for (unsigned k = 0; k < 5; k++) bits += vfc_frame_next_bits(&plan);
    assert(bits == 336); /* 28800 * eight symbols * five frames / (24000/7) */
    unsigned saved = plan.high_bits;
    assert(!vfc_frame_plan(&plan, 6, 28800, false));
    assert(!vfc_frame_plan(&plan, 5, 28900, false));
    assert(!vfc_frame_plan(&plan, 5, 0, false));
    assert(plan.high_bits == saved);
    printf("PASS: V.FC frame plans match %u native cases (%u fitting table geometry)\n",
           cases, valid);
    return 0;
}
