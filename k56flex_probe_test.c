/* Probe-stage sample generator against original-firmware vectors. */
#include "k56flex_probe.h"
#include "k56flex_probe_vectors.h"

#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(c, ...) do { if (!(c)) { ++failures; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

int main(void)
{
    unsigned vi, ok = 0, bad_by_stage[8] = {0}, total_by_stage[8] = {0};
    for (vi = 0; vi < K56FLEX_PROBE_VECTOR_COUNT; ++vi) {
        const k56flex_probe_vector_t *v = &k56flex_probe_vectors[vi];
        k56flex_probe_t p;
        const uint16_t *w = &k56flex_probe_vector_words[v->word_offset];
        const int16_t *want = &k56flex_probe_vector_samples[v->sample_offset];
        unsigned b, i, bad = 0;
        if (v->flags) continue;      /* pad-group variants of training records are not modelled */
        if (k56flex_probe_init(&p, (k56flex_probe_stage_t)v->stage, (k56flex_law_t)v->law, v->flags)) { CHECK(0, "init %u", vi); continue; }
        p.scrambled = v->scrambled;
        p.seq = ((uint32_t)v->seq_hi << 16) | v->seq_lo;
        k56flex_probe_seed(&p, w[0]);
        for (i = 1; i < v->nwords; ++i) k56flex_probe_append(&p, w[i]);
        ++total_by_stage[v->stage];
        for (b = 0; b < v->blocks && !bad; ++b) {
            int16_t got[6];
            unsigned n = k56flex_probe_block(&p, got);
            if (n != v->per_block || memcmp(got, want + b * n, n * sizeof(got[0]))) {
                bad = 1;
                if (!bad_by_stage[v->stage]) printf("vec %u stage %u law %u flags %02x scr %u: block %u differs (got %d.. want %d..)\n", vi, v->stage, v->law, v->flags, v->scrambled, b, got[0], want[b * n]);
                ++bad_by_stage[v->stage];
            }
        }
        if (!bad) ++ok;
    }
    for (vi = 0; vi < 7; ++vi) printf("stage %u: %u/%u vectors differ\n", vi, bad_by_stage[vi], total_by_stage[vi]);
    CHECK(ok == K56FLEX_PROBE_VECTOR_COUNT / 3, "%u probe vectors match (pad flag 0 only)", ok);
    printf(failures ? "k56flex_probe_test: %d FAILURES\n" : "k56flex_probe_test: all passed\n", failures);
    return failures != 0;
}
