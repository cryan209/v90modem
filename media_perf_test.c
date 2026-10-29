/* Tests of diagnostic wall-time accounting; no protocol or DSP dependencies. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "media_perf.h"

static int compare(const void *a, const void *b)
{
    uint64_t x = *(const uint64_t *)a, y = *(const uint64_t *)b;
    return (x > y) - (x < y);
}

int main(void)
{
    media_perf_t s;
    media_perf_reset(&s);
    assert(media_perf_percentile_ns(&s, 95) == 0);
    media_perf_note(&s, 123, 0, 1);
    assert(s.calls == 0);
    media_perf_note(&s, 20000000, 160, 0);
    assert(s.over_budget == 0);
    media_perf_note(&s, 20000001, 160, 1);
    assert(s.over_budget == 1 && s.transitions == 1);
    media_perf_note(&s, 10000001, 80, 0);
    assert(s.over_budget == 2);
    media_perf_note(&s, 30000000, 320, 0);
    assert(s.over_budget == 2);
    assert(s.calls == 4 && s.samples == 720 && s.wall_ns == 80000002);
    assert(s.max_ns == 30000000);
    assert(media_perf_percentile_ns(&s, 100) == s.max_ns);
    assert(media_perf_percentile_ns(&s, 0) == 0);
    assert(media_perf_percentile_ns(&s, 101) == 0);

    /* Deterministic broad distribution, including scheduling-stall outliers.
     * Every reported percentile must bound the exact nearest-rank value. */
    uint64_t values[10000];
    uint32_t rng = 12345;
    media_perf_reset(&s);
    for (unsigned i = 0; i < 10000; i++) {
        rng = rng * 1664525u + 1013904223u;
        values[i] = rng % 200000000;
        media_perf_note(&s, values[i], 160, i % 101 == 0);
    }
    qsort(values, 10000, sizeof(values[0]), compare);
    for (unsigned p = 1; p <= 100; p++) {
        uint64_t upper = media_perf_percentile_ns(&s, p);
        assert(upper >= values[p * 100 - 1]);
        assert(upper <= s.max_ns);
    }
    /* Probe every histogram boundary on both sides, including saturation. */
    for (unsigned shift = 0; shift < 54; shift++) {
        uint64_t ns = UINT64_C(1) << shift;
        for (int delta = -1; delta <= 1; delta++) {
            media_perf_reset(&s);
            uint64_t value = ns + delta;
            media_perf_note(&s, value, 160, 0);
            assert(media_perf_bin(value) < MEDIA_PERF_BINS);
            assert(media_perf_percentile_ns(&s, 95) == value);
        }
    }
    media_perf_reset(&s);
    media_perf_note(&s, UINT64_MAX, 1, 1);
    assert(media_perf_percentile_ns(&s, 99) == UINT64_MAX);
    media_perf_reset(&s);
    assert(s.calls == 0 && s.max_ns == 0 && s.transitions == 0);
    for (unsigned i = 0; i < MEDIA_PERF_BINS; i++)
        assert(s.histogram[i] == 0);
    puts("media_perf_test: PASS");
    return 0;
}
