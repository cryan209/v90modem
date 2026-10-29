/* media_perf.h -- bounded, allocation-free wall-time accounting for media
 * callbacks. Diagnostics only: no protocol decisions or sample modification.
 * Caller provides synchronization. Percentiles are histogram upper bounds
 * (four subdivisions per power of two in microseconds), capped by exact max.
 */
#ifndef MEDIA_PERF_H
#define MEDIA_PERF_H

#include <stdint.h>
#include <string.h>

#define MEDIA_PERF_BINS 128

typedef struct {
    uint64_t calls, samples, wall_ns, max_ns, over_budget, transitions;
    uint64_t histogram[MEDIA_PERF_BINS];
} media_perf_t;

static inline void media_perf_reset(media_perf_t *s)
{
    memset(s, 0, sizeof(*s));
}

static inline unsigned media_perf_bin(uint64_t ns)
{
    uint64_t us = ns / 1000 + (ns % 1000 != 0);
    uint64_t base = 1, v = us;
    unsigned order = 0;
    while (v > 1 && order < 31) {
        v >>= 1;
        base <<= 1;
        order++;
    }
    if (us >= (base << 1))
        return MEDIA_PERF_BINS - 1;
    unsigned sub = us > base ? (unsigned)((us - base) * 4 / base) : 0;
    return order * 4 + sub;
}

static inline void media_perf_note(media_perf_t *s, uint64_t ns,
                                   unsigned samples, int transitioned)
{
    if (!samples)
        return;
    s->calls++;
    s->samples += samples;
    s->wall_ns += ns;
    if (ns > s->max_ns)
        s->max_ns = ns;
    /* The DS0 is exactly 8000 samples/s. A callback has the duration of its
     * input block, not a hard-coded 20 ms budget. Equality is not a miss. */
    if (ns > (uint64_t)samples * 125000)
        s->over_budget++;
    s->transitions += transitioned != 0;
    s->histogram[media_perf_bin(ns)]++;
}

static inline uint64_t media_perf_percentile_ns(const media_perf_t *s,
                                               unsigned percentile)
{
    if (!s->calls || !percentile || percentile > 100)
        return 0;
    uint64_t rank = s->calls / 100 * percentile
                  + (s->calls % 100 * percentile + 99) / 100;
    uint64_t count = 0;
    for (unsigned i = 0; i < MEDIA_PERF_BINS; i++) {
        count += s->histogram[i];
        if (count >= rank) {
            if (i == MEDIA_PERF_BINS - 1)
                return s->max_ns;
            uint64_t base = UINT64_C(1) << (i / 4);
            uint64_t upper = (base + ((i % 4 + 1) * base + 3) / 4) * 1000;
            return upper < s->max_ns ? upper : s->max_ns;
        }
    }
    return s->max_ns;
}

#endif
