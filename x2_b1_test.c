/* Recorded Courier 403 x2 B1 regression; see docs/x2_implementation.md.
 * Acquisition must retain its strict fit and independent DATA checks. */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "spandsp.h"
#include "spandsp/private/bitstream.h"
#include "spandsp/private/power_meter.h"
#include "spandsp/private/logging.h"
#include "spandsp/private/v34.h"
static int ones(void *p) { (void)p; return 1; }
static void sink(void *p, int bit) { (void)p; (void)bit; }
static void check(const unsigned char *capture, size_t len, int chunk, int silence)
{
    v34_state_t *s = v34_init(NULL, 3200, 24000, false, true, ones, NULL, sink, NULL);
    assert(s);
    assert(v34_x2_prepare_upstream(s, 4, 1) == 0);
    assert(v34_v90_prepare_upstream_data(s, 4, 1, 24000, 0) == 0);
    /* Start before MP/E and preserve the exact PCMU sample clock. */
    size_t at = 93000;
    const size_t e = 100074;
    int begun = 0;
    while (at < len)
    {
        int16_t pcm[160];
        size_t n = len - at;
        if (n > (size_t)chunk) n = chunk;
        if (!begun && at + n > e) n = e - at;
        if (at == e && !begun)
        {
            assert(v34_begin_rx_data(s) == 0);
            begun = 1;
            continue;
        }
        for (size_t i = 0; i < n; ++i)
            pcm[i] = silence ? 0 : ulaw_to_linear(capture[at+i]);
        v34_rx(s, pcm, (int)n);
        at += n;
    }
    if (silence)
        assert(!s->rx.v90_t3_acquired);
    else
    {
        assert(s->rx.v90_t3_acquired);
        assert(s->rx.v90_t3_training_match >= 0.95f);
        assert(s->rx.parms.expanded_shaping);
        assert(s->rx.v90_far_tap_measured == 4);
        assert(s->rx.v90_t3_trellis_size == 2 /* MP trellis selector for 64 states */);
        assert(s->rx.viterbi.state_count == 64);
        printf("x2 B1: block=%d fit=%.3f expanded=%d trellis=%d\n", chunk,
               s->rx.v90_t3_training_match, s->rx.parms.expanded_shaping,
               s->rx.viterbi.state_count);
    }
    v34_free(s);
}
int main(int argc, char **argv)
{
    assert(argc == 2);
    FILE *f = fopen(argv[1], "rb"); assert(f);
    assert(fseek(f, 0, SEEK_END) == 0);
    long len = ftell(f); assert(len == 278249);
    rewind(f);
    unsigned char *capture = malloc((size_t)len); assert(capture);
    assert(fread(capture, 1, (size_t)len, f) == (size_t)len); fclose(f);
    check(capture, (size_t)len, 17, 0);
    check(capture, (size_t)len, 160, 0);
    check(capture, 110000, 160, 1);
    free(capture);
    puts("x2 B1: recorded acquisition and silence rejection passed");
    return 0;
}
