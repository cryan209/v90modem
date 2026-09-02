/*
 * v90_analogue_linear.c — see v90_analogue_linear.h.
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <spandsp.h>

#include "v90_analogue_linear.h"

/* One block of level estimation.  30 ms is five Sd repetitions (§8.4.4 puts Sd
 * on a six-codeword cycle), so a block always spans whole cycles' worth of
 * both the W slots and the zero slots. */
#define BLOCK           240
/* The Ucode the measured line level is mapped onto.  Arbitrary -- see the
 * header on why the absolute scale is neither recoverable nor needed -- but it
 * must sit high enough that the ladder below it has resolution and low enough
 * that a level 6.6 dB above it (§8.4.4's W against §8.4.5's TRN1d, seen the
 * other way round) does not run off the top. */
#define REF_UCODE       64
/* Below this the input is silence rather than a signal, and a gain derived
 * from it would be meaningless. */
#define MIN_LEVEL       40.0

struct v90a_linear_s {
    v90_law_t law;
    double    lvl[128];        /* |decoded level| per Ucode, ascending */
    double    ref;             /* lvl[REF_UCODE] */

    double    dc;              /* one-pole DC estimate */
    double    level;           /* EMA of block peaks, in input units */
    double    gain;
    bool      have_level;
    bool      locked;

    int16_t   block[BLOCK];
    int       block_len;
};

static double codeword_level(v90_law_t law, int ucode)
{
    uint8_t c = v90_codeword_compose(law, ucode, 1);
    int16_t linear = (law == V90_LAW_ALAW) ? alaw_to_linear(c) : ulaw_to_linear(c);

    return fabs((double) linear);
}

v90a_linear_t *v90a_linear_init(v90_law_t law)
{
    v90a_linear_t *s;

    if ((s = calloc(1, sizeof(*s))) == NULL)
        return NULL;
    s->law = law;
    for (int u = 0; u < 128; u++)
        s->lvl[u] = codeword_level(law, u);
    s->ref = s->lvl[REF_UCODE];
    if (s->ref <= 0.0)
        s->ref = 1.0;
    s->gain = 1.0;
    return s;
}

void v90a_linear_free(v90a_linear_t *s)
{
    free(s);
}

void v90a_linear_lock(v90a_linear_t *s)
{
    if (s != NULL  &&  s->have_level)
        s->locked = true;
}

bool v90a_linear_locked(const v90a_linear_t *s)
{
    return s != NULL  &&  s->locked;
}

double v90a_linear_gain(const v90a_linear_t *s)
{
    return s ? s->gain : 0.0;
}

double v90a_linear_level(const v90a_linear_t *s)
{
    return s ? s->level : 0.0;
}

/*
 * Nearest Ucode by level.  The ladder is monotonic in the Ucode index for both
 * laws (§5.1: Ucode counts up from the smallest magnitude), so a linear scan
 * that stops once the levels start moving away is exact.
 */
static int slice_ucode(const v90a_linear_t *s, double magnitude)
{
    int best = 0;
    double best_d = fabs(magnitude - s->lvl[0]);

    for (int u = 1; u < 128; u++) {
        double d = fabs(magnitude - s->lvl[u]);

        if (d > best_d  &&  s->lvl[u] > magnitude)
            break;
        if (d < best_d) {
            best_d = d;
            best = u;
        }
    }
    return best;
}

static void flush_block(v90a_linear_t *s, uint8_t *out, int *n)
{
    double peak = 0.0;

    for (int i = 0; i < s->block_len; i++) {
        double x = (double) s->block[i] - s->dc;

        if (fabs(x) > peak)
            peak = fabs(x);
    }
    if (!s->locked  &&  peak >= MIN_LEVEL) {
        /* Attack fast enough to follow the start of Sd, slow enough that one
         * loud block cannot set the scale for the whole call. */
        s->level = s->have_level ? (0.75*s->level + 0.25*peak) : peak;
        s->have_level = true;
        s->gain = s->ref/s->level;
    }
    for (int i = 0; i < s->block_len; i++) {
        double x = (double) s->block[i] - s->dc;
        int ucode = s->have_level ? slice_ucode(s, fabs(x)*s->gain) : 0;

        out[(*n)++] = v90_codeword_compose(s->law, ucode, x >= 0.0 ? 1 : 0);
    }
    s->block_len = 0;
}

int v90a_linear_put(v90a_linear_t *s,
                    const int16_t *amp, int len,
                    uint8_t *out, int max)
{
    int n = 0;

    if (s == NULL  ||  amp == NULL  ||  out == NULL)
        return 0;
    /* A block held over from the previous call because its output would not
     * fit.  Sliced first, so no sample is ever dropped. */
    if (s->block_len == BLOCK  &&  max >= BLOCK)
        flush_block(s, out, &n);
    for (int i = 0; i < len; i++) {
        /* The HSF codec's receive stream carries a standing DC offset of about
         * 900 counts, which shifts every sliced level and inverts the sign of
         * the small ones.  A one-pole estimate at ~1 Hz is far below anything
         * §8.4 puts on the line. */
        s->dc += ((double) amp[i] - s->dc)/8192.0;
        if (s->block_len >= BLOCK)
            return n;          /* no room; the rest arrives next call */
        s->block[s->block_len++] = amp[i];
        if (s->block_len == BLOCK  &&  n + BLOCK <= max)
            flush_block(s, out, &n);
    }
    return n;
}
