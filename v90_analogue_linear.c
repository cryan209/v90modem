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
#define FIFO            512
#define FIFO_MASK       (FIFO - 1)

struct v90a_linear_s {
    v90_law_t law;
    double    lvl[128];        /* |decoded level| per Ucode, ascending */
    double    ref;             /* lvl[REF_UCODE] */

    /*
     * The Ucodes actually in use, when they are known.
     *
     * Slicing onto all 128 is wrong wherever the transmitted constellation is a
     * subset, which is everywhere: §8.4.5's TRN1d is ONE Ucode, §8.4.1's DIL is
     * the descriptor's ladder, and §8.6's Phase 4 is whatever CP selected.  It
     * is not only that a full-ladder slice makes more errors -- it also makes
     * the decision region meaningless, and that region is what tells an
     * equaliser whether to believe a decision.  On TRN1d, where the true
     * constellation is two points a long way apart and a decision is therefore
     * certain, full-ladder tolerances refused 29567 decisions out of 30000.
     */
    uint8_t   set[128];
    int       set_len;

    double    dc;              /* one-pole DC estimate */
    double    level;           /* EMA of block peaks, in input units */
    double    gain;
    bool      have_level;
    bool      locked;

    int16_t   block[BLOCK];
    int       block_len;
    double    last_decision;
    double    last_tolerance;

    /*
     * Sliced codewords, and the level each was decided to be, waiting to be
     * handed over.  This decouples how many the slicer produces from how many
     * the caller asked for -- which matters because those numbers are very
     * different at the two ends of a call: unlocked it produces a whole block
     * at once, and the equaliser path feeds it one symbol at a time.  Without
     * it the block filled, the caller had room for one, and every sample after
     * that was dropped for the rest of the call.
     */
    uint8_t   fifo[FIFO];
    double    fifo_dec[FIFO];
    double    fifo_tol[FIFO];
    int       fifo_head;
    int       fifo_tail;
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

void v90a_linear_set_reference(v90a_linear_t *s, int ucode, double level)
{
    if (s == NULL  ||  ucode < 0  ||  ucode > 127  ||  level <= 0.0)
        return;
    if (s->lvl[ucode] <= 0.0)
        return;
    s->level = level;
    s->gain = s->lvl[ucode]/level;
    s->have_level = true;
    s->locked = true;
}

double v90a_linear_last_decision(const v90a_linear_t *s)
{
    return s ? s->last_decision : 0.0;
}

double v90a_linear_last_tolerance(const v90a_linear_t *s)
{
    return s ? s->last_tolerance : 0.0;
}

void v90a_linear_set_constellation(v90a_linear_t *s,
                                   const uint8_t *ucodes, int n)
{
    if (s == NULL)
        return;
    if (ucodes == NULL  ||  n <= 0) {
        s->set_len = 0;
        return;
    }
    if (n > 128)
        n = 128;
    for (int i = 0; i < n; i++)
        s->set[i] = (uint8_t) (ucodes[i] & 0x7F);
    s->set_len = n;
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
    int best;
    double best_d;

    if (s->set_len > 0) {
        best = s->set[0];
        best_d = fabs(magnitude - s->lvl[best]);
        for (int i = 1; i < s->set_len; i++) {
            double d = fabs(magnitude - s->lvl[s->set[i]]);

            if (d < best_d) {
                best_d = d;
                best = s->set[i];
            }
        }
        return best;
    }
    best = 0;
    best_d = fabs(magnitude - s->lvl[0]);
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

static void fifo_put(v90a_linear_t *s, uint8_t c, double decision, double tol)
{
    int next = (s->fifo_head + 1) & FIFO_MASK;

    if (next == s->fifo_tail)
        return;                        /* caller is not draining; drop */
    s->fifo[s->fifo_head] = c;
    s->fifo_dec[s->fifo_head] = decision;
    s->fifo_tol[s->fifo_head] = tol;
    s->fifo_head = next;
}

/*
 * Half the distance from this Ucode to its nearer neighbour, in input units:
 * the decision region the slice was taken inside.
 *
 * An equaliser adapting on decisions needs this, not a threshold of its own.
 * The G.711 ladder's steps span a factor of a hundred from the bottom of the
 * scale to the top, so any fixed tolerance is most of a step at one end and a
 * fraction of one at the other -- and a tolerance that scales with the LEVEL,
 * which is the obvious substitute, is worse than useless: it accepts errors on
 * the loud points most readily, so the loop is pulled towards them and the gain
 * ratchets up.  Measured, that took a run whose frozen taps recovered 14% of
 * codewords down to 0.7%.
 */
static double slice_tolerance(const v90a_linear_t *s, int ucode)
{
    double here = s->lvl[ucode];
    double gap = -1.0;

    if (s->set_len > 0) {
        for (int i = 0; i < s->set_len; i++) {
            double d = fabs(s->lvl[s->set[i]] - here);

            if (s->set[i] != ucode  &&  (gap < 0.0  ||  d < gap))
                gap = d;
        }
        if (gap <= 0.0)
            gap = here > 0.0 ? here : 1.0;
        return 0.5*gap/s->gain;
    }
    if (ucode > 0)
        gap = here - s->lvl[ucode - 1];
    if (ucode < 127) {
        double up = s->lvl[ucode + 1] - here;

        if (gap < 0.0  ||  up < gap)
            gap = up;
    }
    if (gap <= 0.0)
        gap = here;
    return 0.5*gap/s->gain;
}

static void flush_block(v90a_linear_t *s)
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

        fifo_put(s, v90_codeword_compose(s->law, ucode, x >= 0.0 ? 1 : 0),
                 (x >= 0.0 ? 1.0 : -1.0)*s->lvl[ucode]/s->gain,
                 slice_tolerance(s, ucode));
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
    for (int i = 0; i < len; i++) {
        /* The HSF codec's receive stream carries a standing DC offset of about
         * 900 counts, which shifts every sliced level and inverts the sign of
         * the small ones.  A one-pole estimate at ~1 Hz is far below anything
         * §8.4 puts on the line. */
        /*
         * ...and stop once the gain is locked, for the same reason the
         * equaliser's AGC stops: a one-pole tracker cannot tell a codec's
         * standing offset from a signal that holds a level, and §8.4.1's DIL
         * holds each level for a whole segment by construction.  Left running
         * it eats the DIL -- measured, 63% of its symbols came back more than
         * four Ucodes out while a window at the start of the same run was
         * exact.
         */
        if (!s->locked)
            s->dc += ((double) amp[i] - s->dc)/8192.0;
        s->block[s->block_len++] = amp[i];
        /* The block exists only to measure a level.  Once the gain is settled
         * there is nothing left to measure and nothing to be gained by holding
         * symbols back -- and holding them back would make the decision an
         * equaliser adapts on up to 30 ms stale, which is 240 symbols of a
         * channel it is meant to be tracking. */
        if (s->block_len >= (s->locked ? 1 : BLOCK))
            flush_block(s);
    }
    while (n < max  &&  s->fifo_tail != s->fifo_head) {
        s->last_decision = s->fifo_dec[s->fifo_tail];
        s->last_tolerance = s->fifo_tol[s->fifo_tail];
        out[n++] = s->fifo[s->fifo_tail];
        s->fifo_tail = (s->fifo_tail + 1) & FIFO_MASK;
    }
    return n;
}
