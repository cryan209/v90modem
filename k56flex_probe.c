#include "k56flex_probe.h"
#include "k56flex_probe_tables.h"

#include <string.h>

int k56flex_probe_init(k56flex_probe_t *p, k56flex_probe_stage_t stage, k56flex_law_t law,
                       unsigned flags)
{
    unsigned i;
    memset(p, 0, sizeof(*p));
    if (law != K56FLEX_LAW_MU && law != K56FLEX_LAW_A) return -1;
    /* D811 would pick a pad-group variant when DM 8FEF bits 6/7 are set, but in the
     * shipped records those pointers mostly lead to unrelated words (levels such as 31104
     * and -32768 appear), so what the firmware does with them in training is unrecovered.
     * Training therefore always uses the base tables; the data mapper still honours them. */
    flags = 0;
    for (i = 0; i < K56FLEX_PROBE_TABLE_COUNT; ++i) {
        const k56flex_probe_table_t *t = &k56flex_probe_tables[i];
        if (t->stage == stage && t->law == law && t->flags == flags) {
            p->table = t;
            memcpy(p->cfg, t->cfg, sizeof(p->cfg));
            p->levels = &k56flex_probe_levels[t->first_level];
            p->law = (uint8_t)law;
            p->scrambled = stage != K56FLEX_PROBE_ID;
            if (stage == K56FLEX_PROBE_2) p->seq = (2u << 16) | 0x800;
            return 0;
        }
    }
    return -1;
}

/* The FIFO holds raw source bits; the scrambler runs as bits are taken, so scr_hist
 * is always the history of what has actually been transmitted. */
static void push_word(k56flex_probe_t *p, uint16_t w)
{
    unsigned i;
    if (p->fifo_bits + 16 > sizeof(p->fifo)) return;       /* never reached by the stage flow */
    for (i = 0; i < 16; ++i)
        p->fifo[(p->fifo_head + p->fifo_bits + i) % sizeof(p->fifo)] = (uint8_t)((w >> i) & 1);
    p->fifo_bits += 16;
    p->last_word = w;
}

void k56flex_probe_seed(k56flex_probe_t *p, uint16_t word)
{
    p->fifo_head = 0;
    p->fifo_bits = 0;
    p->scr_hist = 0;
    push_word(p, word);
}

void k56flex_probe_append(k56flex_probe_t *p, uint16_t word)
{
    push_word(p, word);
}

static uint32_t fetch_bits(k56flex_probe_t *p, unsigned n)
{
    uint32_t v = 0;
    unsigned i;
    while (p->fifo_bits < n) push_word(p, p->last_word);
    for (i = 0; i < n; ++i) {
        unsigned x = p->fifo[p->fifo_head];
        p->fifo_head = (p->fifo_head + 1) % sizeof(p->fifo);
        if (p->scrambled) {
            unsigned y = x ^ ((p->scr_hist >> 4) & 1) ^ ((p->scr_hist >> 22) & 1);
            p->scr_hist = (p->scr_hist << 1) | y;
            x = y;
        }
        v |= (uint32_t)x << i;
    }
    p->fifo_bits -= n;
    return v;
}

/* 65-bit chain C : ACC : ACCB rotated right, as RORB. */
static void rorb(uint32_t *acc, uint32_t *accb, unsigned *c)
{
    unsigned lo = *acc & 1, bb = *accb & 1;
    *accb = (*accb >> 1) | (lo << 31);
    *acc = (*acc >> 1) | ((uint32_t)*c << 31);
    *c = bb;
}

static void rolb(uint32_t *acc, uint32_t *accb, unsigned *c)
{
    unsigned hi = *acc >> 31, bb = *accb >> 31;
    *acc = (*acc << 1) | bb;
    *accb = (*accb << 1) | *c;
    *c = hi;
}

static void ror1(uint32_t *acc, unsigned *c)
{
    unsigned lo = *acc & 1;
    *acc = (*acc >> 1) | ((uint32_t)*c << 31);
    *c = lo;
}

/* D890.  `src` is DM 8C08:8C07; the carry flag entering it is `c0`. */
static uint32_t sequence(k56flex_probe_t *p, uint32_t src, unsigned c0)
{
    const uint16_t *g = p->cfg;
    uint32_t t, s = src, acc, accb = 0, out;
    unsigned c = c0, i, k, tc = (g[10] >> 2) & 1;
    t = (uint32_t)((int32_t)p->seq >> 7);
    for (k = 0; k <= g[4]; ++k) t = (uint32_t)((int32_t)t >> 1);
    for (i = 0; i < g[0]; ++i) {
        acc = s;
        for (k = 0; k <= g[7]; ++k) rorb(&acc, &accb, &c);
        s = acc;
        acc = t;
        rorb(&acc, &accb, &c);
        if (tc)
            for (k = 0; k <= g[5]; ++k) ror1(&acc, &c);
        t = acc;
    }
    acc = 0;
    for (k = 0; k <= g[2]; ++k) rolb(&acc, &accb, &c);
    out = acc;
    accb = p->seq;
    acc = src;
    for (k = 0; k <= g[8]; ++k) rorb(&acc, &accb, &c);
    p->seq = accb;
    return out;
}

unsigned k56flex_probe_block_src(k56flex_probe_t *p, uint32_t src, int16_t out[K56FLEX_PROBE_MAX_SAMPLES])
{
    const uint16_t *g = p->cfg;
    unsigned count = g[0], i, mask = g[9];
    uint32_t bits;
    bits = (g[10] & 1) ? sequence(p, src, 1) : (src & 0xffff);
    for (i = 0; i < count; ++i) {
        out[i] = p->levels[bits & mask];
        bits = (uint32_t)((int32_t)bits >> (g[6] + 1));
    }
    if (g[10] & 2) {
        unsigned flip = 0;
        for (i = 0; i < count; ++i) {
            int32_t v = out[i];
            if (flip && (g[10] & 8)) {
                v = v < 0 ? -v : v;
                if (p->dc >= 0) v = -v;
                out[i] = (int16_t)v;
            }
            p->dc += out[i];
            flip ^= 1;
        }
    }
    return count;
}

unsigned k56flex_probe_block(k56flex_probe_t *p, int16_t out[K56FLEX_PROBE_MAX_SAMPLES])
{
    unsigned nbits = p->cfg[3];
    return k56flex_probe_block_src(p, fetch_bits(p, nbits > 32 ? 32 : nbits), out);
}

unsigned k56flex_probe_invert(const k56flex_probe_t *p, const int16_t samples[K56FLEX_PROBE_MAX_SAMPLES],
                              uint32_t *src)
{
    unsigned nbits = p->cfg[3] > 20 ? 20 : p->cfg[3], matches = 0;
    uint32_t cand, first = 0;
    for (cand = 0; cand < (1u << nbits); ++cand) {
        k56flex_probe_t t = *p;
        int16_t out[K56FLEX_PROBE_MAX_SAMPLES];
        unsigned n = k56flex_probe_block_src(&t, cand, out), i, ok = 1;
        for (i = 0; i < n; ++i) if (out[i] != samples[i]) { ok = 0; break; }
        if (ok) { if (!matches++) first = cand; }
    }
    if (src) *src = first;
    return matches;
}
