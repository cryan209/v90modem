/* V.FC c1bf..c20d integer frame planning; c88a..c89b fractional scheduling.
 * See vfc_frame.h. Do not replace these rational rates with rounded bauds. */
#include "vfc_frame.h"
#include <string.h>

bool vfc_frame_plan(vfc_frame_plan_t *p, unsigned baud_index, unsigned bit_rate,
                    bool extra_ring)
{
    /* cb35, cb3b, cb47. cb47 has EIGHT entries, indexed by the residual
     * shell bits, unlike cb35/cb3b's six symbol-rate entries. */
    static const unsigned factor[6] = {40, 35, 48, 32, 30, 28};
    static const unsigned divisor[6] = {5, 5, 7, 5, 5, 5};
    static const unsigned rings[8] = {3, 4, 4, 5, 5, 6, 6, 7};
    if (!p || baud_index > 5 || bit_rate < 9600 || bit_rate > 28800
        || bit_rate % 2400)
        return false;
    unsigned n = factor[baud_index] * (bit_rate / 2400);
    unsigned d = divisor[baud_index];
    unsigned b = (n + d - 1) / d;
    if (b < 32) return false;
    unsigned q = (b - 32) / 8, k = (b - 32) & 7;
    unsigned m = rings[k] + extra_ring;
    if (q > 7 || ((m + 1) << q) > 160) return false;
    unsigned remainder = n - (b - 1) * d;
    unsigned mask = ((1u << d) - 1) & (0u - (1u << remainder));
    *p = (vfc_frame_plan_t){b, q, m, k, d, mask, (2 * mask) >> d};
    return true;
}

unsigned vfc_frame_next_bits(vfc_frame_plan_t *p)
{
    if (!p) return 0;
    unsigned low = p->schedule & 1;
    p->schedule >>= 1;
    if (!p->schedule) p->schedule = p->schedule_mask;
    return p->high_bits - low;
}

/* Enumerative shells have the same recursive counts as V.34 9.6.3, equations
 * (9-8)..(9-24). V.FC's ring bound is inclusive (c81b), and the split and
 * output order below are checked against its c88a..c98c, not assumed from
 * V.34's point ordering. Coefficients remain exact integers. */
typedef struct { uint32_t g[4][65]; unsigned m; } shells_t;

static void shells_init(shells_t *s, unsigned m)
{
    memset(s, 0, sizeof(*s));
    s->m = m;
    for (unsigned e = 0; e <= m; e++) s->g[0][e] = 1;
    for (unsigned l = 1; l < 4; l++) {
        unsigned limit = m << (l-1);
        for (unsigned a = 0; a <= limit; a++)
            for (unsigned b = 0; b <= limit; b++)
                s->g[l][a+b] += s->g[l-1][a] * s->g[l-1][b];
    }
}

static bool split_decode(const shells_t *s, unsigned level, unsigned energy,
                         uint32_t rank, uint8_t *ring)
{
    if (!level) {
        if (energy > s->m || rank) return false;
        *ring = energy;
        return true;
    }
    unsigned limit = s->m << (level-1);
    for (unsigned a = 0; a <= limit && a <= energy; a++) {
        unsigned b = energy - a;
        if (b > limit) continue;
        uint32_t left = s->g[level-1][a];
        uint32_t count = left * s->g[level-1][b];
        if (rank >= count) { rank -= count; continue; }
        return split_decode(s, level-1, a, rank % left, ring)
            && split_decode(s, level-1, b, rank / left, ring + (1u << (level-1)));
    }
    return false;
}

static uint32_t split_encode(const shells_t *s, unsigned level,
                             const uint8_t *ring, unsigned *energy)
{
    if (!level) { *energy = *ring; return 0; }
    unsigned a, b, limit = s->m << (level-1);
    uint32_t left = split_encode(s, level-1, ring, &a);
    uint32_t right = split_encode(s, level-1, ring + (1u << (level-1)), &b);
    uint32_t rank = left + s->g[level-1][a] * right;
    *energy = a + b;
    for (unsigned i = 0; i < a; i++)
        if (*energy >= i && *energy-i <= limit)
            rank += s->g[level-1][i] * s->g[level-1][*energy-i];
    return rank;
}

static unsigned ring_read(const uint8_t input[16], unsigned *cursor, unsigned n)
{
    unsigned v = 0;
    for (unsigned k = 0; k < n; k++) {
        v |= ((input[*cursor >> 3] >> (*cursor & 7)) & 1u) << k;
        *cursor = (*cursor + 1) & 127;
    }
    return v;
}

static void ring_write(uint8_t output[16], unsigned *cursor, unsigned n, unsigned v)
{
    for (unsigned k = 0; k < n; k++) {
        unsigned mask = 1u << (*cursor & 7);
        output[*cursor >> 3] = (output[*cursor >> 3] & ~mask)
                            | (((v >> k) & 1u) << (*cursor & 7));
        *cursor = (*cursor + 1) & 127;
    }
}

static bool geometry(const vfc_frame_plan_t *p)
{
    return p && p->uncoded_bits <= 5 && p->max_ring >= 3 && p->max_ring <= 8
        && ((p->max_ring+1) << p->uncoded_bits) <= 160
        && p->shell_extra <= 7 && (p->period == 5 || p->period == 7)
        && p->high_bits == 32 + 8*p->uncoded_bits + p->shell_extra;
}

static unsigned distribution(const vfc_frame_plan_t *p, unsigned bits)
{
    /* cb65: two bits per symbol, low symbol first. */
    static const uint16_t masks[8] = {0xaaaa, 0xaaab, 0xaaaf, 0xaabf,
                                    0xaaff, 0xabff, 0xafff, 0xbfff};
    return masks[p->shell_extra] - (bits < p->high_bits ? 0x4000 : 0);
}

bool vfc_frame_map(vfc_frame_plan_t *p, const uint8_t input[16], unsigned cursor,
                   vfc_symbol_frame_t *out)
{
    if (!input || !out || cursor > 127 || !geometry(p)) return false;
    vfc_frame_plan_t next = *p;
    vfc_symbol_frame_t frame = {0};
    frame.bits = vfc_frame_next_bits(&next);
    unsigned pattern = distribution(p, frame.bits), shift = 0;
    uint32_t rank = 0;
    for (unsigned i = 0; i < 8; i++, pattern >>= 2) {
        frame.direct[i] = ring_read(input, &cursor, p->uncoded_bits + 2);
        unsigned n = pattern & 3;
        rank |= ring_read(input, &cursor, n) << shift;
        shift += n;
    }
    shells_t shells;
    shells_init(&shells, p->max_ring);
    unsigned energy = 0;
    while (energy <= 8*p->max_ring && rank >= shells.g[3][energy])
        rank -= shells.g[3][energy++];
    if (energy > 8*p->max_ring || !split_decode(&shells, 3, energy, rank, frame.ring))
        return false;
    frame.next_cursor = cursor;
    *out = frame;
    *p = next;
    return true;
}

bool vfc_frame_unmap(vfc_frame_plan_t *p, const vfc_symbol_frame_t *frame,
                     unsigned cursor, uint8_t output[16])
{
    if (!frame || !output || cursor > 127 || !geometry(p)) return false;
    vfc_frame_plan_t next = *p;
    unsigned bits = vfc_frame_next_bits(&next);
    if (frame->bits != bits || frame->next_cursor != ((cursor + bits) & 127))
        return false;
    for (unsigned i = 0; i < 8; i++)
        if (frame->ring[i] > p->max_ring || frame->direct[i] >= (1u << (p->uncoded_bits+2)))
            return false;
    shells_t shells;
    shells_init(&shells, p->max_ring);
    unsigned energy;
    uint32_t rank = split_encode(&shells, 3, frame->ring, &energy);
    for (unsigned e = 0; e < energy; e++) rank += shells.g[3][e];
    unsigned shell_bits = bits - 8*(p->uncoded_bits+2);
    if (rank >= (1u << shell_bits)) return false;
    uint8_t result[16];
    memcpy(result, output, sizeof(result));
    unsigned pattern = distribution(p, bits);
    for (unsigned i = 0; i < 8; i++, pattern >>= 2) {
        ring_write(result, &cursor, p->uncoded_bits+2, frame->direct[i]);
        unsigned n = pattern & 3;
        ring_write(result, &cursor, n, rank);
        rank >>= n;
    }
    memcpy(output, result, sizeof(result));
    *p = next;
    return true;
}
