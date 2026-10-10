/* K56flex downstream payload core; see k56flex.h for scope and evidence. */
#include "k56flex.h"
#include "k56flex_tables.h"

#include <string.h>

/* ------------------------------------------------------------ G.711 ----- */

typedef struct {
    int16_t value[256];     /* decoded 16-bit-scale value of every octet */
    int count;              /* distinct signed values */
    int16_t sorted[256];    /* ascending */
    uint8_t octet[256];
} g711_map_t;

static g711_map_t g711_map[2];
static int g711_ready;

static int mu_decode(uint8_t c)
{
    int t;
    c = (uint8_t)~c;
    t = (((c & 15) << 3) + 132) << ((c & 0x70) >> 4);
    t -= 132;
    return (c & 0x80) ? -t : t;
}

static int a_decode(uint8_t c)
{
    int t, seg;
    c ^= 0x55;
    t = (c & 15) << 4;
    seg = (c & 0x70) >> 4;
    if (seg == 0) t += 8;
    else { t += 0x108; t <<= seg - 1; }
    return (c & 0x80) ? t : -t;
}

static void g711_build(void)
{
    int law, c, i, j;
    if (g711_ready) return;
    for (law = 0; law < 2; ++law) {
        g711_map_t *m = &g711_map[law];
        m->count = 0;
        for (c = 0; c < 256; ++c) {
            int v = law ? a_decode((uint8_t)c) : mu_decode((uint8_t)c);
            m->value[c] = (int16_t)v;
            if (law == 0 && c == 0x7f) continue;   /* -0: keep +0 = 0xff */
            for (i = 0; i < m->count && m->sorted[i] < v; ++i) {}
            for (j = m->count; j > i; --j) {
                m->sorted[j] = m->sorted[j - 1];
                m->octet[j] = m->octet[j - 1];
            }
            m->sorted[i] = (int16_t)v;
            m->octet[i] = (uint8_t)c;
            ++m->count;
        }
    }
    g711_ready = 1;
}

int k56flex_g711_from_level(k56flex_law_t law, int level)
{
    const g711_map_t *m;
    int lo = 0, hi;
    g711_build();
    m = &g711_map[law == K56FLEX_LAW_A];
    hi = m->count - 1;
    if (level == 0 && law == K56FLEX_LAW_A) return 0xd5;   /* A-law has no zero: idle code (+8) */
    while (lo <= hi) {
        int mid = (lo + hi) / 2;
        if (m->sorted[mid] == level) return m->octet[mid];
        if (m->sorted[mid] < level) lo = mid + 1; else hi = mid - 1;
    }
    return -1;
}

int k56flex_level_from_g711(k56flex_law_t law, uint8_t octet)
{
    g711_build();
    return g711_map[law == K56FLEX_LAW_A].value[octet];
}

/* ------------------------------------------------------- shell counts ---- */

static void shell_init(k56flex_pcm_tx_t *t)
{
    unsigned n = t->n, k, i, j, len = 1;
    uint64_t a[130], b[130];
    a[0] = 1;
    for (k = 1; k <= 8; ++k) {
        unsigned nl = len + n;
        memset(b, 0, sizeof(b[0]) * nl);
        for (i = 0; i < len; ++i)
            for (j = 0; j <= n; ++j) b[i + j] += a[i];
        memcpy(a, b, sizeof(a[0]) * nl);
        len = nl;
        if (k == 2) memcpy(t->c[0], a, sizeof(a[0]) * len);
        else if (k == 4) memcpy(t->c[1], a, sizeof(a[0]) * len);
    }
    memcpy(t->c[2], a, sizeof(a[0]) * len);
    t->cum[0] = 0;
    for (i = 0; i < len; ++i) t->cum[i + 1] = t->cum[i] + a[i];
}

static void unrank(const k56flex_pcm_tx_t *t, int k, int s, uint64_t r, uint8_t *out)
{
    int n = t->n, h, a, lo, hi;
    const uint64_t *ct;
    if (k == 2) {
        int first = (s > n ? s - n : 0) + (int)r;
        out[0] = (uint8_t)first;
        out[1] = (uint8_t)(s - first);
        return;
    }
    h = k / 2;
    ct = h == 4 ? t->c[1] : t->c[0];
    lo = s - h * n > 0 ? s - h * n : 0;
    hi = h * n < s ? h * n : s;
    for (a = lo; a <= hi; ++a) {
        uint64_t size = ct[a] * ct[s - a];
        if (r < size) {
            unrank(t, h, a, r % ct[a], out);
            unrank(t, h, s - a, r / ct[a], out + h);
            return;
        }
        r -= size;
    }
}

static uint64_t rank_of(const k56flex_pcm_tx_t *t, int k, const uint8_t *v)
{
    int n = t->n, h, i, a = 0, b = 0, s, lo, x;
    const uint64_t *ct;
    uint64_t off = 0;
    if (k == 2) {
        s = v[0] + v[1];
        return (uint64_t)(v[0] - (s > n ? s - n : 0));
    }
    h = k / 2;
    for (i = 0; i < h; ++i) { a += v[i]; b += v[h + i]; }
    s = a + b;
    ct = h == 4 ? t->c[1] : t->c[0];
    lo = s - h * n > 0 ? s - h * n : 0;
    for (x = lo; x < a; ++x) off += ct[x] * ct[s - x];
    return off + rank_of(t, h, v) + ct[a] * rank_of(t, h, v + h);
}

/* ------------------------------------------------------------ config ---- */

static const k56flex_table_t *find_table(k56flex_law_t law, unsigned flags, int rate_index)
{
    unsigned i;
    for (i = 0; i < K56FLEX_TABLE_COUNT; ++i) {
        const k56flex_table_t *t = &k56flex_tables[i];
        if (t->law == law && t->flags == flags && t->rate_index == rate_index) return t;
    }
    return NULL;
}

static unsigned pop2(unsigned v) { return (v & 1) + ((v >> 1) & 1); }

unsigned k56flex_pcm_frame_bits(const k56flex_pcm_config_t *cfg)
{
    const k56flex_table_t *tb;
    if (!cfg || cfg->rate_bps % 2000) return 0;
    tb = find_table(cfg->law, cfg->report_field & 0xc0, cfg->rate_bps / 2000 + 3);
    return tb ? tb->alloc[0] : 0;
}

int k56flex_pcm_tx_init(k56flex_pcm_tx_t *tx, const k56flex_pcm_config_t *cfg,
                        k56flex_source_fn source, void *user)
{
    const k56flex_table_t *tb;
    unsigned field, flags, selector;
    int rate_index;
    if (!tx || !cfg || (cfg->law != K56FLEX_LAW_MU && cfg->law != K56FLEX_LAW_A)) return -1;
    if (cfg->rate_bps < 32000 || cfg->rate_bps > 60000 || cfg->rate_bps % 2000) return -1;
    field = cfg->report_field;
    if (field & ~0xfffu) return -1;
    flags = field & 0xc0;
    if (flags == 0xc0) return -1;
    rate_index = cfg->rate_bps / 2000 + 3;
    tb = find_table(cfg->law, flags, rate_index);
    if (!tb) return -1;
    memset(tx, 0, sizeof(*tx));
    tx->table = tb;
    tx->law = (uint8_t)cfg->law;
    tx->n = (uint8_t)(tb->alphabet - 1);
    tx->total = tb->alloc[0];
    tx->alloc_sel = tb->alloc[1];
    tx->amp_bits = tb->alloc[2];
    tx->width = tb->alloc[3];
    tx->rate_index = (uint8_t)rate_index;
    selector = (field >> 9) & 7;
    if (rate_index > 20 && selector) {
        tx->mask = tx->mask_field = (uint8_t)(selector >= 5 ? 63 : field & 63);
        tx->selector = (uint8_t)(selector >= 5 ? 6 : selector);
    }
    if (tx->n < 1 || tx->n > 15) return -1;
    shell_init(tx);
    tx->source = source;
    tx->source_user = user;
    tx->out_pos = K56FLEX_FRAME_SAMPLES;
    g711_build();
    return 0;
}

/* ------------------------------------------------------------- frame ---- */

typedef struct {
    int budget, loss, shift, request, remaining;
    unsigned phase;
} geometry_t;

static int geometry(const k56flex_pcm_tx_t *t, geometry_t *g)
{
    g->phase = (t->alloc_sel & 4) ? !t->phase : 0;
    g->budget = t->amp_bits - (int)g->phase;
    g->loss = g->shift = 0;
    if (t->rate_index > 20 && t->selector) {
        g->loss = t->selector + (int)pop2((t->mask_field >> (2 * (t->frame % 3))) & 3);
        switch (t->selector) {
        case 1: g->shift = g->loss == 2 ? 0 : 1; break;
        case 2: g->shift = g->loss == 4 ? 0 : (g->loss == 2 ? 2 : 1); break;
        case 4: g->shift = g->loss == 6 ? 0 : (g->loss == 4 ? 2 : 1); break;
        default: break;
        }
    }
    g->request = g->budget - g->shift;
    g->remaining = t->total - g->budget - g->loss;
    if (g->request < 0 || g->remaining < 0 || g->request + g->remaining > K56FLEX_MAX_FRAME_BITS)
        return -1;
    return 0;
}

/* Walk the eight positions once to learn the stolen bits, sign flags and
 * the number of source bits the extras consume; -1 if they do not add up. */
static int partition(const k56flex_pcm_tx_t *t, const geometry_t *g, unsigned *mask_out,
                     uint8_t stolen[8], uint8_t sign[8])
{
    const k56flex_table_t *tb = t->table;
    unsigned mask = t->mask, i;
    int consumed = 0;
    for (i = 0; i < 8; ++i) {
        unsigned s = mask & 1;
        mask = (mask >> 1) | (s ? 0x20 : 0);
        if (t->width < s) return -1;
        stolen[i] = (uint8_t)s;
        sign[i] = (uint8_t)((tb->sign_pattern >> (8 * g->phase + i)) & 1);
        consumed += (int)(t->width - s) + sign[i];
    }
    if (consumed != g->remaining) return -1;
    *mask_out = mask;
    return 0;
}

/* The queue holds raw source bits; the scrambler runs as they are taken, so
 * t->scrambler is the history of what has actually been transmitted and discarding
 * queued bits (the hand-off from training) cannot desynchronise the descrambler. */
static unsigned q_take_bit(k56flex_pcm_tx_t *t)
{
    unsigned x = t->q[t->q_head], y = x ^ ((t->scrambler >> 4) & 1) ^ ((t->scrambler >> 22) & 1);
    t->scrambler = (t->scrambler << 1) | y;
    t->q_head = (t->q_head + 1) % sizeof(t->q);
    --t->q_count;
    return y;
}

static unsigned q_take(k56flex_pcm_tx_t *t, unsigned n)
{
    unsigned v = 0, i;
    for (i = 0; i < n; ++i) v |= q_take_bit(t) << i;
    return v;
}

static uint64_t q_take64(k56flex_pcm_tx_t *t, unsigned n)
{
    uint64_t v = 0;
    unsigned i;
    for (i = 0; i < n; ++i) v |= (uint64_t)q_take_bit(t) << i;
    return v;
}

static int q_fill(k56flex_pcm_tx_t *t, unsigned need)
{
    while (t->q_count < need) {
        uint16_t w;
        unsigned i;
        if (!t->source || t->source(t->source_user, &w) < 0) return 0;
        for (i = 0; i < 16; ++i) {
            t->q[(t->q_head + t->q_count) % sizeof(t->q)] = (uint8_t)((w >> i) & 1);
            ++t->q_count;
        }
    }
    return 1;
}

int k56flex_pcm_tx_frame(k56flex_pcm_tx_t *t, int16_t out[K56FLEX_FRAME_SAMPLES])
{
    const k56flex_table_t *tb = t->table;
    const int16_t *levels = &k56flex_levels[tb->first_level];
    geometry_t g;
    uint8_t stolen[8], sign[8], amp[8];
    unsigned mask, i, s;
    uint64_t rank;
    int32_t dc;
    if (geometry(t, &g) < 0 || partition(t, &g, &mask, stolen, sign) < 0) return -1;
    if (!q_fill(t, (unsigned)(g.request + g.remaining))) return 0;
    rank = q_take64(t, (unsigned)g.request) << g.shift;
    for (s = 0; s < 8u * t->n && t->cum[s + 1] <= rank; ++s) {}
    unrank(t, 8, (int)s, rank - t->cum[s], amp);
    dc = t->dc;
    for (i = 0; i < 8; ++i) {
        unsigned sel = sign[i] ? q_take(t, 1) : 2;
        unsigned extra = q_take(t, t->width - stolen[i] + 0u) << stolen[i];
        unsigned idx = ((unsigned)tb->remap[amp[i]] << t->width) | extra;
        int level;
        if (idx >= tb->level_count) return -1;
        level = levels[idx];
        if (sel == 1 || (sel == 2 && dc >= 0)) level = -level;
        dc += level;
        out[i] = (int16_t)level;
    }
    t->dc = dc;
    t->mask = (uint8_t)mask;
    t->phase = (uint8_t)g.phase;
    ++t->frame;
    return 1;
}

size_t k56flex_pcm_tx_g711(k56flex_pcm_tx_t *t, uint8_t *out, size_t n)
{
    size_t done = 0;
    while (done < n) {
        if (t->out_pos == K56FLEX_FRAME_SAMPLES) {
            if (k56flex_pcm_tx_frame(t, t->out) != 1) break;
            t->out_pos = 0;
        }
        {
            int oct = k56flex_g711_from_level((k56flex_law_t)t->law, t->out[t->out_pos]);
            if (oct < 0) break;
            out[done++] = (uint8_t)oct;
            ++t->out_pos;
        }
    }
    return done;
}

/* ---------------------------------------------------------------- rx ---- */

unsigned k56flex_pcm_levels(const k56flex_pcm_tx_t *tx, int16_t *out, unsigned max)
{
    const k56flex_table_t *tb = tx->table;
    unsigned i, n = 0;
    for (i = 0; i < tb->level_count && n + 2 <= max; ++i) {
        int16_t v = k56flex_levels[tb->first_level + i];
        out[n++] = v;
        out[n++] = (int16_t)-v;
    }
    return n;
}

int k56flex_pcm_rx_init(k56flex_pcm_rx_t *rx, const k56flex_pcm_config_t *cfg)
{
    memset(rx, 0, sizeof(*rx));
    return k56flex_pcm_tx_init(&rx->m, cfg, NULL, NULL);
}

int k56flex_pcm_rx_frame(k56flex_pcm_rx_t *rx, const uint8_t octets[K56FLEX_FRAME_SAMPLES],
                         uint8_t bits[K56FLEX_MAX_FRAME_BITS])
{
    k56flex_pcm_tx_t *t = &rx->m;
    const k56flex_table_t *tb = t->table;
    const int16_t *levels = &k56flex_levels[tb->first_level];
    geometry_t g;
    uint8_t stolen[8], sign[8], amp[8], extra[8], neg[8];
    uint8_t raw[K56FLEX_MAX_FRAME_BITS];
    unsigned mask, i, j, nbits = 0, wmask = (1u << t->width) - 1;
    uint64_t rank;
    int32_t dc;
    if (geometry(t, &g) < 0 || partition(t, &g, &mask, stolen, sign) < 0) return -1;
    dc = t->dc;
    for (i = 0; i < 8; ++i) {
        int v = k56flex_level_from_g711((k56flex_law_t)t->law, octets[i]);
        int mag = v < 0 ? -v : v, idx = -1;
        unsigned cls, a;
        for (j = 0; j < tb->level_count; ++j)
            if (levels[j] == mag) { idx = (int)j; break; }
        if (idx < 0) return -1;
        cls = (unsigned)idx >> t->width;
        for (a = 0; a <= t->n && tb->remap[a] != cls; ++a) {}
        if (a > t->n) return -1;
        amp[i] = (uint8_t)a;
        extra[i] = (uint8_t)(idx & wmask);
        if (extra[i] & stolen[i]) return -1;       /* forced-even index */
        neg[i] = v < 0;
        if (!sign[i] && neg[i] != (dc >= 0)) return -1;
        dc += v;
    }
    rank = t->cum[0];
    for (i = 0, j = 0; i < 8; ++i) j += amp[i];
    rank = t->cum[j] + rank_of(t, 8, amp);
    if (g.shift && (rank & ((UINT64_C(1) << g.shift) - 1))) return -1;
    rank >>= g.shift;
    if (g.request < 64 && (rank >> g.request)) return -1;
    for (i = 0; i < (unsigned)g.request; ++i) raw[nbits++] = (uint8_t)((rank >> i) & 1);
    for (i = 0; i < 8; ++i) {
        unsigned usable = t->width - stolen[i], e = extra[i] >> stolen[i];
        if (sign[i]) raw[nbits++] = neg[i];
        for (j = 0; j < usable; ++j) raw[nbits++] = (uint8_t)((e >> j) & 1);
    }
    for (i = 0; i < nbits; ++i) {
        unsigned y = raw[i];
        bits[i] = (uint8_t)(y ^ ((rx->descrambler >> 4) & 1) ^ ((rx->descrambler >> 22) & 1));
        rx->descrambler = (rx->descrambler << 1) | y;
    }
    t->dc = dc;
    t->mask = (uint8_t)mask;
    t->phase = (uint8_t)g.phase;
    ++t->frame;
    return (int)nbits;
}

/* ------------------------------------------------------------ report ---- */

unsigned k56flex_report_field(uint8_t ext, uint16_t report)
{
    unsigned m = ext & 0x3f, pop = 0, i;
    for (i = 0; i < 6; ++i) pop += (m >> i) & 1;
    return (unsigned)ext | (pop << 9) | (((report >> 4) & 1u) << 8);
}

int k56flex_report_header_ok(uint16_t report) { return (report & 0x888f) == 0x8880; }

uint32_t k56flex_report_record(uint8_t ext, uint16_t report)
{
    return (uint32_t)ext | ((uint32_t)report << 8);
}

void k56flex_report_scramble(const uint8_t *in, uint8_t *out, size_t nbits)
{
    uint32_t h = 0;
    size_t i;
    for (i = 0; i < nbits; ++i) {
        unsigned y = (in[i] & 1) ^ ((h >> 4) & 1) ^ ((h >> 22) & 1);
        h = (h << 1) | y;
        out[i] = (uint8_t)y;
    }
}

void k56flex_report_rx_init(k56flex_report_rx_t *rx, unsigned tap)
{
    memset(rx, 0, sizeof(*rx));
    rx->tap = tap;
}

int k56flex_report_rx_bit(k56flex_report_rx_t *rx, int bit)
{
    unsigned y = bit & 1, x, k, i, win = sizeof(rx->window);
    x = y ^ ((rx->hist >> (rx->tap - 1)) & 1) ^ ((rx->hist >> 22) & 1);
    rx->hist = (rx->hist << 1) | y;
    rx->window[rx->pos] = (uint8_t)x;
    rx->pos = (rx->pos + 1) % win;
    ++rx->count;
    if (rx->accepted || rx->count < 23 + 72) return rx->accepted;
    for (i = 0; i < 48; ++i) {
        unsigned a = (rx->pos + win - 1 - i) % win, b = (rx->pos + win - 1 - i - 24) % win;
        if (rx->window[a] != rx->window[b]) return 0;
    }
    for (k = 0; k < 24; ++k) {
        uint32_t rec = 0;
        for (i = 0; i < 24; ++i)
            rec |= (uint32_t)rx->window[(rx->pos + win - 24 + ((k + i) % 24)) % win] << i;
        if (k56flex_report_header_ok((uint16_t)(rec >> 8))) {
            rx->ext = (uint8_t)(rec & 0xff);
            rx->report = (uint16_t)(rec >> 8);
            rx->accepted = 1;
            return 1;
        }
    }
    return 0;
}

/* 1D27 checks at eight-dibit intervals: a header candidate followed by
 * two identical repetitions. A mismatch resumes sliding header search.
 * Keep the original two-bit decision boundary and confirmation latency. */
int k56flex_response_rx_init(k56flex_response_rx_t *rx, unsigned tap)
{
    memset(rx, 0, sizeof(*rx));
    rx->remaining = -1;
    if (tap != 5 && tap != 18) return -1;
    rx->tap = tap;
    return 0;
}

int k56flex_response_rx_dibit(k56flex_response_rx_t *rx, unsigned dibit)
{
    unsigned i, decoded = 0;
    if (rx->tap != 5 && rx->tap != 18) return 0;
    for (i = 0; i < 2; ++i) {
        unsigned y = (dibit >> i) & 1;
        unsigned x = y ^ ((rx->hist >> (rx->tap - 1)) & 1)
                       ^ ((rx->hist >> 22) & 1);
        rx->hist = (rx->hist << 1) | y;
        decoded |= x << i;
    }
    rx->window = (rx->window >> 2) | (decoded << 14);
    if (rx->remaining < 0) {
        rx->word = rx->window;
        if (k56flex_report_header_ok(rx->word)) rx->remaining = 16;
    } else if ((--rx->remaining & 7) == 0) {
        if (rx->word != rx->window) rx->remaining = -1;
        else if (rx->remaining == 0) {
            rx->remaining = 8;
            rx->accepted = 1;
        }
    }
    return rx->accepted;
}

/* Literal bank-8E feedback constellation and labels. Keep distinct from
 * resident 1335's diagonal constellation and from C5CC's alternate slicer. */
unsigned k56flex_feedback_slice(int16_t real, int16_t imag)
{
    static const unsigned labels[8] = {0, 2, 0, 2, 1, 1, 3, 3};
    int64_t a = real < 0 ? -(int64_t)real : real;
    int64_t b = imag < 0 ? -(int64_t)imag : imag;
    int64_t d0 = (a - 12953)*(a - 12953) + b*b;
    int64_t d1 = a*a + (b - 12953)*(b - 12953);
    unsigned k = d1 < d0; /* 5903 retains the first point on a tie. */
    return labels[4*k + 2*(imag < 0) + (real < 0)];
}

unsigned k56flex_feedback_dibit(unsigned *previous_raw, unsigned raw)
{
    static const unsigned table[16] = {
        2, 0, 1, 3, 3, 2, 0, 1, 1, 3, 2, 0, 0, 1, 3, 2
    };
    unsigned result = table[4*(*previous_raw & 3) + (raw & 3)];
    *previous_raw = raw & 3;
    return result;
}

/* 5A54's ZALR adds half an LSB before the signed products. Converting
 * the wide sum to uint32_t gives DSP wrapping without signed C overflow;
 * extracting its high word avoids implementation-defined negative shifts. */
void k56flex_feedback_rotate(int16_t real, int16_t imag,
                             int16_t u, int16_t v, int16_t bias,
                             int16_t *out_real, int16_t *out_imag)
{
    int64_t base = (int64_t)bias * 65536 + 32768;
    uint32_t a = (uint32_t)(base + 2*((int64_t)real*u - (int64_t)imag*v));
    uint32_t b = (uint32_t)(base + 2*((int64_t)imag*u + (int64_t)real*v));
    unsigned ah = a >> 16, bh = b >> 16;
    *out_real = (int16_t)((int32_t)(ah ^ 0x8000) - 32768);
    *out_imag = (int16_t)((int32_t)(bh ^ 0x8000) - 32768);
}

static int16_t feedback_fir_store(int64_t sum)
{
    int64_t rounded = sum + 4096;
    int64_t scaled = rounded >= 0 ? rounded/8192 : -((-rounded + 8191)/8192);
    unsigned word = (uint16_t)scaled;
    return (int16_t)((int32_t)(word ^ 0x8000) - 32768);
}

void k56flex_feedback_fir(const int16_t input[256], unsigned phase,
                          const int16_t coefficients[192], int16_t output[4])
{
    unsigned row, tap;
    for (row = 0; row < 2; ++row) {
        int64_t real = 0, imag = 0;
        for (tap = 0; tap < 48; ++tap) {
            int64_t x = input[(phase - 2 - 2*tap) & 255];
            int64_t y = input[(phase - 1 - 2*tap) & 255];
            int64_t u = coefficients[96*row + tap];
            int64_t v = coefficients[96*row + 48 + tap];
            real += x*u - y*v;
            imag += y*u + x*v;
        }
        output[2*row] = feedback_fir_store(real);
        output[2*row + 1] = feedback_fir_store(imag);
    }
}

static int16_t feedback_adapt_store(int16_t old, int64_t delta)
{
    uint32_t value = (uint32_t)((int64_t)old*65536 + 2*delta + 32768);
    unsigned word = value >> 16;
    return (int16_t)((int32_t)(word ^ 0x8000) - 32768);
}

int k56flex_feedback_adapt(k56flex_feedback_adapt_t *state,
                           int16_t coefficients[192], int16_t history[256],
                           unsigned spacing, unsigned wrap,
                           const int16_t input[256], unsigned input_phase,
                           const int16_t errors[128], unsigned error_phase)
{
    unsigned row, tap = state->tap, index = state->history_index;
    unsigned remaining = state->remaining;
    int64_t ha, hb;
    if (tap > 48 || spacing > 4 || wrap > 3 ||
        (tap == 48 && !spacing) ||
        (tap < 48 && (index >= 256 || spacing >= 256-index))) return -1;
    if (tap == 48) {
        unsigned group, block, j, pos = 0;
        /* 6B54 rebuilds three groups of complex regressors in PM history. */
        for (group = 0; group < 3; ++group)
            for (block = 0; block < 2; ++block)
                for (j = 0; j < spacing; ++j)
                    history[pos++] = input[(input_phase - 2*group + block - 6*j) & 255];
        tap = index = 0;
        remaining = 2;
    }
    ha = history[index]; hb = history[index + spacing];
    for (row = 0; row < 2; ++row) {
        int64_t er = errors[(error_phase - 4 + 2*row) & 127];
        int64_t ei = errors[(error_phase - 3 + 2*row) & 127];
        unsigned u = 96*row + tap, v = u + 48;
        coefficients[u] = feedback_adapt_store(coefficients[u], ha*er + hb*ei);
        coefficients[v] = feedback_adapt_store(coefficients[v], ha*ei - hb*er);
    }
    state->tap = tap + 1;
    /* The DSP keeps an absolute 16-bit PM address. A negative relative
     * index here becomes out of workspace and is rejected on the next call. */
    state->history_index = index + 2*spacing - 1 - (remaining ? 0 : wrap);
    state->remaining = remaining ? remaining - 1 : 2;
    return 0;
}

int k56flex_feedback_predictor_adapt(k56flex_feedback_adapt_t *state,
                                      int16_t coefficients[288], int16_t history[512],
                                      const int16_t input[8192], unsigned input_phase,
                                      const int16_t errors[128], unsigned error_phase)
{
    unsigned row,tap,index=state->history_index,remaining=state->remaining;
    /* Recovered 6CF8/6ADD DAB7 profile: 48 updates, m70=1, m77=24,
     * m78=95, m73=47, m74=0 (Draft 0.23 clause 7.18). The 512-word
     * retained PM window includes words preceding the refresh destination. */
    if((state->tap!=0 && state->tap!=48) || remaining>1 ||
       (state->tap==0 && (index<47 || index>416))) return -1;
    if(state->tap==48) {
        unsigned group,block,j,pos=256;
        /* 6B54 retains AR3's group origin while AR2 copies each block.
         * m6B gives initial -6, m6A gives the -4 copy stride. */
        for(group=0;group<2;++group)
            for(block=0;block<2;++block)
                for(j=0;j<24;++j)
                    history[pos++]=input[(input_phase-6-2*group+block-4*j)&8191];
        index=256; remaining=1;
    }
    for(row=0;row<3;++row) {
        unsigned h=index,r=remaining;
        int64_t er=errors[(error_phase-6+2*row)&127];
        int64_t ei=errors[(error_phase-5+2*row)&127];
        for(tap=0;tap<48;++tap) {
            int64_t ha=history[h],hb=history[h+24];
            unsigned u=96*row+tap,v=u+48;
            coefficients[u]=feedback_adapt_store(coefficients[u],ha*er+hb*ei);
            coefficients[v]=feedback_adapt_store(coefficients[v],ha*ei-hb*er);
            h=h+48-(r ? 0 : 95);
            r=r ? r-1 : 1;
        }
    }
    state->tap=48;
    /* 69B3 subtracts one only after all 48 updates, not after each tap. */
    state->history_index=index+23;
    state->remaining=remaining;
    return 0;
}

#include "k56flex_feedback_tables.h"

int k56flex_feedback_resample(k56flex_feedback_resample_t *state,
                              const int16_t raw[128], unsigned available,
                              unsigned table_phase, unsigned shift,
                              int16_t gain, int16_t bias, int16_t output[256])
{
    int16_t block[256];
    unsigned count, first, i, j, tap;
    if (state->slip < -1 || state->slip > 1 || available > 127 ||
        (!available && state->slip < 0) || table_phase >= 64 ||
        shift < 1 || shift > 15 || state->source_cursor >= 128 ||
        state->output_cursor >= 256 || state->output_available > 128) return -1;
    count = (int)available + state->slip;
    if (count > 128 - state->output_available) return -1;
    first = (state->source_cursor - 2*state->slip) & 127;
    for (i = 0; i < count; ++i) {
        for (j = 0; j < 2; ++j) {
            int64_t dot = 0, rounded, scaled, full;
            unsigned word;
            int32_t r;
            for (tap = 0; tap < 6; ++tap)
                dot += (int64_t)raw[(first + 2*i + j - 2*tap) & 127]
                       * k56flex_feedback_resampler[table_phase][tap];
            rounded = 2*dot + ((int64_t)1 << (shift-1));
            if (rounded < INT32_MIN || rounded > INT32_MAX) return -1;
            scaled = rounded >= 0 ? rounded/((int64_t)1 << shift)
                     : -((-rounded + (((int64_t)1 << shift)-1))/((int64_t)1 << shift));
            word = (uint16_t)scaled;
            r = (int32_t)(word ^ 0x8000) - 32768;
            /* SPM=2 scales P by 16 modulo 32 bits; each of the four
             * APACs saturates A separately (4580, 45A0..45AC). */
            {
                uint32_t product = (uint32_t)((int64_t)16*r*gain);
                int64_t delta = (int64_t)(product ^ UINT32_C(0x80000000)) - INT64_C(2147483648);
                unsigned add;
                full = (int64_t)bias*65536 + 32768;
                for (add = 0; add < 4; ++add) {
                    full += delta;
                    if (full > INT32_MAX) full = INT32_MAX;
                    if (full < INT32_MIN) full = INT32_MIN;
                }
            }
            word = (uint32_t)full >> 16;
            block[2*i+j] = (int16_t)((int32_t)(word ^ 0x8000) - 32768);
        }
    }
    for (i = 0; i < 2*count; ++i) output[(state->output_cursor+i)&255] = block[i];
    state->source_cursor = (first + 2*count) & 127;
    state->output_cursor = (state->output_cursor + 2*count) & 255;
    state->output_available += count;
    state->slip = 0;
    return 0;
}

static int32_t feedback_signed16(unsigned v)
{
    return (int32_t)((v & 65535) ^ 32768) - 32768;
}

static int64_t feedback_floor_shift(int64_t v, unsigned bits)
{
    int64_t divisor = INT64_C(1) << bits;
    return v >= 0 ? v/divisor : -((-v + divisor - 1)/divisor);
}

int k56flex_feedback_timing(uint16_t state[128], const int16_t ring[256],
                            unsigned phase, unsigned tick)
{
    uint16_t s[128];
    int reset;
    int64_t acc = 0, threshold, l;
    memcpy(s, state, sizeof(s));
    reset = (s[0x5d] & 0x10) != 0;
#define G(n) feedback_signed16(s[n])
#define X(k) ((int64_t)ring[(phase-(k)) & 255])
#define HI(v) ((uint16_t)((uint64_t)(v) >> 16))
#define CHECK(v) do { if ((v) < INT32_MIN || (v) > INT32_MAX) return -1; } while (0)
    if (reset) {
        s[0x4f] = s[0x50] = s[0x51] = s[0x52] = 0;
        s[0x4a] = s[0x47] = s[0x48] = s[0x49] = 0;
    } else {
        int64_t a = 32768 + 10000*X(2) - 5000*X(4) - 5000*X(6);
        int64_t b = 8660*(X(3)-X(5));
        int64_t a2 = 32768 + 10000*X(1) - 5000*X(3) - 5000*X(5);
        int64_t b2 = 8660*(X(4)-X(6));
        int64_t u, v, w, fraction, old6c, value;
        const unsigned dest[4] = {0x51,0x52,0x4f,0x50};
        const unsigned src[4] = {0x4b,0x4c,0x4d,0x4e};
        uint16_t smooth[4];
        unsigned j;
        CHECK(a); CHECK(b); CHECK(a2); CHECK(b2);
        CHECK(a+b); CHECK(a-b); CHECK(a2+b2); CHECK(a2-b2);
        s[0x4b]=HI(a+b); s[0x4d]=HI(a-b);
        s[0x4e]=HI(a2+b2); s[0x4c]=HI(a2-b2);
        for (j=0;j<4;++j) {
            value=32768 + 2*(int64_t)G(0x5a)*G(src[j]); CHECK(value);
            value+=2*(int64_t)G(0x56)*G(dest[j]); CHECK(value);
            smooth[j]=HI(value);
        }
        for(j=0;j<4;++j) s[dest[j]]=smooth[j];
        u=2*(int64_t)G(0x4f)*G(0x43)+2*(int64_t)G(0x50)*G(0x44); CHECK(u);
        v=2*(int64_t)G(0x50)*G(0x43)-2*(int64_t)G(0x4f)*G(0x44); CHECK(v);
        s[0x4d]=HI(u); s[0x4e]=HI(v);
        value=4*(int64_t)G(0x51)*G(0x4d); CHECK(value);
        value+=4*(int64_t)G(0x52)*G(0x4e); CHECK(value);
        s[0x4c]=HI(value);
        value=(int64_t)s[0x47]*65536+32768+(int64_t)G(0x4c)*32768; CHECK(value);
        s[0x47]=HI(value);
        a=(int64_t)G(0x40)*65536-(int64_t)G(0x40)*16+(int64_t)G(0x4c)*1024; CHECK(a);
        s[0x40]=HI(a); s[0x4b]=a>0x400000 ? HI(a) : 0x40;
        value=4*((int64_t)G(0x52)*G(0x4d)-(int64_t)G(0x51)*G(0x4e)); CHECK(value);
        w=HI(value); fraction=((uint32_t)value & 0xffc0) >> 6;
        s[0x4d]=(uint16_t)w;
        value=(int64_t)s[0x48]*65536+32768+(int64_t)feedback_signed16(w)*32768; CHECK(value);
        s[0x48]=HI(value);
        old6c=G(0x6c); s[0x6c]=s[0x6b]; s[0x6b]=(uint16_t)w;
        l=(int64_t)G(0x41)*65536+s[0x42];
        l-=feedback_floor_shift(l,12); CHECK(l);
        l+=(int64_t)feedback_signed16(w)*1024; CHECK(l);
        l+=fraction; CHECK(l);
        l-=2*(int64_t)G(0x59)*old6c; CHECK(l);
        s[0x41]=HI(l); s[0x42]=(uint16_t)l; acc=l;
        if(s[0x5d]&1) { memcpy(state,s,sizeof(s)); return 0; }
    }
    threshold=(int64_t)G(0x4b)*G(0x57);
    if (reset || (acc<0 ? -acc : acc)<threshold) acc=0;
    else {
        l=(int64_t)G(0x41)*65536+s[0x42];
        if(l>0) { l-=threshold; s[0x4a]+=3; acc=0x180000; }
        else { l+=threshold; s[0x4a]-=3; acc=-0x180000; }
        s[0x41]=HI(l); s[0x42]=(uint16_t)l;
    }
    acc+=(int64_t)G(0x45)*32;
    s[0x5b]=HI(acc); s[0x5c]=(uint16_t)acc;
    if (!(tick&31)) {
        acc=(int64_t)G(0x45)*512+(int64_t)G(0x4a)*G(0x58)+256;
        s[0x45]=(uint16_t)feedback_floor_shift(acc,9); s[0x4a]=0;
    }
    memcpy(state,s,sizeof(s));
#undef G
#undef X
#undef HI
#undef CHECK
    return 0;
}

void k56flex_feedback_phase(uint16_t state[128])
{
    uint32_t phase = ((uint32_t)state[0x5e] << 16) | state[0x5f];
    uint32_t correction = ((uint32_t)state[0x5b] << 16) | state[0x5c];
    int update = (state[0x5d] & 0x20) || !(state[0x5d] & 0x10);
    if (update) {
        phase -= correction;
        state[0x5e] = phase >> 16;
        state[0x5f] = phase;
        state[0x60] = phase >> 16;
    }
    if (!update || !state[0x3e]) {
        uint16_t difference = (phase >> 16) - state[0x64];
        state[0x3e] = (difference & 0xf000) - (state[0x3f] & 0xf000);
        state[0x3f] = difference;
    }
    state[0x5b] = state[0x5c] = state[0x65] = state[0x66] = 0;
}

void k56flex_feedback_timing_init(uint16_t state[128], int symbol_loop)
{
    static const unsigned cleared[] = {0x45,0x40,0x41,0x42,0x4a,0x47,0x48,0x49,
                                       0x44,0x4f,0x50,0x51,0x52};
    unsigned i;
    state[0x56]=0x7ef9; state[0x57]=symbol_loop ? 0x0324 : 0x0140;
    state[0x58]=symbol_loop ? 0x0800 : 0x1800; state[0x59]=0x0180;
    for(i=0;i<sizeof(cleared)/sizeof(cleared[0]);++i) state[cleared[i]]=0;
    state[0x43]=0x7fff; state[0x5a]=0x20c4;
}

unsigned k56flex_feedback_block_count(uint16_t state[128])
{
    uint32_t accumulator=((uint32_t)state[9]+state[0x42]) << 12;
    unsigned i, blocks;
    for(i=0;i<4;++i) {
        int64_t trial=(int64_t)accumulator-((uint32_t)state[0x45]<<15);
        accumulator=trial>=0 ? (uint32_t)(2*trial+1) : accumulator*2;
    }
    state[0x42]=accumulator>>16;
    blocks=accumulator&65535;
    state[0x2b]=blocks*state[0x45];
    state[0x43]=blocks-1;
    return blocks;
}

void k56flex_feedback_residual(int16_t raw[2], const int16_t predicted[2], int bypass)
{
    unsigned i;
    if (bypass) return;
    for(i=0;i<2;++i) {
        unsigned word=(uint16_t)((int32_t)raw[i]-predicted[i]);
        raw[i]=(int16_t)((int32_t)(word^32768)-32768);
    }
}

void k56flex_feedback_predictor(int16_t lane[2], const int16_t source[2],
                                const int16_t phasor[2], int bypass,
                                int16_t prediction[2], int16_t error[2])
{
    uint32_t a, b;
    k56flex_feedback_rotate(source[0],source[1],phasor[0],phasor[1],0,
                            &prediction[0],&prediction[1]);
    k56flex_feedback_residual(lane,prediction,bypass);
    /* 4ADC..4AF0 multiplies the residual by the conjugate phasor.
     * Subtract products directly so -32768 need not be negated in int16. */
    a=(uint32_t)(32768+2*((int64_t)lane[0]*phasor[0]+(int64_t)lane[1]*phasor[1]));
    b=(uint32_t)(32768+2*((int64_t)lane[1]*phasor[0]-(int64_t)lane[0]*phasor[1]));
    error[0]=(int16_t)feedback_signed16(a>>16);
    error[1]=(int16_t)feedback_signed16(b>>16);
}

void k56flex_feedback_predictor_fir(const int16_t input[8192], unsigned phase,
                                    const int16_t coefficients[288], int16_t output[6])
{
    unsigned row,tap;
    for(row=0;row<3;++row) {
        int64_t real=0,imag=0;
        unsigned word;
        for(tap=0;tap<48;++tap) {
            int64_t x=input[(phase-2-2*tap)&8191],y=input[(phase-1-2*tap)&8191];
            int64_t u=coefficients[96*row+tap],v=coefficients[96*row+48+tap];
            real+=x*u-y*v; imag+=y*u+x*v;
        }
        word=(uint16_t)feedback_floor_shift(real+131072,18);
        output[2*row]=(int16_t)feedback_signed16(word);
        word=(uint16_t)feedback_floor_shift(imag+131072,18);
        output[2*row+1]=(int16_t)feedback_signed16(word);
    }
}

static int64_t feedback_signed32(uint32_t v)
{
    return v<=INT32_MAX ? (int64_t)v : (int64_t)v-4294967296LL;
}

static uint32_t feedback_divide_steps(uint32_t acc, uint16_t divisor, unsigned steps)
{
    while(steps--) {
        uint32_t difference=acc-((uint32_t)divisor<<15);
        acc=(difference&0x80000000U) ? acc<<1 : (difference<<1)|1;
    }
    return acc;
}

/* MICA 6831/6890 coefficients at PM0320..0326 in the retained image.
 * These polynomial and restoring-division operations are firmware evidence
 * for Draft 0.23 clause 7.18's predictor, not generic atan approximations. */
static unsigned feedback_angle(int x, int y)
{
    unsigned quadrant=(y<0 ? 2 : 0)+((x<0)!=(y<0));
    unsigned ax=x<0 ? -x : x, ay=y<0 ? -y : y;
    unsigned base=quadrant*128,ratio;
    uint32_t acc;
    int square,poly;
    if(ax==ay) return base+64;
    if(ay>ax) {
        if(!(quadrant&1)) base+=64;
        if(!ax) return (base&384) ? base&384 : 128;
        ratio=feedback_divide_steps(ax<<16,ay,15)&65535;
    } else {
        if(quadrant&1) base+=64;
        if(!ay) return (base&384) ? (base&384)+128 : 0;
        ratio=feedback_divide_steps(ay<<16,ax,15)&65535;
    }
    square=(2*(uint32_t)ratio*ratio)>>16;
    acc=((uint32_t)(uint16_t)-107<<16)+32768;
    acc+=(uint32_t)(2*22404*(int64_t)(int16_t)ratio);
    acc+=(uint32_t)(2*-5984*(int64_t)(int16_t)square);
    poly=feedback_signed16(acc>>16);
    acc=32768+(uint32_t)(256*(int64_t)poly);
    poly=feedback_signed16(acc>>16);
    if(base&64) poly=64-poly;
    return (base+poly)&511;
}

static int feedback_small_angle(int x, int y)
{
    uint32_t acc,ratio;
    int square,poly;
    unsigned ay=y<0 ? -y : y;
    if(!y) return 0;
    acc=(uint32_t)(ay*32768LL)-(uint32_t)(6493*(int64_t)x);
    if(!(acc&0x80000000U)) poly=128;
    else {
        acc=feedback_divide_steps(ay<<16,(uint16_t)x,16);
        ratio=((acc+1)>>1)&32767;
        square=(2*ratio*ratio)>>16;
        acc=((uint32_t)(uint16_t)-1<<16)+32768;
        acc+=(uint32_t)(2*10446*(int64_t)ratio);
        acc+=(uint32_t)(2*-1003*(int64_t)(int16_t)square);
        poly=feedback_signed16(acc>>16);
        poly=feedback_signed16((32768+(uint32_t)(4096*(int64_t)poly))>>16);
    }
    return y<0 ? -poly : poly;
}

uint32_t k56flex_feedback_predictor_angle(const uint16_t correlation[4])
{
    uint32_t real=((uint32_t)correlation[0]<<16)|correlation[1];
    uint32_t imag=((uint32_t)correlation[2]<<16)|correlation[3];
    int64_t ar=feedback_signed32(real),ai=feedback_signed32(imag),maximum;
    unsigned shifts=0;
    int x,y,angle;
    ar=ar<0 ? -ar : ar; ai=ai<0 ? -ai : ai;
    /* ABS of INT32_MIN wraps with OVM clear; compare remains signed. */
    ar=feedback_signed32((uint32_t)ar); ai=feedback_signed32((uint32_t)ai);
    maximum=ar>ai ? ar : ai;
    while(shifts<15 && maximum &&
          !((((uint32_t)maximum>>31)^((uint32_t)maximum>>30))&1)) {
        maximum=feedback_signed32((uint32_t)maximum<<1); ++shifts;
    }
    real=(uint32_t)feedback_floor_shift(feedback_signed32(real),3);
    imag=(uint32_t)feedback_floor_shift(feedback_signed32(imag),3);
    /* ROL rotates through carry rather than shifting. ADD(S) of each
     * disjoint high/low input pair has left carry clear. */
    {
        unsigned i,rc=0,ic=0;
        for(i=0;i<=shifts;++i) {
            unsigned nr=real>>31,ni=imag>>31;
            real=(real<<1)|rc; imag=(imag<<1)|ic;
            rc=nr; ic=ni;
        }
    }
    x=feedback_signed16(real>>16); y=feedback_signed16(imag>>16);
    ai=feedback_signed32(imag); ai=ai<0 ? -ai : ai;
    if(feedback_floor_shift(feedback_signed32(real),2)>ai)
        angle=feedback_small_angle(x,y)*8;
    else {
        angle=feedback_angle(x,y);
        if(angle>=256) angle-=512;
        angle*=64;
    }
    return (uint32_t)(uint16_t)angle<<16;
}

void k56flex_feedback_predictor_correlate(uint16_t correlation[4],
                                         const int16_t input[6], const int16_t reference[6])
{
    /* MICA 4B59..4B86, mode 15: accumulate three complex pairs into
     * D930..D933 before 6D4E converts that correlation to a loop coefficient. */
    int64_t dot=0,cross=0;
    uint32_t terms[2];
    unsigned i;
    for(i=0;i<6;i+=2) {
        dot+=(int64_t)input[i]*reference[i]+(int64_t)input[i+1]*reference[i+1];
        cross+=(int64_t)input[i]*reference[i+1]-(int64_t)input[i+1]*reference[i];
    }
    terms[0]=(uint32_t)(2*dot); terms[1]=(uint32_t)(2*cross);
    for(i=0;i<2;++i) {
        uint32_t acc=((uint32_t)correlation[2*i]<<16)|correlation[2*i+1];
        int64_t term=terms[i]<=INT32_MAX ? (int64_t)terms[i] : (int64_t)terms[i]-4294967296LL;
        acc+=(uint32_t)feedback_floor_shift(term,5);
        correlation[2*i]=acc>>16; correlation[2*i+1]=acc;
    }
}

uint32_t k56flex_feedback_predictor_increment(uint16_t state[128])
{
    /* MICA 6D93..6DA8, DP119. SPH observes the SPM-scaled product;
     * the low-word multiply uses unsigned T even for negative errors. */
    uint32_t low=(uint32_t)state[0x36]*state[0x34];
    uint32_t high=(uint32_t)(feedback_signed16(state[0x36])*feedback_signed16(state[0x33]));
    uint32_t term=high+(low>>16)+4;
    int64_t signed_term=term<=INT32_MAX ? (int64_t)term : (int64_t)term-4294967296LL;
    uint32_t integral=((uint32_t)state[0x38]<<16)|state[0x39];
    integral+=(uint32_t)feedback_floor_shift(signed_term,3);
    state[0x38]=integral>>16; state[0x39]=integral;
    low=((uint32_t)state[0x37]*state[0x34])<<4;
    high=(uint32_t)(feedback_signed16(state[0x37])*feedback_signed16(state[0x33]))<<4;
    state[0x55]=low>>16;
    return integral+high+(low>>16);
}

void k56flex_feedback_predictor_phase(uint16_t state[128], uint32_t increment,
                                      const int16_t cosine[512])
{
    uint32_t phase=((uint32_t)state[0x3a]<<16)|state[0x3b];
    unsigned index;
    phase+=increment;
    state[0x3a]=phase>>16; state[0x3b]=phase;
    index=((phase>>16)+64)>>7;
    state[0x3c]=(uint16_t)cosine[index&511];
    state[0x3d]=(uint16_t)cosine[(index+384)&511];
}

void k56flex_feedback_predictor_control15(uint16_t state[128], uint16_t correlation[4],
                                          const int16_t input[6], const int16_t reference[6],
                                          const int16_t cosine[512])
{
    uint32_t increment=((uint32_t)state[0x38]<<16)|state[0x39];
    if(!state[0x57]) return;
    if(state[0x2c]) {
        unsigned timer=state[0x4e];
        k56flex_feedback_predictor_correlate(correlation,input,reference);
        state[0x4e]=timer-1;
        if(!timer) {
            uint32_t angle=k56flex_feedback_predictor_angle(correlation);
            uint32_t low,high;
            state[0x33]=angle>>16; state[0x34]=angle;
            state[0x4e]=40;
            memset(correlation,0,4*sizeof(*correlation));
            increment=k56flex_feedback_predictor_increment(state);
            /* Mode 15's 4BA9..4BB0 repeats the proportional contribution. */
            low=((uint32_t)state[0x37]*state[0x34])<<4;
            high=(uint32_t)(feedback_signed16(state[0x37])*feedback_signed16(state[0x33]))<<4;
            state[0x55]=low>>16;
            increment+=high+(low>>16);
        }
    }
    k56flex_feedback_predictor_phase(state,increment,cosine);
}

void k56flex_feedback_predictor_control(uint16_t state[128],
                                        const int16_t input[6], const int16_t reference[6],
                                        const int16_t cosine[512])
{
    uint32_t increment=((uint32_t)state[0x38]<<16)|state[0x39];
    if(!state[0x57]) return;
    if(state[0x2c]) {
        /* Original 4B07..4B57, all EE9B modes except 15 (Draft 0.23
         * clause 7.18). SATL is a TREG1-selected arithmetic shift here,
         * not an accumulator clamp. PMST.TRM=1 keeps the explicit shifts
         * installed from words 2C and 32 while the multiply T register changes. */
        uint32_t energy=0,low,term,filtered;
        int64_t cross=0,level;
        unsigned i,gain=state[0x35];
        for(i=0;i<6;++i) energy+=(uint32_t)((int64_t)input[i]*input[i]);
        low=(uint32_t)gain*(energy&65535);
        state[0x56]=low>>16;
        term=(uint32_t)gain*(energy>>16)+(low>>16);
        term=0x300-term;
        term=(uint32_t)feedback_floor_shift(feedback_signed32(term),state[0x2c]&15)+0x1000;
        level=feedback_signed32(term);
        if(level>32767) level=32767;
        if(level<8) level=8;
        state[0x55]=(uint16_t)level;
        term=(uint32_t)gain*(uint32_t)level;
        level=feedback_signed32(term);
        if(level>0x7fff000) level=0x7fff000;
        if(level<0x10000) level=0x10000;
        state[0x35]=((uint32_t)level<<4)>>16;
        for(i=0;i<6;i+=2)
            cross+=(int64_t)input[i]*reference[i+1]-(int64_t)input[i+1]*reference[i];
        term=(uint32_t)(2*cross)<<4;
        state[0x2e]=term>>16; state[0x2f]=term;
        low=((uint32_t)state[0x2d]*state[0x31])<<1;
        state[0x55]=low>>16;
        filtered=(uint32_t)(feedback_signed16(state[0x2d])*feedback_signed16(state[0x30]))<<1;
        filtered+=low>>16;
        filtered+=(uint32_t)feedback_floor_shift(feedback_signed32(term),state[0x32]&15);
        state[0x30]=filtered>>16; state[0x31]=filtered;
        low=((uint32_t)state[0x35]*state[0x31])<<1;
        state[0x55]=low>>16;
        term=(uint32_t)(feedback_signed16(state[0x35])*feedback_signed16(state[0x30]))<<1;
        term=(term+(low>>16))<<7;
        state[0x33]=term>>16; state[0x34]=term;
        increment=k56flex_feedback_predictor_increment(state);
    }
    k56flex_feedback_predictor_phase(state,increment,cosine);
}

int k56flex_feedback_predictor_block(uint16_t state[128],
                                      k56flex_feedback_adapt_t *adaptation,
                                      int16_t coefficients[288], int16_t history[512],
                                      const int16_t source[8192], unsigned source_phase,
                                      int16_t raw[6], int16_t errors[128], unsigned error_phase,
                                      unsigned mode, int bypass, const int16_t cosine[512],
                                      uint16_t correlation[4], int16_t prediction[6])
{
    int16_t filtered[6],original[6],phasor[2];
    unsigned pair;
    /* 6C78 -> 4A8F -> 4AF3, Draft 0.23 clause 7.18. Adaptation uses
     * retained errors before this block overwrites the next three pairs. */
    if(adaptation && k56flex_feedback_predictor_adapt(adaptation,coefficients,history,
                                                      source,source_phase,errors,error_phase)) return -1;
    k56flex_feedback_predictor_fir(source,source_phase,coefficients,filtered);
    memcpy(original,raw,sizeof(original));
    phasor[0]=(int16_t)feedback_signed16(state[0x3c]);
    phasor[1]=(int16_t)feedback_signed16(state[0x3d]);
    for(pair=0;pair<3;++pair) {
        int16_t error[2];
        k56flex_feedback_predictor(&raw[2*pair],&filtered[2*pair],phasor,bypass,
                                    &prediction[2*pair],error);
        errors[(error_phase+2*pair)&127]=error[0];
        errors[(error_phase+2*pair+1)&127]=error[1];
    }
    for(pair=0;pair<6;++pair) state[0x4f+pair]=(uint16_t)prediction[pair];
    state[0x55]=64;
    if(mode==15) k56flex_feedback_predictor_control15(state,correlation,prediction,original,cosine);
    else k56flex_feedback_predictor_control(state,prediction,raw,cosine);
    return 0;
}

int k56flex_feedback_predictor_run(uint16_t state[128],
                                    k56flex_feedback_predictor_cursors_t *cursors,
                                    k56flex_feedback_adapt_t *adaptation,
                                    int16_t coefficients[288], int16_t history[512],
                                    const int16_t source[8192], int16_t raw[128],
                                    int16_t errors[128], unsigned mode, int bypass,
                                    const int16_t cosine[512], uint16_t correlation[4])
{
    unsigned blocks,block,i;
    /* Original 6C50, Draft 0.23 clause 7.18. Source production is a
     * separate caller responsibility; preserve the +4 / six-word cadence. */
    if(state[0x44]!=2 || state[0x45]!=3 || state[9]>45 || state[0x42]>2 ||
       cursors->source>=8192 || cursors->raw>=128 || cursors->error>=128 || cursors->output>=8 ||
       (adaptation && ((adaptation->tap!=0 && adaptation->tap!=48) ||
          adaptation->remaining>1 || (adaptation->tap==0 &&
          (adaptation->history_index<47 || adaptation->history_index>416))))) return -1;
    blocks=k56flex_feedback_block_count(state);
    for(block=0;block<blocks;++block) {
        int16_t lane[6],prediction[6];
        state[0x46]+=2;
        cursors->source=(cursors->source+4)&8191;
        for(i=0;i<6;++i) lane[i]=raw[(cursors->raw+i)&127];
        if(k56flex_feedback_predictor_block(state,adaptation,coefficients,history,
                                             source,cursors->source,lane,errors,cursors->error,
                                             mode,bypass,cosine,correlation,prediction)) return -1;
        for(i=0;i<6;++i) raw[(cursors->raw+i)&127]=lane[i];
        cursors->raw=(cursors->raw+6)&127;
        cursors->error=(cursors->error+6)&127;
        cursors->output=(cursors->output+6)&7;
        --state[0x43];
    }
    return (int)blocks;
}

void k56flex_feedback_source_pair(int16_t ring[8192], unsigned *cursor,
                                  const int16_t pair[2], const int16_t phasor[2])
{
    int64_t x=pair[0],y=pair[1],u=phasor[0],v=phasor[1];
    unsigned p=*cursor&8191;
    ring[p]=feedback_fir_store(x*u-y*v);
    ring[(p+1)&8191]=feedback_fir_store(x*v+y*u);
    *cursor=(p+2)&8191;
}

void k56flex_feedback_startup_bits(uint16_t state[128], uint16_t input,
                                   int scramble)
{
    /* Original 8F6F/8F74, DP118. Draft 0.23 clause 4.12 transmit
     * parameter-record path. Preserve the firmware's word ordering. */
    uint32_t history=((uint32_t)state[1]<<16)|state[2];
    if(scramble) {
        state[0x79]=state[2];
        state[2]=state[1];
        state[1]=(uint16_t)((history>>9)^(history>>14)^input);
    } else {
        state[2]=state[1];
        state[1]=input;
    }
}

void k56flex_feedback_extended_bits(uint16_t state[128], uint16_t input)
{
    /* Original BE68. Alternate B3BD profile; Draft 0.23 clause 4.12
     * transmit record path. Each masked recurrence uses the updated word. */
    uint32_t history=((uint32_t)state[1]<<16)|state[2];
    uint16_t word;
    state[0x79]=state[2];
    state[2]=state[1];
    state[0x12]=(uint16_t)(history>>9);
    word=input^state[0x12]^(state[1]>>11);
    word^=(uint16_t)((word<<5)&0x03e0);
    word^=(uint16_t)((word<<5)&0x7c00);
    word^=(uint16_t)((word<<5)&0x8000);
    state[1]=word;
}

int k56flex_feedback_startup_samples(uint16_t state[128], unsigned symbols,
                                    const int16_t history[64], unsigned *history_cursor,
                                    const int16_t *banks, unsigned bank_words,
                                    int16_t output[128], unsigned *output_cursor)
{
    /* Bounded whole 8270 datapath, Draft 0.23 clause 4.12. Banks are
     * normalized from its DM pointer table into phase-major PM coefficients.
     * Hardware output counter and physical cursor stores remain caller-owned. */
    uint16_t next[128],address;
    unsigned h=*history_cursor,o=*output_cursor,taps=state[0x29]+1u;
    int count;
    if(h>=64 || o>=128 || !state[0x27] || state[0x27]>32767
       || state[0x28]>state[0x27] || state[0x4d]>=state[0x27]
       || taps>64 || bank_words<(unsigned)state[0x27]*taps) return -1;
    memcpy(next,state,sizeof(next));
    count=k56flex_feedback_startup_sample_count(next,symbols);
    if(count<=0) return -1; /* Original BRC=count-1 cannot represent zero. */
    for(int j=0;j<count;++j) {
        k56flex_feedback_startup_phase(next,&h,&address);
        k56flex_feedback_startup_fir(history,h,banks+next[0x4d]*taps,taps,&output[o]);
        o=(o+1)&127;
    }
    memcpy(state,next,sizeof(next));
    *history_cursor=h; *output_cursor=o;
    return count;
}

int k56flex_startup_tx_init(k56flex_startup_tx_t *tx, unsigned width,
                            unsigned producer, unsigned differential,
                            uint16_t amplitude, uint16_t gain)
{
    /* Draft 0.23 clause 4.12. The order is the module's own (D698..D6A2). */
    uint16_t *s=tx->state;
    if(!width || width>15 || producer>2 || differential>1) return -1;
    memset(tx,0,sizeof(*tx));
    tx->producer=producer; tx->differential=differential;
    s[0x1f]=(uint16_t)width; s[0x3e]=amplitude; s[0x11]=gain;
    s[0x21]=(uint16_t)(producer==0 ? 0x8f6f : producer==1 ? 0x8f74 : 0xbe68); /* B3CA/B3BD */
    /* 8F23(0): PM 8F96[0] -> 8F9C, twelve words to 27..32. */
    memcpy(&s[0x27],k56flex_startup_profile0,sizeof(k56flex_startup_profile0));
    /* 94BA(1): 2C = select, 2D/2E from PM 8FE4 + 4*2B + 2*2C, 94B6 sets
     * 13 = 2D, 4F = 2*2B + 2C and 2F..31 from PM 2ECD + 3*4F. */
    s[0x2c]=1;
    s[0x2d]=(uint16_t)(K56FLEX_STARTUP_CARRIER_PM+6);
    s[0x2e]=(uint16_t)(K56FLEX_STARTUP_CARRIER_PM+14);
    s[0x13]=s[0x2d];
    s[0x4f]=1;
    memcpy(&s[0x2f],k56flex_startup_select1_words,sizeof(k56flex_startup_select1_words));
    /* 82B4: history cleared, read cursor one word behind the writer. */
    tx->history_write=0; tx->history_read=63;
    s[0x4c]=0;
    s[0x4d]=s[0x55]=(uint16_t)(s[0x27]-s[0x28]);
    s[0x4e]=0x6116;
    /* 83A3: E504 pointer table, 46-word banks. */
    s[0x2a]=0xe504; s[0x29]=0x2d;
    return 0;
}

int k56flex_startup_tx_symbol(k56flex_startup_tx_t *tx, uint16_t word,
                              int16_t pcm[4], int *consumed)
{
    k56flex_startup_tx_t next=*tx;
    int16_t pair[2];
    unsigned start=next.output_write,carrier;
    int used,count;
    if(next.state[0x13]<K56FLEX_STARTUP_CARRIER_PM) return -1;
    carrier=next.state[0x13]-K56FLEX_STARTUP_CARRIER_PM;
    if(carrier+1>=sizeof(k56flex_startup_carriers)/sizeof(k56flex_startup_carriers[0])) return -1;
    used=k56flex_feedback_startup_take(next.state,word,next.producer);
    if(used<0) return -1;
    k56flex_feedback_startup_symbol(next.state,next.differential);
    pair[0]=(int16_t)next.state[0x0f]; pair[1]=(int16_t)next.state[0x10];
    if(k56flex_feedback_startup_rotate(next.state,pair,&k56flex_startup_carriers[carrier])
       || k56flex_feedback_startup_output(next.state,pair,next.history,&next.history_write))
        return -1;
    next.state[0x0f]=(uint16_t)pair[0]; next.state[0x10]=(uint16_t)pair[1];
    count=k56flex_feedback_startup_samples(next.state,1,next.history,&next.history_read,
                                           k56flex_startup_fir_banks,460,
                                           next.output,&next.output_write);
    if(count<3 || count>4) return -1;
    for(int i=0;i<count;++i) pcm[i]=next.output[(start+i)&127];
    next.samples+=(unsigned)count;
    *tx=next; *consumed=used;
    return count;
}

int k56flex_feedback_startup_phase(uint16_t state[128], unsigned *cursor,
                                  uint16_t *bank_address)
{
    /* Original 828B..8299, Draft 0.23 clause 4.12 sample generator.
     * Word 4D is signed: 82B4 initializes it to word 27 minus word 28.
     * One boundary crossing advances the history by one complex pair. */
    int phase=feedback_signed16(state[0x4d]);
    unsigned limit=state[0x27],increment=state[0x28];
    if(*cursor>=64 || !limit || limit>32767 || increment>limit
       || phase>= (int)limit || phase+(int)increment < -32768) return -1;
    phase+=increment;
    if(phase>=(int)limit) {
        phase-=limit;
        *cursor=(*cursor+2)&63;
    }
    state[0x4d]=(uint16_t)phase;
    *bank_address=(uint16_t)(phase+state[0x2a]);
    return 0;
}

int k56flex_feedback_startup_fir(const int16_t history[64], unsigned cursor,
                                const int16_t *coefficients, unsigned taps,
                                int16_t *sample)
{
    /* Original 829E..82A3, SPM=0, OVM=0. Draft 0.23 clause 4.12
     * transmit sample generation. MADS walks PM coefficients forward and
     * BR0 history backward; final LTA adds the last pending product. */
    int64_t sum=0,scaled;
    if(cursor>=64 || !taps || taps>64) return -1;
    for(unsigned j=0;j<taps;++j)
        sum+=(int64_t)history[(cursor-j)&63]*coefficients[j];
    scaled=sum>=0 ? sum/4096 : -((-sum+4095)/4096);
    *sample=(int16_t)feedback_signed16((uint16_t)scaled);
    return 0;
}

int k56flex_feedback_startup_sample_count(uint16_t state[128], unsigned symbols)
{
    /* Original 8274..827E at SPM=0, five SUBC steps. Draft 0.23
     * clause 4.12 startup transmit sample accounting. Bounded nonnegative
     * operands keep the original five-bit quotient representable. */
    unsigned product,total,count;
    if(symbols>32767 || !state[0x28] || state[0x28]>32767
       || state[0x27]>32767 || state[0x4c]>=state[0x28]) return -1;
    product=symbols*state[0x27];
    if(product>32767) return -1;
    total=product+state[0x4c]; count=total/state[0x28];
    if(count>31) return -1;
    state[0x57]=product;
    state[0x4c]=total%state[0x28];
    state[0x55]=(uint16_t)(count-1);
    return (int)count;
}

int k56flex_feedback_startup_output(const uint16_t state[128],
                                   const int16_t pair[2], int16_t ring[64],
                                   unsigned *cursor)
{
    /* Original 9367, SPM=0 after overlay 88 43D1. Draft 0.23 clause
     * 4.12 transmit path. Logical order is imaginary then real, with
     * SACH shift 2 (no rounding or saturation). Physical AR3 is BR0. */
    int gain=feedback_signed16(state[0x11]);
    unsigned p=*cursor;
    if(p>=64) return -1;
    for(unsigned j=0;j<2;++j) {
        int64_t product=(int64_t)gain*pair[1-j];
        int64_t scaled=product>=0 ? product/16384 : -((-product+16383)/16384);
        ring[(p+j)&63]=(int16_t)feedback_signed16((uint16_t)scaled);
    }
    *cursor=(p+2)&63;
    return 0;
}

int k56flex_feedback_startup_rotate(uint16_t state[128], int16_t pair[2],
                                   const int16_t phasor[2])
{
    /* Overlay 88, 43D1 reached through 2C27. Draft 0.23 clause 4.12
     * startup transmit path: SPM=1 complex rotation, then phase walk.
     * Caller supplies the two original PM words at state[13]. */
    int16_t real,imag;
    if(state[0x2d]>=state[0x2e] || state[0x13]<state[0x2d]
       || state[0x13]>=state[0x2e]
       || ((state[0x13]-state[0x2d])&1)) return -1;
    k56flex_feedback_rotate(pair[0],pair[1],phasor[0],phasor[1],0,&real,&imag);
    pair[0]=real; pair[1]=imag;
    state[0x13]=(uint16_t)(state[0x13]+2);
    if(state[0x13]>=state[0x2e]) state[0x13]=state[0x2d];
    return 0;
}

int k56flex_feedback_startup_take(uint16_t state[128], uint16_t input,
                                 unsigned producer)
{
    /* Bounded 90FF extraction, Draft 0.23 clause 4.12. Caller supplies
     * the next queued word when the remaining bits do not cover the symbol.
     * Queue refill and multiword (>15-bit) paths remain caller-owned. */
    unsigned width=state[0x1f],remaining=state[4];
    uint32_t bits;
    int consumed;
    if(!width || width>15 || remaining>15 || producer>2) return -1;
    consumed=width>remaining;
    if(consumed) {
        if(producer==2) k56flex_feedback_extended_bits(state,input);
        else k56flex_feedback_startup_bits(state,input,producer);
        bits=((uint32_t)state[1]<<16)|state[2];
        bits>>=16-remaining;
    } else bits=(uint32_t)state[1]>>(16-remaining);
    state[7]=(uint16_t)(bits&((1u<<width)-1));
    state[8]=0;
    state[0x12]=1; /* 9157: scratch for the 1<<width mask, after any producer */
    state[4]=(remaining-width)&15;
    return consumed;
}

void k56flex_feedback_startup_symbol(uint16_t state[128], int differential)
{
    /* Original 97A2/97A8 -> 97C3, SPM=0. Retained DM72A0 bases and
     * PM97BB rotations; Draft 0.23 clause 4.12 parameter-record transmit path.
     * Amplitude word 3E is configured separately by original 97B6. */
    static const int8_t base[4][2]={{1,1},{-3,1},{1,-3},{-3,-3}};
    static const int8_t rotation[4][2]={{1,0},{0,-1},{-1,0},{0,1}};
    unsigned word=state[7],quadrant=word&3,index=(word>>2)&3;
    int u,v,a,b;
    if(differential) {
        quadrant=(quadrant+state[0x0c])&3;
        state[7]=(word&12)|quadrant;
    }
    state[0x0c]=quadrant;
    u=feedback_signed16((unsigned)(feedback_signed16(state[0x3e])*base[index][0]));
    v=feedback_signed16((unsigned)(feedback_signed16(state[0x3e])*base[index][1]));
    a=rotation[quadrant][0]; b=rotation[quadrant][1];
    state[0x0f]=(uint16_t)(u*a-v*b);
    state[0x10]=(uint16_t)(v*a+u*b);
}

int k56flex_feedback_source_init(uint16_t state[128], unsigned profile)
{
    if(profile>=11) return -1;
    state[0x1b]=state[0x1c]=k56flex_source_profiles[profile][0];
    state[0x1d]=k56flex_source_profiles[profile][1];
    state[0x1e]=k56flex_source_profiles[profile][2];
    return 0;
}

int k56flex_feedback_source_emit_scaled(uint16_t state[128], int16_t ring[8192],
                                         unsigned *cursor, const int16_t pair[2], unsigned spm)
{
    unsigned phase=state[0x1b],p=*cursor,shift;
    int64_t x=pair[0],y=pair[1],u,v;
    if(*cursor>=8192 || phase<0x6490 || phase>0x650a || (phase&1) || spm>2) return -1;
    /* Original 4A6C, PM carrier lookup and phase-cycle reset.
     * Draft 0.23 clause 7.18. 4A8A's cursor store is in RETCD's delay
     * slots, so it executes on non-boundary calls too. */
    shift=spm==2 ? 4 : spm;
    u=k56flex_source_carrier[phase-0x6490]; v=k56flex_source_carrier[phase-0x6490+1];
    ring[p]=(int16_t)feedback_signed16((uint16_t)feedback_floor_shift((x*u-y*v)*(INT64_C(1)<<shift)+8192,14));
    ring[(p+1)&8191]=(int16_t)feedback_signed16((uint16_t)feedback_floor_shift((x*v+y*u)*(INT64_C(1)<<shift)+8192,14));
    *cursor=(p+2)&8191;
    state[0x1b]+=state[0x1e];
    if(state[0x1b]!=state[0x1d]) return 0;
    state[0x1b]=state[0x1c];
    return 1;
}

int k56flex_feedback_source_emit(uint16_t state[128], int16_t ring[8192],
                                  unsigned *cursor, const int16_t pair[2])
{
    return k56flex_feedback_source_emit_scaled(state,ring,cursor,pair,1);
}

/* ------------------------------------------------------------ V.8bis ---- */

static const uint8_t v8bis_template[16] = {
    0x11, 0xc9, 0x80, 0x80, 0x81, 0xc2, 0x09, 0xb5, 0x02, 0x00, 0x94, 0x81, 0x81, 0x42, 0x46, 0xc0
};

size_t k56flex_v8bis_payload(k56flex_v8bis_msg_t type, int v90_capable, int mu_law,
                             uint8_t out[16])
{
    switch (type) {
    case K56FLEX_V8BIS_ACK1: out[0] = 0x14; return 1;
    case K56FLEX_V8BIS_NAK1: out[0] = 0x18; return 1;
    default: break;
    }
    memcpy(out, v8bis_template, 16);
    out[0] = type == K56FLEX_V8BIS_CL ? 0x12 : 0x11;
    out[12] = v90_capable ? 0x83 : 0x81;
    if (mu_law) out[15] |= 0x20;
    return 16;
}

uint16_t k56flex_v8bis_fcs(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xffff;
    size_t i;
    unsigned k;
    for (i = 0; i < len; ++i)
        for (k = 0; k < 8; ++k) {
            unsigned fb = (crc ^ (data[i] >> k)) & 1;
            crc >>= 1;
            if (fb) crc ^= 0x8408;
        }
    return (uint16_t)(crc ^ 0xffff);
}

size_t k56flex_v8bis_frame(const uint8_t *payload, size_t len, uint8_t *out)
{
    uint16_t fcs = k56flex_v8bis_fcs(payload, len);
    size_t n = 0;
    out[n++] = 0x7e; out[n++] = 0x7e; out[n++] = 0x7e;
    memcpy(out + n, payload, len);
    n += len;
    out[n++] = (uint8_t)fcs;
    out[n++] = (uint8_t)(fcs >> 8);
    out[n++] = 0x7e; out[n++] = 0x7e;
    return n;
}

size_t k56flex_v8bis_stuff(const uint8_t *octets, size_t len, uint8_t *bits)
{
    size_t n = 0, i;
    unsigned ones = 0, k;
    for (i = 0; i < len; ++i)
        for (k = 0; k < 8; ++k) {
            unsigned b = (octets[i] >> k) & 1;
            if (octets[i] != 0x7e && ones >= 5) { bits[n++] = 0; ones = 0; }
            bits[n++] = (uint8_t)b;
            ones = b ? ones + 1 : 0;
        }
    return n;
}

/* --------------------------------------------------------- parameters ---- */

size_t k56flex_param_raw(const k56flex_param_t *p, uint16_t raw[9])
{
    unsigned first = (p->mode + (p->suppress_extra ? 0 : p->extra << 2) + (p->rate << 6)
                      + (p->v << 13) + (p->u << 14) + (p->final << 15)) & 0xffff;
    unsigned second = ((p->special ? 0xfff : p->control) + (p->bit << 15)) & 0xffff;
    memset(raw, 0, 9 * sizeof(raw[0]));
    raw[0] = (uint16_t)first;
    raw[1] = (uint16_t)second;
    if (p->mode) return 9;
    raw[2] = (uint16_t)second;
    return 3;
}

uint16_t k56flex_param_crc(const uint16_t *words, size_t n)
{
    uint16_t crc = 0xffff;
    size_t i;
    unsigned b;
    for (i = 0; i < n; ++i)
        for (b = 0; b < 16; ++b) crc = (uint16_t)((crc >> 1) ^ (((crc ^ (words[i] >> b)) & 1) ? 0x8408 : 0));
    return crc;
}

size_t k56flex_param_source(const k56flex_param_t *p, uint16_t out[12])
{
    uint16_t raw[10];
    uint8_t bits[16 * 12 + 16];
    size_t nraw = k56flex_param_raw(p, raw), nwords = p->mode ? 12 : 6, i, b;
    memset(bits, 0, sizeof(bits));
    raw[nraw] = k56flex_param_crc(raw, nraw);
    bits[0] = 1;                                   /* marker; bit 1 is the zero separator */
    for (i = 0; i <= nraw; ++i)
        for (b = 0; b < 16; ++b) bits[2 + 17 * i + b] = (raw[i] >> b) & 1;
    out[0] = 0xffff;
    for (i = 1; i < nwords; ++i) {
        out[i] = 0;
        for (b = 0; b < 16; ++b) out[i] |= (uint16_t)(bits[16 * (i - 1) + b] << b);
    }
    return nwords;
}

int k56flex_param_parse(unsigned mode, const uint16_t *source, uint16_t raw[9])
{
    size_t nwords = mode ? 12 : 6, nraw = mode ? 9 : 3, i, b;
    uint16_t all[10];
    uint8_t bits[16 * 12];
    if (source[0] != 0xffff) return -1;
    for (i = 1; i < nwords; ++i)
        for (b = 0; b < 16; ++b) bits[16 * (i - 1) + b] = (source[i] >> b) & 1;
    if (bits[0] != 1) return -1;
    for (i = 0; i <= nraw; ++i) {
        if (bits[1 + 17 * i]) return -1;
        all[i] = 0;
        for (b = 0; b < 16; ++b) all[i] |= (uint16_t)(bits[2 + 17 * i + b] << b);
    }
    if (k56flex_param_crc(all, nraw + 1) != 0) return -1;
    memcpy(raw, all, nraw * sizeof(raw[0]));
    return (int)nraw;
}
