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
