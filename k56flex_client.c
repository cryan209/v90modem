#include "k56flex_client.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define MAX_BITS 262144u

k56flex_client_t *k56flex_client_new(const k56flex_client_cfg_t *cfg)
{
    k56flex_client_t *c = calloc(1, sizeof(*c));
    k56flex_train_cfg_t tc;
    if (!c) return NULL;
    c->cfg = *cfg;
    c->y = malloc(MAX_BITS);
    c->x = malloc(MAX_BITS);
    memset(&tc, 0, sizeof(tc));
    tc.law = cfg->law;
    tc.rate_bps = 56000;                      /* placeholder: the real rate is in the record */
    tc.training_word = 0xffff;
    c->report_field = k56flex_report_field(cfg->report_ext, cfg->report_word);
    if (!c->y || !c->x || k56flex_train_init(&c->shadow, &tc) < 0) {
        k56flex_client_free(c);
        return NULL;
    }
    return c;
}

void k56flex_client_free(k56flex_client_t *c)
{
    if (!c) return;
    k56flex_rxfe_free(c->fe);
    free(c->y);
    free(c->x);
    free(c);
}

k56flex_train_phase_t k56flex_client_phase(const k56flex_client_t *c) { return k56flex_train_phase(&c->shadow); }

static void send_status(k56flex_client_t *c, unsigned bits)
{
    if (!c->detect) k56flex_train_status(&c->shadow, bits);
    if (c->cfg.ctl.status) c->cfg.ctl.status(c->cfg.ctl.user, bits);
}

static void send_report(k56flex_client_t *c)
{
    k56flex_train_set_report(&c->shadow, c->report_field);
    if (c->cfg.ctl.report) c->cfg.ctl.report(c->cfg.ctl.user, c->report_field);
    c->report_sent = 1;
}

/* Find a framed record in x[] at or after `from`: the marker is the one before a zero
 * that follows at least 17 ones (the FFFF prefix plus the marker; separators keep every
 * other run of ones to 16 or fewer). */
static int find_record(const k56flex_client_t *c, unsigned from, unsigned *marker)
{
    unsigned z, i;
    for (z = from < 17 ? 17 : from; z < c->nbits; ++z) {
        if (c->x[z]) continue;
        for (i = 1; i <= 17 && c->x[z - i]; ++i) {}
        if (i > 17) { *marker = z - 1; return 1; }
    }
    return 0;
}

static int parse_record(const k56flex_client_t *c, unsigned marker, uint16_t raw[9])
{
    uint16_t src[12];
    unsigned i, b;
    if (marker + 11 * 16 > c->nbits) return -1;
    src[0] = 0xffff;
    for (i = 1; i < 12; ++i) {
        src[i] = 0;
        for (b = 0; b < 16; ++b) src[i] |= (uint16_t)(c->x[marker + 16 * (i - 1) + b] << b);
    }
    return k56flex_param_parse(1, src, raw);
}

static void try_decode_param(k56flex_client_t *c, k56flex_train_phase_t ph)
{
    unsigned marker, from = c->param_ok ? c->rec_end : c->param_from;
    uint16_t raw[9];
    (void)ph;
    while (find_record(c, from, &marker)) {
        if (marker + 11 * 16 > c->nbits) return;               /* wait for more bits */
        if (parse_record(c, marker, raw) < 0) { from = marker + 2; continue; }
        {
            unsigned w = raw[0], final = (w >> 15) & 1;
            if (!c->param_ok) {
                c->param.mode = w & 3;
                c->param.extra = (w >> 2) & 15;
                c->param.rate = (w >> 6) & 0x7f;
                c->param.v = (w >> 13) & 1;
                c->param.u = (w >> 14) & 1;
                c->param.final = final;
                c->param.control = raw[1] & 0x7fff;
                c->param.bit = raw[1] >> 15;
                c->rate_bps = ((int)c->param.rate + 18 - 3) * 2000;
                c->param_ok = 1;
                c->shadow.cfg.rate_bps = c->rate_bps;
            }
            c->rec_end = marker + 2 + 17 * 10;
            if (!c->resp_sent) { c->resp_sent = 1; send_status(c, 1u << 13); }
            if (final && !c->done_sent) { c->done_sent = 1; send_status(c, K56FLEX_STATUS_PARAM_DONE); }
            from = c->rec_end;
        }
    }
}

static void level_block(const k56flex_client_t *c, const uint8_t *o, unsigned n, int16_t *lv)
{
    unsigned i;
    for (i = 0; i < n; ++i) lv[i] = (int16_t)k56flex_level_from_g711(c->cfg.law, o[i]);
}

static void training_block(k56flex_client_t *c, const uint8_t *o)
{
    k56flex_train_t *sh = &c->shadow;
    k56flex_probe_t pre = sh->probe;
    k56flex_train_phase_t ph = k56flex_train_phase(sh);
    int16_t lv[6], exp[6], tmp[6];
    uint32_t src = 0;
    unsigned matches, i;
    const k56flex_train_phase_t unknown_from = c->detect ? K56T_GATE_C : K56T_PARAM_1;
    int known_ref = ph < unknown_from;
    level_block(c, o, 6, lv);
    k56flex_train_samples(sh, exp, 6);           /* advances the mirror phase machine */
    if (ph < unknown_from) {
        int same = 1;
        for (i = 0; i < 6; ++i) {
            int e = k56flex_g711_from_level(c->cfg.law, exp[i]);
            if (e != o[i]) same = 0;
        }
        ++c->blocks_checked;
        if (!same && !c->blocks_mismatched++) c->first_bad = (int)ph * 100000 + (int)sh->pairs_total;
    }
    if (ph >= K56T_PT_A) {
        if (!c->cp_valid || ph < unknown_from) { c->cp = pre; c->cp_valid = 1; }
        matches = k56flex_probe_invert(&c->cp, lv, &src);
        if (matches == 0) { c->failed = 1; return; }
        if (matches > 1) ++c->ambiguous_blocks;
        k56flex_probe_block_src(&c->cp, src, tmp);
        for (i = 0; i < c->cp.cfg[3]; ++i) {
            unsigned y = (src >> i) & 1, k = c->nbits;
            if (k >= MAX_BITS) { c->failed = 1; return; }
            c->y[k] = (uint8_t)y;
            c->x[k] = (uint8_t)(y ^ (k >= 5 ? c->y[k - 5] : 0) ^ (k >= 23 ? c->y[k - 23] : 0));
            c->nbits = k + 1;
        }
    }
    if (ph == unknown_from && !c->param_from_set) { c->param_from = c->nbits - c->cp.cfg[3]; c->param_from_set = 1; }
    if (ph >= unknown_from && ph <= K56T_PARAM_2) try_decode_param(c, ph);
    /* upstream signalling, keyed to the phase the mirror has just entered */
    ph = k56flex_train_phase(sh);
    if (ph == K56T_GATE_A && !c->gate_a_sent) {
        c->gate_a_sent = 1;
        send_report(c);
        send_status(c, K56FLEX_STATUS_PROBE_PEER);
    }
    if (ph == K56T_GATE_B && !c->gate_b_sent && sh->gate_pairs >= c->cfg.gate_b_pairs) {
        c->gate_b_sent = 1;
        send_status(c, K56FLEX_STATUS_SILENCE_END);
    }
    if (ph == K56T_GATE_C && !c->gate_c_sent) {
        c->gate_c_sent = 1;
        send_status(c, 1u << 12);
    }
    if (c->fe) k56flex_rxfe_block(c->fe, c->qfirst, known_ref ? exp : lv, 6, known_ref ? 1 : -1);
    c->sym_idx += 6;
}

static int ensure_rx(k56flex_client_t *c)
{
    k56flex_pcm_config_t pc;
    unsigned i, hist = 0;
    if (c->rx_ready) return 1;
    if (!c->param_ok) return 0;
    pc.law = c->cfg.law;
    pc.rate_bps = c->rate_bps;
    pc.report_field = c->report_field;
    if (k56flex_pcm_rx_init(&c->rx, &pc) < 0) return 0;
    for (i = 0; i < 23 && i < c->nbits; ++i) hist |= (unsigned)c->y[c->nbits - 1 - i] << i;
    c->rx.descrambler = hist;
    c->rx_ready = 1;
    return 1;
}

static void frame_block(k56flex_client_t *c, const uint8_t *o)
{
    k56flex_train_t *sh = &c->shadow;
    k56flex_train_phase_t ph = k56flex_train_phase(sh);
    int16_t exp[8];
    uint8_t bits[K56FLEX_MAX_FRAME_BITS];
    int nb;
    if (!ensure_rx(c)) { c->failed = 1; return; }
    k56flex_train_samples(sh, exp, 8);           /* advance the mirror */
    nb = k56flex_pcm_rx_frame(&c->rx, o, bits);
    if (c->fe) { int16_t dl[8]; level_block(c, o, 8, dl); k56flex_rxfe_block(c->fe, c->qfirst, dl, 8, 0); }
    if (nb < 0) { c->failed = 1; return; }
    if (ph == K56T_PRIME) {
        int i;
        ++c->prime_frames;
        for (i = 0; i < nb; ++i) if (!bits[i]) ++c->prime_ones_bad;
    } else {
        ++c->data_frames;
        c->data_bits_out += (unsigned)nb;
        if (c->cfg.ctl.data) c->cfg.ctl.data(c->cfg.ctl.user, bits, (unsigned)nb);
    }
}

void k56flex_client_rx(k56flex_client_t *c, const uint8_t *octets, size_t n)
{
    size_t i;
    for (i = 0; i < n && !c->failed; ++i) {
        k56flex_train_phase_t ph = k56flex_train_phase(&c->shadow);
        unsigned want = (ph == K56T_PRIME || ph == K56T_DATA) ? 8 : 6;
        c->pend[c->pend_n++] = octets[i];
        if (c->pend_n == want) {
            c->pend_n = 0;
            if (want == 6) training_block(c, c->pend); else frame_block(c, c->pend);
        }
    }
}

/* ---- linear-audio path ------------------------------------------------------------ */

#define SILENT_LEVEL 400.0f           /* equalized |y| below this is silence, above it a signal */

static void slice(const k56flex_client_t *c, const float *y, unsigned n, const int16_t *lv, unsigned nl, uint8_t *o)
{
    unsigned i, j;
    for (i = 0; i < n; ++i) {
        float bd = 1e30f;
        int best = 0;
        for (j = 0; j < nl; ++j) {
            float d = fabsf(y[i] - (float)lv[j]);
            if (d < bd) { bd = d; best = lv[j]; }
        }
        {
            int oct = k56flex_g711_from_level(c->cfg.law, best);
            o[i] = (uint8_t)(oct < 0 ? 0xff : oct);
        }
    }
}

static unsigned stage_levels(const k56flex_client_t *c, int16_t *lv, unsigned max)
{
    const k56flex_probe_t *p = &c->shadow.probe;
    unsigned i, n = p->cfg[9] + 1u;
    for (i = 0; i < n && i < max; ++i) lv[i] = p->levels[i];
    return i;
}

static int is_silent(const float *y, unsigned n)
{
    unsigned i;
    for (i = 0; i < n; ++i) if (fabsf(y[i]) > SILENT_LEVEL) return 0;
    return 1;
}

static void training_symbols(k56flex_client_t *c, const float *y)
{
    int16_t lv[256];
    uint8_t o[6];
    unsigned nl;
    k56flex_train_phase_t ph = k56flex_train_phase(&c->shadow);
    /* The server leaves its gates at pair boundaries; the signal shows when. */
    if (ph == K56T_GATE_A && is_silent(y, 6)) k56flex_train_force_phase(&c->shadow, K56T_GATE_B);
    else if (ph == K56T_GATE_B && !is_silent(y, 6)) k56flex_train_force_phase(&c->shadow, K56T_ID2_A);
    nl = stage_levels(c, lv, 256);
    slice(c, y, 6, lv, nl, o);
    training_block(c, o);
}

static void frame_symbols(k56flex_client_t *c, const float *y)
{
    int16_t lv[256];
    uint8_t o[8];
    unsigned nl;
    if (!ensure_rx(c)) { c->failed = 1; return; }
    nl = k56flex_pcm_levels(&c->rx.m, lv, 256);
    slice(c, y, 8, lv, nl, o);
    frame_block(c, o);
}

/* After "parameters accepted" the server finishes its pass, sends the tail and starts the
 * priming frames.  A pair whose symbols all sit on the training levels is still training;
 * the first one that does not is the start of priming. */
static int pair_is_training(const k56flex_client_t *c, const float *y)
{
    int16_t lv[256];
    unsigned nl = stage_levels(c, lv, 256), i, j;
    float gap = 1e30f;
    for (i = 0; i < nl; ++i)
        for (j = i + 1; j < nl; ++j) {
            float d = fabsf((float)lv[i] - (float)lv[j]);
            if (d > 0 && d < gap) gap = d;
        }
    for (i = 0; i < 12; ++i) {
        float bd = 1e30f;
        for (j = 0; j < nl; ++j) {
            float d = fabsf(y[i] - (float)lv[j]);
            if (d < bd) bd = d;
        }
        if (bd > 0.3f * gap) return 0;
    }
    return 1;
}

static void fe_sink(void *user, float y)
{
    k56flex_client_t *c = user;
    if (!c->fe_started) {
        /* acquired at the start of P1: run the mirror through silence and identification */
        int16_t tmp[6];
        c->fe_started = 1;
        while (k56flex_train_phase(&c->shadow) < K56T_P1) k56flex_train_samples(&c->shadow, tmp, 6);
        c->qn = 0;
    }
    if (c->failed) return;
    if (c->qn == 0) c->qfirst = k56flex_rxfe_symbols(c->fe) - 1;
    c->qy[c->qn++] = y;
    if (c->in_frames) {
        if (c->qn >= 8) {
            frame_symbols(c, c->qy);
            c->qn -= 8;
            c->qfirst += 8;
            memmove(c->qy, c->qy + 8, c->qn * sizeof(c->qy[0]));
        }
        return;
    }
    if (!c->seek) {
        if (c->qn < 6) return;
        if (c->done_sent && c->sym_idx % 12 == 0) { c->seek = 1; }   /* wait for a pair boundary */
        else { training_symbols(c, c->qy); c->qn = 0; return; }
    }
    if (c->qn < 12) return;
    if (pair_is_training(c, c->qy)) {
        training_symbols(c, c->qy);
        c->qfirst += 6;
        training_symbols(c, c->qy + 6);
        c->qn = 0;
    } else {
        k56flex_train_force_phase(&c->shadow, K56T_PRIME);
        c->in_frames = 1;
        frame_symbols(c, c->qy);
        c->qn -= 8;
        c->qfirst += 8;
        memmove(c->qy, c->qy + 8, c->qn * sizeof(c->qy[0]));
    }
}

void k56flex_client_rx_linear(k56flex_client_t *c, const int16_t *samples, size_t n)
{
    size_t i;
    if (!c->fe) {
        c->detect = 1;
        c->fe = k56flex_rxfe_new(c->cfg.law, fe_sink, c);
        if (!c->fe) { c->failed = 1; return; }
    }
    for (i = 0; i < n && !c->failed; ++i) k56flex_rxfe_push(c->fe, samples[i]);
}
