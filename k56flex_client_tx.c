/* See k56flex_client_tx.h.  The conventions below are the receiver's
 * (k56flex_upstream_pcm.c, k56flex_upstream.c), so a symbol produced here
 * decodes there by construction; the tests check it through real audio. */
#include "k56flex_client_tx.h"

#include <math.h>
#include <string.h>

#define R2 0.7071067811865475f

/* Four-point label -> point.  TRN index d and the differential label share
 * this table: labels (++,-+,+-,--) are (0,3,1,2) (resident PM 0294). */
static const float pt_re[4] = { R2,  R2, -R2, -R2};
static const float pt_im[4] = { R2, -R2, -R2,  R2};

/* V.34 Figure 5 quarter-superconstellation points 0..3; the sixteen-point
   set has mean energy 10, so 1/sqrt(10) makes it unit RMS. */
static const float q16_re[4] = {1, -3,  1, -3};
static const float q16_im[4] = {1,  1, -3, -3};
#define RS10 0.31622776601683794f

/* 10.1.3.8, 10.1.3.9: 2*Q2+Q1 selects the quarter point, which is rotated
   clockwise by z*90 degrees (z = 0 leaves it where label 0's point is). */
static void point16(unsigned q, unsigned z, float *re, float *im)
{
    float x = q16_re[q & 3]*RS10, y = q16_im[q & 3]*RS10, t;

    for (z &= 3; z; z--) {
        t = x;
        x = y;
        y = -t;
    }
    *re = x;
    *im = y;
}

void k56flex_client_tx_defaults(k56flex_client_tx_cfg_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->s_symbols = 128;
    cfg->sbar_symbols = 16;
    cfg->pp_symbols = 288;
    cfg->trn_symbols = 3910;
    cfg->report_records = 21;
    cfg->ext = 0x00;
    cfg->report = 0x89b0;
    cfg->gap_symbols = 9614;
    cfg->mp_repeats = 2;
    cfg->mp_ack_repeats = 1;
    cfg->mp_up_code = 4;
    cfg->mp_down_code = 11;
    cfg->mp_mask = 0x7fff;
    cfg->mp_asymmetric = 1;
}

int k56flex_client_tx_init(k56flex_client_tx_t *t, const k56flex_client_tx_cfg_t *cfg)
{
    if (!t || !cfg || cfg->mp_up_code > 15 || cfg->mp_down_code > 15 || cfg->mp_mask > 0x7fff)
        return -1;
    memset(t, 0, sizeof(*t));
    t->cfg = *cfg;
    t->stage = K56CT_IDLE_SILENT;
    return 0;
}

void k56flex_client_tx_start(k56flex_client_tx_t *t)
{
    if (t->stage != K56CT_IDLE_SILENT)
        return;
    t->stage = K56CT_S;
    t->n = 0;
}

void k56flex_client_tx_release_trn2(k56flex_client_tx_t *t)
{
    t->trn2_released = 1;
}

const char *k56flex_client_tx_stage_name(k56flex_client_tx_stage_t st)
{
    static const char *n[] = {"silent", "S", "S-bar", "PP", "TRN", "report", "J'",
                              "TRN2", "MP", "MP'", "tail", "hold"};
    return (unsigned) st < sizeof(n)/sizeof(n[0]) ? n[st] : "?";
}

void k56flex_client_tx_mp_bits(const k56flex_client_tx_cfg_t *cfg, int ack, uint8_t b[88])
{
    uint16_t crc = 0xffff;
    int i, j;

    memset(b, 0, 88);
    for (i = 0; i < 17; i++)
        b[i] = 1;                                   /* frame sync */
    /* 17, 34, 51, 68: start bits 0.  18: type 0. */
    for (i = 0; i < 4; i++) {
        b[20 + i] = (cfg->mp_up_code >> i) & 1;
        b[24 + i] = (cfg->mp_down_code >> i) & 1;
    }
    b[33] = ack ? 1 : 0;
    for (i = 0; i < 15; i++)
        b[35 + i] = (cfg->mp_mask >> i) & 1;
    b[50] = cfg->mp_asymmetric ? 1 : 0;
    /* 10.1.2.3.2: CRC over the 16-bit groups between the start bits. */
    for (i = 17; i < 68; i += 17)
        for (j = 1; j <= 16; j++)
            crc = (crc >> 1) ^ (((crc ^ b[i + j]) & 1) ? 0x8408 : 0);
    for (i = 0; i < 16; i++)
        b[69 + i] = (crc >> i) & 1;
    /* 85:87 fill 0 */
}

static unsigned trn_bit(k56flex_client_tx_t *t)
{
    unsigned y = 1 ^ ((t->trn_scr >> 17) & 1) ^ ((t->trn_scr >> 22) & 1);
    t->trn_scr = (t->trn_scr << 1) | y;
    return y;
}

/* One data bit through the scrambler (y = x ^ y[n-18] ^ y[n-23]). */
static unsigned scramble(k56flex_client_tx_t *t, unsigned x)
{
    unsigned y = (x & 1) ^ ((t->scr >> 17) & 1) ^ ((t->scr >> 22) & 1);
    t->scr = (t->scr << 1) | y;
    return y;
}

/* Two data bits, LSB first in time, as one differentially encoded symbol. */
static void diff_symbol(k56flex_client_tx_t *t, unsigned b0, unsigned b1, float *re, float *im)
{
    unsigned d = scramble(t, b0) | (scramble(t, b1) << 1);
    unsigned raw = (t->prev + d) & 3;
    /* raw label -> point: 0 ++, 1 +-, 2 --, 3 -+ (labels {0,3,1,2} inverted) */
    t->prev = raw;
    *re = pt_re[raw];
    *im = pt_im[raw];
}

/* Four data bits, LSB first in time, as one sixteen-point symbol (V.34
   10.1.3.9): I1 I2 differentially encoded as in the four-point case, Q1 Q2
   pick the quarter point. */
static void diff_symbol16(k56flex_client_tx_t *t, const unsigned b[4], float *re, float *im)
{
    unsigned i = scramble(t, b[0]) | (scramble(t, b[1]) << 1);
    unsigned q = scramble(t, b[2]) | (scramble(t, b[3]) << 1);

    t->prev = (t->prev + i) & 3;
    point16(q, t->prev, re, im);
}

static void load_record(k56flex_client_tx_t *t)
{
    uint32_t rec = (uint32_t) t->cfg.ext | ((uint32_t) t->cfg.report << 8);
    for (unsigned i = 0; i < 24; i++)
        t->bits[i] = (rec >> i) & 1;
    t->nbits = 24;
    t->bitpos = 0;
}

static void enter(k56flex_client_tx_t *t, k56flex_client_tx_stage_t st)
{
    t->stage = st;
    t->n = 0;
    t->frames = 0;
    switch (st) {
    case K56CT_REPORT:
        load_record(t);
        break;
    case K56CT_JPRIME:
        /* J' takes the report word's place in one more record: the
           extension octet, then 899F (native HSF, decoded from
           reference-data/k56flex-hsf/upstream-mp-ulaw.g711).  A bare 899F
           after the last record misses the receiver's record phase. */
        for (unsigned i = 0; i < 8; i++)
            t->bits[i] = (t->cfg.ext >> i) & 1;
        for (unsigned i = 0; i < 16; i++)
            t->bits[8 + i] = (0x899f >> i) & 1;
        t->nbits = 24;
        t->bitpos = 0;
        break;
    case K56CT_GAP:
        /* A second TRN from a fresh scrambler, on which the server trains
           before it looks for MP (native HSF: 9614 symbols straight after
           J', decoded from reference-data/k56flex-hsf/upstream-mp-ulaw.g711).
           The receiver's MP framer starts with zero differential and
           scrambler history on the symbol after J', and runs through it. */
        t->trn_scr = 0;
        t->prev = 0;
        t->scr = 0;
        break;
    case K56CT_MP:
    case K56CT_MP_ACK:
        k56flex_client_tx_mp_bits(&t->cfg, st == K56CT_MP_ACK, t->bits);
        t->nbits = 88;
        t->bitpos = 0;
        break;
    default:
        break;
    }
}

/* Next two bits of the current frame; advances frames.  Returns 0 when the
   stage's frames are exhausted before these bits. */
static int frame_bits(k56flex_client_tx_t *t, unsigned *b0, unsigned *b1, unsigned frames)
{
    if (t->bitpos >= t->nbits) {
        if (++t->frames >= frames)
            return 0;
        t->bitpos = 0;
    }
    *b0 = t->bits[t->bitpos++];
    *b1 = t->bits[t->bitpos++];
    return 1;
}

k56flex_client_tx_stage_t k56flex_client_tx_symbol(k56flex_client_tx_t *t, float *re, float *im)
{
    k56flex_client_tx_stage_t st;
    unsigned b0, b1, b[4];

    *re = *im = 0.0f;
    for (;;) {
        st = t->stage;
        switch (st) {
        case K56CT_IDLE_SILENT:
            return st;
        case K56CT_S:
        case K56CT_SBAR:
            if (t->n >= (st == K56CT_S ? t->cfg.s_symbols : t->cfg.sbar_symbols)) {
                enter(t, st == K56CT_S ? K56CT_SBAR : K56CT_PP);
                continue;
            }
            /* The alternation runs on through S-bar: count from the start of S. */
            {
                unsigned k = (st == K56CT_S) ? t->n : t->cfg.s_symbols + t->n;
                float sr = (k & 1) ? -R2 : R2, si = R2;
                if (st == K56CT_SBAR) { sr = -sr; si = -si; }
                *re = sr;
                *im = si;
            }
            break;
        case K56CT_PP:
            if (t->n >= t->cfg.pp_symbols) {
                enter(t, K56CT_TRN);
                continue;
            }
            {
                unsigned k = (t->n % 48)/4, i = t->n % 4;
                double a = 3.14159265358979323846*(k*i + (k%3 == 1 ? 4 : 0))/6;
                *re = (float) cos(a);
                *im = (float) sin(a);
            }
            break;
        case K56CT_TRN:
        case K56CT_GAP:
            if (t->n >= (st == K56CT_TRN ? t->cfg.trn_symbols : t->cfg.gap_symbols)) {
                if (st == K56CT_TRN)
                    enter(t, K56CT_REPORT);
                else
                    enter(t, t->cfg.mp_repeats ? K56CT_MP : K56CT_TAIL);
                continue;
            }
            {
                unsigned d = trn_bit(t) | (trn_bit(t) << 1);
                /* The receiver's report (or MP) decoder already runs through
                   this TRN: carry its differential and descrambler histories. */
                unsigned dd = (d - t->prev) & 3;
                t->scr = (t->scr << 1) | (dd & 1);
                t->scr = (t->scr << 1) | (dd >> 1);
                t->prev = d;
                *re = pt_re[d];
                *im = pt_im[d];
                if (st == K56CT_GAP  &&  t->cfg.trn16) {
                    /* 10.1.3.8: Q1 Q2 follow I1 I2 in time. */
                    unsigned q = trn_bit(t) | (trn_bit(t) << 1);
                    t->scr = (t->scr << 1) | (q & 1);
                    t->scr = (t->scr << 1) | (q >> 1);
                    point16(q, d, re, im);
                }
            }
            break;
        case K56CT_REPORT:
            if (!frame_bits(t, &b0, &b1, t->cfg.report_records)) {
                enter(t, K56CT_JPRIME);
                continue;
            }
            diff_symbol(t, b0, b1, re, im);
            break;
        case K56CT_JPRIME:
            if (!frame_bits(t, &b0, &b1, 1)) {
                enter(t, (t->cfg.trn2_hold && !t->trn2_released) ? K56CT_HOLD : K56CT_GAP);
                continue;
            }
            diff_symbol(t, b0, b1, re, im);
            break;
        case K56CT_HOLD:
            if (t->trn2_released) {
                enter(t, K56CT_GAP);
                continue;
            }
            diff_symbol(t, 1, 1, re, im);
            break;
        case K56CT_MP:
        case K56CT_MP_ACK:
            if (!frame_bits(t, &b0, &b1, st == K56CT_MP ? t->cfg.mp_repeats : t->cfg.mp_ack_repeats)) {
                enter(t, (st == K56CT_MP && t->cfg.mp_ack_repeats) ? K56CT_MP_ACK : K56CT_TAIL);
                continue;
            }
            if (t->cfg.trn16) {
                /* 88 bits is 22 symbols: the second pair never crosses a frame. */
                b[0] = b0;
                b[1] = b1;
                frame_bits(t, &b[2], &b[3], ~0u);
                diff_symbol16(t, b, re, im);
                break;
            }
            diff_symbol(t, b0, b1, re, im);
            break;
        case K56CT_TAIL:
            /* E (10.1.3.2) is ones on J's constellation; so is the idle after it. */
            if (t->cfg.trn16) {
                b[0] = b[1] = b[2] = b[3] = 1;
                diff_symbol16(t, b, re, im);
                break;
            }
            diff_symbol(t, 1, 1, re, im);
            break;
        }
        t->n++;
        t->symbols++;
        return st;
    }
}

void k56flex_jd_rx_init(k56flex_jd_rx_t *j)
{
    memset(j, 0, sizeof(*j));
}

/* w, LSB first in time, is a rotation of `want` or of its complement. */
static int is_rotation(uint16_t w, uint16_t want)
{
    for (unsigned r = 0; r < 16; r++) {
        uint16_t x = (uint16_t) ((w >> r) | (w << (16 - r)));
        if (x == want  ||  (uint16_t) ~x == want)
            return 1;
    }
    return 0;
}

uint16_t k56flex_jd_rx_sample(k56flex_jd_rx_t *j, int16_t v)
{
    unsigned y, x;
    uint16_t w = 0;

    if (j->word)
        return j->word;
    if ((j->odd ^= 1) == 0)
        return 0;
    y = v < 0;
    x = y ^ ((j->y >> 4) & 1) ^ ((j->y >> 22) & 1);
    j->y = (j->y << 1) | y;
    j->d = (j->d << 1) | x;
    j->run = (x == ((j->d >> 16) & 1)) ? j->run + 1 : 0;
    if (j->run < 32*16)
        return 0;
    for (unsigned i = 0; i < 16; i++)
        w |= (uint16_t) (((j->d >> (15 - i)) & 1) << i);
    if (is_rotation(w, 0x8990))
        j->word = 0x8990;
    else if (is_rotation(w, 0x89b0))
        j->word = 0x89b0;
    return j->word;
}
