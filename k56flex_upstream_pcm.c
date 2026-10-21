/* See k56flex_upstream_pcm.h. Fixed-rate preamble/report acquisition in
 * batch or incremental chunks; it does not replace the V.34 data receiver. */
#include "k56flex_upstream_pcm.h"
#include "p3_demod.h"

#include <math.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>

/* Nine complex T/2 observations, solved as two real-output FIRs. Allowing
 * both I and Q as independent columns also handles conjugate orientation. */
enum { TAPS = 9, COLS = 2 * TAPS, TRN = 512 };

typedef struct { double re, im; } point_t;

static void reference(point_t ref[TRN], unsigned tap)
{
    static const double re[4] = {1, 1, -1, -1};
    static const double im[4] = {1, -1, -1, 1};
    uint32_t history = 0;
    for (int n = 0; n < TRN; ++n) {
        unsigned dibit = 0;
        for (unsigned i = 0; i < 2; ++i) {
            unsigned y = 1 ^ ((history >> (tap - 1)) & 1)
                           ^ ((history >> 22) & 1);
            history = (history << 1) | y;
            dibit |= y << i;
        }
        ref[n].re = re[dibit] * 0.7071067811865475;
        ref[n].im = im[dibit] * 0.7071067811865475;
    }
}

static void row(const p3_result_t *r, int at, double x[COLS])
{
    for (int j = 0; j < TAPS; ++j) {
        x[2*j] = r->symbols[at + j - TAPS/2].re;
        x[2*j+1] = r->symbols[at + j - TAPS/2].im;
    }
}

static point_t equalize(const p3_result_t *r, int at, const double w[COLS][2])
{
    double x[COLS];
    point_t y = {0, 0};
    row(r, at, x);
    for (int j = 0; j < COLS; ++j) {
        y.re += x[j] * w[j][0];
        y.im += x[j] * w[j][1];
    }
    return y;
}

static int train(const p3_result_t *r, int start, const point_t ref[TRN],
                 double w[COLS][2])
{
    double a[COLS][COLS+2] = {{0}}, trace = 0;
    /* Do not train on the PP/TRN edge, nor on the held-out tail. */
    for (int n = 16; n < 272; ++n) {
        double x[COLS];
        row(r, start + 2*n, x);
        for (int j = 0; j < COLS; ++j) {
            for (int k = 0; k < COLS; ++k) a[j][k] += x[j]*x[k];
            a[j][COLS] += x[j]*ref[n].re;
            a[j][COLS+1] += x[j]*ref[n].im;
        }
    }
    for (int j = 0; j < COLS; ++j) trace += a[j][j];
    if (trace < 1e-12) return 0;
    /* Local regularization, not an ITU constant. */
    for (int j = 0; j < COLS; ++j) a[j][j] += trace * 1e-6 / COLS;
    for (int j = 0; j < COLS; ++j) {
        int pivot = j;
        for (int k = j+1; k < COLS; ++k)
            if (fabs(a[k][j]) > fabs(a[pivot][j])) pivot = k;
        if (fabs(a[pivot][j]) < trace * 1e-12) return 0;
        for (int k = j; k < COLS+2; ++k) {
            double tmp = a[j][k]; a[j][k] = a[pivot][k]; a[pivot][k] = tmp;
        }
        double scale = a[j][j];
        for (int k = j; k < COLS+2; ++k) a[j][k] /= scale;
        for (int i = 0; i < COLS; ++i) if (i != j) {
            scale = a[i][j];
            for (int k = j; k < COLS+2; ++k) a[i][k] -= scale*a[j][k];
        }
    }
    for (int j = 0; j < COLS; ++j) {
        w[j][0] = a[j][COLS]; w[j][1] = a[j][COLS+1];
    }
    return 1;
}

static double error(point_t a, point_t b)
{
    return (a.re-b.re)*(a.re-b.re) + (a.im-b.im)*(a.im-b.im);
}

static int validate_training(const p3_result_t *r, int start,
                             const point_t ref[TRN], const double w[COLS][2],
                             k56flex_upstream_pcm_result_t *out)
{
    double e = 0;
    for (int n = 288; n < 480; ++n)
        e += error(equalize(r, start+2*n, w), ref[n]);
    out->trn_rms = sqrt(e/192);
    if (out->trn_rms > 0.25) return 0;
    e = 0;
    for (int n = 16; n < 272; ++n) {
        /* V.34 10.1.3.6, 48-symbol PP, repeated six times. */
        int k = (n%48)/4, i = n%4;
        double angle = 3.14159265358979323846 * (k*i + (k%3 == 1 ? 4 : 0))/6;
        point_t pp = {cos(angle), sin(angle)};
        e += error(equalize(r, start - 2*288 + 2*n, w), pp);
    }
    out->pp_rms = sqrt(e/256);
    e = 0;
    for (int n = -64; n < 16; ++n) {
        /* S alternates (++,-+); S-bar alternates (--,+-). */
        point_t s = {(n&1) ? -0.7071067811865475 : 0.7071067811865475,
                     0.7071067811865475};
        if (n >= 0) { s.re = -s.re; s.im = -s.im; }
        e += error(equalize(r, start - 2*(288+16) + 2*n, w), s);
    }
    out->s_rms = sqrt(e/80);
    if (out->pp_rms > 0.3 || out->s_rms > 0.3) return 0;
    return 1;
}

int k56flex_upstream_pcm_decode(const int16_t *pcm, int count, unsigned tap,
                               k56flex_upstream_pcm_result_t *out)
{
    p3_demod_t d;
    p3_result_t *r;
    point_t ref[TRN];
    double best = 0, w[COLS][2];
    int start = -1, events = 0;
    if (!out) return -1;
    memset(out, 0, sizeof(*out));
    out->trn_sample = out->report_sample = out->end_sample = -1;
    out->sbar_sample = -1;
    out->mp_sample = out->mp_ack_sample = -1;
    if (!pcm || count <= 0 || count > 80000 || (tap != 5 && tap != 18)) return -1;
    r = p3_result_alloc(count + 100, 0);
    if (!r) return -1;
    p3_demod_init(&d, P3_BAUD_3200, P3_CARRIER_LOW, 8000);
    /* Use only p3's vendored matched-filter bank and carrier mixer. The
     * observation stream must not be steered by its blind CMA or AGC.
     * Fixed-phase T/2 observations retain both timing parities for the FIR.
     * Disable the Godard correction explicitly, independent of env knobs. */
    d.use_rrc_frontend = true;
    d.emit_half_baud = true;
    d.bypass_equalizer = true;
    d.cma_freeze_after_sample = 0;
    d.rrc_agc_gain = 0.0001f;
    d.ted_low_coeff[2] = d.ted_high_coeff[2] = d.ted_mixed_coeff = 0;
    p3_demod_process(&d, pcm, count, 0, r);
    reference(ref, tap);
    /* Find zero-initialized TRN without knowing the signal's onset. Search
     * a non-edge 128-symbol window; test both complex orientations. */
    for (int at = 2*(288+16+64); at + 2*TRN + TAPS < r->symbol_count; ++at) {
        double cr = 0, ci = 0, dr = 0, di = 0, power = 0;
        for (int n = 32; n < 160; ++n) {
            const p3_symbol_t *s = &r->symbols[at+2*n];
            cr += s->re*ref[n].re + s->im*ref[n].im;
            ci += s->im*ref[n].re - s->re*ref[n].im;
            dr += s->re*ref[n].re - s->im*ref[n].im;
            di += s->im*ref[n].re + s->re*ref[n].im;
            power += s->re*s->re + s->im*s->im;
        }
        double corr = power > 1e-12 ? sqrt(fmax(cr*cr+ci*ci, dr*dr+di*di)/(128*power)) : 0;
        if (corr > best) { best = corr; start = at; }
    }
    out->correlation = best;
    /* These are conservative experimental confidence floors, not spec
     * thresholds. Held-out training and the preceding preamble must agree
     * before report bits are ever examined. */
    if (start < 0 || best < 0.65 || !train(r, start, ref, w)) goto done;
    if (!validate_training(r, start, ref, w, out)) goto done;
    out->acquired = 1;
    /* p3's timestamps subtract 32T for its full equalizer. Here we use
     * only its matched-filter output: restore that bookkeeping offset. */
    out->trn_sample = r->symbols[start].sample_index + 80;
    k56flex_upstream_t rx;
    k56flex_upstream_init(&rx, tap);
    for (int at = start; at + TAPS/2 < r->symbol_count; at += 2) {
        point_t y = equalize(r, at, w);
        int ev = k56flex_upstream_symbol(&rx, (float)y.re, (float)y.im);
        if (ev < 0) continue;
        if (ev & K56U_REPORT) {
            out->ext = rx.report.ext; out->report = rx.report.report;
            out->report_sample = r->symbols[at].sample_index + 80;
        }
        if (ev & K56U_REPORT_END) out->end_sample = r->symbols[at].sample_index + 80;
        events |= ev;
        if (rx.ended) break;
    }
done:
    p3_result_free(r);
    return events;
}

/* Incremental version of the same acquired-reference decoder. Candidate
 * windows are examined once after their held-out tail arrives; later chunks
 * retain carrier/filter and descrambler state. No media samples are inserted
 * or removed and no caller-supplied stage boundary or report is accepted. */
struct k56flex_upstream_pcm_s {
    p3_demod_t demod;
    p3_result_t *observations;
    point_t ref[TRN];
    double weights[COLS][2];
    k56flex_upstream_t control;
    k56flex_upstream_pcm_result_t result;
    int samples, search_at, next_symbol, s_search_at, sbar_seen, timestamp_delay;
    unsigned mp_previous, mp_history, mp_count, mp_repeats, mp_seen, mp_ack_seen;
    uint8_t mp_bits[188], mp_last[188], mp_accepted[188];
    int mp_last_length;
};

static unsigned mp_field(const uint8_t *bits, int at, int count)
{
    unsigned value = 0;
    for (int i = 0; i < count; i++) value |= (unsigned)bits[at+i] << i;
    return value;
}

/* V.34 10.1.3.9, Tables 20/21 and 10.1.2.3.2: strict sync, start,
 * reflected-8408 CRC and fill. No preamble-only or elapsed-time acceptance.
 * K56 uses bits 24:27 as PCM rate/2000-15 (original HSF 12023dc0),
 * rather than the ordinary V.34 downstream rate multiplier. */
static int mp_bit(k56flex_upstream_pcm_t *s, unsigned bit)
{
    memmove(s->mp_bits, s->mp_bits+1, sizeof(s->mp_bits)-1);
    s->mp_bits[187] = (uint8_t)bit;
    if (s->mp_count < 188) ++s->mp_count;
    for (int type = 0; type < 2; type++) {
        int length = type ? 188 : 88, end = type ? 170 : 68;
        const uint8_t *b = s->mp_bits + 188-length;
        if (s->mp_count < (unsigned)length || b[18] != type) continue;
        if (mp_field(b, 0, 17) != 0x1ffff) continue;
        int valid = 1;
        for (int i = 17; i <= end; i += 17) if (b[i]) valid = 0;
        for (int i = end+17; i < length; i++) if (b[i]) valid = 0;
        uint16_t crc = 0xffff;
        for (int i = 17; i < end; i += 17)
            for (int j = 1; j <= 16; j++)
                crc = (crc >> 1) ^ (((crc ^ b[i+j]) & 1) ? 0x8408 : 0);
        if (!valid || crc != mp_field(b, end+1, 16)) continue;
        /* MP' need not repeat: original HSF transmits one accepted MP'
         * then E/B1. Require a complete CRC-valid acknowledgement of the
         * already accepted payload, not a second acknowledgement frame. */
        if (s->mp_seen && b[33] && !s->mp_ack_seen && type == (int)s->result.mp_type) {
            int same = 1;
            for (int i = 18; i < end; i++)
                if (i != 33 && b[i] != s->mp_accepted[i]) same = 0;
            if (same) { s->mp_ack_seen = 1; return K56U_MP_ACK; }
        }
        if (s->mp_last_length == length && !memcmp(s->mp_last, b, length))
            s->mp_repeats = 2;
        else {
            memcpy(s->mp_last, b, length);
            s->mp_last_length = length;
            s->mp_repeats = 1;
        }
        if (s->mp_repeats < 2) continue;
        if (!s->mp_seen) {
            s->mp_seen = 1;
            memcpy(s->mp_accepted, b, length);
            s->result.mp_type = type;
            s->result.mp_up = mp_field(b, 20, 4);
            s->result.mp_down = mp_field(b, 24, 4);
            s->result.mp_mask = mp_field(b, 35, 15);
            s->result.mp_trellis = mp_field(b, 29, 2);
            s->result.mp_nonlinear = b[31];
            s->result.mp_shaping = b[32];
            return K56U_MP;
        }
    }
    return 0;
}

static int mp_symbol(k56flex_upstream_pcm_t *s, point_t y)
{
    static const unsigned labels[4] = {0, 3, 1, 2};
    if (!isfinite(y.re) || !isfinite(y.im)) return 0;
    unsigned raw = labels[(y.re < 0) + 2*(y.im < 0)];
    unsigned dibit = (raw - s->mp_previous) & 3;
    s->mp_previous = raw;
    int events = 0;
    for (int i = 0; i < 2; i++) {
        unsigned scrambled = (dibit >> i) & 1;
        unsigned bit = scrambled ^ ((s->mp_history >> 17) & 1) ^ ((s->mp_history >> 22) & 1);
        s->mp_history = (s->mp_history << 1) | scrambled;
        events |= mp_bit(s, bit);
    }
    return events;
}

k56flex_upstream_pcm_t *k56flex_upstream_pcm_new(int baud_code, int high_carrier,
                                               unsigned tap)
{
    if (baud_code < 0 || baud_code > 5 || (tap != 5 && tap != 18)) return NULL;
    k56flex_upstream_pcm_t *s = calloc(1, sizeof(*s));
    if (!s) return NULL;
    s->observations = p3_result_alloc(80100, 0);
    if (!s->observations) { free(s); return NULL; }
    p3_demod_init(&s->demod, baud_code, high_carrier != 0, 8000);
    s->timestamp_delay = (int)lround(32.0*8000/s->demod.baud_rate);
    s->demod.use_rrc_frontend = true;
    s->demod.emit_half_baud = true;
    s->demod.bypass_equalizer = true;
    s->demod.cma_freeze_after_sample = 0;
    s->demod.rrc_agc_gain = 0.0001f;
    s->demod.ted_low_coeff[2] = s->demod.ted_high_coeff[2] = s->demod.ted_mixed_coeff = 0;
    reference(s->ref, tap);
    k56flex_upstream_init(&s->control, tap);
    s->result.trn_sample = s->result.report_sample = s->result.end_sample = -1;
    s->result.sbar_sample = -1;
    s->result.mp_sample = s->result.mp_ack_sample = -1;
    s->search_at = 2*(288+16+64);
    return s;
}

void k56flex_upstream_pcm_free(k56flex_upstream_pcm_t *s)
{
    if (!s) return;
    p3_result_free(s->observations);
    free(s);
}

int k56flex_upstream_pcm_rx(k56flex_upstream_pcm_t *s, const int16_t *pcm, int count,
                           k56flex_upstream_pcm_result_t *out)
{
    int events = 0;
    if (!s || !pcm || count < 0 || !out) return -1;
    if (count > 80000 || count > INT_MAX - s->samples
        || (!s->result.acquired && count > 80000 - s->samples)) return -1;
    p3_result_t *r = s->observations;
    if (s->result.acquired && s->next_symbol > TAPS/2) {
        /* Keep the FIR's history and unprocessed lookahead; timestamps and
         * demodulator/sample clocks remain absolute and uninterrupted. */
        int discard = s->next_symbol - TAPS/2;
        memmove(r->symbols, r->symbols + discard,
                (r->symbol_count - discard)*sizeof(r->symbols[0]));
        r->symbol_count -= discard;
        s->next_symbol -= discard;
    }
    p3_demod_process(&s->demod, pcm, count, s->samples, r);
    s->samples += count;
    /* V.34 10.1.3.7 S/S-bar, used by K56flex's initial upstream
     * preamble: observe the last 64T of S and 16T of S-bar. MICA 13DE/1258
     * releases the downstream Jd wait here, BEFORE PP/TRN and echo training.
     * Waiting for the full 512T training check leaves Jd up during MD.
     * 0.92 is a local normalized-correlation confidence floor, not an ITU
     * timing constant. An unreversed S scores only 0.6 in this window. */
    while (!s->sbar_seen && s->s_search_at + 2*80 + TAPS/2 < r->symbol_count) {
        int at = s->s_search_at++;
        double cr = 0, ci = 0, dr = 0, di = 0, power = 0;
        for (int n = 0; n < 80; n++) {
            double re = (n & 1) ? -0.7071067811865475 : 0.7071067811865475;
            double im = 0.7071067811865475;
            if (n >= 64) { re = -re; im = -im; }
            const p3_symbol_t *p = &r->symbols[at + 2*n];
            cr += p->re*re + p->im*im;
            ci += p->im*re - p->re*im;
            dr += p->re*re - p->im*im;
            di += p->im*re + p->re*im;
            power += p->re*p->re + p->im*p->im;
        }
        double corr = power > 1e-12 ? sqrt(fmax(cr*cr+ci*ci, dr*dr+di*di)/(80*power)) : 0;
        if (corr >= 0.92) {
            s->sbar_seen = 1;
            s->result.sbar_sample = r->symbols[at + 2*64].sample_index + s->timestamp_delay;
            events |= K56U_SBAR;
        }
    }
    while (!s->result.acquired && s->search_at + 2*TRN + TAPS < r->symbol_count) {
        int at = s->search_at++;
        double cr = 0, ci = 0, dr = 0, di = 0, power = 0;
        for (int n = 32; n < 160; ++n) {
            const p3_symbol_t *p = &r->symbols[at + 2*n];
            cr += p->re*s->ref[n].re + p->im*s->ref[n].im;
            ci += p->im*s->ref[n].re - p->re*s->ref[n].im;
            dr += p->re*s->ref[n].re - p->im*s->ref[n].im;
            di += p->im*s->ref[n].re + p->re*s->ref[n].im;
            power += p->re*p->re + p->im*p->im;
        }
        double corr = power > 1e-12 ? sqrt(fmax(cr*cr+ci*ci, dr*dr+di*di)/(128*power)) : 0;
        if (corr > s->result.correlation) s->result.correlation = corr;
        if (corr < 0.65 || !train(r, at, s->ref, s->weights)
            || !validate_training(r, at, s->ref, s->weights, &s->result)) continue;
        s->result.acquired = 1;
        events |= K56U_ACQUIRED;
        s->result.trn_sample = r->symbols[at].sample_index + s->timestamp_delay;
        s->next_symbol = at;
    }
    if (s->result.acquired) {
        while (s->next_symbol + TAPS/2 < r->symbol_count) {
            int at = s->next_symbol;
            point_t y = equalize(r, at, s->weights);
            int ev = s->control.ended ? mp_symbol(s, y)
                     : k56flex_upstream_symbol(&s->control, (float)y.re, (float)y.im);
            s->next_symbol += 2;
            if (ev < 0) continue;
            if (ev & K56U_REPORT) {
                s->result.ext = s->control.report.ext;
                s->result.report = s->control.report.report;
                s->result.report_sample = r->symbols[at].sample_index + s->timestamp_delay;
            }
            if (ev & K56U_REPORT_END) s->result.end_sample = r->symbols[at].sample_index + s->timestamp_delay;
            if (ev & K56U_MP) s->result.mp_sample = r->symbols[at].sample_index + s->timestamp_delay;
            if (ev & K56U_MP_ACK) s->result.mp_ack_sample = r->symbols[at].sample_index + s->timestamp_delay;
            events |= ev;
        }
    }
    *out = s->result;
    return events;
}
