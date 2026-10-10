/* K56flex payload core, V.8bis framing, report and parameter record tests.
 * Firmware vectors come from tools/generate_k56flex_vectors.py (original MICA C53
 * module 8C executed in the isolated repeat-fetch core); everything else is an
 * independent model check. */
#include "k56flex.h"
#include "k56flex_tables.h"
#include "k56flex_firmware_vectors.h"

#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(cond, ...) do { if (!(cond)) { ++failures; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

typedef struct { const uint16_t *w; unsigned n, i; int pause_every, calls; } wsrc_t;

static int wsrc(void *u, uint16_t *w)
{
    wsrc_t *s = u;
    if (s->pause_every && ++s->calls % s->pause_every == 0) return -1;
    if (s->i >= s->n) return -1;
    *w = s->w[s->i++];
    return 0;
}

static void vector_cfg(const k56flex_vector_t *v, k56flex_pcm_config_t *c)
{
    c->law = (k56flex_law_t)v->law;
    c->rate_bps = (v->rate_index - 3) * 2000;
    c->report_field = v->field;
}

static void test_table_law(void)
{
    unsigned i, j;
    for (i = 0; i < K56FLEX_TABLE_COUNT; ++i) {
        const k56flex_table_t *t = &k56flex_tables[i];
        for (j = 0; j < t->level_count; ++j) {
            int level = k56flex_levels[t->first_level + j];
            CHECK(level > 0 && k56flex_g711_from_level((k56flex_law_t)t->law, level) >= 0,
                  "table %u level %d is not a %s codeword magnitude", i, level,
                  t->law ? "A-law" : "mu-law");
        }
    }
    /* The probe values in clause 4.10.1 are codewords of the same law the table says. */
    CHECK(k56flex_g711_from_level(K56FLEX_LAW_MU, 3772) >= 0 && k56flex_g711_from_level(K56FLEX_LAW_A, 3772) < 0,
          "3772 is a mu-law value");
    CHECK(k56flex_g711_from_level(K56FLEX_LAW_A, 3904) >= 0 && k56flex_g711_from_level(K56FLEX_LAW_MU, 3904) < 0,
          "3904 is an A-law value");
    CHECK(k56flex_g711_from_level(K56FLEX_LAW_MU, 0) == 0xff && k56flex_g711_from_level(K56FLEX_LAW_MU, -3772) >= 0, "mu zero/neg");
    /* Spec compander: code = ((seg<<4)|mant) ^ (0x7f if v<0 else 0xff). */
    {
        int v, c = 0;
        for (v = -8000; v <= 8000; ++v) {
            int a = (v < 0 ? -v : v) + 132, seg = 0, got = k56flex_g711_from_level(K56FLEX_LAW_MU, v);
            while ((a >> (seg + 8)) != 0) ++seg;
            if (got >= 0) {
                int code = ((seg << 4) | ((a >> (seg + 3)) & 15)) ^ (v < 0 ? 0x7f : 0xff);
                CHECK(got == code || v == 0, "compander %d: %02x vs %02x", v, got, code);
                ++c;
            }
        }
        CHECK(c > 150, "compander coverage %d", c);
    }
}

static void test_firmware_vectors(void)
{
    unsigned vi, frames = 0;
    for (vi = 0; vi < K56FLEX_VECTOR_COUNT; ++vi) {
        const k56flex_vector_t *v = &k56flex_vectors[vi];
        const uint16_t *words = &k56flex_vector_words[v->word_offset];
        const int16_t *want = &k56flex_vector_samples[v->sample_offset];
        k56flex_pcm_config_t cfg;
        k56flex_pcm_tx_t tx;
        wsrc_t src = {words, v->nwords, 0, 0, 0};
        unsigned f;
        vector_cfg(v, &cfg);
        if (k56flex_pcm_tx_init(&tx, &cfg, wsrc, &src) < 0) { CHECK(0, "init vec %u", vi); continue; }
        for (f = 0; f < v->frames; ++f) {
            int16_t got[8];
            int r = k56flex_pcm_tx_frame(&tx, got);
            if (r != 1) { CHECK(0, "vec %u frame %u -> %d", vi, f, r); break; }
            if (memcmp(got, want + 8 * f, sizeof(got))) {
                CHECK(0, "vec %u (law %u rate %u field %03x) frame %u differs", vi, v->law, v->rate_index, v->field, f);
                break;
            }
            ++frames;
        }
    }
    printf("firmware vectors: %u runs, %u frames, %u samples\n", K56FLEX_VECTOR_COUNT, frames, frames * 8);
}

static void test_roundtrip_and_blocks(void)
{
    unsigned vi, rt = 0;
    for (vi = 0; vi < K56FLEX_VECTOR_COUNT; ++vi) {
        const k56flex_vector_t *v = &k56flex_vectors[vi];
        const uint16_t *words = &k56flex_vector_words[v->word_offset];
        k56flex_pcm_config_t cfg;
        k56flex_pcm_tx_t tx;
        k56flex_pcm_rx_t rx;
        wsrc_t src = {words, v->nwords, 0, 0, 0};
        uint8_t octets[8], bits[K56FLEX_MAX_FRAME_BITS];
        uint8_t ref[8192], paused[2048];
        unsigned f, consumed = 0, b;
        size_t n = (size_t)v->frames * 8, got, blk;
        vector_cfg(v, &cfg);
        k56flex_pcm_tx_init(&tx, &cfg, wsrc, &src);
        k56flex_pcm_rx_init(&rx, &cfg);
        CHECK(k56flex_pcm_frame_bits(&cfg) != 0, "frame bits");
        for (f = 0; f < v->frames; ++f) {
            int16_t lv[8];
            unsigned i;
            int r, nb;
            if (k56flex_pcm_tx_g711(&tx, octets, 8) != 8) { CHECK(0, "vec %u g711 short", vi); break; }
            memcpy(ref + 8 * f, octets, 8);
            (void)lv; (void)i;
            nb = k56flex_pcm_rx_frame(&rx, octets, bits);
            r = nb;
            if (r < 0) { CHECK(0, "vec %u frame %u rx failed", vi, f); break; }
            for (b = 0; b < (unsigned)nb; ++b) {
                unsigned pos = consumed + b;
                CHECK(bits[b] == ((words[pos / 16] >> (pos % 16)) & 1), "vec %u bit %u mismatch", vi, pos);
                if (bits[b] != ((words[pos / 16] >> (pos % 16)) & 1)) goto next;
            }
            consumed += (unsigned)nb;
            ++rt;
        }
        /* The same stream through odd output block sizes and a source that pauses. */
        for (blk = 1; blk <= 160; blk = blk * 7 + 3) {
            wsrc_t s2 = {words, v->nwords, 0, 3, 0};
            k56flex_pcm_tx_t t2;
            size_t pos = 0;
            int guard = 0;
            k56flex_pcm_tx_init(&t2, &cfg, wsrc, &s2);
            while (pos < n && guard++ < 100000) {
                size_t want = n - pos < blk ? n - pos : blk;
                got = k56flex_pcm_tx_g711(&t2, paused, want);
                if (got > want) { CHECK(0, "overrun"); break; }
                if (memcmp(paused, ref + pos, got)) { CHECK(0, "vec %u block %zu stream differs", vi, blk); pos = n; break; }
                pos += got;
            }
            CHECK(pos == n, "vec %u block %zu stalled at %zu/%zu", vi, blk, pos, n);
        }
    next:;
    }
    printf("round trip: %u frames recovered exactly through G.711\n", rt);
}

static void test_rejects(void)
{
    k56flex_pcm_config_t c = {K56FLEX_LAW_MU, 56000, 0};
    k56flex_pcm_tx_t tx;
    k56flex_pcm_rx_t rx;
    uint8_t bits[K56FLEX_MAX_FRAME_BITS], oct[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
    CHECK(k56flex_pcm_tx_init(&tx, &c, NULL, NULL) == 0, "56k ok");
    c.rate_bps = 31000; CHECK(k56flex_pcm_tx_init(&tx, &c, NULL, NULL) < 0, "31k");
    c.rate_bps = 62000; CHECK(k56flex_pcm_tx_init(&tx, &c, NULL, NULL) < 0, "62k");
    c.rate_bps = 60000; CHECK(k56flex_pcm_tx_init(&tx, &c, NULL, NULL) == 0, "60k unadjusted");
    c.report_field = 0x40; CHECK(k56flex_pcm_tx_init(&tx, &c, NULL, NULL) < 0, "60k has no adjusted table");
    c.rate_bps = 56000; c.report_field = 0xc0; CHECK(k56flex_pcm_tx_init(&tx, &c, NULL, NULL) < 0, "both table bits");
    c.report_field = 0x1000; CHECK(k56flex_pcm_tx_init(&tx, &c, NULL, NULL) < 0, "field overflow");
    c.law = (k56flex_law_t)5; c.report_field = 0; CHECK(k56flex_pcm_tx_init(&tx, &c, NULL, NULL) < 0, "law");
    c.law = K56FLEX_LAW_MU;
    k56flex_pcm_rx_init(&rx, &c);
    CHECK(k56flex_pcm_rx_frame(&rx, oct, bits) < 0, "all-zero octets are not a valid frame");
}

static void test_report(void)
{
    unsigned ext, pace;
    const unsigned tap = 5;
    uint8_t in[24 * 6], out[24 * 6];
    for (ext = 0; ext < 256; ++ext)
        for (pace = 0; pace < 2; ++pace) {
            uint16_t rep = (uint16_t)(0x8880 | (pace << 4));
            unsigned want = ext | ((unsigned)__builtin_popcount(ext & 63) << 9) | (pace << 8);
            CHECK(k56flex_report_field((uint8_t)ext, rep) == want, "field ext %u", ext);
        }
    CHECK(k56flex_report_header_ok(0x8880) && !k56flex_report_header_ok(0x0880) && !k56flex_report_header_ok(0x8881), "header");
    for (ext = 0; ext < 64; ++ext) {
            unsigned off, i;
            uint32_t rec = k56flex_report_record((uint8_t)ext, 0x8890);
            for (i = 0; i < sizeof(in); ++i) in[i] = (rec >> (i % 24)) & 1;
            for (off = 0; off < 24; off += 5) {
                k56flex_report_rx_t rx;
                unsigned n, got = 0;
                k56flex_report_scramble(in, out, sizeof(out));
                k56flex_report_rx_init(&rx, tap);
                /* start mid-record: drop `off` bits so the alignment is unknown */
                for (n = off; n < sizeof(out); ++n) if (k56flex_report_rx_bit(&rx, out[n])) { got = 1; break; }
                                CHECK(got, "report ext %u off %u not acquired", ext, off);
                if (got) CHECK(rx.ext == ext && rx.report == 0x8890, "report decoded %02x/%04x", rx.ext, rx.report);
            }
        }
}

static void test_response(void)
{
    unsigned tap, payload, offset, valid;
    for (tap = 5; tap <= 18; tap += 13)
        for (payload = 0; payload < 32; ++payload)
            for (offset = 0; offset < 16; offset += 2)
                for (valid = 0; valid < 2; ++valid) {
                    k56flex_response_rx_t rx;
                    uint32_t history = 0;
                    uint16_t word = 0x8880 | ((payload * 0x1230) & 0x7770) | !valid;
                    unsigned n, dibit = 0;
                    CHECK(k56flex_response_rx_init(&rx, tap) == 0, "response init");
                    for (n = 0; n < 224; ++n) {
                        unsigned x = n < offset ? 0 : (word >> ((n-offset) % 16)) & 1;
                        unsigned y = x ^ ((history >> (tap-1)) & 1) ^ ((history >> 22) & 1);
                        history = (history << 1) | y;
                        dibit |= y << (n & 1);
                        if (n & 1) {
                            k56flex_response_rx_dibit(&rx, dibit);
                            dibit = 0;
                        }
                    }
                    CHECK(rx.accepted == (int)valid, "response tap %u offset %u valid %u", tap, offset, valid);
                    if (valid) CHECK(rx.word == word, "response word");
                }
}

static void test_feedback_coordinates(void)
{
    static const int16_t points[4][2] = {{12953,0},{0,12953},{-12953,0},{0,-12953}};
    static const unsigned labels[4] = {0,1,2,3};
    static const unsigned diff[16] = {2,0,1,3,3,2,0,1,1,3,2,0,0,1,3,2};
    unsigned i, j;
    for (i = 0; i < 4; ++i) {
        CHECK(k56flex_feedback_slice(points[i][0], points[i][1]) == labels[i], "feedback point %u", i);
        for (j = 0; j < 4; ++j) {
            unsigned previous = i;
            CHECK(k56flex_feedback_dibit(&previous, j) == diff[4*i+j], "feedback differential %u/%u", i, j);
            CHECK(previous == j, "feedback history");
        }
    }
    CHECK(k56flex_feedback_slice(9000,9000) == 0, "feedback positive tie");
    CHECK(k56flex_feedback_slice(-9000,-9000) == 2, "feedback negative tie");
}

static void test_feedback_filter(void)
{
    int16_t input[256] = {0}, coefficients[192] = {0}, out[4], real, imag;
    unsigned phase;
    k56flex_feedback_rotate(-1, 1, 16384, 0, 0, &real, &imag);
    CHECK(real == 0 && imag == 1, "rotor signed half rounding");
    k56flex_feedback_rotate(32767, -32768, 0, 32767, 0, &real, &imag);
    CHECK(real == 32767 && imag == 32766, "rotor quadrant");
    coefficients[0] = 8192;
    coefficients[96 + 48] = 8192;
    for (phase = 0; phase < 256; ++phase) {
        memset(input, 0, sizeof(input));
        input[(phase - 2) & 255] = -123;
        input[(phase - 1) & 255] = 456;
        k56flex_feedback_fir(input, phase, coefficients, out);
        CHECK(out[0] == -123 && out[1] == 456 && out[2] == -456 && out[3] == -123,
              "FIR row/phase %u", phase);
    }
    memset(coefficients, 0, sizeof(coefficients));
    coefficients[0] = 4096;
    input[0] = -1; input[1] = 1;
    k56flex_feedback_fir(input, 2, coefficients, out);
    CHECK(out[0] == 0 && out[1] == 1, "FIR signed half rounding");
}

static void test_feedback_adaptation(void)
{
    int16_t cf[192] = {0}, h[256] = {0}, input[256] = {0}, errors[128] = {0};
    k56flex_feedback_adapt_t state = {7, 12, 0};
    h[12] = 16384;
    errors[0] = 100; errors[1] = 40;
    errors[2] = -20; errors[3] = 80;
    CHECK(k56flex_feedback_adapt(&state, cf, h, 0, 0, input, 0, errors, 4) == 0, "adapt call");
    CHECK(cf[7] == 70 && cf[55] == -30 && cf[103] == 30 && cf[151] == 50,
          "adapt updates both complex rows");
    CHECK(state.tap == 8 && state.history_index == 11 && state.remaining == 2, "adapt sweep state");
    state.tap = 48;
    CHECK(k56flex_feedback_adapt(&state, cf, h, 0, 0, input, 0, errors, 4) == -1 && state.tap == 48,
          "zero-spacing refresh needs full PM model");
    input[0] = 10; input[1] = 20; input[254] = 30; input[255] = 40;
    input[252] = 50; input[253] = 60;
    CHECK(k56flex_feedback_adapt(&state, cf, h, 1, 0, input, 0, errors, 4) == 0, "adapt refresh");
    CHECK(h[0] == 10 && h[1] == 20 && h[2] == 30 && h[3] == 40 && h[4] == 50 && h[5] == 60,
          "adapt history source wrap");
    CHECK(state.tap == 1 && state.history_index == 1 && state.remaining == 1, "adapt refreshed sweep");
}

static void test_feedback_resampling(void)
{
    int16_t raw[128] = {0}, output[256] = {0};
    k56flex_feedback_resample_t state = {127, 254, 0, 1};
    unsigned i;
    CHECK(k56flex_feedback_resample(&state, raw, 2, 0, 15, 512, 3, output) == 0, "resample zero/bias");
    CHECK(state.source_cursor == 3 && state.output_cursor == 4 && state.output_available == 3 && !state.slip,
          "resample slip and ring accounting");
    for (i = 0; i < 6; ++i) CHECK(output[(254+i)&255] == 3, "resample bias output");
    state.slip = -1;
    CHECK(k56flex_feedback_resample(&state, raw, 0, 0, 15, 512, 0, output) == -1 && state.slip == -1,
          "resample invalid negative count does not mutate");
    for (i = 0; i < 128; ++i) raw[i] = 1200;
    state = (k56flex_feedback_resample_t){0, 0, 0, 0};
    CHECK(k56flex_feedback_resample(&state, raw, 1, 1, 14, -32768, 0, output) == 0, "resample extreme gain");
    CHECK(output[0] == 32767 && output[1] == 32767, "resample scaled product wraps before saturation");
}

static void test_feedback_clock(void)
{
    uint16_t s[128] = {0};
    int16_t ring[256] = {0};
    s[0x5d] = 0x10; /* freeze still applies the existing rate correction */
    s[0x45] = 2; s[0x5e] = 1;
    CHECK(k56flex_feedback_timing(s, ring, 0, 1) == 0 && s[0x5c] == 64, "timing frozen rate term");
    k56flex_feedback_phase(s);
    CHECK(s[0x5e] == 1 && s[0x5f] == 0 && s[0x5c] == 0, "phase freeze clears correction");
    s[0x5d] = 0x20; s[0x5b] = 0; s[0x5c] = 64; s[0x3e] = 1;
    k56flex_feedback_phase(s);
    CHECK(s[0x5e] == 0 && s[0x5f] == 65472 && s[0x3e] == 1, "phase subtraction retains pending slip");
    s[0x3e] = 0; s[0x64] = 0x1000;
    k56flex_feedback_phase(s);
    CHECK(s[0x3f] == 0xf000 && s[0x3e] == 0xf000, "phase coarse boundary");
}

static void test_feedback_initialization(void)
{
    uint16_t state[128] = {0};
    unsigned i, count=0;
    state[0x5d]=0x10;
    k56flex_feedback_timing_init(state,0);
    CHECK(state[0x57]==0x140 && state[0x58]==0x1800 && state[0x43]==0x7fff && state[0x5d]==0x10,
          "timing startup initializer preserves flags");
    k56flex_feedback_timing_init(state,1);
    CHECK(state[0x57]==0x324 && state[0x58]==0x800, "timing symbol constants");
    memset(state,0,sizeof(state)); state[9]=1; state[0x45]=3;
    for(i=0;i<30;++i) { k56flex_feedback_block_count(state); count+=state[0x2b]; }
    CHECK(count==30 && state[0x42]==0, "block cadence conserves supplied samples");
}

static void test_v8bis(void)
{
    static const uint8_t check[9] = {'1','2','3','4','5','6','7','8','9'};
    static const uint8_t fcs_tail[4][2] = {{0x76,0xd2},{0x57,0x48},{0xdd,0xa6},{0xb1,0x6c}};
    static const k56flex_v8bis_msg_t kinds[4] = {K56FLEX_V8BIS_MS, K56FLEX_V8BIS_CL, K56FLEX_V8BIS_ACK1, K56FLEX_V8BIS_NAK1};
    static const unsigned bitcount[4] = {184, 185, 64, 64};
    unsigned i;
    CHECK(k56flex_v8bis_fcs(check, 9) == 0x906e, "CRC check value");
    for (i = 0; i < 4; ++i) {
        uint8_t p[16], f[32], bits[300];
        size_t n = k56flex_v8bis_payload(kinds[i], 0, 0, p), fn = k56flex_v8bis_frame(p, n, f);
        CHECK(fn == n + 7 && f[3 + n] == fcs_tail[i][0] && f[4 + n] == fcs_tail[i][1], "FCS template %u: %02x %02x", i, f[3 + n], f[4 + n]);
        CHECK(k56flex_v8bis_stuff(f, fn, bits) == bitcount[i], "stuffed bits template %u", i);
    }
    {
        uint8_t p[16];
        k56flex_v8bis_payload(K56FLEX_V8BIS_MS, 1, 1, p);
        CHECK(p[12] == 0x83 && p[15] == 0xe0 && p[0] == 0x11, "template edits");
        k56flex_v8bis_payload(K56FLEX_V8BIS_CL, 0, 0, p);
        CHECK(p[12] == 0x81 && p[15] == 0xc0 && p[0] == 0x12 && p[13] == 0x42, "CL template");
    }
    /* Stuffing: no six-ones run outside literal 0x7e, and removal restores the octets. */
    {
        uint8_t o[32], bits[300], back[32];
        size_t n, i2, k, ones = 0, m = 0;
        for (i2 = 0; i2 < sizeof(o); ++i2) o[i2] = (uint8_t)(i2 % 3 ? 0xff : 0xbf);
        n = k56flex_v8bis_stuff(o, sizeof(o), bits);
        for (i2 = 0; i2 < n; ++i2) {
            ones = bits[i2] ? ones + 1 : 0;
            CHECK(ones < 6, "six ones at %zu", i2);
        }
        for (i2 = 0, ones = 0; i2 < n; ++i2) {
            if (ones >= 5 && !bits[i2]) { ones = 0; continue; }
            k = m++;
            if (k / 8 < sizeof(back)) { if (k % 8 == 0) back[k / 8] = 0; back[k / 8] |= (uint8_t)(bits[i2] << (k % 8)); }
            ones = bits[i2] ? ones + 1 : 0;
        }
        CHECK(m == 8 * sizeof(o) && !memcmp(o, back, sizeof(o)), "unstuff");
    }
}

static void test_params(void)
{
    unsigned i, checked = 0;
    for (i = 0; i < K56FLEX_PARAM_VECTOR_COUNT; ++i) {
        const k56flex_param_vector_t *v = &k56flex_param_vectors[i];
        k56flex_param_t p = {v->mode, v->extra, v->rate, v->suppress_extra, v->u, v->v, v->final, v->control, v->bit, v->special};
        uint16_t out[12], raw[9], parsed[9];
        size_t n = k56flex_param_source(&p, out), nraw = k56flex_param_raw(&p, raw);
        CHECK(n == (v->mode ? 12u : 6u) && !memcmp(out, v->queued, n * sizeof(out[0])), "param vec %u source differs", i);
        CHECK(k56flex_param_parse(v->mode, out, parsed) == (int)nraw && !memcmp(parsed, raw, nraw * sizeof(raw[0])), "param vec %u parse", i);
        out[2] ^= 0x0100;
        CHECK(k56flex_param_parse(v->mode, out, parsed) < 0, "param vec %u corruption not detected", i);
        ++checked;
    }
    printf("parameter records: %u firmware vectors\n", checked);
}

int main(void)
{
    test_table_law();
    test_firmware_vectors();
    test_roundtrip_and_blocks();
    test_rejects();
    test_report();
    test_response();
    test_feedback_coordinates();
    test_feedback_filter();
    test_feedback_adaptation();
    test_feedback_resampling();
    test_feedback_clock();
    test_feedback_initialization();
    test_v8bis();
    test_params();
    printf(failures ? "k56flex_test: %d FAILURES\n" : "k56flex_test: all passed\n", failures);
    return failures != 0;
}
