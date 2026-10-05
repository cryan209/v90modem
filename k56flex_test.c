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
    test_v8bis();
    test_params();
    printf(failures ? "k56flex_test: %d FAILURES\n" : "k56flex_test: all passed\n", failures);
    return failures != 0;
}
