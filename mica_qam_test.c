/* MICA resident QAM engine tests (mica_qam.c). Firmware matches are the
 * tools/mica_qam_*_oracle.py fixtures; these pin their properties without
 * MicaEmu and add independent checks. */
#include "mica_qam.h"

#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(cond, ...) do { if (!(cond)) { ++failures; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static double goertzel(const int16_t *x, unsigned n, double coefficient)
{
    /* coefficient = 2 cos(2 pi f / 8000); n = 512, Hann window built by
     * rotating (cos, sin) through 2 pi / 512 so no libm is needed. */
    double s1 = 0, s2 = 0, c = 1, si = 0;
    for (unsigned i = 0; i < n; ++i) {
        double s0 = x[i]*(0.5 - 0.5*c) + coefficient*s1 - s2, t;
        s2 = s1; s1 = s0;
        t = c*0.99992470183914450 - si*0.012271538285719925;
        si = si*0.99992470183914450 + c*0.012271538285719925; c = t;
    }
    return s1*s1 + s2*s2 - coefficient*s1*s2;
}

static void test_word_rx(void)
{
    unsigned tap, payload, offset, valid;
    for (tap = 5; tap <= 18; tap += 13)
        for (payload = 0; payload < 32; ++payload)
            for (offset = 0; offset < 16; offset += 2)
                for (valid = 0; valid < 2; ++valid) {
                    mica_qam_word_rx_t rx;
                    uint32_t history = 0;
                    uint16_t word = 0x8880 | ((payload * 0x1230) & 0x7770) | !valid;
                    unsigned n, dibit = 0;
                    CHECK(mica_qam_word_rx_init(&rx, tap) == 0, "response init");
                    for (n = 0; n < 224; ++n) {
                        unsigned x = n < offset ? 0 : (word >> ((n-offset) % 16)) & 1;
                        unsigned y = x ^ ((history >> (tap-1)) & 1) ^ ((history >> 22) & 1);
                        history = (history << 1) | y;
                        dibit |= y << (n & 1);
                        if (n & 1) {
                            mica_qam_word_rx_dibit(&rx, dibit);
                            dibit = 0;
                        }
                    }
                    CHECK(rx.accepted == (int)valid, "response tap %u offset %u valid %u", tap, offset, valid);
                    if (valid) CHECK(rx.word == word, "response word");
                }
}

static void test_coordinates(void)
{
    static const int16_t points[4][2] = {{12953,0},{0,12953},{-12953,0},{0,-12953}};
    static const unsigned labels[4] = {0,1,2,3};
    static const unsigned diff[16] = {2,0,1,3,3,2,0,1,1,3,2,0,0,1,3,2};
    unsigned i, j;
    for (i = 0; i < 4; ++i) {
        CHECK(mica_qam_slice(points[i][0], points[i][1]) == labels[i], "feedback point %u", i);
        for (j = 0; j < 4; ++j) {
            unsigned previous = i;
            CHECK(mica_qam_dibit(&previous, j) == diff[4*i+j], "feedback differential %u/%u", i, j);
            CHECK(previous == j, "feedback history");
        }
    }
    CHECK(mica_qam_slice(9000,9000) == 0, "feedback positive tie");
    CHECK(mica_qam_slice(-9000,-9000) == 2, "feedback negative tie");
}

static void test_filter(void)
{
    int16_t input[256] = {0}, coefficients[192] = {0}, out[4], real, imag;
    unsigned phase;
    mica_qam_rotate(-1, 1, 16384, 0, 0, &real, &imag);
    CHECK(real == 0 && imag == 1, "rotor signed half rounding");
    mica_qam_rotate(32767, -32768, 0, 32767, 0, &real, &imag);
    CHECK(real == 32767 && imag == 32766, "rotor quadrant");
    coefficients[0] = 8192;
    coefficients[96 + 48] = 8192;
    for (phase = 0; phase < 256; ++phase) {
        memset(input, 0, sizeof(input));
        input[(phase - 2) & 255] = -123;
        input[(phase - 1) & 255] = 456;
        mica_qam_fir(input, phase, coefficients, out);
        CHECK(out[0] == -123 && out[1] == 456 && out[2] == -456 && out[3] == -123,
              "FIR row/phase %u", phase);
    }
    memset(coefficients, 0, sizeof(coefficients));
    coefficients[0] = 4096;
    input[0] = -1; input[1] = 1;
    mica_qam_fir(input, 2, coefficients, out);
    CHECK(out[0] == 0 && out[1] == 1, "FIR signed half rounding");
}

static void test_adaptation(void)
{
    int16_t cf[192] = {0}, h[256] = {0}, input[256] = {0}, errors[128] = {0};
    mica_qam_adapt_t state = {7, 12, 0};
    h[12] = 16384;
    errors[0] = 100; errors[1] = 40;
    errors[2] = -20; errors[3] = 80;
    CHECK(mica_qam_adapt(&state, cf, h, 0, 0, input, 0, errors, 4) == 0, "adapt call");
    CHECK(cf[7] == 70 && cf[55] == -30 && cf[103] == 30 && cf[151] == 50,
          "adapt updates both complex rows");
    CHECK(state.tap == 8 && state.history_index == 11 && state.remaining == 2, "adapt sweep state");
    state.tap = 48;
    CHECK(mica_qam_adapt(&state, cf, h, 0, 0, input, 0, errors, 4) == -1 && state.tap == 48,
          "zero-spacing refresh needs full PM model");
    input[0] = 10; input[1] = 20; input[254] = 30; input[255] = 40;
    input[252] = 50; input[253] = 60;
    CHECK(mica_qam_adapt(&state, cf, h, 1, 0, input, 0, errors, 4) == 0, "adapt refresh");
    CHECK(h[0] == 10 && h[1] == 20 && h[2] == 30 && h[3] == 40 && h[4] == 50 && h[5] == 60,
          "adapt history source wrap");
    CHECK(state.tap == 1 && state.history_index == 1 && state.remaining == 1, "adapt refreshed sweep");
}

static void test_resampling(void)
{
    int16_t raw[128] = {0}, output[256] = {0};
    mica_qam_resample_t state = {127, 254, 0, 1};
    unsigned i;
    CHECK(mica_qam_resample(&state, raw, 2, 0, 15, 512, 3, output) == 0, "resample zero/bias");
    CHECK(state.source_cursor == 3 && state.output_cursor == 4 && state.output_available == 3 && !state.slip,
          "resample slip and ring accounting");
    for (i = 0; i < 6; ++i) CHECK(output[(254+i)&255] == 3, "resample bias output");
    state.slip = -1;
    CHECK(mica_qam_resample(&state, raw, 0, 0, 15, 512, 0, output) == -1 && state.slip == -1,
          "resample invalid negative count does not mutate");
    for (i = 0; i < 128; ++i) raw[i] = 1200;
    state = (mica_qam_resample_t){0, 0, 0, 0};
    CHECK(mica_qam_resample(&state, raw, 1, 1, 14, -32768, 0, output) == 0, "resample extreme gain");
    CHECK(output[0] == 32767 && output[1] == 32767, "resample scaled product wraps before saturation");
}

static void test_clock(void)
{
    uint16_t s[128] = {0};
    int16_t ring[256] = {0};
    s[0x5d] = 0x10; /* freeze still applies the existing rate correction */
    s[0x45] = 2; s[0x5e] = 1;
    CHECK(mica_qam_timing(s, ring, 0, 1) == 0 && s[0x5c] == 64, "timing frozen rate term");
    mica_qam_phase(s);
    CHECK(s[0x5e] == 1 && s[0x5f] == 0 && s[0x5c] == 0, "phase freeze clears correction");
    s[0x5d] = 0x20; s[0x5b] = 0; s[0x5c] = 64; s[0x3e] = 1;
    mica_qam_phase(s);
    CHECK(s[0x5e] == 0 && s[0x5f] == 65472 && s[0x3e] == 1, "phase subtraction retains pending slip");
    s[0x3e] = 0; s[0x64] = 0x1000;
    mica_qam_phase(s);
    CHECK(s[0x3f] == 0xf000 && s[0x3e] == 0xf000, "phase coarse boundary");
}

static void test_initialization(void)
{
    uint16_t state[128] = {0};
    unsigned i, count=0;
    state[0x5d]=0x10;
    mica_qam_timing_init(state,0);
    CHECK(state[0x57]==0x140 && state[0x58]==0x1800 && state[0x43]==0x7fff && state[0x5d]==0x10,
          "timing startup initializer preserves flags");
    mica_qam_timing_init(state,1);
    CHECK(state[0x57]==0x324 && state[0x58]==0x800, "timing symbol constants");
    memset(state,0,sizeof(state)); state[9]=1; state[0x45]=3;
    for(i=0;i<30;++i) { mica_qam_block_count(state); count+=state[0x2b]; }
    CHECK(count==30 && state[0x42]==0, "block cadence conserves supplied samples");
}

static void test_startup_samples(void)
{
    /* Properties the 8270 oracles pinned (tools/mica_qam_startup_*_oracle.py). */
    uint16_t state[128] = {0}, before[128], address;
    int16_t history[64], banks[20], output[128] = {0}, sample;
    const int16_t unit[1] = {4096}, tiny[1] = {1};
    unsigned h = 5, o = 126, j, pattern[3], total = 0;
    for (j = 0; j < 64; ++j) history[j] = (int16_t)(100 * j - 3000);
    CHECK(mica_qam_startup_fir(history, 5, unit, 1, &sample) == 0 && sample == history[5],
          "startup FIR unit tap");
    history[7] = -1;
    CHECK(mica_qam_startup_fir(history, 7, tiny, 1, &sample) == 0 && sample == -1,
          "startup FIR SACH shift 4 floors negative sums");
    for (j = 0; j < 10; ++j) { banks[2*j] = 4096; banks[2*j+1] = (int16_t)(j * 409); }
    state[0x27] = 10; state[0x28] = 3; state[0x29] = 1; state[0x2a] = 0x7900;
    state[0x4d] = 10 - 3; /* 82B4 initializer */
    for (j = 0; j < 3; ++j) {
        uint16_t ref[128]; unsigned rh = h, ro = o; int k, n;
        int16_t want[128];
        memcpy(ref, state, sizeof(ref)); memcpy(want, output, sizeof(want));
        n = mica_qam_startup_sample_count(ref, 1);
        for (k = 0; k < n; ++k) {
            mica_qam_startup_phase(ref, &rh, &address);
            CHECK(address == (uint16_t)(ref[0x4d] + 0x7900), "startup bank lookup address");
            mica_qam_startup_fir(history, rh, banks + ref[0x4d]*2, 2, &want[ro]);
            ro = (ro + 1) & 127;
        }
        pattern[j] = (unsigned)mica_qam_startup_samples(state, 1, history, &h, banks, 20, output, &o);
        CHECK(pattern[j] == (unsigned)n && !memcmp(state, ref, sizeof(ref)) && h == rh && o == ro
              && !memcmp(output, want, sizeof(want)), "startup samples equal phase+FIR composition");
        total += pattern[j];
    }
    CHECK(pattern[0] == 3 && pattern[1] == 3 && pattern[2] == 4 && total == 10,
          "startup 10/3 sample pattern 3,3,4 (got %u,%u,%u)", pattern[0], pattern[1], pattern[2]);
    CHECK(h == 11 && o == 8, "one history pair per symbol, output ring wraps (h=%u o=%u)", h, o);
    memcpy(before, state, sizeof(before));
    CHECK(mica_qam_startup_samples(state, 1, history, &h, banks, 19, output, &o) == -1
          && !memcmp(state, before, sizeof(before)) && h == 11 && o == 8,
          "startup samples rejects short banks without mutation");
}

static void test_v32bis_tx(void)
{
    /* Module configuration (8F23 profile 0, 94BA select 1, 82B4, 83A3); the
     * per-symbol match with the DSP is tools/mica_v32bis_tx_oracle.py.
     * Independent check: 2400 baud on 1800 Hz occupies 600..3000 Hz. */
    static int16_t pcm[16000];
    mica_v32bis_tx_t tx, saved;
    unsigned n = 0, words = 0, symbols = 0, seed = 1;
    int16_t out[4] = {11, 22, 33, 44};
    int used = -1, count;
    double band = 0, low = 0, high = 0;
    CHECK(mica_v32bis_tx_init(&tx, 2, 1, 1, 4096, 4096) == 0, "startup tx init");
    CHECK(tx.state[0x27] == 10 && tx.state[0x28] == 3 && tx.state[0x29] == 0x2d && tx.state[0x2a] == 0xe504
          && tx.state[0x2d] == 0x9002 && tx.state[0x2e] == 0x900a && tx.state[0x4d] == 7,
          "startup tx profile 0, carrier select 1, 82B4/83A3");
    while (n + 4 <= sizeof(pcm)/sizeof(pcm[0])) {
        seed = seed*1103515245u + 12345u;
        count = mica_v32bis_tx_symbol(&tx, (uint16_t)(seed >> 8), &pcm[n], &used);
        if (count < 0) break;
        n += (unsigned)count; words += (unsigned)used; ++symbols;
    }
    CHECK(n == symbols*10/3 || n == symbols*10/3 + 1, "startup tx 10 samples per 3 symbols (%u, %u)", n, symbols);
    CHECK(words == (symbols*2 + 15)/16, "startup tx consumes one word per 8 dibits (%u, %u)", words, symbols);
    for (unsigned i = 1024; i + 512 <= n; i += 512) {
        band += goertzel(&pcm[i], 512, 0.31286893008046185) + goertzel(&pcm[i], 512, 1.7820130483767358)
              + goertzel(&pcm[i], 512, -1.414213562373095);
        low += goertzel(&pcm[i], 512, 1.9447398407953531);
        high += goertzel(&pcm[i], 512, -1.7052803287081844);
    }
    CHECK(band > 3000*low && band > 3000*high,
          "startup tx passband 600..3000 Hz, >30 dB above 300/3300 Hz (%.0f %.0f %.0f)", band, low, high);
    saved = tx; used = 99;
    tx.state[0x13] = 0x900a;
    CHECK(mica_v32bis_tx_symbol(&tx, 0, out, &used) == -1 && out[0] == 11 && used == 99,
          "startup tx rejects a carrier pointer outside the table");
    tx = saved;
}

int main(void)
{
    test_word_rx();
    test_coordinates();
    test_filter();
    test_adaptation();
    test_resampling();
    test_clock();
    test_initialization();
    test_startup_samples();
    test_v32bis_tx();
    printf(failures ? "mica_qam_test: %d FAILURES\n" : "mica_qam_test: all passed\n", failures);
    return failures != 0;
}
