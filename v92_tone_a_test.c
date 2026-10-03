/*
 * v92_tone_a_test.c — Tone A on a V.92 PCM upstream (Cor.1 9.7.1.2).
 */
#include "v92_tone_a.h"
#include "v92_trn2u.h"
#include "v92_upstream_data.h"
#include "v91.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint32_t prng = 0x7A9E2U;
static uint32_t rnd(void) { prng = prng * 1664525U + 1013904223U; return prng >> 8; }

static int16_t g711(int law, int v)
{
    if (v > 32767) v = 32767;
    if (v < -32768) v = -32768;
    return (int16_t)v91_codeword_to_linear((v91_law_t)law, v91_linear_to_codeword((v91_law_t)law, (int16_t)v));
}

/* Feed n samples from gen(); return the sample index of detection, or -1. */
static int run(int law, int n, int (*gen)(int), int echo_db)
{
    v92_tone_a_t t;
    int16_t x[80];

    v92_tone_a_init(&t);
    for (int i = 0; i < n; i += 80) {
        for (int k = 0; k < 80; k++) {
            int v = gen(i + k);
            if (echo_db)
                v += (int)((double)((int)(rnd() % 16000) - 8000) * pow(10.0, -echo_db / 20.0));
            x[k] = g711(law, v);
        }
        if (v92_tone_a_put(&t, x, 80))
            return i + 80;
    }
    return -1;
}

static int onset;
static int tone_len;
static int gen_tone(int i)
{
    if (i < onset || i >= onset + tone_len)
        return (int)(rnd() % 4000) - 2000;            /* a wideband upstream */
    return (int)(5690.0 * cos(2.0 * M_PI * 2400.0 * i / 8000.0));   /* -12 dBm0 */
}

static v92_trn2u_tx_t trn;
static int gen_trn(int i) { int16_t s; (void)i; v92_trn2u_tx_ones_linear(&trn, &s, 1); return s; }

static int gen_pcm(int i) { (void)i; return v91_codeword_to_linear(V91_LAW_ULAW, (uint8_t)(rnd() & 0xFF)); }

static v92_cpd_frame_t cpd;
static v92_upstream_wave_tx_t wtx;
static double wbuf[12];
static int gen_data(int i)
{
    if (i % 12 == 0) {
        uint8_t bits[V92_UPSTREAM_MAX_FRAME_BITS];
        int k = v92_upstream_bits_per_frame(cpd.selected_upstream_drn);
        for (int b = 0; b < k; b++) bits[b] = rnd() & 1;
        assert(v92_upstream_wave_encode_frame(&wtx, &cpd, bits, k, wbuf));
    }
    return (int)lround(wbuf[i % 12]);
}

int main(void)
{
    for (int law = 0; law < 2; law++) {
        int worst = 0;

        tone_len = 8000;
        for (onset = 4000; onset < 4080; onset += 8) {
            int at = run(law, 16000, gen_tone, 0);
            int ms = (at - onset) / 8;
            assert(at > 0 && ms > 50 && ms <= 70);
            if (ms > worst) worst = ms;
        }
        onset = 4000;
        assert(run(law, 16000, gen_tone, 20) > 0);        /* under our own echo */
        tone_len = 320;                                    /* 40 ms: not enough */
        assert(run(law, 16000, gen_tone, 0) < 0);

        for (int pts = 4; pts <= 8; pts += 4) {
            v92_trn2u_tx_init(&trn, pts, 4000.0, law);
            v92_trn2u_tx_start(&trn, 0);
            assert(run(law, 80000, gen_trn, 0) < 0);
        }
        assert(run(law, 80000, gen_pcm, 0) < 0);

        memset(&cpd, 0, sizeof(cpd));
        cpd.modulus_present = cpd.constellations_present = true;
        cpd.selected_upstream_drn = 9;
        cpd.gain_q0_16 = 0xFFFF;
        {
            int k = v92_upstream_bits_per_frame(9), base = k / 12, extra = k % 12;
            for (int i = 0; i < 12; i++) cpd.moduli[i] = (uint8_t)(1U << (base + (i < extra)));
        }
        cpd.set_sizes[0] = 64;
        for (int i = 0; i < 64; i++) cpd.points[0][i] = (uint16_t)(60 * i + 30);
        assert(v92_upstream_wave_profile_validate(&cpd));
        v92_upstream_wave_tx_init(&wtx);
        assert(run(law, 80000, gen_data, 0) < 0);

        printf("PASS: %s Tone A on a V.92 upstream: detected 51-%d ms after onset at 10 offsets, "
               "under a -20 dB echo; not for 40 ms of tone, nor 10 s each of TRN2u (4/8 pt), "
               "random PCM, upstream data\n", law ? "A-law" : "u-law", worst);
    }
    puts("v92_tone_a_test: all passed");
    return 0;
}
