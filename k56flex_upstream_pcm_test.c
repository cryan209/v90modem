/* Offline PCM regression, Draft 0.23 clause 11. Synthesized MICA research
 * TX fixture, not a hardware recording or proof of live K56flex interop. */
#include "k56flex_upstream_pcm.h"
#include <spandsp/telephony.h>
#include <spandsp/bit_operations.h>
#include <spandsp/g711.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define SAMPLES 5120
static unsigned failures;
#define CHECK(c, ...) do { if (!(c)) { ++failures; printf("FAIL: "); \
    printf(__VA_ARGS__); puts(""); } } while (0)

static int load(int baud, int16_t pcm[SAMPLES])
{
    char path[128];
    snprintf(path, sizeof(path), "reference-data/k56flex-upstream/%d.s16le", baud);
    FILE *f = fopen(path, "rb");
    if (!f) { CHECK(0, "cannot open %s", path); return 0; }
    unsigned char bytes[2*SAMPLES];
    size_t got = fread(bytes, 1, sizeof(bytes), f);
    int extra = fgetc(f);
    fclose(f);
    CHECK(got == sizeof(bytes) && extra == EOF, "fixture length %s", path);
    if (got != sizeof(bytes)) return 0;
    for (int i = 0; i < SAMPLES; ++i) {
        unsigned u = bytes[2*i] | ((unsigned)bytes[2*i+1] << 8);
        pcm[i] = (int16_t)(u < 32768 ? (int)u : (int)u - 65536);
    }
    return 1;
}

static void matrix(const int16_t pcm[SAMPLES])
{
    const int delay[] = {0, 1, 7, 37, 159, 317};
    unsigned cases = 0;
    double worst_trn = 0, worst_pp = 0, worst_s = 0;
    for (int law = 0; law < 2; ++law)
    for (int channel = 0; channel < 3; ++channel)
    for (unsigned d = 0; d < sizeof(delay)/sizeof(delay[0]); ++d) {
        int16_t input[SAMPLES+317] = {0};
        unsigned seed = 12345;
        for (int n = 0; n < SAMPLES; ++n) {
            double value = pcm[n];
            if (channel == 1) {
                /* Test-only channel BEFORE the codec: gain, modest ISI,
                 * fractional delay and additive noise. Not a live gain. */
                value = 0.5*(0.7*pcm[n] + (n ? 0.2*pcm[n-1] : 0)
                                      + (n > 1 ? 0.1*pcm[n-2] : 0));
                seed = seed*1664525u + 1013904223u;
                value += 20*((double)(seed >> 16)/32768 - 1);
            } else if (channel == 2) value *= -0.25;
            int sample = (int)lrint(value);
            /* Encode once, then expand the chosen bearer law. No G.711
             * law-to-law transcoding or sample insertion in the decoder. */
            input[n+delay[d]] = law ? alaw_to_linear(linear_to_alaw(sample))
                                   : ulaw_to_linear(linear_to_ulaw(sample));
        }
        k56flex_upstream_pcm_result_t r;
        int events = k56flex_upstream_pcm_decode(input, SAMPLES+delay[d], 18, &r);
        CHECK(events == (K56U_REPORT|K56U_REPORT_END) && r.acquired,
              "law%d channel%d delay%d events%d corr%.3f trn%.3f pp%.3f S%.3f",
              law, channel, delay[d], events, r.correlation, r.trn_rms, r.pp_rms, r.s_rms);
        CHECK(r.ext == 0x63 && r.report == 0x8990, "report %02x/%04x", r.ext, r.report);
        CHECK(r.trn_sample >= 1080+delay[d] && r.trn_sample < 1160+delay[d],
              "TRN observation at %d", r.trn_sample);
        CHECK(r.report_sample > 2400+delay[d] && r.report_sample < 3360+delay[d],
              "report observation at %d", r.report_sample);
        CHECK(r.end_sample > 3360+delay[d] && r.end_sample < 3550+delay[d],
              "terminator observation at %d", r.end_sample);
        worst_trn = fmax(worst_trn, r.trn_rms);
        worst_pp = fmax(worst_pp, r.pp_rms);
        worst_s = fmax(worst_s, r.s_rms);
        ++cases;
    }
    printf("PCM matrix: %u cases, both G.711 laws, delay/gain/polarity/ISI/noise; "
           "worst RMS TRN %.4f PP %.4f S %.4f\n", cases, worst_trn, worst_pp, worst_s);
}

static void negatives(const int16_t pcm[SAMPLES], const int16_t wrong[SAMPLES])
{
    k56flex_upstream_pcm_result_t r;
    int16_t input[SAMPLES] = {0};
    CHECK(!k56flex_upstream_pcm_decode(wrong, SAMPLES, 18, &r) && !r.acquired,
          "2400-baud control accepted as 3200");
    CHECK(!k56flex_upstream_pcm_decode(pcm, SAMPLES, 5, &r) && !r.acquired,
          "wrong tap accepted");
    CHECK(!k56flex_upstream_pcm_decode(input, SAMPLES, 18, &r) && !r.acquired,
          "silence accepted");
    unsigned seed = 54321;
    for (int n = 0; n < SAMPLES; ++n) {
        seed = seed*1664525u + 1013904223u;
        input[n] = (int)(seed >> 16) / 4 - 8192;
    }
    CHECK(!k56flex_upstream_pcm_decode(input, SAMPLES, 18, &r) && !r.acquired,
          "noise accepted");
    for (int n = 0; n < SAMPLES; ++n)
        input[n] = (int16_t)(8000*cos(2*3.14159265358979323846*1828.5714285714*n/8000));
    CHECK(!k56flex_upstream_pcm_decode(input, SAMPLES, 18, &r) && !r.acquired,
          "carrier tone accepted");
    memcpy(input, pcm, sizeof(input));
    memset(input, 0, 1080*sizeof(*input));
    CHECK(!k56flex_upstream_pcm_decode(input, SAMPLES, 18, &r) && !r.acquired,
          "TRN/report without S/S-bar/PP accepted");
    CHECK(!k56flex_upstream_pcm_decode(pcm+2400, SAMPLES-2400, 18, &r) && !r.acquired,
          "report without preamble accepted");
    CHECK(k56flex_upstream_pcm_decode(pcm, 3000, 18, &r) == K56U_REPORT
          && r.acquired && r.report == 0x8990 && r.end_sample == -1,
          "report must be exposed before terminator arrives");
    CHECK(!k56flex_upstream_pcm_decode(pcm, 2440, 18, &r), "truncated report accepted");
    CHECK(!k56flex_upstream_pcm_decode(pcm, 1, 18, &r), "short input accepted");
    CHECK(k56flex_upstream_pcm_decode(NULL, SAMPLES, 18, &r) == -1, "null input");
    CHECK(k56flex_upstream_pcm_decode(pcm, SAMPLES, 0, &r) == -1, "bad tap");
    CHECK(k56flex_upstream_pcm_decode(pcm, -1, 18, &r) == -1, "negative count");
    CHECK(k56flex_upstream_pcm_decode(pcm, 80001, 18, &r) == -1, "oversized count");
    CHECK(k56flex_upstream_pcm_decode(pcm, SAMPLES, 18, NULL) == -1, "null output");
    /* A new call after all the failures must still recover normally. */
    CHECK(k56flex_upstream_pcm_decode(pcm, SAMPLES, 18, &r) == 3
          && r.ext == 0x63 && r.report == 0x8990, "call reset/recovery");
}

int main(void)
{
    int16_t pcm[SAMPLES], wrong[SAMPLES];
    if (load(3200, pcm) && load(2400, wrong)) {
        matrix(pcm);
        negatives(pcm, wrong);
    }
    printf("k56flex_upstream_pcm_test: %u failures\n", failures);
    return failures != 0;
}
