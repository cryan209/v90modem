/*
 * v34_pp_fit_test.c — the PP equaliser fit, and a mode that runs it over a
 * recorded G.711 tap so the same code can be pointed at a live call.
 *
 *   ./v34_pp_fit_test                       run the tests
 *   ./v34_pp_fit_test <tap.g711> <t> [baud] [fc]   fit at t seconds
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "v34_pp_fit.h"
#include "v90_analogue_tx.h"

static int failures;

static void check(bool ok, const char *what)
{
    printf("%s: %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok)
        failures++;
}

static uint32_t rng = 987654321u;

static double urand(void)
{
    rng = rng*1103515245u + 12345u;
    return ((double) ((rng >> 8) & 0xFFFFFF))/16777216.0 - 0.5;
}

/* A raised-cosine pulse, rolloff 0.25, truncated at +/-4 symbols. */
static double pulse(double t)
{
    double a = 0.25, s, c;

    if (fabs(t) > 4.0)
        return 0.0;
    s = (fabs(t) < 1e-9) ? 1.0 : sin(M_PI*t)/(M_PI*t);
    c = 1.0 - (2.0*a*t)*(2.0*a*t);
    if (fabs(c) < 1e-9)
        return s*M_PI/4.0;
    return s*cos(M_PI*a*t)/c;
}

/*
 * Synthesise §10.1.3.6's PP as a real passband waveform, through a dispersive
 * channel, at an arbitrary sampling phase.  `what` selects the reference (0) or
 * §10.1.3.8's TRN, which is the signal that follows PP and must NOT fit (1).
 */
static void synth(int16_t *amp, int n, double fs, double baud, double fc,
                  double phase, double gain, double snr_db, int what)
{
    static const double ch[5] = { 0.06, 0.20, 0.78, 0.20, 0.06 };
    double *raw = calloc((size_t) n + 64, sizeof(double));
    int i, k, nsym = (int) (n/(fs/baud)) + 8;

    for (i = 0; i < n; i++) {
        double t = (i - phase)/fs;
        double acc = 0.0;

        for (k = 0; k < nsym; k++) {
            double tk = t*baud - k;
            double p = pulse(tk);
            double sr, si;

            if (p == 0.0)
                continue;
            if (what == 0) {
                v34_pp_reference(k%V34_PP_SYMBOLS, &sr, &si);
            } else {
                /* TRN: four constellation points from scrambled bits. */
                unsigned r = ((unsigned) k*2654435761u);
                double ang = M_PI_4 + (M_PI/2.0)*((r >> 13)&3);

                sr = cos(ang);
                si = sin(ang);
            }
            {
                double ph = 2.0*M_PI*fc*t;

                acc += p*(sr*cos(ph) - si*sin(ph));
            }
        }
        raw[i] = acc*gain;
    }
    for (i = 0; i < n; i++) {
        double acc = 0.0;
        int j;

        for (j = 0; j < 5; j++)
            acc += ch[j]*((i - j >= 0) ? raw[i - j] : 0.0);
        if (snr_db > 0.0)
            acc += gain*pow(10.0, -snr_db/20.0)*urand()/0.29;
        amp[i] = (int16_t) ((acc > 32000.0) ? 32000.0
                            : (acc < -32000.0) ? -32000.0 : acc);
    }
    free(raw);
}

static void test_reference_matches_the_table(void)
{
    /* The equation against the table v90_analogue_tx.c transmits from: two
     * independent expressions of §10.1.3.6, and if they disagree the fit is
     * training on something the transmitter does not send. */
    double worst = 0.0;
    int i;

    for (i = 0; i < V34_PP_PERIOD; i++) {
        float tre, tim;
        double re, im, d;

        v90_analogue_tx_pp_symbol(i, &tre, &tim);
        v34_pp_reference(i, &re, &im);
        d = hypot(re - tre, im - tim);
        if (d > worst)
            worst = d;
    }
    printf("      equation vs transmitter table: worst |diff| %.2e\n", worst);
    check(worst < 1e-6, "equation 10-1 matches the transmitter's PP table");
}

static void test_fit(double fs, double baud, double fc, double phase,
                     double gain, double snr_db)
{
    int n = (int) (fs*V34_PP_SYMBOLS/baud) + 128;
    int16_t *amp = calloc((size_t) n, sizeof(int16_t));
    v34_pp_fit_t f;
    char msg[160];
    bool ok;

    synth(amp, n, fs, baud, fc, phase, gain, snr_db, 0);
    ok = v34_pp_fit(amp, n, fs, baud, fc, 32, &f);
    snprintf(msg, sizeof msg,
             "PP fits at fs=%.0f baud=%.0f phase=%.2f gain=%.0f SNR=%.0f dB "
             "(score %.3f, %d branches, %d symbols)",
             fs, baud, phase, gain, snr_db, f.score, f.branches, f.symbols);
    check(ok, msg);
    free(amp);
}

static void test_rejects(void)
{
    double fs = 8000.0, baud = 3200.0, fc = 1828.571;
    int n = (int) (fs*V34_PP_SYMBOLS/baud) + 128;
    int16_t *amp = calloc((size_t) n, sizeof(int16_t));
    v34_pp_fit_t f;
    char msg[128];

    synth(amp, n, fs, baud, fc, 0.0, 3000.0, 0.0, 1);
    check(!v34_pp_fit(amp, n, fs, baud, fc, 32, &f),
          "§10.1.3.8's TRN does not fit PP");
    snprintf(msg, sizeof msg, "      TRN score %.3f", f.score);
    puts(msg);

    for (int i = 0; i < n; i++)
        amp[i] = (int16_t) (6000.0*urand());
    check(!v34_pp_fit(amp, n, fs, baud, fc, 32, &f), "noise does not fit PP");
    snprintf(msg, sizeof msg, "      noise score %.3f", f.score);
    puts(msg);

    memset(amp, 0, (size_t) n*sizeof(int16_t));
    check(!v34_pp_fit(amp, n, fs, baud, fc, 32, &f), "silence does not fit PP");
    free(amp);
}

static int16_t ulaw_decode_local(uint8_t u)
{
    int t;

    u = ~u;
    t = ((u & 0x0F) << 3) + 0x84;
    t <<= (u & 0x70) >> 4;
    return (int16_t) ((u & 0x80) ? (0x84 - t) : (t - 0x84));
}

static int analyse(const char *path, double at, double baud, double fc, int syms)
{
    FILE *fp = fopen(path, "rb");
    long len;
    uint8_t *raw;
    int16_t *amp;
    double fs = 8000.0;
    int n, i, start;
    v34_pp_fit_t f;
    bool ok;

    if (!fp) {
        perror(path);
        return 1;
    }
    fseek(fp, 0, SEEK_END);
    len = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    raw = malloc((size_t) len);
    if (fread(raw, 1, (size_t) len, fp) != (size_t) len) {
        fclose(fp);
        free(raw);
        return 1;
    }
    fclose(fp);
    n = (int) (fs*syms/baud) + 128;
    start = (int) (at*fs);
    if (start < 0 || start + n > len) {
        fprintf(stderr, "window outside the file\n");
        free(raw);
        return 1;
    }
    amp = malloc((size_t) n*sizeof(int16_t));
    for (i = 0; i < n; i++)
        amp[i] = ulaw_decode_local(raw[start + i]);
    ok = v34_pp_fit(amp, n, fs, baud, fc, 32, &f);
    printf("%s at %.4f s, %.0f baud, carrier %.3f Hz: %s "
           "(held-out score %.3f, %d branches, %d symbols)\n",
           path, at, baud, fc, ok ? "PP FITS" : "no fit", f.score,
           f.branches, f.symbols);
    free(amp);
    free(raw);
    return ok ? 0 : 2;
}

int main(int argc, char **argv)
{
    if (argc >= 3) {
        double baud = (argc > 3) ? atof(argv[3]) : 3200.0;
        double fc   = (argc > 4) ? atof(argv[4]) : 1828.571;

        int syms = (argc > 5) ? atoi(argv[5]) : V34_PP_SYMBOLS;

        return analyse(argv[1], atof(argv[2]), baud, fc, syms);
    }
    test_reference_matches_the_table();
    test_fit(8000.0, 3200.0, 1828.571, 0.00, 3000.0, 0.0);
    test_fit(8000.0, 3200.0, 1828.571, 0.25, 3000.0, 0.0);
    test_fit(8000.0, 3200.0, 1828.571, 0.50, 3000.0, 0.0);
    test_fit(8000.0, 3200.0, 1828.571, 0.75, 3000.0, 0.0);
    test_fit(8000.0, 3200.0, 1920.000, 0.30, 3000.0, 0.0);
    test_fit(16000.0, 3200.0, 1828.571, 0.40, 3000.0, 0.0);
    test_fit(8000.0, 2400.0, 1800.000, 0.20, 3000.0, 0.0);
    test_fit(8000.0, 3200.0, 1828.571, 0.35, 300.0, 0.0);
    test_fit(8000.0, 3200.0, 1828.571, 0.35, 3000.0, 25.0);
    test_fit(8000.0, 3200.0, 1828.571, 0.35, 3000.0, 18.0);
    test_rejects();

    if (failures) {
        printf("v34_pp_fit_test: %d FAILURES\n", failures);
        return 1;
    }
    printf("v34_pp_fit_test: OK\n");
    return 0;
}
