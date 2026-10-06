/*
 * v92_slm_chain_test.c -- our V.92 PCM-upstream receive chain against a
 * model of slmodemd's transmit chain, built from its disassembly.
 *
 * What slmodemd (SmartLink DSP behind d-modem) does to the upstream, read
 * out of tmp/slmodemd-symbolized:
 *   - symbols at 8 kHz as int16: TRN1u +/-LU, TRN2u Table 28 levels x LU
 *     (V92Mapper, LU a literal 4000), data 4000 x G x point truncated toward
 *     zero (V92Transmitter: min-x^2 member, identity prefilter for LZ2=0);
 *   - Resampler 8000 -> 9600, 120-phase polyphase windowed sinc, 16 taps a
 *     phase, cut-off 0.98 of the 8 kHz Nyquist;
 *   - FloatFIR, 35 symmetric taps at 9600 Hz (table at 0x109860, below);
 * and d-modem then interpolates 9600 -> 8000 with a windowed sinc and the
 * bearer is mu-law (its upstream tap equals our received tap to mu-law).
 *
 * The receiver is the engine's: v92_p3_eq trained data-aided on TRN1u,
 * decision-directed on four-level TRN2u, then frozen and slicing on the CPd
 * levels, its output fed to the equalised B1u receiver.  The CPd is the one
 * slmodemd accepted in slm-r9-v92-in-lu (15 points, drn 6, 4G = 23).
 *
 * Rows: an identity channel (the harness itself), the slmodemd chain, and
 * the chain with each of its pieces alone, so a failure names its stage.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v92_p3_eq.h"
#include "v92_upstream_rx.h"

static int failures;

static uint32_t rng = 777;
static uint32_t rnd(void) { rng = rng*1103515245u + 12345u; return rng >> 8; }

/* V92Modulator's FloatFIR: 36 entries at 0x109860, the 36th zero. */
static const double slm_tx_fir[35] = {
     0.00029, -0.00000, -0.00046, -0.00130, -0.00272, -0.00493, -0.00806,
    -0.01218, -0.01727, -0.02318, -0.02971, -0.03653, -0.04328, -0.04954,
    -0.05491, -0.05904, -0.06164,  0.93790, -0.06164, -0.05904, -0.05491,
    -0.04954, -0.04328, -0.03653, -0.02971, -0.02318, -0.01727, -0.01218,
    -0.00806, -0.00493, -0.00272, -0.00130, -0.00046, -0.00000,  0.00029 };

static double sinc(double x) { return fabs(x) < 1e-12 ? 1.0 : sin(M_PI*x)/(M_PI*x); }

/* Rational resampler by up/down through a windowed-sinc low-pass at the
 * up-rate, half-length `half` input samples, cut-off `fc` of the lower
 * Nyquist.  Linear phase; returns the output length. */
static int resample(const double *in, int n, int up, int down, int half,
                    double fc, double *out, int out_max)
{
    int m = 0;
    double ratio = (double)up/down;
    double cutoff = fc*(ratio < 1.0 ? ratio : 1.0);   /* of input Nyquist */

    for (int k = 0; m < out_max; k++, m++) {
        double t = (double)k*down/up;                 /* input-sample time */
        int c = (int)floor(t);
        double acc = 0.0;

        if (c >= n)
            break;
        for (int i = c - half + 1; i <= c + half; i++) {
            double d = t - i;
            double w;

            if (i < 0 || i >= n || fabs(d) >= half)
                continue;
            w = 0.42 + 0.5*cos(M_PI*d/half) + 0.08*cos(2*M_PI*d/half);
            acc += in[i]*cutoff*sinc(cutoff*d)*w;
        }
        out[m] = acc;
    }
    return m;
}

static double mulaw(double x)
{
    /* G.711 mu-law round trip on the 16-bit scale. */
    double s = x < 0 ? -1.0 : 1.0;
    double v = fabs(x);
    int e, mant;

    if (v > 32124.0) v = 32124.0;
    v = floor(v/4.0 + 0.5) + 33.0;
    for (e = 0; e < 7 && v >= (64 << e); e++)
        ;
    mant = (int)(v/(1 << (e + 1))) - 16;
    if (mant > 15) mant = 15;
    if (mant < 0) mant = 0;
    return s*(double)((((2*mant + 33) << e) - 33)*4);
}

static bool k3_half_profile;
static int member_policy = V92_UPSTREAM_MEMBER_MIN_X;

static void make_cpd(v92_cpd_frame_t *cpd)
{
    static const int pts[15] = { 1983, 6079, 10110, 14271, 19212, 23374,
                                 27535, 31696, 39498, 43659, 47820, 51981,
                                 56142, 60303, 64464 };

    memset(cpd, 0, sizeof(*cpd));
    cpd->modulus_present = true;
    cpd->constellations_present = true;
    cpd->selected_upstream_drn = 6;
    cpd->gain_q0_16 = 23;
    int n = k3_half_profile ? 14 : 15;

    for (int i = 0; i < n; i++)
        cpd->points[0][i] = (uint16_t)pts[i];
    cpd->set_sizes[0] = (uint8_t)n;
    for (int i = 0; i < 12; i++)
        cpd->moduli[i] = (uint8_t)((k3_half_profile && i%4 == 3) ? n/2 : n);
    if (k3_half_profile)
        cpd->selected_upstream_drn = 4;      /* 14^9 x 7^3 ~ 2^42.7 */
}

typedef struct { const uint8_t *exp; int nexp; int got; int wrong; } sink_t;
static void put_byte(void *u, uint8_t b)
{
    sink_t *s = u;

    for (int i = 0; i < 8; i++) {
        if (s->got < s->nexp && ((b >> i) & 1) != s->exp[s->got])
            s->wrong++;
        s->got++;
    }
}

enum { CH_IDENTITY, CH_SLM, CH_RESAMPLERS_ONLY, CH_FIR_ONLY };
static const char *ch_name[] = { "identity", "slmodemd chain",
                                 "resamplers only", "tx FIR only" };

static void run(int channel)
{
    enum { LEAD = 400, TRN1U = 4000, TRN2U = 16000, E2U = 12,
           DATA_FRAMES = 300 };
    const double lu = 4000.0;
    v92_cpd_frame_t cpd;
    v92_upstream_wave_tx_t tx;
    v92_upstream_rx_t *rx = calloc(1, sizeof(*rx));
    v92_p3_eq_t *eq = calloc(1, sizeof(*eq));
    v92_p3_eq_config_t cfg;
    int k, nsym, n96, nout, b1u_at;
    double *sym, *s96, *f96, *ds0;
    int8_t *ref;
    uint8_t *bits, ones[V92_UPSTREAM_MAX_FRAME_BITS];
    sink_t sink = { 0 };
    double g;
    double levels[15];
    int nlev;
    double e2 = 0.0, d2 = 0.0;
    int ne = 0, nd = 0;
    double tx_rms = 0.0, rx_rms = 0.0;

    make_cpd(&cpd);
    g = cpd.gain_q0_16/(4.0*65536.0);
    k = v92_upstream_bits_per_frame(cpd.selected_upstream_drn);
    nsym = LEAD + TRN1U + TRN2U + E2U + (V92_B1U_FRAMES + DATA_FRAMES)*12;
    sym = calloc((size_t)nsym, sizeof(double));
    ref = calloc(TRN1U, 1);
    bits = calloc((size_t)DATA_FRAMES*k, 1);
    memset(ones, 1, sizeof(ones));

    /* slmodemd's symbols, int16 as V92Transmitter/V92Mapper leave them. */
    v92_p3_eq_reference(ref, TRN1U);
    int p = LEAD;
    for (int i = 0; i < TRN1U; i++)
        sym[p++] = ref[i]*lu;
    for (int i = 0; i < TRN2U + E2U; i++) {
        static const int lv[4] = { 1, 3, -1, -3 };
        sym[p++] = trunc(lv[rnd() & 3]*lu/sqrt(5.0));
    }
    b1u_at = p;
    v92_upstream_wave_tx_init(&tx);
    tx.class_member = member_policy;
    for (int f = 0; f < V92_B1U_FRAMES + DATA_FRAMES; f++) {
        double v[12];
        const uint8_t *src = ones;

        if (f >= V92_B1U_FRAMES) {
            for (int i = 0; i < k; i++)
                bits[(f - V92_B1U_FRAMES)*k + i] = (uint8_t)(rnd() & 1);
            src = &bits[(f - V92_B1U_FRAMES)*k];
        }
        if (!v92_upstream_wave_encode_frame(&tx, &cpd, src, k, v)) {
            printf("FAIL encode\n");
            failures++;
            return;
        }
        for (int i = 0; i < 12; i++)
            sym[p++] = trunc(lu*v[i]);       /* 4000 x G x point, fistp */
    }

    if (channel == CH_SLM) {
        double a = 0, b = 0;
        int na = 0, nb = 0;

        for (int i = LEAD + TRN1U; i < LEAD + TRN1U + TRN2U; i++) { a += sym[i]*sym[i]; na++; }
        for (int i = b1u_at + 576; i < nsym; i++) { b += sym[i]*sym[i]; nb++; }
        printf("symbol-domain data/TRN2u rms ratio %.3f\n", sqrt((b/nb)/(a/na)));
    }
    /* The chain. */
    s96 = calloc((size_t)nsym*2, sizeof(double));
    f96 = calloc((size_t)nsym*2, sizeof(double));
    ds0 = calloc((size_t)nsym*2, sizeof(double));
    if (channel == CH_IDENTITY || channel == CH_FIR_ONLY) {
        memcpy(s96, sym, (size_t)nsym*sizeof(double));
        n96 = nsym;
    } else {
        n96 = resample(sym, nsym, 6, 5, 8, 0.98, s96, nsym*2);
    }
    if (channel == CH_SLM || channel == CH_FIR_ONLY) {
        for (int i = 0; i < n96; i++) {
            double acc = 0.0;

            for (int t = 0; t < 35; t++)
                if (i - t + 17 >= 0 && i - t + 17 < n96)
                    acc += slm_tx_fir[t]*s96[i - t + 17];
            f96[i] = acc;
        }
    } else {
        memcpy(f96, s96, (size_t)n96*sizeof(double));
    }
    if (channel == CH_SLM && getenv("SLM_DUMP96")) {
        FILE *f = fopen(getenv("SLM_DUMP96"), "wb");
        int from = (int)((b1u_at)*1.2);

        if (f) { fwrite(f96 + from, sizeof(double), (size_t)(n96 - from), f); fclose(f); }
    }
    if (channel == CH_IDENTITY || channel == CH_FIR_ONLY) {
        memcpy(ds0, f96, (size_t)n96*sizeof(double));
        nout = n96;
    } else {
        nout = resample(f96, n96, 5, 6, 16, 0.95, ds0, nsym*2);
    }
    /* Received TRN2u was ~1400 rms on the r9 call; scale to that before
     * mu-law, which is what makes the quantisation realistic. */
    for (int i = LEAD + TRN1U + 1000; i < LEAD + TRN1U + TRN2U - 1000; i++) {
        tx_rms += sym[i]*sym[i];
        rx_rms += ds0[i]*ds0[i];
    }
    double scale = 1403.0/sqrt(rx_rms/(TRN2U - 2000));
    for (int i = 0; i < nout; i++)
        ds0[i] = mulaw(ds0[i]*scale);

    {
        double a = 0, b = 0;
        int na = 0, nb = 0;

        for (int i = LEAD + TRN1U + 500; i < LEAD + TRN1U + TRN2U - 500; i++) { a += ds0[i]*ds0[i]; na++; }
        for (int i = b1u_at + 800; i < nout - 100; i++) { b += ds0[i]*ds0[i]; nb++; }
        printf("%-16s received data/TRN2u rms ratio %.3f\n", ch_name[channel], sqrt((b/nb)/(a/na)));
    }
    /* The receiver, as the engine runs it. */
    sink.exp = bits;
    sink.nexp = DATA_FRAMES*k;
    v92_upstream_b1_rx_init_equalized(rx, &cpd, put_byte, &sink);
    v92_p3_eq_default_config(&cfg);
    cfg.ntaps = 63;                          /* as the engine asks */
    if (getenv("EQ_TAPS")) cfg.ntaps = atoi(getenv("EQ_TAPS"));
    if (getenv("EQ_FB")) cfg.nfb = atoi(getenv("EQ_FB"));
    v92_p3_eq_init(eq, &cfg);
    nlev = cpd.set_sizes[0];
    for (int i = 0; i < nlev; i++)
        levels[i] = g*cpd.points[0][i];      /* LU units: TRN2u rms = LU */
    bool started = false, pam4 = false, frozen = false;
    for (int i = 0; i < nout; i++) {
        v92_p3_eq_push(eq, ds0[i]);
        /* TRN1u's first symbol is at LEAD; the channel delay is absorbed by
         * the seed least-squares fit, as the engine relies on. */
        if (!started && i >= LEAD + 200)
            started = v92_p3_eq_start(eq, LEAD);
        while (started && v92_p3_eq_step(eq)) {
            int64_t sk = eq->k - 1;          /* symbol just produced */

            if (!pam4 && sk >= TRN1U) {
                v92_p3_eq_set_pam4(eq, true);
                pam4 = true;
            }
            if (pam4 && !frozen && sk >= TRN1U + 2000 && sk < TRN1U + TRN2U - 200) {
                double y = eq->y*sqrt(5.0);
                double d = 2.0*floor(y/2.0) + 1.0;

                if (d > 3) d = 3;
                if (d < -3) d = -3;
                e2 += (y - d)*(y - d)/5.0;
                ne++;
            }
            if (!frozen && sk >= TRN1U + TRN2U - 60) {
                v92_p3_eq_hold(eq, true);
                v92_p3_eq_set_levels(eq, levels, nlev);
                frozen = true;
            }
            if (frozen) {
                double best = 1e9;

                for (int j = 0; j < nlev; j++) {
                    if (fabs(eq->y - levels[j]) < fabs(best)) best = eq->y - levels[j];
                    if (fabs(eq->y + levels[j]) < fabs(best)) best = eq->y + levels[j];
                }
                if (sk >= TRN1U + TRN2U + 100 && sk < TRN1U + TRN2U + 4000) {
                    d2 += best*best;
                    nd++;
                }
                v92_upstream_b1_rx_feed_values(rx, &eq->y, 1);
            }
        }
    }
    int ok = rx->locked && sink.got >= (DATA_FRAMES - 4)*k && sink.wrong == 0;
    printf("%-16s data error %.3f LU (closest spacing 0.36); ", ch_name[channel], nd ? sqrt(d2/nd) : -1.0);
    printf("TRN2u error %.3f LU; B1u %s (%llu alignments, best frame 0 "
           "%d/%d ones, best %d frames); %d data bits, %d wrong\n",
           ne ? sqrt(e2/ne) : -1.0,
           rx->locked ? "LOCKED" : "no lock",
           (unsigned long long)rx->candidates_started, rx->best_first_ones, k,
           rx->best_frames, sink.got, sink.wrong);
    if (channel == CH_IDENTITY && !ok) {
        printf("FAIL: the harness itself does not carry B1u\n");
        failures++;
    }
    (void)b1u_at;
    (void)tx_rms;
    free(sym); free(ref); free(bits); free(s96); free(f96); free(ds0);
    free(rx); free(eq);
}

int main(void)
{
    /* Our min-|x| transmitter, the profile slmodemd accepted in r9. */
    run(CH_IDENTITY);
    run(CH_SLM);
    run(CH_RESAMPLERS_ONLY);
    run(CH_FIR_ONLY);
    /* slmodemd's own precoder (V92_UPSTREAM_MEMBER_SLMODEMD) through its
     * chain: the r9 profile, then LC even with Mi = LC/2 at k = 3. */
    member_policy = V92_UPSTREAM_MEMBER_SLMODEMD;
    printf("--- slmodemd precoder, r9 profile (Mi = LC = 15)\n");
    run(CH_SLM);
    k3_half_profile = true;
    printf("--- slmodemd precoder, LC 14, Mi = 7 at k = 3\n");
    run(CH_IDENTITY);
    run(CH_SLM);
    return failures ? 1 : 0;
}
