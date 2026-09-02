/*
 * v90_analogue_rx_test.c — does the analogue Phase 3 receiver find the four
 * moments §9.3.2 turns on, in a downstream we did not generate?
 *
 * The fixtures in artifacts/eicon-digital-downstream/ are the point of this
 * test.  They are transmit-side DS0 captures of the Eicon card's own V.90
 * firmware on calls a USR Courier answered with CONNECT — the only downstream
 * in this tree that our own transmitter did not produce.  A convention error
 * that is wrong but self-consistent survives every loopback test by
 * construction; it does not survive these.
 *
 * The expected values are the ones tools/eicon_rx_conformance.py pins, derived
 * from the captures without this decoder (µ-law Ucode decomposition plus the
 * §5.3 GPC generator): Sd 64 reps and S̄d 8 reps.  They are exact, not
 * thresholds.
 *
 * TRN1d is 30000T here, where that script says 30005T, and the two are
 * measuring different things.  30005 is where the stream stops descrambling
 * to ones; 30000 is where Jd starts.  They differ because §8.4.2 switches the
 * mapping to differential at the frame boundary and TRN1d's non-differential
 * reading survives five more bits of Jd's sync ones by coincidence.  30000 is
 * the one that can be checked rather than estimated: a Table 13 frame decodes
 * there with zero structural errors and a zero CRC remainder, and at no other
 * offset within 10 symbols either side, on both fixtures.  It is also 3750 ms
 * exactly, the figure docs/eicon_downstream_comparison.md gives for what the
 * card transmits.
 *
 * DIL is deliberately not exercised against the fixtures.  §9.3.2.9 measures
 * the sequence *this* side requested in Ja, and the descriptor in those calls
 * is the Courier's, which we do not have.  Measuring it is the separate job
 * v90_dil_rx.c does, and its open defect (docs/eicon_downstream_comparison.md,
 * Finding 5) is not this module's.  The DIL path is checked instead against a
 * stream generated from a descriptor we chose, which tests the plumbing and
 * the §9.3.2.10 stopping rule and claims nothing more.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <spandsp.h>

#include "v90.h"
#include "v90_analogue_phase3.h"
#include "v90_analogue_rx.h"
#include "v90_analogue_linear.h"
#include "v90_analogue_fse.h"
#include "v90_dil_presets.h"

static int failures;

#define CHECK(cond, fmt, ...)                                           \
    do {                                                                \
        if (!(cond)) {                                                  \
            printf("  FAIL: " fmt "\n", ##__VA_ARGS__);                 \
            failures++;                                                 \
        }                                                               \
    } while (0)

typedef struct {
    const char *path;
    int         u_info;
    int         sd_reps;
    int         sd_bar_reps;
    int         trn1d_symbols;
} fixture_t;

/* From tools/eicon_rx_conformance.py, which derived them independently. */
static const fixture_t fixtures[] = {
    {"artifacts/eicon-digital-downstream/call1-connect-32000.ulaw", 48, 64, 8, 30000},
    {"artifacts/eicon-digital-downstream/call3-connect-42666.ulaw", 48, 64, 8, 30000},
};

static uint8_t *read_file(const char *path, long *len_out)
{
    uint8_t *buf;
    FILE *f;
    long len;

    if ((f = fopen(path, "rb")) == NULL)
        return NULL;
    fseek(f, 0, SEEK_END);
    len = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (len <= 0  ||  (buf = malloc((size_t) len)) == NULL) {
        fclose(f);
        return NULL;
    }
    if (fread(buf, 1, (size_t) len, f) != (size_t) len) {
        free(buf);
        fclose(f);
        return NULL;
    }
    fclose(f);
    *len_out = len;
    return buf;
}

static void test_fixture(const fixture_t *fx)
{
    v90_analogue_rx_config_t cfg;
    v90_analogue_rx_t *rx;
    uint8_t *data;
    unsigned events;
    long len;

    printf("§9.3.2 receive against %s\n", fx->path);

    if ((data = read_file(fx->path, &len)) == NULL) {
        printf("  SKIP: fixture not present\n");
        return;
    }

    memset(&cfg, 0, sizeof(cfg));
    cfg.law = V90_LAW_ULAW;
    cfg.u_info = fx->u_info;
    /* No descriptor: this fixture's DIL was requested by the Courier, not by
     * us, so there is nothing here to measure against. */
    rx = v90_analogue_rx_init(&cfg);
    if (rx == NULL) {
        printf("  FAIL: receiver did not initialise\n");
        failures++;
        free(data);
        return;
    }

    /* Feed it the way a call would: 20 ms of RTP at a time. */
    events = 0;
    for (long off = 0; off < len; off += 160) {
        int n = (int) ((len - off < 160) ? (len - off) : 160);

        events |= v90_analogue_rx_put(rx, data + off, n);
    }

    CHECK((events & V90A_RX_EVENT_SD) != 0, "Sd never acquired");
    CHECK((events & V90A_RX_EVENT_SD_BAR) != 0,
          "§9.3.2.4's Sd-to-S̄d transition never seen — Ja would never end");
    CHECK((events & V90A_RX_EVENT_TRN1D) != 0, "TRN1d never started");
    CHECK((events & V90A_RX_EVENT_JD) != 0, "no Jd frame passed structure and CRC");

    CHECK(v90_analogue_rx_sd_reps(rx) == fx->sd_reps,
          "Sd %d reps, expected %d",
          v90_analogue_rx_sd_reps(rx), fx->sd_reps);
    CHECK(v90_analogue_rx_sd_bar_reps(rx) == fx->sd_bar_reps,
          "S̄d %d reps, expected %d",
          v90_analogue_rx_sd_bar_reps(rx), fx->sd_bar_reps);
    CHECK(v90_analogue_rx_trn1d_symbols(rx) == fx->trn1d_symbols,
          "TRN1d %d symbols, expected %d",
          v90_analogue_rx_trn1d_symbols(rx), fx->trn1d_symbols);

    printf("  Sd@%lld %d reps, S̄d@%lld %d reps, TRN1d@%lld %dT, "
           "Jd@%lld %d frames (%s-point), J'd=%s\n",
           (long long) v90_analogue_rx_sd_start(rx), v90_analogue_rx_sd_reps(rx),
           (long long) v90_analogue_rx_sd_bar_start(rx), v90_analogue_rx_sd_bar_reps(rx),
           (long long) v90_analogue_rx_trn1d_start(rx), v90_analogue_rx_trn1d_symbols(rx),
           (long long) v90_analogue_rx_jd_start(rx), v90_analogue_rx_jd_frames(rx),
           v90_analogue_rx_jd_trn16(rx) ? "16" : "4",
           (events & V90A_RX_EVENT_JD_PRIME) ? "yes" : "no");

    v90_analogue_rx_free(rx);
    free(data);
}

/*
 * The same fixture over an ANALOGUE bearer: levels, not codewords.
 *
 * The digital-bearer tests above hand the receiver the exact octets.  A
 * two-wire line does not: what arrives is the level the far D/A produced,
 * scaled by the line's loss and sitting on the codec's DC offset.  Nothing in
 * §8.4 changes -- one G.711 level per DS0 interval either way -- so the same
 * receiver must acquire Sd, S̄d, TRN1d and Jd once v90_analogue_linear.c has
 * sliced the levels back into Ucodes.
 *
 * The attenuation is deliberately not a round number and the DC offset is the
 * ~900 counts the HSF codec actually adds (docs/hsf_analogue_v90_coupler.md),
 * so a slicer that quietly assumed unity gain or a zero mean fails here.
 */
static void test_linear_bearer(const fixture_t *fx)
{
    v90_analogue_rx_config_t cfg;
    v90_analogue_rx_t *rx;
    v90a_linear_t *lin;
    uint8_t *data;
    unsigned events = 0;
    long len;

    printf("§9.3.2 receive over an analogue bearer (levels) against %s\n", fx->path);

    if ((data = read_file(fx->path, &len)) == NULL) {
        printf("  SKIP: fixture not present\n");
        return;
    }

    memset(&cfg, 0, sizeof(cfg));
    cfg.law = V90_LAW_ULAW;
    cfg.u_info = fx->u_info;
    rx = v90_analogue_rx_init(&cfg);
    lin = v90a_linear_init(V90_LAW_ULAW);
    if (rx == NULL  ||  lin == NULL) {
        printf("  FAIL: receiver did not initialise\n");
        failures++;
        v90_analogue_rx_free(rx);
        v90a_linear_free(lin);
        free(data);
        return;
    }

    for (long off = 0; off < len; off += 160) {
        int n = (int) ((len - off < 160) ? (len - off) : 160);
        int16_t amp[160];
        uint8_t sliced[480];
        int got;

        for (int i = 0; i < n; i++)
            amp[i] = (int16_t) (ulaw_to_linear(data[off + i])*0.37 + 900.0);
        got = v90a_linear_put(lin, amp, n, sliced, (int) sizeof(sliced));
        if (got > 0)
            events |= v90_analogue_rx_put(rx, sliced, got);
        if (!v90a_linear_locked(lin)
            &&  v90_analogue_rx_stage(rx) != V90A_RX_HUNT_SD)
            v90a_linear_lock(lin);
    }

    CHECK((events & V90A_RX_EVENT_SD) != 0, "Sd never acquired");
    CHECK((events & V90A_RX_EVENT_SD_BAR) != 0,
          "§9.3.2.4's Sd-to-S̄d transition never seen — Ja would never end");
    CHECK((events & V90A_RX_EVENT_TRN1D) != 0, "TRN1d never started");
    CHECK((events & V90A_RX_EVENT_JD) != 0, "no Jd frame passed structure and CRC");
    /* The level ladder is scaled, so the Ucode indices are not the digital
     * bearer's; the *structure* is what has to survive.  A whole TRN1d within
     * a few per cent of the 30000T the digital path reads says the S̄d→TRN1d
     * boundary was found on the level ratio, not on a lucky codeword match. */
    CHECK(v90_analogue_rx_trn1d_symbols(rx) > fx->trn1d_symbols*95/100,
          "TRN1d %d symbols, expected about %d",
          v90_analogue_rx_trn1d_symbols(rx), fx->trn1d_symbols);

    printf("  gain %.4f from line level %.0f: Sd %d reps at W=%d, S̄d %d reps, "
           "TRN1d %dT, Jd %d frames\n",
           v90a_linear_gain(lin), v90a_linear_level(lin),
           v90_analogue_rx_sd_reps(rx), v90_analogue_rx_w(rx),
           v90_analogue_rx_sd_bar_reps(rx),
           v90_analogue_rx_trn1d_symbols(rx),
           v90_analogue_rx_jd_frames(rx));

    v90_analogue_rx_free(rx);
    v90a_linear_free(lin);
    free(data);
}

/*
 * The T/2 fractionally-spaced equaliser, over a band-limited analogue channel.
 *
 * Channel model, and it is the one the coupler faces: the far codec holds each
 * G.711 level for a whole DS0 interval, the line and the two codecs band-limit
 * the result, and we sample at 16 kHz on a clock sharing no phase
 * with the far end's.  So y(t) = sum_k a_k p(t - k) with p the zero-order hold
 * through that low-pass, sampled at t = n/2 + phi.
 *
 * The signal is §8.4.5's TRN1d -- scrambled ones on one Ucode, so ±U, constant
 * modulus, signs unknown -- which is what CMA needs and what the analogue modem
 * is given 30000T of.  What is asserted is the thing that matters and the thing
 * the raw slicer could not do at any sampling phase: that the SIGNS come back.
 * They are the scrambler's output, so recovering them is recovering TRN1d, and
 * §8.4.2's Jd rides on the same mapping.
 *
 * phi is swept over a whole symbol, which is both half-symbol positions of the
 * 16 kHz grid: an equaliser that only worked at one of them would be a
 * symbol-spaced equaliser wearing a disguise.
 */
#define FSE_SPAN     8
#define FSE_GRID     512
/*
 * The channel, MEASURED rather than modelled.
 *
 * The far end transmits V.34's line probing signal in Phase 2 -- 21 cosines of
 * equal amplitude from 150 Hz to 3750 Hz -- so a recorded call contains a
 * complete channel sounding, and tools/hsf_probe_response.py pulls it out.  The
 * table below is what the real HSF path does, taken off
 * artifacts/hsf-v90/call-c1 and identical to 0.1 dB in the nine recorded calls
 * that got far enough to carry a probe.  It covers everything between the two
 * modems: the far end's G.711 transmit and its D/A reconstruction, the SIP leg,
 * the ATA, the two-wire line and the HSF codec's own receive filter.
 *
 * It is much kinder than the brickwall this test used to guess at, and the
 * difference decides things.  Flat within 1.5 dB to 3000 Hz, -2.5 dB at 3450,
 * -4.4 at 3600 and -8.4 at 3750 -- so there IS usable energy right up against
 * the 4 kHz Nyquist of the 8 kHz symbol rate, where the guessed 3600 Hz
 * low-pass had none at all.  The phase is smooth and its residual after the
 * bulk delay is removed reaches -72 degrees at 3750, which is ordinary
 * band-edge group delay and exactly what a fractionally-spaced equaliser is
 * for.
 *
 * Two caveats belong with it.  The probe stops at 3750, so what the path does
 * in the last 250 Hz is not measured -- it is taken as nothing here, which is
 * the pessimistic reading.  And 900, 1200, 1800 and 2400 Hz are gaps in the
 * probe itself (they are its noise references), so those are interpolated from
 * their neighbours, which the smoothness of everything around them justifies.
 */

/* Measured on artifacts/hsf-v90/call-c1/hsf-rx.raw, probe at 16.440 s. */
static const struct { int freq; double db; double phase; }
    hsf_measured_response[] = {
        { 150,   -0.38,  -0.0168},
        { 300,   -0.13,  -0.3326},
        { 450,   -0.05,  -0.3526},
        { 600,    0.00,  -0.3144},
        { 750,   -0.05,  -0.2560},
        {1050,   -0.20,  -0.1053},
        {1350,   -0.41,  +0.0516},
        {1500,   -0.37,  +0.1283},
        {1650,   -0.36,  +0.2004},
        {1950,   -0.54,  +0.3263},
        {2100,   -0.64,  +0.3795},
        {2250,   -0.69,  +0.4231},
        {2550,   -0.90,  +0.4700},
        {2700,   -1.19,  +0.4657},
        {2850,   -1.27,  +0.4377},
        {3000,   -1.45,  +0.3708},
        {3150,   -1.66,  +0.2559},
        {3300,   -1.83,  +0.0660},
        {3450,   -2.51,  -0.2383},
        {3600,   -4.36,  -0.6970},
        {3750,   -8.42,  -1.2625},
    };

#define FSE_LOSS     0.37

/*
 * The channel's response to one DS0 sample, from the measurement above.
 *
 * The probe was generated by the far end AS 8 kHz PCM samples, so what the
 * table holds is already the transfer function from a sample sequence to our
 * 16 kHz stream -- the reconstruction filter, the zero-order hold and
 * everything else are inside it, and modelling any of them again would be
 * counting them twice.  t is in symbols.
 */
static double fse_pulse(double t)
{
    static double tab[2*FSE_SPAN*FSE_GRID + 1];
    static bool built = false;

    if (!built) {
        int n = (int) (sizeof(hsf_measured_response)
                       /sizeof(hsf_measured_response[0]));
        double peak = 0.0;

        for (int i = 0; i <= 2*FSE_SPAN*FSE_GRID; i++) {
            double u = (double) i/FSE_GRID - FSE_SPAN;
            double sum = 0.0;

            for (int k = 0; k < n; k++) {
                double a = pow(10.0, hsf_measured_response[k].db/20.0);

                sum += a*cos(2.0*M_PI*hsf_measured_response[k].freq*u/8000.0
                             + hsf_measured_response[k].phase);
            }
            /*
             * Windowed to the span this pulse is used over.  Twenty-one
             * samples of a spectrum reconstruct a response that repeats every
             * 1/150 s -- 53 symbols -- and those repeats are an artefact of
             * having sampled the spectrum, not something the line does.
             */
            sum *= 0.5*(1.0 + cos(M_PI*u/FSE_SPAN));
            tab[i] = sum;
            if (fabs(sum) > peak)
                peak = fabs(sum);
        }
        if (peak > 0.0) {
            for (int i = 0; i <= 2*FSE_SPAN*FSE_GRID; i++)
                tab[i] /= peak;
        }
        /*endif*/
        built = true;
    }
    /*endif*/
    if (fabs(t) >= FSE_SPAN)
        return 0.0;
    {
        double f = (t + FSE_SPAN)*FSE_GRID;
        int i = (int) f;

        f -= i;
        return tab[i]*(1.0 - f) + tab[i + 1]*f;
    }
}

/*
 * Run TRN1d through the channel at sampling phase phi and report how much of
 * the sign stream the equaliser recovers, once converged.
 */
static bool fse_run(const char *path, long first, long count, double phi,
                    double *match_out, double *disp_out, double *centre_out)
{
    v90a_fse_t *fse;
    uint8_t *data;
    long len;
    double *y;
    int8_t *tx;
    int ny = 0;
    int best_match = -1;
    int best_delay = 0;
    long settled;

    if ((data = read_file(path, &len)) == NULL)
        return false;
    if (first + count + FSE_SPAN >= len) {
        free(data);
        return false;
    }
    fse = v90a_fse_init(32, 0.0);
    y = malloc(sizeof(double)*(size_t) (count + 2));
    tx = malloc((size_t) count);
    if (fse == NULL  ||  y == NULL  ||  tx == NULL) {
        free(fse); free(y); free(tx); free(data);
        return false;
    }
    v90a_fse_set_mode(fse, V90A_FSE_CMA);

    for (long k = 0; k < count; k++)
        tx[k] = (data[first + k] & 0x80) ? 1 : -1;

    for (long k = FSE_SPAN; k < count - FSE_SPAN; k++) {
        int16_t amp[2];

        /* The two 16 kHz samples inside symbol k. */
        for (int h = 0; h < 2; h++) {
            double t = k + 0.5*h + phi;
            double v = 0.0;

            for (long m = -FSE_SPAN; m <= FSE_SPAN; m++)
                v += ulaw_to_linear(data[first + k + m])*fse_pulse(t - (k + m));
            amp[h] = (int16_t) (v*FSE_LOSS);
        }
        ny += v90a_fse_put(fse, amp, 2, y + ny, (int) (count + 2 - ny));
    }

    /* Alignment is not known in advance -- it is exactly what the equaliser
     * absorbed -- so search it, over the settled part of the run only. */
    settled = ny/2;
    /* The search has to span the whole start-up latency, not just the
     * equaliser's own delay: the AGC priming and the delay line together
     * swallow the first ~150 symbols before an output appears, so a window of a
     * couple of dozen symbols around the identity mapping finds nothing and
     * reports chance.  Which it did. */
    for (int delay = -32; delay <= 512; delay++) {
        int match = 0;
        int total = 0;

        for (long j = settled; j < ny; j++) {
            long k = j + FSE_SPAN + delay;

            if (k < 0  ||  k >= count)
                continue;
            total++;
            if ((y[j] >= 0.0 ? 1 : -1) == tx[k])
                match++;
        }
        if (total > 0  &&  match > best_match) {
            best_match = match;
            best_delay = delay;
        }
    }
    *match_out = (double) best_match/(double) (ny - settled);
    *disp_out = v90a_fse_dispersion(fse);
    *centre_out = v90a_fse_centre(fse);
    (void) best_delay;

    v90a_fse_free(fse);
    free(y);
    free(tx);
    free(data);
    return true;
}

/*
 * Levels, not just signs: the DIL and Phase 4 case.
 *
 * §8.4's training signals are one Ucode with a sign on it, so they survive an
 * arbitrary scale and a sign-only receiver reads them.  §8.4.1's DIL and §8.6's
 * Phase 4 are level ladders and neither survives either: they need the ladder
 * calibrated in absolute terms, and they need the equaliser to keep working on
 * a constellation CMA's constant-modulus assumption is false for.
 *
 * The stream is built rather than taken from the fixture, and that is a result
 * rather than a convenience.  Run against the multilevel tail of the Eicon
 * capture -- a real Phase 4, 22 Ucodes -- this recovers 14% of codewords with
 * frozen taps and no arrangement of the loop does better, because that
 * constellation was chosen for a clean digital bearer: its steps are ~6% apart
 * and the residual after equalising this line is a few per cent.  Which is what
 * §9.3.2.9's DIL measurement and §8.5.2's constellation selection exist to
 * prevent.  So the constellations here are ones an analogue modem on this line
 * would actually ask for, and the DIL is the real §8.4.1 sequence.
 */
typedef enum {
    FSE_ML_DIL = 0,             /* §8.4.1's DIL, from the measurement preset */
    FSE_ML_SPARSE,              /* a Phase 4 constellation sized for this line */
} fse_ml_stream_t;

#define FSE_ML_TRAIN   20000    /* symbols of TRN1d-like training */
/*
 * Where CMA hands over.  Deliberately inside the training, not at the end of
 * it: TRN1d is two levels a long way apart, so a decision on it is certain, and
 * that is the one stretch of the call where a decision-directed loop can be
 * trusted absolutely.  Handing over at the DIL instead -- once the signal is a
 * ladder -- leaves CMA's residual in the taps, and CMA's residual is about 9%
 * of the level, which is larger than the ladder's steps.
 */
#define FSE_ML_SWITCH  10000
#define FSE_ML_BODY    20000
#define FSE_ML_TRN1D   48       /* the Ucode the training runs on */

static uint32_t fse_ml_rand(uint32_t *state)
{
    *state = (*state)*1103515245u + 12345u;
    return (*state >> 16) & 0x7FFFu;
}

/* Build TRN1d-like training followed by the stream under test. */
static uint8_t *fse_ml_build(fse_ml_stream_t which, long *len_out,
                             uint8_t *set, int *set_len)
{
    long total = FSE_ML_TRAIN + FSE_ML_BODY;
    uint8_t *buf = malloc((size_t) total);
    uint32_t state = 1;
    v90_dil_desc_t dil;
    uint8_t *cycle = NULL;
    int cycle_len = 0;

    if (buf == NULL)
        return NULL;
    for (long k = 0; k < FSE_ML_TRAIN; k++)
        buf[k] = v90_codeword_compose(V90_LAW_ULAW, FSE_ML_TRN1D,
                                      (fse_ml_rand(&state) & 1) ? 1 : 0);
    if (which == FSE_ML_DIL) {
        if (!v90_dil_preset_load(V90_DIL_PRESET_MEASUREMENT, &dil)
            ||  (cycle_len = v90_dil_cycle_len(&dil)) <= 0
            ||  (cycle = malloc((size_t) cycle_len)) == NULL
            ||  v90_dil_generate_codewords(V90_LAW_ULAW, &dil, cycle,
                                           cycle_len) != cycle_len) {
            free(cycle);
            free(buf);
            return NULL;
        }
        /* What the engine passes the slicer: the Ucodes this side asked for
         * in Ja, straight off the descriptor it authored. */
        *set_len = v90_dil_ucode_set(V90_LAW_ULAW, &dil, set, 128);
        for (long k = 0; k < FSE_ML_BODY; k++)
            buf[FSE_ML_TRAIN + k] = cycle[k % cycle_len];
        free(cycle);
    } else {
        /*
         * Every fourth Ucode over the upper half of the ladder: about 26%
         * between neighbours in the top chords, which is what a line with this
         * much residual intersymbol interference supports and roughly what a
         * §8.5.2 constellation for it would look like.
         */
        *set_len = 17;
        for (int i = 0; i < 17; i++)
            set[i] = (uint8_t) (32 + 4*i);
        for (long k = 0; k < FSE_ML_BODY; k++) {
            int u = 32 + 4*(int) (fse_ml_rand(&state) % 17u);

            buf[FSE_ML_TRAIN + k] = v90_codeword_compose(V90_LAW_ULAW, u,
                                        (fse_ml_rand(&state) & 1) ? 1 : 0);
        }
    }
    *len_out = total;
    return buf;
}

/* Run one stream through the channel and report exact codeword recovery over
 * the body, plus how many decisions the equaliser refused. */
static double fse_multilevel_run(fse_ml_stream_t which, double phi,
                                 v90a_fse_mode_t after, int *rejected_out,
                                 double *ucode_err_out,
                                 uint8_t **rx_out, long *nrx_out)
{
    v90a_fse_t *fse;
    v90a_linear_t *lin;
    uint8_t *tx;
    uint8_t *rx;
    long total = 0;
    long nrx = 0;
    uint8_t body_set[128];
    int body_set_len = 0;
    int best = -1;
    long scored = 0;

    if ((tx = fse_ml_build(which, &total, body_set, &body_set_len)) == NULL)
        return -1.0;
    fse = v90a_fse_init(32, 0.0);
    lin = v90a_linear_init(V90_LAW_ULAW);
    rx = malloc((size_t) total);
    if (fse == NULL  ||  lin == NULL  ||  rx == NULL) {
        v90a_fse_free(fse);
        v90a_linear_free(lin);
        free(rx);
        free(tx);
        return -1.0;
    }
    v90a_fse_set_mode(fse, V90A_FSE_CMA);

    for (long k = FSE_SPAN; k < total - FSE_SPAN; k++) {
        int16_t amp[2];
        double sym;
        int16_t scaled;
        uint8_t code;
        double v;

        for (int h = 0; h < 2; h++) {
            double t = k + 0.5*h + phi;
            double acc = 0.0;

            if (getenv("FSE_ML_IDEAL")) {
                amp[h] = (int16_t) (ulaw_to_linear(tx[k])*FSE_LOSS);
                continue;
            }
            for (long m = -FSE_SPAN; m <= FSE_SPAN; m++)
                acc += ulaw_to_linear(tx[k + m])*fse_pulse(t - (k + m));
            amp[h] = (int16_t) (acc*FSE_LOSS);
        }
        if (k == FSE_ML_SWITCH) {
            /* What the engine does once the receiver reports the Ucode TRN1d
             * arrived on: pin the ladder, say what is on it, and hand CMA over
             * to a loop whose decisions are certain because the constellation
             * at this moment is two points. */
            uint8_t one = FSE_ML_TRN1D;

            v90a_linear_set_reference(lin, FSE_ML_TRN1D, 3000.0);
            v90a_linear_set_constellation(lin, &one, 1);
            v90a_fse_set_mode(fse, after);
            v90a_fse_set_mu(fse, V90A_FSE_MU_TRAIN);
        }
        if (k == FSE_ML_TRAIN) {
            /* And the constellation the body is transmitted on, which the
             * protocol tells a real receiver: §8.4.1's descriptor was sent in
             * Ja by this side, §8.6's is what CP selected. */
            v90a_linear_set_constellation(lin, body_set, body_set_len);
            v90a_fse_set_mu(fse, V90A_FSE_MU_TRACK);
        }
        if (v90a_fse_put(fse, amp, 2, &sym, 1) != 1)
            continue;
        v = sym*3000.0;
        if (v > 32000.0)
            v = 32000.0;
        else if (v < -32000.0)
            v = -32000.0;
        scaled = (int16_t) v;
        if (v90a_linear_put(lin, &scaled, 1, &code, 1) == 1) {
            rx[nrx++] = code;
            if (v90a_fse_mode(fse) == V90A_FSE_DD)
                v90a_fse_decide(fse, v90a_linear_last_decision(lin)/3000.0,
                                v90a_linear_last_tolerance(lin)/3000.0);
        }
        /*endif*/
    }

    /* The body only, and past the seam where the level changes. */
    for (int delay = -32; delay <= 512; delay++) {
        int match = 0;
        long n = 0;
        long nerr = 0;
        double err = 0.0;

        for (long j = FSE_ML_TRAIN; j < nrx; j++) {
            long k = j + FSE_SPAN + delay;
            int ur;
            int ut;

            if (k < FSE_ML_TRAIN + 512  ||  k >= total)
                continue;
            n++;
            if (rx[j] == tx[k])
                match++;
            v90_codeword_decompose(V90_LAW_ULAW, rx[j], &ur, NULL);
            v90_codeword_decompose(V90_LAW_ULAW, tx[k], &ut, NULL);
            /*
             * Ucodes are logarithmic -- sixteen of them are a factor of two --
             * so a level error counted in Ucodes is a RELATIVE error, and at
             * the bottom of the ladder an enormous count of them is a
             * vanishing absolute error.  Measured over the whole DIL that
             * reads 17.7 Ucodes and means almost nothing.  Count it where a
             * constellation would actually be built (§8.5.2 picks from the
             * upper ladder), which is the region §9.3.2.9's measurement has to
             * get right.
             */
            if (ut >= 48) {
                err += fabs((double) (ur - ut));
                nerr++;
            }
        }
        if (n > 0  &&  match > best) {
            best = match;
            scored = n;
            if (ucode_err_out)
                *ucode_err_out = (nerr > 0) ? err/nerr : 0.0;
        }
    }
    if (rejected_out)
        *rejected_out = v90a_fse_dd_rejected(fse);
    v90a_fse_free(fse);
    v90a_linear_free(lin);
    if (rx_out != NULL) {
        *rx_out = rx;
        *nrx_out = nrx;
    } else {
        free(rx);
    }
    /*endif*/
    free(tx);
    return (scored > 0) ? (double) best/(double) scored : -1.0;
}

/*
 * §9.3.2.9's actual question, asked the way the clause asks it.
 *
 * Exact codeword recovery is the wrong measure of a DIL and the row above says
 * so: the DIL probes the whole ladder deliberately, and most of the ladder is
 * not separable over a real line -- discovering exactly that is what it is for.
 * What §9.3.2.9 produces is a verdict on which Ucodes arrived distinguishable
 * from their neighbours, and the way that verdict can be WRONG is asymmetric.
 * Missing a usable Ucode costs a little rate.  Calling an unusable one usable
 * puts it in CP, and the digital modem then transmits a constellation the line
 * cannot carry for the rest of the call.
 *
 * So the assertion is that the measurement taken through the equaliser claims
 * nothing the same measurement on a clean bearer does not, and recovers a
 * usable fraction of it.
 */
static void test_fse_dil_measurement(void)
{
    v90_dil_desc_t dil;
    v90_dil_measurement_t ideal;
    uint8_t *clean;
    uint8_t ideal_set[128];
    int cycle_len;
    int ideal_n;

    printf("§9.3.2.9 DIL measurement through the equaliser\n");
    if (!v90_dil_preset_load(V90_DIL_PRESET_MEASUREMENT, &dil)
        ||  (cycle_len = v90_dil_cycle_len(&dil)) <= 0) {
        printf("  SKIP: no measurement preset\n");
        return;
    }
    if ((clean = malloc((size_t) cycle_len*2)) == NULL)
        return;
    v90_dil_generate_codewords(V90_LAW_ULAW, &dil, clean, cycle_len*2);
    if (!v90_dil_measure(clean, cycle_len*2, V90_LAW_ULAW, &dil, 0, &ideal)) {
        printf("  FAIL: the reference measurement did not run\n");
        failures++;
        free(clean);
        return;
    }
    ideal_n = v90_dil_measure_usable_ucodes(&ideal, ideal_set,
                                            (int) sizeof(ideal_set));
    free(clean);
    printf("  clean bearer: %d Ucodes measured, %d usable\n",
           ideal.ucodes_measured, ideal_n);

    for (int p = 0; p < 4; p++) {
        double phi = p/4.0;
        uint8_t *rx = NULL;
        long nrx = 0;
        v90_dil_measurement_t got;
        uint8_t got_set[128];
        int got_n;
        int overlap = 0;
        int false_usable = 0;
        int offset = 0;
        double score = 0.0;

        if (fse_multilevel_run(FSE_ML_DIL, phi, V90A_FSE_DD,
                               NULL, NULL, &rx, &nrx) < 0.0  ||  rx == NULL) {
            printf("  SKIP: could not build the stream\n");
            return;
        }
        /* The body starts somewhere near FSE_ML_TRAIN in the recovered stream
         * -- the equaliser and the start-up latency move it -- so let §9.3.2.8's
         * own windowed search find it, which is what the live path does. */
        if (!v90_dil_measure_align(rx, (int) nrx, V90_LAW_ULAW, &dil,
                                   FSE_ML_TRAIN - 512, 2048, &offset, &score)
            ||  !v90_dil_measure(rx, (int) nrx, V90_LAW_ULAW, &dil, offset,
                                 &got)) {
            CHECK(false, "sampling phase %.3f: the DIL was not located", phi);
            free(rx);
            continue;
        }
        got_n = v90_dil_measure_usable_ucodes(&got, got_set,
                                              (int) sizeof(got_set));
        for (int i = 0; i < got_n; i++) {
            bool in_ideal = false;

            for (int j = 0; j < ideal_n; j++)
                if (ideal_set[j] == got_set[i])
                    in_ideal = true;
            if (in_ideal)
                overlap++;
            else
                false_usable++;
        }
        CHECK(false_usable == 0,
              "sampling phase %.3f: %d Ucodes called usable that are not",
              phi, false_usable);
        CHECK(overlap*2 >= ideal_n,
              "sampling phase %.3f: only %d of %d usable Ucodes recovered",
              phi, overlap, ideal_n);
        printf("    phase %.3f -> aligned at %d (score %.3f), %d usable: "
               "%d of the clean bearer's %d, %d it should not have claimed\n",
               phi, offset, score, got_n, overlap, ideal_n, false_usable);
        free(rx);
    }
}

static void test_fse_multilevel(void)
{
    static const struct {
        fse_ml_stream_t which;
        const char *name;
    } streams[] = {
        {FSE_ML_SPARSE, "§8.6 constellation sized for this line"},
        {FSE_ML_DIL,    "§8.4.1 DIL (the §9.3.2.9 measurement)"},
    };

    printf("T/2 equaliser on multilevel downstreams, ladder pinned to TRN1d\n");
    for (size_t i = 0; i < sizeof(streams)/sizeof(streams[0]); i++) {
        printf("  %s\n", streams[i].name);
        for (int p = 0; p < 4; p++) {
            double phi = p/4.0;
            int rejected = 0;
            double dd_err = 0.0;
            double frozen = fse_multilevel_run(streams[i].which, phi,
                                               V90A_FSE_FROZEN, NULL, NULL,
                                               NULL, NULL);
            double dd = fse_multilevel_run(streams[i].which, phi,
                                           V90A_FSE_DD,
                                           &rejected, &dd_err, NULL, NULL);

            if (dd < 0.0  ||  frozen < 0.0) {
                printf("    SKIP: could not build the stream\n");
                return;
            }
            /*
             * Asserted only for the constellation an analogue modem on this
             * line would ask for.  The
             * DIL deliberately probes the whole ladder -- that is what it is
             * for -- so most of its Ucodes are not separable over any real line
             * and exact recovery of them is not the measurement §9.3.2.9 wants;
             * its number is here to be read, not to pass.
             */
            if (streams[i].which == FSE_ML_SPARSE) {
                CHECK(dd > 0.85,
                      "sampling phase %.3f: %.2f%% of codewords exact",
                      phi, dd*100.0);
                CHECK(dd >= frozen,
                      "sampling phase %.3f: decision-directed %.2f%% is worse "
                      "than frozen taps %.2f%%",
                      phi, dd*100.0, frozen*100.0);
            }
            /*
             * The DIL row is REPORTED AND NOT ASSERTED, and the reason is that
             * exact codeword recovery is the wrong question to ask of it.
             *
             * It probes the whole ladder by design -- 121 Ucodes here -- and
             * most of a real ladder is not separable over a real line, which is
             * the thing it exists to find out.  So this row reads about 25%
             * exact and four Ucodes of level error where the constellation row
             * beside it reads 90% and 0.1, and neither number is a defect.
             * What §9.3.2.9 has to get right is its VERDICT, and that is
             * asserted in test_fse_dil_measurement() below: through the
             * equaliser it recovers 90 of the clean bearer's 121 usable Ucodes
             * and wrongly claims none.
             *
             * (An earlier note here blamed this row on the slicer not being
             * told the DIL's Ucode set.  It was already being told: the set is
             * v90_dil_ucode_set() of the descriptor this side authored, and
             * the row does not move without it.  The metric was the problem.)
             */
            printf("    phase %.3f -> %.2f%% of codewords exact "
                   "(frozen taps %.2f%%), upper-ladder level error %.2f "
                   "Ucodes, %d decisions refused\n",
                   phi, dd*100.0, frozen*100.0, dd_err, rejected);
        }
    }
}

static void test_fse(void)
{
    /* TRN1d on this fixture starts at 67367 and runs 30000T (§8.4.5). */
    const long trn1d_start = 67400;
    const long trn1d_count = 24000;
    double worst = 1.0;

    printf("T/2 fractionally-spaced equaliser on TRN1d, measured HSF channel\n");
    printf("  channel: measured on the rig, symbol-spaced taps");
    for (int m = -2; m <= 2; m++)
        printf(" %+.3f", fse_pulse(m + 0.5));
    printf("\n");

    for (int p = 0; p < 8; p++) {
        double phi = p/8.0;
        double match, disp, centre;

        if (!fse_run(fixtures[0].path, trn1d_start, trn1d_count, phi,
                     &match, &disp, &centre)) {
            printf("  SKIP: fixture not present\n");
            return;
        }
        if (match < worst)
            worst = match;
        CHECK(match > 0.99,
              "sampling phase %.3f: %.2f%% of TRN1d's signs recovered",
              phi, match*100.0);
        printf("    phase %.3f -> signs %.2f%%, dispersion %.4f, "
               "tap centre %.2f symbols\n",
               phi, match*100.0, disp, centre);
    }
    printf("  worst phase: %.2f%% of signs\n", worst*100.0);
}

/*
 * The DIL stage and §9.3.2.10's stopping rule, against a DIL generated from a
 * descriptor we chose — which is what the live path has, since the analogue
 * modem authored it.
 */
static void test_dil_stage(void)
{
    v90_analogue_rx_config_t cfg;
    v90_analogue_rx_t *rx;
    const v90_dil_measurement_t *m;
    uint8_t *dil;
    unsigned events;
    int cycle;

    printf("§9.3.2.9/§9.3.2.10 DIL measurement and stopping rule\n");

    memset(&cfg, 0, sizeof(cfg));
    cfg.law = V90_LAW_ULAW;
    cfg.u_info = 78;
    if (!v90_dil_preset_load(V90_DIL_PRESET_MEASUREMENT, &cfg.dil)) {
        printf("  FAIL: could not load the measurement DIL preset\n");
        failures++;
        return;
    }
    cycle = v90_dil_cycle_len(&cfg.dil);
    CHECK(cycle > 0, "descriptor has no DIL cycle");
    if (cycle <= 0)
        return;

    if ((dil = malloc((size_t) cycle)) == NULL) {
        printf("  FAIL: out of memory\n");
        failures++;
        return;
    }
    CHECK(v90_dil_generate_codewords(V90_LAW_ULAW, &cfg.dil, dil, cycle) == cycle,
          "could not generate a DIL cycle");

    rx = v90_analogue_rx_init(&cfg);
    if (rx == NULL) {
        printf("  FAIL: receiver did not initialise\n");
        failures++;
        free(dil);
        return;
    }

    /* Drive it straight into the DIL stage: this test is about §9.3.2.9 and
     * §9.3.2.10, and the signals before them are the fixtures' job. */
    v90_analogue_rx_begin_dil(rx);
    events = v90_analogue_rx_put(rx, dil, cycle);

    CHECK((events & V90A_RX_EVENT_DIL_ENOUGH) != 0,
          "§9.3.2.10 never fired on a full DIL cycle");
    CHECK(v90_analogue_rx_stage(rx) == V90A_RX_DONE,
          "receiver stage is %s, expected done",
          v90_analogue_rx_stage_name(v90_analogue_rx_stage(rx)));
    CHECK(v90_analogue_rx_dil_symbols(rx) < cycle,
          "took the whole %d-symbol cycle; §9.3.2.10 exists to stop earlier", cycle);

    m = v90_analogue_rx_measurement(rx);
    CHECK(m != NULL, "no measurement after §9.3.2.10 fired");
    if (m) {
        CHECK(m->ucodes_measured > 0, "measurement covers no Ucodes");
        /* An unimpaired stream: what went out is what came back. */
        CHECK(m->gain_db > -0.5  &&  m->gain_db < 0.5,
              "clean DIL measured %.2f dB of gain", m->gain_db);
        CHECK(m->rbs_slot_mask == 0,
              "clean DIL reported robbed bits in slots 0x%02X", m->rbs_slot_mask);
        printf("  measured %d Ucodes from %d of %d symbols (%.0f%% of a cycle), "
               "gain %.2f dB, usable %d\n",
               m->ucodes_measured, v90_analogue_rx_dil_symbols(rx), cycle,
               100.0*m->coverage, m->gain_db, m->usable_count);
    }

    v90_analogue_rx_free(rx);
    free(dil);
}

/*
 * The join: our Phase 3 transmitter driven by a real digital modem's Phase 3.
 *
 * This is the whole point of the receive path.  §9.3.2 makes four transitions
 * conditional on the far end, and until now nothing supplied those conditions
 * but a timer in a test.  Here the Eicon card's own downstream does it, and
 * the transmitter has to walk Ja -> silence -> S -> S̄ -> Phase 4 in response.
 *
 * A zero-length DIL is requested, so §9.3.2.8 goes straight to Phase 4: the
 * DIL in these captures is the one the Courier asked for, and measuring
 * somebody else's DIL is not a thing §9.3.2.9 does.
 *
 * The two directions are not time-aligned to a real call -- our transmitter
 * starts at the first codeword of the capture, which is still in V.8.  What is
 * being checked is causality, not timing: no transition happens without the
 * signal that licenses it, and every one happens once it arrives.
 */
static void test_driven_by_fixture(const fixture_t *fx)
{
    v90_analogue_phase3_config_t cfg;
    v90_analogue_phase3_t *p3;
    uint8_t *data;
    int16_t amp[160];
    long len;
    long off;
    bool saw_ja;
    bool saw_ja_silence;
    bool saw_s_after_jd;

    printf("§9.3.2 transmitter driven by %s\n", fx->path);

    if ((data = read_file(fx->path, &len)) == NULL) {
        printf("  SKIP: fixture not present\n");
        return;
    }

    memset(&cfg, 0, sizeof(cfg));
    cfg.law = V90_LAW_ULAW;
    cfg.baud_rate_code = 4;         /* 3200 */
    cfg.high_carrier = true;
    cfg.u_info = fx->u_info;
    /* cfg.dil.n stays 0: a DIL of zero length. */
    p3 = v90_analogue_phase3_init(&cfg);
    if (p3 == NULL) {
        printf("  FAIL: Phase 3 did not initialise\n");
        failures++;
        free(data);
        return;
    }

    saw_ja = false;
    saw_ja_silence = false;
    saw_s_after_jd = false;
    for (off = 0; off < len; off += 160) {
        int n = (int) ((len - off < 160) ? (len - off) : 160);

        /* One upstream sample per downstream codeword: 8 kHz both ways. */
        v90_analogue_phase3_tx(p3, amp, n);
        v90_analogue_phase3_rx(p3, data + off, n);

        switch (v90_analogue_phase3_tx_stage(p3)) {
        case V90A_TX_JA:              saw_ja = true;         break;
        case V90A_TX_JA_SILENCE:      saw_ja_silence = true; break;
        case V90A_TX_S_AFTER_JD:      saw_s_after_jd = true; break;
        default:                                             break;
        }
    }

    CHECK(saw_ja, "the transmitter never reached Ja");
    CHECK(saw_ja_silence,
          "Ja never ended — §9.3.2.4's Sd-to-S̄d transition did not reach the transmitter");
    CHECK(saw_s_after_jd,
          "S after Jd never started — §9.3.2.7 did not reach the transmitter");
    CHECK(v90_analogue_phase3_complete(p3),
          "transmitter finished in %s, expected Phase 4",
          v90_analogue_tx_stage_name(v90_analogue_phase3_tx_stage(p3)));
    printf("  transmitter reached %s; receiver reached %s\n",
           v90_analogue_tx_stage_name(v90_analogue_phase3_tx_stage(p3)),
           v90_analogue_rx_stage_name(v90_analogue_phase3_rx_stage(p3)));

    v90_analogue_phase3_free(p3);
    free(data);
}

/*
 * §9.4.2: does the Phase 4 receiver read a Phase 4 downstream?
 *
 * The stream is generated by v90.c — the *digital* side of this tree — so this
 * is the same pairing a real call has, rather than the analogue transmitter
 * grading itself.  What it cannot check is the convention questions the Eicon
 * fixtures answer for Phase 3; there is no foreign Phase 4 downstream in this
 * tree yet, and until there is, §8.6.4's level and §8.6.5's frame alignment are
 * agreed between two halves of the same codebase.
 */
/* §8.6.3 Table 16 bits 0:16. */
#define MP_SYNC_BITS_EXPECTED   17

static void put_r_pattern(uint8_t *out, int reps, v90_law_t law, int ucode,
                          bool reversed)
{
    for (int rep = 0; rep < reps; rep++) {
        for (int slot = 0; slot < 6; slot++) {
            /* §8.6.4: + + + − − − , and R̄ the same six reversed. */
            bool positive = (slot < 3) != reversed;

            out[rep*6 + slot] = v90_codeword_compose(law, ucode, positive ? 1 : 0);
        }
    }
}

static int build_mp_type0(uint8_t *bits, uint8_t max_drn, bool acknowledge)
{
    uint16_t crc;
    int i;

    memset(bits, 0, 102);
    for (i = 0; i < 17; i++)
        bits[i] = 1;
    for (i = 0; i < 4; i++)
        bits[24 + i] = (uint8_t) ((max_drn >> i) & 1);
    bits[33] = acknowledge ? 1 : 0;
    for (i = 0; i < 13; i++)             /* bits 36:48 — the rate mask */
        bits[36 + i] = 1;
    crc = 0xFFFF;
    for (int start = 17; start < 68; start += 17) {
        for (int bit = start + 1; bit <= start + 16; bit++)
            crc = crc_itu16_bits(bits[bit] & 1U, 1, crc);
    }
    for (i = 0; i < 16; i++)
        bits[69 + i] = (uint8_t) ((crc >> i) & 1);
    return 86;
}

static int build_mp_type1(uint8_t *bits, uint8_t max_drn, bool acknowledge)
{
    uint16_t crc;
    int i;

    memset(bits, 0, 204);
    for (i = 0; i < 17; i++)
        bits[i] = 1;
    bits[18] = 1;
    for (i = 0; i < 4; i++)
        bits[24 + i] = (uint8_t) ((max_drn >> i) & 1);
    bits[33] = acknowledge ? 1 : 0;
    for (i = 0; i < 13; i++)
        bits[36 + i] = 1;
    for (i = 0; i < 16; i++) {
        bits[52 + i] = (uint8_t) ((0x1234U >> i) & 1U);
        bits[69 + i] = (uint8_t) ((0xFEDCU >> i) & 1U);
        bits[86 + i] = (uint8_t) ((0x2345U >> i) & 1U);
        bits[103 + i] = (uint8_t) ((0xDCBAU >> i) & 1U);
        bits[120 + i] = (uint8_t) ((0x3456U >> i) & 1U);
        bits[137 + i] = (uint8_t) ((0xCBA9U >> i) & 1U);
    }
    crc = 0xFFFF;
    for (int start = 17; start <= 153; start += 17) {
        for (int bit = start + 1; bit <= start + 16; bit++)
            crc = crc_itu16_bits(bits[bit] & 1U, 1, crc);
    }
    for (i = 0; i < 16; i++)
        bits[171 + i] = (uint8_t) ((crc >> i) & 1);
    return 188;
}

static void test_phase4_receive(int sr)
{
    v90_analogue_phase4_config_t cfg;
    v90_analogue_phase4_t *rx;
    v90_shaped_rx_state_t zero;
    vpcm_cp_frame_t cpt;
    vpcm_cp_frame_t cp;
    vpcm_cp_frame_t rr_mapping;
    const v90_analogue_mp_t *mp;
    uint8_t mp_bits[204];
    uint8_t *plain;
    uint8_t data_plain[49*64];
    uint8_t *stream;
    unsigned events;
    int bits_per_frame;
    int trn2d_frames;
    int frames;
    int mp_nbits;
    int n;
    int b1_n;
    int startup_n;
    int len;
    const int r_reps = 32;               /* §9.4.1.1: at least 192T */
    const int r_bar_reps = 4;            /* §8.6.4: exactly four */
    const int ucode = 40;

    printf("§9.4.2 receive against a v90.c-generated Phase 4 downstream (Sr=%d)\n",
           sr);

    vpcm_cp_init(&cpt);
    cpt.v90_compatibility = false;       /* Table 14 bit 19: 0 = CPt */
    cpt.drn = 12;
    /*
     * §5.4.5: Sr = 0 disables spectral shaping and the six sign bits are
     * §5.4.5.1's differential chain; 1 to 3 run the shaper.  Both are legal
     * for a CPt and the two demap by different routes, so the receiver is run
     * against a downstream generated each way.
     */
    cpt.shaping_redundancy = (uint8_t) sr;
    cpt.shaping_lookahead = 0;
    cpt.constellation_count = 1;
    vpcm_cp_enable_all_ucodes(cpt.masks[0]);
    for (int i = 0; i < VPCM_CP_FRAME_INTERVALS; i++)
        cpt.dfi[i] = 0;

    cp = cpt;
    cp.v90_compatibility = true;
    cp.drn = 1;                         /* D=21, valid data-mode CP for init */

    bits_per_frame = cpt.drn + 8;
    /* §9.4.1.2 sends at least 2040T of TRN2d; 400 frames is 2400T. */
    trn2d_frames = 400;
    mp_nbits = (sr == 0) ? build_mp_type1(mp_bits, 14, true)
                         : build_mp_type0(mp_bits, 14, true);
    frames = trn2d_frames + (mp_nbits + bits_per_frame - 1)/bits_per_frame + 2;

    plain = calloc((size_t) frames*bits_per_frame, 1);
    /* Startup plus one complete §9.6.2.2 response cycle: 384T Rd, 24T R̄d,
     * TRN2d/MP/Ed under CP with CPt's K, then another B1d/data pair. */
    len = (r_reps + r_bar_reps)*6
        + (frames + 49 + 64 + 4 + frames + 64 + 4 + frames + 49)*6;
    stream = calloc((size_t) len, 1);
    if (plain == NULL  ||  stream == NULL) {
        printf("  FAIL: out of memory\n");
        failures++;
        free(plain);
        free(stream);
        return;
    }
    /* §8.6.5: TRN2d is scrambled ones.  MP follows in the same bit stream. */
    memset(plain, 1, (size_t) trn2d_frames*bits_per_frame);
    memcpy(plain + trn2d_frames*bits_per_frame, mp_bits, (size_t) mp_nbits);
    /* Fill to the MP mapping boundary and §8.6.2's following two complete Ed
     * frames are zero.  B1d below starts a separately reset CP mapper. */
    for (int i = trn2d_frames*bits_per_frame + mp_nbits;
         i < frames*bits_per_frame;
         i++) {
        plain[i] = 0;
    }

    put_r_pattern(stream, r_reps, V90_LAW_ULAW, ucode, false);
    put_r_pattern(stream + r_reps*6, r_bar_reps, V90_LAW_ULAW, ucode, true);
    memset(&zero, 0, sizeof(zero));
    n = v90_generate_phase4_codewords(V90_LAW_ULAW, &cpt, &zero, plain, frames,
                                      stream + (r_reps + r_bar_reps)*6,
                                      len - (r_reps + r_bar_reps)*6);
    if (n <= 0) {
        printf("  FAIL: could not generate a Phase 4 downstream\n");
        failures++;
        free(plain);
        free(stream);
        return;
    }
    /* §8.6.1: reset mapper, 48 CP-mapped frames of ones, then one data frame
     * whose alternating payload verifies that state remains continuous. */
    memset(data_plain, 1, 48*(cp.drn + 20));
    for (int i = 0; i < cp.drn + 20; i++)
        data_plain[48*(cp.drn + 20) + i] = (uint8_t)(i & 1);
    memset(&zero, 0, sizeof(zero));
    b1_n = v90_generate_phase4_codewords(
        V90_LAW_ULAW, &cp, &zero, data_plain, 49,
        stream + (r_reps + r_bar_reps)*6 + n,
        len - (r_reps + r_bar_reps)*6 - n);
    if (b1_n != 49*6) {
        printf("  FAIL: could not generate B1d/data downstream\n");
        failures++;
        free(plain);
        free(stream);
        return;
    }
    n += b1_n;
    startup_n = n;

    /* Digital-modem-initiated rate renegotiation (§9.6.1.1/§9.6.2.2). */
    put_r_pattern(stream + (r_reps + r_bar_reps)*6 + n,
                  64, V90_LAW_ULAW, 127, false);       /* Rd: 384T */
    n += 64*6;
    put_r_pattern(stream + (r_reps + r_bar_reps)*6 + n,
                  4, V90_LAW_ULAW, 127, true);         /* R̄d: 24T */
    n += 4*6;
    rr_mapping = cp;
    rr_mapping.v90_compatibility = false;
    rr_mapping.drn = cpt.drn;  /* §8.6: CP constellation/shaping, CPt K */
    memset(&zero, 0, sizeof(zero));
    b1_n = v90_generate_phase4_codewords(
        V90_LAW_ULAW, &rr_mapping, &zero, plain, frames,
        stream + (r_reps + r_bar_reps)*6 + n,
        len - (r_reps + r_bar_reps)*6 - n);
    CHECK(b1_n == frames*6,
          "could not generate rate-renegotiation TRN2d/MP/Ed");
    n += b1_n;
    if (sr == 0) {
        /* §9.6.2.1.6-.8 CPs path: after first Ed/silence, Rt/R̄t starts a
         * second clear-bit-30 CP/MP transaction. */
        put_r_pattern(stream + (r_reps + r_bar_reps)*6 + n,
                      64, V90_LAW_ULAW, 127, false);
        n += 64*6;
        put_r_pattern(stream + (r_reps + r_bar_reps)*6 + n,
                      4, V90_LAW_ULAW, 127, true);
        n += 4*6;
        memset(&zero, 0, sizeof(zero));
        b1_n = v90_generate_phase4_codewords(
            V90_LAW_ULAW, &rr_mapping, &zero, plain, frames,
            stream + (r_reps + r_bar_reps)*6 + n,
            len - (r_reps + r_bar_reps)*6 - n);
        CHECK(b1_n == frames*6,
              "could not generate post-Rt TRN2d/MP/Ed");
        n += b1_n;
    }
    memset(&zero, 0, sizeof(zero));
    b1_n = v90_generate_phase4_codewords(
        V90_LAW_ULAW, &cp, &zero, data_plain, 49,
        stream + (r_reps + r_bar_reps)*6 + n,
        len - (r_reps + r_bar_reps)*6 - n);
    CHECK(b1_n == 49*6, "could not generate post-renegotiation B1d/data");
    n += b1_n;

    memset(&cfg, 0, sizeof(cfg));
    cfg.law = V90_LAW_ULAW;
    cfg.u_info = 48;
    cfg.cpt = cpt;
    cfg.cp = cp;
    if ((rx = v90_analogue_phase4_init(&cfg)) == NULL) {
        printf("  FAIL: Phase 4 receiver did not initialise\n");
        failures++;
        free(plain);
        free(stream);
        return;
    }

    events = 0;
    {
        int startup_end = (r_reps + r_bar_reps)*6 + startup_n;
        int total = (r_reps + r_bar_reps)*6 + n;

        for (int off = 0; off < startup_end; off += 160) {
            int take = startup_end - off;
            if (take > 160)
                take = 160;
            events |= v90_analogue_phase4_put(rx, stream + off, take);
        }
        CHECK(v90_analogue_phase4_start_rate_renegotiation(rx, sr == 0),
              "could not arm local rate renegotiation");
        for (int off = startup_end; off < total; off += 160) {
            int take = total - off;
            if (take > 160)
                take = 160;
            events |= v90_analogue_phase4_put(rx, stream + off, take);
        }
    }

    CHECK((events & V90A4_RX_EVENT_R) != 0, "Ri never acquired");
    CHECK((events & V90A4_RX_EVENT_R_BAR) != 0,
          "§9.4.2.2's Ri-to-R̄i transition never seen — CPt would never end");
    CHECK((events & V90A4_RX_EVENT_TRN2D) != 0, "TRN2d never started");
    CHECK((events & V90A4_RX_EVENT_MP) != 0,
          "no MP frame passed Table 16's structure and CRC");
    CHECK((events & V90A4_RX_EVENT_MP_PRIME) != 0,
          "MP' not recognised — §9.4.2.4's E would never be sent");
    CHECK((events & V90A4_RX_EVENT_ED) != 0,
          "two complete Ed frames were not recognised");
    CHECK((events & V90A4_RX_EVENT_B1D) != 0,
          "B1d did not start on the reset CP mapper");
    CHECK((events & V90A4_RX_EVENT_DATA) != 0,
          "48 B1d frames did not release downstream data");
    CHECK((events & V90A4_RX_EVENT_RD) != 0,
          "§9.6.2.2.1 Rd was not acquired on the data-frame grid");
    CHECK((events & V90A4_RX_EVENT_RD_BAR) != 0,
          "§9.6.2.2.2 Rd-to-R̄d transition was not detected");
    if (sr == 0) {
        CHECK((events & V90A4_RX_EVENT_RT) != 0,
              "§9.6.2.1.7 Rt was not acquired after CPs silence");
        CHECK((events & V90A4_RX_EVENT_RT_BAR) != 0,
              "§9.6.2.1.8 Rt-to-R̄t transition was not detected");
    }
    CHECK(v90_analogue_phase4_rate_renegotiations(rx) == 1,
          "completed %d rate renegotiations, expected one",
          v90_analogue_phase4_rate_renegotiations(rx));
    CHECK(v90_analogue_phase4_b1d_frames(rx) == 48,
          "B1d had %d frames, expected 48",
          v90_analogue_phase4_b1d_frames(rx));
    CHECK(v90_analogue_phase4_b1d_bit_errors(rx) == 0,
          "B1d known plaintext had %d bit errors",
          v90_analogue_phase4_b1d_bit_errors(rx));
    {
        uint8_t got[64];
        int got_n = v90_analogue_phase4_get_data_bits(rx, got, sizeof(got));

        CHECK(got_n == 2*(cp.drn + 20), "decoded %d data bits, expected %d",
              got_n, 2*(cp.drn + 20));
        for (int i = 0; i < got_n; i++)
            CHECK(got[i] == (uint8_t)((i % (cp.drn + 20)) & 1),
                  "data bit %d broke B1d mapper continuity", i);
    }
    CHECK(v90_analogue_phase4_demap_failures(rx) == 0,
          "%d frames would not demap against the CPt they were mapped with",
          v90_analogue_phase4_demap_failures(rx));
    /*
     * §8.6.5 is scrambled ones, so the run before the first zero covers all of
     * TRN2d — plus §8.6.3's seventeen sync ones, since MP follows with no gap
     * and its start bit is the first zero either side can see.  That +17 is
     * the same coincidence §8.4.2's Jd plays against TRN1d, and asserting it
     * exactly is what says the two signals were told apart at the right bit.
     */
    CHECK(v90_analogue_phase4_trn2d_ones(rx)
          == trn2d_frames*bits_per_frame + MP_SYNC_BITS_EXPECTED,
          "TRN2d demapped to %d ones, expected %d",
          v90_analogue_phase4_trn2d_ones(rx),
          trn2d_frames*bits_per_frame + MP_SYNC_BITS_EXPECTED);

    if ((mp = v90_analogue_phase4_mp(rx)) != NULL) {
        CHECK(mp->max_drn == 14, "MP max drn %u, expected 14", mp->max_drn);
        CHECK(mp->acknowledge, "MP acknowledge bit not read back");
        CHECK(mp->type1 == (sr == 0), "MP type was not decoded correctly");
        if (sr == 0) {
            CHECK(mp->precoder[0][0] == 0x1234
                  &&  (uint16_t) mp->precoder[0][1] == 0xFEDC
                  &&  mp->precoder[2][0] == 0x3456,
                  "Type-1 precoder coefficients were not decoded correctly");
        }
        printf("  Ri %dT, TRN2d %dT, %d MP frame(s): max drn=%u (%d bps), "
               "rate mask 0x%04X, %s\n",
               v90_analogue_phase4_r_symbols(rx),
               v90_analogue_phase4_trn2d_symbols(rx),
               v90_analogue_phase4_mp_frames(rx),
               mp->max_drn, mp->max_drn*2400, mp->rate_mask,
               mp->acknowledge ? "MP'" : "MP");
    } else {
        printf("  FAIL: no MP decoded\n");
        failures++;
    }

    v90_analogue_phase4_free(rx);
    free(plain);
    free(stream);
}

/*
 * §8.5.2: does a measurement turn into a CPt/CP pair the digital modem could
 * act on?  The trap Table 14 sets is that CP's rate field is (drn + 20) and
 * CPt's is (drn + 8), so the same line rate is a different number in each.
 */
static void test_phase4_cp_from_measurement(void)
{
    v90_dil_measurement_t m;
    vpcm_cp_frame_t cpt;
    vpcm_cp_frame_t cp;

    printf("§8.5.2 CPt/CP from a DIL measurement\n");

    memset(&m, 0, sizeof(m));
    /* A clean line: every Ucode arrived where it was sent, in every interval. */
    for (int u = 1; u < V90_DIL_UCODES; u++) {
        m.usable[u] = true;
        m.u[u].tx_count = 6;
        m.u[u].rx_ucode = u;
        m.u[u].rx_agree = 6;
        m.u[u].rx_distinct = 1;
        m.u[u].tx_level = u*16;
        m.u[u].rx_level = u*16;
        for (int i = 0; i < 6; i++) {
            m.u[u].rx_ucode_slot[i] = u;
            m.u[u].rx_level_slot[i] = u*16;
        }
    }
    m.ucodes_measured = V90_DIL_UCODES - 1;
    m.usable_count = V90_DIL_UCODES - 1;
    m.coverage = 1.0;

    /*
     * Every Sr §5.4.5 allows, because the rate arithmetic is different in each
     * and getting it wrong is invisible from this side: a CPt that no digital
     * modem can build a mapper from still encodes, still passes its own CRC,
     * and still demaps against itself.
     */
    for (int sr = 0; sr <= 3; sr++) {
        int ld = (sr == 0) ? 0 : 1;
        v90_state_t *digital;
        int cpt_k;
        int cp_k;

        if (!v90_analogue_phase4_build_cp(&m, V90_LAW_ULAW, 0.0,
                                           sr, ld, &cpt, &cp)) {
            printf("  FAIL: Sr=%d yielded no offerable constellation\n", sr);
            failures++;
            continue;
        }
        CHECK(cpt.v90_compatibility == false, "CPt has Table 14 bit 19 set");
        CHECK(cp.v90_compatibility == true, "CP has Table 14 bit 19 clear");
        CHECK(cpt.drn <= 22, "CPt drn %u exceeds Table 14's range", cpt.drn);
        CHECK(cp.drn <= 28, "CP drn %u exceeds what vpcm_cp accepts", cp.drn);
        CHECK(cpt.shaping_redundancy == sr  &&  cp.shaping_redundancy == sr,
              "Sr=%d asked for, Sr=%u/%u built", sr,
              cpt.shaping_redundancy, cp.shaping_redundancy);
        /*
         * §8.6.5: TRN2d/MP/Ed are mapped with the CPt set, and a digital modem
         * takes K from the constellation it is handed -- so CPt's mask capacity
         * must itself fit Table 17's K <= 24, not merely its drn field.  A CPt
         * that keeps CP's full masks (prod(Mi) ~ 2^31) while declaring K=24 is
         * what let the Eicon card build a K=31 TRN2d that no K=24 receiver can
         * demap (artifacts/eicon-phase4-downstream/run79).  CPt must therefore
         * be a *reduced subset* of CP: every CPt point is in CP (so §8.5.2's
         * "CP no more than 3 dB above CPt" holds in the safe direction, CPt
         * keeping the higher-amplitude points), and prod(Mi) is commensurate
         * with K rather than with the data rate. */
        {
            uint64_t cpt_prod = 1;

            for (int i = 0; i < 6; i++) {
                for (int u = 0; u < VPCM_CP_MASK_BITS; u++) {
                    if (vpcm_cp_mask_get(cpt.masks[i], u)) {
                        CHECK(vpcm_cp_mask_get(cp.masks[i], u),
                              "Sr=%d: CPt point Ucode %d in interval %d is not "
                              "in CP -- CPt is not a subset of the data set",
                              sr, u, i);
                    }
                }
                cpt_prod *= (uint64_t) vpcm_cp_mask_population(cpt.masks[i]);
            }
            CHECK(cpt_prod < (1ULL << 25),
                  "Sr=%d: CPt mask capacity prod(Mi)=%llu implies K>24, so a "
                  "digital modem can build TRN2d above Table 17's cap",
                  sr, (unsigned long long) cpt_prod);
        }

        /* Table 17 and §5.4.3, which is where a wrong Sr shows up. */
        cpt_k = v90_analogue_phase4_cp_k(&cpt);
        cp_k = v90_analogue_phase4_cp_k(&cp);
        CHECK(cpt_k >= 6  &&  cpt_k <= 24,
              "Sr=%d: CPt K=%d is outside Table 17's 6..24", sr, cpt_k);
        CHECK(cp_k > 0, "Sr=%d: CP K does not fit §5.4.3", sr);
        CHECK(cpt_k == cpt.drn + 8 - (6 - sr),
              "Sr=%d: CPt K=%d does not follow §5.4.1 from D=%d",
              sr, cpt_k, cpt.drn + 8);

        /*
         * The check that matters, made by the other half of this tree rather
         * than by an assertion written next to the code under test.  §9.4.1.2
         * sends R̄i only after *receiving* a CPt, and a digital modem that
         * cannot configure a mapper from one has not received it -- which is
         * exactly how a Phase 4 stalls in Ri with the line looking perfect.
         */
        if ((digital = v90_init_data_pump(V90_LAW_ULAW)) != NULL) {
            CHECK(v90_set_phase4_cp(digital, &cpt),
                  "Sr=%d: a V.90 digital modem rejects our CPt "
                  "(drn=%u, D=%d, K=%d)",
                  sr, cpt.drn, cpt.drn + 8, cpt_k);
            CHECK(v90_set_phase4_cp(digital, &cp),
                  "Sr=%d: a V.90 digital modem rejects our CP "
                  "(drn=%u, D=%d, K=%d)",
                  sr, cp.drn, cp.drn + 20, cp_k);
            v90_free(digital);
        }
        /*endif*/
        printf("  Sr=%d: CPt drn=%u (%d bits/frame, K=%d), "
               "CP drn=%u (%d bits/frame, K=%d, %.0f bps)\n",
               sr, cpt.drn, cpt.drn + 8, cpt_k,
               cp.drn, cp.drn + 20, cp_k, vpcm_cp_drn_to_bps(cp.drn));

        /* §9.3.2.8's N=0 path has no measurement but must still reach Phase 4. */
        CHECK(v90_analogue_phase4_build_zero_dil_cp(
                  V90_LAW_ULAW, sr, ld, &cpt, &cp),
              "Sr=%d: zero-length DIL produced no conservative CP", sr);
        CHECK(cp.drn == 1 && cpt.drn == 4,
              "Sr=%d: zero-DIL rates were CP=%u CPt=%u", sr, cp.drn, cpt.drn);
        if ((digital = v90_init_data_pump(V90_LAW_ULAW)) != NULL) {
            CHECK(v90_set_phase4_cp(digital, &cpt),
                  "Sr=%d: digital modem rejects zero-DIL CPt", sr);
            CHECK(v90_set_phase4_cp(digital, &cp),
                  "Sr=%d: digital modem rejects zero-DIL CP", sr);
            v90_free(digital);
        }
    }
}

/*
 * --trace <file.ulaw> [u_info] — run the receiver over an arbitrary downstream
 * and print every stage change, rather than checking a fixture's known values.
 *
 * This is the offline half of a live call: ME_G711_CAPTURE records what the
 * analogue role actually consumed, and this replays it.  A stage trace is what
 * separates "the decoder is wrong" from "the stream is not what was sent" --
 * a receiver oscillating between TRN1d and Jd looks, in the engine's own log,
 * exactly like a receiver that has not reached Jd yet.
 */
static int trace_stream(const char *path, int u_info)
{
    v90_analogue_rx_config_t cfg;
    v90_analogue_rx_stage_t last;
    v90_analogue_rx_t *rx;
    uint8_t *data;
    long len;
    int transitions;

    if ((data = read_file(path, &len)) == NULL) {
        printf("cannot read %s\n", path);
        return 1;
    }
    memset(&cfg, 0, sizeof(cfg));
    cfg.law = V90_LAW_ULAW;
    cfg.u_info = u_info;
    /* The descriptor the engine requests by default, so a replayed live
     * capture reaches §9.3.2.9 the same way the call did. */
    (void) v90_dil_preset_load(V90_DIL_PRESET_MEASUREMENT, &cfg.dil);
    if ((rx = v90_analogue_rx_init(&cfg)) == NULL) {
        free(data);
        printf("receiver did not initialise\n");
        return 1;
    }

    printf("§9.3.2 trace of %s (%ld octets, %.1f s), U_INFO=%d\n",
           path, len, len/8000.0, u_info);
    last = v90_analogue_rx_stage(rx);
    transitions = 0;
    for (long off = 0; off < len; off += 160) {
        int n = (int) ((len - off < 160) ? (len - off) : 160);
        v90_analogue_rx_stage_t now;

        (void) v90_analogue_rx_put(rx, data + off, n);
        if ((now = v90_analogue_rx_stage(rx)) != last) {
            last = now;
            transitions++;
            /* An oscillating seam produces hundreds of these; print enough to
             * see the pattern and then just count. */
            if (transitions <= 40) {
                printf("  [%8.1f ms] -> %-10s Sd %d, S̄d %d, TRN1d %dT, Jd %d\n",
                       (off + n)/8.0,
                       v90_analogue_rx_stage_name(now),
                       v90_analogue_rx_sd_reps(rx),
                       v90_analogue_rx_sd_bar_reps(rx),
                       v90_analogue_rx_trn1d_symbols(rx),
                       v90_analogue_rx_jd_frames(rx));
            }
        }
    }
    printf("  end: %s after %d stage changes — Sd %d reps, S̄d %d reps, "
           "TRN1d %dT, Jd %d frames, DIL %dT\n",
           v90_analogue_rx_stage_name(v90_analogue_rx_stage(rx)), transitions,
           v90_analogue_rx_sd_reps(rx), v90_analogue_rx_sd_bar_reps(rx),
           v90_analogue_rx_trn1d_symbols(rx), v90_analogue_rx_jd_frames(rx),
           v90_analogue_rx_dil_symbols(rx));
    {
        const v90_dil_measurement_t *m = v90_analogue_rx_measurement(rx);

        if (m != NULL) {
            printf("  DIL: %d Ucodes measured, %d usable, gain %.2f dB, "
                   "RBS slots 0x%02X, coverage %.0f%%\n",
                   m->ucodes_measured, m->usable_count, m->gain_db,
                   m->rbs_slot_mask, 100.0*m->coverage);
        }
    }

    v90_analogue_rx_free(rx);
    free(data);
    return 0;
}

static void test_tone_b_retrain_detector(void)
{
    v90_analogue_phase3_config_t cfg;
    v90_analogue_phase3_t *p3;
    uint8_t pcm[480];
    unsigned events;

    printf("§9.5.2.2 Tone B retrain detection (>50 ms)\n");
    memset(&cfg, 0, sizeof(cfg));
    cfg.law = V90_LAW_ULAW;
    cfg.baud_rate_code = 4;
    cfg.u_info = 48;
    p3 = v90_analogue_phase3_init(&cfg);
    CHECK(p3 != NULL, "could not initialise Tone B detector fixture");
    if (p3 == NULL)
        return;
    for (int i = 0; i < (int)sizeof(pcm); i++) {
        double x = 5000.0*sin(2.0*3.14159265358979323846*1200.0*i/8000.0);
        pcm[i] = linear_to_ulaw((int16_t)x);
    }
    events = v90_analogue_phase3_rx(p3, pcm, 400); /* exactly 50 ms */
    CHECK((events & V90A_EVENT_TONE_B_RETRAIN) == 0,
          "Tone B fired at 50 ms instead of more than 50 ms");
    events = v90_analogue_phase3_rx(p3, pcm + 400, 80);
    CHECK((events & V90A_EVENT_TONE_B_RETRAIN) != 0,
          "Tone B did not fire after 60 ms");
    v90_analogue_phase3_free(p3);
}


/*
 * §9.4.2 against the one foreign Phase 4 downstream we have.
 *
 * artifacts/eicon-phase4-downstream/run79-ri-rbar-trn2d.ulaw is 30 s of an
 * Eicon Diva Server's own §9.4 downstream, captured on the analogue side of a
 * live call.  Every other Phase 4 test in this file is fed by v90.c, so both
 * halves share any wrong assumption and agree anyway; this one cannot.
 *
 * What it pins is the acquisition, which is what run 79 actually proved: Ri is
 * found without being told its Ucode, §9.4.1.2's R̄i transition is detected,
 * and TRN2d starts on the right symbol.  The card ignores U_INFO and sends Ri
 * at Ucode 22 where this call announced 48, so a receiver that trusts the
 * announced value finds nothing.  None of this had automated coverage.
 *
 * It also pins the answer to the question the fixture was kept to settle.  The
 * README asked whether the card's TRN2d fails because it declines the
 * constellation our CPt named, or because our six-symbol frame grid is off at
 * the R̄i seam.  Measured here, neither is quite it:
 *
 *   - The grid is right.  This receiver puts TRN2d's first symbol at sample
 *     6135, and an independent scan agrees: Ri's +++--- reverses to ---+++
 *     after 2544 symbols and runs 24 more, ending at 6135.
 *   - Nothing is outside the constellation.  Splitting the demap failure by
 *     cause gives out-of-constellation 0 and modulus overflow on every frame.
 *   - The card maps TRN2d at K ~= 31, the data K, not CPt's K = 24.  Reading
 *     each frame back through the §5.4.3 modulus decoder over run 79's data
 *     constellation Mi = 39 39 37 39 39 39 gives modulus values reaching
 *     2^31.6 (median 2^31.3); 99.6% exceed 2^24.  A receiver holding TRN2d to
 *     CPt's K = 24 overflows on every frame.
 *
 * That was ours: run 79 built CPt from the *full* data masks (prod(Mi) ~ 2^31)
 * and only relabelled drn to K = 24, and §8.6.5 has the digital modem take K
 * from the constellation it is handed, so the card faithfully mapped TRN2d at
 * K = 31.  v90_analogue_phase4_build_cp now reduces CPt to a genuine subset
 * with prod(Mi) commensurate with K <= 24 (see the test at the top of this
 * file).  This capture predates the fix; against it, demapping at CPt's K = 24
 * still overflows, so MP is asserted NOT to decode.  Live re-verification is
 * the open item.
 */
static void test_phase4_foreign_downstream(void)
{
    static const char path[] =
        "artifacts/eicon-phase4-downstream/run79-ri-rbar-trn2d.ulaw";
    /* Independently derived from the capture, not from this decoder: the
     * Ucode-22 run starts at sample 3567 and its +++--- grid reverses to
     * ---+++ after 2544 symbols, running 24 more.  See the fixture README. */
    const int expect_ri_start = 3567;
    const int expect_ri_symbols = 2544;
    const int expect_trn2d_start = 6135;   /* 3567 + 2544 + 24 */
    const int r_rep_len = 6;               /* §8.6.4: +++--- is six symbols */
    v90_analogue_phase4_config_t cfg;
    v90_analogue_phase4_t *rx;
    vpcm_cp_frame_t cpt;
    vpcm_cp_frame_t cp;
    unsigned events = 0;
    uint8_t *stream;
    long len;

    printf("§9.4.2 receive against the Eicon card's own Phase 4 downstream\n");
    (void)expect_ri_start;

    if ((stream = read_file(path, &len)) == NULL) {
        printf("  FAIL: could not read %s\n", path);
        failures++;
        return;
    }

    /* The widest legal CPt: the point is to let acquisition run, not to
     * reproduce run 79's negotiated constellation. */
    vpcm_cp_init(&cpt);
    cpt.v90_compatibility = false;
    cpt.drn = 12;
    cpt.shaping_redundancy = 0;
    cpt.shaping_lookahead = 0;
    cpt.constellation_count = 1;
    vpcm_cp_enable_all_ucodes(cpt.masks[0]);
    for (int i = 0; i < VPCM_CP_FRAME_INTERVALS; i++)
        cpt.dfi[i] = 0;
    cp = cpt;
    cp.v90_compatibility = true;
    cp.drn = 1;

    memset(&cfg, 0, sizeof(cfg));
    cfg.law = V90_LAW_ULAW;
    cfg.u_info = 48;                 /* what the call announced */
    cfg.cpt = cpt;
    cfg.cp = cp;
    if ((rx = v90_analogue_phase4_init(&cfg)) == NULL) {
        printf("  FAIL: Phase 4 receiver did not initialise\n");
        failures++;
        free(stream);
        return;
    }

    for (long off = 0; off < len; off += 160) {
        int take = (int) ((len - off > 160) ? 160 : (len - off));
        events |= v90_analogue_phase4_put(rx, stream + off, take);
    }

    printf("  Ri %dT, TRN2d %dT, MP frames %d\n",
           v90_analogue_phase4_r_symbols(rx),
           v90_analogue_phase4_trn2d_symbols(rx),
           v90_analogue_phase4_mp_frames(rx));
    printf("  demap failures %d (out-of-constellation %d, modulus overflow %d)\n",
           v90_analogue_phase4_demap_failures(rx),
           v90_analogue_phase4_demap_out_of_constellation(rx),
           v90_analogue_phase4_demap_modulus_overflow(rx));

    CHECK((events & V90A4_RX_EVENT_R) != 0,
          "Ri never acquired in the card's downstream -- §8.6.4's Ucode is "
          "learned from the wire because this card does not honour U_INFO "
          "(it sent Ucode 22 against the announced %d)", cfg.u_info);
    CHECK((events & V90A4_RX_EVENT_R_BAR) != 0,
          "§9.4.1.2's Ri-to-R̄i transition never seen -- CPt would never end");
    CHECK((events & V90A4_RX_EVENT_TRN2D) != 0,
          "TRN2d never started after the R̄i transition");
    /*
     * The load-bearing quantity is where TRN2d begins, not how the preceding
     * symbols are labelled: it sets the six-symbol mapping grid for everything
     * after it.  Check it directly against the offset an independent scan of
     * the Ri sign pattern gives.
     */
    CHECK((int)len - v90_analogue_phase4_trn2d_symbols(rx) == expect_trn2d_start,
          "TRN2d starts at sample %d, expected %d -- the mapping grid for the "
          "whole of §9.4 hangs off this",
          (int) len - v90_analogue_phase4_trn2d_symbols(rx), expect_trn2d_start);
    /*
     * Ri is reported as 2556T, not the 2544T it actually runs: the reversal
     * detector needs two of R̄i's four repetitions before it fires, and those
     * 12 symbols are counted against Ri.  That costs nothing -- TRN2d still
     * starts in the right place, as checked above -- but the number is not the
     * length of Ri, and pinning it here stops it being read as one.
     */
    CHECK(v90_analogue_phase4_r_symbols(rx)
              == expect_ri_symbols + 2*r_rep_len,
          "Ri reported %dT, expected %dT (%dT of Ri plus the %dT of R̄i the "
          "reversal detector consumes before firing)",
          v90_analogue_phase4_r_symbols(rx),
          expect_ri_symbols + 2*r_rep_len, expect_ri_symbols,
          2*r_rep_len);
    /*
     * The finding, not a tolerance.  Every frame is rejected by §5.4.3's
     * modulus check and not one codeword is outside the constellation, so the
     * card is addressing more points than the K a CPt can announce -- it is
     * not using the constellation we named.
     */
    CHECK(v90_analogue_phase4_demap_out_of_constellation(rx) == 0,
          "%d frames carried a codeword outside the constellation; the "
          "measured failure is modulus overflow, not membership",
          v90_analogue_phase4_demap_out_of_constellation(rx));
    CHECK(v90_analogue_phase4_demap_modulus_overflow(rx)
              == v90_analogue_phase4_trn2d_symbols(rx)/6,
          "modulus overflow on %d of %d frames; it was every frame when this "
          "was measured",
          v90_analogue_phase4_demap_modulus_overflow(rx),
          v90_analogue_phase4_trn2d_symbols(rx)/6);
    CHECK(v90_analogue_phase4_mp_frames(rx) == 0,
          "the card's TRN2d/MP now decodes (%d MP frames) -- that is the "
          "question the fixture README was kept open for; update it and "
          "tighten this test",
          v90_analogue_phase4_mp_frames(rx));

    v90_analogue_phase4_free(rx);
    free(stream);
}

int main(int argc, char *argv[])
{
    size_t i;

    if (argc >= 3  &&  strcmp(argv[1], "--trace") == 0)
        return trace_stream(argv[2], (argc >= 4) ? atoi(argv[3]) : 48);

    for (i = 0; i < sizeof(fixtures)/sizeof(fixtures[0]); i++)
        test_fixture(&fixtures[i]);
    for (i = 0; i < sizeof(fixtures)/sizeof(fixtures[0]); i++)
        test_linear_bearer(&fixtures[i]);
    for (i = 0; i < sizeof(fixtures)/sizeof(fixtures[0]); i++)
        test_driven_by_fixture(&fixtures[i]);
    test_fse();
    test_fse_multilevel();
    test_fse_dil_measurement();
    test_dil_stage();
    for (int sr = 0; sr <= 3; sr++)
        test_phase4_receive(sr);
    test_phase4_foreign_downstream();
    test_phase4_cp_from_measurement();
    test_tone_b_retrain_detector();

    if (failures) {
        printf("%d failure%s\n", failures, failures == 1 ? "" : "s");
        return 1;
    }
    printf("all analogue Phase 3 receiver checks passed\n");
    return 0;
}
