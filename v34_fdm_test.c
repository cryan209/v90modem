/*
 * Frequency-division multiplex of N unmodified 8 kHz V.34 modem pairs over
 * one wideband linear channel -- a "poor man's OFDM".
 *
 * Each pair is two ordinary v34_state_t instances (caller and answerer), run
 * exactly as v34_duplex_test runs them.  A channel bank moves each pair's
 * 8 kHz line signal into its own 4 kHz slot of a wideband stream at FDM_RATE
 * (48 or 96 kHz, an integer multiple of 8000), and back out again at the far
 * end.  Nothing but the wideband waveform crosses the seam, so every slot has
 * to train, negotiate its rate (V.34 clause 11 Phase 2's own L1/L2 probe of
 * its own slot) and carry payload from the waveform alone.
 *
 * The bearer is four-wire: caller-to-answerer and answer-to-caller are two
 * separate wideband streams (a stereo sound card looped back L->L, R->R), so
 * there is no echo path and no echo canceller.
 *
 * Slot k occupies [4000k, 4000k + 4000) Hz.  An 8 kHz real line signal lives
 * in 0..4000 Hz (V.34 at 3429 baud: 245..3674 Hz, roll-off to ~40..3880), so
 * the shift has to be single-sideband or the negative-frequency image lands in
 * the neighbouring slot:
 *
 *   TX  z[n] = x[n] e^{-j pi n/2}           (8 kHz; the wanted half now sits
 *                                            in -1960..+1880 Hz)
 *       u    = L-fold interpolation of z through a low-pass that keeps only
 *              that half (the other half sits 2120..5960 Hz, mod 8000)
 *       w[m] = 2 Re{ u[m] e^{j 2 pi (4000k + 2000) m / fs} }
 *   RX  v    = low-pass + L-fold decimation of w[m] e^{-j 2 pi (...) m / fs}
 *       x[n] = 2 Re{ v[n] e^{+j pi n/2} }
 *
 * which returns x exactly (to the filters) for a pure delay channel; a delay
 * also rotates the carrier, which the modem's own carrier recovery absorbs.
 * Adjacent slots are only ~160 Hz apart at the roll-off edges, so the one
 * low-pass prototype is long (Kaiser, see FDM_TAPS / FDM_BETA).
 *
 * The wideband stream is quantised to 16 bits after scaling the composite to
 * FDM_LEVEL_DBFS, so the 16-bit sound-card floor is in the loop.
 *
 * Usage: v34_fdm_test [baud] [bps]
 * Environment:
 *   FDM_RATE=96000        wideband sample rate (multiple of 8000)
 *   FDM_SLOTS=N           slots to populate (default: all, fs/8000)
 *   FDM_FIRST_SLOT=k      first populated slot (default 0)
 *   FDM_LEVEL_DBFS=-20    composite RMS target on the 16-bit stream
 *   FDM_NOISE_DBFS=x      white noise on the wideband stream (off by default)
 *   FDM_SLOT_SNR=a,b,...  per-slot in-band noise, dB under that slot's own
 *                         signal (models a frequency-dependent noise floor)
 *   FDM_STAGGER_MS=t      start slot s at s*t ms (default 37).  Started in
 *                         lockstep, all slots send the same training tones at
 *                         the same instant and the composite peaks coherently
 *                         (22 dB crest, clipping); a stagger decorrelates them.
 *   FDM_DELAY=n           one-way bulk delay in WIDEBAND samples (a fractional
 *                         delay at 8 kHz)
 *   FDM_TAPS=P            prototype taps per 8 kHz sample (default 96; the
 *                         slots stay apart at 96, and the total group delay
 *                         it sets moves which marginal rows acquire, as
 *                         V34_DUPLEX_DELAY does -- score over FDM_DELAY)
 *   FDM_BETA=b            Kaiser beta (default 7)
 *   FDM_BLOCKS=n          20 ms blocks to run (default 3000 = 60 s)
 *   FDM_PAYLOAD=n         payload bits each direction of each slot (16000)
 *   FDM_LOG=k             SpanDSP flow log for slot k's two modems (stderr)
 *   FDM_TAP=<path>        raw int16 wideband streams, <path>.c2a / <path>.a2c
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include <spandsp.h>

#define BLOCK_8K 160
#define MAX_SLOTS 32
#define SLOT_HZ 4000
#define SHIFT_HZ 2000          /* e^{-j pi n/2} at 8 kHz */

/* ------------------------------------------------------------------------ */
/* Payload endpoints: identical pattern and sync to v34_duplex_test.        */

typedef struct {
    uint32_t tx_lfsr;
    uint32_t expected_lfsr;
    int tx_bits;
    int rx_bits;
    int bit_errors;
    uint32_t sync_window;
    uint32_t sync_target;
    int sync_bits;
    bool payload_synced;
    bool trained;
    bool failed;
    int done_block;
} endpoint_t;

static int pattern_bit(uint32_t *state)
{
    int bit = (int)(*state & 1U);
    uint32_t feedback = ((*state >> 0) ^ (*state >> 2)
                       ^ (*state >> 3) ^ (*state >> 5)) & 1U;
    *state = (*state >> 1) | (feedback << 15);
    return bit;
}

static int get_bit(void *user_data)
{
    endpoint_t *ep = (endpoint_t *)user_data;
    ep->tx_bits++;
    return pattern_bit(&ep->tx_lfsr);
}

static void put_bit(void *user_data, int bit)
{
    endpoint_t *ep = (endpoint_t *)user_data;

    if (bit < 0) {
        if (bit == SIG_STATUS_TRAINING_SUCCEEDED)
            ep->trained = true;
        else if (bit == SIG_STATUS_TRAINING_FAILED
                 || bit == SIG_STATUS_CARRIER_DOWN)
            ep->failed = true;
        return;
    }
    if (!ep->payload_synced) {
        ep->sync_window = (ep->sync_window << 1) | (uint32_t)(bit & 1);
        ep->sync_bits++;
        if (ep->sync_bits >= 32 && ep->sync_window == ep->sync_target) {
            for (int i = 0; i < 32; i++)
                (void)pattern_bit(&ep->expected_lfsr);
            ep->payload_synced = true;
            ep->rx_bits = 32;
        }
        return;
    }
    if (bit != pattern_bit(&ep->expected_lfsr))
        ep->bit_errors++;
    ep->rx_bits++;
}

static void endpoint_init(endpoint_t *ep, uint32_t tx_seed, uint32_t rx_seed)
{
    uint32_t s = rx_seed;

    memset(ep, 0, sizeof(*ep));
    ep->tx_lfsr = tx_seed;
    ep->expected_lfsr = rx_seed;
    ep->done_block = -1;
    for (int i = 0; i < 32; i++)
        ep->sync_target = (ep->sync_target << 1) | (uint32_t)pattern_bit(&s);
}

/* ------------------------------------------------------------------------ */
/* Channel bank.                                                            */

typedef struct { float re, im; } cf_t;

static int g_fs;            /* wideband rate */
static int g_l;             /* fs / 8000 */
static int g_p;             /* prototype taps per 8 kHz sample */
static int g_ntaps;         /* g_l * g_p */
static float *g_h;          /* low-pass prototype at fs, unity DC gain */
static float *g_cos;        /* cos(2 pi i / fs), i < fs */
static float *g_sin;

static double bessel_i0(double x)
{
    double sum = 1.0, term = 1.0;

    for (int k = 1; k < 64; k++) {
        term *= (x/(2.0*k))*(x/(2.0*k));
        sum += term;
        if (term < sum*1e-12)
            break;
    }
    return sum;
}

static void design_prototype(double beta)
{
    /* Passband to ~1880 Hz, adjacent slot's roll-off edge at ~2040 Hz:
       centre the cutoff in that gap. */
    double fc = 1960.0/g_fs;
    double mid = (g_ntaps - 1)/2.0;
    double i0b = bessel_i0(beta);
    double sum = 0.0;

    g_h = calloc((size_t)g_ntaps, sizeof(float));
    for (int i = 0; i < g_ntaps; i++) {
        double t = i - mid;
        double sinc = (t == 0.0) ? 2.0*fc : sin(2.0*M_PI*fc*t)/(M_PI*t);
        double r = t/mid;
        double w = bessel_i0(beta*sqrt(fmax(0.0, 1.0 - r*r)))/i0b;

        g_h[i] = (float)(sinc*w);
        sum += g_h[i];
    }
    for (int i = 0; i < g_ntaps; i++)
        g_h[i] = (float)(g_h[i]/sum);

    g_cos = malloc((size_t)g_fs*sizeof(float));
    g_sin = malloc((size_t)g_fs*sizeof(float));
    for (int i = 0; i < g_fs; i++) {
        g_cos[i] = (float)cos(2.0*M_PI*i/g_fs);
        g_sin[i] = (float)sin(2.0*M_PI*i/g_fs);
    }
}

/* One slot's modulator: 8 kHz real in, wideband real added into out[]. */
typedef struct {
    int carrier_hz;     /* 4000k + 2000 */
    int phase;          /* carrier phase index, mod fs */
    int n8;             /* 8 kHz sample counter, for e^{-j pi n/2} */
    cf_t *hist;         /* last g_p baseband samples, newest at [0] */
} slot_tx_t;

/* One slot's demodulator: wideband real in, 8 kHz real out. */
typedef struct {
    int carrier_hz;
    int phase;
    int n8;
    int fill;           /* wideband samples since the last 8 kHz output */
    int pos;            /* ring write position */
    cf_t *ring;         /* 2*g_ntaps, mirrored for contiguous reads */
} slot_rx_t;

static void slot_tx_init(slot_tx_t *t, int slot)
{
    t->carrier_hz = slot*SLOT_HZ + SHIFT_HZ;
    t->phase = 0;
    t->n8 = 0;
    t->hist = calloc((size_t)g_p, sizeof(cf_t));
}

static void slot_rx_init(slot_rx_t *r, int slot)
{
    r->carrier_hz = slot*SLOT_HZ + SHIFT_HZ;
    r->phase = 0;
    r->n8 = 0;
    r->fill = 0;
    r->pos = 0;
    r->ring = calloc((size_t)2*g_ntaps, sizeof(cf_t));
}

static void slot_tx_run(slot_tx_t *t, const float *x, int n, float *out)
{
    /* e^{-j pi n/2}: 1, -j, -1, +j */
    static const float qr[4] = { 1.0f, 0.0f, -1.0f, 0.0f };
    static const float qi[4] = { 0.0f, -1.0f, 0.0f, 1.0f };

    for (int i = 0; i < n; i++) {
        int q = t->n8++ & 3;

        memmove(t->hist + 1, t->hist, (size_t)(g_p - 1)*sizeof(cf_t));
        t->hist[0].re = x[i]*qr[q];
        t->hist[0].im = x[i]*qi[q];
        for (int ph = 0; ph < g_l; ph++) {
            float ur = 0.0f, ui = 0.0f;
            const float *h = g_h + ph;

            for (int k = 0; k < g_p; k++) {
                ur += h[k*g_l]*t->hist[k].re;
                ui += h[k*g_l]*t->hist[k].im;
            }
            /* gain L restores the interpolated amplitude; x2 for the real
               part of a one-sided signal */
            ur *= (float)g_l;
            ui *= (float)g_l;
            *out++ += 2.0f*(ur*g_cos[t->phase] - ui*g_sin[t->phase]);
            t->phase += t->carrier_hz;
            if (t->phase >= g_fs)
                t->phase -= g_fs;
        }
    }
}

/* Consumes n wideband samples, writes up to n/L 8 kHz samples; returns count. */
static int slot_rx_run(slot_rx_t *r, const float *w, int n, float *x)
{
    static const float qr[4] = { 1.0f, 0.0f, -1.0f, 0.0f };
    static const float qi[4] = { 0.0f, 1.0f, 0.0f, -1.0f };   /* e^{+j pi n/2} */
    int produced = 0;

    for (int i = 0; i < n; i++) {
        cf_t s;

        s.re = w[i]*g_cos[r->phase];
        s.im = -w[i]*g_sin[r->phase];
        r->phase += r->carrier_hz;
        if (r->phase >= g_fs)
            r->phase -= g_fs;
        r->ring[r->pos] = s;
        r->ring[r->pos + g_ntaps] = s;
        if (++r->pos >= g_ntaps)
            r->pos = 0;
        if (++r->fill < g_l)
            continue;
        r->fill = 0;
        {
            /* ring[pos .. pos+ntaps-1] is oldest..newest */
            const cf_t *c = r->ring + r->pos;
            float vr = 0.0f, vi = 0.0f;
            int q = r->n8++ & 3;

            for (int k = 0; k < g_ntaps; k++) {
                float hk = g_h[g_ntaps - 1 - k];
                vr += hk*c[k].re;
                vi += hk*c[k].im;
            }
            x[produced++] = 2.0f*(vr*qr[q] - vi*qi[q]);
        }
    }
    return produced;
}

/* ------------------------------------------------------------------------ */

static double gauss(void)
{
    double u1 = (rand() + 1.0)/(RAND_MAX + 2.0);
    double u2 = (rand() + 1.0)/(RAND_MAX + 2.0);

    return sqrt(-2.0*log(u1))*cos(2.0*M_PI*u2);
}

typedef struct {
    endpoint_t caller, answer;
    v34_state_t *call_modem, *answer_modem;
    slot_tx_t tx_c2a, tx_a2c;
    slot_rx_t rx_c2a, rx_a2c;
    float snr_db;          /* in-band noise; <= 0 = none */
    double pow_c, pow_a;   /* running signal power, for snr_db */
    long npow;
    float rx_pending_c[BLOCK_8K*2], rx_pending_a[BLOCK_8K*2];
    int n_pending_c, n_pending_a;
} slot_t;

static int16_t sat16(float v)
{
    if (v > 32767.0f) return 32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)lrintf(v);
}

int main(int argc, char *argv[])
{
    int baud = argc > 1 ? atoi(argv[1]) : 3200;
    int bps = argc > 2 ? atoi(argv[2]) : 21600;
    int nslots, first;
    int max_blocks = getenv("FDM_BLOCKS") ? atoi(getenv("FDM_BLOCKS")) : 3000;
    int payload = getenv("FDM_PAYLOAD") ? atoi(getenv("FDM_PAYLOAD")) : 16000;
    float level_dbfs = getenv("FDM_LEVEL_DBFS") ? strtof(getenv("FDM_LEVEL_DBFS"), NULL) : -20.0f;
    float noise_dbfs = getenv("FDM_NOISE_DBFS") ? strtof(getenv("FDM_NOISE_DBFS"), NULL) : 0.0f;
    int delay = getenv("FDM_DELAY") ? atoi(getenv("FDM_DELAY")) : 0;
    double beta = getenv("FDM_BETA") ? strtod(getenv("FDM_BETA"), NULL) : 7.0;
    slot_t *slots;
    float *wide[2], *dline[2];
    int dpos = 0;
    int stagger_blocks = (getenv("FDM_STAGGER_MS") ? atoi(getenv("FDM_STAGGER_MS")) : 37)/20;
    int start_block[MAX_SLOTS];
    FILE *tap[2] = { NULL, NULL };
    float gain;
    long clipped = 0;
    int peak = 0;
    double wide_pow = 0.0;
    long wide_n = 0;
    int completed_block = -1;
    clock_t t0 = clock();
    int pass;

    g_fs = getenv("FDM_RATE") ? atoi(getenv("FDM_RATE")) : 96000;
    if (g_fs < 8000 || g_fs % 8000) {
        fprintf(stderr, "FDM_RATE must be a multiple of 8000\n");
        return 2;
    }
    g_l = g_fs/8000;
    g_p = getenv("FDM_TAPS") ? atoi(getenv("FDM_TAPS")) : 96;
    if (g_p < 8) g_p = 8;
    g_ntaps = g_l*g_p;
    first = getenv("FDM_FIRST_SLOT") ? atoi(getenv("FDM_FIRST_SLOT")) : 0;
    nslots = getenv("FDM_SLOTS") ? atoi(getenv("FDM_SLOTS")) : g_fs/(2*SLOT_HZ) - first;
    if (first < 0 || nslots < 1 || first + nslots > g_fs/(2*SLOT_HZ) || nslots > MAX_SLOTS) {
        fprintf(stderr, "slots %d..%d do not fit below %d Hz\n",
                first, first + nslots - 1, g_fs/2);
        return 2;
    }
    if (delay < 0) delay = 0;
    design_prototype(beta);

    /* V.34 transmits at -12 dBm0, about RMS 4000 on this int16 scale; scale
       each slot so the N-slot composite sits at FDM_LEVEL_DBFS. */
    gain = (float)(32768.0*pow(10.0, level_dbfs/20.0)/(4000.0*sqrt((double)nslots)));

    slots = calloc((size_t)nslots, sizeof(slot_t));
    {
        const char *snr = getenv("FDM_SLOT_SNR");
        for (int s = 0; s < nslots; s++) {
            slot_t *sl = &slots[s];
            int k = first + s;

            endpoint_init(&sl->caller, 0xACE1U + 17U*s, 0x1D0FU + 31U*s);
            endpoint_init(&sl->answer, 0x1D0FU + 31U*s, 0xACE1U + 17U*s);
            sl->call_modem = v34_init(NULL, baud, bps, true, true,
                                      get_bit, &sl->caller, put_bit, &sl->caller);
            sl->answer_modem = v34_init(NULL, baud, bps, false, true,
                                        get_bit, &sl->answer, put_bit, &sl->answer);
            if (!sl->call_modem || !sl->answer_modem) {
                fprintf(stderr, "v34_init failed (slot %d)\n", k);
                return 1;
            }
            v34_tx_power(sl->call_modem, -12.0f);
            v34_tx_power(sl->answer_modem, -12.0f);
            if (getenv("FDM_LOG") && atoi(getenv("FDM_LOG")) == k) {
                int lv = SPAN_LOG_SHOW_SEVERITY | SPAN_LOG_SHOW_PROTOCOL
                       | SPAN_LOG_SHOW_TAG | SPAN_LOG_SHOW_SAMPLE_TIME | SPAN_LOG_FLOW;
                span_log_set_level(v34_get_logging_state(sl->call_modem), lv);
                span_log_set_tag(v34_get_logging_state(sl->call_modem), "caller");
                span_log_set_level(v34_get_logging_state(sl->answer_modem), lv);
                span_log_set_tag(v34_get_logging_state(sl->answer_modem), "answer");
            }
            start_block[s] = s*stagger_blocks;
            slot_tx_init(&sl->tx_c2a, k);
            slot_tx_init(&sl->tx_a2c, k);
            slot_rx_init(&sl->rx_c2a, k);
            slot_rx_init(&sl->rx_a2c, k);
            if (snr) {
                sl->snr_db = strtof(snr, NULL);
                snr = strchr(snr, ',');
                if (snr) snr++;
            }
        }
    }
    for (int d = 0; d < 2; d++) {
        wide[d] = malloc((size_t)BLOCK_8K*g_l*sizeof(float));
        dline[d] = calloc((size_t)(delay + 1), sizeof(float));
    }
    if (getenv("FDM_TAP")) {
        char q[1024];
        snprintf(q, sizeof(q), "%s.c2a", getenv("FDM_TAP"));
        tap[0] = fopen(q, "wb");
        snprintf(q, sizeof(q), "%s.a2c", getenv("FDM_TAP"));
        tap[1] = fopen(q, "wb");
    }

    printf("V.34 FDM: fs=%d, slots %d..%d (%d), %d baud/%d bps per slot, "
           "prototype %d taps (beta %.1f), composite %.1f dBFS%s\n",
           g_fs, first, first + nslots - 1, nslots, baud, bps, g_ntaps, beta,
           level_dbfs, delay ? ", delayed" : "");
    fflush(stdout);

    srand(12345);
    for (int block = 0; block < max_blocks; block++) {
        int nw = BLOCK_8K*g_l;

        memset(wide[0], 0, (size_t)nw*sizeof(float));
        memset(wide[1], 0, (size_t)nw*sizeof(float));

        /* Modulate every slot's two directions into the two wide streams. */
        for (int s = 0; s < nslots; s++) {
            slot_t *sl = &slots[s];
            int16_t ct[BLOCK_8K], at[BLOCK_8K];
            float cf[BLOCK_8K], af[BLOCK_8K];
            int cn = 0, an = 0;

            if (block >= start_block[s]) {
                cn = v34_tx(sl->call_modem, ct, BLOCK_8K);
                an = v34_tx(sl->answer_modem, at, BLOCK_8K);
            }

            if (cn < BLOCK_8K) memset(ct + cn, 0, (size_t)(BLOCK_8K - cn)*sizeof(int16_t));
            if (an < BLOCK_8K) memset(at + an, 0, (size_t)(BLOCK_8K - an)*sizeof(int16_t));
            for (int i = 0; i < BLOCK_8K; i++) {
                sl->pow_c += (double)ct[i]*ct[i];
                sl->pow_a += (double)at[i]*at[i];
            }
            sl->npow += BLOCK_8K;
            for (int i = 0; i < BLOCK_8K; i++) {
                cf[i] = ct[i];
                af[i] = at[i];
                if (sl->snr_db > 0.0f) {
                    /* in-band noise: added at 8 kHz so the mux confines it
                       to this slot, i.e. a frequency-dependent floor */
                    cf[i] += (float)(gauss()*sqrt(sl->pow_c/sl->npow)*pow(10.0, -sl->snr_db/20.0));
                    af[i] += (float)(gauss()*sqrt(sl->pow_a/sl->npow)*pow(10.0, -sl->snr_db/20.0));
                }
                cf[i] *= gain;
                af[i] *= gain;
            }
            slot_tx_run(&sl->tx_c2a, cf, BLOCK_8K, wide[0]);
            slot_tx_run(&sl->tx_a2c, af, BLOCK_8K, wide[1]);
        }

        /* The wideband bearer: noise, 16-bit quantisation, bulk delay. */
        for (int d = 0; d < 2; d++) {
            int16_t q16[BLOCK_8K*24];

            for (int i = 0; i < nw; i++) {
                float v = wide[d][i];
                int16_t q;

                if (noise_dbfs < 0.0f)
                    v += (float)(gauss()*32768.0*pow(10.0, noise_dbfs/20.0));
                q = sat16(v);
                if (q == 32767 || q == -32768) clipped++;
                if (abs(q) > peak) peak = abs(q);
                wide_pow += (double)q*q;
                wide_n++;
                if (tap[d] && i < (int)(sizeof(q16)/sizeof(q16[0])))
                    q16[i] = q;
                v = (float)q;
                if (delay > 0) {
                    float out = dline[d][(dpos + i) % (delay + 1)];
                    dline[d][(dpos + i) % (delay + 1)] = v;
                    v = out;
                }
                wide[d][i] = v/gain;
            }
            if (tap[d])
                fwrite(q16, sizeof(int16_t), (size_t)nw, tap[d]);
        }
        if (delay > 0)
            dpos = (dpos + nw) % (delay + 1);

        /* Demodulate every slot and feed the receivers. */
        for (int s = 0; s < nslots; s++) {
            slot_t *sl = &slots[s];
            float xf[BLOCK_8K + 4];
            int16_t x16[BLOCK_8K + 4];
            int n;

            n = slot_rx_run(&sl->rx_c2a, wide[0], nw, xf);
            for (int i = 0; i < n; i++) x16[i] = sat16(xf[i]);
            if (block >= start_block[s])
                (void)v34_rx(sl->answer_modem, x16, n);
            n = slot_rx_run(&sl->rx_a2c, wide[1], nw, xf);
            for (int i = 0; i < n; i++) x16[i] = sat16(xf[i]);
            if (block >= start_block[s]) {
                (void)v34_rx(sl->call_modem, x16, n);
                span_log_bump_samples(v34_get_logging_state(sl->call_modem), n);
                span_log_bump_samples(v34_get_logging_state(sl->answer_modem), n);
            }

            if (sl->caller.done_block < 0 && sl->caller.trained && sl->answer.trained
                && sl->caller.rx_bits >= payload && sl->answer.rx_bits >= payload)
                sl->caller.done_block = block;
        }

        {
            bool all = true;
            for (int s = 0; s < nslots && all; s++)
                all = slots[s].caller.done_block >= 0 || slots[s].caller.failed
                   || slots[s].answer.failed;
            if (all) {
                completed_block = block;
                break;
            }
        }
    }

    {
        double secs = (completed_block >= 0 ? completed_block + 1 : max_blocks)*0.020;
        double cpu = (double)(clock() - t0)/CLOCKS_PER_SEC;
        long agg = 0;

        pass = 1;
        printf("slot  band(Hz)        trained  a2c    c2a    rx_bits(c/a)     errors(c/a)  done(s, from its start)\n");
        for (int s = 0; s < nslots; s++) {
            slot_t *sl = &slots[s];
            int k = first + s;
            int a2c = 0, c2a = 0;
            bool ok;

            if (v34_get_negotiated_mp_rates(sl->call_modem, &a2c, &c2a) != 0)
                a2c = c2a = 0;
            ok = sl->caller.done_block >= 0
              && sl->caller.bit_errors == 0 && sl->answer.bit_errors == 0;
            if (ok)
                agg += (long)(a2c + c2a)*2400;
            pass &= ok;
            {
                char done[16] = "--";
                if (sl->caller.done_block >= 0)
                    snprintf(done, sizeof(done), "%.2f",
                             (sl->caller.done_block + 1 - start_block[s])*0.020);
                printf("%3d   %5d-%-5d     %d/%d    %-6d %-6d %7d/%-7d %5d/%-5d   %s\n",
                       k, k*SLOT_HZ, k*SLOT_HZ + SLOT_HZ, sl->caller.trained, sl->answer.trained,
                       a2c*2400, c2a*2400, sl->caller.rx_bits, sl->answer.rx_bits,
                       sl->caller.bit_errors, sl->answer.bit_errors, done);
            }
        }
        {
            double rms = sqrt(wide_pow/(wide_n ? wide_n : 1));
            printf("wideband: rms %.1f dBFS, peak %d (crest %.1f dB), clipped %ld samples\n",
                   20.0*log10(rms/32768.0), peak, 20.0*log10(peak/(rms > 0 ? rms : 1)), clipped);
        }
        printf("aggregate (passing slots, both directions summed): %ld bit/s; "
               "simulated %.1f s in %.1f s CPU\n", agg, secs, cpu);
        printf("%s\n", pass ? "PASS" : "FAIL");
    }

    for (int s = 0; s < nslots; s++) {
        v34_free(slots[s].call_modem);
        v34_free(slots[s].answer_modem);
    }
    for (int d = 0; d < 2; d++)
        if (tap[d]) fclose(tap[d]);
    return pass ? 0 : 1;
}
