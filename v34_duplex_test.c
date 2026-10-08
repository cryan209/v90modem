/*
 * In-process full-duplex V.34 waveform harness over a G.711 round trip.
 *
 * This deliberately connects two independent modem instances through only
 * the bearer transformation.  No internal symbols or INFO/MP state cross the
 * seam.  ITU-T V.34 (10/96) clauses 10.1 and 11 must therefore complete from
 * the waveform in both directions before payload bits can pass.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <spandsp.h>

#include "v34_line_ec.h"

#define BLOCK_SAMPLES 160
#define MAX_BLOCKS 3000       /* 60 seconds at 8 kHz */
#define PAYLOAD_BITS 16000
/* Payload required on the far side of a rate renegotiation.  Enough to be a
   real data mode rather than a lucky frame, short enough that a run that
   renegotiates still fits the harness's block budget. */
#define POST_RENEG_BITS 8000
/* Bits the re-derived LFSR position must predict before it is believed, and
   how many times to slide forward and try again. */
#define RESYNC_VALIDATE_BITS 64
#define RESYNC_MAX_RESTARTS 200

typedef struct endpoint_s {
    const char *name;
    uint32_t tx_lfsr;
    uint32_t expected_lfsr;
    int tx_bits;
    int rx_bits;
    int bit_errors;
    uint32_t sync_window;
    uint32_t sync_target;
    int sync_bits;
    int skipped_bits;
    bool payload_synced;
    int clipped_samples;
    int16_t peak_sample;
    bool trained;
    bool failed;
    /* V.34 11.6 rate renegotiation exercise -- see reneg_* below. */
    bool resyncing;
    int resync_bits;
    uint32_t resync_state;
    int resync_validate;
    int resync_bad;
    int resync_restarts;
    int post_reneg_bits;
    int post_reneg_errors;
} endpoint_t;

/* The pattern is a 16-bit LFSR whose output is the successive low bit of its
   state, so sixteen consecutive output bits ARE the state that produced the
   first of them: b_i is bit i.  That is what lets the payload check resume
   after a rate renegotiation without any state crossing the seam -- the
   receiver re-derives the transmitter's position from the bits it actually
   demodulated, and every bit after that is checked against the recurrence.
   A receiver that came back from 11.6 producing anything but the peer's real
   stream fails within a few bits. */
static void endpoint_begin_resync(endpoint_t *ep)
{
    ep->resyncing = true;
    ep->resync_bits = 0;
    ep->resync_state = 0;
    ep->resync_validate = 0;
    ep->resync_bad = 0;
    ep->resync_restarts = 0;
}

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
    if (ep->resyncing) {
        /* Sixteen bits give the state, but only if they are really payload:
           a couple of stragglers from the seam would derive a state that is
           wrong for everything after it, which looks exactly like a modem
           producing white output.  So derive, then VALIDATE against the next
           RESYNC_VALIDATE_BITS, and start over from here if it does not hold.
           The harness must be able to tell its own mis-anchoring apart from
           a receiver that came back broken. */
        if (ep->resync_bits < 16) {
            ep->resync_state |= (uint32_t)(bit & 1) << ep->resync_bits;
            if (++ep->resync_bits == 16) {
                /* The sixteen bits give the state that produced the FIRST of
                   them, and all sixteen have now been consumed -- so advance
                   past them before predicting anything, exactly as the
                   initial sync above does after matching its 32-bit target.
                   Without this the check is off by sixteen bits and reads as
                   a receiver producing white output. */
                ep->expected_lfsr = ep->resync_state;
                for (int i = 0; i < 16; i++)
                    (void)pattern_bit(&ep->expected_lfsr);
            }
            return;
        }
        if (bit != pattern_bit(&ep->expected_lfsr))
            ep->resync_bad++;
        if (++ep->resync_validate >= RESYNC_VALIDATE_BITS) {
            if (ep->resync_bad == 0) {
                ep->resyncing = false;
            } else if (++ep->resync_restarts <= RESYNC_MAX_RESTARTS) {
                ep->resync_bits = 0;
                ep->resync_state = 0;
                ep->resync_validate = 0;
                ep->resync_bad = 0;
            } else {
                /* Out of attempts: stop resyncing and let the errors be
                   counted, which is the honest outcome for a stream that
                   never became the peer's pattern again. */
                ep->resyncing = false;
            }
        }
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
        } else {
            ep->skipped_bits++;
        }
        return;
    }
    if (bit != pattern_bit(&ep->expected_lfsr)) {
        ep->bit_errors++;
        ep->post_reneg_errors++;
    }
    ep->rx_bits++;
    ep->post_reneg_bits++;
}

/* V34_DUPLEX_NOISE_DB adds white Gaussian noise at the stated SNR, in dB below
   the running signal power.  The harness had no way to put a known impairment
   on the bearer, so nothing could say how much of a symbol-level residual came
   from the bearer and how much the receiver added to it. */
static int16_t add_noise(int16_t sample)
{
    static float snr_db = -1.0f;
    static double sp;
    static long n;
    double g;
    int i;
    double v;

    if (snr_db < 0.0f)
    {
        const char *value = getenv("V34_DUPLEX_NOISE_DB");

        snr_db = (value && *value) ? (float) atof(value) : 0.0f;
    }
    if (snr_db <= 0.0f)
        return sample;
    sp += (double) sample*sample;
    n++;
    g = 0.0;
    for (i = 0; i < 12; i++)
        g += (double) rand()/RAND_MAX - 0.5;
    v = sample + g*sqrt(sp/n)*pow(10.0, -snr_db/20.0);
    if (v > 32000.0) v = 32000.0;
    if (v < -32000.0) v = -32000.0;
    return (int16_t) v;
}

/* V34_DUPLEX_DELAY=<samples> puts the same one-way bulk delay in both
   directions.  Acquisition here is sensitive to where symbol boundaries fall
   against the 8 kHz grid, so a single run of a marginal row is one draw of a
   coin; sweeping the delay turns a row into a pass rate. */
#define CHANNEL_DELAY_MAX 4096
static int16_t channel_delay(int dir, int16_t sample)
{
    static int delay = -1;
    static int16_t line[2][CHANNEL_DELAY_MAX];
    static int pos[2];
    int16_t out;

    if (delay < 0) {
        const char *value = getenv("V34_DUPLEX_DELAY");
        delay = (value && *value) ? atoi(value) : 0;
        if (delay < 0) delay = 0;
        if (delay >= CHANNEL_DELAY_MAX) delay = CHANNEL_DELAY_MAX - 1;
    }
    if (delay == 0)
        return sample;
    out = line[dir][pos[dir]];
    line[dir][pos[dir]] = sample;
    if (++pos[dir] >= delay)
        pos[dir] = 0;
    return out;
}

/* Offline clock-error bearer.  Resample only the synthetic analogue waveform,
   before G.711, never the production DS0 path.  A 256-sample look-ahead keeps
   the window available throughout this harness's bounded 60-second run at
   +/-200 ppm.  V34_DUPLEX_PPM=0 exercises the same FIR without clock drift. */
static int16_t channel_clock(int dir, int16_t sample)
{
    enum { SIZE = 4096, PHASES = 4096, HALF = 32 };
    static int initialized, enabled;
    static float taps[PHASES][2*HALF + 1];
    static int16_t history[2][SIZE];
    static int64_t written[2];
    static double position[2] = {-256.0, -256.0};
    static double ratio;
    double out = 0.0;
    int64_t centre;
    int phase;

    if (!initialized)
    {
        const char *value = getenv("V34_DUPLEX_PPM");
        enabled = value != NULL;
        ratio = 1.0 + (value ? strtod(value, NULL) : 0.0)*1e-6;
        if (!isfinite(ratio) || ratio < 0.9998 || ratio > 1.0002)
        {
            fprintf(stderr, "V34_DUPLEX_PPM must be -200..200\n");
            exit(2);
        }
        for (int p = 0; enabled && p < PHASES; p++)
        {
            double sum = 0.0;
            for (int k = -HALF; k <= HALF; k++)
            {
                double x = k - (double)p/PHASES;
                double sinc = fabs(x) < 1e-12 ? 1.0 : sin(M_PI*x)/(M_PI*x);
                double window = 0.42 + 0.5*cos(M_PI*x/HALF)
                              + 0.08*cos(2.0*M_PI*x/HALF);
                taps[p][k + HALF] = fabs(x) <= HALF ? sinc*window : 0.0;
                sum += taps[p][k + HALF];
            }
            for (int k = 0; k <= 2*HALF; k++) taps[p][k] /= sum;
        }
        initialized = 1;
    }
    if (!enabled) return sample;
    history[dir][written[dir]++ & (SIZE - 1)] = sample;
    centre = (int64_t)floor(position[dir]);
    phase = (int)((position[dir] - centre)*PHASES);
    for (int k = -HALF; k <= HALF; k++)
        out += taps[phase][k + HALF]*history[dir][(centre + k) & (SIZE - 1)];
    position[dir] += ratio;
    return (int16_t)lrint(fmax(-32768.0, fmin(32767.0, out)));
}

/* Near-end hybrid echo.  V34_DUPLEX_ECHO_DB=<return loss> returns each
   side's own transmission to its own receiver through a three-tap hybrid
   (three taps, so a canceller cannot pass by being a pure delay and gain)
   after V34_DUPLEX_ECHO_DELAY samples of bulk delay (default 2136, the
   267 ms measured on the VG224 path to the RasFinder).  It is added before
   the codec, as the far ATA's hybrid does.  V34_DUPLEX_ECHO_SIDE=caller or
   answer restricts it to one side.  The engine's line echo canceller
   (v34_line_ec.c) runs in front of each receiver exactly as modem_engine.c
   runs it; V34_DUPLEX_LEC=0 removes it. */
#define ECHO_RING 8192
typedef struct {
    float gain;
    int delay;
    bool on;
    int16_t ring[ECHO_RING];
    uint32_t pos;
} echo_path_t;
static const float hybrid_taps[3] = {0.80f, -0.45f, 0.25f};
static echo_path_t echo_path[2];   /* [0] answer side, [1] caller side */

static void echo_init(void)
{
    const char *db = getenv("V34_DUPLEX_ECHO_DB");
    const char *dl = getenv("V34_DUPLEX_ECHO_DELAY");
    const char *side = getenv("V34_DUPLEX_ECHO_SIDE");
    float p = 0.0f;

    for (int i = 0; i < 3; i++)
        p += hybrid_taps[i]*hybrid_taps[i];
    for (int d = 0; d < 2; d++) {
        memset(&echo_path[d], 0, sizeof(echo_path[d]));
        if (!db || !*db)
            continue;
        if (side && strcmp(side, d ? "caller" : "answer") != 0
            && strcmp(side, "both") != 0)
            continue;
        echo_path[d].on = true;
        echo_path[d].gain = powf(10.0f, -strtof(db, NULL)/20.0f)/sqrtf(p);
        echo_path[d].delay = (dl && *dl) ? atoi(dl) : 2136;
        if (echo_path[d].delay < 3) echo_path[d].delay = 3;
        if (echo_path[d].delay > ECHO_RING - 4) echo_path[d].delay = ECHO_RING - 4;
    }
}

/* Feed one of this side's transmitted samples; return the echo arriving now. */
static float echo_step(int d, int16_t tx)
{
    echo_path_t *e = &echo_path[d];
    float v = 0.0f;

    if (!e->on)
        return 0.0f;
    e->ring[e->pos & (ECHO_RING - 1)] = tx;
    for (int k = 0; k < 3; k++)
        v += hybrid_taps[k]*e->ring[(e->pos - (uint32_t) e->delay - (uint32_t) k) & (ECHO_RING - 1)];
    e->pos++;
    return v*e->gain;
}

static int16_t sat16(float v)
{
    if (v > 32767.0f) return 32767;
    if (v < -32768.0f) return -32768;
    return (int16_t) lrintf(v);
}

static int16_t g711_roundtrip(int16_t sample, bool alaw)
{
    sample = add_noise(sample);

    /* V34_DUPLEX_LINEAR bypasses the codec entirely, so a defect can be
       attributed to the bearer or exonerated from it.  Diagnostic only: the
       point of this harness is that the two modems meet through G.711. */
    static int linear = -1;

    if (linear < 0)
        linear = (getenv("V34_DUPLEX_LINEAR") != NULL);
    if (linear)
        return sample;
    {
        int16_t out = alaw ? alaw_to_linear(linear_to_alaw(sample))
                           : ulaw_to_linear(linear_to_ulaw(sample));
        /* What the codec itself costs, measured on the actual waveform rather
           than on an assumed one: V34_DUPLEX_G711_SNR=1 reports it. */
        if (getenv("V34_DUPLEX_G711_SNR")) {
            static double sp, np;
            static long n;
            sp += (double) sample*sample;
            np += (double)(out - sample)*(out - sample);
            if (++n % 400000 == 0)
                fprintf(stderr, "[G711] round-trip SNR %.1f dB over %ld samples\n",
                        10.0*log10(sp/np), n);
        }
        return out;
    }
}

static int run_case(int baud, int bps, bool alaw)
{
    endpoint_t caller = {.name = "caller", .tx_lfsr = 0xACE1U,
                         .expected_lfsr = 0x1D0FU};
    endpoint_t answer = {.name = "answer", .tx_lfsr = 0x1D0FU,
                         .expected_lfsr = 0xACE1U};
    v34_state_t *call_modem;
    v34_state_t *answer_modem;
    int16_t call_tx[BLOCK_SAMPLES];
    int16_t answer_tx[BLOCK_SAMPLES];
    int16_t call_rx[BLOCK_SAMPLES];
    int16_t answer_rx[BLOCK_SAMPLES];
    int completed_block = -1;
    int max_blocks = getenv("V34_DUPLEX_BLOCKS")
                   ? atoi(getenv("V34_DUPLEX_BLOCKS")) : MAX_BLOCKS;
    /* V34_DUPLEX_PAYLOAD lengthens the run: the data mode's own SNR report
       needs 4096 symbols, more than PAYLOAD_BITS reaches at high rates. */
    int payload_bits = getenv("V34_DUPLEX_PAYLOAD")
                     ? atoi(getenv("V34_DUPLEX_PAYLOAD")) : PAYLOAD_BITS;
    /* V.34 11.6 exercise.  V34_DUPLEX_RENEG=<bits> runs normally until both
       directions have carried that many payload bits, then has the CALLER
       initiate a rate renegotiation (11.6.1.1) and leaves the answerer to
       detect its S and respond (11.6.1.2) exactly as the engine would --
       through the detector and the public entry point, not by being told.
       The run then requires POST_RENEG_BITS of error-free payload in both
       directions on the far side of it. */
    int reneg_at = getenv("V34_DUPLEX_RENEG")
                 ? atoi(getenv("V34_DUPLEX_RENEG")) : 0;
    /* V34_DUPLEX_CLEARDOWN=<bits>: the same exercise with the caller opening
       an 11.7 cleardown instead (S, S-bar, MP requesting zero rates).  The
       answerer responds as it does to 11.6; the pass criterion is both ends
       reporting the cleardown complete (MP' sent and received). */
    int cleardown_at = getenv("V34_DUPLEX_CLEARDOWN")
                     ? atoi(getenv("V34_DUPLEX_CLEARDOWN")) : 0;
    bool cleardown_ok = false;
    if (cleardown_at > 0)
        reneg_at = cleardown_at;
    bool reneg_started = false;
    int reneg_block = -1;
    int caller_data_stage = -1;
    int answer_data_stage = -1;
    bool caller_left_data = false;
    bool answer_left_data = false;
    bool caller_resynced = false;
    bool answer_resynced = false;

    static v34_line_ec_t lec[2];
    bool use_lec = !(getenv("V34_DUPLEX_LEC") && strcmp(getenv("V34_DUPLEX_LEC"), "0") == 0);

    echo_init();
    v34_line_ec_reset(&lec[0]);
    v34_line_ec_reset(&lec[1]);
    if (reneg_at > 0) {
        /* The responder's S detector is opt-in, for the reasons in
           docs/retrain_and_resync.md.  This harness is what exercises it. */
        setenv("ME_V90_RENEG_RESPOND", "1", 1);
    }

    {
        uint32_t caller_sync_state = caller.expected_lfsr;
        uint32_t answer_sync_state = answer.expected_lfsr;
        for (int i = 0; i < 32; i++) {
            caller.sync_target = (caller.sync_target << 1)
                               | (uint32_t)pattern_bit(&caller_sync_state);
            answer.sync_target = (answer.sync_target << 1)
                               | (uint32_t)pattern_bit(&answer_sync_state);
        }
    }

    call_modem = v34_init(NULL, baud, bps, true, true,
                          get_bit, &caller, put_bit, &caller);
    answer_modem = v34_init(NULL, baud, bps, false, true,
                            get_bit, &answer, put_bit, &answer);
    if (!call_modem || !answer_modem) {
        fprintf(stderr, "v34_duplex_test: v34_init failed\n");
        if (call_modem) v34_free(call_modem);
        if (answer_modem) v34_free(answer_modem);
        return 1;
    }
    v34_tx_power(call_modem, -12.0f);
    v34_tx_power(answer_modem, -12.0f);
    /* V.250 6.4.1 +MS bounds on either modem's MP, "min_tx,max_tx,min_rx,max_rx"
       in bit/s from that modem's side (v34_set_mp_rate_limits()). */
    {
        const char *names[2] = { "V34_DUPLEX_CALL_LIMITS", "V34_DUPLEX_ANSWER_LIMITS" };
        v34_state_t *modems[2] = { call_modem, answer_modem };

        for (int k = 0; k < 2; k++) {
            const char *v = getenv(names[k]);
            int l[4] = { 0, 0, 0, 0 };

            if (v && sscanf(v, "%d,%d,%d,%d", &l[0], &l[1], &l[2], &l[3]) == 4)
                v34_set_mp_rate_limits(modems[k], l[0], l[1], l[2], l[3]);
        }
    }
    if (getenv("V34_DUPLEX_ROT") || getenv("V34_DUPLEX_CONJ")
        || getenv("V34_DUPLEX_SCALE")) {
        int rotation = getenv("V34_DUPLEX_ROT") ? atoi(getenv("V34_DUPLEX_ROT")) : 0;
        int conjugate = getenv("V34_DUPLEX_CONJ") ? atoi(getenv("V34_DUPLEX_CONJ")) : 0;
        float scale = getenv("V34_DUPLEX_SCALE") ? strtof(getenv("V34_DUPLEX_SCALE"), NULL) : 1.0f;
        v34_set_rx_data_transform(call_modem, scale, rotation, conjugate);
        v34_set_rx_data_transform(answer_modem, scale, rotation, conjugate);
    }
    if (getenv("V34_DUPLEX_LOG")) {
        span_log_set_level(v34_get_logging_state(call_modem),
                           SPAN_LOG_SHOW_SEVERITY | SPAN_LOG_SHOW_PROTOCOL
                         | SPAN_LOG_SHOW_TAG | SPAN_LOG_SHOW_SAMPLE_TIME
                         | SPAN_LOG_FLOW);
        span_log_set_tag(v34_get_logging_state(call_modem), "caller");
        span_log_set_level(v34_get_logging_state(answer_modem),
                           SPAN_LOG_SHOW_SEVERITY | SPAN_LOG_SHOW_PROTOCOL
                         | SPAN_LOG_SHOW_TAG | SPAN_LOG_SHOW_SAMPLE_TIME
                         | SPAN_LOG_FLOW);
        span_log_set_tag(v34_get_logging_state(answer_modem), "answer");
    }

    for (int block = 0; block < max_blocks; block++) {
        int call_n = v34_tx(call_modem, call_tx, BLOCK_SAMPLES);
        int answer_n = v34_tx(answer_modem, answer_tx, BLOCK_SAMPLES);

        if (call_n < BLOCK_SAMPLES) {
            if (getenv("V34_DUPLEX_SHORT_LOG"))
                fprintf(stderr, "[SHORT] block=%d caller n=%d rx_stage=%d tx_stage=%d\n",
                        block, call_n, v34_get_rx_stage(call_modem),
                        v34_get_tx_stage(call_modem));
            memset(call_tx + call_n, 0,
                   (size_t)(BLOCK_SAMPLES - call_n)*sizeof(call_tx[0]));
        }
        if (answer_n < BLOCK_SAMPLES) {
            if (getenv("V34_DUPLEX_SHORT_LOG"))
                fprintf(stderr, "[SHORT] block=%d answer n=%d rx_stage=%d tx_stage=%d\n",
                        block, answer_n, v34_get_rx_stage(answer_modem),
                        v34_get_tx_stage(answer_modem));
            memset(answer_tx + answer_n, 0,
                   (size_t)(BLOCK_SAMPLES - answer_n)*sizeof(answer_tx[0]));
        }
        {
            /* V34_DUPLEX_TX_WAV=<path>: raw 8 kHz int16 of what each side put
               on the line, <path>.caller / <path>.answer, so the transmitter
               can be graded offline without the receiver in the loop. */
            static FILE *wav[2];
            static int wav_init;
            if (!wav_init) {
                const char *p = getenv("V34_DUPLEX_TX_WAV");
                char q[1024];
                wav_init = 1;
                if (p && *p) {
                    snprintf(q, sizeof(q), "%s.caller", p);
                    wav[1] = fopen(q, "wb");
                    snprintf(q, sizeof(q), "%s.answer", p);
                    wav[0] = fopen(q, "wb");
                }
            }
            if (wav[1]) fwrite(call_tx, sizeof(int16_t), BLOCK_SAMPLES, wav[1]);
            if (wav[0]) fwrite(answer_tx, sizeof(int16_t), BLOCK_SAMPLES, wav[0]);
        }
        for (int i = 0; i < BLOCK_SAMPLES; i++) {
            int call_abs = abs(call_tx[i]);
            int answer_abs = abs(answer_tx[i]);
            if (call_abs > caller.peak_sample) caller.peak_sample = (int16_t)call_abs;
            if (answer_abs > answer.peak_sample) answer.peak_sample = (int16_t)answer_abs;
            if (call_abs >= 32760) caller.clipped_samples++;
            if (answer_abs >= 32760) answer.clipped_samples++;
            answer_rx[i] = g711_roundtrip(sat16(channel_delay(0, channel_clock(0, call_tx[i]))
                                                + echo_step(0, answer_tx[i])), alaw);
            call_rx[i] = g711_roundtrip(sat16(channel_delay(1, channel_clock(1, answer_tx[i]))
                                              + echo_step(1, call_tx[i])), alaw);
        }
        if (getenv("V34_DUPLEX_TXRMS")) {
            double ca = 0.0, aa = 0.0;
            int ci = 0, ai = 0;
            for (int i = 0; i < BLOCK_SAMPLES; i++) {
                ca += (double)call_tx[i]*call_tx[i];
                aa += (double)answer_tx[i]*answer_tx[i];
                if (abs(call_tx[i]) > ci) ci = abs(call_tx[i]);
                if (abs(answer_tx[i]) > ai) ai = abs(answer_tx[i]);
            }
            fprintf(stderr, "[TXRMS] t=%.3f caller_rms=%.0f peak=%d answer_rms=%.0f peak=%d\n",
                    block*0.020, sqrt(ca/BLOCK_SAMPLES), ci,
                    sqrt(aa/BLOCK_SAMPLES), ai);
        }
        if (getenv("V34_DUPLEX_RXTX_DUMP")) {
            static FILE *f[4];
            if (!f[0]) {
                char q[1024];
                const char *names[4] = {"answer.rx", "answer.tx", "caller.rx", "caller.tx"};
                for (int k = 0; k < 4; k++) {
                    snprintf(q, sizeof(q), "%s.%s", getenv("V34_DUPLEX_RXTX_DUMP"), names[k]);
                    f[k] = fopen(q, "wb");
                }
            }
            fwrite(answer_rx, 2, BLOCK_SAMPLES, f[0]);
            fwrite(answer_tx, 2, BLOCK_SAMPLES, f[1]);
            fwrite(call_rx, 2, BLOCK_SAMPLES, f[2]);
            fwrite(call_tx, 2, BLOCK_SAMPLES, f[3]);
        }
        if (use_lec) {
            char msg[256];
            static int prev_echo[2] = {-1, -1};
            int now_echo[2] = {v34_rx_line_ec_window(answer_modem),
                               v34_rx_line_ec_window(call_modem)};

            for (int d = 0; d < 2; d++) {
                if (getenv("V34_DUPLEX_LEC_TRACE") && now_echo[d] != prev_echo[d])
                    fprintf(stderr, "[LEC %s] t=%.2f echo_only=%d rx_stage=%d tx_stage=%d\n",
                            d ? "caller" : "answer", block*0.020, now_echo[d],
                            v34_get_rx_stage(d ? call_modem : answer_modem),
                            v34_get_tx_stage(d ? call_modem : answer_modem));
                prev_echo[d] = now_echo[d];
            }

            v34_line_ec_tx(&lec[0], answer_tx, BLOCK_SAMPLES);
            v34_line_ec_tx(&lec[1], call_tx, BLOCK_SAMPLES);
            if (v34_line_ec_rx(&lec[0], answer_rx, BLOCK_SAMPLES,
                               v34_rx_line_ec_window(answer_modem),
                               msg, sizeof(msg)) && msg[0])
                fprintf(stderr, "[LEC answer] t=%.2f %s\n", block*0.020, msg);
            if (v34_line_ec_rx(&lec[1], call_rx, BLOCK_SAMPLES,
                               v34_rx_line_ec_window(call_modem),
                               msg, sizeof(msg)) && msg[0])
                fprintf(stderr, "[LEC caller] t=%.2f %s\n", block*0.020, msg);
        }
        (void)v34_rx(answer_modem, answer_rx, BLOCK_SAMPLES);
        (void)v34_rx(call_modem, call_rx, BLOCK_SAMPLES);
        span_log_bump_samples(v34_get_logging_state(call_modem), BLOCK_SAMPLES);
        span_log_bump_samples(v34_get_logging_state(answer_modem), BLOCK_SAMPLES);

        if (caller.failed || answer.failed)
            break;

        if (reneg_at > 0) {
            int caller_stage = v34_get_rx_stage(call_modem);
            int answer_stage = v34_get_rx_stage(answer_modem);

            if (!reneg_started) {
                /* Learn what "in data mode" reads as rather than hardcoding
                   the enum value in a test that only sees the public API. */
                if (caller.payload_synced && caller_data_stage < 0)
                    caller_data_stage = caller_stage;
                if (answer.payload_synced && answer_data_stage < 0)
                    answer_data_stage = answer_stage;
                if (caller.rx_bits >= reneg_at && answer.rx_bits >= reneg_at
                    && caller_data_stage >= 0 && answer_data_stage >= 0) {
                    if ((cleardown_at > 0 ? v34_start_cleardown(call_modem)
                                          : v34_start_rate_renegotiation(call_modem)) == 0) {
                        reneg_started = true;
                        reneg_block = block;
                        fprintf(stderr,
                                "[RENEG] block=%d caller initiated 11.6 after "
                                "%d/%d payload bits\n",
                                block, caller.rx_bits, answer.rx_bits);
                    }
                }
            } else {
                /* 11.6.1.2: the answerer responds off its own detection of
                   the caller's S.  V34_EVENT_PEER_RENEG_S == 20; the engine
                   mirrors the same private enum. */
                if (v34_get_rx_event(answer_modem) == 20
                    && !v34_rate_renegotiation_active(answer_modem)) {
                    v34_clear_peer_reneg_s_event(answer_modem);
                    if (v34_start_rate_renegotiation(answer_modem) == 0)
                        fprintf(stderr,
                                "[RENEG] block=%d answerer detected S and "
                                "responded per 11.6.1.2\n", block);
                }
                if (cleardown_at > 0 && v34_cleardown_complete(call_modem)
                    && v34_cleardown_complete(answer_modem)) {
                    fprintf(stderr, "[CLEARDOWN] block=%d both ends complete\n", block);
                    cleardown_ok = true;
                    completed_block = block;
                    break;
                }
                if (caller_stage != caller_data_stage)
                    caller_left_data = true;
                if (answer_stage != answer_data_stage)
                    answer_left_data = true;
                if (caller_left_data && !caller_resynced
                    && caller_stage == caller_data_stage) {
                    caller_resynced = true;
                    caller.post_reneg_bits = 0;
                    caller.post_reneg_errors = 0;
                    endpoint_begin_resync(&caller);
                    fprintf(stderr, "[RENEG] block=%d caller back in data mode\n",
                            block);
                }
                if (answer_left_data && !answer_resynced
                    && answer_stage == answer_data_stage) {
                    answer_resynced = true;
                    answer.post_reneg_bits = 0;
                    answer.post_reneg_errors = 0;
                    endpoint_begin_resync(&answer);
                    fprintf(stderr, "[RENEG] block=%d answerer back in data mode\n",
                            block);
                }
            }
            if (caller_resynced && answer_resynced
                && caller.post_reneg_bits >= POST_RENEG_BITS
                && answer.post_reneg_bits >= POST_RENEG_BITS) {
                completed_block = block;
                break;
            }
            continue;
        }

        if (caller.trained && answer.trained
            && caller.rx_bits >= payload_bits
            && answer.rx_bits >= payload_bits) {
            completed_block = block;
            break;
        }
    }

    /* NOTE on `errors` when V34_DUPLEX_RENEG is set: this is the WHOLE-RUN
       bit_errors count, and it spans the renegotiation seam, where §11.6.1.2.1's
       "clamp circuit 104" is approximate -- the responder cannot clamp until its
       detector has seen ~30 ms of S, so it delivers a stretch of garbage to the
       DTE first (V.42's frame CRC is what discards it on a real link).  Half of
       those bits match the LFSR by chance, so a clean run still reports a
       nonzero figure here: 51-89 across the ten rate rows, varying with where
       the seam lands.  The PASS CRITERION is post_reneg_errors on the line
       below, which must be 0 in both directions -- do not read this one as the
       result. */
    printf("V.34 duplex %d baud/%d bps/%s: trained=%d/%d "
           "rx_bits=%d/%d errors=%d/%d time=%.2fs\n",
           baud, bps, alaw ? "alaw" : "ulaw",
           caller.trained, answer.trained,
           caller.rx_bits, answer.rx_bits,
           caller.bit_errors, answer.bit_errors,
           completed_block >= 0 ? (completed_block + 1)*0.020 : max_blocks*0.020);
    /* V34_DUPLEX_EXPECT_RATES=a2c,c2a: the MP exchange must have settled on
       exactly these rates (bit/s), at both ends. */
    if (getenv("V34_DUPLEX_EXPECT_RATES")) {
        int want_a2c = 0, want_c2a = 0;
        int got[2][2] = { { 0, 0 }, { 0, 0 } };
        bool ok;

        sscanf(getenv("V34_DUPLEX_EXPECT_RATES"), "%d,%d", &want_a2c, &want_c2a);
        ok = v34_get_negotiated_mp_rates(call_modem, &got[0][0], &got[0][1]) == 0
          && v34_get_negotiated_mp_rates(answer_modem, &got[1][0], &got[1][1]) == 0;
        printf("MP rates: caller a2c=%d c2a=%d, answerer a2c=%d c2a=%d (want %d/%d)\n",
               got[0][0]*2400, got[0][1]*2400, got[1][0]*2400, got[1][1]*2400,
               want_a2c, want_c2a);
        for (int k = 0; k < 2; k++)
            ok = ok && got[k][0]*2400 == want_a2c && got[k][1]*2400 == want_c2a;
        if (!ok) {
            printf("FAIL: MP rates differ from V34_DUPLEX_EXPECT_RATES\n");
            return 1;
        }
    }
    printf("  source bits: caller=%d answer=%d; sync skipped=%d/%d; "
           "peaks=%d/%d clipped=%d/%d\n",
           caller.tx_bits, answer.tx_bits, caller.skipped_bits, answer.skipped_bits,
           caller.peak_sample, answer.peak_sample,
           caller.clipped_samples, answer.clipped_samples);
    printf("  stages: caller rx=%d tx=%d event=%d; answer rx=%d tx=%d event=%d\n",
           v34_get_rx_stage(call_modem), v34_get_tx_stage(call_modem),
           v34_get_rx_event(call_modem),
           v34_get_rx_stage(answer_modem), v34_get_tx_stage(answer_modem),
           v34_get_rx_event(answer_modem));

    if (reneg_at > 0) {
        printf("  11.6 renegotiation: initiated at block %d, caller "
               "resynced=%d (%d bits, %d errors), answerer resynced=%d "
               "(%d bits, %d errors)\n",
               reneg_block,
               caller_resynced, caller.post_reneg_bits, caller.post_reneg_errors,
               answer_resynced, answer.post_reneg_bits, answer.post_reneg_errors);
        printf("  11.6 resync: caller restarts=%d still_searching=%d; "
               "answerer restarts=%d still_searching=%d\n",
               caller.resync_restarts, caller.resyncing,
               answer.resync_restarts, answer.resyncing);
    }

    v34_free(call_modem);
    v34_free(answer_modem);
    if (cleardown_at > 0)
        return (cleardown_ok && completed_block >= 0) ? 0 : 1;
    if (reneg_at > 0) {
        return (completed_block >= 0
                && caller_resynced && answer_resynced
                && caller.post_reneg_errors == 0
                && answer.post_reneg_errors == 0) ? 0 : 1;
    }
    return (completed_block >= 0
            && caller.bit_errors == 0
            && answer.bit_errors == 0) ? 0 : 1;
}

int main(int argc, char *argv[])
{
    int baud = 2400;
    int bps = 9600;
    bool alaw = false;

    if (argc > 1) baud = atoi(argv[1]);
    if (argc > 2) bps = atoi(argv[2]);
    if (argc > 3) alaw = strcmp(argv[3], "alaw") == 0;
    return run_case(baud, bps, alaw);
}
