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

#define BLOCK_SAMPLES 160
#define MAX_BLOCKS 3000       /* 60 seconds at 8 kHz */
#define PAYLOAD_BITS 16000

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
        } else {
            ep->skipped_bits++;
        }
        return;
    }
    if (bit != pattern_bit(&ep->expected_lfsr))
        ep->bit_errors++;
    ep->rx_bits++;
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
        for (int i = 0; i < BLOCK_SAMPLES; i++) {
            int call_abs = abs(call_tx[i]);
            int answer_abs = abs(answer_tx[i]);
            if (call_abs > caller.peak_sample) caller.peak_sample = (int16_t)call_abs;
            if (answer_abs > answer.peak_sample) answer.peak_sample = (int16_t)answer_abs;
            if (call_abs >= 32760) caller.clipped_samples++;
            if (answer_abs >= 32760) answer.clipped_samples++;
            answer_rx[i] = g711_roundtrip(call_tx[i], alaw);
            call_rx[i] = g711_roundtrip(answer_tx[i], alaw);
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
        (void)v34_rx(answer_modem, answer_rx, BLOCK_SAMPLES);
        (void)v34_rx(call_modem, call_rx, BLOCK_SAMPLES);
        span_log_bump_samples(v34_get_logging_state(call_modem), BLOCK_SAMPLES);
        span_log_bump_samples(v34_get_logging_state(answer_modem), BLOCK_SAMPLES);

        if (caller.failed || answer.failed)
            break;
        if (caller.trained && answer.trained
            && caller.rx_bits >= PAYLOAD_BITS
            && answer.rx_bits >= PAYLOAD_BITS) {
            completed_block = block;
            break;
        }
    }

    printf("V.34 duplex %d baud/%d bps/%s: trained=%d/%d "
           "rx_bits=%d/%d errors=%d/%d time=%.2fs\n",
           baud, bps, alaw ? "alaw" : "ulaw",
           caller.trained, answer.trained,
           caller.rx_bits, answer.rx_bits,
           caller.bit_errors, answer.bit_errors,
           completed_block >= 0 ? (completed_block + 1)*0.020 : max_blocks*0.020);
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

    v34_free(call_modem);
    v34_free(answer_modem);
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
