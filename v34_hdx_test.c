/*
 * v34_hdx_test.c - ITU-T V.34 clause 12 half-duplex startup harness.
 *
 * Drives a source/recipient V.34 half-duplex pair at each other over a G.711
 * bearer and reports exactly how far clause 12 gets: Phase 2 (12.2, INFOh in
 * place of INFO1a/INFO1c per 10.2.2), Phase 3 (12.3, S/S-bar/PP/TRN) and
 * control channel start-up (12.4, PPh/ALT/MPh/E).
 *
 * Grades independent payload generators on the bidirectional control channel.
 * V34_HDX_PRIMARY=1 then requests 12.5 S/S-bar/PP/B1 and grades the source's
 * primary payload, requiring no skipped payload bits and a silent recipient.
 * V34_HDX_RX_DUMP=<path> saves recovered primary bits (one byte per bit).
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "spandsp.h"
#include "spandsp/private/bitstream.h"
#include "spandsp/private/power_meter.h"
#include "spandsp/private/logging.h"
#include "spandsp/private/v34.h"

#define MAX_BLOCK_SAMPLES 160

#define RX_CAPTURE_BITS 65536

typedef struct
{
    const char *name;
    uint32_t lfsr;
    int bits_out;
    int bits_in;
    /*! The bits this endpoint received, so the run can be graded against the
        far end's generator rather than merely counted.  A bit count says a
        modulator ran; it says nothing about whether the control channel
        carries what was put into it, and the point of 12.4 is the data. */
    uint8_t rx_bits[RX_CAPTURE_BITS];
    int rx_len;
} endpoint_t;

static uint32_t lfsr_next(uint32_t *lfsr)
{
    uint32_t bit = *lfsr & 1;

    *lfsr = (*lfsr >> 1) ^ ((uint32_t) -(int32_t) bit & 0x80200003U);
    return bit;
}

/*! Grade a received bit stream against the far end's generator.  The receiver
    starts part way into the sequence -- the control channel begins carrying
    data at E, and the far end has been pulling bits since its own E -- so the
    alignment is searched, requiring a long exact run before it is believed.
    Returns the number of bit errors after the alignment, or -1 if no
    alignment was found. */
static int grade_rx_skip(const endpoint_t *e, uint32_t far_seed,
                         int max_skip, int *graded, int *offset, int *skipped)
{
    uint8_t reference[RX_CAPTURE_BITS + 4096];
    uint32_t state = far_seed;
    int skip, off, i, errors;

    *graded = 0;
    *offset = -1;
    *skipped = 0;
    if (e->rx_len < 256)
        return -1;
    for (i = 0; i < (int) sizeof(reference); i++)
        reference[i] = (uint8_t) lfsr_next(&state);
    for (skip = 0; skip <= max_skip && skip + 256 <= e->rx_len; skip++)
    {
        for (off = 0; off < 4096; off++)
        {
            if (memcmp(reference + off, e->rx_bits + skip, 128))
                continue;
            errors = 0;
            for (i = skip; i < e->rx_len; i++)
                errors += reference[off + i - skip] != e->rx_bits[i];
            *graded = e->rx_len - skip;
            *offset = off;
            *skipped = skip;
            return errors;
        }
    }
    return -1;
}

static int grade_rx(const endpoint_t *e, uint32_t far_seed, int *graded, int *offset)
{
    int skipped;
    return grade_rx_skip(e, far_seed, 0, graded, offset, &skipped);
}

static int get_bit(void *user_data)
{
    endpoint_t *e = (endpoint_t *) user_data;

    e->bits_out++;
    return (int) lfsr_next(&e->lfsr);
}

static void put_bit(void *user_data, int bit)
{
    endpoint_t *e = (endpoint_t *) user_data;

    if (bit < 0)
        return;
    /*endif*/
    e->bits_in++;
    if (e->rx_len < RX_CAPTURE_BITS)
        e->rx_bits[e->rx_len++] = (uint8_t) (bit & 1);
    /*endif*/
}

/* Test-only analogue channel before G.711 quantization, followed by a
   sample-exact propagation delay. Never insert/drop samples after startup. */
typedef struct
{
    int16_t delay[8001];
    int delay_samples;
    int pos;
    int noise_peak;
    uint32_t noise_state;
} test_channel_t;

static void channel_process(test_channel_t *c, int16_t out[], const int16_t in[], int len, int alaw)
{
    for (int i = 0; i < len; i++)
    {
        int sample = in[i];
        if (c->noise_peak)
        {
            c->noise_state = c->noise_state*1664525U + 1013904223U;
            sample += (int)((c->noise_state >> 16) % (2*c->noise_peak + 1)) - c->noise_peak;
            if (sample > 32767) sample = 32767;
            if (sample < -32768) sample = -32768;
        }
        int16_t pcm = alaw ? alaw_to_linear(linear_to_alaw(sample))
                          : ulaw_to_linear(linear_to_ulaw(sample));
        if (c->delay_samples)
        {
            out[i] = c->delay[c->pos];
            c->delay[c->pos] = pcm;
            c->pos = (c->pos + 1) % c->delay_samples;
        }
        else
            out[i] = pcm;
    }
}

int main(int argc, char *argv[])
{
    int frame_samples = getenv("V34_HDX_BLOCK_SAMPLES")
                      ? atoi(getenv("V34_HDX_BLOCK_SAMPLES")) : MAX_BLOCK_SAMPLES;
    if (frame_samples < 1 || frame_samples > MAX_BLOCK_SAMPLES) {
        fprintf(stderr, "V34_HDX_BLOCK_SAMPLES must be 1..160\n");
        return 1;
    }
    static test_channel_t source_channel = {.noise_state = 0x12345678U};
    static test_channel_t recipient_channel = {.noise_state = 0x87654321U};
    int delay_ms = getenv("V34_HDX_DELAY_MS") ? atoi(getenv("V34_HDX_DELAY_MS")) : 0;
    int reverse_delay_ms = getenv("V34_HDX_REVERSE_DELAY_MS")
                         ? atoi(getenv("V34_HDX_REVERSE_DELAY_MS")) : delay_ms;
    int noise_peak = getenv("V34_HDX_NOISE_PEAK") ? atoi(getenv("V34_HDX_NOISE_PEAK")) : 0;
    if (delay_ms < 0 || delay_ms > 1000 || reverse_delay_ms < 0
        || reverse_delay_ms > 1000 || noise_peak < 0 || noise_peak > 32767)
    {
        fprintf(stderr, "channel delay must be 0..1000 ms and noise peak 0..32767\n");
        return 1;
    }
    source_channel.delay_samples = delay_ms*8;
    recipient_channel.delay_samples = reverse_delay_ms*8;
    source_channel.noise_peak = recipient_channel.noise_peak = noise_peak;
    int baud = (argc > 1) ? atoi(argv[1]) : 3200;
    int bps = (argc > 2) ? atoi(argv[2]) : 9600;
    int alaw = (argc > 3  &&  strcmp(argv[3], "alaw") == 0);
    double seconds = (argc > 4) ? atof(argv[4]) : 20.0;
    /* An optional second bit rate lets the two ends be configured
       differently, which is the only way to see V.34 12.4.1.3/12.4.2.4 do
       anything: with both ends the same, a negotiation that ignored the far
       end entirely would give the same answer. */
    int answ_bps = (argc > 5) ? atoi(argv[5]) : bps;
    int expect_bps = (bps < answ_bps) ? bps : answ_bps;
    int call_rate;
    int answ_rate;
    static endpoint_t call_e;
    static endpoint_t answ_e;
    static const uint32_t call_seed = 0x13579BDFU;
    static const uint32_t answ_seed = 0x2468ACE1U;
    int call_errors;
    int answ_errors;
    int call_graded;
    int answ_graded;
    int call_offset;
    int answ_offset;
    int primary = getenv("V34_HDX_PRIMARY") != NULL;
    int primary_started = 0;
    int control_ok = 0;
    int source_probe_seen = 0;
    int recipient_probe_seen = 0;
    int restart = getenv("V34_HDX_RESTART") != NULL;
    int restarted = 0;
    int primary_retrain_peer_seen = 0;
    int primary_retrain = getenv("V34_HDX_PRIMARY_RETRAIN")
                        ? atoi(getenv("V34_HDX_PRIMARY_RETRAIN")) : 0;
    int answer_source = getenv("V34_HDX_ANSWER_SOURCE") != NULL;
    int turnaround = getenv("V34_HDX_TURNAROUND") != NULL;
    int control_retrain = getenv("V34_HDX_CONTROL_RETRAIN")
                        ? atoi(getenv("V34_HDX_CONTROL_RETRAIN")) : 0;
    int control_retrain_started = 0;
    int control_retrain_complete = 0;
    int control_retrain_peer_seen = 0;
    int parameters = getenv("V34_HDX_PARAMETERS") != NULL;
    int parameter_pph_seen = 0;
    int parameter_ac_seen = 0;
    int returning = 0;
    int second_primary = 0;
    int drop_control = getenv("V34_HDX_DROP_CONTROL")
                     ? atoi(getenv("V34_HDX_DROP_CONTROL")) : 0;
    int drop_start = -1;
    int drop_reset = 0;
    int drop_retrain_seen = 0;
    int bad_mph = getenv("V34_HDX_BAD_MPH")
                ? atoi(getenv("V34_HDX_BAD_MPH")) : 0;
    int startup_tone = getenv("V34_HDX_STARTUP_TONE") != NULL;
    int startup_tone_sent = 0;
    int startup_tone_seen = 0;
#if defined(SPANDSP_USE_FIXED_POINT)
    complexi16_t (*recipient_tone_getbaud)(v34_state_t *) = NULL;
#else
    complexf_t (*recipient_tone_getbaud)(v34_state_t *) = NULL;
#endif
    int bad_mph_sent = 0;
    int bad_mph_recovered = 0;
    v34_state_t *call_modem;
    v34_state_t *answ_modem;
    int failed = 0;
    int16_t call_tx[MAX_BLOCK_SAMPLES];
    int16_t answ_tx[MAX_BLOCK_SAMPLES];
    int16_t call_rx[MAX_BLOCK_SAMPLES];
    int16_t answ_rx[MAX_BLOCK_SAMPLES];
    int blocks = (int) (seconds*8000.0/frame_samples);
    int block;
    int best_call_tx = -1;
    int best_answ_tx = -1;
    int best_call_rx = -1;
    int best_answ_rx = -1;

    call_e.name = "call";
    call_e.lfsr = call_seed;
    answ_e.name = "answer";
    answ_e.lfsr = answ_seed;

    /* duplex = false selects the clause 12 half-duplex modem. V.34 3.11/3.14:
       the source modem transmits primary channel data, the recipient receives
       it; the control channel is bidirectional throughout. */
    call_modem = v34_init(NULL, baud, bps, !answer_source, false,
                          get_bit, &call_e, put_bit, &call_e);
    answ_modem = v34_init(NULL, baud, answ_bps, answer_source, false,
                          get_bit, &answ_e, put_bit, &answ_e);
    if (call_modem == NULL  ||  answ_modem == NULL)
    {
        fprintf(stderr, "v34_hdx_test: v34_init failed\n");
        return 1;
    }
    /* Foreign recipient selects the low carrier; the source starts with its
       default high carrier and must learn the selection from INFOh. */
    if (getenv("V34_HDX_LOW_CARRIER"))
    {
        answ_modem->tx.high_carrier = false;
        answ_modem->rx.high_carrier = false;
    }
    v34_tx_power(call_modem, -12.0f);
    v34_tx_power(answ_modem, -12.0f);
    /* Unknown modes and a primary request before MPh/E must be refused. */
    if (v34_half_duplex_change_mode(NULL, V34_HALF_DUPLEX_PRIMARY_CHANNEL) != -1
        || v34_half_duplex_start_control_retrain(NULL) != -1
        || v34_half_duplex_start_control_retrain(call_modem) != -1
        || v34_half_duplex_request_parameters(NULL, 7200) != -1
        || v34_half_duplex_request_parameters(call_modem, 7200) != -1
        || v34_half_duplex_change_mode(call_modem, -1) != -1
        || v34_half_duplex_change_mode(call_modem, V34_HALF_DUPLEX_PRIMARY_CHANNEL) != -1)
    {
        fprintf(stderr, "invalid/early HDX mode request accepted\n");
        v34_free(call_modem);
        v34_free(answ_modem);
        return 1;
    }
    /* 12.2.1: call modem as source modem. */
    v34_half_duplex_change_mode(call_modem, V34_HALF_DUPLEX_SOURCE);
    v34_half_duplex_change_mode(answ_modem, V34_HALF_DUPLEX_RECIPIENT);
    if (getenv("V34_HDX_LOG"))
    {
        span_log_set_level(v34_get_logging_state(call_modem),
                           SPAN_LOG_SHOW_SEVERITY | SPAN_LOG_SHOW_TAG | SPAN_LOG_FLOW);
        span_log_set_level(v34_get_logging_state(answ_modem),
                           SPAN_LOG_SHOW_SEVERITY | SPAN_LOG_SHOW_TAG | SPAN_LOG_FLOW);
        /* Both modems log to the same stderr. Without distinct tags the two
           are indistinguishable, and a diagnostic read off the wrong one sends
           the investigation after a fault that is not there. */
        span_log_set_tag(v34_get_logging_state(call_modem), "SRC");
        span_log_set_tag(v34_get_logging_state(answ_modem), "RCP");
    }

    for (block = 0;  block < blocks;  block++)
    {
        if (call_modem->tx.current_modulator == V34_MODULATION_L1_L2)
            source_probe_seen = 1;
        if (answ_modem->tx.current_modulator == V34_MODULATION_L1_L2)
            recipient_probe_seen = 1;
        if (startup_tone && (answ_modem->tx.stage == V34_TX_STAGE_HDX_INITIAL_A
            || answ_modem->tx.stage == V34_TX_STAGE_HDX_FIRST_A))
            recipient_tone_getbaud = answ_modem->tx.current_getbaud;
        if (startup_tone && !startup_tone_sent && recipient_tone_getbaud
            && call_modem->tx.stage == V34_TX_STAGE_HDX_CC_SILENCE)
        {
            /* Emulate a foreign recipient falling back to 12.2.1.2.5:
               Tone A waits for returned Tone B, then INFOh. Only mutate
               the peer; the source must identify its actual G.711 signal. */
            span_sample_timer_t tx_time = answ_modem->tx.sample_time;
            span_sample_timer_t rx_time = answ_modem->rx.sample_time;
            v34_restart(answ_modem, baud, answ_bps, false);
            v34_half_duplex_change_mode(answ_modem, V34_HALF_DUPLEX_RECIPIENT);
            answ_modem->tx.sample_time = tx_time;
            answ_modem->rx.sample_time = rx_time;
            answ_modem->tx.training_stage = 0;
            answ_modem->tx.stage = V34_TX_STAGE_HDX_SECOND_A;
            answ_modem->tx.current_modulator = V34_MODULATION_CC;
            answ_modem->tx.current_getbaud = recipient_tone_getbaud;
            answ_modem->tx.tone_duration = 0;
            answ_modem->tx.lastbit.re = 4;
            answ_modem->tx.lastbit.im = 0;
            answ_modem->rx.current_demodulator = V34_MODULATION_TONES;
            answ_modem->rx.stage = answ_modem->calling_party ? V34_RX_STAGE_TONE_A : V34_RX_STAGE_TONE_B;
            answ_modem->rx.tone_b_present = false;
            answ_modem->rx.received_event = V34_EVENT_NONE;
            startup_tone_sent = 1;
        }
        if (startup_tone_sent && call_modem->tx.stage == V34_TX_STAGE_HDX_POST_L2_B)
            startup_tone_seen = 1;
        if (control_retrain && !control_retrain_started
            && v34_get_hdx_control_channel_ready(call_modem)
            && v34_get_hdx_control_channel_ready(answ_modem)
            && call_e.rx_len >= 512 && answ_e.rx_len >= 512)
        {
            int call_skip, answ_skip;
            if (grade_rx_skip(&call_e, answ_seed, 64, &call_graded, &call_offset, &call_skip)
                || grade_rx_skip(&answ_e, call_seed, 64, &answ_graded, &answ_offset, &answ_skip)
                || (control_retrain != 2 && v34_half_duplex_start_control_retrain(call_modem))
                || (control_retrain != 1 && v34_half_duplex_start_control_retrain(answ_modem)))
            {
                fprintf(stderr, "explicit control retrain request failed\n");
                return 1;
            }
            call_e.lfsr = call_seed;
            answ_e.lfsr = answ_seed;
            call_e.bits_out = call_e.bits_in = call_e.rx_len = 0;
            answ_e.bits_out = answ_e.bits_in = answ_e.rx_len = 0;
            control_retrain_started = 1;
        }
        if (control_retrain_started && !control_retrain_peer_seen
            && call_modem->tx.stage != V34_TX_STAGE_HDX_CC_DATA
            && answ_modem->tx.stage != V34_TX_STAGE_HDX_CC_DATA)
        {
            /* The responder may deliver the in-flight old stream until AC
               has been sustained for 100 ms. Grade only the new exchange. */
            call_e.lfsr = call_seed;
            answ_e.lfsr = answ_seed;
            call_e.bits_out = call_e.bits_in = call_e.rx_len = 0;
            answ_e.bits_out = answ_e.bits_in = answ_e.rx_len = 0;
            control_retrain_peer_seen = 1;
        }
        if (control_retrain_started && control_retrain_peer_seen
            && v34_get_hdx_control_channel_ready(call_modem)
            && v34_get_hdx_control_channel_ready(answ_modem)
            && call_e.rx_len >= 512 && answ_e.rx_len >= 512)
            control_retrain_complete = 1;
        if (primary && turnaround && primary_started && !returning
            && !second_primary && answ_e.rx_len >= 10000)
        {
            int skip;
            if (parameters)
            {
                if (v34_half_duplex_request_parameters(call_modem, 7200) != -1
                    || v34_half_duplex_request_parameters(answ_modem, 0) != -1
                    || v34_half_duplex_request_parameters(answ_modem, 10000) != -1
                    || v34_half_duplex_request_parameters(answ_modem, 36000) != -1
                    || v34_half_duplex_request_parameters(answ_modem, 7200) != 0)
                {
                    fprintf(stderr, "recipient parameter request failed\n");
                    return 1;
                }
                expect_bps = 7200;
            }
            if (grade_rx_skip(&answ_e, call_seed, 2048, &answ_graded,
                              &answ_offset, &skip) != 0 || answ_offset != 0
                || answ_graded < 8000
                || v34_half_duplex_change_mode(answ_modem, V34_HALF_DUPLEX_CONTROL_CHANNEL)
                || v34_half_duplex_change_mode(call_modem, V34_HALF_DUPLEX_CONTROL_CHANNEL))
            {
                fprintf(stderr, "primary/control turnaround failed\n");
                failed = 1;
                break;
            }
            call_e.lfsr = call_seed;
            answ_e.lfsr = answ_seed;
            call_e.bits_out = call_e.bits_in = call_e.rx_len = 0;
            answ_e.bits_out = answ_e.bits_in = answ_e.rx_len = 0;
            returning = 1;
            printf("  return to control requested at %.3fs\n", block*frame_samples/8000.0);
        }
        if (returning && parameters)
        {
            parameter_pph_seen |= call_modem->tx.stage == V34_TX_STAGE_HDX_PPH;
            parameter_ac_seen |= call_modem->tx.stage == V34_TX_STAGE_HDX_AC
                              || answ_modem->tx.stage == V34_TX_STAGE_HDX_AC;
            if (answ_modem->tx.stage == V34_TX_STAGE_HDX_MPH
                && !answ_modem->rx.pph_detected)
            {
                fprintf(stderr, "recipient MPh preceded real source PPh\n");
                return 1;
            }
        }
        if (returning && v34_get_hdx_control_channel_ready(call_modem)
            && v34_get_hdx_control_channel_ready(answ_modem)
            && call_e.rx_len >= 512 && answ_e.rx_len >= 512)
        {
            int call_skip, answ_skip;
            if ((parameters && (!parameter_pph_seen || parameter_ac_seen
                    || v34_get_hdx_negotiated_bit_rate(call_modem) != 7200
                    || v34_get_hdx_negotiated_bit_rate(answ_modem) != 7200))
                || grade_rx_skip(&call_e, answ_seed, 64, &call_graded, &call_offset, &call_skip) != 0
                || grade_rx_skip(&answ_e, call_seed, 64, &answ_graded, &answ_offset, &answ_skip) != 0
                || call_offset != 0 || answ_offset != 0
                || v34_half_duplex_change_mode(answ_modem, V34_HALF_DUPLEX_PRIMARY_CHANNEL)
                || v34_half_duplex_change_mode(call_modem, V34_HALF_DUPLEX_PRIMARY_CHANNEL))
            {
                fprintf(stderr, "returned control payload/second primary failed\n");
                failed = 1;
                break;
            }
            call_e.lfsr = call_seed;
            answ_e.lfsr = answ_seed;
            call_e.bits_out = call_e.bits_in = call_e.rx_len = 0;
            answ_e.bits_out = answ_e.bits_in = answ_e.rx_len = 0;
            returning = 0;
            second_primary = 1;
            printf("  second primary requested after verified bidirectional control at %.3fs\n", block*frame_samples/8000.0);
        }
        if (primary && (restart || primary_retrain) && primary_started && !restarted
            && answ_e.rx_len >= 10000)
        {
            int skip;
            if (grade_rx_skip(&answ_e, call_seed, 2048, &answ_graded,
                              &answ_offset, &skip) != 0 || answ_offset != 0
                || answ_graded < 8000
                || (!primary_retrain && (v34_restart(call_modem, baud, bps, false)
                    || v34_restart(answ_modem, baud, answ_bps, false))))
            {
                fprintf(stderr, "primary payload/restart failed\n");
                failed = 1;
                break;
            }
            call_e.lfsr = call_seed;
            answ_e.lfsr = answ_seed;
            call_e.bits_out = call_e.bits_in = call_e.rx_len = 0;
            answ_e.bits_out = answ_e.bits_in = answ_e.rx_len = 0;
            if (primary_retrain)
            {
                for (int end = 1; end <= 2; end++)
                {
                    if (primary_retrain != 3 && primary_retrain != end)
                        continue;
                    v34_state_t *initiator = end == 1 ? call_modem : answ_modem;
                    span_sample_timer_t tx_time = initiator->tx.sample_time;
                    span_sample_timer_t rx_time = initiator->rx.sample_time;
                    v34_start_retrain(initiator);
                    if (initiator->tx.sample_time != tx_time || initiator->rx.sample_time != rx_time
                        || initiator->tx.current_modulator != V34_MODULATION_SILENCE
                        || initiator->tx.tone_duration != 560
                        || initiator->rx.current_demodulator != V34_MODULATION_SILENCE)
                    {
                        fprintf(stderr, "primary retrain silence/clamp/clock violation\n");
                        return 1;
                    }
                }
            }
            primary_started = control_ok = 0;
            source_probe_seen = recipient_probe_seen = 0;
            restarted = 1;
            printf("  restarted after verified primary payload at %.3fs\n",
                   block*frame_samples/8000.0);
        }
        if (primary_retrain && restarted && !primary_retrain_peer_seen
            && call_modem->half_duplex_state != V34_HALF_DUPLEX_PRIMARY_CHANNEL
            && answ_modem->half_duplex_state != V34_HALF_DUPLEX_PRIMARY_CHANNEL)
        {
            /* The unprompted peer clamps only once it detects the tone;
               discard old in-flight primary bits before grading the fresh
               control stream, not bits from the recovered user interval. */
            call_e.lfsr = call_seed;
            answ_e.lfsr = answ_seed;
            call_e.bits_out = call_e.bits_in = call_e.rx_len = 0;
            answ_e.bits_out = answ_e.bits_in = answ_e.rx_len = 0;
            primary_retrain_peer_seen = 1;
        }
        if (primary && !primary_started
            && v34_get_hdx_control_channel_ready(call_modem)
            && v34_get_hdx_control_channel_ready(answ_modem)
            && call_e.rx_len >= 512 && answ_e.rx_len >= 512)
        {
            control_ok = grade_rx(&call_e, answ_seed, &call_graded, &call_offset) == 0
                      && grade_rx(&answ_e, call_seed, &answ_graded, &answ_offset) == 0;
            if (!control_ok
                || v34_half_duplex_change_mode(answ_modem, V34_HALF_DUPLEX_PRIMARY_CHANNEL)
                || v34_half_duplex_change_mode(call_modem, V34_HALF_DUPLEX_PRIMARY_CHANNEL))
            {
                fprintf(stderr, "primary transition failed\n");
                failed = 1;
                break;
            }
            call_e.lfsr = call_seed;
            answ_e.lfsr = answ_seed;
            call_e.bits_out = call_e.bits_in = call_e.rx_len = 0;
            answ_e.bits_out = answ_e.bits_in = answ_e.rx_len = 0;
            primary_started = 1;
            printf("  primary channel requested at %.3fs after verified control data\n",
                   block*frame_samples/8000.0);
        }
        if (bad_mph && !bad_mph_sent && answ_modem->tx.stage == V34_TX_STAGE_HDX_PPH)
        {
            /* CRC-valid Table 23 offer asks this implementation to transmit
               its unsupported 2400 bit/s CC. It must reject and recover,
               not silently accept the rate and keep sending 1200 bit/s. */
            if (bad_mph == 1)
                answ_modem->tx.mph.control_channel_2400 = 1;
            else
                answ_modem->tx.mph.signalling_rate_mask = 1 << 13;
            bad_mph_sent = 1;
        }
        if (bad_mph && bad_mph_sent && !bad_mph_recovered
            && call_modem->tx.stage == V34_TX_STAGE_HDX_AC)
        {
            call_e.lfsr = call_seed;
            answ_e.lfsr = answ_seed;
            call_e.bits_out = call_e.bits_in = call_e.rx_len = 0;
            answ_e.bits_out = answ_e.bits_in = answ_e.rx_len = 0;
            bad_mph_recovered = 1;
        }
        memset(call_tx, 0, sizeof(call_tx));
        memset(answ_tx, 0, sizeof(answ_tx));
        v34_tx(call_modem, call_tx, frame_samples);
        v34_tx(answ_modem, answ_tx, frame_samples);
        channel_process(&source_channel, answ_rx, call_tx, frame_samples, alaw);
        channel_process(&recipient_channel, call_rx, answ_tx, frame_samples, alaw);
        /* Drop one control direction for four seconds starting at the
           first PPh. The peers must recover through 12.8, not restart
           the whole modem or receive any ideal symbols from the harness. */
        if (drop_control && drop_start < 0
            && !v34_get_hdx_control_channel_ready(call_modem)
            && ((drop_control == 1 && v34_get_tx_stage(call_modem) == V34_TX_STAGE_HDX_PPH)
                || (drop_control == 2 && call_modem->rx.pph_detected && call_modem->rx.mp_seen == 0)
                || (drop_control == 3 && call_modem->rx.mp_seen == 1)))
            drop_start = block;
        if (drop_start >= 0 && block < drop_start + 32000/frame_samples)
            memset(call_rx, 0, sizeof(call_rx));
        if (drop_start >= 0 && block == drop_start + 32000/frame_samples && !drop_reset)
        {
            call_e.lfsr = call_seed;
            answ_e.lfsr = answ_seed;
            call_e.bits_out = call_e.bits_in = call_e.rx_len = 0;
            answ_e.bits_out = answ_e.bits_in = answ_e.rx_len = 0;
            drop_reset = 1;
        }
        v34_rx(answ_modem, answ_rx, frame_samples);
        if (drop_control == 3 && call_modem->rx.stage == V34_RX_STAGE_CC)
        {
            /* MPh and E may arrive in one media block. Observe the
               receive state at sample boundaries to remove E specifically,
               rather than accidentally missing this fault window. */
            for (int k = 0; k < frame_samples; k++)
            {
                if (drop_start < 0 && call_modem->rx.mp_seen == 1)
                    drop_start = block;
                int16_t sample = (drop_start >= 0
                    && block < drop_start + 32000/frame_samples) ? 0 : call_rx[k];
                v34_rx(call_modem, &sample, 1);
            }
        }
        else
            v34_rx(call_modem, call_rx, frame_samples);
        if (drop_control && (v34_get_tx_stage(call_modem) == V34_TX_STAGE_HDX_AC
            || v34_get_tx_stage(answ_modem) == V34_TX_STAGE_HDX_AC))
            drop_retrain_seen = 1;

        if (getenv("V34_HDX_RMS"))
        {
            double e = 0.0;
            double f = 0.0;
            int k;

            for (k = 0;  k < frame_samples;  k++)
            {
                e += (double) call_tx[k]*call_tx[k];
                f += (double) answ_tx[k]*answ_tx[k];
            }
            printf("  %7.3fs  src tx rms %8.1f  rcp tx rms %8.1f  (src stage %d)\n",
                   block*frame_samples/8000.0,
                   sqrt(e/frame_samples), sqrt(f/frame_samples),
                   v34_get_tx_stage(call_modem));
        }
        /*endif*/
        if (getenv("V34_HDX_STAGES"))
        {
            static int p_ct = -1, p_at = -1, p_cr = -1, p_ar = -1;
            int ct = v34_get_tx_stage(call_modem);
            int at = v34_get_tx_stage(answ_modem);
            int cr = v34_get_rx_stage(call_modem);
            int ar = v34_get_rx_stage(answ_modem);

            if (ct != p_ct  ||  at != p_at  ||  cr != p_cr  ||  ar != p_ar)
            {
                printf("  %7.3fs  src tx=%-2d rx=%-2d | rcp tx=%-2d rx=%-2d\n",
                       block*frame_samples/8000.0, ct, cr, at, ar);
                p_ct = ct;  p_at = at;  p_cr = cr;  p_ar = ar;
            }
            /*endif*/
        }
        /*endif*/
        if (v34_get_tx_stage(call_modem) > best_call_tx)
            best_call_tx = v34_get_tx_stage(call_modem);
        if (v34_get_tx_stage(answ_modem) > best_answ_tx)
            best_answ_tx = v34_get_tx_stage(answ_modem);
        if (v34_get_rx_stage(call_modem) > best_call_rx)
            best_call_rx = v34_get_rx_stage(call_modem);
        if (v34_get_rx_stage(answ_modem) > best_answ_rx)
            best_answ_rx = v34_get_rx_stage(answ_modem);
    }

    if (getenv("V34_HDX_LOW_CARRIER") && call_modem->tx.high_carrier)
    {
        fprintf(stderr, "source ignored INFOh low-carrier selection\n");
        return 1;
    }

    printf("V.34 half-duplex (clause 12), %d baud %d bps %s, %.1f s\n",
           baud, bps, alaw ? "A-law" : "u-law", seconds);
    if (delay_ms || reverse_delay_ms || noise_peak)
        printf("  test channel: source->recipient %d ms, reverse %d ms, uniform noise peak %d PCM units\n",
               delay_ms, reverse_delay_ms, noise_peak);
    printf("  call   (source)    tx stage %2d (max %2d)  rx stage %2d (max %2d)\n",
           v34_get_tx_stage(call_modem), best_call_tx,
           v34_get_rx_stage(call_modem), best_call_rx);
    printf("  answer (recipient) tx stage %2d (max %2d)  rx stage %2d (max %2d)\n",
           v34_get_tx_stage(answ_modem), best_answ_tx,
           v34_get_rx_stage(answ_modem), best_answ_rx);
    call_rate = v34_get_hdx_negotiated_bit_rate(call_modem);
    answ_rate = v34_get_hdx_negotiated_bit_rate(answ_modem);
    printf("  MPh rate (12.4.1.3/12.4.2.4): source tx %d bit/s, recipient rx %d bit/s (expected %d)\n",
           call_rate, answ_rate, expect_bps);
    printf("  payload bits: call out %d in %d, answer out %d in %d\n",
           call_e.bits_out, call_e.bits_in, answ_e.bits_out, answ_e.bits_in);
    if (!primary)
    {
        int call_skip, answ_skip;
        call_errors = grade_rx_skip(&call_e, answ_seed, (drop_control || bad_mph || control_retrain) ? 64 : 0,
                                    &call_graded, &call_offset, &call_skip);
        answ_errors = grade_rx_skip(&answ_e, call_seed, (drop_control || bad_mph || control_retrain) ? 64 : 0,
                                    &answ_graded, &answ_offset, &answ_skip);
        if (call_errors < 0)
            printf("  control channel data call<-answer: NO ALIGNMENT in %d bits\n", call_e.rx_len);
        else
            printf("  control channel data call<-answer: %d errors in %d bits (offset %d)\n",
                   call_errors, call_graded, call_offset);
        /*endif*/
        if (answ_errors < 0)
            printf("  control channel data answer<-call: NO ALIGNMENT in %d bits\n", answ_e.rx_len);
        else
            printf("  control channel data answer<-call: %d errors in %d bits (offset %d)\n",
                   answ_errors, answ_graded, answ_offset);
        /*endif*/
    }
    if (primary)
    {
        const char *dump = getenv("V34_HDX_RX_DUMP");
        if (dump)
        {
            FILE *fp = fopen(dump, "wb");
            if (!fp || fwrite(answ_e.rx_bits, 1, answ_e.rx_len, fp) != (size_t) answ_e.rx_len)
                failed = 1;
            if (fp)
                fclose(fp);
        }
        /* B1 is not payload. Find a 128-bit exact prefix of the independent
           source generator after the reset-state B1 interval, then grade all
           remaining bits. Require source offset zero: acquisition may skip
           B1, but it must not hide a corrupt payload prefix. */
        int skip;
        answ_errors = grade_rx_skip(&answ_e, call_seed, 2048,
                                    &answ_graded, &answ_offset, &skip);
        printf("  primary recipient: %d errors in %d bits (B1 skipped %d, source offset %d)\n",
               answ_errors, answ_graded, skip, answ_offset);
        failed |= !primary_started || !control_ok || ((restart || primary_retrain) && !restarted)
               || (turnaround && !second_primary)
               || answ_errors != 0
               || answ_graded < 8000 || answ_offset != 0
               || answ_e.bits_out != 0 || call_e.bits_in != 0;
    }
    else
    {
        failed |= call_errors != 0 || answ_errors != 0;
    }
    failed |= bad_mph && (!bad_mph_sent || !bad_mph_recovered);
    failed |= drop_control && (!drop_reset || drop_start < 0 || !drop_retrain_seen
              || !v34_get_hdx_control_channel_ready(call_modem)
              || !v34_get_hdx_control_channel_ready(answ_modem)
              || call_graded < 512 || answ_graded < 512);
    failed |= !source_probe_seen || recipient_probe_seen;
    failed |= startup_tone && (!startup_tone_sent || !startup_tone_seen);
    failed |= primary_retrain && !primary_retrain_peer_seen;
    failed |= control_retrain && !control_retrain_complete;
    failed |= call_rate != expect_bps || answ_rate != expect_bps;
    v34_free(call_modem);
    v34_free(answ_modem);
    return failed;
}
