/*
 * vpcm_loopback_test.c - Local PCM-modem self-test
 *
 * This harness is intended for the V.9x PCM modem family. The first
 * implementation exercises the current V.91 transparent-mode codec by
 * wiring two local instances back-to-back over simulated G.711. Future
 * V.90/V.92 cases can be added here without needing a separate test
 * executable for each PCM-mode modem flavor.
 */

#include "v91.h"

#include <spandsp.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TEST_PAYLOAD_LEN 4096
#define TEST_CHUNK_MAX    257
#define VPCM_CHUNK_SAMPLES 160
#define VPCM_V8_MAX_CHUNKS 1500

typedef enum {
    VPCM_PATH_ANALOG_G711 = 0,
    VPCM_PATH_PCM_G711 = 1
} vpcm_path_mode_t;

typedef struct {
    v91_law_t law;
    vpcm_path_mode_t mode;
} vpcm_channel_t;

typedef struct {
    bool seen;
    v8_parms_t result;
} vpcm_v8_result_t;

static uint32_t prng_next(uint32_t *state)
{
    uint32_t x;

    x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}

static void fill_pattern(uint8_t *buf, int len, uint32_t seed)
{
    int i;

    for (i = 0; i < len; i++)
        buf[i] = (uint8_t) prng_next(&seed);
}

static bool expect_equal(const char *label,
                         const uint8_t *expected,
                         const uint8_t *actual,
                         int len)
{
    int i;

    for (i = 0; i < len; i++) {
        if (expected[i] != actual[i]) {
            fprintf(stderr,
                    "%s mismatch at byte %d: expected 0x%02X got 0x%02X\n",
                    label, i, expected[i], actual[i]);
            return false;
        }
    }
    return true;
}

static uint8_t canonical_codeword(v91_law_t law, uint8_t codeword)
{
    return v91_linear_to_codeword(law, v91_codeword_to_linear(law, codeword));
}

static void vpcm_transport_linear(vpcm_channel_t channel,
                                  int16_t *dst,
                                  const int16_t *src,
                                  int len)
{
    int i;

    if (channel.mode != VPCM_PATH_ANALOG_G711) {
        fprintf(stderr, "linear transport requested on non-analog channel\n");
        abort();
    }

    for (i = 0; i < len; i++) {
        uint8_t codeword = v91_linear_to_codeword(channel.law, src[i]);
        dst[i] = v91_codeword_to_linear(channel.law, codeword);
    }
}

static void vpcm_transport_codewords(vpcm_channel_t channel,
                                     uint8_t *dst,
                                     const uint8_t *src,
                                     int len)
{
    if (channel.mode != VPCM_PATH_PCM_G711) {
        fprintf(stderr, "codeword transport requested on non-PCM channel\n");
        abort();
    }

    memcpy(dst, src, (size_t) len);
}

static void vpcm_v8_result_handler(void *user_data, v8_parms_t *result)
{
    vpcm_v8_result_t *capture;

    capture = (vpcm_v8_result_t *) user_data;
    capture->seen = true;
    capture->result = *result;
}

static void init_v8_parms(v8_parms_t *parms,
                          bool calling_party,
                          int modulations,
                          int pcm_availability)
{
    memset(parms, 0, sizeof(*parms));
    parms->modem_connect_tone = calling_party ? MODEM_CONNECT_TONES_NONE
                                              : MODEM_CONNECT_TONES_ANSAM;
    parms->send_ci = calling_party;
    parms->v92 = -1;
    parms->jm_cm.call_function = V8_CALL_V_SERIES;
    parms->jm_cm.modulations = modulations;
    parms->jm_cm.protocols = V8_PROTOCOL_NONE;
    parms->jm_cm.pstn_access = (pcm_availability != 0) ? V8_PSTN_ACCESS_DCE_ON_DIGITAL : 0;
    parms->jm_cm.pcm_modem_availability = pcm_availability;
    parms->jm_cm.nsf = -1;
    parms->jm_cm.t66 = -1;
}

static bool run_v8_exchange(v91_law_t law,
                            const v8_parms_t *caller_parms,
                            const v8_parms_t *answer_parms,
                            vpcm_v8_result_t *caller_result,
                            vpcm_v8_result_t *answer_result)
{
    vpcm_channel_t analog_channel;
    v8_state_t *caller;
    v8_state_t *answer;
    int chunk;

    analog_channel.law = law;
    analog_channel.mode = VPCM_PATH_ANALOG_G711;
    memset(caller_result, 0, sizeof(*caller_result));
    memset(answer_result, 0, sizeof(*answer_result));

    caller = v8_init(NULL, true, (v8_parms_t *) caller_parms, vpcm_v8_result_handler, caller_result);
    answer = v8_init(NULL, false, (v8_parms_t *) answer_parms, vpcm_v8_result_handler, answer_result);
    if (caller == NULL || answer == NULL) {
        if (caller)
            v8_free(caller);
        if (answer)
            v8_free(answer);
        fprintf(stderr, "failed to initialize V.8 states\n");
        return false;
    }

    for (chunk = 0; chunk < VPCM_V8_MAX_CHUNKS; chunk++) {
        int16_t caller_tx[VPCM_CHUNK_SAMPLES];
        int16_t answer_tx[VPCM_CHUNK_SAMPLES];
        int16_t caller_rx[VPCM_CHUNK_SAMPLES];
        int16_t answer_rx[VPCM_CHUNK_SAMPLES];

        v8_tx(caller, caller_tx, VPCM_CHUNK_SAMPLES);
        v8_tx(answer, answer_tx, VPCM_CHUNK_SAMPLES);
        vpcm_transport_linear(analog_channel, answer_rx, caller_tx, VPCM_CHUNK_SAMPLES);
        vpcm_transport_linear(analog_channel, caller_rx, answer_tx, VPCM_CHUNK_SAMPLES);
        v8_rx(caller, caller_rx, VPCM_CHUNK_SAMPLES);
        v8_rx(answer, answer_rx, VPCM_CHUNK_SAMPLES);

        if (caller_result->seen
            && answer_result->seen
            && caller_result->result.status == V8_STATUS_V8_CALL
            && answer_result->result.status == V8_STATUS_V8_CALL)
            break;
    }

    v8_free(caller);
    v8_free(answer);

    if (!caller_result->seen || !answer_result->seen) {
        fprintf(stderr, "V.8 exchange produced no final result\n");
        return false;
    }
    if (caller_result->result.status != V8_STATUS_V8_CALL
        || answer_result->result.status != V8_STATUS_V8_CALL) {
        fprintf(stderr, "V.8 exchange failed: caller=%d answer=%d\n",
                caller_result->result.status, answer_result->result.status);
        return false;
    }

    return true;
}

static bool test_v8_v90_startup_over_analog_g711(v91_law_t law)
{
    v8_parms_t caller_parms;
    v8_parms_t answer_parms;
    vpcm_v8_result_t caller_result;
    vpcm_v8_result_t answer_result;
    uint32_t expected_mods;
    int expected_pcm;

    expected_mods = V8_MOD_V22 | V8_MOD_V34 | V8_MOD_V90;
    expected_pcm = V8_PSTN_PCM_MODEM_V90_V92_DIGITAL;
    init_v8_parms(&caller_parms, true, expected_mods, expected_pcm);
    init_v8_parms(&answer_parms, false, expected_mods, expected_pcm);

    if (!run_v8_exchange(law, &caller_parms, &answer_parms, &caller_result, &answer_result))
        return false;

    if (caller_result.result.jm_cm.call_function != V8_CALL_V_SERIES
        || answer_result.result.jm_cm.call_function != V8_CALL_V_SERIES) {
        fprintf(stderr, "V.8 V.90 startup call function mismatch\n");
        return false;
    }
    if ((caller_result.result.jm_cm.modulations & expected_mods) != expected_mods
        || (answer_result.result.jm_cm.modulations & expected_mods) != expected_mods) {
        fprintf(stderr, "V.8 V.90 startup modulation mismatch caller=0x%X answer=0x%X\n",
                caller_result.result.jm_cm.modulations,
                answer_result.result.jm_cm.modulations);
        return false;
    }
    if ((caller_result.result.jm_cm.pcm_modem_availability & expected_pcm) != expected_pcm
        || (answer_result.result.jm_cm.pcm_modem_availability & expected_pcm) != expected_pcm) {
        fprintf(stderr, "V.8 V.90 startup PCM availability mismatch caller=0x%X answer=0x%X\n",
                caller_result.result.jm_cm.pcm_modem_availability,
                answer_result.result.jm_cm.pcm_modem_availability);
        return false;
    }

    return true;
}

static bool test_v8_v91_advertisement_over_analog_g711(v91_law_t law)
{
    v8_parms_t caller_parms;
    v8_parms_t answer_parms;
    vpcm_v8_result_t caller_result;
    vpcm_v8_result_t answer_result;
    uint32_t expected_mods;
    int expected_pcm;

    expected_mods = V8_MOD_V22 | V8_MOD_V34;
    expected_pcm = V8_PSTN_PCM_MODEM_V91;
    init_v8_parms(&caller_parms, true, expected_mods, expected_pcm);
    init_v8_parms(&answer_parms, false, expected_mods, expected_pcm);

    if (!run_v8_exchange(law, &caller_parms, &answer_parms, &caller_result, &answer_result))
        return false;

    if ((caller_result.result.jm_cm.pcm_modem_availability & V8_PSTN_PCM_MODEM_V91) == 0
        || (answer_result.result.jm_cm.pcm_modem_availability & V8_PSTN_PCM_MODEM_V91) == 0) {
        fprintf(stderr, "V.8 V.91 advertisement missing caller=0x%X answer=0x%X\n",
                caller_result.result.jm_cm.pcm_modem_availability,
                answer_result.result.jm_cm.pcm_modem_availability);
        return false;
    }

    return true;
}

static bool test_v91_codeword_loopback(v91_law_t law)
{
    v91_state_t tx;
    v91_state_t rx;
    uint8_t input[TEST_PAYLOAD_LEN];
    uint8_t g711[TEST_CHUNK_MAX];
    uint8_t output[TEST_PAYLOAD_LEN];
    vpcm_channel_t pcm_channel;
    int in_pos;
    int out_pos;
    int chunk_seed;

    v91_init(&tx, law, V91_MODE_TRANSPARENT);
    v91_init(&rx, law, V91_MODE_TRANSPARENT);
    pcm_channel.law = law;
    pcm_channel.mode = VPCM_PATH_PCM_G711;
    fill_pattern(input, TEST_PAYLOAD_LEN, 0x12345678U ^ (uint32_t) law);

    in_pos = 0;
    out_pos = 0;
    chunk_seed = 17;
    while (in_pos < TEST_PAYLOAD_LEN) {
        int want;
        int produced;
        int consumed;

        want = 1 + (chunk_seed % TEST_CHUNK_MAX);
        if (want > (TEST_PAYLOAD_LEN - in_pos))
            want = TEST_PAYLOAD_LEN - in_pos;
        chunk_seed = (chunk_seed * 73 + 19) % 997;

        produced = v91_tx_codewords(&tx, g711, TEST_CHUNK_MAX, input + in_pos, want);
        vpcm_transport_codewords(pcm_channel, g711, g711, produced);
        consumed = v91_rx_codewords(&rx, output + out_pos, TEST_PAYLOAD_LEN - out_pos, g711, produced);
        if (produced != want || consumed != want) {
            fprintf(stderr, "V.91 codeword loopback length mismatch: tx=%d rx=%d want=%d\n",
                    produced, consumed, want);
            return false;
        }

        in_pos += want;
        out_pos += consumed;
    }

    return expect_equal("V.91 codeword loopback", input, output, TEST_PAYLOAD_LEN);
}

static bool test_v91_linear_loopback(v91_law_t law)
{
    v91_state_t tx;
    v91_state_t rx;
    uint8_t input[TEST_PAYLOAD_LEN];
    uint8_t canonical[TEST_PAYLOAD_LEN];
    int16_t linear[TEST_CHUNK_MAX];
    uint8_t output[TEST_PAYLOAD_LEN];
    int in_pos;
    int out_pos;
    int chunk_seed;

    v91_init(&tx, law, V91_MODE_TRANSPARENT);
    v91_init(&rx, law, V91_MODE_TRANSPARENT);
    fill_pattern(input, TEST_PAYLOAD_LEN, 0xA5A55A5AU ^ (uint32_t) law);
    for (in_pos = 0; in_pos < TEST_PAYLOAD_LEN; in_pos++)
        canonical[in_pos] = canonical_codeword(law, input[in_pos]);

    in_pos = 0;
    out_pos = 0;
    chunk_seed = 29;
    while (in_pos < TEST_PAYLOAD_LEN) {
        int want;
        int produced;
        int consumed;

        want = 1 + (chunk_seed % TEST_CHUNK_MAX);
        if (want > (TEST_PAYLOAD_LEN - in_pos))
            want = TEST_PAYLOAD_LEN - in_pos;
        chunk_seed = (chunk_seed * 61 + 7) % 1009;

        produced = v91_tx_linear(&tx, linear, TEST_CHUNK_MAX, input + in_pos, want);
        consumed = v91_rx_linear(&rx, output + out_pos, TEST_PAYLOAD_LEN - out_pos, linear, produced);
        if (produced != want || consumed != want) {
            fprintf(stderr, "V.91 linear loopback length mismatch: tx=%d rx=%d want=%d\n",
                    produced, consumed, want);
            return false;
        }

        in_pos += want;
        out_pos += consumed;
    }

    return expect_equal("V.91 linear loopback", canonical, output, TEST_PAYLOAD_LEN);
}

static bool test_v91_full_duplex(v91_law_t law)
{
    v91_state_t a_tx;
    v91_state_t a_rx;
    v91_state_t b_tx;
    v91_state_t b_rx;
    uint8_t a_input[TEST_PAYLOAD_LEN];
    uint8_t b_input[TEST_PAYLOAD_LEN];
    uint8_t a_output[TEST_PAYLOAD_LEN];
    uint8_t b_output[TEST_PAYLOAD_LEN];
    uint8_t a_to_b[160];
    uint8_t b_to_a[160];
    int a_in_pos;
    int b_in_pos;
    int a_out_pos;
    int b_out_pos;
    int stride;

    v91_init(&a_tx, law, V91_MODE_TRANSPARENT);
    v91_init(&a_rx, law, V91_MODE_TRANSPARENT);
    v91_init(&b_tx, law, V91_MODE_TRANSPARENT);
    v91_init(&b_rx, law, V91_MODE_TRANSPARENT);

    fill_pattern(a_input, TEST_PAYLOAD_LEN, 0xCAFEBABEU ^ (uint32_t) law);
    fill_pattern(b_input, TEST_PAYLOAD_LEN, 0x0BADF00DU ^ (uint32_t) law);

    a_in_pos = 0;
    b_in_pos = 0;
    a_out_pos = 0;
    b_out_pos = 0;
    stride = 97;

    while (a_in_pos < TEST_PAYLOAD_LEN || b_in_pos < TEST_PAYLOAD_LEN) {
        int a_want;
        int b_want;
        int a_prod;
        int b_prod;
        int a_cons;
        int b_cons;

        a_want = (a_in_pos < TEST_PAYLOAD_LEN) ? stride : 0;
        b_want = (b_in_pos < TEST_PAYLOAD_LEN) ? (stride + 23) : 0;
        if (a_want > (int) sizeof(a_to_b))
            a_want = (int) sizeof(a_to_b);
        if (b_want > (int) sizeof(b_to_a))
            b_want = (int) sizeof(b_to_a);
        if (a_want > (TEST_PAYLOAD_LEN - a_in_pos))
            a_want = TEST_PAYLOAD_LEN - a_in_pos;
        if (b_want > (TEST_PAYLOAD_LEN - b_in_pos))
            b_want = TEST_PAYLOAD_LEN - b_in_pos;

        a_prod = v91_tx_codewords(&a_tx, a_to_b, 160, a_input + a_in_pos, a_want);
        b_prod = v91_tx_codewords(&b_tx, b_to_a, 160, b_input + b_in_pos, b_want);
        a_cons = v91_rx_codewords(&a_rx, a_output + a_out_pos, TEST_PAYLOAD_LEN - a_out_pos, b_to_a, b_prod);
        b_cons = v91_rx_codewords(&b_rx, b_output + b_out_pos, TEST_PAYLOAD_LEN - b_out_pos, a_to_b, a_prod);

        if (a_cons != b_prod || b_cons != a_prod) {
            fprintf(stderr,
                    "V.91 full duplex length mismatch: A tx=%d B rx=%d, B tx=%d A rx=%d\n",
                    a_prod, b_cons, b_prod, a_cons);
            return false;
        }

        a_in_pos += a_want;
        b_in_pos += b_want;
        a_out_pos += a_cons;
        b_out_pos += b_cons;
        stride = (stride + 37) % 149 + 1;
    }

    return expect_equal("V.91 full duplex A<-B", b_input, a_output, TEST_PAYLOAD_LEN)
        && expect_equal("V.91 full duplex B<-A", a_input, b_output, TEST_PAYLOAD_LEN);
}

int main(void)
{
    if (!test_v91_codeword_loopback(V91_LAW_ULAW))
        return 1;
    if (!test_v91_codeword_loopback(V91_LAW_ALAW))
        return 1;
    if (!test_v8_v90_startup_over_analog_g711(V91_LAW_ULAW))
        return 1;
    if (!test_v8_v90_startup_over_analog_g711(V91_LAW_ALAW))
        return 1;
    if (!test_v8_v91_advertisement_over_analog_g711(V91_LAW_ULAW))
        return 1;
    if (!test_v8_v91_advertisement_over_analog_g711(V91_LAW_ALAW))
        return 1;
    if (!test_v91_linear_loopback(V91_LAW_ULAW))
        return 1;
    if (!test_v91_linear_loopback(V91_LAW_ALAW))
        return 1;
    if (!test_v91_full_duplex(V91_LAW_ULAW))
        return 1;
    if (!test_v91_full_duplex(V91_LAW_ALAW))
        return 1;

    puts("vpcm_loopback_test: OK");
    return 0;
}
