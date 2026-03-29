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
#include <stdarg.h>

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

static bool g_vpcm_verbose = false;

static void vpcm_log(const char *fmt, ...);

static const char *vpcm_law_to_str(v91_law_t law)
{
    return (law == V91_LAW_ALAW) ? "A-law" : "u-law";
}

static const char *vpcm_path_mode_to_str(vpcm_path_mode_t mode)
{
    return (mode == VPCM_PATH_ANALOG_G711) ? "analog-over-G.711" : "raw-G.711";
}

static void vpcm_log(const char *fmt, ...)
{
    va_list ap;

    fprintf(stderr, "[VPCM] ");
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
}

static bool vpcm_info_frames_equal(const v91_info_frame_t *a, const v91_info_frame_t *b)
{
    return a->reserved_12_25 == b->reserved_12_25
        && a->request_default_dil == b->request_default_dil
        && a->request_control_channel == b->request_control_channel
        && a->acknowledge_info_frame == b->acknowledge_info_frame
        && a->reserved_29_32 == b->reserved_29_32
        && a->max_tx_power == b->max_tx_power
        && a->power_measured_after_digital_impairments == b->power_measured_after_digital_impairments
        && a->tx_uses_alaw == b->tx_uses_alaw
        && a->request_transparent_mode == b->request_transparent_mode
        && a->cleardown_if_transparent_denied == b->cleardown_if_transparent_denied;
}

static void vpcm_bits_to_str(char *out, size_t out_len, const uint8_t *bits, int nbits)
{
    int i;
    size_t pos;

    if (out_len == 0)
        return;
    pos = 0;
    for (i = 0; i < nbits && pos + 1 < out_len; i++)
        out[pos++] = bits[i] ? '1' : '0';
    out[pos] = '\0';
}

static const char *vpcm_info_law_to_str(bool tx_uses_alaw)
{
    return tx_uses_alaw ? "A-law" : "u-law";
}

static void vpcm_log_info_compare_row(const char *field, const char *tx, const char *rx)
{
    vpcm_log("| %-10s | %-18s | %-18s |", field, tx, rx);
}

static void vpcm_format_info_status(char *buf, size_t len, bool ok)
{
    snprintf(buf, len, "%s", ok ? "correct" : "incorrect");
}

static void vpcm_format_info_reserved14(char *buf, size_t len, uint16_t value)
{
    snprintf(buf, len, "0x%04x", value);
}

static void vpcm_format_info_reserved4(char *buf, size_t len, uint8_t value)
{
    snprintf(buf, len, "0x%x", value & 0x0F);
}

static void vpcm_format_info_dil(char *buf, size_t len, bool request_default_dil)
{
    snprintf(buf, len, "%s", request_default_dil ? "default" : "non-default");
}

static void vpcm_format_info_request(char *buf, size_t len, bool requested, const char *true_value, const char *false_value)
{
    snprintf(buf, len, "%s", requested ? true_value : false_value);
}

static void vpcm_format_info_power(char *buf, size_t len, uint8_t max_tx_power)
{
    if (max_tx_power == 0) {
        snprintf(buf, len, "unspecified");
    } else {
        snprintf(buf, len, "%u (%.1f dBm0)", max_tx_power, -0.5 * (double) (max_tx_power + 1));
    }
}

static void vpcm_log_info_diag_compare(const v91_info_diag_t *tx, const v91_info_diag_t *rx)
{
    char tx_bits[V91_INFO_SYMBOLS + 1];
    char rx_bits[V91_INFO_SYMBOLS + 1];
    char tx_buf[64];
    char rx_buf[64];

    vpcm_bits_to_str(tx_bits, sizeof(tx_bits), tx->bits, V91_INFO_SYMBOLS);
    vpcm_bits_to_str(rx_bits, sizeof(rx_bits), rx->bits, V91_INFO_SYMBOLS);

    vpcm_log("+------------+--------------------+--------------------+");
    vpcm_log("| Field      | TX                 | RX                 |");
    vpcm_log("+------------+--------------------+--------------------+");
    vpcm_format_info_status(tx_buf, sizeof(tx_buf), tx->fill_ok);
    vpcm_format_info_status(rx_buf, sizeof(rx_buf), rx->fill_ok);
    vpcm_log_info_compare_row("Fill", tx_buf, rx_buf);
    vpcm_format_info_status(tx_buf, sizeof(tx_buf), tx->sync_ok);
    vpcm_format_info_status(rx_buf, sizeof(rx_buf), rx->sync_ok);
    vpcm_log_info_compare_row("FS", tx_buf, rx_buf);
    vpcm_format_info_reserved14(tx_buf, sizeof(tx_buf), tx->frame.reserved_12_25);
    vpcm_format_info_reserved14(rx_buf, sizeof(rx_buf), rx->frame.reserved_12_25);
    vpcm_log_info_compare_row("RSVD12:25", tx_buf, rx_buf);
    vpcm_format_info_dil(tx_buf, sizeof(tx_buf), tx->frame.request_default_dil);
    vpcm_format_info_dil(rx_buf, sizeof(rx_buf), rx->frame.request_default_dil);
    vpcm_log_info_compare_row("DIL", tx_buf, rx_buf);
    vpcm_format_info_request(tx_buf, sizeof(tx_buf), tx->frame.request_control_channel, "request", "none");
    vpcm_format_info_request(rx_buf, sizeof(rx_buf), rx->frame.request_control_channel, "request", "none");
    vpcm_log_info_compare_row("CC", tx_buf, rx_buf);
    vpcm_format_info_request(tx_buf, sizeof(tx_buf), tx->frame.acknowledge_info_frame, "yes", "no");
    vpcm_format_info_request(rx_buf, sizeof(rx_buf), rx->frame.acknowledge_info_frame, "yes", "no");
    vpcm_log_info_compare_row("ACK", tx_buf, rx_buf);
    vpcm_format_info_reserved4(tx_buf, sizeof(tx_buf), tx->frame.reserved_29_32);
    vpcm_format_info_reserved4(rx_buf, sizeof(rx_buf), rx->frame.reserved_29_32);
    vpcm_log_info_compare_row("RSVD29:32", tx_buf, rx_buf);
    vpcm_format_info_power(tx_buf, sizeof(tx_buf), tx->frame.max_tx_power);
    vpcm_format_info_power(rx_buf, sizeof(rx_buf), rx->frame.max_tx_power);
    vpcm_log_info_compare_row("MaxPwr", tx_buf, rx_buf);
    vpcm_format_info_request(tx_buf, sizeof(tx_buf), tx->frame.power_measured_after_digital_impairments, "receiver", "terminals");
    vpcm_format_info_request(rx_buf, sizeof(rx_buf), rx->frame.power_measured_after_digital_impairments, "receiver", "terminals");
    vpcm_log_info_compare_row("PwrRef", tx_buf, rx_buf);
    snprintf(tx_buf, sizeof(tx_buf), "%s", vpcm_info_law_to_str(tx->frame.tx_uses_alaw));
    snprintf(rx_buf, sizeof(rx_buf), "%s", vpcm_info_law_to_str(rx->frame.tx_uses_alaw));
    vpcm_log_info_compare_row("PCM", tx_buf, rx_buf);
    vpcm_format_info_request(tx_buf, sizeof(tx_buf), tx->frame.request_transparent_mode, "transparent", "normal");
    vpcm_format_info_request(rx_buf, sizeof(rx_buf), rx->frame.request_transparent_mode, "transparent", "normal");
    vpcm_log_info_compare_row("Mode", tx_buf, rx_buf);
    vpcm_format_info_request(tx_buf, sizeof(tx_buf), tx->frame.cleardown_if_transparent_denied, "cleardown", "continue");
    vpcm_format_info_request(rx_buf, sizeof(rx_buf), rx->frame.cleardown_if_transparent_denied, "cleardown", "continue");
    vpcm_log_info_compare_row("Deny", tx_buf, rx_buf);
    snprintf(tx_buf, sizeof(tx_buf), "0x%04x", tx->crc_field);
    snprintf(rx_buf, sizeof(rx_buf), "0x%04x", rx->crc_field);
    vpcm_log_info_compare_row("CRC", tx_buf, rx_buf);
    snprintf(tx_buf, sizeof(tx_buf), "0x%04x", tx->crc_remainder);
    snprintf(rx_buf, sizeof(rx_buf), "0x%04x", rx->crc_remainder);
    vpcm_log_info_compare_row("CRC rem", tx_buf, rx_buf);
    vpcm_format_info_status(tx_buf, sizeof(tx_buf), tx->valid);
    vpcm_format_info_status(rx_buf, sizeof(rx_buf), rx->valid);
    vpcm_log_info_compare_row("INFO", tx_buf, rx_buf);
    vpcm_log("+------------+--------------------+--------------------+");
    vpcm_log("INFO bits TX: %s", tx_bits);
    vpcm_log("INFO bits RX: %s", rx_bits);
    if (g_vpcm_verbose)
        vpcm_log("INFO codeword[0..7] TX/RX: %02x %02x %02x %02x %02x %02x %02x %02x / %02x %02x %02x %02x %02x %02x %02x %02x",
                 tx->codewords[0], tx->codewords[1], tx->codewords[2], tx->codewords[3],
                 tx->codewords[4], tx->codewords[5], tx->codewords[6], tx->codewords[7],
                 rx->codewords[0], rx->codewords[1], rx->codewords[2], rx->codewords[3],
                 rx->codewords[4], rx->codewords[5], rx->codewords[6], rx->codewords[7]);
}

static void vpcm_trace(const char *fmt, ...)
{
    va_list ap;

    if (!g_vpcm_verbose)
        return;
    fprintf(stderr, "[VPCM:trace] ");
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
}

static void vpcm_log_v8_result(const char *side, const v8_parms_t *result)
{
    char mods[128];

    mods[0] = '\0';
    if (result->jm_cm.modulations & V8_MOD_V22)
        strncat(mods, mods[0] ? "|V22" : "V22", sizeof(mods) - strlen(mods) - 1);
    if (result->jm_cm.modulations & V8_MOD_V34)
        strncat(mods, mods[0] ? "|V34" : "V34", sizeof(mods) - strlen(mods) - 1);
    if (result->jm_cm.modulations & V8_MOD_V90)
        strncat(mods, mods[0] ? "|V90" : "V90", sizeof(mods) - strlen(mods) - 1);
    if (result->jm_cm.modulations & V8_MOD_V92)
        strncat(mods, mods[0] ? "|V92" : "V92", sizeof(mods) - strlen(mods) - 1);
    if (mods[0] == '\0')
        strlcpy(mods, "none", sizeof(mods));

    vpcm_log("V.8 %s result: status=%s (%d) call=%s mods=%s pcm=0x%X pstn=0x%X",
             side,
             v8_status_to_str(result->status), result->status,
             v8_call_function_to_str(result->jm_cm.call_function),
             mods,
             result->jm_cm.pcm_modem_availability,
             result->jm_cm.pstn_access);
}

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

static bool run_v91_info_roundtrip_case(v91_state_t *tx,
                                        v91_state_t *rx,
                                        const char *label,
                                        const v91_info_frame_t *expected_info,
                                        bool require_ack)
{
    uint8_t info_buf[V91_INFO_SYMBOLS];
    v91_info_frame_t info_rx;
    v91_info_diag_t tx_diag;
    v91_info_diag_t rx_diag;
    char validate_reason[128];

    vpcm_log("Case: %s", label);
    if (!v91_info_frame_validate(expected_info, validate_reason, sizeof(validate_reason))) {
        fprintf(stderr, "V.91 INFO validation failed for %s: %s\n", label, validate_reason);
        return false;
    }
    if (!v91_info_build_diag(tx, expected_info, &tx_diag)) {
        fprintf(stderr, "V.91 INFO TX diag build failed for %s\n", label);
        return false;
    }
    if (v91_tx_info_codewords(tx, info_buf, (int) sizeof(info_buf), expected_info) != V91_INFO_SYMBOLS) {
        fprintf(stderr, "V.91 INFO encode length mismatch for %s\n", label);
        return false;
    }
    if (!tx->last_tx_info_valid || !vpcm_info_frames_equal(expected_info, &tx->last_tx_info)) {
        fprintf(stderr, "V.91 TX tracking mismatch for %s\n", label);
        return false;
    }
    if (!v91_info_decode_diag(rx, info_buf, (int) sizeof(info_buf), &rx_diag)) {
        vpcm_log_info_diag_compare(&tx_diag, &rx_diag);
        fprintf(stderr, "V.91 INFO decode failed for %s\n", label);
        return false;
    }
    info_rx = rx_diag.frame;
    if (!rx->last_rx_info_valid || !vpcm_info_frames_equal(expected_info, &rx->last_rx_info)) {
        vpcm_log_info_diag_compare(&tx_diag, &rx_diag);
        fprintf(stderr, "V.91 RX tracking mismatch for %s\n", label);
        return false;
    }
    if (!vpcm_info_frames_equal(expected_info, &info_rx)) {
        vpcm_log_info_diag_compare(&tx_diag, &rx_diag);
        fprintf(stderr, "V.91 INFO round-trip mismatch for %s\n", label);
        return false;
    }
    if (require_ack && !info_rx.acknowledge_info_frame) {
        vpcm_log_info_diag_compare(&tx_diag, &rx_diag);
        fprintf(stderr, "V.91 INFO acknowledgement bit missing for %s\n", label);
        return false;
    }
    vpcm_log_info_diag_compare(&tx_diag, &rx_diag);
    return true;
}

static bool test_v91_bilateral_info_tracking(v91_law_t law_a, v91_law_t law_b)
{
    v91_state_t side_a;
    v91_state_t side_b;
    v91_info_frame_t info_a;
    v91_info_frame_t info_b;

    v91_init(&side_a, law_a, V91_MODE_TRANSPARENT);
    v91_init(&side_b, law_b, V91_MODE_TRANSPARENT);
    vpcm_log("Test: V.91 bilateral INFO tracking (%s -> %s)",
             vpcm_law_to_str(law_a), vpcm_law_to_str(law_b));

    memset(&info_a, 0, sizeof(info_a));
    info_a.reserved_12_25 = 0x0155;
    info_a.request_default_dil = true;
    info_a.request_control_channel = false;
    info_a.acknowledge_info_frame = false;
    info_a.reserved_29_32 = 0x3;
    info_a.max_tx_power = 7;
    info_a.power_measured_after_digital_impairments = true;
    info_a.tx_uses_alaw = (law_a == V91_LAW_ALAW);
    info_a.request_transparent_mode = true;
    info_a.cleardown_if_transparent_denied = true;

    memset(&info_b, 0, sizeof(info_b));
    info_b.reserved_12_25 = 0x02aa;
    info_b.request_default_dil = false;
    info_b.request_control_channel = true;
    info_b.acknowledge_info_frame = true;
    info_b.reserved_29_32 = 0xc;
    info_b.max_tx_power = 21;
    info_b.power_measured_after_digital_impairments = false;
    info_b.tx_uses_alaw = (law_b == V91_LAW_ALAW);
    info_b.request_transparent_mode = false;
    info_b.cleardown_if_transparent_denied = false;

    if (!run_v91_info_roundtrip_case(&side_a, &side_b, "A -> B INFO", &info_a, false))
        return false;
    if (!run_v91_info_roundtrip_case(&side_b, &side_a, "B -> A INFO", &info_b, true))
        return false;

    if (!side_a.last_tx_info_valid
        || !side_a.last_rx_info_valid
        || !vpcm_info_frames_equal(&info_a, &side_a.last_tx_info)
        || !vpcm_info_frames_equal(&info_b, &side_a.last_rx_info)) {
        fprintf(stderr, "V.91 bilateral tracking mismatch on side A\n");
        return false;
    }
    if (!side_b.last_tx_info_valid
        || !side_b.last_rx_info_valid
        || !vpcm_info_frames_equal(&info_b, &side_b.last_tx_info)
        || !vpcm_info_frames_equal(&info_a, &side_b.last_rx_info)) {
        fprintf(stderr, "V.91 bilateral tracking mismatch on side B\n");
        return false;
    }

    vpcm_log("PASS: V.91 bilateral INFO tracking (%s -> %s)",
             vpcm_law_to_str(law_a), vpcm_law_to_str(law_b));
    return true;
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
    vpcm_log("Starting V.8 exchange over %s using %s",
             vpcm_path_mode_to_str(analog_channel.mode),
             vpcm_law_to_str(law));

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
        if (chunk == 0 || ((chunk + 1) % 50) == 0) {
            vpcm_trace("V.8 exchange progress: chunk=%d caller_seen=%d answer_seen=%d",
                       chunk + 1, caller_result->seen, answer_result->seen);
        }

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
    vpcm_log_v8_result("caller", &caller_result->result);
    vpcm_log_v8_result("answerer", &answer_result->result);
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
    vpcm_log("Test: V.8 V.90 startup over analog-over-G.711 (%s)", vpcm_law_to_str(law));
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

    vpcm_log("PASS: V.8 V.90 startup over analog-over-G.711 (%s)", vpcm_law_to_str(law));
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
    vpcm_log("Test: V.8 V.91 advertisement over analog-over-G.711 (%s)", vpcm_law_to_str(law));
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

    vpcm_log("PASS: V.8 V.91 advertisement over analog-over-G.711 (%s)", vpcm_law_to_str(law));
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
    vpcm_log("Test: V.91 codeword loopback over %s (%s)",
             vpcm_path_mode_to_str(pcm_channel.mode), vpcm_law_to_str(law));
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
        if (chunk_seed == 17 || ((in_pos / TEST_CHUNK_MAX) % 16) == 0) {
            vpcm_trace("V.91 codeword loopback progress: in=%d out=%d chunk=%d",
                       in_pos, out_pos, want);
        }
        if (produced != want || consumed != want) {
            fprintf(stderr, "V.91 codeword loopback length mismatch: tx=%d rx=%d want=%d\n",
                    produced, consumed, want);
            return false;
        }

        in_pos += want;
        out_pos += consumed;
    }

    vpcm_log("PASS: V.91 codeword loopback over %s (%s), payload=%d bytes",
             vpcm_path_mode_to_str(pcm_channel.mode), vpcm_law_to_str(law), TEST_PAYLOAD_LEN);
    return expect_equal("V.91 codeword loopback", input, output, TEST_PAYLOAD_LEN);
}

static bool test_v91_startup_primitives(v91_law_t law)
{
    v91_state_t tx;
    v91_state_t rx;
    uint8_t silence[V91_PHASE1_SILENCE_SYMBOLS];
    uint8_t ez[V91_EZ_SYMBOLS];
    uint8_t expected_silence;
    uint8_t expected_ez;
    v91_info_frame_t info_tx;
    uint32_t random_state;
    int i;

    v91_init(&tx, law, V91_MODE_TRANSPARENT);
    v91_init(&rx, law, V91_MODE_TRANSPARENT);

    vpcm_log("Test: V.91 startup primitives over raw-G.711 (%s)", vpcm_law_to_str(law));

    if (v91_tx_phase1_silence_codewords(&tx, silence, (int) sizeof(silence)) != V91_PHASE1_SILENCE_SYMBOLS) {
        fprintf(stderr, "V.91 phase1 silence length mismatch\n");
        return false;
    }
    expected_silence = v91_ucode_to_codeword(law, 0, true);
    for (i = 0; i < V91_PHASE1_SILENCE_SYMBOLS; i++) {
        if (silence[i] != expected_silence) {
            fprintf(stderr, "V.91 phase1 silence mismatch at %d: %02X != %02X\n",
                    i, silence[i], expected_silence);
            return false;
        }
    }

    if (v91_tx_ez_codewords(&tx, ez, (int) sizeof(ez)) != V91_EZ_SYMBOLS) {
        fprintf(stderr, "V.91 Ez length mismatch\n");
        return false;
    }
    expected_ez = v91_ucode_to_codeword(law, 66, false);
    for (i = 0; i < V91_EZ_SYMBOLS; i++) {
        if (ez[i] != expected_ez) {
            fprintf(stderr, "V.91 Ez mismatch at %d: %02X != %02X\n",
                    i, ez[i], expected_ez);
            return false;
        }
    }

    memset(&info_tx, 0, sizeof(info_tx));
    info_tx.reserved_12_25 = 0x0000;
    info_tx.request_default_dil = true;
    info_tx.request_control_channel = false;
    info_tx.acknowledge_info_frame = false;
    info_tx.reserved_29_32 = 0x0;
    info_tx.max_tx_power = 0;
    info_tx.power_measured_after_digital_impairments = true;
    info_tx.tx_uses_alaw = (law == V91_LAW_ALAW);
    info_tx.request_transparent_mode = true;
    info_tx.cleardown_if_transparent_denied = true;

    if (!run_v91_info_roundtrip_case(&tx, &rx, "INFO", &info_tx, false))
        return false;

    info_tx.acknowledge_info_frame = true;
    if (!run_v91_info_roundtrip_case(&tx, &rx, "INFO'", &info_tx, true))
        return false;

    random_state = 0x91C0DE00U ^ (uint32_t) law;
    memset(&info_tx, 0, sizeof(info_tx));
    info_tx.reserved_12_25 = (uint16_t) (prng_next(&random_state) & 0x3FFFU);
    info_tx.request_default_dil = false;
    info_tx.request_control_channel = true;
    info_tx.acknowledge_info_frame = false;
    info_tx.reserved_29_32 = (uint8_t) (prng_next(&random_state) & 0x0FU);
    info_tx.max_tx_power = (uint8_t) (1 + (prng_next(&random_state) % 31U));
    info_tx.power_measured_after_digital_impairments = false;
    info_tx.tx_uses_alaw = (law == V91_LAW_ALAW);
    info_tx.request_transparent_mode = false;
    info_tx.cleardown_if_transparent_denied = false;
    if (!run_v91_info_roundtrip_case(&tx, &rx, "INFO variant (non-default/random power)", &info_tx, false))
        return false;

    vpcm_log("PASS: V.91 startup primitives over raw-G.711 (%s)", vpcm_law_to_str(law));
    return true;
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
    vpcm_log("Test: V.91 linear simulation loopback over analog-over-G.711 (%s)",
             vpcm_law_to_str(law));
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
        if (chunk_seed == 29 || ((in_pos / TEST_CHUNK_MAX) % 16) == 0) {
            vpcm_trace("V.91 linear loopback progress: in=%d out=%d chunk=%d",
                       in_pos, out_pos, want);
        }
        if (produced != want || consumed != want) {
            fprintf(stderr, "V.91 linear loopback length mismatch: tx=%d rx=%d want=%d\n",
                    produced, consumed, want);
            return false;
        }

        in_pos += want;
        out_pos += consumed;
    }

    vpcm_log("PASS: V.91 linear simulation loopback over analog-over-G.711 (%s), payload=%d bytes",
             vpcm_law_to_str(law), TEST_PAYLOAD_LEN);
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
    vpcm_log("Test: V.91 full duplex loopback over raw-G.711 (%s)", vpcm_law_to_str(law));

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
        if (stride == 97 || ((a_in_pos / 512) != ((a_in_pos + a_want) / 512))) {
            vpcm_trace("V.91 full duplex progress: a_in=%d b_in=%d a_out=%d b_out=%d",
                       a_in_pos, b_in_pos, a_out_pos, b_out_pos);
        }

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

    vpcm_log("PASS: V.91 full duplex loopback over raw-G.711 (%s), payload=%d bytes each way",
             vpcm_law_to_str(law), TEST_PAYLOAD_LEN);
    return expect_equal("V.91 full duplex A<-B", b_input, a_output, TEST_PAYLOAD_LEN)
        && expect_equal("V.91 full duplex B<-A", a_input, b_output, TEST_PAYLOAD_LEN);
}

int main(void)
{
    const char *verbose_env;

    verbose_env = getenv("VPCM_VERBOSE");
    g_vpcm_verbose = (verbose_env != NULL && verbose_env[0] != '\0' && strcmp(verbose_env, "0") != 0);
    vpcm_log("PCM modem loopback harness starting (verbose=%s)",
             g_vpcm_verbose ? "on" : "off");

    if (!test_v91_codeword_loopback(V91_LAW_ULAW))
        return 1;
    if (!test_v91_codeword_loopback(V91_LAW_ALAW))
        return 1;
    if (!test_v91_startup_primitives(V91_LAW_ULAW))
        return 1;
    if (!test_v91_startup_primitives(V91_LAW_ALAW))
        return 1;
    if (!test_v91_bilateral_info_tracking(V91_LAW_ALAW, V91_LAW_ULAW))
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

    vpcm_log("All PCM modem loopback tests passed");
    puts("vpcm_loopback_test: OK");
    return 0;
}
