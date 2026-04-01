/*
 * vpcm_decode.c — Offline V.PCM signal decoder
 *
 * Reads a WAV (stereo s16le, 8kHz) or raw G.711 file and decodes
 * V.8, V.90, V.91, and V.92 protocol signals found in the stream.
 *
 * Usage:
 *   vpcm_decode [--law ulaw|alaw] [--wav] [--g711] [--channel L|R|mono]
 *               [--v8] [--v91] [--v90] [--all] <file>
 *
 * Default: --all --law ulaw; format auto-detected from extension.
 */

#include "v90.h"
#include "v91.h"
#include "vpcm_cp.h"

#include <spandsp.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static uint16_t read_le16(const uint8_t *p)
{
    return (uint16_t) p[0] | ((uint16_t) p[1] << 8);
}

static uint32_t read_le32(const uint8_t *p)
{
    return (uint32_t) p[0]
         | ((uint32_t) p[1] << 8)
         | ((uint32_t) p[2] << 16)
         | ((uint32_t) p[3] << 24);
}

static double sample_to_ms(int sample, int rate)
{
    return (double) sample * 1000.0 / (double) rate;
}

typedef struct {
    int saved_stderr_fd;
    bool active;
} stderr_silence_guard_t;

static stderr_silence_guard_t silence_stderr_begin(void)
{
    stderr_silence_guard_t guard = { -1, false };

    fflush(stderr);
    guard.saved_stderr_fd = dup(STDERR_FILENO);
    if (guard.saved_stderr_fd < 0)
        return guard;

    if (!freopen("/dev/null", "a", stderr)) {
        close(guard.saved_stderr_fd);
        guard.saved_stderr_fd = -1;
        return guard;
    }

    guard.active = true;
    return guard;
}

static void silence_stderr_end(stderr_silence_guard_t *guard)
{
    if (!guard || guard->saved_stderr_fd < 0)
        return;

    fflush(stderr);
    dup2(guard->saved_stderr_fd, STDERR_FILENO);
    clearerr(stderr);
    close(guard->saved_stderr_fd);
    guard->saved_stderr_fd = -1;
    guard->active = false;
}

/* ------------------------------------------------------------------ */
/* Call log event collection                                           */
/* ------------------------------------------------------------------ */

typedef struct {
    int sample_offset;
    int duration_samples;
    char protocol[24];
    char summary[160];
    char detail[320];
} call_log_event_t;

typedef struct {
    call_log_event_t *events;
    size_t count;
    size_t cap;
} call_log_t;

static void call_log_init(call_log_t *log)
{
    if (!log)
        return;
    log->events = NULL;
    log->count = 0;
    log->cap = 0;
}

static void call_log_reset(call_log_t *log)
{
    if (!log)
        return;
    free(log->events);
    log->events = NULL;
    log->count = 0;
    log->cap = 0;
}

static bool call_log_append(call_log_t *log,
                            int sample_offset,
                            int duration_samples,
                            const char *protocol,
                            const char *summary,
                            const char *detail)
{
    call_log_event_t *next;

    if (!log || !protocol || !summary)
        return false;
    if (log->count == log->cap) {
        size_t new_cap = (log->cap == 0) ? 16 : log->cap * 2;
        next = realloc(log->events, new_cap * sizeof(*next));
        if (!next)
            return false;
        log->events = next;
        log->cap = new_cap;
    }

    log->events[log->count].sample_offset = sample_offset;
    log->events[log->count].duration_samples = duration_samples;
    snprintf(log->events[log->count].protocol,
             sizeof(log->events[log->count].protocol),
             "%s",
             protocol);
    snprintf(log->events[log->count].summary,
             sizeof(log->events[log->count].summary),
             "%s",
             summary);
    snprintf(log->events[log->count].detail,
             sizeof(log->events[log->count].detail),
             "%s",
             detail ? detail : "");
    log->count++;
    return true;
}

static int call_log_event_cmp(const void *a, const void *b)
{
    const call_log_event_t *ea = a;
    const call_log_event_t *eb = b;

    if (ea->sample_offset < eb->sample_offset)
        return -1;
    if (ea->sample_offset > eb->sample_offset)
        return 1;
    if (ea->duration_samples < eb->duration_samples)
        return -1;
    if (ea->duration_samples > eb->duration_samples)
        return 1;
    return strcmp(ea->protocol, eb->protocol);
}

static void call_log_sort(call_log_t *log)
{
    if (!log || log->count < 2)
        return;
    qsort(log->events, log->count, sizeof(log->events[0]), call_log_event_cmp);
}

static void format_v91_info_summary(char *buf, size_t len, const v91_info_diag_t *diag)
{
    if (!buf || len == 0 || !diag) {
        return;
    }
    snprintf(buf, len,
             "default_dil=%s transparent=%s control=%s alaw=%s power=%u",
             diag->frame.request_default_dil ? "yes" : "no",
             diag->frame.request_transparent_mode ? "yes" : "no",
             diag->frame.request_control_channel ? "yes" : "no",
             diag->frame.tx_uses_alaw ? "yes" : "no",
             (unsigned) diag->frame.max_tx_power);
}

static bool map_v34_received_info0a(v90_info0a_t *dst, const v34_v90_info0a_t *src)
{
    if (!dst || !src)
        return false;

    memset(dst, 0, sizeof(*dst));
    dst->support_2743 = src->support_2743;
    dst->support_2800 = src->support_2800;
    dst->support_3429 = src->support_3429;
    dst->support_3000_low = src->support_3000_low;
    dst->support_3000_high = src->support_3000_high;
    dst->support_3200_low = src->support_3200_low;
    dst->support_3200_high = src->support_3200_high;
    dst->rate_3429_allowed = src->rate_3429_allowed;
    dst->support_power_reduction = src->support_power_reduction;
    dst->max_baud_rate_difference = src->max_baud_rate_difference;
    dst->from_cme_modem = src->from_cme_modem;
    dst->support_1664_point_constellation = src->support_1664_point_constellation;
    dst->tx_clock_source = src->tx_clock_source;
    dst->acknowledge_info0d = src->acknowledge_info0d;
    return v90_info0a_validate(dst);
}

static bool map_v34_received_info1a(v90_info1a_t *dst, const v34_v90_info1a_t *src)
{
    if (!dst || !src)
        return false;

    memset(dst, 0, sizeof(*dst));
    dst->md = (uint8_t) src->md;
    dst->u_info = (uint8_t) src->u_info;
    dst->upstream_symbol_rate_code = (uint8_t) src->upstream_symbol_rate_code;
    dst->downstream_rate_code = (uint8_t) src->downstream_rate_code;
    dst->freq_offset = (int16_t) src->freq_offset;
    return v90_info1a_validate(dst);
}

static void format_cp_summary(char *buf, size_t len, const vpcm_cp_frame_t *cp)
{
    if (!buf || len == 0 || !cp)
        return;
    snprintf(buf, len,
             "transparent=%s ack=%s drn=%u rate=%.0f bps constellations=%u",
             cp->transparent_mode_granted ? "yes" : "no",
             cp->acknowledge ? "yes" : "no",
             (unsigned) cp->drn,
             vpcm_cp_drn_to_bps(cp->drn),
             (unsigned) cp->constellation_count);
}

static void format_v91_dil_summary(char *buf, size_t len, const v91_dil_analysis_t *analysis)
{
    if (!buf || len == 0 || !analysis)
        return;
    snprintf(buf, len,
             "default_like=%s impairment=%u downstream=%.0f bps upstream=%.0f bps",
             analysis->default_like ? "yes" : "no",
             (unsigned) analysis->impairment_score,
             vpcm_cp_drn_to_bps(analysis->recommended_downstream_drn),
             vpcm_cp_drn_to_bps(analysis->recommended_upstream_drn));
}

static void format_v90_dil_summary(char *buf, size_t len, const v90_dil_analysis_t *analysis)
{
    if (!buf || len == 0 || !analysis)
        return;
    snprintf(buf, len,
             "default_125x12=%s impairment=%u downstream=%.0f bps upstream=%.0f bps",
             analysis->looks_default_125x12 ? "yes" : "no",
             (unsigned) analysis->impairment_score,
             vpcm_cp_drn_to_bps(analysis->recommended_downstream_drn),
             vpcm_cp_drn_to_bps(analysis->recommended_upstream_drn));
}

static void print_call_log(const char *label,
                           const call_log_t *log,
                           int total_samples,
                           int sample_rate)
{
    int covered_until = 0;
    int coverage = 0;

    if (!label || !log)
        return;

    printf("\n=== Call Log: %s ===\n", label);
    printf("  Duration: %.3f seconds\n", (double) total_samples / (double) sample_rate);
    printf("  Recognized events: %zu\n", log->count);

    if (log->count == 0) {
        printf("  No implemented call phases were recognized.\n");
        return;
    }

    for (size_t i = 0; i < log->count; i++) {
        int start = log->events[i].sample_offset;
        int end = start + log->events[i].duration_samples;
        if (end > covered_until) {
            if (start < covered_until)
                start = covered_until;
            coverage += end - start;
            covered_until = end;
        }
    }

    printf("  Approximate covered time: %.1f ms (%.1f%%)\n",
           sample_to_ms(coverage, sample_rate),
           total_samples > 0 ? (100.0 * (double) coverage / (double) total_samples) : 0.0);

    covered_until = 0;
    for (size_t i = 0; i < log->count; i++) {
        const call_log_event_t *event = &log->events[i];
        double start_ms = sample_to_ms(event->sample_offset, sample_rate);
        double dur_ms = sample_to_ms(event->duration_samples, sample_rate);

        if (event->sample_offset > covered_until + (sample_rate / 20)) {
            printf("  [%7.1f ms] GAP: no implemented decoder matched for %.1f ms\n",
                   sample_to_ms(covered_until, sample_rate),
                   sample_to_ms(event->sample_offset - covered_until, sample_rate));
        }

        printf("  [%7.1f ms] %-5s %s",
               start_ms,
               event->protocol,
               event->summary);
        if (event->duration_samples > 0)
            printf(" (%.1f ms)", dur_ms);
        printf("\n");
        if (event->detail[0] != '\0')
            printf("             %s\n", event->detail);

        if (event->sample_offset + event->duration_samples > covered_until)
            covered_until = event->sample_offset + event->duration_samples;
    }

    if (covered_until < total_samples - (sample_rate / 20)) {
        printf("  [%7.1f ms] GAP: trailing undecoded span of %.1f ms\n",
               sample_to_ms(covered_until, sample_rate),
               sample_to_ms(total_samples - covered_until, sample_rate));
    }
}

/* ------------------------------------------------------------------ */
/* WAV reader                                                          */
/* ------------------------------------------------------------------ */

typedef struct {
    int channels;
    int sample_rate;
    int bits_per_sample;
    long data_offset;
    long data_bytes;
} wav_info_t;

static bool wav_read_header(FILE *f, wav_info_t *info)
{
    uint8_t hdr[44];

    if (fread(hdr, 1, 44, f) != 44)
        return false;
    if (memcmp(hdr, "RIFF", 4) != 0 || memcmp(hdr + 8, "WAVE", 4) != 0)
        return false;
    if (memcmp(hdr + 12, "fmt ", 4) != 0)
        return false;

    info->channels = read_le16(hdr + 22);
    info->sample_rate = (int) read_le32(hdr + 24);
    info->bits_per_sample = read_le16(hdr + 34);

    /* Find data chunk — may not be at offset 36 if extra fmt bytes */
    long pos = 12;
    fseek(f, 12, SEEK_SET);
    while (1) {
        uint8_t chunk_hdr[8];
        if (fread(chunk_hdr, 1, 8, f) != 8)
            return false;
        uint32_t chunk_size = read_le32(chunk_hdr + 4);
        if (memcmp(chunk_hdr, "data", 4) == 0) {
            info->data_offset = ftell(f);
            info->data_bytes = (long) chunk_size;
            return true;
        }
        fseek(f, (long) chunk_size, SEEK_CUR);
        pos = ftell(f);
        (void) pos;
    }
}

/* ------------------------------------------------------------------ */
/* RMS energy measurement                                              */
/* ------------------------------------------------------------------ */

static double rms_energy_db(const int16_t *samples, int len)
{
    if (len <= 0) return -100.0;
    double sum = 0.0;
    for (int i = 0; i < len; i++)
        sum += (double) samples[i] * (double) samples[i];
    double rms = sqrt(sum / (double) len);
    if (rms < 1.0) return -100.0;
    return 20.0 * log10(rms / 32768.0);
}

/* ------------------------------------------------------------------ */
/* V.8 decode pass                                                     */
/* ------------------------------------------------------------------ */

typedef struct {
    bool seen;
    v8_parms_t result;
} decode_v8_result_t;

typedef struct {
    bool info0_seen;
    bool info1_seen;
    bool phase3_seen;
    bool phase4_ready_seen;
    bool phase4_seen;
    bool training_failed;
    int info0_sample;
    int info1_sample;
    int phase3_sample;
    int phase4_ready_sample;
    int phase4_sample;
    int failure_sample;
    int final_rx_stage;
    int final_tx_stage;
    int final_rx_event;
    int u_info;
    v90_info0a_t info0a;
    v90_info1a_t info1a;
} decode_v34_result_t;

typedef struct {
    bool adaptive_used;
    double gain;
    int bias;
    int direct_v90_score;
    int selected_v90_score;
} codeword_stream_info_t;

typedef struct {
    bool ok;
    bool calling_party;
    int start_sample;
    int u_info;
    int decoded_octets;
    int preview_len;
    int printable_octets;
    uint8_t preview[32];
} post_phase3_decode_t;

typedef struct {
    bool ok;
    bool calling_party;
    int u_info;
    int phase3_start_sample;
    int jd_start_sample;
    int jd_frame_errors;
    int jd_repetitions;
    bool jd_prime_seen;
    int jd_prime_sample;
    int jd_prime_zero_count;
} jd_stage_decode_t;

#define OFFLINE_V90_SD_SYMBOLS      (64 * 6)
#define OFFLINE_V90_SBAR_SYMBOLS    (8 * 6)
#define OFFLINE_V90_TRN1D_SYMBOLS   2046
#define OFFLINE_V90_JD_BITS         72
#define OFFLINE_V90_JD_PRIME_BITS   12

static const decode_v34_result_t *pick_post_phase3_source(const decode_v34_result_t *answerer,
                                                          const decode_v34_result_t *caller,
                                                          bool *calling_party_out);
static bool decode_jd_stage(const uint8_t *codewords,
                            int total_codewords,
                            const decode_v34_result_t *answerer,
                            const decode_v34_result_t *caller,
                            jd_stage_decode_t *out);

static decode_v8_result_t g_v8_result;

static void v8_decode_result_handler(void *user_data, v8_parms_t *result)
{
    (void) user_data;
    g_v8_result.seen = true;
    g_v8_result.result = *result;
}

static void print_v8_modulations(int mods)
{
    if (mods & V8_MOD_V17)    printf("  V.17");
    if (mods & V8_MOD_V21)    printf("  V.21");
    if (mods & V8_MOD_V22)    printf("  V.22bis");
    if (mods & V8_MOD_V23)    printf("  V.23");
    if (mods & V8_MOD_V26BIS) printf("  V.26bis");
    if (mods & V8_MOD_V26TER) printf("  V.26ter");
    if (mods & V8_MOD_V27TER) printf("  V.27ter");
    if (mods & V8_MOD_V29)    printf("  V.29");
    if (mods & V8_MOD_V32)    printf("  V.32bis");
    if (mods & V8_MOD_V34)    printf("  V.34");
    if (mods & V8_MOD_V90)    printf("  V.90");
    if (mods & V8_MOD_V92)    printf("  V.92");
    printf("\n");
}

static void print_v8_result(const v8_parms_t *r)
{
    printf("  Status:         %s\n", v8_status_to_str(r->status));
    printf("  Modulations:   ");
    print_v8_modulations(r->jm_cm.modulations);
    printf("  Protocol:       %s\n", v8_protocol_to_str(r->jm_cm.protocols));
    printf("  PSTN access:    %s\n", v8_pstn_access_to_str(r->jm_cm.pstn_access));
    printf("  PCM modem:      %s\n",
           v8_pcm_modem_availability_to_str(r->jm_cm.pcm_modem_availability));
    if (r->v92 >= 0)
        printf("  V.92:           0x%02x\n", r->v92);
}

static int v34_dummy_get_bit(void *user_data)
{
    (void) user_data;
    return 1;
}

static void v34_dummy_put_bit(void *user_data, int bit)
{
    (void) user_data;
    (void) bit;
}

static const char *v34_rx_stage_to_str_local(int stage)
{
    switch (stage) {
    case 1: return "INFO0";
    case 2: return "INFOH";
    case 3: return "INFO1C";
    case 4: return "INFO1A";
    case 5: return "TONE_A";
    case 6: return "TONE_B";
    case 7: return "L1_L2";
    case 8: return "CC";
    case 9: return "PRIMARY_CHANNEL";
    case 10: return "PHASE3_WAIT_S";
    case 11: return "PHASE3_TRAINING";
    case 12: return "PHASE3_DONE";
    case 13: return "PHASE4_S";
    case 14: return "PHASE4_S_BAR";
    case 15: return "PHASE4_TRN";
    case 16: return "PHASE4_MP";
    case 17: return "DATA";
    default: return "UNKNOWN";
    }
}

static const char *v34_tx_stage_to_str_local(int stage)
{
    switch (stage) {
    case 1: return "INITIAL_PREAMBLE";
    case 2: return "INFO0";
    case 3: return "INITIAL_A";
    case 4: return "FIRST_A";
    case 5: return "FIRST_NOT_A";
    case 6: return "FIRST_NOT_A_REV";
    case 7: return "SECOND_A";
    case 8: return "L1";
    case 9: return "L2";
    case 10: return "POST_L2_A";
    case 11: return "POST_L2_NOT_A";
    case 12: return "A_SILENCE";
    case 13: return "PRE_INFO1_A";
    case 14: return "V90_WAIT_TONE_A";
    case 15: return "V90_WAIT_INFO1A";
    case 16: return "V90_WAIT_RX_L2";
    case 17: return "V90_WAIT_TONE_A_REV";
    case 18: return "V90_B_REV_DELAY";
    case 19: return "V90_B_REV_10MS";
    case 20: return "V90_PHASE2_B";
    case 21: return "V90_PHASE2_B_INFO0_SEEN";
    case 22: return "INFO1";
    case 23: return "FIRST_B";
    case 24: return "FIRST_B_INFO_SEEN";
    case 25: return "FIRST_NOT_B_WAIT";
    case 26: return "FIRST_NOT_B";
    case 27: return "FIRST_B_SILENCE";
    case 28: return "FIRST_B_POST_REV";
    case 29: return "SECOND_B";
    case 30: return "SECOND_B_WAIT";
    case 31: return "SECOND_NOT_B";
    case 32: return "INFO0_RETRY";
    case 33: return "FIRST_S";
    case 34: return "FIRST_NOT_S";
    case 35: return "MD";
    case 36: return "SECOND_S";
    case 37: return "SECOND_NOT_S";
    case 38: return "TRN";
    case 39: return "J";
    case 40: return "J_DASHED";
    case 41: return "PHASE4_WAIT";
    case 42: return "PHASE4_S";
    case 43: return "PHASE4_NOT_S";
    case 44: return "PHASE4_TRN";
    case 45: return "MP";
    default: return "UNKNOWN";
    }
}

static const char *v34_event_to_str_local(int event)
{
    switch (event) {
    case 0: return "NONE";
    case 1: return "TONE_SEEN";
    case 2: return "REVERSAL_1";
    case 3: return "REVERSAL_2";
    case 4: return "REVERSAL_3";
    case 5: return "INFO0_OK";
    case 6: return "INFO0_BAD";
    case 7: return "INFO1_OK";
    case 8: return "INFO1_BAD";
    case 9: return "INFOH_OK";
    case 10: return "INFOH_BAD";
    case 11: return "L2_SEEN";
    case 12: return "S";
    case 13: return "J";
    case 14: return "J_DASHED";
    case 15: return "PHASE4_TRN_READY";
    case 16: return "TRAINING_FAILED";
    default: return "UNKNOWN";
    }
}

static void decode_v8_pass(const int16_t *samples, int total_samples,
                           bool calling_party)
{
    v8_parms_t parms;
    v8_state_t *v8;

    memset(&parms, 0, sizeof(parms));
    parms.modem_connect_tone = calling_party
        ? MODEM_CONNECT_TONES_NONE
        : MODEM_CONNECT_TONES_ANSAM;
    parms.send_ci = calling_party;
    parms.v92 = -1;

    parms.jm_cm.modulations = V8_MOD_V90 | V8_MOD_V34 | V8_MOD_V22
                            | V8_MOD_V92 | V8_MOD_V21 | V8_MOD_V17
                            | V8_MOD_V29 | V8_MOD_V27TER | V8_MOD_V23
                            | V8_MOD_V32;
    parms.jm_cm.protocols = V8_PROTOCOL_LAPM_V42;
    parms.jm_cm.pstn_access = V8_PSTN_ACCESS_DCE_ON_DIGITAL;
    parms.jm_cm.pcm_modem_availability = V8_PSTN_PCM_MODEM_V90_V92_DIGITAL;

    memset(&g_v8_result, 0, sizeof(g_v8_result));

    v8 = v8_init(NULL, calling_party, &parms, v8_decode_result_handler, NULL);
    if (!v8) {
        fprintf(stderr, "  V.8 init failed\n");
        return;
    }
    span_log_set_level(v8_get_logging_state(v8), SPAN_LOG_SHOW_SEVERITY | SPAN_LOG_WARNING);

    /* Feed samples in 160-sample chunks */
    int offset = 0;
    while (offset < total_samples && !g_v8_result.seen) {
        int chunk = total_samples - offset;
        if (chunk > 160) chunk = 160;
        /* v8_rx takes non-const int16_t* in some SpanDSP builds */
        v8_rx(v8, (int16_t *)(samples + offset), chunk);
        /* Also drive TX so the V.8 state machine progresses */
        int16_t tx_buf[160];
        v8_tx(v8, tx_buf, chunk);
        offset += chunk;
    }

    if (g_v8_result.seen) {
        printf("  V.8 negotiation decoded at ~%.1f ms:\n",
               sample_to_ms(offset, 8000));
        print_v8_result(&g_v8_result.result);
    } else {
        printf("  No V.8 negotiation detected in %d samples (%.1f ms)\n",
               total_samples, sample_to_ms(total_samples, 8000));
    }

    v8_free(v8);
}

static void collect_v8_event(call_log_t *log,
                             const int16_t *samples,
                             int total_samples,
                             bool calling_party)
{
    v8_parms_t parms;
    v8_state_t *v8;
    int offset = 0;
    char summary[160];
    char detail[320];

    if (!log || !samples || total_samples <= 0)
        return;

    memset(&parms, 0, sizeof(parms));
    parms.modem_connect_tone = calling_party
        ? MODEM_CONNECT_TONES_NONE
        : MODEM_CONNECT_TONES_ANSAM;
    parms.send_ci = calling_party;
    parms.v92 = -1;
    parms.jm_cm.modulations = V8_MOD_V90 | V8_MOD_V34 | V8_MOD_V22
                            | V8_MOD_V92 | V8_MOD_V21 | V8_MOD_V17
                            | V8_MOD_V29 | V8_MOD_V27TER | V8_MOD_V23
                            | V8_MOD_V32;
    parms.jm_cm.protocols = V8_PROTOCOL_LAPM_V42;
    parms.jm_cm.pstn_access = V8_PSTN_ACCESS_DCE_ON_DIGITAL;
    parms.jm_cm.pcm_modem_availability = V8_PSTN_PCM_MODEM_V90_V92_DIGITAL;

    memset(&g_v8_result, 0, sizeof(g_v8_result));
    v8 = v8_init(NULL, calling_party, &parms, v8_decode_result_handler, NULL);
    if (!v8)
        return;

    span_log_set_level(v8_get_logging_state(v8), SPAN_LOG_SHOW_SEVERITY | SPAN_LOG_WARNING);
    while (offset < total_samples && !g_v8_result.seen) {
        int chunk = total_samples - offset;
        int16_t tx_buf[160];

        if (chunk > 160)
            chunk = 160;
        v8_rx(v8, (int16_t *)(samples + offset), chunk);
        v8_tx(v8, tx_buf, chunk);
        offset += chunk;
    }

    if (g_v8_result.seen) {
        snprintf(summary, sizeof(summary),
                 "V.8 negotiation decoded as %s",
                 calling_party ? "caller" : "answerer");
        snprintf(detail, sizeof(detail),
                 "status=%s protocol=%s pcm=%s pstn=%s",
                 v8_status_to_str(g_v8_result.result.status),
                 v8_protocol_to_str(g_v8_result.result.jm_cm.protocols),
                 v8_pcm_modem_availability_to_str(g_v8_result.result.jm_cm.pcm_modem_availability),
                 v8_pstn_access_to_str(g_v8_result.result.jm_cm.pstn_access));
        call_log_append(log, offset, 0, "V.8", summary, detail);
    }

    v8_free(v8);
}

static bool decode_v34_pass(const int16_t *samples,
                            int total_samples,
                            v91_law_t law,
                            bool calling_party,
                            decode_v34_result_t *result)
{
    v34_state_t *v34;
    v34_v90_info0a_t raw_info0a;
    v34_v90_info1a_t raw_info1a;
    stderr_silence_guard_t stderr_guard;
    int offset = 0;

    if (!samples || total_samples <= 0 || !result)
        return false;

    memset(result, 0, sizeof(*result));
    result->info0_sample = -1;
    result->info1_sample = -1;
    result->phase3_sample = -1;
    result->phase4_ready_sample = -1;
    result->phase4_sample = -1;
    result->failure_sample = -1;

    v34 = v34_init(NULL, 3200, 21600, calling_party, true,
                   v34_dummy_get_bit, NULL,
                   v34_dummy_put_bit, NULL);
    if (!v34)
        return false;

    v34_set_v90_mode(v34, law == V91_LAW_ALAW ? 1 : 0);
    span_log_set_level(v34_get_logging_state(v34), SPAN_LOG_SHOW_SEVERITY | SPAN_LOG_WARNING);
    stderr_guard = silence_stderr_begin();

    while (offset < total_samples) {
        int chunk = total_samples - offset;
        int16_t tx_buf[160];
        int rx_stage;
        int tx_stage;
        int rx_event;

        if (chunk > 160)
            chunk = 160;

        v34_rx(v34, samples + offset, chunk);
        v34_tx(v34, tx_buf, chunk);
        offset += chunk;

        rx_stage = v34_get_rx_stage(v34);
        tx_stage = v34_get_tx_stage(v34);
        rx_event = v34_get_rx_event(v34);

        if (!result->info0_seen
            && v34_get_v90_received_info0a(v34, &raw_info0a) > 0
            && map_v34_received_info0a(&result->info0a, &raw_info0a)) {
            result->info0_seen = true;
            result->info0_sample = offset;
        }
        if (!result->info1_seen
            && v34_get_v90_received_info1a(v34, &raw_info1a) > 0
            && map_v34_received_info1a(&result->info1a, &raw_info1a)) {
            result->info1_seen = true;
            result->info1_sample = offset;
            result->u_info = result->info1a.u_info;
        }
        if (!result->phase3_seen
            && (rx_stage >= 11 || v34_get_primary_channel_active(v34))) {
            result->phase3_seen = true;
            result->phase3_sample = offset;
        }
        if (!result->phase4_ready_seen && rx_event == 15) {
            result->phase4_ready_seen = true;
            result->phase4_ready_sample = offset;
        }
        if (!result->phase4_seen && (rx_stage >= 16 || tx_stage >= 41 || rx_event == 15)) {
            result->phase4_seen = true;
            result->phase4_sample = offset;
        }
        if (!result->training_failed && rx_event == 16) {
            result->training_failed = true;
            result->failure_sample = offset;
        }
    }

    silence_stderr_end(&stderr_guard);

    result->final_rx_stage = v34_get_rx_stage(v34);
    result->final_tx_stage = v34_get_tx_stage(v34);
    result->final_rx_event = v34_get_rx_event(v34);
    if (!result->phase4_ready_seen && result->final_rx_event == 15) {
        result->phase4_ready_seen = true;
        result->phase4_ready_sample = offset;
    }
    if (!result->phase4_seen
        && (result->final_rx_stage >= 16 || result->final_tx_stage >= 41 || result->final_rx_event == 15)) {
        result->phase4_seen = true;
        result->phase4_sample = offset;
    }
    if (!result->u_info)
        result->u_info = v34_get_v90_u_info(v34);

    v34_free(v34);
    return true;
}

static void print_v34_result(const decode_v34_result_t *result, bool calling_party)
{
    if (!result)
        return;

    printf("  Role:            %s\n", calling_party ? "caller" : "answerer");
    printf("  Final RX stage:  %s (%d)\n",
           v34_rx_stage_to_str_local(result->final_rx_stage),
           result->final_rx_stage);
    printf("  Final TX stage:  %s (%d)\n",
           v34_tx_stage_to_str_local(result->final_tx_stage),
           result->final_tx_stage);
    printf("  Final RX event:  %s (%d)\n",
           v34_event_to_str_local(result->final_rx_event),
           result->final_rx_event);
    if (result->info0_seen) {
        printf("  INFO0a:          decoded at %.1f ms\n", sample_to_ms(result->info0_sample, 8000));
        printf("                   3429=%s 1664pt=%s clock=%u ack_info0d=%s\n",
               result->info0a.support_3429 ? "yes" : "no",
               result->info0a.support_1664_point_constellation ? "yes" : "no",
               (unsigned) result->info0a.tx_clock_source,
               result->info0a.acknowledge_info0d ? "yes" : "no");
    } else {
        printf("  INFO0a:          not decoded\n");
    }
    if (result->info1_seen) {
        printf("  INFO1a:          decoded at %.1f ms\n", sample_to_ms(result->info1_sample, 8000));
        printf("                   md=%u u_info=%u up_rate_code=%u down_rate_code=%u freq_offset=%d\n",
               (unsigned) result->info1a.md,
               (unsigned) result->info1a.u_info,
               (unsigned) result->info1a.upstream_symbol_rate_code,
               (unsigned) result->info1a.downstream_rate_code,
               result->info1a.freq_offset);
    } else {
        printf("  INFO1a:          not decoded\n");
    }
    if (result->phase3_seen)
        printf("  Phase 3:         seen at %.1f ms\n", sample_to_ms(result->phase3_sample, 8000));
    if (result->phase4_ready_seen)
        printf("  Phase 4 ready:   seen at %.1f ms\n", sample_to_ms(result->phase4_ready_sample, 8000));
    if (result->phase4_seen)
        printf("  Phase 4 / MP:    seen at %.1f ms\n", sample_to_ms(result->phase4_sample, 8000));
    if (result->training_failed)
        printf("  Training fail:   observed at %.1f ms\n", sample_to_ms(result->failure_sample, 8000));
}

static void collect_v34_events(call_log_t *log,
                               const int16_t *samples,
                               int total_samples,
                               v91_law_t law)
{
    decode_v34_result_t answerer;
    decode_v34_result_t caller;
    char detail[256];

    if (!log || !samples || total_samples <= 0)
        return;

    if (decode_v34_pass(samples, total_samples, law, false, &answerer)) {
        if (answerer.info0_seen) {
            snprintf(detail, sizeof(detail),
                     "role=answerer 3429=%s 1664pt=%s clock=%u ack_info0d=%s",
                     answerer.info0a.support_3429 ? "yes" : "no",
                     answerer.info0a.support_1664_point_constellation ? "yes" : "no",
                     (unsigned) answerer.info0a.tx_clock_source,
                     answerer.info0a.acknowledge_info0d ? "yes" : "no");
            call_log_append(log, answerer.info0_sample, 0, "V.34", "INFO0a decoded", detail);
        }
        if (answerer.info1_seen) {
            snprintf(detail, sizeof(detail),
                     "role=answerer md=%u u_info=%u up_rate_code=%u down_rate_code=%u freq_offset=%d",
                     (unsigned) answerer.info1a.md,
                     (unsigned) answerer.info1a.u_info,
                     (unsigned) answerer.info1a.upstream_symbol_rate_code,
                     (unsigned) answerer.info1a.downstream_rate_code,
                     answerer.info1a.freq_offset);
            call_log_append(log, answerer.info1_sample, 0, "V.34", "INFO1a decoded", detail);
        }
        if (answerer.phase3_seen) {
            snprintf(detail, sizeof(detail), "role=answerer u_info=%d", answerer.u_info);
            call_log_append(log, answerer.phase3_sample, 0, "V.34", "Phase 3 reached", detail);
        }
        if (answerer.phase4_ready_seen) {
            snprintf(detail, sizeof(detail), "role=answerer event=%s (%d)",
                     v34_event_to_str_local(answerer.final_rx_event),
                     answerer.final_rx_event);
            call_log_append(log, answerer.phase4_ready_sample, 0, "V.90/V.92", "Phase 4 training ready", detail);
        }
        if (answerer.phase4_seen) {
            snprintf(detail, sizeof(detail), "role=answerer rx=%s(%d) tx=%s(%d)",
                     v34_rx_stage_to_str_local(answerer.final_rx_stage),
                     answerer.final_rx_stage,
                     v34_tx_stage_to_str_local(answerer.final_tx_stage),
                     answerer.final_tx_stage);
            call_log_append(log, answerer.phase4_sample, 0, "V.90/V.92", "Phase 4 / MP reached", detail);
        }
    }

    if (decode_v34_pass(samples, total_samples, law, true, &caller)) {
        if (caller.info0_seen) {
            snprintf(detail, sizeof(detail),
                     "role=caller 3429=%s 1664pt=%s clock=%u ack_info0d=%s",
                     caller.info0a.support_3429 ? "yes" : "no",
                     caller.info0a.support_1664_point_constellation ? "yes" : "no",
                     (unsigned) caller.info0a.tx_clock_source,
                     caller.info0a.acknowledge_info0d ? "yes" : "no");
            call_log_append(log, caller.info0_sample, 0, "V.34", "INFO0a decoded", detail);
        }
        if (caller.info1_seen) {
            snprintf(detail, sizeof(detail),
                     "role=caller md=%u u_info=%u up_rate_code=%u down_rate_code=%u freq_offset=%d",
                     (unsigned) caller.info1a.md,
                     (unsigned) caller.info1a.u_info,
                     (unsigned) caller.info1a.upstream_symbol_rate_code,
                     (unsigned) caller.info1a.downstream_rate_code,
                     caller.info1a.freq_offset);
            call_log_append(log, caller.info1_sample, 0, "V.34", "INFO1a decoded", detail);
        }
        if (caller.phase3_seen) {
            snprintf(detail, sizeof(detail), "role=caller u_info=%d", caller.u_info);
            call_log_append(log, caller.phase3_sample, 0, "V.34", "Phase 3 reached", detail);
        }
        if (caller.phase4_ready_seen) {
            snprintf(detail, sizeof(detail), "role=caller event=%s (%d)",
                     v34_event_to_str_local(caller.final_rx_event),
                     caller.final_rx_event);
            call_log_append(log, caller.phase4_ready_sample, 0, "V.90/V.92", "Phase 4 training ready", detail);
        }
        if (caller.phase4_seen) {
            snprintf(detail, sizeof(detail), "role=caller rx=%s(%d) tx=%s(%d)",
                     v34_rx_stage_to_str_local(caller.final_rx_stage),
                     caller.final_rx_stage,
                     v34_tx_stage_to_str_local(caller.final_tx_stage),
                     caller.final_tx_stage);
            call_log_append(log, caller.phase4_sample, 0, "V.90/V.92", "Phase 4 / MP reached", detail);
        }
    }
}

/* ------------------------------------------------------------------ */
/* V.91 signal detection                                               */
/* ------------------------------------------------------------------ */

static void print_v91_info_frame(const v91_info_frame_t *info)
{
    printf("    request_default_dil:            %s\n", info->request_default_dil ? "yes" : "no");
    printf("    request_control_channel:        %s\n", info->request_control_channel ? "yes" : "no");
    printf("    acknowledge_info_frame:         %s\n", info->acknowledge_info_frame ? "yes" : "no");
    printf("    max_tx_power:                   %d\n", info->max_tx_power);
    printf("    power_after_digital_impairment: %s\n",
           info->power_measured_after_digital_impairments ? "yes" : "no");
    printf("    tx_uses_alaw:                   %s\n", info->tx_uses_alaw ? "yes" : "no");
    printf("    request_transparent_mode:       %s\n", info->request_transparent_mode ? "yes" : "no");
    printf("    cleardown_if_transparent_denied:%s\n",
           info->cleardown_if_transparent_denied ? "yes" : "no");
}

static void print_v91_info_diag(const v91_info_diag_t *diag)
{
    printf("    CRC field:     0x%04x\n", diag->crc_field);
    printf("    CRC remainder: 0x%04x\n", diag->crc_remainder);
    printf("    Fill OK:       %s\n", diag->fill_ok ? "yes" : "no");
    printf("    Sync OK:       %s\n", diag->sync_ok ? "yes" : "no");
    printf("    Valid:         %s\n", diag->valid ? "yes" : "no");
    if (diag->valid)
        print_v91_info_frame(&diag->frame);
}

static void print_cp_frame(const vpcm_cp_frame_t *cp)
{
    printf("    transparent_mode_granted: %s\n", cp->transparent_mode_granted ? "yes" : "no");
    printf("    v90_compatibility:        %s\n", cp->v90_compatibility ? "yes" : "no");
    printf("    DRN:                      %d (%.0f bps)\n", cp->drn, vpcm_cp_drn_to_bps(cp->drn));
    printf("    acknowledge:              %s\n", cp->acknowledge ? "yes" : "no");
    printf("    constellation_count:      %d\n", cp->constellation_count);
    for (int c = 0; c < cp->constellation_count && c < VPCM_CP_MAX_CONSTELLATIONS; c++) {
        int pop = vpcm_cp_mask_population(cp->masks[c]);
        printf("    mask[%d]: %d ucodes enabled\n", c, pop);
    }
    printf("    DFI: [%d, %d, %d, %d, %d, %d]\n",
           cp->dfi[0], cp->dfi[1], cp->dfi[2], cp->dfi[3], cp->dfi[4], cp->dfi[5]);
}

static void print_dil_analysis(const v91_dil_analysis_t *a)
{
    printf("    n=%d  lsp=%d  ltp=%d\n", a->n, a->lsp, a->ltp);
    printf("    unique_train_u:           %d\n", a->unique_train_u);
    printf("    repeated_uchords:         %d\n", a->repeated_uchords);
    printf("    non_default_refs:         %d\n", a->non_default_refs);
    printf("    non_default_h:            %d\n", a->non_default_h);
    printf("    impairment_score:         %d\n", a->impairment_score);
    printf("    default_like:             %s\n", a->default_like ? "yes" : "no");
    printf("    robbed_bit_limited:       %s\n", a->robbed_bit_limited ? "yes" : "no");
    printf("    echo_limited:             %s\n", a->echo_limited ? "yes" : "no");
    printf("    recommended_down_drn:     %d (%.0f bps)\n",
           a->recommended_downstream_drn,
           vpcm_cp_drn_to_bps(a->recommended_downstream_drn));
    printf("    recommended_up_drn:       %d (%.0f bps)\n",
           a->recommended_upstream_drn,
           vpcm_cp_drn_to_bps(a->recommended_upstream_drn));
}

/*
 * Sliding-window V.91 signal scanner.
 *
 * Scans through G.711 codewords looking for recognizable V.91 signal
 * sequences. Uses the v91 RX functions which expect exact-length segments.
 */
static void decode_v91_signals(const uint8_t *codewords, int total,
                               v91_law_t law)
{
    v91_state_t state;
    v91_init(&state, law, V91_MODE_TRANSPARENT);

    int offset = 0;

    printf("\n=== V.91 Signal Scan ===\n");

    /* Scan for silence (phase 1) — look for runs of idle codewords */
    {
        uint8_t idle = v91_idle_codeword(law);
        int silence_start = -1;
        int silence_len = 0;
        for (int i = 0; i < total; i++) {
            if (codewords[i] == idle) {
                if (silence_start < 0) silence_start = i;
                silence_len++;
            } else {
                if (silence_len >= 48) {
                    printf("  [%7.1f ms] Silence/idle: %d codewords (%.1f ms)\n",
                           sample_to_ms(silence_start, 8000),
                           silence_len, sample_to_ms(silence_len, 8000));
                }
                silence_start = -1;
                silence_len = 0;
            }
        }
        if (silence_len >= 48) {
            printf("  [%7.1f ms] Silence/idle: %d codewords (%.1f ms)\n",
                   sample_to_ms(silence_start, 8000),
                   silence_len, sample_to_ms(silence_len, 8000));
        }
    }

    /* Pre-generate reference patterns once */
    uint8_t ez_pos_cw = v91_ucode_to_codeword(law, 127, true);
    uint8_t ez_neg_cw = v91_ucode_to_codeword(law, 127, false);

    uint8_t eu_pattern[V91_EU_SYMBOLS];
    uint8_t em_pattern[V91_EM_SYMBOLS];
    {
        v91_state_t eu_state, em_state;
        v91_init(&eu_state, law, V91_MODE_TRANSPARENT);
        v91_init(&em_state, law, V91_MODE_TRANSPARENT);
        v91_tx_eu_codewords(&eu_state, eu_pattern, V91_EU_SYMBOLS);
        v91_tx_em_codewords(&em_state, em_pattern, V91_EM_SYMBOLS);
    }

    /* Pre-generate default DIL pattern once */
    uint8_t expected_dil[V91_DEFAULT_DIL_SYMBOLS];
    int dil_pattern_len;
    {
        v91_dil_desc_t desc;
        v91_default_dil_init(&desc);
        v91_state_t dil_state;
        v91_init(&dil_state, law, V91_MODE_TRANSPARENT);
        dil_pattern_len = v91_tx_default_dil_codewords(&dil_state, expected_dil,
                                                        V91_DEFAULT_DIL_SYMBOLS);
    }

    vpcm_cp_frame_t default_cp;
    vpcm_cp_init(&default_cp);
    default_cp.drn = 7;
    default_cp.constellation_count = 1;
    vpcm_cp_enable_all_ucodes(default_cp.masks[0]);

    static const int cp_try_lengths[] = { 60, 66, 72, 120, 180, 294, 300, 600, 972 };
    int n_cp_try = (int)(sizeof(cp_try_lengths) / sizeof(cp_try_lengths[0]));

    /*
     * Single-pass scan at 6-symbol (frame) boundaries.
     * At each position we check all signal types and skip past matches.
     */
    for (offset = 0; offset < total; offset += 6) {
        /* Ez (24 symbols) */
        if (offset + V91_EZ_SYMBOLS <= total) {
            bool is_ez = true;
            for (int i = 0; i < V91_EZ_SYMBOLS && is_ez; i++) {
                uint8_t expected = (i % 2 == 0) ? ez_pos_cw : ez_neg_cw;
                if (codewords[offset + i] != expected) is_ez = false;
            }
            if (is_ez) {
                printf("  [%7.1f ms] Ez signal detected (%d symbols)\n",
                       sample_to_ms(offset, 8000), V91_EZ_SYMBOLS);
                offset += V91_EZ_SYMBOLS - 6;
                continue;
            }
        }

        /* Eu (12 symbols) */
        if (offset + V91_EU_SYMBOLS <= total
            && memcmp(codewords + offset, eu_pattern, V91_EU_SYMBOLS) == 0) {
            printf("  [%7.1f ms] Eu alignment signal detected\n",
                   sample_to_ms(offset, 8000));
            offset += V91_EU_SYMBOLS - 6;
            continue;
        }

        /* Em (12 symbols) */
        if (offset + V91_EM_SYMBOLS <= total
            && memcmp(codewords + offset, em_pattern, V91_EM_SYMBOLS) == 0) {
            printf("  [%7.1f ms] Em alignment signal detected\n",
                   sample_to_ms(offset, 8000));
            offset += V91_EM_SYMBOLS - 6;
            continue;
        }

        /* INFO frame (62 symbols) */
        if (offset + V91_INFO_SYMBOLS <= total) {
            v91_state_t info_state;
            v91_init(&info_state, law, V91_MODE_TRANSPARENT);
            v91_info_diag_t diag;
            if (v91_info_decode_diag(&info_state, codewords + offset,
                                     V91_INFO_SYMBOLS, &diag) && diag.valid) {
                printf("  [%7.1f ms] V.91 INFO frame decoded:\n",
                       sample_to_ms(offset, 8000));
                print_v91_info_diag(&diag);
                offset += V91_INFO_SYMBOLS - 6;
                continue;
            }
        }

        /* CP frame (variable length, try common sizes) */
        if (offset + 60 <= total) {
            bool found_cp = false;
            for (int t = 0; t < n_cp_try; t++) {
                int cp_try_len = cp_try_lengths[t];
                if (offset + cp_try_len > total) continue;

                v91_state_t try_state;
                v91_init(&try_state, law, V91_MODE_TRANSPARENT);

                vpcm_cp_frame_t cp_out;
                if (v91_rx_cp_codewords(&try_state, codewords + offset,
                                        cp_try_len, &cp_out, false)) {
                    printf("  [%7.1f ms] V.91 CP frame decoded (%d codewords):\n",
                           sample_to_ms(offset, 8000), cp_try_len);
                    print_cp_frame(&cp_out);
                    if (cp_out.drn > 0) {
                        printf("    Data rate: %.0f bps (DRN=%d, K=%d)\n",
                               vpcm_cp_drn_to_bps(cp_out.drn),
                               cp_out.drn,
                               vpcm_cp_drn_to_k(cp_out.drn));
                    }
                    offset += cp_try_len - 6;
                    found_cp = true;
                    break;
                }
            }
            if (found_cp) continue;
        }

        /* Default DIL (1500 symbols) */
        if (dil_pattern_len > 0 && offset + dil_pattern_len <= total
            && memcmp(codewords + offset, expected_dil, (size_t) dil_pattern_len) == 0) {
            v91_dil_desc_t desc;
            v91_default_dil_init(&desc);
            printf("  [%7.1f ms] V.91 default DIL detected (%d symbols)\n",
                   sample_to_ms(offset, 8000), dil_pattern_len);
            v91_dil_analysis_t analysis;
            if (v91_analyse_dil_descriptor(&desc, &analysis)) {
                printf("  DIL analysis:\n");
                print_dil_analysis(&analysis);
            }
            offset += dil_pattern_len - 6;
            continue;
        }

        /* Es (12 symbols) */
        if (offset + V91_ES_SYMBOLS <= total) {
            v91_state_t es_state;
            v91_init(&es_state, law, V91_MODE_TRANSPARENT);
            if (v91_rx_es_codewords(&es_state, codewords + offset,
                                    V91_ES_SYMBOLS, false)) {
                printf("  [%7.1f ms] V.91 Es signal detected\n",
                       sample_to_ms(offset, 8000));
                offset += V91_ES_SYMBOLS - 6;
                continue;
            }
        }

        /* B1 (12 symbols) */
        if (offset + V91_B1_SYMBOLS <= total) {
            v91_state_t b1_state;
            v91_init(&b1_state, law, V91_MODE_TRANSPARENT);
            if (v91_rx_b1_codewords(&b1_state, codewords + offset,
                                    V91_B1_SYMBOLS, &default_cp)) {
                printf("  [%7.1f ms] V.91 B1 signal detected\n",
                       sample_to_ms(offset, 8000));
                offset += V91_B1_SYMBOLS - 6;
                continue;
            }
        }
    }
}

static void collect_v91_events(call_log_t *log, const uint8_t *codewords, int total, v91_law_t law)
{
    uint8_t idle;
    uint8_t ez_pos_cw;
    uint8_t ez_neg_cw;
    uint8_t eu_pattern[V91_EU_SYMBOLS];
    uint8_t em_pattern[V91_EM_SYMBOLS];
    uint8_t expected_dil[V91_DEFAULT_DIL_SYMBOLS];
    int dil_pattern_len;
    static const int cp_try_lengths[] = { 60, 66, 72, 120, 180, 294, 300, 600, 972 };
    int n_cp_try = (int)(sizeof(cp_try_lengths) / sizeof(cp_try_lengths[0]));

    if (!log || !codewords || total <= 0)
        return;

    idle = v91_idle_codeword(law);

    {
        int silence_start = -1;
        int silence_len = 0;

        for (int i = 0; i < total; i++) {
            if (codewords[i] == idle) {
                if (silence_start < 0)
                    silence_start = i;
                silence_len++;
            } else {
                if (silence_len >= 48) {
                    char detail[128];
                    snprintf(detail, sizeof(detail), "%d idle codewords", silence_len);
                    call_log_append(log, silence_start, silence_len, "V.91", "Silence/idle", detail);
                }
                silence_start = -1;
                silence_len = 0;
            }
        }
        if (silence_len >= 48) {
            char detail[128];
            snprintf(detail, sizeof(detail), "%d idle codewords", silence_len);
            call_log_append(log, silence_start, silence_len, "V.91", "Silence/idle", detail);
        }
    }

    ez_pos_cw = v91_ucode_to_codeword(law, 127, true);
    ez_neg_cw = v91_ucode_to_codeword(law, 127, false);

    {
        v91_state_t eu_state;
        v91_state_t em_state;
        v91_dil_desc_t desc;
        v91_state_t dil_state;

        v91_init(&eu_state, law, V91_MODE_TRANSPARENT);
        v91_init(&em_state, law, V91_MODE_TRANSPARENT);
        v91_tx_eu_codewords(&eu_state, eu_pattern, V91_EU_SYMBOLS);
        v91_tx_em_codewords(&em_state, em_pattern, V91_EM_SYMBOLS);

        v91_default_dil_init(&desc);
        v91_init(&dil_state, law, V91_MODE_TRANSPARENT);
        dil_pattern_len = v91_tx_default_dil_codewords(&dil_state, expected_dil,
                                                       V91_DEFAULT_DIL_SYMBOLS);
    }

    for (int offset = 0; offset < total; offset += 6) {
        if (offset + V91_EZ_SYMBOLS <= total) {
            bool is_ez = true;

            for (int i = 0; i < V91_EZ_SYMBOLS && is_ez; i++) {
                uint8_t expected = (i % 2 == 0) ? ez_pos_cw : ez_neg_cw;
                if (codewords[offset + i] != expected)
                    is_ez = false;
            }
            if (is_ez) {
                call_log_append(log, offset, V91_EZ_SYMBOLS, "V.91", "Ez alignment signal", "");
                offset += V91_EZ_SYMBOLS - 6;
                continue;
            }
        }

        if (offset + V91_EU_SYMBOLS <= total
            && memcmp(codewords + offset, eu_pattern, V91_EU_SYMBOLS) == 0) {
            call_log_append(log, offset, V91_EU_SYMBOLS, "V.91", "Eu alignment signal", "");
            offset += V91_EU_SYMBOLS - 6;
            continue;
        }

        if (offset + V91_EM_SYMBOLS <= total
            && memcmp(codewords + offset, em_pattern, V91_EM_SYMBOLS) == 0) {
            call_log_append(log, offset, V91_EM_SYMBOLS, "V.91", "Em alignment signal", "");
            offset += V91_EM_SYMBOLS - 6;
            continue;
        }

        if (offset + V91_INFO_SYMBOLS <= total) {
            v91_state_t info_state;
            v91_info_diag_t diag;

            v91_init(&info_state, law, V91_MODE_TRANSPARENT);
            if (v91_info_decode_diag(&info_state, codewords + offset,
                                     V91_INFO_SYMBOLS, &diag) && diag.valid) {
                char detail[192];
                format_v91_info_summary(detail, sizeof(detail), &diag);
                call_log_append(log, offset, V91_INFO_SYMBOLS, "V.91", "INFO frame", detail);
                offset += V91_INFO_SYMBOLS - 6;
                continue;
            }
        }

        if (offset + 60 <= total) {
            bool found_cp = false;

            for (int t = 0; t < n_cp_try; t++) {
                int cp_try_len = cp_try_lengths[t];
                v91_state_t try_state;
                vpcm_cp_frame_t cp_out;
                char detail[192];

                if (offset + cp_try_len > total)
                    continue;

                v91_init(&try_state, law, V91_MODE_TRANSPARENT);
                if (!v91_rx_cp_codewords(&try_state, codewords + offset,
                                         cp_try_len, &cp_out, false)) {
                    continue;
                }

                format_cp_summary(detail, sizeof(detail), &cp_out);
                call_log_append(log, offset, cp_try_len, "V.91", "CP frame", detail);
                offset += cp_try_len - 6;
                found_cp = true;
                break;
            }
            if (found_cp)
                continue;
        }

        if (dil_pattern_len > 0 && offset + dil_pattern_len <= total
            && memcmp(codewords + offset, expected_dil, (size_t) dil_pattern_len) == 0) {
            v91_dil_desc_t desc;
            v91_dil_analysis_t analysis;
            char detail[192];

            v91_default_dil_init(&desc);
            if (v91_analyse_dil_descriptor(&desc, &analysis)) {
                format_v91_dil_summary(detail, sizeof(detail), &analysis);
            } else {
                snprintf(detail, sizeof(detail), "default DIL sequence");
            }
            call_log_append(log, offset, dil_pattern_len, "V.91", "Default DIL", detail);
            offset += dil_pattern_len - 6;
            continue;
        }

    }
}

/* ------------------------------------------------------------------ */
/* V.90 signal detection on G.711 codewords                            */
/* ------------------------------------------------------------------ */

/*
 * Convert a G.711 codeword to its Ucode magnitude (0..127).
 * Returns -1 if the codeword cannot be mapped (A-law only).
 */
static int codeword_to_ucode(v91_law_t law, uint8_t codeword)
{
    uint8_t positive = codeword | 0x80; /* strip sign, force positive */
    if (law == V91_LAW_ULAW)
        return 0xFF - positive;
    /* A-law: reverse lookup */
    for (int u = 0; u < 128; u++) {
        uint8_t expected = v91_ucode_to_codeword(law, u, true);
        if (expected == positive)
            return u;
    }
    return -1;
}

static void decode_v90_signals(const uint8_t *codewords, int total,
                               v91_law_t law)
{
    bool found_sequence = false;

    printf("\n=== V.90 Signal Scan ===\n");

    uint8_t idle = v91_idle_codeword(law);

    /*
     * Phase 3/4 signals all use PCM codewords at a specific Ucode magnitude.
     * Sd pattern: {+W, +0, +W, -W, -0, -W} where W = Ucode(16 + U_INFO),
     *             0 = Ucode 0 (which is the idle codeword with sign).
     *
     * Strategy: scan for the Sd 6-symbol pattern at every 6-sample boundary,
     * trying all plausible W_UCODE values (26..143, i.e. U_INFO 10..127).
     * Once Sd is found, follow the stream sequentially for S̄d, TRN1d, etc.
     */

    /* Pre-compute positive-zero and negative-zero codewords */
    uint8_t pos_zero = v91_ucode_to_codeword(law, 0, true);
    uint8_t neg_zero = v91_ucode_to_codeword(law, 0, false);

    for (int u_info = 10; u_info <= 127; u_info++) {
        int w_ucode = 16 + u_info;
        if (w_ucode > 127) w_ucode = 127;

        uint8_t pos_w = v91_ucode_to_codeword(law, w_ucode, true);
        uint8_t neg_w = v91_ucode_to_codeword(law, w_ucode, false);

        /* Sd pattern: +W, +0, +W, -W, -0, -W */
        uint8_t sd_pat[6] = { pos_w, pos_zero, pos_w, neg_w, neg_zero, neg_w };

        for (int offset = 0; offset + 6 * 4 <= total; offset += 6) {
            /* Quick check: first codeword must match */
            if (codewords[offset] != sd_pat[0])
                continue;

            /* Check at least 4 consecutive Sd repetitions */
            bool match = true;
            for (int rep = 0; rep < 4 && match; rep++) {
                for (int j = 0; j < 6 && match; j++) {
                    if (codewords[offset + rep * 6 + j] != sd_pat[j])
                        match = false;
                }
            }
            if (!match)
                continue;

            /* Count total Sd repetitions */
            int sd_reps = 0;
            while (offset + (sd_reps + 1) * 6 <= total) {
                bool ok = true;
                for (int j = 0; j < 6 && ok; j++) {
                    if (codewords[offset + sd_reps * 6 + j] != sd_pat[j])
                        ok = false;
                }
                if (!ok) break;
                sd_reps++;
            }
            printf("  [%7.1f ms] V.90 Sd: W_UCODE=%d (U_INFO=%d), %d reps (%d symbols, %.1f ms)\n",
                   sample_to_ms(offset, 8000), w_ucode, u_info,
                   sd_reps, sd_reps * 6, sample_to_ms(sd_reps * 6, 8000));

            int pos = offset + sd_reps * 6;

            /* Check for S̄d: {-W, -0, -W, +W, +0, +W} */
            uint8_t sbar_pat[6] = { neg_w, neg_zero, neg_w, pos_w, pos_zero, pos_w };
            int sbar_reps = 0;
            while (pos + (sbar_reps + 1) * 6 <= total) {
                bool ok = true;
                for (int j = 0; j < 6 && ok; j++) {
                    if (codewords[pos + sbar_reps * 6 + j] != sbar_pat[j])
                        ok = false;
                }
                if (!ok) break;
                sbar_reps++;
            }
            if (sbar_reps > 0) {
                printf("  [%7.1f ms] V.90 S̄d: %d reps (%d symbols, %.1f ms)\n",
                       sample_to_ms(pos, 8000), sbar_reps, sbar_reps * 6,
                       sample_to_ms(sbar_reps * 6, 8000));
                pos += sbar_reps * 6;
            }

            /* TRN1d: scrambled ones at U_INFO magnitude.
             * All codewords should have Ucode == u_info. Count run length. */
            int trn1d_start = pos;
            int trn1d_len = 0;
            while (pos < total) {
                int u = codeword_to_ucode(law, codewords[pos]);
                if (u != u_info) break;
                trn1d_len++;
                pos++;
            }
            if (trn1d_len > 0) {
                printf("  [%7.1f ms] V.90 TRN1d: %d symbols (%.1f ms) at U_INFO=%d\n",
                       sample_to_ms(trn1d_start, 8000), trn1d_len,
                       sample_to_ms(trn1d_len, 8000), u_info);
            }

            /* Jd/J'd: also at U_INFO magnitude with varying signs.
             * Count remaining U_INFO-magnitude codewords. */
            int jd_start = pos;
            int jd_len = 0;
            while (pos < total) {
                int u = codeword_to_ucode(law, codewords[pos]);
                if (u != u_info) break;
                jd_len++;
                pos++;
            }
            if (jd_len > 0) {
                int jd_frames = jd_len / 72;
                int jd_remainder = jd_len % 72;
                printf("  [%7.1f ms] V.90 Jd+J'd: %d symbols (%.1f ms)",
                       sample_to_ms(jd_start, 8000), jd_len,
                       sample_to_ms(jd_len, 8000));
                if (jd_frames > 0)
                    printf(", ~%d Jd frame repetitions", jd_frames);
                if (jd_remainder == 12)
                    printf(", 12-symbol J'd tail");
                printf("\n");
            }

            /* DIL: segments at various Ucode magnitudes with idle gaps.
             * Look for alternating non-idle / idle pattern at 12+12 boundaries. */
            {
                int dil_start = pos;
                int dil_segments = 0;
                while (pos + 24 <= total) {
                    /* A DIL segment is typically LSP (12) non-idle codewords
                     * followed by LTP (12) idle codewords */
                    bool has_active = false;
                    for (int i = 0; i < 12 && !has_active; i++) {
                        if (codewords[pos + i] != idle)
                            has_active = true;
                    }
                    bool has_idle_tail = true;
                    for (int i = 12; i < 24 && has_idle_tail; i++) {
                        if (codewords[pos + i] != idle)
                            has_idle_tail = false;
                    }
                    if (has_active && has_idle_tail) {
                        dil_segments++;
                        pos += 24;
                    } else {
                        break;
                    }
                }
                if (dil_segments >= 3) {
                    printf("  [%7.1f ms] V.90 DIL: %d segments (%d symbols, %.1f ms)\n",
                           sample_to_ms(dil_start, 8000), dil_segments,
                           dil_segments * 24, sample_to_ms(dil_segments * 24, 8000));
                }
            }

            /* Ri: idle codewords (Phase 4 start) */
            {
                int ri_start = pos;
                int ri_len = 0;
                while (pos < total && codewords[pos] == idle) {
                    ri_len++;
                    pos++;
                }
                if (ri_len >= 8) {
                    printf("  [%7.1f ms] V.90 Ri: %d idle symbols (%.1f ms)\n",
                           sample_to_ms(ri_start, 8000), ri_len,
                           sample_to_ms(ri_len, 8000));
                }
            }

            /* TRN2d: scrambled ones at U_INFO magnitude again */
            {
                int trn2d_start = pos;
                int trn2d_len = 0;
                while (pos < total) {
                    int u = codeword_to_ucode(law, codewords[pos]);
                    if (u != u_info) break;
                    trn2d_len++;
                    pos++;
                }
                if (trn2d_len > 0) {
                    printf("  [%7.1f ms] V.90 TRN2d: %d symbols (%.1f ms) at U_INFO=%d\n",
                           sample_to_ms(trn2d_start, 8000), trn2d_len,
                           sample_to_ms(trn2d_len, 8000), u_info);
                }
            }

            /* CP: also at U_INFO magnitude with sign-encoded bits */
            {
                int cp_start = pos;
                int cp_len = 0;
                while (pos < total) {
                    int u = codeword_to_ucode(law, codewords[pos]);
                    if (u != u_info) break;
                    cp_len++;
                    pos++;
                }
                if (cp_len > 0) {
                    printf("  [%7.1f ms] V.90 CP: %d symbols (%.1f ms), %d bits\n",
                           sample_to_ms(cp_start, 8000), cp_len,
                           sample_to_ms(cp_len, 8000), cp_len);
                }
            }

            /* B1d: idle codewords marking data mode entry */
            {
                int b1d_start = pos;
                int b1d_len = 0;
                while (pos < total && codewords[pos] == idle) {
                    b1d_len++;
                    pos++;
                }
                if (b1d_len >= 6) {
                    printf("  [%7.1f ms] V.90 B1d: %d idle symbols (%.1f ms), entering data mode\n",
                           sample_to_ms(b1d_start, 8000), b1d_len,
                           sample_to_ms(b1d_len, 8000));
                }
            }

            /* Data mode: remaining codewords are scrambled payload */
            if (pos < total) {
                int data_len = total - pos;
                double data_ms = sample_to_ms(data_len, 8000);
                /* Estimate throughput: 1 byte per codeword in simplified mode */
                printf("  [%7.1f ms] V.90 data mode: %d codewords (%.1f ms, ~%.0f bps)\n",
                       sample_to_ms(pos, 8000), data_len, data_ms,
                       data_ms > 0 ? (double) data_len * 8.0 * 1000.0 / data_ms : 0.0);
            }

            found_sequence = true;
            /* Only report first Sd match per U_INFO value */
            goto next_u_info;
        }
        next_u_info:;
    }

    if (!found_sequence) {
        printf("  No raw PCM-side V.90 sequence matched.\n");
        printf("  This scanner expects exact downstream G.711/PCM codewords after INFO1a.\n");
        printf("  These successful-call WAVs appear to be analog line audio, so later V.90/V.92\n");
        printf("  startup is better inferred from the V.34 state machine than from raw codeword matching.\n");
    }

    printf("  Note: V.90 INFO0a/INFO1a are V.34 Phase 2 DPSK modulated signals.\n"
           "        They require V.34 demodulation and are not visible as raw G.711 codewords.\n");
}

static void collect_v90_events(call_log_t *log, const uint8_t *codewords, int total, v91_law_t law)
{
    uint8_t idle;
    uint8_t pos_zero;
    uint8_t neg_zero;
    bool found_sequence = false;

    if (!log || !codewords || total <= 0)
        return;

    idle = v91_idle_codeword(law);
    pos_zero = v91_ucode_to_codeword(law, 0, true);
    neg_zero = v91_ucode_to_codeword(law, 0, false);

    for (int u_info = 10; u_info <= 127 && !found_sequence; u_info++) {
        int w_ucode = 16 + u_info;
        uint8_t pos_w;
        uint8_t neg_w;
        uint8_t sd_pat[6];

        if (w_ucode > 127)
            w_ucode = 127;

        pos_w = v91_ucode_to_codeword(law, w_ucode, true);
        neg_w = v91_ucode_to_codeword(law, w_ucode, false);
        sd_pat[0] = pos_w;
        sd_pat[1] = pos_zero;
        sd_pat[2] = pos_w;
        sd_pat[3] = neg_w;
        sd_pat[4] = neg_zero;
        sd_pat[5] = neg_w;

        for (int offset = 0; offset + 24 <= total && !found_sequence; offset += 6) {
            int sd_reps = 0;
            int pos;
            char detail[192];

            if (codewords[offset] != sd_pat[0])
                continue;

            for (int rep = 0; rep < 4; rep++) {
                bool ok = true;
                for (int j = 0; j < 6; j++) {
                    if (codewords[offset + rep * 6 + j] != sd_pat[j]) {
                        ok = false;
                        break;
                    }
                }
                if (!ok) {
                    sd_reps = 0;
                    break;
                }
                sd_reps++;
            }
            if (sd_reps < 4)
                continue;

            while (offset + (sd_reps + 1) * 6 <= total) {
                bool ok = true;
                for (int j = 0; j < 6; j++) {
                    if (codewords[offset + sd_reps * 6 + j] != sd_pat[j]) {
                        ok = false;
                        break;
                    }
                }
                if (!ok)
                    break;
                sd_reps++;
            }

            snprintf(detail, sizeof(detail), "w_ucode=%d u_info=%d reps=%d", w_ucode, u_info, sd_reps);
            call_log_append(log, offset, sd_reps * 6, "V.90", "Sd sequence", detail);

            pos = offset + sd_reps * 6;

            {
                uint8_t sbar_pat[6] = { neg_w, neg_zero, neg_w, pos_w, pos_zero, pos_w };
                int sbar_reps = 0;

                while (pos + (sbar_reps + 1) * 6 <= total) {
                    bool ok = true;
                    for (int j = 0; j < 6; j++) {
                        if (codewords[pos + sbar_reps * 6 + j] != sbar_pat[j]) {
                            ok = false;
                            break;
                        }
                    }
                    if (!ok)
                        break;
                    sbar_reps++;
                }
                if (sbar_reps > 0) {
                    snprintf(detail, sizeof(detail), "reps=%d", sbar_reps);
                    call_log_append(log, pos, sbar_reps * 6, "V.90", "S-bar sequence", detail);
                    pos += sbar_reps * 6;
                }
            }

            {
                int start = pos;
                int len = 0;

                while (pos < total) {
                    int u = codeword_to_ucode(law, codewords[pos]);
                    if (u != u_info)
                        break;
                    len++;
                    pos++;
                }
                if (len > 0) {
                    snprintf(detail, sizeof(detail), "u_info=%d", u_info);
                    call_log_append(log, start, len, "V.90", "TRN1d or Jd run", detail);
                }
            }

            {
                int dil_start = pos;
                int dil_segments = 0;
                v90_dil_analysis_t analysis;

                while (pos + 24 <= total) {
                    bool has_active = false;
                    bool has_idle_tail = true;

                    for (int i = 0; i < 12; i++) {
                        if (codewords[pos + i] != idle) {
                            has_active = true;
                            break;
                        }
                    }
                    for (int i = 12; i < 24; i++) {
                        if (codewords[pos + i] != idle) {
                            has_idle_tail = false;
                            break;
                        }
                    }
                    if (!has_active || !has_idle_tail)
                        break;
                    dil_segments++;
                    pos += 24;
                }
                if (dil_segments >= 3) {
                    v90_dil_desc_t default_like;
                    memset(&default_like, 0, sizeof(default_like));
                    default_like.n = V91_DEFAULT_DIL_SEGMENTS;
                    default_like.lsp = 12;
                    default_like.ltp = 12;
                    for (int i = 0; i < 8; i++) {
                        default_like.h[i] = (uint8_t) i;
                        default_like.ref[i] = (uint8_t) i;
                    }
                    for (int i = 0; i < 255; i++)
                        default_like.train_u[i] = 0x7F;
                    if (v90_analyse_dil_descriptor(&default_like, &analysis)) {
                        format_v90_dil_summary(detail, sizeof(detail), &analysis);
                    } else {
                        snprintf(detail, sizeof(detail), "%d apparent DIL segments", dil_segments);
                    }
                    call_log_append(log, dil_start, dil_segments * 24, "V.90", "Apparent DIL region", detail);
                }
            }

            {
                int start = pos;
                int len = 0;
                while (pos < total && codewords[pos] == idle) {
                    len++;
                    pos++;
                }
                if (len >= 8)
                    call_log_append(log, start, len, "V.90", "Ri/B1 idle run", "");
            }

            {
                int start = pos;
                int len = 0;

                while (pos < total) {
                    int u = codeword_to_ucode(law, codewords[pos]);
                    if (u != u_info)
                        break;
                    len++;
                    pos++;
                }
                if (len > 0) {
                    snprintf(detail, sizeof(detail), "u_info=%d", u_info);
                    call_log_append(log, start, len, "V.90", "TRN2d/CP/data-magnitude run", detail);
                }
            }

            if (pos < total) {
                int remaining = total - pos;
                snprintf(detail, sizeof(detail), "remaining carrier/data codewords=%d", remaining);
                call_log_append(log, pos, remaining, "V.90", "Remaining data-bearing region", detail);
            }

            found_sequence = true;
        }
    }

}

static void collect_post_phase3_stage_events(call_log_t *log,
                                             const uint8_t *codewords,
                                             int total_codewords,
                                             const decode_v34_result_t *answerer,
                                             const decode_v34_result_t *caller)
{
    jd_stage_decode_t jd_stage;
    char detail[192];

    if (!log || !codewords || total_codewords <= 0)
        return;

    if (!decode_jd_stage(codewords, total_codewords, answerer, caller, &jd_stage))
        return;

    snprintf(detail, sizeof(detail),
             "role=%s u_info=%d reps=%d frame_errors=%d",
             jd_stage.calling_party ? "caller" : "answerer",
             jd_stage.u_info,
             jd_stage.jd_repetitions,
             jd_stage.jd_frame_errors);
    call_log_append(log,
                    jd_stage.jd_start_sample,
                    jd_stage.jd_repetitions * OFFLINE_V90_JD_BITS,
                    "V.90",
                    "Jd capability frames",
                    detail);

    if (jd_stage.jd_prime_seen) {
        snprintf(detail, sizeof(detail),
                 "role=%s zero_bits=%d/12",
                 jd_stage.calling_party ? "caller" : "answerer",
                 jd_stage.jd_prime_zero_count);
        call_log_append(log,
                        jd_stage.jd_prime_sample,
                        OFFLINE_V90_JD_PRIME_BITS,
                        "V.90",
                        "J'd termination",
                        detail);
    }
}

static int v90_sequence_score(const uint8_t *codewords, int total, v91_law_t law)
{
    uint8_t pos_zero;
    uint8_t neg_zero;
    int best_score = 0;

    if (!codewords || total <= 0)
        return 0;

    pos_zero = v91_ucode_to_codeword(law, 0, true);
    neg_zero = v91_ucode_to_codeword(law, 0, false);

    for (int u_info = 10; u_info <= 127; u_info++) {
        int w_ucode = 16 + u_info;
        uint8_t pos_w;
        uint8_t neg_w;
        uint8_t sd_pat[6];

        if (w_ucode > 127)
            w_ucode = 127;

        pos_w = v91_ucode_to_codeword(law, w_ucode, true);
        neg_w = v91_ucode_to_codeword(law, w_ucode, false);
        sd_pat[0] = pos_w;
        sd_pat[1] = pos_zero;
        sd_pat[2] = pos_w;
        sd_pat[3] = neg_w;
        sd_pat[4] = neg_zero;
        sd_pat[5] = neg_w;

        for (int offset = 0; offset + 24 <= total; offset += 6) {
            int sd_reps = 0;
            int pos;
            int sbar_reps = 0;
            int trn1d_len = 0;
            int score;

            if (codewords[offset] != sd_pat[0])
                continue;

            for (int rep = 0; rep < 4; rep++) {
                bool ok = true;
                for (int j = 0; j < 6; j++) {
                    if (codewords[offset + rep * 6 + j] != sd_pat[j]) {
                        ok = false;
                        break;
                    }
                }
                if (!ok) {
                    sd_reps = 0;
                    break;
                }
                sd_reps++;
            }
            if (sd_reps < 4)
                continue;

            while (offset + (sd_reps + 1) * 6 <= total) {
                bool ok = true;
                for (int j = 0; j < 6; j++) {
                    if (codewords[offset + sd_reps * 6 + j] != sd_pat[j]) {
                        ok = false;
                        break;
                    }
                }
                if (!ok)
                    break;
                sd_reps++;
            }

            pos = offset + sd_reps * 6;
            {
                uint8_t sbar_pat[6] = { neg_w, neg_zero, neg_w, pos_w, pos_zero, pos_w };
                while (pos + (sbar_reps + 1) * 6 <= total) {
                    bool ok = true;
                    for (int j = 0; j < 6; j++) {
                        if (codewords[pos + sbar_reps * 6 + j] != sbar_pat[j]) {
                            ok = false;
                            break;
                        }
                    }
                    if (!ok)
                        break;
                    sbar_reps++;
                }
                pos += sbar_reps * 6;
            }

            while (pos < total) {
                int u = codeword_to_ucode(law, codewords[pos]);
                if (u != u_info)
                    break;
                trn1d_len++;
                pos++;
            }

            score = sd_reps * 1000 + sbar_reps * 100 + (trn1d_len > 999 ? 999 : trn1d_len);
            if (score > best_score)
                best_score = score;
        }
    }

    return best_score;
}

static void build_g711_codewords_from_linear(const int16_t *samples,
                                             int total_samples,
                                             v91_law_t law,
                                             double gain,
                                             int bias,
                                             uint8_t *out)
{
    if (!samples || !out || total_samples <= 0)
        return;

    for (int i = 0; i < total_samples; i++) {
        long scaled = lrint((double) samples[i] * gain) + bias;
        if (scaled < -32768)
            scaled = -32768;
        else if (scaled > 32767)
            scaled = 32767;
        out[i] = (law == V91_LAW_ULAW)
            ? linear_to_ulaw((int16_t) scaled)
            : linear_to_alaw((int16_t) scaled);
    }
}

static void adaptive_reslice_codewords(const int16_t *samples,
                                       int total_samples,
                                       v91_law_t law,
                                       uint8_t *direct_codewords,
                                       uint8_t *work_codewords,
                                       codeword_stream_info_t *info)
{
    static const double gain_candidates[] = { 0.50, 0.67, 0.80, 1.00, 1.25, 1.50, 2.00 };
    static const int bias_candidates[] = { -2048, -1024, -512, 0, 512, 1024, 2048 };
    int direct_score;
    int best_score;
    double best_gain;
    int best_bias;

    if (!samples || !direct_codewords || !work_codewords || !info || total_samples <= 0)
        return;

    memset(info, 0, sizeof(*info));
    direct_score = v90_sequence_score(direct_codewords, total_samples, law);
    best_score = direct_score;
    best_gain = 1.0;
    best_bias = 0;

    for (size_t gi = 0; gi < sizeof(gain_candidates) / sizeof(gain_candidates[0]); gi++) {
        for (size_t bi = 0; bi < sizeof(bias_candidates) / sizeof(bias_candidates[0]); bi++) {
            int score;

            build_g711_codewords_from_linear(samples, total_samples, law,
                                             gain_candidates[gi], bias_candidates[bi],
                                             work_codewords);
            score = v90_sequence_score(work_codewords, total_samples, law);
            if (score > best_score) {
                best_score = score;
                best_gain = gain_candidates[gi];
                best_bias = bias_candidates[bi];
            }
        }
    }

    info->direct_v90_score = direct_score;
    info->selected_v90_score = best_score;
    info->gain = best_gain;
    info->bias = best_bias;
    info->adaptive_used = (best_score > direct_score && (best_gain != 1.0 || best_bias != 0));

    if (info->adaptive_used) {
        build_g711_codewords_from_linear(samples, total_samples, law,
                                         best_gain, best_bias, direct_codewords);
    }
}

static int offline_v90_scramble_bit(uint32_t *sr, int in_bit)
{
    int fb = ((int)(*sr >> 22) ^ (int)(*sr >> 4)) & 1;
    int out_bit = in_bit ^ fb;
    *sr = ((*sr << 1) | (uint32_t) out_bit) & 0x7FFFFF;
    return out_bit;
}

static int offline_v90_descramble_reg_bit(uint32_t *reg, int in_bit)
{
    int out_bit = (in_bit ^ (int) (*reg >> 22) ^ (int) (*reg >> 4)) & 1;
    *reg = (*reg << 1) | (uint32_t) in_bit;
    return out_bit;
}

static void offline_v90_build_jd_bits(uint8_t bits[OFFLINE_V90_JD_BITS])
{
    uint8_t packed[(OFFLINE_V90_JD_BITS + 7) / 8];
    int pos = 0;
    uint16_t crc = 0xFFFF;

    memset(bits, 0, OFFLINE_V90_JD_BITS);
    memset(packed, 0, sizeof(packed));

    for (int i = 0; i < 17; i++)
        packed[pos / 8] |= (uint8_t) (1U << (pos % 8)), pos++;
    pos++;
    for (int i = 18; i <= 33; i++)
        packed[pos / 8] |= (uint8_t) (1U << (pos % 8)), pos++;
    pos++;
    for (int i = 35; i <= 40; i++)
        packed[pos / 8] |= (uint8_t) (1U << (pos % 8)), pos++;
    pos += 6;
    pos++;
    pos++;
    packed[pos / 8] |= (uint8_t) (1U << (pos % 8));
    pos++;
    pos++;
    pos++;

    for (int i = 0; i < 52; i++) {
        int bit = (packed[i / 8] >> (i % 8)) & 1;
        int fb = ((crc >> 15) ^ bit) & 1;
        crc <<= 1;
        if (fb)
            crc ^= 0x8005;
        crc &= 0xFFFF;
    }
    for (int i = 0; i < 16; i++) {
        if ((crc >> (15 - i)) & 1)
            packed[(52 + i) / 8] |= (uint8_t) (1U << ((52 + i) % 8));
    }

    for (int i = 0; i < OFFLINE_V90_JD_BITS; i++)
        bits[i] = (uint8_t) ((packed[i / 8] >> (i % 8)) & 1U);
}

static void offline_v90_seed_after_trn1d(uint32_t *descramble_reg_out, int *last_sign_out)
{
    uint32_t tx_sr = 0;
    uint32_t rx_reg = 0;
    int last_sign = 0;

    for (int i = 0; i < OFFLINE_V90_TRN1D_SYMBOLS; i++) {
        int scrambled = offline_v90_scramble_bit(&tx_sr, 1);
        last_sign = scrambled;
        (void) offline_v90_descramble_reg_bit(&rx_reg, scrambled);
    }

    if (descramble_reg_out)
        *descramble_reg_out = rx_reg;
    if (last_sign_out)
        *last_sign_out = last_sign;
}

static int offline_v90_decode_jd_bits(const uint8_t *codewords,
                                      int total_codewords,
                                      int start_sample,
                                      bool invert_sign,
                                      uint8_t out_bits[OFFLINE_V90_JD_BITS])
{
    uint32_t descramble_reg;
    int prev_sign;
    int errors = 0;
    uint8_t expected[OFFLINE_V90_JD_BITS];

    if (!codewords || !out_bits || start_sample < 0 || start_sample + OFFLINE_V90_JD_BITS > total_codewords)
        return OFFLINE_V90_JD_BITS + 1;

    offline_v90_build_jd_bits(expected);
    offline_v90_seed_after_trn1d(&descramble_reg, &prev_sign);

    for (int i = 0; i < OFFLINE_V90_JD_BITS; i++) {
        int sign = (codewords[start_sample + i] & 0x80) ? 1 : 0;
        int scrambled;
        int plain;

        if (invert_sign)
            sign ^= 1;
        scrambled = sign ^ prev_sign;
        prev_sign = sign;
        plain = offline_v90_descramble_reg_bit(&descramble_reg, scrambled);
        out_bits[i] = (uint8_t) plain;
        if (plain != expected[i])
            errors++;
    }

    return errors;
}

static bool decode_jd_stage(const uint8_t *codewords,
                            int total_codewords,
                            const decode_v34_result_t *answerer,
                            const decode_v34_result_t *caller,
                            jd_stage_decode_t *out)
{
    const decode_v34_result_t *src;
    bool calling_party = false;
    int search_start;
    int search_end;
    int best_errors = OFFLINE_V90_JD_BITS + 1;
    int best_start = -1;
    bool best_invert = false;

    if (!codewords || total_codewords <= 0 || !out)
        return false;

    memset(out, 0, sizeof(*out));
    src = pick_post_phase3_source(answerer, caller, &calling_party);
    if (!src)
        return false;

    out->u_info = src->u_info;
    out->calling_party = calling_party;
    out->phase3_start_sample = src->phase3_sample >= 0 ? src->phase3_sample : src->info1_sample;
    search_start = src->info1_sample >= 0 ? src->info1_sample : out->phase3_start_sample;
    if (search_start < 0)
        return false;

    if (src->phase4_seen && src->phase4_sample > search_start)
        search_end = src->phase4_sample;
    else if (src->phase4_ready_seen && src->phase4_ready_sample > search_start)
        search_end = src->phase4_ready_sample;
    else if (src->failure_sample > search_start)
        search_end = src->failure_sample;
    else
        search_end = search_start + 4096;
    if (search_end > total_codewords - OFFLINE_V90_JD_BITS)
        search_end = total_codewords - OFFLINE_V90_JD_BITS;
    if (search_end < search_start)
        return false;

    for (int candidate = search_start; candidate <= search_end; candidate++) {
        uint8_t decoded_bits[OFFLINE_V90_JD_BITS];

        for (int invert = 0; invert <= 1; invert++) {
            int errors = offline_v90_decode_jd_bits(codewords, total_codewords, candidate,
                                                    invert != 0, decoded_bits);
            if (errors < best_errors) {
                best_errors = errors;
                best_start = candidate;
                best_invert = (invert != 0);
            }
        }
    }

    if (best_start < 0 || best_errors > 20)
        return false;

    out->ok = true;
    out->jd_start_sample = best_start;
    out->jd_frame_errors = best_errors;
    out->jd_repetitions = 0;

    for (int rep = 0; ; rep++) {
        int rep_start = best_start + rep * OFFLINE_V90_JD_BITS;
        uint8_t decoded_bits[OFFLINE_V90_JD_BITS];
        int errors = offline_v90_decode_jd_bits(codewords, total_codewords, rep_start,
                                                best_invert, decoded_bits);
        if (errors > 16)
            break;
        out->jd_repetitions++;
    }

    {
        int jd_prime_start = best_start + out->jd_repetitions * OFFLINE_V90_JD_BITS;
        uint32_t descramble_reg;
        int prev_sign;

        offline_v90_seed_after_trn1d(&descramble_reg, &prev_sign);
        for (int i = 0; i < out->jd_repetitions * OFFLINE_V90_JD_BITS; i++) {
            int sign = (codewords[best_start + i] & 0x80) ? 1 : 0;
            int scrambled;
            if (best_invert)
                sign ^= 1;
            scrambled = sign ^ prev_sign;
            prev_sign = sign;
            (void) offline_v90_descramble_reg_bit(&descramble_reg, scrambled);
        }

        if (jd_prime_start + OFFLINE_V90_JD_PRIME_BITS <= total_codewords) {
            int zero_count = 0;
            for (int i = 0; i < OFFLINE_V90_JD_PRIME_BITS; i++) {
                int sign = (codewords[jd_prime_start + i] & 0x80) ? 1 : 0;
                int scrambled;
                int plain;
                if (best_invert)
                    sign ^= 1;
                scrambled = sign ^ prev_sign;
                prev_sign = sign;
                plain = offline_v90_descramble_reg_bit(&descramble_reg, scrambled);
                if (plain == 0)
                    zero_count++;
            }
            out->jd_prime_zero_count = zero_count;
            if (zero_count >= 10) {
                out->jd_prime_seen = true;
                out->jd_prime_sample = jd_prime_start;
            }
        }
    }

    return true;
}

static void print_jd_stage_decode(const jd_stage_decode_t *result)
{
    if (!result || !result->ok)
        return;

    printf("\n=== Phase 3 Stage Decode ===\n");
    printf("  Source role:      %s\n", result->calling_party ? "caller" : "answerer");
    printf("  Phase 3 start:    %.1f ms\n", sample_to_ms(result->phase3_start_sample, 8000));
    printf("  U_INFO:           %d\n", result->u_info);
    printf("  Jd start:         %.1f ms\n", sample_to_ms(result->jd_start_sample, 8000));
    printf("  Jd repetitions:   %d\n", result->jd_repetitions);
    printf("  Jd frame errors:  %d/72 on best alignment\n", result->jd_frame_errors);
    if (result->jd_prime_seen) {
        printf("  J'd:              seen at %.1f ms (%d/12 zero bits)\n",
               sample_to_ms(result->jd_prime_sample, 8000),
               result->jd_prime_zero_count);
    } else {
        printf("  J'd:              not confirmed (%d/12 zero bits)\n",
               result->jd_prime_zero_count);
    }
}

static const decode_v34_result_t *pick_post_phase3_source(const decode_v34_result_t *answerer,
                                                          const decode_v34_result_t *caller,
                                                          bool *calling_party_out)
{
    if (answerer && answerer->info1_seen && answerer->u_info > 0) {
        if (calling_party_out)
            *calling_party_out = false;
        return answerer;
    }
    if (caller && caller->info1_seen && caller->u_info > 0) {
        if (calling_party_out)
            *calling_party_out = true;
        return caller;
    }
    if (answerer && answerer->phase3_seen && answerer->u_info > 0) {
        if (calling_party_out)
            *calling_party_out = false;
        return answerer;
    }
    if (caller && caller->phase3_seen && caller->u_info > 0) {
        if (calling_party_out)
            *calling_party_out = true;
        return caller;
    }
    return NULL;
}

static bool decode_post_phase3_codewords(const uint8_t *codewords,
                                         int total_codewords,
                                         v91_law_t law,
                                         const decode_v34_result_t *answerer,
                                         const decode_v34_result_t *caller,
                                         post_phase3_decode_t *out)
{
    const decode_v34_result_t *src;
    v90_state_t *v90;
    uint8_t decoded[512];
    stderr_silence_guard_t stderr_guard;
    int start;
    int decode_len;
    bool calling_party = false;

    if (!codewords || total_codewords <= 0 || !out)
        return false;

    memset(out, 0, sizeof(*out));
    src = pick_post_phase3_source(answerer, caller, &calling_party);
    if (!src)
        return false;

    start = src->info1_seen ? src->info1_sample : src->phase3_sample;
    if (start < 0 || start >= total_codewords)
        return false;

    v90 = v90_init_data_pump(law == V91_LAW_ALAW ? V90_LAW_ALAW : V90_LAW_ULAW);
    if (!v90)
        return false;

    stderr_guard = silence_stderr_begin();
    v90_start_phase3(v90, src->u_info);
    decode_len = total_codewords - start;
    if (decode_len > (int) sizeof(decoded))
        decode_len = (int) sizeof(decoded);

    out->decoded_octets = v90_rx_codewords(v90, decoded, decode_len, codewords + start, decode_len);
    silence_stderr_end(&stderr_guard);
    out->ok = (out->decoded_octets > 0);
    out->calling_party = calling_party;
    out->start_sample = start;
    out->u_info = src->u_info;
    out->preview_len = out->decoded_octets;
    if (out->preview_len > (int) sizeof(out->preview))
        out->preview_len = (int) sizeof(out->preview);

    for (int i = 0; i < out->preview_len; i++) {
        out->preview[i] = decoded[i];
        if (decoded[i] >= 32 && decoded[i] <= 126)
            out->printable_octets++;
    }

    v90_free(v90);
    return out->ok;
}

static void print_post_phase3_decode(const post_phase3_decode_t *result)
{
    if (!result || !result->ok)
        return;

    printf("\n=== Post-Phase-3 Codeword Decode ===\n");
    printf("  Source role:      %s\n", result->calling_party ? "caller" : "answerer");
    printf("  Start time:       %.1f ms\n", sample_to_ms(result->start_sample, 8000));
    printf("  U_INFO:           %d\n", result->u_info);
    printf("  Octets decoded:   %d\n", result->decoded_octets);
    printf("  Printable ratio:  %.1f%% of first %d octets\n",
           result->preview_len > 0 ? (100.0 * (double) result->printable_octets / (double) result->preview_len) : 0.0,
           result->preview_len);
    printf("  Preview hex:      ");
    for (int i = 0; i < result->preview_len; i++) {
        if (i > 0)
            printf(" ");
        printf("%02X", result->preview[i]);
    }
    printf("\n");
    printf("  Preview ASCII:    ");
    for (int i = 0; i < result->preview_len; i++) {
        uint8_t c = result->preview[i];
        putchar((c >= 32 && c <= 126) ? (int) c : '.');
    }
    printf("\n");
}

/* ------------------------------------------------------------------ */
/* RMS energy profile                                                  */
/* ------------------------------------------------------------------ */

static void print_energy_profile(const int16_t *samples, int total, int rate)
{
    int window = rate / 10; /* 100 ms windows */
    printf("\n=== Energy Profile (100ms windows) ===\n");
    for (int offset = 0; offset + window <= total; offset += window) {
        double db = rms_energy_db(samples + offset, window);
        printf("  [%7.1f ms] %6.1f dBFS", sample_to_ms(offset, rate), db);

        /* Simple bar graph */
        int bar = (int)((db + 80.0) * 0.5);
        if (bar < 0) bar = 0;
        if (bar > 40) bar = 40;
        printf("  ");
        for (int i = 0; i < bar; i++) printf("#");
        printf("\n");
    }
}

/* ------------------------------------------------------------------ */
/* Codeword histogram                                                  */
/* ------------------------------------------------------------------ */

static void print_codeword_stats(const uint8_t *codewords, int total,
                                 v91_law_t law)
{
    int histogram[256];
    memset(histogram, 0, sizeof(histogram));
    for (int i = 0; i < total; i++)
        histogram[codewords[i]]++;

    printf("\n=== Codeword Statistics ===\n");
    printf("  Total codewords: %d (%.1f ms)\n", total, sample_to_ms(total, 8000));

    uint8_t idle = v91_idle_codeword(law);
    printf("  Idle codeword (0x%02X): %d (%.1f%%)\n",
           idle, histogram[idle],
           total > 0 ? 100.0 * (double) histogram[idle] / (double) total : 0.0);

    /* Count positive vs negative */
    int positive = 0, negative = 0;
    for (int i = 0; i < 256; i++) {
        if (histogram[i] == 0) continue;
        if (i & 0x80) positive += histogram[i];
        else negative += histogram[i];
    }
    printf("  Positive sign: %d (%.1f%%)  Negative sign: %d (%.1f%%)\n",
           positive, total > 0 ? 100.0 * (double) positive / (double) total : 0.0,
           negative, total > 0 ? 100.0 * (double) negative / (double) total : 0.0);

    /* Top 10 most frequent codewords */
    printf("  Top 10 codewords:\n");
    for (int rank = 0; rank < 10; rank++) {
        int best_cw = -1, best_count = 0;
        for (int i = 0; i < 256; i++) {
            if (histogram[i] > best_count) {
                best_count = histogram[i];
                best_cw = i;
            }
        }
        if (best_cw < 0 || best_count == 0) break;
        int16_t linear = (law == V91_LAW_ULAW)
            ? ulaw_to_linear((uint8_t) best_cw)
            : alaw_to_linear((uint8_t) best_cw);
        printf("    0x%02X: %6d (%5.1f%%)  linear=%6d  sign=%c\n",
               best_cw, best_count,
               100.0 * (double) best_count / (double) total,
               linear,
               (best_cw & 0x80) ? '+' : '-');
        histogram[best_cw] = 0;
    }
}

/* ------------------------------------------------------------------ */
/* Main                                                                */
/* ------------------------------------------------------------------ */

typedef enum {
    FMT_AUTO,
    FMT_WAV,
    FMT_G711
} input_format_t;

typedef enum {
    CH_LEFT = 0,
    CH_RIGHT = 1,
    CH_MONO = 2
} channel_select_t;

typedef struct {
    bool do_v34;
    bool do_v8;
    bool do_v91;
    bool do_v90;
    bool do_energy;
    bool do_stats;
    bool do_call_log;
    bool raw_output_enabled;
} decode_options_t;

static void run_decode_suite(const char *label,
                             const int16_t *linear_samples,
                             const uint8_t *g711_codewords,
                             int total_samples,
                             int total_codewords,
                             int sample_rate,
                             v91_law_t law,
                             const decode_options_t *opts,
                             const codeword_stream_info_t *codeword_info)
{
    decode_v34_result_t answerer;
    decode_v34_result_t caller;
    bool have_answerer = false;
    bool have_caller = false;

    if (!linear_samples || !g711_codewords || !opts)
        return;

    printf("\n--- Channel: %s ---\n", label);
    if (codeword_info && (opts->do_v90 || opts->do_v91 || opts->do_call_log)) {
        if (codeword_info->adaptive_used) {
            printf("Codewords: adaptive re-slice selected (gain=%.2f, bias=%d, V.90 score %d -> %d)\n",
                   codeword_info->gain,
                   codeword_info->bias,
                   codeword_info->direct_v90_score,
                   codeword_info->selected_v90_score);
        } else {
            printf("Codewords: direct companding retained (adaptive re-slice found no stronger V.90 candidate; score %d)\n",
                   codeword_info->selected_v90_score);
        }
    }

    if (opts->raw_output_enabled && opts->do_stats)
        print_codeword_stats(g711_codewords, total_codewords, law);

    if (opts->raw_output_enabled && opts->do_energy)
        print_energy_profile(linear_samples, total_samples, sample_rate);

    if ((opts->raw_output_enabled && (opts->do_v34 || opts->do_v90))) {
        have_answerer = decode_v34_pass(linear_samples, total_samples, law, false, &answerer);
        have_caller = decode_v34_pass(linear_samples, total_samples, law, true, &caller);
    }

    if (opts->raw_output_enabled && opts->do_v34) {
        printf("\n=== V.34/V.90 Phase 2 Decode (as answerer) ===\n");
        if (have_answerer)
            print_v34_result(&answerer, false);
        else
            printf("  V.34 probe init failed\n");

        printf("\n=== V.34/V.90 Phase 2 Decode (as caller) ===\n");
        if (have_caller)
            print_v34_result(&caller, true);
        else
            printf("  V.34 probe init failed\n");
    }

    if (opts->raw_output_enabled && opts->do_v8) {
        printf("\n=== V.8 Decode (as answerer) ===\n");
        decode_v8_pass(linear_samples, total_samples, false);
        printf("\n=== V.8 Decode (as caller) ===\n");
        decode_v8_pass(linear_samples, total_samples, true);
    }

    if (opts->raw_output_enabled && opts->do_v91)
        decode_v91_signals(g711_codewords, total_codewords, law);

    if (opts->raw_output_enabled && opts->do_v90) {
        jd_stage_decode_t jd_stage;
        post_phase3_decode_t post_phase3;

        decode_v90_signals(g711_codewords, total_codewords, law);
        if (decode_jd_stage(g711_codewords, total_codewords,
                            have_answerer ? &answerer : NULL,
                            have_caller ? &caller : NULL,
                            &jd_stage)) {
            print_jd_stage_decode(&jd_stage);
        }
        if (decode_post_phase3_codewords(g711_codewords, total_codewords, law,
                                         have_answerer ? &answerer : NULL,
                                         have_caller ? &caller : NULL,
                                         &post_phase3)) {
            print_post_phase3_decode(&post_phase3);
        }
    }

    if (opts->do_call_log) {
        call_log_t log;
        call_log_init(&log);

        if (opts->do_v34)
            collect_v34_events(&log, linear_samples, total_samples, law);
        if (opts->do_v8) {
            collect_v8_event(&log, linear_samples, total_samples, false);
            collect_v8_event(&log, linear_samples, total_samples, true);
        }
        if (opts->do_v91)
            collect_v91_events(&log, g711_codewords, total_codewords, law);
        if (opts->do_v90) {
            collect_v90_events(&log, g711_codewords, total_codewords, law);
            if (have_answerer || have_caller) {
                collect_post_phase3_stage_events(&log,
                                                 g711_codewords,
                                                 total_codewords,
                                                 have_answerer ? &answerer : NULL,
                                                 have_caller ? &caller : NULL);
            }
        }

        call_log_sort(&log);
        print_call_log(label, &log, total_samples, sample_rate);
        call_log_reset(&log);
    }
}

int main(int argc, char **argv)
{
    const char *input_path = NULL;
    v91_law_t law = V91_LAW_ULAW;
    input_format_t fmt = FMT_AUTO;
    channel_select_t channel = CH_MONO;
    bool channel_explicit = false;
    bool explicit_decode_output = false;
    bool do_v34 = false;
    bool do_v8 = false;
    bool do_v91 = false;
    bool do_v90 = false;
    bool do_energy = false;
    bool do_stats = false;
    bool do_call_log = false;
    bool do_all = false;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--law") == 0 && i + 1 < argc) {
            i++;
            if (strcasecmp(argv[i], "alaw") == 0 || strcasecmp(argv[i], "a-law") == 0)
                law = V91_LAW_ALAW;
            else
                law = V91_LAW_ULAW;
        } else if (strcmp(argv[i], "--wav") == 0) {
            fmt = FMT_WAV;
        } else if (strcmp(argv[i], "--g711") == 0) {
            fmt = FMT_G711;
        } else if (strcmp(argv[i], "--channel") == 0 && i + 1 < argc) {
            i++;
            channel_explicit = true;
            if (strcasecmp(argv[i], "L") == 0 || strcasecmp(argv[i], "left") == 0)
                channel = CH_LEFT;
            else if (strcasecmp(argv[i], "R") == 0 || strcasecmp(argv[i], "right") == 0)
                channel = CH_RIGHT;
            else
                channel = CH_MONO;
        } else if (strcmp(argv[i], "--v8") == 0) {
            do_v8 = true;
            explicit_decode_output = true;
        } else if (strcmp(argv[i], "--v34") == 0) {
            do_v34 = true;
            explicit_decode_output = true;
        } else if (strcmp(argv[i], "--v91") == 0) {
            do_v91 = true;
            explicit_decode_output = true;
        } else if (strcmp(argv[i], "--v90") == 0) {
            do_v90 = true;
            explicit_decode_output = true;
        } else if (strcmp(argv[i], "--energy") == 0) {
            do_energy = true;
            explicit_decode_output = true;
        } else if (strcmp(argv[i], "--stats") == 0) {
            do_stats = true;
            explicit_decode_output = true;
        } else if (strcmp(argv[i], "--call-log") == 0) {
            do_call_log = true;
        } else if (strcmp(argv[i], "--all") == 0) {
            do_all = true;
            explicit_decode_output = true;
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            printf("Usage: %s [options] <file.wav|file.g711>\n\n"
                   "Options:\n"
                   "  --law ulaw|alaw    G.711 law (default: ulaw)\n"
                   "  --wav              Force WAV input format\n"
                   "  --g711             Force raw G.711 input format\n"
                   "  --channel L|R|mono Channel to decode from stereo WAV\n"
                   "                     (L=TX, R=RX for tap files; default: mono)\n"
                   "  --v34              Decode V.34/V.90 Phase 2 symbols via SpanDSP\n"
                   "  --v8               Decode V.8 negotiation\n"
                   "  --v91              Decode V.91 signals (INFO, CP, DIL, Ez, etc.)\n"
                   "  --v90              Decode V.90 signals (Sd, CP, etc.)\n"
                   "  --energy           Print RMS energy profile\n"
                   "  --stats            Print codeword histogram/statistics\n"
                   "  --call-log         Print a best-effort chronological call log\n"
                   "  --all              Enable all decoders (default)\n"
                   "\nWAV files should be 8000 Hz, 16-bit (mono or stereo).\n"
                   "G.711 files are raw codeword bytes at 8000 Hz.\n",
                   argv[0]);
            return 0;
        } else if (argv[i][0] != '-') {
            input_path = argv[i];
        } else {
            fprintf(stderr, "Unknown option: %s\n", argv[i]);
            return 1;
        }
    }

    if (!input_path) {
        fprintf(stderr, "No input file specified. Use --help for usage.\n");
        return 1;
    }

    if (!do_v34 && !do_v8 && !do_v91 && !do_v90 && !do_energy && !do_stats && !do_call_log)
        do_all = true;

    if (do_all) {
        do_v34 = do_v8 = do_v91 = do_v90 = do_energy = do_stats = do_call_log = true;
    }

    if (do_call_log && !do_v34 && !do_v8 && !do_v91 && !do_v90) {
        do_v34 = true;
        do_v8 = true;
        do_v91 = true;
        do_v90 = true;
    }

    /* Auto-detect format from extension */
    if (fmt == FMT_AUTO) {
        const char *ext = strrchr(input_path, '.');
        if (ext && strcasecmp(ext, ".wav") == 0)
            fmt = FMT_WAV;
        else if (ext && strcasecmp(ext, ".g711") == 0)
            fmt = FMT_G711;
        else if (ext && (strcasecmp(ext, ".raw") == 0 || strcasecmp(ext, ".pcm") == 0))
            fmt = FMT_G711;
        else {
            fprintf(stderr, "Cannot determine format from extension. Use --wav or --g711.\n");
            return 1;
        }
    }

    FILE *f = fopen(input_path, "rb");
    if (!f) {
        perror(input_path);
        return 1;
    }

    int16_t *linear_samples = NULL;
    uint8_t *g711_codewords = NULL;
    int16_t *left_linear_samples = NULL;
    int16_t *right_linear_samples = NULL;
    uint8_t *left_g711_codewords = NULL;
    uint8_t *right_g711_codewords = NULL;
    uint8_t *adaptive_work_codewords = NULL;
    uint8_t *left_adaptive_work_codewords = NULL;
    uint8_t *right_adaptive_work_codewords = NULL;
    codeword_stream_info_t codeword_info = {0};
    codeword_stream_info_t left_codeword_info = {0};
    codeword_stream_info_t right_codeword_info = {0};
    int total_samples = 0;
    int total_codewords = 0;
    int sample_rate = 8000;

    if (fmt == FMT_WAV) {
        wav_info_t wav;
        if (!wav_read_header(f, &wav)) {
            fprintf(stderr, "Invalid WAV file: %s\n", input_path);
            fclose(f);
            return 1;
        }
        printf("WAV: %d ch, %d Hz, %d bit, %.1f seconds\n",
               wav.channels, wav.sample_rate, wav.bits_per_sample,
               (double) wav.data_bytes / (double)(wav.channels * wav.bits_per_sample / 8) / (double) wav.sample_rate);

        sample_rate = wav.sample_rate;
        int total_frames = (int)(wav.data_bytes / (wav.channels * wav.bits_per_sample / 8));

        if (wav.bits_per_sample != 16) {
            fprintf(stderr, "Only 16-bit WAV supported (got %d-bit)\n", wav.bits_per_sample);
            fclose(f);
            return 1;
        }

        /* Read all sample data */
        int16_t *raw_data = malloc((size_t) total_frames * (size_t) wav.channels * sizeof(int16_t));
        if (!raw_data) { fclose(f); return 1; }

        fseek(f, wav.data_offset, SEEK_SET);
        size_t read_samples = fread(raw_data, sizeof(int16_t), (size_t)(total_frames * wav.channels), f);
        total_frames = (int)(read_samples / (size_t) wav.channels);

        /* Extract selected channel */
        if (wav.channels == 1)
            channel = CH_MONO;

        total_samples = total_frames;
        linear_samples = malloc((size_t) total_samples * sizeof(int16_t));
        g711_codewords = malloc((size_t) total_samples);
        adaptive_work_codewords = malloc((size_t) total_samples);
        if (wav.channels >= 2 && !channel_explicit && (do_call_log || do_v34)) {
            left_linear_samples = malloc((size_t) total_samples * sizeof(int16_t));
            right_linear_samples = malloc((size_t) total_samples * sizeof(int16_t));
            left_g711_codewords = malloc((size_t) total_samples);
            right_g711_codewords = malloc((size_t) total_samples);
            left_adaptive_work_codewords = malloc((size_t) total_samples);
            right_adaptive_work_codewords = malloc((size_t) total_samples);
        }
        if (!linear_samples || !g711_codewords || !adaptive_work_codewords) { fclose(f); return 1; }

        for (int i = 0; i < total_samples; i++) {
            if (channel == CH_MONO || wav.channels == 1)
                linear_samples[i] = raw_data[i * wav.channels];
            else
                linear_samples[i] = raw_data[i * wav.channels + (int) channel];

            if (left_linear_samples && right_linear_samples) {
                left_linear_samples[i] = raw_data[i * wav.channels];
                right_linear_samples[i] = raw_data[i * wav.channels + 1];
            }
        }
        free(raw_data);

        /* Convert linear to G.711 codewords */
        total_codewords = total_samples;
        build_g711_codewords_from_linear(linear_samples, total_codewords, law, 1.0, 0, g711_codewords);
        adaptive_reslice_codewords(linear_samples, total_codewords, law,
                                   g711_codewords, adaptive_work_codewords, &codeword_info);
        if (left_g711_codewords && right_g711_codewords
            && left_adaptive_work_codewords && right_adaptive_work_codewords) {
            build_g711_codewords_from_linear(left_linear_samples, total_codewords, law, 1.0, 0, left_g711_codewords);
            adaptive_reslice_codewords(left_linear_samples, total_codewords, law,
                                       left_g711_codewords, left_adaptive_work_codewords, &left_codeword_info);
            build_g711_codewords_from_linear(right_linear_samples, total_codewords, law, 1.0, 0, right_g711_codewords);
            adaptive_reslice_codewords(right_linear_samples, total_codewords, law,
                                       right_g711_codewords, right_adaptive_work_codewords, &right_codeword_info);
        }

        printf("Channel: %s, Law: %s\n",
               channel == CH_LEFT ? "Left (TX)" : channel == CH_RIGHT ? "Right (RX)" : "Mono",
               law == V91_LAW_ULAW ? "µ-law" : "A-law");

    } else {
        /* Raw G.711 */
        fseek(f, 0, SEEK_END);
        long file_size = ftell(f);
        fseek(f, 0, SEEK_SET);

        total_codewords = (int) file_size;
        g711_codewords = malloc((size_t) total_codewords);
        if (!g711_codewords) { fclose(f); return 1; }
        total_codewords = (int) fread(g711_codewords, 1, (size_t) total_codewords, f);

        /* Convert to linear for V.8 decode */
        total_samples = total_codewords;
        linear_samples = malloc((size_t) total_samples * sizeof(int16_t));
        if (!linear_samples) { fclose(f); return 1; }
        for (int i = 0; i < total_samples; i++) {
            linear_samples[i] = (law == V91_LAW_ULAW)
                ? ulaw_to_linear(g711_codewords[i])
                : alaw_to_linear(g711_codewords[i]);
        }

        printf("G.711: %d codewords (%.1f ms), Law: %s\n",
               total_codewords, sample_to_ms(total_codewords, 8000),
               law == V91_LAW_ULAW ? "µ-law" : "A-law");
        codeword_info.direct_v90_score = v90_sequence_score(g711_codewords, total_codewords, law);
        codeword_info.selected_v90_score = codeword_info.direct_v90_score;
    }

    fclose(f);

    printf("File: %s\n", input_path);
    printf("Duration: %.3f seconds (%d samples)\n\n",
           (double) total_samples / (double) sample_rate, total_samples);

    /* Run decoders */
    {
        decode_options_t opts;
        opts.do_v34 = do_v34;
        opts.do_v8 = do_v8;
        opts.do_v91 = do_v91;
        opts.do_v90 = do_v90;
        opts.do_energy = do_energy;
        opts.do_stats = do_stats;
        opts.do_call_log = do_call_log;
        opts.raw_output_enabled = explicit_decode_output || !do_call_log;

        if (left_linear_samples && right_linear_samples && left_g711_codewords && right_g711_codewords) {
            run_decode_suite("Left (TX)", left_linear_samples, left_g711_codewords,
                             total_samples, total_codewords, sample_rate, law, &opts, &left_codeword_info);
            run_decode_suite("Right (RX)", right_linear_samples, right_g711_codewords,
                             total_samples, total_codewords, sample_rate, law, &opts, &right_codeword_info);
        } else {
            run_decode_suite(channel == CH_LEFT ? "Left (TX)"
                             : channel == CH_RIGHT ? "Right (RX)"
                             : "Mono",
                             linear_samples, g711_codewords,
                             total_samples, total_codewords, sample_rate, law, &opts, &codeword_info);
        }
    }

    free(linear_samples);
    free(g711_codewords);
    free(left_linear_samples);
    free(right_linear_samples);
    free(left_g711_codewords);
    free(right_g711_codewords);
    free(adaptive_work_codewords);
    free(left_adaptive_work_codewords);
    free(right_adaptive_work_codewords);

    return 0;
}
