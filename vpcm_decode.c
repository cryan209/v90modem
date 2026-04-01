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

            /* Only report first Sd match per U_INFO value */
            goto next_u_info;
        }
        next_u_info:;
    }

    /* Note about V.34 Phase 2 signals */
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

    call_log_append(log, total, 0, "V.90",
                    "Phase 2 INFO0a/INFO1a not decoded here",
                    "Those frames are V.34 DPSK symbols; this offline path currently logs only the PCM-visible portions.");
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
                             const decode_options_t *opts)
{
    if (!linear_samples || !g711_codewords || !opts)
        return;

    printf("\n--- Channel: %s ---\n", label);

    if (opts->raw_output_enabled && opts->do_stats)
        print_codeword_stats(g711_codewords, total_codewords, law);

    if (opts->raw_output_enabled && opts->do_energy)
        print_energy_profile(linear_samples, total_samples, sample_rate);

    if (opts->raw_output_enabled && opts->do_v8) {
        printf("\n=== V.8 Decode (as answerer) ===\n");
        decode_v8_pass(linear_samples, total_samples, false);
        printf("\n=== V.8 Decode (as caller) ===\n");
        decode_v8_pass(linear_samples, total_samples, true);
    }

    if (opts->raw_output_enabled && opts->do_v91)
        decode_v91_signals(g711_codewords, total_codewords, law);

    if (opts->raw_output_enabled && opts->do_v90)
        decode_v90_signals(g711_codewords, total_codewords, law);

    if (opts->do_call_log) {
        call_log_t log;
        call_log_init(&log);

        if (opts->do_v8) {
            collect_v8_event(&log, linear_samples, total_samples, false);
            collect_v8_event(&log, linear_samples, total_samples, true);
        }
        if (opts->do_v91)
            collect_v91_events(&log, g711_codewords, total_codewords, law);
        if (opts->do_v90)
            collect_v90_events(&log, g711_codewords, total_codewords, law);

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

    if (!do_v8 && !do_v91 && !do_v90 && !do_energy && !do_stats && !do_call_log)
        do_all = true;

    if (do_all) {
        do_v8 = do_v91 = do_v90 = do_energy = do_stats = do_call_log = true;
    }

    if (do_call_log && !do_v8 && !do_v91 && !do_v90) {
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
        if (wav.channels >= 2 && do_call_log && !channel_explicit) {
            left_linear_samples = malloc((size_t) total_samples * sizeof(int16_t));
            right_linear_samples = malloc((size_t) total_samples * sizeof(int16_t));
            left_g711_codewords = malloc((size_t) total_samples);
            right_g711_codewords = malloc((size_t) total_samples);
        }
        if (!linear_samples || !g711_codewords) { fclose(f); return 1; }

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
        for (int i = 0; i < total_codewords; i++) {
            g711_codewords[i] = (law == V91_LAW_ULAW)
                ? linear_to_ulaw(linear_samples[i])
                : linear_to_alaw(linear_samples[i]);
            if (left_g711_codewords && right_g711_codewords) {
                left_g711_codewords[i] = (law == V91_LAW_ULAW)
                    ? linear_to_ulaw(left_linear_samples[i])
                    : linear_to_alaw(left_linear_samples[i]);
                right_g711_codewords[i] = (law == V91_LAW_ULAW)
                    ? linear_to_ulaw(right_linear_samples[i])
                    : linear_to_alaw(right_linear_samples[i]);
            }
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
    }

    fclose(f);

    printf("File: %s\n", input_path);
    printf("Duration: %.3f seconds (%d samples)\n\n",
           (double) total_samples / (double) sample_rate, total_samples);

    /* Run decoders */
    {
        decode_options_t opts;
        opts.do_v8 = do_v8;
        opts.do_v91 = do_v91;
        opts.do_v90 = do_v90;
        opts.do_energy = do_energy;
        opts.do_stats = do_stats;
        opts.do_call_log = do_call_log;
        opts.raw_output_enabled = explicit_decode_output || !do_call_log;

        if (left_linear_samples && right_linear_samples && left_g711_codewords && right_g711_codewords) {
            run_decode_suite("Left (TX)", left_linear_samples, left_g711_codewords,
                             total_samples, total_codewords, sample_rate, law, &opts);
            run_decode_suite("Right (RX)", right_linear_samples, right_g711_codewords,
                             total_samples, total_codewords, sample_rate, law, &opts);
        } else {
            run_decode_suite(channel == CH_LEFT ? "Left (TX)"
                             : channel == CH_RIGHT ? "Right (RX)"
                             : "Mono",
                             linear_samples, g711_codewords,
                             total_samples, total_codewords, sample_rate, law, &opts);
        }
    }

    free(linear_samples);
    free(g711_codewords);
    free(left_linear_samples);
    free(right_linear_samples);
    free(left_g711_codewords);
    free(right_g711_codewords);

    return 0;
}
