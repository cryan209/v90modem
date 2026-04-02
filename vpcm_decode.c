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
#include "v8bis_decode.h"
#include "v92_short_phase1_decode.h"

#include <spandsp.h>
#include <spandsp/private/bitstream.h>
#include <spandsp/private/power_meter.h>
#include <spandsp/private/fsk.h>
#include <spandsp/private/modem_connect_tones.h>
#include <spandsp/private/logging.h>
#include <spandsp/private/v34.h>
#include <spandsp/private/v8.h>
#include <spandsp/tone_detect.h>

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

/* call_log_event_t and call_log_t are defined in v8bis_decode.h */

typedef struct {
    const char *key;
    const char *label;
    double freq_hz;
} visual_tone_def_t;

typedef struct {
    int sample_rate;
    int total_samples;
    int window_samples;
    int point_count;
    int freq_bin_count;
    int freq_bin_start_hz;
    int freq_bin_step_hz;
    int *envelope;
    int *freq_heatmap;
    int *tone_levels;
    int tone_count;
} audio_visualization_t;

static void call_log_init(call_log_t *log)
{
    if (!log)
        return;
    log->events = NULL;
    log->count = 0;
    log->cap = 0;
}

static void audio_visualization_reset(audio_visualization_t *viz)
{
    if (!viz)
        return;
    free(viz->envelope);
    free(viz->freq_heatmap);
    free(viz->tone_levels);
    viz->envelope = NULL;
    viz->freq_heatmap = NULL;
    viz->tone_levels = NULL;
    viz->sample_rate = 0;
    viz->total_samples = 0;
    viz->window_samples = 0;
    viz->point_count = 0;
    viz->freq_bin_count = 0;
    viz->freq_bin_start_hz = 0;
    viz->freq_bin_step_hz = 0;
    viz->tone_count = 0;
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

bool call_log_append(call_log_t *log,
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

static bool call_log_event_is_phase2_boundary(const call_log_event_t *event)
{
    if (!event)
        return false;
    if (strcmp(event->protocol, "V.34") != 0)
        return false;
    return (strncmp(event->summary, "INFO0", 5) == 0
            || strncmp(event->summary, "INFO1", 5) == 0);
}

static void call_log_prune_v8_after_phase2(call_log_t *log)
{
    int phase2_cutoff = -1;
    size_t out = 0;

    if (!log || log->count == 0)
        return;

    for (size_t i = 0; i < log->count; i++) {
        if (call_log_event_is_phase2_boundary(&log->events[i])) {
            phase2_cutoff = log->events[i].sample_offset;
            break;
        }
    }
    if (phase2_cutoff < 0)
        return;

    for (size_t i = 0; i < log->count; i++) {
        const call_log_event_t *event = &log->events[i];

        if (strcmp(event->protocol, "V.8") == 0
            && event->sample_offset >= phase2_cutoff) {
            continue;
        }
        if (out != i)
            log->events[out] = log->events[i];
        out++;
    }
    log->count = out;
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
    bool seen;
    int start_sample;
    int duration_samples;
    double peak_ratio;
    int tone_type;
} ans_fallback_hit_t;

typedef struct {
    bool seen;
    int preamble_type;
    int sample_offset;
    int byte_len;
    int score;
    uint8_t bytes[64];
    int bit_len;
    uint8_t bit_run[96];
} v8_raw_msg_hit_t;

typedef struct {
    bool seen;
    int start_sample;
    int duration_samples;
    double peak_strength;
} v8_fsk_burst_hit_t;

typedef struct {
    int preamble_type;
    uint32_t bit_stream;
    int bit_cnt;
    int zero_byte_count;
    int rx_data_ptr;
    uint8_t rx_data[64];
    int bits_seen;
    int bit_count;
    uint8_t bits[16384];
    bool calling_party;
    v8_raw_msg_hit_t best_cm_jm;
} v8_raw_scan_state_t;

/* V.8bis types, signal defs, and scan/decode functions are in v8bis_decode.h/c */

enum
{
    V8_LOCAL_CALL_FUNCTION_TAG = 0x01,
    V8_LOCAL_MODULATION_TAG = 0x05,
    V8_LOCAL_PROTOCOLS_TAG = 0x0A,
    V8_LOCAL_PSTN_ACCESS_TAG = 0x0D,
    V8_LOCAL_NSF_TAG = 0x0F,
    V8_LOCAL_PCM_MODEM_AVAILABILITY_TAG = 0x07,
    V8_LOCAL_T66_TAG = 0x0E
};

enum
{
    V8_LOCAL_SYNC_UNKNOWN = 0,
    V8_LOCAL_SYNC_CI,
    V8_LOCAL_SYNC_CM_JM,
    V8_LOCAL_SYNC_V92
};

typedef struct {
    bool ok;
    bool calling_party;
    bool cm_jm_salvaged;
    int ansam_sample;
    int ansam_tone;
    int cm_jm_raw_len;
    uint8_t cm_jm_raw[64];
    int ci_sample;
    int cm_jm_sample;
    int cj_sample;
    int v8_call_sample;
    int last_status;
    v8_parms_t result;
} v8_probe_result_t;

#define V8_EARLY_SEARCH_LIMIT_SAMPLES   ((8000 * 10) / 1)

typedef struct {
    bool info0_seen;
    bool info0_is_d;
    bool info1_seen;
    bool phase3_seen;
    bool phase4_ready_seen;
    bool phase4_seen;
    bool training_failed;
    bool u_info_from_info1a;
    bool ja_bits_from_local_tx;
    bool ja_bits_estimated;
    int info0_sample;
    int info1_sample;
    int phase3_sample;
    int tx_first_s_sample;
    int tx_first_not_s_sample;
    int tx_md_sample;
    int tx_second_s_sample;
    int tx_second_not_s_sample;
    int tx_pp_sample;
    int tx_pp_end_sample;
    int rx_pp_detect_sample;
    int rx_pp_complete_sample;
    int rx_pp_phase;
    int rx_pp_phase_score;
    int rx_pp_acquire_hits;
    bool rx_pp_started;
    int tx_trn_sample;
    int phase3_trn_lock_sample;
    int phase3_trn_strong_sample;
    int phase3_trn_lock_score;
    int tx_ja_sample;
    int tx_jdashed_sample;
    bool ja_bits_known;
    int ja_trn16;
    int ja_detector_bits;
    int ja_observed_bits;
    char ja_bits[129];
    int rx_s_event_sample;
    int rx_phase4_s_sample;
    int rx_phase4_sbar_sample;
    int rx_phase4_trn_sample;
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
    int analog_ja_sample;
    int far_end_s_sample;
    int jd_start_sample;
    int jd_frame_errors;
    int jd_repetitions;
    bool jd_prime_seen;
    int jd_prime_sample;
    int jd_prime_zero_count;
} jd_stage_decode_t;

typedef struct {
    bool ok;
    bool calling_party;
    int u_info;
    int start_sample;
    bool invert_sign;
    v90_dil_desc_t desc;
    v90_dil_analysis_t analysis;
} ja_dil_decode_t;

#define OFFLINE_V90_JD_BITS         72
#define OFFLINE_V90_JD_PRIME_BITS   12
#define OFFLINE_V90_SCRAMBLER_HISTORY 23

static const decode_v34_result_t *pick_post_phase3_source(const decode_v34_result_t *answerer,
                                                          const decode_v34_result_t *caller,
                                                          bool *calling_party_out);
static bool decode_jd_stage(const uint8_t *codewords,
                            int total_codewords,
                            const decode_v34_result_t *answerer,
                            const decode_v34_result_t *caller,
                            jd_stage_decode_t *out);
static bool decode_ja_dil_stage(const uint8_t *codewords,
                                int total_codewords,
                                const decode_v34_result_t *answerer,
                                const decode_v34_result_t *caller,
                                const jd_stage_decode_t *jd_stage,
                                ja_dil_decode_t *out);
static void collect_v91_events(call_log_t *log,
                               const uint8_t *codewords,
                               int total,
                               v91_law_t law);
static void collect_v90_events(call_log_t *log,
                               const uint8_t *codewords,
                               int total,
                               v91_law_t law);
static void collect_post_phase3_stage_events(call_log_t *log,
                                             const uint8_t *codewords,
                                             int total_codewords,
                                             const decode_v34_result_t *answerer,
                                             const decode_v34_result_t *caller);
static int codeword_to_ucode(v91_law_t law, uint8_t codeword);

static decode_v8_result_t g_v8_result;

static void v8_decode_result_handler(void *user_data, v8_parms_t *result)
{
    (void) user_data;
    g_v8_result.seen = true;
    g_v8_result.result = *result;
}

static void v8_note_first_sample(int *dst, int sample)
{
    if (!dst || sample < 0)
        return;
    if (*dst < 0)
        *dst = sample;
}

double window_energy(const int16_t *samples, int len);
double tone_energy_ratio(const int16_t *samples, int len, int sample_rate, double freq_hz, double total_energy);
static double series_energy_ratio(const double *samples, int len, int sample_rate, double freq_hz, double total_energy);

static bool detect_ansam_am(const int16_t *samples,
                            int len,
                            int sample_rate)
{
    enum { BLOCK_SAMPLES = 80 };
    static const double competitor_freqs[] = { 10.0, 12.0, 18.0, 20.0, 23.0 };
    double carrier_w;
    double carrier_cos;
    double carrier_sin;
    double osc_re = 1.0;
    double osc_im = 0.0;
    double env_energy = 0.0;
    double env_15_ratio;
    double env_competitor_peak = 0.0;
    double *env;
    double env_mean = 0.0;
    int env_len;

    if (!samples || len < BLOCK_SAMPLES * 20 || sample_rate <= 0)
        return false;

    env_len = len / BLOCK_SAMPLES;
    env = calloc((size_t) env_len, sizeof(*env));
    if (!env)
        return false;

    carrier_w = 2.0 * M_PI * 2100.0 / (double) sample_rate;
    carrier_cos = cos(carrier_w);
    carrier_sin = sin(carrier_w);

    for (int i = 0; i < env_len; i++) {
        double re = 0.0;
        double im = 0.0;

        for (int j = 0; j < BLOCK_SAMPLES; j++) {
            double sample = (double) samples[i * BLOCK_SAMPLES + j];
            double next_re = osc_re * carrier_cos - osc_im * carrier_sin;
            double next_im = osc_im * carrier_cos + osc_re * carrier_sin;

            re += sample * osc_re;
            im -= sample * osc_im;
            osc_re = next_re;
            osc_im = next_im;
        }
        env[i] = sqrt(re * re + im * im) / (double) BLOCK_SAMPLES;
        env_mean += env[i];
    }

    env_mean /= (double) env_len;
    for (int i = 0; i < env_len; i++) {
        env[i] -= env_mean;
        env_energy += env[i] * env[i];
    }
    if (env_energy <= 0.0) {
        free(env);
        return false;
    }

    env_15_ratio = series_energy_ratio(env, env_len, sample_rate / BLOCK_SAMPLES, 15.0, env_energy);
    for (size_t i = 0; i < sizeof(competitor_freqs)/sizeof(competitor_freqs[0]); i++) {
        double ratio = series_energy_ratio(env,
                                           env_len,
                                           sample_rate / BLOCK_SAMPLES,
                                           competitor_freqs[i],
                                           env_energy);
        if (ratio > env_competitor_peak)
            env_competitor_peak = ratio;
    }

    free(env);
    return (env_15_ratio >= 0.05 && env_15_ratio >= env_competitor_peak * 3.0);
}

static bool detect_standalone_ans_fallback(const int16_t *samples,
                                           int total_samples,
                                           int sample_rate,
                                           int max_sample,
                                           ans_fallback_hit_t *out)
{
    enum {
        WINDOW_SAMPLES = 160,
        MIN_RUN_WINDOWS = 40,
        MAX_GAP_WINDOWS = 4
    };
    static const double competitor_freqs[] = { 1300.0, 1375.0, 1529.0, 1650.0, 2002.0, 2225.0, 2743.0, 3000.0, 3429.0 };
    int limit;
    int run_start = -1;
    int run_windows = 0;
    int gap_windows = 0;
    double run_peak = 0.0;
    ans_fallback_hit_t best = { false, -1, 0, 0.0, MODEM_CONNECT_TONES_NONE };

    if (!samples || total_samples <= 0 || sample_rate <= 0 || !out)
        return false;

    memset(out, 0, sizeof(*out));
    out->start_sample = -1;

    limit = total_samples;
    if (max_sample > 0 && max_sample < limit)
        limit = max_sample;

    for (int start = 0; start + WINDOW_SAMPLES <= limit; start += WINDOW_SAMPLES) {
        double energy = window_energy(samples + start, WINDOW_SAMPLES);
        double ans_ratio;
        double competitor_peak = 0.0;
        bool strong_ans = false;

        if (energy <= 0.0)
            goto reset_run;

        ans_ratio = tone_energy_ratio(samples + start, WINDOW_SAMPLES, sample_rate, 2100.0, energy);
        for (size_t i = 0; i < sizeof(competitor_freqs)/sizeof(competitor_freqs[0]); i++) {
            double ratio = tone_energy_ratio(samples + start, WINDOW_SAMPLES, sample_rate, competitor_freqs[i], energy);
            if (ratio > competitor_peak)
                competitor_peak = ratio;
        }

        strong_ans = (ans_ratio >= 0.12 && ans_ratio >= competitor_peak * 2.0);
        if (strong_ans) {
            if (run_start < 0) {
                run_start = start;
                run_windows = 0;
                run_peak = 0.0;
            }
            run_windows++;
            gap_windows = 0;
            if (ans_ratio > run_peak)
                run_peak = ans_ratio;
            continue;
        }

reset_run:
        if (run_start >= 0 && gap_windows < MAX_GAP_WINDOWS) {
            gap_windows++;
            run_windows++;
            continue;
        }
        if (run_start >= 0 && run_windows >= MIN_RUN_WINDOWS) {
            best.seen = true;
            best.start_sample = run_start;
            best.duration_samples = run_windows * WINDOW_SAMPLES;
            best.peak_ratio = run_peak;
            best.tone_type = detect_ansam_am(samples + run_start,
                                             run_windows * WINDOW_SAMPLES,
                                             sample_rate)
                           ? MODEM_CONNECT_TONES_ANSAM
                           : MODEM_CONNECT_TONES_ANS;
            break;
        }
        run_start = -1;
        run_windows = 0;
        gap_windows = 0;
        run_peak = 0.0;
    }

    if (!best.seen && run_start >= 0 && run_windows >= MIN_RUN_WINDOWS) {
        best.seen = true;
        best.start_sample = run_start;
        best.duration_samples = run_windows * WINDOW_SAMPLES;
        best.peak_ratio = run_peak;
        best.tone_type = detect_ansam_am(samples + run_start,
                                         run_windows * WINDOW_SAMPLES,
                                         sample_rate)
                       ? MODEM_CONNECT_TONES_ANSAM
                       : MODEM_CONNECT_TONES_ANS;
    }

    if (!best.seen)
        return false;
    *out = best;
    return true;
}

double window_energy(const int16_t *samples, int len)
{
    double sum = 0.0;

    if (!samples || len <= 0)
        return 0.0;
    for (int i = 0; i < len; i++) {
        double v = (double) samples[i];
        sum += v * v;
    }
    return sum;
}

double tone_energy_ratio(const int16_t *samples, int len, int sample_rate, double freq_hz, double total_energy)
{
    double w;
    double cos_w;
    double sin_w;
    double osc_re;
    double osc_im;
    double re = 0.0;
    double im = 0.0;

    if (!samples || len <= 0 || sample_rate <= 0 || freq_hz <= 0.0 || total_energy <= 0.0)
        return 0.0;

    w = 2.0 * M_PI * freq_hz / (double) sample_rate;
    cos_w = cos(w);
    sin_w = sin(w);
    osc_re = 1.0;
    osc_im = 0.0;

    for (int i = 0; i < len; i++) {
        double sample = (double) samples[i];
        double next_re = osc_re * cos_w - osc_im * sin_w;
        double next_im = osc_im * cos_w + osc_re * sin_w;

        re += sample * osc_re;
        im -= sample * osc_im;
        osc_re = next_re;
        osc_im = next_im;
    }

    return (re * re + im * im) / (total_energy * (double) len);
}

static double series_energy_ratio(const double *samples, int len, int sample_rate, double freq_hz, double total_energy)
{
    double w;
    double cos_w;
    double sin_w;
    double osc_re;
    double osc_im;
    double re = 0.0;
    double im = 0.0;

    if (!samples || len <= 0 || sample_rate <= 0 || freq_hz <= 0.0 || total_energy <= 0.0)
        return 0.0;

    w = 2.0 * M_PI * freq_hz / (double) sample_rate;
    cos_w = cos(w);
    sin_w = sin(w);
    osc_re = 1.0;
    osc_im = 0.0;

    for (int i = 0; i < len; i++) {
        double sample = samples[i];
        double next_re = osc_re * cos_w - osc_im * sin_w;
        double next_im = osc_im * cos_w + osc_re * sin_w;

        re += sample * osc_re;
        im -= sample * osc_im;
        osc_re = next_re;
        osc_im = next_im;
    }

    return (re * re + im * im) / (total_energy * (double) len);
}

static const visual_tone_def_t g_visual_tone_defs[] = {
    { "ansam_2100", "2100 Hz ANSam/ANS", 2100.0 },
    { "v8_ci_1300", "1300 Hz CI", 1300.0 },
    { "v8bis_1375", "1375 Hz V.8bis", 1375.0 },
    { "v8bis_1529", "1529 Hz V.8bis", 1529.0 },
    { "v8bis_1650", "1650 Hz ESr", 1650.0 },
    { "v8bis_1900", "1900 Hz CRd", 1900.0 },
    { "v8bis_2002", "2002 Hz V.8bis", 2002.0 },
    { "v8bis_2225", "2225 Hz V.8bis", 2225.0 },
    { "v34_2743", "2743 Hz", 2743.0 },
    { "v34_2800", "2800 Hz", 2800.0 },
    { "v34_3000", "3000 Hz", 3000.0 },
    { "v34_3200", "3200 Hz", 3200.0 },
    { "v34_3429", "3429 Hz", 3429.0 }
};

static bool build_audio_visualization(audio_visualization_t *viz,
                                      const int16_t *samples,
                                      int total_samples,
                                      int sample_rate)
{
    const int window_samples = (sample_rate > 0) ? (sample_rate / 100) : 80;
    const int tone_count = (int) (sizeof(g_visual_tone_defs) / sizeof(g_visual_tone_defs[0]));
    const int freq_bin_count = 48;
    const int freq_bin_start_hz = 200;
    const int freq_bin_step_hz = 75;
    int point_count;

    if (!viz || !samples || total_samples <= 0 || sample_rate <= 0 || window_samples <= 0)
        return false;

    memset(viz, 0, sizeof(*viz));
    point_count = (total_samples + window_samples - 1) / window_samples;
    viz->envelope = calloc((size_t) point_count, sizeof(*viz->envelope));
    viz->freq_heatmap = calloc((size_t) point_count * (size_t) freq_bin_count, sizeof(*viz->freq_heatmap));
    viz->tone_levels = calloc((size_t) point_count * (size_t) tone_count, sizeof(*viz->tone_levels));
    if (!viz->envelope || !viz->freq_heatmap || !viz->tone_levels) {
        audio_visualization_reset(viz);
        return false;
    }

    viz->sample_rate = sample_rate;
    viz->total_samples = total_samples;
    viz->window_samples = window_samples;
    viz->point_count = point_count;
    viz->freq_bin_count = freq_bin_count;
    viz->freq_bin_start_hz = freq_bin_start_hz;
    viz->freq_bin_step_hz = freq_bin_step_hz;
    viz->tone_count = tone_count;

    for (int point = 0; point < point_count; point++) {
        int start = point * window_samples;
        int len = window_samples;
        double energy;
        double rms_norm;

        if (start + len > total_samples)
            len = total_samples - start;
        if (len <= 0)
            break;

        energy = window_energy(samples + start, len);
        rms_norm = sqrt(energy / (double) len) / 32768.0;
        if (rms_norm > 1.0)
            rms_norm = 1.0;
        viz->envelope[point] = (int) lrint(rms_norm * 1000.0);

        if (energy > 0.0) {
            for (int bin = 0; bin < freq_bin_count; bin++) {
                double freq_hz = (double) (freq_bin_start_hz + bin * freq_bin_step_hz);
                double ratio = tone_energy_ratio(samples + start, len, sample_rate, freq_hz, energy);
                int scaled = (int) lrint(ratio * 1600.0);

                if (scaled < 0)
                    scaled = 0;
                if (scaled > 1000)
                    scaled = 1000;
                viz->freq_heatmap[point * freq_bin_count + bin] = scaled;
            }
            for (int tone = 0; tone < tone_count; tone++) {
                double ratio = tone_energy_ratio(samples + start,
                                                 len,
                                                 sample_rate,
                                                 g_visual_tone_defs[tone].freq_hz,
                                                 energy);
                int scaled = (int) lrint(ratio * 1000.0);

                if (scaled < 0)
                    scaled = 0;
                if (scaled > 1000)
                    scaled = 1000;
                viz->tone_levels[point * tone_count + tone] = scaled;
            }
        }
    }

    return true;
}

static void json_write_escaped(FILE *f, const char *text)
{
    const unsigned char *p = (const unsigned char *) text;

    if (!f) return;
    fputc('"', f);
    if (!p) {
        fputc('"', f);
        return;
    }

    for (; *p; p++) {
        switch (*p) {
        case '\\': fputs("\\\\", f); break;
        case '"':  fputs("\\\"", f); break;
        case '\b': fputs("\\b", f); break;
        case '\f': fputs("\\f", f); break;
        case '\n': fputs("\\n", f); break;
        case '\r': fputs("\\r", f); break;
        case '\t': fputs("\\t", f); break;
        default:
            if (*p < 0x20)
                fprintf(f, "\\u%04x", (unsigned) *p);
            else
                fputc(*p, f);
            break;
        }
    }
    fputc('"', f);
}

static void write_visualization_track_json(FILE *f,
                                           const char *label,
                                           const call_log_t *log,
                                           const audio_visualization_t *viz)
{
    fprintf(f, "{sampleRate:%d,totalSamples:%d,windowSamples:%d,freqBinCount:%d,freqBinStartHz:%d,freqBinStepHz:%d,label:",
            viz->sample_rate,
            viz->total_samples,
            viz->window_samples,
            viz->freq_bin_count,
            viz->freq_bin_start_hz,
            viz->freq_bin_step_hz);
    json_write_escaped(f, label);
    fprintf(f, ",envelope:[");
    for (int i = 0; i < viz->point_count; i++) {
        if (i) fputc(',', f);
        fprintf(f, "%d", viz->envelope[i]);
    }
    fprintf(f, "],freqHeatmap:[");
    for (int i = 0; i < viz->point_count * viz->freq_bin_count; i++) {
        if (i) fputc(',', f);
        fprintf(f, "%d", viz->freq_heatmap[i]);
    }
    fprintf(f, "],tones:[");
    for (int tone = 0; tone < viz->tone_count; tone++) {
        if (tone) fputc(',', f);
        fprintf(f, "{key:");
        json_write_escaped(f, g_visual_tone_defs[tone].key);
        fprintf(f, ",label:");
        json_write_escaped(f, g_visual_tone_defs[tone].label);
        fprintf(f, ",freq:%.1f,values:[", g_visual_tone_defs[tone].freq_hz);
        for (int i = 0; i < viz->point_count; i++) {
            if (i) fputc(',', f);
            fprintf(f, "%d", viz->tone_levels[i * viz->tone_count + tone]);
        }
        fprintf(f, "]}");
    }
    fprintf(f, "],events:[");
    for (size_t i = 0; i < log->count; i++) {
        const call_log_event_t *event = &log->events[i];
        if (i) fputc(',', f);
        fprintf(f, "{start:%d,duration:%d,protocol:", event->sample_offset, event->duration_samples);
        json_write_escaped(f, event->protocol);
        fprintf(f, ",summary:");
        json_write_escaped(f, event->summary);
        fprintf(f, ",detail:");
        json_write_escaped(f, event->detail);
        fprintf(f, "}");
    }
    fprintf(f, "]}");
}

static bool write_visualization_html(const char *output_path,
                                     const char *input_path,
                                     const char *const *labels,
                                     const call_log_t *const *logs,
                                     const audio_visualization_t *const *viz_list,
                                     int track_count)
{
    FILE *f;

    if (!output_path || !input_path || !labels || !logs || !viz_list || track_count <= 0)
        return false;
    for (int i = 0; i < track_count; i++) {
        if (!labels[i] || !logs[i] || !viz_list[i]
            || !viz_list[i]->envelope || !viz_list[i]->freq_heatmap || !viz_list[i]->tone_levels)
            return false;
    }

    f = fopen(output_path, "w");
    if (!f)
        return false;

    fprintf(f,
            "<!doctype html>\n"
            "<html lang=\"en\">\n"
            "<head>\n"
            "<meta charset=\"utf-8\">\n"
            "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">\n"
            "<title>vpcm_decode visualization</title>\n"
            "<style>\n"
            ":root{--bg:#f7f3ea;--panel:#fffdf8;--ink:#1d1b18;--muted:#6a6257;--grid:#d9cfc0;--accent:#b14d2d;}\n"
            "body{margin:0;font:14px/1.4 ui-monospace,SFMono-Regular,Menlo,Monaco,monospace;background:linear-gradient(180deg,#efe6d4 0,#f7f3ea 45%%,#f3eee5 100%%);color:var(--ink);}\n"
            ".wrap{max-width:1400px;margin:0 auto;padding:24px;}\n"
            ".card{background:rgba(255,253,248,.92);backdrop-filter:blur(6px);border:1px solid #e3d9ca;border-radius:18px;box-shadow:0 18px 40px rgba(67,44,20,.08);padding:18px 20px;margin-bottom:18px;}\n"
            "h1,h2{margin:0 0 10px 0;font-weight:700;letter-spacing:.02em;}\n"
            ".meta{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:8px 16px;color:var(--muted);margin-bottom:14px;}\n"
            ".controls{display:flex;flex-wrap:wrap;gap:10px 14px;align-items:center;margin-bottom:14px;}\n"
            ".legend{display:flex;flex-wrap:wrap;gap:8px;}\n"
            ".legend button{border:1px solid #d5cab8;background:#fff8ee;color:var(--ink);padding:6px 10px;border-radius:999px;cursor:pointer;}\n"
            ".legend button.off{opacity:.45;background:#f3ede3;}\n"
            "canvas{width:100%%;height:760px;display:block;background:linear-gradient(180deg,#fffaf2 0,#fffefb 100%%);border-radius:14px;border:1px solid #eadfcf;}\n"
            ".hint{color:var(--muted);margin-top:8px;}\n"
            "table{width:100%%;border-collapse:collapse;font-size:13px;margin-top:12px;}\n"
            "th,td{text-align:left;padding:8px 10px;border-bottom:1px solid #eee3d3;vertical-align:top;}\n"
            "tbody tr{cursor:pointer;}\n"
            "tbody tr:hover{background:#fff6e7;}\n"
            ".cursor{color:var(--accent);font-weight:700;}\n"
            ".swatches{display:flex;gap:10px;align-items:center;flex-wrap:wrap;color:var(--muted);margin-top:8px;}\n"
            ".swatch{display:inline-flex;gap:6px;align-items:center;}\n"
            ".swatch i{display:inline-block;width:26px;height:10px;border-radius:999px;}\n"
            ".track-title{display:flex;justify-content:space-between;gap:10px;align-items:baseline;margin-bottom:10px;}\n"
            ".modebar{display:flex;gap:8px;flex-wrap:wrap;margin-top:12px;}\n"
            ".modebar button{border:1px solid #d5cab8;background:#fff8ee;color:var(--ink);padding:8px 12px;border-radius:999px;cursor:pointer;}\n"
            ".modebar button.active{background:#1d1b18;color:#fffdf8;border-color:#1d1b18;}\n"
            "</style>\n"
            "</head>\n"
            "<body>\n"
            "<div class=\"wrap\">\n"
            "<div class=\"card\">\n"
            "<h1>vpcm_decode HTML visualization</h1>\n"
            "<div class=\"meta\">\n");

    fprintf(f, "<div><strong>Input:</strong> ");
    json_write_escaped(f, input_path);
    fprintf(f, "</div>\n<div><strong>Tracks:</strong> %d</div>\n", track_count);
    fprintf(f, "<div><strong>Duration:</strong> %.3f s</div>\n",
            (double) viz_list[0]->total_samples / (double) viz_list[0]->sample_rate);
    fprintf(f, "<div><strong>Window:</strong> %d samples (%.1f ms)</div>\n",
            viz_list[0]->window_samples,
            sample_to_ms(viz_list[0]->window_samples, viz_list[0]->sample_rate));
    fprintf(f, "<div><strong>Tone tracks:</strong> %d</div>\n", viz_list[0]->tone_count);
    fprintf(f, "<div><strong>Spectral bins:</strong> %d</div>\n", viz_list[0]->freq_bin_count);
    fprintf(f,
            "</div>\n"
            "<div class=\"hint\">Use the selector to switch between channel views. In stereo mode, `Merged` overlays both channels without mixing their colors together.</div>\n"
            "<div class=\"swatches\"><span class=\"swatch\"><i style=\"background:linear-gradient(90deg,#08131d,#12425d,#1f8ca8,#f3b34c,#fbeec5)\"></i> weaker to stronger spectral energy</span></div>\n"
            "</div>\n"
            "<div id=\"tracks\"></div>\n"
            "</div>\n"
            "<script>\n"
            "const data={input:");
    json_write_escaped(f, input_path);
    fprintf(f, ",tracks:[");
    for (int i = 0; i < track_count; i++) {
        if (i) fputc(',', f);
        write_visualization_track_json(f, labels[i], logs[i], viz_list[i]);
    }
    fprintf(f,
            "]};\n"
            "const palette=['#b14d2d','#186d7a','#5d7a1f','#8b5cf6','#d97706','#2563eb','#a21caf','#0f766e','#c02626','#4338ca','#ca8a04','#7c3aed','#0f766e'];\n"
            "const mergePalette={a:'#c66a2b',b:'#1f7a8c'};\n"
            "function xForPoint(i,left,width,count){return left+(i/Math.max(1,count-1))*width;}\n"
            "function eventColor(protocol){if(protocol==='V.8'||protocol==='V.8bis')return '#f08a5d';if(protocol==='V.34')return '#d4a017';if(protocol==='V.90 Phase 3'||protocol==='V.90')return '#2f7f6f';if(protocol==='V.90/V.92')return '#6b5b95';if(protocol==='V.91')return '#2d5fb1';return '#777';}\n"
            "function levelNorm(v,gamma,boost){const base=Math.max(0,Math.min(1,v/1000));return Math.min(1,Math.pow(base,gamma)*boost);}\n"
            "function heatColor(v,gamma,boost){const t=levelNorm(v,gamma,boost);const stops=[[8,19,29],[18,66,93],[31,140,168],[243,179,76],[251,238,197]];const seg=Math.min(stops.length-2,Math.floor(t*(stops.length-1)));const local=t*(stops.length-1)-seg;const a=stops[seg],b=stops[seg+1];const mix=n=>Math.round(a[n]+(b[n]-a[n])*local);return `rgb(${mix(0)},${mix(1)},${mix(2)})`;}\n"
            "function mergedTrack(a,b,mode,label){return{kind:'merged',label:label||'Merged',mode:mode||'overlay',primary:a,secondary:b,sampleRate:a.sampleRate,totalSamples:a.totalSamples,windowSamples:a.windowSamples,freqBinCount:a.freqBinCount,freqBinStartHz:a.freqBinStartHz,freqBinStepHz:a.freqBinStepHz};}\n"
            "function renderTrack(track,mount){const root=mount||document.getElementById('tracks');root.innerHTML='';const merged=track.kind==='merged';const sharedSpectrogram=merged&&track.mode==='shared';const baseTrack=merged?track.primary:track;const otherTrack=merged?track.secondary:null;const card=document.createElement('div');card.className='card';card.innerHTML=`<div class=\"track-title\"><h2>${track.label}</h2><div class=\"hint\">${merged?(baseTrack.events.length+otherTrack.events.length):baseTrack.events.length} decoded events</div></div><div class=\"controls\"><div class=\"cursor\">Cursor: <span class=\"cursor-time\">0.0 ms</span></div><div class=\"hint hover-label\">Move across the chart to inspect this ${merged?(sharedSpectrogram?'shared diagnostic':'overlay'):'channel'}.</div></div><div class=\"controls\"><label>Spectral lift <input class=\"spec-boost\" type=\"range\" min=\"100\" max=\"900\" step=\"25\" value=\"320\"></label><label>Contrast <input class=\"spec-gamma\" type=\"range\" min=\"20\" max=\"100\" step=\"1\" value=\"42\"></label><label>Tone lift <input class=\"tone-boost\" type=\"range\" min=\"100\" max=\"900\" step=\"25\" value=\"260\"></label></div><div class=\"legend\"></div><canvas width=\"1360\" height=\"760\"></canvas><div class=\"hint\">${merged?(sharedSpectrogram?`Shared spectrogram with split tone overlays. ${baseTrack.label} uses orange and ${otherTrack.label} uses teal.`:`Merged overlay view. ${baseTrack.label} uses orange and ${otherTrack.label} uses teal.`):'Separate channel render.'} Lower-energy structure is display-boosted only; raw decode data is unchanged.</div><table><thead><tr><th>Start</th><th>Duration</th><th>Protocol</th><th>Summary</th><th>Detail</th></tr></thead><tbody></tbody></table>`;root.appendChild(card);const canvas=card.querySelector('canvas');const ctx=canvas.getContext('2d');const legend=card.querySelector('.legend');const tbody=card.querySelector('tbody');const cursorTime=card.querySelector('.cursor-time');const hoverLabel=card.querySelector('.hover-label');const specBoostInput=card.querySelector('.spec-boost');const specGammaInput=card.querySelector('.spec-gamma');const toneBoostInput=card.querySelector('.tone-boost');const enabled=new Set(baseTrack.tones.map(t=>t.key));let cursorIndex=0;function settings(){return{specBoost:Number(specBoostInput.value)/100,specGamma:Number(specGammaInput.value)/100,toneBoost:Number(toneBoostInput.value)/100};}function allEvents(){if(!merged)return baseTrack.events;return baseTrack.events.map(e=>({...e,trackLabel:baseTrack.label,trackColor:mergePalette.a})).concat(otherTrack.events.map(e=>({...e,trackLabel:otherTrack.label,trackColor:mergePalette.b}))).sort((x,y)=>x.start-y.start);}function draw(){const cfg=settings();const w=canvas.width,h=canvas.height;ctx.clearRect(0,0,w,h);const left=64,right=20,top=24,bottom=28;const plotW=w-left-right;const envelopeH=150;const gap=28;const specTop=top+envelopeH+gap;const specH=300;const toneTop=specTop+specH+gap;const toneAreaH=h-toneTop-bottom;const toneTrackH=toneAreaH/Math.max(1,baseTrack.tones.length);ctx.fillStyle='#fffdf8';ctx.fillRect(0,0,w,h);for(let i=0;i<=10;i++){const x=left+(plotW*i/10);ctx.strokeStyle='#d9cfc0';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(x,top);ctx.lineTo(x,h-bottom);ctx.stroke();const ms=(baseTrack.totalSamples*i/10)*1000/baseTrack.sampleRate;ctx.fillStyle='#6a6257';ctx.fillText(ms.toFixed(0)+' ms',x-16,h-8);}for(let i=0;i<=6;i++){const y=top+(envelopeH*i/6);ctx.strokeStyle='#d9cfc0';ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(left+plotW,y);ctx.stroke();}ctx.fillStyle='#1d1b18';ctx.fillText(merged?'Envelope + Decoded Events (Overlay)':'Envelope + Decoded Events',left,16);allEvents().forEach(ev=>{const x1=left+(ev.start/baseTrack.totalSamples)*plotW;const x2=left+((ev.start+Math.max(ev.duration,baseTrack.windowSamples))/baseTrack.totalSamples)*plotW;ctx.fillStyle=(merged?(ev.trackColor||eventColor(ev.protocol)):eventColor(ev.protocol))+'38';ctx.fillRect(x1,top,Math.max(2,x2-x1),envelopeH);ctx.strokeStyle=merged?(ev.trackColor||eventColor(ev.protocol)):eventColor(ev.protocol);ctx.beginPath();ctx.moveTo(x1,top);ctx.lineTo(x1,specTop+specH);ctx.stroke();});const envA=baseTrack.envelope;const envB=merged?otherTrack.envelope:null;ctx.beginPath();for(let i=0;i<envA.length;i++){const x=xForPoint(i,left,plotW,envA.length);const v=merged?Math.max(envA[i],envB[i]):envA[i];const y=top+envelopeH-(v/1000)*envelopeH;if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);}ctx.lineTo(left+plotW,top+envelopeH);ctx.lineTo(left,top+envelopeH);ctx.closePath();ctx.fillStyle=merged?'rgba(90,94,128,.20)':'rgba(177,77,45,.22)';ctx.fill();ctx.strokeStyle=merged?'#4f6b8a':'#b14d2d';ctx.lineWidth=1.6;ctx.stroke();if(merged){for(const series of [{env:envA,color:mergePalette.a},{env:envB,color:mergePalette.b}]){ctx.beginPath();series.env.forEach((v,i)=>{const x=xForPoint(i,left,plotW,series.env.length);const y=top+envelopeH-(v/1000)*envelopeH;if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);});ctx.strokeStyle=series.color;ctx.lineWidth=1.05;ctx.stroke();}}ctx.fillStyle='#1d1b18';ctx.fillText(merged?(sharedSpectrogram?'Frequency Diagnosis (Shared)':'Frequency Diagnosis (Overlay)'):'Frequency Diagnosis',left,specTop-8);const colW=plotW/Math.max(1,baseTrack.envelope.length);const rowH=specH/Math.max(1,baseTrack.freqBinCount);for(let i=0;i<baseTrack.envelope.length;i++){const x=left+i*colW;for(let b=0;b<baseTrack.freqBinCount;b++){const y=specTop+specH-(b+1)*rowH;if(merged&&sharedSpectrogram){const va=baseTrack.freqHeatmap[i*baseTrack.freqBinCount+b];const vb=otherTrack.freqHeatmap[i*otherTrack.freqBinCount+b];const v=va>vb?va:vb;ctx.fillStyle=heatColor(v,cfg.specGamma,cfg.specBoost);ctx.fillRect(x,y,Math.max(1,colW+0.4),Math.max(1,rowH+0.4));}else if(merged){const va=baseTrack.freqHeatmap[i*baseTrack.freqBinCount+b];const vb=otherTrack.freqHeatmap[i*otherTrack.freqBinCount+b];const a=levelNorm(va,cfg.specGamma,cfg.specBoost);const bb=levelNorm(vb,cfg.specGamma,cfg.specBoost);ctx.fillStyle=`rgba(198,106,43,${Math.min(.95,a)})`;ctx.fillRect(x,y,Math.max(1,colW+0.4),Math.max(1,rowH+0.4));ctx.fillStyle=`rgba(31,122,140,${Math.min(.95,bb)})`;ctx.fillRect(x,y,Math.max(1,colW+0.4),Math.max(1,rowH+0.4));}else{const v=baseTrack.freqHeatmap[i*baseTrack.freqBinCount+b];ctx.fillStyle=heatColor(v,cfg.specGamma,cfg.specBoost);ctx.fillRect(x,y,Math.max(1,colW+0.4),Math.max(1,rowH+0.4));}}}ctx.strokeStyle='#d9cfc0';for(let j=0;j<=6;j++){const freq=baseTrack.freqBinStartHz+j*((baseTrack.freqBinCount-1)*baseTrack.freqBinStepHz/6);const bin=(freq-baseTrack.freqBinStartHz)/baseTrack.freqBinStepHz;const y=specTop+specH-bin*rowH;ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(left+plotW,y);ctx.stroke();ctx.fillStyle='#6a6257';ctx.fillText(Math.round(freq)+' Hz',6,y+4);}ctx.strokeStyle='#8a7f70';ctx.strokeRect(left,specTop,plotW,specH);ctx.fillStyle='#1d1b18';ctx.fillText(merged?(sharedSpectrogram?'Tracked Tone Energies (Split)':'Tracked Tone Energies (Overlay)'):'Tracked Tone Energies',left,toneTop-8);baseTrack.tones.forEach((tone,toneIdx)=>{const y0=toneTop+toneIdx*toneTrackH;ctx.strokeStyle='#e6ddcf';ctx.beginPath();ctx.moveTo(left,y0+toneTrackH);ctx.lineTo(left+plotW,y0+toneTrackH);ctx.stroke();ctx.fillStyle='#6a6257';ctx.fillText(tone.label,left-4,y0+12);if(!enabled.has(tone.key))return;const drawTone=(values,color,width)=>{ctx.beginPath();values.forEach((v,i)=>{const x=xForPoint(i,left,plotW,baseTrack.envelope.length);const display=levelNorm(v,Math.min(0.85,cfg.specGamma+0.06),cfg.toneBoost);const y=y0+toneTrackH-display*(toneTrackH-6)-3;if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);});ctx.strokeStyle=color;ctx.lineWidth=width;ctx.stroke();};if(merged){drawTone(baseTrack.tones[toneIdx].values,sharedSpectrogram?'rgba(198,106,43,0.95)':mergePalette.a,sharedSpectrogram?1.3:1.15);drawTone(otherTrack.tones[toneIdx].values,sharedSpectrogram?'rgba(31,122,140,0.95)':mergePalette.b,sharedSpectrogram?1.3:1.15);}else{drawTone(baseTrack.tones[toneIdx].values,palette[toneIdx%%palette.length],1.35);}});const seen=new Set();allEvents().forEach(ev=>{const tag=`${ev.trackLabel||''}:${ev.protocol}:${ev.summary}`;if(seen.has(tag))return;seen.add(tag);const x=left+(ev.start/baseTrack.totalSamples)*plotW;ctx.save();ctx.translate(x+2,top+12);ctx.rotate(-Math.PI/10);ctx.fillStyle=merged?(ev.trackColor||eventColor(ev.protocol)):eventColor(ev.protocol);const prefix=merged&&ev.trackLabel?`[${ev.trackLabel}] `:'';const text=prefix+ev.summary;ctx.fillText(text.length>24?text.slice(0,24)+'...':text,0,0);ctx.restore();});const cursorX=xForPoint(cursorIndex,left,plotW,baseTrack.envelope.length);ctx.strokeStyle='#111';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(cursorX,top);ctx.lineTo(cursorX,h-bottom);ctx.stroke();}function updateCursor(index){const cfg=settings();cursorIndex=Math.max(0,Math.min(baseTrack.envelope.length-1,index));const start=cursorIndex*baseTrack.windowSamples;cursorTime.textContent=(start*1000/baseTrack.sampleRate).toFixed(1)+' ms';const toneSource=merged?baseTrack.tones.map(t=>({label:`${baseTrack.label} ${t.label}`,v:t.values[cursorIndex]})).concat(otherTrack.tones.map(t=>({label:`${otherTrack.label} ${t.label}`,v:t.values[cursorIndex]}))):baseTrack.tones.map(t=>({label:t.label,v:t.values[cursorIndex]}));const active=toneSource.sort((a,b)=>b.v-a.v).slice(0,4).filter(x=>x.v>6);let dominantA=0,dominantValA=-1;for(let b=0;b<baseTrack.freqBinCount;b++){const v=baseTrack.freqHeatmap[cursorIndex*baseTrack.freqBinCount+b];if(v>dominantValA){dominantValA=v;dominantA=b;}}let spectral=`dominant bin ${baseTrack.freqBinStartHz+dominantA*baseTrack.freqBinStepHz} Hz raw ${(dominantValA/10).toFixed(1)}%% display x${cfg.specBoost.toFixed(1)}`;if(merged){let dominantB=0,dominantValB=-1;for(let b=0;b<otherTrack.freqBinCount;b++){const v=otherTrack.freqHeatmap[cursorIndex*otherTrack.freqBinCount+b];if(v>dominantValB){dominantValB=v;dominantB=b;}}spectral=sharedSpectrogram?`shared ${baseTrack.freqBinStartHz+((dominantValA>dominantValB?dominantA:dominantB)*baseTrack.freqBinStepHz)} Hz | ${baseTrack.label}: ${(baseTrack.freqBinStartHz+dominantA*baseTrack.freqBinStepHz)} Hz ${(dominantValA/10).toFixed(1)}%% | ${otherTrack.label}: ${(otherTrack.freqBinStartHz+dominantB*otherTrack.freqBinStepHz)} Hz ${(dominantValB/10).toFixed(1)}%%`:`${baseTrack.label}: ${(baseTrack.freqBinStartHz+dominantA*baseTrack.freqBinStepHz)} Hz ${(dominantValA/10).toFixed(1)}%% | ${otherTrack.label}: ${(otherTrack.freqBinStartHz+dominantB*otherTrack.freqBinStepHz)} Hz ${(dominantValB/10).toFixed(1)}%%`;}hoverLabel.textContent=active.length?`${spectral} | `+active.map(x=>x.label+' raw '+(x.v/10).toFixed(1)+'%%').join(' | '):spectral;draw();}function rebuildLegend(){legend.innerHTML='';baseTrack.tones.forEach((tone,toneIdx)=>{const b=document.createElement('button');b.textContent=tone.label;b.style.borderColor=merged?mergePalette.a:palette[toneIdx%%palette.length];if(!enabled.has(tone.key))b.classList.add('off');b.onclick=()=>{if(enabled.has(tone.key))enabled.delete(tone.key);else enabled.add(tone.key);rebuildLegend();updateCursor(cursorIndex);};legend.appendChild(b);});if(merged){const key=document.createElement('div');key.className='hint';key.textContent=sharedSpectrogram?`${baseTrack.label} and ${otherTrack.label} share the spectrogram; tone overlays stay split orange/teal`:`${baseTrack.label} = orange, ${otherTrack.label} = teal`;legend.appendChild(key);}}canvas.addEventListener('mousemove',ev=>{const r=canvas.getBoundingClientRect();const x=(ev.clientX-r.left)*(canvas.width/r.width);const left=64;const plotW=canvas.width-left-20;updateCursor(Math.round(((x-left)/plotW)*(baseTrack.envelope.length-1)));});canvas.addEventListener('click',ev=>{const r=canvas.getBoundingClientRect();const x=(ev.clientX-r.left)*(canvas.width/r.width);const sample=((x-64)/(canvas.width-84))*baseTrack.totalSamples;let best=null;allEvents().forEach((e,rowIdx)=>{const end=e.start+Math.max(e.duration,baseTrack.windowSamples);if(sample>=e.start&&sample<=end)best=rowIdx;});if(best!==null){tbody.querySelectorAll('tr')[best]?.scrollIntoView({block:'center'});}});specBoostInput.addEventListener('input',()=>updateCursor(cursorIndex));specGammaInput.addEventListener('input',()=>updateCursor(cursorIndex));toneBoostInput.addEventListener('input',()=>updateCursor(cursorIndex));allEvents().forEach(ev=>{const tr=document.createElement('tr');const prefix=merged&&ev.trackLabel?`[${ev.trackLabel}] `:'';tr.innerHTML=`<td>${(ev.start*1000/baseTrack.sampleRate).toFixed(1)} ms</td><td>${(ev.duration*1000/baseTrack.sampleRate).toFixed(1)} ms</td><td>${ev.protocol}</td><td>${prefix}${ev.summary}</td><td>${ev.detail||''}</td>`;tr.onclick=()=>updateCursor(Math.round(ev.start/baseTrack.windowSamples));tbody.appendChild(tr);});rebuildLegend();updateCursor(0);}\n"
            "function initViewer(){const root=document.getElementById('tracks');if(data.tracks.length<=1){renderTrack(data.tracks[0],root);return;}root.innerHTML='';const shell=document.createElement('div');shell.className='card';shell.innerHTML='<div class=\"track-title\"><h2>View Selector</h2><div class=\"hint\">Choose Left, Right, Merged, or Split</div></div><div class=\"hint selector-copy\"></div><div class=\"modebar\"></div><div class=\"swatches merge-key\"></div><div class=\"track-mount\"></div>';root.appendChild(shell);const modebar=shell.querySelector('.modebar');const selectorCopy=shell.querySelector('.selector-copy');const mergeKey=shell.querySelector('.merge-key');const mount=shell.querySelector('.track-mount');const views=[{button:'L',hint:data.tracks[0].label,track:data.tracks[0]},{button:'R',hint:data.tracks[1].label,track:data.tracks[1]},{button:'Merged',hint:`Overlay ${data.tracks[0].label} and ${data.tracks[1].label}`,track:mergedTrack(data.tracks[0],data.tracks[1],'overlay','Merged')},{button:'Split',hint:`Shared spectrogram with split ${data.tracks[0].label}/${data.tracks[1].label} tones`,track:mergedTrack(data.tracks[0],data.tracks[1],'shared','Merged Split') }];function setView(idx){modebar.querySelectorAll('button').forEach((x,i)=>x.classList.toggle('active',i===idx));const view=views[idx];selectorCopy.textContent=view.hint;mergeKey.innerHTML=idx>=2?`<span class=\"swatch\"><i style=\"background:${mergePalette.a}\"></i>${data.tracks[0].label}</span><span class=\"swatch\"><i style=\"background:${mergePalette.b}\"></i>${data.tracks[1].label}</span>`:'';renderTrack(view.track,mount);}views.forEach((view,idx)=>{const b=document.createElement('button');b.textContent=view.button;b.title=view.hint;b.onclick=()=>setView(idx);modebar.appendChild(b);});setView(0);}\n"
            "initViewer();\n"
            "</script>\n"
            "</body>\n"
            "</html>\n");

    fclose(f);
    return true;
}

/* scan_v8bis_signals and scan_v8bis_weak_candidate moved to v8bis_decode.c */

static bool v8_parse_cm_jm_candidate(v8_parms_t *dst,
                                     const uint8_t *data,
                                     int len,
                                     bool calling_party)
{
    const uint8_t *p;
    const uint8_t *end;
    int modulations;
    bool parsed_any = false;

    if (!dst || !data || len <= 0)
        return false;

    memset(dst, 0, sizeof(*dst));
    dst->status = V8_STATUS_V8_OFFERED;
    p = data;
    end = data + len;

    while (p < end && *p) {
        switch (*p & 0x1F) {
        case V8_LOCAL_CALL_FUNCTION_TAG:
            dst->jm_cm.call_function = (*p >> 5) & 0x07;
            p++;
            parsed_any = true;
            break;
        case V8_LOCAL_MODULATION_TAG:
            modulations = 0;
            if (*p & 0x80)
                modulations |= V8_MOD_V34HDX;
            if (*p & 0x40)
                modulations |= V8_MOD_V34;
            if (*p & 0x20)
                modulations |= V8_MOD_V90;
            p++;
            if (p < end && (*p & 0x38) == 0x10) {
                if (*p & 0x80)
                    modulations |= V8_MOD_V27TER;
                if (*p & 0x40)
                    modulations |= V8_MOD_V29;
                if (*p & 0x04)
                    modulations |= V8_MOD_V17;
                if (*p & 0x02)
                    modulations |= V8_MOD_V22;
                if (*p & 0x01)
                    modulations |= V8_MOD_V32;
                p++;
                if (p < end && (*p & 0x38) == 0x10) {
                    if (*p & 0x80)
                        modulations |= V8_MOD_V21;
                    if (*p & 0x40)
                        modulations |= V8_MOD_V23HDX;
                    if (*p & 0x04)
                        modulations |= V8_MOD_V23;
                    if (*p & 0x02)
                        modulations |= V8_MOD_V26BIS;
                    if (*p & 0x01)
                        modulations |= V8_MOD_V26TER;
                    p++;
                }
            }
            if (!calling_party)
                modulations &= V8_MOD_V90 | V8_MOD_V34 | V8_MOD_V22
                             | V8_MOD_V92 | V8_MOD_V21 | V8_MOD_V17
                             | V8_MOD_V29 | V8_MOD_V27TER | V8_MOD_V23
                             | V8_MOD_V32;
            dst->jm_cm.modulations = modulations;
            parsed_any = true;
            break;
        case V8_LOCAL_PROTOCOLS_TAG:
            dst->jm_cm.protocols = (*p >> 5) & 0x07;
            p++;
            parsed_any = true;
            break;
        case V8_LOCAL_PSTN_ACCESS_TAG:
            dst->jm_cm.pstn_access = (*p >> 5) & 0x07;
            p++;
            parsed_any = true;
            break;
        case V8_LOCAL_NSF_TAG:
            dst->jm_cm.nsf = 1;
            p++;
            while (p < end && (*p & 0x38) == 0x10)
                p++;
            parsed_any = true;
            break;
        case V8_LOCAL_PCM_MODEM_AVAILABILITY_TAG:
            dst->jm_cm.pcm_modem_availability = (*p >> 5) & 0x07;
            p++;
            parsed_any = true;
            break;
        case V8_LOCAL_T66_TAG:
            dst->jm_cm.t66 = (*p >> 5) & 0x07;
            p++;
            parsed_any = true;
            break;
        default:
            p++;
            break;
        }
        while (p < end && (*p & 0x38) == 0x10)
            p++;
    }

    return parsed_any;
}

static int v8_candidate_score(const v8_parms_t *parsed, int len)
{
    int score = 0;

    if (!parsed || len <= 0)
        return -1;

    score += len * 8;
    if (parsed->jm_cm.call_function != 0)
        score += 120;
    if (parsed->jm_cm.modulations != 0)
        score += 180;
    if (parsed->jm_cm.protocols != 0)
        score += 70;
    if (parsed->jm_cm.pcm_modem_availability != 0)
        score += 90;
    if (parsed->jm_cm.pstn_access != 0)
        score += 60;
    if (parsed->jm_cm.t66 >= 0)
        score += 20;
    if (parsed->jm_cm.nsf >= 0)
        score += 10;
    return score;
}

static void v8_raw_scan_process_message(v8_raw_scan_state_t *scan,
                                        int preamble_type,
                                        bool calling_party)
{
    v8_parms_t parsed;
    v8_raw_msg_hit_t hit;
    int approx_end_sample;

    if (!scan || preamble_type != V8_LOCAL_SYNC_CM_JM || scan->rx_data_ptr <= 0)
        return;
    if (!v8_parse_cm_jm_candidate(&parsed, scan->rx_data, scan->rx_data_ptr, calling_party))
        return;

    memset(&hit, 0, sizeof(hit));
    hit.seen = true;
    hit.byte_len = scan->rx_data_ptr;
    hit.score = v8_candidate_score(&parsed, scan->rx_data_ptr);
    if (hit.score < 0)
        return;
    if (hit.byte_len > (int) sizeof(hit.bytes))
        hit.byte_len = (int) sizeof(hit.bytes);
    memcpy(hit.bytes, scan->rx_data, (size_t) hit.byte_len);

    approx_end_sample = (scan->bits_seen * 8000) / 300;
    hit.sample_offset = approx_end_sample - (scan->rx_data_ptr * 10 * 8000) / 300;
    if (hit.sample_offset < 0)
        hit.sample_offset = 0;

    if (!scan->best_cm_jm.seen || hit.score > scan->best_cm_jm.score)
        scan->best_cm_jm = hit;
}

static void v8_raw_scan_put_bit(void *user_data, int bit)
{
    v8_raw_scan_state_t *scan = user_data;
    int new_preamble_type;
    uint8_t data;

    if (!scan)
        return;
    if (bit < 0)
        return;

    if (scan->bit_count < (int) sizeof(scan->bits))
        scan->bits[scan->bit_count++] = (uint8_t) (bit & 1);
    scan->bits_seen++;
    scan->bit_stream = (scan->bit_stream >> 1) | ((uint32_t) bit << 19);
    switch (scan->bit_stream) {
    case 0x803FF:
        new_preamble_type = V8_LOCAL_SYNC_CI;
        break;
    case 0xF03FF:
        new_preamble_type = V8_LOCAL_SYNC_CM_JM;
        break;
    case 0xAABFF:
        new_preamble_type = V8_LOCAL_SYNC_V92;
        break;
    default:
        new_preamble_type = V8_LOCAL_SYNC_UNKNOWN;
        break;
    }

    if (new_preamble_type != V8_LOCAL_SYNC_UNKNOWN) {
        v8_raw_scan_process_message(scan, scan->preamble_type, scan->calling_party);
        scan->preamble_type = new_preamble_type;
        scan->bit_cnt = 0;
        scan->rx_data_ptr = 0;
        scan->zero_byte_count = 0;
    }

    if (scan->preamble_type != V8_LOCAL_SYNC_UNKNOWN) {
        scan->bit_cnt++;
        if ((scan->bit_stream & 0x80400) == 0x80000 && scan->bit_cnt >= 10) {
            data = (uint8_t) ((scan->bit_stream >> 11) & 0xFF);
            if (data == 0)
                scan->zero_byte_count++;
            else
                scan->zero_byte_count = 0;
            if (scan->rx_data_ptr < (int) (sizeof(scan->rx_data) - 1))
                scan->rx_data[scan->rx_data_ptr++] = data;
            scan->bit_cnt = 0;
        }
    }
}

static bool v8_decode_async_octet(const uint8_t *bits, int bit_count, int start, uint8_t *octet)
{
    uint8_t data = 0;

    if (!bits || !octet || start < 0 || start + 10 > bit_count)
        return false;
    if (bits[start] != 0 || bits[start + 9] != 1)
        return false;
    for (int i = 0; i < 8; i++)
        data |= (uint8_t) (bits[start + 1 + i] & 1) << i;
    *octet = data;
    return true;
}

static int v8_sync_score(const uint8_t *bits,
                         const double *confidence,
                         int bit_count,
                         int start,
                         const uint8_t pattern[10])
{
    double score = 0.0;

    if (!bits || !confidence || !pattern || start < 0 || start + 10 > bit_count)
        return -1000000;

    for (int i = 0; i < 10; i++) {
        double w = confidence[start + i];

        if (w < 0.015)
            w = 0.015;
        if (bits[start + i] == pattern[i])
            score += 1.0 + 12.0 * w;
        else
            score -= 1.0 + 12.0 * w;
    }

    return (int) lround(score * 10.0);
}

static int v8_ones_lock_score(const uint8_t *bits,
                              const double *confidence,
                              int bit_count,
                              int start)
{
    static const uint8_t ones_pattern[10] = {
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1
    };

    return v8_sync_score(bits, confidence, bit_count, start, ones_pattern);
}

static int v8_sync_lock_score(const uint8_t *bits,
                              const double *confidence,
                              int bit_count,
                              int start,
                              int preamble_type)
{
    static const uint8_t cm_jm_sync_pattern[10] = {
        0, 0, 0, 0, 0, 0, 1, 1, 1, 1
    };
    static const uint8_t v92_sync_pattern[10] = {
        0, 1, 0, 1, 0, 1, 0, 1, 0, 1
    };

    if (preamble_type == V8_LOCAL_SYNC_CM_JM)
        return v8_sync_score(bits, confidence, bit_count, start, cm_jm_sync_pattern);
    if (preamble_type == V8_LOCAL_SYNC_V92)
        return v8_sync_score(bits, confidence, bit_count, start, v92_sync_pattern);
    return -1000000;
}

static int v8_decode_soft_async_bytes(const uint8_t *bits,
                                      const double *confidence,
                                      int bit_count,
                                      int start,
                                      uint8_t *bytes,
                                      int max_bytes,
                                      int *framing_penalty_out)
{
    int byte_len = 0;
    int penalty = 0;

    if (!bits || !confidence || !bytes || max_bytes <= 0 || start < 0)
        return 0;

    for (int bit_pos = start; bit_pos + 10 <= bit_count && byte_len < max_bytes; bit_pos += 10) {
        uint8_t byte = 0;

        if (bits[bit_pos] != 0)
            penalty += 20 + (int) lround(confidence[bit_pos] * 80.0);
        if (bits[bit_pos + 9] != 1)
            penalty += 20 + (int) lround(confidence[bit_pos + 9] * 80.0);
        for (int j = 0; j < 8; j++)
            byte |= (uint8_t) (bits[bit_pos + 1 + j] & 1) << j;
        bytes[byte_len++] = byte;
        if (byte == 0)
            break;
    }

    if (framing_penalty_out)
        *framing_penalty_out = penalty;
    return byte_len;
}

static bool v8_decode_soft_async_octet(const uint8_t *bits,
                                       const double *confidence,
                                       int bit_count,
                                       int start,
                                       uint8_t *octet,
                                       int *framing_penalty_out)
{
    uint8_t byte = 0;
    int penalty = 0;

    if (!bits || !confidence || !octet || start < 0 || start + 10 > bit_count)
        return false;

    if (bits[start] != 0)
        penalty += 20 + (int) lround(confidence[start] * 80.0);
    if (bits[start + 9] != 1)
        penalty += 20 + (int) lround(confidence[start + 9] * 80.0);
    for (int j = 0; j < 8; j++)
        byte |= (uint8_t) (bits[start + 1 + j] & 1) << j;

    *octet = byte;
    if (framing_penalty_out)
        *framing_penalty_out = penalty;
    return true;
}

static void v8_syncless_scan_bits(v8_raw_scan_state_t *scan)
{
    v8_raw_msg_hit_t hit;

    if (!scan || scan->bit_count < 20)
        return;

    for (int start = 0; start + 20 <= scan->bit_count; start++) {
        uint8_t first;
        uint8_t bytes[64];
        int byte_len = 0;
        int bit = start;
        v8_parms_t parsed;

        if (!v8_decode_async_octet(scan->bits, scan->bit_count, start, &first))
            continue;
        if (first != 0xE0)
            continue;

        bit += 10;
        while (bit + 10 <= scan->bit_count && byte_len < (int) sizeof(bytes)) {
            uint8_t data;

            if (!v8_decode_async_octet(scan->bits, scan->bit_count, bit, &data))
                break;
            bytes[byte_len++] = data;
            bit += 10;
            if (data == 0)
                break;
        }
        if (byte_len <= 0)
            continue;
        if (!v8_parse_cm_jm_candidate(&parsed, bytes, byte_len, scan->calling_party))
            continue;

        memset(&hit, 0, sizeof(hit));
        hit.seen = true;
        hit.byte_len = byte_len;
        hit.sample_offset = (start * 8000) / 300;
        hit.score = v8_candidate_score(&parsed, byte_len) - 15;
        memcpy(hit.bytes, bytes, (size_t) byte_len);

        if (!scan->best_cm_jm.seen || hit.score > scan->best_cm_jm.score)
            scan->best_cm_jm = hit;
    }
}

static bool v8_scan_raw_cm_jm(const int16_t *samples,
                              int total_samples,
                              int max_sample,
                              bool calling_party,
                              v8_raw_msg_hit_t *out)
{
    int limit;
    v8_raw_msg_hit_t best = {0};

    if (!samples || total_samples <= 0 || !out)
        return false;

    limit = total_samples;
    if (max_sample > 0 && max_sample < limit)
        limit = max_sample;

    for (int pass = 0; pass < 4; pass++) {
        v8_raw_scan_state_t scan;
        fsk_rx_state_t fsk;
        int16_t *scratch = NULL;
        const int16_t *src = samples;
        bool invert = (pass & 1) != 0;
        bool use_ch2 = (pass & 2) != 0;

        memset(&scan, 0, sizeof(scan));
        scan.preamble_type = V8_LOCAL_SYNC_UNKNOWN;
        scan.calling_party = calling_party;

        if (invert) {
            scratch = malloc((size_t) limit * sizeof(*scratch));
            if (!scratch)
                continue;
            for (int i = 0; i < limit; i++)
                scratch[i] = (int16_t) -samples[i];
            src = scratch;
        }

        fsk_rx_init(&fsk,
                    &preset_fsk_specs[use_ch2 ? FSK_V21CH2 : FSK_V21CH1],
                    FSK_FRAME_MODE_ASYNC,
                    v8_raw_scan_put_bit,
                    &scan);
        fsk_rx_set_signal_cutoff(&fsk, -45.5f);
        fsk_rx(&fsk, src, limit);
        v8_raw_scan_process_message(&scan, scan.preamble_type, calling_party);
        if (!scan.best_cm_jm.seen)
            v8_syncless_scan_bits(&scan);

        if (scan.best_cm_jm.seen) {
            int score = scan.best_cm_jm.score;

            if (use_ch2 == calling_party)
                score += 25;
            if (invert)
                score -= 5;
            scan.best_cm_jm.score = score;
            if (!best.seen || score > best.score)
                best = scan.best_cm_jm;
        }

        free(scratch);
    }

    if (!best.seen)
        return false;
    *out = best;
    return true;
}

static bool v8_detect_v21_burst(const int16_t *samples,
                                int total_samples,
                                int sample_rate,
                                int search_start,
                                int search_end,
                                bool use_ch2,
                                v8_fsk_burst_hit_t *out)
{
    enum {
        WINDOW_SAMPLES = 160,
        STEP_SAMPLES = 80,
        MIN_RUN_WINDOWS = 3
    };
    const double f0 = use_ch2 ? 1650.0 : 980.0;
    const double f1 = use_ch2 ? 1850.0 : 1180.0;
    int run_start = -1;
    int run_windows = 0;
    double run_peak = 0.0;
    v8_fsk_burst_hit_t best = { false, 0, 0, 0.0 };

    if (!samples || total_samples <= 0 || sample_rate <= 0 || !out)
        return false;

    if (search_start < 0)
        search_start = 0;
    if (search_end <= 0 || search_end > total_samples)
        search_end = total_samples;
    if (search_end - search_start < WINDOW_SAMPLES)
        return false;

    for (int start = search_start; start + WINDOW_SAMPLES <= search_end; start += STEP_SAMPLES) {
        double energy = window_energy(samples + start, WINDOW_SAMPLES);
        double p0;
        double p1;
        double strength;

        if (energy <= 0.0)
            goto reset_run;
        p0 = tone_energy_ratio(samples + start, WINDOW_SAMPLES, sample_rate, f0, energy);
        p1 = tone_energy_ratio(samples + start, WINDOW_SAMPLES, sample_rate, f1, energy);
        strength = p0 + p1;
        if (strength >= 0.18) {
            if (run_start < 0) {
                run_start = start;
                run_windows = 0;
                run_peak = 0.0;
            }
            run_windows++;
            if (strength > run_peak)
                run_peak = strength;
            continue;
        }

reset_run:
        if (run_start >= 0 && run_windows >= MIN_RUN_WINDOWS) {
            best.seen = true;
            best.start_sample = run_start;
            best.duration_samples = run_windows * STEP_SAMPLES + (WINDOW_SAMPLES - STEP_SAMPLES);
            best.peak_strength = run_peak;
            break;
        }
        run_start = -1;
        run_windows = 0;
        run_peak = 0.0;
    }

    if (!best.seen && run_start >= 0 && run_windows >= MIN_RUN_WINDOWS) {
        best.seen = true;
        best.start_sample = run_start;
        best.duration_samples = run_windows * STEP_SAMPLES + (WINDOW_SAMPLES - STEP_SAMPLES);
        best.peak_strength = run_peak;
    }

    if (!best.seen)
        return false;
    *out = best;
    return true;
}

static bool v8_targeted_v21_decode(const int16_t *samples,
                                   int total_samples,
                                   int sample_rate,
                                   const v8_fsk_burst_hit_t *burst,
                                   bool use_ch2,
                                   bool calling_party,
                                   v8_raw_msg_hit_t *out)
{
    const double mark_hz = use_ch2 ? 1650.0 : 980.0;
    const double space_hz = use_ch2 ? 1850.0 : 1180.0;
    int search_start;
    int search_end;
    v8_raw_msg_hit_t best = {0};

    if (!samples || total_samples <= 0 || sample_rate <= 0 || !burst || !burst->seen || !out)
        return false;

    search_start = burst->start_sample - 800;
    search_end = burst->start_sample + burst->duration_samples + 320;
    if (search_start < 0)
        search_start = 0;
    if (search_end > total_samples)
        search_end = total_samples;
    if (search_end - search_start < (sample_rate / 300) * 12)
        return false;

    for (int invert = 0; invert <= 1; invert++) {
        for (int bit_rate = 296; bit_rate <= 304; bit_rate++) {
            double symbol_samples = (double) sample_rate / (double) bit_rate;

            for (int phase_q = 0; phase_q < (int) (symbol_samples * 4.0); phase_q++) {
                double phase = (double) phase_q / 4.0;
                uint8_t bits[512];
                double confidence[512];
                int bit_count = 0;

                for (int k = 0; ; k++) {
                    int a = search_start + (int) lround(phase + (double) k * symbol_samples);
                    int b = search_start + (int) lround(phase + (double) (k + 1) * symbol_samples);
                    double e;
                    double pm;
                    double ps;
                    int bit;

                    if (b <= a || b > search_end || bit_count >= (int) sizeof(bits))
                        break;
                    e = window_energy(samples + a, b - a);
                    if (e <= 0.0)
                        break;
                    pm = tone_energy_ratio(samples + a, b - a, sample_rate, mark_hz, e);
                    ps = tone_energy_ratio(samples + a, b - a, sample_rate, space_hz, e);
                    bit = (pm >= ps) ? 1 : 0;
                    if (invert)
                        bit ^= 1;
                    bits[bit_count++] = (uint8_t) bit;
                    confidence[bit_count - 1] = fabs(pm - ps);
                }

                for (int i = 0; i + 20 <= bit_count; i++) {
                    uint8_t payload[64];
                    int payload_len;
                    int framing_penalty = 0;
                    int ones_score;
                    int sync_score;
                    int score_bias;
                    uint8_t first_tag;
                    uint8_t second_tag;
                    v8_parms_t parsed;

                    ones_score = v8_ones_lock_score(bits, confidence, bit_count, i);
                    if (ones_score < 40)
                        continue;
                    sync_score = v8_sync_lock_score(bits, confidence, bit_count, i + 10, V8_LOCAL_SYNC_CM_JM);
                    if (sync_score < 40)
                        continue;

                    payload_len = v8_decode_soft_async_bytes(bits,
                                                             confidence,
                                                             bit_count,
                                                             i + 20,
                                                             payload,
                                                             (int) sizeof(payload),
                                                             &framing_penalty);
                    if (payload_len <= 0)
                        continue;
                    first_tag = payload[0] & 0x1F;
                    second_tag = (payload_len > 1) ? (payload[1] & 0x1F) : 0xFF;
                    if (payload_len < 2)
                        continue;
                    if (first_tag != V8_LOCAL_CALL_FUNCTION_TAG
                        || second_tag != V8_LOCAL_MODULATION_TAG)
                        continue;
                    if (!v8_parse_cm_jm_candidate(&parsed, payload, payload_len, calling_party))
                        continue;

                    score_bias = ones_score + sync_score;
                    score_bias += (bit_rate == 300) ? 8 : (6 - abs(bit_rate - 300));
                    score_bias -= framing_penalty;
                    if (!best.seen || v8_candidate_score(&parsed, payload_len) + score_bias > best.score) {
                        memset(&best, 0, sizeof(best));
                        best.seen = true;
                        best.preamble_type = V8_LOCAL_SYNC_CM_JM;
                        best.sample_offset = search_start + (int) lround(phase + ((double) i * symbol_samples));
                        best.byte_len = payload_len;
                        best.score = v8_candidate_score(&parsed, payload_len) + score_bias;
                        memcpy(best.bytes, payload, (size_t) payload_len);
                    }
                }
            }
        }
    }

    if (!best.seen)
        return false;
    *out = best;
    return true;
}

static bool v8_targeted_v21_bytes_candidate(const int16_t *samples,
                                            int total_samples,
                                            int sample_rate,
                                            const v8_fsk_burst_hit_t *burst,
                                            bool use_ch2,
                                            v8_raw_msg_hit_t *out)
{
    const double mark_hz = use_ch2 ? 1650.0 : 980.0;
    const double space_hz = use_ch2 ? 1850.0 : 1180.0;
    int search_start;
    int search_end;
    v8_raw_msg_hit_t best = {0};

    if (!samples || total_samples <= 0 || sample_rate <= 0 || !burst || !burst->seen || !out)
        return false;

    search_start = burst->start_sample - 800;
    search_end = burst->start_sample + burst->duration_samples + 320;
    if (search_start < 0)
        search_start = 0;
    if (search_end > total_samples)
        search_end = total_samples;

    for (int invert = 0; invert <= 1; invert++) {
        for (int bit_rate = 296; bit_rate <= 304; bit_rate++) {
            double symbol_samples = (double) sample_rate / (double) bit_rate;

            for (int phase_q = 0; phase_q < (int) (symbol_samples * 4.0); phase_q++) {
                double phase = (double) phase_q / 4.0;
                uint8_t bits[512];
                double confidence[512];
                int bit_count = 0;

                for (int k = 0; ; k++) {
                    int a = search_start + (int) lround(phase + (double) k * symbol_samples);
                    int b = search_start + (int) lround(phase + (double) (k + 1) * symbol_samples);
                    double e;
                    double pm;
                    double ps;
                    int bit;

                    if (b <= a || b > search_end || bit_count >= (int) sizeof(bits))
                        break;
                    e = window_energy(samples + a, b - a);
                    if (e <= 0.0)
                        break;
                    pm = tone_energy_ratio(samples + a, b - a, sample_rate, mark_hz, e);
                    ps = tone_energy_ratio(samples + a, b - a, sample_rate, space_hz, e);
                    bit = (pm >= ps) ? 1 : 0;
                    if (invert)
                        bit ^= 1;
                    bits[bit_count++] = (uint8_t) bit;
                    confidence[bit_count - 1] = fabs(pm - ps);
                }

                for (int i = 0; i + 20 <= bit_count; i++) {
                    uint8_t bytes[16];
                    int byte_len = 0;
                    int score = 0;
                    int framing_penalty = 0;
                    int ones_score;
                    int sync_score_v92;
                    int sync_score_cm_jm;
                    uint8_t prev_tag = 0xFF;

                    ones_score = v8_ones_lock_score(bits, confidence, bit_count, i);
                    if (ones_score < 40)
                        continue;

                    sync_score_v92 = v8_sync_lock_score(bits, confidence, bit_count, i + 10, V8_LOCAL_SYNC_V92);
                    if (sync_score_v92 >= 40) {
                        int repeat_penalty = 0;
                        int tail_ones_score = 0;

                        byte_len = 0;
                        framing_penalty = 0;
                        if (v8_decode_soft_async_octet(bits,
                                                       confidence,
                                                       bit_count,
                                                       i + 20,
                                                       &bytes[byte_len],
                                                       &framing_penalty)) {
                            byte_len++;
                        }
                        if (i + 60 <= bit_count) {
                            int repeat_ones_score = v8_ones_lock_score(bits, confidence, bit_count, i + 30);
                            int repeat_sync_score = v8_sync_lock_score(bits, confidence, bit_count, i + 40, V8_LOCAL_SYNC_V92);
                            uint8_t repeat_byte = 0;
                            int repeat_frame_penalty = 0;

                            if (repeat_ones_score >= 40 && repeat_sync_score >= 40
                                && v8_decode_soft_async_octet(bits,
                                                              confidence,
                                                              bit_count,
                                                              i + 50,
                                                              &repeat_byte,
                                                              &repeat_frame_penalty)) {
                                bytes[byte_len++] = repeat_byte;
                                repeat_penalty += repeat_frame_penalty;
                                score = ones_score + sync_score_v92 + repeat_ones_score + repeat_sync_score;
                                if (byte_len >= 2 && bytes[0] == bytes[1])
                                    score += 30;
                                else if (byte_len >= 2)
                                    score -= 20;
                                if (i + 70 <= bit_count) {
                                    tail_ones_score = v8_ones_lock_score(bits, confidence, bit_count, i + 60);
                                    if (tail_ones_score >= 40)
                                        score += tail_ones_score;
                                }
                            } else {
                                score = ones_score + sync_score_v92;
                            }
                        } else {
                            score = ones_score + sync_score_v92;
                        }
                        if (byte_len > 0) {
                            score += byte_len * 12 + 8 - abs(bit_rate - 300) - framing_penalty - repeat_penalty - i / 2;
                            if (invert)
                                score -= 2;
                            if (!best.seen || score > best.score) {
                                memset(&best, 0, sizeof(best));
                                best.seen = true;
                                best.preamble_type = V8_LOCAL_SYNC_V92;
                                best.sample_offset = search_start + (int) lround(phase + ((double) i * symbol_samples));
                                best.byte_len = byte_len;
                                best.score = score;
                                memcpy(best.bytes, bytes, (size_t) byte_len);
                                best.bit_len = bit_count - i;
                                if (best.bit_len > 70)
                                    best.bit_len = 70;
                                if (best.bit_len > (int) sizeof(best.bit_run))
                                    best.bit_len = (int) sizeof(best.bit_run);
                                memcpy(best.bit_run, bits + i, (size_t) best.bit_len);
                            }
                        }
                    }

                    sync_score_cm_jm = v8_sync_lock_score(bits, confidence, bit_count, i + 10, V8_LOCAL_SYNC_CM_JM);
                    if (sync_score_cm_jm < 40)
                        continue;

                    byte_len = v8_decode_soft_async_bytes(bits,
                                                          confidence,
                                                          bit_count,
                                                          i + 20,
                                                          bytes,
                                                          (int) sizeof(bytes),
                                                          &framing_penalty);
                    if (byte_len < 2)
                        continue;
                    if ((bytes[0] & 0x1F) != V8_LOCAL_CALL_FUNCTION_TAG
                        || (bytes[1] & 0x1F) != V8_LOCAL_MODULATION_TAG)
                        continue;

                    score = 0;
                    for (int j = 0; j < byte_len; j++) {
                        uint8_t b = bytes[j];
                        uint8_t tag = b & 0x1F;

                        if (b == 0xE0) {
                            score += 40;
                        } else if (j == 0 && tag == V8_LOCAL_CALL_FUNCTION_TAG) {
                            score += 20;
                        } else if (j == 1 && tag == V8_LOCAL_MODULATION_TAG) {
                            score += 18;
                        } else if (tag == V8_LOCAL_CALL_FUNCTION_TAG
                                   || tag == V8_LOCAL_MODULATION_TAG
                                   || tag == V8_LOCAL_PROTOCOLS_TAG
                                   || tag == V8_LOCAL_PSTN_ACCESS_TAG
                                   || tag == V8_LOCAL_PCM_MODEM_AVAILABILITY_TAG
                                   || tag == V8_LOCAL_T66_TAG
                                   || tag == V8_LOCAL_NSF_TAG) {
                            score += 8;
                        } else if ((tag & 0x18) == 0x10
                                   && (prev_tag == V8_LOCAL_MODULATION_TAG || (prev_tag & 0x18) == 0x10)) {
                            score += 6;
                        } else if (tag == 0) {
                            score -= 6;
                        } else {
                            score -= 4;
                        }
                        prev_tag = tag;
                    }
                    score += ones_score + sync_score_cm_jm;
                    score += byte_len * 2;
                    score += 8 - abs(bit_rate - 302);
                    if (use_ch2)
                        score += 6;
                    score -= framing_penalty;
                    score -= i / 2;
                    if (invert)
                        score -= 2;
                    if (!best.seen || score > best.score) {
                        memset(&best, 0, sizeof(best));
                        best.seen = true;
                        best.preamble_type = V8_LOCAL_SYNC_CM_JM;
                        best.sample_offset = search_start + (int) lround(phase + ((double) i * symbol_samples));
                        best.byte_len = byte_len;
                        best.score = score;
                        memcpy(best.bytes, bytes, (size_t) byte_len);
                        best.bit_len = bit_count - i;
                        if (best.bit_len > 70)
                            best.bit_len = 70;
                        if (best.bit_len > (int) sizeof(best.bit_run))
                            best.bit_len = (int) sizeof(best.bit_run);
                        memcpy(best.bit_run, bits + i, (size_t) best.bit_len);
                    }
                }
            }
        }
    }

    if (!best.seen)
        return false;
    *out = best;
    return true;
}


static void format_hex_bytes(char *buf, size_t len, const uint8_t *bytes, int byte_len)
{
    size_t used = 0;

    if (!buf || len == 0)
        return;
    buf[0] = '\0';
    if (!bytes || byte_len <= 0)
        return;

    for (int i = 0; i < byte_len; i++) {
        int written = snprintf(buf + used,
                               len - used,
                               "%s%02X",
                               (i == 0) ? "" : " ",
                               bytes[i]);
        if (written < 0 || (size_t) written >= len - used)
            break;
        used += (size_t) written;
    }
}

static void format_bit_slice(char *buf, size_t len, const uint8_t *bits, int start, int count)
{
    size_t used = 0;

    if (!buf || len == 0)
        return;
    buf[0] = '\0';
    if (!bits || start < 0 || count <= 0)
        return;

    for (int i = 0; i < count && used + 2 < len; i++) {
        buf[used++] = bits[start + i] ? '1' : '0';
    }
    buf[used] = '\0';
}

static const char *v8_preamble_to_candidate_label(int preamble_type, bool calling_party)
{
    switch (preamble_type) {
    case V8_LOCAL_SYNC_CM_JM:
        return calling_party ? "CM-like" : "JM-like";
    case V8_LOCAL_SYNC_V92:
        return "V.92 short Phase 1 control-like";
    default:
        return calling_party ? "CM-like" : "JM-like";
    }
}

static const char *v8_preamble_detail(int preamble_type)
{
    switch (preamble_type) {
    case V8_LOCAL_SYNC_CM_JM:
        return "V.8 CM/JM-style preamble";
    case V8_LOCAL_SYNC_V92:
        return "V.92 short Phase 1 sync (10 ones + 0101010101)";
    default:
        return "unclassified V.21 burst";
    }
}

static bool v8_scan_v92_window_candidate(const int16_t *samples,
                                         int total_samples,
                                         int sample_rate,
                                         int search_start,
                                         int search_end,
                                         bool use_ch2,
                                         int expected_qca,
                                         int expected_digital,
                                         v8_raw_msg_hit_t *out)
{
    const double mark_hz = use_ch2 ? 1650.0 : 980.0;
    const double space_hz = use_ch2 ? 1850.0 : 1180.0;
    v8_raw_msg_hit_t best = {0};

    if (!samples || total_samples <= 0 || sample_rate <= 0 || !out)
        return false;
    if (search_start < 0)
        search_start = 0;
    if (search_end <= 0 || search_end > total_samples)
        search_end = total_samples;
    if (search_end - search_start < 160)
        return false;

    for (int invert = 0; invert <= 1; invert++) {
        for (int bit_rate = 296; bit_rate <= 304; bit_rate++) {
            double symbol_samples = (double) sample_rate / (double) bit_rate;

            for (int phase_q = 0; phase_q < (int) (symbol_samples * 4.0); phase_q++) {
                double phase = (double) phase_q / 4.0;
                uint8_t bits[512];
                double confidence[512];
                int bit_count = 0;

                for (int k = 0; ; k++) {
                    int a = search_start + (int) lround(phase + (double) k * symbol_samples);
                    int b = search_start + (int) lround(phase + (double) (k + 1) * symbol_samples);
                    double e;
                    double pm;
                    double ps;
                    int bit;

                    if (b <= a || b > search_end || bit_count >= (int) sizeof(bits))
                        break;
                    e = window_energy(samples + a, b - a);
                    if (e <= 0.0)
                        break;
                    pm = tone_energy_ratio(samples + a, b - a, sample_rate, mark_hz, e);
                    ps = tone_energy_ratio(samples + a, b - a, sample_rate, space_hz, e);
                    bit = (pm >= ps) ? 1 : 0;
                    if (invert)
                        bit ^= 1;
                    bits[bit_count++] = (uint8_t) bit;
                    confidence[bit_count - 1] = fabs(pm - ps);
                }

                for (int i = 0; i + 30 <= bit_count; i++) {
                    int ones_score;
                    int sync_score;
                    int score;
                    v92_short_phase1_candidate_t v92c;

                    ones_score = v8_ones_lock_score(bits, confidence, bit_count, i);
                    if (ones_score < 32)
                        continue;
                    sync_score = v8_sync_lock_score(bits, confidence, bit_count, i + 10, V8_LOCAL_SYNC_V92);
                    if (sync_score < 32)
                        continue;
                    if (!v92_decode_short_phase1_candidate(bits + i, bit_count - i, use_ch2, &v92c))
                        continue;
                    if (expected_qca >= 0 && (int) v92c.qca != expected_qca)
                        continue;
                    if (expected_digital >= 0 && (int) v92c.digital_modem != expected_digital)
                        continue;

                    score = ones_score + sync_score;
                    if (v92c.repeat_seen)
                        score += v92c.repeat_match ? 30 : 8;
                    if (v92c.qca == use_ch2)
                        score += 10;
                    if (bit_rate == 300)
                        score += 8;
                    else
                        score += 6 - abs(bit_rate - 300);
                    score -= i / 2;
                    if (invert)
                        score -= 2;

                    if (!best.seen || score > best.score) {
                        memset(&best, 0, sizeof(best));
                        best.seen = true;
                        best.preamble_type = V8_LOCAL_SYNC_V92;
                        best.sample_offset = search_start + (int) lround(phase + ((double) i * symbol_samples));
                        best.score = score;
                        best.bit_len = bit_count - i;
                        if (best.bit_len > 70)
                            best.bit_len = 70;
                        if (best.bit_len > (int) sizeof(best.bit_run))
                            best.bit_len = (int) sizeof(best.bit_run);
                        memcpy(best.bit_run, bits + i, (size_t) best.bit_len);
                    }
                }
            }
        }
    }

    if (!best.seen)
        return false;
    *out = best;
    return true;
}

static void v8_salvage_partial_probe(v8_state_t *v8,
                                     int sample_offset,
                                     v8_probe_result_t *out)
{
    v8_parms_t parsed;

    if (!v8 || !out)
        return;
    if (out->cm_jm_sample >= 0 || v8->cm_jm_len <= 0)
        return;
    if (!v8_parse_cm_jm_candidate(&parsed, v8->cm_jm_data, v8->cm_jm_len, out->calling_party))
        return;

    out->result = parsed;
    out->last_status = parsed.status;
    out->cm_jm_sample = sample_offset;
    out->cm_jm_salvaged = true;
    out->cm_jm_raw_len = v8->cm_jm_len;
    if (out->cm_jm_raw_len > (int) sizeof(out->cm_jm_raw))
        out->cm_jm_raw_len = (int) sizeof(out->cm_jm_raw);
    memcpy(out->cm_jm_raw, v8->cm_jm_data, (size_t) out->cm_jm_raw_len);
}

static int v8_probe_last_sample(const v8_probe_result_t *probe)
{
    int end = -1;

    if (!probe)
        return -1;
    if (probe->ansam_sample >= 0)
        end = probe->ansam_sample;
    if (end < 0 || (probe->ci_sample >= 0 && probe->ci_sample > end))
        end = probe->ci_sample;
    if (end < 0 || (probe->cm_jm_sample >= 0 && probe->cm_jm_sample > end))
        end = probe->cm_jm_sample;
    if (end < 0 || (probe->cj_sample >= 0 && probe->cj_sample > end))
        end = probe->cj_sample;
    if (end < 0 || (probe->v8_call_sample >= 0 && probe->v8_call_sample > end))
        end = probe->v8_call_sample;
    return end;
}

static int v8_probe_first_sample(const v8_probe_result_t *probe)
{
    int start = -1;

    if (!probe)
        return -1;
    if (probe->ansam_sample >= 0)
        start = probe->ansam_sample;
    if (start < 0 || (probe->ci_sample >= 0 && probe->ci_sample < start))
        start = probe->ci_sample;
    if (start < 0 || (probe->cm_jm_sample >= 0 && probe->cm_jm_sample < start))
        start = probe->cm_jm_sample;
    if (start < 0 || (probe->cj_sample >= 0 && probe->cj_sample < start))
        start = probe->cj_sample;
    if (start < 0 || (probe->v8_call_sample >= 0 && probe->v8_call_sample < start))
        start = probe->v8_call_sample;
    return start;
}

static int v8_probe_milestone_count(const v8_probe_result_t *probe)
{
    int count = 0;

    if (!probe)
        return 0;
    if (probe->ansam_sample >= 0)
        count++;
    if (probe->ci_sample >= 0)
        count++;
    if (probe->cm_jm_sample >= 0)
        count++;
    if (probe->cj_sample >= 0)
        count++;
    if (probe->v8_call_sample >= 0 || probe->last_status == V8_STATUS_V8_CALL)
        count++;
    return count;
}

static int v8_probe_chain_score(const v8_probe_result_t *probe)
{
    int score = 0;
    int prev = -1;
    int ordered_steps = 0;
    const int milestones[] = {
        probe ? probe->ansam_sample : -1,
        probe ? probe->ci_sample : -1,
        probe ? probe->cm_jm_sample : -1,
        probe ? probe->cj_sample : -1,
        probe ? probe->v8_call_sample : -1
    };

    if (!probe || !probe->ok)
        return -1;

    if (probe->ansam_sample >= 0)
        score += 100;
    if (probe->ci_sample >= 0)
        score += 300;
    if (probe->cm_jm_sample >= 0)
        score += 700;
    if (probe->cj_sample >= 0)
        score += 1000;
    if (probe->v8_call_sample >= 0 || probe->last_status == V8_STATUS_V8_CALL)
        score += 1400;
    if (probe->result.jm_cm.pcm_modem_availability == V8_PSTN_PCM_MODEM_V90_V92_DIGITAL
        || probe->result.jm_cm.pcm_modem_availability == (V8_PSTN_PCM_MODEM_V90_V92_DIGITAL | V8_PSTN_PCM_MODEM_V90_V92_ANALOGUE))
        score += 200;
    if (probe->result.jm_cm.pstn_access & V8_PSTN_ACCESS_DCE_ON_DIGITAL)
        score += 120;

    for (size_t i = 0; i < sizeof(milestones)/sizeof(milestones[0]); i++) {
        if (milestones[i] < 0)
            continue;
        if (prev < 0 || milestones[i] >= prev) {
            ordered_steps++;
            prev = milestones[i];
        } else {
            score -= 500;
        }
    }
    score += ordered_steps * 120;

    prev = v8_probe_last_sample(probe);
    if (prev >= 0)
        score -= prev / 160;
    return score;
}

static int v8_probe_role_score(const v8_probe_result_t *probe)
{
    int score;
    int first_sample;
    int last_sample;

    if (!probe || !probe->ok)
        return -1;

    score = v8_probe_chain_score(probe);
    first_sample = v8_probe_first_sample(probe);
    last_sample = v8_probe_last_sample(probe);

    if (probe->calling_party) {
        if (probe->ci_sample >= 0)
            score += 600;
        if (probe->cm_jm_sample >= 0)
            score += 500;
        if (probe->ci_sample >= 0 && probe->cm_jm_sample >= 0 && probe->cm_jm_sample >= probe->ci_sample)
            score += 250;
        if (probe->cm_jm_sample >= 0 && probe->ci_sample < 0)
            score -= 900;
        if (probe->ansam_sample >= 0 && probe->ci_sample < 0 && probe->cm_jm_sample < 0)
            score -= 350;
    } else {
        if (probe->ansam_sample >= 0)
            score += 450;
        if (probe->cm_jm_sample >= 0)
            score += 700;
        if (probe->cj_sample >= 0)
            score += 250;
        if (probe->ansam_sample >= 0 && probe->cm_jm_sample >= 0 && probe->cm_jm_sample >= probe->ansam_sample)
            score += 350;
    }

    if (probe->cm_jm_salvaged && probe->cm_jm_raw_len <= 1)
        score -= 700;

    if (first_sample >= 0 && last_sample >= first_sample)
        score -= (last_sample - first_sample) / 320;
    score += v8_probe_milestone_count(probe) * 150;
    return score;
}

static const char *v8_answer_tone_name(int tone)
{
    switch (tone) {
    case MODEM_CONNECT_TONES_ANSAM:
        return "ANSam";
    case MODEM_CONNECT_TONES_ANSAM_PR:
        return "ANSam+PR";
    case MODEM_CONNECT_TONES_ANS:
        return "ANS";
    case MODEM_CONNECT_TONES_ANS_PR:
        return "ANS+PR";
    default:
        return "ANS/ANSam";
    }
}

static const char *v8_answer_tone_detail(int tone)
{
    switch (tone) {
    case MODEM_CONNECT_TONES_ANSAM:
        return "2100 Hz answer tone with phase reversals disabled";
    case MODEM_CONNECT_TONES_ANSAM_PR:
        return "2100 Hz answer tone with phase reversals";
    case MODEM_CONNECT_TONES_ANS:
        return "2100 Hz answer tone without ANSam modulation";
    case MODEM_CONNECT_TONES_ANS_PR:
        return "2100 Hz answer tone with phase reversals and no ANSam modulation";
    default:
        return "2100 Hz answer tone";
    }
}

static bool v8_collect_probe(const int16_t *samples,
                             int total_samples,
                             bool calling_party,
                             int max_sample,
                             v8_probe_result_t *out)
{
    v8_parms_t parms;
    v8_state_t *v8;
    int offset = 0;
    int limit;

    if (!samples || total_samples <= 0 || !out)
        return false;

    memset(out, 0, sizeof(*out));
    out->calling_party = calling_party;
    out->ansam_sample = -1;
    out->ansam_tone = MODEM_CONNECT_TONES_NONE;
    out->cm_jm_raw_len = 0;
    out->ci_sample = -1;
    out->cm_jm_sample = -1;
    out->cj_sample = -1;
    out->v8_call_sample = -1;
    out->last_status = V8_STATUS_IN_PROGRESS;

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
        return false;

    span_log_set_level(v8_get_logging_state(v8), SPAN_LOG_SHOW_SEVERITY | SPAN_LOG_WARNING);
    limit = total_samples;
    if (max_sample > 0 && max_sample < limit)
        limit = max_sample;

    while (offset < limit) {
        int chunk = limit - offset;
        int tone;

        if (chunk > 160)
            chunk = 160;

        modem_connect_tones_rx(&v8->ansam_rx, samples + offset, chunk);
        if ((tone = modem_connect_tones_rx_get(&v8->ansam_rx)) != MODEM_CONNECT_TONES_NONE) {
            if (tone == MODEM_CONNECT_TONES_ANSAM
                || tone == MODEM_CONNECT_TONES_ANSAM_PR
                || tone == MODEM_CONNECT_TONES_ANS
                || tone == MODEM_CONNECT_TONES_ANS_PR) {
                v8_note_first_sample(&out->ansam_sample, offset + chunk);
                if (out->ansam_tone == MODEM_CONNECT_TONES_NONE)
                    out->ansam_tone = tone;
            }
        }
        modem_connect_tones_rx(&v8->calling_tone_rx, samples + offset, chunk);
        modem_connect_tones_rx_get(&v8->calling_tone_rx);
        modem_connect_tones_rx(&v8->cng_tone_rx, samples + offset, chunk);
        modem_connect_tones_rx_get(&v8->cng_tone_rx);
        fsk_rx(&v8->v21rx, samples + offset, chunk);
        offset += chunk;

        if (v8->got_ci)
            v8_note_first_sample(&out->ci_sample, offset);
        if (v8->got_cm_jm)
            v8_note_first_sample(&out->cm_jm_sample, offset);
        if (v8->got_cj)
            v8_note_first_sample(&out->cj_sample, offset);
        if (g_v8_result.seen) {
            out->result = g_v8_result.result;
            out->last_status = g_v8_result.result.status;
            if (g_v8_result.result.status == V8_STATUS_V8_CALL)
                v8_note_first_sample(&out->v8_call_sample, offset);
        }
    }

    v8_salvage_partial_probe(v8, offset, out);
    if (out->cm_jm_sample < 0) {
        v8_raw_msg_hit_t raw_hit;
        v8_parms_t parsed;

        if (v8_scan_raw_cm_jm(samples, total_samples, limit, calling_party, &raw_hit)
            && v8_parse_cm_jm_candidate(&parsed, raw_hit.bytes, raw_hit.byte_len, calling_party)) {
            out->result = parsed;
            out->last_status = parsed.status;
            out->cm_jm_sample = raw_hit.sample_offset;
            out->cm_jm_salvaged = true;
            out->cm_jm_raw_len = raw_hit.byte_len;
            memcpy(out->cm_jm_raw, raw_hit.bytes, (size_t) raw_hit.byte_len);
        } else if (out->ansam_sample >= 0) {
            v8_fsk_burst_hit_t burst;

            if (v8_detect_v21_burst(samples,
                                    total_samples,
                                    8000,
                                    out->ansam_sample + 12000,
                                    limit,
                                    !calling_party,
                                    &burst)
                && v8_targeted_v21_decode(samples,
                                          total_samples,
                                          8000,
                                          &burst,
                                          !calling_party,
                                          calling_party,
                                          &raw_hit)
                && v8_parse_cm_jm_candidate(&parsed, raw_hit.bytes, raw_hit.byte_len, calling_party)) {
                out->result = parsed;
                out->last_status = parsed.status;
                out->cm_jm_sample = raw_hit.sample_offset;
                out->cm_jm_salvaged = true;
                out->cm_jm_raw_len = raw_hit.byte_len;
                memcpy(out->cm_jm_raw, raw_hit.bytes, (size_t) raw_hit.byte_len);
            }
        }
    }

    if (out->ansam_sample < 0) {
        ans_fallback_hit_t ans_hit;

        if (detect_standalone_ans_fallback(samples, total_samples, 8000, limit, &ans_hit)) {
            out->ansam_sample = ans_hit.start_sample;
            out->ansam_tone = ans_hit.tone_type;
        }
    }

    out->ok = (out->ansam_sample >= 0
               || out->ci_sample >= 0
               || out->cm_jm_sample >= 0
               || out->cj_sample >= 0
               || out->v8_call_sample >= 0);

    v8_free(v8);
    return out->ok;
}

static bool v8_select_best_probe_core(const int16_t *samples,
                                      int total_samples,
                                      bool calling_party,
                                      int max_sample,
                                      v8_probe_result_t *out)
{
    v8_probe_result_t best;
    int best_score = -1;
    int search_limit;

    if (!samples || total_samples <= 0 || !out)
        return false;

    memset(&best, 0, sizeof(best));
    best.ansam_sample = -1;
    best.ci_sample = -1;
    best.cm_jm_sample = -1;
    best.cj_sample = -1;
    best.v8_call_sample = -1;

    search_limit = total_samples;
    if (max_sample > 0 && max_sample < search_limit)
        search_limit = max_sample;
    if (search_limit > V8_EARLY_SEARCH_LIMIT_SAMPLES)
        search_limit = V8_EARLY_SEARCH_LIMIT_SAMPLES;

    for (int window_end = 3200; window_end <= search_limit; window_end += 1600) {
        v8_probe_result_t candidate;
        int score;

        if (!v8_collect_probe(samples, total_samples, calling_party, window_end, &candidate))
            continue;
        score = v8_probe_chain_score(&candidate);
        if (score > best_score) {
            best = candidate;
            best_score = score;
        }
    }

    if (search_limit > 0) {
        v8_probe_result_t full_candidate;
        int full_score;

        if (v8_collect_probe(samples, total_samples, calling_party, search_limit, &full_candidate)) {
            full_score = v8_probe_chain_score(&full_candidate);
            if (full_score > best_score) {
                best = full_candidate;
                best_score = full_score;
            }
        }
    }

    if (best_score < 0)
        return false;
    if (best.ansam_sample >= 0) {
        ans_fallback_hit_t refined_ans;

        if (detect_standalone_ans_fallback(samples, total_samples, 8000, search_limit, &refined_ans)) {
            int delta = refined_ans.start_sample - best.ansam_sample;

            if (delta < 0)
                delta = -delta;
            if (delta <= 800 && refined_ans.tone_type == MODEM_CONNECT_TONES_ANSAM)
                best.ansam_tone = MODEM_CONNECT_TONES_ANSAM;
        }
    }
    *out = best;
    return true;
}

static bool v8_select_best_probe(const int16_t *samples,
                                 int total_samples,
                                 bool calling_party,
                                 int max_sample,
                                 v8_probe_result_t *out)
{
    v8_probe_result_t normal_probe;
    v8_probe_result_t inverted_probe;
    int normal_score = -1;
    int inverted_score = -1;
    int16_t *inverted_samples = NULL;
    bool have_normal;
    bool have_inverted = false;

    if (!samples || total_samples <= 0 || !out)
        return false;

    have_normal = v8_select_best_probe_core(samples, total_samples, calling_party, max_sample, &normal_probe);
    if (have_normal)
        normal_score = v8_probe_chain_score(&normal_probe);

    inverted_samples = malloc((size_t) total_samples * sizeof(*inverted_samples));
    if (inverted_samples) {
        for (int i = 0; i < total_samples; i++)
            inverted_samples[i] = (int16_t) -samples[i];
        have_inverted = v8_select_best_probe_core(inverted_samples,
                                                  total_samples,
                                                  calling_party,
                                                  max_sample,
                                                  &inverted_probe);
        if (have_inverted)
            inverted_score = v8_probe_chain_score(&inverted_probe);
        free(inverted_samples);
    }

    if (!have_normal && !have_inverted)
        return false;
    if (have_inverted && (!have_normal || inverted_score > normal_score)) {
        *out = inverted_probe;
        return true;
    }

    *out = normal_probe;
    return true;
}

static bool v8_select_best_channel_probe(const int16_t *samples,
                                         int total_samples,
                                         int max_sample,
                                         v8_probe_result_t *out)
{
    v8_probe_result_t caller_probe;
    v8_probe_result_t answerer_probe;
    int caller_score = -1;
    int answerer_score = -1;
    bool have_caller;
    bool have_answerer;

    if (!samples || total_samples <= 0 || !out)
        return false;

    have_answerer = v8_select_best_probe(samples, total_samples, false, max_sample, &answerer_probe);
    have_caller = v8_select_best_probe(samples, total_samples, true, max_sample, &caller_probe);

    if (!have_answerer && !have_caller)
        return false;

    if (have_answerer)
        answerer_score = v8_probe_role_score(&answerer_probe);
    if (have_caller)
        caller_score = v8_probe_role_score(&caller_probe);

    if (have_answerer && (!have_caller || answerer_score >= caller_score)) {
        *out = answerer_probe;
        return true;
    }

    *out = caller_probe;
    return true;
}

static void print_v8_modulations(int mods)
{
    if (mods & V8_MOD_V17)    printf("  V.17");
    if (mods & V8_MOD_V21)    printf("  V.21");
    if (mods & V8_MOD_V22)    printf("  V.22bis");
    if (mods & V8_MOD_V23HDX) printf("  V.23hdx");
    if (mods & V8_MOD_V23)    printf("  V.23");
    if (mods & V8_MOD_V26BIS) printf("  V.26bis");
    if (mods & V8_MOD_V26TER) printf("  V.26ter");
    if (mods & V8_MOD_V27TER) printf("  V.27ter");
    if (mods & V8_MOD_V29)    printf("  V.29");
    if (mods & V8_MOD_V32)    printf("  V.32bis");
    if (mods & V8_MOD_V34HDX) printf("  V.34hdx");
    if (mods & V8_MOD_V34)    printf("  V.34");
    if (mods & V8_MOD_V90)    printf("  V.90");
    if (mods & V8_MOD_V92)    printf("  V.92");
    printf("\n");
}

static void print_v8_result(const v8_parms_t *r)
{
    printf("  Status:         %s\n", v8_status_to_str(r->status));
    printf("  Call function:  %s\n", v8_call_function_to_str(r->jm_cm.call_function));
    printf("  Modulations:   ");
    print_v8_modulations(r->jm_cm.modulations);
    printf("  Protocol:       %s\n", v8_protocol_to_str(r->jm_cm.protocols));
    printf("  PSTN access:    %s\n", v8_pstn_access_to_str(r->jm_cm.pstn_access));
    printf("  PCM modem:      %s\n",
           v8_pcm_modem_availability_to_str(r->jm_cm.pcm_modem_availability));
    if (r->jm_cm.nsf >= 0)
        printf("  NSF:            %s\n", v8_nsf_to_str(r->jm_cm.nsf));
    if (r->jm_cm.t66 >= 0)
        printf("  T.66:           %s\n", v8_t66_to_str(r->jm_cm.t66));
    if (r->v92 >= 0)
        printf("  V.92:           0x%02x\n", r->v92);
    if (r->gateway_mode)
        printf("  Gateway mode:   yes\n");
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

typedef struct {
    int total_bits;
    int preview_len;
    char preview[257];
} v34_aux_bit_collector_t;

static void v34_collect_aux_bit(void *user_data, int bit)
{
    v34_aux_bit_collector_t *collector = (v34_aux_bit_collector_t *) user_data;

    if (!collector)
        return;
    if (collector->preview_len < (int) sizeof(collector->preview) - 1)
        collector->preview[collector->preview_len++] = bit ? '1' : '0';
    collector->preview[collector->preview_len] = '\0';
    collector->total_bits++;
}

static void note_first_sample(int *dst, int sample)
{
    if (!dst || sample < 0)
        return;
    if (*dst < 0)
        *dst = sample;
}

static void v34_phase3_ja_bits_to_str(int trn16, char out[17])
{
    uint16_t pattern = 0x8990U;

    if (!out)
        return;
    if (trn16 > 0)
        pattern = 0x89B0U;

    for (int i = 0; i < 16; i++)
        out[i] = ((pattern >> (15 - i)) & 1U) ? '1' : '0';
    out[16] = '\0';
}

static void v34_phase3_repeat_ja_preview(int trn16, char *out, size_t out_len)
{
    char pattern[17];

    if (!out || out_len == 0)
        return;
    v34_phase3_ja_bits_to_str(trn16, pattern);
    if (out_len == 1) {
        out[0] = '\0';
        return;
    }
    for (size_t i = 0; i + 1 < out_len; i++)
        out[i] = pattern[i % 16];
    out[out_len - 1] = '\0';
}

static int samples_to_v34_symbols(int sample_count)
{
    if (sample_count <= 0)
        return 0;
    return (sample_count * 3200 + 4000) / 8000;
}

static int first_non_negative(int a, int b)
{
    if (a < 0)
        return b;
    if (b < 0)
        return a;
    return (a < b) ? a : b;
}

static int max_non_negative(int a, int b)
{
    if (a < 0)
        return b;
    if (b < 0)
        return a;
    return (a > b) ? a : b;
}

static int earliest_explicit_phase3_anchor_sample(const decode_v34_result_t *result)
{
    int anchor = -1;

    if (!result)
        return -1;
    anchor = first_non_negative(anchor, result->tx_first_s_sample);
    anchor = first_non_negative(anchor, result->tx_first_not_s_sample);
    anchor = first_non_negative(anchor, result->tx_md_sample);
    anchor = first_non_negative(anchor, result->tx_second_s_sample);
    anchor = first_non_negative(anchor, result->tx_second_not_s_sample);
    anchor = first_non_negative(anchor, result->tx_pp_sample);
    anchor = first_non_negative(anchor, result->tx_trn_sample);
    return anchor;
}

static void normalize_v34_result_to_spec_flow(decode_v34_result_t *result, bool calling_party)
{
    int phase3_anchor;

    if (!result)
        return;
    phase3_anchor = earliest_explicit_phase3_anchor_sample(result);
    if (phase3_anchor >= 0) {
        result->phase3_seen = true;
        result->phase3_sample = phase3_anchor;
    } else if (result->phase3_seen && result->info1_seen && result->phase3_sample < result->info1_sample) {
        result->phase3_sample = result->info1_sample;
    } else if (!result->phase3_seen && !calling_party && result->info1_seen && result->final_rx_stage >= 11) {
        result->phase3_seen = true;
        result->phase3_sample = result->info1_sample;
    }

    if (result->info1_seen && result->phase3_seen && result->phase3_sample >= 0
        && result->info1_sample > result->phase3_sample) {
        result->phase3_sample = result->info1_sample;
    }

    if (result->tx_pp_end_sample >= 0 && result->tx_pp_sample >= 0
        && result->tx_pp_end_sample < result->tx_pp_sample) {
        result->tx_pp_end_sample = result->tx_pp_sample;
    }
    if (result->tx_trn_sample >= 0)
        result->tx_trn_sample = max_non_negative(result->tx_trn_sample, result->tx_pp_end_sample);
    if (result->tx_ja_sample >= 0)
        result->tx_ja_sample = max_non_negative(result->tx_ja_sample, result->tx_trn_sample);
    if (result->tx_jdashed_sample >= 0)
        result->tx_jdashed_sample = max_non_negative(result->tx_jdashed_sample, result->tx_ja_sample);
    if (result->phase4_ready_sample >= 0)
        result->phase4_ready_sample = max_non_negative(result->phase4_ready_sample, result->rx_phase4_trn_sample);
    if (result->phase4_sample >= 0)
        result->phase4_sample = max_non_negative(result->phase4_sample, result->rx_phase4_s_sample);
}

static int v34_result_spec_score(const decode_v34_result_t *result)
{
    int score = 0;

    if (!result)
        return -1;
    if (result->tx_ja_sample >= 0)
        score += 4000;
    if (result->tx_trn_sample >= 0)
        score += 1500;
    if (result->tx_pp_sample >= 0)
        score += 800;
    if (result->tx_first_s_sample >= 0 || result->tx_first_not_s_sample >= 0)
        score += 400;
    if (result->phase4_seen)
        score += 1000;
    if (result->phase4_ready_seen)
        score += 300;
    if (result->rx_pp_started)
        score += 80;
    if (result->phase3_seen)
        score += 60;
    if (result->info1_seen)
        score += 20;
    if (result->info0_seen)
        score += 10;
    return score;
}

static bool should_emit_phase2_event(int sample, int latest_allowed_sample)
{
    if (sample < 0)
        return false;
    if (latest_allowed_sample < 0)
        return true;
    return sample <= latest_allowed_sample;
}

static const char *v34_info0_label(const decode_v34_result_t *result)
{
    return (result && result->info0_is_d) ? "INFO0d decoded" : "INFO0a decoded";
}

static const char *v34_info0_clock_or_law_label(const decode_v34_result_t *result)
{
    return (result && result->info0_is_d) ? "pcm_law" : "clock";
}

static const char *v34_info0_clock_or_law_value(const decode_v34_result_t *result)
{
    static const char *clock_values[] = { "0", "1", "2", "3" };

    if (!result)
        return "unknown";
    if (result->info0_is_d)
        return result->info0a.tx_clock_source ? "alaw" : "ulaw";
    if (result->info0a.tx_clock_source < (int) (sizeof(clock_values) / sizeof(clock_values[0])))
        return clock_values[result->info0a.tx_clock_source];
    return "unknown";
}

static int v34_effective_ja_bit_count(const decode_v34_result_t *result)
{
    int ja_end_sample = -1;

    if (!result || result->tx_ja_sample < 0)
        return 0;
    if (result->ja_observed_bits > 0)
        return result->ja_observed_bits;
    if (result->tx_jdashed_sample > result->tx_ja_sample)
        ja_end_sample = result->tx_jdashed_sample;
    else if (result->rx_s_event_sample > result->tx_ja_sample)
        ja_end_sample = result->rx_s_event_sample;
    else if (result->rx_phase4_s_sample > result->tx_ja_sample)
        ja_end_sample = result->rx_phase4_s_sample;
    else if (result->phase4_sample > result->tx_ja_sample)
        ja_end_sample = result->phase4_sample;
    else if (result->failure_sample > result->tx_ja_sample)
        ja_end_sample = result->failure_sample;
    if (ja_end_sample <= result->tx_ja_sample)
        return 0;
    return ((ja_end_sample - result->tx_ja_sample) * 3200 / 8000) * 2;
}

static void v34_effective_ja_preview(const decode_v34_result_t *result, char *out, size_t out_len)
{
    if (!out || out_len == 0) return;
    out[0] = '\0';
    if (!result) return;
    if (result->ja_bits_known && result->ja_bits[0] != '\0') {
        snprintf(out, out_len, "%s", result->ja_bits);
        return;
    }
    v34_phase3_repeat_ja_preview(result->ja_trn16, out, out_len);
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

static void decode_v8_pass(const int16_t *samples,
                           const uint8_t *codewords,
                           int total_samples,
                           v91_law_t law,
                           bool calling_party)
{
    v8_probe_result_t probe;
    v8bis_scan_result_t v8bis;
    v8bis_weak_candidate_t weak_v8bis;
    ans_fallback_hit_t ans_fallback;
    v8_fsk_burst_hit_t v21_burst;

    memset(&weak_v8bis, 0, sizeof(weak_v8bis));
    if (v8bis_scan_signals(samples, total_samples, V8_EARLY_SEARCH_LIMIT_SAMPLES, &v8bis)) {
        static const char *v8bis_desc[] = {
            "Modem Relay establishing (auto-answer)",
            "Call Relay establishing (auto-answer)",
            "Extension Signal initiating",
            "Modem Relay detected (initiating/calling-side)",
            "Call Relay detected (initiating/calling-side)",
            "Modem Relay detected (responding)",
            "Call Relay detected (responding)",
            "Extension Signal responding"
        };
        printf("  V.8bis signals detected:\n");
        for (size_t i = 0; i < sizeof(g_v8bis_signal_defs)/sizeof(g_v8bis_signal_defs[0]); i++) {
            if (!v8bis.hits[i].seen)
                continue;
            printf("    %s (%s) at %.1f ms\n",
                   g_v8bis_signal_defs[i].name,
                   i < sizeof(v8bis_desc)/sizeof(v8bis_desc[0]) ? v8bis_desc[i] : "",
                   sample_to_ms(v8bis.hits[i].sample_offset, 8000));
        }
    } else if (v8bis_scan_weak_candidate(samples, total_samples, V8_EARLY_SEARCH_LIMIT_SAMPLES, &weak_v8bis)) {
        const v8bis_signal_def_t *def = &g_v8bis_signal_defs[weak_v8bis.signal_index];

        printf("  Weak V.8bis:     %s-like energy at %.1f ms (score %.0f)\n",
               def->name,
               sample_to_ms(weak_v8bis.sample_offset, 8000),
               weak_v8bis.score);
    }

    if (v8_select_best_probe(samples, total_samples, calling_party, total_samples, &probe)) {
        printf("  Milestones:      ");
        if (probe.ansam_sample >= 0)
            printf("%s@%.1f ",
                   v8_answer_tone_name(probe.ansam_tone),
                   sample_to_ms(probe.ansam_sample, 8000));
        if (probe.ci_sample >= 0)
            printf("CI@%.1f ", sample_to_ms(probe.ci_sample, 8000));
        if (probe.cm_jm_sample >= 0)
            printf("%s%s@%.1f ",
                   calling_party ? "JM" : "CM",
                   probe.cm_jm_salvaged ? "?" : "",
                   sample_to_ms(probe.cm_jm_sample, 8000));
        if (probe.cj_sample >= 0)
            printf("CJ@%.1f ", sample_to_ms(probe.cj_sample, 8000));
        if (probe.v8_call_sample >= 0)
            printf("V8CALL@%.1f ", sample_to_ms(probe.v8_call_sample, 8000));
        printf("\n");
        if (probe.v8_call_sample >= 0 || probe.last_status == V8_STATUS_V8_CALL) {
            printf("  V.8 negotiation decoded at ~%.1f ms:\n",
                   sample_to_ms(probe.v8_call_sample >= 0 ? probe.v8_call_sample : probe.cm_jm_sample, 8000));
            print_v8_result(&probe.result);
        } else if (probe.ci_sample >= 0 || probe.cm_jm_sample >= 0 || probe.cj_sample >= 0) {
            printf("  Partial V.8 decode%s; final negotiation not yet confirmed\n",
                   probe.cm_jm_salvaged ? " (single CM/JM candidate salvage)" : "");
            if (probe.ansam_sample >= 0
                && probe.cm_jm_sample > probe.ansam_sample) {
                v8_raw_msg_hit_t pre_cm_candidate;

                memset(&pre_cm_candidate, 0, sizeof(pre_cm_candidate));
                if (v8_scan_v92_window_candidate(samples,
                                                 total_samples,
                                                 8000,
                                                 probe.ansam_sample + 12000,
                                                 probe.cm_jm_sample,
                                                 false,
                                                 0,
                                                 0,
                                                 &pre_cm_candidate)
                    && pre_cm_candidate.preamble_type == V8_LOCAL_SYNC_V92) {
                    v92_short_phase1_candidate_t pre_cm_v92c;

                    if (v92_decode_short_phase1_candidate(pre_cm_candidate.bit_run,
                                                          pre_cm_candidate.bit_len,
                                                          false,
                                                          &pre_cm_v92c)) {
                        printf("  Pre-CM V.92:     %s at %.1f-%.1f ms on 980/1180 Hz pair\n",
                               pre_cm_v92c.name,
                               sample_to_ms(pre_cm_candidate.sample_offset, 8000),
                               sample_to_ms(pre_cm_candidate.sample_offset + 70 * 8000 / 1000, 8000));
                    }
                }
            }
            if (probe.cm_jm_sample >= 0) {
                char hexbuf[256];

                format_hex_bytes(hexbuf, sizeof(hexbuf), probe.cm_jm_raw, probe.cm_jm_raw_len);
                printf("  Partial CM/JM fields at ~%.1f ms:\n",
                       sample_to_ms(probe.cm_jm_sample, 8000));
                printf("  Call function:  %s\n",
                       v8_call_function_to_str(probe.result.jm_cm.call_function));
                printf("  Modulations:   ");
                print_v8_modulations(probe.result.jm_cm.modulations);
                printf("  Protocol:       %s\n",
                       v8_protocol_to_str(probe.result.jm_cm.protocols));
                printf("  PSTN access:    %s\n",
                       v8_pstn_access_to_str(probe.result.jm_cm.pstn_access));
                printf("  PCM modem:      %s\n",
                       v8_pcm_modem_availability_to_str(probe.result.jm_cm.pcm_modem_availability));
                if (probe.result.jm_cm.nsf >= 0)
                    printf("  NSF:            %s\n", v8_nsf_to_str(probe.result.jm_cm.nsf));
                if (probe.result.jm_cm.t66 >= 0)
                    printf("  T.66:           %s\n", v8_t66_to_str(probe.result.jm_cm.t66));
                if (hexbuf[0] != '\0')
                    printf("  Raw bytes:      %s\n", hexbuf);
            }
        } else {
            printf("  Tone-level early negotiation detected, but no valid CI/CM/JM/CJ sequence yet\n");
            printf("  Possible V.8bis or clipped pre-V.8 exchange\n");
            if (probe.ansam_sample >= 0) {
                v8_raw_msg_hit_t post_ans_candidate;

                memset(&post_ans_candidate, 0, sizeof(post_ans_candidate));
                if (v8_scan_v92_window_candidate(samples,
                                                 total_samples,
                                                 8000,
                                                 probe.ansam_sample + 12000,
                                                 total_samples,
                                                 !calling_party,
                                                 !calling_party ? 1 : 0,
                                                 -1,
                                                 &post_ans_candidate)
                    && post_ans_candidate.preamble_type == V8_LOCAL_SYNC_V92) {
                    v92_short_phase1_candidate_t post_ans_v92c;

                    if (v92_decode_short_phase1_candidate(post_ans_candidate.bit_run,
                                                          post_ans_candidate.bit_len,
                                                          !calling_party,
                                                          &post_ans_v92c)) {
                        printf("  Post-ANSam V.92: %s at %.1f-%.1f ms on %s pair\n",
                               post_ans_v92c.name,
                               sample_to_ms(post_ans_candidate.sample_offset, 8000),
                               sample_to_ms(post_ans_candidate.sample_offset + 560, 8000),
                               calling_party ? "980/1180 Hz" : "1650/1850 Hz");
                    }
                }
            }
            if (probe.ansam_sample >= 0
                && probe.cm_jm_sample < 0
                && v8_detect_v21_burst(samples,
                                       total_samples,
                                       8000,
                                       probe.ansam_sample + 12000,
                                       total_samples,
                                       !calling_party,
                                       &v21_burst)) {
                v8_raw_msg_hit_t raw_candidate;
                char hexbuf[256];

                memset(&raw_candidate, 0, sizeof(raw_candidate));
                hexbuf[0] = '\0';
                if (v8_targeted_v21_bytes_candidate(samples,
                                                    total_samples,
                                                    8000,
                                                    &v21_burst,
                                                    !calling_party,
                                                    &raw_candidate)) {
                    format_hex_bytes(hexbuf, sizeof(hexbuf), raw_candidate.bytes, raw_candidate.byte_len);
                }
                if (raw_candidate.preamble_type == V8_LOCAL_SYNC_V92) {
                    v92_short_phase1_candidate_t v92c;
                    v92_qts_hit_t qts_hit;

                    if (v92_decode_short_phase1_candidate(raw_candidate.bit_run, raw_candidate.bit_len, !calling_party, &v92c)) {
                        char ones1[16], sync1[16], frame1[16], ones2[16], sync2[16], frame2[16], tail[16];

                        printf("  %s candidate at %.1f-%.1f ms on %s pair (peak %.1f%%)\n",
                               v92c.name,
                               sample_to_ms(v21_burst.start_sample, 8000),
                               sample_to_ms(v21_burst.start_sample + v21_burst.duration_samples, 8000),
                               calling_party ? "980/1180 Hz" : "1650/1850 Hz",
                               v21_burst.peak_strength * 100.0);
                        printf("  Lock detail:     %s\n",
                               v8_preamble_detail(raw_candidate.preamble_type));
                        if (v92c.decoded_frame_index > 0)
                            printf("  Decoded frame:   %03X (selected from repeated frame)\n", v92c.decoded_frame_bits);
                        else
                            printf("  Decoded frame:   %03X\n", v92c.decoded_frame_bits);
                        if (v92c.digital_modem) {
                            printf("  ANSpcm level:    %s\n",
                                   v92_anspcm_level_to_str(v92c.aux_value));
                        } else {
                            printf("  UQTS index:      0x%X (Ucode %d)\n", v92c.aux_value, v92c.uqts_ucode);
                            if (codewords
                                && v92c.uqts_ucode >= 0
                            && v92_detect_qts_sequence(codewords,
                                                       total_samples,
                                                       law,
                                                       v92c.uqts_ucode,
                                                       v21_burst.start_sample + v21_burst.duration_samples + 400,
                                                       v21_burst.start_sample + v21_burst.duration_samples + 4000,
                                                       codeword_to_ucode,
                                                       &qts_hit)) {
                                printf("  QTS candidate:   %.1f ms (%d QTS reps, %d QTS\\\\ reps)\n",
                                       sample_to_ms(qts_hit.start_sample, 8000),
                                       qts_hit.qts_reps,
                                       qts_hit.qts_bar_reps);
                            }
                        }
                        if (v92c.repeat_seen) {
                            printf("  Repeat frame:    %03X (%s)\n",
                                   v92c.frame2_bits,
                                   v92c.repeat_match ? "matches" : "differs");
                        }
                        if (raw_candidate.bit_len >= 60) {
                            format_bit_slice(ones1, sizeof(ones1), raw_candidate.bit_run, 0, 10);
                            format_bit_slice(sync1, sizeof(sync1), raw_candidate.bit_run, 10, 10);
                            format_bit_slice(frame1, sizeof(frame1), raw_candidate.bit_run, 20, 10);
                            format_bit_slice(ones2, sizeof(ones2), raw_candidate.bit_run, 30, 10);
                            format_bit_slice(sync2, sizeof(sync2), raw_candidate.bit_run, 40, 10);
                            format_bit_slice(frame2, sizeof(frame2), raw_candidate.bit_run, 50, 10);
                            if (raw_candidate.bit_len >= 70)
                                format_bit_slice(tail, sizeof(tail), raw_candidate.bit_run, 60, 10);
                            else
                                format_bit_slice(tail, sizeof(tail), raw_candidate.bit_run, 60, raw_candidate.bit_len - 60);
                            printf("  Bit run:         %s | %s | %s | %s | %s | %s | %s\n",
                                   ones1, sync1, frame1, ones2, sync2, frame2, tail);
                        }
                    } else {
                        printf("  %s V.21 burst candidate at %.1f-%.1f ms on %s pair (peak %.1f%%)\n",
                               v8_preamble_to_candidate_label(raw_candidate.preamble_type, calling_party),
                               sample_to_ms(v21_burst.start_sample, 8000),
                               sample_to_ms(v21_burst.start_sample + v21_burst.duration_samples, 8000),
                               calling_party ? "980/1180 Hz" : "1650/1850 Hz",
                               v21_burst.peak_strength * 100.0);
                        printf("  Lock detail:     %s\n",
                               v8_preamble_detail(raw_candidate.preamble_type));
                    }
                } else {
                    printf("  %s V.21 burst candidate at %.1f-%.1f ms on %s pair (peak %.1f%%)\n",
                           v8_preamble_to_candidate_label(raw_candidate.preamble_type, calling_party),
                           sample_to_ms(v21_burst.start_sample, 8000),
                           sample_to_ms(v21_burst.start_sample + v21_burst.duration_samples, 8000),
                           calling_party ? "980/1180 Hz" : "1650/1850 Hz",
                           v21_burst.peak_strength * 100.0);
                    printf("  Lock detail:     %s\n",
                           v8_preamble_detail(raw_candidate.preamble_type));
                }
                if (hexbuf[0] != '\0')
                    printf("  Candidate bytes: %s\n", hexbuf);
            }
        }
    } else {
        printf("  No V.8 negotiation detected in %d samples (%.1f ms)\n",
               total_samples, sample_to_ms(total_samples, 8000));
        if (detect_standalone_ans_fallback(samples, total_samples, 8000, total_samples, &ans_fallback)) {
            printf("  Standalone %s candidate at %.1f-%.1f ms (peak %.1f%%)\n",
                   v8_answer_tone_name(ans_fallback.tone_type),
                   sample_to_ms(ans_fallback.start_sample, 8000),
                   sample_to_ms(ans_fallback.start_sample + ans_fallback.duration_samples, 8000),
                   ans_fallback.peak_ratio * 100.0);
        }
    }
}

static void collect_v8_event(call_log_t *log,
                             const int16_t *samples,
                             const uint8_t *codewords,
                             int total_samples,
                             int max_sample,
                             v91_law_t law)
{
    v8_probe_result_t probe;
    ans_fallback_hit_t ans_fallback;
    v8_fsk_burst_hit_t v21_burst;
    char summary[160];
    char detail[320];

    if (!log || !samples || total_samples <= 0)
        return;

    if (!v8_select_best_channel_probe(samples, total_samples, max_sample, &probe))
        goto fallback_only;
    if (probe.ansam_sample >= 0) {
        snprintf(summary, sizeof(summary),
                 "%s detected",
                 v8_answer_tone_name(probe.ansam_tone));
        snprintf(detail, sizeof(detail),
                 "role=%s tone=%s",
                 probe.calling_party ? "caller" : "answerer",
                 v8_answer_tone_detail(probe.ansam_tone));
        call_log_append(log, probe.ansam_sample, 0, "V.8", summary, detail);
    }
    if (probe.ci_sample >= 0) {
        snprintf(detail, sizeof(detail),
                 "role=%s call_function=%s",
                 probe.calling_party ? "caller" : "answerer",
                 v8_call_function_to_str(probe.result.jm_cm.call_function));
        call_log_append(log, probe.ci_sample, 0, "V.8", "CI decoded", detail);
    }
    if (probe.cm_jm_sample >= 0) {
        char hexbuf[256];

        format_hex_bytes(hexbuf, sizeof(hexbuf), probe.cm_jm_raw, probe.cm_jm_raw_len);
        snprintf(summary, sizeof(summary),
                 "%s%s decoded",
                 probe.calling_party ? "JM" : "CM",
                 probe.cm_jm_salvaged ? "?" : "");
        snprintf(detail, sizeof(detail),
                 "role=%s call_fn=%s protocol=%s pcm=%s pstn=%s confidence=%s%s%s",
                 probe.calling_party ? "caller" : "answerer",
                 v8_call_function_to_str(probe.result.jm_cm.call_function),
                 v8_protocol_to_str(probe.result.jm_cm.protocols),
                 v8_pcm_modem_availability_to_str(probe.result.jm_cm.pcm_modem_availability),
                 v8_pstn_access_to_str(probe.result.jm_cm.pstn_access),
                 probe.cm_jm_salvaged ? "salvaged_single_message" : "confirmed",
                 hexbuf[0] != '\0' ? " raw=" : "",
                 hexbuf[0] != '\0' ? hexbuf : "");
        call_log_append(log, probe.cm_jm_sample, 0, "V.8", summary, detail);
    }
    if (probe.ansam_sample >= 0
        && probe.cm_jm_sample > probe.ansam_sample) {
        v8_raw_msg_hit_t pre_cm_candidate;

        memset(&pre_cm_candidate, 0, sizeof(pre_cm_candidate));
        if (v8_scan_v92_window_candidate(samples,
                                         total_samples,
                                         8000,
                                         probe.ansam_sample + 12000,
                                         probe.cm_jm_sample,
                                         false,
                                         0,
                                         0,
                                         &pre_cm_candidate)
            && pre_cm_candidate.preamble_type == V8_LOCAL_SYNC_V92) {
            v92_short_phase1_candidate_t pre_cm_v92c;

            if (v92_decode_short_phase1_candidate(pre_cm_candidate.bit_run,
                                                  pre_cm_candidate.bit_len,
                                                  false,
                                                  &pre_cm_v92c)) {
                snprintf(summary, sizeof(summary), "Pre-CM %s", pre_cm_v92c.name);
                snprintf(detail, sizeof(detail),
                         "role=%s start=%.1f ms duration=%.1f ms pair=980/1180 Hz pre_cm=yes",
                         probe.calling_party ? "caller" : "answerer",
                         sample_to_ms(pre_cm_candidate.sample_offset, 8000),
                         70.0);
                call_log_append(log, pre_cm_candidate.sample_offset, 560, "V.92", summary, detail);
            }
        }
    }
    if (probe.cj_sample >= 0) {
        snprintf(detail, sizeof(detail),
                 "role=%s",
                 probe.calling_party ? "caller" : "answerer");
        call_log_append(log, probe.cj_sample, 0, "V.8", "CJ decoded", detail);
    }
    if (probe.ansam_sample >= 0
        && probe.ci_sample < 0
        && probe.cm_jm_sample < 0
        && probe.cj_sample < 0
        && probe.v8_call_sample < 0) {
        v8_raw_msg_hit_t post_ans_candidate;

        snprintf(detail, sizeof(detail),
                 "role=%s early negotiation energy without CI/CM/JM/CJ",
                 probe.calling_party ? "caller" : "answerer");
        call_log_append(log,
                        probe.ansam_sample,
                        0,
                        "V.8bis?",
                        "Possible V.8bis / pre-V.8 negotiation",
                        detail);
        memset(&post_ans_candidate, 0, sizeof(post_ans_candidate));
        if (v8_scan_v92_window_candidate(samples,
                                         total_samples,
                                         8000,
                                         probe.ansam_sample + 12000,
                                         max_sample > 0 ? max_sample : total_samples,
                                         !probe.calling_party,
                                         !probe.calling_party ? 1 : 0,
                                         -1,
                                         &post_ans_candidate)
            && post_ans_candidate.preamble_type == V8_LOCAL_SYNC_V92) {
            v92_short_phase1_candidate_t post_ans_v92c;

            if (v92_decode_short_phase1_candidate(post_ans_candidate.bit_run,
                                                  post_ans_candidate.bit_len,
                                                  !probe.calling_party,
                                                  &post_ans_v92c)) {
                snprintf(summary, sizeof(summary), "Post-ANSam %s", post_ans_v92c.name);
                snprintf(detail, sizeof(detail),
                         "role=%s start=%.1f ms duration=%.1f ms pair=%s post_ansam=yes",
                         probe.calling_party ? "caller" : "answerer",
                         sample_to_ms(post_ans_candidate.sample_offset, 8000),
                         70.0,
                         probe.calling_party ? "980/1180 Hz" : "1650/1850 Hz");
                call_log_append(log, post_ans_candidate.sample_offset, 560, "V.92", summary, detail);
            }
        }
    }
    if (probe.last_status == V8_STATUS_V8_CALL && probe.v8_call_sample >= 0) {
        snprintf(summary, sizeof(summary),
                 "V.8 negotiation decoded as %s",
                 probe.calling_party ? "caller" : "answerer");
        snprintf(detail, sizeof(detail),
                 "status=%s protocol=%s pcm=%s pstn=%s",
                 v8_status_to_str(probe.result.status),
                 v8_protocol_to_str(probe.result.jm_cm.protocols),
                 v8_pcm_modem_availability_to_str(probe.result.jm_cm.pcm_modem_availability),
                 v8_pstn_access_to_str(probe.result.jm_cm.pstn_access));
        call_log_append(log, probe.v8_call_sample, 0, "V.8", summary, detail);
    }
    if (probe.cm_jm_sample < 0
        && probe.ansam_sample >= 0
        && v8_detect_v21_burst(samples,
                               total_samples,
                               8000,
                               probe.ansam_sample + 12000,
                               max_sample > 0 ? max_sample : total_samples,
                               !probe.calling_party,
                               &v21_burst)) {
        v8_raw_msg_hit_t raw_candidate;
        char hexbuf[256];

        memset(&raw_candidate, 0, sizeof(raw_candidate));
        hexbuf[0] = '\0';
        if (v8_targeted_v21_bytes_candidate(samples,
                                            total_samples,
                                            8000,
                                            &v21_burst,
                                            !probe.calling_party,
                                            &raw_candidate)) {
            format_hex_bytes(hexbuf, sizeof(hexbuf), raw_candidate.bytes, raw_candidate.byte_len);
        }
        if (raw_candidate.preamble_type == V8_LOCAL_SYNC_V92) {
            v92_short_phase1_candidate_t v92c;
            v92_qts_hit_t qts_hit;

            if (v92_decode_short_phase1_candidate(raw_candidate.bit_run, raw_candidate.bit_len, !probe.calling_party, &v92c)) {
                char auxbuf[64];
                char bitbuf[128];
                char framebuf[64];
                char qtsbuf[96];

                if (v92c.digital_modem)
                    snprintf(auxbuf, sizeof(auxbuf), "anspcm_level=%s", v92_anspcm_level_to_str(v92c.aux_value));
                else
                    snprintf(auxbuf, sizeof(auxbuf), "uqts_index=0x%X uqts_ucode=%d", v92c.aux_value, v92c.uqts_ucode);
                if (v92c.decoded_frame_index > 0)
                    snprintf(framebuf, sizeof(framebuf), " decoded=%03X/repeat", v92c.decoded_frame_bits);
                else
                    snprintf(framebuf, sizeof(framebuf), " decoded=%03X", v92c.decoded_frame_bits);
                bitbuf[0] = '\0';
                qtsbuf[0] = '\0';
                if (raw_candidate.bit_len >= 60) {
                    char ones1[16], sync1[16], frame1[16], ones2[16], sync2[16], frame2[16], tail[16];

                    format_bit_slice(ones1, sizeof(ones1), raw_candidate.bit_run, 0, 10);
                    format_bit_slice(sync1, sizeof(sync1), raw_candidate.bit_run, 10, 10);
                    format_bit_slice(frame1, sizeof(frame1), raw_candidate.bit_run, 20, 10);
                    format_bit_slice(ones2, sizeof(ones2), raw_candidate.bit_run, 30, 10);
                    format_bit_slice(sync2, sizeof(sync2), raw_candidate.bit_run, 40, 10);
                    format_bit_slice(frame2, sizeof(frame2), raw_candidate.bit_run, 50, 10);
                    if (raw_candidate.bit_len >= 70)
                        format_bit_slice(tail, sizeof(tail), raw_candidate.bit_run, 60, 10);
                    else
                        format_bit_slice(tail, sizeof(tail), raw_candidate.bit_run, 60, raw_candidate.bit_len - 60);
                    snprintf(bitbuf, sizeof(bitbuf),
                             " bits=%s|%s|%s|%s|%s|%s|%s",
                             ones1, sync1, frame1, ones2, sync2, frame2, tail);
                }
                if (!v92c.digital_modem
                    && codewords
                    && v92c.uqts_ucode >= 0
                    && v92_detect_qts_sequence(codewords,
                                               total_samples,
                                               law,
                                               v92c.uqts_ucode,
                                               v21_burst.start_sample + v21_burst.duration_samples + 400,
                                               v21_burst.start_sample + v21_burst.duration_samples + 4000,
                                               codeword_to_ucode,
                                               &qts_hit)) {
                    snprintf(qtsbuf, sizeof(qtsbuf),
                             " qts=%.1fms/%dreps qts_bar=%dreps",
                             sample_to_ms(qts_hit.start_sample, 8000),
                             qts_hit.qts_reps,
                             qts_hit.qts_bar_reps);
                }
                snprintf(summary, sizeof(summary),
                         "%s candidate",
                         v92c.name);
                snprintf(detail, sizeof(detail),
                         "role=%s start=%.1f ms duration=%.1f ms pair=%s peak=%.1f%% lock=\"%s\" lapm=%s %s%s repeat=%s%s%s%s%s",
                         probe.calling_party ? "caller" : "answerer",
                         sample_to_ms(v21_burst.start_sample, 8000),
                         sample_to_ms(v21_burst.duration_samples, 8000),
                         probe.calling_party ? "980/1180 Hz" : "1650/1850 Hz",
                         v21_burst.peak_strength * 100.0,
                         v8_preamble_detail(raw_candidate.preamble_type),
                         v92c.lapm ? "yes" : "no",
                         auxbuf,
                         framebuf,
                         v92c.repeat_seen ? (v92c.repeat_match ? "match" : "differs") : "missing",
                         qtsbuf,
                         bitbuf,
                         hexbuf[0] != '\0' ? " raw=" : "",
                         hexbuf[0] != '\0' ? hexbuf : "");
            } else {
                snprintf(summary, sizeof(summary),
                         "%s V.21 burst candidate",
                         v8_preamble_to_candidate_label(raw_candidate.preamble_type, probe.calling_party));
                snprintf(detail, sizeof(detail),
                         "role=%s start=%.1f ms duration=%.1f ms pair=%s peak=%.1f%% lock=\"%s\"%s%s",
                         probe.calling_party ? "caller" : "answerer",
                         sample_to_ms(v21_burst.start_sample, 8000),
                         sample_to_ms(v21_burst.duration_samples, 8000),
                         probe.calling_party ? "980/1180 Hz" : "1650/1850 Hz",
                         v21_burst.peak_strength * 100.0,
                         v8_preamble_detail(raw_candidate.preamble_type),
                         hexbuf[0] != '\0' ? " raw=" : "",
                         hexbuf[0] != '\0' ? hexbuf : "");
            }
        } else {
            snprintf(summary, sizeof(summary),
                     "%s V.21 burst candidate",
                     v8_preamble_to_candidate_label(raw_candidate.preamble_type, probe.calling_party));
            snprintf(detail, sizeof(detail),
                     "role=%s start=%.1f ms duration=%.1f ms pair=%s peak=%.1f%% lock=\"%s\"%s%s",
                     probe.calling_party ? "caller" : "answerer",
                     sample_to_ms(v21_burst.start_sample, 8000),
                     sample_to_ms(v21_burst.duration_samples, 8000),
                     probe.calling_party ? "980/1180 Hz" : "1650/1850 Hz",
                     v21_burst.peak_strength * 100.0,
                     v8_preamble_detail(raw_candidate.preamble_type),
                     hexbuf[0] != '\0' ? " raw=" : "",
                     hexbuf[0] != '\0' ? hexbuf : "");
        }
        call_log_append(log,
                        v21_burst.start_sample,
                        v21_burst.duration_samples,
                        "V.8?",
                        summary,
                        detail);
    }
    if (probe.ansam_sample < 0
        && detect_standalone_ans_fallback(samples, total_samples, 8000, max_sample, &ans_fallback)) {
        snprintf(detail, sizeof(detail),
                 "duration=%.1f ms peak_2100=%.1f%% tone=%s",
                 sample_to_ms(ans_fallback.duration_samples, 8000),
                 ans_fallback.peak_ratio * 100.0,
                 v8_answer_tone_detail(ans_fallback.tone_type));
        call_log_append(log,
                        ans_fallback.start_sample,
                        ans_fallback.duration_samples,
                        "V.8?",
                        "Standalone answer tone candidate",
                        detail);
    }
    return;

fallback_only:
    if (detect_standalone_ans_fallback(samples, total_samples, 8000, max_sample, &ans_fallback)) {
        snprintf(detail, sizeof(detail),
                 "duration=%.1f ms peak_2100=%.1f%% tone=%s",
                 sample_to_ms(ans_fallback.duration_samples, 8000),
                 ans_fallback.peak_ratio * 100.0,
                 v8_answer_tone_detail(ans_fallback.tone_type));
        call_log_append(log,
                        ans_fallback.start_sample,
                        ans_fallback.duration_samples,
                        "V.8?",
                        "Standalone answer tone candidate",
                        detail);
    }
}

/* V.8bis HDLC message decoder, signal/event collectors, and helpers
 * have been moved to v8bis_decode.c / v8bis_decode.h */


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
    v34_aux_bit_collector_t aux_bits;
    int offset = 0;
    int prev_rx_stage = -1;
    int prev_tx_stage = -1;
    int prev_rx_event = -1;
    int ja_aux_start_bit = -1;

    if (!samples || total_samples <= 0 || !result)
        return false;

    memset(result, 0, sizeof(*result));
    result->info0_sample = -1;
    result->info1_sample = -1;
    result->phase3_sample = -1;
    result->tx_first_s_sample = -1;
    result->tx_first_not_s_sample = -1;
    result->tx_md_sample = -1;
    result->tx_second_s_sample = -1;
    result->tx_second_not_s_sample = -1;
    result->tx_pp_sample = -1;
    result->tx_pp_end_sample = -1;
    result->rx_pp_detect_sample = -1;
    result->rx_pp_complete_sample = -1;
    result->rx_pp_phase = -1;
    result->rx_pp_phase_score = -1;
    result->rx_pp_acquire_hits = 0;
    result->rx_pp_started = false;
    result->tx_trn_sample = -1;
    result->phase3_trn_lock_sample = -1;
    result->phase3_trn_strong_sample = -1;
    result->phase3_trn_lock_score = -1;
    result->tx_ja_sample = -1;
    result->tx_jdashed_sample = -1;
    result->ja_trn16 = -1;
    result->ja_detector_bits = -1;
    result->ja_observed_bits = 0;
    result->ja_bits[0] = '\0';
    result->rx_s_event_sample = -1;
    result->rx_phase4_s_sample = -1;
    result->rx_phase4_sbar_sample = -1;
    result->rx_phase4_trn_sample = -1;
    result->phase4_ready_sample = -1;
    result->phase4_sample = -1;
    result->failure_sample = -1;

    v34 = v34_init(NULL, 3200, 21600, calling_party, true,
                   v34_dummy_get_bit, NULL,
                   v34_dummy_put_bit, NULL);
    if (!v34)
        return false;

    v34_set_v90_mode(v34, law == V91_LAW_ALAW ? 1 : 0);
    memset(&aux_bits, 0, sizeof(aux_bits));
    v34_set_put_aux_bit(v34, v34_collect_aux_bit, &aux_bits);
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

        {
            int trn_score = v34_get_phase3_trn_lock_score(v34);

            if (trn_score > result->phase3_trn_lock_score) {
                result->phase3_trn_lock_score = trn_score;
                if (trn_score >= 0)
                    note_first_sample(&result->phase3_trn_lock_sample, offset);
                if (trn_score >= 70)
                    note_first_sample(&result->phase3_trn_strong_sample, offset);
            }
        }

        if (v34->rx.phase3_pp_phase >= 0) {
            if (result->rx_pp_detect_sample < 0)
                result->rx_pp_detect_sample = offset;
            result->rx_pp_phase = v34->rx.phase3_pp_phase;
            result->rx_pp_phase_score = v34->rx.phase3_pp_phase_score;
            result->rx_pp_acquire_hits = v34->rx.phase3_pp_acquire_hits;
        }
        if (v34->rx.phase3_pp_started) {
            result->rx_pp_started = true;
            if (result->rx_pp_complete_sample < 0)
                result->rx_pp_complete_sample = offset;
        }

        if (!result->ja_bits_known && v34_get_phase3_j_trn16(v34) >= 0) {
        result->ja_trn16 = v34_get_phase3_j_trn16(v34);
        result->ja_detector_bits = v34_get_phase3_j_bits(v34);
        v34_phase3_ja_bits_to_str(result->ja_trn16, result->ja_bits);
        result->ja_bits_known = true;
        result->ja_bits_from_local_tx = false;
    } else if (result->ja_bits_known) {
        result->ja_detector_bits = v34_get_phase3_j_bits(v34);
    }

        if (tx_stage != prev_tx_stage) {
            switch (tx_stage) {
            case 33:
                note_first_sample(&result->tx_first_s_sample, offset);
                break;
            case 34:
                note_first_sample(&result->tx_first_not_s_sample, offset);
                break;
            case 35:
                note_first_sample(&result->tx_md_sample, offset);
                break;
            case 36:
                note_first_sample(&result->tx_second_s_sample, offset);
                break;
            case 37:
                note_first_sample(&result->tx_second_not_s_sample, offset);
                note_first_sample(&result->tx_pp_sample, offset);
                break;
            case 38:
                if (result->tx_pp_sample >= 0 && result->tx_pp_end_sample < 0)
                    result->tx_pp_end_sample = offset;
                note_first_sample(&result->tx_trn_sample, offset);
                break;
            case 39:
                note_first_sample(&result->tx_ja_sample, offset);
                if (ja_aux_start_bit < 0)
                    ja_aux_start_bit = aux_bits.total_bits;
                if (!result->ja_bits_known) {
                    /* In this SpanDSP checkout, local Phase 3 TX uses 4-point TRN/J. */
                    result->ja_trn16 = 0;
                    result->ja_detector_bits = v34_get_phase3_j_bits(v34);
                    v34_phase3_ja_bits_to_str(result->ja_trn16, result->ja_bits);
                    result->ja_bits_known = true;
                    result->ja_bits_from_local_tx = true;
                }
                break;
            case 40:
                note_first_sample(&result->tx_jdashed_sample, offset);
                break;
            default:
                break;
            }
            prev_tx_stage = tx_stage;
        }

        if (rx_stage != prev_rx_stage) {
            switch (rx_stage) {
            case 13:
                note_first_sample(&result->rx_phase4_s_sample, offset);
                break;
            case 14:
                note_first_sample(&result->rx_phase4_sbar_sample, offset);
                break;
            case 15:
                note_first_sample(&result->rx_phase4_trn_sample, offset);
                break;
            default:
                break;
            }
            prev_rx_stage = rx_stage;
        }

        if (rx_event != prev_rx_event) {
            if (rx_event == 12)
                note_first_sample(&result->rx_s_event_sample, offset);
            prev_rx_event = rx_event;
        }

        if (!result->info0_seen
            && v34_get_v90_received_info0a(v34, &raw_info0a) > 0
            && map_v34_received_info0a(&result->info0a, &raw_info0a)) {
            result->info0_seen = true;
            result->info0_is_d = calling_party;
            result->info0_sample = offset;
        }
        if (!result->info1_seen
            && v34_get_v90_received_info1a(v34, &raw_info1a) > 0
            && map_v34_received_info1a(&result->info1a, &raw_info1a)) {
            result->info1_seen = true;
            result->info1_sample = offset;
            result->u_info = result->info1a.u_info;
            result->u_info_from_info1a = true;
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

    if (ja_aux_start_bit >= 0
        && ja_aux_start_bit < aux_bits.preview_len
        && aux_bits.total_bits > ja_aux_start_bit) {
        int available = aux_bits.total_bits - ja_aux_start_bit;
        int stored = aux_bits.preview_len - ja_aux_start_bit;
        int copy_len = stored;

        if (copy_len > (int) sizeof(result->ja_bits) - 1)
            copy_len = (int) sizeof(result->ja_bits) - 1;
        memcpy(result->ja_bits, aux_bits.preview + ja_aux_start_bit, (size_t) copy_len);
        result->ja_bits[copy_len] = '\0';
        result->ja_observed_bits = available;
        result->ja_bits_known = true;
    }
    if (result->tx_ja_sample >= 0 && result->ja_observed_bits == 0) {
        int ja_end_sample = -1;

        if (result->tx_jdashed_sample > result->tx_ja_sample)
            ja_end_sample = result->tx_jdashed_sample;
        else if (result->rx_s_event_sample > result->tx_ja_sample)
            ja_end_sample = result->rx_s_event_sample;
        else if (result->rx_phase4_s_sample > result->tx_ja_sample)
            ja_end_sample = result->rx_phase4_s_sample;
        else if (result->phase4_sample > result->tx_ja_sample)
            ja_end_sample = result->phase4_sample;
        else if (result->failure_sample > result->tx_ja_sample)
            ja_end_sample = result->failure_sample;
        else
            ja_end_sample = total_samples;

        if (ja_end_sample > result->tx_ja_sample) {
            int ja_symbols = (ja_end_sample - result->tx_ja_sample) * 3200 / 8000;
            result->ja_observed_bits = ja_symbols * 2;
            v34_phase3_repeat_ja_preview(result->ja_trn16, result->ja_bits, sizeof(result->ja_bits));
            result->ja_bits_known = true;
            result->ja_bits_estimated = true;
        }
    }

    normalize_v34_result_to_spec_flow(result, calling_party);

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
        printf("  %-16sdecoded at %.1f ms\n",
               result->info0_is_d ? "INFO0d:" : "INFO0a:",
               sample_to_ms(result->info0_sample, 8000));
        printf("                   3429=%s 1664pt=%s %s=%s ack_info0d=%s\n",
               result->info0a.support_3429 ? "yes" : "no",
               result->info0a.support_1664_point_constellation ? "yes" : "no",
               v34_info0_clock_or_law_label(result),
               v34_info0_clock_or_law_value(result),
               result->info0a.acknowledge_info0d ? "yes" : "no");
    } else {
        printf("  INFO0a/INFO0d:   not decoded\n");
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
    if (result->u_info > 0) {
        printf("  U_INFO source:   %s (%d)\n",
               result->u_info_from_info1a ? "INFO1a bits 25:31" : "SpanDSP fallback state",
               result->u_info);
    }
    if (result->phase3_seen)
        printf("  Phase 3:         seen at %.1f ms\n", sample_to_ms(result->phase3_sample, 8000));
    if (result->tx_first_s_sample >= 0
        || result->tx_first_not_s_sample >= 0
        || result->tx_second_s_sample >= 0
        || result->tx_trn_sample >= 0
        || result->tx_ja_sample >= 0) {
        printf("  Phase 3 TX:      ");
        if (result->tx_first_s_sample >= 0)
            printf("S@%.1f ",
                   sample_to_ms(result->tx_first_s_sample, 8000));
        if (result->tx_first_not_s_sample >= 0)
            printf("S-bar@%.1f ",
                   sample_to_ms(result->tx_first_not_s_sample, 8000));
        if (result->tx_md_sample >= 0)
            printf("MD@%.1f ",
                   sample_to_ms(result->tx_md_sample, 8000));
        if (result->tx_second_s_sample >= 0)
            printf("S2@%.1f ",
                   sample_to_ms(result->tx_second_s_sample, 8000));
        if (result->tx_second_not_s_sample >= 0)
            printf("S2-bar/PP@%.1f ",
                   sample_to_ms(result->tx_second_not_s_sample, 8000));
        if (result->tx_trn_sample >= 0)
            printf("TRN@%.1f ",
                   sample_to_ms(result->tx_trn_sample, 8000));
        if (result->tx_ja_sample >= 0)
            printf("J/Ja@%.1f ",
                   sample_to_ms(result->tx_ja_sample, 8000));
        if (result->tx_jdashed_sample >= 0)
            printf("J'@%.1f ",
                   sample_to_ms(result->tx_jdashed_sample, 8000));
        printf("\n");
    }
    if (result->tx_pp_sample >= 0) {
        int pp_end = (result->tx_pp_end_sample >= 0) ? result->tx_pp_end_sample : result->tx_trn_sample;
        int pp_samples = (pp_end > result->tx_pp_sample) ? (pp_end - result->tx_pp_sample) : 0;
        int pp_symbols = samples_to_v34_symbols(pp_samples);
        printf("  Phase 3 PP:      start@%.1f",
               sample_to_ms(result->tx_pp_sample, 8000));
        if (pp_end > result->tx_pp_sample) {
            printf(" end@%.1f (~%d symbols, %.1f ms)",
                   sample_to_ms(pp_end, 8000),
                   pp_symbols,
                   sample_to_ms(pp_samples, 8000));
        } else {
            printf(" end@unknown");
        }
        printf("\n");
    }
    if (result->rx_pp_detect_sample >= 0) {
        printf("  Phase 3 PP RX:   detect@%.1f phase=%d score=%d hold=%d%s\n",
               sample_to_ms(result->rx_pp_detect_sample, 8000),
               result->rx_pp_phase,
               result->rx_pp_phase_score,
               result->rx_pp_acquire_hits,
               result->rx_pp_started ? " started=yes" : "");
    }
    if (result->rx_pp_complete_sample >= 0) {
        printf("  Phase 3 PP lock: start@%.1f\n",
               sample_to_ms(result->rx_pp_complete_sample, 8000));
    }
    if (result->phase3_trn_lock_score >= 0) {
        printf("  Phase 3 TRN:     best_lock=%d%%", result->phase3_trn_lock_score);
        if (result->phase3_trn_lock_sample >= 0)
            printf(" first_lock@%.1f",
                   sample_to_ms(result->phase3_trn_lock_sample, 8000));
        if (result->phase3_trn_strong_sample >= 0)
            printf(" strong_lock@%.1f",
                   sample_to_ms(result->phase3_trn_strong_sample, 8000));
        printf("\n");
    }
    if (result->tx_ja_sample >= 0) {
        char ja_preview[129];
        int ja_bits = v34_effective_ja_bit_count(result);
        const char *ja_source = result->ja_bits_from_local_tx
            ? (ja_bits > 16 ? "window_estimate" : "local_tx_mode")
            : "phase3_j_detector";
        v34_effective_ja_preview(result, ja_preview, sizeof(ja_preview));
        printf("  Ja bits:         %s (%d total bits, %s-point TRN, detector_bits=%d, source=%s)\n",
               ja_preview,
               ja_bits,
               result->ja_trn16 > 0 ? "16" : "4",
               result->ja_detector_bits,
               ja_source);
    }
    if (result->rx_s_event_sample >= 0) {
        printf("  Far-end S:       seen at %.1f ms\n",
               sample_to_ms(result->rx_s_event_sample, 8000));
    }
    if (result->rx_phase4_s_sample >= 0
        || result->rx_phase4_sbar_sample >= 0
        || result->rx_phase4_trn_sample >= 0) {
        printf("  Phase 4 RX:      ");
        if (result->rx_phase4_s_sample >= 0)
            printf("S@%.1f ", sample_to_ms(result->rx_phase4_s_sample, 8000));
        if (result->rx_phase4_sbar_sample >= 0)
            printf("S-bar@%.1f ", sample_to_ms(result->rx_phase4_sbar_sample, 8000));
        if (result->rx_phase4_trn_sample >= 0)
            printf("SCR/TRN@%.1f ", sample_to_ms(result->rx_phase4_trn_sample, 8000));
        printf("\n");
    }
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
    bool have_answerer = false;
    bool have_caller = false;
    const decode_v34_result_t *primary = NULL;
    const decode_v34_result_t *secondary = NULL;
    const char *primary_role = NULL;
    const char *secondary_role = NULL;
    int secondary_phase2_cutoff = -1;

    if (!log || !samples || total_samples <= 0)
        return;

#define APPEND_V34_STAGE_EVENT(RES_PTR, SAMPLE_FIELD, SUMMARY_STR, DETAIL_STR) \
    do { \
        if ((RES_PTR)->SAMPLE_FIELD >= 0) { \
            snprintf(detail, sizeof(detail), "role=%s %s", \
                     role_name, DETAIL_STR); \
            call_log_append(log, (RES_PTR)->SAMPLE_FIELD, 0, "V.90 Phase 3", SUMMARY_STR, detail); \
        } \
    } while (0)

    have_answerer = decode_v34_pass(samples, total_samples, law, false, &answerer);
    have_caller = decode_v34_pass(samples, total_samples, law, true, &caller);

    if (have_answerer && have_caller) {
        if (v34_result_spec_score(&answerer) >= v34_result_spec_score(&caller)) {
            primary = &answerer;
            primary_role = "answerer";
            secondary = &caller;
            secondary_role = "caller";
        } else {
            primary = &caller;
            primary_role = "caller";
            secondary = &answerer;
            secondary_role = "answerer";
        }
    } else if (have_answerer) {
        primary = &answerer;
        primary_role = "answerer";
    } else if (have_caller) {
        primary = &caller;
        primary_role = "caller";
    }

    if (primary && primary->phase3_seen)
        secondary_phase2_cutoff = primary->phase3_sample;

#define APPEND_V34_RESULT_EVENTS(RES_PTR, ROLE_STR, ALLOW_PHASE3_PLUS, PHASE2_CUTOFF) \
    do { \
        const decode_v34_result_t *res__ = (RES_PTR); \
        const char *role_name = (ROLE_STR); \
        if (!res__) \
            break; \
        if (res__->info0_seen && should_emit_phase2_event(res__->info0_sample, (PHASE2_CUTOFF))) { \
            snprintf(detail, sizeof(detail), \
                     "role=%s 3429=%s 1664pt=%s %s=%s ack_info0d=%s", \
                     role_name, \
                     res__->info0a.support_3429 ? "yes" : "no", \
                     res__->info0a.support_1664_point_constellation ? "yes" : "no", \
                     v34_info0_clock_or_law_label(res__), \
                     v34_info0_clock_or_law_value(res__), \
                     res__->info0a.acknowledge_info0d ? "yes" : "no"); \
            call_log_append(log, res__->info0_sample, 0, "V.34", v34_info0_label(res__), detail); \
        } \
        if (res__->info1_seen && should_emit_phase2_event(res__->info1_sample, (PHASE2_CUTOFF))) { \
            snprintf(detail, sizeof(detail), \
                     "role=%s md=%u u_info=%u up_rate_code=%u down_rate_code=%u freq_offset=%d", \
                     role_name, \
                     (unsigned) res__->info1a.md, \
                     (unsigned) res__->info1a.u_info, \
                     (unsigned) res__->info1a.upstream_symbol_rate_code, \
                     (unsigned) res__->info1a.downstream_rate_code, \
                     res__->info1a.freq_offset); \
            call_log_append(log, res__->info1_sample, 0, "V.34", "INFO1a decoded", detail); \
        } \
        if (!(ALLOW_PHASE3_PLUS)) \
            break; \
        if (res__->phase3_seen) { \
            snprintf(detail, sizeof(detail), "role=%s u_info=%d source=%s", \
                     role_name, \
                     res__->u_info, \
                     res__->u_info_from_info1a ? "info1a_bits25_31" : "spandsp_fallback"); \
            call_log_append(log, res__->phase3_sample, 0, "V.34", "Phase 3 reached", detail); \
        } \
        APPEND_V34_STAGE_EVENT(res__, tx_first_s_sample, "Phase 3 TX S", "sequence_start=s"); \
        APPEND_V34_STAGE_EVENT(res__, tx_first_not_s_sample, "Phase 3 TX S-bar", "sequence=first_not_s"); \
        APPEND_V34_STAGE_EVENT(res__, tx_md_sample, "Phase 3 TX MD", "sequence=md"); \
        APPEND_V34_STAGE_EVENT(res__, tx_second_s_sample, "Phase 3 TX second S", "sequence=second_s"); \
        APPEND_V34_STAGE_EVENT(res__, tx_second_not_s_sample, "Phase 3 TX second S-bar / PP lead-in", "sequence=second_not_s"); \
        if (res__->tx_pp_sample >= 0) { \
            int pp_end__ = (res__->tx_pp_end_sample >= 0) ? res__->tx_pp_end_sample : res__->tx_trn_sample; \
            int pp_samples__ = (pp_end__ > res__->tx_pp_sample) ? (pp_end__ - res__->tx_pp_sample) : 0; \
            snprintf(detail, sizeof(detail), \
                     "role=%s start=%.1fms end=%.1fms symbols=%d", \
                     role_name, \
                     sample_to_ms(res__->tx_pp_sample, 8000), \
                     pp_end__ > res__->tx_pp_sample ? sample_to_ms(pp_end__, 8000) : sample_to_ms(res__->tx_pp_sample, 8000), \
                     samples_to_v34_symbols(pp_samples__)); \
            call_log_append(log, res__->tx_pp_sample, pp_samples__, "V.90 Phase 3", "Phase 3 TX PP", detail); \
        } \
        if (res__->rx_pp_detect_sample >= 0) { \
            snprintf(detail, sizeof(detail), \
                     "role=%s phase=%d score=%d hold=%d started=%s", \
                     role_name, \
                     res__->rx_pp_phase, \
                     res__->rx_pp_phase_score, \
                     res__->rx_pp_acquire_hits, \
                     res__->rx_pp_started ? "yes" : "no"); \
            call_log_append(log, res__->rx_pp_detect_sample, 0, "V.90 Phase 3", "Phase 3 RX PP detect", detail); \
        } \
        if (res__->rx_pp_complete_sample >= 0) { \
            snprintf(detail, sizeof(detail), "role=%s phase=%d score=%d", role_name, res__->rx_pp_phase, res__->rx_pp_phase_score); \
            call_log_append(log, res__->rx_pp_complete_sample, 0, "V.90 Phase 3", "Phase 3 RX PP lock", detail); \
        } \
        APPEND_V34_STAGE_EVENT(res__, tx_trn_sample, "Phase 3 TX TRN", "sequence=trn"); \
        if (res__->phase3_trn_lock_score >= 0) { \
            snprintf(detail, sizeof(detail), \
                     "role=%s best_lock=%d%%%s%s", \
                     role_name, \
                     res__->phase3_trn_lock_score, \
                     res__->phase3_trn_strong_sample >= 0 ? " strong_lock=yes" : "", \
                     res__->phase3_trn_strong_sample >= 0 ? "" : " strong_lock=no"); \
            call_log_append(log, \
                            res__->phase3_trn_lock_sample >= 0 ? res__->phase3_trn_lock_sample : res__->tx_trn_sample, \
                            0, \
                            "V.90 Phase 3", \
                            "Phase 3 TRN lock", \
                            detail); \
        } \
        if (res__->tx_ja_sample >= 0) { \
            char ja_preview__[129]; \
            int ja_bits__ = v34_effective_ja_bit_count(res__); \
            const char *ja_source__ = res__->ja_bits_from_local_tx \
                ? (ja_bits__ > 16 ? "window_estimate" : "local_tx_mode") \
                : "phase3_j_detector"; \
            v34_effective_ja_preview(res__, ja_preview__, sizeof(ja_preview__)); \
            if (ja_bits__ > 0) { \
                snprintf(detail, sizeof(detail), \
                         "role=%s sequence=j bits=%s total_bits=%d%s source=%s", \
                         role_name, \
                         ja_preview__, \
                         ja_bits__, \
                         res__->ja_trn16 > 0 ? " trn=16" : " trn=4", \
                         ja_source__); \
            } else { \
                snprintf(detail, sizeof(detail), "role=%s sequence=j", role_name); \
            } \
            call_log_append(log, res__->tx_ja_sample, 0, "V.90 Phase 3", "Phase 3 TX J/Ja", detail); \
        } \
        APPEND_V34_STAGE_EVENT(res__, tx_jdashed_sample, "Phase 3 TX J'", "sequence=j_dashed"); \
        APPEND_V34_STAGE_EVENT(res__, rx_s_event_sample, "Far-end S seen during Ja/Jd", "rx_event=s"); \
        APPEND_V34_STAGE_EVENT(res__, rx_phase4_s_sample, "Far-end Phase 4 S", "rx_stage=phase4_s"); \
        APPEND_V34_STAGE_EVENT(res__, rx_phase4_sbar_sample, "Far-end Phase 4 S-bar", "rx_stage=phase4_s_bar"); \
        APPEND_V34_STAGE_EVENT(res__, rx_phase4_trn_sample, "Far-end Phase 4 SCR/TRN", "rx_stage=phase4_trn"); \
        if (res__->phase4_ready_seen) { \
            snprintf(detail, sizeof(detail), "role=%s event=%s (%d)", role_name, v34_event_to_str_local(res__->final_rx_event), res__->final_rx_event); \
            call_log_append(log, res__->phase4_ready_sample, 0, "V.90/V.92", "Phase 4 training ready", detail); \
        } \
        if (res__->phase4_seen) { \
            snprintf(detail, sizeof(detail), "role=%s rx=%s(%d) tx=%s(%d)", \
                     role_name, \
                     v34_rx_stage_to_str_local(res__->final_rx_stage), \
                     res__->final_rx_stage, \
                     v34_tx_stage_to_str_local(res__->final_tx_stage), \
                     res__->final_tx_stage); \
            call_log_append(log, res__->phase4_sample, 0, "V.90/V.92", "Phase 4 / MP reached", detail); \
        } \
    } while (0)

    APPEND_V34_RESULT_EVENTS(primary, primary_role, true, -1);
    APPEND_V34_RESULT_EVENTS(secondary, secondary_role, false, secondary_phase2_cutoff);

#undef APPEND_V34_RESULT_EVENTS

#undef APPEND_V34_STAGE_EVENT
}

static void collect_stream_call_log(call_log_t *log,
                                    const int16_t *linear_samples,
                                    const uint8_t *g711_codewords,
                                    int total_samples,
                                    int total_codewords,
                                    v91_law_t law,
                                    bool do_v34,
                                    bool do_v8,
                                    bool do_v91,
                                    bool do_v90)
{
    decode_v34_result_t answerer;
    decode_v34_result_t caller;
    bool have_answerer = false;
    bool have_caller = false;
    int earliest_phase2_sample = -1;

    if (!log || !linear_samples || !g711_codewords)
        return;

    if (do_v34 || do_v90) {
        have_answerer = decode_v34_pass(linear_samples, total_samples, law, false, &answerer);
        have_caller = decode_v34_pass(linear_samples, total_samples, law, true, &caller);
        if (have_answerer) {
            if (answerer.info0_seen)
                earliest_phase2_sample = first_non_negative(earliest_phase2_sample, answerer.info0_sample);
            if (answerer.info1_seen)
                earliest_phase2_sample = first_non_negative(earliest_phase2_sample, answerer.info1_sample);
        }
        if (have_caller) {
            if (caller.info0_seen)
                earliest_phase2_sample = first_non_negative(earliest_phase2_sample, caller.info0_sample);
            if (caller.info1_seen)
                earliest_phase2_sample = first_non_negative(earliest_phase2_sample, caller.info1_sample);
        }
    }

    if (do_v34)
        collect_v34_events(log, linear_samples, total_samples, law);
    if (do_v8) {
        v8bis_collect_signal_events(log, linear_samples, total_samples, earliest_phase2_sample);
        v8bis_collect_msg_events(log, linear_samples, total_samples, earliest_phase2_sample);
        collect_v8_event(log, linear_samples, g711_codewords, total_samples, earliest_phase2_sample, law);
    }
    if (do_v91)
        collect_v91_events(log, g711_codewords, total_codewords, law);
    if (do_v90) {
        collect_v90_events(log, g711_codewords, total_codewords, law);
        if (have_answerer || have_caller) {
            collect_post_phase3_stage_events(log,
                                             g711_codewords,
                                             total_codewords,
                                             have_answerer ? &answerer : NULL,
                                             have_caller ? &caller : NULL);
        }
    }

    call_log_sort(log);
    call_log_prune_v8_after_phase2(log);
    call_log_sort(log);
}

static void call_log_merge_with_channel(call_log_t *dst,
                                        const call_log_t *src,
                                        const char *channel_label)
{
    char detail[320];

    if (!dst || !src || !channel_label)
        return;

    for (size_t i = 0; i < src->count; i++) {
        const call_log_event_t *event = &src->events[i];

        if (event->detail[0] != '\0')
            snprintf(detail, sizeof(detail), "channel=%s %s", channel_label, event->detail);
        else
            snprintf(detail, sizeof(detail), "channel=%s", channel_label);

        call_log_append(dst,
                        event->sample_offset,
                        event->duration_samples,
                        event->protocol,
                        event->summary,
                        detail);
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
    ja_dil_decode_t ja_dil;
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

    if (decode_ja_dil_stage(codewords, total_codewords, answerer, caller, &jd_stage, &ja_dil)) {
        snprintf(detail, sizeof(detail),
                 "role=%s n=%u lsp=%u ltp=%u uniq_u=%u uchords=%u impairment=%u",
                 ja_dil.calling_party ? "caller" : "answerer",
                 (unsigned) ja_dil.desc.n,
                 (unsigned) ja_dil.desc.lsp,
                 (unsigned) ja_dil.desc.ltp,
                 (unsigned) ja_dil.analysis.unique_train_u,
                 (unsigned) ja_dil.analysis.used_uchords,
                 (unsigned) ja_dil.analysis.impairment_score);
        call_log_append(log,
                        ja_dil.start_sample,
                        0,
                        "V.90",
                        "Ja/DIL descriptor decoded",
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

    if (!codewords || !out_bits
        || start_sample < (OFFLINE_V90_SCRAMBLER_HISTORY + 1)
        || start_sample + OFFLINE_V90_JD_BITS > total_codewords)
        return OFFLINE_V90_JD_BITS + 1;

    offline_v90_build_jd_bits(expected);
    descramble_reg = 0;
    prev_sign = ((codewords[start_sample - OFFLINE_V90_SCRAMBLER_HISTORY - 1] & 0x80) ? 1 : 0);
    if (invert_sign)
        prev_sign ^= 1;

    for (int i = start_sample - OFFLINE_V90_SCRAMBLER_HISTORY; i < start_sample; i++) {
        int sign = (codewords[i] & 0x80) ? 1 : 0;
        int scrambled;
        if (invert_sign)
            sign ^= 1;
        scrambled = sign ^ prev_sign;
        prev_sign = sign;
        (void) offline_v90_descramble_reg_bit(&descramble_reg, scrambled);
    }

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

static bool offline_v90_decode_plain_bits_packed(const uint8_t *codewords,
                                                 int total_codewords,
                                                 int start_sample,
                                                 int bit_count,
                                                 bool invert_sign,
                                                 uint8_t *packed_out,
                                                 int packed_len)
{
    uint32_t descramble_reg;
    int prev_sign;

    if (!codewords || !packed_out || packed_len <= 0 || bit_count <= 0
        || start_sample < (OFFLINE_V90_SCRAMBLER_HISTORY + 1)
        || start_sample + bit_count > total_codewords
        || packed_len < ((bit_count + 7) / 8)) {
        return false;
    }

    memset(packed_out, 0, (size_t) packed_len);
    descramble_reg = 0;
    prev_sign = ((codewords[start_sample - OFFLINE_V90_SCRAMBLER_HISTORY - 1] & 0x80) ? 1 : 0);
    if (invert_sign)
        prev_sign ^= 1;

    for (int i = start_sample - OFFLINE_V90_SCRAMBLER_HISTORY; i < start_sample; i++) {
        int sign = (codewords[i] & 0x80) ? 1 : 0;
        int scrambled;
        if (invert_sign)
            sign ^= 1;
        scrambled = sign ^ prev_sign;
        prev_sign = sign;
        (void) offline_v90_descramble_reg_bit(&descramble_reg, scrambled);
    }

    for (int i = 0; i < bit_count; i++) {
        int sign = (codewords[start_sample + i] & 0x80) ? 1 : 0;
        int scrambled;
        int plain;

        if (invert_sign)
            sign ^= 1;
        scrambled = sign ^ prev_sign;
        prev_sign = sign;
        plain = offline_v90_descramble_reg_bit(&descramble_reg, scrambled);
        if (plain)
            packed_out[i / 8] |= (uint8_t) (1U << (i % 8));
    }

    return true;
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
    out->analog_ja_sample = src->tx_ja_sample;
    out->far_end_s_sample = src->rx_s_event_sample;
    search_start = src->tx_ja_sample >= 0 ? src->tx_ja_sample
                 : (src->tx_trn_sample >= 0 ? src->tx_trn_sample
                    : (src->info1_sample >= 0 ? src->info1_sample : out->phase3_start_sample));
    if (search_start < 0)
        return false;
    if (search_start < (OFFLINE_V90_SCRAMBLER_HISTORY + 1))
        search_start = OFFLINE_V90_SCRAMBLER_HISTORY + 1;

    if (src->tx_jdashed_sample > search_start)
        search_end = src->tx_jdashed_sample;
    else if (src->rx_phase4_s_sample > search_start)
        search_end = src->rx_phase4_s_sample;
    else if (src->phase4_seen && src->phase4_sample > search_start)
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

    if (out->jd_repetitions < 1)
        return false;

    {
        int jd_prime_start = best_start + out->jd_repetitions * OFFLINE_V90_JD_BITS;
        uint32_t descramble_reg;
        int prev_sign;

        descramble_reg = 0;
        prev_sign = ((codewords[best_start - OFFLINE_V90_SCRAMBLER_HISTORY - 1] & 0x80) ? 1 : 0);
        if (best_invert)
            prev_sign ^= 1;
        for (int i = best_start - OFFLINE_V90_SCRAMBLER_HISTORY; i < best_start + out->jd_repetitions * OFFLINE_V90_JD_BITS; i++) {
            int sign = (codewords[i] & 0x80) ? 1 : 0;
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
    if (result->analog_ja_sample >= 0)
        printf("  Phase 3 TX J/Ja:  %.1f ms\n", sample_to_ms(result->analog_ja_sample, 8000));
    if (result->far_end_s_sample >= 0)
        printf("  Far-end S:        %.1f ms\n", sample_to_ms(result->far_end_s_sample, 8000));
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

static bool decode_ja_dil_stage(const uint8_t *codewords,
                                int total_codewords,
                                const decode_v34_result_t *answerer,
                                const decode_v34_result_t *caller,
                                const jd_stage_decode_t *jd_stage,
                                ja_dil_decode_t *out)
{
    const decode_v34_result_t *src;
    bool calling_party = false;
    int search_start;
    int search_end;
    int best_score = -1;
    bool best_invert = false;
    int best_start = -1;
    v90_dil_desc_t best_desc;
    v90_dil_analysis_t best_analysis;
    uint8_t packed_bits[512];

    if (!codewords || total_codewords <= 0 || !out)
        return false;

    memset(out, 0, sizeof(*out));
    src = pick_post_phase3_source(answerer, caller, &calling_party);
    if (!src)
        return false;

    /*
     * Prefer the locally decoded Ja anchor when it exists. The DIL descriptor
     * is determined by the negotiated Ja parameters, so using a later Jd/J'd
     * alignment as the primary seed can pull the search onto the wrong region.
     */
    if (src->tx_ja_sample >= 0) {
        search_start = src->tx_ja_sample;
    } else if (jd_stage && jd_stage->ok) {
        if (jd_stage->jd_prime_seen)
            search_start = jd_stage->jd_prime_sample + OFFLINE_V90_JD_PRIME_BITS;
        else if (jd_stage->jd_repetitions > 0)
            search_start = jd_stage->jd_start_sample + jd_stage->jd_repetitions * OFFLINE_V90_JD_BITS;
        else
            search_start = jd_stage->jd_start_sample;
    } else {
        search_start = src->info1_sample >= 0 ? src->info1_sample : src->phase3_sample;
    }
    if (search_start < (OFFLINE_V90_SCRAMBLER_HISTORY + 1))
        search_start = OFFLINE_V90_SCRAMBLER_HISTORY + 1;

    if (src->phase4_seen && src->phase4_sample > search_start)
        search_end = src->phase4_sample;
    else if (src->phase4_ready_seen && src->phase4_ready_sample > search_start)
        search_end = src->phase4_ready_sample;
    else if (src->rx_phase4_trn_sample > search_start)
        search_end = src->rx_phase4_trn_sample;
    else if (src->failure_sample > search_start)
        search_end = src->failure_sample;
    else
        search_end = search_start + 4096;
    if (search_end > total_codewords - 206)
        search_end = total_codewords - 206;
    if (search_end < search_start)
        return false;

    for (int candidate = search_start; candidate <= search_end; candidate++) {
        for (int invert = 0; invert <= 1; invert++) {
            v90_dil_desc_t desc;
            v90_dil_analysis_t analysis;
            int bit_count = total_codewords - candidate;
            int packed_len;
            int score;

            if (bit_count > (int) sizeof(packed_bits) * 8)
                bit_count = (int) sizeof(packed_bits) * 8;
            packed_len = (bit_count + 7) / 8;
            if (!offline_v90_decode_plain_bits_packed(codewords, total_codewords,
                                                      candidate, bit_count, invert != 0,
                                                      packed_bits, packed_len)) {
                continue;
            }
            if (!v90_parse_dil_descriptor(&desc, packed_bits, bit_count))
                continue;
            if (!v90_analyse_dil_descriptor(&desc, &analysis))
                continue;

            score = analysis.unique_train_u * 100
                  + analysis.used_uchords * 50
                  - analysis.impairment_score * 10
                  - analysis.non_default_h * 5;
            if (src->tx_ja_sample >= 0) {
                int dist = candidate - src->tx_ja_sample;
                if (dist < 0)
                    dist = -dist;
                score -= dist / 8;
            }
            if (score > best_score) {
                best_score = score;
                best_invert = (invert != 0);
                best_start = candidate;
                best_desc = desc;
                best_analysis = analysis;
            }
        }
    }

    if (best_score < 0)
        return false;

    out->ok = true;
    out->calling_party = calling_party;
    out->u_info = src->u_info;
    out->start_sample = best_start;
    out->invert_sign = best_invert;
    out->desc = best_desc;
    out->analysis = best_analysis;
    return true;
}

static void print_ja_dil_decode(const ja_dil_decode_t *result)
{
    if (!result || !result->ok)
        return;

    printf("\n=== Ja/DIL Decode ===\n");
    printf("  Source role:      %s\n", result->calling_party ? "caller" : "answerer");
    printf("  Start time:       %.1f ms\n", sample_to_ms(result->start_sample, 8000));
    printf("  U_INFO:           %d\n", result->u_info);
    printf("  Descriptor:       n=%u lsp=%u ltp=%u\n",
           (unsigned) result->desc.n,
           (unsigned) result->desc.lsp,
           (unsigned) result->desc.ltp);
    printf("  Analysis:         unique_train_u=%u used_uchords=%u impairment=%u\n",
           (unsigned) result->analysis.unique_train_u,
           (unsigned) result->analysis.used_uchords,
           (unsigned) result->analysis.impairment_score);
    printf("  Recommendations:  down_drn=%u up_drn=%u\n",
           (unsigned) result->analysis.recommended_downstream_drn,
           (unsigned) result->analysis.recommended_upstream_drn);
}

static const decode_v34_result_t *pick_post_phase3_source(const decode_v34_result_t *answerer,
                                                          const decode_v34_result_t *caller,
                                                          bool *calling_party_out)
{
    if (answerer && answerer->u_info_from_info1a && answerer->u_info > 0) {
        if (calling_party_out)
            *calling_party_out = false;
        return answerer;
    }
    if (caller && caller->u_info_from_info1a && caller->u_info > 0) {
        if (calling_party_out)
            *calling_party_out = true;
        return caller;
    }
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
        decode_v8_pass(linear_samples, g711_codewords, total_samples, law, false);
        printf("\n=== V.8 Decode (as caller) ===\n");
        decode_v8_pass(linear_samples, g711_codewords, total_samples, law, true);
    }

    if (opts->raw_output_enabled && opts->do_v91)
        decode_v91_signals(g711_codewords, total_codewords, law);

    if (opts->raw_output_enabled && opts->do_v90) {
        jd_stage_decode_t jd_stage;
        ja_dil_decode_t ja_dil;
        post_phase3_decode_t post_phase3;

        decode_v90_signals(g711_codewords, total_codewords, law);
        if (decode_jd_stage(g711_codewords, total_codewords,
                            have_answerer ? &answerer : NULL,
                            have_caller ? &caller : NULL,
                            &jd_stage)) {
            print_jd_stage_decode(&jd_stage);
        }
        if (decode_ja_dil_stage(g711_codewords, total_codewords,
                                have_answerer ? &answerer : NULL,
                                have_caller ? &caller : NULL,
                                &jd_stage,
                                &ja_dil)) {
            print_ja_dil_decode(&ja_dil);
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

        collect_stream_call_log(&log,
                                linear_samples,
                                g711_codewords,
                                total_samples,
                                total_codewords,
                                law,
                                opts->do_v34,
                                opts->do_v8,
                                opts->do_v91,
                                opts->do_v90);
        print_call_log(label, &log, total_samples, sample_rate);
        call_log_reset(&log);
    }
}

int main(int argc, char **argv)
{
    const char *input_path = NULL;
    const char *visualize_html_path = NULL;
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
    bool do_visualize_html = false;

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
        } else if (strcmp(argv[i], "--visualize-html") == 0 && i + 1 < argc) {
            do_visualize_html = true;
            visualize_html_path = argv[++i];
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
                   "                     (default: mono; stereo auto-runs left+right)\n"
                   "  --v34              Decode V.34/V.90 Phase 2 symbols via SpanDSP\n"
                   "  --v8               Decode V.8 negotiation\n"
                   "  --v91              Decode V.91 signals (INFO, CP, DIL, Ez, etc.)\n"
                   "  --v90              Decode V.90 signals (Sd, CP, etc.)\n"
                   "  --energy           Print RMS energy profile\n"
                   "  --stats            Print codeword histogram/statistics\n"
                   "  --call-log         Print a best-effort chronological call log\n"
                   "  --visualize-html   Export a self-contained HTML audio/tone viewer\n"
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

    if (!do_v34 && !do_v8 && !do_v91 && !do_v90 && !do_energy && !do_stats && !do_call_log && !do_visualize_html)
        do_all = true;

    if (do_all) {
        do_v34 = do_v8 = do_v91 = do_v90 = do_energy = do_stats = do_call_log = true;
    }

    if (do_visualize_html)
        do_call_log = true;

    if (do_call_log) {
        if (!do_v34 && !do_v8 && !do_v91 && !do_v90) {
            do_v34 = true;
            do_v91 = true;
            do_v90 = true;
        }
        /* V.8 helps establish caller/answerer orientation, so keep it in call logs
           even when the user explicitly asks for only later-stage decoders. */
        do_v8 = true;
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
               channel == CH_LEFT ? "Left" : channel == CH_RIGHT ? "Right" : "Mono",
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
            run_decode_suite("Left", left_linear_samples, left_g711_codewords,
                             total_samples, total_codewords, sample_rate, law, &opts, &left_codeword_info);
            run_decode_suite("Right", right_linear_samples, right_g711_codewords,
                             total_samples, total_codewords, sample_rate, law, &opts, &right_codeword_info);
            if (opts.do_call_log) {
                call_log_t left_log;
                call_log_t right_log;
                call_log_t combined_log;

                call_log_init(&left_log);
                call_log_init(&right_log);
                call_log_init(&combined_log);

                collect_stream_call_log(&left_log,
                                        left_linear_samples,
                                        left_g711_codewords,
                                        total_samples,
                                        total_codewords,
                                        law,
                                        opts.do_v34,
                                        opts.do_v8,
                                        opts.do_v91,
                                        opts.do_v90);
                collect_stream_call_log(&right_log,
                                        right_linear_samples,
                                        right_g711_codewords,
                                        total_samples,
                                        total_codewords,
                                        law,
                                        opts.do_v34,
                                        opts.do_v8,
                                        opts.do_v91,
                                        opts.do_v90);
                call_log_merge_with_channel(&combined_log, &left_log, "left");
                call_log_merge_with_channel(&combined_log, &right_log, "right");
                call_log_sort(&combined_log);
                v8bis_dedup_msgs(&combined_log);
                print_call_log("Stereo Combined", &combined_log, total_samples, sample_rate);

                call_log_reset(&left_log);
                call_log_reset(&right_log);
                call_log_reset(&combined_log);
            }
        } else {
            run_decode_suite(channel == CH_LEFT ? "Left"
                             : channel == CH_RIGHT ? "Right"
                             : "Mono",
                             linear_samples, g711_codewords,
                             total_samples, total_codewords, sample_rate, law, &opts, &codeword_info);
        }

        if (do_visualize_html) {
            call_log_t viz_logs[2];
            audio_visualization_t viz_data[2];
            const char *viz_labels[2];
            const call_log_t *viz_log_ptrs[2];
            const audio_visualization_t *viz_data_ptrs[2];
            bool gough_lui_caller_view = (strstr(input_path, "gough-lui-v90-v92-modem-sounds/") != NULL);
            int viz_track_count = 0;
            bool wrote_html = false;

            memset(viz_logs, 0, sizeof(viz_logs));
            memset(viz_data, 0, sizeof(viz_data));

            if (left_linear_samples && right_linear_samples && left_g711_codewords && right_g711_codewords) {
                call_log_init(&viz_logs[0]);
                call_log_init(&viz_logs[1]);
                collect_stream_call_log(&viz_logs[0],
                                        left_linear_samples,
                                        left_g711_codewords,
                                        total_samples,
                                        total_codewords,
                                        law,
                                        opts.do_v34,
                                        opts.do_v8,
                                        opts.do_v91,
                                        opts.do_v90);
                collect_stream_call_log(&viz_logs[1],
                                        right_linear_samples,
                                        right_g711_codewords,
                                        total_samples,
                                        total_codewords,
                                        law,
                                        opts.do_v34,
                                        opts.do_v8,
                                        opts.do_v91,
                                        opts.do_v90);
                viz_labels[0] = gough_lui_caller_view ? "Caller RX (L)" : "Left (L)";
                viz_labels[1] = gough_lui_caller_view ? "Caller TX (R)" : "Right (R)";
                viz_log_ptrs[0] = &viz_logs[0];
                viz_log_ptrs[1] = &viz_logs[1];
                viz_track_count = 2;
                if (!build_audio_visualization(&viz_data[0], left_linear_samples, total_samples, sample_rate)
                    || !build_audio_visualization(&viz_data[1], right_linear_samples, total_samples, sample_rate)) {
                    viz_track_count = 0;
                } else {
                    viz_data_ptrs[0] = &viz_data[0];
                    viz_data_ptrs[1] = &viz_data[1];
                }
            } else {
                call_log_init(&viz_logs[0]);
                collect_stream_call_log(&viz_logs[0],
                                        linear_samples,
                                        g711_codewords,
                                        total_samples,
                                        total_codewords,
                                        law,
                                        opts.do_v34,
                                        opts.do_v8,
                                        opts.do_v91,
                                        opts.do_v90);
                viz_labels[0] = channel == CH_LEFT ? "Left (L)"
                                : channel == CH_RIGHT ? "Right (R)"
                                : "Mono";
                viz_log_ptrs[0] = &viz_logs[0];
                viz_track_count = build_audio_visualization(&viz_data[0], linear_samples, total_samples, sample_rate) ? 1 : 0;
                if (viz_track_count == 1)
                    viz_data_ptrs[0] = &viz_data[0];
            }

            if (viz_track_count > 0)
                wrote_html = write_visualization_html(visualize_html_path,
                                                      input_path,
                                                      viz_labels,
                                                      viz_log_ptrs,
                                                      viz_data_ptrs,
                                                      viz_track_count);

            if (wrote_html)
                printf("\nVisualization HTML written to %s\n", visualize_html_path);
            else
                fprintf(stderr, "\nFailed to write visualization HTML: %s\n", visualize_html_path);

            for (int i = 0; i < 2; i++) {
                audio_visualization_reset(&viz_data[i]);
                call_log_reset(&viz_logs[i]);
            }
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
