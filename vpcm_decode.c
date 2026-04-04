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
#include "v92_short_phase2_decode.h"
#include "v92_phase3_decode.h"
#include "v92_ja_decode.h"
#include "v34_phase2_decode.h"
#include "v34_info_decode.h"
#include "p3_demod.h"
#include "phase12_decode.h"

#include <spandsp.h>
#include <spandsp/crc.h>
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
#include <stdarg.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <limits.h>

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

static bool span_is_decimal(const char *start, const char *end)
{
    if (!start || !end || start >= end)
        return false;
    for (const char *p = start; p < end; p++) {
        if (*p < '0' || *p > '9')
            return false;
    }
    return true;
}

static bool parse_filename_rate_pair(const char *path, int *first_bps, int *second_bps)
{
    const char *base;
    const char *dot;
    const char *last_dash;
    const char *prev_dash;

    if (first_bps)
        *first_bps = -1;
    if (second_bps)
        *second_bps = -1;
    if (!path)
        return false;

    base = strrchr(path, '/');
    base = base ? (base + 1) : path;
    dot = strrchr(base, '.');
    if (!dot)
        dot = base + strlen(base);

    last_dash = dot;
    while (last_dash > base && *last_dash != '-')
        last_dash--;
    if (last_dash <= base || *last_dash != '-')
        return false;

    prev_dash = last_dash - 1;
    while (prev_dash > base && *prev_dash != '-')
        prev_dash--;
    if (prev_dash < base || *prev_dash != '-')
        return false;

    if (!span_is_decimal(prev_dash + 1, last_dash)
        || !span_is_decimal(last_dash + 1, dot))
        return false;

    if (first_bps)
        *first_bps = atoi(prev_dash + 1);
    if (second_bps)
        *second_bps = atoi(last_dash + 1);
    return true;
}

static void appendf(char *buf, size_t len, const char *fmt, ...)
{
    size_t used;
    int wrote;
    va_list ap;

    if (!buf || len == 0 || !fmt)
        return;
    used = strlen(buf);
    if (used >= len - 1)
        return;
    va_start(ap, fmt);
    wrote = vsnprintf(buf + used, len - used, fmt, ap);
    va_end(ap);
    if (wrote < 0)
        buf[len - 1] = '\0';
}

static void sanitize_detail_value(char *dst, size_t dst_len, const char *src)
{
    size_t i;

    if (!dst || dst_len == 0)
        return;
    if (!src) {
        dst[0] = '\0';
        return;
    }
    for (i = 0; i < dst_len - 1 && src[i]; i++)
        dst[i] = (src[i] == ' ') ? '_' : src[i];
    dst[i] = '\0';
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

static void format_v34_info0_table_summary(char *buf,
                                           size_t len,
                                           const v90_info0a_t *info,
                                           const v34_v90_info0a_t *raw_info,
                                           const char *role_name,
                                           bool info0_is_d)
{
    v90_info0a_diag_t diag;
    bool have_crc = false;
    uint8_t raw_26_27 = 0;

    if (!buf || len == 0) {
        return;
    }
    buf[0] = '\0';
    if (!info) {
        return;
    }
    if (!info0_is_d && v90_info0a_build_diag(info, &diag))
        have_crc = true;

    if (raw_info)
        raw_26_27 = raw_info->raw_26_27;

    appendf(buf, len, "role=%s", role_name ? role_name : "unknown");
    appendf(buf, len, " profile=%s", info0_is_d ? "V90V92_INFO0d_T7_T15" : "V90V92_INFO0a_T8_T16");
    appendf(buf, len, " fsync=0x%03X", V90_INFO_FILL_AND_SYNC_BITS);
    appendf(buf, len, " b12_2743=%u", info->support_2743 ? 1U : 0U);
    appendf(buf, len, " b13_2800=%u", info->support_2800 ? 1U : 0U);
    appendf(buf, len, " b14_3429=%u", info->support_3429 ? 1U : 0U);
    appendf(buf, len, " b15_3000l=%u", info->support_3000_low ? 1U : 0U);
    appendf(buf, len, " b16_3000h=%u", info->support_3000_high ? 1U : 0U);
    appendf(buf, len, " b17_3200l=%u", info->support_3200_low ? 1U : 0U);
    appendf(buf, len, " b18_3200h=%u", info->support_3200_high ? 1U : 0U);
    appendf(buf, len, " b19_3429ok=%u", info->rate_3429_allowed ? 1U : 0U);
    appendf(buf, len, " b20_pwrred=%u", info->support_power_reduction ? 1U : 0U);
    appendf(buf, len, " b21_23_maxbaud=%u", (unsigned) info->max_baud_rate_difference);
    appendf(buf, len, " b24_cme=%u", info->from_cme_modem ? 1U : 0U);
    appendf(buf, len, " b25_1664pt=%u", info->support_1664_point_constellation ? 1U : 0U);
    if (info0_is_d) {
        appendf(buf, len, " b26_shortp2_req=%u", (unsigned) (raw_26_27 & 0x01U));
        appendf(buf, len, " b27_v92_cap=%u", (unsigned) ((raw_26_27 >> 1) & 0x01U));
        appendf(buf, len, " b28_ack_i0a=%u", info->acknowledge_info0d ? 1U : 0U);
        appendf(buf, len, " b29_32_nom_tx_pwr_code=%s",
                (raw_info && raw_info->info0d_extensions_valid) ? "" : "unavailable");
        if (raw_info && raw_info->info0d_extensions_valid)
            appendf(buf, len, "%u", (unsigned) raw_info->info0d_nominal_power_code);
        appendf(buf, len, " b33_37_max_tx_pwr_code=%s",
                (raw_info && raw_info->info0d_extensions_valid) ? "" : "unavailable");
        if (raw_info && raw_info->info0d_extensions_valid)
            appendf(buf, len, "%u", (unsigned) raw_info->info0d_max_power_code);
        appendf(buf, len, " b38_pwr_meas_codec_out=%s",
                (raw_info && raw_info->info0d_extensions_valid)
                    ? (raw_info->info0d_power_measured_at_codec_output ? "1" : "0")
                    : "unavailable");
        appendf(buf, len, " b39_pcm_law=%s",
                (raw_info && raw_info->info0d_extensions_valid)
                    ? (raw_info->info0d_pcm_alaw ? "alaw" : "ulaw")
                    : "unavailable");
        appendf(buf, len, " b40_v90_us3429=%s",
                (raw_info && raw_info->info0d_extensions_valid)
                    ? (raw_info->info0d_upstream_3429_support ? "1" : "0")
                    : "unavailable");
        appendf(buf, len, " b41_reserved=%s",
                (raw_info && raw_info->info0d_extensions_valid)
                    ? (raw_info->info0d_reserved_41 ? "1" : "0")
                    : "unavailable");
        appendf(buf, len, " crc_bits=42:57");
        appendf(buf, len, " crc16=unavailable");
        appendf(buf, len, " tail_bits=58:61");
        appendf(buf, len, " tail=0xF");
    } else {
        appendf(buf, len, " b26_v92_cap=%u", (unsigned) (raw_26_27 & 0x01U));
        appendf(buf, len, " b27_shortp2_req=%u", (unsigned) ((raw_26_27 >> 1) & 0x01U));
        appendf(buf, len, " b28_ack_i0d=%u", info->acknowledge_info0d ? 1U : 0U);
        appendf(buf, len, " crc_bits=29:44");
        appendf(buf, len, " crc16=%s", have_crc ? "" : "n/a");
        if (have_crc)
            appendf(buf, len, "0x%04X", (unsigned) diag.crc_field);
        appendf(buf, len, " tail_bits=45:48");
        appendf(buf, len, " tail=0xF");
    }
}

static void format_v34_info1_table15_summary(char *buf,
                                             size_t len,
                                             const v90_info1a_t *info,
                                             const v34_v90_info1a_t *raw_info,
                                             const char *role_name)
{
    v90_info1a_diag_t diag;
    uint16_t freq_raw;
    bool have_crc = false;
    bool looks_like_v92_table18 = false;

    if (!buf || len == 0) {
        return;
    }
    buf[0] = '\0';
    if (!info) {
        return;
    }
    freq_raw = (info->freq_offset < 0)
        ? (uint16_t) (0x400 + info->freq_offset)
        : (uint16_t) info->freq_offset;
    if (v90_info1a_build_diag(info, &diag))
        have_crc = true;
    if (raw_info
        && raw_info->upstream_symbol_rate_code == 6
        && raw_info->downstream_rate_code == 6) {
        looks_like_v92_table18 = true;
    }

    appendf(buf, len, "role=%s", role_name ? role_name : "unknown");
    appendf(buf, len, " table=%s",
            looks_like_v92_table18
                ? "V92_T18_INFO1a_pcm_upstream"
                : "V90_T10_or_V92_T19_INFO1a_v34_upstream");
    appendf(buf, len, " fsync=0x%03X", V90_INFO_FILL_AND_SYNC_BITS);
    appendf(buf, len, " b12_17_raw=%u", raw_info ? (unsigned) raw_info->raw_12_17 : 0U);
    appendf(buf, len, " md=%u", (unsigned) info->md);
    appendf(buf, len, " u_info=%u", (unsigned) info->u_info);
    appendf(buf, len, " b32_33_raw=%u", raw_info ? (unsigned) raw_info->raw_32_33 : 0U);
    appendf(buf, len, " upstream_symbol_rate_code=%u", (unsigned) info->upstream_symbol_rate_code);
    appendf(buf, len, " downstream_rate_code=%u", (unsigned) info->downstream_rate_code);
    appendf(buf, len, " freq_offset_10b=0x%03X(%+d)", (unsigned) (freq_raw & 0x3FFU), info->freq_offset);
    appendf(buf, len, " crc_bits=50:65");
    appendf(buf, len, " crc16=%s", have_crc ? "" : "n/a");
    if (have_crc)
        appendf(buf, len, "0x%04X", (unsigned) diag.crc_field);
    appendf(buf, len, " tail_bits=66:69");
    appendf(buf, len, " tail=0xF");
}

static void format_v34_info1d_table_summary(char *buf,
                                            size_t len,
                                            const v34_v90_info1d_t *info,
                                            const char *role_name)
{
    if (!buf || len == 0) {
        return;
    }
    buf[0] = '\0';
    if (!info)
        return;

    appendf(buf, len, "role=%s", role_name ? role_name : "unknown");
    appendf(buf, len, " table=V90_T9_or_V92_T17_INFO1d");
    appendf(buf, len, " fsync=0x%03X", V90_INFO_FILL_AND_SYNC_BITS);
    appendf(buf, len, " b12_14_min_pwr_red=%d", info->power_reduction);
    appendf(buf, len, " b15_17_addl_pwr_red=%d", info->additional_power_reduction);
    appendf(buf, len, " b18_24_md=%d", info->md);
    appendf(buf, len, " row2400_hicar=%u row2400_preemp=%d row2400_maxrate=%d",
            info->rate_data[0].use_high_carrier ? 1U : 0U,
            info->rate_data[0].pre_emphasis,
            info->rate_data[0].max_bit_rate);
    appendf(buf, len, " row2743_hicar=%u row2743_preemp=%d row2743_maxrate=%d",
            info->rate_data[1].use_high_carrier ? 1U : 0U,
            info->rate_data[1].pre_emphasis,
            info->rate_data[1].max_bit_rate);
    appendf(buf, len, " row2800_hicar=%u row2800_preemp=%d row2800_maxrate=%d",
            info->rate_data[2].use_high_carrier ? 1U : 0U,
            info->rate_data[2].pre_emphasis,
            info->rate_data[2].max_bit_rate);
    appendf(buf, len, " row3000_hicar=%u row3000_preemp=%d row3000_maxrate=%d",
            info->rate_data[3].use_high_carrier ? 1U : 0U,
            info->rate_data[3].pre_emphasis,
            info->rate_data[3].max_bit_rate);
    appendf(buf, len, " row3200_hicar=%u row3200_preemp=%d row3200_maxrate=%d",
            info->rate_data[4].use_high_carrier ? 1U : 0U,
            info->rate_data[4].pre_emphasis,
            info->rate_data[4].max_bit_rate);
    appendf(buf, len, " row3429_hicar_or_v92b70=%u row3429_preemp_or_v92b71_74=%d row3429_maxrate_or_v92b75_78=%d",
            info->rate_data[5].use_high_carrier ? 1U : 0U,
            info->rate_data[5].pre_emphasis,
            info->rate_data[5].max_bit_rate);
    appendf(buf, len, " freq_offset_10b=0x%03X(%+d)",
            (unsigned) ((info->freq_offset < 0) ? (0x400 + info->freq_offset) : info->freq_offset) & 0x3FFU,
            info->freq_offset);
    appendf(buf, len, " crc_bits=89:104");
    appendf(buf, len, " crc16=unavailable");
    appendf(buf, len, " tail_bits=105:108");
    appendf(buf, len, " tail=0xF");
}

static bool snapshot_v34_info0a_from_rx_state(v34_v90_info0a_t *dst, const v34_state_t *v34)
{
    enum {
        V34_CAP_2743 = 1,
        V34_CAP_2800 = 2,
        V34_CAP_3000 = 3,
        V34_CAP_3200 = 4,
        V34_CAP_3429 = 5
    };

    if (!dst || !v34 || !v34->rx.v90_mode)
        return false;

    memset(dst, 0, sizeof(*dst));
    dst->support_2743 = v34->rx.far_capabilities.support_baud_rate_low_carrier[V34_CAP_2743];
    dst->support_2800 = v34->rx.far_capabilities.support_baud_rate_low_carrier[V34_CAP_2800];
    dst->support_3429 = v34->rx.far_capabilities.support_baud_rate_low_carrier[V34_CAP_3429];
    dst->support_3000_low = v34->rx.far_capabilities.support_baud_rate_low_carrier[V34_CAP_3000];
    dst->support_3000_high = v34->rx.far_capabilities.support_baud_rate_high_carrier[V34_CAP_3000];
    dst->support_3200_low = v34->rx.far_capabilities.support_baud_rate_low_carrier[V34_CAP_3200];
    dst->support_3200_high = v34->rx.far_capabilities.support_baud_rate_high_carrier[V34_CAP_3200];
    dst->rate_3429_allowed = v34->rx.far_capabilities.rate_3429_allowed;
    dst->support_power_reduction = v34->rx.far_capabilities.support_power_reduction;
    dst->max_baud_rate_difference = v34->rx.far_capabilities.max_baud_rate_difference;
    dst->from_cme_modem = v34->rx.far_capabilities.from_cme_modem;
    dst->support_1664_point_constellation = v34->rx.far_capabilities.support_1664_point_constellation;
    dst->tx_clock_source = v34->rx.far_capabilities.tx_clock_source;
    dst->acknowledge_info0d = v34->rx.info0_acknowledgement;
    dst->raw_26_27 = v34->rx.info0_raw_26_27;
    dst->info0d_nominal_power_code = v34->rx.info0d_nominal_power_code;
    dst->info0d_max_power_code = v34->rx.info0d_max_power_code;
    dst->info0d_power_measured_at_codec_output = v34->rx.info0d_power_measured_at_codec_output;
    dst->info0d_pcm_alaw = v34->rx.info0d_pcm_alaw;
    dst->info0d_upstream_3429_support = v34->rx.info0d_upstream_3429_support;
    dst->info0d_reserved_41 = v34->rx.info0d_reserved_41;
    dst->info0d_extensions_valid = v34->rx.info0d_extensions_valid;
    return true;
}

static bool snapshot_v34_info1a_from_rx_state(v34_v90_info1a_t *dst, const v34_state_t *v34)
{
    if (!dst || !v34 || !v34->rx.v90_mode)
        return false;

    memset(dst, 0, sizeof(*dst));
    dst->md = v34->rx.info1a.md;
    dst->u_info = v34->rx.info1a.max_data_rate;
    dst->upstream_symbol_rate_code = v34->rx.info1a.baud_rate_a_to_c;
    dst->downstream_rate_code = v34->rx.info1a.baud_rate_c_to_a;
    dst->freq_offset = v34->rx.info1a.freq_offset;
    dst->raw_12_17 = v34->rx.info1a_raw_12_17;
    dst->raw_32_33 = v34->rx.info1a_raw_32_33;
    dst->raw_40_49 = v34->rx.info1a_raw_40_49;
    return true;
}

static bool parse_v34_info0_from_rx_buf(v34_v90_info0a_t *raw_out,
                                        v90_info0a_t *mapped_out,
                                        const v34_info_collector_t *collector)
{
    v34_info_frame_t frame;

    if (!collector || !v34_info_frame_from_collector(&frame, collector))
        return false;
    return v34_info_parse_info0a_v90_frame(&frame, raw_out, mapped_out);
}

static bool parse_v34_info1a_from_rx_buf(v34_v90_info1a_t *raw_out,
                                         v90_info1a_t *mapped_out,
                                         const v34_info_collector_t *collector)
{
    v34_info_frame_t frame;

    if (!collector || !v34_info_frame_from_collector(&frame, collector))
        return false;
    return v34_info_parse_info1a_v90_frame(&frame, raw_out, mapped_out);
}

static bool parse_v34_info1d_from_rx_buf(v34_v90_info1d_t *raw_out,
                                         const v34_info_collector_t *collector)
{
    v34_info_frame_t frame;

    if (!collector || !v34_info_frame_from_collector(&frame, collector))
        return false;
    return v34_info_parse_info1d_v90_frame(&frame, raw_out);
}

static void sync_v34_info_collector_from_rx(v34_info_collector_t *collector,
                                            const v34_state_t *v34)
{
    if (!collector || !v34 || !v34->rx.v90_mode)
        return;
    v34_info_collector_load_snapshot(collector,
                                     (uint16_t) (v34->rx.bitstream & 0xFFFFU),
                                     v34->rx.crc,
                                     v34->rx.bit_count,
                                     v34->rx.target_bits,
                                     v34->rx.info_buf,
                                     (int) sizeof(v34->rx.info_buf));
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
    bool repeated_confirmed;
    int preamble_type;
    int sample_offset;
    int byte_len;
    int score;
    int repeat_count;
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

#define V8_MAX_TARGETED_BURSTS 8

typedef struct {
    bool seen;
    int burst_count;
    int best_burst_index;
    int best_preamble_type;
    int best_score;
    int best_ones_score;
    int best_sync_score;
    int best_bit_rate;
    int best_invert;
    int best_phase_q;
    int best_bit_count;
    int best_byte_len;
    int best_sample_offset;
    int repeat_count;
    bool repeated_confirmed;
    uint8_t best_bytes[16];
} v8_targeted_diag_t;

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
    v8_raw_msg_hit_t last_cm_jm;
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
    bool cm_jm_complete;
    bool cm_jm_salvaged;
    int ansam_sample;
    int ansam_tone;
    int ct_sample;
    int cng_sample;
    int cm_jm_raw_len;
    uint8_t cm_jm_raw[64];
    int ci_sample;
    int cm_jm_sample;
    int cj_sample;
    int v8_call_sample;
    int last_status;
    v8_parms_t result;
} v8_probe_result_t;

typedef struct {
    bool ok;
    bool left_is_caller;
    v8_probe_result_t left_probe;
    v8_probe_result_t right_probe;
    int score;
} v8_stereo_pair_result_t;

static bool v8_select_best_probe(const int16_t *samples,
                                 int total_samples,
                                 bool calling_party,
                                 int max_sample,
                                 v8_probe_result_t *out);

static bool v8_probe_has_cm_jm_capabilities(const v8_probe_result_t *probe)
{
    return probe && probe->ok && probe->cm_jm_complete && probe->cm_jm_sample >= 0;
}

static bool v8_probe_allows_v90_v92_digital(const v8_probe_result_t *probe)
{
    int mods;
    int pcm;

    if (!v8_probe_has_cm_jm_capabilities(probe))
        return true;

    mods = probe->result.jm_cm.modulations;
    pcm = probe->result.jm_cm.pcm_modem_availability;

    if ((mods & (V8_MOD_V90 | V8_MOD_V92)) == 0)
        return false;
    if ((pcm & V8_PSTN_PCM_MODEM_V90_V92_DIGITAL) == 0)
        return false;
    return true;
}

static bool v8_result_has_analog_only_pcm(const v8_parms_t *r)
{
    if (!r)
        return false;
    return (r->jm_cm.pcm_modem_availability & V8_PSTN_PCM_MODEM_V90_V92_ANALOGUE) != 0
        && (r->jm_cm.pcm_modem_availability & V8_PSTN_PCM_MODEM_V90_V92_DIGITAL) == 0;
}

static bool v8_result_looks_like_v34_relay(const v8_parms_t *r)
{
    if (!r)
        return false;
    return (r->jm_cm.modulations & V8_MOD_V34) != 0
        && v8_result_has_analog_only_pcm(r);
}

static bool v8_samples_near(int a, int b, int max_delta)
{
    int delta;

    if (a < 0 || b < 0)
        return false;
    delta = a - b;
    if (delta < 0)
        delta = -delta;
    return delta <= max_delta;
}

static void print_v8_capability_note(const v8_parms_t *r)
{
    if (!r)
        return;
    if (v8_result_has_analog_only_pcm(r)
        && (r->jm_cm.modulations & (V8_MOD_V90 | V8_MOD_V92)) != 0) {
        printf("  Capability note: V.90/V.92 is advertised without digital PCM; treat this as analogue-side endpoint capability, not a digital V.90/V.92 path.\n");
    }
}

#define V8_EARLY_SEARCH_LIMIT_SAMPLES   ((8000 * 14) / 1)

typedef struct {
    bool valid;
    const int16_t *samples;
    int total_samples;
    int max_sample;
    v8_probe_result_t probe;
} v8_probe_cache_entry_t;

#define VPCM_PREPASS_CACHE_SLOTS 8

static v8_probe_cache_entry_t g_v8_probe_cache[VPCM_PREPASS_CACHE_SLOTS];
static int g_v8_probe_cache_next = 0;
static const v34_offline_decode_config_t g_v34_offline_decode_config = {
    .enable_tx = false,
};
static bool decode_v34_pass_mode(const int16_t *samples,
                                 int total_samples,
                                 v91_law_t law,
                                 bool calling_party,
                                 float initial_signal_cutoff_db,
                                 bool allow_info_rate_infer,
                                 const v34_offline_decode_config_t *config,
                                 bool phase2_only,
                                 decode_v34_result_t *result);

static bool decode_v34_pass(const int16_t *samples,
                            int total_samples,
                            v91_law_t law,
                            bool calling_party,
                            float info_db_cutoff,
                            bool allow_info_rate_infer,
                            const v34_offline_decode_config_t *config,
                            decode_v34_result_t *result);
static bool decode_v34_phase2_only_pass(const int16_t *samples,
                                        int total_samples,
                                        v91_law_t law,
                                        bool calling_party,
                                        float info_db_cutoff,
                                        bool allow_info_rate_infer,
                                        const v34_offline_decode_config_t *config,
                                        decode_v34_result_t *result);

static bool decode_v34_pass_bridge(void *ctx,
                                   const int16_t *samples,
                                   int total_samples,
                                   v91_law_t law,
                                   bool calling_party,
                                   float info_db_cutoff,
                                   bool allow_info_rate_infer,
                                   decode_v34_result_t *result)
{
    const v34_offline_decode_config_t *config = (const v34_offline_decode_config_t *) ctx;

    return decode_v34_pass(samples,
                           total_samples,
                           law,
                           calling_party,
                           info_db_cutoff,
                           allow_info_rate_infer,
                           config,
                           result);
}

static bool decode_v34_phase2_only_pass_bridge(void *ctx,
                                               const int16_t *samples,
                                               int total_samples,
                                               v91_law_t law,
                                               bool calling_party,
                                               float info_db_cutoff,
                                               bool allow_info_rate_infer,
                                               decode_v34_result_t *result)
{
    const v34_offline_decode_config_t *config = (const v34_offline_decode_config_t *) ctx;

    return decode_v34_phase2_only_pass(samples,
                                       total_samples,
                                       law,
                                       calling_party,
                                       info_db_cutoff,
                                       allow_info_rate_infer,
                                       config,
                                       result);
}

static v34_phase2_engine_t g_v34_phase2_engine = {
    .decode_pass = decode_v34_pass_bridge,
    .decode_pass_ctx = (void *) &g_v34_offline_decode_config,
    .decode_phase2_pass = decode_v34_phase2_only_pass_bridge,
    .decode_phase2_pass_ctx = (void *) &g_v34_offline_decode_config,
};

typedef struct {
    bool adaptive_used;
    double gain;
    int bias;
    int direct_v90_score;
    int selected_v90_score;
} codeword_stream_info_t;

static int v34_mp_rate_n_to_bps(int rate_n)
{
    if (rate_n < 1 || rate_n > 14)
        return -1;
    return rate_n * 2400;
}

static int v34_symbol_rate_code_to_baud(int code)
{
    static const int baud_by_code[6] = {2400, 2743, 2800, 3000, 3200, 3429};

    if (code >= 0 && code < 6)
        return baud_by_code[code];
    if (code == 6)
        return 8000; /* V.90 PCM downstream code */
    return -1;
}

static int v34_symbol_rate_code_to_theoretical_max_bps(int code)
{
    static const int max_by_code[6] = {21600, 26400, 26400, 28800, 31200, 33600};

    if (code < 0 || code >= 6)
        return -1;
    return max_by_code[code];
}

static bool v34_infer_mp_rates_from_info_sequences(const decode_v34_result_t *result,
                                                   int *a_to_c_bps_out,
                                                   int *c_to_a_bps_out)
{
    int a_to_c = -1;
    int c_to_a = -1;

    if (!result || !a_to_c_bps_out || !c_to_a_bps_out)
        return false;

    if (result->info1_seen && !result->info1_is_d) {
        int up_code = result->info1a.upstream_symbol_rate_code;
        int dn_code = result->info1a.downstream_rate_code;

        a_to_c = v34_symbol_rate_code_to_theoretical_max_bps(up_code);
        c_to_a = v34_symbol_rate_code_to_theoretical_max_bps(dn_code);
        if (a_to_c > 0 && c_to_a < 0)
            c_to_a = a_to_c;
        if (c_to_a > 0 && a_to_c < 0)
            a_to_c = c_to_a;
    } else if (result->info1_seen && result->info1_is_d) {
        int best_n = -1;

        for (int i = 0; i < 6; i++) {
            if (result->info1d.rate_data[i].max_bit_rate > best_n)
                best_n = result->info1d.rate_data[i].max_bit_rate;
        }
        if (best_n > 0)
            a_to_c = c_to_a = v34_mp_rate_n_to_bps(best_n);
    }

    if (a_to_c <= 0 || c_to_a <= 0)
        return false;

    *a_to_c_bps_out = a_to_c;
    *c_to_a_bps_out = c_to_a;
    return true;
}

static const char *v34_mp_trellis_name(int trellis_size)
{
    switch (trellis_size) {
    case 0: return "16-state";
    case 1: return "32-state";
    case 2: return "64-state";
    default: return "reserved";
    }
}

static void v34_mp_rate_mask_to_text(int mask, char *out, size_t out_len)
{
    if (!out || out_len == 0)
        return;
    out[0] = '\0';
    for (int i = 0; i < 14; i++) {
        if ((mask & (1 << i)) == 0)
            continue;
        appendf(out, out_len, "%s%d", out[0] ? "," : "", (i + 1) * 2400);
    }
    if (!out[0])
        appendf(out, out_len, "none");
}

static bool v34_parse_mp_from_raw_bits(const uint8_t frame_bits[188],
                                       int frame_target,
                                       mp_t *mp_out,
                                       int *bit_count_out)
{
    bitstream_state_t bs;
    uint8_t packed[25];
    uint8_t *t;
    const uint8_t *r;
    int type;
    int limit;

    if (!frame_bits || !mp_out)
        return false;

    type = frame_bits[18] & 1;
    limit = (type == 1) ? 188 : 88;
    if (frame_target == 88 || frame_target == 188) {
        if (frame_target < limit)
            return false;
    }

    memset(packed, 0, sizeof(packed));
    bitstream_init(&bs, true);
    t = packed;
    for (int i = 18; i < limit; i++)
        bitstream_put(&bs, &t, frame_bits[i] & 1, 1);
    bitstream_flush(&bs, &t);

    memset(mp_out, 0, sizeof(*mp_out));
    bitstream_init(&bs, true);
    r = packed;
    mp_out->type = bitstream_get(&bs, &r, 1);
    bitstream_get(&bs, &r, 1); /* reserved bit 19 */
    mp_out->bit_rate_c_to_a = bitstream_get(&bs, &r, 4);
    mp_out->bit_rate_a_to_c = bitstream_get(&bs, &r, 4);
    mp_out->aux_channel_supported = bitstream_get(&bs, &r, 1);
    mp_out->trellis_size = bitstream_get(&bs, &r, 2);
    mp_out->use_non_linear_encoder = bitstream_get(&bs, &r, 1);
    mp_out->expanded_shaping = bitstream_get(&bs, &r, 1);
    mp_out->mp_acknowledged = bitstream_get(&bs, &r, 1);
    bitstream_get(&bs, &r, 1); /* start bit 34 */
    mp_out->signalling_rate_mask = bitstream_get(&bs, &r, 15);
    mp_out->asymmetric_rates_allowed = bitstream_get(&bs, &r, 1);
    if (mp_out->type == 1) {
        for (int i = 0; i < 3; i++) {
            bitstream_get(&bs, &r, 1);
            mp_out->precoder_coeffs[i].re = bitstream_get(&bs, &r, 16);
            bitstream_get(&bs, &r, 1);
            mp_out->precoder_coeffs[i].im = bitstream_get(&bs, &r, 16);
        }
    }

    if (mp_out->type != type)
        return false;
    if (mp_out->bit_rate_a_to_c < 1 || mp_out->bit_rate_a_to_c > 14)
        return false;
    if (mp_out->bit_rate_c_to_a < 1 || mp_out->bit_rate_c_to_a > 14)
        return false;
    if (mp_out->trellis_size < 0 || mp_out->trellis_size > 2)
        return false;

    if (bit_count_out)
        *bit_count_out = limit;
    return true;
}

static bool v34_parse_mp_from_frame_bits(const uint8_t frame_bits[188],
                                         int mp_seen,
                                         int frame_target,
                                         mp_t *mp_out,
                                         int *bit_count_out)
{
    if (mp_seen < 1)
        return false;
    return v34_parse_mp_from_raw_bits(frame_bits,
                                      frame_target,
                                      mp_out,
                                      bit_count_out);
}

static uint16_t v34_mp_crc_bits(const uint8_t bits[188], int type)
{
    uint16_t crc = 0xFFFF;
    int len = (type == 1) ? 170 : 68;

    for (int i = 17; i < len; i += 17) {
        for (int j = 0; j < 16; j++)
            crc = crc_itu16_bits(bits[i + 1 + j] & 1U, 1, crc);
    }
    return crc;
}

static bool v34_mp_crc_ok(const uint8_t bits[188], int type)
{
    int crc_start = (type == 1) ? 171 : 69;
    uint16_t crc = v34_mp_crc_bits(bits, type);

    for (int i = 0; i < 16; i++)
        crc = crc_itu16_bits(bits[crc_start + i] & 1U, 1, crc);
    return (crc == 0);
}

static bool v34_mp_fill_ok(const uint8_t bits[188], int type)
{
    if (type == 1)
        return (bits[187] & 1U) == 0;
    return (((bits[85] | bits[86] | bits[87]) & 1U) == 0);
}

static int v34_mp_start_error_count(const uint8_t bits[188], int type, int target)
{
    int errs = 0;
    int limit = target;

    if (type == 0 && (limit <= 0 || limit > 88))
        limit = 88;
    else if (type == 1 && (limit <= 0 || limit > 188))
        limit = 188;

    for (int i = 17; i < limit; i++) {
        bool is_start = (i == 17 || i == 34 || i == 51 || i == 68);
        if (type == 1) {
            if (i == 85 || i == 102 || i == 119 || i == 136 || i == 153 || i == 170)
                is_start = true;
        }
        if (is_start && (bits[i] & 1U) != 0)
            errs++;
    }
    return errs;
}

static bool v34_mp_semantic_ok_strict(const mp_t *mp)
{
    int mask;
    int a_bit;
    int c_bit;

    if (!mp)
        return false;
    if (mp->type != 0 && mp->type != 1)
        return false;
    if (mp->bit_rate_a_to_c < 1 || mp->bit_rate_a_to_c > 14)
        return false;
    if (mp->bit_rate_c_to_a < 1 || mp->bit_rate_c_to_a > 14)
        return false;
    if (mp->trellis_size < 0 || mp->trellis_size > 2)
        return false;

    mask = mp->signalling_rate_mask & 0x3FFF;
    if (mask == 0)
        return false;

    a_bit = 1 << (mp->bit_rate_a_to_c - 1);
    c_bit = 1 << (mp->bit_rate_c_to_a - 1);
    if ((mask & a_bit) == 0 || (mask & c_bit) == 0)
        return false;

    return true;
}

static void append_v34_mp_detail_fields(char *detail, size_t detail_len, const decode_v34_result_t *result)
{
    if (!detail || detail_len == 0 || !result)
        return;

    appendf(detail, detail_len, " mp_seen=%u", result->mp_seen ? 1U : 0U);
    appendf(detail, detail_len, " mp_remote_ack=%u", result->mp_remote_ack_seen ? 1U : 0U);
    appendf(detail, detail_len, " mp_source=%s",
        result->mp_from_rx_recovery ? "rx_frame_recovered"
            : (result->mp_from_rx_frame ? "rx_frame"
               : (result->mp_from_info_sequence ? "info_sequence" : "tx_state")));
    appendf(detail, detail_len, " mp_rate_a_to_c=%s", result->mp_rate_a_to_c_bps > 0 ? "" : "n/a");
    if (result->mp_rate_a_to_c_bps > 0)
        appendf(detail, detail_len, "%d", result->mp_rate_a_to_c_bps);
    appendf(detail, detail_len, " mp_rate_c_to_a=%s", result->mp_rate_c_to_a_bps > 0 ? "" : "n/a");
    if (result->mp_rate_c_to_a_bps > 0)
        appendf(detail, detail_len, "%d", result->mp_rate_c_to_a_bps);
    appendf(detail, detail_len, " mp_type=%s", result->mp_type >= 0 ? "" : "n/a");
    if (result->mp_type >= 0)
        appendf(detail, detail_len, "%d", result->mp_type);
    appendf(detail, detail_len, " mp_trellis=%s", result->mp_trellis_size >= 0 ? "" : "n/a");
    if (result->mp_trellis_size >= 0)
        appendf(detail, detail_len, "%d", result->mp_trellis_size);
    appendf(detail, detail_len, " mp_aux=%u", result->mp_aux_channel_supported ? 1U : 0U);
    appendf(detail, detail_len, " mp_q3125=%u", result->mp_use_non_linear_encoder ? 1U : 0U);
    appendf(detail, detail_len, " mp_expanded_shaping=%u", result->mp_expanded_shaping ? 1U : 0U);
    appendf(detail, detail_len, " mp_asymmetric=%u", result->mp_asymmetric_rates_allowed ? 1U : 0U);
    appendf(detail, detail_len, " mp_local_ack_bit=%u", result->mp_local_ack_bit ? 1U : 0U);
    appendf(detail, detail_len, " mp_mask=0x%04X", (unsigned) (result->mp_signalling_rate_mask & 0x7FFF));
    appendf(detail, detail_len, " mp_frame_bits=%d", result->mp_frame_bits_captured);
    appendf(detail, detail_len, " mp_crc=%u", result->mp_crc_valid ? 1U : 0U);
    appendf(detail, detail_len, " mp_fill=%u", result->mp_fill_valid ? 1U : 0U);
    appendf(detail, detail_len, " mp_start_err=%d", result->mp_start_error_count);
    appendf(detail, detail_len, " mp_rates_valid=%u", result->mp_rates_valid ? 1U : 0U);
    appendf(detail, detail_len, " mp_candidate=%u", result->mp_candidate_seen ? 1U : 0U);
    if (result->mp_candidate_seen) {
        appendf(detail, detail_len, " mp_candidate_votes=%d", result->mp_candidate_votes);
        appendf(detail, detail_len, " mp_candidate_crc=%u", result->mp_candidate_crc_valid ? 1U : 0U);
        appendf(detail, detail_len, " mp_candidate_fill=%u", result->mp_candidate_fill_valid ? 1U : 0U);
        appendf(detail, detail_len, " mp_candidate_start_err=%d", result->mp_candidate_start_error_count);
        appendf(detail, detail_len, " mp_candidate_rate_a_to_c=%d", result->mp_candidate_rate_a_to_c_bps);
        appendf(detail, detail_len, " mp_candidate_rate_c_to_a=%d", result->mp_candidate_rate_c_to_a_bps);
        appendf(detail, detail_len, " mp_candidate_type=%d", result->mp_candidate_type);
        appendf(detail, detail_len, " mp_candidate_trellis=%d", result->mp_candidate_trellis_size);
        appendf(detail, detail_len, " mp_candidate_mask=0x%04X",
                (unsigned) (result->mp_candidate_signalling_rate_mask & 0x7FFF));
    }
}

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

/* ja_dil_decode_t is now defined in v92_ja_decode.h */

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

typedef struct {
    char key[64];
    char value[128];
} detail_kv_t;

static void append_html_escaped(char *out, size_t out_len, const char *text)
{
    const unsigned char *p = (const unsigned char *) text;

    if (!out || out_len == 0)
        return;
    if (!text)
        return;
    for (; *p; p++) {
        switch (*p) {
        case '&': appendf(out, out_len, "&amp;"); break;
        case '<': appendf(out, out_len, "&lt;"); break;
        case '>': appendf(out, out_len, "&gt;"); break;
        case '"': appendf(out, out_len, "&quot;"); break;
        default:
            if (*p >= 0x20)
                appendf(out, out_len, "%c", *p);
            break;
        }
    }
}

static int parse_detail_key_values(const char *detail, detail_kv_t *pairs, int max_pairs)
{
    int count = 0;
    const char *p = detail;

    if (!detail || !pairs || max_pairs <= 0)
        return 0;

    while (*p != '\0' && count < max_pairs) {
        const char *key_start;
        const char *key_end;
        const char *value_start;
        const char *value_end;
        size_t key_len;
        size_t value_len;

        while (*p == ' ')
            p++;
        if (*p == '\0')
            break;

        key_start = p;
        while (*p != '\0' && *p != '=' && *p != ' ')
            p++;
        key_end = p;
        if (*p != '=') {
            while (*p != '\0' && *p != ' ')
                p++;
            continue;
        }
        p++;

        if (*p == '"') {
            p++;
            value_start = p;
            while (*p != '\0' && *p != '"')
                p++;
            value_end = p;
            if (*p == '"')
                p++;
        } else {
            value_start = p;
            while (*p != '\0' && *p != ' ')
                p++;
            value_end = p;
        }

        key_len = (size_t) (key_end - key_start);
        value_len = (size_t) (value_end - value_start);
        if (key_len == 0)
            continue;

        snprintf(pairs[count].key, sizeof(pairs[count].key), "%.*s", (int) key_len, key_start);
        snprintf(pairs[count].value, sizeof(pairs[count].value), "%.*s", (int) value_len, value_start);
        count++;
    }
    return count;
}

static const char *detail_value(const detail_kv_t *pairs, int count, const char *key)
{
    if (!pairs || count <= 0 || !key)
        return NULL;
    for (int i = 0; i < count; i++) {
        if (strcmp(pairs[i].key, key) == 0)
            return pairs[i].value;
    }
    return NULL;
}

static const char *detail_bit_to_yes_no(const char *value)
{
    if (!value)
        return "unknown";
    if (strcmp(value, "1") == 0 || strcasecmp(value, "yes") == 0)
        return "yes";
    if (strcmp(value, "0") == 0 || strcasecmp(value, "no") == 0)
        return "no";
    return value;
}

static void desanitize_value(char *dst, size_t dst_len, const char *src)
{
    size_t i;
    if (!dst || dst_len == 0) return;
    if (!src) { dst[0] = '\0'; return; }
    for (i = 0; i < dst_len - 1 && src[i]; i++)
        dst[i] = (src[i] == '_') ? ' ' : src[i];
    dst[i] = '\0';
}

static void append_html_label_value(char *out, size_t out_len, const char *label, const char *value)
{
    if (!out || out_len == 0 || !label)
        return;
    appendf(out, out_len, "<div><strong>");
    append_html_escaped(out, out_len, label);
    appendf(out, out_len, ":</strong> ");
    append_html_escaped(out, out_len, value ? value : "unknown");
    appendf(out, out_len, "</div>");
}

static void append_html_label_desanitized(char *out, size_t out_len, const char *label, const char *value)
{
    char clean[512];
    if (!value) {
        append_html_label_value(out, out_len, label, NULL);
        return;
    }
    desanitize_value(clean, sizeof(clean), value);
    append_html_label_value(out, out_len, label, clean);
}

static bool detail_value_present(const char *value)
{
    return value && value[0] != '\0' && strcmp(value, "n/a") != 0 && strcmp(value, "unknown") != 0;
}

static void append_html_label_if_present(char *out, size_t out_len, const char *label, const char *value)
{
    if (detail_value_present(value))
        append_html_label_value(out, out_len, label, value);
}

static const char *v34_mp_source_label(const char *source)
{
    if (!source)
        return "unknown";
    if (strcmp(source, "rx_frame_recovered") == 0)
        return "Recovered RX MP frame";
    if (strcmp(source, "rx_frame") == 0)
        return "RX MP frame";
    if (strcmp(source, "info_sequence") == 0)
        return "INFO-sequence inferred";
    if (strcmp(source, "tx_state") == 0)
        return "TX-state inferred";
    return source;
}

static const char *v34_mp_trellis_label_from_detail(const char *value)
{
    if (!value || !*value)
        return "unknown";
    if (strcmp(value, "0") == 0)
        return "16-state";
    if (strcmp(value, "1") == 0)
        return "32-state";
    if (strcmp(value, "2") == 0)
        return "64-state";
    return value;
}

static void append_v34_mp_html_section(char *out,
                                       size_t out_len,
                                       detail_kv_t *pairs,
                                       int pair_count,
                                       bool include_candidate)
{
    const char *mp_seen = detail_value(pairs, pair_count, "mp_seen");
    const char *mp_candidate = detail_value(pairs, pair_count, "mp_candidate");

    if (!out || out_len == 0 || !pairs || pair_count <= 0)
        return;

    if (!(mp_seen && strcmp(mp_seen, "1") == 0)
        && !(include_candidate && mp_candidate && strcmp(mp_candidate, "1") == 0))
        return;

    appendf(out, out_len, "<hr><div><strong>Phase 4 MP:</strong></div>");
    append_html_label_value(out, out_len, "MP seen", detail_bit_to_yes_no(mp_seen));
    append_html_label_value(out, out_len, "Remote ACK", detail_bit_to_yes_no(detail_value(pairs, pair_count, "mp_remote_ack")));
    append_html_label_value(out, out_len, "Decode source", v34_mp_source_label(detail_value(pairs, pair_count, "mp_source")));
    append_html_label_if_present(out, out_len, "Negotiated A->C rate", detail_value(pairs, pair_count, "mp_rate_a_to_c"));
    append_html_label_if_present(out, out_len, "Negotiated C->A rate", detail_value(pairs, pair_count, "mp_rate_c_to_a"));
    append_html_label_if_present(out, out_len, "MP type", detail_value(pairs, pair_count, "mp_type"));
    append_html_label_if_present(out, out_len, "Trellis", v34_mp_trellis_label_from_detail(detail_value(pairs, pair_count, "mp_trellis")));
    append_html_label_value(out, out_len, "Aux channel", detail_bit_to_yes_no(detail_value(pairs, pair_count, "mp_aux")));
    append_html_label_value(out, out_len, "Non-linear encoder", detail_bit_to_yes_no(detail_value(pairs, pair_count, "mp_q3125")));
    append_html_label_value(out, out_len, "Shaping", strcmp(detail_value(pairs, pair_count, "mp_expanded_shaping"), "1") == 0 ? "Expanded" : "Minimal");
    append_html_label_value(out, out_len, "Asymmetric rates", detail_bit_to_yes_no(detail_value(pairs, pair_count, "mp_asymmetric")));
    append_html_label_value(out, out_len, "Local ACK bit", detail_bit_to_yes_no(detail_value(pairs, pair_count, "mp_local_ack_bit")));
    append_html_label_if_present(out, out_len, "Signalling-rate mask", detail_value(pairs, pair_count, "mp_mask"));
    append_html_label_if_present(out, out_len, "Frame bits captured", detail_value(pairs, pair_count, "mp_frame_bits"));
    append_html_label_value(out, out_len, "CRC valid", detail_bit_to_yes_no(detail_value(pairs, pair_count, "mp_crc")));
    append_html_label_value(out, out_len, "Fill valid", detail_bit_to_yes_no(detail_value(pairs, pair_count, "mp_fill")));
    append_html_label_if_present(out, out_len, "Frame-start errors", detail_value(pairs, pair_count, "mp_start_err"));
    append_html_label_value(out, out_len, "Rates valid", detail_bit_to_yes_no(detail_value(pairs, pair_count, "mp_rates_valid")));

    if (include_candidate && mp_candidate && strcmp(mp_candidate, "1") == 0) {
        appendf(out, out_len, "<hr><div><strong>Fallback MP Candidate:</strong></div>");
        append_html_label_if_present(out, out_len, "Votes", detail_value(pairs, pair_count, "mp_candidate_votes"));
        append_html_label_if_present(out, out_len, "Candidate A->C rate", detail_value(pairs, pair_count, "mp_candidate_rate_a_to_c"));
        append_html_label_if_present(out, out_len, "Candidate C->A rate", detail_value(pairs, pair_count, "mp_candidate_rate_c_to_a"));
        append_html_label_if_present(out, out_len, "Candidate type", detail_value(pairs, pair_count, "mp_candidate_type"));
        append_html_label_if_present(out, out_len, "Candidate trellis", v34_mp_trellis_label_from_detail(detail_value(pairs, pair_count, "mp_candidate_trellis")));
        append_html_label_if_present(out, out_len, "Candidate mask", detail_value(pairs, pair_count, "mp_candidate_mask"));
        append_html_label_value(out, out_len, "Candidate CRC valid", detail_bit_to_yes_no(detail_value(pairs, pair_count, "mp_candidate_crc")));
        append_html_label_value(out, out_len, "Candidate fill valid", detail_bit_to_yes_no(detail_value(pairs, pair_count, "mp_candidate_fill")));
        append_html_label_if_present(out, out_len, "Candidate frame-start errors", detail_value(pairs, pair_count, "mp_candidate_start_err"));
    }
}

static void build_visual_event_detail_html(const call_log_event_t *event, char *out, size_t out_len)
{
    detail_kv_t pairs[128];
    int pair_count;

    if (!out || out_len == 0) {
        return;
    }
    out[0] = '\0';
    if (!event) {
        return;
    }

    pair_count = parse_detail_key_values(event->detail, pairs, (int) (sizeof(pairs) / sizeof(pairs[0])));
    if (strcmp(event->protocol, "V.90/V.92") == 0
        && strncmp(event->summary, "Short Phase 2 sequence", 22) == 0
        && pair_count > 0) {
        const char *seq = detail_value(pairs, pair_count, "sequence");
        bool is_seq_a = (seq && strcmp(seq, "A") == 0);
        appendf(out, out_len, "<div class=\"detail-kv\">");
        append_html_label_value(out, out_len, "Role", detail_value(pairs, pair_count, "role"));
        append_html_label_value(out, out_len, "Figure", detail_value(pairs, pair_count, "figure"));
        append_html_label_value(out, out_len, "Sequence", detail_value(pairs, pair_count, "sequence"));
        append_html_label_value(out, out_len, "Local modem", detail_value(pairs, pair_count, "local_modem"));
        append_html_label_value(out, out_len, "Short Phase 2 requested", detail_bit_to_yes_no(detail_value(pairs, pair_count, "shortp2_req")));
        append_html_label_value(out, out_len, "V.92 capable", detail_bit_to_yes_no(detail_value(pairs, pair_count, "v92_cap")));
        append_html_label_value(out, out_len, "INFO0 decoded at", detail_value(pairs, pair_count, "info0_ms"));
        append_html_label_value(out, out_len, "TX tone start", detail_value(pairs, pair_count, "tx_tone_ms"));
        append_html_label_value(out, out_len, "TX tone reversal", detail_value(pairs, pair_count, "tx_reversal_ms"));
        append_html_label_value(out, out_len, "RX far-end tone", detail_value(pairs, pair_count, "rx_tone_ms"));
        append_html_label_value(out, out_len, "RX far-end reversal", detail_value(pairs, pair_count, "rx_reversal_ms"));
        append_html_label_value(out, out_len, "TX INFO1 start", detail_value(pairs, pair_count, "tx_info1_ms"));
        append_html_label_value(out, out_len, "INFO1 seen", detail_bit_to_yes_no(detail_value(pairs, pair_count, "info1_seen")));
        append_html_label_value(out, out_len, "INFO1 decoded at", detail_value(pairs, pair_count, "info1_ms"));
        append_html_label_value(out, out_len, "Sequence status", detail_value(pairs, pair_count, "status"));
        appendf(out, out_len, "<hr><div><strong>Figure 9 State Machine:</strong></div>");
        if (is_seq_a) {
            append_html_label_value(out, out_len, "9.4.2.1.1 INFO0d received / Tone A sent", detail_bit_to_yes_no(detail_value(pairs, pair_count, "c94211")));
            append_html_label_value(out, out_len, "9.4.2.1.2 Tone B detected while receiving INFO0d", detail_bit_to_yes_no(detail_value(pairs, pair_count, "c94212")));
            append_html_label_value(out, out_len, "9.4.2.1.3 Tone B reversal then Tone A reversal sent", detail_bit_to_yes_no(detail_value(pairs, pair_count, "c94213")));
            append_html_label_value(out, out_len, "9.4.2.1.4 INFO1a sent", detail_bit_to_yes_no(detail_value(pairs, pair_count, "c94214")));
            appendf(out, out_len, "<hr><div><strong>Recovery (9.4.2.2):</strong></div>");
            append_html_label_value(out, out_len, "9.4.2.2.1 repeated INFO0 handling observed", detail_bit_to_yes_no(detail_value(pairs, pair_count, "c94221")));
            append_html_label_value(out, out_len, "9.4.2.2.2 retrain/timeout observed", detail_bit_to_yes_no(detail_value(pairs, pair_count, "c94222")));
        } else {
            append_html_label_value(out, out_len, "9.4.1.1.1 INFO0d sent / Tone B sent", detail_bit_to_yes_no(detail_value(pairs, pair_count, "c94111")));
            append_html_label_value(out, out_len, "9.4.1.1.2 INFO0a received / Tone A detected", detail_bit_to_yes_no(detail_value(pairs, pair_count, "c94112")));
            append_html_label_value(out, out_len, "9.4.1.1.3 Tone B phase reversal sent", detail_bit_to_yes_no(detail_value(pairs, pair_count, "c94113")));
            append_html_label_value(out, out_len, "9.4.1.1.4 Tone A phase reversal detected", detail_bit_to_yes_no(detail_value(pairs, pair_count, "c94114")));
            append_html_label_value(out, out_len, "9.4.1.1.5 INFO1a received", detail_bit_to_yes_no(detail_value(pairs, pair_count, "c94115")));
            appendf(out, out_len, "<hr><div><strong>Recovery (9.4.1.2):</strong></div>");
            append_html_label_value(out, out_len, "9.4.1.2.1 repeated INFO0 handling observed", detail_bit_to_yes_no(detail_value(pairs, pair_count, "c94121")));
            append_html_label_value(out, out_len, "9.4.1.2.2 fallback to full Phase 2 observed", detail_bit_to_yes_no(detail_value(pairs, pair_count, "c94122")));
            append_html_label_value(out, out_len, "9.4.1.2.3 INFO1a timeout fallback observed", detail_bit_to_yes_no(detail_value(pairs, pair_count, "c94123")));
        }
        appendf(out, out_len, "</div>");
        return;
    }
    if (strcmp(event->protocol, "V.92 Phase 3") == 0
        && strcmp(event->summary, "Phase 3 sequence from Ru") == 0
        && pair_count > 0) {
        appendf(out, out_len, "<div class=\"detail-kv\">");
        append_html_label_value(out, out_len, "Role", detail_value(pairs, pair_count, "role"));
        append_html_label_value(out, out_len, "Spec profile", detail_value(pairs, pair_count, "spec"));
        append_html_label_value(out, out_len, "Local modem", detail_value(pairs, pair_count, "local_modem"));
        append_html_label_value(out, out_len, "Short Phase 2 requested", detail_bit_to_yes_no(detail_value(pairs, pair_count, "shortp2_req")));
        append_html_label_value(out, out_len, "V.92 capable", detail_bit_to_yes_no(detail_value(pairs, pair_count, "v92_cap")));
        append_html_label_value(out, out_len, "Ru source", detail_value(pairs, pair_count, "ru_source"));
        append_html_label_value(out, out_len, "Ru start", detail_value(pairs, pair_count, "ru_ms"));
        append_html_label_value(out, out_len, "Ru pattern", detail_value(pairs, pair_count, "ru_pattern_primary"));
        append_html_label_value(out, out_len, "uR complement pattern", detail_value(pairs, pair_count, "ru_pattern_complement"));
        append_html_label_value(out, out_len, "Ru precoder bypass expected", detail_bit_to_yes_no(detail_value(pairs, pair_count, "ru_precoder_bypass")));
        append_html_label_value(out, out_len, "Ru prefilter bypass expected", detail_bit_to_yes_no(detail_value(pairs, pair_count, "ru_prefilter_bypass")));
        append_html_label_value(out, out_len, "Ru uses TRN1u structure", detail_bit_to_yes_no(detail_value(pairs, pair_count, "ru_trn1u_structure")));
        append_html_label_value(out, out_len, "Ru transmitter", detail_value(pairs, pair_count, "ru_tx_modem"));
        append_html_label_value(out, out_len, "Digital quiet before Sd", detail_bit_to_yes_no(detail_value(pairs, pair_count, "digital_quiet_pre_sd")));
        append_html_label_value(out, out_len, "First digital signal is Sd during Ja", detail_bit_to_yes_no(detail_value(pairs, pair_count, "first_digital_signal_sd_ja")));
        append_html_label_value(out, out_len, "Ru duration (T)", detail_value(pairs, pair_count, "ru_t"));
        append_html_label_value(out, out_len, "uR duration (T)", detail_value(pairs, pair_count, "ur_t"));
        append_html_label_value(out, out_len, "Ru+uR cycle (T)", detail_value(pairs, pair_count, "ru_ur_cycle_t"));
        append_html_label_value(out, out_len, "Ru/uR repeat after MD expected", detail_bit_to_yes_no(detail_value(pairs, pair_count, "repeats_after_md_expected")));
        append_html_label_value(out, out_len, "Ru/uR repeat after MD", detail_value(pairs, pair_count, "repeats_after_md_status"));
        append_html_label_value(out, out_len, "Ru symbol window size", detail_value(pairs, pair_count, "ru_window_symbols"));
        append_html_label_value(out, out_len, "Ru 2-point window score", detail_value(pairs, pair_count, "ru_window_score"));
        append_html_label_value(out, out_len, "TRN1U mean symbol magnitude", detail_value(pairs, pair_count, "trn1u_mag_mean"));
        append_html_label_value(out, out_len, "TRN1U magnitude sample count", detail_value(pairs, pair_count, "trn1u_mag_count"));
        append_html_label_value(out, out_len, "Ru decoded from symbols", detail_bit_to_yes_no(detail_value(pairs, pair_count, "ru_decoded")));
        append_html_label_value(out, out_len, "Ru pattern match", detail_bit_to_yes_no(detail_value(pairs, pair_count, "ru_match")));
        append_html_label_value(out, out_len, "Ru pattern match percent", detail_value(pairs, pair_count, "ru_match_pct"));
        append_html_label_value(out, out_len, "Ru symbol pair (A/B)", detail_value(pairs, pair_count, "ru_symbols"));
        append_html_label_value(out, out_len, "LU definition", detail_value(pairs, pair_count, "lu_definition"));
        append_html_label_value(out, out_len, "LU absolute level", detail_value(pairs, pair_count, "lu_absolute_level"));
        append_html_label_value(out, out_len, "LU reference", detail_value(pairs, pair_count, "lu_reference"));
        append_html_label_value(out, out_len, "Ru/LU consistency with TRN1U", detail_value(pairs, pair_count, "ru_lu_consistency"));
        append_html_label_value(out, out_len, "Ru/LU to TRN1U ratio", detail_value(pairs, pair_count, "ru_lu_ratio"));
        appendf(out, out_len, "<hr><div><strong>Ja Sequencing (9.5.1.1.3):</strong></div>");
        append_html_label_value(out, out_len, "TRNlu to Ja (T)", detail_value(pairs, pair_count, "trn_to_ja_t"));
        append_html_label_value(out, out_len, "Ja timing anchor", detail_value(pairs, pair_count, "ja_anchor_source"));
        append_html_label_value(out, out_len, "Ja aux bits (local)", detail_value(pairs, pair_count, "ja_aux_bits_local"));
        append_html_label_value(out, out_len, "Ja aux bits (peer)", detail_value(pairs, pair_count, "ja_aux_bits_peer"));
        append_html_label_value(out, out_len, "TRNlu first-2040T requirement", detail_value(pairs, pair_count, "trn_first_2040t"));
        append_html_label_value(out, out_len, "TRNlu 2040T reached before Ja", detail_bit_to_yes_no(detail_value(pairs, pair_count, "trn_2040t_ready")));
        append_html_label_value(out, out_len, "Ja start", detail_value(pairs, pair_count, "ja_ms"));
        append_html_label_value(out, out_len, "Ja DIL descriptor decoded", detail_bit_to_yes_no(detail_value(pairs, pair_count, "ja_dil_seen")));
        append_html_label_value(out, out_len, "Ja DIL source", detail_value(pairs, pair_count, "ja_dil_source"));
        append_html_label_value(out, out_len, "Ja DIL start", detail_value(pairs, pair_count, "ja_dil_ms"));
        append_html_label_value(out, out_len, "Ja DIL bits", detail_value(pairs, pair_count, "ja_dil_bits"));
        append_html_label_value(out, out_len, "Ja DIL end", detail_value(pairs, pair_count, "ja_dil_end_ms"));
        append_html_label_value(out, out_len, "Ja descriptor n", detail_value(pairs, pair_count, "ja_n"));
        append_html_label_value(out, out_len, "Ja descriptor lsp", detail_value(pairs, pair_count, "ja_lsp"));
        append_html_label_value(out, out_len, "Ja descriptor ltp", detail_value(pairs, pair_count, "ja_ltp"));
        append_html_label_value(out, out_len, "Ja unique train-U", detail_value(pairs, pair_count, "ja_unique_u"));
        append_html_label_value(out, out_len, "Ja used uchords", detail_value(pairs, pair_count, "ja_uchords"));
        append_html_label_value(out, out_len, "Ja impairment score", detail_value(pairs, pair_count, "ja_impairment"));
        append_html_label_value(out, out_len, "Sd start", detail_value(pairs, pair_count, "sd_ms"));
        append_html_label_value(out, out_len, "Sd duration (T)", detail_value(pairs, pair_count, "sd_t"));
        append_html_label_value(out, out_len, "Sd repetitions", detail_value(pairs, pair_count, "sd_reps"));
        append_html_label_value(out, out_len, "dS start", detail_value(pairs, pair_count, "ds_ms"));
        append_html_label_value(out, out_len, "dS duration (T)", detail_value(pairs, pair_count, "ds_t"));
        append_html_label_value(out, out_len, "dS repetitions", detail_value(pairs, pair_count, "ds_reps"));
        append_html_label_value(out, out_len, "Sd wait after Ja DIL", detail_value(pairs, pair_count, "sd_wait_ms"));
        append_html_label_value(out, out_len, "Sd wait <= 500 ms", detail_bit_to_yes_no(detail_value(pairs, pair_count, "sd_wait_le_500ms")));
        append_html_label_value(out, out_len, "Sd matches 384T", detail_bit_to_yes_no(detail_value(pairs, pair_count, "sd_384t_ok")));
        append_html_label_value(out, out_len, "dS matches 48T", detail_bit_to_yes_no(detail_value(pairs, pair_count, "ds_48t_ok")));
        append_html_label_value(out, out_len, "Ja status", detail_value(pairs, pair_count, "ja_status"));
        append_html_label_value(out, out_len, "Phase 3 marker", detail_value(pairs, pair_count, "phase3_ms"));
        append_html_label_value(out, out_len, "Phase 4 marker", detail_value(pairs, pair_count, "phase4_ms"));
        append_html_label_value(out, out_len, "Phase 4 status", detail_value(pairs, pair_count, "phase4_status"));
        append_html_label_value(out, out_len, "Status", detail_value(pairs, pair_count, "status"));
        append_v34_mp_html_section(out, out_len, pairs, pair_count, true);
        appendf(out, out_len, "</div>");
        return;
    }
    if (strcmp(event->protocol, "V.90/V.92") == 0
        && (strcmp(event->summary, "Phase 4 / MP reached") == 0
            || strcmp(event->summary, "Phase 4 training ready") == 0)
        && pair_count > 0) {
        appendf(out, out_len, "<div class=\"detail-kv\">");
        append_html_label_value(out, out_len, "Role", detail_value(pairs, pair_count, "role"));
        append_html_label_if_present(out, out_len, "RX stage", detail_value(pairs, pair_count, "rx"));
        append_html_label_if_present(out, out_len, "TX stage", detail_value(pairs, pair_count, "tx"));
        append_html_label_if_present(out, out_len, "Event", detail_value(pairs, pair_count, "event"));
        append_v34_mp_html_section(out, out_len, pairs, pair_count, true);
        appendf(out, out_len, "</div>");
        return;
    }
    if (strcmp(event->protocol, "V.34") == 0 && strncmp(event->summary, "INFO0", 5) == 0 && pair_count > 0) {
        char rates[128] = "";
        const char *profile = detail_value(pairs, pair_count, "profile");
        bool is_info0d = false;

        if (profile && strstr(profile, "INFO0d") != NULL)
            is_info0d = true;
        if (strncmp(event->summary, "INFO0d", 6) == 0)
            is_info0d = true;

        if (strcmp(detail_bit_to_yes_no(detail_value(pairs, pair_count, "b12_2743")), "yes") == 0)
            appendf(rates, sizeof(rates), "%s2743", rates[0] ? ", " : "");
        if (strcmp(detail_bit_to_yes_no(detail_value(pairs, pair_count, "b13_2800")), "yes") == 0)
            appendf(rates, sizeof(rates), "%s2800", rates[0] ? ", " : "");
        if (strcmp(detail_bit_to_yes_no(detail_value(pairs, pair_count, "b15_3000l")), "yes") == 0
            || strcmp(detail_bit_to_yes_no(detail_value(pairs, pair_count, "b16_3000h")), "yes") == 0)
            appendf(rates, sizeof(rates), "%s3000", rates[0] ? ", " : "");
        if (strcmp(detail_bit_to_yes_no(detail_value(pairs, pair_count, "b17_3200l")), "yes") == 0
            || strcmp(detail_bit_to_yes_no(detail_value(pairs, pair_count, "b18_3200h")), "yes") == 0)
            appendf(rates, sizeof(rates), "%s3200", rates[0] ? ", " : "");
        if (strcmp(detail_bit_to_yes_no(detail_value(pairs, pair_count, "b14_3429")), "yes") == 0)
            appendf(rates, sizeof(rates), "%s3429", rates[0] ? ", " : "");
        if (!rates[0])
            snprintf(rates, sizeof(rates), "none");

        appendf(out, out_len, "<div class=\"detail-kv\">");
        append_html_label_value(out, out_len, "Role", detail_value(pairs, pair_count, "role"));
        append_html_label_value(out, out_len, "Profile", profile);
        append_html_label_value(out, out_len, "Fill/sync", detail_value(pairs, pair_count, "fsync"));
        append_html_label_value(out, out_len, "Supported symbol rates", rates);
        append_html_label_value(out, out_len, "3429 allowed", detail_bit_to_yes_no(detail_value(pairs, pair_count, "b19_3429ok")));
        append_html_label_value(out, out_len, "Power reduction", detail_bit_to_yes_no(detail_value(pairs, pair_count, "b20_pwrred")));
        append_html_label_value(out, out_len, "Max baud-rate difference code", detail_value(pairs, pair_count, "b21_23_maxbaud"));
        append_html_label_value(out, out_len, "CME modem", detail_bit_to_yes_no(detail_value(pairs, pair_count, "b24_cme")));
        append_html_label_value(out, out_len, "1664-point constellation", detail_bit_to_yes_no(detail_value(pairs, pair_count, "b25_1664pt")));
        if (is_info0d) {
            append_html_label_value(out, out_len, "Bit 26 short Phase 2 request", detail_bit_to_yes_no(detail_value(pairs, pair_count, "b26_shortp2_req")));
            append_html_label_value(out, out_len, "Bit 27 V.92 capability", detail_bit_to_yes_no(detail_value(pairs, pair_count, "b27_v92_cap")));
            append_html_label_value(out, out_len, "INFO0a acknowledged (bit 28)", detail_bit_to_yes_no(detail_value(pairs, pair_count, "b28_ack_i0a")));
            append_html_label_value(out, out_len, "Nominal TX power code bits 29:32", detail_value(pairs, pair_count, "b29_32_nom_tx_pwr_code"));
            append_html_label_value(out, out_len, "Maximum TX power code bits 33:37", detail_value(pairs, pair_count, "b33_37_max_tx_pwr_code"));
            append_html_label_value(out, out_len, "Power measured at codec output (bit 38)", detail_bit_to_yes_no(detail_value(pairs, pair_count, "b38_pwr_meas_codec_out")));
            append_html_label_value(out, out_len, "PCM law bit 39", detail_value(pairs, pair_count, "b39_pcm_law"));
            append_html_label_value(out, out_len, "V.90 upstream 3429 support bit 40", detail_value(pairs, pair_count, "b40_v90_us3429"));
            append_html_label_value(out, out_len, "Bit 41", detail_value(pairs, pair_count, "b41_reserved"));
        } else {
            append_html_label_value(out, out_len, "Bit 26 V.92 capability", detail_bit_to_yes_no(detail_value(pairs, pair_count, "b26_v92_cap")));
            append_html_label_value(out, out_len, "Bit 27 short Phase 2 request", detail_bit_to_yes_no(detail_value(pairs, pair_count, "b27_shortp2_req")));
            append_html_label_value(out, out_len, "INFO0d acknowledged (bit 28)", detail_bit_to_yes_no(detail_value(pairs, pair_count, "b28_ack_i0d")));
        }
        append_html_label_value(out, out_len, "CRC bit range", detail_value(pairs, pair_count, "crc_bits"));
        append_html_label_value(out, out_len, "CRC16", detail_value(pairs, pair_count, "crc16"));
        append_html_label_value(out, out_len, "Tail bit range", detail_value(pairs, pair_count, "tail_bits"));
        append_html_label_value(out, out_len, "Tail", detail_value(pairs, pair_count, "tail"));
        appendf(out, out_len, "</div>");
        return;
    }

    if (strcmp(event->protocol, "V.34") == 0 && strncmp(event->summary, "INFO1", 5) == 0 && pair_count > 0) {
        bool is_info1d = (strncmp(event->summary, "INFO1d", 6) == 0);

        appendf(out, out_len, "<div class=\"detail-kv\">");
        append_html_label_value(out, out_len, "Role", detail_value(pairs, pair_count, "role"));
        append_html_label_value(out, out_len, "Table", detail_value(pairs, pair_count, "table"));
        append_html_label_value(out, out_len, "Fill/sync", detail_value(pairs, pair_count, "fsync"));
        if (is_info1d) {
            append_html_label_value(out, out_len, "Min power reduction bits 12:14", detail_value(pairs, pair_count, "b12_14_min_pwr_red"));
            append_html_label_value(out, out_len, "Additional power reduction bits 15:17", detail_value(pairs, pair_count, "b15_17_addl_pwr_red"));
            append_html_label_value(out, out_len, "MD bits 18:24", detail_value(pairs, pair_count, "b18_24_md"));
            append_html_label_value(out, out_len, "2400 row", detail_value(pairs, pair_count, "row2400_hicar"));
            append_html_label_value(out, out_len, "3429 row / V.92 bits 70:78", detail_value(pairs, pair_count, "row3429_hicar_or_v92b70"));
        } else {
            append_html_label_value(out, out_len, "Bits 12:17 (raw)", detail_value(pairs, pair_count, "b12_17_raw"));
            append_html_label_value(out, out_len, "MD", detail_value(pairs, pair_count, "md"));
            append_html_label_value(out, out_len, "U_INFO", detail_value(pairs, pair_count, "u_info"));
            append_html_label_value(out, out_len, "Bits 32:33 (raw)", detail_value(pairs, pair_count, "b32_33_raw"));
            append_html_label_value(out, out_len, "Upstream symbol-rate code", detail_value(pairs, pair_count, "upstream_symbol_rate_code"));
            append_html_label_value(out, out_len, "Downstream rate code", detail_value(pairs, pair_count, "downstream_rate_code"));
        }
        append_html_label_value(out, out_len, "Frequency offset", detail_value(pairs, pair_count, "freq_offset_10b"));
        append_html_label_value(out, out_len, "CRC bit range", detail_value(pairs, pair_count, "crc_bits"));
        append_html_label_value(out, out_len, "CRC16", detail_value(pairs, pair_count, "crc16"));
        append_html_label_value(out, out_len, "Tail bit range", detail_value(pairs, pair_count, "tail_bits"));
        append_html_label_value(out, out_len, "Tail", detail_value(pairs, pair_count, "tail"));
        appendf(out, out_len, "</div>");
        return;
    }

    /* V.8 answer tone events */
    if (strcmp(event->protocol, "V.8") == 0
        && strstr(event->summary, "detected") != NULL
        && detail_value(pairs, pair_count, "tone") != NULL) {
        appendf(out, out_len, "<div class=\"detail-kv\">");
        append_html_label_value(out, out_len, "Role", detail_value(pairs, pair_count, "role"));
        append_html_label_desanitized(out, out_len, "Tone", detail_value(pairs, pair_count, "tone"));
        appendf(out, out_len, "</div>");
        return;
    }

    /* V.8bis tone-signal events */
    if ((strcmp(event->protocol, "V.8bis") == 0 || strcmp(event->protocol, "V.8bis?") == 0)
        && detail_value(pairs, pair_count, "seg1") != NULL
        && detail_value(pairs, pair_count, "seg2") != NULL) {
        char seg1_start[32];
        char seg1_end[32];
        char seg2_start[32];
        char seg2_end[32];

        snprintf(seg1_start, sizeof(seg1_start), "%.1f ms", sample_to_ms(event->sample_offset, 8000));
        snprintf(seg1_end, sizeof(seg1_end), "%.1f ms",
                 sample_to_ms(event->sample_offset + V8BIS_SEGMENT1_SAMPLES, 8000));
        snprintf(seg2_start, sizeof(seg2_start), "%.1f ms",
                 sample_to_ms(event->sample_offset + V8BIS_SEGMENT1_SAMPLES, 8000));
        snprintf(seg2_end, sizeof(seg2_end), "%.1f ms",
                 sample_to_ms(event->sample_offset + V8BIS_SEGMENT1_SAMPLES + V8BIS_SEGMENT2_SAMPLES, 8000));

        appendf(out, out_len, "<div class=\"detail-kv\">");
        append_html_label_value(out, out_len, "Role", detail_value(pairs, pair_count, "role"));
        append_html_label_value(out, out_len, "Segment 1 dual-tone", detail_value(pairs, pair_count, "seg1"));
        append_html_label_value(out, out_len, "Segment 1 start", seg1_start);
        append_html_label_value(out, out_len, "Segment 1 end", seg1_end);
        append_html_label_value(out, out_len, "Segment 1 duration", detail_value(pairs, pair_count, "seg1_ms"));
        append_html_label_value(out, out_len, "Segment 2 identifying tone", detail_value(pairs, pair_count, "seg2"));
        append_html_label_value(out, out_len, "Segment 2 start", seg2_start);
        append_html_label_value(out, out_len, "Segment 2 end", seg2_end);
        append_html_label_value(out, out_len, "Segment 2 duration", detail_value(pairs, pair_count, "seg2_ms"));
        if (detail_value(pairs, pair_count, "seg1_a"))
            append_html_label_value(out, out_len, "Segment 1 tone A strength", detail_value(pairs, pair_count, "seg1_a"));
        if (detail_value(pairs, pair_count, "seg1_b"))
            append_html_label_value(out, out_len, "Segment 1 tone B strength", detail_value(pairs, pair_count, "seg1_b"));
        if (detail_value(pairs, pair_count, "dual"))
            append_html_label_value(out, out_len, "Segment 1 combined strength", detail_value(pairs, pair_count, "dual"));
        if (detail_value(pairs, pair_count, "balance"))
            append_html_label_value(out, out_len, "Segment 1 tone balance", detail_value(pairs, pair_count, "balance"));
        if (detail_value(pairs, pair_count, "seg2_strength"))
            append_html_label_value(out, out_len, "Segment 2 tone strength", detail_value(pairs, pair_count, "seg2_strength"));
        append_html_label_value(out, out_len, "Detector score", detail_value(pairs, pair_count, "score"));
        if (detail_value(pairs, pair_count, "weak"))
            append_html_label_value(out, out_len, "Weak candidate", detail_bit_to_yes_no(detail_value(pairs, pair_count, "weak")));
        appendf(out, out_len, "</div>");
        return;
    }

    /* V.8bis partial HDLC frame events */
    if (strcmp(event->protocol, "V.8bis?") == 0
        && strncmp(event->summary, "Partial ", 8) == 0
        && pair_count > 0
        && detail_value(pairs, pair_count, "fsk_ch") != NULL) {
        appendf(out, out_len, "<div class=\"detail-kv\">");
        append_html_label_value(out, out_len, "FSK channel", detail_value(pairs, pair_count, "fsk_ch"));
        append_html_label_value(out, out_len, "Revision nibble", detail_value(pairs, pair_count, "rev"));
        append_html_label_value(out, out_len, "Captured bytes", detail_value(pairs, pair_count, "bytes"));
        append_html_label_value(out, out_len, "Trailing bits", detail_value(pairs, pair_count, "trailing_bits"));
        append_html_label_value(out, out_len, "CRC", detail_value(pairs, pair_count, "crc"));
        append_html_label_value(out, out_len, "Reject reason", detail_value(pairs, pair_count, "reason"));
        append_html_label_value(out, out_len, "Raw bytes", detail_value(pairs, pair_count, "raw"));
        appendf(out, out_len, "</div>");
        return;
    }

    /* V.8 CI event */
    if (strcmp(event->protocol, "V.8") == 0
        && strcmp(event->summary, "CI decoded") == 0
        && pair_count > 0) {
        appendf(out, out_len, "<div class=\"detail-kv\">");
        append_html_label_value(out, out_len, "Role", detail_value(pairs, pair_count, "role"));
        append_html_label_desanitized(out, out_len, "Call function", detail_value(pairs, pair_count, "call_function"));
        appendf(out, out_len, "</div>");
        return;
    }

    /* V.8 CM/JM decoded events */
    if (strcmp(event->protocol, "V.8") == 0
        && (strcmp(event->summary, "CM decoded") == 0
            || strcmp(event->summary, "JM decoded") == 0)
        && pair_count > 0) {
        const char *mods = detail_value(pairs, pair_count, "modulations");
        const char *raw = detail_value(pairs, pair_count, "raw");

        appendf(out, out_len, "<div class=\"detail-kv\">");
        append_html_label_value(out, out_len, "Message", detail_value(pairs, pair_count, "msg"));
        append_html_label_value(out, out_len, "Role", detail_value(pairs, pair_count, "role"));
        append_html_label_desanitized(out, out_len, "Confidence", detail_value(pairs, pair_count, "confidence"));
        append_html_label_desanitized(out, out_len, "Call function", detail_value(pairs, pair_count, "call_fn"));
        if (mods) {
            char spaced_mods[512];
            size_t j = 0;
            for (size_t k = 0; mods[k] && j < sizeof(spaced_mods) - 2; k++) {
                spaced_mods[j++] = mods[k];
                if (mods[k] == ',' && mods[k + 1] && j < sizeof(spaced_mods) - 1)
                    spaced_mods[j++] = ' ';
            }
            spaced_mods[j] = '\0';
            append_html_label_value(out, out_len, "Modulations", spaced_mods);
        }
        append_html_label_desanitized(out, out_len, "Protocol", detail_value(pairs, pair_count, "protocol"));
        append_html_label_desanitized(out, out_len, "PCM modem", detail_value(pairs, pair_count, "pcm"));
        append_html_label_desanitized(out, out_len, "PSTN access", detail_value(pairs, pair_count, "pstn"));
        {
            const char *nsf = detail_value(pairs, pair_count, "nsf");
            const char *t66 = detail_value(pairs, pair_count, "t66");
            const char *v92 = detail_value(pairs, pair_count, "v92");
            if (nsf && strcmp(nsf, "???") != 0)
                append_html_label_desanitized(out, out_len, "NSF", nsf);
            if (t66 && strcmp(t66, "???") != 0)
                append_html_label_desanitized(out, out_len, "T.66", t66);
            if (v92)
                append_html_label_value(out, out_len, "V.92", v92);
        }
        if (raw)
            append_html_label_value(out, out_len, "Raw bytes", raw);
        appendf(out, out_len, "</div>");
        return;
    }

    /* V.8 CJ event */
    if (strcmp(event->protocol, "V.8") == 0
        && strcmp(event->summary, "CJ decoded") == 0
        && pair_count > 0) {
        appendf(out, out_len, "<div class=\"detail-kv\">");
        append_html_label_value(out, out_len, "Role", detail_value(pairs, pair_count, "role"));
        appendf(out, out_len, "</div>");
        return;
    }

    /* V.92 QC1/QCA1 short Phase 1 events */
    if (strcmp(event->protocol, "V.92") == 0
        && (strstr(event->summary, "QC1") != NULL
            || strstr(event->summary, "QCA1") != NULL)
        && pair_count > 0) {
        appendf(out, out_len, "<div class=\"detail-kv\">");
        append_html_label_value(out, out_len, "Role", detail_value(pairs, pair_count, "role"));
        if (detail_value(pairs, pair_count, "pre_cm"))
            append_html_label_value(out, out_len, "Window", "Pre-CM");
        if (detail_value(pairs, pair_count, "post_ansam"))
            append_html_label_value(out, out_len, "Window", "Post-ANSam");
        append_html_label_value(out, out_len, "Start", detail_value(pairs, pair_count, "start"));
        append_html_label_value(out, out_len, "Duration", detail_value(pairs, pair_count, "duration"));
        append_html_label_value(out, out_len, "FSK pair", detail_value(pairs, pair_count, "pair"));
        if (detail_value(pairs, pair_count, "peak"))
            append_html_label_value(out, out_len, "Peak strength", detail_value(pairs, pair_count, "peak"));
        if (detail_value(pairs, pair_count, "lapm"))
            append_html_label_value(out, out_len, "LAPM", detail_value(pairs, pair_count, "lapm"));
        if (detail_value(pairs, pair_count, "anspcm_level"))
            append_html_label_value(out, out_len, "ANSpcm level", detail_value(pairs, pair_count, "anspcm_level"));
        if (detail_value(pairs, pair_count, "uqts_index"))
            append_html_label_value(out, out_len, "UQTS index", detail_value(pairs, pair_count, "uqts_index"));
        if (detail_value(pairs, pair_count, "uqts_ucode"))
            append_html_label_value(out, out_len, "UQTS Ucode", detail_value(pairs, pair_count, "uqts_ucode"));
        if (detail_value(pairs, pair_count, "lock"))
            append_html_label_value(out, out_len, "Lock detail", detail_value(pairs, pair_count, "lock"));
        if (detail_value(pairs, pair_count, "repeat"))
            append_html_label_value(out, out_len, "Repeat frame", detail_value(pairs, pair_count, "repeat"));
        if (detail_value(pairs, pair_count, "decoded"))
            append_html_label_value(out, out_len, "Decoded frame", detail_value(pairs, pair_count, "decoded"));
        if (detail_value(pairs, pair_count, "qts"))
            append_html_label_value(out, out_len, "QTS candidate", detail_value(pairs, pair_count, "qts"));
        if (detail_value(pairs, pair_count, "qts_bar"))
            append_html_label_value(out, out_len, "QTS\\ reps", detail_value(pairs, pair_count, "qts_bar"));
        if (detail_value(pairs, pair_count, "bits"))
            append_html_label_value(out, out_len, "Bit slices", detail_value(pairs, pair_count, "bits"));
        if (detail_value(pairs, pair_count, "raw"))
            append_html_label_value(out, out_len, "Raw bytes", detail_value(pairs, pair_count, "raw"));
        appendf(out, out_len, "</div>");
        return;
    }

    /* V.8 negotiation completed event */
    if (strcmp(event->protocol, "V.8") == 0
        && strstr(event->summary, "V.8 negotiation decoded") != NULL
        && pair_count > 0) {
        appendf(out, out_len, "<div class=\"detail-kv\">");
        append_html_label_value(out, out_len, "Status", detail_value(pairs, pair_count, "status"));
        append_html_label_value(out, out_len, "Protocol", detail_value(pairs, pair_count, "protocol"));
        append_html_label_value(out, out_len, "PCM modem", detail_value(pairs, pair_count, "pcm"));
        append_html_label_value(out, out_len, "PSTN access", detail_value(pairs, pair_count, "pstn"));
        appendf(out, out_len, "</div>");
        return;
    }

    append_html_escaped(out, out_len, event->detail);
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
        char detail_html[2048];

        build_visual_event_detail_html(event, detail_html, sizeof(detail_html));
        if (i) fputc(',', f);
        fprintf(f, "{start:%d,duration:%d,protocol:", event->sample_offset, event->duration_samples);
        json_write_escaped(f, event->protocol);
        fprintf(f, ",summary:");
        json_write_escaped(f, event->summary);
        fprintf(f, ",detail:");
        json_write_escaped(f, event->detail);
        fprintf(f, ",detailHtml:");
        json_write_escaped(f, detail_html);
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
            ".detail-kv div{margin:0 0 2px 0;line-height:1.3;}\n"
            ".detail-kv strong{color:#3b3329;}\n"
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
            "function eventColor(protocol){if(protocol==='V.8'||protocol==='V.8bis')return '#f08a5d';if(protocol==='V.34')return '#d4a017';if(protocol==='V.90 Phase 3'||protocol==='V.90')return '#2f7f6f';if(protocol==='V.92 Phase 3')return '#4b8a3f';if(protocol==='V.90/V.92')return '#6b5b95';if(protocol==='V.91')return '#2d5fb1';return '#777';}\n"
            "function levelNorm(v,gamma,boost){const base=Math.max(0,Math.min(1,v/1000));return Math.min(1,Math.pow(base,gamma)*boost);}\n"
            "function heatColor(v,gamma,boost){const t=levelNorm(v,gamma,boost);const stops=[[8,19,29],[18,66,93],[31,140,168],[243,179,76],[251,238,197]];const seg=Math.min(stops.length-2,Math.floor(t*(stops.length-1)));const local=t*(stops.length-1)-seg;const a=stops[seg],b=stops[seg+1];const mix=n=>Math.round(a[n]+(b[n]-a[n])*local);return `rgb(${mix(0)},${mix(1)},${mix(2)})`;}\n"
            "function mergedTrack(a,b,mode,label){return{kind:'merged',label:label||'Merged',mode:mode||'overlay',primary:a,secondary:b,sampleRate:a.sampleRate,totalSamples:a.totalSamples,windowSamples:a.windowSamples,freqBinCount:a.freqBinCount,freqBinStartHz:a.freqBinStartHz,freqBinStepHz:a.freqBinStepHz};}\n"
            "function renderTrack(track,mount){const root=mount||document.getElementById('tracks');root.innerHTML='';const merged=track.kind==='merged';const sharedSpectrogram=merged&&track.mode==='shared';const baseTrack=merged?track.primary:track;const otherTrack=merged?track.secondary:null;const card=document.createElement('div');card.className='card';card.innerHTML=`<div class=\"track-title\"><h2>${track.label}</h2><div class=\"hint\">${merged?(baseTrack.events.length+otherTrack.events.length):baseTrack.events.length} decoded events</div></div><div class=\"controls\"><div class=\"cursor\">Cursor: <span class=\"cursor-time\">0.0 ms</span></div><div class=\"hint hover-label\">Move across the chart to inspect this ${merged?(sharedSpectrogram?'shared diagnostic':'overlay'):'channel'}.</div></div><div class=\"controls\"><label>Spectral lift <input class=\"spec-boost\" type=\"range\" min=\"100\" max=\"900\" step=\"25\" value=\"320\"></label><label>Contrast <input class=\"spec-gamma\" type=\"range\" min=\"20\" max=\"100\" step=\"1\" value=\"42\"></label><label>Tone lift <input class=\"tone-boost\" type=\"range\" min=\"100\" max=\"900\" step=\"25\" value=\"260\"></label></div><div class=\"legend\"></div><canvas width=\"1360\" height=\"760\"></canvas><div class=\"hint\">${merged?(sharedSpectrogram?`Shared spectrogram with split tone overlays. ${baseTrack.label} uses orange and ${otherTrack.label} uses teal.`:`Merged overlay view. ${baseTrack.label} uses orange and ${otherTrack.label} uses teal.`):'Separate channel render.'} Lower-energy structure is display-boosted only; raw decode data is unchanged.</div><table><thead><tr><th>Start</th><th>Duration</th><th>Protocol</th><th>Summary</th><th>Detail</th></tr></thead><tbody></tbody></table>`;root.appendChild(card);const canvas=card.querySelector('canvas');const ctx=canvas.getContext('2d');const legend=card.querySelector('.legend');const tbody=card.querySelector('tbody');const cursorTime=card.querySelector('.cursor-time');const hoverLabel=card.querySelector('.hover-label');const specBoostInput=card.querySelector('.spec-boost');const specGammaInput=card.querySelector('.spec-gamma');const toneBoostInput=card.querySelector('.tone-boost');const enabled=new Set(baseTrack.tones.map(t=>t.key));let cursorIndex=0;function settings(){return{specBoost:Number(specBoostInput.value)/100,specGamma:Number(specGammaInput.value)/100,toneBoost:Number(toneBoostInput.value)/100};}function allEvents(){if(!merged)return baseTrack.events;return baseTrack.events.map(e=>({...e,trackLabel:baseTrack.label,trackColor:mergePalette.a})).concat(otherTrack.events.map(e=>({...e,trackLabel:otherTrack.label,trackColor:mergePalette.b}))).sort((x,y)=>x.start-y.start);}function draw(){const cfg=settings();const w=canvas.width,h=canvas.height;ctx.clearRect(0,0,w,h);const left=64,right=20,top=24,bottom=28;const plotW=w-left-right;const envelopeH=150;const gap=28;const specTop=top+envelopeH+gap;const specH=300;const toneTop=specTop+specH+gap;const toneAreaH=h-toneTop-bottom;const toneTrackH=toneAreaH/Math.max(1,baseTrack.tones.length);ctx.fillStyle='#fffdf8';ctx.fillRect(0,0,w,h);for(let i=0;i<=10;i++){const x=left+(plotW*i/10);ctx.strokeStyle='#d9cfc0';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(x,top);ctx.lineTo(x,h-bottom);ctx.stroke();const ms=(baseTrack.totalSamples*i/10)*1000/baseTrack.sampleRate;ctx.fillStyle='#6a6257';ctx.fillText(ms.toFixed(0)+' ms',x-16,h-8);}for(let i=0;i<=6;i++){const y=top+(envelopeH*i/6);ctx.strokeStyle='#d9cfc0';ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(left+plotW,y);ctx.stroke();}ctx.fillStyle='#1d1b18';ctx.fillText(merged?'Envelope + Decoded Events (Overlay)':'Envelope + Decoded Events',left,16);allEvents().forEach(ev=>{const x1=left+(ev.start/baseTrack.totalSamples)*plotW;const x2=left+((ev.start+Math.max(ev.duration,baseTrack.windowSamples))/baseTrack.totalSamples)*plotW;ctx.fillStyle=(merged?(ev.trackColor||eventColor(ev.protocol)):eventColor(ev.protocol))+'38';ctx.fillRect(x1,top,Math.max(2,x2-x1),envelopeH);ctx.strokeStyle=merged?(ev.trackColor||eventColor(ev.protocol)):eventColor(ev.protocol);ctx.beginPath();ctx.moveTo(x1,top);ctx.lineTo(x1,specTop+specH);ctx.stroke();});const envA=baseTrack.envelope;const envB=merged?otherTrack.envelope:null;ctx.beginPath();for(let i=0;i<envA.length;i++){const x=xForPoint(i,left,plotW,envA.length);const v=merged?Math.max(envA[i],envB[i]):envA[i];const y=top+envelopeH-(v/1000)*envelopeH;if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);}ctx.lineTo(left+plotW,top+envelopeH);ctx.lineTo(left,top+envelopeH);ctx.closePath();ctx.fillStyle=merged?'rgba(90,94,128,.20)':'rgba(177,77,45,.22)';ctx.fill();ctx.strokeStyle=merged?'#4f6b8a':'#b14d2d';ctx.lineWidth=1.6;ctx.stroke();if(merged){for(const series of [{env:envA,color:mergePalette.a},{env:envB,color:mergePalette.b}]){ctx.beginPath();series.env.forEach((v,i)=>{const x=xForPoint(i,left,plotW,series.env.length);const y=top+envelopeH-(v/1000)*envelopeH;if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);});ctx.strokeStyle=series.color;ctx.lineWidth=1.05;ctx.stroke();}}ctx.fillStyle='#1d1b18';ctx.fillText(merged?(sharedSpectrogram?'Frequency Diagnosis (Shared)':'Frequency Diagnosis (Overlay)'):'Frequency Diagnosis',left,specTop-8);const colW=plotW/Math.max(1,baseTrack.envelope.length);const rowH=specH/Math.max(1,baseTrack.freqBinCount);for(let i=0;i<baseTrack.envelope.length;i++){const x=left+i*colW;for(let b=0;b<baseTrack.freqBinCount;b++){const y=specTop+specH-(b+1)*rowH;if(merged&&sharedSpectrogram){const va=baseTrack.freqHeatmap[i*baseTrack.freqBinCount+b];const vb=otherTrack.freqHeatmap[i*otherTrack.freqBinCount+b];const v=va>vb?va:vb;ctx.fillStyle=heatColor(v,cfg.specGamma,cfg.specBoost);ctx.fillRect(x,y,Math.max(1,colW+0.4),Math.max(1,rowH+0.4));}else if(merged){const va=baseTrack.freqHeatmap[i*baseTrack.freqBinCount+b];const vb=otherTrack.freqHeatmap[i*otherTrack.freqBinCount+b];const a=levelNorm(va,cfg.specGamma,cfg.specBoost);const bb=levelNorm(vb,cfg.specGamma,cfg.specBoost);ctx.fillStyle=`rgba(198,106,43,${Math.min(.95,a)})`;ctx.fillRect(x,y,Math.max(1,colW+0.4),Math.max(1,rowH+0.4));ctx.fillStyle=`rgba(31,122,140,${Math.min(.95,bb)})`;ctx.fillRect(x,y,Math.max(1,colW+0.4),Math.max(1,rowH+0.4));}else{const v=baseTrack.freqHeatmap[i*baseTrack.freqBinCount+b];ctx.fillStyle=heatColor(v,cfg.specGamma,cfg.specBoost);ctx.fillRect(x,y,Math.max(1,colW+0.4),Math.max(1,rowH+0.4));}}}ctx.strokeStyle='#d9cfc0';for(let j=0;j<=6;j++){const freq=baseTrack.freqBinStartHz+j*((baseTrack.freqBinCount-1)*baseTrack.freqBinStepHz/6);const bin=(freq-baseTrack.freqBinStartHz)/baseTrack.freqBinStepHz;const y=specTop+specH-bin*rowH;ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(left+plotW,y);ctx.stroke();ctx.fillStyle='#6a6257';ctx.fillText(Math.round(freq)+' Hz',6,y+4);}ctx.strokeStyle='#8a7f70';ctx.strokeRect(left,specTop,plotW,specH);ctx.fillStyle='#1d1b18';ctx.fillText(merged?(sharedSpectrogram?'Tracked Tone Energies (Split)':'Tracked Tone Energies (Overlay)'):'Tracked Tone Energies',left,toneTop-8);baseTrack.tones.forEach((tone,toneIdx)=>{const y0=toneTop+toneIdx*toneTrackH;ctx.strokeStyle='#e6ddcf';ctx.beginPath();ctx.moveTo(left,y0+toneTrackH);ctx.lineTo(left+plotW,y0+toneTrackH);ctx.stroke();ctx.fillStyle='#6a6257';ctx.fillText(tone.label,left-4,y0+12);if(!enabled.has(tone.key))return;const drawTone=(values,color,width)=>{ctx.beginPath();values.forEach((v,i)=>{const x=xForPoint(i,left,plotW,baseTrack.envelope.length);const display=levelNorm(v,Math.min(0.85,cfg.specGamma+0.06),cfg.toneBoost);const y=y0+toneTrackH-display*(toneTrackH-6)-3;if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);});ctx.strokeStyle=color;ctx.lineWidth=width;ctx.stroke();};if(merged){drawTone(baseTrack.tones[toneIdx].values,sharedSpectrogram?'rgba(198,106,43,0.95)':mergePalette.a,sharedSpectrogram?1.3:1.15);drawTone(otherTrack.tones[toneIdx].values,sharedSpectrogram?'rgba(31,122,140,0.95)':mergePalette.b,sharedSpectrogram?1.3:1.15);}else{drawTone(baseTrack.tones[toneIdx].values,palette[toneIdx%%palette.length],1.35);}});const seen=new Set();allEvents().forEach(ev=>{const tag=`${ev.trackLabel||''}:${ev.protocol}:${ev.summary}`;if(seen.has(tag))return;seen.add(tag);const x=left+(ev.start/baseTrack.totalSamples)*plotW;ctx.save();ctx.translate(x+2,top+12);ctx.rotate(-Math.PI/10);ctx.fillStyle=merged?(ev.trackColor||eventColor(ev.protocol)):eventColor(ev.protocol);const prefix=merged&&ev.trackLabel?`[${ev.trackLabel}] `:'';const text=prefix+ev.summary;ctx.fillText(text.length>24?text.slice(0,24)+'...':text,0,0);ctx.restore();});const cursorX=xForPoint(cursorIndex,left,plotW,baseTrack.envelope.length);ctx.strokeStyle='#111';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(cursorX,top);ctx.lineTo(cursorX,h-bottom);ctx.stroke();}function updateCursor(index){const cfg=settings();cursorIndex=Math.max(0,Math.min(baseTrack.envelope.length-1,index));const start=cursorIndex*baseTrack.windowSamples;cursorTime.textContent=(start*1000/baseTrack.sampleRate).toFixed(1)+' ms';const toneSource=merged?baseTrack.tones.map(t=>({label:`${baseTrack.label} ${t.label}`,v:t.values[cursorIndex]})).concat(otherTrack.tones.map(t=>({label:`${otherTrack.label} ${t.label}`,v:t.values[cursorIndex]}))):baseTrack.tones.map(t=>({label:t.label,v:t.values[cursorIndex]}));const active=toneSource.sort((a,b)=>b.v-a.v).slice(0,4).filter(x=>x.v>6);let dominantA=0,dominantValA=-1;for(let b=0;b<baseTrack.freqBinCount;b++){const v=baseTrack.freqHeatmap[cursorIndex*baseTrack.freqBinCount+b];if(v>dominantValA){dominantValA=v;dominantA=b;}}let spectral=`dominant bin ${baseTrack.freqBinStartHz+dominantA*baseTrack.freqBinStepHz} Hz raw ${(dominantValA/10).toFixed(1)}%% display x${cfg.specBoost.toFixed(1)}`;if(merged){let dominantB=0,dominantValB=-1;for(let b=0;b<otherTrack.freqBinCount;b++){const v=otherTrack.freqHeatmap[cursorIndex*otherTrack.freqBinCount+b];if(v>dominantValB){dominantValB=v;dominantB=b;}}spectral=sharedSpectrogram?`shared ${baseTrack.freqBinStartHz+((dominantValA>dominantValB?dominantA:dominantB)*baseTrack.freqBinStepHz)} Hz | ${baseTrack.label}: ${(baseTrack.freqBinStartHz+dominantA*baseTrack.freqBinStepHz)} Hz ${(dominantValA/10).toFixed(1)}%% | ${otherTrack.label}: ${(otherTrack.freqBinStartHz+dominantB*otherTrack.freqBinStepHz)} Hz ${(dominantValB/10).toFixed(1)}%%`:`${baseTrack.label}: ${(baseTrack.freqBinStartHz+dominantA*baseTrack.freqBinStepHz)} Hz ${(dominantValA/10).toFixed(1)}%% | ${otherTrack.label}: ${(otherTrack.freqBinStartHz+dominantB*otherTrack.freqBinStepHz)} Hz ${(dominantValB/10).toFixed(1)}%%`;}hoverLabel.textContent=active.length?`${spectral} | `+active.map(x=>x.label+' raw '+(x.v/10).toFixed(1)+'%%').join(' | '):spectral;draw();}function rebuildLegend(){legend.innerHTML='';baseTrack.tones.forEach((tone,toneIdx)=>{const b=document.createElement('button');b.textContent=tone.label;b.style.borderColor=merged?mergePalette.a:palette[toneIdx%%palette.length];if(!enabled.has(tone.key))b.classList.add('off');b.onclick=()=>{if(enabled.has(tone.key))enabled.delete(tone.key);else enabled.add(tone.key);rebuildLegend();updateCursor(cursorIndex);};legend.appendChild(b);});if(merged){const key=document.createElement('div');key.className='hint';key.textContent=sharedSpectrogram?`${baseTrack.label} and ${otherTrack.label} share the spectrogram; tone overlays stay split orange/teal`:`${baseTrack.label} = orange, ${otherTrack.label} = teal`;legend.appendChild(key);}}canvas.addEventListener('mousemove',ev=>{const r=canvas.getBoundingClientRect();const x=(ev.clientX-r.left)*(canvas.width/r.width);const left=64;const plotW=canvas.width-left-20;updateCursor(Math.round(((x-left)/plotW)*(baseTrack.envelope.length-1)));});canvas.addEventListener('click',ev=>{const r=canvas.getBoundingClientRect();const x=(ev.clientX-r.left)*(canvas.width/r.width);const sample=((x-64)/(canvas.width-84))*baseTrack.totalSamples;let best=null;allEvents().forEach((e,rowIdx)=>{const end=e.start+Math.max(e.duration,baseTrack.windowSamples);if(sample>=e.start&&sample<=end)best=rowIdx;});if(best!==null){tbody.querySelectorAll('tr')[best]?.scrollIntoView({block:'center'});}});specBoostInput.addEventListener('input',()=>updateCursor(cursorIndex));specGammaInput.addEventListener('input',()=>updateCursor(cursorIndex));toneBoostInput.addEventListener('input',()=>updateCursor(cursorIndex));allEvents().forEach(ev=>{const tr=document.createElement('tr');const prefix=merged&&ev.trackLabel?`[${ev.trackLabel}] `:'';tr.innerHTML=`<td>${(ev.start*1000/baseTrack.sampleRate).toFixed(1)} ms</td><td>${(ev.duration*1000/baseTrack.sampleRate).toFixed(1)} ms</td><td>${ev.protocol}</td><td>${prefix}${ev.summary}</td><td>${ev.detailHtml||ev.detail||''}</td>`;tr.onclick=()=>updateCursor(Math.round(ev.start/baseTrack.windowSamples));tbody.appendChild(tr);});rebuildLegend();updateCursor(0);}\n"
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
    dst->jm_cm.nsf = -1;
    dst->jm_cm.t66 = -1;
    dst->v92 = -1;
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

static bool v8_byte_is_valid_top_level_tag(uint8_t byte)
{
    switch (byte & 0x1F) {
    case V8_LOCAL_CALL_FUNCTION_TAG:
    case V8_LOCAL_MODULATION_TAG:
    case V8_LOCAL_PROTOCOLS_TAG:
    case V8_LOCAL_PSTN_ACCESS_TAG:
    case V8_LOCAL_NSF_TAG:
    case V8_LOCAL_PCM_MODEM_AVAILABILITY_TAG:
    case V8_LOCAL_T66_TAG:
        return true;
    default:
        return false;
    }
}

static bool v8_byte_is_continuation_tag(uint8_t byte)
{
    return (byte & 0x18) == 0x10;
}

static bool v8_cm_jm_candidate_plausible(const uint8_t *data,
                                         int len,
                                         const v8_parms_t *parsed)
{
    int first_tag = -1;
    int top_level_count = 0;
    int modulation_tag_count = 0;
    int continuation_count = 0;

    if (!data || len <= 0 || !parsed)
        return false;

    for (int i = 0; i < len; i++) {
        uint8_t byte = data[i];

        if (byte == 0)
            break;
        if (v8_byte_is_continuation_tag(byte)) {
            continuation_count++;
            continue;
        }
        if (!v8_byte_is_valid_top_level_tag(byte))
            continue;
        if (first_tag < 0)
            first_tag = byte & 0x1F;
        top_level_count++;
        if ((byte & 0x1F) == V8_LOCAL_MODULATION_TAG)
            modulation_tag_count++;
    }

    if (first_tag < 0)
        return false;
    if (first_tag != V8_LOCAL_CALL_FUNCTION_TAG
        && first_tag != V8_LOCAL_MODULATION_TAG)
        return false;
    if (parsed->jm_cm.modulations == 0
        && parsed->jm_cm.protocols == 0
        && parsed->jm_cm.pstn_access == 0
        && parsed->jm_cm.pcm_modem_availability == 0)
        return false;
    if (parsed->jm_cm.modulations != 0)
        return true;
    if (modulation_tag_count > 0 || continuation_count > 0)
        return top_level_count >= 1;
    return top_level_count >= 2;
}

static int v8_candidate_score(const v8_parms_t *parsed, int len);

static bool v8_extract_best_cm_jm_payload(v8_parms_t *dst,
                                          uint8_t *cleaned,
                                          int *cleaned_len,
                                          const uint8_t *data,
                                          int len,
                                          bool calling_party)
{
    v8_parms_t best_parsed;
    int best_start = -1;
    int best_len = 0;
    int best_score = -1000000;

    if (dst)
        memset(dst, 0, sizeof(*dst));
    if (cleaned_len)
        *cleaned_len = 0;
    if (!data || len <= 0)
        return false;

    for (int start = 0; start < len; start++) {
        int end;

        if (!v8_byte_is_valid_top_level_tag(data[start]))
            continue;
        for (end = start; end < len; end++) {
            v8_parms_t parsed;
            int span_len;
            int score;

            if (data[end] == 0)
                break;
            if (end > start
                && !v8_byte_is_valid_top_level_tag(data[end])
                && !v8_byte_is_continuation_tag(data[end])) {
                break;
            }
            span_len = end - start + 1;
            if (!v8_parse_cm_jm_candidate(&parsed, data + start, span_len, calling_party))
                continue;
            if (!v8_cm_jm_candidate_plausible(data + start, span_len, &parsed))
                continue;
            score = v8_candidate_score(&parsed, span_len);
            if ((data[start] & 0x1F) == V8_LOCAL_CALL_FUNCTION_TAG)
                score += 24;
            if ((data[start] & 0x1F) == V8_LOCAL_MODULATION_TAG)
                score += 12;
            if (end + 1 < len && data[end + 1] == 0)
                score += 8;
            if (score > best_score) {
                best_score = score;
                best_start = start;
                best_len = span_len;
                best_parsed = parsed;
            }
        }
    }

    if (best_start < 0 || best_len <= 0)
        return false;
    if (dst)
        *dst = best_parsed;
    if (cleaned && cleaned_len && *cleaned_len >= 0) {
        memcpy(cleaned, data + best_start, (size_t) best_len);
        *cleaned_len = best_len;
    } else if (cleaned_len) {
        *cleaned_len = best_len;
    }
    return true;
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

static bool v8_raw_hits_similar(const v8_raw_msg_hit_t *a, const v8_raw_msg_hit_t *b)
{
    int overlap;
    int matches = 0;
    int mismatches = 0;

    if (!a || !b || !a->seen || !b->seen)
        return false;
    if (a->preamble_type != b->preamble_type)
        return false;

    overlap = (a->byte_len < b->byte_len) ? a->byte_len : b->byte_len;
    if (overlap < 3)
        return false;

    for (int i = 0; i < overlap; i++) {
        if (a->bytes[i] == b->bytes[i])
            matches++;
        else
            mismatches++;
    }

    if (matches < 3)
        return false;
    if (matches * 4 < overlap * 3)
        return false;
    if (mismatches > 1 && mismatches * 3 > overlap)
        return false;
    return true;
}

static void v8_raw_scan_consider_hit(v8_raw_scan_state_t *scan, const v8_raw_msg_hit_t *hit)
{
    v8_raw_msg_hit_t candidate;

    if (!scan || !hit || !hit->seen)
        return;

    candidate = *hit;
    candidate.repeat_count = 1;
    candidate.repeated_confirmed = false;

    if (scan->last_cm_jm.seen
        && v8_raw_hits_similar(&scan->last_cm_jm, &candidate)
        && abs(candidate.sample_offset - scan->last_cm_jm.sample_offset) >= 80) {
        candidate.repeat_count = scan->last_cm_jm.repeat_count + 1;
        candidate.repeated_confirmed = (candidate.repeat_count >= 2);
        candidate.score += (candidate.repeat_count - 1) * 80;
    }

    scan->last_cm_jm = candidate;
    if (!scan->best_cm_jm.seen || candidate.score > scan->best_cm_jm.score)
        scan->best_cm_jm = candidate;
}

static void v8_consider_repeated_hit(v8_raw_msg_hit_t *last,
                                     v8_raw_msg_hit_t *best,
                                     const v8_raw_msg_hit_t *hit,
                                     int min_repeat_gap,
                                     int repeat_score_bonus)
{
    v8_raw_msg_hit_t candidate;

    if (!best || !hit || !hit->seen)
        return;

    candidate = *hit;
    if (candidate.repeat_count <= 0)
        candidate.repeat_count = 1;
    candidate.repeated_confirmed = false;

    if (last
        && last->seen
        && v8_raw_hits_similar(last, &candidate)
        && abs(candidate.sample_offset - last->sample_offset) >= min_repeat_gap) {
        candidate.repeat_count = last->repeat_count + 1;
        candidate.repeated_confirmed = (candidate.repeat_count >= 2);
        candidate.score += (candidate.repeat_count - 1) * repeat_score_bonus;
    }

    if (last)
        *last = candidate;
    if (!best->seen || candidate.score > best->score)
        *best = candidate;
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
    if (!v8_cm_jm_candidate_plausible(scan->rx_data, scan->rx_data_ptr, &parsed))
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

    v8_raw_scan_consider_hit(scan, &hit);
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
        if (!v8_cm_jm_candidate_plausible(bytes, byte_len, &parsed))
            continue;

        memset(&hit, 0, sizeof(hit));
        hit.seen = true;
        hit.byte_len = byte_len;
        hit.sample_offset = (start * 8000) / 300;
        hit.score = v8_candidate_score(&parsed, byte_len) - 15;
        memcpy(hit.bytes, bytes, (size_t) byte_len);

        v8_raw_scan_consider_hit(scan, &hit);
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

static int v8_collect_v21_bursts(const int16_t *samples,
                                 int total_samples,
                                 int sample_rate,
                                 int search_start,
                                 int search_end,
                                 bool use_ch2,
                                 v8_fsk_burst_hit_t *bursts,
                                 int max_bursts)
{
    enum {
        WINDOW_SAMPLES = 160,
        STEP_SAMPLES = 80,
        MIN_RUN_WINDOWS = 2,
        MERGE_GAP_SAMPLES = 240
    };
    const double f0 = use_ch2 ? 1650.0 : 980.0;
    const double f1 = use_ch2 ? 1850.0 : 1180.0;
    int run_start = -1;
    int run_windows = 0;
    double run_peak = 0.0;
    int burst_count = 0;

    if (!samples || total_samples <= 0 || sample_rate <= 0 || !bursts || max_bursts <= 0)
        return 0;

    if (search_start < 0)
        search_start = 0;
    if (search_end <= 0 || search_end > total_samples)
        search_end = total_samples;
    if (search_end - search_start < WINDOW_SAMPLES)
        return 0;

    memset(bursts, 0, (size_t) max_bursts * sizeof(*bursts));

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
        if (strength >= 0.12) {
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
            v8_fsk_burst_hit_t candidate;

            memset(&candidate, 0, sizeof(candidate));
            candidate.seen = true;
            candidate.start_sample = run_start;
            candidate.duration_samples = run_windows * STEP_SAMPLES + (WINDOW_SAMPLES - STEP_SAMPLES);
            candidate.peak_strength = run_peak;

            if (burst_count > 0
                && candidate.start_sample <= bursts[burst_count - 1].start_sample
                                          + bursts[burst_count - 1].duration_samples
                                          + MERGE_GAP_SAMPLES) {
                int prev_end = bursts[burst_count - 1].start_sample + bursts[burst_count - 1].duration_samples;
                int new_end = candidate.start_sample + candidate.duration_samples;

                if (new_end > prev_end)
                    bursts[burst_count - 1].duration_samples = new_end - bursts[burst_count - 1].start_sample;
                if (candidate.peak_strength > bursts[burst_count - 1].peak_strength)
                    bursts[burst_count - 1].peak_strength = candidate.peak_strength;
            } else if (burst_count < max_bursts) {
                bursts[burst_count++] = candidate;
            } else {
                break;
            }
        }
        run_start = -1;
        run_windows = 0;
        run_peak = 0.0;
    }

    if (run_start >= 0 && run_windows >= MIN_RUN_WINDOWS && burst_count < max_bursts) {
        bursts[burst_count].seen = true;
        bursts[burst_count].start_sample = run_start;
        bursts[burst_count].duration_samples = run_windows * STEP_SAMPLES + (WINDOW_SAMPLES - STEP_SAMPLES);
        bursts[burst_count].peak_strength = run_peak;
        burst_count++;
    }

    return burst_count;
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

    search_start = burst->start_sample - 1600;
    search_end = burst->start_sample + burst->duration_samples + 800;
    if (search_start < 0)
        search_start = 0;
    if (search_end > total_samples)
        search_end = total_samples;
    if (search_end - search_start < (sample_rate / 300) * 12)
        return false;

    for (int invert = 0; invert <= 1; invert++) {
        for (int bit_rate = 294; bit_rate <= 306; bit_rate++) {
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
                    v8_parms_t parsed;

                    ones_score = v8_ones_lock_score(bits, confidence, bit_count, i);
                    if (ones_score < 32)
                        continue;
                    sync_score = v8_sync_lock_score(bits, confidence, bit_count, i + 10, V8_LOCAL_SYNC_CM_JM);
                    if (sync_score < 32)
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
                    if (!v8_parse_cm_jm_candidate(&parsed, payload, payload_len, calling_party))
                        continue;
                    if (!v8_cm_jm_candidate_plausible(payload, payload_len, &parsed))
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

static bool v8_targeted_v21_decode_repeated(const int16_t *samples,
                                            int total_samples,
                                            int sample_rate,
                                            const v8_fsk_burst_hit_t *bursts,
                                            int burst_count,
                                            bool use_ch2,
                                            bool calling_party,
                                            v8_raw_msg_hit_t *out)
{
    v8_raw_msg_hit_t best = {0};
    v8_raw_msg_hit_t last = {0};

    if (!samples || total_samples <= 0 || sample_rate <= 0 || !bursts || burst_count <= 0 || !out)
        return false;

    for (int i = 0; i < burst_count; i++) {
        v8_raw_msg_hit_t hit;

        memset(&hit, 0, sizeof(hit));
        if (!bursts[i].seen)
            continue;
        if (!v8_targeted_v21_decode(samples,
                                    total_samples,
                                    sample_rate,
                                    &bursts[i],
                                    use_ch2,
                                    calling_party,
                                    &hit)) {
            continue;
        }
        v8_consider_repeated_hit(&last, &best, &hit, 160, 100);
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
                                            bool calling_party,
                                            v8_raw_msg_hit_t *out,
                                            v8_targeted_diag_t *diag)
{
    const double mark_hz = use_ch2 ? 1650.0 : 980.0;
    const double space_hz = use_ch2 ? 1850.0 : 1180.0;
    int search_start;
    int search_end;
    v8_raw_msg_hit_t best = {0};

    if (!samples || total_samples <= 0 || sample_rate <= 0 || !burst || !burst->seen || !out)
        return false;

    if (diag) {
        memset(diag, 0, sizeof(*diag));
        diag->best_burst_index = -1;
        diag->best_bit_rate = -1;
        diag->best_phase_q = -1;
    }

    search_start = burst->start_sample - 1600;
    search_end = burst->start_sample + burst->duration_samples + 800;
    if (search_start < 0)
        search_start = 0;
    if (search_end > total_samples)
        search_end = total_samples;

    for (int invert = 0; invert <= 1; invert++) {
        for (int bit_rate = 294; bit_rate <= 306; bit_rate++) {
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
                    v8_parms_t parsed;

                    ones_score = v8_ones_lock_score(bits, confidence, bit_count, i);
                    if (ones_score < 32)
                        continue;

                    sync_score_v92 = v8_sync_lock_score(bits, confidence, bit_count, i + 10, V8_LOCAL_SYNC_V92);
                    if (sync_score_v92 >= 32) {
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

                            if (repeat_ones_score >= 32 && repeat_sync_score >= 32
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
                                    if (tail_ones_score >= 32)
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
                            if (diag && (!diag->seen || score > diag->best_score)) {
                                diag->seen = true;
                                diag->best_preamble_type = V8_LOCAL_SYNC_V92;
                                diag->best_score = score;
                                diag->best_ones_score = ones_score;
                                diag->best_sync_score = sync_score_v92;
                                diag->best_bit_rate = bit_rate;
                                diag->best_invert = invert;
                                diag->best_phase_q = phase_q;
                                diag->best_bit_count = bit_count;
                                diag->best_byte_len = byte_len;
                                diag->best_sample_offset = search_start + (int) lround(phase + ((double) i * symbol_samples));
                                if (diag->best_byte_len > (int) sizeof(diag->best_bytes))
                                    diag->best_byte_len = (int) sizeof(diag->best_bytes);
                                memcpy(diag->best_bytes, bytes, (size_t) diag->best_byte_len);
                            }
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
                    if (sync_score_cm_jm < 32)
                        continue;

                    byte_len = v8_decode_soft_async_bytes(bits,
                                                          confidence,
                                                          bit_count,
                                                          i + 20,
                                                          bytes,
                                                          (int) sizeof(bytes),
                                                          &framing_penalty);
                    if (!v8_parse_cm_jm_candidate(&parsed, bytes, byte_len, calling_party))
                        continue;
                    if (!v8_cm_jm_candidate_plausible(bytes, byte_len, &parsed))
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
                    if (parsed.jm_cm.modulations & V8_MOD_V34)
                        score += 18;
                    if (parsed.jm_cm.modulations & (V8_MOD_V90 | V8_MOD_V92))
                        score += 10;
                    if (parsed.jm_cm.pcm_modem_availability != 0)
                        score += 8;
                    score += 8 - abs(bit_rate - 302);
                    if (use_ch2)
                        score += 6;
                    score -= framing_penalty;
                    score -= i / 2;
                    if (invert)
                        score -= 2;
                    if (diag && (!diag->seen || score > diag->best_score)) {
                        diag->seen = true;
                        diag->best_preamble_type = V8_LOCAL_SYNC_CM_JM;
                        diag->best_score = score;
                        diag->best_ones_score = ones_score;
                        diag->best_sync_score = sync_score_cm_jm;
                        diag->best_bit_rate = bit_rate;
                        diag->best_invert = invert;
                        diag->best_phase_q = phase_q;
                        diag->best_bit_count = bit_count;
                        diag->best_byte_len = byte_len;
                        diag->best_sample_offset = search_start + (int) lround(phase + ((double) i * symbol_samples));
                        if (diag->best_byte_len > (int) sizeof(diag->best_bytes))
                            diag->best_byte_len = (int) sizeof(diag->best_bytes);
                        memcpy(diag->best_bytes, bytes, (size_t) diag->best_byte_len);
                    }
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

static bool v8_targeted_v21_bytes_candidate_repeated(const int16_t *samples,
                                                     int total_samples,
                                                     int sample_rate,
                                                     const v8_fsk_burst_hit_t *bursts,
                                                     int burst_count,
                                                     bool use_ch2,
                                                     bool calling_party,
                                                     v8_raw_msg_hit_t *out,
                                                     v8_targeted_diag_t *diag)
{
    v8_raw_msg_hit_t best = {0};
    v8_raw_msg_hit_t last = {0};

    if (!samples || total_samples <= 0 || sample_rate <= 0 || !bursts || burst_count <= 0 || !out)
        return false;

    if (diag) {
        memset(diag, 0, sizeof(*diag));
        diag->burst_count = burst_count;
        diag->best_burst_index = -1;
        diag->best_bit_rate = -1;
        diag->best_phase_q = -1;
    }

    for (int i = 0; i < burst_count; i++) {
        v8_raw_msg_hit_t hit;
        v8_targeted_diag_t burst_diag;

        memset(&hit, 0, sizeof(hit));
        if (!bursts[i].seen)
            continue;
        if (!v8_targeted_v21_bytes_candidate(samples,
                                             total_samples,
                                             sample_rate,
                                             &bursts[i],
                                             use_ch2,
                                             calling_party,
                                             &hit,
                                             &burst_diag)) {
            if (diag && burst_diag.seen && (!diag->seen || burst_diag.best_score > diag->best_score)) {
                *diag = burst_diag;
                diag->burst_count = burst_count;
                diag->best_burst_index = i;
            }
            continue;
        }
        if (diag && burst_diag.seen && (!diag->seen || burst_diag.best_score > diag->best_score)) {
            *diag = burst_diag;
            diag->burst_count = burst_count;
            diag->best_burst_index = i;
        }
        v8_consider_repeated_hit(&last, &best, &hit, 160, 100);
        if (diag) {
            diag->repeat_count = best.repeat_count;
            diag->repeated_confirmed = best.repeated_confirmed;
        }
    }

    if (!best.seen)
        return false;
    *out = best;
    if (diag) {
        diag->repeat_count = best.repeat_count;
        diag->repeated_confirmed = best.repeated_confirmed;
    }
    return true;
}

static bool v8_detect_cj_run(const int16_t *samples,
                             int total_samples,
                             int sample_rate,
                             int search_start,
                             int search_end,
                             bool use_ch2,
                             int *cj_sample_out)
{
    const double mark_hz = use_ch2 ? 1650.0 : 980.0;
    const double space_hz = use_ch2 ? 1850.0 : 1180.0;
    int best_score = -1;
    int best_sample = -1;

    if (!samples || total_samples <= 0 || sample_rate <= 0 || !cj_sample_out)
        return false;

    if (search_start < 0)
        search_start = 0;
    if (search_end <= 0 || search_end > total_samples)
        search_end = total_samples;
    if (search_end - search_start < 40)
        return false;

    for (int invert = 0; invert <= 1; invert++) {
        for (int bit_rate = 294; bit_rate <= 306; bit_rate++) {
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
                    int zero_count = 0;
                    int framing_penalty = 0;
                    int confidence_bonus = 0;

                    for (int j = 0; j < 3; j++) {
                        int pos = i + j * 10;

                        if (bits[pos] != 0)
                            framing_penalty += 25 + (int) lround(confidence[pos] * 80.0);
                        if (bits[pos + 9] != 1)
                            framing_penalty += 25 + (int) lround(confidence[pos + 9] * 80.0);
                        for (int k = 0; k < 8; k++) {
                            if (bits[pos + 1 + k] != 0)
                                framing_penalty += 10 + (int) lround(confidence[pos + 1 + k] * 60.0);
                            else
                                confidence_bonus += (int) lround(confidence[pos + 1 + k] * 20.0);
                        }
                        if (framing_penalty < 140)
                            zero_count++;
                    }

                    if (zero_count < 3)
                        continue;

                    {
                        int score = 700 + confidence_bonus - framing_penalty - abs(bit_rate - 300) * 4;

                        if (invert)
                            score -= 4;
                        if (score > best_score) {
                            best_score = score;
                            best_sample = search_start + (int) lround(phase + ((double) i * symbol_samples));
                        }
                    }
                }
            }
        }
    }

    if (best_sample < 0)
        return false;
    *cj_sample_out = best_sample;
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

static void append_v92_short_phase1_fields(char *detail,
                                           size_t detail_len,
                                           const v92_short_phase1_candidate_t *v92c)
{
    size_t used;

    if (!detail || detail_len == 0 || !v92c)
        return;

    used = strlen(detail);
    if (used >= detail_len)
        return;

    if (v92c->digital_modem) {
        snprintf(detail + used,
                 detail_len - used,
                 " lapm=%s anspcm_level=%s",
                 v92c->lapm ? "yes" : "no",
                 v92_anspcm_level_to_str(v92c->aux_value));
    } else {
        snprintf(detail + used,
                 detail_len - used,
                 " lapm=%s uqts_index=0x%X uqts_ucode=%d",
                 v92c->lapm ? "yes" : "no",
                 v92c->aux_value,
                 v92c->uqts_ucode);
    }

    used = strlen(detail);
    if (used < detail_len) {
        if (v92c->decoded_frame_index > 0) {
            snprintf(detail + used,
                     detail_len - used,
                     " decoded=%03X/repeat",
                     v92c->decoded_frame_bits);
        } else {
            snprintf(detail + used,
                     detail_len - used,
                     " decoded=%03X",
                     v92c->decoded_frame_bits);
        }
    }

    used = strlen(detail);
    if (used < detail_len) {
        snprintf(detail + used,
                 detail_len - used,
                 " repeat=%s",
                 v92c->repeat_seen ? (v92c->repeat_match ? "match" : "differs") : "missing");
    }
}

static const char *v8_preamble_detail(int preamble_type);

static void print_v8_targeted_diag(const v8_targeted_diag_t *diag,
                                   const v8_fsk_burst_hit_t *bursts,
                                   bool use_ch2)
{
    char hexbuf[96];

    if (!diag || !diag->seen)
        return;

    hexbuf[0] = '\0';
    format_hex_bytes(hexbuf, sizeof(hexbuf), diag->best_bytes, diag->best_byte_len);
    printf("  Lock debug:      bursts=%d best_burst=%d pair=%s preamble=%s score=%d ones=%d sync=%d bitrate=%d invert=%s phase=%.2f bits=%d",
           diag->burst_count,
           diag->best_burst_index,
           use_ch2 ? "1650/1850 Hz" : "980/1180 Hz",
           v8_preamble_detail(diag->best_preamble_type),
           diag->best_score,
           diag->best_ones_score,
           diag->best_sync_score,
           diag->best_bit_rate,
           diag->best_invert ? "yes" : "no",
           diag->best_phase_q >= 0 ? ((double) diag->best_phase_q / 4.0) : -1.0,
           diag->best_bit_count);
    if (diag->repeat_count > 0)
        printf(" repeats=%d%s", diag->repeat_count, diag->repeated_confirmed ? " confirmed" : "");
    printf("\n");
    if (diag->best_burst_index >= 0 && bursts && diag->best_burst_index < diag->burst_count && bursts[diag->best_burst_index].seen) {
        printf("  Lock window:     %.1f-%.1f ms peak %.1f%%\n",
               sample_to_ms(bursts[diag->best_burst_index].start_sample, 8000),
               sample_to_ms(bursts[diag->best_burst_index].start_sample + bursts[diag->best_burst_index].duration_samples, 8000),
               bursts[diag->best_burst_index].peak_strength * 100.0);
    }
    if (hexbuf[0] != '\0')
        printf("  Lock bytes:      %s\n", hexbuf);
}

static const char *v8_preamble_to_candidate_label(int preamble_type, bool calling_party)
{
    switch (preamble_type) {
    case V8_LOCAL_SYNC_CM_JM:
        return calling_party ? "CM observed" : "JM observed";
    case V8_LOCAL_SYNC_V92:
        return "V.92 short Phase 1 control observed";
    default:
        return calling_party ? "CM observed" : "JM observed";
    }
}

static const char *v8_cm_jm_name(bool calling_party)
{
    return calling_party ? "CM" : "JM";
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

static bool is_v92_qc2_label(const char *summary)
{
    if (!summary)
        return false;
    return strcmp(summary, "QC2a") == 0
        || strcmp(summary, "QCA2a") == 0
        || strcmp(summary, "QC2d") == 0
        || strcmp(summary, "QCA2d") == 0;
}

static void print_v92_qc2_from_v8bis_messages(const int16_t *samples,
                                              int total_samples)
{
    call_log_t msg_log;
    int max_sample;
    bool printed_header = false;

    if (!samples || total_samples <= 0)
        return;

    call_log_init(&msg_log);
    max_sample = total_samples;
    if (max_sample > V8_EARLY_SEARCH_LIMIT_SAMPLES)
        max_sample = V8_EARLY_SEARCH_LIMIT_SAMPLES;
    v8bis_collect_msg_events(&msg_log, samples, total_samples, max_sample);
    for (size_t i = 0; i < msg_log.count; i++) {
        const call_log_event_t *event = &msg_log.events[i];

        if (strcmp(event->protocol, "V.92") != 0 || !is_v92_qc2_label(event->summary))
            continue;
        if (!printed_header) {
            printf("  V.92 QC2/QCA2 via V.8bis ID field:\n");
            printed_header = true;
        }
        printf("    %s at %.1f ms\n", event->summary, sample_to_ms(event->sample_offset, 8000));
        if (event->detail[0] != '\0')
            printf("      %s\n", event->detail);
    }
    call_log_reset(&msg_log);
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
    out->cm_jm_raw_len = v8->cm_jm_len;
    if (out->cm_jm_raw_len > (int) sizeof(out->cm_jm_raw))
        out->cm_jm_raw_len = (int) sizeof(out->cm_jm_raw);
    memcpy(out->cm_jm_raw, v8->cm_jm_data, (size_t) out->cm_jm_raw_len);
    if (!v8_extract_best_cm_jm_payload(&parsed,
                                       out->cm_jm_raw,
                                       &out->cm_jm_raw_len,
                                       out->cm_jm_raw,
                                       out->cm_jm_raw_len,
                                       out->calling_party))
        return;

    out->result = parsed;
    out->last_status = parsed.status;
    out->cm_jm_sample = sample_offset;
    out->cm_jm_complete = false;
    out->cm_jm_salvaged = true;
}

static int v8_probe_last_sample(const v8_probe_result_t *probe)
{
    int end = -1;

    if (!probe)
        return -1;
    if (probe->ansam_sample >= 0)
        end = probe->ansam_sample;
    if (end < 0 || (probe->ct_sample >= 0 && probe->ct_sample > end))
        end = probe->ct_sample;
    if (end < 0 || (probe->cng_sample >= 0 && probe->cng_sample > end))
        end = probe->cng_sample;
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
    if (start < 0 || (probe->ct_sample >= 0 && probe->ct_sample < start))
        start = probe->ct_sample;
    if (start < 0 || (probe->cng_sample >= 0 && probe->cng_sample < start))
        start = probe->cng_sample;
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

static int v8_probe_targeted_search_start(const v8_probe_result_t *probe, int total_samples)
{
    int start = -1;

    if (!probe)
        return 0;
    if (probe->ansam_sample >= 0)
        start = probe->ansam_sample + 12000;
    else if (probe->ct_sample >= 0)
        start = probe->ct_sample + 2000;
    else if (probe->cng_sample >= 0)
        start = probe->cng_sample + 2000;
    else if (probe->ci_sample >= 0)
        start = probe->ci_sample;
    else
        start = v8_probe_first_sample(probe);

    if (start < 0)
        start = 0;
    if (total_samples > 0 && start > total_samples)
        start = total_samples;
    return start;
}

static int v8_probe_milestone_count(const v8_probe_result_t *probe)
{
    int count = 0;

    if (!probe)
        return 0;
    if (probe->ansam_sample >= 0)
        count++;
    if (probe->ct_sample >= 0)
        count++;
    if (probe->cng_sample >= 0)
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
        probe ? probe->ct_sample : -1,
        probe ? probe->cng_sample : -1,
        probe ? probe->ci_sample : -1,
        probe ? probe->cm_jm_sample : -1,
        probe ? probe->cj_sample : -1,
        probe ? probe->v8_call_sample : -1
    };

    if (!probe || !probe->ok)
        return -1;

    if (probe->ansam_sample >= 0)
        score += 100;
    if (probe->ct_sample >= 0)
        score += 220;
    if (probe->cng_sample >= 0)
        score += 220;
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
        if (probe->ct_sample >= 0)
            score += 500;
        if (probe->cng_sample >= 0)
            score += 220;
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
        if (probe->ct_sample >= 0)
            score -= 420;
        if (probe->cng_sample >= 0)
            score -= 160;
        if (probe->cm_jm_sample >= 0)
            score += 700;
        if (probe->cj_sample >= 0)
            score -= 220;
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

static int v8_probe_stereo_role_fit(const v8_probe_result_t *probe, bool expect_caller)
{
    int score;

    if (!probe || !probe->ok)
        return -1000000;

    score = v8_probe_role_score(probe);
    if (expect_caller) {
        if (probe->ci_sample >= 0)
            score += 400;
        if (probe->ct_sample >= 0)
            score += 360;
        if (probe->cng_sample >= 0)
            score += 120;
        if (probe->cm_jm_sample >= 0 && probe->calling_party)
            score += probe->cm_jm_complete ? 500 : 220;
        if (probe->cj_sample >= 0)
            score += 220;
        if (probe->cm_jm_sample >= 0 && probe->cj_sample >= probe->cm_jm_sample)
            score += 320;
        if (probe->cj_sample >= 0 && probe->cm_jm_sample < 0)
            score -= 120;
        if (probe->ansam_sample >= 0)
            score -= 260;
    } else {
        if (probe->ansam_sample >= 0)
            score += 420;
        if (probe->ct_sample >= 0)
            score -= 360;
        if (probe->cng_sample >= 0)
            score -= 120;
        if (probe->cm_jm_sample >= 0 && !probe->calling_party)
            score += probe->cm_jm_complete ? 500 : 220;
        if (probe->cj_sample >= 0)
            score -= 180;
        if (probe->ci_sample >= 0)
            score -= 180;
        if (probe->ansam_sample >= 0 && probe->cm_jm_sample >= probe->ansam_sample)
            score += 260;
    }

    return score;
}

static bool v8_select_best_stereo_pair(const int16_t *left_samples,
                                       const int16_t *right_samples,
                                       int total_samples,
                                       int max_sample,
                                       v8_stereo_pair_result_t *out)
{
    v8_probe_result_t left_caller;
    v8_probe_result_t left_answerer;
    v8_probe_result_t right_caller;
    v8_probe_result_t right_answerer;
    bool have_left_caller;
    bool have_left_answerer;
    bool have_right_caller;
    bool have_right_answerer;
    int left_caller_score;
    int left_answerer_score;
    int right_caller_score;
    int right_answerer_score;
    int score_lr;
    int score_rl;

    if (!left_samples || !right_samples || total_samples <= 0 || !out)
        return false;

    memset(out, 0, sizeof(*out));

    have_left_caller = v8_select_best_probe(left_samples, total_samples, true, max_sample, &left_caller);
    have_left_answerer = v8_select_best_probe(left_samples, total_samples, false, max_sample, &left_answerer);
    have_right_caller = v8_select_best_probe(right_samples, total_samples, true, max_sample, &right_caller);
    have_right_answerer = v8_select_best_probe(right_samples, total_samples, false, max_sample, &right_answerer);

    if ((!have_left_caller && !have_left_answerer) || (!have_right_caller && !have_right_answerer))
        return false;

    left_caller_score = have_left_caller ? v8_probe_stereo_role_fit(&left_caller, true) : -1000000;
    left_answerer_score = have_left_answerer ? v8_probe_stereo_role_fit(&left_answerer, false) : -1000000;
    right_caller_score = have_right_caller ? v8_probe_stereo_role_fit(&right_caller, true) : -1000000;
    right_answerer_score = have_right_answerer ? v8_probe_stereo_role_fit(&right_answerer, false) : -1000000;

    score_lr = left_caller_score + right_answerer_score;
    score_rl = left_answerer_score + right_caller_score;

    if (score_lr <= -1000000 && score_rl <= -1000000)
        return false;

    out->ok = true;
    if (score_lr >= score_rl) {
        out->left_is_caller = true;
        out->left_probe = left_caller;
        out->right_probe = right_answerer;
        out->score = score_lr;
    } else {
        out->left_is_caller = false;
        out->left_probe = left_answerer;
        out->right_probe = right_caller;
        out->score = score_rl;
    }
    return true;
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

static const char *v8_aux_tone_name(int tone)
{
    switch (tone) {
    case MODEM_CONNECT_TONES_CALLING_TONE:
        return "CT";
    case MODEM_CONNECT_TONES_FAX_CNG:
        return "CNG";
    default:
        return "Tone";
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
    out->ct_sample = -1;
    out->cng_sample = -1;
    out->cm_jm_complete = false;
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
        if ((tone = modem_connect_tones_rx_get(&v8->calling_tone_rx)) == MODEM_CONNECT_TONES_CALLING_TONE)
            v8_note_first_sample(&out->ct_sample, offset + chunk);
        modem_connect_tones_rx(&v8->cng_tone_rx, samples + offset, chunk);
        if ((tone = modem_connect_tones_rx_get(&v8->cng_tone_rx)) == MODEM_CONNECT_TONES_FAX_CNG)
            v8_note_first_sample(&out->cng_sample, offset + chunk);
        fsk_rx(&v8->v21rx, samples + offset, chunk);
        offset += chunk;

        if (v8->got_ci)
            v8_note_first_sample(&out->ci_sample, offset);
        if (v8->got_cm_jm)
            v8_note_first_sample(&out->cm_jm_sample, offset);
        if (v8->got_cm_jm) {
            out->cm_jm_complete = true;
            out->cm_jm_raw_len = v8->cm_jm_len;
            if (out->cm_jm_raw_len > (int) sizeof(out->cm_jm_raw))
                out->cm_jm_raw_len = (int) sizeof(out->cm_jm_raw);
            if (out->cm_jm_raw_len > 0)
                memcpy(out->cm_jm_raw, v8->cm_jm_data, (size_t) out->cm_jm_raw_len);
            if (out->cm_jm_raw_len > 0) {
                v8_parms_t reparsed;

                if (v8_extract_best_cm_jm_payload(&reparsed,
                                                  out->cm_jm_raw,
                                                  &out->cm_jm_raw_len,
                                                  out->cm_jm_raw,
                                                  out->cm_jm_raw_len,
                                                  calling_party)) {
                    out->result = reparsed;
                    out->last_status = reparsed.status;
                }
            }
        }
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
            && v8_extract_best_cm_jm_payload(&parsed,
                                             raw_hit.bytes,
                                             &raw_hit.byte_len,
                                             raw_hit.bytes,
                                             raw_hit.byte_len,
                                             calling_party)) {
            out->result = parsed;
            out->last_status = parsed.status;
            out->cm_jm_sample = raw_hit.sample_offset;
            out->cm_jm_complete = raw_hit.repeated_confirmed;
            out->cm_jm_salvaged = true;
            out->cm_jm_raw_len = raw_hit.byte_len;
            memcpy(out->cm_jm_raw, raw_hit.bytes, (size_t) raw_hit.byte_len);
        } else if (out->ansam_sample >= 0 || out->ct_sample >= 0 || out->cng_sample >= 0) {
            v8_fsk_burst_hit_t bursts[V8_MAX_TARGETED_BURSTS];
            int burst_count;
            bool have_targeted_candidate = false;

            burst_count = v8_collect_v21_bursts(samples,
                                                total_samples,
                                                8000,
                                                v8_probe_targeted_search_start(out, limit),
                                                limit,
                                                !calling_party,
                                                bursts,
                                                V8_MAX_TARGETED_BURSTS);
            if (burst_count > 0
                && v8_targeted_v21_decode_repeated(samples,
                                                   total_samples,
                                                   8000,
                                                   bursts,
                                                   burst_count,
                                                   !calling_party,
                                                   calling_party,
                                                   &raw_hit)
                && v8_extract_best_cm_jm_payload(&parsed,
                                                 raw_hit.bytes,
                                                 &raw_hit.byte_len,
                                                 raw_hit.bytes,
                                                 raw_hit.byte_len,
                                                 calling_party)) {
                out->result = parsed;
                out->last_status = parsed.status;
                out->cm_jm_sample = raw_hit.sample_offset;
                out->cm_jm_complete = raw_hit.repeated_confirmed;
                out->cm_jm_salvaged = true;
                out->cm_jm_raw_len = raw_hit.byte_len;
                memcpy(out->cm_jm_raw, raw_hit.bytes, (size_t) raw_hit.byte_len);
                have_targeted_candidate = true;
            }
            if (!have_targeted_candidate
                && burst_count > 0
                && v8_targeted_v21_bytes_candidate_repeated(samples,
                                                            total_samples,
                                                            8000,
                                                            bursts,
                                                            burst_count,
                                                            !calling_party,
                                                            calling_party,
                                                            &raw_hit,
                                                            NULL)
                && raw_hit.preamble_type == V8_LOCAL_SYNC_CM_JM
                && v8_extract_best_cm_jm_payload(&parsed,
                                                 raw_hit.bytes,
                                                 &raw_hit.byte_len,
                                                 raw_hit.bytes,
                                                 raw_hit.byte_len,
                                                 calling_party)) {
                out->result = parsed;
                out->last_status = parsed.status;
                out->cm_jm_sample = raw_hit.sample_offset;
                out->cm_jm_complete = raw_hit.repeated_confirmed;
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

    if (calling_party && out->cj_sample < 0 && out->cm_jm_sample >= 0) {
        int cj_search_start = out->cm_jm_sample + 400;
        int cj_search_end = out->cm_jm_sample + 8000;
        int cj_sample;

        if (cj_search_end > limit)
            cj_search_end = limit;
        if (v8_detect_cj_run(samples,
                             total_samples,
                             8000,
                             cj_search_start,
                             cj_search_end,
                             false,
                             &cj_sample)) {
            out->cj_sample = cj_sample;
        }
    }

    out->ok = (out->ansam_sample >= 0
               || out->ct_sample >= 0
               || out->cng_sample >= 0
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

    if (best_score < 0
        && best.ansam_sample < 0
        && best.ct_sample < 0
        && best.cng_sample < 0
        && best.ci_sample < 0
        && best.cm_jm_sample < 0
        && best.cj_sample < 0
        && best.v8_call_sample < 0)
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

static int format_v8_modulations_str(int mods, char *buf, size_t buf_len)
{
    static const struct { int flag; const char *name; } mod_table[] = {
        { V8_MOD_V17,    "V.17"    },
        { V8_MOD_V21,    "V.21"    },
        { V8_MOD_V22,    "V.22bis" },
        { V8_MOD_V23HDX, "V.23hdx" },
        { V8_MOD_V23,    "V.23"    },
        { V8_MOD_V26BIS, "V.26bis" },
        { V8_MOD_V26TER, "V.26ter" },
        { V8_MOD_V27TER, "V.27ter" },
        { V8_MOD_V29,    "V.29"    },
        { V8_MOD_V32,    "V.32bis" },
        { V8_MOD_V34HDX, "V.34hdx" },
        { V8_MOD_V34,    "V.34"    },
        { V8_MOD_V90,    "V.90"    },
        { V8_MOD_V92,    "V.92"    },
    };
    size_t pos = 0;

    if (!buf || buf_len == 0)
        return 0;
    buf[0] = '\0';
    for (size_t i = 0; i < sizeof(mod_table) / sizeof(mod_table[0]); i++) {
        if (mods & mod_table[i].flag) {
            if (pos > 0 && pos + 1 < buf_len)
                buf[pos++] = ',';
            size_t n = strlen(mod_table[i].name);
            if (pos + n < buf_len) {
                memcpy(buf + pos, mod_table[i].name, n);
                pos += n;
            }
        }
    }
    buf[pos] = '\0';
    return (int) pos;
}

static void print_v8_modulations(int mods)
{
    char buf[256];
    format_v8_modulations_str(mods, buf, sizeof(buf));
    printf("  %s\n", buf);
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
    print_v8_capability_note(r);
    if (r->jm_cm.nsf >= 0)
        printf("  NSF:            %s\n", v8_nsf_to_str(r->jm_cm.nsf));
    if (r->jm_cm.t66 >= 0)
        printf("  T.66:           %s\n", v8_t66_to_str(r->jm_cm.t66));
    if (r->v92 >= 0)
        printf("  V.92:           0x%02x\n", r->v92);
    if (r->gateway_mode)
        printf("  Gateway mode:   yes\n");
}

static int v8_next_sample_after(int sample, int a, int b, int c)
{
    int next = -1;
    const int vals[3] = { a, b, c };

    for (int i = 0; i < 3; i++) {
        if (vals[i] < 0 || vals[i] <= sample)
            continue;
        if (next < 0 || vals[i] < next)
            next = vals[i];
    }
    return next;
}

static void print_v8_flow_step(const char *label,
                               const char *channel,
                               int start_sample,
                               int end_sample,
                               const char *suffix)
{
    if (!label || !channel || start_sample < 0)
        return;

    if (end_sample > start_sample) {
        printf("  %-18s %s %.1f-%.1f ms (%.1f ms)%s\n",
               label,
               channel,
               sample_to_ms(start_sample, 8000),
               sample_to_ms(end_sample, 8000),
               sample_to_ms(end_sample - start_sample, 8000),
               suffix ? suffix : "");
    } else {
        printf("  %-18s %s at %.1f ms%s\n",
               label,
               channel,
               sample_to_ms(start_sample, 8000),
               suffix ? suffix : "");
    }
}

static void print_v8_stereo_pair_summary(const v8_stereo_pair_result_t *pair)
{
    const v8_probe_result_t *caller;
    const v8_probe_result_t *answerer;
    const char *caller_ch;
    const char *answerer_ch;
    bool answerer_cj_is_echo = false;
    int first_msg = -1;
    int last_msg = -1;
    int ci_end;
    int cm_end;
    int ansam_end;
    int jm_end;
    int cj_end;

    if (!pair || !pair->ok)
        return;

    caller = pair->left_is_caller ? &pair->left_probe : &pair->right_probe;
    answerer = pair->left_is_caller ? &pair->right_probe : &pair->left_probe;
    caller_ch = pair->left_is_caller ? "Left" : "Right";
    answerer_ch = pair->left_is_caller ? "Right" : "Left";

    printf("\n=== Stereo V.8 Pairing ===\n");
    printf("  Best orientation: %s=caller, %s=answerer (score %d)\n",
           caller_ch, answerer_ch, pair->score);
    printf("  Reconstructed flow:\n");
    ci_end = v8_next_sample_after(caller->ci_sample,
                                  caller->cm_jm_sample,
                                  answerer->cm_jm_sample,
                                  caller->cj_sample);
    cm_end = v8_next_sample_after(caller->cm_jm_sample,
                                  answerer->cm_jm_sample,
                                  caller->cj_sample,
                                  -1);
    ansam_end = v8_next_sample_after(answerer->ansam_sample,
                                     answerer->cm_jm_sample,
                                     caller->cm_jm_sample,
                                     caller->cj_sample);
    jm_end = v8_next_sample_after(answerer->cm_jm_sample,
                                  caller->cj_sample,
                                  -1,
                                  -1);
    cj_end = v8_next_sample_after(caller->cj_sample,
                                  answerer->cj_sample,
                                  -1,
                                  -1);
    answerer_cj_is_echo = v8_samples_near(answerer->cj_sample, caller->cj_sample, 1200);

    if (caller->ci_sample >= 0) {
        print_v8_flow_step("Caller CI", caller_ch, caller->ci_sample, ci_end, "");
        first_msg = caller->ci_sample;
    } else {
        printf("  Caller (%s):      CI not observed\n", caller_ch);
    }
    if (caller->cm_jm_sample >= 0) {
        print_v8_flow_step("Caller CM",
                           caller_ch,
                           caller->cm_jm_sample,
                           cm_end,
                           caller->cm_jm_complete ? "" : " [weak field decode]");
        if (first_msg < 0 || caller->cm_jm_sample < first_msg)
            first_msg = caller->cm_jm_sample;
        if (caller->cm_jm_complete)
            last_msg = caller->cm_jm_sample;
    } else {
        printf("  Caller (%s):      CM not observed\n", caller_ch);
    }
    if (answerer->ansam_sample >= 0) {
        char suffix[48];

        snprintf(suffix, sizeof(suffix), " [%s]", v8_answer_tone_name(answerer->ansam_tone));
        print_v8_flow_step("Answerer tone", answerer_ch, answerer->ansam_sample, ansam_end, suffix);
        if (first_msg < 0 || answerer->ansam_sample < first_msg)
            first_msg = answerer->ansam_sample;
    } else {
        printf("  Answerer (%s):    answer tone not observed\n", answerer_ch);
    }
    if (answerer->cm_jm_sample >= 0) {
        print_v8_flow_step("Answerer JM",
                           answerer_ch,
                           answerer->cm_jm_sample,
                           jm_end,
                           answerer->cm_jm_complete ? "" : " [weak field decode]");
        if (first_msg < 0 || answerer->cm_jm_sample < first_msg)
            first_msg = answerer->cm_jm_sample;
        if (answerer->cm_jm_complete)
            last_msg = answerer->cm_jm_sample;
    } else {
        printf("  Answerer (%s):    JM not observed\n", answerer_ch);
    }
    if (caller->cj_sample >= 0) {
        print_v8_flow_step("Caller CJ", caller_ch, caller->cj_sample, cj_end, "");
        if (last_msg < caller->cj_sample)
            last_msg = caller->cj_sample;
    } else {
        printf("  Caller (%s):      CJ not observed\n", caller_ch);
    }
    if (answerer->cj_sample >= 0 && answerer_cj_is_echo) {
        printf("  Answerer (%s):    CJ-like echo at %.1f ms\n",
               answerer_ch, sample_to_ms(answerer->cj_sample, 8000));
    } else if (answerer->cj_sample >= 0) {
        printf("  Answerer (%s):    CJ also observed at %.1f ms\n",
               answerer_ch, sample_to_ms(answerer->cj_sample, 8000));
    }
    if (caller->cm_jm_sample >= 0 && answerer->cm_jm_sample < 0) {
        printf("  Exchange state:   caller-side CM observed; no clear JM observed on %s\n",
               answerer_ch);
    } else if (caller->cm_jm_sample < 0 && answerer->cm_jm_sample >= 0) {
        printf("  Exchange state:   answerer-side JM observed; caller CM is missing or buried by echo\n");
    } else if (caller->cm_jm_sample >= 0 && answerer->cm_jm_sample >= 0 && caller->cj_sample < 0) {
        printf("  Exchange state:   CM/JM reconstructed; caller-side CJ not observed\n");
    } else if (caller->cm_jm_sample >= 0 && answerer->cm_jm_sample >= 0 && caller->cj_sample >= 0) {
        printf("  Exchange state:   CM/JM/CJ sequence reconstructed across stereo pair\n");
    }
    if (first_msg >= 0 && last_msg >= first_msg) {
        printf("  Exchange span:    %.1f ms to %.1f ms\n",
               sample_to_ms(first_msg, 8000),
               sample_to_ms(last_msg, 8000));
    }
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
    int capture_start_bit;
    int capture_len;
    char capture[8193];
} v34_aux_bit_collector_t;

static void v34_collect_aux_bit(void *user_data, int bit)
{
    v34_aux_bit_collector_t *collector = (v34_aux_bit_collector_t *) user_data;

    if (!collector)
        return;
    if (collector->capture_start_bit >= 0
        && collector->total_bits >= collector->capture_start_bit
        && collector->capture_len < (int) sizeof(collector->capture) - 1) {
        collector->capture[collector->capture_len++] = bit ? '1' : '0';
        collector->capture[collector->capture_len] = '\0';
    }
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

typedef struct {
    bool evaluated;
    bool sequence_ok;
    bool trn_min_ok;
    bool handoff_ok;
    bool pass;
    int trn_t;
    int s1_t;
    int s2_t;
    char notes[256];
} v34_phase3_errorfree_eval_t;

static int v34_samples_to_t_interval(int start_sample, int end_sample)
{
    if (start_sample < 0 || end_sample < 0 || end_sample <= start_sample)
        return -1;
    return samples_to_v34_symbols(end_sample - start_sample);
}

static void v34_phase3_eval_append_note(v34_phase3_errorfree_eval_t *out, const char *text)
{
    if (!out || !text || !text[0])
        return;
    if (out->notes[0])
        appendf(out->notes, sizeof(out->notes), "; ");
    appendf(out->notes, sizeof(out->notes), "%s", text);
}

static void v34_evaluate_errorfree_phase3(const decode_v34_result_t *result,
                                          v34_phase3_errorfree_eval_t *out)
{
    bool sequence_ok = true;
    bool handoff_ok = true;
    bool trn_min_ok = false;
    bool pp_observed_or_inferred;

    if (!out)
        return;
    memset(out, 0, sizeof(*out));
    out->s1_t = -1;
    out->s2_t = -1;
    out->trn_t = -1;

    if (!result)
        return;
    out->evaluated = true;

    pp_observed_or_inferred = (result->tx_pp_sample >= 0
                               || result->tx_second_not_s_sample >= 0
                               || result->tx_trn_sample >= 0);
    if (!pp_observed_or_inferred) {
        sequence_ok = false;
        v34_phase3_eval_append_note(out, "missing PP");
    } else if (result->tx_pp_sample < 0) {
        v34_phase3_eval_append_note(out, "PP inferred from later stages");
    }
    if (result->tx_trn_sample < 0) {
        sequence_ok = false;
        v34_phase3_eval_append_note(out, "missing TRN");
    }
    if (result->tx_ja_sample < 0)
        v34_phase3_eval_append_note(out, "missing J/Ja");

    if (result->tx_pp_sample >= 0 && result->tx_trn_sample >= 0
        && result->tx_pp_sample > result->tx_trn_sample) {
        sequence_ok = false;
        v34_phase3_eval_append_note(out, "PP->TRN order");
    }
    if (result->tx_trn_sample >= 0 && result->tx_ja_sample >= 0
        && result->tx_trn_sample > result->tx_ja_sample) {
        sequence_ok = false;
        v34_phase3_eval_append_note(out, "TRN->J order");
    }
    if (result->tx_first_s_sample >= 0 && result->tx_first_not_s_sample >= 0
        && result->tx_first_s_sample > result->tx_first_not_s_sample) {
        sequence_ok = false;
        v34_phase3_eval_append_note(out, "S->S' order");
    }
    if (result->tx_second_s_sample >= 0 && result->tx_second_not_s_sample >= 0
        && result->tx_second_s_sample > result->tx_second_not_s_sample) {
        sequence_ok = false;
        v34_phase3_eval_append_note(out, "S2->S2' order");
    }

    out->s1_t = v34_samples_to_t_interval(result->tx_first_s_sample,
                                          result->tx_first_not_s_sample);
    out->s2_t = v34_samples_to_t_interval(result->tx_second_s_sample,
                                          result->tx_second_not_s_sample);
    out->trn_t = v34_samples_to_t_interval(result->tx_trn_sample,
                                           result->tx_ja_sample);

    if (out->trn_t >= 512) {
        trn_min_ok = true;
    } else if (out->trn_t >= 0) {
        v34_phase3_eval_append_note(out, "TRN<512T");
    } else if (result->tx_trn_sample >= 0 || result->tx_ja_sample >= 0) {
        v34_phase3_eval_append_note(out, "TRN span unknown");
    }

    if (result->rx_s_event_sample < 0 && !result->phase4_seen && !result->phase4_ready_seen) {
        handoff_ok = false;
        v34_phase3_eval_append_note(out, "no far-end S/Phase4 handoff");
    } else if (result->rx_s_event_sample >= 0 && result->tx_ja_sample >= 0
               && result->rx_s_event_sample < result->tx_ja_sample) {
        handoff_ok = false;
        v34_phase3_eval_append_note(out, "far-end S before J/Ja");
    }

    out->sequence_ok = sequence_ok;
    out->trn_min_ok = trn_min_ok;
    out->handoff_ok = handoff_ok;
    out->pass = sequence_ok && trn_min_ok && handoff_ok;
}

static bool should_emit_phase2_event(int sample, int latest_allowed_sample)
{
    if (sample < 0)
        return false;
    if (latest_allowed_sample < 0)
        return true;
    return sample <= latest_allowed_sample;
}

static int ru_window_two_point_score(const uint8_t *symbols, int len)
{
    int counts[4] = {0, 0, 0, 0};
    int best = 0;
    int second = 0;

    if (!symbols || len <= 0)
        return -1;
    for (int i = 0; i < len; i++) {
        int s = symbols[i] & 0x3;
        counts[s]++;
    }
    for (int i = 0; i < 4; i++) {
        if (counts[i] > best) {
            second = best;
            best = counts[i];
        } else if (counts[i] > second) {
            second = counts[i];
        }
    }
    /* Reward two-point occupancy and balance between the two points.
       Single-point windows should score near 0. */
    if (best + second <= 0)
        return 0;
    {
        int occupancy = ((best + second) * 100 + len / 2) / len;
        int balance = (second * 100 + (best + second) / 2) / (best + second);
        return (occupancy * balance + 50) / 100;
    }
}

static int v92_ru_flow_min_score(const decode_v34_result_t *result)
{
    int min_score = 40;

    if (!result)
        return min_score;

    /* If INFO0 indicates short Phase 2 desire, allow slightly weaker Ru windows. */
    if (result->info0_seen
        && v92_short_phase2_req_from_info0_bits(result->info0_is_d,
                                                result->info0_raw.raw_26_27)) {
        min_score -= 2;
    }

    /* Strong local Phase 3 anchors increase confidence in the Ru interpretation. */
    if (result->tx_trn_sample >= 0)
        min_score -= 2;
    if (result->tx_ja_sample >= 0)
        min_score -= 2;

    /* RX PP/TRN lock evidence supports this being true Phase 3 training. */
    if (result->rx_pp_started)
        min_score -= 2;
    if (result->phase3_trn_lock_score >= 50)
        min_score -= 2;

    if (min_score < 34)
        min_score = 34;
    return min_score;
}

typedef struct {
    bool found;
    int sd_sample;
    int sd_reps;
    int sd_t;
    int ds_sample;
    int ds_reps;
    int ds_t;
} v90_sd_ds_hit_t;

typedef struct {
    bool found;
    int bit_offset;
    bool invert_bits;
    v90_dil_desc_t desc;
    v90_dil_analysis_t analysis;
} v90_aux_dil_hit_t;

static bool find_v90_dil_in_aux_str(const char *bit_str,
                                    int bit_len,
                                    v90_aux_dil_hit_t *out)
{
    int max_offset;
    int best_score = -1;
    v90_aux_dil_hit_t best;
    uint8_t packed[1024];

    if (!bit_str || !out || bit_len < 206 || bit_str[0] == '\0')
        return false;

    memset(&best, 0, sizeof(best));
    if (bit_len > 8192)
        bit_len = 8192;
    max_offset = bit_len - 206;

    for (int offset = 0; offset <= max_offset; offset++) {
        int candidate_bits = bit_len - offset;
        int packed_len = (candidate_bits + 7) / 8;

        if (packed_len <= 0 || packed_len > (int) sizeof(packed))
            continue;
        for (int swap_pairs = 0; swap_pairs <= 1; swap_pairs++) {
            if (swap_pairs && candidate_bits < 2)
                continue;
            for (int invert = 0; invert <= 1; invert++) {
            v90_dil_desc_t desc;
            v90_dil_analysis_t analysis;
            int score;
            bool valid = true;

            memset(packed, 0, (size_t) packed_len);
            for (int i = 0; i < candidate_bits; i++) {
                int bit;
                int src_i = offset + i;
                char c;

                if (swap_pairs)
                    src_i = offset + (i & ~1) + (1 - (i & 1));
                c = bit_str[src_i];

                if (c != '0' && c != '1') {
                    valid = false;
                    break;
                }
                bit = (c == '1') ? 1 : 0;
                if (invert)
                    bit ^= 1;
                if (bit)
                    packed[i / 8] |= (uint8_t) (1U << (i % 8));
            }
            if (!valid)
                continue;
            if (!v90_parse_dil_descriptor(&desc, packed, candidate_bits))
                continue;
            if (!v90_analyse_dil_descriptor(&desc, &analysis))
                continue;

            score = analysis.unique_train_u * 100
                  + analysis.used_uchords * 50
                  - analysis.impairment_score * 10
                  - analysis.non_default_h * 5
                  - offset / 4
                  - (swap_pairs ? 8 : 0);
            if (score > best_score) {
                best_score = score;
                best.found = true;
                best.bit_offset = offset;
                best.invert_bits = (invert != 0);
                best.desc = desc;
                best.analysis = analysis;
            }
        }
        }
    }

    if (!best.found)
        return false;
    *out = best;
    return true;
}

static int v90_aux_dil_hit_score(const v90_aux_dil_hit_t *hit)
{
    if (!hit || !hit->found)
        return INT_MIN;
    return hit->analysis.unique_train_u * 100
         + hit->analysis.used_uchords * 50
         - hit->analysis.impairment_score * 10
         - hit->analysis.non_default_h * 5
         - hit->bit_offset / 4
         - (hit->invert_bits ? 4 : 0);
}

static void v34_descramble_bits(const char *src_bits,
                                int bit_len,
                                int scrambler_tap,
                                char *out_bits,
                                int out_cap)
{
    uint32_t reg = 0;
    int n;

    if (!out_bits || out_cap <= 0)
        return;
    if (!src_bits || bit_len <= 0) {
        out_bits[0] = '\0';
        return;
    }
    n = bit_len;
    if (n > out_cap - 1)
        n = out_cap - 1;
    for (int i = 0; i < n; i++) {
        int in_bit = (src_bits[i] == '1') ? 1 : 0;
        int out_bit = (in_bit ^ ((int) (reg >> scrambler_tap)) ^ ((int) (reg >> (23 - 1)))) & 1;

        reg = (reg << 1) | (uint32_t) in_bit;
        out_bits[i] = out_bit ? '1' : '0';
    }
    out_bits[n] = '\0';
}

static bool find_v90_dil_in_aux_any(const decode_v34_result_t *src,
                                    v90_aux_dil_hit_t *out,
                                    int *hyp_idx_out,
                                    int *tap_out,
                                    bool *raw_out)
{
    int best_score = INT_MIN;
    v90_aux_dil_hit_t best_hit;
    int best_hyp = -1;
    int best_tap = -1;
    bool best_raw = false;

    if (!src || !out)
        return false;
    memset(&best_hit, 0, sizeof(best_hit));

    if (find_v90_dil_in_aux_str(src->ja_aux_bits, src->ja_aux_bit_len, out)) {
        int score = v90_aux_dil_hit_score(out);

        best_score = score;
        best_hit = *out;
        best_hyp = -1;
        best_tap = -1;
        best_raw = false;
    }

    /*
     * The ja_aux_bits are raw QAM-demodulated sign bits from SpanDSP's V.34
     * receiver.  The V.90 DIL is encoded with the x^23+x^5+1 scrambler before
     * being placed into the V.34 J-phase payload, so we must descramble before
     * parsing.  Try both possible V.90 scrambler tap configurations (tap 4 =
     * x^5, the standard V.90 polynomial; tap 17 as an alternative).
     */
    {
        static const int taps[2] = {4, 17};
        char descrambled[8193];

        for (int ti = 0; ti < 2; ti++) {
            v90_aux_dil_hit_t hit;
            int score;

            memset(&hit, 0, sizeof(hit));
            if (src->ja_aux_bit_len < 206 || src->ja_aux_bits[0] == '\0')
                break;
            v34_descramble_bits(src->ja_aux_bits, src->ja_aux_bit_len,
                                taps[ti], descrambled, (int) sizeof(descrambled));
            if (!find_v90_dil_in_aux_str(descrambled, src->ja_aux_bit_len, &hit))
                continue;
            score = v90_aux_dil_hit_score(&hit) + 1; /* slight preference over raw */
            if (score > best_score) {
                best_score = score;
                best_hit = hit;
                best_hyp = -1;
                best_tap = taps[ti];
                best_raw = true;
            }
        }
    }

    for (int h = 0; h < 8; h++) {
        v90_aux_dil_hit_t hit;
        int score;

        memset(&hit, 0, sizeof(hit));
        if (!find_v90_dil_in_aux_str(src->ja_aux_hyp_bits[h], src->ja_aux_hyp_bit_len[h], &hit))
            continue;
        score = v90_aux_dil_hit_score(&hit);
        if (score > best_score) {
            best_score = score;
            best_hit = hit;
            best_hyp = h;
            best_tap = -1;
            best_raw = false;
        }
    }
    for (int h = 0; h < 8; h++) {
        static const int taps[2] = {17, 4};

        for (int ti = 0; ti < 2; ti++) {
            char descrambled[8193];
            v90_aux_dil_hit_t hit;
            int score;

            memset(&hit, 0, sizeof(hit));
            if (src->ja_aux_hyp_raw_bit_len[h] < 206 || src->ja_aux_hyp_raw_bits[h][0] == '\0')
                continue;
            v34_descramble_bits(src->ja_aux_hyp_raw_bits[h],
                                src->ja_aux_hyp_raw_bit_len[h],
                                taps[ti],
                                descrambled,
                                (int) sizeof(descrambled));
            if (!find_v90_dil_in_aux_str(descrambled, src->ja_aux_hyp_raw_bit_len[h], &hit))
                continue;
            score = v90_aux_dil_hit_score(&hit) + 2;
            if (score > best_score) {
                best_score = score;
                best_hit = hit;
                best_hyp = h;
                best_tap = taps[ti];
                best_raw = true;
            }
        }
    }
    if (best_score == INT_MIN)
        return false;
    *out = best_hit;
    if (hyp_idx_out)
        *hyp_idx_out = best_hyp;
    if (tap_out)
        *tap_out = best_tap;
    if (raw_out)
        *raw_out = best_raw;
    return true;
}

static bool find_v90_sd_ds_after(const uint8_t *codewords,
                                 int total_codewords,
                                 v91_law_t law,
                                 int u_info,
                                 int search_start,
                                 int search_end,
                                 v90_sd_ds_hit_t *out)
{
    uint8_t pos_zero;
    uint8_t neg_zero;
    int w_ucode;
    uint8_t pos_w;
    uint8_t neg_w;
    uint8_t sd_pat[6];
    uint8_t ds_pat[6];
    v90_sd_ds_hit_t best;

    if (!codewords || total_codewords <= 0 || !out || u_info < 10 || u_info > 127)
        return false;

    memset(&best, 0, sizeof(best));
    if (search_start < 0)
        search_start = 0;
    if (search_end <= 0 || search_end > total_codewords)
        search_end = total_codewords;
    if (search_end - search_start < 24)
        return false;

    pos_zero = v91_ucode_to_codeword(law, 0, true);
    neg_zero = v91_ucode_to_codeword(law, 0, false);
    w_ucode = 16 + u_info;
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
    ds_pat[0] = neg_w;
    ds_pat[1] = neg_zero;
    ds_pat[2] = neg_w;
    ds_pat[3] = pos_w;
    ds_pat[4] = pos_zero;
    ds_pat[5] = pos_w;

    for (int offset = search_start; offset + 24 <= search_end; offset++) {
        int sd_reps = 0;
        int ds_reps = 0;
        int pos;

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
        while (offset + (sd_reps + 1) * 6 <= search_end) {
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
        if (sd_reps < 8)
            continue;

        pos = offset + sd_reps * 6;
        while (pos + (ds_reps + 1) * 6 <= search_end) {
            bool ok = true;

            for (int j = 0; j < 6; j++) {
                if (codewords[pos + ds_reps * 6 + j] != ds_pat[j]) {
                    ok = false;
                    break;
                }
            }
            if (!ok)
                break;
            ds_reps++;
        }

        if (!best.found
            || sd_reps > best.sd_reps
            || (sd_reps == best.sd_reps && ds_reps > best.ds_reps)
            || (sd_reps == best.sd_reps && ds_reps == best.ds_reps && offset < best.sd_sample)) {
            best.found = true;
            best.sd_sample = offset;
            best.sd_reps = sd_reps;
            best.sd_t = sd_reps * 6;
            best.ds_sample = ds_reps > 0 ? pos : -1;
            best.ds_reps = ds_reps;
            best.ds_t = ds_reps * 6;
        }
    }

    if (!best.found)
        return false;
    *out = best;
    return true;
}

static const decode_v34_result_t *pick_analogue_phase2_side(const decode_v34_result_t *answerer,
                                                             const decode_v34_result_t *caller);

static void collect_v92_phase3_event(call_log_t *log,
                                     const decode_v34_result_t *res,
                                     const char *role_name,
                                     int latest_allowed_sample,
                                     const uint8_t *codewords,
                                     int total_codewords,
                                     v91_law_t law,
                                     const decode_v34_result_t *answerer,
                                     const decode_v34_result_t *caller)
{
    v92_phase3_observation_t obs;
    v92_phase3_result_t phase3;
    const decode_v34_result_t *peer = NULL;
    const decode_v34_result_t *analogue_side = NULL;
    const decode_v34_result_t *ja_aux_src = NULL;
    int ja_aux_hyp = -2;
    int ja_aux_tap = -1;
    bool ja_aux_raw = false;
    uint8_t raw_26_27;
    int event_sample;
    const char *ja_anchor_source = "local";
    const char *ja_dil_source = "none";
    char ja_dil_source_buf[32];
    char detail[8192];

    if (!log || !res || !res->info0_seen)
        return;
    if (!should_emit_phase2_event(res->info0_sample, latest_allowed_sample))
        return;

    raw_26_27 = res->info0_raw.raw_26_27;
    if (res == answerer && caller)
        peer = caller;
    else if (res == caller && answerer)
        peer = answerer;
    analogue_side = pick_analogue_phase2_side(answerer, caller);
    memset(&obs, 0, sizeof(obs));
    ja_dil_source_buf[0] = '\0';
    obs.info0_seen = res->info0_seen;
    obs.info0_is_d = res->info0_is_d;
    obs.short_phase2_requested = v92_short_phase2_req_from_info0_bits(res->info0_is_d, raw_26_27);
    obs.v92_capable = v92_short_phase2_v92_cap_from_info0_bits(res->info0_is_d, raw_26_27);
    obs.phase3_seen = res->phase3_seen;
    obs.phase4_seen = res->phase4_seen;
    obs.training_failed = res->training_failed;
    obs.info0_sample = res->info0_sample;
    obs.phase3_sample = res->phase3_sample;
    obs.phase4_sample = res->phase4_sample;
    obs.tx_first_s_sample = res->tx_first_s_sample;
    obs.tx_first_not_s_sample = res->tx_first_not_s_sample;
    obs.tx_md_sample = res->tx_md_sample;
    obs.tx_second_s_sample = res->tx_second_s_sample;
    obs.tx_second_not_s_sample = res->tx_second_not_s_sample;
    obs.tx_pp_sample = res->tx_pp_sample;
    obs.tx_trn_sample = res->tx_trn_sample;
    obs.tx_ja_sample = res->tx_ja_sample;
    obs.tx_jdashed_sample = res->tx_jdashed_sample;
    if (analogue_side) {
        if (obs.tx_trn_sample < 0 && analogue_side->tx_trn_sample >= 0) {
            obs.tx_trn_sample = analogue_side->tx_trn_sample;
            ja_anchor_source = (analogue_side == res) ? "local" : "peer";
        }
        if (obs.tx_ja_sample < 0 && analogue_side->tx_ja_sample >= 0) {
            obs.tx_ja_sample = analogue_side->tx_ja_sample;
            ja_anchor_source = (analogue_side == res) ? "local" : "peer";
        }
        if (obs.tx_jdashed_sample < 0 && analogue_side->tx_jdashed_sample >= 0)
            obs.tx_jdashed_sample = analogue_side->tx_jdashed_sample;
    }
    if (peer) {
        if (obs.tx_trn_sample < 0 && peer->tx_trn_sample >= 0) {
            obs.tx_trn_sample = peer->tx_trn_sample;
            ja_anchor_source = "peer";
        }
        if (obs.tx_ja_sample < 0 && peer->tx_ja_sample >= 0) {
            obs.tx_ja_sample = peer->tx_ja_sample;
            ja_anchor_source = "peer";
        }
        if (obs.tx_jdashed_sample < 0 && peer->tx_jdashed_sample >= 0)
            obs.tx_jdashed_sample = peer->tx_jdashed_sample;
    }
    if (obs.tx_ja_sample < 0)
        ja_anchor_source = "none";
    obs.rx_s_event_sample = res->rx_s_event_sample;
    obs.ru_window_len = res->ru_window_len;
    obs.ru_window_score = res->ru_window_score;
    obs.trn1u_mag_mean = res->trn1u_mag_mean;
    obs.trn1u_mag_count = res->trn1u_mag_count;
    if (obs.ru_window_len > 32)
        obs.ru_window_len = 32;
    if (obs.ru_window_len > 0) {
        memcpy(obs.ru_window_symbols, res->ru_window_symbols, (size_t) obs.ru_window_len);
        memcpy(obs.ru_window_mags, res->ru_window_mags, (size_t) obs.ru_window_len * sizeof(float));
    }
    if (codewords && total_codewords > 0) {
        jd_stage_decode_t jd_stage;
        ja_dil_decode_t ja_dil;
        memset(&jd_stage, 0, sizeof(jd_stage));
        memset(&ja_dil, 0, sizeof(ja_dil));

        /*
         * V.90: Jd capability frames precede the DIL — decode Jd first so
         *       its timing anchors the Ja DIL search window.
         * V.92: There is no Jd; the analog modem moves directly from TRN1u
         *       to Ja.  Skip Jd entirely and rely on tx_ja_sample instead.
         */
        if (!obs.v92_capable)
            decode_jd_stage(codewords, total_codewords, answerer, caller, &jd_stage);

        if (decode_ja_dil_stage(codewords, total_codewords, answerer, caller, &jd_stage, &ja_dil)
            && ja_dil.ok) {
            int bit_len = v90_dil_descriptor_bit_len(&ja_dil.desc);

            if (bit_len > 0) {
                obs.ja_dil_seen = true;
                obs.ja_dil_sample = ja_dil.start_sample;
                obs.ja_dil_bits = bit_len;
                obs.ja_dil_n = ja_dil.desc.n;
                obs.ja_dil_lsp = ja_dil.desc.lsp;
                obs.ja_dil_ltp = ja_dil.desc.ltp;
                obs.ja_dil_unique_train_u = ja_dil.analysis.unique_train_u;
                obs.ja_dil_uchords = ja_dil.analysis.used_uchords;
                obs.ja_dil_impairment = ja_dil.analysis.impairment_score;
                ja_dil_source = "pcm";
            }
        }

        if (res->u_info >= 10 && res->u_info <= 127) {
            v90_sd_ds_hit_t sd_ds;
            int search_start = obs.ja_dil_seen
                               ? (obs.ja_dil_sample + obs.ja_dil_bits)
                               : (res->tx_ja_sample >= 0 ? res->tx_ja_sample : res->info1_sample);
            int search_end = search_start + 12000;

            if (search_start < 0)
                search_start = 0;
            if (search_end > total_codewords)
                search_end = total_codewords;
            memset(&sd_ds, 0, sizeof(sd_ds));
            if (find_v90_sd_ds_after(codewords,
                                     total_codewords,
                                     law,
                                     res->u_info,
                                     search_start,
                                     search_end,
                                     &sd_ds)) {
                obs.sd_seen = true;
                obs.sd_sample = sd_ds.sd_sample;
                obs.sd_t = sd_ds.sd_t;
                obs.sd_reps = sd_ds.sd_reps;
                obs.ds_seen = sd_ds.ds_reps > 0;
                obs.ds_sample = sd_ds.ds_sample;
                obs.ds_t = sd_ds.ds_t;
                obs.ds_reps = sd_ds.ds_reps;
            }
        }
    }
    if (!obs.ja_dil_seen) {
        v90_aux_dil_hit_t aux_hit;
        const decode_v34_result_t *primary_aux = NULL;
        const decode_v34_result_t *secondary_aux = NULL;

        memset(&aux_hit, 0, sizeof(aux_hit));
        primary_aux = analogue_side ? analogue_side : res;
        if (primary_aux == res)
            secondary_aux = peer;
        else if (primary_aux == peer)
            secondary_aux = res;
        else
            secondary_aux = res;

        if (primary_aux && find_v90_dil_in_aux_any(primary_aux, &aux_hit, &ja_aux_hyp, &ja_aux_tap, &ja_aux_raw)) {
            ja_aux_src = primary_aux;
        } else if (secondary_aux && find_v90_dil_in_aux_any(secondary_aux, &aux_hit, &ja_aux_hyp, &ja_aux_tap, &ja_aux_raw)) {
            ja_aux_src = secondary_aux;
        }

        if (ja_aux_src && aux_hit.found) {
            int bit_len = v90_dil_descriptor_bit_len(&aux_hit.desc);
            int bit_to_sample = (aux_hit.bit_offset * 5 + 2) / 4;
            int base_sample = (ja_aux_src->tx_ja_sample >= 0)
                              ? ja_aux_src->tx_ja_sample
                              : (obs.tx_ja_sample >= 0 ? obs.tx_ja_sample : res->info1_sample);

            if (bit_len > 0 && base_sample >= 0) {
                obs.ja_dil_seen = true;
                if (ja_aux_hyp >= 0) {
                    snprintf(ja_dil_source_buf,
                             sizeof(ja_dil_source_buf),
                             "%s_h%d%s",
                             (ja_aux_src == res) ? "ja_aux_local" : "ja_aux_peer",
                             ja_aux_hyp,
                             ja_aux_raw ? (ja_aux_tap == 4 ? "_rawtap4" : "_rawtap17") : "");
                    ja_dil_source = ja_dil_source_buf;
                } else {
                    ja_dil_source = (ja_aux_src == res) ? "ja_aux_local" : "ja_aux_peer";
                }
                obs.ja_dil_sample = base_sample + bit_to_sample;
                obs.ja_dil_bits = bit_len;
                obs.ja_dil_n = aux_hit.desc.n;
                obs.ja_dil_lsp = aux_hit.desc.lsp;
                obs.ja_dil_ltp = aux_hit.desc.ltp;
                obs.ja_dil_unique_train_u = aux_hit.analysis.unique_train_u;
                obs.ja_dil_uchords = aux_hit.analysis.used_uchords;
                obs.ja_dil_impairment = aux_hit.analysis.impairment_score;
            }
        }
    }

    if (!v92_phase3_analyze(&obs, &phase3))
        return;

    event_sample = phase3.ru_sample;
    if (event_sample < 0)
        event_sample = res->phase3_sample;
    if (!should_emit_phase2_event(event_sample, latest_allowed_sample))
        return;

    snprintf(detail, sizeof(detail),
             "role=%s spec=V92_tables15_19 local_modem=%s shortp2_req=%u v92_cap=%u ru_source=%s status=%s",
             role_name ? role_name : "unknown",
             v92_phase3_role_id(phase3.local_role),
             phase3.short_phase2_requested ? 1U : 0U,
             phase3.v92_capable ? 1U : 0U,
             v92_phase3_ru_source_id(phase3.ru_source),
             phase3.status ? phase3.status : "unknown");
    appendf(detail, sizeof(detail), " ru_ms=%s", phase3.ru_sample >= 0 ? "" : "n/a");
    if (phase3.ru_sample >= 0)
        appendf(detail, sizeof(detail), "%.1f", sample_to_ms(phase3.ru_sample, 8000));
    appendf(detail, sizeof(detail), " phase3_ms=%s", phase3.phase3_sample >= 0 ? "" : "n/a");
    if (phase3.phase3_sample >= 0)
        appendf(detail, sizeof(detail), "%.1f", sample_to_ms(phase3.phase3_sample, 8000));
    appendf(detail, sizeof(detail), " phase4_ms=%s", phase3.phase4_sample >= 0 ? "" : "n/a");
    if (phase3.phase4_sample >= 0)
        appendf(detail, sizeof(detail), "%.1f", sample_to_ms(phase3.phase4_sample, 8000));
    appendf(detail, sizeof(detail), " phase4_status=%s",
            phase3.phase4_status ? phase3.phase4_status : "unknown");
    appendf(detail, sizeof(detail), " ja_anchor_source=%s", ja_anchor_source);
    appendf(detail, sizeof(detail), " ja_aux_bits_local=%d", res->ja_aux_bit_len);
    appendf(detail, sizeof(detail), " ja_aux_bits_peer=%d", peer ? peer->ja_aux_bit_len : 0);
    appendf(detail, sizeof(detail), " trn_to_ja_t=%s",
            phase3.ja_seen && phase3.trnlu_to_ja_t > 0 ? "" : "n/a");
    if (phase3.ja_seen && phase3.trnlu_to_ja_t > 0)
        appendf(detail, sizeof(detail), "%d", phase3.trnlu_to_ja_t);
    appendf(detail, sizeof(detail), " trn_first_2040t=%d", phase3.trnlu_first_2040_t);
    appendf(detail, sizeof(detail), " trn_2040t_ready=%u",
            phase3.trnlu_2040_t_ready_for_ja ? 1U : 0U);
    appendf(detail, sizeof(detail), " ja_ms=%s", phase3.ja_sample >= 0 ? "" : "n/a");
    if (phase3.ja_sample >= 0)
        appendf(detail, sizeof(detail), "%.1f", sample_to_ms(phase3.ja_sample, 8000));
    appendf(detail, sizeof(detail), " ja_dil_seen=%u", phase3.ja_dil_seen ? 1U : 0U);
    appendf(detail, sizeof(detail), " ja_dil_source=%s", ja_dil_source);
    appendf(detail, sizeof(detail), " ja_dil_ms=%s", phase3.ja_dil_sample >= 0 ? "" : "n/a");
    if (phase3.ja_dil_sample >= 0)
        appendf(detail, sizeof(detail), "%.1f", sample_to_ms(phase3.ja_dil_sample, 8000));
    appendf(detail, sizeof(detail), " ja_dil_bits=%s", phase3.ja_dil_seen ? "" : "n/a");
    if (phase3.ja_dil_seen)
        appendf(detail, sizeof(detail), "%d", phase3.ja_dil_bits);
    appendf(detail, sizeof(detail), " ja_dil_end_ms=%s", phase3.ja_dil_end_sample >= 0 ? "" : "n/a");
    if (phase3.ja_dil_end_sample >= 0)
        appendf(detail, sizeof(detail), "%.1f", sample_to_ms(phase3.ja_dil_end_sample, 8000));
    appendf(detail, sizeof(detail), " ja_n=%s", phase3.ja_dil_seen ? "" : "n/a");
    if (phase3.ja_dil_seen)
        appendf(detail, sizeof(detail), "%d", phase3.ja_dil_n);
    appendf(detail, sizeof(detail), " ja_lsp=%s", phase3.ja_dil_seen ? "" : "n/a");
    if (phase3.ja_dil_seen)
        appendf(detail, sizeof(detail), "%d", phase3.ja_dil_lsp);
    appendf(detail, sizeof(detail), " ja_ltp=%s", phase3.ja_dil_seen ? "" : "n/a");
    if (phase3.ja_dil_seen)
        appendf(detail, sizeof(detail), "%d", phase3.ja_dil_ltp);
    appendf(detail, sizeof(detail), " ja_unique_u=%s", phase3.ja_dil_seen ? "" : "n/a");
    if (phase3.ja_dil_seen)
        appendf(detail, sizeof(detail), "%d", phase3.ja_dil_unique_train_u);
    appendf(detail, sizeof(detail), " ja_uchords=%s", phase3.ja_dil_seen ? "" : "n/a");
    if (phase3.ja_dil_seen)
        appendf(detail, sizeof(detail), "%d", phase3.ja_dil_uchords);
    appendf(detail, sizeof(detail), " ja_impairment=%s", phase3.ja_dil_seen ? "" : "n/a");
    if (phase3.ja_dil_seen)
        appendf(detail, sizeof(detail), "%d", phase3.ja_dil_impairment);
    appendf(detail, sizeof(detail), " sd_ms=%s", phase3.sd_seen ? "" : "n/a");
    if (phase3.sd_seen)
        appendf(detail, sizeof(detail), "%.1f", sample_to_ms(phase3.sd_sample, 8000));
    appendf(detail, sizeof(detail), " sd_t=%s", phase3.sd_seen ? "" : "n/a");
    if (phase3.sd_seen)
        appendf(detail, sizeof(detail), "%d", phase3.sd_t);
    appendf(detail, sizeof(detail), " sd_reps=%s", phase3.sd_seen ? "" : "n/a");
    if (phase3.sd_seen)
        appendf(detail, sizeof(detail), "%d", phase3.sd_reps);
    appendf(detail, sizeof(detail), " ds_ms=%s", phase3.ds_seen ? "" : "n/a");
    if (phase3.ds_seen)
        appendf(detail, sizeof(detail), "%.1f", sample_to_ms(phase3.ds_sample, 8000));
    appendf(detail, sizeof(detail), " ds_t=%s", phase3.ds_seen ? "" : "n/a");
    if (phase3.ds_seen)
        appendf(detail, sizeof(detail), "%d", phase3.ds_t);
    appendf(detail, sizeof(detail), " ds_reps=%s", phase3.ds_seen ? "" : "n/a");
    if (phase3.ds_seen)
        appendf(detail, sizeof(detail), "%d", phase3.ds_reps);
    appendf(detail, sizeof(detail), " sd_wait_ms=%s",
            phase3.ja_dil_end_sample >= 0 && phase3.sd_seen ? "" : "n/a");
    if (phase3.ja_dil_end_sample >= 0 && phase3.sd_seen)
        appendf(detail, sizeof(detail), "%d", phase3.sd_wait_after_ja_dil_ms);
    appendf(detail, sizeof(detail), " sd_wait_le_500ms=%u",
            phase3.sd_wait_within_500ms ? 1U : 0U);
    appendf(detail, sizeof(detail), " sd_384t_ok=%u",
            phase3.sd_duration_384t_ok ? 1U : 0U);
    appendf(detail, sizeof(detail), " ds_48t_ok=%u",
            phase3.ds_duration_48t_ok ? 1U : 0U);
    appendf(detail, sizeof(detail), " ja_status=%s",
            phase3.ja_status ? phase3.ja_status : "unknown");
    appendf(detail, sizeof(detail), " ru_pattern_primary=%s",
            phase3.ru_pattern_primary ? phase3.ru_pattern_primary : "unknown");
    appendf(detail, sizeof(detail), " ru_pattern_complement=%s",
            phase3.ru_pattern_complement ? phase3.ru_pattern_complement : "unknown");
    appendf(detail, sizeof(detail), " ru_precoder_bypass=%u",
            phase3.ru_precoder_bypass_expected ? 1U : 0U);
    appendf(detail, sizeof(detail), " ru_prefilter_bypass=%u",
            phase3.ru_prefilter_bypass_expected ? 1U : 0U);
    appendf(detail, sizeof(detail), " ru_trn1u_structure=%u",
            phase3.ru_trn1u_structure_expected ? 1U : 0U);
    appendf(detail, sizeof(detail), " ru_tx_modem=%s",
            phase3.ru_transmitter_is_analogue ? "analogue" : "unknown");
    appendf(detail, sizeof(detail), " digital_quiet_pre_sd=%u",
            phase3.digital_quiet_before_sd ? 1U : 0U);
    appendf(detail, sizeof(detail), " first_digital_signal_sd_ja=%u",
            phase3.first_digital_signal_sd_during_ja ? 1U : 0U);
    appendf(detail, sizeof(detail), " ru_t=%d", phase3.ru_expected_t);
    appendf(detail, sizeof(detail), " ur_t=%d", phase3.ur_expected_t);
    appendf(detail, sizeof(detail), " ru_ur_cycle_t=%d", phase3.ru_ur_cycle_t);
    appendf(detail, sizeof(detail), " repeats_after_md_expected=%u",
            phase3.ru_ur_repeats_after_md_expected ? 1U : 0U);
    appendf(detail, sizeof(detail), " repeats_after_md_status=%s",
            phase3.ru_ur_repeats_after_md_status ? phase3.ru_ur_repeats_after_md_status : "unknown");
    appendf(detail, sizeof(detail), " ru_window_symbols=%d", obs.ru_window_len);
    appendf(detail, sizeof(detail), " ru_window_score=%s", obs.ru_window_score >= 0 ? "" : "n/a");
    if (obs.ru_window_score >= 0)
        appendf(detail, sizeof(detail), "%d", obs.ru_window_score);
    appendf(detail, sizeof(detail), " trn1u_mag_mean=%s", obs.trn1u_mag_count > 0 ? "" : "n/a");
    if (obs.trn1u_mag_count > 0)
        appendf(detail, sizeof(detail), "%.3f", obs.trn1u_mag_mean);
    appendf(detail, sizeof(detail), " trn1u_mag_count=%d", obs.trn1u_mag_count);
    appendf(detail, sizeof(detail), " ru_decoded=%u", phase3.ru_pattern_decoded ? 1U : 0U);
    appendf(detail, sizeof(detail), " ru_match=%u", phase3.ru_pattern_match ? 1U : 0U);
    appendf(detail, sizeof(detail), " ru_match_pct=%s", phase3.ru_pattern_decoded ? "" : "n/a");
    if (phase3.ru_pattern_decoded)
        appendf(detail, sizeof(detail), "%d", phase3.ru_match_percent);
    appendf(detail, sizeof(detail), " ru_symbols=%s", phase3.ru_pattern_decoded ? "" : "n/a");
    if (phase3.ru_pattern_decoded)
        appendf(detail, sizeof(detail), "%d/%d", phase3.ru_symbol_a, phase3.ru_symbol_b);
    appendf(detail, sizeof(detail), " lu_definition=%s",
            phase3.lu_definition ? phase3.lu_definition : "unknown");
    appendf(detail, sizeof(detail), " lu_absolute_level=%s",
            phase3.lu_absolute_level ? phase3.lu_absolute_level : "unknown");
    appendf(detail, sizeof(detail), " lu_reference=%s",
            phase3.lu_reference ? phase3.lu_reference : "unknown");
    appendf(detail, sizeof(detail), " ru_lu_consistency=%s",
            phase3.ru_lu_consistency_with_trn1u ? phase3.ru_lu_consistency_with_trn1u : "unknown");
    appendf(detail, sizeof(detail), " ru_lu_ratio=%s",
            phase3.ru_lu_ratio > 0.0f ? "" : "n/a");
    if (phase3.ru_lu_ratio > 0.0f)
        appendf(detail, sizeof(detail), "%.3f", phase3.ru_lu_ratio);
    call_log_append(log,
                    event_sample >= 0 ? event_sample : res->info0_sample,
                    0,
                    "V.92 Phase 3",
                    "Phase 3 sequence from Ru",
                    detail);
}

static const decode_v34_result_t *pick_analogue_phase2_side(const decode_v34_result_t *answerer,
                                                             const decode_v34_result_t *caller)
{
    const decode_v34_result_t *a = NULL;

    if (answerer && answerer->info0_seen && !answerer->info0_is_d)
        a = answerer;
    if (caller && caller->info0_seen && !caller->info0_is_d) {
        if (!a) {
            a = caller;
        } else {
            int score_a = 0;
            int score_c = 0;

            if (a->tx_ja_sample >= 0) score_a += 4;
            if (a->phase3_seen) score_a += 2;
            if (a->ja_aux_bit_len > 0) score_a += 1;
            if (caller->tx_ja_sample >= 0) score_c += 4;
            if (caller->phase3_seen) score_c += 2;
            if (caller->ja_aux_bit_len > 0) score_c += 1;
            if (score_c > score_a)
                a = caller;
        }
    }
    return a;
}

static const char *v34_info0_label(const decode_v34_result_t *result)
{
    return (result && result->info0_is_d) ? "INFO0d decoded" : "INFO0a decoded";
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
                           v91_law_t law)
{
    v8_probe_result_t probe;
    v8bis_scan_result_t v8bis;
    v8bis_weak_candidate_t weak_v8bis;
    ans_fallback_hit_t ans_fallback;
    bool calling_party = false;
    v8_fsk_burst_hit_t v21_burst;
    bool have_cre = false;

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
            if (strcmp(g_v8bis_signal_defs[i].name, "CRe") == 0)
                have_cre = true;
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
    print_v92_qc2_from_v8bis_messages(samples, total_samples);

    if (v8_select_best_channel_probe(samples, total_samples, total_samples, &probe)) {
        calling_party = probe.calling_party;
        printf("  Recording role:  %s (auto-detected)\n", calling_party ? "caller" : "answerer");
        printf("  Milestones:      ");
        if (probe.ansam_sample >= 0)
            printf("%s@%.1f ",
                   v8_answer_tone_name(probe.ansam_tone),
                   sample_to_ms(probe.ansam_sample, 8000));
        if (probe.ct_sample >= 0)
            printf("CT@%.1f ", sample_to_ms(probe.ct_sample, 8000));
        if (probe.cng_sample >= 0)
            printf("CNG@%.1f ", sample_to_ms(probe.cng_sample, 8000));
        if (probe.ci_sample >= 0)
            printf("CI@%.1f ", sample_to_ms(probe.ci_sample, 8000));
        if (probe.cm_jm_sample >= 0)
            printf("%s@%.1f%s ",
                   v8_cm_jm_name(calling_party),
                   sample_to_ms(probe.cm_jm_sample, 8000),
                   probe.cm_jm_complete ? "" : "(weak)");
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
            printf("  Observed V.8 signaling; %s\n",
                   probe.cm_jm_complete
                   ? "message timing is coherent, but full CM/JM fields are still incomplete"
                   : "message timing is present, but field decode is weak");
            if (!have_cre
                && !v8_result_looks_like_v34_relay(&probe.result)
                && probe.ansam_sample >= 0
                && probe.cm_jm_sample > probe.ansam_sample) {
                for (int ch = 0; ch <= 1; ch++) {
                    bool use_ch2 = (ch == 1);
                    v8_raw_msg_hit_t pre_cm_candidate;

                    memset(&pre_cm_candidate, 0, sizeof(pre_cm_candidate));
                    if (v8_scan_v92_window_candidate(samples,
                                                     total_samples,
                                                     8000,
                                                     probe.ansam_sample + 12000,
                                                     probe.cm_jm_sample,
                                                     use_ch2,
                                                     use_ch2 ? 1 : 0,
                                                     -1,
                                                     &pre_cm_candidate)
                        && pre_cm_candidate.preamble_type == V8_LOCAL_SYNC_V92) {
                        v92_short_phase1_candidate_t pre_cm_v92c;

                        if (v92_decode_short_phase1_candidate(pre_cm_candidate.bit_run,
                                                              pre_cm_candidate.bit_len,
                                                              use_ch2,
                                                              &pre_cm_v92c)) {
                            printf("  Pre-CM V.92:     %s (%s) at %.1f-%.1f ms on %s pair\n",
                                   pre_cm_v92c.name,
                                   use_ch2 ? "answering/CH2" : "calling/CH1",
                                   sample_to_ms(pre_cm_candidate.sample_offset, 8000),
                                   sample_to_ms(pre_cm_candidate.sample_offset + 70 * 8000 / 1000, 8000),
                                   use_ch2 ? "1650/1850 Hz" : "980/1180 Hz");
                        }
                    }
                }
            }
            if (probe.cm_jm_sample >= 0) {
                char hexbuf[256];
                char rawbitbuf[512];
                int approx_bits;

                format_hex_bytes(hexbuf, sizeof(hexbuf), probe.cm_jm_raw, probe.cm_jm_raw_len);
                rawbitbuf[0] = '\0';
                approx_bits = probe.cm_jm_raw_len * 10;
                if (approx_bits > 0) {
                    if (approx_bits > (int) sizeof(probe.cm_jm_raw) * 10)
                        approx_bits = (int) sizeof(probe.cm_jm_raw) * 10;
                    if (approx_bits > (int) sizeof(rawbitbuf) - 1)
                        approx_bits = (int) sizeof(rawbitbuf) - 1;
                    for (int i = 0; i < probe.cm_jm_raw_len && (i * 10 + 10) <= approx_bits; i++) {
                        size_t used = strlen(rawbitbuf);
                        snprintf(rawbitbuf + used,
                                 sizeof(rawbitbuf) - used,
                                 "%s0",
                                 (i == 0) ? "" : " ");
                        used = strlen(rawbitbuf);
                        for (int b = 0; b < 8 && used + 2 < sizeof(rawbitbuf); b++)
                            rawbitbuf[used++] = (probe.cm_jm_raw[i] & (1U << b)) ? '1' : '0';
                        rawbitbuf[used++] = '1';
                        rawbitbuf[used] = '\0';
                    }
                }
                printf("  CM/JM observed fields at ~%.1f ms:\n",
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
                print_v8_capability_note(&probe.result);
                if (probe.result.jm_cm.nsf >= 0)
                    printf("  NSF:            %s\n", v8_nsf_to_str(probe.result.jm_cm.nsf));
                if (probe.result.jm_cm.t66 >= 0)
                    printf("  T.66:           %s\n", v8_t66_to_str(probe.result.jm_cm.t66));
                if (hexbuf[0] != '\0')
                    printf("  Raw bytes:      %s\n", hexbuf);
                if (rawbitbuf[0] != '\0')
                    printf("  Raw bits:       %s\n", rawbitbuf);
                if (probe.cm_jm_salvaged)
                    printf("  Decode quality: %s\n",
                           probe.cm_jm_complete ? "stable repeated CM/JM observation" : "single-burst weak observation");
            }
        } else {
            printf("  Tone-level early negotiation observed, but no CI/CM/JM/CJ sequence was reconstructed\n");
            printf("  Possible V.8bis or clipped pre-V.8 exchange\n");
            if (probe.ansam_sample >= 0) {
                for (int ch = 0; ch <= 1; ch++) {
                    bool use_ch2 = (ch == 1);
                    v8_raw_msg_hit_t post_ans_candidate;

                    memset(&post_ans_candidate, 0, sizeof(post_ans_candidate));
                    if (v8_scan_v92_window_candidate(samples,
                                                     total_samples,
                                                     8000,
                                                     probe.ansam_sample + 12000,
                                                     total_samples,
                                                     use_ch2,
                                                     use_ch2 ? 1 : 0,
                                                     -1,
                                                     &post_ans_candidate)
                        && post_ans_candidate.preamble_type == V8_LOCAL_SYNC_V92) {
                        v92_short_phase1_candidate_t post_ans_v92c;

                        if (v92_decode_short_phase1_candidate(post_ans_candidate.bit_run,
                                                              post_ans_candidate.bit_len,
                                                              use_ch2,
                                                              &post_ans_v92c)) {
                            printf("  Post-ANSam V.92: %s (%s) at %.1f-%.1f ms on %s pair\n",
                                   post_ans_v92c.name,
                                   use_ch2 ? "answering/CH2" : "calling/CH1",
                                   sample_to_ms(post_ans_candidate.sample_offset, 8000),
                                   sample_to_ms(post_ans_candidate.sample_offset + 560, 8000),
                                   use_ch2 ? "1650/1850 Hz" : "980/1180 Hz");
                        }
                    }
                }
            }
            for (int ch = 0; ch <= 1; ch++) {
                bool use_ch2 = (ch == 1);
                const char *ch_label = use_ch2 ? "answering/CH2" : "calling/CH1";
                const char *hz_label = use_ch2 ? "1650/1850 Hz" : "980/1180 Hz";

                if ((probe.ansam_sample >= 0 || probe.ct_sample >= 0 || probe.cng_sample >= 0)
                    && probe.cm_jm_sample < 0) {
                    v8_raw_msg_hit_t raw_candidate;
                    v8_fsk_burst_hit_t bursts[V8_MAX_TARGETED_BURSTS];
                    v8_targeted_diag_t diag;
                    int burst_count;
                    char hexbuf[256];

                    memset(&raw_candidate, 0, sizeof(raw_candidate));
                    memset(&diag, 0, sizeof(diag));
                    hexbuf[0] = '\0';
                    burst_count = v8_collect_v21_bursts(samples,
                                                        total_samples,
                                                        8000,
                                                        v8_probe_targeted_search_start(&probe, total_samples),
                                                        total_samples,
                                                        use_ch2,
                                                        bursts,
                                                        V8_MAX_TARGETED_BURSTS);
                    if (burst_count <= 0)
                        continue;
                    v21_burst = bursts[0];
                    if (v8_targeted_v21_bytes_candidate_repeated(samples,
                                                                 total_samples,
                                                                 8000,
                                                                 bursts,
                                                                 burst_count,
                                                                 use_ch2,
                                                                 !use_ch2,
                                                                 &raw_candidate,
                                                                 &diag)) {
                        format_hex_bytes(hexbuf, sizeof(hexbuf), raw_candidate.bytes, raw_candidate.byte_len);
                    }
                    print_v8_targeted_diag(&diag, bursts, use_ch2);
                    if (raw_candidate.preamble_type == V8_LOCAL_SYNC_V92 && have_cre) {
                        printf("  V.92 short-Phase-1 interpretation suppressed because CRe indicates a V.8bis/V.34 relay path\n");
                    } else if (raw_candidate.preamble_type == V8_LOCAL_SYNC_V92) {
                        v92_short_phase1_candidate_t v92c;
                        v92_qts_hit_t qts_hit;

                        if (v92_decode_short_phase1_candidate(raw_candidate.bit_run, raw_candidate.bit_len, use_ch2, &v92c)) {
                            char ones1[16], sync1[16], frame1[16], ones2[16], sync2[16], frame2[16], tail[16];

                            printf("  %s (%s) candidate at %.1f-%.1f ms on %s pair (peak %.1f%%)\n",
                                   v92c.name, ch_label,
                                   sample_to_ms(v21_burst.start_sample, 8000),
                                   sample_to_ms(v21_burst.start_sample + v21_burst.duration_samples, 8000),
                                   hz_label,
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
                                   v8_preamble_to_candidate_label(raw_candidate.preamble_type, !use_ch2),
                                   sample_to_ms(v21_burst.start_sample, 8000),
                                   sample_to_ms(v21_burst.start_sample + v21_burst.duration_samples, 8000),
                                   hz_label,
                                   v21_burst.peak_strength * 100.0);
                            printf("  Lock detail:     %s\n",
                                   v8_preamble_detail(raw_candidate.preamble_type));
                        }
                    } else {
                        printf("  %s V.21 burst candidate at %.1f-%.1f ms on %s pair (peak %.1f%%)\n",
                               v8_preamble_to_candidate_label(raw_candidate.preamble_type, !use_ch2),
                               sample_to_ms(v21_burst.start_sample, 8000),
                               sample_to_ms(v21_burst.start_sample + v21_burst.duration_samples, 8000),
                               hz_label,
                               v21_burst.peak_strength * 100.0);
                        printf("  Lock detail:     %s\n",
                               v8_preamble_detail(raw_candidate.preamble_type));
                        if (raw_candidate.preamble_type == V8_LOCAL_SYNC_CM_JM && diag.repeat_count > 0) {
                            printf("  Repeat status:   %d burst%s%s\n",
                                   diag.repeat_count,
                                   diag.repeat_count == 1 ? "" : "s",
                                   diag.repeated_confirmed ? ", stable repeat" : "");
                        }
                    }
                    if (hexbuf[0] != '\0')
                        printf("  Candidate bytes: %s\n", hexbuf);
                }
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
        char s_tone[128];

        sanitize_detail_value(s_tone, sizeof(s_tone), v8_answer_tone_detail(probe.ansam_tone));
        snprintf(summary, sizeof(summary),
                 "%s detected",
                 v8_answer_tone_name(probe.ansam_tone));
        snprintf(detail, sizeof(detail),
                 "role=%s tone=%s",
                 probe.calling_party ? "caller" : "answerer",
                 s_tone);
        call_log_append(log, probe.ansam_sample, 0, "V.8", summary, detail);
    }
    if (probe.ct_sample >= 0) {
        snprintf(detail, sizeof(detail),
                 "role=%s tone=%s",
                 probe.calling_party ? "caller" : "answerer",
                 v8_aux_tone_name(MODEM_CONNECT_TONES_CALLING_TONE));
        call_log_append(log, probe.ct_sample, 0, "V.8", "CT detected", detail);
    }
    if (probe.cng_sample >= 0) {
        snprintf(detail, sizeof(detail),
                 "role=%s tone=%s",
                 probe.calling_party ? "caller" : "answerer",
                 v8_aux_tone_name(MODEM_CONNECT_TONES_FAX_CNG));
        call_log_append(log, probe.cng_sample, 0, "V.8", "CNG detected", detail);
    }
    if (probe.ci_sample >= 0) {
        char s_ci_fn[64];

        sanitize_detail_value(s_ci_fn, sizeof(s_ci_fn), v8_call_function_to_str(probe.result.jm_cm.call_function));
        snprintf(detail, sizeof(detail),
                 "role=%s call_function=%s",
                 probe.calling_party ? "caller" : "answerer",
                 s_ci_fn);
        call_log_append(log, probe.ci_sample, 0, "V.8", "CI decoded", detail);
    }
    if (probe.cm_jm_sample >= 0) {
        char hexbuf[256];
        char modbuf[256];
        char s_call_fn[64], s_proto[64], s_pcm[64], s_pstn[64], s_nsf[64], s_t66[64];
        int pos;

        format_hex_bytes(hexbuf, sizeof(hexbuf), probe.cm_jm_raw, probe.cm_jm_raw_len);
        format_v8_modulations_str(probe.result.jm_cm.modulations, modbuf, sizeof(modbuf));
        sanitize_detail_value(s_call_fn, sizeof(s_call_fn), v8_call_function_to_str(probe.result.jm_cm.call_function));
        sanitize_detail_value(s_proto, sizeof(s_proto), v8_protocol_to_str(probe.result.jm_cm.protocols));
        sanitize_detail_value(s_pcm, sizeof(s_pcm), v8_pcm_modem_availability_to_str(probe.result.jm_cm.pcm_modem_availability));
        sanitize_detail_value(s_pstn, sizeof(s_pstn), v8_pstn_access_to_str(probe.result.jm_cm.pstn_access));
        snprintf(summary, sizeof(summary),
                 "%s %s",
                 v8_cm_jm_name(probe.calling_party),
                 probe.cm_jm_complete ? "decoded" : "candidate");
        pos = snprintf(detail, sizeof(detail),
                 "role=%s msg=%s confidence=%s call_fn=%s modulations=%s protocol=%s pcm=%s pstn=%s",
                 probe.calling_party ? "caller" : "answerer",
                 v8_cm_jm_name(probe.calling_party),
                 probe.cm_jm_complete ? "confirmed" : "candidate_fragment",
                 s_call_fn,
                 modbuf[0] ? modbuf : "none",
                 s_proto,
                 s_pcm,
                 s_pstn);
        if (probe.result.jm_cm.nsf >= 0 && pos < (int) sizeof(detail)) {
            sanitize_detail_value(s_nsf, sizeof(s_nsf), v8_nsf_to_str(probe.result.jm_cm.nsf));
            pos += snprintf(detail + pos, sizeof(detail) - (size_t) pos,
                            " nsf=%s", s_nsf);
        }
        if (probe.result.jm_cm.t66 >= 0 && pos < (int) sizeof(detail)) {
            sanitize_detail_value(s_t66, sizeof(s_t66), v8_t66_to_str(probe.result.jm_cm.t66));
            pos += snprintf(detail + pos, sizeof(detail) - (size_t) pos,
                            " t66=%s", s_t66);
        }
        if (probe.result.v92 >= 0 && pos < (int) sizeof(detail))
            pos += snprintf(detail + pos, sizeof(detail) - (size_t) pos,
                            " v92=0x%02x", probe.result.v92);
        if (hexbuf[0] != '\0' && pos < (int) sizeof(detail))
            snprintf(detail + pos, sizeof(detail) - (size_t) pos,
                     " raw=%s", hexbuf);
        call_log_append(log, probe.cm_jm_sample, 0, "V.8", summary, detail);
    }
    if (probe.ansam_sample >= 0
        && probe.cm_jm_sample > probe.ansam_sample) {
        for (int ch = 0; ch <= 1; ch++) {
            bool use_ch2 = (ch == 1);
            v8_raw_msg_hit_t pre_cm_candidate;

            memset(&pre_cm_candidate, 0, sizeof(pre_cm_candidate));
            if (v8_scan_v92_window_candidate(samples,
                                             total_samples,
                                             8000,
                                             probe.ansam_sample + 12000,
                                             probe.cm_jm_sample,
                                             use_ch2,
                                             use_ch2 ? 1 : 0,
                                             -1,
                                             &pre_cm_candidate)
                && pre_cm_candidate.preamble_type == V8_LOCAL_SYNC_V92) {
                v92_short_phase1_candidate_t pre_cm_v92c;

                if (v92_decode_short_phase1_candidate(pre_cm_candidate.bit_run,
                                                      pre_cm_candidate.bit_len,
                                                      use_ch2,
                                                      &pre_cm_v92c)) {
                    snprintf(summary, sizeof(summary), "Pre-CM %s (%s)",
                             pre_cm_v92c.name,
                             use_ch2 ? "answering/CH2" : "calling/CH1");
                    snprintf(detail, sizeof(detail),
                             "role=%s start=%.1f ms duration=%.1f ms pair=%s pre_cm=yes lock=\"%s\"",
                             use_ch2 ? "answering" : "calling",
                             sample_to_ms(pre_cm_candidate.sample_offset, 8000),
                             70.0,
                             use_ch2 ? "1650/1850 Hz" : "980/1180 Hz",
                             v8_preamble_detail(pre_cm_candidate.preamble_type));
                    append_v92_short_phase1_fields(detail, sizeof(detail), &pre_cm_v92c);
                    call_log_append(log, pre_cm_candidate.sample_offset, 560, "V.92", summary, detail);
                }
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
        snprintf(detail, sizeof(detail),
                 "role=%s early negotiation energy without CI/CM/JM/CJ",
                 probe.calling_party ? "caller" : "answerer");
        call_log_append(log,
                        probe.ansam_sample,
                        0,
                        "V.8bis?",
                        "Possible V.8bis / pre-V.8 negotiation",
                        detail);
        for (int ch = 0; ch <= 1; ch++) {
            bool use_ch2 = (ch == 1);
            v8_raw_msg_hit_t post_ans_ch_candidate;

            memset(&post_ans_ch_candidate, 0, sizeof(post_ans_ch_candidate));
            if (v8_scan_v92_window_candidate(samples,
                                             total_samples,
                                             8000,
                                             probe.ansam_sample + 12000,
                                             max_sample > 0 ? max_sample : total_samples,
                                             use_ch2,
                                             use_ch2 ? 1 : 0,
                                             -1,
                                             &post_ans_ch_candidate)
                && post_ans_ch_candidate.preamble_type == V8_LOCAL_SYNC_V92) {
                v92_short_phase1_candidate_t post_ans_v92c;

                if (v92_decode_short_phase1_candidate(post_ans_ch_candidate.bit_run,
                                                      post_ans_ch_candidate.bit_len,
                                                      use_ch2,
                                                      &post_ans_v92c)) {
                    snprintf(summary, sizeof(summary), "Post-ANSam %s (%s)",
                             post_ans_v92c.name,
                             use_ch2 ? "answering/CH2" : "calling/CH1");
                    snprintf(detail, sizeof(detail),
                             "role=%s start=%.1f ms duration=%.1f ms pair=%s post_ansam=yes lock=\"%s\"",
                             use_ch2 ? "answering" : "calling",
                             sample_to_ms(post_ans_ch_candidate.sample_offset, 8000),
                             70.0,
                             use_ch2 ? "1650/1850 Hz" : "980/1180 Hz",
                             v8_preamble_detail(post_ans_ch_candidate.preamble_type));
                    append_v92_short_phase1_fields(detail, sizeof(detail), &post_ans_v92c);
                    call_log_append(log, post_ans_ch_candidate.sample_offset, 560, "V.92", summary, detail);
                }
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
    for (int ch = 0; ch <= 1; ch++) {
        bool use_ch2 = (ch == 1);
        const char *ch_role = use_ch2 ? "answering" : "calling";
        const char *hz_label = use_ch2 ? "1650/1850 Hz" : "980/1180 Hz";

        if (probe.cm_jm_sample < 0
            && (probe.ansam_sample >= 0 || probe.ct_sample >= 0 || probe.cng_sample >= 0)) {
            v8_raw_msg_hit_t raw_candidate;
            v8_fsk_burst_hit_t bursts[V8_MAX_TARGETED_BURSTS];
            int burst_count;
            char hexbuf[256];

            memset(&raw_candidate, 0, sizeof(raw_candidate));
            hexbuf[0] = '\0';
            burst_count = v8_collect_v21_bursts(samples,
                                                total_samples,
                                                8000,
                                                v8_probe_targeted_search_start(&probe, max_sample > 0 ? max_sample : total_samples),
                                                max_sample > 0 ? max_sample : total_samples,
                                                use_ch2,
                                                bursts,
                                                V8_MAX_TARGETED_BURSTS);
            if (burst_count <= 0)
                continue;
            v21_burst = bursts[0];
            if (v8_targeted_v21_bytes_candidate_repeated(samples,
                                                         total_samples,
                                                         8000,
                                                         bursts,
                                                         burst_count,
                                                         use_ch2,
                                                         !use_ch2,
                                                         &raw_candidate,
                                                         NULL)) {
                format_hex_bytes(hexbuf, sizeof(hexbuf), raw_candidate.bytes, raw_candidate.byte_len);
            }
            if (raw_candidate.preamble_type == V8_LOCAL_SYNC_V92) {
                v92_short_phase1_candidate_t v92c;
                v92_qts_hit_t qts_hit;

                if (v92_decode_short_phase1_candidate(raw_candidate.bit_run, raw_candidate.bit_len, use_ch2, &v92c)) {
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
                             "%s (%s) candidate",
                             v92c.name, ch_role);
                    snprintf(detail, sizeof(detail),
                             "role=%s start=%.1f ms duration=%.1f ms pair=%s peak=%.1f%% lock=\"%s\" lapm=%s %s%s repeat=%s%s%s%s%s",
                             ch_role,
                             sample_to_ms(v21_burst.start_sample, 8000),
                             sample_to_ms(v21_burst.duration_samples, 8000),
                             hz_label,
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
                             "%s V.21 burst candidate (%s)",
                             v8_preamble_to_candidate_label(raw_candidate.preamble_type, !use_ch2),
                             ch_role);
                    snprintf(detail, sizeof(detail),
                             "role=%s start=%.1f ms duration=%.1f ms pair=%s peak=%.1f%% lock=\"%s\"%s%s",
                             ch_role,
                             sample_to_ms(v21_burst.start_sample, 8000),
                             sample_to_ms(v21_burst.duration_samples, 8000),
                             hz_label,
                             v21_burst.peak_strength * 100.0,
                             v8_preamble_detail(raw_candidate.preamble_type),
                             hexbuf[0] != '\0' ? " raw=" : "",
                             hexbuf[0] != '\0' ? hexbuf : "");
                }
            } else {
                snprintf(summary, sizeof(summary),
                         "%s V.21 burst candidate (%s)",
                         v8_preamble_to_candidate_label(raw_candidate.preamble_type, !use_ch2),
                         ch_role);
                snprintf(detail, sizeof(detail),
                         "role=%s start=%.1f ms duration=%.1f ms pair=%s peak=%.1f%% lock=\"%s\"%s%s",
                         ch_role,
                         sample_to_ms(v21_burst.start_sample, 8000),
                         sample_to_ms(v21_burst.duration_samples, 8000),
                         hz_label,
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


static bool decode_v34_pass_mode(const int16_t *samples,
                                 int total_samples,
                                 v91_law_t law,
                                 bool calling_party,
                                 float initial_signal_cutoff_db,
                                 bool allow_info_rate_infer,
                                 const v34_offline_decode_config_t *config,
                                 bool phase2_only,
                                 decode_v34_result_t *result)
{
    v34_state_t *v34;
    v34_v90_info0a_t raw_info0a;
    v34_v90_info1a_t raw_info1a;
    v34_v90_info1d_t raw_info1d;
    stderr_silence_guard_t stderr_guard;
    v34_aux_bit_collector_t aux_bits;
    int offset = 0;
    int prev_rx_stage = -1;
    int prev_tx_stage = -1;
    int prev_rx_event = -1;
    int ja_aux_start_bit = -1;
    mp_t rx_mp;
    mp_t recovered_mp;
    mp_t heuristic_mp;
    v34_info_collector_t info0_collector;
    v34_info_collector_t info1_collector;
    bool have_rx_mp = false;
    bool have_recovered_mp = false;
    bool have_heuristic_mp = false;
    int recovered_mp_bits = 0;
    int recovered_mp_start_errs = INT_MAX;
    bool recovered_mp_crc_ok = false;
    bool recovered_mp_fill_ok = false;
    int heuristic_mp_bits = 0;
    int heuristic_mp_start_errs = INT_MAX;
    bool heuristic_mp_crc_ok = false;
    bool heuristic_mp_fill_ok = false;
    int heuristic_mp_votes = 0;
    int mp_tuple_votes[2][15][15][3];
    bool flow_debug = false;
    bool info_cutoff_relaxed = false;
    float post_info_signal_cutoff_db;
    int last_progress_sample = 0;

    if (!samples || total_samples <= 0 || !result)
        return false;

    memset(result, 0, sizeof(*result));
    memset(mp_tuple_votes, 0, sizeof(mp_tuple_votes));
    v34_info_collector_init(&info0_collector, 62 - (4 + 8 + 4));
    v34_info_collector_init(&info1_collector, 70 - (4 + 8 + 4));
    result->info0_sample = -1;
    result->info1_sample = -1;
    result->info0_ok_event_sample = -1;
    result->info0_bad_event_sample = -1;
    result->info1_ok_event_sample = -1;
    result->info1_bad_event_sample = -1;
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
    result->ja_aux_bit_len = 0;
    result->ja_aux_bits[0] = '\0';
    result->rx_s_event_sample = -1;
    result->rx_tone_a_sample = -1;
    result->rx_tone_b_sample = -1;
    result->rx_tone_a_reversal_sample = -1;
    result->rx_tone_b_reversal_sample = -1;
    result->tx_tone_a_sample = -1;
    result->tx_tone_b_sample = -1;
    result->tx_tone_a_reversal_sample = -1;
    result->tx_tone_b_reversal_sample = -1;
    result->tx_info1_sample = -1;
    result->rx_phase4_s_sample = -1;
    result->rx_phase4_sbar_sample = -1;
    result->rx_phase4_trn_sample = -1;
    result->ru_window_len = 0;
    memset(result->ru_window_symbols, 0, sizeof(result->ru_window_symbols));
    memset(result->ru_window_mags, 0, sizeof(result->ru_window_mags));
    result->ru_window_captured = false;
    result->ru_window_score = -1;
    result->trn1u_mag_mean = 0.0f;
    result->trn1u_mag_count = 0;
    result->phase4_ready_sample = -1;
    result->phase4_sample = -1;
    result->failure_sample = -1;
    result->mp_from_rx_frame = false;
    result->mp_from_rx_recovery = false;
    result->mp_from_info_sequence = false;
    result->mp_crc_valid = false;
    result->mp_fill_valid = false;
    result->mp_candidate_seen = false;
    result->mp_candidate_crc_valid = false;
    result->mp_candidate_fill_valid = false;
    result->mp_aux_channel_supported = false;
    result->mp_use_non_linear_encoder = false;
    result->mp_expanded_shaping = false;
    result->mp_asymmetric_rates_allowed = false;
    result->mp_local_ack_bit = false;
    result->mp_type = -1;
    result->mp_trellis_size = -1;
    result->mp_signalling_rate_mask = 0;
    result->mp_frame_bits_captured = 0;
    result->mp_start_error_count = -1;
    result->mp_candidate_type = -1;
    result->mp_candidate_trellis_size = -1;
    result->mp_candidate_signalling_rate_mask = 0;
    result->mp_candidate_frame_bits_captured = 0;
    result->mp_candidate_start_error_count = -1;
    result->mp_candidate_votes = 0;
    result->mp_candidate_rate_a_to_c_bps = -1;
    result->mp_candidate_rate_c_to_a_bps = -1;
    result->mp_rate_a_to_c_bps = -1;
    result->mp_rate_c_to_a_bps = -1;

    if (!config)
        config = &g_v34_offline_decode_config;

    v34 = v34_init(NULL, 3200, 21600, calling_party, config->enable_tx,
                   v34_dummy_get_bit, NULL,
                   v34_dummy_put_bit, NULL);
    if (!v34)
        return false;

    v34_set_v90_mode(v34, law == V91_LAW_ALAW ? 1 : 0);
    /* Keep a stricter threshold for initial INFO0 lock, then relax after
       INFO0. For already-permissive rescue passes, preserve that permissive
       threshold so INFO1 can still be acquired. */
    post_info_signal_cutoff_db = initial_signal_cutoff_db;
    if (post_info_signal_cutoff_db > -60.0f)
        post_info_signal_cutoff_db = -60.0f;
    v34_rx_set_signal_cutoff(v34, initial_signal_cutoff_db);
    memset(&aux_bits, 0, sizeof(aux_bits));
    aux_bits.capture_start_bit = -1;
    v34_set_put_aux_bit(v34, v34_collect_aux_bit, &aux_bits);
    if (getenv("VPCM_V34_FLOW"))
        flow_debug = true;
    span_log_set_level(v34_get_logging_state(v34),
                       SPAN_LOG_SHOW_SEVERITY | (flow_debug ? SPAN_LOG_FLOW : SPAN_LOG_WARNING));
    if (!flow_debug)
        stderr_guard = silence_stderr_begin();

    offset = 0;
    last_progress_sample = 0;
    while (offset < total_samples) {
        int chunk = total_samples - offset;
        int16_t tx_buf[160];
        int rx_stage;
        int tx_stage;
        int rx_event;
        int phase2_chunk = (!result->phase3_seen && (!result->info0_seen || !result->info1_seen)) ? 80 : 160;

        if (chunk > phase2_chunk)
            chunk = phase2_chunk;

        v34_rx(v34, samples + offset, chunk);
        if (config->enable_tx)
            v34_tx(v34, tx_buf, chunk);
        offset += chunk;

        rx_stage = v34_get_rx_stage(v34);
        tx_stage = v34_get_tx_stage(v34);
        rx_event = v34_get_rx_event(v34);

        if (phase2_only) {
            if (rx_stage == V34_RX_STAGE_INFO0
                || rx_event == V34_EVENT_INFO0_OK
                || rx_event == V34_EVENT_INFO0_BAD) {
                sync_v34_info_collector_from_rx(&info0_collector, v34);
            }
            if (rx_stage == V34_RX_STAGE_INFO1A
                || rx_stage == V34_RX_STAGE_INFO1C
                || rx_event == V34_EVENT_INFO1_OK
                || rx_event == V34_EVENT_INFO1_BAD) {
                sync_v34_info_collector_from_rx(&info1_collector, v34);
            }
        }

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
            last_progress_sample = offset;
            switch (tx_stage) {
            case 3:
                note_first_sample(&result->tx_tone_a_sample, offset);
                break;
            case 33:
                note_first_sample(&result->tx_first_s_sample, offset);
                break;
            case 4:
            case 7:
            case 10:
                note_first_sample(&result->tx_tone_a_sample, offset);
                break;
            case 5:
            case 11:
                note_first_sample(&result->tx_tone_a_reversal_sample, offset);
                break;
            case 20:
            case 21:
            case 23:
            case 24:
            case 29:
                note_first_sample(&result->tx_tone_b_sample, offset);
                break;
            case 19:
            case 26:
                note_first_sample(&result->tx_tone_b_reversal_sample, offset);
                break;
            case 22:
                note_first_sample(&result->tx_info1_sample, offset);
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
                if (aux_bits.capture_start_bit < 0)
                    aux_bits.capture_start_bit = aux_bits.total_bits;
                break;
            case 39:
                note_first_sample(&result->tx_ja_sample, offset);
                if (ja_aux_start_bit < 0) {
                    ja_aux_start_bit = aux_bits.total_bits;
                    if (aux_bits.capture_start_bit < 0)
                        aux_bits.capture_start_bit = ja_aux_start_bit;
                }
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
            last_progress_sample = offset;
            switch (rx_stage) {
            case 5:
                note_first_sample(&result->rx_tone_a_sample, offset);
                break;
            case 6:
                note_first_sample(&result->rx_tone_b_sample, offset);
                break;
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
            last_progress_sample = offset;
            if (rx_event == V34_EVENT_INFO0_OK) {
                result->info0_ok_event_seen = true;
                note_first_sample(&result->info0_ok_event_sample, offset);
            } else if (rx_event == V34_EVENT_INFO0_BAD) {
                result->info0_bad_event_seen = true;
                note_first_sample(&result->info0_bad_event_sample, offset);
            } else if (rx_event == V34_EVENT_INFO1_OK) {
                result->info1_ok_event_seen = true;
                note_first_sample(&result->info1_ok_event_sample, offset);
            } else if (rx_event == V34_EVENT_INFO1_BAD) {
                result->info1_bad_event_seen = true;
                note_first_sample(&result->info1_bad_event_sample, offset);
            }
            if (rx_event == 1) {
                if (rx_stage == 5)
                    note_first_sample(&result->rx_tone_a_sample, offset);
                else if (rx_stage == 6)
                    note_first_sample(&result->rx_tone_b_sample, offset);
            } else if (rx_event == 2 || rx_event == 3 || rx_event == 4) {
                if (rx_stage == 5)
                    note_first_sample(&result->rx_tone_a_reversal_sample, offset);
                else if (rx_stage == 6)
                    note_first_sample(&result->rx_tone_b_reversal_sample, offset);
            }
            if (rx_event == 12)
                note_first_sample(&result->rx_s_event_sample, offset);
            prev_rx_event = rx_event;
        }

        if (!result->info0_seen) {
            int have_info0 = 0;

            if (phase2_only && rx_event == V34_EVENT_INFO0_OK
                && parse_v34_info0_from_rx_buf(&raw_info0a, &result->info0a, &info0_collector)) {
                have_info0 = 1;
            } else {
                have_info0 = v34_get_v90_received_info0a(v34, &raw_info0a);
            }

            if (have_info0 <= 0 && rx_event == V34_EVENT_INFO0_OK
                && snapshot_v34_info0a_from_rx_state(&raw_info0a, v34)) {
                have_info0 = 1;
            }
            if (have_info0 > 0
                && ((phase2_only && rx_event == V34_EVENT_INFO0_OK)
                    || v34_map_received_info0a(&result->info0a, &raw_info0a))) {
                result->info0_seen = true;
                result->info0_is_d = calling_party;
                result->info0_raw = raw_info0a;
                result->info0_sample = offset;
                last_progress_sample = offset;
                if (!info_cutoff_relaxed) {
                    v34_rx_set_signal_cutoff(v34, post_info_signal_cutoff_db);
                    info_cutoff_relaxed = true;
                }
            }
        }
        if (!result->info1_seen) {
            int have_info1a = 0;

            if (phase2_only && rx_event == V34_EVENT_INFO1_OK
                && !calling_party
                && parse_v34_info1a_from_rx_buf(&raw_info1a, &result->info1a, &info1_collector)) {
                have_info1a = 1;
            } else {
                have_info1a = v34_get_v90_received_info1a(v34, &raw_info1a);
            }

            if (have_info1a <= 0 && rx_event == V34_EVENT_INFO1_OK
                && snapshot_v34_info1a_from_rx_state(&raw_info1a, v34)) {
                have_info1a = 1;
            }
            if (have_info1a > 0
                && ((phase2_only && rx_event == V34_EVENT_INFO1_OK && !calling_party)
                    || v34_map_received_info1a(&result->info1a, &raw_info1a))) {
                result->info1_seen = true;
                result->info1_is_d = false;
                result->info1a_raw = raw_info1a;
                result->info1_sample = offset;
                result->u_info = result->info1a.u_info;
                result->u_info_from_info1a = true;
                last_progress_sample = offset;
            } else if ((phase2_only && rx_event == V34_EVENT_INFO1_OK && calling_party
                        && parse_v34_info1d_from_rx_buf(&raw_info1d, &info1_collector))
                       || v34_get_v90_received_info1d(v34, &raw_info1d) > 0) {
                result->info1_seen = true;
                result->info1_is_d = true;
                result->info1d = raw_info1d;
                result->info1_sample = offset;
                last_progress_sample = offset;
            }
        }
        if (!result->ru_window_captured
            && (rx_stage == 10 || rx_stage == 11)) {
            int d = v34->rx.duration;
            int len = d;

            if (len > 32)
                len = 32;
            if (len >= 12) {
                int start = d - len + 1;

                for (int i = 0; i < len; i++) {
                    int baud = start + i;
                    int idx = (baud - 1) & 31;
                    result->ru_window_symbols[i] = (uint8_t) (v34->rx.phase3_s_ring[idx] & 0x3);
                    result->ru_window_mags[i] = v34->rx.phase3_s_mag_ring[idx];
                }
                result->ru_window_len = len;
                result->ru_window_captured = true;
                result->ru_window_score = ru_window_two_point_score(result->ru_window_symbols, len);
            }
        } else if ((rx_stage == 10 || rx_stage == 11)) {
            int d = v34->rx.duration;
            int len = d;
            uint8_t candidate[32];
            int score;

            if (len > 32)
                len = 32;
            if (len >= 12) {
                int start = d - len + 1;
                for (int i = 0; i < len; i++) {
                    int baud = start + i;
                    int idx = (baud - 1) & 31;
                    candidate[i] = (uint8_t) (v34->rx.phase3_s_ring[idx] & 0x3);
                }
                score = ru_window_two_point_score(candidate, len);
                if (score > result->ru_window_score) {
                    memcpy(result->ru_window_symbols, candidate, (size_t) len);
                    for (int i = 0; i < len; i++) {
                        int baud = start + i;
                        int idx = (baud - 1) & 31;
                        result->ru_window_mags[i] = v34->rx.phase3_s_mag_ring[idx];
                    }
                    result->ru_window_len = len;
                    result->ru_window_score = score;
                    result->ru_window_captured = true;
                }
            }
        }
        if (!result->phase3_seen
            && (rx_stage >= 11 || v34_get_primary_channel_active(v34))) {
            result->phase3_seen = true;
            result->phase3_sample = offset;
            last_progress_sample = offset;
        }
        {
            int target = v34->rx.mp_frame_target;
            int type = v34->rx.mp_frame_bits[18] & 1U;
            int expected_target = type ? 188 : 88;

            if (target <= 0)
                target = expected_target;
            if (target >= expected_target) {
                    bool crc_ok = v34_mp_crc_ok(v34->rx.mp_frame_bits, type);
                    bool fill_ok = v34_mp_fill_ok(v34->rx.mp_frame_bits, type);
                    int start_errs = v34_mp_start_error_count(v34->rx.mp_frame_bits, type, target);
                    mp_t candidate_mp;
                    int candidate_bits = 0;

                    if (v34_parse_mp_from_raw_bits(v34->rx.mp_frame_bits,
                                                   target,
                                                   &candidate_mp,
                                                   &candidate_bits)
                        && fill_ok
                        && start_errs <= 2
                        && candidate_mp.bit_rate_a_to_c >= 1
                        && candidate_mp.bit_rate_a_to_c <= 14
                        && candidate_mp.bit_rate_c_to_a >= 1
                        && candidate_mp.bit_rate_c_to_a <= 14
                        && candidate_mp.trellis_size >= 0
                        && candidate_mp.trellis_size <= 2) {
                        int votes = ++mp_tuple_votes[type]
                                                   [candidate_mp.bit_rate_a_to_c]
                                                   [candidate_mp.bit_rate_c_to_a]
                                                   [candidate_mp.trellis_size];
                        if (!have_heuristic_mp
                            || votes > heuristic_mp_votes
                            || (votes == heuristic_mp_votes && start_errs < heuristic_mp_start_errs)) {
                            heuristic_mp = candidate_mp;
                            heuristic_mp_bits = candidate_bits;
                            heuristic_mp_start_errs = start_errs;
                            heuristic_mp_crc_ok = crc_ok;
                            heuristic_mp_fill_ok = fill_ok;
                            heuristic_mp_votes = votes;
                            have_heuristic_mp = true;
                        }
                    }

                    if (crc_ok
                        && fill_ok
                        && start_errs <= 2
                        && v34_parse_mp_from_raw_bits(v34->rx.mp_frame_bits,
                                                      target,
                                                      &candidate_mp,
                                                      &candidate_bits)
                        && v34_mp_semantic_ok_strict(&candidate_mp)) {
                        if (!have_recovered_mp || start_errs < recovered_mp_start_errs) {
                            recovered_mp = candidate_mp;
                            recovered_mp_bits = candidate_bits;
                            recovered_mp_start_errs = start_errs;
                            recovered_mp_crc_ok = crc_ok;
                            recovered_mp_fill_ok = fill_ok;
                            have_recovered_mp = true;
                        }
                    }
                }
        }
        if (!result->phase4_ready_seen && rx_event == 15) {
            result->phase4_ready_seen = true;
            result->phase4_ready_sample = offset;
            last_progress_sample = offset;
        }
        if (!result->phase4_seen && (rx_stage >= 16 || tx_stage >= 41 || rx_event == 15)) {
            result->phase4_seen = true;
            result->phase4_sample = offset;
            last_progress_sample = offset;
        }
        if (!result->training_failed && rx_event == 16) {
            result->training_failed = true;
            result->failure_sample = offset;
            last_progress_sample = offset;
        }
        if (phase2_only
            && result->info0_seen
            && result->info1_seen
            && offset >= last_progress_sample + 160) {
            break;
        }
        if (!result->phase3_seen && !result->phase4_seen && !result->training_failed) {
            int phase2_dead_end_anchor = -1;
            int phase2_dead_end_holdoff = phase2_only ? 4000 : 8000;
            int phase2_dead_end_idle = phase2_only ? 2000 : 4000;

            if (result->info1_bad_event_seen && !result->info1_seen) {
                phase2_dead_end_anchor = result->info1_bad_event_sample;
            } else if (result->info0_bad_event_seen && !result->info0_seen) {
                phase2_dead_end_anchor = result->info0_bad_event_sample;
            }
            if (phase2_dead_end_anchor >= 0
                && offset >= phase2_dead_end_anchor + phase2_dead_end_holdoff
                && offset >= last_progress_sample + phase2_dead_end_idle) {
                break;
            }
        }
        if ((result->phase4_seen || result->training_failed)
            && offset >= max_non_negative(result->phase4_sample, result->failure_sample) + 8000) {
            break;
        }
        if ((result->info1_seen || result->phase3_seen)
            && offset >= last_progress_sample + 16000) {
            break;
        }
    }

    if (!flow_debug)
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
    if (phase2_only) {
        normalize_v34_result_to_spec_flow(result, calling_party);
        v34_free(v34);
        return true;
    }
    result->mp_seen = (v34->rx.mp_seen >= 1);
    result->mp_remote_ack_seen = (v34->rx.mp_remote_ack_seen > 0);
    have_rx_mp = v34_parse_mp_from_frame_bits(v34->rx.mp_frame_bits,
                                              v34->rx.mp_seen,
                                              v34->rx.mp_frame_target,
                                              &rx_mp,
                                              &result->mp_frame_bits_captured);
    if (have_rx_mp && !v34_mp_semantic_ok_strict(&rx_mp))
        have_rx_mp = false;
    if (!have_rx_mp && have_recovered_mp) {
        rx_mp = recovered_mp;
        result->mp_frame_bits_captured = recovered_mp_bits;
        result->mp_start_error_count = recovered_mp_start_errs;
        result->mp_crc_valid = recovered_mp_crc_ok;
        result->mp_fill_valid = recovered_mp_fill_ok;
        have_rx_mp = true;
        result->mp_from_rx_recovery = true;
    }
    if (!have_rx_mp) {
        int target = v34->rx.mp_frame_target;
        int type = v34->rx.mp_frame_bits[18] & 1U;
        int expected_target = type ? 188 : 88;
        int start_errs = -1;
        bool crc_ok = false;
        bool fill_ok = false;

        if (target <= 0)
            target = expected_target;
        if (target >= expected_target) {
            start_errs = v34_mp_start_error_count(v34->rx.mp_frame_bits, type, target);
            crc_ok = v34_mp_crc_ok(v34->rx.mp_frame_bits, type);
            fill_ok = v34_mp_fill_ok(v34->rx.mp_frame_bits, type);
            if (crc_ok
                && fill_ok
                && start_errs <= 2
                && v34_parse_mp_from_raw_bits(v34->rx.mp_frame_bits,
                                              target,
                                              &rx_mp,
                                              &result->mp_frame_bits_captured)
                && v34_mp_semantic_ok_strict(&rx_mp)) {
                have_rx_mp = true;
                result->mp_from_rx_recovery = true;
            }
        }
        result->mp_crc_valid = crc_ok;
        result->mp_fill_valid = fill_ok;
        result->mp_start_error_count = start_errs;
    } else if (!result->mp_from_rx_recovery) {
        int type = v34->rx.mp_frame_bits[18] & 1U;
        int target = v34->rx.mp_frame_target > 0 ? v34->rx.mp_frame_target : (type ? 188 : 88);
        result->mp_crc_valid = v34_mp_crc_ok(v34->rx.mp_frame_bits, type);
        result->mp_fill_valid = v34_mp_fill_ok(v34->rx.mp_frame_bits, type);
        result->mp_start_error_count = v34_mp_start_error_count(v34->rx.mp_frame_bits, type, target);
    }
    if (!have_rx_mp && have_heuristic_mp) {
        result->mp_candidate_seen = true;
        result->mp_candidate_crc_valid = heuristic_mp_crc_ok;
        result->mp_candidate_fill_valid = heuristic_mp_fill_ok;
        result->mp_candidate_start_error_count = heuristic_mp_start_errs;
        result->mp_candidate_votes = heuristic_mp_votes;
        result->mp_candidate_type = heuristic_mp.type;
        result->mp_candidate_trellis_size = heuristic_mp.trellis_size;
        result->mp_candidate_signalling_rate_mask = heuristic_mp.signalling_rate_mask;
        result->mp_candidate_frame_bits_captured = heuristic_mp_bits;
        result->mp_candidate_rate_a_to_c_bps = v34_mp_rate_n_to_bps(heuristic_mp.bit_rate_a_to_c);
        result->mp_candidate_rate_c_to_a_bps = v34_mp_rate_n_to_bps(heuristic_mp.bit_rate_c_to_a);
    }
    if (have_rx_mp) {
        result->mp_from_rx_frame = !result->mp_from_rx_recovery;
        result->mp_type = rx_mp.type;
        result->mp_rate_a_to_c_bps = v34_mp_rate_n_to_bps(rx_mp.bit_rate_a_to_c);
        result->mp_rate_c_to_a_bps = v34_mp_rate_n_to_bps(rx_mp.bit_rate_c_to_a);
        result->mp_aux_channel_supported = (rx_mp.aux_channel_supported != 0);
        result->mp_trellis_size = rx_mp.trellis_size;
        result->mp_use_non_linear_encoder = rx_mp.use_non_linear_encoder;
        result->mp_expanded_shaping = rx_mp.expanded_shaping;
        result->mp_asymmetric_rates_allowed = rx_mp.asymmetric_rates_allowed;
        result->mp_local_ack_bit = rx_mp.mp_acknowledged;
        result->mp_signalling_rate_mask = rx_mp.signalling_rate_mask;
        result->mp_rates_valid = (result->mp_rate_a_to_c_bps > 0 && result->mp_rate_c_to_a_bps > 0);
    } else {
        if (allow_info_rate_infer
            && v34_infer_mp_rates_from_info_sequences(result,
                                                      &result->mp_rate_a_to_c_bps,
                                                      &result->mp_rate_c_to_a_bps)) {
            result->mp_from_info_sequence = true;
        } else {
            result->mp_rate_a_to_c_bps = v34_mp_rate_n_to_bps(v34->tx.mp.bit_rate_a_to_c);
            result->mp_rate_c_to_a_bps = v34_mp_rate_n_to_bps(v34->tx.mp.bit_rate_c_to_a);
        }
        result->mp_rates_valid = (result->mp_rate_a_to_c_bps > 0 && result->mp_rate_c_to_a_bps > 0);
    }
    if (v34->rx.phase3_trn_mag_count > 0) {
        result->trn1u_mag_count = v34->rx.phase3_trn_mag_count;
        result->trn1u_mag_mean = v34->rx.phase3_trn_mag_sum / (float) v34->rx.phase3_trn_mag_count;
    }
    if (v34->rx.phase3_ja_capture_len > 0) {
        int n = v34->rx.phase3_ja_capture_len;

        if (n > (int) sizeof(result->ja_aux_bits) - 1)
            n = (int) sizeof(result->ja_aux_bits) - 1;
        for (int i = 0; i < n; i++)
            result->ja_aux_bits[i] = (v34->rx.phase3_ja_capture[i] & 1U) ? '1' : '0';
        result->ja_aux_bits[n] = '\0';
        result->ja_aux_bit_len = n;
        if (result->ja_observed_bits <= 0)
            result->ja_observed_bits = n;
        if (!result->ja_bits_known && n > 0) {
            int copy_len = n;

            if (copy_len > (int) sizeof(result->ja_bits) - 1)
                copy_len = (int) sizeof(result->ja_bits) - 1;
            memcpy(result->ja_bits, result->ja_aux_bits, (size_t) copy_len);
            result->ja_bits[copy_len] = '\0';
            result->ja_bits_known = true;
            result->ja_bits_from_local_tx = false;
        }
    }
    for (int h = 0; h < 8; h++) {
        int n = v34->rx.phase3_ja_capture_hyp_len[h];
        int raw_n = v34->rx.phase3_ja_capture_hyp_raw_len[h];

        if (n < 0)
            n = 0;
        if (n > (int) sizeof(result->ja_aux_hyp_bits[h]) - 1)
            n = (int) sizeof(result->ja_aux_hyp_bits[h]) - 1;
        result->ja_aux_hyp_bit_len[h] = n;
        for (int i = 0; i < n; i++)
            result->ja_aux_hyp_bits[h][i] = (v34->rx.phase3_ja_capture_hyp[h][i] & 1U) ? '1' : '0';
        result->ja_aux_hyp_bits[h][n] = '\0';

        if (raw_n < 0)
            raw_n = 0;
        if (raw_n > (int) sizeof(result->ja_aux_hyp_raw_bits[h]) - 1)
            raw_n = (int) sizeof(result->ja_aux_hyp_raw_bits[h]) - 1;
        result->ja_aux_hyp_raw_bit_len[h] = raw_n;
        for (int i = 0; i < raw_n; i++)
            result->ja_aux_hyp_raw_bits[h][i] = (v34->rx.phase3_ja_capture_hyp_raw[h][i] & 1U) ? '1' : '0';
        result->ja_aux_hyp_raw_bits[h][raw_n] = '\0';
    }

    if (ja_aux_start_bit >= 0
        && aux_bits.capture_start_bit >= 0
        && aux_bits.total_bits > ja_aux_start_bit
        && aux_bits.capture_len > 0) {
        int available = aux_bits.total_bits - ja_aux_start_bit;
        int copy_aux_len = aux_bits.capture_len;
        int copy_preview_len = aux_bits.capture_len;

        if (copy_aux_len > (int) sizeof(result->ja_aux_bits) - 1)
            copy_aux_len = (int) sizeof(result->ja_aux_bits) - 1;
        memcpy(result->ja_aux_bits, aux_bits.capture, (size_t) copy_aux_len);
        result->ja_aux_bits[copy_aux_len] = '\0';
        result->ja_aux_bit_len = available;
        if (result->ja_aux_bit_len > copy_aux_len)
            result->ja_aux_bit_len = copy_aux_len;
        if (copy_preview_len > (int) sizeof(result->ja_bits) - 1)
            copy_preview_len = (int) sizeof(result->ja_bits) - 1;
        memcpy(result->ja_bits, aux_bits.capture, (size_t) copy_preview_len);
        result->ja_bits[copy_preview_len] = '\0';
        result->ja_observed_bits = available;
        if (result->ja_observed_bits > copy_aux_len)
            result->ja_observed_bits = copy_aux_len;
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

static bool decode_v34_pass(const int16_t *samples,
                            int total_samples,
                            v91_law_t law,
                            bool calling_party,
                            float initial_signal_cutoff_db,
                            bool allow_info_rate_infer,
                            const v34_offline_decode_config_t *config,
                            decode_v34_result_t *result)
{
    return decode_v34_pass_mode(samples,
                                total_samples,
                                law,
                                calling_party,
                                initial_signal_cutoff_db,
                                allow_info_rate_infer,
                                config,
                                false,
                                result);
}

static bool decode_v34_phase2_only_pass(const int16_t *samples,
                                        int total_samples,
                                        v91_law_t law,
                                        bool calling_party,
                                        float info_db_cutoff,
                                        bool allow_info_rate_infer,
                                        const v34_offline_decode_config_t *config,
                                        decode_v34_result_t *result)
{
    return decode_v34_pass_mode(samples,
                                total_samples,
                                law,
                                calling_party,
                                info_db_cutoff,
                                allow_info_rate_infer,
                                config,
                                true,
                                result);
}

/* ------------------------------------------------------------------ */
/*  Lightweight Phase 3 demodulator — supplementary analysis          */
/* ------------------------------------------------------------------ */

static int p3_find_symbol_index_at_or_after(const p3_result_t *detail, int sample)
{
    if (!detail || detail->symbol_count <= 0)
        return 0;
    for (int i = 0; i < detail->symbol_count; i++) {
        if (detail->symbols[i].sample_index >= sample)
            return i;
    }
    return detail->symbol_count;
}

static float p3_repeat6_score(const p3_result_t *detail, int sym_start, int sym_end)
{
    int checks = 0;
    int matches = 0;

    if (!detail || sym_end - sym_start < 12)
        return 0.0f;
    if (sym_start < 0)
        sym_start = 0;
    if (sym_end > detail->symbol_count)
        sym_end = detail->symbol_count;
    for (int i = sym_start + 6; i < sym_end; i++) {
        checks++;
        if (detail->symbols[i].dibit == detail->symbols[i - 6].dibit)
            matches++;
    }
    return (checks > 0) ? ((float) matches / (float) checks) : 0.0f;
}

static int p3_symbol_scrambled_bit(const p3_result_t *detail, int sym_start, int bit_pos)
{
    int sym_idx = sym_start + (bit_pos / 2);
    int bit_sel = bit_pos & 1;
    int dibit;

    if (!detail || sym_idx < 0 || sym_idx >= detail->symbol_count)
        return 0;
    dibit = detail->symbols[sym_idx].dibit;
    return bit_sel ? ((dibit >> 1) & 1) : (dibit & 1);
}

static float p3_trn_recurrence_score(const p3_result_t *detail, int sym_start, int sym_end)
{
    int bits;
    int checks = 0;
    int errors = 0;

    if (!detail)
        return 0.0f;
    if (sym_start < 0)
        sym_start = 0;
    if (sym_end > detail->symbol_count)
        sym_end = detail->symbol_count;
    bits = (sym_end - sym_start) * 2;
    if (bits < 24)
        return 0.0f;

    for (int b = 23; b < bits; b++) {
        int prev23 = p3_symbol_scrambled_bit(detail, sym_start, b - 23);
        int prev5 = p3_symbol_scrambled_bit(detail, sym_start, b - 5);
        int expected = 1 ^ prev23 ^ prev5;
        int got = p3_symbol_scrambled_bit(detail, sym_start, b);
        checks++;
        if (got != expected)
            errors++;
    }
    return (checks > 0) ? ((float) (checks - errors) / (float) checks) : 0.0f;
}

static int p3_segment_descrambled_bit(const p3_result_t *detail,
                                      const p3_segment_t *seg,
                                      int bit_pos)
{
    int sym_idx;
    int bit_sel;

    if (!detail || !seg || bit_pos < 0)
        return 0;
    sym_idx = seg->start_symbol + (bit_pos / 2);
    bit_sel = bit_pos & 1;
    if (sym_idx < 0 || sym_idx >= detail->symbol_count)
        return 0;
    return bit_sel ? detail->symbols[sym_idx].bit1 : detail->symbols[sym_idx].bit0;
}

static void p3_segment_bit_preview(const p3_result_t *detail,
                                   const p3_segment_t *seg,
                                   bool descrambled,
                                   int max_bits,
                                   char *out,
                                   size_t out_len)
{
    int bits;
    int limit;

    if (!out || out_len == 0) {
        return;
    }
    out[0] = '\0';
    if (!detail || !seg || seg->length <= 0)
        return;

    bits = seg->length * 2;
    limit = bits;
    if (max_bits > 0 && limit > max_bits)
        limit = max_bits;
    if (limit > (int) out_len - 1)
        limit = (int) out_len - 1;
    if (limit < 0)
        limit = 0;

    for (int b = 0; b < limit; b++) {
        int bit = descrambled
            ? p3_segment_descrambled_bit(detail, seg, b)
            : p3_symbol_scrambled_bit(detail, seg->start_symbol, b);
        out[b] = bit ? '1' : '0';
    }
    out[limit] = '\0';
}

static float p3_segment_descrambled_one_rate(const p3_result_t *detail,
                                             const p3_segment_t *seg)
{
    int bits;
    int ones = 0;

    if (!detail || !seg || seg->length <= 0)
        return 0.0f;
    bits = seg->length * 2;
    if (bits <= 0)
        return 0.0f;
    for (int b = 0; b < bits; b++) {
        if (p3_segment_descrambled_bit(detail, seg, b))
            ones++;
    }
    return (float) ones / (float) bits;
}

static void p3_u16_to_bitstr(uint16_t value, char out[17])
{
    if (!out)
        return;
    for (int i = 0; i < 16; i++) {
        int bit = (value >> (15 - i)) & 1U;
        out[i] = bit ? '1' : '0';
    }
    out[16] = '\0';
}

typedef enum {
    P3_FLOW_STANDARD_UNKNOWN = 0,
    P3_FLOW_STANDARD_V34,
    P3_FLOW_STANDARD_V90,
    P3_FLOW_STANDARD_V92
} p3_flow_standard_t;

static const char *p3_flow_standard_name(p3_flow_standard_t standard)
{
    switch (standard) {
    case P3_FLOW_STANDARD_V34: return "V.34";
    case P3_FLOW_STANDARD_V90: return "V.90";
    case P3_FLOW_STANDARD_V92: return "V.92";
    default:                   return "unknown";
    }
}

static const char *p3_flow_protocol_name(p3_flow_standard_t standard)
{
    switch (standard) {
    case P3_FLOW_STANDARD_V90: return "V.90 Phase 3";
    case P3_FLOW_STANDARD_V92: return "V.92 Phase 3";
    case P3_FLOW_STANDARD_V34:
    case P3_FLOW_STANDARD_UNKNOWN:
    default:
        return "V.34";
    }
}

static bool p3_result_indicates_v92(const decode_v34_result_t *result)
{
    if (!result || !result->info0_seen)
        return false;
    return v92_short_phase2_v92_cap_from_info0_bits(result->info0_is_d,
                                                     result->info0_raw.raw_26_27);
}

static bool p3_result_indicates_v90(const decode_v34_result_t *result)
{
    if (!result || !result->info1_seen || result->info1_is_d)
        return false;
    return (result->info1a.downstream_rate_code == 6
            || result->info1a.upstream_symbol_rate_code == 6);
}

static bool p3_result_indicates_v34(const decode_v34_result_t *result)
{
    if (!result || !result->info1_seen || result->info1_is_d)
        return false;
    return (result->info1a.downstream_rate_code >= 0
            && result->info1a.downstream_rate_code <= 5);
}

static p3_flow_standard_t p3_detect_flow_standard(const decode_v34_result_t *primary,
                                                  const decode_v34_result_t *peer)
{
    const decode_v34_result_t *candidates[2] = { primary, peer };
    bool has_v92 = false;
    bool has_v90 = false;
    bool has_v34 = false;

    for (int i = 0; i < 2; i++) {
        const decode_v34_result_t *res = candidates[i];

        if (!res)
            continue;
        has_v92 = has_v92 || p3_result_indicates_v92(res);
        has_v90 = has_v90 || p3_result_indicates_v90(res);
        has_v34 = has_v34 || p3_result_indicates_v34(res);
    }

    if (has_v92)
        return P3_FLOW_STANDARD_V92;
    if (has_v90)
        return P3_FLOW_STANDARD_V90;
    if (has_v34)
        return P3_FLOW_STANDARD_V34;
    return P3_FLOW_STANDARD_UNKNOWN;
}

typedef struct {
    p3_flow_standard_t standard;
    bool j_seen;
    int j_symbols;
    int j_table_bits;
    int j_table_match_pct;
    bool jprime_seen;
    int jprime_match_pct;
    bool trn_after_j_seen;
    bool silence_after_j_seen;
    bool ru_seen;
    bool role_aligned;
    char notes[256];
} p3_j_role_eval_t;

static float p3_sequence_guided_score(const p3_result_t *detail,
                                      const decode_v34_result_t *result,
                                      int phase3_start,
                                      int phase3_end);
static bool p3_demod_get_phase3_window(const decode_v34_result_t *result,
                                       int total_samples,
                                       int *phase3_start_out,
                                       int *phase3_end_out);

static void p3_j_role_eval_append(p3_j_role_eval_t *ev, const char *txt)
{
    if (!ev || !txt || !txt[0])
        return;
    if (ev->notes[0])
        appendf(ev->notes, sizeof(ev->notes), "; ");
    appendf(ev->notes, sizeof(ev->notes), "%s", txt);
}

static void p3_evaluate_j_role_flow(const p3_result_t *detail,
                                    bool calling_party,
                                    p3_flow_standard_t standard,
                                    p3_j_role_eval_t *out)
{
    int j_idx = -1;
    const p3_segment_t *j_seg = NULL;
    int next_non_sil = -1;

    if (!out)
        return;
    memset(out, 0, sizeof(*out));
    if (!detail)
        return;
    if (standard == P3_FLOW_STANDARD_UNKNOWN)
        standard = P3_FLOW_STANDARD_V34;
    out->standard = standard;

    for (int i = 0; i < detail->segment_count; i++) {
        const p3_segment_t *seg = &detail->segments[i];
        if (seg->type == P3_SIGNAL_J && (!j_seg || seg->length > j_seg->length)) {
            j_seg = seg;
            j_idx = i;
        }
        if (seg->type == P3_SIGNAL_RU || seg->type == P3_SIGNAL_UR)
            out->ru_seen = true;
    }
    if (!j_seg)
        return;

    out->j_seen = true;
    out->j_symbols = j_seg->length;
    out->j_table_bits = j_seg->j_table_bits;
    out->j_table_match_pct = j_seg->j_table_match_pct;
    if (j_seg->j_table_bits <= 0)
        p3_j_role_eval_append(out, "J table18 class unknown");

    for (int i = j_idx + 1; i < detail->segment_count; i++) {
        const p3_segment_t *seg = &detail->segments[i];
        if (seg->type == P3_SIGNAL_J_PRIME) {
            out->jprime_seen = true;
            out->jprime_match_pct = seg->jprime_match_pct;
        }
        if (seg->type == P3_SIGNAL_TRN)
            out->trn_after_j_seen = true;
        if (next_non_sil < 0
            && seg->type != P3_SIGNAL_SILENCE
            && seg->type != P3_SIGNAL_UNKNOWN)
            next_non_sil = i;
    }
    if (next_non_sil < 0)
        out->silence_after_j_seen = true;
    else
        out->silence_after_j_seen = (detail->segments[next_non_sil].type == P3_SIGNAL_SILENCE);

    if (standard == P3_FLOW_STANDARD_V34) {
        if (calling_party) {
            if (!out->jprime_seen)
                p3_j_role_eval_append(out, "missing J'");
            if (!out->trn_after_j_seen)
                p3_j_role_eval_append(out, "no TRN seen after J/J' in this window");
            out->role_aligned = out->jprime_seen && out->trn_after_j_seen;
        } else {
            if (out->jprime_seen)
                p3_j_role_eval_append(out, "unexpected J' on answer modem path");
            if (!out->silence_after_j_seen && !out->trn_after_j_seen)
                p3_j_role_eval_append(out, "no trailing silence observed after J");
            out->role_aligned = !out->jprime_seen
                                && (out->silence_after_j_seen || !out->trn_after_j_seen);
        }
    } else if (standard == P3_FLOW_STANDARD_V90) {
        if (calling_party) {
            if (out->jprime_seen)
                p3_j_role_eval_append(out, "unexpected J' on analogue-caller Ja path");
            out->role_aligned = !out->jprime_seen;
        } else {
            if (!out->jprime_seen)
                p3_j_role_eval_append(out, "missing J' on digital-answerer Jd path");
            out->role_aligned = out->jprime_seen;
        }
    } else if (standard == P3_FLOW_STANDARD_V92) {
        if (!out->ru_seen)
            p3_j_role_eval_append(out, "missing Ru/uR signature in Phase 3 window");
        if (calling_party) {
            if (out->jprime_seen)
                p3_j_role_eval_append(out, "unexpected J' on analogue-caller Ja/Jp path");
            out->role_aligned = !out->jprime_seen && out->ru_seen;
        } else {
            if (!out->jprime_seen)
                p3_j_role_eval_append(out, "missing Jp' termination on digital-answerer path");
            out->role_aligned = out->jprime_seen && out->ru_seen;
        }
    }
}

typedef struct {
    bool valid;
    p3_flow_standard_t standard;
    int event_sample;
    int j_table_bits;
    int j_table_match_pct;
    bool jprime_seen;
    int jprime_match_pct;
    bool trn_after_j_seen;
    bool ru_seen;
    bool role_aligned;
    char notes[256];
} p3_j_log_summary_t;

static bool p3_extract_j_log_summary(const int16_t *samples,
                                     int total_samples,
                                     const decode_v34_result_t *result,
                                     bool calling_party,
                                     p3_flow_standard_t standard,
                                     p3_j_log_summary_t *out)
{
    p3_hypothesis_t hypotheses[P3_BAUD_COUNT * 2];
    p3_result_t *best_detail = NULL;
    int phase3_start = -1;
    int phase3_end = -1;
    int count;
    float best_total = -1.0e30f;
    const p3_segment_t *best_j = NULL;
    const p3_segment_t *best_ru = NULL;
    p3_j_role_eval_t eval;

    if (!out)
        return false;
    memset(out, 0, sizeof(*out));
    if (!samples || total_samples <= 0 || !result)
        return false;
    if (!p3_demod_get_phase3_window(result, total_samples, &phase3_start, &phase3_end))
        return false;
    if (phase3_end <= phase3_start)
        return false;

    count = p3_scan_all_hypotheses(samples + phase3_start,
                                   phase3_end - phase3_start,
                                   phase3_start,
                                   8000,
                                   hypotheses,
                                   P3_BAUD_COUNT * 2);
    if (count <= 0)
        return false;

    for (int i = 0; i < count; i++) {
        const p3_hypothesis_t *h = &hypotheses[i];
        p3_result_t *candidate = p3_demod_run(samples + phase3_start,
                                              phase3_end - phase3_start,
                                              phase3_start,
                                              h->baud_code,
                                              h->carrier_sel,
                                              8000);
        float seq_score;
        float total_score;

        if (!candidate)
            continue;
        seq_score = p3_sequence_guided_score(candidate, result, phase3_start, phase3_end);
        total_score = h->score + seq_score;
        if (!best_detail || total_score > best_total) {
            if (best_detail)
                p3_result_free(best_detail);
            best_detail = candidate;
            best_total = total_score;
        } else {
            p3_result_free(candidate);
        }
    }
    if (!best_detail)
        return false;

    p3_evaluate_j_role_flow(best_detail, calling_party, standard, &eval);
    if (!eval.j_seen && !(standard == P3_FLOW_STANDARD_V92 && eval.ru_seen)) {
        p3_result_free(best_detail);
        return false;
    }

    for (int i = 0; i < best_detail->segment_count; i++) {
        const p3_segment_t *seg = &best_detail->segments[i];
        if (seg->type == P3_SIGNAL_J && (!best_j || seg->length > best_j->length))
            best_j = seg;
        if ((seg->type == P3_SIGNAL_RU || seg->type == P3_SIGNAL_UR)
            && (!best_ru || seg->length > best_ru->length)) {
            best_ru = seg;
        }
    }
    out->valid = true;
    out->standard = eval.standard;
    out->event_sample = best_j
        ? best_j->start_sample
        : (best_ru ? best_ru->start_sample : result->tx_ja_sample);
    out->j_table_bits = eval.j_table_bits;
    out->j_table_match_pct = eval.j_table_match_pct;
    out->jprime_seen = eval.jprime_seen;
    out->jprime_match_pct = eval.jprime_match_pct;
    out->trn_after_j_seen = eval.trn_after_j_seen;
    out->ru_seen = eval.ru_seen;
    out->role_aligned = eval.role_aligned;
    if (eval.notes[0])
        snprintf(out->notes, sizeof(out->notes), "%s", eval.notes);

    p3_result_free(best_detail);
    return true;
}

static void p3_print_data_signatures(const p3_result_t *detail)
{
    const p3_segment_t *best_j = NULL;
    const p3_segment_t *best_jprime = NULL;
    const p3_segment_t *best_pp = NULL;
    const p3_segment_t *best_trn = NULL;
    const p3_segment_t *best_ru = NULL;
    int s_like_symbols = 0;
    int pp_symbols = 0;
    int trn_symbols = 0;
    int j_symbols = 0;
    int jprime_symbols = 0;
    int ru_symbols = 0;

    if (!detail)
        return;

    for (int i = 0; i < detail->segment_count; i++) {
        const p3_segment_t *seg = &detail->segments[i];
        switch (seg->type) {
        case P3_SIGNAL_S:
        case P3_SIGNAL_S_BAR:
            s_like_symbols += seg->length;
            break;
        case P3_SIGNAL_TRN:
            trn_symbols += seg->length;
            if (!best_trn || seg->length > best_trn->length)
                best_trn = seg;
            break;
        case P3_SIGNAL_PP:
            pp_symbols += seg->length;
            if (!best_pp || seg->length > best_pp->length)
                best_pp = seg;
            break;
        case P3_SIGNAL_J:
            j_symbols += seg->length;
            if (!best_j || seg->length > best_j->length)
                best_j = seg;
            break;
        case P3_SIGNAL_J_PRIME:
            jprime_symbols += seg->length;
            if (!best_jprime || seg->length > best_jprime->length)
                best_jprime = seg;
            break;
        case P3_SIGNAL_RU:
        case P3_SIGNAL_UR:
            ru_symbols += seg->length;
            if (!best_ru || seg->length > best_ru->length)
                best_ru = seg;
            break;
        default:
            break;
        }
    }

    printf("    Data signatures: S-like=%d sym, PP=%d sym, TRN=%d sym, J=%d sym, J'=%d sym, Ru/uR=%d sym\n",
           s_like_symbols, pp_symbols, trn_symbols, j_symbols, jprime_symbols, ru_symbols);

    if (best_pp) {
        printf("      PP longest: %d sym (%d x 48T), phase=%d, corr=%.2f\n",
               best_pp->length,
               best_pp->pp_blocks,
               best_pp->pp_phase,
               best_pp->confidence);
    }

    if (best_j) {
        char trn16_bits[17];
        char j_preview[65];
        const char *j_table_label = "unknown";

        p3_u16_to_bitstr(best_j->j_trn16, trn16_bits);
        p3_segment_bit_preview(detail, best_j, true, 64, j_preview, sizeof(j_preview));
        if (best_j->j_table_bits == 4)
            j_table_label = "table18-4pt";
        else if (best_j->j_table_bits == 16)
            j_table_label = "table18-16pt";
        printf("      J longest: %d sym, trn16=0x%04X (%s), phase=%d, bits(desc)=%s\n",
               best_j->length,
               best_j->j_trn16,
               trn16_bits,
               best_j->j_hypothesis,
               j_preview);
        printf("        J table18: %s match=%d%% phase=%d xform=%d",
               j_table_label,
               best_j->j_table_match_pct,
               best_j->j_table_phase,
               best_j->j_table_transform);
        if (best_j->jprime_match_pct > 0)
            printf(" j'after=%d%%", best_j->jprime_match_pct);
        printf("\n");
    }

    if (best_jprime) {
        printf("      J' longest: %d sym (one-shot) match=%d%% phase=%d xform=%d\n",
               best_jprime->length,
               best_jprime->jprime_match_pct,
               best_jprime->j_table_phase,
               best_jprime->j_table_transform);
    }

    if (best_trn) {
        char trn_scr[65];
        char trn_desc[65];
        float trn_rec = p3_trn_recurrence_score(detail,
                                                best_trn->start_symbol,
                                                best_trn->start_symbol + best_trn->length);
        float one_rate = p3_segment_descrambled_one_rate(detail, best_trn);

        p3_segment_bit_preview(detail, best_trn, false, 64, trn_scr, sizeof(trn_scr));
        p3_segment_bit_preview(detail, best_trn, true, 64, trn_desc, sizeof(trn_desc));
        printf("      TRN longest: %d sym, recurrence=%.1f%%, ones(desc)=%.1f%%\n",
               best_trn->length,
               100.0f * trn_rec,
               100.0f * one_rate);
        printf("        bits(scr)=%s\n", trn_scr);
        printf("        bits(desc)=%s\n", trn_desc);
    }

    if (best_ru) {
        char ru_bits[49];
        p3_segment_bit_preview(detail, best_ru, false, 48, ru_bits, sizeof(ru_bits));
        printf("      %s longest: %d sym, orientation=%s, dibit-bits(scr)=%s\n",
               (best_ru->type == P3_SIGNAL_RU) ? "Ru" : "uR",
               best_ru->length,
               best_ru->ru_positive_first ? "+first" : "-first",
               ru_bits);
    }
}

static float p3_sequence_guided_score(const p3_result_t *detail,
                                      const decode_v34_result_t *result,
                                      int phase3_start,
                                      int phase3_end)
{
    int s_start;
    int sbar_start;
    int trn_start;
    int phase3_len;
    int i0, i1, i2, i3;
    float s_score;
    float sbar_score;
    float trn_score;

    if (!detail || !result || phase3_end <= phase3_start)
        return 0.0f;

    s_start = (result->tx_first_s_sample >= 0) ? result->tx_first_s_sample : phase3_start;
    sbar_start = result->tx_first_not_s_sample;
    trn_start = result->tx_trn_sample;
    phase3_len = phase3_end - phase3_start;

    if (sbar_start < 0 || sbar_start <= s_start)
        sbar_start = s_start + phase3_len/3;
    if (trn_start < 0 || trn_start <= sbar_start)
        trn_start = sbar_start + phase3_len/3;

    if (s_start < phase3_start) s_start = phase3_start;
    if (sbar_start < phase3_start) sbar_start = phase3_start;
    if (trn_start < phase3_start) trn_start = phase3_start;
    if (s_start > phase3_end) s_start = phase3_end;
    if (sbar_start > phase3_end) sbar_start = phase3_end;
    if (trn_start > phase3_end) trn_start = phase3_end;

    if (sbar_start <= s_start)
        sbar_start = s_start;
    if (trn_start <= sbar_start)
        trn_start = sbar_start;

    i0 = p3_find_symbol_index_at_or_after(detail, s_start);
    i1 = p3_find_symbol_index_at_or_after(detail, sbar_start);
    i2 = p3_find_symbol_index_at_or_after(detail, trn_start);
    i3 = detail->symbol_count;

    if (i1 < i0) i1 = i0;
    if (i2 < i1) i2 = i1;

    s_score = p3_repeat6_score(detail, i0, i1);
    sbar_score = p3_repeat6_score(detail, i1, i2);
    trn_score = p3_trn_recurrence_score(detail, i2, i3);

    return 28.0f*s_score + 28.0f*sbar_score + 44.0f*trn_score;
}

static void p3_demod_analyse_phase3(const int16_t *samples,
                                    int total_samples,
                                    const decode_v34_result_t *result,
                                    bool calling_party)
{
    int phase3_start = -1;
    int phase3_end = -1;
    int phase3_len;
    const char *role = calling_party ? "caller" : "answerer";
    p3_flow_standard_t flow_standard;

    /* Determine Phase 3 sample range from V.34 decode results */
    if (!result)
        return;

    phase3_start = result->phase3_sample;
    if (phase3_start < 0)
        phase3_start = result->tx_first_s_sample;
    if (phase3_start < 0)
        return;

    phase3_end = result->phase4_sample;
    if (phase3_end < 0)
        phase3_end = result->phase4_ready_sample;
    if (phase3_end < 0)
        phase3_end = result->failure_sample;
    if (phase3_end < 0 || phase3_end > total_samples)
        phase3_end = total_samples;
    if (phase3_end <= phase3_start)
        return;

    phase3_len = phase3_end - phase3_start;
    if (phase3_len < 200)
        return;
    flow_standard = p3_detect_flow_standard(result, NULL);
    if (flow_standard == P3_FLOW_STANDARD_UNKNOWN)
        flow_standard = P3_FLOW_STANDARD_V34;

    printf("\n  --- p3_demod Phase 3 analysis (%s) ---\n", role);
    printf("  Flow target: %s\n", p3_flow_standard_name(flow_standard));
    printf("  Sample range: %d – %d (%.1f ms – %.1f ms, %.1f ms)\n",
           phase3_start, phase3_end,
           sample_to_ms(phase3_start, 8000),
           sample_to_ms(phase3_end, 8000),
           (float)phase3_len / 8.0f);

    /* Try all baud/carrier combinations */
    {
        p3_hypothesis_t hypotheses[P3_BAUD_COUNT * 2];
        int count;
        int best_idx = -1;
        float best_total_score = -1.0f;
        float best_base_score = 0.0f;
        float best_seq_score = 0.0f;
        p3_result_t *best_detail = NULL;

        count = p3_scan_all_hypotheses(samples + phase3_start,
                                       phase3_len,
                                       phase3_start,
                                       8000,
                                       hypotheses,
                                       P3_BAUD_COUNT * 2);

        printf("  Hypotheses tested: %d\n", count);
        for (int i = 0; i < count; i++) {
            const p3_hypothesis_t *h = &hypotheses[i];
            p3_result_t *candidate = p3_demod_run(samples + phase3_start,
                                                  phase3_len,
                                                  phase3_start,
                                                  h->baud_code,
                                                  h->carrier_sel,
                                                  8000);
            float seq_score = p3_sequence_guided_score(candidate, result, phase3_start, phase3_end);
            float total_score = h->score + seq_score;

            if (total_score > best_total_score) {
                best_total_score = total_score;
                best_base_score = h->score;
                best_seq_score = seq_score;
                best_idx = i;
                if (best_detail)
                    p3_result_free(best_detail);
                best_detail = candidate;
                candidate = NULL;
            }
            printf("    [%d] %d baud %s carrier (%.0f Hz): %d symbols, %d segments, score=%.1f seq=%.1f total=%.1f%s%s%s%s\n",
                   i,
                   (int)h->baud_rate,
                   h->carrier_sel == P3_CARRIER_HIGH ? "high" : "low",
                   h->carrier_hz,
                   h->symbol_count,
                   h->segment_count,
                   h->score,
                   seq_score,
                   total_score,
                   h->has_s ? " [S]" : "",
                   h->has_trn ? " [TRN]" : "",
                   h->has_j ? " [J]" : "",
                   h->has_ru ? " [Ru]" : "");
            if (candidate)
                p3_result_free(candidate);
        }

        if (best_idx >= 0 && best_total_score > 0.0f) {
            const p3_hypothesis_t *best = &hypotheses[best_idx];
            p3_result_t *detail = best_detail;

            printf("  Best: %d baud %s carrier (%.0f Hz), total=%.1f (base=%.1f seq=%.1f)\n",
                   (int)best->baud_rate,
                   best->carrier_sel == P3_CARRIER_HIGH ? "high" : "low",
                   best->carrier_hz,
                   best_total_score,
                   best_base_score,
                   best_seq_score);

            /* Run detailed demod on the best hypothesis */
            if (!detail) {
                detail = p3_demod_run(samples + phase3_start,
                                      phase3_len,
                                      phase3_start,
                                      best->baud_code,
                                      best->carrier_sel,
                                      8000);
            }
            if (detail) {
                p3_j_role_eval_t j_eval;

                printf("  Symbols: %d, Segments: %d\n",
                       detail->symbol_count, detail->segment_count);
                printf("  Carrier estimate: %.1f Hz, SNR: %.1f dB\n",
                       detail->carrier_freq_estimate,
                       detail->snr_estimate_db);

                for (int i = 0; i < detail->segment_count; i++) {
                    const p3_segment_t *seg = &detail->segments[i];
                    if (!(seg->type == P3_SIGNAL_S
                          || seg->type == P3_SIGNAL_S_BAR
                          || seg->type == P3_SIGNAL_PP
                          || seg->type == P3_SIGNAL_TRN
                          || seg->type == P3_SIGNAL_J
                          || seg->type == P3_SIGNAL_J_PRIME
                          || seg->type == P3_SIGNAL_RU
                          || seg->type == P3_SIGNAL_UR))
                        continue;
                    printf("    [%d] %s: symbols %d–%d (%d), samples %d–%d (%.1f–%.1f ms), mag=%.3f conf=%.2f",
                           i,
                           p3_signal_type_name(seg->type),
                           seg->start_symbol,
                           seg->start_symbol + seg->length - 1,
                           seg->length,
                           seg->start_sample,
                           seg->end_sample,
                           sample_to_ms(seg->start_sample, 8000),
                           sample_to_ms(seg->end_sample, 8000),
                           seg->avg_magnitude,
                           seg->confidence);
                    if (seg->type == P3_SIGNAL_TRN)
                        printf(" errors=%d", seg->trn_errors);
                    if (seg->type == P3_SIGNAL_PP)
                        printf(" blocks=%d phase=%d", seg->pp_blocks, seg->pp_phase);
                    if (seg->type == P3_SIGNAL_J) {
                        printf(" trn16=0x%04X", seg->j_trn16);
                        if (seg->j_table_bits > 0)
                            printf(" table18=%dpt(%d%%) phase=%d xform=%d",
                                   seg->j_table_bits,
                                   seg->j_table_match_pct,
                                   seg->j_table_phase,
                                   seg->j_table_transform);
                    }
                    if (seg->type == P3_SIGNAL_J_PRIME && seg->jprime_match_pct > 0)
                        printf(" table19=%d%% phase=%d xform=%d",
                               seg->jprime_match_pct,
                               seg->j_table_phase,
                               seg->j_table_transform);
                    if (seg->type == P3_SIGNAL_RU || seg->type == P3_SIGNAL_UR)
                        printf(" %s-first", seg->ru_positive_first ? "+" : "-");
                    printf("\n");
                }
                p3_evaluate_j_role_flow(detail, calling_party, flow_standard, &j_eval);
                if (j_eval.j_seen) {
                    printf("    J flow (%s/%s): class=%s(%d%%) J'=%s",
                           p3_flow_standard_name(flow_standard),
                           calling_party ? "caller" : "answerer",
                           j_eval.j_table_bits == 4 ? "4pt"
                           : (j_eval.j_table_bits == 16 ? "16pt" : "unknown"),
                           j_eval.j_table_match_pct,
                           j_eval.jprime_seen ? "yes" : "no");
                    if (j_eval.jprime_seen && j_eval.jprime_match_pct > 0)
                        printf("(%d%%)", j_eval.jprime_match_pct);
                    printf(" trn_after=%s ru=%s status=%s\n",
                           j_eval.trn_after_j_seen ? "yes" : "no",
                           j_eval.ru_seen ? "yes" : "no",
                           j_eval.role_aligned ? "aligned" : "partial");
                    if (j_eval.notes[0])
                        printf("      notes: %s\n", j_eval.notes);
                }
                p3_print_data_signatures(detail);
                p3_result_free(detail);
            }
        } else {
            if (best_detail)
                p3_result_free(best_detail);
            printf("  No recognizable Phase 3 signals found by p3_demod\n");
        }
    }
}

static bool p3_demod_get_phase3_window(const decode_v34_result_t *result,
                                       int total_samples,
                                       int *phase3_start_out,
                                       int *phase3_end_out)
{
    int phase3_start;
    int phase3_end;

    if (!result || !phase3_start_out || !phase3_end_out || total_samples <= 0)
        return false;

    phase3_start = result->phase3_sample;
    if (phase3_start < 0)
        phase3_start = result->tx_first_s_sample;
    if (phase3_start < 0 || phase3_start >= total_samples)
        return false;

    phase3_end = result->phase4_sample;
    if (phase3_end < 0)
        phase3_end = result->phase4_ready_sample;
    if (phase3_end < 0)
        phase3_end = result->failure_sample;
    if (phase3_end < 0 || phase3_end > total_samples)
        phase3_end = total_samples;
    if (phase3_end <= phase3_start)
        return false;

    if ((phase3_end - phase3_start) < 200)
        return false;

    *phase3_start_out = phase3_start;
    *phase3_end_out = phase3_end;
    return true;
}

static bool get_cached_v8_channel_probe(const int16_t *samples,
                                        int total_samples,
                                        int max_sample,
                                        v8_probe_result_t *out)
{
    v8_probe_cache_entry_t *slot;

    if (!samples || total_samples <= 0 || !out)
        return false;

    for (int i = 0; i < VPCM_PREPASS_CACHE_SLOTS; i++) {
        const v8_probe_cache_entry_t *entry = &g_v8_probe_cache[i];

        if (!entry->valid)
            continue;
        if (entry->samples != samples || entry->total_samples != total_samples || entry->max_sample != max_sample)
            continue;
        *out = entry->probe;
        return true;
    }

    if (!v8_select_best_channel_probe(samples, total_samples, max_sample, out))
        return false;

    slot = &g_v8_probe_cache[g_v8_probe_cache_next];
    g_v8_probe_cache_next = (g_v8_probe_cache_next + 1) % VPCM_PREPASS_CACHE_SLOTS;
    slot->valid = true;
    slot->samples = samples;
    slot->total_samples = total_samples;
    slot->max_sample = max_sample;
    slot->probe = *out;
    return true;
}

static void p3_demod_scan_window(const int16_t *samples,
                                 int sample_count,
                                 int sample_offset,
                                 int sample_rate,
                                 const char *label)
{
    p3_hypothesis_t hypotheses[P3_BAUD_COUNT * 2];
    int count;
    int best_idx = -1;
    float best_score = -1.0f;

    if (!samples || sample_count <= 0)
        return;

    printf("  %s: %.1f–%.1f ms (%.1f ms)\n",
           label,
           sample_to_ms(sample_offset, sample_rate),
           sample_to_ms(sample_offset + sample_count, sample_rate),
           sample_to_ms(sample_count, sample_rate));

    /* p3_scan_all_hypotheses internally caps/narrows the scan range;
     * cap the detail run similarly to avoid huge demod on full streams. */
    {
        int detail_cap = 5 * sample_rate;
        if (sample_count > detail_cap)
            sample_count = detail_cap;
    }

    printf("  Scanning all %d baud/carrier hypotheses...\n", P3_BAUD_COUNT * 2);

    count = p3_scan_all_hypotheses(samples,
                                   sample_count,
                                   sample_offset,
                                   sample_rate,
                                   hypotheses,
                                   P3_BAUD_COUNT * 2);

    for (int i = 0; i < count; i++) {
        const p3_hypothesis_t *h = &hypotheses[i];
        if (h->score > best_score) {
            best_score = h->score;
            best_idx = i;
        }
        printf("    [%d] %d baud %s (%.0f Hz): %d sym, %d seg, score=%.1f%s%s%s%s\n",
               i,
               (int) h->baud_rate,
               h->carrier_sel == P3_CARRIER_HIGH ? "high" : "low",
               h->carrier_hz,
               h->symbol_count,
               h->segment_count,
               h->score,
               h->has_s ? " [S]" : "",
               h->has_trn ? " [TRN]" : "",
               h->has_j ? " [J]" : "",
               h->has_ru ? " [Ru]" : "");
    }

    if (best_idx >= 0 && best_score > 0.0f) {
        const p3_hypothesis_t *best = &hypotheses[best_idx];
        p3_result_t *detail;

        printf("    Best: %d baud %s (%.0f Hz), score=%.1f\n",
               (int) best->baud_rate,
               best->carrier_sel == P3_CARRIER_HIGH ? "high" : "low",
               best->carrier_hz,
               best->score);

        detail = p3_demod_run(samples,
                              sample_count,
                              sample_offset,
                              best->baud_code,
                              best->carrier_sel,
                              sample_rate);
        if (detail) {
            p3_j_role_eval_t j_eval;

            printf("    Symbols: %d, Segments: %d\n",
                   detail->symbol_count, detail->segment_count);
            printf("    Carrier: %.1f Hz, SNR: %.1f dB\n",
                   detail->carrier_freq_estimate,
                   detail->snr_estimate_db);

            for (int i = 0; i < detail->segment_count; i++) {
                const p3_segment_t *seg = &detail->segments[i];
                if (seg->type == P3_SIGNAL_SILENCE || seg->type == P3_SIGNAL_UNKNOWN)
                    continue;
                printf("      %s: %.1f–%.1f ms (%d sym) mag=%.3f conf=%.2f",
                       p3_signal_type_name(seg->type),
                       sample_to_ms(seg->start_sample, sample_rate),
                       sample_to_ms(seg->end_sample, sample_rate),
                       seg->length,
                       seg->avg_magnitude,
                       seg->confidence);
                if (seg->type == P3_SIGNAL_TRN)
                    printf(" errors=%d", seg->trn_errors);
                if (seg->type == P3_SIGNAL_PP)
                    printf(" blocks=%d phase=%d", seg->pp_blocks, seg->pp_phase);
                if (seg->type == P3_SIGNAL_J) {
                    printf(" trn16=0x%04X", seg->j_trn16);
                    if (seg->j_table_bits > 0)
                        printf(" table18=%dpt(%d%%) phase=%d xform=%d",
                               seg->j_table_bits,
                               seg->j_table_match_pct,
                               seg->j_table_phase,
                               seg->j_table_transform);
                }
                if (seg->type == P3_SIGNAL_J_PRIME && seg->jprime_match_pct > 0)
                    printf(" table19=%d%% phase=%d xform=%d",
                           seg->jprime_match_pct,
                           seg->j_table_phase,
                           seg->j_table_transform);
                if (seg->type == P3_SIGNAL_RU || seg->type == P3_SIGNAL_UR)
                    printf(" %s-first", seg->ru_positive_first ? "+" : "-");
                printf("\n");
            }
            p3_evaluate_j_role_flow(detail, false, P3_FLOW_STANDARD_UNKNOWN, &j_eval);
            if (j_eval.j_seen) {
                printf("      J flow (answerer): class=%s(%d%%) J'=%s",
                       j_eval.j_table_bits == 4 ? "4pt"
                       : (j_eval.j_table_bits == 16 ? "16pt" : "unknown"),
                       j_eval.j_table_match_pct,
                       j_eval.jprime_seen ? "yes" : "no");
                if (j_eval.jprime_seen && j_eval.jprime_match_pct > 0)
                    printf("(%d%%)", j_eval.jprime_match_pct);
                printf(" trn_after=%s ru=%s status=%s\n",
                       j_eval.trn_after_j_seen ? "yes" : "no",
                       j_eval.ru_seen ? "yes" : "no",
                       j_eval.role_aligned ? "aligned" : "partial");
                if (j_eval.notes[0])
                    printf("        notes: %s\n", j_eval.notes);
            }
            p3_print_data_signatures(detail);
            p3_result_free(detail);
        }
    } else {
        printf("    No recognizable Phase 3 signals found\n");
    }
}

static void print_v34_result(const decode_v34_result_t *result,
                             bool calling_party,
                             int expected_rate_1,
                             int expected_rate_2,
                             bool suppress_v90_phase2)
{
    v34_phase3_errorfree_eval_t ef_eval;
    p3_flow_standard_t phase3_target;

    if (!result)
        return;
    phase3_target = p3_detect_flow_standard(result, NULL);
    if (phase3_target == P3_FLOW_STANDARD_UNKNOWN)
        phase3_target = suppress_v90_phase2 ? P3_FLOW_STANDARD_V34 : P3_FLOW_STANDARD_V90;

    printf("  Role:            %s\n", calling_party ? "caller" : "answerer");
    if (!suppress_v90_phase2) {
        printf("  Phase 3 target:  %s\n",
               phase3_target == P3_FLOW_STANDARD_UNKNOWN ? "unknown" : p3_flow_standard_name(phase3_target));
    }
    printf("  Final RX stage:  %s (%d)\n",
           v34_rx_stage_to_str_local(result->final_rx_stage),
           result->final_rx_stage);
    printf("  Final TX stage:  %s (%d)\n",
           v34_tx_stage_to_str_local(result->final_tx_stage),
           result->final_tx_stage);
    printf("  Final RX event:  %s (%d)\n",
           v34_event_to_str_local(result->final_rx_event),
           result->final_rx_event);
    if (result->phase2_selected_window_index >= 0) {
        printf("  Phase 2 window:  #%d %.1f-%.1f ms tried=%d/%d active_hits=%d peak=%.1f dB\n",
               result->phase2_selected_window_index + 1,
               sample_to_ms(result->phase2_selected_window_start_sample, 8000),
               sample_to_ms(result->phase2_selected_window_end_sample, 8000),
               result->phase2_candidate_windows_tried,
               result->phase2_candidate_windows_seen,
               result->phase2_selected_window_active_hits,
               (double) result->phase2_selected_window_peak_db_tenths / 10.0);
    }
    if (suppress_v90_phase2) {
        printf("  INFO0a/INFO0d:   suppressed by V.8 gate (no V.90/V.92 digital capability in CM/JM)\n");
    } else if (result->info0_seen) {
        char info0_detail[1024];

        printf("  %-16sdecoded at %.1f ms\n",
               result->info0_is_d ? "INFO0d:" : "INFO0a:",
               sample_to_ms(result->info0_sample, 8000));
        if (result->info0_from_rescue)
            printf("                   source=rescue_pass\n");
        format_v34_info0_table_summary(info0_detail,
                                       sizeof(info0_detail),
                                       &result->info0a,
                                       &result->info0_raw,
                                       "active_pass",
                                       result->info0_is_d);
        printf("                   %s\n", info0_detail);
    } else {
        printf("  INFO0a/INFO0d:   not decoded");
        if (result->info0_bad_event_seen)
            printf(" (INFO0_BAD at %.1f ms)", sample_to_ms(result->info0_bad_event_sample, 8000));
        else if (result->info0_ok_event_seen)
            printf(" (INFO0_OK event seen at %.1f ms; frame payload unavailable)", sample_to_ms(result->info0_ok_event_sample, 8000));
        printf("\n");
    }
    if (suppress_v90_phase2) {
        printf("  INFO1a/INFO1d:   suppressed by V.8 gate (no V.90/V.92 digital capability in CM/JM)\n");
    } else if (result->info1_seen && !result->info1_is_d) {
        char info1_detail[1024];

        printf("  INFO1a:          decoded at %.1f ms\n", sample_to_ms(result->info1_sample, 8000));
        if (result->info1_from_rescue)
            printf("                   source=rescue_pass\n");
        format_v34_info1_table15_summary(info1_detail,
                                         sizeof(info1_detail),
                                         &result->info1a,
                                         &result->info1a_raw,
                                         "active_pass");
        printf("                   %s\n", info1_detail);
    } else if (result->info1_seen) {
        char info1_detail[512];

        printf("  INFO1d:          decoded at %.1f ms\n", sample_to_ms(result->info1_sample, 8000));
        if (result->info1_from_rescue)
            printf("                   source=rescue_pass\n");
        format_v34_info1d_table_summary(info1_detail,
                                        sizeof(info1_detail),
                                        &result->info1d,
                                        "active_pass");
        printf("                   %s\n", info1_detail);
    } else {
        printf("  INFO1a/INFO1d:   not decoded");
        if (result->info1_bad_event_seen)
            printf(" (INFO1_BAD at %.1f ms)", sample_to_ms(result->info1_bad_event_sample, 8000));
        else if (result->info1_ok_event_seen)
            printf(" (INFO1_OK event seen at %.1f ms; frame payload unavailable)", sample_to_ms(result->info1_ok_event_sample, 8000));
        else if (result->final_rx_stage == 4)
            printf(" (stalled in INFO1A stage)");
        printf("\n");
    }
    if (!suppress_v90_phase2 && result->u_info > 0) {
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

    v34_evaluate_errorfree_phase3(result, &ef_eval);
    if (ef_eval.evaluated) {
        printf("  11.3.1 flow:     %s (%s side) seq=%s trn=%s handoff=%s",
               ef_eval.pass ? "PASS" : "PARTIAL",
               calling_party ? "call modem" : "answer modem",
               ef_eval.sequence_ok ? "ok" : "issue",
               ef_eval.trn_min_ok ? ">=512T" : "<512T/unknown",
               ef_eval.handoff_ok ? "ok" : "issue");
        if (ef_eval.trn_t >= 0)
            printf(" trn=%dT", ef_eval.trn_t);
        if (ef_eval.s1_t >= 0)
            printf(" s1=%dT", ef_eval.s1_t);
        if (ef_eval.s2_t >= 0)
            printf(" s2=%dT", ef_eval.s2_t);
        printf("\n");
        if (ef_eval.notes[0])
            printf("                   notes: %s\n", ef_eval.notes);
    }

    if (!suppress_v90_phase2 && result->info1_seen && !result->info1_is_d) {
        int up_code = result->info1a.upstream_symbol_rate_code;
        int dn_code = result->info1a.downstream_rate_code;
        int up_baud = v34_symbol_rate_code_to_baud(up_code);
        int dn_baud = v34_symbol_rate_code_to_baud(dn_code);
        int up_max = v34_symbol_rate_code_to_theoretical_max_bps(up_code);
        int dn_max = v34_symbol_rate_code_to_theoretical_max_bps(dn_code);

        printf("  INFO params:     up_code=%d", up_code);
        if (up_baud > 0)
            printf(" (%d baud)", up_baud);
        else
            printf(" (unknown)");
        printf(" down_code=%d", dn_code);
        if (dn_baud == 8000)
            printf(" (8000 pcm)");
        else if (dn_baud > 0)
            printf(" (%d baud)", dn_baud);
        if (up_max > 0 || dn_max > 0) {
            printf(" inferred_max A->C/C->A=");
            if (up_max > 0)
                printf("%d", up_max);
            else
                printf("n/a");
            printf("/");
            if (dn_max > 0)
                printf("%d", dn_max);
            else
                printf("n/a");
            printf(" bps");
        }
        printf("\n");
    } else if (!suppress_v90_phase2 && result->info1_seen && result->info1_is_d) {
        int best_idx = -1;
        int best_n = -1;

        for (int i = 0; i < 6; i++) {
            if (result->info1d.rate_data[i].max_bit_rate > best_n) {
                best_n = result->info1d.rate_data[i].max_bit_rate;
                best_idx = i;
            }
        }
        if (best_idx >= 0 && best_n > 0) {
            static const int baud_list[6] = {2400, 2743, 2800, 3000, 3200, 3429};
            printf("  INFO params:     best_row=%d (%d baud) max_rate=%d bps carrier=%s preemph=%d\n",
                   best_idx,
                   baud_list[best_idx],
                   best_n * 2400,
                   result->info1d.rate_data[best_idx].use_high_carrier ? "high" : "low",
                   result->info1d.rate_data[best_idx].pre_emphasis);
        }
    }
    if (result->mp_seen || result->mp_rates_valid) {
        printf("  MP exchange:     seen=%s remote_ack=%s",
               result->mp_seen ? "yes" : "no",
               result->mp_remote_ack_seen ? "yes" : "no");
        printf(" source=%s",
               result->mp_from_rx_recovery ? "rx_frame_recovered"
               : (result->mp_from_rx_frame ? "rx_frame"
                  : (result->mp_from_info_sequence ? "info_sequence" : "tx_state")));
        if (result->mp_rates_valid) {
            printf(" negotiated A->C/C->A=%d/%d bps",
                   result->mp_rate_a_to_c_bps,
                   result->mp_rate_c_to_a_bps);
            if (expected_rate_1 > 0 && expected_rate_2 > 0) {
                bool direct_match = (result->mp_rate_a_to_c_bps == expected_rate_1
                                     && result->mp_rate_c_to_a_bps == expected_rate_2);
                bool swapped_match = (result->mp_rate_a_to_c_bps == expected_rate_2
                                      && result->mp_rate_c_to_a_bps == expected_rate_1);

                printf(" filename=%d/%d", expected_rate_1, expected_rate_2);
                if (direct_match)
                    printf(" match=direct");
                else if (swapped_match)
                    printf(" match=swapped");
                else
                    printf(" match=no");
            }
        }
        printf("\n");
        if (result->mp_from_rx_frame || result->mp_from_rx_recovery) {
            char rate_mask[160];

            v34_mp_rate_mask_to_text(result->mp_signalling_rate_mask, rate_mask, sizeof(rate_mask));
            printf("  MP decode:       type=MP%d bits=%d local_ack=%s trellis=%s aux=%s q3125=%s shaping=%s asym=%s mask={%s}%s crc=%s fill=%s start_err=%d\n",
                   result->mp_type,
                   result->mp_frame_bits_captured,
                   result->mp_local_ack_bit ? "yes" : "no",
                   v34_mp_trellis_name(result->mp_trellis_size),
                   result->mp_aux_channel_supported ? "yes" : "no",
                   result->mp_use_non_linear_encoder ? "yes" : "no",
                   result->mp_expanded_shaping ? "expanded" : "min",
                   result->mp_asymmetric_rates_allowed ? "yes" : "no",
                   rate_mask,
                   result->mp_from_rx_recovery ? " recovered=yes" : "",
                   result->mp_crc_valid ? "ok" : "no",
                   result->mp_fill_valid ? "ok" : "no",
                   result->mp_start_error_count);
        } else {
            if (result->mp_candidate_seen) {
                char cand_rate_mask[160];

                v34_mp_rate_mask_to_text(result->mp_candidate_signalling_rate_mask, cand_rate_mask, sizeof(cand_rate_mask));
                printf("  MP candidate:    type=MP%d bits=%d votes=%d trellis=%s rates=%d/%d mask={%s} crc=%s fill=%s start_err=%d (not accepted)\n",
                       result->mp_candidate_type,
                       result->mp_candidate_frame_bits_captured,
                       result->mp_candidate_votes,
                       v34_mp_trellis_name(result->mp_candidate_trellis_size),
                       result->mp_candidate_rate_a_to_c_bps,
                       result->mp_candidate_rate_c_to_a_bps,
                       cand_rate_mask,
                       result->mp_candidate_crc_valid ? "ok" : "no",
                       result->mp_candidate_fill_valid ? "ok" : "no",
                       result->mp_candidate_start_error_count);
            }
            if (result->mp_from_info_sequence)
                printf("  MP decode:       no accepted RX MP frame; negotiated rates inferred from INFO sequences\n");
            else
                printf("  MP decode:       no accepted RX MP frame decoded; using TX-state negotiated settings\n");
        }
    } else if (expected_rate_1 > 0 && expected_rate_2 > 0) {
        printf("  MP exchange:     seen=no remote_ack=no filename=%d/%d\n",
               expected_rate_1,
               expected_rate_2);
    }
    if (result->training_failed)
        printf("  Training fail:   observed at %.1f ms\n", sample_to_ms(result->failure_sample, 8000));
}

static void collect_v34_events(call_log_t *log,
                               const int16_t *samples,
                               int total_samples,
                               const uint8_t *g711_codewords,
                               int total_codewords,
                               v91_law_t law,
                               bool suppress_v90_phase2,
                               bool suppress_phase12_early_events)
{
    decode_v34_result_t answerer;
    decode_v34_result_t caller;
    char detail[2048];
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
            call_log_append(log, (RES_PTR)->SAMPLE_FIELD, 0, phase3_proto__, SUMMARY_STR, detail); \
        } \
    } while (0)

    v34_phase2_decode_pair(&g_v34_phase2_engine,
                           samples,
                           total_samples,
                           law,
                           !suppress_v90_phase2,
                           &answerer,
                           &have_answerer,
                           &caller,
                           &have_caller);

    if (have_answerer && have_caller) {
        if (v34_phase2_result_spec_score(&answerer) >= v34_phase2_result_spec_score(&caller)) {
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

    p3_flow_standard_t phase3_standard_global = p3_detect_flow_standard(have_answerer ? &answerer : NULL,
                                                                         have_caller ? &caller : NULL);
    if (phase3_standard_global == P3_FLOW_STANDARD_UNKNOWN)
        phase3_standard_global = suppress_v90_phase2 ? P3_FLOW_STANDARD_V34 : P3_FLOW_STANDARD_V90;
    const char *phase3_proto_global = p3_flow_protocol_name(phase3_standard_global);

#define APPEND_V34_RESULT_EVENTS(RES_PTR, ROLE_STR, ALLOW_PHASE3_PLUS, PHASE2_CUTOFF) \
    do { \
        const decode_v34_result_t *res__ = (RES_PTR); \
        const char *role_name = (ROLE_STR); \
        const char *phase3_proto__ = phase3_proto_global; \
        if (!res__) \
            break; \
        if (!(suppress_v90_phase2) && !(suppress_phase12_early_events) && res__->info0_seen && should_emit_phase2_event(res__->info0_sample, (PHASE2_CUTOFF))) { \
            format_v34_info0_table_summary(detail, sizeof(detail), &res__->info0a, &res__->info0_raw, role_name, res__->info0_is_d); \
            if (res__->info0_from_rescue) \
                appendf(detail, sizeof(detail), " source=rescue_pass"); \
            call_log_append(log, res__->info0_sample, 0, "V.34", v34_info0_label(res__), detail); \
        } \
        if (!(suppress_v90_phase2) && !(suppress_phase12_early_events) && res__->info1_seen && should_emit_phase2_event(res__->info1_sample, (PHASE2_CUTOFF))) { \
            if (res__->info1_is_d) { \
                format_v34_info1d_table_summary(detail, sizeof(detail), &res__->info1d, role_name); \
                if (res__->info1_from_rescue) \
                    appendf(detail, sizeof(detail), " source=rescue_pass"); \
                call_log_append(log, res__->info1_sample, 0, "V.34", "INFO1d decoded", detail); \
            } else { \
                format_v34_info1_table15_summary(detail, sizeof(detail), &res__->info1a, &res__->info1a_raw, role_name); \
                if (res__->info1_from_rescue) \
                    appendf(detail, sizeof(detail), " source=rescue_pass"); \
                call_log_append(log, res__->info1_sample, 0, "V.34", "INFO1a decoded", detail); \
            } \
        } \
        if (!(suppress_v90_phase2) && !(suppress_phase12_early_events) && res__->info0_seen && should_emit_phase2_event(res__->info0_sample, (PHASE2_CUTOFF))) { \
            v92_short_phase2_observation_t sp2_obs__; \
            v92_short_phase2_result_t sp2_res__; \
            memset(&sp2_obs__, 0, sizeof(sp2_obs__)); \
            sp2_obs__.info0_seen = res__->info0_seen; \
            sp2_obs__.info0_is_d = res__->info0_is_d; \
            sp2_obs__.short_phase2_requested = v92_short_phase2_req_from_info0_bits(res__->info0_is_d, res__->info0_raw.raw_26_27); \
            sp2_obs__.v92_capable = v92_short_phase2_v92_cap_from_info0_bits(res__->info0_is_d, res__->info0_raw.raw_26_27); \
            sp2_obs__.info0_ack = res__->info0a.acknowledge_info0d; \
            sp2_obs__.info1_seen = res__->info1_seen; \
            sp2_obs__.phase3_seen = res__->phase3_seen; \
            sp2_obs__.training_failed = res__->training_failed; \
            sp2_obs__.info0_sample = res__->info0_sample; \
            sp2_obs__.info1_sample = res__->info1_sample; \
            sp2_obs__.tx_tone_a_sample = res__->tx_tone_a_sample; \
            sp2_obs__.tx_tone_b_sample = res__->tx_tone_b_sample; \
            sp2_obs__.tx_tone_a_reversal_sample = res__->tx_tone_a_reversal_sample; \
            sp2_obs__.tx_tone_b_reversal_sample = res__->tx_tone_b_reversal_sample; \
            sp2_obs__.rx_tone_a_sample = res__->rx_tone_a_sample; \
            sp2_obs__.rx_tone_b_sample = res__->rx_tone_b_sample; \
            sp2_obs__.rx_tone_a_reversal_sample = res__->rx_tone_a_reversal_sample; \
            sp2_obs__.rx_tone_b_reversal_sample = res__->rx_tone_b_reversal_sample; \
            sp2_obs__.tx_info1_sample = res__->tx_info1_sample; \
            if (v92_short_phase2_analyze(&sp2_obs__, &sp2_res__)) { \
                detail[0] = '\0'; \
                appendf(detail, sizeof(detail), "role=%s", role_name); \
                appendf(detail, sizeof(detail), " figure=V92_Fig9"); \
                appendf(detail, sizeof(detail), " shortp2_req=1"); \
                appendf(detail, sizeof(detail), " v92_cap=%u", sp2_res__.v92_capable ? 1U : 0U); \
                appendf(detail, sizeof(detail), " local_modem=%s", v92_short_phase2_local_modem(sp2_res__.sequence)); \
                appendf(detail, sizeof(detail), " sequence=%s", v92_short_phase2_sequence_id(sp2_res__.sequence)); \
                appendf(detail, sizeof(detail), " info0_ms=%.1f", sample_to_ms(res__->info0_sample, 8000)); \
                appendf(detail, sizeof(detail), " tx_tone_ms=%s", sp2_res__.tx_tone_sample >= 0 ? "" : "n/a"); \
                if (sp2_res__.tx_tone_sample >= 0) appendf(detail, sizeof(detail), "%.1f", sample_to_ms(sp2_res__.tx_tone_sample, 8000)); \
                appendf(detail, sizeof(detail), " tx_reversal_ms=%s", sp2_res__.tx_reversal_sample >= 0 ? "" : "n/a"); \
                if (sp2_res__.tx_reversal_sample >= 0) appendf(detail, sizeof(detail), "%.1f", sample_to_ms(sp2_res__.tx_reversal_sample, 8000)); \
                appendf(detail, sizeof(detail), " rx_tone_ms=%s", sp2_res__.rx_tone_sample >= 0 ? "" : "n/a"); \
                if (sp2_res__.rx_tone_sample >= 0) appendf(detail, sizeof(detail), "%.1f", sample_to_ms(sp2_res__.rx_tone_sample, 8000)); \
                appendf(detail, sizeof(detail), " rx_reversal_ms=%s", sp2_res__.rx_reversal_sample >= 0 ? "" : "n/a"); \
                if (sp2_res__.rx_reversal_sample >= 0) appendf(detail, sizeof(detail), "%.1f", sample_to_ms(sp2_res__.rx_reversal_sample, 8000)); \
                appendf(detail, sizeof(detail), " tx_info1_ms=%s", sp2_res__.tx_info1_sample >= 0 ? "" : "n/a"); \
                if (sp2_res__.tx_info1_sample >= 0) appendf(detail, sizeof(detail), "%.1f", sample_to_ms(sp2_res__.tx_info1_sample, 8000)); \
                appendf(detail, sizeof(detail), " info1_seen=%u", res__->info1_seen ? 1U : 0U); \
                appendf(detail, sizeof(detail), " info1_ms=%s", res__->info1_sample >= 0 ? "" : "n/a"); \
                if (res__->info1_sample >= 0) appendf(detail, sizeof(detail), "%.1f", sample_to_ms(res__->info1_sample, 8000)); \
                appendf(detail, sizeof(detail), " c94111=%u", sp2_res__.c94111 ? 1U : 0U); \
                appendf(detail, sizeof(detail), " c94112=%u", sp2_res__.c94112 ? 1U : 0U); \
                appendf(detail, sizeof(detail), " c94113=%u", sp2_res__.c94113 ? 1U : 0U); \
                appendf(detail, sizeof(detail), " c94114=%u", sp2_res__.c94114 ? 1U : 0U); \
                appendf(detail, sizeof(detail), " c94115=%u", sp2_res__.c94115 ? 1U : 0U); \
                appendf(detail, sizeof(detail), " c94121=%u", sp2_res__.c94121 ? 1U : 0U); \
                appendf(detail, sizeof(detail), " c94122=%u", sp2_res__.c94122 ? 1U : 0U); \
                appendf(detail, sizeof(detail), " c94123=%u", sp2_res__.c94123 ? 1U : 0U); \
                appendf(detail, sizeof(detail), " c94211=%u", sp2_res__.c94211 ? 1U : 0U); \
                appendf(detail, sizeof(detail), " c94212=%u", sp2_res__.c94212 ? 1U : 0U); \
                appendf(detail, sizeof(detail), " c94213=%u", sp2_res__.c94213 ? 1U : 0U); \
                appendf(detail, sizeof(detail), " c94214=%u", sp2_res__.c94214 ? 1U : 0U); \
                appendf(detail, sizeof(detail), " c94221=%u", sp2_res__.c94221 ? 1U : 0U); \
                appendf(detail, sizeof(detail), " c94222=%u", sp2_res__.c94222 ? 1U : 0U); \
                appendf(detail, sizeof(detail), " status=%s", sp2_res__.status ? sp2_res__.status : "in_progress"); \
                call_log_append(log, \
                                res__->info0_sample, \
                                0, \
                                "V.90/V.92", \
                                (sp2_res__.sequence == V92_SHORT_PHASE2_SEQUENCE_A) \
                                    ? "Short Phase 2 sequence A (Figure 9)" \
                                    : "Short Phase 2 sequence B (Figure 9)", \
                                detail); \
            } \
        } \
        if (phase3_standard_global == P3_FLOW_STANDARD_V92 || p3_result_indicates_v92(res__)) { \
            collect_v92_phase3_event(log, \
                                     res__, \
                                     role_name, \
                                     -1, \
                                     g711_codewords, \
                                     total_codewords, \
                                     law, \
                                     have_answerer ? &answerer : NULL, \
                                     have_caller ? &caller : NULL); \
        } \
        if (!(ALLOW_PHASE3_PLUS)) \
            break; \
        if (res__->phase3_seen) { \
            if (suppress_v90_phase2) { \
                snprintf(detail, sizeof(detail), "role=%s u_info=n/a source=v8_no_digital_pcm_gate", role_name); \
            } else { \
                snprintf(detail, sizeof(detail), "role=%s u_info=%d source=%s", \
                         role_name, \
                         res__->u_info, \
                         res__->u_info_from_info1a ? "info1a_bits25_31" : "spandsp_fallback"); \
            } \
            append_v34_mp_detail_fields(detail, sizeof(detail), res__); \
            call_log_append(log, res__->phase3_sample, 0, phase3_proto__, "Phase 3 reached", detail); \
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
            call_log_append(log, res__->tx_pp_sample, pp_samples__, phase3_proto__, "Phase 3 TX PP", detail); \
        } \
        if (res__->rx_pp_detect_sample >= 0) { \
            snprintf(detail, sizeof(detail), \
                     "role=%s phase=%d score=%d hold=%d started=%s", \
                     role_name, \
                     res__->rx_pp_phase, \
                     res__->rx_pp_phase_score, \
                     res__->rx_pp_acquire_hits, \
                     res__->rx_pp_started ? "yes" : "no"); \
            call_log_append(log, res__->rx_pp_detect_sample, 0, phase3_proto__, "Phase 3 RX PP detect", detail); \
        } \
        if (res__->rx_pp_complete_sample >= 0) { \
            snprintf(detail, sizeof(detail), "role=%s phase=%d score=%d", role_name, res__->rx_pp_phase, res__->rx_pp_phase_score); \
            call_log_append(log, res__->rx_pp_complete_sample, 0, phase3_proto__, "Phase 3 RX PP lock", detail); \
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
                            phase3_proto__, \
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
            call_log_append(log, res__->tx_ja_sample, 0, phase3_proto__, "Phase 3 TX J/Ja", detail); \
        } \
        { \
            p3_j_log_summary_t jflow__; \
            bool calling_role__ = (strcmp(role_name, "caller") == 0); \
            const char *flow_summary__ = (phase3_standard_global == P3_FLOW_STANDARD_V34) \
                ? "Phase 3 J/J' flow" \
                : ((phase3_standard_global == P3_FLOW_STANDARD_V92) \
                    ? "Phase 3 Ru/Jp flow" \
                    : "Phase 3 Ja/Jd flow"); \
            bool jflow_emitted__ = false; \
            memset(&jflow__, 0, sizeof(jflow__)); \
            if (p3_extract_j_log_summary(samples, total_samples, res__, calling_role__, phase3_standard_global, &jflow__) \
                && jflow__.valid \
                && should_emit_phase2_event(jflow__.event_sample, (PHASE2_CUTOFF))) { \
                const char *j_class__ = (jflow__.j_table_bits == 4) ? "4pt" \
                                      : ((jflow__.j_table_bits == 16) ? "16pt" : "unknown"); \
                snprintf(detail, sizeof(detail), \
                         "role=%s standard=%s class=%s match=%d%% jprime=%s trn_after=%s ru=%s status=%s", \
                         role_name, \
                         p3_flow_standard_name(jflow__.standard), \
                         j_class__, \
                         jflow__.j_table_match_pct, \
                         jflow__.jprime_seen ? "yes" : "no", \
                         jflow__.trn_after_j_seen ? "yes" : "no", \
                         jflow__.ru_seen ? "yes" : "no", \
                         jflow__.role_aligned ? "aligned" : "partial"); \
                if (jflow__.jprime_seen && jflow__.jprime_match_pct > 0) \
                    appendf(detail, sizeof(detail), " jprime_match=%d%%", jflow__.jprime_match_pct); \
                if (jflow__.notes[0]) \
                    appendf(detail, sizeof(detail), " notes=%s", jflow__.notes); \
                call_log_append(log, jflow__.event_sample, 0, phase3_proto__, flow_summary__, detail); \
                jflow_emitted__ = true; \
            } \
            if (!jflow_emitted__ && phase3_standard_global == P3_FLOW_STANDARD_V92) { \
                int flow_sample__ = res__->phase3_sample; \
                int ru_min_score__ = v92_ru_flow_min_score(res__); \
                bool ru_ok__ = res__->ru_window_captured && res__->ru_window_score >= ru_min_score__; \
                bool jprime_hint__ = res__->tx_jdashed_sample >= 0; \
                bool trn_after_hint__ = (res__->tx_trn_sample >= 0 && res__->tx_ja_sample >= res__->tx_trn_sample); \
                if (flow_sample__ < 0) \
                    flow_sample__ = res__->tx_first_s_sample; \
                if (flow_sample__ < 0) \
                    flow_sample__ = res__->tx_trn_sample; \
                if (flow_sample__ >= 0 && should_emit_phase2_event(flow_sample__, (PHASE2_CUTOFF))) { \
                    snprintf(detail, sizeof(detail), \
                             "role=%s standard=V.92 class=unknown match=0%% jprime=%s trn_after=%s ru=%s status=%s ru_window=%d score=%d min_score=%d notes=fallback_from_ru_window", \
                             role_name, \
                             jprime_hint__ ? "yes" : "no", \
                             trn_after_hint__ ? "yes" : "no", \
                             ru_ok__ ? "yes" : "no", \
                             ru_ok__ ? "partial" : "weak", \
                             res__->ru_window_len, \
                             res__->ru_window_score, \
                             ru_min_score__); \
                    call_log_append(log, flow_sample__, 0, phase3_proto__, flow_summary__, detail); \
                } \
            } \
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
            append_v34_mp_detail_fields(detail, sizeof(detail), res__); \
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
    phase12_result_t phase12;
    v8_probe_result_t capability_probe;
    bool have_answerer = false;
    bool have_caller = false;
    bool have_capability_probe = false;
    bool have_phase12 = false;
    bool suppress_v90_phase2 = false;
    bool have_phase12_capability = false;
    int earliest_phase2_sample = -1;

    if (!log || !linear_samples || !g711_codewords)
        return;

    phase12_result_init(&phase12);

    if (do_v8 || do_v34 || do_v90) {
        have_phase12 = phase12_decode(linear_samples,
                                      total_samples,
                                      8000,
                                      0,
                                      &phase12);
        if (have_phase12) {
            phase12_merge_to_call_log(&phase12, log, 8000);
            have_phase12_capability = phase12.cm.detected || phase12.jm.detected
                                      || phase12.info0.detected || phase12.info1.detected;
            if (have_phase12_capability)
                suppress_v90_phase2 = !(phase12.pcm_modem_capable || phase12.v90_capable || phase12.v92_capable);
            if (phase12.info0.detected)
                earliest_phase2_sample = first_non_negative(earliest_phase2_sample, phase12.info0.sample_offset);
            if (phase12.info1.detected)
                earliest_phase2_sample = first_non_negative(earliest_phase2_sample, phase12.info1.sample_offset);
        }
    }

    if (!have_phase12_capability) {
        have_capability_probe = get_cached_v8_channel_probe(linear_samples,
                                                            total_samples,
                                                            total_samples,
                                                            &capability_probe);
        suppress_v90_phase2 = have_capability_probe && !v8_probe_allows_v90_v92_digital(&capability_probe);
    }

    if (do_v34 || do_v90) {
        v34_phase2_decode_pair_cached(&g_v34_phase2_engine,
                                      linear_samples,
                                      total_samples,
                                      law,
                                      !suppress_v90_phase2,
                                      &answerer,
                                      &have_answerer,
                                      &caller,
                                      &have_caller);
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
        collect_v34_events(log,
                           linear_samples,
                           total_samples,
                           g711_codewords,
                           total_codewords,
                           law,
                           suppress_v90_phase2,
                           have_phase12);
    if (do_v8 && !have_phase12) {
        v8bis_collect_signal_events(log, linear_samples, total_samples, earliest_phase2_sample);
        v8bis_collect_msg_events(log, linear_samples, total_samples, earliest_phase2_sample);
        collect_v8_event(log, linear_samples, g711_codewords, total_samples, earliest_phase2_sample, law);
    }
    if (do_v91)
        collect_v91_events(log, g711_codewords, total_codewords, law);
    if (do_v90 && !suppress_v90_phase2)
        collect_v90_events(log, g711_codewords, total_codewords, law);

    /*
     * Post-Phase 3 stage events (Jd/Ja frames, DIL descriptor) are relevant
     * for both V.90 and V.92 calls.  Run whenever we have V.34 decode results,
     * regardless of whether the explicit --v90 flag was set.
     */
    if (!suppress_v90_phase2 && (do_v34 || do_v90) && (have_answerer || have_caller)) {
        collect_post_phase3_stage_events(log,
                                         g711_codewords,
                                         total_codewords,
                                         have_answerer ? &answerer : NULL,
                                         have_caller ? &caller : NULL);
    }

    call_log_sort(log);
    call_log_prune_v8_after_phase2(log);
    call_log_sort(log);
    phase12_result_reset(&phase12);
}

static void call_log_merge_with_channel(call_log_t *dst,
                                        const call_log_t *src,
                                        const char *channel_label)
{
    char detail[4096];

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
    const decode_v34_result_t *src;
    bool is_v92 = false;
    char detail[192];

    if (!log || !codewords || total_codewords <= 0)
        return;

    /*
     * Determine whether this is a V.92 call from INFO0 so we know whether
     * to expect Jd capability frames before the DIL descriptor.
     */
    src = pick_post_phase3_source(answerer, caller, NULL);
    if (src && src->info0_seen)
        is_v92 = v92_short_phase2_v92_cap_from_info0_bits(src->info0_is_d,
                                                          src->info0_raw.raw_26_27);

    memset(&jd_stage, 0, sizeof(jd_stage));

    if (!is_v92) {
        /*
         * V.90: Jd capability frames are mandatory before the DIL.
         * Bail out early if they cannot be found — without them we have no
         * reliable timing anchor for the DIL search.
         */
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

    /*
     * Decode the Ja/DIL descriptor for both V.90 and V.92.
     * For V.90 the jd_stage result above provides the timing anchor.
     * For V.92 decode_ja_dil_stage() falls back to tx_ja_sample.
     */
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
                        is_v92 ? "V.92 Phase 3" : "V.90",
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
    ja_dil_search_params_t params;
    int search_start;
    int search_end;

    if (!codewords || total_codewords <= 0 || !out)
        return false;

    src = pick_post_phase3_source(answerer, caller, &calling_party);
    if (!src)
        return false;

    /*
     * Compute the search window from the V.34 decode context.
     *
     * Prefer the locally decoded Ja anchor when it exists — using a later
     * Jd/J'd alignment as the primary seed can pull the search onto the
     * wrong region.
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

    /*
     * When the Ja onset is known (tx_ja_sample), the DIL descriptor starts
     * within a few hundred symbols of it.  The maximum descriptor bit-length
     * is bounded by the Ja protocol timing (n≤128 U-chords ≈ 2000 bits).
     * Clamp search_end to avoid scanning megabytes of training audio.
     */
    if (src->tx_ja_sample >= 0 && search_end > src->tx_ja_sample + 4096)
        search_end = src->tx_ja_sample + 4096;

    params.search_start  = search_start;
    params.search_end    = search_end;
    params.tx_ja_sample  = src->tx_ja_sample;
    params.u_info        = src->u_info;
    params.calling_party = calling_party;

    return v92_ja_dil_search(codewords, total_codewords, &params, out);
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
    bool do_p3;
    bool do_energy;
    bool do_tone_probe;
    bool do_stats;
    bool do_call_log;
    bool do_phase12;
    bool raw_output_enabled;
    double tone_probe_start_ms;
    double tone_probe_duration_ms;
} decode_options_t;

static void print_tone_probe(const char *label,
                             const int16_t *samples,
                             int total_samples,
                             int sample_rate,
                             double start_ms,
                             double duration_ms)
{
    static const double freqs[] = { 1200.0, 1800.0, 2100.0, 2400.0 };
    int start_sample;
    int window_samples;
    int end_sample;
    double energy;
    double rms_db;
    double ratios[sizeof(freqs) / sizeof(freqs[0])];

    if (!label || !samples || total_samples <= 0 || sample_rate <= 0)
        return;

    if (start_ms < 0.0)
        start_ms = 0.0;
    if (duration_ms <= 0.0)
        return;

    start_sample = (int) llrint(start_ms * (double) sample_rate / 1000.0);
    window_samples = (int) llrint(duration_ms * (double) sample_rate / 1000.0);
    if (window_samples <= 0)
        return;
    if (start_sample >= total_samples)
        return;

    end_sample = start_sample + window_samples;
    if (end_sample > total_samples)
        end_sample = total_samples;
    window_samples = end_sample - start_sample;
    if (window_samples <= 0)
        return;

    energy = window_energy(samples + start_sample, window_samples);
    rms_db = rms_energy_db(samples + start_sample, window_samples);

    printf("\n=== Tone Probe (%s) ===\n", label);
    printf("  Window: %.1f-%.1f ms (%.1f ms, %d samples)\n",
           sample_to_ms(start_sample, sample_rate),
           sample_to_ms(end_sample, sample_rate),
           sample_to_ms(window_samples, sample_rate),
           window_samples);
    printf("  RMS: %.1f dBFS\n", rms_db);

    if (energy <= 0.0) {
        printf("  No signal energy in probe window.\n");
        return;
    }

    for (size_t i = 0; i < sizeof(freqs) / sizeof(freqs[0]); i++) {
        ratios[i] = tone_energy_ratio(samples + start_sample,
                                      window_samples,
                                      sample_rate,
                                      freqs[i],
                                      energy);
        printf("  %4.0f Hz: ratio=%.6f\n", freqs[i], ratios[i]);
    }

    if (ratios[0] > 0.0 && ratios[3] > 0.0) {
        printf("  2400 vs 1200: %.2f dB\n",
               10.0 * log10(ratios[3] / ratios[0]));
    }
    if (ratios[1] > 0.0 && ratios[3] > 0.0) {
        printf("  1800 vs 2400: %.2f dB\n",
               10.0 * log10(ratios[1] / ratios[3]));
    }
    if (ratios[1] > 0.0 && ratios[0] > 0.0) {
        printf("  1800 vs 1200: %.2f dB\n",
               10.0 * log10(ratios[1] / ratios[0]));
    }
}

static void run_decode_suite(const char *label,
                             const int16_t *linear_samples,
                             const uint8_t *g711_codewords,
                             int total_samples,
                             int total_codewords,
                             int sample_rate,
                             v91_law_t law,
                             const decode_options_t *opts,
                             const codeword_stream_info_t *codeword_info,
                             int expected_rate_1,
                             int expected_rate_2)
{
    decode_v34_result_t answerer;
    decode_v34_result_t caller;
    phase12_result_t p12;
    v8_probe_result_t capability_probe;
    bool have_answerer = false;
    bool have_caller = false;
    bool have_capability_probe = false;
    bool have_phase12 = false;
    bool have_phase12_capability = false;
    bool suppress_v90_phase2 = false;

    if (!linear_samples || !g711_codewords || !opts)
        return;

    phase12_result_init(&p12);

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

    if (opts->do_tone_probe) {
        print_tone_probe(label,
                         linear_samples,
                         total_samples,
                         sample_rate,
                         opts->tone_probe_start_ms,
                         opts->tone_probe_duration_ms);
    }

    if (opts->do_phase12 || opts->do_v34 || opts->do_v90 || opts->do_v8) {
        have_phase12 = phase12_decode(linear_samples, total_samples, sample_rate, 0, &p12);
        if (have_phase12)
            have_phase12_capability = p12.cm.detected || p12.jm.detected
                                      || p12.info0.detected || p12.info1.detected;
        if (have_phase12_capability)
            suppress_v90_phase2 = !(p12.pcm_modem_capable || p12.v90_capable || p12.v92_capable);
    }
    if (!have_phase12_capability) {
        have_capability_probe = get_cached_v8_channel_probe(linear_samples,
                                                            total_samples,
                                                            total_samples,
                                                            &capability_probe);
        suppress_v90_phase2 = have_capability_probe && !v8_probe_allows_v90_v92_digital(&capability_probe);
    }
    if (suppress_v90_phase2 && opts->raw_output_enabled && (opts->do_v34 || opts->do_v90)) {
        char modbuf[256];

        if (have_phase12_capability) {
            unsigned mods = 0;

            if (p12.cm.detected)
                mods |= (unsigned) p12.cm.modulations;
            if (p12.jm.detected)
                mods |= (unsigned) p12.jm.modulations;
            format_v8_modulations_str((int) mods, modbuf, sizeof(modbuf));
            printf("Phase 1/2 gate: standalone Phase 1/2 decode does not indicate V.90/V.92 digital capability (mods=%s pcm=%s), so V.90/V.92 decode paths are suppressed.\n",
                   modbuf[0] ? modbuf : "none",
                   p12.pcm_modem_capable ? "available" : "not_available");
        } else {
            format_v8_modulations_str(capability_probe.result.jm_cm.modulations, modbuf, sizeof(modbuf));
            printf("V.8 gate: CM/JM does not advertise V.90/V.92 digital capability (mods=%s pcm=%s), so V.90/V.92 decode paths are suppressed.\n",
                   modbuf[0] ? modbuf : "none",
                   v8_pcm_modem_availability_to_str(capability_probe.result.jm_cm.pcm_modem_availability));
        }
    }

    if ((opts->raw_output_enabled && (opts->do_v34 || opts->do_v90))) {
        v34_phase2_decode_pair_cached(&g_v34_phase2_engine,
                                      linear_samples,
                                      total_samples,
                                      law,
                                      !suppress_v90_phase2,
                                      &answerer,
                                      &have_answerer,
                                      &caller,
                                      &have_caller);
    }

    if (opts->raw_output_enabled && opts->do_v34) {
        printf("\n=== V.34/V.90 Phase 2 Decode (as answerer) ===\n");
        if (have_answerer)
            print_v34_result(&answerer, false, expected_rate_1, expected_rate_2, suppress_v90_phase2);
        else
            printf("  V.34 probe init failed\n");

        printf("\n=== V.34/V.90 Phase 2 Decode (as caller) ===\n");
        if (have_caller)
            print_v34_result(&caller, true, expected_rate_1, expected_rate_2, suppress_v90_phase2);
        else
            printf("  V.34 probe init failed\n");

        /* Lightweight Phase 3 demodulation (supplementary) */
        if (have_answerer && answerer.phase3_seen)
            p3_demod_analyse_phase3(linear_samples, total_samples, &answerer, false);
        if (have_caller && caller.phase3_seen)
            p3_demod_analyse_phase3(linear_samples, total_samples, &caller, true);
    }

    if (opts->raw_output_enabled && opts->do_v8) {
        printf("\n=== V.8 Decode ===\n");
        decode_v8_pass(linear_samples, g711_codewords, total_samples, law);
    }

    if (opts->raw_output_enabled && opts->do_phase12) {
        printf("\n=== Standalone Phase 1/2 Decode (no spandsp) ===\n");
        if (have_phase12) {
            /* Phase 1 tones */
            if (p12.cng.detected)
                printf("  CNG tone at %.1f ms (%.0f ms, ratio %.3f)\n",
                       sample_to_ms(p12.cng.start_sample, sample_rate),
                       sample_to_ms(p12.cng.duration_samples, sample_rate),
                       p12.cng.peak_ratio);
            if (p12.ct.detected)
                printf("  CT tone at %.1f ms (%.0f ms, ratio %.3f)\n",
                       sample_to_ms(p12.ct.start_sample, sample_rate),
                       sample_to_ms(p12.ct.duration_samples, sample_rate),
                       p12.ct.peak_ratio);
            if (p12.answer_tone.detected) {
                const char *tone_names[] = { "?", "CNG", "CT", "ANS", "ANS/PR", "ANSam", "ANSam/PR" };
                int ti = (int)p12.answer_tone.type;
                printf("  %s at %.1f ms (%.0f ms, ratio %.3f)\n",
                       (ti >= 0 && ti < 7) ? tone_names[ti] : "?",
                       sample_to_ms(p12.answer_tone.start_sample, sample_rate),
                       sample_to_ms(p12.answer_tone.duration_samples, sample_rate),
                       p12.answer_tone.peak_ratio);
                printf("  Answer-tone handoff at %.1f ms\n",
                       sample_to_ms(p12.answer_tone.start_sample + p12.answer_tone.duration_samples,
                                    sample_rate));
            }
            if (p12.call_init.v8bis_signal_seen) {
                printf("  Pre-ANS V.8bis: %s%s at %.1f ms (%.0f ms)\n",
                       p12.call_init.v8bis_signal_weak ? "weak " : "",
                       p12.call_init.v8bis_signal_name,
                       sample_to_ms(p12.call_init.v8bis_signal_sample, sample_rate),
                       sample_to_ms(p12.call_init.v8bis_signal_duration, sample_rate));
            }
            if (p12.call_init.v92_qc2_seen) {
                printf("  Call-init QC: %s at %.1f ms\n",
                       p12.call_init.v92_qc2_name,
                       sample_to_ms(p12.call_init.v92_qc2_sample, sample_rate));
            }

            /* V.21 FSK bursts */
            printf("  V.21 CH1 bursts: %d, CH2 bursts: %d\n",
                   p12.ch1_burst_count, p12.ch2_burst_count);

            /* V.8 messages */
            if (p12.cm.detected) {
                printf("  CM at %.1f ms: modulations=0x%04X call_func=%d proto=%d pcm=%d (%d bytes)\n",
                       sample_to_ms(p12.cm.sample_offset, sample_rate),
                       p12.cm.modulations, p12.cm.call_function,
                       p12.cm.protocols, p12.cm.pcm_modem_availability,
                       p12.cm.byte_count);
            }
            if (p12.jm.detected) {
                printf("  JM at %.1f ms: modulations=0x%04X call_func=%d proto=%d pcm=%d (%d bytes)\n",
                       sample_to_ms(p12.jm.sample_offset, sample_rate),
                       p12.jm.modulations, p12.jm.call_function,
                       p12.jm.protocols, p12.jm.pcm_modem_availability,
                       p12.jm.byte_count);
            }
            if (p12.cj.detected)
                printf("  CJ at %.1f ms\n",
                       sample_to_ms(p12.cj.sample_offset, sample_rate));

            /* Role */
            if (p12.role_detected)
                printf("  Detected role: %s\n", p12.is_caller ? "caller" : "answerer");

            /* Phase 2 INFO */
            if (p12.info0.detected)
                printf("  INFO0 at %.1f ms (%.0f ms, kind=%s, bits=%d)\n",
                       sample_to_ms(p12.info0.sample_offset, sample_rate),
                       sample_to_ms(p12.info0.duration_samples, sample_rate),
                       phase12_info0_kind_name(p12.info0.kind),
                       p12.info0.frame.total_bits);
            if (p12.info1.detected)
                printf("  INFO1 at %.1f ms (%.0f ms, kind=%s, bits=%d)\n",
                       sample_to_ms(p12.info1.sample_offset, sample_rate),
                       sample_to_ms(p12.info1.duration_samples, sample_rate),
                       phase12_info1_kind_name(p12.info1.kind),
                       p12.info1.frame.total_bits);

            /* Probing tones */
            for (int pt = 0; pt < p12.probe_tone_count; pt++)
                printf("  Probing tone %.0f Hz at %.1f ms (%.0f ms, ratio %.3f)\n",
                       p12.probe_tones[pt].freq_hz,
                       sample_to_ms(p12.probe_tones[pt].start_sample, sample_rate),
                       sample_to_ms(p12.probe_tones[pt].duration_samples, sample_rate),
                       p12.probe_tones[pt].peak_ratio);

            if (p12.phase2_window_known) {
                printf("  Phase 2 window: %.1f ms to %.1f ms\n",
                       sample_to_ms(p12.phase2_start_sample, sample_rate),
                       sample_to_ms(p12.phase2_end_sample, sample_rate));
            }
            printf("  Phase 1/2 diag: pcm=%s v90=%s v92=%s short_p2=%s digital_side=%s\n",
                   p12.pcm_modem_capable ? "yes" : "no",
                   p12.v90_capable ? "yes" : "no",
                   p12.v92_capable ? "yes" : "no",
                   p12.short_phase2_requested ? "yes" : "no",
                   p12.digital_side_likely ? "yes" : "no");
            if (p12.phase2_state.valid) {
                printf("  Phase 2 state: role=%s next=%s\n",
                       phase12_phase2_role_name(p12.phase2_state.role),
                       phase12_phase2_step_name(p12.phase2_state.next_step));
            }
            if (p12.info_path_known) {
                printf("  INFO handoff: u_info=%d up_code=%d down_code=%d\n",
                       p12.inferred_u_info,
                       p12.inferred_upstream_symbol_rate_code,
                       p12.inferred_downstream_rate_code);
            }
            if (p12.short_phase2_analysis_valid) {
                printf("  Short Phase 2: seq=%s status=%s\n",
                       v92_short_phase2_sequence_id(p12.short_phase2.sequence),
                       p12.short_phase2.status ? p12.short_phase2.status : "unknown");
            }
        } else {
            printf("  No Phase 1/2 signals detected.\n");
        }
    }

    if (opts->raw_output_enabled && opts->do_v91)
        decode_v91_signals(g711_codewords, total_codewords, law);

    if (opts->raw_output_enabled && opts->do_v90 && !suppress_v90_phase2) {
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
    } else if (opts->raw_output_enabled && opts->do_v90 && suppress_v90_phase2) {
        printf("\n=== V.90/V.92 PCM Decode ===\n");
        printf("  Suppressed by V.8 gate: decoded CM/JM does not advertise V.90/V.92 digital PCM capability.\n");
    }

    if (opts->raw_output_enabled && opts->do_p3) {
        int phase3_start = -1;
        int phase3_end = -1;
        int last_phase3_start = -1;
        int last_phase3_end = -1;
        bool scanned_targeted_window = false;

        printf("\n=== Phase 3 Lightweight Demodulator Scan ===\n");

        if (have_answerer
            && p3_demod_get_phase3_window(&answerer, total_samples, &phase3_start, &phase3_end)) {
            p3_demod_scan_window(linear_samples + phase3_start,
                                 phase3_end - phase3_start,
                                 phase3_start,
                                 sample_rate,
                                 "Answerer Phase 3 window");
            scanned_targeted_window = true;
            last_phase3_start = phase3_start;
            last_phase3_end = phase3_end;
        }

        if (have_caller
            && p3_demod_get_phase3_window(&caller, total_samples, &phase3_start, &phase3_end)) {
            if (!scanned_targeted_window
                || phase3_start != last_phase3_start
                || phase3_end != last_phase3_end) {
                p3_demod_scan_window(linear_samples + phase3_start,
                                     phase3_end - phase3_start,
                                     phase3_start,
                                     sample_rate,
                                     "Caller Phase 3 window");
                scanned_targeted_window = true;
            }
        }

        if (!scanned_targeted_window) {
            printf("  Phase 3 boundaries unavailable from V.34 decode; falling back to full-stream scan.\n");
            p3_demod_scan_window(linear_samples,
                                 total_samples,
                                 0,
                                 sample_rate,
                                 "Full stream fallback");
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

    phase12_result_reset(&p12);
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
    bool do_tone_probe = false;
    bool do_stats = false;
    bool do_call_log = false;
    bool do_all = false;
    bool do_visualize_html = false;
    bool do_p3 = false;
    bool do_phase12 = false;
    double tone_probe_start_ms = 0.0;
    double tone_probe_duration_ms = 0.0;

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
        } else if (strcmp(argv[i], "--p3") == 0) {
            do_p3 = true;
            explicit_decode_output = true;
        } else if (strcmp(argv[i], "--energy") == 0) {
            do_energy = true;
            explicit_decode_output = true;
        } else if (strcmp(argv[i], "--tone-probe") == 0 && i + 2 < argc) {
            char *end1 = NULL;
            char *end2 = NULL;

            tone_probe_start_ms = strtod(argv[++i], &end1);
            tone_probe_duration_ms = strtod(argv[++i], &end2);
            if (!end1 || *end1 != '\0' || !end2 || *end2 != '\0'
                || tone_probe_start_ms < 0.0 || tone_probe_duration_ms <= 0.0) {
                fprintf(stderr, "Invalid --tone-probe arguments. Use: --tone-probe <start_ms> <duration_ms>\n");
                return 1;
            }
            do_tone_probe = true;
            explicit_decode_output = true;
        } else if (strcmp(argv[i], "--stats") == 0) {
            do_stats = true;
            explicit_decode_output = true;
        } else if (strcmp(argv[i], "--call-log") == 0) {
            do_call_log = true;
        } else if (strcmp(argv[i], "--visualize-html") == 0 && i + 1 < argc) {
            do_visualize_html = true;
            visualize_html_path = argv[++i];
        } else if (strcmp(argv[i], "--phase12") == 0) {
            do_phase12 = true;
            explicit_decode_output = true;
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
                   "  --p3               Lightweight Phase 3 demodulator scan\n"
                   "  --phase12          Standalone Phase 1/2 decoder (no spandsp)\n"
                   "  --energy           Print RMS energy profile\n"
                   "  --tone-probe a b   Print tone ratios in window a..a+b ms\n"
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

    if (!do_v34 && !do_v8 && !do_v91 && !do_v90 && !do_p3 && !do_energy && !do_tone_probe && !do_stats && !do_call_log && !do_visualize_html && !do_phase12)
        do_all = true;

    if (do_all) {
        do_v34 = do_v8 = do_v91 = do_v90 = do_p3 = do_energy = do_stats = do_call_log = true;
    }

    if (do_visualize_html)
        do_call_log = true;

    if (do_call_log) {
        if (!do_v34 && !do_v8 && !do_v91 && !do_v90) {
            do_v34 = true;
            do_v91 = true;
            do_v90 = true;
            /* V.8 helps establish caller/answerer orientation; include it
               when we are auto-selecting all decoders. */
            do_v8 = true;
        }
        /* When the user has explicitly listed decoders, respect that choice.
           V.34 orientation (caller/answerer) is inferred from the V.34 decode
           itself, so V.8 is not strictly required. */
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
        if (wav.channels >= 2 && !channel_explicit && (do_call_log || do_v34 || do_v8 || do_phase12)) {
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
               (left_linear_samples && right_linear_samples)
                   ? "Stereo auto (L+R)"
                   : (channel == CH_LEFT ? "Left" : channel == CH_RIGHT ? "Right" : "Mono"),
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

    {
        int expected_rate_1 = -1;
        int expected_rate_2 = -1;

        printf("File: %s\n", input_path);
        printf("Duration: %.3f seconds (%d samples)\n\n",
               (double) total_samples / (double) sample_rate, total_samples);
        if (parse_filename_rate_pair(input_path, &expected_rate_1, &expected_rate_2)) {
            printf("Filename expected negotiated rates: %d / %d bps\n\n",
                   expected_rate_1,
                   expected_rate_2);
        }

        /* Run decoders */
        decode_options_t opts;
        opts.do_v34 = do_v34;
        opts.do_v8 = do_v8;
        opts.do_v91 = do_v91;
        opts.do_v90 = do_v90;
        opts.do_p3 = do_p3;
        opts.do_energy = do_energy;
        opts.do_tone_probe = do_tone_probe;
        opts.do_stats = do_stats;
        opts.do_call_log = do_call_log;
        opts.do_phase12 = do_phase12;
        opts.raw_output_enabled = explicit_decode_output || !do_call_log;
        opts.tone_probe_start_ms = tone_probe_start_ms;
        opts.tone_probe_duration_ms = tone_probe_duration_ms;

        if (left_linear_samples && right_linear_samples && left_g711_codewords && right_g711_codewords) {
            if (opts.raw_output_enabled && opts.do_v8) {
                v8_stereo_pair_result_t stereo_pair;

                if (v8_select_best_stereo_pair(left_linear_samples,
                                               right_linear_samples,
                                               total_samples,
                                               total_samples,
                                               &stereo_pair)) {
                    print_v8_stereo_pair_summary(&stereo_pair);
                }
            }
            run_decode_suite("Left", left_linear_samples, left_g711_codewords,
                             total_samples, total_codewords, sample_rate, law, &opts, &left_codeword_info,
                             expected_rate_1, expected_rate_2);
            run_decode_suite("Right", right_linear_samples, right_g711_codewords,
                             total_samples, total_codewords, sample_rate, law, &opts, &right_codeword_info,
                             expected_rate_1, expected_rate_2);
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
                             total_samples, total_codewords, sample_rate, law, &opts, &codeword_info,
                             expected_rate_1, expected_rate_2);
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
