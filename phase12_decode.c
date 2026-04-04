/*
 * phase12_decode.c — Standalone V.PCM Phase 1 and Phase 2 offline decoder
 *
 * Decodes V.8 negotiation (Phase 1) and V.34 probing/INFO exchange (Phase 2)
 * signals using custom Goertzel-based tone detection and V.21 FSK
 * demodulation. No spandsp dependency for signal detection.
 *
 * Produces call_log_event_t entries compatible with the HTML visualization.
 */

#include "phase12_decode.h"
#include "v21_fsk_demod.h"
#include "v34_info_decode.h"
#include "v90.h"
#include "v92_short_phase2_decode.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* ------------------------------------------------------------------ */
/* DSP helpers (shared with v8bis_decode via v8bis_decode.h)           */
/* ------------------------------------------------------------------ */

/* These are defined in vpcm_decode.c and declared in v8bis_decode.h */
extern double window_energy(const int16_t *samples, int len);
extern double tone_energy_ratio(const int16_t *samples, int len,
                                int sample_rate, double freq_hz,
                                double total_energy);

/* ------------------------------------------------------------------ */
/* Constants                                                           */
/* ------------------------------------------------------------------ */

enum {
    TONE_WINDOW_SAMPLES = 160,   /* 20 ms at 8000 Hz */
    TONE_STEP_SAMPLES = 80,     /* 10 ms step */
    MIN_CNG_RUN_MS = 350,       /* minimum CNG burst duration */
    MIN_CT_RUN_MS = 300,        /* minimum CT duration */
    MIN_ANS_RUN_MS = 500,       /* minimum ANS/ANSam duration */
    ANS_FREQ_HZ = 2100,
    CNG_FREQ_HZ = 1100,
    CT_FREQ_HZ = 1300,
    AM_CHECK_BLOCK = 80,        /* samples per AM check block */
    FSK_BURST_WINDOW = 160,
    FSK_BURST_STEP = 80,
    FSK_BURST_MIN_WINDOWS = 2,
    FSK_BURST_MERGE_GAP = 240,
    CJ_MIN_MARK_BITS = 3,      /* minimum all-marks bits for CJ */
    PHASE2_MAX_SCAN_MS = 15000  /* max ms to scan for Phase 2 signals */
};

/* Tone detection thresholds */
static const double TONE_RATIO_THRESHOLD = 0.15;
static const double TONE_COMPETITOR_MAX = 0.08;
static const double FSK_BURST_THRESHOLD = 0.12;

static int p12_first_non_negative(int a, int b)
{
    if (a >= 0 && b >= 0)
        return (a < b) ? a : b;
    if (a >= 0)
        return a;
    return b;
}

/* ------------------------------------------------------------------ */
/* Tone detection                                                      */
/* ------------------------------------------------------------------ */


/*
 * Detect a continuous or cadenced tone using Goertzel energy.
 * Returns the first run that meets the minimum duration.
 */
static bool detect_tone(const int16_t *samples,
                        int total_samples,
                        int sample_rate,
                        double freq_hz,
                        const double *competitor_freqs,
                        int n_competitors,
                        int min_run_ms,
                        bool require_cadence,
                        p12_tone_hit_t *out)
{
    int min_run_windows = (min_run_ms * sample_rate) / (1000 * TONE_STEP_SAMPLES);
    int run_start = -1;
    int run_windows = 0;
    double run_peak = 0.0;

    if (!samples || total_samples <= 0 || !out)
        return false;

    memset(out, 0, sizeof(*out));

    if (min_run_windows < 1)
        min_run_windows = 1;

    for (int pos = 0; pos + TONE_WINDOW_SAMPLES <= total_samples; pos += TONE_STEP_SAMPLES) {
        double energy = window_energy(samples + pos, TONE_WINDOW_SAMPLES);
        double ratio;
        bool competitor_reject = false;

        if (energy <= 0.0)
            goto tone_gap;

        ratio = tone_energy_ratio(samples + pos, TONE_WINDOW_SAMPLES,
                                  sample_rate, freq_hz, energy);

        if (ratio < TONE_RATIO_THRESHOLD)
            goto tone_gap;

        /* Check competitor frequencies */
        for (int c = 0; c < n_competitors; c++) {
            double c_ratio = tone_energy_ratio(samples + pos, TONE_WINDOW_SAMPLES,
                                               sample_rate, competitor_freqs[c], energy);
            if (c_ratio > ratio * 0.6 && c_ratio > TONE_COMPETITOR_MAX) {
                competitor_reject = true;
                break;
            }
        }
        if (competitor_reject)
            goto tone_gap;

        if (run_start < 0)
            run_start = pos;
        run_windows++;
        if (ratio > run_peak)
            run_peak = ratio;
        continue;

tone_gap:
        if (run_start >= 0 && run_windows >= min_run_windows) {
            out->detected = true;
            out->start_sample = run_start;
            out->duration_samples = (run_windows - 1) * TONE_STEP_SAMPLES + TONE_WINDOW_SAMPLES;
            out->peak_ratio = run_peak;
            return true;
        }
        run_start = -1;
        run_windows = 0;
        run_peak = 0.0;
    }

    /* Check trailing run */
    if (run_start >= 0 && run_windows >= min_run_windows) {
        out->detected = true;
        out->start_sample = run_start;
        out->duration_samples = (run_windows - 1) * TONE_STEP_SAMPLES + TONE_WINDOW_SAMPLES;
        out->peak_ratio = run_peak;
        return true;
    }

    (void)require_cadence;
    return false;
}

/*
 * Detect ANSam 15 Hz amplitude modulation on a 2100 Hz carrier.
 */
static bool detect_am_15hz(const int16_t *samples,
                           int len,
                           int sample_rate)
{
    /* Envelope detection: compute RMS in short blocks, then look for
       15 Hz periodicity in the envelope using Goertzel. */
    int n_blocks = len / AM_CHECK_BLOCK;
    double *envelope;
    double env_energy = 0.0;
    double am_ratio;
    static const double am_competitors[] = { 10.0, 12.0, 18.0, 20.0, 23.0 };
    double competitor_peak = 0.0;

    if (n_blocks < 8)
        return false;

    envelope = malloc((size_t)n_blocks * sizeof(double));
    if (!envelope)
        return false;

    for (int i = 0; i < n_blocks; i++) {
        double sum = 0.0;
        int base = i * AM_CHECK_BLOCK;

        for (int j = 0; j < AM_CHECK_BLOCK && base + j < len; j++) {
            double s = (double)samples[base + j];
            sum += s * s;
        }
        envelope[i] = sqrt(sum / (double)AM_CHECK_BLOCK);
        env_energy += envelope[i] * envelope[i];
    }

    if (env_energy <= 0.0) {
        free(envelope);
        return false;
    }

    /* Goertzel at 15 Hz on the envelope signal */
    {
        double w = 2.0 * M_PI * 15.0 / ((double)sample_rate / (double)AM_CHECK_BLOCK);
        double cos_w = cos(w), sin_w = sin(w);
        double osc_re = 1.0, osc_im = 0.0;
        double re = 0.0, im = 0.0;

        for (int i = 0; i < n_blocks; i++) {
            double s = envelope[i];
            double next_re = osc_re * cos_w - osc_im * sin_w;
            double next_im = osc_im * cos_w + osc_re * sin_w;
            re += s * osc_re;
            im -= s * osc_im;
            osc_re = next_re;
            osc_im = next_im;
        }
        am_ratio = (re * re + im * im) / (env_energy * (double)n_blocks);
    }

    /* Check competitor AM frequencies */
    for (int c = 0; c < (int)(sizeof(am_competitors) / sizeof(am_competitors[0])); c++) {
        double w = 2.0 * M_PI * am_competitors[c] / ((double)sample_rate / (double)AM_CHECK_BLOCK);
        double cos_w = cos(w), sin_w = sin(w);
        double osc_re = 1.0, osc_im = 0.0;
        double re = 0.0, im = 0.0;

        for (int i = 0; i < n_blocks; i++) {
            double s = envelope[i];
            double next_re = osc_re * cos_w - osc_im * sin_w;
            double next_im = osc_im * cos_w + osc_re * sin_w;
            re += s * osc_re;
            im -= s * osc_im;
            osc_re = next_re;
            osc_im = next_im;
        }
        double c_ratio = (re * re + im * im) / (env_energy * (double)n_blocks);
        if (c_ratio > competitor_peak)
            competitor_peak = c_ratio;
    }

    free(envelope);
    return (am_ratio > 0.03 && am_ratio > competitor_peak * 1.8);
}

/*
 * Detect 180-degree phase reversals in a 2100 Hz carrier.
 * Uses complex Goertzel output to track carrier phase across windows.
 */
static bool detect_phase_reversals(const int16_t *samples,
                                   int len,
                                   int sample_rate,
                                   int *reversal_count_out)
{
    enum { PHASE_WINDOW = 160 };
    int n_windows = len / PHASE_WINDOW;
    double prev_phase = 0.0;
    bool have_prev = false;
    int reversals = 0;

    for (int w = 0; w < n_windows; w++) {
        goertzel_result_t r = goertzel_analyze(samples + w * PHASE_WINDOW,
                                               PHASE_WINDOW,
                                               sample_rate, 2100.0);
        double phase;

        if (r.energy < 1e6)
            continue;

        phase = atan2(r.im, r.re);
        if (have_prev) {
            double diff = fabs(phase - prev_phase);
            /* Normalize to [0, pi] */
            if (diff > M_PI)
                diff = 2.0 * M_PI - diff;
            /* A phase reversal is close to pi */
            if (diff > 2.5 && diff < 3.8)
                reversals++;
        }
        prev_phase = phase;
        have_prev = true;
    }

    if (reversal_count_out)
        *reversal_count_out = reversals;

    /* ANS_PR has reversals every 450ms, so in a 3-second tone
       we expect ~6 reversals. Be lenient for partial captures. */
    return (reversals >= 2);
}

static void detect_phase1_tones(const int16_t *samples,
                                int total_samples,
                                int sample_rate,
                                int max_sample,
                                phase12_result_t *result)
{
    int limit = (max_sample > 0 && max_sample < total_samples) ? max_sample : total_samples;
    static const double cng_competitors[] = { 980.0, 1180.0, 1300.0 };
    static const double ct_competitors[] = { 1100.0, 1180.0, 980.0 };
    static const double ans_competitors[] = { 1800.0, 2400.0, 1650.0, 1850.0 };

    /* CNG: 1100 Hz */
    if (detect_tone(samples, limit, sample_rate, CNG_FREQ_HZ,
                    cng_competitors, 3, MIN_CNG_RUN_MS, true, &result->cng)) {
        result->cng.type = P12_TONE_CNG;
    }

    /* CT: 1300 Hz */
    if (detect_tone(samples, limit, sample_rate, CT_FREQ_HZ,
                    ct_competitors, 3, MIN_CT_RUN_MS, false, &result->ct)) {
        result->ct.type = P12_TONE_CT;
    }

    /* ANS/ANSam: 2100 Hz */
    if (detect_tone(samples, limit, sample_rate, ANS_FREQ_HZ,
                    ans_competitors, 4, MIN_ANS_RUN_MS, false, &result->answer_tone)) {
        /* Classify: check for AM modulation and phase reversals */
        int ans_start = result->answer_tone.start_sample;
        int ans_len = result->answer_tone.duration_samples;
        int reversal_count = 0;
        bool has_am = detect_am_15hz(samples + ans_start, ans_len, sample_rate);
        bool has_pr = detect_phase_reversals(samples + ans_start, ans_len,
                                             sample_rate, &reversal_count);

        if (has_am && has_pr)
            result->answer_tone.type = P12_TONE_ANSAM_PR;
        else if (has_am)
            result->answer_tone.type = P12_TONE_ANSAM;
        else if (has_pr)
            result->answer_tone.type = P12_TONE_ANS_PR;
        else
            result->answer_tone.type = P12_TONE_ANS;
    }
}

/* ------------------------------------------------------------------ */
/* V.21 FSK burst detection (Goertzel energy scan)                     */
/* ------------------------------------------------------------------ */

static int detect_fsk_bursts(const int16_t *samples,
                             int total_samples,
                             int sample_rate,
                             int search_start,
                             int search_end,
                             int channel,
                             p12_fsk_burst_t *bursts,
                             int max_bursts)
{
    double f0, f1;
    int run_start = -1, run_windows = 0;
    double run_peak = 0.0;
    int count = 0;

    if (!samples || !bursts || max_bursts <= 0)
        return 0;
    if (search_start < 0) search_start = 0;
    if (search_end <= 0 || search_end > total_samples) search_end = total_samples;

    if (channel == V21_CH2) {
        f0 = 1650.0; f1 = 1850.0;
    } else {
        f0 = 980.0; f1 = 1180.0;
    }

    memset(bursts, 0, (size_t)max_bursts * sizeof(*bursts));

    for (int pos = search_start; pos + FSK_BURST_WINDOW <= search_end; pos += FSK_BURST_STEP) {
        double energy = window_energy(samples + pos, FSK_BURST_WINDOW);
        double p0, p1, strength;

        if (energy <= 0.0)
            goto fsk_gap;

        p0 = tone_energy_ratio(samples + pos, FSK_BURST_WINDOW, sample_rate, f0, energy);
        p1 = tone_energy_ratio(samples + pos, FSK_BURST_WINDOW, sample_rate, f1, energy);
        strength = p0 + p1;

        if (strength >= FSK_BURST_THRESHOLD) {
            if (run_start < 0) {
                run_start = pos;
                run_windows = 0;
                run_peak = 0.0;
            }
            run_windows++;
            if (strength > run_peak)
                run_peak = strength;
            continue;
        }

fsk_gap:
        if (run_start >= 0 && run_windows >= FSK_BURST_MIN_WINDOWS) {
            p12_fsk_burst_t candidate = {0};
            candidate.seen = true;
            candidate.start_sample = run_start;
            candidate.duration_samples = (run_windows - 1) * FSK_BURST_STEP + FSK_BURST_WINDOW;
            candidate.peak_energy = run_peak;

            /* Merge with previous if close */
            if (count > 0
                && candidate.start_sample <= bursts[count - 1].start_sample
                                           + bursts[count - 1].duration_samples
                                           + FSK_BURST_MERGE_GAP) {
                int prev_end = bursts[count - 1].start_sample + bursts[count - 1].duration_samples;
                int new_end = candidate.start_sample + candidate.duration_samples;
                if (new_end > prev_end)
                    bursts[count - 1].duration_samples = new_end - bursts[count - 1].start_sample;
                if (candidate.peak_energy > bursts[count - 1].peak_energy)
                    bursts[count - 1].peak_energy = candidate.peak_energy;
            } else if (count < max_bursts) {
                bursts[count++] = candidate;
            }
        }
        run_start = -1;
        run_windows = 0;
        run_peak = 0.0;
    }

    /* Flush trailing run */
    if (run_start >= 0 && run_windows >= FSK_BURST_MIN_WINDOWS && count < max_bursts) {
        bursts[count].seen = true;
        bursts[count].start_sample = run_start;
        bursts[count].duration_samples = (run_windows - 1) * FSK_BURST_STEP + FSK_BURST_WINDOW;
        bursts[count].peak_energy = run_peak;
        count++;
    }

    return count;
}

/* ------------------------------------------------------------------ */
/* V.21 FSK bit-level HDLC / V.8 message decoding                     */
/* ------------------------------------------------------------------ */

typedef struct {
    /* Async byte accumulator */
    uint32_t bit_stream;
    int bit_count;
    uint8_t bytes[64];
    int byte_count;
    int zero_byte_seen;

    /* Sync detection */
    bool sync_seen;
    int sync_type;   /* 0=unknown, 1=CI, 2=CM/JM */

    /* Results */
    bool message_complete;
    int message_sample_offset;

    /* Raw bit recording */
    int raw_bit_count;
    uint8_t raw_bits[16384];
    int first_bit_sample;
} v8_msg_decoder_t;

enum {
    V8_SYNC_NONE = 0,
    V8_SYNC_CI = 1,
    V8_SYNC_CM_JM = 2
};

static void v8_msg_decoder_init(v8_msg_decoder_t *d)
{
    memset(d, 0, sizeof(*d));
    d->first_bit_sample = -1;
}

static void v8_msg_decoder_put_bit(void *ctx, int bit, int sample_offset)
{
    v8_msg_decoder_t *d = (v8_msg_decoder_t *)ctx;

    if (!d)
        return;

    if (d->first_bit_sample < 0)
        d->first_bit_sample = sample_offset;

    /* Record raw bits */
    if (d->raw_bit_count < (int)sizeof(d->raw_bits))
        d->raw_bits[d->raw_bit_count++] = (uint8_t)(bit ? 1 : 0);

    /* Shift into bit_stream for sync detection */
    d->bit_stream = (d->bit_stream << 1) | (bit ? 1U : 0U);
    d->bit_count++;

    /* Look for V.8 preamble patterns */
    if (!d->sync_seen) {
        /* CI preamble: 10 bits of alternating 0/1 pattern followed by sync */
        /* CM/JM preamble: 0xE0 start byte (async: 0-11100000-1) */
        /* The key sync pattern is a series of flags (0x7E) for HDLC or
           the CM/JM start octet 0xE0 */

        /* Check for CM/JM: look for async 0xE0 byte = 0_00000111_1 (LSB first) */
        if ((d->bit_stream & 0x3FF) == 0x30F) {
            /* 10-bit async frame: start(0) + 0xE0 LSB-first + stop(1) */
            d->sync_seen = true;
            d->sync_type = V8_SYNC_CM_JM;
            d->message_sample_offset = sample_offset;
            d->byte_count = 0;
        }
    }

    if (d->sync_seen && d->sync_type == V8_SYNC_CM_JM) {
        /* Decode subsequent async bytes: start(0) + 8 data bits + stop(1) */
        /* We already found the 0xE0 start; now accumulate data bytes */
        /* Simple approach: every 10 bits after sync, try to extract a byte */
        uint32_t window = d->bit_stream & 0x3FF;
        if (d->bit_count >= 10
            && (window & 0x001) == 0   /* start bit */
            && (window & 0x200) != 0)  /* stop bit */
        {
            uint8_t byte_val = (uint8_t)((window >> 1) & 0xFF);

            if (d->byte_count < (int)sizeof(d->bytes)) {
                d->bytes[d->byte_count++] = byte_val;
            }
            if (byte_val == 0) {
                d->message_complete = true;
                d->zero_byte_seen = 1;
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/* V.8 CM/JM capability parsing (standalone, no spandsp types)         */
/* ------------------------------------------------------------------ */

/* V.8 tag definitions (match spandsp V8_MOD_* values for compatibility) */
enum {
    P12_V8_TAG_CALL_FUNCTION = 0x01,
    P12_V8_TAG_MODULATION = 0x05,
    P12_V8_TAG_PROTOCOLS = 0x0A,
    P12_V8_TAG_PSTN_ACCESS = 0x0D,
    P12_V8_TAG_NSF = 0x0F,
    P12_V8_TAG_PCM_MODEM = 0x07,
    P12_V8_TAG_T66 = 0x0E
};

/* Modulation bit flags (compatible with V8_MOD_* from spandsp) */
enum {
    P12_MOD_V17 = 0x0001,
    P12_MOD_V21 = 0x0002,
    P12_MOD_V22 = 0x0004,
    P12_MOD_V23HDX = 0x0008,
    P12_MOD_V23 = 0x0010,
    P12_MOD_V26BIS = 0x0020,
    P12_MOD_V26TER = 0x0040,
    P12_MOD_V27TER = 0x0080,
    P12_MOD_V29 = 0x0100,
    P12_MOD_V32 = 0x0200,
    P12_MOD_V34HDX = 0x0400,
    P12_MOD_V34 = 0x0800,
    P12_MOD_V90 = 0x1000,
    P12_MOD_V92 = 0x2000
};

static bool parse_cm_jm_bytes(const uint8_t *data, int len,
                               p12_cm_jm_hit_t *out)
{
    const uint8_t *p = data;
    const uint8_t *end = data + len;
    int modulations = 0;

    if (!data || len <= 0 || !out)
        return false;

    while (p < end && *p) {
        switch (*p & 0x1F) {
        case P12_V8_TAG_CALL_FUNCTION:
            out->call_function = (*p >> 5) & 0x07;
            p++;
            break;
        case P12_V8_TAG_MODULATION:
            if (*p & 0x80) modulations |= P12_MOD_V34HDX;
            if (*p & 0x40) modulations |= P12_MOD_V34;
            if (*p & 0x20) modulations |= P12_MOD_V90;
            p++;
            if (p < end && (*p & 0x38) == 0x10) {
                if (*p & 0x80) modulations |= P12_MOD_V27TER;
                if (*p & 0x40) modulations |= P12_MOD_V29;
                if (*p & 0x04) modulations |= P12_MOD_V17;
                if (*p & 0x02) modulations |= P12_MOD_V22;
                if (*p & 0x01) modulations |= P12_MOD_V32;
                p++;
                if (p < end && (*p & 0x38) == 0x10) {
                    if (*p & 0x80) modulations |= P12_MOD_V21;
                    if (*p & 0x40) modulations |= P12_MOD_V23HDX;
                    if (*p & 0x04) modulations |= P12_MOD_V23;
                    if (*p & 0x02) modulations |= P12_MOD_V26BIS;
                    if (*p & 0x01) modulations |= P12_MOD_V26TER;
                    p++;
                }
            }
            out->modulations = modulations;
            break;
        case P12_V8_TAG_PROTOCOLS:
            out->protocols = (*p >> 5) & 0x07;
            p++;
            break;
        case P12_V8_TAG_PSTN_ACCESS:
            out->pstn_access = (*p >> 5) & 0x07;
            p++;
            break;
        case P12_V8_TAG_PCM_MODEM:
            out->pcm_modem_availability = (*p >> 5) & 0x07;
            p++;
            break;
        case P12_V8_TAG_NSF:
            p++;
            while (p < end && (*p & 0x38) == 0x10)
                p++;
            break;
        default:
            p++;
            break;
        }
    }

    return (modulations != 0 || out->call_function != 0);
}

/* ------------------------------------------------------------------ */
/* Phase 1 V.8 message decode                                         */
/* ------------------------------------------------------------------ */

/*
 * Decode V.8 messages from V.21 FSK bursts in a single channel.
 * Uses the block-mode demodulator for each burst, then looks for
 * CM/JM byte patterns in the demodulated bits.
 */
static void decode_v8_fsk_channel(const int16_t *samples,
                                  int total_samples,
                                  int sample_rate,
                                  int channel,
                                  const p12_fsk_burst_t *bursts,
                                  int burst_count,
                                  p12_cm_jm_hit_t *cm_jm_out,
                                  p12_cj_hit_t *cj_out)
{
    for (int b = 0; b < burst_count; b++) {
        v8_msg_decoder_t decoder;
        int start, len;

        if (!bursts[b].seen)
            continue;

        start = bursts[b].start_sample;
        len = bursts[b].duration_samples;
        if (start < 0 || start + len > total_samples)
            continue;

        /* Try both polarities */
        for (int inv = 0; inv < 2; inv++) {
            v8_msg_decoder_init(&decoder);

            v21_fsk_demod_block(samples + start, len, sample_rate,
                                channel, (bool)inv,
                                v8_msg_decoder_put_bit, &decoder);

            /* Check for CJ: sustained all-marks */
            if (cj_out && !cj_out->detected && decoder.raw_bit_count >= CJ_MIN_MARK_BITS) {
                int mark_run = 0, max_mark_run = 0;
                for (int i = 0; i < decoder.raw_bit_count; i++) {
                    if (decoder.raw_bits[i])
                        mark_run++;
                    else {
                        if (mark_run > max_mark_run) max_mark_run = mark_run;
                        mark_run = 0;
                    }
                }
                if (mark_run > max_mark_run) max_mark_run = mark_run;
                /* CJ is all marks for >= 10ms (3 bits at 300 baud) */
                if (max_mark_run >= CJ_MIN_MARK_BITS
                    && max_mark_run > decoder.raw_bit_count * 0.8) {
                    cj_out->detected = true;
                    cj_out->sample_offset = start;
                }
            }

            /* Try extracting CM/JM bytes from raw bits using async framing */
            if (cm_jm_out && !cm_jm_out->detected && decoder.raw_bit_count >= 20) {
                /* Scan raw bits for async-framed CM/JM messages */
                for (int bit_pos = 0; bit_pos + 10 <= decoder.raw_bit_count; bit_pos++) {
                    uint8_t msg_bytes[64];
                    int msg_len = 0;
                    int scan_pos = bit_pos;

                    /* Look for 0xE0 start byte (async: 0-00000111-1 LSB-first) */
                    if (decoder.raw_bits[scan_pos] != 0)  /* start bit must be 0 */
                        continue;

                    /* Extract first byte */
                    if (scan_pos + 10 > decoder.raw_bit_count)
                        continue;

                    uint8_t first = 0;
                    for (int j = 0; j < 8; j++)
                        first |= (decoder.raw_bits[scan_pos + 1 + j] << j);

                    if (first != 0xE0)
                        continue;

                    /* Found 0xE0 preamble; extract subsequent bytes */
                    scan_pos += 10;
                    while (scan_pos + 10 <= decoder.raw_bit_count && msg_len < 64) {
                        if (decoder.raw_bits[scan_pos] != 0)
                            break;  /* no valid start bit */

                        uint8_t byte_val = 0;
                        for (int j = 0; j < 8; j++)
                            byte_val |= (decoder.raw_bits[scan_pos + 1 + j] << j);

                        msg_bytes[msg_len++] = byte_val;
                        scan_pos += 10;
                        if (byte_val == 0)
                            break;
                    }

                    if (msg_len > 0) {
                        p12_cm_jm_hit_t candidate;
                        memset(&candidate, 0, sizeof(candidate));
                        candidate.byte_count = msg_len;
                        memcpy(candidate.bytes, msg_bytes, (size_t)msg_len);

                        if (parse_cm_jm_bytes(msg_bytes, msg_len, &candidate)) {
                            candidate.detected = true;
                            candidate.complete = true;
                            candidate.sample_offset = start;
                            candidate.duration_samples = len;
                            *cm_jm_out = candidate;
                            break;
                        }
                    }
                }
            }

            if ((cm_jm_out && cm_jm_out->detected) || (cj_out && cj_out->detected))
                break; /* found what we need */
        }
    }
}

static void detect_phase1_v8(const int16_t *samples,
                             int total_samples,
                             int sample_rate,
                             int max_sample,
                             phase12_result_t *result)
{
    int limit = (max_sample > 0 && max_sample < total_samples) ? max_sample : total_samples;

    /* Find V.21 FSK bursts on both channels */
    result->ch1_burst_count = detect_fsk_bursts(samples, total_samples, sample_rate,
                                                 0, limit, V21_CH1,
                                                 result->ch1_bursts, P12_MAX_FSK_BURSTS);
    result->ch2_burst_count = detect_fsk_bursts(samples, total_samples, sample_rate,
                                                 0, limit, V21_CH2,
                                                 result->ch2_bursts, P12_MAX_FSK_BURSTS);

    /* Decode V.8 messages from CH1 bursts (CM from caller) */
    decode_v8_fsk_channel(samples, total_samples, sample_rate,
                          V21_CH1, result->ch1_bursts, result->ch1_burst_count,
                          &result->cm, NULL);

    /* Decode V.8 messages from CH2 bursts (JM from answerer, CJ) */
    decode_v8_fsk_channel(samples, total_samples, sample_rate,
                          V21_CH2, result->ch2_bursts, result->ch2_burst_count,
                          &result->jm, &result->cj);

    /* Role detection based on what we found */
    if (result->cm.detected && !result->jm.detected) {
        result->role_detected = true;
        result->is_caller = true;
    } else if (result->jm.detected && !result->cm.detected) {
        result->role_detected = true;
        result->is_caller = false;
    } else if (result->answer_tone.detected && !result->cng.detected && !result->ct.detected) {
        result->role_detected = true;
        result->is_caller = false;
    } else if ((result->cng.detected || result->ct.detected) && !result->answer_tone.detected) {
        result->role_detected = true;
        result->is_caller = true;
    }
}

/* ------------------------------------------------------------------ */
/* Phase 2: V.34 INFO frame decode from V.21 FSK                      */
/* ------------------------------------------------------------------ */

typedef struct {
    v34_info_collector_t collector;
    bool frame_complete;
    uint8_t frame_bytes[V34_INFO_MAX_BUF_BYTES];
    int frame_byte_count;
    int frame_sample_offset;
} info_frame_decoder_t;

static void info_decoder_put_bit(void *ctx, int bit, int sample_offset)
{
    info_frame_decoder_t *d = (info_frame_decoder_t *)ctx;

    if (!d || d->frame_complete)
        return;

    if (v34_info_collector_push_bit(&d->collector, bit,
                                     d->frame_bytes, V34_INFO_MAX_BUF_BYTES)) {
        d->frame_complete = true;
        d->frame_sample_offset = sample_offset;
        d->frame_byte_count = (d->collector.target_bits + 7) / 8;
    }
}

static bool decode_info_from_fsk_burst(const int16_t *samples,
                                       int total_samples,
                                       int sample_rate,
                                       int channel,
                                       const p12_fsk_burst_t *burst,
                                       int target_bits,
                                       uint8_t *frame_out,
                                       int *frame_sample_out)
{
    info_frame_decoder_t decoder;

    if (!burst || !burst->seen || !frame_out)
        return false;

    int start = burst->start_sample;
    int len = burst->duration_samples;
    if (start < 0 || start + len > total_samples)
        return false;

    /* Try both polarities */
    for (int inv = 0; inv < 2; inv++) {
        memset(&decoder, 0, sizeof(decoder));
        v34_info_collector_init(&decoder.collector, target_bits);

        v21_fsk_demod_block(samples + start, len, sample_rate,
                            channel, (bool)inv,
                            info_decoder_put_bit, &decoder);

        if (decoder.frame_complete) {
            memcpy(frame_out, decoder.frame_bytes, V34_INFO_MAX_BUF_BYTES);
            if (frame_sample_out)
                *frame_sample_out = start + decoder.frame_sample_offset;
            return true;
        }
    }

    return false;
}

/*
 * Scan for V.34 line probing tones (2743, 2800, 3000, 3200, 3429 Hz).
 */
static void detect_phase2_probing_tones(const int16_t *samples,
                                        int total_samples,
                                        int sample_rate,
                                        int search_start,
                                        int search_end,
                                        phase12_result_t *result)
{
    static const double probe_freqs[] = { 2743.0, 2800.0, 3000.0, 3200.0, 3429.0 };
    int n_freqs = (int)(sizeof(probe_freqs) / sizeof(probe_freqs[0]));

    if (search_start < 0) search_start = 0;
    if (search_end <= 0 || search_end > total_samples) search_end = total_samples;

    for (int fi = 0; fi < n_freqs && result->probe_tone_count < P12_MAX_PROBE_TONES; fi++) {
        int run_start = -1, run_windows = 0;
        double run_peak = 0.0;
        int min_windows = 3;

        for (int pos = search_start; pos + TONE_WINDOW_SAMPLES <= search_end; pos += TONE_STEP_SAMPLES) {
            double energy = window_energy(samples + pos, TONE_WINDOW_SAMPLES);
            double ratio;

            if (energy <= 0.0) goto probe_gap;
            ratio = tone_energy_ratio(samples + pos, TONE_WINDOW_SAMPLES,
                                      sample_rate, probe_freqs[fi], energy);
            if (ratio < 0.10) goto probe_gap;

            if (run_start < 0) run_start = pos;
            run_windows++;
            if (ratio > run_peak) run_peak = ratio;
            continue;

probe_gap:
            if (run_start >= 0 && run_windows >= min_windows) {
                p12_probe_tone_hit_t *hit = &result->probe_tones[result->probe_tone_count];
                hit->detected = true;
                hit->start_sample = run_start;
                hit->duration_samples = (run_windows - 1) * TONE_STEP_SAMPLES + TONE_WINDOW_SAMPLES;
                hit->freq_hz = probe_freqs[fi];
                hit->peak_ratio = run_peak;
                result->probe_tone_count++;
                if (result->probe_tone_count >= P12_MAX_PROBE_TONES)
                    return;
            }
            run_start = -1;
            run_windows = 0;
            run_peak = 0.0;
        }

        /* Trailing */
        if (run_start >= 0 && run_windows >= min_windows
            && result->probe_tone_count < P12_MAX_PROBE_TONES) {
            p12_probe_tone_hit_t *hit = &result->probe_tones[result->probe_tone_count];
            hit->detected = true;
            hit->start_sample = run_start;
            hit->duration_samples = (run_windows - 1) * TONE_STEP_SAMPLES + TONE_WINDOW_SAMPLES;
            hit->freq_hz = probe_freqs[fi];
            hit->peak_ratio = run_peak;
            result->probe_tone_count++;
        }
    }
}

static void detect_phase2_info(const int16_t *samples,
                               int total_samples,
                               int sample_rate,
                               int max_sample,
                               phase12_result_t *result)
{
    int limit = (max_sample > 0 && max_sample < total_samples) ? max_sample : total_samples;
    int search_start = 0;
    int phase2_cap = (sample_rate * PHASE2_MAX_SCAN_MS) / 1000;
    int phase2_end_hint = -1;

    if (result->answer_tone.detected)
        search_start = result->answer_tone.start_sample + result->answer_tone.duration_samples;
    if (result->cj.detected && result->cj.sample_offset > search_start)
        search_start = result->cj.sample_offset;
    if (result->cm.detected
        && result->cm.sample_offset + result->cm.duration_samples > search_start) {
        search_start = result->cm.sample_offset + result->cm.duration_samples;
    }
    if (result->jm.detected
        && result->jm.sample_offset + result->jm.duration_samples > search_start) {
        search_start = result->jm.sample_offset + result->jm.duration_samples;
    }
    if (phase2_cap > 0 && search_start + phase2_cap < limit)
        limit = search_start + phase2_cap;

    /* INFO0 is sent on V.21 CH2 (answerer → caller direction)
     * INFO1 is sent on V.21 CH1 (caller → answerer direction)
     *
     * Scan for V.21 FSK bursts, then try to decode INFO frames from each.
     * The v34_info_collector handles sync code detection (0x372).
     */

    /* Refresh burst detection if not already done (in case phase1 was skipped) */
    if (result->ch2_burst_count == 0) {
        result->ch2_burst_count = detect_fsk_bursts(samples, total_samples, sample_rate,
                                                     search_start, limit, V21_CH2,
                                                     result->ch2_bursts, P12_MAX_FSK_BURSTS);
    }
    if (result->ch1_burst_count == 0) {
        result->ch1_burst_count = detect_fsk_bursts(samples, total_samples, sample_rate,
                                                     search_start, limit, V21_CH1,
                                                     result->ch1_bursts, P12_MAX_FSK_BURSTS);
    }

    /* Try to decode INFO0 from CH2 bursts */
    /* INFO0a is 49 bits (Table 8), INFO0d is 62 bits (Table 7) */
    for (int b = 0; b < result->ch2_burst_count && !result->info0.detected; b++) {
        uint8_t frame_bytes[V34_INFO_MAX_BUF_BYTES];
        int frame_sample = 0;

        /* Try INFO0a (49 bits) first, then INFO0d (62 bits) */
        for (int try_info0d = 0; try_info0d < 2 && !result->info0.detected; try_info0d++) {
            int target = try_info0d ? 62 : 49;

            if (decode_info_from_fsk_burst(samples, total_samples, sample_rate,
                                           V21_CH2, &result->ch2_bursts[b],
                                           target, frame_bytes, &frame_sample)) {
                v34_info_frame_t frame;
                v34_v90_info0a_t raw;
                v90_info0a_t mapped;

                memset(&frame, 0, sizeof(frame));
                frame.valid = true;
                frame.target_bits = target;
                frame.payload_bytes = (target + 7) / 8;
                memcpy(frame.payload, frame_bytes, (size_t)frame.payload_bytes);

                if (v34_info_parse_info0a_v90_frame(&frame, &raw, &mapped)) {
                    result->info0.detected = true;
                    result->info0.sample_offset = frame_sample;
                    result->info0.duration_samples = result->ch2_bursts[b].duration_samples;
                    result->info0.is_info0d = (bool)try_info0d;
                    result->info0.raw = raw;
                    result->info0.parsed = mapped;
                    phase2_end_hint = result->info0.sample_offset + result->info0.duration_samples;
                }
            }
        }
    }

    /* Try to decode INFO1 from CH1 bursts */
    /* INFO1d is 109 bits (Table 9), INFO1a is 70 bits (Table 10) */
    for (int b = 0; b < result->ch1_burst_count && !result->info1.detected; b++) {
        uint8_t frame_bytes[V34_INFO_MAX_BUF_BYTES];
        int frame_sample = 0;

        /* Try INFO1a (70 bits) first, then INFO1d (109 bits) */
        for (int try_1d = 0; try_1d < 2 && !result->info1.detected; try_1d++) {
            int target = try_1d ? 109 : 70;

            if (decode_info_from_fsk_burst(samples, total_samples, sample_rate,
                                           V21_CH1, &result->ch1_bursts[b],
                                           target, frame_bytes, &frame_sample)) {
                v34_info_frame_t frame;

                memset(&frame, 0, sizeof(frame));
                frame.valid = true;
                frame.target_bits = target;
                frame.payload_bytes = (target + 7) / 8;
                memcpy(frame.payload, frame_bytes, (size_t)frame.payload_bytes);

                if (try_1d) {
                    /* INFO1d */
                    v34_v90_info1d_t info1d;
                    if (v34_info_parse_info1d_v90_frame(&frame, &info1d)) {
                        result->info1.detected = true;
                        result->info1.sample_offset = frame_sample;
                        result->info1.duration_samples = result->ch1_bursts[b].duration_samples;
                        result->info1.is_info1d = true;
                        result->info1.info1d = info1d;
                        if (result->info1.sample_offset + result->info1.duration_samples > phase2_end_hint)
                            phase2_end_hint = result->info1.sample_offset + result->info1.duration_samples;
                    }
                } else {
                    /* INFO1a */
                    v34_v90_info1a_t raw;
                    v90_info1a_t mapped;
                    if (v34_info_parse_info1a_v90_frame(&frame, &raw, &mapped)) {
                        result->info1.detected = true;
                        result->info1.sample_offset = frame_sample;
                        result->info1.duration_samples = result->ch1_bursts[b].duration_samples;
                        result->info1.is_info1d = false;
                        result->info1.info1a_raw = raw;
                        result->info1.info1a_parsed = mapped;
                        if (result->info1.sample_offset + result->info1.duration_samples > phase2_end_hint)
                            phase2_end_hint = result->info1.sample_offset + result->info1.duration_samples;
                    }
                }
            }
        }
    }

    /* Detect line probing tones */
    {
        int probe_search_end = limit;

        if (phase2_end_hint >= 0) {
            int post_info_tail = sample_rate * 2;

            if (phase2_end_hint + post_info_tail < probe_search_end)
                probe_search_end = phase2_end_hint + post_info_tail;
        }

        detect_phase2_probing_tones(samples, total_samples, sample_rate,
                                    search_start, probe_search_end, result);
    }
}

static void phase12_finalize_diagnostics(phase12_result_t *result)
{
    int phase2_start = -1;
    int phase2_end = -1;

    if (!result)
        return;

    if (result->answer_tone.detected)
        phase2_start = result->answer_tone.start_sample + result->answer_tone.duration_samples;
    if (result->cj.detected)
        phase2_start = p12_first_non_negative(phase2_start, result->cj.sample_offset);
    if (result->cm.detected)
        phase2_start = p12_first_non_negative(phase2_start,
                                              result->cm.sample_offset + result->cm.duration_samples);
    if (result->jm.detected)
        phase2_start = p12_first_non_negative(phase2_start,
                                              result->jm.sample_offset + result->jm.duration_samples);

    if (result->info0.detected) {
        phase2_start = p12_first_non_negative(phase2_start, result->info0.sample_offset);
        phase2_end = p12_first_non_negative(phase2_end,
                                            result->info0.sample_offset + result->info0.duration_samples);
    }
    if (result->info1.detected) {
        phase2_start = p12_first_non_negative(phase2_start, result->info1.sample_offset);
        phase2_end = p12_first_non_negative(phase2_end,
                                            result->info1.sample_offset + result->info1.duration_samples);
    }
    for (int i = 0; i < result->probe_tone_count; i++) {
        phase2_start = p12_first_non_negative(phase2_start, result->probe_tones[i].start_sample);
        phase2_end = p12_first_non_negative(phase2_end,
                                            result->probe_tones[i].start_sample
                                            + result->probe_tones[i].duration_samples);
    }
    if (phase2_start >= 0) {
        result->phase2_window_known = true;
        result->phase2_start_sample = phase2_start;
        result->phase2_end_sample = (phase2_end >= phase2_start) ? phase2_end : phase2_start;
    }

    if (result->cm.detected) {
        result->pcm_modem_capable = (result->cm.modulations & (P12_MOD_V90 | P12_MOD_V92)) != 0
                                    || result->cm.pcm_modem_availability > 0;
        result->v90_capable = (result->cm.modulations & P12_MOD_V90) != 0;
        result->v92_capable = (result->cm.modulations & P12_MOD_V92) != 0;
    }
    if (result->jm.detected) {
        result->pcm_modem_capable = result->pcm_modem_capable
                                    || (result->jm.modulations & (P12_MOD_V90 | P12_MOD_V92)) != 0
                                    || result->jm.pcm_modem_availability > 0;
        result->v90_capable = result->v90_capable || ((result->jm.modulations & P12_MOD_V90) != 0);
        result->v92_capable = result->v92_capable || ((result->jm.modulations & P12_MOD_V92) != 0);
    }

    if (result->info0.detected) {
        result->short_phase2_requested = v92_short_phase2_req_from_info0_bits(result->info0.is_info0d,
                                                                               result->info0.raw.raw_26_27);
        result->v92_capable = result->v92_capable
                              || v92_short_phase2_v92_cap_from_info0_bits(result->info0.is_info0d,
                                                                          result->info0.raw.raw_26_27);
        result->v90_capable = result->v90_capable || result->info0.parsed.acknowledge_info0d;
        result->digital_side_likely = result->info0.is_info0d;
        result->info_path_known = true;
    }
    if (result->info1.detected && !result->info1.is_info1d) {
        result->inferred_u_info = result->info1.info1a_parsed.u_info;
        result->inferred_upstream_symbol_rate_code = result->info1.info1a_parsed.upstream_symbol_rate_code;
        result->inferred_downstream_rate_code = result->info1.info1a_parsed.downstream_rate_code;
        result->info_path_known = true;
    }
    if (result->role_detected)
        result->role_confident = true;

    if (result->info0.detected) {
        v92_short_phase2_observation_t obs;

        memset(&obs, 0, sizeof(obs));
        obs.info0_seen = true;
        obs.info0_is_d = result->info0.is_info0d;
        obs.short_phase2_requested = result->short_phase2_requested;
        obs.v92_capable = result->v92_capable;
        obs.info0_ack = result->info0.parsed.acknowledge_info0d;
        obs.info1_seen = result->info1.detected;
        obs.info0_sample = result->info0.sample_offset;
        obs.info1_sample = result->info1.detected ? result->info1.sample_offset : -1;
        if (v92_short_phase2_analyze(&obs, &result->short_phase2))
            result->short_phase2_analysis_valid = true;
    }
}

/* ------------------------------------------------------------------ */
/* Call log event generation                                           */
/* ------------------------------------------------------------------ */

static const char *p12_tone_type_name(p12_tone_type_t type)
{
    switch (type) {
    case P12_TONE_CNG:       return "CNG";
    case P12_TONE_CT:        return "CT";
    case P12_TONE_ANS:       return "ANS";
    case P12_TONE_ANS_PR:    return "ANS/PR";
    case P12_TONE_ANSAM:     return "ANSam";
    case P12_TONE_ANSAM_PR:  return "ANSam/PR";
    default:                 return "Unknown";
    }
}

static void emit_tone_event(call_log_t *log, const p12_tone_hit_t *tone,
                            const char *protocol)
{
    char summary[160];
    char detail[256];

    if (!tone->detected)
        return;

    snprintf(summary, sizeof(summary), "%s tone detected", p12_tone_type_name(tone->type));
    snprintf(detail, sizeof(detail), "peak_ratio=%.3f", tone->peak_ratio);
    call_log_append(log, tone->start_sample, tone->duration_samples,
                    protocol, summary, detail);
}

static void emit_cm_jm_event(call_log_t *log, const p12_cm_jm_hit_t *msg,
                              const char *label)
{
    char summary[160];
    char detail[1024];

    if (!msg->detected)
        return;

    snprintf(summary, sizeof(summary), "%s decoded", label);
    detail[0] = '\0';

    /* Format modulation flags */
    static const struct { int flag; const char *name; } mod_table[] = {
        { P12_MOD_V17, "V.17" }, { P12_MOD_V21, "V.21" },
        { P12_MOD_V22, "V.22bis" }, { P12_MOD_V23, "V.23" },
        { P12_MOD_V27TER, "V.27ter" }, { P12_MOD_V29, "V.29" },
        { P12_MOD_V32, "V.32bis" }, { P12_MOD_V34, "V.34" },
        { P12_MOD_V90, "V.90" }, { P12_MOD_V92, "V.92" }
    };

    snprintf(detail, sizeof(detail), "modulations=");
    for (int i = 0; i < (int)(sizeof(mod_table) / sizeof(mod_table[0])); i++) {
        if (msg->modulations & mod_table[i].flag) {
            size_t dlen = strlen(detail);
            snprintf(detail + dlen, sizeof(detail) - dlen, "%s%s",
                     dlen > 12 ? "," : "", mod_table[i].name);
        }
    }

    {
        size_t dlen = strlen(detail);
        snprintf(detail + dlen, sizeof(detail) - dlen,
                 " call_function=%d protocols=%d pstn=%d pcm_modem=%d bytes=%d",
                 msg->call_function, msg->protocols, msg->pstn_access,
                 msg->pcm_modem_availability, msg->byte_count);
    }

    call_log_append(log, msg->sample_offset, msg->duration_samples,
                    "V.8", summary, detail);
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

void phase12_result_init(phase12_result_t *r)
{
    if (!r)
        return;
    memset(r, 0, sizeof(*r));
    r->log.events = NULL;
    r->log.count = 0;
    r->log.cap = 0;
}

void phase12_result_reset(phase12_result_t *r)
{
    if (!r)
        return;
    free(r->log.events);
    r->log.events = NULL;
    r->log.count = 0;
    r->log.cap = 0;
}

bool phase12_decode_phase1(const int16_t *samples,
                           int total_samples,
                           int sample_rate,
                           int max_sample,
                           phase12_result_t *result)
{
    if (!samples || total_samples <= 0 || !result)
        return false;

    detect_phase1_tones(samples, total_samples, sample_rate, max_sample, result);
    detect_phase1_v8(samples, total_samples, sample_rate, max_sample, result);
    phase12_finalize_diagnostics(result);

    return (result->cng.detected || result->ct.detected
            || result->answer_tone.detected
            || result->cm.detected || result->jm.detected
            || result->cj.detected);
}

bool phase12_decode_phase2(const int16_t *samples,
                           int total_samples,
                           int sample_rate,
                           int max_sample,
                           const phase12_result_t *phase1_ctx,
                           phase12_result_t *result)
{
    if (!samples || total_samples <= 0 || !result)
        return false;

    /* If phase1 context provided, copy FSK burst info to avoid re-scanning */
    if (phase1_ctx) {
        result->ch1_burst_count = phase1_ctx->ch1_burst_count;
        memcpy(result->ch1_bursts, phase1_ctx->ch1_bursts, sizeof(result->ch1_bursts));
        result->ch2_burst_count = phase1_ctx->ch2_burst_count;
        memcpy(result->ch2_bursts, phase1_ctx->ch2_bursts, sizeof(result->ch2_bursts));
        result->answer_tone = phase1_ctx->answer_tone;
        result->cj = phase1_ctx->cj;
    }

    detect_phase2_info(samples, total_samples, sample_rate, max_sample, result);
    phase12_finalize_diagnostics(result);

    return (result->info0.detected || result->info1.detected
            || result->probe_tone_count > 0);
}

bool phase12_decode(const int16_t *samples,
                    int total_samples,
                    int sample_rate,
                    int max_sample,
                    phase12_result_t *result)
{
    bool p1, p2;

    if (!samples || total_samples <= 0 || !result)
        return false;

    p1 = phase12_decode_phase1(samples, total_samples, sample_rate, max_sample, result);
    p2 = phase12_decode_phase2(samples, total_samples, sample_rate, max_sample,
                                result, result);
    return p1 || p2;
}

void phase12_merge_to_call_log(const phase12_result_t *result,
                               call_log_t *log,
                               int sample_rate)
{
    if (!result || !log)
        return;

    (void)sample_rate;

    /* Phase 1 tones */
    emit_tone_event(log, &result->cng, "V.8");
    emit_tone_event(log, &result->ct, "V.8");
    emit_tone_event(log, &result->answer_tone, "V.8");

    /* Phase 1 V.8 messages */
    emit_cm_jm_event(log, &result->cm, "CM");
    emit_cm_jm_event(log, &result->jm, "JM");

    if (result->cj.detected) {
        call_log_append(log, result->cj.sample_offset, 0,
                        "V.8", "CJ (ack)", NULL);
    }

    /* Phase 2 INFO frames */
    if (result->info0.detected) {
        char summary[160];
        snprintf(summary, sizeof(summary), "INFO0%s decoded",
                 result->info0.is_info0d ? "d" : "a");
        call_log_append(log, result->info0.sample_offset,
                        result->info0.duration_samples,
                        "V.34", summary, NULL);
    }

    if (result->info1.detected) {
        char summary[160];
        snprintf(summary, sizeof(summary), "INFO1%s decoded",
                 result->info1.is_info1d ? "d" : "a");
        call_log_append(log, result->info1.sample_offset,
                        result->info1.duration_samples,
                        "V.34", summary, NULL);
    }

    if (result->phase2_window_known || result->pcm_modem_capable || result->short_phase2_analysis_valid) {
        char detail[512];
        int event_sample = result->phase2_window_known ? result->phase2_start_sample : 0;

        snprintf(detail, sizeof(detail),
                 "role=%s pcm_modem=%u v90=%u v92=%u short_p2_req=%u digital_side=%u phase2_start=%.1fms phase2_end=%.1fms",
                 result->role_detected ? (result->is_caller ? "caller" : "answerer") : "unknown",
                 result->pcm_modem_capable ? 1U : 0U,
                 result->v90_capable ? 1U : 0U,
                 result->v92_capable ? 1U : 0U,
                 result->short_phase2_requested ? 1U : 0U,
                 result->digital_side_likely ? 1U : 0U,
                 result->phase2_window_known ? (double) result->phase2_start_sample * 1000.0 / (double) sample_rate : -1.0,
                 result->phase2_window_known ? (double) result->phase2_end_sample * 1000.0 / (double) sample_rate : -1.0);
        if (result->info_path_known) {
            size_t dlen = strlen(detail);
            snprintf(detail + dlen, sizeof(detail) - dlen,
                     " u_info=%d up_code=%d down_code=%d",
                     result->inferred_u_info,
                     result->inferred_upstream_symbol_rate_code,
                     result->inferred_downstream_rate_code);
        }
        if (result->short_phase2_analysis_valid) {
            size_t dlen = strlen(detail);
            snprintf(detail + dlen, sizeof(detail) - dlen,
                     " short_p2_seq=%s status=%s",
                     v92_short_phase2_sequence_id(result->short_phase2.sequence),
                     result->short_phase2.status ? result->short_phase2.status : "unknown");
        }
        call_log_append(log, event_sample, 0, "V.34", "Phase 1/2 diagnostic", detail);
    }

    /* Phase 2 probing tones */
    for (int i = 0; i < result->probe_tone_count; i++) {
        char summary[160];
        char detail[128];

        snprintf(summary, sizeof(summary), "Probing tone %.0f Hz",
                 result->probe_tones[i].freq_hz);
        snprintf(detail, sizeof(detail), "peak_ratio=%.3f",
                 result->probe_tones[i].peak_ratio);
        call_log_append(log, result->probe_tones[i].start_sample,
                        result->probe_tones[i].duration_samples,
                        "V.34", summary, detail);
    }
}
