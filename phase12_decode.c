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
#include "call_init_tone_probe.h"
#include "v21_fsk_demod.h"
#include "v34_info_decode.h"
#include "v90.h"
#include "v92_short_phase2_decode.h"

#include <math.h>
#include <stdarg.h>
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

enum {
    P12_INFO0A_PAYLOAD_BITS = 49 - (4 + 8 + 4),
    P12_INFO0D_PAYLOAD_BITS = 62 - (4 + 8 + 4),
    P12_INFO1A_PAYLOAD_BITS = 70 - (4 + 8 + 4),
    P12_INFO1D_PAYLOAD_BITS = 109 - (4 + 8 + 4)
};

enum {
    P12_INFO_PRE_PAD_MS = 120,
    P12_INFO_POST_PAD_MS = 160,
    P12_INFO_STREAM_PHASES = 4,
    P12_INFO_STREAM_BAUD_VARIANTS = 5
};

enum {
    P12_PHASE1_REPEAT_GAP_MS = 500,
    P12_PHASE1_PRE_PAD_MS = 80,
    P12_PHASE1_POST_PAD_MS = 120
};

enum {
    P12_PHASE2_REPEAT_GAP_MS = 500
};

#define P12_PHASE2_BURST_RELATIVE_SIGNAL_CUTOFF 0.35

enum {
    P12_CJ_TO_INFO0_MS = 75,
    P12_CJ_TO_INFO0_TOL_MS = 10,
    P12_INFO_RETRY_WINDOW_MS = 260
};

enum {
    P12_INFO0_PRE_TONE_MAX_MS = 220,
    P12_INFO0_PRE_TONE_GUARD_MS = 25,
    P12_INFO0_RETRY_EARLY_MS = 80,
    P12_INFO0_RETRY_LEADIN_MS = 40,
    P12_INFO0_DECODE_MAX_MS = 320
};

enum {
    P12_INFO0_SUBWINDOW_MS = 90,
    P12_INFO0_SUBWINDOW_STEP_MS = 20
};

enum {
    P12_PHASE2_SIGNAL_MIN_MS = 40
};

enum {
    P12_PHASE2_CC_WINDOW_SAMPLES = 80,
    P12_PHASE2_CC_STEP_SAMPLES = 40
};

enum {
    P12_ANS_WINDOW_SAMPLES = 160,
    P12_ANS_STEP_SAMPLES = 80
};

enum {
    P12_CALL_INIT_MAX_MS = 10000
};

enum {
    P12_ANSAM_END_GRACE_MS = 30
};

/* Tone detection thresholds */
static const double TONE_RATIO_THRESHOLD = 0.15;
static const double TONE_COMPETITOR_MAX = 0.08;
static const double FSK_BURST_THRESHOLD = 0.12;
static int g_p12_debug_enabled = -1;

static bool p12_debug_enabled(void)
{
    if (g_p12_debug_enabled < 0)
        g_p12_debug_enabled = (getenv("VPCM_P12_DEBUG") != NULL) ? 1 : 0;
    return g_p12_debug_enabled != 0;
}

static int p12_answer_tone_post_hold_samples(const p12_tone_hit_t *tone,
                                             int sample_rate)
{
    if (!tone || !tone->detected || sample_rate <= 0)
        return 0;

    switch (tone->type) {
    case P12_TONE_ANSAM:
    case P12_TONE_ANSAM_PR:
        return (sample_rate * P12_ANSAM_END_GRACE_MS) / 1000;
    default:
        return 0;
    }
}

static void p12_debug_log_bursts(const char *label,
                                 const p12_fsk_burst_t *bursts,
                                 int burst_count,
                                 int sample_rate)
{
    if (!p12_debug_enabled())
        return;
    fprintf(stderr, "[p12] %s bursts=%d\n", label ? label : "?", burst_count);
    for (int i = 0; i < burst_count; i++) {
        fprintf(stderr,
                "[p12]   #%d start=%.1fms dur=%.1fms peak=%.3f sig=%.0f\n",
                i + 1,
                (double) bursts[i].start_sample * 1000.0 / (double) sample_rate,
                (double) bursts[i].duration_samples * 1000.0 / (double) sample_rate,
                bursts[i].peak_energy,
                bursts[i].peak_signal_energy);
    }
}

static int p12_first_non_negative(int a, int b)
{
    if (a >= 0 && b >= 0)
        return (a < b) ? a : b;
    if (a >= 0)
        return a;
    return b;
}

static int p12_merge_bursts_in_place(p12_fsk_burst_t *bursts,
                                     int burst_count,
                                     int merge_gap_samples)
{
    int write_idx = 0;

    if (!bursts || burst_count <= 0)
        return 0;

    for (int i = 0; i < burst_count; i++) {
        if (!bursts[i].seen)
            continue;
        if (write_idx > 0) {
            int prev_end = bursts[write_idx - 1].start_sample
                           + bursts[write_idx - 1].duration_samples;
            if (bursts[i].start_sample <= prev_end + merge_gap_samples) {
                int new_end = bursts[i].start_sample + bursts[i].duration_samples;

                if (new_end > prev_end)
                    bursts[write_idx - 1].duration_samples =
                        new_end - bursts[write_idx - 1].start_sample;
                if (bursts[i].peak_energy > bursts[write_idx - 1].peak_energy)
                    bursts[write_idx - 1].peak_energy = bursts[i].peak_energy;
                if (bursts[i].peak_signal_energy > bursts[write_idx - 1].peak_signal_energy)
                    bursts[write_idx - 1].peak_signal_energy = bursts[i].peak_signal_energy;
                continue;
            }
        }
        if (write_idx != i)
            bursts[write_idx] = bursts[i];
        write_idx++;
    }

    for (int i = write_idx; i < burst_count; i++)
        memset(&bursts[i], 0, sizeof(bursts[i]));
    return write_idx;
}

static bool detect_tone(const int16_t *samples,
                        int total_samples,
                        int sample_rate,
                        double freq_hz,
                        const double *competitor_freqs,
                        int n_competitors,
                        int min_run_ms,
                        bool require_cadence,
                        p12_tone_hit_t *out);
static bool detect_phase_reversals(const int16_t *samples,
                                   int len,
                                   int sample_rate,
                                   int *reversal_count_out);

static void p12_fill_timing_hint_from_cj(p12_timing_hint_t *hint,
                                         const p12_cj_hit_t *cj,
                                         int sample_rate,
                                         int limit)
{
    int cj_end;
    int expected;
    int tol;
    int retry_len;

    if (!hint)
        return;
    memset(hint, 0, sizeof(*hint));
    if (!cj || !cj->detected || sample_rate <= 0)
        return;

    cj_end = cj->sample_offset + cj->duration_samples;
    expected = cj_end + (sample_rate * P12_CJ_TO_INFO0_MS) / 1000;
    tol = (sample_rate * P12_CJ_TO_INFO0_TOL_MS) / 1000;
    retry_len = (sample_rate * P12_INFO_RETRY_WINDOW_MS) / 1000;

    hint->valid = true;
    hint->anchor_sample = cj_end;
    hint->expected_sample = expected;
    hint->window_start_sample = expected - tol;
    hint->window_end_sample = hint->window_start_sample + retry_len;
    if (hint->window_start_sample < 0)
        hint->window_start_sample = 0;
    if (limit > 0 && hint->window_end_sample > limit)
        hint->window_end_sample = limit;
    if (hint->window_end_sample < hint->window_start_sample)
        hint->window_end_sample = hint->window_start_sample;
}

static void p12_fill_timing_hint_from_jm_end(p12_timing_hint_t *hint,
                                             const p12_cm_jm_hit_t *jm,
                                             int sample_rate,
                                             int limit)
{
    int jm_end;
    int expected;
    int tol;
    int retry_len;

    if (!hint)
        return;
    memset(hint, 0, sizeof(*hint));
    if (!jm || !jm->detected || sample_rate <= 0 || jm->duration_samples <= 0)
        return;

    jm_end = jm->sample_offset + jm->duration_samples;
    expected = jm_end + (sample_rate * P12_CJ_TO_INFO0_MS) / 1000;
    tol = (sample_rate * P12_CJ_TO_INFO0_TOL_MS) / 1000;
    retry_len = (sample_rate * P12_INFO_RETRY_WINDOW_MS) / 1000;

    hint->valid = true;
    hint->anchor_sample = jm_end;
    hint->expected_sample = expected;
    hint->window_start_sample = expected - tol;
    hint->window_end_sample = hint->window_start_sample + retry_len;
    if (hint->window_start_sample < 0)
        hint->window_start_sample = 0;
    if (limit > 0 && hint->window_end_sample > limit)
        hint->window_end_sample = limit;
    if (hint->window_end_sample < hint->window_start_sample)
        hint->window_end_sample = hint->window_start_sample;
}

static void p12_append_retry_window(p12_fsk_burst_t *bursts,
                                    int *burst_count,
                                    int max_bursts,
                                    const p12_timing_hint_t *hint)
{
    p12_fsk_burst_t retry;

    if (!bursts || !burst_count || !hint || !hint->valid)
        return;
    if (*burst_count >= max_bursts)
        return;
    if (hint->window_end_sample <= hint->window_start_sample)
        return;

    memset(&retry, 0, sizeof(retry));
    retry.seen = true;
    retry.start_sample = hint->window_start_sample;
    retry.duration_samples = hint->window_end_sample - hint->window_start_sample;
    retry.peak_energy = 0.0;
    retry.hint_id = 0;
    bursts[*burst_count] = retry;
    (*burst_count)++;
}

static void p12_append_retry_window_range(p12_fsk_burst_t *bursts,
                                          int *burst_count,
                                          int max_bursts,
                                          int start_sample,
                                          int end_sample)
{
    p12_timing_hint_t hint;

    memset(&hint, 0, sizeof(hint));
    hint.valid = true;
    hint.anchor_sample = start_sample;
    hint.expected_sample = start_sample;
    hint.window_start_sample = start_sample;
    hint.window_end_sample = end_sample;
    p12_append_retry_window(bursts, burst_count, max_bursts, &hint);
}

static void p12_append_info0_retry_bank(p12_fsk_burst_t *bursts,
                                        int *burst_count,
                                        int max_bursts,
                                        const p12_timing_hint_t *hint,
                                        int tone_anchor_sample,
                                        int search_start,
                                        int search_limit,
                                        int sample_rate)
{
    int early_shift;
    int leadin;
    int earlier_start;
    int overlap_end;

    if (!hint || !hint->valid || !bursts || !burst_count || sample_rate <= 0)
        return;

    p12_append_retry_window(bursts, burst_count, max_bursts, hint);

    early_shift = (sample_rate * P12_INFO0_RETRY_EARLY_MS) / 1000;
    earlier_start = hint->window_start_sample - early_shift;
    if (earlier_start < search_start)
        earlier_start = search_start;
    if (earlier_start < hint->window_start_sample) {
        p12_append_retry_window_range(bursts,
                                      burst_count,
                                      max_bursts,
                                      earlier_start,
                                      hint->window_end_sample);
    }

    if (tone_anchor_sample > 0) {
        leadin = (sample_rate * P12_INFO0_RETRY_LEADIN_MS) / 1000;
        overlap_end = tone_anchor_sample + leadin;
        if (overlap_end > search_limit)
            overlap_end = search_limit;
        if (overlap_end > hint->window_start_sample) {
            p12_append_retry_window_range(bursts,
                                          burst_count,
                                          max_bursts,
                                          hint->window_start_sample,
                                          overlap_end);
        }
    }
}

static p12_phase2_role_t p12_phase2_role_from_observations(const phase12_result_t *result)
{
    if (!result)
        return P12_PHASE2_ROLE_UNKNOWN;
    if (result->v90_capable && result->role_detected && !result->is_caller)
        return P12_PHASE2_ROLE_V90_DIGITAL_ANSWERER;
    if (result->v90_capable && result->role_detected && result->is_caller)
        return P12_PHASE2_ROLE_V90_ANALOG_CALLER;
    if (result->role_detected && result->is_caller)
        return P12_PHASE2_ROLE_V34_CALLER;
    if (result->role_detected && !result->is_caller)
        return P12_PHASE2_ROLE_V34_ANSWERER;
    return P12_PHASE2_ROLE_UNKNOWN;
}

static p12_phase2_role_t p12_phase2_role_v90_swapped(p12_phase2_role_t role)
{
    switch (role) {
    case P12_PHASE2_ROLE_V90_DIGITAL_ANSWERER:
        return P12_PHASE2_ROLE_V90_ANALOG_CALLER;
    case P12_PHASE2_ROLE_V90_ANALOG_CALLER:
        return P12_PHASE2_ROLE_V90_DIGITAL_ANSWERER;
    default:
        return role;
    }
}

static p12_info0_kind_t p12_classify_info0_kind(const phase12_result_t *result,
                                                bool is_info0d)
{
    if (is_info0d)
        return P12_INFO0_KIND_V90_INFO0D;
    if (result && result->short_phase2_requested)
        return P12_INFO0_KIND_V92_SHORT;
    if (result && result->role_detected && result->is_caller)
        return P12_INFO0_KIND_SHARED_INFO0A;
    if (result && result->role_detected && !result->is_caller)
        return P12_INFO0_KIND_V34_INFO0C;
    return P12_INFO0_KIND_SHARED_INFO0A;
}

static p12_info1_kind_t p12_classify_info1_kind(const phase12_result_t *result,
                                                bool is_info1d)
{
    if (is_info1d)
        return P12_INFO1_KIND_V90_INFO1D;
    if (result && result->short_phase2_requested)
        return P12_INFO1_KIND_V92_SHORT;
    if (result && result->v90_capable)
        return P12_INFO1_KIND_V90_INFO1A;
    if (result && result->role_detected && result->is_caller)
        return P12_INFO1_KIND_V34_INFO1C;
    if (result && result->role_detected && !result->is_caller)
        return P12_INFO1_KIND_V34_INFO1A;
    return P12_INFO1_KIND_UNKNOWN;
}

static bool p12_role_expects_info0_first(p12_phase2_role_t role)
{
    switch (role) {
    case P12_PHASE2_ROLE_V34_CALLER:
    case P12_PHASE2_ROLE_V34_ANSWERER:
    case P12_PHASE2_ROLE_V90_ANALOG_CALLER:
    case P12_PHASE2_ROLE_V90_DIGITAL_ANSWERER:
        return true;
    default:
        return false;
    }
}

static void p12_fill_handoff_info0_hint(p12_timing_hint_t *hint,
                                        p12_phase2_role_t role,
                                        int search_start,
                                        int search_limit,
                                        const p12_fsk_burst_t *opposite_windows,
                                        int opposite_window_count,
                                        int sample_rate)
{
    int retry_len;
    int window_end;

    if (!hint)
        return;
    memset(hint, 0, sizeof(*hint));
    if (!p12_role_expects_info0_first(role) || sample_rate <= 0 || search_limit <= search_start)
        return;

    retry_len = (sample_rate * P12_INFO_RETRY_WINDOW_MS) / 1000;
    window_end = search_start + retry_len;
    if (opposite_windows && opposite_window_count > 0) {
        int opposite_guard = opposite_windows[0].start_sample + (sample_rate * P12_INFO_POST_PAD_MS) / 1000;
        if (opposite_guard > window_end)
            window_end = opposite_guard;
    }
    if (window_end > search_limit)
        window_end = search_limit;
    if (window_end <= search_start)
        return;

    hint->valid = true;
    hint->anchor_sample = search_start;
    hint->expected_sample = search_start;
    hint->window_start_sample = search_start;
    hint->window_end_sample = window_end;
}

static void p12_refine_info0_hint_from_tone(p12_timing_hint_t *hint,
                                            const p12_signal_tone_hit_t *tone,
                                            int sample_rate)
{
    int max_span;
    int guard;
    int refined_start;
    int refined_end;

    if (!hint || !hint->valid || !tone || !tone->detected || sample_rate <= 0)
        return;

    max_span = (sample_rate * P12_INFO0_PRE_TONE_MAX_MS) / 1000;
    guard = (sample_rate * P12_INFO0_PRE_TONE_GUARD_MS) / 1000;
    refined_end = tone->start_sample - guard;
    if (refined_end <= hint->window_start_sample)
        return;

    refined_start = refined_end - max_span;
    if (refined_start < hint->window_start_sample)
        refined_start = hint->window_start_sample;
    if (refined_end > hint->window_end_sample)
        refined_end = hint->window_end_sample;
    if (refined_end <= refined_start)
        return;

    hint->anchor_sample = tone->start_sample;
    hint->expected_sample = refined_start;
    hint->window_start_sample = refined_start;
    hint->window_end_sample = refined_end;
}

static int p12_burst_end_sample(const p12_fsk_burst_t *burst)
{
    if (!burst)
        return -1;
    return burst->start_sample + burst->duration_samples;
}

static int p12_burst_center_sample(const p12_fsk_burst_t *burst)
{
    int end;

    if (!burst)
        return -1;
    end = p12_burst_end_sample(burst);
    if (end < burst->start_sample)
        return burst->start_sample;
    return burst->start_sample + (end - burst->start_sample) / 2;
}

static int p12_burst_window_distance(const p12_fsk_burst_t *burst,
                                     int window_start,
                                     int window_end)
{
    int burst_end;

    if (!burst || window_end <= window_start)
        return 0;

    burst_end = p12_burst_end_sample(burst);
    if (burst_end <= window_start)
        return window_start - burst_end;
    if (burst->start_sample >= window_end)
        return burst->start_sample - window_end;
    return 0;
}

static bool p12_info_burst_precedes(const p12_fsk_burst_t *a,
                                    const p12_fsk_burst_t *b,
                                    const p12_timing_hint_t *hint,
                                    int tone_anchor_sample)
{
    int a_end;
    int b_end;
    bool a_before_tone;
    bool b_before_tone;
    bool a_hits_hint = false;
    bool b_hits_hint = false;
    int a_hint_dist = 0;
    int b_hint_dist = 0;
    int a_expected_dist = 0;
    int b_expected_dist = 0;

    if (!a || !b)
        return false;

    a_end = p12_burst_end_sample(a);
    b_end = p12_burst_end_sample(b);
    a_before_tone = (tone_anchor_sample < 0) || (a_end <= tone_anchor_sample);
    b_before_tone = (tone_anchor_sample < 0) || (b_end <= tone_anchor_sample);
    if (a_before_tone != b_before_tone)
        return a_before_tone;

    if (hint && hint->valid && hint->window_end_sample > hint->window_start_sample) {
        a_hint_dist = p12_burst_window_distance(a, hint->window_start_sample, hint->window_end_sample);
        b_hint_dist = p12_burst_window_distance(b, hint->window_start_sample, hint->window_end_sample);
        a_hits_hint = (a_hint_dist == 0);
        b_hits_hint = (b_hint_dist == 0);

        if (a_hits_hint != b_hits_hint)
            return a_hits_hint;
        if (a_hint_dist != b_hint_dist)
            return a_hint_dist < b_hint_dist;

        a_expected_dist = abs(p12_burst_center_sample(a) - hint->expected_sample);
        b_expected_dist = abs(p12_burst_center_sample(b) - hint->expected_sample);
        if (a_expected_dist != b_expected_dist)
            return a_expected_dist < b_expected_dist;
    } else if (tone_anchor_sample >= 0) {
        a_expected_dist = abs(a_end - tone_anchor_sample);
        b_expected_dist = abs(b_end - tone_anchor_sample);
        if (a_expected_dist != b_expected_dist)
            return a_expected_dist < b_expected_dist;
    }

    if (a->peak_signal_energy != b->peak_signal_energy)
        return a->peak_signal_energy > b->peak_signal_energy;
    if (a->peak_energy != b->peak_energy)
        return a->peak_energy > b->peak_energy;
    if (a->start_sample != b->start_sample)
        return a->start_sample < b->start_sample;
    return a->duration_samples < b->duration_samples;
}

static void p12_rank_bursts_for_info(p12_fsk_burst_t *bursts,
                                     int burst_count,
                                     const p12_timing_hint_t *hint,
                                     int tone_anchor_sample,
                                     const char *label,
                                     int sample_rate)
{
    if (!bursts || burst_count <= 1)
        return;

    for (int i = 1; i < burst_count; i++) {
        p12_fsk_burst_t key = bursts[i];
        int j = i - 1;

        while (j >= 0 && p12_info_burst_precedes(&key, &bursts[j], hint, tone_anchor_sample)) {
            bursts[j + 1] = bursts[j];
            j--;
        }
        bursts[j + 1] = key;
    }

    if (p12_debug_enabled()) {
        fprintf(stderr,
                "[p12] ranked %s bursts=%d tone_anchor=%.1fms hint=%s\n",
                label ? label : "INFO",
                burst_count,
                (tone_anchor_sample >= 0) ? ((double) tone_anchor_sample * 1000.0 / (double) sample_rate) : -1.0,
                (hint && hint->valid) ? "yes" : "no");
        for (int i = 0; i < burst_count; i++) {
            int end = p12_burst_end_sample(&bursts[i]);
            int hint_dist = 0;

            if (hint && hint->valid)
                hint_dist = p12_burst_window_distance(&bursts[i], hint->window_start_sample, hint->window_end_sample);
            fprintf(stderr,
                    "[p12]   rank #%d start=%.1fms end=%.1fms hint_dist=%.1fms sig=%.0f peak=%.3f\n",
                    i + 1,
                    (double) bursts[i].start_sample * 1000.0 / (double) sample_rate,
                    (double) end * 1000.0 / (double) sample_rate,
                    (double) hint_dist * 1000.0 / (double) sample_rate,
                    bursts[i].peak_signal_energy,
                    bursts[i].peak_energy);
        }
    }
}

static void p12_focus_bursts_to_window(p12_fsk_burst_t *bursts,
                                       int burst_count,
                                       const p12_timing_hint_t *hint,
                                       int tone_anchor_sample,
                                       const char *label,
                                       int sample_rate)
{
    int focus_start = -1;
    int focus_end = -1;

    if (!bursts || burst_count <= 0)
        return;

    if (hint && hint->valid && hint->window_end_sample > hint->window_start_sample) {
        focus_start = hint->window_start_sample;
        focus_end = hint->window_end_sample;
    }
    if (tone_anchor_sample >= 0 && (focus_end < 0 || tone_anchor_sample < focus_end))
        focus_end = tone_anchor_sample;
    if (focus_end <= focus_start)
        return;

    for (int i = 0; i < burst_count; i++) {
        int burst_start = bursts[i].start_sample;
        int burst_end = p12_burst_end_sample(&bursts[i]);
        int clipped_start = burst_start;
        int clipped_end = burst_end;

        if (burst_end <= focus_start || burst_start >= focus_end)
            continue;
        if (clipped_start < focus_start)
            clipped_start = focus_start;
        if (clipped_end > focus_end)
            clipped_end = focus_end;
        if (clipped_end <= clipped_start)
            continue;

        if (clipped_start != burst_start || clipped_end != burst_end) {
            if (p12_debug_enabled()) {
                fprintf(stderr,
                        "[p12] focused %s burst #%d from %.1f-%.1fms to %.1f-%.1fms\n",
                        label ? label : "INFO",
                        i + 1,
                        (double) burst_start * 1000.0 / (double) sample_rate,
                        (double) burst_end * 1000.0 / (double) sample_rate,
                        (double) clipped_start * 1000.0 / (double) sample_rate,
                        (double) clipped_end * 1000.0 / (double) sample_rate);
            }
            bursts[i].start_sample = clipped_start;
            bursts[i].duration_samples = clipped_end - clipped_start;
        }
    }
}

static void p12_prepare_info0_decode_burst(p12_fsk_burst_t *dst,
                                           const p12_fsk_burst_t *src,
                                           const p12_timing_hint_t *hint,
                                           int tone_anchor_sample,
                                           int sample_rate)
{
    int decode_max;
    int start;
    int end;

    if (!dst || !src) {
        return;
    }
    *dst = *src;
    if (!dst->seen)
        return;

    start = dst->start_sample;
    end = p12_burst_end_sample(dst);
    if (hint && hint->valid && hint->window_end_sample > hint->window_start_sample) {
        if (start < hint->window_start_sample)
            start = hint->window_start_sample;
        if (end > hint->window_end_sample)
            end = hint->window_end_sample;
    }
    if (tone_anchor_sample >= 0 && end > tone_anchor_sample)
        end = tone_anchor_sample;
    if (sample_rate > 0) {
        decode_max = (sample_rate * P12_INFO0_DECODE_MAX_MS) / 1000;
        if (decode_max > 0 && end > start + decode_max)
            end = start + decode_max;
    }
    if (end <= start) {
        dst->duration_samples = 0;
        return;
    }
    dst->start_sample = start;
    dst->duration_samples = end - start;
}

static double p12_info0_tone_window_score(const int16_t *samples,
                                          int total_samples,
                                          int sample_rate,
                                          int start_sample,
                                          int duration_samples)
{
    double energy;
    double r1200;
    double r1800;
    double r2100;
    double r2400;
    double v90_pair;
    double score;

    if (!samples || total_samples <= 0 || sample_rate <= 0 || duration_samples <= 0)
        return -1.0;
    if (start_sample < 0 || start_sample + duration_samples > total_samples)
        return -1.0;

    energy = window_energy(samples + start_sample, duration_samples);
    if (energy <= 0.0)
        return -1.0;

    r1200 = tone_energy_ratio(samples + start_sample, duration_samples, sample_rate, 1200.0, energy);
    r1800 = tone_energy_ratio(samples + start_sample, duration_samples, sample_rate, 1800.0, energy);
    r2100 = tone_energy_ratio(samples + start_sample, duration_samples, sample_rate, 2100.0, energy);
    r2400 = tone_energy_ratio(samples + start_sample, duration_samples, sample_rate, 2400.0, energy);

    /* Reward either a clean 1200 Hz side or the 2400 Hz side carrying
       companion 1800 Hz energy, and strongly penalize lingering 2100 Hz. */
    v90_pair = r2400 + (0.75 * r1800) + (2.0 * sqrt(r2400 * r1800));
    score = fmax(r1200, v90_pair) - (3.0 * r2100);

    /* If 2100 Hz still clearly dominates, treat the window as likely
       ANSam/probe overlap rather than INFO-bearing phase 2. */
    if (r2100 > (fmax(fmax(r1200, r1800), r2400) * 1.5))
        score -= (2.0 * r2100);

    return score;
}

static void p12_refine_info0_decode_burst_by_tone(p12_fsk_burst_t *burst,
                                                  const int16_t *samples,
                                                  int total_samples,
                                                  int sample_rate)
{
    int subwindow_samples;
    int step_samples;
    int burst_end;
    double best_score;
    int best_start;
    int best_len;

    if (!burst || !burst->seen || !samples || total_samples <= 0 || sample_rate <= 0)
        return;

    subwindow_samples = (sample_rate * P12_INFO0_SUBWINDOW_MS) / 1000;
    step_samples = (sample_rate * P12_INFO0_SUBWINDOW_STEP_MS) / 1000;
    if (subwindow_samples <= 0 || step_samples <= 0 || burst->duration_samples <= subwindow_samples)
        return;

    burst_end = p12_burst_end_sample(burst);
    best_score = p12_info0_tone_window_score(samples,
                                             total_samples,
                                             sample_rate,
                                             burst->start_sample,
                                             burst->duration_samples);
    best_start = burst->start_sample;
    best_len = burst->duration_samples;

    for (int pos = burst->start_sample; pos + subwindow_samples <= burst_end; pos += step_samples) {
        double score = p12_info0_tone_window_score(samples,
                                                   total_samples,
                                                   sample_rate,
                                                   pos,
                                                   subwindow_samples);

        if (score > best_score + 1e-9) {
            best_score = score;
            best_start = pos;
            best_len = subwindow_samples;
        }
    }

    if (best_start != burst->start_sample || best_len != burst->duration_samples) {
        if (p12_debug_enabled()) {
            fprintf(stderr,
                    "[p12] INFO0 tone-refined window %.1f-%.1fms -> %.1f-%.1fms score=%.5f\n",
                    (double) burst->start_sample * 1000.0 / (double) sample_rate,
                    (double) burst_end * 1000.0 / (double) sample_rate,
                    (double) best_start * 1000.0 / (double) sample_rate,
                    (double) (best_start + best_len) * 1000.0 / (double) sample_rate,
                    best_score);
        }
        burst->start_sample = best_start;
        burst->duration_samples = best_len;
    }
}

static bool p12_detect_phase2_cc_run(const int16_t *samples,
                                     int total_samples,
                                     int sample_rate,
                                     double freq_hz,
                                     int min_run_ms,
                                     p12_tone_hit_t *out)
{
    int min_windows = (min_run_ms * sample_rate) / (1000 * P12_PHASE2_CC_STEP_SAMPLES);
    int run_start = -1;
    int run_windows = 0;
    double run_peak = 0.0;
    double alt_freq = (freq_hz < 1800.0) ? 2400.0 : 1200.0;

    if (!samples || !out || total_samples < P12_PHASE2_CC_WINDOW_SAMPLES)
        return false;
    memset(out, 0, sizeof(*out));
    if (min_windows < 1)
        min_windows = 1;

    for (int pos = 0; pos + P12_PHASE2_CC_WINDOW_SAMPLES <= total_samples; pos += P12_PHASE2_CC_STEP_SAMPLES) {
        double energy = window_energy(samples + pos, P12_PHASE2_CC_WINDOW_SAMPLES);
        double ratio;
        double alt_ratio;
        double guard_1800;
        double guard_2100;

        if (energy <= 0.0)
            goto cc_gap;

        ratio = tone_energy_ratio(samples + pos, P12_PHASE2_CC_WINDOW_SAMPLES, sample_rate, freq_hz, energy);
        alt_ratio = tone_energy_ratio(samples + pos, P12_PHASE2_CC_WINDOW_SAMPLES, sample_rate, alt_freq, energy);
        guard_1800 = tone_energy_ratio(samples + pos, P12_PHASE2_CC_WINDOW_SAMPLES, sample_rate, 1800.0, energy);
        guard_2100 = tone_energy_ratio(samples + pos, P12_PHASE2_CC_WINDOW_SAMPLES, sample_rate, 2100.0, energy);

        if (ratio < 0.07)
            goto cc_gap;
        if (alt_ratio > ratio * 0.85)
            goto cc_gap;
        if (guard_1800 > ratio * 0.80 || guard_2100 > ratio * 0.80)
            goto cc_gap;

        if (run_start < 0)
            run_start = pos;
        run_windows++;
        if (ratio > run_peak)
            run_peak = ratio;
        continue;

cc_gap:
        if (run_start >= 0 && run_windows >= min_windows) {
            out->detected = true;
            out->start_sample = run_start;
            out->duration_samples = (run_windows - 1) * P12_PHASE2_CC_STEP_SAMPLES + P12_PHASE2_CC_WINDOW_SAMPLES;
            out->peak_ratio = run_peak;
            return true;
        }
        run_start = -1;
        run_windows = 0;
        run_peak = 0.0;
    }

    if (run_start >= 0 && run_windows >= min_windows) {
        out->detected = true;
        out->start_sample = run_start;
        out->duration_samples = (run_windows - 1) * P12_PHASE2_CC_STEP_SAMPLES + P12_PHASE2_CC_WINDOW_SAMPLES;
        out->peak_ratio = run_peak;
        return true;
    }
    return false;
}

static bool detect_answer_tone_run(const int16_t *samples,
                                   int total_samples,
                                   int sample_rate,
                                   int min_run_ms,
                                   p12_tone_hit_t *out)
{
    int min_windows = (min_run_ms * sample_rate) / (1000 * P12_ANS_STEP_SAMPLES);
    int run_start = -1;
    int run_windows = 0;
    double run_peak = 0.0;
    int best_start = -1;
    int best_windows = 0;
    double best_peak = 0.0;

    if (!samples || !out || total_samples < P12_ANS_WINDOW_SAMPLES)
        return false;
    memset(out, 0, sizeof(*out));
    if (min_windows < 1)
        min_windows = 1;

    for (int pos = 0; pos + P12_ANS_WINDOW_SAMPLES <= total_samples; pos += P12_ANS_STEP_SAMPLES) {
        double energy = window_energy(samples + pos, P12_ANS_WINDOW_SAMPLES);
        double r2100;
        double r1800;
        double r2400;
        double r1650;
        double r1850;
        bool good = false;

        if (energy > 0.0) {
            goertzel_result_t g2100 = goertzel_analyze(samples + pos, P12_ANS_WINDOW_SAMPLES, sample_rate, 2100.0);
            goertzel_result_t g1800 = goertzel_analyze(samples + pos, P12_ANS_WINDOW_SAMPLES, sample_rate, 1800.0);
            goertzel_result_t g2400 = goertzel_analyze(samples + pos, P12_ANS_WINDOW_SAMPLES, sample_rate, 2400.0);
            goertzel_result_t g1650 = goertzel_analyze(samples + pos, P12_ANS_WINDOW_SAMPLES, sample_rate, 1650.0);
            goertzel_result_t g1850 = goertzel_analyze(samples + pos, P12_ANS_WINDOW_SAMPLES, sample_rate, 1850.0);
            r2100 = g2100.energy / (energy * (double) P12_ANS_WINDOW_SAMPLES);
            r1800 = g1800.energy / (energy * (double) P12_ANS_WINDOW_SAMPLES);
            r2400 = g2400.energy / (energy * (double) P12_ANS_WINDOW_SAMPLES);
            r1650 = g1650.energy / (energy * (double) P12_ANS_WINDOW_SAMPLES);
            r1850 = g1850.energy / (energy * (double) P12_ANS_WINDOW_SAMPLES);
            good = (r2100 >= 0.08
                    && r1800 < r2100 * 0.35
                    && r2400 < r2100 * 0.35
                    && r1650 < r2100 * 0.25
                    && r1850 < r2100 * 0.25);
        }

        if (good) {
            if (run_start < 0)
                run_start = pos;
            run_windows++;
            if (r2100 > run_peak)
                run_peak = r2100;
            continue;
        }

        if (run_start >= 0 && run_windows >= min_windows) {
            if (run_windows > best_windows || (run_windows == best_windows && run_peak > best_peak)) {
                best_start = run_start;
                best_windows = run_windows;
                best_peak = run_peak;
            }
        }
        run_start = -1;
        run_windows = 0;
        run_peak = 0.0;
    }

    if (run_start >= 0 && run_windows >= min_windows) {
        if (run_windows > best_windows || (run_windows == best_windows && run_peak > best_peak)) {
            best_start = run_start;
            best_windows = run_windows;
            best_peak = run_peak;
        }
    }

    if (best_start < 0)
        return false;

    out->detected = true;
    out->start_sample = best_start;
    out->duration_samples = (best_windows - 1) * P12_ANS_STEP_SAMPLES + P12_ANS_WINDOW_SAMPLES;
    out->peak_ratio = best_peak;
    if (p12_debug_enabled()) {
        fprintf(stderr,
                "[p12] answer tone candidate start=%.1fms dur=%.1fms peak=%.3f windows=%d\n",
                (double) out->start_sample * 1000.0 / (double) sample_rate,
                (double) out->duration_samples * 1000.0 / (double) sample_rate,
                out->peak_ratio,
                best_windows);
    }
    return true;
}

static bool detect_answer_tone_run_near(const int16_t *samples,
                                        int total_samples,
                                        int sample_rate,
                                        int anchor_sample,
                                        int min_run_ms,
                                        p12_tone_hit_t *out)
{
    int min_windows = (min_run_ms * sample_rate) / (1000 * P12_ANS_STEP_SAMPLES);
    int search_start;
    int search_end;
    int best_start = -1;
    int best_windows = 0;
    double best_peak = 0.0;
    int run_start = -1;
    int run_windows = 0;
    double run_peak = 0.0;

    if (!samples || !out || total_samples < P12_ANS_WINDOW_SAMPLES || anchor_sample < 0)
        return false;
    memset(out, 0, sizeof(*out));
    if (min_windows < 1)
        min_windows = 1;

    search_start = anchor_sample - (sample_rate * 4000) / 1000;
    search_end = anchor_sample + (sample_rate * 600) / 1000;
    if (search_start < 0)
        search_start = 0;
    if (search_end > total_samples)
        search_end = total_samples;

    for (int pos = search_start; pos + P12_ANS_WINDOW_SAMPLES <= search_end; pos += P12_ANS_STEP_SAMPLES) {
        double energy = window_energy(samples + pos, P12_ANS_WINDOW_SAMPLES);
        double r2100;
        double r1800;
        double r2400;
        double r1650;
        double r1850;
        bool good = false;

        if (energy > 0.0) {
            r2100 = tone_energy_ratio(samples + pos, P12_ANS_WINDOW_SAMPLES, sample_rate, 2100.0, energy);
            r1800 = tone_energy_ratio(samples + pos, P12_ANS_WINDOW_SAMPLES, sample_rate, 1800.0, energy);
            r2400 = tone_energy_ratio(samples + pos, P12_ANS_WINDOW_SAMPLES, sample_rate, 2400.0, energy);
            r1650 = tone_energy_ratio(samples + pos, P12_ANS_WINDOW_SAMPLES, sample_rate, 1650.0, energy);
            r1850 = tone_energy_ratio(samples + pos, P12_ANS_WINDOW_SAMPLES, sample_rate, 1850.0, energy);
            good = (r2100 >= 0.05
                    && r1800 < r2100 * 0.60
                    && r2400 < r2100 * 0.60
                    && r1650 < r2100 * 0.45
                    && r1850 < r2100 * 0.45);
        }

        if (good) {
            if (run_start < 0)
                run_start = pos;
            run_windows++;
            if (r2100 > run_peak)
                run_peak = r2100;
            continue;
        }

        if (run_start >= 0 && run_windows >= min_windows) {
            int run_end = run_start + (run_windows - 1) * P12_ANS_STEP_SAMPLES + P12_ANS_WINDOW_SAMPLES;

            if (run_end >= anchor_sample
                && (best_start < 0 || run_windows > best_windows || (run_windows == best_windows && run_peak > best_peak))) {
                best_start = run_start;
                best_windows = run_windows;
                best_peak = run_peak;
            }
        }
        run_start = -1;
        run_windows = 0;
        run_peak = 0.0;
    }

    if (run_start >= 0 && run_windows >= min_windows) {
        int run_end = run_start + (run_windows - 1) * P12_ANS_STEP_SAMPLES + P12_ANS_WINDOW_SAMPLES;

        if (run_end >= anchor_sample
            && (best_start < 0 || run_windows > best_windows || (run_windows == best_windows && run_peak > best_peak))) {
            best_start = run_start;
            best_windows = run_windows;
            best_peak = run_peak;
        }
    }

    if (best_start < 0)
        return false;

    out->detected = true;
    out->start_sample = best_start;
    out->duration_samples = (best_windows - 1) * P12_ANS_STEP_SAMPLES + P12_ANS_WINDOW_SAMPLES;
    out->peak_ratio = best_peak;
    return true;
}

static bool detect_answer_tone_fallback(const int16_t *samples,
                                        int total_samples,
                                        int sample_rate,
                                        int max_sample,
                                        p12_tone_hit_t *out)
{
    enum {
        WINDOW_SAMPLES = 160,
        MIN_RUN_WINDOWS = 40,
        MAX_GAP_WINDOWS = 4
    };
    static const double competitor_freqs[] = {
        1300.0, 1375.0, 1529.0, 1650.0, 2002.0, 2225.0, 2743.0, 3000.0, 3429.0
    };
    int limit;
    int run_start = -1;
    int run_windows = 0;
    int gap_windows = 0;
    double run_peak = 0.0;
    p12_tone_hit_t best;

    if (!samples || total_samples <= 0 || sample_rate <= 0 || !out)
        return false;

    memset(&best, 0, sizeof(best));
    memset(out, 0, sizeof(*out));

    limit = total_samples;
    if (max_sample > 0 && max_sample < limit)
        limit = max_sample;

    for (int start = 0; start + WINDOW_SAMPLES <= limit; start += WINDOW_SAMPLES) {
        double energy = window_energy(samples + start, WINDOW_SAMPLES);
        double ans_ratio;
        double competitor_peak = 0.0;
        bool strong_ans = false;

        if (energy > 0.0) {
            ans_ratio = tone_energy_ratio(samples + start,
                                          WINDOW_SAMPLES,
                                          sample_rate,
                                          2100.0,
                                          energy);
            for (size_t i = 0; i < sizeof(competitor_freqs)/sizeof(competitor_freqs[0]); i++) {
                double ratio = tone_energy_ratio(samples + start,
                                                 WINDOW_SAMPLES,
                                                 sample_rate,
                                                 competitor_freqs[i],
                                                 energy);
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
        }

        if (run_start >= 0 && gap_windows < MAX_GAP_WINDOWS) {
            gap_windows++;
            run_windows++;
            continue;
        }
        if (run_start >= 0 && run_windows >= MIN_RUN_WINDOWS) {
            best.detected = true;
            best.start_sample = run_start;
            best.duration_samples = run_windows * WINDOW_SAMPLES;
            best.peak_ratio = run_peak;
            break;
        }
        run_start = -1;
        run_windows = 0;
        gap_windows = 0;
        run_peak = 0.0;
    }

    if (!best.detected && run_start >= 0 && run_windows >= MIN_RUN_WINDOWS) {
        best.detected = true;
        best.start_sample = run_start;
        best.duration_samples = run_windows * WINDOW_SAMPLES;
        best.peak_ratio = run_peak;
    }

    if (!best.detected)
        return false;

    *out = best;
    return true;
}

static bool detect_phase_reversals_at_freq(const int16_t *samples,
                                           int len,
                                           int sample_rate,
                                           double freq_hz,
                                           int *reversal_count_out)
{
    enum { PHASE_WINDOW = 80 };
    int n_windows = len / PHASE_WINDOW;
    double prev_phase = 0.0;
    bool have_prev = false;
    int reversals = 0;

    for (int w = 0; w < n_windows; w++) {
        goertzel_result_t r = goertzel_analyze(samples + w * PHASE_WINDOW,
                                               PHASE_WINDOW,
                                               sample_rate,
                                               freq_hz);
        double phase;

        if (r.energy < 1e5)
            continue;

        phase = atan2(r.im, r.re);
        if (have_prev) {
            double diff = fabs(phase - prev_phase);
            if (diff > M_PI)
                diff = 2.0 * M_PI - diff;
            if (diff > 2.3 && diff < 3.9)
                reversals++;
        }
        prev_phase = phase;
        have_prev = true;
    }

    if (reversal_count_out)
        *reversal_count_out = reversals;
    return reversals >= 1;
}

static void detect_call_initiation_signals(const int16_t *samples,
                                           int total_samples,
                                           int sample_rate,
                                           int max_sample,
                                           phase12_result_t *result)
{
    int limit;
    v8bis_scan_result_t strong;
    v8bis_weak_candidate_t weak;
    call_log_t tmp_log;

    if (!samples || !result || sample_rate <= 0)
        return;

    limit = (max_sample > 0 && max_sample < total_samples) ? max_sample : total_samples;
    if (limit <= 0)
        return;
    if (result->answer_tone.detected && result->answer_tone.start_sample > 0 && result->answer_tone.start_sample < limit)
        limit = result->answer_tone.start_sample;
    if (limit > (sample_rate * P12_CALL_INIT_MAX_MS) / 1000)
        limit = (sample_rate * P12_CALL_INIT_MAX_MS) / 1000;
    if (limit <= 0)
        return;

    memset(&strong, 0, sizeof(strong));
    memset(&weak, 0, sizeof(weak));
    if (v8bis_scan_signals(samples, total_samples, limit, &strong)) {
        int best_idx = -1;
        double best_score = -1.0;
        for (int i = 0; i < V8BIS_NUM_SIGNALS; i++) {
            if (!strong.hits[i].seen)
                continue;
            if (strong.hits[i].score > best_score) {
                best_idx = i;
                best_score = strong.hits[i].score;
            }
        }
        if (best_idx >= 0) {
            result->call_init.v8bis_signal_seen = true;
            result->call_init.v8bis_signal_sample = strong.hits[best_idx].sample_offset;
            result->call_init.v8bis_signal_duration = strong.hits[best_idx].duration_samples;
            snprintf(result->call_init.v8bis_signal_name,
                     sizeof(result->call_init.v8bis_signal_name),
                     "%s",
                     g_v8bis_signal_defs[best_idx].name);
        }
    } else if (v8bis_scan_weak_candidate(samples, total_samples, limit, &weak)) {
        result->call_init.v8bis_signal_seen = true;
        result->call_init.v8bis_signal_weak = true;
        result->call_init.v8bis_signal_sample = weak.sample_offset;
        result->call_init.v8bis_signal_duration = weak.duration_samples;
        snprintf(result->call_init.v8bis_signal_name,
                 sizeof(result->call_init.v8bis_signal_name),
                 "%s",
                 g_v8bis_signal_defs[weak.signal_index].name);
    }

    memset(&tmp_log, 0, sizeof(tmp_log));
    v8bis_collect_msg_events(&tmp_log, samples, total_samples, limit);
    for (size_t i = 0; i < tmp_log.count; i++) {
        const call_log_event_t *ev = &tmp_log.events[i];
        if (strcmp(ev->protocol, "V.92") != 0)
            continue;
        if (!(strcmp(ev->summary, "QC2a") == 0
              || strcmp(ev->summary, "QCA2a") == 0
              || strcmp(ev->summary, "QC2d") == 0
              || strcmp(ev->summary, "QCA2d") == 0)) {
            continue;
        }
        result->call_init.v92_qc2_seen = true;
        result->call_init.v92_qc2_sample = ev->sample_offset;
        snprintf(result->call_init.v92_qc2_name,
                 sizeof(result->call_init.v92_qc2_name),
                 "%s",
                 ev->summary);
        break;
    }
    free(tmp_log.events);
    tmp_log.events = NULL;
    tmp_log.count = 0;
    tmp_log.cap = 0;

    if (result->answer_tone.detected) {
        result->call_init.answer_tone_handoff_known = true;
        result->call_init.answer_tone_handoff_sample =
            result->answer_tone.start_sample + result->answer_tone.duration_samples;
    }
}

static bool detect_phase2_signal_tone_runs(const int16_t *samples,
                                           int total_samples,
                                           int sample_rate,
                                           int search_start,
                                           int search_end,
                                           p12_phase2_role_t role,
                                           p12_signal_tone_hit_t *tone_a,
                                           p12_signal_tone_hit_t *tone_b)
{
    p12_tone_hit_t hit = {0};
    p12_signal_tone_hit_t best_a = {0};
    p12_signal_tone_hit_t best_b = {0};
    const double *freqs = NULL;
    const bool *is_tone_a = NULL;
    int freq_count = 0;
    static const double caller_like_freqs[] = { 2400.0, 2100.0, 1800.0, 1200.0 };
    static const bool caller_like_types[] = { true, true, true, false };
    static const double answerer_like_freqs[] = { 1200.0, 2100.0, 1800.0, 2400.0 };
    static const bool answerer_like_types[] = { false, false, false, true };

    if (!samples || !tone_a || !tone_b)
        return false;
    memset(tone_a, 0, sizeof(*tone_a));
    memset(tone_b, 0, sizeof(*tone_b));
    if (search_start < 0)
        search_start = 0;
    if (search_end <= search_start || search_end > total_samples)
        search_end = total_samples;

    switch (role) {
    case P12_PHASE2_ROLE_V34_CALLER:
    case P12_PHASE2_ROLE_V90_DIGITAL_ANSWERER:
        freqs = caller_like_freqs;
        is_tone_a = caller_like_types;
        freq_count = 4;
        break;
    case P12_PHASE2_ROLE_V34_ANSWERER:
    case P12_PHASE2_ROLE_V90_ANALOG_CALLER:
        freqs = answerer_like_freqs;
        is_tone_a = answerer_like_types;
        freq_count = 4;
        break;
    default:
        freqs = caller_like_freqs;
        is_tone_a = caller_like_types;
        freq_count = 4;
        break;
    }

    for (int i = 0; i < freq_count; i++) {
        memset(&hit, 0, sizeof(hit));
        if (!p12_detect_phase2_cc_run(samples + search_start,
                                      search_end - search_start,
                                      sample_rate,
                                      freqs[i],
                                      P12_PHASE2_SIGNAL_MIN_MS,
                                      &hit)) {
            continue;
        }
        if (is_tone_a[i]) {
            p12_signal_tone_hit_t candidate;

            memset(&candidate, 0, sizeof(candidate));
            candidate.detected = true;
            candidate.start_sample = search_start + hit.start_sample;
            candidate.duration_samples = hit.duration_samples;
            candidate.freq_hz = freqs[i];
            candidate.phase_reversal_seen = detect_phase_reversals_at_freq(samples + candidate.start_sample,
                                                                           candidate.duration_samples,
                                                                           sample_rate,
                                                                           freqs[i],
                                                                           NULL);
            if (candidate.phase_reversal_seen)
                candidate.reversal_sample = candidate.start_sample + candidate.duration_samples/2;
            if (!best_a.detected
                || candidate.start_sample < best_a.start_sample
                || (candidate.start_sample == best_a.start_sample
                    && candidate.duration_samples > best_a.duration_samples)) {
                best_a = candidate;
            }
        } else {
            p12_signal_tone_hit_t candidate;

            memset(&candidate, 0, sizeof(candidate));
            candidate.detected = true;
            candidate.start_sample = search_start + hit.start_sample;
            candidate.duration_samples = hit.duration_samples;
            candidate.freq_hz = freqs[i];
            candidate.phase_reversal_seen = detect_phase_reversals_at_freq(samples + candidate.start_sample,
                                                                           candidate.duration_samples,
                                                                           sample_rate,
                                                                           freqs[i],
                                                                           NULL);
            if (candidate.phase_reversal_seen)
                candidate.reversal_sample = candidate.start_sample + candidate.duration_samples/2;
            if (!best_b.detected
                || candidate.start_sample < best_b.start_sample
                || (candidate.start_sample == best_b.start_sample
                    && candidate.duration_samples > best_b.duration_samples)) {
                best_b = candidate;
            }
        }
    }
    *tone_a = best_a;
    *tone_b = best_b;
    return tone_a->detected || tone_b->detected;
}

static int p12_phase2_signal_tone_score(const p12_signal_tone_hit_t *tone_a,
                                        const p12_signal_tone_hit_t *tone_b)
{
    int score = 0;

    if (tone_a && tone_a->detected) {
        score += 4;
        if (tone_a->phase_reversal_seen)
            score += 1;
        if (tone_a->freq_hz == 2400.0 || tone_a->freq_hz == 1200.0)
            score += 1;
    }
    if (tone_b && tone_b->detected) {
        score += 4;
        if (tone_b->phase_reversal_seen)
            score += 1;
        if (tone_b->freq_hz == 2400.0 || tone_b->freq_hz == 1200.0)
            score += 1;
    }
    return score;
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
            int run_end = run_start + (run_windows - 1) * TONE_STEP_SAMPLES + TONE_WINDOW_SAMPLES;
            if (!out->detected) {
                out->detected = true;
                out->start_sample = run_start;
            }
            out->duration_samples = run_end - out->start_sample;
            if (run_peak > out->peak_ratio)
                out->peak_ratio = run_peak;
            /* Non-cadenced: return on first qualifying run */
            if (!require_cadence)
                return true;
        } else if (out->detected && require_cadence) {
            /* Check whether the gap since the last confirmed run end is too
             * long to be a cadence interval (CNG repeats every ~3.5 s). */
            int gap_samples = pos - (out->start_sample + out->duration_samples);
            if (gap_samples > (4000 * sample_rate) / 1000)
                return true; /* cadence ended */
        }
        run_start = -1;
        run_windows = 0;
        run_peak = 0.0;
    }

    /* Check trailing run */
    if (run_start >= 0 && run_windows >= min_run_windows) {
        int run_end = run_start + (run_windows - 1) * TONE_STEP_SAMPLES + TONE_WINDOW_SAMPLES;
        if (!out->detected) {
            out->detected = true;
            out->start_sample = run_start;
        }
        out->duration_samples = run_end - out->start_sample;
        if (run_peak > out->peak_ratio)
            out->peak_ratio = run_peak;
    }

    return out->detected;
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
    call_init_tone_probe_t early_probe;
    static const double cng_competitors[] = { 980.0, 1180.0, 1300.0 };
    static const double ct_competitors[] = { 1100.0, 1180.0, 980.0 };

    memset(&early_probe, 0, sizeof(early_probe));
    call_init_collect_tones(samples, total_samples, limit, &early_probe);

    /* CNG: 1100 Hz */
    if (detect_tone(samples, limit, sample_rate, CNG_FREQ_HZ,
                    cng_competitors, 3, MIN_CNG_RUN_MS, true, &result->cng)) {
        result->cng.type = P12_TONE_CNG;
    } else if (early_probe.cng_sample >= 0) {
        result->cng.detected = true;
        result->cng.type = P12_TONE_CNG;
        result->cng.start_sample = early_probe.cng_sample;
    }

    /* CT: 1300 Hz */
    if (detect_tone(samples, limit, sample_rate, CT_FREQ_HZ,
                    ct_competitors, 3, MIN_CT_RUN_MS, false, &result->ct)) {
        result->ct.type = P12_TONE_CT;
    } else if (early_probe.ct_sample >= 0) {
        result->ct.detected = true;
        result->ct.type = P12_TONE_CT;
        result->ct.start_sample = early_probe.ct_sample;
    }

    /* ANS/ANSam: 2100 Hz */
    if ((early_probe.ans_sample >= 0
         && detect_answer_tone_run_near(samples,
                                        limit,
                                        sample_rate,
                                        early_probe.ans_sample,
                                        MIN_ANS_RUN_MS,
                                        &result->answer_tone))
        || detect_answer_tone_fallback(samples, limit, sample_rate,
                                       limit, &result->answer_tone)
        || detect_answer_tone_run(samples, limit, sample_rate,
                                  MIN_ANS_RUN_MS, &result->answer_tone)) {
        /* Classify: check for AM modulation and phase reversals */
        int ans_start = result->answer_tone.start_sample;
        int ans_len = result->answer_tone.duration_samples;
        int reversal_count = 0;
        bool has_am = detect_am_15hz(samples + ans_start, ans_len, sample_rate);
        bool has_pr = detect_phase_reversals(samples + ans_start, ans_len,
                                             sample_rate, &reversal_count);

        if (early_probe.ans_tone == MODEM_CONNECT_TONES_ANSAM_PR)
            result->answer_tone.type = P12_TONE_ANSAM_PR;
        else if (early_probe.ans_tone == MODEM_CONNECT_TONES_ANSAM)
            result->answer_tone.type = P12_TONE_ANSAM;
        else if (early_probe.ans_tone == MODEM_CONNECT_TONES_ANS_PR)
            result->answer_tone.type = P12_TONE_ANS_PR;
        else if (early_probe.ans_tone == MODEM_CONNECT_TONES_ANS)
            result->answer_tone.type = P12_TONE_ANS;
        else if (has_am && has_pr)
            result->answer_tone.type = P12_TONE_ANSAM_PR;
        else if (has_am)
            result->answer_tone.type = P12_TONE_ANSAM;
        else if (has_pr)
            result->answer_tone.type = P12_TONE_ANS_PR;
        else
            result->answer_tone.type = P12_TONE_ANS;
    } else if (early_probe.ans_sample >= 0) {
        result->answer_tone.detected = true;
        result->answer_tone.start_sample = early_probe.ans_sample;
        result->answer_tone.duration_samples = 0;
        result->answer_tone.type =
            (early_probe.ans_tone == MODEM_CONNECT_TONES_ANSAM_PR) ? P12_TONE_ANSAM_PR :
            (early_probe.ans_tone == MODEM_CONNECT_TONES_ANSAM) ? P12_TONE_ANSAM :
            (early_probe.ans_tone == MODEM_CONNECT_TONES_ANS_PR) ? P12_TONE_ANS_PR :
            P12_TONE_ANS;
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
    double run_peak_signal = 0.0;
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
                run_peak_signal = 0.0;
            }
            run_windows++;
            if (strength > run_peak)
                run_peak = strength;
            if (energy > run_peak_signal)
                run_peak_signal = energy;
            continue;
        }

fsk_gap:
        if (run_start >= 0 && run_windows >= FSK_BURST_MIN_WINDOWS) {
            p12_fsk_burst_t candidate = {0};
            candidate.seen = true;
            candidate.start_sample = run_start;
            candidate.duration_samples = (run_windows - 1) * FSK_BURST_STEP + FSK_BURST_WINDOW;
            candidate.peak_energy = run_peak;
            candidate.peak_signal_energy = run_peak_signal;

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
                if (candidate.peak_signal_energy > bursts[count - 1].peak_signal_energy)
                    bursts[count - 1].peak_signal_energy = candidate.peak_signal_energy;
            } else if (count < max_bursts) {
                bursts[count++] = candidate;
            }
        }
        run_start = -1;
        run_windows = 0;
        run_peak = 0.0;
        run_peak_signal = 0.0;
    }

    /* Flush trailing run */
    if (run_start >= 0 && run_windows >= FSK_BURST_MIN_WINDOWS && count < max_bursts) {
        bursts[count].seen = true;
        bursts[count].start_sample = run_start;
        bursts[count].duration_samples = (run_windows - 1) * FSK_BURST_STEP + FSK_BURST_WINDOW;
        bursts[count].peak_energy = run_peak;
        bursts[count].peak_signal_energy = run_peak_signal;
        count++;
    }

    return count;
}

static int p12_prune_low_energy_bursts_in_place(p12_fsk_burst_t *bursts,
                                                int burst_count,
                                                double relative_cutoff,
                                                const char *label,
                                                int sample_rate)
{
    double strongest = 0.0;
    int write_idx = 0;

    if (!bursts || burst_count <= 1 || relative_cutoff <= 0.0)
        return burst_count;

    for (int i = 0; i < burst_count; i++) {
        if (bursts[i].peak_signal_energy > strongest)
            strongest = bursts[i].peak_signal_energy;
    }
    if (strongest <= 0.0)
        return burst_count;

    for (int i = 0; i < burst_count; i++) {
        double rel = bursts[i].peak_signal_energy / strongest;

        if (rel + 1e-12 < relative_cutoff) {
            if (p12_debug_enabled()) {
                fprintf(stderr,
                        "[p12] pruned %s burst start=%.1fms dur=%.1fms rel_sig=%.3f cutoff=%.3f\n",
                        label ? label : "?",
                        (double) bursts[i].start_sample * 1000.0 / (double) sample_rate,
                        (double) bursts[i].duration_samples * 1000.0 / (double) sample_rate,
                        rel,
                        relative_cutoff);
            }
            continue;
        }
        if (write_idx != i)
            bursts[write_idx] = bursts[i];
        write_idx++;
    }

    for (int i = write_idx; i < burst_count; i++)
        memset(&bursts[i], 0, sizeof(*bursts));
    return write_idx;
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

static bool p12_find_cj_zero_octets(const uint8_t *bits,
                                    int bit_count,
                                    int start_bit,
                                    int *cj_start_bit_out,
                                    int *cj_end_bit_out)
{
    if (!bits || bit_count < 30)
        return false;
    if (start_bit < 0)
        start_bit = 0;

    for (int pos = start_bit; pos + 30 <= bit_count; pos++) {
        bool ok = true;

        for (int octet = 0; octet < 3 && ok; octet++) {
            int base = pos + octet * 10;

            /* Async 0x00 octet: start=0, 8 zero data bits, stop=1 */
            if (bits[base] != 0 || bits[base + 9] != 1) {
                ok = false;
                break;
            }
            for (int j = 1; j <= 8; j++) {
                if (bits[base + j] != 0) {
                    ok = false;
                    break;
                }
            }
        }

        if (ok) {
            if (cj_start_bit_out)
                *cj_start_bit_out = pos;
            if (cj_end_bit_out)
                *cj_end_bit_out = pos + 30;
            return true;
        }
    }

    return false;
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

static bool p12_cm_jm_bytes_equal(const p12_cm_jm_hit_t *msg,
                                  const uint8_t *bytes,
                                  int byte_count)
{
    if (!msg || !bytes || byte_count < 0)
        return false;
    if (msg->byte_count != byte_count)
        return false;
    if (byte_count == 0)
        return true;
    return memcmp(msg->bytes, bytes, (size_t) byte_count) == 0;
}

static void p12_record_cm_jm_observation(p12_cm_jm_hit_t *dst,
                                         const p12_cm_jm_hit_t *candidate)
{
    if (!dst || !candidate || !candidate->detected)
        return;

    if (!dst->detected) {
        *dst = *candidate;
        dst->observed_count = 1;
        dst->differing_count = 0;
        dst->saved_count = 1;
        dst->saved_sample_offsets[0] = candidate->sample_offset;
        dst->saved_byte_counts[0] = candidate->byte_count;
        if (candidate->byte_count > 0) {
            memcpy(dst->saved_bytes[0],
                   candidate->bytes,
                   (size_t) candidate->byte_count);
        }
        dst->last_sample_offset = candidate->sample_offset;
        dst->last_duration_samples = candidate->duration_samples;
        dst->complete = false;
        return;
    }

    dst->observed_count++;
    dst->last_sample_offset = candidate->sample_offset;
    dst->last_duration_samples = candidate->duration_samples;

    if (!p12_cm_jm_bytes_equal(dst, candidate->bytes, candidate->byte_count))
        dst->differing_count++;

    if (dst->saved_count < (int) (sizeof(dst->saved_sample_offsets) / sizeof(dst->saved_sample_offsets[0]))) {
        int idx = dst->saved_count++;
        dst->saved_sample_offsets[idx] = candidate->sample_offset;
        dst->saved_byte_counts[idx] = candidate->byte_count;
        if (candidate->byte_count > 0) {
            memcpy(dst->saved_bytes[idx],
                   candidate->bytes,
                   (size_t) candidate->byte_count);
        }
    }

    /* Keep the parsed fields from the most recent valid decode, while the
       original bytes remain the reference message for repeat comparison. */
    dst->sample_offset = candidate->sample_offset;
    dst->duration_samples = candidate->duration_samples;
    dst->call_function = candidate->call_function;
    dst->modulations = candidate->modulations;
    dst->protocols = candidate->protocols;
    dst->pstn_access = candidate->pstn_access;
    dst->pcm_modem_availability = candidate->pcm_modem_availability;
    dst->complete = (dst->observed_count >= 2 && dst->differing_count == 0);
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
        int pre_pad, post_pad;

        if (!bursts[b].seen)
            continue;

        start = bursts[b].start_sample;
        len = bursts[b].duration_samples;
        pre_pad = (sample_rate * P12_PHASE1_PRE_PAD_MS) / 1000;
        post_pad = (sample_rate * P12_PHASE1_POST_PAD_MS) / 1000;
        if (start > pre_pad) {
            start -= pre_pad;
            len += pre_pad;
        } else {
            len += start;
            start = 0;
        }
        if (start + len + post_pad <= total_samples)
            len += post_pad;
        else
            len = total_samples - start;
        if (start < 0 || start + len > total_samples)
            continue;

        /* Try both polarities */
        for (int inv = 0; inv < 2; inv++) {
            int message_end_bit = -1;
            bool found_message_this_pass = false;

            v8_msg_decoder_init(&decoder);

            v21_fsk_demod_block(samples + start, len, sample_rate,
                                channel, (bool)inv,
                                v8_msg_decoder_put_bit, &decoder);

            /* Try extracting CM/JM bytes from raw bits using async framing */
            if (cm_jm_out && decoder.raw_bit_count >= 20) {
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
                            candidate.complete = false;
                            candidate.sample_offset = start;
                            candidate.duration_samples = len;
                            p12_record_cm_jm_observation(cm_jm_out, &candidate);
                            message_end_bit = scan_pos;
                            found_message_this_pass = true;
                            bit_pos = scan_pos - 1;
                        }
                    }
                }
            }

            /* Check for CJ only after a valid CM/JM path exists.
             * This prevents stray all-mark runs from being labeled as CJ
             * before negotiation has actually been reconstructed. */
            if (cj_out && !cj_out->detected
                && cm_jm_out && cm_jm_out->detected
                && decoder.raw_bit_count >= 30) {
                int samp_per_bit = sample_rate / 300;
                int search_bit = 0;
                int cj_start_bit = -1;
                int cj_end_bit = -1;

                if (message_end_bit > 0)
                    search_bit = message_end_bit;
                if (p12_find_cj_zero_octets(decoder.raw_bits,
                                            decoder.raw_bit_count,
                                            search_bit,
                                            &cj_start_bit,
                                            &cj_end_bit)) {
                    int offset = start + cj_start_bit * samp_per_bit;
                    int duration = (cj_end_bit - cj_start_bit) * samp_per_bit;

                    cj_out->detected = true;
                    if (offset < 0) offset = 0;
                    if (duration < samp_per_bit)
                        duration = samp_per_bit;
                    cj_out->sample_offset = offset;
                    cj_out->duration_samples = duration;
                    if (p12_debug_enabled()) {
                        fprintf(stderr,
                                "[p12] CJ zero-octet sequence start=%.1fms dur=%.1fms\n",
                                (double) cj_out->sample_offset * 1000.0 / (double) sample_rate,
                                (double) cj_out->duration_samples * 1000.0 / (double) sample_rate);
                    }
                }
            }

            if (cj_out && cj_out->detected)
                break; /* CJ ends the repeating CM/JM phase */
            if (found_message_this_pass)
                continue; /* keep decoding repeats until CJ is found */
        }
        if (cj_out && cj_out->detected)
            break;
    }
}

static void detect_phase1_v8(const int16_t *samples,
                             int total_samples,
                             int sample_rate,
                             int max_sample,
                             phase12_result_t *result)
{
    p12_fsk_burst_t phase1_ch1_windows[P12_MAX_FSK_BURSTS];
    p12_fsk_burst_t phase1_ch2_windows[P12_MAX_FSK_BURSTS];
    int phase1_merge_gap;
    int phase1_ch1_window_count;
    int phase1_ch2_window_count;
    int limit = (max_sample > 0 && max_sample < total_samples) ? max_sample : total_samples;

    /* Find V.21 FSK bursts on both channels */
    result->ch1_burst_count = detect_fsk_bursts(samples, total_samples, sample_rate,
                                                 0, limit, V21_CH1,
                                                 result->ch1_bursts, P12_MAX_FSK_BURSTS);
    result->ch2_burst_count = detect_fsk_bursts(samples, total_samples, sample_rate,
                                                 0, limit, V21_CH2,
                                                 result->ch2_bursts, P12_MAX_FSK_BURSTS);

    memcpy(phase1_ch1_windows, result->ch1_bursts, sizeof(phase1_ch1_windows));
    memcpy(phase1_ch2_windows, result->ch2_bursts, sizeof(phase1_ch2_windows));
    phase1_merge_gap = (sample_rate * P12_PHASE1_REPEAT_GAP_MS) / 1000;
    phase1_ch1_window_count = p12_merge_bursts_in_place(phase1_ch1_windows,
                                                        result->ch1_burst_count,
                                                        phase1_merge_gap);
    phase1_ch2_window_count = p12_merge_bursts_in_place(phase1_ch2_windows,
                                                        result->ch2_burst_count,
                                                        phase1_merge_gap);
    if (p12_debug_enabled()) {
        p12_debug_log_bursts("phase1 CH1 windows", phase1_ch1_windows, phase1_ch1_window_count, sample_rate);
        p12_debug_log_bursts("phase1 CH2 windows", phase1_ch2_windows, phase1_ch2_window_count, sample_rate);
    }

    /* Decode V.8 messages from CH1 repeat windows (CM from caller). */
    decode_v8_fsk_channel(samples, total_samples, sample_rate,
                          V21_CH1, phase1_ch1_windows, phase1_ch1_window_count,
                          &result->cm, NULL);

    /* Decode V.8 messages from CH2 repeat windows (JM from answerer, CJ). */
    decode_v8_fsk_channel(samples, total_samples, sample_rate,
                          V21_CH2, phase1_ch2_windows, phase1_ch2_window_count,
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
    bool frame_candidate_seen;
    uint8_t candidate_bytes[V34_INFO_MAX_BUF_BYTES];
    v34_info_frame_t frame;
    int frame_sample_offset;
    int candidate_sample_offset;
    uint16_t candidate_crc;
} info_frame_decoder_t;

static void p12_build_info_frame_from_payload(v34_info_frame_t *frame,
                                              const uint8_t *payload,
                                              int target_bits);

static void p12_pack_lsb_bits(uint8_t *dst, int dst_len, const uint8_t *bits, int bit_count)
{
    if (!dst || dst_len <= 0)
        return;
    memset(dst, 0, (size_t) dst_len);
    if (!bits || bit_count <= 0)
        return;
    for (int i = 0; i < bit_count; i++) {
        if (bits[i] && (i >> 3) < dst_len)
            dst[i >> 3] |= (uint8_t) (1U << (i & 7));
    }
}

static void p12_pack_msb_bits(uint8_t *dst, int dst_len, const uint8_t *bits, int bit_count)
{
    if (!dst || dst_len <= 0)
        return;
    memset(dst, 0, (size_t) dst_len);
    if (!bits || bit_count <= 0)
        return;
    for (int i = 0; i < bit_count; i++) {
        if (bits[i] && (i >> 3) < dst_len)
            dst[i >> 3] |= (uint8_t) (1U << (7 - (i & 7)));
    }
}

static void p12_debug_dump_payload_bytes(const char *label,
                                         const uint8_t *bits,
                                         int bit_count)
{
    uint8_t lsb[V34_INFO_MAX_BUF_BYTES];
    uint8_t msb[V34_INFO_MAX_BUF_BYTES];
    int byte_count;

    if (!p12_debug_enabled() || !bits || bit_count <= 0)
        return;

    byte_count = (bit_count + 7) / 8;
    if (byte_count > V34_INFO_MAX_BUF_BYTES)
        byte_count = V34_INFO_MAX_BUF_BYTES;
    p12_pack_lsb_bits(lsb, (int) sizeof(lsb), bits, bit_count);
    p12_pack_msb_bits(msb, (int) sizeof(msb), bits, bit_count);

    fprintf(stderr, "[p12] %s bytes-lsb=", label ? label : "payload");
    for (int i = 0; i < byte_count; i++)
        fprintf(stderr, "%s%02x", (i == 0) ? "" : " ", lsb[i]);
    fprintf(stderr, " bytes-msb=");
    for (int i = 0; i < byte_count; i++)
        fprintf(stderr, "%s%02x", (i == 0) ? "" : " ", msb[i]);
    fprintf(stderr, "\n");
}

static bool p12_try_payload_pack_variants(v34_info_frame_t *frame_out,
                                          const uint8_t *payload_bits,
                                          int target_bits)
{
    uint8_t payload_lsb[V34_INFO_MAX_BUF_BYTES];
    uint8_t payload_msb[V34_INFO_MAX_BUF_BYTES];

    if (!frame_out || !payload_bits || target_bits <= 0)
        return false;

    p12_pack_lsb_bits(payload_lsb, (int) sizeof(payload_lsb), payload_bits, target_bits);
    if (v34_info_validate_frame_bytes(payload_lsb, target_bits)) {
        p12_build_info_frame_from_payload(frame_out, payload_lsb, target_bits);
        return true;
    }

    p12_pack_msb_bits(payload_msb, (int) sizeof(payload_msb), payload_bits, target_bits);
    if (v34_info_validate_frame_bytes(payload_msb, target_bits)) {
        if (p12_debug_enabled())
            fprintf(stderr, "[p12] INFO pack-variant recovery kind=msb target=%d\n", target_bits);
        p12_build_info_frame_from_payload(frame_out, payload_msb, target_bits);
        return true;
    }

    for (int i = 0; i < (target_bits + 7) / 8 && i < V34_INFO_MAX_BUF_BYTES; i++)
        payload_msb[i] = bit_reverse8(payload_lsb[i]);
    if (v34_info_validate_frame_bytes(payload_msb, target_bits)) {
        if (p12_debug_enabled())
            fprintf(stderr, "[p12] INFO pack-variant recovery kind=byte-reverse target=%d\n", target_bits);
        p12_build_info_frame_from_payload(frame_out, payload_msb, target_bits);
        return true;
    }

    return false;
}

static bool p12_try_soft_bit_flips(v34_info_frame_t *frame_out,
                                   const uint8_t *base_bits,
                                   const double *confidences,
                                   int target_bits,
                                   int *flip_a_out,
                                   int *flip_b_out)
{
    enum { P12_SOFT_FLIP_LIMIT = 10 };
    uint8_t trial_bits[256];
    uint8_t payload[V34_INFO_MAX_BUF_BYTES];
    int low_idx[P12_SOFT_FLIP_LIMIT];
    int low_count = 0;

    if (!frame_out || !base_bits || !confidences || target_bits <= 0 || target_bits > 256)
        return false;

    for (int i = 0; i < target_bits; i++) {
        double conf = confidences[i];
        int insert_at = low_count;

        if (insert_at < P12_SOFT_FLIP_LIMIT) {
            low_idx[insert_at] = i;
            low_count++;
        } else if (conf >= confidences[low_idx[P12_SOFT_FLIP_LIMIT - 1]]) {
            continue;
        } else {
            low_idx[P12_SOFT_FLIP_LIMIT - 1] = i;
            insert_at = P12_SOFT_FLIP_LIMIT - 1;
        }

        while (insert_at > 0 && confidences[low_idx[insert_at - 1]] > confidences[low_idx[insert_at]]) {
            int tmp = low_idx[insert_at - 1];
            low_idx[insert_at - 1] = low_idx[insert_at];
            low_idx[insert_at] = tmp;
            insert_at--;
        }
    }

    memcpy(trial_bits, base_bits, (size_t) target_bits);
    for (int i = 0; i < low_count; i++) {
        int a = low_idx[i];

        memcpy(trial_bits, base_bits, (size_t) target_bits);
        trial_bits[a] ^= 1U;
        p12_pack_lsb_bits(payload, (int) sizeof(payload), trial_bits, target_bits);
        if (v34_info_validate_frame_bytes(payload, target_bits)) {
            p12_build_info_frame_from_payload(frame_out, payload, target_bits);
            if (flip_a_out)
                *flip_a_out = a;
            if (flip_b_out)
                *flip_b_out = -1;
            return true;
        }
    }

    for (int i = 0; i < low_count; i++) {
        int a = low_idx[i];

        for (int j = i + 1; j < low_count; j++) {
            int b = low_idx[j];

            memcpy(trial_bits, base_bits, (size_t) target_bits);
            trial_bits[a] ^= 1U;
            trial_bits[b] ^= 1U;
            p12_pack_lsb_bits(payload, (int) sizeof(payload), trial_bits, target_bits);
            if (v34_info_validate_frame_bytes(payload, target_bits)) {
                p12_build_info_frame_from_payload(frame_out, payload, target_bits);
                if (flip_a_out)
                    *flip_a_out = a;
                if (flip_b_out)
                    *flip_b_out = b;
                return true;
            }
        }
    }

    return false;
}

static double p12_v21_symbol_decision_value(const int16_t *samples,
                                            int total_samples,
                                            int sample_rate,
                                            int channel,
                                            int offset,
                                            double symbol_samples,
                                            bool invert,
                                            double *mark_energy_out,
                                            double *space_energy_out,
                                            bool *ok_out)
{
    double mark_hz = 980.0;
    double space_hz = 1180.0;
    double total_energy;
    int symbol_len;
    goertzel_result_t mark_r;
    goertzel_result_t space_r;
    double decision;

    if (ok_out)
        *ok_out = false;
    if (mark_energy_out)
        *mark_energy_out = 0.0;
    if (space_energy_out)
        *space_energy_out = 0.0;
    if (!samples || total_samples <= 0 || sample_rate <= 0 || offset < 0)
        return 0.0;

    if (channel == V21_CH2) {
        mark_hz = 1650.0;
        space_hz = 1850.0;
    }

    symbol_len = (int) floor(symbol_samples + 0.5);
    if (symbol_len <= 0 || offset + symbol_len > total_samples)
        return 0.0;

    mark_r = goertzel_analyze(samples + offset, symbol_len, sample_rate, mark_hz);
    space_r = goertzel_analyze(samples + offset, symbol_len, sample_rate, space_hz);
    if (mark_energy_out)
        *mark_energy_out = mark_r.energy;
    if (space_energy_out)
        *space_energy_out = space_r.energy;
    total_energy = mark_r.energy + space_r.energy;
    if (total_energy <= 1e-12)
        decision = 0.0;
    else
        decision = (mark_r.energy - space_r.energy) / total_energy;
    if (invert)
        decision = -decision;
    if (ok_out)
        *ok_out = true;
    return decision;
}

static void p12_debug_dump_symbol_trace(const int16_t *samples,
                                        int total_samples,
                                        int sample_rate,
                                        int channel,
                                        int sync_start,
                                        int target_bits,
                                        bool invert,
                                        double symbol_samples,
                                        double bias)
{
    int dump_sync = V34_INFO_SYNC_BITS;
    int dump_payload = (target_bits < 12) ? target_bits : 12;

    if (!p12_debug_enabled())
        return;

    fprintf(stderr,
            "[p12] INFO symbol-trace channel=%d sync=%.1fms invert=%u bias=%.6f\n",
            channel,
            (double) sync_start * 1000.0 / (double) sample_rate,
            invert ? 1U : 0U,
            bias);
    for (int i = 0; i < dump_sync; i++) {
        bool ok = false;
        double mark_energy = 0.0;
        double space_energy = 0.0;
        double decision;
        int offset = sync_start + (int) floor((double) i * symbol_samples + 0.5);
        int expected_bit = (V34_INFO_SYNC_CODE >> (V34_INFO_SYNC_BITS - 1 - i)) & 1;

        decision = p12_v21_symbol_decision_value(samples,
                                                 total_samples,
                                                 sample_rate,
                                                 channel,
                                                 offset,
                                                 symbol_samples,
                                                 invert,
                                                 &mark_energy,
                                                 &space_energy,
                                                 &ok);
        if (!ok)
            break;
        fprintf(stderr,
                "[p12]   sync[%02d] off=%.1fms exp=%d mark=%.3e space=%.3e dec=%.6f slice=%.6f\n",
                i,
                (double) offset * 1000.0 / (double) sample_rate,
                expected_bit,
                mark_energy,
                space_energy,
                decision,
                decision + bias);
    }
    for (int i = 0; i < dump_payload; i++) {
        bool ok = false;
        double mark_energy = 0.0;
        double space_energy = 0.0;
        double decision;
        int offset = sync_start + (int) floor((double) (V34_INFO_SYNC_BITS + i) * symbol_samples + 0.5);
        int bit;

        decision = p12_v21_symbol_decision_value(samples,
                                                 total_samples,
                                                 sample_rate,
                                                 channel,
                                                 offset,
                                                 symbol_samples,
                                                 invert,
                                                 &mark_energy,
                                                 &space_energy,
                                                 &ok);
        if (!ok)
            break;
        bit = (decision + bias >= 0.0) ? 1 : 0;
        fprintf(stderr,
                "[p12]   data[%02d] off=%.1fms bit=%d mark=%.3e space=%.3e dec=%.6f slice=%.6f\n",
                i,
                (double) offset * 1000.0 / (double) sample_rate,
                bit,
                mark_energy,
                space_energy,
                decision,
                decision + bias);
    }
}

static bool p12_try_sync_trained_info_redecode(const int16_t *samples,
                                               int total_samples,
                                               int sample_rate,
                                               int channel,
                                               int start,
                                               int phase_offset,
                                               int target_bits,
                                               bool invert,
                                               double symbol_samples,
                                               int candidate_sample_offset,
                                               v34_info_frame_t *frame_out,
                                               int *frame_sample_out)
{
    double best_sync_score = -1e300;
    double best_bias = 0.0;
    int best_sync_start = -1;
    double top_scores[3] = { -1e300, -1e300, -1e300 };
    int top_starts[3] = { -1, -1, -1 };
    int coarse_step;
    int fine_step;

    if (!samples || !frame_out || target_bits <= 0 || symbol_samples <= 0.0)
        return false;

    coarse_step = (int) floor(symbol_samples / 4.0 + 0.5);
    fine_step = 1;
    if (coarse_step < 1)
        coarse_step = 1;

    for (int pass = 0; pass < 2; pass++) {
        int delta_min;
        int delta_max;
        int delta_step;

        if (pass == 0) {
            delta_min = -2 * coarse_step;
            delta_max = 2 * coarse_step;
            delta_step = coarse_step;
        } else {
            delta_min = -coarse_step;
            delta_max = coarse_step;
            delta_step = fine_step;
        }

        for (int delta = delta_min; delta <= delta_max; delta += delta_step) {
            double lower_bias = -1e300;
            double upper_bias = 1e300;
            double sync_score = 0.0;
            int sync_start;
            bool have_lower = false;
            bool have_upper = false;
            bool valid = true;

            if (pass == 0) {
                sync_start = start + phase_offset + candidate_sample_offset
                             - (int) floor((double) (target_bits - 1 + V34_INFO_SYNC_BITS) * symbol_samples + 0.5)
                             + delta;
            } else {
                if (best_sync_start < 0)
                    continue;
                sync_start = best_sync_start + delta;
            }
            if (sync_start < 0)
                continue;

            for (int i = 0; i < V34_INFO_SYNC_BITS; i++) {
                bool ok = false;
                double decision;
                int expected_bit;
                double signed_margin;
                double threshold;

                decision = p12_v21_symbol_decision_value(samples,
                                                         total_samples,
                                                         sample_rate,
                                                         channel,
                                                         sync_start + (int) floor((double) i * symbol_samples + 0.5),
                                                         symbol_samples,
                                                         invert,
                                                         NULL,
                                                         NULL,
                                                         &ok);
                if (!ok) {
                    valid = false;
                    break;
                }

                expected_bit = (V34_INFO_SYNC_CODE >> (V34_INFO_SYNC_BITS - 1 - i)) & 1;
                signed_margin = expected_bit ? decision : -decision;
                sync_score += signed_margin;
                threshold = -decision;
                if (expected_bit) {
                    lower_bias = fmax(lower_bias, threshold);
                    have_lower = true;
                } else {
                    upper_bias = fmin(upper_bias, threshold);
                    have_upper = true;
                }
            }

            if (!valid)
                continue;
            if (!have_lower || !have_upper)
                continue;
            if (!(lower_bias < upper_bias))
                continue;
            if (sync_score > best_sync_score) {
                best_sync_score = sync_score;
                best_sync_start = sync_start;
                best_bias = 0.5 * (lower_bias + upper_bias);
            }
            for (int rank = 0; rank < 3; rank++) {
                if (sync_score > top_scores[rank]) {
                    for (int move = 2; move > rank; move--) {
                        top_scores[move] = top_scores[move - 1];
                        top_starts[move] = top_starts[move - 1];
                    }
                    top_scores[rank] = sync_score;
                    top_starts[rank] = sync_start;
                    break;
                }
            }
        }
    }

    if (best_sync_start < 0)
        return false;

    {
        uint8_t payload_bits[256];
        double payload_conf[256];
        uint8_t payload[V34_INFO_MAX_BUF_BYTES];
        int recovery_shift = 0;
        int recovery_pivot = 0;
        int flip_a = -1;
        int flip_b = -1;

        for (int payload_shift = -1; payload_shift <= 1; payload_shift++) {
            bool payload_ok = true;

            for (int i = 0; i < target_bits; i++) {
                bool ok = false;
                double decision;
                int bit;

                decision = p12_v21_symbol_decision_value(samples,
                                                         total_samples,
                                                         sample_rate,
                                                         channel,
                                                         best_sync_start + (int) floor((double) (V34_INFO_SYNC_BITS + payload_shift + i) * symbol_samples + 0.5),
                                                         symbol_samples,
                                                         invert,
                                                         NULL,
                                                         NULL,
                                                         &ok);
                if (!ok) {
                    payload_ok = false;
                    break;
                }
                bit = (decision + best_bias >= 0.0) ? 1 : 0;
                payload_bits[i] = (uint8_t) bit;
                payload_conf[i] = fabs(decision + best_bias);
            }
            if (!payload_ok)
                continue;

            if (p12_debug_enabled()) {
                fprintf(stderr,
                        "[p12] INFO sync-scan target=%d best_sync=%.1fms bias=%.3f score=%.3f payload_shift=%d top=[%.1fms %.3f][%.1fms %.3f][%.1fms %.3f]\n",
                        target_bits,
                        (double) best_sync_start * 1000.0 / (double) sample_rate,
                        best_bias,
                        best_sync_score,
                        payload_shift,
                        top_starts[0] >= 0 ? (double) top_starts[0] * 1000.0 / (double) sample_rate : -1.0,
                        top_scores[0],
                        top_starts[1] >= 0 ? (double) top_starts[1] * 1000.0 / (double) sample_rate : -1.0,
                        top_scores[1],
                        top_starts[2] >= 0 ? (double) top_starts[2] * 1000.0 / (double) sample_rate : -1.0,
                        top_scores[2]);
                fprintf(stderr, "[p12] INFO payload-soft target=%d bits=", target_bits);
                for (int i = 0; i < target_bits && i < 24; i++) {
                    fprintf(stderr, "%u", (unsigned) payload_bits[i]);
                }
                fprintf(stderr, " conf=");
                for (int i = 0; i < target_bits && i < 8; i++) {
                    fprintf(stderr, "%s%.3f", (i == 0) ? "" : ",", payload_conf[i]);
                }
                fprintf(stderr, "\n");
                p12_debug_dump_payload_bytes("INFO payload", payload_bits, target_bits);
                if (payload_shift == 0 && target_bits <= P12_INFO0D_PAYLOAD_BITS) {
                    p12_debug_dump_symbol_trace(samples,
                                                total_samples,
                                                sample_rate,
                                                channel,
                                                best_sync_start,
                                                target_bits,
                                                invert,
                                                symbol_samples,
                                                best_bias);
                }
            }

            p12_pack_lsb_bits(payload, (int) sizeof(payload), payload_bits, target_bits);
            if (p12_try_payload_pack_variants(frame_out, payload_bits, target_bits)) {
                if (frame_sample_out) {
                    *frame_sample_out = best_sync_start
                                        + (int) floor((double) (V34_INFO_SYNC_BITS + payload_shift) * symbol_samples + 0.5);
                }
                return true;
            }

            if (v34_info_try_boundary_recovery(payload,
                                               payload,
                                               target_bits,
                                               &recovery_shift)) {
                if (p12_debug_enabled()) {
                    fprintf(stderr,
                            "[p12] INFO sync-boundary recovery shift=%d payload_shift=%d target=%d\n",
                            recovery_shift,
                            payload_shift,
                            target_bits);
                }
                p12_build_info_frame_from_payload(frame_out, payload, target_bits);
                if (frame_sample_out) {
                    *frame_sample_out = best_sync_start
                                        + (int) floor((double) (V34_INFO_SYNC_BITS + payload_shift) * symbol_samples + 0.5);
                }
                return true;
            }

            if (v34_info_try_local_slip_recovery(payload,
                                                 payload,
                                                 target_bits,
                                                 &recovery_pivot,
                                                 &recovery_shift)) {
                if (p12_debug_enabled()) {
                    fprintf(stderr,
                            "[p12] INFO sync-local-slip recovery pivot=%d shift=%d payload_shift=%d target=%d\n",
                            recovery_pivot,
                            recovery_shift,
                            payload_shift,
                            target_bits);
                }
                p12_build_info_frame_from_payload(frame_out, payload, target_bits);
                if (frame_sample_out) {
                    *frame_sample_out = best_sync_start
                                        + (int) floor((double) (V34_INFO_SYNC_BITS + payload_shift) * symbol_samples + 0.5);
                }
                return true;
            }

            if (p12_try_soft_bit_flips(frame_out,
                                       payload_bits,
                                       payload_conf,
                                       target_bits,
                                       &flip_a,
                                       &flip_b)) {
                if (p12_debug_enabled()) {
                    fprintf(stderr,
                            "[p12] INFO sync-soft recovery flip_a=%d flip_b=%d payload_shift=%d target=%d\n",
                            flip_a,
                            flip_b,
                            payload_shift,
                            target_bits);
                }
                if (frame_sample_out) {
                    *frame_sample_out = best_sync_start
                                        + (int) floor((double) (V34_INFO_SYNC_BITS + payload_shift) * symbol_samples + 0.5);
                }
                return true;
            }
        }
    }

    return false;
}

static void p12_build_info_frame_from_payload(v34_info_frame_t *frame,
                                              const uint8_t *payload,
                                              int target_bits)
{
    uint8_t bits[256];

    if (!frame)
        return;

    memset(frame, 0, sizeof(*frame));
    if (!payload || target_bits <= 0)
        return;

    frame->valid = true;
    frame->target_bits = target_bits;
    frame->payload_bytes = (target_bits + 7) / 8;
    if (frame->payload_bytes > V34_INFO_MAX_BUF_BYTES)
        frame->payload_bytes = V34_INFO_MAX_BUF_BYTES;
    frame->total_bits = V34_INFO_SYNC_BITS + target_bits;
    frame->total_bytes = (frame->total_bits + 7) / 8;
    if (frame->total_bytes > V34_INFO_MAX_BUF_BYTES)
        frame->total_bytes = V34_INFO_MAX_BUF_BYTES;
    memcpy(frame->payload, payload, (size_t) frame->payload_bytes);

    memset(bits, 0, sizeof(bits));
    for (int i = 0; i < V34_INFO_SYNC_BITS; i++)
        bits[i] = (uint8_t) ((V34_INFO_SYNC_CODE >> (V34_INFO_SYNC_BITS - 1 - i)) & 1);
    for (int i = 0; i < target_bits; i++) {
        uint8_t octet = bit_reverse8(payload[i >> 3]);

        bits[V34_INFO_SYNC_BITS + i] = (octet >> (7 - (i & 7))) & 1;
    }
    memset(frame->bytes, 0, sizeof(frame->bytes));
    for (int i = 0; i < frame->total_bits; i++) {
        if (bits[i])
            frame->bytes[i >> 3] |= (uint8_t) (1U << (i & 7));
    }
}

static void info_decoder_put_bit(void *ctx, int bit, int sample_offset)
{
    info_frame_decoder_t *d = (info_frame_decoder_t *)ctx;
    bool frame_done;
    uint16_t crc;

    if (!d || d->frame_complete)
        return;

    if (v34_info_collector_push_bit_verbose(&d->collector, bit,
                                            NULL, 0,
                                            &frame_done, &crc)) {
        d->frame_complete = true;
        (void) v34_info_frame_from_collector(&d->frame, &d->collector);
        d->frame_sample_offset = sample_offset;
        return;
    }
    if (frame_done) {
        int byte_count = (d->collector.target_bits + 7) / 8;

        if (byte_count > V34_INFO_MAX_BUF_BYTES)
            byte_count = V34_INFO_MAX_BUF_BYTES;
        memcpy(d->candidate_bytes, d->collector.info_buf, (size_t) byte_count);
        d->frame_candidate_seen = true;
        d->candidate_sample_offset = sample_offset;
        d->candidate_crc = crc;
    }
}

static bool decode_info_from_fsk_burst(const int16_t *samples,
                                       int total_samples,
                                       int sample_rate,
                                       int channel,
                                       const p12_fsk_burst_t *burst,
                                       int target_bits,
                                       v34_info_frame_t *frame_out,
                                       int *frame_sample_out)
{
    info_frame_decoder_t decoder;
    int pre_pad;
    int post_pad;

    if (!burst || !burst->seen || !frame_out)
        return false;

    int start = burst->start_sample;
    int len = burst->duration_samples;

    pre_pad = (sample_rate * P12_INFO_PRE_PAD_MS) / 1000;
    post_pad = (sample_rate * P12_INFO_POST_PAD_MS) / 1000;
    if (pre_pad < 0)
        pre_pad = 0;
    if (post_pad < 0)
        post_pad = 0;

    if (start > pre_pad) {
        start -= pre_pad;
        len += pre_pad;
    } else {
        len += start;
        start = 0;
    }
    if (start + len + post_pad <= total_samples)
        len += post_pad;
    else
        len = total_samples - start;

    if (start < 0 || start + len > total_samples)
        return false;

    /* Try both polarities with the fixed-phase block demod first. */
    for (int inv = 0; inv < 2; inv++) {
        memset(&decoder, 0, sizeof(decoder));
        v34_info_collector_init(&decoder.collector, target_bits);

                    v21_fsk_demod_block(samples + start, len, sample_rate,
                            channel, (bool)inv,
                            info_decoder_put_bit, &decoder);

        if (decoder.frame_complete) {
            *frame_out = decoder.frame;
            if (frame_sample_out)
                *frame_sample_out = start + decoder.frame_sample_offset;
            return true;
        }
        if (decoder.frame_candidate_seen) {
            int recovery_shift = 0;
            int recovery_pivot = 0;
            int recovery_bit_a = -1;
            int recovery_bit_b = -1;

            if (p12_debug_enabled()) {
                fprintf(stderr,
                        "[p12] INFO candidate crc=0x%04x target=%d inv=%d mode=block sample=%.1fms\n",
                        decoder.candidate_crc,
                        target_bits,
                        inv,
                        (double) (start + decoder.candidate_sample_offset) * 1000.0 / (double) sample_rate);
            }

            if (v34_info_try_boundary_recovery(decoder.candidate_bytes,
                                               decoder.candidate_bytes,
                                               target_bits,
                                               &recovery_shift)) {
                p12_build_info_frame_from_payload(frame_out, decoder.candidate_bytes, target_bits);
                if (p12_debug_enabled()) {
                    fprintf(stderr,
                            "[p12] INFO boundary recovery shift=%d target=%d inv=%d mode=block\n",
                            recovery_shift,
                            target_bits,
                            inv);
                }
                if (frame_sample_out)
                    *frame_sample_out = start + decoder.candidate_sample_offset;
                return true;
            }
            if (v34_info_try_local_slip_recovery(decoder.candidate_bytes,
                                                 decoder.candidate_bytes,
                                                 target_bits,
                                                 &recovery_pivot,
                                                 &recovery_shift)) {
                p12_build_info_frame_from_payload(frame_out, decoder.candidate_bytes, target_bits);
                if (p12_debug_enabled()) {
                    fprintf(stderr,
                            "[p12] INFO local-slip recovery pivot=%d shift=%d target=%d inv=%d mode=block\n",
                            recovery_pivot,
                            recovery_shift,
                            target_bits,
                            inv);
                }
                if (frame_sample_out)
                    *frame_sample_out = start + decoder.candidate_sample_offset;
                return true;
            }
            if (v34_info_try_bit_error_recovery(decoder.candidate_bytes,
                                                decoder.candidate_bytes,
                                                target_bits,
                                                &recovery_bit_a,
                                                &recovery_bit_b)) {
                p12_build_info_frame_from_payload(frame_out, decoder.candidate_bytes, target_bits);
                if (p12_debug_enabled()) {
                    fprintf(stderr,
                            "[p12] INFO bit-error recovery bit_a=%d bit_b=%d target=%d inv=%d mode=block\n",
                            recovery_bit_a,
                            recovery_bit_b,
                            target_bits,
                            inv);
                }
                if (frame_sample_out)
                    *frame_sample_out = start + decoder.candidate_sample_offset;
                return true;
            }
        } else if (p12_debug_enabled()) {
            fprintf(stderr,
                    "[p12] INFO no candidate target=%d inv=%d mode=block window=%.1f-%.1fms\n",
                    target_bits,
                    inv,
                    (double) start * 1000.0 / (double) sample_rate,
                    (double) (start + len) * 1000.0 / (double) sample_rate);
        }
    }

    /* Then try the timing-tracking streaming demod with a few phase and baud offsets. */
    {
        static const double baud_variants[P12_INFO_STREAM_BAUD_VARIANTS] = {
            288.0, 294.0, 300.0, 306.0, 312.0
        };

        for (int inv = 0; inv < 2; inv++) {
            for (int baud_idx = 0; baud_idx < P12_INFO_STREAM_BAUD_VARIANTS; baud_idx++) {
                double symbol_samples = (double) sample_rate / baud_variants[baud_idx];

                for (int phase = 0; phase < P12_INFO_STREAM_PHASES; phase++) {
                    v21_fsk_stream_t stream;
                    int phase_offset = (int) (((double) phase * symbol_samples)
                                              / (double) P12_INFO_STREAM_PHASES);

                    if (phase_offset >= len)
                        continue;

                    memset(&decoder, 0, sizeof(decoder));
                    v34_info_collector_init(&decoder.collector, target_bits);
                    v21_fsk_stream_init(&stream,
                                        channel,
                                        sample_rate,
                                        (bool) inv,
                                        info_decoder_put_bit,
                                        &decoder);
                    stream.symbol_samples = symbol_samples;
                    v21_fsk_stream_rx(&stream, samples + start + phase_offset, len - phase_offset);

                    if (decoder.frame_complete) {
                        *frame_out = decoder.frame;
                        if (frame_sample_out)
                            *frame_sample_out = start + phase_offset + decoder.frame_sample_offset;
                        return true;
                    }
                    if (decoder.frame_candidate_seen) {
                        int recovery_shift = 0;
                        int recovery_pivot = 0;
                        int recovery_bit_a = -1;
                        int recovery_bit_b = -1;

                        if (p12_debug_enabled()) {
                            fprintf(stderr,
                                    "[p12] INFO candidate crc=0x%04x target=%d inv=%d mode=stream baud=%.1f phase=%d sample=%.1fms\n",
                                    decoder.candidate_crc,
                                    target_bits,
                                    inv,
                                    baud_variants[baud_idx],
                                    phase,
                                    (double) (start + phase_offset + decoder.candidate_sample_offset) * 1000.0 / (double) sample_rate);
                        }

                        if (v34_info_try_boundary_recovery(decoder.candidate_bytes,
                                                           decoder.candidate_bytes,
                                                           target_bits,
                                                           &recovery_shift)) {
                            p12_build_info_frame_from_payload(frame_out, decoder.candidate_bytes, target_bits);
                            if (p12_debug_enabled()) {
                                fprintf(stderr,
                                        "[p12] INFO boundary recovery shift=%d target=%d inv=%d mode=stream baud=%.1f phase=%d\n",
                                        recovery_shift,
                                        target_bits,
                                        inv,
                                        baud_variants[baud_idx],
                                        phase);
                            }
                            if (frame_sample_out)
                                *frame_sample_out = start + phase_offset + decoder.candidate_sample_offset;
                            return true;
                        }
                        if (v34_info_try_local_slip_recovery(decoder.candidate_bytes,
                                                             decoder.candidate_bytes,
                                                             target_bits,
                                                             &recovery_pivot,
                                                             &recovery_shift)) {
                            p12_build_info_frame_from_payload(frame_out, decoder.candidate_bytes, target_bits);
                            if (p12_debug_enabled()) {
                                fprintf(stderr,
                                        "[p12] INFO local-slip recovery pivot=%d shift=%d target=%d inv=%d mode=stream baud=%.1f phase=%d\n",
                                        recovery_pivot,
                                        recovery_shift,
                                        target_bits,
                                        inv,
                                        baud_variants[baud_idx],
                                        phase);
                            }
                            if (frame_sample_out)
                                *frame_sample_out = start + phase_offset + decoder.candidate_sample_offset;
                            return true;
                        }
                        if (v34_info_try_bit_error_recovery(decoder.candidate_bytes,
                                                            decoder.candidate_bytes,
                                                            target_bits,
                                                            &recovery_bit_a,
                                                            &recovery_bit_b)) {
                            p12_build_info_frame_from_payload(frame_out, decoder.candidate_bytes, target_bits);
                            if (p12_debug_enabled()) {
                                fprintf(stderr,
                                        "[p12] INFO bit-error recovery bit_a=%d bit_b=%d target=%d inv=%d mode=stream baud=%.1f phase=%d\n",
                                        recovery_bit_a,
                                        recovery_bit_b,
                                        target_bits,
                                        inv,
                                        baud_variants[baud_idx],
                                        phase);
                            }
                            if (frame_sample_out)
                                *frame_sample_out = start + phase_offset + decoder.candidate_sample_offset;
                            return true;
                        }
                        if (p12_try_sync_trained_info_redecode(samples,
                                                               total_samples,
                                                               sample_rate,
                                                               channel,
                                                               start,
                                                               phase_offset,
                                                               target_bits,
                                                               (bool) inv,
                                                               symbol_samples,
                                                               decoder.candidate_sample_offset,
                                                               frame_out,
                                                               frame_sample_out)) {
                            if (p12_debug_enabled()) {
                                fprintf(stderr,
                                        "[p12] INFO sync-trained recovery target=%d inv=%d mode=stream baud=%.1f phase=%d\n",
                                        target_bits,
                                        inv,
                                        baud_variants[baud_idx],
                                        phase);
                            }
                            return true;
                        }
                    } else if (p12_debug_enabled()) {
                        fprintf(stderr,
                                "[p12] INFO no candidate target=%d inv=%d mode=stream baud=%.1f phase=%d window=%.1f-%.1fms\n",
                                target_bits,
                                inv,
                                baud_variants[baud_idx],
                                phase,
                                (double) (start + phase_offset) * 1000.0 / (double) sample_rate,
                                (double) (start + len) * 1000.0 / (double) sample_rate);
                    }
                }
            }
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
    p12_fsk_burst_t phase2_ch1_bursts[P12_MAX_FSK_BURSTS];
    p12_fsk_burst_t phase2_ch2_bursts[P12_MAX_FSK_BURSTS];
    int phase2_ch1_burst_count = 0;
    int phase2_ch2_burst_count = 0;
    int limit = (max_sample > 0 && max_sample < total_samples) ? max_sample : total_samples;
    int search_start = 0;
    int phase2_cap = (sample_rate * PHASE2_MAX_SCAN_MS) / 1000;
    int phase2_end_hint = -1;
    int phase2_merge_gap = (sample_rate * P12_PHASE2_REPEAT_GAP_MS) / 1000;
    int tone_anchor_sample = -1;
    p12_phase2_role_t phase2_role = p12_phase2_role_from_observations(result);
    p12_phase2_role_t alt_phase2_role = P12_PHASE2_ROLE_UNKNOWN;
    p12_signal_tone_hit_t alt_tone_a;
    p12_signal_tone_hit_t alt_tone_b;
    bool keep_alt_v90_hypothesis = false;
    p12_timing_hint_t handoff_info0_hint;
    p12_timing_hint_t alt_handoff_info0_hint;

    memset(phase2_ch1_bursts, 0, sizeof(phase2_ch1_bursts));
    memset(phase2_ch2_bursts, 0, sizeof(phase2_ch2_bursts));
    memset(&handoff_info0_hint, 0, sizeof(handoff_info0_hint));
    memset(&alt_handoff_info0_hint, 0, sizeof(alt_handoff_info0_hint));
    memset(&alt_tone_a, 0, sizeof(alt_tone_a));
    memset(&alt_tone_b, 0, sizeof(alt_tone_b));

    if (result->answer_tone.detected)
        search_start = result->answer_tone.start_sample
                     + result->answer_tone.duration_samples
                     + p12_answer_tone_post_hold_samples(&result->answer_tone, sample_rate);
    if (result->jm.detected && result->jm.duration_samples > 0) {
        int jm_info_start = result->jm.sample_offset
                          + result->jm.duration_samples
                          + (sample_rate * P12_CJ_TO_INFO0_MS) / 1000;

        if (jm_info_start > search_start)
            search_start = jm_info_start;
    } else if (result->cj.detected && result->cj.duration_samples > 0) {
        int cj_info_start = result->cj.sample_offset
                          + result->cj.duration_samples
                          + (sample_rate * P12_CJ_TO_INFO0_MS) / 1000;

        if (cj_info_start > search_start)
            search_start = cj_info_start;
    }
    if (phase2_cap > 0 && search_start + phase2_cap < limit)
        limit = search_start + phase2_cap;
    if (result->jm.detected && result->jm.duration_samples > 0)
        p12_fill_timing_hint_from_jm_end(&result->info0_from_cj_hint, &result->jm, sample_rate, limit);
    else
        p12_fill_timing_hint_from_cj(&result->info0_from_cj_hint, &result->cj, sample_rate, limit);
    if (p12_debug_enabled()) {
        fprintf(stderr,
                "[p12] phase2 search start=%.1fms limit=%.1fms cap=%.1fms\n",
                (double) search_start * 1000.0 / (double) sample_rate,
                (double) limit * 1000.0 / (double) sample_rate,
                (double) phase2_cap * 1000.0 / (double) sample_rate);
        if (result->info0_from_cj_hint.valid) {
            fprintf(stderr,
                    "[p12] info0 timing hint anchor=%.1fms expected=%.1fms window=%.1f-%.1fms\n",
                    (double) result->info0_from_cj_hint.anchor_sample * 1000.0 / (double) sample_rate,
                    (double) result->info0_from_cj_hint.expected_sample * 1000.0 / (double) sample_rate,
                    (double) result->info0_from_cj_hint.window_start_sample * 1000.0 / (double) sample_rate,
                    (double) result->info0_from_cj_hint.window_end_sample * 1000.0 / (double) sample_rate);
        }
    }

    detect_phase2_signal_tone_runs(samples,
                                   total_samples,
                                   sample_rate,
                                   search_start,
                                   limit,
                                   phase2_role,
                                   &result->tone_a,
                                   &result->tone_b);
    if (result->v90_capable) {
        p12_phase2_role_t alt_role = p12_phase2_role_v90_swapped(phase2_role);

        if (alt_role != phase2_role) {
            p12_signal_tone_hit_t alt_tone_a;
            p12_signal_tone_hit_t alt_tone_b;
            int base_score;
            int alt_score;

            memset(&alt_tone_a, 0, sizeof(alt_tone_a));
            memset(&alt_tone_b, 0, sizeof(alt_tone_b));
            detect_phase2_signal_tone_runs(samples,
                                           total_samples,
                                           sample_rate,
                                           search_start,
                                           limit,
                                           alt_role,
                                           &alt_tone_a,
                                           &alt_tone_b);
            base_score = p12_phase2_signal_tone_score(&result->tone_a, &result->tone_b);
            alt_score = p12_phase2_signal_tone_score(&alt_tone_a, &alt_tone_b);
            if (p12_debug_enabled()) {
                fprintf(stderr,
                        "[p12] v90 tone hypothesis primary=%s score=%d alt=%s score=%d\n",
                        phase12_phase2_role_name(phase2_role),
                        base_score,
                        phase12_phase2_role_name(alt_role),
                        alt_score);
            }
            if (alt_score > base_score) {
                result->tone_a = alt_tone_a;
                result->tone_b = alt_tone_b;
                phase2_role = alt_role;
                if (p12_debug_enabled()) {
                    fprintf(stderr,
                            "[p12] selected alternate v90 tone hypothesis role=%s\n",
                            phase12_phase2_role_name(phase2_role));
                }
            } else if (alt_score == base_score && alt_score > 0) {
                keep_alt_v90_hypothesis = true;
                alt_phase2_role = alt_role;
            }
        }
    }
    if (p12_debug_enabled()) {
        if (result->tone_b.detected) {
            fprintf(stderr,
                    "[p12] tone B start=%.1fms dur=%.1fms reversal=%s\n",
                    (double) result->tone_b.start_sample * 1000.0 / (double) sample_rate,
                    (double) result->tone_b.duration_samples * 1000.0 / (double) sample_rate,
                    result->tone_b.phase_reversal_seen ? "yes" : "no");
        }
        if (result->tone_a.detected) {
            fprintf(stderr,
                    "[p12] tone A start=%.1fms dur=%.1fms reversal=%s\n",
                    (double) result->tone_a.start_sample * 1000.0 / (double) sample_rate,
                    (double) result->tone_a.duration_samples * 1000.0 / (double) sample_rate,
                    result->tone_a.phase_reversal_seen ? "yes" : "no");
        }
    }
    if (result->tone_a.detected)
        tone_anchor_sample = result->tone_a.start_sample;
    else if (result->tone_b.detected)
        tone_anchor_sample = result->tone_b.start_sample;

    /* INFO0 is sent on V.21 CH2 (answerer → caller direction)
     * INFO1 is sent on V.21 CH1 (caller → answerer direction)
     *
     * Scan for V.21 FSK bursts, then try to decode INFO frames from each.
     * The v34_info_collector handles sync code detection (0x372).
     */

    /* Always rescan V.21 bursts for Phase 2 from the handoff point onward.
       Reusing the broad Phase 1 burst list can hide later INFO bursts on
       long captures where the early negotiation already filled the cache. */
    phase2_ch2_burst_count = detect_fsk_bursts(samples, total_samples, sample_rate,
                                               search_start, limit, V21_CH2,
                                               phase2_ch2_bursts, P12_MAX_FSK_BURSTS);
    phase2_ch1_burst_count = detect_fsk_bursts(samples, total_samples, sample_rate,
                                               search_start, limit, V21_CH1,
                                               phase2_ch1_bursts, P12_MAX_FSK_BURSTS);
    phase2_ch2_burst_count = p12_merge_bursts_in_place(phase2_ch2_bursts,
                                                       phase2_ch2_burst_count,
                                                       phase2_merge_gap);
    phase2_ch1_burst_count = p12_merge_bursts_in_place(phase2_ch1_bursts,
                                                       phase2_ch1_burst_count,
                                                       phase2_merge_gap);
    phase2_ch2_burst_count = p12_prune_low_energy_bursts_in_place(phase2_ch2_bursts,
                                                                  phase2_ch2_burst_count,
                                                                  P12_PHASE2_BURST_RELATIVE_SIGNAL_CUTOFF,
                                                                  "phase2 CH2",
                                                                  sample_rate);
    phase2_ch1_burst_count = p12_prune_low_energy_bursts_in_place(phase2_ch1_bursts,
                                                                  phase2_ch1_burst_count,
                                                                  P12_PHASE2_BURST_RELATIVE_SIGNAL_CUTOFF,
                                                                  "phase2 CH1",
                                                                  sample_rate);
    p12_fill_handoff_info0_hint(&handoff_info0_hint,
                                phase2_role,
                                search_start,
                                limit,
                                phase2_ch1_bursts,
                                phase2_ch1_burst_count,
                                sample_rate);
    if (keep_alt_v90_hypothesis) {
        p12_fill_handoff_info0_hint(&alt_handoff_info0_hint,
                                    alt_phase2_role,
                                    search_start,
                                    limit,
                                    phase2_ch1_bursts,
                                    phase2_ch1_burst_count,
                                    sample_rate);
    }
    if (!result->info0.detected) {
        if (result->tone_b.detected)
            p12_refine_info0_hint_from_tone(&handoff_info0_hint, &result->tone_b, sample_rate);
        else if (result->tone_a.detected)
            p12_refine_info0_hint_from_tone(&handoff_info0_hint, &result->tone_a, sample_rate);
        if (keep_alt_v90_hypothesis) {
            if (alt_tone_b.detected)
                p12_refine_info0_hint_from_tone(&alt_handoff_info0_hint, &alt_tone_b, sample_rate);
            else if (alt_tone_a.detected)
                p12_refine_info0_hint_from_tone(&alt_handoff_info0_hint, &alt_tone_a, sample_rate);
        }
        if (p12_debug_enabled() && handoff_info0_hint.valid) {
            fprintf(stderr,
                    "[p12] refined INFO0 handoff window=%.1f-%.1fms anchor=%.1fms\n",
                    (double) handoff_info0_hint.window_start_sample * 1000.0 / (double) sample_rate,
                    (double) handoff_info0_hint.window_end_sample * 1000.0 / (double) sample_rate,
                    (double) handoff_info0_hint.anchor_sample * 1000.0 / (double) sample_rate);
        }
        if (p12_debug_enabled() && keep_alt_v90_hypothesis && alt_handoff_info0_hint.valid) {
            fprintf(stderr,
                    "[p12] refined ALT INFO0 handoff window=%.1f-%.1fms anchor=%.1fms role=%s\n",
                    (double) alt_handoff_info0_hint.window_start_sample * 1000.0 / (double) sample_rate,
                    (double) alt_handoff_info0_hint.window_end_sample * 1000.0 / (double) sample_rate,
                    (double) alt_handoff_info0_hint.anchor_sample * 1000.0 / (double) sample_rate,
                    phase12_phase2_role_name(alt_phase2_role));
        }
        if (keep_alt_v90_hypothesis && alt_handoff_info0_hint.valid) {
            if (!handoff_info0_hint.valid) {
                handoff_info0_hint = alt_handoff_info0_hint;
            } else {
                if (alt_handoff_info0_hint.window_start_sample < handoff_info0_hint.window_start_sample)
                    handoff_info0_hint.window_start_sample = alt_handoff_info0_hint.window_start_sample;
                if (alt_handoff_info0_hint.window_end_sample > handoff_info0_hint.window_end_sample)
                    handoff_info0_hint.window_end_sample = alt_handoff_info0_hint.window_end_sample;
                if (alt_handoff_info0_hint.expected_sample < handoff_info0_hint.expected_sample)
                    handoff_info0_hint.expected_sample = alt_handoff_info0_hint.expected_sample;
            }
        }
    }
    if (keep_alt_v90_hypothesis && alt_handoff_info0_hint.valid) {
        int before_alt = phase2_ch2_burst_count;

        p12_append_retry_window(phase2_ch2_bursts,
                                &phase2_ch2_burst_count,
                                P12_MAX_FSK_BURSTS,
                                &alt_handoff_info0_hint);
        if (phase2_ch2_burst_count > before_alt)
            phase2_ch2_bursts[phase2_ch2_burst_count - 1].hint_id = 1;
        if (p12_debug_enabled() && phase2_ch2_burst_count > before_alt) {
            fprintf(stderr,
                    "[p12] appended ALT INFO0 retry window role=%s window=%.1f-%.1fms\n",
                    phase12_phase2_role_name(alt_phase2_role),
                    (double) alt_handoff_info0_hint.window_start_sample * 1000.0 / (double) sample_rate,
                    (double) alt_handoff_info0_hint.window_end_sample * 1000.0 / (double) sample_rate);
        }
    }
    p12_focus_bursts_to_window(phase2_ch2_bursts,
                               phase2_ch2_burst_count,
                               handoff_info0_hint.valid ? &handoff_info0_hint : &result->info0_from_cj_hint,
                               tone_anchor_sample,
                               "phase2 CH2/INFO0",
                               sample_rate);
    p12_rank_bursts_for_info(phase2_ch2_bursts,
                             phase2_ch2_burst_count,
                             handoff_info0_hint.valid ? &handoff_info0_hint : &result->info0_from_cj_hint,
                             tone_anchor_sample,
                             "phase2 CH2/INFO0",
                             sample_rate);
    p12_debug_log_bursts("phase2 CH2 windows", phase2_ch2_bursts, phase2_ch2_burst_count, sample_rate);
    p12_debug_log_bursts("phase2 CH1 windows", phase2_ch1_bursts, phase2_ch1_burst_count, sample_rate);

    if (phase2_ch2_burst_count == 0 && result->info0_from_cj_hint.valid) {
        p12_append_retry_window(phase2_ch2_bursts,
                                &phase2_ch2_burst_count,
                                P12_MAX_FSK_BURSTS,
                                &result->info0_from_cj_hint);
        result->info0_from_cj_hint.used_for_retry = (phase2_ch2_burst_count > 0);
        if (phase2_ch2_burst_count > 0) {
            result->info0_from_cj_hint.retry_count++;
            if (p12_debug_enabled())
                fprintf(stderr, "[p12] appended INFO0 retry window from CJ timing hint\n");
        }
    }
    /* Also try the retry bank if all detected CH2 bursts fall after the
     * expected INFO0 window.  INFO0 must precede Tone A/B, so any burst
     * that starts at or after the tone start cannot be the INFO0 frame. */
    {
        int tone_start = tone_anchor_sample;
        bool bursts_before_tone = false;

        if (tone_start >= 0) {
            for (int i = 0; i < phase2_ch2_burst_count; i++) {
                if (phase2_ch2_bursts[i].start_sample < tone_start) {
                    bursts_before_tone = true;
                    break;
                }
            }
        } else {
            /* No tone detected — existing burst list is our best guess. */
            bursts_before_tone = (phase2_ch2_burst_count > 0);
        }

        if (!result->info0.detected && !bursts_before_tone) {
            int before_count = phase2_ch2_burst_count;

            if (handoff_info0_hint.valid) {
                p12_append_info0_retry_bank(phase2_ch2_bursts,
                                            &phase2_ch2_burst_count,
                                            P12_MAX_FSK_BURSTS,
                                            &handoff_info0_hint,
                                            tone_start,
                                            search_start,
                                            limit,
                                            sample_rate);
            } else if (tone_start >= 0 && phase2_ch2_burst_count < P12_MAX_FSK_BURSTS) {
                /* Role unknown (no CM/JM decoded) but Tone A/B is present.
                 * In V.34 Phase 2 INFO0 always precedes Tone A, so synthesize
                 * a search window anchored directly to the tone timing. */
                int guard     = (sample_rate * P12_INFO0_PRE_TONE_GUARD_MS) / 1000;
                int max_span  = (sample_rate * P12_INFO0_PRE_TONE_MAX_MS)   / 1000;
                int win_end   = tone_start - guard;
                int win_start = win_end - max_span;

                if (win_start < search_start)
                    win_start = search_start;
                if (win_end > win_start) {
                    p12_fsk_burst_t *b = &phase2_ch2_bursts[phase2_ch2_burst_count];
                    b->seen             = true;
                    b->start_sample     = win_start;
                    b->duration_samples = win_end - win_start;
                    b->peak_energy      = 0.0;
                    phase2_ch2_burst_count++;
                }
            }
            if (p12_debug_enabled() && phase2_ch2_burst_count > before_count) {
                fprintf(stderr,
                        "[p12] appended INFO0 tone-anchor retry bank role=%s windows=%d\n",
                        phase12_phase2_role_name(phase2_role),
                        phase2_ch2_burst_count - before_count);
                for (int i = before_count; i < phase2_ch2_burst_count; i++) {
                    fprintf(stderr,
                            "[p12]   INFO0 tone-retry #%d window=%.1f-%.1fms\n",
                            i - before_count + 1,
                            (double) phase2_ch2_bursts[i].start_sample * 1000.0 / (double) sample_rate,
                            (double) (phase2_ch2_bursts[i].start_sample + phase2_ch2_bursts[i].duration_samples) * 1000.0 / (double) sample_rate);
                }
            }
            p12_focus_bursts_to_window(phase2_ch2_bursts,
                                       phase2_ch2_burst_count,
                                       handoff_info0_hint.valid ? &handoff_info0_hint : &result->info0_from_cj_hint,
                                       tone_start,
                                       "phase2 CH2/INFO0 retry",
                                       sample_rate);
            p12_rank_bursts_for_info(phase2_ch2_bursts,
                                     phase2_ch2_burst_count,
                                     handoff_info0_hint.valid ? &handoff_info0_hint : &result->info0_from_cj_hint,
                                     tone_start,
                                     "phase2 CH2/INFO0 retry",
                                     sample_rate);
        }
    }

    if (!result->info0.detected
        && phase2_ch2_burst_count == 0
        && handoff_info0_hint.valid) {
        int tone_anchor_sample = result->tone_b.detected ? result->tone_b.start_sample
                               : result->tone_a.detected ? result->tone_a.start_sample
                               : -1;
        int before_count = phase2_ch2_burst_count;

        p12_append_info0_retry_bank(phase2_ch2_bursts,
                                    &phase2_ch2_burst_count,
                                    P12_MAX_FSK_BURSTS,
                                    &handoff_info0_hint,
                                    tone_anchor_sample,
                                    search_start,
                                    limit,
                                    sample_rate);
        if (p12_debug_enabled() && phase2_ch2_burst_count > before_count) {
            fprintf(stderr,
                    "[p12] appended INFO0 retry bank from phase2 handoff role=%s windows=%d\n",
                    phase12_phase2_role_name(phase2_role),
                    phase2_ch2_burst_count - before_count);
            for (int i = before_count; i < phase2_ch2_burst_count; i++) {
                fprintf(stderr,
                        "[p12]   INFO0 retry #%d window=%.1f-%.1fms\n",
                        i - before_count + 1,
                        (double) phase2_ch2_bursts[i].start_sample * 1000.0 / (double) sample_rate,
                        (double) (phase2_ch2_bursts[i].start_sample + phase2_ch2_bursts[i].duration_samples) * 1000.0 / (double) sample_rate);
            }
        }
        p12_focus_bursts_to_window(phase2_ch2_bursts,
                                   phase2_ch2_burst_count,
                                   &handoff_info0_hint,
                                   tone_anchor_sample,
                                   "phase2 CH2/INFO0 fallback",
                                   sample_rate);
        p12_rank_bursts_for_info(phase2_ch2_bursts,
                                 phase2_ch2_burst_count,
                                 &handoff_info0_hint,
                                 tone_anchor_sample,
                                 "phase2 CH2/INFO0 fallback",
                                 sample_rate);
    }

    /* Try to decode INFO0 from CH2 sequence windows. */
    /* The collector wants payload bits, not the full framed bit count. */
    for (int b = 0; b < phase2_ch2_burst_count && !result->info0.detected; b++) {
        const p12_timing_hint_t *decode_hint =
            (phase2_ch2_bursts[b].hint_id == 1 && alt_handoff_info0_hint.valid)
                ? &alt_handoff_info0_hint
                : (handoff_info0_hint.valid ? &handoff_info0_hint : &result->info0_from_cj_hint);
        p12_fsk_burst_t decode_burst;
        v34_info_frame_t frame;
        int frame_sample = 0;
        int duplicate_of = -1;
        bool duplicate = false;

        p12_prepare_info0_decode_burst(&decode_burst,
                                       &phase2_ch2_bursts[b],
                                       decode_hint,
                                       tone_anchor_sample,
                                       sample_rate);
        p12_refine_info0_decode_burst_by_tone(&decode_burst,
                                              samples,
                                              total_samples,
                                              sample_rate);
        if (!decode_burst.seen || decode_burst.duration_samples <= 0)
            continue;
        for (int prev = 0; prev < b; prev++) {
            const p12_timing_hint_t *prev_hint =
                (phase2_ch2_bursts[prev].hint_id == 1 && alt_handoff_info0_hint.valid)
                    ? &alt_handoff_info0_hint
                    : (handoff_info0_hint.valid ? &handoff_info0_hint : &result->info0_from_cj_hint);
            p12_fsk_burst_t prev_decode;

            p12_prepare_info0_decode_burst(&prev_decode,
                                           &phase2_ch2_bursts[prev],
                                           prev_hint,
                                           tone_anchor_sample,
                                           sample_rate);
            p12_refine_info0_decode_burst_by_tone(&prev_decode,
                                                  samples,
                                                  total_samples,
                                                  sample_rate);
            if (!prev_decode.seen || prev_decode.duration_samples <= 0)
                continue;
            if (prev_decode.start_sample == decode_burst.start_sample
                && prev_decode.duration_samples == decode_burst.duration_samples) {
                duplicate = true;
                duplicate_of = prev + 1;
                break;
            }
        }
        if (duplicate) {
            if (p12_debug_enabled()) {
                fprintf(stderr,
                        "[p12] INFO0 skip duplicate window=%d same_as=%d start=%.1fms dur=%.1fms\n",
                        b + 1,
                        duplicate_of,
                        (double) decode_burst.start_sample * 1000.0 / (double) sample_rate,
                        (double) decode_burst.duration_samples * 1000.0 / (double) sample_rate);
            }
            continue;
        }

        /* Try INFO0a first, then INFO0d. */
        for (int try_info0d = 0; try_info0d < 2 && !result->info0.detected; try_info0d++) {
            int target = try_info0d ? P12_INFO0D_PAYLOAD_BITS : P12_INFO0A_PAYLOAD_BITS;

            if (p12_debug_enabled()) {
                fprintf(stderr,
                        "[p12] INFO0 try window=%d target=%d start=%.1fms dur=%.1fms raw=%.1f-%.1fms hint=%u\n",
                        b + 1,
                        target,
                        (double) decode_burst.start_sample * 1000.0 / (double) sample_rate,
                        (double) decode_burst.duration_samples * 1000.0 / (double) sample_rate,
                        (double) phase2_ch2_bursts[b].start_sample * 1000.0 / (double) sample_rate,
                        (double) p12_burst_end_sample(&phase2_ch2_bursts[b]) * 1000.0 / (double) sample_rate,
                        (unsigned) phase2_ch2_bursts[b].hint_id);
            }

            if (decode_info_from_fsk_burst(samples, total_samples, sample_rate,
                                           V21_CH2, &decode_burst,
                                           target, &frame, &frame_sample)) {
                v34_v90_info0a_t raw;
                v90_info0a_t mapped;
                bool parsed_ok = false;
                p12_info0_kind_t kind = P12_INFO0_KIND_UNKNOWN;

                if (!try_info0d) {
                    if (result->role_detected && result->is_caller) {
                        parsed_ok = v34_info_parse_info0a_v34_frame(&frame, &raw, &mapped);
                        kind = P12_INFO0_KIND_SHARED_INFO0A;
                    } else if (result->role_detected && !result->is_caller) {
                        parsed_ok = v34_info_parse_info0c_v34_frame(&frame, &raw, &mapped);
                        kind = P12_INFO0_KIND_V34_INFO0C;
                    } else {
                        parsed_ok = v34_info_parse_info0a_v34_frame(&frame, &raw, &mapped);
                        kind = P12_INFO0_KIND_SHARED_INFO0A;
                    }
                } else {
                    parsed_ok = v34_info_parse_info0a_v90_frame(&frame, &raw, &mapped);
                    kind = p12_classify_info0_kind(result, true);
                }

                if (parsed_ok) {
                    result->info0.detected = true;
                    result->info0.sample_offset = frame_sample;
                    result->info0.duration_samples = decode_burst.duration_samples;
                    result->info0.is_info0d = (bool)try_info0d;
                    result->info0.kind = kind;
                    result->info0.frame = frame;
                    result->info0.raw = raw;
                    result->info0.parsed = mapped;
                    phase2_end_hint = result->info0.sample_offset + result->info0.duration_samples;
                    if (p12_debug_enabled()) {
                        fprintf(stderr,
                                "[p12] INFO0 hit target=%d sample=%.1fms kind=%s is_d=%u\n",
                                target,
                                (double) result->info0.sample_offset * 1000.0 / (double) sample_rate,
                                phase12_info0_kind_name(result->info0.kind),
                                result->info0.is_info0d ? 1U : 0U);
                    }
                } else if (p12_debug_enabled()) {
                    fprintf(stderr, "[p12] INFO0 frame candidate failed parse target=%d\n", target);
                }
            } else if (p12_debug_enabled()) {
                fprintf(stderr, "[p12] INFO0 no frame target=%d\n", target);
            }
        }
    }

    /* Try to decode INFO1 from CH1 sequence windows. */
    /* The collector wants payload bits, not the full framed bit count. */
    for (int b = 0; b < phase2_ch1_burst_count && !result->info1.detected; b++) {
        v34_info_frame_t frame;
        int frame_sample = 0;

        /* Try INFO1a first, then INFO1d. */
        for (int try_1d = 0; try_1d < 2 && !result->info1.detected; try_1d++) {
            int target = try_1d ? P12_INFO1D_PAYLOAD_BITS : P12_INFO1A_PAYLOAD_BITS;

            if (p12_debug_enabled()) {
                fprintf(stderr,
                        "[p12] INFO1 try window=%d target=%d start=%.1fms dur=%.1fms\n",
                        b + 1,
                        target,
                        (double) phase2_ch1_bursts[b].start_sample * 1000.0 / (double) sample_rate,
                        (double) phase2_ch1_bursts[b].duration_samples * 1000.0 / (double) sample_rate);
            }

            if (decode_info_from_fsk_burst(samples, total_samples, sample_rate,
                                           V21_CH1, &phase2_ch1_bursts[b],
                                           target, &frame, &frame_sample)) {
                if (try_1d) {
                    /* INFO1d */
                    v34_v90_info1d_t info1d;
                    v34_info1c_generic_t info1c;

                    if (v34_info_parse_info1c_v34_frame(&frame, &info1c)) {
                        result->info1.detected = true;
                        result->info1.sample_offset = frame_sample;
                        result->info1.duration_samples = phase2_ch1_bursts[b].duration_samples;
                        result->info1.is_info1d = false;
                        result->info1.kind = P12_INFO1_KIND_V34_INFO1C;
                        result->info1.frame = frame;
                        result->info1.v34_info1c = info1c;
                        if (result->info1.sample_offset + result->info1.duration_samples > phase2_end_hint)
                            phase2_end_hint = result->info1.sample_offset + result->info1.duration_samples;
                        if (p12_debug_enabled()) {
                            fprintf(stderr,
                                    "[p12] INFO1c hit target=%d sample=%.1fms\n",
                                    target,
                                    (double) result->info1.sample_offset * 1000.0 / (double) sample_rate);
                        }
                    } else if (v34_info_parse_info1d_v90_frame(&frame, &info1d)) {
                        result->info1.detected = true;
                        result->info1.sample_offset = frame_sample;
                        result->info1.duration_samples = phase2_ch1_bursts[b].duration_samples;
                        result->info1.is_info1d = true;
                        result->info1.kind = p12_classify_info1_kind(result, true);
                        result->info1.frame = frame;
                        result->info1.info1d = info1d;
                        if (result->info1.sample_offset + result->info1.duration_samples > phase2_end_hint)
                            phase2_end_hint = result->info1.sample_offset + result->info1.duration_samples;
                        if (p12_debug_enabled()) {
                            fprintf(stderr,
                                    "[p12] INFO1d hit target=%d sample=%.1fms\n",
                                    target,
                                    (double) result->info1.sample_offset * 1000.0 / (double) sample_rate);
                        }
                    } else if (p12_debug_enabled()) {
                        fprintf(stderr, "[p12] INFO1c/INFO1d frame candidate failed parse target=%d\n", target);
                    }
                } else {
                    /* INFO1a */
                    v34_v90_info1a_t raw;
                    v90_info1a_t mapped;
                    v34_info1a_generic_t info1a;

                    if (v34_info_parse_info1a_v34_frame(&frame, &info1a)) {
                        result->info1.detected = true;
                        result->info1.sample_offset = frame_sample;
                        result->info1.duration_samples = phase2_ch1_bursts[b].duration_samples;
                        result->info1.is_info1d = false;
                        result->info1.kind = P12_INFO1_KIND_V34_INFO1A;
                        result->info1.frame = frame;
                        result->info1.v34_info1a = info1a;
                        if (result->info1.sample_offset + result->info1.duration_samples > phase2_end_hint)
                            phase2_end_hint = result->info1.sample_offset + result->info1.duration_samples;
                        if (p12_debug_enabled()) {
                            fprintf(stderr,
                                    "[p12] INFO1a(V.34) hit target=%d sample=%.1fms\n",
                                    target,
                                    (double) result->info1.sample_offset * 1000.0 / (double) sample_rate);
                        }
                    } else if (v34_info_parse_info1a_v90_frame(&frame, &raw, &mapped)) {
                        result->info1.detected = true;
                        result->info1.sample_offset = frame_sample;
                        result->info1.duration_samples = phase2_ch1_bursts[b].duration_samples;
                        result->info1.is_info1d = false;
                        result->info1.kind = p12_classify_info1_kind(result, false);
                        result->info1.frame = frame;
                        result->info1.info1a_raw = raw;
                        result->info1.info1a_parsed = mapped;
                        if (result->info1.sample_offset + result->info1.duration_samples > phase2_end_hint)
                            phase2_end_hint = result->info1.sample_offset + result->info1.duration_samples;
                        if (p12_debug_enabled()) {
                            fprintf(stderr,
                                    "[p12] INFO1a(V.90) hit target=%d sample=%.1fms\n",
                                    target,
                                    (double) result->info1.sample_offset * 1000.0 / (double) sample_rate);
                        }
                    } else if (p12_debug_enabled()) {
                        fprintf(stderr, "[p12] INFO1a(V.34/V.90) frame candidate failed parse target=%d\n", target);
                    }
                }
            } else if (p12_debug_enabled()) {
                fprintf(stderr, "[p12] INFO1 no frame target=%d\n", target);
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
        phase2_start = result->answer_tone.start_sample
                     + result->answer_tone.duration_samples
                     + p12_answer_tone_post_hold_samples(&result->answer_tone, 8000);
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
    if (!result->info0.detected && result->info0_from_cj_hint.valid)
        phase2_start = p12_first_non_negative(phase2_start, result->info0_from_cj_hint.window_start_sample);
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
    if (result->tone_b.detected) {
        phase2_start = p12_first_non_negative(phase2_start, result->tone_b.start_sample);
        phase2_end = p12_first_non_negative(phase2_end,
                                            result->tone_b.start_sample + result->tone_b.duration_samples);
    }
    if (result->tone_a.detected) {
        phase2_start = p12_first_non_negative(phase2_start, result->tone_a.start_sample);
        phase2_end = p12_first_non_negative(phase2_end,
                                            result->tone_a.start_sample + result->tone_a.duration_samples);
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
    if (result->info1.detected && result->info1.kind == P12_INFO1_KIND_V90_INFO1A) {
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

    memset(&result->phase2_state, 0, sizeof(result->phase2_state));
    result->phase2_state.valid = true;
    result->phase2_state.handoff_sample = phase2_start;
    result->phase2_state.info0_seen = result->info0.detected;
    result->phase2_state.info1_seen = result->info1.detected;
    result->phase2_state.tone_a_seen = result->tone_a.detected;
    result->phase2_state.tone_b_seen = result->tone_b.detected;
    result->phase2_state.l1_l2_seen = (result->probe_tone_count > 0);
    result->phase2_state.info0_sample = result->info0.detected ? result->info0.sample_offset : -1;
    result->phase2_state.tone_a_sample = result->tone_a.detected ? result->tone_a.start_sample : -1;
    result->phase2_state.tone_b_sample = result->tone_b.detected ? result->tone_b.start_sample : -1;
    result->phase2_state.l1_l2_sample = (result->probe_tone_count > 0) ? result->probe_tones[0].start_sample : -1;
    result->phase2_state.info1_sample = result->info1.detected ? result->info1.sample_offset : -1;
    result->phase2_state.info0_hint = result->info0_from_cj_hint;
    if (!result->phase2_state.info0_hint.valid
        && phase2_start >= 0
        && !result->phase2_state.info0_seen) {
        p12_fill_handoff_info0_hint(&result->phase2_state.info0_hint,
                                    result->phase2_state.role,
                                    phase2_start,
                                    phase2_start + (8000 * P12_INFO_RETRY_WINDOW_MS) / 1000,
                                    NULL,
                                    0,
                                    8000);
    }

    if (result->v90_capable && !result->is_caller) {
        result->phase2_state.role = P12_PHASE2_ROLE_V90_DIGITAL_ANSWERER;
    } else if (result->v90_capable && result->is_caller) {
        result->phase2_state.role = P12_PHASE2_ROLE_V90_ANALOG_CALLER;
    } else if (result->role_detected && result->is_caller) {
        result->phase2_state.role = P12_PHASE2_ROLE_V34_CALLER;
    } else if (result->role_detected && !result->is_caller) {
        result->phase2_state.role = P12_PHASE2_ROLE_V34_ANSWERER;
    } else {
        result->phase2_state.role = P12_PHASE2_ROLE_UNKNOWN;
    }

    if (!result->phase2_state.info0_seen) {
        result->phase2_state.next_step = P12_PHASE2_STEP_EXPECT_INFO0;
    } else if (!result->phase2_state.tone_a_seen && !result->phase2_state.tone_b_seen) {
        result->phase2_state.next_step = P12_PHASE2_STEP_EXPECT_TONE_REVERSAL;
    } else if (!result->phase2_state.l1_l2_seen) {
        result->phase2_state.next_step = P12_PHASE2_STEP_EXPECT_L1_L2;
    } else if (!result->phase2_state.info1_seen) {
        result->phase2_state.next_step = P12_PHASE2_STEP_EXPECT_INFO1;
    } else {
        result->phase2_state.next_step = P12_PHASE2_STEP_COMPLETE;
    }

    if (result->phase2_state.info0_seen && !result->phase2_state.info1_seen) {
        memset(&result->phase2_state.info1_hint, 0, sizeof(result->phase2_state.info1_hint));
        result->phase2_state.info1_hint.valid = true;
        if (result->tone_a.detected)
            result->phase2_state.info1_hint.anchor_sample = result->tone_a.start_sample;
        else if (result->tone_b.detected)
            result->phase2_state.info1_hint.anchor_sample = result->tone_b.start_sample;
        else if (result->probe_tone_count > 0)
            result->phase2_state.info1_hint.anchor_sample = result->probe_tones[0].start_sample;
        else
            result->phase2_state.info1_hint.anchor_sample = result->info0.sample_offset;
        result->phase2_state.info1_hint.expected_sample = result->phase2_state.info1_hint.anchor_sample;
        result->phase2_state.info1_hint.window_start_sample = result->phase2_state.info1_hint.anchor_sample;
        result->phase2_state.info1_hint.window_end_sample = result->phase2_state.info1_hint.anchor_sample + 8000/3;
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

static void emit_tone_event(call_log_t *log, const p12_tone_hit_t *tone,
                            const char *protocol, bool is_caller)
{
    char summary[160];
    char detail[256];

    if (!tone->detected)
        return;

    snprintf(summary, sizeof(summary), "%s tone detected", p12_tone_type_name(tone->type));
    /* Include role= and tone= so the HTML renderer can apply its specialized
     * V.8 tone template (same keys as the spandsp-based decode path). */
    snprintf(detail, sizeof(detail), "role=%s tone=%s peak_ratio=%.3f duration=%.1fms",
             is_caller ? "caller" : "answerer",
             p12_tone_type_name(tone->type),
             tone->peak_ratio,
             (double) tone->duration_samples * 1000.0 / 8000.0);
    call_log_append(log, tone->start_sample, tone->duration_samples,
                    protocol, summary, detail);
}

static void emit_answer_tone_handoff_event(call_log_t *log,
                                           const p12_tone_hit_t *tone)
{
    char detail[256];
    int handoff_sample;

    if (!log || !tone || !tone->detected)
        return;

    handoff_sample = tone->start_sample + tone->duration_samples;
    snprintf(detail, sizeof(detail),
             "tone=%s start=%.1fms end=%.1fms duration=%.1fms",
             p12_tone_type_name(tone->type),
             (double) tone->start_sample * 1000.0 / 8000.0,
             (double) handoff_sample * 1000.0 / 8000.0,
             (double) tone->duration_samples * 1000.0 / 8000.0);
    call_log_append(log,
                    handoff_sample,
                    0,
                    "V.8",
                    "Answer-tone handoff",
                    detail);
}

static void emit_cm_jm_event(call_log_t *log, const p12_cm_jm_hit_t *msg,
                              const char *label, bool is_caller)
{
    char summary[160];
    char detail[1024];
    char modbuf[256];
    char hexbuf[192];
    size_t dlen;
    int i;

    if (!msg->detected)
        return;

    /* Build modulation string (comma-separated, no spaces — HTML renderer
     * adds ", " when displaying). */
    static const struct { int flag; const char *name; } mod_table[] = {
        { P12_MOD_V17, "V.17" }, { P12_MOD_V21, "V.21" },
        { P12_MOD_V22, "V.22bis" }, { P12_MOD_V23, "V.23" },
        { P12_MOD_V27TER, "V.27ter" }, { P12_MOD_V29, "V.29" },
        { P12_MOD_V32, "V.32bis" }, { P12_MOD_V34, "V.34" },
        { P12_MOD_V90, "V.90" }, { P12_MOD_V92, "V.92" }
    };
    modbuf[0] = '\0';
    for (i = 0; i < (int)(sizeof(mod_table) / sizeof(mod_table[0])); i++) {
        if (msg->modulations & mod_table[i].flag) {
            dlen = strlen(modbuf);
            snprintf(modbuf + dlen, sizeof(modbuf) - dlen,
                     "%s%s", modbuf[0] ? "," : "", mod_table[i].name);
        }
    }
    if (!modbuf[0])
        snprintf(modbuf, sizeof(modbuf), "none");

    /* Hex-encode raw bytes for the raw= field */
    hexbuf[0] = '\0';
    for (i = 0; i < msg->byte_count && i < 32; i++) {
        dlen = strlen(hexbuf);
        snprintf(hexbuf + dlen, sizeof(hexbuf) - dlen,
                 "%s%02X", i ? "_" : "", msg->bytes[i]);
    }

    /* Use the same key names as the spandsp decode path so the HTML renderer
     * (which looks for role=, msg=, confidence=, call_fn=, modulations=,
     * protocol=, pcm=, pstn=, raw=) can apply its CM/JM template. */
    snprintf(summary, sizeof(summary), "%s decoded", label);
    snprintf(detail, sizeof(detail),
             "role=%s msg=%s confidence=%s call_fn=%d modulations=%s"
             " protocol=%d pcm=%d pstn=%d",
             is_caller ? "caller" : "answerer",
             label,
             msg->complete ? "confirmed" : "candidate_fragment",
             msg->call_function,
             modbuf,
             msg->protocols,
             msg->pcm_modem_availability,
             msg->pstn_access);
    if (hexbuf[0]) {
        dlen = strlen(detail);
        snprintf(detail + dlen, sizeof(detail) - dlen, " raw=%s", hexbuf);
    }

    call_log_append(log, msg->sample_offset, msg->duration_samples,
                    "V.8", summary, detail);
}

const char *phase12_phase2_role_name(p12_phase2_role_t role)
{
    switch (role) {
    case P12_PHASE2_ROLE_V34_CALLER: return "v34_caller";
    case P12_PHASE2_ROLE_V34_ANSWERER: return "v34_answerer";
    case P12_PHASE2_ROLE_V90_ANALOG_CALLER: return "v90_analog_caller";
    case P12_PHASE2_ROLE_V90_DIGITAL_ANSWERER: return "v90_digital_answerer";
    default: return "unknown";
    }
}

const char *phase12_phase2_step_name(p12_phase2_step_t step)
{
    switch (step) {
    case P12_PHASE2_STEP_EXPECT_INFO0: return "expect_info0";
    case P12_PHASE2_STEP_EXPECT_TONE_REVERSAL: return "expect_tone_reversal";
    case P12_PHASE2_STEP_EXPECT_L1_L2: return "expect_l1_l2";
    case P12_PHASE2_STEP_EXPECT_INFO1: return "expect_info1";
    case P12_PHASE2_STEP_COMPLETE: return "complete";
    default: return "unknown";
    }
}

const char *phase12_info0_kind_name(p12_info0_kind_t kind)
{
    switch (kind) {
    case P12_INFO0_KIND_SHARED_INFO0A: return "shared_info0a";
    case P12_INFO0_KIND_V34_INFO0C: return "v34_info0c";
    case P12_INFO0_KIND_V90_INFO0D: return "v90_info0d";
    case P12_INFO0_KIND_V92_SHORT: return "v92_short";
    default: return "unknown";
    }
}

const char *phase12_info1_kind_name(p12_info1_kind_t kind)
{
    switch (kind) {
    case P12_INFO1_KIND_V34_INFO1A: return "v34_info1a";
    case P12_INFO1_KIND_V34_INFO1C: return "v34_info1c";
    case P12_INFO1_KIND_V90_INFO1A: return "v90_info1a";
    case P12_INFO1_KIND_V90_INFO1D: return "v90_info1d";
    case P12_INFO1_KIND_V92_SHORT: return "v92_short";
    default: return "unknown";
    }
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
    detect_call_initiation_signals(samples, total_samples, sample_rate, max_sample, result);
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

    /* Phase 1 tones */
    emit_tone_event(log, &result->cng, "V.8", result->is_caller);
    emit_tone_event(log, &result->ct, "V.8", result->is_caller);
    emit_tone_event(log, &result->answer_tone, "V.8", result->is_caller);
    emit_answer_tone_handoff_event(log, &result->answer_tone);
    if (result->call_init.v8bis_signal_seen) {
        char summary[160];
        char detail[256];

        snprintf(summary, sizeof(summary), "%s%s detected before ANS",
                 result->call_init.v8bis_signal_weak ? "Weak " : "",
                 result->call_init.v8bis_signal_name[0] ? result->call_init.v8bis_signal_name : "V.8bis");
        snprintf(detail, sizeof(detail),
                 "duration=%.1fms weak=%s",
                 (double) result->call_init.v8bis_signal_duration * 1000.0 / (double) sample_rate,
                 result->call_init.v8bis_signal_weak ? "yes" : "no");
        call_log_append(log,
                        result->call_init.v8bis_signal_sample,
                        result->call_init.v8bis_signal_duration,
                        result->call_init.v92_qc2_seen ? "V.92" : "V.8bis",
                        summary,
                        detail);
    }
    if (result->call_init.v92_qc2_seen) {
        call_log_append(log,
                        result->call_init.v92_qc2_sample,
                        0,
                        "V.92",
                        result->call_init.v92_qc2_name,
                        "source=call_initiation");
    }

    /* Phase 1 V.8 messages */
    emit_cm_jm_event(log, &result->cm, "CM", result->is_caller);
    emit_cm_jm_event(log, &result->jm, "JM", result->is_caller);

    if (result->cj.detected) {
        char cj_detail[64];
        snprintf(cj_detail, sizeof(cj_detail), "role=%s",
                 result->is_caller ? "caller" : "answerer");
        call_log_append(log, result->cj.sample_offset, 0,
                        "V.8", "CJ decoded", cj_detail);
    }

    /* Phase 2 INFO frames */
    if (result->info0.detected) {
        char summary[160];
        char detail[1024];
        const char *role_name = result->role_detected
            ? (result->is_caller ? "caller" : "answerer") : "unknown";

        detail[0] = '\0';
        snprintf(summary, sizeof(summary), "%s decoded",
                 result->info0.kind == P12_INFO0_KIND_V34_INFO0C ? "INFO0c"
                 : (result->info0.kind == P12_INFO0_KIND_V90_INFO0D ? "INFO0d" : "INFO0a"));
        appendf(detail, sizeof(detail), "role=%s", role_name);
        appendf(detail, sizeof(detail), " kind=%s", phase12_info0_kind_name(result->info0.kind));
        appendf(detail, sizeof(detail), " profile=%s",
                result->info0.kind == P12_INFO0_KIND_V90_INFO0D ? "V90V92_INFO0d_T7_T15"
                : (result->info0.kind == P12_INFO0_KIND_V34_INFO0C ? "V34_INFO0c"
                   : "V34V90_SHARED_INFO0a"));
        appendf(detail, sizeof(detail), " total_bits=%d", result->info0.frame.total_bits);
        appendf(detail, sizeof(detail), " fsync=0x%03X", V90_INFO_FILL_AND_SYNC_BITS);
        appendf(detail, sizeof(detail), " b12_2743=%u",
                result->info0.parsed.support_2743 ? 1U : 0U);
        appendf(detail, sizeof(detail), " b13_2800=%u",
                result->info0.parsed.support_2800 ? 1U : 0U);
        appendf(detail, sizeof(detail), " b14_3429=%u",
                result->info0.parsed.support_3429 ? 1U : 0U);
        appendf(detail, sizeof(detail), " b15_3000l=%u",
                result->info0.parsed.support_3000_low ? 1U : 0U);
        appendf(detail, sizeof(detail), " b16_3000h=%u",
                result->info0.parsed.support_3000_high ? 1U : 0U);
        appendf(detail, sizeof(detail), " b17_3200l=%u",
                result->info0.parsed.support_3200_low ? 1U : 0U);
        appendf(detail, sizeof(detail), " b18_3200h=%u",
                result->info0.parsed.support_3200_high ? 1U : 0U);
        appendf(detail, sizeof(detail), " b19_3429ok=%u",
                result->info0.parsed.rate_3429_allowed ? 1U : 0U);
        appendf(detail, sizeof(detail), " b20_pwrred=%u",
                result->info0.parsed.support_power_reduction ? 1U : 0U);
        appendf(detail, sizeof(detail), " b21_23_maxbaud=%u",
                (unsigned) result->info0.parsed.max_baud_rate_difference);
        appendf(detail, sizeof(detail), " b24_cme=%u",
                result->info0.parsed.from_cme_modem ? 1U : 0U);
        appendf(detail, sizeof(detail), " b25_1664pt=%u",
                result->info0.parsed.support_1664_point_constellation ? 1U : 0U);
        if (result->info0.is_info0d) {
            appendf(detail, sizeof(detail), " b26_shortp2_req=%u",
                    (unsigned) (result->info0.raw.raw_26_27 & 0x01U));
            appendf(detail, sizeof(detail), " b27_v92_cap=%u",
                    (unsigned) ((result->info0.raw.raw_26_27 >> 1) & 0x01U));
            appendf(detail, sizeof(detail), " b28_ack_i0a=%u",
                    result->info0.parsed.acknowledge_info0d ? 1U : 0U);
            appendf(detail, sizeof(detail), " crc16=unavailable tail=0xF");
        } else {
            if (result->info0.kind == P12_INFO0_KIND_V34_INFO0C
                || result->info0.kind == P12_INFO0_KIND_SHARED_INFO0A) {
                appendf(detail, sizeof(detail), " b26_27_txclk=%u",
                        (unsigned) (result->info0.raw.raw_26_27 & 0x03U));
                appendf(detail, sizeof(detail), " b28_ack=%u",
                        result->info0.parsed.acknowledge_info0d ? 1U : 0U);
                appendf(detail, sizeof(detail), " crc16=frame_valid tail=0xF");
            } else {
                v90_info0a_diag_t diag;
                bool have_crc = v90_info0a_build_diag(&result->info0.parsed, &diag);
                appendf(detail, sizeof(detail), " b26_v92_cap=%u",
                        (unsigned) (result->info0.raw.raw_26_27 & 0x01U));
                appendf(detail, sizeof(detail), " b27_shortp2_req=%u",
                        (unsigned) ((result->info0.raw.raw_26_27 >> 1) & 0x01U));
                appendf(detail, sizeof(detail), " b28_ack_i0d=%u",
                        result->info0.parsed.acknowledge_info0d ? 1U : 0U);
                if (have_crc)
                    appendf(detail, sizeof(detail), " crc16=0x%04X", (unsigned) diag.crc_field);
                else
                    appendf(detail, sizeof(detail), " crc16=n/a");
                appendf(detail, sizeof(detail), " tail=0xF");
            }
        }
        call_log_append(log, result->info0.sample_offset,
                        result->info0.duration_samples,
                        "V.34", summary, detail);
    }

    if (result->info1.detected) {
        char summary[160];
        char detail[1024];
        const char *role_name = result->role_detected
            ? (result->is_caller ? "caller" : "answerer") : "unknown";

        detail[0] = '\0';
        snprintf(summary, sizeof(summary), "%s decoded",
                 result->info1.is_info1d ? "INFO1d" : "INFO1");
        appendf(detail, sizeof(detail), " kind=%s", phase12_info1_kind_name(result->info1.kind));
        appendf(detail, sizeof(detail), " total_bits=%d", result->info1.frame.total_bits);
        if (result->info1.kind == P12_INFO1_KIND_V34_INFO1C) {
            const v34_info1c_generic_t *i1c = &result->info1.v34_info1c;

            appendf(detail, sizeof(detail), "role=%s", role_name);
            appendf(detail, sizeof(detail), " table=V34_INFO1c");
            appendf(detail, sizeof(detail), " pwr_red=%d", i1c->power_reduction);
            appendf(detail, sizeof(detail), " addl_pwr_red=%d", i1c->additional_power_reduction);
            appendf(detail, sizeof(detail), " md=%d", i1c->md);
            for (int i = 0; i < 6; i++) {
                appendf(detail, sizeof(detail),
                        " row%d_hicar=%u row%d_preemp=%d row%d_maxrate=%d",
                        i,
                        i1c->rate_data[i].use_high_carrier ? 1U : 0U,
                        i,
                        i1c->rate_data[i].pre_emphasis,
                        i,
                        i1c->rate_data[i].max_bit_rate);
            }
            appendf(detail, sizeof(detail), " freq_offset=%d", i1c->freq_offset);
        } else if (result->info1.kind == P12_INFO1_KIND_V34_INFO1A) {
            const v34_info1a_generic_t *i1a = &result->info1.v34_info1a;

            appendf(detail, sizeof(detail), "role=%s", role_name);
            appendf(detail, sizeof(detail), " table=V34_INFO1a");
            appendf(detail, sizeof(detail), " pwr_red=%d", i1a->power_reduction);
            appendf(detail, sizeof(detail), " addl_pwr_red=%d", i1a->additional_power_reduction);
            appendf(detail, sizeof(detail), " md=%d", i1a->md);
            appendf(detail, sizeof(detail), " hi_carrier=%u", i1a->use_high_carrier ? 1U : 0U);
            appendf(detail, sizeof(detail), " preemp=%d", i1a->preemphasis_filter);
            appendf(detail, sizeof(detail), " max_data_rate=%d", i1a->max_data_rate);
            appendf(detail, sizeof(detail), " baud_a_to_c=%d", i1a->baud_rate_a_to_c);
            appendf(detail, sizeof(detail), " baud_c_to_a=%d", i1a->baud_rate_c_to_a);
            appendf(detail, sizeof(detail), " freq_offset=%d", i1a->freq_offset);
        } else if (result->info1.is_info1d) {
            const v34_v90_info1d_t *i1d = &result->info1.info1d;
            appendf(detail, sizeof(detail), "role=%s", role_name);
            appendf(detail, sizeof(detail), " table=V90_T9_or_V92_T17_INFO1d");
            appendf(detail, sizeof(detail), " fsync=0x%03X", V90_INFO_FILL_AND_SYNC_BITS);
            appendf(detail, sizeof(detail), " b12_14_min_pwr_red=%d",
                    i1d->power_reduction);
            appendf(detail, sizeof(detail), " b15_17_addl_pwr_red=%d",
                    i1d->additional_power_reduction);
            appendf(detail, sizeof(detail), " b18_24_md=%d", i1d->md);
            appendf(detail, sizeof(detail),
                    " row2400_hicar=%u row2400_preemp=%d row2400_maxrate=%d",
                    i1d->rate_data[0].use_high_carrier ? 1U : 0U,
                    i1d->rate_data[0].pre_emphasis, i1d->rate_data[0].max_bit_rate);
            appendf(detail, sizeof(detail),
                    " row2743_hicar=%u row2743_preemp=%d row2743_maxrate=%d",
                    i1d->rate_data[1].use_high_carrier ? 1U : 0U,
                    i1d->rate_data[1].pre_emphasis, i1d->rate_data[1].max_bit_rate);
            appendf(detail, sizeof(detail),
                    " row2800_hicar=%u row2800_preemp=%d row2800_maxrate=%d",
                    i1d->rate_data[2].use_high_carrier ? 1U : 0U,
                    i1d->rate_data[2].pre_emphasis, i1d->rate_data[2].max_bit_rate);
            appendf(detail, sizeof(detail),
                    " row3000_hicar=%u row3000_preemp=%d row3000_maxrate=%d",
                    i1d->rate_data[3].use_high_carrier ? 1U : 0U,
                    i1d->rate_data[3].pre_emphasis, i1d->rate_data[3].max_bit_rate);
            appendf(detail, sizeof(detail),
                    " row3200_hicar=%u row3200_preemp=%d row3200_maxrate=%d",
                    i1d->rate_data[4].use_high_carrier ? 1U : 0U,
                    i1d->rate_data[4].pre_emphasis, i1d->rate_data[4].max_bit_rate);
            appendf(detail, sizeof(detail),
                    " row3429_hicar_or_v92b70=%u row3429_preemp_or_v92b71_74=%d row3429_maxrate_or_v92b75_78=%d",
                    i1d->rate_data[5].use_high_carrier ? 1U : 0U,
                    i1d->rate_data[5].pre_emphasis, i1d->rate_data[5].max_bit_rate);
            appendf(detail, sizeof(detail), " freq_offset_10b=0x%03X(%+d)",
                    (unsigned) ((i1d->freq_offset < 0)
                        ? (0x400 + i1d->freq_offset) : i1d->freq_offset) & 0x3FFU,
                    i1d->freq_offset);
            appendf(detail, sizeof(detail), " crc16=unavailable tail=0xF");
        } else {
            const v90_info1a_t *i1a = &result->info1.info1a_parsed;
            const v34_v90_info1a_t *r1a = &result->info1.info1a_raw;
            v90_info1a_diag_t diag;
            bool have_crc = v90_info1a_build_diag(i1a, &diag);
            bool looks_v92 = (r1a->upstream_symbol_rate_code == 6
                              && r1a->downstream_rate_code == 6);
            uint16_t freq_raw = (i1a->freq_offset < 0)
                ? (uint16_t)(0x400 + i1a->freq_offset)
                : (uint16_t) i1a->freq_offset;

            appendf(detail, sizeof(detail), "role=%s", role_name);
            appendf(detail, sizeof(detail), " table=%s",
                    looks_v92 ? "V92_T18_INFO1a_pcm_upstream"
                              : "V90_T10_or_V92_T19_INFO1a_v34_upstream");
            appendf(detail, sizeof(detail), " fsync=0x%03X", V90_INFO_FILL_AND_SYNC_BITS);
            appendf(detail, sizeof(detail), " b12_17_raw=%u",
                    (unsigned) r1a->raw_12_17);
            appendf(detail, sizeof(detail), " md=%u", (unsigned) i1a->md);
            appendf(detail, sizeof(detail), " u_info=%u", (unsigned) i1a->u_info);
            appendf(detail, sizeof(detail), " b32_33_raw=%u",
                    (unsigned) r1a->raw_32_33);
            appendf(detail, sizeof(detail), " upstream_symbol_rate_code=%u",
                    (unsigned) i1a->upstream_symbol_rate_code);
            appendf(detail, sizeof(detail), " downstream_rate_code=%u",
                    (unsigned) i1a->downstream_rate_code);
            appendf(detail, sizeof(detail), " freq_offset_10b=0x%03X(%+d)",
                    (unsigned)(freq_raw & 0x3FFU), i1a->freq_offset);
            if (have_crc)
                appendf(detail, sizeof(detail), " crc16=0x%04X", (unsigned) diag.crc_field);
            else
                appendf(detail, sizeof(detail), " crc16=n/a");
            appendf(detail, sizeof(detail), " tail=0xF");
        }
        call_log_append(log, result->info1.sample_offset,
                        result->info1.duration_samples,
                        "V.34", summary, detail);
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
        if (result->answer_tone.detected) {
            size_t dlen = strlen(detail);
            snprintf(detail + dlen, sizeof(detail) - dlen,
                     " ans=%s ans_start=%.1fms ans_end=%.1fms ans_dur=%.1fms",
                     p12_tone_type_name(result->answer_tone.type),
                     (double) result->answer_tone.start_sample * 1000.0 / (double) sample_rate,
                     (double) (result->answer_tone.start_sample + result->answer_tone.duration_samples) * 1000.0 / (double) sample_rate,
                     (double) result->answer_tone.duration_samples * 1000.0 / (double) sample_rate);
        }
        if (result->call_init.v8bis_signal_seen) {
            size_t dlen = strlen(detail);
            snprintf(detail + dlen, sizeof(detail) - dlen,
                     " pre_v8bis=%s%s@%.1fms",
                     result->call_init.v8bis_signal_weak ? "weak_" : "",
                     result->call_init.v8bis_signal_name,
                     (double) result->call_init.v8bis_signal_sample * 1000.0 / (double) sample_rate);
        }
        if (result->call_init.v92_qc2_seen) {
            size_t dlen = strlen(detail);
            snprintf(detail + dlen, sizeof(detail) - dlen,
                     " qc=%s@%.1fms",
                     result->call_init.v92_qc2_name,
                     (double) result->call_init.v92_qc2_sample * 1000.0 / (double) sample_rate);
        }
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
    if (result->phase2_state.valid) {
        size_t dlen = strlen(detail);
        snprintf(detail + dlen, sizeof(detail) - dlen,
                 " phase2_role=%s next=%s",
                 phase12_phase2_role_name(result->phase2_state.role),
                 phase12_phase2_step_name(result->phase2_state.next_step));
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

    if (result->tone_b.detected) {
        char detail[192];

        snprintf(detail, sizeof(detail),
                 "freq=%.0f reversal=%s",
                 result->tone_b.freq_hz,
                 result->tone_b.phase_reversal_seen ? "yes" : "no");
        call_log_append(log, result->tone_b.start_sample,
                        result->tone_b.duration_samples,
                        "V.34", "Tone B detected", detail);
    }

    if (result->tone_a.detected) {
        char detail[192];

        snprintf(detail, sizeof(detail),
                 "freq=%.0f reversal=%s",
                 result->tone_a.freq_hz,
                 result->tone_a.phase_reversal_seen ? "yes" : "no");
        call_log_append(log, result->tone_a.start_sample,
                        result->tone_a.duration_samples,
                        "V.34", "Tone A detected", detail);
    }
}
