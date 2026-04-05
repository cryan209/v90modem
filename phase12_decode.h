/*
 * phase12_decode.h — Standalone V.PCM Phase 1 and Phase 2 offline decoder
 *
 * Decodes modem Phase 1 (V.8 negotiation) and Phase 2 (V.34 probing / INFO
 * exchange) signals from an audio stream without using spandsp for signal
 * detection or FSK demodulation.
 *
 * Phase 1 signals decoded:
 *   - CNG (1100 Hz calling tone)
 *   - CT (1300 Hz calling tone)
 *   - ANS/ANSam/ANS_PR/ANSam_PR (2100 Hz answer tones)
 *   - CI (V.21 CH1 FSK initiating sequence)
 *   - CM (V.21 CH1 FSK capability message)
 *   - JM (V.21 CH2 FSK joint menu response)
 *   - CJ (V.21 CH2 FSK all-marks acknowledgement)
 *
 * Phase 2 signals decoded:
 *   - INFO0a/INFO0d (V.21 CH2 FSK, V.34 info frames)
 *   - INFO1a/INFO1d (V.21 CH1 FSK, V.34 info frames)
 *   - Line probing tones (L1/L2 at 2743-3429 Hz)
 *   - Tone A / Tone B with phase reversal detection
 */

#ifndef PHASE12_DECODE_H
#define PHASE12_DECODE_H

#include "v8bis_decode.h"   /* call_log_t, call_log_event_t */
#include "v90.h"            /* v90_info0a_t, v90_info1a_t, v34_v90_info* */
#include "v34_info_decode.h"
#include "v91.h"
#include "v92_short_phase2_decode.h"

#include <stdbool.h>
#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Phase 1 tone detection results                                      */
/* ------------------------------------------------------------------ */

typedef enum {
    P12_TONE_NONE = 0,
    P12_TONE_CNG,              /* 1100 Hz with cadence */
    P12_TONE_CT,               /* 1300 Hz continuous */
    P12_TONE_ANS,              /* 2100 Hz, no phase reversal, no AM */
    P12_TONE_ANS_PR,           /* 2100 Hz with phase reversals */
    P12_TONE_ANSAM,            /* 2100 Hz with 15 Hz AM */
    P12_TONE_ANSAM_PR          /* 2100 Hz with 15 Hz AM + phase reversals */
} p12_tone_type_t;

typedef struct {
    bool detected;
    p12_tone_type_t type;
    int start_sample;
    int duration_samples;
    double peak_ratio;         /* peak Goertzel energy ratio */
} p12_tone_hit_t;

/* ------------------------------------------------------------------ */
/* Phase 1 V.8 message results                                        */
/* ------------------------------------------------------------------ */

typedef struct {
    bool detected;
    int sample_offset;
    int bit_count;
} p12_ci_hit_t;

typedef struct {
    bool detected;
    bool complete;             /* repeated / CRC-confirmed */
    int sample_offset;
    int duration_samples;
    int last_sample_offset;
    int last_duration_samples;
    int byte_count;
    uint8_t bytes[64];
    int observed_count;
    int differing_count;
    int saved_count;
    int saved_sample_offsets[8];
    int saved_byte_counts[8];
    uint8_t saved_bytes[8][64];
    /* Parsed capability fields */
    int call_function;
    int modulations;           /* bitmask, matches V8_MOD_* values */
    int protocols;
    int pstn_access;
    int pcm_modem_availability;
} p12_cm_jm_hit_t;

typedef struct {
    bool detected;
    int sample_offset;
    int duration_samples;
} p12_cj_hit_t;

/* ------------------------------------------------------------------ */
/* Phase 2 INFO results                                                */
/* ------------------------------------------------------------------ */

typedef enum {
    P12_INFO0_KIND_UNKNOWN = 0,
    P12_INFO0_KIND_SHARED_INFO0A,
    P12_INFO0_KIND_V34_INFO0C,
    P12_INFO0_KIND_V90_INFO0D,
    P12_INFO0_KIND_V92_SHORT
} p12_info0_kind_t;

typedef enum {
    P12_INFO1_KIND_UNKNOWN = 0,
    P12_INFO1_KIND_V34_INFO1A,
    P12_INFO1_KIND_V34_INFO1C,
    P12_INFO1_KIND_V90_INFO1A,
    P12_INFO1_KIND_V90_INFO1D,
    P12_INFO1_KIND_V92_SHORT
} p12_info1_kind_t;

typedef struct {
    bool detected;
    int sample_offset;
    int duration_samples;
    bool is_info0d;            /* true if INFO0d (digital side) */
    p12_info0_kind_t kind;
    v34_info_frame_t frame;
    v34_v90_info0a_t raw;
    v90_info0a_t parsed;
} p12_info0_hit_t;

typedef struct {
    bool detected;
    int sample_offset;
    int duration_samples;
    bool is_info1d;            /* true if INFO1d */
    p12_info1_kind_t kind;
    v34_info_frame_t frame;
    v34_info1a_generic_t v34_info1a;
    v34_info1c_generic_t v34_info1c;
    v34_v90_info1a_t info1a_raw;
    v90_info1a_t info1a_parsed;
    v34_v90_info1d_t info1d;
} p12_info1_hit_t;

/* ------------------------------------------------------------------ */
/* Phase 2 line probing tone results                                   */
/* ------------------------------------------------------------------ */

typedef struct {
    bool detected;
    int start_sample;
    int duration_samples;
    double freq_hz;
    double peak_ratio;
} p12_probe_tone_hit_t;

#define P12_MAX_PROBE_TONES 8

typedef struct {
    bool detected;
    int start_sample;
    int duration_samples;
    double freq_hz;
    bool phase_reversal_seen;
    int reversal_sample;
} p12_signal_tone_hit_t;

typedef struct {
    bool valid;
    bool used_for_retry;
    int anchor_sample;
    int expected_sample;
    int window_start_sample;
    int window_end_sample;
    int retry_count;
} p12_timing_hint_t;

typedef enum {
    P12_PHASE2_ROLE_UNKNOWN = 0,
    P12_PHASE2_ROLE_V34_CALLER,
    P12_PHASE2_ROLE_V34_ANSWERER,
    P12_PHASE2_ROLE_V90_ANALOG_CALLER,
    P12_PHASE2_ROLE_V90_DIGITAL_ANSWERER
} p12_phase2_role_t;

typedef enum {
    P12_PHASE2_STEP_UNKNOWN = 0,
    P12_PHASE2_STEP_EXPECT_INFO0,
    P12_PHASE2_STEP_EXPECT_TONE_REVERSAL,
    P12_PHASE2_STEP_EXPECT_L1_L2,
    P12_PHASE2_STEP_EXPECT_INFO1,
    P12_PHASE2_STEP_COMPLETE
} p12_phase2_step_t;

typedef struct {
    bool valid;
    p12_phase2_role_t role;
    p12_phase2_step_t next_step;
    bool info0_seen;
    bool info1_seen;
    bool tone_a_seen;
    bool tone_b_seen;
    bool l1_l2_seen;
    int handoff_sample;
    int info0_sample;
    int tone_a_sample;
    int tone_b_sample;
    int l1_l2_sample;
    int info1_sample;
    p12_timing_hint_t info0_hint;
    p12_timing_hint_t info1_hint;
} p12_phase2_state_t;

typedef struct {
    bool v8bis_signal_seen;
    bool v8bis_signal_weak;
    int v8bis_signal_sample;
    int v8bis_signal_duration;
    char v8bis_signal_name[16];

    bool v92_qc2_seen;
    int v92_qc2_sample;
    char v92_qc2_name[16];

    bool v92_short_p1_seen;
    int v92_short_p1_sample;
    char v92_short_p1_name[16];
    bool v92_short_p1_digital;
    bool v92_short_p1_qca;
    int v92_short_p1_uqts_ucode;

    bool v92_qts_seen;
    int v92_qts_sample;
    int v92_qts_reps;
    int v92_qts_bar_reps;

    bool v92_toneq_seen;
    int v92_toneq_sample;
    int v92_toneq_duration_samples;

    bool answer_tone_handoff_known;
    int answer_tone_handoff_sample;
} p12_call_init_t;

typedef enum {
    P12_SHORT_P1_FORM_UNKNOWN = -1,
    P12_SHORT_P1_FORM_ANALOG = 0,
    P12_SHORT_P1_FORM_DIGITAL = 1
} p12_short_p1_form_t;

/* ------------------------------------------------------------------ */
/* Phase 2 V.21 FSK burst detection                                    */
/* ------------------------------------------------------------------ */

typedef struct {
    bool seen;
    int start_sample;
    int duration_samples;
    double peak_energy;
    double peak_signal_energy;
    uint8_t hint_id;
} p12_fsk_burst_t;

#define P12_MAX_FSK_BURSTS 16

/* ------------------------------------------------------------------ */
/* Combined Phase 1 + Phase 2 decode result                            */
/* ------------------------------------------------------------------ */

typedef struct {
    /* Phase 1 tones */
    p12_tone_hit_t cng;
    p12_tone_hit_t ct;
    p12_tone_hit_t answer_tone;    /* ANS/ANSam/ANS_PR/ANSam_PR */

    /* Phase 1 V.8 messages */
    p12_ci_hit_t ci;
    p12_cm_jm_hit_t cm;           /* caller's CM */
    p12_cm_jm_hit_t jm;           /* answerer's JM */
    p12_cj_hit_t cj;

    /* Role detection */
    bool role_detected;
    bool is_caller;               /* true if this channel is the caller */

    /* Phase 2 INFO */
    p12_info0_hit_t info0;
    p12_info1_hit_t info1;

    /* Derived call diagnostics */
    bool phase2_window_known;
    int phase2_start_sample;
    int phase2_end_sample;
    bool pcm_modem_capable;
    bool v90_capable;
    bool v92_capable;
    bool short_phase2_requested;
    bool role_confident;
    bool digital_side_likely;
    bool info_path_known;
    int inferred_u_info;
    int inferred_upstream_symbol_rate_code;
    int inferred_downstream_rate_code;
    bool short_phase2_analysis_valid;
    v92_short_phase2_result_t short_phase2;

    /* Phase 2 probing tones */
    int probe_tone_count;
    p12_probe_tone_hit_t probe_tones[P12_MAX_PROBE_TONES];

    /* Phase 2 signal tones (Tone A / Tone B) */
    p12_signal_tone_hit_t tone_a;
    p12_signal_tone_hit_t tone_b;

    /* Phase 2 timing hints */
    p12_timing_hint_t info0_from_cj_hint;
    p12_phase2_state_t phase2_state;

    /* Call initiation / pre-V.8 findings */
    p12_call_init_t call_init;

    /* Optional stereo arbitration hints supplied by the caller. */
    bool stereo_short_p1_hint_valid;
    p12_short_p1_form_t stereo_short_p1_expected_form;
    bool stereo_short_p1_followup_allowed;

    /* V.21 FSK bursts detected */
    int ch1_burst_count;
    p12_fsk_burst_t ch1_bursts[P12_MAX_FSK_BURSTS];
    int ch2_burst_count;
    p12_fsk_burst_t ch2_bursts[P12_MAX_FSK_BURSTS];

    /* Call log events generated */
    call_log_t log;
} phase12_result_t;

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

/*
 * Initialize a result structure. Must be called before decode.
 */
void phase12_result_init(phase12_result_t *r);

/*
 * Free any allocations in the result structure.
 */
void phase12_result_reset(phase12_result_t *r);

/*
 * Decode Phase 1 and Phase 2 signals from a linear PCM buffer.
 *
 * samples:       16-bit linear PCM at sample_rate Hz
 * total_samples: total number of samples
 * sample_rate:   must be 8000
 * max_sample:    limit scan to this many samples (0 = no limit)
 *
 * Returns true if any signals were detected.
 */
bool phase12_decode(const int16_t *samples,
                    int total_samples,
                    int sample_rate,
                    int max_sample,
                    phase12_result_t *result);

bool phase12_decode_with_codewords(const int16_t *samples,
                                   const uint8_t *codewords,
                                   int total_samples,
                                   int total_codewords,
                                   v91_law_t law,
                                   int sample_rate,
                                   int max_sample,
                                   phase12_result_t *result);

/*
 * Decode Phase 1 only (V.8 negotiation).
 */
bool phase12_decode_phase1(const int16_t *samples,
                           int total_samples,
                           int sample_rate,
                           int max_sample,
                           phase12_result_t *result);

bool phase12_decode_phase1_with_codewords(const int16_t *samples,
                                          const uint8_t *codewords,
                                          int total_samples,
                                          int total_codewords,
                                          v91_law_t law,
                                          int sample_rate,
                                          int max_sample,
                                          phase12_result_t *result);

const char *phase12_phase2_role_name(p12_phase2_role_t role);
const char *phase12_phase2_step_name(p12_phase2_step_t step);
const char *phase12_info0_kind_name(p12_info0_kind_t kind);
const char *phase12_info1_kind_name(p12_info1_kind_t kind);

/*
 * Decode Phase 2 only (V.34 probing / INFO exchange).
 * Optionally takes a Phase 1 result for context (may be NULL).
 */
bool phase12_decode_phase2(const int16_t *samples,
                           int total_samples,
                           int sample_rate,
                           int max_sample,
                           const phase12_result_t *phase1_ctx,
                           phase12_result_t *result);

/*
 * Merge a Phase 1/2 result into an existing call_log_t for HTML export.
 * Events are appended and can be sorted with call_log_sort() afterwards.
 */
void phase12_merge_to_call_log(const phase12_result_t *result,
                               call_log_t *log,
                               int sample_rate);

#endif /* PHASE12_DECODE_H */
