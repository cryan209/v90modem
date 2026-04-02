/*
 * v8bis_decode.h — V.8bis signal and message decoder for vpcm_decode
 *
 * Implements ITU-T V.8bis tone-based signal detection (MRe, CRe, ESi, MRd, CRd, ESr)
 * and HDLC-framed V.21 FSK message decoding (MS, CL, CLR, ACK, NAK).
 *
 * Reference: ITU-T Recommendation V.8 bis (08/96)
 */

#ifndef V8BIS_DECODE_H
#define V8BIS_DECODE_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/* ------------------------------------------------------------------ */
/* Forward declarations for types owned by vpcm_decode.c              */
/* ------------------------------------------------------------------ */

typedef struct {
    int sample_offset;
    int duration_samples;
    char protocol[24];
    char summary[160];
    char detail[1024];
} call_log_event_t;

typedef struct {
    call_log_event_t *events;
    size_t count;
    size_t cap;
} call_log_t;

/* ------------------------------------------------------------------ */
/* V.8bis signal definitions (ITU-T V.8bis §7.1, Tables 1 & 2)       */
/* ------------------------------------------------------------------ */

/*
 * Each V.8bis signal has two segments:
 *   Segment 1: dual-tone pair (400 ms nominal)
 *     - Initiating signals: 1375 + 2002 Hz
 *     - Responding signals: 1529 + 2225 Hz
 *   Segment 2: single identifying tone (100 ms nominal)
 *     - MRe: 650 Hz    MRd: 1150 Hz
 *     - CRe: 400 Hz    CRd: 1900 Hz
 *     - ESi: 980 Hz    ESr: 1650 Hz
 */

typedef struct {
    const char *name;
    const char *role;       /* "initiating" or "responding" */
    int seg1_a_hz;
    int seg1_b_hz;
    int seg2_hz;
} v8bis_signal_def_t;

typedef struct {
    bool seen;
    int sample_offset;
    int duration_samples;
    double score;
} v8bis_signal_hit_t;

#define V8BIS_NUM_SIGNALS 8

typedef struct {
    v8bis_signal_hit_t hits[V8BIS_NUM_SIGNALS];
} v8bis_scan_result_t;

typedef struct {
    bool seen;
    int signal_index;
    int sample_offset;
    int duration_samples;
    double score;
} v8bis_weak_candidate_t;

/* ------------------------------------------------------------------ */
/* V.8bis HDLC message types (ITU-T V.8bis §8.3.1, Table 3)          */
/* ------------------------------------------------------------------ */

#define V8BIS_MSG_MAX 12

typedef struct {
    uint8_t msg_type;       /* Lower 4 bits of I-field octet 1 */
    uint8_t revision;       /* Upper 4 bits of I-field octet 1 */
    uint8_t info[66];       /* Parameter/S/NS fields */
    int info_len;
    int sample_offset;
    int channel;            /* 0 = initiating/CH1, 1 = responding/CH2 */
} v8bis_decoded_msg_t;

/* ------------------------------------------------------------------ */
/* Timing constants (at 8000 Hz sample rate)                          */
/* ------------------------------------------------------------------ */

#define V8BIS_SEGMENT1_SAMPLES  ((8000 * 400) / 1000)   /* 3200 */
#define V8BIS_SEGMENT2_SAMPLES  ((8000 * 100) / 1000)   /* 800  */
#define V8BIS_SIGNAL_SAMPLES    (V8BIS_SEGMENT1_SAMPLES + V8BIS_SEGMENT2_SAMPLES)
#define V8BIS_SCAN_STEP_SAMPLES 160

/* ------------------------------------------------------------------ */
/* Global signal definition table                                     */
/* ------------------------------------------------------------------ */

extern const v8bis_signal_def_t g_v8bis_signal_defs[V8BIS_NUM_SIGNALS];

/* ------------------------------------------------------------------ */
/* Public API                                                         */
/* ------------------------------------------------------------------ */

/*
 * Scan for strong V.8bis tone signals in the sample buffer.
 * Populates out->hits[] for each signal type found.
 * Returns true if at least one signal was detected.
 */
bool v8bis_scan_signals(const int16_t *samples,
                        int total_samples,
                        int max_sample,
                        v8bis_scan_result_t *out);

/*
 * Scan for weak V.8bis tone signals (lower thresholds).
 * Returns the single best weak candidate, if any.
 */
bool v8bis_scan_weak_candidate(const int16_t *samples,
                               int total_samples,
                               int max_sample,
                               v8bis_weak_candidate_t *out);

/*
 * Decode V.8bis HDLC messages from V.21 FSK in the sample buffer.
 * Appends decoded message events to the call log.
 */
void v8bis_collect_msg_events(call_log_t *log,
                              const int16_t *samples,
                              int total_samples,
                              int max_sample);

/*
 * Collect V.8bis tone signal events (strong + weak) into the call log.
 */
void v8bis_collect_signal_events(call_log_t *log,
                                 const int16_t *samples,
                                 int total_samples,
                                 int max_sample);

/*
 * Remove near-duplicate V.8bis FSK messages from a combined stereo log.
 * Both channels independently decode the same FSK signal; keep only the
 * first occurrence of each message type within a ~1s (8000-sample) window.
 */
void v8bis_dedup_msgs(call_log_t *log);

/*
 * Convert V.8bis message type code to string (per Table 3).
 */
const char *v8bis_msg_type_str(int type);

/*
 * Convert V.8bis SPar(1) capability bits to a mode description string.
 */
const char *v8bis_spar1_mode_str(int spar1_bits);

/* ------------------------------------------------------------------ */
/* DSP helpers (defined in vpcm_decode.c, shared with this module)    */
/* ------------------------------------------------------------------ */

double window_energy(const int16_t *samples, int len);
double tone_energy_ratio(const int16_t *samples, int len,
                         int sample_rate, double freq_hz,
                         double total_energy);

/* call_log_append is defined in vpcm_decode.c */
bool call_log_append(call_log_t *log,
                     int sample_offset,
                     int duration_samples,
                     const char *protocol,
                     const char *summary,
                     const char *detail);

#endif /* V8BIS_DECODE_H */
