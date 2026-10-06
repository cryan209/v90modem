/*
 * v8bis_tones.h -- V.8bis tone signals: generator and detector.
 *
 * V.8bis (08/96) 7.1.  A signal is two segments: segment 1 is a 400 ms dual
 * tone (1375+2002 Hz from an initiating station, 1529+2225 Hz from a
 * responding one), segment 2 a 100 ms single tone naming the signal (Table 2).
 * MRd and CRd exist in both tone sets (an initiator in telephony mode sends the
 * initiating pair, the responder to MRe/CRe the responding pair); MRe, CRe and
 * ESi are initiating only, ESr responding only.
 *
 * ES signals are followed by a message whose 100 ms V.21 marking preamble IS
 * their segment 2 (7.2.4): 980 Hz is V.21(L) mark and 1650 Hz V.21(H) mark.
 * The generator emits segment 2 of ES as a plain tone, which is what a
 * detector sees; a caller that sends the message modulates the preamble with
 * V.21 instead and stops the generator after segment 1
 * (v8bis_tone_tx_seg1_only()).
 *
 * Levels: 7.1.4 puts MRe and CRe 12 to 15 dB below the nominal permitted
 * level (13.5 dB here).  Everything is linear 8 kHz, so it goes through the
 * same G.711 path V.8 uses; nothing here is codec-aware.
 */
#ifndef V8BIS_TONES_H
#define V8BIS_TONES_H

#include <stdbool.h>
#include <stdint.h>

#define V8BIS_RATE 8000
#define V8BIS_SEG1_SAMPLES 3200   /* 400 ms */
#define V8BIS_SEG2_SAMPLES 800    /* 100 ms */
#define V8BIS_SIGNAL_E_ATTEN_DB 13.5

typedef enum {
    V8BIS_SIG_MRE = 0,
    V8BIS_SIG_MRD,
    V8BIS_SIG_CRE,
    V8BIS_SIG_CRD,
    V8BIS_SIG_ESI,
    V8BIS_SIG_ESR,
    V8BIS_SIG_COUNT
} v8bis_signal_t;

const char *v8bis_signal_name(v8bis_signal_t sig);
/* Segment 2 frequency, Table 2. */
double v8bis_signal_seg2_hz(v8bis_signal_t sig);
/* Segment 1 pair for a tone set, Table 1. */
void v8bis_toneset_hz(bool responding_set, double *f1, double *f2);
/* Table 1: which signals exist in which tone set. */
bool v8bis_signal_in_toneset(v8bis_signal_t sig, bool responding_set);
/* 7.1.4: MRe and CRe are sent V8BIS_SIGNAL_E_ATTEN_DB below `nominal_dbm0`. */
double v8bis_signal_default_level_dbm0(v8bis_signal_t sig, double nominal_dbm0);

/* ---- generator ------------------------------------------------------ */

typedef struct {
    bool active;
    unsigned pos;
    unsigned total;           /* samples left to emit counted from start */
    unsigned seg1_len;
    double f[3];              /* seg1 a, seg1 b, seg2 */
    double phase[3];
    double amp1, amp2;
    double ppm;               /* frequency error applied to every tone */
} v8bis_tone_tx_t;

/* level_dbm0 is the level of the signal as a whole (each tone of the pair
 * carries half the power).  ppm scales every tone, for tolerance tests. */
void v8bis_tone_tx_start(v8bis_tone_tx_t *s, v8bis_signal_t sig, bool responding_set,
                         double level_dbm0, double ppm);
/* Stop after segment 1 (the caller sends the ES message preamble itself). */
void v8bis_tone_tx_seg1_only(v8bis_tone_tx_t *s);
/* Fill up to len samples; returns the number produced (0 once finished). */
int v8bis_tone_tx(v8bis_tone_tx_t *s, int16_t *amp, int len);
bool v8bis_tone_tx_done(const v8bis_tone_tx_t *s);

/* ---- detector -------------------------------------------------------- */

#define V8BIS_RX_BLOCK 160                /* 20 ms */
#define V8BIS_RX_QUEUE 4

typedef struct {
    v8bis_signal_t sig;
    bool responding_set;
    uint64_t seg1_start;                  /* estimated first sample of segment 1 */
    uint64_t detect_sample;               /* sample at which segment 2 was confirmed */
    unsigned seg1_blocks;                 /* 20 ms blocks of segment 1 seen */
    double seg1_quality;                  /* mean fraction of block energy in the pair */
} v8bis_tone_event_t;

typedef struct {
    int16_t buf[V8BIS_RX_BLOCK];
    unsigned fill;
    uint64_t sample;                      /* samples consumed (end of buf) */
    int run[2];                           /* seg1 run length per tone set, blocks */
    int miss[2];                          /* blocks since the pair last passed */
    double qsum[2];
    uint64_t run_start[2];
    int cand, cand_run;                   /* seg2 candidate (sig + 8*set) and its run */
    uint64_t cand_start;
    v8bis_tone_event_t q[V8BIS_RX_QUEUE];
    unsigned qn, qh;
} v8bis_tone_rx_t;

void v8bis_tone_rx_init(v8bis_tone_rx_t *s);
void v8bis_tone_rx(v8bis_tone_rx_t *s, const int16_t *amp, int len);
/* Pop the oldest detected signal; false when none is queued. */
bool v8bis_tone_rx_event(v8bis_tone_rx_t *s, v8bis_tone_event_t *ev);

#endif
