/*
 * v92_qc.h -- V.92 Quick Connect, short Phase 1 for the DIGITAL modem
 *             (V.92 9.2.4; signals 8.2.5 TONEq, 8.3.1 ANSpcm, 8.3.6 QTS,
 *             Tables 4/5 QCA1d)
 *
 * What the answering digital modem does once it has detected QC1a (9.2.4.1):
 *
 *   QCA1d   70 bits at 300 bit/s on V.21(H), Table 5
 *   silence 75 +/- 5 ms
 *   QTS     128 x {+V +0 +V -V -0 -V}, V = the codeword whose Ucode is the
 *           caller's UQTS (the WXYZ it sent in QC1a, Table 2/3)
 *   QTS\    8 x {-V -0 -V +V +0 +V}
 *   ANSpcm  the 301-symbol tone, at the level QCA1d's LM announced, until
 *           TONEq arrives (9.2.4.3)
 *   silence 75 +/- 5 ms, after which Phase 2 starts
 *
 * and, if no TONEq arrives in the 2 s after QCA1d, ANSam and ordinary V.8
 * (9.2.4.3).  This module is only that sequence: it owns the transmit
 * stream from the first QCA1d bit to the last silent codeword, listens for
 * TONEq, and says which way the call goes.  It does not decide whether to
 * start (the engine does, on the strength of a received QC1a) and it does
 * not start Phase 2 (the engine does, when told PHASE2).
 *
 * QTS and ANSpcm are G.711 codeword sequences, so everything is produced and
 * consumed as codewords: nothing here may be passed through a linear
 * transcode.  The data frame alignment of 8.3.6 starts at the first QTS
 * codeword; qts_start_octet() reports when that was.
 */
#ifndef V92_QC_H
#define V92_QC_H

#include <stdbool.h>
#include <stdint.h>

#include "v91.h"

typedef enum {
    V92_QC_IDLE = 0,
    V92_QC_QCA1D,       /* the 70-bit V.21(H) sequence */
    V92_QC_GAP,         /* 75 ms of silence before QTS */
    V92_QC_QTS,
    V92_QC_QTS_BAR,
    V92_QC_ANSPCM,      /* until TONEq or the 2 s limit */
    V92_QC_SILENCE,     /* 75 ms after TONEq */
    V92_QC_PHASE2,      /* done: proceed to Phase 2 */
    V92_QC_V8,          /* done: no TONEq, send ANSam and run V.8 */
    V92_QC_ANSAM,       /* done (analogue answerer): QCA1a sent, ANSam next */
    V92_QC_WATCH        /* analogue answerer: V.8 sends ANSam, we listen for TONEq */
} v92_qc_state_t;

#define V92_QC_SILENCE_SYMBOLS   600     /* 75 ms */
#define V92_QC_TONEQ_TIMEOUT     16000   /* 2 s after QCA1d, 9.2.4.3 */
#define V92_QC_QTS_SYMBOLS       768      /* 128 x 6 */
#define V92_QC_QTS_BAR_SYMBOLS   48       /* 8 x 6 */
#define V92_QC_TONEQ_BLOCK       80       /* 10 ms detector blocks */
#define V92_QC_TONEQ_BLOCKS      5        /* consecutive blocks: 50 ms */

typedef struct {
    v91_law_t law;
    int uqts_ucode;
    int anspcm_level;                 /* 0..3, Table 6 */
    bool lapm;                        /* the P bit of QCA1d */

    v92_qc_state_t state;
    long pos;                         /* symbols into the current state */
    long since_qca;                   /* symbols since the end of QCA1d */
    long total;                       /* symbols produced since start */
    long qts_start;                   /* symbol index of the first QTS codeword, -1 before */
    long anspcm_pos;                  /* ANSpcm symbols sent */
    bool toneq_seen;

    bool ana_answer;                  /* the sequence is the analogue answerer's QCA1a (9.2.3.1) */
    void *fsk;                        /* spandsp fsk_tx_state_t, QCA1d only */
    uint8_t qca_bits[70];
    int qca_bit;
    int qca_len;

    /* TONEq detector (980 Hz), 80-sample blocks */
    int blk_n;
    double blk_e;                     /* sum of x^2 */
    double blk_s1, blk_s2;            /* Goertzel */
    int toneq_run;
    long toneq_first;                 /* rx symbol index where the run began */
    long rx_total;
} v92_qc_t;

void v92_qc_init(v92_qc_t *q);
void v92_qc_free(v92_qc_t *q);

/* Begin the 9.2.4.1 response.  uqts_ucode is the Ucode UQTS the caller's
 * QC1a asked for (v92_short_phase1_uqts_from_wxyz), anspcm_level 0..3 the
 * level QCA1d announces, lapm its P bit.  False if the arguments are not
 * usable (the caller then stays on ordinary V.8). */
bool v92_qc_start(v92_qc_t *q, v91_law_t law, int uqts_ucode,
                  int anspcm_level, bool lapm);

/* 9.2.3.1: this is an analogue answering modem and the caller's QC1a came from an
 * analogue modem too (Figure 7).  Sends QCA1a, then ends in V92_QC_ANSAM: the
 * caller (the engine) starts V.8's ANSam, which brings its own 75 ms of silence. */
bool v92_qc_start_analogue_answer(v92_qc_t *q, v91_law_t law, bool lapm);
/* 9.2.3.4: ANSam is being sent by V.8; the detector runs and TONEq moves the state
 * to V92_QC_SILENCE (75 ms, then PHASE2) -- from then on tx is ours. */
bool v92_qc_watch_start(v92_qc_t *q, v91_law_t law);

/* As v92_qc_start(), for the V.8bis route (9.2.4.2): the caller's QC2a has been
 * answered by QCA2d, which the V.8bis modem sent, so the sequence resumes at the
 * 75 ms of silence after it.  `silent` of those codewords have already gone out
 * with the end of the last block. */
bool v92_qc_start_after_qca(v92_qc_t *q, v91_law_t law, int uqts_ucode,
                            int anspcm_level, bool lapm, unsigned silent);

/* Fill up to n codewords of the transmit stream.  Returns how many were
 * produced: n, unless the sequence reached PHASE2 or V8 part-way, in which
 * case it is the count up to that instant and the caller supplies the rest
 * (that is what makes the 75 ms of silence exact rather than 75 ms plus
 * most of an RTP frame). */
int v92_qc_tx(v92_qc_t *q, uint8_t *codewords, int n);

/* Feed received codewords to the TONEq detector. */
void v92_qc_rx(v92_qc_t *q, const uint8_t *codewords, int n);

/* ------------------------------------------------------------------ */
/* The analogue caller's side (9.2.1.1 after QCA1d, 9.2.1.3)           */
/* ------------------------------------------------------------------ */

typedef enum {
    V92_QCA_IDLE = 0,
    V92_QCA_WAIT_PCM,   /* silence; waiting for ANSpcm to have been heard for 1 s */
    V92_QCA_TONEQ,      /* sending TONEq until ANSpcm stops */
    V92_QCA_SILENCE,    /* 75 ms */
    V92_QCA_PHASE2,     /* done: proceed to Phase 2 */
    V92_QCA_V8          /* done: no ANSpcm, back to V.8 */
} v92_qca_state_t;

#define V92_QCA_PCM_BLOCKS     100     /* 1 s of ANSpcm */
#define V92_QCA_PCM_MISS       3       /* blocks without it that mean it has stopped */
#define V92_QCA_WAIT_LIMIT     48000   /* 6 s from QCA1d: the digital modem has given up */

typedef struct {
    v91_law_t law;
    v92_qca_state_t state;
    long pos;
    long waited;                      /* symbols since the start */
    double tone_phase;
    int blk_n;
    double blk_e, blk_s1, blk_s2;
    int pcm_run, pcm_miss;
    long tonq_symbols;
    int pcm_blocks_needed;
    long wait_limit;                  /* symbols before giving up on the tone */
} v92_qca_t;

void v92_qca_init(v92_qca_t *q);
/* Begin once QCA1d has been detected: stop CM, go silent, listen for ANSpcm. */
bool v92_qca_start(v92_qca_t *q, v91_law_t law);
/* The answerer is an analogue modem (QCA1a/QCA2a): it is ANSam that is waited for
 * and then answered with TONEq, for `blocks` 10 ms blocks (9.2.1.1: until
 * detected; 9.2.1.2: for 1 s). */
bool v92_qca_start_ansam(v92_qca_t *q, v91_law_t law, int blocks);
/* The same, with the limit (symbols) the caller's clause gives: 9.2.3.3 allows the
 * analogue answerer 2 s for ANSpcm after QCA1a/QCA2a. */
bool v92_qca_start_limit(v92_qca_t *q, v91_law_t law, int blocks, long limit);
/* As v92_qc_tx(): returns how many codewords it supplied, short at the end. */
int v92_qca_tx(v92_qca_t *q, uint8_t *codewords, int n);
void v92_qca_rx(v92_qca_t *q, const uint8_t *codewords, int n);
static inline bool v92_qca_running(const v92_qca_t *q)
{
    return q->state != V92_QCA_IDLE && q->state != V92_QCA_PHASE2
        && q->state != V92_QCA_V8;
}
const char *v92_qca_state_name(v92_qca_state_t s);

static inline v92_qc_state_t v92_qc_state(const v92_qc_t *q) { return q->state; }
static inline bool v92_qc_running(const v92_qc_t *q)
{
    return q->state != V92_QC_IDLE && q->state != V92_QC_PHASE2
        && q->state != V92_QC_V8 && q->state != V92_QC_ANSAM
        && q->state != V92_QC_WATCH;
}
const char *v92_qc_state_name(v92_qc_state_t s);

#endif
