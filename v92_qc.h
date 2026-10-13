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
    V92_QC_V8           /* done: no TONEq, send ANSam and run V.8 */
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

/* Fill up to n codewords of the transmit stream.  Returns how many were
 * produced: n, unless the sequence reached PHASE2 or V8 part-way, in which
 * case it is the count up to that instant and the caller supplies the rest
 * (that is what makes the 75 ms of silence exact rather than 75 ms plus
 * most of an RTP frame). */
int v92_qc_tx(v92_qc_t *q, uint8_t *codewords, int n);

/* Feed received codewords to the TONEq detector. */
void v92_qc_rx(v92_qc_t *q, const uint8_t *codewords, int n);

static inline v92_qc_state_t v92_qc_state(const v92_qc_t *q) { return q->state; }
static inline bool v92_qc_running(const v92_qc_t *q)
{
    return q->state != V92_QC_IDLE && q->state != V92_QC_PHASE2
        && q->state != V92_QC_V8;
}
const char *v92_qc_state_name(v92_qc_state_t s);

#endif
