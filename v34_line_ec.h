/*
 * v34_line_ec.h - fixed line echo canceller for plain V.34, fitted once on
 * the window where this modem hears nothing but its own echo.
 *
 * V.34 is full duplex from Phase 4 on and needs its own echo removed
 * (V.34 clause 1: "echo cancellation techniques for channel separation").
 * See v34_line_ec.c for the measurement that motivates it and for why it is
 * a least-squares fit rather than an adaptive filter.
 */
#ifndef V34_LINE_EC_H
#define V34_LINE_EC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define V34_LEC_RING        32768   /* TX history, 4.1 s; power of two */
#define V34_LEC_TAPS        96
#define V34_LEC_TRAIN_MAX   8000    /* 1 s of echo-only receive */
#define V34_LEC_TRAIN_MIN   2000
#define V34_LEC_FIT_MIN     1000    /* samples left after the head guard */
#define V34_LEC_MAX_LAG     6000    /* 750 ms of bulk delay */
#define V34_LEC_SEARCH_LEN  1024    /* samples correlated in the lag search */
#define V34_LEC_TAIL_MAX    2400    /* most samples a window may discard at its end */

/* Bulk-delay tracking (v34_line_ec.c): how far either way a moved echo is
   looked for, the correlation sub-window and how many of them one decision
   spans, the order of the whitening predictor, and the predicted-echo ring. */
#define V34_LEC_TRACK_RANGE 1200    /* 150 ms */
#define V34_LEC_TRACK_SUB   2000    /* 250 ms */
#define V34_LEC_TRACK_NSUB  4       /* decide over the last second */
#define V34_LEC_TRACK_LPC   16
#define V34_LEC_TRACK_RING  4096    /* > 2*RANGE + SUB; power of two */

typedef struct
{
    int16_t tx[V34_LEC_RING];
    uint64_t tx_count;
    uint64_t rx_count;

    /* The most recent V34_LEC_TRAIN_MAX samples of the window, as a ring:
       the far end may still be finishing its previous signal at the start of
       the window, and is certainly silent at its end. */
    int16_t train_ring[V34_LEC_TRAIN_MAX + V34_LEC_TAIL_MAX];
    int16_t train_rx[V34_LEC_TRAIN_MAX];
    uint64_t train_start;           /* rx_count of train_rx[0] */
    uint64_t train_total;           /* samples seen in this window */
    int train_len;
    bool training;
    /* Samples dropped from the END of a window before fitting.  Zero for
       plain V.34, whose window closes on our own transmit stage before the
       far end can answer.  A V.90 digital modem's window closes when it has
       DETECTED the analogue modem's S, which is already on the line by then,
       so the tail carries far-end signal and has to go. */
    int tail_trim;

    bool fitted;
    int lag;                        /* rx[n] ~ sum h[k] tx[n - lag - k + TAPS/2] */
    float h[V34_LEC_TAPS];
    float erle_db;
    int fits;

    /* Bulk-delay tracking.  `track` is the caller's to set, and only while
       its transmission is aperiodic data (see track_rx()); the rest is the
       tracker's state, reset whenever a fit is put in force or tracking is
       re-armed. */
    bool track;
    bool trk_armed;                         /* tracking since the last reset */
    int shifts;
    uint64_t trk_next;                      /* next rx index to predict */
    float trk_p[V34_LEC_TRACK_RING];        /* predicted echo, by rx index */
    float trk_pw[V34_LEC_TRACK_RING];       /* the same, whitened */
    float trk_rx[V34_LEC_TRACK_LPC + 1];    /* recent raw rx, newest first */
    float trk_lpc[V34_LEC_TRACK_LPC + 1];   /* whitening filter, [0] = 1 */
    double trk_rac[V34_LEC_TRACK_LPC + 1];  /* rx autocorrelation, this sub-window */
    float trk_acc[V34_LEC_TRACK_NSUB][2*V34_LEC_TRACK_RANGE + 1];
    int trk_sub;                            /* sub-window being accumulated */
    int trk_pos;                            /* samples into it */
    int trk_filled;                         /* complete sub-windows held */
} v34_line_ec_t;

void v34_line_ec_reset(v34_line_ec_t *ec);

/* Our transmitted samples, in the order they went out. */
void v34_line_ec_tx(v34_line_ec_t *ec, const int16_t *amp, int len);

/* Received samples, cancelled in place once a fit is in force.  echo_only is
   the receiver's statement that the far end is silent and what it hears is
   our own transmission; the fit is taken when that window closes.  Returns
   true when a fit was attempted during this call, with a line in log. */
/* Discard an open training window without fitting: it ended in something
   other than the far end's expected answer (a retrain, say), so its tail
   may carry the far end and cannot be trusted.  Any fit already in force is
   kept. */
void v34_line_ec_abort_window(v34_line_ec_t *ec);

bool v34_line_ec_rx(v34_line_ec_t *ec, int16_t *amp, int len, bool echo_only,
                    char *log, size_t log_len);

#endif
