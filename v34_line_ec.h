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

typedef struct
{
    int16_t tx[V34_LEC_RING];
    uint64_t tx_count;
    uint64_t rx_count;

    /* The most recent V34_LEC_TRAIN_MAX samples of the window, as a ring:
       the far end may still be finishing its previous signal at the start of
       the window, and is certainly silent at its end. */
    int16_t train_ring[V34_LEC_TRAIN_MAX];
    int16_t train_rx[V34_LEC_TRAIN_MAX];
    uint64_t train_start;           /* rx_count of train_rx[0] */
    uint64_t train_total;           /* samples seen in this window */
    int train_len;
    bool training;

    bool fitted;
    int lag;                        /* rx[n] ~ sum h[k] tx[n - lag - k + TAPS/2] */
    float h[V34_LEC_TAPS];
    float erle_db;
    int fits;
} v34_line_ec_t;

void v34_line_ec_reset(v34_line_ec_t *ec);

/* Our transmitted samples, in the order they went out. */
void v34_line_ec_tx(v34_line_ec_t *ec, const int16_t *amp, int len);

/* Received samples, cancelled in place once a fit is in force.  echo_only is
   the receiver's statement that the far end is silent and what it hears is
   our own transmission; the fit is taken when that window closes.  Returns
   true when a fit was attempted during this call, with a line in log. */
bool v34_line_ec_rx(v34_line_ec_t *ec, int16_t *amp, int len, bool echo_only,
                    char *log, size_t log_len);

#endif
