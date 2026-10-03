/*
 * v92_mh_line.h — the audio side of V.92 modem-on-hold.
 *
 * 8.9.1: Tone RT is the modem's own retrain tone (Tone A 2400 Hz or Tone B
 * 1200 Hz, 8.2/V.90) and it detects the other one.  8.9.2: MH sequences use
 * the Phase 2 INFO modulation, 8.2.3.1/V.90 -- binary DPSK at 600 baud on
 * that same carrier, a 1 sent as a 180 degree reversal and a 0 as none
 * (the same mapping v34tx.c's get_info0_baud() uses).  An unmodulated
 * carrier is therefore a run of zeros, which is what makes the detectors
 * below work:
 *
 *   - Tone RT    : >= 24 consecutive zero bits on the peer carrier, with
 *                  that carrier dominating the band.  No MH stream has more
 *                  than 8 zeros in a row (fill ones bound it), so a running
 *                  MH sequence never reads as RT;
 *   - reversal   : a single one with >= 10 zeros either side (Cor.1 9.7.1.2
 *                  NOTE: MH must not be mistaken for a retrain reversal; 10
 *                  clears MH's 8 and keeps the confirmation to ~17 ms, which
 *                  matters because 11.2.1.1.3 answers it 40 ms later);
 *   - ANSam      : 2100 Hz dominating;
 *   - silence    : band power under about -45 dBm0.
 *
 * The receiver is non-coherent: the peer carrier mixed to baseband, a
 * one-symbol box integrator, the product with its value one symbol earlier,
 * and the sampling instant chosen as the bin of the symbol phase with the
 * largest mean |product|.  Linear 8 kHz in and out; the engine applies
 * G.711 itself.
 */
#ifndef V92_MH_LINE_H
#define V92_MH_LINE_H

#include <stdbool.h>
#include <stdint.h>

#include "v92_mh.h"

#define V92_MH_LINE_PHASE_BINS 20

typedef struct {
    /* role */
    double own_hz, peer_hz;
    double tx_amplitude;

    /* transmit */
    double tx_phase;                  /* carrier phase, cycles */
    double tx_symbol_pos;             /* 0..1 within the current symbol */
    double tx_sign, tx_prev_sign;
    double ansam_phase;
    int ansam_samples;
    int ansam_sign;
    v92_mh_tx_t last_tx;

    /* receive: demodulator */
    double rx_phase;
    double box_re[14], box_im[14];
    double sum_re, sum_im;
    int box_pos;
    double hist_re[32], hist_im[32];  /* integrator output, one-symbol delay line */
    int hist_pos;
    double sym_pos;                   /* symbol phase of this sample, 0..1 */
    double bin_energy[V92_MH_LINE_PHASE_BINS];
    int bin_best;
    int last_bin;
    unsigned bits;
    int zero_run;
    int ones_since_tone;
    bool rt;
    int pending_reversal;             /* zeros seen after a lone one */
    bool reversal;
    uint64_t rx_samples;              /* received samples so far */
    uint64_t rev_sample;              /* rx_samples when the last reversal's one was decided */

    /* transmit: the 11.2.1.1.3 reply to a retrain's first Tone A reversal */
    bool retrain_reply;
    int reply_tone;                   /* samples of Tone B still to send */
    int reply_reversed;               /* samples of reversed Tone B still to send */
    bool reply_done;

    /* receive: block detectors */
    int blk_n;
    double g_peer[3], g_ansam[3], energy;
    bool ansam, silence, peer_dominant;
} v92_mh_line_t;

/* `own_is_tone_a`: true if this modem's retrain tone is Tone A (8.9.1). */
void v92_mh_line_init(v92_mh_line_t *l, bool own_is_tone_a, double tx_dbm0);

/* Fill `out` from the controller's transmit state.  Returns false when the
 * controller is in V92_MH_TX_DATA (the caller produces data mode itself). */
bool v92_mh_line_tx(v92_mh_line_t *l, v92_mh_ctrl_t *c, int16_t *out, int n);

/* The controller decided the far end is retraining (a lone Tone A reversal,
 * V92_MH_ACT_RETRAIN with c->retrain_by_reversal).  Answer that reversal per
 * V.34 11.2.1.1.3 from this layer, on the carrier already on the line: Tone
 * B until 40 ms after the reversal (`since_reversal` samples have passed),
 * then 10 ms of it reversed, then silence and v92_mh_line_retrain_reply_done()
 * -- the moment to hand the transmitter to V.34 Phase 2.  While the reply runs
 * v92_mh_line_tx() ignores the controller. */
void v92_mh_line_retrain_reply(v92_mh_line_t *l, int since_reversal);
bool v92_mh_line_retrain_reply_done(const v92_mh_line_t *l);
/* As v92_mh_line_tx() during the reply, but stops at the sample where the
 * reversed tone ends and returns how many it wrote (n while still running):
 * the hand-over to V.34 belongs on that sample, because 11.2.1.1.4 times the
 * round trip from it. */
int v92_mh_line_retrain_reply_fill(v92_mh_line_t *l, int16_t *out, int n);

/* Demodulate `n` received samples: MH bits go to the controller, and it is
 * ticked once per 10 ms with the detectors' state. */
void v92_mh_line_rx(v92_mh_line_t *l, v92_mh_ctrl_t *c, const int16_t *in, int n);

#endif
