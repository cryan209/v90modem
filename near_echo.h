/*
 * near_echo.h — a near-end echo canceller for a codec-attached analogue modem,
 * fitted by least squares on correlations accumulated under double talk.
 *
 * Why it exists: on the Apple USB modem the analogue V.90 role transmits for
 * the whole of Phase 4 (§9.4.2: CPt, then CP) while it receives the digital
 * modem's Ri/TRN2d/MP, and its own transmit comes back through the hybrid
 * 118 ms later (the USB round trip) at -16.7 dB against the downstream.
 * Nothing cancelled it.  Measured on artifacts/apple-pve-20261015/v90-lt6, the
 * best equaliser the TRN2d era supports (128 T/2 taps plus 24 of ideal
 * decision feedback, fitted against the digital side's own tap) reaches
 * 20.0 dB with the echo in and 25.1 dB with a 64-tap fit of it subtracted --
 * the latter is the line's own quiet floor.
 *
 * The echo is linear but LONG: on the echo-only window of v90-lt6 (V.34
 * Phase 2 probing, 38.65 s) a 64-tap fit removes 14 dB, 384 taps 31 dB --
 * down to the line's own floor (rms 4) -- and the fit still holds 30 dB on
 * the next attempt's probing 20 s later, so the path is stable within a call.
 * The bulk delay is not stable between calls (1133, 1142, 1165 samples at
 * 9600 Hz on three calls), so the window is made long enough to hold the
 * response at any of them rather than searched for.
 *
 * Least squares, exact (covariance method), and only over ECHO-ONLY blocks
 * (near_echo.c says how they are recognised).  An LS fit's excess error is
 * taps/samples times the interference power, so a fit made under Phase 4's
 * double talk would need hours: measured, it removed 7 dB.  The echo-only
 * windows are V.34 Phase 2's L1/L2 (§10.2) and anything else where the far end
 * is silent while we transmit; a call without one is simply never cancelled.
 *
 * Both streams are device samples on one clock -- what was written to the
 * codec and what was read from it, in order -- which is what makes the path
 * linear and time-invariant.  Push transmit samples in output order from any
 * thread; process receive samples in input order from one other thread.
 */
#ifndef NEAR_ECHO_H
#define NEAR_ECHO_H

#include <stdint.h>

typedef struct near_echo_s near_echo_t;

/* Taps cover delays [lag, lag+taps) samples.  solve_every is how often
 * (receive samples) a fit is refreshed when new echo-only blocks exist. */
near_echo_t *near_echo_init(int lag, int taps, int solve_every);
/* Solve on a worker thread instead of inline.  Call it for a real-time
 * receive path (the solve is ~taps^3/6 operations); leave it off for a replay,
 * which must be deterministic. */
int near_echo_set_threaded(near_echo_t *s, int on);
void near_echo_free(near_echo_t *s);

/* Transmit samples exactly as they went to the device, in order. */
void near_echo_tx(near_echo_t *s, const int16_t *tx, int n);
/* Receive samples (DC already removed), in order: accumulates, then subtracts
 * the current estimate in place. */
void near_echo_rx(near_echo_t *s, int16_t *rx, int n);

/* Diagnostics. */
int    near_echo_solved(const near_echo_t *s);     /* fits computed so far */
int    near_echo_active(const near_echo_t *s);     /* a stable fit is being subtracted */
int    near_echo_lag(const near_echo_t *s);        /* first tap's delay */
long   near_echo_blocks_used(const near_echo_t *s); /* echo-only 50 ms blocks fitted */
double near_echo_rms_in(const near_echo_t *s);     /* receive rms, last ~1 s */
double near_echo_rms_echo(const near_echo_t *s);   /* subtracted rms, last ~1 s */

#endif
