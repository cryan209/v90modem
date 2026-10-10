/*
 * line_monitor.h -- what is on the line now, for the DTE's diagnostic pages.
 *
 * Keeps the last second of received and transmitted audio (8 kHz linear, as
 * the engine sends and receives it -- for a SIP call the expanded G.711
 * codewords, i.e. the DS0 itself) and answers two questions from it on
 * demand: the total level each way, and the level in each 150 Hz band from
 * 150 to 3900 Hz.  The second is ATY11, after the USRobotics Courier's
 * frequency/level table; the Courier reported its V.34 line probe there, this
 * reports the actual signal, so it works for every modulation.  The rings are
 * cleared when a call starts and kept when it ends, so after NO CARRIER the
 * page shows the call's last second.
 *
 * Feeding is cheap (a copy into a ring under a leaf mutex) because it runs on
 * the media thread; the spectrum is computed only when asked for, on the AT
 * thread.  Nothing here calls out, so the lock nests under anything.
 */

#ifndef LINE_MONITOR_H
#define LINE_MONITOR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LM_RX 0
#define LM_TX 1

/* Band centres 150, 300, ... 3900 Hz; each band is centre +/- 75 Hz. */
#define LM_BAND_HZ    150
#define LM_BANDS      26

void lm_reset(void);
void lm_feed(int dir, const int16_t *amp, int len);
void lm_feed_g711(int dir, const uint8_t *codewords, int len, bool alaw);

/* Level in dBm0 over the last second (SpanDSP's scale: a full-scale sine is
   +3.14 dBm0).  Returns false if less than 0.1 s has been fed. */
bool lm_level_dbm0(int dir, float *dbm0);

/* Per-band levels, dBm0, LM_BANDS entries each (either pointer may be NULL).
   Welch-averaged 256-point Hann periodograms over the last second.  Returns
   the seconds of audio that went into it (0 if none). */
float lm_bands_dbm0(float *rx, float *tx);

/* The ATY11 page (CR LF line ends, no final OK). */
int lm_format_bands(char *out, size_t len);

/* Passive GUI taps; datapump bytes are eight consecutive LSB-first line bits
   (V.14 §6 / V.42 §7), not DTE characters or decoded LAPM frames. */
void lm_gui_enable(void);
void lm_wire_bit(int dir, int bit);
void lm_qam(float re, float im, bool decision);
void lm_eye(float in_re, float in_im, float eq_re, float eq_im, int phase);
void lm_event(const char *text);
int lm_gui_json(char *out, size_t size);

#endif
