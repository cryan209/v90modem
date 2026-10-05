/*
 * V.25 / V.32bis Annex A automode signal detection.
 *
 * The signals an automode modem has to tell apart before any datapump runs:
 *
 *   AA   V.32bis 6.1 / Figure 3: the call modem repeating carrier state A,
 *        i.e. a pure 1800 Hz line (V.8 8.2.2 calls it the call modem's sigC).
 *   AC   V.32bis 6.2: the answer modem alternating states A and C, i.e. a
 *        suppressed-carrier pair at 1800 -/+ 1200 Hz = 600 and 3000 Hz
 *        (V.8 8.1.1's sigA for V.32/V.32bis).
 *   ANS  V.25's 2100 Hz answer tone, classified as plain ANS or ANSam by
 *        SpanDSP's own connect-tone detector, because V.8 8.1.1 sends a call
 *        modem that hears ANS rather than ANSam to Annex A/V.32bis.
 *
 * The tone detectors are 10 ms Goertzel blocks at 8 kHz.  80 samples puts
 * 600, 1800, 2100 and 3000 Hz on exact bins (100 Hz spacing), so a tone on
 * one bin is orthogonal to the others and our own ANSam -- which is the
 * strongest thing in the receive path while V.8 runs -- cannot leak into the
 * AA bin.  2100 Hz is measured so it can be excluded from the denominator.
 */
#ifndef V25_AUTOMODE_H
#define V25_AUTOMODE_H

#include <stdbool.h>
#include <stdint.h>

#include <spandsp.h>

#define V25AM_BINS 6    /* 600, 1800, 2100, 3000, 980, 1180 Hz */

typedef struct {
    /* Goertzel block state. */
    int n;
    float s1[V25AM_BINS];
    float s2[V25AM_BINS];
    double energy;

    /* Consecutive 10 ms blocks satisfying each signal's test. */
    int aa_run;
    int ac_run;
    bool aa;
    bool ac;
    int v21l_run;
    bool v21l_ever;
    long v21l_last_sample;    /* last block V.21 channel 1 was present */

    /* V.25 answer tone classification. */
    bool watch_ans;
    modem_connect_tones_rx_state_t *ans_rx;
    int ans_tone;             /* last tone class SpanDSP reported */
    bool ansam_seen;          /* any modulated (V.8) answer tone ever seen */
    long ans_report_sample;   /* sample count when plain ANS was reported */
    long samples;
} v25_automode_rx_t;

/* want_ans: also classify V.25 ANS against ANSam (the call modem side). */
void v25am_rx_init(v25_automode_rx_t *s, bool want_ans);
void v25am_rx(v25_automode_rx_t *s, const int16_t amp[], int len);
void v25am_rx_release(v25_automode_rx_t *s);

/* V.32bis 6.2: "an incoming tone has been detected at 1800 +/- 7 Hz for 64
   symbol periods" -- 64T at 2400 baud is 26.7 ms; three 10 ms blocks. */
bool v25am_aa_detected(const v25_automode_rx_t *s);
/* How long AA has been continuously present, in ms (0 if it is not). */
int v25am_aa_ms(const v25_automode_rx_t *s);
/* Has V.21 channel 1 been seen at all since init? */
bool v25am_v21_low_ever(const v25_automode_rx_t *s);
/* V.32bis A.2.1.1: AC from the answering modem. */
bool v25am_ac_detected(const v25_automode_rx_t *s);
/* V.21 channel 1 (980/1180 Hz, where V.8 sends CI and CM) has dominated the
   band for at least 30 ms, ending within the last `ms` milliseconds. */
bool v25am_v21_low_recent(const v25_automode_rx_t *s, int ms);
/* Milliseconds of plain (unmodulated) V.25 ANS heard so far, or -1 if none or
   if an ANSam has been seen on this call.  The detector reports a plain tone
   only after 550 ms unbroken or three 450 ms phase-reversal cycles, and the
   time before its report is added back, so this is a lower bound on the
   tone's duration. */
int v25am_plain_ans_ms(const v25_automode_rx_t *s);

#endif
