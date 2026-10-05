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
 *   USB1 V.22bis 6.3.1.2.1: the V.22bis answer modem's unscrambled binary
 *        1 at 1200 bit/s in the high channel.  Dibit 11 is a 270 degree
 *        step every 600 baud symbol (V.22 Table 2), i.e. a pure line at
 *        2400 - 150 = 2250 Hz.  A V.22bis answerer may add its 1800 Hz (or
 *        550 Hz) guard tone, which the test leaves out of the denominator.
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

#define V25AM_BINS 8    /* 600, 1800, 2100, 3000, 980, 1180, 2250, 550 Hz */

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
    int usb1_run;
    bool usb1;
    int v21l_run;
    bool v21l_ever;
    long v21l_last_sample;    /* last block V.21 channel 1 was present */

    /* V.25 answer tone classification. */
    bool watch_ans;
    modem_connect_tones_rx_state_t *ans_rx;
    int ans_tone;             /* last tone class SpanDSP reported */
    bool ansam_seen;          /* any modulated (V.8) answer tone ever seen */
    long ans_report_sample;   /* sample count when plain ANS was reported */
    long ans_line_last_sample; /* end of the last block 2100 Hz dominated */
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
/* V.22bis 6.3.1.1.1: USB1 from a V.22bis (or V.22) answer modem, present for
   at least 100 ms.  The call modem's own 155 ms detection then runs in the
   V.22bis datapump, so this only has to say which datapump to start. */
bool v25am_usb1_detected(const v25_automode_rx_t *s);
/* USB1 is on the line now (the last 100 ms), as opposed to ever. */
bool v25am_usb1_present(const v25_automode_rx_t *s);
/* Sample count at the end of the last 10 ms block in which a 2100 Hz answer
   tone held most of the power, or -1.  V.32bis A.2.1.3 times "the remaining
   answer tone" with it. */
long v25am_ans_last_sample(const v25_automode_rx_t *s);
/* Samples received since init. */
long v25am_samples(const v25_automode_rx_t *s);
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
