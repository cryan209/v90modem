/*
 * V.25 / V.32bis Annex A automode signal detection.  See v25_automode.h.
 */
#include "v25_automode.h"

#include <math.h>
#include <string.h>

#define V25AM_BLOCK     80          /* 10 ms at 8 kHz; 100 Hz bin spacing */

enum { BIN_600 = 0, BIN_1800, BIN_2100, BIN_3000, BIN_980, BIN_1180, BINS };
_Static_assert(BINS == V25AM_BINS, "v25_automode.h bin count");

/* 980 and 1180 Hz are V.21 channel 1, where V.8's CI and CM travel.  They are
   not on 100 Hz bins, which costs nothing: a Goertzel evaluates the spectrum
   at exactly its own frequency, and on-bin orthogonality only matters for the
   lines whose leakage the other tests must not see. */
static const float bin_hz[BINS] = {600.0f, 1800.0f, 2100.0f, 3000.0f, 980.0f, 1180.0f};
static float bin_coeff[BINS];

static void coeffs_init(void)
{
    if (bin_coeff[0] != 0.0f)
        return;
    for (int b = 0; b < BINS; b++)
        bin_coeff[b] = 2.0f*cosf(2.0f*(float) M_PI*bin_hz[b]/8000.0f);
}

/* Below about -45 dBm0 nothing is classified: RMS 50 in 16 bit linear. */
#define V25AM_FLOOR_MS  (50.0*50.0)
/* Fraction of the non-2100 Hz power a signal's own lines must hold. */
#define V25AM_LINE_FRAC 0.70
/* V.32bis 6.2's 64 symbol periods at 2400 baud is 26.7 ms. */
#define V25AM_AA_BLOCKS 3
/* AC needs no stated dwell; 50 ms keeps a phase-reversal transient or a
   burst of anything else from being taken for it. */
#define V25AM_AC_BLOCKS 5

void v25am_rx_init(v25_automode_rx_t *s, bool want_ans)
{
    coeffs_init();
    v25am_rx_release(s);
    memset(s, 0, sizeof(*s));
    s->watch_ans = want_ans;
    s->ans_tone = MODEM_CONNECT_TONES_NONE;
    s->ans_report_sample = -1;
    s->v21l_last_sample = -1;
    if (want_ans)
        s->ans_rx = modem_connect_tones_rx_init(NULL, MODEM_CONNECT_TONES_ANS_PR, NULL, NULL);
}

void v25am_rx_release(v25_automode_rx_t *s)
{
    if (s->ans_rx) {
        modem_connect_tones_rx_free(s->ans_rx);
        s->ans_rx = NULL;
    }
}

static void block_done(v25_automode_rx_t *s)
{
    double p[BINS];
    double ms = s->energy/V25AM_BLOCK;
    double rest;

    for (int b = 0; b < BINS; b++) {
        float c = bin_coeff[b];
        double mag2 = (double) s->s1[b]*s->s1[b] + (double) s->s2[b]*s->s2[b]
                    - (double) c*s->s1[b]*s->s2[b];

        /* A sine of amplitude A on an exact bin gives |X| = A*N/2, so
           2|X|^2/N^2 is its mean square, comparable with ms. */
        p[b] = 2.0*mag2/((double) V25AM_BLOCK*V25AM_BLOCK);
        s->s1[b] = s->s2[b] = 0.0f;
    }
    rest = ms - p[BIN_2100];
    if (rest < 0.0)
        rest = 0.0;

    if (rest > V25AM_FLOOR_MS && p[BIN_1800] >= V25AM_LINE_FRAC*rest) {
        if (++s->aa_run >= V25AM_AA_BLOCKS)
            s->aa = true;
    } else {
        s->aa_run = 0;
    }

    {
        double pair = p[BIN_600] + p[BIN_3000];
        double lo = (p[BIN_600] < p[BIN_3000]) ? p[BIN_600] : p[BIN_3000];

        if (rest > V25AM_FLOOR_MS && pair >= V25AM_LINE_FRAC*rest && lo >= 0.25*pair) {
            if (++s->ac_run >= V25AM_AC_BLOCKS)
                s->ac = true;
        } else {
            s->ac_run = 0;
        }
    }
    if (rest > V25AM_FLOOR_MS && p[BIN_980] + p[BIN_1180] >= 0.5*rest) {
        if (++s->v21l_run >= 3) {
            s->v21l_last_sample = s->samples;
            s->v21l_ever = true;
        }
    } else {
        s->v21l_run = 0;
    }
    s->energy = 0.0;
    s->n = 0;
}

void v25am_rx(v25_automode_rx_t *s, const int16_t amp[], int len)
{
    if (s->ans_rx) {
        int tone;

        modem_connect_tones_rx(s->ans_rx, amp, len);
        tone = modem_connect_tones_rx_get(s->ans_rx);
        if (tone == MODEM_CONNECT_TONES_ANSAM || tone == MODEM_CONNECT_TONES_ANSAM_PR) {
            s->ansam_seen = true;
            s->ans_tone = tone;
        } else if ((tone == MODEM_CONNECT_TONES_ANS || tone == MODEM_CONNECT_TONES_ANS_PR)
                   && s->ans_report_sample < 0) {
            s->ans_tone = tone;
            s->ans_report_sample = s->samples + len;
        }
    }
    for (int i = 0; i < len; i++) {
        float x = amp[i];

        s->samples++;

        for (int b = 0; b < BINS; b++) {
            float t = x + bin_coeff[b]*s->s1[b] - s->s2[b];

            s->s2[b] = s->s1[b];
            s->s1[b] = t;
        }
        s->energy += (double) x*x;
        if (++s->n >= V25AM_BLOCK)
            block_done(s);
    }
}

bool v25am_aa_detected(const v25_automode_rx_t *s)
{
    return s->aa;
}

int v25am_aa_ms(const v25_automode_rx_t *s)
{
    return s->aa_run*10;
}

bool v25am_v21_low_ever(const v25_automode_rx_t *s)
{
    return s->v21l_ever;
}

bool v25am_ac_detected(const v25_automode_rx_t *s)
{
    return s->ac;
}

bool v25am_v21_low_recent(const v25_automode_rx_t *s, int ms)
{
    return s->v21l_last_sample >= 0 && s->samples - s->v21l_last_sample <= (long) ms*8;
}

int v25am_plain_ans_ms(const v25_automode_rx_t *s)
{
    long since;
    int before;

    if (!s->watch_ans || s->ansam_seen || s->ans_report_sample < 0)
        return -1;
    since = (s->samples - s->ans_report_sample)/8;
    /* modem_connect_tones.c reports ANS after 550 ms unbroken and ANS_PR
       after its third 450 ms reversal cycle. */
    before = (s->ans_tone == MODEM_CONNECT_TONES_ANS_PR) ? 3*425 : 550;
    return (int) since + before;
}
