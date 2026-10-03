/*
 * v92_mh_line.c — Tone RT, MH DPSK, ANSam and the receive detectors for
 * V.92 modem-on-hold.  See v92_mh_line.h.
 */
#include "v92_mh_line.h"

#include <math.h>
#include <string.h>

#define FS 8000.0
#define BAUD 600.0
#define BLOCK 80                      /* 10 ms */
#define TONE_A_HZ 2400.0
#define TONE_B_HZ 1200.0
#define ANSAM_HZ 2100.0
#define DBM0_RMS 16017.0              /* G.711 0 dBm0 sine (mu-law) */

#define RT_ZEROS 24                   /* 40 ms of unmodulated carrier */
#define REV_GUARD 10
#define PEER_TONE_FRAC 0.55
#define ANSAM_FRAC 0.5
#define SILENCE_RMS 90.0              /* about -45 dBm0 */

void v92_mh_line_init(v92_mh_line_t *l, bool own_is_tone_a, double tx_dbm0)
{
    memset(l, 0, sizeof(*l));
    l->own_hz = own_is_tone_a ? TONE_A_HZ : TONE_B_HZ;
    l->peer_hz = own_is_tone_a ? TONE_B_HZ : TONE_A_HZ;
    l->tx_amplitude = DBM0_RMS * sqrt(2.0) * pow(10.0, tx_dbm0 / 20.0);
    l->tx_sign = l->tx_prev_sign = 1.0;
    l->ansam_sign = 1;
    l->last_tx = V92_MH_TX_DATA;
}

static int16_t clip16(double v)
{
    if (v > 32767.0) return 32767;
    if (v < -32768.0) return -32768;
    return (int16_t)lrint(v);
}

void v92_mh_line_retrain_reply(v92_mh_line_t *l, int since_reversal)
{
    l->retrain_reply = true;
    l->reply_tone = 320 - since_reversal;        /* 40 ms after the reversal */
    if (l->reply_tone < 0)
        l->reply_tone = 0;
    l->reply_reversed = 80;                      /* 10 ms */
    l->reply_done = false;
}

bool v92_mh_line_retrain_reply_done(const v92_mh_line_t *l)
{
    return l->retrain_reply && l->reply_done;
}

int v92_mh_line_retrain_reply_fill(v92_mh_line_t *l, int16_t *out, int n)
{
    int i;

    for (i = 0; i < n; i++) {
        if (l->reply_tone > 0) {
            l->reply_tone--;
        } else if (l->reply_reversed > 0) {
            if (l->reply_reversed == 80)
                l->tx_sign = -l->tx_sign;       /* 11.2.1.1.3's reversal */
            l->reply_reversed--;
        } else {
            l->reply_done = true;
            break;
        }
        out[i] = clip16(l->tx_amplitude * l->tx_sign * cos(2.0 * M_PI * l->tx_phase));
        l->tx_phase += l->own_hz / FS;
        if (l->tx_phase >= 1.0) l->tx_phase -= 1.0;
    }
    return i;
}

bool v92_mh_line_tx(v92_mh_line_t *l, v92_mh_ctrl_t *c, int16_t *out, int n)
{
    if (l->retrain_reply) {
        int k = v92_mh_line_retrain_reply_fill(l, out, n);

        memset(out + k, 0, (size_t)(n - k) * sizeof(out[0]));
        return true;
    }
    if (c->tx == V92_MH_TX_DATA) {
        l->last_tx = V92_MH_TX_DATA;
        return false;
    }
    for (int i = 0; i < n; i++) {
        v92_mh_tx_t tx = c->tx;
        double v = 0.0;

        if (tx != l->last_tx) {
            if (tx == V92_MH_TX_ANSAM) {
                l->ansam_samples = 0;
                l->ansam_sign = 1;
            }
            if (tx == V92_MH_TX_MH && l->last_tx != V92_MH_TX_RT)
                l->tx_symbol_pos = 1.0;          /* first bit at once */
            l->last_tx = tx;
        }
        switch (tx) {
        case V92_MH_TX_RT:
            l->tx_prev_sign = l->tx_sign;
            v = l->tx_amplitude * l->tx_sign * cos(2.0 * M_PI * l->tx_phase);
            break;
        case V92_MH_TX_MH: {
            double m;

            l->tx_symbol_pos += BAUD / FS;
            if (l->tx_symbol_pos >= 1.0) {
                l->tx_symbol_pos -= 1.0;
                l->tx_prev_sign = l->tx_sign;
                if (v92_mh_ctrl_tx_bit(c))
                    l->tx_sign = -l->tx_sign;    /* 1 = 180 degree reversal */
            }
            /* Raised-cosine crossfade across the symbol after a reversal,
             * so the envelope goes through zero smoothly. */
            m = l->tx_prev_sign + (l->tx_sign - l->tx_prev_sign)
                * 0.5 * (1.0 - cos(M_PI * l->tx_symbol_pos));
            v = l->tx_amplitude * m * cos(2.0 * M_PI * l->tx_phase);
            break;
        }
        case V92_MH_TX_ANSAM: {
            /* V.8 ANSam: 2100 Hz, 15 Hz AM at 20 %, reversal every 450 ms. */
            double t = l->ansam_samples / FS;

            if (l->ansam_samples > 0 && l->ansam_samples % 3600 == 0)
                l->ansam_sign = -l->ansam_sign;
            v = l->tx_amplitude * (1.0 + 0.2 * sin(2.0 * M_PI * 15.0 * t))
                * l->ansam_sign * sin(2.0 * M_PI * l->ansam_phase);
            l->ansam_phase += ANSAM_HZ / FS;
            if (l->ansam_phase >= 1.0) l->ansam_phase -= 1.0;
            l->ansam_samples++;
            break;
        }
        default:
            break;                               /* silence */
        }
        l->tx_phase += l->own_hz / FS;
        if (l->tx_phase >= 1.0) l->tx_phase -= 1.0;
        out[i] = clip16(v);
    }
    return true;
}

static void goertzel_step(double g[3], double hz, double x)
{
    double coeff = 2.0 * cos(2.0 * M_PI * hz / FS);
    double s = x + coeff * g[0] - g[1];

    g[1] = g[0];
    g[0] = s;
    g[2] = coeff;
}

/* Fraction of the block's energy in the Goertzel bin: 1.0 for a pure tone. */
static double goertzel_frac(const double g[3], double energy)
{
    double p = g[0] * g[0] + g[1] * g[1] - g[2] * g[0] * g[1];

    return energy > 0.0 ? 2.0 * p / ((double)BLOCK * energy) : 0.0;
}

static void on_bit(v92_mh_line_t *l, v92_mh_ctrl_t *c, int bit)
{
    l->bits++;
    v92_mh_ctrl_rx_bit(c, bit);
    if (bit == 0) {
        l->zero_run++;
        if (l->pending_reversal > 0 && ++l->pending_reversal > REV_GUARD) {
            l->reversal = true;
            l->pending_reversal = 0;
        }
    } else {
        /* A lone one after a long run of zeros is a candidate reversal; a
         * second one before the guard has passed makes it MH, not a
         * reversal. */
        l->pending_reversal = (l->zero_run >= REV_GUARD && l->pending_reversal == 0) ? 1 : 0;
        if (l->pending_reversal)
            l->rev_sample = l->rx_samples;
        l->zero_run = 0;
    }
    l->rt = l->peer_dominant && l->zero_run >= RT_ZEROS;
}

/* One 10 ms block: settle the detectors and tick the controller. */
static void end_of_block(v92_mh_line_t *l, v92_mh_ctrl_t *c)
{
    double rms = sqrt(l->energy / BLOCK);
    v92_mh_detect_t d;

    l->silence = rms <= SILENCE_RMS;
    l->peer_dominant = !l->silence
                       && goertzel_frac(l->g_peer, l->energy) > PEER_TONE_FRAC;
    l->ansam = !l->silence
               && goertzel_frac(l->g_ansam, l->energy) > ANSAM_FRAC;
    if (!l->peer_dominant) {
        l->rt = false;
        if (l->silence)
            l->zero_run = 0;          /* digital silence demodulates as zeros */
    }

    memset(&d, 0, sizeof(d));
    d.rt = l->rt;
    d.silence = l->silence;
    d.ansam = l->ansam;
    d.reversal = l->reversal;
    l->reversal = false;
    v92_mh_ctrl_tick(c, 10, &d);

    memset(l->g_peer, 0, sizeof(l->g_peer));
    memset(l->g_ansam, 0, sizeof(l->g_ansam));
    l->energy = 0.0;
    l->blk_n = 0;
}

void v92_mh_line_rx(v92_mh_line_t *l, v92_mh_ctrl_t *c, const int16_t *in, int n)
{
    const double sym = FS / BAUD;     /* 13.333 samples */
    const int whole = (int)floor(sym);
    const double frac = sym - whole;

    for (int i = 0; i < n; i++) {
        double x = in[i];
        double re;
        double im;
        double dre, dim, d;
        int k0, k1, bin;

        l->rx_samples++;
        re = x * cos(2.0 * M_PI * l->rx_phase);
        im = -x * sin(2.0 * M_PI * l->rx_phase);

        l->rx_phase += l->peer_hz / FS;
        if (l->rx_phase >= 1.0) l->rx_phase -= 1.0;

        /* One-symbol box integrator, 13 taps. */
        l->sum_re += re - l->box_re[l->box_pos];
        l->sum_im += im - l->box_im[l->box_pos];
        l->box_re[l->box_pos] = re;
        l->box_im[l->box_pos] = im;
        l->box_pos = (l->box_pos + 1) % 13;

        /* Integrator output one symbol (13.33 samples) ago, interpolated. */
        l->hist_re[l->hist_pos] = l->sum_re;
        l->hist_im[l->hist_pos] = l->sum_im;
        k0 = (l->hist_pos - whole + 32) % 32;
        k1 = (k0 - 1 + 32) % 32;
        dre = (1.0 - frac) * l->hist_re[k0] + frac * l->hist_re[k1];
        dim = (1.0 - frac) * l->hist_im[k0] + frac * l->hist_im[k1];
        l->hist_pos = (l->hist_pos + 1) % 32;

        /* Re{y * conj(y one symbol earlier)}: negative across a reversal. */
        d = l->sum_re * dre + l->sum_im * dim;

        bin = (int)(l->sym_pos * V92_MH_LINE_PHASE_BINS);
        if (bin >= V92_MH_LINE_PHASE_BINS)
            bin = V92_MH_LINE_PHASE_BINS - 1;
        l->bin_energy[bin] = 0.98 * l->bin_energy[bin] + 0.02 * fabs(d);

        /* One decision per symbol, at the first sample on or past the
         * chosen bin's centre.  A sample advances the phase by 0.075, more
         * than a bin, so "entering the bin" would skip symbols. */
        {
            double centre = (l->bin_best + 0.5) / V92_MH_LINE_PHASE_BINS;
            double next = l->sym_pos + BAUD / FS;

            if (l->sym_pos <= centre && next > centre)
                on_bit(l, c, d < 0.0 ? 1 : 0);
            else if (next >= 1.0 && centre < next - 1.0)
                on_bit(l, c, d < 0.0 ? 1 : 0);
        }

        l->sym_pos += BAUD / FS;
        if (l->sym_pos >= 1.0) {
            int best = 0;

            l->sym_pos -= 1.0;
            for (int b = 1; b < V92_MH_LINE_PHASE_BINS; b++)
                if (l->bin_energy[b] > l->bin_energy[best])
                    best = b;
            if (l->bin_energy[best] > 1.2 * l->bin_energy[l->bin_best])
                l->bin_best = best;
        }

        goertzel_step(l->g_peer, l->peer_hz, x);
        goertzel_step(l->g_ansam, ANSAM_HZ, x);
        l->energy += x * x;
        if (++l->blk_n == BLOCK)
            end_of_block(l, c);
    }
}
