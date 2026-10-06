/*
 * v8bis_tones.c -- V.8bis tone signals: generator and detector.  See the
 * header for the clauses; the numbers below are Tables 1 and 2 of V.8bis.
 */
#include "v8bis_tones.h"

#include <math.h>
#include <string.h>

#define PI 3.14159265358979323846
#define DBM0_MAX_SINE 3.14       /* a full-scale sine is +3.14 dBm0 (V.8's convention) */

static const char *const SIG_NAME[V8BIS_SIG_COUNT] = {"MRe", "MRd", "CRe", "CRd", "ESi", "ESr"};
static const double SEG2_HZ[V8BIS_SIG_COUNT] = {650.0, 1150.0, 400.0, 1900.0, 980.0, 1650.0};

const char *v8bis_signal_name(v8bis_signal_t sig)
{
    return sig >= 0 && sig < V8BIS_SIG_COUNT ? SIG_NAME[sig] : "?";
}

double v8bis_signal_seg2_hz(v8bis_signal_t sig)
{
    return sig >= 0 && sig < V8BIS_SIG_COUNT ? SEG2_HZ[sig] : 0.0;
}

void v8bis_toneset_hz(bool responding_set, double *f1, double *f2)
{
    *f1 = responding_set ? 1529.0 : 1375.0;
    *f2 = responding_set ? 2225.0 : 2002.0;
}

bool v8bis_signal_in_toneset(v8bis_signal_t sig, bool responding_set)
{
    switch (sig) {
    case V8BIS_SIG_MRD:
    case V8BIS_SIG_CRD:
        return true;                          /* Table 1 lists both */
    case V8BIS_SIG_ESR:
        return responding_set;
    case V8BIS_SIG_MRE:
    case V8BIS_SIG_CRE:
    case V8BIS_SIG_ESI:
        return !responding_set;
    default:
        return false;
    }
}

double v8bis_signal_default_level_dbm0(v8bis_signal_t sig, double nominal_dbm0)
{
    return (sig == V8BIS_SIG_MRE || sig == V8BIS_SIG_CRE) ? nominal_dbm0 - V8BIS_SIGNAL_E_ATTEN_DB
                                                         : nominal_dbm0;
}

/* ---- generator ------------------------------------------------------ */

static double peak_for_dbm0(double dbm0)
{
    return 32767.0 * pow(10.0, (dbm0 - DBM0_MAX_SINE) / 20.0);
}

void v8bis_tone_tx_start(v8bis_tone_tx_t *s, v8bis_signal_t sig, bool responding_set,
                         double level_dbm0, double ppm)
{
    double a, b;

    memset(s, 0, sizeof(*s));
    v8bis_toneset_hz(responding_set, &a, &b);
    s->f[0] = a;
    s->f[1] = b;
    s->f[2] = v8bis_signal_seg2_hz(sig);
    s->ppm = ppm;
    s->seg1_len = V8BIS_SEG1_SAMPLES;
    s->total = V8BIS_SEG1_SAMPLES + V8BIS_SEG2_SAMPLES;
    /* Two tones sharing the power: each is 3.01 dB under the signal level. */
    s->amp1 = peak_for_dbm0(level_dbm0 - 3.0103);
    s->amp2 = peak_for_dbm0(level_dbm0);
    s->active = true;
}

void v8bis_tone_tx_seg1_only(v8bis_tone_tx_t *s)
{
    s->total = s->seg1_len;
}

bool v8bis_tone_tx_done(const v8bis_tone_tx_t *s)
{
    return !s->active || s->pos >= s->total;
}

int v8bis_tone_tx(v8bis_tone_tx_t *s, int16_t *amp, int len)
{
    int n = 0;

    while (n < len && s->active && s->pos < s->total) {
        double v;

        if (s->pos < s->seg1_len) {
            v = s->amp1 * (sin(s->phase[0]) + sin(s->phase[1]));
            s->phase[0] += 2.0 * PI * s->f[0] * (1.0 + s->ppm * 1e-6) / V8BIS_RATE;
            s->phase[1] += 2.0 * PI * s->f[1] * (1.0 + s->ppm * 1e-6) / V8BIS_RATE;
        } else {
            v = s->amp2 * sin(s->phase[2]);
            s->phase[2] += 2.0 * PI * s->f[2] * (1.0 + s->ppm * 1e-6) / V8BIS_RATE;
        }
        if (v > 32767.0)
            v = 32767.0;
        if (v < -32768.0)
            v = -32768.0;
        amp[n++] = (int16_t)lrint(v);
        s->pos++;
    }
    if (s->pos >= s->total)
        s->active = false;
    return n;
}

/* ---- detector -------------------------------------------------------- */

/* The detector measures, per 20 ms block, the fraction of the block's energy
 * sitting at each frequency of interest (a Goertzel evaluated AT the exact
 * frequency, so a half-bin frequency such as 2225 Hz does not lose 4 dB to
 * scalloping).  Segment 1 is a block where both tones of a pair each hold a
 * share and together most of the energy; segment 2 is a block where one
 * allowed identifying tone holds most of it.  Fractions rather than absolute
 * levels, so the signal is found wherever the line level puts it, and the
 * "most of the energy" test is what keeps loud voice from satisfying it. */

#define MIN_SEG1_BLOCKS 9          /* 180 ms of the nominal 400 */
#define SEG2_WINDOW 8              /* blocks after seg1 in which seg2 must start */
#define SEG2_CONFIRM 3             /* consecutive blocks naming the same signal */
#define PAIR_EACH_MIN 0.12
#define PAIR_SUM_MIN 0.40
#define PAIR_BALANCE_MAX 6.0
#define SEG2_FRAC_MIN 0.50
#define MIN_RMS 30.0

enum { F_1375, F_2002, F_1529, F_2225, F_650, F_1150, F_400, F_1900, F_980, F_1650, F_COUNT };
static const double FREQ[F_COUNT] = {1375, 2002, 1529, 2225, 650, 1150, 400, 1900, 980, 1650};
/* seg2 frequency index per signal */
static const int SEG2_F[V8BIS_SIG_COUNT] = {F_650, F_1150, F_400, F_1900, F_980, F_1650};

static double goertzel_frac(const int16_t *x, int n, double f, double total)
{
    double w = 2.0 * PI * f / V8BIS_RATE, c = 2.0 * cos(w), s1 = 0.0, s2 = 0.0, p;

    for (int i = 0; i < n; i++) {
        double s0 = x[i] + c * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    p = s1 * s1 + s2 * s2 - c * s1 * s2;
    return 2.0 * p / ((double)n * total);
}

void v8bis_tone_rx_init(v8bis_tone_rx_t *s)
{
    memset(s, 0, sizeof(*s));
    s->cand = -1;
    for (int i = 0; i < 2; i++)
        s->miss[i] = 1000;
}

static void push_event(v8bis_tone_rx_t *s, const v8bis_tone_event_t *ev)
{
    if (s->qn == V8BIS_RX_QUEUE) {          /* drop the oldest */
        s->qh = (s->qh + 1) % V8BIS_RX_QUEUE;
        s->qn--;
    }
    s->q[(s->qh + s->qn) % V8BIS_RX_QUEUE] = *ev;
    s->qn++;
}

static void process_block(v8bis_tone_rx_t *s)
{
    const int16_t *x = s->buf;
    double total = 0.0, fr[F_COUNT];
    uint64_t block_start = s->sample - V8BIS_RX_BLOCK;
    bool pass[2] = {false, false}, seg1_any;
    double q[2] = {0.0, 0.0};

    for (int i = 0; i < V8BIS_RX_BLOCK; i++)
        total += (double)x[i] * x[i];
    if (total < MIN_RMS * MIN_RMS * V8BIS_RX_BLOCK) {
        for (int set = 0; set < 2; set++) {
            s->miss[set]++;
            if (s->miss[set] >= 2)
                s->run[set] = 0;
        }
        s->cand = -1;
        s->cand_run = 0;
        return;
    }
    for (int f = 0; f < F_COUNT; f++)
        fr[f] = goertzel_frac(x, V8BIS_RX_BLOCK, FREQ[f], total);

    for (int set = 0; set < 2; set++) {
        double a = fr[set ? F_1529 : F_1375], b = fr[set ? F_2225 : F_2002];
        double hi = a > b ? a : b, lo = a > b ? b : a;

        q[set] = a + b;
        pass[set] = a >= PAIR_EACH_MIN && b >= PAIR_EACH_MIN && a + b >= PAIR_SUM_MIN
                    && hi <= PAIR_BALANCE_MAX * lo;
    }
    seg1_any = pass[0] || pass[1];

    for (int set = 0; set < 2; set++) {
        if (pass[set]) {
            if (s->miss[set] >= 2)           /* the old run is over: start a new one */
                s->run[set] = 0;
            if (s->run[set] == 0) {
                s->run_start[set] = block_start;
                s->qsum[set] = 0.0;
            }
            s->run[set]++;
            s->qsum[set] += q[set];
            s->miss[set] = 0;
        } else {
            if (s->miss[set] < 1000)
                s->miss[set]++;
        }
    }

    /* Segment 2: only on a block that is not itself segment 1, within a few
     * blocks of a run long enough to have been one. */
    if (seg1_any) {
        s->cand = -1;
        s->cand_run = 0;
        return;
    }
    for (int set = 0; set < 2; set++) {
        int best = -1;
        double bestf = 0.0;

        if (s->run[set] < MIN_SEG1_BLOCKS || s->miss[set] < 1 || s->miss[set] > SEG2_WINDOW)
            continue;
        for (int sig = 0; sig < V8BIS_SIG_COUNT; sig++) {
            double v;

            if (!v8bis_signal_in_toneset((v8bis_signal_t)sig, set != 0))
                continue;
            v = fr[SEG2_F[sig]];
            if (v > bestf) {
                bestf = v;
                best = sig;
            }
        }
        if (best < 0 || bestf < SEG2_FRAC_MIN) {
            s->cand = -1;
            s->cand_run = 0;
            continue;
        }
        if (s->cand == best + 8 * set) {
            s->cand_run++;
        } else {
            s->cand = best + 8 * set;
            s->cand_run = 1;
            s->cand_start = block_start;
        }
        if (s->cand_run >= SEG2_CONFIRM) {
            v8bis_tone_event_t ev;

            ev.sig = (v8bis_signal_t)best;
            ev.responding_set = set != 0;
            ev.seg1_start = s->run_start[set];
            ev.detect_sample = s->sample;
            ev.seg1_blocks = (unsigned)s->run[set];
            ev.seg1_quality = s->qsum[set] / s->run[set];
            push_event(s, &ev);
            s->run[set] = 0;
            s->miss[set] = 1000;
            s->cand = -1;
            s->cand_run = 0;
            return;
        }
    }
}

void v8bis_tone_rx(v8bis_tone_rx_t *s, const int16_t *amp, int len)
{
    for (int i = 0; i < len; i++) {
        s->buf[s->fill++] = amp[i];
        s->sample++;
        if (s->fill == V8BIS_RX_BLOCK) {
            process_block(s);
            s->fill = 0;
        }
    }
}

bool v8bis_tone_rx_event(v8bis_tone_rx_t *s, v8bis_tone_event_t *ev)
{
    if (!s->qn)
        return false;
    *ev = s->q[s->qh];
    s->qh = (s->qh + 1) % V8BIS_RX_QUEUE;
    s->qn--;
    return true;
}
