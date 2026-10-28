/*
 * v34_line_ec.c - fixed line echo canceller for plain V.34.
 *
 * V.34 clause 1 makes the modem full duplex with "echo cancellation
 * techniques for channel separation", and from Phase 4 on both ends
 * transmit at once in the same band.  Against the RasFinder (MT5634SMI)
 * over the VG224 ATA our own transmission comes back 267 ms later
 * (2136 samples -- the same bulk delay the V.90 work measured on this path)
 * at about -21 dB.  That is the whole of the Phase 4 receive SNR:
 * demodulated offline with an adaptive equalizer, the peer's Phase 4 signal
 * sits 5.5 degrees from the grid (~22 dB), against 1.6 degrees (~33 dB) once
 * the echo is subtracted -- the difference between ~14400 and 31200 bit/s
 * (rf-tower-v34cma-1).
 *
 * The call modem is handed a clean training window by the protocol itself:
 * 11.3.1.2.4 has the answer modem silent from our S-to-S-bar transition until
 * its Phase 4 S, so for about a second the receiver hears nothing but our own
 * Phase 3.  A 96-tap least-squares fit over that window, at a bulk delay found
 * by cross-correlation, removes 29 dB of it offline, and the path is linear
 * and stable (the V.90 work found the same echo 99.7% explained at the same
 * delay).  So the filter is fitted once and then held: it never adapts
 * during double talk, which is what the NLMS canceller in modem_engine.c
 * cannot avoid -- and that one spans 128 ms from the latest sample, short of
 * a 267 ms echo altogether.
 *
 * A fit that does not remove at least 6 dB is not put in force, so a path
 * with no echo (a digital loopback, the SmartLink rig) is left untouched.
 *
 * "Stable" holds for the echo's shape but not for its delay.  The bulk delay
 * includes the far-end gateway's playout buffer, and that moves whenever our
 * packets arrive late: against the RasFinder's AudioCodes a 107 ms stall in
 * our media callback put the echo 80 ms later, mid-call, and a canceller still
 * subtracting at the old delay doubles the echo instead of removing it -- the
 * V.90 upstream at 21600 went from 0.02 to 0.62 from the lattice in twenty
 * symbols, and back to 0.05 offline with the same filter moved to the new
 * delay (docs/v90_rasfinder_phase4.md, 2026-10-16).  So with `track` set the
 * canceller keeps measuring where its own prediction of the echo sits in what
 * is received, and follows it when it moves.  It does not adapt the filter:
 * the far end's signal is twenty-odd dB above the echo, and it is only a
 * delay that changes.  See track_rx().
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v34_line_ec.h"

#define RING_MASK (V34_LEC_RING - 1)
#define TRAIN_RING (V34_LEC_TRAIN_MAX + V34_LEC_TAIL_MAX)

void v34_line_ec_reset(v34_line_ec_t *ec)
{
    memset(ec, 0, sizeof(*ec));
}

void v34_line_ec_abort_window(v34_line_ec_t *ec)
{
    ec->training = false;
    ec->train_total = 0;
    ec->fit_pending = false;
}

void v34_line_ec_tx(v34_line_ec_t *ec, const int16_t *amp, int len)
{
    for (int i = 0;  i < len;  i++)
    {
        ec->tx[ec->tx_count & RING_MASK] = amp[i];
        ec->tx_count++;
    }
}

/* TX sample with absolute index idx, or 0 if it is not (or no longer) held. */
static float tx_at(const v34_line_ec_t *ec, int64_t idx)
{
    if (idx < 0
        ||  (uint64_t) idx >= ec->tx_count
        ||  ec->tx_count - (uint64_t) idx > V34_LEC_RING)
        return 0.0f;
    return (float) ec->tx[(uint64_t) idx & RING_MASK];
}

/* Cholesky solve of the symmetric positive definite n x n system a x = b.
   a is overwritten.  Returns false if a is not positive definite. */
static bool chol_solve(double *a, const double *b, double *x, int n)
{
    for (int j = 0;  j < n;  j++)
    {
        double d = a[j*n + j];

        for (int k = 0;  k < j;  k++)
            d -= a[j*n + k]*a[j*n + k];
        if (d <= 0.0)
            return false;
        d = sqrt(d);
        a[j*n + j] = d;
        for (int i = j + 1;  i < n;  i++)
        {
            double s = a[i*n + j];

            for (int k = 0;  k < j;  k++)
                s -= a[i*n + k]*a[j*n + k];
            a[i*n + j] = s/d;
        }
    }
    for (int i = 0;  i < n;  i++)
    {
        double s = b[i];

        for (int k = 0;  k < i;  k++)
            s -= a[i*n + k]*x[k];
        x[i] = s/a[i*n + i];
    }
    for (int i = n - 1;  i >= 0;  i--)
    {
        double s = x[i];

        for (int k = i + 1;  k < n;  k++)
            s -= a[k*n + i]*x[k];
        x[i] = s/a[i*n + i];
    }
    return true;
}

static void track_reset(v34_line_ec_t *ec);

/* Bulk-delay search block: how many lags one pass over the received samples
   correlates at once.  Each lag keeps its own accumulator, summed in the same
   order as one lag at a time, so the block only changes the speed. */
#define LAG_BLOCK V34_LEC_FIT_LAG_BLOCK
#define LIN_LEN V34_LEC_FIT_LIN_LEN

static void fit_begin(v34_line_ec_t *ec)
{
    const int64_t lin0 = (int64_t) ec->train_start - V34_LEC_MAX_LAG
                       - V34_LEC_TAPS - LAG_BLOCK;

    ec->fits++;
    for (int i = 0; i < LIN_LEN; i++)
        ec->fit_tx[i] = tx_at(ec, lin0 + i);
    ec->fit_best_corr = -1.0;
    ec->fit_best_lag = -1;
    ec->fit_next_lag = V34_LEC_TAPS/2;
    ec->fit_pending = true;
}

/* Preserve each correlation's summation order and the original tie-break.
   Only the number of lags evaluated in one callback changes. */
static bool fit_search(v34_line_ec_t *ec)
{
    const int s0 = ec->train_len - V34_LEC_SEARCH_LEN;
    const int64_t lin0 = (int64_t) ec->train_start - V34_LEC_MAX_LAG
                       - V34_LEC_TAPS - LAG_BLOCK;
    const int end = ec->defer_fit ? ec->fit_next_lag + 16*LAG_BLOCK
                                : V34_LEC_MAX_LAG + 1;

    for (; ec->fit_next_lag <= V34_LEC_MAX_LAG && ec->fit_next_lag < end;
         ec->fit_next_lag += LAG_BLOCK)
    {
        const int d0 = ec->fit_next_lag;
        double c[LAG_BLOCK] = {0.0};
        const double *t = ec->fit_tx
                       + ((int64_t) ec->train_start + s0 - d0 - lin0);

        for (int n = 0; n < V34_LEC_SEARCH_LEN; n++)
        {
            const double y = ec->train_rx[s0 + n];

            for (int j = 0; j < LAG_BLOCK; j++)
                c[j] += y*t[n - j];
        }
        for (int j = 0; j < LAG_BLOCK && d0 + j <= V34_LEC_MAX_LAG; j++)
        {
            const double cj = fabs(c[j]);

            if (cj > ec->fit_best_corr)
            {
                ec->fit_best_corr = cj;
                ec->fit_best_lag = d0 + j;
            }
        }
    }
    return ec->fit_next_lag > V34_LEC_MAX_LAG;
}

static void fit_finish(v34_line_ec_t *ec, char *log, size_t log_len)
{
    double r[V34_LEC_TAPS*V34_LEC_TAPS];
    const double *lin = ec->fit_tx;
    double p[V34_LEC_TAPS];
    double x[V34_LEC_TAPS];
    double trace;
    int first;
    const int n_train = ec->train_len;
    const int half = V34_LEC_TAPS/2;
    const int64_t lin0 = (int64_t) ec->train_start - V34_LEC_MAX_LAG
                       - V34_LEC_TAPS - LAG_BLOCK;
    const int lag = ec->fit_best_lag;
    double rx_pow = 0.0;
    double res_pow = 0.0;
    int64_t base;
    int boff;

    ec->fit_pending = false;
    if (lag < 0)
        return;
    /* Least squares over the whole window, on the exact covariance matrix.
       The Toeplitz shortcut this replaces -- one lag product summed over the
       window, standing in for every (j, k) pair at that lag -- is not
       guaranteed positive definite, and on Phase 3's S and PP, which are
       line spectra, it was not: the fit at the right bulk delay failed as
       singular and the call ran with no canceller.  The covariance of real
       data is positive semi-definite by construction, and the diagonal
       loading makes it definite whatever the transmit spectrum. */
    /* What arrives in the first round trip of the window can still carry the
       far end: it stops only once it has heard our S, and its last sample
       then takes a one-way trip to reach us.  The echo's bulk delay IS that
       round trip when the hybrid is at the far end, so fit only on what
       arrived at least one bulk delay (plus 30 ms for its S detector) after
       the window opened.  Measured in v34_duplex_test: the call modem's
       window opened on 10 ms of the answer modem's J at full level, and
       including it held the fit to 2.5 dB where the rest of the window
       supports 35. */
    first = lag + 240;
    if (first > n_train - V34_LEC_FIT_MIN)
        first = n_train - V34_LEC_FIT_MIN;
    if (first < 0)
        first = 0;
    base = (int64_t) ec->train_start - lag + half;
    boff = (int) (base - lin0);
    memset(r, 0, sizeof(r));
    memset(p, 0, sizeof(p));
    /* First row and the cross-correlation directly, O(N*L)... */
    for (int n = first;  n < n_train;  n++)
    {
        const double y = ec->train_rx[n];
        const double *xn = lin + boff + n;
        const double x0 = xn[0];

        for (int k = 0;  k < V34_LEC_TAPS;  k++)
        {
            p[k] += y*xn[-k];
            r[k] += x0*xn[-k];
        }
    }
    /* ...and the rest by the exact shift recursion, O(L^2):
       R[j+1][k+1] = R[j][k] + x(first-1-j)x(first-1-k) - x(N-1-j)x(N-1-k),
       with x(m-j) = tx(base + m - j).  This runs once per Phase 3 inside the
       media callback, where the direct O(N*L^2) sum (37M products) would be
       a stall of several frames. */
    for (int j = 0;  j < V34_LEC_TAPS - 1;  j++)
    {
        const double *xa = lin + boff + first - 1;
        const double *xb = lin + boff + n_train - 1;
        const double a = xa[-j];
        const double b = xb[-j];

        for (int k = j;  k < V34_LEC_TAPS - 1;  k++)
            r[(j + 1)*V34_LEC_TAPS + k + 1] = r[j*V34_LEC_TAPS + k]
                                            + a*xa[-k]
                                            - b*xb[-k];
    }
    trace = 0.0;
    for (int j = 0;  j < V34_LEC_TAPS;  j++)
    {
        for (int k = 0;  k < j;  k++)
            r[j*V34_LEC_TAPS + k] = r[k*V34_LEC_TAPS + j];
        trace += r[j*V34_LEC_TAPS + j];
    }
    for (int j = 0;  j < V34_LEC_TAPS;  j++)
        r[j*V34_LEC_TAPS + j] += 1e-6*trace/V34_LEC_TAPS + 1.0;
    if (!chol_solve(r, p, x, V34_LEC_TAPS))
    {
        if (log)
            snprintf(log, log_len, "line echo canceller: fit failed (singular) at lag %d", lag);
        return;
    }
    for (int n = first;  n < n_train;  n++)
    {
        const double *xn = lin + boff + n;
        double e = ec->train_rx[n];

        for (int k = 0;  k < V34_LEC_TAPS;  k++)
            e -= x[k]*xn[-k];
        rx_pow += (double) ec->train_rx[n]*ec->train_rx[n];
        res_pow += e*e;
    }
    ec->erle_db = (res_pow > 0.0  &&  rx_pow > 0.0)
                ?  (float) (10.0*log10(rx_pow/res_pow))  :  0.0f;
    if (ec->erle_db >= 6.0f)
    {
        ec->lag = lag;
        for (int k = 0;  k < V34_LEC_TAPS;  k++)
            ec->h[k] = (float) x[k];
        ec->fitted = true;
        track_reset(ec);
    }
    if (log)
    {
        snprintf(log, log_len,
                 "line echo canceller: fit %d over %d echo-only samples, bulk delay %d samples "
                 "(%.1f ms), echo rms %.0f -> %.0f (%.1f dB)%s",
                 ec->fits, n_train - first, lag, lag/8.0,
                 sqrt(rx_pow/(n_train - first)), sqrt(res_pow/(n_train - first)), ec->erle_db,
                 (ec->erle_db >= 6.0f)  ?  ", in force"  :  ", too little echo to cancel; not applied");
    }
}

#define TRK_MASK (V34_LEC_TRACK_RING - 1)
#define TRK_R V34_LEC_TRACK_RANGE

/* The fitted filter's prediction of the echo in received sample m, or 0 for
   any part of it not yet transmitted. */
static float echo_at(const v34_line_ec_t *ec, int64_t m)
{
    const int64_t base = m - ec->lag + V34_LEC_TAPS/2;
    float est = 0.0f;

    for (int k = 0;  k < V34_LEC_TAPS;  k++)
        est += ec->h[k]*tx_at(ec, base - k);
    return est;
}

/* Predict (and whiten) the echo up to received index `upto`, starting afresh
   TRK_R + LPC samples before `from` if the tracker has just been (re)armed. */
static void track_predict(v34_line_ec_t *ec, uint64_t from, uint64_t upto)
{
    if (ec->trk_next == 0)
    {
        int64_t start = (int64_t) from - TRK_R - V34_LEC_TRACK_LPC;

        ec->trk_next = (start > 1)  ?  (uint64_t) start  :  1;
        for (int i = 0;  i < V34_LEC_TRACK_LPC;  i++)
            ec->trk_p[(ec->trk_next - 1 - (uint64_t) i) & TRK_MASK] = 0.0f;
    }
    for (  ;  ec->trk_next <= upto;  ec->trk_next++)
    {
        const uint64_t m = ec->trk_next;
        float w;

        ec->trk_p[m & TRK_MASK] = echo_at(ec, (int64_t) m);
        w = 0.0f;
        for (int j = 0;  j <= V34_LEC_TRACK_LPC;  j++)
            w += ec->trk_lpc[j]*ec->trk_p[(m - (uint64_t) j) & TRK_MASK];
        ec->trk_pw[m & TRK_MASK] = w;
    }
}

static void track_reset(v34_line_ec_t *ec)
{
    if (ec->trk_lpc[0] == 0.0f)
        ec->trk_lpc[0] = 1.0f;
    ec->trk_next = 0;
    ec->trk_sub = 0;
    ec->trk_pos = 0;
    ec->trk_filled = 0;
    memset(ec->trk_acc, 0, sizeof(ec->trk_acc));
    memset(ec->trk_rac, 0, sizeof(ec->trk_rac));
}

/* Levinson-Durbin on the sub-window's autocorrelation: the prediction-error
   filter that whitens what is received.  The far end's signal is what fills
   the band, and the echo stands out of it only at the edges (the RasFinder
   through the AudioCodes: -25 dB in band, -5 to -9 dB at 3.6-3.9 kHz), so
   weighting by the inverse of the received spectrum is what makes the echo's
   correlation peak visible in a second rather than several (offline, 9 sigma
   against 6 with no filter, and the first difference was worse than none). */
static void track_whiten(v34_line_ec_t *ec)
{
    double r[V34_LEC_TRACK_LPC + 1];
    double a[V34_LEC_TRACK_LPC + 1];
    double tmp[V34_LEC_TRACK_LPC + 1];
    double err;

    if (ec->trk_rac[0] <= 0.0)
        return;
    for (int k = 0;  k <= V34_LEC_TRACK_LPC;  k++)
        r[k] = ec->trk_rac[k];
    r[0] *= 1.0001;
    memset(a, 0, sizeof(a));
    a[0] = 1.0;
    err = r[0];
    for (int i = 1;  i <= V34_LEC_TRACK_LPC;  i++)
    {
        double acc = r[i];
        double kref;

        for (int j = 1;  j < i;  j++)
            acc += a[j]*r[i - j];
        kref = -acc/err;
        memcpy(tmp, a, sizeof(a));
        for (int j = 1;  j < i;  j++)
            a[j] = tmp[j] + kref*tmp[i - j];
        a[i] = kref;
        err *= (1.0 - kref*kref);
        if (err <= 0.0)
            return;
    }
    for (int j = 0;  j <= V34_LEC_TRACK_LPC;  j++)
        ec->trk_lpc[j] = (float) a[j];
}

/* One raw (not yet cancelled) received sample with absolute index n.
   Correlates the whitened sample against the whitened predicted echo at every
   shift D in [-TRK_R, TRK_R]: echo that has moved by D samples correlates at
   D.  Every sub-window the last second's sum is examined, and the bulk delay
   moves when a shifted peak stands six times the correlation's spread clear of
   the floor and three times above the peak where the canceller is now.
   Returns the shift made, or 0. */
static int track_rx(v34_line_ec_t *ec, uint64_t n, float raw)
{
    float y;
    float *acc;
    int best_d = 0;
    float best = 0.0f;
    float at0;
    float second = 0.0f;
    double floor_sum = 0.0;
    int floor_n = 0;
    double sigma;

    track_predict(ec, n, n + TRK_R);
    memmove(&ec->trk_rx[1], &ec->trk_rx[0], V34_LEC_TRACK_LPC*sizeof(ec->trk_rx[0]));
    ec->trk_rx[0] = raw;
    y = 0.0f;
    for (int j = 0;  j <= V34_LEC_TRACK_LPC;  j++)
    {
        y += ec->trk_lpc[j]*ec->trk_rx[j];
        ec->trk_rac[j] += (double) ec->trk_rx[0]*ec->trk_rx[j];
    }
    acc = ec->trk_acc[ec->trk_sub];
    for (int d = -TRK_R;  d <= TRK_R;  d++)
        acc[d + TRK_R] += y*ec->trk_pw[(n - (uint64_t) (int64_t) d) & TRK_MASK];
    if (++ec->trk_pos < V34_LEC_TRACK_SUB)
        return 0;
    /* A sub-window is complete: refresh the whitening filter for the next. */
    track_whiten(ec);
    memset(ec->trk_rac, 0, sizeof(ec->trk_rac));
    ec->trk_pos = 0;
    ec->trk_sub = (ec->trk_sub + 1) % V34_LEC_TRACK_NSUB;
    if (ec->trk_filled < V34_LEC_TRACK_NSUB)
        ec->trk_filled++;
    if (ec->trk_filled < V34_LEC_TRACK_NSUB)
    {
        memset(ec->trk_acc[ec->trk_sub], 0, sizeof(ec->trk_acc[0]));
        return 0;
    }
    {
        float sum[2*TRK_R + 1];

        for (int i = 0;  i <= 2*TRK_R;  i++)
        {
            float v = 0.0f;

            for (int s = 0;  s < V34_LEC_TRACK_NSUB;  s++)
                v += ec->trk_acc[s][i];
            sum[i] = v;
            if (fabsf(v) > best)
            {
                best = fabsf(v);
                best_d = i - TRK_R;
            }
        }
        at0 = fabsf(sum[TRK_R]);
        /* The spread, away from the peak and from the present alignment:
           the echo's own autocorrelation is a filter's width either side. */
        for (int i = 0;  i <= 2*TRK_R;  i++)
        {
            int d = i - TRK_R;

            if (abs(d - best_d) <= V34_LEC_TAPS  ||  abs(d) <= V34_LEC_TAPS)
                continue;
            floor_sum += (double) sum[i]*sum[i];
            floor_n++;
            if (fabsf(sum[i]) > second)
                second = fabsf(sum[i]);
        }
    }
    /* The oldest sub-window is the one accumulated next. */
    memset(ec->trk_acc[ec->trk_sub], 0, sizeof(ec->trk_acc[0]));
    sigma = (floor_n > 0)  ?  sqrt(floor_sum/floor_n)  :  0.0;
    /* A periodic transmission (Phase 3's S, a retrain's TRN) correlates
       equally at every multiple of its period, and which alias wins is
       noise: insist that the peak stands well clear of the next one too.
       Offline on the RasFinder's 9600 call the true peak is 9-12 sigma and
       the floor's largest about 3.5. */
    if (best_d == 0  ||  sigma <= 0.0  ||  best < 6.0*sigma  ||  best < 3.0f*at0
        ||  best < 1.5f*second)
        return 0;
    ec->lag += best_d;
    ec->shifts++;
    track_reset(ec);
    return best_d;
}

bool v34_line_ec_rx(v34_line_ec_t *ec, int16_t *amp, int len, bool echo_only,
                    char *log, size_t log_len)
{
    bool fitted_now = false;

    if (log  &&  log_len)
        log[0] = '\0';
    if (echo_only)
    {
        if (!ec->training)
        {
            ec->training = true;
            ec->train_total = 0;
            ec->fit_pending = false;
        }
        /* Recorded before any cancellation below, so a retrain's refit sees
           the raw echo rather than what an older fit left of it. */
        for (int i = 0;  i < len;  i++)
            ec->train_ring[(ec->train_total++) % TRAIN_RING] = amp[i];
    }
    else if (ec->training)
    {
        uint64_t trim = (ec->tail_trim > 0)  ?  (uint64_t) ec->tail_trim  :  0;
        uint64_t usable;

        if (trim > V34_LEC_TAIL_MAX)
            trim = V34_LEC_TAIL_MAX;
        usable = (ec->train_total > trim)  ?  ec->train_total - trim  :  0;
        ec->training = false;
        ec->train_len = (usable < V34_LEC_TRAIN_MAX)
                      ?  (int) usable  :  V34_LEC_TRAIN_MAX;
        /* rx_count has not yet advanced past this block, so it is the index
           of the first sample after the window; the kept samples end `trim`
           before that. */
        ec->train_start = ec->rx_count - trim - (uint64_t) ec->train_len;
        for (int i = 0;  i < ec->train_len;  i++)
            ec->train_rx[i] = ec->train_ring[(usable - (uint64_t) ec->train_len + (uint64_t) i)
                                             % TRAIN_RING];
        if (ec->train_len >= V34_LEC_TRAIN_MIN)
        {
            fit_begin(ec);
        }
    }
    if (ec->fit_pending && fit_search(ec))
    {
        fit_finish(ec, log, log_len);
        fitted_now = true;
    }
    if (ec->fitted)
    {
        const int half = V34_LEC_TAPS/2;

        for (int i = 0;  i < len;  i++)
        {
            /* Follow the echo if it has moved; never while a training window
               is open, which will refit it anyway. */
            if (!ec->track  ||  ec->training)
                ec->trk_armed = false;
            else
            {
                int old_lag = ec->lag;
                int moved;

                if (!ec->trk_armed)
                {
                    track_reset(ec);
                    ec->trk_armed = true;
                }
                moved = track_rx(ec, ec->rx_count + (uint64_t) i, (float) amp[i]);

                if (moved  &&  log  &&  log_len)
                {
                    snprintf(log, log_len,
                             "line echo canceller: echo moved %+d samples (%+.1f ms), "
                             "bulk delay %d -> %d samples (shift %d)",
                             moved, moved/8.0, old_lag, ec->lag, ec->shifts);
                    fitted_now = true;
                }
            }
            int64_t base = (int64_t) (ec->rx_count + (uint64_t) i) - ec->lag + half;
            float est = 0.0f;
            float y;

            for (int k = 0;  k < V34_LEC_TAPS;  k++)
                est += ec->h[k]*tx_at(ec, base - k);
            y = (float) amp[i] - est;
            if (y > 32767.0f)
                y = 32767.0f;
            else if (y < -32768.0f)
                y = -32768.0f;
            amp[i] = (int16_t) lrintf(y);
        }
    }
    ec->rx_count += (uint64_t) len;
    return fitted_now;
}
