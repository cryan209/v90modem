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
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v34_line_ec.h"

#define RING_MASK (V34_LEC_RING - 1)

void v34_line_ec_reset(v34_line_ec_t *ec)
{
    memset(ec, 0, sizeof(*ec));
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

static void fit(v34_line_ec_t *ec, char *log, size_t log_len)
{
    static double r[V34_LEC_TAPS*V34_LEC_TAPS];
    double p[V34_LEC_TAPS];
    double x[V34_LEC_TAPS];
    double ac[V34_LEC_TAPS];
    const int n_train = ec->train_len;
    const int half = V34_LEC_TAPS/2;
    const int s0 = n_train - V34_LEC_SEARCH_LEN;
    double best = -1.0;
    int lag = -1;
    double rx_pow = 0.0;
    double res_pow = 0.0;
    int64_t base;

    ec->fits++;
    /* Bulk delay: the lag of the largest cross-correlation between the last
       V34_LEC_SEARCH_LEN received samples and our transmission before them. */
    for (int d = half;  d <= V34_LEC_MAX_LAG;  d++)
    {
        double c = 0.0;

        for (int n = 0;  n < V34_LEC_SEARCH_LEN;  n++)
        {
            int64_t rn = (int64_t) ec->train_start + s0 + n;

            c += (double) ec->train_rx[s0 + n]*tx_at(ec, rn - d);
        }
        c = fabs(c);
        if (c > best)
        {
            best = c;
            lag = d;
        }
    }
    if (lag < 0)
        return;
    /* Least squares over the whole window.  Toeplitz normal equations from
       the transmit autocorrelation: the window is 20-80 times the filter
       length, where the end effects this ignores are negligible. */
    base = (int64_t) ec->train_start - lag + half;
    for (int k = 0;  k < V34_LEC_TAPS;  k++)
    {
        double a = 0.0;
        double q = 0.0;

        for (int n = 0;  n < n_train;  n++)
        {
            double t0 = tx_at(ec, base + n);

            a += t0*tx_at(ec, base + n - k);
            q += (double) ec->train_rx[n]*tx_at(ec, base + n - k);
        }
        ac[k] = a;
        p[k] = q;
    }
    for (int j = 0;  j < V34_LEC_TAPS;  j++)
        for (int k = 0;  k < V34_LEC_TAPS;  k++)
            r[j*V34_LEC_TAPS + k] = ac[abs(j - k)] + ((j == k)  ?  1e-6*ac[0] + 1.0  :  0.0);
    if (!chol_solve(r, p, x, V34_LEC_TAPS))
    {
        if (log)
            snprintf(log, log_len, "line echo canceller: fit failed (singular) at lag %d", lag);
        return;
    }
    for (int n = 0;  n < n_train;  n++)
    {
        double e = ec->train_rx[n];

        for (int k = 0;  k < V34_LEC_TAPS;  k++)
            e -= x[k]*tx_at(ec, base + n - k);
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
    }
    if (log)
    {
        snprintf(log, log_len,
                 "line echo canceller: fit %d over %d echo-only samples, bulk delay %d samples "
                 "(%.1f ms), echo rms %.0f -> %.0f (%.1f dB)%s",
                 ec->fits, n_train, lag, lag/8.0,
                 sqrt(rx_pow/n_train), sqrt(res_pow/n_train), ec->erle_db,
                 (ec->erle_db >= 6.0f)  ?  ", in force"  :  ", too little echo to cancel; not applied");
    }
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
            ec->train_len = 0;
            ec->train_start = ec->rx_count;
        }
        for (int i = 0;  i < len  &&  ec->train_len < V34_LEC_TRAIN_MAX;  i++)
            ec->train_rx[ec->train_len++] = amp[i];
    }
    else if (ec->training)
    {
        ec->training = false;
        if (ec->train_len >= V34_LEC_TRAIN_MIN)
        {
            fit(ec, log, log_len);
            fitted_now = true;
        }
    }
    if (ec->fitted)
    {
        const int half = V34_LEC_TAPS/2;

        for (int i = 0;  i < len;  i++)
        {
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
