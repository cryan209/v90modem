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
#define TRAIN_RING (V34_LEC_TRAIN_MAX + V34_LEC_TAIL_MAX)

void v34_line_ec_reset(v34_line_ec_t *ec)
{
    memset(ec, 0, sizeof(*ec));
}

void v34_line_ec_abort_window(v34_line_ec_t *ec)
{
    ec->training = false;
    ec->train_total = 0;
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
    double xv[V34_LEC_TAPS];
    double trace;
    int first;
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
    memset(r, 0, sizeof(r));
    memset(p, 0, sizeof(p));
    /* First row and the cross-correlation directly, O(N*L)... */
    for (int n = first;  n < n_train;  n++)
    {
        const double y = ec->train_rx[n];
        const double x0 = tx_at(ec, base + n);

        for (int k = 0;  k < V34_LEC_TAPS;  k++)
        {
            xv[k] = tx_at(ec, base + n - k);
            p[k] += y*xv[k];
            r[k] += x0*xv[k];
        }
    }
    /* ...and the rest by the exact shift recursion, O(L^2):
       R[j+1][k+1] = R[j][k] + x(first-1-j)x(first-1-k) - x(N-1-j)x(N-1-k),
       with x(m-j) = tx(base + m - j).  This runs once per Phase 3 inside the
       media callback, where the direct O(N*L^2) sum (37M products) would be
       a stall of several frames. */
    for (int j = 0;  j < V34_LEC_TAPS - 1;  j++)
    {
        const double a = tx_at(ec, base + first - 1 - j);
        const double b = tx_at(ec, base + n_train - 1 - j);

        for (int k = j;  k < V34_LEC_TAPS - 1;  k++)
            r[(j + 1)*V34_LEC_TAPS + k + 1] = r[j*V34_LEC_TAPS + k]
                                            + a*tx_at(ec, base + first - 1 - k)
                                            - b*tx_at(ec, base + n_train - 1 - k);
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
                 ec->fits, n_train - first, lag, lag/8.0,
                 sqrt(rx_pow/(n_train - first)), sqrt(res_pow/(n_train - first)), ec->erle_db,
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
            ec->train_total = 0;
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
