/*
 * v34_line_ec_test.c - the line echo canceller follows a moved bulk delay.
 *
 * Against the RasFinder's AudioCodes gateway the echo of our transmission
 * moved 45-85 ms later in the middle of a call, when a long media callback
 * held up our packets and the gateway re-centred its playout buffer; the
 * canceller, fitted once in Phase 3, then subtracted at the old delay and
 * shut the V.90 upstream eye (docs/v90_rasfinder_phase4.md, 2026-10-16).
 * This builds that situation from synthetic signals: an echo-only training
 * window, then a far-end signal 25 dB above the echo, and part way through
 * the echo moves by 400 samples.  It checks that
 *
 *   - the canceller moves its delay by exactly that much, within 2 s;
 *   - afterwards it cancels the echo again (residual >= 20 dB below it);
 *   - an echo that never moves is never moved, over 60 s;
 *   - a periodic transmission, whose correlation has an alias at every
 *     period, does not move it to a wrong delay.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v34_line_ec.h"

#define FS          8000
#define BLOCK       160
#define TRAIN       12000
#define ECHO_TAPS   48
#define PEER_TAPS   63
#define L1          1900
#define L2          2300

static uint64_t rng_state = 0x9e3779b97f4a7c15ULL;

static double gauss(void)
{
    /* Box-Muller on a 64-bit LCG; deterministic across platforms. */
    double u1;
    double u2;

    rng_state = rng_state*6364136223846793005ULL + 1442695040888963407ULL;
    u1 = ((rng_state >> 11) + 1.0)/9007199254740993.0;
    rng_state = rng_state*6364136223846793005ULL + 1442695040888963407ULL;
    u2 = (rng_state >> 11)/9007199254740992.0;
    return sqrt(-2.0*log(u1))*cos(2.0*M_PI*u2);
}

static int16_t clip16(double v)
{
    if (v > 32767.0)
        return 32767;
    if (v < -32768.0)
        return -32768;
    return (int16_t) lrint(v);
}

typedef struct
{
    int total;          /* samples after training */
    int move_at;        /* sample (after training) the echo moves, or -1 */
    int period;         /* > 0: periodic transmission of this period */
    int shifts;
    int lag_change;
    int moved_after;    /* samples after move_at the canceller moved */
    double echo_rms;
    double residual_rms;  /* of (output - far end) after the move settles */
} run_t;

static void run(run_t *r)
{
    static v34_line_ec_t ec;
    static int16_t tx_hist[1 << 20];
    double g[ECHO_TAPS];
    double peer_h[PEER_TAPS];
    double peer_mem[PEER_TAPS];
    double gnorm = 0.0;
    int16_t period_buf[4096];
    int n_total = TRAIN + r->total;
    int initial_lag = -1;
    double echo_pow = 0.0;
    double res_pow = 0.0;
    int res_n = 0;
    int echo_n = 0;

    rng_state = 0x9e3779b97f4a7c15ULL;
    v34_line_ec_reset(&ec);
    memset(tx_hist, 0, sizeof(tx_hist));
    memset(peer_mem, 0, sizeof(peer_mem));
    /* A decaying echo response at -27 dB against the transmit level. */
    for (int k = 0;  k < ECHO_TAPS;  k++)
    {
        g[k] = gauss()*exp(-k/10.0);
        gnorm += g[k]*g[k];
    }
    for (int k = 0;  k < ECHO_TAPS;  k++)
        g[k] *= 0.045/sqrt(gnorm);
    /* The far end: 200-3400 Hz noise, windowed-sinc band-pass. */
    for (int k = 0;  k < PEER_TAPS;  k++)
    {
        double m = k - (PEER_TAPS - 1)/2.0;
        double hi = (m == 0.0)  ?  2.0*3400.0/FS  :  sin(2.0*M_PI*3400.0/FS*m)/(M_PI*m);
        double lo = (m == 0.0)  ?  2.0*200.0/FS  :  sin(2.0*M_PI*200.0/FS*m)/(M_PI*m);
        double w = 0.54 - 0.46*cos(2.0*M_PI*k/(PEER_TAPS - 1));

        peer_h[k] = (hi - lo)*w;
    }
    for (int i = 0;  i < (int) (sizeof(period_buf)/sizeof(period_buf[0]));  i++)
        period_buf[i] = clip16(4000.0*gauss());
    r->shifts = 0;
    r->moved_after = -1;
    for (int b = 0;  b < n_total;  b += BLOCK)
    {
        int16_t tx[BLOCK];
        int16_t rx[BLOCK];
        double peer[BLOCK];
        bool echo_only = b < TRAIN;

        for (int i = 0;  i < BLOCK;  i++)
        {
            int n = b + i;

            tx[i] = (r->period > 0)  ?  period_buf[n % r->period]  :  clip16(4000.0*gauss());
            tx_hist[n] = tx[i];
        }
        v34_line_ec_tx(&ec, tx, BLOCK);
        for (int i = 0;  i < BLOCK;  i++)
        {
            int n = b + i;
            int lag = (r->move_at >= 0  &&  n >= TRAIN + r->move_at)  ?  L2  :  L1;
            double e = 0.0;
            double p = 0.0;

            for (int k = 0;  k < ECHO_TAPS;  k++)
            {
                int m = n - lag - k;

                if (m >= 0)
                    e += g[k]*tx_hist[m];
            }
            memmove(&peer_mem[1], &peer_mem[0], (PEER_TAPS - 1)*sizeof(peer_mem[0]));
            peer_mem[0] = echo_only  ?  0.0  :  4500.0*gauss();
            for (int k = 0;  k < PEER_TAPS;  k++)
                p += peer_h[k]*peer_mem[k];
            peer[i] = p;
            rx[i] = clip16(e + p);
            if (!echo_only)
            {
                echo_pow += e*e;
                echo_n++;
            }
        }
        {
            char log[256];
            int before = ec.shifts;

            ec.track = !echo_only;
            if (v34_line_ec_rx(&ec, rx, BLOCK, echo_only, log, sizeof(log))  &&  log[0])
                printf("    %6.2f s: %s\n", (double) (b - TRAIN)/FS, log);
            if (initial_lag < 0  &&  ec.fitted)
                initial_lag = ec.lag;
            if (ec.shifts != before  &&  r->moved_after < 0  &&  r->move_at >= 0)
                r->moved_after = b - TRAIN - r->move_at;
        }
        /* Residual echo: what is left once the far end is taken away, over
           the last 2 s of the run. */
        if (b >= n_total - 2*FS)
        {
            for (int i = 0;  i < BLOCK;  i++)
            {
                double d = rx[i] - peer[i];

                res_pow += d*d;
                res_n++;
            }
        }
    }
    r->shifts = ec.shifts;
    r->lag_change = ec.lag - initial_lag;
    r->echo_rms = sqrt(echo_pow/(echo_n ? echo_n : 1));
    r->residual_rms = sqrt(res_pow/(res_n ? res_n : 1));
}

int main(void)
{
    int failures = 0;
    run_t a = {.total = 10*FS, .move_at = 4*FS, .period = 0};
    run_t b = {.total = 60*FS, .move_at = -1, .period = 0};
    run_t c = {.total = 10*FS, .move_at = 4*FS, .period = 640};

    printf("moved echo (+%d samples at 4 s):\n", L2 - L1);
    run(&a);
    printf("  shifts %d, delay change %+d, moved %.2f s after the echo did, "
           "echo rms %.0f, residual %.1f (%.1f dB)\n",
           a.shifts, a.lag_change, a.moved_after/(double) FS, a.echo_rms,
           a.residual_rms, 20.0*log10(a.echo_rms/a.residual_rms));
    if (a.lag_change != L2 - L1  ||  a.moved_after < 0  ||  a.moved_after > 2*FS
        ||  20.0*log10(a.echo_rms/a.residual_rms) < 20.0)
    {
        printf("FAIL: the canceller did not follow the moved echo\n");
        failures++;
    }

    printf("echo that never moves (60 s):\n");
    run(&b);
    printf("  shifts %d, residual %.1f (%.1f dB)\n",
           b.shifts, b.residual_rms, 20.0*log10(b.echo_rms/b.residual_rms));
    if (b.shifts != 0)
    {
        printf("FAIL: a stationary echo was moved\n");
        failures++;
    }

    printf("periodic transmission (period 640), echo moved +%d:\n", L2 - L1);
    run(&c);
    printf("  shifts %d, delay change %+d\n", c.shifts, c.lag_change);
    if (c.lag_change != 0  &&  c.lag_change != L2 - L1)
    {
        printf("FAIL: moved to an alias of a periodic transmission\n");
        failures++;
    }

    printf("%s\n", failures  ?  "v34_line_ec_test: FAILED"  :  "v34_line_ec_test: PASSED");
    return failures  ?  1  :  0;
}
