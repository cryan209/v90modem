/*
 * near_echo.c — see near_echo.h.
 *
 * One exact least-squares fit (the covariance method) of a fixed window of
 * taps, accumulated only over echo-only blocks.  An earlier version located
 * the response with a wide Toeplitz fit and subtracted a shorter exact one;
 * on gated 50 ms blocks against lags reaching 1024 samples back the Toeplitz
 * matrix is simply the wrong one, and the window it chose wandered by 130
 * samples between solves of the same echo.  A window long enough to hold the
 * response at any of the delays seen needs no locating at all.
 *
 * The normal equations are built per block from its first row and the sliding
 * recursion (accept_block()), so a block costs taps*(BLOCK + taps) operations.
 * The Cholesky solve costs taps^3/6 -- ~75 M at 768 taps -- which is far too
 * long to spend in a device's receive callback, so near_echo_set_threaded()
 * moves it to a worker.
 *
 * Either way a solve requested at receive sample n is INSTALLED at exactly
 * n + solve_every, never "whenever the worker finished".  The canceller's
 * output feeds the DIL measurement that picks the constellation, so an
 * install instant that varied with thread timing made a live call and its
 * own --rx-replay choose different constellations (measured, v90-dfe1:
 * Mi 19 18 19 25 17 19 live against 17 17 16 16 17 17 replayed), and then
 * nothing after Phase 3 could be replayed.  The worker has a whole interval
 * (0.5 s) for ~75 M operations; if it ever has not finished, the receive
 * path waits for it rather than install at a different instant.
 */

#include <math.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>

#include "near_echo.h"

#define RING        16384           /* transmit history; power of two */
#define RMS_ALPHA   (1.0/9600.0)    /* ~1 s at the device rate */
/*
 * Only echo-only blocks are accumulated.  An LS fit's excess error is
 * taps/samples times the interference power, so a fit made under Phase 4's
 * double talk (downstream ~1000 rms, echo ~140) would need hours to converge
 * -- measured, a double-talk fit removed 7 dB where an echo-only one removes
 * 31.  A block is echo-only when what came back is at least ECHO_GATE below
 * what went out: the Apple modem's echo returns at -27 dB, a downstream during
 * our transmit at -10 dB or louder.  V.34 Phase 2's L1/L2 (§10.2) is such a
 * window on every attempt -- the far end is silent by design while the caller
 * probes.
 */
#define BLOCK       480             /* 50 ms */
#define ECHO_GATE   0.01            /* -20 dB, power */
#define BLOCK_TX_MIN (300.0*300.0)
/* Echo-only blocks before the first fit is believed: half a second. */
#define MIN_BLOCKS  10
/* Diagonal loading, relative to the diagonal's mean.  Keeps the solve defined
 * on a narrowband transmit (V.21, tones) without visibly biasing a broadband
 * one. */
#define LOAD        1e-5

struct near_echo_s {
    int lag, taps, solve_every;

    int16_t  ring[RING];
    volatile uint64_t tx_count;     /* written by the transmit side only */
    uint64_t rx_count;

    /* The current block, until it is judged. */
    int16_t *cur_y;
    int      blk_n;
    double   blk_rx, blk_tx;
    int16_t *bx;                    /* scratch: the block's transmit span */

    /* Normal equations over the echo-only blocks so far. */
    double  *R, *q;
    long     blocks_used;
    int      fresh;                 /* blocks added since the last solve */
    int      since_solve;

    /* The taps being subtracted; replaced whole by a finished solve. */
    double  *h;
    int      solved, active;
    double   pow_in, pow_echo;
    int64_t  install_at;            /* rx sample a pending solve lands at, -1 none */

    /* Solving: inline, or on a worker that publishes into h_next. */
    int      threaded;
    pthread_t worker;
    pthread_mutex_t mtx;
    pthread_cond_t  cond;
    int      busy, quit, ready;
    double  *Rw, *qw, *hw, *chol;   /* worker's copies */
};

static inline double tx_at(const near_echo_t *s, int64_t m)
{
    uint64_t have = __atomic_load_n(&s->tx_count, __ATOMIC_ACQUIRE);

    if (m < 0  ||  (uint64_t) m >= have  ||  have - (uint64_t) m > RING)
        return 0.0;
    return s->ring[(uint64_t) m & (RING - 1)];
}

/* Cholesky solve of (R + load) h = q.  0 if not positive definite. */
static int cholesky_solve(const double *R, const double *q, int T,
                          double *L, double *h)
{
    double mean = 0.0;

    for (int i = 0; i < T; i++)
        mean += R[i*T + i];
    mean /= T;
    if (mean <= 0.0)
        return 0;
    for (int i = 0; i < T; i++) {
        for (int j = 0; j <= i; j++) {
            double v = R[i*T + j] + (i == j ? LOAD*mean : 0.0);
            const double *Li = L + (size_t) i*T, *Lj = L + (size_t) j*T;

            for (int k = 0; k < j; k++)
                v -= Li[k]*Lj[k];
            if (i == j) {
                if (v <= 0.0)
                    return 0;
                L[i*T + i] = sqrt(v);
            } else {
                L[i*T + j] = v/L[j*T + j];
            }
        }
    }
    for (int i = 0; i < T; i++) {
        double v = q[i];

        for (int k = 0; k < i; k++)
            v -= L[i*T + k]*h[k];
        h[i] = v/L[i*T + i];
    }
    for (int i = T - 1; i >= 0; i--) {
        double v = h[i];

        for (int k = i + 1; k < T; k++)
            v -= L[k*T + i]*h[k];
        h[i] = v/L[i*T + i];
    }
    return 1;
}

static void *worker_main(void *arg)
{
    near_echo_t *s = arg;

    pthread_mutex_lock(&s->mtx);
    for (;;) {
        while (!s->busy  &&  !s->quit)
            pthread_cond_wait(&s->cond, &s->mtx);
        if (s->quit)
            break;
        pthread_mutex_unlock(&s->mtx);
        /* Rw/qw were filled by the requester; nothing else touches them while
         * busy is set. */
        {
            int ok = cholesky_solve(s->Rw, s->qw, s->taps, s->chol, s->hw);

            pthread_mutex_lock(&s->mtx);
            s->ready = ok;
        }
        s->busy = 0;
        pthread_cond_broadcast(&s->cond);
    }
    pthread_mutex_unlock(&s->mtx);
    return NULL;
}

near_echo_t *near_echo_init(int lag, int taps, int solve_every)
{
    near_echo_t *s;
    size_t T = (size_t) taps;

    if (lag < 0  ||  taps < 1  ||  lag + taps + BLOCK >= RING/2)
        return NULL;
    if ((s = calloc(1, sizeof(*s))) == NULL)
        return NULL;
    s->lag = lag;
    s->taps = taps;
    s->solve_every = solve_every > 0 ? solve_every : 4800;
    s->cur_y = calloc(BLOCK, sizeof(int16_t));
    s->bx = calloc(T + BLOCK, sizeof(int16_t));
    s->R = calloc(T*T, sizeof(double));
    s->q = calloc(T, sizeof(double));
    s->h = calloc(T, sizeof(double));
    s->Rw = calloc(T*T, sizeof(double));
    s->qw = calloc(T, sizeof(double));
    s->hw = calloc(T, sizeof(double));
    s->chol = calloc(T*T, sizeof(double));
    pthread_mutex_init(&s->mtx, NULL);
    pthread_cond_init(&s->cond, NULL);
    if (!s->cur_y || !s->bx || !s->R || !s->q || !s->h
        || !s->Rw || !s->qw || !s->hw || !s->chol) {
        near_echo_free(s);
        return NULL;
    }
    s->install_at = -1;
    return s;
}

int near_echo_set_threaded(near_echo_t *s, int on)
{
    if (s == NULL  ||  s->threaded  ||  !on)
        return s ? s->threaded : 0;
    if (pthread_create(&s->worker, NULL, worker_main, s) == 0)
        s->threaded = 1;
    return s->threaded;
}

void near_echo_free(near_echo_t *s)
{
    if (s == NULL)
        return;
    if (s->threaded) {
        pthread_mutex_lock(&s->mtx);
        s->quit = 1;
        pthread_cond_signal(&s->cond);
        pthread_mutex_unlock(&s->mtx);
        pthread_join(s->worker, NULL);
    }
    pthread_mutex_destroy(&s->mtx);
    pthread_cond_destroy(&s->cond);
    free(s->cur_y); free(s->bx); free(s->R); free(s->q); free(s->h);
    free(s->Rw); free(s->qw); free(s->hw); free(s->chol);
    free(s);
}

void near_echo_tx(near_echo_t *s, const int16_t *tx, int n)
{
    uint64_t c;

    if (s == NULL)
        return;
    c = s->tx_count;
    for (int i = 0; i < n; i++)
        s->ring[(c + (uint64_t) i) & (RING - 1)] = tx[i];
    __atomic_store_n(&s->tx_count, c + (uint64_t) n, __ATOMIC_RELEASE);
}

/*
 * The just-completed block [m0, m0+BLOCK) is echo-only: add it to the normal
 * equations, R(i,j) = sum_m x[m-lag-i] x[m-lag-j] and q(j) = sum_m y[m]
 * x[m-lag-j], by the first row and the sliding recursion
 *   R(i,j) = R(i-1,j-1) + x[m0-lag-i] x[m0-lag-j] - x[m1-lag-i] x[m1-lag-j].
 */
static void accept_block(near_echo_t *s, int64_t m0)
{
    int T = s->taps;
    int64_t m1 = m0 + BLOCK, base = m0 - s->lag - (T - 1);
    int16_t *x = s->bx;             /* x[k] = tx[base + k], k < T + BLOCK - 1 */

    for (int k = 0; k < T + BLOCK - 1; k++)
        x[k] = (int16_t) tx_at(s, base + k);
#define X(m) ((double) x[(m) - base])
    for (int j = 0; j < T; j++) {
        double acc = 0.0, accq = 0.0;

        for (int64_t m = m0; m < m1; m++) {
            double xj = X(m - s->lag - j);

            acc += X(m - s->lag)*xj;
            accq += s->cur_y[m - m0]*xj;
        }
        s->q[j] += accq;
        /* Walk this diagonal from its first-row element. */
        for (int i = 0; i + j < T; i++) {
            int jj = i + j;

            if (i > 0) {
                int64_t a = m0 - s->lag, b = m1 - s->lag;

                acc += X(a - i)*X(a - jj) - X(b - i)*X(b - jj);
            }
            s->R[i*T + jj] += acc;
            if (jj != i)
                s->R[jj*T + i] += acc;
        }
    }
#undef X
    s->blocks_used++;
    s->fresh = 1;
}

static void install(near_echo_t *s, const double *h)
{
    memcpy(s->h, h, (size_t) s->taps*sizeof(double));
    s->active = 1;
    s->solved++;
}

/* Snapshot the normal equations and start a solve; it lands at install_at. */
static void request_solve(near_echo_t *s)
{
    size_t T = (size_t) s->taps;

    if (s->blocks_used < MIN_BLOCKS  ||  !s->fresh  ||  s->install_at >= 0)
        return;
    s->fresh = 0;
    s->install_at = (int64_t) s->rx_count + s->solve_every;
    pthread_mutex_lock(&s->mtx);
    memcpy(s->Rw, s->R, T*T*sizeof(double));
    memcpy(s->qw, s->q, T*sizeof(double));
    s->ready = 0;
    if (s->threaded) {
        s->busy = 1;
        pthread_cond_signal(&s->cond);
        pthread_mutex_unlock(&s->mtx);
        return;
    }
    pthread_mutex_unlock(&s->mtx);
    /* Inline: solve now, install at the same instant a worker's would. */
    s->ready = cholesky_solve(s->Rw, s->qw, s->taps, s->chol, s->hw);
}

/* At install_at exactly: wait for the worker if it is somehow still busy. */
static void install_pending(near_echo_t *s)
{
    int ok;

    pthread_mutex_lock(&s->mtx);
    while (s->threaded  &&  s->busy)
        pthread_cond_wait(&s->cond, &s->mtx);
    ok = s->ready;
    s->ready = 0;
    pthread_mutex_unlock(&s->mtx);
    if (ok)
        install(s, s->hw);
    s->install_at = -1;
}

void near_echo_rx(near_echo_t *s, int16_t *rx, int n)
{
    if (s == NULL)
        return;
    for (int i = 0; i < n; i++) {
        if (s->install_at >= 0  &&  (int64_t) s->rx_count == s->install_at)
            install_pending(s);
        int64_t m = (int64_t) s->rx_count;
        double y = rx[i], est = 0.0, out;
        double xm = tx_at(s, m - s->lag - s->taps/2);

        s->cur_y[s->blk_n] = rx[i];
        s->blk_rx += y*y;
        s->blk_tx += xm*xm;
        if (s->active)
            for (int j = 0; j < s->taps; j++)
                est += s->h[j]*tx_at(s, m - s->lag - j);
        out = y - est;
        if (out > 32767.0) out = 32767.0;
        if (out < -32768.0) out = -32768.0;
        rx[i] = (int16_t) lrint(out);
        s->pow_in += RMS_ALPHA*(y*y - s->pow_in);
        s->pow_echo += RMS_ALPHA*(est*est - s->pow_echo);
        s->rx_count++;
        if (++s->blk_n >= BLOCK) {
            if (s->blk_tx >= BLOCK*BLOCK_TX_MIN  &&  s->blk_rx <= ECHO_GATE*s->blk_tx)
                accept_block(s, (int64_t) s->rx_count - BLOCK);
            s->blk_rx = s->blk_tx = 0.0;
            s->blk_n = 0;
        }
        if (++s->since_solve >= s->solve_every) {
            s->since_solve = 0;
            request_solve(s);
        }
    }
}

int near_echo_solved(const near_echo_t *s) { return s ? s->solved : 0; }
int near_echo_active(const near_echo_t *s) { return s ? s->active : 0; }
long near_echo_blocks_used(const near_echo_t *s) { return s ? s->blocks_used : 0; }
int near_echo_lag(const near_echo_t *s) { return s ? s->lag : 0; }
double near_echo_rms_in(const near_echo_t *s) { return s ? sqrt(s->pow_in) : 0.0; }
double near_echo_rms_echo(const near_echo_t *s) { return s ? sqrt(s->pow_echo) : 0.0; }
