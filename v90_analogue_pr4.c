/*
 * v90_analogue_pr4.c — see v90_analogue_pr4.h.
 */

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "v90_analogue_pr4.h"

#define MAX_STATES  260         /* 128 Ucodes x 2 signs, plus room */
#define HIST        64          /* decision history kept for training targets */
#define OUT_RING    1024        /* reorder buffer, symbols */
#define TRACK_MIN_SET 8         /* candidates (levels x signs) a step needs to adapt on */

typedef struct {
    int      n;
    double   lv[MAX_STATES];
    uint8_t  cw[MAX_STATES];
    int16_t  bp[MAX_STATES];    /* index into the previous step, -1 at the start */
    int64_t  index;             /* absolute symbol index of this step */
    double   z;                 /* the equaliser's output for it */
    double   dec_lv;            /* once decided */
    uint8_t  dec_cw;
} step_t;

typedef struct {
    step_t  *steps;             /* ring of depth + 1 steps */
    int      count;             /* steps taken */
    double   pm[MAX_STATES];
    int64_t  last_index;        /* the last decided step on this interleave */
    double   last_lv;
} interleave_t;

struct v90a_pr4_s {
    int      taps, dmax, depth;
    double  *x;                 /* T/2 delay line, x[0] newest */
    int      filled;
    int64_t  sym;               /* symbol calls so far */
    double   dh[HIST];          /* decisions, by symbol index mod HIST */

    /* training */
    int      train_left;
    double  *R, *q, *e;         /* R: taps x taps; q: (dmax+1) x taps; e: dmax+1 */
    long     train_n;
    double  *w;
    int      delay;
    bool     trained;
    double   snr_db;

    /* detection */
    bool     running;
    int64_t  first;
    v90a_pr4_set_fn set_fn;
    void    *user;
    interleave_t il[2];
    /* decision-directed tracking: the input each output was made from */
    double   mu;
    double  *xr;                /* XR(depth) rows of `taps` */
    int      xr_rows;
    long     updates;
    uint8_t  out_cw[OUT_RING];
    bool     out_ok[OUT_RING];
    int64_t  next_out;
};

v90a_pr4_t *v90a_pr4_init(int taps, int dmax, int depth)
{
    v90a_pr4_t *s;

    if (taps < 4  ||  taps > 512  ||  dmax < 0  ||  dmax > HIST - 4
        ||  depth < 1  ||  depth > 256)
        return NULL;
    if ((s = calloc(1, sizeof(*s))) == NULL)
        return NULL;
    s->taps = taps;
    s->dmax = dmax;
    s->depth = depth;
    s->x = calloc((size_t) taps, sizeof(double));
    s->w = calloc((size_t) taps, sizeof(double));
    s->il[0].steps = calloc((size_t) depth + 1, sizeof(step_t));
    s->il[1].steps = calloc((size_t) depth + 1, sizeof(step_t));
    s->xr_rows = 2*depth + 8;
    s->xr = calloc((size_t) s->xr_rows*(size_t) taps, sizeof(double));
    if (!s->x || !s->w || !s->il[0].steps || !s->il[1].steps || !s->xr) {
        v90a_pr4_free(s);
        return NULL;
    }
    return s;
}

void v90a_pr4_free(v90a_pr4_t *s)
{
    if (s == NULL)
        return;
    free(s->x); free(s->w); free(s->xr); free(s->R); free(s->q); free(s->e);
    free(s->il[0].steps); free(s->il[1].steps);
    free(s);
}

void v90a_pr4_sample(v90a_pr4_t *s, int16_t sample)
{
    if (s == NULL)
        return;
    memmove(s->x + 1, s->x, sizeof(double)*(size_t) (s->taps - 1));
    /* Scaled to keep the normal equations' entries near unity; the fit does
     * not care about the scale. */
    s->x[0] = sample/1000.0;
    if (s->filled < s->taps)
        s->filled++;
}

bool v90a_pr4_train(v90a_pr4_t *s, int symbols)
{
    size_t T;

    if (s == NULL  ||  symbols <= 0)
        return false;
    T = (size_t) s->taps;
    free(s->R); free(s->q); free(s->e);
    s->R = calloc(T*T, sizeof(double));
    s->q = calloc((size_t) (s->dmax + 1)*T, sizeof(double));
    s->e = calloc((size_t) s->dmax + 1, sizeof(double));
    if (!s->R || !s->q || !s->e)
        return false;
    s->train_left = symbols;
    s->train_n = 0;
    s->trained = false;
    return true;
}

bool v90a_pr4_trained(const v90a_pr4_t *s) { return s && s->trained; }
int64_t v90a_pr4_symbols(const v90a_pr4_t *s) { return s ? s->sym : 0; }
int v90a_pr4_delay(const v90a_pr4_t *s) { return s ? s->delay : 0; }
double v90a_pr4_train_snr_db(const v90a_pr4_t *s) { return s ? s->snr_db : 0.0; }
bool v90a_pr4_running(const v90a_pr4_t *s) { return s && s->running; }
int64_t v90a_pr4_next_index(const v90a_pr4_t *s) { return s ? s->next_out : 0; }
void v90a_pr4_set_tracking(v90a_pr4_t *s, double mu) { if (s) s->mu = (mu > 0.0) ? mu : 0.0; }
long v90a_pr4_updates(const v90a_pr4_t *s) { return s ? s->updates : 0; }

static double dec_at(const v90a_pr4_t *s, int64_t k)
{
    return (k < 0) ? 0.0 : s->dh[k % HIST];
}

/* Cholesky of R (lightly loaded per element) once, then one solve per delay;
 * keep the delay whose residual is smallest. */
static void train_finish(v90a_pr4_t *s)
{
    int T = s->taps;
    double *L = calloc((size_t) T*T, sizeof(double)), *w = calloc((size_t) T, sizeof(double));
    double best = HUGE_VAL;

    if (L == NULL  ||  w == NULL)
        goto out;
    for (int i = 0; i < T; i++) {
        for (int j = 0; j <= i; j++) {
            double v = s->R[i*T + j]*(i == j ? 1.0 + 1e-6 : 1.0) + (i == j ? 1e-12 : 0.0);

            for (int k = 0; k < j; k++)
                v -= L[i*T + k]*L[j*T + k];
            if (i == j) {
                if (v <= 0.0)
                    goto out;
                L[i*T + i] = sqrt(v);
            } else {
                L[i*T + j] = v/L[j*T + j];
            }
        }
    }
    for (int d = 0; d <= s->dmax; d++) {
        const double *q = s->q + (size_t) d*T;
        double res;

        for (int i = 0; i < T; i++) {
            double v = q[i];

            for (int k = 0; k < i; k++)
                v -= L[i*T + k]*w[k];
            w[i] = v/L[i*T + i];
        }
        for (int i = T - 1; i >= 0; i--) {
            double v = w[i];

            for (int k = i + 1; k < T; k++)
                v -= L[k*T + i]*w[k];
            w[i] = v/L[i*T + i];
        }
        res = s->e[d];
        for (int i = 0; i < T; i++)
            res -= q[i]*w[i];
        if (res < best) {
            best = res;
            s->delay = d;
            memcpy(s->w, w, sizeof(double)*(size_t) T);
            s->snr_db = 10.0*log10(s->e[d]/(res > 1e-30 ? res : 1e-30));
        }
    }
    s->trained = (best < HUGE_VAL);
out:
    free(L);
    free(w);
    free(s->R); free(s->q); free(s->e);
    s->R = s->q = s->e = NULL;
}

/* ---- Viterbi ------------------------------------------------------------ */

static void emit(v90a_pr4_t *s, int64_t index, uint8_t cw)
{
    if (index < s->first  ||  index >= s->next_out + OUT_RING)
        return;
    s->out_cw[index % OUT_RING] = cw;
    s->out_ok[index % OUT_RING] = true;
}

/* Fill `cur` for its index (querying the set) and update il->pm from `prev`
 * (NULL: an unknown start). */
static void trellis_step(v90a_pr4_t *s, interleave_t *il, step_t *cur, const step_t *prev)
{
    double npm[MAX_STATES];
    int n;

    n = s->set_fn(s->user, cur->index, cur->lv, cur->cw, MAX_STATES);
    if (n <= 0) {
        if (prev == NULL) {
            n = 1;
            cur->lv[0] = 0.0;
            cur->cw[0] = 0;
        } else {
            /* Nothing known: keep the previous step's candidates. */
            n = prev->n;
            memcpy(cur->lv, prev->lv, sizeof(double)*(size_t) n);
            memcpy(cur->cw, prev->cw, (size_t) n);
        }
    }
    cur->n = n;
    if (prev == NULL) {
        for (int c = 0; c < n; c++) {
            npm[c] = 0.0;
            cur->bp[c] = -1;
        }
    } else {
        double lo = HUGE_VAL;

        for (int c = 0; c < n; c++) {
            double best = HUGE_VAL;
            int arg = 0;

            for (int p = 0; p < prev->n; p++) {
                double r = cur->z - (cur->lv[c] - prev->lv[p]);
                double m = il->pm[p] + r*r;

                if (m < best) {
                    best = m;
                    arg = p;
                }
            }
            npm[c] = best;
            cur->bp[c] = (int16_t) arg;
            if (best < lo)
                lo = best;
        }
        for (int c = 0; c < n; c++)
            npm[c] -= lo;
    }
    memcpy(il->pm, npm, sizeof(double)*(size_t) n);
}

/*
 * Decision-directed NLMS on the traced-back decision: the target for the
 * output z[k] is s[k] - s[k-2], the decided symbol less the one before it on
 * the same interleave.  Training on TRN1d fixes the taps once, but the call
 * goes on for minutes: held, the codeword error rate on Apple rig v90-rr-3
 * climbed from 0 at B1d to 5-9% twenty seconds later, which is what lost a
 * renegotiation's two-frame Ed.  The decision is the Viterbi's, `depth`
 * symbols late, so it is far more reliable than a slicer's.
 */
static void track(v90a_pr4_t *s, const step_t *dp, const interleave_t *il)
{
    const double *x;
    double e, norm = 0.0;

    /* Only on a real constellation: §8.6.4's R signals are one level with a
     * sign pattern, which any taps that reproduce a period-6 square wave
     * fit -- adapting on Ri took the trained 41 dB to 24 within a second. */
    if (il->last_index != dp->index - 2  ||  dp->n < TRACK_MIN_SET)
        return;
    if (dp->index < 0  ||  s->sym - dp->index - s->delay > s->xr_rows - 1)
        return;
    x = s->xr + (size_t) (dp->index % s->xr_rows)*(size_t) s->taps;
    e = (dp->dec_lv - il->last_lv) - dp->z;
    for (int i = 0; i < s->taps; i++)
        norm += x[i]*x[i];
    if (norm <= 1e-12)
        return;
    for (int i = 0; i < s->taps; i++)
        s->w[i] += s->mu*e*x[i]/norm;
    s->updates++;
}

static void viterbi_step(v90a_pr4_t *s, int64_t index, double z)
{
    interleave_t *il = &s->il[index & 1];
    step_t *prev = (il->count > 0) ? &il->steps[(il->count - 1) % (s->depth + 1)] : NULL;
    step_t *cur = &il->steps[il->count % (s->depth + 1)];

    cur->index = index;
    cur->z = z;
    trellis_step(s, il, cur, prev);
    il->count++;
    /* Decide the step `depth` back along the best survivor. */
    if (il->count > s->depth) {
        int st = 0;
        double bm = HUGE_VAL;
        int t = il->count - 1;

        for (int c = 0; c < cur->n; c++)
            if (il->pm[c] < bm)
                bm = il->pm[c], st = c;
        for (int u = 0; u < s->depth; u++, t--) {
            step_t *sp = &il->steps[t % (s->depth + 1)];

            st = sp->bp[st];
            if (st < 0)
                return;
        }
        {
            step_t *dp = &il->steps[t % (s->depth + 1)];

            dp->dec_lv = dp->lv[st];
            dp->dec_cw = dp->cw[st];
            emit(s, dp->index, dp->cw[st]);
            if (s->mu > 0.0)
                track(s, dp, il);
            il->last_index = dp->index;
            il->last_lv = dp->dec_lv;
        }
    }
}

/*
 * Re-decide everything not yet decided, with freshly queried candidate sets.
 * The steps still inside the traceback hold their equaliser outputs, so when
 * the caller learns something about the symbols ahead -- §8.6.4's R̄i found,
 * so TRN2d's frame grid is known; Ed passed, so CP replaces CPt -- the trellis
 * is rebuilt from the last decided symbol as a known state.
 */
void v90a_pr4_resync(v90a_pr4_t *s)
{
    if (s == NULL  ||  !s->running)
        return;
    for (int i = 0; i < 2; i++) {
        interleave_t *il = &s->il[i];
        int c = il->count, t0 = (c > s->depth) ? c - s->depth : 0;
        step_t *prev = NULL;

        if (c == 0)
            continue;
        if (t0 > 0) {
            prev = &il->steps[(t0 - 1) % (s->depth + 1)];
            prev->n = 1;
            prev->lv[0] = prev->dec_lv;
            prev->cw[0] = prev->dec_cw;
            prev->bp[0] = -1;
            il->pm[0] = 0.0;
        }
        for (int t = t0; t < c; t++) {
            step_t *cur = &il->steps[t % (s->depth + 1)];

            trellis_step(s, il, cur, prev);
            prev = cur;
        }
    }
}

bool v90a_pr4_start(v90a_pr4_t *s, int64_t first,
                    v90a_pr4_set_fn set_fn, void *user)
{
    if (s == NULL  ||  !s->trained  ||  set_fn == NULL)
        return false;
    s->running = true;
    s->first = first;
    s->next_out = first;
    s->set_fn = set_fn;
    s->user = user;
    s->il[0].count = s->il[1].count = 0;
    s->il[0].last_index = s->il[1].last_index = -1;
    memset(s->out_ok, 0, sizeof(s->out_ok));
    return true;
}

void v90a_pr4_symbol(v90a_pr4_t *s, double decision)
{
    int64_t k;

    if (s == NULL)
        return;
    k = s->sym++;
    s->dh[k % HIST] = decision;
    if (s->filled < s->taps)
        return;
    if (s->train_left > 0) {
        int T = s->taps;

        for (int i = 0; i < T; i++) {
            double xi = s->x[i];
            double *row = s->R + (size_t) i*T;

            for (int j = 0; j <= i; j++)
                row[j] += xi*s->x[j];
        }
        for (int d = 0; d <= s->dmax; d++) {
            double t = dec_at(s, k - d) - dec_at(s, k - d - 2);
            double *q = s->q + (size_t) d*T;

            for (int i = 0; i < T; i++)
                q[i] += s->x[i]*t;
            s->e[d] += t*t;
        }
        s->train_n++;
        if (--s->train_left == 0)
            train_finish(s);
        return;
    }
    if (s->running) {
        double z = 0.0;

        for (int i = 0; i < s->taps; i++)
            z += s->w[i]*s->x[i];
        if (k - s->delay >= 0)
            memcpy(s->xr + (size_t) ((k - s->delay) % s->xr_rows)*(size_t) s->taps,
                   s->x, sizeof(double)*(size_t) s->taps);
        viterbi_step(s, k - s->delay, z);
    }
}

int v90a_pr4_take(v90a_pr4_t *s, uint8_t *codewords, int max)
{
    int n = 0;

    if (s == NULL  ||  codewords == NULL)
        return 0;
    while (n < max  &&  s->out_ok[s->next_out % OUT_RING]) {
        s->out_ok[s->next_out % OUT_RING] = false;
        codewords[n++] = s->out_cw[s->next_out % OUT_RING];
        s->next_out++;
    }
    return n;
}
