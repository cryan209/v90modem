/*
 * fdm_bank.c -- SSB channel bank: 8 kHz line signals in and out of 4 kHz
 * slots of a wideband real stream.  See fdm_bank.h for the arithmetic.
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "fdm_bank.h"

static double bessel_i0(double x)
{
    double sum = 1.0, term = 1.0;

    for (int k = 1; k < 64; k++) {
        term *= (x/(2.0*k))*(x/(2.0*k));
        sum += term;
        if (term < sum*1e-12)
            break;
    }
    return sum;
}

int fdm_bank_init(fdm_bank_t *b, int fs, int taps_per_8k, double beta)
{
    double fc, mid, i0b, sum = 0.0;

    memset(b, 0, sizeof(*b));
    if (fs < 8000 || fs % 8000)
        return -1;
    b->fs = fs;
    b->l = fs/8000;
    b->p = taps_per_8k < 8 ? 8 : taps_per_8k;
    b->ntaps = b->l*b->p;

    /* Passband to ~1880 Hz, adjacent slot's roll-off edge at ~2040 Hz:
       centre the cutoff in that gap. */
    fc = 1960.0/b->fs;
    mid = (b->ntaps - 1)/2.0;
    i0b = bessel_i0(beta);
    b->h = calloc((size_t)b->ntaps, sizeof(float));
    b->cos_t = malloc((size_t)b->fs*sizeof(float));
    b->sin_t = malloc((size_t)b->fs*sizeof(float));
    if (!b->h || !b->cos_t || !b->sin_t) {
        fdm_bank_free(b);
        return -1;
    }
    for (int i = 0; i < b->ntaps; i++) {
        double t = i - mid;
        double sinc = (t == 0.0) ? 2.0*fc : sin(2.0*M_PI*fc*t)/(M_PI*t);
        double r = t/mid;
        double w = bessel_i0(beta*sqrt(fmax(0.0, 1.0 - r*r)))/i0b;

        b->h[i] = (float)(sinc*w);
        sum += b->h[i];
    }
    for (int i = 0; i < b->ntaps; i++)
        b->h[i] = (float)(b->h[i]/sum);
    for (int i = 0; i < b->fs; i++) {
        b->cos_t[i] = (float)cos(2.0*M_PI*i/b->fs);
        b->sin_t[i] = (float)sin(2.0*M_PI*i/b->fs);
    }
    return 0;
}

void fdm_bank_free(fdm_bank_t *b)
{
    free(b->h);
    free(b->cos_t);
    free(b->sin_t);
    b->h = b->cos_t = b->sin_t = NULL;
}

int fdm_bank_slots(const fdm_bank_t *b)
{
    return b->fs/(2*FDM_SLOT_HZ);
}

int fdm_slot_tx_init(fdm_slot_tx_t *t, const fdm_bank_t *b, int slot)
{
    t->b = b;
    t->carrier_hz = slot*FDM_SLOT_HZ + FDM_SHIFT_HZ;
    t->phase = 0;
    t->n8 = 0;
    t->hist = calloc((size_t)b->p, sizeof(fdm_cf_t));
    return t->hist ? 0 : -1;
}

int fdm_slot_rx_init_rate(fdm_slot_rx_t *r, const fdm_bank_t *b, int slot, int out_rate)
{
    memset(r, 0, sizeof(*r));
    if (out_rate != 8000 && out_rate != 16000)
        return -1;
    if (b->fs % out_rate)
        return -1;
    r->b = b;
    r->carrier_hz = slot*FDM_SLOT_HZ + FDM_SHIFT_HZ;
    r->dec = b->fs/out_rate;
    if (out_rate == 8000) {
        /* e^{+j pi n/2}, exactly */
        static const float qr[4] = { 1.0f, 0.0f, -1.0f, 0.0f };
        static const float qi[4] = { 0.0f, 1.0f, 0.0f, -1.0f };

        r->nrot = 4;
        memcpy(r->rot_re, qr, sizeof(qr));
        memcpy(r->rot_im, qi, sizeof(qi));
    } else {
        /* e^{+j pi n/4}: the same 2000 Hz at 16 kHz */
        r->nrot = 8;
        for (int i = 0; i < 8; i++) {
            r->rot_re[i] = (float)cos(M_PI*i/4.0);
            r->rot_im[i] = (float)sin(M_PI*i/4.0);
        }
    }
    r->ring = calloc((size_t)2*b->ntaps, sizeof(fdm_cf_t));
    return r->ring ? 0 : -1;
}

int fdm_slot_rx_init(fdm_slot_rx_t *r, const fdm_bank_t *b, int slot)
{
    return fdm_slot_rx_init_rate(r, b, slot, 8000);
}

void fdm_slot_tx_free(fdm_slot_tx_t *t)
{
    free(t->hist);
    t->hist = NULL;
}

void fdm_slot_rx_free(fdm_slot_rx_t *r)
{
    free(r->ring);
    r->ring = NULL;
}

void fdm_slot_tx_run(fdm_slot_tx_t *t, const float *x, int n, float *out)
{
    /* e^{-j pi n/2}: 1, -j, -1, +j */
    static const float qr[4] = { 1.0f, 0.0f, -1.0f, 0.0f };
    static const float qi[4] = { 0.0f, -1.0f, 0.0f, 1.0f };
    const fdm_bank_t *b = t->b;

    for (int i = 0; i < n; i++) {
        int q = t->n8++ & 3;

        memmove(t->hist + 1, t->hist, (size_t)(b->p - 1)*sizeof(fdm_cf_t));
        t->hist[0].re = x[i]*qr[q];
        t->hist[0].im = x[i]*qi[q];
        for (int ph = 0; ph < b->l; ph++) {
            float ur = 0.0f, ui = 0.0f;
            const float *h = b->h + ph;

            for (int k = 0; k < b->p; k++) {
                ur += h[k*b->l]*t->hist[k].re;
                ui += h[k*b->l]*t->hist[k].im;
            }
            /* gain L restores the interpolated amplitude; x2 for the real
               part of a one-sided signal */
            ur *= (float)b->l;
            ui *= (float)b->l;
            *out++ += 2.0f*(ur*b->cos_t[t->phase] - ui*b->sin_t[t->phase]);
            t->phase += t->carrier_hz;
            if (t->phase >= b->fs)
                t->phase -= b->fs;
        }
    }
}

int fdm_slot_rx_run(fdm_slot_rx_t *r, const float *w, int n, float *x)
{
    const fdm_bank_t *b = r->b;
    int produced = 0;

    for (int i = 0; i < n; i++) {
        fdm_cf_t s;

        s.re = w[i]*b->cos_t[r->phase];
        s.im = -w[i]*b->sin_t[r->phase];
        r->phase += r->carrier_hz;
        if (r->phase >= b->fs)
            r->phase -= b->fs;
        r->ring[r->pos] = s;
        r->ring[r->pos + b->ntaps] = s;
        if (++r->pos >= b->ntaps)
            r->pos = 0;
        if (++r->fill < r->dec)
            continue;
        r->fill = 0;
        {
            /* ring[pos .. pos+ntaps-1] is oldest..newest */
            const fdm_cf_t *c = r->ring + r->pos;
            float vr = 0.0f, vi = 0.0f;
            int q = r->n8++ % r->nrot;

            for (int k = 0; k < b->ntaps; k++) {
                float hk = b->h[b->ntaps - 1 - k];
                vr += hk*c[k].re;
                vi += hk*c[k].im;
            }
            x[produced++] = 2.0f*(vr*r->rot_re[q] - vi*r->rot_im[q]);
        }
    }
    return produced;
}
