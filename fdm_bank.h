/*
 * fdm_bank.h -- the SSB channel bank that puts 8 kHz line signals into 4 kHz
 * slots of one wideband (48/96 kHz) real stream and takes them out again.
 *
 * Slot k occupies [4000k, 4000k + 4000) Hz.  An 8 kHz real line signal lives
 * in 0..4000 Hz, so the shift has to be single-sideband or the
 * negative-frequency image lands in the neighbouring slot:
 *
 *   TX  z[n] = x[n] e^{-j pi n/2}           (8 kHz; the wanted half now sits
 *                                            in -1960..+1880 Hz)
 *       u    = L-fold interpolation of z through a low-pass that keeps only
 *              that half (the other half sits 2120..5960 Hz, mod 8000)
 *       w[m] = 2 Re{ u[m] e^{j 2 pi (4000k + 2000) m / fs} }
 *   RX  v    = low-pass + L-fold decimation of w[m] e^{-j 2 pi (...) m / fs}
 *       x[n] = 2 Re{ v[n] e^{+j pi n/2} }
 *
 * which returns x exactly (to the filters) for a pure delay channel; a delay
 * also rotates the carrier, which a passband modem's carrier recovery
 * absorbs.  The prototype's cutoff (1960 Hz at baseband) means a slot passes
 * roughly 40..3880 Hz of the 8 kHz signal.  Adjacent slots are only ~160 Hz
 * apart at the roll-off edges, so the one Kaiser low-pass is long.
 *
 * Shared by v34_fdm_test (N V.34 pairs over one bank) and engine_pair_test
 * --fdm-slot (a whole-engine call carried through one slot).  docs/v34_fdm.md.
 */
#ifndef FDM_BANK_H
#define FDM_BANK_H

#define FDM_SLOT_HZ  4000
#define FDM_SHIFT_HZ 2000      /* e^{-j pi n/2} at 8 kHz */

typedef struct { float re, im; } fdm_cf_t;

typedef struct {
    int fs;             /* wideband rate, a multiple of 8000 */
    int l;              /* fs / 8000 */
    int p;              /* prototype taps per 8 kHz sample */
    int ntaps;          /* l * p */
    float *h;           /* low-pass prototype at fs, unity DC gain */
    float *cos_t;       /* cos(2 pi i / fs), i < fs */
    float *sin_t;
} fdm_bank_t;

/* One slot's modulator: 8 kHz real in, wideband real added into out[]. */
typedef struct {
    const fdm_bank_t *b;
    int carrier_hz;     /* 4000k + 2000 */
    int phase;          /* carrier phase index, mod fs */
    int n8;             /* 8 kHz sample counter, for e^{-j pi n/2} */
    fdm_cf_t *hist;     /* last p baseband samples, newest at [0] */
} fdm_slot_tx_t;

/* One slot's demodulator: wideband real in, 8 kHz (or 16 kHz) real out. */
typedef struct {
    const fdm_bank_t *b;
    int carrier_hz;
    int phase;
    int n8;             /* output sample counter, for the e^{+j 2 pi 2000 n/r} */
    int dec;            /* wideband samples per output sample */
    int nrot;           /* period of that rotation in output samples */
    float rot_re[8], rot_im[8];
    int fill;           /* wideband samples since the last output */
    int pos;            /* ring write position */
    fdm_cf_t *ring;     /* 2*ntaps, mirrored for contiguous reads */
} fdm_slot_rx_t;

/* fs a multiple of 8000; taps_per_8k >= 8 (default 96); Kaiser beta
 * (default 7).  Returns 0, or -1 on a bad rate or allocation failure. */
int fdm_bank_init(fdm_bank_t *b, int fs, int taps_per_8k, double beta);
void fdm_bank_free(fdm_bank_t *b);
/* Slots that fit below fs/2. */
int fdm_bank_slots(const fdm_bank_t *b);

int fdm_slot_tx_init(fdm_slot_tx_t *t, const fdm_bank_t *b, int slot);
int fdm_slot_rx_init(fdm_slot_rx_t *r, const fdm_bank_t *b, int slot);
/* The same at out_rate 8000 or 16000.  16000 is the T/2 stream a fractionally
 * spaced PCM receiver needs; it requires fs/16000 to be whole. */
int fdm_slot_rx_init_rate(fdm_slot_rx_t *r, const fdm_bank_t *b, int slot, int out_rate);
void fdm_slot_tx_free(fdm_slot_tx_t *t);
void fdm_slot_rx_free(fdm_slot_rx_t *r);

/* n 8 kHz samples in; n*l wideband samples ADDED into out[]. */
void fdm_slot_tx_run(fdm_slot_tx_t *t, const float *x, int n, float *out);
/* n wideband samples in; up to n/dec (+1) samples out; returns count. */
int fdm_slot_rx_run(fdm_slot_rx_t *r, const float *w, int n, float *x);

#endif
