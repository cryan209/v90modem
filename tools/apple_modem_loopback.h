/* On-hook DAA transport loopback, Si3056 §5.30; analogue front end of V.90 §5.
 * Test-only PRBS and analysis. No modem training coefficients or samples change.
 * A high correlation alone is not a digital PASS: every compared sample must
 * match and a substantial non-silent PRBS interval must have been received. */
#ifndef APPLE_MODEM_LOOPBACK_H
#define APPLE_MODEM_LOOPBACK_H
#include <stdint.h>
#include <math.h>
#include <string.h>
struct apple_loop_result {
    int found, lag, frames;
    unsigned mismatches, bit_errors;
    double correlation, gain, dc, rmse;
};
static inline void apple_loop_pattern(int16_t *out, unsigned count, unsigned rate)
{
    uint32_t state = 0x6d56a108u;
    for (unsigned i = 0; i < count; i++) {
        state ^= state << 13; state ^= state >> 17; state ^= state << 5;
        out[i] = i >= rate && i < rate*5/2 ? (int16_t)((state & 16383)-8192) : 0;
    }
}
static inline struct apple_loop_result apple_loop_analyze(const int16_t *tx, unsigned nt,
                                                         const int16_t *rx, unsigned nr,
                                                         unsigned rate)
{
    struct apple_loop_result r = {0};
    /* One second of silent HAL startup precedes the measured PRBS. */
    const unsigned anchor = rate*5/4, window = 512;
    if (rate < 1000 || nt < anchor+window || nr < window) return r;
    double sx = 0, xx = 0, best = 0;
    for (unsigned j = 0; j < window; j++) { sx += tx[anchor+j]; xx += (double)tx[anchor+j]*tx[anchor+j]; }
    xx -= sx*sx/window;
    if (xx < window*100.0) return r;
    for (unsigned pos = 0; pos+window <= nr && pos <= anchor+rate; pos++) {
        double sy = 0, yy = 0, xy = 0;
        for (unsigned j = 0; j < window; j++) {
            double x = tx[anchor+j], y = rx[pos+j]; sy += y; yy += y*y; xy += x*y;
        }
        yy -= sy*sy/window; xy -= sx*sy/window;
        if (yy <= window*100.0) continue;
        double corr = xy/sqrt(xx*yy);
        if (fabs(corr) > best) { best = fabs(corr); r.correlation = corr; r.lag = (int)pos-(int)anchor; r.found = 1; }
    }
    if (!r.found) return r;
    unsigned begin = rate*5/4, end = rate*9/4;
    if (end > nt) end = nt;
    if ((int)begin+r.lag < 0) begin = (unsigned)-r.lag;
    if ((int)end+r.lag > (int)nr) end = nr-r.lag;
    if (end <= begin || end-begin < rate/2) { r.found = 0; return r; }
    double sy = 0, yy = 0, xy = 0, error = 0; sx = 0; xx = 0;
    for (unsigned i = begin; i < end; i++) {
        int16_t x = tx[i], y = rx[(int)i+r.lag];
        uint16_t bits = (uint16_t)x ^ (uint16_t)y;
        r.mismatches += x != y;
        while (bits) { r.bit_errors += bits & 1; bits >>= 1; }
        sx += x; sy += y; xx += (double)x*x; yy += (double)y*y; xy += (double)x*y;
        error += (double)(x-y)*(x-y);
    }
    r.frames = (int)(end-begin); xx -= sx*sx/r.frames; yy -= sy*sy/r.frames; xy -= sx*sy/r.frames;
    if (xx <= 0 || yy <= 0) { r.found = 0; return r; }
    r.gain = xy/xx; r.dc = (sy-r.gain*sx)/r.frames;
    r.correlation = xy/sqrt(xx*yy); r.rmse = sqrt(error/r.frames);
    return r;
}
#endif
