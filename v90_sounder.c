/*
 * v90_sounder.c — see v90_sounder.h.
 */

#include <math.h>
#include <string.h>

#include <spandsp.h>

#include "v90_sounder.h"

static const int tones[] = {
     150,  300,  450,  600,  750, 1050, 1350, 1500, 1650, 1950,
    2100, 2250, 2550, 2700, 2850, 3000, 3150, 3300, 3450, 3600,
    3750, 3800, 3850, 3900, 3950
};
static const int empty_bins[] = {900, 1200, 1800, 2400};

/*
 * Transmit level.
 *
 * §8.5.2's Table 15 limit on this path works out at about 3588 RMS (-13 dBm0),
 * and a sounder has no reason to sit above what the modem itself may transmit.
 * Spread over 25 tones that is about -27 dBm0 each, which is far enough above
 * µ-law's quantisation noise for the weak end of the band to be a measurement
 * rather than a floor reading.
 */
#define TARGET_RMS      3500.0
/*
 * ...but the level has to be settable, because the constraint that binds is at
 * the FAR end.  Measured on the HSF coupler's path, a sounding at the default
 * arrives clipping 23% of its samples, and a clipped receive is exactly the
 * failure this signal is least able to survive: the flat tops put energy into
 * the empty bins, so the noise floor reads high and every weak tone at the top
 * of the band -- the ones the sounding exists to measure -- is dismissed.  The
 * measurement is two-port (RX against a recording of TX), so dropping the level
 * costs nothing: it is divided out.
 */
static double sounder_rms(void)
{
    const char *v = getenv("ME_SOUNDER_RMS");
    double rms = TARGET_RMS;

    if (v != NULL  &&  v[0] != '\0') {
        double parsed = atof(v);

        if (parsed >= 50.0  &&  parsed <= 8000.0)
            rms = parsed;
    }
    return rms;
}

const int *v90_sounder_tones(int *count)
{
    if (count)
        *count = (int) (sizeof(tones)/sizeof(tones[0]));
    return tones;
}

const int *v90_sounder_empty_bins(int *count)
{
    if (count)
        *count = (int) (sizeof(empty_bins)/sizeof(empty_bins[0]));
    return empty_bins;
}

void v90_sounder_block_linear(int16_t *out)
{
    static double block[V90_SOUNDER_BLOCK];
    static bool built = false;
    int n = (int) (sizeof(tones)/sizeof(tones[0]));

    if (!built) {
        double sum_sq = 0.0;
        double scale;

        for (int i = 0; i < V90_SOUNDER_BLOCK; i++) {
            double acc = 0.0;

            for (int k = 0; k < n; k++) {
                /*
                 * Schroeder phases.  Twenty-five cosines all starting at zero
                 * peak together at over five times their RMS, and a peak that
                 * clips is not a measurement of anything -- the clipping puts
                 * energy in exactly the empty bins the floor is read from.
                 * pi*k^2/n spreads them and brings the crest factor to under
                 * two, which the self-check asserts.
                 */
                double phase = M_PI*k*k/(double) n;

                acc += cos(2.0*M_PI*tones[k]*i/8000.0 + phase);
            }
            block[i] = acc;
            sum_sq += acc*acc;
        }
        scale = sounder_rms()/sqrt(sum_sq/V90_SOUNDER_BLOCK);
        for (int i = 0; i < V90_SOUNDER_BLOCK; i++)
            block[i] *= scale;
        built = true;
    }
    /*endif*/
    for (int i = 0; i < V90_SOUNDER_BLOCK; i++) {
        double v = block[i];

        if (v > 32000.0)
            v = 32000.0;
        else if (v < -32000.0)
            v = -32000.0;
        out[i] = (int16_t) lrint(v);
    }
}

void v90_sounder_fill(v90_law_t law, uint8_t *codewords, int count, int *phase)
{
    int16_t block[V90_SOUNDER_BLOCK];
    int p = (phase != NULL) ? *phase : 0;

    if (codewords == NULL  ||  count <= 0)
        return;
    v90_sounder_block_linear(block);
    for (int i = 0; i < count; i++) {
        int16_t v = block[p];

        codewords[i] = (law == V90_LAW_ALAW) ? linear_to_alaw(v)
                                             : linear_to_ulaw(v);
        if (++p >= V90_SOUNDER_BLOCK)
            p = 0;
    }
    if (phase != NULL)
        *phase = p;
}
