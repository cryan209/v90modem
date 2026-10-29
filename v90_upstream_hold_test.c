/*
 * V.90 digital upstream continuity regression (V.90 8.5.2/9.4.2.5).
 *
 * Exercise the actual T/3 receiver, not an independent DSP model: B1 followed
 * by twenty seconds of PRBS through G.711, with a phase-continuous carrier
 * frequency change after acquisition. The carrier must traverse full turns
 * while NLMS adapts, or the wrong-coordinate gradient can escape a short test.
 * There is no LAPM to conceal bit errors. This is not hardware qualification.
 * The private TX field is used only to inject a known frequency offset; no
 * received samples are inserted, removed, resampled or replayed.
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "spandsp.h"
#include "spandsp/private/bitstream.h"
#include "spandsp/private/logging.h"
#include "spandsp/private/power_meter.h"
#include "spandsp/private/v34.h"

#define BIT_CAPACITY (1 << 20)
#define BLOCK 160
#define SECONDS 20

typedef struct {
    uint8_t bits[BIT_CAPACITY];
    int count;
    bool overflow;
    uint32_t rng;
} bit_stream_t;

static int send_bit(void *user_data)
{
    bit_stream_t *s = user_data;
    int bit;

    s->rng ^= s->rng << 13;
    s->rng ^= s->rng >> 17;
    s->rng ^= s->rng << 5;
    bit = s->rng & 1;
    if (s->count < BIT_CAPACITY)
        s->bits[s->count++] = (uint8_t) bit;
    else
        s->overflow = true;
    return bit;
}

static int idle_bit(void *user_data)
{
    (void) user_data;
    return 1;
}

static void receive_bit(void *user_data, int bit)
{
    bit_stream_t *s = user_data;

    if (bit < 0)
        return;
    if (s->count < BIT_CAPACITY)
        s->bits[s->count++] = (uint8_t) (bit & 1);
    else
        s->overflow = true;
}

static void discard_bit(void *user_data, int bit)
{
    (void) user_data;
    (void) bit;
}

static bool run_case(int bps, bool alaw, float offset_hz)
{
    bit_stream_t *sent = calloc(1, sizeof(*sent));
    bit_stream_t *received = calloc(1, sizeof(*received));
    v34_state_t *tx = NULL, *rx = NULL;
    int16_t wire[BLOCK];
    int alignment = -1, checked = 0, errors = 0;
    bool ok = false;

    if (!sent || !received)
        goto done;
    sent->rng = 0x90B1D47AU;
    tx = v34_init(NULL, 3200, bps, false, true,
                  send_bit, sent, discard_bit, NULL);
    rx = v34_init(NULL, 3200, bps, false, true,
                  idle_bit, NULL, receive_bit, received);
    if (!tx || !rx)
        goto done;
    v34_set_v90_mode(tx, alaw ? 1 : 0);
    v34_set_v90_mode(rx, alaw ? 1 : 0);
    v34_set_put_phase4_bit(rx, discard_bit, NULL);
    v34_force_v90_phase4_cp_rx(rx);
    if (v34_v90_prepare_upstream_data(rx, 4, 0, bps, 0) != 0
        || v34_begin_rx_data(rx) != 0
        || v34_v90_begin_tx_data(tx, bps/2400, 0, 0, 0, NULL) != 0)
        goto done;

    for (int block = 0; block < SECONDS*8000/BLOCK; block++) {
        /* Change frequency, not phase, after the two-second acquisition
           interval. +/-0.1 Hz gives 1.8 full turns before the test ends. */
        if (block == 2*8000/BLOCK)
            tx->tx.v34_carrier_phase_rate += dds_phase_ratef(offset_hz);
        if (v34_tx(tx, wire, BLOCK) != BLOCK)
            goto done;
        for (int i = 0; i < BLOCK; i++)
            wire[i] = alaw ? alaw_to_linear(linear_to_alaw(wire[i]))
                           : ulaw_to_linear(linear_to_ulaw(wire[i]));
        if (v34_rx(rx, wire, BLOCK) != 0)
            goto done;
    }
    if (sent->overflow || received->overflow
        || !v34_v90_upstream_rx_acquired(rx))
        goto done;

    /* Find the first PRBS bits after the receiver's startup/B1 output.
       Only one alignment is allowed, then compare the entire held stream:
       searching again after errors would silently forgive a symbol slip. */
    for (int shift = 0; shift < 4096 && shift + 128 <= received->count;
         shift++) {
        int mismatch = 0;
        for (int bit = 0; bit < 128; bit++)
            mismatch += received->bits[shift + bit] != sent->bits[bit];
        if (mismatch == 0) {
            alignment = shift;
            break;
        }
    }
    if (alignment < 0)
        goto done;
    checked = received->count - alignment;
    if (checked > sent->count)
        checked = sent->count;
    if (checked < (SECONDS - 1)*bps)
        goto done;
    for (int bit = 0; bit < checked; bit++)
        errors += received->bits[alignment + bit] != sent->bits[bit];
    ok = errors == 0
         && !v34_v90_upstream_resync_required(rx)
         && !v34_v90_upstream_carrier_lost(rx);
done:
    printf("%s: V.90 upstream hold %d %s %+.1f Hz: %d bits, %d errors, "
           "alignment=%d acquired=%d resync=%d lost=%d\n",
           ok ? "PASS" : "FAIL", bps, alaw ? "A-law" : "mu-law", offset_hz,
           checked, errors, alignment,
           rx ? v34_v90_upstream_rx_acquired(rx) : 0,
           rx ? v34_v90_upstream_resync_required(rx) : 0,
           rx ? v34_v90_upstream_carrier_lost(rx) : 0);
    if (tx)
        v34_free(tx);
    if (rx)
        v34_free(rx);
    free(sent);
    free(received);
    return ok;
}

int main(void)
{
    const int rates[] = {21600, 26400};
    const float offsets[] = {0.0f, -0.1f, 0.1f};
    int failed = 0;

    for (unsigned r = 0; r < sizeof(rates)/sizeof(rates[0]); r++)
        for (int law = 0; law < 2; law++)
            for (unsigned o = 0; o < sizeof(offsets)/sizeof(offsets[0]); o++)
                if (!run_case(rates[r], law != 0, offsets[o]))
                    failed++;
    return failed ? 1 : 0;
}
