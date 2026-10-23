/* V.90 8.5.1 / V.34 10.1.3.1: B1 specifies a reset trellis, but captured
 * signals can match a nonzero state. Exercise acquisition with real G.711
 * audio and exact known payload, not an acquired flag or an all-ones score.
 * The nonzero starts are deliberate synthetic robustness cases. They do
 * not qualify hardware interop. */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "spandsp.h"
#include "spandsp/private/bitstream.h"
#include "spandsp/private/power_meter.h"
#include "spandsp/private/logging.h"
#include "spandsp/private/v34.h"

typedef struct {
    uint32_t source, expected, target, window, training;
    int sent, received, errors;
    bool synced;
} payload_t;

static int pattern_bit(uint32_t *state)
{
    int bit = *state & 1U;
    uint32_t feedback = ((*state >> 0) ^ (*state >> 2)
                         ^ (*state >> 3) ^ (*state >> 5)) & 1U;
    *state = (*state >> 1) | (feedback << 15);
    return bit;
}

static int get_bit(void *user)
{
    payload_t *p = user;
    /* Allow idle-based phase acquisition before sending known random data. */
    return p->sent++ < 4000 ? 1 : pattern_bit(&p->source);
}

static void put_bit(void *user, int bit)
{
    payload_t *p = user;
    if (bit < 0) return;
    p->window = (p->window << 1) | (bit & 1);
    if (!p->synced) {
        if (p->window == p->target) {
            p->synced = true;
            p->received = 32;
        }
        return;
    }
    if (bit != pattern_bit(&p->expected)) p->errors++;
    p->received++;
}

static void phase4_bit(void *user, int bit)
{
    (void)user;
    (void)bit;
}

static void training_symbol(void *user, float *re, float *im)
{
    payload_t *p = user;
    p->training = p->training*1664525U + 1013904223U;
    *re = (p->training & 1) ? 1.0f : -1.0f;
    *im = (p->training & 0x10000) ? 1.0f : -1.0f;
}

/* Negative state -1 holds four-point training forever, -2 sends noise. */
static int run_case(int state, int baud_code, int rate, int trellis)
{
    payload_t p = {.source = 0xACE1, .expected = 0xACE1, .training = 1971};
    int baud = baud_code == 3 ? 3000 : 3200;
    int16_t audio[160];
    int failed = 1;
    uint32_t noise = 9511;
    for (int i = 0; i < 32; i++)
        p.target = (p.target << 1) | pattern_bit(&p.expected);
    v34_state_t *tx = v34_init(NULL, baud, rate, true, true, get_bit, &p, put_bit, &p);
    v34_state_t *rx = v34_init(NULL, baud, 21600, false, true, get_bit, &p, put_bit, &p);
    if (!tx || !rx) goto done;
    v34_set_v90_mode(tx, 0);
    v34_set_v90_mode(rx, 0);
    if (v34_tx_start_external_symbols(tx, baud_code, 0, training_symbol, &p)) goto done;
    v34_set_put_phase4_bit(rx, phase4_bit, NULL);
    v34_force_v90_phase4_cp_rx(rx);
    if (v34_v90_prepare_upstream_data(rx, baud_code, 0, rate, trellis)) goto done;
    for (int at = 0; at < 8000*9; at += 160) {
        if (at == 8000) {
            if (state >= 0) {
                if (v34_v90_begin_tx_data(tx, rate/2400, trellis, 0, 0, NULL)) goto done;
                tx->tx.state = state;
            }
            int event = v34_get_rx_event(rx);
            if (v34_v90_begin_b1_search(rx) || v34_get_rx_event(rx) != event) goto done;
        }
        if (v34_tx(tx, audio, 160) != 160) goto done;
        for (int i = 0; i < 160; i++) {
            if (state == -2) {
                noise = noise*1664525U + 1013904223U;
                audio[i] = (int16_t)((int)(noise >> 16) - 32768);
            }
            audio[i] = ulaw_to_linear(linear_to_ulaw(audio[i]));
        }
        if (v34_rx(rx, audio, 160)) goto done;
    }
    if (state < 0)
        failed = v34_v90_upstream_rx_acquired(rx) || p.received != 0;
    else
        failed = !v34_v90_upstream_rx_acquired(rx) || !p.synced
            || p.received < 20000 || p.errors != 0 || v34_v90_upstream_carrier_lost(rx);
done:
    printf("%s baud=%d rate=%d trellis=%d initial=%d payload_bits=%d errors=%d\n",
           failed ? "FAIL" : "PASS", baud, rate, trellis, state, p.received, p.errors);
    if (tx) v34_free(tx);
    if (rx) v34_free(rx);
    return failed;
}

int main(void)
{
    int failures = 0;
    for (int state = 0; state < 16; state++)
        failures += run_case(state, 4, 9600, 0);
    failures += run_case(4, 3, 9600, 0);
    failures += run_case(15, 4, 21600, 0);
    failures += run_case(17, 4, 9600, 1);
    failures += run_case(31, 4, 9600, 1);
    failures += run_case(37, 4, 9600, 2);
    failures += run_case(63, 4, 9600, 2);
    failures += run_case(-1, 4, 9600, 0);
    failures += run_case(-2, 4, 9600, 0);
    return failures ? 1 : 0;
}
