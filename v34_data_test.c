/* Exact-symbol V.34 mapper/demapper regression (ITU-T V.34 clauses 7-9). */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <spandsp.h>

typedef struct bit_state_s {
    uint32_t lfsr;
    uint32_t expected;
    uint32_t window;
    uint32_t target;
    int observed;
    int output;
    int errors;
    bool synced;
} bit_state_t;

static int pattern_bit(uint32_t *state)
{
    int bit = (int)(*state & 1U);
    uint32_t feedback = ((*state >> 0) ^ (*state >> 2)
                       ^ (*state >> 3) ^ (*state >> 5)) & 1U;
    *state = (*state >> 1) | (feedback << 15);
    return bit;
}

static int get_bit(void *user_data)
{
    return pattern_bit(&((bit_state_t *)user_data)->lfsr);
}

static void put_bit(void *user_data, int bit)
{
    bit_state_t *state = (bit_state_t *)user_data;

    if (bit < 0)
        return;
    state->window = (state->window << 1) | (uint32_t)(bit & 1);
    state->observed++;
    if (!state->synced) {
        if (state->observed >= 32 && state->window == state->target) {
            for (int i = 0; i < 32; i++)
                (void)pattern_bit(&state->expected);
            state->synced = true;
            state->output = 32;
        }
        return;
    }
    if (bit != pattern_bit(&state->expected))
        state->errors++;
    state->output++;
}

static int run_case(int rate_n, int trellis)
{
    bit_state_t source = {.lfsr = 0xACE1U};
    bit_state_t sink = {.expected = 0xACE1U};
    v34_state_t *tx;
    v34_state_t *rx;
    uint32_t sync_state = sink.expected;

    for (int i = 0; i < 32; i++)
        sink.target = (sink.target << 1) | (uint32_t)pattern_bit(&sync_state);
    tx = v34_init(NULL, 2400, rate_n*2400, true, true,
                  get_bit, &source, put_bit, &source);
    rx = v34_init(NULL, 2400, rate_n*2400, false, true,
                  get_bit, &sink, put_bit, &sink);
    if (!tx || !rx
        || v34_seed_tx_data(tx, rate_n, trellis, 0, 0, NULL) != 0
        || v34_seed_rx_mp(rx, rate_n, trellis, 0, 0, NULL) != 0
        || v34_begin_rx_data(rx) != 0) {
        fprintf(stderr, "v34_data_test: setup failed N=%d trellis=%d\n",
                rate_n, trellis);
        if (tx) v34_free(tx);
        if (rx) v34_free(rx);
        return 1;
    }
    for (int frame_no = 0; frame_no < 256; frame_no++) {
        int16_t frame[16];
        if (v34_get_mapping_frame_state(tx, frame) != 16) {
            fprintf(stderr, "v34_data_test: mapper failed\n");
            return 1;
        }
        v34_put_mapping_frame_state(rx, frame);
    }
    v34_free(tx);
    v34_free(rx);
    if (!sink.synced || sink.output < 1000 || sink.errors != 0) {
        fprintf(stderr,
                "v34_data_test: FAIL N=%d trellis=%d sync=%d bits=%d errors=%d\n",
                rate_n, trellis, sink.synced, sink.output, sink.errors);
        return 1;
    }
    return 0;
}

int main(void)
{
    for (int trellis = 0; trellis < 3; trellis++) {
        /* 2400 baud supports MP rate N=1..9 (V.34 Tables 8/20). */
        for (int rate_n = 1; rate_n <= 9; rate_n++) {
            if (run_case(rate_n, trellis) != 0)
                return 1;
        }
    }
    puts("v34_data_test: OK");
    return 0;
}
