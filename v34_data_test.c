/* Exact-symbol V.34 mapper/demapper regression (ITU-T V.34 clauses 7-9). */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <spandsp.h>
#include <spandsp/expose.h>

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

static int run_case_baud(int baud, int rate_n, int trellis, int shaping, int impaired)
{
    bit_state_t source = {.lfsr = 0xACE1U};
    bit_state_t sink = {.expected = 0xACE1U};
    v34_state_t *tx;
    v34_state_t *rx;
    uint32_t sync_state = sink.expected;

    for (int i = 0; i < 32; i++)
        sink.target = (sink.target << 1) | (uint32_t)pattern_bit(&sync_state);
    tx = v34_init(NULL, baud, rate_n*2400, true, true,
                  get_bit, &source, put_bit, &source);
    rx = v34_init(NULL, baud, rate_n*2400, false, true,
                  get_bit, &sink, put_bit, &sink);
    if (!tx || !rx
        || v34_seed_tx_data(tx, rate_n, trellis, 0, shaping, NULL) != 0
        || v34_seed_rx_mp(rx, rate_n, trellis, 0, shaping, NULL) != 0
        || v34_begin_rx_data(rx) != 0) {
        /* Not every N is legal at every symbol rate (V.34 Tables 8/20);
           an unsupported combination is a skip, not a failure. */
        if (tx) v34_free(tx);
        if (rx) v34_free(rx);
        return -1;
    }
    for (int frame_no = 0; frame_no < 256; frame_no++) {
        int16_t frame[16];
        if (v34_get_mapping_frame_state(tx, frame) != 16) {
            fprintf(stderr, "v34_data_test: mapper failed\n");
            return 1;
        }
        /* Cross one hard-slicer boundary, but remain inside the correct
           trellis subset's decision region. An unconstrained decoder emits
           errors; Table 11's U0/Y0 constraint must recover the point. */
        if (impaired && frame_no % 16 == 8)
            frame[6] += 160; /* 1.25 in Q9.7. */
        v34_put_mapping_frame_state(rx, frame);
    }
    v34_free(tx);
    v34_free(rx);
    if (!sink.synced || sink.output < 1000 || sink.errors != 0) {
        fprintf(stderr,
                "v34_data_test: FAIL baud=%d N=%d trellis=%d shaping=%d "
                "sync=%d bits=%d errors=%d\n",
                baud, rate_n, trellis, shaping, sink.synced, sink.output,
                sink.errors);
        return 1;
    }
    return 0;
}

static int run_v0_phase_case(int baud, int frame_offset, int impaired)
{
    bit_state_t source = {.lfsr = 0xACE1U};
    bit_state_t sink = {.expected = 0xACE1U};
    uint32_t sync_state = sink.expected;
    v34_state_t *tx;
    v34_state_t *rx;
    int span;
    int delta;
    int locked_at = -1;
    int errors_at_lock = 0;
    int rc = 1;

    for (int i = 0; i < 32; i++)
        sink.target = (sink.target << 1) | (uint32_t)pattern_bit(&sync_state);
    tx = v34_init(NULL, baud, 19200, true, true, get_bit, &source, NULL, NULL);
    rx = v34_init(NULL, baud, 19200, false, true, NULL, NULL, put_bit, &sink);
    if (!tx || !rx || v34_seed_tx_data(tx, 8, 0, 0, 1, NULL)
        || v34_seed_rx_mp(rx, 8, 0, 0, 1, NULL) || v34_begin_rx_data(rx))
        goto done;
    span = 4*rx->rx.parms.p*rx->rx.parms.j;
    delta = ((4*frame_offset % span) + span) % span;
    /* A foreign encoder with a non-reset B1 state and displaced Table-12
       epoch; payload scrambling and mapping-frame bit packing remain normal.
       Exercise both seven- and eight-data-frame superframes. */
    tx->tx.state = 13;
    tx->tx.y0 = 1;
    tx->tx.super_frame = (tx->tx.parms.j - 1
                         + (frame_offset + tx->tx.parms.p*tx->tx.parms.j)
                           /tx->tx.parms.p) % tx->tx.parms.j;
    tx->tx.data_frame = ((frame_offset % tx->tx.parms.p)
                         + tx->tx.parms.p) % tx->tx.parms.p;
    tx->tx.v0_pattern = 2*tx->tx.super_frame
                     + (tx->tx.data_frame*4 >= 2*tx->tx.parms.p);
    for (int state = 0; state < 16; state++)
        rx->rx.viterbi.vit[15].cumulative_path_metric[state] =
            state == 13 ? 0 : UINT32_MAX;
    rx->rx.viterbi.zero_precoder = false;
    rx->rx.v0_acquiring = true;
    rx->rx.v0_pairs = 0;
    memset(rx->rx.v0_score, 0, sizeof(rx->rx.v0_score));
    for (int frame_no = 0; frame_no < 500; frame_no++)
    {
        int16_t frame[16];
        v34_get_mapping_frame_state(tx, frame);
        if (impaired && frame_no == 5)
            frame[6] += 256;
        v34_put_mapping_frame_state(rx, frame);
        if (!rx->rx.v0_acquiring && locked_at < 0)
        {
            locked_at = frame_no;
            errors_at_lock = sink.errors;
            if (rx->rx.input_4d !=
                ((rx->rx.parms.j - 1)*4*rx->rx.parms.p
                   + 4*(frame_no + 1) + delta) % span)
                goto done;
            if (impaired && 4*(frame_no + 1) < 2*span)
                goto done; /* A damaged first window must not be accepted. */
        }
        /* Allow the four-interval encoder-memory and 15-pair traceback interval for
           decisions made before phase lock to leave the decoder pipeline. */
        if (locked_at >= 0 && frame_no == locked_at + 5)
            errors_at_lock = sink.errors;
    }
    if (locked_at >= 0 && sink.synced && sink.output > 10000
        && sink.errors == errors_at_lock
        && (impaired || sink.errors == 0))
        rc = 0;
done:
    if (rc)
        fprintf(stderr, "V0 phase failed: baud=%d offset=%d impaired=%d "
                        "lock=%d bits=%d errors=%d/%d\n",
                baud, frame_offset, impaired, locked_at, sink.output,
                sink.errors, errors_at_lock);
    v34_free(tx);
    v34_free(rx);
    return rc;
}

int main(void)
{
    /* V.34 Tables 8/20: the maximum N rises with the symbol rate.  The V.90
       upstream this proves out runs at 3200 baud (31200 = N 13), which no
       case here used to cover -- the live upstream decoded to white while
       every 2400-baud case passed. */
    static const struct
    {
        int baud;
        int max_n;
    } rates[] = {{2400, 9}, {2743, 11}, {2800, 11}, {3000, 12}, {3200, 13},
                 {3429, 14}};

    int cases = 0;

    for (int trellis = 0; trellis < 3; trellis++) {
        for (size_t r = 0; r < sizeof(rates)/sizeof(rates[0]); r++) {
            for (int rate_n = 1; rate_n <= rates[r].max_n; rate_n++) {
                for (int shaping = 0; shaping <= 1; shaping++) {
                    int rc = run_case_baud(rates[r].baud, rate_n, trellis,
                                           shaping, 0);

                    if (rc > 0)
                        return 1;
                    /*endif*/
                    if (rc == 0)
                        cases++;
                    /*endif*/
                }
            }
        }
    }
    for (int trellis = 0; trellis < 3; trellis++) {
        for (int shaping = 0; shaping <= 1; shaping++) {
            if (run_case_baud(3200, 12, trellis, shaping, 1) != 0)
                return 1;
            cases++;
        }
    }
    for (int baud_case = 0; baud_case < 2; baud_case++) {
        int baud = baud_case ? 3429 : 3200;
        for (int phase_case = 0; phase_case < 3; phase_case++) {
            if (run_v0_phase_case(baud, phase_case == 1 ? -1 : 1,
                                  phase_case == 2))
                return 1;
            cases++;
        }
    }
    printf("v34_data_test: OK (%d cases, including 6 trellis correction and 6 V0 acquisition cases)\n", cases);
    return 0;
}
