/*
 * ITU-T V.32bis clause 6 duplex start-up test.
 *
 * Two real v32bis instances, a calling one and an answering one, are pointed
 * at each other through a G.711 bearer in both directions and left to run the
 * reactive Figure 3 dialogue: the answer modem opens with a receiver
 * conditioning signal and R1, the call modem answers with its own
 * conditioning and R2, the answer modem conditions again and sends R3, and
 * the two E words hand both sides into data at the negotiated rate.
 *
 * Nothing here tells either side what the other supports, so the rate that
 * comes out is the one clause 6 negotiated.  Both directions then have to
 * carry the PRBS without error.
 *
 * The clause 6 tone phases (AA/CC against AC/CA, and the NT/MT round-trip
 * estimates they produce) are not in this path yet; the modems start at the
 * point where the call modem has ceased transmitting.  NT and MT are
 * therefore zero unless a test sets them.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <spandsp.h>

typedef struct
{
    uint32_t state;
    int total;
    int errors;
    int first_err;
} bit_stats_t;

static int pattern_bit(void *user_data)
{
    uint32_t *state = (uint32_t *) user_data;
    int bit = *state & 1;

    *state = (*state >> 1) ^ ((uint32_t) -(int32_t) bit & 0x80200003U);
    return bit;
}

static void collect_bit(void *user_data, int bit)
{
    bit_stats_t *stats = (bit_stats_t *) user_data;
    int expected;

    if (bit < 0)
        return;
    expected = pattern_bit(&stats->state);
    stats->total++;
    if (bit != expected)
    {
        if (stats->errors == 0)
            stats->first_err = stats->total;
        stats->errors++;
    }
}

static void bearer(const int16_t in[], int16_t out[], int len, int alaw)
{
    int i;

    for (i = 0;  i < len;  i++)
    {
        out[i] = alaw ? alaw_to_linear(linear_to_alaw(in[i]))
                      : ulaw_to_linear(linear_to_ulaw(in[i]));
    }
}

static int run_duplex(int alaw,
                      int call_rates,
                      int answer_rates,
                      int expected_rate,
                      int nt,
                      int mt)
{
    int16_t call_audio[160];
    int16_t answer_audio[160];
    int16_t to_answer[160];
    int16_t to_call[160];
    uint32_t call_tx_pattern = 0x13579BDFU;
    uint32_t answer_tx_pattern = 0x2468ACE1U;
    bit_stats_t call_rx = {0x2468ACE1U, 0, 0, 0};
    bit_stats_t answer_rx = {0x13579BDFU, 0, 0, 0};
    v32bis_state_t *call;
    v32bis_state_t *answer;
    int block;
    int failed = 0;

    call = v32bis_init(NULL, 14400, true, pattern_bit, &call_tx_pattern, collect_bit, &call_rx);
    answer = v32bis_init(NULL, 14400, false, pattern_bit, &answer_tx_pattern, collect_bit, &answer_rx);
    if (call == NULL  ||  answer == NULL)
    {
        fprintf(stderr, "V.32bis duplex initialisation failed\n");
        return -1;
    }
    if (v32bis_set_supported_bit_rates(call, call_rates) != 0
        || v32bis_set_supported_bit_rates(answer, answer_rates) != 0
        || v32bis_set_round_trip_symbols(call, nt, mt) != 0
        || v32bis_set_round_trip_symbols(answer, nt, mt) != 0
        || v32bis_start_startup(call) != 0
        || v32bis_start_startup(answer) != 0)
    {
        fprintf(stderr, "V.32bis duplex start-up setup failed\n");
        v32bis_free(call);
        v32bis_free(answer);
        return -1;
    }

    /* 10 s is about four times what the dialogue needs, so a run that does
       not finish has stalled rather than run short. */
    for (block = 0;  block < 500;  block++)
    {
        v32bis_tx(call, call_audio, 160);
        v32bis_tx(answer, answer_audio, 160);
        bearer(call_audio, to_answer, 160, alaw);
        bearer(answer_audio, to_call, 160, alaw);
        v32bis_rx(answer, to_answer, 160);
        v32bis_rx(call, to_call, 160);
    }

    if (!v32bis_startup_complete(call)  ||  !v32bis_startup_complete(answer))
        failed = 1;
    if (v32bis_current_bit_rate(call) != expected_rate
        || v32bis_current_bit_rate(answer) != expected_rate)
        failed = 1;
    if (call_rx.total < 2000  ||  answer_rx.total < 2000)
        failed = 1;
    if (call_rx.errors != 0  ||  answer_rx.errors != 0)
        failed = 1;
    printf("V.32bis duplex %s call=%03x answer=%03x -> %d bit/s%s: "
           "rate call=%d answer=%d, call rx %d bits %d errors, "
           "answer rx %d bits %d errors%s\n",
           alaw ? "A-law" : "u-law",
           call_rates,
           answer_rates,
           expected_rate,
           (nt || mt) ? " (NT/MT set)" : "",
           v32bis_current_bit_rate(call),
           v32bis_current_bit_rate(answer),
           call_rx.total,
           call_rx.errors,
           answer_rx.total,
           answer_rx.errors,
           failed ? "   FAILED" : "");
    if (failed)
    {
        fprintf(stderr,
                "  startup complete: call=%d answer=%d; first bad bit call=%d answer=%d\n",
                v32bis_startup_complete(call),
                v32bis_startup_complete(answer),
                call_rx.first_err,
                answer_rx.first_err);
    }
    v32bis_free(call);
    v32bis_free(answer);
    return failed ? -1 : 0;
}

int main(int argc, char *argv[])
{
    static const struct
    {
        int call_rates;
        int answer_rates;
        int expected;
    } cases[] =
    {
        /* Both ends offer everything: clause 6 must land on the top rate. */
        {V32BIS_RATE_14400 | V32BIS_RATE_12000 | V32BIS_RATE_9600
         | V32BIS_RATE_7200 | V32BIS_RATE_4800,
         V32BIS_RATE_14400 | V32BIS_RATE_12000 | V32BIS_RATE_9600
         | V32BIS_RATE_7200 | V32BIS_RATE_4800,
         14400},
        /* The answer modem's R3 has to stay inside what R2 offered. */
        {V32BIS_RATE_9600 | V32BIS_RATE_7200 | V32BIS_RATE_4800,
         V32BIS_RATE_14400 | V32BIS_RATE_12000 | V32BIS_RATE_9600,
         9600},
        /* R2 has to exclude rates absent from R1, so the call modem's larger
           list cannot win. */
        {V32BIS_RATE_14400 | V32BIS_RATE_12000 | V32BIS_RATE_9600
         | V32BIS_RATE_7200 | V32BIS_RATE_4800,
         V32BIS_RATE_7200 | V32BIS_RATE_4800,
         7200},
        {V32BIS_RATE_12000 | V32BIS_RATE_4800,
         V32BIS_RATE_12000 | V32BIS_RATE_9600,
         12000},
        {V32BIS_RATE_4800, V32BIS_RATE_14400 | V32BIS_RATE_4800, 4800}
    };
    size_t i;
    int bad = 0;
    int one = (argc > 1) ? atoi(argv[1]) : -1;

    for (i = 0;  i < sizeof(cases)/sizeof(cases[0]);  i++)
    {
        if (one >= 0  &&  (int) i != one)
            continue;
        if (run_duplex(0, cases[i].call_rates, cases[i].answer_rates, cases[i].expected, 0, 0) != 0)
            bad++;
        if (run_duplex(1, cases[i].call_rates, cases[i].answer_rates, cases[i].expected, 0, 0) != 0)
            bad++;
    }
    /* The NT/MT paths are exercised once the tone phases can supply real
       estimates; a non-zero pair must not break the dialogue in the meantime. */
    if (one < 0)
    {
        if (run_duplex(0,
                       V32BIS_RATE_14400 | V32BIS_RATE_9600,
                       V32BIS_RATE_14400 | V32BIS_RATE_9600,
                       14400,
                       64,
                       64) != 0)
            bad++;
    }
    if (bad != 0)
    {
        fprintf(stderr, "V.32bis duplex start-up: %d case(s) failed\n", bad);
        return 1;
    }
    printf("V.32bis clause 6 duplex start-up passed\n");
    return 0;
}
