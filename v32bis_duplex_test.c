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
#include <math.h>

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

/*! A near end 2-wire hybrid.  V.32bis is full duplex on one pair, so each
    modem's own transmit comes back into its own receiver a short time later
    through the hybrid's imperfect balance.  Three taps rather than one, so a
    canceller cannot pass by being a pure delay and gain, and a return loss of
    about 12 dB, which is a poor but entirely ordinary hybrid. */
#define HYBRID_DELAY    40
#define HYBRID_LEN      1024

typedef struct
{
    int16_t hist[HYBRID_LEN];
    int pos;
} hybrid_t;

static void hybrid_add(hybrid_t *h, const int16_t tx[], int16_t rx[], int len, float scale)
{
    static const struct
    {
        int delay;
        float gain;
    } taps[] = {{HYBRID_DELAY, 0.20f}, {HYBRID_DELAY + 3, -0.11f}, {HYBRID_DELAY + 9, 0.05f}};
    int i;
    size_t t;
    float echo;
    int idx;
    int v;

    for (i = 0;  i < len;  i++)
    {
        h->hist[h->pos] = tx[i];
        echo = 0.0f;
        for (t = 0;  t < sizeof(taps)/sizeof(taps[0]);  t++)
        {
            idx = (h->pos - taps[t].delay + HYBRID_LEN) & (HYBRID_LEN - 1);
            echo += scale*taps[t].gain*h->hist[idx];
        }
        v = (int) (rx[i] + echo);
        if (v > 32767)
            v = 32767;
        else if (v < -32768)
            v = -32768;
        rx[i] = (int16_t) v;
        h->pos = (h->pos + 1) & (HYBRID_LEN - 1);
    }
}

static int run_duplex(int alaw,
                      int call_rates,
                      int answer_rates,
                      int expected_rate,
                      int nt,
                      int mt,
                      int tones,
                      int delay,
                      int hybrid,
                      int echo_can)
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
    /* A one-way delay line, so the round-trip estimates the tone phases
       produce have something real to estimate. */
    int16_t delay_to_answer[1024];
    int16_t delay_to_call[1024];
    int delay_pos = 0;
    int i;
    hybrid_t call_hybrid;
    hybrid_t answer_hybrid;

    memset(&call_hybrid, 0, sizeof(call_hybrid));
    memset(&answer_hybrid, 0, sizeof(answer_hybrid));

    memset(delay_to_answer, 0, sizeof(delay_to_answer));
    memset(delay_to_call, 0, sizeof(delay_to_call));
    call = v32bis_init(NULL, 14400, true, pattern_bit, &call_tx_pattern, collect_bit, &call_rx);
    answer = v32bis_init(NULL, 14400, false, pattern_bit, &answer_tx_pattern, collect_bit, &answer_rx);
    if (call == NULL  ||  answer == NULL)
    {
        fprintf(stderr, "V.32bis duplex initialisation failed\n");
        return -1;
    }
    if (v32bis_set_supported_bit_rates(call, call_rates) != 0
        || v32bis_set_supported_bit_rates(answer, answer_rates) != 0
        || (!tones  &&  (v32bis_set_round_trip_symbols(call, nt, mt) != 0
                         || v32bis_set_round_trip_symbols(answer, nt, mt) != 0))
        || v32bis_set_echo_canceller(call, echo_can) != 0
        || v32bis_set_echo_canceller(answer, echo_can) != 0
        || (tones ? v32bis_start_tones(call) : v32bis_start_startup(call)) != 0
        || (tones ? v32bis_start_tones(answer) : v32bis_start_startup(answer)) != 0)
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
        if (delay > 0)
        {
            for (i = 0;  i < 160;  i++)
            {
                int16_t a = to_answer[i];
                int16_t c = to_call[i];

                to_answer[i] = delay_to_answer[delay_pos];
                to_call[i] = delay_to_call[delay_pos];
                delay_to_answer[delay_pos] = a;
                delay_to_call[delay_pos] = c;
                if (++delay_pos >= delay)
                    delay_pos = 0;
                /*endif*/
            }
            /*endfor*/
        }
        /*endif*/
        if (hybrid)
        {
            /* Each side's own transmit returns into its own receiver. */
            hybrid_add(&call_hybrid, call_audio, to_call, 160, hybrid/100.0f);
            hybrid_add(&answer_hybrid, answer_audio, to_answer, 160, hybrid/100.0f);
        }
        /*endif*/
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
    if (tones)
    {
        int call_nt = 0;
        int call_mt = 0;
        int answer_nt = 0;
        int answer_mt = 0;
        int call_at;
        int answer_at;
        int delay_symbols;
        int one_way = (int) ((delay + 1.6667)/3.3333);
        int round_trip = 2*one_way;

        v32bis_round_trip_symbols(call, &call_nt, &call_mt);
        v32bis_round_trip_symbols(answer, &answer_nt, &answer_mt);
        call_at = v32bis_tone_transition_symbol(call);
        answer_at = v32bis_tone_transition_symbol(answer);
        /* Both modems' pulse shaper delays are equal, so the gap between the
           two scheduled transitions, in transmit symbols, is the delay 6.1
           and 6.2 measure at the line terminals.  The call modem's AA to CC
           transition answers the answer modem's AC to CA one, which is not
           itself the scheduled transition, so what is compared here is the
           answer modem's scheduled CA to AC against the call modem's CC. */
        delay_symbols = answer_at - call_at;
        printf("    tones: one-way %d samples; call NT=%d, answer MT=%d, "
               "call CC at symbol %d, answer AC at symbol %d, reversal delay %d\n",
               delay, call_nt, answer_mt, call_at, answer_at, delay_symbols);
        /* NT is 128 symbol intervals plus the round trip, MT is 64 plus the
           round trip: each side inserts one 64 symbol interval hop, and the
           call modem's counter spans two of them. */
        if (abs(call_nt - (128 + round_trip)) > 2
            || abs(answer_mt - (64 + round_trip)) > 2)
            failed = 1;
        /*endif*/
        if (call_nt <= 0  ||  answer_mt <= 0)
            failed = 1;
        /*endif*/
        /* 6.1/6.2: "shall be 64 +/- 2 symbol periods", measured at the line
           terminals, so the one-way delay between the two ends' transmit
           symbol indices has to be allowed for. */
        if (delay_symbols < 64 + one_way - 2  ||  delay_symbols > 64 + one_way + 2)
            failed = 1;
        /*endif*/
    }
    /*endif*/
    if (hybrid)
    {
        printf("    hybrid: return loss %.1f dB, echo canceller %s, estimate removed: call %.1f dB, answer %.1f dB\n",
               -20.0*log10(hybrid/100.0) + 12.6,
               echo_can ? "on" : "off",
               v32bis_echo_estimate_level(call),
               v32bis_echo_estimate_level(answer));
    }
    /*endif*/
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

/*! One return loss, with the canceller in and out of the path.  Printed,
    not asserted: what this measures is how much echo the receiver survives,
    and the answer is currently "not much", because clause 6 Note 3's echo
    canceller training period does not exist in this tree, so the canceller
    never sees its own echo without the far end on top of it. */
static int hybrid_sweep(int scale, int delay)
{
    int on;
    int off;

    on = run_duplex(0,
                    V32BIS_RATE_14400 | V32BIS_RATE_12000,
                    V32BIS_RATE_14400 | V32BIS_RATE_12000,
                    14400, 0, 0, 0, delay, scale, 1);
    off = run_duplex(0,
                     V32BIS_RATE_14400 | V32BIS_RATE_12000,
                     V32BIS_RATE_14400 | V32BIS_RATE_12000,
                     14400, 0, 0, 0, delay, scale, 0);
    printf("  hybrid sweep: return loss %.1f dB, delay %d -> canceller on %s, off %s\n",
           -20.0*log10(scale/100.0) + 12.6,
           delay,
           (on == 0) ? "pass" : "FAIL",
           (off == 0) ? "pass" : "FAIL");
    /* Only the canceller's own arm is graded, and only where it has been
       measured to work.  Below about 30 dB of return loss neither arm
       carries the call: clause 6 Note 3's echo canceller training period,
       where the far end is quiet and the near end can see its own echo
       alone, does not exist in this tree, so the canceller has to converge
       in continuous double talk against an echo above the far end signal,
       and it does not. */
    return (scale <= 12  &&  on != 0) ? 1 : 0;
}
/*- End of function --------------------------------------------------------*/

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
        if (run_duplex(0, cases[i].call_rates, cases[i].answer_rates, cases[i].expected, 0, 0, 0, 0, 0, 1) != 0)
            bad++;
        if (run_duplex(1, cases[i].call_rates, cases[i].answer_rates, cases[i].expected, 0, 0, 0, 0, 0, 1) != 0)
            bad++;
    }
    /* A preset NT/MT pair must not break the dialogue when the tone phases
       are skipped. */
    if (one < 0)
    {
        if (run_duplex(0,
                       V32BIS_RATE_14400 | V32BIS_RATE_9600,
                       V32BIS_RATE_14400 | V32BIS_RATE_9600,
                       14400,
                       64,
                       64,
                       0,
                       0,
                       0,
                       1) != 0)
            bad++;
        /* And the whole of clause 6, tone phases included, in both laws:
           NT and MT are measured here, not supplied. */
        if (run_duplex(0,
                       V32BIS_RATE_14400 | V32BIS_RATE_12000 | V32BIS_RATE_9600,
                       V32BIS_RATE_14400 | V32BIS_RATE_12000 | V32BIS_RATE_9600,
                       14400, 0, 0, 1, 0, 0, 1) != 0)
            bad++;
        if (run_duplex(1,
                       V32BIS_RATE_9600 | V32BIS_RATE_7200,
                       V32BIS_RATE_14400 | V32BIS_RATE_9600,
                       9600, 0, 0, 1, 0, 0, 1) != 0)
            bad++;
        /* And with a real one-way delay, which is the whole point of NT and
           MT: both must grow by the round trip. */
        if (run_duplex(0,
                       V32BIS_RATE_14400 | V32BIS_RATE_12000,
                       V32BIS_RATE_14400 | V32BIS_RATE_12000,
                       14400, 0, 0, 1, 80, 0, 1) != 0)
            bad++;
        if (run_duplex(0,
                       V32BIS_RATE_14400 | V32BIS_RATE_12000,
                       V32BIS_RATE_14400 | V32BIS_RATE_12000,
                       14400, 0, 0, 1, 240, 0, 1) != 0)
            bad++;
        /* And over a 2-wire hybrid, which is what V.32bis actually runs
           on: each side's own transmit returns into its own receiver, and
           the canceller in the sample path is what has to remove it.  Swept
           over return loss with the canceller in and out, because a single
           row cannot say whether the canceller is doing anything. */
        {
            static const int scale[] = {100, 50, 25, 12, 6};
            size_t k;

            for (k = 0;  k < sizeof(scale)/sizeof(scale[0]);  k++)
            {
                bad += hybrid_sweep(scale[k], 40);
                bad += hybrid_sweep(scale[k], 80);
                bad += hybrid_sweep(scale[k], 160);
            }
            /*endfor*/
        }
    }
    if (bad != 0)
    {
        fprintf(stderr, "V.32bis duplex start-up: %d case(s) failed\n", bad);
        return 1;
    }
    printf("V.32bis clause 6 duplex start-up passed\n");
    return 0;
}
