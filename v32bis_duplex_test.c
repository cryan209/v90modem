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

#define SPANDSP_EXPOSE_INTERNAL_STRUCTURES
#include <spandsp.h>

typedef struct
{
    uint32_t state;
    int total;
    int errors;
    int first_err;
    /* ITU-T V.32bis 8.2 clamps circuit 104 "when a preamble is detected",
       and detection necessarily lags the start of the preamble, so a few
       tens of bits of the preamble reach the DTE decoded as data before the
       clamp.  That is what the Recommendation asks for and what a real
       error control layer discards on its frame check; here it displaces
       the pattern, so the checker re-anchors itself once and then requires
       the rest to be exact.  The re-anchor SEARCHES for the displacement
       rather than assuming it: a checker that could not fail to re-anchor
       would pass a modem that had lost the stream entirely. */
    int resync;
    int resync_len;
    uint32_t resync_state;
    uint8_t resync_buf[128];
} bit_stats_t;

static int pattern_bit(void *user_data)
{
    uint32_t *state = (uint32_t *) user_data;
    int bit = *state & 1;

    *state = (*state >> 1) ^ ((uint32_t) -(int32_t) bit & 0x80200003U);
    return bit;
}

#define RESYNC_SKIP_MAX 64
#define RESYNC_MATCH    64

static void collect_bit(void *user_data, int bit)
{
    bit_stats_t *stats = (bit_stats_t *) user_data;
    int expected;
    int skip;
    int advance;
    uint32_t anchor;
    int i;
    uint32_t trial;
    int ok;

    if (bit < 0)
        return;
    if (stats->resync)
    {
        stats->resync_buf[stats->resync_len++] = (uint8_t) bit;
        if (stats->resync_len < RESYNC_SKIP_MAX + RESYNC_MATCH)
            return;
        /* The detector's clamp lag advances the receive checker over
           non-data, while the transmit generator pauses during clause 8.
           Search from the last verified pre-procedure state, rather than
           treating those non-data bits as PRBS progress.  Bound both the
           expected displacement and the received prefix; every later bit
           must match the transmitter's original PRBS. */
        anchor = stats->resync_state;
        ok = 0;
        for (advance = 0; advance <= 4096 && !ok; advance++)
        {
            for (skip = 0; skip <= RESYNC_SKIP_MAX; skip++)
            {
                trial = anchor;
                for (i = 0; i < RESYNC_MATCH; i++)
                    if (stats->resync_buf[skip + i] != pattern_bit(&trial))
                        break;
                if (i == RESYNC_MATCH)
                {
                    ok = 1;
                    break;
                }
            }
            if (!ok)
                pattern_bit(&anchor);
        }
        stats->resync = 0;
        stats->resync_len = 0;
        if (!ok)
        {
            /* Nothing in the window matches, so the stream really is lost.
               Say so by counting the whole window as bad. */
            stats->errors += RESYNC_MATCH;
            stats->total += RESYNC_MATCH;
            return;
        }
        stats->state = trial;
        stats->total += RESYNC_MATCH;
        for (i = skip + RESYNC_MATCH; i < RESYNC_SKIP_MAX + RESYNC_MATCH; i++)
        {
            if (stats->resync_buf[i] != pattern_bit(&stats->state))
                stats->errors++;
            stats->total++;
        }
        return;
    }
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

/*! ITU-T V.32bis 6, Note 3 constrains the optional echo canceller training
    sequence's spectrum, and the constraint is numeric, so it is checked
    rather than asserted in a comment: "the echo cancellation sequence shall
    produce a transmitted signal such that the sum of its power in the three
    200 Hz bands centred at 600 Hz, 1800 Hz and 3000 Hz is at least 1 dB
    less than its power in the remaining bandwidth.  This applies for the
    relative power averaged over any 6 ms time interval."

    Those three frequencies are the carrier and the two lines S and S-bar
    put on the line at 1800 +/- 1200 Hz, so what is being required is a
    signal with no spectral lines -- something the far end cannot mistake
    for Segment 1 or 2 of the 5.2 receiver conditioning signal.

    6 ms at 8000 samples/s is 48 samples.  A 48 point Goertzel's main lobe is
    wider than 200 Hz, so this over-counts the in-band power and the test is
    the conservative side of the Recommendation. */
#define NOTE3_WINDOW    48

static int note3_spectrum_ok(const int16_t amp[], int len, double *worst_db)
{
    static const double centre[3] = {600.0, 1800.0, 3000.0};
    int off;
    int i;
    size_t b;
    double total;
    double bands;
    double s1;
    double s2;
    double t;
    double coeff;
    double ratio_db;
    int windows = 0;

    *worst_db = 1000.0;
    for (off = 0;  off + NOTE3_WINDOW <= len;  off += NOTE3_WINDOW)
    {
        total = 0.0;
        for (i = 0;  i < NOTE3_WINDOW;  i++)
            total += (double) amp[off + i]*amp[off + i];
        if (total < 1.0)
            continue;
        bands = 0.0;
        for (b = 0;  b < 3;  b++)
        {
            coeff = 2.0*cos(2.0*M_PI*centre[b]/8000.0);
            s1 = 0.0;
            s2 = 0.0;
            for (i = 0;  i < NOTE3_WINDOW;  i++)
            {
                t = amp[off + i] + coeff*s1 - s2;
                s2 = s1;
                s1 = t;
            }
            bands += (s1*s1 + s2*s2 - coeff*s1*s2)/NOTE3_WINDOW;
        }
        if (bands >= total)
            return 0;
        ratio_db = 10.0*log10((total - bands)/bands);
        if (ratio_db < *worst_db)
            *worst_db = ratio_db;
        windows++;
    }
    if (windows == 0)
        return 0;
    return (*worst_db >= 1.0);
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
                      int echo_can,
                      int call_ec_train,
                      int answer_ec_train,
                      int reneg_by_call,
                      int reneg_rate)
{
    int16_t call_audio[160];
    int16_t answer_audio[160];
    int16_t to_answer[160];
    int16_t to_call[160];
    uint32_t call_tx_pattern = 0x13579BDFU;
    uint32_t answer_tx_pattern = 0x2468ACE1U;
    bit_stats_t call_rx = {.state = 0x2468ACE1U};
    bit_stats_t answer_rx = {.state = 0x13579BDFU};
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
    static int16_t ec_audio[8192*4];
    int ec_len = 0;
    int ec_before = 0;
    int reneg_opened = 0;
    int reneg_done = 0;
    int reneg_goal = 1;
    int clear = (reneg_by_call >= 4);
    int repeat = (reneg_by_call == 3);
    int simultaneous = (reneg_by_call == 2);
    int reneg_block = 0;
    int call_before = 0;
    int answer_before = 0;
    int call_after_base = 0;
    int answer_after_base = 0;
    int call_err_base = 0;
    int rate_at_reneg = 0;
    int answer_err_base = 0;
    double worst_db = 0.0;

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
        || v32bis_set_ec_training_symbols(call, call_ec_train) != 0
        || v32bis_set_ec_training_symbols(answer, answer_ec_train) != 0
        || v32bis_set_echo_canceller(call, echo_can) != 0
        || v32bis_set_echo_canceller(answer, echo_can) != 0
        || v32bis_start_rate_renegotiation(call, 9600) != -1
        || v32bis_start_rate_renegotiation(NULL, 9600) != -1
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
        ec_before = v32bis_tx_in_ec_training(call);
        v32bis_tx(call, call_audio, 160);
        v32bis_tx(answer, answer_audio, 160);
        /* The phase can change part way through a block, and the block it
           changes in carries either silence or the conditioning signal's S,
           whose whole content is lines at exactly the three frequencies
           Note 3 is talking about.  Measuring it would be measuring the
           wrong signal, so only blocks that both began and ended inside the
           sequence are kept. */
        if (ec_before  &&  v32bis_tx_in_ec_training(call))
        {
            for (i = 0;  i < 160  &&  ec_len < (int) (sizeof(ec_audio)/sizeof(ec_audio[0]));  i++)
                ec_audio[ec_len++] = call_audio[i];
            /*endfor*/
        }
        /*endif*/
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
        if (repeat)
        {
            /* Odd callback boundaries must not move the clamp or duplicate
               or discard samples around the E/B1 transition. */
            v32bis_rx(answer, to_answer, 13);
            v32bis_rx(answer, to_answer + 13, 67);
            v32bis_rx(answer, to_answer + 80, 80);
            v32bis_rx(call, to_call, 80);
            v32bis_rx(call, to_call + 80, 80);
        }
        else
        {
            v32bis_rx(answer, to_answer, 160);
            v32bis_rx(call, to_call, 160);
        }
        /* ITU-T V.32bis 8.1: "Rate renegotiation may be initiated at any
           time during data transmission."  Open one once both directions
           are demonstrably carrying data, so the renegotiation is graded
           against a link that was working before it. */
        if (reneg_rate != 0  &&  !reneg_opened
            &&  call_rx.total > 4000  &&  answer_rx.total > 4000)
        {
            if (!(call_rates & V32BIS_RATE_7200)
                && v32bis_start_rate_renegotiation(call, 7200) != -1)
                failed = 1;
            /* Enable higher rates only after start-up, so the upgrade rows
               cannot accidentally start at the requested final rate. */
            if (simultaneous && reneg_rate > expected_rate)
            {
                int all = V32BIS_RATE_14400 | V32BIS_RATE_12000
                        | V32BIS_RATE_9600 | V32BIS_RATE_7200 | V32BIS_RATE_4800;
                if (v32bis_set_supported_bit_rates(call, all) != 0
                    || v32bis_set_supported_bit_rates(answer, all) != 0)
                    failed = 1;
            }
            if (v32bis_start_rate_renegotiation((reneg_by_call && reneg_goal == 1) ? call : answer,
                                                reneg_rate) != 0)
            {
                fprintf(stderr, "  could not open a rate renegotiation\n");
                failed = 1;
            }
            /*endif*/
            if (reneg_by_call == 5)
            {
                /* Foreign-peer fixture: Table 5 Note 3's cleardown R4,
                   including B4=0.  Use the normal on-wire encoder and
                   preamble; only the advertised word is overridden.
                   B7, B8, B11 and B15 set, every rate bit clear. */
                call->reneg_local_rates = 0;
                call->reneg_tx_word = 0x8980;
            }
            if (simultaneous
                && v32bis_start_rate_renegotiation(answer, reneg_rate) != 0)
                failed = 1;
            /* Invalid and overlapping requests must leave the exchange intact. */
            if (v32bis_start_rate_renegotiation(call, 12345) != -1
                || v32bis_start_rate_renegotiation(
                    (reneg_by_call && reneg_goal == 1) ? call : answer,
                    reneg_rate) != -1)
                failed = 1;
            call_rx.resync_state = call_rx.state;
            answer_rx.resync_state = answer_rx.state;
            reneg_opened = 1;
            reneg_block = block;
            call_before = call_rx.errors;
            answer_before = answer_rx.errors;
            if (reneg_goal == 1)
                rate_at_reneg = v32bis_current_bit_rate(call);
        }
        /*endif*/
        if (reneg_opened  &&  !reneg_done
            &&  v32bis_rate_renegotiation_count(call) == reneg_goal
            &&  v32bis_rate_renegotiation_count(answer) == reneg_goal)
        {
            /* Both ends are back in data mode.  8.2 clamps circuit 104
               "when a preamble is detected", and detection lags the start of
               the preamble, so a few tens of bits of preamble reached the
               DTE decoded as data -- which is what the Recommendation asks
               for, and what a real error control layer throws away on its
               frame check.  Here it displaces the pattern, so the checker
               re-anchors once and everything after that has to be exact. */
            reneg_done = 1;
            call_rx.resync = 1;
            call_rx.resync_len = 0;
            answer_rx.resync = 1;
            answer_rx.resync_len = 0;
            call_after_base = call_rx.total;
            answer_after_base = answer_rx.total;
            call_err_base = call_rx.errors;
            answer_err_base = answer_rx.errors;
        }
        /*endif*/
        if (repeat && reneg_goal == 1 && reneg_done
            && call_rx.total - call_after_base > 2000
            && answer_rx.total - answer_after_base > 2000)
        {
            if (call_before || answer_before
                || call_rx.errors != call_err_base
                || answer_rx.errors != answer_err_base)
                failed = 1;
            call_rx.errors = answer_rx.errors = 0;
            reneg_goal = 2;
            reneg_opened = reneg_done = 0;
        }
    }

    if (clear)
    {
        /* Clause 8 Note 2: neither end may resume payload when R4/R5
           have no common rate.  Cleardown reports rate zero and TX stops. */
        int ok = reneg_opened && call_before == 0 && answer_before == 0
              && v32bis_current_bit_rate(call) == 0
              && v32bis_current_bit_rate(answer) == 0
              && !v32bis_startup_complete(call) && !v32bis_startup_complete(answer)
              && call->tx_symbol_index - call->reneg_clear_start_symbol >= 64
              && answer->tx_symbol_index - answer->reneg_clear_start_symbol >= 64
              && v32bis_rate_renegotiation_count(call) == 0
              && v32bis_rate_renegotiation_count(answer) == 0
              && v32bis_tx(call, call_audio, 160) == 0
              && v32bis_tx(answer, answer_audio, 160) == 0;
        printf("V.32bis clause 8 no-common-rate cleardown: %s\n", ok ? "passed" : "FAILED");
        v32bis_free(call);
        v32bis_free(answer);
        return !ok || failed;
    }
    if (!v32bis_startup_complete(call)  ||  !v32bis_startup_complete(answer))
        failed = 1;
    if (ec_len > 0)
    {
        /* 6, Note 3's spectral constraint, and its 8192 symbol interval
           ceiling, on the signal that actually went out. */
        if (!note3_spectrum_ok(ec_audio, ec_len, &worst_db))
        {
            fprintf(stderr,
                    "  Note 3 echo training spectrum: worst 6 ms window is only "
                    "%.2f dB, needs 1 dB\n",
                    worst_db);
            failed = 1;
        }
        /*endif*/
        if (ec_len > (int) (8192*10.0/3.0))
        {
            fprintf(stderr,
                    "  Note 3 echo training ran %d samples, over the 8192 symbol "
                    "interval ceiling\n",
                    ec_len);
            failed = 1;
        }
        /*endif*/
        printf("    Note 3 echo training: %d samples on the line, worst 6 ms "
               "out-of-band margin %.1f dB\n",
               ec_len, worst_db);
    }
    /*endif*/
    if (reneg_rate != 0)
    {
        /* The start-up rate is what expected_rate names; the renegotiation
           has moved on from it by the time this runs. */
        if (rate_at_reneg != expected_rate)
            failed = 1;
        /*endif*/
    }
    else if (v32bis_current_bit_rate(call) != expected_rate
             || v32bis_current_bit_rate(answer) != expected_rate)
    {
        failed = 1;
    }
    /*endif*/
    if (call_rx.total < 2000  ||  answer_rx.total < 2000)
        failed = 1;
    if (reneg_rate == 0
        &&  (v32bis_rate_renegotiation_count(call) != 0
             || v32bis_rate_renegotiation_count(answer) != 0))
    {
        /* Nothing asked for a renegotiation here, so the clause 8 preamble
           watcher has fired on data or on an echo.  That would be invisible
           in the bit counts of a row that recovered afterwards, so it is
           checked directly. */
        fprintf(stderr,
                "  a clause 8 renegotiation fired with none requested: "
                "call %d, answer %d\n",
                v32bis_rate_renegotiation_count(call),
                v32bis_rate_renegotiation_count(answer));
        failed = 1;
    }
    /*endif*/
    if (reneg_rate != 0)
    {
        int after_call = call_rx.total - call_after_base;
        int after_answer = answer_rx.total - answer_after_base;

        printf("    clause 8: %s asked for %d bit/s at block %d -> call %d, "
               "answer %d bit/s, %d/%d renegotiations, %d/%d bits after it\n",
               (repeat ? "call then answer" : simultaneous ? "both" : reneg_by_call ? "call" : "answer"),
               reneg_rate,
               reneg_block,
               v32bis_current_bit_rate(call),
               v32bis_current_bit_rate(answer),
               v32bis_rate_renegotiation_count(call),
               v32bis_rate_renegotiation_count(answer),
               after_call,
               after_answer);
        /* The stretch before the renegotiation and the stretch after it must
           both be error free; the clamp lag between them is not graded, and
           the whole-run counters below would otherwise condemn it. */
        if (!reneg_done
            || call_before != 0  ||  answer_before != 0
            || call_rx.errors != call_err_base
            || answer_rx.errors != answer_err_base)
            failed = 1;
        /*endif*/
        call_rx.errors = 0;
        answer_rx.errors = 0;
        /* Both ends must have completed the requested exchanges, both must
           have landed on the requested rate, and -- the point of clause 8 --
           the data must still be flowing afterwards with the PRBS unbroken,
           which the error counters above already require over the whole run. */
        if (v32bis_rate_renegotiation_count(call) != reneg_goal
            || v32bis_rate_renegotiation_count(answer) != reneg_goal)
            failed = 1;
        /*endif*/
        if (v32bis_current_bit_rate(call) != reneg_rate
            || v32bis_current_bit_rate(answer) != reneg_rate)
            failed = 1;
        /*endif*/
        if (after_call < 2000  ||  after_answer < 2000)
            failed = 1;
        /*endif*/
    }
    /*endif*/
    if (v32bis_retrain_count(call) != 0  ||  v32bis_retrain_count(answer) != 0)
    {
        /* Nothing asked for a clause 7 retrain, so the retrain watch fired on
           data, on an echo, or on a clause 8 preamble's 56T head. */
        fprintf(stderr,
                "  a clause 7 retrain fired with none requested: call %d, answer %d\n",
                v32bis_retrain_count(call),
                v32bis_retrain_count(answer));
        failed = 1;
    }
    /*endif*/
    /* With a renegotiation in the run the whole-run error count is not the
       measure: the clamp lag across the procedure displaces the pattern by
       design, and the clause 8 block below grades the stretch before it and
       the stretch after it separately. */
    if (reneg_rate == 0  &&  (call_rx.errors != 0  ||  answer_rx.errors != 0))
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
        /* NT and MT are both the round trip plus ONE 64 symbol interval
           turnaround, the far end's: Figure 3's legend, "including 64T +/- 2T
           modem turn round delay".  The call modem's counter physically spans
           its own 64T as well, and that is taken off (see
           v32bis_tone_rx()). */
        if (abs(call_nt - (64 + round_trip)) > 2
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

/*! One return loss, three arms: the canceller trained on clause 6 Note 3's
    optional sequence, the canceller trained on the extended TRN of the first
    conditioning signal (the default, Note 3 sequence off), and no canceller.
    Both canceller arms must carry the call; the bare arm failing below about
    30 dB is the control that says the canceller rows prove something. */
static int hybrid_sweep(int scale, int delay)
{
    int on;
    int off;
    int trn;

    on = run_duplex(0,
                    V32BIS_RATE_14400 | V32BIS_RATE_12000,
                    V32BIS_RATE_14400 | V32BIS_RATE_12000,
                    14400, 0, 0, 1, delay, scale, 1, 2048, 2048, 0, 0);
    off = run_duplex(0,
                     V32BIS_RATE_14400 | V32BIS_RATE_12000,
                     V32BIS_RATE_14400 | V32BIS_RATE_12000,
                     14400, 0, 0, 1, delay, scale, 0, 2048, 2048, 0, 0);
    trn = run_duplex(0,
                     V32BIS_RATE_14400 | V32BIS_RATE_12000,
                     V32BIS_RATE_14400 | V32BIS_RATE_12000,
                     14400, 0, 0, 1, delay, scale, 1, 0, 0, 0, 0);
    printf("  hybrid sweep: return loss %.1f dB, delay %d -> canceller on %s, off %s, "
           "on with no Note 3 sequence (TRN-trained) %s\n",
           -20.0*log10(scale/100.0) + 12.6,
           delay,
           (on == 0) ? "pass" : "FAIL",
           (off == 0) ? "pass" : "FAIL",
           (trn == 0) ? "pass" : "FAIL");
    /* The canceller's arm has to carry the call at every return loss in the
       sweep.  Below about 30 dB the bare receiver cannot, and that failure
       is the control: without it a canceller that did nothing at all would
       pass every row here. */
    if (on != 0  ||  trn != 0)
        return 1;
    /*endif*/
    if (scale >= 50  &&  off == 0)
    {
        fprintf(stderr,
                "  the hybrid at %.1f dB return loss is not disturbing the call, "
                "so the canceller rows prove nothing\n",
                -20.0*log10(scale/100.0) + 12.6);
        return 1;
    }
    /*endif*/
    return 0;
}
/*- End of function --------------------------------------------------------*/

/*! ITU-T V.32bis clause 7.  Bring two modems to data through the whole of
    clause 6, have ONE of them initiate a retrain, and leave the other to find
    it the way 7.1/7.2 say it must: by the far end's tone outlasting 128
    symbol intervals in the middle of data.  Nothing tells the responder.
    Both then have to run 6.1/6.2 again from the third paragraph -- the tone
    phases, so NT and MT are measured a second time and must agree with the
    first -- re-enter data, and carry the PRBS clean in both directions.

    The bits between the request and the end of the second start-up are not
    graded: the responder's circuit 104 is clamped only once the tone is
    detected (that is what 7.1/7.2 ask), so the far end's tone reaches it
    as data until then, exactly as in clause 8. */
static int run_retrain(int alaw, int by_call, int delay, int hybrid, int rates, int expected_rate)
{
    int16_t call_audio[160];
    int16_t answer_audio[160];
    int16_t to_answer[160];
    int16_t to_call[160];
    int16_t delay_to_answer[1024];
    int16_t delay_to_call[1024];
    uint32_t call_tx_pattern = 0x13579BDFU;
    uint32_t answer_tx_pattern = 0x2468ACE1U;
    bit_stats_t call_rx = {.state = 0x2468ACE1U};
    bit_stats_t answer_rx = {.state = 0x13579BDFU};
    hybrid_t call_hybrid;
    hybrid_t answer_hybrid;
    v32bis_state_t *call;
    v32bis_state_t *answer;
    int delay_pos = 0;
    int block;
    int i;
    int failed = 0;
    int asked = 0;
    int asked_block = -1;
    int detected_block = -1;
    int done = 0;
    int done_block = -1;
    int call_before = 0;
    int answer_before = 0;
    int call_base = 0;
    int answer_base = 0;
    int call_err_base = 0;
    int answer_err_base = 0;
    int nt1 = 0;
    int mt1 = 0;
    int nt2 = 0;
    int mt2 = 0;
    int dummy;

    memset(delay_to_answer, 0, sizeof(delay_to_answer));
    memset(delay_to_call, 0, sizeof(delay_to_call));
    memset(&call_hybrid, 0, sizeof(call_hybrid));
    memset(&answer_hybrid, 0, sizeof(answer_hybrid));
    call = v32bis_init(NULL, 14400, true, pattern_bit, &call_tx_pattern, collect_bit, &call_rx);
    answer = v32bis_init(NULL, 14400, false, pattern_bit, &answer_tx_pattern, collect_bit, &answer_rx);
    if (call == NULL  ||  answer == NULL
        || v32bis_set_supported_bit_rates(call, rates) != 0
        || v32bis_set_supported_bit_rates(answer, rates) != 0
        /* 7: "A retrain may be initiated during data transmission" -- and
           not before. */
        || v32bis_start_retrain(call) != -1
        || v32bis_start_retrain(NULL) != -1
        || v32bis_start_tones(call) != 0
        || v32bis_start_tones(answer) != 0
        || v32bis_start_retrain(answer) != -1)
    {
        fprintf(stderr, "V.32bis retrain setup failed\n");
        return -1;
    }
    for (block = 0;  block < 1500;  block++)
    {
        v32bis_tx(call, call_audio, 160);
        v32bis_tx(answer, answer_audio, 160);
        bearer(call_audio, to_answer, 160, alaw);
        bearer(answer_audio, to_call, 160, alaw);
        for (i = 0;  i < 160  &&  delay > 0;  i++)
        {
            int16_t a = to_answer[i];
            int16_t c = to_call[i];

            to_answer[i] = delay_to_answer[delay_pos];
            to_call[i] = delay_to_call[delay_pos];
            delay_to_answer[delay_pos] = a;
            delay_to_call[delay_pos] = c;
            if (++delay_pos >= delay)
                delay_pos = 0;
        }
        if (hybrid)
        {
            hybrid_add(&call_hybrid, call_audio, to_call, 160, hybrid/100.0f);
            hybrid_add(&answer_hybrid, answer_audio, to_answer, 160, hybrid/100.0f);
        }
        /* Odd callback lengths, so a detection instant part way through a
           block is exercised. */
        v32bis_rx(answer, to_answer, 37);
        v32bis_rx(answer, to_answer + 37, 123);
        v32bis_rx(call, to_call, 160);
        if (!asked  &&  call_rx.total > 4000  &&  answer_rx.total > 4000)
        {
            v32bis_round_trip_symbols(call, &nt1, &dummy);
            v32bis_round_trip_symbols(answer, &dummy, &mt1);
            call_before = call_rx.errors;
            answer_before = answer_rx.errors;
            /* The last verified PRBS position, which the checker searches
               forward from once data resumes. */
            call_rx.resync_state = call_rx.state;
            answer_rx.resync_state = answer_rx.state;
            if (v32bis_start_retrain(by_call ? call : answer) != 0)
            {
                fprintf(stderr, "  could not initiate a retrain\n");
                failed = 1;
                break;
            }
            asked = 1;
            asked_block = block;
        }
        if (asked  &&  detected_block < 0
            &&  v32bis_retrain_count(by_call ? answer : call) == 1)
            detected_block = block;
        if (asked  &&  !done
            &&  v32bis_startup_complete(call)  &&  v32bis_startup_complete(answer)
            &&  v32bis_retrain_count(call) == 1  &&  v32bis_retrain_count(answer) == 1)
        {
            done = 1;
            done_block = block;
            v32bis_round_trip_symbols(call, &nt2, &dummy);
            v32bis_round_trip_symbols(answer, &dummy, &mt2);
            call_rx.resync = 1;
            call_rx.resync_len = 0;
            answer_rx.resync = 1;
            answer_rx.resync_len = 0;
            call_base = call_rx.total;
            answer_base = answer_rx.total;
            call_err_base = call_rx.errors;
            answer_err_base = answer_rx.errors;
        }
        if (done  &&  call_rx.total - call_base > 6000  &&  answer_rx.total - answer_base > 6000)
            break;
    }
    printf("V.32bis clause 7 retrain by %s, %s, one-way %d, hybrid %d: asked at block %d, "
           "far end detected at block %d, data again at block %d at %d/%d bit/s; "
           "NT %d -> %d, MT %d -> %d; after: call %d bits %d errors, answer %d bits %d errors",
           by_call ? "call" : "answer",
           alaw ? "A-law" : "u-law",
           delay,
           hybrid,
           asked_block,
           detected_block,
           done_block,
           v32bis_current_bit_rate(call),
           v32bis_current_bit_rate(answer),
           nt1, nt2, mt1, mt2,
           call_rx.total - call_base,
           call_rx.errors - call_err_base,
           answer_rx.total - answer_base,
           answer_rx.errors - answer_err_base);
    if (!asked  ||  !done  ||  detected_block < 0)
        failed = 1;
    /* The data before the retrain and the data after it must both be exact. */
    if (call_before != 0  ||  answer_before != 0
        || call_rx.errors != call_err_base
        || answer_rx.errors != answer_err_base)
        failed = 1;
    if (call_rx.total - call_base < 6000  ||  answer_rx.total - answer_base < 6000)
        failed = 1;
    if (v32bis_current_bit_rate(call) != expected_rate
        || v32bis_current_bit_rate(answer) != expected_rate)
        failed = 1;
    /* Exactly one retrain each: neither the responder's own tones nor the
       initiator's echo may set off a second. */
    if (v32bis_retrain_count(call) != 1  ||  v32bis_retrain_count(answer) != 1)
        failed = 1;
    /* The channel has not changed, so the second round-trip measurement has
       to agree with the first within 6.1/6.2's own +/- 2.  This is what
       says both sample clocks were carried correctly through data mode. */
    if (nt1 <= 0  ||  mt1 <= 0  ||  abs(nt2 - nt1) > 2  ||  abs(mt2 - mt1) > 2)
        failed = 1;
    printf("%s\n", failed ? "   FAILED" : "");
    v32bis_free(call);
    v32bis_free(answer);
    return failed;
}
/*- End of function --------------------------------------------------------*/

static int retrain_tests(void)
{
    const int all = V32BIS_RATE_14400 | V32BIS_RATE_12000 | V32BIS_RATE_9600
                  | V32BIS_RATE_7200 | V32BIS_RATE_4800;
    int bad = 0;
    int law;
    int by_call;

    for (law = 0;  law < 2;  law++)
    {
        for (by_call = 0;  by_call < 2;  by_call++)
        {
            bad += run_retrain(law, by_call, 0, 0, all, 14400);
            bad += run_retrain(law, by_call, 240, 0, all, 14400);
        }
    }
    /* Over a 2-wire hybrid with the canceller in: the canceller keeps its
       estimate across the retrain, and the responder's tone watch must not
       be set off by its own echo. */
    bad += run_retrain(0, 1, 80, 25, all, 14400);
    bad += run_retrain(1, 0, 80, 25, all, 14400);
    bad += run_retrain(0, 1, 80, 0, V32BIS_RATE_9600 | V32BIS_RATE_4800, 9600);
    bad += run_retrain(1, 0, 80, 0, V32BIS_RATE_4800, 4800);
    return bad;
}
/*- End of function --------------------------------------------------------*/

static int rate_renegotiation_tests(void)
{
    int bad = 0;
        /* Clause 8: every rate, both initiating roles, both G.711 laws.
           Include unchanged-rate requests: they still require the full
           preamble/R/E/24T exchange and must resume clean payload. */
        {
            static const int rates[] = {4800, 7200, 9600, 12000, 14400};
            const int all_rates = V32BIS_RATE_14400 | V32BIS_RATE_12000
                                | V32BIS_RATE_9600 | V32BIS_RATE_7200
                                | V32BIS_RATE_4800;
            int law;
            int role;
            size_t rate;
            for (law = 0; law < 2; law++)
                for (role = 0; role < 4; role++)
                    for (rate = 0; rate < sizeof(rates)/sizeof(rates[0]); rate++)
                        bad += run_duplex(law, all_rates, all_rates, 14400,
                                          0, 0, 1, 80, 0, 1, 2048, 2048,
                                          role, rates[rate]);
        }
    /* 8.1 explicitly permits both ends initiating together.  Enable rates
       after a 7200 start-up and request 14400 at both ends: an actual upgrade. */
    for (int law = 0; law < 2; law++)
        bad += run_duplex(law, V32BIS_RATE_7200 | V32BIS_RATE_4800,
                          V32BIS_RATE_7200 | V32BIS_RATE_4800, 7200,
                          0, 0, 1, 80, 0, 1, 2048, 2048, 2, 14400);
    for (int law = 0; law < 2; law++)
        bad += run_duplex(law, V32BIS_RATE_14400 | V32BIS_RATE_9600,
                          V32BIS_RATE_14400 | V32BIS_RATE_7200, 14400,
                          0, 0, 1, 80, 0, 1, 2048, 2048, 4, 9600);
    for (int law = 0; law < 2; law++)
        bad += run_duplex(law, V32BIS_RATE_14400 | V32BIS_RATE_9600,
                          V32BIS_RATE_14400 | V32BIS_RATE_9600, 14400,
                          0, 0, 0, 0, 0, 1, 2048, 2048, 1, 9600);
    for (int law = 0; law < 2; law++)
        bad += run_duplex(law, V32BIS_RATE_14400 | V32BIS_RATE_9600,
                          V32BIS_RATE_14400 | V32BIS_RATE_9600, 14400,
                          0, 0, 1, 80, 0, 1, 2048, 2048, 5, 9600);
    return bad;
}

int main(int argc, char *argv[])
{
    if (argc > 1 && strcmp(argv[1], "--reneg-only") == 0)
        return rate_renegotiation_tests() != 0;
    if (argc > 1 && strcmp(argv[1], "--retrain-only") == 0)
        return retrain_tests() != 0;
    if (argc > 3 && strcmp(argv[1], "--hybrid-one") == 0)
    {
        /* One hybrid row: --hybrid-one <scale> <delay> [note3 symbols]. */
        int note3 = (argc > 4) ? atoi(argv[4]) : 0;

        return run_duplex(0,
                          V32BIS_RATE_14400 | V32BIS_RATE_12000,
                          V32BIS_RATE_14400 | V32BIS_RATE_12000,
                          14400, 0, 0, 1, atoi(argv[3]), atoi(argv[2]), 1,
                          note3, note3, 0, 0) != 0;
    }

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
        if (run_duplex(0, cases[i].call_rates, cases[i].answer_rates, cases[i].expected, 0, 0, 0, 0, 0, 1, 2048, 2048, 0, 0) != 0)
            bad++;
        if (run_duplex(1, cases[i].call_rates, cases[i].answer_rates, cases[i].expected, 0, 0, 0, 0, 0, 1, 2048, 2048, 0, 0) != 0)
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
                       1,
                       2048,
                       2048,
                       0,
                       0) != 0)
            bad++;
        /* And the whole of clause 6, tone phases included, in both laws:
           NT and MT are measured here, not supplied. */
        if (run_duplex(0,
                       V32BIS_RATE_14400 | V32BIS_RATE_12000 | V32BIS_RATE_9600,
                       V32BIS_RATE_14400 | V32BIS_RATE_12000 | V32BIS_RATE_9600,
                       14400, 0, 0, 1, 0, 0, 1, 2048, 2048, 0, 0) != 0)
            bad++;
        if (run_duplex(1,
                       V32BIS_RATE_9600 | V32BIS_RATE_7200,
                       V32BIS_RATE_14400 | V32BIS_RATE_9600,
                       9600, 0, 0, 1, 0, 0, 1, 2048, 2048, 0, 0) != 0)
            bad++;
        /* And with a real one-way delay, which is the whole point of NT and
           MT: both must grow by the round trip. */
        if (run_duplex(0,
                       V32BIS_RATE_14400 | V32BIS_RATE_12000,
                       V32BIS_RATE_14400 | V32BIS_RATE_12000,
                       14400, 0, 0, 1, 80, 0, 1, 2048, 2048, 0, 0) != 0)
            bad++;
        if (run_duplex(0,
                       V32BIS_RATE_14400 | V32BIS_RATE_12000,
                       V32BIS_RATE_14400 | V32BIS_RATE_12000,
                       14400, 0, 0, 1, 240, 0, 1, 2048, 2048, 0, 0) != 0)
            bad++;
        /* Note 3 makes the sequence optional, so a conformant far end may
           send none, or a different amount, and each side has to cope with
           whatever the other does.  One row each way. */
        if (run_duplex(0,
                       V32BIS_RATE_14400 | V32BIS_RATE_12000,
                       V32BIS_RATE_14400 | V32BIS_RATE_12000,
                       14400, 0, 0, 1, 80, 0, 1, 0, 2048, 0, 0) != 0)
            bad++;
        if (run_duplex(1,
                       V32BIS_RATE_14400 | V32BIS_RATE_12000,
                       V32BIS_RATE_14400 | V32BIS_RATE_12000,
                       14400, 0, 0, 1, 80, 0, 1, 8192, 0, 0, 0) != 0)
            bad++;
        bad += rate_renegotiation_tests();
        bad += retrain_tests();
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
    printf("V.32bis clauses 6/7/8 duplex tests passed\n");
    return 0;
}
