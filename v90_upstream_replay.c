/*
 * v90_upstream_replay.c - replay a recorded call into the V.90 upstream
 *                         receiver, off the wire.
 *
 * The upstream receiver acquires B1, decodes the peer's idle stream
 * perfectly for thirty to forty seconds, and then the symbols leave the
 * constellation and never come back.  Everything testable from outside a
 * live call has been ruled out -- timing, gain, equalizer taps, carrier,
 * payload, framing, alignment, and the media path (the received level is
 * flat at 305-315 RMS straight through the collapse).  What is left are
 * questions of the form "what does the receiver do with THESE samples, and
 * what changes if I alter X", and each one has been costing a five-minute
 * call on a rig whose Phase 3 succeeds sporadically.
 *
 * The taps already hold the audio of calls that collapsed.  This feeds one
 * back through the same receive path, so the fault can be bisected on the
 * desk in seconds.
 *
 *   v90_upstream_replay <tap.g711> [ulaw|alaw] [baud] [bps]
 *
 * The E handover is not recorded in the tap, so it is searched for: the
 * receiver is prepared at a candidate instant, given the preceding history
 * that its B1 search reaches back over, and asked whether it acquired.  The
 * first candidate that acquires with a good fit is the real one -- B1
 * correlates at 98% and nothing else correlates at all.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <inttypes.h>

#include "spandsp.h"

#define BLOCK           160
/* How far back the receiver's own B1 search reaches, plus margin. */
#define HISTORY_SEC     14
#define ACQUIRE_WINDOW  8

static int16_t *samples;
static int sample_count;

static int64_t bits_out;

/* Keep the published bits so the replay can report PAYLOAD, not just a bit
   count.  Every metric inside the receiver is a proxy -- the marks fraction,
   the V.14 ratio, the shell-index bound -- and the frame-phase work is
   precisely where a proxy can read perfect while nothing decodes.  The soak
   pattern is "U%07d" lines, so counting intact ones is the end-to-end answer
   and it owes nothing to the receiver's own opinion of itself. */
static uint8_t *bit_log;
static int64_t bit_log_len;
static int64_t bit_log_cap;

static void replay_put_bit(void *user_data, int bit)
{
    (void) user_data;
    if (bit < 0)
        return;
    /*endif*/
    bits_out++;
    if (bit_log_len >= bit_log_cap)
    {
        int64_t want = bit_log_cap ? bit_log_cap*2 : (1 << 20);
        uint8_t *bigger = realloc(bit_log, (size_t) want);

        if (bigger == NULL)
            return;
        /*endif*/
        bit_log = bigger;
        bit_log_cap = want;
    }
    /*endif*/
    bit_log[bit_log_len++] = (uint8_t) (bit & 1);
}
/*- End of function --------------------------------------------------------*/

/* V.14 async framing: a zero start bit, eight data bits LSB first, a one stop
   bit.  The bit phase is not known, so try all ten and keep the best. */
static int replay_count_pattern_lines(void)
{
    int best = 0;

    for (int ph = 0;  ph < 10;  ph++)
    {
        int64_t i = ph;
        int lines = 0;
        int run = 0;      /* how much of "U" + 7 digits has matched */

        while (i + 9 < bit_log_len)
        {
            int c;

            if (bit_log[i] != 0  ||  bit_log[i + 9] != 1)
            {
                i++;
                run = 0;
                continue;
            }
            /*endif*/
            c = 0;
            for (int b = 0;  b < 8;  b++)
                c |= bit_log[i + 1 + b] << b;
            /*endfor*/
            i += 10;
            if (run == 0)
                run = (c == 'U') ? 1 : 0;
            else if (c >= '0'  &&  c <= '9')
            {
                if (++run == 8)
                {
                    lines++;
                    run = 0;
                }
                /*endif*/
            }
            else
                run = (c == 'U') ? 1 : 0;
            /*endif*/
        }
        /*endwhile*/
        if (lines > best)
            best = lines;
        /*endif*/
    }
    /*endfor*/
    return best;
}
/*- End of function --------------------------------------------------------*/

static int replay_get_bit(void *user_data)
{
    (void) user_data;
    return 1;
}
/*- End of function --------------------------------------------------------*/

static void replay_phase4_bit(void *user_data, int bit)
{
    (void) user_data;
    (void) bit;
}
/*- End of function --------------------------------------------------------*/

static void replay_log(void *user_data, int level, const char *text)
{
    (void) user_data;
    (void) level;
    fputs(text, stdout);
}
/*- End of function --------------------------------------------------------*/

static int load_tap(const char *path, int alaw)
{
    FILE *f = fopen(path, "rb");
    long len;
    uint8_t *raw;

    if (!f)
    {
        fprintf(stderr, "cannot open %s\n", path);
        return -1;
    }
    /*endif*/
    fseek(f, 0, SEEK_END);
    len = ftell(f);
    fseek(f, 0, SEEK_SET);
    raw = malloc(len);
    samples = malloc(len*sizeof(int16_t));
    if (!raw || !samples || fread(raw, 1, len, f) != (size_t) len)
    {
        fprintf(stderr, "cannot read %s\n", path);
        fclose(f);
        return -1;
    }
    /*endif*/
    fclose(f);
    for (long i = 0;  i < len;  i++)
        samples[i] = alaw ? alaw_to_linear(raw[i]) : ulaw_to_linear(raw[i]);
    /*endfor*/
    free(raw);
    sample_count = (int) len;
    return 0;
}
/*- End of function --------------------------------------------------------*/

/* Feed the receiver from `from` up to `to`, in whole blocks. */
static int feed(v34_state_t *rx, int from, int to)
{
    if (from < 0)
        from = 0;
    /*endif*/
    if (to > sample_count)
        to = sample_count;
    /*endif*/
    for (int i = from;  i + BLOCK <= to;  i += BLOCK)
    {
        if (v34_rx(rx, samples + i, BLOCK) != 0)
            return -1;
        /*endif*/
    }
    /*endfor*/
    return 0;
}
/*- End of function --------------------------------------------------------*/

static v34_state_t *make_rx(int alaw, int baud_code, int high_carrier,
                            int bps, int verbose)
{
    v34_state_t *rx = v34_init(NULL, baud_code == 3 ? 3000 : 3200, 21600,
                               false, true,
                               replay_get_bit, NULL, replay_put_bit, NULL);
    logging_state_t *log;

    if (!rx)
        return NULL;
    /*endif*/
    v34_set_v90_mode(rx, alaw ? 1 : 0);
    /* The Phase 4 CP receiver is what puts this context on the primary-channel
       demodulator, and it declines to do so without a Phase 4 bit handler --
       leaving v34_rx() dispatching into the INFO demodulator, where the T/3
       ring is never filled and B1 is never searched for. */
    v34_set_put_phase4_bit(rx, replay_phase4_bit, NULL);
    v34_force_v90_phase4_cp_rx(rx);
    log = v34_get_logging_state(rx);
    if (log)
    {
        span_log_set_message_handler(log, replay_log, NULL);
        span_log_set_level(log,
                           SPAN_LOG_SHOW_SEVERITY | SPAN_LOG_SHOW_PROTOCOL
                           | (verbose ? SPAN_LOG_FLOW : SPAN_LOG_WARNING));
    }
    /*endif*/
    /* Prepare only.  The receiver starts capturing into its T/3 ring here,
       and v34_begin_rx_data() is what marks the E handover -- so the history
       the B1 search reaches back over has to be fed BETWEEN the two, or the
       anchor lands at the start of the replay and the search looks at the
       wrong half second entirely. */
    if (v34_v90_prepare_upstream_data(rx, baud_code, high_carrier, bps, 0) != 0)
    {
        v34_free(rx);
        return NULL;
    }
    /*endif*/
    return rx;
}
/*- End of function --------------------------------------------------------*/

int main(int argc, char *argv[])
{
    const char *path;
    int alaw;
    int baud_code;
    int bps;
    int found = -1;
    v34_state_t *rx = NULL;

    if (argc < 2)
    {
        fprintf(stderr,
                "usage: %s <tap.g711> [ulaw|alaw] [3000|3200] [bps]\n",
                argv[0]);
        return 2;
    }
    /*endif*/
    path = argv[1];
    alaw = (argc > 2 && argv[2][0] == 'a');
    baud_code = (argc > 3 && atoi(argv[3]) == 3000) ? 3 : 4;
    bps = (argc > 4) ? atoi(argv[4]) : 9600;
    if (load_tap(path, alaw) != 0)
        return 1;
    /*endif*/
    printf("replay: %s, %s, %d baud, %d bps, %.1f s\n",
           path, alaw ? "A-law" : "u-law", baud_code == 3 ? 3000 : 3200, bps,
           sample_count/8000.0);

    /* Find the E handover.  Nothing but B1 correlates, so the candidate that
       acquires is the right one. */
    if (argc > 5)
    {
        /* A forced handover instant, with the receiver's own logs turned all
           the way up -- for asking why a candidate did not acquire.

           An optional sixth argument gives the seconds of history to feed
           before the handover, which is the one thing a replay does not get
           right by default: live, capture starts when the upstream is armed,
           so by the time E fires the T/3 ring may hold only a few tenths of a
           second, where this harness hands the receiver fourteen seconds.
           Live and offline fit the SAME B1 to 98.8% and 100.0% respectively,
           and the length of the ring behind it is the obvious suspect. */
        int t = (int) (atof(argv[5])*8000.0);
        double history = (argc > 6) ? atof(argv[6]) : (double) HISTORY_SEC;

        rx = make_rx(alaw, baud_code, 0, bps, 1);
        printf("replay: feeding %.2f s of history before the handover\n",
               history);
        feed(rx, t - (int) (history*8000.0), t);
        printf("replay: handover forced at %.1f s\n", t/8000.0);
        (void) v34_begin_rx_data(rx);
        feed(rx, t, sample_count);
        printf("replay: acquired=%d\n", v34_v90_upstream_rx_acquired(rx));
        return 0;
    }
    /*endif*/
    for (int t = HISTORY_SEC*8000;
         t + ACQUIRE_WINDOW*8000 < sample_count;
         t += 8000/2)
    {
        /* The slip search, the gain sweep and the offset profiles are all
           logged at FLOW: without them the run says the symbols were lost
           and not what the receiver tried. */
        rx = make_rx(alaw, baud_code, 0, bps,
                     getenv("V90_REPLAY_VERBOSE") != NULL);
        if (!rx)
        {
            fprintf(stderr, "cannot prepare the receiver\n");
            return 1;
        }
        /*endif*/
        feed(rx, t - HISTORY_SEC*8000, t);
        if (v34_begin_rx_data(rx) != 0)
        {
            v34_free(rx);
            rx = NULL;
            continue;
        }
        /*endif*/
        feed(rx, t, t + ACQUIRE_WINDOW*8000);
        if (v34_v90_upstream_rx_acquired(rx))
        {
            found = t;
            printf("replay: B1 acquired with the handover at %.1f s\n",
                   t/8000.0);
            break;
        }
        /*endif*/
        v34_free(rx);
        rx = NULL;
    }
    /*endfor*/
    if (found < 0)
    {
        printf("replay: no B1 anywhere in this tap -- either it holds no "
               "upstream data mode, or the rate/law given is not the one "
               "the call used\n");
        return 1;
    }
    /*endif*/

    /* Now run the rest of the call through the acquired receiver.  Its own
       probes report the symbol error, the ones fraction and the timing and
       carrier loops on the way past. */
    bits_out = 0;
    feed(rx, found + ACQUIRE_WINDOW*8000, sample_count);
    printf("replay: finished, %" PRId64 " bits published, "
           "%d intact U%%07d pattern lines\n",
           bits_out, replay_count_pattern_lines());
    v34_free(rx);
    free(samples);
    return 0;
}
/*- End of function --------------------------------------------------------*/
