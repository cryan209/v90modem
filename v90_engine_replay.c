/*
 * v90_engine_replay.c - replay a recorded call through the WHOLE modem
 *                       engine, off the wire.
 *
 * `v90_upstream_replay` drives the T/3 upstream receiver on its own: it forces
 * the context into Phase 4 CP RX with v34_force_v90_phase4_cp_rx() after a
 * second of audio and starts from there.  That is exactly the right tool for
 * asking what the upstream receiver does with a given stretch of samples, and
 * exactly the wrong one for asking why a LIVE call behaves differently from a
 * replay of its own recording -- because the state a real call accumulates on
 * the way to Phase 4 is the difference under investigation.
 *
 * Measured across four live calls against the d-modem rig on 2026-08-21: every
 * live acquisition fitted B1 at 98.3-98.8% for an out-of-sample distance of
 * 0.088 to 0.229, while the identical recorded audio replayed through the
 * upstream receiver alone fits the same B1 at 100.0% for 0.002.  Five
 * explanations were tested from outside and all fell -- ring length, the
 * samples themselves, the negotiated parameters, the anchor position and the
 * resampler's sub-sample phase -- which is the point at which guessing stops
 * being useful and the two paths have to be compared directly.
 *
 * This harness is the second path.  It feeds a recorded live-rx.g711 into
 * me_rx_g711() from the start of the call and lets the engine run V.8, Phase
 * 2, Phase 3 and Phase 4 off it, exactly as the media thread does.  The
 * transmit side is pulled and discarded, the same way the upstream replay
 * discards it: the peer's recorded audio already contains its responses, so a
 * receive-driven replay follows the same trajectory as long as the engine is
 * deterministic given the same samples.
 *
 *   v90_engine_replay <tap.g711> [ulaw|alaw] [--fast] [--from SECONDS]
 *
 * Timing is REAL TIME by default, one 20 ms frame at a time, and that is not
 * an oversight: the engine's phase timeouts are wall-clock (trace_now_ms()),
 * and the V.90 CP decoder runs on its own worker thread, so a free-running
 * replay changes the very timing it is meant to reproduce.  --fast is there
 * for questions where that does not matter, and says so in its output.
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "data_interface.h"
#include "modem_engine.h"

/* One media frame: 20 ms of G.711 at 8 kHz, as the RTP path delivers it. */
#define FRAME_BYTES     160
#define FRAME_NS        20000000LL

static uint8_t *tap;
static long tap_len;

/* The engine writes payload to the DTE the moment it reaches data mode, so
   the PTY has to exist before the replay starts -- without it the DATA path
   dereferences an unopened interface and the harness dies with SIGSEGV at the
   most interesting instant of the call.  sip_modem.c does this from main();
   nothing else does it for us.  The dial and answer callbacks are the AT
   interpreter's, and a replay never dials. */
static void replay_dial(const char *sip_uri, void *user_data)
{
    (void)sip_uri;
    (void)user_data;
}
/*- End of function --------------------------------------------------------*/

static void replay_answer(void *user_data)
{
    (void)user_data;
}
/*- End of function --------------------------------------------------------*/

static void replay_hangup(void *user_data)
{
    (void)user_data;
}
/*- End of function --------------------------------------------------------*/

static int64_t now_ns(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec*1000000000LL + ts.tv_nsec;
}
/*- End of function --------------------------------------------------------*/

static int load_tap(const char *path)
{
    FILE *f = fopen(path, "rb");

    if (!f) {
        fprintf(stderr, "cannot open %s\n", path);
        return -1;
    }
    fseek(f, 0, SEEK_END);
    tap_len = ftell(f);
    fseek(f, 0, SEEK_SET);
    tap = malloc((size_t)tap_len);
    if (!tap || fread(tap, 1, (size_t)tap_len, f) != (size_t)tap_len) {
        fprintf(stderr, "cannot read %s\n", path);
        fclose(f);
        return -1;
    }
    fclose(f);
    return 0;
}
/*- End of function --------------------------------------------------------*/

/* Where the call starts.
 *
 * A tap begins when the server process does, not when the call arrives, so it
 * can open with a long stretch of nothing.  The engine's V.8 timers start at
 * me_on_sip_connected(), so handing it that silence first is not a neutral
 * act -- it times the call out before the audio arrives.  Find the first
 * frame carrying real energy and back off a little. */
static long find_call_start(int alaw)
{
    const long margin = 8000/2;     /* half a second */
    long i;

    for (i = 0; i + FRAME_BYTES <= tap_len; i += FRAME_BYTES) {
        double sum = 0.0;
        int k;

        for (k = 0; k < FRAME_BYTES; k++) {
            /* Both laws encode silence as a small set of codewords around the
               sign bit; a magnitude estimate off the exponent is enough to
               find the start of a call and needs no decode table. */
            uint8_t c = alaw ? (uint8_t)(tap[i + k] ^ 0x55)
                             : (uint8_t)(~tap[i + k]);
            int exponent = (c >> 4) & 0x07;

            sum += exponent;
        }
        if (sum/FRAME_BYTES >= 2.0)
            break;
    }
    if (i + FRAME_BYTES > tap_len)
        return 0;
    return (i > margin) ? i - margin : 0;
}
/*- End of function --------------------------------------------------------*/

int main(int argc, char *argv[])
{
    const char *path;
    int alaw = 0;
    int fast = 0;
    double from = -1.0;
    /* Its own link, so a replay never fights a live server for /tmp/modem0. */
    const char *pty_link = "/tmp/modem-replay";
    long start;
    long pos;
    int64_t t0;
    int frames = 0;
    me_diag_snapshot_t snap;

    if (argc < 2) {
        fprintf(stderr,
                "usage: %s <tap.g711> [ulaw|alaw] [--fast] [--from SECONDS]"
                " [--pty-link PATH]\n",
                argv[0]);
        return 2;
    }
    path = argv[1];
    for (int i = 2; i < argc; i++) {
        if (strcmp(argv[i], "alaw") == 0)
            alaw = 1;
        else if (strcmp(argv[i], "ulaw") == 0)
            alaw = 0;
        else if (strcmp(argv[i], "--fast") == 0)
            fast = 1;
        else if (strcmp(argv[i], "--from") == 0 && i + 1 < argc)
            from = atof(argv[++i]);
        else if (strcmp(argv[i], "--pty-link") == 0 && i + 1 < argc)
            pty_link = argv[++i];
        else
            fprintf(stderr, "ignoring unknown argument '%s'\n", argv[i]);
    }
    if (load_tap(path) != 0)
        return 1;

    start = (from >= 0.0) ? (long)(from*8000.0) : find_call_start(alaw);
    start -= start % FRAME_BYTES;
    if (start < 0)
        start = 0;
    if (start >= tap_len) {
        fprintf(stderr, "start is past the end of the tap\n");
        return 1;
    }

    printf("engine replay: %s, %s, %.1f s, call starts at %.2f s%s\n",
           path, alaw ? "A-law" : "u-law", tap_len/8000.0, start/8000.0,
           fast ? " [--fast: wall-clock timers will NOT match a live call]"
                : "");
    fflush(stdout);

    me_set_verbose(1);
    me_init();
    di_set_callbacks(replay_dial, replay_answer, replay_hangup, NULL);
    if (di_open(pty_link) < 0) {
        fprintf(stderr, "cannot open the replay PTY at %s\n", pty_link);
        me_destroy();
        return 1;
    }
    printf("engine replay: DTE on %s\n", pty_link);
    fflush(stdout);
    me_set_law(alaw ? ME_LAW_ALAW : ME_LAW_ULAW);
    me_on_sip_connected();

    t0 = now_ns();
    for (pos = start; pos + FRAME_BYTES <= tap_len; pos += FRAME_BYTES) {
        uint8_t tx[FRAME_BYTES];

        me_rx_g711(tap + pos, FRAME_BYTES);
        /* Pull the transmit side and throw it away.  The engine's phase
           machine advances on both directions, and the peer's answers are
           already in the recording. */
        (void)me_tx_g711(tx, FRAME_BYTES);
        me_flush_g711_taps();
        if (!fast) {
            /* Absolute schedule rather than a fixed sleep, so the replay does
               not drift away from the media clock it is imitating. */
            int64_t due = t0 + (int64_t)(++frames)*FRAME_NS;
            int64_t wait = due - now_ns();

            if (wait > 0) {
                struct timespec ts;

                ts.tv_sec = (time_t)(wait/1000000000LL);
                ts.tv_nsec = (long)(wait%1000000000LL);
                nanosleep(&ts, NULL);
            }
        }
    }

    me_get_diag_snapshot(&snap);
    printf("engine replay: finished %.1f s of audio, state=%s modulation=%s\n",
           (pos - start)/8000.0,
           me_state_to_str(snap.state),
           me_modulation_to_str(snap.modulation));
    fflush(stdout);
    di_close();
    me_destroy();
    free(tap);
    return 0;
}
/*- End of function --------------------------------------------------------*/
