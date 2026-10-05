/*
 * v70_v34_test.c -- two complete V.70 DSVD terminals (V.76 mux + V.75 control
 * entity + SCF) running over two real V.34 datapumps joined by a G.711 round
 * trip.  The V.70 stack is the synchronous bit source and sink of each modem,
 * exactly the place V.70 5.7 puts it.
 *
 * Nothing but the waveform crosses between the two sides: V.34 Phases 2-4
 * must complete from audio in both directions before the multiplex layer sees
 * a single bit, and the first SABME may be lost to that start-up and have to
 * be retransmitted.  Voice is opaque 10-octet frames (the coder is outside
 * this module); data is a DTE byte stream.
 *
 *   v70_v34_test [baud bps ulaw|alaw]     default 3200 21600 ulaw
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <spandsp.h>

#include "v70.h"

#define BLOCK 160

typedef struct {
    const char *name;
    v70_t *t;
    int voice_seq, voice_next;
    int voice_got, voice_bad, voice_gaps, voice_lost;
    uint8_t *tx;
    size_t tx_len, tx_pos;
    uint8_t *rx;
    size_t rx_len, rx_cap;
    long data_bits_seen;
} side_t;

static int failures, checks;

#define CHECK(cond, ...) \
    do { \
        checks++; \
        if (!(cond)) { \
            printf("FAIL: "); printf(__VA_ARGS__); printf("  [%s:%d]\n", __FILE__, __LINE__); \
            failures++; \
        } \
    } while (0)

static void voice_fill(uint8_t *f, int seq)
{
    int i;

    f[0] = (uint8_t)seq;
    f[1] = (uint8_t)(seq >> 8);
    for (i = 2; i < 9; i++)
        f[i] = (uint8_t)(seq * 13 + i);
    f[9] = (uint8_t)(f[0] ^ f[1] ^ f[5]);
}

static int io_voice_get(void *c, uint8_t *frame, int max, bool *sil, bool *sid)
{
    side_t *s = c;

    (void)max; (void)sil; (void)sid;
    voice_fill(frame, s->voice_seq++);
    return 10;
}

static void io_voice_put(void *c, const uint8_t *f, int len, const v75_audio_hdr_t *h)
{
    side_t *s = c;
    uint8_t g[10];
    int seq;

    (void)h;
    if (!f) {
        s->voice_lost++;
        return;
    }
    if (len != 10) {
        s->voice_bad++;
        return;
    }
    seq = f[0] | (f[1] << 8);
    voice_fill(g, seq);
    if (memcmp(f, g, 10) != 0) {
        s->voice_bad++;
        return;
    }
    if (s->voice_got > 0 && seq != s->voice_next)
        s->voice_gaps++;
    s->voice_next = seq + 1;
    s->voice_got++;
}

static int io_pull(void *c)
{
    side_t *s = c;

    return s->tx_pos < s->tx_len ? s->tx[s->tx_pos++] : -1;
}

static void io_push(void *c, uint8_t b)
{
    side_t *s = c;

    if (s->rx_len == s->rx_cap) {
        s->rx_cap = s->rx_cap * 2 + 4096;
        s->rx = realloc(s->rx, s->rx_cap);
    }
    s->rx[s->rx_len++] = b;
}

/* The modem's synchronous interface. */
static int modem_get_bit(void *c)
{
    side_t *s = c;

    s->data_bits_seen++;
    return v70_tx_get_bit(s->t);
}

static void modem_put_bit(void *c, int bit)
{
    side_t *s = c;

    v70_rx_put_bit(s->t, bit);
}

static int16_t g711(int16_t v, bool alaw)
{
    return alaw ? alaw_to_linear(linear_to_alaw(v)) : ulaw_to_linear(linear_to_ulaw(v));
}

static void make_side(side_t *s, const char *name, bool initiator, int bps, bool sr, int seed)
{
    v70_config_t cfg;
    v70_io_t io;
    int i;
    uint32_t x = (uint32_t)seed;

    memset(s, 0, sizeof(*s));
    s->name = name;
    v70_config_default(&cfg, initiator);
    cfg.line_bit_rate = bps;
    cfg.suspend_resume = sr;
    /* The default T401 (1 s) is deliberately left alone: it must exceed the
     * acknowledgement latency, which with both sides saturating the link and
     * suspend/resume stretching every data frame is several hundred ms.  A
     * 300 ms T401 here fired 39 times in 20 s (each poll was answered, so the
     * link survived, but it is wasted air time). */
    cfg.data_n401 = 256;
    memset(&io, 0, sizeof(io));
    io.ctx = s;
    io.voice_get_frame = io_voice_get;
    io.voice_put_frame = io_voice_put;
    io.dte_pull = io_pull;
    io.dte_push = io_push;
    s->t = v70_create(&cfg, &io);
    s->tx_len = 20000;
    s->tx = malloc(s->tx_len);
    for (i = 0; i < (int)s->tx_len; i++) {
        x = x * 1664525u + 1013904223u;
        s->tx[i] = (uint8_t)(x >> 24);
    }
}

static int run(int baud, int bps, bool alaw, bool sr)
{
    side_t caller, answer;
    v34_state_t *cm, *am;
    int16_t ctx[BLOCK], atx[BLOCK], crx[BLOCK], arx[BLOCK];
    int block, started_at = -1, active_at = -1;
    int voice_at_active = 0;
    int f0 = failures;
    char label[64];

    snprintf(label, sizeof(label), "%d baud / %d bit/s %s%s", baud, bps, alaw ? "A-law" : "u-law",
             sr ? " + S/R" : "");
    make_side(&caller, "caller", true, bps, sr, 1);
    make_side(&answer, "answer", false, bps, sr, 2);

    cm = v34_init(NULL, baud, bps, true, true, modem_get_bit, &caller, modem_put_bit, &caller);
    am = v34_init(NULL, baud, bps, false, true, modem_get_bit, &answer, modem_put_bit, &answer);
    if (!cm || !am) {
        printf("FAIL: v34_init\n");
        return 1;
    }
    v34_tx_power(cm, -12.0f);
    v34_tx_power(am, -12.0f);

    for (block = 0; block < 4500; block++) {
        int i;

        if (v34_tx(cm, ctx, BLOCK) < BLOCK)
            ;
        if (v34_tx(am, atx, BLOCK) < BLOCK)
            ;
        for (i = 0; i < BLOCK; i++) {
            arx[i] = g711(ctx[i], alaw);
            crx[i] = g711(atx[i], alaw);
        }
        (void)v34_rx(am, arx, BLOCK);
        (void)v34_rx(cm, crx, BLOCK);

        /* DSVD mode begins when the modem says it has trained (6.2); the
         * first bit the modem asks for is that moment for this harness. */
        if (started_at < 0 && caller.data_bits_seen > 0 && answer.data_bits_seen > 0) {
            started_at = block;
            v70_start(caller.t);
            v70_start(answer.t);
        }
        if (active_at < 0 && v70_state(caller.t) == V70_ACTIVE && v70_state(answer.t) == V70_ACTIVE) {
            active_at = block;
            voice_at_active = caller.voice_got + answer.voice_got;
        }
        if (getenv("V70_DEBUG") && active_at >= 0 && (block - active_at) % 100 == 0) {
            int dc = v70_data_channel(caller.t), da = v70_data_channel(answer.t);
            int dlc_c = dc >= 0 ? v75_dlci_of(v70_ce(caller.t), dc) : -1;
            int dlc_a = da >= 0 ? v75_dlci_of(v70_ce(answer.t), da) : -1;

            printf("    t=%.1f caller: unacked=%d backlog=%d t401=%llu rr?  answer: unacked=%d backlog=%d t401=%llu "
                   "(rx %zu/%zu)\n", (block - active_at) * 0.02,
                   v76_unacked_frames(v70_mf(caller.t), dlc_c), v76_data_backlog(v70_mf(caller.t), dlc_c),
                   (unsigned long long)v76_stats(v70_mf(caller.t))->t401_expiries,
                   v76_unacked_frames(v70_mf(answer.t), dlc_a), v76_data_backlog(v70_mf(answer.t), dlc_a),
                   (unsigned long long)v76_stats(v70_mf(answer.t))->t401_expiries, caller.rx_len, answer.rx_len);
        }
        if (active_at >= 0 && block - active_at > 1000)         /* 20 s of DSVD */
            break;
        if (v70_state(caller.t) == V70_FAILED || v70_state(answer.t) == V70_FAILED)
            break;
    }

    printf("  %s: data mode at %.2f s, DSVD ACTIVE at %.2f s, end %.2f s\n", label,
           started_at * 0.02, active_at * 0.02, block * 0.02);
    CHECK(started_at >= 0, "%s: V.34 reached data mode in both directions", label);
    CHECK(active_at >= 0, "%s: both V.70 terminals reached DSVD operation over the real modem "
          "(states %d, %d)", label, v70_state(caller.t), v70_state(answer.t));
    if (active_at >= 0) {
        double secs = (block - active_at) * 0.02;

        CHECK(caller.voice_got > 100 && answer.voice_got > 100 && caller.voice_got + answer.voice_got > voice_at_active,
              "%s: voice flows both ways (%d / %d frames)", label, caller.voice_got, answer.voice_got);
        CHECK(caller.voice_bad == 0 && answer.voice_bad == 0,
              "%s: no corrupt voice frame delivered", label);
        CHECK(caller.voice_gaps == 0 && answer.voice_gaps == 0,
              "%s: no voice gaps (%d / %d)", label, caller.voice_gaps, answer.voice_gaps);
        CHECK(answer.rx_len > 8000 && answer.rx_len <= caller.tx_pos &&
              memcmp(answer.rx, caller.tx, answer.rx_len) == 0,
              "%s: caller->answer data exact, in order (%zu octets)", label, answer.rx_len);
        CHECK(caller.rx_len > 8000 && caller.rx_len <= answer.tx_pos &&
              memcmp(caller.rx, answer.tx, caller.rx_len) == 0,
              "%s: answer->caller data exact, in order (%zu octets)", label, caller.rx_len);
        printf("  %s: %.1f s of DSVD: voice %d/%d frames, data %zu/%zu octets, "
               "REJ %llu/%llu, T401 %llu/%llu\n", label, secs, caller.voice_got, answer.voice_got,
               answer.rx_len, caller.rx_len,
               (unsigned long long)v76_stats(v70_mf(caller.t))->rej_sent,
               (unsigned long long)v76_stats(v70_mf(answer.t))->rej_sent,
               (unsigned long long)v76_stats(v70_mf(caller.t))->t401_expiries,
               (unsigned long long)v76_stats(v70_mf(answer.t))->t401_expiries);
        CHECK(v76_stats(v70_mf(caller.t))->t401_expiries + v76_stats(v70_mf(answer.t))->t401_expiries == 0,
              "%s: no acknowledgement timeout on a clean link (%llu)", label,
              (unsigned long long)(v76_stats(v70_mf(caller.t))->t401_expiries +
                                   v76_stats(v70_mf(answer.t))->t401_expiries));
        CHECK(v76_stats(v70_mf(caller.t))->rx_fcs_errors + v76_stats(v70_mf(answer.t))->rx_fcs_errors < 20,
              "%s: essentially error-free datapump (%llu FCS errors)", label,
              (unsigned long long)(v76_stats(v70_mf(caller.t))->rx_fcs_errors +
                                   v76_stats(v70_mf(answer.t))->rx_fcs_errors));
    }
    v34_free(cm);
    v34_free(am);
    v70_destroy(caller.t);
    v70_destroy(answer.t);
    free(caller.tx); free(answer.tx); free(caller.rx); free(answer.rx);
    return failures != f0;
}

int main(int argc, char **argv)
{
    if (argc == 4) {
        run(atoi(argv[1]), atoi(argv[2]), strcmp(argv[3], "alaw") == 0, false);
        run(atoi(argv[1]), atoi(argv[2]), strcmp(argv[3], "alaw") == 0, true);
    } else {
        run(3200, 21600, false, false);
        run(3200, 21600, false, true);
        run(3200, 21600, true, false);
    }
    printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
