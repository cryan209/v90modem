/*
 * v70_test.c -- two V.70 DSVD terminals, each a full V.76 + V.75 stack,
 * joined by a bit pipe.  Voice is opaque 10-octet frames (the speech coder is
 * outside this module), checked for integrity and cadence; data is a DTE byte
 * stream or a stream of HDLC frames.
 */

#include "v70.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures, checks;

#define CHECK(cond, ...) \
    do { \
        checks++; \
        if (!(cond)) { \
            printf("FAIL: "); printf(__VA_ARGS__); printf("  [%s:%d]\n", __FILE__, __LINE__); \
            failures++; \
        } \
    } while (0)

static uint32_t rng_state = 0x13579BDu;
static uint32_t rnd(void)
{
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return rng_state;
}

/* ---------------------------------------------------------------------- */

static void test_tunnel(void)
{
    uint8_t out[600], f[300];
    v70_tunnel_rx_t r;
    int n, i, got = 0, t;

    /* Annex A: flag and control escape get bit 6 complemented behind an escape. */
    {
        const uint8_t fr[5] = { 0x01, 0x7E, 0x7D, 0x55, 0x7E };

        n = v70_tunnel_encode(fr, 5, out, sizeof(out));
        CHECK(n == 1 + 1 + 2 + 2 + 1 + 2 + 1, "encoded length %d", n);
        CHECK(out[0] == 0x7E && out[n - 1] == 0x7E, "flag delimited");
        CHECK(out[2] == 0x7D && out[3] == 0x5E, "0x7E -> 7D 5E (bit 6 complemented)");
        CHECK(out[4] == 0x7D && out[5] == 0x5D, "0x7D -> 7D 5D");
        CHECK(V70_HDLC_ESCAPE == 0x7D, "control escape octet is 0 1 1 1 1 1 0 1 (Figure A.1)");
        v70_tunnel_rx_init(&r);
        for (i = 0; i < n; i++) {
            int k = v70_tunnel_rx_put(&r, out[i]);

            if (k > 0) {
                got = k;
                CHECK(k == 5 && memcmp(r.buf, fr, 5) == 0, "frame recovered exactly");
            }
        }
        CHECK(got == 5, "one frame came out");
    }
    /* Random frames, back to back with shared flags. */
    for (t = 0; t < 200; t++) {
        int len = 1 + (int)(rnd() % 200);
        int frames = 0, k;

        for (i = 0; i < len; i++)
            f[i] = (uint8_t)(rnd() % 4 == 0 ? (rnd() & 1 ? 0x7E : 0x7D) : rnd());
        n = v70_tunnel_encode(f, len, out, sizeof(out));
        v70_tunnel_rx_init(&r);
        for (i = 0; i < n; i++) {
            k = v70_tunnel_rx_put(&r, out[i]);
            if (k > 0) {
                frames++;
                CHECK(k == len && memcmp(r.buf, f, (size_t)len) == 0, "random frame %d exact", t);
            }
        }
        CHECK(frames == 1, "exactly one frame per encode");
    }
    /* Abort: control escape immediately followed by a flag (ISO 3309). */
    v70_tunnel_rx_init(&r);
    CHECK(v70_tunnel_rx_put(&r, 0x7E) == 0, "opening flag");
    v70_tunnel_rx_put(&r, 0x11);
    v70_tunnel_rx_put(&r, 0x7D);
    CHECK(v70_tunnel_rx_put(&r, 0x7E) == -1, "escape + flag aborts the frame");
    v70_tunnel_rx_put(&r, 0x22);
    CHECK(v70_tunnel_rx_put(&r, 0x7E) == 1 && r.buf[0] == 0x22, "and the next frame is fine");
    /* Empty frames (contiguous flags) produce nothing. */
    v70_tunnel_rx_init(&r);
    v70_tunnel_rx_put(&r, 0x7E);
    CHECK(v70_tunnel_rx_put(&r, 0x7E) == 0, "back-to-back flags are not an empty frame");
}

/* ---------------------------------------------------------------------- *
 * Terminal pair
 * ---------------------------------------------------------------------- */

#define MAXV 4000

typedef struct {
    v70_t *t;
    /* voice source */
    int voice_seq;
    bool talking;               /* false: SID/silence frames */
    int silence_period;         /* in silence, a frame every N periods */
    int silence_ctr;
    /* voice sink */
    int voice_got, voice_bad, voice_lost_notes, voice_silence_rx;
    long arrive[MAXV];
    int next_expected;
    int gaps;
    /* DTE */
    uint8_t *tx;                /* bytes to send */
    size_t tx_len, tx_pos;
    uint8_t *rx;
    size_t rx_len, rx_cap;
    long *clock;
    int break_ind, last_break_opt, last_break_len;
    v70_state_t last_state;
    int state_changes;
} term_t;

static void voice_frame_fill(uint8_t *f, int seq)
{
    int i;

    f[0] = (uint8_t)seq;
    f[1] = (uint8_t)(seq >> 8);
    for (i = 2; i < 9; i++)
        f[i] = (uint8_t)(seq * 31 + i);
    f[9] = (uint8_t)(f[0] ^ f[1] ^ f[2] ^ f[8]);
}

static bool voice_frame_ok(const uint8_t *f)
{
    uint8_t g[10];
    int seq = f[0] | (f[1] << 8);

    voice_frame_fill(g, seq);
    return memcmp(f, g, 10) == 0;
}

static int io_voice_get(void *c, uint8_t *frame, int max, bool *sil, bool *sid)
{
    term_t *x = c;

    (void)max;
    if (x->talking) {
        voice_frame_fill(frame, x->voice_seq++);
        return 10;
    }
    *sil = true;
    if (x->silence_period > 0 && ++x->silence_ctr >= x->silence_period) {
        x->silence_ctr = 0;
        *sid = true;
        frame[0] = 0xAA;
        frame[1] = 0x55;
        return 2;                       /* a 2-octet SID, as in G.729 Annex B */
    }
    return 0;
}

static void io_voice_put(void *c, const uint8_t *f, int len, const v75_audio_hdr_t *h)
{
    term_t *x = c;

    if (!f) {
        x->voice_lost_notes++;
        return;
    }
    if (h && h->silence) {
        x->voice_silence_rx++;
        return;
    }
    if (len == 10 && voice_frame_ok(f)) {
        int seq = f[0] | (f[1] << 8);

        if (x->voice_got < MAXV)
            x->arrive[x->voice_got] = *x->clock;
        x->voice_got++;
        if (seq != x->next_expected)
            x->gaps++;
        x->next_expected = seq + 1;
    } else {
        x->voice_bad++;
    }
}

static int io_dte_pull(void *c)
{
    term_t *x = c;

    if (x->tx_pos < x->tx_len)
        return x->tx[x->tx_pos++];
    return -1;
}

static void io_dte_push(void *c, uint8_t b)
{
    term_t *x = c;

    if (x->rx_len == x->rx_cap) {
        x->rx_cap = x->rx_cap * 2 + 4096;
        x->rx = realloc(x->rx, x->rx_cap);
    }
    x->rx[x->rx_len++] = b;
}

static void io_break(void *c, int opt, int len)
{
    term_t *x = c;

    x->break_ind++;
    x->last_break_opt = opt;
    x->last_break_len = len;
}

static void io_state(void *c, v70_state_t s)
{
    term_t *x = c;

    x->last_state = s;
    x->state_changes++;
}

typedef struct {
    term_t a, b;
    long clock;
    int delay;
    uint8_t dab[4096], dba[4096];
    int dpos;
    uint32_t ber_ab, ber_ba;
    long black_from, black_to;
    uint64_t flips;
} duo_t;

static void duo_init(duo_t *d, const v70_config_t *ca, const v70_config_t *cb)
{
    v70_io_t io;

    memset(d, 0, sizeof(*d));
    memset(d->dab, 1, sizeof(d->dab));
    memset(d->dba, 1, sizeof(d->dba));
    d->a.talking = d->b.talking = true;
    d->a.clock = d->b.clock = &d->clock;
    memset(&io, 0, sizeof(io));
    io.voice_get_frame = io_voice_get;
    io.voice_put_frame = io_voice_put;
    io.dte_pull = io_dte_pull;
    io.dte_push = io_dte_push;
    io.break_ind = io_break;
    io.state_ind = io_state;
    io.ctx = &d->a;
    d->a.t = v70_create(ca, &io);
    io.ctx = &d->b;
    d->b.t = v70_create(cb, &io);
}

static void duo_free(duo_t *d)
{
    v70_destroy(d->a.t);
    v70_destroy(d->b.t);
    free(d->a.tx); free(d->b.tx); free(d->a.rx); free(d->b.rx);
}

static void duo_run(duo_t *d, long bits)
{
    long i;

    for (i = 0; i < bits; i++) {
        int a = v70_tx_get_bit(d->a.t);
        int b = v70_tx_get_bit(d->b.t);

        if (d->ber_ab && rnd() < d->ber_ab) { a ^= 1; d->flips++; }
        if (d->ber_ba && rnd() < d->ber_ba) { b ^= 1; d->flips++; }
        if (d->black_to && d->clock >= d->black_from && d->clock < d->black_to) {
            a = 1;
            b = 1;
        }
        if (d->delay > 0) {
            int s = d->dpos % d->delay;
            int da = d->dab[s], db = d->dba[s];

            d->dab[s] = (uint8_t)a;
            d->dba[s] = (uint8_t)b;
            d->dpos++;
            a = da;
            b = db;
        }
        v70_rx_put_bit(d->b.t, a);
        v70_rx_put_bit(d->a.t, b);
        d->clock++;
    }
}

static void duo_start(duo_t *d)
{
    v70_start(d->a.t);
    v70_start(d->b.t);
}

static void load_dte(term_t *x, size_t n, uint32_t seed)
{
    size_t i;
    uint32_t s = seed;

    free(x->tx);
    x->tx = malloc(n);
    for (i = 0; i < n; i++) {
        s = s * 1664525u + 1013904223u;
        x->tx[i] = (uint8_t)(s >> 24);
    }
    x->tx_len = n;
    x->tx_pos = 0;
}

static bool stream_equals(const term_t *tx, const term_t *rx)
{
    return rx->rx_len == tx->tx_pos && memcmp(rx->rx, tx->tx, tx->tx_pos) == 0;
}

static void jitter(const term_t *x, double period_bits, double *mean, double *maxdev)
{
    int i, n = x->voice_got < MAXV ? x->voice_got : MAXV;
    double sum = 0, md = 0;

    if (n < 3) {
        *mean = 0;
        *maxdev = 1e9;
        return;
    }
    for (i = 1; i < n; i++)
        sum += (double)(x->arrive[i] - x->arrive[i - 1]);
    *mean = sum / (n - 1);
    for (i = 1; i < n; i++) {
        double dev = (double)(x->arrive[i] - x->arrive[i - 1]) - period_bits;

        if (dev < 0)
            dev = -dev;
        if (dev > md)
            md = dev;
    }
    *maxdev = md;
}

/* ---------------------------------------------------------------------- */

static void test_basic(void)
{
    duo_t d;
    v70_config_t ca, cb;
    double mean, dev;

    rng_state = 5;
    v70_config_default(&ca, true);
    v70_config_default(&cb, false);
    duo_init(&d, &ca, &cb);
    CHECK(v70_state(d.a.t) == V70_IDLE, "idle before the modem trains");
    duo_start(&d);
    duo_run(&d, 60000);
    CHECK(v70_state(d.a.t) == V70_ACTIVE && v70_state(d.b.t) == V70_ACTIVE,
          "both terminals reach DSVD operation (%d, %d)", v70_state(d.a.t), v70_state(d.b.t));
    CHECK(v70_voice_channel(d.a.t) >= 0 && v70_data_channel(d.a.t) >= 0, "voice and data channels open");

    load_dte(&d.a, 6000, 11);
    load_dte(&d.b, 6000, 22);
    duo_run(&d, 28800 * 6);
    CHECK(d.a.voice_got > 500 && d.b.voice_got > 500,
          "voice flows both ways (%d / %d frames in ~6 s)", d.a.voice_got, d.b.voice_got);
    CHECK(d.a.voice_bad == 0 && d.b.voice_bad == 0, "every voice frame intact");
    CHECK(d.a.gaps == 0 && d.b.gaps == 0, "no voice gaps on a clean line");
    CHECK(stream_equals(&d.a, &d.b) && d.a.tx_pos == 6000, "A->B DTE data exact (%zu of 6000)", d.b.rx_len);
    CHECK(stream_equals(&d.b, &d.a) && d.b.tx_pos == 6000, "B->A DTE data exact (%zu of 6000)", d.a.rx_len);
    jitter(&d.b, 288.0, &mean, &dev);
    printf("  voice period mean %.1f bits (nominal 288), worst deviation %.0f bits (%.1f ms)\n",
           mean, dev, dev / 28.8);
    CHECK(mean > 286 && mean < 290, "voice frames arrive every 10 ms on average");
    CHECK(dev < 1700, "voice jitter bounded by one data frame (%.0f bits)", dev);
    CHECK(v76_stats(v70_mf(d.a.t))->rej_sent == 0, "no recovery needed on a clean line");

    /* 6.3 */
    CHECK(v70_end(d.a.t) == 0, "end of DSVD mode");
    duo_run(&d, 60000);
    CHECK(v70_state(d.a.t) == V70_ENDED && v70_state(d.b.t) == V70_ENDED,
          "both ended (%d, %d)", v70_state(d.a.t), v70_state(d.b.t));
    duo_free(&d);
}

static void test_oob_capabilities(void)
{
    duo_t d;
    v70_config_t ca, cb;
    const v75_tcs_t *pc;

    v70_config_default(&ca, true);
    v70_config_default(&cb, false);
    ca.oob_control = cb.oob_control = true;
    ca.audio_blocking_factor = 2;
    cb.audio_blocking_factor = 2;
    duo_init(&d, &ca, &cb);
    duo_start(&d);
    duo_run(&d, 120000);
    CHECK(v70_state(d.a.t) == V70_ACTIVE && v70_state(d.b.t) == V70_ACTIVE,
          "ACTIVE after capability exchange (%d, %d)", v70_state(d.a.t), v70_state(d.b.t));
    pc = v70_peer_capabilities(d.a.t);
    CHECK(pc && pc->n_caps == 2 && pc->caps[0].is_audio &&
          pc->caps[0].audio.cap == V75_AUDIO_G729_ANNEX_A && pc->caps[0].audio.frames == 2,
          "A learned B's capabilities out of band");
    CHECK(v70_peer_capabilities(d.b.t) != NULL, "and B learned A's (Cor.1 6.2.1)");
    CHECK(pc && pc->mux.crc8 && pc->mux.crc16 && pc->mux.n401 >= 128 && pc->sequence_number == 0,
          "V76Capability and sequence number 0");
    CHECK(pc && pc->desc[0].n_sets == 2, "two simultaneous sets allowed on the control channel");
    duo_run(&d, 28800 * 2);
    CHECK(d.b.voice_got > 40 && d.b.voice_bad == 0, "voice with blocking factor 2 (%d)", d.b.voice_got);
    CHECK(v70_end(d.b.t) == 0, "responder ends the session");
    duo_run(&d, 80000);
    CHECK(v70_state(d.a.t) == V70_ENDED && v70_state(d.b.t) == V70_ENDED,
          "EndSessionCommand and releases close everything (%d, %d)", v70_state(d.a.t),
          v70_state(d.b.t));
    duo_free(&d);
}

static void run_jitter_case(bool sr, double *mean, double *dev, int *data_ok)
{
    duo_t d;
    v70_config_t ca, cb;

    rng_state = 31;
    v70_config_default(&ca, true);
    v70_config_default(&cb, false);
    ca.suspend_resume = cb.suspend_resume = sr;
    ca.data_n401 = cb.data_n401 = 1000;          /* long data frames: the worst case for voice */
    duo_init(&d, &ca, &cb);
    duo_start(&d);
    duo_run(&d, 60000);
    load_dte(&d.a, 40000, 3);
    load_dte(&d.b, 40000, 4);
    duo_run(&d, 28800 * 8);
    jitter(&d.b, 288.0, mean, dev);
    /* Data still in flight at the end is fine; what arrived must be an exact
     * prefix of what was sent. */
    *data_ok = d.b.rx_len > 5000 && d.b.rx_len <= d.a.tx_pos && d.a.tx_pos - d.b.rx_len < 4000 &&
               memcmp(d.b.rx, d.a.tx, d.b.rx_len) == 0 && d.a.voice_bad == 0 && d.b.voice_bad == 0;
    duo_free(&d);
}

static void test_suspend_resume_voice(void)
{
    double m0, d0, m1, d1;
    int ok0, ok1;

    run_jitter_case(false, &m0, &d0, &ok0);
    run_jitter_case(true, &m1, &d1, &ok1);
    printf("  voice jitter with 1000-octet data frames: S/R off %.0f bits (%.1f ms), on %.0f bits (%.1f ms)\n",
           d0, d0 / 28.8, d1, d1 / 28.8);
    CHECK(ok0 && ok1, "data and voice intact with and without suspend/resume");
    CHECK(d1 < d0 / 3, "suspend/resume cuts the voice jitter (%.0f -> %.0f bits)", d0, d1);
    CHECK(d1 < 600, "with S/R the voice waits for at most a few octets (%.0f bits)", d1);
}

static long data_octets_in(bool talking, int silence_period)
{
    duo_t d;
    v70_config_t ca, cb;
    long got;

    rng_state = 8;
    v70_config_default(&ca, true);
    v70_config_default(&cb, false);
    ca.audio_header = cb.audio_header = true;
    duo_init(&d, &ca, &cb);
    d.a.talking = d.b.talking = talking;
    d.a.silence_period = d.b.silence_period = silence_period;
    duo_start(&d);
    duo_run(&d, 60000);
    load_dte(&d.a, 400000, 9);
    load_dte(&d.b, 10, 9);
    duo_run(&d, 28800 * 6);
    got = (long)d.b.rx_len;
    duo_free(&d);
    return got;
}

static void test_silence_frees_bandwidth(void)
{
    /* V.70 5.4: "periods of silence in voice signals may be used to increase
     * the bit rate available for data communication". */
    long talk = data_octets_in(true, 0);
    long quiet = data_octets_in(false, 2);          /* SID every other period */
    long mute = data_octets_in(false, 0);

    printf("  data octets in 6 s at 28.8 kbit/s: talking %ld, silent+SID %ld, silent %ld\n",
           talk, quiet, mute);
    CHECK(talk > 8000, "data flows while talking (%ld)", talk);
    CHECK(quiet > talk * 11 / 10, "silence suppression gives data more of the line (%ld > %ld)", quiet, talk);
    CHECK(mute >= quiet, "and no SID gives it the most (%ld)", mute);
}

static void test_blocking_factor_and_loss(void)
{
    duo_t d;
    v70_config_t ca, cb;
    int i;

    rng_state = 66;
    v70_config_default(&ca, true);
    v70_config_default(&cb, false);
    ca.audio_blocking_factor = cb.audio_blocking_factor = 3;
    duo_init(&d, &ca, &cb);
    duo_start(&d);
    duo_run(&d, 60000);
    CHECK(v70_state(d.a.t) == V70_ACTIVE, "ACTIVE with blocking factor 3 (audio header forced on)");
    duo_run(&d, 28800 * 2);
    CHECK(d.b.voice_got > 150 && d.b.voice_bad == 0 && d.b.gaps == 0,
          "3 coded frames per multiplex frame, split back (%d frames)", d.b.voice_got);
    {
        /* 30 ms period: 864 bits. */
        double mean, dev;

        jitter(&d.b, 288.0, &mean, &dev);
        CHECK(mean > 285 && mean < 291, "frames of one multiplex frame arrive together: mean %.1f bits", mean);
    }
    {
        int before = d.b.voice_got, lost_before = d.b.voice_lost_notes;

        d.black_from = d.clock;
        d.black_to = d.clock + 6000;                /* ~200 ms: several SDUs gone */
        duo_run(&d, 28800);
        CHECK(d.b.voice_lost_notes > lost_before, "the audio header's sequence number revealed the loss (%d)",
              d.b.voice_lost_notes - lost_before);
        CHECK(d.b.voice_got > before, "voice resumes after the gap");
        CHECK(d.b.voice_bad == 0, "no corrupt frame delivered");
    }
    (void)i;
    duo_free(&d);

    /* Bit errors: a frame with a bad FCS is dropped and reported lost; data
     * is recovered by ERM. */
    rng_state = 67;
    v70_config_default(&ca, true);
    v70_config_default(&cb, false);
    duo_init(&d, &ca, &cb);
    duo_start(&d);
    duo_run(&d, 60000);
    load_dte(&d.a, 30000, 5);
    d.ber_ab = 0xFFFFFFFFu / 2500;
    duo_run(&d, 28800 * 8);
    CHECK(d.flips > 50, "errors injected (%llu)", (unsigned long long)d.flips);
    CHECK(d.b.voice_bad == 0, "no corrupted voice frame is ever delivered");
    CHECK(v70_stats(d.b.t)->voice_fcs_lost > 0 && d.b.voice_lost_notes > 0,
          "bad-FCS voice frames are reported as lost (%llu)",
          (unsigned long long)v70_stats(d.b.t)->voice_fcs_lost);
    CHECK(d.b.rx_len > 3000 && d.b.rx_len <= d.a.tx_pos && memcmp(d.b.rx, d.a.tx, d.b.rx_len) == 0,
          "data through the same errors is exact, in order, no gaps (%zu of %zu sent)", d.b.rx_len,
          d.a.tx_pos);
    duo_free(&d);
}

static void test_break(void)
{
    duo_t d;
    v70_config_t ca, cb;

    v70_config_default(&ca, true);
    v70_config_default(&cb, false);
    duo_init(&d, &ca, &cb);
    duo_start(&d);
    duo_run(&d, 60000);
    CHECK(v70_break(d.a.t, 0x80, 30) == 0, "break from the DTE side");
    duo_run(&d, 30000);
    CHECK(d.b.break_ind == 1 && d.b.last_break_opt == 0x80 && d.b.last_break_len == 30,
          "break delivered to the far DTE with its handling option and length");
    duo_free(&d);
}

static size_t encode_stream(const uint8_t frames[][200], const int *lens, int n,
                            uint8_t *out, size_t max)
{
    size_t pos = 0;
    int i;

    for (i = 0; i < n; i++) {
        int k = v70_tunnel_encode(frames[i], lens[i], out + pos, max - pos);

        pos += (size_t)k;
        /* tunnelling output carries its own opening flag; drop the duplicate */
    }
    return pos;
}

static void run_tunnel(v70_data_mode_t mode, int frame_max, const char *label)
{
    duo_t d;
    v70_config_t ca, cb;
    static uint8_t frames[40][200];
    int lens[40], i, nf = 40;
    uint8_t *stream = malloc(40 * 420);
    size_t sl;
    v70_tunnel_rx_t r;
    int got = 0, bad = 0;
    size_t k;

    rng_state = 90;
    v70_config_default(&ca, true);
    v70_config_default(&cb, false);
    ca.data_mode = cb.data_mode = mode;
    ca.data_n401 = cb.data_n401 = mode == V70_DATA_TUNNEL_SAR ? 64 : 256;
    duo_init(&d, &ca, &cb);
    duo_start(&d);
    duo_run(&d, 60000);
    CHECK(v70_state(d.a.t) == V70_ACTIVE, "%s: ACTIVE", label);
    for (i = 0; i < nf; i++) {
        int j;

        lens[i] = 5 + (int)(rnd() % (unsigned)(frame_max - 5));
        for (j = 0; j < lens[i]; j++)
            frames[i][j] = (uint8_t)(rnd() % 5 == 0 ? (rnd() & 1 ? 0x7E : 0x7D) : rnd());
    }
    sl = encode_stream(frames, lens, nf, stream, 40 * 420);
    free(d.a.tx);
    d.a.tx = stream;
    d.a.tx_len = sl;
    d.a.tx_pos = 0;
    duo_run(&d, 28800 * 6);
    CHECK(d.a.tx_pos == sl, "%s: the DTE stream was consumed", label);

    /* Decode what reached the far DTE. */
    v70_tunnel_rx_init(&r);
    for (k = 0; k < d.b.rx_len; k++) {
        int n = v70_tunnel_rx_put(&r, d.b.rx[k]);

        if (n > 0) {
            if (got < nf && n == lens[got] && memcmp(r.buf, frames[got], (size_t)n) == 0)
                got++;
            else
                bad++;
        }
    }
    CHECK(got == nf && bad == 0, "%s: all %d HDLC frames arrive intact and in order (%d, bad %d)",
          label, nf, got, bad);
    CHECK(v70_stats(d.a.t)->frames_tx == (uint64_t)nf && v70_stats(d.b.t)->frames_rx == (uint64_t)nf,
          "%s: one multiplex frame per HDLC frame at the MF boundary", label);
    /* The wire carries no HDLC flags or escapes: transparency was removed. */
    CHECK(d.a.tx != NULL, "%s: done", label);
    d.a.tx = NULL;
    free(stream);
    duo_free(&d);
}

static void test_collision_and_failure(void)
{
    duo_t d;
    v70_config_t ca, cb;
    int voices = 0;

    /* 6.2.2: both ends ask for a voice channel at once.  The initiator
     * refuses the responder's; one voice channel results. */
    rng_state = 3;
    v70_config_default(&ca, true);
    v70_config_default(&cb, false);
    duo_init(&d, &ca, &cb);
    duo_start(&d);
    /* The responder, impatient, opens its own voice channel at the same time. */
    CHECK(v70_open_voice(d.b.t) == 33, "responder's channel numbers start at 33");
    duo_run(&d, 80000);
    CHECK(v70_state(d.a.t) == V70_ACTIVE && v70_state(d.b.t) == V70_ACTIVE,
          "both ACTIVE despite the crossing requests (%d, %d)", v70_state(d.a.t), v70_state(d.b.t));
    voices += v70_voice_channel(d.a.t) >= 0;
    CHECK(voices == 1 && v70_voice_channel(d.a.t) == 1 && v70_voice_channel(d.b.t) == 1,
          "one voice channel, and it is the initiator's (A:%d B:%d)", v70_voice_channel(d.a.t),
          v70_voice_channel(d.b.t));
    duo_run(&d, 28800);
    CHECK(d.b.voice_got > 40 && d.a.voice_got > 40, "voice works over the surviving channel");
    duo_free(&d);

    /* No common speech coder: the call fails. */
    v70_config_default(&ca, true);
    v70_config_default(&cb, false);
    cb.voice_codec = V75_AUDIO_G728;
    ca.oob_control = cb.oob_control = true;
    duo_init(&d, &ca, &cb);
    duo_start(&d);
    duo_run(&d, 120000);
    CHECK(v70_state(d.a.t) == V70_FAILED || v70_state(d.a.t) == V70_ENDED,
          "no common speech coder -> no DSVD call (%d)", v70_state(d.a.t));
    CHECK(v70_voice_channel(d.a.t) < 0, "and no voice channel was opened");
    duo_free(&d);

    /* Without the out-of-band capability exchange the responder simply
     * refuses the unsupported channel. */
    v70_config_default(&ca, true);
    v70_config_default(&cb, false);
    cb.voice_codec = V75_AUDIO_G728;
    duo_init(&d, &ca, &cb);
    duo_start(&d);
    duo_run(&d, 120000);
    CHECK(v70_state(d.a.t) != V70_ACTIVE, "refused voice channel: the initiator does not go ACTIVE (%d)",
          v70_state(d.a.t));
    duo_free(&d);
}

static void test_link_loss(void)
{
    duo_t d;
    v70_config_t ca, cb;

    v70_config_default(&ca, true);
    v70_config_default(&cb, false);
    ca.t401_ms = 100;
    cb.t401_ms = 100;
    duo_init(&d, &ca, &cb);
    duo_start(&d);
    duo_run(&d, 60000);
    load_dte(&d.a, 20000, 1);
    d.black_from = d.clock;
    d.black_to = d.clock + 100000000;
    duo_run(&d, 28800 * 4);
    CHECK(v70_state(d.a.t) == V70_FAILED, "a dead line fails the call via N400 (%d)", v70_state(d.a.t));
    duo_free(&d);
}

int main(void)
{
    test_tunnel();
    test_basic();
    test_oob_capabilities();
    test_suspend_resume_voice();
    test_silence_frees_bandwidth();
    test_blocking_factor_and_loss();
    test_break();
    run_tunnel(V70_DATA_TUNNEL_UNERM, 120, "UNERM tunnelling");
    run_tunnel(V70_DATA_TUNNEL_SAR, 190, "tunnelling with SAR");
    test_collision_and_failure();
    test_link_loss();
    printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
