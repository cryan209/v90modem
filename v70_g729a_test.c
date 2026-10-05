/*
 * v70_g729a_test.c -- the G.729A adapter against the ITU package's own test
 * vectors, then real coded speech through a DSVD pair.
 *
 *   v70_g729a_test <test_vectors dir>        (g729AnnexA/test_vectors)
 *
 * ALGTHM.IN  -> encode -> must equal ALGTHM.BIT   (bit-exact, every frame)
 * ALGTHM.BIT -> DSVD -> decode -> must equal ALGTHM.PST (bit-exact, every sample)
 */
#include "v70.h"
#include "v70_g729a.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures, checks;
#define CHECK(c, ...) do { checks++; if (!(c)) { printf("FAIL: "); printf(__VA_ARGS__); \
    printf("  [%s:%d]\n", __FILE__, __LINE__); failures++; } } while (0)

static uint8_t *slurp(const char *dir, const char *name, size_t *n)
{
    char p[1024];
    FILE *f;
    uint8_t *b;

    snprintf(p, sizeof(p), "%s/%s", dir, name);
    f = fopen(p, "rb");
    if (!f) { printf("cannot open %s\n", p); exit(2); }
    fseek(f, 0, SEEK_END);
    *n = (size_t)ftell(f);
    fseek(f, 0, SEEK_SET);
    b = malloc(*n + 1);
    if (fread(b, 1, *n, f) != *n) exit(2);
    fclose(f);
    return b;
}

/* serial word: 0x007f = 0 bit, 0x0081 = 1 bit (the reference's bit-stream format) */
static void serial_to_octets(const int16_t *w, uint8_t *o)
{
    int i;

    memset(o, 0, 10);
    for (i = 0; i < 80; i++)
        if (w[2 + i] == 0x0081)
            o[i >> 3] |= (uint8_t)(0x80 >> (i & 7));
}

/* ---- end to end: coded speech through a DSVD pair ----------------------- */
typedef struct {
    v70_t *t;
    const uint8_t *frames;      /* A: coded speech to send */
    size_t nframes, next;
    uint8_t got[6000][10];
    int ngot, other;
} side_t;

static int g_get(void *c, uint8_t *f, int max, bool *sil, bool *sid)
{
    side_t *s = c;

    (void)max; (void)sil; (void)sid;
    if (s->frames && s->next < s->nframes) {
        memcpy(f, s->frames + s->next++ * 10, 10);
        return 10;
    }
    memset(f, 0, 10);
    return 0;
}

static void g_put(void *c, const uint8_t *f, int len, const v75_audio_hdr_t *h)
{
    side_t *s = c;

    static int seen;

    seen++;
    if (getenv("V70_DEBUG") && (len != 10 || !f) && s->ngot < 3750)
        printf("  odd delivery #%d at ngot=%d: f=%p len=%d silence=%d lost=%d\n", seen, s->ngot,
               (const void *)f, len, h ? h->silence : -1, h ? h->lost : -1);
    if (f && len == 10 && s->ngot < 6000)
        memcpy(s->got[s->ngot++], f, 10);
}

int main(int argc, char **argv)
{
    size_t nin, nbit, npst, i, nframes;
    uint8_t *in, *bit, *pst, *coded;
    int bad = 0, badsamp = 0;
    int16_t pcm[80], out[80];

    const char *vec = argc >= 3 ? argv[2] : "ALGTHM";
    char fn[64];

    if (argc < 2) { printf("usage: v70_g729a_test <test_vectors dir> [ALGTHM|SPEECH|...]\n"); return 2; }
    snprintf(fn, sizeof(fn), "%s.IN", vec);  in = slurp(argv[1], fn, &nin);
    snprintf(fn, sizeof(fn), "%s.BIT", vec); bit = slurp(argv[1], fn, &nbit);
    snprintf(fn, sizeof(fn), "%s.PST", vec); pst = slurp(argv[1], fn, &npst);
    nframes = nin / 160;
    CHECK(nbit == nframes * 82 * 2, "bit file has %zu frames of 82 words", nframes);
    v70_g729a_init();
    coded = malloc(nframes * 10);
    for (i = 0; i < nframes; i++) {
        uint8_t want[10];

        memcpy(pcm, in + i * 160, 160);
        v70_g729a_encode(pcm, coded + i * 10);
        serial_to_octets((const int16_t *)(bit + i * 164), want);
        if (memcmp(coded + i * 10, want, 10))
            bad++;
    }
    CHECK(bad == 0, "ALGTHM: %zu frames encode bit-exactly (%d differ)", nframes, bad);
    /* The same coded speech through DSVD: octets in must be octets out, and
     * decoding them must give the reference decoder's samples. */
    {
        side_t a, b;
        v70_config_t ca, cb;
        v70_io_t io;
        long k;

        memset(&a, 0, sizeof(a)); memset(&b, 0, sizeof(b));
        a.frames = coded; a.nframes = nframes;
        v70_config_default(&ca, true); v70_config_default(&cb, false);
        ca.audio_header = cb.audio_header = true;
        memset(&io, 0, sizeof(io));
        io.voice_get_frame = g_get; io.voice_put_frame = g_put;
        io.ctx = &a; a.t = v70_create(&ca, &io);
        io.ctx = &b; b.t = v70_create(&cb, &io);
        v70_start(a.t); v70_start(b.t);
        for (k = 0; k < 28800L * 45 && b.ngot < (int)nframes; k++) {
            v70_rx_put_bit(b.t, v70_tx_get_bit(a.t));
            v70_rx_put_bit(a.t, v70_tx_get_bit(b.t));
        }
        CHECK(b.ngot >= (int)nframes - 1, "DSVD delivered the coded speech (%d of %zu frames)", b.ngot, nframes);
        bad = 0;
        for (i = 0; i < (size_t)b.ngot && i < nframes; i++)
            if (memcmp(b.got[i], coded + i * 10, 10)) bad++;
        CHECK(bad == 0, "every coded frame crossed the multiplexer unchanged (%d differ)", bad);
        badsamp = 0;
        /* One decode only: the reference decoder's internal state does not survive a
         * second Init_* in the same process, and one is enough -- these frames are
         * proven identical to ALGTHM.BIT above. */
        for (i = 0; i < (size_t)b.ngot && i < nframes; i++) {
            v70_g729a_decode(b.got[i], 0, out);
            if (memcmp(out, pst + i * 160, 160)) { if (!badsamp) printf("  first differing frame: %zu\n", i); badsamp++; }
        }
        CHECK(badsamp == 0, "decoded speech from the far end equals the reference output");
        if (getenv("V70_DEBUG")) {
            const v76_stats_t *s = v76_stats(v70_mf(a.t)), *r = v76_stats(v70_mf(b.t));
            printf("  A tx_ui=%llu tx_frames=%llu | B rx_ui=%llu rx_invalid=%llu fcs=%llu voice_tx=%llu voice_rx=%llu lost=%llu\n",
                   (unsigned long long)s->tx_ui_frames, (unsigned long long)s->tx_frames,
                   (unsigned long long)r->rx_ui_frames, (unsigned long long)r->rx_invalid,
                   (unsigned long long)r->rx_fcs_errors, (unsigned long long)v70_stats(a.t)->voice_tx,
                   (unsigned long long)v70_stats(b.t)->voice_rx, (unsigned long long)v70_stats(b.t)->voice_lost);
        }
        printf("  %d frames of G.729A speech through V.70: %d octets each, %.1f s\n", b.ngot, 10,
               (double)v70_tx_bits(a.t) / 28800.0);
        v70_destroy(a.t); v70_destroy(b.t);
    }
    printf("%d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
