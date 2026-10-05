/* Server (k56flex_train) looped back into the client (k56flex_client) through raw G.711:
 * training, peer gates, report, parameter record, priming and data, bit-exact. */
#include "k56flex_channel.h"
#include "k56flex_client.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures;
#define CHECK(c, ...) do { if (!(c)) { ++failures; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

typedef struct {
    k56flex_train_t *server;
    uint16_t sent[4096];
    unsigned nsent, seed;
    uint8_t *rx;
    unsigned nrx, rx_cap;
} loop_t;

static void up_status(void *u, unsigned bits) { k56flex_train_status(((loop_t *)u)->server, bits); }
static void up_report(void *u, unsigned f) { k56flex_train_set_report(((loop_t *)u)->server, f); }
static void down_data(void *u, const uint8_t *bits, unsigned n)
{
    loop_t *l = u;
    unsigned i;
    for (i = 0; i < n && l->nrx < l->rx_cap; ++i) l->rx[l->nrx++] = bits[i];
}

static int src(void *u, uint16_t *w)
{
    loop_t *l = u;
    l->seed = l->seed * 1103515245u + 12345u;
    *w = (uint16_t)(l->seed >> 8);
    if (l->nsent < sizeof(l->sent) / sizeof(l->sent[0])) l->sent[l->nsent++] = *w;
    return 0;
}

static void one(k56flex_law_t law, int rate, uint8_t ext, uint16_t report_word, unsigned data_bits)
{
    k56flex_train_cfg_t tc;
    k56flex_client_cfg_t cc;
    k56flex_train_t server;
    k56flex_client_t *client;
    loop_t l;
    uint8_t buf[160];
    unsigned guard = 0, i;
    const char *tag = law == K56FLEX_LAW_MU ? "mu" : "A";
    memset(&l, 0, sizeof(l));
    l.server = &server;
    l.seed = (unsigned)rate + ext;
    l.rx_cap = data_bits + 2000;
    l.rx = malloc(l.rx_cap);
    memset(&tc, 0, sizeof(tc));
    tc.law = law;
    tc.rate_bps = rate;
    tc.training_word = 0xffff;
    tc.param.extra = 5; tc.param.u = 1; tc.param.control = 0x123;
    CHECK(k56flex_train_init(&server, &tc) == 0, "server init");
    k56flex_train_set_data_source(&server, src, &l);
    memset(&cc, 0, sizeof(cc));
    cc.law = law;
    cc.report_ext = ext;
    cc.report_word = report_word;
    cc.gate_b_pairs = 5;
    cc.ctl.status = up_status;
    cc.ctl.report = up_report;
    cc.ctl.data = down_data;
    cc.ctl.user = &l;
    client = k56flex_client_new(&cc);
    CHECK(client != NULL, "client new");
    if (!client) { free(l.rx); return; }
    /* Zero-latency upstream: advance the server one block at a time so that an event the
     * client raises after a block reaches the server before its next pair completes. */
    while (l.nrx < data_bits && !client->failed && guard++ < 2000000) {
        k56flex_train_phase_t ph = k56flex_train_phase(&server);
        size_t want = (ph == K56T_PRIME || ph == K56T_DATA) ? 8 : 6;
        size_t n = k56flex_train_g711(&server, buf, want);
        if (n != want) { CHECK(0, "%s %d ext %02x: server stalled in %s", tag, rate, ext, k56flex_train_phase_name(ph)); break; }
        k56flex_client_rx(client, buf, n);
    }
    CHECK(!client->failed, "%s %d ext %02x: client failed in %s (nbits %u)", tag, rate, ext,
          k56flex_train_phase_name(k56flex_client_phase(client)), client->nbits);
    if (client->blocks_mismatched) printf("  first mismatch: phase %s pairs %d\n", k56flex_train_phase_name((k56flex_train_phase_t)(client->first_bad / 100000)), client->first_bad % 100000);
    CHECK(client->blocks_mismatched == 0 && client->blocks_checked > 3000, "%s %d ext %02x: fixed stages %u/%u blocks mismatched", tag, rate, ext,
          client->blocks_mismatched, client->blocks_checked);
    CHECK(client->ambiguous_blocks == 0, "ambiguous training blocks %u", client->ambiguous_blocks);
    CHECK(client->param_ok && client->rate_bps == rate && client->param.extra == 5 && client->param.u == 1
          && client->param.v == 0 && client->param.control == 0x123 && client->param.mode == 1,
          "%s %d: record decoded: ok %d rate %d extra %u u %u control %x", tag, rate, client->param_ok,
          client->rate_bps, client->param.extra, client->param.u, client->param.control);
    CHECK(client->prime_frames == 6 && client->prime_ones_bad == 0, "priming: %u frames, %u non-one bits", client->prime_frames, client->prime_ones_bad);
    CHECK(l.nrx >= data_bits, "%s %d ext %02x: only %u data bits", tag, rate, ext, l.nrx);
    {
        unsigned bad = 0, n = l.nrx < data_bits ? l.nrx : data_bits;
        for (i = 0; i < n; ++i) {
            unsigned w = i / 16;
            if (w >= l.nsent || l.rx[i] != ((l.sent[w] >> (i % 16)) & 1)) { ++bad; if (bad == 1) printf("  first bad bit %u\n", i); }
        }
        CHECK(bad == 0, "%s %d ext %02x: %u of %u data bits differ", tag, rate, ext, bad, n);
        if (!bad) printf("  %-2s %5d bit/s report %02x/%04x: training verified (%u blocks), record decoded, %u data bits exact\n",
                         tag, rate, ext, report_word, client->blocks_checked, n);
    }
    k56flex_client_free(client);
    free(l.rx);
}


/* ---- linear audio: server -> G.711 -> D/A levels -> simulated loop -> client codec ------ */

typedef struct {
    k56flex_train_t *server;
    uint64_t server_sym;                 /* symbols the server has produced */
    struct { uint64_t due; int kind; unsigned value; } ev[16];
    unsigned nev;
    unsigned latency;                    /* upstream delay in symbols */
    uint16_t sent[4096];
    unsigned nsent, seed;
    uint8_t *rx;
    unsigned nrx, rx_cap;
} lin_t;

static void lin_queue(lin_t *l, int kind, unsigned value)
{
    if (l->nev < 16) {
        l->ev[l->nev].due = l->server_sym + l->latency;
        l->ev[l->nev].kind = kind;
        l->ev[l->nev].value = value;
        ++l->nev;
    }
}
static void lin_status(void *u, unsigned bits) { lin_queue(u, 0, bits); }
static void lin_report(void *u, unsigned f) { lin_queue(u, 1, f); }
static void lin_deliver(lin_t *l)
{
    unsigned i, keep = 0;
    for (i = 0; i < l->nev; ++i) {
        if (l->ev[i].due <= l->server_sym) {
            if (l->ev[i].kind == 0) k56flex_train_status(l->server, l->ev[i].value);
            else k56flex_train_set_report(l->server, l->ev[i].value);
        } else {
            l->ev[keep++] = l->ev[i];
        }
    }
    l->nev = keep;
}
static void lin_data(void *u, const uint8_t *bits, unsigned n)
{
    lin_t *l = u;
    unsigned i;
    for (i = 0; i < n && l->nrx < l->rx_cap; ++i) l->rx[l->nrx++] = bits[i];
}
static int lin_src(void *u, uint16_t *w)
{
    lin_t *l = u;
    l->seed = l->seed * 1103515245u + 12345u;
    *w = (uint16_t)(l->seed >> 8);
    if (l->nsent < sizeof(l->sent) / sizeof(l->sent[0])) l->sent[l->nsent++] = *w;
    return 0;
}

static void linear(const char *name, k56flex_law_t law, int rate, uint8_t ext, uint16_t report_word,
                   const k56flex_channel_cfg_t *chcfg, unsigned latency, unsigned data_bits, int data_exact)
{
    k56flex_train_cfg_t tc;
    k56flex_client_cfg_t cc;
    k56flex_train_t server;
    k56flex_client_t *client;
    k56flex_channel_t *ch = k56flex_channel_new(chcfg);
    lin_t *l = calloc(1, sizeof(*l));
    uint8_t buf[8];
    unsigned guard = 0, i, bad = 0, n;
    l->server = &server;
    l->latency = latency;
    l->seed = (unsigned)rate * 7 + ext;
    l->rx_cap = data_bits + 2000;
    l->rx = malloc(l->rx_cap);
    memset(&tc, 0, sizeof(tc));
    tc.law = law;
    tc.rate_bps = rate;
    tc.training_word = 0xffff;
    tc.param.extra = 5; tc.param.u = 1; tc.param.control = 0x123;
    CHECK(k56flex_train_init(&server, &tc) == 0, "server init");
    k56flex_train_set_data_source(&server, lin_src, l);
    memset(&cc, 0, sizeof(cc));
    cc.law = law;
    cc.report_ext = ext;
    cc.report_word = report_word;
    cc.gate_b_pairs = 5;
    cc.ctl.status = lin_status;
    cc.ctl.report = lin_report;
    cc.ctl.data = lin_data;
    cc.ctl.user = l;
    client = k56flex_client_new(&cc);
    while ((data_exact ? l->nrx < data_bits : !client->param_ok) && !client->failed && guard++ < 3000000) {
        k56flex_train_phase_t ph = k56flex_train_phase(&server);
        size_t want = (ph == K56T_PRIME || ph == K56T_DATA) ? 8 : 6, k;
        lin_deliver(l);
        if (k56flex_train_g711(&server, buf, want) != want) { CHECK(0, "%s: server stalled in %s", name, k56flex_train_phase_name(ph)); break; }
        l->server_sym += want;
        for (k = 0; k < want; ++k) {
            int16_t out[2];
            unsigned m = k56flex_channel_push(ch, (double)k56flex_level_from_g711(law, buf[k]), out);
            k56flex_client_rx_linear(client, out, m);
        }
    }
    CHECK(client->fe && k56flex_rxfe_locked(client->fe), "%s: front end never locked", name);
    CHECK(!client->failed, "%s: client failed in %s (nbits %u, snr %.1f dB, ppm %.1f)", name,
          k56flex_train_phase_name(k56flex_client_phase(client)), client->nbits,
          client->fe ? k56flex_rxfe_snr_db(client->fe) : 0.0, client->fe ? k56flex_rxfe_ppm(client->fe) : 0.0);
    CHECK(client->param_ok && client->rate_bps == rate && client->param.extra == 5 && client->param.control == 0x123,
          "%s: record not decoded (ok %d rate %d)", name, client->param_ok, client->rate_bps);
    if (data_exact) {
        CHECK(client->prime_frames == 6 && client->prime_ones_bad == 0, "%s: priming %u frames, %u non-one bits", name, client->prime_frames, client->prime_ones_bad);
        n = l->nrx < data_bits ? l->nrx : data_bits;
        for (i = 0; i < n; ++i)
            if (i / 16 >= l->nsent || l->rx[i] != ((l->sent[i / 16] >> (i % 16)) & 1)) ++bad;
        CHECK(l->nrx >= data_bits && bad == 0, "%s: %u data bits, %u wrong", name, l->nrx, bad);
    }
    if (client->fe && client->param_ok && (!data_exact || (bad == 0 && l->nrx >= data_bits)))
        printf("  %-38s lock %u, gain %.3f, eq SNR %.1f dB: %s\n", name, k56flex_rxfe_locked_at(client->fe),
               k56flex_rxfe_gain(client->fe), k56flex_rxfe_snr_db(client->fe),
               data_exact ? "record decoded, priming read, data bits exact" : "record decoded through the gates");
    k56flex_client_free(client);
    k56flex_channel_free(ch);
    free(l->rx);
    free(l);
}

static void negative(void)
{
    k56flex_train_cfg_t tc;
    k56flex_client_cfg_t cc;
    k56flex_train_t server;
    k56flex_client_t *client;
    loop_t l;
    uint8_t buf[160];
    unsigned guard = 0, flipped = 0, bad = 0, i;
    memset(&l, 0, sizeof(l));
    l.server = &server;
    l.seed = 99;
    l.rx_cap = 12000;
    l.rx = malloc(l.rx_cap);
    memset(&tc, 0, sizeof(tc));
    tc.law = K56FLEX_LAW_MU;
    tc.rate_bps = 56000;
    tc.training_word = 0xffff;
    k56flex_train_init(&server, &tc);
    k56flex_train_set_data_source(&server, src, &l);
    memset(&cc, 0, sizeof(cc));
    cc.law = K56FLEX_LAW_MU;
    cc.report_word = 0x8880;
    cc.gate_b_pairs = 5;
    cc.ctl.status = up_status; cc.ctl.report = up_report; cc.ctl.data = down_data; cc.ctl.user = &l;
    client = k56flex_client_new(&cc);
    while (l.nrx < 10000 && !client->failed && guard++ < 2000000) {
        k56flex_train_phase_t ph = k56flex_train_phase(&server);
        size_t want = (ph == K56T_PRIME || ph == K56T_DATA) ? 8 : 6;
        k56flex_train_g711(&server, buf, want);
        if (ph == K56T_DATA && client->data_frames == 40 && !flipped) { buf[3] ^= 0x04; flipped = 1; }
        k56flex_client_rx(client, buf, want);
    }
    CHECK(flipped, "corruption injected");
    for (i = 0; i < l.nrx; ++i)
        if (i / 16 < l.nsent && l.rx[i] != ((l.sent[i / 16] >> (i % 16)) & 1)) ++bad;
    CHECK(bad > 0 || client->failed, "one corrupted data octet is detected (%u bad bits, failed %d)", bad, client->failed);
    printf("  corrupted octet in data frame 40: %u bit errors%s (self-synchronising descrambler)\n", bad, client->failed ? ", client rejected the frame" : "");
    k56flex_client_free(client);
    free(l.rx);
}

int main(void)
{
    {
        /* gain, delay, lowpass corner, highpass corner, dc, noise rms, clock ppm, seed */
        k56flex_channel_cfg_t a = {0.35, 0.0, 0, 0, 0, 0, 0, 1};
        k56flex_channel_cfg_t b = {0.5, 3.0, 1500, 100, 20, 1.5, 0, 2};
        k56flex_channel_cfg_t c = {0.45, 4.0, 800, 100, 0, 1.5, 0, 3};
        k56flex_channel_cfg_t d = {0.5, 5.7, 1500, 100, 0, 2, 3, 4};
        linear("ideal wire, pad 9 dB, mu 56k", K56FLEX_LAW_MU, 56000, 0x00, 0x8880, &a, 0, 12000, 1);
        linear("RC loop + HP + noise, 30 sym latency, A 32k", K56FLEX_LAW_A, 32000, 0x03, 0x8890, &b, 30, 12000, 1);
        linear("heavy roll-off (edge -20 dB), training only", K56FLEX_LAW_MU, 32000, 0x00, 0x8880, &c, 17, 0, 0);
        linear("fractional delay 5.7, +3 ppm (training only)", K56FLEX_LAW_MU, 32000, 0x00, 0x8880, &d, 25, 0, 0);
    }
    negative();
    one(K56FLEX_LAW_MU, 56000, 0x00, 0x8880, 20000);
    one(K56FLEX_LAW_A, 56000, 0x00, 0x8880, 20000);
    one(K56FLEX_LAW_MU, 32000, 0x00, 0x8890, 8000);          /* pacing bit: 8 pairs of 2-bit record */
    one(K56FLEX_LAW_MU, 44000, 0x03, 0x8880, 20000);         /* RBS positions 0,1: selector 2 */
    one(K56FLEX_LAW_A, 48000, 0x15, 0x8890, 20000);          /* three positions, pacing */
    one(K56FLEX_LAW_MU, 40000, 0x40, 0x8880, 10000);         /* table-select bit 6 */
    one(K56FLEX_LAW_A, 52000, 0x80 | 0x0f, 0x8890, 10000);   /* table-select bit 7, four positions */
    printf(failures ? "k56flex_client_test: %d FAILURES\n" : "k56flex_client_test: all passed\n", failures);
    return failures != 0;
}
