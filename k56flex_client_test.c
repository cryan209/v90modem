/* K56flex downstream prefix over G.711/linear audio only.
 * Draft 0.23 4.10-4.12/11: an unanswered upstream gate must not be released
 * by receiver callbacks. Upstream waveform generation is not implemented. */
#include "k56flex_channel.h"
#include "k56flex_client.h"

#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(c, ...) do { if (!(c)) { ++failures; printf("FAIL: "); \
    printf(__VA_ARGS__); puts(""); } } while (0)

static void data(void *user, const uint8_t *bits, unsigned n)
{
    (void)bits;
    *(unsigned *)user += n;
}

static int source(void *user, uint16_t *word)
{
    ++*(unsigned *)user;
    *word = 0x1234;
    return 0;
}

static void prefix(k56flex_law_t law, int rate, int linear)
{
    k56flex_train_cfg_t tc = {0};
    k56flex_client_cfg_t cc = {0};
    k56flex_train_t server;
    k56flex_channel_cfg_t cfg = {0.35, 0, 0, 0, 0, 0, 0, 1};
    k56flex_channel_t *channel = linear ? k56flex_channel_new(&cfg) : NULL;
    unsigned received = 0, sourced = 0, samples = 0;
    int reached_gate = 0;
    tc.law = law;
    tc.rate_bps = rate;
    tc.training_word = 0xffff;
    tc.gate_timeout_pairs = 1000;
    CHECK(!k56flex_train_init(&server, &tc), "server init");
    k56flex_train_set_data_source(&server, source, &sourced);
    cc.law = law;
    cc.report_word = 0x8880;
    cc.gate_b_pairs = 5;
    cc.ctl.data = data; /* Host payload sink only, no reference to the peer. */
    cc.ctl.user = &received;
    k56flex_client_t *client = k56flex_client_new(&cc);
    CHECK(client && (!linear || channel), "receiver/channel allocation");
    if (!client || (linear && !channel)) goto done;
    while (server.phase != K56T_FAILED && samples < 8*8000) {
        uint8_t octets[6];
        size_t n = k56flex_train_g711(&server, octets, sizeof(octets));
        CHECK(n == sizeof(octets), "prefix sample accounting");
        if (n != sizeof(octets)) break;
        samples += (unsigned)n;
        if (linear) {
            for (size_t i = 0; i < n; i++) {
                int16_t pcm[2];
                unsigned count = k56flex_channel_push(channel,
                    k56flex_level_from_g711(law, octets[i]), pcm);
                k56flex_client_rx_linear(client, pcm, count);
            }
        } else if (!reached_gate) {
            /* The raw-octet decoder's supported prefix ends at this gate.
             * Continue TX below without fabricating the missing upstream. */
            k56flex_client_rx(client, octets, n);
        }
        if (!reached_gate && server.phase == K56T_GATE_A) {
            reached_gate = 1;
            if (!linear)
                CHECK(!client->blocks_mismatched && client->blocks_checked > 3000,
                      "G.711 fixed prefix must match");
        }
        CHECK(server.phase <= K56T_GATE_A || server.phase == K56T_FAILED,
              "unanswered peer gate advanced to %s", k56flex_train_phase_name(server.phase));
    }
    CHECK(reached_gate && server.phase == K56T_FAILED, "unanswered gate must time out");
    CHECK(!server.status && !server.cfg.report_field, "no peer status/report injected");
    CHECK(!server.data_ready && !sourced && !received && !client->param_ok,
          "no synthetic DATA or parameter completion");
    CHECK(!client->failed, "prefix receiver failed");
    if (linear) CHECK(client->fe_started, "linear receiver must acquire P1");
    printf("%s %d %s: prefix received; unanswered upstream gate timed out; no DATA\n",
           law == K56FLEX_LAW_MU ? "mu" : "A", rate, linear ? "audio" : "G.711");
done:
    k56flex_client_free(client);
    k56flex_channel_free(channel);
}

/* The 7910 (Lucent) on our PCMU leg: its PT_C signs follow neither our
 * sequence state nor our DC, so the client reads PT_C by magnitude, which
 * yields the card's short (mode 0) parameter record, and finds priming by
 * content (k56flex_client.c blind_symbol).  Fixture cut from a live call with
 * our report 00/89b0: record 6374 9fff 0000 then the final e374, A-law mapper
 * at 52000 through the A->mu transcode, priming at 13.72 s, then V.42 ODP and
 * HDLC flags. */
static uint8_t blind_bits[65536];
static unsigned blind_nbits;

static void blind_data(void *user, const uint8_t *bits, unsigned n)
{
    (void)user;
    for (unsigned i = 0; i < n && blind_nbits < sizeof(blind_bits); i++) blind_bits[blind_nbits++] = bits[i];
}

static void blind_7910(const char *path)
{
    k56flex_client_cfg_t cc = {0};
    k56flex_client_t *c;
    FILE *f = fopen(path, "rb");
    unsigned flags = 0, i, shift = 0;
    int b;
    CHECK(f != NULL, "open %s", path);
    if (!f) return;
    cc.law = K56FLEX_LAW_A;
    cc.report_word = 0x89b0;
    cc.gate_b_pairs = 2;
    cc.codec_identity = 1;
    cc.ctl.data = blind_data;
    c = k56flex_client_new(&cc);
    while ((b = fgetc(f)) != EOF && !c->failed) {
        int16_t x = (int16_t)k56flex_level_from_g711(K56FLEX_LAW_MU, (uint8_t)b);
        k56flex_client_rx_linear(c, &x, 1);
    }
    fclose(f);
    for (i = 0; i < blind_nbits; i++) {
        shift = ((shift << 1) | blind_bits[i]) & 0xff;
        if (i >= 7 && shift == 0x7e) ++flags;
    }
    CHECK(c->param_ok && c->final_param_seen && c->param.mode == 0 && c->param.rate == 13 &&
          c->param.control == 0x1fff && c->param.bit == 1 && c->param.u && c->param.v,
          "7910 short parameter record: ok %d final %d mode %u rate %u control %04x",
          c->param_ok, c->final_param_seen, c->param.mode, c->param.rate, c->param.control);
    CHECK(c->blind == 3 && c->rate_bps == 52000, "7910 priming by content: blind %d rate %d", c->blind, c->rate_bps);
    CHECK(c->data_frames >= 770 && !c->failed, "7910 data frames %u failed %d", c->data_frames, c->failed);
    CHECK(flags >= 200, "7910 data carries HDLC flags: %u", flags);
    printf("7910 capture: record %04x final, %u PT blocks by magnitude, blind priming at %d bps, "
           "%u data frames, %u flags, %u derived signs off our DC\n",
           c->param.control, c->abs_blocks, c->rate_bps, c->data_frames, flags, c->rx.derived_sign_misses);
    k56flex_client_free(c);
}

int main(void)
{
    for (int law = 0; law < 2; law++)
        for (int linear = 0; linear < 2; linear++) {
            prefix((k56flex_law_t)law, 32000, linear);
            prefix((k56flex_law_t)law, 56000, linear);
        }
    blind_7910("reference-data/k56flex-7910/blind-data-ulaw.g711");
    printf("k56flex_client_test: %d failures\n", failures);
    return failures != 0;
}
