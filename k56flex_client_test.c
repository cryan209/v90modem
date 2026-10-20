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

int main(void)
{
    for (int law = 0; law < 2; law++)
        for (int linear = 0; linear < 2; linear++) {
            prefix((k56flex_law_t)law, 32000, linear);
            prefix((k56flex_law_t)law, 56000, linear);
        }
    printf("k56flex_client_test: %d failures (prefix/gate checks only)\n", failures);
    return failures != 0;
}
