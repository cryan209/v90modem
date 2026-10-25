/* The client receive model on a real server's P1: the Eicon/MultiTech 7910
 * card, captured on our PCMU leg (reference-data/k56flex-7910/p1-ulaw.*).
 * The card's P1 is our A-law P1 converted A->mu with every sign inverted
 * (Draft 0.23 4.10; docs/k56flex_implementation.md, 2026-10-25), so the front
 * end must acquire either polarity and the decisions must check the whole
 * probe, with either law's model.  A sign-flipped copy is the control: the
 * same P1 at normal polarity.  The card leaves our timeline after P1, so
 * nothing past the P1/P2 boundary is asserted. */
#include "k56flex_client.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures;
#define CHECK(c, ...) do { if (!(c)) { ++failures; printf("FAIL: "); \
    printf(__VA_ARGS__); puts(""); } } while (0)

static void run(const uint8_t *octets, size_t n, k56flex_law_t server_law, int flip)
{
    k56flex_client_cfg_t cc = {0};
    k56flex_client_t *c;
    unsigned checked = 0, mismatched = 0;
    float snr = 0;
    int left_p1 = 0;
    size_t i;
    cc.law = server_law;
    cc.report_word = 0x8880;
    cc.gate_b_pairs = 2;
    c = k56flex_client_new(&cc);
    CHECK(c != NULL, "client");
    if (!c) return;
    for (i = 0; i < n && !c->failed; ++i) {
        /* the line is PCMU whatever the server's law */
        int16_t x = (int16_t)k56flex_level_from_g711(K56FLEX_LAW_MU, octets[i] ^ (flip ? 0x80 : 0));
        k56flex_client_rx_linear(c, &x, 1);
        if (c->fe_started && !left_p1 && k56flex_client_phase(c) > K56T_P1) {
            left_p1 = 1;
            checked = c->blocks_checked;
            mismatched = c->blocks_mismatched;
            snr = k56flex_rxfe_snr_db(c->fe);
        }
    }
    printf("%s model, %s polarity: locked=%d gain=%.4f P1 blocks %u mismatched %u, SNR at P1 end %.1f dB\n",
           server_law == K56FLEX_LAW_A ? "A-law" : "mu-law", flip ? "flipped" : "card",
           c->fe_started, c->fe ? k56flex_rxfe_gain(c->fe) : 0.0f, checked, mismatched, snr);
    CHECK(c->fe_started, "P1 acquired");
    CHECK(c->fe && (k56flex_rxfe_gain(c->fe) < 0) == !flip, "gain sign carries the polarity");
    CHECK(left_p1, "model reached the end of P1");
    /* 1364 pairs = 2728 blocks; acquisition starts the check a few blocks in */
    CHECK(checked >= 2600, "P1 blocks checked (%u)", checked);
    CHECK(mismatched == 0, "P1 blocks mismatched (%u)", mismatched);
    CHECK(snr > 25, "equalized P1 SNR (%.1f dB)", snr);
    k56flex_client_free(c);
}

int main(int argc, char **argv)
{
    const char *path = argc > 1 ? argv[1] : "reference-data/k56flex-7910/p1-ulaw.g711";
    FILE *f = fopen(path, "rb");
    uint8_t *buf = malloc(1 << 16);
    size_t n;
    if (!f || !buf) { printf("FAIL: open %s\n", path); return 1; }
    n = fread(buf, 1, 1 << 16, f);
    fclose(f);
    run(buf, n, K56FLEX_LAW_MU, 0);
    run(buf, n, K56FLEX_LAW_A, 0);
    run(buf, n, K56FLEX_LAW_MU, 1);
    run(buf, n, K56FLEX_LAW_A, 1);
    free(buf);
    printf("%s\n", failures ? "k56flex_7910_p1_test FAILED" : "k56flex_7910_p1_test passed");
    return failures != 0;
}
