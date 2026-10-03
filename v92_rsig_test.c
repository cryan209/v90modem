/*
 * v92_rsig_test.c — Rf (V.92 8.8.4) and the R-family detector, both laws.
 */
#include "v92_rsig.h"
#include "v91.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t prng = 0x92F00DU;
static uint32_t rnd(void) { prng = prng * 1664525U + 1013904223U; return prng >> 8; }

static uint8_t random_cw(int law, int max_ucode)
{
    return v91_ucode_to_codeword((v91_law_t)law, (int)(rnd() % (unsigned)(max_ucode + 1)), rnd() & 1);
}

static void test_rf(int law, int lead)
{
    vpcm_cp_frame_t cp;
    uint8_t uc[6], cw[1200];
    int n = 0, rf_start, bar_start;
    v92_rsig_rx_t rx;
    v92_rsig_event_t got_r = V92_RSIG_EV_NONE, got_bar = V92_RSIG_EV_NONE;

    vpcm_cp_init(&cp);
    cp.constellation_count = 2;
    for (int u = 10; u <= 60; u++) vpcm_cp_mask_set(cp.masks[0], u, true);
    for (int u = 5; u <= 70; u += 5) vpcm_cp_mask_set(cp.masks[1], u, true);
    for (int i = 0; i < 6; i++) cp.dfi[i] = (uint8_t)(i & 1);
    assert(v92_rf_ucodes(&cp, uc));
    for (int i = 0; i < 6; i++) assert(uc[i] == ((i & 1) ? 70 : 60));

    for (int i = 0; i < lead; i++) cw[n++] = random_cw(law, 70);
    rf_start = n;
    v92_rf_codewords(law, uc, false, 0, cw + n, 384); n += 384;
    bar_start = n;
    v92_rf_codewords(law, uc, true, 384, cw + n, 24); n += 24;
    for (int i = 0; i < 300; i++) cw[n++] = random_cw(law, 70);

    /* What went out: the interval's top Ucode, signs + + - - (bar - - + +). */
    for (int k = 0; k < 408; k++) {
        int lin = v91_codeword_to_linear((v91_law_t)law, cw[rf_start + k]);
        int ref = v91_codeword_to_linear((v91_law_t)law,
                                         v91_ucode_to_codeword((v91_law_t)law, uc[k % 6], true));
        bool pos = (k % 4) < 2;
        if (k >= 384) pos = !pos;
        assert((pos ? lin : -lin) == ref);
    }

    v92_rsig_rx_init(&rx, 0);
    for (int i = 0; i < n; i++) {
        v92_rsig_event_t e = v92_rsig_rx_put(&rx, v91_codeword_to_linear((v91_law_t)law, cw[i]));
        if (e == V92_RSIG_EV_R) got_r = e;
        if (e == V92_RSIG_EV_BAR) got_bar = e;
    }
    assert(got_r == V92_RSIG_EV_R && rx.kind == V92_RSIG_P4);
    assert(got_bar == V92_RSIG_EV_BAR);
    /* A few random symbols just before Rf may happen to match and join the
     * run, so the lock can land a little early -- never before Rf. */
    assert(rx.r_at < (uint64_t)bar_start && rx.r_at >= (uint64_t)rf_start + 40);
    if ((int)rx.bar_at != bar_start)
        printf("  bar at %llu, expected %d (lead %d)\n", (unsigned long long)rx.bar_at, bar_start, lead);
    assert((int)rx.bar_at == bar_start);
}

static void test_r6(int law)
{
    uint8_t cw[900];
    int n = 0;
    v92_rsig_rx_t rx;
    static const int p[6] = {1,1,1,-1,-1,-1};

    for (int i = 0; i < 97; i++) cw[n++] = random_cw(law, 80);
    for (int k = 0; k < 384 + 24; k++) {
        bool pos = p[k % 6] > 0;
        if (k >= 384) pos = !pos;
        cw[n++] = v91_ucode_to_codeword((v91_law_t)law, 78, pos);
    }
    for (int i = 0; i < 300; i++) cw[n++] = random_cw(law, 80);
    v92_rsig_rx_init(&rx, 0);
    for (int i = 0; i < n; i++)
        (void)v92_rsig_rx_put(&rx, v91_codeword_to_linear((v91_law_t)law, cw[i]));
    assert(rx.kind == V92_RSIG_P6 && rx.bar_seen && rx.bar_at == 97 + 384);
}

static void test_no_false(int law)
{
    v92_rsig_rx_t rx;
    int events = 0;

    v92_rsig_rx_init(&rx, 0);
    for (int i = 0; i < 200000; i++)
        events += v92_rsig_rx_put(&rx, v91_codeword_to_linear((v91_law_t)law, random_cw(law, 127))) != V92_RSIG_EV_NONE;
    assert(events == 0);
    /* Locked on an R that is never followed by its bar: data must not
     * fake one. */
    v92_rsig_rx_init(&rx, 0);
    for (int k = 0; k < 120; k++)
        (void)v92_rsig_rx_put(&rx, ((k % 4) < 2) ? 3000 : -3000);
    assert(rx.kind == V92_RSIG_P4);
    for (int i = 0; i < 200000; i++)
        assert(v92_rsig_rx_put(&rx, v91_codeword_to_linear((v91_law_t)law, random_cw(law, 127))) == V92_RSIG_EV_NONE);
}

int main(void)
{
    for (int law = 0; law < 2; law++) {
        for (int lead = 0; lead < 24; lead++)
            test_rf(law, 97 + lead);
        test_r6(law);
        test_no_false(law);
        printf("PASS: %s Rf/R-bar-f per 8.8.4 (top Ucode per interval, + + - -), detected as period 4 "
               "with the bar to the symbol at 24 offsets; Rd period 6; no detection in 200000 random symbols\n",
               law ? "A-law" : "u-law");
    }
    puts("v92_rsig_test: all passed");
    return 0;
}
