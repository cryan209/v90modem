#include <math.h>
#include <stdlib.h>
/* Loop an initiator against a responder through 8 kHz linear samples. */
#include "k56flex_v8bis.h"

#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(c, ...) do { if (!(c)) { ++failures; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static void run(k56flex_v8bis_t *ini, k56flex_v8bis_t *rsp, unsigned seconds, int swap_gain)
{
    int16_t a[160], b[160], za[160] = {0};
    unsigned i, n = seconds * 50;
    unsigned seed = 12345;
    for (i = 0; i < n; ++i) {
        if (ini) k56flex_v8bis_tx(ini, a, 160); else memcpy(a, za, sizeof(a));
        if (rsp) k56flex_v8bis_tx(rsp, b, 160); else memcpy(b, za, sizeof(b));
        if (swap_gain) {
            int k;
            for (k = 0; k < 160; ++k) {
                seed = seed * 1103515245u + 12345u; a[k] = (int16_t)(a[k] / 3 + (int)((seed >> 16) % 17) - 8);
                seed = seed * 1103515245u + 12345u; b[k] = (int16_t)(b[k] / 3 + (int)((seed >> 16) % 17) - 8);
            }
        }
        if (ini) k56flex_v8bis_rx(ini, b, 160);
        if (rsp) k56flex_v8bis_rx(rsp, a, 160);
    }
}

/* A CRe a gateway mutes early (the 7910 path: 23-32 ms of 1375+2002 Hz,
 * then digital silence), at every 5-sample phase against the responder's
 * block clock.  Returns how many phases detected it. */
static int clipped_cre_phases(unsigned tone_samples)
{
    int hits = 0;
    for (unsigned phase = 0; phase < 160; phase += 5) {
        k56flex_v8bis_cfg_t c = {K56FLEX_V8BIS_RESPOND, 0, 0, 1, 0, 0, 0, 0};
        k56flex_v8bis_t *s = k56flex_v8bis_new(&c);
        int16_t tx[160], rx[160];
        unsigned t = 0, on = 8000 + phase;
        for (int blk = 0; blk < 150 && k56flex_v8bis_state(s) == K56V8B_IDLE_WAIT; ++blk) {
            k56flex_v8bis_tx(s, tx, 160);
            for (int k = 0; k < 160; ++k, ++t)
                rx[k] = (t >= on && t < on + tone_samples)
                      ? (int16_t) (1800.0*sin(2*M_PI*1375.0*t/8000) + 1800.0*sin(2*M_PI*2002.0*t/8000)) : 0;
            k56flex_v8bis_rx(s, rx, 160);
        }
        hits += k56flex_v8bis_state(s) == K56V8B_DELAY_TX;
        k56flex_v8bis_free(s);
    }
    return hits;
}

int main(void)
{
    k56flex_v8bis_cfg_t ic = {K56FLEX_V8BIS_INITIATE, 1, 1, 1, 0, 0, 0, 0}, rc = {K56FLEX_V8BIS_RESPOND, 1, 1, 0, 0, 0, 0, 0};
    k56flex_v8bis_t *ini, *rsp;
    const uint8_t *p;
    size_t n;
    uint8_t cl[16], rcl[18];

    /* Detection of a clipped CRe does not depend on its phase. */
    {
        int all = 160/5, h;
        setenv("K56FLEX_DETECT_MS", "20", 1);
        h = clipped_cre_phases(184);                 /* 23 ms */
        CHECK(h == all, "23 ms CRe at 20 ms acceptance: %d/%d phases", h, all);
        unsetenv("K56FLEX_DETECT_MS");
        setenv("K56FLEX_DETECT_BLOCKS", "2", 1);     /* the earlier recipe's form: 20 ms */
        h = clipped_cre_phases(184);
        CHECK(h == all, "23 ms CRe with K56FLEX_DETECT_BLOCKS=2: %d/%d phases", h, all);
        unsetenv("K56FLEX_DETECT_BLOCKS");
        h = clipped_cre_phases(184);
        CHECK(h == 0, "23 ms CRe at the default 120 ms acceptance: %d/%d phases", h, all);
        h = clipped_cre_phases(1000);                /* 125 ms */
        CHECK(h == all, "125 ms CRe at the default acceptance: %d/%d phases", h, all);
    }

    /* Predicate on recovered messages, before any audio. */
    {
        uint8_t m[18];
        k56flex_v8bis_rockwell_client_payload(K56FLEX_V8BIS_CL, m);
        CHECK(k56flex_v8bis_ns_offset(m, 18) == 8 && k56flex_v8bis_flex_version(m, 18) == 0x03,
              "Rockwell CL: NS at %d, version %d", k56flex_v8bis_ns_offset(m, 18), k56flex_v8bis_flex_version(m, 18));
        CHECK(k56flex_v8bis_peer_acceptable(m, 18), "Rockwell CL offer 0x03 accepted");
        k56flex_v8bis_rockwell_client_payload(K56FLEX_V8BIS_MS, m);
        CHECK(k56flex_v8bis_peer_acceptable(m, 18), "Rockwell MS reply 0x02 accepted");
        m[15] = 0x01;
        CHECK(!k56flex_v8bis_peer_acceptable(m, 18), "version 0x01 rejected");
        m[15] = 0x43;
        CHECK(!k56flex_v8bis_peer_acceptable(m, 18), "server version 0x43 rejected");
        k56flex_v8bis_rockwell_client_payload(K56FLEX_V8BIS_CL, m);
        m[1] = 0x81;                     /* HSF 7.12: NS-present bit never set */
        CHECK(k56flex_v8bis_flex_version(m, 18) < 0 && !k56flex_v8bis_peer_acceptable(m, 18),
              "message without NS-present bit rejected");
        k56flex_v8bis_rockwell_client_payload(K56FLEX_V8BIS_CL, m);
        for (n = 0; n < sizeof(m); ++n) {
            CHECK(k56flex_v8bis_ns_offset(m, n) < 0,
                  "Rockwell NS truncated at %zu rejected", n);
            CHECK(!k56flex_v8bis_peer_acceptable(m, n),
                  "Rockwell peer truncated at %zu rejected", n);
        }
        m[8] = 10;                      /* declared NS extends past the frame */
        CHECK(!k56flex_v8bis_peer_acceptable(m, sizeof(m)), "oversized NS rejected");
        m[8] = 7;
        m[10] = 3;                      /* version byte now lies outside NS */
        CHECK(!k56flex_v8bis_peer_acceptable(m, sizeof(m)), "version outside NS rejected");
        m[8] = 9;
        m[10] = 8;                      /* provider length crosses NS boundary */
        CHECK(!k56flex_v8bis_peer_acceptable(m, sizeof(m)), "oversized provider rejected");

        k56flex_v8bis_payload(K56FLEX_V8BIS_CL, 0, 1, cl);
        CHECK(k56flex_v8bis_ns_offset(cl, 16) == 6 && k56flex_v8bis_flex_version(cl, 16) == 0x42,
              "MICA template: NS at 6, version 0x42");
        CHECK(!k56flex_v8bis_peer_acceptable(cl, 16), "server template rejected as a client");
        cl[13] = 0x02;
        CHECK(k56flex_v8bis_peer_acceptable(cl, 16), "MICA-layout 0x02 still accepted");
        for (n = 0; n < sizeof(cl); ++n)
            CHECK(!k56flex_v8bis_peer_acceptable(cl, n),
                  "MICA peer truncated at %zu rejected", n);
        cl[13] = 0x21;
        CHECK(k56flex_v8bis_peer_acceptable(cl, 16), "MICA-layout 0x21 still accepted");
    }
    /* The MultiTech MT5634MU's MS to our server (live, 27 October): a 7-octet
       NS block with the version octet 0x82 last, 13 octets in all. */
    {
        static const uint8_t lucent[13] = {0x11, 0xc9, 0x80, 0x80, 0x80, 0x07, 0xb5, 0x02, 0x00, 0x94, 0x01, 0x83, 0x82};
        CHECK(k56flex_v8bis_flex_version(lucent, sizeof(lucent)) == 0x82, "Lucent MS version %d",
              k56flex_v8bis_flex_version(lucent, sizeof(lucent)));
        CHECK(k56flex_v8bis_peer_acceptable(lucent, sizeof(lucent)), "Lucent client MS accepted");
        for (n = 0; n < sizeof(lucent); ++n)
            CHECK(!k56flex_v8bis_peer_acceptable(lucent, n), "Lucent MS truncated at %zu rejected", n);
    }

    /* Full exchange: the initiator stands in for a Rockwell client (CL offer 0x03). */
    ini = k56flex_v8bis_new(&ic);
    rsp = k56flex_v8bis_new(&rc);
    run(ini, rsp, 12, 0);
    CHECK(k56flex_v8bis_state(rsp) == K56V8B_DONE, "responder state %s (%s)",
          k56flex_v8bis_state_name(k56flex_v8bis_state(rsp)), k56flex_v8bis_result_name(k56flex_v8bis_result(rsp)));
    CHECK(k56flex_v8bis_state(ini) == K56V8B_DONE, "initiator state %s (%s)",
          k56flex_v8bis_state_name(k56flex_v8bis_state(ini)), k56flex_v8bis_result_name(k56flex_v8bis_result(ini)));
    p = k56flex_v8bis_peer_payload(rsp, &n);
    k56flex_v8bis_rockwell_client_payload(K56FLEX_V8BIS_CL, rcl);
    CHECK(p && n == 18 && !memcmp(p, rcl, 18), "responder saw the Rockwell client CL");
    p = k56flex_v8bis_peer_payload(ini, &n);
    k56flex_v8bis_payload(K56FLEX_V8BIS_MS, 1, 1, cl);
    CHECK(p && n == 16 && !memcmp(p, cl, 16), "initiator saw the server MS (v90, mu-law edits)");
    CHECK(k56flex_v8bis_ack(rsp) == 1 && k56flex_v8bis_ack(ini) == 1, "ACK1 delivered %d/%d", k56flex_v8bis_ack(rsp), k56flex_v8bis_ack(ini));
    CHECK(k56flex_v8bis_result(rsp) == K56V8B_OK && k56flex_v8bis_result(ini) == K56V8B_OK, "results");
    k56flex_v8bis_free(ini);
    k56flex_v8bis_free(rsp);

    /* The lab case: v90modem (digital server) dials, the analogue client answers. */
    {
        k56flex_v8bis_cfg_t si = {K56FLEX_V8BIS_INITIATE, 1, 0, 0, 0, 0, 0, 0}, cr = {K56FLEX_V8BIS_RESPOND, 1, 0, 1, 0, 0, 0, 0};
        ini = k56flex_v8bis_new(&si);
        rsp = k56flex_v8bis_new(&cr);
        run(ini, rsp, 12, 0);
        CHECK(k56flex_v8bis_result(ini) == K56V8B_OK && k56flex_v8bis_state(ini) == K56V8B_DONE
              && k56flex_v8bis_result(rsp) == K56V8B_OK && k56flex_v8bis_state(rsp) == K56V8B_DONE,
              "server-initiated exchange: %s(%s) / %s(%s)", k56flex_v8bis_state_name(k56flex_v8bis_state(ini)),
              k56flex_v8bis_result_name(k56flex_v8bis_result(ini)), k56flex_v8bis_state_name(k56flex_v8bis_state(rsp)),
              k56flex_v8bis_result_name(k56flex_v8bis_result(rsp)));
        p = k56flex_v8bis_peer_payload(ini, &n);
        CHECK(p && n == 18 && k56flex_v8bis_flex_version(p, n) == 0x02 && p[0] == 0x11, "server saw the client MS");
        p = k56flex_v8bis_peer_payload(rsp, &n);
        CHECK(p && n == 16 && p[13] == 0x42 && p[0] == 0x12, "client saw the server CL");
        k56flex_v8bis_free(ini);
        k56flex_v8bis_free(rsp);
    }

    /* The K56flex client call (modem_engine.c): native HSF's MS, which the
       7910 takes, in place of the Rockwell model's. */
    {
        static const uint8_t hsf[15] = {0x11, 0xc1, 0x80, 0x80, 0x80, 0x09, 0xb5, 0x02, 0x00, 0x94,
                                        0x81, 0x81, 0x02, 0x47, 0x85};
        k56flex_v8bis_cfg_t si = {K56FLEX_V8BIS_INITIATE, 1, 0, 0, 0, 0, 0, 0}, cr = {K56FLEX_V8BIS_RESPOND, 1, 0, 1, 0, 0, 20, 1};
        ini = k56flex_v8bis_new(&si);
        rsp = k56flex_v8bis_new(&cr);
        run(ini, rsp, 12, 0);
        p = k56flex_v8bis_sent_payload(rsp, &n);
        CHECK(p && n == sizeof(hsf) && !memcmp(p, hsf, sizeof(hsf)), "client sent native HSF's MS (%zu octets)", p ? n : 0);
        p = k56flex_v8bis_peer_payload(ini, &n);
        CHECK(p && n == sizeof(hsf) && !memcmp(p, hsf, sizeof(hsf)), "server received it intact");
        k56flex_v8bis_free(ini);
        k56flex_v8bis_free(rsp);
    }

    /* Same exchange 10 dB down with noise. */
    ini = k56flex_v8bis_new(&ic);
    rsp = k56flex_v8bis_new(&rc);
    run(ini, rsp, 12, 1);
    CHECK(k56flex_v8bis_result(rsp) == K56V8B_OK && k56flex_v8bis_state(rsp) == K56V8B_DONE
          && k56flex_v8bis_result(ini) == K56V8B_OK && k56flex_v8bis_state(ini) == K56V8B_DONE,
          "attenuated exchange: %s(%s) / %s(%s)", k56flex_v8bis_state_name(k56flex_v8bis_state(rsp)), k56flex_v8bis_result_name(k56flex_v8bis_result(rsp)),
          k56flex_v8bis_state_name(k56flex_v8bis_state(ini)), k56flex_v8bis_result_name(k56flex_v8bis_result(ini)));
    k56flex_v8bis_free(ini);
    k56flex_v8bis_free(rsp);

    /* Responder with no initiator times out; initiator with no responder times out. */
    rsp = k56flex_v8bis_new(&rc);
    run(NULL, rsp, 5, 0);
    CHECK(k56flex_v8bis_state(rsp) == K56V8B_FAILED && k56flex_v8bis_result(rsp) == K56V8B_NO_CRE, "no CRe");
    k56flex_v8bis_free(rsp);
    ini = k56flex_v8bis_new(&ic);
    run(ini, NULL, 6, 0);
    CHECK(k56flex_v8bis_state(ini) == K56V8B_FAILED && k56flex_v8bis_result(ini) == K56V8B_NO_CRD, "no CRd");
    k56flex_v8bis_free(ini);

    /* Blind client: no CRd ever comes, but the CL goes out and the wait is for an MS. */
    {
        k56flex_v8bis_cfg_t bc = {K56FLEX_V8BIS_INITIATE, 1, 0, 1, 1, 0, 0, 0};
        int16_t a[160], b[160];
        double cl_energy = 0;
        unsigned i, k;
        ini = k56flex_v8bis_new(&bc);
        for (i = 0; i < 50 * 12; ++i) {
            k56flex_v8bis_tx(ini, a, 160);
            if (i > 50 * 4.5) for (k = 0; k < 160; ++k) cl_energy += (double)a[k] * a[k];
            memset(b, 0, sizeof(b));
            k56flex_v8bis_rx(ini, b, 160);
        }
        CHECK(k56flex_v8bis_state(ini) == K56V8B_FAILED && k56flex_v8bis_result(ini) == K56V8B_NO_MESSAGE,
              "blind client ends waiting for MS: %s(%s)", k56flex_v8bis_state_name(k56flex_v8bis_state(ini)),
              k56flex_v8bis_result_name(k56flex_v8bis_result(ini)));
        CHECK(cl_energy > 1e6, "blind client transmitted its CL after the CRd wait (energy %.0f)", cl_energy);
        k56flex_v8bis_free(ini);
    }

    printf(failures ? "k56flex_v8bis_test: %d FAILURES\n" : "k56flex_v8bis_test: all passed\n", failures);
    return failures != 0;
}
