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

int main(void)
{
    k56flex_v8bis_cfg_t ic = {K56FLEX_V8BIS_INITIATE, 1, 1, 1, 0, 0}, rc = {K56FLEX_V8BIS_RESPOND, 1, 1, 0, 0, 0};
    k56flex_v8bis_t *ini, *rsp;
    const uint8_t *p;
    size_t n;
    uint8_t cl[16];

    /* Full exchange: the initiator stands in for a client (version octet 0x02). */
    ini = k56flex_v8bis_new(&ic);
    rsp = k56flex_v8bis_new(&rc);
    run(ini, rsp, 12, 0);
    CHECK(k56flex_v8bis_state(rsp) == K56V8B_DONE, "responder state %s (%s)",
          k56flex_v8bis_state_name(k56flex_v8bis_state(rsp)), k56flex_v8bis_result_name(k56flex_v8bis_result(rsp)));
    CHECK(k56flex_v8bis_state(ini) == K56V8B_DONE, "initiator state %s (%s)",
          k56flex_v8bis_state_name(k56flex_v8bis_state(ini)), k56flex_v8bis_result_name(k56flex_v8bis_result(ini)));
    p = k56flex_v8bis_peer_payload(rsp, &n);
    k56flex_v8bis_payload(K56FLEX_V8BIS_CL, 1, 1, cl);
    cl[13] = 0x02;
    CHECK(p && n == 16 && !memcmp(p, cl, 16), "responder saw the client CL");
    p = k56flex_v8bis_peer_payload(ini, &n);
    k56flex_v8bis_payload(K56FLEX_V8BIS_MS, 1, 1, cl);
    CHECK(p && n == 16 && !memcmp(p, cl, 16), "initiator saw the server MS (v90, mu-law edits)");
    CHECK(k56flex_v8bis_ack(rsp) == 1 && k56flex_v8bis_ack(ini) == 1, "ACK1 delivered %d/%d", k56flex_v8bis_ack(rsp), k56flex_v8bis_ack(ini));
    CHECK(k56flex_v8bis_result(rsp) == K56V8B_OK && k56flex_v8bis_result(ini) == K56V8B_OK, "results");
    k56flex_v8bis_free(ini);
    k56flex_v8bis_free(rsp);

    /* The lab case: v90modem (digital server) dials, the analogue client answers. */
    {
        k56flex_v8bis_cfg_t si = {K56FLEX_V8BIS_INITIATE, 1, 0, 0, 0, 0}, cr = {K56FLEX_V8BIS_RESPOND, 1, 0, 1, 0, 0};
        ini = k56flex_v8bis_new(&si);
        rsp = k56flex_v8bis_new(&cr);
        run(ini, rsp, 12, 0);
        CHECK(k56flex_v8bis_result(ini) == K56V8B_OK && k56flex_v8bis_state(ini) == K56V8B_DONE
              && k56flex_v8bis_result(rsp) == K56V8B_OK && k56flex_v8bis_state(rsp) == K56V8B_DONE,
              "server-initiated exchange: %s(%s) / %s(%s)", k56flex_v8bis_state_name(k56flex_v8bis_state(ini)),
              k56flex_v8bis_result_name(k56flex_v8bis_result(ini)), k56flex_v8bis_state_name(k56flex_v8bis_state(rsp)),
              k56flex_v8bis_result_name(k56flex_v8bis_result(rsp)));
        p = k56flex_v8bis_peer_payload(ini, &n);
        CHECK(p && n == 16 && p[13] == 0x02 && p[0] == 0x11, "server saw the client MS");
        p = k56flex_v8bis_peer_payload(rsp, &n);
        CHECK(p && n == 16 && p[13] == 0x42 && p[0] == 0x12, "client saw the server CL");
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
        k56flex_v8bis_cfg_t bc = {K56FLEX_V8BIS_INITIATE, 1, 0, 1, 1, 0};
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
