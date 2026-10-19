/* Draft 0.23 4.1: negotiated V.8 before the common Phase-2 prelude.
 * SpanDSP peers test the boundary; the native HSF experiment separately
 * exercises the actual legacy K56 client's short upstream probe. No DATA. */
#include "k56flex_startup.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <spandsp.h>

static unsigned failures;
#define CHECK(c, ...) do { if (!(c)) { ++failures; printf("FAIL: "); \
    printf(__VA_ARGS__); puts(""); } } while (0)

typedef struct { int done, failed; unsigned mods; } result_t;
static void result(void *user, v8_parms_t *p)
{
    result_t *r = user;
    if (p->status == V8_STATUS_V8_CALL) { r->done = 1; r->mods = p->jm_cm.modulations; }
    else if (p->status == V8_STATUS_FAILED) r->failed = 1;
}
static void pair(int server_v8_calling, int chunk)
{
    k56flex_startup_t *s = k56flex_startup_new(server_v8_calling, 0);
    v8_parms_t p = {0};
    result_t r = {0};
    int16_t to_server[160] = {0}, to_peer[160];
    p.modem_connect_tone = server_v8_calling ? MODEM_CONNECT_TONES_ANSAM_PR
                                           : MODEM_CONNECT_TONES_NONE;
    p.ansam_no_delay = true;
    p.v92 = -1;
    p.jm_cm.call_function = V8_CALL_V_SERIES;
    p.jm_cm.modulations = V8_MOD_V34;
    p.jm_cm.protocols = V8_PROTOCOL_LAPM_V42;
    p.jm_cm.nsf = p.jm_cm.t66 = -1;
    v8_state_t *peer = v8_init(NULL, !server_v8_calling, &p, result, &r);
    CHECK(s && peer, "allocation");
    if (!s || !peer) { k56flex_startup_free(s); if (peer) v8_free(peer); return; }
    for (int n = 0; n < 15*8000; n += chunk) {
        k56flex_startup_rx(s, to_server, chunk);
        k56flex_startup_tx(s, to_peer, chunk);
        v8_rx(peer, to_peer, chunk);
        v8_tx(peer, to_server, chunk);
        if (r.done && k56flex_startup_state(s) == K56S_PHASE2) break;
    }
    CHECK(r.done && r.mods == V8_MOD_V34 && !r.failed,
          "common V.8 mode, role%d chunk%d", server_v8_calling, chunk);
    CHECK(k56flex_startup_state(s) == K56S_PHASE2,
          "Phase 2 entry role%d chunk%d state%d", server_v8_calling, chunk,
          k56flex_startup_state(s));
    /* Withhold a Phase-2 peer: neither elapsed time nor the V.8 result
     * alone may satisfy INFO1/PCM handoff. There is no V.34 DATA fallback. */
    memset(to_server, 0, sizeof(to_server));
    for (int n = 0; n < 31*8000; n += chunk) {
        k56flex_startup_rx(s, to_server, chunk);
        k56flex_startup_tx(s, to_peer, chunk);
    }
    CHECK(k56flex_startup_state(s) == K56S_FAILED, "Phase 2 timeout must fail closed");
    for (int i = 0; i < chunk; ++i) CHECK(!to_peer[i], "failed startup must stay silent");
    k56flex_startup_free(s);
    v8_free(peer);
}

static void no_peer(void)
{
    for (int role = 0; role < 2; ++role) {
        k56flex_startup_t *s = k56flex_startup_new(role, 0);
        int16_t zero[160] = {0}, tx[160];
        CHECK(s && k56flex_startup_state(s) == K56S_V8, "fresh call starts V.8");
        if (!s) continue;
        for (int i = 0; i < 16*50; ++i) {
            k56flex_startup_rx(s, zero, 160);
            k56flex_startup_tx(s, tx, 160);
        }
        CHECK(k56flex_startup_state(s) == K56S_FAILED && k56flex_startup_failure(s),
              "silence bounded, no synthetic Phase-2 gate");
        for (int i = 0; i < 160; ++i) CHECK(!tx[i], "failed startup must stay silent");
        k56flex_startup_free(s);
    }
    k56flex_startup_free(NULL);
}

int main(void)
{
    no_peer();
    pair(0, 160);
    pair(1, 160);
    pair(0, 7);
    printf("k56flex_startup_test: %u failures\n", failures);
    return failures != 0;
}
