/* See k56flex_startup.h. No ordinary V.34 DATA handoff is possible. */
#include "k56flex_startup.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <spandsp.h>
#include <spandsp/private/bitstream.h>
#include <spandsp/private/power_meter.h>
#include <spandsp/private/logging.h>
#include <spandsp/private/v34.h>

struct k56flex_startup_s {
    v8_state_t *v8;
    v34_state_t *phase2;
    int state, v8_done, verbose;
    int peer_lapm;               /* the peer's CM/JM named V.42 LAPM (V.8 Table 7) */
    int last_rx, last_tx;
    uint64_t samples;
    const char *failure;
};

static void fail(k56flex_startup_t *s, const char *why)
{
    s->failure = why;
    s->state = K56S_FAILED;
}

static void v8_result(void *user, v8_parms_t *p)
{
    k56flex_startup_t *s = user;
    switch (p->status) {
    case V8_STATUS_IN_PROGRESS:
    case V8_STATUS_V8_OFFERED:
    case V8_STATUS_CALL_FUNCTION_RECEIVED:
    case V8_STATUS_CALLING_TONE_RECEIVED:
        break;
    case V8_STATUS_V8_CALL:
        /* The original Flex-only HSF controller uses common V.8 mode 1
         * for K56 after its proprietary latch, not V.34 DATA fallback. */
        s->peer_lapm = p->jm_cm.protocols == V8_PROTOCOL_LAPM_V42;
        if (p->jm_cm.modulations & V8_MOD_V34) s->v8_done = 1;
        else fail(s, "V.8 did not agree the K56flex common startup mode");
        break;
    default:
        fail(s, "negotiated K56flex V.8 startup failed; no data fallback");
        break;
    }
}

static int idle_bit(void *user)
{
    (void)user;
    return 1;
}

static void unexpected_data(void *user, int bit)
{
    k56flex_startup_t *s = user;
    (void)bit;
    fail(s, "unexpected V.34 primary data before K56flex handoff");
}

static void progress(k56flex_startup_t *s)
{
    if (s->state == K56S_V8 && s->v8_done) {
        /* Do not destroy/reinitialize inside SpanDSP's result callback. */
        v8_free(s->v8);
        s->v8 = NULL;
        /* Common Phase 2, answer-shaped A/INFO0, not V.90's 62-bit INFO0d.
         * A low initial rate is a research ceiling, not a selected data rate. */
        s->phase2 = v34_init(NULL, 3200, 31200, false, true,
                             idle_bit, s, unexpected_data, s);
        if (!s->phase2) { fail(s, "Phase-2 allocation failed"); return; }
        v34_set_k56_phase2(s->phase2, true);
        v34_tx_power(s->phase2, -10.0f);
        if (s->verbose) {
            span_log_set_level(v34_get_logging_state(s->phase2),
                               SPAN_LOG_FLOW | SPAN_LOG_SHOW_TAG);
            fprintf(stderr, "[K56S] V.8 complete; enter digital Phase 2 (A/INFO0)\n");
        }
        s->state = K56S_PHASE2;
        s->samples = 0;
    }
    if (s->state != K56S_PHASE2) return;
    v34_state_t *v = s->phase2;
    /* Private state is read-only, confined here to avoid testing the shared
     * received_event slot. INFO1 flags survive other receiver events. */
    if (s->verbose && (v->rx.stage != s->last_rx || v->tx.stage != s->last_tx)) {
        fprintf(stderr, "[K56S] Phase 2 rx=%d tx=%d INFO0=%d INFO1a=%d INFO1c=%d\n",
                v->rx.stage, v->tx.stage, v->rx.info0_received,
                v->rx.info1a_received, v->rx.info1c_received);
        s->last_rx = v->rx.stage;
        s->last_tx = v->tx.stage;
    }
    if ((v->rx.info1a_received || v->rx.info1c_received)
        && v->tx.stage >= V34_TX_STAGE_FIRST_S) {
        if (!v->rx.info0_received || v->tx.info1a.max_data_rate <= 0) {
            fail(s, "no usable measured Phase-2 INFO1 selection");
            return;
        }
        s->state = K56S_READY;
        if (s->verbose) fprintf(stderr, "[K56S] CRC-valid INFO1 and local INFO1 complete\n");
    }
}

k56flex_startup_t *k56flex_startup_new(int v8_calling_party, int verbose)
{
    k56flex_startup_t *s = calloc(1, sizeof(*s));
    v8_parms_t p;
    if (!s) return NULL;
    s->verbose = verbose;
    s->last_rx = s->last_tx = -1;
    memset(&p, 0, sizeof(p));
    p.modem_connect_tone = v8_calling_party ? MODEM_CONNECT_TONES_NONE : MODEM_CONNECT_TONES_ANSAM_PR;
    p.v92 = -1;
    p.ansam_no_delay = true; /* ACK already drained; no extra answer delay. */
    p.jm_cm.call_function = V8_CALL_V_SERIES;
    p.jm_cm.modulations = V8_MOD_V34;
    p.jm_cm.protocols = V8_PROTOCOL_LAPM_V42;
    p.jm_cm.pstn_access = V8_PSTN_ACCESS_DCE_ON_DIGITAL;
    p.jm_cm.nsf = p.jm_cm.t66 = -1;
    s->v8 = v8_init(NULL, v8_calling_party != 0, &p, v8_result, s);
    if (!s->v8) { free(s); return NULL; }
    if (verbose) fprintf(stderr, "[K56S] negotiated K56flex: V.8 %s, no fallback\n",
                         v8_calling_party ? "caller (MS sender)" : "answerer (MS receiver)");
    return s;
}

void k56flex_startup_free(k56flex_startup_t *s)
{
    if (!s) return;
    if (s->v8) v8_free(s->v8);
    if (s->phase2) v34_free(s->phase2);
    free(s);
}

void k56flex_startup_rx(k56flex_startup_t *s, const int16_t *pcm, int count)
{
    for (int i = 0; i < count; ++i) {
        if (s->state == K56S_V8) v8_rx(s->v8, pcm+i, 1);
        else if (s->state == K56S_PHASE2) v34_rx(s->phase2, pcm+i, 1);
        progress(s);
        /* Bounds are local policy, not recovered protocol timers. They run
         * on received samples, so a fast offline call has the same budget. */
        ++s->samples;
        if ((s->state == K56S_V8 && s->samples > 15*8000)
            || (s->state == K56S_PHASE2 && s->samples > 30*8000))
            fail(s, "K56flex startup timeout; no data fallback");
    }
}

void k56flex_startup_tx(k56flex_startup_t *s, int16_t *pcm, int count)
{
    for (int i = 0; i < count; ++i) {
        pcm[i] = 0;
        if (s->state == K56S_V8) v8_tx(s->v8, pcm+i, 1);
        else if (s->state == K56S_PHASE2) v34_tx(s->phase2, pcm+i, 1);
        progress(s);
    }
}

int k56flex_startup_state(const k56flex_startup_t *s) { return s->state; }
int k56flex_startup_peer_lapm(const k56flex_startup_t *s) { return s && s->peer_lapm; }
const char *k56flex_startup_failure(const k56flex_startup_t *s) { return s->failure; }

int k56flex_startup_upstream_profile(const k56flex_startup_t *s, int *baud_code, int *high_carrier)
{
    if (!s || s->state != K56S_READY || !s->phase2 || !baud_code || !high_carrier) return -1;
    *baud_code = s->phase2->tx.info1a.baud_rate_c_to_a;
    *high_carrier = s->phase2->tx.info1a.use_high_carrier;
    return 0;
}
