/*
 * v90_analogue_phase3.c — the analogue modem's Phase 3, both directions.
 *
 * See the header.  The interesting part of this file is apply_events(), which
 * is the whole of §9.3.2's dependency on the far end.
 */

#include <stdlib.h>
#include <string.h>

#include <spandsp.h>

#include "v90_analogue_phase3.h"

static const int baud_rates[6] = {2400, 2743, 2800, 3000, 3200, 3429};

struct v90_analogue_phase3_s {
    v90_analogue_tx_t *tx;
    v90_analogue_rx_t *rx;
    v34_state_t       *v34;
    bool               owns_v34;
};

v90_analogue_phase3_t *v90_analogue_phase3_init(const v90_analogue_phase3_config_t *cfg)
{
    v90_analogue_phase3_t *s;
    v90_analogue_tx_config_t txc;
    v90_analogue_rx_config_t rxc;

    if (cfg == NULL  ||  cfg->baud_rate_code < 0  ||  cfg->baud_rate_code > 5)
        return NULL;
    if ((s = calloc(1, sizeof(*s))) == NULL)
        return NULL;

    memset(&txc, 0, sizeof(txc));
    txc.baud_rate_code = cfg->baud_rate_code;
    txc.md_units = cfg->md_units;
    txc.dil = cfg->dil;
    txc.scr_during_dil = cfg->scr_during_dil;

    memset(&rxc, 0, sizeof(rxc));
    rxc.law = cfg->law;
    rxc.u_info = cfg->u_info;
    rxc.dil = cfg->dil;
    rxc.dil_coverage = cfg->dil_coverage;

    s->tx = v90_analogue_tx_init(&txc);
    s->rx = v90_analogue_rx_init(&rxc);
    if (s->tx == NULL  ||  s->rx == NULL) {
        v90_analogue_phase3_free(s);
        return NULL;
    }

    /*
     * The V.34 modem here is a modulator and nothing else: its transmit state
     * machine and its receiver are both out of the picture, because the
     * analogue role's Phase 3 sequencing comes from the PCM downstream.  The
     * bit rate only has to pair validly with the symbol rate.
     */
    if (cfg->v34) {
        s->v34 = cfg->v34;
        s->owns_v34 = false;
    } else {
        s->v34 = v34_init(NULL, baud_rates[cfg->baud_rate_code], 28800, true, true,
                          NULL, NULL, NULL, NULL);
        s->owns_v34 = true;
    }
    if (s->v34 == NULL
        ||
        v34_tx_start_external_symbols(s->v34, cfg->baud_rate_code,
                                      cfg->high_carrier,
                                      v90_analogue_tx_get_symbol, s->tx) != 0) {
        v90_analogue_phase3_free(s);
        return NULL;
    }
    return s;
}

void v90_analogue_phase3_free(v90_analogue_phase3_t *s)
{
    if (s == NULL)
        return;
    if (s->v34  &&  s->owns_v34)
        v34_free(s->v34);
    v90_analogue_tx_free(s->tx);
    v90_analogue_rx_free(s->rx);
    free(s);
}

/*
 * §9.3.2's four conditional moments, and nothing else.
 *
 * Each transmitter entry point is a no-op unless the transmitter is in the
 * stage that signal is supposed to end, so an event that arrives early, twice,
 * or out of order cannot skip a stage — which matters because a real
 * downstream repeats Jd until it sees our S, so V90A_RX_EVENT_JD arrives once
 * per repetition.
 */
static void apply_events(v90_analogue_phase3_t *s, unsigned events)
{
    if (events & V90A_RX_EVENT_SD_BAR)
        v90_analogue_tx_sd_bar_seen(s->tx);        /* §9.3.2.4: end Ja */
    if (events & V90A_RX_EVENT_JD)
        v90_analogue_tx_jd_seen(s->tx);            /* §9.3.2.7: start S */
    if (events & V90A_RX_EVENT_JD_PRIME)
        v90_analogue_tx_jd_prime_seen(s->tx);      /* §9.3.2.8: S̄ for 16T */
    if (events & V90A_RX_EVENT_DIL_ENOUGH)
        v90_analogue_tx_dil_enough(s->tx);         /* §9.3.2.10: S, then S̄ */
}

unsigned v90_analogue_phase3_rx(v90_analogue_phase3_t *s,
                                const uint8_t *codewords,
                                int count)
{
    unsigned events;

    if (s == NULL)
        return 0;
    events = v90_analogue_rx_put(s->rx, codewords, count);
    apply_events(s, events);
    return events;
}

int v90_analogue_phase3_tx(v90_analogue_phase3_t *s, int16_t *amp, int max_len)
{
    if (s == NULL  ||  amp == NULL  ||  max_len <= 0)
        return 0;
    return v34_tx(s->v34, amp, max_len);
}

v90_analogue_tx_stage_t v90_analogue_phase3_tx_stage(const v90_analogue_phase3_t *s)
{
    return s ? v90_analogue_tx_stage(s->tx) : V90A_TX_PHASE4;
}

v90_analogue_rx_stage_t v90_analogue_phase3_rx_stage(const v90_analogue_phase3_t *s)
{
    return s ? v90_analogue_rx_stage(s->rx) : V90A_RX_HUNT_SD;
}

const v90_analogue_tx_t *v90_analogue_phase3_tx_state(const v90_analogue_phase3_t *s)
{
    return s ? s->tx : NULL;
}

const v90_analogue_rx_t *v90_analogue_phase3_rx_state(const v90_analogue_phase3_t *s)
{
    return s ? s->rx : NULL;
}

bool v90_analogue_phase3_complete(const v90_analogue_phase3_t *s)
{
    return s  &&  v90_analogue_tx_stage(s->tx) == V90A_TX_PHASE4;
}

bool v90_analogue_phase3_retrain_due(const v90_analogue_phase3_t *s)
{
    return s  &&  v90_analogue_tx_deadline_passed(s->tx);
}

const v90_dil_measurement_t *v90_analogue_phase3_measurement(const v90_analogue_phase3_t *s)
{
    return s ? v90_analogue_rx_measurement(s->rx) : NULL;
}
