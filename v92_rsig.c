/*
 * v92_rsig.c — Rf generation and the R-family sign-pattern detector.
 * See v92_rsig.h.
 */
#include "v92_rsig.h"

#include <string.h>

#include "v91.h"

static const int pat6[6] = { 1, 1, 1, -1, -1, -1 };
static const int pat4[4] = { 1, 1, -1, -1 };

bool v92_rf_ucodes(const vpcm_cp_frame_t *cpu, uint8_t ucodes[6])
{
    if (!cpu || !ucodes || cpu->constellation_count == 0)
        return false;
    for (int i = 0; i < 6; i++) {
        int c = cpu->dfi[i];
        int top = -1;

        if (c >= cpu->constellation_count || c >= VPCM_CP_MAX_CONSTELLATIONS)
            return false;
        for (int u = 127; u >= 0 && top < 0; u--)
            if (vpcm_cp_mask_get(cpu->masks[c], u))
                top = u;
        if (top < 0)
            return false;
        ucodes[i] = (uint8_t)top;
    }
    return true;
}

void v92_rf_codewords(int law, const uint8_t ucodes[6], bool bar,
                      int pos, uint8_t *out, int n)
{
    for (int i = 0; i < n; i++) {
        int k = pos + i;
        bool positive = (pat4[k % 4] > 0) != bar;

        out[i] = v91_ucode_to_codeword((v91_law_t)law, ucodes[k % 6], positive);
    }
}

void v92_rsig_rx_init(v92_rsig_rx_t *r, int threshold)
{
    memset(r, 0, sizeof(*r));
    r->threshold = threshold;
    /* R is 384T, its bar 24T.  48 consecutive matches is 8 or 12 periods;
     * 18 inside the 24T bar leaves room for a symbol or two of the edge
     * and is 2^-18 per position against random signs. */
    r->lock_symbols = 48;
    r->bar_symbols = 18;
}

v92_rsig_event_t v92_rsig_rx_put(v92_rsig_rx_t *r, int sample)
{
    int s = sample > r->threshold ? 1 : sample < -r->threshold ? -1 : 0;
    v92_rsig_event_t ev = V92_RSIG_EV_NONE;
    uint64_t n = r->symbols;

    for (int p = 0; p < 6; p++)
        r->run6[p] = (s != 0 && s == pat6[(n + (uint64_t)p) % 6]) ? r->run6[p] + 1 : 0;
    for (int p = 0; p < 4; p++)
        r->run4[p] = (s != 0 && s == pat4[(n + (uint64_t)p) % 4]) ? r->run4[p] + 1 : 0;

    if (r->kind == V92_RSIG_NONE) {
        for (int p = 0; p < 6 && r->kind == V92_RSIG_NONE; p++)
            if (r->run6[p] >= (unsigned)r->lock_symbols) {
                r->kind = V92_RSIG_P6;
                r->phase = p;
            }
        for (int p = 0; p < 4 && r->kind == V92_RSIG_NONE; p++)
            if (r->run4[p] >= (unsigned)r->lock_symbols) {
                r->kind = V92_RSIG_P4;
                r->phase = p;
            }
        if (r->kind != V92_RSIG_NONE) {
            r->r_at = n;
            ev = V92_RSIG_EV_R;
        }
    } else if (!r->bar_seen) {
        int period = r->kind == V92_RSIG_P6 ? 6 : 4;
        int barp = (r->phase + period / 2) % period;
        unsigned run = r->kind == V92_RSIG_P6 ? r->run6[barp] : r->run4[barp];

        if (run >= (unsigned)r->bar_symbols) {
            r->bar_seen = true;
            /* The transition was where the shifted run began. */
            r->bar_at = n + 1 - run;
            ev = V92_RSIG_EV_BAR;
        }
    }
    r->symbols++;
    return ev;
}
