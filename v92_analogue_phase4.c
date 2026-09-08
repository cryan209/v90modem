#include "v92_analogue_phase4.h"
#include "v90_analogue_linear.h"
#include "v92_trn2u.h"
#include "v92_upstream_data.h"
#include "v91.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

struct v92a4_s {
    v92a4_stage_t stage;
    const char *failure;
    v90_analogue_phase4_t *rx;
    v90a_linear_t *linear;
    v92_trn2u_tx_t tx;
    v92_upstream_wave_tx_t data;
    v92_cp_frame_t cpu;
    v92_cpd_frame_t cpd;
    uint32_t rate_mask;
    bool suvd, cpd_seen, remote_ack, ack_sent, cpu_sent, downstream, cpu_retry;
    unsigned symbols, stage_symbols, cpu_end, round_trip_symbols;
    int alignment, window_len;
    uint8_t window[V92_CPD_MAX_BITS];
    uint8_t bits[V92_CP_RX_MAX_BITS];
    int bit_pos, bit_count;
    double frame[12];
    int (*get_bit)(void *);
    void *data_user;
};

static void fail(v92a4_t *s, const char *why)
{
    s->failure = why;
    s->stage = V92A4_FAILED;
}

/* Messages begin and end on downstream mapping-frame boundaries (8.8).
 * Keep candidate starts until a CRC-valid frame arrives; a corrupt frame
 * cannot hide a later valid one by supplying a fictitious length. */
static bool control(void *user, const uint8_t *bits, int n)
{
    v92a4_t *s = user;
    if (s->window_len + n > (int)sizeof(s->window)) {
        memmove(s->window, s->window+n, s->window_len-n);
        s->window_len -= n;
    }
    memcpy(s->window+s->window_len, bits, n);
    s->window_len += n;
    for (int start = 0; start + 52 <= s->window_len; start += s->alignment) {
        const uint8_t *b = s->window+start;
        int have = s->window_len-start;
        bool sync = !b[17];
        for (int i = 0; i < 17; i++) sync &= b[i] == 1;
        if (!sync) continue;
        if (b[18]) {
            v92_suvd_frame_t f;
            v92_suvd_diag_t d;
            if (have != v92_suvd_bit_length(s->alignment)
                || !v92_suvd_decode_bits(b, have, &f, &d)) continue;
            if (f.silent_period_requested) {
                fail(s, "SUVd silent-period request is unsupported");
                return false;
            }
            s->suvd = true;
            s->remote_ack |= f.acknowledge;
        } else {
            v92_cpd_frame_t f;
            v92_cpd_diag_t d;
            if (!v92_cpd_decode(b, have, &f, &d)) continue;
            if (!f.selected_upstream_drn || f.selected_upstream_drn > 19
                || !(s->rate_mask & (1u << (f.selected_upstream_drn-1)))
                || !v92_upstream_wave_profile_validate(&f)) {
                fail(s, "CPd selects an unsupported upstream profile");
                return false;
            }
            s->cpd = f;
            s->cpd_seen = true;
            s->remote_ack |= f.acknowledge;
        }
        if (!s->remote_ack && s->cpu_sent
            && s->symbols-s->cpu_end >= 800ULL + s->round_trip_symbols)
            s->cpu_retry = true;
        s->window_len = 0;
        return s->cpd_seen && s->remote_ack;
    }
    return false;
}

v92a4_t *v92a4_init(const v90_analogue_phase4_config_t *cfg,
                    int points, double lu, uint32_t rate_mask, unsigned round_trip_symbols)
{
    if (!cfg || (points != 4 && points != 8) || !rate_mask
        || !isfinite(lu) || lu <= 0 || lu > 12000) return NULL;
    v92a4_t *s = calloc(1, sizeof(*s));
    if (!s) return NULL;
    s->rx = v90_analogue_phase4_init(cfg);
    s->linear = v90a_linear_init(cfg->law);
    if (!s->rx || !s->linear) { v92a4_free(s); return NULL; }
    s->alignment = cfg->cpt.drn+8;
    s->rate_mask = rate_mask;
    s->round_trip_symbols = round_trip_symbols;
    v90_analogue_phase4_set_control_receiver(s->rx, control, s);
    uint8_t cw = v91_ucode_to_codeword((v91_law_t)cfg->law, cfg->u_info, true);
    v90a_linear_set_reference(s->linear, cfg->u_info,
                              v91_codeword_to_linear((v91_law_t)cfg->law, cw));
    v92_trn2u_tx_init(&s->tx, points, lu, cfg->law == V90_LAW_ALAW);
    s->cpu.type = V92_CP_TYPE_CPU;
    s->cpu.codec_alaw = cfg->law == V90_LAW_ALAW;
    s->cpu.drn = cfg->cp.drn;
    s->cpu.trn1d_gain_q3_13 = cfg->cp.trn1d_gain_q3_13;
    s->cpu.constellation_count = cfg->cp.constellation_count;
    s->cpu.shaping_redundancy = cfg->cp.shaping_redundancy;
    s->cpu.shaping_lookahead = cfg->cp.shaping_lookahead;
    s->cpu.shaping_a1_q1_6 = cfg->cp.shaping_a1_q1_6;
    s->cpu.shaping_a2_q1_6 = cfg->cp.shaping_a2_q1_6;
    s->cpu.shaping_b1_q1_6 = cfg->cp.shaping_b1_q1_6;
    s->cpu.shaping_b2_q1_6 = cfg->cp.shaping_b2_q1_6;
    memcpy(s->cpu.dfi, cfg->cp.dfi, sizeof(s->cpu.dfi));
    memcpy(s->cpu.masks, cfg->cp.masks, sizeof(s->cpu.masks));
    return s;
}
void v92a4_free(v92a4_t *s)
{
    if (!s) return;
    v90_analogue_phase4_free(s->rx);
    v90a_linear_free(s->linear);
    free(s);
}
void v92a4_start(v92a4_t *s, int sign)
{
    if (!s || s->stage != V92A4_WAIT) return;
    v92_trn2u_tx_start(&s->tx, sign);
    s->stage = V92A4_TRN;
}
static bool message(v92a4_t *s, bool cpu)
{
    s->bit_pos = 0;
    s->stage_symbols = 0;
    s->stage = cpu ? V92A4_CP : V92A4_SUV;
    if (cpu) {
        s->cpu_retry = false;
        s->cpu.acknowledge = s->cpd_seen;
        return v92_cp_encode(&s->cpu, s->tx.constellation_points,
                             s->bits, sizeof(s->bits), &s->bit_count);
    }
    v92_suvu_frame_t suv = {.acknowledge = s->cpd_seen, .prefilter_level_q2_2 = 16};
    return v92_suvu_encode(&suv, s->tx.constellation_points,
                            s->bits, sizeof(s->bits), &s->bit_count);
}
static int16_t sample(v92a4_t *s)
{
    int16_t out = 0;
    int bps = v92_trn2u_bits_per_symbol(s->tx.constellation_points);
    if (s->stage == V92A4_TRN && s->stage_symbols%12 == 0
        && (s->stage_symbols >= 12000 || s->suvd)) {
        if (!message(s, false)) fail(s, "cannot encode SUVu");
    }
    if (s->stage == V92A4_SUV || s->stage == V92A4_CP) {
        if (s->bit_pos == s->bit_count) {
            /* 9.6.2.1.4: both acknowledgements, after a complete message. */
            s->ack_sent |= s->bits[33] != 0;
            if (s->stage == V92A4_CP) {
                s->cpu_sent = true;
                s->cpu_end = s->symbols;
            }
            if (s->ack_sent && s->remote_ack && s->cpd_seen) {
                s->stage = V92A4_E;
                s->stage_symbols = 0;
            } else if (!message(s, s->suvd && (!s->cpu_sent
                         || s->cpu_retry))) {
                fail(s, "cannot encode Phase-4 control message");
            }
        }
    }
    if (s->stage == V92A4_TRN) {
        v92_trn2u_tx_ones_linear(&s->tx, &out, 1);
    } else if (s->stage == V92A4_SUV || s->stage == V92A4_CP) {
        v92_trn2u_tx_bits_linear(&s->tx, s->bits+s->bit_pos, bps, &out, 1);
        s->bit_pos += bps;
    } else if (s->stage == V92A4_E) {
        if (s->stage_symbols == 12u + s->cpd.extend_e2u) {
            v92_upstream_wave_tx_init(&s->data); /* 8.7.1: reset all memories */
            s->stage = V92A4_B1;
            s->stage_symbols = 0;
            return sample(s);
        }
        uint8_t zeros[3] = {0};
        v92_trn2u_tx_bits_linear(&s->tx, zeros, bps, &out, 1);
    } else if (s->stage == V92A4_B1 || s->stage == V92A4_DATA) {
        if (s->stage_symbols == 48*12) s->stage = V92A4_DATA;
        int pos = s->stage_symbols%12;
        if (!pos) {
            uint8_t ones[V92_UPSTREAM_MAX_FRAME_BITS];
            memset(ones, 1, sizeof(ones));
            if (s->stage == V92A4_DATA && s->get_bit) {
                int count = v92_upstream_bits_per_frame(s->cpd.selected_upstream_drn);
                for (int i = 0; i < count; i++) ones[i] = s->get_bit(s->data_user) & 1;
            }
            if (!v92_upstream_wave_encode_frame(&s->data, &s->cpd, ones,
                     v92_upstream_bits_per_frame(s->cpd.selected_upstream_drn), s->frame)) {
                fail(s, "cannot encode B1u"); return 0;
            }
        }
        double v = s->frame[pos];
        if (!isfinite(v) || v < -32768 || v > 32767) {
            fail(s, "upstream waveform exceeds linear PCM range"); return 0;
        }
        out = (int16_t)lround(v);
    }
    s->stage_symbols++;
    s->symbols++;
    return out;
}
int v92a4_tx(v92a4_t *s, int16_t *samples, int count)
{
    if (!s || !samples || count <= 0) return 0;
    for (int i = 0; i < count; i++) samples[i] = sample(s);
    return count;
}
void v92a4_rx(v92a4_t *s, const int16_t *samples, int count)
{
    if (!s || !samples || count <= 0) return;
    for (int i = 0; i < count; i++) {
        uint8_t cw;
        if (v90a_linear_put(s->linear, samples+i, 1, &cw, 1) != 1) continue;
        unsigned e = v90_analogue_phase4_put(s->rx, &cw, 1);
        if (e & V90A4_RX_EVENT_DATA) {
            if (v90_analogue_phase4_b1d_bit_errors(s->rx)) fail(s, "B1d validation failed");
            else s->downstream = true;
        }
    }
}
v92a4_stage_t v92a4_stage(const v92a4_t *s) { return s ? s->stage : V92A4_FAILED; }
bool v92a4_downstream_ready(const v92a4_t *s) { return s && s->downstream; }
const v92_cpd_frame_t *v92a4_cpd(const v92a4_t *s) { return s && s->cpd_seen ? &s->cpd : NULL; }
const char *v92a4_failure(const v92a4_t *s) { return s ? s->failure : "invalid configuration"; }

void v92a4_set_data_source(v92a4_t *s, int (*get_bit)(void *), void *user)
{
    if (!s) return;
    s->get_bit = get_bit;
    s->data_user = user;
}
int v92a4_get_data_bits(v92a4_t *s, uint8_t *bits, int capacity)
{
    return s ? v90_analogue_phase4_get_data_bits(s->rx, bits, capacity) : 0;
}
int v92a4_downstream_rate(const v92a4_t *s)
{
    return s ? (int)vpcm_cp_drn_to_bps(s->cpu.drn) : 0;
}
