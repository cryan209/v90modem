#include "v92_analogue_phase3.h"
#include "v90_analogue_rx.h"
#include "v90_analogue_linear.h"
#include "v90_analogue_phase4.h"
#include "v92_cp_rx.h"
#include "v92_trn2u.h"
#include "v91.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

struct v92a_s {
    v92a_config_t cfg;
    v92a_stage_t stage;
    const char *failure;
    unsigned ticks;
    uint64_t tx_ticks;
    uint64_t su_epoch;
    v90_analogue_rx_t *rx;
    v90a_linear_t *linear;
    v92a4_t *phase4;
    int training_points;
    v92_trn2u_tx_t pam;
    uint8_t ja[4096], cpt_bits[V92_CP_RX_MAX_BITS];
    int ja_length, cpt_length, bit_pos;
    int16_t held_sample;
    bool sd_bar, jd, jp, jp_prime, dil_enough, ri, ri_bar;
    bool cpt_ready;
    unsigned ri_run[6], ri_bar_run;
    int ri_lock;
    uint64_t rx_samples;
    vpcm_cp_frame_t cpt, cp;
};

static void stage(v92a_t *s, v92a_stage_t next)
{
    if (next == V92A_SU || next == V92A_WAIT_JP)
        s->su_epoch = s->tx_ticks;
    s->stage = next;
    s->ticks = 0;
    s->bit_pos = 0;
}

static void fail(v92a_t *s, const char *why)
{
    s->failure = why;
    stage(s, V92A_FAILED);
}

/* Table 20 extends the V.90 descriptor before its CRC. Output is unpacked,
 * bit zero first, including all start and fill bits. */
static bool build_ja(v92a_t *s)
{
    uint8_t packed[512];
    if (!v90_build_dil_descriptor_bits(packed, sizeof(packed), NULL, &s->cfg.dil))
        return false;
    int beta = ((s->cfg.dil.lsp + 15)/16 + (s->cfg.dil.ltp + 15)/16)*17;
    int pos = 188 + beta + ((s->cfg.dil.n + 1)/2)*17;
    for (int i = 0; i < pos; i++)
        s->ja[i] = (packed[i/8] >> (i%8)) & 1;
    for (int i = 0; i < 16; i++)
        s->ja[pos++] = (s->cfg.upstream_rate_mask >> i) & 1;
    s->ja[pos++] = 0;
    for (int i = 0; i < 16; i++)
        s->ja[pos++] = i < 3 ? (s->cfg.upstream_rate_mask >> (16+i)) & 1 : 0;
    uint16_t crc = 0xffff;
    for (int i = 18; i < pos; i++) {
        if (i == 34 || (i >= 51 && (i-51)%17 == 0))
            continue;
        crc = (crc ^ s->ja[i]) & 1 ? (crc >> 1) ^ 0x8408 : crc >> 1;
    }
    s->ja[pos++] = 0;
    for (int i = 0; i < 16; i++)
        s->ja[pos++] = (crc >> i) & 1;
    s->ja[pos++] = 0;
    while (pos%12)
        s->ja[pos++] = 0;
    s->ja_length = pos;
    return true;
}

v92a_t *v92a_init(const v92a_config_t *cfg)
{
    if (!cfg || cfg->law > V90_LAW_ALAW || cfg->law < V90_LAW_ULAW
        || cfg->u_info < 67 || cfg->u_info > 111 || cfg->md_units != 0
        || !isfinite(cfg->digital_max_tx_dbm0)
        || !isfinite(cfg->lu) || cfg->lu < 1000 || cfg->lu > 20000
        || !cfg->upstream_rate_mask || cfg->upstream_rate_mask > 0x7ffff)
        return NULL;
    v92a_t *s = calloc(1, sizeof(*s));
    if (!s)
        return NULL;
    s->cfg = *cfg;
    s->ri_lock = -1;
    v90_analogue_rx_config_t rc = {
        .law = cfg->law, .u_info = cfg->u_info, .dil = cfg->dil,
        .dil_coverage = 1.0
    };
    s->rx = v90_analogue_rx_init(&rc);
    s->linear = v90a_linear_init(cfg->law);
    if (!s->rx || !s->linear || !build_ja(s)) {
        v92a_free(s);
        return NULL;
    }
    v90_analogue_rx_enable_v92(s->rx);
    /* The initial API takes calibrated linear samples. A physical line
     * front end must recover this scale and symbol clock before this seam. */
    uint8_t cw = v91_ucode_to_codeword((v91_law_t)cfg->law, cfg->u_info, true);
    v90a_linear_set_reference(s->linear, cfg->u_info,
                              v91_codeword_to_linear((v91_law_t)cfg->law, cw));
    v92_trn2u_tx_init(&s->pam, 2, cfg->lu, cfg->law == V90_LAW_ALAW);
    return s;
}

void v92a_free(v92a_t *s)
{
    if (!s) return;
    v90_analogue_rx_free(s->rx);
    v90a_linear_free(s->linear);
    v92a4_free(s->phase4);
    free(s);
}

static bool prepare_cpt(v92a_t *s)
{
    bool ok;
    if (s->cfg.dil.n) {
        const v90_dil_measurement_t *m = v90_analogue_rx_measurement(s->rx);
        ok = m && v90_analogue_phase4_build_cp(m, s->cfg.law,
                    s->cfg.digital_max_tx_dbm0, 0, 0, &s->cpt, &s->cp, NULL);
    } else {
        ok = v90_analogue_phase4_build_zero_dil_cp(s->cfg.law, 0, 0,
                                                  &s->cpt, &s->cp);
    }
    if (!ok) return false;
    v92_cp_frame_t f = {
        .type = V92_CP_TYPE_CPT, .drn = s->cpt.drn,
        .codec_alaw = s->cfg.law == V90_LAW_ALAW,
        .trn1d_gain_q3_13 = s->cpt.trn1d_gain_q3_13,
        .constellation_count = s->cpt.constellation_count
    };
    memcpy(f.dfi, s->cpt.dfi, sizeof(f.dfi));
    memcpy(f.masks, s->cpt.masks, sizeof(f.masks));
    s->cpt_ready = v92_cp_encode(&f, 2, s->cpt_bits, sizeof(s->cpt_bits), &s->cpt_length);
    if (s->cpt_ready) {
        v90_analogue_phase4_config_t cfg = {
            .law = s->cfg.law, .u_info = s->cfg.u_info,
            .cpt = s->cpt, .cp = s->cp
        };
        s->phase4 = v92a4_init(&cfg, s->training_points, s->cfg.lu,
                               s->cfg.upstream_rate_mask, s->cfg.round_trip_symbols);
        s->cpt_ready = s->phase4 != NULL;
    }
    return s->cpt_ready;
}

static void receive_ri(v92a_t *s, int16_t sample)
{
    static const int p[6] = {1,1,1,-1,-1,-1};
    int observed = sample > 100 ? 1 : sample < -100 ? -1 : 0;
    unsigned phase = s->rx_samples%6;
    if (s->ri_lock < 0) {
        for (unsigned h = 0; h < 6; h++) {
            s->ri_run[h] = observed == p[(phase+h)%6] ? s->ri_run[h]+1 : 0;
            if (s->ri_run[h] >= 24) {
                s->ri_lock = (int)h;
                s->ri = true;
                break;
            }
        }
    } else {
        int expected = -p[(phase+(unsigned)s->ri_lock)%6];
        s->ri_bar_run = observed == expected ? s->ri_bar_run+1 : 0;
        if (s->ri_bar_run >= 18)
            s->ri_bar = true;
    }
}

void v92a_rx(v92a_t *s, const int16_t *samples, int count)
{
    if (!s || !samples || count <= 0) return;
    for (int i = 0; i < count; i++, s->rx_samples++) {
        uint8_t cw;
        if (s->phase4) v92a4_rx(s->phase4, samples+i, 1);
        if (s->jp_prime)
            receive_ri(s, samples[i]);
        if (v90a_linear_put(s->linear, samples+i, 1, &cw, 1) != 1)
            continue;
        unsigned e = v90_analogue_rx_put(s->rx, &cw, 1);
        s->sd_bar |= (e & V90A_RX_EVENT_SD_BAR) != 0;
        s->jd |= (e & V90A_RX_EVENT_JD) != 0;
        if (e & V90A_RX_EVENT_JP) {
            const uint8_t *bits = v90_analogue_rx_jd_bits(s->rx);
            s->training_points = bits[48] ? 8 : 4;
            unsigned correction = 0;
            for (int b = 0; b < 16; b++) correction |= (unsigned)bits[18+b] << b;
            if (correction)
                fail(s, "nonzero Jp fractional phase correction is not implemented");
            else
                s->jp = true;
        }
        s->jp_prime |= (e & V90A_RX_EVENT_JP_PRIME) != 0;
        s->dil_enough |= (e & V90A_RX_EVENT_DIL_ENOUGH) != 0;
    }
}

static int16_t tick(v92a_t *s)
{
    static const int su[6] = {1,0,1,-1,0,-1};
    int16_t value = 0;
    /* 9.6.2.2.1: no B1d by 20 s + 6 round-trip delays from INFO1a.
     * The enclosing call owner receives failure and performs 9.7 retrain. */
    if (s->tx_ticks >= 320000ULL + 12ULL*s->cfg.round_trip_symbols
        && (!s->phase4 || !v92a4_downstream_ready(s->phase4))) {
        fail(s, "B1d timeout (9.6.2.2.1)");
        return 0;
    }
    switch (s->stage) {
    case V92A_SILENCE:
        if (s->ticks == 1120) { stage(s, V92A_RU); return tick(s); }
        break;
    case V92A_RU:
        if (s->ticks == 768) { stage(s, V92A_RU_BAR); return tick(s); }
        value = (s->ticks/2)%6 < 3 ? s->cfg.lu : -s->cfg.lu;
        break;
    case V92A_RU_BAR:
        if (s->ticks == 48) {
            v92_trn2u_tx_start(&s->pam, 0);
            stage(s, V92A_TRN1U); return tick(s);
        }
        value = (s->ticks/2)%6 < 3 ? -s->cfg.lu : s->cfg.lu;
        break;
    case V92A_TRN1U:
        if (s->ticks == 4080) { stage(s, V92A_JA); return tick(s); }
        if (!(s->ticks%2)) v92_trn1u_tx_linear(&s->pam, &s->held_sample, 1);
        value = s->held_sample;
        break;
    case V92A_JA:
        if (s->sd_bar && s->ticks%24 == 0) {
            stage(s, V92A_WAIT_JD); return tick(s);
        }
        if (s->ticks >= 24000) { fail(s, "Sd-bar timeout (9.5.2.2.1)"); break; }
        if (!(s->ticks%2)) {
            uint8_t bit = s->ticks < 48 ? 1 : s->ja[((s->ticks-48)/2)%s->ja_length];
            v92_trn2u_tx_bits_linear(&s->pam, &bit, 1, &s->held_sample, 1);
        }
        value = s->held_sample;
        break;
    case V92A_WAIT_JD:
        if (s->jd) { stage(s, V92A_SU); return tick(s); }
        if (s->ticks >= 72000) fail(s, "Jd timeout (9.5.2.2.2)");
        break;
    case V92A_SU:
    case V92A_SU_BAR:
    case V92A_WAIT_JP:
    case V92A_SU_FINAL:
        if (s->stage == V92A_SU && s->ticks == 288) {
            stage(s, V92A_SU_BAR); return tick(s);
        }
        if (s->stage == V92A_SU_BAR && s->ticks == 49) {
            stage(s, V92A_WAIT_JP); return tick(s);
        }
        if (s->stage == V92A_WAIT_JP && s->jp) {
            stage(s, V92A_SU_FINAL); return tick(s);
        }
        if (s->stage == V92A_SU_FINAL && s->ticks == 48) {
            v92_trn2u_tx_start(&s->pam, 0);
            stage(s, V92A_DIL); return tick(s);
        }
        value = (int16_t)lround(su[((s->tx_ticks-s->su_epoch)/2)%6]*sqrt(1.5)*s->cfg.lu);
        if (s->stage == V92A_SU_BAR || s->stage == V92A_SU_FINAL) value = -value;
        break;
    case V92A_DIL:
        if (s->jp_prime && s->ticks%24 == 0 && (s->cfg.dil.n ? s->dil_enough && s->ticks >= 4080 : s->ri)) {
            if (!prepare_cpt(s)) { fail(s, "DIL cannot produce a CPt constellation"); break; }
            stage(s, V92A_CPT); return tick(s);
        }
        if (!(s->ticks%2)) v92_trn1u_tx_linear(&s->pam, &s->held_sample, 1);
        value = s->held_sample;
        break;
    case V92A_CPT:
        if (s->ticks >= 48u + 2u*(unsigned)s->cpt_length && ((s->ticks-48)/2)%s->cpt_length == 0
            && s->ticks%2 == 0 && (s->cfg.dil.n ? s->ri : s->ri_bar)) {
            stage(s, V92A_E1U); return tick(s);
        }
        if (!(s->ticks%2)) {
            uint8_t bit = s->ticks < 48 ? 1 : s->cpt_bits[((s->ticks-48)/2)%s->cpt_length];
            v92_trn2u_tx_bits_linear(&s->pam, &bit, 1, &s->held_sample, 1);
        }
        value = s->held_sample;
        break;
    case V92A_E1U:
        if (s->ticks == 24) {
            v92a4_start(s->phase4, s->pam.prev_sign);
            stage(s, V92A_PHASE4); return tick(s);
        }
        if (!(s->ticks%2)) {
            uint8_t zero = 0;
            v92_trn2u_tx_bits_linear(&s->pam, &zero, 1, &s->held_sample, 1);
        }
        value = s->held_sample;
        break;
    case V92A_PHASE4:
        if (!(s->ticks%2)) v92a4_tx(s->phase4, &s->held_sample, 1);
        value = s->held_sample;
        break;
    case V92A_FAILED:
        return 0;
    }
    s->ticks++;
    return value;
}

int v92a_tx(v92a_t *s, int16_t *samples, int count)
{
    if (!s || !samples || count <= 0) return 0;
    for (int i = 0; i < count; i++, s->tx_ticks++)
        samples[i] = tick(s);
    return count;
}
v92a_stage_t v92a_stage(const v92a_t *s) { return s ? s->stage : V92A_FAILED; }
const char *v92a_failure(const v92a_t *s) { return s ? s->failure : "invalid configuration"; }
const vpcm_cp_frame_t *v92a_cpt(const v92a_t *s) { return s && s->cpt_ready ? &s->cpt : NULL; }
int v92a_final_e1u_sign(const v92a_t *s) { return s ? s->pam.prev_sign : 0; }

v92a4_t *v92a_phase4(v92a_t *s) { return s ? s->phase4 : NULL; }
