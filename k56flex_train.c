#include "k56flex_train.h"

#include <string.h>

static const char *const phase_names[] = {
    "SIL0", "ID_A", "ID_B", "P1", "P2", "P3", "GATE_A", "GATE_B", "ID2_A", "ID2_B", "PT_A",
    "PT_B", "PT_C", "GATE_C", "PARAM_1", "PARAM_2", "TAIL", "PRIME", "DATA", "FAILED"
};

const char *k56flex_train_phase_name(k56flex_train_phase_t p)
{
    return (unsigned)p <= K56T_FAILED ? phase_names[p] : "?";
}

k56flex_train_phase_t k56flex_train_phase(const k56flex_train_t *t) { return t->phase; }

static int bit8(const k56flex_train_t *t) { return (t->cfg.report_field >> 8) & 1; }

static void probe_clear_state(k56flex_train_t *t)
{
    t->probe.dc = 0;
    t->probe.seq = 0;
}

static void use_stage(k56flex_train_t *t, k56flex_probe_stage_t stage, int scrambled)
{
    k56flex_probe_t keep = t->probe;
    if (k56flex_probe_init(&t->probe, stage, t->cfg.law, t->cfg.report_field & 0xc0) < 0) {
        t->phase = K56T_FAILED;
        return;
    }
    t->probe.dc = keep.dc;
    t->probe.seq = keep.seq;
    if (stage == K56FLEX_PROBE_2) t->probe.seq = (2u << 16) | 0x800;
    t->probe.scrambled = scrambled;
    memcpy(t->probe.fifo, keep.fifo, sizeof(keep.fifo));
    t->probe.fifo_head = keep.fifo_head;
    t->probe.fifo_bits = keep.fifo_bits;
    t->probe.scr_hist = keep.scr_hist;
    t->probe.last_word = keep.last_word;
}

static void enter(k56flex_train_t *t, k56flex_train_phase_t ph);

static void enter_ident(k56flex_train_t *t, k56flex_train_phase_t ph)
{
    if (ph == K56T_ID_A || ph == K56T_ID2_A) {
        use_stage(t, K56FLEX_PROBE_ID, 0);
        k56flex_probe_seed(&t->probe, 0xe4e4);
        t->pairs_left = 22;
    } else {
        k56flex_probe_append(&t->probe, 0x1b1b);
        t->pairs_left = 2;
    }
}

static void enter(k56flex_train_t *t, k56flex_train_phase_t ph)
{
    t->phase = ph;
    t->gate_pairs = 0;
    switch (ph) {
    case K56T_SIL0:
        probe_clear_state(t);
        use_stage(t, K56FLEX_PROBE_SILENCE, 1);
        k56flex_probe_seed(&t->probe, 0xffff);
        t->pairs_left = 47;
        break;
    case K56T_ID_A: case K56T_ID_B: case K56T_ID2_A: case K56T_ID2_B:
        enter_ident(t, ph);
        break;
    case K56T_P1:
        probe_clear_state(t);
        use_stage(t, K56FLEX_PROBE_1, 1);
        k56flex_probe_seed(&t->probe, 0xffff);
        t->pairs_left = 1364;
        break;
    case K56T_P2:
        use_stage(t, K56FLEX_PROBE_2, 1);
        t->pairs_left = 704;
        break;
    case K56T_P3:
        use_stage(t, K56FLEX_PROBE_3, 1);
        t->pairs_left = 1366;
        break;
    case K56T_GATE_A:
        k56flex_probe_append(&t->probe, t->cfg.training_word);
        break;
    case K56T_GATE_B:
        use_stage(t, K56FLEX_PROBE_SILENCE, 1);
        break;
    case K56T_PT_A:
        probe_clear_state(t);
        use_stage(t, K56FLEX_PROBE_3, 1);
        k56flex_probe_seed(&t->probe, 0xffff);
        t->pairs_left = 688;
        break;
    case K56T_PT_B:
        k56flex_probe_append(&t->probe, 0xffff);
        k56flex_probe_append(&t->probe, 0xffff);
        k56flex_probe_append(&t->probe, 0x0000);
        t->pairs_left = 8;
        break;
    case K56T_PT_C:
        use_stage(t, bit8(t) ? K56FLEX_PROBE_PARAM_B : K56FLEX_PROBE_PARAM_A, 1);
        k56flex_probe_append(&t->probe, 0xffff);
        t->pairs_left = 340;
        break;
    case K56T_GATE_C:
        t->status &= ~(1u << 12);              /* DA45 clears bit 12 */
        break;
    case K56T_PARAM_1: case K56T_PARAM_2: {
        k56flex_param_t p = t->cfg.param;
        uint16_t w[12];
        size_t n, i;
        p.mode = 1;                            /* DA45 sets DM 8C76 = 1 */
        p.rate = (unsigned)(t->cfg.rate_bps / 2000 + 3 - 18);
        p.final = ph == K56T_PARAM_2;
        n = k56flex_param_source(&p, w);
        for (i = 0; i < n; ++i) k56flex_probe_append(&t->probe, w[i]);
        t->pairs_left = bit8(t) ? 8 : 16;      /* DA56: halved by report bit 8 */
        break;
    }
    case K56T_TAIL:
        k56flex_probe_append(&t->probe, 0xffff);
        t->pairs_left = bit8(t) ? 1 : 2;       /* DA75 */
        break;
    case K56T_PRIME: {
        k56flex_pcm_config_t pc;
        pc.law = t->cfg.law;
        pc.rate_bps = t->cfg.rate_bps;
        pc.report_field = t->cfg.report_field;
        t->data_ready = k56flex_pcm_tx_init(&t->data, &pc, NULL, NULL) == 0;
        if (!t->data_ready) { t->phase = K56T_FAILED; break; }
        t->data.scrambler = t->probe.scr_hist;  /* BE68 history carries over */
        t->prime_left = 6;
        break;
    }
    default:
        break;
    }
}

int k56flex_train_init(k56flex_train_t *t, const k56flex_train_cfg_t *cfg)
{
    memset(t, 0, sizeof(*t));
    t->cfg = *cfg;
    if (cfg->rate_bps < K56FLEX_RATE_MIN_BPS || cfg->rate_bps > K56FLEX_RATE_MAX_BPS || cfg->rate_bps % 2000)
        return -1;
    enter(t, K56T_SIL0);
    return t->phase == K56T_FAILED ? -1 : 0;
}

void k56flex_train_force_phase(k56flex_train_t *t, k56flex_train_phase_t ph)
{
    t->blocks_in_pair = 0;
    enter(t, ph);
}

void k56flex_train_status(k56flex_train_t *t, unsigned bits) { t->status |= bits; }
void k56flex_train_set_report(k56flex_train_t *t, unsigned f) { t->cfg.report_field = f; }

void k56flex_train_set_data_source(k56flex_train_t *t, k56flex_source_fn fn, void *user)
{
    t->data_source = fn;
    t->data_user = user;
    if (t->data_ready) { t->data.source = fn; t->data.source_user = user; }
}

/* One pair finished: advance the phase machine. */
static void pair_done(k56flex_train_t *t)
{
    ++t->pairs_total;
    if (t->pairs_left) --t->pairs_left;
    switch (t->phase) {
    case K56T_SIL0:  if (!t->pairs_left) enter(t, K56T_ID_A); break;
    case K56T_ID_A:  if (!t->pairs_left) enter(t, K56T_ID_B); break;
    case K56T_ID_B:  if (!t->pairs_left) enter(t, K56T_P1); break;
    case K56T_P1:    if (!t->pairs_left) enter(t, K56T_P2); break;
    case K56T_P2:    if (!t->pairs_left) enter(t, K56T_P3); break;
    case K56T_P3:    if (!t->pairs_left) enter(t, K56T_GATE_A); break;
    case K56T_GATE_A:
        ++t->gate_pairs;
        if (t->status & K56FLEX_STATUS_PROBE_PEER) enter(t, K56T_GATE_B);
        else if (t->cfg.gate_timeout_pairs && t->gate_pairs >= t->cfg.gate_timeout_pairs) t->phase = K56T_FAILED;
        break;
    case K56T_GATE_B:
        ++t->gate_pairs;
        if (t->status & K56FLEX_STATUS_SILENCE_END) enter(t, K56T_ID2_A);
        else if (t->cfg.gate_timeout_pairs && t->gate_pairs >= t->cfg.gate_timeout_pairs) t->phase = K56T_FAILED;
        break;
    case K56T_ID2_A: if (!t->pairs_left) enter(t, K56T_ID2_B); break;
    case K56T_ID2_B: if (!t->pairs_left) enter(t, K56T_PT_A); break;
    case K56T_PT_A:  if (!t->pairs_left) enter(t, K56T_PT_B); break;
    case K56T_PT_B:  if (!t->pairs_left) enter(t, K56T_PT_C); break;
    case K56T_PT_C:  if (!t->pairs_left) enter(t, K56T_GATE_C); break;
    case K56T_GATE_C:
        ++t->gate_pairs;
        if (t->status & K56FLEX_STATUS_TRAIN_READY) enter(t, K56T_PARAM_1);
        else if (t->cfg.gate_timeout_pairs && t->gate_pairs >= t->cfg.gate_timeout_pairs) t->phase = K56T_FAILED;
        break;
    case K56T_PARAM_1:
        if (t->pairs_left) break;
        if (t->status & K56FLEX_STATUS_PARAM_RESP) {
            enter(t, K56T_PARAM_2);
        } else {
            ++t->gate_pairs;
            if (t->cfg.gate_timeout_pairs && t->gate_pairs >= t->cfg.gate_timeout_pairs) t->phase = K56T_FAILED;
            else { unsigned g = t->gate_pairs; enter(t, K56T_PARAM_1); t->gate_pairs = g; }
        }
        break;
    case K56T_PARAM_2:
        if (t->pairs_left) break;
        if (t->status & K56FLEX_STATUS_PARAM_DONE) {
            t->status &= ~(K56FLEX_STATUS_TRAIN_READY | K56FLEX_STATUS_PARAM_RESP | K56FLEX_STATUS_PARAM_DONE);
            enter(t, K56T_TAIL);
        } else {
            ++t->gate_pairs;
            if (t->cfg.gate_timeout_pairs && t->gate_pairs >= t->cfg.gate_timeout_pairs) t->phase = K56T_FAILED;
            else { unsigned g = t->gate_pairs; enter(t, K56T_PARAM_2); t->gate_pairs = g; }
        }
        break;
    case K56T_TAIL:  if (!t->pairs_left) enter(t, K56T_PRIME); break;
    default: break;
    }
}

static int ones_source(void *u, uint16_t *w) { (void)u; *w = 0xffff; return 0; }

/* Generate the next block into t->buf.  A pair's phase bookkeeping runs as soon as
 * its second block is generated: the samples already sit in buf, and only the next
 * block depends on the (possibly new) phase.  Returns 0 if no block is available. */
static int next_block(k56flex_train_t *t)
{
    t->buf_pos = 0;
    switch (t->phase) {
    case K56T_PRIME:
        t->data.source = ones_source;          /* training word FFFF until DC51 */
        if (k56flex_pcm_tx_frame(&t->data, t->buf) != 1) { t->phase = K56T_FAILED; return 0; }
        t->buf_len = K56FLEX_FRAME_SAMPLES;
        if (--t->prime_left == 0) {
            t->data.source = t->data_source;
            t->data.source_user = t->data_user;
            /* DC51 switches the source; bits already buffered from FFFF are dropped. */
            t->data.q_head = t->data.q_count = 0;
            t->phase = K56T_DATA;
        }
        return 1;
    case K56T_DATA:
        if (!t->data_ready || k56flex_pcm_tx_frame(&t->data, t->buf) != 1) return 0;
        t->buf_len = K56FLEX_FRAME_SAMPLES;
        return 1;
    case K56T_FAILED:
        return 0;
    default:
        t->buf_len = k56flex_probe_block(&t->probe, t->buf);
        if (++t->blocks_in_pair == 2) {
            t->blocks_in_pair = 0;
            pair_done(t);
        }
        return 1;
    }
}

size_t k56flex_train_samples(k56flex_train_t *t, int16_t *out, size_t n)
{
    size_t done = 0;
    while (done < n) {
        size_t take;
        if (t->buf_pos == t->buf_len && !next_block(t)) break;
        take = t->buf_len - t->buf_pos;
        if (take > n - done) take = n - done;
        memcpy(out + done, t->buf + t->buf_pos, take * sizeof(int16_t));
        t->buf_pos += (unsigned)take;
        done += take;
    }
    return done;
}

size_t k56flex_train_g711(k56flex_train_t *t, uint8_t *out, size_t n)
{
    int16_t tmp[160];
    size_t done = 0;
    while (done < n) {
        size_t want = n - done < 160 ? n - done : 160, got, i;
        got = k56flex_train_samples(t, tmp, want);
        for (i = 0; i < got; ++i) {
            int o = k56flex_g711_from_level(t->cfg.law, tmp[i]);
            if (o < 0) return done;
            out[done++] = (uint8_t)o;
        }
        if (got < want) break;
    }
    return done;
}
