/* Training sequencer: stage order, pair counts, composite stream hashes versus the
 * original sample generator, gates, parameter record and hand-off to the data mapper. */
#include "k56flex_train.h"
#include "k56flex_train_vectors.h"

#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(c, ...) do { if (!(c)) { ++failures; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static uint64_t fnv(const int16_t *s, size_t n, uint64_t h)
{
    size_t i;
    for (i = 0; i < n; ++i) {
        uint16_t u = (uint16_t)s[i];
        h = (h ^ (u & 0xff)) * UINT64_C(0x100000001b3);
        h = (h ^ (u >> 8)) * UINT64_C(0x100000001b3);
    }
    return h;
}

static void test_composite(void)
{
    unsigned vi, segs_ok = 0;
    for (vi = 0; vi < 4; ++vi) {
        const k56flex_train_vector_t *v = &k56flex_train_vectors[vi];
        k56flex_train_cfg_t cfg;
        k56flex_train_t t;
        uint64_t hash[K56FLEX_TRAIN_SEGMENTS];
        unsigned pairs[K56FLEX_TRAIN_SEGMENTS] = {0}, s;
        memset(&cfg, 0, sizeof(cfg));
        cfg.law = (k56flex_law_t)v->law;
        cfg.rate_bps = 48000;
        cfg.report_field = v->bit8 ? 0x100 : 0;
        cfg.training_word = 0xffff;
        for (s = 0; s < K56FLEX_TRAIN_SEGMENTS; ++s) hash[s] = UINT64_C(0xcbf29ce484222325);
        if (k56flex_train_init(&t, &cfg) < 0) { CHECK(0, "init"); continue; }
        for (;;) {
            int16_t pair[12];
            unsigned seg = (unsigned)k56flex_train_phase(&t);
            if (seg >= K56FLEX_TRAIN_SEGMENTS) break;
            if (seg == K56T_GATE_A && t.gate_pairs == K56FLEX_TRAIN_GATE_A - 1) k56flex_train_status(&t, K56FLEX_STATUS_PROBE_PEER);
            if (seg == K56T_GATE_B && t.gate_pairs == K56FLEX_TRAIN_GATE_B - 1) k56flex_train_status(&t, K56FLEX_STATUS_SILENCE_END);
            if (seg == K56T_GATE_C && t.gate_pairs == K56FLEX_TRAIN_GATE_C - 1) k56flex_train_status(&t, 1u << 12);
            if (k56flex_train_samples(&t, pair, 12) != 12) { CHECK(0, "short pair"); break; }
            hash[seg] = fnv(pair, 12, hash[seg]);
            ++pairs[seg];
        }
        for (s = 0; s < K56FLEX_TRAIN_SEGMENTS; ++s) {
            CHECK(pairs[s] == k56flex_train_pairs[s], "law %u bit8 %u segment %s: %u pairs, want %u", v->law, v->bit8,
                  k56flex_train_phase_name((k56flex_train_phase_t)s), pairs[s], k56flex_train_pairs[s]);
            if (hash[s] == v->hash[s]) ++segs_ok;
            else CHECK(0, "law %u bit8 %u segment %s differs", v->law, v->bit8, k56flex_train_phase_name((k56flex_train_phase_t)s));
        }
    }
    printf("training timeline: %u/%u segments match the firmware primitives\n", segs_ok, 4 * K56FLEX_TRAIN_SEGMENTS);
}

static int rnd_src(void *u, uint16_t *w)
{
    unsigned *s = u;
    *s = *s * 1103515245u + 12345u;
    *w = (uint16_t)(*s >> 8);
    return 0;
}

static void to_data(k56flex_law_t law, int rate)
{
    k56flex_train_cfg_t cfg;
    k56flex_train_t t;
    unsigned seed = 7, n, saw_param1 = 0, saw_param2 = 0, saw_tail = 0, saw_prime = 0;
    uint8_t oct[160];
    memset(&cfg, 0, sizeof(cfg));
    cfg.law = law;
    cfg.rate_bps = rate;
    /* Draft 0.23 4.12: 58/60k tables exist only without pad adjustment. */
    cfg.report_field = rate > 56000 ? 0 : k56flex_report_field(0x03, 0x8880);
    cfg.training_word = 0xffff;
    cfg.gate_timeout_pairs = 5000;
    CHECK(k56flex_train_init(&t, &cfg) == 0, "init 56k");
    k56flex_train_set_data_source(&t, rnd_src, &seed);
    for (n = 0; n < 400000 && k56flex_train_phase(&t) != K56T_DATA && k56flex_train_phase(&t) != K56T_FAILED; ++n) {
        k56flex_train_phase_t ph = k56flex_train_phase(&t);
        if (ph == K56T_GATE_A) k56flex_train_status(&t, K56FLEX_STATUS_PROBE_PEER);
        if (ph == K56T_GATE_B) k56flex_train_status(&t, K56FLEX_STATUS_SILENCE_END);
        if (ph == K56T_GATE_C) k56flex_train_status(&t, 1u << 13);
        if (ph == K56T_PARAM_1) { saw_param1 = 1; k56flex_train_status(&t, 1u << 14); }
        if (ph == K56T_PARAM_2) saw_param2 = 1;
        if (ph == K56T_TAIL) saw_tail = 1;
        if (ph == K56T_PRIME) saw_prime = 1;
        if (k56flex_train_g711(&t, oct, 12) != 12) { CHECK(0, "G.711 conversion failed in %s", k56flex_train_phase_name(ph)); break; }
    }
    CHECK(k56flex_train_phase(&t) == K56T_DATA, "reached DATA, ended in %s", k56flex_train_phase_name(k56flex_train_phase(&t)));
    CHECK(saw_param1 && saw_param2 && saw_tail && saw_prime, "param/tail/prime phases seen %u %u %u %u", saw_param1, saw_param2, saw_tail, saw_prime);
    /* In DATA the output is the data mapper: the stream decodes back to the source bits. */
    {
        k56flex_pcm_config_t pc = {law, rate, cfg.report_field};
        k56flex_pcm_rx_t rx;
        uint8_t o[8], bits[K56FLEX_MAX_FRAME_BITS];
        unsigned f, ok = 0;
        k56flex_pcm_rx_init(&rx, &pc);
        rx.m.dc = t.data.dc;
        rx.m.phase = t.data.phase;
        rx.m.mask = t.data.mask;
        rx.m.frame = t.data.frame;
        rx.descrambler = t.data.scrambler;
        /* The frame just generated advanced past our starting point; regenerate from the same state. */
        for (f = 0; f < 50; ++f) {
            k56flex_pcm_tx_t snap = t.data;
            int16_t lv[8];
            unsigned i;
            int nb;
            if (k56flex_pcm_tx_frame(&snap, lv) != 1) break;
            for (i = 0; i < 8; ++i) o[i] = (uint8_t)k56flex_g711_from_level(law, lv[i]);
            nb = k56flex_pcm_rx_frame(&rx, o, bits);
            if (nb < 0) break;
            t.data = snap;
            ++ok;
        }
        CHECK(ok == 50, "data frames decode after hand-off (%u/50)", ok);
    }
}

static void test_to_data(void)
{
    to_data(K56FLEX_LAW_MU, 56000);
    to_data(K56FLEX_LAW_A, 56000);
    to_data(K56FLEX_LAW_MU, 58000);
    to_data(K56FLEX_LAW_A, 58000);
    to_data(K56FLEX_LAW_MU, 60000);
    to_data(K56FLEX_LAW_A, 60000);
    to_data(K56FLEX_LAW_MU, 32000);
    to_data(K56FLEX_LAW_A, 40000);
}

static void test_gate_timeout(void)
{
    k56flex_train_cfg_t cfg;
    k56flex_train_t t;
    unsigned n;
    uint8_t oct[12];
    memset(&cfg, 0, sizeof(cfg));
    cfg.law = K56FLEX_LAW_A;
    cfg.rate_bps = 32000;
    cfg.training_word = 0xffff;
    cfg.gate_timeout_pairs = 100;
    CHECK(k56flex_train_init(&t, &cfg) == 0, "init");
    for (n = 0; n < 100000 && k56flex_train_phase(&t) != K56T_FAILED; ++n)
        if (k56flex_train_g711(&t, oct, 12) != 12) break;
    CHECK(k56flex_train_phase(&t) == K56T_FAILED, "unanswered gate A fails after the timeout");
    CHECK(t.pairs_total == 47 + 24 + 1364 + 704 + 1366 + 100, "pairs sent before giving up: %u", t.pairs_total);
    cfg.rate_bps = 31000;
    CHECK(k56flex_train_init(&t, &cfg) < 0, "bad rate rejected");
}

static void test_mp_limits(void)
{
    for (int cap = K56FLEX_RATE_MIN_BPS; cap <= K56FLEX_RATE_MAX_BPS; cap += 2000)
    for (unsigned down = 1; down <= 15; down++) {
        k56flex_train_t t;
        k56flex_train_cfg_t cfg = {0};
        cfg.rate_bps = cap;
        CHECK(!k56flex_train_init(&t, &cfg), "MP limit init");
        CHECK(k56flex_train_accept_mp(&t, 4, down, 0x5) == 7200,
              "choose supported upstream rate below maximum");
        int peer = (int)(down+15)*2000;
        CHECK(t.cfg.rate_bps == (cap < peer ? cap : peer), "respect both downstream caps");
        CHECK(t.cfg.param.extra == 3 && t.cfg.param.control == 5 && t.cfg.param.bit == 1,
              "negotiated upstream MP fields");
        k56flex_train_cfg_t before = t.cfg;
        CHECK(k56flex_train_accept_mp(&t, 0, down, 0x7fff) < 0, "cleardown rejected");
        CHECK(k56flex_train_accept_mp(&t, 4, down, 0x10) < 0, "no enabled upstream rejected");
        CHECK(k56flex_train_accept_mp(&t, 15, down, 0x7fff) < 0, "reserved upstream rejected");
        CHECK(!memcmp(&t.cfg, &before, sizeof(before)), "rejected MP cannot mutate limits");
        t.phase = K56T_PARAM_1;
        CHECK(k56flex_train_accept_mp(&t, 4, down, 15) < 0, "late MP cannot replace transmitted parameters");
    }
}

static void test_mp_pad_tables(void)
{
    for (int law = 0; law < 2; ++law)
    for (unsigned pad = 0x40; pad <= 0x80; pad += 0x40) {
        k56flex_train_cfg_t cfg = {0};
        k56flex_train_t t;
        cfg.law = (k56flex_law_t)law;
        cfg.rate_bps = 60000;
        cfg.report_field = pad;
        CHECK(k56flex_train_init(&t, &cfg) == 0, "pad MP init");
        CHECK(k56flex_train_accept_mp(&t, 13, 15, 0x1fff) == 31200,
              "pad MP accepted");
        CHECK(t.cfg.rate_bps == 56000, "pad report selects highest available table");
        k56flex_pcm_config_t pc = {cfg.law, t.cfg.rate_bps, cfg.report_field};
        CHECK(k56flex_pcm_frame_bits(&pc) == 56, "selected pad constellation exists");
        cfg.report_field = 0xc0; /* no such pad-group table */
        CHECK(k56flex_train_init(&t, &cfg) == 0, "invalid report deferred until MP");
        k56flex_train_cfg_t before = t.cfg;
        CHECK(k56flex_train_accept_mp(&t, 13, 15, 0x1fff) < 0,
              "missing constellation rejected before priming");
        CHECK(!memcmp(&t.cfg, &before, sizeof(before)), "failed table selection is atomic");
    }
}

/* A twelve-sample pair moves an eight-sample DATA grid by four samples.
 * Exercise independently moving first/second IDs, both event-driven tail
 * parities, and both parameter widths, including an MP below the ceiling. */
static void test_prime_grid(void)
{
    for (unsigned law = 0; law < 2; ++law)
    for (unsigned bit8 = 0; bit8 < 2; ++bit8)
    for (int rate = 32000; rate <= 60000; rate += 2000)
    for (unsigned gate_parity = 0; gate_parity < 2; ++gate_parity)
    for (unsigned id2_parity = 0; id2_parity < 2; ++id2_parity) {
        k56flex_train_cfg_t cfg = {0};
        k56flex_train_t t;
        cfg.law = (k56flex_law_t)law;
        cfg.rate_bps = 60000;
        cfg.report_field = bit8 << 8;
        CHECK(k56flex_train_init(&t, &cfg) == 0, "grid init");
        CHECK(k56flex_train_accept_mp(&t, 13, (unsigned)(rate/2000-15), 0x1fff) == 31200,
              "grid accepted MP %d", rate);
        t.pairs_total = 47;
        k56flex_train_force_phase(&t, K56T_ID_A);
        CHECK(t.id_a_pair == 47, "first identification captured");
        t.pairs_total = 4000 + id2_parity;
        k56flex_train_force_phase(&t, K56T_ID2_A);
        CHECK(t.id2_a_pair == 4000 + id2_parity, "second identification captured independently");
        t.pairs_total = 7240 + gate_parity;
        k56flex_train_force_phase(&t, K56T_PARAM_2);
        k56flex_train_force_phase(&t, K56T_TAIL);
        unsigned pairs = t.pairs_left;
        CHECK(((t.pairs_total + pairs - t.id2_a_pair)*12) % 8 == 4,
              "grid law=%u bit8=%u rate=%d gate=%u id2=%u", law, bit8, rate, gate_parity, id2_parity);
        CHECK(pairs >= (bit8 ? 1u : 2u) && pairs <= (bit8 ? 2u : 3u), "tail adds at most one pair");
        int16_t samples[36];
        CHECK(k56flex_train_samples(&t, samples, pairs*12) == pairs*12, "whole tail emitted");
        CHECK(t.phase == K56T_PRIME && t.prime_left == 6 && t.data_ready,
              "tail hands off once to six prime frames");
        CHECK(t.pairs_total*12 % 8 == (t.id2_a_pair*12 + 4) % 8,
              "emitted grid matches planned grid");
    }
}

int main(void)
{
    test_composite();
    test_to_data();
    test_gate_timeout();
    test_mp_limits();
    test_mp_pad_tables();
    test_prime_grid();
    printf(failures ? "k56flex_train_test: %d FAILURES\n" : "k56flex_train_test: all passed\n", failures);
    return failures != 0;
}
