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
    cfg.report_field = k56flex_report_field(0x03, 0x8880);
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

int main(void)
{
    test_composite();
    test_to_data();
    test_gate_timeout();
    printf(failures ? "k56flex_train_test: %d FAILURES\n" : "k56flex_train_test: all passed\n", failures);
    return failures != 0;
}
