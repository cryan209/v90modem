/* K56flex client upstream transmitter through SpanDSP's V.34 modulator and
 * G.711, into our server's own upstream receiver (Draft 0.23 clause 11;
 * V.34 10.1.3.6-10.1.3.9).  Each case sets up the Phase 3 transmitter from
 * INFO1a values (symbol rate, carrier, power reduction, pre-emphasis,
 * V.34 Table 16), plays the generator through v34_tx_start_external_symbols()
 * and requires every receiver event: S/S-bar, PP/TRN acquisition, the report,
 * its 899F terminator, MP and MP'.  Offline: says nothing about a real server. */
#include "k56flex_client_tx.h"
#include "k56flex_upstream_pcm.h"

#include <spandsp.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* V.34 Table 1 symbol-rate index (SpanDSP's enum is private). */
enum { BAUD_3200 = 4 };

static unsigned failures;
#define CHECK(c, ...) do { if (!(c)) { ++failures; printf("FAIL: "); \
    printf(__VA_ARGS__); puts(""); } } while (0)

static int dummy_get_bit(void *user) { (void) user; return 1; }
static void dummy_put_bit(void *user, int bit) { (void) user; (void) bit; }

static void source(void *user, float *re, float *im)
{
    k56flex_client_tx_symbol((k56flex_client_tx_t *) user, re, im);
}

typedef struct {
    int events;
    k56flex_upstream_pcm_result_t r;
    double trn_rms;             /* linear RMS over 0.25 s of TRN */
} outcome_t;

static outcome_t run(int baud, int high, int power_reduction, int preemph, int alaw,
                     const k56flex_client_tx_cfg_t *cfg, int seconds)
{
    outcome_t o;
    k56flex_client_tx_t tx;
    v34_state_t *v34;
    k56flex_upstream_pcm_t *rx;
    int16_t amp[160];
    double acc = 0;
    int accn = 0;

    memset(&o, 0, sizeof(o));
    v34 = v34_init(NULL, 3200, 31200, true, true, dummy_get_bit, NULL, dummy_put_bit, NULL);
    k56flex_client_tx_init(&tx, cfg);
    v34_tx_phase3_from_info1a(v34, baud, high, power_reduction, preemph);
    v34_tx_start_external_symbols(v34, baud, high, source, &tx);
    rx = k56flex_upstream_pcm_new(baud, high, 18);
    for (int block = 0; block < seconds*50; block++) {
        if (block == 25)
            k56flex_client_tx_start(&tx);           /* after 0.5 s of silence */
        if (tx.stage == K56CT_HOLD  &&  tx.n >= 1500)
            k56flex_client_tx_release_trn2(&tx);    /* a held TRN2, released */
        v34_tx(v34, amp, 160);
        for (int i = 0; i < 160; i++) {
            /* The bearer: one G.711 encode/decode, no transcoding. */
            amp[i] = alaw ? alaw_to_linear(linear_to_alaw(amp[i]))
                          : ulaw_to_linear(linear_to_ulaw(amp[i]));
            if (tx.stage == K56CT_TRN && tx.n > 800 && accn < 2000) {
                acc += (double) amp[i]*amp[i];
                accn++;
            }
        }
        o.events |= k56flex_upstream_pcm_rx(rx, amp, 160, &o.r);
        if (o.events & K56U_MP_ACK)
            break;
    }
    o.trn_rms = accn ? sqrt(acc/accn) : 0;
    k56flex_upstream_pcm_free(rx);
    v34_free(v34);
    return o;
}

static void mp_matches_native_hsf(void)
{
    /* Native HSF's MP to our server, decoded from
       reference-data/k56flex-hsf/upstream-mp-ulaw.g711 (9600 up, 52000 down). */
    static const char *hsf =
        "11111111111111111" "00000101101000000" "01111111111111111"
        "00000000000000000" "00101100000011010" "000";
    k56flex_client_tx_cfg_t cfg;
    uint8_t b[88];
    int same = 1;

    k56flex_client_tx_defaults(&cfg);
    k56flex_client_tx_mp_bits(&cfg, 0, b);
    for (int i = 0; i < 88; i++)
        if (b[i] != hsf[i] - '0')
            same = 0;
    CHECK(same, "default MP is native HSF's frame bit for bit");
}

/* Symbol level: after J' the generator must send a fresh, non-differential
   TRN (V.34 10.1.3.8), as native HSF does, before MP. */
static void second_trn_is_fresh(void)
{
    static const float re4[4] = {1, 1, -1, -1}, im4[4] = {1, -1, -1, 1};
    k56flex_client_tx_cfg_t cfg;
    k56flex_client_tx_t tx;
    uint32_t h = 0;
    unsigned seen = 0, match = 0;
    float re, im;

    k56flex_client_tx_defaults(&cfg);
    k56flex_client_tx_init(&tx, &cfg);
    k56flex_client_tx_start(&tx);
    while (seen < cfg.gap_symbols) {
        if (k56flex_client_tx_symbol(&tx, &re, &im) != K56CT_GAP)
            continue;
        unsigned d = 0;
        for (int q = 0; q < 2; q++) {
            unsigned y = 1 ^ ((h >> 17) & 1) ^ ((h >> 22) & 1);
            h = (h << 1) | y;
            d |= y << q;
        }
        match += (re > 0) == (re4[d] > 0)  &&  (im > 0) == (im4[d] > 0);
        seen++;
    }
    CHECK(match == cfg.gap_symbols, "second TRN is a fresh TRN: %u/%u", match, cfg.gap_symbols);
    CHECK(k56flex_client_tx_symbol(&tx, &re, &im) == K56CT_MP, "MP follows the second TRN");
}

/* trn2_hold: idle after J' until released, then the same fresh TRN and MP. */
static void trn2_hold_waits(void)
{
    k56flex_client_tx_cfg_t cfg;
    k56flex_client_tx_t tx;
    float re, im;
    unsigned held = 0;
    k56flex_client_tx_stage_t st;

    k56flex_client_tx_defaults(&cfg);
    cfg.trn2_hold = 1;
    k56flex_client_tx_init(&tx, &cfg);
    k56flex_client_tx_start(&tx);
    do
        st = k56flex_client_tx_symbol(&tx, &re, &im);
    while (st != K56CT_HOLD  &&  tx.symbols < 20000);
    while (held < 5000  &&  k56flex_client_tx_symbol(&tx, &re, &im) == K56CT_HOLD)
        held++;
    CHECK(held == 5000, "holds after J' until released (%u)", held);
    k56flex_client_tx_release_trn2(&tx);
    CHECK(k56flex_client_tx_symbol(&tx, &re, &im) == K56CT_GAP, "TRN2 starts on release");
}

int main(void)
{
    static const struct { int baud, high, pr, pe, alaw; } cases[] = {
        {BAUD_3200, 1, 0, 0, 0},
        {BAUD_3200, 1, 7, 3, 0},           /* what the 7910 asked for */
        {BAUD_3200, 1, 7, 3, 1},
        {BAUD_3200, 1, 0, 6, 0},
        {BAUD_3200, 0, 3, 0, 0},           /* low carrier */
    };
    k56flex_client_tx_cfg_t cfg;

    mp_matches_native_hsf();
    second_trn_is_fresh();
    trn2_hold_waits();

    k56flex_client_tx_defaults(&cfg);
    cfg.gap_symbols = 1600;                         /* keep the run short */
    for (unsigned c = 0; c < sizeof(cases)/sizeof(cases[0]); c++) {
        outcome_t o = run(cases[c].baud, cases[c].high, cases[c].pr, cases[c].pe,
                          cases[c].alaw, &cfg, 6);
        const k56flex_upstream_pcm_result_t *r = &o.r;
        int want = K56U_SBAR | K56U_ACQUIRED | K56U_REPORT | K56U_REPORT_END | K56U_MP | K56U_MP_ACK;

        printf("case %u (%s carrier, -%d dB, pre-emphasis %d, %s): events %02x corr %.3f "
               "trn %.4f pp %.4f s %.4f report %02x/%04x MP %u/%u/%04x TRN rms %.0f\n",
               c, cases[c].high ? "high" : "low", cases[c].pr, cases[c].pe,
               cases[c].alaw ? "A-law" : "mu-law", o.events, r->correlation,
               r->trn_rms, r->pp_rms, r->s_rms, r->ext, r->report,
               r->mp_up, r->mp_down, r->mp_mask, o.trn_rms);
        CHECK((o.events & want) == want, "case %u: every receiver event (got %02x)", c, o.events);
        CHECK(r->ext == cfg.ext && r->report == cfg.report, "case %u: report decoded", c);
        CHECK(r->mp_type == 0 && r->mp_up == cfg.mp_up_code && r->mp_down == cfg.mp_down_code
              && r->mp_mask == cfg.mp_mask && r->mp_trellis == 0,
              "case %u: MP fields decoded", c);
        CHECK(r->sbar_sample >= 0 && r->trn_sample > r->sbar_sample
              && r->report_sample > r->trn_sample && r->end_sample > r->report_sample
              && r->mp_sample > r->end_sample && r->mp_ack_sample > r->mp_sample,
              "case %u: events in order", c);
    }

    /* A held TRN2 (cfg.trn2_hold) still ends in a decodable MP and MP'. */
    {
        k56flex_client_tx_cfg_t held = cfg;
        held.trn2_hold = 1;
        outcome_t o = run(BAUD_3200, 1, 7, 3, 0, &held, 7);
        CHECK((o.events & (K56U_MP | K56U_MP_ACK)) == (K56U_MP | K56U_MP_ACK)
              && o.r.mp_down == cfg.mp_down_code, "held TRN2: MP and MP' decoded (events %02x)", o.events);
    }

    /* V.34 11.2.1.1.8: the INFO1a power reduction must reach the line. */
    {
        outcome_t a = run(BAUD_3200, 1, 0, 0, 0, &cfg, 2);
        outcome_t b = run(BAUD_3200, 1, 7, 0, 0, &cfg, 2);
        double db = 20*log10(a.trn_rms/b.trn_rms);
        printf("power reduction 7 dB measured %.2f dB\n", db);
        CHECK(fabs(db - 7.0) < 0.5, "7 dB INFO1a power reduction measured %.2f dB", db);
    }

    /* A report the server's collector would reject (header 888F != 8880)
       must not be accepted. */
    {
        k56flex_client_tx_cfg_t bad = cfg;
        bad.report = 0x0000;
        outcome_t o = run(BAUD_3200, 1, 0, 0, 0, &bad, 3);
        CHECK((o.events & K56U_ACQUIRED) && !(o.events & K56U_REPORT),
              "invalid report header rejected (events %02x)", o.events);
    }

    if (failures) {
        printf("k56flex_client_tx_test: %u failures\n", failures);
        return 1;
    }
    printf("k56flex_client_tx_test: all passed\n");
    return 0;
}
