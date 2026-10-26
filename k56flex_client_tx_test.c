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

/* trn16 (the server's Jd 89B0): TRN2 is V.34 10.1.3.8's sixteen-point TRN,
   and MP, MP' are 10.1.3.9's sixteen-point MP.  Checked against the
   clause's text with an independent mapper and decoder: quarter point
   2*Q2+Q1 of Figure 5 ((1,1) (-3,1) (1,-3) (-3,-3)) rotated clockwise by
   I*90 degrees, I differential for MP. */
static void sixteen_point_follows_jd(void)
{
    static const double qr[4] = {1, -3, 1, -3}, qi[4] = {1, 1, -3, -3};
    k56flex_client_tx_cfg_t cfg;
    k56flex_client_tx_t tx;
    uint32_t h = 0, dh = 0;
    unsigned seen = 0, match = 0, prev = 0, nbits = 0, frames[2] = {0, 0};
    double e = 0;
    float re, im;
    uint8_t bits[4096], want[2][88];
    k56flex_client_tx_stage_t st;

    k56flex_client_tx_defaults(&cfg);
    cfg.trn16 = 1;
    cfg.mp_repeats = 3;
    cfg.mp_ack_repeats = 2;
    k56flex_client_tx_init(&tx, &cfg);
    k56flex_client_tx_start(&tx);
    k56flex_client_tx_mp_bits(&cfg, 0, want[0]);
    k56flex_client_tx_mp_bits(&cfg, 1, want[1]);
    while ((st = k56flex_client_tx_symbol(&tx, &re, &im)) != K56CT_TAIL  &&  tx.symbols < 40000) {
        double best = 1e9;
        unsigned bq = 0, bz = 0;

        if (st != K56CT_GAP  &&  st != K56CT_MP  &&  st != K56CT_MP_ACK)
            continue;
        for (unsigned q = 0; q < 4; q++)
            for (unsigned z = 0; z < 4; z++) {
                /* clockwise z*90: multiply by exp(-j*z*pi/2) */
                double c = cos(-z*M_PI/2), sn = sin(-z*M_PI/2);
                double pr = (qr[q]*c - qi[q]*sn)/sqrt(10), pi = (qr[q]*sn + qi[q]*c)/sqrt(10);
                double dd = (re - pr)*(re - pr) + (im - pi)*(im - pi);
                if (dd < best) { best = dd; bq = q; bz = z; }
            }
        CHECK(best < 1e-6, "symbol %llu is a sixteen-point symbol", (unsigned long long) tx.symbols);
        if (st == K56CT_GAP) {
            unsigned y[4];
            for (int k = 0; k < 4; k++) {
                y[k] = 1 ^ ((h >> 17) & 1) ^ ((h >> 22) & 1);
                h = (h << 1) | y[k];
            }
            match += bz == (y[0] | y[1] << 1)  &&  bq == (y[2] | y[3] << 1);
            e += (double) re*re + (double) im*im;
            seen++;
        }
        /* The receiver's view: I differential, everything descrambled with
           1 + x^-18 + x^-23 from history that runs on through TRN2. */
        unsigned i = (bz - prev) & 3, y4[4] = {i & 1, i >> 1, bq & 1, bq >> 1};
        prev = bz;
        for (int k = 0; k < 4; k++) {
            unsigned x = y4[k] ^ ((dh >> 17) & 1) ^ ((dh >> 22) & 1);
            dh = (dh << 1) | y4[k];
            if (st != K56CT_GAP  &&  nbits < sizeof(bits))
                bits[nbits++] = (uint8_t) x;
        }
    }
    CHECK(seen == cfg.gap_symbols  &&  match == seen, "sixteen-point TRN2: %u/%u", match, seen);
    CHECK(fabs(e/seen - 1.0) < 0.05, "sixteen-point TRN2 is unit RMS (%.3f)", e/seen);
    for (unsigned at = 0; at + 88 <= nbits; at += 88)
        for (int a = 0; a < 2; a++)
            frames[a] += memcmp(bits + at, want[a], 88) == 0;
    CHECK(frames[0] == cfg.mp_repeats  &&  frames[1] == cfg.mp_ack_repeats,
          "sixteen-point MP %u/%u, MP' %u/%u", frames[0], cfg.mp_repeats, frames[1], cfg.mp_ack_repeats);
}

/* The gate-A Jd detector on a synthetic server signal: P3's ones then the
   word, scrambled with 1 + x^-5 + x^-23 and sent biphase, at either
   polarity and pair alignment. */
static void jd_detector(void)
{
    static const uint16_t words[2] = {0x8990, 0x89b0};

    for (int c = 0; c < 8; c++) {
        k56flex_jd_rx_t j;
        uint32_t h = 0;
        uint16_t got = 0, word = words[c & 1];
        int sign = (c & 2) ? -1 : 1;
        long at = -1, n = 0;

        k56flex_jd_rx_init(&j);
        if (c & 4)
            k56flex_jd_rx_sample(&j, 0);            /* shift the pair alignment */
        for (int b = 0; b < 8000; b++) {
            unsigned x = b < 4000 ? 1 : (word >> (b & 15)) & 1;
            unsigned y = x ^ ((h >> 4) & 1) ^ ((h >> 22) & 1);
            int16_t v = (int16_t) (sign*(y ? 1000 : -1000));
            h = (h << 1) | y;
            for (int k = 0; k < 2; k++, n++) {
                uint16_t w = k56flex_jd_rx_sample(&j, (int16_t) (k ? -v : v));
                if (w  &&  !got) {
                    got = w;
                    at = n;
                }
            }
        }
        CHECK(got == word  &&  at > 8000  &&  at < 8000 + 2*16*40,
              "Jd %04x (case %d) detected as %04x at sample %ld", word, c, got, at);
    }
}

/* J' variants (bare 899F, repeated, none) and V.34 11.3.1's MD: stage
   order and lengths in symbols. */
static void jprime_and_md_options(void)
{
    static const struct { unsigned bare, none, records, md; unsigned jsyms; } v[] = {
        {0, 0, 0, 0, 12}, {0, 0, 100, 0, 1200}, {1, 0, 150, 0, 1200}, {0, 1, 0, 0, 0}, {0, 0, 0, 2200, 12}
    };
    for (unsigned c = 0; c < sizeof(v)/sizeof(v[0]); c++) {
        k56flex_client_tx_cfg_t cfg;
        k56flex_client_tx_t tx;
        unsigned count[16] = {0}, order[32], n = 0, s_runs = 0;
        k56flex_client_tx_stage_t st, last = K56CT_IDLE_SILENT;
        float re, im;

        k56flex_client_tx_defaults(&cfg);
        cfg.jprime_bare = v[c].bare;
        cfg.jprime_none = v[c].none;
        cfg.jprime_records = v[c].records;
        cfg.md_symbols = v[c].md;
        k56flex_client_tx_init(&tx, &cfg);
        k56flex_client_tx_start(&tx);
        while ((st = k56flex_client_tx_symbol(&tx, &re, &im)) != K56CT_MP  &&  tx.symbols < 40000) {
            count[st]++;
            if (st != last) {
                if (n < 32) order[n++] = st;
                s_runs += st == K56CT_S;
                last = st;
            }
            if (st == K56CT_MD)
                CHECK(re == 0  &&  im == 0, "case %u: MD is silent", c);
        }
        CHECK(count[K56CT_JPRIME] == v[c].jsyms, "case %u: J' %u symbols (want %u)",
              c, count[K56CT_JPRIME], v[c].jsyms);
        CHECK(count[K56CT_MD] == v[c].md  &&  s_runs == (v[c].md ? 2u : 1u),
              "case %u: MD %u symbols, %u S runs", c, count[K56CT_MD], s_runs);
        CHECK(n >= 2  &&  order[n - 1] == K56CT_GAP  &&  order[n - 2] == (v[c].none ? K56CT_REPORT : K56CT_JPRIME),
              "case %u: TRN2 follows %s", c, v[c].none ? "the report" : "J'");
    }
}

/* Bare 16-bit report, a 96-bit MP frame (native HSF's fill) and MP' alone:
   stage lengths in symbols. */
static void report_and_mp_framing(void)
{
    static const struct { unsigned bare, fill, trn16, mprep, ackrep; unsigned rep_syms, mp_syms, ack_syms; } v[] = {
        {0, 0, 0, 2, 1, 21*12, 2*44, 44},
        {1, 0, 0, 2, 1, 21*8, 2*44, 44},
        {0, 8, 0, 2, 1, 21*12, 2*48, 48},
        {0, 8, 1, 0, 3, 21*12, 0, 3*24},
        {0, 0, 1, 2, 1, 21*12, 2*22, 22},
    };
    for (unsigned c = 0; c < sizeof(v)/sizeof(v[0]); c++) {
        k56flex_client_tx_cfg_t cfg;
        k56flex_client_tx_t tx;
        unsigned count[16] = {0};
        k56flex_client_tx_stage_t st;
        float re, im;

        k56flex_client_tx_defaults(&cfg);
        cfg.report_bare = v[c].bare;
        cfg.mp_extra_fill = v[c].fill;
        cfg.trn16 = v[c].trn16;
        cfg.mp_repeats = v[c].mprep;
        cfg.mp_ack_repeats = v[c].ackrep;
        cfg.gap_symbols = 100;
        k56flex_client_tx_init(&tx, &cfg);
        k56flex_client_tx_start(&tx);
        while ((st = k56flex_client_tx_symbol(&tx, &re, &im)) != K56CT_TAIL  &&  tx.symbols < 40000)
            count[st]++;
        CHECK(count[K56CT_REPORT] == v[c].rep_syms  &&  count[K56CT_MP] == v[c].mp_syms
              &&  count[K56CT_MP_ACK] == v[c].ack_syms,
              "case %u: report %u MP %u MP' %u symbols (want %u %u %u)", c, count[K56CT_REPORT],
              count[K56CT_MP], count[K56CT_MP_ACK], v[c].rep_syms, v[c].mp_syms, v[c].ack_syms);
    }
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
    sixteen_point_follows_jd();
    jd_detector();
    jprime_and_md_options();
    report_and_mp_framing();

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

    /* Other report values (pad-table groups 0x40/0x80, field bits clear,
       pacing only) decode as sent. */
    {
        static const struct { uint8_t ext; uint16_t report; } v[] = {
            {0x40, 0x89b0}, {0x80, 0x89b0}, {0x00, 0x8880}, {0x00, 0x8890},
            /* 89b0 with one field bit changed: 5, 8 clear; 6, 9, 10, 12-14 set */
            {0x00, 0x8990}, {0x00, 0x88b0}, {0x00, 0x89f0}, {0x00, 0x8bb0},
            {0x00, 0x8db0}, {0x00, 0xa9b0}, {0x00, 0xc9b0}
        };
        for (unsigned i = 0; i < sizeof(v)/sizeof(v[0]); i++) {
            k56flex_client_tx_cfg_t c = cfg;
            c.ext = v[i].ext;
            c.report = v[i].report;
            outcome_t o = run(BAUD_3200, 1, 7, 3, 0, &c, 4);
            CHECK((o.events & K56U_REPORT_END) && o.r.ext == c.ext && o.r.report == c.report,
                  "report %02x/%04x decoded as %02x/%04x (events %02x)",
                  c.ext, c.report, o.r.ext, o.r.report, o.events);
        }
    }

    /* 00/99b0 is ambiguous to a header-only collector: rotated by 21 bits,
       the record reads 04/cd80, which also passes (report & 888F) == 8880, so
       the record phase (and with it J') is not determined by the header. */
    {
        uint32_t rec = 0x99b0u << 8, rot = ((rec >> 21) | (rec << 3)) & 0xffffff;
        CHECK((uint16_t)(rot >> 8) == 0xcd80 && (rot & 0xff) == 0x04
              && ((rot >> 8) & 0x888f) == 0x8880, "00/99b0 aliases 04/cd80");
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
