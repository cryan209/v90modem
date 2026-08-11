/*
 * v90_analogue_tx_test.c — does the analogue Phase 3 transmitter emit what
 * §9.3.2 says, for as long as it says?
 *
 * Two things are checked here, and they are different in kind:
 *
 *   1. Structure and timing, at the symbol level.  The durations §9.3.2 fixes
 *      (silence 70 ms, S 128T, S̄ 16T, PP 288T) and the ones it makes
 *      conditional (Ja until the Sd-to-S̄d transition, S until J'd) are
 *      counted directly out of the symbol source.
 *
 *   2. That the Ja we transmit is the descriptor we meant to send.  The bits
 *      are re-derived from the emitted symbols by inverting the §10.1.3.3/V.34
 *      mapping — differential decode, then the GPA scrambler run forward — and
 *      parsed with v90_parse_dil_descriptor(), which CRC-checks.  That is a
 *      real check of scrambler, differential encoder and bit order together;
 *      it is not a check of the modulator, which belongs to SpanDSP.
 *
 * --write-ulaw <file> dumps the modulated Phase 3 as 8 kHz µ-law, for
 * ./vpcm_decode --v34 to read with the same receiver that reads a foreign
 * modem's upstream.  That is the end-to-end grader; this binary only proves
 * the symbols are right before they reach it.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <spandsp.h>

#include "v90.h"
#include "v90_analogue_tx.h"
#include "v90_dil_presets.h"

static int failures;

#define CHECK(cond, fmt, ...)                                           \
    do {                                                                \
        if (!(cond)) {                                                  \
            printf("  FAIL: " fmt "\n", ##__VA_ARGS__);                 \
            failures++;                                                 \
        }                                                               \
    } while (0)

/* Pull one symbol and report the stage it was emitted in. */
static v90_analogue_tx_stage_t pull(v90_analogue_tx_t *tx, float *re, float *im)
{
    v90_analogue_tx_stage_t stage;

    stage = v90_analogue_tx_stage(tx);
    v90_analogue_tx_get_symbol(tx, re, im);
    return stage;
}

/* Count how many symbols the transmitter spends in its current stage, pulling
 * until the stage changes or the limit is hit. */
static int run_stage(v90_analogue_tx_t *tx, v90_analogue_tx_stage_t expect, int limit)
{
    float re;
    float im;
    int n;

    n = 0;
    while (n < limit  &&  v90_analogue_tx_stage(tx) == expect) {
        v90_analogue_tx_get_symbol(tx, &re, &im);
        n++;
    }
    return n;
}

/*
 * Recover the Ja bits from the transmitted symbols.
 *
 * §10.1.3.3/V.34 in reverse: the symbol index is Z_n, so I_n = (Z_n - Z_{n-1})
 * mod 4 gives the two scrambled bits back, and running the same GPA scrambler
 * forward over them recovers the input.  Z_{-1} is the final TRN symbol, which
 * is what the transmitter initialised its differential encoder with.
 */
static int symbol_index(float re, float im)
{
    if (re < 0.0f)
        return (im < 0.0f) ? 0 : 1;
    return (im > 0.0f) ? 2 : 3;
}

/* The scrambler runs continuously from TRN into Ja, so the descrambler has to
 * start from the register state TRN left behind — hence reg_init, accumulated
 * from TRN's own scrambled bits by the caller. */
static void descramble_bits(uint32_t reg_init, const uint8_t *scrambled,
                            int count, uint8_t *out)
{
    uint32_t reg;
    int i;

    reg = reg_init;
    for (i = 0; i < count; i++) {
        int out_bit = scrambled[i] & 1;
        /* GPA: out = in ^ x^-5 ^ x^-23, so in = out ^ x^-5 ^ x^-23. */
        out[i] = (uint8_t) ((out_bit ^ (reg >> 4) ^ (reg >> 22)) & 1);
        reg = (reg << 1) | (uint32_t) out_bit;
    }
}

static void test_sequence(void)
{
    v90_analogue_tx_config_t cfg;
    v90_analogue_tx_t *tx;
    float re;
    float im;
    int n;

    printf("§9.3.2 sequence and durations (3200 baud, zero MD)\n");

    memset(&cfg, 0, sizeof(cfg));
    cfg.baud_rate_code = 4;     /* 3200 */
    cfg.md_units = 0;
    if (!v90_dil_preset_load(V90_DIL_PRESET_MEASUREMENT, &cfg.dil)) {
        printf("  FAIL: could not load the measurement DIL preset\n");
        failures++;
        return;
    }

    tx = v90_analogue_tx_init(&cfg);
    CHECK(tx != NULL, "transmitter did not initialise");
    if (tx == NULL)
        return;

    CHECK(v90_analogue_tx_ja_bits(tx) == v90_dil_descriptor_bit_len(&cfg.dil),
          "Ja is %d bits, descriptor is %d",
          v90_analogue_tx_ja_bits(tx), v90_dil_descriptor_bit_len(&cfg.dil));

    /* §9.3.2.1: 70 ms of silence — 224 symbols at 3200 baud. */
    n = run_stage(tx, V90A_TX_INITIAL_SILENCE, 4000);
    CHECK(n == 224, "initial silence %d symbols, expected 224 (70 ms at 3200)", n);

    n = run_stage(tx, V90A_TX_S, 4000);
    CHECK(n == 128, "S %d symbols, expected 128", n);

    n = run_stage(tx, V90A_TX_S_BAR, 4000);
    CHECK(n == 16, "S-bar %d symbols, expected 16", n);

    /* MD is zero, so §9.3.2.2 follows straight on. */
    CHECK(v90_analogue_tx_stage(tx) == V90A_TX_PP,
          "expected PP after S-bar, got %s",
          v90_analogue_tx_stage_name(v90_analogue_tx_stage(tx)));
    n = run_stage(tx, V90A_TX_PP, 4000);
    CHECK(n == 288, "PP %d symbols, expected 288", n);

    n = run_stage(tx, V90A_TX_TRN, 8000);
    CHECK(n == 2048, "TRN %d symbols, expected 2048", n);

    /* §9.3.2.4: Ja runs until the Sd-to-S̄d transition, whenever that lands. */
    CHECK(v90_analogue_tx_stage(tx) == V90A_TX_JA,
          "expected Ja after TRN, got %s",
          v90_analogue_tx_stage_name(v90_analogue_tx_stage(tx)));
    for (n = 0; n < 1000; n++)
        pull(tx, &re, &im);
    CHECK(v90_analogue_tx_stage(tx) == V90A_TX_JA,
          "Ja ended without an Sd-to-S̄d transition");
    CHECK(!v90_analogue_tx_deadline_passed(tx),
          "§9.3.2.4's 1500 ms deadline reported at %d symbols", n);

    v90_analogue_tx_sd_bar_seen(tx);
    CHECK(v90_analogue_tx_stage(tx) == V90A_TX_JA_SILENCE,
          "expected silence after the Sd-to-S̄d transition, got %s",
          v90_analogue_tx_stage_name(v90_analogue_tx_stage(tx)));

    /* §9.3.2.4/§9.3.2.7: silent until Jd, and silent means silent. */
    for (n = 0; n < 500; n++) {
        pull(tx, &re, &im);
        CHECK(re == 0.0f  &&  im == 0.0f, "post-Ja silence emitted a symbol");
        if (failures)
            break;
    }

    /* §9.3.2.4 gives 1500 ms from the start of Ja for the transition, and
     * §9.3.2.7 gives 4500 ms from the end of Ja for Jd.  Sitting in the
     * silence past that must be reported, not waited out. */
    for (n = 0; n < 4500*32/10; n++)
        pull(tx, &re, &im);
    CHECK(v90_analogue_tx_deadline_passed(tx),
          "§9.3.2.7's 4500 ms deadline not reported after %d symbols of silence", n);

    v90_analogue_tx_jd_seen(tx);
    CHECK(v90_analogue_tx_stage(tx) == V90A_TX_S_AFTER_JD,
          "expected S after Jd, got %s",
          v90_analogue_tx_stage_name(v90_analogue_tx_stage(tx)));

    /* §9.3.2.7: S has no fixed length here — it runs until J'd. */
    for (n = 0; n < 700; n++)
        pull(tx, &re, &im);
    CHECK(v90_analogue_tx_stage(tx) == V90A_TX_S_AFTER_JD,
          "S ended before J'd was detected");

    v90_analogue_tx_jd_prime_seen(tx);
    n = run_stage(tx, V90A_TX_S_BAR_AFTER_JD, 4000);
    CHECK(n == 16, "S-bar after J'd %d symbols, expected 16", n);

    /* §9.3.2.9: a non-zero DIL was requested, so DIL arrives now. */
    CHECK(v90_analogue_tx_stage(tx) == V90A_TX_DIL_RX,
          "expected DIL receive, got %s",
          v90_analogue_tx_stage_name(v90_analogue_tx_stage(tx)));
    for (n = 0; n < 500; n++) {
        pull(tx, &re, &im);
        CHECK(re == 0.0f  &&  im == 0.0f, "silent DIL receive emitted a symbol");
        if (failures)
            break;
    }

    v90_analogue_tx_dil_enough(tx);
    n = run_stage(tx, V90A_TX_S_DIL_ENOUGH, 4000);
    CHECK(n == 128, "§9.3.2.10 S %d symbols, expected 128", n);
    n = run_stage(tx, V90A_TX_S_BAR_DIL_ENOUGH, 4000);
    CHECK(n == 16, "§9.3.2.10 S-bar %d symbols, expected 16", n);

    CHECK(v90_analogue_tx_stage(tx) == V90A_TX_PHASE4,
          "expected Phase 4 after §9.3.2.10, got %s",
          v90_analogue_tx_stage_name(v90_analogue_tx_stage(tx)));

    v90_analogue_tx_free(tx);
}

static void test_zero_length_dil(void)
{
    v90_analogue_tx_config_t cfg;
    v90_analogue_tx_t *tx;
    float re;
    float im;
    int n;

    printf("§9.3.2.8 zero-length DIL goes straight to Phase 4\n");

    memset(&cfg, 0, sizeof(cfg));
    cfg.baud_rate_code = 4;
    /* cfg.dil.n stays 0: a DIL of zero length. */
    tx = v90_analogue_tx_init(&cfg);
    CHECK(tx != NULL, "transmitter did not initialise with a zero-length DIL");
    if (tx == NULL)
        return;
    CHECK(v90_analogue_tx_ja_bits(tx) == 0, "zero-length DIL produced Ja bits");

    run_stage(tx, V90A_TX_INITIAL_SILENCE, 4000);
    run_stage(tx, V90A_TX_S, 4000);
    run_stage(tx, V90A_TX_S_BAR, 4000);
    run_stage(tx, V90A_TX_PP, 4000);
    run_stage(tx, V90A_TX_TRN, 8000);
    v90_analogue_tx_sd_bar_seen(tx);
    v90_analogue_tx_jd_seen(tx);
    v90_analogue_tx_jd_prime_seen(tx);
    n = run_stage(tx, V90A_TX_S_BAR_AFTER_JD, 4000);
    CHECK(n == 16, "S-bar after J'd %d symbols, expected 16", n);
    CHECK(v90_analogue_tx_stage(tx) == V90A_TX_PHASE4,
          "expected Phase 4, got %s",
          v90_analogue_tx_stage_name(v90_analogue_tx_stage(tx)));
    for (n = 0; n < 100; n++) {
        pull(tx, &re, &im);
        CHECK(re == 0.0f  &&  im == 0.0f, "Phase 4 stage emitted a symbol");
        if (failures)
            break;
    }
    v90_analogue_tx_free(tx);
}

static void test_ja_carries_the_descriptor(void)
{
    v90_analogue_tx_config_t cfg;
    v90_analogue_tx_t *tx;
    v90_dil_desc_t recovered;
    uint8_t scrambled[8192];
    uint8_t plain[8192];
    uint8_t packed[1024];
    uint32_t scramble_reg;
    float re;
    float im;
    int prev_z;
    int count;
    int ja_bits;
    int i;
    int want;

    printf("§8.3.1 Ja carries the descriptor we asked for\n");

    memset(&cfg, 0, sizeof(cfg));
    cfg.baud_rate_code = 4;
    if (!v90_dil_preset_load(V90_DIL_PRESET_MEASUREMENT, &cfg.dil)) {
        printf("  FAIL: could not load the measurement DIL preset\n");
        failures++;
        return;
    }
    tx = v90_analogue_tx_init(&cfg);
    if (tx == NULL) {
        printf("  FAIL: transmitter did not initialise\n");
        failures++;
        return;
    }
    ja_bits = v90_analogue_tx_ja_bits(tx);
    CHECK(ja_bits > 0  &&  ja_bits <= (int) sizeof(scrambled),
          "Ja length %d bits is out of range for this test", ja_bits);
    if (ja_bits <= 0  ||  ja_bits > (int) sizeof(scrambled)) {
        v90_analogue_tx_free(tx);
        return;
    }

    /* Run to the start of Ja.  The differential encoder is initialised from
     * the final TRN symbol (§10.1.3.3/V.34), so that symbol is Z_{-1}. */
    run_stage(tx, V90A_TX_INITIAL_SILENCE, 4000);
    run_stage(tx, V90A_TX_S, 4000);
    run_stage(tx, V90A_TX_S_BAR, 4000);
    run_stage(tx, V90A_TX_PP, 4000);
    prev_z = 0;
    scramble_reg = 0;
    while (v90_analogue_tx_stage(tx) == V90A_TX_TRN) {
        v90_analogue_tx_get_symbol(tx, &re, &im);
        prev_z = symbol_index(re, im);
        /* TRN maps the scrambler output straight to the symbol index, so its
         * bits carry the scrambler state forward into Ja. */
        scramble_reg = (scramble_reg << 1) | (uint32_t) (prev_z & 1);
        scramble_reg = (scramble_reg << 1) | (uint32_t) ((prev_z >> 1) & 1);
    }

    /* One full descriptor is 2 bits per symbol. */
    count = 0;
    while (count < ja_bits) {
        int z;
        int in;

        CHECK(v90_analogue_tx_stage(tx) == V90A_TX_JA,
              "Ja ended after %d of %d bits", count, ja_bits);
        if (v90_analogue_tx_stage(tx) != V90A_TX_JA)
            break;
        v90_analogue_tx_get_symbol(tx, &re, &im);
        z = symbol_index(re, im);
        in = (z - prev_z) & 3;
        prev_z = z;
        scrambled[count++] = (uint8_t) (in & 1);
        if (count < ja_bits)
            scrambled[count++] = (uint8_t) ((in >> 1) & 1);
    }

    descramble_bits(scramble_reg, scrambled, ja_bits, plain);

    memset(packed, 0, sizeof(packed));
    for (i = 0; i < ja_bits; i++) {
        if (plain[i])
            packed[i >> 3] |= (uint8_t) (1u << (i & 7));
    }

    CHECK(v90_parse_dil_descriptor(&recovered, packed, ja_bits),
          "the transmitted Ja did not parse back into a descriptor");
    CHECK(recovered.n == cfg.dil.n, "N came back %u, sent %u",
          recovered.n, cfg.dil.n);
    CHECK(recovered.lsp == cfg.dil.lsp, "LSP came back %u, sent %u",
          recovered.lsp, cfg.dil.lsp);
    CHECK(recovered.ltp == cfg.dil.ltp, "LTP came back %u, sent %u",
          recovered.ltp, cfg.dil.ltp);
    want = 0;
    for (i = 0; i < cfg.dil.n; i++) {
        if (recovered.train_u[i] != cfg.dil.train_u[i])
            want++;
    }
    CHECK(want == 0, "%d of %u training Ucodes came back changed", want, cfg.dil.n);
    for (i = 0; i < 8; i++) {
        CHECK(recovered.h[i] == cfg.dil.h[i], "H%d came back %u, sent %u",
              i + 1, recovered.h[i], cfg.dil.h[i]);
        CHECK(recovered.ref[i] == cfg.dil.ref[i], "REF%d came back %u, sent %u",
              i + 1, recovered.ref[i], cfg.dil.ref[i]);
    }

    v90_analogue_tx_free(tx);
}

/*
 * Modulate the whole of Phase 3 through SpanDSP's V.34 transmitter and write
 * it as µ-law, so ./vpcm_decode --v34 can be pointed at it.
 *
 * The events are delivered on a timer here rather than from a real digital
 * peer: this produces a plausible waveform to decode, not a real call.
 */
static void test_rate_renegotiation_silence_cycle(void)
{
    v90_analogue_tx_config_t cfg;
    v90_analogue_tx_t *tx;
    vpcm_cp_frame_t cpt;
    vpcm_cp_frame_t cp;
    vpcm_cp_frame_t cps;
    vpcm_cp_frame_t decoded;
    uint8_t cp_bits[VPCM_CP_MAX_BITS];
    int cp_nbits;
    float re;
    float im;
    int n;

    printf("§9.6.2.1 analogue-initiated CPs/echo-reconditioning cycle\n");
    memset(&cfg, 0, sizeof(cfg));
    cfg.baud_rate_code = 4;
    tx = v90_analogue_tx_init(&cfg);
    CHECK(tx != NULL, "rate-renegotiation transmitter did not initialise");
    if (tx == NULL)
        return;

    /* Reach Phase 4 through the zero-length-DIL startup path. */
    run_stage(tx, V90A_TX_INITIAL_SILENCE, 4000);
    run_stage(tx, V90A_TX_S, 4000);
    run_stage(tx, V90A_TX_S_BAR, 4000);
    run_stage(tx, V90A_TX_PP, 4000);
    run_stage(tx, V90A_TX_TRN, 8000);
    v90_analogue_tx_sd_bar_seen(tx);
    v90_analogue_tx_jd_seen(tx);
    v90_analogue_tx_jd_prime_seen(tx);
    run_stage(tx, V90A_TX_S_BAR_AFTER_JD, 4000);

    vpcm_cp_init(&cpt);
    cpt.v90_compatibility = false;
    cpt.drn = 12;
    vpcm_cp_enable_all_ucodes(cpt.masks[0]);
    cp = cpt;
    cp.v90_compatibility = true;
    cp.drn = 1;
    cp.upstream_rate_mask = 1;
    cps = cp;
    cps.silence_request = true;
    CHECK(vpcm_cp_encode_bits(&cps, cp_bits, &cp_nbits)
          && vpcm_cp_decode_bits(cp_bits, cp_nbits, &decoded)
          && decoded.silence_request,
          "Table 14 bit 30 did not survive CPs encode/decode");
    CHECK(v90_analogue_tx_start_phase4(tx, &cpt, &cp, false),
          "could not arm Phase 4 for rate-renegotiation test");
    pull(tx, &re, &im);                 /* PHASE4 -> CPt */
    v90_analogue_tx_r_transition_seen(tx);
    run_stage(tx, V90A_TX_CPT, 4000);
    v90_analogue_tx_mp_seen(tx);
    run_stage(tx, V90A_TX_CP, 4000);
    v90_analogue_tx_mp_prime_seen(tx);
    run_stage(tx, V90A_TX_CP_PRIME, 4000);
    run_stage(tx, V90A_TX_E, 100);
    CHECK(v90_analogue_tx_stage(tx) == V90A_TX_B1_PENDING,
          "startup did not reach B1 before rate renegotiation");

    CHECK(v90_analogue_tx_start_rate_renegotiation(tx, true),
          "could not initiate CPs rate renegotiation");
    n = run_stage(tx, V90A_TX_RR_S, 4000);
    CHECK(n == 128, "rate-renegotiation S was %dT, expected 128T", n);
    n = run_stage(tx, V90A_TX_RR_S_BAR, 4000);
    CHECK(n == 16, "rate-renegotiation S-bar was %dT, expected 16T", n);
    CHECK(v90_analogue_tx_stage(tx) == V90A_TX_RR_CPS,
          "silence request did not select CPs");
    v90_analogue_tx_mp_seen(tx);
    run_stage(tx, V90A_TX_RR_CPS, 4000);
    v90_analogue_tx_mp_prime_seen(tx);  /* Ed after MP' */
    run_stage(tx, V90A_TX_RR_CPS_PRIME, 4000);
    n = run_stage(tx, V90A_TX_RR_EC_SCR, 4000);
    CHECK(n == 320, "echo-reconditioning SCR was %dT, expected 100 ms", n);
    CHECK(v90_analogue_tx_stage(tx) == V90A_TX_RR_CP,
          "echo SCR did not return to clear-bit-30 CP");
    v90_analogue_tx_rt_transition_seen(tx);
    v90_analogue_tx_mp_seen(tx);
    run_stage(tx, V90A_TX_RR_CP, 4000);
    v90_analogue_tx_mp_prime_seen(tx);
    run_stage(tx, V90A_TX_CP_PRIME, 4000);
    run_stage(tx, V90A_TX_E, 100);
    CHECK(v90_analogue_tx_stage(tx) == V90A_TX_B1_PENDING,
          "second CP/MP exchange did not return to B1");

    v90_analogue_tx_free(tx);
}

static void test_v34_analogue_retrain_entry(void)
{
    v34_state_t *v34;
    int16_t amp[160];
    int nonzero;

    printf("§9.5.2 analogue retrain entry: 70 ms silence then Tone A\n");
    v34 = v34_init(NULL, 3200, 21600, true, true,
                   NULL, NULL, NULL, NULL);
    CHECK(v34 != NULL, "could not initialise V.34 retrain fixture");
    if (v34 == NULL)
        return;
    v34_set_v90_mode(v34, 0);
    CHECK(v34_restart(v34, 3200, 21600, true) == 0,
          "could not restart V.34 retrain fixture");
    v34_v90_start_analogue_retrain(v34);
    for (int left = 560; left > 0; ) {
        int take = left > (int)(sizeof(amp)/sizeof(amp[0]))
                 ? (int)(sizeof(amp)/sizeof(amp[0])) : left;
        int got = v34_tx(v34, amp, take);

        CHECK(got == take, "retrain silence produced %d/%d samples", got, take);
        for (int i = 0; i < got; i++)
            CHECK(amp[i] == 0, "§9.5.2 silence became non-zero at %d ms",
                  (560 - left + i)/8);
        left -= got;
    }
    CHECK(v34_tx(v34, amp, 160) == 160,
          "Tone A block did not produce 160 samples");
    nonzero = 0;
    for (int i = 0; i < 160; i++)
        nonzero += (amp[i] != 0);
    CHECK(nonzero > 100, "Tone A block had only %d non-zero samples", nonzero);
    v34_free(v34);
}

static int write_ulaw(const char *path, const v90_analogue_tx_config_t *cfg)
{
    static const int baud_rates[6] = {2400, 2743, 2800, 3000, 3200, 3429};
    v90_analogue_tx_t *tx;
    v34_state_t *v34;
    FILE *f;
    int16_t amp[160];
    uint8_t out[160];
    uint64_t symbols_at_ja;
    int len;
    int i;
    int frames;

    if ((tx = v90_analogue_tx_init(cfg)) == NULL) {
        fprintf(stderr, "could not initialise the analogue transmitter\n");
        return -1;
    }
    /* The analogue modem is the calling party; INFO1d puts its upstream on the
     * high carrier (§8.2.3.2, Table 9). */
    /* The bit rate only has to be a valid pairing with the symbol rate; the
     * symbols come from outside, so no mapper of SpanDSP's is used. */
    v34 = v34_init(NULL, baud_rates[cfg->baud_rate_code], 28800, true, true,
                   NULL, NULL, NULL, NULL);
    if (v34 == NULL) {
        fprintf(stderr, "v34_init failed\n");
        v90_analogue_tx_free(tx);
        return -1;
    }
    if (v34_tx_start_external_symbols(v34, cfg->baud_rate_code, true,
                                      v90_analogue_tx_get_symbol, tx) != 0) {
        fprintf(stderr, "v34_tx_start_external_symbols failed\n");
        v34_free(v34);
        v90_analogue_tx_free(tx);
        return -1;
    }
    if ((f = fopen(path, "wb")) == NULL) {
        perror(path);
        v34_free(v34);
        v90_analogue_tx_free(tx);
        return -1;
    }

    symbols_at_ja = 0;
    /* 20 s at 8 kHz is more than Phase 3 needs at any symbol rate. */
    for (frames = 0; frames < 1000; frames++) {
        len = v34_tx(v34, amp, 160);
        if (len <= 0)
            break;
        for (i = 0; i < len; i++)
            out[i] = (uint8_t) linear_to_ulaw(amp[i]);
        if (fwrite(out, 1, (size_t) len, f) != (size_t) len) {
            perror(path);
            break;
        }
        /* Stand in for the digital peer: end Ja after ~600 ms of it, then walk
         * the rest of §9.3.2 at the intervals a peer would take. */
        switch (v90_analogue_tx_stage(tx)) {
        case V90A_TX_JA:
            if (symbols_at_ja == 0)
                symbols_at_ja = v90_analogue_tx_total_symbols(tx);
            if (v90_analogue_tx_stage_symbols(tx) > 1920)
                v90_analogue_tx_sd_bar_seen(tx);
            break;
        case V90A_TX_JA_SILENCE:
            if (v90_analogue_tx_stage_symbols(tx) > 640)
                v90_analogue_tx_jd_seen(tx);
            break;
        case V90A_TX_S_AFTER_JD:
            if (v90_analogue_tx_stage_symbols(tx) > 320)
                v90_analogue_tx_jd_prime_seen(tx);
            break;
        case V90A_TX_DIL_RX:
            if (v90_analogue_tx_stage_symbols(tx) > 3200)
                v90_analogue_tx_dil_enough(tx);
            break;
        case V90A_TX_PHASE4:
            frames = 1000;
            break;
        default:
            break;
        }
    }
    fclose(f);
    v34_free(v34);
    printf("wrote %s (%llu symbols, Ja %d bits)\n",
           path,
           (unsigned long long) v90_analogue_tx_total_symbols(tx),
           v90_analogue_tx_ja_bits(tx));
    v90_analogue_tx_free(tx);
    return 0;
}

int main(int argc, char *argv[])
{
    const char *ulaw_path;
    int i;

    ulaw_path = NULL;
    for (i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--write-ulaw") == 0  &&  i + 1 < argc)
            ulaw_path = argv[++i];
        else {
            fprintf(stderr, "usage: %s [--write-ulaw <file>]\n", argv[0]);
            return 2;
        }
    }

    test_sequence();
    test_zero_length_dil();
    test_ja_carries_the_descriptor();
    test_rate_renegotiation_silence_cycle();
    test_v34_analogue_retrain_entry();

    if (ulaw_path) {
        v90_analogue_tx_config_t cfg;

        memset(&cfg, 0, sizeof(cfg));
        cfg.baud_rate_code = 4;
        if (!v90_dil_preset_load(V90_DIL_PRESET_MEASUREMENT, &cfg.dil)) {
            fprintf(stderr, "could not load the measurement DIL preset\n");
            return 1;
        }
        if (write_ulaw(ulaw_path, &cfg) != 0)
            return 1;
    }

    if (failures) {
        printf("%d failure%s\n", failures, failures == 1 ? "" : "s");
        return 1;
    }
    printf("all analogue Phase 3 transmitter checks passed\n");
    return 0;
}
