/*
 * v90_analogue_rx_test.c — does the analogue Phase 3 receiver find the four
 * moments §9.3.2 turns on, in a downstream we did not generate?
 *
 * The fixtures in artifacts/eicon-digital-downstream/ are the point of this
 * test.  They are transmit-side DS0 captures of the Eicon card's own V.90
 * firmware on calls a USR Courier answered with CONNECT — the only downstream
 * in this tree that our own transmitter did not produce.  A convention error
 * that is wrong but self-consistent survives every loopback test by
 * construction; it does not survive these.
 *
 * The expected values are the ones tools/eicon_rx_conformance.py pins, derived
 * from the captures without this decoder (µ-law Ucode decomposition plus the
 * §5.3 GPC generator): Sd 64 reps and S̄d 8 reps.  They are exact, not
 * thresholds.
 *
 * TRN1d is 30000T here, where that script says 30005T, and the two are
 * measuring different things.  30005 is where the stream stops descrambling
 * to ones; 30000 is where Jd starts.  They differ because §8.4.2 switches the
 * mapping to differential at the frame boundary and TRN1d's non-differential
 * reading survives five more bits of Jd's sync ones by coincidence.  30000 is
 * the one that can be checked rather than estimated: a Table 13 frame decodes
 * there with zero structural errors and a zero CRC remainder, and at no other
 * offset within 10 symbols either side, on both fixtures.  It is also 3750 ms
 * exactly, the figure docs/eicon_downstream_comparison.md gives for what the
 * card transmits.
 *
 * DIL is deliberately not exercised against the fixtures.  §9.3.2.9 measures
 * the sequence *this* side requested in Ja, and the descriptor in those calls
 * is the Courier's, which we do not have.  Measuring it is the separate job
 * v90_dil_rx.c does, and its open defect (docs/eicon_downstream_comparison.md,
 * Finding 5) is not this module's.  The DIL path is checked instead against a
 * stream generated from a descriptor we chose, which tests the plumbing and
 * the §9.3.2.10 stopping rule and claims nothing more.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v90.h"
#include "v90_analogue_phase3.h"
#include "v90_analogue_rx.h"
#include "v90_dil_presets.h"

static int failures;

#define CHECK(cond, fmt, ...)                                           \
    do {                                                                \
        if (!(cond)) {                                                  \
            printf("  FAIL: " fmt "\n", ##__VA_ARGS__);                 \
            failures++;                                                 \
        }                                                               \
    } while (0)

typedef struct {
    const char *path;
    int         u_info;
    int         sd_reps;
    int         sd_bar_reps;
    int         trn1d_symbols;
} fixture_t;

/* From tools/eicon_rx_conformance.py, which derived them independently. */
static const fixture_t fixtures[] = {
    {"artifacts/eicon-digital-downstream/call1-connect-32000.ulaw", 48, 64, 8, 30000},
    {"artifacts/eicon-digital-downstream/call3-connect-42666.ulaw", 48, 64, 8, 30000},
};

static uint8_t *read_file(const char *path, long *len_out)
{
    uint8_t *buf;
    FILE *f;
    long len;

    if ((f = fopen(path, "rb")) == NULL)
        return NULL;
    fseek(f, 0, SEEK_END);
    len = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (len <= 0  ||  (buf = malloc((size_t) len)) == NULL) {
        fclose(f);
        return NULL;
    }
    if (fread(buf, 1, (size_t) len, f) != (size_t) len) {
        free(buf);
        fclose(f);
        return NULL;
    }
    fclose(f);
    *len_out = len;
    return buf;
}

static void test_fixture(const fixture_t *fx)
{
    v90_analogue_rx_config_t cfg;
    v90_analogue_rx_t *rx;
    uint8_t *data;
    unsigned events;
    long len;

    printf("§9.3.2 receive against %s\n", fx->path);

    if ((data = read_file(fx->path, &len)) == NULL) {
        printf("  SKIP: fixture not present\n");
        return;
    }

    memset(&cfg, 0, sizeof(cfg));
    cfg.law = V90_LAW_ULAW;
    cfg.u_info = fx->u_info;
    /* No descriptor: this fixture's DIL was requested by the Courier, not by
     * us, so there is nothing here to measure against. */
    rx = v90_analogue_rx_init(&cfg);
    if (rx == NULL) {
        printf("  FAIL: receiver did not initialise\n");
        failures++;
        free(data);
        return;
    }

    /* Feed it the way a call would: 20 ms of RTP at a time. */
    events = 0;
    for (long off = 0; off < len; off += 160) {
        int n = (int) ((len - off < 160) ? (len - off) : 160);

        events |= v90_analogue_rx_put(rx, data + off, n);
    }

    CHECK((events & V90A_RX_EVENT_SD) != 0, "Sd never acquired");
    CHECK((events & V90A_RX_EVENT_SD_BAR) != 0,
          "§9.3.2.4's Sd-to-S̄d transition never seen — Ja would never end");
    CHECK((events & V90A_RX_EVENT_TRN1D) != 0, "TRN1d never started");
    CHECK((events & V90A_RX_EVENT_JD) != 0, "no Jd frame passed structure and CRC");

    CHECK(v90_analogue_rx_sd_reps(rx) == fx->sd_reps,
          "Sd %d reps, expected %d",
          v90_analogue_rx_sd_reps(rx), fx->sd_reps);
    CHECK(v90_analogue_rx_sd_bar_reps(rx) == fx->sd_bar_reps,
          "S̄d %d reps, expected %d",
          v90_analogue_rx_sd_bar_reps(rx), fx->sd_bar_reps);
    CHECK(v90_analogue_rx_trn1d_symbols(rx) == fx->trn1d_symbols,
          "TRN1d %d symbols, expected %d",
          v90_analogue_rx_trn1d_symbols(rx), fx->trn1d_symbols);

    printf("  Sd@%lld %d reps, S̄d@%lld %d reps, TRN1d@%lld %dT, "
           "Jd@%lld %d frames (%s-point), J'd=%s\n",
           (long long) v90_analogue_rx_sd_start(rx), v90_analogue_rx_sd_reps(rx),
           (long long) v90_analogue_rx_sd_bar_start(rx), v90_analogue_rx_sd_bar_reps(rx),
           (long long) v90_analogue_rx_trn1d_start(rx), v90_analogue_rx_trn1d_symbols(rx),
           (long long) v90_analogue_rx_jd_start(rx), v90_analogue_rx_jd_frames(rx),
           v90_analogue_rx_jd_trn16(rx) ? "16" : "4",
           (events & V90A_RX_EVENT_JD_PRIME) ? "yes" : "no");

    v90_analogue_rx_free(rx);
    free(data);
}

/*
 * The DIL stage and §9.3.2.10's stopping rule, against a DIL generated from a
 * descriptor we chose — which is what the live path has, since the analogue
 * modem authored it.
 */
static void test_dil_stage(void)
{
    v90_analogue_rx_config_t cfg;
    v90_analogue_rx_t *rx;
    const v90_dil_measurement_t *m;
    uint8_t *dil;
    unsigned events;
    int cycle;

    printf("§9.3.2.9/§9.3.2.10 DIL measurement and stopping rule\n");

    memset(&cfg, 0, sizeof(cfg));
    cfg.law = V90_LAW_ULAW;
    cfg.u_info = 78;
    if (!v90_dil_preset_load(V90_DIL_PRESET_MEASUREMENT, &cfg.dil)) {
        printf("  FAIL: could not load the measurement DIL preset\n");
        failures++;
        return;
    }
    cycle = v90_dil_cycle_len(&cfg.dil);
    CHECK(cycle > 0, "descriptor has no DIL cycle");
    if (cycle <= 0)
        return;

    if ((dil = malloc((size_t) cycle)) == NULL) {
        printf("  FAIL: out of memory\n");
        failures++;
        return;
    }
    CHECK(v90_dil_generate_codewords(V90_LAW_ULAW, &cfg.dil, dil, cycle) == cycle,
          "could not generate a DIL cycle");

    rx = v90_analogue_rx_init(&cfg);
    if (rx == NULL) {
        printf("  FAIL: receiver did not initialise\n");
        failures++;
        free(dil);
        return;
    }

    /* Drive it straight into the DIL stage: this test is about §9.3.2.9 and
     * §9.3.2.10, and the signals before them are the fixtures' job. */
    v90_analogue_rx_begin_dil(rx);
    events = v90_analogue_rx_put(rx, dil, cycle);

    CHECK((events & V90A_RX_EVENT_DIL_ENOUGH) != 0,
          "§9.3.2.10 never fired on a full DIL cycle");
    CHECK(v90_analogue_rx_stage(rx) == V90A_RX_DONE,
          "receiver stage is %s, expected done",
          v90_analogue_rx_stage_name(v90_analogue_rx_stage(rx)));
    CHECK(v90_analogue_rx_dil_symbols(rx) < cycle,
          "took the whole %d-symbol cycle; §9.3.2.10 exists to stop earlier", cycle);

    m = v90_analogue_rx_measurement(rx);
    CHECK(m != NULL, "no measurement after §9.3.2.10 fired");
    if (m) {
        CHECK(m->ucodes_measured > 0, "measurement covers no Ucodes");
        /* An unimpaired stream: what went out is what came back. */
        CHECK(m->gain_db > -0.5  &&  m->gain_db < 0.5,
              "clean DIL measured %.2f dB of gain", m->gain_db);
        CHECK(m->rbs_slot_mask == 0,
              "clean DIL reported robbed bits in slots 0x%02X", m->rbs_slot_mask);
        printf("  measured %d Ucodes from %d of %d symbols (%.0f%% of a cycle), "
               "gain %.2f dB, usable %d\n",
               m->ucodes_measured, v90_analogue_rx_dil_symbols(rx), cycle,
               100.0*m->coverage, m->gain_db, m->usable_count);
    }

    v90_analogue_rx_free(rx);
    free(dil);
}

/*
 * The join: our Phase 3 transmitter driven by a real digital modem's Phase 3.
 *
 * This is the whole point of the receive path.  §9.3.2 makes four transitions
 * conditional on the far end, and until now nothing supplied those conditions
 * but a timer in a test.  Here the Eicon card's own downstream does it, and
 * the transmitter has to walk Ja -> silence -> S -> S̄ -> Phase 4 in response.
 *
 * A zero-length DIL is requested, so §9.3.2.8 goes straight to Phase 4: the
 * DIL in these captures is the one the Courier asked for, and measuring
 * somebody else's DIL is not a thing §9.3.2.9 does.
 *
 * The two directions are not time-aligned to a real call -- our transmitter
 * starts at the first codeword of the capture, which is still in V.8.  What is
 * being checked is causality, not timing: no transition happens without the
 * signal that licenses it, and every one happens once it arrives.
 */
static void test_driven_by_fixture(const fixture_t *fx)
{
    v90_analogue_phase3_config_t cfg;
    v90_analogue_phase3_t *p3;
    uint8_t *data;
    int16_t amp[160];
    long len;
    long off;
    bool saw_ja;
    bool saw_ja_silence;
    bool saw_s_after_jd;

    printf("§9.3.2 transmitter driven by %s\n", fx->path);

    if ((data = read_file(fx->path, &len)) == NULL) {
        printf("  SKIP: fixture not present\n");
        return;
    }

    memset(&cfg, 0, sizeof(cfg));
    cfg.law = V90_LAW_ULAW;
    cfg.baud_rate_code = 4;         /* 3200 */
    cfg.high_carrier = true;
    cfg.u_info = fx->u_info;
    /* cfg.dil.n stays 0: a DIL of zero length. */
    p3 = v90_analogue_phase3_init(&cfg);
    if (p3 == NULL) {
        printf("  FAIL: Phase 3 did not initialise\n");
        failures++;
        free(data);
        return;
    }

    saw_ja = false;
    saw_ja_silence = false;
    saw_s_after_jd = false;
    for (off = 0; off < len; off += 160) {
        int n = (int) ((len - off < 160) ? (len - off) : 160);

        /* One upstream sample per downstream codeword: 8 kHz both ways. */
        v90_analogue_phase3_tx(p3, amp, n);
        v90_analogue_phase3_rx(p3, data + off, n);

        switch (v90_analogue_phase3_tx_stage(p3)) {
        case V90A_TX_JA:              saw_ja = true;         break;
        case V90A_TX_JA_SILENCE:      saw_ja_silence = true; break;
        case V90A_TX_S_AFTER_JD:      saw_s_after_jd = true; break;
        default:                                             break;
        }
    }

    CHECK(saw_ja, "the transmitter never reached Ja");
    CHECK(saw_ja_silence,
          "Ja never ended — §9.3.2.4's Sd-to-S̄d transition did not reach the transmitter");
    CHECK(saw_s_after_jd,
          "S after Jd never started — §9.3.2.7 did not reach the transmitter");
    CHECK(v90_analogue_phase3_complete(p3),
          "transmitter finished in %s, expected Phase 4",
          v90_analogue_tx_stage_name(v90_analogue_phase3_tx_stage(p3)));
    printf("  transmitter reached %s; receiver reached %s\n",
           v90_analogue_tx_stage_name(v90_analogue_phase3_tx_stage(p3)),
           v90_analogue_rx_stage_name(v90_analogue_phase3_rx_stage(p3)));

    v90_analogue_phase3_free(p3);
    free(data);
}

/*
 * §9.4.2: does the Phase 4 receiver read a Phase 4 downstream?
 *
 * The stream is generated by v90.c — the *digital* side of this tree — so this
 * is the same pairing a real call has, rather than the analogue transmitter
 * grading itself.  What it cannot check is the convention questions the Eicon
 * fixtures answer for Phase 3; there is no foreign Phase 4 downstream in this
 * tree yet, and until there is, §8.6.4's level and §8.6.5's frame alignment are
 * agreed between two halves of the same codebase.
 */
/* §8.6.3 Table 16 bits 0:16. */
#define MP_SYNC_BITS_EXPECTED   17

static void put_r_pattern(uint8_t *out, int reps, v90_law_t law, int ucode,
                          bool reversed)
{
    for (int rep = 0; rep < reps; rep++) {
        for (int slot = 0; slot < 6; slot++) {
            /* §8.6.4: + + + − − − , and R̄ the same six reversed. */
            bool positive = (slot < 3) != reversed;

            out[rep*6 + slot] = v90_codeword_compose(law, ucode, positive ? 1 : 0);
        }
    }
}

static int build_mp_type0(uint8_t *bits, uint8_t max_drn, bool acknowledge)
{
    uint16_t crc;
    int i;

    memset(bits, 0, 102);
    for (i = 0; i < 17; i++)
        bits[i] = 1;
    for (i = 0; i < 4; i++)
        bits[24 + i] = (uint8_t) ((max_drn >> i) & 1);
    bits[33] = acknowledge ? 1 : 0;
    for (i = 0; i < 13; i++)             /* bits 36:48 — the rate mask */
        bits[36 + i] = 1;
    crc = 0xFFFF;
    for (int start = 17; start < 68; start += 17) {
        for (int bit = start + 1; bit <= start + 16; bit++)
            crc = crc_itu16_bits(bits[bit] & 1U, 1, crc);
    }
    for (i = 0; i < 16; i++)
        bits[69 + i] = (uint8_t) ((crc >> i) & 1);
    return 86;
}

static int build_mp_type1(uint8_t *bits, uint8_t max_drn, bool acknowledge)
{
    uint16_t crc;
    int i;

    memset(bits, 0, 204);
    for (i = 0; i < 17; i++)
        bits[i] = 1;
    bits[18] = 1;
    for (i = 0; i < 4; i++)
        bits[24 + i] = (uint8_t) ((max_drn >> i) & 1);
    bits[33] = acknowledge ? 1 : 0;
    for (i = 0; i < 13; i++)
        bits[36 + i] = 1;
    for (i = 0; i < 16; i++) {
        bits[52 + i] = (uint8_t) ((0x1234U >> i) & 1U);
        bits[69 + i] = (uint8_t) ((0xFEDCU >> i) & 1U);
        bits[86 + i] = (uint8_t) ((0x2345U >> i) & 1U);
        bits[103 + i] = (uint8_t) ((0xDCBAU >> i) & 1U);
        bits[120 + i] = (uint8_t) ((0x3456U >> i) & 1U);
        bits[137 + i] = (uint8_t) ((0xCBA9U >> i) & 1U);
    }
    crc = 0xFFFF;
    for (int start = 17; start <= 153; start += 17) {
        for (int bit = start + 1; bit <= start + 16; bit++)
            crc = crc_itu16_bits(bits[bit] & 1U, 1, crc);
    }
    for (i = 0; i < 16; i++)
        bits[171 + i] = (uint8_t) ((crc >> i) & 1);
    return 188;
}

static void test_phase4_receive(int sr)
{
    v90_analogue_phase4_config_t cfg;
    v90_analogue_phase4_t *rx;
    v90_shaped_rx_state_t zero;
    vpcm_cp_frame_t cpt;
    vpcm_cp_frame_t cp;
    vpcm_cp_frame_t rr_mapping;
    const v90_analogue_mp_t *mp;
    uint8_t mp_bits[204];
    uint8_t *plain;
    uint8_t data_plain[49*64];
    uint8_t *stream;
    unsigned events;
    int bits_per_frame;
    int trn2d_frames;
    int frames;
    int mp_nbits;
    int n;
    int b1_n;
    int startup_n;
    int len;
    const int r_reps = 32;               /* §9.4.1.1: at least 192T */
    const int r_bar_reps = 4;            /* §8.6.4: exactly four */
    const int ucode = 40;

    printf("§9.4.2 receive against a v90.c-generated Phase 4 downstream (Sr=%d)\n",
           sr);

    vpcm_cp_init(&cpt);
    cpt.v90_compatibility = false;       /* Table 14 bit 19: 0 = CPt */
    cpt.drn = 12;
    /*
     * §5.4.5: Sr = 0 disables spectral shaping and the six sign bits are
     * §5.4.5.1's differential chain; 1 to 3 run the shaper.  Both are legal
     * for a CPt and the two demap by different routes, so the receiver is run
     * against a downstream generated each way.
     */
    cpt.shaping_redundancy = (uint8_t) sr;
    cpt.shaping_lookahead = 0;
    cpt.constellation_count = 1;
    vpcm_cp_enable_all_ucodes(cpt.masks[0]);
    for (int i = 0; i < VPCM_CP_FRAME_INTERVALS; i++)
        cpt.dfi[i] = 0;

    cp = cpt;
    cp.v90_compatibility = true;
    cp.drn = 1;                         /* D=21, valid data-mode CP for init */

    bits_per_frame = cpt.drn + 8;
    /* §9.4.1.2 sends at least 2040T of TRN2d; 400 frames is 2400T. */
    trn2d_frames = 400;
    mp_nbits = (sr == 0) ? build_mp_type1(mp_bits, 14, true)
                         : build_mp_type0(mp_bits, 14, true);
    frames = trn2d_frames + (mp_nbits + bits_per_frame - 1)/bits_per_frame + 2;

    plain = calloc((size_t) frames*bits_per_frame, 1);
    /* Startup plus one complete §9.6.2.2 response cycle: 384T Rd, 24T R̄d,
     * TRN2d/MP/Ed under CP with CPt's K, then another B1d/data pair. */
    len = (r_reps + r_bar_reps)*6
        + (frames + 49 + 64 + 4 + frames + 64 + 4 + frames + 49)*6;
    stream = calloc((size_t) len, 1);
    if (plain == NULL  ||  stream == NULL) {
        printf("  FAIL: out of memory\n");
        failures++;
        free(plain);
        free(stream);
        return;
    }
    /* §8.6.5: TRN2d is scrambled ones.  MP follows in the same bit stream. */
    memset(plain, 1, (size_t) trn2d_frames*bits_per_frame);
    memcpy(plain + trn2d_frames*bits_per_frame, mp_bits, (size_t) mp_nbits);
    /* Fill to the MP mapping boundary and §8.6.2's following two complete Ed
     * frames are zero.  B1d below starts a separately reset CP mapper. */
    for (int i = trn2d_frames*bits_per_frame + mp_nbits;
         i < frames*bits_per_frame;
         i++) {
        plain[i] = 0;
    }

    put_r_pattern(stream, r_reps, V90_LAW_ULAW, ucode, false);
    put_r_pattern(stream + r_reps*6, r_bar_reps, V90_LAW_ULAW, ucode, true);
    memset(&zero, 0, sizeof(zero));
    n = v90_generate_phase4_codewords(V90_LAW_ULAW, &cpt, &zero, plain, frames,
                                      stream + (r_reps + r_bar_reps)*6,
                                      len - (r_reps + r_bar_reps)*6);
    if (n <= 0) {
        printf("  FAIL: could not generate a Phase 4 downstream\n");
        failures++;
        free(plain);
        free(stream);
        return;
    }
    /* §8.6.1: reset mapper, 48 CP-mapped frames of ones, then one data frame
     * whose alternating payload verifies that state remains continuous. */
    memset(data_plain, 1, 48*(cp.drn + 20));
    for (int i = 0; i < cp.drn + 20; i++)
        data_plain[48*(cp.drn + 20) + i] = (uint8_t)(i & 1);
    memset(&zero, 0, sizeof(zero));
    b1_n = v90_generate_phase4_codewords(
        V90_LAW_ULAW, &cp, &zero, data_plain, 49,
        stream + (r_reps + r_bar_reps)*6 + n,
        len - (r_reps + r_bar_reps)*6 - n);
    if (b1_n != 49*6) {
        printf("  FAIL: could not generate B1d/data downstream\n");
        failures++;
        free(plain);
        free(stream);
        return;
    }
    n += b1_n;
    startup_n = n;

    /* Digital-modem-initiated rate renegotiation (§9.6.1.1/§9.6.2.2). */
    put_r_pattern(stream + (r_reps + r_bar_reps)*6 + n,
                  64, V90_LAW_ULAW, 127, false);       /* Rd: 384T */
    n += 64*6;
    put_r_pattern(stream + (r_reps + r_bar_reps)*6 + n,
                  4, V90_LAW_ULAW, 127, true);         /* R̄d: 24T */
    n += 4*6;
    rr_mapping = cp;
    rr_mapping.v90_compatibility = false;
    rr_mapping.drn = cpt.drn;  /* §8.6: CP constellation/shaping, CPt K */
    memset(&zero, 0, sizeof(zero));
    b1_n = v90_generate_phase4_codewords(
        V90_LAW_ULAW, &rr_mapping, &zero, plain, frames,
        stream + (r_reps + r_bar_reps)*6 + n,
        len - (r_reps + r_bar_reps)*6 - n);
    CHECK(b1_n == frames*6,
          "could not generate rate-renegotiation TRN2d/MP/Ed");
    n += b1_n;
    if (sr == 0) {
        /* §9.6.2.1.6-.8 CPs path: after first Ed/silence, Rt/R̄t starts a
         * second clear-bit-30 CP/MP transaction. */
        put_r_pattern(stream + (r_reps + r_bar_reps)*6 + n,
                      64, V90_LAW_ULAW, 127, false);
        n += 64*6;
        put_r_pattern(stream + (r_reps + r_bar_reps)*6 + n,
                      4, V90_LAW_ULAW, 127, true);
        n += 4*6;
        memset(&zero, 0, sizeof(zero));
        b1_n = v90_generate_phase4_codewords(
            V90_LAW_ULAW, &rr_mapping, &zero, plain, frames,
            stream + (r_reps + r_bar_reps)*6 + n,
            len - (r_reps + r_bar_reps)*6 - n);
        CHECK(b1_n == frames*6,
              "could not generate post-Rt TRN2d/MP/Ed");
        n += b1_n;
    }
    memset(&zero, 0, sizeof(zero));
    b1_n = v90_generate_phase4_codewords(
        V90_LAW_ULAW, &cp, &zero, data_plain, 49,
        stream + (r_reps + r_bar_reps)*6 + n,
        len - (r_reps + r_bar_reps)*6 - n);
    CHECK(b1_n == 49*6, "could not generate post-renegotiation B1d/data");
    n += b1_n;

    memset(&cfg, 0, sizeof(cfg));
    cfg.law = V90_LAW_ULAW;
    cfg.u_info = 48;
    cfg.cpt = cpt;
    cfg.cp = cp;
    if ((rx = v90_analogue_phase4_init(&cfg)) == NULL) {
        printf("  FAIL: Phase 4 receiver did not initialise\n");
        failures++;
        free(plain);
        free(stream);
        return;
    }

    events = 0;
    {
        int startup_end = (r_reps + r_bar_reps)*6 + startup_n;
        int total = (r_reps + r_bar_reps)*6 + n;

        for (int off = 0; off < startup_end; off += 160) {
            int take = startup_end - off;
            if (take > 160)
                take = 160;
            events |= v90_analogue_phase4_put(rx, stream + off, take);
        }
        CHECK(v90_analogue_phase4_start_rate_renegotiation(rx, sr == 0),
              "could not arm local rate renegotiation");
        for (int off = startup_end; off < total; off += 160) {
            int take = total - off;
            if (take > 160)
                take = 160;
            events |= v90_analogue_phase4_put(rx, stream + off, take);
        }
    }

    CHECK((events & V90A4_RX_EVENT_R) != 0, "Ri never acquired");
    CHECK((events & V90A4_RX_EVENT_R_BAR) != 0,
          "§9.4.2.2's Ri-to-R̄i transition never seen — CPt would never end");
    CHECK((events & V90A4_RX_EVENT_TRN2D) != 0, "TRN2d never started");
    CHECK((events & V90A4_RX_EVENT_MP) != 0,
          "no MP frame passed Table 16's structure and CRC");
    CHECK((events & V90A4_RX_EVENT_MP_PRIME) != 0,
          "MP' not recognised — §9.4.2.4's E would never be sent");
    CHECK((events & V90A4_RX_EVENT_ED) != 0,
          "two complete Ed frames were not recognised");
    CHECK((events & V90A4_RX_EVENT_B1D) != 0,
          "B1d did not start on the reset CP mapper");
    CHECK((events & V90A4_RX_EVENT_DATA) != 0,
          "48 B1d frames did not release downstream data");
    CHECK((events & V90A4_RX_EVENT_RD) != 0,
          "§9.6.2.2.1 Rd was not acquired on the data-frame grid");
    CHECK((events & V90A4_RX_EVENT_RD_BAR) != 0,
          "§9.6.2.2.2 Rd-to-R̄d transition was not detected");
    if (sr == 0) {
        CHECK((events & V90A4_RX_EVENT_RT) != 0,
              "§9.6.2.1.7 Rt was not acquired after CPs silence");
        CHECK((events & V90A4_RX_EVENT_RT_BAR) != 0,
              "§9.6.2.1.8 Rt-to-R̄t transition was not detected");
    }
    CHECK(v90_analogue_phase4_rate_renegotiations(rx) == 1,
          "completed %d rate renegotiations, expected one",
          v90_analogue_phase4_rate_renegotiations(rx));
    CHECK(v90_analogue_phase4_b1d_frames(rx) == 48,
          "B1d had %d frames, expected 48",
          v90_analogue_phase4_b1d_frames(rx));
    CHECK(v90_analogue_phase4_b1d_bit_errors(rx) == 0,
          "B1d known plaintext had %d bit errors",
          v90_analogue_phase4_b1d_bit_errors(rx));
    {
        uint8_t got[64];
        int got_n = v90_analogue_phase4_get_data_bits(rx, got, sizeof(got));

        CHECK(got_n == 2*(cp.drn + 20), "decoded %d data bits, expected %d",
              got_n, 2*(cp.drn + 20));
        for (int i = 0; i < got_n; i++)
            CHECK(got[i] == (uint8_t)((i % (cp.drn + 20)) & 1),
                  "data bit %d broke B1d mapper continuity", i);
    }
    CHECK(v90_analogue_phase4_demap_failures(rx) == 0,
          "%d frames would not demap against the CPt they were mapped with",
          v90_analogue_phase4_demap_failures(rx));
    /*
     * §8.6.5 is scrambled ones, so the run before the first zero covers all of
     * TRN2d — plus §8.6.3's seventeen sync ones, since MP follows with no gap
     * and its start bit is the first zero either side can see.  That +17 is
     * the same coincidence §8.4.2's Jd plays against TRN1d, and asserting it
     * exactly is what says the two signals were told apart at the right bit.
     */
    CHECK(v90_analogue_phase4_trn2d_ones(rx)
          == trn2d_frames*bits_per_frame + MP_SYNC_BITS_EXPECTED,
          "TRN2d demapped to %d ones, expected %d",
          v90_analogue_phase4_trn2d_ones(rx),
          trn2d_frames*bits_per_frame + MP_SYNC_BITS_EXPECTED);

    if ((mp = v90_analogue_phase4_mp(rx)) != NULL) {
        CHECK(mp->max_drn == 14, "MP max drn %u, expected 14", mp->max_drn);
        CHECK(mp->acknowledge, "MP acknowledge bit not read back");
        CHECK(mp->type1 == (sr == 0), "MP type was not decoded correctly");
        if (sr == 0) {
            CHECK(mp->precoder[0][0] == 0x1234
                  &&  (uint16_t) mp->precoder[0][1] == 0xFEDC
                  &&  mp->precoder[2][0] == 0x3456,
                  "Type-1 precoder coefficients were not decoded correctly");
        }
        printf("  Ri %dT, TRN2d %dT, %d MP frame(s): max drn=%u (%d bps), "
               "rate mask 0x%04X, %s\n",
               v90_analogue_phase4_r_symbols(rx),
               v90_analogue_phase4_trn2d_symbols(rx),
               v90_analogue_phase4_mp_frames(rx),
               mp->max_drn, mp->max_drn*2400, mp->rate_mask,
               mp->acknowledge ? "MP'" : "MP");
    } else {
        printf("  FAIL: no MP decoded\n");
        failures++;
    }

    v90_analogue_phase4_free(rx);
    free(plain);
    free(stream);
}

/*
 * §8.5.2: does a measurement turn into a CPt/CP pair the digital modem could
 * act on?  The trap Table 14 sets is that CP's rate field is (drn + 20) and
 * CPt's is (drn + 8), so the same line rate is a different number in each.
 */
static void test_phase4_cp_from_measurement(void)
{
    v90_dil_measurement_t m;
    vpcm_cp_frame_t cpt;
    vpcm_cp_frame_t cp;

    printf("§8.5.2 CPt/CP from a DIL measurement\n");

    memset(&m, 0, sizeof(m));
    /* A clean line: every Ucode arrived where it was sent, in every interval. */
    for (int u = 1; u < V90_DIL_UCODES; u++) {
        m.usable[u] = true;
        m.u[u].tx_count = 6;
        m.u[u].rx_ucode = u;
        m.u[u].rx_agree = 6;
        m.u[u].rx_distinct = 1;
        m.u[u].tx_level = u*16;
        m.u[u].rx_level = u*16;
        for (int i = 0; i < 6; i++) {
            m.u[u].rx_ucode_slot[i] = u;
            m.u[u].rx_level_slot[i] = u*16;
        }
    }
    m.ucodes_measured = V90_DIL_UCODES - 1;
    m.usable_count = V90_DIL_UCODES - 1;
    m.coverage = 1.0;

    /*
     * Every Sr §5.4.5 allows, because the rate arithmetic is different in each
     * and getting it wrong is invisible from this side: a CPt that no digital
     * modem can build a mapper from still encodes, still passes its own CRC,
     * and still demaps against itself.
     */
    for (int sr = 0; sr <= 3; sr++) {
        int ld = (sr == 0) ? 0 : 1;
        v90_state_t *digital;
        int cpt_k;
        int cp_k;

        if (!v90_analogue_phase4_build_cp(&m, V90_LAW_ULAW, 0.0,
                                           sr, ld, &cpt, &cp)) {
            printf("  FAIL: Sr=%d yielded no offerable constellation\n", sr);
            failures++;
            continue;
        }
        CHECK(cpt.v90_compatibility == false, "CPt has Table 14 bit 19 set");
        CHECK(cp.v90_compatibility == true, "CP has Table 14 bit 19 clear");
        CHECK(cpt.drn <= 22, "CPt drn %u exceeds Table 14's range", cpt.drn);
        CHECK(cp.drn <= 28, "CP drn %u exceeds what vpcm_cp accepts", cp.drn);
        CHECK(cpt.shaping_redundancy == sr  &&  cp.shaping_redundancy == sr,
              "Sr=%d asked for, Sr=%u/%u built", sr,
              cpt.shaping_redundancy, cp.shaping_redundancy);
        /* §8.5.2 caps CP's average power 3 dB above CPt's; naming the same
         * constellations in both makes the difference zero by construction. */
        CHECK(memcmp(cpt.masks, cp.masks, sizeof(cpt.masks)) == 0,
              "CPt and CP name different constellations, so §8.5.2 needs checking");

        /* Table 17 and §5.4.3, which is where a wrong Sr shows up. */
        cpt_k = v90_analogue_phase4_cp_k(&cpt);
        cp_k = v90_analogue_phase4_cp_k(&cp);
        CHECK(cpt_k >= 6  &&  cpt_k <= 24,
              "Sr=%d: CPt K=%d is outside Table 17's 6..24", sr, cpt_k);
        CHECK(cp_k > 0, "Sr=%d: CP K does not fit §5.4.3", sr);
        CHECK(cpt_k == cpt.drn + 8 - (6 - sr),
              "Sr=%d: CPt K=%d does not follow §5.4.1 from D=%d",
              sr, cpt_k, cpt.drn + 8);

        /*
         * The check that matters, made by the other half of this tree rather
         * than by an assertion written next to the code under test.  §9.4.1.2
         * sends R̄i only after *receiving* a CPt, and a digital modem that
         * cannot configure a mapper from one has not received it -- which is
         * exactly how a Phase 4 stalls in Ri with the line looking perfect.
         */
        if ((digital = v90_init_data_pump(V90_LAW_ULAW)) != NULL) {
            CHECK(v90_set_phase4_cp(digital, &cpt),
                  "Sr=%d: a V.90 digital modem rejects our CPt "
                  "(drn=%u, D=%d, K=%d)",
                  sr, cpt.drn, cpt.drn + 8, cpt_k);
            CHECK(v90_set_phase4_cp(digital, &cp),
                  "Sr=%d: a V.90 digital modem rejects our CP "
                  "(drn=%u, D=%d, K=%d)",
                  sr, cp.drn, cp.drn + 20, cp_k);
            v90_free(digital);
        }
        /*endif*/
        printf("  Sr=%d: CPt drn=%u (%d bits/frame, K=%d), "
               "CP drn=%u (%d bits/frame, K=%d, %.0f bps)\n",
               sr, cpt.drn, cpt.drn + 8, cpt_k,
               cp.drn, cp.drn + 20, cp_k, vpcm_cp_drn_to_bps(cp.drn));

        /* §9.3.2.8's N=0 path has no measurement but must still reach Phase 4. */
        CHECK(v90_analogue_phase4_build_zero_dil_cp(
                  V90_LAW_ULAW, sr, ld, &cpt, &cp),
              "Sr=%d: zero-length DIL produced no conservative CP", sr);
        CHECK(cp.drn == 1 && cpt.drn == 4,
              "Sr=%d: zero-DIL rates were CP=%u CPt=%u", sr, cp.drn, cpt.drn);
        if ((digital = v90_init_data_pump(V90_LAW_ULAW)) != NULL) {
            CHECK(v90_set_phase4_cp(digital, &cpt),
                  "Sr=%d: digital modem rejects zero-DIL CPt", sr);
            CHECK(v90_set_phase4_cp(digital, &cp),
                  "Sr=%d: digital modem rejects zero-DIL CP", sr);
            v90_free(digital);
        }
    }
}

/*
 * --trace <file.ulaw> [u_info] — run the receiver over an arbitrary downstream
 * and print every stage change, rather than checking a fixture's known values.
 *
 * This is the offline half of a live call: ME_G711_CAPTURE records what the
 * analogue role actually consumed, and this replays it.  A stage trace is what
 * separates "the decoder is wrong" from "the stream is not what was sent" --
 * a receiver oscillating between TRN1d and Jd looks, in the engine's own log,
 * exactly like a receiver that has not reached Jd yet.
 */
static int trace_stream(const char *path, int u_info)
{
    v90_analogue_rx_config_t cfg;
    v90_analogue_rx_stage_t last;
    v90_analogue_rx_t *rx;
    uint8_t *data;
    long len;
    int transitions;

    if ((data = read_file(path, &len)) == NULL) {
        printf("cannot read %s\n", path);
        return 1;
    }
    memset(&cfg, 0, sizeof(cfg));
    cfg.law = V90_LAW_ULAW;
    cfg.u_info = u_info;
    /* The descriptor the engine requests by default, so a replayed live
     * capture reaches §9.3.2.9 the same way the call did. */
    (void) v90_dil_preset_load(V90_DIL_PRESET_MEASUREMENT, &cfg.dil);
    if ((rx = v90_analogue_rx_init(&cfg)) == NULL) {
        free(data);
        printf("receiver did not initialise\n");
        return 1;
    }

    printf("§9.3.2 trace of %s (%ld octets, %.1f s), U_INFO=%d\n",
           path, len, len/8000.0, u_info);
    last = v90_analogue_rx_stage(rx);
    transitions = 0;
    for (long off = 0; off < len; off += 160) {
        int n = (int) ((len - off < 160) ? (len - off) : 160);
        v90_analogue_rx_stage_t now;

        (void) v90_analogue_rx_put(rx, data + off, n);
        if ((now = v90_analogue_rx_stage(rx)) != last) {
            last = now;
            transitions++;
            /* An oscillating seam produces hundreds of these; print enough to
             * see the pattern and then just count. */
            if (transitions <= 40) {
                printf("  [%8.1f ms] -> %-10s Sd %d, S̄d %d, TRN1d %dT, Jd %d\n",
                       (off + n)/8.0,
                       v90_analogue_rx_stage_name(now),
                       v90_analogue_rx_sd_reps(rx),
                       v90_analogue_rx_sd_bar_reps(rx),
                       v90_analogue_rx_trn1d_symbols(rx),
                       v90_analogue_rx_jd_frames(rx));
            }
        }
    }
    printf("  end: %s after %d stage changes — Sd %d reps, S̄d %d reps, "
           "TRN1d %dT, Jd %d frames, DIL %dT\n",
           v90_analogue_rx_stage_name(v90_analogue_rx_stage(rx)), transitions,
           v90_analogue_rx_sd_reps(rx), v90_analogue_rx_sd_bar_reps(rx),
           v90_analogue_rx_trn1d_symbols(rx), v90_analogue_rx_jd_frames(rx),
           v90_analogue_rx_dil_symbols(rx));
    {
        const v90_dil_measurement_t *m = v90_analogue_rx_measurement(rx);

        if (m != NULL) {
            printf("  DIL: %d Ucodes measured, %d usable, gain %.2f dB, "
                   "RBS slots 0x%02X, coverage %.0f%%\n",
                   m->ucodes_measured, m->usable_count, m->gain_db,
                   m->rbs_slot_mask, 100.0*m->coverage);
        }
    }

    v90_analogue_rx_free(rx);
    free(data);
    return 0;
}

static void test_tone_b_retrain_detector(void)
{
    v90_analogue_phase3_config_t cfg;
    v90_analogue_phase3_t *p3;
    uint8_t pcm[480];
    unsigned events;

    printf("§9.5.2.2 Tone B retrain detection (>50 ms)\n");
    memset(&cfg, 0, sizeof(cfg));
    cfg.law = V90_LAW_ULAW;
    cfg.baud_rate_code = 4;
    cfg.u_info = 48;
    p3 = v90_analogue_phase3_init(&cfg);
    CHECK(p3 != NULL, "could not initialise Tone B detector fixture");
    if (p3 == NULL)
        return;
    for (int i = 0; i < (int)sizeof(pcm); i++) {
        double x = 5000.0*sin(2.0*3.14159265358979323846*1200.0*i/8000.0);
        pcm[i] = linear_to_ulaw((int16_t)x);
    }
    events = v90_analogue_phase3_rx(p3, pcm, 400); /* exactly 50 ms */
    CHECK((events & V90A_EVENT_TONE_B_RETRAIN) == 0,
          "Tone B fired at 50 ms instead of more than 50 ms");
    events = v90_analogue_phase3_rx(p3, pcm + 400, 80);
    CHECK((events & V90A_EVENT_TONE_B_RETRAIN) != 0,
          "Tone B did not fire after 60 ms");
    v90_analogue_phase3_free(p3);
}

int main(int argc, char *argv[])
{
    size_t i;

    if (argc >= 3  &&  strcmp(argv[1], "--trace") == 0)
        return trace_stream(argv[2], (argc >= 4) ? atoi(argv[3]) : 48);

    for (i = 0; i < sizeof(fixtures)/sizeof(fixtures[0]); i++)
        test_fixture(&fixtures[i]);
    for (i = 0; i < sizeof(fixtures)/sizeof(fixtures[0]); i++)
        test_driven_by_fixture(&fixtures[i]);
    test_dil_stage();
    for (int sr = 0; sr <= 3; sr++)
        test_phase4_receive(sr);
    test_phase4_cp_from_measurement();
    test_tone_b_retrain_detector();

    if (failures) {
        printf("%d failure%s\n", failures, failures == 1 ? "" : "s");
        return 1;
    }
    printf("all analogue Phase 3 receiver checks passed\n");
    return 0;
}
