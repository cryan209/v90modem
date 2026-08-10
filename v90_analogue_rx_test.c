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

int main(void)
{
    size_t i;

    for (i = 0; i < sizeof(fixtures)/sizeof(fixtures[0]); i++)
        test_fixture(&fixtures[i]);
    for (i = 0; i < sizeof(fixtures)/sizeof(fixtures[0]); i++)
        test_driven_by_fixture(&fixtures[i]);
    test_dil_stage();

    if (failures) {
        printf("%d failure%s\n", failures, failures == 1 ? "" : "s");
        return 1;
    }
    printf("all analogue Phase 3 receiver checks passed\n");
    return 0;
}
