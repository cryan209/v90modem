/* V.90 (09/98) §9.6.1.2: fresh strict CP reception while the existing
 * data link remains ME_DATA. Includes the engine to exercise its actual
 * capture/worker gates, not a duplicate model. No audio peer is simulated.
 * This test does not establish physical-hardware interoperability. */
#include "v90.c"
#include "v90_cp_live.c"
#include "modem_engine.c"
#include <assert.h>
/* Kept in a separate TU: the engine mirrors the public stage enum. */
void v90_reneg_test_seed_old_data(v34_state_t *s);
bool v90_reneg_test_fresh_cp(const v34_state_t *s);

static void test_recorded_cp_e(void)
{
    FILE *f = fopen("reference-data/courier-v90-reneg/cp-e.g711", "rb");
    int16_t samples[2350];
    vpcm_cp_diag_t cp;
    v90_cp_live_meta_t meta;
    assert(f);
    for (int i = 0; i < 2350; i++) {
        int codeword = fgetc(f);
        assert(codeword != EOF);
        samples[i] = ulaw_to_linear((uint8_t)codeword);
    }
    assert(fgetc(f) == EOF);
    fclose(f);
    assert(v90_cp_live_recover(samples, 2350, 0, 4, 1, false, &cp, &meta));
    assert(cp.valid && cp.crc_remainder == 0);
    assert(cp.frame.drn == 21 && !cp.frame.acknowledge);
    assert(meta.e_sample >= 2050 && meta.e_sample <= 2060);
    assert(meta.e_sample > meta.last_sample);
}

int main(void)
{
    test_recorded_cp_e();
    int16_t audio[80];
    vpcm_cp_frame_t cpt, cp;
    {
        vpcm_cp_diag_t plain, ack;
        memset(&plain, 0, sizeof(plain));
        memset(&ack, 0, sizeof(ack));
        ack.frame.acknowledge = true;
        assert(v90_cp_live_direct_candidate_better(false, &plain, &ack, 1., 0., false, false));
        assert(v90_cp_live_direct_candidate_better(true, &ack, &plain, 1., 0., false, false));
        assert(!v90_cp_live_direct_candidate_better(true, &plain, &ack, 0., 1., false, false));
        assert(v90_cp_live_direct_candidate_better(true, &ack, &ack, 0., 1., false, false));
        assert(v90_cp_live_direct_candidate_better(true, &plain, &plain, 1., 0., true, false));
        uint8_t bits[64];
        memset(bits, 1, sizeof(bits));
        plain.valid = true;
        plain.frame.v90_compatibility = true;
        assert(v90_cp_live_adjacent_e(bits, 64, 19, &plain) == 20);
        bits[37] = 0; /* ordinary 17-one CP preamble, not E */
        assert(v90_cp_live_adjacent_e(bits, 64, 19, &plain) == -1);
        bits[37] = 1;
        assert(v90_cp_live_adjacent_e(bits, 39, 19, &plain) == -1);
        plain.frame.silence_request = true; /* SCR is not E */
        assert(v90_cp_live_adjacent_e(bits, 64, 19, &plain) == -1);
        plain.frame.silence_request = false;
        plain.valid = false;
        assert(v90_cp_live_adjacent_e(bits, 64, 19, &plain) == -1);
    }
    unsigned generation;
    memset(audio, 0x12, sizeof(audio));
    pthread_mutex_init(&g_v90_cp_live_mtx, NULL);
    pthread_cond_init(&g_v90_cp_live_cond, NULL);
    /* Append requires a running consumer; no worker is needed to inspect
     * queue eligibility. Never call the worker lifecycle with this fake flag. */
    g_v90_cp_live_thread_started = true;
    g_mod = ME_MOD_V90;
    g_state = ME_TRAINING;
    g_v90 = v90_init_data_pump(V90_LAW_ULAW);
    assert(g_v90);
    vpcm_cp_init(&cpt);
    cpt.v90_compatibility = false;
    cpt.drn = 4;
    cpt.upstream_rate_mask = 0x1fff;
    vpcm_cp_enable_all_ucodes(cpt.masks[0]);
    assert(v90_set_phase4_cp(g_v90, &cpt));
    cp = cpt;
    cp.v90_compatibility = true;
    cp.drn = 8;
    assert(v90_set_phase4_cp(g_v90, &cp));
    v90_cp_live_capture_reset_locked();
    v90_cp_live_capture_append(audio, 80, true);
    v90_cp_live_note_phase4_hint_locked();
    assert(g_v90_cp_live_sample_count == 80);
    assert(g_v90_cp_live_phase4_hint == 80);
    assert(g_v90_cp_live_expected_compatibility == 0);
    assert(v90_cp_rx_window_locked());

    g_state = ME_DATA;
    assert(!v90_cp_rx_window_locked());
    v90_cp_live_capture_append(audio, 80, false);
    assert(g_v90_cp_live_sample_count == 80);
    /* Model a completed startup, then use the actual renegotiation API. */
    g_v90->training_complete = true;
    g_v90->tx_phase = V90_TX_DATA;
    assert(v90_request_rate_renegotiation(g_v90));
    assert(v90_rate_renegotiation_start(g_v90));
    generation = g_v90_cp_live_generation;
    v90_cp_live_capture_reset_locked();
    assert(g_v90_cp_live_generation != generation); /* stale worker rejected */
    assert(g_v90_cp_live_sample_count == 0);
    assert(g_v90_cp_live_phase4_hint == -1);
    assert(!g_v90_cp_live_reneg);
    v90_cp_live_note_phase4_hint_locked();
    assert(g_v90_cp_live_phase4_hint == 0);
    assert(g_v90_cp_live_expected_compatibility == 1); /* CP, not CPt */
    assert(g_v90_cp_live_reneg);
    assert(v90_cp_rx_window_locked());
    v90_cp_live_capture_append(audio, 80, false);
    assert(g_v90_cp_live_sample_count == 80); /* ME_DATA must feed fresh CP */
    assert(memcmp(audio, g_v90_cp_live_samples, sizeof(audio)) == 0);
    {
        vpcm_cp_diag_t ack;
        memset(&ack, 0, sizeof(ack));
        ack.frame.v90_compatibility = true;
        ack.frame.acknowledge = true;
        v90_cp_live_mark_accepted_locked(&ack);
    }
    assert(!g_v90_cp_live_reneg);
    assert(g_v90_cp_live_next_request == -1);
    v90_cp_live_capture_append(audio, 80, false);
    assert(g_v90_cp_live_sample_count == 80);
    /* A responder must retire the PREVIOUS acquired DATA decoder. Otherwise
     * its CP gets decoded as old data and creates a spurious rate step-down. */
    g_v34 = v34_init(NULL, 3200, 31200, false, true, NULL, NULL, NULL, NULL);
    assert(g_v34);
    v34_set_v90_mode(g_v34, 0);
    v34_set_put_phase4_bit(g_v34, v90_live_cp_bit, NULL);
    v90_reneg_test_seed_old_data(g_v34);
    enter_v90_phase4_rx_locked();
    assert(v90_reneg_test_fresh_cp(g_v34));
    assert(g_v90_cp_live_expected_compatibility == 1);
    v34_free(g_v34);
    g_v34 = NULL;
    assert(v90_handle_rx_event(g_v90, V90_RX_EVENT_RETRAIN));
    assert(!v90_cp_rx_window_locked());
    g_mod = ME_MOD_V34;
    g_state = ME_TRAINING;
    assert(!v90_cp_rx_window_locked());

    v90_free(g_v90);
    g_v90 = NULL;
    g_v90_cp_live_thread_started = false;
    pthread_cond_destroy(&g_v90_cp_live_cond);
    pthread_mutex_destroy(&g_v90_cp_live_mtx);
    puts("PASS: strict V.90 CP capture lifecycle and DATA exclusion");
    return 0;
}
