/* V.90 9.6.1.2.3-.6: the digital modem's side of a rate renegotiation that
 * the analogue modem opens with echo reconditioning requested (CPs, Figure
 * 10).  Includes v90.c so the transmit state machine can be driven and
 * inspected directly; no audio is involved, the checks are on codewords.
 *
 *   MP ... CPs -> MP' ... CPs' -> (current MP') Ed, Ucode-0 silence on the
 *   frame grid ... CP (bit 30 clear) -> Rt 384T, Rt-bar 24T on a frame
 *   boundary -> MP (no TRN2d) ... CP' -> Ed -> B1d -> DATA.
 */
#include "v90.c"

#include <assert.h>

static uint8_t next_cw(v90_state_t *s) { return v90_phase3_codeword(s); }

static void run_until(v90_state_t *s, v90_tx_phase_t want, int cap)
{
    for (int i = 0; i < cap && s->tx_phase != want; i++)
        (void)next_cw(s);
    assert(s->tx_phase == want);
}

static vpcm_cp_frame_t make_cp(bool silence, bool ack)
{
    vpcm_cp_frame_t cp;

    vpcm_cp_init(&cp);
    cp.v90_compatibility = true;
    cp.drn = 8;
    cp.silence_request = silence;
    cp.acknowledge = ack;
    cp.upstream_rate_mask = 0x1FFF;
    vpcm_cp_enable_all_ucodes(cp.masks[0]);
    return cp;
}

/* §9.6.1: downstream B1d completion must not discharge the upstream-E
 * deadline. Drive real mapped DATA frames after CP', without supplying E. */
/* e_at: 0 = absent, 1 = after B1d, 2 = while still transmitting MP'. */
static void test_missing_e(v90_law_t law, int e_at)
{
    v90_state_t *s = v90_init_data_pump(law);
    vpcm_cp_frame_t cp = make_cp(false, false);
    vpcm_cp_frame_t cpa = make_cp(false, true);
    uint8_t cw[V90_FRAME_LEN];

    assert(s);
    {
        vpcm_cp_frame_t cpt = make_cp(false, false);
        cpt.v90_compatibility = false;
        cpt.drn = 4;
        assert(v90_set_phase4_cp(s, &cpt));
        assert(v90_set_phase4_cp(s, &cp));
    }
    s->training_complete = true;
    s->tx_phase = V90_TX_DATA;
    /* Model the E latch retained from a successful preceding exchange. */
    s->e_received = true;
    assert(v90_request_rate_renegotiation(s));
    assert(v90_rate_renegotiation_start(s));
    assert(!s->e_received);
    run_until(s, V90_TX_MP, 20000);
    assert(v90_set_phase4_cp(s, &cp));
    assert(v90_set_phase4_cp(s, &cpa));
    v90_rate_renegotiation_note_answer(s);
    if (e_at == 2)
        assert(v90_handle_rx_event(s, V90_RX_EVENT_E));
    run_until(s, V90_TX_DATA, 12000);
    /* Engine recovery gates must remain closed while the receiver awaits E,
     * even though the downstream transmitter is already in DATA. */
    assert(v90_rate_renegotiation_active(s) == (e_at != 2));
    assert(!v90_rate_renegotiation_timed_out(s));
    if (e_at != 2)
        assert(!v90_request_rate_renegotiation(s));
    if (e_at == 1) {
        assert(v90_handle_rx_event(s, V90_RX_EVENT_E));
        assert(s->tx_phase == V90_TX_DATA);
    }
    if (e_at)
        assert(!v90_rate_renegotiation_active(s));
    while (s->reneg_symbol_clock - s->reneg_rbar_symbol
           + V90_FRAME_LEN <= V90_RENEG_E_TIMEOUT_SYMBOLS) {
        assert(v90_tx_data_frame_codewords(s, cw, NULL, 0, NULL, true)
               == V90_FRAME_LEN);
        assert(!v90_rate_renegotiation_timed_out(s));
        if (e_at)
            break;  /* E stops the deadline clock. */
    }
    assert(v90_tx_data_frame_codewords(s, cw, NULL, 0, NULL, true)
           == V90_FRAME_LEN);
    assert(v90_rate_renegotiation_timed_out(s) == !e_at);
    for (int i = 0; i < V90_RENEG_E_TIMEOUT_SYMBOLS / V90_FRAME_LEN + 1; i++)
        assert(v90_tx_data_frame_codewords(s, cw, NULL, 0, NULL, true)
               == V90_FRAME_LEN);
    assert(v90_rate_renegotiation_timed_out(s) == !e_at);
    if (e_at)
        assert(v90_request_rate_renegotiation(s));

    /* §9.5/§9.6.1: a full restart supersedes both a pending request and a
     * failed E wait. Exercise the two public restart entry points. */
    if (law == V90_LAW_ULAW)
        assert(v90_handle_rx_event(s, V90_RX_EVENT_RETRAIN));
    else
        v90_start_phase3(s, s->u_info);
    assert(!v90_rate_renegotiation_active(s));
    assert(!v90_rate_renegotiation_pending(s));
    assert(!v90_rate_renegotiation_waiting_for_e(s));
    assert(!v90_rate_renegotiation_timed_out(s));
    assert(!v90_training_complete(s));
    assert(v90_rate_renegotiation_count(s) == 1);  /* preserve per-call cap */
    assert(s->reneg_rbar_symbol == -1);
    assert(s->reneg_symbol_clock == 0);
    for (int i = 0; i < V90_RENEG_E_TIMEOUT_SYMBOLS; i++)
        (void)next_cw(s);
    assert(!v90_rate_renegotiation_timed_out(s));
    v90_free(s);
}

/* §8.6.4: startup Ri is U_INFO, but Rd MUST use the highest codeword in
 * each of the preceding data constellations. Checking against ri_codeword()
 * itself concealed the old U_INFO-for-Rd defect. Use independent expectations
 * with six distinct constellations, both laws and the complete 384T/24T. */
static void test_rd_magnitudes(v90_law_t law)
{
    v90_state_t *s = v90_init_data_pump(law);
    vpcm_cp_frame_t cpt = make_cp(false, false);
    vpcm_cp_frame_t cp = make_cp(false, false);
    assert(s);
    cpt.v90_compatibility = false;
    cpt.drn = 4;
    assert(v90_set_phase4_cp(s, &cpt));
    cp.constellation_count = 6;
    memset(cp.masks, 0, sizeof(cp.masks));
    for (int i = 0; i < 6; i++) {
        cp.dfi[i] = i;
        for (int u = 20 + 12*i; u <= 35 + 12*i; u++)
            vpcm_cp_mask_set(cp.masks[i], u, true);
    }
    assert(v90_set_phase4_cp(s, &cp));
    /* Leave genuine nonzero mapper memories, as the preceding startup does. */
    for (int i = 0; i < 606; i++)
        (void)v90_phase4_codeword(s, V90_PHASE4_INPUT_ONES);
    for (int i = 0; i < 6; i++)
        assert(v90_ri_codeword(s, i, false)
               == v90_codeword_compose(law, s->u_info, i < 3));
    s->training_complete = true;
    s->tx_phase = V90_TX_DATA;
    assert(v90_request_rate_renegotiation(s));
    assert(v90_rate_renegotiation_start(s));
    /* A later CP cannot change an Rd already in progress. */
    memset(s->data_cp_frame.masks, 0, sizeof(s->data_cp_frame.masks));
    for (int i = 0; i < 384; i++)
        assert(next_cw(s) == v90_codeword_compose(law, 35+12*(i%6), i%6 < 3));
    for (int i = 0; i < 24; i++)
        assert(next_cw(s) == v90_codeword_compose(law, 35+12*(i%6), i%6 >= 3));
    {
        v90_state_t *reference = v90_init_data_pump(law);
        assert(reference && v90_set_phase4_cp(reference, &cpt));
        /* §8.6.5: first TRN2d frame is the zero-initialized mapping, not
         * the continuation of the previous exchange's mapper memories. */
        for (int i = 0; i < V90_FRAME_LEN; i++)
            assert(next_cw(s) == v90_phase4_codeword(reference, V90_PHASE4_INPUT_ONES));
        v90_free(reference);
    }
    run_until(s, V90_TX_MP, 20000);
    /* §9.6 explicitly permits a new rate: do not reject a fresh CP as a
     * mismatching repeat of the preceding startup's CP. */
    cp.drn = 9;
    assert(v90_set_phase4_cp(s, &cp));
    assert(v90_data_bits_per_frame(s) == 29);
    cp.acknowledge = true;
    assert(v90_set_phase4_cp(s, &cp));
    v90_free(s);
}

int main(void)
{
    test_rd_magnitudes(V90_LAW_ULAW);
    test_rd_magnitudes(V90_LAW_ALAW);
    for (int e_at = 0; e_at < 3; e_at++) {
        test_missing_e(V90_LAW_ULAW, e_at);
        test_missing_e(V90_LAW_ALAW, e_at);
    }
    v90_state_t *s = v90_init_data_pump(V90_LAW_ULAW);
    vpcm_cp_frame_t cpt, cp, cps, cpsa, cpa;
    const uint8_t silence = v90_pcm_signed_codeword(V90_LAW_ULAW, 0, 1);

    assert(s);
    vpcm_cp_init(&cpt);
    cpt.v90_compatibility = false;
    cpt.drn = 4;
    cpt.upstream_rate_mask = 0x1FFF;
    vpcm_cp_enable_all_ucodes(cpt.masks[0]);
    assert(v90_set_phase4_cp(s, &cpt));
    cp = make_cp(false, false);
    cps = make_cp(true, false);
    cpsa = make_cp(true, true);
    cpa = make_cp(false, true);

    /* A CPs outside a renegotiation is not something this procedure can act on. */
    assert(!v90_set_phase4_cp(s, &cps));
    assert(v90_set_phase4_cp(s, &cp));          /* the startup data CP */
    s->training_complete = true;
    s->tx_phase = V90_TX_DATA;
    assert(v90_request_rate_renegotiation(s));
    assert(v90_rate_renegotiation_start(s));

    /* Rd 384T, Rd-bar 24T, TRN2d, MP. */
    run_until(s, V90_TX_MP, 20000);
    assert(!s->reneg_silence_req);

    /* CPs: MP' follows; CPs' releases Ed. */
    assert(v90_set_phase4_cp(s, &cps));
    assert(s->reneg_silence_req && s->data_cp_received && !s->cp_ack_received);
    assert(v90_set_phase4_cp(s, &cpsa));
    assert(s->cp_ack_received);
    run_until(s, V90_TX_ED, 5000);
    run_until(s, V90_TX_RENEG_SILENCE, 5000);

    /* Silence: Ucode 0 only, and it stays on the data frame grid. */
    for (int i = 0; i < 60; i++)
        assert(next_cw(s) == silence);
    assert(s->tx_phase == V90_TX_RENEG_SILENCE);
    /* Late copies of CPs/CPs' do not end it; a CP with bit 30 clear does. */
    assert(!v90_set_phase4_cp(s, &cps));
    assert(!v90_set_phase4_cp(s, &cpsa));
    assert(s->tx_phase == V90_TX_RENEG_SILENCE);
    for (int i = 0; i < 3; i++)                 /* off the frame boundary */
        assert(next_cw(s) == silence);
    assert(v90_set_phase4_cp(s, &cp));
    {
        int before = s->sample_count;
        int to_boundary = (V90_FRAME_LEN - before % V90_FRAME_LEN) % V90_FRAME_LEN;

        assert(to_boundary != 0);
        for (int i = 0; i < to_boundary; i++)
            assert(next_cw(s) == silence);      /* Rt waits for the frame */
    }

    /* Rt: 384T of the Ri pattern at U_INFO, Rt-bar: 24T of it inverted. */
    for (int i = 0; i < V90_RD_RENEG_SYMBOLS; i++)
        assert(next_cw(s) == v90_ri_codeword(s, i, false));
    for (int i = 0; i < V90_RI_POST_CP_SYMBOLS; i++)
        assert(next_cw(s) == v90_ri_codeword(s, i, true));
    /* Then MP directly -- no TRN2d. */
    assert(s->tx_phase == V90_TX_MP);
    assert(!s->cp_ack_received && !s->data_cp_received);
    assert(!s->reneg_silence_req && !s->reneg_rt);

    /* 9.4.1.4: CP, MP', CP', Ed, B1d, data. */
    assert(v90_set_phase4_cp(s, &cp));
    assert(v90_set_phase4_cp(s, &cpa));
    run_until(s, V90_TX_ED, 5000);
    run_until(s, V90_TX_B1D, 5000);
    run_until(s, V90_TX_DATA, 5000);
    assert(!s->reneg_active && s->training_complete);

    /* A peer may retrain while our next Rd is still being sent (§9.6.1).
     * Cancel the private transmit-sequence state as well as the E wait. */
    assert(v90_handle_rx_event(s, V90_RX_EVENT_E));
    assert(v90_request_rate_renegotiation(s));
    assert(v90_rate_renegotiation_start(s));
    (void)next_cw(s);
    assert(s->reneg_active);
    assert(v90_handle_rx_event(s, V90_RX_EVENT_RETRAIN));
    assert(!s->reneg_active && !v90_rate_renegotiation_active(s));
    assert(!s->reneg_silence_req && !s->reneg_rt_pending && !s->reneg_rt);
    assert(!v90_training_complete(s));
    assert(v90_rate_renegotiation_count(s) == 2);
    assert(v90_trn2d_symbols_for(s) == v90_trn2d_symbols());

    v90_free(s);
    puts("PASS: V.90 9.6.1 upstream E deadline (both laws) and 9.6.1.2.3-.6 CPs silence procedure");
    return 0;
}
