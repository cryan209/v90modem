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

int main(void)
{
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

    v90_free(s);
    puts("PASS: V.90 9.6.1.2.3-.6 CPs silence procedure");
    return 0;
}
