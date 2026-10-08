/* V.34 10.1.3.3: the J a duplex modem RECEIVES selects the constellation of
 * its own Phase 4 TRN, MP, MP' and E (10.1.3.2, 10.1.3.8, 10.1.3.9).  Includes
 * the production v34tx.c and calls the real static generators.  Generator
 * level only: our receiver has no 16-point Phase 4 path, so there is no
 * waveform loopback for it.
 */
#include "spandsp-master/src/v34tx.c"

#include <assert.h>
#include "spandsp-master/src/v34rx_internal.h"

static int count_primary_bit(void *user_data)
{
    ++*(int *)user_data;
    return 1;
}

static void check_low_rate_mapping_bit_count(void)
{
    const int baud[] = {3200, 3429};
    const int bits_per_data_frame[] = {192, 168}; /* Tables 7/8, 4800. */
    for (int row = 0; row < 2; row++)
    {
        int count = 0;
        int16_t frame[16];
        v34_state_t *s = v34_init(NULL, baud[row], 4800, true, false,
                                  count_primary_bit, &count, NULL, NULL);
        assert(s);
        assert(!v34_seed_tx_data(s, 2, V34_TRELLIS_16, 0, 1, NULL));
        for (int i = 0; i < s->tx.parms.p; i++)
            assert(v34_get_mapping_frame(&s->tx, frame) == 16);
        assert(count == bits_per_data_frame[row]);
        v34_free(s);
    }
}

static void check_hdx_source_mph_unused_fields(void)
{
    /* Table 23 note 2 applies to the primary source in either call role.
       Inspect serialized MPh, not just the intermediate offer structure. */
    for (int caller = 0; caller < 2; caller++)
    {
        v34_state_t *s = v34_init(NULL, 3429, 9600, caller, false,
                                  fake_get_bit, NULL, NULL, NULL);
        assert(s);
        v34_half_duplex_change_mode(s, V34_HALF_DUPLEX_SOURCE);
        s->rx.use_non_linear_encoder = true;
        s->rx.parms.expanded_shaping = true;
        prepare_mph(s);
        mph_sequence_tx(&s->tx, &s->tx.mph);
        for (int bit = 29; bit <= 32; bit++)
            assert(((s->tx.txbuf[bit >> 3] >> (bit & 7)) & 1) == 0);
        v34_half_duplex_change_mode(s, V34_HALF_DUPLEX_RECIPIENT);
        prepare_mph(s);
        mph_sequence_tx(&s->tx, &s->tx.mph);
        assert(((s->tx.txbuf[31 >> 3] >> (31 & 7)) & 1) == 1);
        assert(((s->tx.txbuf[32 >> 3] >> (32 & 7)) & 1) == 1);
        v34_free(s);
    }
}

static void check_hdx_pph_power(void)
{
    v34_state_t *s = v34_init(NULL, 3429, 9600, true, false,
                              fake_get_bit, NULL, NULL, NULL);
    assert(s);
    pph_baud_init(s);
    for (int i = 0; i < PPH_SYMBOLS; i++)
    {
        complex_sig_t x = get_pph_baud(s);
        double power = x.re*x.re + x.im*x.im;
        assert(fabs(power - TRAINING_AMP*TRAINING_AMP) < 0.001);
    }
    assert(s->tx.stage == V34_TX_STAGE_HDX_SECOND_ALT);
    v34_free(s);
}

static void check_hdx_primary_s_sequence(void)
{
    v34_state_t *s = v34_init(NULL, 3200, 9600, true, false,
                             fake_get_bit, NULL, NULL, NULL);
    assert(s);
    s->half_duplex_source = V34_HALF_DUPLEX_SOURCE;
    s->tx.half_duplex_state = V34_HALF_DUPLEX_PRIMARY_CHANNEL;
    hdx_primary_channel_enter(s);
    for (int i = 0; i < 224; i++)
    {
        complex_sig_t p = get_s_not_s_baud(s);
        assert(p.re == 0 && p.im == 0);
    }
    /* 10.1.3.7: 0,90 for 128T, then 180,270 for 16T. */
    for (int i = 0; i < 144; i++)
    {
        complex_sig_t p = get_s_not_s_baud(s);
        float sign = i < 128 ? 1.0f : -1.0f;
        assert(fabs(p.re - ((i & 1) ? 0 : sign*TRAINING_AMP)) < 0.001);
        assert(fabs(p.im - ((i & 1) ? sign*TRAINING_AMP : 0)) < 0.001);
    }
    assert(s->tx.current_getbaud == get_pp_baud);
    v34_free(s);
}

static void check_hdx_control_burst_after_silence(void)
{
    v34_state_t *clean = v34_init(NULL, 3429, 9600, true, false,
                                 fake_get_bit, NULL, NULL, NULL);
    v34_state_t *dirty = v34_init(NULL, 3429, 9600, true, false,
                                 fake_get_bit, NULL, NULL, NULL);
    assert(clean && dirty);
    for (int i = 0; i < V34_TX_FILTER_STEPS; i++)
    {
        dirty->tx.rrc_filter_re[i] = 200.0f + i;
        dirty->tx.rrc_filter_im[i] = -300.0f - i;
    }
    dirty->tx.baud_phase = 6;
    dirty->tx.rrc_filter_step = V34_TX_FILTER_STEPS - 1;
    int16_t silence[560], a[600], b[600];
    v34_state_t *states[] = {clean, dirty};
    for (int i = 0; i < 2; i++)
    {
        states[i]->tx.tone_duration = 560;
        states[i]->tx.training_stage = 0;
        states[i]->tx.hdx_pph_after_silence = true;
        assert(tx_silence(states[i], silence, 560) == 560);
        assert(states[i]->tx.stage == V34_TX_STAGE_HDX_PPH);
        for (int j = 0; j < 560; j++)
            assert(silence[j] == 0);
    }
    assert(v34_tx(clean, a, 600) == 600);
    assert(v34_tx(dirty, b, 600) == 600);
    assert(memcmp(a, b, sizeof(a)) == 0);
    v34_free(clean);
    v34_free(dirty);
}

static bool on_16point_table(complex_sig_t p)
{
    for (int i = 0; i < 16; i++)
    {
        complex_sig_t expected = training_constellation_16[i];
        expected.re *= 0.316227766f;
        expected.im *= 0.316227766f;
        if (p.re == expected.re && p.im == expected.im)
            return true;
    }
    return false;
}

static int mp_bits_per_symbol(v34_state_t *s, int j, bool reneg)
{
    int before;

    s->rx.phase3_j_trn16 = j;
    s->tx.reneg_active = reneg;
    memset(&s->tx.mp, 0, sizeof(s->tx.mp));
    s->tx.mp.type = 0;
    s->tx.mp.bit_rate_a_to_c = 9600;
    s->tx.mp.bit_rate_c_to_a = 9600;
    s->tx.mp.signalling_rate_mask = 0x1FFF;
    s->tx.txbits = mp_sequence_tx(&s->tx, &s->tx.mp);
    s->tx.txptr = 0;
    before = s->tx.txptr;
    (void) get_mp_or_mph_baud(s);
    return s->tx.txptr - before;
}

/* Exercise the native data_baud_init(), not V.90's separate handover.
 * Clause 9.6.2's feedback uses raw x(n), 9.7 projects the output, and
 * 10.1.3.1 resets all precoding memory before every B1, including 11.6.
 * A second transmitter using the V.90 seam is an ordering/reset control;
 * the nonlinear equation below is evaluated independently as well. */
static void check_native_data_entry(int calling, int rate_n, int nonlinear, int stale_history)
{
    static const int16_t h[6] = {4096, -1024, -2048, 512, 1024, -256};
    v34_state_t *native = v34_init(NULL, 3200, rate_n*2400, calling, true,
                                  fake_get_bit, NULL, NULL, NULL);
    v34_state_t *control = v34_init(NULL, 3200, rate_n*2400, calling, true,
                                   fake_get_bit, NULL, NULL, NULL);
    v34_state_t *raw = v34_init(NULL, 3200, rate_n*2400, calling, true,
                               fake_get_bit, NULL, NULL, NULL);
    assert(native && control && raw);
    assert(!v34_seed_tx_data(native, rate_n, 0, nonlinear, 1, h));
    native->tx.negotiated_rates_valid = true;
    native->tx.negotiated_rate_c_to_a = rate_n;
    native->tx.negotiated_rate_a_to_c = rate_n;
    native->rx.last_rx_mp_valid = true;
    native->rx.last_rx_mp.expanded_shaping = 1;
    native->rx.last_rx_mp.use_non_linear_encoder = nonlinear;
    if (stale_history)
    {
        for (int i = 0; i < V34_XOFF + 8; i++)
        {
            native->tx.x[i].re = 1024 + 128*i;
            native->tx.x[i].im = -512 - 64*i;
        }
    }
    data_baud_init(native);
    assert(!v34_v90_begin_tx_data(control, rate_n, 0, nonlinear, 1, h));
    /* V.90 uses GPA in either role. Native V.34 uses GPC for the caller,
       GPA for the answerer (7-1/7-2); retain that directional distinction. */
    control->tx.scrambler_tap = native->tx.scrambler_tap;
    v34_normalise_data_symbol_scale(control);
    assert(!v34_seed_tx_data(raw, rate_n, 0, 0, 1, h));
    raw->tx.scrambler_tap = native->tx.scrambler_tap;
    for (int frame = 0; frame < raw->tx.parms.p; frame++)
    {
        int16_t x[16];
        assert(v34_get_mapping_frame_state(raw, x) == 16);
        for (int k = 0; k < 8; k++)
        {
            complex_sig_t a = get_data_baud(native);
            complex_sig_t b = get_data_baud(control);
            double re = x[2*k]/128.0;
            double im = x[2*k + 1]/128.0;
            double zeta = nonlinear
                        ? 0.3125*(re*re + im*im)/control->tx.nl_avg_energy : 0;
            double phi = 1 + zeta/6 + zeta*zeta/120;
            double expected_re = re*phi*control->tx.data_symbol_scale;
            double expected_im = im*phi*control->tx.data_symbol_scale;
            if (fabs(a.re - expected_re) > 1e-5
                || fabs(a.im - expected_im) > 1e-5)
            {
                fprintf(stderr, "native V.34 B1 FAIL calling=%d N=%d nonlinear=%d stale=%d "
                        "frame=%d symbol=%d got=(%g,%g) expected=(%g,%g)\n",
                        calling, rate_n, nonlinear, stale_history, frame, k,
                        (double)a.re, (double)a.im, expected_re, expected_im);
                abort();
            }
            assert(fabs(b.re - expected_re) < 1e-5);
            assert(fabs(b.im - expected_im) < 1e-5);
        }
    }
    v34_free(native);
    v34_free(control);
    v34_free(raw);
}

/* Spec oracles independent of the matching receiver: Figure 9 subset labels,
 * Figure 10 delay-register wiring, and 9.6.2's ties towards zero. */
static void check_spec_encoder(v34_state_t *s)
{
    static const int labels[4][4] = {
        {0, 7, 4, 3}, {5, 2, 1, 6}, {4, 3, 0, 7}, {1, 6, 5, 2}
    };
    for (int row = 0; row < 4; row++)
        for (int col = 0; col < 4; col++)
        {
            complexi16_t y = {2*col - 3, 2*row - 3};
            assert(get_binary_subset_label(&y) == labels[row][col]);
        }
    for (int state = 0; state < 16; state++)
        for (int input = 0; input < 16; input++)
        {
            int output = state & 1;
            int first = output;
            int second = ((state >> 3) & 1) ^ output ^ ((input >> 1) & 1);
            int third = ((state >> 2) & 1) ^ ((input >> 1) & 1);
            int fourth = ((state >> 1) & 1) ^ (input & 1);
            assert(v34_conv16_encode_table[state][input]
                   == (first << 3 | second << 2 | third << 1 | fourth));
        }
    memset(s->tx.x, 0, sizeof(s->tx.x));
    memset(s->tx.precoder_coeffs, 0, sizeof(s->tx.precoder_coeffs));
    s->tx.precoder_coeffs[0].re = 8192; /* exactly half, exercises round ties */
    s->tx.step_2d = 0;
    for (int value = -32767; value <= 32767; value++)
    {
        s->tx.x[V34_XOFF].re = value;
        complexi16_t p = precoder_tx_filter(&s->tx);
        assert(p.re == value/2); /* odd half-integers choose smaller magnitude */
        for (int wide = 0; wide <= 1; wide++)
        {
            int quantum = wide ? 512 : 256;
            int magnitude = abs(value);
            int index = magnitude/quantum;
            if (2*(magnitude % quantum) > quantum)
                index++;
            int expected = index*(wide ? 4 : 2)*(value < 0 ? -1 : 1);
            complexi16_t x = {value, 0};
            s->tx.parms.b = wide ? 56 : 55;
            assert(quantize_tx(&s->tx, &x).re == expected);
        }
    }
}

int main(void)
{
    v34_state_t *s = v34_init(NULL, 2400, 9600, true, true, NULL, NULL, NULL, NULL);
    int symbols;

    assert(s);
    /* Table 18's exact 16-point request was overridden as "nearly tied".
       Exercise the production classifier and the selected MP generator. */
    int distance;
    assert(v34_rx_j_classify(0x8990U, &distance) == 0 && distance == 0);
    assert(v34_rx_j_classify(0x89B0U, &distance) == 1 && distance == 0);
    assert(v34_rx_j_classify(0x899FU, &distance) == 2 && distance == 0);
    assert(mp_bits_per_symbol(s, v34_rx_j_classify(0x89B0U, &distance), false) == 4);
    assert(mp_bits_per_symbol(s, 0, false) == 2);      /* J asked for 4-point */
    assert(mp_bits_per_symbol(s, -1, false) == 2);     /* J never decoded */
    assert(mp_bits_per_symbol(s, 1, false) == 4);      /* J asked for 16-point */
    assert(mp_bits_per_symbol(s, 1, true) == 2);       /* 11.6 is four-point */

    /* Independently grade mean symbol energy, not just table membership. */
    double mean_energy = 0;
    s->rx.phase3_j_trn16 = 1;
    s->tx.reneg_active = false;
    for (int i = 0; i < 16; i++)
    {
        complex_sig_t p = phase4_training_point(s, i);
        mean_energy += p.re*p.re + p.im*p.im;
    }
    mean_energy /= 16;
    double nominal = TRAINING_SCALE(TRAINING_AMP);
    assert(fabs(mean_energy/(nominal*nominal) - 1.0) < 0.002);
    /* E: 20 bits = ten 4-point symbols or five 16-point ones, all on the
     * 16-point table when 16-point. */
    for (int j = 0; j <= 1; j++)
    {
        s->rx.phase3_j_trn16 = j;
        s->tx.reneg_active = false;
        s->tx.scramble_reg = 0;
        s->tx.diff = 0;
        e_baud_init(s);
        symbols = 0;
        while (s->tx.current_getbaud == get_e_baud  &&  symbols < 40)
        {
            complex_sig_t p = get_e_baud(s);

            symbols++;
            assert(j ? on_16point_table(p) : true);
        }
        assert(symbols == (j ? 5 : 10));
    }
    check_spec_encoder(s);
    v34_free(s);
    /* Table 22 bit 30 selects half-duplex Phase 3 TRN size. Grade the
     * actual generator over a long scrambled sequence: 10.2.3 requires
     * the same selected line power for either constellation. */
    v34_state_t *hdx = v34_init(NULL, 3429, 9600, true, false,
                               fake_get_bit, NULL, NULL, NULL);
    assert(hdx);
    for (int point16 = 0; point16 <= 1; point16++)
    {
        hdx->tx.stage = V34_TX_STAGE_TRN;
        hdx->tx.tone_duration = 0;
        hdx->tx.scramble_reg = 0;
        hdx->tx.infoh.trn16 = point16;
        hdx->rx.infoh.baud_rate = V34_BAUD_RATE_3429;
        hdx->rx.infoh.length_of_trn = 127;
        double energy = 0;
        for (int i = 0; i < 8192; i++)
        {
            complex_sig_t p = get_trn_baud(hdx);
            energy += p.re*p.re + p.im*p.im;
        }
        assert(fabs(energy/8192/(nominal*nominal) - 1.0) < 0.025);
    }
    v34_free(hdx);
    puts("PASS: half-duplex Phase 3 TRN preserves selected power in both constellations (10.2.3)");
    check_hdx_control_burst_after_silence();
    check_hdx_source_mph_unused_fields();
    check_hdx_pph_power();
    check_hdx_primary_s_sequence();
    check_low_rate_mapping_bit_count();
    puts("PASS: 8.2/9.3.2 low-rate mapping consumes the normative bit count");
    puts("PASS: 10.2.4.5 PPh preserves nominal control-channel power");
    puts("PASS: Table 23 note 2 source MPh bits 29:32 are zero in both call roles");
    puts("PASS: 12.4.1.1 control burst cannot inherit primary pulse-shaper history");
    for (int calling = 0; calling <= 1; calling++)
        for (int rate_n = 5; rate_n <= 13; rate_n += 4)
            for (int nonlinear = 1; nonlinear >= 0; nonlinear--)
                for (int stale_history = 0; stale_history <= 1; stale_history++)
                    check_native_data_entry(calling, rate_n, nonlinear, stale_history);
    puts("PASS: Table 18 J classification selects 16-point and Phase 4 preserves nominal power");
    puts("PASS: V.34 Phase 4 MP/E constellation follows the received J");
    puts("PASS: native V.34 B1 resets precoder history and projects output (9.6.2/9.7/10.1.3.1)");
    puts("PASS: Figure 9/10 encoder and 9.6.2 rounding match independent spec oracles");
    return 0;
}
