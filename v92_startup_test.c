/* V.92 full Phase 2 (9.3, Tables 15-17) and coupled Phase 3 (9.5).
 * Two independent Phase-2 modems exchange only samples through a G.711
 * codec boundary. Inspecting received fields grades the wire, not TX intent.
 */
#define SPANDSP_EXPOSE_INTERNAL_STRUCTURES
#include <spandsp.h>
#include "v92_trn2u.h"
#include "v92_p3_rx.h"
#include "v92_analogue_phase3.h"
#include "v92_analogue_audio.h"
#include "v92_su.h"
#include "v90_dil_presets.h"
#include "v92_upstream_rx.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static int marks(void *user) { (void)user; return 1; }
static void discard(void *user, int bit) { (void)user; (void)bit; }

static int test_phase2(bool alaw, bool capable, bool request_short, bool pcm)
{
    v34_state_t *analogue = v34_init(NULL, 3200, 9600, true, true,
                                     marks, NULL, discard, NULL);
    v34_state_t *digital = v34_init(NULL, 3200, 9600, false, true,
                                    marks, NULL, discard, NULL);
    assert(analogue && digital);
    v34_set_v90_mode(analogue, alaw);
    v34_set_v90_mode(digital, alaw);
    v34_set_v92_info0_capabilities(analogue, capable, request_short);
    v34_set_v92_info0_capabilities(digital, 1, 0);
    v34_set_v92_pcm_upstream_capability(digital, 1);
    v34_set_v92_pcm_upstream_capability(analogue, pcm);
    v34_tx_power(analogue, -12.0f);
    v34_tx_power(digital, -12.0f);

    for (int block = 0; block < 1000; block++) {
        int16_t upstream[80], downstream[80];
        int na = v34_tx(analogue, upstream, 80);
        int nd = v34_tx(digital, downstream, 80);
        assert(na == 80 && nd == 80);
        for (int i = 0; i < 80; i++) {
            /* Network A/D upstream; digital tone generator's codec output
             * followed by network D/A downstream. Exactly one DS0 interval
             * per sample. No symbol/event injection between the modems. */
            uint8_t u = alaw ? linear_to_alaw(upstream[i]) : linear_to_ulaw(upstream[i]);
            uint8_t d = alaw ? linear_to_alaw(downstream[i]) : linear_to_ulaw(downstream[i]);
            upstream[i] = alaw ? alaw_to_linear(u) : ulaw_to_linear(u);
            downstream[i] = alaw ? alaw_to_linear(d) : ulaw_to_linear(d);
        }
        v34_rx(digital, upstream, 80);
        v34_rx(analogue, downstream, 80);
        if (digital->rx.info1a_received)
            break;
    }
    v34_v90_info0a_t received;
    assert(v34_get_v90_received_info0a(digital, &received));
    assert(received.raw_26_27 == ((capable ? 1 : 0) | (request_short ? 2 : 0)));
    assert(analogue->rx.info0_raw_26_27 == 2);
    assert(analogue->rx.info1c_received);
    v34_v90_info1a_t selection;
    assert(v34_get_v90_received_info1a(digital, &selection));
    /* Grade the received Table 18 selection, not a transmitter mode flag. */
    assert(selection.downstream_rate_code == 6);
    if (pcm && capable) {
        assert(selection.upstream_symbol_rate_code == 6);
        assert(digital->rx.info1a_raw_12_17 == 0);
        assert(digital->rx.info1a_raw_40_49 == 0x3FF);
    } else {
        assert(selection.upstream_symbol_rate_code >= 3 && selection.upstream_symbol_rate_code <= 5);
    }
    assert(digital->tx.v92_info1d_mode == capable);
    /* On the wire bit 70 occupies the old 3429 carrier-bit position.
     * Table 17 uses it for PCM-upstream support; a mode flag in the sender
     * alone cannot prove it transmitted the right layout. */
    assert(analogue->rx.info1c.rate_data[5].use_high_carrier == capable);
    v34_free(analogue);
    v34_free(digital);
    printf("PASS: full Phase 2 %s capability=%d peer-short=%d\n",
           alaw ? "PCMA" : "PCMU", capable, request_short);
    return selection.u_info;
}

static void test_linear(void)
{
    for (int points = 2; points <= 8; points *= 2) {
        v92_trn2u_tx_t linear, other_law, wire, chunks;
        uint8_t bits[144], codewords[144];
        int16_t samples[144], other[144], split[144];
        for (int i = 0; i < 144; i++)
            bits[i] = (i*13 + i/7) & 1;
        v92_trn2u_tx_init(&linear, points, 6123.0, false);
        v92_trn2u_tx_start(&linear, 1);
        other_law = wire = chunks = linear;
        other_law.alaw = true;
        int n = v92_trn2u_tx_bits_linear(&linear, bits, 144, samples, 144);
        assert(n == 144/v92_trn2u_bits_per_symbol(points));
        assert(v92_trn2u_tx_bits_linear(&other_law, bits, 144, other, 144) == n);
        assert(!memcmp(samples, other, n*sizeof(*samples)));
        assert(v92_trn2u_tx_bits(&wire, bits, 144, codewords, 144) == n);
        int bps = v92_trn2u_bits_per_symbol(points);
        for (int i = 0; i < n; i++) {
            assert(v92_trn2u_tx_bits_linear(&chunks, bits + i*bps, bps, split+i, 1) == 1);
            assert(codewords[i] == linear_to_ulaw(samples[i]));
            /* Table 28/29: amplitudes are odd multiples of LU/sqrt(5/21).
             * Phase-3 two-point CPt is +/-LU. No G.711 quantization here. */
            double unit = points == 2 ? 6123.0 : 6123.0/sqrt(points == 4 ? 5.0 : 21.0);
            int label = (int)lround(fabs((double)samples[i])/unit);
            assert(label > 0 && label < points && (label & 1));
            assert(fabs(fabs((double)samples[i]) - label*unit) <= 0.5);
        }
        assert(!memcmp(samples, split, n*sizeof(*samples)));
        assert(linear.scramble_reg == chunks.scramble_reg && linear.prev_sign == chunks.prev_sign);
        chunks = linear;
        assert(v92_trn2u_tx_bits_linear(&linear, bits, 144, samples, n-1) == 0);
        assert(!memcmp(&linear, &chunks, sizeof(linear)));
    }
    puts("PASS: analogue linear PCM amplitudes, law independence and chunk continuity");
}

static void test_audio(void)
{
    const double pi = 3.14159265358979323846;
    for (int factor = 3; factor <= 6; factor += 3) {
        v92_pcm_interpolator_t fir;
        assert(v92_pcm_interpolator_init(&fir, factor));
        /* Bound ALL possible int16 input histories, including a full-scale
         * alternating waveform. Headroom must survive reconstruction peaks. */
        for (int p = 0; p < factor; p++) {
            double bound = 0;
            for (int j = 0; j < V92_AUDIO_INTERPOLATOR_TAPS; j++)
                bound += fabs(fir.coefficients[p][j])*32768;
            assert(bound < 32767);
        }
        double error = 0, energy = 0;
        for (int n = 0; n < 1600; n++) {
            int16_t out[6];
            double input_rate = (double)V92_AUDIO_RATE/factor;
            int16_t input = (int16_t)lround(20000*sin(2*pi*3000*n/input_rate));
            assert(v92_pcm_interpolator_put(&fir, input, out) == factor);
            if (n < 32) continue;
            for (int p = 0; p < factor; p++) {
                double expected = 5000*sin(2*pi*3000*(n-8+(double)p/factor)/input_rate);
                error += (out[p]-expected)*(out[p]-expected);
                energy += expected*expected;
            }
        }
        assert(sqrt(error/energy) < 0.03);
        assert(fir.clipped == 0);
        /* The network DAC must preserve every G.711 level on the sampling
         * lattice after its eight-input delay, without re-encoding bytes. */
        for (int law = 0; law < 2; law++) {
            assert(v92_pcm_interpolator_init(&fir, factor));
            int16_t reference[256];
            for (int n = 0; n < 264; n++) {
                int16_t out[6];
                if (n < 256) reference[n] = law ? alaw_to_linear(n) : ulaw_to_linear(n);
                v92_pcm_interpolator_put(&fir, n < 256 ? reference[n] : 0, out);
                if (n >= 8) assert(out[0]*V92_AUDIO_LINEAR_SCALE == reference[n-8]);
            }
            assert(fir.clipped == 0);
        }
    }
    v92a_config_t cfg = {
        .law = V90_LAW_ULAW, .u_info = 78, .lu = 6000,
        .digital_max_tx_dbm0 = -13, .upstream_rate_mask = 1,
        .dil = {.n = 0, .lsp = 1, .ltp = 1}
    };
    v92a_audio_t *whole = v92a_audio_init(&cfg, 0);
    v92a_audio_t *chunks = v92a_audio_init(&cfg, 0);
    assert(whole && chunks);
    int16_t a[19001], b[19001];
    assert(v92a_audio_tx(whole, a, 19001) == 19001);
    for (int i = 0; i < 19001;) {
        int count = 1 + i%127;
        if (count > 19001-i) count = 19001-i;
        assert(v92a_audio_tx(chunks, b+i, count) == count);
        i += count;
    }
    assert(memcmp(a, b, sizeof(a)) == 0);
    assert(v92a_audio_clipped(whole) == 0 && v92a_audio_clipped(chunks) == 0);
    v92a_audio_free(whole);
    v92a_audio_free(chunks);
    assert(!v92a_audio_init(&cfg, 6));
    puts("PASS: 48 kHz reconstruction, full-scale headroom, both G.711 ladders and arbitrary TX chunks");
}

static void test_trn1u(void)
{
    enum { TRAIN = 2040, JA_PREAMBLE = 24 };
    v92_trn2u_tx_t tx, split;
    int16_t samples[TRAIN + JA_PREAMBLE], chunks[TRAIN];
    uint8_t scrambled[TRAIN + JA_PREAMBLE], ones[JA_PREAMBLE];
    memset(ones, 1, sizeof(ones));
    v92_trn2u_tx_init(&tx, 2, 6123.0, false);
    v92_trn2u_tx_start(&tx, 0);
    split = tx;
    assert(v92_trn1u_tx_linear(&tx, samples, TRAIN) == TRAIN);
    assert(v92_trn2u_tx_bits_linear(&tx, ones, JA_PREAMBLE,
                                    samples + TRAIN, JA_PREAMBLE) == JA_PREAMBLE);
    for (int i = 0; i < TRAIN; i++)
        assert(v92_trn1u_tx_linear(&split, chunks+i, 1) == 1);
    assert(!memcmp(samples, chunks, sizeof(chunks)));
    int wire_sign = 0;
    for (int i = 0; i < TRAIN + JA_PREAMBLE; i++) {
        /* Independent delay-line oracle for GPA (6.3), not the production
         * shift-register code; TRN1u has absolute signs, Ja differential. */
        scrambled[i] = 1 ^ (i >= 5 ? scrambled[i-5] : 0)
                         ^ (i >= 23 ? scrambled[i-23] : 0);
        wire_sign = i < TRAIN ? scrambled[i] : wire_sign ^ scrambled[i];
        assert(samples[i] == (wire_sign ? -6123 : 6123));
    }
    puts("PASS: analogue TRN1u absolute GPA signs and differential Ja handoff");
}

static uint8_t network_adc(bool alaw, int16_t sample)
{
    return alaw ? linear_to_alaw(sample) : linear_to_ulaw(sample);
}

static void test_md(bool alaw, int units)
{
    v92_p3_rx_t rx;
    int t = 0;
    v92_p3_rx_start(&rx, 0);
    v92_p3_rx_set_md_length(&rx, units*276); /* Table 18, not V.90's 280 */
    for (int i = 0; i < 384; i++)
        v92_p3_rx_feed(&rx, network_adc(alaw, i%6 < 3 ? 6000 : -6000), t++);
    for (int i = 0; i < 24; i++)
        v92_p3_rx_feed(&rx, network_adc(alaw, i%6 < 3 ? -6000 : 6000), t++);
    /* An unrelated signal inside MD must not become the second Ru, even
     * when its samples are deliberately identical to that training signal. */
    for (int i = 0; i < units*276; i++) {
        int16_t sample = i < 12 ? 0 : (i%6 < 3 ? 6000 : -6000);
        v92_p3_rx_feed(&rx, network_adc(alaw, sample), t++);
        assert(rx.state != V92_P3_RX_RU2);
        assert(rx.last_reject != V92_P3_RX_REJECT_MD_TIMEOUT);
    }
    int expected_ru2 = t;
    for (int i = 0; i < 384; i++)
        v92_p3_rx_feed(&rx, network_adc(alaw, i%6 < 3 ? 6000 : -6000), t++);
    assert(rx.state == V92_P3_RX_RU2);
    /* The soft Ru-bar detector confirms the boundary after it occurs.
     * Its acquisition latency must not be mistaken for transmitter timing. */
    assert(rx.ru2_start >= expected_ru2);
    assert(rx.ru2_start == rx.ur1_end + 1 + units*276);
    assert(rx.ru2_start - expected_ru2 < 48);
    printf("PASS: MD=%d x 276 symbols, %s, no premature Ru or timeout\n",
           units, alaw ? "PCMA" : "PCMU");
}

typedef struct {
    v90_state_t *digital;
    int cpt_count, e1u_count, cpu_count;
    bool b1_armed;
    v92_upstream_rx_t b1;
    unsigned payload_bits, payload_bytes, payload_errors;
} p3_pair_sink_t;

static uint8_t payload_byte(unsigned n)
{
    return (uint8_t)((n*73) ^ (n>>3) ^ 0xa5);
}

static int pair_payload_bit(void *user)
{
    p3_pair_sink_t *sink = user;
    unsigned n = sink->payload_bits++;
    return (payload_byte(n/8) >> (n%8)) & 1;
}

static void pair_payload_byte(void *user, uint8_t byte)
{
    p3_pair_sink_t *sink = user;
    sink->payload_errors += byte != payload_byte(sink->payload_bytes++);
}

static void pair_cpt(void *user, v92_p4u_kind_t kind,
                     const v92_cp_diag_t *cp, const v92_cpus_diag_t *cpus,
                     const v92_suvu_diag_t *suvu)
{
    (void)cpus;
    p3_pair_sink_t *sink = user;
    if (kind == V92_P4U_KIND_SUVU && suvu)
        assert(v90_set_v92_suvu(sink->digital, suvu->frame.acknowledge));
    if (kind == V92_P4U_KIND_CPU && cp) {
        vpcm_cp_frame_t mapped;
        assert(v92_cp_frame_to_vpcm(&cp->frame, &mapped));
        assert(v90_set_v92_cpu(sink->digital, &mapped));
        if (!sink->b1_armed) {
            v92_cpd_frame_t profile;
            assert(v90_build_v92_cpd_frame(sink->digital, &profile));
            assert(v92_upstream_b1_rx_init(&sink->b1, &profile, pair_payload_byte, sink));
            sink->b1_armed = true;
        }
        sink->cpu_count++;
    }
    if (kind == V92_P4U_KIND_E1U) {
        (void)v90_handle_rx_event(sink->digital, V90_RX_EVENT_E);
        sink->e1u_count++;
    }
    if (kind == V92_P4U_KIND_CPT && cp) {
        vpcm_cp_frame_t mapped;
        assert(v92_cp_frame_to_vpcm(&cp->frame, &mapped));
        assert(v90_set_phase4_cp(sink->digital, &mapped));
        (void)v90_handle_rx_event(sink->digital, V90_RX_EVENT_CP_VALID);
        sink->cpt_count++;
    }
}

static void test_phase3_pair(bool alaw, bool dil, bool drop_cpd, unsigned audio_rate)

{
    bool audio = audio_rate != 0;
    v92a_config_t cfg = {
        .law = alaw ? V90_LAW_ALAW : V90_LAW_ULAW, .u_info = 78,
        .lu = 6000, .digital_max_tx_dbm0 = -13, .upstream_rate_mask = 1,
        .dil = {.n = 0, .lsp = 1, .ltp = 1}
    };
    cfg.u_info = test_phase2(alaw, true, false, true);
    if (dil) assert(v90_dil_preset_load(V90_DIL_PRESET_MEASUREMENT, &cfg.dil));
    /* TX reconstruction adds four symbols; network DAC adds eight. */
    if (audio) cfg.round_trip_symbols = 12;
    v92a_audio_t *frontend = audio ? v92a_audio_init_rate(&cfg, audio_rate) : NULL;
    v92a_t *analogue = audio ? v92a_audio_core(frontend) : v92a_init(&cfg);
    v92_pcm_interpolator_t dac;
    assert(v92_pcm_interpolator_init(&dac, 6));
    v90_state_t *digital = v90_init_data_pump(cfg.law);
    assert(analogue && digital);
    v90_enable_v92_phase3(digital);
    v90_enable_v92_mode(digital);
    v90_enable_v92_native_cpu_rx(digital);
    assert(v90_set_v92_cpd_profile(digital, 1, 0, 0x8000));
    v90_start_phase3(digital, cfg.u_info);
    v92_p3_rx_t ja_rx;
    v92_su_t su_rx;
    v92_cp_rx_t cpt_rx;
    v92_trn2u_demod_t cpt_demod, p4_demod;
    v92_cp_rx_t p4_rx;
    bool p4_started = false;
    p3_pair_sink_t sink = {.digital = digital};
    bool ja_seen = false, cpt_started = false;
    v92_p3_rx_start(&ja_rx, 0);
    v92_su_init(&su_rx, alaw);

    bool dropped_cpd = false, dropping_cpd = false;
    bool trace_audio = getenv("V92_AUDIO_TRACE") != NULL;
    int audio_count = audio ? (int)audio_rate/8000 : 0;
    assert(!audio || (audio_count > 0 && audio_rate%8000 == 0));
    for (int i = 0; i < 160000; i++) {
        int16_t upstream[6], downstream;
        uint8_t d;
        if (audio) assert(v92a_audio_tx(frontend, upstream, audio_count) == audio_count);
        else assert(v92a_tx(analogue, upstream, 2) == 2);
        int before_tx = v90_get_tx_phase(digital);
        assert(v90_phase3_tx_codewords(digital, &d, 1) == 1);
        if (drop_cpd && before_tx == V90_TX_CP && !dropped_cpd) {
            dropping_cpd = true;
            d = v90_idle_codeword(cfg.law);
        }
        if (dropping_cpd && v90_get_tx_phase(digital) != V90_TX_CP) {
            dropping_cpd = false;
            dropped_cpd = true;
        }
        /* Codec scaling is a calibration of the analogue volts, not gain
         * on the digital DS0. Quantize only at the network ADC. */
        int adc_sample = upstream[0] * (audio ? V92_AUDIO_LINEAR_SCALE : 1);
        assert(adc_sample >= -32768 && adc_sample <= 32767);
        uint8_t u = network_adc(alaw, (int16_t)adc_sample);
        downstream = alaw ? alaw_to_linear(d) : ulaw_to_linear(d);
        if (trace_audio) fprintf(stderr, "TXRAW %d %d\n", i, downstream);
        if (audio) {
            int16_t line[6];
            assert(v92_pcm_interpolator_put(&dac, downstream, line) == 6);
            int16_t local[6];
            for (int j = 0; j < audio_count; j++) local[j] = line[j*6/audio_count];
            /* Callback boundaries are independent of receiver half symbols. */
            int first = 1 + i%audio_count;
            v92a_audio_rx(frontend, local, first);
            v92a_audio_rx(frontend, local+first, audio_count-first);
        } else v92a_rx(analogue, &downstream, 1);
        if (!ja_seen) {
            v92_p3_rx_feed(&ja_rx, u, i);
            if (v92_p3_rx_ja_ok(&ja_rx)) {
                const ja_dil_decode_t *ja = v92_p3_rx_get_ja(&ja_rx);
                assert(ja && ja->parsed_v92 && ja->desc.n == cfg.dil.n);
                v90_set_dil_descriptor(digital, &ja->desc);
                assert(v90_handle_rx_event(digital, V90_RX_EVENT_J));
                ja_seen = true;
            }
        } else {
            switch (v92_su_put(&su_rx, u)) {
            case V92_SU_ACQUIRED: assert(v90_handle_rx_event(digital, V90_RX_EVENT_SU)); break;
            case V92_SU_BAR: assert(v90_handle_rx_event(digital, V90_RX_EVENT_SU_BAR)); break;
            case V92_SU_TRAINED: (void)v90_handle_rx_event(digital, V90_RX_EVENT_TRN_LOCK); break;
            case V92_SU_FINAL: assert(v90_handle_rx_event(digital, V90_RX_EVENT_SU_FINAL)); break;
            default: break;
            }
        }
        if (!cpt_started && (v90_get_tx_phase(digital) == V90_TX_SCR || v90_get_tx_phase(digital) == V90_TX_DIL)) {
            v92_cp_rx_init(&cpt_rx, 2, alaw, pair_cpt, &sink);
            v92_trn2u_demod_init(&cpt_demod, 2, cfg.lu, alaw, &cpt_rx);
            cpt_started = true;
        }
        if (sink.e1u_count && !p4_started) {
            v92_cp_rx_init(&p4_rx, 4, alaw, pair_cpt, &sink);
            v92_trn2u_demod_init(&p4_demod, 4, cfg.lu, alaw, &p4_rx);
            p4_started = true;
        }
        if (p4_started) v92_trn2u_demod_feed(&p4_demod, &u, 1);
        else if (cpt_started) v92_trn2u_demod_feed(&cpt_demod, &u, 1);
        if (sink.b1_armed) {
            int16_t sample = alaw ? alaw_to_linear(u) : ulaw_to_linear(u);
            v92_upstream_b1_rx_feed(&sink.b1, &sample, 1);
        }
        if (audio && trace_audio && i%8000 == 0)
            fprintf(stderr, "AUDIO t=%d stage=%d rx=%d acquired=%d ppm=%.1f eq=%.4f clips=%llu\n", i/8000,
                    v92a_stage(analogue), v92a_rx_training(analogue), v92a_audio_acquired(frontend),
                    v92a_audio_clock_ppm(frontend), v92a_audio_eq_error(frontend),
                    (unsigned long long)v92a_audio_clipped(frontend));
        v92a4_t *p4 = v92a_phase4(analogue);
        if (p4) v92a4_set_data_source(p4, pair_payload_bit, &sink);
        if (p4 && v92a4_stage(p4) == V92A4_FAILED) {
            fprintf(stderr, "Phase 4 failed: %s\n", v92a4_failure(p4));
            break;
        }
        if (p4 && v92a4_stage(p4) == V92A4_DATA
            && v92a4_downstream_ready(p4) && sink.b1.locked
            && sink.payload_bytes >= 1024) break;
        if (v92a_stage(analogue) == V92A_FAILED) {
            fprintf(stderr, "analogue failure: %s\n", v92a_failure(analogue));
            break;
        }
    }
    assert(ja_seen);
    assert(v92a_stage(analogue) == V92A_PHASE4);
    assert(sink.cpt_count > 0);
    assert(!drop_cpd || dropped_cpd);
    assert(sink.e1u_count == 1);
    assert(v92a4_stage(v92a_phase4(analogue)) == V92A4_DATA);
    assert(v92a4_downstream_ready(v92a_phase4(analogue)));
    assert(sink.b1.locked);
    assert(sink.payload_bytes >= 1024 && sink.payload_errors == 0);
    assert(sink.b1.rejected_frames == 0);
    assert(v90_get_tx_phase(digital) == V90_TX_DATA);
    assert(v92a_cpt(analogue));
    if (audio) {
        assert(v92a_audio_clipped(frontend) == 0 && dac.clipped == 0);
        v92a_audio_free(frontend);
    } else v92a_free(analogue);
    v90_free(digital);
    printf("PASS: V.92 Phases 3–4 linear analogue / G.711 digital pair %s, %s DIL%s%s\n",
           alaw ? "PCMA" : "PCMU", dil ? "measured" : "zero",
           drop_cpd ? ", first CPd erased" : "", audio ? ", reconstructed audio" : "");
}

/* Independently scripted Su segments: sustained Su must not be called its
 * own inverse at the three-slot phase alias. Only the middle return permits
 * the +/- one-sample clock change implied by the 24.5T interval. */
static void test_su(bool alaw)
{
    const int p[6] = {1,0,1,-1,0,-1};
    for (int phase = 0; phase < 6; phase++) {
        for (int shift = -1; shift <= 1; shift++) {
            v92_su_t rx;
            v92_su_init(&rx, alaw);
            int t = 0, events[6] = {0};
            const int lengths[4] = {600, 24, 144, 24};
            for (int segment = 0; segment < 4; segment++) {
                for (int i = 0; i < lengths[segment]; i++, t++) {
                    int ph = (t + phase + (segment >= 2 ? shift + 6 : 0))%6;
                    int level = p[ph]*6000*(segment%2 ? -1 : 1);
                    v92_su_event_t e = v92_su_put(&rx, network_adc(alaw, level));
                    if (e) {
                        assert((int)e == segment + 1);
                        events[e]++;
                    }
                }
                assert(events[segment+1] == 1);
            }
            for (int i = 0; i < 4000; i++)
                assert(v92_su_put(&rx, network_adc(alaw, 0)) == V92_SU_NONE);
            v92_trn2u_tx_t tx;
            v92_trn2u_tx_init(&tx, 2, 6000, alaw);
            for (int i = 0; i < 2200; i++) {
                int16_t sample;
                assert(v92_trn1u_tx_linear(&tx, &sample, 1) == 1);
                v92_su_event_t e = v92_su_put(&rx, network_adc(alaw, sample));
                if (e) { assert(e == V92_SU_TRAINED); events[e]++; }
            }
            assert(events[V92_SU_TRAINED] == 1);
        }
    }
    printf("PASS: Su transitions, phase ambiguity and silence rejection %s\n", alaw ? "PCMA" : "PCMU");
}

static void test_filter_baseline(void)
{
    v90_state_t *digital = v90_init_data_pump(V90_LAW_ULAW);
    v90_enable_v92_mode(digital);
    assert(v90_set_v92_cpd_profile(digital, 1, 0, 0x8000));
    v92_cpd_frame_t cp, decoded;
    v92_cpd_diag_t diag;
    uint8_t packed[V92_CPD_MAX_BITS];
    int length;
    assert(v90_build_v92_cpd_frame(digital, &cp));
    /* Table 18 baseline: p1/z2, 192 total taps, 128 in one section. */
    cp.coeffs_present = true;
    cp.lp1 = 128;
    cp.lz2 = 64;
    cp.prefilter_ff[0] = 32767;
    assert(v92_cpd_encode(&cp, 12, packed, sizeof(packed), &length));
    assert(v92_cpd_decode(packed, length, &decoded, &diag));
    assert(decoded.lp1 == 128 && decoded.lz2 == 64);
    assert(v92_upstream_wave_profile_validate(&decoded));
    v92_upstream_wave_tx_t tx;
    v92_upstream_wave_rx_t rx;
    v92_upstream_wave_tx_init(&tx);
    v92_upstream_wave_rx_init(&rx);
    uint8_t bits[36], received[36];
    for (int frame = 0; frame < 24; frame++) {
        double wave[12];
        for (int i = 0; i < 36; i++) bits[i] = (frame*7 + i + i/3)%2;
        assert(v92_upstream_wave_encode_frame(&tx, &decoded, bits, 36, wave));
        assert(v92_upstream_wave_decode_frame(&rx, &decoded, wave, received, 36));
        assert(!memcmp(bits, received, 36));
    }
    v90_free(digital);
    puts("PASS: Table 18 baseline 192 total / 128 per-section filter capacity");
}

int main(int argc, char **argv)
{
    if (argc == 3 && !strcmp(argv[1], "--audio-case")) {
        test_phase3_pair(false, false, false, (unsigned)atoi(argv[2]));
        return 0;
    }
    test_audio();
    test_phase3_pair(false, false, false, false);
    test_phase3_pair(false, false, false, 48000);
    test_phase3_pair(true, false, false, false);
    test_phase3_pair(true, false, false, 48000);
    test_phase3_pair(false, true, false, false);
    test_phase3_pair(false, true, false, 48000);
    test_phase3_pair(true, true, false, false);
    test_phase3_pair(true, true, false, 48000);
    test_phase3_pair(false, false, true, false);
    test_phase3_pair(false, false, true, 48000);
    test_phase3_pair(true, false, true, false);
    test_phase3_pair(true, false, true, 48000);
    test_filter_baseline();
    test_su(false);
    test_su(true);
    test_linear();
    test_trn1u();
    for (int law = 0; law < 2; law++) {
        test_md(law, 1);
        test_md(law, 40);
    }
    for (int law = 0; law < 2; law++)
        for (int capable = 0; capable < 2; capable++)
            for (int short_request = 0; short_request < 2; short_request++)
                test_phase2(law, capable, short_request, false);
    return 0;
}
