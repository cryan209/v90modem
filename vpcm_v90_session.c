#include "vpcm_v90_session.h"
#include "v90.h"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

enum { VPCM_V90_NOMINAL_10S_FRAMES = 13328 };
enum { VPCM_V90_PHASE2_CHUNK_SAMPLES = 160 };
enum { VPCM_V90_PHASE2_MAX_CHUNKS = 2500 };
enum { VPCM_V90_PHASE2_V8_HANDOFF_CHUNKS = 4 };
enum { VPCM_V90_PHASE3_NATIVE_CHUNK_SAMPLES = 96 };
enum { VPCM_V90_PHASE3_NATIVE_MAX_SAMPLES = 200000 };
enum { VPCM_V90_PHASE3_V34_MAX_CHUNKS = 2500 };
enum { VPCM_V90_PHASE4_V34_MAX_CHUNKS = 4000 };
/* Data mode chunk: 96 samples = natural V.34 TX chunk size (= 16 CP frames of 6 codewords each). */
enum { VPCM_V90_DATA_CHUNK_CODEWORDS = 96 };

enum vpcm_v90_v34_rx_stages_e {
    VPCM_V90_V34_RX_STAGE_PHASE3_TRAINING = 11,
    VPCM_V90_V34_RX_STAGE_PHASE4_S = 13,
    VPCM_V90_V34_RX_STAGE_PHASE4_TRN = 15,
    VPCM_V90_V34_RX_STAGE_PHASE4_MP = 16,
    VPCM_V90_V34_RX_STAGE_DATA = 17
};

enum vpcm_v90_v34_tx_stages_e {
    VPCM_V90_V34_TX_STAGE_INITIAL_PREAMBLE = 1,
    VPCM_V90_V34_TX_STAGE_INFO0,
    VPCM_V90_V34_TX_STAGE_INITIAL_A,
    VPCM_V90_V34_TX_STAGE_FIRST_A,
    VPCM_V90_V34_TX_STAGE_FIRST_NOT_A,
    VPCM_V90_V34_TX_STAGE_FIRST_NOT_A_REVERSAL_SEEN,
    VPCM_V90_V34_TX_STAGE_SECOND_A,
    VPCM_V90_V34_TX_STAGE_L1,
    VPCM_V90_V34_TX_STAGE_L2,
    VPCM_V90_V34_TX_STAGE_POST_L2_A,
    VPCM_V90_V34_TX_STAGE_POST_L2_NOT_A,
    VPCM_V90_V34_TX_STAGE_A_SILENCE,
    VPCM_V90_V34_TX_STAGE_PRE_INFO1_A,
    VPCM_V90_V34_TX_STAGE_V90_WAIT_TONE_A,
    VPCM_V90_V34_TX_STAGE_V90_WAIT_INFO1A,
    VPCM_V90_V34_TX_STAGE_V90_WAIT_RX_L2,
    VPCM_V90_V34_TX_STAGE_V90_WAIT_TONE_A_REV,
    VPCM_V90_V34_TX_STAGE_V90_B_REV_DELAY,
    VPCM_V90_V34_TX_STAGE_V90_B_REV_10MS,
    VPCM_V90_V34_TX_STAGE_V90_PHASE2_B,
    VPCM_V90_V34_TX_STAGE_V90_PHASE2_B_INFO0_SEEN,
    VPCM_V90_V34_TX_STAGE_INFO1,
    VPCM_V90_V34_TX_STAGE_FIRST_B,
    VPCM_V90_V34_TX_STAGE_FIRST_B_INFO_SEEN,
    VPCM_V90_V34_TX_STAGE_FIRST_NOT_B_WAIT,
    VPCM_V90_V34_TX_STAGE_FIRST_NOT_B,
    VPCM_V90_V34_TX_STAGE_FIRST_B_SILENCE,
    VPCM_V90_V34_TX_STAGE_FIRST_B_POST_REVERSAL_SILENCE,
    VPCM_V90_V34_TX_STAGE_SECOND_B,
    VPCM_V90_V34_TX_STAGE_SECOND_B_WAIT,
    VPCM_V90_V34_TX_STAGE_SECOND_NOT_B,
    VPCM_V90_V34_TX_STAGE_INFO0_RETRY,
    VPCM_V90_V34_TX_STAGE_FIRST_S,
    VPCM_V90_V34_TX_STAGE_FIRST_NOT_S,
    VPCM_V90_V34_TX_STAGE_MD,
    VPCM_V90_V34_TX_STAGE_SECOND_S,
    VPCM_V90_V34_TX_STAGE_SECOND_NOT_S,
    VPCM_V90_V34_TX_STAGE_TRN,
    VPCM_V90_V34_TX_STAGE_J,
    VPCM_V90_V34_TX_STAGE_J_DASHED,
    VPCM_V90_V34_TX_STAGE_PHASE4_WAIT,
    VPCM_V90_V34_TX_STAGE_PHASE4_S,
    VPCM_V90_V34_TX_STAGE_PHASE4_NOT_S,
    VPCM_V90_V34_TX_STAGE_PHASE4_TRN,
    VPCM_V90_V34_TX_STAGE_MP,
    VPCM_V90_V34_TX_STAGE_HDX_INITIAL_A,
    VPCM_V90_V34_TX_STAGE_HDX_FIRST_A,
    VPCM_V90_V34_TX_STAGE_HDX_FIRST_NOT_A,
    VPCM_V90_V34_TX_STAGE_HDX_FIRST_A_SILENCE,
    VPCM_V90_V34_TX_STAGE_HDX_SECOND_A,
    VPCM_V90_V34_TX_STAGE_HDX_SECOND_A_WAIT,
    VPCM_V90_V34_TX_STAGE_HDX_FIRST_B,
    VPCM_V90_V34_TX_STAGE_HDX_FIRST_B_INFO_SEEN,
    VPCM_V90_V34_TX_STAGE_HDX_FIRST_NOT_B_WAIT,
    VPCM_V90_V34_TX_STAGE_HDX_FIRST_NOT_B,
    VPCM_V90_V34_TX_STAGE_HDX_POST_L2_B,
    VPCM_V90_V34_TX_STAGE_HDX_POST_L2_SILENCE,
    VPCM_V90_V34_TX_STAGE_HDX_SH,
    VPCM_V90_V34_TX_STAGE_HDX_FIRST_ALT,
    VPCM_V90_V34_TX_STAGE_HDX_PPH,
    VPCM_V90_V34_TX_STAGE_HDX_SECOND_ALT,
    VPCM_V90_V34_TX_STAGE_HDX_MPH,
    VPCM_V90_V34_TX_STAGE_HDX_E
};

enum vpcm_v90_v34_events_e {
    VPCM_V90_V34_EVENT_NONE = 0,
    VPCM_V90_V34_EVENT_INFO0_OK = 5,
    VPCM_V90_V34_EVENT_INFO1_OK = 7,
    VPCM_V90_V34_EVENT_S = 12,
    VPCM_V90_V34_EVENT_J = 13,
    VPCM_V90_V34_EVENT_J_DASHED = 14,
    VPCM_V90_V34_EVENT_TRAINING_FAILED = 16
};

/* Bit feeder: drives V.34 caller TX with a real upstream data buffer. */
typedef struct {
    const uint8_t *data;
    int len_bytes;
    int bit_pos;
} vpcm_v90_bit_feeder_t;

static int vpcm_v90_bit_feeder_get_bit(void *user_data)
{
    vpcm_v90_bit_feeder_t *f = (vpcm_v90_bit_feeder_t *) user_data;
    int byte_idx;
    int bit_idx;

    if (!f || !f->data || f->bit_pos >= f->len_bytes * 8)
        return 1; /* stuffing */
    byte_idx = f->bit_pos / 8;
    bit_idx  = f->bit_pos & 7;
    f->bit_pos++;
    return (f->data[byte_idx] >> bit_idx) & 1;
}

static void vpcm_v90_bit_feeder_init(vpcm_v90_bit_feeder_t *f,
                                     const uint8_t *data, int len_bytes)
{
    if (!f)
        return;
    f->data      = data;
    f->len_bytes = len_bytes;
    f->bit_pos   = 0;
}

static int vpcm_v90_dummy_get_bit(void *user_data)
{
    (void) user_data;
    return 1;
}

static void vpcm_v90_dummy_put_bit(void *user_data, int bit)
{
    (void) user_data;
    (void) bit;
}

static bool vpcm_v90_record_simplex(const vpcm_v90_startup_contract_io_t *io,
                                    v91_law_t law,
                                    bool digital_to_analogue,
                                    const uint8_t *tx_codewords,
                                    int codeword_len)
{
    if (!io || !io->record_simplex_g711)
        return true;
    return io->record_simplex_g711(io->user_data,
                                   law,
                                   digital_to_analogue,
                                   tx_codewords,
                                   codeword_len);
}

static bool vpcm_v90_record_duplex(const vpcm_v90_startup_contract_io_t *io,
                                   const uint8_t *digital_tx_codewords,
                                   const uint8_t *analogue_tx_codewords,
                                   int codeword_len)
{
    if (!io || !io->record_duplex_g711)
        return true;
    return io->record_duplex_g711(io->user_data,
                                  digital_tx_codewords,
                                  analogue_tx_codewords,
                                  codeword_len);
}

static void vpcm_v90_fill_pattern(uint8_t *buf, int len, uint32_t seed)
{
    uint32_t x;
    int i;

    if (!buf || len <= 0)
        return;
    x = seed ? seed : 0x12345678U;
    for (i = 0; i < len; i++) {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        buf[i] = (uint8_t) (x & 0xFFU);
    }
}

static bool vpcm_v90_expect_equal(const char *label,
                                  const uint8_t *expected,
                                  const uint8_t *actual,
                                  int len)
{
    int i;

    for (i = 0; i < len; i++) {
        if (expected[i] != actual[i]) {
            fprintf(stderr,
                    "%s mismatch at byte %d: expected 0x%02X got 0x%02X\n",
                    label, i, expected[i], actual[i]);
            return false;
        }
    }
    return true;
}

static int vpcm_v90_frames_from_seconds(int seconds)
{
    long long frames;

    if (seconds <= 0)
        return 0;
    frames = ((long long) seconds * 8000LL) / (long long) VPCM_CP_FRAME_INTERVALS;
    if (frames < 1)
        frames = 1;
    if (frames > 2000000000LL)
        frames = 2000000000LL;
    return (int) frames;
}

static int vpcm_v90_align_frames_for_duplex_bytes(int frames,
                                                  int downstream_bits_per_frame,
                                                  int upstream_bits_per_frame)
{
    int aligned;

    if (frames < 1)
        return 1;
    aligned = frames;
    while (((aligned * downstream_bits_per_frame) % 8) != 0
           || ((aligned * upstream_bits_per_frame) % 8) != 0) {
        aligned++;
    }
    return aligned;
}

static void vpcm_v90_copy_sample(uint8_t *dst, int dst_len, const uint8_t *src, int src_len)
{
    if (!dst || dst_len <= 0)
        return;
    memset(dst, 0, (size_t) dst_len);
    if (!src || src_len <= 0)
        return;
    if (src_len > dst_len)
        src_len = dst_len;
    memcpy(dst, src, (size_t) src_len);
}

static void vpcm_v90_seed_placeholder_info_state(v91_state_t *local,
                                                 const v91_info_frame_t *local_info,
                                                 const v91_info_frame_t *remote_info)
{
    if (!local || !local_info || !remote_info)
        return;
    local->last_tx_info = *local_info;
    local->last_tx_info_valid = true;
    local->last_rx_info = *remote_info;
    local->last_rx_info_valid = true;
}

static v90_law_t vpcm_v90_data_law(v91_law_t law)
{
    return (law == V91_LAW_ALAW) ? V90_LAW_ALAW : V90_LAW_ULAW;
}

static bool vpcm_v90_map_received_info0a(v90_info0a_t *dst, const v34_v90_info0a_t *src)
{
    if (!dst || !src)
        return false;

    memset(dst, 0, sizeof(*dst));
    dst->support_2743 = src->support_2743;
    dst->support_2800 = src->support_2800;
    dst->support_3429 = src->support_3429;
    dst->support_3000_low = src->support_3000_low;
    dst->support_3000_high = src->support_3000_high;
    dst->support_3200_low = src->support_3200_low;
    dst->support_3200_high = src->support_3200_high;
    dst->rate_3429_allowed = src->rate_3429_allowed;
    dst->support_power_reduction = src->support_power_reduction;
    dst->max_baud_rate_difference = src->max_baud_rate_difference;
    dst->from_cme_modem = src->from_cme_modem;
    dst->support_1664_point_constellation = src->support_1664_point_constellation;
    dst->tx_clock_source = src->tx_clock_source;
    dst->acknowledge_info0d = src->acknowledge_info0d;
    return v90_info0a_validate(dst);
}

static bool vpcm_v90_map_received_info1a(v90_info1a_t *dst, const v34_v90_info1a_t *src)
{
    if (!dst || !src)
        return false;

    memset(dst, 0, sizeof(*dst));
    dst->md = (uint8_t) src->md;
    dst->u_info = (uint8_t) src->u_info;
    dst->upstream_symbol_rate_code = (uint8_t) src->upstream_symbol_rate_code;
    dst->downstream_rate_code = (uint8_t) src->downstream_rate_code;
    dst->freq_offset = (int16_t) src->freq_offset;
    return v90_info1a_validate(dst);
}

static void vpcm_v90_init_default_like_dil(v90_dil_desc_t *desc)
{
    static const uint8_t clean_training_offsets[8] = {2, 4, 6, 8, 10, 12, 14, 15};
    static const uint16_t clean_sp_bits = 0x0A6DU;
    static const uint16_t clean_tp_bits = 0x0DB7U;
    int i;

    if (!desc)
        return;

    memset(desc, 0, sizeof(*desc));
    desc->n = 125;
    desc->lsp = 12;
    desc->ltp = 12;
    for (i = 0; i < 12; i++) {
        desc->sp[i] = (uint8_t) ((clean_sp_bits >> i) & 1U);
        desc->tp[i] = (uint8_t) ((clean_tp_bits >> i) & 1U);
    }
    for (i = 0; i < 8; i++) {
        desc->h[i] = 1;
        desc->ref[i] = (uint8_t) ((i << 4) | 1);
    }
    for (i = 0; i < desc->n; i++) {
        int uchord = i % 8;
        int variant = (i / 8) % 8;

        desc->train_u[i] = (uint8_t) ((uchord << 4) | clean_training_offsets[variant]);
    }
}

static void vpcm_v90_init_test_ja_profile(v90_dil_desc_t *desc, bool echo_limited)
{
    static const uint8_t echo_training_offsets[4] = {11, 13, 14, 15};
    static const uint8_t echo_ref_ucodes[2] = {33, 49};
    static const uint8_t echo_sp_bits[6] = {1, 0, 1, 1, 0, 0};
    static const uint8_t echo_tp_bits[6] = {1, 0, 1, 0, 1, 0};
    int i;

    if (!desc)
        return;

    vpcm_v90_init_default_like_dil(desc);
    if (!echo_limited)
        return;

    desc->n = 96;
    desc->lsp = 6;
    desc->ltp = 6;
    memcpy(desc->sp, echo_sp_bits, sizeof(echo_sp_bits));
    memcpy(desc->tp, echo_tp_bits, sizeof(echo_tp_bits));
    for (i = 0; i < 8; i++) {
        desc->h[i] = 1;
        desc->ref[i] = 0;
    }
    desc->ref[2] = echo_ref_ucodes[0];
    desc->ref[3] = echo_ref_ucodes[1];
    for (i = 0; i < desc->n; i++) {
        int uchord = 2 + ((i / 12) & 1);
        int variant = (i / 6) & 3;

        desc->train_u[i] = (uint8_t) ((uchord << 4) | echo_training_offsets[variant]);
    }
}

static void vpcm_v90_copy_dil_to_v91_compat(v91_dil_desc_t *dst, const v90_dil_desc_t *src)
{
    if (!dst || !src)
        return;

    memset(dst, 0, sizeof(*dst));
    dst->n = src->n;
    dst->lsp = src->lsp;
    dst->ltp = src->ltp;
    memcpy(dst->sp, src->sp, sizeof(dst->sp));
    memcpy(dst->tp, src->tp, sizeof(dst->tp));
    memcpy(dst->h, src->h, sizeof(dst->h));
    memcpy(dst->ref, src->ref, sizeof(dst->ref));
    memcpy(dst->train_u, src->train_u, sizeof(dst->train_u));
}

static void vpcm_v90_map_dil_analysis_to_v91_compat(v91_dil_analysis_t *dst,
                                                     const v90_dil_analysis_t *src)
{
    if (!dst || !src)
        return;

    memset(dst, 0, sizeof(*dst));
    dst->n = src->n;
    dst->lsp = src->lsp;
    dst->ltp = src->ltp;
    dst->unique_train_u = src->unique_train_u;
    dst->repeated_uchords = src->used_uchords;
    dst->non_default_refs = src->non_default_refs;
    dst->non_default_h = src->non_default_h;
    dst->impairment_score = src->impairment_score;
    dst->default_like = src->looks_default_125x12;
    dst->robbed_bit_limited = src->robbed_bit_limited;
    dst->echo_limited = src->echo_limited;
    dst->recommended_downstream_drn = src->recommended_downstream_drn;
    dst->recommended_upstream_drn = src->recommended_upstream_drn;
}

static bool vpcm_v90_dil_is_default_like_for_v91_compat(const v90_dil_analysis_t *analysis)
{
    return analysis && analysis->looks_default_125x12;
}

static bool vpcm_v90_phase2_info_is_default_like(const v90_info0a_t *info0a,
                                                 const v90_info1a_t *info1a)
{
    v90_info0a_t default_info0a;
    v90_info1a_t default_info1a;

    if (!info0a || !info1a)
        return false;

    v90_info0a_init(&default_info0a);
    v90_info1a_init(&default_info1a);
    return memcmp(info0a, &default_info0a, sizeof(default_info0a)) == 0
        && info1a->md == default_info1a.md
        && info1a->upstream_symbol_rate_code == default_info1a.upstream_symbol_rate_code
        && info1a->downstream_rate_code == default_info1a.downstream_rate_code
        && info1a->freq_offset == default_info1a.freq_offset;
}

static void vpcm_v90_init_placeholder_info_frame(v91_info_frame_t *info,
                                                 v91_law_t law)
{
    if (!info)
        return;

    memset(info, 0, sizeof(*info));
    info->tx_uses_alaw = (law == V91_LAW_ALAW);
    info->power_measured_after_digital_impairments = true;
}

static void vpcm_v90_prepare_placeholder_info_frames(const vpcm_v90_startup_contract_params_t *params,
                                                     const v90_dil_analysis_t *digital_dil_analysis,
                                                     const vpcm_v90_startup_contract_report_t *report,
                                                     v91_info_frame_t *digital_info,
                                                     v91_info_frame_t *analogue_info)
{
    bool analogue_default_like;

    if (!params || !digital_info || !analogue_info)
        return;

    vpcm_v90_init_placeholder_info_frame(digital_info, params->law);
    vpcm_v90_init_placeholder_info_frame(analogue_info, params->law);

    digital_info->request_default_dil = vpcm_v90_dil_is_default_like_for_v91_compat(digital_dil_analysis);

    analogue_default_like = report
        && report->phase2_received_info0a_valid
        && report->phase2_received_info1a_valid
        && vpcm_v90_phase2_info_is_default_like(&report->phase2_received_info0a,
                                                &report->phase2_received_info1a);
    analogue_info->request_default_dil = analogue_default_like;
    analogue_info->acknowledge_info_frame =
        report
        && report->phase2_received_info0a_valid
        && report->phase2_received_info0a.acknowledge_info0d;
}

static void vpcm_v90_encode_linear_chunk_to_g711(v91_law_t law,
                                                 const int16_t *src,
                                                 uint8_t *dst,
                                                 int samples)
{
    int i;

    if (!src || !dst || samples <= 0)
        return;
    for (i = 0; i < samples; i++)
        dst[i] = v91_linear_to_codeword(law, src[i]);
}

static void vpcm_v90_transport_linear(v91_law_t law,
                                      int16_t *dst,
                                      const int16_t *src,
                                      int len)
{
    int i;

    if (!dst || !src || len <= 0)
        return;
    for (i = 0; i < len; i++) {
        uint8_t codeword;

        codeword = v91_linear_to_codeword(law, src[i]);
        dst[i] = v91_codeword_to_linear(law, codeword);
    }
}

/*
 * vpcm_v90_run_coupled_training — V.90 Phase 3+4 synchronized loop.
 *
 * Runs three things in lockstep:
 *   A. The V.34 answerer↔caller pair — provides proper V.34 training signals
 *      so the caller can complete Phase 3/4 (SpanDSP V.34 cannot decode V.90
 *      PCM training waveforms, so the answerer acts as a V.34 training proxy).
 *   B. The V.90 PCM state machine — generates the correct downstream G.711
 *      codewords (Sd/TRN1d/Jd/DIL/Ri/TRN2d/CP/B1d) for recording.
 *   C. Event bridging — real V.34 milestones from the caller's TX drive the
 *      V.90 PCM state machine, replacing the old fixed-duration timers:
 *        caller TX >= FIRST_S  + digital in Jd  → v90_notify_s_detected (Jd)
 *        caller TX >= PHASE4_WAIT + digital in DIL → v90_notify_s_detected (DIL)
 *        caller TX >= MP        + digital in TRN2d → v90_notify_cp_ready
 *
 * Downstream recording comes from the V.90 PCM machine (correct waveform).
 * Upstream recording comes from the caller's V.34 TX (correct waveform).
 * The answerer's V.34 TX is used only for training and is not recorded.
 */
static bool vpcm_v90_run_coupled_training(v91_law_t law,
                                          v34_state_t *caller,
                                          v34_state_t *answerer,
                                          int u_info,
                                          const v90_dil_desc_t *digital_dil,
                                          const v90_dil_analysis_t *dil_analysis,
                                          bool v92_mode,
                                          const vpcm_v90_startup_contract_io_t *io,
                                          vpcm_v90_startup_contract_report_t *report)
{
    v90_state_t *digital;
    vpcm_cp_frame_t cp_frame;
    uint8_t downstream_drn;
    uint8_t upstream_drn;
    int16_t answerer_tx[VPCM_V90_PHASE3_NATIVE_CHUNK_SAMPLES];
    int16_t caller_tx[VPCM_V90_PHASE3_NATIVE_CHUNK_SAMPLES];
    int16_t answerer_rx[VPCM_V90_PHASE3_NATIVE_CHUNK_SAMPLES];
    int16_t caller_rx[VPCM_V90_PHASE3_NATIVE_CHUNK_SAMPLES];
    int16_t downstream_linear[VPCM_V90_PHASE3_NATIVE_CHUNK_SAMPLES];
    uint8_t downstream_g711[VPCM_V90_PHASE3_NATIVE_CHUNK_SAMPLES];
    uint8_t upstream_g711[VPCM_V90_PHASE3_NATIVE_CHUNK_SAMPLES];
    int total_samples;
    int caller_tx_stage;
    int caller_event;
    int answerer_event;
    v90_tx_phase_t tx_phase;
    bool jd_notified;
    bool dil_notified;
    bool cp_notified;
    bool ok;

    if (!caller || !answerer || !digital_dil || !dil_analysis)
        return false;

    digital = v90_init_data_pump(vpcm_v90_data_law(law));
    if (!digital) {
        fprintf(stderr, "V.90 coupled training: failed to initialize digital state\n");
        return false;
    }

    /* Pre-build CP frame from DIL analysis. */
    vpcm_v92_select_profile_from_dil(dil_analysis, &downstream_drn, &upstream_drn);
    vpcm_cp_init(&cp_frame);
    cp_frame.transparent_mode_granted = false;
    cp_frame.v90_compatibility = true;
    cp_frame.drn = downstream_drn;
    cp_frame.constellation_count = 1;
    memset(cp_frame.dfi, 0, sizeof(cp_frame.dfi));
    vpcm_cp_enable_all_ucodes(cp_frame.masks[0]);
    if (!v90_set_phase4_cp(digital, &cp_frame)) {
        fprintf(stderr, "V.90 coupled training: failed to encode CP frame (drn=%u)\n",
                (unsigned) downstream_drn);
        v90_free(digital);
        return false;
    }

    v90_set_dil_descriptor(digital, digital_dil);
    v90_start_phase3(digital, u_info);

    total_samples = 0;
    jd_notified = false;
    dil_notified = false;
    cp_notified = false;
    ok = false;
    while (total_samples < VPCM_V90_PHASE3_NATIVE_MAX_SAMPLES) {

        /* A. V.34 training pair: answerer provides downstream V.34 signals
         *    to the caller so Phase 3/4 training completes normally. */
        if (v34_tx(answerer, answerer_tx, VPCM_V90_PHASE3_NATIVE_CHUNK_SAMPLES)
                != VPCM_V90_PHASE3_NATIVE_CHUNK_SAMPLES
            || v34_tx(caller, caller_tx, VPCM_V90_PHASE3_NATIVE_CHUNK_SAMPLES)
                != VPCM_V90_PHASE3_NATIVE_CHUNK_SAMPLES)
            break;

        vpcm_v90_transport_linear(law, caller_rx, answerer_tx,
                                  VPCM_V90_PHASE3_NATIVE_CHUNK_SAMPLES);
        vpcm_v90_transport_linear(law, answerer_rx, caller_tx,
                                  VPCM_V90_PHASE3_NATIVE_CHUNK_SAMPLES);
        if (v34_rx(caller, caller_rx, VPCM_V90_PHASE3_NATIVE_CHUNK_SAMPLES) != 0
            || v34_rx(answerer, answerer_rx, VPCM_V90_PHASE3_NATIVE_CHUNK_SAMPLES) != 0)
            break;

        /* B. V.90 PCM machine: generate downstream recording (correct waveform). */
        v90_phase3_tx(digital, downstream_linear, VPCM_V90_PHASE3_NATIVE_CHUNK_SAMPLES);
        vpcm_v90_encode_linear_chunk_to_g711(law, downstream_linear, downstream_g711,
                                             VPCM_V90_PHASE3_NATIVE_CHUNK_SAMPLES);
        if (!vpcm_v90_record_simplex(io, law, true, downstream_g711,
                                     VPCM_V90_PHASE3_NATIVE_CHUNK_SAMPLES)) {
            fprintf(stderr, "V.90 coupled training: downstream record failed\n");
            break;
        }

        /* C. Record caller's V.34 TX as the upstream waveform. */
        vpcm_v90_encode_linear_chunk_to_g711(law, caller_tx, upstream_g711,
                                             VPCM_V90_PHASE3_NATIVE_CHUNK_SAMPLES);
        if (!vpcm_v90_record_simplex(io, law, false, upstream_g711,
                                     VPCM_V90_PHASE3_NATIVE_CHUNK_SAMPLES)) {
            fprintf(stderr, "V.90 coupled training: upstream record failed\n");
            break;
        }

        total_samples += VPCM_V90_PHASE3_NATIVE_CHUNK_SAMPLES;
        tx_phase = v90_get_tx_phase(digital);
        caller_tx_stage = v34_get_tx_stage(caller);
        caller_event = v34_get_rx_event(caller);
        answerer_event = v34_get_rx_event(answerer);

        /* D. Bridge V.34 caller milestones → V.90 PCM state machine. */
        if (!jd_notified && tx_phase == V90_TX_JD
                && caller_tx_stage >= VPCM_V90_V34_TX_STAGE_FIRST_S) {
            v90_notify_s_detected(digital);
            jd_notified = true;
        }
        if (!dil_notified && tx_phase == V90_TX_DIL
                && caller_tx_stage >= VPCM_V90_V34_TX_STAGE_PHASE4_WAIT) {
            v90_notify_s_detected(digital);
            dil_notified = true;
        }
        if (!cp_notified && tx_phase == V90_TX_TRN2D
                && caller_tx_stage >= VPCM_V90_V34_TX_STAGE_MP) {
            v90_notify_cp_ready(digital);
            cp_notified = true;
        }

        /* E. Exit conditions. */
        if (v90_using_internal_v34_tx(digital)
            || v90_training_complete(digital)) {
            ok = true;
            break;
        }
        if (caller_event == VPCM_V90_V34_EVENT_TRAINING_FAILED
            || answerer_event == VPCM_V90_V34_EVENT_TRAINING_FAILED)
            break;
    }

    if (!ok) {
        fprintf(stderr,
                "V.90 coupled training: did not complete "
                "(samples=%d jd=%d dil=%d cp=%d tx_phase=%d caller_tx=%d)\n",
                total_samples, jd_notified ? 1 : 0, dil_notified ? 1 : 0,
                cp_notified ? 1 : 0, (int) v90_get_tx_phase(digital),
                v34_get_tx_stage(caller));
    }

    if (report) {
        caller_tx_stage = v34_get_tx_stage(caller);
        report->phase3_native_analogue_started   = jd_notified || dil_notified;
        report->phase3_native_analogue_completed = ok;
        report->phase3_native_caller_tx_stage    = caller_tx_stage;
        report->phase3_native_caller_rx_stage    = v34_get_rx_stage(caller);
        report->phase3_native_caller_rx_event    = v34_get_rx_event(caller);
        report->phase3_native_caller_j_bits      = v34_get_phase3_j_bits(caller);
        report->phase3_native_caller_j_trn16     = v34_get_phase3_j_trn16(caller);
        report->phase3_native_caller_trn_lock_score = v34_get_phase3_trn_lock_score(caller);
        report->phase3_native_answerer_tx_stage  = v34_get_tx_stage(answerer);
        report->phase3_native_answerer_rx_stage  = v34_get_rx_stage(answerer);
        report->phase3_native_answerer_rx_event  = v34_get_rx_event(answerer);
        report->phase3_native_answerer_j_bits    = v34_get_phase3_j_bits(answerer);
        report->phase3_native_answerer_j_trn16   = v34_get_phase3_j_trn16(answerer);
        report->phase3_native_answerer_trn_lock_score = v34_get_phase3_trn_lock_score(answerer);
        report->phase4_native_analogue_started   = (caller_tx_stage >= VPCM_V90_V34_TX_STAGE_PHASE4_WAIT);
        report->phase4_native_analogue_completed = ok && cp_notified;
        report->phase4_native_caller_tx_data_mode = (v34_get_tx_data_mode(caller) != 0);
        report->phase4_native_caller_tx_stage    = caller_tx_stage;
        report->phase4_native_caller_rx_stage    = v34_get_rx_stage(caller);
        report->phase4_native_caller_rx_event    = v34_get_rx_event(caller);
        report->phase4_native_answerer_tx_data_mode = (v34_get_tx_data_mode(answerer) != 0);
        report->phase4_native_answerer_tx_stage  = v34_get_tx_stage(answerer);
        report->phase4_native_answerer_rx_stage  = v34_get_rx_stage(answerer);
        report->phase4_native_answerer_rx_event  = v34_get_rx_event(answerer);
    }

    v90_free(digital);
    return ok;
}


static bool vpcm_v90_run_phase2_exchange(v91_law_t law,
                                         const vpcm_v90_startup_contract_io_t *io,
                                         const v90_dil_desc_t *digital_dil,
                                         const v90_dil_analysis_t *dil_analysis,
                                         vpcm_v90_startup_contract_report_t *report,
                                         vpcm_v90_bit_feeder_t *caller_feeder,
                                         v34_state_t **caller_out)
{
    v34_state_t *caller;
    v34_state_t *answerer;
    v34_v90_info0a_t raw_info0a;
    v34_v90_info1a_t raw_info1a;
    v90_info0a_t received_info0a;
    v90_info1a_t received_info1a;
    int chunk;
    int final_caller_tx_stage;
    int final_caller_rx_stage;
    int final_answerer_tx_stage;
    int final_answerer_rx_stage;
    bool caller_phase3_tx_ready;
    bool answerer_phase3_tx_ready;
    bool caller_saw_info1;
    bool answerer_saw_info0;
    bool answerer_saw_info1;
    bool answerer_saw_uinfo;
    bool phase3_seen;
    bool phase2_ok;
    bool received_info0a_valid;
    bool received_info1a_valid;
    bool ok;

    if (caller_out)
        *caller_out = NULL;

    /* Use the bit feeder for the caller so data-mode TX carries real payload.
     * The feeder data pointer is populated before vpcm_v90_run_data_mode is called;
     * during training v34_tx only invokes get_bit after data mode is entered,
     * at which point the feeder will have been filled with the upstream data. */
    caller = v34_init(NULL, 3200, 21600, true, true,
                      caller_feeder ? vpcm_v90_bit_feeder_get_bit : vpcm_v90_dummy_get_bit,
                      caller_feeder,
                      vpcm_v90_dummy_put_bit, NULL);
    answerer = v34_init(NULL, 3200, 21600, false, true,
                        vpcm_v90_dummy_get_bit, NULL,
                        vpcm_v90_dummy_put_bit, NULL);
    if (!caller || !answerer) {
        if (caller)
            v34_free(caller);
        if (answerer)
            v34_free(answerer);
        fprintf(stderr, "V.90 Phase 2 probe: failed to initialize V.34 states\n");
        return false;
    }

    for (chunk = 0; chunk < VPCM_V90_PHASE2_V8_HANDOFF_CHUNKS; chunk++) {
        int16_t caller_tx[VPCM_V90_PHASE2_CHUNK_SAMPLES];
        int16_t answer_tx[VPCM_V90_PHASE2_CHUNK_SAMPLES];
        int16_t caller_rx[VPCM_V90_PHASE2_CHUNK_SAMPLES];
        int16_t answer_rx[VPCM_V90_PHASE2_CHUNK_SAMPLES];
        uint8_t caller_tx_g711[VPCM_V90_PHASE2_CHUNK_SAMPLES];
        uint8_t answer_tx_g711[VPCM_V90_PHASE2_CHUNK_SAMPLES];

        if (v34_tx(caller, caller_tx, VPCM_V90_PHASE2_CHUNK_SAMPLES) != VPCM_V90_PHASE2_CHUNK_SAMPLES
            || v34_tx(answerer, answer_tx, VPCM_V90_PHASE2_CHUNK_SAMPLES) != VPCM_V90_PHASE2_CHUNK_SAMPLES) {
            fprintf(stderr, "V.90 Phase 2 probe: handoff TX failed\n");
            v34_free(caller);
            v34_free(answerer);
            return false;
        }

        vpcm_v90_encode_linear_chunk_to_g711(law, caller_tx, caller_tx_g711, VPCM_V90_PHASE2_CHUNK_SAMPLES);
        vpcm_v90_encode_linear_chunk_to_g711(law, answer_tx, answer_tx_g711, VPCM_V90_PHASE2_CHUNK_SAMPLES);
        if (!vpcm_v90_record_duplex(io, caller_tx_g711, answer_tx_g711, VPCM_V90_PHASE2_CHUNK_SAMPLES)) {
            fprintf(stderr, "V.90 Phase 2 probe: handoff recording failed\n");
            v34_free(caller);
            v34_free(answerer);
            return false;
        }

        vpcm_v90_transport_linear(law, answer_rx, caller_tx, VPCM_V90_PHASE2_CHUNK_SAMPLES);
        vpcm_v90_transport_linear(law, caller_rx, answer_tx, VPCM_V90_PHASE2_CHUNK_SAMPLES);
        if (v34_rx(caller, caller_rx, VPCM_V90_PHASE2_CHUNK_SAMPLES) != 0
            || v34_rx(answerer, answer_rx, VPCM_V90_PHASE2_CHUNK_SAMPLES) != 0) {
            fprintf(stderr, "V.90 Phase 2 probe: handoff RX failed\n");
            v34_free(caller);
            v34_free(answerer);
            return false;
        }
    }

    v34_set_v90_mode(caller, law == V91_LAW_ALAW ? 1 : 0);
    v34_set_v90_mode(answerer, law == V91_LAW_ALAW ? 1 : 0);

    answerer_saw_info0 = false;
    answerer_saw_info1 = false;
    answerer_saw_uinfo = false;
    phase3_seen = false;
    caller_phase3_tx_ready = false;
    answerer_phase3_tx_ready = false;
    caller_saw_info1 = false;
    final_caller_tx_stage = v34_get_tx_stage(caller);
    final_caller_rx_stage = v34_get_rx_stage(caller);
    final_answerer_tx_stage = v34_get_tx_stage(answerer);
    final_answerer_rx_stage = v34_get_rx_stage(answerer);
    received_info0a_valid = false;
    received_info1a_valid = false;
    memset(&received_info0a, 0, sizeof(received_info0a));
    memset(&received_info1a, 0, sizeof(received_info1a));
    ok = false;
    for (chunk = 0; chunk < VPCM_V90_PHASE2_MAX_CHUNKS; chunk++) {
        int16_t caller_tx[VPCM_V90_PHASE2_CHUNK_SAMPLES];
        int16_t answer_tx[VPCM_V90_PHASE2_CHUNK_SAMPLES];
        int16_t caller_rx[VPCM_V90_PHASE2_CHUNK_SAMPLES];
        int16_t answer_rx[VPCM_V90_PHASE2_CHUNK_SAMPLES];
        uint8_t caller_tx_g711[VPCM_V90_PHASE2_CHUNK_SAMPLES];
        uint8_t answer_tx_g711[VPCM_V90_PHASE2_CHUNK_SAMPLES];
        int caller_event;
        int answerer_event;
        int caller_rx_stage;
        int answerer_rx_stage;
        int caller_tx_stage;
        int answerer_tx_stage;
        bool caller_phase3_tx_now;
        bool answerer_phase3_tx_now;

        if (v34_tx(caller, caller_tx, VPCM_V90_PHASE2_CHUNK_SAMPLES) != VPCM_V90_PHASE2_CHUNK_SAMPLES
            || v34_tx(answerer, answer_tx, VPCM_V90_PHASE2_CHUNK_SAMPLES) != VPCM_V90_PHASE2_CHUNK_SAMPLES) {
            fprintf(stderr, "V.90 Phase 2 probe: TX failed at chunk %d\n", chunk);
            break;
        }

        vpcm_v90_encode_linear_chunk_to_g711(law, caller_tx, caller_tx_g711, VPCM_V90_PHASE2_CHUNK_SAMPLES);
        vpcm_v90_encode_linear_chunk_to_g711(law, answer_tx, answer_tx_g711, VPCM_V90_PHASE2_CHUNK_SAMPLES);
        if (!vpcm_v90_record_duplex(io, caller_tx_g711, answer_tx_g711, VPCM_V90_PHASE2_CHUNK_SAMPLES)) {
            fprintf(stderr, "V.90 Phase 2 probe: recording failed at chunk %d\n", chunk);
            break;
        }

        vpcm_v90_transport_linear(law, answer_rx, caller_tx, VPCM_V90_PHASE2_CHUNK_SAMPLES);
        vpcm_v90_transport_linear(law, caller_rx, answer_tx, VPCM_V90_PHASE2_CHUNK_SAMPLES);
        if (v34_rx(caller, caller_rx, VPCM_V90_PHASE2_CHUNK_SAMPLES) != 0
            || v34_rx(answerer, answer_rx, VPCM_V90_PHASE2_CHUNK_SAMPLES) != 0) {
            fprintf(stderr, "V.90 Phase 2 probe: RX failed at chunk %d\n", chunk);
            break;
        }

        caller_event = v34_get_rx_event(caller);
        answerer_event = v34_get_rx_event(answerer);
        caller_tx_stage = v34_get_tx_stage(caller);
        answerer_tx_stage = v34_get_tx_stage(answerer);
        caller_rx_stage = v34_get_rx_stage(caller);
        answerer_rx_stage = v34_get_rx_stage(answerer);
        final_caller_tx_stage = caller_tx_stage;
        final_caller_rx_stage = caller_rx_stage;
        final_answerer_tx_stage = answerer_tx_stage;
        final_answerer_rx_stage = answerer_rx_stage;

        caller_saw_info1 |= (caller_event == VPCM_V90_V34_EVENT_INFO1_OK);
        answerer_saw_info0 |= (answerer_event == VPCM_V90_V34_EVENT_INFO0_OK);
        answerer_saw_info1 |= (answerer_event == VPCM_V90_V34_EVENT_INFO1_OK);
        answerer_saw_uinfo |= (v34_get_v90_u_info(answerer) > 0);
        phase3_seen |= (caller_rx_stage >= VPCM_V90_V34_RX_STAGE_PHASE3_TRAINING)
                    || (answerer_rx_stage >= VPCM_V90_V34_RX_STAGE_PHASE3_TRAINING)
                    || v34_get_primary_channel_active(caller)
                    || v34_get_primary_channel_active(answerer);
        caller_phase3_tx_now = (caller_tx_stage >= VPCM_V90_V34_TX_STAGE_FIRST_S)
                            && (caller_tx_stage < VPCM_V90_V34_TX_STAGE_PHASE4_WAIT);
        answerer_phase3_tx_now = (answerer_tx_stage >= VPCM_V90_V34_TX_STAGE_FIRST_S)
                              && (answerer_tx_stage < VPCM_V90_V34_TX_STAGE_PHASE4_WAIT);
        caller_phase3_tx_ready |= caller_phase3_tx_now;
        answerer_phase3_tx_ready |= answerer_phase3_tx_now;

        if (answerer_saw_info0
            && answerer_saw_info1
            && answerer_saw_uinfo
            && phase3_seen
            && caller_phase3_tx_now
            && answerer_phase3_tx_now) {
            ok = true;
            break;
        }

        if (caller_event == VPCM_V90_V34_EVENT_TRAINING_FAILED
            || answerer_event == VPCM_V90_V34_EVENT_TRAINING_FAILED) {
            break;
        }
    }

    if (answerer_saw_info0
        && v34_get_v90_received_info0a(answerer, &raw_info0a) > 0
        && vpcm_v90_map_received_info0a(&received_info0a, &raw_info0a)) {
        received_info0a_valid = true;
    }
    if (answerer_saw_info1
        && v34_get_v90_received_info1a(answerer, &raw_info1a) > 0
        && vpcm_v90_map_received_info1a(&received_info1a, &raw_info1a)) {
        received_info1a_valid = true;
    }

    if (!ok
        && answerer_saw_info0
        && answerer_saw_info1
        && answerer_saw_uinfo
        && phase3_seen
        && (final_caller_tx_stage >= VPCM_V90_V34_TX_STAGE_FIRST_S)
        && (final_caller_tx_stage < VPCM_V90_V34_TX_STAGE_PHASE4_WAIT)
        && (final_answerer_tx_stage >= VPCM_V90_V34_TX_STAGE_FIRST_S)
        && (final_answerer_tx_stage < VPCM_V90_V34_TX_STAGE_PHASE4_WAIT)) {
        ok = true;
    }

    if (ok && (!received_info0a_valid || !received_info1a_valid)) {
        fprintf(stderr,
                "V.90 Phase 2 probe could not consume received INFO frames: info0a=%d info1a=%d\n",
                received_info0a_valid ? 1 : 0,
                received_info1a_valid ? 1 : 0);
        ok = false;
    }

    phase2_ok = ok;
    if (report) {
        report->phase2_completed = phase2_ok;
        report->phase2_phase3_seen = phase3_seen;
        report->phase2_caller_saw_info1 = caller_saw_info1;
        report->phase2_caller_phase3_tx_ready = caller_phase3_tx_ready;
        report->phase2_u_info = received_info1a_valid ? received_info1a.u_info : v34_get_v90_u_info(answerer);
        report->phase2_caller_tx_stage = final_caller_tx_stage;
        report->phase2_caller_rx_stage = final_caller_rx_stage;
        report->phase2_answerer_tx_stage = final_answerer_tx_stage;
        report->phase2_answerer_rx_stage = final_answerer_rx_stage;
        report->phase2_received_info0a_valid = received_info0a_valid;
        report->phase2_received_info1a_valid = received_info1a_valid;
        report->phase2_received_info0a = received_info0a;
        report->phase2_received_info1a = received_info1a;
        report->phase3_native_analogue_started = false;
        report->phase3_native_analogue_completed = false;
        report->phase3_native_caller_tx_stage = final_caller_tx_stage;
        report->phase3_native_caller_rx_stage = final_caller_rx_stage;
        report->phase3_native_caller_rx_event = VPCM_V90_V34_EVENT_NONE;
        report->phase3_native_caller_j_bits = 0;
        report->phase3_native_caller_j_trn16 = -1;
        report->phase3_native_caller_trn_lock_score = -1;
        report->phase3_native_answerer_tx_stage = final_answerer_tx_stage;
        report->phase3_native_answerer_rx_stage = final_answerer_rx_stage;
        report->phase3_native_answerer_rx_event = VPCM_V90_V34_EVENT_NONE;
        report->phase3_native_answerer_j_bits = 0;
        report->phase3_native_answerer_j_trn16 = -1;
        report->phase3_native_answerer_trn_lock_score = -1;
        report->phase4_native_analogue_started = false;
        report->phase4_native_analogue_completed = false;
        report->phase4_native_caller_tx_data_mode = false;
        report->phase4_native_answerer_tx_data_mode = false;
        report->phase4_native_caller_tx_stage = final_caller_tx_stage;
        report->phase4_native_caller_rx_stage = final_caller_rx_stage;
        report->phase4_native_caller_rx_event = VPCM_V90_V34_EVENT_NONE;
        report->phase4_native_answerer_tx_stage = final_answerer_tx_stage;
        report->phase4_native_answerer_rx_stage = final_answerer_rx_stage;
        report->phase4_native_answerer_rx_event = VPCM_V90_V34_EVENT_NONE;
    }

    if (!phase2_ok) {
        fprintf(stderr,
                "V.90 Phase 2 probe incomplete: caller_info1=%d caller_phase3_tx=%d answerer_phase3_tx=%d info0=%d info1=%d uinfo=%d phase3=%d consumed_info0=%d consumed_info1=%d final_stages caller=%d/%d answerer=%d/%d final_uinfo=%d\n",
                caller_saw_info1 ? 1 : 0,
                caller_phase3_tx_ready ? 1 : 0,
                answerer_phase3_tx_ready ? 1 : 0,
                answerer_saw_info0 ? 1 : 0,
                answerer_saw_info1 ? 1 : 0,
                answerer_saw_uinfo ? 1 : 0,
                phase3_seen ? 1 : 0,
                received_info0a_valid ? 1 : 0,
                received_info1a_valid ? 1 : 0,
                final_caller_tx_stage,
                final_caller_rx_stage,
                final_answerer_tx_stage,
                final_answerer_rx_stage,
                v34_get_v90_u_info(answerer));
    } else {
        if (caller_phase3_tx_ready)
            vpcm_v90_run_coupled_training(law, caller, answerer,
                                          report ? report->phase2_u_info : 0,
                                          digital_dil, dil_analysis, io, report);
    }

    /* answerer is always an internal training proxy; free it now. */
    v34_free(answerer);

    /* Transfer caller ownership to the data-mode loop when native training
     * completed — the caller V.34 state is in data mode and its v34_tx()
     * output is the correct V.92 upstream waveform.  In all other cases
     * (phase 2 failed, training did not complete, or no output pointer
     * provided) the caller is freed here. */
    if (phase2_ok
        && report && report->phase3_native_analogue_completed
        && caller_out) {
        *caller_out = caller;
    } else {
        v34_free(caller);
    }
    return ok;
}

void vpcm_v90_session_init(vpcm_v90_session_t *session, v91_law_t law)
{
    if (!session)
        return;
    memset(session, 0, sizeof(*session));
    session->law = law;
    session->state = VPCM_V90_MODEM_IDLE;
}

void vpcm_v90_session_reset(vpcm_v90_session_t *session)
{
    if (!session)
        return;
    session->state = VPCM_V90_MODEM_IDLE;
    session->data_mode_active = false;
}

void vpcm_v90_session_set_state(vpcm_v90_session_t *session, vpcm_v90_modem_state_t state)
{
    if (!session)
        return;
    session->state = state;
    session->data_mode_active = (state == VPCM_V90_MODEM_DATA);
}

bool vpcm_v92_init_digital_dil_from_ja(v90_dil_desc_t *desc, bool echo_limited)
{
    v90_dil_desc_t profile;
    v90_dil_desc_t parsed;
    uint8_t bits[512];
    int bit_len;

    if (!desc)
        return false;

    vpcm_v90_init_test_ja_profile(&profile, echo_limited);
    if (!v90_build_dil_descriptor_bits(bits, (int) sizeof(bits), &bit_len, &profile))
        return false;
    if (!v90_parse_dil_descriptor(&parsed, bits, bit_len))
        return false;

    *desc = parsed;
    return true;
}

void vpcm_v92_select_profile_from_dil(const v90_dil_analysis_t *analysis,
                                      uint8_t *downstream_drn,
                                      uint8_t *upstream_drn)
{
    if (!downstream_drn || !upstream_drn)
        return;
    if (analysis && analysis->recommended_downstream_drn != 0 && analysis->recommended_upstream_drn != 0) {
        *downstream_drn = analysis->recommended_downstream_drn;
        *upstream_drn = analysis->recommended_upstream_drn;
    } else {
        *downstream_drn = 19;
        *upstream_drn = 16;
    }
}

/*
 * vpcm_v90_run_data_mode — per-chunk data mode loop for V.90/V.92.
 *
 * Downstream (digital→analogue): V.90 PCM codewords via v90_tx_codewords.
 * Upstream (analogue→digital): V.34 caller TX → G.711 encode when native_caller
 * is non-NULL; falls back to V.91 transparent encoding otherwise.
 *
 * Both directions are recorded to the call pair via vpcm_v90_record_duplex
 * (analogue TX first, digital TX second — matching vpcm_record_call_duplex_g711
 * which maps the first argument to pair->caller->tx).
 */
static bool vpcm_v90_run_data_mode(v91_law_t law,
                                   const vpcm_v90_startup_contract_io_t *io,
                                   v34_state_t *native_caller,
                                   v90_state_t *downstream_tx,
                                   v90_state_t *downstream_rx,
                                   v91_state_t *upstream_rx,
                                   v91_state_t *upstream_tx_fallback,
                                   const uint8_t *down_data_in,
                                   uint8_t *down_data_out,
                                   const uint8_t *up_data_in,
                                   uint8_t *up_data_out,
                                   uint8_t *down_pcm_tx,
                                   uint8_t *down_pcm_rx,
                                   uint8_t *up_pcm_tx,
                                   uint8_t *up_pcm_rx,
                                   int total_codewords,
                                   int down_total_bytes,
                                   int up_total_bytes,
                                   const vpcm_cp_frame_t *cp_up_ack)
{
    int up_bits_per_frame;
    int offset;

    up_bits_per_frame = (int) cp_up_ack->drn + 20;

    for (offset = 0; offset < total_codewords; offset += VPCM_V90_DATA_CHUNK_CODEWORDS) {
        int chunk_codewords;
        int chunk_frames;
        int up_byte_offset;
        int chunk_up_bytes;
        int chunk_down_bytes;
        int down_produced;
        int down_consumed;
        int up_consumed;
        int16_t up_linear[VPCM_V90_DATA_CHUNK_CODEWORDS];

        chunk_codewords  = total_codewords - offset;
        if (chunk_codewords > VPCM_V90_DATA_CHUNK_CODEWORDS)
            chunk_codewords = VPCM_V90_DATA_CHUNK_CODEWORDS;
        chunk_frames     = chunk_codewords / VPCM_CP_FRAME_INTERVALS;
        up_byte_offset   = (offset / VPCM_CP_FRAME_INTERVALS) * up_bits_per_frame / 8;
        chunk_up_bytes   = (chunk_frames * up_bits_per_frame) / 8;
        chunk_down_bytes = chunk_codewords;

        /* Downstream: V.90 PCM encode from data_in */
        down_produced = v90_tx_codewords(downstream_tx,
                                         down_pcm_tx + offset, chunk_codewords,
                                         down_data_in + offset, chunk_down_bytes);
        if (down_produced != chunk_codewords) {
            fprintf(stderr, "V.90 data mode: downstream TX short (got %d want %d) at offset %d\n",
                    down_produced, chunk_codewords, offset);
            return false;
        }

        /* Upstream: V.34 caller TX → G.711 (or V.91 transparent fallback) */
        if (native_caller) {
            if (v34_tx(native_caller, up_linear, chunk_codewords) != chunk_codewords) {
                fprintf(stderr, "V.90 data mode: V.34 upstream TX short at offset %d\n", offset);
                return false;
            }
            vpcm_v90_encode_linear_chunk_to_g711(law, up_linear,
                                                 up_pcm_tx + offset, chunk_codewords);
        } else {
            /* Fallback: V.91 transparent codewords */
            v91_tx_codewords(upstream_tx_fallback,
                             up_pcm_tx + offset, chunk_codewords,
                             up_data_in + up_byte_offset, chunk_up_bytes);
        }

        /* Record to call pair: analogue (upstream) TX first, digital (downstream) TX second.
         * vpcm_record_call_duplex_g711 maps first arg → pair->caller->tx (analogue modem),
         * second arg → pair->answerer->tx (digital server). */
        if (!vpcm_v90_record_duplex(io, up_pcm_tx + offset, down_pcm_tx + offset,
                                    chunk_codewords)) {
            fprintf(stderr, "V.90 data mode: record failed at offset %d\n", offset);
            return false;
        }

        /* Loopback transport */
        memcpy(down_pcm_rx + offset, down_pcm_tx + offset, (size_t) chunk_codewords);
        memcpy(up_pcm_rx + offset, up_pcm_tx + offset, (size_t) chunk_codewords);

        /* Downstream decode */
        down_consumed = v90_rx_codewords(downstream_rx,
                                         down_data_out + offset, chunk_down_bytes,
                                         down_pcm_rx + offset, chunk_codewords);
        if (down_consumed != chunk_down_bytes) {
            fprintf(stderr, "V.90 data mode: downstream RX short at offset %d\n", offset);
            return false;
        }

        /* Upstream decode — V.91 transparent RX.
         * When native_caller is set the upstream signal is real V.34 modulation;
         * the V.91 transparent decoder will produce garbage bytes (expected),
         * so we skip the consume-count check in that case. */
        up_consumed = v91_rx_codewords(upstream_rx,
                                       up_data_out + up_byte_offset, chunk_up_bytes,
                                       up_pcm_rx + offset, chunk_codewords);
        if (!native_caller && up_consumed != chunk_up_bytes) {
            fprintf(stderr, "V.90 data mode: upstream RX short at offset %d\n", offset);
            return false;
        }
        (void) up_consumed;
        (void) up_total_bytes;
        (void) down_total_bytes;
    }
    return true;
}

bool vpcm_v90_session_run_startup_contract(vpcm_v90_session_t *session,
                                           const vpcm_v90_startup_contract_params_t *params,
                                           const vpcm_v90_startup_contract_io_t *io,
                                           vpcm_v90_startup_contract_report_t *report)
{
    v91_dil_desc_t default_dil;
    v91_dil_desc_t digital_dil_compat;
    v90_dil_desc_t digital_dil;
    v90_dil_analysis_t digital_dil_analysis;
    v91_state_t caller_startup;
    v91_state_t answerer_startup;
    v91_state_t caller_rx;
    v91_state_t answerer_tx;
    vpcm_cp_frame_t cp_rx;
    vpcm_v90_startup_contract_report_t local_report;
    uint8_t phase1_peer_buf[V91_EU_SYMBOLS + V91_DEFAULT_DIL_SYMBOLS];
    uint8_t startup_buf[V91_EU_SYMBOLS + V91_DEFAULT_DIL_SYMBOLS];
    uint8_t scr_buf[18];
    uint8_t cp_buf[VPCM_CP_MAX_BITS];
    uint8_t es_buf[V91_ES_SYMBOLS];
    uint8_t b1_buf[V91_B1_SYMBOLS];
    uint8_t *down_data_in;
    uint8_t *down_data_out;
    uint8_t *up_data_in;
    uint8_t *up_data_out;
    uint8_t *down_pcm_tx;
    uint8_t *down_pcm_rx;
    uint8_t *up_pcm_tx;
    uint8_t *up_pcm_rx;
    v90_state_t *downstream_tx;
    v90_state_t *downstream_rx;
    v34_state_t *native_caller;
    vpcm_v90_bit_feeder_t caller_bit_feeder;
    int data_frames;
    int total_codewords;
    int up_total_bits;
    int down_total_bytes;
    int up_total_bytes;
    int startup_len;
    int cp_len;
    int data_seconds;
    uint8_t downstream_drn;
    uint8_t upstream_drn;

    if (!session || !params)
        return false;

    memset(&local_report, 0, sizeof(local_report));
    memset(&digital_dil_analysis, 0, sizeof(digital_dil_analysis));
    memset(&caller_bit_feeder, 0, sizeof(caller_bit_feeder));
    downstream_tx = NULL;
    downstream_rx = NULL;
    native_caller = NULL;
    data_seconds = params->data_seconds;
    if (data_seconds <= 0)
        data_seconds = 10;

    vpcm_v90_session_init(session, params->law);
    v90_info0a_init(&local_report.analogue_info0a);
    v90_info1a_init(&local_report.analogue_info1a);
    local_report.phase2_contract_valid =
        v90_build_info0a_bits(local_report.analogue_info0a_bits,
                              (int) sizeof(local_report.analogue_info0a_bits),
                              &local_report.analogue_info0a)
        && v90_parse_info0a_bits(&local_report.analogue_info0a,
                                 local_report.analogue_info0a_bits,
                                 V90_INFO0A_BITS)
        && v90_build_info1a_bits(local_report.analogue_info1a_bits,
                                 (int) sizeof(local_report.analogue_info1a_bits),
                                 &local_report.analogue_info1a)
        && v90_parse_info1a_bits(&local_report.analogue_info1a,
                                 local_report.analogue_info1a_bits,
                                 V90_INFO1A_BITS);
    v91_default_dil_init(&default_dil);
    if (!vpcm_v92_init_digital_dil_from_ja(&digital_dil, params->echo_limited))
        return false;
    vpcm_v90_copy_dil_to_v91_compat(&digital_dil_compat, &digital_dil);
    if (!v90_analyse_dil_descriptor(&digital_dil, &digital_dil_analysis))
        return false;
    local_report.phase3_digital_dil = digital_dil;
    local_report.phase3_digital_dil_valid = true;
    local_report.phase3_digital_dil_analysis = digital_dil_analysis;
    local_report.phase3_digital_dil_analysis_valid = true;
    vpcm_v90_map_dil_analysis_to_v91_compat(&local_report.digital_dil_analysis, &digital_dil_analysis);
    local_report.digital_dil_analysis_valid = true;
    v91_init(&caller_startup, params->law, V91_MODE_TRANSPARENT);
    v91_init(&answerer_startup, params->law, V91_MODE_TRANSPARENT);
    v91_init(&caller_rx, params->law, V91_MODE_TRANSPARENT);
    v91_init(&answerer_tx, params->law, V91_MODE_TRANSPARENT);

    vpcm_v90_session_set_state(session, VPCM_V90_MODEM_PHASE1);
    if (v91_tx_phase1_silence_codewords(&caller_startup, startup_buf, (int) sizeof(startup_buf)) != V91_PHASE1_SILENCE_SYMBOLS
        || v91_tx_phase1_silence_codewords(&answerer_startup, phase1_peer_buf, (int) sizeof(phase1_peer_buf)) != V91_PHASE1_SILENCE_SYMBOLS
        || !vpcm_v90_record_duplex(io, startup_buf, phase1_peer_buf, V91_PHASE1_SILENCE_SYMBOLS)
        || v91_tx_ez_codewords(&caller_startup, startup_buf, (int) sizeof(startup_buf)) != V91_EZ_SYMBOLS
        || v91_tx_ez_codewords(&answerer_startup, phase1_peer_buf, (int) sizeof(phase1_peer_buf)) != V91_EZ_SYMBOLS
        || !vpcm_v90_record_duplex(io, startup_buf, phase1_peer_buf, V91_EZ_SYMBOLS)) {
        return false;
    }

    vpcm_v90_session_set_state(session, VPCM_V90_MODEM_PHASE2);
    if (!vpcm_v90_run_phase2_exchange(params->law, io,
                                       &digital_dil, &digital_dil_analysis,
                                       &local_report,
                                       &caller_bit_feeder,
                                       &native_caller)) {
        return false;
    }
    if (local_report.phase2_received_info1a_valid) {
        local_report.analogue_info1a.u_info = local_report.phase2_received_info1a.u_info;
        local_report.phase2_contract_valid =
            local_report.phase2_contract_valid
            && v90_build_info1a_bits(local_report.analogue_info1a_bits,
                                     (int) sizeof(local_report.analogue_info1a_bits),
                                     &local_report.analogue_info1a)
            && v90_parse_info1a_bits(&local_report.analogue_info1a,
                                     local_report.analogue_info1a_bits,
                                     V90_INFO1A_BITS);
    }
    vpcm_v90_prepare_placeholder_info_frames(params,
                                             &digital_dil_analysis,
                                             &local_report,
                                             &local_report.caller_info,
                                             &local_report.answerer_info);
    vpcm_v90_seed_placeholder_info_state(&caller_startup,
                                         &local_report.caller_info,
                                         &local_report.answerer_info);
    vpcm_v90_seed_placeholder_info_state(&answerer_startup,
                                         &local_report.answerer_info,
                                         &local_report.caller_info);

    /* Phase 3+4 downstream PCM and upstream V.34 are now recorded together
       by the coupled training loop inside vpcm_v90_run_phase2_exchange above.
       The compat V.91 DIL/SCR path below runs only when native training did not
       complete (phase3/4_native_analogue_completed flags gate its recording). */
    vpcm_v90_session_set_state(session, VPCM_V90_MODEM_PHASE3);
    startup_len = v91_tx_startup_dil_sequence_codewords(&caller_startup,
                                                        startup_buf,
                                                        (int) sizeof(startup_buf),
                                                        &digital_dil_compat,
                                                        NULL);
    if (startup_len <= 0
        || !v91_note_received_dil(&answerer_startup, &digital_dil_compat, NULL)) {
        return false;
    }
    vpcm_v92_select_profile_from_dil(&digital_dil_analysis, &downstream_drn, &upstream_drn);

    startup_len = v91_tx_startup_dil_sequence_codewords(&answerer_startup,
                                                        startup_buf,
                                                        (int) sizeof(startup_buf),
                                                        &default_dil,
                                                        NULL);
    if (startup_len <= 0
        || !v91_note_received_dil(&caller_startup, &default_dil, NULL)
        || (!local_report.phase3_native_analogue_completed
            && !vpcm_v90_record_simplex(io, params->law, false, startup_buf, startup_len))) {
        return false;
    }

    vpcm_v90_session_set_state(session, VPCM_V90_MODEM_PHASE3);
    if (v91_tx_scr_codewords(&caller_startup, scr_buf, (int) sizeof(scr_buf), 18) != 18
        || !v91_rx_scr_codewords(&answerer_startup, scr_buf, 18, false)
        || v91_tx_scr_codewords(&answerer_startup, scr_buf, (int) sizeof(scr_buf), 18) != 18
        || !v91_rx_scr_codewords(&caller_startup, scr_buf, 18, false)
        || (!local_report.phase3_native_analogue_completed
            && !vpcm_v90_record_simplex(io, params->law, false, scr_buf, 18))) {
        return false;
    }

    /* Phase 4 is still approximated internally by the shared CP/B1
       compatibility path. When the live native analogue probe already carried
       the real Phase 4 waveform through MP/E/data, we suppress CP/B1 session
       recording so the harness timeline reflects the native path. */
    vpcm_v90_session_set_state(session, VPCM_V90_MODEM_PHASE4);
    vpcm_cp_init(&local_report.cp_down_offer);
    local_report.cp_down_offer.transparent_mode_granted = false;
    local_report.cp_down_offer.v90_compatibility = true;
    local_report.cp_down_offer.drn = downstream_drn;
    local_report.cp_down_offer.constellation_count = 1;
    memset(local_report.cp_down_offer.dfi, 0, sizeof(local_report.cp_down_offer.dfi));
    vpcm_cp_enable_all_ucodes(local_report.cp_down_offer.masks[0]);
    cp_len = v91_tx_cp_codewords(&caller_startup,
                                 cp_buf,
                                 (int) sizeof(cp_buf),
                                 &local_report.cp_down_offer,
                                 true);
    if (cp_len <= 0
        || !v91_rx_cp_codewords(&answerer_startup, cp_buf, cp_len, &cp_rx, true)
        || !vpcm_cp_frames_equal(&local_report.cp_down_offer, &cp_rx)
        || (!local_report.phase4_native_analogue_completed
            && !vpcm_v90_record_simplex(io, params->law, true, cp_buf, cp_len))) {
        return false;
    }

    local_report.cp_down_ack = local_report.cp_down_offer;
    local_report.cp_down_ack.acknowledge = true;
    cp_len = v91_tx_cp_codewords(&answerer_startup,
                                 cp_buf,
                                 (int) sizeof(cp_buf),
                                 &local_report.cp_down_ack,
                                 true);
    if (cp_len <= 0
        || !v91_rx_cp_codewords(&caller_startup, cp_buf, cp_len, &cp_rx, true)
        || !vpcm_cp_frames_equal(&local_report.cp_down_ack, &cp_rx)
        || (!local_report.phase4_native_analogue_completed
            && !vpcm_v90_record_simplex(io, params->law, false, cp_buf, cp_len))) {
        return false;
    }

    vpcm_v90_session_set_state(session, VPCM_V90_MODEM_PHASE4);
    if (v91_tx_es_codewords(&caller_startup, es_buf, (int) sizeof(es_buf)) != V91_ES_SYMBOLS
        || !v91_rx_es_codewords(&answerer_startup, es_buf, V91_ES_SYMBOLS, true)
        || (!local_report.phase4_native_analogue_completed
            && !vpcm_v90_record_simplex(io, params->law, true, es_buf, V91_ES_SYMBOLS))
        || v91_tx_b1_codewords(&caller_startup, b1_buf, (int) sizeof(b1_buf), &local_report.cp_down_ack) != V91_B1_SYMBOLS
        || !v91_rx_b1_codewords(&answerer_startup, b1_buf, V91_B1_SYMBOLS, &local_report.cp_down_ack)
        || (!local_report.phase4_native_analogue_completed
            && !vpcm_v90_record_simplex(io, params->law, true, b1_buf, V91_B1_SYMBOLS))) {
        return false;
    }

    vpcm_v90_session_set_state(session, VPCM_V90_MODEM_PHASE4);
    vpcm_cp_init(&local_report.cp_up_offer);
    local_report.cp_up_offer.transparent_mode_granted = false;
    local_report.cp_up_offer.v90_compatibility = true;
    local_report.cp_up_offer.drn = upstream_drn;
    local_report.cp_up_offer.constellation_count = 1;
    memset(local_report.cp_up_offer.dfi, 0, sizeof(local_report.cp_up_offer.dfi));
    vpcm_cp_enable_all_ucodes(local_report.cp_up_offer.masks[0]);
    cp_len = v91_tx_cp_codewords(&answerer_startup,
                                 cp_buf,
                                 (int) sizeof(cp_buf),
                                 &local_report.cp_up_offer,
                                 true);
    if (cp_len <= 0
        || !v91_rx_cp_codewords(&caller_startup, cp_buf, cp_len, &cp_rx, true)
        || !vpcm_cp_frames_equal(&local_report.cp_up_offer, &cp_rx)
        || (!local_report.phase4_native_analogue_completed
            && !vpcm_v90_record_simplex(io, params->law, false, cp_buf, cp_len))) {
        return false;
    }

    local_report.cp_up_ack = local_report.cp_up_offer;
    local_report.cp_up_ack.acknowledge = true;
    cp_len = v91_tx_cp_codewords(&caller_startup,
                                 cp_buf,
                                 (int) sizeof(cp_buf),
                                 &local_report.cp_up_ack,
                                 true);
    if (cp_len <= 0
        || !v91_rx_cp_codewords(&answerer_startup, cp_buf, cp_len, &cp_rx, true)
        || !vpcm_cp_frames_equal(&local_report.cp_up_ack, &cp_rx)
        || (!local_report.phase4_native_analogue_completed
            && !vpcm_v90_record_simplex(io, params->law, true, cp_buf, cp_len))) {
        return false;
    }

    vpcm_v90_session_set_state(session, VPCM_V90_MODEM_PHASE4);
    if (v91_tx_es_codewords(&answerer_startup, es_buf, (int) sizeof(es_buf)) != V91_ES_SYMBOLS
        || !v91_rx_es_codewords(&caller_startup, es_buf, V91_ES_SYMBOLS, true)
        || (!local_report.phase4_native_analogue_completed
            && !vpcm_v90_record_simplex(io, params->law, false, es_buf, V91_ES_SYMBOLS))
        || v91_tx_b1_codewords(&answerer_startup, b1_buf, (int) sizeof(b1_buf), &local_report.cp_up_ack) != V91_B1_SYMBOLS
        || !v91_rx_b1_codewords(&caller_startup, b1_buf, V91_B1_SYMBOLS, &local_report.cp_up_ack)
        || (!local_report.phase4_native_analogue_completed
            && !vpcm_v90_record_simplex(io, params->law, false, b1_buf, V91_B1_SYMBOLS))) {
        return false;
    }

    vpcm_v90_session_set_state(session, VPCM_V90_MODEM_DATA);
    downstream_tx = v90_init_data_pump(vpcm_v90_data_law(params->law));
    downstream_rx = v90_init_data_pump(vpcm_v90_data_law(params->law));
    if (!downstream_tx
        || !downstream_rx
        || !v91_activate_data_mode(&answerer_tx, &local_report.cp_up_ack)
        || !v91_activate_data_mode(&caller_rx, &local_report.cp_up_ack)) {
        v90_free(downstream_tx);
        v90_free(downstream_rx);
        v34_free(native_caller);
        return false;
    }

    data_frames = vpcm_v90_frames_from_seconds(data_seconds);
    if (data_frames < 1)
        data_frames = VPCM_V90_NOMINAL_10S_FRAMES;
    data_frames = vpcm_v90_align_frames_for_duplex_bytes(data_frames,
                                                         8 * VPCM_CP_FRAME_INTERVALS,
                                                         (int) local_report.cp_up_ack.drn + 20);
    /* Also align to V.34 natural chunk so the per-chunk loop has no partial tail. */
    while ((data_frames * VPCM_CP_FRAME_INTERVALS) % VPCM_V90_DATA_CHUNK_CODEWORDS != 0)
        data_frames++;
    total_codewords = data_frames * VPCM_CP_FRAME_INTERVALS;
    up_total_bits = data_frames * ((int) local_report.cp_up_ack.drn + 20);
    down_total_bytes = total_codewords;
    up_total_bytes = up_total_bits / 8;

    down_data_in = (uint8_t *) malloc((size_t) down_total_bytes);
    down_data_out = (uint8_t *) malloc((size_t) down_total_bytes);
    up_data_in = (uint8_t *) malloc((size_t) up_total_bytes);
    up_data_out = (uint8_t *) malloc((size_t) up_total_bytes);
    down_pcm_tx = (uint8_t *) malloc((size_t) total_codewords);
    down_pcm_rx = (uint8_t *) malloc((size_t) total_codewords);
    up_pcm_tx = (uint8_t *) malloc((size_t) total_codewords);
    up_pcm_rx = (uint8_t *) malloc((size_t) total_codewords);
    if (!down_data_in || !down_data_out || !up_data_in || !up_data_out
        || !down_pcm_tx || !down_pcm_rx || !up_pcm_tx || !up_pcm_rx) {
        free(down_data_in);
        free(down_data_out);
        free(up_data_in);
        free(up_data_out);
        free(down_pcm_tx);
        free(down_pcm_rx);
        free(up_pcm_tx);
        free(up_pcm_rx);
        v90_free(downstream_tx);
        v90_free(downstream_rx);
        v34_free(native_caller);
        return false;
    }

    vpcm_v90_fill_pattern(down_data_in, down_total_bytes, params->seed_base ^ 0x00D04E00U);
    vpcm_v90_fill_pattern(up_data_in, up_total_bytes, params->seed_base ^ 0x00A0B000U);
    memset(down_data_out, 0, (size_t) down_total_bytes);
    memset(up_data_out, 0, (size_t) up_total_bytes);

    /* Arm the bit feeder with the real upstream data now that up_data_in is filled. */
    vpcm_v90_bit_feeder_init(&caller_bit_feeder, up_data_in, up_total_bytes);

    if (!vpcm_v90_run_data_mode(params->law, io,
                                 native_caller,
                                 downstream_tx, downstream_rx,
                                 &caller_rx, &answerer_tx,
                                 down_data_in, down_data_out,
                                 up_data_in, up_data_out,
                                 down_pcm_tx, down_pcm_rx,
                                 up_pcm_tx, up_pcm_rx,
                                 total_codewords,
                                 down_total_bytes, up_total_bytes,
                                 &local_report.cp_up_ack)) {
        free(down_data_in);
        free(down_data_out);
        free(up_data_in);
        free(up_data_out);
        free(down_pcm_tx);
        free(down_pcm_rx);
        free(up_pcm_tx);
        free(up_pcm_rx);
        v90_free(downstream_tx);
        v90_free(downstream_rx);
        v34_free(native_caller);
        return false;
    }

    if (!vpcm_v90_expect_equal("V.90 downstream payload", down_data_in, down_data_out, down_total_bytes)
        || (!native_caller
            && !vpcm_v90_expect_equal("V.92 upstream payload", up_data_in, up_data_out, up_total_bytes))) {
        free(down_data_in);
        free(down_data_out);
        free(up_data_in);
        free(up_data_out);
        free(down_pcm_tx);
        free(down_pcm_rx);
        free(up_pcm_tx);
        free(up_pcm_rx);
        v90_free(downstream_tx);
        v90_free(downstream_rx);
        v34_free(native_caller);
        return false;
    }

    local_report.data_seconds = data_seconds;
    local_report.total_codewords = total_codewords;
    local_report.down_total_bytes = down_total_bytes;
    local_report.up_total_bytes = up_total_bytes;
    local_report.sample_down_data_len = VPCM_V90_SAMPLE_PCM_LEN;
    local_report.sample_up_data_len = (VPCM_V90_SAMPLE_PCM_LEN * ((int) local_report.cp_up_ack.drn + 20)) / 48;
    if (local_report.sample_down_data_len < 1)
        local_report.sample_down_data_len = 1;
    if (local_report.sample_up_data_len < 1)
        local_report.sample_up_data_len = 1;
    if (local_report.sample_down_data_len > VPCM_V90_SAMPLE_DATA_MAX)
        local_report.sample_down_data_len = VPCM_V90_SAMPLE_DATA_MAX;
    if (local_report.sample_up_data_len > VPCM_V90_SAMPLE_DATA_MAX)
        local_report.sample_up_data_len = VPCM_V90_SAMPLE_DATA_MAX;
    if (local_report.sample_down_data_len > down_total_bytes)
        local_report.sample_down_data_len = down_total_bytes;
    if (local_report.sample_up_data_len > up_total_bytes)
        local_report.sample_up_data_len = up_total_bytes;

    vpcm_v90_copy_sample(local_report.down_data_in_sample,
                         VPCM_V90_SAMPLE_DATA_MAX,
                         down_data_in,
                         local_report.sample_down_data_len);
    vpcm_v90_copy_sample(local_report.down_pcm_tx_sample,
                         VPCM_V90_SAMPLE_PCM_LEN,
                         down_pcm_tx,
                         VPCM_V90_SAMPLE_PCM_LEN);
    vpcm_v90_copy_sample(local_report.down_pcm_rx_sample,
                         VPCM_V90_SAMPLE_PCM_LEN,
                         down_pcm_rx,
                         VPCM_V90_SAMPLE_PCM_LEN);
    vpcm_v90_copy_sample(local_report.down_data_out_sample,
                         VPCM_V90_SAMPLE_DATA_MAX,
                         down_data_out,
                         local_report.sample_down_data_len);
    vpcm_v90_copy_sample(local_report.up_data_in_sample,
                         VPCM_V90_SAMPLE_DATA_MAX,
                         up_data_in,
                         local_report.sample_up_data_len);
    vpcm_v90_copy_sample(local_report.up_pcm_tx_sample,
                         VPCM_V90_SAMPLE_PCM_LEN,
                         up_pcm_tx,
                         VPCM_V90_SAMPLE_PCM_LEN);
    vpcm_v90_copy_sample(local_report.up_pcm_rx_sample,
                         VPCM_V90_SAMPLE_PCM_LEN,
                         up_pcm_rx,
                         VPCM_V90_SAMPLE_PCM_LEN);
    vpcm_v90_copy_sample(local_report.up_data_out_sample,
                         VPCM_V90_SAMPLE_DATA_MAX,
                         up_data_out,
                         local_report.sample_up_data_len);

    free(down_data_in);
    free(down_data_out);
    free(up_data_in);
    free(up_data_out);
    free(down_pcm_tx);
    free(down_pcm_rx);
    free(up_pcm_tx);
    free(up_pcm_rx);
    v90_free(downstream_tx);
    v90_free(downstream_rx);
    v34_free(native_caller);

    if (report)
        *report = local_report;
    return true;
}

const char *vpcm_v90_modem_state_to_str(vpcm_v90_modem_state_t state)
{
    switch (state) {
    case VPCM_V90_MODEM_IDLE: return "IDLE";
    case VPCM_V90_MODEM_PHASE1: return "PHASE1";
    case VPCM_V90_MODEM_PHASE2: return "PHASE2_INFO";
    case VPCM_V90_MODEM_PHASE3: return "PHASE3";
    case VPCM_V90_MODEM_PHASE4: return "PHASE4";
    case VPCM_V90_MODEM_DATA: return "DATA";
    case VPCM_V90_MODEM_CLEARDOWN: return "CLEARDOWN";
    default: return "UNKNOWN";
    }
}
