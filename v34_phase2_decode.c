#include "v34_phase2_decode.h"

#include <math.h>
#include <string.h>

static double v34_phase2_rms_energy_db(const int16_t *samples, int len)
{
    double sum = 0.0;
    double rms;

    if (!samples || len <= 0)
        return -100.0;
    for (int i = 0; i < len; i++)
        sum += (double) samples[i] * (double) samples[i];
    rms = sqrt(sum / (double) len);
    if (rms < 1.0)
        return -100.0;
    return 20.0 * log10(rms / 32768.0);
}

static bool v34_phase2_find_activity_window(const int16_t *samples,
                                            int total_samples,
                                            int sample_rate,
                                            int *start_out,
                                            int *end_out)
{
    enum {
        WINDOW_MS = 100,
        STEP_MS = 50,
        PAD_MS = 300
    };
    const double activity_floor_db = -58.0;
    int window;
    int step;
    int pad;
    double max_db = -100.0;
    int first_active = -1;
    int last_active = -1;

    if (!samples || total_samples <= 0 || sample_rate <= 0 || !start_out || !end_out)
        return false;

    window = (sample_rate * WINDOW_MS) / 1000;
    step = (sample_rate * STEP_MS) / 1000;
    pad = (sample_rate * PAD_MS) / 1000;
    if (window <= 0 || step <= 0) {
        *start_out = 0;
        *end_out = total_samples;
        return false;
    }

    for (int start = 0; start < total_samples; start += step) {
        int len = total_samples - start;
        double db;

        if (len > window)
            len = window;
        db = v34_phase2_rms_energy_db(samples + start, len);
        if (db > max_db)
            max_db = db;
    }

    if (max_db <= activity_floor_db) {
        *start_out = 0;
        *end_out = total_samples;
        return false;
    }

    for (int start = 0; start < total_samples; start += step) {
        int len = total_samples - start;
        double db;

        if (len > window)
            len = window;
        db = v34_phase2_rms_energy_db(samples + start, len);
        if (db < activity_floor_db)
            continue;
        if (first_active < 0)
            first_active = start;
        last_active = start + len;
    }

    if (first_active < 0 || last_active <= first_active) {
        *start_out = 0;
        *end_out = total_samples;
        return false;
    }

    *start_out = first_active - pad;
    *end_out = last_active + pad;
    if (*start_out < 0)
        *start_out = 0;
    if (*end_out > total_samples)
        *end_out = total_samples;
    return (*end_out - *start_out) < total_samples;
}

static void v34_phase2_offset_result_samples(decode_v34_result_t *result, int sample_offset)
{
#define V34_PHASE2_OFFSET_FIELD(field) \
    do { \
        if (result->field >= 0) \
            result->field += sample_offset; \
    } while (0)

    if (!result || sample_offset == 0)
        return;

    V34_PHASE2_OFFSET_FIELD(info0_sample);
    V34_PHASE2_OFFSET_FIELD(info1_sample);
    V34_PHASE2_OFFSET_FIELD(info0_ok_event_sample);
    V34_PHASE2_OFFSET_FIELD(info0_bad_event_sample);
    V34_PHASE2_OFFSET_FIELD(info1_ok_event_sample);
    V34_PHASE2_OFFSET_FIELD(info1_bad_event_sample);
    V34_PHASE2_OFFSET_FIELD(phase3_sample);
    V34_PHASE2_OFFSET_FIELD(tx_first_s_sample);
    V34_PHASE2_OFFSET_FIELD(tx_first_not_s_sample);
    V34_PHASE2_OFFSET_FIELD(tx_md_sample);
    V34_PHASE2_OFFSET_FIELD(tx_second_s_sample);
    V34_PHASE2_OFFSET_FIELD(tx_second_not_s_sample);
    V34_PHASE2_OFFSET_FIELD(tx_pp_sample);
    V34_PHASE2_OFFSET_FIELD(tx_pp_end_sample);
    V34_PHASE2_OFFSET_FIELD(rx_pp_detect_sample);
    V34_PHASE2_OFFSET_FIELD(rx_pp_complete_sample);
    V34_PHASE2_OFFSET_FIELD(tx_trn_sample);
    V34_PHASE2_OFFSET_FIELD(phase3_trn_lock_sample);
    V34_PHASE2_OFFSET_FIELD(phase3_trn_strong_sample);
    V34_PHASE2_OFFSET_FIELD(tx_ja_sample);
    V34_PHASE2_OFFSET_FIELD(tx_jdashed_sample);
    V34_PHASE2_OFFSET_FIELD(rx_s_event_sample);
    V34_PHASE2_OFFSET_FIELD(rx_tone_a_sample);
    V34_PHASE2_OFFSET_FIELD(rx_tone_b_sample);
    V34_PHASE2_OFFSET_FIELD(rx_tone_a_reversal_sample);
    V34_PHASE2_OFFSET_FIELD(rx_tone_b_reversal_sample);
    V34_PHASE2_OFFSET_FIELD(tx_tone_a_sample);
    V34_PHASE2_OFFSET_FIELD(tx_tone_b_sample);
    V34_PHASE2_OFFSET_FIELD(tx_tone_a_reversal_sample);
    V34_PHASE2_OFFSET_FIELD(tx_tone_b_reversal_sample);
    V34_PHASE2_OFFSET_FIELD(tx_info1_sample);
    V34_PHASE2_OFFSET_FIELD(rx_phase4_s_sample);
    V34_PHASE2_OFFSET_FIELD(rx_phase4_sbar_sample);
    V34_PHASE2_OFFSET_FIELD(rx_phase4_trn_sample);
    V34_PHASE2_OFFSET_FIELD(phase4_ready_sample);
    V34_PHASE2_OFFSET_FIELD(phase4_sample);
    V34_PHASE2_OFFSET_FIELD(failure_sample);

#undef V34_PHASE2_OFFSET_FIELD
}

static void merge_info0_from_rescue(decode_v34_result_t *dst, const decode_v34_result_t *src)
{
    if (!dst || !src || !src->info0_seen)
        return;

    dst->info0_seen = true;
    dst->info0_is_d = src->info0_is_d;
    dst->info0_sample = src->info0_sample;
    if (!dst->info0_ok_event_seen && src->info0_ok_event_seen) {
        dst->info0_ok_event_seen = true;
        dst->info0_ok_event_sample = src->info0_ok_event_sample;
    }
    if (!dst->info0_bad_event_seen && src->info0_bad_event_seen) {
        dst->info0_bad_event_seen = true;
        dst->info0_bad_event_sample = src->info0_bad_event_sample;
    }
    dst->info0_raw = src->info0_raw;
    dst->info0a = src->info0a;
    dst->info0_from_rescue = true;
}

static void merge_info_event_diagnostics_from_rescue(decode_v34_result_t *dst, const decode_v34_result_t *src)
{
    if (!dst || !src)
        return;

    if (!dst->info0_ok_event_seen && src->info0_ok_event_seen) {
        dst->info0_ok_event_seen = true;
        dst->info0_ok_event_sample = src->info0_ok_event_sample;
    }
    if (!dst->info0_bad_event_seen && src->info0_bad_event_seen) {
        dst->info0_bad_event_seen = true;
        dst->info0_bad_event_sample = src->info0_bad_event_sample;
    }
    if (!dst->info1_ok_event_seen && src->info1_ok_event_seen) {
        dst->info1_ok_event_seen = true;
        dst->info1_ok_event_sample = src->info1_ok_event_sample;
    }
    if (!dst->info1_bad_event_seen && src->info1_bad_event_seen) {
        dst->info1_bad_event_seen = true;
        dst->info1_bad_event_sample = src->info1_bad_event_sample;
    }
}

static void merge_info1_from_rescue(decode_v34_result_t *dst, const decode_v34_result_t *src)
{
    if (!dst || !src || !src->info1_seen)
        return;

    dst->info1_seen = true;
    dst->info1_is_d = src->info1_is_d;
    dst->info1_sample = src->info1_sample;
    if (!dst->info1_ok_event_seen && src->info1_ok_event_seen) {
        dst->info1_ok_event_seen = true;
        dst->info1_ok_event_sample = src->info1_ok_event_sample;
    }
    if (!dst->info1_bad_event_seen && src->info1_bad_event_seen) {
        dst->info1_bad_event_seen = true;
        dst->info1_bad_event_sample = src->info1_bad_event_sample;
    }
    dst->u_info = src->u_info;
    dst->u_info_from_info1a = src->u_info_from_info1a;
    dst->tx_info1_sample = src->tx_info1_sample;
    if (src->info1_is_d)
        dst->info1d = src->info1d;
    else {
        dst->info1a = src->info1a;
        dst->info1a_raw = src->info1a_raw;
    }
    dst->info1_from_rescue = true;
}

void v34_phase2_engine_init(v34_phase2_engine_t *engine,
                            v34_phase2_pass_fn_t decode_pass,
                            void *decode_pass_ctx)
{
    if (!engine)
        return;

    memset(engine, 0, sizeof(*engine));
    engine->decode_pass = decode_pass;
    engine->decode_pass_ctx = decode_pass_ctx;
}

int v34_phase2_result_spec_score(const decode_v34_result_t *result)
{
    int score = 0;

    if (!result)
        return -1;
    if (result->tx_ja_sample >= 0)
        score += 4000;
    if (result->tx_trn_sample >= 0)
        score += 1500;
    if (result->tx_pp_sample >= 0)
        score += 800;
    if (result->tx_first_s_sample >= 0 || result->tx_first_not_s_sample >= 0)
        score += 400;
    if (result->phase4_seen)
        score += 1000;
    if (result->phase4_ready_seen)
        score += 300;
    if (result->rx_pp_started)
        score += 80;
    if (result->phase3_seen)
        score += 60;
    if (result->info1_seen)
        score += 30;
    if (result->info0_seen)
        score += 15;

    return score;
}

void v34_phase2_decode_pair(v34_phase2_engine_t *engine,
                            const int16_t *samples,
                            int total_samples,
                            v91_law_t law,
                            bool allow_info_rate_infer,
                            decode_v34_result_t *answerer,
                            bool *have_answerer,
                            decode_v34_result_t *caller,
                            bool *have_caller)
{
    static const float rescue_cutoffs[] = { -60.0f, -68.0f };
    int decode_start = 0;
    int decode_end = total_samples;
    const int16_t *decode_samples = samples;
    int decode_sample_count = total_samples;

    if (have_answerer)
        *have_answerer = false;
    if (have_caller)
        *have_caller = false;
    if (!engine || !engine->decode_pass
        || !samples || total_samples <= 0
        || !answerer || !caller || !have_answerer || !have_caller) {
        return;
    }

    v34_phase2_find_activity_window(samples, total_samples, 8000, &decode_start, &decode_end);
    decode_samples = samples + decode_start;
    decode_sample_count = decode_end - decode_start;

    *have_answerer = engine->decode_pass(engine->decode_pass_ctx,
                                         decode_samples,
                                         decode_sample_count,
                                         law,
                                         false,
                                         -52.0f,
                                         allow_info_rate_infer,
                                         answerer);
    if (*have_answerer)
        v34_phase2_offset_result_samples(answerer, decode_start);
    *have_caller = engine->decode_pass(engine->decode_pass_ctx,
                                       decode_samples,
                                       decode_sample_count,
                                       law,
                                       true,
                                       -52.0f,
                                       allow_info_rate_infer,
                                       caller);
    if (*have_caller)
        v34_phase2_offset_result_samples(caller, decode_start);

    if (*have_answerer && (!answerer->info0_seen || !answerer->info1_seen)) {
        for (size_t i = 0; i < sizeof(rescue_cutoffs)/sizeof(rescue_cutoffs[0]); i++) {
            decode_v34_result_t rescue;

            if (!engine->decode_pass(engine->decode_pass_ctx,
                                     decode_samples,
                                     decode_sample_count,
                                     law,
                                     false,
                                     rescue_cutoffs[i],
                                     allow_info_rate_infer,
                                     &rescue)) {
                continue;
            }
            v34_phase2_offset_result_samples(&rescue, decode_start);
            merge_info_event_diagnostics_from_rescue(answerer, &rescue);
            if (!answerer->info0_seen && rescue.info0_seen)
                merge_info0_from_rescue(answerer, &rescue);
            if (!answerer->info1_seen && rescue.info1_seen)
                merge_info1_from_rescue(answerer, &rescue);
            if (answerer->info0_seen && answerer->info1_seen)
                break;
        }
    }
    if (*have_caller && (!caller->info0_seen || !caller->info1_seen)) {
        for (size_t i = 0; i < sizeof(rescue_cutoffs)/sizeof(rescue_cutoffs[0]); i++) {
            decode_v34_result_t rescue;

            if (!engine->decode_pass(engine->decode_pass_ctx,
                                     decode_samples,
                                     decode_sample_count,
                                     law,
                                     true,
                                     rescue_cutoffs[i],
                                     allow_info_rate_infer,
                                     &rescue)) {
                continue;
            }
            v34_phase2_offset_result_samples(&rescue, decode_start);
            merge_info_event_diagnostics_from_rescue(caller, &rescue);
            if (!caller->info0_seen && rescue.info0_seen)
                merge_info0_from_rescue(caller, &rescue);
            if (!caller->info1_seen && rescue.info1_seen)
                merge_info1_from_rescue(caller, &rescue);
            if (caller->info0_seen && caller->info1_seen)
                break;
        }
    }
}

void v34_phase2_decode_pair_cached(v34_phase2_engine_t *engine,
                                   const int16_t *samples,
                                   int total_samples,
                                   v91_law_t law,
                                   bool allow_info_rate_infer,
                                   decode_v34_result_t *answerer,
                                   bool *have_answerer,
                                   decode_v34_result_t *caller,
                                   bool *have_caller)
{
    v34_phase2_cache_entry_t *slot;

    if (have_answerer)
        *have_answerer = false;
    if (have_caller)
        *have_caller = false;
    if (!engine || !engine->decode_pass
        || !samples || total_samples <= 0
        || !answerer || !caller || !have_answerer || !have_caller) {
        return;
    }

    for (int i = 0; i < V34_PHASE2_CACHE_SLOTS; i++) {
        const v34_phase2_cache_entry_t *entry = &engine->cache[i];

        if (!entry->valid)
            continue;
        if (entry->samples != samples
            || entry->total_samples != total_samples
            || entry->law != law
            || entry->allow_info_rate_infer != allow_info_rate_infer) {
            continue;
        }
        *answerer = entry->answerer;
        *caller = entry->caller;
        *have_answerer = entry->have_answerer;
        *have_caller = entry->have_caller;
        return;
    }

    v34_phase2_decode_pair(engine,
                           samples,
                           total_samples,
                           law,
                           allow_info_rate_infer,
                           answerer,
                           have_answerer,
                           caller,
                           have_caller);

    slot = &engine->cache[engine->cache_next];
    engine->cache_next = (engine->cache_next + 1) % V34_PHASE2_CACHE_SLOTS;
    slot->valid = true;
    slot->samples = samples;
    slot->total_samples = total_samples;
    slot->law = law;
    slot->allow_info_rate_infer = allow_info_rate_infer;
    slot->answerer = *answerer;
    slot->caller = *caller;
    slot->have_answerer = *have_answerer;
    slot->have_caller = *have_caller;
}
