#include "v34_phase2_decode.h"

#include <math.h>
#include <string.h>

typedef struct {
    int start_sample;
    int end_sample;
    int active_hits;
    double peak_db;
} v34_phase2_candidate_window_t;

enum {
    V34_PHASE2_MAX_CANDIDATE_WINDOWS = 6
};

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

static int v34_phase2_candidate_rank(const v34_phase2_candidate_window_t *window)
{
    int duration_samples;
    int peak_term;

    if (!window)
        return INT_MIN;
    duration_samples = window->end_sample - window->start_sample;
    if (duration_samples < 0)
        duration_samples = 0;
    peak_term = (int) ((window->peak_db + 100.0) * 10.0);
    return duration_samples + (window->active_hits * 400) + peak_term;
}

static void v34_phase2_insert_candidate_window(v34_phase2_candidate_window_t *windows,
                                               int *count,
                                               int max_windows,
                                               const v34_phase2_candidate_window_t *candidate)
{
    int insert_at;

    if (!windows || !count || !candidate || max_windows <= 0)
        return;

    insert_at = *count;
    if (insert_at > max_windows)
        insert_at = max_windows;

    while (insert_at > 0
           && v34_phase2_candidate_rank(candidate) > v34_phase2_candidate_rank(&windows[insert_at - 1])) {
        if (insert_at < max_windows)
            windows[insert_at] = windows[insert_at - 1];
        insert_at--;
    }

    if (insert_at >= max_windows)
        return;

    windows[insert_at] = *candidate;
    if (*count < max_windows)
        (*count)++;
}

static int v34_phase2_find_candidate_windows(const int16_t *samples,
                                             int total_samples,
                                             int sample_rate,
                                             v34_phase2_candidate_window_t *windows,
                                             int max_windows)
{
    enum {
        WINDOW_MS = 100,
        STEP_MS = 50,
        PAD_MS = 300,
        GAP_MS = 200
    };
    const double activity_floor_db = -58.0;
    int window;
    int step;
    int pad;
    int gap_limit;
    double max_db = -100.0;
    bool in_region = false;
    int region_start = -1;
    int region_end = -1;
    int region_last_active_end = -1;
    int region_active_hits = 0;
    double region_peak_db = -100.0;
    int found = 0;
    int scan_limit;

    if (!windows || max_windows <= 0)
        return 0;
    memset(windows, 0, sizeof(*windows) * (size_t) max_windows);
    if (!samples || total_samples <= 0 || sample_rate <= 0)
        return 0;

    window = (sample_rate * WINDOW_MS) / 1000;
    step = (sample_rate * STEP_MS) / 1000;
    pad = (sample_rate * PAD_MS) / 1000;
    gap_limit = (sample_rate * GAP_MS) / 1000;
    if (window <= 0 || step <= 0)
        return 0;

    for (int start = 0; start < total_samples; start += step) {
        int len = total_samples - start;
        double db;

        if (len > window)
            len = window;
        db = v34_phase2_rms_energy_db(samples + start, len);
        if (db > max_db)
            max_db = db;
    }

    if (max_db <= activity_floor_db)
        return 0;

    scan_limit = total_samples + gap_limit + step;
    for (int start = 0; start < scan_limit; start += step) {
        int len = total_samples - start;
        double db = -100.0;
        bool active;

        if (start < total_samples) {
            if (len > window)
                len = window;
            db = v34_phase2_rms_energy_db(samples + start, len);
        } else {
            len = 0;
        }
        active = (db >= activity_floor_db);

        if (active) {
            if (!in_region) {
                in_region = true;
                region_start = start;
                region_end = start + len;
                region_last_active_end = start + len;
                region_active_hits = 1;
                region_peak_db = db;
            } else {
                region_end = start + len;
                region_last_active_end = start + len;
                region_active_hits++;
                if (db > region_peak_db)
                    region_peak_db = db;
            }
            continue;
        }

        if (!in_region)
            continue;
        if (start < region_last_active_end + gap_limit)
            continue;

        if (region_end > region_start) {
            v34_phase2_candidate_window_t candidate;

            candidate.start_sample = region_start - pad;
            candidate.end_sample = region_end + pad;
            if (candidate.start_sample < 0)
                candidate.start_sample = 0;
            if (candidate.end_sample > total_samples)
                candidate.end_sample = total_samples;
            candidate.active_hits = region_active_hits;
            candidate.peak_db = region_peak_db;
            if (candidate.end_sample > candidate.start_sample)
                v34_phase2_insert_candidate_window(windows, &found, max_windows, &candidate);
        }

        in_region = false;
        region_start = -1;
        region_end = -1;
        region_last_active_end = -1;
        region_active_hits = 0;
        region_peak_db = -100.0;
    }

    return found;
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

static int v34_phase2_result_quality(const decode_v34_result_t *result, bool have_result)
{
    int score;

    if (!have_result || !result)
        return INT_MIN;

    score = v34_phase2_result_spec_score(result);
    if (result->info0_seen && result->info1_seen)
        score += 500;
    else if (result->info0_seen || result->info1_seen)
        score += 120;
    if (result->phase3_seen)
        score += 300;
    if (result->phase4_seen)
        score += 500;
    if (result->training_failed && !result->info0_seen && !result->info1_seen)
        score -= 200;
    if (result->info0_sample >= 0)
        score -= result->info0_sample / 4000;
    return score;
}

static bool v34_phase2_decode_candidate_with_pass(v34_phase2_pass_fn_t pass_fn,
                                                  void *pass_ctx,
                                                  const int16_t *samples,
                                                  int total_samples,
                                                  const v34_phase2_candidate_window_t *window,
                                                  int candidate_index,
                                                  v91_law_t law,
                                                  bool calling_party,
                                                  bool allow_info_rate_infer,
                                                  decode_v34_result_t *result_out)
{
    static const float rescue_cutoffs[] = { -60.0f, -68.0f };
    decode_v34_result_t candidate;
    bool have_candidate = false;
    int decode_start;
    int decode_end;

    if (!pass_fn || !samples || !window || !result_out)
        return false;
    decode_start = window->start_sample;
    decode_end = window->end_sample;
    if (decode_start < 0)
        decode_start = 0;
    if (decode_end > total_samples)
        decode_end = total_samples;
    if (decode_end <= decode_start)
        return false;

    have_candidate = pass_fn(pass_ctx,
                             samples + decode_start,
                             decode_end - decode_start,
                             law,
                             calling_party,
                             -52.0f,
                             allow_info_rate_infer,
                             &candidate);
    if (!have_candidate)
        return false;
    v34_phase2_offset_result_samples(&candidate, decode_start);

    if (!candidate.info0_seen || !candidate.info1_seen) {
        for (size_t i = 0; i < sizeof(rescue_cutoffs)/sizeof(rescue_cutoffs[0]); i++) {
            decode_v34_result_t rescue;

            if (!pass_fn(pass_ctx,
                         samples + decode_start,
                         decode_end - decode_start,
                         law,
                         calling_party,
                         rescue_cutoffs[i],
                         allow_info_rate_infer,
                         &rescue)) {
                continue;
            }
            v34_phase2_offset_result_samples(&rescue, decode_start);
            merge_info_event_diagnostics_from_rescue(&candidate, &rescue);
            if (!candidate.info0_seen && rescue.info0_seen)
                merge_info0_from_rescue(&candidate, &rescue);
            if (!candidate.info1_seen && rescue.info1_seen)
                merge_info1_from_rescue(&candidate, &rescue);
            if (candidate.info0_seen && candidate.info1_seen)
                break;
        }
    }

    candidate.phase2_selected_window_index = candidate_index;
    candidate.phase2_selected_window_start_sample = decode_start;
    candidate.phase2_selected_window_end_sample = decode_end;
    candidate.phase2_selected_window_active_hits = window->active_hits;
    candidate.phase2_selected_window_peak_db_tenths = (int) (window->peak_db * 10.0);
    *result_out = candidate;
    return true;
}

static bool v34_phase2_try_candidate_side(v34_phase2_engine_t *engine,
                                          const int16_t *samples,
                                          int total_samples,
                                          const v34_phase2_candidate_window_t *window,
                                          int candidate_index,
                                          v91_law_t law,
                                          bool calling_party,
                                          bool allow_info_rate_infer,
                                          decode_v34_result_t *best_result,
                                          bool *have_best)
{
    decode_v34_result_t candidate;
    v34_phase2_pass_fn_t pass_fn;
    void *pass_ctx;

    if (!engine || !samples || !window || !best_result || !have_best)
        return false;
    pass_fn = engine->decode_phase2_pass ? engine->decode_phase2_pass : engine->decode_pass;
    pass_ctx = engine->decode_phase2_pass ? engine->decode_phase2_pass_ctx : engine->decode_pass_ctx;
    if (!v34_phase2_decode_candidate_with_pass(pass_fn,
                                               pass_ctx,
                                               samples,
                                               total_samples,
                                               window,
                                               candidate_index,
                                               law,
                                               calling_party,
                                               allow_info_rate_infer,
                                               &candidate)) {
        return false;
    }
    if (!*have_best
        || v34_phase2_result_quality(&candidate, true) > v34_phase2_result_quality(best_result, true)) {
        *best_result = candidate;
        *have_best = true;
    }
    return true;
}

static void v34_phase2_finalize_selected_side(v34_phase2_engine_t *engine,
                                              const int16_t *samples,
                                              int total_samples,
                                              v91_law_t law,
                                              bool calling_party,
                                              bool allow_info_rate_infer,
                                              decode_v34_result_t *result,
                                              bool *have_result)
{
    decode_v34_result_t full_result;
    v34_phase2_candidate_window_t window;

    if (!engine || !engine->decode_pass || !result || !have_result || !*have_result)
        return;
    if (!engine->decode_phase2_pass
        || (engine->decode_phase2_pass == engine->decode_pass
            && engine->decode_phase2_pass_ctx == engine->decode_pass_ctx)) {
        return;
    }

    window.start_sample = result->phase2_selected_window_start_sample;
    window.end_sample = result->phase2_selected_window_end_sample;
    window.active_hits = result->phase2_selected_window_active_hits;
    window.peak_db = (double) result->phase2_selected_window_peak_db_tenths / 10.0;
    if (!v34_phase2_decode_candidate_with_pass(engine->decode_pass,
                                               engine->decode_pass_ctx,
                                               samples,
                                               total_samples,
                                               &window,
                                               result->phase2_selected_window_index,
                                               law,
                                               calling_party,
                                               allow_info_rate_infer,
                                               &full_result)) {
        return;
    }

    if (!full_result.info0_seen && result->info0_seen)
        merge_info0_from_rescue(&full_result, result);
    if (!full_result.info1_seen && result->info1_seen)
        merge_info1_from_rescue(&full_result, result);
    merge_info_event_diagnostics_from_rescue(&full_result, result);
    full_result.phase2_candidate_windows_seen = result->phase2_candidate_windows_seen;
    full_result.phase2_candidate_windows_tried = result->phase2_candidate_windows_tried;
    *result = full_result;
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
    engine->decode_phase2_pass = decode_pass;
    engine->decode_phase2_pass_ctx = decode_pass_ctx;
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
    v34_phase2_candidate_window_t windows[V34_PHASE2_MAX_CANDIDATE_WINDOWS];
    int window_count;
    int answerer_windows_tried = 0;
    int caller_windows_tried = 0;

    if (have_answerer)
        *have_answerer = false;
    if (have_caller)
        *have_caller = false;
    if (!engine || !engine->decode_pass
        || !samples || total_samples <= 0
        || !answerer || !caller || !have_answerer || !have_caller) {
        return;
    }

    window_count = v34_phase2_find_candidate_windows(samples,
                                                     total_samples,
                                                     8000,
                                                     windows,
                                                     V34_PHASE2_MAX_CANDIDATE_WINDOWS);
    if (window_count <= 0) {
        windows[0].start_sample = 0;
        windows[0].end_sample = total_samples;
        windows[0].active_hits = 0;
        windows[0].peak_db = -100.0;
        window_count = 1;
    }

    for (int i = 0; i < window_count; i++) {
        if (v34_phase2_try_candidate_side(engine,
                                          samples,
                                          total_samples,
                                          &windows[i],
                                          i,
                                          law,
                                          false,
                                          allow_info_rate_infer,
                                          answerer,
                                          have_answerer)) {
            answerer_windows_tried++;
        }
        if (v34_phase2_try_candidate_side(engine,
                                          samples,
                                          total_samples,
                                          &windows[i],
                                          i,
                                          law,
                                          true,
                                          allow_info_rate_infer,
                                          caller,
                                          have_caller)) {
            caller_windows_tried++;
        }

        if (*have_answerer && *have_caller
            && answerer->info0_seen && answerer->info1_seen
            && caller->info0_seen && caller->info1_seen
            && (answerer->phase3_seen || caller->phase3_seen)) {
            break;
        }
    }

    if (*have_answerer) {
        answerer->phase2_candidate_windows_seen = window_count;
        answerer->phase2_candidate_windows_tried = answerer_windows_tried;
        v34_phase2_finalize_selected_side(engine,
                                          samples,
                                          total_samples,
                                          law,
                                          false,
                                          allow_info_rate_infer,
                                          answerer,
                                          have_answerer);
    }
    if (*have_caller) {
        caller->phase2_candidate_windows_seen = window_count;
        caller->phase2_candidate_windows_tried = caller_windows_tried;
        v34_phase2_finalize_selected_side(engine,
                                          samples,
                                          total_samples,
                                          law,
                                          true,
                                          allow_info_rate_infer,
                                          caller,
                                          have_caller);
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
