/*
 * v92_short_phase1_decode.c — V.92 short Phase 1 helper decoder
 */

#include "v92_short_phase1_decode.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static int v92_uqts_from_wxyz(int wxyz)
{
    static const int table[16] = {
        61, 62, 63, 66,
        67, 70, 71, 74,
        75, 78, 79, 82,
        83, 86, 87, 87
    };

    if (wxyz < 0 || wxyz > 15)
        return -1;
    return table[wxyz];
}

static uint16_t pack_lsb_bits(const uint8_t *bits, int start, int count)
{
    uint16_t packed = 0;

    if (!bits || start < 0 || count <= 0 || count > 16)
        return 0;
    for (int i = 0; i < count; i++)
        packed |= (uint16_t) (bits[start + i] & 1) << i;
    return packed;
}

static bool v92_frame_matches_channel_bits(uint16_t frame_bits, bool use_ch2)
{
    bool qca = ((frame_bits >> 2) & 1U) != 0;

    return use_ch2 ? qca : !qca;
}

static bool v92_unpack_frame_bits(uint16_t frame_bits,
                                  v92_short_phase1_candidate_t *out)
{
    bool digital_modem;
    bool qca;
    bool lapm;

    if (!out)
        return false;
    if ((frame_bits & 0x001U) != 0)
        return false;

    memset(out, 0, sizeof(*out));
    digital_modem = ((frame_bits >> 1) & 1U) != 0;
    qca = ((frame_bits >> 2) & 1U) != 0;
    lapm = ((frame_bits >> 3) & 1U) != 0;

    out->digital_modem = digital_modem;
    out->qca = qca;
    out->lapm = lapm;

    if (digital_modem) {
        int lm;

        if (((frame_bits >> 4) & 0x7U) != 0 || ((frame_bits >> 9) & 1U) != 1)
            return false;
        lm = (int) ((frame_bits >> 7) & 0x3U);
        out->name = qca ? "QCA1d" : "QC1d";
        out->aux_value = lm;
        out->uqts_ucode = -1;
    } else {
        int wxyz;

        if (((frame_bits >> 5) & 1U) != 0 || ((frame_bits >> 9) & 1U) != 1)
            return false;
        wxyz = (int) ((((frame_bits >> 4) & 1U) << 3)
                    | (((frame_bits >> 6) & 1U) << 2)
                    | (((frame_bits >> 7) & 1U) << 1)
                    | ((frame_bits >> 8) & 1U));
        out->name = qca ? "QCA1a" : "QC1a";
        out->aux_value = wxyz;
        out->uqts_ucode = v92_uqts_from_wxyz(wxyz);
    }
    out->ok = true;
    return true;
}

static bool v92_same_decoded_payload(const v92_short_phase1_candidate_t *a,
                                     const v92_short_phase1_candidate_t *b)
{
    if (!a || !b || !a->ok || !b->ok)
        return false;
    return a->digital_modem == b->digital_modem
        && a->qca == b->qca
        && a->lapm == b->lapm
        && a->aux_value == b->aux_value;
}

bool v92_decode_short_phase1_candidate(const uint8_t *bits,
                                       int bit_len,
                                       bool use_ch2,
                                       v92_short_phase1_candidate_t *out)
{
    v92_short_phase1_candidate_t first = {0};
    v92_short_phase1_candidate_t repeat = {0};
    bool first_ok;
    bool repeat_ok = false;
    int chosen_index = -1;

    if (!bits || bit_len < 30 || !out)
        return false;

    memset(out, 0, sizeof(*out));
    out->frame1_bits = pack_lsb_bits(bits, 20, 10);
    first_ok = v92_unpack_frame_bits(out->frame1_bits, &first);
    if (bit_len >= 60) {
        out->frame2_bits = pack_lsb_bits(bits, 50, 10);
        out->repeat_seen = true;
        repeat_ok = v92_unpack_frame_bits(out->frame2_bits, &repeat);
        out->repeat_match = first_ok && repeat_ok && v92_same_decoded_payload(&first, &repeat);
    }

    if (repeat_ok && v92_frame_matches_channel_bits(out->frame2_bits, use_ch2)) {
        *out = repeat;
        out->frame1_bits = pack_lsb_bits(bits, 20, 10);
        if (bit_len >= 60) {
            out->frame2_bits = pack_lsb_bits(bits, 50, 10);
            out->repeat_seen = true;
            out->repeat_match = first_ok && repeat_ok && v92_same_decoded_payload(&first, &repeat);
        }
        chosen_index = 1;
    } else if (first_ok && v92_frame_matches_channel_bits(out->frame1_bits, use_ch2)) {
        *out = first;
        out->frame1_bits = pack_lsb_bits(bits, 20, 10);
        if (bit_len >= 60) {
            out->frame2_bits = pack_lsb_bits(bits, 50, 10);
            out->repeat_seen = true;
            out->repeat_match = first_ok && repeat_ok && v92_same_decoded_payload(&first, &repeat);
        }
        chosen_index = 0;
    } else if (repeat_ok) {
        *out = repeat;
        out->frame1_bits = pack_lsb_bits(bits, 20, 10);
        out->frame2_bits = pack_lsb_bits(bits, 50, 10);
        out->repeat_seen = bit_len >= 60;
        out->repeat_match = first_ok && repeat_ok && v92_same_decoded_payload(&first, &repeat);
        chosen_index = 1;
    } else if (first_ok) {
        *out = first;
        out->frame1_bits = pack_lsb_bits(bits, 20, 10);
        if (bit_len >= 60) {
            out->frame2_bits = pack_lsb_bits(bits, 50, 10);
            out->repeat_seen = true;
            out->repeat_match = first_ok && repeat_ok && v92_same_decoded_payload(&first, &repeat);
        }
        chosen_index = 0;
    } else {
        return false;
    }

    out->decoded_frame_index = chosen_index;
    out->decoded_frame_bits = (chosen_index > 0) ? out->frame2_bits : out->frame1_bits;
    out->ok = true;
    return true;
}

const char *v92_anspcm_level_to_str(int level)
{
    switch (level & 0x03) {
    case 0: return "-9.5 dBm0";
    case 1: return "-12 dBm0";
    case 2: return "-15 dBm0";
    case 3: return "-18 dBm0";
    default: return "unknown";
    }
}

static bool v92_matches_pcm_symbol(v91_law_t law,
                                   uint8_t codeword,
                                   int target_ucode,
                                   bool positive,
                                   v92_codeword_to_ucode_fn codeword_to_ucode)
{
    int ucode;
    bool sign_positive;
    int tolerance;

    if (!codeword_to_ucode)
        return false;
    ucode = codeword_to_ucode(law, codeword);
    if (ucode < 0)
        return false;
    sign_positive = (codeword & 0x80) != 0;
    if (sign_positive != positive)
        return false;
    /* Reconstructed G.711 from analog captures is often coarsened by
     * padding, compander-law ambiguity, and effective level collapse.
     * Be noticeably more tolerant than exact-codeword matching. */
    tolerance = (target_ucode == 0) ? 3 : 6;
    return abs(ucode - target_ucode) <= tolerance;
}

static int v92_anspcm_scl_for_level(v91_law_t law, int level)
{
    static const int ulaw_scl[4] = { 1334, 1000, 708, 500 };
    static const int alaw_scl[4] = { 667, 500, 354, 250 };
    const int *table = (law == V91_LAW_ALAW) ? alaw_scl : ulaw_scl;

    if (level < 0 || level > 3)
        return -1;
    return table[level];
}

static void v92_generate_anspcm_period(v91_law_t law,
                                       int level,
                                       int16_t *linear_out,
                                       uint8_t *codeword_out)
{
    const double theta = 0.25 * M_PI / 301.0;
    int scl = v92_anspcm_scl_for_level(law, level);

    if (!linear_out || !codeword_out || scl < 0)
        return;

    for (int k = 0; k < 301; k++) {
        double x = floor((double) scl * M_SQRT2
                         * cos((2.0 * M_PI * (double) k * 79.0 / 301.0) + theta)
                         + 0.5);
        int16_t sample = (int16_t) x;

        codeword_out[k] = v91_linear_to_codeword(law, sample);
        linear_out[k] = v91_codeword_to_linear(law, codeword_out[k]);
    }
}

static int v92_score_pcm_symbol(v91_law_t law,
                                uint8_t codeword,
                                int target_ucode,
                                bool positive,
                                v92_codeword_to_ucode_fn codeword_to_ucode)
{
    int ucode;
    bool sign_positive;
    int diff;
    int score;

    if (!codeword_to_ucode)
        return -100;
    ucode = codeword_to_ucode(law, codeword);
    if (ucode < 0)
        return -100;

    sign_positive = (codeword & 0x80) != 0;
    diff = abs(ucode - target_ucode);
    if (target_ucode == 0)
        score = 18 - (diff * 4);
    else
        score = 30 - (diff * 3);
    if (sign_positive != positive)
        score -= 18;
    return score;
}

static int v92_score_qts_block(v91_law_t law,
                               const uint8_t *codewords,
                               int pos,
                               int uqts_ucode,
                               bool qts_bar,
                               v92_codeword_to_ucode_fn codeword_to_ucode,
                               int *weak_symbols_out)
{
    static const int target_is_zero[6] = { 0, 1, 0, 0, 1, 0 };
    static const bool qts_positive[6] = { true, true, true, false, false, false };
    static const bool qts_bar_positive[6] = { false, false, false, true, true, true };
    const bool *signs = qts_bar ? qts_bar_positive : qts_positive;
    int weak_symbols = 0;
    int total_score = 0;

    for (int i = 0; i < 6; i++) {
        int target_ucode = target_is_zero[i] ? 0 : uqts_ucode;
        int sym_score = v92_score_pcm_symbol(law,
                                             codewords[pos + i],
                                             target_ucode,
                                             signs[i],
                                             codeword_to_ucode);

        total_score += sym_score;
        if (sym_score < 10)
            weak_symbols++;
    }

    if (weak_symbols_out)
        *weak_symbols_out = weak_symbols;
    return total_score;
}

bool v92_detect_qts_sequence(const uint8_t *codewords,
                             int total,
                             v91_law_t law,
                             int uqts_ucode,
                             int search_start,
                             int search_end,
                             v92_codeword_to_ucode_fn codeword_to_ucode,
                             v92_qts_hit_t *out)
{
    v92_qts_hit_t best = {0};
    int best_score = -1000000;

    if (!codewords || total <= 0 || uqts_ucode < 0 || !codeword_to_ucode || !out)
        return false;

    if (search_start < 0)
        search_start = 0;
    if (search_end <= 0 || search_end > total)
        search_end = total;
    if (search_end - search_start < 6 * 12)
        return false;

    for (int offset = search_start; offset + 6 * 12 <= search_end; offset++) {
        int qts_reps = 0;
        int qts_bar_reps = 0;
        int total_score = 0;
        int symbol_count = 0;
        int pos = offset;
        int weak_symbols = 0;
        int block_score;

        if (!v92_matches_pcm_symbol(law, codewords[offset], uqts_ucode, true, codeword_to_ucode))
            continue;
        block_score = v92_score_qts_block(law,
                                          codewords,
                                          offset,
                                          uqts_ucode,
                                          false,
                                          codeword_to_ucode,
                                          &weak_symbols);
        if (block_score < 100 || weak_symbols > 1)
            continue;

        while (pos + 6 <= search_end) {
            block_score = v92_score_qts_block(law,
                                              codewords,
                                              pos,
                                              uqts_ucode,
                                              false,
                                              codeword_to_ucode,
                                              &weak_symbols);
            if (block_score < 90 || weak_symbols > 2)
                break;
            qts_reps++;
            total_score += block_score;
            symbol_count += 6;
            pos += 6;
        }
        if (qts_reps < 8)
            continue;

        while (pos + 6 <= search_end) {
            block_score = v92_score_qts_block(law,
                                              codewords,
                                              pos,
                                              uqts_ucode,
                                              true,
                                              codeword_to_ucode,
                                              &weak_symbols);
            if (block_score < 90 || weak_symbols > 2)
                break;
            qts_bar_reps++;
            total_score += block_score;
            symbol_count += 6;
            pos += 6;
        }

        total_score += qts_reps * 24 + qts_bar_reps * 12;

        if (!best.seen
            || total_score > best_score
            || (total_score == best_score
                && (qts_reps > best.qts_reps
                    || (qts_reps == best.qts_reps && qts_bar_reps > best.qts_bar_reps)))) {
            best.seen = true;
            best.start_sample = offset;
            best.qts_reps = qts_reps;
            best.qts_bar_reps = qts_bar_reps;
            best.alignment_phase = offset % 6;
            best.symbol_count = symbol_count;
            best.score = total_score;
            best_score = total_score;
        }
    }

    if (!best.seen)
        return false;
    *out = best;
    return true;
}

bool v92_detect_anspcm_sequence(const uint8_t *codewords,
                                int total,
                                v91_law_t law,
                                int lm_level,
                                int alignment_phase,
                                int search_start,
                                int search_end,
                                v92_anspcm_hit_t *out)
{
    static const int min_periods = 2;
    static const int eval_periods = 4;
    int16_t expected_linear[301];
    uint8_t expected_codewords[301];
    v92_anspcm_hit_t best = {0};
    int best_score = -1000000;
    int min_symbols = 301 * min_periods;
    int eval_symbols = 301 * eval_periods;

    if (!codewords || total <= 0 || !out)
        return false;
    if (lm_level < 0 || lm_level > 3)
        return false;

    v92_generate_anspcm_period(law, lm_level, expected_linear, expected_codewords);

    if (search_start < 0)
        search_start = 0;
    if (search_end <= 0 || search_end > total)
        search_end = total;
    if (search_end - search_start < min_symbols)
        return false;

    for (int offset = search_start; offset + min_symbols <= search_end; offset++) {
        int duration_symbols = 0;
        int sum_abs_error = 0;
        int sum_score = 0;
        int symbols_to_check;

        if (alignment_phase >= 0 && (offset % 6) != alignment_phase)
            continue;

        symbols_to_check = search_end - offset;
        if (symbols_to_check > eval_symbols)
            symbols_to_check = eval_symbols;

        for (int i = 0; i < symbols_to_check; i++) {
            int actual = v91_codeword_to_linear(law, codewords[offset + i]);
            int expected = expected_linear[i % 301];
            int phase_reversal_block = i / 3612;
            int diff;

            if (phase_reversal_block & 1)
                expected = -expected;
            diff = abs(actual - expected);

            if (diff <= 96)
                sum_score += 20;
            else if (diff <= 192)
                sum_score += 10;
            else if (diff <= 320)
                sum_score += 2;
            else
                sum_score -= 14;

            if ((actual >= 0) != (expected >= 0))
                sum_score -= 10;

            sum_abs_error += diff;
            duration_symbols++;

            if (duration_symbols >= min_symbols
                && duration_symbols % 301 == 0
                && (sum_score / duration_symbols) < 5) {
                duration_symbols -= 301;
                break;
            }
        }

        if (duration_symbols < min_symbols)
            continue;

        {
            int avg_abs_error = sum_abs_error / duration_symbols;
            int normalized_score = sum_score - (avg_abs_error / 8);

            if (!best.seen || normalized_score > best_score
                || (normalized_score == best_score && duration_symbols > best.duration_symbols)) {
                best.seen = true;
                best.start_sample = offset;
                best.duration_symbols = duration_symbols;
                best.level = lm_level;
                best.score = normalized_score;
                best.avg_abs_error = avg_abs_error;
                best_score = normalized_score;
            }
        }
    }

    if (!best.seen)
        return false;

    *out = best;
    return true;
}
