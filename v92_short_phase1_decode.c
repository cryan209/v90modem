/*
 * v92_short_phase1_decode.c — V.92 short Phase 1 helper decoder
 */

#include "v92_short_phase1_decode.h"

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
        int pos = offset;

        if (!v92_matches_pcm_symbol(law, codewords[offset], uqts_ucode, true, codeword_to_ucode))
            continue;

        while (pos + 6 <= search_end) {
            bool ok = true;
            ok &= v92_matches_pcm_symbol(law, codewords[pos + 0], uqts_ucode, true, codeword_to_ucode);
            ok &= v92_matches_pcm_symbol(law, codewords[pos + 1], 0, true, codeword_to_ucode);
            ok &= v92_matches_pcm_symbol(law, codewords[pos + 2], uqts_ucode, true, codeword_to_ucode);
            ok &= v92_matches_pcm_symbol(law, codewords[pos + 3], uqts_ucode, false, codeword_to_ucode);
            ok &= v92_matches_pcm_symbol(law, codewords[pos + 4], 0, false, codeword_to_ucode);
            ok &= v92_matches_pcm_symbol(law, codewords[pos + 5], uqts_ucode, false, codeword_to_ucode);
            if (!ok)
                break;
            qts_reps++;
            pos += 6;
        }
        if (qts_reps < 8)
            continue;

        while (pos + 6 <= search_end) {
            bool ok = true;
            ok &= v92_matches_pcm_symbol(law, codewords[pos + 0], uqts_ucode, false, codeword_to_ucode);
            ok &= v92_matches_pcm_symbol(law, codewords[pos + 1], 0, false, codeword_to_ucode);
            ok &= v92_matches_pcm_symbol(law, codewords[pos + 2], uqts_ucode, false, codeword_to_ucode);
            ok &= v92_matches_pcm_symbol(law, codewords[pos + 3], uqts_ucode, true, codeword_to_ucode);
            ok &= v92_matches_pcm_symbol(law, codewords[pos + 4], 0, true, codeword_to_ucode);
            ok &= v92_matches_pcm_symbol(law, codewords[pos + 5], uqts_ucode, true, codeword_to_ucode);
            if (!ok)
                break;
            qts_bar_reps++;
            pos += 6;
        }

        if (!best.seen
            || qts_reps > best.qts_reps
            || (qts_reps == best.qts_reps && qts_bar_reps > best.qts_bar_reps)) {
            best.seen = true;
            best.start_sample = offset;
            best.qts_reps = qts_reps;
            best.qts_bar_reps = qts_bar_reps;
        }
    }

    if (!best.seen)
        return false;
    *out = best;
    return true;
}
