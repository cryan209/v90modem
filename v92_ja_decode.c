/*
 * v92_ja_decode.c — V.92 Phase 3 Ja / DIL descriptor decoder
 *
 * Core implementation of v92_ja_dil_search().  The bit-extraction helpers
 * (V.90 differential sign + x^23+x^5+1 descrambler) live here as statics so
 * this module has no dependency on vpcm_decode internals.
 */

#include "v92_ja_decode.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* -------------------------------------------------------------------------
 * V.90 scrambler constants (ITU-T V.90 §5.4.4)
 * ------------------------------------------------------------------------- */

/* Length of the scrambler shift register (polynomial degree). */
#define JA_SCRAMBLER_HISTORY   23

/* Minimum bits of lookahead needed to hold the smallest valid DIL descriptor. */
#define JA_MIN_DESCRIPTOR_BITS 206
#define JA_FRAME17_CHECK_BITS  96

static int ja_get_packed_bit(const uint8_t *bits, int pos)
{
    return (int) ((bits[pos / 8] >> (pos % 8)) & 1U);
}

static int ja_get_packed_bits(const uint8_t *bits, int pos, int n)
{
    int i;
    int v = 0;

    for (i = 0; i < n; i++) {
        v |= ja_get_packed_bit(bits, pos + i) << i;
    }
    return v;
}

static uint16_t ja_crc16_bits(const uint8_t *bits, int bit_count)
{
    uint16_t crc = 0xFFFFu;
    int i;

    for (i = 0; i < bit_count; i++) {
        int b = ja_get_packed_bit(bits, i);
        int fb = ((int) (crc >> 15) ^ b) & 1;

        crc = (uint16_t) (crc << 1);
        if (fb)
            crc ^= 0x8005u;
    }
    return crc;
}

static int ja_sync_hamming18(const uint8_t *bits, int bit_count)
{
    int i;
    int hd = 0;

    if (!bits || bit_count < 18)
        return 18;
    for (i = 0; i < 17; i++) {
        if (ja_get_packed_bit(bits, i) == 0)
            hd++;
    }
    if (ja_get_packed_bit(bits, 17) != 0)
        hd++;
    return hd;
}

static int ja_frame17_zero_violations(const uint8_t *bits, int bit_count)
{
    int k;
    int viol = 0;

    if (!bits || bit_count <= 51)
        return JA_FRAME17_CHECK_BITS;
    for (k = 0; k < JA_FRAME17_CHECK_BITS; k++) {
        int p = 51 + 17 * k;

        if (p >= bit_count)
            break;
        if (ja_get_packed_bit(bits, p) != 0)
            viol++;
    }
    return viol;
}

static int ja_frame17_best_zero_run(const uint8_t *bits, int bit_count)
{
    int k;
    int best = 0;
    int run = 0;

    if (!bits || bit_count <= 51)
        return 0;
    for (k = 0; k < JA_FRAME17_CHECK_BITS; k++) {
        int p = 51 + 17 * k;

        if (p >= bit_count)
            break;
        if (ja_get_packed_bit(bits, p) == 0) {
            run++;
            if (run > best)
                best = run;
        } else {
            run = 0;
        }
    }
    return best;
}

static int ja_fs12_position(const uint8_t *bits, int bit_count)
{
    static const int fs12[12] = {1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 1, 0};
    int i;

    if (!bits || bit_count < 12)
        return -1;
    for (i = 0; i <= bit_count - 12; i++) {
        int j;
        bool ok = true;

        for (j = 0; j < 12; j++) {
            if (ja_get_packed_bit(bits, i + j) != fs12[j]) {
                ok = false;
                break;
            }
        }
        if (ok)
            return i;
    }
    return -1;
}

static int ja_reserved_zero_violations(const uint8_t *bits, int bit_count)
{
    static const int fixed_pos[] = {17, 34, 42, 50};
    int v = 0;
    int i;

    if (!bits || bit_count <= 0)
        return 12;
    for (i = 0; i < 8; i++) {
        int p = 26 + i;
        if (p < bit_count && ja_get_packed_bit(bits, p) != 0)
            v++;
    }
    for (i = 0; i < (int) (sizeof(fixed_pos) / sizeof(fixed_pos[0])); i++) {
        int p = fixed_pos[i];
        if (p < bit_count && ja_get_packed_bit(bits, p) != 0)
            v++;
    }
    return v;
}

static int ja_crc_nearness_hd(const uint8_t *bits, int bit_count)
{
    int n, lsp, ltp;
    int alpha, beta;
    int training_start, training_bits;
    int crc_start;
    uint16_t exp_crc;
    uint16_t calc_crc;
    uint16_t d;
    int hd = 0;

    if (!bits || bit_count < 64)
        return 16;
    n = ja_get_packed_bits(bits, 18, 8);
    lsp = ja_get_packed_bits(bits, 35, 7) + 1;
    ltp = ja_get_packed_bits(bits, 43, 7) + 1;
    if (lsp < 1) lsp = 1;
    if (ltp < 1) ltp = 1;
    if (lsp > 128) lsp = 128;
    if (ltp > 128) ltp = 128;
    if (n < 0) n = 0;
    if (n > 255) n = 255;

    alpha = ((int) lsp + 15) / 16 * 17;
    beta = alpha + (((int) ltp + 15) / 16) * 17;
    training_start = 187 + beta;
    training_bits = (((int) n + 1) / 2) * 17;
    crc_start = training_start + training_bits;
    if (crc_start + 17 > bit_count)
        return 16;

    exp_crc = (uint16_t) ja_get_packed_bits(bits, crc_start + 1, 16);
    calc_crc = ja_crc16_bits(bits, crc_start);
    d = (uint16_t) (exp_crc ^ calc_crc);
    while (d) {
        d &= (uint16_t) (d - 1U);
        hd++;
    }
    return hd;
}

static int ja_soft_score(const uint8_t *bits, int bit_count)
{
    int sync_hd = ja_sync_hamming18(bits, bit_count);
    int frame17 = ja_frame17_zero_violations(bits, bit_count);
    int frame17_run = ja_frame17_best_zero_run(bits, bit_count);
    int zviol = ja_reserved_zero_violations(bits, bit_count);
    int crc_hd = ja_crc_nearness_hd(bits, bit_count);
    int fspos = ja_fs12_position(bits, bit_count);
    int score = 500;

    score -= sync_hd * 24;
    score -= frame17 * 10;
    score += frame17_run * 5;
    score -= zviol * 10;
    score -= crc_hd * 8;
    if (fspos >= 0) {
        int fs_err = fspos - 4;
        if (fs_err < 0)
            fs_err = -fs_err;
        score += 40;
        score -= fs_err * 2;
        if (fspos <= 8)
            score += 20;
    } else {
        score -= 25;
    }
    return score;
}

static int ja_debug_enabled(void)
{
    const char *v = getenv("VPCM_JA_DEBUG");

    return (v && *v && *v != '0');
}

static bool v92_parse_table20_descriptor_strict(const uint8_t *bits,
                                                int bit_count,
                                                v90_dil_desc_t *out_desc,
                                                int *rate_mask_lo_out,
                                                int *rate_mask_hi_out,
                                                int *bit_len_out)
{
    int n, lsp, ltp;
    int alpha, beta;
    int training_start, training_bits;
    int crc_start;
    int v92_start2;
    int v92_start3;
    int v92_crc_pos;
    int v92_fill_pos;
    uint16_t exp_crc, calc_crc;
    int rate_lo;
    int rate_hi;
    uint8_t patched[512];
    int patched_len;
    uint16_t synthetic_v90_crc;
    v90_dil_desc_t desc;

    if (!bits || !out_desc || bit_count < JA_MIN_DESCRIPTOR_BITS)
        return false;

    for (int i = 0; i < 17; i++) {
        if (ja_get_packed_bit(bits, i) == 0)
            return false;
    }
    if (ja_get_packed_bit(bits, 17) != 0)
        return false;

    n = ja_get_packed_bits(bits, 18, 8);
    lsp = ja_get_packed_bits(bits, 35, 7) + 1;
    ltp = ja_get_packed_bits(bits, 43, 7) + 1;
    if (lsp < 1) lsp = 1;
    if (ltp < 1) ltp = 1;
    if (lsp > 128) lsp = 128;
    if (ltp > 128) ltp = 128;
    if (n < 0) n = 0;
    if (n > 255) n = 255;

    alpha = ((int) lsp + 15) / 16 * 17;
    beta = alpha + (((int) ltp + 15) / 16) * 17;
    training_start = 187 + beta;
    training_bits = (((int) n + 1) / 2) * 17;
    crc_start = training_start + training_bits;    /* V.90 CRC start-bit position */

    /*
     * Table 20 / V.92:
     *   [crc_start+1 .. +16]   : rate mask (24k..44k)
     *   [crc_start+17]         : start bit (0)
     *   [crc_start+18 .. +33]  : rate mask continuation (45.3k/46.6k/48k + reserved)
     *   [crc_start+34]         : start bit (0)
     *   [crc_start+35 .. +50]  : CRC
     *   [crc_start+51]         : fill bit (0)
     */
    v92_start2 = crc_start + 17;
    v92_start3 = crc_start + 34;
    v92_crc_pos = crc_start + 35;
    v92_fill_pos = crc_start + 51;
    if (v92_fill_pos >= bit_count)
        return false;

    if (ja_get_packed_bit(bits, crc_start) != 0)
        return false;
    if (ja_get_packed_bit(bits, v92_start2) != 0)
        return false;
    if (ja_get_packed_bit(bits, v92_start3) != 0)
        return false;
    if (ja_get_packed_bit(bits, v92_fill_pos) != 0)
        return false;

    rate_lo = ja_get_packed_bits(bits, crc_start + 1, 16);
    rate_hi = ja_get_packed_bits(bits, crc_start + 18, 16);
    for (int b = 3; b < 16; b++) {
        if (((rate_hi >> b) & 1) != 0)
            return false;
    }

    exp_crc = (uint16_t) ja_get_packed_bits(bits, v92_crc_pos, 16);
    calc_crc = ja_crc16_bits(bits, v92_start3);
    if (exp_crc != calc_crc)
        return false;

    {
        int p = v92_fill_pos + 1;

        while ((p % 12) != 0) {
            if (p >= bit_count || ja_get_packed_bit(bits, p) != 0)
                return false;
            p++;
        }
        if (bit_len_out)
            *bit_len_out = p;
    }

    patched_len = (bit_count + 7) / 8;
    if (patched_len <= 0 || patched_len > (int) sizeof(patched))
        return false;
    memcpy(patched, bits, (size_t) patched_len);

    /* Build synthetic V.90 CRC so we can reuse v90_parse_dil_descriptor. */
    synthetic_v90_crc = ja_crc16_bits(bits, crc_start);
    for (int i = 0; i < 16; i++) {
        int p = crc_start + 1 + i;
        uint8_t mask = (uint8_t) (1U << (p % 8));
        if ((synthetic_v90_crc >> i) & 1U)
            patched[p / 8] |= mask;
        else
            patched[p / 8] &= (uint8_t) ~mask;
    }
    {
        int p = crc_start + 17;
        patched[p / 8] &= (uint8_t) ~(1U << (p % 8));
    }

    memset(&desc, 0, sizeof(desc));
    if (!v90_parse_dil_descriptor(&desc, patched, bit_count))
        return false;

    *out_desc = desc;
    if (rate_mask_lo_out)
        *rate_mask_lo_out = rate_lo;
    if (rate_mask_hi_out)
        *rate_mask_hi_out = rate_hi;
    return true;
}

bool v92_parse_ja_descriptor_strict(v90_dil_desc_t *out_desc,
                                    const uint8_t *bits,
                                    int bit_count,
                                    v92_ja_parse_meta_t *meta)
{
    v90_dil_desc_t desc;
    v92_ja_parse_meta_t local_meta;
    int rate_lo = 0;
    int rate_hi = 0;
    int bit_len = 0;

    if (!out_desc || !bits || bit_count < JA_MIN_DESCRIPTOR_BITS)
        return false;

    memset(&desc, 0, sizeof(desc));
    memset(&local_meta, 0, sizeof(local_meta));

    if (v90_parse_dil_descriptor(&desc, bits, bit_count)) {
        local_meta.ok = true;
        local_meta.is_v92 = false;
        local_meta.bit_len = v90_dil_descriptor_bit_len(&desc);
        *out_desc = desc;
        if (meta)
            *meta = local_meta;
        return true;
    }

    if (!v92_parse_table20_descriptor_strict(bits,
                                             bit_count,
                                             &desc,
                                             &rate_lo,
                                             &rate_hi,
                                             &bit_len))
        return false;

    local_meta.ok = true;
    local_meta.is_v92 = true;
    local_meta.bit_len = bit_len;
    local_meta.rate_mask_lo = (uint16_t) rate_lo;
    local_meta.rate_mask_hi = (uint16_t) rate_hi;
    *out_desc = desc;
    if (meta)
        *meta = local_meta;
    return true;
}

/* -------------------------------------------------------------------------
 * Static bit-extraction helpers
 * ------------------------------------------------------------------------- */

/*
 * ja_descramble_reg_bit() — advance the V.90 x^23+x^5+1 descrambler by one
 * bit.
 *
 * @reg     Shift register state (caller-maintained across consecutive calls).
 * @in_bit  Next scrambled input bit (0 or 1).
 * Returns the descrambled output bit.
 */
static int ja_descramble_reg_bit(uint32_t *reg, int in_bit)
{
    int out_bit = (in_bit ^ (int) (*reg >> 22) ^ (int) (*reg >> 4)) & 1;
    *reg = (*reg << 1) | (uint32_t) in_bit;
    return out_bit;
}

/*
 * ja_decode_bits_packed() — extract and descramble a block of bits from the
 * V.90 PCM codeword stream into a packed byte array.
 *
 * The V.90 downstream channel uses differential sign coding followed by the
 * x^23+x^5+1 self-synchronising scrambler (V.90 §5.4.4).  This function
 * seeds the descrambler from the JA_SCRAMBLER_HISTORY samples that precede
 * @start_sample, then extracts @bit_count descrambled bits.
 *
 * @codewords       Raw μ-law (or A-law) PCM bytes; MSB is the sign bit.
 * @total_codewords Number of valid entries in @codewords.
 * @start_sample    Index of the first codeword to decode.
 * @bit_count       Number of bits to produce.
 * @invert_sign     When true, flip every sign bit before differential decode.
 * @packed_out      Output buffer; bit i written to byte i/8, bit i%8.
 * @packed_len      Byte length of @packed_out (must be ≥ ⌈bit_count/8⌉).
 *
 * Returns true on success, false when parameters are out of range.
 */
static bool ja_decode_bits_packed(const uint8_t *codewords,
                                  int total_codewords,
                                  int start_sample,
                                  int bit_count,
                                  bool invert_sign,
                                  uint8_t *packed_out,
                                  int packed_len)
{
    uint32_t descramble_reg;
    int prev_sign;
    int i;

    if (!codewords || !packed_out || packed_len <= 0 || bit_count <= 0
        || start_sample < (JA_SCRAMBLER_HISTORY + 1)
        || start_sample + bit_count > total_codewords
        || packed_len < ((bit_count + 7) / 8)) {
        return false;
    }

    memset(packed_out, 0, (size_t) packed_len);
    descramble_reg = 0;

    prev_sign = ((codewords[start_sample - JA_SCRAMBLER_HISTORY - 1] & 0x80) ? 1 : 0);
    if (invert_sign)
        prev_sign ^= 1;

    /* Seed the descrambler with the history window. */
    for (i = start_sample - JA_SCRAMBLER_HISTORY; i < start_sample; i++) {
        int sign = (codewords[i] & 0x80) ? 1 : 0;
        int scrambled;
        if (invert_sign)
            sign ^= 1;
        scrambled = sign ^ prev_sign;
        prev_sign = sign;
        (void) ja_descramble_reg_bit(&descramble_reg, scrambled);
    }

    /* Extract and descramble the payload bits. */
    for (i = 0; i < bit_count; i++) {
        int sign = (codewords[start_sample + i] & 0x80) ? 1 : 0;
        int scrambled;
        int plain;

        if (invert_sign)
            sign ^= 1;
        scrambled = sign ^ prev_sign;
        prev_sign = sign;
        plain = ja_descramble_reg_bit(&descramble_reg, scrambled);
        if (plain)
            packed_out[i / 8] |= (uint8_t) (1U << (i % 8));
    }

    return true;
}

/* -------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

bool v92_ja_dil_search(const uint8_t *codewords,
                       int total_codewords,
                       const ja_dil_search_params_t *params,
                       ja_dil_decode_t *out)
{
    enum { JA_TOP_SOFT = 8 };
    struct {
        int start;
        bool invert;
        int score;
        int sync_hd;
        int frame17;
        int frame17_run;
        int zviol;
        int crc_hd;
        int fspos;
    } top_soft[JA_TOP_SOFT];
    int best_score = -1;
    int best_soft_score = -1000000;
    int best_soft_sync_hd = 18;
    int best_soft_frame17 = JA_FRAME17_CHECK_BITS;
    int best_soft_frame17_run = 0;
    int best_soft_zviol = 12;
    int best_soft_crc_hd = 16;
    int best_soft_fspos = -1;
    bool best_invert = false;
    bool best_soft_invert = false;
    int best_start = -1;
    int best_soft_start = -1;
    v90_dil_desc_t best_desc;
    v90_dil_analysis_t best_analysis;
    v92_ja_parse_meta_t best_meta;
    uint8_t packed_bits[512];
    int search_start;
    int search_end;
    int candidate;
    int considered = 0;
    int decode_ok = 0;
    int parse_ok = 0;
    int parse_v92_ok = 0;
    int analyse_ok = 0;
    int debug = ja_debug_enabled();
    int i;

    if (!codewords || total_codewords <= 0 || !params || !out)
        return false;

    memset(out, 0, sizeof(*out));
    memset(&best_desc, 0, sizeof(best_desc));
    memset(&best_analysis, 0, sizeof(best_analysis));
    memset(&best_meta, 0, sizeof(best_meta));
    for (i = 0; i < JA_TOP_SOFT; i++) {
        top_soft[i].start = -1;
        top_soft[i].invert = false;
        top_soft[i].score = -1000000;
        top_soft[i].sync_hd = 18;
        top_soft[i].frame17 = JA_FRAME17_CHECK_BITS;
        top_soft[i].frame17_run = 0;
        top_soft[i].zviol = 12;
        top_soft[i].crc_hd = 16;
        top_soft[i].fspos = -1;
    }

    search_start = params->search_start;
    search_end   = params->search_end;

    /* Enforce minimum scrambler seed window. */
    if (search_start < (JA_SCRAMBLER_HISTORY + 1))
        search_start = JA_SCRAMBLER_HISTORY + 1;

    /* Enforce minimum lookahead so v90_parse_dil_descriptor has enough bits. */
    if (search_end > total_codewords - JA_MIN_DESCRIPTOR_BITS)
        search_end = total_codewords - JA_MIN_DESCRIPTOR_BITS;

    if (search_end < search_start)
        return false;

    for (candidate = search_start; candidate <= search_end; candidate++) {
        int invert;

        for (invert = 0; invert <= 1; invert++) {
            v90_dil_desc_t desc;
            v90_dil_analysis_t analysis;
            v92_ja_parse_meta_t parse_meta;
            int bit_count = total_codewords - candidate;
            int packed_len;
            int score;
            int soft_score;
            int sync_hd;
            int frame17;
            int frame17_run;
            int zviol;
            int crc_hd;
            int fspos;
            considered++;

            if (bit_count > (int) sizeof(packed_bits) * 8)
                bit_count = (int) sizeof(packed_bits) * 8;
            packed_len = (bit_count + 7) / 8;

            if (!ja_decode_bits_packed(codewords, total_codewords,
                                       candidate, bit_count, invert != 0,
                                       packed_bits, packed_len)) {
                continue;
            }
            memset(&parse_meta, 0, sizeof(parse_meta));
            decode_ok++;
            sync_hd = ja_sync_hamming18(packed_bits, bit_count);
            frame17 = ja_frame17_zero_violations(packed_bits, bit_count);
            frame17_run = ja_frame17_best_zero_run(packed_bits, bit_count);
            zviol = ja_reserved_zero_violations(packed_bits, bit_count);
            crc_hd = ja_crc_nearness_hd(packed_bits, bit_count);
            fspos = ja_fs12_position(packed_bits, bit_count);

            soft_score = ja_soft_score(packed_bits, bit_count);
            if (params->tx_ja_sample >= 0) {
                int sdist = candidate - params->tx_ja_sample;

                if (sdist < 0)
                    sdist = -sdist;
                soft_score -= sdist / 16;
            }
            if (soft_score > best_soft_score) {
                best_soft_score = soft_score;
                best_soft_start = candidate;
                best_soft_invert = (invert != 0);
                best_soft_sync_hd = sync_hd;
                best_soft_frame17 = frame17;
                best_soft_frame17_run = frame17_run;
                best_soft_zviol = zviol;
                best_soft_crc_hd = crc_hd;
                best_soft_fspos = fspos;
            }
            if (debug) {
                int pos;

                for (pos = 0; pos < JA_TOP_SOFT; pos++) {
                    if (soft_score > top_soft[pos].score) {
                        int j;

                        for (j = JA_TOP_SOFT - 1; j > pos; j--)
                            top_soft[j] = top_soft[j - 1];
                        top_soft[pos].start = candidate;
                        top_soft[pos].invert = (invert != 0);
                        top_soft[pos].score = soft_score;
                        top_soft[pos].sync_hd = sync_hd;
                        top_soft[pos].frame17 = frame17;
                        top_soft[pos].frame17_run = frame17_run;
                        top_soft[pos].zviol = zviol;
                        top_soft[pos].crc_hd = crc_hd;
                        top_soft[pos].fspos = fspos;
                        break;
                    }
                }
            }
            if (!v92_parse_ja_descriptor_strict(&desc, packed_bits, bit_count, &parse_meta))
                continue;
            if (parse_meta.is_v92)
                parse_v92_ok++;
            parse_ok++;
            if (!v90_analyse_dil_descriptor(&desc, &analysis))
                continue;
            analyse_ok++;

            /*
             * Score this candidate.  Higher unique_train_u and used_uchords
             * indicate a well-formed descriptor; impairment and non-default H
             * are penalties.  Distance from the known Ja anchor is also
             * penalised to favour candidates close to the expected position.
             */
            score = analysis.unique_train_u * 100
                  + analysis.used_uchords * 50
                  - analysis.impairment_score * 10
                  - analysis.non_default_h * 5;
            /* Prefer candidates that keep the expected 17-bit framing rhythm. */
            score -= frame17 * 6;
            score += frame17_run * 4;
            /* Lightweight sync sanity (should already be zero for valid parses). */
            score -= sync_hd * 12;
            if (fspos >= 0) {
                int fs_err = fspos - 4;
                if (fs_err < 0)
                    fs_err = -fs_err;
                score += 30;
                score -= fs_err * 2;
            } else {
                score -= 15;
            }
            score -= zviol * 5;
            score -= crc_hd * 4;
            if (parse_meta.is_v92) {
                int enabled_rates = 0;

                for (int rb = 0; rb < 16; rb++)
                    enabled_rates += (parse_meta.rate_mask_lo >> rb) & 1;
                for (int rb = 0; rb < 3; rb++)
                    enabled_rates += (parse_meta.rate_mask_hi >> rb) & 1;
                score += 24;
                score += enabled_rates * 2;
            }
            if (params->tx_ja_sample >= 0) {
                int dist = candidate - params->tx_ja_sample;
                if (dist < 0)
                    dist = -dist;
                score -= dist / 8;
            }

            if (score > best_score) {
                best_score    = score;
                best_invert   = (invert != 0);
                best_start    = candidate;
                best_desc     = desc;
                best_analysis = analysis;
                best_meta     = parse_meta;
            }
        }
    }

    if (best_score < 0) {
        if (debug) {
            fprintf(stderr,
                    "[JA] search=%d..%d considered=%d decode_ok=%d parse_ok=%d parse_v92_ok=%d analyse_ok=%d hard=none soft_start=%d soft_score=%d sync_hd=%d frame17=%d zviol=%d crc_hd=%d\n",
                    search_start,
                    search_end,
                    considered,
                    decode_ok,
                    parse_ok,
                    parse_v92_ok,
                    analyse_ok,
                    best_soft_start,
                    best_soft_score,
                    best_soft_sync_hd,
                    best_soft_frame17,
                    best_soft_zviol,
                    best_soft_crc_hd);
            if (best_soft_start >= 0) {
                fprintf(stderr,
                        "[JA] soft_detail start=%d invert=%d frame17_run=%d fs12_pos=%d\n",
                        best_soft_start,
                        best_soft_invert ? 1 : 0,
                        best_soft_frame17_run,
                        best_soft_fspos);
            }
            if (top_soft[0].start >= 0) {
                int r;
                for (r = 0; r < JA_TOP_SOFT && top_soft[r].start >= 0; r++) {
                    fprintf(stderr,
                            "[JA] top_soft[%d] start=%d invert=%d score=%d sync_hd=%d frame17=%d frame17_run=%d zviol=%d crc_hd=%d fs12_pos=%d\n",
                            r,
                            top_soft[r].start,
                            top_soft[r].invert ? 1 : 0,
                            top_soft[r].score,
                            top_soft[r].sync_hd,
                            top_soft[r].frame17,
                            top_soft[r].frame17_run,
                            top_soft[r].zviol,
                            top_soft[r].crc_hd,
                            top_soft[r].fspos);
                }
            }
        }
        if (best_soft_start < 0)
            return false;
        out->ok = false;
        out->soft_lock = true;
        out->calling_party = params->calling_party;
        out->u_info = params->u_info;
        out->start_sample = best_soft_start;
        out->invert_sign = best_soft_invert;
        out->soft_score = best_soft_score;
        out->soft_sync_hd = best_soft_sync_hd;
        out->soft_frame17_viol = best_soft_frame17;
        out->soft_zero_viol = best_soft_zviol;
        out->soft_crc_hd = best_soft_crc_hd;
        return true;
    }
    if (debug) {
        fprintf(stderr,
                "[JA] search=%d..%d considered=%d decode_ok=%d parse_ok=%d parse_v92_ok=%d analyse_ok=%d hard_start=%d hard_score=%d soft_start=%d soft_score=%d\n",
                search_start,
                search_end,
                considered,
                decode_ok,
                parse_ok,
                parse_v92_ok,
                analyse_ok,
                best_start,
                best_score,
                best_soft_start,
                best_soft_score);
    }

    out->ok           = true;
    out->soft_lock    = false;
    out->calling_party = params->calling_party;
    out->u_info        = params->u_info;
    out->start_sample  = best_start;
    out->invert_sign   = best_invert;
    out->parsed_v92    = best_meta.is_v92;
    out->descriptor_bits = best_meta.bit_len;
    out->soft_score = best_score;
    out->soft_sync_hd = 0;
    out->soft_frame17_viol = 0;
    out->soft_zero_viol = 0;
    out->soft_crc_hd = 0;
    out->desc          = best_desc;
    out->analysis      = best_analysis;
    return true;
}
