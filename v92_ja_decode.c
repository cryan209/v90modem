/*
 * v92_ja_decode.c — V.92 Phase 3 Ja / DIL descriptor decoder
 *
 * Core implementation of v92_ja_dil_search().  The bit-extraction helpers
 * (V.90 differential sign + x^23+x^5+1 descrambler) live here as statics so
 * this module has no dependency on vpcm_decode internals.
 */

#include "v92_ja_decode.h"

#include <string.h>

/* -------------------------------------------------------------------------
 * V.90 scrambler constants (ITU-T V.90 §5.4.4)
 * ------------------------------------------------------------------------- */

/* Length of the scrambler shift register (polynomial degree). */
#define JA_SCRAMBLER_HISTORY   23

/* Minimum bits of lookahead needed to hold the smallest valid DIL descriptor. */
#define JA_MIN_DESCRIPTOR_BITS 206

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
    int best_score = -1;
    bool best_invert = false;
    int best_start = -1;
    v90_dil_desc_t best_desc;
    v90_dil_analysis_t best_analysis;
    uint8_t packed_bits[512];
    int search_start;
    int search_end;
    int candidate;

    if (!codewords || total_codewords <= 0 || !params || !out)
        return false;

    memset(out, 0, sizeof(*out));
    memset(&best_desc, 0, sizeof(best_desc));
    memset(&best_analysis, 0, sizeof(best_analysis));

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
            int bit_count = total_codewords - candidate;
            int packed_len;
            int score;

            if (bit_count > (int) sizeof(packed_bits) * 8)
                bit_count = (int) sizeof(packed_bits) * 8;
            packed_len = (bit_count + 7) / 8;

            if (!ja_decode_bits_packed(codewords, total_codewords,
                                       candidate, bit_count, invert != 0,
                                       packed_bits, packed_len)) {
                continue;
            }
            if (!v90_parse_dil_descriptor(&desc, packed_bits, bit_count))
                continue;
            if (!v90_analyse_dil_descriptor(&desc, &analysis))
                continue;

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
            }
        }
    }

    if (best_score < 0)
        return false;

    out->ok           = true;
    out->calling_party = params->calling_party;
    out->u_info        = params->u_info;
    out->start_sample  = best_start;
    out->invert_sign   = best_invert;
    out->desc          = best_desc;
    out->analysis      = best_analysis;
    return true;
}
