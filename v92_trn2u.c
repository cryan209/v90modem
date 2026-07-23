/*
 * v92_trn2u.c — V.92 TRN2u PAM modem (upstream Phase 4 control channel)
 */

#include "v92_trn2u.h"

#include <spandsp.h>

#include <math.h>
#include <string.h>

/* Upstream GPA scrambler, 1 + x^-5 + x^-23 (delay taps 5/23; V.92 §6.3).
 * Read taps 18/23 = GPC until 2026-07-23 — see v92_p3_rx.c's
 * gpa_descramble() comment for the misread and the live tap-4 evidence.
 * Matches v92_p3_rx.c. */
static inline int v92_gpa_scramble(uint32_t *reg, int in_bit)
{
    int out = (in_bit ^ (int)(*reg >> 22) ^ (int)(*reg >> 4)) & 1;

    *reg = (*reg << 1) | (uint32_t)out;
    return out;
}

static inline int v92_trn2u_descramble(v92_trn2u_demod_t *demod, int in_bit)
{
    uint32_t *reg = &demod->descramble_reg;
    int out;

    /* Tap sets corrected 2026-07-23: each label now implements the V.34
     * clause-7 polynomial it names (GPA = delay taps 5/23, GPC = 18/23);
     * they had been swapped.  LEFT = newest output at bit 0 of the shift
     * register, RIGHT = the reflected register convention. */
    switch ((v92_trn2u_descrambler_mode_t)demod->descrambler_mode) {
    case V92_TRN2U_DESCRAMBLER_GPA_RIGHT:
        out = (in_bit ^ (int)(*reg) ^ (int)(*reg >> 18)) & 1;
        *reg = (*reg >> 1) | ((uint32_t)(in_bit & 1) << 22);
        break;
    case V92_TRN2U_DESCRAMBLER_GPC_LEFT:
        out = (in_bit ^ (int)(*reg >> 22) ^ (int)(*reg >> 17)) & 1;
        *reg = ((*reg << 1) | (uint32_t)(in_bit & 1)) & 0x7FFFFFU;
        break;
    case V92_TRN2U_DESCRAMBLER_GPC_RIGHT:
        out = (in_bit ^ (int)(*reg) ^ (int)(*reg >> 5)) & 1;
        *reg = (*reg >> 1) | ((uint32_t)(in_bit & 1) << 22);
        break;
    case V92_TRN2U_DESCRAMBLER_GPA_LEFT:
    default:
        out = (in_bit ^ (int)(*reg >> 22) ^ (int)(*reg >> 4)) & 1;
        *reg = ((*reg << 1) | (uint32_t)(in_bit & 1)) & 0x7FFFFFU;
        break;
    }
    return out;
}

int v92_trn2u_bits_per_symbol(int constellation_points)
{
    if (constellation_points == 2)
        return 1;
    if (constellation_points == 4)
        return 2;
    if (constellation_points == 8)
        return 3;
    return 0;
}

/* Phase-3 TRN1u uses ±L_U.  Table 28/29 magnitudes are
 * (1,3)/sqrt(5) and (1..7)/sqrt(21). */
static double v92_trn2u_level(int constellation_points, int label, double lu)
{
    if (constellation_points == 2)
        return lu;
    if (constellation_points == 4)
        return ((double)(2 * label + 1) / sqrt(5.0)) * lu;
    return ((double)(2 * label + 1) / sqrt(21.0)) * lu;
}

static uint8_t v92_trn2u_encode_linear(bool alaw, double value)
{
    int16_t sample;

    if (value > 32767.0)
        value = 32767.0;
    if (value < -32768.0)
        value = -32768.0;
    sample = (int16_t)lrint(value);
    return alaw ? linear_to_alaw(sample) : linear_to_ulaw(sample);
}

static int16_t v92_trn2u_decode_linear(bool alaw, uint8_t codeword)
{
    return alaw ? alaw_to_linear(codeword) : ulaw_to_linear(codeword);
}

void v92_trn2u_tx_init(v92_trn2u_tx_t *tx,
                       int constellation_points,
                       double lu,
                       bool alaw)
{
    if (!tx)
        return;
    memset(tx, 0, sizeof(*tx));
    tx->constellation_points = constellation_points;
    tx->lu = lu;
    tx->alaw = alaw;
}

void v92_trn2u_tx_start(v92_trn2u_tx_t *tx, int preceding_e1u_sign)
{
    if (!tx)
        return;
    tx->scramble_reg = 0;
    tx->prev_sign = preceding_e1u_sign ? 1 : 0;
}

static uint8_t v92_trn2u_tx_symbol(v92_trn2u_tx_t *tx, const int *bits)
{
    int bps = v92_trn2u_bits_per_symbol(tx->constellation_points);
    int msb;
    int label = 0;
    double value;

    /* First bit is the constellation MSB (sign), differentially encoded. */
    msb = bits[0] ^ tx->prev_sign;
    tx->prev_sign = msb;
    for (int i = 1; i < bps; i++)
        label = (label << 1) | bits[i];
    value = v92_trn2u_level(tx->constellation_points, label, tx->lu);
    if (msb)
        value = -value;
    return v92_trn2u_encode_linear(tx->alaw, value);
}

int v92_trn2u_tx_bits(v92_trn2u_tx_t *tx,
                      const uint8_t *bits,
                      int nbits,
                      uint8_t *codewords,
                      int codewords_max)
{
    int bps;
    int nsymbols;
    int scrambled[3];

    if (!tx || !bits || !codewords)
        return 0;
    bps = v92_trn2u_bits_per_symbol(tx->constellation_points);
    if (bps == 0 || nbits <= 0 || (nbits % bps) != 0)
        return 0;
    nsymbols = nbits / bps;
    if (nsymbols > codewords_max)
        return 0;
    for (int s = 0; s < nsymbols; s++) {
        for (int i = 0; i < bps; i++)
            scrambled[i] = v92_gpa_scramble(&tx->scramble_reg,
                                            bits[s * bps + i] & 1);
        codewords[s] = v92_trn2u_tx_symbol(tx, scrambled);
    }
    return nsymbols;
}

int v92_trn2u_tx_ones(v92_trn2u_tx_t *tx,
                      uint8_t *codewords,
                      int nsymbols)
{
    int bps;
    int scrambled[3];

    if (!tx || !codewords || nsymbols <= 0)
        return 0;
    bps = v92_trn2u_bits_per_symbol(tx->constellation_points);
    if (bps == 0)
        return 0;
    for (int s = 0; s < nsymbols; s++) {
        for (int i = 0; i < bps; i++)
            scrambled[i] = v92_gpa_scramble(&tx->scramble_reg, 1);
        codewords[s] = v92_trn2u_tx_symbol(tx, scrambled);
    }
    return nsymbols;
}

void v92_trn2u_demod_init(v92_trn2u_demod_t *demod,
                          int constellation_points,
                          double lu,
                          bool alaw,
                          v92_cp_rx_t *sink)
{
    if (!demod)
        return;
    memset(demod, 0, sizeof(*demod));
    demod->constellation_points = constellation_points;
    demod->lu = lu;
    demod->alaw = alaw;
    demod->sink = sink;
    demod->bit_permutation[0] = 0;
    demod->bit_permutation[1] = 1;
    demod->bit_permutation[2] = 2;
}

bool v92_trn2u_demod_set_hypothesis(
    v92_trn2u_demod_t *demod,
    const int bit_permutation[3],
    v92_trn2u_sign_mode_t sign_mode,
    v92_trn2u_descrambler_mode_t descrambler_mode)
{
    int bps;
    bool seen[3] = {false, false, false};

    if (!demod || !bit_permutation
        || sign_mode < V92_TRN2U_SIGN_DIFFERENTIAL
        || sign_mode > V92_TRN2U_SIGN_ABSOLUTE_INVERTED
        || descrambler_mode < V92_TRN2U_DESCRAMBLER_GPA_LEFT
        || descrambler_mode > V92_TRN2U_DESCRAMBLER_GPC_RIGHT)
        return false;
    bps = v92_trn2u_bits_per_symbol(demod->constellation_points);
    for (int i = 0; i < bps; i++) {
        int p = bit_permutation[i];

        if (p < 0 || p >= bps || seen[p])
            return false;
        seen[p] = true;
        demod->bit_permutation[i] = p;
    }
    demod->sign_mode = sign_mode;
    demod->descrambler_mode = descrambler_mode;
    return true;
}

static void v92_trn2u_put_recovered_bit(v92_trn2u_demod_t *demod,
                                         int raw_bit,
                                         int *accepted)
{
    int bit = v92_trn2u_descramble(demod, raw_bit);

    if (bit) {
        demod->descrambled_one_run++;
        if (demod->descrambled_one_run > demod->longest_descrambled_one_run)
            demod->longest_descrambled_one_run = demod->descrambled_one_run;
    } else {
        demod->descrambled_one_run = 0;
    }
    if (demod->sink && v92_cp_rx_put_bit(demod->sink, bit))
        (*accepted)++;
}

static int v92_trn2u_slice_label(const v92_trn2u_demod_t *demod,
                                 double magnitude)
{
    int labels = demod->constellation_points / 2;
    int label = labels - 1;

    (void)magnitude;
    if (demod->constellation_points == 2)
        return 0;

    for (int i = 0; i < labels - 1; i++) {
        double threshold = ((double)(2 * i + 2)
                            / sqrt(demod->constellation_points == 4
                                       ? 5.0 : 21.0))
                           * demod->lu;

        if (magnitude < threshold) {
            label = i;
            break;
        }
    }
    return label;
}

int v92_trn2u_demod_feed(v92_trn2u_demod_t *demod,
                         const uint8_t *codewords,
                         int count)
{
    int bps;
    int accepted = 0;

    if (!demod || !codewords || count <= 0)
        return 0;
    bps = v92_trn2u_bits_per_symbol(demod->constellation_points);
    if (bps == 0)
        return 0;
    for (int s = 0; s < count; s++) {
        int16_t linear = v92_trn2u_decode_linear(demod->alaw, codewords[s]);
        int sign = (linear < 0) ? 1 : 0;
        int label = v92_trn2u_slice_label(demod,
                                          linear < 0 ? -(double)linear
                                                     : (double)linear);
        int recovered[3] = {0, 0, 0};
        int msb;

        demod->symbols++;
        if (!demod->prev_sign_valid
            && (demod->sign_mode == V92_TRN2U_SIGN_DIFFERENTIAL
                || demod->sign_mode == V92_TRN2U_SIGN_DIFFERENTIAL_INVERTED)) {
            demod->prev_sign = sign;
            demod->prev_sign_valid = true;
            continue;
        }
        if (demod->sign_mode == V92_TRN2U_SIGN_ABSOLUTE
            || demod->sign_mode == V92_TRN2U_SIGN_ABSOLUTE_INVERTED) {
            msb = sign;
        } else {
            msb = sign ^ demod->prev_sign;
        }
        if (demod->sign_mode == V92_TRN2U_SIGN_DIFFERENTIAL_INVERTED
            || demod->sign_mode == V92_TRN2U_SIGN_ABSOLUTE_INVERTED)
            msb ^= 1;
        demod->prev_sign = sign;
        demod->prev_sign_valid = true;
        recovered[0] = msb;
        for (int i = 1; i < bps; i++)
            recovered[i] = (label >> (bps - 1 - i)) & 1;

        for (int i = 0; i < bps; i++)
            v92_trn2u_put_recovered_bit(
                demod, recovered[demod->bit_permutation[i]], &accepted);
    }
    demod->frames_accepted += (uint32_t)accepted;
    return accepted;
}
