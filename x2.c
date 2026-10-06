#include "x2.h"
#include <limits.h>
#include <stddef.h>
#include <string.h>

int x2_pcm_validate(const x2_pcm_config_t *c)
{
    uint64_t capacity = 1;
    unsigned i, j;
    if (!c || c->amplitude_bits < 19 || c->amplitude_bits > 37
        || c->independent_signs > 6 || (c->format_xor != 0 && c->format_xor != 0x2a))
        return -1;
    for (i = 0; i < 6; ++i) {
        if (!c->sizes[i] || c->sizes[i] > 128) return -1;
        capacity *= c->sizes[i];
        {
            uint64_t seen[2] = {0,0};
            for (j = 0; j < c->sizes[i]; ++j) {
                unsigned code = c->banks[i][j] & 127;
                uint64_t mask = UINT64_C(1) << (code & 63);
                if (seen[code >> 6] & mask) return -1;
                seen[code >> 6] |= mask;
            }
        }
    }
    return capacity < (UINT64_C(1) << c->amplitude_bits) ? -1 : 0;
}
unsigned x2_pcm_frame_bits(const x2_pcm_config_t *c)
{
    return c ? (unsigned)c->amplitude_bits + c->independent_signs : 0;
}
/* QF060003 C5DF..C6E4; Draft 0.33 payload clauses. Equal level keys
 * rank earlier positions first; independent signs are assigned in emission
 * order, not rank order. MD=6 bypasses the shaping search entirely.
 */
static void ranks_for(const uint16_t entries[6], unsigned md, unsigned ranks[6])
{
    unsigned i, j;
    for (i = 0; i < 6; ++i) {
        unsigned key = (entries[i] & 0x7f00) | (5 - i);
        ranks[i] = 0;
        if (md == 6) { ranks[i] = i; continue; }
        for (j = 0; j < 6; ++j)
            if (((entries[j] & 0x7f00) | (5 - j)) > key) ++ranks[i];
    }
}
int x2_pcm_encode(const x2_pcm_config_t *c, x2_pcm_state_t *s,
                  uint64_t bits, int16_t monitor, uint8_t octets[6])
{
    uint16_t entries[6];
    unsigned ranks[6], toggles[6], free_positions[6], free_count = 0;
    unsigned i, candidate, best_mask = 0, parity;
    int32_t fixed = (int32_t)monitor * 8, best = INT32_MAX;
    uint64_t amplitude;
    if (!s || !octets || x2_pcm_validate(c) || s->parity > 1
        || bits >= (UINT64_C(1) << x2_pcm_frame_bits(c))) return -1;
    amplitude = bits >> c->independent_signs;
    for (i = 0; i < 6; ++i) {
        entries[i] = c->banks[i][amplitude % c->sizes[i]];
        amplitude /= c->sizes[i];
    }
    ranks_for(entries, c->independent_signs, ranks);
    parity = s->parity;
    for (i = 0; i < 6; ++i) {
        int32_t level = c->levels[(entries[i] >> 8) & 127];
        if (ranks[i] < c->independent_signs) {
            parity ^= bits & 1;
            bits >>= 1;
            toggles[i] = parity;
            fixed += parity ? -level : level;
        } else free_positions[free_count++] = i;
    }
    /* Descending masks and strict improvement preserve firmware tie order. */
    for (candidate = 1u << free_count; candidate-- > 0;) {
        int32_t score = fixed;
        for (i = 0; i < free_count; ++i) {
            int32_t level = c->levels[(entries[free_positions[i]] >> 8) & 127];
            score += (candidate & (1u << i)) ? -level : level;
        }
        if (score < 0) score = -score;
        if (score < best) { best = score; best_mask = candidate; }
    }
    for (i = 0; i < free_count; ++i) toggles[free_positions[i]] = (best_mask >> i) & 1;
    for (i = 0; i < 6; ++i)
        octets[i] = (uint8_t)(entries[i] ^ (toggles[i] << 7) ^ c->format_xor);
    s->parity = (uint8_t)parity;
    return 0;
}
int x2_pcm_decode(const x2_pcm_config_t *c, x2_pcm_state_t *s,
                  const uint8_t octets[6], uint64_t *bits)
{
    uint16_t entries[6];
    unsigned digits[6], signs[6], ranks[6], i, j, parity, n = 0, raw = 0;
    uint64_t amplitude = 0;
    if (!s || !octets || !bits || x2_pcm_validate(c) || s->parity > 1) return -1;
    for (i = 0; i < 6; ++i) {
        unsigned code = octets[i] ^ c->format_xor;
        for (j = 0; j < c->sizes[i]; ++j)
            if (((c->banks[i][j] ^ code) & 127) == 0) break;
        if (j == c->sizes[i]) return -1;
        entries[i] = c->banks[i][j]; digits[i] = j;
        signs[i] = ((entries[i] ^ code) >> 7) & 1;
    }
    for (i = 6; i-- > 0;) amplitude = amplitude * c->sizes[i] + digits[i];
    if (amplitude >= (UINT64_C(1) << c->amplitude_bits)) return -1;
    ranks_for(entries, c->independent_signs, ranks);
    parity = s->parity;
    for (i = 0; i < 6; ++i) if (ranks[i] < c->independent_signs) {
        raw |= (signs[i] ^ parity) << n++;
        parity = signs[i];
    }
    *bits = (amplitude << c->independent_signs) | raw;
    s->parity = (uint8_t)parity;
    return 0;
}
int x2_scrambler_init(x2_scrambler_t *s, unsigned tap, uint32_t history)
{
    if (!s || (tap != 5 && tap != 18) || history > 0x7fffff) return -1;
    s->tap = (uint8_t)tap; s->history = history; return 0;
}
static unsigned scrambler_step(x2_scrambler_t *s, unsigned bit, unsigned decode)
{
    unsigned out = (bit & 1) ^ ((s->history >> (23 - s->tap)) & 1) ^ (s->history & 1);
    s->history = (s->history >> 1) | ((decode ? bit & 1 : out) << 22);
    return out;
}
unsigned x2_scramble_bit(x2_scrambler_t *s, unsigned bit) { return scrambler_step(s, bit, 0); }
unsigned x2_descramble_bit(x2_scrambler_t *s, unsigned bit) { return scrambler_step(s, bit, 1); }
/* Courier four-word MP: 17 ones, zero, four (16 LSB-first bits, zero)
 * words, reflected 0x8408 CRC (init ffff, no final XOR), two tail bits.
 */
uint16_t x2_mp_crc(const uint16_t words[4])
{
    uint16_t crc = 0xffff;
    unsigned i, j;
    for (i = 0; i < 4; ++i) for (j = 0; j < 16; ++j) {
        unsigned feedback = (crc ^ (words[i] >> j)) & 1;
        crc = (uint16_t)((crc >> 1) ^ (feedback ? 0x8408 : 0));
    }
    return crc;
}
int x2_mp_encode(const x2_mp_t *mp, uint8_t bits[104])
{
    unsigned i, j;
    uint16_t crc;
    if (!mp || !bits || mp->tail > 3) return -1;
    for (i = 0; i < 17; ++i) bits[i] = 1;
    bits[17] = 0;
    for (i = 0; i < 4; ++i) {
        for (j = 0; j < 16; ++j) bits[18 + 17*i + j] = (mp->words[i] >> j) & 1;
        bits[34 + 17*i] = 0;
    }
    crc = x2_mp_crc(mp->words);
    for (i = 0; i < 16; ++i) bits[86 + i] = (crc >> i) & 1;
    bits[102] = mp->tail & 1; bits[103] = (mp->tail >> 1) & 1;
    return 0;
}
int x2_mp_decode(const uint8_t bits[104], x2_mp_t *mp)
{
    x2_mp_t result = {{0,0,0,0},0};
    uint16_t crc = 0;
    unsigned i, j;
    if (!bits || !mp) return -1;
    for (i = 0; i < 104; ++i) if (bits[i] > 1) return -1;
    for (i = 0; i < 17; ++i) if (bits[i] != 1) return -1;
    if (bits[17]) return -1;
    for (i = 0; i < 4; ++i) {
        if (bits[34 + 17*i]) return -1;
        for (j = 0; j < 16; ++j) result.words[i] |= (uint16_t)(bits[18 + 17*i+j] << j);
    }
    for (i = 0; i < 16; ++i) crc |= (uint16_t)(bits[86+i] << i);
    if (crc != x2_mp_crc(result.words)) return -1;
    result.tail = bits[102] | (bits[103] << 1);
    *mp = result; return 0;
}
unsigned x2_nominal_rate(unsigned index)
{
    if (!index || index > 15) return 0;
    if (index == 1) return 33333;
    if (index == 2) return 37333;
    return (index + 28) * 8000 / 6;
}

int x2_pcm_tx_init(x2_pcm_tx_t *tx, const x2_pcm_config_t *config,
                   unsigned tap, x2_get_bit_func_t get_bit, void *user_data)
{
    x2_pcm_tx_t initialized;
    if (!tx || !get_bit || x2_pcm_validate(config)) return -1;
    memset(&initialized, 0, sizeof(initialized));
    if (x2_scrambler_init(&initialized.scrambler, tap, 0)) return -1;
    initialized.config = *config;
    initialized.get_bit = get_bit; initialized.user_data = user_data;
    initialized.output_position = 6;
    *tx = initialized;
    return 0;
}
/* QF060003 C544/C558/C565, Draft 0.33 11.3. Monitor state uses
 * signed 16-bit storage after every sample, exactly as the DSP does.
 */
size_t x2_pcm_tx_g711(x2_pcm_tx_t *tx, uint8_t *octets, size_t count)
{
    size_t written = 0;
    if (!tx || !octets || !tx->get_bit) return 0;
    while (written < count) {
        unsigned position, code, j;
        int32_t level, previous;
        if (tx->output_position == 6) {
            while (tx->pending_count < x2_pcm_frame_bits(&tx->config)) {
                int bit = tx->get_bit(tx->user_data);
                if (bit < 0 || bit > 1) return written;
                tx->pending |= (uint64_t)x2_scramble_bit(&tx->scrambler, (unsigned)bit) << tx->pending_count++;
            }
            if (x2_pcm_encode(&tx->config, &tx->mapper, tx->pending,
                              tx->disparity_history[4], tx->output)) return written;
            tx->pending = 0; tx->pending_count = 0; tx->output_position = 0;
        }
        position = tx->output_position++;
        code = tx->output[position] ^ tx->config.format_xor;
        for (j = 0; j < tx->config.sizes[position]; ++j)
            if (((tx->config.banks[position][j] ^ code) & 127) == 0) break;
        level = tx->config.levels[(tx->config.banks[position][j] >> 8) & 127];
        if (!(code & 128)) level = -level;
        /* Defined floor division avoids relying on signed right-shift. */
        level = level >= 0 ? level / 8 : -((-level + 7) / 8);
        previous = tx->disparity_history[3];
        for (j = 5; j > 0; --j) tx->disparity_history[j] = tx->disparity_history[j-1];
        tx->disparity_history[0] = (int16_t)level;
        tx->disparity_history[3] = (int16_t)(previous + level);
        octets[written++] = tx->output[position];
    }
    return written;
}

/* Draft 0.33 §12.2; Ie030002 95AE..95C2 and QF060003 9749..975D.
 * ITU 18 (body 6) is high-carrier 3200 ability, 23 (body 11) symmetric
 * ability, 24 (body 12) CME/server. Symmetric takes priority over CME.
 * The caller chooses a role only after validating the received INFO0 CRC. */
x2_role_t x2_info_role_select(uint32_t local, uint32_t peer)
{
    if (local >= (1u << 17) || peer >= (1u << 17) || !(peer & 0x40))
        return X2_ROLE_NONE;
    if (local & peer & 0x800) return X2_ROLE_SYMMETRIC;
    if (!((local ^ peer) & 0x1000)) return X2_ROLE_NONE;
    return (local & 0x1000) ? X2_ROLE_HOST : X2_ROLE_CLIENT;
}
