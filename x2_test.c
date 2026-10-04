#include "x2.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "x2_firmware_vectors.h"

static uint32_t seed = 0x12345678;
static unsigned random_bit(void)
{
    seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5;
    return seed & 1;
}
static void firmware_test(void)
{
    unsigned i;
    for (i = 0; i < sizeof(firmware_cases)/sizeof(firmware_cases[0]); ++i) {
        const struct firmware_case *v = &firmware_cases[i];
        x2_pcm_config_t c = firmware_profiles[v->profile];
        x2_pcm_state_t tx = {(uint8_t)v->parity}, rx = tx;
        uint8_t octets[6]; uint64_t recovered;
        c.independent_signs = (uint8_t)v->md; c.format_xor = (uint8_t)v->format;
        assert(x2_pcm_encode(&c, &tx, v->payload, v->monitor, octets) == 0);
        assert(memcmp(octets, v->octets, 6) == 0);
        assert(tx.parity == v->final_parity);
        assert(x2_pcm_decode(&c, &rx, octets, &recovered) == 0);
        assert(recovered == v->payload && rx.parity == tx.parity);
    }
}
static void stream_test(void)
{
    unsigned md, tap, fmt, frame, i;
    for (md = 0; md <= 6; ++md) for (tap = 5; tap <= 18; tap += 13)
    for (fmt = 0; fmt < 2; ++fmt) {
        x2_pcm_config_t c = firmware_profiles[28]; /* B=37 */
        x2_pcm_state_t tx = {0}, rx = {0};
        x2_scrambler_t scrambler, descrambler;
        c.independent_signs = (uint8_t)md; c.format_xor = fmt ? 0x2a : 0;
        assert(x2_scrambler_init(&scrambler, tap, 0) == 0);
        assert(x2_scrambler_init(&descrambler, tap, 0) == 0);
        for (frame = 0; frame < 256; ++frame) {
            uint64_t plain = 0, scrambled = 0, decoded;
            uint8_t octets[6];
            for (i = 0; i < x2_pcm_frame_bits(&c); ++i) {
                unsigned b = random_bit();
                plain |= (uint64_t)b << i;
                scrambled |= (uint64_t)x2_scramble_bit(&scrambler, b) << i;
            }
            assert(x2_pcm_encode(&c, &tx, scrambled, (int16_t)(frame*251), octets) == 0);
            assert(x2_pcm_decode(&c, &rx, octets, &decoded) == 0);
            for (i = 0; i < x2_pcm_frame_bits(&c); ++i)
                assert(x2_descramble_bit(&descrambler, (unsigned)(decoded >> i)) == ((plain >> i) & 1));
            assert(tx.parity == rx.parity);
            assert(scrambler.history == descrambler.history);
        }
    }
}
struct bit_source { const uint8_t *bytes; unsigned position, available; };
static int get_source_bit(void *user_data)
{
    struct bit_source *source = user_data;
    unsigned position;
    if (source->position >= source->available) return -1;
    position = source->position++;
    return (source->bytes[position/8] >> (position%8)) & 1;
}
static void firmware_stream_test(void)
{
    unsigned i, chunk;
    for (i = 0; i < sizeof(firmware_streams)/sizeof(firmware_streams[0]); ++i)
    for (chunk = 0; chunk < 4; ++chunk) {
        const struct firmware_stream *v = &firmware_streams[i];
        x2_pcm_config_t c = firmware_profiles[v->profile];
        x2_pcm_tx_t tx;
        struct bit_source source = {v->source, 0, chunk == 3 ? 0 : v->source_count*8};
        uint8_t actual[192]; size_t position = 0;
        const size_t chunks[] = {1,17,160,7};
        c.independent_signs = (uint8_t)v->md; c.format_xor = (uint8_t)v->format;
        assert(x2_pcm_tx_init(&tx, &c, v->tap, get_source_bit, &source) == 0);
        while (position < sizeof(actual)) {
            size_t count = chunks[chunk];
            if (chunk == 3) source.available += 3;
            if (count > sizeof(actual)-position) count = sizeof(actual)-position;
            position += x2_pcm_tx_g711(&tx, actual+position, count);
        }
        assert(memcmp(actual, v->octets, sizeof(actual)) == 0);
        assert(source.position == 32*x2_pcm_frame_bits(&c));
    }
}
static void mp_test(void)
{
    x2_mp_t mp = {{0x0344,0x03fe,0,0x0500},0}, decoded;
    uint8_t bits[104]; unsigned i, tail;
    assert(x2_mp_crc(mp.words) == 0x14ad);
    for (tail = 0; tail < 4; ++tail) {
        mp.tail = (uint8_t)tail;
        assert(x2_mp_encode(&mp, bits) == 0);
        assert(x2_mp_decode(bits, &decoded) == 0);
        assert(memcmp(mp.words, decoded.words, sizeof(mp.words)) == 0 && decoded.tail == tail);
        /* Every protected body/framing bit must reject a single-bit error. */
        for (i = 0; i < 102; ++i) {
            x2_mp_t sentinel = {{1,2,3,4},3}; decoded = sentinel;
            bits[i] ^= 1;
            assert(x2_mp_decode(bits, &decoded) == -1);
            assert(memcmp(decoded.words, sentinel.words, sizeof(decoded.words)) == 0 && decoded.tail == 3);
            bits[i] ^= 1;
        }
    }
    bits[103] = 2; assert(x2_mp_decode(bits, &decoded) == -1);
    assert(x2_nominal_rate(1) == 33333 && x2_nominal_rate(2) == 37333);
    assert(x2_nominal_rate(12) == 53333 && x2_nominal_rate(15) == 57333);
    assert(x2_nominal_rate(0) == 0 && x2_nominal_rate(16) == 0);
}
static void invalid_test(void)
{
    x2_pcm_config_t c = firmware_profiles[0];
    x2_pcm_state_t s = {1}; uint8_t octets[6] = {1,2,3,4,5,6}; uint64_t bits = 123;
    assert(x2_pcm_encode(&c, &s, UINT64_C(1) << 19, 0, octets) == -1);
    assert(s.parity == 1 && octets[0] == 1);
    c.sizes[0] = 0; assert(x2_pcm_validate(&c) == -1);
    assert(x2_pcm_decode(&c, &s, octets, &bits) == -1 && bits == 123 && s.parity == 1);
    c = firmware_profiles[0]; c.banks[0][1] = c.banks[0][0];
    assert(x2_pcm_validate(&c) == -1);
    c = firmware_profiles[0]; c.amplitude_bits = 37;
    assert(x2_pcm_validate(&c) == -1); /* insufficient mixed-radix capacity */
    c = firmware_profiles[0]; c.independent_signs = 7;
    assert(x2_pcm_validate(&c) == -1);
    c = firmware_profiles[0]; c.format_xor = 1;
    assert(x2_pcm_validate(&c) == -1);
    assert(x2_pcm_validate(NULL) == -1);
    { x2_scrambler_t h; assert(x2_scrambler_init(&h, 17, 0) == -1); }
}
int main(void)
{
    firmware_test(); stream_test(); firmware_stream_test(); mp_test(); invalid_test();
    puts("x2: 840 original DSP vectors, 7168 continuous frames, 56 DSP streams at four chunk/pause patterns, MP CRC/framing and invalid inputs passed");
    return 0;
}
