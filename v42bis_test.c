/* V.42bis 5.6/5.8, 7.8.3 and 7.9 lifecycle/fragmentation regressions. */
#include <stdint.h>
#include <stdbool.h>
#include <spandsp/telephony.h>
#include <spandsp/logging.h>
#include <spandsp/async.h>
#include <spandsp/v42bis.h>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CAP 1048576
struct output { uint8_t *bytes; int length; };
static void collect(void *ctx, const uint8_t *bytes, int length)
{
    struct output *o = ctx;
    assert(length >= 0 && o->length + length <= CAP);
    memcpy(o->bytes + o->length, bytes, length);
    o->length += length;
}
static uint32_t seed = 0x12345678;
static uint8_t random_byte(void)
{
    seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5;
    return (uint8_t)seed;
}
int main(void)
{
    struct output wire = {malloc(CAP), 0}, plain = {malloc(CAP), 0};
    uint8_t *data = malloc(250000);
    assert(wire.bytes && plain.bytes && data);
    const int sizes[] = {512, 768, 4096, 8192, 32768, 65535};
    for (unsigned n = 0; n < sizeof(sizes)/sizeof(sizes[0]); n++)
    {
        v42bis_state_t *s = v42bis_init(NULL, 3, sizes[n], 250,
                                       collect, &wire, 31, collect, &plain, 17);
        assert(s);
        for (int phase = 0; phase < 3; phase++)
        {
            wire.length = plain.length = 0;
            v42bis_compression_control(s, phase == 1 ? 2 : 1);
            for (int i = 0; i < 250000; i++) data[i] = random_byte();
            for (int offset = 0; offset < 250000; offset += 71)
            {
                int length = 250000 - offset;
                if (length > 71) length = 71;
                assert(v42bis_compress(s, data + offset, length) == 0);
                assert(v42bis_compress_flush(s) == 0);
            }
            int before = wire.length;
            assert(v42bis_compress_flush(s) == 0 && wire.length == before);
            assert(v42bis_compress_reset(s) == 0);
            for (int i = 0; i < wire.length; i++)
            {
                assert(v42bis_decompress(s, wire.bytes + i, 1) == 0);
                assert(v42bis_decompress_flush(s) == 0);
            }
            assert(plain.length == 250000 && !memcmp(plain.bytes, data, 250000));
        }
        assert(v42bis_release(s) == 0);
        assert(v42bis_release(s) == 0);
        v42bis_free(s);
    }
    /* Deterministic malformed streams: bounds and latched C-ERROR recovery. */
    v42bis_state_t *s = v42bis_init(NULL, 3, 768, 6,
                                   collect, &wire, 31, collect, &plain, 17);
    assert(s);
    for (int trial = 0; trial < 2000; trial++)
    {
        assert(v42bis_restart(s) == 0);
        wire.length = plain.length = 0;
        data[0] = data[1] = 0; /* ECM, then arbitrary codewords. */
        for (int i = 2; i < 256; i++) data[i] = random_byte();
        int result = v42bis_decompress(s, data, 256);
        if (result == -1)
        {
            int before = plain.length;
            assert(v42bis_decompress(s, (const uint8_t *)"abc", 3) == -1);
            assert(v42bis_decompress_flush(s) == -1 && plain.length == before);
        }
        else assert(v42bis_decompress_flush(s) == 0);
    }
    v42bis_free(s);
    free(data); free(wire.bytes); free(plain.bytes);
    puts("V.42bis lifecycle, dictionary limits and malformed-stream tests passed");
    return 0;
}
