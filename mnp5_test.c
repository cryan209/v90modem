/* MNP5 wire vectors from an independent modem codec, streaming and restart
 * tests. Reference provenance and regeneration: docs/mnp.md. */
#include "mnp5.h"
/* These assertions execute the tests, even in a release build. */
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct { uint8_t bytes[262144]; size_t len; } buffer_t;
static buffer_t encoded, decoded;
static void collect(void *user, const uint8_t *data, int len)
{
    buffer_t *b = user;
    assert(len >= 0 && b->len + (size_t)len <= sizeof(b->bytes));
    memcpy(b->bytes + b->len, data, (size_t)len);
    b->len += (size_t)len;
}
static void stdout_output(void *user, const uint8_t *data, int len)
{
    (void)user;
    assert(fwrite(data, 1, (size_t)len, stdout) == (size_t)len);
}
static size_t unhex(const char *text, uint8_t *out)
{
    size_t n = 0;
    if (!strcmp(text, "-")) return 0;
    assert(strlen(text) % 2 == 0);
    while (*text) {
        unsigned value;
        assert(sscanf(text, "%2x", &value) == 1);
        out[n++] = (uint8_t)value;
        text += 2;
    }
    return n;
}
static void vectors(void)
{
    FILE *f = fopen("reference-data/mnp5/hsf-vectors.txt", "r");
    assert(f);
    static char line[65536];
    static uint8_t plain[16384], wire[16384];
    mnp5_t *tx = mnp5_init(collect, &encoded), *rx = mnp5_init(collect, &decoded);
    assert(tx && rx);
    unsigned segments = 0;
    while (fgets(line, sizeof(line), f)) {
        if (line[0] == '#' || line[0] == '\n') continue;
        if (line[0] == '!') { mnp5_restart(tx); mnp5_restart(rx); continue; }
        char *sep = strchr(line, ' ');
        assert(sep);
        *sep++ = 0;
        sep[strcspn(sep, "\r\n")] = 0;
        assert(strlen(line) < 2*sizeof(plain) && strlen(sep) < 2*sizeof(wire));
        size_t np = unhex(line, plain), nw = unhex(sep, wire);
        encoded.len = decoded.len = 0;
        for (size_t i = 0; i < np; i++) assert(mnp5_encode(tx, plain+i, 1) == 0);
        assert(mnp5_flush(tx) == 0);
        assert(encoded.len == nw && !memcmp(encoded.bytes, wire, nw));
        assert(mnp5_flush(tx) == 0 && encoded.len == nw); /* idle is silent */
        for (size_t i = 0; i < nw; i++) assert(mnp5_decode(rx, wire+i, 1) == 0);
        assert(decoded.len == np && !memcmp(decoded.bytes, plain, np));
        segments++;
    }
    fclose(f);
    mnp5_free(tx); mnp5_free(rx);
    printf("MNP5: %u independent HSF wire segments passed\n", segments);
}
static void streaming(void)
{
    static uint8_t plain[65536];
    unsigned rng = 0x6d6e7035;
    for (size_t i = 0; i < sizeof(plain); i++) {
        rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
        plain[i] = i%10000 < 1000 ? 0xc6 : (uint8_t)rng;
    }
    for (unsigned fragment = 1; fragment <= 1024; fragment *= 4) {
        mnp5_t *tx = mnp5_init(collect, &encoded), *rx = mnp5_init(collect, &decoded);
        assert(tx && rx);
        encoded.len = decoded.len = 0;
        for (size_t p = 0; p < sizeof(plain); ) {
            size_t n = sizeof(plain)-p < fragment ? sizeof(plain)-p : fragment;
            assert(mnp5_encode(tx, plain+p, n) == 0);
            if (p%17 == 0) assert(mnp5_flush(tx) == 0);
            p += n;
        }
        assert(mnp5_flush(tx) == 0);
        for (size_t p = 0; p < encoded.len; ) {
            size_t n = encoded.len-p < fragment ? encoded.len-p : fragment;
            assert(mnp5_decode(rx, encoded.bytes+p, n) == 0);
            p += n;
        }
        assert(decoded.len == sizeof(plain) && !memcmp(decoded.bytes, plain, sizeof(plain)));
        mnp5_free(tx); mnp5_free(rx);
    }
    /* Deterministic malformed-stream bounds / failure latching. */
    mnp5_t *rx = mnp5_init(collect, &decoded);
    assert(rx);
    for (unsigned i = 0; i < 2000; i++) {
        mnp5_restart(rx); decoded.len = 0;
        size_t n = i % 256;
        if (mnp5_decode(rx, plain+i, n) < 0)
            assert(mnp5_decode(rx, NULL, 0) < 0);
    }
    mnp5_free(rx);
    puts("MNP5: fragmented streams, idle flush, restart and malformed inputs passed");
}
int main(int argc, char **argv)
{
    if (argc == 2 && (!strcmp(argv[1], "--encode") || !strcmp(argv[1], "--decode"))) {
        uint8_t input[128]; size_t n;
        mnp5_t *s = mnp5_init(stdout_output, NULL);
        assert(s);
        while ((n = fread(input, 1, sizeof(input), stdin))) {
            int rc = !strcmp(argv[1], "--encode") ? mnp5_encode(s, input, n) : mnp5_decode(s, input, n);
            if (rc) { mnp5_free(s); return 1; }
        }
        int rc = !strcmp(argv[1], "--encode") ? mnp5_flush(s) : 0;
        mnp5_free(s);
        return rc || ferror(stdin) || ferror(stdout) ? 1 : 0;
    }
    assert(argc == 1);
    vectors(); streaming();
    return 0;
}
