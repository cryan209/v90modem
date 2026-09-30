/* Public codec tests, including ITU wire vectors and a foreign CX93001 capture. */
#include "v44.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct { uint8_t bytes[131072]; size_t n; bool overflow; } sink_t;
static int failures;
#define CHECK(c, label) do { if (!(c)) { fprintf(stderr, "FAIL: %s\n", label); failures++; } } while (0)
static void output(void *ctx, const uint8_t *p, int n)
{
    sink_t *s = ctx;
    if (n < 0 || (size_t)n > sizeof(s->bytes) - s->n) { s->overflow = true; return; }
    memcpy(s->bytes + s->n, p, (size_t)n); s->n += (size_t)n;
}
static void roundtrip(int n2, int n7, int n8, bool random)
{
    sink_t wire = {0}, plain = {0};
    uint8_t data[12000];
    unsigned seed = 44;
    for (size_t i = 0; i < sizeof(data); i++) {
        seed = seed * 1664525u + 1013904223u;
        data[i] = random ? (uint8_t)(seed >> 24) : (uint8_t)"ABCDEFGH"[i % 8];
    }
    v44_encoder_t *enc = v44_encoder_init(n2,n7,n8,output,&wire);
    v44_decoder_t *dec = v44_decoder_init(n2,n7,n8,output,&plain);
    CHECK(enc && dec, "codec initialization");
    if (!enc || !dec) goto done;
    for (size_t i = 0; i < sizeof(data); i += 73) {
        size_t n = sizeof(data) - i; if (n > 73) n = 73;
        CHECK(v44_encoder_feed(enc,data+i,n) == 0 && v44_encoder_flush(enc) == 0,
              "incremental encoding and flush");
    }
    for (size_t i = 0; i < wire.n; i++)
        CHECK(v44_decoder_feed(dec,wire.bytes+i,1) == 0, "one-byte decode fragments");
    CHECK(!wire.overflow && !plain.overflow && plain.n == sizeof(data)
          && memcmp(plain.bytes,data,sizeof(data)) == 0, "dictionary continuity and reinitialization");
    if (!random) CHECK(wire.n < sizeof(data)/2, "repetition compresses");
    /* C-INIT discards both partially received bits and old dictionaries. */
    plain.n = 0;
    v44_decoder_reset(dec);
    v44_decoder_feed(dec,wire.bytes,1);
    v44_decoder_reset(dec);
    plain.n = 0;
    CHECK(v44_decoder_feed(dec,wire.bytes,wire.n) == 0 && plain.n == sizeof(data)
          && memcmp(plain.bytes,data,sizeof(data)) == 0, "decoder C-INIT resets partial stream");
done:
    v44_encoder_free(enc); v44_decoder_free(dec);
}
typedef struct { uint8_t p[2048]; size_t n; uint32_t bits; int count; } writer_t;
static void put(writer_t *w, unsigned v, int width)
{
    w->bits |= v << w->count; w->count += width;
    while (w->count >= 8) { w->p[w->n++] = (uint8_t)w->bits; w->bits >>= 8; w->count -= 8; }
}
static void align(writer_t *w)
{
    if (w->count) w->p[w->n++] = (uint8_t)w->bits;
    w->bits = 0; w->count = 0;
}
static void extensions(void)
{
    for (int length = 1; length <= 253; length++) {
        writer_t w = {0}; sink_t plain = {0};
        for (unsigned b = 'A'; b <= 'C'; b++) { put(&w,0,1); put(&w,b,7); }
        put(&w,1,1); put(&w,4,6); /* AB */
        put(&w,0,1); put(&w,1,1); /* string-extension prefix 01 */
        if (length == 1) put(&w,1,1);
        else if (length <= 4) { put(&w,0,1); put(&w,(unsigned)length-1,2); }
        else if (length <= 12) { put(&w,0,4); put(&w,(unsigned)length-5,3); }
        else { put(&w,8,4); put(&w,(unsigned)length-13,8); }
        put(&w,1,1); put(&w,1,6); align(&w);
        v44_decoder_t *dec = v44_decoder_init(512,255,1024,output,&plain);
        CHECK(dec != NULL, "extension initialization");
        if (!dec) continue;
        for (size_t i = 0; i < w.n; i++) CHECK(v44_decoder_feed(dec,w.p+i,1) == 0,"Table 4 extension length");
        uint8_t expected[258] = {'A','B','C','A','B'};
        for (int i = 0; i < length; i++) expected[5+i] = expected[2+i];
        CHECK(plain.n == (size_t)(5+length) && memcmp(plain.bytes,expected,plain.n) == 0,
              "overlapping extension contents");
        v44_decoder_free(dec);
    }
}
static void captured_peer(void)
{
    static const uint8_t wire[] = {0xc6,0xf0,0x5a,0xec,0x68,0x68,0x5a,0x82,0x17,0x31,0x66,0x32,0x99,0x4c,0x26,0x93,0xc9,0x64,0x32,0x99,0x4c,0x26,0x73,0x11,0xa1,0x06,0xc5,0x00};
    sink_t plain = {0};
    v44_decoder_t *dec = v44_decoder_init(512,32,1024,output,&plain);
    CHECK(dec != NULL,"captured peer initialization");
    if (!dec) return;
    for (size_t i = 0; i < sizeof(wire); i++) CHECK(v44_decoder_feed(dec,wire+i,1) == 0,"CX93001 fragmented stream");
    CHECK(plain.n == 521 && memcmp(plain.bytes,"cx-v44-",7) == 0
          && plain.bytes[519] == '\r' && plain.bytes[520] == '\n',"CX93001 banner and framing");
    for (int i = 7; i < 519; i++) CHECK(plain.bytes[i] == 'A',"CX93001 overlapping run");
    v44_decoder_free(dec);
}
static void errors(void)
{
    sink_t plain = {0};
    CHECK(!v44_encoder_init(255,32,1024,output,&plain),"reject too few codewords");
    CHECK(!v44_decoder_init(512,31,1024,output,&plain),"reject too short strings");
    CHECK(!v44_decoder_init(512,32,511,output,&plain),"reject too short history");
    v44_decoder_t *dec = v44_decoder_init(512,32,1024,output,&plain);
    uint8_t invalid = 21; /* prefix 1, codeword 10 > initial C1=4 */
    CHECK(dec && v44_decoder_feed(dec,&invalid,1) == -1 && plain.n == 0,"missing dictionary entry is an error");
    if (dec) {
        CHECK(v44_decoder_feed(dec,&invalid,1) == -1,"failed decoder stays failed until C-INIT");
        v44_decoder_free(dec);
    }
    /* Deterministic malformed streams under sanitizers: no out-of-bounds copies. */
    unsigned seed = 1234;
    for (int i = 0; i < 1000; i++) {
        plain.n = 0;
        dec = v44_decoder_init(256,32,512,output,&plain);
        if (!dec) { CHECK(false,"fuzz initialization"); return; }
        for (int j = 0; j < 256; j++) {
            seed = seed * 1664525u + 1013904223u;
            uint8_t b = (uint8_t)(seed >> 24);
            if (v44_decoder_feed(dec,&b,1) != 0) break;
        }
        CHECK(!plain.overflow,"malformed stream bounded output");
        v44_decoder_free(dec);
    }
}
int main(void)
{
    static const int sizes[][3] = {{256,32,512},{512,32,1024},{768,48,2048},{2048,64,4096},{65535,255,65535}};
    for (size_t i = 0; i < sizeof(sizes)/sizeof(sizes[0]); i++) {
        roundtrip(sizes[i][0],sizes[i][1],sizes[i][2],false);
        roundtrip(sizes[i][0],sizes[i][1],sizes[i][2],true);
    }
    extensions(); captured_peer(); errors();
    if (failures) { fprintf(stderr,"v44_test: %d failures\n",failures); return 1; }
    puts("v44_test: OK (roundtrips, 253 extension lengths, CX93001 capture, malformed streams)");
    return 0;
}
