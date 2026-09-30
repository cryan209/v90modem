/* V.44 stream method; independent Python reference: modem-dsp-emu/tools/v44.py.
 * Wire prefixes: Table 5; initialization: 7.5; STEPUP/REINIT: 7.11/7.12;
 * FLUSH: 7.13. Dictionaries live across flushes, never across C-INIT. */
#include "v44.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

enum { ETM, FLUSH, STEPUP, REINIT, FIRST = 4 };
enum { NONE, ORDINAL, CODEWORD, EXTENSION };
typedef struct {
    v44_output_fn fn;
    void *ctx;
    uint8_t bytes[256];
    int len;
} output_t;
static void emit(output_t *o, uint8_t b)
{
    o->bytes[o->len++] = b;
    if (o->len == (int)sizeof(o->bytes)) {
        o->fn(o->ctx, o->bytes, o->len);
        o->len = 0;
    }
}
static void drain(output_t *o)
{
    if (o->len) {
        o->fn(o->ctx, o->bytes, o->len);
        o->len = 0;
    }
}
static bool limits(int n2, int n7, int n8)
{
    return n2 >= 256 && n2 <= 65535 && n7 >= 32 && n7 <= 255
           && n8 >= 512 && n8 <= 65535;
}
static int max_width(int n2)
{
    int w = 0;
    for (unsigned n = (unsigned)n2 - 1; n; n >>= 1)
        w++;
    return w;
}
struct v44_encoder_s {
    int n2, n7, n8, max_width, next, width, ordinal_width, history_count;
    uint32_t bits, threshold;
    int bit_count, previous_kind, previous_len;
    uint8_t previous[255];
    bool wire_codeword;
    uint8_t *strings;
    uint16_t *lengths;
    uint32_t *slots;
    size_t slot_count;
    output_t output;
};
static void encoder_dictionary(v44_encoder_t *s)
{
    memset(s->slots, 0, s->slot_count * sizeof(*s->slots));
    s->next = FIRST;
    s->width = 6;
    s->threshold = 64;
    s->ordinal_width = 7;
    s->history_count = 0;
    s->previous_kind = NONE;
    s->previous_len = 0;
    s->wire_codeword = false;
}
void v44_encoder_reset(v44_encoder_t *s)
{
    if (!s) return;
    s->bits = 0;
    s->bit_count = 0;
    s->output.len = 0;
    encoder_dictionary(s);
}
void v44_encoder_free(v44_encoder_t *s)
{
    if (!s) return;
    free(s->strings);
    free(s->lengths);
    free(s->slots);
    free(s);
}
v44_encoder_t *v44_encoder_init(int n2, int n7, int n8, v44_output_fn fn, void *ctx)
{
    if (!limits(n2, n7, n8) || !fn) return NULL;
    v44_encoder_t *s = calloc(1, sizeof(*s));
    if (!s) return NULL;
    s->n2 = n2; s->n7 = n7; s->n8 = n8; s->max_width = max_width(n2);
    s->output.fn = fn; s->output.ctx = ctx;
    for (s->slot_count = 1; s->slot_count < 2u * (unsigned)n2; s->slot_count <<= 1) {}
    s->strings = malloc((size_t)n2 * (size_t)n7);
    s->lengths = calloc((size_t)n2, sizeof(*s->lengths));
    s->slots = calloc(s->slot_count, sizeof(*s->slots));
    if (!s->strings || !s->lengths || !s->slots) {
        v44_encoder_free(s);
        return NULL;
    }
    v44_encoder_reset(s);
    return s;
}
static uint32_t string_hash(const uint8_t *p, int n)
{
    uint32_t h = 2166136261u;
    for (int i = 0; i < n; i++) h = (h ^ p[i]) * 16777619u;
    return h;
}
static size_t hash_slot_value(v44_encoder_t *s, const uint8_t *p, int n, uint32_t hash)
{
    size_t i = hash & (s->slot_count - 1);
    while (s->slots[i]) {
        unsigned c = s->slots[i];
        if (s->lengths[c] == n && memcmp(s->strings + (size_t)c * s->n7, p, (size_t)n) == 0)
            break;
        i = (i + 1) & (s->slot_count - 1);
    }
    return i;
}
static void add_string(v44_encoder_t *s, const uint8_t *p, int n)
{
    if (n > s->n7 || s->next >= s->n2) return;
    size_t i = hash_slot_value(s, p, n, string_hash(p, n));
    int c = s->next++;
    memcpy(s->strings + (size_t)c * s->n7, p, (size_t)n);
    s->lengths[c] = (uint16_t)n;
    s->slots[i] = (unsigned)c;
}
static void put_bits(v44_encoder_t *s, unsigned value, int width)
{
    s->bits |= value << s->bit_count;
    s->bit_count += width;
    while (s->bit_count >= 8) {
        emit(&s->output, (uint8_t)s->bits);
        s->bits >>= 8;
        s->bit_count -= 8;
    }
}
static void encoder_control(v44_encoder_t *s, int value)
{
    put_bits(s, 1, 1);
    put_bits(s, (unsigned)value, s->width);
    s->wire_codeword = false;
}
static void record(v44_encoder_t *s, int kind, const uint8_t *p, int n)
{
    if (s->previous_kind == ORDINAL || s->previous_kind == CODEWORD) {
        uint8_t added[256];
        memcpy(added, s->previous, (size_t)s->previous_len);
        added[s->previous_len] = p[0];
        add_string(s, added, s->previous_len + 1);
    }
    memcpy(s->previous, p, (size_t)n);
    s->previous_len = n;
    s->previous_kind = kind;
}
int v44_encoder_feed(v44_encoder_t *s, const uint8_t *p, size_t n)
{
    if (!s || (!p && n)) return -1;
    for (size_t pos = 0; pos < n;) {
        if (s->next >= s->n2 || s->history_count >= s->n8) {
            encoder_control(s, REINIT);
            encoder_dictionary(s);
        }
        int limit = s->n7;
        if ((size_t)limit > n - pos) limit = (int)(n - pos);
        if (limit > s->n8 - s->history_count) limit = s->n8 - s->history_count;
        unsigned c = 0;
        int length = 1;
        uint32_t hash = 2166136261u;
        /* One rolling hash pass avoids quadratic work on incompressible input. */
        for (int n = 1; n <= limit; n++) {
            hash = (hash ^ p[pos + (size_t)n - 1]) * 16777619u;
            if (n > 1) {
                unsigned match = s->slots[hash_slot_value(s, p + pos, n, hash)];
                if (match) { c = match; length = n; }
            }
        }
        if (c) {
            while (c >= s->threshold) {
                if (s->width >= s->max_width) return -1;
                encoder_control(s, STEPUP);
                s->width++;
                s->threshold <<= 1;
            }
            put_bits(s, 1, 1);
            put_bits(s, c, s->width);
            s->wire_codeword = true;
            record(s, CODEWORD, p + pos, length);
        } else {
            length = 1;
            if (p[pos] >= 128 && s->ordinal_width == 7) {
                encoder_control(s, STEPUP);
                s->ordinal_width = 8;
            }
            put_bits(s, 0, 1);
            if (s->wire_codeword) put_bits(s, 0, 1);
            put_bits(s, p[pos], s->ordinal_width);
            s->wire_codeword = false;
            record(s, ORDINAL, p + pos, 1);
        }
        pos += (size_t)length;
        s->history_count += length;
    }
    drain(&s->output);
    return 0;
}
int v44_encoder_flush(v44_encoder_t *s)
{
    if (!s) return -1;
    encoder_control(s, FLUSH);
    if (s->bit_count) emit(&s->output, (uint8_t)s->bits);
    s->bits = 0;
    s->bit_count = 0;
    drain(&s->output);
    return 0;
}

typedef struct { int end, length; } entry_t;
struct v44_decoder_s {
    int n2, n7, n8, max_width, next, width, ordinal_width;
    int history_count, bit_count, previous_kind, previous_len, previous_code;
    uint32_t bits;
    bool compressed, escaped, wire_codeword, pending_stepup, failed;
    uint8_t escape, previous[255];
    uint8_t *history;
    entry_t *entries;
    output_t output;
};
static void decoder_dictionary(v44_decoder_t *s)
{
    s->next = FIRST;
    s->width = 6;
    s->ordinal_width = 7;
    s->history_count = 0;
    s->previous_kind = NONE;
    s->previous_len = 0;
    s->previous_code = -1;
    s->wire_codeword = s->pending_stepup = false;
}
void v44_decoder_reset(v44_decoder_t *s)
{
    if (!s) return;
    s->bits = 0;
    s->bit_count = s->output.len = 0;
    s->compressed = true;
    s->escaped = s->failed = false;
    s->escape = 0;
    decoder_dictionary(s);
}
void v44_decoder_free(v44_decoder_t *s)
{
    if (!s) return;
    free(s->history);
    free(s->entries);
    free(s);
}
v44_decoder_t *v44_decoder_init(int n2, int n7, int n8, v44_output_fn fn, void *ctx)
{
    if (!limits(n2, n7, n8) || !fn) return NULL;
    v44_decoder_t *s = calloc(1, sizeof(*s));
    if (!s) return NULL;
    s->n2 = n2; s->n7 = n7; s->n8 = n8; s->max_width = max_width(n2);
    s->output.fn = fn; s->output.ctx = ctx;
    s->history = malloc((size_t)n8);
    s->entries = calloc((size_t)n2, sizeof(*s->entries));
    if (!s->history || !s->entries) {
        v44_decoder_free(s);
        return NULL;
    }
    v44_decoder_reset(s);
    return s;
}
static unsigned peek(v44_decoder_t *s, int off, int width)
{
    return (s->bits >> off) & ((1u << width) - 1);
}
static void consume(v44_decoder_t *s, int n)
{
    s->bits >>= n;
    s->bit_count -= n;
}
static int decoder_add(v44_decoder_t *s, int length, int end)
{
    if (length > s->n7) return 0;
    if (s->next >= s->n2 || end < length - 1 || end >= s->history_count) return -1;
    s->entries[s->next++] = (entry_t){end, length};
    return 0;
}
static int decoded_string(v44_decoder_t *s, int kind, int code, const uint8_t *p, int n)
{
    if (n < 1 || n > s->n7 || n > s->n8 - s->history_count) return -1;
    int start = s->history_count;
    memcpy(s->history + start, p, (size_t)n);
    s->history_count += n;
    if ((s->previous_kind == ORDINAL || s->previous_kind == CODEWORD)
        && decoder_add(s, s->previous_len + 1, start) != 0) return -1;
    for (int i = 0; i < n; i++) emit(&s->output, p[i]);
    memcpy(s->previous, p, (size_t)n);
    s->previous_len = n;
    s->previous_kind = kind;
    s->previous_code = code;
    s->wire_codeword = kind == CODEWORD;
    return 0;
}
static int decoder_codeword(v44_decoder_t *s, unsigned code)
{
    uint8_t current[256];
    int n;
    if (code > (unsigned)s->next || code >= (unsigned)s->n2) return -1;
    if (code == (unsigned)s->next) {
        if (s->previous_kind != ORDINAL && s->previous_kind != CODEWORD) return -1;
        n = s->previous_len + 1;
        if (n > s->n7) return -1;
        memcpy(current, s->previous, (size_t)s->previous_len);
        current[n - 1] = s->previous[0];
    } else {
        entry_t e = s->entries[code];
        n = e.length;
        if (n < 1 || e.end < n - 1 || e.end >= s->history_count) return -1;
        memcpy(current, s->history + e.end - n + 1, (size_t)n);
    }
    return decoded_string(s, CODEWORD, (int)code, current, n);
}
static int decoder_extension(v44_decoder_t *s, int length)
{
    if (s->previous_kind != CODEWORD || s->previous_code < FIRST
        || s->previous_code >= s->next || length < 1
        || length > s->n7 - s->previous_len || length > s->n8 - s->history_count) return -1;
    int source = s->entries[s->previous_code].end + 1;
    int start = s->history_count;
    /* 6.4.1: copy incrementally; source can overlap newly appended output. */
    for (int i = 0; i < length; i++) {
        if (source + i >= s->history_count) return -1;
        uint8_t b = s->history[source + i];
        s->history[s->history_count++] = b;
        emit(&s->output, b);
    }
    if (decoder_add(s, s->previous_len + length, start + length - 1) != 0) return -1;
    s->previous_kind = EXTENSION;
    s->previous_len = 0;
    s->previous_code = -1;
    s->wire_codeword = false;
    return 0;
}
/* 6.6.2/Tables 3 and 4: return 0 for incomplete input, 1 for a complete length. */
static int extension_length(v44_decoder_t *s, int *length, int *width)
{
    if (s->bit_count < 3) return 0;
    if (peek(s, 2, 1)) { *length = 1; *width = 1; return 1; }
    if (s->bit_count < 5) return 0;
    unsigned short_length = peek(s, 3, 2);
    if (short_length) { *length = (int)short_length + 1; *width = 3; return 1; }
    if (s->bit_count < 6) return 0;
    if (!peek(s, 5, 1)) {
        if (s->bit_count < 9) return 0;
        *length = 5 + (int)peek(s, 6, 3); *width = 7; return 1;
    }
    int n = s->n7 <= 46 ? 5 : s->n7 <= 78 ? 6 : s->n7 <= 142 ? 7 : 8;
    if (s->bit_count < 6 + n) return 0;
    *length = 13 + (int)peek(s, 6, n); *width = 4 + n;
    return 1;
}
static int decoder_drain(v44_decoder_t *s)
{
    while (s->compressed && s->bit_count) {
        unsigned first = peek(s, 0, 1);
        if (s->pending_stepup) {
            int width = (first ? s->width : s->ordinal_width) + 1;
            if ((first && width > s->max_width) || (!first && width > 8)) return -1;
            if (s->bit_count < 1 + width) return 0;
            unsigned value = peek(s, 1, width);
            consume(s, 1 + width);
            s->pending_stepup = false;
            if (first) {
                s->width = width;
                /* 7.11.2(d) permits repeated STEPUPs before a large codeword. */
                if (value == STEPUP) s->pending_stepup = true;
                else if (value < FIRST || decoder_codeword(s, value) != 0) return -1;
            } else {
                s->ordinal_width = width;
                uint8_t b = (uint8_t)value;
                if (decoded_string(s, ORDINAL, -1, &b, 1) != 0) return -1;
            }
            continue;
        }
        if (first) {
            if (s->bit_count < 1 + s->width) return 0;
            unsigned value = peek(s, 1, s->width);
            consume(s, 1 + s->width);
            if (value >= FIRST) {
                if (decoder_codeword(s, value) != 0) return -1;
            } else {
                s->wire_codeword = false;
                switch (value) {
                case STEPUP: s->pending_stepup = true; break;
                case FLUSH: s->bits = 0; s->bit_count = 0; break;
                case REINIT: decoder_dictionary(s); break;
                case ETM: s->bits = 0; s->bit_count = 0; s->compressed = false; break;
                }
            }
            continue;
        }
        int prefix = 1;
        if (s->wire_codeword) {
            if (s->bit_count < 2) return 0;
            if (peek(s, 1, 1)) {
                int length, width;
                if (!extension_length(s, &length, &width)) return 0;
                if (length > 253 || length > s->n7 - 2) return -1;
                consume(s, 2 + width);
                if (decoder_extension(s, length) != 0) return -1;
                continue;
            }
            prefix = 2;
        }
        if (s->bit_count < prefix + s->ordinal_width) return 0;
        uint8_t b = (uint8_t)peek(s, prefix, s->ordinal_width);
        consume(s, prefix + s->ordinal_width);
        if (decoded_string(s, ORDINAL, -1, &b, 1) != 0) return -1;
    }
    return 0;
}
int v44_decoder_feed(v44_decoder_t *s, const uint8_t *p, size_t n)
{
    if (!s || s->failed || (!p && n)) return -1;
    for (size_t i = 0; i < n; i++) {
        if (!s->compressed) {
            if (s->escaped) {
                s->escaped = false;
                switch (p[i]) {
                case 0: decoder_dictionary(s); s->compressed = true; break; /* ECM */
                case 1: emit(&s->output, s->escape); s->escape = (uint8_t)(s->escape + 51); break;
                default: s->failed = true; return -1; /* EPM not advertised */
                }
            } else if (p[i] == s->escape) s->escaped = true;
            else emit(&s->output, p[i]);
            continue;
        }
        s->bits |= (uint32_t)p[i] << s->bit_count;
        s->bit_count += 8;
        if (decoder_drain(s) != 0) { s->failed = true; return -1; }
    }
    drain(&s->output);
    return 0;
}
