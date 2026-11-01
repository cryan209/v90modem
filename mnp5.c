/* MNP5 AFE/RLE codec. See mnp5.h and docs/mnp.md. No firmware code or
 * tables are linked into this implementation. The rank code is constructed
 * arithmetically; HSF is used only as an independent wire-format oracle. */
#include "mnp5.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

struct mnp5_s {
    mnp5_output_fn output;
    void *user;
    uint8_t symbol[256], rank[256], frequency[256];
    unsigned bits, nbits;
    unsigned run;
    uint8_t previous;
    bool dirty, failed;
};

void mnp5_restart(mnp5_t *s)
{
    mnp5_output_fn output = s->output;
    void *user = s->user;
    memset(s, 0, sizeof(*s));
    s->output = output;
    s->user = user;
    for (unsigned i = 0; i < 256; i++)
        s->symbol[i] = s->rank[i] = (uint8_t)i;
}

mnp5_t *mnp5_init(mnp5_output_fn output, void *user)
{
    if (!output) return NULL;
    mnp5_t *s = calloc(1, sizeof(*s));
    if (!s) return NULL;
    s->output = output;
    s->user = user;
    mnp5_restart(s);
    return s;
}

void mnp5_free(mnp5_t *s) { free(s); }

/* Frequencies belong to ranks. Promote to the FIRST equal-frequency rank,
 * then increment. Halve BEFORE updating when the leading frequency is 255,
 * rounding DOWN. Run-count and end-marker codes never update this table. */
static void update(mnp5_t *s, unsigned r)
{
    if (s->frequency[0] == 255)
        for (unsigned i = 0; i < 256; i++) s->frequency[i] >>= 1;
    unsigned b = s->symbol[r];
    while (r && s->frequency[r - 1] == s->frequency[r]) {
        unsigned previous = s->symbol[r - 1];
        s->symbol[r] = (uint8_t)previous;
        s->rank[previous] = (uint8_t)r;
        r--;
    }
    s->symbol[r] = (uint8_t)b;
    s->rank[b] = (uint8_t)r;
    s->frequency[r]++;
}

/* 3-bit length prefix followed by that many rank-offset bits; ranks 0/1
 * use one offset bit too. The top leaf splits into rank 255 and marker 256.
 * Codes are packed MOST-significant bit first (unlike V.42bis). */
static unsigned code(unsigned r, unsigned *width)
{
    if (r >= 255) { *width = 11; return r == 255 ? 0x7fe : 0x7ff; }
    unsigned group = 0;
    for (unsigned n = r; n >= 2; n >>= 1) group++;
    unsigned offset = group ? group : 1;
    *width = 3 + offset;
    return (group << offset) | (group ? r - (1U << group) : r);
}

static void emit(mnp5_t *s, unsigned r)
{
    unsigned width;
    unsigned c = code(r, &width);
    s->bits = (s->bits << width) | c;
    s->nbits += width;
    while (s->nbits >= 8) {
        s->nbits -= 8;
        uint8_t b = (uint8_t)(s->bits >> s->nbits);
        s->output(s->user, &b, 1);
    }
    s->bits &= (1U << s->nbits) - 1;
}

int mnp5_encode(mnp5_t *s, const uint8_t *data, size_t len)
{
    if (!s || s->failed || (!data && len)) return -1;
    for (size_t i = 0; i < len; i++) {
        unsigned b = data[i];
        s->dirty = true;
        if (s->run && b != s->previous) {
            if (s->run >= 3) emit(s, s->run - 3);
            s->run = 0;
        }
        s->previous = (uint8_t)b;
        s->run++;
        if (s->run <= 3) {
            unsigned r = s->rank[b];
            emit(s, r);
            update(s, r);
        } else if (s->run == 253) {
            emit(s, 250);
            s->run = 0;
        }
    }
    return 0;
}

int mnp5_flush(mnp5_t *s)
{
    if (!s || s->failed) return -1;
    if (!s->dirty) return 0;
    if (s->run >= 3) emit(s, s->run - 3);
    emit(s, 256);
    if (s->nbits) {
        uint8_t b = (uint8_t)((s->bits << (8 - s->nbits)) | ((1U << (8 - s->nbits)) - 1));
        s->output(s->user, &b, 1);
    }
    s->bits = s->nbits = s->run = 0;
    s->dirty = false;
    return 0;
}

int mnp5_decode(mnp5_t *s, const uint8_t *data, size_t len)
{
    if (!s || s->failed || (!data && len)) return -1;
    for (size_t i = 0; i < len; i++) {
        s->bits = (s->bits << 8) | data[i];
        s->nbits += 8;
        while (s->nbits >= 4) {
            unsigned group = s->bits >> (s->nbits - 3);
            unsigned offset = group ? group : 1;
            unsigned width = 3 + offset;
            if (s->nbits < width) break;
            unsigned low = (s->bits >> (s->nbits - width)) & ((1U << offset) - 1);
            unsigned r = group ? (1U << group) + low : low;
            if (r == 255) {
                if (s->nbits < 11) break;
                width = 11;
                r += (s->bits >> (s->nbits - width)) & 1;
            }
            s->nbits -= width;
            s->bits &= (1U << s->nbits) - 1;
            if (r == 256) {
                /* Discard the marker's pad bits, not dictionary state. */
                s->bits = s->nbits = s->run = 0;
                break;
            }
            if (s->run == 3) {
                if (r > 250) { s->failed = true; return -1; }
                uint8_t repeat[250];
                memset(repeat, s->previous, r);
                if (r) s->output(s->user, repeat, (int)r);
                s->run = 0;
            } else {
                uint8_t b = s->symbol[r];
                update(s, r);
                s->output(s->user, &b, 1);
                s->run = s->run && b == s->previous ? s->run + 1 : 1;
                s->previous = b;
            }
        }
    }
    return 0;
}
