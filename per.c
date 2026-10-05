/*
 * per.c -- ASN.1 aligned PER (ITU-T X.691) interpreter.  See per.h.
 * Clause numbers are X.691's.
 */

#include "per.h"

#include <string.h>

const a_type_t T_BOOLEAN = { A_BOOL, 0, 0, 0, 0, 0, 0, 0, 0, 0, "BOOLEAN" };
const a_type_t T_NULL = { A_NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, "NULL" };

static const char *g_err;

const char *a_error(void)
{
    return g_err ? g_err : "no error";
}

static int fail(const char *what)
{
    if (!g_err)
        g_err = what;
    return -1;
}

const a_type_t *a_find_type(const char *name)
{
    int i;

    for (i = 0; h245_types[i].name; i++)
        if (strcmp(h245_types[i].name, name) == 0)
            return h245_types[i].type;
    return NULL;
}

/* ------------------------------------------------------------------------ *
 * Arena and value construction
 * ------------------------------------------------------------------------ */

void a_arena_init(a_arena_t *a, void *buf, size_t size)
{
    a->buf = buf;
    a->size = size;
    a->used = 0;
}

static void *a_alloc(a_arena_t *a, size_t n)
{
    void *p;

    n = (n + 7u) & ~(size_t)7u;
    if (a->used + n > a->size) {
        fail("arena exhausted");
        return NULL;
    }
    p = a->buf + a->used;
    a->used += n;
    memset(p, 0, n);
    return p;
}

a_val_t *a_new(a_arena_t *a, const a_type_t *t)
{
    a_val_t *v = a_alloc(a, sizeof(*v));

    if (!v)
        return NULL;
    v->type = t;
    if (t->kind == A_SEQ && t->n_total) {
        v->kids = a_alloc(a, sizeof(a_val_t *) * t->n_total);
        if (!v->kids)
            return NULL;
    } else if (t->kind == A_CHOICE) {
        v->kids = a_alloc(a, sizeof(a_val_t *));
        if (!v->kids)
            return NULL;
    }
    return v;
}

static int member_index(const a_type_t *t, const char *name)
{
    int i;

    for (i = 0; i < t->n_total; i++)
        if (strcmp(t->members[i].name, name) == 0)
            return i;
    return -1;
}

a_val_t *a_member(a_arena_t *a, a_val_t *seq, const char *name)
{
    int i;

    if (!seq || seq->type->kind != A_SEQ)
        return NULL;
    i = member_index(seq->type, name);
    if (i < 0 || !seq->type->members[i].type)
        return NULL;
    if (!seq->kids[i])
        seq->kids[i] = a_new(a, seq->type->members[i].type);
    return seq->kids[i];
}

a_val_t *a_choice(a_arena_t *a, a_val_t *ch, const char *alt)
{
    int i;

    if (!ch || ch->type->kind != A_CHOICE)
        return NULL;
    i = member_index(ch->type, alt);
    if (i < 0 || !ch->type->members[i].type)
        return NULL;
    ch->i = i;
    ch->kids[0] = a_new(a, ch->type->members[i].type);
    return ch->kids[0];
}

a_val_t *a_choice_at(a_arena_t *a, a_val_t *ch, int idx)
{
    if (!ch || ch->type->kind != A_CHOICE || idx < 0 || idx >= ch->type->n_total ||
        !ch->type->members[idx].type)
        return NULL;
    ch->i = idx;
    ch->kids[0] = a_new(a, ch->type->members[idx].type);
    return ch->kids[0];
}

int a_choice_index(const a_val_t *ch)
{
    if (!ch || ch->type->kind != A_CHOICE || !ch->kids || !ch->kids[0])
        return -1;
    return (int)ch->i;
}

a_val_t *a_append(a_arena_t *a, a_val_t *s)
{
    a_val_t *e;

    if (!s || s->type->kind != A_SEQOF || !s->type->elem)
        return NULL;
    if (s->n >= s->len) {
        int cap = s->len ? s->len * 2 : 8;
        a_val_t **k = a_alloc(a, sizeof(a_val_t *) * (size_t)cap);

        if (!k)
            return NULL;
        if (s->n)
            memcpy(k, s->kids, sizeof(a_val_t *) * (size_t)s->n);
        s->kids = k;
        s->len = cap;
    }
    e = a_new(a, s->type->elem);
    if (!e)
        return NULL;
    s->kids[s->n++] = e;
    return e;
}

int a_set_int(a_val_t *v, int64_t x)
{
    if (!v || v->type->kind != A_INT)
        return -1;
    if ((v->type->has_lb && x < v->type->lb) || (v->type->has_ub && x > v->type->ub))
        return fail("INTEGER out of range");
    v->i = x;
    return 0;
}

int a_set_bool(a_val_t *v, int x)
{
    if (!v || v->type->kind != A_BOOL)
        return -1;
    v->i = x != 0;
    return 0;
}

int a_set_octets(a_arena_t *a, a_val_t *v, const uint8_t *p, int n)
{
    if (!v || v->type->kind != A_OCTETS || n < 0)
        return -1;
    if ((v->type->has_lb && n < v->type->lb) || (v->type->has_ub && n > v->type->ub))
        return fail("OCTET STRING size out of range");
    v->data = a_alloc(a, (size_t)n + 1);
    if (!v->data)
        return -1;
    if (n)
        memcpy(v->data, p, (size_t)n);
    v->len = n;
    return 0;
}

int a_set_oid(a_arena_t *a, a_val_t *v, const uint32_t *arcs, int n)
{
    uint8_t tmp[64];
    int len = 0, i;

    if (!v || v->type->kind != A_OID || n < 2 || arcs[0] > 2)
        return -1;
    /* X.690 8.19: the first two arcs share one subidentifier. */
    for (i = 1; i < n; i++) {
        uint32_t x = i == 1 ? arcs[0] * 40u + arcs[1] : arcs[i];
        uint8_t b[5];
        int nb = 0, k;

        do {
            b[nb++] = (uint8_t)(x & 0x7F);
            x >>= 7;
        } while (x);
        for (k = nb - 1; k >= 0; k--) {
            if (len >= (int)sizeof(tmp))
                return fail("OID too long");
            tmp[len++] = (uint8_t)(b[k] | (k ? 0x80 : 0));
        }
    }
    v->data = a_alloc(a, (size_t)len + 1);
    if (!v->data)
        return -1;
    memcpy(v->data, tmp, (size_t)len);
    v->len = len;
    return 0;
}

const a_val_t *a_get(const a_val_t *seq, const char *name)
{
    int i;

    if (!seq || seq->type->kind != A_SEQ)
        return NULL;
    i = member_index(seq->type, name);
    return i < 0 ? NULL : seq->kids[i];
}

const char *a_choice_name(const a_val_t *ch)
{
    if (!ch || ch->type->kind != A_CHOICE || !ch->kids || !ch->kids[0])
        return NULL;
    return ch->type->members[ch->i].name;
}

const a_val_t *a_choice_val(const a_val_t *ch)
{
    return (ch && ch->type->kind == A_CHOICE && ch->kids) ? ch->kids[0] : NULL;
}

int a_oid_arcs(const a_val_t *v, uint32_t *arcs, int max)
{
    int n = 0, i = 0;
    uint32_t acc = 0;

    if (!v || v->type->kind != A_OID || v->len < 1)
        return -1;
    for (i = 0; i < v->len; i++) {
        acc = (acc << 7) | (v->data[i] & 0x7Fu);
        if (!(v->data[i] & 0x80)) {
            if (n == 0) {
                if (max < 2)
                    return -1;
                arcs[0] = acc >= 80 ? 2 : acc / 40;
                arcs[1] = acc - arcs[0] * 40;
                n = 2;
            } else {
                if (n >= max)
                    return -1;
                arcs[n++] = acc;
            }
            acc = 0;
        }
    }
    return n;
}

/* ------------------------------------------------------------------------ *
 * Encoder
 * ------------------------------------------------------------------------ */

typedef struct {
    uint8_t *p;
    int max;                    /* bytes */
    int nbits;
} bw_t;

static int bw_bit(bw_t *w, int b)
{
    int byte = w->nbits >> 3;

    if (byte >= w->max)
        return fail("output buffer full");
    if ((w->nbits & 7) == 0)
        w->p[byte] = 0;
    if (b)
        w->p[byte] |= (uint8_t)(0x80 >> (w->nbits & 7));
    w->nbits++;
    return 0;
}

static int bw_bits(bw_t *w, uint64_t v, int n)
{
    int i;

    for (i = n - 1; i >= 0; i--)
        if (bw_bit(w, (int)((v >> i) & 1)) < 0)
            return -1;
    return 0;
}

static int bw_align(bw_t *w)
{
    while (w->nbits & 7)
        if (bw_bit(w, 0) < 0)
            return -1;
    return 0;
}

static int bw_octets(bw_t *w, const uint8_t *p, int n)
{
    int i;

    if (bw_align(w) < 0)
        return -1;
    for (i = 0; i < n; i++)
        if (bw_bits(w, p[i], 8) < 0)
            return -1;
    return 0;
}

static int bits_for(uint64_t maxval)
{
    int n = 0;

    while (maxval) {
        n++;
        maxval >>= 1;
    }
    return n;
}

static int bytes_for(uint64_t v)
{
    int n = 1;

    while (v >>= 8)
        n++;
    return n;
}

/* 10.5: a whole number n in 0..range-1 (the value less the lower bound). */
static int enc_cwn(bw_t *w, uint64_t n, uint64_t range)
{
    if (range == 1)
        return 0;
    if (range <= 255)
        return bw_bits(w, n, bits_for(range - 1));
    if (range <= 65536) {
        if (bw_align(w) < 0)
            return -1;
        return bw_bits(w, n, range == 256 ? 8 : 16);
    }
    {   /* 10.5.7 d): indefinite length case.  The length is a bit-field that is
         * NOT aligned; only the value octets that follow are. */
        int nb = bytes_for(n), maxb = bytes_for(range - 1), i;

        if (enc_cwn(w, (uint64_t)(nb - 1), (uint64_t)maxb) < 0 || bw_align(w) < 0)
            return -1;
        for (i = nb - 1; i >= 0; i--)
            if (bw_bits(w, (n >> (8 * i)) & 0xFF, 8) < 0)
                return -1;
    }
    return 0;
}

/* 10.9: a length determinant, unfragmented. */
static int enc_len(bw_t *w, int64_t n, int has_ub, int64_t lb, int64_t ub)
{
    if (has_ub && ub < 65536) {
        if (n < lb || n > ub)
            return fail("length outside its constraint");
        return enc_cwn(w, (uint64_t)(n - lb), (uint64_t)(ub - lb + 1));
    }
    if (bw_align(w) < 0)
        return -1;
    if (n < 128)
        return bw_bits(w, (uint64_t)n, 8);
    if (n < 16384)
        return bw_bits(w, 0x8000u | (uint64_t)n, 16);
    return fail("fragmentation (>= 16K) not supported");
}

/* 10.6: normally small non-negative whole number. */
static int enc_nsn(bw_t *w, uint64_t n)
{
    int nb, i;

    if (n < 64)
        return bw_bit(w, 0) < 0 ? -1 : bw_bits(w, n, 6);
    if (bw_bit(w, 1) < 0)
        return -1;
    nb = bytes_for(n);
    if (enc_len(w, nb, 0, 0, 0) < 0)
        return -1;
    for (i = nb - 1; i >= 0; i--)
        if (bw_bits(w, (n >> (8 * i)) & 0xFF, 8) < 0)
            return -1;
    return 0;
}

static int enc_val(bw_t *w, const a_val_t *v);

/* 10.2: an open type is a complete encoding behind a length determinant. */
static int enc_open(bw_t *w, const a_val_t *v)
{
    uint8_t tmp[4096];
    bw_t t = { tmp, (int)sizeof(tmp), 0 };
    int n;

    if (enc_val(&t, v) < 0)
        return -1;
    n = (t.nbits + 7) / 8;
    if (n == 0) {
        tmp[0] = 0;                              /* 10.1.3: never empty */
        n = 1;
    }
    if (enc_len(w, n, 0, 0, 0) < 0)
        return -1;
    return bw_octets(w, tmp, n);
}

static int enc_val(bw_t *w, const a_val_t *v)
{
    const a_type_t *t = v->type;
    int i;

    switch (t->kind) {
    case A_BOOL:
        return bw_bit(w, (int)v->i);
    case A_NULL:
        return 0;
    case A_INT:
        if (!t->has_lb)
            return fail("unconstrained INTEGER not supported");
        if (v->i < t->lb || (t->has_ub && v->i > t->ub))
            return fail("INTEGER out of range");
        if (t->has_ub)
            return enc_cwn(w, (uint64_t)(v->i - t->lb), (uint64_t)(t->ub - t->lb) + 1);
        {   /* 12.2.3 semi-constrained: non-negative-binary-integer, length first */
            uint64_t n = (uint64_t)(v->i - t->lb);
            int nb = bytes_for(n), k;

            if (enc_len(w, nb, 0, 0, 0) < 0)
                return -1;
            for (k = nb - 1; k >= 0; k--)
                if (bw_bits(w, (n >> (8 * k)) & 0xFF, 8) < 0)
                    return -1;
            return 0;
        }
    case A_OCTETS: {
        int n = v->len;

        if ((t->has_lb && n < t->lb) || (t->has_ub && n > t->ub))
            return fail("OCTET STRING size outside its constraint");
        if (t->has_lb && t->has_ub && t->lb == t->ub) {
            if (t->ub <= 2) {                       /* 16.6: not octet-aligned */
                for (i = 0; i < n; i++)
                    if (bw_bits(w, v->data[i], 8) < 0)
                        return -1;
                return 0;
            }
            if (t->ub < 65536)
                return bw_octets(w, v->data, n);
            return fail("fixed OCTET STRING >= 64K not supported");
        }
        if (enc_len(w, n, t->has_ub, t->has_lb ? t->lb : 0, t->has_ub ? t->ub : 0) < 0)
            return -1;
        return n ? bw_octets(w, v->data, n) : 0;
    }
    case A_OID:
        if (enc_len(w, v->len, 0, 0, 0) < 0)
            return -1;
        return bw_octets(w, v->data, v->len);
    case A_SEQOF: {
        int n = v->n;

        if (!t->elem)
            return fail("SEQUENCE OF element type not in the subset");
        if (t->has_lb && t->has_ub && t->lb == t->ub) {
            if (n != t->lb)
                return fail("SEQUENCE OF size mismatch");
        } else if (enc_len(w, n, t->has_ub, t->has_lb ? t->lb : 0, t->has_ub ? t->ub : 0) < 0) {
            return -1;
        }
        for (i = 0; i < n; i++)
            if (enc_val(w, v->kids[i]) < 0)
                return -1;
        return 0;
    }
    case A_SEQ: {
        int ext_present = 0;

        for (i = t->n_root; i < t->n_total; i++)
            if (v->kids[i])
                ext_present = 1;
        if (t->ext && bw_bit(w, ext_present) < 0)
            return -1;
        for (i = 0; i < t->n_root; i++) {
            if (t->members[i].optional) {
                if (bw_bit(w, v->kids[i] != NULL) < 0)
                    return -1;
            } else if (!v->kids[i]) {
                return fail("mandatory SEQUENCE component missing");
            }
        }
        for (i = 0; i < t->n_root; i++) {
            if (!v->kids[i])
                continue;
            if (!t->members[i].type)
                return fail("component not in the DSVD subset");
            if (enc_val(w, v->kids[i]) < 0)
                return -1;
        }
        if (ext_present) {
            int nadd = t->n_total - t->n_root;

            if (enc_nsn(w, (uint64_t)(nadd - 1)) < 0)
                return -1;
            for (i = t->n_root; i < t->n_total; i++)
                if (bw_bit(w, v->kids[i] != NULL) < 0)
                    return -1;
            for (i = t->n_root; i < t->n_total; i++) {
                if (!v->kids[i])
                    continue;
                if (!t->members[i].type)
                    return fail("extension addition not in the DSVD subset");
                if (enc_open(w, v->kids[i]) < 0)
                    return -1;
            }
        }
        return 0;
    }
    case A_CHOICE: {
        int idx = (int)v->i;

        if (!v->kids || !v->kids[0] || idx < 0 || idx >= t->n_total)
            return fail("CHOICE has no alternative selected");
        if (!t->members[idx].type)
            return fail("CHOICE alternative not in the DSVD subset");
        if (t->ext && bw_bit(w, idx >= t->n_root) < 0)
            return -1;
        if (idx < t->n_root) {
            if (t->n_root > 1 && enc_cwn(w, (uint64_t)idx, (uint64_t)t->n_root) < 0)
                return -1;
            return enc_val(w, v->kids[0]);
        }
        if (enc_nsn(w, (uint64_t)(idx - t->n_root)) < 0)
            return -1;
        return enc_open(w, v->kids[0]);
    }
    }
    return fail("unknown type kind");
}

int a_encode(const a_val_t *v, uint8_t *out, int max)
{
    bw_t w = { out, max, 0 };
    int n;

    g_err = NULL;
    if (!v)
        return fail("no value");
    if (enc_val(&w, v) < 0)
        return -1;
    n = (w.nbits + 7) / 8;
    if (n == 0) {
        if (max < 1)
            return fail("output buffer full");
        out[0] = 0;
        return 1;
    }
    if (w.nbits & 7)
        bw_align(&w);
    return n;
}

/* ------------------------------------------------------------------------ *
 * Decoder
 * ------------------------------------------------------------------------ */

typedef struct {
    const uint8_t *p;
    int nbits;
    int pos;
} br_t;

static int br_bit(br_t *r)
{
    int b;

    if (r->pos >= r->nbits)
        return fail("truncated encoding");
    b = (r->p[r->pos >> 3] >> (7 - (r->pos & 7))) & 1;
    r->pos++;
    return b;
}

static int br_bits(br_t *r, int n, uint64_t *out)
{
    uint64_t v = 0;
    int i;

    for (i = 0; i < n; i++) {
        int b = br_bit(r);

        if (b < 0)
            return -1;
        v = (v << 1) | (uint64_t)b;
    }
    *out = v;
    return 0;
}

static int br_align(br_t *r)
{
    while (r->pos & 7) {
        int b = br_bit(r);

        if (b < 0)
            return -1;
        if (b)
            return fail("non-zero padding bits");
    }
    return 0;
}

static int dec_cwn(br_t *r, uint64_t range, uint64_t *out)
{
    if (range == 1) {
        *out = 0;
        return 0;
    }
    if (range <= 255)
        return br_bits(r, bits_for(range - 1), out);
    if (range <= 65536) {
        if (br_align(r) < 0)
            return -1;
        return br_bits(r, range == 256 ? 8 : 16, out);
    }
    {
        uint64_t nbm1, v = 0, b;
        int maxb = bytes_for(range - 1), nb, i;

        if (dec_cwn(r, (uint64_t)maxb, &nbm1) < 0 || br_align(r) < 0)
            return -1;
        nb = (int)nbm1 + 1;
        for (i = 0; i < nb; i++) {
            if (br_bits(r, 8, &b) < 0)
                return -1;
            v = (v << 8) | b;
        }
        *out = v;
    }
    return 0;
}

static int64_t dec_len(br_t *r, int has_ub, int64_t lb, int64_t ub)
{
    uint64_t v;

    if (has_ub && ub < 65536) {
        if (dec_cwn(r, (uint64_t)(ub - lb + 1), &v) < 0)
            return -1;
        return lb + (int64_t)v;
    }
    if (br_align(r) < 0 || br_bits(r, 8, &v) < 0)
        return -1;
    if (!(v & 0x80))
        return (int64_t)v;
    if ((v & 0xC0) == 0x80) {
        uint64_t lo;

        if (br_bits(r, 8, &lo) < 0)
            return -1;
        return (int64_t)(((v & 0x3F) << 8) | lo);
    }
    return fail("fragmented length not supported");
}

static int dec_nsn(br_t *r, uint64_t *out)
{
    int b = br_bit(r);

    if (b < 0)
        return -1;
    if (!b)
        return br_bits(r, 6, out);
    {
        int64_t nb = dec_len(r, 0, 0, 0);
        uint64_t v = 0, x;
        int64_t i;

        if (nb < 1 || nb > 8)
            return fail("bad normally-small length");
        for (i = 0; i < nb; i++) {
            if (br_bits(r, 8, &x) < 0)
                return -1;
            v = (v << 8) | x;
        }
        *out = v;
    }
    return 0;
}

static a_val_t *dec_val(a_arena_t *a, br_t *r, const a_type_t *t);

/* Decode an open type's contents as type t; skip it if t is NULL. */
static a_val_t *dec_open(a_arena_t *a, br_t *r, const a_type_t *t, int *skipped)
{
    int64_t n = dec_len(r, 0, 0, 0);
    br_t sub;

    if (n < 0)
        return NULL;
    if (br_align(r) < 0)
        return NULL;
    if (r->pos + n * 8 > r->nbits) {
        fail("open type longer than the encoding");
        return NULL;
    }
    sub.p = r->p + (r->pos >> 3);
    sub.nbits = (int)n * 8;
    sub.pos = 0;
    r->pos += (int)n * 8;
    if (!t) {
        *skipped = 1;
        return NULL;
    }
    return dec_val(a, &sub, t);
}

static a_val_t *dec_val(a_arena_t *a, br_t *r, const a_type_t *t)
{
    a_val_t *v = a_new(a, t);
    int i;

    if (!v)
        return NULL;
    switch (t->kind) {
    case A_BOOL: {
        int b = br_bit(r);

        if (b < 0)
            return NULL;
        v->i = b;
        return v;
    }
    case A_NULL:
        return v;
    case A_INT: {
        uint64_t x;

        if (!t->has_lb) {
            fail("unconstrained INTEGER not supported");
            return NULL;
        }
        if (t->has_ub) {
            if (dec_cwn(r, (uint64_t)(t->ub - t->lb) + 1, &x) < 0)
                return NULL;
            v->i = t->lb + (int64_t)x;
            if (v->i > t->ub) {
                fail("INTEGER out of range");
                return NULL;
            }
            return v;
        }
        {
            int64_t nb = dec_len(r, 0, 0, 0), k;
            uint64_t b;

            if (nb < 1 || nb > 8 || br_align(r) < 0)
                return NULL;
            x = 0;
            for (k = 0; k < nb; k++) {
                if (br_bits(r, 8, &b) < 0)
                    return NULL;
                x = (x << 8) | b;
            }
            v->i = t->lb + (int64_t)x;
            return v;
        }
    }
    case A_OCTETS: {
        int64_t n;
        uint64_t b;

        if (t->has_lb && t->has_ub && t->lb == t->ub) {
            n = t->lb;
            if (n > 2 && n < 65536 && br_align(r) < 0)
                return NULL;
        } else {
            n = dec_len(r, t->has_ub, t->has_lb ? t->lb : 0, t->has_ub ? t->ub : 0);
            if (n < 0)
                return NULL;
            if (n && br_align(r) < 0)
                return NULL;
        }
        if ((t->has_lb && n < t->lb) || (t->has_ub && n > t->ub)) {
            fail("OCTET STRING size outside its constraint");
            return NULL;
        }
        v->data = a_alloc(a, (size_t)n + 1);
        if (!v->data)
            return NULL;
        for (i = 0; i < n; i++) {
            if (br_bits(r, 8, &b) < 0)
                return NULL;
            v->data[i] = (uint8_t)b;
        }
        v->len = (int)n;
        return v;
    }
    case A_OID: {
        int64_t n = dec_len(r, 0, 0, 0);
        uint64_t b;

        if (n < 1 || br_align(r) < 0)
            return NULL;
        v->data = a_alloc(a, (size_t)n + 1);
        if (!v->data)
            return NULL;
        for (i = 0; i < n; i++) {
            if (br_bits(r, 8, &b) < 0)
                return NULL;
            v->data[i] = (uint8_t)b;
        }
        v->len = (int)n;
        return v;
    }
    case A_SEQOF: {
        int64_t n;

        if (!t->elem) {
            fail("SEQUENCE OF element type not in the subset");
            return NULL;
        }
        if (t->has_lb && t->has_ub && t->lb == t->ub)
            n = t->lb;
        else
            n = dec_len(r, t->has_ub, t->has_lb ? t->lb : 0, t->has_ub ? t->ub : 0);
        if (n < 0)
            return NULL;
        if (n > r->nbits) {
            fail("implausible SEQUENCE OF count");
            return NULL;
        }
        for (i = 0; i < n; i++) {
            a_val_t *e = dec_val(a, r, t->elem), **k;

            if (!e)
                return NULL;
            if (v->n >= v->len) {
                int cap = v->len ? v->len * 2 : 8;

                k = a_alloc(a, sizeof(a_val_t *) * (size_t)cap);
                if (!k)
                    return NULL;
                if (v->n)
                    memcpy(k, v->kids, sizeof(a_val_t *) * (size_t)v->n);
                v->kids = k;
                v->len = cap;
            }
            v->kids[v->n++] = e;
        }
        return v;
    }
    case A_SEQ: {
        int ext = 0;
        uint8_t present[256];

        if (t->n_total > 256) {
            fail("SEQUENCE too large");
            return NULL;
        }
        if (t->ext) {
            ext = br_bit(r);
            if (ext < 0)
                return NULL;
        }
        for (i = 0; i < t->n_root; i++) {
            if (t->members[i].optional) {
                int b = br_bit(r);

                if (b < 0)
                    return NULL;
                present[i] = (uint8_t)b;
            } else {
                present[i] = 1;
            }
        }
        for (i = 0; i < t->n_root; i++) {
            if (!present[i])
                continue;
            if (!t->members[i].type) {
                fail("component not in the DSVD subset");
                return NULL;
            }
            v->kids[i] = dec_val(a, r, t->members[i].type);
            if (!v->kids[i])
                return NULL;
        }
        if (ext) {
            uint64_t nm1;
            int nadd;

            if (dec_nsn(r, &nm1) < 0)
                return NULL;
            nadd = (int)nm1 + 1;
            if (nadd > 256) {
                fail("too many extension additions");
                return NULL;
            }
            for (i = 0; i < nadd; i++) {
                int b = br_bit(r);

                if (b < 0)
                    return NULL;
                present[i] = (uint8_t)b;
            }
            for (i = 0; i < nadd; i++) {
                const a_type_t *mt = (t->n_root + i < t->n_total) ? t->members[t->n_root + i].type : NULL;
                int skipped = 0;
                a_val_t *e;

                if (!present[i])
                    continue;
                e = dec_open(a, r, mt, &skipped);
                if (!e && !skipped)
                    return NULL;
                if (e)
                    v->kids[t->n_root + i] = e;     /* skipped: unknown or pruned, ignored */
            }
        }
        return v;
    }
    case A_CHOICE: {
        int ext = 0;
        uint64_t x;

        if (t->ext) {
            ext = br_bit(r);
            if (ext < 0)
                return NULL;
        }
        if (!ext) {
            if (t->n_root > 1) {
                if (dec_cwn(r, (uint64_t)t->n_root, &x) < 0)
                    return NULL;
            } else {
                x = 0;
            }
            if ((int)x >= t->n_root || !t->members[x].type) {
                fail("CHOICE alternative not in the DSVD subset");
                return NULL;
            }
            v->i = (int64_t)x;
            v->kids[0] = dec_val(a, r, t->members[x].type);
            return v->kids[0] ? v : NULL;
        }
        if (dec_nsn(r, &x) < 0)
            return NULL;
        if (t->n_root + (int)x >= t->n_total || !t->members[t->n_root + (int)x].type) {
            fail("CHOICE extension alternative not in the DSVD subset");
            return NULL;
        }
        v->i = t->n_root + (int64_t)x;
        {
            int skipped = 0;

            v->kids[0] = dec_open(a, r, t->members[v->i].type, &skipped);
        }
        return v->kids[0] ? v : NULL;
    }
    }
    fail("unknown type kind");
    return NULL;
}

a_val_t *a_decode(a_arena_t *a, const a_type_t *t, const uint8_t *in, int len)
{
    br_t r = { in, len * 8, 0 };

    g_err = NULL;
    if (len < 1) {
        fail("empty encoding");
        return NULL;
    }
    return dec_val(a, &r, t);
}
