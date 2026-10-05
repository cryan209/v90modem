/*
 * per.h -- a small ASN.1 aligned PER (ITU-T X.691, "basic ALIGNED variant")
 * encoder and decoder driven by constant schema tables.
 *
 * H.245 messages are PER encoded with the aligned variant (H.245 Annex A), and
 * V.75 carries them in the user data of V.76 frames, so a DSVD control entity
 * has to speak it.  The schema tables for the DSVD subset of H.245 are
 * generated from the Recommendation's own ASN.1 by tools/h245/per_gen.py
 * (h245_schema.c); this file is the interpreter.
 *
 * Scope: BOOLEAN, NULL, INTEGER (constrained), OCTET STRING, OBJECT IDENTIFIER,
 * SEQUENCE / SET (optional components, extension marker and additions),
 * SEQUENCE OF / SET OF, CHOICE (extension marker and additions).  Not here:
 * strings, ENUMERATED, BIT STRING, DEFAULT, fragmentation (>= 16K), constraint
 * extension markers.  The generator stops on anything outside this list.
 *
 * A type pruned from the DSVD subset keeps its place (a CHOICE alternative is
 * still counted, so the index and its width are right) but has no schema: the
 * encoder refuses it and the decoder reports it as unsupported.  Extension
 * additions that are unknown or pruned are skipped on decode, which is what
 * the encoding rules allow because each is length-prefixed.
 */

#ifndef PER_H
#define PER_H

#include <stddef.h>
#include <stdint.h>

typedef enum { A_BOOL = 0, A_NULL, A_INT, A_OCTETS, A_OID, A_SEQ, A_SEQOF, A_CHOICE } a_kind_t;

typedef struct a_type a_type_t;

typedef struct {
    const char *name;
    const a_type_t *type;       /* NULL: not in the subset */
    uint8_t optional;           /* presence bit (always set for extension additions) */
} a_member_t;

struct a_type {
    uint8_t kind;
    uint8_t ext;                /* extensible ("...") */
    uint8_t has_lb, has_ub;     /* INTEGER range, or SIZE range for OCTETS / SEQOF */
    int64_t lb, ub;
    uint16_t n_root, n_total;   /* SEQ / CHOICE: root members, and root + additions */
    const a_member_t *members;
    const a_type_t *elem;       /* SEQOF */
    const char *name;           /* the ASN.1 path, for diagnostics */
};

typedef struct {
    const char *name;
    const a_type_t *type;
} a_named_t;

extern const a_type_t T_BOOLEAN, T_NULL;
extern const a_named_t h245_types[];

/* ---- values ------------------------------------------------------------- */

typedef struct a_val a_val_t;
struct a_val {
    const a_type_t *type;
    int64_t i;                  /* BOOL 0/1, INT, CHOICE alternative index */
    uint8_t *data;              /* OCTETS, OID (BER contents octets) */
    int len;
    a_val_t **kids;             /* SEQ: n_total slots, NULL = absent; SEQOF: elements;
                                 * CHOICE: kids[0] */
    int n;                      /* SEQOF element count */
};

/* Values and decode results live in an arena the caller provides. */
typedef struct {
    uint8_t *buf;
    size_t size, used;
} a_arena_t;

void a_arena_init(a_arena_t *a, void *buf, size_t size);

a_val_t *a_new(a_arena_t *a, const a_type_t *t);
/* SEQ: the child for member `name` (created if absent).  NULL if there is no
 * such member, it is pruned, or the arena is full. */
a_val_t *a_member(a_arena_t *a, a_val_t *seq, const char *name);
/* CHOICE: select alternative `name` and return its (new) value. */
a_val_t *a_choice(a_arena_t *a, a_val_t *ch, const char *alt);
/* The same, by alternative index (for cause codes that are the index). */
a_val_t *a_choice_at(a_arena_t *a, a_val_t *ch, int idx);
int a_choice_index(const a_val_t *ch);                  /* -1 if none */
/* SEQOF: append a new element. */
a_val_t *a_append(a_arena_t *a, a_val_t *seqof);

/* Setters return 0, or -1 if the value is out of range for the type. */
int a_set_int(a_val_t *v, int64_t x);
int a_set_bool(a_val_t *v, int x);
int a_set_octets(a_arena_t *a, a_val_t *v, const uint8_t *p, int n);
int a_set_oid(a_arena_t *a, a_val_t *v, const uint32_t *arcs, int n);

/* Accessors; all tolerate NULL. */
const a_val_t *a_get(const a_val_t *seq, const char *name);
const char *a_choice_name(const a_val_t *ch);
const a_val_t *a_choice_val(const a_val_t *ch);
int a_oid_arcs(const a_val_t *v, uint32_t *arcs, int max);      /* arcs, or -1 */

/* Encode the complete value (X.691 10.1: padded to octets; an empty encoding
 * is one zero octet).  Returns the length, or -1 (see a_error()). */
int a_encode(const a_val_t *v, uint8_t *out, int max);
/* Decode a complete encoding of type t.  NULL on error (see a_error()). */
a_val_t *a_decode(a_arena_t *a, const a_type_t *t, const uint8_t *in, int len);

const char *a_error(void);
const a_type_t *a_find_type(const char *asn1_name);

#endif
