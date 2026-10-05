/*
 * per_tool -- test bridge between the PER engine (per.c + h245_schema.c) and
 * an independent ASN.1 implementation (tools/h245/per_oracle.py).
 *
 *   per_tool dump <Type>   hex on stdin  -> decoded value as JSON on stdout
 *   per_tool enc  <Type>   JSON on stdin -> PER encoding as hex on stdout
 *
 * JSON conventions (they are asn1tools' value conventions): SEQUENCE is an
 * object holding only the components present; CHOICE is ["alternative",
 * value]; SEQUENCE OF is an array; INTEGER a number; BOOLEAN true/false; NULL
 * null; OCTET STRING a hex string; OBJECT IDENTIFIER a dotted string.
 */
#include "../../per.h"

#include <ctype.h>
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct j_s {
    char kind;                  /* o a s n t f z */
    char *key;
    char *str;
    long long num;
    struct j_s *kid, *next;
} j_t;

static const char *jp;

static void skip(void) { while (isspace((unsigned char)*jp)) jp++; }

static jmp_buf jb;
static char errmsg[256];

static void die(const char *m)
{
    snprintf(errmsg, sizeof(errmsg), "%s", m);
    longjmp(jb, 1);
}

static j_t *parse(void)
{
    j_t *j = calloc(1, sizeof(*j));

    skip();
    if (*jp == '{') {
        j_t **tail = &j->kid;

        j->kind = 'o';
        jp++;
        skip();
        while (*jp != '}') {
            j_t *k;
            const char *s;

            skip();
            if (*jp != '"') die("object key");
            s = ++jp;
            while (*jp != '"') jp++;
            k = NULL;
            {
                char *key = strndup(s, (size_t)(jp - s));

                jp++;
                skip();
                if (*jp++ != ':') die("colon");
                k = parse();
                k->key = key;
            }
            *tail = k;
            tail = &k->next;
            skip();
            if (*jp == ',') jp++;
            skip();
        }
        jp++;
    } else if (*jp == '[') {
        j_t **tail = &j->kid;

        j->kind = 'a';
        jp++;
        skip();
        while (*jp != ']') {
            j_t *k = parse();

            *tail = k;
            tail = &k->next;
            skip();
            if (*jp == ',') jp++;
            skip();
        }
        jp++;
    } else if (*jp == '"') {
        const char *s = ++jp;

        while (*jp != '"') jp++;
        j->kind = 's';
        j->str = strndup(s, (size_t)(jp - s));
        jp++;
    } else if (!strncmp(jp, "true", 4)) {
        j->kind = 't'; jp += 4;
    } else if (!strncmp(jp, "false", 5)) {
        j->kind = 'f'; jp += 5;
    } else if (!strncmp(jp, "null", 4)) {
        j->kind = 'z'; jp += 4;
    } else {
        char *e;

        j->kind = 'n';
        j->num = strtoll(jp, &e, 10);
        if (e == jp) die("bad JSON");
        jp = e;
    }
    return j;
}

static void fill(a_arena_t *ar, a_val_t *v, j_t *j)
{
    const a_type_t *t = v->type;
    j_t *k;

    switch (t->kind) {
    case A_BOOL:
        if (j->kind != 't' && j->kind != 'f') die("expected boolean");
        a_set_bool(v, j->kind == 't');
        break;
    case A_NULL:
        if (j->kind != 'z') die("expected null");
        break;
    case A_INT:
        if (j->kind != 'n' || a_set_int(v, j->num) < 0) die("integer out of range");
        break;
    case A_OCTETS: {
        size_t n = strlen(j->str) / 2, i;
        uint8_t *b = malloc(n + 1);

        for (i = 0; i < n; i++) {
            unsigned x;

            sscanf(j->str + 2 * i, "%2x", &x);
            b[i] = (uint8_t)x;
        }
        if (a_set_octets(ar, v, b, (int)n) < 0) die(a_error());
        break;
    }
    case A_OID: {
        uint32_t arcs[32];
        int n = 0;
        char *s = strdup(j->str), *p = s, *q;

        while (p && n < 32) {
            q = strchr(p, '.');
            if (q) *q++ = 0;
            arcs[n++] = (uint32_t)strtoul(p, NULL, 10);
            p = q;
        }
        if (a_set_oid(ar, v, arcs, n) < 0) die("bad OID");
        break;
    }
    case A_SEQ:
        if (j->kind != 'o') die("expected object");
        for (k = j->kid; k; k = k->next) {
            a_val_t *c = a_member(ar, v, k->key);

            if (!c) {
                char m[200];

                snprintf(m, sizeof(m), "no such (or pruned) member %s in %s", k->key, t->name);
                die(m);
            }
            fill(ar, c, k);
        }
        break;
    case A_SEQOF:
        if (j->kind != 'a') die("expected array");
        for (k = j->kid; k; k = k->next) {
            a_val_t *c = a_append(ar, v);

            if (!c) die("sequence-of element not in the subset");
            fill(ar, c, k);
        }
        break;
    case A_CHOICE: {
        a_val_t *c;

        if (j->kind != 'a' || !j->kid || !j->kid->next) die("expected [alt, value]");
        c = a_choice(ar, v, j->kid->str);
        if (!c) {
            char m[200];

            snprintf(m, sizeof(m), "no such (or pruned) alternative %s in %s", j->kid->str, t->name);
            die(m);
        }
        fill(ar, c, j->kid->next);
        break;
    }
    }
}

static void out(const a_val_t *v)
{
    const a_type_t *t = v->type;
    int i;

    switch (t->kind) {
    case A_BOOL: printf(v->i ? "true" : "false"); break;
    case A_NULL: printf("null"); break;
    case A_INT: printf("%lld", (long long)v->i); break;
    case A_OCTETS:
        printf("\"");
        for (i = 0; i < v->len; i++) printf("%02x", v->data[i]);
        printf("\"");
        break;
    case A_OID: {
        uint32_t arcs[32];
        int n = a_oid_arcs(v, arcs, 32);

        printf("\"");
        for (i = 0; i < n; i++) printf(i ? ".%u" : "%u", arcs[i]);
        printf("\"");
        break;
    }
    case A_SEQ: {
        int first = 1;

        printf("{");
        for (i = 0; i < t->n_total; i++)
            if (v->kids[i]) {
                printf("%s\"%s\":", first ? "" : ",", t->members[i].name);
                first = 0;
                out(v->kids[i]);
            }
        printf("}");
        break;
    }
    case A_SEQOF:
        printf("[");
        for (i = 0; i < v->n; i++) {
            if (i) printf(",");
            out(v->kids[i]);
        }
        printf("]");
        break;
    case A_CHOICE:
        printf("[\"%s\",", t->members[v->i].name);
        out(v->kids[0]);
        printf("]");
        break;
    }
}

static int do_one(const a_type_t *t, int enc, char *line, a_arena_t *ar)
{
    static uint8_t b[1 << 16];

    if (setjmp(jb)) {
        printf("ERR: %s\n", errmsg);
        return 1;
    }
    if (!enc) {
        size_t i, nb = 0, n = strlen(line);
        a_val_t *v;

        for (i = 0; i + 1 < n; i += 2) {
            unsigned x;

            if (sscanf(line + i, "%2x", &x) != 1) break;
            b[nb++] = (uint8_t)x;
        }
        v = a_decode(ar, t, b, (int)nb);
        if (!v) { printf("ERR: %s\n", a_error()); return 1; }
        out(v);
        printf("\n");
    } else {
        a_val_t *v = a_new(ar, t);
        int len, i;

        jp = line;
        fill(ar, v, parse());
        len = a_encode(v, b, sizeof(b));
        if (len < 0) { printf("ERR: %s\n", a_error()); return 1; }
        for (i = 0; i < len; i++) printf("%02x", b[i]);
        printf("\n");
    }
    return 0;
}

int main(int argc, char **argv)
{
    static char in[1 << 24];
    static uint8_t arena_buf[1 << 20];
    a_arena_t ar;
    const a_type_t *t;
    size_t n;
    char *line, *save;

    if (argc != 3) { printf("ERR: usage: per_tool dump|enc Type\n"); return 2; }
    t = a_find_type(argv[2]);
    if (!t) { printf("ERR: unknown type\n"); return 2; }
    n = fread(in, 1, sizeof(in) - 1, stdin);
    in[n] = 0;
    for (line = strtok_r(in, "\n", &save); line; line = strtok_r(NULL, "\n", &save)) {
        a_arena_init(&ar, arena_buf, sizeof(arena_buf));
        do_one(t, !strcmp(argv[1], "enc"), line, &ar);
        fflush(stdout);
    }
    return 0;
}
