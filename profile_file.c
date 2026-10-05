/*
 * profile_file.c — the --profile configuration file: Cisco-style or JSON
 *
 * Parses and renders the file that holds the AT&W stored profiles.  It
 * knows the SYNTAX of a setting and which AT command it spells; it knows
 * nothing about what values are valid, which the AT interpreter decides when
 * data_interface.c replays the settings.  See profile_file.h for the format.
 */

#include "profile_file.h"

#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

void pf_doc_init(pf_doc_t *doc)
{
    memset(doc, 0, sizeof(*doc));
    doc->power_on = -1;
}

bool pf_add(pf_profile_t *p, int src, const char *fmt, ...)
{
    va_list ap;
    int n;

    if (p->n >= PF_LINES)
        return false;
    va_start(ap, fmt);
    n = vsnprintf(p->line[p->n], PF_LINE, fmt, ap);
    va_end(ap);
    if (n < 0 || n >= PF_LINE)
        return false;
    p->src[p->n++] = src;
    p->present = true;
    return true;
}

bool pf_path_is_json(const char *path)
{
    size_t n = path ? strlen(path) : 0;

    return n >= 5 && !strcasecmp(path + n - 5, ".json");
}

static bool is_uint(const char *s)
{
    if (!*s)
        return false;
    for (; *s; s++) {
        if (!isdigit((unsigned char) *s))
            return false;
    }
    return true;
}

/* Split "word rest": word into w, rest (leading blanks skipped) returned. */
static const char *first_word(const char *s, char *w, size_t cap)
{
    size_t k = 0;

    while (*s == ' ' || *s == '\t')
        s++;
    while (*s && *s != ' ' && *s != '\t') {
        if (k + 1 < cap)
            w[k++] = *s;
        s++;
    }
    w[k] = '\0';
    while (*s == ' ' || *s == '\t')
        s++;
    return s;
}

int pf_setting_to_at(const char *setting, char *out, size_t cap)
{
    static const struct { const char *name; char cmd; } flags[] = {
        { "echo", 'E' }, { "quiet", 'Q' }, { "verbose", 'V' },
    };
    static const struct { const char *name; const char *cmd; } numeric[] = {
        { "result-codes", "X" }, { "dcd", "&C" }, { "dtr", "&D" },
    };
    char w[40];
    const char *rest = first_word(setting, w, sizeof(w));
    bool neg = false;

    if (!strcmp(w, "no")) {
        neg = true;
        rest = first_word(rest, w, sizeof(w));
    }
    for (size_t i = 0; i < sizeof(flags) / sizeof(flags[0]); i++) {
        if (!strcmp(w, flags[i].name)) {
            if (*rest) {
                snprintf(out, cap, "'%s' takes no value (use 'no %s' for off)", w, w);
                return -1;
            }
            snprintf(out, cap, "%c%d", flags[i].cmd, neg ? 0 : 1);
            return 0;
        }
    }
    if (neg) {
        snprintf(out, cap, "'no' applies only to echo, quiet and verbose");
        return -1;
    }
    for (size_t i = 0; i < sizeof(numeric) / sizeof(numeric[0]); i++) {
        if (!strcmp(w, numeric[i].name)) {
            if (!is_uint(rest)) {
                snprintf(out, cap, "'%s' takes a number", w);
                return -1;
            }
            snprintf(out, cap, "%s%s", numeric[i].cmd, rest);
            return 0;
        }
    }
    if (!strcmp(w, "dial")) {
        if (!strcmp(rest, "tone") || !strcmp(rest, "pulse")) {
            snprintf(out, cap, "%s", rest[0] == 't' ? "T" : "P");
            return 0;
        }
        snprintf(out, cap, "'dial' takes tone or pulse");
        return -1;
    }
    if (!strcmp(w, "s-register") || !strcmp(w, "number")) {
        char n[16];
        const char *v = first_word(rest, n, sizeof(n));

        if (!is_uint(n) || !*v || (w[0] == 's' && !is_uint(v))) {
            snprintf(out, cap, w[0] == 's' ? "'s-register' takes a register and a value"
                                           : "'number' takes a slot and a quoted dial string");
            return -1;
        }
        if (w[0] == 's')
            snprintf(out, cap, "S%s=%s", n, v);
        else
            snprintf(out, cap, "+ASTO=%s,%s", n, v);
        return 0;
    }
    if (!strcmp(w, "at")) {
        if (!*rest) {
            snprintf(out, cap, "'at' takes the commands to send");
            return -1;
        }
        snprintf(out, cap, "%s", rest);
        return 0;
    }
    if (w[0] == '+' && w[1]) {
        if (!*rest) {
            snprintf(out, cap, "'%s' takes a value", w);
            return -1;
        }
        snprintf(out, cap, "%s=%s", w, rest);
        return 0;
    }
    snprintf(out, cap, "'%s' is not a setting", w);
    return -1;
}

/* ------------------------------------------------------------------ */
/* Cisco-style and legacy AT-line files                                */
/* ------------------------------------------------------------------ */

static int parse_lines(const char *text, pf_doc_t *doc, char *err, size_t errlen)
{
    const char *p = text;
    int lineno = 0;
    int cur = -1;
    bool legacy = false;
    bool decided = false;

    while (*p) {
        char raw[PF_LINE + 8];
        char at[PF_LINE];
        size_t n = strcspn(p, "\n");
        char *s;
        size_t len;
        char w[40];
        const char *rest;

        lineno++;
        if (n >= sizeof(raw)) {
            snprintf(err, errlen, "line %d: too long", lineno);
            return -1;
        }
        memcpy(raw, p, n);
        raw[n] = '\0';
        p += n + (p[n] == '\n');
        s = raw;
        while (*s == ' ' || *s == '\t')
            s++;
        len = strlen(s);
        while (len && (s[len - 1] == '\r' || s[len - 1] == ' ' || s[len - 1] == '\t'))
            s[--len] = '\0';
        if (!len || s[0] == '!' || s[0] == '#')
            continue;
        if (!decided) {
            /* The first --profile files were AT command lines. */
            legacy = !strncasecmp(s, "AT", 2) && s[2] && s[2] != ' ' && s[2] != '\t';
            decided = true;
            if (legacy)
                doc->power_on = 0;
        }
        if (legacy) {
            if (strncasecmp(s, "AT", 2)) {
                snprintf(err, errlen, "line %d: expected an AT command line", lineno);
                return -1;
            }
            if (!pf_add(&doc->profile[0], lineno, "at %s", s + 2)) {
                snprintf(err, errlen, "line %d: too many settings", lineno);
                return -1;
            }
            continue;
        }
        rest = first_word(s, w, sizeof(w));
        if (!strcmp(w, "end"))
            break;
        if (!strcmp(w, "profile") || !strcmp(w, "power-on-profile")) {
            int v = atoi(rest);

            if (!is_uint(rest) || v >= PF_PROFILES) {
                snprintf(err, errlen, "line %d: '%s' takes a profile number 0-%d",
                         lineno, w, PF_PROFILES - 1);
                return -1;
            }
            if (w[0] == 'p' && w[1] == 'o') {
                doc->power_on = v;
            } else {
                cur = v;
                doc->profile[cur].present = true;
            }
            continue;
        }
        if (pf_setting_to_at(s, at, sizeof(at)) < 0) {
            snprintf(err, errlen, "line %d: %s", lineno, at);
            return -1;
        }
        if (strcmp(w, "number") && cur < 0) {
            snprintf(err, errlen, "line %d: setting before any 'profile N' line", lineno);
            return -1;
        }
        /* Stored numbers are not part of a profile (Z does not touch them)
         * wherever they are written. */
        if (!pf_add(strcmp(w, "number") ? &doc->profile[cur] : &doc->global, lineno, "%s", s)) {
            snprintf(err, errlen, "line %d: too many settings", lineno);
            return -1;
        }
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* JSON                                                               */
/* ------------------------------------------------------------------ */

typedef struct {
    const char *p;
    int line;
    char *err;
    size_t errlen;
} js_t;

enum { JS_STRING, JS_NUMBER, JS_TRUE, JS_FALSE, JS_NULL, JS_OBJECT, JS_ARRAY };

static int js_fail(js_t *j, const char *what)
{
    snprintf(j->err, j->errlen, "line %d: %s", j->line, what);
    return -1;
}

static void js_ws(js_t *j)
{
    while (*j->p == ' ' || *j->p == '\t' || *j->p == '\r' || *j->p == '\n') {
        if (*j->p == '\n')
            j->line++;
        j->p++;
    }
}

static bool js_eat(js_t *j, char c)
{
    js_ws(j);
    if (*j->p != c)
        return false;
    j->p++;
    return true;
}

static int js_string(js_t *j, char *out, size_t cap)
{
    size_t k = 0;

    js_ws(j);
    if (*j->p != '"')
        return js_fail(j, "expected a string");
    j->p++;
    while (*j->p && *j->p != '"') {
        char c = *j->p++;

        if (c == '\n')
            return js_fail(j, "newline inside a string");
        if (c == '\\') {
            c = *j->p++;
            switch (c) {
            case 'n': c = '\n'; break;
            case 't': c = '\t'; break;
            case 'r': c = '\r'; break;
            case 'b': c = '\b'; break;
            case 'f': c = '\f'; break;
            case 'u': {
                unsigned v = 0;

                for (int i = 0; i < 4; i++) {
                    if (!isxdigit((unsigned char) *j->p))
                        return js_fail(j, "bad \\u escape");
                    v = v * 16 + (unsigned) (isdigit((unsigned char) *j->p) ? *j->p - '0'
                                             : (tolower((unsigned char) *j->p) - 'a' + 10));
                    j->p++;
                }
                if (v == 0 || v > 0x7e)
                    return js_fail(j, "only printable ASCII is allowed in a setting");
                c = (char) v;
                break;
            }
            case '"': case '\\': case '/': break;
            default:
                return js_fail(j, "bad escape");
            }
        }
        if (k + 1 >= cap)
            return js_fail(j, "string too long");
        out[k++] = c;
    }
    if (*j->p != '"')
        return js_fail(j, "unterminated string");
    j->p++;
    out[k] = '\0';
    return 0;
}

/* A scalar, or the type of a container left for the caller to open.
 * Numbers must be whole. */
static int js_peek_value(js_t *j, char *out, size_t cap)
{
    js_ws(j);
    switch (*j->p) {
    case '"':
        return js_string(j, out, cap) < 0 ? -1 : JS_STRING;
    case '{':
        return JS_OBJECT;
    case '[':
        return JS_ARRAY;
    }
    if (!strncmp(j->p, "true", 4)) { j->p += 4; return JS_TRUE; }
    if (!strncmp(j->p, "false", 5)) { j->p += 5; return JS_FALSE; }
    if (!strncmp(j->p, "null", 4)) { j->p += 4; return JS_NULL; }
    if (*j->p == '-' || isdigit((unsigned char) *j->p)) {
        size_t k = 0;

        if (*j->p == '-')
            out[k++] = *j->p++;
        while (isdigit((unsigned char) *j->p) && k + 1 < cap)
            out[k++] = *j->p++;
        out[k] = '\0';
        if (*j->p == '.' || *j->p == 'e' || *j->p == 'E')
            return js_fail(j, "numbers must be whole");
        return JS_NUMBER;
    }
    return js_fail(j, "expected a value");
}

static int js_skip(js_t *j);

static int js_skip_container(js_t *j, char open, char close)
{
    j->p++;
    if (js_eat(j, close))
        return 0;
    do {
        if (open == '{') {
            char key[PF_LINE];

            if (js_string(j, key, sizeof(key)) < 0 || !js_eat(j, ':'))
                return js_fail(j, "expected \"key\": value");
        }
        if (js_skip(j) < 0)
            return -1;
    } while (js_eat(j, ','));
    return js_eat(j, close) ? 0 : js_fail(j, "expected , or a closing bracket");
}

static int js_skip(js_t *j)
{
    char tmp[PF_LINE];
    int t = js_peek_value(j, tmp, sizeof(tmp));

    if (t == JS_OBJECT)
        return js_skip_container(j, '{', '}');
    if (t == JS_ARRAY)
        return js_skip_container(j, '[', ']');
    return t < 0 ? -1 : 0;
}

/* Iterate an object: calls fn for each key with the cursor on its value. */
typedef int (*js_member_fn)(js_t *j, const char *key, void *ctx);

static int js_object(js_t *j, js_member_fn fn, void *ctx)
{
    if (!js_eat(j, '{'))
        return js_fail(j, "expected an object");
    if (js_eat(j, '}'))
        return 0;
    do {
        char key[64];

        if (js_string(j, key, sizeof(key)) < 0)
            return -1;
        if (!js_eat(j, ':'))
            return js_fail(j, "expected ':'");
        if (fn(j, key, ctx) < 0)
            return -1;
    } while (js_eat(j, ','));
    return js_eat(j, '}') ? 0 : js_fail(j, "expected ',' or '}'");
}

static int js_add(js_t *j, pf_profile_t *p, int line, const char *setting)
{
    char at[PF_LINE];

    if (pf_setting_to_at(setting, at, sizeof(at)) < 0) {
        snprintf(j->err, j->errlen, "line %d: %s", line, at);
        return -1;
    }
    if (!pf_add(p, line, "%s", setting))
        return js_fail(j, "too many settings");
    return 0;
}

/* A dial string as the +ASTO argument: quoted, with " as V.250's \22. */
static void quote_number(const char *s, char *out, size_t cap)
{
    size_t k = 0;

    if (cap < 3)
        return;
    out[k++] = '"';
    for (; *s && k + 5 < cap; s++) {
        if (*s == '"') {
            memcpy(out + k, "\\22", 3);
            k += 3;
        } else {
            out[k++] = *s;
        }
    }
    out[k++] = '"';
    out[k] = '\0';
}

static int js_sreg(js_t *j, const char *key, void *ctx)
{
    char v[32];
    char s[PF_LINE];
    int line = j->line;

    if (js_peek_value(j, v, sizeof(v)) != JS_NUMBER)
        return js_fail(j, "s-registers values must be numbers");
    snprintf(s, sizeof(s), "s-register %s %s", key, v);
    return js_add(j, ctx, line, s);
}

static int js_profile_member(js_t *j, const char *key, void *ctx)
{
    pf_profile_t *p = ctx;
    char v[PF_LINE];
    char s[PF_LINE + 80];
    int line;
    int t;

    js_ws(j);
    line = j->line;
    t = js_peek_value(j, v, sizeof(v));
    switch (t) {
    case JS_TRUE:
    case JS_FALSE:
        snprintf(s, sizeof(s), "%s%s", t == JS_FALSE ? "no " : "", key);
        return js_add(j, p, line, s);
    case JS_NUMBER:
    case JS_STRING:
        snprintf(s, sizeof(s), "%s %s", key, v);
        return js_add(j, p, line, s);
    case JS_NULL:
        return 0;
    case JS_OBJECT:
        if (strcmp(key, "s-registers"))
            return js_fail(j, "only \"s-registers\" takes an object");
        return js_object(j, js_sreg, p);
    case JS_ARRAY:
        if (strcmp(key, "at"))
            return js_fail(j, "only \"at\" takes a list");
        j->p++;
        if (js_eat(j, ']'))
            return 0;
        do {
            js_ws(j);
            line = j->line;
            if (js_string(j, v, sizeof(v)) < 0)
                return -1;
            snprintf(s, sizeof(s), "at %s", v);
            if (js_add(j, p, line, s) < 0)
                return -1;
        } while (js_eat(j, ','));
        return js_eat(j, ']') ? 0 : js_fail(j, "expected ',' or ']'");
    }
    return -1;
}

static int profile_number(js_t *j, const char *key)
{
    if (!is_uint(key) || atoi(key) >= PF_PROFILES)
        return js_fail(j, "profile numbers are 0-9");
    return atoi(key);
}

static int js_profiles(js_t *j, const char *key, void *ctx)
{
    pf_doc_t *doc = ctx;
    int n = profile_number(j, key);

    if (n < 0)
        return -1;
    doc->profile[n].present = true;
    return js_object(j, js_profile_member, &doc->profile[n]);
}

static int js_numbers(js_t *j, const char *key, void *ctx)
{
    pf_doc_t *doc = ctx;
    char v[PF_LINE];
    char q[PF_LINE];
    char s[PF_LINE + 40];
    int line = j->line;
    int t = js_peek_value(j, v, sizeof(v));

    if (t != JS_STRING && t != JS_NUMBER)
        return js_fail(j, "numbers are strings");
    quote_number(v, q, sizeof(q));
    snprintf(s, sizeof(s), "number %s %s", key, q);
    return js_add(j, &doc->global, line, s);
}

static int js_top(js_t *j, const char *key, void *ctx)
{
    pf_doc_t *doc = ctx;
    char v[32];

    if (!strcmp(key, "power-on-profile")) {
        if (js_peek_value(j, v, sizeof(v)) != JS_NUMBER)
            return js_fail(j, "power-on-profile is a number");
        doc->power_on = profile_number(j, v);
        return doc->power_on < 0 ? -1 : 0;
    }
    if (!strcmp(key, "profiles"))
        return js_object(j, js_profiles, doc);
    if (!strcmp(key, "numbers"))
        return js_object(j, js_numbers, doc);
    if (key[0] == '_')                  /* "_comment" and the like */
        return js_skip(j);
    return js_fail(j, "unknown key (expected power-on-profile, numbers or profiles)");
}

int pf_parse(const char *text, pf_doc_t *doc, char *err, size_t errlen)
{
    const char *p = text;

    pf_doc_init(doc);
    err[0] = '\0';
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')
        p++;
    if (*p == '{') {
        js_t j = { text, 1, err, errlen };

        if (js_object(&j, js_top, doc) < 0)
            return -1;
        js_ws(&j);
        return *j.p ? js_fail(&j, "text after the closing '}'") : 0;
    }
    return parse_lines(text, doc, err, errlen);
}

/* ------------------------------------------------------------------ */
/* Rendering                                                          */
/* ------------------------------------------------------------------ */

typedef struct {
    char *out;
    size_t cap;
    size_t used;
    bool overflow;
} sb_t;

static void sb(sb_t *b, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

static void sb(sb_t *b, const char *fmt, ...)
{
    va_list ap;
    int n;

    if (b->overflow)
        return;
    va_start(ap, fmt);
    n = vsnprintf(b->out + b->used, b->cap - b->used, fmt, ap);
    va_end(ap);
    if (n < 0 || (size_t) n >= b->cap - b->used) {
        b->overflow = true;
        return;
    }
    b->used += (size_t) n;
}

static void sb_json_string(sb_t *b, const char *s)
{
    sb(b, "\"");
    for (; *s; s++) {
        if (*s == '"' || *s == '\\')
            sb(b, "\\%c", *s);
        else if ((unsigned char) *s < 0x20)
            sb(b, "\\u%04x", (unsigned char) *s);
        else
            sb(b, "%c", *s);
    }
    sb(b, "\"");
}

/* "\"55\\225\"" -> 55"5: the dial string inside a number setting. */
static void unquote_number(const char *s, char *out, size_t cap)
{
    size_t k = 0;
    size_t n = strlen(s);

    if (n >= 2 && s[0] == '"' && s[n - 1] == '"') {
        s++;
        n -= 2;
    }
    for (size_t i = 0; i < n && k + 1 < cap; i++) {
        if (s[i] == '\\' && i + 2 < n && s[i + 1] == '2' && s[i + 2] == '2') {
            out[k++] = '"';
            i += 2;
        } else {
            out[k++] = s[i];
        }
    }
    out[k] = '\0';
}

static void render_json_profile(sb_t *b, const pf_profile_t *p)
{
    bool first = true;
    int nsreg = 0;
    int nat = 0;

    sb(b, "{");
    for (int i = 0; i < p->n; i++) {
        char w[40];
        const char *rest = first_word(p->line[i], w, sizeof(w));

        if (!strcmp(w, "s-register")) {
            nsreg++;
            continue;
        }
        if (!strcmp(w, "at")) {
            nat++;
            continue;
        }
        sb(b, "%s\n      ", first ? "" : ",");
        first = false;
        if (!strcmp(w, "no")) {
            first_word(rest, w, sizeof(w));
            sb_json_string(b, w);
            sb(b, ": false");
        } else if (!*rest) {
            sb_json_string(b, w);
            sb(b, ": true");
        } else {
            sb_json_string(b, w);
            sb(b, ": ");
            if (w[0] != '+' && is_uint(rest) && strlen(rest) < 10)
                sb(b, "%s", rest);
            else
                sb_json_string(b, rest);
        }
    }
    if (nsreg) {
        int k = 0;

        sb(b, "%s\n      \"s-registers\": {", first ? "" : ",");
        first = false;
        for (int i = 0; i < p->n; i++) {
            char w[40];
            char r[16];
            const char *rest = first_word(p->line[i], w, sizeof(w));

            if (strcmp(w, "s-register"))
                continue;
            rest = first_word(rest, r, sizeof(r));
            sb(b, "%s\"%s\": %s", k++ ? ", " : " ", r, rest);
        }
        sb(b, " }");
    }
    if (nat) {
        int k = 0;

        sb(b, "%s\n      \"at\": [", first ? "" : ",");
        first = false;
        for (int i = 0; i < p->n; i++) {
            char w[40];
            const char *rest = first_word(p->line[i], w, sizeof(w));

            if (strcmp(w, "at"))
                continue;
            sb(b, "%s", k++ ? ", " : " ");
            sb_json_string(b, rest);
        }
        sb(b, " ]");
    }
    sb(b, "\n    }");
}

static void render_json(sb_t *b, const pf_doc_t *doc)
{
    bool first = true;

    sb(b, "{\n  \"_comment\": \"v90modem stored profiles: AT&W writes this file, "
          "start-up and ATZn read it\"");
    if (doc->power_on >= 0)
        sb(b, ",\n  \"power-on-profile\": %d", doc->power_on);
    if (doc->global.n) {
        int k = 0;

        sb(b, ",\n  \"numbers\": {");
        for (int i = 0; i < doc->global.n; i++) {
            char w[40];
            char slot[16];
            char num[PF_LINE];
            const char *rest = first_word(doc->global.line[i], w, sizeof(w));

            if (strcmp(w, "number"))
                continue;
            rest = first_word(rest, slot, sizeof(slot));
            unquote_number(rest, num, sizeof(num));
            sb(b, "%s\n    \"%s\": ", k++ ? "," : "", slot);
            sb_json_string(b, num);
        }
        sb(b, "\n  }");
    }
    sb(b, ",\n  \"profiles\": {");
    for (int n = 0; n < PF_PROFILES; n++) {
        if (!doc->profile[n].present)
            continue;
        sb(b, "%s\n    \"%d\": ", first ? "" : ",", n);
        first = false;
        render_json_profile(b, &doc->profile[n]);
    }
    sb(b, "\n  }\n}\n");
}

static void render_cisco(sb_t *b, const pf_doc_t *doc)
{
    sb(b, "! v90modem stored profiles: AT&W writes this file, start-up and ATZn read it.\n"
          "! One setting per line, each one AT command: \"no echo\" is E0, \"s-register 0 2\"\n"
          "! is S0=2, \"+ES 3,0,2\" is +ES=3,0,2, and \"at <commands>\" is sent as written.\n"
          "!\n");
    if (doc->power_on >= 0)
        sb(b, "power-on-profile %d\n!\n", doc->power_on);
    for (int i = 0; i < doc->global.n; i++)
        sb(b, "%s\n", doc->global.line[i]);
    if (doc->global.n)
        sb(b, "!\n");
    for (int n = 0; n < PF_PROFILES; n++) {
        if (!doc->profile[n].present)
            continue;
        sb(b, "profile %d\n", n);
        for (int i = 0; i < doc->profile[n].n; i++)
            sb(b, " %s\n", doc->profile[n].line[i]);
        sb(b, "!\n");
    }
    sb(b, "end\n");
}

int pf_render(const pf_doc_t *doc, bool json, char *out, size_t cap)
{
    sb_t b = { out, cap, 0, cap == 0 };

    if (json)
        render_json(&b, doc);
    else
        render_cisco(&b, doc);
    return b.overflow ? -1 : (int) b.used;
}
