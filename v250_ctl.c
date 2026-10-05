/*
 * v250_ctl.c -- see v250_ctl.h.  V.250 (07/2003) 6.4.3, 6.5.1, 6.5.5, 6.6.1,
 * 6.6.3, with the syntax rules of 5.4.4.
 */

#include "v250_ctl.h"

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>

/* Supported values, as bit masks over the value (all small). */
#define BIT(n) (1u << (n))
#define ES_RQST_OK (BIT(1) | BIT(2) | BIT(3))
#define ES_ORIG_FBK_OK (BIT(0) | BIT(2) | BIT(3))
#define ES_ANS_FBK_OK (BIT(1) | BIT(2) | BIT(4) | BIT(5))

#define DS_DICT_MIN 512
#define DS_DICT_MAX 65535
#define DS_STRING_MIN 6
#define DS_STRING_MAX 250

static int parse_compound(const char *s, int max, long *vals, bool *present);
static bool in_mask(long v, unsigned mask);

void v250_ctl_reset(v250_ctl_t *c)
{
    memset(c, 0, sizeof(*c));
    c->es[0] = 3;
    c->es[1] = 0;
    c->es[2] = 2;
    c->ds[0] = 3;
    c->ds[1] = 0;
    c->ds[2] = 1024;
    c->ds[3] = 32;
    /* +EB, +EFCS: the only values supported.  +ETBM: 6.5.6 recommends 1,1,20;
     * a DTE-requested hang-up discards pending transmit data here, so TD is 0,
     * and received data is already in the DTE's port, so RD 1 is what
     * happens.  +EWIND/+EFRAM: what this LAP.M has always offered. */
    c->etbm[1] = 1;
    c->etbm[2] = 20;
    c->ewind[0] = 15;
    c->efram[0] = 128;
    /* 6.2.10-6.2.12 recommended defaults: autodetect, 8N1 (3,3), circuit flow
     * control both ways.  +ILRR 0; +MSC 1 (6.4.8), which is what the engine
     * has always done. */
    /* +DS44: 6.6.2 recommends <direction> 3.  V.44 has passed every offline
     * test here but has never been negotiated with a foreign modem on a call,
     * so it is not offered unless the DTE asks; the rest are 6.6.2's own
     * example values (V.44 Appendix I). */
    c->ds44[0] = 0;
    c->ds44[3] = c->ds44[4] = 1024;
    c->ds44[5] = c->ds44[6] = 255;
    c->ds44[7] = c->ds44[8] = 3072;
    c->icf[0] = 3;
    c->icf[1] = 3;
    c->ifc[0] = 2;
    c->ifc[1] = 2;
    c->msc = 1;
}

/* +IPR's rates: the ones a pty's termios can name.  0 is "what the DTE sets". */
static const int ipr_rates[] = {
    0, 300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400
};

static v250_ctl_result_t ipr_command(v250_ctl_t *c, const char *arg, char *info, size_t len)
{
    long vals[1] = { 0 };
    bool present[1] = { false };
    size_t used;

    if (arg[0] == '?' && arg[1] == '\0') {
        snprintf(info, len, "+IPR: %d", c->ipr);
        return V250_CTL_OK;
    }
    if (!strcmp(arg, "=?")) {
        used = (size_t) snprintf(info, len, "+IPR: (");
        for (size_t i = 0; i < sizeof(ipr_rates) / sizeof(ipr_rates[0]) && used < len; i++)
            used += (size_t) snprintf(info + used, len - used, "%s%d", i ? "," : "", ipr_rates[i]);
        if (used < len)
            snprintf(info + used, len - used, "),()");
        return V250_CTL_OK;
    }
    if (arg[0] != '=' || parse_compound(arg + 1, 1, vals, present) < 0)
        return V250_CTL_ERROR;
    if (!present[0])
        return V250_CTL_OK;
    for (size_t i = 0; i < sizeof(ipr_rates) / sizeof(ipr_rates[0]); i++)
        if (vals[0] == ipr_rates[i]) {
            c->ipr = (int) vals[0];
            return V250_CTL_OK;
        }
    return V250_CTL_ERROR;
}

/* A compound parameter of up to three numeric fields, each with its own
 * supported range, stored as given (omitted fields keep their value). */
typedef struct {
    const char *name;
    int fields;
    int min[9];
    int max[9];
    bool first_required;    /* <value1> is not optional (+EWIND, +EFRAM) */
    const char *test;       /* a test response that is not a set of ranges */
    unsigned mask[9];       /* if set: exactly these values (small ones) */
} range_param_t;

static const range_param_t range_params[] = {
    { "EB",    3, { 0, 0, 0 },  { 0, 0, 0 },     false, NULL, { 0 } },
    { "EFCS",  1, { 0 },        { 0 },           false, NULL, { 0 } },
    { "ETBM",  3, { 0, 1, 0 },  { 0, 2, 30 },    false, NULL, { 0 } },
    { "EWIND", 2, { 1, 0 },     { 15, 15 },      true,  NULL, { 0 } },
    { "EFRAM", 2, { 1, 0 },     { 128, 128 },    true,  NULL, { 0 } },
    /* 6.2.11: a pty is 8-bit transparent, so 8N1 (or "auto"); parity is
       meaningless without a parity bit and any value is kept. */
    { "ICF",   2, { 0, 0 },     { 3, 3 },        false, "+ICF: (0,3),(0-3)", { BIT(0) | BIT(3), 0 } },
    /* 6.2.12: no XON/XOFF filtering; 2 is the pty's own back-pressure. */
    { "IFC",   2, { 0, 0 },     { 2, 2 },        false, "+IFC: (0,2),(0,2)", { BIT(0) | BIT(2), BIT(0) | BIT(2) } },
    /* 6.6.2: the stream method only (the packet methods are not
       implemented); the V.44 codec's own limits. */
    { "DS44",  9, { 0, 0, 0, 256, 256, 32, 32, 512, 512 },
                  { 3, 1, 0, 65535, 65535, 255, 255, 65535, 65535 }, false, NULL, { 0 } },
};

static int *range_store(v250_ctl_t *c, const char *name)
{
    if (!strcmp(name, "DS44"))  return c->ds44;
    if (!strcmp(name, "ICF"))   return c->icf;
    if (!strcmp(name, "IFC"))   return c->ifc;
    if (!strcmp(name, "EB"))    return c->eb;
    if (!strcmp(name, "EFCS"))  return &c->efcs;
    if (!strcmp(name, "ETBM"))  return c->etbm;
    if (!strcmp(name, "EWIND")) return c->ewind;
    return c->efram;
}

static v250_ctl_result_t range_command(v250_ctl_t *c, const range_param_t *p,
                                       const char *arg, char *info, size_t len)
{
    int *store = range_store(c, p->name);
    long vals[9] = { 0 };
    bool present[9] = { false };
    size_t used;
    int n;

    if (arg[0] == '?' && arg[1] == '\0') {
        used = (size_t) snprintf(info, len, "+%s: ", p->name);
        for (int i = 0; i < p->fields && used < len; i++)
            used += (size_t) snprintf(info + used, len - used, "%s%d", i ? "," : "", store[i]);
        return V250_CTL_OK;
    }
    if (!strcmp(arg, "=?") && p->test) {
        snprintf(info, len, "%s", p->test);
        return V250_CTL_OK;
    }
    if (!strcmp(arg, "=?")) {
        used = (size_t) snprintf(info, len, "+%s: ", p->name);
        for (int i = 0; i < p->fields && used < len; i++) {
            if (p->min[i] == p->max[i])
                used += (size_t) snprintf(info + used, len - used, "%s(%d)", i ? "," : "", p->min[i]);
            else
                used += (size_t) snprintf(info + used, len - used, "%s(%d-%d)", i ? "," : "",
                                          p->min[i], p->max[i]);
        }
        return V250_CTL_OK;
    }
    if (arg[0] != '=')
        return V250_CTL_ERROR;
    n = parse_compound(arg + 1, p->fields, vals, present);
    if (n < 0 || (p->first_required && !present[0]))
        return V250_CTL_ERROR;
    for (int i = 0; i < n; i++)
        if (present[i] && (vals[i] < p->min[i] || vals[i] > p->max[i]
                           || (p->mask[i] && !in_mask(vals[i], p->mask[i]))))
            return V250_CTL_ERROR;
    for (int i = 0; i < n; i++)
        if (present[i])
            store[i] = (int) vals[i];
    /* 6.5.7/6.5.8: value2 not included means value1 for both directions. */
    if (p->first_required && n == 1)
        store[1] = 0;
    return V250_CTL_OK;
}

void v250_ctl_link_params(const v250_ctl_t *c, int *tx_k, int *rx_k,
                          int *tx_n401, int *rx_n401)
{
    *tx_k = c->ewind[0];
    *rx_k = c->ewind[1] ? c->ewind[1] : c->ewind[0];
    *tx_n401 = c->efram[0];
    *rx_n401 = c->efram[1] ? c->efram[1] : c->efram[0];
}

/* One decimal field, strictly: digits only, no sign, no spaces.  An empty field
 * is "omitted" (*omitted set, value untouched).  Returns false on anything
 * else. */
static bool parse_field(const char *s, size_t len, long *value, bool *omitted)
{
    long v = 0;

    *omitted = (len == 0);
    if (len == 0)
        return true;
    if (len > 9)
        return false;
    for (size_t i = 0; i < len; i++) {
        if (!isdigit((unsigned char) s[i]))
            return false;
        v = v * 10 + (s[i] - '0');
    }
    *value = v;
    return true;
}

/* Splits "a,b,,d" into up to max fields.  Returns the field count, or -1 if
 * there are more than max or a field is malformed.  vals[i] is only written for
 * fields that are present; present[i] says which. */
static int parse_compound(const char *s, int max, long *vals, bool *present)
{
    int n = 0;
    const char *p = s;

    for (;;) {
        size_t len = strcspn(p, ",");
        bool omitted;

        if (n >= max)
            return -1;
        if (!parse_field(p, len, &vals[n], &omitted))
            return -1;
        present[n] = !omitted;
        n++;
        p += len;
        if (*p == '\0')
            break;
        p++;            /* the comma */
    }
    return n;
}

static bool in_mask(long v, unsigned mask)
{
    return v >= 0 && v < 32 && (mask & BIT((unsigned) v));
}

static v250_ctl_result_t flag_command(int *store, const char *name,
                                      const char *arg, char *info, size_t len)
{
    long vals[1] = { 0 };
    bool present[1] = { false };

    if (arg[0] == '?') {
        if (arg[1] != '\0')
            return V250_CTL_ERROR;
        snprintf(info, len, "+%s: %d", name, *store);
        return V250_CTL_OK;
    }
    if (arg[0] == '=' && arg[1] == '?' && arg[2] == '\0') {
        snprintf(info, len, "+%s: (0,1)", name);
        return V250_CTL_OK;
    }
    if (arg[0] != '=')
        return V250_CTL_ERROR;
    if (parse_compound(arg + 1, 1, vals, present) < 0)
        return V250_CTL_ERROR;
    if (present[0]) {
        if (vals[0] != 0 && vals[0] != 1)
            return V250_CTL_ERROR;
        *store = (int) vals[0];
    }
    return V250_CTL_OK;
}

static v250_ctl_result_t es_command(v250_ctl_t *c, const char *arg,
                                    char *info, size_t len)
{
    long vals[3] = { 0 };
    bool present[3] = { false };
    int n;

    if (arg[0] == '?' && arg[1] == '\0') {
        snprintf(info, len, "+ES: %d,%d,%d", c->es[0], c->es[1], c->es[2]);
        return V250_CTL_OK;
    }
    if (!strcmp(arg, "=?")) {
        snprintf(info, len, "+ES: (1-3),(0,2-3),(1-2,4-5)");
        return V250_CTL_OK;
    }
    if (arg[0] != '=')
        return V250_CTL_ERROR;
    n = parse_compound(arg + 1, 3, vals, present);
    if (n < 0)
        return V250_CTL_ERROR;
    if ((present[0] && !in_mask(vals[0], ES_RQST_OK))
        || (n > 1 && present[1] && !in_mask(vals[1], ES_ORIG_FBK_OK))
        || (n > 2 && present[2] && !in_mask(vals[2], ES_ANS_FBK_OK)))
        return V250_CTL_ERROR;
    for (int i = 0; i < 3; i++)
        if (i < n && present[i])
            c->es[i] = (int) vals[i];
    if (n > 0 && (present[0] || present[1] || present[2]))
        c->es_set = true;
    return V250_CTL_OK;
}

static v250_ctl_result_t ds_command(v250_ctl_t *c, const char *arg,
                                    char *info, size_t len)
{
    long vals[4] = { 0 };
    bool present[4] = { false };
    int n;

    if (arg[0] == '?' && arg[1] == '\0') {
        snprintf(info, len, "+DS: %d,%d,%d,%d", c->ds[0], c->ds[1], c->ds[2], c->ds[3]);
        return V250_CTL_OK;
    }
    if (!strcmp(arg, "=?")) {
        snprintf(info, len, "+DS: (0-3),(0,1),(%d-%d),(%d-%d)",
                 DS_DICT_MIN, DS_DICT_MAX, DS_STRING_MIN, DS_STRING_MAX);
        return V250_CTL_OK;
    }
    if (arg[0] != '=')
        return V250_CTL_ERROR;
    n = parse_compound(arg + 1, 4, vals, present);
    if (n < 0)
        return V250_CTL_ERROR;
    if ((present[0] && vals[0] > 3)
        || (n > 1 && present[1] && vals[1] > 1)
        || (n > 2 && present[2] && (vals[2] < DS_DICT_MIN || vals[2] > DS_DICT_MAX))
        || (n > 3 && present[3] && (vals[3] < DS_STRING_MIN || vals[3] > DS_STRING_MAX)))
        return V250_CTL_ERROR;
    for (int i = 0; i < 4; i++)
        if (i < n && present[i]) {
            c->ds[i] = (int) vals[i];
            c->ds_set = true;
        }
    return V250_CTL_OK;
}

v250_ctl_result_t v250_ctl_command(v250_ctl_t *c, const char *text,
                                   char *info, size_t info_len)
{
    char name[8];
    size_t n = 0;

    if (info_len > 0)
        info[0] = '\0';
    while (text[n] && isalnum((unsigned char) text[n]) && n < sizeof(name) - 1) {
        name[n] = (char) toupper((unsigned char) text[n]);
        n++;
    }
    name[n] = '\0';
    if (text[n] && isalnum((unsigned char) text[n]))
        return V250_CTL_UNKNOWN;        /* a longer name: not ours */

    if (!strcmp(name, "MR"))
        return flag_command(&c->mr, "MR", text + n, info, info_len);
    if (!strcmp(name, "ER"))
        return flag_command(&c->er, "ER", text + n, info, info_len);
    if (!strcmp(name, "DR"))
        return flag_command(&c->dr, "DR", text + n, info, info_len);
    if (!strcmp(name, "ES"))
        return es_command(c, text + n, info, info_len);
    if (!strcmp(name, "DS"))
        return ds_command(c, text + n, info, info_len);
    for (size_t i = 0; i < sizeof(range_params) / sizeof(range_params[0]); i++)
        if (!strcmp(name, range_params[i].name))
            return range_command(c, &range_params[i], text + n, info, info_len);
    if (!strcmp(name, "IPR"))
        return ipr_command(c, text + n, info, info_len);
    if (!strcmp(name, "ILRR"))
        return flag_command(&c->ilrr, "ILRR", text + n, info, info_len);
    if (!strcmp(name, "MSC"))
        return flag_command(&c->msc, "MSC", text + n, info, info_len);
    /* 6.4.2 +MA is optional and not implemented: every form is ERROR rather
       than an OK that changes nothing. */
    if (!strcmp(name, "MA"))
        return V250_CTL_ERROR;
    return V250_CTL_UNKNOWN;
}

void v250_ctl_ec_policy(const v250_ctl_t *c, bool calling_party,
                        v250_ec_policy_t *out)
{
    memset(out, 0, sizeof(*out));
    if (calling_party) {
        switch (c->es[0]) {
        case 1:
            out->attempt = false;
            return;
        case 2:
            out->attempt = true;
            out->detect = false;
            break;
        default:
            out->attempt = true;
            out->detect = true;
            break;
        }
        out->required = (c->es[1] == 2 || c->es[1] == 3);
    } else {
        if (c->es[2] == 1) {
            out->attempt = false;
            return;
        }
        out->attempt = true;
        out->detect = true;
        out->required = (c->es[2] == 4 || c->es[2] == 5);
    }
}

void v250_ctl_compression(const v250_ctl_t *c, bool calling_party,
                          v250_compression_t *out)
{
    int dir = c->ds[0];

    memset(out, 0, sizeof(*out));
    out->direction = dir;
    out->enabled = dir != 0;
    /* Annex A P0 bit 0 is initiator -> responder.  +DS <direction> is from this
     * DCE's own point of view: 1 transmit, 2 receive. */
    out->p0 = calling_party ? dir : (((dir & 1) << 1) | ((dir & 2) >> 1));
    out->p1 = c->ds[2];
    out->p2 = c->ds[3];
    out->required = out->enabled && c->ds[1] == 1;
}

void v250_ctl_v44(const v250_ctl_t *c, v250_v44_t *out)
{
    memset(out, 0, sizeof(*out));
    out->enabled = c->ds44[0] != 0;
    out->required = out->enabled && c->ds44[1] == 1;
    out->directions = c->ds44[0];
    out->tx_codewords = c->ds44[3];
    out->rx_codewords = c->ds44[4];
    out->tx_max_string = c->ds44[5];
    out->rx_max_string = c->ds44[6];
    out->tx_history = c->ds44[7];
    out->rx_history = c->ds44[8];
}

bool v250_ctl_v44_satisfied(const v250_ctl_t *c, int scheme, bool tx, bool rx)
{
    if (c->ds44[0] == 0 || c->ds44[1] == 0)
        return true;
    if (scheme != 2)
        return false;
    switch (c->ds44[0]) {
    case 1:
        return tx;
    case 2:
        return rx;
    default:
        return tx || rx;
    }
}

bool v250_ctl_compression_satisfied(const v250_ctl_t *c, bool tx, bool rx)
{
    switch (c->ds[0]) {
    case 1:
        return tx;
    case 2:
        return rx;
    case 3:
        return tx || rx;
    default:
        return true;
    }
}

size_t v250_ctl_format_report(const v250_ctl_t *c, const v250_connect_report_t *r,
                              char *out, size_t out_len)
{
    size_t n = 0;

#define EMIT(...) \
    do { \
        if (n < out_len) { \
            int w = snprintf(out + n, out_len - n, __VA_ARGS__); \
            if (w > 0) n += (size_t) w; \
        } \
    } while (0)

    if (out_len)
        out[0] = '\0';
    if (c->mr) {
        EMIT("+MCR: %s\r\n", r->carrier ? r->carrier : "V34");
        if (r->rx_rate > 0 && r->rx_rate != r->tx_rate)
            EMIT("+MRR: %d,%d\r\n", r->tx_rate, r->rx_rate);
        else
            EMIT("+MRR: %d\r\n", r->tx_rate);
    }
    if (c->er)
        EMIT("+ER: %s\r\n", r->ec ? r->ec : "NONE");
    if (c->dr) {
        const char *scheme = r->dc_scheme == 2 ? "V44" : "V42B";

        if (r->dc_scheme == 0 || (!r->dc_tx && !r->dc_rx))
            EMIT("+DR: NONE\r\n");
        else if (r->dc_tx && r->dc_rx)
            EMIT("+DR: %s\r\n", scheme);
        else
            EMIT("+DR: %s %s\r\n", scheme, r->dc_rx ? "RD" : "TD");
    }
    /* 6.2.13: after the modulation, error control and compression reports. */
    if (c->ilrr && r->dte_rate > 0)
        EMIT("+ILRR: %d\r\n", r->dte_rate);
#undef EMIT
    return n < out_len ? n : (out_len ? out_len - 1 : 0);
}

/* The Courier/Rockwell spellings.  Each maps onto the V.250 command it is
 * another name for and goes through v250_ctl_command(), so validation and
 * the es_set/ds_set bookkeeping are the V.250 path's own. */
v250_ctl_result_t v250_ctl_alias(v250_ctl_t *c, const char *text)
{
    static const struct {
        const char *name;
        int value;              /* -1: any value (subject to max) */
        const char *v250[2];    /* NULL, NULL: accepted, no effect */
    } map[] = {
        { "&M", 0, { "ES=1,0,1", NULL } },
        { "&M", 4, { "ES=3,0,2", NULL } },
        { "&M", 5, { "ES=3,2,4", NULL } },
        { "\\N", 0, { "ES=1,0,1", NULL } },
        { "\\N", 2, { "ES=3,2,4", NULL } },
        { "\\N", 3, { "ES=3,0,2", NULL } },
        { "\\N", 4, { "ES=3,3,5", NULL } },
        { "&K", 0, { "DS=0", "DS44=0" } },
        { "&K", 1, { "DS=3", NULL } },
        { "&K", 2, { "DS=3", NULL } },
        { "&K", 3, { "DS=3", NULL } },
        { "%C", 0, { "DS=0", "DS44=0" } },
        { "%C", 1, { "DS=3", NULL } },
        { "%C", 2, { "DS=3", NULL } },
        { "&H", 0, { "IFC=,0", NULL } },
        { "&H", 1, { "IFC=,2", NULL } },
        { "&R", 1, { "IFC=0", NULL } },
        { "&R", 2, { "IFC=2", NULL } },
        { "&I", 0, { NULL, NULL } },
        { "&B", 0, { NULL, NULL } },
        { "&B", 1, { NULL, NULL } },
        { "&B", 2, { NULL, NULL } },
    };
    static const char *const known[] = { "&M", "\\N", "&K", "%C", "&H", "&R", "&I", "&B", "&A" };
    char name[3];
    char *end;
    long v;
    bool ours = false;

    if (!text || strlen(text) < 3)
        return V250_CTL_UNKNOWN;
    name[0] = text[0];
    name[1] = (char) toupper((unsigned char) text[1]);
    name[2] = '\0';
    for (size_t i = 0; i < sizeof(known) / sizeof(known[0]); i++)
        ours |= !strcmp(name, known[i]);
    if (!ours)
        return V250_CTL_UNKNOWN;
    if (!isdigit((unsigned char) text[2]))
        return V250_CTL_ERROR;
    v = strtol(text + 2, &end, 10);
    if (*end)
        return V250_CTL_ERROR;
    if (!strcmp(name, "&A")) {
        if (v > 3)
            return V250_CTL_ERROR;
        c->arq = (int) v;
        return V250_CTL_OK;
    }
    for (size_t i = 0; i < sizeof(map) / sizeof(map[0]); i++) {
        v250_ctl_t trial = *c;
        char info[64];

        if (strcmp(map[i].name, name) || map[i].value != v)
            continue;
        for (int k = 0; k < 2 && map[i].v250[k]; k++)
            if (v250_ctl_command(&trial, map[i].v250[k], info, sizeof(info)) != V250_CTL_OK)
                return V250_CTL_ERROR;
        *c = trial;             /* all of it or none (5.4.4.2) */
        return V250_CTL_OK;
    }
    return V250_CTL_ERROR;
}

void v250_ctl_connect_suffix(const v250_ctl_t *c, const v250_connect_report_t *r,
                             char *out, size_t out_len)
{
    bool ec = r->ec && strcmp(r->ec, "NONE");
    size_t used = 0;

    if (out_len == 0)
        return;
    out[0] = '\0';
    if (c->arq >= 1 && ec)
        used += (size_t) snprintf(out + used, out_len - used, "/ARQ");
    if (c->arq >= 2 && r->carrier && r->carrier[0] && used < out_len)
        used += (size_t) snprintf(out + used, out_len - used, "/%s", r->carrier);
    if (c->arq >= 3 && ec && used < out_len) {
        used += (size_t) snprintf(out + used, out_len - used, "/%s", r->ec);
        if (r->dc_scheme && used < out_len)
            snprintf(out + used, out_len - used, "/%s", r->dc_scheme == 2 ? "V44" : "V42BIS");
    }
}
