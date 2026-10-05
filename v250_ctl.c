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
}

/* A compound parameter of up to three numeric fields, each with its own
 * supported range, stored as given (omitted fields keep their value). */
typedef struct {
    const char *name;
    int fields;
    int min[3];
    int max[3];
    bool first_required;    /* <value1> is not optional (+EWIND, +EFRAM) */
} range_param_t;

static const range_param_t range_params[] = {
    { "EB",    3, { 0, 0, 0 },  { 0, 0, 0 },     false },
    { "EFCS",  1, { 0 },        { 0 },           false },
    { "ETBM",  3, { 0, 1, 0 },  { 0, 2, 30 },    false },
    { "EWIND", 2, { 1, 0 },     { 15, 15 },      true },
    { "EFRAM", 2, { 1, 0 },     { 128, 128 },    true },
};

static int *range_store(v250_ctl_t *c, const char *name)
{
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
    long vals[3] = { 0 };
    bool present[3] = { false };
    size_t used;
    int n;

    if (arg[0] == '?' && arg[1] == '\0') {
        used = (size_t) snprintf(info, len, "+%s: ", p->name);
        for (int i = 0; i < p->fields && used < len; i++)
            used += (size_t) snprintf(info + used, len - used, "%s%d", i ? "," : "", store[i]);
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
        if (present[i] && (vals[i] < p->min[i] || vals[i] > p->max[i]))
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
#undef EMIT
    return n < out_len ? n : (out_len ? out_len - 1 : 0);
}
