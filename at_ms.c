/*
 * at_ms.c — V.250 6.4.1 +MS modulation selection, parsing and responses
 *
 * See at_ms.h.  One table row per carrier name a DTE may send, carrying the
 * engine mode it maps to with automode on and off and the carrier's highest
 * rate.  V.250 makes automode the DCE's licence to negotiate anything lower
 * than the carrier named, so a carrier the engine cannot run itself is still
 * accepted with automode 1 when something below it can (V.32bis falls back
 * to the V.22bis offer); with automode 0 it is ERROR rather than a promise
 * the modem cannot keep.  K56 is the same case for a different reason: the
 * engine runs K56flex's V.8bis identification and then ordinary V.8, so a
 * K56flex call always lands in V.90/V.34 and "K56 and nothing else" cannot
 * be honoured.
 */
#include "at_ms.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const struct {
    const char *carrier;    /* canonical name, as +MS? reports it */
    const char *alias[3];   /* other spellings accepted on input */
    const char *mode_exact; /* engine mode for automode 0, NULL = ERROR */
    const char *mode_auto;  /* engine mode for automode 1, NULL = ERROR */
    int max_rate;
} carriers[] = {
    { "V22",  { NULL },                       "v22", "v22",  1200 },
    { "V22B", { "V22BIS", NULL },             "v22", "v22",  2400 },
    { "V32",  { NULL },                       NULL,  "v22",  9600 },
    { "V32B", { "V32BIS", NULL },             NULL,  "v22", 14400 },
    { "V34",  { NULL },                       "v34", "v34", 33600 },
    { "K56",  { "56", "56K", "K56FLEX" },     NULL,  "k56", 56000 },
    { "V90",  { NULL },                       "v90", "v90", 56000 },
    { "V92",  { NULL },                       "v92", "v92", 56000 },
    { "V91",  { NULL },                       "v91", "v91", 64000 },
    { "X2",   { NULL },                       "x2",  "x2",  56000 },
};

#define N_CARRIERS (sizeof(carriers) / sizeof(carriers[0]))

static int find_carrier(const char *name)
{
    for (size_t i = 0; i < N_CARRIERS; i++) {
        if (!strcmp(name, carriers[i].carrier))
            return (int) i;
        for (size_t a = 0; a < 3 && carriers[i].alias[a]; a++)
            if (!strcmp(name, carriers[i].alias[a]))
                return (int) i;
    }
    return -1;
}

const char *at_ms_carrier_to_mode(const char *carrier, bool automode)
{
    int i = find_carrier(carrier);

    if (i < 0)
        return NULL;
    return automode ? carriers[i].mode_auto : carriers[i].mode_exact;
}

const char *at_ms_mode_to_carrier(const char *mode)
{
    /* v22 reads back as V22B: V.8 offers V.22 and V.22bis together and the
     * engine's V.22 path is SpanDSP's V.22bis modem. */
    if (!strcmp(mode, "v22"))
        return "V22B";
    for (size_t i = 0; i < N_CARRIERS; i++)
        if (carriers[i].mode_exact && !strcmp(mode, carriers[i].mode_exact))
            return carriers[i].carrier;
    for (size_t i = 0; i < N_CARRIERS; i++)
        if (carriers[i].mode_auto && !strcmp(mode, carriers[i].mode_auto))
            return carriers[i].carrier;
    return NULL;
}

int at_ms_carrier_max_rate(const char *carrier)
{
    int i = find_carrier(carrier);

    return i < 0 ? 0 : carriers[i].max_rate;
}

/* One decimal subparameter.  An empty one ("V34,,0,9600") leaves *val. */
static bool parse_rate(const char **t, int *val)
{
    const char *p = *t;
    long v = 0;
    int digits = 0;

    while (*p == ' ')
        p++;
    while (isdigit((unsigned char) *p)) {
        v = v * 10 + (*p - '0');
        if (v > AT_MS_MAX_RATE)
            return false;
        p++;
        digits++;
    }
    while (*p == ' ')
        p++;
    if (*p != ',' && *p != '\0')
        return false;
    if (digits)
        *val = (int) v;
    *t = p;
    return true;
}

at_ms_op_t at_ms_parse(const char *args, at_ms_settings_t *out)
{
    at_ms_settings_t s;
    const char *t = args;
    char name[sizeof(s.carrier)];
    size_t n = 0;
    int rates[4] = { 0, 0, 0, 0 };
    int nrates = 0;
    int ci;
    bool quoted = false;

    if (!t)
        return AT_MS_ERROR;
    if (!strcmp(t, "?"))
        return AT_MS_READ;
    if (!strcmp(t, "=?"))
        return AT_MS_TEST;
    if (*t++ != '=')
        return AT_MS_ERROR;

    while (*t == ' ')
        t++;
    if (*t == '"') {
        quoted = true;
        t++;
    }
    while (*t && *t != ',' && *t != '"' && *t != ' ') {
        if (n + 1 >= sizeof(name))
            return AT_MS_ERROR;
        name[n++] = (char) toupper((unsigned char) *t++);
    }
    name[n] = '\0';
    if (quoted && *t++ != '"')
        return AT_MS_ERROR;
    while (*t == ' ')
        t++;
    if ((ci = find_carrier(name)) < 0)
        return AT_MS_ERROR;

    memset(&s, 0, sizeof(s));
    strcpy(s.carrier, carriers[ci].carrier);
    s.automode = 1;

    if (*t == ',') {
        t++;
        while (*t == ' ')
            t++;
        if (*t == '0' || *t == '1')
            s.automode = *t++ - '0';
        while (*t == ' ')
            t++;
        if (*t != ',' && *t != '\0')
            return AT_MS_ERROR;
        while (*t == ',') {
            if (nrates == 4)
                return AT_MS_ERROR;
            t++;
            if (!parse_rate(&t, &rates[nrates]))
                return AT_MS_ERROR;
            nrates++;
        }
    }
    if (*t != '\0')
        return AT_MS_ERROR;
    if (!(s.automode ? carriers[ci].mode_auto : carriers[ci].mode_exact))
        return AT_MS_ERROR;

    if (nrates <= 2) {
        /* <min_rate>,<max_rate>: one pair for both directions */
        s.min_tx_rate = s.min_rx_rate = rates[0];
        s.max_tx_rate = s.max_rx_rate = rates[1];
    } else {
        s.min_tx_rate = rates[0];
        s.max_tx_rate = rates[1];
        s.min_rx_rate = rates[2];
        s.max_rx_rate = rates[3];
    }
    if ((s.max_tx_rate && s.min_tx_rate > s.max_tx_rate)
        || (s.max_rx_rate && s.min_rx_rate > s.max_rx_rate))
        return AT_MS_ERROR;
    /* Nothing above what the named carrier can carry.  V.250 lets a lower
     * minimum stand with automode on, since fallback is what it permits. */
    if (s.min_tx_rate > carriers[ci].max_rate || s.max_tx_rate > carriers[ci].max_rate
        || s.min_rx_rate > carriers[ci].max_rate || s.max_rx_rate > carriers[ci].max_rate)
        return AT_MS_ERROR;

    *out = s;
    return AT_MS_SET;
}

void at_ms_format_read(const at_ms_settings_t *s, char *buf, size_t len)
{
    snprintf(buf, len, "+MS: %s,%d,%d,%d,%d,%d", s->carrier, s->automode,
             s->min_tx_rate, s->max_tx_rate, s->min_rx_rate, s->max_rx_rate);
}

void at_ms_format_test(char *buf, size_t len)
{
    size_t used;

    used = (size_t) snprintf(buf, len, "+MS: (");
    for (size_t i = 0; i < N_CARRIERS && used < len; i++)
        used += (size_t) snprintf(buf + used, len - used, "%s%s",
                                  i ? "," : "", carriers[i].carrier);
    if (used < len)
        snprintf(buf + used, len - used,
                 "),(0,1),(0-%d),(0-%d),(0-%d),(0-%d)", AT_MS_MAX_RATE,
                 AT_MS_MAX_RATE, AT_MS_MAX_RATE, AT_MS_MAX_RATE);
}
