/*
 * at_ms.c — V.250 6.4.1 +MS modulation selection, parsing and responses
 *
 * See at_ms.h.  The carriers offered are the ones the engine can put in a
 * V.8 CM/JM: V22 and V22B (V.8's single V.22/V.22bis bit), V34, V90 and V92,
 * plus X2, which is not a V.250 carrier name but is what the engine's x2
 * mode reads back as, so that +MS? output can always be written back.
 */
#include "at_ms.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const struct {
    const char *carrier;
    const char *mode;
} carriers[] = {
    { "V22",  "v22" },
    { "V22B", "v22" },
    { "V34",  "v34" },
    { "V90",  "v90" },
    { "V92",  "v92" },
    { "X2",   "x2"  },
};

#define N_CARRIERS (sizeof(carriers) / sizeof(carriers[0]))

const char *at_ms_carrier_to_mode(const char *carrier)
{
    for (size_t i = 0; i < N_CARRIERS; i++)
        if (!strcmp(carrier, carriers[i].carrier))
            return carriers[i].mode;
    return NULL;
}

const char *at_ms_mode_to_carrier(const char *mode)
{
    /* v22 reads back as V22B: V.8 offers V.22 and V.22bis together and the
     * engine's V.22 path is SpanDSP's V.22bis modem. */
    if (!strcmp(mode, "v22"))
        return "V22B";
    for (size_t i = 0; i < N_CARRIERS; i++)
        if (!strcmp(mode, carriers[i].mode))
            return carriers[i].carrier;
    return NULL;
}

/* One decimal subparameter.  An empty one ("V34,,0,9600") keeps *val. */
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
    if (!at_ms_carrier_to_mode(name))
        return AT_MS_ERROR;

    memset(&s, 0, sizeof(s));
    strcpy(s.carrier, name);
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
