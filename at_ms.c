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
    const char *offer;      /* what the next call offers, for +MS$ */
} carriers[] = {
    /* V.22 proper: the V.22bis datapump held at 1200 bit/s, which never
     * sends S1 and so trains as a V.22 modem against either (V.22bis
     * 6.3.1.1.1/6.3.1.2.1).  V.22 is the bottom of the ladder, so automode
     * changes nothing. */
    { "V22",  { NULL },                       "v22-1200", "v22-1200", 1200,
      "V.22 (1200)" },
    { "V22B", { "V22BIS", NULL },             "v22", "v22",  2400,
      "V.22/V.22bis" },
    /* V.32 is V.32bis's 9600/4800 subset and runs on the same datapump. */
    { "V32",  { NULL },                       "v32",    "v32",     9600,
      "V.32 (9600/4800), V.22bis; ,0: V.32" },
    { "V32B", { "V32BIS", NULL },             "v32bis", "v32bis", 14400,
      "V.32bis, V.22bis; ,0: V.32bis" },
    /* The Courier's pre-V.34 proprietary set (docs/courier_firmware_
     * analysis.md: HST, Terbo, V.FC).  No datapump: under automode they
     * fall back to V.32bis, which is what the Courier itself falls to. */
    { "HST",  { NULL },                       NULL,  "v32bis", 16800,
      "V.32bis, V.22bis (no HST here)" },
    { "V32TERBO", { "TERBO", "V32T", NULL },  NULL,  "v32bis", 19200,
      "V.32bis, V.22bis (no V.32terbo here)" },
    { "VFC",  { "V.FC", "VFAST", NULL },      NULL,  "v32bis", 28800,
      "V.32bis, V.22bis (no V.FC here)" },
    /* V34+/V34B: 33600, which V.34 (1996) already is. */
    { "V34",  { "V34+", "V34B", "V34BIS" },   "v34", "v34", 33600,
      "V.34, V.22bis; ,0: V.34" },
    /* 60000: the shipped MICA K56flex tables run past 56k -- rate indices
     * 32 and 33 (58000, 60000) exist for both laws, base pad group only
     * (k56flex_tables.h; k56flex_tx_init() accepts them). */
    { "K56",  { "56", "56K", "K56FLEX" },     NULL,  "k56", 60000,
      "K56flex V.8bis, then V.90" },
    { "V90",  { NULL },                       "v90", "v90", 56000,
      "V.90, V.34, V.22bis" },
    { "V92",  { NULL },                       "v92", "v92", 56000,
      "V.92/V.90, V.34, V.22bis" },
    { "V91",  { NULL },                       "v91", "v91", 64000,
      "V.91+V.90, V.34; ,0: V.91+V.34" },
    /* 64000: x2's digital symmetric mode (x2 up and down) runs PCM both
     * ways at up to 64000.  Only the asymmetric session (PCM down, V.34
     * up) exists here; docs/x2_implementation.md. */
    { "X2",   { NULL },                       "x2",  "x2",  64000,
      "x2 asym (symmetric 64k not here)" },
    /* Recognised, with no datapump in this engine.  Both modes NULL, so
     * always ERROR; +MS=? leaves them out and +MS$ lists them apart, so a
     * DTE that sends them learns why rather than meeting a bare ERROR.
     * Bell 103 is 300 bit/s with nothing below it to fall back to, and
     * 212A answers with 2225 Hz rather than V.8.  V.110 and X.75 would need
     * no datapump either (see CLEAR/V120 below), only their framing. */
    { "B103",  { NULL },                      NULL,  NULL,     300,
      "Bell 103" },
    { "B212",  { "B212A", NULL },             NULL,  NULL,    1200,
      "Bell 212A" },
    { "V110",  { NULL },                      NULL,  NULL,   64000,
      "ISDN V.110 rate adaption" },
    { "X75",   { NULL },                      NULL,  NULL,   64000,
      "ISDN X.75" },
    /* The bearer is a byte-exact 64 kbit/s DS0 (clear_channel.h), so these
     * need no datapump: no V.8, both ends set alike, as on ISDN.  A maximum
     * rate of 56000 or less selects restricted 56k (at_ms_settings_to_mode).
     * Automode means nothing without a negotiation, so either is accepted. */
    { "CLEAR", { "CLEARMODE", "64K", NULL },  "clear", "clear", 64000,
      "DS0 bits, V.14/LAPM; <=56000: 56k" },
    { "V120",  { NULL },                      "v120", "v120",   64000,
      "V.120 UI frames; <=56000: 56k" },
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
    /* v22 is V.22bis (V.8's one bit names V.22 and V.22bis together);
     * v22-1200 is V.22 and is found in the table. */
    if (!strcmp(mode, "v22"))
        return "V22B";
    if (!strcmp(mode, "v32bis"))
        return "V32B";
    if (!strcmp(mode, "clear56"))
        return "CLEAR";
    if (!strcmp(mode, "v120-56"))
        return "V120";
    for (size_t i = 0; i < N_CARRIERS; i++)
        if (carriers[i].mode_exact && !strcmp(mode, carriers[i].mode_exact))
            return carriers[i].carrier;
    for (size_t i = 0; i < N_CARRIERS; i++)
        if (carriers[i].mode_auto && !strcmp(mode, carriers[i].mode_auto))
            return carriers[i].carrier;
    return NULL;
}

static bool available(size_t i)
{
    return carriers[i].mode_exact || carriers[i].mode_auto;
}

bool at_ms_carrier_available(const char *carrier)
{
    int i = find_carrier(carrier);

    return i >= 0 && available((size_t) i);
}

const char *at_ms_settings_to_mode(const at_ms_settings_t *s)
{
    const char *mode = at_ms_carrier_to_mode(s->carrier, s->automode != 0);
    int max = s->max_tx_rate;

    if (s->max_rx_rate && (!max || s->max_rx_rate < max))
        max = s->max_rx_rate;
    /* V.22bis limited to 1200 bit/s is V.22. */
    if (mode && max && max <= 1200 && !strcmp(mode, "v22"))
        return "v22-1200";
    if (mode && max && max <= 56000) {
        if (!strcmp(mode, "clear"))
            return "clear56";
        if (!strcmp(mode, "v120"))
            return "v120-56";
    }
    return mode;
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
    if (!strcmp(t, "$"))
        return AT_MS_HELP;
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

    bool first = true;

    used = (size_t) snprintf(buf, len, "+MS: (");
    for (size_t i = 0; i < N_CARRIERS && used < len; i++) {
        if (!available(i))
            continue;
        used += (size_t) snprintf(buf + used, len - used, "%s%s",
                                  first ? "" : ",", carriers[i].carrier);
        first = false;
    }
    if (used < len)
        snprintf(buf + used, len - used,
                 "),(0,1),(0-%d),(0-%d),(0-%d),(0-%d)", AT_MS_MAX_RATE,
                 AT_MS_MAX_RATE, AT_MS_MAX_RATE, AT_MS_MAX_RATE);
}

/* One +MS$ table row. */
static size_t help_row(size_t i, char *buf, size_t len)
{
    char aliases[32] = "";
    size_t a_used = 0;
    const char *autos = !available(i) ? "-" : carriers[i].mode_exact ? "0,1" : "1";

    for (size_t a = 0; a < 3 && carriers[i].alias[a]; a++)
        a_used += (size_t) snprintf(aliases + a_used, sizeof(aliases) - a_used,
                                    "%s%s", a ? "," : "", carriers[i].alias[a]);
    return (size_t) snprintf(buf, len, "  %-8s %-16s %-5s %9d  %s\r\n",
                             carriers[i].carrier, aliases, autos,
                             carriers[i].max_rate, carriers[i].offer);
}

/* Courier-style help: syntax, then one row per carrier with its aliases,
 * the automodes it accepts, its maximum rate and what the next call offers,
 * then the names recognised but not available, then the current setting.
 * Built from the table above, so it cannot disagree with the parser. */
void at_ms_format_help(const at_ms_settings_t *cur, char *buf, size_t len)
{
    size_t used = 0;
    char cur_text[64];

#define HELP_PUT(...) do { \
        if (used < len) \
            used += (size_t) snprintf(buf + used, len - used, __VA_ARGS__); \
    } while (0)

    HELP_PUT("+MS  V.250 6.4.1 modulation selection\r\n");
    HELP_PUT("  +MS=<carrier>[,<automode>[,<min>,<max>]]\r\n");
    HELP_PUT("  +MS=<carrier>,<automode>,<min_tx>,<max_tx>,<min_rx>,<max_rx>\r\n");
    HELP_PUT("  +MS?  current   +MS=?  ranges   +MS$  this help\r\n");
    HELP_PUT("  <automode> 1 = may fall back to lower modulations (default)\r\n");
    HELP_PUT("             0 = the named carrier only\r\n");
    HELP_PUT("  <rate> bit/s, 0 = no limit, at most the carrier maximum;\r\n");
    HELP_PUT("         reported, not enforced -- training picks the rate\r\n");
    HELP_PUT("  Takes effect on the next call. ATZ, AT&F restore the default.\r\n");
    HELP_PUT("\r\n");
    HELP_PUT("  Carrier  Also             Auto  Max bit/s  Next call offers\r\n");
    for (size_t i = 0; i < N_CARRIERS; i++)
        if (available(i) && used < len)
            used += help_row(i, buf + used, len - used);
    HELP_PUT("  CLEAR, V120: no V.8 or negotiation -- set both ends alike, as on\r\n");
    HELP_PUT("  ISDN; the bearer must be byte-exact end to end (no transcoding).\r\n");
    HELP_PUT("\r\n  Recognised, no datapump here (always ERROR):\r\n");
    for (size_t i = 0; i < N_CARRIERS; i++)
        if (!available(i) && used < len)
            used += help_row(i, buf + used, len - used);
    if (cur) {
        at_ms_format_read(cur, cur_text, sizeof(cur_text));
        HELP_PUT("\r\n  Current: %s", cur_text + 5);   /* past "+MS: " */
    }
#undef HELP_PUT
}
