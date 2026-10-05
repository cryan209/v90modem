/*
 * at_help.c -- Courier-style "$" help pages.  See at_help.h.
 *
 * Keep each row honest about THIS modem.  The interpreter is SpanDSP's, which
 * parses a full Hayes/V.250 command set; several commands it accepts change
 * nothing here, because the line is a SIP call and the DTE port is a pty.
 * Those rows say so rather than describe the Hayes behaviour.
 */

#include "at_help.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define N(a) (sizeof(a) / sizeof((a)[0]))

/* AT$: the basic (single letter) commands, and the way to every other page. */
static const at_help_entry_t basic_rows[] = {
    { "$",       "This list",                                         NULL },
    { "D$",      "Dialling commands",                                 "D$" },
    { "&$",      "Ampersand (&) commands",                            "&$" },
    { "+$",      "Extended (+) commands",                             "+$" },
    { "+MS$",    "Modulations (generated from the +MS table)",        "+MS$" },
    { "I$",      "Identification and diagnostic (ATIn) pages",        "I$" },
    { "S$",      "S-registers, with current values",                  "S$" },
    { "A",       "Answer the ringing call",                           NULL },
    { "Dn",      "Dial n (see D$)",                                   NULL },
    { "E0/E1",   "Command echo off/on",                               "E0" },
    { "H0",      "Hang up, or end a dial/answer still in progress",   NULL },
    { "H1",      "Off-hook command state (no effect on the SIP line)", NULL },
    { "In",      "Identification/diagnostics (see I$)",               "I0" },
    { "Y11",     "Line spectrum: level per 150 Hz band (see I$)",     "Y11" },
    { "L0-3",    "Speaker volume (accepted; there is no speaker)",    NULL },
    { "M0-3",    "Speaker mode (accepted; there is no speaker)",      NULL },
    { "O0",      "Return to online data (NO CARRIER if no call)",     NULL },
    { "P",       "Pulse dial default (SIP sends the digits as-is)",   NULL },
    { "Q0/Q1",   "Result codes on/off",                               "Q0" },
    { "Sr=n",    "Set S-register r (see S$)",                         NULL },
    { "Sr?",     "Read S-register r",                                 "S0?" },
    { "T",       "Tone dial default",                                 "T" },
    { "V0/V1",   "Numeric/verbose result codes",                      "V1" },
    { "X0-4",    "X0 bare CONNECT; X1+ CONNECT rate; X3/4 BUSY; X2/4 NO DIALTONE", "X4" },
    { "Z",       "Reset to the power-on profile (Z0 only)",           NULL },
    { "+++",     "Escape to online command (S2 char, S12 guard)",      NULL },
    { "A/",      "Repeat the last command line (no Enter needed)",   NULL },
};

/* D$: what the dial parser takes, and what reaches the SIP INVITE. */
static const at_help_entry_t dial_rows[] = {
    { "D0-9 * #",  "Digits; sent as the SIP user part",               NULL },
    { "DA-D",      "DTMF A-D digits (tone dial only)",                NULL },
    { "D+",        "International prefix, kept in the number",        NULL },
    { "D,",        "Pause (S8); dropped -- a SIP call has no pause",  NULL },
    { "DT",        "Tone dialling (the default)",                     NULL },
    { "DP",        "Pulse dialling (also drops A-D * #)",             NULL },
    { "D space -", "Ignored, for readability",                        NULL },
    { "DW !",      "Accepted and ignored",                            NULL },
    { "D@",        "NO ANSWER if the far end does not answer; class 1 fax: no CNG", NULL },
    { "D;",        "Class 1 fax: stay in command state; otherwise ignored", NULL },
    { "DS=n DSn",  "Dial stored number n (+ASTO / &Zn); rest of line ignored", NULL },
    { "DL",        "Redial the last number; DL? shows it",            "DL?" },
    { "",          "The number goes to the --sip-server as sip:n@server.", NULL },
    { "",          "Any key aborts a dial in progress (OK); S7 bounds it (NO CARRIER).", NULL },
    { "",          "Busy: BUSY (X3/X4). Network refused: NO DIALTONE (X2/X4).", NULL },
    { "",          "Mode: +MS (see +MS$) and +FCLASS choose what the call runs.", NULL },
    { "",          "Stored numbers are kept until the modem restarts (no NVRAM).", NULL },
};

/* &$ */
static const at_help_entry_t amp_rows[] = {
    { "&C0/&C1", "DCD behaviour (stored; a pty has no DCD line)",     "&C1" },
    { "&D0-2",   "DTR behaviour (stored; a pty has no DTR line)",     "&D2" },
    { "&F",      "Factory defaults, incl. +MS/+ES/+DS and diagnostics", NULL },
    { "&V",      "View the active configuration (as ATI4)",            "&V" },
    { "&Zn=s",   "Store number s in slot n (0-9); &Zn? shows it",     "&Z0?" },
};

/* +$: only commands that do what V.250/T.31/T.32 says.  Accepted-but-ignored
   ones are named together at the end, as the audit lists them. */
static const at_help_entry_t plus_rows[] = {
    { "+GMI +GMM +GMR", "Manufacturer, model, revision",              "+GMI" },
    { "+GCAP",     "Capabilities: +FCLASS, +MS, +ES, +DS",            "+GCAP" },
    { "+ASTO=n,s", "Store dial string s in slot n (0-9); D S=n dials it", "+ASTO?" },
    { "+MS",       "Modulation for the next call (see +MS$)",         "+MS?" },
    { "+MR=0/1",   "Report +MCR/+MRR before CONNECT",                 "+MR?" },
    { "+ES=o,f,a", "Error control: request, fallback, answer mode",   "+ES?" },
    { "+ER=0/1",   "Report +ER (LAPM/NONE) before CONNECT",           "+ER?" },
    { "+EWIND=t,r", "V.42 window k offered per direction (1-15)",     "+EWIND?" },
    { "+EFRAM=t,r", "V.42 frame size N401 offered (1-128 octets)",    "+EFRAM?" },
    { "+ETBM=0,r,t", "Call end buffers: TX discarded, RX delivered",  "+ETBM?" },
    { "+EFCS=0",   "16-bit FCS (32-bit is not offered)",              "+EFCS?" },
    { "+EB=0,0,0", "Break handling: none (a pty carries no break)",   "+EB?" },
    { "+DS=d,n,s,l", "V.42bis: direction, required, dict, string",    "+DS?" },
    { "+DR=0/1",   "Report +DR before CONNECT",                       "+DR?" },
    { "+TLDL=0/1", "Local digital loop of the DTE data (in a call)",  "+TLDL?" },
    { "+TTER=t,l,n,p", "Bit/block error test on the loop",            "+TTER?" },
    { "+TNUM?",    "Error counts of the last test",                   "+TNUM?" },
    { "+TSELF=1",  "Partial self test; +TRES? reads the result",      "+TRES?" },
    { "+TMODE?",   "Test mode (point to point only)",                 "+TMODE?" },
    { "+FCLASS=n", "0 data, 1/1.0 T.31 fax, 2.0 T.32 fax",            "+FCLASS?" },
    { "+FTS +FRS", "Class 1: silence send/wait",                      NULL },
    { "+FTM +FRM", "Class 1: send/receive image modulation",          NULL },
    { "+FTH +FRH", "Class 1: send/receive HDLC",                      NULL },
    { "+FDT +FDR", "Class 2.0: send/receive a page",                  NULL },
    { "+FIS +FCC", "Class 2.0: session/capability parameters",        NULL },
    { "+FLI +FPI", "Class 2.0: local and polling IDs",                NULL },
    { "+FNR +FBU", "Class 2.0: negotiation and HDLC reports",         NULL },
    { "+FCT +FIE", "Class 2.0: phase C timeout, procedure interrupts", NULL },
    { "",          "Accepted, not yet applied: +IPR +ICF +IFC +MSC +MA +DS44", NULL },
    { "",          "  and the V.92 +P commands.", NULL },
};

/* I$: the ATIn pages data_interface.c answers. */
static const at_help_entry_t info_rows[] = {
    { "I0", "Product identification",                                 "I0" },
    { "I3", "Software version",                                       "I3" },
    { "I4", "Current settings",                                       "I4" },
    { "I6", "Link diagnostics, live during a call, else the last one", "I6" },
    { "I7", "Product configuration (what this build supports)",       "I7" },
    { "I11", "Extended link diagnostics (live during a call)",        "I11" },
    { "Y11", "Line level per 150 Hz band, 150-3900 Hz, RX and TX",    "Y11" },
};

/* S$: exactly the registers the interpreter has.  The rest are ERROR. */
typedef struct {
    int reg;
    const char *desc;
} s_row_t;

static const s_row_t s_rows[] = {
    { 0,  "Rings before auto-answer (0 = never)" },
    { 1,  "Rings counted on this call" },
    { 2,  "Escape character (43 = '+'; above 127 disables)" },
    { 3,  "Command line terminator (13 = CR)" },
    { 4,  "Response formatting character (10 = LF)" },
    { 5,  "Command line editing character (8 = BS)" },
    { 6,  "Pause before blind dialling, s (stored only)" },
    { 7,  "Connection completion timeout, s (NO CARRIER after)" },
    { 8,  "Comma pause, s (stored; SIP has no pause)" },
    { 10, "Carrier loss disconnect delay, 1/10 s (stored only)" },
    { 12, "Escape guard time, 1/50 s (50 = 1 s; 0 = none)" },
};

/* S$ rows as table rows, for the drift test. */
static const at_help_entry_t s_entries[] = {
    { "S0",  NULL, "S0?" },
    { "S1",  NULL, "S1?" },
    { "S2",  NULL, "S2?" },
    { "S3",  NULL, "S3?" },
    { "S4",  NULL, "S4?" },
    { "S5",  NULL, "S5?" },
    { "S6",  NULL, "S6?" },
    { "S7",  NULL, "S7?" },
    { "S8",  NULL, "S8?" },
    { "S10", NULL, "S10?" },
    { "S12", NULL, "S12?" },
};

const at_help_entry_t *at_help_table(const char *topic, size_t *n)
{
    if (!topic)
        return NULL;
    if (!strcmp(topic, "")) { *n = N(basic_rows); return basic_rows; }
    if (!strcmp(topic, "D")) { *n = N(dial_rows); return dial_rows; }
    if (!strcmp(topic, "&")) { *n = N(amp_rows); return amp_rows; }
    if (!strcmp(topic, "+")) { *n = N(plus_rows); return plus_rows; }
    if (!strcmp(topic, "I")) { *n = N(info_rows); return info_rows; }
    if (!strcmp(topic, "S")) { *n = N(s_entries); return s_entries; }
    return NULL;
}

typedef struct {
    char *p;
    size_t left;
    size_t used;
} sink_t;

static void put(sink_t *s, const char *fmt, ...)
{
    va_list ap;
    int n;

    va_start(ap, fmt);
    n = vsnprintf(s->p, s->left, fmt, ap);
    va_end(ap);
    if (n < 0)
        return;
    if ((size_t) n >= s->left)
        n = s->left ? (int) s->left - 1 : 0;
    s->p += n;
    s->left -= (size_t) n;
    s->used += (size_t) n;
}

static const char *title(const char *topic)
{
    if (!strcmp(topic, ""))  return "HELP, Command Quick Reference";
    if (!strcmp(topic, "D")) return "HELP, Dial Commands";
    if (!strcmp(topic, "&")) return "HELP, Ampersand Commands";
    if (!strcmp(topic, "+")) return "HELP, Extended Commands";
    if (!strcmp(topic, "I")) return "HELP, Identification and Diagnostics";
    return "HELP, S-Registers";
}

int at_help_format(const char *topic, const uint8_t *s_regs, char *out, size_t len)
{
    const at_help_entry_t *rows;
    size_t n;
    sink_t s = { out, len, 0 };

    if (!out || len == 0)
        return -1;
    out[0] = '\0';
    if (!(rows = at_help_table(topic, &n)))
        return -1;
    put(&s, "%s\r\n", title(topic));
    if (!strcmp(topic, "S")) {
        put(&s, "Reg  Value  Function\r\n");
        for (size_t i = 0; i < N(s_rows); i++)
            put(&s, "S%-3d %03d    %s\r\n", s_rows[i].reg,
                s_regs ? s_regs[s_rows[i].reg] : 0, s_rows[i].desc);
        put(&s, "Sr=n sets, Sr? reads; other registers answer ERROR.\r\n");
        return (int) s.used;
    }
    for (size_t i = 0; i < n; i++) {
        if (rows[i].cmd[0])
            put(&s, "%-14s %s\r\n", rows[i].cmd, rows[i].desc);
        else
            put(&s, "%s\r\n", rows[i].desc);
    }
    return (int) s.used;
}
