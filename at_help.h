/*
 * at_help.h -- Courier-style "$" help for the DTE command interface.
 *
 * AT$ lists the basic commands and how to reach the other help pages; D$, &$,
 * +$, I$ and S$ list the dialling modifiers, ampersand commands, extended
 * commands, identification/diagnostic pages and S-registers.  +MS$ stays in
 * at_ms.c, generated from its own carrier table.
 *
 * None of this is V.250: "$" is a manufacturer extension in the style of the
 * USRobotics Courier.  V.250 5.1 lets a manufacturer add commands, so long as
 * they do not take a name the Recommendation reserves, and none of these do.
 *
 * Every row says what THIS modem does with the command, not what a Hayes modem
 * would: a command the interpreter accepts and then ignores (a speaker, a
 * DCD line a pty does not have) says so.  Each row also carries a probe -- a
 * harmless command line that must answer OK -- so a test can check that the
 * help never lists a command the interpreter does not accept.
 */

#ifndef AT_HELP_H
#define AT_HELP_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
    const char *cmd;        /* as typed after "AT", e.g. "E0/E1" */
    const char *desc;       /* what it does here */
    const char *probe;      /* a command line (no "AT") that answers OK, or NULL */
} at_help_entry_t;

/* The rows behind one help page.  topic is "" (AT$), "D", "&", "+", "I" or "S".
   Returns NULL for a topic with no page. */
const at_help_entry_t *at_help_table(const char *topic, size_t *n);

/* Format a page as response text (CR LF line ends, no final OK).  s_regs is the
   interpreter's S-register array, read for S$'s current values; it may be NULL
   for the other pages.  Returns the length written, or -1 for an unknown
   topic. */
int at_help_format(const char *topic, const uint8_t *s_regs, char *out, size_t len);

#endif
