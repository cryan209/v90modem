/*
 * at_ms.h — V.250 6.4.1 +MS modulation selection, parsing and responses
 *
 * Pure text handling, no engine or SpanDSP dependency, so it can be unit
 * tested on its own (at_ms_test).  data_interface.c hands it the subparameter
 * text SpanDSP's +MS hook delivers; sip_modem.c applies the result to the
 * engine through me_set_modulation_offer().
 *
 * Syntax accepted:
 *   +MS=<carrier>[,<automode>[,<min_rate>[,<max_rate>]]]
 *   +MS=<carrier>[,<automode>[,<min_tx_rate>[,<max_tx_rate>
 *                              [,<min_rx_rate>[,<max_rx_rate>]]]]]
 *   +MS?    read:  +MS: <carrier>,<automode>,<min_tx>,<max_tx>,<min_rx>,<max_rx>
 *   +MS$    help:  syntax, and every carrier with its limits (Courier style)
 *   +MS=?   test:  +MS: (<carriers>),(0,1),(0-64000),(0-64000),(0-64000),(0-64000)
 *
 * The four-subparameter form is the common single-rate-pair variant: its
 * min/max apply to both directions.  V.250's own form is the six-subparameter
 * one, tx before rx.  Carriers may be quoted or bare.  Omitted subparameters
 * take their defaults (automode 1, rates 0) rather than the previous value,
 * so AT+MS=V34 means the same thing whatever was set before it.
 *
 * Carriers: V22, V22B, V32, V32B, HST, V32TERBO, VFC, V34 (also V34+,
 * V34B), K56 (also "56", "56K", "K56FLEX"), V90, V92, V91, X2.  V32, V32B,
 * HST, V32TERBO, VFC and K56 need automode 1: the engine has no datapump
 * for them, so they fall back (to the V.22bis offer; K56flex's V.8bis then
 * V.8 to V.90/V.34) and naming one alone is ERROR.  CLEAR (CLEARMODE, 64K)
 * and V120 skip V.8 and use the DS0 itself (clear_channel.h); a maximum
 * rate of 56000 or less selects restricted 56k.  B103, B212, V110 and X75
 * are recognised and always ERROR; +MS$ lists them.
 *
 * A rate of 0 means "no limit"; a rate above the carrier's maximum is ERROR.
 * Rates are otherwise stored and reported but NOT enforced: the engine
 * chooses rates from its own training.
 */
#ifndef AT_MS_H
#define AT_MS_H

#include <stdbool.h>
#include <stddef.h>

#define AT_MS_MAX_RATE 64000   /* V.91 */

typedef struct {
    char carrier[12];  /* canonical upper-case name, e.g. "V34" */
    int  automode;     /* 1: may fall back to lower modulations via V.8 */
    int  min_tx_rate, max_tx_rate;
    int  min_rx_rate, max_rx_rate;
} at_ms_settings_t;

typedef enum {
    AT_MS_ERROR = -1,
    AT_MS_SET   = 0,
    AT_MS_READ  = 1,
    AT_MS_TEST  = 2,
    AT_MS_HELP  = 3    /* +MS$, Courier-style help */
} at_ms_op_t;

/* args is the text after "+MS".  On AT_MS_SET, *out holds the new settings
 * (with defaults filled in); nothing is written otherwise. */
at_ms_op_t at_ms_parse(const char *args, at_ms_settings_t *out);

/* Information text for +MS? and +MS=? (without CR/LF). */
void at_ms_format_read(const at_ms_settings_t *s, char *buf, size_t len);
void at_ms_format_test(char *buf, size_t len);
/* +MS$ help, lines separated by CR LF, ending with the current setting when
 * cur is not NULL.  Sized for 4096 bytes. */
void at_ms_format_help(const at_ms_settings_t *cur, char *buf, size_t len);

/* Carrier (or alias) -> engine mode name for that automode ("V34" -> "v34"),
 * NULL if unknown or not available with that automode.  mode -> canonical
 * carrier, NULL if unknown. */
const char *at_ms_carrier_to_mode(const char *carrier, bool automode);
const char *at_ms_mode_to_carrier(const char *mode);
/* The engine mode a whole setting selects: the carrier's mode, except that
 * CLEAR and V120 with a maximum rate of 56000 or less select restricted 56k
 * ("clear56", "v120-56").  NULL if the carrier is unusable. */
const char *at_ms_settings_to_mode(const at_ms_settings_t *s);
/* Highest rate the carrier carries, 0 if unknown. */
/* Whether the engine can use it at all (false for B103, CLEAR, ...). */
bool at_ms_carrier_available(const char *carrier);
int at_ms_carrier_max_rate(const char *carrier);

#endif
