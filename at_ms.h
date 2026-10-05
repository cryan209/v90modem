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
 *   +MS=?   test:  +MS: (<carriers>),(0,1),(0-56000),(0-56000),(0-56000),(0-56000)
 *
 * The four-subparameter form is the common single-rate-pair variant: its
 * min/max apply to both directions.  V.250's own form is the six-subparameter
 * one, tx before rx.  Carriers may be quoted or bare.  Omitted subparameters
 * take their defaults (automode 1, rates 0) rather than the previous value,
 * so AT+MS=V34 means the same thing whatever was set before it.
 *
 * A rate of 0 means "no limit".  Rates are range-checked, stored and reported
 * but NOT enforced: the engine chooses rates from its own training.
 */
#ifndef AT_MS_H
#define AT_MS_H

#include <stdbool.h>
#include <stddef.h>

#define AT_MS_MAX_RATE 56000

typedef struct {
    char carrier[8];   /* canonical upper-case name, e.g. "V34" */
    int  automode;     /* 1: may fall back to lower modulations via V.8 */
    int  min_tx_rate, max_tx_rate;
    int  min_rx_rate, max_rx_rate;
} at_ms_settings_t;

typedef enum {
    AT_MS_ERROR = -1,
    AT_MS_SET   = 0,
    AT_MS_READ  = 1,
    AT_MS_TEST  = 2
} at_ms_op_t;

/* args is the text after "+MS".  On AT_MS_SET, *out holds the new settings
 * (with defaults filled in); nothing is written otherwise. */
at_ms_op_t at_ms_parse(const char *args, at_ms_settings_t *out);

/* Information text for +MS? and +MS=? (without CR/LF). */
void at_ms_format_read(const at_ms_settings_t *s, char *buf, size_t len);
void at_ms_format_test(char *buf, size_t len);

/* Carrier <-> engine mode name ("V34" <-> "v34").  NULL if unknown. */
const char *at_ms_carrier_to_mode(const char *carrier);
const char *at_ms_mode_to_carrier(const char *mode);

#endif
