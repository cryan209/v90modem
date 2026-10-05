/*
 * data_interface.h — PTY creation and AT command interface
 *
 * Two DTE presentation modes:
 *
 *   Mode A (classic): one PTY carries AT commands and data inline. "+++"
 *   with 1 s guard times escapes online data to command mode; ATO returns
 *   online.
 *
 *   Mode B (split):  a control PTY that always speaks AT (RING/CONNECT and
 *   other unsolicited codes appear here, commands work mid-call) and a
 *   separate data PTY that carries only the connection payload.
 *
 * ATO (V.250 6.3.7): on the combined console it resumes the data connection
 * (CONNECT) or answers NO CARRIER with no call up; on the control port of a
 * split console it prints the data PTY's device path and answers OK.
 *
 * Uses SpanDSP's at_state_t for AT command parsing (ATD, ATA, ATH, ATZ, ...).
 */

#ifndef DATA_INTERFACE_H
#define DATA_INTERFACE_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "v250_ctl.h"

/* Opaque callbacks set by the modem engine */
typedef void (*di_dial_cb_t)(const char *sip_uri, void *user_data);
typedef void (*di_answer_cb_t)(void *user_data);
typedef void (*di_hangup_cb_t)(void *user_data);

/*
 * Mode A: open one PTY pair and start the reader thread.
 * symlink_path: path for the convenience symlink (e.g. "/tmp/modem0").
 * Returns 0 on success, -1 on error.
 */
int di_open(const char *symlink_path);

/*
 * Mode B: open a control PTY (AT commands, result codes) and a data PTY
 * (connection payload only). Returns 0 on success, -1 on error.
 */
int di_open_split(const char *control_symlink_path,
                  const char *data_symlink_path);

/* Close the PTY(s) and stop the reader thread. */
void di_close(void);

/* Register callbacks invoked when the AT interpreter requests modem actions. */
void di_set_callbacks(di_dial_cb_t  dial_cb,
                      di_answer_cb_t answer_cb,
                      di_hangup_cb_t hangup_cb,
                      void *user_data);

/*
 * AT+MS (V.250 6.4.1) modulation selection.  The engine registers these in
 * me_init(); with none registered, +MS answers ERROR.  set takes an engine
 * mode name ("v34", "v90", ...) and returns <0 to reject it; get reads the
 * offer the next call will make; reset restores the power-on default (ATZ,
 * AT&F).  Rate subparameters are kept here, reported by +MS?, and not
 * enforced -- see at_ms.h.
 */
typedef int  (*di_ms_set_cb_t)(const char *mode, bool automode);
typedef void (*di_ms_get_cb_t)(char *mode, size_t len, bool *automode);
typedef void (*di_ms_reset_cb_t)(void);
void di_set_modulation_ops(di_ms_set_cb_t set, di_ms_get_cb_t get,
                           di_ms_reset_cb_t reset);

/*
 * V.250 +MR/+ES/+ER/+DS/+DR (v250_ctl.h).  di_get_v250_settings() copies what
 * the DTE has configured for the next call.  The engine registers a callback
 * that di_on_connected() uses to learn what the call actually settled on --
 * carrier, directional rates, error control, compression -- so +MCR/+MRR/+ER/
 * +DR report the negotiated outcome, never the requested one.  On entry the
 * report holds the CONNECT rate as tx_rate and "NONE" error control.
 */
typedef void (*di_connect_info_cb_t)(int connect_rate, v250_connect_report_t *report);
void di_get_v250_settings(v250_ctl_t *out);
void di_set_connect_info_cb(di_connect_info_cb_t cb);

/*
 * Called by the modem engine when a connection is established.
 * rate: negotiated data rate in bit/s (e.g. 2400, 56000).
 * Sends CONNECT <rate> to the control port; in Mode A also switches the
 * port to DATA mode, in Mode B activates the data port.
 */
void di_on_connected(int rate);

/*
 * Called by the modem engine when the call ends.
 * Sends NO CARRIER and returns to command/idle state.
 */
void di_on_disconnected(void);

/*
 * Called by the modem engine when an incoming call rings.
 * Sends RING to the control port.
 */
void di_on_ring(void);

/*
 * Read DTE payload bytes (application → modem). Non-blocking; returns the
 * number of bytes copied (0 when idle, not in data transfer, or while the
 * V.250 +TLDL local DTE loop owns the payload route).
 */
int di_read_data(uint8_t *buf, int max_len);

/*
 * Write received payload bytes (modem → application). Non-blocking.
 * During +TLDL these bytes are consumed and clamped from the DTE.
 */
int di_write_data(const uint8_t *buf, int len);

/*
 * Fax service class (T.31).  di_fax_active() is non-zero once the DTE has
 * selected a fax class with AT+FCLASS=1 or 1.0; while it is, the call's
 * audio belongs to the T.31 fax datapumps rather than the data modem, and
 * the engine feeds them through di_fax_rx()/di_fax_tx().  Both take linear
 * 16-bit PCM at 8 kHz; di_fax_tx() always fills the whole buffer, padding
 * with silence when the fax transmitter has nothing to send.
 */
int di_fax_active(void);
int di_fax_rx(const int16_t *amp, int len);
int di_fax_tx(int16_t *amp, int len);
int di_fax_v34hdx_start_control(int primary_bit_rate);
int di_fax_v34hdx_get_bit(void);
void di_fax_v34hdx_put_bit(int bit);
int di_fax_v34hdx_get_mode(void);

#endif /* DATA_INTERFACE_H */
