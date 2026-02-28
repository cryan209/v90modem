/*
 * data_interface.h — PTY creation and AT command interface
 *
 * Creates a virtual serial port (PTY) that applications can open as a modem.
 * Uses SpanDSP's at_state_t for AT command parsing (ATD, ATA, ATH, ATZ, ...).
 * Switches between COMMAND mode and DATA mode when a connection is established.
 */

#ifndef DATA_INTERFACE_H
#define DATA_INTERFACE_H

#include <stdint.h>
#include <stddef.h>

/* Opaque callbacks set by the modem engine */
typedef void (*di_dial_cb_t)(const char *sip_uri, void *user_data);
typedef void (*di_answer_cb_t)(void *user_data);
typedef void (*di_hangup_cb_t)(void *user_data);

/*
 * Open a PTY pair and start the reader thread.
 * symlink_path: path for the convenience symlink (e.g. "/tmp/modem0").
 * Returns 0 on success, -1 on error.
 */
int di_open(const char *symlink_path);

/* Close the PTY and stop the reader thread. */
void di_close(void);

/* Register callbacks invoked when the AT interpreter requests modem actions. */
void di_set_callbacks(di_dial_cb_t  dial_cb,
                      di_answer_cb_t answer_cb,
                      di_hangup_cb_t hangup_cb,
                      void *user_data);

/*
 * Called by the modem engine when a connection is established.
 * rate: negotiated data rate in bit/s (e.g. 2400, 56000).
 * Sends CONNECT <rate> to the PTY and switches to DATA mode.
 */
void di_on_connected(int rate);

/*
 * Called by the modem engine when the call ends.
 * Sends NO CARRIER to the PTY and switches back to COMMAND mode.
 */
void di_on_disconnected(void);

/*
 * Called by the modem engine when an incoming call rings.
 * Sends RING to the PTY; if S0 register > 0, auto-answers.
 */
void di_on_ring(void);

/*
 * Read bytes from the PTY (upstream: application → modem).
 * Returns number of bytes copied (0 if none available).
 * Non-blocking; only valid in DATA mode.
 */
int di_read_data(uint8_t *buf, int max_len);

/*
 * Write bytes to the PTY (downstream: modem → application).
 * Returns number of bytes written.
 * Non-blocking; only valid in DATA mode.
 */
int di_write_data(const uint8_t *buf, int len);

#endif /* DATA_INTERFACE_H */
