/*
 * fax_class2.h -- Service class 2.0 fax (ITU-T T.32) over the AT interface
 *
 * Class 1 (T.31, see data_interface.c) puts T.30 in the DTE: the DTE drives
 * every HDLC frame and image carrier itself with +FTH/+FRH/+FTM/+FRM.  Class
 * 2.0 is the other division of labour -- the DCE runs T.30, and the DTE only
 * declares its capabilities (+FCC), reads what was negotiated (+FCS), and
 * hands over or takes back page image data (+FDT/+FDR).
 *
 * That makes this module a T.32 command layer over SpanDSP's fax_state_t,
 * which is a complete T.30 terminal with the V.21/V.27ter/V.29/V.17 fax
 * datapumps under it.  The audio seam is the same one class 1 uses.
 *
 * Threading: the DTE bytes arrive on the data_interface reader thread and the
 * audio on the media thread.  fax_state_t is not thread safe, so everything
 * here takes one lock.
 */

#ifndef FAX_CLASS2_H
#define FAX_CLASS2_H

#include <stdint.h>
#include <stddef.h>

/* Writes a response (or a data-mode byte stream) back to the control PTY. */
typedef void (*fc2_write_cb_t)(const uint8_t *buf, int len, void *user_data);

/* Asks the engine to place or clear a call, for ATD/ATA/ATH in class 2.0. */
typedef void (*fc2_dial_cb_t)(const char *number, void *user_data);
typedef void (*fc2_action_cb_t)(void *user_data);

void fc2_init(fc2_write_cb_t write_cb,
              fc2_dial_cb_t dial_cb,
              fc2_action_cb_t answer_cb,
              fc2_action_cb_t hangup_cb,
              void *user_data);
void fc2_release(void);

/*
 * Class 2.0 is selected by AT+FCLASS=2.0 and left by AT+FCLASS=0.  While it
 * is selected the DTE's AT lines come here rather than to the class 1 (T.31)
 * interpreter, and the call's audio belongs to this module's T.30 terminal.
 */
void fc2_select(int on);
int  fc2_active(void);

/*
 * Feed one complete AT line (without the terminating CR).  Returns non-zero
 * if this module handled it; zero means the caller should pass it on -- the
 * non-fax commands (ATZ, ATE, ATQ, ATV, ATS) stay with the T.31 interpreter,
 * which owns their state.
 */
int fc2_at_line(const char *line);

/*
 * True while a +FDT or +FDR transfer is running, when the DTE's bytes are
 * DLE-stuffed image data rather than AT commands.  fc2_dte_bytes() consumes
 * them.
 */
int  fc2_in_dte_data(void);

/*
 * True while T.32 8.6 requires command echo to be off, which it does whenever
 * +FBU HDLC frame reporting is on: the reports are unsolicited and interleave
 * with whatever the DTE is typing.
 */
int  fc2_echo_suppressed(void);
void fc2_dte_bytes(const uint8_t *buf, int len);

/* Call progress from the engine. */
void fc2_on_connected(void);
void fc2_on_disconnected(void);

/* Audio, linear 16-bit PCM at 8 kHz.  fc2_tx() always fills the buffer. */
int fc2_rx(const int16_t *amp, int len);
int fc2_tx(int16_t *amp, int len);

/*
 * Deferred work: reports that become due while the media thread holds the
 * lock are queued and emitted from here, which the DTE reader thread calls.
 */
void fc2_poll(void);

#endif /* FAX_CLASS2_H */
