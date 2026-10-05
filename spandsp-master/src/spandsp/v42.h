/*
 * SpanDSP - a series of DSP components for telephony
 *
 * v42.h
 *
 * Written by Steve Underwood <steveu@coppice.org>
 *
 * Copyright (C) 2003, 2011 Steve Underwood
 *
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License version 2.1,
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/*! \page v42_page V.42 modem error correction
\section v42_page_sec_1 What does it do?
The V.42 specification defines an error correcting protocol for PSTN modems, based on
HDLC and LAP. This makes it similar to an X.25 link. A special variant of LAP, known
as LAP-M, is defined in the V.42 specification. A means for modems to determine if the
far modem supports V.42 is also defined.

\section v42_page_sec_2 How does it work?
*/

#if !defined(_SPANDSP_V42_H_)
#define _SPANDSP_V42_H_

typedef struct v42_state_s v42_state_t;

/* Positive V.42-specific status callback values.  Link lifecycle events use
 * the public negative SIG_STATUS_LINK_* values from async.h. */
typedef enum
{
    V42_STATUS_DETECTING = 0,
    V42_STATUS_DETECTION_SUCCEEDED = 1,
    V42_STATUS_DETECTION_UNSUPPORTED = 2,
    V42_STATUS_XID_NEGOTIATED = 3
} v42_status_t;

/* V.44 Annex A: parameters are relative to the sending/local endpoint,
 * unlike V.42bis P0, which is relative to the XID initiator. */
typedef struct v42_v44_parameters_s
{
    int capability;
    int directions;
    int tx_codewords, rx_codewords;
    int tx_max_string, rx_max_string;
    int tx_history, rx_history;
} v42_v44_parameters_t;

typedef struct
{
    bool valid;
    int tx_n401;
    int rx_n401;
    int tx_window_size_k;
    int rx_window_size_k;
    int xid_optional_functions_octets;
    int compression_p0;
    int compression_p1;
    int compression_p2;
    bool v44_valid;
    v42_v44_parameters_t v44;
} v42_negotiated_parameters_t;

#if defined(__cplusplus)
extern "C"
{
#endif

SPAN_DECLARE(const char *) lapm_status_to_str(int status);

SPAN_DECLARE(const char *) v42_status_to_str(int status);

SPAN_DECLARE(void) lapm_receive(void *user_data, const uint8_t *frame, int len, int ok);

SPAN_DECLARE(void) v42_start(v42_state_t *s);

SPAN_DECLARE(void) v42_stop(v42_state_t *s);

/*! Set the busy status of the local end of a V.42 context.
    \param s The V.42 context.
    \param busy The new local end busy status.
    \return The previous local end busy status.
*/
SPAN_DECLARE(bool) v42_set_local_busy_status(v42_state_t *s, bool busy);

/*! Get the busy status of the far end of a V.42 context.
    \param s The V.42 context.
    \return The far end busy status.
*/
SPAN_DECLARE(bool) v42_get_far_busy_status(v42_state_t *s);

SPAN_DECLARE(void) v42_rx_bit(void *user_data, int bit);

SPAN_DECLARE(int) v42_tx_bit(void *user_data);

/*! Set the synchronous transmit bit rate used to express V.42 timers in
    clocked bit calls.  Configure this before v42_restart().
    \param s The V.42 context.
    \param bit_rate Positive line transmit bit rate in bit/s.
    \return 0 on success, or -1 for an invalid rate. */
SPAN_DECLARE(int) v42_set_bit_rate(v42_state_t *s, int bit_rate);

/*! Select the XID optional-functions encoding length. Default 0 starts with
    V.42 (03/2002) Table 11a's four octets and renegotiates before establishment
    when a peer advertises the older three-octet format. Values 3 or 4 force it.
    Call before v42_restart(). Returns -1 for invalid arguments. */
SPAN_DECLARE(int) v42_set_xid_optional_functions_octets(v42_state_t *s, int octets);

/*! Set the LAP.M window sizes (k) and maximum information field lengths
    (N401, octets) this end offers in XID, per direction (V.250 +EWIND,
    +EFRAM).  Takes effect when the link is next established.
    eturn 0, or -1 if a value is outside 1..V42_MAX_WINDOW_SIZE_K or
            1..V42_MAX_N_401 (15 and 128 here). */
SPAN_DECLARE(int) v42_set_link_parameters(v42_state_t *s, int tx_k, int rx_k,
                                          int tx_n401, int rx_n401);

/*! Set N400, the maximum number of retransmissions (V.42 9.2.2, minimum 1).
    \return 0 on success, -1 for an out-of-range value. */
SPAN_DECLARE(void) v42_restart_t400(v42_state_t *s);

SPAN_DECLARE(int) v42_set_t400(v42_state_t *s, int t400_ms);

SPAN_DECLARE(int) v42_set_n400(v42_state_t *s, int n400);

/*! Configure the V.42bis offer before v42_restart. P0 is relative to the
    XID initiator; zero disables compression. The application must attach
    codecs when enabling it. Limits follow V.42bis 5.1/Annex A. */
SPAN_DECLARE(int) v42_set_compression(v42_state_t *s, int p0, int p1, int p2);

/*! Select V.44 stream compression, replacing the V.42bis offer. Call before
 * restart. C0 must be zero: only stream method with XID negotiation is offered.
 * Values are local TX/RX proposals (V.44 7.4, Annex A, Cor.1/2002). */
SPAN_DECLARE(int) v42_set_v44(v42_state_t *s, const v42_v44_parameters_t *parameters);

/*! Return the transmit bit rate currently used by the V.42 timers. */
SPAN_DECLARE(int) v42_get_bit_rate(const v42_state_t *s);

/*! Copy the parameters most recently agreed through XID.
    \return 0 when a valid XID result is available, otherwise -1. */
SPAN_DECLARE(int) v42_get_negotiated_parameters(const v42_state_t *s,
                                                v42_negotiated_parameters_t *out);

SPAN_DECLARE(void) v42_set_status_callback(v42_state_t *s, span_modem_status_func_t callback, void *user_data);

/*! Get the logging context associated with a V.42 context.
    \brief Get the logging context associated with a V.42 context.
    \param s The V.42 context.
    \return A pointer to the logging context */
SPAN_DECLARE(logging_state_t *) v42_get_logging_state(v42_state_t *s);

/*! Initialise a V.42 context.
    \param s The V.42 context.
    \param calling_party True if caller mode, else answerer mode.
    \param detect True to perform the V.42 detection, else go straight into LAP.M
    \param iframe_get A callback function to get frames for transmission.
    \param iframe_put A callback function to handle received frames of data.
    \param user_data An opaque pointer passed to the frame handler routines.
    \return ???
*/
SPAN_DECLARE(v42_state_t *) v42_init(v42_state_t *s,
                                     bool calling_party,
                                     bool detect,
                                     span_get_msg_func_t iframe_get,
                                     span_put_msg_func_t iframe_put,
                                     void *user_data);

/*! Restart a V.42 context.
    \param s The V.42 context.
*/
SPAN_DECLARE(void) v42_restart(v42_state_t *s);

/*! V.92 9.10.3 (Amd.2): suspend error correction for modem-on-hold.  The
    running acknowledgement timer is frozen; while suspended v42_tx_bit()
    returns marks without clocking anything and v42_rx_bit() discards. */
SPAN_DECLARE(void) v42_suspend(v42_state_t *s);

/*! Resume after modem-on-hold at the new line rate (0 keeps the old one).
    No XID, no SABME, and the C/R addresses of the original connection are
    kept whatever the new physical roles (9.10.3).  A frame cut off by the
    suspension is discarded at both ends; if I-frames are outstanding an
    enquiry (RR/RNR with P=1) goes out at once rather than after T401. */
SPAN_DECLARE(void) v42_resume(v42_state_t *s, int bit_rate);

SPAN_DECLARE(bool) v42_is_suspended(const v42_state_t *s);

/*! Release a V.42 context.
    \param s The V.42 context.
    \return 0 if OK */
SPAN_DECLARE(int) v42_release(v42_state_t *s);

/*! Free a V.42 context.
    \param s The V.42 context.
    \return 0 if OK */
SPAN_DECLARE(int) v42_free(v42_state_t *s);

#if defined(__cplusplus)
}
#endif

#endif
/*- End of file ------------------------------------------------------------*/
