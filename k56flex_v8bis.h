/* K56flex V.8bis capability exchange as a sample-driven state machine.
 *
 * Draft 0.23 clauses 4.7-4.9: initiating side sends CRe, waits for CRd, sends
 * CL, receives MS and answers ACK1/NAK1; responding side detects CRe, sends
 * CRd, receives CL and sends MS or NAK1.  Messages are V.21 (initiator channel
 * 1, responder channel 2) with the HDLC framing of k56flex.h.
 *
 * Tone frequencies and segment lengths are the recovered oscillator
 * increments and block counts; levels, the CRe/CRd detection thresholds and
 * all waits are this implementation's, because the firmware ticks are not
 * calibrated (clause 4.7).  Nothing here has met a real peer.
 */
#ifndef K56FLEX_V8BIS_H
#define K56FLEX_V8BIS_H

#include "k56flex.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum { K56FLEX_V8BIS_INITIATE = 0, K56FLEX_V8BIS_RESPOND } k56flex_v8bis_role_t;

typedef enum {
    K56V8B_IDLE_WAIT,       /* initiator: quiet before CRe; responder: listening for CRe */
    K56V8B_DELAY_TX,        /* fixed wait before our first signal */
    K56V8B_TX_TONES,        /* CRe or CRd */
    K56V8B_WAIT_TONES,      /* initiator: waiting for CRd */
    K56V8B_WAIT_PEER_MSG,   /* waiting for CL (responder) or MS (initiator) */
    K56V8B_TX_MSG,          /* sending CL/MS/ACK1/NAK1 */
    K56V8B_WAIT_ACK,        /* responder: collecting a trailing ACK1/NAK1 */
    K56V8B_DONE,
    K56V8B_FAILED
} k56flex_v8bis_state_t;

typedef enum {
    K56V8B_OK = 0,
    K56V8B_NO_CRE,          /* responder heard no CRe in the listen window */
    K56V8B_NO_CRD,          /* initiator heard no CRd */
    K56V8B_NO_MESSAGE,      /* peer CL/MS never arrived */
    K56V8B_BAD_PEER         /* CL/MS arrived but failed the peer-field predicate */
} k56flex_v8bis_result_t;

typedef struct k56flex_v8bis_s k56flex_v8bis_t;

typedef struct {
    k56flex_v8bis_role_t role;
    int mu_law;             /* octet-18 law bit we advertise */
    int v90_capable;        /* octet-12 capability bit we advertise */
    int client_model;       /* test only: send the client version octet (0x02) and accept the
                             * server's (0x42).  Default is the digital-side server, whichever
                             * role it plays on a call. */
    int blind;              /* initiator: if no CRd arrives, send CL anyway and wait for MS */
    unsigned listen_ms;     /* responder CRe window / initiator CRd window; 0 = defaults */
} k56flex_v8bis_cfg_t;

k56flex_v8bis_t *k56flex_v8bis_new(const k56flex_v8bis_cfg_t *cfg);
void k56flex_v8bis_free(k56flex_v8bis_t *s);
/* Feed received 8 kHz linear samples. */
void k56flex_v8bis_rx(k56flex_v8bis_t *s, const int16_t *amp, int len);
/* Produce `len` transmit samples (silence when nothing is being sent).  The
 * clock advances here; call once per rx block with the same length. */
void k56flex_v8bis_tx(k56flex_v8bis_t *s, int16_t *amp, int len);

k56flex_v8bis_state_t k56flex_v8bis_state(const k56flex_v8bis_t *s);
k56flex_v8bis_result_t k56flex_v8bis_result(const k56flex_v8bis_t *s);
const char *k56flex_v8bis_state_name(k56flex_v8bis_state_t st);
const char *k56flex_v8bis_result_name(k56flex_v8bis_result_t r);
/* Peer CL/MS payload (FCS stripped); NULL until one has been received. */
const uint8_t *k56flex_v8bis_peer_payload(const k56flex_v8bis_t *s, size_t *len);
/* 1 = peer sent ACK1, 2 = NAK1, 0 = none seen.  For the initiator this is what
 * we sent. */
int k56flex_v8bis_ack(const k56flex_v8bis_t *s);
/* Server-side peer-field acceptance (AC6D checks the client's CL or MS this way,
 * whichever direction it arrives; a client_model object instead requires the
 * server value 0x42).  AC6D's documented predicate (offset 13 is 0x02 or
 * 0x21; the server value 0x42 and 0x00/0x01 are rejected), plus type nibble
 * MS/CL and the minimum length. */
bool k56flex_v8bis_peer_acceptable(const uint8_t *payload, size_t len);
/* Milliseconds from our clock start to the last state change (for logs). */
unsigned k56flex_v8bis_elapsed_ms(const k56flex_v8bis_t *s);

#endif
