/*
 * v8bis_modem.h -- V.8bis at sample level: the audio layer under the
 * transaction state machine.
 *
 * Feed it received 8 kHz linear samples, pull transmit samples from it, and it
 * runs one V.8bis station: the stage 1 tone detector and generator, V.21(L) and
 * V.21(H) FSK with HDLC framing, the 100 ms marking preamble that doubles as
 * segment 2 of an ES signal (7.2.4), the 1.5 s echo-suppressor gap (9.4), the
 * 400 ms silence before MRe/CRe and their retransmission (10.2.2), and the
 * V.8bis state machine on top.  Nothing here knows about the rest of the modem
 * engine; stage 3b wires it in.
 *
 * rx and tx are called in step, the same number of samples each (the engine's
 * 20 ms blocks).  The transmit side is the clock.
 *
 * Which V.21 channel carries what (7.2): an initiating station transmits on
 * V.21(L) and listens on V.21(H); a responding station the reverse.  A station
 * in the Initial State listens on V.21(L) for an initiator's message.  Frames
 * from the channel a station is itself using are ignored, so a hybrid's echo
 * of our own message is not mistaken for the peer's.
 */
#ifndef V8BIS_MODEM_H
#define V8BIS_MODEM_H

#include "v8bis_fsm.h"

typedef struct {
    v8bis_fsm_cfg_t fsm;
    double level_dbm0;            /* nominal transmit level of a continuous signal (default -13) */
    unsigned initial_silence_ms;  /* 10.2.2: at least 400 before MRe/CRe (default 400) */
    unsigned retransmit_ms;       /* 10.2.2: resend MRe/CRe after 3 s of nothing (default 3000) */
    unsigned retries;             /* how many times (default 2) before giving up */
    unsigned es_gap_ms;           /* 9.4: silence after ES with an echo suppressor (default 1500) */
    unsigned open_flags;          /* 7.2.5: 2-5 (default 2) */
    unsigned close_flags;         /* 1-3 (default 1) */
} v8bis_modem_cfg_t;

typedef enum {
    V8BIS_MEV_MODE,               /* a transaction ended in MS mode: `mode` */
    V8BIS_MEV_INITIAL,            /* back in Initial: `why` and `nak` */
    V8BIS_MEV_NO_PEER,            /* nobody answered (after the retries) */
    V8BIS_MEV_RX_SIGNAL,          /* a tone signal was detected (diagnostic) */
    V8BIS_MEV_RX_MESSAGE,         /* a good message arrived: `msg_type` (diagnostic) */
    V8BIS_MEV_RX_BAD_FRAME        /* a frame failed 7.2.9 and NAK(1) was queued (diagnostic) */
} v8bis_modem_event_type_t;

typedef struct {
    v8bis_modem_event_type_t type;
    uint64_t at_sample;
    v8bis_mode_t mode;
    v8bis_why_t why;
    unsigned nak;
    v8bis_signal_t sig;
    bool responding_set;
    unsigned msg_type;
} v8bis_modem_event_t;

typedef struct v8bis_modem_s v8bis_modem_t;

void v8bis_modem_cfg_default(v8bis_modem_cfg_t *cfg);
v8bis_modem_t *v8bis_modem_new(const v8bis_modem_cfg_t *cfg);
void v8bis_modem_free(v8bis_modem_t *m);

/* Start a transaction (Initial State only). */
bool v8bis_modem_initiate(v8bis_modem_t *m, v8bis_init_t how);
void v8bis_modem_rx(v8bis_modem_t *m, const int16_t *amp, int len);
/* Fill `len` samples (silence when there is nothing to send) and advance the clock. */
void v8bis_modem_tx(v8bis_modem_t *m, int16_t *amp, int len);
bool v8bis_modem_event(v8bis_modem_t *m, v8bis_modem_event_t *ev);

/* The ANS/ANSam of the following start-up has begun (9.7). */
void v8bis_modem_startup_signal(v8bis_modem_t *m);

bool v8bis_modem_tx_busy(const v8bis_modem_t *m);
uint64_t v8bis_modem_now(const v8bis_modem_t *m);
const v8bis_fsm_t *v8bis_modem_fsm(const v8bis_modem_t *m);

#endif
