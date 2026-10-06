/*
 * v92_mh.h — V.92 modem-on-hold (8.9, 9.10; Amd.1 items 7-9; Cor.1 items 2-4)
 *
 * Three layers, none of which touches audio:
 *
 *  - the MH sequence codec, Table 32 as replaced by Amd.1 item 7: 4 fill
 *    ones, the 01110010 frame sync, 4 signal indication bits, 4 information
 *    bits, the V.34 10.1.2.3.2 CRC over bits 12:19, 4 fill ones -- 40 bits,
 *    bit 0 first in time.  Table 33's T1 encoding and Table 34's
 *    initiating/response pairs are here too;
 *  - a receive framer for the continuous back-to-back stream 9.10.1
 *    requires ("the first 4 fill bits immediately following the last 4 fill
 *    bits of the preceding sequence");
 *  - the 9.10.1/9.10.2 transaction controller, driven by detector states and
 *    decoded frames and reporting what to transmit and what the engine must
 *    do next.
 *
 * 8.9.2 puts MH on the Phase 2 INFO modulation (8.2.3.1/V.90, 600 bit/s
 * DPSK) and 8.9.1 makes Tone RT the modem's own retrain tone, so the line
 * signals already exist in the V.34 control channel; wiring this controller
 * to them is the remaining step and is not done here.
 *
 * Bit order of the 4-bit table entries: fields are LSB:MSB, so an entry is a
 * binary number with bit 12 (resp. 16) its least significant bit.  That is
 * the reading under which Table 33 is monotonic (0001 = 10 s ... 1100 =
 * 16 min) and every Table 32 signal code is odd.  The constants below are
 * the only place this is decided.
 */
#ifndef V92_MH_H
#define V92_MH_H

#include <stdbool.h>
#include <stdint.h>

#define V92_MH_BITS 40
#define V92_MH_BIT_RATE 600           /* 8.2.3.1/V.90 */
#define V92_MH_SEQ_MS (V92_MH_BITS * 1000 / V92_MH_BIT_RATE) /* 66 ms (66.7) */

typedef enum {
    V92_MH_NONE  = 0,
    V92_MH_REQ   = 0x3,   /* 0011 */
    V92_MH_ACK   = 0x5,   /* 0101 */
    V92_MH_NACK  = 0x7,   /* 0111 */
    V92_MH_CLRD  = 0x9,   /* 1001 */
    V92_MH_CDA   = 0xB,   /* 1011 */
    V92_MH_FRR   = 0xD    /* 1101 */
} v92_mh_signal_t;

/* MHclrd information bits (Table 32). */
#define V92_MH_CLRD_INCOMING 0x5      /* 0101 */
#define V92_MH_CLRD_OUTGOING 0x6      /* 0110 */
#define V92_MH_CLRD_OTHER    0xA      /* 1010 */
/* MHnack information bits (Amd.1 item 7). */
#define V92_MH_NACK_NEVER    0x5      /* 0101: do not request again for an outgoing call */
#define V92_MH_NACK_LATER    0x7      /* 0111: may request again later */
/* Table 33 "no limit". */
#define V92_MH_T1_NO_LIMIT   0xD

typedef struct {
    v92_mh_signal_t signal;
    uint8_t info;                     /* bits 16:19 */
} v92_mh_frame_t;

typedef struct {
    bool fill_ok, sync_ok, crc_ok;
    bool signal_known;                /* Table 32 note 1: unknown -> ignore */
    bool info_defined;                /* note 2: reserved info is not interpreted */
    uint16_t crc_field, crc_expected;
    v92_mh_frame_t frame;
} v92_mh_decode_t;

bool v92_mh_signal_valid(v92_mh_signal_t s);
const char *v92_mh_signal_name(v92_mh_signal_t s);
bool v92_mh_is_initiating(v92_mh_signal_t s);   /* REQ, CLRD, FRR (and NACK, 9.10.1.1) */
/* Table 34: is `resp` an allowed response to `init`?  MHfrr's response is
 * ANSam, which is not an MH sequence, so nothing is. */
bool v92_mh_is_response_to(v92_mh_signal_t init, v92_mh_signal_t resp);
/* Table 33: seconds, 0 for no limit, -1 for reserved. */
int v92_mh_t1_seconds(uint8_t code);

/* Builds the 40 bits.  For REQ/CDA/FRR the information bits repeat the
 * signal bits and `info` is ignored.  Returns false for an undefined signal. */
bool v92_mh_encode(const v92_mh_frame_t *f, uint8_t bits[V92_MH_BITS]);
/* Accept = fill_ok && sync_ok && crc_ok && signal_known. */
bool v92_mh_decode(const uint8_t bits[V92_MH_BITS], v92_mh_decode_t *out);

/* ---- receive framer ---- */
typedef struct {
    uint8_t hist[V92_MH_BITS];
    unsigned filled;
    unsigned pos;                     /* ring write position */
    unsigned frames, crc_rejects, unknown;
} v92_mh_rx_t;

void v92_mh_rx_init(v92_mh_rx_t *r);
/* Returns true and fills *out when the last 40 bits form an accepted MH. */
bool v92_mh_rx_put_bit(v92_mh_rx_t *r, int bit, v92_mh_frame_t *out);

/* ---- 9.10 transaction controller ---- */
typedef enum {
    V92_MH_TX_DATA,                   /* nothing of ours; data mode continues */
    V92_MH_TX_SILENCE,
    V92_MH_TX_RT,
    V92_MH_TX_MH,                     /* v92_mh_ctrl_tx_bit() supplies the bits */
    V92_MH_TX_ANSAM
} v92_mh_tx_t;

typedef enum {
    V92_MH_ACT_NONE,
    V92_MH_ACT_SUSPEND_LINK,          /* Amd.2 9.10.3: leaving data mode, keep V.42 state */
    V92_MH_ACT_ON_HOLD,               /* entered the on-hold state (answer side) */
    V92_MH_ACT_PHASE1_ANSWER,         /* proceed with Phase 1 as answer modem (9.10.2.1/3) */
    V92_MH_ACT_PHASE1_CALL,           /* initiator of MHfrr: ANSam held 1 s (9.10.2.3) */
    V92_MH_ACT_RETRAIN,               /* responder saw a Tone B reversal (9.10.1.1) or initiator timed out */
    V92_MH_ACT_DISCONNECT
} v92_mh_action_t;

typedef enum {
    V92_MH_ST_IDLE,                   /* data mode */
    V92_MH_ST_INIT_SILENCE,           /* 70 +/- 5 ms (Cor.1 9.10.1) */
    V92_MH_ST_INIT_RT,
    V92_MH_ST_INIT_SEND,              /* sending an initiating sequence */
    V92_MH_ST_INIT_AWAIT_ANSAM,       /* MHfrr sent, waiting for 1 s of ANSam */
    V92_MH_ST_INIT_HOLD,              /* MHack received: requester holds */
    V92_MH_ST_RESP_RT,                /* responder: answering RT, conditioned for MH or reversal */
    V92_MH_ST_RESP_SEND,              /* sending a response sequence */
    V92_MH_ST_RESP_ANSAM_GAP,         /* <= 80 ms silence before ANSam */
    V92_MH_ST_ON_HOLD,                /* sending ANSam for T1 */
    V92_MH_ST_DONE
} v92_mh_state_t;

/* What the line detectors currently say, sampled at every tick. */
typedef struct {
    bool rt;                          /* far end's Tone RT (8.9.1: the OTHER tone) */
    bool silence;
    bool ansam;
    bool reversal;                    /* Tone B phase reversal seen this tick */
    bool phase1;                      /* QC or CM detected */
    bool qc_cleardown;                /* QC with UQTS 1111 */
    bool cm_null;                     /* CM: no PCM category, all modulation modes zero */
} v92_mh_detect_t;

typedef struct {
    /* configuration */
    int round_trip_ms;
    bool grant;                       /* answer to MHreq: MHack or MHnack */
    uint8_t t1_code;                  /* offered in MHack */
    uint8_t nack_reason;
    bool after_nack_reconnect;        /* initiator: MHfrr (true) or MHcda after MHnack */
    bool skip_rt_if_peer_rt;          /* Cor.1 9.10.1 option */

    v92_mh_state_t state;
    v92_mh_tx_t tx;
    v92_mh_action_t actions[8];       /* FIFO, drained by v92_mh_ctrl_take_action() */
    int n_actions;
    bool null_cm;                     /* 9.10.2.1: answer the null CM with a null JM, then disconnect after CJ */
    bool initiator;
    bool retrain_by_reversal;         /* the RETRAIN action came from a Tone B reversal */
    bool no_outgoing_requests;        /* MHnack 0101 received */

    v92_mh_frame_t tx_frame;
    bool deferred;                    /* a change waits for the open sequence to finish */
    v92_mh_tx_t deferred_tx;
    v92_mh_state_t deferred_state;
    v92_mh_action_t deferred_action;
    uint8_t tx_bits[V92_MH_BITS];
    int tx_bit;                       /* 0..39 within the current sequence */

    int ms;                           /* time in the current state */
    int total_ms;                     /* time since the transaction began */
    int peer_rt_ms, silence_ms, ansam_ms;
    int since_init_seen_ms;           /* responder: time since initiating sequence last decoded */
    int since_response_ms;            /* initiator: time since the response arrived */
    bool peer_rt_during_silence;
    bool first_mhack_sent;
    int t1_ms;                        /* -1 = no limit */
    int on_hold_ms;                   /* since the end of the first MHack */
    v92_mh_signal_t initiated;        /* what we initiated */
    v92_mh_signal_t peer_initiated;   /* what we are responding to */
    v92_mh_signal_t last_response;
    /* How the far end answered an MHreq we initiated, as V.250 6.8.4 Table
     * 34 numbers it for +PMHR: the granted T1 code (1-13, MHack), 0 denied
     * or no answer, 14 MHnack 0101 (never again this call).  -1 until then,
     * and for anything but MHreq. */
    int request_result;
    v92_mh_rx_t rx;
} v92_mh_ctrl_t;

void v92_mh_ctrl_init(v92_mh_ctrl_t *c, int round_trip_ms);
/* Start a transaction from data mode (9.10.1.1).  Returns false if one is
 * already running, or if `s` is MHreq after MHnack 0101 forbade it. */
bool v92_mh_ctrl_initiate(v92_mh_ctrl_t *c, v92_mh_signal_t s, uint8_t info);
/* One received MH bit (from the INFO-modulation receiver). */
void v92_mh_ctrl_rx_bit(v92_mh_ctrl_t *c, int bit);
/* Inject a decoded frame directly (tests, or a receiver that frames itself). */
void v92_mh_ctrl_rx_frame(v92_mh_ctrl_t *c, const v92_mh_frame_t *f);
/* Advance by `ms` with the given detector states. */
void v92_mh_ctrl_tick(v92_mh_ctrl_t *c, int ms, const v92_mh_detect_t *line);
/* Next bit to transmit while tx == V92_MH_TX_MH. */
int v92_mh_ctrl_tx_bit(v92_mh_ctrl_t *c);
v92_mh_action_t v92_mh_ctrl_take_action(v92_mh_ctrl_t *c);
const char *v92_mh_state_name(v92_mh_state_t s);

#endif
