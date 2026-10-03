/*
 * v92_rn.h — V.92 rate renegotiation (9.8), fast parameter exchange (9.9)
 * and cleardown (9.11 as replaced by Amd.1 item 6), both roles.
 *
 * All three end in the 9.6 Phase 4 exchange -- SUV sequences, a single CP,
 * acknowledgement bits, E, B1 -- so that exchange is the core here, and the
 * procedures differ only in how they get to it:
 *
 *   9.8  rate renegotiation  Rd|Ru 384T + bar 24T, TRN2, SUV; optionally a
 *                            silent period (SUV bit 32), ended by Rt (digital)
 *                            or a second TRN2u (analogue)        Figures 15-18
 *   9.9  fast param exchange Rf|RM 384T + bar 24T, scrambler re-initialised,
 *                            straight to SUV; analogue adds FB1u  Figure 19
 *   9.11 cleardown           either of the above with drn = 0 in this
 *                            modem's CP, no silence requested, and the
 *                            Amd.1 waits before going on-hook
 *
 * This is the procedure only: it works in units on the line (an R signal, a
 * TRN chunk, one SUV or CP frame, E, B1) and in received events (an R signal
 * detected, its transition to the bar, a decoded frame, E, B1).  Frame
 * contents come from the existing codecs (v92_cp_rx.h's SUVu/CPu/CPus and
 * SUVd/CPd).  Time is in symbols T at 8000 Hz.  The engine supplies data
 * frame boundaries: initiate on one, and start any R signal on one
 * (9.8/9.9's "shall begin on the boundary of a data frame").
 */
#ifndef V92_RN_H
#define V92_RN_H

#include <stdbool.h>
#include <stdint.h>

typedef enum { V92_RN_DIGITAL, V92_RN_ANALOGUE } v92_rn_role_t;
typedef enum { V92_RN_RENEG, V92_RN_FPE } v92_rn_proc_t;

/* R-family signals: the 384T signal; its 24T bar is the transition. */
typedef enum { V92_R_NONE, V92_R_RD, V92_R_RU, V92_R_RF, V92_R_RM, V92_R_RT } v92_r_t;

typedef enum {
    V92_RN_TX_DATA,
    V92_RN_TX_R,          /* 384T of `r` */
    V92_RN_TX_RBAR,       /* 24T of its bar */
    V92_RN_TX_TRN,        /* TRN2d / TRN2u chunk */
    V92_RN_TX_SILENCE,    /* Ucode 0, data frame alignment kept (9.8.1.1.3) */
    V92_RN_TX_SUV,
    V92_RN_TX_CP,
    V92_RN_TX_E,          /* Ed or E2u */
    V92_RN_TX_FB1,        /* FB1u (analogue, 9.9) */
    V92_RN_TX_B1          /* B1d / B1u, then data at the new parameters */
} v92_rn_tx_kind_t;

typedef struct {
    v92_rn_tx_kind_t kind;
    v92_r_t r;
    int symbols;
    bool ack;             /* SUV/CP bit 33 */
    bool silence;         /* SUV bit 32 */
    uint8_t drn;          /* CP only; 0 = cleardown */
} v92_rn_unit_t;

typedef enum {
    V92_RN_EV_R,          /* r signal detected */
    V92_RN_EV_RBAR,       /* r -> bar transition detected */
    V92_RN_EV_SUV,
    V92_RN_EV_CP,
    V92_RN_EV_E,
    V92_RN_EV_FB1,
    V92_RN_EV_B1
} v92_rn_ev_kind_t;

typedef struct {
    v92_rn_ev_kind_t kind;
    v92_r_t r;
    bool ack, silence;
    uint8_t drn;
} v92_rn_event_t;

typedef enum {
    V92_RN_ACT_NONE,
    V92_RN_ACT_CLAMP,             /* circuit 106 off / 104 clamped */
    V92_RN_ACT_REINIT_SCRAMBLER,  /* 9.9.x: scrambler, diff. encoder, shaper to zero */
    V92_RN_ACT_TX_DATA,           /* B1 sent: transmit data at the new parameters */
    V92_RN_ACT_RX_DATA,           /* B1 received: unclamp 104, demodulate */
    V92_RN_ACT_DISCONNECT,        /* 9.11 */
    V92_RN_ACT_RETRAIN            /* 9.6.x.2.1: no B1 within 20 s + 6 RTD */
} v92_rn_action_t;

typedef enum {
    V92_RN_IDLE,          /* data mode */
    V92_RN_WAIT_RBAR,     /* responder: R detected, waiting for its transition */
    V92_RN_SEND_R,        /* our R + bar */
    V92_RN_TRN,           /* 9.8 TRN2 */
    V92_RN_SUV1,          /* 9.8 first SUV exchange, silence undecided */
    V92_RN_SUV_ACK,       /* 9.8 silence path: SUV with bit 33 until acknowledged */
    V92_RN_SILENT,        /* digital: Ucode 0 after Ed (9.8.1.1.3-5) */
    V92_RN_TRN_AFTER_E,   /* analogue: TRN2u after E2u (9.8.2.1.5-6) */
    V92_RN_FPE_WAIT_R,    /* 9.9 initiator: SUV out, waiting for the peer's R pair */
    V92_RN_CORE,          /* the 9.6 exchange */
    V92_RN_SEND_E,        /* E (and FB1/B1) going out */
    V92_RN_WAIT_B1,       /* ours sent, waiting for the peer's B1 */
    V92_RN_CLEARDOWN,     /* 9.11 wait before going on-hook */
    V92_RN_DONE
} v92_rn_state_t;

typedef struct {
    /* configuration */
    v92_rn_role_t role;
    int rtd;                      /* round-trip delay estimate, T */
    int trn_symbols;              /* TRN2d <= 16008, TRN2u 2400..16008 */
    int silence_symbols;          /* digital's own silent period (9.8.1.1.5) */
    int trn_after_e_symbols;      /* analogue's second TRN2u, <= 8004 */
    uint8_t drn;                  /* rate this modem proposes in its CP */
    bool respond_silence;         /* as responder to 9.8, request a silent period (Figure 18) */

    /* state */
    v92_rn_state_t state;
    v92_rn_proc_t proc;
    bool initiator;
    bool want_silence;            /* our SUV bit 32 */
    bool peer_silence;            /* peer's SUV bit 32 at the decision */
    bool silence_mine;            /* silent period requested by this modem */
    bool r_bar_sent;
    v92_r_t r_tx;
    int trn_left;
    bool suv_sent;                /* at least one SUV out in this phase */
    bool peer_suv;                /* an SUV received this phase */
    bool peer_suv_ack;            /* ... with bit 33 */
    bool peer_r_done;             /* FPE initiator: peer's R pair seen */
    v92_r_t r_wait;               /* responder: the R whose bar we wait for */
    int r_stage;                  /* 0 = R next, 1 = bar next */
    int trn_sent;
    bool rt_seen;                 /* analogue: Rt received (9.8.2.1.6) */
    bool rt_go;                   /* digital: leave the silent period */
    int silent_sent;
    bool suv_first_in_core;       /* 9.8.1.1.4: Rt, bar, SUVd, then 9.6 */
    bool e_sent, fb1_sent, b1_sent;

    /* the 9.6 core */
    bool cp_due, cp_sent, cp_repeat;
    bool ack_tx, ack_rx;
    bool ack_frame_sent;          /* 9.6.x.1.4: "has sent a CP or SUV with the acknowledgement bit set" */
    long cp_end;                  /* time our last CP ended */
    bool cp_with_ack_sent;
    long cp_with_ack_end;
    bool peer_cleardown;          /* peer's CP had drn = 0 */
    bool cleardown;               /* ours does */
    uint8_t peer_drn;
    bool fb1_due, b1_due;
    bool tx_data, rx_data;
    long deadline;                /* B1 deadline, 9.6.x.2.1 */
    long disconnect_at;

    long now;
    v92_rn_action_t actions[8];
    int n_actions;
} v92_rn_t;

void v92_rn_init(v92_rn_t *rn, v92_rn_role_t role, int rtd_symbols);

/* Start 9.8 or 9.9 from data mode, on a data frame boundary.  `cleardown`
 * sends drn = 0 (9.11); it may not be combined with `silence`, which Amd.1
 * forbids the cleardown initiator to request.  Returns false if busy or if
 * the combination is not allowed. */
bool v92_rn_initiate(v92_rn_t *rn, v92_rn_proc_t proc, bool silence, bool cleardown);

/* Something received. */
void v92_rn_rx(v92_rn_t *rn, const v92_rn_event_t *ev);

/* The transmitter is ready for its next unit: fill *u.  A DATA unit means
 * data mode; anything else is sent whole before asking again, which is what
 * makes every "complete the current sequence" in 9.6/9.8 hold. */
void v92_rn_next(v92_rn_t *rn, v92_rn_unit_t *u);

/* Advance time (T) for the timers. */
void v92_rn_tick(v92_rn_t *rn, int symbols);

v92_rn_action_t v92_rn_take_action(v92_rn_t *rn);
const char *v92_rn_state_name(v92_rn_state_t s);

#endif
