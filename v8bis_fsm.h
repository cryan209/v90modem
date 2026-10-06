/*
 * v8bis_fsm.h -- V.8bis transaction state machine (clause 9, Figures 14 and 15).
 *
 * Event driven and audio-free: the caller reports what the line delivered (a
 * tone signal, a decoded message, a bad frame, time passing) and executes the
 * actions it gets back (send this signal, send these messages).  The tone and
 * V.21 modems are the layer below (stage 3); this layer is the Recommendation's
 * logic and nothing else.
 *
 * A station sits in the Initial V.8bis State.  It either initiates a
 * transaction (v8bis_fsm_initiate(): MR or CR tone, or ESi plus MS, CL or CLR)
 * and then follows Figure 14, or it hears the peer do so and follows Figure 15.
 * The two diagrams share state names, so one enum serves both and `role`
 * says which diagram applies.
 *
 * Table 7's thirteen transactions fall out of the transitions:
 *    1  MR  -> MS -> ACK/NAK                  8  MRe -> MRd -> CRd -> CL  -> MS -> ACK/NAK
 *    2  CR  -> CL -> MS -> ACK/NAK            9  MRe -> MRd -> CRd -> CLR -> CL-MS -> ACK/NAK
 *    3  CR  -> CLR -> CL-MS -> ACK/NAK       10  MRe -> CRd -> CL -> MS -> ACK/NAK
 *    4  MS  -> ACK/NAK                       11  MRe -> CRd -> CLR -> CL-MS -> ACK/NAK
 *    5  CL  -> MS -> ACK/NAK                 12  CRe -> CRd -> CL -> MS -> ACK/NAK
 *    6  CLR -> CL -> MS -> ACK/NAK           13  CRe -> CRd -> CLR -> CL-MS -> ACK/NAK
 *    7  MRe -> MRd -> MS -> ACK/NAK
 * (transactions 7-13 exist only on automatic answering: 9.3, the dotted
 * transitions).
 */
#ifndef V8BIS_FSM_H
#define V8BIS_FSM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "v8bis_ie.h"
#include "v8bis_tones.h"

#define V8BIS_STATE_TIMEOUT_MS 5000          /* 9.8 */
#define V8BIS_MAX_CL_SEGMENTS 8
#define V8BIS_ACTION_QUEUE 8

typedef enum {
    V8BIS_ROLE_NONE = 0,                      /* Initial V.8bis State, nobody has started */
    V8BIS_ROLE_INITIATOR,                     /* Figure 14 */
    V8BIS_ROLE_RESPONDER                      /* Figure 15 */
} v8bis_role_t;

typedef enum {
    V8BIS_S_INITIAL = 0,
    V8BIS_S_SENT_MR,                          /* initiator: sent MRe/d, expects MS, CR or MR */
    V8BIS_S_SENT_CR,                          /* sent CR(e/d); expects CL or CLR */
    V8BIS_S_SENT_MS,                          /* sent MS or CL-MS, expects ACK or NAK */
    V8BIS_S_SENT_CL,                          /* sent CL (or, as responder, MRd): expects MS */
    V8BIS_S_SENT_CLR,                         /* sent CLR, expects CL-MS */
    V8BIS_S_MS_MODE
} v8bis_state_t;

typedef enum { V8BIS_INIT_MR, V8BIS_INIT_CR, V8BIS_INIT_MS, V8BIS_INIT_CL, V8BIS_INIT_CLR } v8bis_init_t;

/* How a station answers a tone signal (Figure 15 from Initial).  The dotted
 * choices need automatic answering. */
typedef enum { V8BIS_MRR_MS, V8BIS_MRR_MRD, V8BIS_MRR_CRD } v8bis_mr_reply_t;
typedef enum { V8BIS_CRR_CL, V8BIS_CRR_CLR, V8BIS_CRR_CRD } v8bis_cr_reply_t;

typedef enum { V8BIS_ES_NONE = 0, V8BIS_ES_ESI, V8BIS_ES_ESR } v8bis_es_t;

typedef enum {
    V8BIS_STARTUP_V25 = 0,                    /* both V.8 codepoints zero (9.9.3) */
    V8BIS_STARTUP_V8,                         /* 9.9.1 */
    V8BIS_STARTUP_SHORT_V8,                   /* 9.9.2 */
    V8BIS_STARTUP_TELEPHONY                   /* MS selected analogue telephony: no modem */
} v8bis_startup_t;

typedef enum {
    V8BIS_ACCEPT_ACK = 1,                     /* positively acknowledge */
    V8BIS_ACCEPT_NAK2 = 2,                    /* temporarily unable (9.5) */
    V8BIS_ACCEPT_NAK3 = 3                     /* does not support or has disabled the mode */
} v8bis_accept_t;

typedef struct {
    bool auto_answer_call;                    /* call establishment on automatic answering */
    bool answering_station;                   /* we answered the call (initiator uses MRe/CRe) */
    v8bis_msg_t caps;                         /* what we put in CL */
    unsigned max_info_octets;                 /* CL segmentation limit, 0 = 64 (8.6) */
    bool want_more_info;                      /* answer MORE_INFO with ACK(2) */
    bool tx_ack1;                             /* ask for ACK(1) in the MS we send (9.7) */
    bool mode_available;                      /* false: NAK(2) every MS */
    bool echo_suppressor;                     /* 9.4: 1.5 s silence after ES */
    v8bis_mr_reply_t mr_reply;
    v8bis_cr_reply_t cr_reply;
    bool on_mrd_ask_caps;                     /* initiator that sent MR and hears MRd: CRd rather than MS */
    bool on_crd_send_clr;                     /* after CRd: CLR rather than CL */
    bool have_preset_ms;                      /* MS to send when no capabilities are known */
    v8bis_msg_t preset_ms;
    /* optional policy hooks; NULL selects the defaults described in v8bis_fsm.c */
    v8bis_accept_t (*accept)(void *user, const v8bis_msg_t *ms);
    bool (*select_ms)(void *user, const v8bis_msg_t *peer_caps_or_null, v8bis_msg_t *ms_out);
    void *user;
} v8bis_fsm_cfg_t;

typedef enum {
    V8BIS_ACT_SIGNAL,                         /* send a tone signal */
    V8BIS_ACT_MESSAGES,                       /* send 1 or 2 messages back to back (CL-MS) */
    V8BIS_ACT_MS_MODE,                        /* transaction complete: start the selected mode */
    V8BIS_ACT_INITIAL                         /* back in the Initial State; see reason */
} v8bis_act_type_t;

typedef enum {
    V8BIS_WHY_NONE = 0,
    V8BIS_WHY_NAK_RECEIVED,                   /* peer refused our MS (type in `nak`) */
    V8BIS_WHY_NAK_SENT,                       /* we refused the peer's MS */
    V8BIS_WHY_TIMEOUT,                        /* 9.8: five seconds out of the Initial State */
    V8BIS_WHY_INVALID_FRAME,                  /* 9.8: we sent NAK(1) */
    V8BIS_WHY_NO_COMMON_MODE                  /* we could not choose an MS from the capabilities */
} v8bis_why_t;

typedef struct {
    bool we_sent_ms;                          /* station A: the MS sender */
    v8bis_msg_t ms;
    v8bis_startup_t startup;
    bool answer_modem;                        /* 9.9: the MS receiver is the answer modem */
    bool ack_sent;                            /* we sent ACK(1) */
    bool ack_expected;                        /* we sent the MS and ACK(1) came (false: 9.7 suppression) */
    bool start_signal_next;                   /* MS receiver: send ANS/ANSam immediately (after any ACK(1)) */
} v8bis_mode_t;

typedef struct {
    v8bis_act_type_t type;
    v8bis_role_t role;                        /* our role when the action was queued: picks the V.21 channel */
    /* SIGNAL */
    v8bis_signal_t sig;
    bool responding_set;
    /* MESSAGES */
    v8bis_es_t es;                            /* signal that precedes the first message */
    bool es_gap;                              /* 1.5 s silence after the ES signal (9.4) */
    unsigned n_msg;
    v8bis_msg_t msg[2];
    /* MS_MODE */
    v8bis_mode_t mode;
    /* INITIAL */
    v8bis_why_t why;
    unsigned nak;                             /* NAK(n) received or sent */
} v8bis_action_t;

typedef struct {
    v8bis_fsm_cfg_t cfg;
    v8bis_role_t role;
    v8bis_state_t state;
    unsigned elapsed_ms;                      /* in this state */
    v8bis_msg_t peer_caps;                    /* merged from every CL received */
    bool peer_caps_known;
    bool peer_more_info;                      /* last CL had MORE_INFO set */
    bool init_clr;                            /* we initiated with CLR and wait for the CL */
    bool got_cl;                              /* CL received, waiting for the MS that completes CL-MS */
    bool sent_mr_signal;                      /* we sent MR in this transaction (responder: MRd) */
    v8bis_msg_t segs[V8BIS_MAX_CL_SEGMENTS];  /* our CL, split to fit */
    unsigned n_segs, seg_next;
    v8bis_msg_t our_ms;                       /* MS we sent, or are about to send with the last CL */
    bool our_ms_valid;
    bool our_ms_wants_ack;
    v8bis_msg_t peer_ms;
    v8bis_action_t q[V8BIS_ACTION_QUEUE];
    unsigned qh, qn;
    bool mode_done;
} v8bis_fsm_t;

void v8bis_fsm_cfg_default(v8bis_fsm_cfg_t *cfg);
void v8bis_fsm_init(v8bis_fsm_t *f, const v8bis_fsm_cfg_t *cfg);

/* Initiate from the Initial State (Figure 14).  False if not in it. */
bool v8bis_fsm_initiate(v8bis_fsm_t *f, v8bis_init_t how);
/* A V.8bis tone signal was detected. */
void v8bis_fsm_signal(v8bis_fsm_t *f, v8bis_signal_t sig, bool responding_set);
/* A message was received and decoded.  CL then MS back to back are two calls. */
void v8bis_fsm_message(v8bis_fsm_t *f, const v8bis_msg_t *m);
/* A frame failed 7.2.9: NAK(1) and back to the Initial State (9.8). */
void v8bis_fsm_invalid_frame(v8bis_fsm_t *f);
/* Give up the current transaction quietly and return to the Initial State
 * (no action is queued): for a station that is about to try again. */
void v8bis_fsm_abandon(v8bis_fsm_t *f);
/* ANS/ANSam heard after we sent an MS that did not ask for ACK(1) (9.7). */
void v8bis_fsm_startup_signal(v8bis_fsm_t *f);
/* Time passes: the 9.8 five second rule. */
void v8bis_fsm_tick(v8bis_fsm_t *f, unsigned ms);

bool v8bis_fsm_next_action(v8bis_fsm_t *f, v8bis_action_t *a);
const char *v8bis_state_name(v8bis_state_t s);
const char *v8bis_why_name(v8bis_why_t w);

/* Helpers, public because the policy hooks and the tests use them. */
/* Merge a received CL into accumulated capabilities (OR of every field). */
void v8bis_caps_merge(v8bis_msg_t *dst, const v8bis_msg_t *src);
/* Split capabilities into CL messages of at most `max_octets`, MORE_INFO on
 * all but the last.  Returns the segment count (0 on error). */
unsigned v8bis_caps_split(const v8bis_msg_t *caps, unsigned max_octets, v8bis_msg_t *segs,
                          unsigned max_segs);
/* The default mode choice from our capabilities and the peer's (NULL = unknown). */
bool v8bis_default_select_ms(const v8bis_msg_t *ours, const v8bis_msg_t *peer, bool tx_ack1,
                             v8bis_msg_t *ms);
/* The default acceptance test: does `ms` ask only for things `ours` offers? */
bool v8bis_ms_within_caps(const v8bis_msg_t *ours, const v8bis_msg_t *ms);
v8bis_startup_t v8bis_ms_startup(const v8bis_msg_t *ms);

#endif
