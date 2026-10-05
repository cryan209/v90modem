/*
 * v76.h -- ITU-T V.76 (08/96) + Cor.1 (01/2005): generic multiplexer using
 * V.42 LAPM-based procedures.  This is the multiplex function (MF) under
 * V.70 DSVD (V.75's control entity sits on top of it).
 *
 * What is here, by clause:
 *   5.1   HDLC framing: 0111 1110 flags, 0-bit insertion, 8/16/32-bit FCS
 *         (5.1.6.1-3), 1- or 2-octet address field (6.1), abort (5.4)
 *   6     I / S / U frame formats, C/R from the initiator/responder role
 *   7     DLC establishment (SABME/UA/DM), release (DISC), collisions,
 *         XID (L-SETPARM), TEST (L-TEST)
 *   8.1   error recovery mode: k window, REJ and single-SREJ recovery, RNR
 *         busy, T401/N400 timer recovery, T403 inactivity, N(R) / frame
 *         rejection with FRMR
 *   8.2   unacknowledged non-error-recovery mode (UI, and UIH of App. II)
 *   Annex A  suspend/resume: real-time frames interrupt a non-real-time frame
 *            with a suspend flag (0 1^7 0) and hand back with a resume flag
 *            (0 1^8 0); abort becomes nine or more 1s
 *
 * What is NOT here: m-SREJ (the span-list encoding of Figure 7 is not
 * recoverable from the text extraction), and everything above the MF -- see
 * v75.h (control entity) and v70.h (the DSVD terminal).
 *
 * The line interface is bit oriented, bit 1 of each octet first (5.2.1.1),
 * exactly like a datapump's synchronous stream.  Idle is contiguous flags
 * (5.5).  Time is kept by the MF itself from the transmit bit count when a
 * line rate is configured, or from v76_advance_ms() otherwise.
 */

#ifndef V76_H
#define V76_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define V76_MAX_DLC        16
#define V76_MAX_N401       4095
#define V76_MAX_K          127
#define V76_FRAME_MAX      (V76_MAX_N401 + 2 + 2 + 4)

/* FCS lengths, in octets (5.1.6). */
#define V76_FCS_8          1
#define V76_FCS_16         2
#define V76_FCS_32         4
/* Bit mask of supported FCS lengths for the receiver (5.1.6: "all received
 * frames shall be checked against all the supported lengths"). */
#define V76_FCS_MASK_8     0x1
#define V76_FCS_MASK_16    0x2
#define V76_FCS_MASK_32    0x4

typedef enum { V76_ERM = 0, V76_UNERM = 1 } v76_mode_t;
typedef enum { V76_REC_REJ = 0, V76_REC_SREJ = 1 } v76_recovery_t;

typedef enum {
    V76_DLC_DISCONNECTED = 0,
    V76_DLC_AWAIT_SU,           /* SABME received, waiting for the SU's answer */
    V76_DLC_AWAIT_EST,          /* our SABME sent */
    V76_DLC_CONNECTED,
    V76_DLC_AWAIT_REL           /* our DISC sent */
} v76_dlc_state_t;

typedef enum {
    V76_REL_DISC_RECEIVED,      /* peer's DISC */
    V76_REL_DM_RECEIVED,        /* peer refused / is disconnected */
    V76_REL_LOCAL_COMPLETE,     /* our DISC was acknowledged */
    V76_REL_N400,               /* retransmissions exhausted (7.1.2.2, 8.4.6) */
    V76_REL_FRAME_REJECT,       /* frame rejection condition (8.4.2) */
    V76_REL_FRMR_RECEIVED,      /* peer sent FRMR (8.4.3) */
    V76_REL_N_R_ERROR,          /* invalid N(R) (8.1.10) */
    V76_REL_BACKOFF             /* responder gave way in a DLCI collision (6.1.1) */
} v76_release_reason_t;

/* Per-DLC operating parameters.  The opener chooses them (V.76 9.8: no
 * negotiation of the mode); the acceptor is told them by the SU, which learns
 * them from the SABME user data (V.75's H.245 OpenLogicalChannel). */
typedef struct {
    v76_mode_t mode;
    int k;                      /* ERM window for frames WE send, 1..127 (9.4) */
    int k_rx;                   /* window the peer uses toward us; 0 = same as k */
    int n401_tx;                /* max info octets we send (9.3) */
    int n401_rx;                /* max info octets we accept */
    v76_recovery_t recovery;
    bool uih;                   /* App. II: UIH in place of UI */
    int uih_protect;            /* octets after the flag covered by the FCS */
    bool realtime;              /* Annex A real-time DLC */
    int n401_rt;                /* Annex A N401RT (defaults to n401_tx) */
    int fcs_len;                /* opener's choice; learned from SABME otherwise */
    int addr_octets;            /* opener's choice; learned from SABME otherwise */
} v76_dlc_params_t;

void v76_dlc_params_default(v76_dlc_params_t *p);

typedef struct {
    bool initiator;             /* role: DLCI allocation and C/R (6.1) */
    int line_bit_rate;          /* 0: time only via v76_advance_ms() */
    int t401_ms;                /* 9.1; default 1000 */
    int n400;                   /* 9.2; default 10 */
    int t403_ms;                /* 9.6; 0 = off */
    int addr_octets;            /* default for DLCs we open: 1 or 2 */
    unsigned fcs_support;       /* receiver's accepted FCS lengths */
    bool suspend_resume;        /* Annex A */
    bool sr_with_address;       /* RT frames carry an address field */
} v76_config_t;

void v76_config_default(v76_config_t *c, bool initiator);

typedef struct v76_s v76_t;

/* Service-user callbacks: the L-xxx indication and confirm primitives of
 * Table 1.  Every callback may be NULL.  They are invoked from within
 * v76_rx_put_bit() / v76_advance_ms(); an SU may call the request functions
 * from inside them. */
typedef struct {
    void *ctx;
    void (*establish_ind)(void *ctx, int dlci, const uint8_t *ud, int len);
    void (*establish_conf)(void *ctx, int dlci, const uint8_t *ud, int len);
    void (*release_ind)(void *ctx, int dlci, const uint8_t *ud, int len,
                        v76_release_reason_t why);
    void (*data_ind)(void *ctx, int dlci, const uint8_t *data, int len);
    /* UI/UIH.  is_response and pf are the received C/R-derived role and P/F,
     * which V.76 Annex B's break exchange (V.75 clause 10) needs. */
    void (*unitdata_ind)(void *ctx, int dlci, const uint8_t *data, int len,
                         bool is_response, bool pf, bool header_only_checked);
    void (*setparm_ind)(void *ctx, int dlci, const uint8_t *ud, int len);
    void (*setparm_conf)(void *ctx, int dlci, const uint8_t *ud, int len);
    void (*setparm_fail)(void *ctx, int dlci);
    void (*test_ind)(void *ctx, int dlci, const uint8_t *data, int len);
    /* An invalid frame with an FCS error was dropped (5.3 NOTE: useful to a
     * voice SU).  dlci is -1 when it could not be read. */
    void (*fcs_error)(void *ctx, int dlci);
    void (*violation)(void *ctx, int dlci, const char *what);
} v76_su_t;

typedef struct {
    uint64_t tx_frames, rx_frames;
    uint64_t tx_i_frames, rx_i_frames, tx_retransmitted_i;
    uint64_t tx_ui_frames, rx_ui_frames;
    uint64_t tx_bits, rx_bits;
    uint64_t rx_invalid, rx_fcs_errors, rx_aborts, rx_unknown_dlci;
    uint64_t rej_sent, rej_received, srej_sent, srej_received;
    uint64_t t401_expiries, sr_suspends, sr_resumes, sr_violations;
    uint64_t rt_frames_in_nrt;
} v76_stats_t;

v76_t *v76_create(const v76_config_t *cfg, const v76_su_t *su);
void v76_destroy(v76_t *mf);
void v76_set_su(v76_t *mf, const v76_su_t *su);
const v76_stats_t *v76_stats(const v76_t *mf);

/* ---- Service requests (SU -> MF) ---------------------------------------- */

/* L-ESTABLISH request.  Picks the DLCI (6.1.1) and sends SABME with the user
 * data in its information field.  Returns the DLCI, or -1. */
int v76_establish_req(v76_t *mf, const v76_dlc_params_t *p,
                      const uint8_t *ud, int len);
/* L-ESTABLISH response: accept an indicated DLC (UA, with user data). */
int v76_establish_rsp(v76_t *mf, int dlci, const v76_dlc_params_t *p,
                      const uint8_t *ud, int len);
/* L-RELEASE request in answer to an establish indication: refuse (DM). */
int v76_establish_reject(v76_t *mf, int dlci, const uint8_t *ud, int len);
/* L-RELEASE request on an established DLC: DISC. */
int v76_release_req(v76_t *mf, int dlci, const uint8_t *ud, int len);
/* L-DATA request (ERM).  false if the DLC is not connected, the SDU exceeds
 * N401 or the queue is full. */
bool v76_data_req(v76_t *mf, int dlci, const uint8_t *data, int len);
/* L-UNITDATA request: a UI (or UIH) command with P=0. */
bool v76_unitdata_req(v76_t *mf, int dlci, const uint8_t *data, int len);
/* The same, as a response with F=pf (V.76 Annex B BRKACK). */
bool v76_unitdata_rsp(v76_t *mf, int dlci, const uint8_t *data, int len, bool pf);
/* L-SETPARM request / response: XID command / response (7.6). */
int v76_setparm_req(v76_t *mf, int dlci, const uint8_t *ud, int len);
int v76_setparm_rsp(v76_t *mf, int dlci, const uint8_t *ud, int len);
/* L-TEST request (7.7). */
int v76_test_req(v76_t *mf, int dlci, const uint8_t *data, int len);

/* From inside release_ind for a peer's DISC: attach user data to the UA that
 * answers it (6.4.10 permits an information field; V.75 6.3.4 uses it for
 * CloseLogicalChannelAck). */
void v76_release_response_data(v76_t *mf, const uint8_t *ud, int len);

/* Own-receiver busy (8.1.7): while set, I frames are discarded and RNR sent. */
void v76_set_busy(v76_t *mf, int dlci, bool busy);

/* ---- Introspection ------------------------------------------------------ */
v76_dlc_state_t v76_dlc_state(const v76_t *mf, int dlci);
bool v76_dlc_params(const v76_t *mf, int dlci, v76_dlc_params_t *out);
/* SDUs / octets queued and not yet transmitted or acknowledged. */
int v76_data_backlog(const v76_t *mf, int dlci);
int v76_unacked_frames(const v76_t *mf, int dlci);
int v76_unitdata_backlog(const v76_t *mf, int dlci);
/* Receiver framing state (diagnostics): 0 normal, 1 suspend, 2 abort, +16 hunting. */
int v76_rx_sr_state(const v76_t *mf);
/* True if some frame is being sent or queued (the line is not idle). */
bool v76_tx_busy(const v76_t *mf);

/* ---- Line side ---------------------------------------------------------- */
int v76_tx_get_bit(v76_t *mf);                 /* 0/1, idle = flags */
void v76_rx_put_bit(v76_t *mf, int bit);
/* Packed octets, bit 1 (LSB) first. */
void v76_tx_fill_bytes(v76_t *mf, uint8_t *out, int len);
void v76_rx_push_bytes(v76_t *mf, const uint8_t *in, int len);
void v76_advance_ms(v76_t *mf, int ms);

/* ---- Pure helpers, exposed for tests ------------------------------------ */
/* FCS over data[0..len) for a 1/2/4-octet FCS, as it appears on the wire
 * (low-order byte first within the field of 5.2.1.2 Figure 5). */
uint32_t v76_fcs(int fcs_len, const uint8_t *data, int len);
/* Final CRC register after running over a frame INCLUDING its FCS: the
 * constant 5.1.6.1-3 give as the error-free remainder. */
uint32_t v76_fcs_residue(int fcs_len, const uint8_t *data_with_fcs, int len);
uint32_t v76_fcs_expected_residue(int fcs_len);
/* Build a complete frame body (address + control + info + FCS), no flags. */
int v76_build_frame(uint8_t *out, int addr_octets, int dlci, bool cr,
                    const uint8_t *ctl, int ctl_len,
                    const uint8_t *info, int info_len,
                    int fcs_len, int protect);

#endif
