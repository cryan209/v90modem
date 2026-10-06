/*
 * clear_channel.h — 64 kbit/s clear channel, V.120 and V.110 over the DS0
 *
 * The SIP/G.711 bearer here is passed through byte-exact (no transcoding,
 * PJMEDIA_HAS_PASSTHROUGH_CODECS), so each RTP payload octet IS a DS0 octet
 * end to end -- the 64 kbit/s unrestricted digital bearer RFC 4040 calls
 * CLEARMODE and ISDN calls a B channel.  No modulation is needed: the octets
 * carry a synchronous bit stream directly.
 *
 *   CC_CLEAR  the bit stream belongs to the caller's data stack (V.14 async
 *             characters, or V.42 LAPM), exactly as a datapump's would.
 *   CC_V120   ITU-T V.120 rate adaption: async DTE characters carried in
 *             HDLC frames (flag, 2-octet address with the LLI, control,
 *             V.120 header octet, data, CRC-16 FCS) on the bit stream.
 *             Unacknowledged mode only (UI frames); the multiple-frame
 *             acknowledged mode (SABME/I-frames) is not implemented, and a
 *             received I-frame is counted and dropped.
 *
 * Bit order on the DS0: the first bit of the stream goes in the octet's most
 * significant bit (bit 8 in ITU numbering), as a B channel is transmitted.
 * Restricted 56 kbit/s (robbed-bit trunks) carries 7 bits per octet in bits
 * 8..2 and sends bit 1 (the LSB, which robbed-bit signalling overwrites) as
 * 1; the receiver ignores it.
 *
 * CLEAR and V120 have no handshake: as on ISDN, the bearer is agreed outside
 * the channel (here, both ends configured alike with AT+MS), and the data
 * starts when the call connects.
 *
 *   CC_V110   ITU-T V.110 (02/2000) rate adaption for an asynchronous DTE:
 *             RA0 (5.3.3, start/stop characters onto a 2^n x 600 bit/s
 *             stream), RA1 (5.1.2, the 80-bit frame of Table 2 with the bit
 *             assignments of Tables 6a/6b/6c/6e) and RA2 (5.1.4 -> I.460: an
 *             8/16/32 kbit/s intermediate rate in the first 1/2/4 bits of
 *             each octet, the rest set to 1).  Clause 7.1's sequence runs on
 *             the S/X status bits: frames with S = X = OFF until the far
 *             end's framing is found (two alignment patterns, 5.1.3.1),
 *             then S = X = ON; the far end's S = X = ON makes the call
 *             "connected" (107/109 ON) and, N = 24 bits later (6.3), data
 *             flows (106 ON).  The far end's X OFF holds our data back at a
 *             character boundary (7.1.5 c, 5.4.2); our loss of framing (three
 *             bad frames, 5.1.3.2) turns our X OFF and is a disconnect if
 *             not recovered in 3 s (7.1.5 e); the far end's S OFF with data
 *             bits 0 is its disconnect request (7.1.4.2); T1 = 10 s bounds
 *             the start (7.1.2.2).  8 data bits, no parity, 1 stop element.
 *
 * V.120 (10/96) and V.110 (02/2000) are in "ITU Docs/"; see
 * docs/clear_channel_v120.md and docs/v120_conformance_audit.md for what
 * is implemented clause by clause.
 */
#ifndef CLEAR_CHANNEL_H
#define CLEAR_CHANNEL_H

#include <stdbool.h>
#include <stdint.h>

#include <spandsp.h>

typedef enum {
    CC_CLEAR = 0,
    CC_V120  = 1,
    CC_V110  = 2
} cc_mode_t;

/* CC_CLEAR: next line bit, or <0 for "nothing to send" (sent as mark, 1). */
typedef int  (*cc_get_bit_fn)(void *ctx);
typedef void (*cc_put_bit_fn)(void *ctx, int bit);
/* CC_V120: next DTE byte to send, or -1 for none; one received DTE byte. */
typedef int  (*cc_pull_byte_fn)(void *ctx);
typedef void (*cc_push_byte_fn)(void *ctx, uint8_t byte);
/* Free space (and total size) of the DTE-side receive buffer, for V.110 5.4.2. */
typedef void (*cc_break_fn)(void *ctx, bool on);
typedef void (*cc_room_fn)(void *ctx, int *free_bytes, int *size);

#define CC_V120_DEFAULT_LLI   256
/* User octets per V.120 frame.  3.2.2: N2120 = N201 (Q.922 5.9.3, agreed
 * per call) minus the header; 256 + H fits Q.922's default N201 of 260. */
#define CC_V120_MAX_DATA      256

/* V.120 header octet (terminal adaption header) bits. */
#define CC_V120_H_E           0x80   /* no control-state octet follows */
#define CC_V120_H_BR          0x40   /* break */
#define CC_V120_H_B           0x02   /* begin (first segment) */
#define CC_V120_H_F           0x01   /* final (last segment) */
#define CC_V120_H_RES         0x30   /* bits 5, 6: reserved, sent 0 */
/* Control-state octet (3.1.2, Figure 5). */
#define CC_V120_CS_E          0x80   /* always 1: CS is the last octet */
#define CC_V120_CS_DR         0x40
#define CC_V120_CS_SR         0x20
#define CC_V120_CS_RR         0x10   /* 0 = flow control asserted (3.2.4.1) */

/* V.110 */
#define CC_V110_N_BITS        24     /* 6.3: 106 ON N bits after 109 ON */
#define CC_V110_FRAME_BITS    80

typedef enum {
    CC_V110_SEARCH = 0,      /* sending S = X = OFF, looking for framing */
    CC_V110_SYNCED,          /* framing found, S = X = ON sent, waiting for
                                the far end's S = X = ON */
    CC_V110_CONNECTED,       /* 107/109 ON: data transfer state (7.1.3) */
    CC_V110_DISCONNECTING,   /* we asked: S OFF, D = 0 (7.1.4.1) */
    CC_V110_DOWN             /* finished: see cc->v110_cause */
} cc_v110_state_t;

typedef enum {
    CC_V110_CAUSE_NONE = 0,
    CC_V110_CAUSE_T1,        /* 7.1.2.4: no S = X = ON within T1 */
    CC_V110_CAUSE_SYNC_LOST, /* 7.1.5 e): framing not recovered in 3 s */
    CC_V110_CAUSE_REMOTE,    /* 7.1.4.2: far end's disconnect request */
    CC_V110_CAUSE_LOCAL,     /* 7.1.4.3: our request acknowledged */
    CC_V110_CAUSE_T2         /* 7.1.4.1: our request unanswered in T2 */
} cc_v110_cause_t;

/* V.120 data link: UI frames only, or Q.922 acknowledged operation (4.2). */
typedef enum {
    CC_LF_UI = 0,            /* UI frames (also the fallback if SABME fails) */
    CC_LF_DOWN,              /* acknowledged mode wanted, SABME not sent yet */
    CC_LF_SETUP,             /* SABME sent, awaiting UA */
    CC_LF_UP                 /* multiple-frame mode established */
} cc_lf_state_t;
#define CC_V120_TM20_OCTETS 20000u  /* 2.5 s: Q.922 App. III XID value */
#define CC_V120_NM20 3
#define CC_LF_K 15           /* window k; a power of two minus one fits mod 128 */

typedef struct {
    cc_mode_t mode;
    bool r56;                /* restricted 56 kbit/s: 7 bits per octet */
    bool caller;             /* informational: C/R does not depend on it */
    int lli;

    cc_get_bit_fn get_bit;
    cc_put_bit_fn put_bit;
    cc_pull_byte_fn pull;
    cc_push_byte_fn push;
    void *ctx;
    cc_break_fn brk_cb;          /* a break arrived (on) / ended (off) */
    bool brk_pending, brk_active, rx_in_break;
    int brk_ms;
    uint64_t brk_end_at;
    int v110_brk_bits;
    cc_room_fn room;             /* optional: V.110 flow control, 5.4.2 */
    bool v110_rx_hold;           /* we are sending X OFF: our buffer is full */

    hdlc_tx_state_t *htx;
    hdlc_rx_state_t *hrx;
    bool tx_frame_queued;
    bool v120_peer_rr;           /* RR(R), 3.2.3.1: 1 until a CS says so */

    /* V.120 acknowledged mode (Q.922 mod 128) */
    bool v120_ack;
    cc_lf_state_t lf_state;
    int lf_vs, lf_va, lf_vnew, lf_vr, lf_retries;
    bool lf_peer_busy, lf_rej_sent, lf_timer_rec, lf_enquire;
    bool lf_pend_ack, lf_pend_ua, lf_pend_dm, lf_pend_f, lf_pend_rr_f, lf_pend_rej, lf_pend_rej_f;
    bool lf_t200_on;
    uint64_t lf_t200_at;
    uint8_t lf_win[16][1 + CC_V120_MAX_DATA];
    int lf_win_len[16];
    /* Annex C: V.42bis */
    bool cz_enable;
    int cz_state, cz_dir, cz_p1, cz_p2, cz_req_dir, cz_xid_retries;   /* state: 0 idle, 1 XID out, 2 settled */
    v42bis_state_t *cz;
    uint8_t cz_out[2048];
    int cz_out_len;
    uint64_t cz_xid_at, cz_tx_in, cz_tx_out, cz_gave_up;
    uint8_t lf_xid_resp[64];
    int lf_xid_resp_len;
    bool vf_enable, lf_pend_xid;
    int vf_state, vf_retries;          /* 0 not started, 1 XID out, 2 verified */
    uint64_t vf_at, vf_ok, vf_gave_up;
    uint64_t lf_fallbacks, lf_resets, lf_rewinds, lf_discarded;

    /* V.110 */
    bool v110_sync;              /* synchronous user data (5.1), octets from the DTE */
    const int8_t *v110_map;      /* Table 6d/6f slot map, or NULL (repetition) */
    uint64_t v110_ir_pos;        /* intermediate-rate bits received */
    bool sy_tx_started;
    uint16_t sy_tx_shift;
    int sy_tx_bits;
    uint8_t sy_tx_phase;
    bool sy_rx_locked, sy_rx_skip;
    uint32_t sy_rx_win;
    uint8_t sy_rx_acc, sy_rx_phase;
    uint64_t sy_rx_last_end, sy_rx_gaps;
    int v110_user_rate;          /* asynchronous DTE rate, Table 8 */
    int v110_ra0_rate;           /* synchronous stream, 2^n x 600 */
    int v110_ir_bits;            /* intermediate rate / 8000: bits per octet */
    int v110_rep;                /* D-bit repetition (Tables 6a/6b/6c) */
    uint8_t v110_e123;           /* E1 E2 E3, Table 5, in bits 2..0 */
    cc_v110_state_t v110_state;
    cc_v110_cause_t v110_cause;
    /* transmit */
    uint8_t v110_txf[CC_V110_FRAME_BITS];
    int v110_txf_pos;
    uint32_t v110_tx_frame_no;
    bool v110_tx_s_on, v110_tx_x_on;
    bool v110_tx_d_zero;         /* disconnect: data bits 0 */
    int v110_n_count;            /* user bits since 109 ON / resync */
    uint16_t v110_tx_shift;      /* RA0: character bits, LSB first */
    int v110_tx_bits;
    int v110_tx_marks;           /* stop elements still owed */
    uint64_t v110_tx_pace;       /* RA0 stop-element padding accumulator */
    int v110_os_acc;             /* below 600 bit/s: sampling accumulator */
    int v110_os_bit;
    /* receive */
    uint8_t v110_hist[CC_V110_FRAME_BITS];
    int v110_hist_n;
    bool v110_synced, v110_verify;
    int v110_bad_run;            /* consecutive frames with framing errors */
    uint64_t v110_lost_at;       /* rx octet count framing was lost at */
    uint64_t v110_start_at;
    bool v110_sync_seen;         /* framing found at least once */
    bool v110_rem_s_on, v110_rem_x_on;
    int v110_rem_on_run, v110_rem_disc_run;
    int v110_disc_frames;        /* frames sent in DISCONNECTING */
    uint64_t v110_disc_at;       /* rx octet count our request began at */
    int v110_rx_bits;            /* RA0: -1 hunting, else bits taken */
    uint16_t v110_rx_shift;
    int v110_zero_run;
    int v110_held_nuls;          /* NULs that may yet be a break */
    int v110_rx_n, v110_rx_next; /* below 600: samples since start edge */
    int v110_rx_prev;
    uint8_t v110_rx_e123;

    /* Statistics */
    uint64_t tx_octets, rx_octets;
    uint64_t tx_frames, rx_frames;
    uint64_t tx_data_bytes, rx_data_bytes;
    uint64_t rx_bad_frames;      /* FCS errors, aborts, runts, bad headers */
    uint64_t rx_unsupported;     /* I-frames and other non-UI frames */
    uint64_t rx_other_lli;       /* V.120 frames for a link we do not serve */
    uint64_t rx_breaks;
    uint64_t v110_frame_errors;  /* V.110 frames with a framing bit wrong */
    uint64_t v110_sync_losses;
    uint64_t v110_flow_holds;    /* times we turned X OFF for our own buffer */
    uint64_t v110_rate_mismatch; /* frames whose E1-E3 name another rate */
} clear_channel_t;

int  cc_init_clear(clear_channel_t *cc, bool r56,
                   cc_get_bit_fn get_bit, cc_put_bit_fn put_bit, void *ctx);
int  cc_init_v120(clear_channel_t *cc, bool r56, bool caller,
                  cc_pull_byte_fn pull, cc_push_byte_fn push, void *ctx);
/* user_rate: one of cc_v110_rates[].  -1 for a rate V.110 does not carry,
 * or one this asynchronous 8N1 profile does not (50 bit/s is 5 data units). */
int  cc_init_v110(clear_channel_t *cc, int user_rate,
                  cc_pull_byte_fn pull, cc_push_byte_fn push, void *ctx);
void cc_release(clear_channel_t *cc);
/* V.110 synchronous user data (5.1.2, Tables 1, 5, 6a-6f): the DTE's octets
 * (LSB first) are the D-bit stream at the user rate, which must be one of
 * 600 1200 2400 4800 7200 9600 12000 14400 19200 24000 28800 38400.  Call
 * right after cc_init_v110(); -1 for any other rate.  Octet alignment is
 * from the first data frame (see clear_channel.c: the stream opens with
 * 0x00 0xFF, which the receiver consumes), idle is 0xFF, no break. */
int  cc_v110_set_sync(clear_channel_t *cc);
/* Break (V.120 3.1.1.2/7.2.2; V.110 5.3.5).  cc_send_break() queues one of
 * `ms` milliseconds after the characters already pulled; a received break
 * is reported through the callback in order, after the characters that
 * preceded it, and again (on = false) when it ends.  No-op for CC_CLEAR. */
void cc_set_break_cb(clear_channel_t *cc, cc_break_fn fn);
void cc_send_break(clear_channel_t *cc, int ms);
/* V.120 4.2: carry the data in Q.922 I-frames (SABME/UA, mod 128, k = 15,
 * T200 1.5 s, N200 3) instead of UI frames.  Call right after cc_init_v120().
 * A peer that refuses (DM) or never answers SABME gets UI frames. */
void cc_v120_set_ack(clear_channel_t *cc, bool ack);
/* V.120 4.2.2: in UI-only mode send an XID command first and hold the data
 * until the XID response (TM20 2.5 s, NM20 3; then data starts anyway, as
 * 4.2.2 allows).  An XID command is answered whether or not this is on. */
void cc_v120_set_verify(clear_channel_t *cc, bool on);
/* V.120 Annex C: negotiate V.42bis by XID once the acknowledged link is up
 * (the caller proposes both directions, P1 1024, P2 32; the responder
 * agrees to no more than asked).  Needs cc_v120_set_ack(); a peer that does
 * not answer in NM20 tries gets uncompressed data (C.2.3 a). */
void cc_v120_set_compression(clear_channel_t *cc, bool on);
/* V.110 5.4.2: turn X OFF towards the far end when the DTE-side receive
 * buffer is under a quarter free (characters already in flight still fit),
 * back ON once it is over a half free.  Only in the data transfer state. */
void cc_v110_set_rx_room(clear_channel_t *cc, cc_room_fn room);

/* Table 8 asynchronous user rates this profile carries, ascending. */
extern const int cc_v110_rates[];
extern const int cc_v110_n_rates;
/* The RA0 stream rate (5.3.3) for a user rate, or 0. */
int  cc_v110_ra0_rate(int user_rate);
/* 7.1.4.1: ask the far end to disconnect (S OFF, X ON, D = 0): 106 OFF,
 * no more data either way.  Ends in DOWN when the far end's S OFF or loss
 * of framing acknowledges it (7.1.4.3), or after T2 = 5 s without. */
void cc_v110_disconnect(clear_channel_t *cc);
/* DOWN, and the disconnect request has been on the line long enough
 * (7.1.5 e): three frames) for the bearer to be released. */
bool cc_v110_finished(const clear_channel_t *cc);
const char *cc_v110_cause_name(cc_v110_cause_t c);

/* Fill n DS0 octets to transmit / consume n received DS0 octets. */
void cc_tx(clear_channel_t *cc, uint8_t *octets, int n);
void cc_rx(clear_channel_t *cc, const uint8_t *octets, int n);

/* 64000 or 56000 (V.110: the user rate). */
int  cc_line_rate(const clear_channel_t *cc);

/* The address + control + header V.120 puts in front of user data (4
 * octets), for tests and for anyone checking a capture.  UI is a command,
 * so C/R is 0 from either end (6.2.2.3, Table 4). */
void cc_v120_frame_header(const clear_channel_t *cc, uint8_t hdr[4]);

#endif
