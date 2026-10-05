/*
 * v75.h -- ITU-T V.75 (08/96) + Cor.1 (01/2005): DSVD terminal control
 * procedures.  The control entity (CE) sits between V.70's supervisory and
 * control function (SCF) / audio and data processing and the V.76 multiplex
 * function (v76.h).
 *
 * Implemented, by V.75 clause:
 *   6.1-6.3  channel establishment / refusal / release, each carried as an
 *            H.245 message in the user data of the V.76 SABME / UA / DM /
 *            DISC (Tables 3 and 4), channel number <-> DLCI one-to-one
 *   6.4      capability exchange: in-band in an XID (L-SETPARM), or out-of-
 *            band on a DSVDControl channel as L-DATA (Tables 5 and 6)
 *   6.5      data transfer: UNERM -> L-UNITDATA with the optional audio
 *            header, ERM -> L-DATA
 *   9        the audio header octet (Table 7), with lost-frame detection from
 *            its 5-bit sequence number
 *   10       break: BRK / BRKACK over UI frames, V(SB)/V(RB) sequencing,
 *            T401/N400 retransmission (V.76 Annex B)
 *   11       segmentation / reassembly of data protocol frames over UNERM,
 *            with the H octet in front (position per Cor.1 item 6)
 *
 * The H.245 messages are a typed model (v75_msg_t, every parameter V.75 Tables
 * 3, 5 and 6 list) behind v75_h245_codec_t.  The default codec is real H.245
 * ASN.1 aligned PER (v75_h245.c over per.c).  Where H.245 (03/2022) differs
 * from V.75's 1996 Annex A the current H.245 wins: V75Parameters.
 * audioHeaderPresent is a BOOLEAN there (V.75 says NULL), and V76ModeParameters
 * is a choice of suspend/resume with or without address, carried in a
 * RequestMode mode element.  Not mapped: the t84 and nlpid data applications
 * (X.263 over network layer) -- they need T84Profile / unconstrained octets.
 *
 *   V.75 8.1 puts the user data "within an FI field encoded as 133 D".  Read
 *   with V.42 12.2.1.3 (the user data subfield "does not contain a GL" and
 *   runs to the FCS) that is one octet 0x85 followed by the message; that is
 *   what v75_wrap()/v75_unwrap() do.
 */

#ifndef V75_H
#define V75_H

#include "v76.h"

#include <stdbool.h>
#include <stdint.h>

#define V75_FI_USER_DATA   0x85     /* "133 D" */
#define V75_MAX_CHANNELS   8
#define V75_MAX_CAPS       16
#define V75_MAX_ALTS       8
#define V75_MAX_SIMUL      4
#define V75_MAX_DESCRIPTORS 4
#define V75_MAX_MSG        1200

/* ---- H.245 data types, as far as V.75 Annex A uses them ------------------ */

typedef enum {
    V75_AUDIO_NONE = 0,
    V75_AUDIO_G711_ALAW_64K, V75_AUDIO_G711_ALAW_56K,
    V75_AUDIO_G711_ULAW_64K, V75_AUDIO_G711_ULAW_56K,
    V75_AUDIO_G722_64K, V75_AUDIO_G722_56K, V75_AUDIO_G722_48K,
    V75_AUDIO_G723, V75_AUDIO_G728, V75_AUDIO_G729, V75_AUDIO_G729_ANNEX_A,
    V75_AUDIO_G729_W_ANNEX_B, V75_AUDIO_G729_ANNEX_A_W_ANNEX_B,
    V75_AUDIO_NONSTANDARD
} v75_audio_cap_t;

typedef struct {
    v75_audio_cap_t cap;
    int frames;                 /* INTEGER (1..256): audio frames per SDU, i.e.
                                 * V.70's "audio blocking factor" */
    bool silence_suppression;   /* g723 only */
} v75_audio_t;

typedef enum {
    V75_APP_NONE = 0,
    V75_APP_T120, V75_APP_T84, V75_APP_T434, V75_APP_X263,
    V75_APP_DSVD_CONTROL,       /* an out-of-band control channel (6.1.4 NOTE) */
    V75_APP_NONSTANDARD
} v75_app_t;

typedef enum {
    V75_DP_NONE = 0,
    V75_DP_V14_BUFFERED, V75_DP_V42_LAPM, V75_DP_HDLC_TUNNELLING,
    V75_DP_TRANSPARENT, V75_DP_SEGMENTATION_REASSEMBLY,
    V75_DP_HDLC_TUNNELLING_W_SAR, V75_DP_V120, V75_DP_V76_W_COMPRESSION
} v75_dataproto_t;

typedef struct {
    v75_app_t app;
    v75_dataproto_t protocol;
    int compression;            /* v76wCompression: P0 = 1 tx, 2 rx, 3 both; 0 none */
    int v42bis_codewords;       /* P1 */
    int v42bis_string;          /* P2 */
    int max_bit_rate;           /* V.70 Cor.1: set to 0 */
} v75_data_t;

typedef enum { V75_SR_NONE = 0, V75_SR_WITH_ADDRESS, V75_SR_WITHOUT_ADDRESS } v75_sr_t;

/* V76LogicalChannelParameters (Annex A, with Cor.1's noSuspendResume). */
typedef struct {
    int crc_len;                /* 1, 2 or 4 octets */
    int n401;                   /* 1..4095 (Cor.1) */
    bool loopback_test;
    v75_sr_t suspend_resume;
    bool uih;
    v76_mode_t mode;
    int window;                 /* ERM */
    v76_recovery_t recovery;    /* ERM */
    bool audio_header;          /* V75Parameters.audioHeaderPresent */
} v75_v76_params_t;

typedef enum { V75_MEDIA_AUDIO = 0, V75_MEDIA_DATA } v75_media_t;

typedef struct {
    int channel;                /* LogicalChannelNumber */
    bool has_port;
    int port;                   /* default 0: unspecified (Cor.1 item 3) */
    v75_media_t media;
    v75_audio_t audio;
    v75_data_t data;
    v75_v76_params_t mux;
} v75_olc_dir_t;

typedef struct {
    v75_olc_dir_t fwd;
    bool has_rev;               /* "shall be present for DSVD" */
    v75_olc_dir_t rev;
} v75_olc_t;

typedef struct {
    int forward_channel;
    int reverse_channel;
    bool has_port;
    int port;
} v75_olc_ack_t;

typedef struct {
    int forward_channel;
    int cause;                  /* OpenLogicalChannelReject.cause */
} v75_olc_reject_t;

typedef struct {
    int forward_channel;
    bool source_lcse;           /* CloseLogicalChannel.source: user / lcse */
} v75_clc_t;

/* V76Capability (Annex A). */
typedef struct {
    bool sr_with_address, sr_without_address;
    bool rej, srej, msrej;
    bool crc8, crc16, crc32;
    bool uih;
    int num_dlcs;               /* 2..8191 */
    bool two_octet_address;
    bool loopback_test;
    int n401;                   /* 1..4095 */
    int max_window;             /* 1..127 */
    bool audio_header;
} v75_v76cap_t;

typedef struct {
    int number;                 /* capabilityTableEntryNumber */
    bool is_audio;
    v75_audio_t audio;
    v75_data_t data;
} v75_capentry_t;

typedef struct {
    int n_alts;
    int alt[V75_MAX_ALTS];      /* capability numbers: exactly one may be used */
} v75_altset_t;

typedef struct {
    int number;                 /* capabilityDescriptorNumber */
    int n_sets;
    v75_altset_t set[V75_MAX_SIMUL];    /* one AlternativeCapabilitySet each */
} v75_capdesc_t;

typedef struct {
    int sequence_number;        /* 0 for DSVD */
    bool has_mux;
    v75_v76cap_t mux;
    int n_caps;
    v75_capentry_t caps[V75_MAX_CAPS];
    int n_desc;
    v75_capdesc_t desc[V75_MAX_DESCRIPTORS];
} v75_tcs_t;

typedef struct {
    int sequence_number;
    int cause;
} v75_tcs_reject_t;

/* RequestMode with one ModeDescription of one ModeElement (what DSVD needs):
 * the mode asked for, and optionally V76ModeParameters (Cor.1 item 2). */
typedef struct {
    int sequence_number;
    v75_media_t media;          /* ModeElementType: audioMode / dataMode */
    v75_audio_cap_t audio;      /* audioMode */
    int audio_frames;           /* for the codecs whose mode carries a count */
    v75_data_t data;            /* dataMode: application + protocol */
    int data_bit_rate;
    v75_sr_t v76_mode;          /* v76ModeParameters; V75_SR_NONE = absent */
    int logical_channel;        /* ModeElement.logicalChannelNumber, 0 = absent */
} v75_request_mode_t;

/* EndSessionCommand choices a DSVD terminal uses (V.70 6.3). */
typedef enum {
    V75_END_DISCONNECT = 0,
    V75_END_GSTN_TELEPHONY,     /* return to analogue telephony */
    V75_END_GSTN_V8BIS,         /* go on to another V.8 bis mode */
    V75_END_GSTN_V34_DSVD,
    V75_END_GSTN_V34_DUPLEX_FAX,
    V75_END_GSTN_V34_H324
} v75_end_kind_t;

typedef enum {
    V75_MSG_OLC = 1, V75_MSG_OLC_ACK, V75_MSG_OLC_REJECT,
    V75_MSG_CLC, V75_MSG_CLC_ACK,
    V75_MSG_TCS, V75_MSG_TCS_ACK, V75_MSG_TCS_REJECT,
    V75_MSG_END_SESSION, V75_MSG_REQUEST_MODE,
    V75_MSG_REQUEST_MODE_ACK, V75_MSG_REQUEST_MODE_REJECT
} v75_msg_type_t;

typedef struct {
    v75_msg_type_t type;
    union {
        v75_olc_t olc;
        v75_olc_ack_t olc_ack;
        v75_olc_reject_t olc_reject;
        v75_clc_t clc;
        int clc_ack_channel;
        v75_tcs_t tcs;
        int tcs_ack_sequence;
        v75_tcs_reject_t tcs_reject;
        v75_request_mode_t request_mode;
        v75_end_kind_t end_session;
        struct { int sequence_number; int response; } request_mode_ack;   /* alternative index */
        struct { int sequence_number; int cause; } request_mode_reject;   /* alternative index */
    } u;
} v75_msg_t;

/* The H.245 codec seam. */
typedef struct {
    int (*encode)(const v75_msg_t *m, uint8_t *out, int max);   /* octets, or -1 */
    int (*decode)(const uint8_t *in, int len, v75_msg_t *m);    /* 0 ok, -1 bad */
} v75_h245_codec_t;

/* H.245 (03/2022) ASN.1 aligned PER, through per.c and the generated
 * h245_schema.c.  Covers the DSVD subset (tools/h245/prune.py); a message
 * outside it fails to encode or decode.  Verified byte-for-byte against an
 * independent X.691 implementation (tools/h245/per_oracle.py). */
extern const v75_h245_codec_t v75_h245_codec;

/* V.75 8.1: user data inside an FI field encoded 133 D. */
int v75_wrap(const uint8_t *msg, int len, uint8_t *out, int max);
int v75_unwrap(const uint8_t *in, int len, const uint8_t **msg);

/* ---- Audio header (clause 9, Table 7) and segmentation header (11.1) ---- */

typedef struct {
    bool present;
    bool silence;               /* bit 0 */
    bool sid;                   /* bit 1: silence insertion descriptor */
    int seq;                    /* bits 2-6, bit 2 = LSB */
    int lost;                   /* receive side: frames missing before this one */
} v75_audio_hdr_t;

uint8_t v75_audio_header_encode(const v75_audio_hdr_t *h);
void v75_audio_header_decode(uint8_t octet, v75_audio_hdr_t *h);

#define V75_H_FINAL    0x01     /* F */
#define V75_H_BEGIN    0x02     /* B */
#define V75_H_IDLE     0x40     /* I */

/* ---- Control entity ------------------------------------------------------ */

typedef struct v75_s v75_t;

typedef enum {
    V75_REL_REMOTE_CLOSE,       /* CloseLogicalChannel received */
    V75_REL_REFUSED,            /* OpenLogicalChannelReject, or DM */
    V75_REL_LINK_LOST,          /* the DLC went away (N400, FRMR, ...) */
    V75_REL_LOCAL_CLOSE_DONE    /* our CloseLogicalChannel was acknowledged */
} v75_release_cause_t;

/* CE service primitives to the user (Table 1a / 1b).  All callbacks optional. */
typedef struct {
    void *ctx;
    void (*establish_ind)(void *ctx, const v75_olc_t *olc);
    void (*establish_conf)(void *ctx, int channel, const v75_olc_ack_t *ack);
    void (*release_ind)(void *ctx, int channel, v75_release_cause_t cause, int reason);
    /* channel = -1: the out-of-band control channel. */
    void (*setparm_ind)(void *ctx, int channel, const v75_tcs_t *caps);
    void (*setparm_conf)(void *ctx, int channel, bool ack, int reason);
    void (*session_end_ind)(void *ctx);
    void (*request_mode_ind)(void *ctx, const v75_request_mode_t *rm);
    void (*data_ind)(void *ctx, int channel, const uint8_t *data, int len,
                     const v75_audio_hdr_t *hdr);
    void (*frame_ind)(void *ctx, int channel, const uint8_t *frame, int len,
                      bool idle);       /* reassembled data protocol frame */
    void (*break_ind)(void *ctx, int channel, int option, int length_10ms);
    void (*break_conf)(void *ctx, int channel);
    void (*break_fail)(void *ctx, int channel);
    /* A frame on this channel was dropped for a bad FCS (V.76 5.3 NOTE: lets
     * a voice user conceal the lost frame). */
    void (*fcs_error_ind)(void *ctx, int channel);
} v75_user_t;

v75_t *v75_create(v76_t *mf, const v75_h245_codec_t *codec);   /* NULL = H.245 PER */
void v75_destroy(v75_t *ce);
void v75_set_user(v75_t *ce, const v75_user_t *u);

/* The MF's service-user callbacks; install with v76_set_su(). */
v76_su_t v75_mf_su(v75_t *ce);

/* CE-ESTABLISH.  req opens a channel; rsp accepts one that was indicated. */
int v75_establish_req(v75_t *ce, const v75_olc_t *olc);
int v75_establish_rsp(v75_t *ce, int channel, const v75_olc_ack_t *ack);
/* CE-RELEASE: refuse an indicated channel (6.2) ... */
int v75_establish_refuse(v75_t *ce, int channel, int cause);
/* ... or close an established one (6.3). */
int v75_release_req(v75_t *ce, int channel);

/* CE-SETPARM.  channel = -1 uses the out-of-band control channel, which must
 * have been opened with data.app = V75_APP_DSVD_CONTROL. */
int v75_setparm_req(v75_t *ce, int channel, const v75_tcs_t *caps);
int v75_setparm_rsp(v75_t *ce, int channel, bool ack, int reason);
int v75_end_session_req(v75_t *ce);                 /* EndSessionCommand disconnect */
int v75_end_session_ex(v75_t *ce, v75_end_kind_t kind);
int v75_request_mode_req(v75_t *ce, const v75_request_mode_t *rm);

/* CE-DATA request (6.5).  hdr may be NULL; for a channel opened with the
 * audio header it supplies the silence/SID flags (the sequence number is the
 * CE's).  A data-protocol channel opened with segmentation/reassembly takes a
 * whole user frame and segments it (clause 11). */
int v75_data_req(v75_t *ce, int channel, const uint8_t *data, int len,
                 const v75_audio_hdr_t *hdr);
/* Clause 11: report an HDLC idle condition at the user interface. */
int v75_sar_idle(v75_t *ce, int channel, bool idle);

/* Clause 10: break. option bit 7 = discard, bit 6 = sequencing (Table B.2). */
int v75_break_req(v75_t *ce, int channel, int option, int length_10ms);

void v75_advance_ms(v75_t *ce, int ms);

/* Introspection. */
int v75_dlci_of(const v75_t *ce, int channel);
int v75_channel_of(const v75_t *ce, int dlci);
bool v75_channel_open(const v75_t *ce, int channel);
int v75_control_channel(const v75_t *ce);       /* -1 if none */

/* Translate a V.75 channel description into V.76 DLC parameters and back. */
void v75_olc_to_mf(const v75_olc_t *olc, bool opener, v76_dlc_params_t *p);

#endif
