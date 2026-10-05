/*
 * v70.h -- ITU-T V.70 (08/96) + Cor.1 (01/2005): simultaneous transmission of
 * data and digitally encoded voice over the GSTN (DSVD).
 *
 * V.70 is a profile: it assembles the multiplex function of V.76 and the
 * control entity of V.75 around a speech coder and a data channel, and hands
 * the multiplexed bit stream to a V.34 or V.32bis datapump.  This module is
 * that assembly:
 *
 *   SCF (5.2, 6.2)   opens the optional out-of-band control channel, exchanges
 *                    capabilities, opens one bidirectional voice DLC and one
 *                    data DLC, resolves open collisions (6.2.2: the initiator
 *                    refuses the responder's request), closes everything
 *                    (6.3); roles from V.8bis (6.1.4)
 *   voice (5.4)      one coded frame per audio frame period, `blocking` of
 *                    them per multiplex frame (the audio blocking factor,
 *                    default 1), with the V.75 audio header whenever the
 *                    factor exceeds 1 (mandatory then) or when negotiated;
 *                    silence and SID frames flagged in that header; an FCS
 *                    error on a voice frame is surfaced as a lost frame (V.76
 *                    5.3 NOTE)
 *   data (5.3)       asynchronous DTE bytes over an ERM channel, or
 *                    synchronous HDLC frames over UNERM with the Annex A
 *                    tunnelling (flags and transparency removed, one frame per
 *                    UI frame), optionally with V.75 segmentation/reassembly
 *   interfaces       v70_tx_get_bit()/v70_rx_put_bit() are the datapump's
 *                    synchronous stream; time is derived from the transmit bit
 *                    count at the configured line rate
 *
 * Not here, and why (see also v75.h): the speech coder.  G.729 Annex A is
 * mandatory in V.70 (5.4) but is neither in `ITU Docs/` nor reimplemented from
 * memory; voice frames are opaque octets to this module, supplied and
 * consumed through v70_io_t.  V.8bis, which V.70 6.1 uses to enter DSVD mode,
 * is not run by this module: the caller says when the modem has trained
 * (v70_start()) and which role it holds.  H.245 is on the far side of the V.75
 * codec seam.
 */

#ifndef V70_H
#define V70_H

#include "v75.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    V70_IDLE = 0,
    V70_ESTABLISHING,
    V70_ACTIVE,
    V70_ENDING,
    V70_ENDED,
    V70_FAILED
} v70_state_t;

typedef enum {
    V70_DATA_ASYNC_ERM = 0,     /* asynchronous DTE bytes, error recovery mode */
    V70_DATA_TUNNEL_UNERM,      /* HDLC frames over UNERM (V.70 Annex A) */
    V70_DATA_TUNNEL_SAR         /* the same, with V.75 clause 11 segmentation */
} v70_data_mode_t;

typedef struct {
    bool initiator;             /* from the V.8bis exchange (6.1.4) */
    int line_bit_rate;          /* bit/s of the datapump */

    bool oob_control;           /* open an out-of-band control channel (6.2.1) */
    bool suspend_resume;        /* agreed beforehand by V.8bis or the OOB channel */
    int audio_blocking_factor;  /* default 1 */
    bool audio_header;          /* forced on when the blocking factor > 1 */
    bool voice_crc8;            /* 8-bit CRC on the voice DLC (V.76 App. II rationale) */
    v75_audio_cap_t voice_codec;
    int voice_frame_octets;     /* coded frame: G.729 Annex A is 10 */
    int voice_frame_ms;         /* and 10 ms */

    v70_data_mode_t data_mode;
    int data_n401;              /* default 128 */
    int data_window;            /* default 15 */
    v76_recovery_t data_recovery;
    int t401_ms;
    int first_channel;          /* LogicalChannelNumber base; 0 = by role */
} v70_config_t;

void v70_config_default(v70_config_t *c, bool initiator);

/* The voice and data processing functions and the DTE. */
typedef struct {
    void *ctx;
    /* Called once per audio frame period.  Return the coded frame's octets
     * (<= max), or 0 for none.  *silence and *sid mark silence-compression
     * frames; they only reach the peer when the audio header is in use. */
    int (*voice_get_frame)(void *ctx, uint8_t *frame, int max, bool *silence, bool *sid);
    /* A received frame (frame != NULL), or a frame known to be lost
     * (frame == NULL, from a sequence gap or an FCS error). */
    void (*voice_put_frame)(void *ctx, const uint8_t *frame, int len,
                            const v75_audio_hdr_t *hdr);
    /* Asynchronous DTE: next byte to send, or -1.  Frame mode: next octet of
     * the DTE's HDLC octet stream (flags, 0x7D transparency), or -1. */
    int (*dte_pull)(void *ctx);
    void (*dte_push)(void *ctx, uint8_t byte);
    void (*break_ind)(void *ctx, int option, int length_10ms);
    void (*state_ind)(void *ctx, v70_state_t state);
} v70_io_t;

typedef struct {
    uint64_t voice_tx, voice_rx, voice_lost, voice_fcs_lost;
    uint64_t data_octets_tx, data_octets_rx;
    uint64_t frames_tx, frames_rx;      /* tunnelled HDLC frames */
} v70_stats_t;

typedef struct v70_s v70_t;

v70_t *v70_create(const v70_config_t *cfg, const v70_io_t *io);
void v70_destroy(v70_t *t);

/* The modem has trained: DSVD mode begins (6.2). */
void v70_start(v70_t *t);
/* Either role may open a channel on its own (5.2 a: the SCF requests a DLC).
 * Returns the LogicalChannelNumber, or -1.  On a crossing request of the same
 * data type the initiator's wins (6.2.2). */
int v70_open_voice(v70_t *t);
int v70_open_data(v70_t *t);
/* End DSVD mode: close all DLCs (6.3). */
int v70_end(v70_t *t);
int v70_break(v70_t *t, int option, int length_10ms);

v70_state_t v70_state(const v70_t *t);
const v70_stats_t *v70_stats(const v70_t *t);
const v75_tcs_t *v70_peer_capabilities(const v70_t *t);   /* NULL until received */
v76_t *v70_mf(v70_t *t);
v75_t *v70_ce(v70_t *t);
int v70_voice_channel(const v70_t *t);                    /* -1 if not open */
int v70_data_channel(const v70_t *t);
/* Bits of the transmit stream consumed so far, and the ms clock they imply. */
uint64_t v70_tx_bits(const v70_t *t);

/* The datapump's synchronous stream. */
int v70_tx_get_bit(v70_t *t);
void v70_rx_put_bit(v70_t *t, int bit);
void v70_tx_fill_bytes(v70_t *t, uint8_t *out, int len);
void v70_rx_push_bytes(v70_t *t, const uint8_t *in, int len);

/* ---- V.70 Annex A: UNERM tunnelling (ISO/IEC 3309 transparency) --------- */

#define V70_HDLC_FLAG     0x7E
#define V70_HDLC_ESCAPE   0x7D      /* control escape octet, 0 1 1 1 1 1 0 1 */

/* Frame -> flag, transparency-protected frame, flag.  Returns octets, or -1. */
int v70_tunnel_encode(const uint8_t *frame, int len, uint8_t *out, int max);

typedef struct {
    uint8_t buf[V76_MAX_N401 + 16];
    int len;
    bool in_frame;
    bool escape;
    bool overflow;
} v70_tunnel_rx_t;

void v70_tunnel_rx_init(v70_tunnel_rx_t *r);
/* Feed one octet from the DTE.  Returns the length of a completed frame,
 * which is then in r->buf, or 0 (nothing complete) or -1 (a frame was dropped:
 * too long, or an escape before a flag). */
int v70_tunnel_rx_put(v70_tunnel_rx_t *r, uint8_t octet);

#endif
