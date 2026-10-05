/*
 * clear_channel.h — 64 kbit/s clear channel and V.120 over the DS0
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
 * Neither mode has a handshake: as on ISDN, the bearer is agreed outside the
 * channel (here, both ends configured alike with AT+MS), and the data starts
 * when the call connects.
 *
 * V.120 is NOT in "ITU Docs/" and could not be fetched from this
 * environment, so the frame layout below is from the Recommendation as
 * commonly implemented (ISDN TAs, Linux isdn4linux) rather than checked
 * clause by clause: default LLI 256, address octets 0x08 0x01 (+ C/R),
 * control 0x03 (UI), header 0x83 (E=1, B=1, F=1).  Check it against V.120
 * (10/96) before claiming interoperability with ISDN equipment.
 */
#ifndef CLEAR_CHANNEL_H
#define CLEAR_CHANNEL_H

#include <stdbool.h>
#include <stdint.h>

#include <spandsp.h>

typedef enum {
    CC_CLEAR = 0,
    CC_V120  = 1
} cc_mode_t;

/* CC_CLEAR: next line bit, or <0 for "nothing to send" (sent as mark, 1). */
typedef int  (*cc_get_bit_fn)(void *ctx);
typedef void (*cc_put_bit_fn)(void *ctx, int bit);
/* CC_V120: next DTE byte to send, or -1 for none; one received DTE byte. */
typedef int  (*cc_pull_byte_fn)(void *ctx);
typedef void (*cc_push_byte_fn)(void *ctx, uint8_t byte);

#define CC_V120_DEFAULT_LLI   256
/* User octets per V.120 frame: N201 is 260 octets of information field,
 * which here is the header octet plus data. */
#define CC_V120_MAX_DATA      256

/* V.120 header octet (terminal adaption header) bits. */
#define CC_V120_H_E           0x80   /* no control-state octet follows */
#define CC_V120_H_BR          0x40   /* break */
#define CC_V120_H_B           0x02   /* begin (first segment) */
#define CC_V120_H_F           0x01   /* final (last segment) */

typedef struct {
    cc_mode_t mode;
    bool r56;                /* restricted 56 kbit/s: 7 bits per octet */
    bool caller;             /* V.120 C/R: originator commands with C/R 0 */
    int lli;

    cc_get_bit_fn get_bit;
    cc_put_bit_fn put_bit;
    cc_pull_byte_fn pull;
    cc_push_byte_fn push;
    void *ctx;

    hdlc_tx_state_t *htx;
    hdlc_rx_state_t *hrx;
    bool tx_frame_queued;

    /* Statistics */
    uint64_t tx_octets, rx_octets;
    uint64_t tx_frames, rx_frames;
    uint64_t tx_data_bytes, rx_data_bytes;
    uint64_t rx_bad_frames;      /* FCS errors, aborts, runts */
    uint64_t rx_unsupported;     /* I-frames and other non-UI frames */
    uint64_t rx_breaks;
} clear_channel_t;

int  cc_init_clear(clear_channel_t *cc, bool r56,
                   cc_get_bit_fn get_bit, cc_put_bit_fn put_bit, void *ctx);
int  cc_init_v120(clear_channel_t *cc, bool r56, bool caller,
                  cc_pull_byte_fn pull, cc_push_byte_fn push, void *ctx);
void cc_release(clear_channel_t *cc);

/* Fill n DS0 octets to transmit / consume n received DS0 octets. */
void cc_tx(clear_channel_t *cc, uint8_t *octets, int n);
void cc_rx(clear_channel_t *cc, const uint8_t *octets, int n);

/* 64000 or 56000. */
int  cc_line_rate(const clear_channel_t *cc);

/* The address + control + header V.120 puts in front of user data (4
 * octets), for tests and for anyone checking a capture. */
void cc_v120_frame_header(const clear_channel_t *cc, uint8_t hdr[4]);

#endif
