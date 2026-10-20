/*
 * mnp.h — the V.42 Annex A "alternative procedure" (MNP error control,
 * classes 2-4), which a V.42 DCE may fall back to when detection finds no
 * LAPM peer (V.42 7.2.1.2; 7.9: disconnect or V.14 only after "establishment
 * of any alternative error-correcting capability has also failed").  Annex A
 * is in V.42 (11/88), (03/93) and (10/96) -- all in `ITU Docs/`, clauses cited
 * from 10/96 -- and was deleted in the 2002 revision.
 *
 * What is implemented:
 *   - Protocol establishment (A.6.4, A.7.1): initiator LR, responder LR,
 *     initiator's confirming LA, all start-stop octet-oriented; negotiation
 *     of framing mode (2 octet-oriented, 3 bit-oriented), k, N401 and the
 *     data phase optimisation facilities (256-octet LTs, fixed-field LT/LA).
 *   - Octet-oriented framing (A.3: SYN DLE STX ... DLE ETX, DLE doubling,
 *     CRC-16 x^16+x^15+x^2+1) and bit-oriented framing (A.4: flags, zero
 *     insertion and the 8.1.1.6.1 FCS) on the synchronous datapump bits.
 *   - Data phase (A.7.3): LT/LA, modulo 256, receive credit (0 is the
 *     receiver-busy condition, A.6.6.2.2), the A.7.3.3.2 acknowledgement
 *     rules, go-back retransmission (A.7.3.5), T401 by the A.7.5.1 formula,
 *     T402, T404, N400 = 12; LD (A.6.5); LN answered with LNA (A.7.4).
 *
 * Not implemented: sending LN (no break signal on the DTE side here), MNP 4
 * adaptive packet assembly (an LT carries what the DTE has queued, up to
 * N401), and MNP 5 compression.  MNP 5 is not in V.42 at all, and has no check
 * after decompression, so an encoder that is not bit-exact with a peer's
 * corrupts user data silently; there is no reference here to be exact with.
 *
 * No foreign MNP DCE has been run against this: see docs/mnp.md.
 *
 * Timers run on the transmit bit clock, as SpanDSP's V.42 does: call
 * mnp_tx_bit() and mnp_rx_bit() once per line bit each.
 */

#ifndef MNP_H
#define MNP_H

#include <stdbool.h>
#include <stdint.h>

typedef struct mnp_s mnp_t;

enum {
    /* Answerer: a valid LR arrived.  Reported once, from mnp_rx_bit(), while
     * the answerer is not yet connected (V.42 7.2.1.3: an alternative
     * procedure protocol frame ends detection). */
    MNP_STATUS_LR_SEEN = 1,
    MNP_STATUS_CONNECTED,
    /* No link: the originator's LRs went unanswered, or the answerer waited
     * out its establishment timeout without a valid LR. */
    MNP_STATUS_ESTABLISH_FAILED,
    /* LD received, or N400 reached (LD sent), or mnp_stop(). */
    MNP_STATUS_DISCONNECTED
};

/* Fill up to max octets of LT user data; return how many (0 = none now). */
typedef int (*mnp_get_data_fn)(void *user, uint8_t *buf, int max);
/* In-sequence LT user data, delivered exactly once. */
typedef void (*mnp_put_data_fn)(void *user, const uint8_t *buf, int len);
typedef void (*mnp_status_fn)(void *user, int status);

typedef struct {
    int service_class;      /* negotiated: 2 octet-oriented, 3 bit-oriented */
    int k;                  /* negotiated credit (window) */
    int n401;               /* negotiated maximum LT user data */
    int optimisation;       /* LR parameter 8 bits agreed by both */
    uint64_t lt_tx, lt_rx;
    uint64_t la_tx, la_rx;
    uint64_t lr_tx, lr_rx;
    uint64_t retransmitted; /* LTs sent again */
    uint64_t timeouts;      /* go-backs started by the retransmission timer */
    uint64_t fcs_errors;    /* complete frames whose CRC failed */
    uint64_t out_of_sequence;
} mnp_stats_t;

/* service_class is what this end offers (2 or 3).  The answerer's
 * establishment timeout starts at init; 0 waits forever (the case where V.42
 * detection runs alongside and owns the verdict). */
mnp_t *mnp_init(bool originator, int service_class, int bit_rate,
                mnp_get_data_fn get, mnp_put_data_fn put,
                mnp_status_fn status, void *user);
void mnp_free(mnp_t *m);

/* Originator: how many LRs before MNP_STATUS_ESTABLISH_FAILED.  Answerer:
 * the wait for an LR, in ms (0 = forever).  Both count from now. */
void mnp_set_establish_limit(mnp_t *m, int originator_lr_tries, int answerer_wait_ms);

void mnp_set_bit_rate(mnp_t *m, int bit_rate);
/* Receiver busy: advertise credit 0 until cleared. */
void mnp_set_local_busy(mnp_t *m, bool busy);
/* Leave the link and report MNP_STATUS_DISCONNECTED, without an LD (A.7.2.1
 * recommends not sending one). */
void mnp_stop(mnp_t *m);

int mnp_tx_bit(mnp_t *m);
void mnp_rx_bit(mnp_t *m, int bit);

bool mnp_is_connected(const mnp_t *m);
void mnp_get_stats(const mnp_t *m, mnp_stats_t *st);

/* Exposed for tests.  mnp_crc16: the octet-oriented CRC (A.3.6), preset 0.
 * mnp_fcs16: the bit-oriented FCS register (8.1.1.6.1), preset 0xFFFF and
 * sent complemented. */
uint16_t mnp_crc16(uint16_t crc, const uint8_t *buf, int len);
uint16_t mnp_fcs16(uint16_t crc, const uint8_t *buf, int len);

#endif
