/*
 * v8bis_ie.h -- V.8bis information field codec (clause 8).
 *
 * The information field is: identification field I (type and revision octet,
 * NPar(1), SPar(1) and a Par(2) block per set SPar(1) bit), standard field S
 * (the same tree shape), and an optional non-standard field NS.  Parameters
 * are bits in octets whose delimiter bits say where each block ends (8.2.3):
 * bit 8 at level 1 and for whole Par(2) blocks, bit 7 at levels 2 and 3.
 *
 * Decoding is structural, as 8.2.3 requires ("receivers shall parse all
 * information blocks and ignore information that is not understood"): reserved
 * bits and unknown SPar(1) blocks are walked past and reported, not rejected.
 * Encoding emits only what this struct can name.
 *
 * Message type 1011 is NAK(4) in V.8bis Table 3; V.92 Tables 3/5/12/14 put its
 * QC2/QCA2 identification fields under the same nibble.  Such a message, or
 * any type not in Table 3, is returned with its octets unparsed in `payload`.
 */
#ifndef V8BIS_IE_H
#define V8BIS_IE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "v8bis_msg.h"

typedef enum {
    V8BIS_MT_MS = 1,
    V8BIS_MT_CL = 2,
    V8BIS_MT_CLR = 3,
    V8BIS_MT_ACK1 = 4,
    V8BIS_MT_ACK2 = 5,
    V8BIS_MT_NAK1 = 8,
    V8BIS_MT_NAK2 = 9,
    V8BIS_MT_NAK3 = 10,
    V8BIS_MT_NAK4 = 11
} v8bis_msg_type_t;

const char *v8bis_msg_type_name(unsigned type);

/* Table 5-1, identification NPar(1) bits (bit 8 is the delimiter). */
#define V8BIS_ID_V8            0x01
#define V8BIS_ID_SHORT_V8      0x02
#define V8BIS_ID_MORE_INFO     0x04   /* additional information available (9.10) */
#define V8BIS_ID_TX_ACK1       0x08
#define V8BIS_ID_NON_STANDARD  0x40   /* an NS field follows S */
/* Table 5-3, network type NPar(2) bits. */
#define V8BIS_NET_CELLULAR     0x01
#define V8BIS_NET_ISDN         0x02
#define V8BIS_NET_NON_STANDARD 0x20

/* Table 6-2, standard SPar(1). */
#define V8BIS_S_DATA           0x01
#define V8BIS_S_SVD            0x02
#define V8BIS_S_H324           0x04
#define V8BIS_S_V18            0x08
#define V8BIS_S_ANALOGUE_TEL   0x20
#define V8BIS_S_T101           0x40
/* Table 6-1: S NPar(1) bit 7. */
#define V8BIS_S_NPAR1_NON_STANDARD 0x40

/* Table 6-3, data NPar(2).  Octet 1: */
#define V8BIS_DATA_TRANSPARENT 0x01
#define V8BIS_DATA_V42         0x02
#define V8BIS_DATA_V42BIS      0x04
#define V8BIS_DATA_V14         0x08
#define V8BIS_DATA_T120        0x10
#define V8BIS_DATA_NONSTD      0x20
/* octet 2: */
#define V8BIS_DATA2_T84        0x01
#define V8BIS_DATA2_T434       0x02
#define V8BIS_DATA2_V80        0x04
#define V8BIS_DATA2_V34        0x10
#define V8BIS_DATA2_V32BIS     0x20
/* octet 3: */
#define V8BIS_DATA3_V32        0x01
#define V8BIS_DATA3_V22BIS     0x02
#define V8BIS_DATA3_V22        0x04
#define V8BIS_DATA3_V21        0x08

/* Table 6-4, simultaneous voice and data NPar(2).  Octet 1: */
#define V8BIS_SVD_V70          0x01
#define V8BIS_SVD_V61          0x02
#define V8BIS_SVD_V34          0x08
#define V8BIS_SVD_V32BIS       0x10
#define V8BIS_SVD_NONSTD       0x20
/* octet 2 is the data octet 1 layout with V.80 at 0x20; octet 3 is T.84/T.434 */

/* Table 6-5, H.324.  NPar(2): */
#define V8BIS_H324_VIDEO       0x01
#define V8BIS_H324_AUDIO       0x02
#define V8BIS_H324_ENCRYPTION  0x04
#define V8BIS_H324_NONSTD      0x20
#define V8BIS_H324_SPAR2_DATA  0x01
/* NPar(3) of its Data SPar(2): */
#define V8BIS_H324D_V42        0x01
#define V8BIS_H324D_V14        0x02
#define V8BIS_H324D_PPP        0x04
#define V8BIS_H324D_T120       0x08
#define V8BIS_H324D_T84        0x10
#define V8BIS_H324D_T434       0x20

/* Tables 6-6, 6-7, 6-8 NPar(2). */
#define V8BIS_V18_V21          0x01
#define V8BIS_V18_V61          0x02
#define V8BIS_V18_NONSTD       0x20
#define V8BIS_TEL_VOICE        0x01
#define V8BIS_TEL_RECORDER     0x02
#define V8BIS_TEL_BRIDGE       0x04
#define V8BIS_TEL_NONSTD       0x20
#define V8BIS_T101_DUPLEX      0x01
#define V8BIS_T101_V29_SHORT   0x02
#define V8BIS_T101_V27TER      0x04
#define V8BIS_T101_NONSTD      0x20

#define V8BIS_NS_BLOCKS_MAX 4
#define V8BIS_NS_PROVIDER_MAX 8
#define V8BIS_NS_DATA_MAX 40

/* 8.5, Figure 10, with the T.35 country code taken as one octet (the figure's
 * own note: "currently defined as one octet in length"). */
typedef struct {
    uint8_t country;
    uint8_t provider_len;
    uint8_t provider[V8BIS_NS_PROVIDER_MAX];
    uint8_t data_len;
    uint8_t data[V8BIS_NS_DATA_MAX];
} v8bis_ns_block_t;

typedef struct {
    unsigned type;                    /* Table 3 nibble */
    unsigned revision;                /* Table 4; the receiver ignores it (V.92 notes) */
    bool known_type;                  /* in V8BIS_MT_* and parsed */

    /* identification field */
    uint8_t id_npar1;                 /* V8BIS_ID_* */
    bool network_type;                /* SPar(1) bit 1: a network type block follows */
    uint8_t network_npar2;            /* V8BIS_NET_* */

    /* standard field */
    uint8_t s_npar1;                  /* V8BIS_S_NPAR1_* */
    uint8_t s_spar1;                  /* V8BIS_S_* */
    uint8_t data[3];                  /* Table 6-3 octets */
    uint8_t svd[3];
    uint8_t h324_npar2;
    uint8_t h324_spar2;               /* V8BIS_H324_SPAR2_DATA */
    uint8_t h324_data;                /* NPar(3) of that SPar(2) */
    uint8_t v18;
    uint8_t analogue_tel;
    uint8_t t101;

    /* non-standard field */
    unsigned ns_count;
    v8bis_ns_block_t ns[V8BIS_NS_BLOCKS_MAX];

    /* what decoding walked past rather than understood */
    unsigned ignored_blocks;          /* Par(2) blocks of reserved SPar(1) bits */
    bool reserved_bits;               /* a reserved parameter bit was set */
    bool delimiter_anomaly;           /* bit 7/8 not as 8.2.3 gives it, structure still parsed */
    bool extra_octets;                /* octets past the defined ones in a block, or trailing */
    unsigned ns_dropped;              /* NS blocks beyond what ns[] holds */

    /* unparsed types: everything after the type/revision octet */
    uint8_t payload[V8BIS_MAX_INFO_OCTETS];
    unsigned payload_len;
} v8bis_msg_t;

typedef enum {
    V8BIS_IE_OK = 0,
    V8BIS_IE_EMPTY = -1,
    V8BIS_IE_TRUNCATED = -2,          /* a block ran off the end of the field */
    V8BIS_IE_TOO_LONG = -4,           /* more than 64 octets (8.6) */
    V8BIS_IE_BAD_NS = -5,
    V8BIS_IE_BAD_ARG = -6
} v8bis_ie_err_t;

const char *v8bis_ie_err_name(int err);

void v8bis_msg_init(v8bis_msg_t *m, unsigned type);
/* Returns the number of octets written, or a negative v8bis_ie_err_t.  ACK and
 * NAK types are one octet; a type outside Table 3 emits its payload verbatim. */
int v8bis_msg_encode(const v8bis_msg_t *m, uint8_t *out, size_t cap);
/* Returns 0 or a negative v8bis_ie_err_t. */
int v8bis_msg_decode(const uint8_t *in, size_t len, v8bis_msg_t *m);

#endif
