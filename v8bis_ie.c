/*
 * v8bis_ie.c -- V.8bis information field codec.  Clause 8 and Tables 3-6.
 *
 * Wire layout of an MS, CL or CLR information field:
 *
 *   octet 0            type (bits 1-4) and revision (bits 5-8), Tables 3 and 4
 *   I: NPar(1)         level 1 block, bit 8 delimits (1 = last octet)
 *      SPar(1)         level 1 block; bit 1 = network type
 *      Par(2) blocks   one per set SPar(1) bit, ascending
 *   S: NPar(1), SPar(1), Par(2) blocks, the same shape
 *   NS blocks          only when the identification NPar(1) says so
 *
 * A Par(2) block is NPar(2) octets, then optionally SPar(2) octets, then
 * NPar(3) blocks, one per set SPar(2) bit.  Bit 7 delimits each of those
 * sub-blocks (1 = last octet), and bit 8 is 1 on the last octet of the whole
 * Par(2) block only -- which is what lets a receiver skip a block it does not
 * understand without knowing what is in it, and is how decoding finds the end.
 */
#include "v8bis_ie.h"

#include <string.h>

const char *v8bis_msg_type_name(unsigned type)
{
    switch (type) {
    case V8BIS_MT_MS:   return "MS";
    case V8BIS_MT_CL:   return "CL";
    case V8BIS_MT_CLR:  return "CLR";
    case V8BIS_MT_ACK1: return "ACK(1)";
    case V8BIS_MT_ACK2: return "ACK(2)";
    case V8BIS_MT_NAK1: return "NAK(1)";
    case V8BIS_MT_NAK2: return "NAK(2)";
    case V8BIS_MT_NAK3: return "NAK(3)";
    case V8BIS_MT_NAK4: return "NAK(4)/V.92 QC2";
    default:            return "reserved";
    }
}

const char *v8bis_ie_err_name(int err)
{
    switch (err) {
    case V8BIS_IE_OK:             return "ok";
    case V8BIS_IE_EMPTY:          return "empty";
    case V8BIS_IE_TRUNCATED:      return "truncated";
    case V8BIS_IE_TOO_LONG:       return "too long";
    case V8BIS_IE_BAD_NS:         return "bad non-standard block";
    case V8BIS_IE_BAD_ARG:        return "bad argument";
    default:                      return "?";
    }
}

void v8bis_msg_init(v8bis_msg_t *m, unsigned type)
{
    memset(m, 0, sizeof(*m));
    m->type = type;
    m->revision = 1;
}

static bool type_has_fields(unsigned t)
{
    return t == V8BIS_MT_MS || t == V8BIS_MT_CL || t == V8BIS_MT_CLR;
}

static bool type_is_ack_nak(unsigned t)
{
    return t == V8BIS_MT_ACK1 || t == V8BIS_MT_ACK2 || t == V8BIS_MT_NAK1
           || t == V8BIS_MT_NAK2 || t == V8BIS_MT_NAK3;
}

/* defined bits per field, for reserved-bit reporting and for refusing to
 * encode something this struct cannot name */
#define M_ID_NPAR1   (V8BIS_ID_V8 | V8BIS_ID_SHORT_V8 | V8BIS_ID_MORE_INFO | V8BIS_ID_TX_ACK1 | V8BIS_ID_NON_STANDARD)
#define M_NET        (V8BIS_NET_CELLULAR | V8BIS_NET_ISDN | V8BIS_NET_NON_STANDARD)
#define M_S_NPAR1    V8BIS_S_NPAR1_NON_STANDARD
#define M_S_SPAR1    (V8BIS_S_DATA | V8BIS_S_SVD | V8BIS_S_H324 | V8BIS_S_V18 | V8BIS_S_ANALOGUE_TEL | V8BIS_S_T101)
#define M_DATA0      0x3f
#define M_DATA1      0x37
#define M_DATA2      0x0f
#define M_SVD0       0x3b
#define M_SVD1       0x3f
#define M_SVD2       0x03
#define M_H324       0x27
#define M_H324D      0x3f
#define M_V18        0x23
#define M_TEL        0x27
#define M_T101       0x27

/* ---- encoder ---------------------------------------------------------- */

typedef struct {
    uint8_t *p;
    size_t n, cap;
    bool over;
} wr_t;

static void w8(wr_t *w, unsigned v)
{
    if (w->n >= w->cap) {
        w->over = true;
        return;
    }
    w->p[w->n++] = (uint8_t)v;
}

static void l1_block(wr_t *w, unsigned payload)
{
    w8(w, 0x80u | (payload & 0x7fu));
}

/* NPar(2) octets only. */
static void par2_npar_only(wr_t *w, const uint8_t *oct, unsigned n)
{
    for (unsigned i = 0; i < n; i++)
        w8(w, (oct[i] & 0x3fu) | (i + 1 == n ? 0xc0u : 0u));
}

static unsigned trimmed(const uint8_t *oct, unsigned n)
{
    while (n > 1 && oct[n - 1] == 0)
        n--;
    return n;
}

static bool within(unsigned v, unsigned mask)
{
    return (v & ~mask) == 0;
}

static int encode_fields(const v8bis_msg_t *m, wr_t *w)
{
    uint8_t one[1];

    if (!within(m->id_npar1, M_ID_NPAR1) || !within(m->network_npar2, M_NET)
        || !within(m->s_npar1, M_S_NPAR1) || !within(m->s_spar1, M_S_SPAR1)
        || !within(m->data[0], M_DATA0) || !within(m->data[1], M_DATA1)
        || !within(m->data[2], M_DATA2) || !within(m->svd[0], M_SVD0)
        || !within(m->svd[1], M_SVD1) || !within(m->svd[2], M_SVD2)
        || !within(m->h324_npar2, M_H324) || !within(m->h324_spar2, V8BIS_H324_SPAR2_DATA)
        || !within(m->h324_data, M_H324D) || !within(m->v18, M_V18)
        || !within(m->analogue_tel, M_TEL) || !within(m->t101, M_T101))
        return V8BIS_IE_BAD_ARG;
    if (m->ns_count > V8BIS_NS_BLOCKS_MAX
        || (m->ns_count && !(m->id_npar1 & V8BIS_ID_NON_STANDARD)))
        return V8BIS_IE_BAD_ARG;

    l1_block(w, m->id_npar1);
    l1_block(w, m->network_type ? 0x01u : 0u);
    if (m->network_type) {
        one[0] = m->network_npar2;
        par2_npar_only(w, one, 1);
    }

    l1_block(w, m->s_npar1);
    l1_block(w, m->s_spar1);
    if (m->s_spar1 & V8BIS_S_DATA)
        par2_npar_only(w, m->data, trimmed(m->data, 3));
    if (m->s_spar1 & V8BIS_S_SVD)
        par2_npar_only(w, m->svd, trimmed(m->svd, 3));
    if (m->s_spar1 & V8BIS_S_H324) {
        if (m->h324_spar2 & V8BIS_H324_SPAR2_DATA) {
            w8(w, 0x40u | (m->h324_npar2 & 0x3fu));        /* NPar(2): bit 7 last, bit 8 more */
            w8(w, 0x40u | (m->h324_spar2 & 0x3fu));        /* SPar(2) */
            w8(w, 0xc0u | (m->h324_data & 0x3fu));         /* NPar(3), ends the block */
        } else {
            one[0] = m->h324_npar2;
            par2_npar_only(w, one, 1);
        }
    }
    if (m->s_spar1 & V8BIS_S_V18) {
        one[0] = m->v18;
        par2_npar_only(w, one, 1);
    }
    if (m->s_spar1 & V8BIS_S_ANALOGUE_TEL) {
        one[0] = m->analogue_tel;
        par2_npar_only(w, one, 1);
    }
    if (m->s_spar1 & V8BIS_S_T101) {
        one[0] = m->t101;
        par2_npar_only(w, one, 1);
    }

    if (m->id_npar1 & V8BIS_ID_NON_STANDARD) {
        for (unsigned i = 0; i < m->ns_count; i++) {
            const v8bis_ns_block_t *b = &m->ns[i];

            if (b->provider_len > V8BIS_NS_PROVIDER_MAX || b->data_len > V8BIS_NS_DATA_MAX)
                return V8BIS_IE_BAD_NS;
            w8(w, 1u + 1u + b->provider_len + b->data_len);   /* K + L + M + 1 */
            w8(w, b->country);
            w8(w, b->provider_len);
            for (unsigned k = 0; k < b->provider_len; k++)
                w8(w, b->provider[k]);
            for (unsigned k = 0; k < b->data_len; k++)
                w8(w, b->data[k]);
        }
    }
    return 0;
}

int v8bis_msg_encode(const v8bis_msg_t *m, uint8_t *out, size_t cap)
{
    /* Build in a private buffer one octet over the 8.6 limit so "too long" is
     * told apart from "the caller's buffer is small". */
    uint8_t tmp[V8BIS_MAX_INFO_OCTETS + 1];
    wr_t w = {tmp, 0, sizeof(tmp), false};
    int rc = 0;

    if (!m || !out || m->type > 15 || m->revision > 15)
        return V8BIS_IE_BAD_ARG;
    w8(&w, (m->revision << 4) | m->type);
    if (type_has_fields(m->type)) {
        rc = encode_fields(m, &w);
    } else if (!type_is_ack_nak(m->type)) {
        if (m->payload_len > sizeof(m->payload))
            return V8BIS_IE_BAD_ARG;
        for (unsigned i = 0; i < m->payload_len; i++)
            w8(&w, m->payload[i]);
    }
    if (rc)
        return rc;
    if (w.over || w.n > V8BIS_MAX_INFO_OCTETS)
        return V8BIS_IE_TOO_LONG;
    if (w.n > cap)
        return V8BIS_IE_BAD_ARG;
    memcpy(out, tmp, w.n);
    return (int)w.n;
}

/* ---- decoder ---------------------------------------------------------- */

typedef struct {
    const uint8_t *p;
    size_t len, pos;
} rd_t;

/* Level 1 block: octets up to and including the one with bit 8 set. */
static int l1_read(rd_t *r, uint8_t *first, unsigned *count, uint8_t *orall)
{
    unsigned n = 0;
    uint8_t acc = 0;

    for (;;) {
        uint8_t o;

        if (r->pos >= r->len)
            return V8BIS_IE_TRUNCATED;
        o = r->p[r->pos++];
        if (n == 0)
            *first = o & 0x7f;
        else
            acc |= o & 0x7f;
        n++;
        if (o & 0x80)
            break;
    }
    *count = n;
    *orall = acc;
    return 0;
}

#define P2_N2 6
#define P2_S2 2
#define P2_N3 4
#define P2_N3O 2

typedef struct {
    uint8_t n2[P2_N2];
    unsigned n2n;               /* octets in the block, whether or not stored */
    uint8_t s2[P2_S2];
    unsigned s2n;
    uint8_t n3[P2_N3][P2_N3O];
    unsigned n3_blocks;         /* NPar(3) blocks present */
    bool has_s2;
    bool anomaly;
    bool extra;                 /* more octets than stored */
} par2_t;

/* One Par(2) block.  Bit 8 ends the block; bit 7 splits it. */
static int par2_read(rd_t *r, par2_t *b)
{
    size_t start = r->pos, end;
    size_t i;

    memset(b, 0, sizeof(*b));
    for (;;) {
        if (r->pos >= r->len)
            return V8BIS_IE_TRUNCATED;
        if (r->p[r->pos++] & 0x80)
            break;
    }
    end = r->pos;

    i = start;
    /* NPar(2) up to the octet with bit 7 */
    for (;;) {
        uint8_t o;

        if (i >= end) {                               /* bit 8 closed the block inside NPar(2) */
            b->anomaly = true;
            return 0;
        }
        o = r->p[i++];
        if (b->n2n < P2_N2)
            b->n2[b->n2n] = o & 0x3f;
        else
            b->extra = true;
        b->n2n++;
        if (o & 0x40)
            break;
    }
    if (i < end) {
        unsigned bits = 0;

        b->has_s2 = true;
        for (;;) {
            uint8_t o;

            if (i >= end) {
                b->anomaly = true;
                return 0;
            }
            o = r->p[i++];
            if (b->s2n < P2_S2)
                b->s2[b->s2n] = o & 0x3f;
            else
                b->extra = true;
            b->s2n++;
            for (int k = 0; k < 6; k++)
                bits += (o >> k) & 1u;
            if (o & 0x40)
                break;
        }
        for (unsigned blk = 0; blk < bits; blk++) {
            unsigned no = 0;

            for (;;) {
                uint8_t o;

                if (i >= end) {                           /* fewer NPar(3) blocks than SPar(2) bits */
                    b->anomaly = true;
                    return 0;
                }
                o = r->p[i++];
                if (blk < P2_N3 && no < P2_N3O)
                    b->n3[blk][no] = o & 0x3f;
                else
                    b->extra = true;
                no++;
                if (o & 0x40)
                    break;
            }
            if (blk < P2_N3)
                b->n3_blocks = blk + 1;
        }
        if (i < end)
            b->anomaly = true;                            /* octets nobody accounts for */
    }
    return 0;
}

static void flag_par2(v8bis_msg_t *m, const par2_t *b)
{
    if (b->anomaly)
        m->delimiter_anomaly = true;
    if (b->extra)
        m->extra_octets = true;
}

static void check_mask(v8bis_msg_t *m, unsigned v, unsigned mask)
{
    if (v & ~mask)
        m->reserved_bits = true;
}

static int decode_fields(rd_t *r, v8bis_msg_t *m)
{
    uint8_t first, rest;
    unsigned n;
    int rc;
    par2_t b;

    /* identification field */
    if ((rc = l1_read(r, &first, &n, &rest)))
        return rc;
    m->id_npar1 = first;
    check_mask(m, first, M_ID_NPAR1);
    if (n > 1) {
        m->extra_octets = true;
        if (rest)
            m->reserved_bits = true;
    }
    if ((rc = l1_read(r, &first, &n, &rest)))
        return rc;
    {
        /* SPar(1) bits across all its octets, one Par(2) block each, in order */
        unsigned start = (unsigned)(r->pos - n);
        unsigned bit_index = 0;

        for (unsigned oi = 0; oi < n; oi++)
            for (int k = 0; k < 7; k++, bit_index++) {
                uint8_t o = r->p[start + oi];

                if (!((o >> k) & 1u))
                    continue;
                if ((rc = par2_read(r, &b)))
                    return rc;
                flag_par2(m, &b);
                if (bit_index == 0) {
                    m->network_type = true;
                    m->network_npar2 = b.n2[0];
                    check_mask(m, b.n2[0], M_NET);
                    if (b.n2n > 1 || b.has_s2)
                        m->extra_octets = true;
                } else {
                    m->ignored_blocks++;
                    m->reserved_bits = true;
                }
            }
    }

    /* standard field */
    if ((rc = l1_read(r, &first, &n, &rest)))
        return rc;
    m->s_npar1 = first;
    check_mask(m, first, M_S_NPAR1);
    if (n > 1) {
        m->extra_octets = true;
        if (rest)
            m->reserved_bits = true;
    }
    if ((rc = l1_read(r, &first, &n, &rest)))
        return rc;
    {
        unsigned start = (unsigned)(r->pos - n);
        unsigned bit_index = 0;

        m->s_spar1 = first & M_S_SPAR1;
        check_mask(m, first, M_S_SPAR1);
        for (unsigned oi = 0; oi < n; oi++)
            for (int k = 0; k < 7; k++, bit_index++) {
                uint8_t o = r->p[start + oi];
                unsigned bitv = 1u << k;

                if (!((o >> k) & 1u))
                    continue;
                if ((rc = par2_read(r, &b)))
                    return rc;
                flag_par2(m, &b);
                if (oi != 0 || !(bitv & M_S_SPAR1)) {
                    m->ignored_blocks++;
                    m->reserved_bits = true;
                    continue;
                }
                switch (bitv) {
                case V8BIS_S_DATA:
                    for (unsigned q = 0; q < 3 && q < b.n2n && q < P2_N2; q++)
                        m->data[q] = b.n2[q];
                    if (b.n2n > 3 || b.has_s2)
                        m->extra_octets = true;
                    check_mask(m, m->data[0], M_DATA0);
                    check_mask(m, m->data[1], M_DATA1);
                    check_mask(m, m->data[2], M_DATA2);
                    break;
                case V8BIS_S_SVD:
                    for (unsigned q = 0; q < 3 && q < b.n2n && q < P2_N2; q++)
                        m->svd[q] = b.n2[q];
                    if (b.n2n > 3 || b.has_s2)
                        m->extra_octets = true;
                    check_mask(m, m->svd[0], M_SVD0);
                    check_mask(m, m->svd[1], M_SVD1);
                    check_mask(m, m->svd[2], M_SVD2);
                    break;
                case V8BIS_S_H324:
                    m->h324_npar2 = b.n2[0];
                    check_mask(m, b.n2[0], M_H324);
                    if (b.n2n > 1)
                        m->extra_octets = true;
                    if (b.has_s2) {
                        m->h324_spar2 = b.s2[0] & V8BIS_H324_SPAR2_DATA;
                        check_mask(m, b.s2[0], V8BIS_H324_SPAR2_DATA);
                        if (b.s2n > 1)
                            m->extra_octets = true;
                        if ((b.s2[0] & V8BIS_H324_SPAR2_DATA) && b.n3_blocks) {
                            m->h324_data = b.n3[0][0];
                            check_mask(m, b.n3[0][0], M_H324D);
                        }
                    }
                    break;
                case V8BIS_S_V18:
                    m->v18 = b.n2[0];
                    check_mask(m, b.n2[0], M_V18);
                    if (b.n2n > 1 || b.has_s2)
                        m->extra_octets = true;
                    break;
                case V8BIS_S_ANALOGUE_TEL:
                    m->analogue_tel = b.n2[0];
                    check_mask(m, b.n2[0], M_TEL);
                    if (b.n2n > 1 || b.has_s2)
                        m->extra_octets = true;
                    break;
                case V8BIS_S_T101:
                    m->t101 = b.n2[0];
                    check_mask(m, b.n2[0], M_T101);
                    if (b.n2n > 1 || b.has_s2)
                        m->extra_octets = true;
                    break;
                }
            }
    }

    /* non-standard field */
    if (m->id_npar1 & V8BIS_ID_NON_STANDARD) {
        while (r->pos < r->len) {
            unsigned l = r->p[r->pos++], plen, mlen;
            const uint8_t *q;

            if (l < 2 || r->pos + l > r->len)
                return V8BIS_IE_BAD_NS;
            q = r->p + r->pos;
            plen = q[1];
            if (2 + plen > l)
                return V8BIS_IE_BAD_NS;
            mlen = l - 2 - plen;
            if (m->ns_count < V8BIS_NS_BLOCKS_MAX && plen <= V8BIS_NS_PROVIDER_MAX
                && mlen <= V8BIS_NS_DATA_MAX) {
                v8bis_ns_block_t *nb = &m->ns[m->ns_count++];

                nb->country = q[0];
                nb->provider_len = (uint8_t)plen;
                memcpy(nb->provider, q + 2, plen);
                nb->data_len = (uint8_t)mlen;
                memcpy(nb->data, q + 2 + plen, mlen);
            } else {
                m->ns_dropped++;
            }
            r->pos += l;
        }
    } else if (r->pos < r->len) {
        m->extra_octets = true;                           /* trailing octets: ignored */
    }
    return 0;
}

int v8bis_msg_decode(const uint8_t *in, size_t len, v8bis_msg_t *m)
{
    rd_t r;
    int rc = 0;

    if (!in || !m)
        return V8BIS_IE_BAD_ARG;
    memset(m, 0, sizeof(*m));
    if (len == 0)
        return V8BIS_IE_EMPTY;
    if (len > V8BIS_MAX_INFO_OCTETS)
        return V8BIS_IE_TOO_LONG;
    m->type = in[0] & 0x0f;
    m->revision = in[0] >> 4;
    r.p = in;
    r.len = len;
    r.pos = 1;
    if (type_has_fields(m->type)) {
        m->known_type = true;
        rc = decode_fields(&r, m);
    } else {
        m->known_type = type_is_ack_nak(m->type);
        m->payload_len = (unsigned)(len - 1);
        memcpy(m->payload, in + 1, len - 1);
        if (m->known_type && len > 1)
            m->extra_octets = true;                       /* 8.3.3: ACK/NAK carry no parameters */
    }
    return rc;
}
