/*
 * mnp.c — the V.42 Annex A alternative error-correcting procedure (MNP
 * classes 2-4).  Clause numbers are V.42 (10/96) Annex A, the last edition to
 * carry it (`ITU Docs/T-REC-V.42-199610-S`; the 1988 and 1993 editions have
 * the same clauses).  See mnp.h for scope, docs/mnp.md for the wiring.
 *
 * Frame = header (A.6.2: LI, type, fixed then variable parameters) +
 * information (LT only) + FCS, framed either
 *   start-stop octet-oriented (A.3): SYN DLE STX, body with DLE doubled,
 *     DLE ETX, CRC-16 x^16+x^15+x^2+1 of body + ETX (A.3.6); or
 *   bit-oriented (A.4): flag, body, V.42's own 16-bit FCS (8.1.1.6.1), flag,
 *     zero insertion (8.1.1.2).
 */

#include "mnp.h"

#include <stdlib.h>
#include <string.h>

/* Table A.1 */
#define MNP_LR 1
#define MNP_LD 2
#define MNP_LT 4
#define MNP_LA 5
#define MNP_LN 6
#define MNP_LNA 7

#define SYN 0x16
#define DLE 0x10
#define STX 0x02
#define ETX 0x03

#define MNP_K_MAX 8                 /* A.7.5.7 NOTE: support k = 8 */
#define MNP_N401_BASE 64            /* A.7.5.6 NOTE: support N401 = 64 */
#define MNP_DATA_MAX 256
#define MNP_HDR_MAX 32
#define MNP_RX_MAX (MNP_HDR_MAX + MNP_DATA_MAX + 8)
/* Octet-oriented worst case: every octet a DLE, 10 line bits each. */
#define MNP_TX_BITS_MAX ((3 + 2*(MNP_HDR_MAX + MNP_DATA_MAX) + 4)*10 + 16)

/* A.6.4.1.8 data phase optimisation bits. */
#define OPT_256 0x01        /* bit 1: information field up to 256 octets */
#define OPT_FIXED 0x02      /* bit 2: fixed-field LT and LA */

#define MNP_N400 12                 /* A.7.5.5 */
/* Adaptive packet assembly floor: an LT never shrinks below this. */
#define MNP_APA_MIN 32
/* A.7.5.1: T401 in the establishment phase is 0.5-9 s. */
#define MNP_T401_ESTABLISH_MS 1500
/* A.7.1.3: the responder resends its LR, then rejects. */
#define MNP_RESPONSE_LR_TRIES 3

enum {
    ST_DOWN,
    ST_ORIG_LR,         /* initiator: LR sent, waiting for the response LR */
    ST_ANS_WAIT,        /* responder: waiting for an LR */
    ST_ANS_LR_RESP,     /* responder: response LR sent, waiting for LA */
    ST_DATA
};

enum { A_HUNT, A_SYN, A_DLE1, A_DATA, A_DATA_DLE, A_CRC1, A_CRC2 };

typedef struct {
    uint8_t data[MNP_DATA_MAX];
    int len;
    uint64_t sent_at;
    int sends;
} mnp_out_t;

struct mnp_s {
    bool originator;
    int offer_class;
    int bit_rate;
    mnp_get_data_fn get;
    mnp_put_data_fn put;
    mnp_status_fn status;
    void *user;

    int state;
    uint64_t now;                   /* transmit bit clock */

    /* Transmitter: the frame being shifted out, one bit per entry. */
    uint8_t txb[MNP_TX_BITS_MAX];
    int tx_len, tx_pos;
    bool tx_sync;                   /* data-phase framing is bit-oriented */
    int flag_pos;                   /* idle flags, bit index */
    bool last_frame_sync;           /* the last frame sent was bit-oriented */
    bool switch_after_frame;        /* initiator: confirming LA, then mode 3 */
    bool report_down_after_frame;
    int down_status;

    /* Establishment. */
    int lr_tries, lr_max_tries;
    uint64_t lr_next;               /* T401 (establishment) */
    uint64_t ans_deadline;          /* 0 = none */
    bool lr_resp_due;
    bool confirm_due;               /* initiator: (re)send the confirming LA */
    bool lr_seen_reported;
    uint64_t lr_sent_at;
    bool peer_sent_opt;             /* the initiating LR carried parameter 8 */

    /* Negotiated (A.7.1.5). */
    int svc, k, n401, opt;

    /* Sender (A.6.3.2, A.6.3.12). */
    uint8_t vs;                     /* V(S) */
    uint8_t acked;                  /* N(R) of the last LA */
    int credit;                     /* N(k) of the last LA */
    mnp_out_t out[16];
    bool retx_active;
    uint8_t retx_ns;
    int retries;
    uint64_t timer_at;
    bool have_rtt;
    double srtt, rttvar;            /* bits */
    double min_rtt;
    uint64_t goback_at;
    int lt_size;                    /* adaptive packet assembly, <= N401 */
    int clean_run;

    /* Receiver (A.6.3.4).  vr is V(R) - 1, the N(R) an LA carries. */
    uint8_t vr;
    int unacked_rx;
    uint64_t first_unacked_at;
    bool ack_due;
    bool dup_seen;
    bool busy;
    uint64_t last_la_at;

    bool lna_due;
    uint8_t lna_nr;

    /* Bit-oriented receiver (HDLC deframing). */
    int s_ones;
    uint8_t s_byte;
    int s_total;
    int s_len;
    bool s_in_frame;
    uint8_t s_buf[MNP_RX_MAX];

    /* Octet-oriented receiver. */
    bool a_hunting;
    int a_bits;
    uint8_t a_shift;
    int a_state;
    int a_len;
    bool a_overflow;
    uint8_t a_crc_lo;
    uint8_t a_buf[MNP_RX_MAX];

    mnp_stats_t st;
};

/* A.3.6: x^16 + x^15 + x^2 + 1, reflected, preset 0. */
uint16_t mnp_crc16(uint16_t crc, const uint8_t *buf, int len)
{
    for (int i = 0; i < len; i++)
    {
        crc ^= buf[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 1)  ?  (uint16_t)((crc >> 1) ^ 0xA001)  :  (uint16_t)(crc >> 1);
    }
    return crc;
}

/* 8.1.1.6.1: x^16 + x^12 + x^5 + 1, preset to ones; the ones complement is
   sent, low-order octet first. */
uint16_t mnp_fcs16(uint16_t crc, const uint8_t *buf, int len)
{
    for (int i = 0; i < len; i++)
    {
        crc ^= buf[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 1)  ?  (uint16_t)((crc >> 1) ^ 0x8408)  :  (uint16_t)(crc >> 1);
    }
    return crc;
}

static void report(mnp_t *m, int status)
{
    if (m->status)
        m->status(m->user, status);
}

/* ---- timers --------------------------------------------------------------- */

static uint64_t ms_bits(const mnp_t *m, int ms)
{
    return (uint64_t)m->bit_rate*(uint64_t)ms/1000u;
}

/* Line bits for one maximum LT in A.7.5.1's terms: Lb bits an octet (10
   octet-oriented; 8 bit-oriented, taken as 10 to cover zero insertion), Lf
   framing octets (7 or 4), Llt header octets (5, or 3 optimised). */
static uint64_t lt_bits(const mnp_t *m)
{
    int lf = m->tx_sync  ?  4  :  7;
    int llt = (m->opt & OPT_FIXED)  ?  3  :  5;

    return 10u*(uint64_t)(lf + llt + m->n401);
}

static uint64_t trt(const mnp_t *m)
{
    if (!m->have_rtt)
        return ms_bits(m, 1000);
    return (uint64_t)(m->srtt + 4.0*m->rttvar);
}

/* A.7.5.1: T401 >= {2(k/2) Lb(Lf+Llt+N401) + Lb(Lf+Lla)}/(bit/s) + Trt. */
static uint64_t t401(const mnp_t *m)
{
    int half = (m->k > 1)  ?  m->k/2  :  1;
    uint64_t t = 2u*(uint64_t)half*lt_bits(m) + 10u*(uint64_t)((m->tx_sync ? 4 : 7) + 8) + trt(m);

    return (t < ms_bits(m, 500))  ?  ms_bits(m, 500)  :  t;
}

/* A.7.5.2: T402 = 0.5 T401. */
static uint64_t t402(const mnp_t *m)
{
    return t401(m)/2;
}

/* A.7.5.4, Table A.10. */
static uint64_t t404(const mnp_t *m)
{
    return ms_bits(m, (m->bit_rate <= 1200)  ?  7000  :  3000);
}

static void rtt_sample(mnp_t *m, double r)
{
    if (!m->have_rtt  ||  r < m->min_rtt)
        m->min_rtt = r;
    if (!m->have_rtt)
    {
        m->srtt = r;
        m->rttvar = r/2.0;
        m->have_rtt = true;
        return;
    }
    m->rttvar = 0.75*m->rttvar + 0.25*((r > m->srtt) ? r - m->srtt : m->srtt - r);
    m->srtt = 0.875*m->srtt + 0.125*r;
}

/* ---- transmitter ---------------------------------------------------------- */

static void put_bit(mnp_t *m, int bit)
{
    if (m->tx_len < MNP_TX_BITS_MAX)
        m->txb[m->tx_len++] = (uint8_t)bit;
}

/* A.3: each octet with a start and a stop bit, bit 1 first (8.1.2.2). */
static void put_async_octet(mnp_t *m, uint8_t c)
{
    put_bit(m, 0);
    for (int b = 0; b < 8; b++)
        put_bit(m, (c >> b) & 1);
    put_bit(m, 1);
}

static void put_flag(mnp_t *m)
{
    for (int b = 0; b < 8; b++)
        put_bit(m, (0x7E >> b) & 1);
}

static void put_sync_octet(mnp_t *m, uint8_t c, int *ones)
{
    for (int b = 0; b < 8; b++)
    {
        int bit = (c >> b) & 1;

        put_bit(m, bit);
        if (bit)
        {
            if (++*ones == 5)
            {
                put_bit(m, 0);
                *ones = 0;
            }
        }
        else
            *ones = 0;
    }
}

static void build_frame(mnp_t *m, bool sync, const uint8_t *hdr, int hlen,
                        const uint8_t *data, int dlen)
{
    m->tx_len = m->tx_pos = 0;
    if (sync)
    {
        int ones = 0;
        uint16_t fcs = (uint16_t)~mnp_fcs16(mnp_fcs16(0xFFFF, hdr, hlen), data, dlen);

        put_flag(m);
        for (int i = 0; i < hlen; i++)
            put_sync_octet(m, hdr[i], &ones);
        for (int i = 0; i < dlen; i++)
            put_sync_octet(m, data[i], &ones);
        put_sync_octet(m, (uint8_t)fcs, &ones);
        put_sync_octet(m, (uint8_t)(fcs >> 8), &ones);
        put_flag(m);
        m->last_frame_sync = true;
        return;
    }
    {
        static const uint8_t etx = ETX;
        uint16_t crc = mnp_crc16(mnp_crc16(mnp_crc16(0, hdr, hlen), data, dlen), &etx, 1);

        /* After bit-oriented frames the far start-stop receiver is somewhere
           inside a "character" of flags and data; two characters of mark put
           it back on a start bit before SYN (the confirming LA, or an LR,
           sent from the data phase). */
        if (m->last_frame_sync)
            for (int i = 0; i < 20; i++)
                put_bit(m, 1);
        m->last_frame_sync = false;

        put_async_octet(m, SYN);
        put_async_octet(m, DLE);
        put_async_octet(m, STX);
        for (int i = 0; i < hlen + dlen; i++)
        {
            uint8_t c = (i < hlen)  ?  hdr[i]  :  data[i - hlen];

            put_async_octet(m, c);
            if (c == DLE)
                put_async_octet(m, DLE);        /* A.3.4 */
        }
        put_async_octet(m, DLE);
        put_async_octet(m, ETX);
        put_async_octet(m, (uint8_t)crc);
        put_async_octet(m, (uint8_t)(crc >> 8));
    }
}

/* A.6.4.1, Table A.2.  N401 is two octets; the field-mapping convention
   (8.1.2.3) would put the high-order value first, but MNP DCEs send 64 as
   (64, 0), so it goes low octet first and parse_lr() reads either. */
static void send_lr(mnp_t *m, bool response)
{
    uint8_t h[MNP_HDR_MAX];
    int n = 0;
    int n401 = response  ?  ((m->n401 > MNP_N401_BASE) ? MNP_N401_BASE : m->n401)  :  MNP_N401_BASE;

    h[n++] = 0;                 /* LI, filled in below */
    h[n++] = MNP_LR;
    h[n++] = 0x02;              /* constant parameter 1 (A.6.4.1.3) */
    h[n++] = 1; h[n++] = 6;     /* constant parameter 2 (A.6.4.1.4) */
    h[n++] = 1; h[n++] = 0; h[n++] = 0; h[n++] = 0; h[n++] = 0; h[n++] = 255;
    h[n++] = 2; h[n++] = 1; h[n++] = (uint8_t)(response ? m->svc : m->offer_class);
    h[n++] = 3; h[n++] = 1; h[n++] = (uint8_t)(response ? m->k : MNP_K_MAX);
    h[n++] = 4; h[n++] = 2; h[n++] = (uint8_t)n401; h[n++] = (uint8_t)(n401 >> 8);
    /* A.6.4.1.8 / A.7.1.5.6: the response carries parameter 8 only if the
       initiator sent it and some facility is agreed. */
    if (!response  ||  (m->peer_sent_opt  &&  m->opt))
    {
        h[n++] = 8; h[n++] = 1; h[n++] = (uint8_t)(response ? m->opt : (OPT_256 | OPT_FIXED));
    }
    h[0] = (uint8_t)(n - 1);
    /* A.6.4: the establishment exchange is always start-stop octet-oriented. */
    build_frame(m, false, h, n, NULL, 0);
    m->st.lr_tx++;
}

/* Tables A.3a/A.3b. */
static void send_la(mnp_t *m, bool async)
{
    uint8_t h[8];
    int n = 0;
    uint8_t credit = (uint8_t)(m->busy  ?  0  :  m->k);

    if (m->opt & OPT_FIXED)
    {
        h[n++] = 3; h[n++] = MNP_LA; h[n++] = m->vr; h[n++] = credit;
    }
    else
    {
        h[n++] = 7; h[n++] = MNP_LA;
        h[n++] = 1; h[n++] = 1; h[n++] = m->vr;
        h[n++] = 2; h[n++] = 1; h[n++] = credit;
    }
    build_frame(m, async  ?  false  :  m->tx_sync, h, n, NULL, 0);
    m->ack_due = false;
    m->unacked_rx = 0;
    m->last_la_at = m->now;
    m->st.la_tx++;
}

/* Tables A.6a/A.6b. */
static void send_lt(mnp_t *m, uint8_t ns)
{
    uint8_t h[8];
    int n = 0;
    mnp_out_t *o = &m->out[ns & 15];

    if (m->opt & OPT_FIXED)
    {
        h[n++] = 2; h[n++] = MNP_LT; h[n++] = ns;
    }
    else
    {
        h[n++] = 4; h[n++] = MNP_LT; h[n++] = 1; h[n++] = 1; h[n++] = ns;
    }
    build_frame(m, m->tx_sync, h, n, o->data, o->len);
    if (o->sends++ > 0)
        m->st.retransmitted++;
    o->sent_at = m->now;        /* A.7.3.1: k > 1 times from the frame's start */
    m->st.lt_tx++;
}

/* Table A.4. */
static void send_ld(mnp_t *m, uint8_t reason, int status)
{
    uint8_t h[5] = { 4, MNP_LD, 1, 1, reason };

    build_frame(m, m->tx_sync, h, 5, NULL, 0);
    m->report_down_after_frame = true;
    m->down_status = status;
}

/* Table A.8; A.7.4.5: N(RA) is the received N(SA). */
static void send_lna(mnp_t *m)
{
    uint8_t h[5] = { 4, MNP_LNA, 1, 1, m->lna_nr };

    build_frame(m, m->tx_sync, h, 5, NULL, 0);
    m->lna_due = false;
}

static int outstanding(const mnp_t *m)
{
    return (uint8_t)(m->vs - 1 - m->acked);
}

/* A.6.3.12: frames may run to N(R) + N(k), never more than k. */
static int window(const mnp_t *m)
{
    return (m->credit < m->k)  ?  m->credit  :  m->k;
}

/* The longest LT adaptive packet assembly builds: about 300 ms of line.
   A retransmitted LT keeps its length (the peer may already hold it under
   that N(S)), so a long frame that keeps being hit cannot be made shorter
   later; on a slow line it must not be built at all.  2400 bit/s gets 72
   octets, and from 9600 the whole N401. */
static int apa_ceiling(const mnp_t *m)
{
    int c = m->bit_rate*3/100;

    if (c < MNP_APA_MIN)
        c = MNP_APA_MIN;
    return (c > m->n401)  ?  m->n401  :  c;
}

/* Shorter LTs only help while k of them still cover a round trip: below
   that the window, not the errors, sets the rate. */
static int apa_floor(const mnp_t *m)
{
    int floor = MNP_APA_MIN;

    if (m->have_rtt)
    {
        int bdp = (int)(m->min_rtt/(8.0*m->k));

        if (bdp > floor)
            floor = bdp;
    }
    return (floor > apa_ceiling(m))  ?  apa_ceiling(m)  :  floor;
}

/* A.7.3.5: retransmission starts with the first unacknowledged LT. */
static void go_back(mnp_t *m)
{
    if (outstanding(m) == 0)
        return;
    m->retx_active = true;
    m->retx_ns = (uint8_t)(m->acked + 1);
    m->goback_at = m->now;
    /* Adaptive packet assembly (the MNP class 4 facility; A.7.3.1 only asks
       that an LT carry no more than N401): every go-back halves the LTs to
       come, so a line whose errors land in most long frames is not paying a
       whole window for each one. */
    m->lt_size /= 2;
    if (m->lt_size < apa_floor(m))
        m->lt_size = apa_floor(m);
    m->clean_run = 0;
}

/* T401 runs from the oldest LT's last transmission, or from its last expiry
   when a closed window kept the go-back from sending anything. */
static uint64_t oldest_timer_start(const mnp_t *m)
{
    uint64_t t = m->out[(uint8_t)(m->acked + 1) & 15].sent_at;

    return (m->timer_at > t)  ?  m->timer_at  :  t;
}

static void enter_down(mnp_t *m, int status)
{
    m->state = ST_DOWN;
    m->tx_sync = false;
    report(m, status);
}

static bool try_lt(mnp_t *m)
{
    if (m->retx_active)
    {
        uint8_t ns = m->retx_ns;

        if (ns == m->vs  ||  (uint8_t)(ns - 1 - m->acked) >= (uint8_t)outstanding(m))
            m->retx_active = false;
        else if ((uint8_t)(ns - m->acked) <= window(m))
        {
            send_lt(m, ns);
            m->retx_ns = (uint8_t)(ns + 1);
            if (m->retx_ns == m->vs)
                m->retx_active = false;
            return true;
        }
        else
            return false;       /* A.7.3.5: no retransmission while S(k) = 0 */
    }
    if ((uint8_t)(m->vs - m->acked) <= window(m)  &&  m->get)
    {
        mnp_out_t *o = &m->out[m->vs & 15];
        int len = m->get(m->user, o->data, m->lt_size);

        if (len > 0)            /* A.6.6.1: no null information field */
        {
            o->len = len;
            o->sends = 0;
            send_lt(m, m->vs);
            m->vs++;
            return true;
        }
    }
    return false;
}

/* Choose and build the next frame; false leaves the line idle. */
static bool next_frame(mnp_t *m)
{
    switch (m->state)
    {
    case ST_ORIG_LR:
        /* A.7.1.2: resend the LR when T401 expires. */
        if (m->now < m->lr_next)
            return false;
        if (m->lr_tries >= m->lr_max_tries)
        {
            /* A.7.2.2: no reply at all, so no LD; the caller falls back. */
            enter_down(m, MNP_STATUS_ESTABLISH_FAILED);
            return false;
        }
        send_lr(m, false);
        m->lr_tries++;
        m->lr_sent_at = m->now;
        m->lr_next = m->now + (uint64_t)m->tx_len + ms_bits(m, MNP_T401_ESTABLISH_MS);
        return true;
    case ST_ANS_WAIT:
        if (m->ans_deadline  &&  m->now >= m->ans_deadline)
            enter_down(m, MNP_STATUS_ESTABLISH_FAILED);
        return false;
    case ST_ANS_LR_RESP:
        /* A.7.1.3: resend the response LR on T401, a bad FCS or another LR. */
        if (!m->lr_resp_due  &&  m->now >= m->lr_next)
        {
            if (m->lr_tries >= MNP_RESPONSE_LR_TRIES)
            {
                enter_down(m, MNP_STATUS_ESTABLISH_FAILED);
                return false;
            }
            m->lr_resp_due = true;
        }
        if (!m->lr_resp_due)
            return false;
        m->lr_resp_due = false;
        send_lr(m, true);
        m->lr_tries++;
        m->lr_next = m->now + (uint64_t)m->tx_len + ms_bits(m, MNP_T401_ESTABLISH_MS);
        return true;
    case ST_DATA:
        break;
    default:
        return false;
    }

    if (m->confirm_due)
    {
        /* A.6.4.2: the confirming LA ends the octet-oriented exchange; the
           negotiated framing starts after it. */
        m->confirm_due = false;
        send_la(m, true);
        m->switch_after_frame = (m->svc == 3);
        return true;
    }
    if (m->lna_due)
    {
        send_lna(m);
        return true;
    }
    /* A.7.3.3.2 a)-c) (and a busy change, A.6.6.2.2), and f): k/2 frames. */
    if (m->ack_due  ||  (m->unacked_rx > 0  &&  m->unacked_rx >= ((m->k > 1) ? m->k/2 : 1)))
    {
        send_la(m, false);
        return true;
    }
    /* g): T402. */
    if (m->unacked_rx > 0  &&  m->now - m->first_unacked_at >= t402(m))
    {
        send_la(m, false);
        return true;
    }
    /* A.7.3.5 b): T401. */
    if (outstanding(m) > 0  &&  m->now - oldest_timer_start(m) >= t401(m))
    {
        m->timer_at = m->now;
        /* A.7.3.6 / A.7.2.4: N400 attempts, then LD. */
        if (++m->retries >= MNP_N400)
        {
            send_ld(m, 255, MNP_STATUS_DISCONNECTED);
            return true;
        }
        m->st.timeouts++;
        go_back(m);
    }
    if (try_lt(m))
        return true;
    /* e): acknowledgements owed and nothing to send; d): T404. */
    if (m->unacked_rx > 0  ||  m->now - m->last_la_at >= t404(m))
    {
        send_la(m, false);
        return true;
    }
    return false;
}

int mnp_tx_bit(mnp_t *m)
{
    int bit;

    m->now++;
    if (m->tx_pos >= m->tx_len)
    {
        m->tx_len = m->tx_pos = 0;
        if (m->report_down_after_frame)
        {
            m->report_down_after_frame = false;
            enter_down(m, m->down_status);
        }
        if (m->switch_after_frame)
        {
            m->switch_after_frame = false;
            m->tx_sync = true;
            m->flag_pos = 0;
        }
        /* A frame starts on an idle boundary: whole flags when bit-oriented. */
        if ((!m->tx_sync  ||  m->flag_pos == 0)  &&  m->state != ST_DOWN)
        {
            if (!next_frame(m))
                m->tx_len = m->tx_pos = 0;
        }
    }
    if (m->tx_pos < m->tx_len)
        return m->txb[m->tx_pos++];
    if (m->tx_sync  &&  m->state != ST_DOWN)
    {
        bit = (0x7E >> m->flag_pos) & 1;
        m->flag_pos = (m->flag_pos + 1) & 7;
        return bit;
    }
    return 1;
}

/* ---- receiver ------------------------------------------------------------- */

typedef struct {
    int svc, k, n401, opt;
    bool has_opt;
} lr_params_t;

/* A.7.1.5.  False: negotiation fails, reason in *ld (Table A.5). */
static bool parse_lr(const uint8_t *f, int li, lr_params_t *p, int *ld)
{
    int i = 3;

    *ld = 3;
    if (li < 2)
        return false;
    if (f[2] != 0x02)
    {
        *ld = 2;                /* A.7.1.5.1 */
        return false;
    }
    memset(p, 0, sizeof(*p));
    p->svc = -1;
    p->k = -1;
    p->n401 = -1;
    while (i + 1 <= li)
    {
        int type = f[i];
        int len = f[i + 1];
        const uint8_t *v = &f[i + 2];

        if (i + 2 + len > li + 1)
            return false;
        if (type == 2  &&  len >= 1)
            p->svc = v[0];
        else if (type == 3  &&  len >= 1)
            p->k = v[0];
        else if (type == 4  &&  len == 2)
        {
            int le = v[0] | (v[1] << 8);
            int be = (v[0] << 8) | v[1];

            p->n401 = (le >= 1  &&  le <= MNP_DATA_MAX)  ?  le  :  be;
        }
        else if (type == 4  &&  len == 1)
            p->n401 = v[0];
        else if (type == 8  &&  len >= 1)
        {
            p->opt = v[0];
            p->has_opt = true;
        }
        /* A.7.1.5.6: anything else is ignored. */
        i += 2 + len;
    }
    /* Mandatory parameters (Table A.2).  A missing framing mode is read as
       mode 2, which every implementation supports. */
    if (p->svc < 0)
        p->svc = 2;
    if (p->k < 1  ||  p->n401 < 1)
        return false;
    return true;
}

static bool negotiate(mnp_t *m, const lr_params_t *p)
{
    int svc = (p->svc < m->offer_class)  ?  p->svc  :  m->offer_class;     /* A.7.1.5.3 */

    if (svc < 2)
        return false;           /* mode 1 is not a two-way mode */
    m->svc = svc;
    m->k = (p->k > MNP_K_MAX)  ?  MNP_K_MAX  :  p->k;                       /* A.7.1.5.4 */
    m->peer_sent_opt = p->has_opt;
    m->opt = p->opt & (OPT_256 | OPT_FIXED);
    if (m->opt & OPT_256)
        m->n401 = MNP_DATA_MAX;
    else
        m->n401 = (p->n401 > MNP_N401_BASE)  ?  MNP_N401_BASE  :  p->n401;  /* A.7.1.5.5 */
    m->st.service_class = m->svc;
    m->st.k = m->k;
    m->st.n401 = m->n401;
    m->st.optimisation = m->opt;
    return true;
}

/* Data phase initialisation: V(S) = V(R) = 1 (A.6.3.2, A.6.3.4), S(k) = k
   (A.6.3.12). */
static void reset_sequence(mnp_t *m)
{
    m->vs = 1;
    m->acked = 0;
    m->vr = 0;
    m->credit = m->k;
    m->retx_active = false;
    m->retries = 0;
    m->timer_at = 0;
    m->goback_at = 0;
    m->lt_size = apa_ceiling(m);
    m->clean_run = 0;
    m->unacked_rx = 0;
    m->ack_due = false;
    m->dup_seen = false;
    m->last_la_at = m->now;
    memset(m->out, 0, sizeof(m->out));
}

static void enter_data(mnp_t *m)
{
    m->state = ST_DATA;
    report(m, MNP_STATUS_CONNECTED);
}

/* A.7.3.4 / A.7.3.5. */
static void rx_la(mnp_t *m, uint8_t nr, int credit)
{
    int out = outstanding(m);
    int adv = (uint8_t)(nr - m->acked);

    m->st.la_rx++;
    if (adv > out)
        return;                 /* N(R) outside what is outstanding */
    for (int i = 1; i <= adv; i++)
    {
        mnp_out_t *o = &m->out[(uint8_t)(m->acked + i) & 15];

        /* Karn: only frames sent once give an unambiguous round trip. */
        if (o->sends == 1)
        {
            rtt_sample(m, (double)(m->now - o->sent_at));
            /* k clean frames grow the LT size back by a quarter of N401. */
            if (++m->clean_run >= m->k  &&  m->lt_size < apa_ceiling(m))
            {
                m->lt_size += (m->n401 + 3)/4;
                if (m->lt_size > apa_ceiling(m))
                    m->lt_size = apa_ceiling(m);
                m->clean_run = 0;
            }
        }
    }
    m->credit = credit;
    if (adv > 0)
    {
        m->acked = nr;
        m->retries = 0;
        m->timer_at = 0;
        /* A go-back in progress skips what this LA acknowledges. */
        if (m->retx_active  &&  (uint8_t)(m->retx_ns - 1 - m->acked) >= (uint8_t)outstanding(m))
        {
            if (m->retx_ns == m->vs  ||  outstanding(m) == 0)
                m->retx_active = false;
            else
                m->retx_ns = (uint8_t)(m->acked + 1);
        }
        return;
    }
    /* a): an LA repeating the last N(R) asks for retransmission -- but not
       one that left the far end before our last go-back, or the oldest
       unacknowledged LT, could have reached it (less than the shortest round
       trip seen since).  The receiver sends LAs for every frame of a gap, and
       those already in flight would each start the retransmission over; and
       an idle peer's T404 LA (A.7.3.3.2 d)) repeats N(R) by construction, so
       one that crosses a fresh LT would resend it.  The RasFinder's did, on
       the first live MNP call. */
    if (out > 0  &&  !m->retx_active)
    {
        uint64_t sent = m->out[(uint8_t)(m->acked + 1) & 15].sent_at;
        bool stale = m->have_rtt
                     &&  ((m->goback_at  &&  (double)(m->now - m->goback_at) < m->min_rtt)
                          ||  (double)(m->now - sent) < m->min_rtt);

        if (!stale)
            go_back(m);
    }
}

/* A.7.3.2. */
static void rx_lt(mnp_t *m, uint8_t ns, const uint8_t *data, int len)
{
    if (ns == (uint8_t)(m->vr + 1))
    {
        m->vr = ns;
            m->dup_seen = false;
        if (m->unacked_rx++ == 0)
            m->first_unacked_at = m->now;
        m->st.lt_rx++;
        if (len > 0  &&  m->put)
            m->put(m->user, data, len);
        return;
    }
    /* A.7.3.2.2: discard and send LA -- except the first repeat of the last
       accepted LT.  Every frame of a gap gets one, as the clause says; our
       sender ignores those that cross its go-back (rx_la()). */
    m->st.out_of_sequence++;
    if (ns == m->vr  &&  !m->dup_seen)
    {
        m->dup_seen = true;
        return;
    }
    m->ack_due = true;
}

static int field(const uint8_t *f, int li, int want_type)
{
    for (int i = 2; i + 1 <= li; )
    {
        int type = f[i];
        int len = f[i + 1];

        if (i + 2 + len > li + 1)
            return -1;
        if (type == want_type  &&  len >= 1)
            return f[i + 2];
        i += 2 + len;
    }
    return -1;
}

static void rx_lr(mnp_t *m, const uint8_t *f, int li)
{
    lr_params_t p;
    int ld;

    if (!parse_lr(f, li, &p, &ld)  ||  !negotiate(m, &p))
    {
        /* A.7.2.2: a failed negotiation is answered with LD. */
        if (m->state == ST_ORIG_LR  ||  m->state == ST_ANS_WAIT  ||  m->state == ST_ANS_LR_RESP)
        {
            m->tx_sync = false;
            send_ld(m, (uint8_t)ld, MNP_STATUS_ESTABLISH_FAILED);
        }
        return;
    }
    m->st.lr_rx++;
    if (m->originator)
    {
        if (m->state == ST_DATA)
        {
            /* The responder resends its LR until our confirming LA arrives
               (A.7.1.3): it did not, so send it again. */
            m->confirm_due = true;
            return;
        }
        if (m->state != ST_ORIG_LR)
            return;
        if (m->lr_sent_at)
            rtt_sample(m, (double)(m->now - m->lr_sent_at));
        reset_sequence(m);
        m->confirm_due = true;
        enter_data(m);
        return;
    }
    if (m->state == ST_DOWN)
        return;
    if (m->state != ST_ANS_LR_RESP)
        m->lr_tries = 0;
    reset_sequence(m);
    m->tx_sync = false;
    m->state = ST_ANS_LR_RESP;
    m->lr_resp_due = true;
    if (!m->lr_seen_reported)
    {
        m->lr_seen_reported = true;
        report(m, MNP_STATUS_LR_SEEN);
    }
}

/* A complete frame failed its FCS. */
static void rx_bad_fcs(mnp_t *m, bool sync)
{
    m->st.fcs_errors++;
    if (m->state == ST_ORIG_LR  &&  !sync)
        m->lr_next = m->now;                /* A.7.1.2 b) */
    else if (m->state == ST_ANS_LR_RESP  &&  !sync)
        m->lr_resp_due = true;              /* A.7.1.3 b) */
    else if (m->state == ST_DATA  &&  sync == m->tx_sync)
        m->ack_due = true;                  /* A.7.3.3.2 a) */
}

static void rx_frame(mnp_t *m, const uint8_t *f, int len, bool sync)
{
    int li;
    int type;

    if (len < 2)
        return;
    li = f[0];
    /* LI 255 (A.6.2.1, extended length) never fits our frames. */
    if (li < 1  ||  li == 255  ||  li + 1 > len)
        return;
    type = f[1];
    /* In the data phase only the negotiated framing counts -- except an LR,
       which is always octet-oriented. */
    if (m->state == ST_DATA  &&  type != MNP_LR  &&  sync != m->tx_sync
        &&  !m->switch_after_frame)
        return;

    if (type == MNP_LR)
    {
        if (!sync)
            rx_lr(m, f, li);
        return;
    }
    if (type == MNP_LD)
    {
        /* A.7.2: terminate. */
        if (m->state != ST_DOWN  &&  m->state != ST_ANS_WAIT)
            enter_down(m, (m->state == ST_DATA)  ?  MNP_STATUS_DISCONNECTED
                                                 :  MNP_STATUS_ESTABLISH_FAILED);
        return;
    }
    if (m->state == ST_ANS_LR_RESP)
    {
        if (type != MNP_LA  ||  sync)
            return;
        if (m->lr_resp_due)
            return;             /* our response LR has not gone yet */
        /* A.6.4.2: the confirming LA.  The negotiated framing starts now. */
        m->tx_sync = (m->svc == 3);
        m->flag_pos = 0;
        enter_data(m);
        return;
    }
    if (m->state != ST_DATA)
        return;
    switch (type)
    {
    case MNP_LT:
    {
        int ns = (li == 2)  ?  f[2]  :  field(f, li, 1);

        if (ns >= 0)
            rx_lt(m, (uint8_t)ns, f + li + 1, len - li - 1);
        break;
    }
    case MNP_LA:
    {
        int nr = (li == 3)  ?  f[2]  :  field(f, li, 1);
        int credit = (li == 3)  ?  f[3]  :  field(f, li, 2);

        if (nr >= 0)
            rx_la(m, (uint8_t)nr, (credit < 0)  ?  m->k  :  credit);
        break;
    }
    case MNP_LN:
    {
        int ns = field(f, li, 1);

        /* A.7.4.3/A.7.4.5: every valid LN gets an LNA.  The break itself is
           not passed on: the DTE side here has no break signal. */
        if (ns >= 0)
        {
            m->lna_nr = (uint8_t)ns;
            m->lna_due = true;
        }
        break;
    }
    default:
        break;
    }
}

static void sync_rx_bit(mnp_t *m, int bit)
{
    if (bit)
    {
        if (++m->s_ones >= 7)
        {
            m->s_in_frame = false;      /* abort / idle mark */
            return;
        }
    }
    else
    {
        if (m->s_ones == 6)
        {
            /* Flag.  The frame's bits are followed by the flag's leading 0
               and six 1s, so a whole number of octets leaves 7 over. */
            if (m->s_in_frame  &&  m->s_total >= 7  &&  ((m->s_total - 7) & 7) == 0)
            {
                int n = (m->s_total - 7) >> 3;

                /* 8.1.3: at least header + FCS, and no longer than ours. */
                if (n >= 4  &&  n <= MNP_RX_MAX)
                {
                    if (mnp_fcs16(0xFFFF, m->s_buf, n) == 0xF0B8)
                        rx_frame(m, m->s_buf, n - 2, true);
                    else
                        rx_bad_fcs(m, true);
                }
            }
            m->s_in_frame = true;
            m->s_total = 0;
            m->s_len = 0;
            m->s_ones = 0;
            return;
        }
        if (m->s_ones == 5)
        {
            m->s_ones = 0;              /* inserted zero */
            return;
        }
        m->s_ones = 0;
    }
    if (!m->s_in_frame)
        return;
    m->s_byte = (uint8_t)((m->s_byte >> 1) | (bit << 7));
    if ((++m->s_total & 7) == 0)
    {
        if (m->s_len < MNP_RX_MAX)
            m->s_buf[m->s_len++] = m->s_byte;
        else
            m->s_in_frame = false;
    }
}

static void async_rx_octet(mnp_t *m, uint8_t c)
{
    switch (m->a_state)
    {
    case A_HUNT:
        if (c == SYN)
            m->a_state = A_SYN;
        break;
    case A_SYN:
        m->a_state = (c == DLE)  ?  A_DLE1  :  (c == SYN)  ?  A_SYN  :  A_HUNT;
        break;
    case A_DLE1:
        if (c == STX)
        {
            m->a_state = A_DATA;
            m->a_len = 0;
            m->a_overflow = false;
        }
        else
            m->a_state = (c == SYN)  ?  A_SYN  :  A_HUNT;
        break;
    case A_DATA:
        if (c == DLE)
            m->a_state = A_DATA_DLE;
        else if (m->a_len < MNP_RX_MAX)
            m->a_buf[m->a_len++] = c;
        else
            m->a_overflow = true;
        break;
    case A_DATA_DLE:
        if (c == DLE)
        {
            if (m->a_len < MNP_RX_MAX)
                m->a_buf[m->a_len++] = c;
            else
                m->a_overflow = true;
            m->a_state = A_DATA;
        }
        else if (c == ETX)
            m->a_state = A_CRC1;
        else if (c == STX)
        {
            m->a_len = 0;       /* DLE STX: a new frame inside a broken one */
            m->a_overflow = false;
            m->a_state = A_DATA;
        }
        else
            m->a_state = (c == SYN)  ?  A_SYN  :  A_HUNT;
        break;
    case A_CRC1:
        m->a_crc_lo = c;
        m->a_state = A_CRC2;
        break;
    case A_CRC2:
    {
        static const uint8_t etx = ETX;
        uint16_t crc = mnp_crc16(mnp_crc16(0, m->a_buf, m->a_len), &etx, 1);

        m->a_state = A_HUNT;
        if (m->a_overflow  ||  m->a_len < 2)
            break;
        if ((uint8_t)crc == m->a_crc_lo  &&  (uint8_t)(crc >> 8) == c)
            rx_frame(m, m->a_buf, m->a_len, false);
        else
            rx_bad_fcs(m, false);
        break;
    }
    }
}

static void async_rx_bit(mnp_t *m, int bit)
{
    if (m->a_hunting)
    {
        if (bit == 0)
        {
            m->a_hunting = false;
            m->a_bits = 0;
            m->a_shift = 0;
        }
        return;
    }
    if (m->a_bits < 8)
    {
        m->a_shift |= (uint8_t)(bit << m->a_bits);
        m->a_bits++;
        return;
    }
    /* Stop bit; a 0 here is taken as the next start bit (V.14 deletion). */
    if (bit == 0)
    {
        uint8_t c = m->a_shift;

        m->a_bits = 0;
        m->a_shift = 0;
        async_rx_octet(m, c);
        return;
    }
    m->a_hunting = true;
    async_rx_octet(m, m->a_shift);
}

void mnp_rx_bit(mnp_t *m, int bit)
{
    bit &= 1;
    sync_rx_bit(m, bit);
    async_rx_bit(m, bit);
}

/* ---- control -------------------------------------------------------------- */

mnp_t *mnp_init(bool originator, int service_class, int bit_rate,
                mnp_get_data_fn get, mnp_put_data_fn put,
                mnp_status_fn status, void *user)
{
    mnp_t *m = calloc(1, sizeof(*m));

    if (!m)
        return NULL;
    m->originator = originator;
    m->offer_class = (service_class == 2)  ?  2  :  3;
    m->bit_rate = (bit_rate > 0)  ?  bit_rate  :  2400;
    m->get = get;
    m->put = put;
    m->status = status;
    m->user = user;
    m->a_hunting = true;
    m->k = MNP_K_MAX;
    m->n401 = MNP_N401_BASE;
    m->lr_max_tries = 3;
    m->state = originator  ?  ST_ORIG_LR  :  ST_ANS_WAIT;
    return m;
}

void mnp_free(mnp_t *m)
{
    free(m);
}

void mnp_set_establish_limit(mnp_t *m, int originator_lr_tries, int answerer_wait_ms)
{
    if (!m)
        return;
    if (originator_lr_tries > 0)
        m->lr_max_tries = originator_lr_tries;
    m->ans_deadline = (answerer_wait_ms > 0)  ?  m->now + ms_bits(m, answerer_wait_ms)  :  0;
}

void mnp_set_bit_rate(mnp_t *m, int bit_rate)
{
    if (!m  ||  bit_rate <= 0  ||  bit_rate == m->bit_rate)
        return;
    /* Round trips measured in bits at the old rate keep their duration. */
    m->srtt *= (double)bit_rate/m->bit_rate;
    m->rttvar *= (double)bit_rate/m->bit_rate;
    m->bit_rate = bit_rate;
}

void mnp_set_local_busy(mnp_t *m, bool busy)
{
    if (!m  ||  m->busy == busy)
        return;
    m->busy = busy;
    /* A.6.6.2.2: credit 0 halts the sender; a non-zero credit resumes it. */
    if (m->state == ST_DATA)
        m->ack_due = true;
}

/* A.7.2.1: "It is recommended that the LD frame not be sent in order to
   promote proper interworking with the installed base." */
void mnp_stop(mnp_t *m)
{
    if (m  &&  m->state != ST_DOWN)
        enter_down(m, MNP_STATUS_DISCONNECTED);
}

bool mnp_is_connected(const mnp_t *m)
{
    return m  &&  m->state == ST_DATA;
}

void mnp_get_stats(const mnp_t *m, mnp_stats_t *st)
{
    *st = m->st;
}
