#include "k56flex_v8bis.h"

#include <math.h>
#include <spandsp.h>
#include <stdlib.h>
#include <string.h>

#define RATE 8000
#define BLOCK 160                     /* 20 ms Goertzel block */
#define DETECT_BLOCKS 6               /* 120 ms of a clean dual tone */
#define TONE_AMP 1800.0               /* per tone, linear peak */
#define DEFAULT_LISTEN_MS 3000u
#define DELAY_TX_MS 500u              /* AB9C / A8FC: "waits 500 ticks" */
#define MSG_GAP_MS 250u               /* A978/A993 wait before ACK1/NAK1 */
#define MSG_DEADLINE_MS 3200u         /* receive deadline 3200 ticks */
#define ACK_WINDOW_MS 1500u
#define PREAMBLE_MARKS 32             /* AA0D */
#define TRAILER_MARKS 16

/* Phase increments 2C00/4010, 0CCC; 30ED/4733, 3CCC at 8 kHz, 16-bit phase. */
static const double CRE1[2] = {11264.0 * RATE / 65536, 16400.0 * RATE / 65536};
static const double CRE2 = 3276.0 * RATE / 65536;
static const double CRD1[2] = {12525.0 * RATE / 65536, 18227.0 * RATE / 65536};
static const double CRD2 = 15564.0 * RATE / 65536;
#define CRE_SEG1 (72 * 32)
#define CRE_SEG2 (25 * 32)
#define CRD_SEG1 (100 * 32)
#define CRD_SEG2 (25 * 32)

typedef struct { double coeff; double s1, s2; } goertzel_t;

struct k56flex_v8bis_s {
    k56flex_v8bis_cfg_t cfg;
    k56flex_v8bis_state_t state;
    k56flex_v8bis_result_t result;
    uint64_t t;                       /* tx samples emitted */
    uint64_t state_t;                 /* t at last state entry */
    uint64_t last_change;
    /* tone detector */
    goertzel_t g[2];
    double sumsq;
    int blk;
    int hits;
    uint64_t detect_t;
    /* tone generator */
    int tone_pos, tone_total1, tone_total2;
    const double *tone_f1;
    double tone_f2;
    double phase[2];
    /* V.21 */
    fsk_tx_state_t *ftx;
    fsk_rx_state_t *frx;
    hdlc_rx_state_t *hrx;
    uint8_t txbits[512];
    unsigned tx_n, tx_i, tx_trail;
    uint8_t peer[32];
    size_t peer_len;
    bool peer_ok;
    int ack;
    int frames_seen;
    bool sent_reply;
    bool have_nak_reply;
    bool next_is_msg;                 /* after DELAY_TX: send the message, not tones */
};

static void state_enter(k56flex_v8bis_t *s, k56flex_v8bis_state_t st)
{
    s->state = st;
    s->state_t = s->t;
    s->last_change = s->t;
}

static void fail(k56flex_v8bis_t *s, k56flex_v8bis_result_t r)
{
    s->result = r;
    state_enter(s, K56V8B_FAILED);
}

static unsigned ms_since(const k56flex_v8bis_t *s, uint64_t from)
{
    return (unsigned)((s->t - from) * 1000 / RATE);
}

static int tx_get_bit(void *user)
{
    k56flex_v8bis_t *s = user;
    if (s->tx_i < s->tx_n) return s->txbits[s->tx_i++];
    ++s->tx_trail;
    return 1;
}

static void rx_put_bit(void *user, int bit)
{
    k56flex_v8bis_t *s = user;
    hdlc_rx_put_bit(s->hrx, bit);
}

bool k56flex_v8bis_peer_acceptable(const uint8_t *p, size_t len)
{
    unsigned type;
    if (!p || len < 14) return false;
    type = p[0] >> 4;
    if (type != 1) return false;                 /* revision-1 MS/CL octets are 11/12 */
    if ((p[0] & 0x0f) != 1 && (p[0] & 0x0f) != 2) return false;
    return p[13] == 0x02 || p[13] == 0x21;
}

static void on_frame(void *user, const uint8_t *pkt, int len, int ok)
{
    k56flex_v8bis_t *s = user;
    if (!ok || len < 1) return;
    ++s->frames_seen;
    if (len == 1 && (pkt[0] == 0x14 || pkt[0] == 0x18)) {
        s->ack = pkt[0] == 0x14 ? 1 : 2;
        return;
    }
    if ((pkt[0] == 0x11 || pkt[0] == 0x12) && !s->peer_len
        && (size_t)len <= sizeof(s->peer)) {
        memcpy(s->peer, pkt, (size_t)len);
        s->peer_len = (size_t)len;
        s->peer_ok = k56flex_v8bis_peer_acceptable(pkt, (size_t)len);
        if (s->cfg.client_model)                        /* test stand-in: peer is the server */
            s->peer_ok = len >= 14 && (pkt[0] >> 4) == 1 && pkt[13] == 0x42;
    }
}

static void queue_message(k56flex_v8bis_t *s, k56flex_v8bis_msg_t type)
{
    uint8_t payload[16], frame[32], bits[300];
    size_t n = k56flex_v8bis_payload(type, s->cfg.v90_capable, s->cfg.mu_law, payload);
    size_t fn, nb, i;
    if (s->cfg.client_model && n == 16)
        payload[13] = 0x02;                             /* client version octet; server is 0x42 */
    fn = k56flex_v8bis_frame(payload, n, frame);
    nb = k56flex_v8bis_stuff(frame, fn, bits);
    s->tx_n = 0;
    for (i = 0; i < PREAMBLE_MARKS; ++i) s->txbits[s->tx_n++] = 1;
    for (i = 0; i < nb; ++i) s->txbits[s->tx_n++] = bits[i];
    s->tx_i = 0;
    s->tx_trail = 0;
    fsk_tx_restart(s->ftx, &preset_fsk_specs[s->cfg.role == K56FLEX_V8BIS_INITIATE ? FSK_V21CH1 : FSK_V21CH2]);
}

k56flex_v8bis_t *k56flex_v8bis_new(const k56flex_v8bis_cfg_t *cfg)
{
    k56flex_v8bis_t *s = calloc(1, sizeof(*s));
    int tx_ch, rx_ch, i;
    static const double det_f[2][2] = {{1375.0, 2002.0}, {1529.0, 2225.0}};
    if (!s) return NULL;
    s->cfg = *cfg;
    if (!s->cfg.listen_ms) s->cfg.listen_ms = DEFAULT_LISTEN_MS;
    tx_ch = cfg->role == K56FLEX_V8BIS_INITIATE ? FSK_V21CH1 : FSK_V21CH2;
    rx_ch = cfg->role == K56FLEX_V8BIS_INITIATE ? FSK_V21CH2 : FSK_V21CH1;
    s->ftx = fsk_tx_init(NULL, &preset_fsk_specs[tx_ch], tx_get_bit, s);
    s->frx = fsk_rx_init(NULL, &preset_fsk_specs[rx_ch], FSK_FRAME_MODE_SYNC, rx_put_bit, s);
    s->hrx = hdlc_rx_init(NULL, false, true, 1, on_frame, s);
    if (!s->ftx || !s->frx || !s->hrx) { k56flex_v8bis_free(s); return NULL; }
    fsk_tx_power(s->ftx, -12.0f);
    /* The responder listens for CRe; the initiator, after its signal, for CRd. */
    for (i = 0; i < 2; ++i) {
        double f = det_f[cfg->role == K56FLEX_V8BIS_INITIATE ? 1 : 0][i];
        s->g[i].coeff = 2.0 * cos(2.0 * M_PI * f / RATE);
    }
    if (cfg->role == K56FLEX_V8BIS_INITIATE) {
        s->tone_f1 = CRE1; s->tone_f2 = CRE2; s->tone_total1 = CRE_SEG1; s->tone_total2 = CRE_SEG2;
        state_enter(s, K56V8B_DELAY_TX);
    } else {
        state_enter(s, K56V8B_IDLE_WAIT);
    }
    return s;
}

void k56flex_v8bis_free(k56flex_v8bis_t *s)
{
    if (!s) return;
    if (s->ftx) fsk_tx_free(s->ftx);
    if (s->frx) fsk_rx_free(s->frx);
    if (s->hrx) hdlc_rx_free(s->hrx);
    free(s);
}

static bool detector_active(const k56flex_v8bis_t *s)
{
    return s->state == K56V8B_IDLE_WAIT || s->state == K56V8B_WAIT_TONES;
}

static void detect_block(k56flex_v8bis_t *s)
{
    double p[2], n = BLOCK, frac1, frac2;
    int i;
    for (i = 0; i < 2; ++i)
        p[i] = s->g[i].s1 * s->g[i].s1 + s->g[i].s2 * s->g[i].s2 - s->g[i].coeff * s->g[i].s1 * s->g[i].s2;
    frac1 = s->sumsq > 0 ? 2.0 * p[0] / (n * s->sumsq) : 0;
    frac2 = s->sumsq > 0 ? 2.0 * p[1] / (n * s->sumsq) : 0;
    if (s->sumsq / n > 60.0 * 60.0 && frac1 > 0.15 && frac2 > 0.15 && frac1 + frac2 > 0.6) {
        if (++s->hits == DETECT_BLOCKS) {
            s->detect_t = s->t - (uint64_t)DETECT_BLOCKS * BLOCK;
            if (s->cfg.role == K56FLEX_V8BIS_RESPOND && s->state == K56V8B_IDLE_WAIT) {
                state_enter(s, K56V8B_DELAY_TX);
                s->state_t = s->detect_t;     /* the wait runs from the tone onset */
            } else if (s->cfg.role == K56FLEX_V8BIS_INITIATE && s->state == K56V8B_WAIT_TONES) {
                s->next_is_msg = true;
                state_enter(s, K56V8B_DELAY_TX);
                s->state_t = s->detect_t;
            }
        }
    } else {
        s->hits = 0;
    }
    for (i = 0; i < 2; ++i) s->g[i].s1 = s->g[i].s2 = 0;
    s->sumsq = 0;
    s->blk = 0;
}

void k56flex_v8bis_rx(k56flex_v8bis_t *s, const int16_t *amp, int len)
{
    int i;
    if (s->state == K56V8B_WAIT_PEER_MSG || s->state == K56V8B_WAIT_ACK)
        fsk_rx(s->frx, amp, len);
    if (!detector_active(s)) return;
    for (i = 0; i < len; ++i) {
        int k;
        double x = amp[i];
        for (k = 0; k < 2; ++k) {
            double s0 = x + s->g[k].coeff * s->g[k].s1 - s->g[k].s2;
            s->g[k].s2 = s->g[k].s1;
            s->g[k].s1 = s0;
        }
        s->sumsq += x * x;
        if (++s->blk == BLOCK) {
            /* the clock for onset timing is tx-driven; approximate with it */
            detect_block(s);
        }
    }
}

static void gen_tones(k56flex_v8bis_t *s, int16_t *amp, int len)
{
    int i;
    for (i = 0; i < len; ++i) {
        double v;
        if (s->tone_pos < s->tone_total1)
            v = TONE_AMP * (sin(s->phase[0]) + sin(s->phase[1]));
        else
            v = 2.0 * TONE_AMP * sin(s->phase[0]);
        amp[i] = (int16_t)lrint(v);
        s->phase[0] += 2.0 * M_PI * (s->tone_pos < s->tone_total1 ? s->tone_f1[0] : s->tone_f2) / RATE;
        s->phase[1] += 2.0 * M_PI * s->tone_f1[1] / RATE;
        ++s->tone_pos;
    }
}

static void begin_tones(k56flex_v8bis_t *s)
{
    if (s->cfg.role == K56FLEX_V8BIS_INITIATE) {
        s->tone_f1 = CRE1; s->tone_f2 = CRE2; s->tone_total1 = CRE_SEG1; s->tone_total2 = CRE_SEG2;
    } else {
        s->tone_f1 = CRD1; s->tone_f2 = CRD2; s->tone_total1 = CRD_SEG1; s->tone_total2 = CRD_SEG2;
    }
    s->tone_pos = 0;
    s->phase[0] = s->phase[1] = 0;
    state_enter(s, K56V8B_TX_TONES);
}

/* Decide what to send after the peer's message, or after the tone exchange. */
static void begin_message(k56flex_v8bis_t *s, k56flex_v8bis_msg_t type)
{
    queue_message(s, type);
    state_enter(s, K56V8B_TX_MSG);
}

void k56flex_v8bis_tx(k56flex_v8bis_t *s, int16_t *amp, int len)
{
    int done = 0;
    memset(amp, 0, sizeof(*amp) * (size_t)len);
    while (done < len) {
        int chunk = len - done;
        switch (s->state) {
        case K56V8B_IDLE_WAIT:
            s->t += (uint64_t)chunk;
            if (ms_since(s, s->state_t) > s->cfg.listen_ms)
                fail(s, K56V8B_NO_CRE);
            done += chunk;
            break;
        case K56V8B_DELAY_TX:
            s->t += (uint64_t)chunk;
            done += chunk;
            if (ms_since(s, s->state_t) >= DELAY_TX_MS) {
                if (s->next_is_msg)
                    begin_message(s, K56FLEX_V8BIS_CL);
                else
                    begin_tones(s);
            }
            break;
        case K56V8B_TX_TONES: {
            int left = s->tone_total1 + s->tone_total2 - s->tone_pos;
            if (chunk > left) chunk = left;
            gen_tones(s, amp + done, chunk);
            s->t += (uint64_t)chunk;
            done += chunk;
            if (s->tone_pos >= s->tone_total1 + s->tone_total2) {
                if (s->cfg.role == K56FLEX_V8BIS_INITIATE) {
                    s->hits = 0; s->blk = 0; s->sumsq = 0;
                    {
                        static const double crd[2] = {1529.0, 2225.0};
                        int i;
                        for (i = 0; i < 2; ++i) {
                            s->g[i].coeff = 2.0 * cos(2.0 * M_PI * crd[i] / RATE);
                            s->g[i].s1 = s->g[i].s2 = 0;
                        }
                    }
                    state_enter(s, K56V8B_WAIT_TONES);
                } else {
                    fsk_rx_restart(s->frx, &preset_fsk_specs[FSK_V21CH1], FSK_FRAME_MODE_SYNC);
                    hdlc_rx_restart(s->hrx);
                    state_enter(s, K56V8B_WAIT_PEER_MSG);
                }
            }
            break;
        }
        case K56V8B_WAIT_TONES:
            s->t += (uint64_t)chunk;
            done += chunk;
            if (ms_since(s, s->state_t) > 2500u) {
                if (s->cfg.blind) {
                    s->next_is_msg = true;               /* no CRd: send CL regardless */
                    state_enter(s, K56V8B_DELAY_TX);
                } else {
                    fail(s, K56V8B_NO_CRD);
                }
            }
            break;
        case K56V8B_WAIT_PEER_MSG:
            s->t += (uint64_t)chunk;
            done += chunk;
            if (s->peer_len) {
                if (s->cfg.role == K56FLEX_V8BIS_RESPOND) {
                    begin_message(s, s->peer_ok ? K56FLEX_V8BIS_MS : K56FLEX_V8BIS_NAK1);
                    s->have_nak_reply = !s->peer_ok;
                } else {
                    s->sent_reply = true;
                    begin_message(s, s->peer_ok ? K56FLEX_V8BIS_ACK1 : K56FLEX_V8BIS_NAK1);
                    s->ack = s->peer_ok ? 1 : 2;
                    s->have_nak_reply = !s->peer_ok;
                }
            } else if (ms_since(s, s->state_t) > MSG_DEADLINE_MS) {
                fail(s, K56V8B_NO_MESSAGE);
            }
            break;
        case K56V8B_TX_MSG: {
            if (chunk > 80) chunk = 80;
            fsk_tx(s->ftx, amp + done, chunk);
            s->t += (uint64_t)chunk;
            done += chunk;
            if (s->tx_i >= s->tx_n && s->tx_trail >= TRAILER_MARKS) {
                if (s->cfg.role == K56FLEX_V8BIS_RESPOND && !s->have_nak_reply) {
                    fsk_rx_restart(s->frx, &preset_fsk_specs[FSK_V21CH1], FSK_FRAME_MODE_SYNC);
                    hdlc_rx_restart(s->hrx);
                    state_enter(s, K56V8B_WAIT_ACK);
                } else if (s->cfg.role == K56FLEX_V8BIS_INITIATE && !s->sent_reply) {
                    fsk_rx_restart(s->frx, &preset_fsk_specs[FSK_V21CH2], FSK_FRAME_MODE_SYNC);
                    hdlc_rx_restart(s->hrx);
                    state_enter(s, K56V8B_WAIT_PEER_MSG);
                } else {
                    s->result = s->peer_ok ? K56V8B_OK : K56V8B_BAD_PEER;
                    state_enter(s, K56V8B_DONE);
                }
            }
            break;
        }
        case K56V8B_WAIT_ACK:
            s->t += (uint64_t)chunk;
            done += chunk;
            if (s->ack || ms_since(s, s->state_t) > ACK_WINDOW_MS) {
                s->result = K56V8B_OK;
                state_enter(s, K56V8B_DONE);
            }
            break;
        case K56V8B_DONE:
        case K56V8B_FAILED:
            s->t += (uint64_t)chunk;
            done += chunk;
            break;
        }
    }
}

k56flex_v8bis_state_t k56flex_v8bis_state(const k56flex_v8bis_t *s) { return s->state; }
k56flex_v8bis_result_t k56flex_v8bis_result(const k56flex_v8bis_t *s) { return s->result; }
int k56flex_v8bis_ack(const k56flex_v8bis_t *s) { return s->ack; }
unsigned k56flex_v8bis_elapsed_ms(const k56flex_v8bis_t *s) { return (unsigned)(s->last_change * 1000 / RATE); }

const uint8_t *k56flex_v8bis_peer_payload(const k56flex_v8bis_t *s, size_t *len)
{
    if (!s->peer_len) return NULL;
    if (len) *len = s->peer_len;
    return s->peer;
}

const char *k56flex_v8bis_state_name(k56flex_v8bis_state_t st)
{
    static const char *n[] = {"IDLE_WAIT", "DELAY_TX", "TX_TONES", "WAIT_TONES", "WAIT_PEER_MSG",
                              "TX_MSG", "WAIT_ACK", "DONE", "FAILED"};
    return (unsigned)st < sizeof(n) / sizeof(n[0]) ? n[st] : "?";
}

const char *k56flex_v8bis_result_name(k56flex_v8bis_result_t r)
{
    static const char *n[] = {"ok", "no CRe heard", "no CRd heard", "no peer message", "peer message rejected"};
    return (unsigned)r < sizeof(n) / sizeof(n[0]) ? n[r] : "?";
}
