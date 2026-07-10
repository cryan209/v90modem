/*
 * modem_engine.h — V.90 modem state machine
 *
 * Implements the digital (server/ISP) side of V.90:
 *   - Downstream: PCM codeword injection per ITU-T V.90 §5
 *   - Upstream:   V.22bis demodulation via SpanDSP (V.34 placeholder)
 *   - Handshake:  V.8 capability exchange via SpanDSP
 *
 * The modem engine is the core of the gateway. It bridges:
 *   data_interface (PTY/AT side) ←→ sip_modem (SIP/RTP side)
 *
 * Thread safety: me_rx_audio/me_tx_audio run in the PJSIP media thread;
 * me_put_data/me_get_data/me_dial/me_answer/me_hangup may run in other
 * threads. A mutex protects the shared state.
 */

#ifndef MODEM_ENGINE_H
#define MODEM_ENGINE_H

#include <stdint.h>

/* Modem state machine states */
typedef enum {
    ME_IDLE,         /* Waiting for ATD or incoming call */
    ME_DIALING,      /* Outgoing SIP call in progress (pre-audio) */
    ME_V8,           /* V.8 capability exchange (audio active) */
    ME_TRAINING,     /* V.34 / V.22bis training sequences */
    ME_DATA,         /* Data mode: V.34 duplex or V.22bis fallback */
    ME_HANGUP        /* Call tear-down in progress */
} me_state_t;

/* Negotiated modulation after V.8 */
typedef enum {
    ME_MOD_NONE,
    ME_MOD_V90,      /* V.90 downstream PCM + V.34 upstream (future) */
    ME_MOD_V34,      /* Full V.34 duplex (up to 33.6 kbps) */
    ME_MOD_V22BIS    /* Full V.22bis duplex (fallback) */
} me_modulation_t;

/* G.711 encoding law for the RTP stream */
typedef enum {
    ME_LAW_ULAW = 0,  /* G.711 µ-law (PCMU, payload type 0) — North America/Japan */
    ME_LAW_ALAW = 1   /* G.711 A-law (PCMA, payload type 8) — Europe/international */
} me_law_t;

typedef struct {
    me_state_t state;
    me_modulation_t modulation;
    me_law_t law;
    int calling_party;
    int v34_rx_stage;
    int v34_tx_stage;
    int v90_bridge_rx_stage;
    int v90_bridge_tx_stage;
    int v90_bridge_rx_event;
    int v90_phase3_started;
    int v90_phase3_j_seen;
    int v90_dil_valid;
    uint64_t phase_elapsed_ms;
    uint64_t g711_rx_octets;
    uint64_t g711_tx_octets;
    uint64_t g711_raw_v90_tx_octets;
    uint64_t g711_linear_tx_octets;
} me_diag_snapshot_t;

/* ------------------------------------------------------------------ */
/* Lifecycle                                                           */
/* ------------------------------------------------------------------ */

/* Initialise the modem engine (call once at startup). */
void me_init(void);

/* Tear down the modem engine (call on exit). */
void me_destroy(void);

/* ------------------------------------------------------------------ */
/* Control — called from data_interface AT command callbacks           */
/* ------------------------------------------------------------------ */

/*
 * Initiate an outgoing SIP call.
 * sip_uri: SIP URI string, e.g. "sip:1234@192.168.1.1"
 * The function is asynchronous; SIP setup happens via sip_modem.c.
 */
void me_dial(const char *sip_uri);

/* Answer an incoming call (received via SIP). */
void me_answer(void);

/* Hang up the current call. */
void me_hangup(void);

/* ------------------------------------------------------------------ */
/* Audio I/O — called from sip_modem.c media port (PJSIP thread)      */
/* ------------------------------------------------------------------ */

/*
 * Receive audio from the network (upstream modem signal → data).
 * amp: linear 16-bit PCM samples at 8000 Hz, len = samples in buffer.
 */
void me_rx_audio(const int16_t *amp, int len);

/*
 * Generate audio to send to the network (data → downstream PCM signal).
 * amp: output buffer to fill, len = samples to generate.
 */
void me_tx_audio(int16_t *amp, int len);

/*
 * Receive and transmit octet-transparent G.711 media. These are the preferred
 * live PJMEDIA entry points when the negotiated codec is PCMU or PCMA.
 */
void me_rx_g711(const uint8_t *codewords, int count);
int  me_tx_g711(uint8_t *codewords, int count);

/* Notify the engine that a SIP call has been connected (audio active). */
void me_on_sip_connected(void);

/*
 * Set the G.711 encoding law for V.90 downstream codeword generation.
 * Must be called before or at the same time as me_on_sip_connected().
 * V.90 uses different codeword tables for µ-law and A-law networks.
 */
void me_set_law(me_law_t law);
me_law_t me_get_law(void);

/* Enable/disable verbose [ME]/[TRACE] debug output to stderr.
 * Also controllable via VPCM_ME_VERBOSE env var.
 * Pass --verbose on the sip_v90_modem command line to activate. */
void me_set_verbose(int v);  /* 0 = quiet, non-zero = verbose */

/* Notify the engine that a SIP call has been disconnected. */
void me_on_sip_disconnected(void);

/* ------------------------------------------------------------------ */
/* Data I/O — called from data_interface (PTY thread)                 */
/* ------------------------------------------------------------------ */

/*
 * Push bytes from the application (upstream: PTY → modem → SIP).
 * Returns number of bytes accepted (may be less than len if buffer full).
 */
int me_put_data(const uint8_t *buf, int len);

/*
 * Pull bytes destined for the application (downstream: SIP → modem → PTY).
 * Returns number of bytes copied (0 if none available).
 */
int me_get_data(uint8_t *buf, int max_len);

/* ------------------------------------------------------------------ */
/* State query (for logging / diagnostics)                             */
/* ------------------------------------------------------------------ */

me_state_t      me_get_state(void);
me_modulation_t me_get_modulation(void);
void            me_get_diag_snapshot(me_diag_snapshot_t *snapshot);
const char     *me_state_to_str(me_state_t state);
const char     *me_modulation_to_str(me_modulation_t modulation);
const char     *me_law_to_str(me_law_t law);

/*
 * Returns the SIP URI requested by the last ATD command (ME_DIALING state).
 * The returned pointer is valid until the next call to me_dial().
 */
const char *me_get_dial_uri(void);

#endif /* MODEM_ENGINE_H */
