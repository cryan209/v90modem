# Modem Data Transfer Plan: DTE Interfaces, V.14, V.42/V.42bis

## Goal

Turn a completed handshake into a usable modem. Today `ME_DATA` moves raw
bytes between one PTY and the V.22bis placeholder with no framing, no error
correction, and no compression. This plan covers the full DTE-facing data
path:

- the async serial layer (V.14) over the synchronous datapump bit stream
- error correction (V.42 LAPM — an HDLC-derived protocol) and compression
  (V.42bis first, V.44 later)
- multiple DTE presentation modes: classic inline AT serial, split
  control/data ports, and a direct "ISP" handoff mode

## Where the layers sit

```
 DTE side                                      line side
 ---------                                     ----------
 PTY / socket
   <-> AT interpreter (command mode)           G.711 RTP (V.152-style VBD)
   <-> user data (online data mode)              <-> datapump (V.90/V.34/V.22bis)
         <-> V.42bis compression                     <-> sync bit stream
             <-> V.42 LAPM (HDLC frames)  <-------------+
             ... or V.14 async framing (no EC) <--------+
```

Two distinct HDLC-shaped layers exist in a dial-up session and must not be
conflated:

1. **V.42 LAPM** — modem-to-modem error correction. HDLC frames (like LAPD),
   negotiated between the two modems, invisible to the DTE.
2. **PPP (RFC 1662 async-HDLC)** — DTE-to-ISP framing carried as ordinary
   user payload. The modem never interprets it, except optionally in the
   "direct ISP" mode below to auto-detect PPP and hand off.

## Existing assets

- `data_interface.c` — PTY + SpanDSP `at_state_t`, command/data mode flag,
  8 KB upstream ring. Works; single-port only, no V.14/V.42.
- `modem_engine.c` `ME_DATA` — raw byte shuttle to `v22bis_tx/rx`. The V.92
  MP/capability tracking already records the far end's LAPM capability bit.
- SpanDSP in-repo: `v42.c` (LAPM with detection phase; bit-level
  `v42_tx_bit`/`v42_rx_bit` API), `v42bis.c`, `async.c` (async character
  framing), `hdlc.c`, and `data_modems.c` as a reference for wiring
  AT + datapump + V.42 together.
- Loopback pair harness (`vpcm_call_pair`, `vpcm_link`) — lets two engine
  instances run a full call against each other for automated testing.
- Local specs in `ITU Docs/`: V.42 (+Cor1), V.42bis, V.44 (+Cor1), V.80
  (+Amd1), V.150.0, V.150.1, V.152.

## DTE presentation modes

### Mode A — classic inline serial (default, exists in skeleton form)

One PTY. AT command mode and online data mode share the port; `+++` (TIES)
escapes back to command mode; `ATO` returns online.

Work needed beyond today:

- Result-code discipline: `CONNECT <dte-rate>` with extended reporting
  (`ATW`/`\V` style `PROTOCOL: LAPM`, `COMPRESSION: V.42bis`, carrier text).
- S-register and `AT\N` / `AT%C` / `AT+ES`, `AT+EB`, `AT+DS` configuration
  for EC/compression selection, with sane V.250 defaults.
- `+++` guard-time handling and `ATO` re-entry into data mode.
- Documented PTY limitations: PTYs carry no modem-control lines, so DCD/DTR
  semantics (`&C1`, `&D2`) are emulated only as result codes and hangup on
  client close. Dial-in software must use CLOCAL. This is inherent, not a
  defect; note it in the README.

### Mode B — split control + data ports

Two PTYs (later optionally TCP/UNIX sockets):

- **Control port**: always in AT command mode. Unsolicited result codes
  (RING, CONNECT, NO CARRIER) appear here even mid-call. Dial, answer,
  hangup, status queries never contend with payload bytes.
- **Data port**: carries only user data; opens "connected" once the call is
  up. No escape sequence needed — `+++` ambiguity disappears.

Precedents: modern cellular modems (AT function + data function), GSM 07.10
CMUX, and ITU V.80's separation of control from sync data access. We do not
need 07.10 multiplexing itself; two PTYs are simpler and sufficient.

CLI shape: `--pty PATH` keeps Mode A; `--control-pty PATH --data-pty PATH`
selects Mode B. Internally both modes drive the same `dte_port` abstraction.

### Mode C — direct ISP / exec handoff

Realistic and well-precedented: this is what a dial-in RAS box does
(mgetty `AUTOPPP`, agetty + login). After CONNECT (and V.42/compression, if
negotiated), the server itself launches a configured program on a fresh PTY
pair and bridges it to the modem data stream. No user-visible serial port at
all.

- `--exec "pppd ..."` — run pppd as the ISP side; the analog caller's PPP
  session terminates in the host network stack.
- `--exec "login"` (or any getty-style program) — classic shell dial-in.
- Optional `AUTOPPP`-style sniffing: in Mode A/B, detect RFC 1662 PPP
  framing (`7E FF 03` / LCP) in the first received bytes and hand off to the
  exec program automatically.

This is "direct ISP interface" without inventing anything: the modem stays a
modem; the handoff is process plumbing.

## What about V.150 / V.152?

- **V.152 (voice-band data)** describes exactly what this project already
  does on the SIP leg: G.711 passthrough with VAD/CNG/echo-cancel disabled.
  Our current transport *is* V.152-style VBD. Nothing to build; cite it.
- **V.150.1 (Modem-over-IP, modem relay)** is a different transport: the
  remote gateway terminates the analog modem's physical layer and relays
  data over IP using SPRT (packet transport) plus SSE (state events),
  negotiated in SDP. If the far-end ATA/gateway supported it, we would
  receive SPRT packets instead of PCM samples, and the entire datapump would
  be bypassed while V.42/V.14 and every DTE mode above remain unchanged.
  That is the architectural payoff of layering this plan correctly: the DTE
  modes and EC stack must sit behind an interface that does not assume the
  bits came from our own demodulator.
- Verdict: V.150.1 is out of scope for this milestone but the layer
  boundaries below are chosen so an SPRT transport can slot in beneath V.42
  later. V.150.0 is the requirements doc; no implementation impact.

## Architecture changes

### `dte_port` abstraction

Replace the `data_interface.c` singletons with a small object:

```c
typedef struct {
    int (*write)(void *ctx, const uint8_t *buf, int len);  /* to DTE */
    int (*read)(void *ctx, uint8_t *buf, int max);          /* from DTE */
    void (*on_close)(void *ctx);
} dte_port_t;
```

Implementations: PTY (existing code), later socket, exec-child. Mode A binds
AT + data to one port; Mode B binds AT to one and data to another; Mode C
binds data to an exec child and drives AT internally (auto-answer profile).

### Engine data-plane interface

The engine currently exposes byte FIFOs. V.42 is a *bit-synchronous*
protocol tied to the modem symbol clock, so the engine needs bit-level
hooks in both directions:

```c
/* engine pulls one tx bit per data-bit slot; pushes each rx bit */
typedef int  (*me_get_tx_bit_fn)(void *ctx);
typedef void (*me_put_rx_bit_fn)(void *ctx, int bit);
```

A `data_stack` module owns what those bits mean:

- **V.14 path**: `async.c`-style start/stop framing of DTE bytes
  (buffered async mode), including V.14 overspeed stop-bit deletion.
- **V.42 path**: `v42_tx_bit`/`v42_rx_bit` drive LAPM; LAPM's payload is the
  DTE byte stream (optionally through V.42bis).

The stack selects between paths at connect time via the V.42 detection
phase (ODP/ADP), falling back to V.14 buffered async when the far end never
answers detection, per V.42 Annex A timers.

### Threading

The data stack runs where the bits are produced/consumed today: the RTP
clock domain (`me_tx_audio`/`me_rx_audio` context), same as the datapump.
DTE-side reads/writes stay in the PTY reader thread with the existing ring
buffers as the boundary. LAPM retransmission timers tick off the sample
clock, not wall time, so replay tests stay deterministic.

## Work breakdown

### Phase 0: interfaces (prerequisite, small)

- Extract `dte_port_t`; port `data_interface.c` onto it (Mode A unchanged).
- Add engine bit-level tx/rx hooks alongside the existing byte FIFOs;
  V.22bis path migrates to the hooks.

Exit: current behavior identical through the new seams; loopback test green.

### Phase 1: V.14 async framing (no EC)

- `data_stack` with the V.14 path: DTE bytes → start/stop-framed bits →
  datapump; reverse on rx. 8N1 first; overspeed handling per V.14 §8.
- CONNECT reporting with real DTE/line rates.

Exit: file transfer through the loopback pair over V.22bis with framing
(not raw bit-shuttle); byte-exact at both ends.

### Phase 2: V.42 LAPM

- Wire SpanDSP `v42.c`: detection phase, XID, LAPM establishment, data
  transfer, disconnect on hangup or DISC.
- AT `+ES`/`\N` control of EC policy (LAPM-required / LAPM-preferred /
  buffered-only); extended result codes (`PROTOCOL: LAPM`).
- Break handling and LAPM reset per V.42 §8.

Exit: loopback pair negotiates LAPM, survives injected G.711 bit-error
bursts (payload intact), reports the protocol in result codes. Real analog
modem call establishes LAPM.

### Phase 3: V.42bis compression

- SpanDSP `v42bis.c` in LAPM payload path; XID negotiation of P1/P2;
  `AT+DS`/`%C` policy; `COMPRESSION:` result text.

Exit: compressible payload (text) shows measurable throughput gain over
loopback; incompressible data does not regress.

### Phase 4: Mode B split ports

- Second PTY; unsolicited codes on control port; data port lifecycle tied
  to call state; no `+++` on the data port.

Exit: dial + transfer scripted entirely over the control port while data
flows on the data port.

### Phase 5: Mode C exec/ISP handoff

- `--exec` child on PTY pair bridged to the data stack; optional AUTOPPP
  sniffing in Modes A/B.

Exit: analog caller running PPP gets an IP session terminated by pppd
spawned from this server; interactive `login` session also works.

### Phase 6 (later, separate plans)

- V.44 compression (no SpanDSP support; new implementation).
- V.80 synchronous access mode, TCP DTE ports.
- V.150.1 SPRT/SSE transport beneath the data stack.

## Testing strategy

- **Automated loopback**: extend `vpcm_loopback_test` with a data-stack
  suite — two engines over `vpcm_link`, scripted AT on both PTYs, payload
  byte-exactness checks, error-burst injection for LAPM retransmission.
- **AT conformance script**: expect-style script exercising command mode,
  `+++`, `ATO`, S-registers, result-code formats in both verbose/numeric.
- **Real-modem sessions**: existing manual flow (minicom + analog modem),
  then pppd end-to-end in Mode C.

## Non-goals for this milestone

- MNP2-4/MNP5 fallback (V.42 Annex A alternative) — LAPM + buffered
  fallback covers real-world peers; revisit only if interop testing demands.
- V.44, V.80, V.150.1 (Phase 6 stubs above).
- Fax (T.30/T.31) — different stack entirely.

## Open questions

- Whether Mode B's data port should also be available as a TCP listener in
  the first cut (tcpser-style BBS use) or PTY-only until Phase 6.
- Whether `ATA`/auto-answer profiles belong in the engine (S0 register) or
  the SIP layer (current auto-answer after 2 rings) — today both exist;
  Phase 0 should pick one owner (proposal: S0 owns policy, SIP layer obeys).
