# VPCM Refactor Plan

## Goal

Make V.91 bring-up reliable over a raw G.711 bearer by separating modem state,
call/stream timing, and transport glue.

## Architecture

### 1. Call layer

`vpcm_call.[ch]` owns the per-call parameters and the duplex stream pair:

- one TX G.711 stream
- one RX G.711 stream
- call metadata such as law and caller/callee identifiers
- tick-based timing derived from sample count
- top-level phone-line lifecycle state

Top-level call state should model the line, not the modem:

- `CALL_IDLE`
- `CALL_WAIT_DIALTONE`
- `CALL_DIAL`
- `CALL_WAIT_RINGING`
- `CALL_ANSWER`
- `CALL_RUN`
- `CALL_HANGUP`
- `CALL_DONE`

Within `CALL_RUN`, the active modem family is selected separately, e.g.:

- `CALL_RUN_V90_MODEM`
- `CALL_RUN_V91_MODEM`

Modem sub-phases such as V.8 audio negotiation, V.91 startup, and V.91 data
belong under the modem/session layer, not in the top-level call lifecycle.

### 2. G.711 stream layer

`vpcm_g711_stream.[ch]` is the canonical framed bearer object:

- fixed 8000 Hz, 8-bit octet stream model
- full-duplex achieved by using separate TX and RX stream instances
- internal frame cadence based on sample count
- suitable for raw codeword carriage, not just voice-like audio

Preferred internal cadence is 8 ms / 64 octets per frame. Larger transport
packetizations can be formed by aggregating frames.

### 3. Modem session layers

- `vpcm_v8_session.[ch]`
  - analog-style negotiation over linear PCM
  - explicit SpanDSP G.711 <-> linear seam
- `vpcm_v91_session.[ch]`
  - V.91 startup and data over raw G.711 codewords
  - no lossy decode/re-encode after V.8 handoff

### 4. Link / transport layers

- loopback link
- file taps for all four directions
- network transports such as UDP/TCP/PJ-SIP

Transport code moves frames. It should not own the modem state machine.

## Loopback target

Proper loopback testing uses two call objects:

- Call A: TX -> Call B RX
- Call B: TX -> Call A RX

Optional file taps should allow raw `.g711` dumps for:

- caller TX
- caller RX
- answerer TX
- answerer RX

## Migration order

1. Extract `vpcm_g711_stream.[ch]`.
2. Extract `vpcm_call.[ch]`.
3. Move V.91 startup/data orchestration out of `vpcm_loopback_test.c`.
4. Move V.8 handling out of `vpcm_loopback_test.c`.
5. Rebuild the pure loopback harness around two `vpcm_call_t` objects.
6. Add raw G.711 file taps.
7. Reattach PJ-SIP as a thin adapter.

## Current status

The first extraction step is in place:

- `vpcm_g711_stream.[ch]`
- `vpcm_call.[ch]`
- `vpcm_link.[ch]`

These modules are intentionally small and transport-agnostic so the next steps
can move harness logic onto them without changing modem behavior all at once.
