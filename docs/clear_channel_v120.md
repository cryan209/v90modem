# Clear channel and V.120 over the DS0

`AT+MS=CLEAR` and `AT+MS=V120` (also `--mode`/`ME_MODE` `clear`, `clear56`,
`v120`, `v120-56`) run a data call with no modem at all. The SIP/G.711 bearer
here is passed through byte-exact (no transcoding,
`PJMEDIA_HAS_PASSTHROUGH_CODECS`), so every RTP payload octet is a DS0 octet
end to end: the 64 kbit/s unrestricted digital bearer RFC 4040 calls
CLEARMODE and ISDN calls a B channel. The octets carry a synchronous bit
stream directly.

Code: `clear_channel.c` (octet packing, V.120 framing), the `ME_MOD_CLEAR`
hooks at the top of `me_tx_g711_impl()`/`me_rx_g711_impl()` and
`me_clear_start_locked()` in `modem_engine.c`. Tests: `clear_channel_test`
(in `make test`).

## The two modes

- **CLEAR**: the bit stream belongs to the ordinary data stack, exactly as a
  datapump's would: V.14 async characters (start, 8 data, stop, mark idle)
  by default, V.42 LAPM with `ME_DATA_FRAMING=lapm`. There is no V.8, so
  `auto` framing has no protocol octet to settle on and means V.14.
- **V120**: ITU-T V.120 rate adaption. DTE characters go into HDLC frames
  (flag, 2-octet address carrying the LLI, control, V.120 header octet,
  data, CRC-16 FCS) on the bit stream, via SpanDSP's `hdlc.c`. Each frame
  carries what the DTE has queued, up to 256 octets.

## Conventions

- **Bit order**: the first bit of the stream goes in the octet's most
  significant bit (bit 8 in ITU numbering), as a B channel is transmitted.
  So a V.120 frame starting `08 01 03 83` (each octet sent LSB first, as
  HDLC does) appears on the DS0 as `10 80 c0 c1`.
- **Restricted 56 kbit/s**: an `AT+MS` maximum rate of 56000 or less (either
  direction) selects it. Seven bits per octet in bits 8..2; bit 1, the LSB
  robbed-bit signalling overwrites, is sent as 1 and ignored on receipt.
  `CONNECT 56000`.
- **No handshake**: as on ISDN the bearer is agreed outside the channel;
  here both ends are configured alike. A mismatch (CLEAR against V120, or
  64k against 56k) is not detected; it just delivers garbage, or nothing.
  Data starts when the call connects; CONNECT is reported straight away.

## V.120: what is and is not implemented

The initial implementation was written without the Recommendation available.
V.120 (10/1996) and Corrigendum 1 are now in `ITU Docs/`; the
[2026-10-05 conformance audit](v120_conformance_audit.md) found a wrong
answerer C/R convention, invalid CS-header acceptance, and no LLI filtering,
as well as missing control-state and break handling. The table below describes
the current implementation, including those defects, rather than a conforming
wire profile:

| Field | Sent | Accepted |
|---|---|---|
| Address | LLI 256 (default), 2 octets, `08 01` from the caller (C/R 0), `0A 01` from the answerer (C/R 1) | any LLI, either C/R |
| Control | `03` (UI) | UI with P/F either way |
| Header | `83`: E=1, B=1, F=1 (complete, no control-state octet) | E=0 with control-state octets skipped; BR counted |

- **Unacknowledged mode only** (UI frames). The multiple-frame acknowledged
  mode (SABME, I-frames, RR/REJ, T200) is not implemented. A received I-frame
  is counted as unsupported and dropped, so a peer that insists on
  acknowledged mode will not get data through.
- Not implemented: break signalling outwards, flow control via the
  control-state octet, segmentation (B/F bits) on receive beyond accepting
  any combination, multiple LLIs.

Address the confirmed defects and validate against an independent peer before
claiming interoperability with ISDN equipment. UI-only service and omission
of optional XID verification can be valid for a profile agreed beforehand;
synchronous segmentation is outside this asynchronous-byte implementation.

## Measured

Two `sip_v90_modem` processes calling each other peer to peer over loopback
(pjsip, real RTP, PCMU passthrough). The caller uses
`--sip-server 127.0.0.1:<answerer port>` so ATD resolves to the answerer.
Each run sends 24000 and 22400 bytes simultaneously and requires both to
arrive byte-exact.

| Mode | Runs exact both ways | DTE payload |
|---|---|---|
| CLEAR 64k | 4/4 | ~48 kbit/s (V.14: 10 line bits per character, ceiling 51.2) |
| CLEAR 56k | 4/4 | ~42 kbit/s |
| V120 64k | 4/4 | ~57 kbit/s (no start/stop) |
| V120 56k | 4/4 | ~50 kbit/s |
| CLEAR 64k + `ME_DATA_FRAMING=lapm` | 3/3 | 145-150 kbit/s of text (V.42bis) |

Nothing here has met a real ISDN terminal adaptor or a CLEARMODE gateway.

## Not done

- **SDP**: we still offer PCMU/PCMA, not RFC 4040's `CLEARMODE/8000`. It
  works wherever the G.711 path is byte-exact end to end; a gateway that
  transcodes, or applies digital pads, echo cancellation or VAD, breaks it.
- V.110 and X.75 are recognised by `AT+MS` and always ERROR.
