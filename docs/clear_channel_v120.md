# Clear channel, V.120 and V.110 over the DS0

`AT+MS=CLEAR`, `AT+MS=V120` and `AT+MS=V110` (also `--mode`/`ME_MODE`
`clear`, `clear56`, `v120`, `v120-56`, `v110`) run a data call with no modem
at all. The SIP/G.711 bearer
here is passed through byte-exact (no transcoding,
`PJMEDIA_HAS_PASSTHROUGH_CODECS`), so every RTP payload octet is a DS0 octet
end to end: the 64 kbit/s unrestricted digital bearer RFC 4040 calls
CLEARMODE and ISDN calls a B channel. The octets carry a synchronous bit
stream directly.

Code: `clear_channel.c` (octet packing, V.120 framing, V.110 rate
adaption), the `ME_MOD_CLEAR` hooks at the top of
`me_tx_g711_impl()`/`me_rx_g711_impl()`, `me_clear_start_locked()` and
`me_v110_progress_locked()` in `modem_engine.c`. Tests: `clear_channel_test`
(in `make test`).

## The three modes

- **CLEAR**: the bit stream belongs to the ordinary data stack, exactly as a
  datapump's would: V.14 async characters (start, 8 data, stop, mark idle)
  by default, V.42 LAPM with `ME_DATA_FRAMING=lapm`. There is no V.8, so
  `auto` framing has no protocol octet to settle on and means V.14.
- **V120**: ITU-T V.120 rate adaption. DTE characters go into HDLC frames
  (flag, 2-octet address carrying the LLI, control, V.120 header octet,
  data, CRC-16 FCS) on the bit stream, via SpanDSP's `hdlc.c`. Each frame
  carries what the DTE has queued, up to 256 octets.
- **V110**: ITU-T V.110 (02/2000) rate adaption for an asynchronous DTE,
  section below.

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
[2026-10-05 conformance audit](v120_conformance_audit.md) found five defects,
and its status section records what has since been fixed (V120-1 to -4) and
what is still open (V120-5, break).

| Field | Sent | Accepted |
|---|---|---|
| Address | LLI 256, 2 octets, `08 01` from **both** ends (6.2.2.3: C/R is symmetric, UI is a command) | LLI 256 only (others counted in `rx_other_lli`, never delivered); either C/R |
| Control | `03` (UI) | UI with P/F either way |
| Header | `83`: E=1, B=1, F=1, no CS octet (3.2.3: CS is optional) | H plus at most one CS; with H.E=0 the CS must be present with E=1 (3.1.2.1), else the frame is bad and nothing is delivered |
| CS | never sent | RR is honoured: RR=0 stops user data, RR=1 resumes (3.2.4.1); DR/SR ignored |

- **Unacknowledged mode only** (UI frames). The multiple-frame acknowledged
  mode (SABME, I-frames, RR/REJ, T200) and 4.2.2's optional XID link
  verification are not implemented; such frames are counted as unsupported
  and dropped, so a peer that insists on either will not get data through.
- A received BR is counted, not delivered: the byte interface to the PTY has
  no way to signal a break (audit V120-5). Break is not sent either.
- Not implemented: synchronous (HDLC) and bit-transparent modes, segmentation,
  multiple LLIs, Q.931/LLC signalling (there is none on SIP).

## V.110

V.110 (02/2000) is in `ITU Docs/`; I.460, which 5.1.4 delegates RA2 to, is
not, so RA2 follows the convention every TA uses (and which I.460 states):
an 8/16/32 kbit/s intermediate rate occupies the first 1/2/4 transmitted bits
of each octet -- the DS0 octet's MSB first -- and the unused bits are 1.

Only the asynchronous path of 5.3 is implemented, since the DTE here is a
byte stream: **RA0** (5.3.3) puts 8N1 characters on a 2^n x 600 bit/s stream,
**RA1** maps that stream into Table 2's 80-bit frame with the bit assignments
of Tables 6a/6b/6c (600/1200/2400, D bits repeated 8/4/2 times) and 6e
(4800/9600/19200/38400), **RA2** onto the DS0.

| Item | Behaviour |
|---|---|
| User rate | Table 8 from 75 to 38400 bit/s (50 bit/s is 5-bit and is refused); the highest the `AT+MS` bounds admit both ways, 38400 by default. `AT+MS=V110,0,0,9600` is 9600. |
| Rates between RA0 rates | 3600, 7200, 12000, 14400, 24000, 28800 ride the next RA0 rate with stop elements padded so the character rate never exceeds the user rate (5.3.3). |
| Below 600 bit/s | The 600 bit/s stream *samples* the start/stop signal (5 or 6 stream bits an element at 110); the receiver is a UART on those samples, reading each element at the sample nearest its middle. |
| E bits | E1-E3 per Table 5, E4-E6 = 1 (no network-independent clock), E7 = 1 except 0 in every fourth frame at 600. A far end whose E1-E3 differ is counted and logged once ("is it set to the same rate?"). |
| Framing | Two consecutive 17-bit alignment patterns to acquire (5.1.3.1); lost after three consecutive frames with a framing-bit error (5.1.3.2). |
| Start (7.1.2) | Frames with D = 1, S = X = OFF; framing found -> S = X = ON; the far end's S = X = ON for two frames -> 107/109 ON (**CONNECT** is reported now, not when the call answers); 106 ON N = 24 bits later (6.3); T1 = 10 s. |
| Data (7.1.3) | S/X not mapped to circuits (7.1.3.3). The far end's X OFF holds our data back at a character boundary (7.1.5 c, 5.4.2). A deleted stop element is re-inserted (5.3.4); 20 or more zeros is a break (5.3.5), counted and not delivered as NULs. |
| Loss of framing (7.1.5) | Stop delivering, X OFF, resynchronize; X back ON and 106 after N bits on success; after 3 s, three frames of all-status-OFF D = 0 and hang up. |
| Disconnect (7.1.4) | The far end's S OFF with D = 0 is its request: NO CARRIER. `cc_v110_disconnect()` sends ours (S OFF, X ON, D = 0) and the far end's S OFF or loss of framing acknowledges it -- but the engine does **not** call it yet: a local ATH ends the SIP call at once, without 7.1.4.1's frames. |

Not implemented: synchronous user rates (Tables 6d/6f, 7a-7c), 7/5-bit
characters, parity and 2 stop elements as separate formats (8 data bits
include any parity, 5.3.6), sending break, the 5.4 flow-control use of X
towards the far end (we never turn X OFF for our own receive buffer), the
in-band parameter exchange of Appendix I, half duplex (7.2), and restricted
56k at 38400 (an IR of 64 kbit/s needs every bit; IR 8/16/32 never uses the
robbed LSB anyway).

Tests (`clear_channel_test`): every Table 8 rate back to back with B joining
late, data both ways byte-exact, with each frame on the wire graded by an
independent checker written from Table 2/5/6 (alignment, E bits, unused bits
1, repeats equal, S = X = OFF before ON); disconnect both sides; T1; a 0.4 s
loss of framing and recovery with no data pulled while X is OFF; a 4 s loss
ending in 7.1.5 e)'s disconnect; RA0 receive from an independent Table 6e
encoder (deleted stop, NUL, break); and the engine through the PTY, where
CONNECT must come after the S/X exchange and not with the call. `make test`
also runs `engine_pair_test` (two whole engines, numbered lines both ways)
for V.120 and for V.110 at 9600 and 38400.

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
V.110 has no live measurement at all yet; the rows above predate it.

## Not done

- **SDP**: we still offer PCMU/PCMA, not RFC 4040's `CLEARMODE/8000`. It
  works wherever the G.711 path is byte-exact end to end; a gateway that
  transcodes, or applies digital pads, echo cancellation or VAD, breaks it.
- X.75 is recognised by `AT+MS` and always ERROR.
