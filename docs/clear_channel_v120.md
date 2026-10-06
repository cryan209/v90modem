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
what is still open (V120-5, break -- since done at the framing level, see below).

| Field | Sent | Accepted |
|---|---|---|
| Address | LLI 256, 2 octets, `08 01` from **both** ends (6.2.2.3: C/R is symmetric, UI is a command) | LLI 256 only (others counted in `rx_other_lli`, never delivered); either C/R |
| Control | `03` (UI) | UI with P/F either way |
| Header | `83`: E=1, B=1, F=1, no CS octet (3.2.3: CS is optional) | H plus at most one CS; with H.E=0 the CS must be present with E=1 (3.1.2.1), else the frame is bad and nothing is delivered |
| CS | never sent | RR is honoured: RR=0 stops user data, RR=1 resumes (3.2.4.1); DR/SR ignored |

- **UI frames by default; Q.922 acknowledged mode with `ME_V120_ACK=1`**
  (`cc_v120_set_ack()`). Acknowledged operation (4.2) is modulo 128 with
  k = 15, T200 = 1.5 s, N200 = 3: SABME (P=1) / UA (F=1) from both ends (a
  collision answers UA and waits for the peer's), I-frames carrying the same
  H octet and data as UI, RR/RNR/REJ, an RR enquiry (P=1) on T200 followed
  by go-back-N, re-establishment when N200 runs out or an N(R) is invalid,
  DISC answered with UA. **Policy, not in the Recommendation:** a caller
  whose SABME is answered DM (an UI-only peer, as this one is by default) or
  not at all (N200 SABMEs, ~6 s) falls back to UI frames, so mixed pairs
  work and nothing is lost; the cost against a silent peer is that delay.
  4.2.2 link verification for UI-only mode is `ME_V120_VERIFY=1`: an XID
  command (empty information field, which 4.2.2 allows), TM20 2.5 s, NM20 3,
  data held until the response and begun anyway when the retries are spent;
  an XID command is always answered, in either mode, and 4.2.3's collision
  (XID while an SABME is outstanding) is answered without a state change.
  **Annex C (V.42bis over V.120)** is `ME_V120_COMPRESS=1` (needs the
  acknowledged link): once it is up, the TA that set it up (the caller)
  sends an XID command -- FI 0x82, GI 0xF0, PI 0 "V120", P0 = 3 (both
  directions), P1 = 1024, P2 = 32, P/F = 0 -- with TM20/NM20; the other end
  agrees to no more than was asked and returns the values in the same form
  and both start SpanDSP's V.42bis (`v42bis_init`, the initiator's P0 bits
  swapped as `data_stack.c` does). The caller's data waits for the answer;
  an answer with no V120 subfield (a peer without Annex C) or NM20 silent
  tries means no compression (C.2.3 a). A new link (SABME) drops compression
  (C.1). The compressor is flushed whenever the DTE runs dry, and before a
  break, so a break still follows its data; the decompressor is flushed after
  every frame. Not implemented: re-negotiating after data has started ("for
  further study" in C.2.1), manufacturer-ID user data, V.44, the P0 = 1/2
  single-direction offers (we ask for both and answer what we are asked).
  Not implemented either: FRMR, our own RNR (the DTE-side ring does not back-pressure V.120 yet),
  and the V.42-style "mode collision" handling of 4.2.3.
- **Break (audit V120-5).** Both directions exist at the framing level:
  `cc_send_break()` sends a frame with BR = 1 after every character already
  pulled (3.1.1.2, 7.2.2), holds the DTE's data for the break's length and
  ends it with a BR = 0 frame; a received BR = 1 is reported through
  `cc_set_break_cb()` after that frame's characters and ends on the next
  BR = 0 frame, before its characters. V.110 sends zeros for the break (never
  fewer than 24, i.e. more than 2M for M = 10, 5.3.5) then a mark, and reads
  more than 19 zeros -- or, below 600 bit/s, an all-zero character with a zero
  stop element -- as one. **The PTY cannot carry a break**: nothing written to
  a pty master makes the slave see BREAK, and a slave's `tcsendbreak` never
  reaches the master. So the DTE surface is Courier's `AT\B<n>` (n x 100 ms,
  default 3; online command mode, i.e. after `+++`; ERROR on CLEAR or with no
  call) for sending, and for receiving a counter in ATI11 ("Breaks sent N
  received M"). The data stream itself carries nothing for a received break;
  a DTE that must see one needs a different console than a pty.
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
| Flow control (5.4.2) | Our X goes OFF while the engine's DTE receive ring is under a quarter free and back ON over a half free (`cc_v110_set_rx_room()`), data transfer state only; counted in `v110_flow_holds`. Characters already in flight still fit in the quarter. |
| Loss of framing (7.1.5) | Stop delivering, X OFF, resynchronize; X back ON and 106 after N bits on success; after 3 s, three frames of all-status-OFF D = 0 and hang up. |
| Disconnect (7.1.4) | The far end's S OFF with D = 0 is its request: hang up, NO CARRIER. A local ATH (the DTE's hang-up callback, `me_hangup()`) sends ours -- S OFF, X ON, D = 0, 106 OFF -- and keeps the SIP call up until the far end's S OFF or loss of framing acknowledges it (7.1.4.3), or T2 = 5 s passes (7.1.4.1); the DTE then gets OK. A second ATH, or one before framing is found, hangs up at once. |

Not implemented: 48/56 kbit/s synchronous (Tables 7a-7c), 7/5-bit
characters, parity and 2 stop elements as separate formats (8 data bits
include any parity, 5.3.6), sending break, the
in-band parameter exchange of Appendix I, half duplex (7.2), and restricted
56k at 38400 (an IR of 64 kbit/s needs every bit; IR 8/16/32 never uses the
robbed LSB anyway).

### V.110 synchronous (`ME_V110_SYNC=1`, `cc_v110_set_sync()`)

The DTE's octets are the D-bit stream at the user rate, **least significant
bit first** (as HDLC and the async path send them), for 600, 1200, 2400,
4800, 7200, 9600, 12000, 14400, 19200, 24000, 28800 and 38400 bit/s: Table 1's
intermediate rate, Table 5's E1-E3 and Tables 6a-6f's D-bit layouts,
including the F fill bits of 6d (N x 3600, 36 D bits a frame) and 6f
(N x 12000, 30). Any other rate stays asynchronous, with a log line. The
S/X/106/109 procedure of 7.1 is unchanged; there is no break (`\B` is ERROR).

**Octet alignment is from the first data frame**, since V.110 itself defines
none (5.1.2.6). When 106 first goes ON, at D1 of the next frame, the sender
opens the stream with `0x00 0xFF` -- a run of eight zeros after the binary-1
fill -- and the receiver, hunting `1 0^8 1^8` in the D stream from the moment
it has framing, takes the first zero as an octet boundary and discards those
two octets. After that every eight D slots are an octet, whether they carried
data, idle or a hold. Three consequences:
- **Idle is 0xFF and is delivered.** A synchronous stream is continuous, so
  the far end's idle fill reaches the DTE as 0xFF octets at the line rate;
  the DTE's protocol (HDLC flags/idle, say) is what tells it from data. A
  DTE that wants only data must discard 0xFF itself, which also discards real
  0xFF data.
- **Holds are octet-aligned.** The far end's X OFF (7.1.5 c, 5.4.2) stops the
  sender at an octet boundary and it resumes at one, counting the 1s it sent
  meanwhile as slots.
- **Alignment survives a loss of framing.** The receiver counts the
  intermediate-rate bits, a frame being 80 of them on a bit-exact bearer, so
  the slots of frames it could not read are added to its phase on
  resynchronisation; the octet that straddles the gap is dropped. Tested at
  every rate with 0.4 s lost, and with that accounting disabled the rates
  whose frames are not a whole number of octets (600, 1200, 7200, 12000,
  14400, 24000, 28800) fail while the rest do not.

The independent wire checker in `clear_channel_test` decodes each frame's D
bits from Tables 6a-6f as printed and finds the opening pattern and the DTE's
octets in the transmitted stream. **A foreign terminal adaptor will not send
the `0x00 0xFF` opening**, so receiving from one hunts forever and delivers
nothing; the alignment rule is ours, as the Recommendation leaves it open.

Tests (`clear_channel_test`): every Table 8 rate back to back with B joining
late, data both ways byte-exact, with each frame on the wire graded by an
independent checker written from Table 2/5/6 (alignment, E bits, unused bits
1, repeats equal, S = X = OFF before ON); disconnect both sides; T1; a 0.4 s
loss of framing and recovery with no data pulled while X is OFF; a 4 s loss
ending in 7.1.5 e)'s disconnect; T2 against a far end that never acknowledges;
a local ATH through the engine (direct and via `+++`/`ATH` on the PTY, held
for the request and then OK) and a far end's request ending in NO CARRIER; RA0 receive from an independent Table 6e
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

- **SDP**: `ME_CLEARMODE=1` offers RFC 4040 `CLEARMODE/8000` ahead of PCMU/PCMA
  when the next call's `AT+MS` offer is CLEAR, V.110 or V.120 (a codec added
  to the vendored pjmedia passthrough table: octets copied, no VAD/PLC).
  The answerer picks it the same way; a gateway that lacks it falls back to
  PCMU/PCMA as before. Default off, since no CLEARMODE gateway has been tried
  (`make sip-clearmode-test` proves two of our own endpoints negotiate it and
  carry V.120 over it; it proves nothing about a real gateway).
- X.75 is recognised by `AT+MS` and always ERROR.
