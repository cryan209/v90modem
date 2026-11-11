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
  `ME_DATA_FRAMING=transparent` removes our framing altogether
  (`CC_TRANSPARENT`): one DTE octet per DS0 octet, so a protocol that frames
  itself keeps its own framing. That is what PPP in HDLC framing (RFC 1662)
  over a B channel needs -- V.14's start and stop elements would otherwise be
  read as part of the peer's frames, and a peer's flag would be sliced into
  two characters. 64 kbit/s only: restricted 56k robs bit 1, so an octet
  cannot survive it, and `cc_init_transparent()` refuses it.
  `ME_DATA_FRAMING=ppp` goes one step further and does what an ISDN terminal
  adaptor does (`CC_PPP`): synchronous HDLC on the B channel -- flags, bit
  stuffing and FCS-16 through SpanDSP's `hdlc.c`, the same machinery V.120
  uses -- with RFC 1662's octet-stuffed asynchronous framing towards the DTE,
  so an ordinary async `pppd` can run an ISDN PPP call. Every control
  character is escaped on the way to the DTE as well as the flag and the
  escape: a receiver's ACCM starts at 0xffffffff (RFC 1662 7.1) and is only
  negotiated down later, so anything less is stripped before the FCS is
  checked and the frame vanishes without a word. Measured against Linux's
  `ppp_async`, which accepted nothing until the control field 0x03 itself was
  escaped.
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

The rows above are loopback only. Live results against the Eicon rig
(Asterisk CLEARMODE → TopRouter → Diva BRI) are in
`eicon_isdn_20261008.md` and `eicon_mlppp_20261107.md`.

### Live status, 2026-11-11

From CT 117 with `tools/eicon_ppp_direct.py --law alaw --sip-user 2906` and
`ME_CLEARMODE=1 ME_JB_MS=40`, against the Diva `AT&F14` PPP answerer on
17920:

| Mode | Result |
|---|---|
| `AT+MS=CLEAR,0`, `ME_DATA_FRAMING=ppp` | CONNECT 64000, PAP, IPCP, 10/10 pings |
| `AT+MS=V120,0`, `ME_V120_ACK=1` | CONNECT 64000, PAP, IPCP, 10/10 pings |
| `AT+MS=V110,0,0,9600` and `38400` | no CONNECT: T1 (7.1.2.4) expires with rx 0 V.110 frames |

V.110 fails at the answerer, not in our rate adaption. The Diva sent
continuous HDLC flags from the moment it answered: 79840 of 80160 received
octets were 0x7E, the rest 0xFF. The Cisco places a CLEARMODE call as BC
`88 90` (unrestricted digital, no layer-1 rate adaption), so profile 14
settles on HDLC and never looks for V.110 frames. The 2026-10-08 V.110
passes (10/10 exact echo lines at 9600) used a temporary answerer fixed to
the V.110 profile (`AT&F3`, `+iB5`). So the rig has no permanent V.110 answerer.

**MultiTech 7910: digital calls ring a different tty.** The isicom card on
eicon420 exposes two sets of ports. ttyM4–M7 are the modem ttys (`multitech-ppp`,
`multitech-ppp-b` on 7910, `multitech-7900-b` on 7900). ttyM0–M3 report
`ATI9` "MultiTech BRI Server" and `ATI2` `ISIHP-TA`: the ISDN terminal
adapter, `S0=0`, with nothing listening on them. A CLEARMODE call to 7910
(BC `88 90`) rings **ttyM0** (`RING`, `FM:2906 TO:7910`), so the earlier
"no answer in 30 s" (here and 2026-11-04) was only an unattended tty. With
a one-off pppd answering ttyM0 (`ATA`, no stored settings changed):

| Mode | TA reported | Result |
|---|---|---|
| `AT+MS=V120,0`, `ME_V120_ACK=1` | `CONNECT 64000/V.120` | PAP, IPCP, 10/10 pings, 0% loss |
| `AT+MS=CLEAR,0`, `ME_DATA_FRAMING=ppp` | CONNECT | PAP, IPCP, 10/10 pings, 0% loss |
| `AT+MS=V110,0,0,38400` | no CONNECT in 10 s | TA sent only HDLC flags (0x7E, 99.5%); our T1 (10 s) ended it |

Whether the TA would settle on V.110 given longer than our fixed T1
(`V110_T1_OCTETS`) or needs a protocol setting is untested.

**Permanent answerer (2026-11-11 12:45):** `multitech-7910-ta.service` on
eicon420 runs `pppd call multitech-7910-ta`. The peer file is a copy of
`multitech-7910` on `/dev/ttyM0` with its own address pair
10.254.121.1:10.254.121.2, `ifname e79p11` and `linkname multitech-7910-ta`.
The chat script is `/etc/chatscripts/multitech-ta-answer` (`ATE0V1S0=1`).
Supporting changes, each with a `.before-ta-20261111` backup:
- `modem-connect` accepts ttyM0/ttyM1;
- `collect-members.py` maps the peer to ttyM0/7910;
- `pap-secrets` allows 10.254.121.0/24 on the four `multitech-7910` lines.
  Without that, IPCP closed with "Unauthorized remote IP address" after PAP
  had succeeded.

Through the service, V.120 and CLEAR sync PPP each gave 64000, PAP, IPCP,
10/10 pings. A voice call to 7910 still lands on ttyM4 (V.34 31200, 10/10).
A second digital call made during the first rings **ttyM1**
(`CONNECT 64000/V.120`), while the first ran 70/70 pings. `multitech-7910-ta-b.service` now answers it: the peer is `multitech-7910-ta-b`
on ttyM1, 10.254.122.1:10.254.122.2, `ifname e79p12`, with pap-secrets
widened to 10.254.122.0/24. Two overlapping V.120 calls: the first on ttyM0
ran 60/60 pings and the second on ttyM1 20/20, both at 64000 and 0% loss.

**Multilink over both B channels (RFC 1990):** `tools/eicon_mlppp.py` from
CT 117, two links `ext=7910 ms=AT+MS=CLEAR,0 framing=ppp clearmode=1`
(users 2906, 2907), with `tools/eicon/http_test_server.py` bound to
10.254.121.1 on eicon420. The ttyM0 link created the bundle (10.2 s) and
the ttyM1 link attached (20.4 s). Both 3 MiB transfers were SHA-256 exact:
- download: 100.7 kbit/s;
- upload: 97.6 kbit/s;
- split about 50/50 per link, no retrains.

That is 79% and 76% of 128 kbit/s, against 93% for the Diva pair (run 11
in `eicon_mlppp_20261107.md`, 12 MiB). The shorter transfer and one 44-packet
RTP loss report on link 0 are both candidates.

**The losses were hidden frame errors.** Link 0's 44 lost packets (40 events,
144–182 s) appear in its RX tap as 36 runs of exactly 160 × 0xFF, the
jitter buffer's fill, at the same times; link 1 has none. Inside a frame,
0xFF is an HDLC abort, which SpanDSP reports as a status and our receivers
ignored, so "0 bad off the line" hid about 40 dropped frames that TCP
resent. Since 2af4189f, `rx_aborts` counts each abort run once and appears in
the end-of-call line (`..., N aborts`) and as `hdlc_aborts` in
`eicon_mlppp.py`. A clean call shows 1: the idle all-1s line before the far
end starts flagging.

**7900 (second span): TA ttys ttyM2/ttyM3.** Asterisk's `7900` goes
through `toprouter-bri` (A-law only), so `17900` was added, a clearmode
route via endpoint 6501 like `17920`–`17950` (backup
`extensions.conf.before-17900-20261111`). A digital call to 17900 rings
**ttyM2** (`CONNECT 64000/V.120`). The answerers are:

| Service | Peer | tty | Addresses | ifname |
|---|---|---|---|---|
| `multitech-7900-ta` | `multitech-7900-ta` | ttyM2 | 10.254.123.1:.2 | `e79p13` |
| `multitech-7900-ta-b` | `multitech-7900-ta-b` | ttyM3 | 10.254.124.1:.2 | `e79p14` |

Both peers use PAP name `multitech-7900-ta` and the `multitech-7910-ta`
options (multilink, nopersist). `pap-secrets` has four new
`multitech-7900-ta` lines that copy the `multitech-7900` clients. The TA
answer chat script and `modem-connect` cover ttyM2/M3. Backups end
`.before-ta79-20261111`.

**ttyM2 is shared with the 7900 routing init.** Before each start,
`multitech-7900-b` runs `multitech-7900-route.py`, which opens ttyM2 and
sends `AT*!DN1`, `AT*!EN2`, `AT*!N2=7900`. To keep those commands out of a
live call:
- the script now exits early while `/run/lock/LCK..ttyM2` exists;
- `multitech-7900-ta` starts `After=multitech-7900-b.service`, so the routing
  is still set at boot.

Results:
- **17900 single calls (ttyM2):** V.120 and CLEAR sync PPP each 64000, PAP,
  IPCP, 10/10 pings. Voice 7900 still lands on ttyM7 (V.34 31200, 10/10).
- **2-link MLPPP to 17900:** ttyM2 created the bundle and ttyM3 attached.
  Both 3 MiB transfers were SHA-256 exact: 104.5 kbit/s down, 104.4 up
  (82%). RTP loss was 0, with 0 bad frames and 1 abort per link (startup),
  and no retrains.

**Four B channels (7910 x2 + 17900 x2), first attempt (2026-11-11 13:33):**
all four links joined the bundle. Download: 6 MiB SHA-256 exact at
133.4 kbit/s. Upload: froze. **ttyM1 (7910's second TA port) went deaf at
the start of the download.** From ~35 s both directions carried only idle
flags. The server's LCP echo requests never reached the line, and ours
never reached its pppd; both ends dropped the link after six echoes (60 s).
The three remaining links pushed 228 kbit/s into the bundle for 45 s, then
the bundle stopped both ways. Only keepalives and a TCP retransmit every
~60 s passed, with no ACK, until the 940 s timeout. Our side shows 0 RTP
transmit gaps over 35 ms, and 0 aborts and 0 bad frames on the working
links. Linux multilink reassembly wedging on a lost fragment would explain
the stall, but the first loss is not located.

**Suspected cause of the deaf port: the isicom driver.** The out-of-tree
`~/isicom-port` module (`isicom.ko`, the loaded one) keeps one
`card->port_status` mask for every port. `isicom_throttle()` and
`isicom_unthrottle()` read-modify-write it and `outw()` it to the card
**without `card_lock`**. Every other writer (open, both termios paths,
shutdown) holds the lock, and upstream `isicom.c` has the same unlocked
code. The PPP async line discipline unthrottles from its receive path, so
four ports receiving at line rate can race on different CPUs and lose one
port's bit. That port then stays throttled at the card until it is
reopened, which is also why ttyM1 worked again after its pppd restarted.
Fix: `tools/eicon/isicom-port-throttle-lock.patch`. A second unpatched run
lost a *different* port (ttyM3, 17900's second B channel) and froze the
upload again, with our side clean: 0 bad FCS or overlong frames from the
DTE, 0 RTP loss, 1 startup abort per link.

**Loaded 2026-11-11 14:24.** The installed module was backed up as
`isicom.ko.before-throttle-lock-20261111`, and `~/isicom-port` was patched
(old files `.pre-throttle-lock`); srcversion A9A2B2F8… is now D2FC2057….
After the reload, voice to 7910 and 7900 ran 31200 and V.120 to 7910 and
17900 ran 64000, all at 0% loss.

**Four B channels on the patched driver:**

| Direction (6 MiB) | Result | Rate | Line use |
|---|---|---|---|
| Download | SHA-256 exact | 233.7 kbit/s | 91% of 256 kbit/s |
| Upload | SHA-256 exact | 237.9 kbit/s | 93% of 256 kbit/s |

Each link carried ~1.9 MB per direction. No echo failures on any server
pppd, 0 RTP loss, 0 bad frames, 1 startup abort per link. The upload freeze
went with the deaf port: it was a consequence of the dead member, not a
separate fault.

**Eight B channels, MultiTech and Diva in one bundle (2026-11-11).** Links
7910 ×2, 17900 ×2 (the MultiTech TA ttys) and 17920 ×2, 17930 ×2 (the Diva
answerers) all joined one MP bundle on eicon420; every link CONNECT 64000 in
0.1 s. 12 MiB each way:

| Direction (12 MiB) | Result | Rate | Line use |
|---|---|---|---|
| Download | SHA-256 exact | 281.7 kbit/s | 55% of 512 kbit/s |
| Upload | SHA-256 exact | 456.5 kbit/s | 89% of 512 kbit/s |

The split is even across both card types: 2.02–2.15 MB per link down,
1.97–2.09 MB up. 0 RTP loss, 0 bad frames, 1 startup abort per link, no
echo failures. The upload scales with links (238 → 457 kbit/s), so mixing
the two cards in one bundle works. The download does not (234 → 282): every
link ran at the same ~45 kbit/s, so a bundle-wide limit, not one slow member,
is pacing eicon420's MP sender.

**Why the 8-link download stalls: the isicom transmit queue.** A 6 MiB rerun
gave 276.5 kbit/s. Our side of the line was clean: 0 RTP loss, 0 bad frames.
TCP on eicon420 was loss-limited, not window-limited: cwnd 16–23, ssthresh
11–14, 149 KB retransmitted (2.3%), and rwnd-limited only 0.1% of the time.
eicon420's fq_codel dropped nothing. CT 117's bundle counted 34–81 receive
drops, which is where Linux counts MP fragments it gives up on.
`eicon_mlppp.py --mp-trace` logs each fragment's RFC 1990 sequence number
and arrival time per link; `tools/mp_skew.py` joins the links:

| Links | Lateness vs neighbouring seqs, median / p95 |
|---|---|
| MultiTech TA (7910 ×2, 17900 ×2) | +306…+384 / +674…+719 ms |
| Diva (17920 ×2, 17930 ×2) | −237…−245 / −31…−38 ms |

The MultiTech fragments arrive ~580 ms behind the Diva ones. Replaying the
arrivals, the receive reassembly queue sits at ppp_generic.c's
PPP_MP_MAX_QLEN (128 fragments) on 1,345 arrivals. At that point the kernel
stops waiting for the gap and drops the packets in it, and TCP backs off.
With 8 links a 1500-octet packet becomes 8 fragments, so 128 fragments is
only ~24 KB, ~0.37 s of bundle data. A 4-link bundle has 4× the slack, and
there all members are MultiTech, so their delays match anyway.

The 580 ms is the isicom driver's per-port host queue: `isicom_write_room`
offers ISICOM_XMIT_SIZE (PAGE_SIZE, 4096 octets) to ppp_async, which is
0.51 s at 64 kbit/s. `tools/eicon/isicom-tx-queue.patch` (on top of the
throttle-lock patch) caps it with a runtime-writable module parameter,
`tx_queue`, default 512 octets (~64 ms). The 10 ms tx timer drains up to
TX_SIZE (254) per tick, and the tty layer refills at WAKEUP_CHARS (256).
Built, not yet loaded. 103 of 34,022 sequence numbers never reached CT 117
on any link, a separate 0.3% loss still to place.

## Not done

- **SDP**: `ME_CLEARMODE=1` offers RFC 4040 `CLEARMODE/8000` ahead of PCMU/PCMA
  when the next call's `AT+MS` offer is CLEAR, V.110 or V.120 (a codec added
  to the vendored pjmedia passthrough table: octets copied, no VAD/PLC).
  The answerer picks it the same way; a gateway that lacks it falls back to
  PCMU/PCMA as before. Default off, since no CLEARMODE gateway has been tried
  (`make sip-clearmode-test` proves two of our own endpoints negotiate it and
  carry V.120 over it; it proves nothing about a real gateway).
- X.75 is recognised by `AT+MS` and always ERROR.
