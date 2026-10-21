# MNP error control (V.42 Annex A, the "alternative procedure")

Added 2026-10-21.  `mnp.c`/`mnp.h`, wired in as `DS_FRAMING_MNP` in
`data_stack.c`.  Before this, a call whose V.42 detection found no LAPM peer
went straight to buffered V.14 (or hung up under a "required" +ES), so an
answering modem that offers only MNP got no error control from us.

## Where the protocol text comes from

V.42 Annex A, "Operation of the error control function -- Alternative
procedure", is the MNP class 2-4 link protocol written up by ITU-T.  It is in
the 11/88, 03/93 and 10/96 editions of V.42, all in `ITU Docs/`
(`T-REC-V.42-1988*`, `-1993*`, `-1996*`).  Clause numbers in the code are from
10/96; the clauses used are the same in all three.  The 2002 revision deleted
Annex A (V.42 (03/2002) Summary) but kept the hooks: the frame names in clause
3, 7.2.1.2 (the originator "may attempt to detect the presence of an
alternative error-correcting capability"), 7.2.1.3 (an alternative-procedure
protocol frame ends the answerer's detection phase), and 7.9 (disconnect or
V.14 only once "any alternative error-correcting capability has also failed").
The +ES values are V.250 (07/2003) Table 20.

MNP 5 data compression is not in any V.42 edition, and is not implemented (see
below).

## When MNP runs

| Role | +ES | What happens |
|---|---|---|
| originator | factory `3,0,x` (or any orig_fbk except 3) | V.42 detection (ODP for T400).  No ADP: switch to MNP, send LR (3 tries, T401 = 1.5 s).  No LR back: V.14 if orig_fbk 0, else NO CARRIER. |
| originator | `4,x` (orig_rqst 4, "initiate Alternative Protocol") | MNP only: LR at once, 10 tries. |
| originator | `x,4` | MNP required, LAPM not acceptable: as `4,4`. |
| originator | `x,3` | LAPM required: no MNP. |
| answerer | factory `x,x,2` (or 4) | V.42 detection, and the same bits go to an MNP receiver.  A valid LR switches to MNP at once (7.2.1.3) and answers it.  No ODP and no LR within T400: V.14 (2) or NO CARRIER (4). |
| answerer | `x,x,6` | MNP only: wait 10 s for an LR. |
| answerer | `x,x,5` | LAPM only: no MNP. |

The factory setting therefore tries MNP before V.14.  Against a plain V.14
answerer that costs the caller about 5 s more before CONNECT (T400, then 3 LRs
at T401 = 1.5 s), and the answerer's DTE sees the ODP and the three LRs as
noise (three 31-octet LRs, NULs among them -- `engine_pair_test` grades payloads
with `memmem` for that reason).  V.42 7.2.1.2 and Appendix I allow exactly
this, and V.42/MNP modems behave the same way at their factory settings.
Table 20 has no "LAPM, else V.14, never MNP" value: `+ES=3,3` (`\N4`) is
LAPM or disconnect.

The result codes: `+ER: ALT` (V.250 6.5.5), `CONNECT <rate>/ARQ/<mod>/MNP`
under `&A3`, "Error control  MNP (V.42 Annex A)" in the call page, and an MNP
section in ATI11.  The engine logs `[ME] V.42 detection found no LAPM peer;
trying the MNP alternative procedure` and `[ME] MNP connected: framing mode
3 (bit-oriented), k 8, N401 256, optimisation 0x3`.

The engine keeps `g_data_framing == DS_FRAMING_V42` for an MNP call (it means
"error control attempted; CONNECT waits for the link"); `g_data_stack.framing`
says which protocol actually came up.

## What is implemented, by clause

* **Establishment (A.6.4, A.7.1).**  Initiator LR, responder LR, initiator's
  confirming LA, all start-stop octet-oriented whatever the data phase uses.
  Negotiation (A.7.1.5): framing mode the lower of the two (we offer 3), k the
  lower (we offer 8), N401 the lower (we offer 64), and parameter 8 facilities
  both agree on (we offer 256-octet LTs and fixed-field LT/LA).  The response
  LR carries only parameters the initiator sent (A.7.1.5.6).  Constant
  parameter 1 other than 2, or k/N401 missing or 0, fails with LD reason 2/3.
  T401 retries on both sides, and a bad FCS or a repeated LR resends the LR
  (A.7.1.2, A.7.1.3).
* **Framing.**  Mode 2 (A.3): SYN DLE STX, DLE doubled, DLE ETX, CRC-16
  x^16+x^15+x^2+1 over body + ETX.  Mode 3 (A.4): HDLC flags and zero
  insertion with the 8.1.1.6.1 FCS (the LAPM one -- A.3.6's NOTE is explicit
  that mode 2's polynomial differs).  `data_stack_test` checks both against
  the catalogued check values (CRC-16/ARC 0xBB3D, X.25 0x906E).  Both receivers
  run all the time; in the data phase only the negotiated framing is accepted
  (an LR is always octet-oriented).
* **Data phase (A.7.3).**  LT/LA mod 256, V(S) = V(R) = 1 at start, receive
  credit N(k) with 0 as the busy condition (A.6.6.2.2 -- `ds_set_rx_room()`
  drives it exactly as it drives V.42's RNR).  LA rules A.7.3.3.2 a)-g):
  on a bad FCS, on every out-of-sequence LT except the first repeat of the last
  one, after k/2 unacknowledged LTs, on T402 (= T401/2), when there is nothing
  to send, and on T404 (3 s, 7 s at 1200).  Go-back retransmission on T401
  (A.7.5.1's formula with the measured round trip as Trt) or on an LA that
  repeats N(R) (A.7.3.5 a)).  N400 = 12, then LD (A.7.3.6, A.7.2.4).
* **Disconnect.**  LD received: down.  User disconnect sends no LD (A.7.2.1:
  "it is recommended that the LD frame not be sent").
* **Break.**  An LN gets its LNA (A.7.4.5); the break is not passed on, and we
  never send LN -- the DTE side here has no break signal.

## Choices the text leaves open, and deviations

* **N401's octet order.**  8.1.2.3 would put the high-order octet first, but
  MNP equipment sends 64 as `40 00` (the RasFinder's LR does, live).  We send low octet first, and the parser
  takes whichever reading lies in 1-256.
* **Duplicate-LA retransmission is guarded.**  The receiver sends an LA for
  every frame of a gap (as A.7.3.2.2 says), so after one go-back the LAs
  already in flight each repeat N(R).  Our sender ignores a repeated N(R) that
  arrives less than the shortest measured round trip after its last go-back,
  or after the oldest unacknowledged LT was sent.  Without the first the 1e-4
  rows stall for 6 s at a time.  The second is for an idle peer's T404 LA
  (A.7.3.3.2 d)), which repeats N(R) by construction: the RasFinder's crossed
  our first LT 53 ms after we sent it, against a 320 ms round trip, and
  A.7.3.5 a) read literally resent the LT (first live call, below).
* **Adaptive packet assembly** (the MNP class 4 facility, not in Annex A
  beyond "no more than N401 user octets"): each go-back halves the LT size,
  k clean LTs grow it by N401/4.  Bounds: never below what keeps k LTs filling
  a round trip (else the window, not the errors, sets the rate -- the 31200
  burst row lost 20% to that), never above ~300 ms of line (72 octets at 2400,
  N401 from 9600).  The ceiling matters because a retransmitted LT must keep
  its length and N(S): the peer may already hold it with its LA lost, so a
  256-octet frame at 2400 that keeps being hit can never be cut down.  Before
  the ceiling, `mnp-slow-2400-bursts` stalled for 20-30 s.
* **LTs are accepted while we advertise credit 0.**  A.7.3.2.3 says discard
  them; we have the room (busy is asserted with half the ring free), and
  keeping them costs the peer nothing.
* **An async frame after mode-3 frames starts with 20 marks.**  Found by the
  stress test: when the confirming LA was lost and had to be resent from the
  data phase, the far start-stop receiver was mid-"character" in our flags
  and never found the SYN, and establishment failed.

## MNP 5: not done, deliberately

MNP 5 (adaptive frequency encoding plus run-length coding) has no
specification in `ITU Docs/`, and nothing checks the data after
decompression: an encoder that is not bit-exact with the peer's delivers wrong
octets to the DTE with no error anywhere.  We never offer it (we send no LR
parameter for it, and the response LR drops parameters we did not understand,
A.7.1.5.6), so a peer cannot switch it on.  Do it only with a reference
implementation or captures to be bit-exact against.

## Tests

* `v42_stress_test` mnp rows (fast tier): both ends MNP in mode 3 and mode 2,
  independent errors to 1e-4, 64-bit bursts at 31200, noise and long outages,
  300 ms each way, 2400 with bursts, a slow and a stalled DTE (credit 0), and
  both ways in from V.42 detection.  8 seeds pass.  Typical figures (DTE
  kbit/s each way): clean 9600 9.2 (mode 2: 7.2); 1e-4 at 14400 8.4-9.7;
  bursts at 31200 13-14.5; 300 ms RTT at 48000 ~24 (window-bound: 8 x 256
  octets per round trip); 2400 with bursts 1.2-1.4.  MNP has no selective
  reject, so the error rows sit well below V.42's.
* `data_stack_test`: the two FCS check values, the initiating LR, octet for
  octet, against Table A.2, and the default path against a plain V.14 answerer
  (V.42 detection, then 3 LRs, then the V.14 fallback, with the peer's
  early data still delivered and ours delivered after), and 200 octets going
  out as one LT once the 256-octet facility is agreed.
* `engine_pair_test` (fast tier): V.32bis caller at factory +ES against an
  MNP-only answerer (`+ES=3,0,6`); an MNP-only caller (`+ES=4,0`) against a
  factory answerer; V.34 with the answering DTE stalled for 6 s.  Each grades
  `+ER: ALT` and `CONNECT .../MNP` on both sides and the payload both ways.

## Live calls

### RasFinder, 21 October 2026

Nine outbound calls from vmhost CT 117 (wired) to the RasFinder's MultiTech
modems on 6311-6313, our side MNP-only (`AT+ES=4,4`, `AT&A3`, `AT+ER=1`),
both bit streams dumped and read with `tools/mnp_frames.py`.  Recipe:
`tools/soak/rasfinder_mnp_call.sh`.  Builds: 0bc12cfe's `mnp.c` (c1-c7) and
the same with the changes committed with this note (c8, c9).  Captures: `/root/mnp-runs/c1`-`c9` on CT
117 (disposable; the frame listings below are the record).

* **8 of 9 calls connected**, `+ER: ALT` then `CONNECT 2400/ARQ/V22B/MNP`
  (V.22bis, 7 calls) or `CONNECT 14400/ARQ/V34/MNP` (V.34, one -- that call
  offered V.90 and fell back).  The banner and `login:` came down in LTs, and
  what we typed went up and was echoed, answered (`Invalid User Name!!!`) and
  finally answered with `You are logged off` and an LD, which took us down
  cleanly.  Data moves both ways under MNP against real equipment.
* **Establishment.**  The peer's response LR is ours with k = 7:
  `02 | 01 06 01 00 00 00 00 ff | 02 01 03 | 03 01 07 | 04 02 40 00 | 08 01 03`.
  So mode 3, k 7, the 256-octet facility and fixed-field LT/LA are all
  agreed; N401 comes low octet first (Not-verified item 2, settled).  Its
  first bit-oriented LA arrives about 0.2 s after our octet-oriented
  confirming LA: the switch to mode 3 straight after that LA is right
  (item 1, settled).  Its first LAs carry credit 0 for ~2 s, until its login
  process attaches.
* **256-octet LTs are delivered whole.**  A 72-octet LT (our adaptive packet
  assembly ceiling at 2400) was acknowledged and its echo was 64 octets: that
  looked like the peer honouring parameter 4's N401 and discarding the rest.
  It is the login field.  It holds 64 characters and drops the rest of the
  write.  A 72-octet LT of 40 `a`, 20 backspaces and 12 `b` comes back as
  20 `a` and all 12 `b` (three calls), and octets 65-72 are `b`.  A cap at
  parameter 4 was written, tested, and withdrawn on this evidence: it cost about a
  tenth of the throughput on most `v42_stress_test` rows, and nearly three
  quarters at 300 ms each way.
  The peer's line editor also echoes stale buffer contents after a line with
  backspaces; read what it echoes, not how it looks.
* **The peer sometimes ignores an in-sequence LT.**  Three LTs in two calls
  (c5, c8), always after its login application had just overflowed or rejected a
  line, the peer neither acknowledged nor rejected a CRC-clean LT (our
  transmit tap, demodulated, shows it intact): no LA for an invalid frame, no
  credit 0.  Its next T404 LA (every 3 s) repeats N(R), our go-back resends
  (A.7.3.5 a)), and the resend is accepted.  Costs up to 3 s; nothing to fix
  on our side, and the T404 guard above did not suppress any of these
  go-backs (they arrive well over a round trip after the LT).
* **One call never established** (c3, V.22bis).  The peer sent marks (its
  detection phase) for 3.8 s, never answered our LRs at 0, 1.6 and 3.3 s --
  all CRC-clean on our demodulated transmit tap -- then gave up on error
  control, sent its banner as plain V.14, and its login read our fourth LR as
  typed text.  Its detection window is under 4 s after training, and in that
  call it heard nothing it accepted; the other eight answered our first LR
  within 0.45 s.  Not reproduced.

### CX93001 dialling in, 21 October 2026

The Conexant CX93001 (`CX93001-EIS_V0.2013-V92`, USB on the dev Mac, now
`/dev/cu.usbmodem246802461`) dials our answerer through the Grandstream
ATA, which presents it as **6005** ("ATA L2"), not 6004.  Our server on
vmhost CT 117 answers as 2904 at the factory `+ES` (V.42 detection, then
MNP, then V.14).  Recipe: `tools/soak/answer_ec_call.sh` on CT 117,
`tools/cx_hold_dial.py <tty> 2904 35 <setup>` here.  `+MS=V22B` on the CX
(it sends no CM, so V.8 fails and we reach V.22bis through USB1).

* **`AT\N2` on this CX is not MNP-only.**  It sends V.42 ODP and, if no ADP
  comes, MNP LRs every 4.06 s.  `+ES=4,4` is MNP only (accepted;
  `+ES=?` is `(0-4,6,7),(0-4),(0-6,8,9)`).
* **Our answerer's T400 was being eaten by our own buffering** (fixed
  here).  First call: the CX's ODP started 0.5 s into the data phase, we
  declared "no error-control peer" at 0.75 s, went to V.14, and ignored the
  ODP and all seven LRs that followed (`CONNECT 2400/V22B`, `+ER: NONE`).
  The bits were clean: replayed into the data stack they detect ODP at
  0.39 s.  The difference is that SpanDSP clocks T400 on transmitted bits,
  and our transmit runs 1214 bits (506 ms) ahead of our receive -- measured
  as the gap between the fourth received DC1 and the start of our ADP on the
  next call.  7.2.1.3 has the answerer examine "the incoming bit stream",
  so `data_stack.c` now counts the answerer's T400 on received bits
  (`ds_arm_t400()`), with the transmit-clocked timer as a 10 s backstop.
  `data_stack_test` checks both directions (ODP 0.5 s in with transmit 506
  ms ahead: detected; marks only: fallback after exactly 750 ms of received
  bits); both checks fail on the old code.  The originator's T400 (waiting
  for ADP) still runs on transmitted bits.
* **With that, `\N2` gives `CONNECT 2400/ARQ/V22B/LAPM/V42BIS`** at the
  default T400, 20 of 20 lines each way (it took `ME_V42_T400_MS=3000` on the
  call before the fix).
* **MNP answerer, live: works, then the call drops.**  CX at `+ES=4,4`:
  `+ER: ALT`, `CONNECT 2400/ARQ/V22B/MNP`, the CX's `&V1` says `PROTOCOL
  MNP`.  Its LR: `02 | 01 06 01 00 00 00 00 ff | 02 01 03 | 03 01 08 | 04 02
  40 00 | 08 01 02 | 09 01 05` -- k 8, N401 64, fixed-field LT/LA but not
  the 256-octet facility, and a parameter 9 we do not know, which our
  response LR leaves out (A.7.1.5.6); the CX accepts that and runs without
  compression.  LTs and LAs then run cleanly both ways (5 lines each way,
  every LA advancing).  All three MNP calls (cx4-cx6) then ended 8-10 s into
  data, including one where we sent nothing: the CX's signal falls to line
  noise for ~30 ms (cx5: it comes back at full level), our V.22bis receiver
  reports carrier down, and the engine hangs up at once.  The LAPM calls
  never had such a gap; why the CX makes one in MNP mode is not known.  The
  fault on our side is that we ignore **S10** (V.250 carrier-loss disconnect
  delay, factory 1.4 s; the CX itself is at S10=14): `at_help.c` lists it as
  "stored only", and a post-training carrier drop restarts SpanDSP's whole
  V.22bis datapump (`v22bis_rx.c`).  Tolerating such a gap needs the trained
  datapump held through it, which is datapump work across modulations, not
  an MNP change.
* One spurious go-back on the answerer: with no round-trip measurement yet
  (only the originator samples one from the LR exchange), the CX's
  pre-data LA repeating N(R)=0 resent our first LT once.  Harmless (the copy
  is the first repeat of the last LT, A.7.3.2.2), not changed.

Still not verified:

1. Our 3-try LR budget after a failed V.42 detection.  The RasFinder runs
   LAPM, so a factory `+ES` call never reaches MNP against it; the c3 window
   above says a peer can stop listening within 4 s.
2. Whether a peer minds the LA-per-out-of-sequence-frame volume: no call here
   had an out-of-sequence LT.
3. MNP over V.32bis (V.32bis does not train against the RasFinder at all,
   `rasfinder_modulation_tests_20261020.md`), and an MNP answerer call that
   lasts: every CX MNP call was cut by the S10 gap above.

The rigs that could answer these: the Courier (`&M4`, `&M5` with MNP) and
MICA (`MNP 2-5` in its banner).
