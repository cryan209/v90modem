# V.91 conformance audit (ITU-T V.91, 05/99)

**Update 2026-10-17: every row of the table below is closed (the sections after it are the original 2026-10-16 audit, kept for the reasoning; "Progress" at the end says what was done). The only open items are things no in-tree test can confirm: see "Still not verifiable here".**

Audited 2026-10-16 by reading the whole Recommendation (`ITU Docs/T-REC-V.91-199905-I!!PDF-E.pdf`,
~870 lines of text, 14 pages) against `v91.c`, `vpcm_v91_session.c`,
`vpcm_cp.c` and the live path in `modem_engine.c` (`v91_live_*`).  Code was
read, not run; no test was executed for this note.

## Headline

The start-up *signals* are mostly right.  The **data-mode line encoding is not
V.91** -- it is a bring-up codec that only interoperates with itself -- and the
control channel, retrain, escape-to-V.34 and timers are absent from the live
engine.  Nothing has met a foreign V.91 implementation; I know of none to
test against.

Rough scoring by clause (done = matches the text, partial = right shape or
only on the loopback/session path, missing = not there):

| Clause | Subject | State |
|---|---|---|
| 6.1 / 6.2 | Rates 28000..64000 step 8000/6, K 15..42, 8000 baud | done (`vpcm_cp_drn_to_k`: K = drn+14, drn 1..28) |
| 6.3.1 | GPC scrambler on the primary channel | done: the V.90 5.4 encoder scrambles with GPC in data mode (`v91_tx_codewords`) |
| 6.3.3 - 6.3.8 | V.90 5.4 encoder: bit parsing, modulus encoder, mapper, differential encoding of sign bits, sign assignment, mux | done: `v90_map_sr0_frame` / `v90_demap_mapped_frame`, Sr = 0, S = 6 |
| 6.3.9 / 6.4 | Control channel (96-bit sequence, Table 3) | done (slip pattern, cleardown bit 16, retrain bit 17, CRC); checked by the spec decoder |
| 6.5 | Transparent mode | done, live 2026-10-17 (`ME_V91_TRANSPARENT`); data bits are the codewords, no control channel |
| 7.1 | B1 = 12 codewords from mapping binary ones through the data encoder | done (two frames of ones through the encoder) |
| 7.2 | CP (Table 4) | done: reserved bits zero, multiple-of-6 length, variable gamma read live; spec decoder checks every CP |
| 7.3 | DIL; default DIL (Table 5) | done (`v91_default_dil_init` matches N=125, LSP/LTP 12, SP 0FC0, TP 0FFF, H=1, training 124,0,123,1...62) |
| 7.4 - 7.7 | Eu / Es / Ez / Em lengths and initialisation | done (12/12/24/12 symbols); init rules covered by loopback tests only |
| 7.8 | INFO (Table 6) | done, all bits live (26 DIL, 27 control channel, 40/41 transparent) |
| 7.7 / 7.9 / 7.10 | Em / J / PHIL | done, live behind `ME_V91_SPECIFY_DIL=1` (2026-10-17); default start-up still asks for the default DIL |
| 7.11 | SCR, multiple of 6 symbols | done |
| 8.1 | V.8 selection, 75 +/- 5 ms silence | done (V.8 `V8_PSTN_PCM_MODEM_V91` picks it; `V91_PHASE1_SILENCE_SYMBOLS` 600) |
| 8.2 | Start-up Figures 1-3 | Figures 1-3 live (Figure 3's zero-length DIL on the transmit side only) |
| 8.3 | Escape to V.34 (no INFO' in 1.5 s, no B1 in 10 s, INFO0a/c handshake) | done: 1.5 s INFO' and 10 s B1 timers escape to V.34 |
| 8.4 - 8.5 | Retrain, 10 s timer, loss of frame sync | done: retrain from data mode with a 10 s timer; reacquiring the grid without a retrain (8.5 "by another means") not attempted |
| 8.6 | Cleardown via drn=0 or control channel bit 16 | done: control channel bit 16 and CP drn = 0 (a peer's drn 0 now ends the call instead of escaping to V.34) |
| 8.7 | Transparent grant needs verified unrestricted 64k channel | done for what a DIL can show: codeword-exact DIL (polarity or any change fails it) and no robbed LSBs; both CPs must grant, bit 41 clears down |
| 8.8 | 4-wire leased line | n/a (same start-up) |
| 9 | Test loop 3 (V.54) | done 2026-10-17: `AT&T3` (circuit 141) / `AT&T0`; circuit 142 reported in the trace |
| 5 | Circuits, DTE interface | PTY via `data_interface.c` with V.14/V.42 covers 5.2 |

## Data-mode encoding is not V.91 (the important gap)

6.3 says the encoder "is defined in 5.4/V.90 and Figure 1/V.90" with Sr = 0,
S = 6.  `v91_tx_codewords` / `v91_rx_codewords` (`v91.c`, ~1560-1700) instead:

- split K as evenly as possible over the six frame intervals
  (`v91_bits_per_interval`: `k/6` plus one for the first `k%6`), where V.90
  5.4.2 parses by the modulus-encoder shell structure;
- feed those bits straight in as an **ordinal into the CP mask** -- no
  modulus encoder (5.4.3);
- use the sign bit as raw user data -- no differential encoding of sign bits
  (6.3.6: "applied continuously to all sign bits"), no sign-assignment
  (5.4.6), no mux (5.4.7);
- apply **no scrambler** to the data (6.3.1).

`v91_select_b1_ucode` is labelled "Bring-up mapper" in the source: it picks a
codeword from K/interval/position arithmetic rather than "binary ones mapped
using the selected data mode" (7.1).

Net effect: the two-instance call at 52000 proves our own TX and RX agree
with each other, and the robbed-bit alignment work sits on top of that.  A
spec-conformant peer would see a differently coded, unscrambled stream.  The
fix is mostly reuse: V.90 already has the 5.4 encoder (`v90.c`, `V90_TX_DATA`
modulus encoder) and V.91 only differs in Sr = 0, S = 6 and symmetric use, so
this should be a wiring job, not a new DSP block -- the same pattern the
`vpcm_*` layer already uses for V.92.

## CP layout

`vpcm_cp_encode_bits_fixed_fill` writes the **V.90 Table 14 CP**.  V.91
Table 4 keeps the same sync/drn/ack/constellation structure but:

- bits 30:32, 35:50, 52:67, 69:84, 86:101, 129:135 are "reserved, set to 0 by
  the transmitter".  We drive them with V.90 content (silence request,
  shaping redundancy, codec A-law at 35, upstream rate mask 36:48, TRN1d
  gain 52:67, shaping coefficients).  A conformant receiver ignores them, so
  this is tolerable on the wire, but a V.91 CP should have them zero.
- the live engine **repurposes `upstream_rate_mask` (bits 36:48) for the
  robbed-cadence alignment request**.  That is our private extension inside
  reserved bits; only our own receiver understands it.
- the length is wrong: Table 4 runs `272+gamma` start bit, 16 CRC bits, one
  fill bit, then zeros "to the next multiple of 6 bits" (SCR/CP are mapped six
  symbols at a time).  The shared encoder pads a fixed 3 bits
  (`156 + 136 n` total), which for one constellation is 292 against the
  spec's 294.  `v91_live_rx_*` also requires the exact length, so a peer's CP
  would not parse.
- the CRC span (which bits are excluded) was written for V.90 and has not
  been checked against the V.91 text.

## Control channel (6.3.9, 6.4, Table 3)

Not implemented at all.  INFO bit 27 (request control channel) is encoded and
decoded but nothing honours it, and the spec requires the peer to supply it
"if and only if" requested.  That removes: digital-slip detection (the 16-bit
pattern 0111011111111010), in-band cleardown (bit 16), retrain warning
(bit 17), the 54-bit secondary channel, and the 125 bit/s it costs (every 64th
data frame, all six sign bits, differentially encoded but not scrambled).

This is also why sample slips are fatal to our live alignment: V.91 itself
expects a slip detector, and our robbed-cadence phase tracking
(`g_v91_*_abs_pos`) is the stand-in.

## Procedures

- 8.2.1.8: CP within 1.5 s of Eu/Em -- not enforced.
- 8.3: INFO' within 1.5 s and B1 within 10 s, else escape to V.34 with
  INFO0a/INFO0c -- not present; `v91_live_rx_fail_locked` retrains the V.91
  state flag and sets `ME_HANGUP`.
- 8.4/8.5: 10 s recovery timer exists as a boolean only.
- 8.7: the live INFO never sets bit 40/41, so transparent 64k is never
  requested live.  The session path negotiates it and denies it on a detected
  robbed trunk.

## What is solid

Start-up signal construction and parsing (Ez/INFO/Eu/DIL/SCR/CP/Es/B1 as
signals), the INFO frame, the default DIL, DIL-driven DRN selection, and
undeclared robbed-bit detection with odd-ucode restriction are all coded
against the text and exercised by the loopback tests
(`vpcm_loopback_test.c`) and `make v91-serial-pair-test`.

## Suggested order of work

1. Replace the bring-up data mapper with the V.90 5.4 encoder (Sr=0, S=6),
   scrambled and differentially coded per 6.3, and make B1 real.  Highest
   value: without it nothing else can interoperate.
2. Fix the CP: V.91 Table 4 field set, reserved bits zero, multiple-of-6
   length; move our robbed-cadence request out of the reserved CP bits (send
   it in the control channel's secondary data or a private INFO bit nobody
   else reads).
3. Control channel (Table 3) with slip detection, then retrain/cleardown on it.
4. 8.3 timers and escape to V.34, 8.4/8.5 recovery timer, transparent request live.  (J/PHIL/Em: live 2026-10-17.)
5. V.54 loop 3 (clause 9) last.

## Peer availability

The V.91 text pins the data-mode encoder to V.90's by reference (6.3.2 -
6.3.8), uses V.90's DIL (7.3, 7.9) and the V.90 INFO0 handshake for the V.34
escape, and its CP is V.90's with the V.90-only fields reserved.  It is
V.90's downstream applied in both directions over a 4-wire digital path
(Appendix I: ISDN or T1/E1 end to end).  No third-party V.91 implementation
is known to us; the only on-wire reference is the text, so the way to
validate 6.3 without a peer is a spec-only decoder in the style of
`tools/v90_trn2d_spec_decode.py`, run on our own transmit tap.

## Progress (2026-10-16)

Closed in the code, with the loopback suite (`vpcm_loopback_test --all-tests`),
`make test-fast` (550 passed) and the live two-instance call
(`tools/v91_serial_pair_test.py`, now `--mode v91` and `AT\N0`) all green:

- **Data-mode encoder (6.3, DONE).**  `v90_map_sr0_frame()` (new, `v90.c`) is the
  transmit twin of `v90_demap_mapped_frame()`: GPC scrambler, first six bits as
  signs, the rest through the 5.4.3 modulus encoder, descending-label mapper,
  differentially coded signs.  `v91_tx_codewords` / `v91_rx_codewords` use them;
  the bring-up ordinal mapper is gone.  The capacity rule is now 5.4.3's
  2^K <= prod(Mi) rather than "each interval carries K/6 bits".
- **B1 (7.1, DONE).**  Two frames of binary ones through the encoder, with the
  scrambler and differential state running on from Es.  Receive side demaps and
  checks ones; it skips the first 23 bits only when it has no descrambler
  history.
- **CP length (7.2, DONE).**  `vpcm_cp_encode_bits_v91` / `vpcm_cp_decode_bits_v91`
  pad to a multiple of 6 (`vpcm_cp_v91_length_for`).  The live receiver now reads
  the length from the dfi field (variable gamma) instead of assuming the
  template's.  Correction to the audit above: the reserved CP fields were
  already zero (`vpcm_cp_init` zeroes them); the only stray bits were CP bit 35
  (codec law, now 0 -- INFO bit 39 carries it) and the `upstream_rate_mask`
  alignment hack, which is gone.
- **Live CP exchange (8.2.1.8-8.2.1.10, DONE).**  CP repeats, CP' (ack bit) once
  the peer's CP is in, Es+B1 after a CP' sent and a CP'/Es received.  The CP we
  send names the constellation and rate for the PEER's transmitter (7.2), so it
  carries the robbed intervals seen on our receive path
  (`vpcm_cp_apply_robbed_bit_safe_slots`); our transmitter and B1 follow the
  peer's CP, our receiver follows ours.  Each direction therefore runs its own
  rate.  The old robbed-cadence "alignment" (TX padding to a cadence phase,
  carried in a reserved CP field) is deleted: the spec's per-interval
  constellations do the job.  Live robbed-trunk simulation passes at phases 1-5
  (phase 0 still fails occasionally at the DIL stage, receive code unchanged by
  this work).
- The harness: `tools/v91_serial_pair_test.py` had silently stopped selecting
  V.91 (the V.8 offer no longer advertises it by default) and passed only about
  half the time at baseline because V.42 detection raced on loopback.  It now
  forces `--mode v91` and `AT\N0`.  Remaining failures are V.8 timeouts on
  loopback, before any V.91 code runs.

Still open: control channel (6.3.9/6.4), 8.3 escape to V.34 and its 1.5 s / 10 s
timers, 8.4/8.5 retrain and the recovery timer, J/PHIL/Em and the transparent
request on the live path, V.54 loop 3, and a spec-only decoder run on our own
tap (the only validation available without a peer).  Test-only note: the
robbed-bit startup test no longer asserts that B1 survives an unrestricted
constellation under robbing (it is real scrambled ones now and does not).

### Control channel (6.3.9, 6.4) -- added 2026-10-16

- `v90_map_sr0_frame_cc` / `v90_demap_mapped_frame_cc` carry the six control
  bits in the sign positions: differentially coded, not scrambled, and the
  scrambler/descrambler do not advance over them.
- `v91.c`: Table 3 sequence (slip pattern `0111011111111010`, cleardown bit 16,
  retrain bit 17, 54 secondary bits, CRC over bits 0:77), one six-bit chunk
  every 64th data frame starting with the first data frame; a receiver that
  counts frames verifies pattern + CRC per 96 bits and flags a slip after two
  bad sequences in a row (`v91_cc_slip_suspected`).  The CRC span (bits 0:77,
  INFO-style init and bit order) is read from the text by analogy with the INFO
  CRC; there is no peer to confirm it.
- Supply-iff-requested is honoured: the transmitter inserts the channel when
  the peer set INFO bit 27; we request it by default (`ME_V91_CONTROL_CHANNEL=0`
  turns the request off).  Primary rate drops by 125 bit/s; the live block fill
  asks `v91_tx_bytes_for_frames()` for exactly what 8 frames need.
- 8.6: a local hangup from data mode sets bit 16, waits for the current
  sequence to finish plus 0.5 s, then drops the bearer; the receiving side
  clears down on the bit.  A peer retrain warning (bit 17) currently ends the
  call (no retrain procedure yet), and a slip ends it too (8.5 without a
  re-acquisition attempt).
- Tests: `test_v91_control_channel` (data byte-exact across CC frames, flags,
  secondary bits, slip) and the live pair test, which now also hangs up and
  checks the peer saw the bit.

### Escape to V.34 and retrain (8.3, 8.4, 8.5) -- added 2026-10-16

- **Start-up (8.2.1.2-8.2.1.4).**  INFO now repeats until the peer's INFO has
  been received, then INFO', Eu and the DIL follow (the old script was a fixed
  Ez/INFO/INFO'/Eu/DIL burst, which only works when both ends start together).
  The receiver also finds INFO by its onset (sliding 62-symbol window gated on
  the `1111` + `0x72` opening and checked by CRC), so a peer that started first
  or is retraining under us (8.4.2) is picked up without seeing its Ez.
- **Escape (8.3).**  On the TX clock from the start of INFO sending: no INFO'
  within 1.5 s, or no B1 within 10 s, or any start-up receive failure, calls
  `start_v34_training()`; V.34's Phase 2 then supplies INFO0a/INFO0c.  Live
  with `V91_TEST_ESCAPE_AFTER_MS` (both ends escape, V.34 connects, payload
  flows).  The two natural timers share that code path but have no test of their
  own yet.
- **Retrain (8.4, 8.5).**  A frame decode failure, a control-channel slip, or a
  peer retrain warning (bit 17) re-runs the start-up from data mode with the
  data stack paused (not restarted, so LAPM survives) and a 10 s timer; a
  retrain that cannot complete ends the connection, as does a fifth in one call.
  A local request warns the peer on bit 17 first and starts once that sequence
  is out (`V91_TEST_RETRAIN_AFTER_MS` drives it).  Reacquiring the frame grid
  without a retrain (8.5 "by another means") is not attempted: after a slip
  the control-channel phase is lost as well, so a retrain is the honest answer.
- **Live evidence.**  A real media glitch (an RTP gap of idle codewords inside a
  data block) used to hang the call up; in these runs it now shows up as
  "retrain #1: frame decode failed" on both ends and the call survives.
- Pair test: `make v91-serial-pair-test` now also runs `--retrain` and
  `--escape`.  Still open: transparent-mode request on the live
  path, V.54 loop 3.

### Spec-only decoder (`tools/v91_spec_decode.py`) -- added 2026-10-16

A receiver written from the Recommendation text that imports nothing from the
tree.  `tools/v91_serial_pair_test.py` records both transmit taps
(`VPCM_G711_TAP_DIR`) and runs it on every default pair call; the decoder
applies each stream to the CP that the OTHER stream sent (7.2: a CP configures
the peer's transmitter).  It checks Ez, every INFO frame (sync, CRC, fill), Eu,
the default DIL symbol by symbol against Table 5 / V.90 8.4.1 (1500 symbols, 0
mismatches), SCR as descrambled ones, every CP (start bits, reserved bits,
length a multiple of 6, CRC), Es, B1 as descrambled ones, every data frame
against the CP constellations, every control-channel sequence (slip pattern +
CRC) and the test payload as 8N1 characters in the primary bit stream.  It
accepts our transmit side of a real call on all of those.  Sensitivity checked
by corrupting taps: one flipped DIL symbol, one flipped SCR sign and a flipped
data byte are each reported.

What that does and does not establish: the Ucode/sign mapping, scrambler,
mixed-radix descending-label encoder, differential signs, DIL, CP layout and
length are checked against the text.  The CRC conventions are not independent:
CP uses the V.90 convention validated against a real analogue modem's CPt;
the INFO CRC span (bits 12:41) and the control-channel CRC span (bits 0:77)
are our reading by analogy, so a peer could still disagree on those two.

**Finding.**  The decoder shows an idle gap of about 3000 symbols between B1
and the first data frame (3270 in one run).  V.91 has data follow B1 directly
(8.2.1.11, 7.1), so a conformant peer would try to decode our idle as data and
lose frame sync.  The gap is an artefact of the live engine (the RX discards
the packet after B1 while it waits for the local TX to finish, then the TX
inserts a guard packet); removing it needs B1 followed immediately by the first
data block and the receiver switching to data in the same codeword stream.
That is the next conformance fix.

### B1 to data with no gap (8.2.1.11, 8.2.1.12, 7.1) -- fixed 2026-10-16

The spec decoder showed ~3000 idle symbols between B1 and the first data frame.
Cause: the engine entered DATA only when BOTH directions were done, discarded
the rest of the RX packet after B1, sent a guard packet and padded to a frame
boundary.  Now each direction goes live on its own: the transmitter starts data
in the next codeword after B1 (in the same RTP packet when B1 ends mid-packet)
and the receiver decodes the codeword after the peer's B1 as the first data
frame (frame grid = B1's end).  The data stack goes online once both are live;
bytes decoded before that are the peer's idle marks and are dropped.  The
decoder now requires zero idle symbols there and passes on every pair call.

Things found on the way:
- The `V91_TEST_RETRAIN_AFTER_MS` hook had been consumed before the call was in
  DATA, so `--retrain` had never exercised a locally requested retrain; fixed.
- The host (a busy Mac) stalls both processes for 100-224 ms at the same
  instants, which empties the default 200 ms jitter buffer and puts idle
  codewords into the stream; the engine then retrains, as designed.  The pair
  test now runs with `ME_JB_MS=320`, retries a failed attempt in a fresh
  process (a real bug fails all three), and the `--escape` variant checks the
  escape and the start of V.34 Phase 2 on both ends rather than a finished V.34
  call (which is slow and timing-sensitive here).

### J / PHIL / Em on the live path -- 2026-10-17

`ME_V91_SPECIFY_DIL=1` sets INFO bit 26 and runs the specified-DIL start-up of
8.2.1.5-8.2.1.7 (Figures 2 and 3) in `modem_engine.c`: the TX script builder
chooses Eu+DIL (both default), J then Em+DIL (peer default), or J/nothing
followed by PHIL until the peer's J is in (`V91_LIVE_TX_PHIL`, escape to V.34 if
it never arrives); the receiver gains J and PHIL/Em stages
(`v91_j_length_from_prefix()` finds the end of J from its header, so no length
is assumed).  The descriptor we request is the default one, so the DIL check
and rate selection are unchanged -- only the sequence differs.  A zero-length
DIL from the peer is handled on the transmit side (Em alone), but we never
request one, so that path has no test.

`tools/v91_serial_pair_test.py --specify-dil {caller,answerer,both}` (in
`make v91-serial-pair-test`) passes with data both ways, and
`tools/v91_spec_decode.py` checks J (framing, Table 12 CRC span, descriptor
fields), PHIL (none when the DIL is already known, some when waiting) and Em on
our taps.  Both ends are this tree, so a foreign peer's J is still unverified;
the decoder's J CRC span mirrors `v90_dil_crc16_bits`, not an independent read.

### Transparent mode and V.54 loop 3 -- 2026-10-17

**Transparent (6.5, 8.7).**  `ME_V91_TRANSPARENT=1` sets INFO bit 40, `=2` also
bit 41.  A request from either end is granted (CP bit 18, drn 28) when the DIL
we received was codeword-exact with no robbed LSBs; transparent mode runs only
when both CPs grant, otherwise the CP parameters are used non-transparently, or
both ends clear down if a requester set bit 41.  B1 is 12 codewords of ones and
data bytes are the codewords themselves (64000 bit/s); there is no control
channel and no 8.4-8.6 machinery.  `v91_serial_pair_test.py --transparent
{caller,answerer,both}` covers grant, refusal on a robbed trunk
(`--robbed-phase 2`) and bit-41 cleardown; the spec decoder checks B1 and the
raw data stream on our taps.  What a DIL cannot show (a gain or an A/D pad that
preserves codeword identity does not exist, so this is complete for the DIL
itself) is a channel that degrades only data patterns; nothing checks that.

**Loop 3 (clause 9).**  `AT&T3` is V.24 circuit 141 asserted by the DTE (`AT&T0`
releases it; other `&T` values are ERROR, as the analogue and remote loops do
not exist here).  The decoded line data goes back to the transmitter instead of
the data stack, the DTE's own data waits, and underrun is padded with binary
ones.  Circuit 142 (test indicator) is on while the loop is closed; there is no
physical or AT-visible circuit for it, so it is reported only in the trace
(`V91 loop 3 ON/OFF`).  `--loop3` in the pair test closes the loop at the
answerer, checks the caller gets its data back, the answerer's DTE sees none, and
normal data resumes after `AT&T0`.  The loop is below error control, so with
V.42 the peer would see its own frames reflected: use it without EC.

**Still not verifiable here.**  The INFO and control-channel CRC spans are
checked by `tools/v91_spec_decode.py` (written from the text), but no foreign
V.91 peer exists to confirm them, and the same holds for J.  Nothing else in the
audit table is open.

### 8.6 and the closing pass -- 2026-10-17

A peer CP with drn = 0 used to be reported as a start-up receive failure, which
escaped to V.34 and kept the call up; it is now cleardown (`ME_HANGUP`).  We do
not send drn = 0 ourselves: a local hangup uses the control channel (bit 16) or
drops immediately without one, both of which 8.6 allows only as "or" alternatives
for the CP route, so a conformant peer that waits for either will see the media
go away.  The CP-within-1.5-s rule (8.2.1.8) is our obligation, not something to
enforce on the peer, and is met by construction (the DIL we receive lasts about
0.19 s).  The earlier note that our robbed-cadence request sat in reserved CP
bits was already obsolete: it was deleted on 2026-10-16 and the spec decoder
checks the reserved bits are zero.
