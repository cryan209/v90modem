# V.34 fax: T.30 Annex F, and the V.34 half-duplex layer under it

V.34 fax is two layers, and they are in very different states:

- **ITU-T V.34 clause 12**, the half-duplex modem (control channel plus
  primary channel, with source/recipient turnarounds).  Partly implemented,
  and the subject of this note.
- **ITU-T T.30 Annex F**, the fax procedures carried over it.  **Absent** --
  `t30.c` is 7658 lines with zero V.34 references.

## The normative sources, and the one that is missing

Everything in clause 12 is in the local spec, `ITU Docs/T-REC-V.34-199610-S`:
6.3 and 6.6.2 (half-duplex interfacing and circuit 109), 10.2 (the signals --
INFOh Table 22, MPh Tables 23 and 24, PPh, Sh, ALT, E, AC, and the control
channel modulation of 10.2.4), and clause 12 itself (12.2 Phase 2, 12.3
Phase 3, 12.4 control channel start-up, 12.5 primary channel resync and
turn-off, 12.6 control channel turn-off and parameter change).

**That was true when this note was written and is not any more.** The fax
service class work put T.30, T.31, T.32 (+ Amd 1), T.4 and T.6 in `ITU Docs/`.
Annex F is at `T-REC-T.30-200509-I!!PDF-E.pdf`, clause F.1 to F.5, and it is
short. Its substance:

- **ECM is mandatory** for every V.34 fax message (F.3).
- **No TCF.** After DCS the source sends control channel flags and waits; the
  recipient answers CFR, and **FTT is not used** (F.3.2.1).
- T.30's binary procedural data goes on the **control channel**, the message
  and RCP on the **primary channel** (F.3.1.3).
- After 12.4/V.34's control channel start-up both terminals send HDLC flags
  and receive HDLC frames at the rate MPh settled, at least two flags before
  the first frame after any start-up, resync or retrain (F.3.1.4).
- The turnarounds are counted, not timed: the recipient sends flags until it
  sees **40 consecutive 1s** then goes silent; the source sends 1s until the
  flags stop and at least 40 have gone, then **70 +/- 5 ms** of silence, the
  primary channel resync signal, T.4 A.3.1 sync, and the message
  (F.3.2.2/F.3.2.3, and again per page at F.3.4.4/F.3.4.5).
- Phase D: primary channel turn-off, then control channel resync **or**
  start-up if the rate is to change; the post-message command follows on the
  control channel (F.3.4.1-F.3.4.3).

So the Annex F layer no longer lacks a normative source. What it lacks is the
control channel underneath it.

## What was already here

More than a first look suggests.  The half-duplex TRANSMITTER is largely
built: `infoh_t`/`mph_t`, `prepare_infoh()`, `infoh_sequence_tx()`,
`mph_sequence_tx()`, `get_pph_baud()`, `sh_baud_init()`,
`first_alt_baud_init()`, `second_alt_baud_init()`, and a full set of
`V34_TX_STAGE_HDX_*` stages, every one of which has a handler.  The receiver
has `process_rx_infoh()` and branches on `duplex` in a dozen places.

What was missing was the wiring between them, and it is measurable now:
`v34_hdx_test` drives a source/recipient pair at each other over G.711 and
prints how far each side's transmitter and receiver got.

```bash
make v34_hdx_test && ./v34_hdx_test 3200 9600 ulaw 20
V34_HDX_LOG=1 ./v34_hdx_test ... 2>&1 | grep -oE "(Tx|Rx) - [a-z0-9_]+_init\(\)"
```

## Five defects, found in that order

1. **`half_duplex_state` was a `bool`** holding
   `V34_HALF_DUPLEX_CONTROL_CHANNEL`, `_PRIMARY_CHANNEL` or `_SILENCE` -- 2, 3
   and 4, which all collapse to `true`.  No reader could have told the three
   apart, so the public `v34_half_duplex_change_mode()` was unimplementable as
   typed.  Both it and `half_duplex_source` are now `int`.  (Nothing reads
   `half_duplex_state` yet; that is the turnaround logic, still to come.)

2. **`mp_or_mph_baud_init()` logged `"Tx - mp_baud_init()"` from both
   branches**, so every half-duplex trace read as though the duplex 11.4 MP had
   run when the 12.4 MPh branch had.  Fix the instrument before trusting it:
   this one cost a wrong diagnosis before it was noticed.

3. **Phase 3 exited into J.**  The TRN handler's exit was unconditional
   `s->tx.stage = V34_TX_STAGE_J`.  12.3.1.3 has the half-duplex modem proceed
   to the control channel of 12.4 after TRN; J belongs to the duplex Phase 3 of
   11.3 and does not exist in half-duplex.  The whole 12.4 chain
   (`pph_baud_init` -> `second_alt_baud_init` -> `mp_or_mph_baud_init`) was
   therefore unreachable, and **PPh and ALT were never transmitted at all**.
   Now `hdx_control_channel_start_init()` runs 12.4.1.1's 70 +/- 5 ms silence
   and then PPh.  Note `tx_silence()` counts `tone_duration` in SAMPLES and
   never calls the getbaud, so the follow-on is a flag it checks, in the same
   shape as the existing `training_stage == 0x100` hand-off.

4. **TRN length was computed from the symbol rate INDEX.**  Table 22 bits 15:21
   give the TRN length in 35 ms increments and bits 27:29 give the symbol rate
   as an index between 0 and 5; the code multiplied by the index.  Worse,
   `prepare_infoh()` set `baud_rate = 14`, which does not fit a 3 bit field at
   all -- it truncates to 6, one past the last legal index.  Together the
   source trained for **1 baud**.  The index now comes from the negotiated
   symbol rate, and the rate table access is bounded.

5. **`V34_RX_STAGE_INFOH` was never assigned anywhere in the tree.**  The
   receiver could not enter the stage that decodes INFOh, so INFOh was
   transmitted every call and never once received -- which is why the source's
   `rx.infoh` was all zeroes and item 4 had nothing to read.  10.2.2 replaces
   INFO1a and INFO1c with INFOh in half-duplex, and the receiver already knew
   its 51 bit length in two places; only the stage assignment was missing.

## The Phase 3 sequencing, and the role inversion behind it

12.2.1.2.6 and 12.2.2.1.6 both end with the same sentence -- "After sending
INFOh, the modem proceeds according to **12.3.2**" -- and 12.2.1.1.4 and
12.2.2.2.4 both end with "After receiving INFOh, the modem shall proceed
according to **12.3.1**".  So the rule is role-based and does not depend on the
call direction at all:

- **whoever SENDS INFOh is the recipient**, and per 12.3.2.1 transmits silence
  and conditions its receiver to detect S followed by S-bar;
- **whoever RECEIVES INFOh is the source**, and per 12.3.1.1 transmits silence
  for 70 +/- 5 ms, then S for 128T, S-bar for 16T, then PP, then TRN.

`get_infoh_baud()` branched on `calling_party` instead, which is not the same
thing, and the two roles came out **swapped**: the answerer called
`s_not_s_baud_init()` and transmitted S/S-bar/PP/TRN, which is the source's job
under 12.3.1, while the caller was given `tx_silence_init(s, 30000)` -- thirty
seconds of silence -- and never started Phase 3 at all.  Three more defects sat
behind that:

- **`V34_EVENT_INFOH_OK` was never raised anywhere in the tree.**  It is
  defined, it has a name string, and `V34_TX_STAGE_HDX_POST_L2_B` waits on it
  (12.2.1.1.4) -- but the receiver raised `V34_EVENT_INFO1_OK` on decoding
  INFOh, so the source could never leave that stage.
- **`V34_TX_STAGE_HDX_POST_L2_SILENCE` was a dead end.**  It counted out
  12.3.1.1's 75 ms and then did nothing, so even a source that got there would
  have stopped.  It now calls `s_not_s_baud_init()`.
- **INFOh was sent on a stopwatch.**  12.2.1.2.6 says "After Tone B is
  detected, the answer modem continues transmitting Tone A for 25 ms, then
  sends INFOh", and the code had a flat 100 baud timer with
  `//if (s->rx.received_event == V34_EVENT_REVERSAL_2)` commented out beside
  it.  INFOh therefore went out whether or not the call modem had reached
  12.2.1.1.4 and conditioned its receiver to receive it -- and it had not, so
  INFOh was transmitted once, into a peer still hunting Tone A, and was never
  received.  Now gated on `rx.tone_b_present`, bounded by 12.2.1.4.3's 2000 ms
  rather than the old 167 ms.

One more thing was in the way of reading any of this: **the stage enums do not
start at zero.**  `v34_rx_stages_e` starts at 1, so a numeric stage trace is
off by one against a naive extraction, and `V34_RX_STAGE_INFOH` is 2, not 0.
An early reading of a trace here was wrong for exactly that reason.
`V34_TX_STAGE_HDX_CC_SILENCE` is appended at the end of the tx enum so the
12.4.1.1 silence is distinguishable from 12.3.1.1's, which shares neither its
meaning nor its follow-on.

## Phase 2 receiver conditioning: INFO0 was being read as INFOh

Three defects, and the first is the root of the other two.

1. **`info0_target_bits()` returned INFOh's length for INFO0.**  It read
   `(s->duplex) ? 49 - 16 : 51 - 16`, and `v34_rx_restart()` conditioned the
   receiver the same way on the line immediately after setting the stage to
   `V34_RX_STAGE_INFO0`.  But 10.2.2 says the half-duplex Phase 2 signals "are
   identical to those specified in 10.1.2, **except that INFO1a and INFO1c are
   replaced by INFOh**" -- INFO0 is not replaced, and Table 7 makes it 49 bits
   (0:48) in both modes.  The receiver therefore read two bits too many and the
   INFO0 CRC failed on a bit-exact loopback, on every call.

2. **The INFO0 single-bit-error recovery hid it.**  With the CRC failing, that
   path flipped each bit in turn looking for one that made the CRC zero, found
   **bit 33** -- the first bit past the true 33-bit payload -- and declared
   success.  Both modems did this on every call:
   `Rx - INFO0 single-bit recovery: flipped bit 33, CRC now 0`.  A deterministic
   "single-bit error" at a fixed position on a clean loopback is not noise; it
   is a length error being papered over.

3. **That recovery path raised `V34_EVENT_INFO0_OK` and returned WITHOUT
   setting the stage.**  The ordinary INFO0 path sets
   `stage = calling_party ? TONE_A : TONE_B` alongside raising the event
   (11.2.1.1.2 / 12.2.1.1.2: condition the receiver for the far tone and its
   phase reversal).  The recovery path did not, so the transmitter moved on
   while the receiver was still hunting INFO0 and never reported the Tone A
   phase reversal.  This is a real bug in its own right -- it would do the same
   on any line where the recovery legitimately fires, in V.34 and V.90 too.

A fourth, in the transmitter: **12.2.1.1.4 is one clause with two actions** --
"the call modem shall transmit Tone B **and condition its receiver to receive
INFOh**" -- and only the first was done.  Nothing else could do it here: the
duplex receiver arms itself for INFO1a on the *third* Tone A reversal, and
half-duplex 12.2.1.2 has only **one** (12.2.1.2.3), so that trigger can never
fire on this path.  New `v34_condition_rx_for_infoh()`, called from
`second_b_baud_init()`.

With those, Phase 2 completes: both receivers condition at 0.18 s instead of
never, the source receives INFOh, and -- because the recipient now leaves Tone A
on `rx.tone_b_present` rather than on the 2000 ms bound -- the whole exchange
runs on events rather than timers.

## Where it stands

The source now runs Phase 2, Phase 3 and the whole of 12.4's transmit chain --
INFOh, S/S-bar, PP, TRN, silence, **PPh, ALT, MPh** -- where before it went
INFOh, S/S-bar, PP, TRN and then straight into the duplex MP.

The roles are now right: the recipient sends INFOh, goes silent and sits in
`V34_RX_STAGE_PHASE3_WAIT_S` waiting for the source's S, which is 12.3.2.1
exactly, and the source starts Phase 3 only on `V34_EVENT_INFOH_OK`.

**The source side now runs the whole of clause 12's transmit sequence, in
order, driven by the far end rather than by timers:**

    0.18  both receivers conditioned (src TONE_A, rcp TONE_B)
    0.28  src Tone B reversal, then L1/L2
    0.86  src conditioned for INFOh
    0.90  rcp leaves Tone A on Tone B DETECTED, sends INFOh
    1.04  src has INFOh -> 12.3.1.1 silence
    1.10  src S, then S-bar
    1.24  src TRN, for 1.06 s (INFOh asked for 30 x 35 ms = 1050 ms)
    2.30  src 12.4.1.1 silence -> PPh -> ALT -> MPh

The TRN length is the check worth reading there: it is the INFOh field being
honoured, which is what the earlier index-versus-rate defect broke.

**The recipient's S detection was not the problem, and that guess was wrong.**
It detects the source's S perfectly well -- `Rx - Phase 3: far-end S detected
... power=4444960` is in the log at every symbol rate. What was missing was a
**consumer**: every site that acts on `V34_EVENT_S` is inside
`if (s->tx.duplex)` and inside the J stage, and 12.3.1.3 has no J in
half-duplex, so the recipient detected S and sat where it was.

The recipient now has its own Phase 3 watcher.  12.3.2.2's "detect S, then
S-bar, then train on PP and TRN" is the same receive conditioning the duplex
answerer uses at 11.3.1.2.4, so `get_hdx_recipient_phase3_baud()` reuses it
without the transmitter, which in half-duplex has nothing to send.  Measured:
the recipient's receiver reaches `PHASE3_TRAINING` instead of stopping at
`PHASE3_WAIT_S`, at 2400, 3000, 3200 and 3429 baud alike.

**One trap worth keeping.**  The first version installed the watcher with
`tx_silence_init()`, and it never ran: `V34_MODULATION_SILENCE` makes the
sample loop call `tx_silence()`, which never consults `current_getbaud`.  A
silent stage that has to *watch* for something must keep a real modulator and
return `zero` from its getbaud -- which is exactly what the duplex answerer's
wait does, and why that one works.

**It still does not complete.**  Both sides are now stuck at the control
channel.  The source, having sent its 12.4.1.1 silence, is transmitting PPh,
ALT and MPh and its receiver is still acquiring PP on the **primary** channel
(`PP acquire baud 512: mag=0.000`) rather than detecting PPh on the control
channel.  The recipient trains through PP and TRN and then has nowhere to go:
12.4.2.1's "conditions its receiver to detect signal PPh" needs a control
channel receiver that does not exist.

That is the gap, and it is one thing rather than two: **there is no
control-channel receive path for PPh, ALT, MPh or E.**  The transmit side of
all four is built.

No payload crosses in either direction, and no claim is made that it does.

`v34tx.c` and `v34rx.c` are shared with V.90 and plain V.34; every change above
is inside a `!duplex` branch or a diagnostic, and the full suite is green.

## The control channel receiver, and 12.4 end to end

Steps 1 and 2 below are now done: **the source and the recipient complete the
whole of 12.4 -- PPh, ALT, the MPh exchange and E -- and control channel user
data crosses in both directions.**  `v34_hdx_test` now grades that data rather
than counting it (the far end's generator, aligned once and then compared bit
for bit; a bit count only says a modulator ran), and eleven of the twelve
symbol-rate/law rows carry it with **zero errors in both directions**.  They
run in `make test`.

**Read the build note at the end of this section before anything else.**  Most
of the time this took went to a stale-object trap, and three of the findings
below were first "measured" against binaries that were not what the source
said.

### Everything above the wire was on the wrong modulation

10.2.4 puts ALT (10.2.4.2), E (10.2.4.3), MPh (10.2.4.4) and PPh (10.2.4.5) on
the **control channel** modulation -- 600 baud, 1200 Hz from the call modem and
2400 Hz from the answer modem -- and 10.2.3.3 says the same of Sh and Sh-bar.
Only PPh was on it.  ALT, MPh and Sh went out through `V34_MODULATION_V34`, at
the Phase 3 symbol rate and carrier, where no control channel receiver could
have read them however good it was.  `get_e_baud()` then ran into
`data_baud_init()`, the duplex primary channel data mode, where 12.4.1.4 wants
control channel data; that is the new `cc_data_baud_init()`.

`get_pph_baud()` also ran PPh four times over: `PPH_SYMBOLS` is already
`8*PPH_REPEATS`, the whole 32 symbol signal, and the loop ran to
`PPH_SYMBOLS*PPH_REPEATS`.  It was invisible because the table is 8-periodic
and the index was masked to 3 bits.

### The receive side: V34_MODULATION_CC was never selected, anywhere

`cc_rx()` and `process_cc_half_baud()` -- RRC, band edge timing recovery,
differential demodulation, and a complete MPh and E scanner -- were **dead
code**.  `V34_MODULATION_CC` is assigned to `rx.current_demodulator` nowhere in
the tree; Phase 2's INFO exchange is done by `info_rx()` instead.  So "there is
no control-channel receive path" was half right: the demodulator existed and
nothing could reach it.

New `v34_condition_rx_for_pph()` is the way in, called from 12.4.1.1 on the
source and from the end of Phase 3 on the recipient.  New in
`process_cc_half_baud()` is the PPh detector: PPh carries neither the scrambler
nor the differential encoder, so it is found by correlating against the known
8-symbol period, scored as |correlation| normalised by signal energy -- which
is invariant to the arbitrary carrier phase the control channel starts on, the
same shape as the primary channel's PP acquisition at 11.3.1.2.4.

Two things about that detector are worth keeping.

**It runs at the T/2 rate, above the `baud_half` gate, with a correlator bank
per parity.**  PPh is 32 symbols and 12.4.1.1 puts 70 ms of silence in front of
it, so the band edge timing recovery has nothing to converge on until PPh
itself starts: which of the two T/2 outputs is the eye centre has to be decided
by the correlation.  Getting the alignment back wrong is not subtle and it is
asymmetric -- setting `baud_half` to the winning parity rather than to 0 (the
gate is "toggle, then return if the result is 1") aligned one modem onto the
eye and the other onto the eye crossing, and the two then behaved completely
differently on a bit-exact loopback with nothing else to tell them apart.

**The decision is taken on the winning PHASE being stable, not on the score.**
With no equalizer and a timing loop that has not converged, the normalised
score tops out well short of 1, and how far short depends on where in the eye
this receiver happened to start: 0.742 at 3200 baud against 0.805 at 3429, with
the same, correct, phase winning every step in both.  A score gate at 0.80
therefore separated the symbol rates from each other rather than a real PPh
from noise.  A longer coherent memory does not fix it and makes it much worse
(decay 0.985, matched to PPh's own 32 symbols, peaks at 0.24) -- the received
PPh is not coherent over its whole length, which is itself worth knowing.

### The dibit is the negation of the transmitted one, here too

MPh would not decode until `process_cc_half_baud()` negated the recovered
dibit.  10.2.4 advances the point index by the dibit and
`training_constellation_4[]` is ordered so an increasing index rotates
clockwise while the receiver measures the difference counter-clockwise.  This
is the same fact already pinned in the V.90 9.4 CP decode and in the duplex
11.4 MP decode -- a property of the encoder and the table, not of the channel.

### The recipient leaves the primary channel on silence, not on arithmetic

12.4.2.1 has the recipient condition its receiver for PPh after Phase 3.  The
first version computed the moment from INFOh's TRN length (Table 22 bits 15:21,
35 ms units).  That gives the length of TRN and *not* the instant this receiver
started counting it -- the PP acquisition ahead of it takes a variable number of
symbols -- and at 2743 and 3000 baud the offset was enough to put the recipient
on the control channel after the source's 32 symbols of PPh had already gone
by, with nothing to lock to for the rest of the call.  It now moves on the
silence that ends Phase 3, which is 12.4.1.1's own 70 +/- 5 ms and needs no
arithmetic at all.

### What the recipient's S detection was actually resting on

12.3.2.1 has the recipient detect S with nothing before it, but
`phase3_s_detect_armed` is only ever set by the J/Ja machinery -- and
**half-duplex has no J**.  `v34_v90_arm_phase3_s_detector()` is called from the
INFOh transmitter, which is what arms it here; without that call the recipient
would sit in `PHASE3_WAIT_S` for the whole call.  Worth knowing before anyone
tidies that call site away as V.90-specific.

### The rate: 12.4.1.3 and 12.4.2.4

INFOh carries no data signalling rate at all -- Table 22 is power reduction,
TRN length, carrier, pre-emphasis, symbol rate and the TRN constellation -- so
MPh is the whole of the rate negotiation, and every MPh this modem sent went
out with `Maximum data signalling rate = 0` and an empty capability mask.
There was nothing to negotiate from.

`prepare_mph()` now fills Table 23 from the symbol rate INFOh settled: bits
35:49 are the Table 16 rates that symbol rate can carry (0x01FF at 2400 baud
= 2400 to 21600, 0x3FFE at 3429 = 4800 to 33600), and bits 20:23 are this
modem's own ceiling within them -- the source's transmitter's, the recipient's
receiver's, because half-duplex has one primary channel and 3.11/3.14 put the
source at the transmitting end.

`mph_apply_parameters()` is the other half.  12.4.1.3 and 12.4.2.4 are the same
sentence read from the two ends -- "the maximum rate enabled that is less than
or equal to the data signalling rates specified in both modems' MPh sequences"
-- so they necessarily produce the same number and each end configures its own
side of it.  "Enabled" is the mask, and a rate the far end did not offer is not
enabled at the far end, so it is the intersection.  Unlike the duplex 11.4
negotiation there is no direction to keep straight and no acknowledge bit,
which is why 12.4.1.3 turns on having received "at least one MPh sequence"
rather than on an MP'.

**The offer has to be built at the START of 12.4, not when MPh comes to be
transmitted.**  The source holds ALT until it sees the recipient's PPh, so the
recipient's MPh is already on the wire while the source is still in ALT: built
at transmit time, the local half of the negotiation was still all zeroes when
the remote half arrived and the intersection was empty.  It is built in
`pph_baud_init()`, and deliberately not rebuilt afterwards -- by then
`mph_apply_parameters()` has configured this modem's working parameters from
the negotiation, and rebuilding the offer from them would make the second MPh
disagree with the first.

`v34_get_hdx_negotiated_bit_rate()` reports it, and `v34_hdx_test` takes an
optional fifth argument that configures the recipient differently from the
source.  That argument is the point: with both ends the same, a negotiation
that ignored the far end's MPh entirely would give the same answer.  Four
asymmetric rows are in `make test`, either end lower.

### The control channel had no AGC, and that was the last of the bit errors

Filling in MPh changed the control channel waveform, and three symbol rates
went from clean to 64, 87 and 9 bit errors -- always in the same direction, the
source's receiver, which is the 2400 Hz one.  The content dependence was real:
zeroing the MPh body again restored them.

The cause was not the content.  **`cc_rx()` had no AGC of its own** -- a comment
said "CC channel AGC not needed, Phase 2 works with fixed scaling", which was
true only because this path had never once run -- so it used whatever the
PRIMARY channel's AGC had been left at.  On the recipient that is a value
adapted to the source's Phase 3.  On the source it is the untouched reset
default, because 12.3.1 has the source TRANSMITTING throughout Phase 3 and its
receiver adapts to nothing at all.  Measured: control channel symbols at
|z| = 4.4 on the source against 1.43 on the recipient, and a differential phase
sitting 13 degrees from the centre of its quadrant against 5.

A differential decode does not care about gain.  `cc_symbol_sync()` does:
the band edge timing loop is fed the scaled sample and its loop gain is
proportional to it, so at 4.4 the timing jitters enough to push isolated
symbols over the decision boundary.  Swept directly, 3000 baud u-law: **87 bit
errors at the inherited 0.0017 and zero at 0.001, 0.00055 and 0.0003 alike.**

The control channel now runs its own AGC to a defined working point, adapting
during PPh, ALT and MPh, which are all constant modulus.  With it, **all twelve
symbol-rate/law rows carry the control channel data with zero errors in BOTH
directions** -- including the 2400 baud u-law three-error burst that was left
open as "the control channel's own timing recovery", which is exactly what it
turned out to be.

`V34_CC_SYM_STATS=1` prints the working point and the eye: |z|, and the mean
and worst distance of the differential angle from the centre of its quadrant,
where 0 degrees is perfect and 45 is the decision boundary.  There was no
quality instrument for this channel at all before.  `V34_HDX_ERRPOS=1` prints
the positions of any payload bit errors.

### The build trap: three v34 objects had no header dependencies at all

`spandsp-master/src/Makefile` (and `Makefile.in`) listed `v34rx_data.lo`,
`v34rx_phase3.lo` and `v34rx_phase4_trn.lo` in `am__objects_2`, so they linked,
but **did not `include` their `.deps/*.Plo` files** -- the generated Makefile
predates those sources being split out and was hand-patched incompletely.  So
any change to `spandsp/private/v34.h` left those three objects compiled against
the OLD `v34_rx_state_t` layout, silently, while everything else rebuilt.  The
symptom is not a link error: it is impossible field values.  `Rx - stage=
PHASE3_WAIT_S demod=UNKNOWN (18)` is what it looked like here, and it cost this
session three separate wrong conclusions -- including "the recipient does not
detect S at HEAD", which was measured twice against mixed builds and is not
true.  The `include` lines are added now.  The lesson generalises: on this tree,
when a change to a header produces behaviour that contradicts the source, check
what actually recompiled before believing the measurement.

## Order of work from here

1. ~~A control-channel receiver.~~  Done -- see the section above.
2. ~~Wire 12.4 end to end, including the 12.4.1.3/12.4.2.4 rate.~~  Done, both
   roles, control channel data crossing in both directions with the rate
   settled from both MPh sequences.  **One piece of MPh is deliberately not
   implemented: bit 27, the control channel data signalling rate.**  10.2.4's
   2400 bit/s mode puts four bits on each symbol by selecting a point from the
   quarter superconstellation with Q1/Q2 rather than always point 0; this
   modem only implements the two-bit form, so it asks for 1200 (and sets bit
   50, asymmetric control channel rates, to 0).  That is honest rather than
   convenient -- asking for 2400 would be asking for something it could not
   then read.
2a. The 12.4.3 and 12.4.4 recovery procedures.  A missed PPh, MPh or E is a
   **control channel retrain (12.8.1)** after three seconds, not a
   retransmission, and none of it exists.  Today a missed PPh is a dead call.
3. `half_duplex_state` actually read: 12.5's primary channel turn-off and
   12.6's control channel turn-off are the source/recipient turnarounds, and
   nothing consumes the mode today.
4. Control channel data at 1200/2400 bit/s (10.2.4) as a byte interface, then
   HDLC over it -- F.3.1.4 wants flags and frames, which is what T.30 needs.
5. T.30 Annex F itself: DIS bit 6 and the V.8 route into fax, T.30's frames on
   the control channel instead of V.21, no TCF, the 40-ones/70 ms turnarounds,
   image on the primary channel, ECM forced on.
6. `fax.c` datapump wiring and a `T30_MODEM_V34HDX` row in
   `fallback_sequence[]` -- without which T.30 can never select it however
   much of the rest works.
7. T.32 Amd 1's BR 6-D and amended EC in `fax_class2.c`; class 1 in `t31.c`.

Steps 1 and 2 are the whole of the modem layer.  Steps 5 to 7 are the fax
layer and are mostly plumbing once 1 to 4 exist.
