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

## Order of work from here

1. **A control-channel receiver.**  PPh detection first, since both 12.4.1.1
   and 12.4.2.1 turn on it, then ALT, MPh and the 20-bit E.  Everything below
   waits on this.  The transmit halves (`pph_baud_init()`,
   `second_alt_baud_init()`, `mph_baud_init()`, `e_baud_init()`) are done.
2. Wire 12.4 end to end: source detects PPh -> trains -> MPh -> E; recipient
   detects PPh -> sends PPh, ALT, MPh -> E.  Both then have a rate from the
   MPh exchange.
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
