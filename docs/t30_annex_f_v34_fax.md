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

**`ITU Docs/` holds no T-series Recommendations at all.**  T.30 is not there,
so the Annex F layer has no local normative source.  Anything written against
it will be written without the document this repo's own rules require citing;
get T.30 before starting that layer.

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

## Where it stands

The source now runs Phase 2, Phase 3 and the whole of 12.4's transmit chain --
INFOh, S/S-bar, PP, TRN, silence, **PPh, ALT, MPh** -- where before it went
INFOh, S/S-bar, PP, TRN and then straight into the duplex MP.

The roles are now right: the recipient sends INFOh, goes silent and sits in
`V34_RX_STAGE_PHASE3_WAIT_S` waiting for the source's S, which is 12.3.2.1
exactly, and the source starts Phase 3 only on `V34_EVENT_INFOH_OK`.

**It still does not complete, and the blocker has moved to Phase 2.**  The
source stalls in `V34_TX_STAGE_HDX_FIRST_B_INFO_SEEN`, which waits for
`V34_EVENT_REVERSAL_1` -- the Tone A phase reversal of 12.2.1.1.2.  The
recipient transmits that reversal on time (`HDX_FIRST_NOT_A` at 0.22 s), but
the source's receiver is still in `V34_RX_STAGE_INFO0` when it arrives and
never reports it, so the source never transmits its Tone B reversal, L1 or L2,
`rx.tone_b_present` never becomes true at the recipient, and the recipient
falls out of Tone A on the 2000 ms bound instead of on the event.

That is a Phase 2 conditioning defect, not a Phase 3 one, and it was there
before: the old trace only got further because the recipient was wrongly
transmitting S/S-bar/PP/TRN, whose energy carried the source's detectors along.
Fixing the roles removed that accident and left the real state visible.

No payload crosses in either direction, and no claim is made that it does.

`v34tx.c` and `v34rx.c` are shared with V.90 and plain V.34; every change above
is inside a `!duplex` branch or a diagnostic, and the full suite is green.

## Order of work from here

1. Phase 2 receiver conditioning: on INFO0a the call modem must condition its
   receiver to detect Tone A and its phase reversal (12.2.1.1.2).  The rx does
   set `V34_RX_STAGE_TONE_A` on INFO0 receipt, but the source's receiver was
   observed still in `V34_RX_STAGE_INFO0` well after its transmitter had acted
   on `V34_EVENT_INFO0_OK`; find out why those two disagree.
2. `half_duplex_state` actually read: 12.5's primary channel turn-off and
   12.6's control channel turn-off are the source/recipient turnarounds, and
   nothing consumes the mode today.
3. Control channel data at 1200/2400 bit/s (10.2.4) as a byte interface.
4. Only then T.30 Annex F -- and not before the Recommendation is to hand.
