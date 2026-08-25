# Retrain and resynchronisation (V.90 §9.5/§9.6, V.34 §11.5/§11.6)

Both Recommendations give a modem two ways to recover a receiver that has
stopped working, and this note records which of them exist here, which do
not, and what each was measured to be worth.

## What the specs offer

**Retrain** — V.90 §9.5, V.34 §11.5.  "Turn OFF circuit 106, clamp circuit
104 to binary one and transmit silence for 70 ± 5 ms", then hold your own
tone (Tone B for the V.34 call modem and the V.90 digital modem, Tone A for
the V.34 answer modem and the V.90 analogue modem) and go back to Phase 2.
Neither clause carries a phase qualifier: a retrain may be initiated, and
must be responded to, **at any time**, data mode included.

Note what §11.5 does *not* say.  It clamps circuit 104 — the DTE's data is
held for the duration — and it does not clear circuit 109 or take CONNECT
back.  The error-control link above the physical layer survives a retrain.

**Rate renegotiation** — V.90 §9.6, V.34 §11.6.  The cheaper of the two, and
V.34 says outright that it "can also be used to resynchronize the receiver
without going through a complete retrain".  It ends in a fresh B1, which is
exactly what an upstream receiver acquires against, and it costs a fraction
of a second rather than a whole startup.  Its cost is that it needs a
*responding* modem: a retrain needs only the peer's tone detector, which
every V.34 modem has.

## What is implemented

| Clause | Role | State |
| --- | --- | --- |
| §9.5.1.1 initiate | V.90 digital | yes, on a receiver that has stopped decoding; bounded |
| §9.5.1.2 respond | V.90 digital | yes, in every stage including DATA |
| §9.5.2.1/.2 | V.90 analogue | yes (Phase 3/4 deadlines) |
| §9.6.1.1 initiate | V.90 digital | built; **default off**, see below |
| §9.6.1.2 respond | V.90 digital | yes, `ME_V90_RENEG_RESPOND=1`; **default off**, unverified against a peer |
| §11.5.1.1/§11.5.2.1 initiate | plain V.34 | yes, on a receiver that has stopped decoding; bounded |
| §11.5.1.2/§11.5.2.2 respond | plain V.34 | yes, in every stage including DATA |
| §11.6.1.1 initiate | plain V.34 | yes, **on by default** — verified live, see below |
| §11.6.1.2 respond | plain V.34 | yes, `ME_V90_RENEG_RESPOND=1`; **default off** |
| §11.6.2.1 recovery | plain V.34 | yes — no E within the timeout falls back to a §11.5 retrain |

`ME_V90_RENEG=1` enables §9.6 initiation.  It is off by default for a
measured reason and not a cautious one: this rig's analogue modem answers
384T of Rd with nothing at all, its own log declares SILENCERETRAIN, and it
retrains.  On a peer that implements §9.6.2 it is the better recovery and
should be preferred; the engine tries it first when it is enabled and falls
through to §9.5.1.1 when it is not.

## Bounds

A retrain costs the whole Phase 2/3/4 startup, so it is worth taking only
where the alternative is a dead link, and only a few times.  Two bounds, both
per call and shared by the V.90 and plain V.34 paths:

* `ME_V90_RETRAIN_ON_LOSS` — how many per call.  Default 4; `0` restores the
  old behaviour of holding a link whose receiver has stopped decoding.
* `ME_V90_RETRAIN_ON_LOSS_DWELL_MS` — how long since the last one.  Default
  20000.  The counter advances whether or not the attempt reaches data mode,
  so a link that collapses again immediately degrades to being left alone
  rather than spending the call retraining.

## Three defects this found, each behind the next

**The retrain tone was watched only in the training stages.**  The detector
was gated on `v34_rx_stage_is_primary_training()`, which excludes
`V34_RX_STAGE_DATA`, so a peer whose own receiver had failed held its tone
into a receiver that was not listening for it — and the SmartLink peer drops
the call about 3.1 s in.

**Even ungated it would not have run.**  With the T/3 upstream receiver
active, `primary_channel_rx()` returns from its DATA branch before reaching
the detector.  It is now `v34_rx_watch_peer_retrain()`, called at the top of
that function, ahead of the return.

**The engine could not leave data mode.**  `restart_v90_phase2_locked()`
never moved `g_state`, so a retrain taken from data mode left the engine in
`ME_DATA`; the V.90 transmit path then hit its `if (!g_v90) return false` and
the call went quiet in both directions rather than resynchronising.

Running the tone watch in DATA is safe for the same reason it is safe in
Phase 3/4: it requires 80 ms in which one bin holds over 70% of the block
energy, and a data-mode primary channel spreads its power across the band by
construction.  Which tone to listen for follows the peer's ROLE, not the call
direction, and is the same predicate the control-channel receive carrier is
already selected on — 2400 Hz where `calling_party` and `v90_mode` differ,
1200 Hz otherwise.  Both land exactly on a bin of the existing 160-sample
Goertzel.

## The fourth: one acquisition window decided the whole call

`v90_t3_try_acquire()`'s in-sample bail (`best_match < 0.95f`) returned
without arming a retry, leaving `v90_t3_acquisition_attempted` set — so the
search never ran again.  A call whose B1 was not in front of the anchor at
the first attempt carried **no upstream at all for its entire life**, with
nothing logged after that one line.  Reproduced on three of the twelve
recorded rate-matrix calls under `v90_engine_replay`: one "acquisition
failed" and then 115 s of silence.

It is the same situation the out-of-sample rejection twenty lines below
already handled by sliding the window forward, so it is handled the same way
now, and both share `v90_t3_acq_retry_or_abandon()`.  When the windows do run
out the receiver says so, and `v34_v90_upstream_carrier_lost()` reports it:
an upstream that never acquired is the same condition as one that has stopped
decoding, and worse, since the call has carried nothing since B1.

## How to see it work

`v90_engine_replay` runs V.8 and Phases 2-4 through the whole engine off a
recording, which is what makes the chain visible end to end:

```
./v90_engine_replay artifacts/goal-matrix-115515Z/rate28800-r1/live-rx.g711 ulaw --fast
```

produces, in order: seven acquisition windows instead of one, `B1 giving up
after 7 windows (in-sample fit)`, `upstream carrier lost and §9.6 is not
available; initiating a §9.5.1.1 retrain (1 of 4)`, and `retraining out of
data mode; DTE data clamped, error-control link retained`.

The replay then falls back to V.22bis, and that is the harness rather than
the code: it is receive-driven off a recording of a *data mode* call, so
there is no peer retrain response in the file for Phase 2 to complete
against.  Proving the peer answers needs the live rig.

## What is not established

* **The retrain paths have no live verification.**  Whether the SmartLink
  peer answers our Tone B from data mode within its own timeout is not known,
  and the rate at which a retrain recovers a call rather than ending it is not
  known.  §11.6 *is* verified live — see the last section.
* **§9.6.1.2 is behind a knob and §11.6 is absent.**  See below for what the
  responder is measured on; it has never met a peer that starts one.
* **The acquisition retry does not rescue acquisition in `v90_engine_replay`**:
  all seven windows fail the in-sample fit on the three calls that failed at
  the first.  That is the same live/replay divergence
  `docs/v90_upstream_data_path.md` records — the standalone upstream replay
  acquires on the same files.  What the change buys *there* is that the
  failure is observable and recoverable rather than silent and terminal.

  In `v90_upstream_replay` the payload counts move a great deal, in both
  directions — and **that is not a measurement of this change.**  Pin the
  handover instant instead of letting the harness search for it
  (`v90_upstream_replay <tap> ulaw <baud> <bps> <seconds>`) and the two
  binaries are **identical**: same window count, same mean symbol error to
  three decimals, logs that diff clean, on all three calls tried.

  | call | pinned at | base | after |
  | --- | --- | --- | --- |
  | rate19200-r1 | 26.0 s | 585 windows, 0.150 | 585 windows, 0.150 |
  | rate24000-r2 | 25.5 s | 1040 windows, 0.358 | 1040 windows, 0.358 |
  | rate28800-r1 | 25.5 s | 1939 windows, 0.583 | 1939 windows, 0.583 |

  So **the receiver at a given anchor is unchanged.**  What the retry changes
  is which candidate instant the harness's outer sweep settles on: every call
  moves about 3 s earlier, because the earlier candidate is no longer
  abandoned after a single window.  With the sweep free to choose, the same
  four calls read 22058→18144, 30721→30721, 23019→12151 and 14575→20201 —
  −10% overall and ±40% per call, which is a measurement of an **unranked
  candidate search**, not of this change.

  The search was already unranked; the retry only made it visible.  The
  in-sample bail rejects a window before the out-of-sample scoring that
  `v90_t3_acq_best_*` uses, so a candidate instant wins on ORDER and nothing
  else.  Ranking candidate instants the way windows within a candidate are
  already ranked is the obvious next step and is not done here.

  **Read the pinned rows, not the swept ones.**  A live call has no outer
  sweep: it has one E anchor, it used to get one shot at it, and it now gets
  up to seven sliding forward.

  The price is CPU: seven least-squares searches per candidate instead of one.
  Live that is bounded by the existing `V34_V90_T3_ACQ_RETRY_GAP` backoff to
  one search per 0.5 s at 3200 baud, which is the budget the out-of-sample
  path was already tuned against; offline in the sweep harness it is a
  straight 7x per candidate, because the harness fast-forwards the backoff.

  Note the two failure kinds now share one retry counter.  That is the safe
  direction: a call that spends some of the budget on in-sample failures
  reaches the "settle for the best window seen" fallback sooner, and the
  fallback being REACHABLE is what `V34_V90_T3_ACQ_MAX_RETRIES` was cut to
  six for in the first place.


## Responding to a peer's rate renegotiation (§9.6.1.2)

The cheap resynchronisation has to be detected on a receiver whose eye may be
shut, which is exactly the state it exists to fix — so the detector does not
look at symbols.  §10.1.3.7's S is the 4-point sequence alternating by 180°,
i.e. a baseband ±1 at half the symbol rate, so in the passband it is two
lines at fc ± baud/2 and, inside the RRC band, almost nothing else.  A pair of
Goertzels against total block energy separates that from a data-mode primary
channel, whose power is spread across the band by construction, without
needing the equalizer, the carrier loop or the timing loop to be working.

Two details worth keeping.  **V.34 §5.1 puts the carrier at fc = S·d/e**, not
2400·d/e, which is why 3200 baud low reads 1828.6 Hz; d/e is the `low_high[]`
pair in `baud_rate_parameters`.  And **the block length is 10 ms, not the
20 ms the retrain tone watch uses**, because the window is short: S is 128T
and S̄ 16T, which is 45 ms at 3200 baud, so three 10 ms blocks fit inside it
with margin and 20 ms blocks would not reliably.

The response itself needs no new transmitter: §9.6.1.2.2's Rd 384T / R̄d 24T /
TRN2d / MP is bit for bit what §9.6.1.1.1 sends when we initiate, so the
event arms the existing request and the transmit path starts it at the next
data-frame boundary, which §9.6 requires anyway.  The S-to-S̄ transition is
not separately detected — S is 128T and S̄ 24T and the peer then runs SCR and
CP for up to 2 s, so the boundary we start on is well inside the window it is
waiting in.  Answering is a "shall" independent of `ME_V90_RENEG`, which only
governs whether we *start* one.

**Measured, in both directions:**

* *Negative.*  Over the recorded rate-matrix calls — live data-mode audio at
  every rate from 19200 to 31200 — the detector fires **zero** times.
* *Positive.*  Splice 45 ms of synthesized S (two tones at 228.6 and
  3428.6 Hz, the exact 3200-baud low-carrier pair) into one of those same
  recordings at t = 60 s and it fires **exactly once**, reporting
  "30 ms of two tones at 229/3429 Hz".

That bounds the false-positive rate and proves the detector works on the
signal it names.  It says nothing about a real peer: nothing in the corpus
renegotiates, every capture being of a call that either held data mode or
died.  A false detection would take down a working call, so it stays behind
`ME_V90_RENEG_RESPOND=1` until a peer proves it.

## §11.6 for plain V.34 — and the defect that hid it

This is the only part of this note that is verified end to end rather than
reasoned about, so read it before the rest.

**It builds no second transmitter.**  Figure 22's sequence — S 128T, S̄ 16T,
TRN, MP, MP′, a single 20-bit E, B1, data — is Phase 4's exactly, so
`v34_start_rate_renegotiation()` re-enters the Phase 4 stages from data mode.
Two seams need care:

* **TRN → MP.**  §11.4 waits on the far end's J′; §11.6 has no J at all, so
  the confirmation the startup path waits for can never arrive.
  `get_phase4_baud()` reads `tx.reneg_active` there and goes to MP at the
  Phase 4 minimum.  §11.6.1.1.1 permits up to 2000 ms and
  `ME_V34_RENEG_TRN_BAUDS` exposes that — **raising it is measured not to
  help**, so the default is the minimum.
* **Where the receiver starts.**  At PHASE4_S, not the PHASE4_TRN that
  startup uses: §11.6.1.1.2 and §11.6.1.2.1 both say "After detecting signal
  S … be conditioned to detect the S-to-S̄ transition", and here there is a
  real far-end S to find.

Both roles run the same sequence.  §11.6.1.1 and §11.6.1.2 differ only in who
sends S first, and a responder is called having already detected the
initiator's.

### The defect: S is 90°, not 180°

§10.1.3.7 alternates between one point and the same point **rotated
counterclockwise by 90 degrees**.  That is not a 180° alternation, and the
difference is the whole detector: a, ja, a, ja, … is a(1+j)/2 plus
a(1−j)/2·(−1)^k, so the energy sits in **equal parts on a line at the carrier
and a line at fc ± baud/2** — three bins, not two.

Watching only fc ± baud/2 caught **0.44** of the block energy on a real S,
exactly the half the decomposition predicts, and never crossed the 0.60 gate.
No responder ever answered, and every symptom pointed at the transmit side.
`V34_RENEG_S_DEBUG=1` prints the ratio per block; that is what found it.

### What the test asserts

`v34_duplex_test` gained `V34_DUPLEX_RENEG=<bits>`.  It runs two real modems
to data mode over a G.711 round trip, has the **caller** initiate §11.6,
leaves the **answerer** to detect its S and respond through the same public
entry point the engine uses — not by being told — and then requires 8000 bits
of payload in **both** directions with **zero errors** on the far side.

Ten rows in `make test`: every symbol rate that trains, both laws, all ten
passing.  3429 is absent because it does not train at all.

The payload check crosses no seam.  Sixteen consecutive outputs of the
harness's LFSR *are* the state that produced the first of them, so the
receiver re-derives the transmitter's position from the bits it actually
demodulated, then validates that position over 64 bits before believing it
and slides forward if it does not hold.  **The first version was off by
sixteen bits** — it set the state and predicted from it without advancing
past the bits already consumed — and reported a perfect receiver (0.0138 from
the grid, 0% bad shell frames) as 50% bit errors.  A resync that cannot
restart and report failure cannot tell its own mis-anchoring from a broken
modem.

### What it does not cover

* **21600 is deliberately not asserted.**  It renegotiates and comes back
  decoding — B1 correlation 0.99, shell index 0–1% — but some rows carry bit
  errors afterwards, and a longer TRN does not help, so the cause is not
  equalizer reconvergence and is not understood.
* **§11.6.1.2.1's "clamp circuit 104" is approximate.**  The detector needs
  30 ms of S to fire, so the responder delivers ~50–90 bits of garbage to the
  DTE before it stops.  Under V.42 the frame CRC discards them, which is the
  normal protection; without error control they reach the DTE.
* **No live verification**, as everywhere else in this note.


## Live: §11.6 against the SmartLink rig (2026-08-26)

Two calls, plain V.34 at 3000 baud / 9600 bps with V.42 LAPM, our side
initiating a renegotiation 20 s into data mode via `ME_V34_RENEG_AFTER_MS`.
Kept as `artifacts/reneg-live-r1` and `-r2`.

**The peer implements §11.6.1.2.**  It detected our S and responded with its
own MP in both calls.  This is worth stating plainly because the assumption
carried over from §9.6 was the opposite — and that assumption does not
transfer: §9.6 is a different procedure with a different signal, and the note
"this rig's analogue modem answers Rd with nothing" was measured on *that*.

**r1 — the whole procedure, and it changed the rate:**

```
Tx - 11.6 rate renegotiation: transmitting S (128T) then S-bar (16T), TRN and MP
Tx - 11.6 rate renegotiation: TRN complete (512 bauds), starting MP
Rx - MP0 diag: ... crc_ok=1 fill_ok=1
Rx - Phase 4 negotiated: a2c=12000 bps c2a=9600 bps
Tx - far-end MP received, switching to MP'
Rx - Phase 4 MP microstate=COMPLETE (E detected)
Rx - B1 acquired: symbols=120 ... normalized-correlation=1.000
Tx - data_baud_init(): rate=12000 bps
```

and the peer's own log agrees from its side: `V34DATARATE, finally txbitrate
9600, rxbitrate 12000`, then `txstate XMITMP=>EXMIT`.

So this was a real **rate** renegotiation, not merely a resync: the link came
back at 12000 where it had been running at 9600.  Afterwards the data mode
measured **0.0023–0.0025 from the grid, 0% bad shell frames over 7680–13312
frames, ~35 dB**, held for the rest of the call.

**And the DTE stream did not break.**  1203 numbered lines reached the peer's
DTE, contiguous, **zero gaps** across the renegotiation.  That is §11.5/§11.6
working as written — circuit 104 clamped for the duration, the error-control
link above it untouched — and it is the direct payoff of *not* re-running
`data_stack_start_online()` on the way back in.

**r2 — the failure path, and §11.6.2.1 fired correctly:**

The peer responded, but its MP never CRC-validated (five frames, `crc_ok=0`,
garbled start bits).  No E arrived, the timeout expired, and:

```
[ME] V.34 §11.6 rate renegotiation produced no E; falling back to a §11.5 retrain
[ME] V.34: rate renegotiation timeout; restarting Phase 2 (3000 baud / 9600 bps)
```

with the transmitter walking INITIAL_PREAMBLE → INFO0 → INITIAL_A → FIRST_A.
356 lines, zero gaps, before the call was ended.

**Why initiating is on by default.**  One of two converging is thin, but the
trade is not: the failure path lands exactly where the code would have gone
without §11.6 at all, and `retrain_on_loss_due()` already bounds the whole
thing to four attempts per call.  Worst case is a few seconds of
renegotiation before the same retrain; best case keeps the link and the
error-control layer up.  `ME_V34_RENEG=0` disables it.

**What r2 does not establish:** the call was ended shortly after the fallback,
so whether that §11.5 retrain went on to reconnect is not known.

**`ME_V34_RENEG_AFTER_MS=<n>` is a test hook**, not a feature: it opens a
renegotiation n ms after data mode, once.  It exists because the engine's own
trigger is a receiver that has stopped decoding, which a healthy call never
produces, and the only open question about §11.6 was whether a real peer
answers.  The analogue role has had the same knob for the same reason.


## Live: the retrain paths against the SmartLink rig (2026-08-26)

Provoked with `ME_V34_RETRAIN_AFTER_MS`, a test hook that initiates a §11.5
retrain n ms into data mode (the engine's own trigger is a receiver that has
stopped decoding, which a healthy call never produces).  Kept as
`artifacts/retrain-live-b2`, `-c1`, `-c2`.

**The peer recognises our retrain, and says so in its own log:**

```
VPcmV34Main: Retrain Detected by Tone detector !
V34HSHAKE: txstate DATAXMIT=>SILENCERETRAIN
V34HSHAKE: txstate SILENCERETRAIN=>TONE_AB
V34RETRAIN, SILENCERETRAIN finished
```

So §11.5.1.1 initiation reaches a real V.34 modem and starts its §11.5.1.2
response.  **Two defects had to be fixed to get that far, and only a live
call could have found either.**

**(a) We restarted into INFO0 instead of the tone.**
`restart_v34_phase2_locked()` used `v34_restart()`, which re-enters Phase 2 at
INITIAL_PREAMBLE/INFO0 — a *modulated control-channel carrier* in front of a
peer that is waiting for a tone.  §11.5.1.1's "condition its receiver to
detect Tone A **and receive INFO0a**" had been misread as licence for the
transmitter to send INFO0; it says what to do *if* the peer sends one.  The
V.90 side has known this since 2026-07-22 and has
`v34_v90_start_retrain_response()` for it; plain V.34 had nothing.  New
`v34_start_retrain()`, and the tone is picked by role — `calling_party ==
v90_mode` is the Tone A side, which is Tone A for the V.34 answer modem
(§11.5.2.1) and for the V.90 analogue modem alike.  The stage handler's old
`if (s->calling_party)` is identical whenever `v90_mode` is 1, so the V.90
paths are unchanged.

**(b) The 70 ms of silence lasted 18 ms.**
`V34_TX_STAGE_V90_RETRAIN_SILENCE` is handled inside
`get_v90_wait_info1a_baud()`, and its `>= 42` count is 70 ms only at the
**600-baud control channel** rate.  Setting the stage without also setting
`current_getbaud` left the data-mode generator running at 3000 baud, so the
count expired in 18 ms and the transmitter fell straight through into
INITIAL_PREAMBLE/INFO0 — i.e. defect (a) again, by a different route.

### Fixed: the Tone A phase reversal (and what stops it now)

With both fixed, `c2` puts the conformant sequence on the wire — 70 ms of
silence, then Tone A — and the peer answers by going SILENCERETRAIN →
TONE_AB.  Then **both sides wait, and the peer drops the call after 3.06 s**
(`DP_DISC`, `NO CARRIER`).

Our side sits at `rx=TONE_B tx=FIRST_A` with signal plainly present
(`carrier=1 sig=1 pwr=~7e6`) and never publishes a Tone B detection, so it
never sends the **Tone A phase reversal** that §11.2.1.2.4 owes the peer and
that §11.5.1.2 leaves it waiting for.  3.06 s is the same "about 3.1 s"
unanswered-tone timeout the V.90 notes record for this peer.

The likely cause is that the tone detector is being asked to work with a
front end that has just come out of a 3000-baud data mode — AGC, equalizer
and carrier state all tuned for a data constellation, where a fresh call
reaches the tone stages from a reset front end.  `b2` is evidence that simply
sending INFO0 instead does not help: it drops at the same 3.1 s.

**The receiver was detecting Tone B all along** — `Rx - Tone B detected
(power=9441985 ref=9462494)` is right there in `c2`'s log.  It just never told
the transmitter.  Two gates, both pre-existing:

* `V34_EVENT_TONE_SEEN` is published only when `v90_mode && calling_party &&
  info0_received`, so on a plain V.34 call it is never published at all.
* FIRST_A's plain-V.34 condition is `received_event == V34_EVENT_INFO0_OK` —
  and §11.5 **omits the INFO0 exchange**, so it can never fire on a retrain.

The transmitter therefore waited for an INFO0 that the spec says will not
come, while the peer held Tone B and dropped on its timeout.

The fix reads `rx.tone_b_present` — the dedicated flag that exists precisely
because "Tone B is detected" is a separate fact from the reversal ordinal
sharing the single `received_event` slot — and only when `tx.retrain_omit_info0`
is set, so an ordinary startup keeps its established INFO0c timing.  This is
§11.2.1.2.3's actual condition ("After Tone B is detected and Tone A has been
transmitted for at least 50 ms"; INITIAL_A served the 50 ms), so it is the
normative test rather than a workaround.

**Live, `d2` and `d3`, reproduced identically:**

```
Tx - FIRST_A: Tone B detected and INFO0 omitted by 11.5, sending !A (11.2.1.2.3)
Tx - FIRST_NOT_A: Tone B ended without a detected reversal; treating it as 11.2.1.1.3
    ... FIRST_NOT_A_REVERSAL -> SECOND_A -> POST_L2_WAIT_TONE_B -> POST_L2_A
    ... -> POST_L2_NOT_A -> A_SILENCE -> PRE_INFO1_A
```

The whole §11.2.1.2 answer-modem timetable now runs on a retrain, the peer
accepts it (`V34RETRAIN, RX_PHASE1_CALL received`), and **the 3.06 s
unanswered-tone drop is gone: the call now lives 15.3 s past the retrain
instead of 3.4 s.**

### Open: the peer's own Phase 1 livelock

What stops it now is in the peer, and it is already documented:

```
V34RETRAIN, RX_PHASE1_CALL received
microstate RX_PHASE1_CALL=>TX_PHASE1_CALL
Repeated info0 is detected, errorrecovery is initialized in TX_PHASE1_CALL
```

That is the same `Repeated info0 is detected` livelock in the shipped
SmartLink DSP that `docs/v34_plain_phase2_call_role.md` records as the
blocker for plain-V.34 *origination* — reproduced there on two peer binaries,
with all three permitted responses measured and all three failing.  Its
recovery has one exit, receiving an INFO0a, which `TX_PHASE1_CALL` reads as a
repeat, and it loops until it gives up.  Our side sits in the §11.2.1.2.5
resume waiting for an L1/L2 probe and INFO1c that never arrive.

So: **initiation reaches the peer, drives its retrain, and completes our half
of the tone exchange; the peer then enters a Phase 1 error-recovery livelock
of its own that this tree has hit before and cannot fix from this end.**
Responding to a peer-initiated retrain (§11.5.1.2/§9.5.1.2) is still untested
live — nothing yet makes this peer start one.
