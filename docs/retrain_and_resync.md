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

`ME_V90_RENEG=1` enables §9.6 initiation.  It is off by default because it is
**unverified against a real peer** -- not, as this note used to say, because
the peer refuses to answer.  That claim was ours.

**Retracted (2026-08-27): "this rig's analogue modem answers 384T of Rd with
nothing at all".**  We never transmitted 384T of Rd.  §9.6.1.1.1 makes Rd
exactly 384T terminated by 24T of R̄d; the only code that advanced that state
lived in the CP **receive** path and was gated on a far-end CPt, which is
§9.4.1.2's *startup* rule, where the barred Ri acknowledges the analogue
modem's CPt.  In a renegotiation the peer cannot send CP until it has seen
Rd, R̄d and MP -- so each side waited for the other and Rd ran until the call
ended.

Demodulated out of the transmit taps of the very two calls the default was
set from, with `tools/v90_rd_verify.py`:

| capture | startup Ri | renegotiation Rd |
| --- | --- | --- |
| `artifacts/goal-v90-reneg-112546Z` | 3804T + **24T** barred | **89120T (11.1 s), no barred symbol** |
| `artifacts/goal-v90-reneg-b-113038Z` | 3822T + **24T** barred | **24160T (3.0 s), no barred symbol** |

The startup Ri in the same files terminates correctly, which is what makes
the renegotiation rows unambiguous.  The peer was never given the signal
§9.6.1.2 requires it to detect, so neither call says anything about whether
it implements the clause -- and the peer demonstrably *does* implement the
V.34 §11.6 form (below): it detected our S, renegotiated 9600 -> 12000 and
kept LAPM up across the change.

Fixed in `v90.c`: on a renegotiation Rd terminates on its own 384T count and
R̄d is unconditional, per §9.6.1.1.1's "transmit signal Rd for 384T and R̄d
for 24T.  After transmitting R̄d, the digital modem shall optionally transmit
TRN2d for no more than 2000 ms followed by MP sequences".  Replaying
`goal-v90-reneg-b-113038Z` through the fixed engine now puts **384T + 24T
exactly** on the transmit tap.

**ANSWERED LIVE (2026-08-27): the peer detects it, and two more defects of
ours were behind that.**  With a conformant Rd the SmartLink peer prints a
message this project's corpus had never contained --
`VPcmFloModem (V90): rate renegotiation detected !!` -- followed by
`setV90RateReneg called` and its Phase 4 `linear mapping study in TRN2`.  So
it implements §9.6.1.2, and the old default rested on a signal it was never
given.

Two further defects, each found by the peer's own stopwatch, and its verdict
is DETERMINISTIC per setting -- one call characterises an arm, exactly as
§33 found for TRN1d:

| arm | peer study | first retrain after detection |
| --- | --- | --- |
| Rd fix only, TRN2d 4000T | **failed 0/4** | 0.82 s x4 |
| + TRN2d 16000T | **succeeded 4/4** | 2.32 s x4 |
| + MP acknowledgement reset | **succeeded 3/3** | none within 5 s |

**(a) The renegotiation's TRN2d was at the startup length.**  §9.6.1.1.1
allows TRN2d "for no more than 2000 ms" and we sent 4000T = 500 ms -- the
§9.4.1.2 startup value, which has its own measured tuning (§38: 10398T and
12000T both graded WORSE at startup, so the two must not share a knob).  But
startup gives the peer 20004T of TRN1d to train against and §9.6.1.1.1 gives
it nothing but TRN2d, and this peer's study needs ~2.76 s.  At 500 ms it gave
up after 0.54 s.  New `ME_V90_RENEG_TRN2D_SYMBOLS`, default 16000 (the
clause's ceiling).

**(b) The renegotiation opened on MP', not MP.**  §9.6.1.1.1 transmits MP and
conditions the receiver to receive CP; §9.6.1.2.3 sends MP' only *after* a CP
arrives.  `cp_ack_received`/`data_cp_received` are latched from the startup
that reached data mode, so we offered the acknowledged form before this
procedure had seen a single CP.  Same family as the Rd defect -- startup
state carried into §9.6.  Both are now cleared in
`v90_rate_renegotiation_start()`.

**The peer DOES send CP -- the premise was wrong (2026-08-28).**  Its own log,
during a renegotiation:

```
+1.520  V90Phase4Demodulator: disable linear mapping study   (our TRN2d graded OK)
+1.520  VPcmFloModem (V90): Building CP, CP length = 700 (clr=0)
+2.020  VPcmFloModem (V90): Building CPnot on MP receive, CPnot length = 700
+7.060  V34HSHAKE: txstate DATAXMIT=>SILENCERETRAIN
```

So it grades our TRN2d, builds CP, **receives our MP** and builds CP' -- the
whole of §9.6.1.2.3 -- and gives up ~5 s later because no Ed comes back.
**We are not decoding its CP.**  Identical in three consecutive
renegotiations.

**Fourth defect: we never conditioned the receiver for the S answer.**
§9.6.1.1.1 says "condition its receiver to detect S, S-bar, **and CP**", and
Figure 8/V.90 says why -- the analogue modem answers Rd with S (128T), S-bar
(16T) and an optional SCR of up to 2000 ms, and only THEN sends CP.
`enter_v90_phase4_rx_locked()` is the STARTUP conditioning and goes straight
to the CP search, which is right there (after DIL the peer begins repeated
CPt at once, and startup Phase 4 contains no S at all) and wrong here.
Measured: the peer's S is on the wire **258 ms after our R-bar-d and 40 ms
long -- 128T at 3200 baud, §9.6.2.1.1 to the symbol -- with 98.7% of the
block energy in §10.1.3.7's three bins**, and we logged no S detection of any
kind.

**Detect it spectrally, not on the constellation.**  Conditioning
`V34_RX_STAGE_PHASE4_S` declared S on ordinary data-mode symbols **140 ms**
after being armed -- 170 ms *before* the peer's real S -- and the CP search
then ran through the transition it exists to find.  The §9.6.1.2 responder's
three-bin Goertzel needs neither equalizer, carrier loop nor timing loop; it
now runs for our own renegotiation too (`v34_v90_watch_reneg_s()`), and its
gate on `V34_RX_STAGE_DATA` had to be relaxed because by then the receiver is
in `V34_RX_STAGE_V90_CP` -- so the DATA gate silently disabled exactly the
case the initiating clause requires.  Live: S detected at **+339 ms**, 1:1
with every renegotiation.

**Still not decoding, and the next lead is the equalizer.**  The CP
conditioning deliberately preserves the Phase-3 channel solution (§9.4.2.2
"assumes the channel is static through this seam"), which is right at startup
and wrong here: at a renegotiation that solution was last trained before data
mode, tens of seconds earlier, because the T/3 upstream receiver owns data
mode.  The `[EQ]` trace during a renegotiation's CP stage reads **mag=26.8
against target_mag=1.29** -- about 20x out.  Figure 8's SCR is training
material (scrambled ones, constant modulus), so
`v34_v90_force_reneg_cp_rx()` now starts from a clean equalizer and lets the
SCR train it.  **That alone does not make CP decode** -- four renegotiations,
no CP accepted in the window, §9.6.1's timeout each time.  Open.

**Where it stops now:** the peer's study succeeds and it does not retrain,
but it sends no CP, so after 56000 symbols we take §9.6.1's own timeout and
fall through to the §9.5.1.1 retrain -- the correct, clause-defined
behaviour, and a bounded cost rather than the old hang.  Its
`setV90RateReneg called, rrn type = 0, constel size = 0` is the next thread
to pull.  Initiation stays default off until a renegotiation completes.

**Superseded: whether the peer answers.**  A recording's peer behaves as
recorded whatever we transmit, so this is proven *conformant* and not proven
*answered*.  `ME_V90_RENEG_AFTER_MS=<n>` provokes one on a healthy call
(`tools/soak/v90_reneg_probe.sh`); the rig spent 2026-08-27 in the §34
Ja-parse blocker and did not reach V.90 data mode in 13 calls, so the live
test is owed.  On a peer that implements §9.6.2 this is the better recovery
and should be preferred; the engine tries it first when it is enabled and
falls through to §9.5.1.1 when it is not.

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

## The retrain wire sequence now belongs to the restart seam

Two later gaps had the same cause: the code reset the Phase-2 state, but did
not always select §9.5/§11.5's retrain waveform afterwards.

* Plain V.34 called `v34_start_retrain()` only when `g_state == ME_DATA`.
  A peer may request a retrain during Phases 2-4, and a data-mode retrain that
  is already under way has already moved the engine to `ME_TRAINING`; both
  paths therefore fell through to `v34_restart()`'s INFO0 waveform instead of
  the required 70 ms silence and role-appropriate tone.  Every caller of
  `restart_v34_phase2_locked()` is a §11.5 path, so the retrain waveform is
  now selected unconditionally after the reset.
* V.90 selected the 70 ms silence/Tone B response only at two call sites.
  The failed-Jd/S path and a timed-out §9.6 exchange reset to Phase 2 and then
  emitted INITIAL_PREAMBLE/INFO0d.  Every caller of
  `restart_v90_phase2_locked()` is likewise a §9.5 path, so the helper now
  selects §9.5.1's silence/Tone B sequence itself; callers can no longer omit
  it or apply it twice.

This is protocol logic, not an interop workaround: V.90 §9.5.1.1/.2 and V.34
§11.5.1/.2 both prescribe silence followed by the role's tone, independent of
the phase from which retraining starts.  `v90_engine_replay` now shows the
failed-acquisition recovery entering `tx=V90_RETRAIN_SILENCE`; previously that
same timeout entered INFO0.

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
### Live retest: peer-initiated V.34 now returns to data mode

The first peer-response result in
`artifacts/v34-peer-retrain-response-20260825T224623Z` was invalid.  The
`ME_TX_DISRUPT_*` hook that removed the data carrier kept running for its full
1.5 s even after Tone B moved the engine from DATA to TRAINING.  Internal TX
logs showed Tone A and its reversal, but the hook zeroed them afterwards in
`me_tx_audio()`; the G.711 wire still carried silence.  SmartLink stayed in
`RX_PHASE1_CALL` because the test itself erased our response.

The hook now stops as soon as the peer request leaves DATA and latches off for
the rest of the call.  The latter matters because a successful retrain creates
a new DATA-entry timestamp: without a per-call completion latch the hook fired
again 20 s after every recovery and retrained forever.  Native V.90 exposed two
more holes in the same instrument: its byte-exact G.711 transmitter bypasses
`me_tx_audio()`, and its DATA handover never populated the shared entry epoch.
The raw path now substitutes the law's digital-silence codeword while the hook
is active, with no transcode on the normal path, and both V.90 DATA handovers
set the epoch.

The corrected plain-V.34 run is
`artifacts/v34-peer-retrain-data-20260825T225827Z`:

* The call reached V.42 LAPM at 3000 baud / 9600 bit/s.
* Twenty seconds into DATA, 1.5 s of deliberate carrier loss made the peer go
  `DATAXMIT=>SILENCERETRAIN=>TONE_AB`.
* Our receiver detected Tone B in DATA after 80 ms.  Leaving DATA immediately
  ended the disruption; the wire then carried **69.8 ms** silence, Tone A and
  the §11.2.1.2.3 reversal.
* Phase 2, Phase 3 and Phase 4 completed again.  The engine reported
  `V.34 retrain complete; resuming the existing data link` and re-entered DATA
  **11.21 s after detecting Tone B**.  The peer independently reported
  `finally txbitrate 9600,rxbitrate 9600` and `EXMIT=>DATAXMIT`.
* LAPM was retained rather than restarted.  The peer's DTE received **1019
  numbered lines, contiguous with zero gaps** across the outage and retrain.
  The hook fired once only.

That is a complete, peer-initiated §11.5.1.1/§11.5.2.2 retrain from DATA back
to DATA, not merely a waveform check.

### V.90 live status

`artifacts/v90-peer-retrain-data-20260825T230105Z` establishes that V.90 can
also get through retrains to data on this peer: two peer-initiated Phase-3/4
retrains ran through `V90_RETRAIN_SILENCE` and Phase 2, the second attempt then
completed Phases 3/4 and entered DATA at 19200 bit/s upstream / 50666 bit/s
downstream.  It held an open eye for 112.2 s and delivered **23357 contiguous
U-lines with zero gaps**.

That run reached DATA before the raw-G.711 disruption hook was fixed, so it is
not a DATA→retrain→DATA proof for V.90.  Subsequent attempts to provoke that
exact transition were dominated by the existing intermittent Phase-3 Sd/S
blocker and did not reach initial DATA.  The instrument now reaches the V.90
raw path and is ready for the next successful handshake; plain V.34 supplies
the complete end-to-end DATA→retrain→DATA proof above.

## §9.5.1.1 on V.90 upstream loss: measured over long calls, and now default off (2026-08-27)

The V.90 §9.5.1.1 initiation added earlier reasoned, at the call site, that
"the alternative to a retrain is not 'leave the link alone' -- it is to keep
transmitting downstream into a receiver whose eye is shut for the rest of the
call". Over the 105 s soak schedule that was untestable. Over 600 s it is
**exactly backwards**, because the receiver whose eye is shut is at *our* end
and the downstream is fine.

The six-call long soak of the shipping defaults
(`artifacts/v90-longsoak-233217Z`, `docs/v90_phase3_s_and_rbs_false_positive.md`
§39) connected cleanly every time — first Phase 4 attempt 6/6 — but only one
call *held*, and in four of the five that did not, the first disruption after
data mode was our own detector firing on the upstream while the downstream was
delivering perfectly.

`tools/soak/v90_loss_retrain_ab.sh`, arms alternated, three 600 s calls each,
`ME_V90_RETRAIN_ON_LOSS` the only variable (`artifacts/lossretrain-ab-*`):

| | retrains after data | carried | downstream lines / missing | upstream lines |
|---|---|---|---|---|
| cap 4 (old default) | 1 / 0 / 4 | 373 / 629 / 629 s | 490775 / **491** | 185229 |
| **cap 0** | **0 / 0 / 0** | **629 / 629 / 629 s** | **667084 / 107** | 156321 |

Holding the link delivers **36% more downstream lines with a fifth of the
losses** — all three calls at the ceiling of what the schedule can send
(222378, 222306, 222400) — against **16% fewer upstream lines**, on a direction
that is more than 45% incomplete in *both* arms. The retrain was buying nothing
and costing the clean, faster direction. §9.5.1.1 says the digital modem
**may** retrain at any time, so both policies conform.

**The two caps are now different, and deliberately.** Plain V.34 is symmetric —
one modulation, one receiver each way, so a receiver that has stopped decoding
means half the link is dead and there is nothing to protect by waiting. That
cap stays at **4** (`ME_V34_RETRAIN_ON_LOSS`). V.90 is not symmetric: PCM
downstream at 52000 and V.34 upstream at 31200, failing independently, and on
this rig it is always the upstream, with the frame-phase/eye collapse of
`docs/v90_upstream_data_path.md` that a fresh handshake does not fix. That cap
is now **0** (`ME_V90_RETRAIN_ON_LOSS=4` restores the old behaviour).

**This is not "give up on recovery".** §9.5.2.1 has the *analogue* modem
retrain if *its* receiver fails, and we already follow that. So a genuinely
two-directional failure is still recovered; what is no longer done is tearing
down a working downstream on our own receiver's account.

**Limits worth stating.** Three calls a side, one peer, on an essentially
noiseless bearer where the downstream never fails. On a real line where the
downstream *does* degrade, §9.5.2.1 is what has to catch it, and that path has
no long-call measurement behind it yet. The upstream's 45% incompleteness is
untouched by any of this and remains the open problem.

## §9.6, closed offline: the peer's CP decodes and the renegotiation completes (2026-08-28)

The previous section left "CP does not decode" open, with a measured lead (the
CP conditioning preserves a stale equalizer) and the honest note that starting
from a fresh one did not fix it. It could not have: three defects sat behind
each other, and the fresh equalizer was the second of them.

**The instrument that broke it open is an offline reproduction, which this
problem turned out to have.**

```bash
ME_V90_RENEG_AFTER_MS=20000 ME_V90_RENEG=1 ./v90_engine_replay artifacts/reneg-eq/reneg-r1/live-rx.g711 ulaw
```

runs V.8, Phases 2–4, data mode and the renegotiation off the recording and
reproduces the live failure to the millisecond — the peer's S is at a fixed
position in the audio, so the replay meets it where the live call did. Every
measurement below is from that command, and every one of them was invisible
before, because the engine's logs said *nothing at all* while a renegotiation
ran. Two diagnostics landed with the fix and should be reached for first next
time: a `[ME] V.90 §9.6 CP window` line at the end of the window (bits, sync
candidates, valid frames, and rejects split into CRC / structure / semantic),
and `V90_RENEG_BIT_DUMP=<path>` for the recovered bit stream. The first
reading of the window classified it immediately: **381 sync candidates, 0
valid, 382 rejects, all structural, not one CRC reject** — "the frame boundary
was never even plausible", which is a different problem from "the symbols are
wrong".

### (a) The DATA-mode upstream receiver never let go of the seam

§9.6.1.1.1's S, S̄, SCR and CP are the Phase-4 four-point signals the ordinary
T/2 chain demodulates. The T/3 branch is the *data-mode* upstream receiver,
and it does not stand down on its own: `v90_t3_put_sample()` calls
`v90_t3_emit_ready()` whenever it has acquired, and that calls
`process_primary_symbol()` directly. Instrumented, **all 42496 symbols the CP
stage saw during the renegotiation came from the data receiver** — whose
equalizer is a supervised least-squares fit to B1 over the negotiated *data*
constellation. `|z|` sat at **26.8** against the four-point slicer's unit
circle for the whole window, whatever the T/2 equalizer did. That is why last
session's fresh equalizer changed nothing: it was resetting a filter whose
output nobody was using.

### (b) Nothing was allowed to train the fresh equalizer

`V34_RX_STAGE_V90_CP` freezes blind CMA. That is right at startup, where Phase
3 trained those taps moments earlier, and wrong at a renegotiation, where they
were last trained before data mode. With the freeze lifted, the
decision-directed Phase-4 tracker took ownership on the **third symbol** and
stood CMA down again — and a decision-directed loop is meaningless at 27× the
unit circle. Both now stand aside while the CP conditioning finds the *level*,
on Figure 8's SCR, which is scrambled ones and therefore constant modulus,
i.e. training material. **Level first, then phase.** Measured: `|z|` 1.377 →
1.049 in 1599 bauds. (`phase4_cma_converged()` could not be reused as-is: it
is scoped to `V34_RX_STAGE_PHASE4_TRN`, and a renegotiation has no TRN stage
of its own.)

### (c) The framer only ever saw the fragments a hypothesis lock covered

The bits reaching the Table-14 framer are emitted under `mp_hypothesis >= 0`.
That is a startup arrangement — the V.34 MP search finds the preamble, replays
the frame so far, then streams — and a renegotiation breaks it: SCR
descrambles to a **solid run of ones**, so the 17-one preamble gate reads
18/18 for up to 2000 ms. The search locked and was rejected 91 times inside
the window and the framer was handed only those fragments.

**There is nothing to search for.** The domain is differential, the dibit
transform is the fixed negation (`MP_HYPOTHESIS_DIFF_INVERSE`), the scrambler
is the analogue modem's GPA (tap 4) and the bit order is b0,b1 — all four
fixed by §8.5.2/§10.1.3.3 and the constellation table, not by the channel.
Decoding **every** symbol that way, offline over the same recording's symbols,
recovers the SCR as 1.5 s of unbroken ones and then **47 CP frames, every gap
exactly 700 bits and every consecutive pair bit-identical**. So the
demodulation had been right all along; only the framer's view of it was not.
The framer owns sync, length, CRC and semantics, which is what it is written
to do. Scoped to the renegotiation (`ME_V90_RENEG_CP_STREAM=0` disables);
startup keeps the search.

### Result

The window now reports **59 valid frames, 700 bits, drn=19** — the length the
peer's own log says it built — and the sequence runs to the end: `valid
far-end data-mode CP` → MP′ → `valid far-end CP′` → `Ed (12 symbols)` → `B1d`
→ **`Rate renegotiation 1 complete; data mode resumed after B1d`**.

Scored across the four recordings that carry a peer-answered renegotiation,
one variable, both arms built from the same tree: **renegotiation 1 completes
4/4 with these fixes and 0/4 without**, every baseline arm ending in §9.6.1's
timeout.

**Not established: whether the peer accepts our Ed and resumes.** A
recording's peer behaves as recorded, and this one never received an Ed to
answer — its CP′ in the capture is a genuine response to the plain MP we were
sending at the time, not to an MP′ we never sent. Initiating therefore stays
default off (`ME_V90_RENEG=1`) until a live call says otherwise;
`ME_V90_RENEG_AFTER_MS=<n>` provokes one on a healthy call
(`tools/soak/v90_reneg_probe.sh`).

**Method note, and it is the fourth time in this project.** The failing thing
was ours and the evidence was in our own signal path, not in the peer's
behaviour: the premise that had to be tested was not "does the peer send CP"
but "what do our own symbols say", and the answer came from decoding the
receiver's own dumped dibits with the settings the code itself pins. When a
receiver locks a perfect preamble and then rejects every frame, dump the bits
and decode them outside the receiver before changing anything inside it.

### Verified LIVE: 3/3, and the peer resumes data mode (2026-08-28)

Three calls against the rig with `tools/soak/v90_reneg_probe.sh`, our side
opening a renegotiation 20 s into data mode
(`artifacts/reneg-cpstream-222512Z`). **All three printed `Rate renegotiation
1 complete; data mode resumed after B1d`**, against a standing record of 0 for
4 before this change, every one of which ended in §9.6.1's timeout.

**The confirmation that matters is in the peer's own log, and it is one word.**
Its startup builds `CPnot on **MP** receive`; during the renegotiation it
builds

```
<621.271583> VPcmFloModem (V90): rate renegotiation detected !!
<622.792002> V90Phase4Demodulator: disable linear mapping study      <- our TRN2d graded OK
<622.793334> VPcmFloModem (V90): Building CP, CP length = 700
<623.291438> VPcmFloModem (V90): Building CPnot on MPnot receive     <- our MP-PRIME
<623.452262> V34HSHAKE: txstate EXMIT=>DATAXMIT
```

`MPnot` is MP′, and we only send MP′ after decoding a CP — so the peer is
reporting our half of the fix from its own side. It then returns to
`DATAXMIT`, i.e. it accepts the exchange and resumes data mode, which is
exactly the question a recording could not answer. All three calls ran their
full 105 s soak schedule through the renegotiation with data flowing both
ways.

**One trap, and the control is why it is not in the result.** The peer's DTE
receives nothing for about 30 s in the middle of every one of these calls,
identically to the byte, which looks exactly like a renegotiation cost. It is
not: two control calls with no renegotiation at all
(`artifacts/reneg-control-223605Z`) show the **same 30 s freeze at the same
byte count** (174591), and the renegotiation arm actually froze slightly later
than the control. It is a pre-existing property of the rig's DTE path. A
deterministic outage sitting near a change is not evidence about the change.

**Initiating stays default off** (`ME_V90_RENEG=1`), deliberately. The engine's
own trigger is an upstream receiver that has stopped decoding, and on this rig
that is the chronically broken direction, so turning it on would re-open
exactly what §39 closed when `ME_V90_RETRAIN_ON_LOSS` went to 0: tearing at a
working downstream on the upstream's account. What would settle it is the same
experiment §39 used — 600 s calls, arms alternated, `ME_V90_RENEG` the only
variable, scored on carried time and line counts — not a shorter run and not
an argument.

### Initiating §9.6 on upstream loss: measured over 600 s calls, and it does not pay (2026-08-28)

The §9.6 fix above makes initiating a renegotiation a real option again, so the
experiment §39 used for the retrain on the *same* trigger was run for it:
`tools/soak/v90_reneg_ab.sh`, three repeats a side, **600 s calls, arms
alternated**, one binary, `ME_V90_RENEG` the only variable, no other overrides
(`artifacts/reneg-ab-225015Z`). Note what the control arm is: with
`ME_V90_RETRAIN_ON_LOSS` at 0 since §39, the control does *nothing* on upstream
loss, so this is "cheap resync vs leave it alone", not "resync vs retrain".

**The result is in the completion count, and it needs no line counting at all:
six renegotiations were requested across the three `reneg` calls and none
completed.** Five reached §9.6.1's timeout; the sixth was on a call that died
first.

| arm | calls | data mode | §9.6 requested | completed | carried | downstream lines | upstream lines |
|---|---|---|---|---|---|---|---|
| control | 3 | 3 | 0 | – | 1540 s | 424390 | 196281 |
| reneg | 3 | 3 | 6 | **0** | 1317 s | 328459 | 131625 |

**Why it fails on this trigger when the probe completes 3/3.** The probe opens a
renegotiation on a *healthy* call; the engine's trigger is
`v34_v90_upstream_carrier_lost()`, i.e. a receiver whose eye has already shut.
The CP window reports say the rest: the healthy probe decoded **59** CRC-valid
CP frames and ran CP → MP′ → CP′ → Ed → B1d, while the five loss-triggered
windows decoded **4, 2, 3 and 1** valid frames and never reached CP′ or Ed at
all. So the fix works on this trigger too — CP *does* decode, which it never
did before — but not enough of it survives to carry the exchange, and the
procedure needs a receiver that is at least intermittently working, which is
precisely what the trigger says we do not have.

**What the throughput columns are and are not.** They point the same way, and
they are not worth an effect size: n=3 a side, one dead call in each arm
(`reneg-r2` at 67 s, `control-r2` at 283 s) and one control call that lost
683844 downstream lines on its own account. The completion count is the
finding; the line counts merely fail to contradict it.

**`ME_V90_RENEG` therefore stays default 0.** Enabling it buys ~17 s of
Rd/R̄d/TRN2d/MP on the downstream per attempt, up to
`ME_V90_MAX_RENEGOTIATIONS` (8) times a call, and recovers nothing — the same
conclusion §39 reached about the retrain on the same trigger, for the same
reason: the upstream's eye collapse (`docs/v90_upstream_data_path.md`) is not a
thing a fresh Phase 4 fixes. §9.6 remains fully implemented, verified live 3/3
when opened on a working link, and answered whenever the *peer* opens one.

**What would change the answer** is fixing the upstream collapse itself, not
this knob. A useful smaller step would be to gate initiation on the upstream
being *recoverable* rather than merely lost — the CP-window valid-frame count
is exactly that measurement, and the split here (59 against 1–4) is clean.

### Gating repeat §9.6 attempts, and a correction to how the last one was scored (2026-08-28)

The section above closed with "gate initiation on the upstream being
recoverable — the CP-window valid-frame count is exactly that measurement, and
the split here (59 against 1–4) is clean". **The split was not clean, because
the two numbers were not measured the same way.** 59 came from an *offline
replay* window spanning 13 s and several of the peer's CP bursts; the 1–4 came
from *live* loss-triggered windows. Measured live against live, scoped to the
renegotiation windows themselves:

| case | CP frames decoded | ack pattern | outcome |
|---|---|---|---|
| probe, healthy link (3 calls) | 6 | `0,0,0,0,0,1` | completes |
| loss-triggered (4 windows) | 4, 3, 2, 1 | all `0` | §9.6.1 timeout |

and they arrive at the same instants — +1617…+2158 ms after the peer's S in
the completing calls, +1553…+2015 ms in the failing ones. The peer's own log
is identical in both cases too: `rate renegotiation detected` → 1.24 s study →
`Building CP` → CP #1..#5 → CP′ → `EXMIT=>DATAXMIT`, 2.2 s end to end, whether
we are healthy or not.

**So the count is a side effect and the signal is a single frame.** What
separates the two outcomes is whether §9.6.1.2.3's terminating **CP′** is
decoded — that is the frame that releases Ed. A threshold on the count would
have been asked to separate 6 from 4.

**The gate is therefore:** a §9.6 attempt that takes §9.6.1's timeout without
ever decoding a CP′ is this call's answer, and no further renegotiation is
opened on it. It does not touch a renegotiation the *peer* opens (§9.6.1.2 is
a "shall"), nor a call where an attempt did complete. Retries were measured
not to recover it and to decay — within one call the successive attempts
decoded 4, then 2, then 1 frames — so on the A/B's own data this turns 6
attempts across three calls into 3, halving the Rd/R̄d/TRN2d/MP the downstream
spends on attempts already known to be futile.
`ME_V90_RENEG_VIABILITY=0` restores retries.

Verified both ways offline on `artifacts/reneg-eq/reneg-r1`, three runs each:
the completing path reports `valid=6 cp_ack=1` and the gate stays silent, and
the forced-failure path (`ME_V90_RENEG_CP_STREAM=0`, `valid=0` in 3 of 3)
reports `cp_ack=0` and the gate fires. The CP-window report now carries
`cp_ack` and is printed on completion as well as on timeout — the two outcomes
differ by exactly one frame, and logging only the failures hid that.

**This makes an enabled renegotiation cheaper; it does not make one worth
enabling.** `ME_V90_RENEG` stays default 0.

**The open question this leaves is better posed than the one it replaces:** we
decode the head of the peer's CP burst and lose its tail, including the CP′ —
4 of 6 frames, then 3, then 2, then 1 as the call degrades. That is a receiver
that works for ~300 ms after its SCR-trained equalizer is frozen and then
stops. Whether the taps should keep adapting through the CP burst (they are
frozen at convergence, by the same reasoning that freezes them at startup —
where Phase 3 has just trained them and CP is short) is the next thing to
measure, and unlike the count it is a real mechanism.

**Method note.** Two numbers from different harnesses are not a measurement.
The offline replay and the live call are the same engine on the same samples
and still do not produce comparable window statistics, because the window
boundaries differ. Score an A/B inside one harness, or state the units.


### The viability gate, tested live and withdrawn (2026-08-28)

The gate above was tested with the experiment it deserved —
`tools/soak/v90_reneg_viability_ab.sh`, `ME_V90_RENEG_VIABILITY` the only
variable, **both** arms with `ME_V90_RENEG=1`, 600 s calls, arms alternated
(`artifacts/renegviab-ab-010014Z`) — and **the first informative call refuted
it**:

```
ungated-r3  attempt 1 -> no E within 56000 symbols        (timeout, no CP')
            attempt 2 -> Rate renegotiation 1 complete    (valid=4 cp_ack=1)
```

A retry after a CP′-less timeout **completed**. The gate would have suppressed
it. One counterexample is enough to withdraw a rule of the form "never retry
after X", and there is nothing on the other side: every call in the gated arm
had exactly **one** upstream-carrier-loss event, so the gate never suppressed
anything there and that arm says nothing about it.

| call | §9.6 attempts | completed |
|---|---|---|
| gated-r1 / r2 / r3 | 1 / 1 / 1 | 0 / 0 / 0 |
| ungated-r1 / r2 / r3 | 1 / 0 / **2** | 0 / 0 / **1** |

**The gated arm's better totals are not evidence for it** — 1206 s carried
against 687 s, but two ungated calls died at 40 s and 46 s, so that is call
quality on a rough session, not the variable. Quoting it would be the same
error as the count.

**And the completing retry kills the count framing too.** It completed on
**4** CP frames — inside the 1–4 range every *failing* attempt sat in. So
whether the terminating CP′ lands varies attempt to attempt: it is a
per-attempt condition, not a property of the call, and neither the count nor
the CP′ can be read as "this upstream cannot carry §9.6".

Withdrawn in full. Kept, because they are what caught it: `cp_ack` in the
CP-window report, and printing that report on completion as well as on
timeout.

**Two lessons, both about the same mistake made twice in a row.** The gate was
proposed off a number from a different harness, and when that was corrected it
was rebuilt off a within-call trend from a single call (4 → 2 → 1 frames).
Neither was a measurement. **A rule that forbids something needs a
counterexample search, not a supporting anecdote** — and the cheapest such
search here was one alternated A/B that found the counterexample in three
calls a side.

### Keeping the CP-stage taps adapting: done, and measured neutral (2026-08-28)

The freeze at CMA convergence copied the startup rule, and that rule does not
carry over: at startup the taps arriving at the CP stage were trained by Phase
3 moments earlier and blind CMA can only walk them off, whereas at a
renegotiation they were trained seconds ago on SCR and there is no trained
solution to protect. Both signals in this window are constant modulus —
Figure 8's SCR is scrambled ones, §8.5.2's CP goes out through J's 4-point
modulation — so CMA is legitimate across the whole of it. Changed;
`ME_V90_RENEG_CP_ADAPT=0` restores the freeze.

**It changes nothing, and this is the first time this question has had a
harness that could say so.** Replaying a live *loss-triggered* call
(`artifacts/reneg-ab-225015Z/reneg-r1`) reaches the same renegotiation and the
same window offline — `valid=4 cp_ack=0` against its own live `valid=4
cp_ack=0` — so the failing case is now reproducible on the desk. Three runs
each way:

| arm | result |
|---|---|
| adapting | `valid=4 cp_ack=0`, timeout — 3/3 |
| frozen | `valid=4 cp_ack=0`, timeout — 3/3 |
| adapting, on the completing recording | `valid=6 cp_ack=1`, complete — 3/3 |

**Why it could not have helped, from the same harness: the level was never the
problem.** Dumping the CP-stage symbols of a completing and a failing
renegotiation side by side (`V34_MP_RX_DUMP`, which now carries the absolute
angle):

| | magnitude | angle from the 4-point family |
|---|---|---|
| completes | 1.167 → 1.05 | **~3°** through SCR, 0.6° through CP |
| times out | 1.167 → 1.05 (identical) | **~21.8°** for the whole 1.4 s of SCR |

**22.5° is the mean for symbols distributed uniformly in angle** — it is this
metric's white, not a rotation, the same trap as 0.667 for distance-to-grid.
So the failing receiver has no constellation at all through the SCR it is
given, acquires only at +1.5 s as CP starts, decodes 4 frames, and collapses
again at +2.05 s (|z| = 0.033) exactly where frames 5, 6 and the CP′ are due.
A level-restoring loop cannot supply what is missing there.

**Ruled out on the way:** an early/false S detection, which was the obvious
suspect — both calls put the first CP frame at +1.62…+1.64 s after the peer's
S, so the detection is correctly timed in both and the failing case simply
stops early.

**The lead this leaves is timing, not taps.** The T/2 chain's symbol timing is
frozen for the whole of data mode (the T/3 branch returns before it), and the
failing call had been in data mode for 132 s against the completing one's 46 s
— three times as long for the peer's clock to walk away from a sampling
instant nobody is correcting. A receiver sampling off the eye centre is white
in exactly the way measured here, and it would explain why the SCR — 1.4 s of
free training material — buys nothing. Re-acquiring the symbol instant at the
§9.6 seam, as a fresh Phase 4 does, is the next thing to try.

### Re-acquiring the symbol instant at the §9.6 seam: tried, refuted, not kept (2026-08-28)

The previous section's lead was that the T/2 chain's sampling instant is frozen
for the whole of data mode, so at the seam it is stale against a peer whose
clock has walked — and that a receiver off the eye centre would be white in
exactly the way the failing renegotiation measures. Implemented: at
`v34_v90_force_reneg_cp_rx()`, clear the band-edge detector's accumulators the
way `process_rx_info1a()` does when the rate is selected, and re-arm the T/2
eye-phase chooser, whose per-call flip cap is normally spent by Phase 3 — so
the one mechanism that can fix a half-symbol error was disabled exactly when it
was needed. Figure 8's SCR is constant modulus and up to 2000 ms long, which
is the material that chooser wants.

**It made things slightly worse and its premise is directly refuted.** Three
runs each way on the offline reproduction of a failing call:

| arm | result |
|---|---|
| re-acquire the instant | `valid=3 cp_ack=0`, timeout — 3/3 |
| leave it frozen | `valid=4 cp_ack=0`, timeout — 3/3 |

and the eye chooser *did* act (`moving the symbol instant (flip 1)` after the
re-arm), while the angle through the SCR was **unchanged at 21.8°**. So the
sampling instant is not what the failing receiver is missing. Reverted.

**Correction to the previous section, which called that 21.8° "white".** It is
not. The mean is the same as a uniform distribution's, but the spread is not:
**sd 3.1°, with 2254 of 2700 symbols in a single decile.** That is a tight
cluster — a *stable ~22° rotation of the raw carrier phase*, not an absent
constellation. Two things follow. The dumped angle is `ang1`, taken **before**
the decision-aided derotator, and §8.5.2's CP is differentially decoded, so a
constant rotation is absorbed and is not by itself harmful — which the failing
case demonstrates by decoding 4 CP frames while sitting at 22°. And "mean 22.5
means white" is only true with the spread beside it: **quote the sd, or the
metric will tell you the opposite of what is happening.**

**A second lead, also refuted.** The amplitude dip at +2.0 s (|z| 0.53, and
0.033 in one run) lands exactly where CP′ is due, and
`mp_v90_cp_reset_at_carrier_gap()` discards the Table-14 search on a
carrier-off reading — an obvious candidate for losing precisely that frame.
Counted in the window with FLOW logging on, it fires **2 times in the failing
call and 3 times in the completing one**. Not the differentiator.

**Where this leaves it.** Four mechanisms have now been tested against the
offline reproduction — tap adaptation, the symbol instant, the carrier angle
and the carrier-gap reset — and none separates the two cases, while the level
converges identically in both. What does separate them is what the trigger
already said: the failing call is one whose upstream had *collapsed*, which is
why the renegotiation was opened at all. That is consistent with the A/B
result (§9.6 on the loss trigger: 6 attempts, 0 completions) and with keeping
`ME_V90_RENEG` default off. A renegotiation is a resynchronisation, not a
repair for a line that has stopped carrying the upstream, and no amount of
re-conditioning at the seam changes that.

**The asset from this round is the harness, not a fix:** a live loss-triggered
call now reproduces offline, window for window (`valid=4 cp_ack=0` against its
own live `valid=4 cp_ack=0`), which is what let four hypotheses be killed in an
afternoon instead of four rig sessions.

### §9.6's unanswered-attempt timeout was dead code, and the probe hooks could not run under `--fast` (2026-08-28)

Two defects in the §9.6 machinery, both of them in the way of the work rather
than in the protocol.

**(a) The unanswered-attempt timeout could never fire.**
`V90_RENEG_ANSWER_TIMEOUT_SYMBOLS` exists so that a renegotiation the peer
ignores completely gives up sooner than §9.6.1's seven-second E timeout — but
it was gated on `reneg_rbar_symbol < 0`, i.e. "still transmitting Rd". That
was right while Rd ran until a far-end CPt arrived; since Rd terminates on its
own 384T count (§9.6.1.1.1, fixed earlier in this document) that window is
**48 ms** against the timeout's 3 s, so the branch was unreachable and every
unanswered attempt ran the full E window. Confirmed on the reproduction: an
attempt with no S and no CP anywhere in it logged `no E within 56000 symbols`.

It now measures from the Rd→R̄d transition and is gated on nothing having been
observed — `v90_rate_renegotiation_note_answer()` is called when the peer's S
is detected and when the Table-14 framer takes a sync candidate inside an
armed CP window, so **an attempt the peer answers keeps §9.6.1's full E
window** and only a genuinely silent one is cut short. The bound is 4 s, from
the measured shape of a conformant answer on this rig: S ~260 ms after R̄d and
the first CP frame ~1.9 s after it. §9.6.1 permits the earlier exit —
"the digital modem may initiate a retrain at any time during a rate
renegotiation according to 9.5.1.1" — and its five seconds is a ceiling on how
long to wait, not a floor.

Measured on the offline reproduction (`artifacts/reneg-ab-225015Z/reneg-r1`,
which carries three attempts), one binary, before and after:

| attempt | peer answer | before | after |
|---|---|---|---|
| 1 | S seen, `valid=4 cp_ack=0` | E timeout, 7 s | **unchanged** — E timeout, 7 s |
| 2 | S seen, `valid=0` | E timeout, 7 s | **unchanged** — E timeout, 7 s |
| 3 | nothing | E timeout, 7 s | **no-answer timeout, 4 s** |

The CP-window reports are identical in the answered attempts, which is the
control: this changes when we give up on silence and nothing else.

**(b) `ME_V90_RENEG_AFTER_MS` (and its `ME_V34_*` siblings) measured wall-clock
time, so they never fired under `v90_engine_replay --fast`.** That is the one
harness this whole investigation runs on — a `--fast` replay consumes a 600 s
recording in a fraction of the time, so a 20000 ms probe simply never came
due, and the alternative was 600 s of desk time per experiment. They now
measure **received audio** since data-mode entry (`g_rx_audio_samples`, the
8 kHz media clock), which is what "n ms into data mode" always meant. Live
behaviour is unchanged — the two clocks agree on a call running in real time —
and a `--fast` replay now opens the probe at a deterministic point in the
recording. Note the consequence when choosing a recording: the probe fires at
that point in the *audio*, so a recording whose peer answered a renegotiation
somewhere else will not answer this one.

### Where the peer's CP burst is lost: the wire is steady, the collapse is ours (2026-08-28)

The open question left above was that we decode the *head* of the peer's CP
burst and lose the tail including §9.6.1.2.3's terminating CP′, and the working
explanation was the one the trigger suggested — a call whose upstream had
collapsed. **At the seam that explanation does not hold: the line is steady
right through the window, and the loss is in our receiver.**

Method: `V90_RENEG_BIT_DUMP` on the offline reproduction
(`ME_V90_RENEG=1 ./v90_engine_replay artifacts/reneg-ab-225015Z/reneg-r1/live-rx.g711 ulaw --fast`),
then read the stream *outside* the receiver — the same method that found the
framer defect earlier in this document. The window's first 42624 bits contain,
in order:

- a run of **9238 ones** — Figure 8's SCR, exactly as §9.6.1.1.1 describes;
- **five CP frames at a spacing of exactly 700 bits** (9724, 10424, 11124,
  11824, 12524), pairwise near-identical, which is the peer's repeated CP and
  matches its own log's `CP length = 700`;
- at bit ~13215 — where the sixth frame is due at 13224 — **a run of about
  225 zero bits**;
- and after it, nothing structured ever again: the two later sync candidates
  (13497, 13928) are off the 700-bit grid and their bodies differ from a good
  frame in ~300 of 700 bits, i.e. noise.

So the burst is not truncated by the peer. **The received audio across the
whole window is flat**: 400-sample RMS over the 3.7 s from the CP conditioning
reads 297–325 with no dip whatsoever, and certainly no 35 ms gap where the
zeros are. The peer is transmitting and we stop demodulating.

Two things worth keeping about the zero run. It sits at ~2.06 s into the
window, which is where the earlier section measured `|z|` falling to 0.033 —
the same event. And a run of *exact zeros* out of a self-synchronizing
descrambler means its input was a constant raw dibit for ~112 symbols, not
noise: with the equalized magnitude that small the differential product
underflows and every symbol decides the same way. So the signature to look for
is not "bit errors" but a stretch of the output going quiet.

**What this changes.** "A renegotiation is a resynchronisation, not a repair
for a line that has stopped carrying the upstream" is still the right reading
of the A/B result, but it is not the reason *this* window fails: the peer
answers, sends SCR and five good CP frames over a steady line, and our CP-stage
receiver dies mid-burst 2 s in. The next thing to measure is what collapses —
the tap adaptation is already refuted (frozen and adapting both give
`valid=4`), so the candidates left are the AGC/level normalisation and the
carrier loop, neither of which has been read across this instant.

### Correction: the lost CP′ is three lost RTP packets, and the AGC and carrier loop are innocent (2026-08-28)

**The previous section is wrong on its central claim and is corrected here.**
It reported that the received audio was flat across the §9.6 CP window, and
concluded the collapse was inside our receiver. The RMS scan behind that was
taken at the wrong offset in the file: `v90_engine_replay` starts feeding at
`find_call_start()`, 2.88 s into this recording, so engine sample *n* is file
offset *n* + 23040, and the scan was looking 2.88 s — and, because the arming
instant was taken from the nearest preceding `RX dump mark` rather than
measured, a further 5.5 s — away from the event.

**What is actually there.** The recording contains two runs of digital
silence in the window: 160 samples at file offset 1097920 and **480 samples at
1102560**, 4640 samples apart. The receiver's own feed shows exactly the same
two, 64 and 192 symbols long and 1856 symbols apart — the same 4640 samples —
so the mapping is now pinned by the data rather than assumed. And
`rtp-rx.csv` names the cause: in **32868 packets the call has exactly one
anomaly**, at `seq=21476 timestamp=1102400 seq_delta=4 ts_delta=640` —
**three lost packets** — landing on the frame where §9.6.1.2.3's terminating
CP′ is due.

**The AGC and the carrier loop, read across it** — which is what the new
`V90_RENEG_SYM_DUMP` exists for. It writes one line per CP-window symbol:
symbol, raw dibit, |z|, absolute angle, `agc_scaling`, `eq_target_mag`, the
carrier loop's frequency, the input power meter, the RMS of the block of line
samples the receiver was handed, and the Godard loop's accumulated timing
correction.

| quantity | before the gap | during | after |
|---|---|---|---|
| input block RMS | 1254 | **0.0** | 1234 |
| input power meter | 1.55e6 | **0** | 1.53e6 |
| `agc_scaling` | 0.001812 | 0.001812 | 0.001812 |
| carrier frequency | 1828.571 Hz | 1828.571 | 1828.571 |
| `total_baud_timing_correction` | −8 | −8 | −8 |
| \|z\| | 1.002 | 0 | 0.904 |
| angle to the 4-point family | **0.92°** | — | **13.9°, then 22.5°** |

So **neither loop moves at all**, and neither is capable of moving: the AGC's
adaptation is inhibited on silence by its own `power > 10` gate, and the
carrier loop's error term is `sym->im*eq_target.re − sym->re*eq_target.im`,
which is identically zero on a zero symbol. Freezing the taps as well
(`ME_V90_RENEG_CP_ADAPT=0`) changes neither the angle nor the frame count.
What the gap costs is the *constellation*, and it never comes back: 22.5° is
the mean for symbols distributed uniformly in angle, i.e. this metric's white,
for the remaining 14000 symbols of the window.

**Read the angle, not the magnitude.** |z| after the gap is 0.904 — within
10% of where it was — while the phase is white. A level check cannot see this
failure at all; the distance of the absolute angle to the nearest 45°+90°*k*
separates the two cases cleanly (0.92 against 22.5).

**A guard was built and it is default off** (`ME_V90_RENEG_CP_REACQUIRE=1`
enables). The reasoning is sound — the CP conditioning's acquisition, a fresh
equalizer trained on Figure 8's SCR, is one-shot, so after a dead stretch
there is nothing left that can re-acquire while the peer is still repeating CP
— and it exposed a real sub-defect on the way: **`reneg_cp_train` is not the
"acquisition finished" signal.** With `ME_V90_RENEG_CP_ADAPT` at its default
the taps keep adapting through the CP burst and that flag is *never* cleared,
so a guard gated on it can never fire. A separate sticky `reneg_cp_settled`,
set when the level first settles, is what the guard now tests. But the guard
does not do what it was built to do: it does not restore the constellation
(22.5° either way), and the only positive is a second window on one recording
(0 → 2 CRC-valid CP frames, 0 → 11 sync candidates). Fired unconditionally on
every silent block it is actively destructive — 23.2° from the 4-point family
for the whole window and 0 valid frames, against 1.5° and 4 with it left
alone — because it re-arms during acquisition and destroys the convergence it
is meant to restore. One recording is not a measurement, so it stays off.

**Where this leaves §9.6 against this rig.** The peer answers, sends SCR and
five CRC-valid 700-bit CP frames over a working line, and a single three-packet
loss on the CP′ frame ends the window — not because any loop drifts, but
because 192 symbols of concealed silence take the constellation and nothing in
this stage can acquire twice. Recovering from that, rather than avoiding it,
is the open work.

### Is the CP′ loss general? No — one window in the whole corpus (2026-08-28)

The previous section traced a failing §9.6 window to three lost RTP packets on
the CP′ frame. The obvious next question is whether that is what §9.6 failure
*is*, so every reneg recording in `artifacts/` was scanned two ways: its
`rtp-rx.csv` for gaps, and a replay for what the CP window itself saw. The CP
window report now carries `dead_blocks`/`dead_samples` — blocks of digital
silence handed to the receiver while the window was armed, counted in the
engine because **a recording cannot supply this**: the tap is written before
the feed, so a concealed loss and a quiet peer look identical in the file.

**The answer is no.**

| recording | trigger | CP windows | dead blocks |
|---|---|---|---|
| `reneg-cpstream-222512Z` r1–r3 | probe | `valid=6 cp_ack=1` ×3 | 0 |
| `reneg-defaults` r1–r3 | probe | `cp_ack=1` ×3 | 0 |
| `reneg-mp` r1–r4, `reneg-s` r1–r4, `reneg-s2` r1–r3, `reneg-s3` r2, `reneg-s4` r1–r3 | probe | `cp_ack=1` ×15 | 0 |
| `reneg-trn2d` r1–r4 | probe | `valid=15 cp_ack=1` ×4 | **162 (3.24 s)** |
| `reneg-ab-225015Z` reneg-r1 | upstream loss | `valid=4 cp_ack=0`, `valid=0 cp_ack=0` | **4 (0.08 s)**, 0 |
| `reneg-ab-225015Z` reneg-r3 | upstream loss | `valid=3 cp_ack=0`, `valid=1 cp_ack=0` | **0**, 0 |

Every probe-triggered renegotiation in the corpus completes — 21 windows,
`cp_ack=1` in all of them — and every loss-triggered one fails. **The packet
loss is unique to reneg-r1's first window.** `reneg-r3` fails in exactly the
same way, twice, on a bearer with no loss at all and no silence in either
window, so loss is a *sufficient* cause for the one window it appears in and
not the explanation for §9.6 failing on the upstream-loss trigger. What those
have in common is still what the trigger says: the upstream had collapsed.

`reneg-trn2d` is the useful control on the other side — **162 dead blocks,
3.24 s of silence in the window, and it completes anyway** with 15 valid CP
frames. Dead line in the window is not by itself fatal; what killed reneg-r1
was 192 symbols of it landing *inside the CP burst*, on the frame that ends
the exchange.

**Two method notes.** Segment an RTP trace by SSRC before believing a gap: a
`server.log` and its trace hold several calls, and unsegmented these files
report "618,648,847 lost packets" where the truth is a new SSRC. Of the
apparent anomalies across all reneg traces, all but one are call boundaries or
multi-second media pauses; the single mid-call loss is reneg-r1's three
packets. And the probe hooks needed one more fix to run this at all:
`enter_v90_data_locked()` is the native V.90 data-entry site and it sets
`g_v34_data_entry_ms` too — missed when these hooks were moved onto the media
clock, so `g_v34_data_entry_samples` stayed 0 there and every V.90 probe fired
the instant data mode was entered, before the peer had renegotiated anything.
The symptom was "no CP window" on recordings that demonstrably contain one.

### Why reneg-r3 fails on a clean bearer, and one default that changes (2026-08-29)

`reneg-ab-225015Z/reneg-r3` is the counterexample from the previous section:
two §9.6 CP windows, `valid=3 cp_ack=0` and `valid=1 cp_ack=0`, with no RTP
loss and no dead line at all. It has two separate answers.

**Its second window fails because the CP-stage taps keep adapting, and that
is now the default no longer.** `ME_V90_RENEG_CP_ADAPT=0` turns that window
into `valid=6 cp_ack=1` and `Rate renegotiation 1 complete`. The knob was
introduced default-on with a sound-sounding argument — both signals in the
window are constant modulus, so blind CMA is legitimate across the whole of
it — and a measurement that found it neutral; **that measurement was one
recording.** Over all 26 reneg recordings in `artifacts/` (28 CP windows,
one binary, the knob the only variable):

| arm | windows completing |
|---|---|
| adapting (old default) | 24 / 28 |
| frozen once the level settles | **25 / 28** |

Neutral on 27 windows, one failure turned into a completion, **nothing worse
anywhere**. So the default is now to freeze, `ME_V90_RENEG_CP_ADAPT=1`
restores adapting. CMA is phase-blind: when it is not needed it does exactly
what the startup path's own comment says, keeps walking a solution that was
already right.

**Its first window fails for a different reason, and the peer is not it.**
The acquisition never holds still: the absolute constellation reads 1.3° from
the 4-point family at symbol 1000, **23–26° (white) at symbols 3000–4000**,
back to 7–9° at 5000–6000, on a feed whose RMS never leaves 1216–1254. The
SCR shows it directly — where a completing call descrambles SCR to **one
unbroken run of 9433 ones**, r3's breaks into runs of 3560, 2731 and 2090
with false syncs between them, i.e. real symbol errors during the training
material. Then 5 CP frames land on the 700-bit grid, only 3 pass CRC, and
the frames after them are off-grid (gaps 572 and 705 against 700) and
structurally rejected. Freezing the taps does not change this window at all
(`valid=3` either way) — the wander is not CMA.

**The peer did its part.** `v2attempt1/slm.log` for that renegotiation reads
`rate renegotiation detected !!` → `Building CP, CP length = 700` →
`Building CPnot on **MPnot** receive` — it decoded our MP′ — and then
`EXMIT=>DATAXMIT` **0.16 s later**. Two things follow. Its CP′ was on the
wire for only ~500 symbols, about 1.4 frames, so a receiver that is wandering
gets one or two attempts at it and no more. And the white stretch that fills
the rest of our window afterwards is **the peer's resumed data-mode signal
seen through a 4-point slicer**, not a receiver collapse: the peer considers
the exchange finished and has moved on while we sit out §9.6.1's seven-second
E timeout. Comparing the logs of a failing and a completing renegotiation on
the `MP` / `MPnot` wording alone does not separate them, though — the
completing `reneg-defaults` r2 reads `on MP receive` — so do not use that as a
success marker.

**What is left.** The remaining question for r3's first window is why the
CP-stage acquisition wanders between 1° and 26° during 2.5 s of constant-
modulus SCR on a steady line, when the completing calls hold it. That is the
same "acquire once, no way back" shape as the previous section, and neither
tap adaptation nor the line explains it.

### It is not the acquisition wandering: five phase slews arrive on the wire (2026-08-29)

The previous section left "why does the CP-stage acquisition wander between 1°
and 26° during 2.5 s of constant-modulus SCR" as the open question.
**Correction: nothing in this receiver wanders.** `V90_RENEG_SYM_DUMP` gained
four columns for this — the input sample counter (`qam_sample_time`), the
carrier NCO's phase, and the equalizer main tap's angle and magnitude — and
they are what makes "ours or the wire" decidable without leaving the receiver.
Over `reneg-ab-225015Z/reneg-r3`'s first CP window:

| quantity | across the whole window |
|---|---|
| `agc_scaling` | 0.001812, unchanged |
| carrier frequency register | **bit-identical**, 1828.571411 Hz, first sample to last |
| NCO phase against an exact ramp | residual ≤ 0.07°, **not one step above 1°** |
| symbol clock | 2/3 samples alternating, **zero cumulative drift** over 7200 symbols |
| `total_baud_timing_correction` | −8, never moves |
| equalizer main tap | magnitude 0.83421, angle −0.03°, constant to five decimals |

So the carrier loop never fires, the timing loop never fires, the AGC never
moves and the taps are frozen. What actually happens is **five discrete
carrier-phase slews of +21.7° to +24.2°, each completed in 3 to 10 symbols**,
at symbols 382, 2181, 3556, 4865 and 6466 — with the phase flat to
0.002°/symbol between them and the cluster tight throughout (sd 1.5–3°).

**Folded into "distance to the 45° family" that reads as a wander through 23°
and 36°, which is what the previous section reported.** It is a *stepping*
constellation, not a smeared one: quote the spread, or this metric describes
the opposite of what is happening — the same trap this document has recorded
once already.

**The control says it is this recording and not the procedure.** The same
detector over a completing call's CP window: `reneg-defaults/reneg-r2` —
**zero** mid-window slews (one at symbol 43, during initial acquisition);
`reneg-cpstream-222512Z/reneg-r1` — clean to symbol 6904, where its window
ends and the peer's data mode resumes. Only the failing recording has them,
and it has five.

**And the last one lands where it does the damage.** Symbol 6466 is bit 12932,
inside frame 5 of the CP burst (12584–13284) — the frame immediately before
where §9.6.1.2.3's CP′ is due — and the frame grid breaks exactly there: the
next sync is at 13156, **+572 instead of +700**.

**Not established: where the slews come from.** With every element of the
receiver provably static across them they are in the received signal, but the
mechanism is open. One symbol of carrier phase at 3200 baud and 1828.571 Hz
is 205.7°, which folds to **25.7°** — close to the measured 21.7–24.2 — so a
one-symbol slip in the peer's transmitter is the obvious candidate and this
peer is already known to slip. Against it: a symbol slip displaces the bit
stream by 2 bits, and the grid break measures 128. A lead, not a conclusion.

One more thing seen and not explained: at symbol ~5500 the main tap jumps to
exactly 1.00000 at 0.00°, which is the signature of an `equalizer_reset()`,
and the call logs only two S detections in total, so it is not a second CP
conditioning.
