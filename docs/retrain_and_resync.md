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
| §11.6 | plain V.34 | **not implemented** |
| §11.6.1.2 respond | plain V.34 | **not implemented** — the detector exists, the S/S̄/TRN/MP transmit response does not |

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

* **No live verification.**  Every figure here is offline.  Whether the
  SmartLink peer answers our Tone B from data mode within its own timeout is
  not known, and the rate at which a retrain recovers a call rather than
  ending it is not known.
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

§11.6 for plain V.34 is the remaining gap.  The detector above is
role-independent in principle, but the plain V.34 *response* is its own
S 128T / S̄ 16T / TRN / MP transmit sequence rather than the Phase 4
transmitter the V.90 path reuses, and that does not exist.
