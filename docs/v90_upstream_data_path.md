# The V.90 upstream data path, digital side

How a byte the analogue peer's DTE sends becomes a byte on our PTY, what
each stage assumes, and which of those assumptions have been measured
rather than believed.  Written against the d-modem/slmodemd rig on tower
(see the interop memory for how to run it), PCMU, 3200 baud low carrier,
52000 downstream / 31200 upstream.

## The chain

1. §9.4.2.4 the peer finishes CP′ and sends the 20-bit E sequence.
2. `v90_live_cp_bit()` counts 20 consecutive ones on the CP bit stream and
   calls `v34_begin_rx_data()`.
3. §9.4.2.5 the peer sends B1 -- one data frame of scrambled ones at the
   negotiated rate, using the data-mode modulation parameters -- and then
   begins data transmission.
4. `v34rx.c`'s T/3 branch (9.6 kHz for 3200 baud, three samples per symbol)
   correlates the wire against a locally generated B1 and least-squares
   fits a 7-tap filter to it.  That filter *is* the upstream equalizer;
   there is no adaptation after it.
5. Symbols go to `v34_put_mapping_frame()`, which shell/trellis decodes
   them, descrambles, and pushes bits at the data stack, which frames them
   as V.14 characters onto the PTY.

## What is measured, and how

Three probes live in the T/3 path, all gated on `v90_t3_acquired`:

- **Decision error per symbol** against the odd-integer grid, with the mean
  symbol power beside it.  Live it reads 0.67 against 715 and is flat for
  the whole call.  **Read this against the minimum distance, not against
  the total power.**  Compared to the power it looks like 30 dB and reads
  as "the waveform is fine"; but the constellation spacing is 2, so a
  mean-square error of 0.67 over two dimensions is 0.58 rms per dimension
  against a decision half-distance of 1.0 -- only about 1.7 sigma, i.e. a
  raw symbol error rate of several percent.  That is marginal, not clean,
  and it is the leading suspect for the remaining fault.  It is also
  consistent with the acquisition fit: 98.6% of the energy matched means
  1.4% residual, about 18 dB.
- **Descrambled ones-fraction under both clause-7 polynomials.**  An idle
  DTE sends marks, so the correct descrambler yields ones and the wrong
  one yields white.  Both reading 50% means the bits were already white
  going *into* the descrambler, which rules the scrambler out entirely.
- **The first 96 published bits**, which say whether the receiver is still
  inside B1 or past it.

`v34_data_test` covers the other half: exact-Q9.7 symbols in, bits out, at
every symbol rate and every legal N and trellis (390 cases).  If that
passes and the live path fails, the decoder is sound and the fault is in
the state around it.  That is exactly what happened.

## The traps this path has already fallen into

**B1 is not where the E anchor says it is.**  E is detected on the CP *bit
stream*, which lags the wire by far more than B1 is long (B1 is ~90 ms).
A search window anchored tightly on E therefore never contains B1, and
every template scores about 5% -- which reads as "wrong template" and
sends you hunting through scramblers and trellises that were never the
problem.  The T/3 branch now starts capturing when the upstream is *armed*,
keeps 3.4 s of ring, and searches 2.5 s back and 0.5 s forward of the
anchor.  Acquired: coarse 96.5%, fit 98.6%.

**Arming on an acknowledged CP′ is too strict.**  This peer's CP′ is not
always decoded, and without `v34_v90_prepare_upstream_data()` the whole
upstream runs on default parameters.  The data-mode CP carries everything
the receiver needs; its acknowledge bit is about the handshake.

**Frame phase is decoder state.**  The shell decoder's position within the
data frame matters, and a data frame is `8*p` symbols -- 128 at 3200 baud.
A post-B1 flush that is a whole number of *mapping* frames but not of
*data* frames leaves the receiver permanently out of phase, and the
symptom is white output with a perfect constellation.  The flush is one
whole data frame.

**Acquisition is one-shot.**  `v90_t3_acquisition_attempted` means a single
failure kills the upstream for the rest of the call.  Worth revisiting if
a peer's B1 is ever marginal.

## Spec versus wire

§6.5/V.90 gives the analogue modem GPA and §5.3 gives the digital modem
GPC.  Measured against this peer, Phase-4 TRN descrambles to 100% ones at
GPC's tap and about 54% at GPA's -- so the Phase-4 receive path keeps the
*measured* tap when it is that decisive, rather than the role rule.  B1,
by contrast, fits at GPA, which is what §6.5 asks for.  Acquisition
therefore builds both scramblers (and all three trellis sizes) and keeps
whichever fits the wire: the cost is one call's worth of nothing, and the
alternative is betting the entire upstream on an assumption that this peer
has already been caught breaking once.

## The frame-phase fix did not resolve the white upstream

Tested live after landing it: the publish point is now a whole data frame
past B1, and the first published bits are still white (`1FC01B46
EAB7C72D 56B1F004`), with both polynomials at 50%.  So data-frame phase
was a real defect but not the one that whitens the output.

**And the decision error is not a noise figure at all.**  0.67 across two
dimensions is exactly 2 x 1/3, which is what symbols distributed
uniformly over a lattice cell of spacing 2 give.  The data-era symbols
therefore bear *no relation to the constellation* -- they are not noisy
points on it.  Two follow-ups, both run live:

- **Decision-directed NLMS on the seven equalizer taps** moved the
  decision error not at all.  If the residual were linear distortion
  those taps could invert, it would have.  (Left off by default:
  adapting towards meaningless decisions cannot help.)
- **A 33-point gain sweep**, distance-to-grid at scales from 0.40 to
  2.00, is flat at 0.65-0.68 everywhere.  No scaling puts these symbols
  on the lattice, so V.34's per-rate modulation factor -- which applies
  to data but not to the B1 fit that set our scale, and was the obvious
  suspect -- is ruled out.

**The B1-era probe then landed, and it moved the fault.**  The same
distance-to-grid measure over the suppressed B1 symbols reads **0.641**
-- indistinguishable from the data era's 0.66.  But those very symbols
are the ones acquisition fits to the B1 template at 98.6%, and the
template is verifiably on the lattice (dump a mapping frame at
3200/31200: `23, 21, 3, -13, ...`, odd integers in the /128 domain).

So the symbols are on the lattice where acquisition measures them and
off it where `process_primary_symbol()` measures them, in *both* eras.
Nothing changes at the B1->data boundary at all.  **That rules out
precoding**, which an earlier revision of this note proposed on the
strength of the data-era number alone: a per-era transform cannot
explain a corruption that is already present during B1.

What sits between those two measurement points is the DATA symbol path:
`phase4_da_derot` and the decision-directed carrier tracker that drives
it, then the conjugate/rotation/scale transform.  Acquisition explicitly
zeroes all of them, so any of them moving is the corruption.  The DD
tracker is the only one that moves on its own, and it slices against the
same lattice, so if it mis-locks it will rotate the constellation off
the grid and hold it there.  It has a kill switch:
`ME_V34_DATA_CARRIER_TRACK=0`.  A run with it off is the next
measurement -- 24 attempts on the rig have not reached data mode since,
so it is queued rather than answered.

Ruled out along the way, all measured rather than argued: the scrambler
polynomial (both give 50%), the trellis size (all three swept at
acquisition), the data-frame phase (fixed, retested, still white), the
carrier (low, matching INFO1d), the decoder itself (390 exact-symbol
cases, every symbol rate, bit-perfect), a gain error (33-point sweep
flat), the equalizer taps (DD-NLMS moved nothing), and now precoding.

A note on the probes themselves: each of the three readings above was
first read wrongly, and each wrong reading cost a round of work.  0.67
against the mean power looked like 30 dB; against the lattice spacing it
is "no relation to the constellation".  And the data-era number alone
looked like a statement about the B1->data boundary; beside the B1-era
number it is a statement about our own symbol path.  Take a new probe's
first reading as a hypothesis, not a result.

## Status

Bidirectional data is live and measured (PCMU, 52000/31200):

- downstream 39025/39025 pattern lines, 0 missing, 100.0% clean, ~40 kbps
- upstream   342817 bytes to the PTY at ~25 kbps, B1 acquired at 98.6%

The upstream *payload* is **not** yet correct: the bytes flow, at a
steady 25 kbps in all three soak phases, but they are white rather than
the peer's data.  The frame-phase fix landed and was then tested live --
it did not change that, for the reasons above.  Downstream is unaffected
and stays at 97-100% clean.

The soak harness (`tools/soak/`) runs the whole thing unattended and
`soak_analyze.py` reports per-phase throughput and per-line integrity in
both directions.  Budget for the rig losing the known Phase-3/4 retrain
lottery: reaching data mode took 5 attempts in one batch and more than
24 in another, with no change in between.


## The rate was the first half of the answer (2026-08-20)

The upstream had been running at 31200 because that is what the line
probe supported *downstream*.  Capping what our MP offers, and preparing
the receiver for the same cap, walks the decision error straight down:

| upstream | decision error | note |
|---|---|---|
| 31200 | 0.67 | = 2 x 1/3: no relation to the lattice at all |
| 19200 | 0.53 | GPA first shows over GPC -- 58% ones against 50% |
|  9600 | 0.106 | B1's own era 0.081; symbols are on the grid |

`ME_V90_UPSTREAM_MAX_BPS` caps both the MP mask *and*
`v34_v90_prepare_upstream_data()`.  Capping only the mask is worse than
not capping: the peer transmits at the capped rate while the B1 template
is still built for the uncapped one, and acquisition never correlates.

Two things fell out of getting the symbols clean:

- **B1 is not one data frame.**  Measured on the wire (the symbols stop
  matching the template) it is **two** at this rate.  That matters because
  10.1.3.1/V.34 has B1 carry the superframe inversions of the *final* data
  frame -- `v34_begin_rx_data()` parks the receiver at j-1 for exactly that
  -- and 9.4.2.5/V.90 has the peer start a new superframe afterwards.  With
  B1 two frames long the peer stays parked for both while we advance, so
  the superframe phase is wrong from the first data frame onward.  The
  receiver now finds B1's end on the wire and holds the phase until then.
- **57% ones is a phase signature, not noise.**  With clean symbols the
  descrambled idle stream held 55-56% ones for a whole call.  That is what
  a superframe phase off by k gives: one data frame in j decodes correctly
  and the rest are noise -- 1/7 x 100% + 6/7 x 50% = 57%.  A correctly
  framed idle stream reads near 100%.  j is only 7 wide, so the phase is
  now searched against the marks rather than derived.

## Where it stands, and the next thing to fix (2026-08-20)

With the rate capped to 9600 the upstream decodes **the peer's actual line
state**: the descrambled idle stream reads 100% ones against the wrong
descrambler's 50%.  That is the whole chain -- filter, symbols, trellis,
shell, descrambler -- working.  It then goes to noise about fifteen
seconds in, while the peer's DTE is still idle (the soak's phase A runs
thirty-five seconds), so this is a receiver that acquires and then drifts
off, not one that fails on payload.

**The T/3 receiver has no timing recovery.**  `v90_t3_next_symbol += 3`,
for the life of the call: exactly three samples per symbol, forever, with
nothing to correct it.  Anything that moves the peer's symbol clock
relative to our 8 kHz bearer accumulates without limit -- a few ppm of
clock offset, or a single sample inserted or dropped anywhere in the RTP
path, which shifts the grid by a third of a symbol instantly.  It also
explains why some calls read 50% from their very first window while
others run correctly for seconds first: that is the size of the offset,
not luck about anything structural.  `clock_recovery.c` exists in this
tree for exactly this reason on the other side of the call.

The next piece of work is therefore a timing error detector on this
branch -- Gardner or Godard on the T/3 stream, adjusting the symbol
instant -- or, failing that, letting the supervised filter follow the
drift: its 21 taps span seven symbols and can absorb a fraction of a
symbol of delay if the adaptation is allowed to track it
(`ME_V90_UPSTREAM_DD_MU`, currently 0.02, with a gate wide enough to
fire).

Also ruled out, all measured: bit order (`ME_V90_UPSTREAM_BIT_ORDER=lsb`
breaks the idle decode too, so MSB-first is right); precoding, non-linear
coding, shaping, trellis and rate (the peer names every one in its own
log and they match what we prepare); and a payload bit permutation (no
periodicity at any frame lag, where a permutation of a 90-bit-periodic
pattern would keep some).

## Timing recovery (2026-08-20)

`v34_gardner.h` is the loop; `v34_gardner_test` is what it is held to.
Three things about fitting Gardner to this receiver were not obvious, and
each cost a wrong version first:

- **The signal is the equalizer output, not the fixed RRC.**  The
  supervised filter is fitted by least squares onto B1, so it *is* the
  matched filter here, and its delay sits wherever the fit left it inside
  21 taps.  A detector reading the RRC stream reported -0.40 on the clean
  loopback with the true error at zero, and inserted symbols on a channel
  with no drift.
- **The actuator has to be continuous.**  Whole samples move the instant;
  the leftover fraction is an interpolation weight.  Without the fraction
  the quantum is a third of a symbol, the steady-state error is up to a
  sixth of one, and the integrator never stops seeing it -- 984
  corrections over 6000 symbols of a perfectly timed signal.
- **Do not shift the taps when the instant slips.**  It looks like the
  right compensation and it cancels the correction: the loop stops seeing
  its own effect and winds the integrator to its clamp.  The position is
  `next_symbol + acc` and `acc` gives up a whole sample exactly when one
  is handed back, so a slip is already continuous.

The gains are slow on purpose (mu 0.005, beta 5e-6).  Gardner carries
data self-noise on a dense QAM -- about +/-0.2 symbol to symbol with the
true error at zero -- and the loop is not asked to acquire phase.  The
equalizer owns phase; the loop only has to catch a clock offset before it
becomes a symbol.

### A drift regression that is not in the suite yet

The obvious test -- resample the loopback wire by a few ppm and require
the payload to survive -- was written and then withdrawn, because at the
lengths where drift matters it measures the harness rather than the
receiver.  Two things get in the way: `test_v90_upstream_t3_case` zero-fills
whenever `v34_tx` produces less than a block, so a six-second run decodes
silence near the end (errors began at bit 118306 of 128304 with the loop
both enabled and disabled, i.e. identically), and at two seconds a
realistic offset is a fraction of a sample, which the equalizer absorbs
whether or not anything is tracking it -- 20 ppm passed with the loop
disabled.  A useful version needs a transmitter that sustains a long run,
and then the window has to be taken at the *end* of the capture: drift is
cumulative, so checking the first second is exactly the blind spot that
let a receiver with no timing recovery at all look healthy for years.

## The upstream carries payload (2026-08-20)

The frame phase is ambiguous in **two** dimensions and only one of them
was being searched.  B1 does not tell us the peer's superframe counter,
and its end pins the position *within* the data frame only if B1 ends
exactly where we think it does.  Sweeping the seven superframe phases
alone enumerated cleanly on a live call -- 0 through 6, one window each
-- and none of them decoded, with the symbols clean and B1 matched at
98.6%.  Sweeping the (superframe, data-frame) pair locks: p*j = 112
candidates at a 0.4 s window, under a minute, which fits while the peer's
DTE is idle or trickling.

Two details make the search safe.  The candidate is handed to the decoder
and applied at *its* data-frame boundary, where the phase is the only
thing that changes -- an earlier version reset the frame state from the
emitter, which also zeroed the mapping-frame position mid-frame, so each
step shifted alignment by part of a frame and the walk wandered (5, 2, 5,
2, 6) instead of enumerating.  And the candidate comes from a counter,
not from "current + 1": the decoder advances between the window that
measures and the boundary that applies, so stepping from the current
phase is a walk, not a sweep.

Live, 9600 upstream, peer trickling 200 B/s:

- downstream 38830 of 39025 pattern lines, 99.5% clean
- **upstream 1329 pattern lines in sequence, one missing** -- the peer's
  own data on our PTY

Across five consecutive calls the sweep locked six times (0, 1, 2, 35,
102 and 128 steps in, at 78% to 100% ones) and three of the five
delivered payload.  Finding a lock is no longer the hard part.

**Read the line counts, not the byte percentages.**  The "% clean" figure
counts every byte on the PTY including the garbage the receiver emits
while it is unlocked and sweeping, so a call that delivered 1329 of 1330
lines can read 35% clean.

### Open: holding a lock

A lock holds about twenty-seven seconds -- thirteen consecutive windows
at 100% ones -- and then decays, with the Gardner slip count going from 7
to 12 in a few seconds.  A lock is now released and re-swept when the
ones-fraction falls back, which stops one bad patch ending the call, but
the underlying question is why a settled receiver starts slipping several
times a second.  The ones-fraction is also only a valid lock metric while
the peer's line is mostly idle; at a full send rate it would need
replacing with something content-independent, and the V.14 framing score
that `tools/soak/upbits_align.py` computes offline is the obvious
candidate.


## Why it started slipping after twenty-seven seconds

Two runs settle it, and neither needed a guess:

| peer's DTE | result |
|---|---|
| silent | 98-100% ones for 40+ s, **zero slips**, freq at noise level (+/-3e-5, about 10 ppm) |
| sending | 100% ones until its first byte, then 89, 58, 50 -- slips 7 -> 12 in a few seconds |

So there is no clock drift on this path worth the name, and the trigger
is the peer's payload rather than elapsed time.  The decisive number is
the symbol error: **0.10 while decoding at 100% ones, and still 0.10 at
t=100 s with the bit stream white.**  The signal was never the problem.
What is lost is frame alignment, and the only thing that moves it is the
timing loop -- three slips in one direction shift the symbol clock a
whole symbol against the transmitter, which the frame-phase sweep cannot
undo because it searches mapping frames, not single symbols.

The slips were spurious.  Gardner's error means something only while the
symbols do; once they do not, the detector reports bias rather than noise
and the loop chases it.  Measured on one call: the integrator pinned at
its -0.002 clamp, 225 corrections requested in forty seconds, on symbols
sitting at 0.67 -- the "no relation to the lattice" figure.  Three
changes came out of that:

- A whole-sample step needs the position to stay beyond half a sample for
  200 symbols, about 60 ms, rather than the instant it first crosses.
- The loop *holds* when the symbols are not on the lattice.  It has to
  hold rather than have its correction ignored: dropping the returned
  correction while the loop had already wrapped its own accumulator moved
  the sampling position by a whole sample, which is the opposite of
  leaving it alone.
- The gate is symbol quality, **not** the frame-phase lock.  Gardner is
  non-data-aided, so gating on the lock froze it exactly when it was
  still useful -- six consecutive calls then never locked at all, sitting
  at a symbol error of 0.2 where the calls that lock read 0.10.

### What this surfaced

Calls fall into two populations.  Some acquire symbols at 0.10 and decode
once the frame phase is found; others sit at 0.2, or at 0.67 and never
reach the lattice at all, and no frame phase can rescue those.  That is
acquisition quality -- the B1 fit and the sampling instant it leaves --
not timing, and it is the next thing to chase.


## The upstream symbols leave the constellation about ten seconds in

The five-minute goal runs isolated the remaining blocker, and it is not
the timing loop.  On call after call the symbol error sits at 0.10 for
nine to thirteen seconds and then jumps to about 0.65 -- the figure for
symbols bearing no relation to the lattice -- and stays there for the
rest of the call, four minutes of it.  Ruled out by experiment:

- **The timing loop.**  With the symbols gone it holds still by
  construction: freq frozen, zero slips.  Watching it do so is what
  showed the collapse is upstream of it.
- **Our own transmitter.**  `SOAK_PTY_RATE=0` silences the downstream
  pump; the collapse still happened at about twelve seconds, so it is not
  echo of what we are sending.
- **The peer retraining or renegotiating.**  Its own log shows it settled
  in data mode throughout, Error Energy about 8 and timing offset -0.7
  ppm -- a healthy receiver, not one about to retrain.
- **Decision-directed adaptation walking the filter off.**  Worth fixing
  on its own account and now fixed (it stops when the constellation is
  not being hit, and the filter is snapshotted while it demonstrably
  works and restored when it stops), and it *did* extend the good stretch
  -- but restoring the filter does not recover the call.  On one call it
  fired 142 times, once a second, each time failing within the second.
  A filter that no longer fits is a statement about the signal, not the
  filter.

So something about the received upstream changes ten seconds in and does
not change back.  One suspect remains unexamined: the raw ring holds
131072 samples, which at 9.6 kHz is 13.6 s, uncomfortably close to the
collapse times -- though one call held for 27 s with the same ring, which
argues against it.  Doubling the ring is a one-line test and the rig
would not produce a single Phase 4 call in twenty-five attempts to run
it.


## The lock metric, and why it still cannot be validated

The frame-phase sweep locks on the ones fraction, which only means
something while the peer's line is mostly idle.  With its DTE sending
from the first second, no phase reads high and the sweep cannot tell a
good phase from a bad one -- a whole batch of calls never locked.

V.14 framing is the content-independent replacement: ten-bit characters
with a zero start and a one stop, so a correct decode has one bit phase
in ten where both hold.  Two things had to be measured rather than
assumed, and the second is not settled:

- **An absolute threshold cannot work.**  On a mostly-idle line the best
  phase reads 0-11% even when the decode is perfect at 100% ones, because
  idle marks contain no start bits at all.  The level is set by how busy
  the line is; the ratio between the best phase and the other nine is
  what says whether the framing is real.  The gate is now that ratio.
- **The ratio has not yet been seen on a good busy line.**  The readings
  taken at 1000 B/s -- best phase 19-25%, about twice the others -- came
  from a call whose symbol error was 0.66, i.e. from garbage, because it
  had collapsed six seconds in.  Two times is what noise gives.  The gate
  is set at three times, so it does not fire on that, but nothing yet
  shows what a correct busy line reads.

Getting that number needs a call that both locks and keeps its symbols
while the peer transmits, which is the same thing that is missing for
everything else here.


## The upstream carrier, and why ME_V34_DATA_CARRIER_TRACK=0 makes it worse

The symbols leaving the constellation is a CARRIER problem, and the
evidence is the difference the decision-directed carrier tracker makes:

    tracking off   symbols go from 0.09 to 0.66 within seconds and stay
                   there -- four minutes of a five-minute call, never
                   recovering, with restoring the equalizer 142 times
                   changing nothing
    tracking on    the same collapse happens, and then it comes BACK:
                   0.66 at t=4 s, 0.415 at t=34, 0.103 at t=42, and it
                   goes on acquiring and losing every few tens of seconds
                   for the rest of the call

Nothing else explains that pair.  A gain error cannot: a 33-point sweep
from 0.40x to 2.00x found no scale that puts the symbols back.  A stale
equalizer cannot: restoring a known-good one does not help.  A rotating
constellation can -- it is uniformly off the lattice, which is exactly
the 2/3 figure, and only a phase tracker can undo it.

So the receiver is pulling the carrier in and losing it again, on a cycle
of tens of seconds.  That is a loop that acquires but cannot hold, and it
is the last thing standing between this path and sustained upstream.

**`ME_V34_DATA_CARRIER_TRACK=0` should not be used on this path.**  It
was set during an earlier investigation to rule the tracker out as a
suspect, and it does rule it out -- but it also removes the only thing
that ever recovers the constellation.  The two calls that delivered real
payload (1698 lines, and 1329 of 1330 in sequence) both ran with it on.


## The frame-phase sweep cannot rescue a busy line (measured)

With the peer's DTE sending continuously and the symbols clean at
0.09-0.10, the sweep was allowed to run and every candidate was scored on
V.14 framing.  **777 candidates across a call, all reading 24-29%** --
chance is 25%.  Not one of the 112 (superframe, data-frame) combinations
shows async structure.

Put beside the other measurements, that is a sharp fork:

    peer idle      the decode is perfect -- 100% ones, sustained 95 s
    peer sending   the symbols stay clean, and the bits show no async
                   framing at any frame phase

So on a busy line the bits are wrong for a reason the frame phase does
not reach.  Two candidates remain, and they are distinguishable:

- The peer is not sending V.14 async on those calls, and our data stack
  is framing a synchronous stream.  Against this: calls have delivered
  1329 of 1330 pattern lines in sequence, so it does send V.14 at least
  sometimes.
- The decode is simply wrong for non-trivial data -- correct for the
  all-ones idle pattern, wrong otherwise -- which would point back at the
  shell/trellis path rather than at framing.

The way to tell them apart is a bit dump taken during a *good busy*
stretch (`ME_V90_UPSTREAM_BIT_DUMP`, now that carrier recovery gives
thirty seconds of clean symbols to dump) and an offline scan for HDLC
flags and for the pattern under sync framing, which
`tools/soak/upbits_align.py` already does.


## Payload bits are noise, not misaligned bits (measured)

A bit dump taken with the peer's DTE sending, analysed in 20000-bit
windows for the ones fraction and for autocorrelation at 90, 80 and 10
bits:

- the idle stretches read ones 0.90-1.00 with autocorrelation ~0.99 at
  *every* lag -- a nearly constant stream, which is what marks are
- every other window reads ones 0.50 and autocorrelation 0.50 at every
  lag -- noise, with no structure at any lag

The peer sends a nine-byte repeating pattern, so a correct-but-misframed
payload stream would stand out: ones near 0.45 and autocorrelation raised
at 90 bits while staying flat at 10.  **No window looks like that.**  Put
with the 777 frame-phase candidates all scoring at chance, alignment is
eliminated: the bits are not shifted, they are wrong.

So the fork closes on the second branch.  The decode is correct for the
all-ones idle pattern and produces noise for anything else, which is a
statement about the shell/trellis path rather than about framing, timing
or carrier.  It also explains why `v34_data_test` passes 390 cases and
sees nothing: our encoder and decoder share whatever convention this is,
so a loopback cancels it, and all-ones is a degenerate input that cannot
distinguish conventions either.

What that needs is validation against something external to this tree --
one mapping frame worked through 9.4/9.5's shell mapping and 9.6's
trellis by hand against the spec tables, or a known-good V.34 capture
decoded end to end.  Not another live call: the rig has already said
everything it can about this one.


## Correction, and the single blocker as it now stands

Two things in the previous section were overstated, and both matter:

- The "777 candidates all at chance" measurement was taken **while the
  sweep was running**, so it scored a moving target.  A correct phase
  would have held for one window and could not have read much above the
  20% the occupancy allows, which is inside the 24-29% band observed.  It
  is weak evidence, not proof that alignment is irrelevant.
- Running the peer's DTE from the first second (`SOAK_SOCK_ALWAYS`)
  removed the idle the lock metric needs, so those calls could never lock
  by construction.  That test was self-defeating.

Run properly -- normal schedule, five minutes, every fix in -- the
picture is simple and the blocker is one thing:

    downstream   111175 of 111274 lines, 99.9% clean, five minutes
    upstream     locks immediately at 100% ones, holds clean symbols for
                 about forty seconds, then the symbol error goes to 0.67
                 and stays; 337 equalizer restores recover nothing

**The collapse happens on idle traffic too**, forty seconds in, with the
peer's DTE silent at the time.  So it is not payload, not framing and not
alignment.  It is not timing (freq at noise, zero slips), not gain (a
33-point sweep finds no scale), and not the equalizer taps (restoring a
known-good set 337 times changes nothing).  Carrier recovery extends the
good stretch from six to thirteen seconds up to thirty or forty, but does
not prevent it.

Something in the received upstream changes about forty seconds into every
call and does not change back.  The strongest untested lead is the media
path rather than the modem: the pty spin fixed earlier in this session
was exactly that kind of fault, invisible in the modem's own logs and
lethal on the wire, and it was found by looking at RTP jitter rather than
at DSP.  Worth checking whether anything periodic happens on this call at
that scale -- jitter-buffer adaptation, RTCP, a tap flush -- before
reaching for another DSP hypothesis.


## Not a media discontinuity either

The received-audio tap from the five-minute call was measured in 200 ms
blocks across its whole length.  Through the entire data-mode stretch --
t=80 s to t=416 s, spanning the collapse -- the received level sits at
305 to 315 RMS with no step, no dropout and no change of any kind.  The
peer's signal keeps arriving exactly as before; our receiver simply stops
decoding it.

So the collapse is not a level change, not a dropout, and not a gross
media event.  Together with everything else ruled out -- timing, gain,
equalizer taps, carrier (which extends it but does not prevent it),
payload, framing and alignment -- there is no hypothesis left that can be
tested from a live call, because a live call has already answered every
question that can be put to it from the outside.

### What is actually missing: an offline replay

Every remaining question is of the form "what does the receiver do with
these samples, and what happens if I change X" -- and each one currently
costs a call that may or may not come, on a rig whose Phase 3 succeeds
sporadically.  The tap files already hold the exact audio of calls that
collapsed.  A harness that feeds a recorded live-rx.g711 into the T/3
upstream receiver, from the E handover onwards, would make this
reproducible on the desk: the collapse could be bisected, hypotheses
tried in seconds, and a fix validated before it ever sees the rig.

That is the enabling piece of work, and it is worth more than another
night of dialling.

## The forty-second collapse, off the wire (2026-08-21)

The previous section ended by saying the enabling piece of work was an
offline replay: a harness feeding a recorded `live-rx.g711` into the T/3
receiver from the E handover onwards, so the fault could be bisected on the
desk instead of over five-minute calls on a rig whose Phase 3 succeeds
sporadically.  That harness is `v90_upstream_replay.c`.  It replays a
500-second call in 108 seconds and is deterministic, which is what made
everything below measurable.

    ./v90_upstream_replay artifacts/dmodem-soak-0821-goalproper/tap/live-rx.g711 ulaw 3200 9600

The E handover is not recorded in the tap, so the harness searches for it:
it prepares a receiver at candidate instants half a second apart, feeds the
14 s of history that the receiver's own B1 search reaches back over, calls
`v34_begin_rx_data()` at the candidate, and asks whether it acquired.  On
this tap it finds the handover at 88.0 s with a **100.0% B1 fit** — nothing
but B1 correlates, so the first candidate that acquires is the real one.
Two things had to be right for that to work, and both are worth knowing
before writing another harness against this receiver: the history has to be
fed *between* `v34_v90_prepare_upstream_data()` and `v34_begin_rx_data()`,
because the first starts the T/3 capture and the second marks the E anchor;
and the context needs a Phase 4 bit handler, because without one
`v34_force_v90_phase4_cp_rx()` silently declines, leaving `v34_rx()`
dispatching into the INFO demodulator where the ring is never filled.

### What the collapse is

The receiver decodes the peer at a mean square distance to the lattice of
**0.002** for 42.1 s and then loses it **inside three symbols**, and the
old receiver never got it back.

It is not the wire.  Across that instant the tap is unchanged: level flat at
~1240 RMS, band 201–3439 Hz before and 205–3449 Hz after, centre 1820 vs
1827 Hz, no duplicated or all-same RTP payload, and no coherent fourth-power
line afterwards (peak/median 4, i.e. noise), so not a rotating constellation
either.

It is not the adaptive loops.  `ME_V90_UPSTREAM_DD_MU=0`,
`ME_V90_UPSTREAM_CARRIER=0`, `ME_V90_UPSTREAM_TIMING=0` and
`ME_V34_DATA_CARRIER_TRACK=0`, in every combination, end their first clean
run at **exactly 42.1 s**.  With all three off the receiver is a fixed FIR
at a fixed sampling step over a free-running mixer — a deterministic
function of the samples — so the trigger is in the samples.

It is a **whole-sample slip**.  Deleting one 8 kHz sample at the collapse
instant doubles the clean stretch, 42.1 s to 81.7 s.  The slips recur every
33–40 s, which is a clock offset of about three parts per million between
the peer and the RTP bearer, absorbed as a whole-sample insertion — the
tower d-modem's resampler is the obvious suspect (it defaults to ZOH; see
the rig notes).

Gardner cannot help, and this is the design defect the slip exposed: **every
adaptive element in this receiver is gated on the symbols already being
good** — the timing loop, the DD-LMS and the carrier loop all on
`sym_err_ema < 0.35`.  A slip closes the eye in one symbol, so at the exact
moment correction is needed they all freeze.  That is why the collapse was
permanent, and why restoring a known-good equalizer hundreds of times
recovered nothing: a good filter at a sampling instant a whole sample out is
not a state this receiver can decode in.

### The fix, and what it is worth

`v90_t3_slip_resync()` searches when the eye closes: it re-runs the current
taps over the recent past at each candidate offset, in thirds of a sample
out to ±3 samples, and adopts the position that puts the symbols back on the
lattice.  A slip is a step, so the recent past is already on the far side of
it and scores it correctly.  Two details matter.  The scorer must read the
ring through the timing loop's fractional accumulator, because that is where
the slicer actually looks — scoring whole samples measures a position the
receiver never uses, and the true offset then scores no better than its
neighbours.  And the acceptance has to take a clear win rather than a
perfect one: the equalizer spends the symbols before the trigger adapting to
nothing, so the best reachable position after a slip scores around 0.42, and
a 0.25 absolute gate rejected exactly those.  Adoption opens a bounded
window (`V34_V90_T3_SLIP_RECOVER`) in which the DD-LMS may adapt at an error
that would otherwise gate it off, so the filter can pull back in.

Measured on the same recorded call: clean time **6.9% → 32.8%**, and the
first three slips now recover in **under a second** each (44.2–46.2 s,
82.5–83.3 s, and clean through to 115.0 s) where the first one used to end
the call's usefulness.

### What is still open

From about 115 s the outages get longer and the search stops helping, and
the reason is visible in its own log: the score profile goes **flat at ~0.65
across all nineteen offsets**.  No sampling phase reopens the eye, and
restoring the last-good equalizer first (which the restore path now does,
before searching) does not change it either — 293 restores, no improvement.
So the later outages are not slips, or not only slips, and the next question
is what the received signal is doing then.  The harness answers questions
like that in under two minutes each, which is the point of it.

Note the score profile is periodic in three samples, as it must be: three
samples is one symbol at 3200 baud, and a whole-symbol shift still lands on
the lattice.  A minimum at −1.67 and at +1.33 is one position, not two.

### A trap in long soak batches (2026-08-21)

A batch that has been dialling for a while stops reaching data mode, and it
looks precisely like the peer refusing to train: the far end reports NO
CARRIER on dial after dial.  It is not the peer.  `sip_v90_modem` runs out
of media transports --

    pjsua_media.c  Unable to create media transport: Too many objects of the
                   specified type (PJ_ETOOMANY) [status=70010]

-- after which every incoming INVITE fails to get a media channel.  Measured
here after roughly nine calls in one server process.  A batch went 0/9 and
was read as the peer's Phase 3/4 retrain lottery running cold; six of those
attempts were genuine retrains, but the later ones never had media at all.

So: **restart the server process between batches**, and treat any attempt
after the first PJ_ETOOMANY as carrying no information about the modem.
`tools/soak/soak_verdict.sh` now says so at the top of its report.

### Where the upstream stands after the live runs (2026-08-21)

Downstream is done: three consecutive five-minute calls delivered 99.4%,
100% (zero lines missing of 76288) and 99.6% of their pattern lines.  The
large downstream losses seen earlier the same day were a soak harness fault
of mine, not the modem -- the pumps were driving 5000 B/s into a link that
carries 5200 char/s, and 2000 B/s into a 9600 bps upstream that carries 960.
Match the rates to the link before reading anything into an integrity figure.

The upstream now gets further than it ever has and still does not carry
payload, and the reason is specific.  On a live call replayed offline:

  * acquisition is right -- B1 fits at 100%, and the out-of-sample check
    measures 0.002 against the template's own power;
  * the symbols are clean for 36% of the call;
  * but through those clean blocks the descrambled stream reads **49% ones,
    peaking at 53**, so the frame phase is wrong nearly all the time, and the
    two moments it does lock (100% and 90% ones) are brief.

The link between those facts is the slip.  This rig's stream really does gain
or lose a whole 8 kHz sample about every four seconds -- 81 of them replaying
the recording, against 70 counted live, so it is in the audio and not an
artefact of the receiver.  Correcting one keeps the symbol timing but can add
or drop a symbol across the event, which moves the shell decoder's position
within the data frame; and a full sweep of the 112 (superframe, data-frame)
candidates takes about forty-five seconds.  A phase fault arriving every four
seconds cannot be repaired by a search that takes forty-five.

So the next piece of work is not another dialling batch.  Either the slips
stop -- they are the peer's resampler tracking a few parts per million, and
the tower d-modem's own `DM_RESAMPLER` is the place to look -- or the frame
phase has to survive a symbol being inserted or dropped, which means carrying
the phase through a slip correction rather than rediscovering it afterwards.
The recording that shows all of this is
`artifacts/dmodem-soak-0821-rounds/round1/tap/live-rx.g711`, and
`v90_upstream_replay` reproduces it in under two minutes.

## The slip carries a carrier step (2026-08-21, later)

The previous section left the upstream acquiring correctly, clean for 36% of
a call, and losing the frame phase to slips arriving faster than a 45 s sweep
could chase.  It also treated the slips as an inference from the receiver's
own behaviour -- one sample deleted at the collapse doubled the clean stretch
-- rather than as something measured.  Both of those are now settled, and the
second one changed the answer.

### Measuring the wire instead of the receiver

`tools/measure_timing_slips.py` reads the transmitter's symbol epoch straight
off a recorded tap.  A V.34 signal with excess bandwidth carries a spectral
line at the symbol rate in the squared envelope of its analytic signal, and
the phase of that line IS the epoch, measured against the 8 kHz bearer.  One
inserted or dropped sample moves it by exactly 360 x baud/8000 degrees -- 144
at 3200 baud -- and nothing else in a call looks like that.

    tools/measure_timing_slips.py artifacts/dmodem-soak-0821-rounds/round1/tap/live-rx.g711 --from 61

On round1 the phase sits at -119 degrees, steps to -263 at t=129.6 s and back
at t=130.6: 28 steps in 290 s, net +7.2 ppm, arriving in bursts of three a
second apart early on and settling to a metronomic +1 every 12 s later.

The RTP is unbroken across every one of them -- 21365 packets, no loss, no
sequence or timestamp discontinuity, media clock within 4 ppm of wall clock.
So the slips are inside the peer's own audio generation, not the transport,
and `DM_RESAMPLER` on the tower remains the place to look if they are ever to
stop.  The receiver had been reporting 81 corrections against those 28 real
events, which is the first thing this measurement bought: most of what it was
correcting was itself.

### Four defects, each hiding the next

**The search span was wider than the thing it searched.**  The score a
candidate offset is judged by is the distance from the equalized symbols to
the lattice, and that is periodic in one symbol -- three samples at T/3 --
because a whole-symbol shift still lands every symbol on the constellation.
Searching +/-3 samples therefore offered every minimum twice, once at its own
position and once a symbol away, and the argmin picked between them on noise.
46 of 81 corrections on round1 were the far copy, 36 of them "+2.67", which is
-0.33 plus a symbol.  Each one moved the shell decoder within the data frame,
and the frame-phase sweep cannot undo that: it searches frame labels, not
single symbols.  The search now covers the half symbol either side and no
more, in sixths of a sample rather than thirds -- the quantum the peer slips
by is one 8 kHz sample, which is 1.2 T/3 samples at 3200 baud, and a
third-of-a-sample grid has no point within 0.13 of it.

**A slip is a passband delay, so it comes with a rotation.**  This is the one
that mattered, and it is pure bookkeeping about where the mixer sits.  The
T/3 ring holds complex baseband, mixed down by an angle indexed on the
ABSOLUTE sample count.  With the received analytic signal a(n) = b(n)e^(jwn),
a delay of D gives ring'(n) = b(n-D)e^(-jwD): reading D samples further on
recovers the right baseband sample **rotated by -wD**.  At 3200 baud low
carrier that is 82 degrees per 8 kHz sample.  The search corrected the delay
and left the rotation, which is exactly why it kept reporting a flat profile
with every candidate at 0.65 and nothing to choose between them -- the
correct sampling instant was in the list and still did not fit the lattice.
It also explains why recovery ever happened at all: 82 degrees is close
enough to 90, which V.34's differential mapping makes harmless, that the
fourth-power carrier estimator sometimes pulled a call back on its own.  That
is what the "lottery" was.  The scorer now derotates by the carrier loop's
standing phase -- which it never did, so it had been scoring a signal the
slicer does not see -- and by the phase the candidate offset implies, and an
adopted offset carries its rotation into the carrier loop.

**Everything adaptive was gated on a 256-symbol average.**  A slip shuts the
eye in one symbol, so the decision-directed LMS went on adapting hard onto
wrong decisions for the hundred symbols the average took to turn round --
walking off the very filter the slip search needs intact to find the new
instant.  A fast estimate (1/16 per symbol) now gates the LMS, the timing
loop and the carrier loop; the slow one still describes the call and drives
the frame-phase logic.  The search also runs after 48 symbols off the
constellation rather than 240.

**The frame-phase sweep could not return to its own best candidate.**  It
recorded absolute (super_frame, data_frame) pairs, and those are free-running
counters: writing a pair back at a later boundary shifts the labelling by
that pair minus whatever the counters naturally held THEN, which is not the
same alignment.  So a sweep that visited the right phase could never go back
to it.  The phase is now a relative shift in data frames, with an accumulated
position that names the candidate.  Two things had also been starving the
sweep outright: an equalizer restore reset its counter, and restores fire as
often as every 375 ms, so on round1 it never once passed step 1 of 112 in five
minutes; and the measurement window straddled the superframe boundary where a
candidate takes effect, so every candidate was scored partly on its
predecessor.

### What it is worth

Replaying round1 offline, symbols on the lattice go from 35.8% of the call to
**60.1%**, and the permanent-collapse mode is gone.  The old receiver decoded
perfectly for 68 s, hit the first burst of three slips, and was still white
145 s later with the wire provably quiet throughout; it now recovers within
20 s and runs 73 s clean.  Payload reaches the PTY in bursts for the first
time on this recording: searching the raw bit dump for the soak pattern finds
14 `U0` hits after the peer's DTE starts, five of the gaps between them exact
multiples of the 90-bit line -- real lines, not coincidence.

### What is still open, stated precisely

Frame-phase lock still does not hold.  With clean symbols the sweep locks at
100% ones and loses it about half a second later, and this is now the whole
of the remaining fault: 60% of the call has an open eye and only a few
percent of it is framed correctly.

Two leads, in order.  **The release rule cannot be right on a line carrying
data.**  It releases a lock at under 70% ones, and the soak pattern `U%07d\n`
through V.14 framing is mostly digits -- `'0'` is 0x30, which LSB-first
between a zero start bit and a one stop bit is three ones in ten.  A
correctly decoded pattern therefore reads 30-40% ones and is indistinguishable
from noise by that measure, so the receiver throws away good locks as soon as
the peer says anything.  The V.14 start/stop ratio is there for exactly this
and reads about 2x on those windows against a 3x lock gate -- close, and worth
measuring properly rather than tuning blind.

**But do not read too much into a window that reads well.**  Measured on the
one stretch of this recording that provably carries payload -- bits 2549148
to 2557684 of the dump, where five of the gaps between `U0` hits are exact
multiples of the 90-bit line -- the V.14 metric reads 32% at its best phase,
a ratio of 1.4x, and 51.9% ones.  That is about fourteen correct lines in the
ninety-five the window could hold.  So even the best region of the call is
only a sixth right, and the metric is reporting that honestly rather than
failing to see a good decode.  The fault is still in holding the alignment,
not in judging it.

A note on the arithmetic, since it is easy to get wrong and it decides
whether the sweep is fast enough.  `data_frame` counts MAPPING frames within
a data frame (p = 16) and `super_frame` counts data frames within the
superframe (j = 7), so the 112-candidate space is one superframe of 112
mapping frames -- 896 symbols, 280 ms at 3200 baud, which is V.34's superframe
to the millisecond.  A phase shift lands when `++data_frame >= p`, every 16
mapping frames, so every 40 ms.  The sweep step rate is therefore set by the
measurement window and not by the boundary, and moving the application to the
mapping-frame boundary would buy nothing.


## The lock metric could not see a correct decode (2026-08-21, later still)

The section above left the lock not holding, and named the release rule as
the first lead: the soak pattern is mostly digits, `'0'` is 0x30, and
LSB-first between a zero start bit and a one stop bit that is three ones in
ten, so a *perfect* decode of the pattern reads 30-40% ones against a rule
that releases below 70%.  That was the fault, and it was the whole of it.

**5217 pattern-line hits off the same recording, where the run before gave
14** -- and 5129 of the gaps between them are exact multiples of the 90-bit
line, so they are runs of the peer's real traffic rather than coincidences.
The receiver had been finding the phase, decoding correctly, and discarding
the lock the moment the peer's DTE said anything.

The evidence needed to judge a lock on a busy line was already being computed
and thrown away.  §9.6.3.3 builds the shell index r0 out of the ring indices
of the eight 2D symbols of a mapping frame, and the transmitter's own
construction bounds it to k bits.  An r0 that does not fit cannot have come
from a correctly grouped frame -- so it is a frame-phase check that owes
nothing to the content.  `bitstream_put()` truncates silently, so nothing had
ever looked.

Measured over round1, the separation is clean:

    correct-phase windows   0% in 194 of 202, never above 3%
    wrong-phase windows     3% or more in 92 of every 100

A standing lock is now judged on that where it is available, falling back on
the marks only where k is zero, and the sweep ranks candidates by it first,
weighted so that one percent of bad frames outweighs anything the marks can
show.

**Read line counts, not the ones fraction, from here on.**  The ones fraction
was the right instrument while the peer was idle and is actively misleading
once it is not: a correct decode of digits reads about the same as noise.
`ME_V90_UPSTREAM_BIT_DUMP` plus `tools/soak/upbits_align.py` -- or just
searching the dump for the pattern and checking the gaps are multiples of 90
-- is what says whether the upstream is working.

Still open: 60% of the call has an open eye, and the recovered lines are a
fraction of what the peer sent, so both the outage time after a slip and the
sweep's dwell are still worth attacking.  Neither is now a mystery.


## Live against the rig (2026-08-21): it works, and live acquisition does not

The offline work above was replay only, so it was run against the live
d-modem/slmodemd rig.  Four calls reached data mode.  Evidence in
`artifacts/dmodem-soak-0821-shellfix/`.

**The fix does what it was meant to.**  One call
(`batcha/round1/try3`, 185 s) delivered **504 in-sequence upstream pattern
lines** with the peer's DTE transmitting from the first second.  That
configuration is the one the earlier notes record as hopeless -- "a whole
batch of calls with the peer transmitting from the first second never locked
at all" -- because the ones fraction cannot tell a correct decode of real
traffic from noise.  It locks now.

**Downstream is healthy**: 99.6% of pattern lines on one call (66489 of
66787, 298 missing), 98.6% on another.  One call read 50%; that is
call-to-call variance, not the pump rate -- the same `SOAK_PTY_RATE=5000`
gave 99.6% on the next call.

**But every live acquisition is poor, and it is 4 for 4.**  This is the thing
to fix next, and the measurement is unambiguous because
`batchb/round2` holds exactly one call, so there is no ambiguity about which
attempt the tap belongs to:

                          acquires at   fit     out-of-sample  call median  windows <0.05
    live                  sample   3047  98.3%         0.125        0.114          0.0%
    replay, same audio    sample 134663 100.0%         0.002        0.002         74.9%

Same samples, same code, fifty times better offline.  The other three live
calls acquired at 0.118, 0.119 and 0.125 -- never once the 0.002 the replay
reaches -- and each then ran its whole length at about 0.11, which is far
above the 0.05 the shell decode needs.  That is why the upstream delivers in
bursts on one call and nothing at all on the next: it is not the frame-phase
logic, which is now sound, but the equalizer the call is stuck with.

Live it fitted at ring sample 3047 -- 0.32 s of capture -- so B1 cannot have
been in the ring and the 98.3% in-sample fit was to something else.
`V34_V90_T3_VALIDATE_ERR` is 0.40, chosen to separate a real acquisition
(0.002) from a hopeless one (0.66), and it lets 0.125 through; the retry
machinery that exists for exactly this never fires.

**A tightened gate was tried and withdrawn.**  Requiring 0.05 while retries
remain, falling back to 0.40 on the last attempt, breaks the offline T/3
regression in `vpcm_loopback_test`: that case runs at 21600 bps, where the
constellation is far bigger and the honest out-of-sample distance is larger,
and its audio ends long before forty retries at 0.5 s each could run.  The
same rate-dependence already bit the power half of this check once -- see the
comment there about a constant ratio rejecting a perfect acquisition -- so an
absolute distance is the wrong shape for the gate.  It needs to be relative
to what that rate can achieve, or the retry needs to be driven by something
other than a fixed threshold.  Not guessed at here.

**A note on tuning `ME_V90_UPSTREAM_DD_MU`.**  A larger step clearly improves
the eye -- on the round1 tap the median symbol error goes from 0.073 at the
0.02 default to about 0.011 at anything from 0.05 up.  It is tempting to tune
it on recovered pattern lines, and that is a trap: across mu = 0.02, 0.05,
0.1, 0.2, 0.4 the line count reads 5217, 2, 12300, 4955, 2.  The swing is
whether the frame-phase sweep happened to lock on that run, not the step
size.  Tune on symbol error; read line counts only as an outcome.


## The acquisition gate, done properly (2026-08-21, later)

The section above ended with a tightened gate tried and withdrawn because an
absolute distance broke the 21600 offline regression.  The gate now judges an
acquisition by its SNR instead, and the live run that followed both validated
it and caught a regression in the first version of the retry around it.

**Judge it as an inverse SNR, not as a distance.**  Instrumenting the offline
T/3 regression settles the shape of the gate rather than arguing about it: at
21600 it acquires with an in-sample fit of 99.9-100% and an honest
out-of-sample distance of **0.053 to 0.074**, so the 0.05 threshold tried
first rejects a perfect acquisition.  Divided by the symbol power carrying it
the populations separate by a factor of thirty:

    21600 loopback, fit 99.9-100%   0.053-0.074 / 93-120 = 0.00058-0.00061
    9600 live audio replayed, good  0.002       / 7.1    = 0.00028
    9600 live, the poor one         0.125       / 7.2    = 0.0174

`V34_V90_T3_ACQ_GOOD_SNR` is 0.002, the geometric middle of the tightest gap
(at 21600 a closed eye reads 0.0056 against a good fit's 0.00061).

**The retry has to have a reachable fallback, and the first version did not.**
With 24 retries allowed, a live fifteen-second call rejected the same window
seventeen times, never reached the fallback, and delivered **no upstream at
all** -- strictly worse than the poor acquisition it was refusing.  Bounded to
six, the worst case is the old behaviour about three seconds later, and the
best window seen is adopted rather than the first.  Live confirmation: a later
call logged six rejects, then `settling for the best of 7 windows`, and
acquired.  The mechanism does what it says.

Retries are worth less on this peer than the count suggests, and it is worth
knowing why before raising it: B1 sits about 0.35 s past the E anchor, inside
the very first search window, so a later window contains no B1 at all.  A
retry buys another look at the SAME B1 with more wire either side of it.

### The live acquisition gap, with three explanations ruled out

Live never finds a good acquisition.  Across every call measured it lands at
0.088 to 0.229, and each call then runs its whole length at a symbol error
between 0.11 and 0.66 -- while the identical recorded audio replayed offline
acquires at 0.002 and holds a median of 0.002.  The gate makes this visible
and stops it making things worse; it does not fix it.

Ruled out, each by measurement:

  * **Ring length.**  `v90_upstream_replay` now takes the seconds of history
    to feed before a forced handover.  At 14, 2, 0.5 and 0.35 seconds it fits
    the same B1 at 100.0% for an out-of-sample 0.002 every time.  Live has
    about 0.3 s and that is not what is wrong.
  * **Different samples.**  The G.711 tap is written from the same buffer the
    modem then consumes -- `modem_engine.c`, immediately before the loop that
    feeds the V.34 receiver -- so live and replay see byte-identical audio.
  * **Different parameters.**  Both paths report the same negotiated frame
    parameters (`b=24 p=16 w=0 j=7 k=12 b1_symbols=128`), so the B1 template
    is identical.

  * **Anchor position.**  Sweeping the forced handover across a recorded call
    shows the fit is strictly BIMODAL: anchors from 52.8 s to 54.0 s all give
    `coarse=98.2% fit=100.0%` and 0.002, and anchors outside that 1.2 s
    plateau give `coarse=6-8% fit=17-19%`.  There is no intermediate regime,
    so live's `coarse=96.6% fit=98.3%` is not an anchor effect -- a bad anchor
    scores 6%, not 96.6%.
  * **Capture sub-sample phase.**  Live and the replay start their T/3 capture
    at different points in the 8 kHz stream, so the 8k->9.6k resampler sits on
    a different phase, and unlike a gain or a rotation that is not something
    the equalizer's complex taps can absorb.  Shifting the capture start
    through all six phases one 8 kHz sample at a time gives 100.0% and 0.002
    every time.
  * **Template state.**  `v34_build_expected_b1_tap_trellis()` builds a fresh
    transmitter per attempt, so the scrambler and trellis start clean, and its
    inputs are identical on both paths -- rate 9600, trellis 0, the same role,
    and no precoder in V.90 mode.

So live matches the right B1 at the right place with the right template and
still fits it at 98.3% where offline fits 100.0%.  Every difference cheap
enough to test from outside has been tested.  What separates the two is the
receiver STATE that a real call accumulates and a replay does not: live has
run the whole of V.8, Phase 2, Phase 3 and Phase 4 through the same
`v34_rx()`, while the replay's receiver is forced straight into Phase 4 CP RX
having seen a second of audio.  The next step is a direct comparison rather
than another hypothesis from outside -- and the reason it has not been done is
that the tool for it does not exist yet.  `v90_upstream_replay` drives the T/3
receiver alone, with `v34_force_v90_phase4_cp_rx()` putting it straight into
Phase 4 after a second of audio; it cannot accumulate the state a real call
does, which is precisely the difference under investigation.  And
`sip_v90_modem` has no file input.

So the enabling piece of work is a **full-engine offline replay**: feed a
recorded `live-rx.g711` into `me_rx_g711()` from the start of the call and let
the engine run V.8, Phase 2, Phase 3 and Phase 4 off it, exactly as the media
thread does.  The transmit side can be discarded, the same way the T/3 replay
discards it -- the peer's recorded audio already contains its responses, and
the engine's trajectory is deterministic given the same received samples.  If
that harness reproduces the live 98.3%, the fault is bisectable on the desk in
minutes; if it reproduces 100.0% instead, the difference is in the media path
rather than the modem, which is equally worth knowing.

This is the same conclusion, and the same shape of tool, that unblocked the
forty-second collapse earlier in this document.  It was worth more than
another night of dialling then, and the four live calls behind this section
say the same now.

## The Phase 2 CC notch was never retired (2026-08-23)

V.90's upstream is V.34-modulated, so it goes through the same receiver as a
plain V.34 call -- and through the same filter that cost that call 17 dB of
receive SNR (`docs/v34_data_mode_rates.md`).

`v8_result_handler()` puts a 30 Hz notch on **1200 Hz**, our own CC transmit
frequency.  Against the Phase 2 CC tones that is exactly right: both signals
are narrowband, they are 1200 Hz apart, and the notch removes our echo at no
cost to the 2400 Hz we are listening to.  From Phase 3 on it is the opposite.
The upstream is then a wideband V.34 signal -- 3200 baud on the low carrier
spans about 36 to 3620 Hz -- and 1200 Hz is deep inside it.  A 30 Hz notch is
an impulse response of some 266 samples, about 170 symbols at 3200 baud,
against the 63 symbols the equalizer spans, so the receiver cannot undo it.

**Nothing retired it.**  `start_v34_training()` disables the notch at 3200 baud
(91 Hz of carrier separation is too narrow to filter), `v8_result_handler()`
then re-enables it at 1200 Hz for Phase 2, and the only later clear is on the
fallback to plain V.34.  Every V.90 soak log in `artifacts/` shows exactly two
notch lines and no third.  The plain-V.34 fix does not reach this path either:
`v34_update_echo_policy()` returns immediately when `g_mod != ME_MOD_V34`.

`v90_retire_phase2_cc_notch()` now clears it when Phase 3 starts.
`ME_V34_ECHO=notch` keeps the old behaviour for an A/B.  The wideband NLMS
canceller is untouched -- it is gated on `g_mod == ME_MOD_V90` and is the right
tool against a broadband PCM echo, which a single notch never was.

### What it is worth, and what it is not

Two A/B pairs, `tools/soak/v90_notch_ab.sh`, upstream pumped for the whole call
(`SOAK_SOCK_ALWAYS=1`), measured as the bytes the upstream actually delivered to
our PTY:

| run | notch | upstream bytes | DATA-bit reports |
|---|---|---|---|
| A  | left in | 37394 (call dropped at 13 s) | 311 |
| A2 | left in | **0** | 0 |
| B  | retired | 341967 | 2906 |
| B2 | retired | 340407 | 2906 |

So the upstream path now runs for the whole call instead of stopping or never
starting.  **It does not fix the upstream decode.**  The symbols stay white in
every one of the four runs -- best `sym err` 0.648 and none at all with the
notch, 0.580 and 0.564 without, against 0.667 for symbols with no relation to
the lattice -- and not one intact `U%07d` pattern line arrives in any run.
Whatever is stopping this upstream from decoding, the notch was not it; that
work is the rest of this document.  The change is kept because the mechanism is
proven in the plain-V.34 path and a notch inside the received band that nothing
retires is a defect by inspection, not because it was measured to fix anything
here.

## The upstream's dominant impairment was our own echo canceller (2026-08-23)

The entry above was written against a receiver whose upstream was white, and it
closed by saying the notch was not what stopped it.  It was not, and the thing
that did is next to it: the NLMS echo canceller, unconditional on every V.90
call, was the largest noise source in the receive path.

Two defects, and the first one hid the second for months.

**(a) The Phase 2 notch retirement only ever fired on the first call of a
server process.**  `v90_retire_phase2_cc_notch()` guarded itself with a
`static bool retired`, which is process lifetime, not call lifetime.
`v8_result_handler()` re-arms the 1200 Hz notch for every call, so call 1
retired it and calls 2..N re-armed it and could never retire it again -- and on
this rig the call that reaches data mode is rarely the first.  Read the notch
lines in `artifacts/v90-cap-20260822T130820Z-9600/server.log` in order: the
retirement appears once, against the first call, and the call that actually
reaches data mode (notch armed at line 7931, Phase 3 complete 8519, data mode
9698) has no retirement line at all.  That call ran its whole upstream at 0.139
from the lattice -- the same figure the plain-V.34 notch left behind, 17.2 dB.
`g_notch.active` is already the correct once-per-call guard, since the function
clears it and V.8 re-arms it, so the latch is redundant within a call and wrong
across calls.  It is gone.

**(b) The echo canceller was not cancelling an echo.  It was adding one.**
The canceller prints its own verdict every 8000 samples and nobody had read it:
across the soak runs in `artifacts/`, its effect on the receive RMS oscillates
around zero -- +0.2 to -0.4 dB, mean nil -- which is what a filter with nothing
to remove looks like.  The proof is in the batches where the received signal is
digital silence: `pre_rms=0 post_rms=106`, `pre_rms=0 post_rms=159`.  A filter
that turns silence into RMS 159 is not removing anything; it is injecting its
own transmit-driven misadjustment, and in V.90 the transmit reference is the
downstream PCM, measured 8.4 dB louder than the upstream we are trying to
receive.  Against a received upstream of RMS ~1240 that injection alone pins the
receive SNR at 18-22 dB.  A 31200 bit/s upstream at 3200 baud is 9.75
bits/symbol and wants far more, so the canceller was capping the exact quantity
it sat in front of.  There is no echo for it to find, either: peak normalised
cross-correlation between our transmit and our receive is 0.0068 over lags 0 to
1024, and a 512-tap least-squares fit of transmit into receive removes 0.307% of
the receive power -- 512/160000, exactly the bias of fitting that many taps to
that many samples.

It is the same shape of defect as the notch, and it hid for the same two
reasons: it is applied **after** the RX G.711 tap, so every recording of a live
call shows a clean signal and every offline analysis exonerates the wire; and no
loopback exercises it, because `v34_duplex_test` drives `v34_rx()` directly and
never runs the engine.

On V.90 the canceller is now opt-in (`ME_V34_ECHO=canceller`).  Measured back to
back on the same binary, four rounds each:

| | B1 out-of-sample distance | data-mode `sym err` min | intact `U%07d` lines |
|---|---|---|---|
| canceller on (`ME_V34_ECHO=canceller`) | 0.657, 0.675 | 0.548, 0.617 | 0, 0 |
| canceller off (default) | 0.226, 0.226, 0.226 | 0.165, 0.186, 0.165 | **399, 0, 730** |

0.667 is the distance for symbols bearing no relation to the lattice, so with
the canceller in, B1 itself was white out of sample and the eye never opened at
all.  The 399 and 730 lines are the peer's own traffic arriving intact on our
PTY -- the first upstream payload this path has delivered.  B1's in-sample fit
goes 98.6% -> 100.0% with it out, which is the same statement made where the
sequence is known.

`artifacts/v90-ecoff-d-233840Z` is kept as the record of the 730-line call.

**What this did not fix.**  Payload is intermittent -- two of three data-mode
calls delivered it, and the median `sym err` is still 0.666, so the eye is open
in bursts rather than continuously.  Frame-phase lock still does not hold.

**Two things measured and ruled out along the way, so they are not re-run.**
The ordinary V.34 data path seeds its carrier loop's *frequency* from B1's two
halves, because a decision-directed loop cannot acquire a frequency once its
first decisions are wrong -- that is what made plain V.34 white above 12000
bit/s.  The T/3 upstream loop never had that seed, and this upstream is well
past that density, so it looked like the same defect.  It is not: the seed is
implemented (`ME_V90_UPSTREAM_B1_FREQ=0` disables) and the residual it measures
on a live call is **0.0229 deg/symbol, 0.20 Hz**, against the 0.1054 deg/symbol
that was fatal in V.34, and with the canceller out it reads -0.0001.  Residual
carrier is not this path's problem.  Precoding is not either: V.34 precoder
coefficients travel from the receiver to the transmitter, and this side only
ever builds MP type 0 (`v90_build_mp_type0()`), so a conformant peer transmits
unprecoded.

## The frame phase cannot hold because the symbols are marginal (2026-08-23)

"Frame-phase lock still does not hold" was the standing open item above.  It is
not a lock defect.  Measured on `artifacts/goal-v90-073744Z` -- a live call that
reached data mode at 52000 down / 31200 up and delivered 730 intact `U%07d`
lines -- the receiver decodes cleanly for the first few seconds and then the
symbols leave the constellation permanently, and for the remaining 95% of the
call there is nothing for any phase metric to measure.  `v90_upstream_replay`
reproduces the whole of it off the recorded tap.

**The wire carries 36 dB and we are already getting 35.4 of it.**
`tools/v34_channel_bound.py`, pointed at the upstream tap with the equalized
symbols the receiver decided in the clean stretch, fits the best possible
fractionally-spaced least-squares equalizer and reports **37.4 dB in sample,
36.3 dB held out** at 81 T/2 taps.  The receiver's own figure over the same
window is **35.4 dB** (mean symbol power 726, mean squared distance to the
odd-integer lattice 0.208).  So it is within a dB of everything a linear
receiver can do, and there is nothing left to win at our end.

**31200 bit/s at 3200 baud is 9.75 bits/symbol, and that needs about what the
wire has and no more.**  The lattice spacing is 2 whatever the rate, so at
mean power 726 an uncoded symbol error rate of 1e-6 wants the noise at 4.75
sigma inside the half-spacing -- 39 dB -- and V.34's trellis coding returns
about 4 dB of that.  35 dB required against 36.3 available is nought to one dB
of margin, which is exactly the behaviour observed: it decodes while B1's
converged filter is fresh, any small disturbance tips it over, and because
every adaptive element in this receiver is gated on the symbols already being
good, it can never climb back.

**Four candidate causes for the collapse are measured and ruled out.**  Do not
re-run them.

- *The wire.*  Level flat at 147-150 RMS and out-of-band/in-band power flat at
  0.11-0.14 straight through the collapse instant.  The peer keeps sending the
  same signal.
- *A timing slip.*  `tools/measure_timing_slips.py` over the collapse: the
  first slip on this tap is at t=100.25 s, **six seconds after** the symbols
  are already gone, and there are none before it.
- *The sampling instant.*  The slip search fires 7238 times and the current
  position wins every profile it prints -- 0.41-0.46 against 0.60-0.71 at every
  offset in the half symbol either side.
- *Rotation, gain and residual carrier frequency.*  Swept offline over the
  dumped symbols (`ME_V90_UPSTREAM_SYM_DUMP`): the best of every gain 0.6-1.6,
  every angle, and every frequency +/-0.02 rad/symbol on a post-collapse window
  is **0.60**, against 0.667 for symbols unrelated to the lattice.  Nothing
  downstream of the front end can recover it.

**One real defect was found and fixed on the way.**  The equalizer
snapshot/restore pair was keyed on a fixed distance, `V34_V90_T3_FSE_KEEP_ERR`
= 0.20, while this receiver's settled operating point on this rig is 0.165-0.22
-- the threshold sat *inside* the healthy range.  So the recovery path fired on
a working receiver, installed taps from 3200 symbols ago, which raised the
error, which triggered the next restore: 287 restores, the first at 4.2 s with
the error at a healthy 0.213, and the error never below 0.3 again in the
remaining 107 seconds.  There was no middle zone either -- anything not good
enough to snapshot counted towards a restore.  The thresholds now come from the
receiver's own settled error (averaged over the first
`V34_V90_T3_ERR_BASE_SYMBOLS` after B1 hands over), with a do-nothing zone
between them.  On that recording the receiver holds to 7.3 s instead of 4.3 s
and the first restore is a genuine one on a real ramp.  It does not change the
outcome, because the outcome is set by the rate.

**Two instruments were lying and are fixed.**  The shell log printed
"shell bad 0%" both for a window in which every mapping frame was well formed
and for a window with no mapping frames at all -- opposite evidence, identical
text, and 1964 of 2829 windows on this call were the second kind while their
symbols were white.  It now reports the frame count.  And `v90_upstream_replay`
now takes `V90_REPLAY_VERBOSE`, without which the slip search, the gain sweep
and the offset profiles -- everything the receiver tried -- are invisible on a
replay.

**The rate was the lever, but it has to be pulled at Phase 2.**  36 dB supports
about 8 bits/symbol, so 24000 bit/s at 3200 baud should have margin where 31200
has none.  `ME_V90_UPSTREAM_MAX_BPS` capped the MP upstream-rate mask and the
receiver's B1 preparation, and that **does not work**: at a 24000 cap the
SmartLink rig replied to none of it -- **22 Phase-4 entries across ten
attempts, 14806 MP frames sent, zero MP' received**, while an uncapped control
on the same binary reached data mode on attempt 1 with 73 MP'.

The MP frame was not the problem in any way this end can see: bits 24:27 carry
`v90_upstream_mask_max_drn()` of the capped mask and bits 36:48 the mask
itself, the mask is an intersection with the peer's own offered set, and the
"offers no rate <= cap; echoing it uncapped" warning never fired.  What was
wrong is that **the rate is already fixed by the Phase 2 V.34 negotiation** --
the call trains "3200 baud, up to 31200 bps" -- and MP may select within what
that training established but not below it.  Capping the mask left one field
disagreeing with the INFO1 capability, with `v34_get_current_bit_rate()` and
with the pump itself, and the peer simply ignored it.

The cap therefore binds in `start_v34_training()`, on the `bps` handed to
`v34_init()`, where every later consumer inherits it by construction.  The V.90
call site already sets `g_mod` before calling in, so the flag is read before
the function overwrites it, in the same save/restore shape already used for
`g_v34_start_baud`.

**Measured on the rig at `ME_V90_UPSTREAM_MAX_BPS=24000`, two consecutive
calls, against 31200 on the same binary:**

| | 31200 | 24000 run 1 | 24000 run 2 |
|---|---|---|---|
| MP' frames | 0 in 22 Phase-4 entries | 73 | 73 |
| data mode | never | attempt 1 | attempt 1 |
| decision error per symbol | 0.17-0.21 | 0.044-0.063 | |
| clean symbol windows | 1.5% | 8.7% | 19.3% |
| longest unbroken eye | 4.3 s, 7.3 s | 29.6 s | 47.6 s |
| intact `U%07d` lines | 730, 399, 0 | 7945 | 12657 |

So the upstream now runs for half a minute or more at a stretch instead of a
few seconds, and delivers an order of magnitude more of the peer's own traffic.
The receiver's own readout agrees: it reports "this line will carry 24000 bit/s
against the 24000 asked for", where at 31200 it read 24000 against 31200.

**Still open.**  The eye still shuts eventually -- 19.3% of the best call is
clean, not all of it -- so this is a large improvement and not a cure, and the
frame-phase machinery above still has to survive the gaps.  And the rate is a
fixed environment variable, not something chosen from the measured SNR: closing
that loop is the same open item as `docs/v34_data_mode_rates.md`, and in the
end it needs V.34 12.2 renegotiation, because the only honest measurement of
this direction appears after the rate has been committed.

## What kills the 24000 upstream: the peer's first timing slip (2026-08-23)

With the rate right, the eye stays open for **48 seconds** and then shuts in a
single window -- 0.059 to 0.670, with `slips 0` reported -- and only flickers
back afterwards.  It is not a decay: read the CINR trace rather than the
average, or the constellation, and the shape is a cliff.

**The cliff is the peer's clock.**  `tools/measure_timing_slips.py` over
`artifacts/goal-v90-p2cap-r2-103404Z` finds the first one-sample slip of the
whole call at tap t=121.25 s, against a B1 handover at 73.0 s -- data
**t=48.25 s** -- and the eye shuts at t=49.0 s.  There are seven slips in the
following eleven seconds, in the bursts this peer is known for.  Receiver-
independent, measured off the wire, and unambiguous.

**The slip search cannot recover it, and that is now bounded rather than
suspected.**  `v90_t3_slip_resync()` fires 4470 times over the rest of the call
and adopts nothing.  Two hypotheses for why were tested and both are wrong:

- *The equalizer had walked off, so every candidate was scored through a
  filter matched to nothing.*  Scoring the search through the known-good
  snapshot instead changes the profiles by 0.01 and the outcome not at all
  (240 of 1402 clean windows either way).  Not committed.
- *A slip is a passband delay and carries a phase, so the rotation term was
  wrong.*  `ME_V90_SLIP_ROT_SWEEP=1` sweeps 16 static rotations at the best
  offset: the best is 0.60 wherever you look, the values repeat with the
  lattice's own 90 degree period, and the angle
  `v90_t3_offset_rotation()` computes is already the minimum.  The term is
  right.

So no (offset, rotation) pair within half a symbol either side and a full turn
recovers the constellation, just as at 31200 no gain, rotation or residual
frequency did.  **Recovery from a slip needs a re-acquisition, not a nudge** --
something with the reach of the B1 acquisition, run again mid-call -- and that
is the open work.  A search over the state the receiver is already in has now
been shown twice not to be able to reach it.

Note for whoever picks this up: `slips 0` in the DATA-bits line is the Gardner
loop's own correction count, not a statement about the wire.  The loop is gated
on `sym_err_fast < V34_V90_T3_TIMING_TRACK_ERR`, so a slip closes the eye, the
gate shuts, and the loop stops adapting exactly when it is needed -- which is
why the count stays at zero through seven real slips.

## §9.6 rate renegotiation: implemented, and this peer does not answer it (2026-08-23)

The section above ends by concluding that recovery from one of this peer's
one-sample timing slips needs a re-acquisition with the reach of the B1 search,
not a nudge to the state the receiver is already in.  V.90 §9.6 is that
mechanism and it is now implemented: §9.6.1.1 has the digital modem send Rd for
384T on a data frame boundary, R̄d for 24T, optional TRN2d and then MP, and the
analogue modem answer with S, S̄, SCR, CP and -- after E -- **a fresh B1**,
which is exactly what our upstream receiver acquires against.  V.34 §11.6 says
the same procedure "can also be used to resynchronize the receiver without
going through a complete retrain".

What is in the tree: `v34_v90_upstream_carrier_lost()` (symbols at the white
level for a sustained second, against the receiver's own settled baseline),
`v90_request_rate_renegotiation()` and `v90_rate_renegotiation_start()`,
§9.6's data-frame-boundary rule taken at the boundary the engine already tracks
for the data mapper, §9.6.1.1's "condition its receiver to detect S, S̄ and CP"
by re-entering the Phase 4 receiver, and §9.6.1's timeout falling back to the
§9.5.1.1 retrain.  Rd's 384T is a separate constant from startup Ri's "at least
192T" (§9.4.1.1) because the two clauses differ.

**It is default off (`ME_V90_RENEG=1` enables), because this rig's analogue
modem does not answer it.**  Two live calls sent Rd for 384T on a data frame
boundary and received no CP at all; the peer's own log declares
`SILENCERETRAIN` and it retrains.  On this peer the procedure costs a retrain
and buys nothing, so the honest default is to leave the link alone.

**One real defect of ours fell out of the first of those calls.**  §9.6.1's
timeout runs from the Rd→R̄d transition, and that transition needs the peer's
CPt -- so a peer that never answers never starts the clock, and the transmitter
held Rd for the rest of the call, which is what provoked the retrain.  §9.6.1
also says the digital modem "may initiate a retrain at any time during a rate
renegotiation according to 9.5.1.1", so the wait for an answer is bounded as
well as the wait for E: three seconds, against a conformant answer of S (128T),
S̄ (16T) and an SCR of at most 2000 ms.

**Call-to-call variance is large and worth knowing before drawing conclusions
from one call.**  Upstream lines delivered at 24000 across the session: 7945,
12657, 7618, 0 and **29194**.  The last is the feature off and is the best
result this path has produced -- **100% of windows clean and a 115.2 s unbroken
eye, the whole call** -- because that call met no timing slip at all.  The 0 is
a call whose eye never settled (baseline 0.338 against the usual 0.045).  So
the Phase 2 rate cap remains the fix; §9.6 is insurance for a peer that
implements the other half of the clause.

## The rate/time/eye matrix (2026-08-24)

`tools/soak/v90_rate_matrix.sh` sweeps the upstream rate and
`tools/eye_summary.py` reports the eye at each one, two live calls per rate.
The table is `docs/v90_upstream_rate_matrix.tsv`.  Every figure is the
receiver's own distance from the V.34 lattice, where 0.667 is the value for
symbols bearing no relation to it -- a ceiling, not an error bar.

| rate | settled | CINR dB | clean% | hold s | lines |
|---|---|---|---|---|---|
| 19200 | 0.014 / 0.012 | 19.7 / 27.8 | 14 / 74 | 18.9 / 60.5 | 0 / 20433 |
| 21600 | - / 0.019 | - / 21.7 | - / 2 | - / 8.8 | 0 / 0 |
| 24000 | 0.045 / 0.039 | 35.3 / 24.6 | **100** / 7 | **115.4** / 21.4 | **29197** / 5621 |
| 26400 | 0.084 / 0.090 | 26.2 / 26.0 | 4 / 2 | 10.6 / 8.4 | 1206 / 2357 |
| 28800 | 0.301 / 0.292 | 27.9 / 28.8 | 0 / 0 | 0.5 / 0.7 | 36 / 19 |
| 31200 | 0.168 / 0.168 | 30.4 / 30.3 | 2 / 1 | 7.3 / 4.0 | 1295 / 649 |

**Three columns, three different stories, and only the first is about the
rate.**

*Eye quality follows the constellation-power law.*  The lattice spacing is 2 at
every rate, so `settled` is directly comparable across rows, and it should
scale as 2^(bits/symbol) at a fixed channel SNR.  Scaling from 19200's 0.013:
predicted 0.022 / 0.037 / 0.062 / 0.104 / 0.175 for 21600 through 31200,
measured 0.019 / 0.042 / 0.087 / 0.297 / 0.168.  Four of the five land on it.
So the channel SNR really is constant across the sweep and each 2400 bit/s
step spends margin exactly as theory says.

*Hold time does not follow the rate at all.*  19200 gave 18.9 s and then
60.5 s; 24000 gave 115.4 s and then 21.4 s.  The spread within one rate is
larger than the difference between rates, because hold time is set by when the
peer's clock slips, not by the constellation.  **A single call at a given rate
therefore says almost nothing**, which is why this harness does repeats.

*Payload follows hold time, not rate.*  The best row in the matrix is 24000 at
29197 lines -- a rate whose eye is three times looser than 19200's -- because
that call met no slip.  Across the plain-V.34 rate calls the same session the
proportionality is almost exact: 45.1 s -> 929 lines, 39.6 -> 751, 20.5 -> 390,
13.7 -> 284, 4.1 -> 91, 2.7 -> 58.

**Open, and reproducible: 28800 is anomalously bad.**  It reads 0.301 and 0.292
on its two calls against a predicted 0.104, three times worse than the law that
fits every other row -- and worse than 31200 above it, which matches its own
prediction (0.168 against 0.175) almost exactly.  Both repeats agree at both
rates, so this is not call variance.  Something about the 28800 configuration
at 3200 baud is wrong in a way 31200 is not; the downstream mapper is identical
across all six rates (D=23, K=18), so it is in the upstream V.34 constellation
or its frame parameters.  Worth noting 28800 at 3200 baud is exactly 9
bits/symbol, the only integer in the sweep.

**Practical consequence: 24000 is the rate to run here**, which is what the
Phase 2 cap already sets -- not because its eye is best (19200's is three times
tighter) but because it is the highest rate whose eye still has margin, and the
payload is set by uptime rather than by the rate.

## The 28800 anomaly is one collapse, and the receiver has no way back (2026-08-24)

The matrix row above reads 0.301 and 0.292 at 28800 against a power law that
fits every other rate, and reads it on both repeats.  It is not the
constellation, and it is not the line.  **`v90_upstream_replay` reproduces the
whole matrix off the recordings** -- 24000 100% of the call clean with a
115.4 s hold, 28800 17%, 31200 7%, the same numbers the live calls gave -- so
the question can be asked deterministically, on the desk, in fifty seconds a
run.

Replayed, `artifacts/goal-matrix-115515Z/rate28800-r1` runs **19.7 s at 0.10 to
0.12 from the lattice with 0% bad shell frames -- exactly the figure its own
B1 out-of-sample check predicted** (0.107 at acquisition) -- meets a
disturbance at 19.5 s, and spends the remaining **95 s at 0.67**, the value for
symbols bearing no relation to the lattice.  The mean of those two stretches is
the 0.30 in the table.  Every rate acquires equally well and the ratio proves
it: distance over symbol power at the B1 check is 2.8e-4, 2.5e-4, 2.4e-4 and
3.1e-4 for 24000, 26400, 28800 and 31200.  One channel, one SNR, four
constellations.

**Why 28800 and not 31200: it is the highest rate whose eye is good enough to
lock and tight enough for a transient to look like a fault.**  The lock is
released on the shell-index check -- 9.6.3.3's r0 must fit in k bits -- which
owes nothing to what the peer is SENDING and was rightly chosen over the ones
fraction for that reason, but it owes everything to the symbols being right: a
ring index the noise moved puts r0 out of range exactly as a wrong grouping
does.  At 19.7 s the transient produced 4% bad frames, over the 3% threshold,
and the lock went.  31200 never locks in the first place, so it has nothing to
release; 24000 has the margin never to cross it.

**Then nothing can recover, and that is the deeper defect.**  Every adaptive
element in this receiver is gated on the symbols being near the constellation
-- the DD-LMS (`V34_V90_T3_TIMING_TRACK_ERR`), the timing loop, the
decision-directed carrier loop, and the slip search
(`V34_V90_T3_SLIP_ACCEPT_ERR`, 0.45, against 0.67) -- and each gate is right on
its own terms.  Together they are a trap: once the eye shuts there is no
decision to adapt on, so nothing adapts, so the eye stays shut.  The one escape
in the tree, restoring the last good equalizer, put back the filter that had
just stopped working, 255 times in that call, and never reopened anything.
Offline the collapsed symbols are not recoverable by any rotation or gain
either -- swept +/-45 degrees and +/-15%, 0.65 stays 0.62 -- so it is the
filter, not the carrier.

Two fixes are in, and one thing that looked like a third is not:

* **The frame-phase machinery now requires a healthy eye before it believes
  what it reads** (`v90_t3_phase_evidence_ok()`, `V34_V90_T3_PHASE_TRUST_MULT`,
  `ME_V90_PHASE_EYE_GATE=0` restores the old behaviour).  Neither the release
  rule nor the sweep score can separate a wrong grouping from symbols the eye
  no longer resolves, and the sweep compounds it: on rate28800-r1 it ran **1880
  sweeps**, stepping the phase every 0.1 s on symbols already at 0.67, each
  step resetting the Viterbi state.  A phase that is genuinely wrong leaves the
  symbols CLEAN -- 0.10 with 50% ones is the case that machinery exists for --
  so gating it on eye health costs it nothing it could ever have fixed.  24000
  still replays at 100% clean with its 115.4 s hold.

* **A diverged receiver can no longer report a perfect one**
  (`V34_V90_T3_DIVERGED_POWER`).  The lattice distance is not scale-invariant
  at the top end: the decision is `2*floorf(y/2) + 1`, and a float32 above 2^24
  has no fractional part left, so every symbol lands exactly on the grid.  A
  run whose taps had run away to a mean symbol power of **1.5e20** reported
  "decision error 0.0078, receive SNR 222.9 dB" for 75 seconds and looked, to
  every summary built on that number, like the best call ever recorded.  It is
  now named in the log and the equalizer is reset from the snapshot when it
  happens.

* **Blind (constant-modulus) recovery is built and is DEFAULT OFF**
  (`ME_V90_UPSTREAM_CMA=1` enables).  CMA is the one loop that needs no
  decisions, so in principle it is what reopens an eye that is shut, and it
  demonstrably can: episodes reached 0.11 and 0.12 from 0.68.  But those
  episodes were running against a dispersion constant that had fed back on its
  own output and reached 2.6e10, and with r2 correctly frozen from the settled
  symbols the loop no longer recovers these calls at all -- the clean fraction
  is unchanged on both 28800 recordings.  The 17% -> 82% that briefly appeared
  was the float32 artefact above, not a result.  What is kept is bounded and
  safe (r2 measured over 32000 settled symbols then fixed, the gradient
  normalised so the step is a fixed fraction of the taps whatever the
  constellation -- written with the bare `|y|^2 - r2` it diverges to NaN at
  28800 while looking stable at 9600 -- episodes capped at 1600 symbols,
  starting from the last good snapshot and putting it back on giving up).

**Open, and it is now the whole of the upstream's uptime problem: recovering a
receiver whose eye has shut.**  CMA from a snapshot is not enough on this
peer; what the collapse most looks like is a timing event that the slip search
cannot see because the slip search is itself gated off at 0.45.  Re-acquisition
-- which needs a B1 that has long gone -- or an ungated slip search run on the
band-edge timing recovery rather than on decisions, are the two candidates.

## The fourth-power carrier estimator was steering the frequency with noise (2026-08-24)

The section above left "recovering a receiver whose eye has shut" open and
named two candidates.  Neither was it.  **The slip search is not the answer**:
run verbose over the 28800 collapse it fires 6630 times and its profile is
**flat at 0.62-0.73 across every offset in the half symbol either side, while
the position it is already on scores 0.275** -- so the collapse is not a
re-alignable delay, and relaxing V34_V90_T3_SLIP_ACCEPT_ERR would only buy
noise.  **Nor is it the line**: 100 ms RMS over the collapse reads 303-318
throughout, dead steady.  **And it is NOT the DD-LMS ratchet**, which is the
fix the plain V.34 data mode needed for exactly this shape of failure and is
therefore the first thing to reach for.  Gating the T/3 DD-LMS on the
receiver's own settled error (0.096 on that call) rather than the absolute
0.35 makes it *much* worse: multiples of 3.0, 2.0 and 1.5 all collapse the
call after 2.0 s where the absolute gate holds it for 19.7 s.  **The
adaptation between two and four times the settled error is not the receiver
walking off -- it is what keeps it on.**  `V34_V90_T3_DD_GATE_MULT` is left in
at 0, with the measurement, so the next person does not spend the round.

What it actually is: **`v34_carrier_update()` switches from the
decision-directed loop to a fourth-power estimator when the symbols stop
meaning anything, and that estimator INTEGRATES ITS RESULT INTO THE
FREQUENCY.**  A fourth-power line is a real thing on the 4-point training
constellations this loop was written for; on the 768-point shaped
constellation V.34 uses at 28800 it is almost nothing, so the term adds noise
to the frequency -- and it runs exactly when the eye is shut, which is the
moment the receiver can least afford its frequency steered by a guess.  In the
log it is unmissable once you know: at the collapse the decision-directed
count freezes at 62538 and the frequency starts wandering, +0.00057, +0.00109,
+0.00048 rad/symbol.

`nda_freq_hold` holds the frequency while unlocked, and is set for the V.90
upstream loop only (`ME_V90_UPSTREAM_NDA=1` restores the old behaviour; the
plain V.34 path is untouched).  With the frequency held, the constant-modulus
recovery from the section above stops being worthless and becomes the other
half of the fix -- it was reopening the eye into a frequency the estimator had
meanwhile walked away -- so it is now **default on** at a step of 0.05
(`ME_V90_UPSTREAM_CMA=0` disables).

Measured over the rate-matrix recordings, one binary, both knob directions,
clean time and longest unbroken hold:

| row | before | after |
|---|---|---|
| 19200-r1 | 18% / 18.7 s | **69% / 36.5 s** |
| 24000-r1 | 100% / 115.4 s | 100% / 115.4 s |
| 26400-r1 | 9% / 10.6 s | 9% / 10.6 s |
| 28800-r1 | 17% / 19.7 s | **55% / 37.1 s** |
| 28800-r2 | 35% / 22.7 s | 35% / 22.7 s |
| 31200-r1 | 7% / 7.3 s | 7% / 7.3 s |

Nothing regresses, 24000 keeps its whole-call hold, and the two rows that move
are the ones that were collapsing.  Attribution: on 19200 the whole of the gain is the
frequency hold (18% -> 48%, with the blind loop contributing nothing either
way); on 28800-r1 the hold alone is 17% -> 23% and the blind loop takes it to
55%, which it cannot do without the hold.

A 48% -> 69% reading for the blind loop on 19200 appeared once and does not
reproduce: it came from a build whose dispersion constant was measured off the
wire as 75.9 where the constellation's own value is 76.0, and that 0.1%
difference is enough to change where a nonlinear loop ends up.  Treat the
blind loop's contribution as established only on 28800 and as chaotic
elsewhere.

**Read the clean TIME, never the window count.**  A white stretch emits short
windows and a clean one long windows, so a window-weighted percentage inverts
the answer -- it read 78% on a call that was 21% clean.

**Open.** 26400, 28800-r2 and 31200 still collapse and stay collapsed; the
blind loop fires on all of them and does not reopen the eye, and on 28800-r1
it diverges 22 times (caught by V34_V90_T3_DIVERGED_POWER, which restores the
snapshot, and that row is still the largest gain in the table).  Two of the
three collapse well before any lock could be blamed, so there is at least one
more mechanism here.

## Live verification, and the fact it turned up: no replay reproduces a live call (2026-08-24)

The rate/eye numbers above are all replays.  Run live against the d-modem rig
(artifacts/live-verify-231747Z), same binary, both knob directions:

| row | fix on | fix off |
|---|---|---|
| 28800 | 1% clean, 0.7 s hold, fails at 0.9 s (x2 calls) | 0% clean, 0.7 s hold, fails at 0.9 s |
| 19200 | 1% clean, 1.7 s hold, fails at 1.1 s | never reached data mode |

**The fix is neither confirmed nor refuted live, because live never gets the
clean stretch it protects.**  It is also not harmful: the control arm, with
all three knobs disabled, fails at the same instant.

The reason is worth more than the result.  **The live receiver collapses
within a second of B1 and every offline path fed the identical recorded
samples runs for nineteen seconds.**  On
`artifacts/goal-matrix-115515Z/rate28800-r1`, whose own live call failed at
0.7 s: `v90_upstream_replay` of its recording holds 19.7 s and reaches 55% of
the call clean, and `v90_engine_replay` -- which exists precisely to close
this gap, running V.8, Phase 2, Phase 3 and Phase 4 through the whole engine
off the same file -- holds 19.7 s as well, in `--fast` and in real time
alike.  Acquisition is identical to three decimal places (B1 fit 100%,
out-of-sample 0.103 against live's 0.107, symbol power 437.0 against 438.1),
so the two paths start in the same place.

Then they part, and the instrument that says how is the timing loop's own
frequency, printed in every DATA-bits line:

```
              slips  freq        sym err
live          0      +0.000002   0.106
              0      -0.000015   0.123
              0      -0.000051   0.151
              0      -0.000064   0.233   <- eye closing
              0      -0.000064   0.300
engine replay 0      +0.000023   0.118
of the same   0      +0.000006   0.115
recording     0      +0.000009   0.107
              0      +0.000009   0.107
              0      -0.000013   0.109
```

Live walks to -6.4e-5 samples per symbol -- **-64 ppm** -- inside half a second
and the eye shuts with it; the replay stays inside +/-2e-5 and stays open.  A
timing loop cannot invent a frequency offset out of samples that do not have
one, so **the live receiver is not consuming the samples the tap recorded**.
The tap is written in `me_rx_g711()` before anything else touches the buffer,
so a recording cannot carry a discrepancy introduced after that point, and a
frame the live path fails to hand to `v34_rx()` looks to the timing loop
exactly like a wire running slow.  Ruled out already: RTP (7076 packets, zero
sequence gaps, zero timestamp jumps, on both the matrix call and today's), the
notch and the canceller (both off, and both are downstream of the tap in the
engine replay too), the T/3 ring's history length (14 s, 2 s and 0.4 s all give
55%), and the feed block size (already 160, matching a 20 ms RTP frame).

**So every rate/eye figure in this document, and every improvement measured
against them, describes a receiver that is not experiencing the live path's
dominant impairment.**  That is the first thing to fix here, and it is
measurable without the rig once the discrepancy is instrumented: count the
samples `v34_rx()` actually receives against the samples the tap wrote.

### The live A/B, completed, and what the sample counter actually says

More live calls at 19200, which is the row where a replay and its own live
call agree on when the eye shuts (19.1 s against 19.0 s), so it is the one
worth running.  Counting only the calls that reached data mode -- the rig's
handshake fails intermittently in Phase 3/4, which none of this touches, and
those rows carry no information either way:

| arm | data-mode calls | hold | payload lines |
|---|---|---|---|
| fix on | 2 | 1.7 s, **40.7 s** | 1, **8586** |
| fix off | 1 | 39.2 s | 3429 |

At 28800 all five calls (three on, two off) collapse 0.9 s after B1 and
deliver 19-27 lines, so that rate says nothing about the change.

**Read that as "no harm, and the best live call of the session was with the
fix on", not as confirmation** -- two calls against one is not a measurement,
and the spread within one arm (1.7 s and 40.7 s) is larger than the
difference between the arms.  The defaults stay on because the mechanism
argument stands on its own and nothing regressed anywhere; they are one env
var to turn off (`ME_V90_UPSTREAM_NDA=1 ME_V90_UPSTREAM_CMA=0`).

**The sample counter is in and it does NOT show the defect it was built to
look for.**  It reports a steady 10% of arriving samples never reaching
`v34_rx()` -- and reports it on the healthiest live call in the session, the
one that held 40.7 s and delivered 8586 lines, as well as on the ones that
collapsed.  A shortfall that large would not let any call decode at all, so
it is almost certainly counting samples that are routed somewhere else by
design rather than samples that are lost.  `g_rx_audio_samples` now sits
between the two counters to say which, and offline `v90_engine_replay`
reports no shortfall at all through the same entry point, which is the
control.  **The live/replay divergence is still real and still unexplained;
this instrument has only ruled itself out as the answer so far.**

### The live A/B, finished: it does confirm, and the log says which piece

The last two calls changed the answer.  Every 19200 call that reached data
mode, with the release and sweep counts beside the payload:

| call | hold | payload lines | frame-phase releases | sweep steps |
|---|---|---|---|---|
| on r2  | 1.7 s  | 1 | 0 | 11 |
| on r5  | 40.7 s | 8586 | 0 | 0 |
| on r6  | **69.7 s** | **15100** | 0 | 1 |
| off r3 | 39.2 s | 3429 | 1 | 448 |
| off r5 | 65.7 s | **0** | 4 | 36 |

**23687 payload lines across three calls with the fix on, against 3429 across
two with it off** -- and payload is the honest measure on this path, not the
clean fraction.  `off r5` is the row that makes the mechanism visible rather
than statistical: it held an open eye for 65.7 seconds, 87% of its windows
clean, and delivered **nothing**, because it released the frame-phase lock
four times and spent 36 sweep steps hunting for a phase it already had.  That
is exactly the failure the phase-evidence gate exists to prevent, and no call
with the gate in released a lock at all.

So the live rig confirms the **phase-evidence gate** specifically: the counts
it acts on -- releases and sweeps -- separate the arms cleanly, 0 in every
"on" call against 1 and 4, and 0-11 sweep steps against 448 and 36.  The
frequency hold and the blind recovery were in the same arm and are not
isolated by this experiment; they keep the replay evidence and the mechanism
argument behind them, nothing more.

Three calls against two is still small, and the within-arm spread is large
(1.7 s and 69.7 s in the same arm).  What is not small is a 65.7-second open
eye delivering zero bytes with the gate out.

## The 28800 collapse: the equalizer the replay never inherits (2026-08-24)

Five 28800 calls die 0.9 s after B1 in both arms of that A/B, and every
offline replay of their own recordings holds.  That gap was read as evidence
that the live path was not feeding the receiver the samples the tap recorded.
It is not.

`v90_upstream_replay` on `live-verify-231747Z/on-28800-r1/live-rx.g711`
reproduces the live call's B1 exactly -- same acquisition, `B1-era distance
0.1055` against the live `0.1035`, same `B1 ended after 2 data frames` -- and
then does **not** collapse: symbol error stays at 0.10-0.12 and the timing
integrator inside +/-2e-5 for the whole file, where live walks to -6.4e-5
within half a second and reaches 0.66 symbol error by t=1.0 s.

Two things follow from measurements rather than argument:

* **Ring length is not the difference.**  Forcing the handover at the same
  instant with 14 s and with 0.4 s of history -- the live receiver had 0.35 s,
  prepare at bearer sample ~154,000 and E at ~156,800 -- produces trajectories
  identical to the printed digit: same `0.1055`, same `freq +0.000015`, same
  symbol error in every window.  The harness comment naming ring length as
  "the obvious suspect" can be retired.
* **The two receivers differ from the first data window, not from a later
  disturbance.**  Live and replay agree on the decision-directed carrier
  update counts (384, then 920) but not on the values in those same windows
  (`freq +0.000002` / `sym err 0.106` live against `+0.000015` / `0.115`).
  From t=0.4 s the counts part too -- live 1407 against 1448, then live's DD
  updates all but stop (1514, 1514, 1526) as the fourth-power loop takes over,
  while the replay keeps adapting on every symbol.

The receiver is deterministic in its input: no wall clock, no threads, no
block-size dependence (the T/3 resampler is exact-rational and carries its
state in a sample counter).  The echo canceller never ran in these calls and
the Phase-2 notch was retired before Phase 3, so `v34_rx()` sees the tap
byte for byte.  Same samples and same parameters with different trajectories
leaves receiver state at the handover, and there is exactly one piece of it
that a live call carries and a replay cannot:

```
    /* CP hypothesis acquisition may move the decision-aided phase tracker,
       but not the Phase-3 equalizer ... */
    cvec_copyf(s->rx.eq_coeff, s->rx.eq_coeff_save, ...);
```

`v34_v90_prepare_upstream_data()` restores the equalizer saved at entry to
Phase-4 CPt acquisition -- *after* the same function has retuned the carrier,
the shaper and the Godard coefficients to the selected upstream rate.  A
fractionally spaced equalizer's taps are a channel solution on the grid they
were adapted on.  Live, those taps come out of the real Phase 3/4; in the
replay `eq_coeff_save` is still the cold-start value, and the cold start is
the one that survives 28800.  It also fits the rate split: the same taps carry
19200's `b=48` and not 28800's `b=72`, which is why every 28800 call dies and
19200 calls run for a minute.

**Refuted by the live A/B, same day.**  `ME_V90_UPSTREAM_EQ_RESTORE=0` on a
live 28800 call reaches data mode on the first attempt and collapses on
exactly the old schedule: symbol error 0.087 at t=0.0, 0.215 at 0.4 s, 0.449
at 0.9 s, 0.602 at 1.0 s, with the timing integrator at -6.7e-5 by 0.4 s and
pinned at -6.9e-5 thereafter, and 22 intact U-lines out of 315,541 bytes over
a 105 s soak against the restored arm's 27 of 315,625.  A cold equalizer at
the handover changes nothing, so the state a live call inherits there is not
what kills 28800.  Capture: `artifacts/eqoff-28800-r1/`.

That call also removes the other standing suspect.  Only 149 samples separate
its RTP arrivals from what reached the engine (7073 packets, 1,131,680 sample
times, 1,131,531 fed), against 2235 on `on-28800-r1`, and the two calls
collapse identically -- so packets lost below the tap are not it either.  And
the paradox reproduces on this fresh capture: `v90_upstream_replay` over its
own `live-rx.g711` holds 0.09-0.11 symbol error with freq inside +/-3e-5 for
the whole file.

What survives is the shape of the thing.  Every live 28800 call settles its
timing integrator at -6.4e-5 to -7.6e-5 -- about -70 ppm of the sample clock,
half a sample per second -- and settles there within half a second, while
every replay of the same samples settles at zero.  The next probe is the
resampler's phase relative to the wire rather than its coefficients: the
exact-rational 6/5 interpolator's output instants are fixed by the absolute
input index it started counting at, live starts that count at the prepare
call and a replay starts it wherever its search put the handover, and there
are five distinct phases.  Sweep the forced handover by one sample at a time
and see whether any offset reproduces -7e-5.

The knob and the seam log stay in either way.  Each seam logs the pair it is
reconciling --

```
Rx - V.90 upstream data prepare: equalizer saved at baud 4 carrier high,
    preparing baud 4 carrier low
```

-- so a mismatched baud or carrier assignment across that seam is visible in
the capture instead of having to be inferred.  `me_init()` also prints every
`ME_`/`V34_`/`VPCM_`/`SIP_`/`SPANDSP_` variable it was started with, because
the arms of the last A/B cannot be reconstructed from the logs it left.

### The phase sweep: the collapse reproduces offline, and it is periodic in 5

Shifting the forced handover one sample at a time moves the absolute input
index the exact-rational 6/5 interpolator starts counting at, and therefore
which input samples land on which of its five phases.  Over
`eqoff-28800-r1`'s own recording, at the instant its own search picks
(25.5 s), with 0.4 s of history:

| offset | B1-era | t=0.0 | 0.4 | 0.9 | 1.0 | freq settles at |
|---|---|---|---|---|---|---|
| k=0 | 0.1107 | 0.108 | 0.111 | 0.112 | 0.093 | ~0 |
| k=1 | 0.1113 | 0.125 | 0.114 | 0.132 | 0.112 | ~0 |
| **k=2** | **0.0940** | 0.087 | **0.215** | **0.449** | **0.602** | **-6.9e-5** |
| k=3 | 0.1112 | 0.111 | 0.111 | 0.111 | 0.091 | ~0 |
| **k=4** | **0.0936** | 0.088 | **0.183** | **0.424** | **0.674** | **-7.8e-5** |
| k=5 | 0.1107 | 0.108 | 0.111 | 0.112 | 0.093 | ~0 |

k=2 is the live call, digit for digit: 0.087, 0.125, 0.215, 0.321, 0.404,
0.449, 0.602 with the integrator pinned at -6.9e-5 is exactly what
`artifacts/eqoff-28800-r1/server.log` recorded off the wire.  **The live
28800 collapse now reproduces offline from a recording**, which is what three
commits of live A/B were missing.

The period is 5, replicated: k=6, k=7 and k=9 match k=1, k=2 and k=4 to every
printed digit including the B1-era distance.  Two of the five phases collapse
and three hold, and the two that collapse are the two that fit B1 *closer* to
the lattice (0.0936/0.0940 against 0.1107-0.1113) before winding the timing
loop to -70 ppm.

**The 19200 control shows no phase sensitivity at all.**  The same sweep over
`on-19200-r6` gives B1-era 0.0118-0.0119 and symbol error 0.012-0.017 on all
five phases, with freq inside +/-3.5e-5 everywhere.  So this is not "the
interpolator is broken": 19200 sits ten times closer to the lattice than
28800 does, and a phase-dependent perturbation that is fatal at 0.11 is
invisible at 0.014.  28800 has no margin, and the phase is what tips it.

What is *not* established is that this is the whole live cause.  Live died
5/5 at 28800 while the sweep says 2 phases in 5 fail, so either the live
starting phase is not uniformly distributed -- the count begins at the
prepare call, whose instant is set by CP' acceptance and may well correlate
with the protocol's frame grid -- or the phase is one contributor on top of a
marginal eye.  The probe for that is cheap and needs no rig: log the prepare
sample mod 5 and compare it across the five 28800 calls already captured.

## The timing loop was the wrong instrument for a constellation carrying data

The previous section left "recovering a receiver whose eye has shut" as the
whole of the upstream uptime problem, with the collapse reproduced offline
and traced as far as the interpolator's phase.  It goes one step further
than that: **the thing that shuts the eye is our own timing loop**, and on
the reproducible failing phase it is provable in one line.

`v90_upstream_replay artifacts/eqoff-28800-r1/live-rx.g711 ulaw 3200 28800
25.50025 0.4` -- the k=2 handover phase, which reproduces the live 28800
collapse digit for digit -- runs 0.087, 0.125, 0.215, 0.321, 0.404, 0.449,
0.602 with the timing integrator pinned at -6.9e-5.  With
`ME_V90_UPSTREAM_TIMING=0` the same samples through the same binary hold
**0.095 for the whole file**.  DD-LMS off, the carrier loop off and the blind
loop off all still collapse; only the timing loop's removal holds it.

The integrator value is a symptom, not the mechanism.  -7e-5 samples/symbol
moves the sampling position by 0.05 samples over the 0.4 s in which the error
goes 0.087 -> 0.215, which is far too little to close an eye; and it is
*pinned* rather than drifting because the loop's own track gate freezes it
once the error passes 0.35.  What actually happens is that the loop is
steering on noise.

### Gardner's error is data self-noise on anything but a training sequence

Gardner's detector is non-data-aided -- that is its virtue during acquisition
-- and its difference term is `(y[k] - y[k-1])`, so what it reports is
dominated by how far apart two RANDOM constellation points happened to fall.
That grows with the constellation while the true timing error does not.  The
V.34 training sequences this loop was built and tested against are four
points.  The V.90 upstream is carrying hundreds by 28800 bit/s.

`v34_gardner_test` now measures it, at a small offset because that is the
regime a loop in lock lives in.  Self-noise per unit of S-curve slope:

| detector | four points | 16 levels per axis |
|---|---|---|
| Gardner | 0.27 | **0.77** |
| decision-directed Gardner | -- | 0.48 |
| Mueller and Muller | -- | **0.00** |

Mueller and Muller (IEEE Trans. Comm. 1976) reads the residual intersymbol
interference the offset leaves on the neighbouring symbols rather than the
shape of the transition:

    e[k] = Re{ conj(a[k-1])*y[k] - conj(a[k])*y[k-1] }

Its expectation is `sigma^2*(h(T-tau) - h(T+tau))`, which is zero at tau = 0
with no data-dependent term at all -- hence a self-noise of nil, whatever the
constellation.  V.34 puts every point on odd integers, so the decisions it
needs are the same slice the DD-LMS already makes.

It is not simply the better detector: its error is built from the decisions
alone, so an instant far enough out droops the amplitude past the slicer's
boundaries, the decisions go wrong and its S-curve flattens exactly where a
loop would need to pull in.  Measured, on 16 levels per axis, the slope falls
from +0.2175 at +/-0.05 of a symbol to **+0.0004** at +/-0.15.  That costs
nothing here, because the fractionally spaced equalizer is fitted to B1 by
least squares and owns acquisition; the loop is only ever asked to track.

### Measured over the recorded rate matrix

`tools/v90_upstream_bench.sh` replays a directory of captures in parallel and
scores each on clean TIME -- never window counts, for the reason recorded
above.  One binary, the detector as the only variable, nine calls that reach
data mode, as total clean seconds:

| detector | total clean | 19200-r1 | 24000-r2 | 26400-r1 | 28800-r1 | 31200-r2 |
|---|---|---|---|---|---|---|
| Gardner (was the default) | 318.6 s | 47% | 34% | 9% | 54% | 2% |
| loop held still | 376.1 s | 16% | 36% | 67% | 55% | 21% |
| decision-directed Gardner | 390.2 s | 49% | 34% | 67% | 54% | 2% |
| Gardner, gains 5x slower | 420.6 s | 57% | 35% | 67% | 55% | 20% |
| **Mueller and Muller** | **492.0 s** | **85%** | **65%** | **86%** | 39% | 20% |

Two things worth reading off that table.  Holding the loop still is *better*
than running it at four of the rates -- which is what says the loop was
subtracting rather than adding -- and it is much worse at 19200, where the
eye has margin to spare and tracking is all that matters.  Every quieter
detector recovers both ends at once.  The longest unbroken hold on 19200-r1
goes 36.0 s -> **59.7 s**, and on 26400-r1 10.4 s -> **65.3 s**.

Mueller and Muller is now the default; `ME_V90_UPSTREAM_TIMING_DET=gardner`
restores the old detector, `dd` and `auto` select the other two.  The single
regression is 28800-r1, 54% -> 39%.

**A hybrid does not fix that and is measured: `auto` -- Mueller and Muller
while the error says the decisions are trustworthy, Gardner otherwise --
totals 379.5 s.** It recovers 28800-r1 to 54% and gives back 19200 (85% ->
53%), 24000-r2 (65% -> 19%) and 31200-r2 (20% -> 6%).  Switching detectors
inside a call disturbs the loop more than Gardner's noise costs, so the knob
is left in with the measurement beside it rather than adopted.

None of this touches plain V.34: `v34_gardner_update()` has exactly one
caller, the V.90 upstream T/3 receiver.

## What is left at 19200 is the wire, and it is measured

The section above changes the receiver.  This one says what is now in front
of it, live, at the rate the receiver is best at.

A live 19200 call on 2026-08-25 (`artifacts/det-live3-104534Z/mm-r1`) acquires
B1 at an out-of-sample lattice distance of **0.011** and holds **0.013** --
an enormous margin, ten times better than 28800's 0.11 -- for 0.8 s.  Then:

    t=0.8  slips=0  e=0.013
    t=0.9  slips=0  e=0.048
    t=1.0  slips=1  e=0.099
    t=1.1  slips=1  e=0.085     ... and the call never returns to 0.013

**Two things follow, and the first contradicts the previous entry.**

**The replay of that call's own tap reproduces it**: clean 0.013 through
0.8 s, a whole-sample correction at 1.1 s, and 0.112 immediately after.  The
"no replay reproduces a live call" result stands only for the 28800 captures
it was measured on; here the disturbance is in the recorded samples, so it is
on the wire and not something the live path does to them.

**And the wire is slipping about once a second.**
`tools/measure_timing_slips.py` reads the transmitter's symbol epoch out of
the phase of the symbol-rate line in the squared analytic envelope -- it owes
nothing to our receiver -- and over the 6.5 s around that handover:

    t=162.05  +0.92 sample     t=166.25  -0.91 sample
    t=163.10  -0.90 sample     t=167.30  +0.91 sample
    t=164.10  +0.67 sample

Data mode starts at 163.0 s, so the collapse at t=1.1 s **is** the slip at
164.10 s.  For comparison the 2026-08-21 soak measured 28 slips in 290 s, one
per ten seconds; this is an order of magnitude worse, and it is the binding
constraint on a receiver whose eye is otherwise 0.013.

### Two corrections that look right and are not

**The timing loop's whole-sample correction must NOT be derotated.**  A slip
is a passband delay and the T/3 ring is mixed on the absolute sample count,
so the slip *search* derotates its own moves by
`v90_t3_offset_rotation()` -- 68.6 degrees a sample at 3200 baud on the low
carrier -- and it is tempting to conclude the loop's own corrections need the
same.  They do not, and the code comment beside the actuator already says
why: the loop takes the whole sample out of `acc` when it hands one back, so
the sampling position is continuous and there is no jump to undo.  Adding the
compensation injects a 68.6 degree kick that is not there: the same call goes
from 27% clean to 14%, and the error after the slip from 0.112 to 0.615.

**Arming the slip search relative to the settled error does not recover it
either.**  The search is armed on an absolute 0.35, which is blind on a call
that settles at 0.018 and steps to 0.112 -- a fault eight times its operating
point that never crosses the threshold.  Arming it at a multiple of the
settled error instead (`ME_V90_UPSTREAM_SLIP_MULT`, `V34_V90_T3_SLIP_MULT`)
gives **25% of the call clean at a multiple of 6 against 27% with the
absolute arm alone**, and 3 is no better: the search finds no offset that
scores better than the one it is on.  So whatever a bearer slip does to this
receiver is not a whole-sample move of the symbol instant that a T/3 search
can undo.  Default 0 -- the behaviour that predates it -- with the knob and
the measurement kept.

**Open, and it is now the whole of the live problem at 19200: survive a
bearer slip.** The eye is not the constraint any more; recovery is.  Whether
the slips are the peer's audio generation or the rig's resampler is not
settled here -- the RTP for this call has two losses, both 155 s before data
mode, and nothing during it.

### Note

`tools/measure_timing_slips.py` needs numpy, which the system python here
does not have and PEP 668 will not let pip install into.  A throwaway venv is
the way: `python3 -m venv /tmp/np-venv && /tmp/np-venv/bin/pip install numpy`.

## The bearer was never lossy — we were splicing the samples ourselves (2026-08-25)

Every entry above chases the upstream eye shutting mid-call, and the last one
left the live/replay divergence unexplained: a live call dies about a second
after B1 while both replays of its own recording hold nineteen seconds, with
the live timing loop walking to −64 ppm that the same samples do not contain.

**The bearer is not the problem, and that is measured, not assumed.** Over
every RTP trace in `artifacts/` — 1,436,859 packets, segmented by SSRC so
call boundaries are not counted as gaps — real loss is **35 packets, 13 gap
events, 0.0024%**. On `goal-matrix-115515Z/rate28800-r1` specifically there is
no loss at all: 7074 packets, and the RX tap holds 1,131,682 bytes against the
1,131,840 those packets carry, the 158-byte difference being the partial frame
at the end. Every sample the wire delivered reached `me_rx_g711()`.

**What killed the calls is `clock_recovery.c`, applied to the received
stream.** `modem_passthrough_get_frame()` asked `me_cr_get_adjustment()` on
every pulled frame and, when it answered ±1, duplicated the frame's last
codeword or dropped it. Across the twelve-call rate matrix:

| call | cr adjustments | outcome |
|---|---|---|
| rate24000-r1 | **0** | **115.4 s, 100% clean** |
| the other eleven | 8 – 81 | all collapse |

The only call in the matrix with no slips is the only one that ran clean.

### The cost of one slip, measured

`tools/inject_sample_slips.py` reproduces the edit byte-exactly on a recording
that has none — the RX tap is written *inside* `me_rx_g711()`, i.e. after the
splice, so a recording already contains whatever slips its own call injected
and can never show what they cost. Replaying `rate24000-r1`:

| injected | clean windows |
|---|---|
| nothing | **100%** (578/578), `sym err` 0.037, 0% bad shell frames |
| one duplicated codeword at 35 s | 2% |
| … at 50 s | 7% |
| … at 90 s | 49% |
| … at 70 s | 100% |
| … at 110 s | 99% |
| duplicate at 40 s, **opposite** slip at 60 s | 54% — it *recovers* at the second |

So a slip is not reliably fatal: three of five instants collapse to 0.68 for
the rest of the call, two are absorbed. One sample is 0.4 T at 3200 baud,
inside the half-symbol the receiver's slip search covers — but that search is
gated on the eye still being open, so it helps only when it fires first.
Surviving the eight slips a quiet call injects is that coin flip won eight
times, which is exactly why the zero-slip call is the only clean one.

That the *opposite* slip recovers a collapsed call is what proves the mechanism
is the **net sample offset** rather than the momentary disturbance.

### It was never closing a loop that needed closing

`cr_update()`'s error is RTP timestamp progression against **our host's wall
clock** — the two oscillators' mismatch. It never measures jitter-buffer
occupancy, and editing the payload of a frame the buffer has already handed
over cannot influence occupancy either. Meanwhile the V.34-family receivers
downstream carry a fractional interpolating timing loop whose entire job is
that same mismatch, continuously and without splicing; `v34_gardner_test`
asserts it tracks 50 ppm, against the 7.2 ppm the peer actually drifts
(`tools/measure_timing_slips.py`). The correction duplicated the receiver's
job in the one way the receiver cannot absorb.

The splice is now off by default (`ME_RX_CLOCK_SLIP=1` restores it for A/B
work) and gated identically on all three receive paths — the linear one never
consulted the existing gate at all. `cr_update()` is still fed, because drift
is worth measuring; only the splice is gone.

### Loss concealment, and why 160 samples is not one number

A lost packet used to *delete* 160 samples: `get_frame()` returned nothing and
the code fed nothing. Whether that matters depends on the symbol rate, because
160 samples is a whole number of symbols only at some of them — 64 at 3200
baud, 60 at 3000, 48 at 2400, but **68.58 at 3429 and 54.86 at 2743**, where a
lost packet is a fractional-symbol step of exactly the lethal kind. Replayed at
3200 baud, losing one packet the way we did costs 1% of the call and concealing
it costs nothing (99% vs 100%). The receive path now feeds a frame of fill
(0xFF u-law, 0xD5 A-law) when nothing arrives, so the stream keeps its length,
and reports `concealed_frames` beside the RTP trace's sequence gaps.

**Caveat on scope.** The recordings already contain the slips their own live
calls injected, so no replay can show the fix's live benefit — that needs a
live A/B on the rig, and it is the open confirmation. What is established here
is the cost of the operation removed, and that the operation had no loop to
close. Note also that the −64 ppm live/replay divergence is consistent with
this (a spliced stream is not the recorded stream) but is **not** thereby
proven to be its only cause.

### Validated by undoing the splices on the real calls (2026-08-25)

The injection experiment above measures what a slip costs a clean call. The
converse is the one that answers "does this lengthen connects", and the
recordings can answer it: because the tap is written inside `me_rx_g711()`,
*after* the splice, every recorded call carries its own splices, and the
server log records each one's sign and brackets it in time between two
DATA-bits lines. `tools/desplice_call.py` undoes them — a −1 (a dropped
codeword) by inserting one, a +1 (a duplicated one) by deleting one. Position
inside the bracket does not matter, because a splice is a net sample offset:
the compensating edit restores alignment for everything after it wherever it
lands, corrupting only the ~1600 samples in between. That corruption is damage
the real fix does **not** cause, so these figures understate it.

`tools/desplice_matrix.sh` replays every call twice. Scored on clean **time**
and longest unbroken hold, never window counts — a white stretch emits short
windows and a clean one long windows, so a window-weighted percentage flatters
a call that spent its seconds white:

| call | splices | as-taped clean / hold | de-spliced clean / hold |
|---|---|---|---|
| rate19200-r1 | 3 | 100.7s / 59.8s | 114.8s / **114.8s** |
| rate19200-r2 | 11 | 114.8s / 114.8s | 114.5s / 89.7s |
| rate21600-r2 | 24 | 35.5s / 19.0s | **132.0s** / 61.7s |
| **rate24000-r1** | **0** | 115.4s / 115.4s | **115.4s / 115.4s** |
| rate24000-r2 | 6 | 81.5s / 44.9s | 93.8s / 43.0s |
| rate26400-r1 | 11 | 98.0s / 65.5s | 109.6s / 66.2s |
| rate26400-r2 | 16 | 8.4s / 8.4s | **96.6s** / 69.9s |
| rate28800-r1 | 5 | 45.3s / 25.5s | 108.7s / 44.4s |
| rate28800-r2 | 10 | 22.7s / 22.7s | 57.9s / 34.7s |
| rate31200-r1 | 13 | 8.3s / 8.3s | **100.7s** / 52.7s |
| rate31200-r2 | 9 | 24.1s / 21.6s | 102.9s / 54.6s |
| **total** | | **654.7s / 505.9s** | **1146.9s / 747.1s** |

**+75% clean time and +48% longest hold.** Ten of the eleven improve;
`rate19200-r2` is the one regression (hold 114.8s -> 89.7s), and it is the call
with the widest brackets, so its compensating edits are the least well placed.

**`rate24000-r1` is the control and it is exact.** It is the only call with no
splices, and de-splicing it returns **+0.0s clean, +0.0s hold** — byte for
byte the same result. A method that changed a call with nothing to undo would
be measuring its own edits; this one does not.

Two ordering facts sit beside it. Counting only in-data splices, the collapse
follows the first one within **0.4–0.9 s** in five of the eleven calls
(18.1->19.0, 14.6->15.3, 8.5->9.2, 10.2->10.8, 8.2->8.6) — the timescale of the
adaptive loops walking off after a disturbance. In the other six the collapse
comes first, so splices are **not** the sole cause of every collapse.

**What this is not.** It is a measurement of the receiver on a reconstructed
stream, not of a live call. The live A/B remains open, and note that the
replay of `rate19200-r1` holds 60 s where its own live call died at 19 s: the
recording contains the splices and the replay survives them longer than live
did, so the live/replay divergence of 2026-08-24 is **still unexplained** and
is not resolved by this. What is established is that removing these edits
lengthens the connection substantially on every real call that has them.

### The live A/B: confirmed, and it does not touch the handshake (2026-08-25)

Twelve live calls against the rig at 19200, six per arm, **arms alternated
rather than run in blocks** so a drifting rig cannot masquerade as an effect,
one binary, `ME_RX_CLOCK_SLIP` the only variable. Each run greps its own
server log to confirm the knob took effect in that call.
`artifacts/slip-ab-211840Z/`, harness `tools/soak/v90_slip_ab.sh`.

| arm | calls | reached data mode | clean | longest hold | U-lines | splices |
|---|---|---|---|---|---|---|
| fixed (default) | 6 | 2 | 228.4s | 228.4s | **46716** | **0** |
| slip (`ME_RX_CLOCK_SLIP=1`) | 6 | 2 | 180.8s | 147.5s | 6727 | 22 |

**Data-mode reachability is identical — 2 of 6 either way — so the change does
not touch the handshake.** That matters, because the first two calls of the
session came out 0/2 fixed against 2/2 slip and looked like a regression. It
was chance: across 228 recorded runs in `artifacts/` the base rate of a run
reaching data mode is 0.57, which puts that split at p≈0.06, and a Fisher
exact test on 0/2 vs 2/2 is p≈0.17. **Do not read a live arm before it has
finished** — a run's directory exists from the moment it starts, and scoring it
early reports a call that has not happened yet as a failure; that misread
happened twice here before `slip_ab_summary.py` learned to wait for
`v90_notch_ab.sh`'s last line.

Conditioned on the calls that reached data mode, two per arm:

- **Both fixed calls held the entire call unbroken** — 114.8s and 113.6s with
  `clean == hold`, i.e. not one window left the lattice — and delivered
  **23353 and 23363 intact `U%07d` lines**.
- The slip calls: one broke at 35.0s after 19 splices (6727 lines), the other
  had only 3 splices, held 112.5s, and delivered **zero**.

**6.9x the payload, and every fixed call ran the whole call clean.** Read the
line counts, not the byte percentages, and note the two arms' clean seconds
(228.4 vs 180.8) understate it badly: the slip arm's second call contributes
112.5 clean seconds that carried no payload at all.

**Honest limits.** Two data-mode calls per arm is small, and the arms are not
separated on longest hold (114.8s vs 112.5s) — what separates them is that the
fixed arm did it on *both* calls and delivered payload on both. That
`slip-r2` held a clean eye for 112.5s and delivered nothing is the
frame-phase-lock problem of the entries above, which is independent of this
change and still open; it is also why payload is not purely a function of hold.
The 3-splice call surviving while the 19-splice call broke at 35s is exactly
the dose-response the offline injection predicted.
