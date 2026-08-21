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

What is left is the anchor.  The replay wins by sweeping the handover instant
half a second at a time until one lands -- its first attempts score
`coarse=7.1%`, nowhere near B1 -- while live has the single anchor its E
detector gives it and scores `coarse=96.6%`, a near miss with an in-sample fit
of 98.3-98.8%.  A near miss is what a matched filter returns when the thing it
is matching straddles an edge of the searched span, and the coarse pass steps
a symbol at a time from `search_start` while the refine only looks +/-2
samples around the eight best coarse points.  That is where to look: whether
`SEARCH_FORWARD` and `SEARCH_BACK` actually bracket B1 on a live anchor, and
whether the true position survives the KEEP=8 shortlist.
