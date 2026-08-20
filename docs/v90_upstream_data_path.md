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
