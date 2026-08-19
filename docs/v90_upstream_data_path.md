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
