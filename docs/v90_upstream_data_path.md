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

What that leaves, in order of suspicion:

1. **The equalizer is never adapted.**  Seven taps, least-squares fitted
   once over the 128 symbols of B1, then frozen for the rest of the call
   -- and the residual it leaves is only ~1.7 sigma of the decision
   half-distance (above).  A few percent of raw symbol errors is enough
   to make shell-decoded output look white.  Decision-directed LMS on
   those taps through DATA is the obvious next move, and the metric to
   judge it by is already in place.
2. **B1 may be longer than the one data frame we model**, in which case
   publishing begins inside B1 -- but that would show as ones, not white,
   so this only matters once the error rate is down.
3. Superframe/V0 phase, if anything in the shell path depends on it
   beyond the data-frame position already handled.

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
