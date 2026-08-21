# V.34 duplex specification gap audit

## Scope

This audit covers full-duplex V.34 on the GSTN.  The normative reference is
ITU-T V.34 (10/96), `ITU Docs/T-REC-V.34-199610-S!!PDF-E-1.pdf`.
Half-duplex operation in clause 12 is out of scope until the duplex path is
hardware-qualified.

The V.90 digital and analogue roles already exercise much of the same V.34
implementation.  Work here should therefore make the existing V.34 machinery
consume negotiated parameters; it must not create a parallel datapump.

## Parameter authority

V.34 selects parameters independently in the answer-to-call and call-to-answer
directions.  The authoritative inputs, in order, are:

1. the local capability ceiling and the peer's INFO0 capabilities;
2. the receiver's L1/L2 measurements;
3. INFO1c's six probing-result rows and INFO1a's final symbol-rate selection;
4. the mutually valid MP/MP-prime fields used for data mode.

Environment settings may reduce the local capability offered for a diagnostic
run.  They must not replace a valid selection received in INFO1 or MP.

The following may be static because the Recommendation defines them: symbol
rate/carrier ratios, pulse-shaping and pre-emphasis tables, scrambler taps,
PP/J/J-prime patterns, CRCs, and normative T-counts.  Symbol rate, carrier,
pre-emphasis, power reduction, data rate, shaping, trellis choice, nonlinear
encoding and precoder coefficients are session state.

## Clause matrix

| V.34 clause | Required duplex behaviour | Existing implementation | Gap / acceptance evidence |
|---|---|---|---|
| 5.1-5.4 | Six symbol rates, legal carriers and selected pre-emphasis | TX/RX tables exist for 2400, 2743, 2800, 3000, 3200 and 3429 | Exercise every rate and carrier selected through INFO1.  At 3429 both carriers are 1959 Hz; this requires echo cancellation and is not an unsupported rate. |
| 6.1-6.2 | Synchronous datapump plus asynchronous V.14 adaptation | Common data stack is connected | Require byte-exact duplex V.14 at multiple line/DTE ratios. |
| 7-9 | Directional scrambler, framing, shell mapping, differential/nonlinear/trellis encoding and precoding | Received MP encoder fields configure TX while the locally sent MP configures RX; MP1 coefficients reach the TX precoder and MP0 preserves them. `v34_data_test` passes exact Q9.7 payload at every legal 2400-baud rate for 16/32/64-state trellis. B1 resets scrambler/trellis/V0 state per §10.1.3.1 and now calibrates phase, gain and conjugation from its known waveform before reset-state replay. §10.1.3 modulation-factor compensation measures a reset-state superframe and keeps DATA at PP/TRN power instead of clipping higher-rate constellations. | Extend the exact and waveform matrices to every baud and shaping/nonlinear/precoder combination. |
| 10.1.2.1-10.1.2.3 | Tone A/B and INFO0/INFO1 framing | Live V.90 work substantially hardened the shared Phase-2 state machine | Run plain-V.34 caller and answerer recovery cases, not only V.90 role inversions. |
| 10.1.2.3.4 | INFO1c reports measured per-rate carrier, pre-emphasis and projected rate | L1/L2 measurements and a per-rate evaluator exist | Plain V.34 previously emitted configured rates with pre-emphasis 6.  INFO1c must be built from the measurement for every enabled row and report -512 when frequency offset is unavailable. |
| 10.1.2.3.5 | INFO1a selects both directional rates and the call-to-answer carrier/pre-emphasis/rate | Parser and serializer exist | Selection must combine INFO1c, local L1/L2 results and both INFO0 asymmetry limits.  Configure TX and RX independently from the result. |
| 10.1.3.1-10.1.3.9 | B1, E, J/J-prime, optional MD, PP, S, TRN and MP | Shared generators and receivers exist; `v34_duplex_test` provides a waveform-only two-instance bearer. E is the normative single 20-bit sequence, E detection is gated by a complete MP-prime, Phase-4 CMA is limited to the first 512T of TRN and frozen for MP, and B1 is received as the complete known data frame before payload is unclamped. | At 2400/9600, PCMU and PCMA both complete startup, synchronize payload and recover over 16,000 bits per direction without error. The former PCMA stall was a false §10.1.3.7 S event on A-law digital silence; S publication now requires non-trivial equalized-symbol energy. Both laws run in `make test`. |
| 11.2 | Probing/ranging and recovery | Main flow exists, with stage/event tracing | Test both roles, INFO retries, reversal deadlines and measured RTD. |
| 11.3 | Equalizer and echo-canceller training | Phase-3 TX/RX exists | Replace the narrowband-notch policy with negotiated-carrier retuning and an echo-canceller path where carriers cannot be separated. |
| 11.4 | Final training and mutually valid MP selection | MP/MP-prime handshake and heuristic receiver exist. Directional maxima now come from INFO1, the local mask intersects both selected baud mappings, final rates intersect both MP masks/maxima, and bit 50 forces the lower symmetric rate unless enabled bilaterally. MP-prime changes only the acknowledge bit rather than rewriting the offer. | `v34_mp_test` covers asymmetric/symmetric maxima, sparse/disjoint masks and all 1..14 maxima. Matrix-test waveform transport for 4/16-point TRN/MP, MP Type 0/1, E and complete B1. |
| 11.5 | Local and peer-initiated retrain | V.90-specific retrain handling exists | Implement and test the plain-V.34 Tone A/B procedures in both roles. |
| 11.6 | Rate renegotiation | V.90-specific external-symbol seam exists | Implement plain-V.34 S/S-bar/TRN/MP/E renegotiation while preserving carrier/timing state. |
| 11.7 | Cleardown | No qualified duplex procedure | Implement zero-rate MP exchange and deterministic DTE carrier loss. |

## Ordered implementation and qualification

1. Make INFO1c and INFO1a measurement-driven and preserve the selected
   parameters independently for TX and RX.
2. Make Phase 3/4 and the media echo-control policy consume that negotiated
   state rather than the startup profile.
3. ~~Audit MP parameter authority and rate selection through B1/data mode.~~
   Directional encoder settings and bilateral rate/mask negotiation are now
   explicit and unit-tested.  Waveform qualification of the advertised
   options remains part of the harness matrix below.
4. **In progress:** `v34_duplex_test` connects independent caller/answerer
   instances through only a selectable PCMU/PCMA round trip and verifies PRBS
   payload in both directions after training and now runs in the green default
   suite for both laws. At 2400/9600 PCMU and PCMA both complete INFO1,
   S/S-bar, PP, TRN, J, MP/MP-prime, E and B1 from waveform evidence. The known B1 frame now
   calibrates arbitrary phase, gain and conjugation before reset-state replay;
   both receivers then synchronize and recover over 16,000 payload bits with
   zero errors. Enforcing the configured 9600-bit/s INFO1/MP ceiling removed
   accidental 21600-bit/s negotiation, while §10.1.3 modulation-factor
   compensation removed high-rate clipping. Phase-4 CMA is bounded to the
   first 512T and frozen during framed signalling as required by §11.4. The
   PCMA blocker was a false S event: A-law silence decodes to ±8, and the
   near-zero equalizer output looked like sustained rotation. Requiring real
   symbol energy before the §10.1.3.7 event makes both laws follow the same
   path. The harness previously exposed and fixed four
   sequencing defects: the caller no longer abandons the control channel
   before INFO1a; J detection is published to the caller transmitter; the
   answerer becomes silent and conditions on caller PP/TRN/J after S/S-bar;
   and 11.4.1.1 now sends caller J-prime followed by at least 512T of TRN,
   rather than sending MP immediately.
5. **In progress:** run `{2400,2743,2800,3000,3200,3429}` with legal
   carrier, rate and asymmetric-rate combinations; assert external sample
   accounting.  `v34_duplex_test <baud> <bps> <ulaw|alaw>` already takes
   the rate, so the matrix is one loop.  Measured state at 9600 bit/s over
   the bare G.711 round trip:

   | baud | u-law | A-law |
   |---|---|---|
   | 2400 | payload, 0 errors | payload, 0 errors |
   | 2743 | payload, 0/149 errors | payload, 0 errors |
   | 2800 | no training | no training |
   | 3000 | no training | payload, 0 errors |
   | 3200 | no training | no training |
   | 3429 | no training | no training |

   When this matrix was first run every rate except 2400 timed out at 60 s
   with nothing trained, and three separate defects were behind that:

   a. **The answerer's 11.3.1.2.6 J-wait bound was 1000 ms**, which no
      conformant call modem can finish Phase 3 inside (S, S-bar, PP, 2048T
      of TRN and J exceed it at every symbol rate).  The interop escape
      therefore fired on the normal path, and the answerer ran ahead into
      Phase 4 while the caller was still in TRN.  2400 passed only because
      the timing happened to land the other way round.  Now 4000 ms
      (`V34_J_WAIT_MAX_MS`).

   b. **The receiver ran 2800 baud 1.56% off the symbol rate.**  The T/2
      accumulator advance is samples-per-symbol in units of 1/192 of a
      sample, and 2800 is the one rate whose value (192*20/7 = 548.57) is
      not a whole number of coefficient sets; the table wrote 189 in place
      of 192 for that row alone to force an integer.  The remainder is now
      carried, so the long-run rate is exact.  The five exact rates are
      bit-identical to before.

   c. **Phase 4 CMA never stopped.**  11.4 begins from the tap solution
      11.3 already trained, which over this bearer is right (Phase 3 TRN
      demodulates at 4th-power coherence 0.98 at every rate) but is scaled
      for Phase 3, not Phase 4.  Blind CMA corrects the level and then
      keeps reshaping the solution with a phase-blind per-tap gradient;
      the existing 512T bound is keyed on `phase4_trn_after_j`, which only
      advances once J' has been seen, so it could not stop it.  CMA now
      stands down once the level estimate settles
      (`ME_V34_PHASE4_CMA=full` restores the old loop).

   What remains is not the same defect at the remaining rates.  2800, 3200
   and 3000 u-law now complete the whole of Phase 3 -- far-end J decoded
   32/32, S detected in both directions, J -> J', Phase 4 TRN -- and both
   sides reach MP, where MP never passes CRC and the locked hypothesis
   wanders (preamble 16-17/18 against 18/18 at 2400).  3429 still fails
   earlier, in Phase 3, with the answerer never detecting the caller's S;
   it is also the one rate whose RX shaper table is shared between both
   carriers, and the only rate where both carriers are the same frequency.

   Diagnostics for this work, all opt-in and all caching their getenv:
   `V34_P3TRN_SYM_DUMP` / `V34_P4TRN_SYM_DUMP` dump equalized training
   symbols (comparing their 4th-power coherence is what separates "the
   constellation is smeared" from "the hypothesis search picked the wrong
   scrambler"), `V34_TRACE` enables the `[EQ]`/`[CMA]`/`[V34 RX]` traces,
   and `V34_DUPLEX_LOG` gives the harness per-role spandsp flow logs.
6. Add retrain, rate renegotiation and cleardown tests.
7. Require a foreign-modem LAPM frame with valid FCS before calling a profile
   hardware-qualified.

Passing an in-process harness is necessary but not hardware interoperability.
All protocol changes must cite the applicable V.34 section in code comments.
