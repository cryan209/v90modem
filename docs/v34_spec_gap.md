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
| 10.1.3.1-10.1.3.9 | B1, E, J/J-prime, optional MD, PP, S, TRN and MP | Shared generators and receivers exist; `v34_duplex_test` provides a waveform-only two-instance bearer. E is the normative single 20-bit sequence, E detection is gated by a complete MP-prime, Phase-4 CMA is limited to the first 512T of TRN and frozen for MP, and B1 is received as the complete known data frame before payload is unclamped. | At 2400/9600 PCMU both directions complete startup, synchronize payload and recover over 16,000 bits without error. PCMA still stalls in Phase 3 and follows before this target enters the default suite. |
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
   payload in both directions after training.  `make v34-duplex-test` is kept
   outside the green default suite because PCMA still stalls in Phase 3.
   At 2400/9600 PCMU both directions complete INFO1, S/S-bar, PP, TRN, J,
   MP/MP-prime, E and B1 from waveform evidence. The known B1 frame now
   calibrates arbitrary phase, gain and conjugation before reset-state replay;
   both receivers then synchronize and recover over 16,000 payload bits with
   zero errors. Enforcing the configured 9600-bit/s INFO1/MP ceiling removed
   accidental 21600-bit/s negotiation, while §10.1.3 modulation-factor
   compensation removed high-rate clipping. Phase-4 CMA is bounded to the
   first 512T and frozen during framed signalling as required by §11.4. The
   harness previously exposed and fixed four
   sequencing defects: the caller no longer abandons the control channel
   before INFO1a; J detection is published to the caller transmitter; the
   answerer becomes silent and conditions on caller PP/TRN/J after S/S-bar;
   and 11.4.1.1 now sends caller J-prime followed by at least 512T of TRN,
   rather than sending MP immediately.
5. Run `{2400,2743,2800,3000,3200,3429}` with legal carrier, rate and
   asymmetric-rate combinations; assert external sample accounting.
6. Add retrain, rate renegotiation and cleardown tests.
7. Require a foreign-modem LAPM frame with valid FCS before calling a profile
   hardware-qualified.

Passing an in-process harness is necessary but not hardware interoperability.
All protocol changes must cite the applicable V.34 section in code comments.
