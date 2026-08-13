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
| 7-9 | Directional scrambler, framing, shell mapping, differential/nonlinear/trellis encoding and precoding | Mapper/demapper and data mode exist; reset-state mapper/demapper tests are clean. Received MP encoder fields now configure TX while the locally sent MP configures RX; MP1 coefficients reach the TX precoder and MP0 preserves them as required by 10.1.3.9. | Test every advertised trellis/shaping combination.  The live modem currently requests 16-state trellis; resolve and test the RX quantizer TODO before requesting 32/64-state trellis from a peer. |
| 10.1.2.1-10.1.2.3 | Tone A/B and INFO0/INFO1 framing | Live V.90 work substantially hardened the shared Phase-2 state machine | Run plain-V.34 caller and answerer recovery cases, not only V.90 role inversions. |
| 10.1.2.3.4 | INFO1c reports measured per-rate carrier, pre-emphasis and projected rate | L1/L2 measurements and a per-rate evaluator exist | Plain V.34 previously emitted configured rates with pre-emphasis 6.  INFO1c must be built from the measurement for every enabled row and report -512 when frequency offset is unavailable. |
| 10.1.2.3.5 | INFO1a selects both directional rates and the call-to-answer carrier/pre-emphasis/rate | Parser and serializer exist | Selection must combine INFO1c, local L1/L2 results and both INFO0 asymmetry limits.  Configure TX and RX independently from the result. |
| 10.1.3.1-10.1.3.9 | B1, E, J/J-prime, optional MD, PP, S, TRN and MP | Shared generators and receivers exist; B1 duration/alignment has been corrected during V.90 work. `v34_duplex_test` now provides a waveform-only two-instance bearer. The plain caller now receives INFO1a before changing demodulators, both Phase-3 directions complete PP/TRN/J, and the answerer waits silently for the caller's J before Phase 4. | At 2400/9600 PCMU both receivers now accept CRC-valid MP and MP-prime, both detect E, both transmit B1 and both enter DATA without an E-timeout fallback. The post-B1 demapper does not recover even a 32-bit payload sync word, so data-mode slicing/trellis decoding is the next blocker. PCMA still stalls in Phase 3 and follows after the PCMU baseline. |
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
   outside the green default suite while it encodes the current open defect:
   at 2400/9600 PCMU, both Phase-3 directions now complete INFO1, S/S-bar,
   PP, TRN and J from waveform evidence and both sides reach Phase-4 TRN/MP.
   Phase 4 now completes bilaterally at 2400/9600 PCMU: both receivers accept
   CRC-valid MP and MP-prime, both detect E, transmit B1 and enter DATA without
   the former 500-baud forced transition.  The MP fix separates direct-mapped
   TRN's absolute-phase lock from MP's differential domain, and a stable
   three-frame CRC failure now rotates the slicer mode rather than pinning a
   wrong hypothesis forever.  MP-prime is transmitted in full before E as
   required by 11.4.1.1.3/11.4.1.2.4.  The remaining PCMU blocker is now the
   post-B1 datapump: neither direction finds the first 32 payload bits in over
   one million decoded bits.  PCMA still stalls in Phase 3.  The harness
   previously exposed and fixed four
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
