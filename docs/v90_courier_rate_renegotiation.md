# Courier V.90 rate renegotiation: stop treating protocol changes as a bad line

## Fault and repair

The software Courier 403 asks to increase downstream rate after stable DATA.
The original digital-side responder was disabled by default. It ignored the
peer's S/S-bar, then interpreted the peer's eventual full-retrain waveform as
an upstream decoding failure and reduced the upstream offer from 31200 to
28800 bit/s. The rate reduction was reacting to a mishandled protocol change,
not establishing that the preceding data channel could not sustain its rate.

The repair does **not** remove genuine channel-rate control, change DSP
thresholds, pin a desired CONNECT rate, or disable recovery. It fixes these
protocol seams:

1. **Respond by default.** V.90 (09/98) section 9.6.1.2 is mandatory regardless
   of whether we initiate renegotiations. `ME_V90_RENEG_RESPOND=0` remains a
   diagnostic negative control. Plain V.34's default remains enabled.
2. **Fresh CP receive epoch in ME_DATA.** Section 9.6 preserves the existing
   data link. Reset the batch generation and synchronous CP framer, receive
   CP rather than startup CPt, feed new audio, and allow worker acceptance
   only during an active renegotiation (or normal startup). Ordinary DATA
   cannot consume a late startup worker result.
3. **Retire the old acquired DATA receiver.** Responding used startup CP
   conditioning, which preserved the already-acquired T/3 DATA decoder. It
   decoded new CP as old DATA, then reported discontinuity and lowered the
   rate. Use the existing `v34_v90_force_reneg_cp_rx()` conditioning, as the
   digital-initiated S handoff already does. Acquire a fresh E/B1 afterwards.
4. **Correct Rd/Rt magnitudes.** Section 8.6.4 specifies the highest-power
   codeword in each interval's preceding DATA constellation for Rd, not
   U_INFO (which is Ri). Snapshot the six magnitudes before accepting new CP.
   Rt uses the retained training constellation. Sign pattern and 384T/24T
   lengths are unchanged.
5. **Accept new data parameters.** Section 9.6 permits a new rate and
   constellation. Invalidate the old data mapper's acceptance latch after
   snapshotting Rd; otherwise the new CP is rejected as a mismatching repeat
   of the previous exchange. Retain CPt's training mapper.
6. **Initialize every TRN2d.** Section 8.6.5 requires zeroed scrambler,
   differential and shaping memories at its start, including renegotiations.
   Keep the existing TRN2d duration and DSP constants.
7. **Observe actual E on the qualified batch hypothesis.** Section 9.4.1.4,
   imported by 9.6, permits CP' **or E** to finish the exchange. The retained
   waveform contains E immediately after the last CRC-valid CP; the native
   receiver trace independently shows reception of a CRC-valid MP'. The
   strict batch decoder now reports all 20 E ones only when they immediately
   follow its own complete CRC-valid data CP. It does not qualify the separate
   streaming decoder. CPt, CPs/SCR, a 17-one CP preamble, incomplete E and
   unqualified input cannot trigger this path. Carrier still requires the
   existing complete B1 fit and out-of-sample validation.

The direct decoder also gives a CRC-valid CP' priority over plain CP across
all timing hypotheses, rather than discarding it for a lower phase-error CP.
This fixes selection semantics, but was not sufficient to fix this capture:
independent decoding found actual E, not a separately recovered CP'.

## Evidence

The first complete corrected 5-ms run used 600M guest instructions,
wall-clock pacing, normal full source injection with active-low CTS, and
38400 payload bytes in each direction. **Both byte streams were exact.**
The peer's short renegotiation completed; its new downstream CP has DRN=21,
which is (20+21)*8000/6 = **54666.67 bit/s**, replacing DRN=19 (**52000**).
Upstream stayed **31200 bit/s**. New upstream B1 fit was **100.0%**. There
was **no full retrain** and no spurious 31200-to-28800 reduction.

Provenance in sibling courier-emu:
`artifacts/analog-v90-reneg-fix/fresh-rx/`.
Pinned engine SHA-256:
`ec7f7f8e9777391dde8f4f8a82b0266df8a61773755270519339955beece5fb9`.

A previous intermediate build that decoded E but kept startup's old DATA
receiver completed downstream renegotiation, immediately declared upstream
discontinuity, reduced the offer to 28800 and opened more renegotiations.
That localized the remaining rate-workaround loop to receiver lifecycle,
not to a need for a lower downstream speed.

The corrected **20-ms** run also passed the full 38400-byte exact duplex
check over 600M instructions, with one short renegotiation, a new 100.0% B1
fit, zero full retrains and no rate-reduction offers. A matched **5-ms
responder-disabled** run with the same executable completed two full retrains
and logged three upstream rate-reduction offers. It still preserved all
38400 bytes each way through CTS and recovery. All arms ended online with
process exit 0. A fresh **x2** 180M-instruction exact-duplex regression passed.

Full checks and environments: courier-emu
`artifacts/analog-v90-reneg-fix/results.json`.
These controls isolate the protocol response; they do not disable genuine
receiver-loss recovery or demonstrate indefinite stability.

## Regression coverage

`v90_reneg_silence_test` checks both G.711 laws, independently expected
384T/24T codewords across six distinct constellations, zero-initialized first
TRN2d frame after nonzero old state, changed CP rate acceptance, missing-E
recovery deadlines and CPs silence/Rt behavior.

`v90_reneg_engine_test` exercises the actual private engine/capture code:
ordinary DATA exclusion, fresh generation, CP-versus-CPt selection, DATA audio
capture, CP'/E candidate priority, old DATA receiver retirement, and E guard
negatives. It also demodulates the unchanged 2350-codeword fixture in
`reference-data/courier-v90-reneg/`: strict CP CRC, DRN=21, acknowledge=0,
then actual adjacent E. The fixture is indexed and is software evidence only.

Selected fast-list tests (both above plus `v90_b1_state_test`) pass. A full
suite or physical Courier qualification is not implied. Finite exact duplex
traffic and a completed short renegotiation are not an indefinite soak.
