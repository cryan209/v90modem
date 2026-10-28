# MT9234 V.90 upstream gradient correction, 28 October 2026

The MT9234ZBA-USB (Agere OCM V.92 firmware 1.02e, native macOS TI USB
bridge) negotiated V.90 with our server but lost upstream decoding after
about six seconds, requested resynchronization and fell back to V.34.
The hardware 7900 control held 46666 down / 12000 up with no retrains.

## Cause and correction

The T/3 FSE computes `z = sum(w*x)` in the input carrier coordinates.
Carrier recovery produces `y = exp(-i*phase)*z` before slicing. Its decision
error `e = target-y` is therefore in the derotated coordinates. The old
NLMS update multiplied that error directly by `conj(x)`. The correct update
is proportional to `exp(i*phase)*e*conj(x)`: return the error to the input
coordinates first. Use the phase that derotated **this** symbol, captured
before `v34_carrier_update()` advances the phase.

`v34_carrier_error_to_input()` supplies that inverse rotation. The T/3
float and fixed-point NLMS branches share the corrected error. The blind
CMA recovery gradient needs the same inverse rotation on `y`: its radial
cost is rotation invariant, but its tap gradient still multiplies raw input.
No DSP thresholds, modulation constants or sample accounting changed.
V.90 8.5.2 and 9.4.2.5 describe the upstream training/data signals; the
coordinate transformation here is the chain rule, not a new wire protocol.

## Offline evidence

The retained fixture is the first 31 seconds of paired, byte-unchanged
PCMU RX/TX taps from `/root/mt9234-20261028-v90-264/` on vmhost CT117.
That endpoint used the `/root/k56j16` build, a 26400 upstream ceiling and
40000 downstream. The cut ends before its first resynchronization request
(TRACE +31183 ms), so later fallback traffic is excluded.

Replay uses 3200 baud / 26400 bps, E handover at 24.321 s, 14 s history,
and the existing line echo canceller fitted over 12..15 s of the paired TX
reference. CP-primed and premature forced-E replays do not reproduce the
live handover; align to the live E event before interpreting acquisition.
The 4-point training error matches live (31.6 dB). This modem's reset-state
B1 template does not fit; the existing 4-point acquisition path is used.

| Receiver | CRC-valid frames | Damaged segments | Aborts | Final symbol error |
|---|---:|---:|---:|---:|
| Original, Mac negative control | 6 | 30 | 12 | 0.297 |
| Corrected, Mac | 6 | 3 | 0 | 0.105 |
| Original, Linux snapshot | 6 | 30 | 13 | about 0.297 |
| Equalizer frozen, Linux diagnostic | 6 | 3 | 1 | about 0.111 |
| Corrected, Linux | 6 | 3 | 0 | about 0.105 |

Acquisition damage remains in the fixture: this is not a zero-error recording.
The regression requires the real CRC-valid frames, bounded damage and a final
symbol error under 0.15; it fails on the original receiver. The separate
finite-difference test checks decision-directed and radial-cost gradients over
65 carrier phases and verifies the decision-directed update decreases loss.
The freeze probe exists only in the isolated replay, not production code.

## Physical confirmation

Both calls used a protected executable name, separate test account and
SIP/RTP ports on wired CT117, PCMU, server upstream ceiling 26400, no PBX
configuration changes. Earlier attempts were sometimes interrupted by other
K56flex test cleanup; the isolated corrected processes were not interrupted.

| Corrected call | Down / up | Hold requested | Final modem diagnostics |
|---|---|---|---|
| Gradient-1, decision-directed correction | 40000 / 26400 | 60 s | V90, LAPM/V42bis; zero TX/RX frame errors, retrains and renegotiations |
| Gradient-2, shared DD/CMA helper | 46666 / 26400 | 60 s | V90, LAPM/V42bis; one TX frame error, zero RX errors; zero retrains and renegotiations |

Gradient-2 delivered the complete known **1920-byte upstream payload
byte-for-byte** to the answerer. Both calls received numbered downstream
text; exact downstream bodies are checked against the AT transcript. Clean
hangup returned OK and termination cause 0. This establishes bounded physical
interoperability, not a long soak or V.92/fixed-point hardware qualification.
The second call's recovered frame error must not be reported as error-free.

Exact DTE transcripts are in the modem-dsp-emu workspace at
`artifacts/mt9234zba-calls-20261028/emu-gradient-{1,2}/`; full answerer taps
and logs are on CT117 in `/root/mt9234-debug-20261028/live-gradient-{1,2}/`.
The isolated source/build snapshot is `/root/mt9234-debug-20261028/`.

## Validation

- `v34_carrier_gradient_test`: passes on Mac and Linux.
- `tools/mt9234_v90_gradient_test.py`: passes on Mac; same Linux replay metrics.
- Fast suite on the current Mac working tree: **629 passed, 1 failed**, 90 s.
  The failure is the existing A-law plain-V.34 noisy-rate expectation:
  `V34_DUPLEX_EXPECT_RATES=31200,28800 ... v34_duplex_test 3429 33600 alaw`
  actually negotiates 31200/31200 and transfers a million bits with zero
  errors. Replacing only the receiver object with the original (gradient
  correction removed) reproduces identical rates and failure. No expectation
  was changed to hide it. The new capture row was added after this run and
  passed separately. Logs are retained in modem-dsp-emu
  `artifacts/mt9234-debug/` and local `test-logs/0116.log`.
