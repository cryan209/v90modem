# x2 upstream V.34 review, 6 October 2026

The host's 4800 bit/s default is an explicit diagnostic policy in
`me_x2_start_locked()`, not a measured channel ceiling. The Courier initial
record 0344/03FE/0000/0500 advertises higher upstream rates. The environment
variable ME_X2_UPSTREAM_MAX_RATE changes the local mask; the PCM downstream
mask remains separate.

Reviewed against ITU-T V.34 (10/1996), local
`ITU Docs/T-REC-V.34-199610-S!!PDF-E-1.pdf`:

- Section 8.1 / Table 7: at 3200 baud J=7 and P=16. A mapping frame has
  eight 2D symbols, a data frame lasts 40 ms, a superframe lasts 280 ms.
- Section 8.2: at 4800 bit/s N=192 bits per data frame, b=12 and r=16.
  All mapping frames are high frames; no frame switching is necessary.
- Section 9.3.2: b=12 has K=0 and four groups of I1/I2/I3. There are no
  shell bits at this profile. Failure here cannot be blamed on shell unranking.
- Sections 9.6.3 / 9.6.3.2: inversion epoch and convolutional state are
  separate from nearest-point correctness. V0 occurs at each half-data-frame
  boundary; the convolutional encoder has one 4D interval of inherent delay.
- Section 10.1.3.1: B1 is one data frame with the final-superframe inversion
  pattern and initialized scrambler, trellis, differential and precoder state.
- Sections 11.4.1.1.4 and 11.4.1.2.4: data begins a new superframe after B1.
  A good fit to B1 does not prove the decoder epoch or subsequent user bits.
- Section 7 distinguishes GPC and GPA. This x2 implementation uses the
  original Courier's observed upstream convention; do not replace that
  empirically verified convention solely from the ordinary V.34 call role.

The T/3 upstream receiver already enables its carrier loop, timing loop and
0.02 decision-directed equalizer step by default. There is no evidence that
4800 is needed because tracking is absent. Its own distance to the chosen
lattice is insufficient to certify correctness; actual foreign source text
and independently captured transmitted points are the acceptance evidence.

## Fresh 9600 bit/s control

Command:

```
ME_X2_UPSTREAM_MAX_RATE=9600 ../courier-emu/.venv/bin/python \
  tools/probe_x2_host_courier.py --fast --fixed-native-rate --pty-source \
  --capture-native --host-message HOST-X2-0123456789-ABCDEFGHIJKLMNOPQRSTUVWXYZ \
  --message COURIER-X2-0123456789-ABCDEFGHIJKLMNOPQRSTUVWXYZ \
  --output artifacts/x2-v34-review-20261006-9600
```

The selected upstream rate is 9600. Native B1 acquires at 100% fit with
64-state trellis and expanded shaping. The Courier receives the complete
host message, but the engine's upstream PTY contains corrupted source text,
including an intact `DEFGHIJKLMNOPQRSTUVWXYZ` suffix. Early receiver grid
error is approximately 0.015..0.026 with zero shell-invalid frames; those
self-consistent metrics do not establish bit correctness.

Read-only native point comparison at b=24 rejects the initial alignment:
best 256-symbol MSE is 0.119984, above the tool's 0.05 qualification bound.
Do not use this failed alignment to claim a measured number of symbol errors.
The next useful measurement is a qualified native-point alignment through
B1 and the first data frames, then a comparison of transmitted points and
Viterbi decisions around each corrupt text burst. This separates front-end
errors from inverse mapping/epoch errors before changing any loop gains.

The diagnostic default remains 4800. Raising it without a complete native
payload test would advertise a rate that this receiver has not qualified.

## 4800 receive corrections and sample-rate decision

The follow-up native Courier investigation found two receiver defects, without
changing DSP constants, scrambler conventions or the G.711 bearer:

1. The T/3 B1 watcher parked the input Table-12 epoch back at the final data
   frame after consuming B1. That can accommodate an extended V.90 training
   stream, but original Courier x2 mapper execution verifies the single
   reset-state frame of V.34 10.1.3.1. x2 now advances naturally into the new
   superframe and enables the h=0 trellis parity constraint (9.6.3/Table 11).
   Trailing B1 bits remain suppressed through the 15-pair traceback delay.
2. After initial idle, the V.90 frame-phase heuristic treated busy 4800 data
   as a loss of synchronization and swept the epoch. At K=0 there is no shell
   bound, and a falling fraction of marks is ordinary user traffic. x2 now
   retains the epoch established by B1; content is not grounds to move it.

Read-only native point capture recovered the complete original source from
exact points. The waveform receiver had isolated pairs of errors despite
approximately 0.00035 MSE in most windows. Enabling the correctly phased
constraint recovers the previously corrupt short source. The first long run
then exposed the phase sweep: 13 complete lines before corruption even with
an open constellation. Preserving the epoch recovers all 70 lines / 3500
bytes from that recording at 17-, 80- and 160-sample receive block sizes.
A fresh native call also delivers all 3500 upstream bytes through the real
engine PTY. Its simultaneous 940-byte downstream burst is incomplete at the
native serial interface, so that run is not a complete bidirectional pass.
A second fresh call (`artifacts/x2-rx-improve-20261006-long-rx-fixed`)
passes all seven checks: the full 3500-byte upstream source and a short
downstream message both reach their real PTYs. The final output-clamp build
repeats this pass in `artifacts/x2-rx-improve-20261006-final`.
Evidence: `artifacts/x2-rx-improve-20261006-*`.

Receive sampling was checked rather than changed speculatively:

| Receiver | Internal equalizer input | Decision |
|---|---|---|
| x2 / V.90 V.34 upstream | Three complex samples/symbol, 9600 Hz at 3200 baud | Retained; verified by complete-engine foreign-payload replays. |
| Plain V.34 | Polyphase matched filter at half-symbol intervals | Retained; its separate acquisition path is not qualified by x2 results. |
| V.32 / V.32bis | V.17-derived matched filter and FSE at half-symbol intervals | Retained; existing duplex suite passes all rates/laws and echo/retrain/renegotiation cases. |

Three samples/symbol is not a normative requirement, and changing the two
working half-symbol receivers would introduce new timing and training
handoffs without fixing the measured errors above. Internal interpolation
operates on the receiver's copy of linear samples; wire codewords and the
8000-sample/s bearer accounting remain unchanged.

Validation: `v34_data_test` (402 exact mapper/trellis/V0 cases), recorded
`x2_b1_test` (17/160-sample acquisition and silence rejection), x2 mapper,
session and symmetric tests, and `v32bis_duplex_test` pass. The new
`tools/test_x2_host_replay.py <tap> --expected-file <source>` requires
the preserved native tap and source
and asserts complete foreign bytes, DATA, exact RX/TX sample counts, T/3
operation, B1 handoff and absence of content-triggered phase shifts.

Echo cancellation was inspected but not retuned on this evidence. V.32bis
already trains its canceller in the clause-6 quiet window and passes its
hybrid sweep. The engine's separate V.34 NLMS path still needs independent
qualification of transmit-reference alignment, per-call reset and adaptation
while the far end is active. Later native x2 MP/rate transitions and hardware
interop remain separate from the verified initial 4800 transfer.

## Higher-rate qualification, 2026-10-06

Fresh original Courier runs with `ME_X2_UPSTREAM_MAX_RATE` set to 7200,
9600 and 12000 use the same 3500-byte source (70 complete lines) and short
host message as the passing 4800 control. No fixed native `&U26&N39` clamp
is applied. Results are preserved under `artifacts/x2-higher-20261006-*`.

| Selected upstream rate | B1 / engine DATA | Foreign source through engine PTY | Host message at native DTE |
|---|---|---|---|
| 4800 control | Acquired; DATA | All 3500 bytes exact | Complete |
| 7200 | B1 acquisition fails; no engine CONNECT | No complete lines | Not queued: engine never CONNECTs |
| 9600 | 100% B1 fit; DATA | Only 6 of 70 lines intact; whole source fails | Complete |
| 12000 | B1 acquisition fails; no engine CONNECT | No complete lines | Not queued: engine never CONNECTs |

Read-only native captures contain 18, 24 and 30 bits per mapping frame,
respectively. Passing those exact points (Q9.7) to the 3200-baud expanded,
64-state, zero-precoder mapping decoder with GPA, starting at native point 5,
recovers all 3500 source bytes at all three rates. Independently generating
V.34 10.1.3.1's reset-state B1 with the same parameters matches the native
first 128 B1 symbols exactly at all three rates. Thus the native mapper and
our inverse mapper agree for this source; neither higher-rate B1 failure is
evidence that the native B1 convention differs.

At 9600 the waveform reports low decision-grid distance initially but has
isolated symbol disturbances and corrupt text well before the later MP/rate
transition. The existing native/waveform comparison tool rejects its first
256-symbol alignment (MSE 0.166878 > 0.05), so do not turn that rejected
alignment into a qualified symbol-error count. Distinguishing damage already
present in the PCM waveform from analytic-filter/equalizer damage requires a
held-out fit against native points and independently established sample
alignment. At 7200/12000, acquisition's coarse shortlist and receive front
end need examination despite the exact native/template match. The validated
default remains 4800; higher rates are diagnostic offers, not qualified ones.

## PCM versus native-point bound, 2026-10-06

The higher-rate failure logs need the FIRST acquisition attempt, not just
later retry failures. At 7200 the search finds the correct first=8413 but
rejects 94.2% fit against 95%. At 12000 it finds 97.6% fit at that same
boundary, then rejects held-out lattice distance 0.119 (0.01125 of power,
limit 0.002). Thus a coarse shortlist miss is not the measured explanation
for either original rejection.

Independent numpy-only analysis forms an FFT analytic signal directly from
the received PCM, interpolates to 9600 Hz, mixes at the 1920 Hz high carrier
and correlates against the exact native-matched B1. It finds B1 at PCM
sample 111140 in all three taps. A complex least-squares equalizer trained
on the first 2048 native symbols and graded on the next 2048 gives:

| Rate | 21-tap held-out MSE | 65-tap held-out MSE |
|---|---|---|
| 7200 | 0.136369 | 0.139571 |
| 9600 | 0.107829 | 0.109328 |
| 12000 | 0.332653 | 0.341256 |

At 9600, independently align the Courier's outgoing `analog-tx.wav` by
waveform values, not file lengths: it is 150880 samples ahead of the engine
RX tap, with normalized waveform correlation 0.999827 over the 1000-sample
B1 window. Decoding G.711 here uses quarter-scale linear units, hence the
fitted gain 0.127380 equals the bearer gain divided by four. Repeating the
native-symbol held-out fit on this PRE-G.711 linear audio gives 0.107162
(21 taps), 0.108860 (65 taps), and the same 16 residuals above unit squared
distance. G.711 quantization and our streaming analytic branch therefore
cannot explain these bursts by themselves. These are empirical bounds for
this captured window, not proof that no nonlinear/time-varying receiver
could improve it.

Correct the earlier inference: exact native mapper points prove the source
and inverse mapping, not an ideal emitted PCM waveform. The remaining
transmit-side boundary between native mapper execution and outgoing codec
samples needs inspection before retuning our receiver or weakening its
acquisition guards. The 9600 residual is already present before G.711;
its precise cause (pulse shaping, codec/resampler, or scheduling) is not yet
established. Sampling at T/3 instead of T/2 does not establish that boundary.
