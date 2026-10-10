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
native serial interface (since shown to be a Courier DTE overrun in the test, not a modem fault; see
`docs/x2_implementation.md`, 10 October 2026), so that run is not a complete bidirectional pass.
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

## Native DAC wrap located, 2026-10-06

`X2_HOST_CAPTURE_CODEC=1` with `--capture-native` enables a read-only
pre-resampler sample/clock capture in `tools/x2_courier_capture.py`.
It preserves generation offsets across DSP resets in `native-codec.json`
and the signed DAC samples in `native-codec.s16`. The fresh failing 9600
call is `artifacts/x2-codec-path-20261006-9600`.

The independent native/B1 correlation puts B1 at sample 254360 in the
concatenated codec stream. Its clock is 9600 Hz throughout this interval.
The held-out native-symbol inverse fit remains 0.107158 with 21 taps and
0.108731 with 65, so the damage precedes socket FIFO handling, line padding
and codec-to-line resampling.

A forward pulse-shaper fit is more decisive. Insert the native symbols every
three codec samples, mix at 1920 Hz, and fit a 61-tap complex FIR (122 real
coefficients) to 6000 DAC samples; grade the next 5900. The large innovations
are approximately +/-32768 DAC units: for example, sample 6482 relative to
B1 is +15864 against a prediction of -16252, and sample 6611 is -16092
against +16318. The AC01 output gain is 2 (-6 dB), so this is a 65536-unit
signed-word wrap before that gain, not clipping at the analogue output.

As a DIAGNOSTIC using the known native symbols, iteratively fit on the
training portion and add integer multiples of 32768 to samples selected by
the fitted waveform. Twelve corrections over the whole window reduce
held-out waveform MSE from 1106309.7 to 2.9255 (RMS 1.7104 DAC units,
maximum residual 5.3046). This is strong evidence of signed output-word
wrapping, rather than a receive timing/echo/equalizer problem in this window.
It is not a production receiver repair: it uses the known transmitted points.

The native serial ISR reads the transmit-ring word and ORs control bits before
writing DXR at 818f. `C5xCore::codec_transmit` masks the low two control bits
and applies the -6 dB gain; its conversion cannot restore a word that already
wrapped. The precise producing instruction or parameter error upstream of
that word remains to be traced. Do not replace an emulator instruction's
wrapping with saturation without checking its architectural semantics; nor
assume this proves the original hardware would emit the same damaged stream.

## Output gain/store cause established, 2026-10-06

Further read-only instruction traces follow the native word through its
producer, runtime gain and serial handoff. The pulse output written by B461
stays within signed 16-bit range (observed -18799..20202). Gain cell 0392 is
31999, loaded from FFF0 at 8E5C. At 80E1, the accumulator equals exactly
`2 * pulse * gain + 65536` for every nonzero pulse in the captured window;
23 of 20001 accumulators exceed the signed range of the following shifted
output store. The data-ring write at 80E7 is `SACH *+,1` in a delayed branch.

A temporary core copy adds ONLY trace provenance to the capture header.
Its instruction behavior is unchanged; the patch is preserved with
`artifacts/x2-pm-origin-20261006-9600`. It records PM=1, last explicitly
set by ADBC (`SPM #1`) in the native receiver routine, or preserved by the
interrupt context restore. This is not a spurious extra multiply invented by
the emulator: TI SPRU056D, SPM instruction (6-252), defines PM=01 as a
one-bit left shift of PREG. SACH (6-221) explicitly loses high bits during
its left shift and does not saturate. Do not change those semantics as a fix.

The headroom bound before the AC01's -6 dB output gain is therefore
`abs(pulse) < 32768**2 / (2*31999)`, approximately 16778, while the native
pulse output reaches 20202. That produces the measured signed wraps. The
codec attenuation happens too late to prevent them. Its resampler and socket
FIFO carry the already damaged samples faithfully.

For a causal experiment, `X2_HOST_DIAGNOSTIC_TX_GAIN=15999` changes native
runtime cells 0392 and FFF0 after codec sample 180000, before B1, while leaving
firmware instructions and our receiver untouched. It requires
`X2_HOST_CAPTURE_CODEC=1` and `--capture-native`. Capture JSON records the
old/new values, and the harness result records `native_runtime_controls`.
These are deliberately MODIFIED runtime-level experiments, not unmodified
native interop qualification and not a production receiver repair.

The initial 130-million-instruction diagnostic runs end before the entire
3500-byte source can be transmitted; do not count their incomplete-source
check as a decode error or a full-transfer pass. Full-length repeats are under
`artifacts/x2-headroom-full-20261006-*`. The 7200, 9600 and 12000 repeats pass all seven
checks, with all 3500 bytes and the host message exact at each rate. B1 fit
is 100% in all three; at 7200 it improves from 94.2% without relaxing
acquisition thresholds. At 12000 the B1 lattice distance falls from 0.119
to 0.002. This independently
confirms the transmit headroom cause. The native profile's chosen transmit
level should be qualified against hardware/protocol power control before
making this diagnostic intervention a default or calling the emulator's
unmodified low-rate waveform an ideal receiver test.


The 14400 diagnostic repeat also passes all seven checks, carrying all 3500
upstream bytes and the short downstream message through the real PTY.
B1 chooses 64-state expanded shaping, fits 100%, and grades the following
256 symbols at lattice distance 0.003. Its working parameters are b=36,
p=16, w=0, j=7, k=24 at 3200 baud. Capture:
`artifacts/x2-headroom-full-20261006-14400`.

This does not resolve the later native MP/data discontinuity: the 14400 call
also eventually asks for resynchronization. Initial payload qualification and
long-term rate-transition handling remain separate issues.

A further source review finds that Courier B06E unconditionally calls B0F6
(the mapper normalization) before its bit-13-selected nonlinear polynomial
at B073..B08B. The optional transform must not be enabled in our MP until the
receiver implements the matching inverse: merely changing the flag would
invalidate its linear B1 template and lattice validation. V.34 (10/96) 9.7
defines the energy-normalized transform; 10.1.3's note requires modulation
power compensation through B1 and data. Whether the native profile's
normalization/power configuration accounts for its missing output headroom
remains open.


The 19200 diagnostic repeat passes all seven checks as well, with the whole
3500-byte source exact and the host message received by the Courier.
Its B1 fit is 100%; the independent following-symbol check measures lattice
distance 0.009, and working parameters are b=48, p=16, w=0, j=7, k=28.
`artifacts/x2-headroom-full-20261006-19200` preserves this run.


The 24000 diagnostic repeat passes all seven checks, including the entire
3500-byte upstream source and short host message. B1 fit remains 100%; the
following-symbol lattice distance is 0.028, with working parameters b=60,
p=16, w=0, j=7, k=24. Capture:
`artifacts/x2-headroom-full-20261006-24000`.

The six successful higher-rate runs are on the harness's default mu-law
bearer. They establish receive capability at 7200/9600/12000/14400/19200/24000
when native output headroom is restored, without changing our receiver or
its thresholds. They do not qualify higher-rate A-law or unmodified native
transmit power, and do not resolve the later MP/data discontinuity. Native
mapper captures now also include DP=7 normalization cells 03E7 and 03F5 for
further power-control analysis; direct-address operands must be resolved
against their live DP rather than inferred from the mapper's other cells.

## The 4800 cap removed: the peer's transmit level was mis-set, 10 October 2026

The output wrap above is real Courier behaviour, but it is driven by a
configuration value the emulated unit should not have. Traced end to end:

- DSP host command **tag 0x1A** (handler 822A, dispatch base 83E9) copies the
  host's word into the transmit gain cells 0392 and FFF0.
- The supervisor sends it from C9B24..C9B39 (segment C800). The gain is
  `table[index] - 0x300` (0x300 subtracted while [014B] bit 0 is clear). The
  table at physical C9B50 runs from 0x7FFF (0 dB) down in 1 dB steps; index 8
  is 13014.
- The index is `[072B]` if [0D74] bit 2 is set, `[072A]` if [04F2] is
  nonzero, and otherwise `[04B5]`, which is **S39** (S-register block based at
  048E; S56 is 04C6). In these calls [0D74]=0 and [04F2]=0, so S39 decides.
  The alternatives are local AT/NVRAM options as well. Nothing the far end
  sends reaches this level: x2 has no INFO1a power-reduction exchange.
- At boot S39 is loaded with its factory default **8** (from [0CD8]), then
  overwritten with **0** when the stored NVRAM profile is restored (loop
  88065..). The AT parser for this register (CA2DB) only accepts 1..29. This
  is the board whose NVRAM was never given `AT&F1, AT+SF, AT&W`
  (courier-emu `docs/idsdl-extended-registers.md`). Its DTMF levels read zero
  for the same reason.

S39 scales the Courier's whole call, not only data. At S39=8 its Phase 3 TRN
arrives about 8.3 dB lower (RMS 2342 against 6121) and the data gain is 12246
instead of 31999, so the shifted output word no longer wraps.

Qualification against **committed** courier-emu (dbcb0a0) with an unmodified
native runtime: `--courier-at S39=8`, a 3500-byte Courier source, the engine
PTY, and a short downstream message. Every rate passes all seven checks, with
100% B1 fit:

| upstream | result | evidence |
|---|---|---|
| 7200, 9600, 12000, 14400 | all 70 lines exact, host message complete | `artifacts/x2-s39q-20261010-{7200,9600,12000,14400}` |
| 19200, 24000 | same | `artifacts/x2-s39q-20261010-{19200,24000}` |
| 26400, 28800 offered | Courier selects 24000 (its MP mask 03FE tops out at N=10); passes | `artifacts/x2-s39q-20261010-{26400,28800}` |
| default build, no environment | offers 33600, Courier selects 24000, passes | `artifacts/x2-final-20261010-default` |
| default build, unmodified NVRAM (S39=0) | 24000 selected, B1 100%, **11 of 70 lines intact** | `artifacts/x2-final-20261010-nvram` |

The engine therefore offers every V.34 rate by default, and the peer's W2
decides (Draft 0.33 section 20). `ME_X2_UPSTREAM_MAX_RATE` remains as a cap.
`tools/probe_x2_host_courier.py` configures the Courier at its factory S39=8;
pass `--courier-at ''` for the raw NVRAM. A real peer with S39=0 would wrap
the same way. The server cannot correct that, and 4800 is the only rate it
carries cleanly.

Two rig hazards found on the way:

- An uncommitted change to courier-emu `native/c5x_core.cpp` (block-repeat
  redirection moved into `ROPCODE`, XC condition latched at a delay slot) is
  compiled into `.build/libcourier_c5x.dylib` whenever the tree builds. With
  it, every 9600 call stalls after RECORD_TX (no upstream E, and the Courier
  holds a constant-RMS signal). This happens with our 6 October engine too, and
  committed courier-emu does not do it. Qualify against a clean courier-emu
  worktree.
- X2 started V.42 detection at the accepted MP, about 1.5 s before upstream
  B1. T400 then ran off our transmitter, and the fallback reported
  `CONNECT 32000` on calls whose upstream never acquired. As on V.90, T400 now
  waits for the first upstream bit (V.42 7.2.1.3, bounded at 10 s), and x2
  host CONNECT waits for DATA.
