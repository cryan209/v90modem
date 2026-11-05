# V.92 upstream: trellis diagnosis, streaming receiver and wired qualification

Investigation: **5 November 2026**. Related: the
[binary rate sweep](v92_quick_connect.md#receiver-verified-binary-upstream-sweep-5-november-2026).

**A concrete limitation is in our receiver, not a demonstrated 36000 bit/s
channel ceiling.** On the same captured live 37333 decoder input, the existing
frame-local decoder makes **248957 wrong bits out of 3447416**, while an offline
continuous-trellis prototype with about **6 ms decision lookahead makes zero**.
That original experiment was receiver-only evidence, not a live higher-rate
interoperability pass. The subsequent production integration and wired binary
qualification are recorded in section 5. Rate-selection defaults remain unchanged.

## 1. Establish the peer's actual bits and symbols

Fresh calls on wired vmhost used the same isolated server/peer configuration
as the rate sweep: PCMU, 48000 downstream, PCM upstream enabled, sinc
resamplers, EC bypass and transmit timing offset held at zero on the shared-clock
rig. Calls sent 131072 deterministic pseudorandom application bytes per direction;
the instrumented 36000 control passed at **33.31 kbit/s verified upload**. The
37333 probes remained incomplete at the **90 s payload deadline**.

An observational native-DSP wrapper records the input to
`V92Transmitter::process` and its output symbols, together with Phase-4
`generateSymbol` states. It neither changes input/output buffers nor forces
acceptance of signalling. Important details:

- `process` input bytes are **already GPA-scrambled bits**, not application
  bytes. Values include 0, 1, 254 and 255; the significant bit is `byte & 1`.
- Undo GPA, V.92 §6.3 / V.34 equation 7-2, using the transmitted bits delayed
  by **5 and 23**. The initial state is zero at B1u (§8.7.1); the recovered
  initial B1u bits are all ones.
- The transmitter hook includes B1u. Select state-100 output records; the
  enclosing `generateSymbol` hook also observes B1u and must not be counted
  as another copy of those transmitted samples.
- Our bit dump starts partway through B1u: source offsets are **972 bits at
  36000** and **1008 at 37333**, both 18 data frames. High-entropy anchors
  establish alignment; repetitive idle flags are not sufficient.

| rate | live received bits compared | incorrect pre-LAPM source bits | ideal native-symbol decode |
|---|---:|---:|---|
| 36000 | 1314090 | **0** | 1326780 bits, **0 errors / 0 rejected frames** |
| 37333 | 3447640 | **249062 (7.224%)** | 3452232 bits, **0 errors / 0 rejected frames** |

The 37333 row repeats exactly in the subsequent input-frame-capture call.
These bit errors are **after trellis/modulus decoding and GPA descrambling,
before LAPM**, not the uncoded symbol-error rate. LAPM keeps delivered application
prefixes correct through rejection/retransmission; it cannot maintain useful
goodput with this many damaged frames.

### CPd and ideal waveform checks

The peer's decoded CPd agrees with ours on the selected drn, all constellation
points and all twelve moduli. There are **28 positive levels at 36000** and
**36 at 37333**. Moduli are LC at positions 0/1/2 of each four-symbol group,
LC/2 at position 3; their product satisfies §6.4.1's `product(Mi) >= 2^K`.
The peer has K=54 / 56, respectively, and trellis selection 0. Its gain field
is 90 (4G, Q0.16), LP1=8, LZ1=0, LZ2=4, LP2=0, and an identity prefilter
with z2(0)=32767/32768. The eight Q1.14 feedback coefficients are
`2819 5710 821 1808 -868 1870 261 360`.

Our §6.3/§6.4 reference encoder, with the native peer's permissible class-member
choice, reproduces its native transmitted samples with **0.577 native PCM-unit
RMS error and at most 1.001 units absolute error**, at both rates. Its inverse
decodes the ideal native samples without a single source-bit error as tabulated
above. Thus there is no observed gross CPd, gain, GPA, modulus, precoder or
convolutional-mapping mismatch in these captures. This is not an exhaustive
conformance audit, nor a claim that every peer or modulation parameter works.

## 2. Pre-fix: the decoder discarded trellis alternatives too early

`v92_upstream_wave_decode_viterbi_frame()` in `v92_upstream_data.c`:

1. Starts each twelve-symbol frame with only
   `state->convolutional_state` marked valid.
2. Runs the sixteen-state trellis for **three four-symbol transitions**.
3. Chooses one survivor, commits its source bits and end state, and discards
   all other metrics before the next frame.

Before this fix the engine used this path even with the upstream precoder:
it strips the CPd filter coefficients for decoding the already-equalised
u(n) levels. The frame-local function remains available for acquisition and
standalone callers; locked equalised reception now uses the streaming path.

This is a short block decision with a hard initial-state commitment, **not a
continuous trellis decoder retaining the competing state metrics**. A wrong
short-block decision can poison the next frame's starting state. Modulus and
differential decoding plus GPA spread symbol-decision errors into many source
bits. A wrong result can still be inside the valid source alphabet, so the
upstream `rejected_frames` counter remains zero despite substantial BER.

V.92 §6.4.4 incorporates the V.34 convolutional encoder with 4T delays. It does
not prescribe this receiver's Viterbi implementation or guarantee every rate
on every path. The finding is an inadequate receive implementation/coding gain,
not evidence of a newly identified mandatory wire-format violation.

## 3. Controlled experiment on the actual live decoder input

An isolated diagnostic build dumps the twelve `double` samples immediately
before `decode_frame_with()` in `v92_upstream_b1_rx_feed_values()`. It changes
no samples, decoder state, RTP accounting or protocol decisions. Re-running the
existing decoder on that captured input reproduces **every live output bit**
over the compared interval, including all **249062** incorrect bits.

The alternative offline decoder uses the **same two nearest constellation
candidates**, the same parity/transition tables and the same captured input.
It retains all sixteen state metrics between transitions and performs delayed
traceback rather than resetting to one state after every data frame. Its initial
state comes from known B1u; expected payload bits are used only for grading.
There is **no oracle payload training or replacement equalisation** in this
comparison.

| retained-metric traceback | nominal decision lookahead | compared bits | current decoder errors | continuous prototype errors |
|---|---:|---:|---:|---:|
| 3 four-symbol groups | 1.5 ms | 3447584 | 249031 | 2761, including 56 undecodable positions |
| 6 groups | 3 ms | 3447528 | 249007 | 521 |
| **12 groups** | **6 ms** | **3447416** | **248957** | **0** |
| 24 groups | 12 ms | 3447192 | 248957 | 0 |
| 48 groups | 24 ms | 3446744 | 248957 | 0 |

Each row excludes the terminal source frames that do not have the stated
lookahead; the current decoder is graded over precisely that same interval.
For 12 groups this is **224 terminal bits excluded**, not a discarded bad
interior segment. Without that guard, the prototype has 130 erroneous or
undecodable positions, all within the last four frames. The capture also has
**487 additional input frames after the last live-decoded frame** during
controlled termination; these have no corresponding output bits and are not
part of the equal-length comparison.

Retaining state metrics already produces a large gain at three groups;
allowing six milliseconds of traceback removes all measured errors in the
fully-looked-ahead interval. This directly implicates the frame-local trellis
commitments, without needing to attribute the failure to a peer defect or an
unavoidable analogue-channel limit.

## 4. Why denser constellations exposed it

The minimum positive-point spacing falls from **307 to 258 CPd units**
(approximately **152 to 128 received DS0 units** at the measured LU_rx=1442).
A separate diagnostic oracle fit uses the same 63 feed-forward taps as the
live equaliser, with known transmitted u(n) as training targets and disjoint
held-out samples. Its held-out residuals are **23.2–23.6 DS0 at 36000** and
**22.0–22.6 DS0 at 37333**, close to the live diagnostic distances.

Despite the similar residual magnitude, hard slicing in three 3996-symbol
held-out windows makes **1 / 1 / 1** symbol errors at 36000 versus
**49 / 41 / 46** at 37333. The denser constellation needs the coding gain the
short frame-local decoder gives away. These oracle fits are supporting
waveform diagnostics only; they are **not** the input used for the continuous
versus production decoder comparison above, and do not establish an absolute
best possible equaliser or channel capacity.

All three scored instrumented calls have **zero missing receive RTP packets,
nonunit sequence deltas or non-160 timestamp deltas**. The initial hook attempt
aborted because the privilege-dropped peer could not open a root-directory
output path; it is excluded. Using an owned writable temporary directory fixes
the instrumentation, and the subsequent 36000 payload control passes.

## 5. Production streaming receiver

`v92_upstream_trellis_init/feed()` now retain all **16 state metrics** across
frame boundaries, normalising every four-symbol transition. The bounded
survivor ring holds **15 groups**: the emitted twelve-symbol frame plus
**12 future groups / four frames / 48 symbols / 6 ms** of lookahead. This
is an implementation choice demonstrated by the experiment, not a delay
mandated by V.92 §6.4.4. No constellation, scrambler tap or rate margin changed.

`v92_upstream_b1_rx_feed_values()` seeds the decoder from the acquired B1u
end state. Latency-fill calls do not emit or reject source frames. Thereafter
modulus/differential/GPA state advances exactly once per emitted source frame.
A survivor outside §6.4.1's source alphabet is an erasure, not a valid frame:
advance the best available low-bit GPA/differential estimate and deliver K
idle-one bits to abort damaged HDLC without deleting bits from source time.
Invalid numerical input, which the trellis cannot consume, drops acquisition
instead of silently shifting later decisions onto the wrong input windows.
Profiles with filter coefficients retain the existing direct inverse.

The engine uses each decision's **earlier input-frame index**, not the current
frame, for trellis-aided equaliser rows. `decision_back()` includes both the
48-symbol delay and newer outputs already produced in the equaliser batch.
`refit_live()` marks the replay/live boundary without resetting pending
survivors; frames whose inputs overlap replay-only windows are excluded.
RTP/audio sample counts and buffering are unchanged. There is no synthetic
tail flush: idle symbols actually received from the peer supply lookahead.

### Deterministic regressions

The existing fast-tier `v92_b1u_lock_test` now covers:

- All **19 upstream drn values**, mixed-radix differential folding, exact delayed
  GPA/differential/trellis state, survivor-ring wrap and per-call reset.
- Existing B1u acquisition, class-member, noise, offset and no-B1u rejection
  cases, transmitting real trailing frames to drain decoder/byte-packer latency.
- Source-alphabet erasures followed by recovery; K=36/54/56 receiver tests
  require exact output bit counts, including non-byte-aligned residues.
- A retained **5000-frame live 37333** noisy-input fixture, portable Q20 samples
  and packed source bits under `reference-data/v92-upstream-trellis/`.
  The old frame-local decoder made **13637 errors / 279776 guarded bits**;
  streaming reception must emit **279776 bits with zero errors** for input
  chunks **1/37/160/320**, retaining exactly four terminal frames.
- Correct delayed refit-window indices, replay/live overlap exclusion and
  fail-closed numerical-input handling.

All **13 V.92 fast-tier cases** and both `vpcm_loopback_test` fast-tier
invocations pass on the isolated Linux build. The new B1u suite also passes
a standalone macOS build and Linux AddressSanitizer/UBSan with leak detection.
The fixture consumer has no dependency on expiring investigation artifacts.

### Wired slmodemd qualification

Fourteen bounded, sequential wired calls used the ordinary, unchanged
`slmodemd_map`, PCMU, **48000 downstream**, the same precoder/margin settings,
and CONNECT-gated deterministic binary payload with mutual receiver SHA-256
acknowledgements. Only the server binary changes in the 37333 new/old/new
comparison. Tests reuse role-specific payload prefixes; these are not
independent-seed or independent-noise trials.

| receiver / upstream bit/s | calls | payload requested per direction per call | verified upload goodput | result |
|---|---:|---|---|---|
| new / 36000 | 3 | 2 × 262144; 1 × 1048576 | **33.290–33.416 kbit/s** | all pass; long call unforced |
| new / 37333 | 4 | 2 × 262144; 2 × 1048576 | shorts **34.013 / 32.516**, longs **33.983 / 33.983 kbit/s** | all pass |
| new / 38666 | 2 | 262144; 1048576 | **34.281 / 34.344 kbit/s** | both pass |
| new / 40000 | 1 | 262144 | **30.141 kbit/s** | pass, 6 source-alphabet erasures |
| new / 41333 | 1 | 262144 | **30.030 kbit/s** | pass |
| old / 37333, between the two new short calls | 1 | 262144 | **149160 correct upload bytes / ~180 s, 6.626 kbit/s** | deadline, reproduces collapse |
| new / 42666 | 2 | 262144 | no DATA / no payload | B1u acquisition fails; first call transport-contaminated, repeat clean |

The **11 passing calls verify 6029312 bytes (5.75 MiB) in each direction**,
with both complete hashes acknowledged. The 37333 long receive times are
**246.849 / 246.850 s**; the 38666 long receive time is **244.252 s**. The
unforced selector still chooses **36000**, transferring 1 MiB each way with
**251.032 s** verified upload. Thus the fix removes the old higher-rate
collapse in real end-to-end traffic; it is not merely a replay improvement.
It does not make nominal 40000/41333 faster than 36000 on this path.

All passing calls and the old negative control have **zero missing RTP
packets, nonunit sequence deltas or non-160 timestamp deltas**, hold the
requested upstream and 48000 downstream rate, and have **no post-DATA
recovery/fallback**. Their initial startup retrain remains. Delivered hashes
do not measure pre-LAPM BER: LAPM may hide errors, and zero source-alphabet
erasures still do not establish zero BER.

The first 42666 attempt lost **16 incoming RTP packets in 14 discontinuities**
during startup and is excluded from rate-limit inference. A fresh repeat has
**zero RTP discontinuities** but still does not lock B1u: its reference fit's
best correlation is **0.188**, below the 0.5 acquisition threshold. No source
frame passes the initial ones test. This is **before the continuous data
receiver is engaged**, not another measured DATA-throughput collapse. The
first contaminated call's correlation was 0.173; do not attribute either
failure to the peer or a specification/channel ceiling without native taps.

### Remaining boundaries and build provenance

The direct native-bit/soft-input diagnosis remains at 37333; the higher-rate
binary passes are application-level qualification on this single peer/path,
not an exhaustive wire-format audit. 44000–48000 upstream remain unqualified;
untested constellations must not be assumed to fail monotonically. Investigate
42666 B1u/reference acquisition with own-transmit and native peer taps before
changing rate margins. Hardware V.92, profile-reuse Quick Connect, MOH and
live §9.8/§9.11 renegotiation gain no new qualification here.

The new server is an isolated Linux build from `1437f336` plus this receiver
change, reusing unchanged vendored libraries from the prior isolated build;
no pending Mac SpanDSP edits or system replacements were included. Qualified
server SHA-256:
`1179e176364a7a119c5497616b8fd80e295b46379c4d686f48b53d551c015c37`.
The old control remains
`bcb49335cf86e2e57dfa24005f2b15f8730ad7cac506a4cadd6a1793ec87179e`;
ordinary peer SHA-256 is
`f5914c79bc2d9c20084138ea7a0e2dfe222259ab828231ad1a23be956440a661`.
Neither ordinary rig binary was replaced. All owned live processes were
stopped after the final repeat.

Disposable evidence: `artifacts/v92-upstream-trellis-live/`, with individual
server/client archives, independent `grade_results.py` / `grading-report.json`,
source-under-test snapshot, drivers and manifest. Host copies are under
`/root/v92-trellis-live/`; redundant CT captures were removed only after host
archival and independent local grading to recover disk space.

## Original diagnostic provenance and cleanup

Disposable local evidence: `artifacts/v92-upstream-investigation/`:
`check_capture.py`, `ideal_peer_check.c`, `fit_oracle.py`, `stream_check.c`,
`compare_trellis.py`,
`probe2-36000`, `probe2-37333`, and `probe3-37333`; the latter contains
`bit-report.json`, `trellis-comparison.json`, and the actual receiver input
frames. vmhost results are under `/root/v92-upstream-investigation/`.

The first two scored calls use the original server SHA-256
`bcb49335cf86e2e57dfa24005f2b15f8730ad7cac506a4cadd6a1793ec87179e`.
The input-dump-only server is a separately named `v92_rx_probe`, SHA-256
`7fa08c0a1d3d9b7bff89131f0456488f978e24e16e6445722ba08c4a019175d0`.
The observational peer is `slm_up_probe`, SHA-256
`c4ae5ee2aa6d6599ac4b6834c0a265baf3a4ed9350f5d33ad42dcfaa5393e4a6`.
The ordinary server binary/source were restored after building the separate
probe; ordinary slmodemd_map was not replaced. No pending Mac SpanDSP edits
were included. Server, peer, endpoint and host-driver probe processes were
confirmed stopped after the final call.
