# V.92 full startup: digital and analogue modems

Reference: `ITU Docs/T-REC-V.92-200011-I!!PDF-E.pdf`, clauses 3, 6,
8.5–8.8 and 9.3–9.6, Tables 15–18, 20, 23, 27, 30 and 31. The local
Amendment 1, Amendment 2 and Corrigendum 1 were checked too. Amendment 1
retains Table 18's 276-symbol MD unit. This implements the full-training
procedure, not Quick Connect.

## Endpoint interfaces

The digital modem is the existing `v90_state_t` with V.92 Phase 3, V.92
mode and native CPu reception enabled. `v90_phase3_tx_codewords()` produces
the actual G.711 DS0 without a decode/re-encode step. Its receivers consume
the network ADC's codewords. The analogue modem is `v92a_t`, declared in
`v92_analogue_phase3.h`; it owns `v92a4_t` for final training.

Use `v92a_audio_t` (`v92_analogue_audio.h`) for the analogue audio interface:
**16 kHz signed 16-bit linear PCM in both directions**, two samples per
8,000-baud symbol. V.92 6.2 specifies the symbol clock, not an 8 kHz sound
card interface. The PCM calibration is four core linear units per audio
sample unit, providing about **12 dB of amplitude headroom** above the
G.711 reconstruction levels. This calibration belongs to the analogue
interface; it is not gain applied to digital codewords.

The internal `v92a_t` protocol core still accepts calibrated, symbol-clock
8 kHz linear PCM and produces a 16 kHz timeline, which represents
9.5.2.1.7's 24.5T interval exactly. The default audio output uses that same
16 kHz timeline, with calibrated amplitude and sixteen half-symbol ticks
(eight symbols) of transmit lookahead. Its streaming windowed-sinc reader
supports fractional clock adjustments; at a synchronous clock its positions
fall on the original sample lattice. This filter is an implementation
choice, not a pulse shape prescribed by V.92.

The experimental analogue receiver acquires Sd and feeds a T/2 equalizer.
Its adaptive clock/equalizer path is not yet validated through complete audio
startup. The legacy `rx_phase` argument is accepted in 0–1 but acquisition
determines the phase itself. Clipping is counted by `v92a_audio_clipped()`.
`v92a_audio_init_rate()` retains explicit alternate-rate experiments; the
default interface and startup matrix use 16 kHz.

In the audio harness, the network DAC decodes each downstream G.711
codeword once and reconstructs it directly at 16 kHz with eight symbols of delay.
The network ADC samples upstream audio at 8 kHz and quantizes once to
G.711. The combined sixteen-symbol delay is supplied as the round-trip
delay. No G.711 codec operation occurs inside `v92a_audio_t`. The digital
transmitter still emits original DS0 codewords directly.

Phase 2 uses two independent SpanDSP instances with opposite roles.
`v34_set_v92_info0_capabilities()` selects the role-specific INFO0 bits;
`v34_set_v92_pcm_upstream_capability()` enables digital advertisement or
analogue selection of PCM upstream. The analogue side selects Table 18 only
after receiving the digital capability in INFO1d bit 70. Its reserved bits
40–49 are ones. Without mutual capability or local PCM support it retains
the V.90 upstream selection. Table 18's baseline filter capacities are
supported by the CPd codec and waveform core: 192 total coefficients,
128 per section.

At the end of INFO1a, supply its selected U_INFO, codec law, MD duration,
measured round-trip delay, upstream amplitude and local DIL descriptor to
`v92a_init()`. Continue calling `v92a_tx()` / `v92a_rx()` through Phases 3
and 4. `v92a_phase4()` exposes final-training state and the received CPd.
`V92A4_DATA` means local B1u was sent; `v92a4_downstream_ready()` separately
means all 48 received B1d frames passed validation. The current endpoint
can send application payload through `v92a4_set_data_source()` after B1u;
`v92a4_get_data_bits()` retrieves downstream payload. The audio endpoint's
`v92a_audio_core()` accessor exposes the controller for this configuration
and status. Do not drive the core's audio calls separately when using the
16 kHz front end.

## Startup behavior and fixes

- Full Phase 2 survives a unilateral short-start request: 9.4 requires both
  requests. INFO0a and INFO0d reverse the capability/request bit positions.
- Table 18 MD is measured in 276 samples, not 280. Ru2 acquisition waits out
  the advertised MD interval before starting its timeout.
- TRN1u uses absolute GPA-scrambled signs (8.5.7). Ja and CPt introduce
  differential signs, seeded by the final TRN1u sign.
- Ja acceptance requires an actual CRC-valid Table 20 frame. Diagnostic CRC
  repair cannot publish a native startup event. A negative ranking score
  cannot reject an otherwise valid zero-DIL descriptor.
- The analogue controller transmits Ru/Ru-bar, TRN1u and Ja, receives
  Sd/Sd-bar and Jd/Jp/Jp-prime, performs the four Su segments, receives DIL
  or SCR, and transmits CPt/E1u. Su polarity transitions preserve waveform
  phase. The digital detector cannot mistake the three-slot polarity alias
  for a reversal; it verifies the middle returned Su separately.
- With zero DIL, actual upstream TRN1u acquisition starts Ri. Waiting for CPt
  first deadlocks 9.5.1.1.13 against 9.5.2.1.10. With nonzero DIL, Ri-bar
  waits for E1u after a whole CRC-valid CPt, not another CPt frame.
- The analogue Phase 4 controller receives mapped SUVd/CPd through the shared
  downstream demapper and transmits TRN2u, SUVu, CPu, E2u and B1u. CPd must
  select an offered upstream rate and a supported waveform profile.
- The zero-DIL constellation excludes Ucode 0: positive and negative mu-law
  zero become indistinguishable after D/A, destroying the encoded sign.
- Digital CPd constellation points precede gain G (6.4.2). The offer uses
  inverse-scaled G.711 levels so applying G lands on distinct network ADC
  levels; the former offer attenuated low points into the same quantizer
  cell. Points exceeding Table 30's 16-bit range are excluded and the rate
  is bounded by the resulting modulus product.
- CPd/CPu retries are driven by a complete unacknowledged peer message after
  100 ms plus the configured round-trip delay (9.6.1.1.3 / 9.6.2.1.3).
  Acknowledged control-frame boundaries arm Ed detection; zero fields inside
  another CPd cannot masquerade as Ed. B1 resets the respective mapper,
  scrambler, trellis and filter memories.

## Verification

`make v92_startup_test && ./v92_startup_test` runs:

- Real Phase 2 waveform exchanges for both laws, including Table 18 PCM
  selection and independent capability / short-request fallback cases.
- Coupled Phases 3–4 through **both B1 directions**, on PCMU and PCMA with
  zero DIL and a measured DIL constellation. Events come from received
  waveforms, never injected from the other endpoint's intended state.
- Each startup/erasure case runs both at the original core seam and through
  reconstructed 16 kHz analogue audio, and grades at least **1,024 varied
  upstream payload bytes** against the source, with zero rejected frames
  and zero byte errors. Startup is no longer the only success condition.
- Audio checks bound reconstruction peaks for every possible int16 input
  history, verify 3 kHz waveform reconstruction, recover all 256 codeword
  levels for both G.711 laws at the DAC sampling lattice, and check arbitrary
  TX/RX callback boundaries. The full audio pairs assert zero clipping.
- Recovery when the network erases the first entire CPd, on both laws.
- Su phase/polarity ambiguity and silence rejection, linear amplitude and
  chunk continuity, an independent GPA/sign oracle, MD gating, and the
  baseline filter-capacity codec/waveform round trip.

Phase 2 and Phases 3–4 are joined in the harness by the decoded INFO1a
selection; this is not yet a single V.8-through-DTE session implementation.
The target is included in `make test`.

## Remaining limits

This is an ideal, calibrated bearer implementation. Nonzero analogue MD
waveforms and nonzero Jp fractional corrections fail explicitly. The
analogue front end now has finite windowed-sinc reconstruction, but still
needs continuous timing recovery and equalization for a physical two-wire
line. The tests use the known sampling instant; they do not establish
operation at an unknown fractional phase or with independent audio clocks.
The native initial Ja receiver now retains a rolling 6,144-symbol window:
9.5.1.1.3's 2040T arms acquisition rather than fixing the analogue peer's
Ja onset. Exact Table 20 decoding continues after the original training
history leaves the window; the equalizing fallback still requires that
original history. The call owner must supply 9.5.1.2.1's deadline from the
end of INFO1a plus measured round-trip delay; this receiver does not implement
that retrain procedure.

The live digital engine uses the receiver fixes. The new analogue controller
is linked into the server but is not yet wired into the SIP analogue role;
that role retains its existing V.90 behavior. V.8 integration, automatic
retrain/restart, DTE delivery and hardware interoperability remain to be
completed. The controller reports the 9.6.2.2.1 B1d timeout to its call owner
rather than pretending to complete or silently falling back.

Passing these tests establishes startup on the modeled bearer, not
interoperability with an external V.92 modem.

## 2026-09-08 acquisition regression checks

`./v92_startup_test --ja-only` sends 12,000T of analogue TRN1u through the
network ADC, followed by a CRC-damaged Table 20 descriptor and valid repeats.
Both G.711 laws must reject training and the damaged frame, accept the next
exact descriptor, and report its absolute sample index after multiple buffer
rolls. `--core-only` runs the six coupled core startup cases (both laws,
zero/measured DIL and first-CPd erasure), including payload grading.

The full startup suite fails in the PCMU zero-DIL audio case at the
analogue Phase-4 DATA assertion, both at the former 48 kHz rate and at the
current 16 kHz baseline. The earlier 48 kHz failure also reproduced with
the unchanged Ja receiver and after rebuilding the local objects. Therefore
the earlier audio-suite pass claims above are historical, not validation of
the current checkout. The current adaptive audio implementation needs further
work before its startup or independent-clock behavior can be claimed.

## Amendment audit

See [the amendment audit](v92_spec_amendment_audit.md) for item-by-item
coverage of both amendments and the corrigendum. The current fixes add
Amd.1's differential SCR, correct control-message CRC coverage in both
modem directions, and apply Table 31's reserved-bit receive rule. Run
`./v92_startup_test --spec-only` for independent wire checks; ordinary
encoder/decoder round trips did not expose these mismatches.

## 16 kHz baseline

`V92_AUDIO_RATE` is 16000 and `V92_AUDIO_PER_SYMBOL` is 2. The test network
DAC generates two samples directly per DS0 codeword; it does not generate
48 kHz audio and decimate it. All six default audio startup cases use this
baseline. The digital DS0 and its ADC sampling clock remain 8 kHz.

`./v92_startup_test --audio-checks` passes the 16 kHz reconstruction,
headroom, both G.711 ladders and arbitrary TX callback checks.
`./v92_startup_test --audio-case 16000` still fails at the analogue Phase-4
DATA assertion. The sample-rate change does not resolve that receiver gap.

## 2026-09-09: the Phase-4 stall was a repeated CPt resetting the transmitter

The preceding audio-failure attribution is superseded by this investigation.
The PCMU zero-DIL audio case receives two valid CPt sequences. The second
arrives after TRN2d begins. `v90_set_phase4_cp()` unconditionally called
`v90_configure_phase4_mapper()` for native V.92 CPt, resetting the scrambler,
differential/shaping memories and partial mapping frame in the running stream.
The V.90 branch already treated identical repeats idempotently.

The relevant specification is V.92 (11/2000) 9.5.2.1.10-.11 (printed
pages 48-49): the analogue modem finishes its current CPt after detecting
barred Ri. A repeat in flight is therefore normal. V.92 8.8.6 inherits
V.90 (09/1998) 8.6.5 (printed page 28), which initializes the mapper
memories before TRN2d, not again on each received CPt. The local amendments
and corrigendum do not replace these clauses.

Before the fix, the downstream receiver recovered 541 consecutive TRN2d
ones and then lost the sequence. Digital SUVd repeated with ack=0 forever.
Comparing the transmitter's TXRAW stream with the receiver's EQRAW/P4CW
stream, using their measured 3391-symbol index offset, found **zero sliced
codeword differences from the first TRN2d symbol through the end of the
20-second run**. Equalized levels were within three linear units. The low
approximately +/-64 levels were the requested zero-DIL constellation, not
equalizer collapse; the CMA dispersion metric near 1 is meaningless on
that multilevel signal. Accurate symbol reception could not undo the
transmitter's unannounced reset.

Native V.92 now accepts an identical repeated CPt without reconfiguring the
mapper and rejects changed or acknowledged CPt. All six core startup cases
explicitly inject a repeat 271 symbols into TRN2d, within a mapping frame,
and still validate both B1 directions and at least 1024 upstream payload
bytes without errors. They also check rejection of changed/acknowledged CPt.
The spec-only, audio reconstruction, full V.PCM loopback and V.92 procedure
evaluation suites pass.

The 16 kHz audio case now completes SUVd/CPd, both acknowledgements, Ed,
and validated B1d; both transmitters reach DATA. It subsequently fails
`sink.b1.locked`: digital upstream B1u acquisition remains unresolved,
with zero payload bytes delivered. The new failure summary reports
`analogue_p4=6 downstream_b1=1 digital_tx=21 cpt=2 cpu=1 upstream_b1=0 payload=0`.
This fixes the original control-exchange failure, not complete audio startup
or foreign-modem interoperability. Temporary symbol/callback diagnostics
used for the comparison were removed; existing `V92_AUDIO_TRACE` TX tracing
and EQRAW tracing can reproduce the waveform comparison.

## 2026-09-09: upstream B1u acquisition uses the data-mode trellis

The B1u failure exposed above is fixed. At the correct 576-symbol window,
the PCMU audio case had correlation 0.999999628, gain 1.000006013 and offset
0.013725371 against the generated B1u reference. Five network-ADC outputs
differed from that reference by one G.711 level. The scalar acquisition
branch used the hard-decision waveform decoder, which rejected frame 41
after sample 496 arrived as -16 instead of -8. A high correlation does not
guarantee that every nearest-point decision is correct.

V.92 8.7.1 sends B1u with the data-mode constellation and convolutional
encoder, initialized at its start. For the supported unfiltered 16-state
profile, acquisition now uses the same Viterbi decoder as payload reception.
The gain/correlation gates and the requirement to decode all 48 frames to
source ones are unchanged. Filtered profiles retain their existing decoder.
No bearer gain, G.711 conversion, symbol count or protocol timing changed.

The focused loopback regression introduces one wrong nearest-point decision
in B1u, then verifies 520 payload bytes and receiver state continuity. It
fails with the old acquisition decoder (no lock, zero bytes) and passes
with the correction. The existing fractional-channel equalizer case also
passes. The full V.PCM loopback suite and six core startup cases pass.

`./v92_startup_test --audio-zero-dil` separately exercises both laws, with
and without first-CPd erasure, through B1u/B1d and at least 1024 correct
upstream payload bytes. The full startup suite still encounters an earlier
Phase-3 `V90_RX_EVENT_SU` assertion in the PCMU measured-DIL audio case;
that case has not reached B1u and is a separate remaining startup failure.


### Sd acquisition after the Su gate correction (2026-09-09)

V.92 §9.5.1.1.4 arms Su detection when Jd begins. The live engine and
startup harness now leave the Su detector unprimed during Sd/Sd-bar, when
the analogue peer is still sending Ja. With that gate, the measured-DIL
PCMU audio case exposed an Sd-bar timeout instead of the false Su event.

The analogue acquisition accepted a 512-half-symbol window whose training
half was mostly silence. Its held-out fit score was 0.831, above the 0.80
threshold, but the steady output's nominal zero slots were 0.523 of the
normalized Sd level. The core's 0.20 zero-slot tolerance could not acquire
that pattern. This was a bad equalizer fit before the reversal, rather than
a missing reversal on the transmitted stream.

`v90a_sd_fit()` now requires the training half to meet the existing fit
threshold as well as retaining the independent held-out check. The sliding
hunt retries incomplete onset windows. The regression in
`v90_analogue_sd_test` rejects silence followed by Sd and acquires the later
window while the finite 384-symbol Sd preamble is still present. The rejection
fails against the previous implementation and passes with this change.

The measured-DIL PCMU audio case now detects Sd-bar, trains on TRN1d,
receives Jd, completes Su and reaches the Phase-4 control exchange. It still
fails downstream B1d validation; complete measured-DIL audio startup is not
established. The standalone Sd suite, all six core startup cases and the
four zero-DIL audio cases (both laws, with/without first-CPd erasure) pass.
`make test` reaches the same measured-DIL B1d validation failure in
`v92_startup_test`; the overall suite is therefore still failing. The
spec-only and audio reconstruction checks also pass.
No G.711 codewords, DSP constants or transmit timings changed in this fix.

### Measured-DIL B1d failure was a stale build (2026-09-27)

The failure above does not reproduce from a clean build of the same protocol
sources.  All four measured-DIL audio rows (PCMU/PCMA, with and without the
first CPd erased) complete the control exchange, receive Ed, validate all 48
B1d frames and deliver at least 1024 upstream payload bytes without error.

The source had been built with the makefile's header-dependency hole.  The
result therefore was not evidence of a malformed B1d waveform: translation
units could retain incompatible private-state layouts after a header edit.
The makefile now emits and includes compiler dependency files (`-MMD -MP`),
and `make clean` removes them.  `./v92_startup_test --audio-measured-b1d` is
the focused PCMU regression for the complete measured-DIL path through the
Ed-to-B1d handoff.  It follows V.92 8.8.1 and 9.6.1.1.5, which inherit V.90
8.6.1's 48 reset-state data-mode frames.  No G.711 conversion, DSP constant,
sample accounting or protocol timing changed.

## 2026-10-11: whole engines reach V.92 data mode, analogue against digital

Two complete engines (`engine_pair_test`, byte-exact G.711, both laws) now
run V.8, full Phase 2 with INFO0 bits 26/27, a Table 18 INFO1a, V.92
Phase 3 (Ru/TRN1u/Ja, Sd/TRN1d/Jd, Su, CPt), Phase 4 (SUVd/SUVu, CPd/CPu,
Ed, B1d/B1u) and data mode, and exchange DTE payload in both directions
over LAPM + V.42bis. Both ends report `Modulation V92` in ATI6.
mu-law connects at 34666 up / 56000 down, A-law at 24000 / 56000. The
regression rows are in `tests/fast.list`; the GUI loopback's V.92 profile
asserts the same over localhost SIP. Three things stood in the way, none of
them in the V.92 layer itself:

1. **+PIG.** PCM upstream is offered only with `AT+PIG=0` (or
   `ME_V92_PCM_UPSTREAM=1`); +PIG is factory 1 here, a documented deviation
   from 6.8.5's default of 0 (`docs/v250_command_conformance_audit.md`). With +PIG=1, V.92 mode still sets the INFO0
   capability bits and falls back to a V.90 INFO1a (V.92 9.3), which the
   third regression row checks. V.92 capability is negotiated in INFO0, not
   in the V.8 QC octet; the analogue role leaving that octet out is correct
   for full startup.
2. **A Phase 2 deadlock in the digital receiver (SpanDSP `v34rx.c`).** The
   2400 Hz spectral gate in `tone_a_carrier_present()` exists to stop the
   V.21 JM tail reading as Tone A before INFO0a arrives. It is measured over
   40 ms blocks and stayed active after INFO0a, so the block straddling
   INFO0a's tail read low and blanked the next 40 ms of real Tone A. V.90
   9.2.2.1.3 lets the analogue modem reverse after 50 ms of Tone A, which is
   the 30 bauds the detector needs anyway, so the reversal was lost and the
   analogue (FIRST_NOT_A) and digital (V90_PHASE2_B_INFO0_SEEN) waited on
   each other for ever. Whether the block landed on the wrong side depended
   on INFO0a's bit content: setting bit 26 was enough. The gate now applies
   only until INFO0a is received (V.90 9.2.1.1.2: Tone A follows INFO0a).
   Any analogue peer that reverses near the 50 ms minimum was exposed to
   this, V.92 or not.
3. **The analogue V.92 receive path never drained to the DTE.**
   `me_rx_audio()`'s `g_v92a` branch returned before reading
   `upstream_ring`. A digital-TX / analogue-RX bit dump
   (`DS_TX_BIT_DUMP`/`DS_RX_BIT_DUMP`) showed 626k downstream bits with
   zero errors, LAPM connected, and 0 octets reached the PTY.
   `v92_startup_test` grades upstream payload only, so nothing had graded
   downstream payload after B1d.

Still open: no foreign V.92 modem has been tried; the bearer here is
byte-exact, not an analogue loop. (The upstream rate is resolved below.)

## 2026-10-11: upstream 34666 -> 48000 -- the CPd design, not the channel

On the byte-exact engine pair the digital modem offered 34666 (A-law
24000) because its Table 30 constellation had 24 levels. Three limits in
`v90_build_v92_cpd_frame()` and its inputs compounded; a model of the
greedy level picker reproduces every observed rate from them:

| design | levels | rate |
|---|---|---|
| before: sigma 27.5, odd Ucodes, G = 0.125 | 24 | 34666 |
| sigma 4.4, every Ucode, G = 0.125 | 64 | 45333 |
| sigma 4.4, every Ucode, G = 0.25, power-bounded | 78 | 48000 |

1. **The noise figure was the network ADC's rounding of TRN2u.** Points are
   spaced 2 x 4 x sigma apart. Sigma was the equaliser output's distance to
   the ideal Table 28 level, but those levels (+/-LU/sqrt5, +/-3LU/sqrt5) are
   off the G.711 grid, so the ADC rounds each one by a fixed amount, and
   3LU/sqrt5 = 8050 clips at the top codeword. Table 30's data points are
   codec levels (6.4.2) and are never rounded. Sigma is now the spread
   *within* each decided level, which drops the fixed per-level offset:
   27.5 -> 4.4 DS0 units (A-law 5.9). A least-squares fit that also
   removed non-linear leakage into neighbours measured the same 4.4, so the
   simple estimator stays.
2. **Only odd Ucodes were candidates.** At most 64 levels; 12 log2 62 - 3 =
   68 bits per frame stops 6.4.1's product(Mi) >= 2^K at drn 17 (45333) even
   with sigma = 0. 48000 (K = 72) needs about 76 levels. Every Ucode is now a
   candidate and the spacing rule thins them.
3. **G = 0.125 cut off mu-law's top two segments.** Points are 16-bit and,
   in the convention our two ends share (the analogue sends G x v), G x point
   is the DS0 level, so 4G = 0x8000 capped levels at 8191. 4G now defaults
   to Table 30's largest (0xFFFF), and the top level is set by Table 30's
   power rule instead: the design assumes G x v at mean square 1 is the
   desired power, which 3.8 makes LU's, so the constellation's mean square is
   held at or below the received LU's.

Validated with `engine_pair_test --hold-seconds 60` and bit dumps
(`DS_TX_BIT_DUMP` on the analogue, `DS_RX_BIT_DUMP` on the digital): 48000 on
both laws, 2,915,280 upstream bits each, zero errors. With the spacing forced
to zero (94 levels, decision distance 4 DS0 units at the bottom) it was also
error-free, so the 4.4 still in sigma is not noise data mode sees on this
bearer; the remaining 4 x sigma margin is deliberate headroom for real lines.

The within-level sigma has only been checked on a byte-exact bearer; on an
analogue loop it includes linear ISI the data path also suffers, which is
intended. (The G x v gain convention used above was replaced the same day;
see the next section.)

## 2026-10-11: the spec's LU x G x v upstream convention, both ends

Table 30: the digital modem "shall design the modulation parameters
assuming that, when the prefilter output multiplied by G has a mean-square
value of 1, the analogue modem will transmit at the desired power", and 3.8
makes LU that power. A conforming analogue modem therefore transmits
LU x G x v, and a point reaches the network ADC as LU_rx x G x point. Ours
used to transmit G x v, a private convention that capped levels at
65535 x G; slmodemd follows the spec, and `ME_V92_CPD_GAIN_PER_LU=1` was the
opt-in that adapted our digital side to it. Now both ends follow the spec
and the knob is gone:

- **Analogue** (`v92_analogue_phase4.c`): data-mode samples are
  LU x (G x v); the wave core stays normalised, so the BER/loopback/B1u
  harnesses that exercise it directly are unchanged.
- **Digital** (`v90_build_v92_cpd_frame()`): once LU_rx (the received TRN2u
  rms) is known, G is derived -- the smallest 4G that still lets a 16-bit
  point reach the top codeword -- and points are placed so G x LU_rx x point
  is the codec level. The Table 30 power bound then sets the top level.
  Without LU_rx, G x point is taken as the DS0 level (LU_rx = 1).
  `v90_set_v92_upstream_lu()` reports LU_rx alone, keeping a pinned drn;
  `v92_startup_test` now measures it the way the engine does.
- `V90_V92_TX_QUEUE_BITS` was 2048. With every codec level in range a single
  128-point set is 2176 bits of points, and the CPd silently failed to
  encode. It is now `V92_CPD_MAX_BITS`.

Engine pair, both laws: 48000 up / 56000 down; 60 s holds carried
2,915,280 upstream and ~3.4 M downstream bits each, zero errors.

### The downstream twin: the analogue's CPu now follows TRN2d

The conversion changed CPd content and tipped one reconstructed-audio row
(PCMA, measured DIL): B1d frame 0 sliced A-law 136 as 152 (equaliser output
145, 88% of the way to the boundary), and the self-synchronising
descrambler carried that into frame 1. The digital sent identical B1d
codewords either way; the analogue's own CPu had offered A-law levels 16
apart. That CPu came from Phase 3's `v90_analogue_phase4_build_cp()`, which
applies 3 sigma, but with sigma measured on the DIL. The DIL holds each
level for a segment, so the equaliser's data-dependent noise on random
symbols never shows there. V.92 9.6.2 puts CPu after TRN2d, and TRN2d is
random mapped symbols, so `v92a4` now measures decision noise on TRN2d and,
just before its first CPu, thins each constellation to the same 3-sigma
separation and backs drn off until 5.4.3's 2^K <= prod(Mi) holds
(`v90_analogue_phase4_set_cp()` gives the receiver the same CP). Thinning
keeps CPu a subset of the Phase 3 CP. Result: byte-exact rows unchanged
(drn 22); the audio rows thin, PCMA from 92/93 to 66/77 points at the same
drn 22, PCMU to drn 21. The shared V.90 analogue role is not changed.

## 2026-10-11: V.92 PCM upstream against slmodemd, with payload

Live on the tower rig (`tools/soak/slm_call.sh`, slmodemd dialling us; our
build from a separate tree via the new `SRV_DIR`, because `/root/v90modem`
also runs a long-lived service):

```sh
SRV_DIR=/root/v90cpu/v90modem ORIG=slm MS=92,0,300,56000 \
SLMODEMD=/src/slmodemd/slmodemd_map SLM_V92_PCM_UPSTREAM=1 DM_V92EC_BYPASS=1 \
ME_MODE=v92 ME_V92_PCM_UPSTREAM=1 PAYLOAD=1 PAY_DELAY=50 ./slm_call.sh <name> 150
```

`v92lu-pay1`: slmodemd reports `Link: DP is V.92, rate: rx 56000, tx 24000`;
our B1u locked by decoding, LAPM + V.42bis came up, and **150,564 numbered
lines arrived upstream and 111,800 downstream with no gaps** until the
harness hung up at the end of the hold. This is the first V.92 PCM-upstream
call against a foreign modem to carry DTE data.

The first attempt under the LU x G x v convention (`v92lu-r1`) never sent a
CPd: the Table 30 power bound took every point as equally likely, which on
slmodemd's quiet upstream (LU_rx 1402) left too few points for drn 1, so the
peer waited in Phase 4 and retrained to V.90. Table 30's note has the
analogue modem minimise power at the precoder output, and with Mi = LC each
class is {p[K], -p[LC-1-K]}, so the power is the mean of
min(p[K], p[LC-1-K])^2. `v90_build_v92_cpd_frame()` now bounds that. A side
effect: `v92_startup_test`'s digital side had never reported noise, so it
designed against sigma = 0 and the densest set now reached the top codec
levels, which the reconstructed-audio transmitter's tracked-clock
interpolation cannot carry; `v90_set_v92_upstream_lu()` now takes the
within-level TRN2u spread too, as the engine measures it.

24000 is what this rig's upstream supports at the 4-sigma spacing and LU
power: sigma 53 against LU_rx 1402 (about 28 dB), and here sigma agrees with
the distance-based error (0.038 LU), so it is real impairment (slmodemd's
9600 Hz transmit FIR and d-modem's resampler), not codec rounding.

Not testable on this rig: A-law (d-modem offers only PCMU; a 2905 call came
up CLEARMODE and dropped at once), and us calling slmodemd in V.92 (its
answer-mode JM offers no PCM).

## 2026-10-11: slmodemd upstream 24000 -> 32000

What bound 24000 was our receiver, not the line. On the `v92lu-pay1` tap,
TRN2u fitted offline:

| fit | residual |
|---|---|
| best 63-tap linear equaliser (receiver) | 55.7 DS0 (0.040 LU) |
| linear channel model (transmitter-side bound) | 21.6 DS0 |
| least-squares DFE, 63 + 8 taps | 26.8 DS0 (0.019 LU) |

The engine's equaliser has that DFE structure (8 feedback taps) but its
decision-directed NLMS had stopped at the LINEAR equaliser's error (53).
Two changes, both receiver-only:

1. `v92_p3_eq_refit()` solves the feed-forward and feedback taps by least
   squares over a window of decided symbols, the same rows as the TRN1u
   seed. The engine runs one over 4000 TRN2u symbols and restarts the noise
   statistics afterwards, so the CPd is designed from the solved equaliser:
   sigma 53 -> 26.9, 10 -> 20 points, drn 1 -> 7, **24000 -> 32000**
   (`Link: DP is V.92, rate: rx 56000, tx 32000`).
2. **Data mode re-solves every 2 s instead of freezing.** At 32000 the frozen
   taps drifted off this channel at ~1.5 DS0 per 10 s (27.6 -> 43.7 over
   120 s; gain and offset steady), rejected frames rose to ~1300 and
   slmodemd hung up. Re-solving the same structure per window offline held
   26-44; its feed-forward centroid wanders by about a sample over 100 s
   (~1 ppm). NLMS with the timing loop on the dense data levels made it
   WORSE (46.8 by 70 s), and the timing loop alone did nothing. Live with
   back-to-back 16000-symbol least-squares solves (`v92ls-pay5`): distance
   to the data levels flat at 26.4-27.2 DS0 for 120 s, **zero rejected
   frames, 163,700 lines up and 108,600 down with no gaps**.

`[ME] V.92 upstream data:` now logs every 10 s of data: rms distance to the
nearest level, frames rejected, and a y = g d + c fit (gain, DC offset).

Precoding (Table 30 z2/p1, which slmodemd parses: `prefilterPrecoderPresent`)
was evaluated and not built: the rate here is bound by Table 30's power
(the levels must fit within LU_rx's mean square), and a transmit-side
prefilter's +1.9 dB power penalty eats most of what its lower noise buys
(modelled 28000 at best with a pessimistic precoder power, against 32000
from the receiver fixes).

## 2026-10-11: repeated runs against slmodemd at 32000

Three 10-call batches (slmodemd dialling us, V.92 PCM upstream, payload both
ways, 150 s holds; `v92_batch.sh` on tower, one summary line per call):
every call linked at V.92 tx 32000 / rx 56000 with sigma 26.87 and drn 7, and
every call delivered its lines with no gaps.

| outcome | calls |
|---|---|
| clean for the whole hold | 24 / 30 |
| ended by slmodemd's LAPM (2 x DISC, 3 x `EC UNLINK`) | 5 / 30 |
| slmodemd retrain, link survived | 1 / 30 |

The five LAPM endings all came with our modem layer healthy (level distance
~27 DS0, few rejects). On `v92b3-4` our transmit tap against what
slmodemd's DSP received (`/tmp/dm_to_dsp.raw`) shows the downstream intact to
the last second (lag 0, correlation constant), so the d-modem/RTP path is
cleared; slmodemd logs ~25 downstream FCS errors per V.92 call against 1-2 per
V.90 call on the same rig. A V.90 control batch (`v90c1`, 6 calls) held every
call -- but its V.34 upstream at 31200 delivered about a tenth of the lines
(we sent ~80 REJ per call; V.92 calls none).

Two defects of ours, fixed:

1. **The data-mode refit could ratchet.** One clean call went 26.8 -> 78.4 DS0
   in a single 10 s window; a fresh least-squares solve on the same samples
   stayed at 35-46, so the signal was fine and our solve had fitted a burst
   of wrong decisions. `v92_p3_eq_refit()` now keeps its current taps when a
   solve's own residual exceeds twice the first accepted one's
   (`v92_p3_eq_refits_rejected()`; the 10 s data log reports solves, refusals
   and the last residual).
2. **A retrain lost V.92.** slmodemd retrained once; we answered with V.90
   9.5.1.2's INFO0-less retrain, but `v34_restart()` had cleared its INFO0a
   bits and the engine its V.92 confirmation, so INFO1d went out in the V.90
   form and slmodemd logged "V92 capabilities: local=1, remote=1,
   selected=90 ... isPCM - 0": V.34 upstream at 31200 for the rest of the
   call. V.92 9.3: "any subsequent retrains shall use Phase 2 of V.92". The
   confirmation now survives `restart_v90_phase2_locked()` (all of whose
   callers are 9.5 retrains) and `v34_set_v90_peer_info0_flags()` restores
   the peer's bits in SpanDSP.

`ME_V90_RETRAIN_AFTER_MS` (test hook, the sibling of
`ME_V34_RETRAIN_AFTER_MS`) initiates a 9.5.1.1 retrain n ms into data. Our own
V.92 analogue role does not answer a retrain yet, so this is checked against
slmodemd only.
