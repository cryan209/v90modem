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
