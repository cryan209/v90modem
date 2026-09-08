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

The analogue endpoint accepts calibrated, symbol-clock **8 kHz linear PCM**
from the network DAC and produces **16 kHz linear PCM**. The doubled TX
clock represents 9.5.2.1.7's 24.5T interval; it uses an ideal sample-and-hold
waveform. The harness samples that waveform at the network's 8 kHz ADC.
G.711 encoding belongs to that network boundary, not the analogue modem.

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
continues sending idle marks after B1u; a DTE payload interface is not yet
attached to this controller.

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
analogue front end still needs bandlimited pulse shaping, continuous timing
recovery and equalization for a physical two-wire line. The native initial
Ja receiver also retains a bounded acquisition window around minimum-length
TRN1u; longer peer training needs a rolling search.

The live digital engine uses the receiver fixes. The new analogue controller
is linked into the server but is not yet wired into the SIP analogue role;
that role retains its existing V.90 behavior. V.8 integration, automatic
retrain/restart, DTE delivery and hardware interoperability remain to be
completed. The controller reports the 9.6.2.2.1 B1d timeout to its call owner
rather than pretending to complete or silently falling back.

Passing these tests establishes startup on the modeled bearer, not
interoperability with an external V.92 modem.
