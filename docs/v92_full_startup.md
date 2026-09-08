# V.92 full startup: digital and analogue interfaces

The reference is `ITU Docs/T-REC-V.92-200011-I!!PDF-E.pdf`, particularly
3.1-3.2, 9.3, 9.4, Tables 15-18 and 8.5/8.7. This work concerns full
startup; it does not implement Quick Connect. The local Amendment 1,
Amendment 2 and Corrigendum 1 PDFs were also checked. Amendment 1 retains
Table 18's 276-symbol MD unit; the later documents concern retrain and
modem-on-hold behavior.

## Audio boundary

The digital modem's downstream is the actual G.711 DS0. Its existing
`v90_phase3_tx_codewords()` path generates the V.92 downstream without
decoding and re-encoding those codewords. Its upstream receiver consumes
the network A/D's G.711 stream.

The analogue transmitter produces signed linear PCM. `v92_trn2u.h` now
exposes `v92_trn1u_tx_linear()`, `v92_trn2u_tx_bits_linear()` and
`v92_trn2u_tx_ones_linear()`. These generate one sample per 8000 Hz symbol,
independently of the selected transport law. The old codeword APIs remain
compatibility wrappers that apply the network A/D exactly once. The V.92
CPt/CPu/SUVu tests now explicitly cross that boundary from linear PCM to
G.711 before feeding the digital receiver.

TRN1u uses absolute GPA-scrambled signs (8.5.7). Ja and CPt use differential
signs seeded by the last TRN1u sign (8.5.1, 8.5.4). The separate TRN1u
generator makes that distinction explicit; calling the differential TRN2u
generator with a two-point constellation is not equivalent.

These are symbol-clock linear outputs. They do not provide analogue line
pulse shaping, fractional timing adjustment or a complete analogue modem
controller. The G.711 law conversion in a test is the simulated network
codec, not part of the analogue modem's signal generation.

## Full Phase 2 corrections

The SpanDSP V.90 analogue role now emits V.92 capability/request bits in
INFO0a when configured through `v34_set_v92_info0_capabilities()`. Table 16
assigns capability to bit 26 and short Phase 2 to bit 27; Table 15 assigns
those meanings in the opposite order for the digital modem's INFO0d.
Previously the analogue transmitter always emitted the V.34 clock-source
field instead.

Both the live engine and the digital INFO1d transmitter now retain mutual
V.92 capability when only the analogue peer requests short Phase 2. Clause
9.4 requires both ends to request shortening; our digital endpoint leaves
its request clear, so 9.3's full V.92 Phase 2 still applies.

V.92 Table 18 codes PCM-upstream MD in 276-symbol (34.5 ms) units, whereas
V.90 and V.92 Table 19 use 35 ms. The live PCM-upstream path now uses 276.
The Phase-3 receiver waits out the signalled MD duration before accepting
Ru2, and its acquisition timeout starts after that interval. Previously it
could acquire an MD signal as Ru2 or time out on MD exceeding one second.
The soft Ru-bar detector's confirmation latency remains distinct from the
transmitter's actual boundary; receive anchors are not transmit timestamps.

## Verification and remaining work

`make v92_startup_test && ./v92_startup_test` exercises two independent
SpanDSP Phase-2 instances over PCMU and PCMA, with analogue capability and
short-request bits varied independently. It checks the received INFO0,
INFO1d bit 70 and INFO1a selection. The analogue Phase-2 implementation
still selects V.90 upstream; these tests do not claim PCM-upstream startup
completion. The target also checks analogue linear amplitudes, law
independence, chunk continuity, the TRN1u-to-Ja scrambler/sign seam, and
short/long MD receive gating. It is included in `make test`.

A complete live V.92 analogue startup remains to be implemented: Table 18
selection after checking INFO1d bit 70, an analogue Phase-3 controller with
Ru/TRN1u/Ja and fractional Su timing, Jp/Jp-prime reception, and the analogue
Phase-4 controller through CPd/E2u/B1u. Existing V.90 analogue startup must
not be relabelled V.92 or advertise PCM upstream before those pieces exist.
The digital Su detector also needs review against the four signal segments
in 9.5.2.1.6-9; polarity searches at independent phases cannot prove a
transition because Su-bar is also Su shifted by three symbols.

No new hardware interoperability claim follows from these offline tests.
