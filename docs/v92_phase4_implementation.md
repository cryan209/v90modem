# V.92 Phase 4 Implementation Status

The V.92 Phase 4 message layer now has strict codecs for the two fixed-size
forms that can be implemented without inventing negotiated filter or
constellation parameters:

- Table 31 SUVd/SUVd-prime, including the silent-period request,
  acknowledgement, reserved bits, start bits, CRC, and fill to 54 bits;
- the mandatory part of Table 30 CPd/CPd-prime when bits 19:21 indicate that
  all optional parts are absent, including selected upstream rate, trellis,
  E2u extension, acknowledgement, prefilter gain, CRC, and fill to 72 bits.

Both decoders are strict. They reject non-binary input, malformed sync or
start fields, non-zero reserved/fill bits, invalid parameter ranges, and CRC
damage. The existing V.92 transmitter now uses the shared Table 31 encoder
instead of maintaining a private SUVd implementation.

The Phase 4 analyzer tracks this procedure progression:

`Phase 4 -> SUVd -> CPd -> SUVd-prime -> Ed -> B1d -> DATA`

It reports the first missing or malformed layer instead of treating entry into
Phase 4 as completion.

## Native CPu/CPu-prime receiver

`v92_cp_rx.c` now implements strict codecs and a bit-level receiver for every
upstream Phase 4 control frame:

- Table 23 CPt/CPu/CPu-prime, including the variable-length constellation
  masks, the bit-128 codec-constellation extension (gamma/delta sizing), and
  fill to a 12-symbol multiple of the 4- or 8-point TRN2u modulation;
- Table 24 CPus;
- Table 27 SUVu/SUVu-prime, including the wait-for-CPu request and the
  measured prefilter level.

The receiver hunts the shared 17-ones sync, dispatches on bit 18 (SUVu) and
the CP type field (bits 19:20), derives the long-frame length from the data
frame interval indexes once bit 128 arrives, and delivers only frames that
pass CRC, start-bit, reserved-bit, parameter-range, fill, and codec-law
validation.  `v92_cp_frame_to_vpcm()` converts an accepted CPu into the
shared V.PCM CP representation so the existing negotiated mapper
configuration consumes it directly.

With `v90_enable_v92_native_cpu_rx()`, the digital Phase 4 transmitter is
gated by real receive events per 9.6.1.1/V.92: SUVd repeats until a valid
SUVu or CPu is received, a single CPd is sent per received SUVu/CPu, the SUVd
acknowledge bit is set only after a valid CPu (which also configures the
negotiated data-mode mapper), Ed starts only after an acknowledged sequence
has been sent and CPu-prime/SUVu-prime (or E2u) is received, and B1d runs
over the negotiated mapper at the CPu rate.  Repeated CPu frames may change
only their acknowledge bit.  `v90_get_v92_cpu()` exposes the accepted
CPu-derived parameters (rate, TRN1d gain, shaping filter, constellations)
for native CPd construction.

## Remaining native runtime work

The live V.92 startup path still needs:

- a TRN2u demodulator/descrambler feeding the CPu bit receiver from the
  G.711 stream (the Phase 3 receiver in `v92_p3_rx.c` stops at Ja);
- full Table 30 optional-part support for modulus parameters, prefilter and
  precoder coefficients, and constellation sets, so the CPd payload becomes
  a native Table 30 frame built from the received CPu instead of the legacy
  compatibility payload;
- selection of a real CPd prefilter gain and upstream rate from receiver
  measurements rather than compatibility-profile values;
- transmission of CPd through the negotiated TRN2d mapper (V.92-mode CPt is
  still applied through the compatibility path, so SUVd/CPd remain
  sign-modulated at U_INFO);
- strict Ed/B1d and DATA transition events connected to the analyzer;
- rate renegotiation, fast parameter exchange, and modem-on-hold procedures.

Until those pieces are connected, the shared startup harness's CP/B1 fallback
remains diagnostic compatibility behavior and is not native V.92 proof.
