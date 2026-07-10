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

## Remaining native runtime work

The live V.92 startup path still needs:

- a CPu/CPu-prime receiver that can supply real acknowledgement events;
- full Table 30 optional-part support for modulus parameters, prefilter and
  precoder coefficients, and constellation sets;
- selection of a real CPd prefilter gain and upstream rate from receiver
  measurements rather than compatibility-profile values;
- transmission of CPd through the negotiated TRN2d mapper;
- strict Ed/B1d and DATA transition events connected to the analyzer;
- rate renegotiation, fast parameter exchange, and modem-on-hold procedures.

Until those pieces are connected, the shared startup harness's CP/B1 fallback
remains diagnostic compatibility behavior and is not native V.92 proof.
