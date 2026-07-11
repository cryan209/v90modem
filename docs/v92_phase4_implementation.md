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

## Live runtime integration

The SIP modem now advertises the V.92 QC/QCA byte alongside the V.90-family
CM/JM modulation set.  When the peer returns a V.92 byte, the live V.90
context switches to native V.92 Phase 4.  Raw, octet-transparent G.711 input
is passed directly to the 4-point TRN2u demodulator, which descrambles and
differentially decodes CPt, CPu/CPu-prime, and SUVu/SUVu-prime.  Accepted
frames drive the native transmitter gates described above.  The legacy V.90
Phase 4 bit callback remains selected when the peer does not negotiate V.92.

The 4-point selection matches the current Jp bit-48 profile.  The initial
TRN2u slicer reference is 8000 linear units and can be calibrated against a
real bearer with `V92_TRN2U_LU`; accepted/rejected frame and symbol counters
are included in the regular modem diagnostic snapshot.

## Remaining native runtime work

- derive TRN2u `L_U` from Phase 3/receiver measurements instead of the
  calibration default;
- apply CPus to a complete rate-renegotiation state machine (it is currently
  strictly decoded and logged but ignored during initial Phase 4);
- implement E2u detection as the alternate final acknowledgement path;
- connect fast parameter exchange and modem-on-hold procedures;
- validate the slicer, acknowledgements, Ed/B1d transition, and sustained
  data mode against real V.92 client modems over a transparent G.711 bearer.
