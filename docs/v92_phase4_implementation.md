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

TRN2u entry follows the split initialization required by 8.7.6/V.92: GPA is
reset to zero at the beginning of TRN2u, while the differential sign memory
is carried from the final E1u sign. `v92_trn2u_tx_start()` makes that boundary
explicit instead of relying on both states having the same initial value.
The demodulator also records its longest run of descrambled ones. A real
TRN2u interval should make this counter very large; values around 20 on an
entire call are indistinguishable from random data and show that symbol or bit
recovery has not locked yet.

Raw bearer captures can be replayed directly through the same receiver with:

```sh
make v92_trn2u_replay
./v92_trn2u_replay capture.g711 --law ulaw --points 4 --lu 8000
```

The tool reports every CRC-valid CPt, CPu, CPus, or SUVu with its sample/time
position and ends with receiver counters.  It is intended for octet-transparent
DS0 captures.  Analogue recordings sampled at 8 kHz are a harder input: their
sample clock and phase are not necessarily locked to the 8 ksymbol/s PAM, so
failure to find frame sync there identifies a need for timing recovery and
equalization rather than proving that the strict frame codec is incorrect.

For those analogue recordings, the same tool has an offline front end with
fractional linear interpolation, sample-clock correction, gain control, and a
five-tap normalized decision-directed PAM equalizer:

```sh
./v92_trn2u_replay call.wav --analog-wav --channel R \
  --start 76000 --max 16000 --phase 0.375 --clock-ppm -250 \
  --points 4 --lu 75 --timing-loop --timing-mu 0.002 \
  --equalizer --eq-mu 0.01
```

The timing loop uses a decision-directed Mueller-and-Muller error detector on
the PAM decisions; `--clock-ppm` supplies its nominal rate. Corpus sweeps should
vary `--start`, `--phase`, `--clock-ppm`, and `--lu` and rank configurations
first by valid frames, then by rejected frame candidates.

Diagnostic wire hypotheses are available without changing the live defaults:
`--perm` selects the recovered bit order within each symbol, `--sign` compares
differential/absolute and normal/inverted sign decisions, and `--descrambler`
compares the GPA/GPC polynomials in both register directions. The compliant
default is `--perm 01 --sign diff --descrambler gpa-left` for 4-point TRN2u
(`012` for 8-point).

Across the Agere, Motorola, and USR QC captures, the complete hypothesis sweep
raises the longest descrambled-one run only to 39 bits. A locked TRN2u interval
should produce thousands of consecutive descrambled ones. No tested bit-order,
polarity, differential, or polynomial alternative therefore explains the
recordings; the dominant remaining issue is symbol recovery/channel filtering
in the critically sampled recordings. The corrected upstream windows and
channel assignments are recorded in `docs/v92_test_call_windows.md`.
On the checked-in Agere, Motorola, and USR V.92 calls, fractional timing and
equalization now expose occasional 17-one frame candidates, but none yet pass
the strict CRC.  This narrows the remaining offline problem to adaptive timing
tracking/equalizer training or a TRN2u wire-format assumption; amplitude alone
is not the cause.

## PCM-upstream data-pump foundation

`v92_upstream_data.c` starts the post-B1u receive path with the two reversible
bit-domain stages from the analogue-modem transmitter.  It implements the
GPA scrambler in 6.3/V.92 and the exact twelve-interval mixed-radix modulus
encoder in 6.4.1/V.92, including the frame-sign differential state and the
Table 30 rate relation `K = 2 * (drn + 17)`.  The inverse recovers and
descrambles a data frame from K0 through K11 and rejects values outside the
K-bit source alphabet when the modulus product is larger than `2^K`.

The module now continues through 6.4.2-6.4.4 for the initial 16-state profile:
it selects the minimum-power member of each modulus equivalence class, applies
the CPd precoder and prefilter, uses V.34 Table 13/Figure 10 with V.92's 4T
delays, and emits linear samples after gain G.  The deterministic inverse
undoes the filters and recovers Ki.  An unfiltered two-candidate, 16-state
Viterbi path retains the Figure-10 survivors over the three trellis frames in
each data frame and can correct a deliberately wrong hard decision through
the k=3/Y0 parity constraint.

The loopback suite runs stateful waveform frames at every upstream rate code
1 through 19 (24 000 through 48 000 bit/s), exercises non-zero recursive
precoder/prefilter coefficients, and injects one wrong hard decision into each
of 64 consecutive Viterbi-decoded frames.

`v92_upstream_rx.c` is the live-facing §8.7.1/§9.6.1.1.6 wrapper.  It generates
the exact 48-frame B1u reference from the CPd we send, searches the incoming
linear G.711 stream for that complete sequence, fits gain and DC offset, and
commits framing only after all 48 frames decode to source ones.  Subsequent
frames are decoded and packed into bytes for the data stack.  The modem engine
arms it when a valid CPu fixes CPd, withholds DATA after B1d until B1u locks,
and then reports the negotiated PCM upstream rate.  The synthetic live-wrapper
test acquires B1u after an unaligned prefix at a different gain/offset and
delivers 520 payload bytes exactly.

The B1u wrapper now trains a seven-tap supervised linear equalizer when a
scalar gain/offset fit is insufficient.  A centred three-symbol decision delay
turns fixed fractional sampling phase and symbol-spaced channel ISI into a
causal live filter; after B1u, re-encoding each accepted source frame supplies
the target for decision-directed normalized-LMS updates.  The regression uses
a 0.2-symbol two-tap channel, reaches 0.99999 post-equalizer correlation, runs
960 decision-directed symbol updates, and still delivers all 520 bytes.

This remains experimental on a foreign bearer: the equalizer handles fixed
fractional phase and slowly varying symbol-spaced ISI, but not sample-clock
slips or a fractionally-spaced timing loop.  Longer Viterbi traceback and the
32/64-state trellises also remain.

## Remaining native runtime work

- add sample-clock-slip detection/fractionally-spaced timing recovery ahead
  of B1u, then validate adaptive-equalizer/PTY delivery against a foreign peer;
- extend the initial frame-local 16-state Viterbi path to continuous traceback
  and the Table 30 32/64-state selections;
- derive TRN2u `L_U` from Phase 3/receiver measurements instead of the
  calibration default;
- apply CPus to a complete rate-renegotiation state machine (it is currently
  strictly decoded and logged but ignored during initial Phase 4);
- implement E2u detection as the alternate final acknowledgement path;
- connect fast parameter exchange and modem-on-hold procedures;
- validate the slicer, acknowledgements, Ed/B1d transition, and sustained
  data mode against real V.92 client modems over a transparent G.711 bearer.
