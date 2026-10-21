# V.FC implementation and evidence

22 October 2026. **V.FC calls are not implemented.** `AT+MS=VFC,1`
continues to offer V.32bis; `AT+MS=VFC,0` continues to return ERROR.
Signature bits, frame/shell packing and the lattice trellis receiver now join
the verified mapper. These are coding components, not a complete datapump.
Two fresh original-firmware peer calls now reach 28,800/VFC and deliver both
markers; the earlier startup blocker was our probe's V.8 configuration.

## Working reference: disable V.8 for proprietary startup

**S56=64 does not disable V.8.** In our initial probes S54 remained 64,
allowing the V.8 path to take priority over the proprietary ROCK exchange.
The native `aa18..aa42` signature decoder refuses the proprietary transition
when both `03a6` bit 0 and `03af` bit 12 are set. A native RX capture showed
those bits set in the failing configuration. This was not evidence of a peer
defect.

Setting **S54=192 on both peers**, with all other settings unchanged, produces
`CONNECT 28800/VFC/NONE` on both native DTEs and delivers their distinct
markers in both directions. Both load overlay 7 at `b000`. A fresh repeat
with read-only state capture also passes. The negotiated state is baud index
5 (3429), rate code 6 (28800), 32-state trellis, four uncoded bits, maximum ring
index 5, and fractional 67/68-bit frames averaging 67.2 bits per eight symbols.

Retained evidence:
`reference-data/artifacts/vfc-no-v8-20261022/` and
`reference-data/artifacts/vfc-native-qualified-start-20261022/`.
These are original-firmware baseline calls, **not interoperability of our C
implementation**, long checked payload traffic or physical-hardware results.
The probe now defaults to S54=192. `--inherit-v8` retains the original negative
control; `--enable-v8` sets S54=0 for the fully enabled comparison.

## Implemented: signature and frame coding

`vfc_signature.c` packs the raw 64-bit proprietary identifier: `ROCK` followed
by four equal bytes, LSB-first dibits. It recognizes the native receive window
after all-ones acquisition, implements V.8 priority and the four-bit receive
field, and latches a single successful transition. Native `aa35..aa40` with
SPM=1 accepts codes 0..3 and yields timer counts 2700, 2316, 1932, 1548. These
are firmware timer counts, not milliseconds. Repeated bytes with nonzero upper
nibbles are deliberately accepted when their low nibble passes, as in the
original decoder. Sixteen TX packets and 1,280 RX cases execute the original
bit routines, including every repeated byte, priority combination and every
single header/body bit corruption for valid codes.

`vfc_frame.c` reproduces `c1bf..c20d`'s rational frame geometry and
`c88a..c89b`'s fractional schedule, without rounding 2743/3429 baud. The `cb47`
ring-bound table has **eight** entries indexed by residual shell bits; it is
not one of the six-entry symbol-rate tables. All 108 rate/baud/ring-option
plans are exercised against native execution. Seventy-eight fit the existing
1,280-point constellation; this is a geometry bound, not a promise that every
combination can be negotiated on a channel.

The same module implements `c88a..c98c`'s interleaved direct/shell bits and
recursive shell enumeration, and its inverse. The count polynomials parallel
V.34 9.6.3 equations (9-8)..(9-24), with V.FC's inclusive ring bound and order
verified independently. All **312 original-firmware frames** match in both
directions, including zero/all-one inputs, random bits, 67/68-bit scheduling
and wraps of the native 128-bit bit buffer. Inverse checks consume the native
ring/direct outputs, not our own encoder output.

## Implemented: lattice trellis reception

`vfc_trellis.c` uses the verified 2D branch geometry for both the 8- and
32-state codes. It accepts equalized points after inverse precoding/scaling,
delivers differential dibits and uncoded indices with a 63-symbol traceback
delay, and drains only existing survivor decisions at stream end. It does not
insert waveform samples. Init resets all per-call state; NaN/Inf input is
rejected without advancing the stream.

Two continuous original-firmware streams provide **8,192 symbols** from known
zero coding states and 96 point indices (the observed 28,800 geometry). They
decode exactly, both unmodified and with bounded 0.1-lattice-unit coordinate
perturbations. The latter is a metric-handling check, not a channel-noise or
hardware acquisition qualification.

Fixtures and generators: `signature-*.csv` / `tools/vfc_signature_vectors.py`,
`frame.csv`, `frames.csv` / `tools/vfc_frame_vectors.py`, and
`trellis-stream.csv` / `tools/vfc_firmware_vectors.py --streams`, under
`reference-data/vfc/`. All generators execute original instructions; they do
not call the C implementation to generate expectations. Focused tests are
registered in `tests/fast.list` and build with `make`.

All five focused test rows pass through the parallel runner. The frame codec,
signature and trellis tests also pass undefined-behaviour sanitizer builds.
The full fast/slow tiers have not been run for these isolated coding changes.

## Implemented: symbol mapper

`vfc_mapper.c` implements original Courier 403 firmware overlay 7,
program `c749..c788`, before precoding and transmit scaling:

- quadrant differential accumulation modulo four;
- the `cb2d` eight-subset permutation, using the old trellis state and new
  quadrant;
- the 320-entry resident `aeab..afea` quarter-constellation and its four
  rotations (1,280 points in all);
- selectable 8-state (`cb5e`) and 32-state (`cb57`) nonlinear trellis
  transitions, one transition per two-dimensional symbol.

The input is a frame mapper's uncoded point index, 0..159, and a scrambled
dibit, 0..3. The output is an integer lattice point, before the precoder,
nonlinear encoder, amplitude scaling, pulse shaping and carrier modulation.
Both code choices keep the firmware's five-bit state representation; the
8-state code does not use contiguous state numbers 0..7.
Initialization resets all state. Invalid arguments change neither state nor
output. There is no process-lifetime static call state.

V.FC is proprietary and has no ITU Recommendation clause to cite. Addresses
above identify the actual rules; **ITU V.34 9.6.3 is not their specification**.
The V.34 superconstellation has a different point ordering and its 4D trellis
cannot replace either V.FC code. The sibling
`../courier-emu/docs/vfc.md` documents the original analysis; its
`docs/datapump-slots.md` identifies V.FC's overlay and entry point.

## Independent regression evidence

`reference-data/vfc/mapper.csv` contains **3,584 vectors executed by the
original DSP program**, not generated by the C implementation under test:

- every encoded state, prior quadrant and input dibit for both codes;
- every point index, quadrant rotation and subset offset for both codes.

The fixture and `vfc_mapper_tables.h` regenerate byte-for-byte. Firmware ROM:
`f1c91621fbf14aad34547056feb4425e64fcf0b38ec5d57f86c9f432ca999db6`.
The manifest records the emulator commit, dirty-worktree status and native
library hash. The sibling emulator currently has local changes, so its commit
alone does not identify the execution environment.

```sh
../courier-emu/.venv/bin/python tools/vfc_firmware_vectors.py \
  --emulator ../courier-emu \
  --rom ../courier-emu/dumps/courier-board-21210-capture-403/courier-board.rom \
  --library ../courier-emu/.build/libcourier_c5x.dylib \
  --output artifacts/vfc-regenerated
make vfc_mapper_test
python3 tools/run_tests.py --filter vfc_mapper tests/fast.list
```

The generator checks instruction signatures before entering the mapper. It
loads the original resident and V.FC overlay, supplies an isolated input frame
word and state, executes `c749..c788`, and observes the point and next state at
`c789`. It does not run training or claim sample-clock equivalence.

Validation: `make vfc_mapper_test`, the fast-list row through the test runner,
and a standalone undefined-behaviour sanitizer build all pass. The full fast
and slow tiers were not run for this isolated component. AddressSanitizer's
runtime hung before producing output on this host, including outside the
sandbox; those attempts were interrupted, not counted as passes.

## Earlier failed firmware baseline (superseded by S54=192)

`tools/probe_vfc_firmware.py` connects two original analogue Courier 403 ROMs
through the emulator's subscriber-loop model. This carries audio and normal
hook/ring supervision; no host-supplied carrier result or training response is
enabled. The original probe cleared V.FC's disable bit, set S56=64 to disable V.34,
sets S58=33 to disable x2/V.90, and requests &N14 (28,800 bit/s). &M0 removes
error control while identifying the modulation. No NVRAM save is issued.

```sh
../courier-emu/.venv/bin/python tools/probe_vfc_firmware.py \
  --emulator ../courier-emu --python ../courier-emu/.venv/bin/python \
  --rom ../courier-emu/dumps/courier-board-21210-capture-403/courier-board.rom \
  --library ../courier-emu/.build/libcourier_c5x.dylib \
  --output artifacts/vfc-native-new
```

The probe requires **both CONNECT reports to say `/VFC`**, with both distinct
markers received. These short markers are not a checked long payload stream.
It deliberately fails if a working call is V.32bis. Its fixed 14,400 control
(`--rate 14400`, with the original S54=64 settings) did exactly that: both native
terminals reported `CONNECT 14400/V32/NONE` and delivered their opposite marker.
The original 28,800 run did not reach CONNECT and never requested overlay 7. Enabling V.8 with
`--enable-v8` also does not load overlay 7 in the measured window. These are
completed 300-million-instruction runs; no modem CPU/DSP execution error was
reported. The result does not identify a peer defect or prove V.FC impossible.

Retained local captures:
`reference-data/artifacts/vfc-firmware-native-s27-20261022/`,
`reference-data/artifacts/vfc-firmware-v32-control-20261022/` and
`reference-data/artifacts/vfc-firmware-v8-20261022/`.
They are not regression dependencies. An &N0 automode comparison also
delivered both markers, but at `19200/V32/NONE`, again without loading
overlay 7; it is a V.32terbo observation, not V.FC. Combining &N0 with
S54=0 fell back to 2,400 bit/s and corrupted one short marker. Neither
comparison entered overlay 7. Their retained captures are
`reference-data/artifacts/vfc-firmware-automode-20261022/` and
`reference-data/artifacts/vfc-firmware-auto-v8-20261022/`.

The final probe preserves the requested venv executable symlink. Fresh
`reference-data/artifacts/vfc-firmware-final-20261022/` and
`reference-data/artifacts/vfc-firmware-final-control-20261022/` repeat the
same no-carrier 28,800 attempt and successful V.32bis 14,400 control with
that interpreter selection.
Earlier I-modem runs also connected only at V.32bis, or failed to train.
The first sandboxed attempt failed to bind a local socket and is not protocol
evidence. Audio-only dialled-pair attempts omitted the required ring
supervision and likewise do not establish a V.FC failure.

## Remaining work

1. Recover the remaining startup messages, field layouts, CRCs, timing and negotiated
   symbol-rate/carrier selection. Do not invent these from V.34.
2. Implement the sample receiver, precoding/nonlinear inverses, pulse shaping and
   training at the byte-exact 8 kHz bearer boundary.
3. Wire the complete datapump into the engine and AT offer only after
   independent firmware-peer payload tests pass. Then qualify real hardware,
   both PCM laws, repeated calls, fallback, retraining and rate changes.

Do not describe the isolated mapper test or any V.32bis CONNECT as V.FC
interoperability.
