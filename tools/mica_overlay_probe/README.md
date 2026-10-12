# MICA overlay probe

Drives the emulated Cisco MICA (sibling repo `MicaEmu`, with `courier-emu`) as
an answerer and reports what `ATS29=n` changes. Findings and the corrected
method: `docs/nonstandard_modulation_inventory.md`.

Run from the `MicaEmu` directory, with `libv32caller.dylib` and
`libv8caller.dylib` (build `v32caller.c`/`v8caller.c` against the vendored
`libspandsp.a`, as the makefile does) next to the scripts.

- `ov.py` S29 N: no caller; which overlay loader stubs execute.
- `call3.py` S29 MODS SECONDS V8BIS START: answerer vs a SpanDSP caller.
  MODS non-zero runs a V.8 caller offering those V.8 bits and hands over to
  V.32bis on success; MODS 0 starts the V.32bis caller at START seconds.
  `V32BIS_TRACE=1` prints the decoded rate-signal words.
- `cfg.py` S29 N: DSP data memory N instructions after the answer command,
  pickled, for diffing settings.

Framing that matters (`mica_line.cpp`): answer with `dsp_command(0, 0xA0, [0])`;
channel 0 is the even lane of `g711_tx()`; feed each received sample into both
lanes; hand 160-octet blocks over between 100,000-instruction slices. Scripts
moved from scratch space keep rewritten paths and were not re-run afterwards.

`demod/` reads the G.711 files `call3.py` saves (`mono_<S29>_<EXT>.bin`):
`ext_decode.py FILE SECONDS` finds the TRN, reports where it ends and decodes
the 16-bit words after it at all eight alignments. `dem.py` is the front end,
`words.py`/`trn.py` the state and word layers. Pure Python, no numpy; about a
minute per 4 s of audio. The state order is the mirrored `[0,2,3,1]`.
