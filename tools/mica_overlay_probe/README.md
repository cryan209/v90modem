# MICA overlay probe

Drives the emulated Cisco MICA (sibling repo `MicaEmu`, with `courier-emu`) as
an answerer with `ATS29=n`, optionally injects a caller, and reports which
overlay loader stubs (C53 program `2BBA`..`2C05`) execute. Findings:
`docs/nonstandard_modulation_inventory.md`.

Run from the `MicaEmu` directory. `ov.py` has no caller; `inj.py` injects a
spandsp V.32bis caller (`v32caller.c`); `v8inj.py` injects a V.8 caller
(`v8caller.c`) that hands over to the V.32bis caller. Build each shim as a
shared library with the vendored `libspandsp.a`, as the makefile does for
`TIFF_CFLAGS`/`SYSTEM_LIBS`. `inj.py`/`v8inj.py` expect `libv32caller.dylib`
and `libv8caller.dylib` next to them. They were written as scratch
scripts: paths are rewritten, not exercised after the move.
