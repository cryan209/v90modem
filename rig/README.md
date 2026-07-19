# V.90 live interop rig (d-modem / slmodemd)

Files here support the live V.90 interop test against a real SmartLink
softmodem DSP, used to validate the digital-modem server against an actual
analogue-modem peer without physical hardware.

## `d-modem/d-modem.c`

Patched copy of the AonCyberLabs **D-Modem** bridge (`d-modem.c`) that runs in the `d-modem` container. Key changes over upstream:

- **8000↔9600 rational (6/5) resampler.** slmodemd's DSP runs at 9600 Hz;
  the SIP/RTP path is 8000 Hz G.711. The conversion must preserve *both* exact
  8 kHz symbol timing *and* near-Nyquist energy — V.90's `Sd` signal
  (`{+W,+0,+W,-W,-0,-W}`) carries most of its energy at 4 kHz.
  - `dmodem_put_frame` (net→DSP): 257-tap windowed-sinc polyphase interpolator,
    cutoff at the exact 4 kHz input Nyquist, six output phases, with a
    one-frame pipeline delay for inter-frame FIR continuity. The long
    fractional-delay kernel preserves the near-Nyquist content required by
    both Sd detection and SmartLink's Phase-4 TRN2d PDSNR check; the earlier
    12-tap kernel passed Sd but added excessive TRN2d interpolation error.
  - `dmodem_get_frame` (DSP→net): linear interpolation (upstream V.34 is band
    limited well below 3.4 kHz).
- **u-law preferred** codec ordering (the SmartLink blob is a US-market u-law
  build); run the server with `SIP_FORCE_PCMU=1`.
- Debug taps: `/tmp/dm_to_dsp.raw` (9600 Hz, what the DSP receives) and
  `/tmp/dm_from_dsp.raw` (8000 Hz, the DSP's transmit after downsampling).

Also patched (not copied here): `slmodemd/modem_cmdline.c` (`-e` option
declared `MANDATORY,STRING` to fix an upstream arg-parsing bug).

## Required env vars for a successful Phase 3 run

- `ME_V90_J_LOOKAHEAD_BITS=3000` — SmartLink's Phase-3 `Sd`-detection window
  is tighter than the normal (score-gated, 6000-bit-floor) `J`/`Ja`
  confirmation path in `v34rx.c` can meet: measured live, the normal path
  takes ~1.6-1.7s from entering the `J`-wait RX stage to confirming `Ja`,
  which is enough for SmartLink's own Phase3Demodulator to give up waiting
  for our `Sd`-to-`S̄d` transition (~3.2s from when it starts transmitting
  `Ja`) and retrain — the call never gets past "no S after Jd symbols;
  resyncing to WAIT_JA". Setting this to 3000 (~470ms) cuts the same gap to
  ~140ms, which is fast enough that SmartLink reliably detects `Sd` and
  later answers our `Jd` with a real `S`. This is the purpose-built escape
  hatch already documented next to the normal path in `v34rx.c` ("test
  rigs that need latency compensation") — do **not** change the global
  default (`0`, i.e. disabled) for this, since the strict path it bypasses
  exists to avoid a real prior false-positive bug (mid-TRN score-25/32
  matches launching `Sd` early) against real hardware modems.
- `ME_V90_JD_RESYNC_SYMBOLS=24000` — still required even with the lookahead
  fix above. SmartLink answers our `Jd` with `S` at ~19296 symbols (~2.4s)
  into the `Jd` wait; the 12000-symbol (1.5s) default fires first and resets
  us to `WAIT_JA` before that `S` ever arrives, undoing the lookahead fix.
  24000 (3s) gives headroom above the observed 19296 without being
  needlessly long.

With both set, a call reliably reaches `Sd`→`S̄d`→`TRN1d`→`Jd`→(`S`
detected)→`J'd`→ DIL, and cycles DIL segments steadily (confirmed
reproducible across repeated live calls). The shutdown-time SIGSEGV
(`process_return_code: -11` in `manifest.json`, whenever the interop
harness's SIGINT lands after a call) is a separate, still-open bug, not
caused by these env vars.
