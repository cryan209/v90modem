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
