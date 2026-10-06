# x2 upstream V.34 review, 6 October 2026

The host's 4800 bit/s default is an explicit diagnostic policy in
`me_x2_start_locked()`, not a measured channel ceiling. The Courier initial
record 0344/03FE/0000/0500 advertises higher upstream rates. The environment
variable ME_X2_UPSTREAM_MAX_RATE changes the local mask; the PCM downstream
mask remains separate.

Reviewed against ITU-T V.34 (10/1996), local
`ITU Docs/T-REC-V.34-199610-S!!PDF-E-1.pdf`:

- Section 8.1 / Table 7: at 3200 baud J=7 and P=16. A mapping frame has
  eight 2D symbols, a data frame lasts 40 ms, a superframe lasts 280 ms.
- Section 8.2: at 4800 bit/s N=192 bits per data frame, b=12 and r=16.
  All mapping frames are high frames; no frame switching is necessary.
- Section 9.3.2: b=12 has K=0 and four groups of I1/I2/I3. There are no
  shell bits at this profile. Failure here cannot be blamed on shell unranking.
- Sections 9.6.3 / 9.6.3.2: inversion epoch and convolutional state are
  separate from nearest-point correctness. V0 occurs at each half-data-frame
  boundary; the convolutional encoder has one 4D interval of inherent delay.
- Section 10.1.3.1: B1 is one data frame with the final-superframe inversion
  pattern and initialized scrambler, trellis, differential and precoder state.
- Sections 11.4.1.1.4 and 11.4.1.2.4: data begins a new superframe after B1.
  A good fit to B1 does not prove the decoder epoch or subsequent user bits.
- Section 7 distinguishes GPC and GPA. This x2 implementation uses the
  original Courier's observed upstream convention; do not replace that
  empirically verified convention solely from the ordinary V.34 call role.

The T/3 upstream receiver already enables its carrier loop, timing loop and
0.02 decision-directed equalizer step by default. There is no evidence that
4800 is needed because tracking is absent. Its own distance to the chosen
lattice is insufficient to certify correctness; actual foreign source text
and independently captured transmitted points are the acceptance evidence.

## Fresh 9600 bit/s control

Command:

```
ME_X2_UPSTREAM_MAX_RATE=9600 ../courier-emu/.venv/bin/python \
  tools/probe_x2_host_courier.py --fast --fixed-native-rate --pty-source \
  --capture-native --host-message HOST-X2-0123456789-ABCDEFGHIJKLMNOPQRSTUVWXYZ \
  --message COURIER-X2-0123456789-ABCDEFGHIJKLMNOPQRSTUVWXYZ \
  --output artifacts/x2-v34-review-20261006-9600
```

The selected upstream rate is 9600. Native B1 acquires at 100% fit with
64-state trellis and expanded shaping. The Courier receives the complete
host message, but the engine's upstream PTY contains corrupted source text,
including an intact `DEFGHIJKLMNOPQRSTUVWXYZ` suffix. Early receiver grid
error is approximately 0.015..0.026 with zero shell-invalid frames; those
self-consistent metrics do not establish bit correctness.

Read-only native point comparison at b=24 rejects the initial alignment:
best 256-symbol MSE is 0.119984, above the tool's 0.05 qualification bound.
Do not use this failed alignment to claim a measured number of symbol errors.
The next useful measurement is a qualified native-point alignment through
B1 and the first data frames, then a comparison of transmitted points and
Viterbi decisions around each corrupt text burst. This separates front-end
errors from inverse mapping/epoch errors before changing any loop gains.

The diagnostic default remains 4800. Raising it without a complete native
payload test would advertise a rate that this receiver has not qualified.
