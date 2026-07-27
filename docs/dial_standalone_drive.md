# DIAL running standalone with direct μ-law input

The kernel task dispatcher cannot be easily coerced into calling DIAL
without the full MIPS `dsp_assign` channel-table hookup (the queue protocol
at DM 0x2F00/0x2F08/0x2F09 with free-list 0x2F27 normalises any invalid
queue back to empty). See `docs/dial_sport_drive.md` for that investigation.

But DIAL does **not** need the kernel dispatcher to run. DIAL reads its line
samples directly from the data-pump database registers **DM 0x3F08 and
0x3F09** (the line/RX registers, guide §5.3), and never from the kernel's
0x2E52 timeslot sink. The kernel normally bridges 0x2E52 → 0x3F08/0x3F09 per
frame, but only when a task is assigned. We skip that bridge and feed
0x3F08/0x3F09 ourselves, calling DIAL's two entry points directly:

  - `0x1BBD` — line/input handler (reads 0x3F08/0x3F09, updates state)
  - `0x1B9C` — state dispatcher (reads 0x3FB0, runs the DSP action loop)

Harness: `tools/dial_standalone_drive.py`.

## Result: DIAL is alive

Feeding μ-law into 0x3F08/0x3F09 and calling DIAL's handlers each frame
makes the state machine advance. Critically, **DIAL responds differently to
different inputs** — confirming it is genuinely processing the audio, not
just running an init sequence:

| Input (μ-law into 0x3F08/0x3F09) | DIAL state 0x3FB0 behaviour |
|---|---|
| silence (0x80 idle code) | settles to `0x000c` and stays (2 state changes total — init only) |
| 440 Hz tone | continuous oscillation `0x000c ↔ 0x0000 ↔ 0x0004` (~60 changes/400 frames) |
| 2100 Hz answer tone | distinct pattern: `0x000c` held 13 frames, then `0x0000/0x0004` bursts |

The state values match the recovered state machine (`docs/dial_state_machine.md`):
`0x000c` is a stable "run DSP loop" state, `0x0000`/`0x0004` are the toggling
action pair. DIAL is detecting energy/tone in the input and cycling its
tone-detect state machine accordingly.

## What this means

1. **DIAL runs and processes μ-law audio** without the MIPS firmware, without
   the kernel task dispatcher, and without the TIKRNL assignment. We bypass
   the kernel entirely and call DIAL's handlers as subroutines.
2. **The codec interface is correctly modeled**: μ-law samples into the
   data-pump database line registers (0x3F08/0x3F09) is exactly what the
   ADDSP guide §5.3 documents and what DIAL's line handler (0x1BBD) expects.
3. **DIAL's tone-detection state machine is live**: different audio inputs
   produce different state trajectories, so the DSP filtering and state
   logic are genuinely executing.

## Limitations / next steps

- The kernel's SPORT0 multichannel ISR (PM 0x0072) is bypassed. To use the
  real 32-channel T1/E1 stream, the kernel must bridge 0x2E52 → 0x3F08/0x3F09,
  which needs the channel-table assignment. The standalone path feeds
  0x3F08/0x3F09 directly.
- DIAL writes its TX output somewhere (likely 0x3F08/0x3F09 in the opposite
  direction, or a TX database register) — capturing what DIAL *transmits*
  in response to the tone is the next step to confirm two-way behaviour.
- The latent OOB write in `adsp2181_core.c` still occasionally segfaults
  heavier runs; worth an ASAN pass before longer simulations.
