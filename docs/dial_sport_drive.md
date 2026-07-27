# Driving DIAL with a μ-law T1/E1 stream on SPORT0

Experiment: feed a μ-law PCM stream into the ADSP-2181 emulator's SPORT0
(port 0 RX callback), 32 timeslots per 8 kHz frame, with the PRI kernel
booted and the DIAL bootpage (overlay 0x0262) layered on top, strobing the
SPORT0_RX interrupt once per timeslot. Goal: see DIAL react via its line
registers (DM 0x3F08/0x3F09) and state word (DM 0x3FB0).

Harness: `tools/dial_sport_drive.py`.

## Codec / serial interface (confirmed from the guides)

- **SPORT0 multichannel mode**, 32-word TDM frame, μ-law companding
  (`docs/addspv90guide.pdf` §3.3: "SPORT0 is programmed for multichannel
  operation and u-law or A-law companding").
- The 5 SPORT0 multichannel control registers live in the **data-pump write
  database** (guide §5.3.1), base 0x3EE0:
  - `0x3F50` Sp0CntrlReg, `0x3F51` Sp0MCRecL, `0x3F52` Sp0MCRecM,
    `0x3F53` Sp0MCTXL, `0x3F54` Sp0MCTXM (32-bit channel enable masks, low/high).
  - These must be initialised "after system reset before the start-up page is
    loaded."
- The DSP exchanges analogue samples with the line interface through SPORT0
  (guide §3.1). The AD1843 codec (`docs/ad1847.pdf`) is the SoundPort codec
  used for PSTN; T1/E1 uses the multichannel mode above.

## Kernel SPORT0 RX ISR (PM 0x0072)

The resident kernel handles the TDM frame. Its ISR (vector at PM 0x0014 →
0x0072) reads the SPORT0 receive register and walks timeslots:

```
0072: MSTAT = $C061
0073: AR = DM($3FF9)        ; ADSP-2181 system reg
0074: AY0 = DM($3FFA)
0076: AY1 = DM($2E50)       ; per-channel substate countdown
0078: IF GE JUMP $0081
0079: ? = DM($2E52)         ; current PCM code
007f: DM($2E52) = RX0       ; <-- SPORT0 receive sample stored here
0084: DM($2E4A) = I4        ; save/restore DAG across the frame
0089: I4 = DM($2E44)       ; channel-table pointer
...
00bd: DM($2E52) = AR
```

So **RX0 → DM 0x2E52** is the kernel's per-timeslot PCM sink, and DM 0x2E50
is the substate countdown that advances through the 32 slots. The ISR runs
on every SPORT0_RX strobe. Verified: with the stream fed, the ISR executes,
DM 0x2E52 updates, and SPORT0 TX emits 1600 words over 50 frames.

## Result: DIAL is NOT reached

Feeding 50 frames of 440 Hz μ-law on timeslot 0 (with SPORT0 multichannel
regs and GEN_SETUP programmed) produces:

```
frame    0: 2E52=0000 3F08=0000 3F09=0000 3FB0=0000 (pc=02a9)
...no further changes; DIAL regs stay 0; kernel stays at pc=02a9 (IDLE)
```

The kernel's SPORT0 ISR absorbs the stream into the 0x2E00 timeslot buffer,
but the **foreground never dispatches to DIAL**. DIAL's registers (0x3F08,
0x3F09, 0x3FB0) never change.

## Why: the channel/task table is unassigned

The kernel foreground idle loop (PM 0x02a8):

```
02a8: IDLE
02a9: AY0 = DM($2E45)       ; channel-table tail pointer
02aa: AR = DM($2E44)        ; channel-table head pointer
02ab: AR = AR - AY0
02ac: IF EQ JUMP $02A8       ; head==tail -> queue empty, idle again
02ad: ...                    ; else: rebuild free list, dispatch
02a1: IF NOT FLAG_IN CALL $01B2   ; service routine
02a3: I4 = SR0               ; vector from service
02a4: IF EQ CALL (I4)         ; dispatch to task
```

At boot, **DM 0x2E44 == DM 0x2E45 == 0x2E00** — the queue-empty condition.
The kernel never dispatches because no task descriptor is linked into the
channel table. DM 0x2E00–0x2E3F is just the 64-entry G.711 timeslot buffer
(all `0x00ff` idle). DM 0x2E40+ holds channel descriptors but 0x2E4A–0x2E4C
(DAG save slots the dispatcher uses) are zero, and the service slots at
0x2F27–0x2F2B point at inactive entries (`0x2800`).

This is exactly the **TIKRNL `dsp_assign` gap** documented in
`docs/eicon_adsp_firmware_analysis.md`: channel activation requires the
MIPS-side `dsp_assign` initial-database commit, which hooks a channel-table
entry to the modem task. Without it, the kernel's `CALL (I4)` dispatcher has
no task to call, so DIAL is never entered — regardless of what's on SPORT0.

## Conclusion

Driving μ-law directly into SPORT0 is **necessary but not sufficient**. The
ADSP-2181 codec interface is correctly understood (SPORT0 multichannel,
μ-law, 32 timeslots, control regs at 0x3F50–0x3F54, RX0→DM 0x2E52 in the
kernel ISR), and the harness feeds it correctly. But the kernel's task
scheduler will not call DIAL until a task descriptor is planted in the
channel table (DM 0x2E40+, linked via 0x2E44/0x2E45) pointing at DIAL's
entry vector.

Two ways forward:
1. **Manually plant a minimal channel-table entry** that makes the kernel's
   `CALL (I4)` at 0x02a4 vector into DIAL's dispatcher (PM 0x08f0 or the
   0x1B9C state dispatcher), bypassing the MIPS `dsp_assign` entirely.
   This requires reverse-engineering the channel-descriptor format at
   0x2E40+ (fields the dispatcher reads: I4 vector, DAG context, state).
2. **Fix the latent OOB write** in `adsp2181_core.c` first (the 10-decl
   ctypes path segfaults; the ISR-heavy run also segfaults), then run the
   MIPS `dsp_assign` in the shim to do the real assignment.

Option 1 is the "skip MIPS" path the user wants; the missing piece is the
channel-descriptor layout, which the dispatcher reads at 0x02a1–0x02a4.
