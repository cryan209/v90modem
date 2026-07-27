# Channel descriptor: corrected understanding

## The 0x2F01/0x2800 was a red herring

After the OOB fix, I traced the dispatcher in detail and confirmed the
channel-table descriptor format is **not** what I thought. Key findings:

1. **No overlay writes 0x2F00-0x2F02** (the "per-frame descriptor" with the
   0x2800 inactive vector). Verified by disasm grep across kernel, TIKRNL,
   DIAL, and V.8. The 0x2800 at 0x2F01 is baked into the kernel's **static
   DM image** — it's the default, never overwritten by any task.

2. **The 0x2F00 region is the host-request service queue**, not the task
   dispatch table. The dispatcher's `CALL (I4)` at 0x02A4 vectors through
   the free-list at 0x2F27 for *host requests*, not for task dispatch.

3. **TIKRNL's init (PM 0x672 → 0x184D) writes service vectors to DM
   0x3307-0x3309**:
   ```
   184e: I0 = $3307
   1850: DM(I0,M1) = $00FF    ; 0x3307 = 0x00FF (task-present flag)
   1851: DM(I0,M1) = $05B1    ; 0x3308 = 0x05B1 (TIKRNL service vector)
   1852: DM(I0,M0) = $05BE    ; 0x3309 = 0x05BE
   ```
   These are TIKRNL's *own* self-pointers (its frame handler at 0x1810 reads
   `I0 = DM($3309)`), not pointers the kernel reads.

4. **The kernel never references 0x3300-0x33FF** (TIKRNL's mailbox area) —
   zero direct DM refs in the entire kernel image. The kernel→TIKRNL
   connection is via the SPORT0 ISR's TDM walk, not a direct pointer read.

## The real dispatch: the SPORT0 ISR TDM walk

The kernel's SPORT0 RX ISR (PM 0x0072) is the TDM timeslot state machine:

```
0089: I4 = DM($2E44)        ; channel-table head (-> 0x2E00 timeslot buffer)
008a: MAC with DM read DAG2 ; walk the timeslot buffer
008b: IF EQ JUMP $0091       ; buffer exhausted -> per-frame completion
...
0091: I4 = $6E40             ; per-frame completion vector
0094-009b: compute I4 from the descriptor walk
009b: I4 = AR                ; task vector
```

- DM 0x2E40-0x2E43 is the **timeslot buffer descriptor** (0x2E40=0x2E90,
  0x2E41=0x0080, 0x2E42=0x2E80, 0x2E43=0x007F — buffer pointer/length
  pairs), NOT the task dispatch table.
- DM 0x2E44/0x2E45 head/tail walk the 64-entry G.711 timeslot buffer
  (0x2E00-0x2E3F).
- The task dispatch (CALL into TIKRNL) happens at **per-frame completion**
  (0x0091+), vectors through 0x6E40 and the computed I4.

So the channel-table "descriptor" I was trying to plant at 0x2F00 was the
wrong structure. The actual TIKRNL dispatch is via the ISR's per-frame
completion path, which requires the timeslot buffer walk to reach a frame
boundary.

## Why TIKRNL never runs

The ISR's per-frame completion (0x0091+) computes the task vector from
state that TIKRNL's init was supposed to set up. TIKRNL's init (0x672)
*does* run (we call it directly), and it writes 0x3307-0x3309. But the
ISR's per-frame path doesn't read 0x3307 — it reads the channel-table
descriptor at 0x2E40 and computes I4 from the timeslot walk.

The missing piece is the **channel-table entry that links a timeslot to
TIKRNL**. On real hardware, `dsp_assign` writes this entry: it sets a
timeslot's descriptor to point at TIKRNL's service vector. The shim's
`--assign` commits the command ring but the DSP never consumes it
(consumer 0x3316 stuck at 0x3327), so the channel-table entry is never
written.

## What's confirmed

1. **OOB fix is solid** — stable under heavy IRQE+SPORT0 load.
2. **The 0x2F00 "descriptor" was the host-request queue, not the task
   dispatch table** — the 0x2800 is a static default, never overwritten.
3. **TIKRNL's init writes 0x3307-0x3309** (service vectors 0x05B1/0x05BE).
4. **The real dispatch is the ISR per-frame completion** at 0x0091+,
   computing I4 from the timeslot-buffer walk.
5. **The blocker remains the command-ring consumer** — TIKRNL must consume
   the command at PM 0x3327 (advance 0x3316) to register itself in the
   channel table, but it only runs when dispatched, and dispatch needs
   the registration. Circular.

## The actual remaining piece

The circular dependency breaks on real hardware because the MIPS firmware
**loads the task overlay and runs its init before expecting dispatch**.
The task init (TIKRNL's 0x672, which we DO call) sets up 0x3307-0x3309,
but it doesn't write the channel-table entry — that's done by the
**command-ring consumption** (TIKRNL's frame handler reading 0x3308→command
ring→register task).

To break the circle without the full task-load: call TIKRNL's frame
handler (0x64A or the 0x1810 path) directly after init, with 0x3308/0x3309
set up, so it consumes the command ring and writes the channel-table entry.
This is the next concrete step — TIKRNL's frame handler is the command-ring
consumer.
