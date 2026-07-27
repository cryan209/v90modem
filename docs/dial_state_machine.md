# DIAL bootpage state machine (Eicon build 117-926, overlay 0x0262 DIAL/FSK/FAX.F34)

Recovered by driving the ADSP-2181 emulator: boot the PRI kernel to IDLE,
overlay the DIAL PM/DM image, poke a candidate value into the DIAL state word
DM `0x3FB0`, enter the dispatcher at PM `0x1B9C`, and trace the conditional
branches until the code settles into a loop, goes idle, or falls through.
Source: `tools/dial_state_map.py`.

## Entry and dispatch

DIAL is **bootpage 0** of the ADDSP V.90 package (`docs/addspv90guide.pdf`
§2.4, Table 1) — the "idle page" providing ring detection, DTMF/tone generation
and the country-specific autodialler. It is an *overlay*: it owns PM
`0x08f0–0x2400` and DM `0x1240–0x3fb4` and has **zero conflicts** with the
resident PRI kernel, which keeps the low PM/DM and the reset vector.

> **Correction.** Everything below that treats `FLAG_IN` as the kernel's task
> strobe is wrong, and it was a disassembler bug, not a firmware feature: the
> ADSP condition field 15 is the *unconditional* encoding, and
> `tools/adsp2181_dis.py` had it labelled `NOT FLAG_IN`. Nothing in these
> images tests FLAG_IN. What actually dispatches DIAL is the TIKRNL task —
> `docs/dial_under_tikrnl.md` for the overlay stubs, and
> `docs/dial_kernel_dispatch.md` for the kernel calling the task off SPORT0.
> The state-machine findings themselves are unaffected.

TIKRNL enters DIAL through the stubs at the top of the overlay:

```
08f0: JUMP $1B9C   ; primary DIAL dispatcher
08f1: JUMP $1BBD   ; secondary path (line-signal handler)
```

`0x1B9C` is the **state dispatcher**; `0x1BBD` is the **line/input handler**
(it reads DM `0x3F08`/`0x3F09`, the data-pump RX/line registers from guide
§5.3).

## The state word and dispatcher (0x1B9C)

DM `0x3FB0` is the DIAL **state/selector**. The dispatcher reads it and runs a
compare-and-branch ladder. Each compare is a `22e20f` (conditional ALU-to-AR,
condition `0xF`=TRUE, so the ALU always runs and sets ASTAT) followed by an
`IF EQ/NE JUMP` that tests the result:

```
1b9c: AX1 = 0
1b9d: AY0 = DM($3FB0)        ; load current state
1b9e: AR = $000B            ; compare...
1b9f: AR = AR - AY0 (cond)  ; sets AZ/EQ if state==0x0B
1ba0: AR = 0
1ba2: IF EQ JUMP $1BAF       ; state == 0x0B -> state-update exit
1ba3: AR = $000F
1ba8: IF EQ JUMP $1BAF       ; state == 0x0F -> state-update exit
1ba9: AR = $0010
1bab: IF NE JUMP $1DA7       ; state != 0x10 -> action dispatch (0x1DA7)
1bac: AR = $0004
1bae: AX1 = $0028            ; (state==0x10 path) set AX1=0x28
1baf: DM($3FB0) = AR         ; write NEW state
```

So states `0x0B` and `0x0F` exit via the *state-update* path (0x1BAF writes a
new state and sets vector pointers), and all other states fall through to the
*action dispatch* at `0x1DA7`.

## Action dispatch (0x1DA7) and the DSP inner loops

```
1da7: AY0 = DM($3FB0)
1daa: AX0 = DM($127B)        ; persistent state/flag
1dab: DM($127B) = ASTAT      ; save flags (ring/tone detect latches)
1dac: IF NE JUMP $1DB2       ; branch on saved compare
1dae: IF EQ JUMP $1BCE       ; -> line-signal handler (0x1BCE)
1db1: JUMP $1DBC
...
1dbc: DM($3FB0) = AX1        ; commit new state
1dbd: I4 = $9400            ; vector-table base for the chosen action
1dbe: JUMP $1DDA   ; -> DSP inner loop A
```

From `0x1DDA` and `0x1DCB` DIAL runs its actual signal-processing kernels.
All three loops are MAC/ALU over DM circular buffers via the DAGs — the tone
generation / ring-detect / DTMF filtering:

| Loop PC | What it does |
|---|---|
| `0x1DDA` | MAC inner loop: `ALU->AF + DM write`, `MAC->MR + DM read`, `IF NE JUMP $1DBF`. Continuously walks a DM buffer doing multiply-accumulate (tone generation / filter). |
| `0x1DCB`/`0x1DCD` | Shift + ALU inner loop with PX (pixel/byte pack) and CNTR-driven block repeats — block DSP work (DTMF/coefficient update). Exits on `JUMP $1DDA`. |
| `0x0756` | **Fall-through to empty NOPs** — states 1/2/3 with no configured action return through the `IF EQ RTS` stub table at `0x0900+` and land here (effectively a no-op return to the kernel). |

Every loop re-enters on `JUMP $1DDA`; DIAL leaves this work only by returning
to whoever called the stub, i.e. once per task slot.

## Vector pointers

The dispatcher also maintains two code-vector pointers in the data-pump
database:

| DM addr | Initial (state 0) | After first dispatch | Role |
|---|---|---|---|
| `0x3FB2` | `0x17BB` | `0x1BCE` | primary action vector (I4 dispatch target) |
| `0x3FB3` | `0x1706` | `0x1C2E` | secondary action vector |

`0x17BB`/`0x1706` are the cold-start vectors (they `CALL` the ring/tone setup
routines `0x175E`/`0x166B`/`0x17C5`); `0x1BCE`/`0x1C2E` are the steady-state
line-signal handlers. The transition happens on the first state update.

## Recovered state transitions

Only these states *change* the state word; all others hold their value and
just run the DSP loop:

```
0x0000  -> 0x000B      cold start: idle -> "ready" (sets vec 3fb2/3fb3)
0x000B  -> 0x0000      state-update exit (toggles back to idle)
0x0004  -> 0x0010      action dispatch
0x0010  -> 0x0004      state-update exit (toggles back)
0x000F  -> 0x0011      state-update exit
0x0011  -> 0x000F      toggles back
```

So the live state machine is three toggling pairs plus a cold-start hop:

```
            0x00 ──(cold)──> 0x0B <──> 0x00      (idle <-> ready)
            0x04 <──────────> 0x10               (action A toggle)
            0x0F <──────────> 0x11               (action B toggle)
```

All other values (0x01–0x03, 0x05–0x0A, 0x0C–0x0E, 0x12–0x1F) are **stable**:
they hold the state, run the DSP inner loop at `0x1DCD`, and wait to be called
again. This is consistent with DIAL being the idle page — most states are
"keep doing the current DSP work, check again next slot."

## What this confirms

1. **DIAL is a real, running modem idle page**, not a stub. It reads/writes the
   data-pump database (DM `0x3F08/0x3F09/0x3FB0/0x3FB2/0x3FB3/0x3FC1/0x3EE1`),
   exactly the registers the ADDSP guide §5.3 documents, and runs continuous
   MAC/ALU DSP kernels over DM buffers.
2. **It is driven by being called**, once per slot, through the PM
   `0x08F0`/`0x08F1` stubs — by TIKRNL, not by a host doorbell and not by any
   flag. To make DIAL do something observable, the task has to be dispatched
   and DIAL's inputs (DM `0x3F08/0x3F09` — the line/RX registers, and
   DM `0x3EE1` — GEN_SETUP1) driven.
3. **The state machine is small and fully recovered**: three toggling pairs
   plus a cold-start, with the steady state being "hold and run the DSP loop."
   DIAL does not advance state on its own beyond these toggles — state
   progression comes from the line-signal handler (`0x1BBD`/`0x1BCE`) writing
   `0x3FB0` in response to detected tones/ring.

Next step to see real behaviour: wire SPORT0 PCM input into DM
`0x3F08/0x3F09` and get the task dispatched so `0x1BBD` (the line handler) can
detect ring/tone and advance the state machine. Both halves of that are done —
`docs/dial_under_tikrnl.md` (task-driven) and `docs/dial_kernel_dispatch.md`
(kernel-driven off SPORT0).
