# DIAL running under TIKRNL — the card's real dispatch chain

`docs/dial_standalone_drive.md` showed DIAL processes audio, but that harness
calls DIAL's internals (PM `0x1B9C` / `0x1BBD`) as subroutines with the DIAL
overlay layered straight onto the PRI kernel. That is not how the card runs
it, and it hides two things that turn out to matter.

Harness: `tools/dial_tikrnl_drive.py`.

## The chain is kernel → task → overlay, not kernel → overlay

| Layer | Download | PM extent | DM extent |
|---|---|---|---|
| PRI 30M kernel | `0x0009` | `0x0000-0x05EB` | `0x2E00-0x2F6A` |
| TIKRNL81.F34 **task** | `0x0258` | `0x0580-0x1902` | `0x3180-0x3FDF` |
| DIAL/FSK/FAX.F34 **overlay** | `0x0262` | `0x08F0-0x23FF` | `0x1240-0x3FB3` |

The task deliberately overwrites the kernel's PM `0x0580-0x05EB` stub block
(108 words). The overlay collides with the task in exactly ten words: PM
`0x08F0`/`0x08F1` and DM `0x32F0-0x32F4`, `0x3EEB`, `0x3FB2`, `0x3FB3` — the
shared task/overlay interface (download segments 4 and 5, DM base `0x32F0`
size 8 and PM base `0x08F0` size 7, identical in both files).

TIKRNL ships PM `0x08F0`/`0x08F1` as `0a000f` (`RTS`, `RTS`) — "no overlay
loaded, do nothing". The DIAL download replaces them with

```
08f0: 19b9cf  IF NOT FLAG_IN JUMP $1B9C   ; state dispatcher
08f1: 19bbdf  IF NOT FLAG_IN JUMP $1BBD   ; line/RX handler
```

`FLAG_IN` is deasserted on this card, so every `IF NOT FLAG_IN JUMP/CALL` in
these images is an unconditional transfer — the whole firmware is written that
way.

## Download order is load-bearing

TIKRNL's init (PM `0x0637`, reached from the task entry) is a PM clear loop:

```
0637: AR = $0000
0639: I4 = $8900       ; PM 0x0900
063a: CNTR = $CF00
063b: <clear loop>
063d: I4 = $9B80       ; PM 0x1B80
063e: CNTR = $A480
```

It wipes PM `0x0900` upward — the entire overlay region. **Layering DIAL
before the task init silently erases most of DIAL**, which is why a
kernel+DIAL image behaves nothing like the card. The overlay has to be
downloaded after the task has initialised, which is also the order the host
driver uses.

## The task entry registers a per-frame continuation

The task's download symbol 0 is PM `0x0672`. Calling it (with the kernel
booted and its idle loop at `0x02A8` as the return) runs the init, then at
`0x0699-0x069C`:

```
0699: AR = $06FC       ; the per-frame continuation
069a: SR1 = $08F6      ; the message-send helper
069c: CALL $0017       ; kernel service: patch a vector slot
```

Kernel service `0x0017` (PM `0x0281`, helper at `0x0294`) *builds an
instruction* — `SR1 = $1C00`, `SR0 = $0F00`, shift the vector in, `PX = SR0`,
then `PM(I4) = SR1:PX` — and patches it into the kernel's vector table.
Diffing PM across the task entry shows exactly two words change:

```
PM 0000: 18580f -> 1c8f6f    JUMP $0580  ->  CALL $08F6
PM 000a: 181def -> 1c6fcf    JUMP $01DE  ->  CALL $06FC
```

The kernel's reset/vector table doubles as its service jump table: the real
interrupt vectors (`0x0004`, `0x0008`, `0x000C`, `0x0018`, …) are `RTI`, and
the gaps (`0x0001-0x0003`, `0x000A-0x000B`, `0x000E`, `0x0015`, `0x0017`,
`0x0019`, `0x001E`) are service entries a task may claim.

## TIKRNL's frame loop is what calls DIAL

PM `0x06BB-0x06EE`:

```
06bb  CALL $0002        ; kernel queue service (host command fetch)
06c0  IF EQ CALL (I4)   ; dispatch the queued command
06c2  CALL $064A        ; frame housekeeping
06c7  AR = DM($3F08)    ; the line/RX register
06ce  DM($3F09) = AR    ; OR 0x1000 into the second line register
06d0  DM($3F05) = $FFFF
06d1  CALL $08F1        ; -> DIAL line handler (0x1BBD)
06d5  IF NE JUMP $0686  ; DM(0x3FC1) & 0x0100 -> back to the bootpage check
06e0  AR = DM($3FC1) AND $FEFF   ; clear the strobe
06e2  CALL $08F0        ; -> DIAL state dispatcher (0x1B9C)
06e7  I4 = DM($3FB2)    ; -> DIAL's own action vector
06e8  CALL (I4)
06eb  IF NE JUMP $0686
06ee  JUMP $06B0        ; next frame
```

Entering at `0x06BB` once per 8 kHz frame, with a μ-law codeword presented in
DM `0x3F08`, runs DIAL through its real interface.

## Result: DIAL runs, dispatched by the task, and reacts to the line

200 frames per run, μ-law into DM `0x3F08`:

| Line input | `08F1` entries | DIAL `0x1BCE` line-signal handler | bootpage_nr (DM `0x3FB0`) |
|---|---:|---:|---|
| silence (`0xFF`) | 200 | 0 | `000c` ×200 — settles and holds |
| 440 Hz | 17 | 485 | `0000` ×189, `000c` ×10, `000b` ×1 |
| 1300 Hz | 253 | 43273 | `0000` ×181, `000c` ×13, `000b` ×6 |
| 2100 Hz | 51 | 1066 | `0000` ×162, `000c` ×23, `000b` ×13, `0010` ×2 |

Silence never enters the line-signal handler at all; every tone does, and the
state word walks the toggling pairs recovered in
`docs/dial_state_machine.md`. DIAL is being driven by the task, through the
overlay interface, and is genuinely filtering the audio.

## Recovered: the bootpage → overlay table at DM 0x31D5

PM `0x0686-0x0694` indexes DM `0x31D5` with bootpage_nr (DM `0x3FB0`) and
publishes the wanted overlay in DM `0x31AA` with a type in DM `0x31A9`. The
table (`--bootpage-table`) matches the ADDSP V.90 guide's Table 1 page
numbering exactly:

| Page | Entry | Download | Page | Entry | Download |
|---:|---|---|---:|---|---|
| 0 | `fd9e` | `0x0262` DIAL/FSK/FAX | 9 | `fd9e` | `0x0262` |
| 1 | `fd9a` | `0x0266` V.22/V.32 LEC | 10 | `fd92` | `0x026e` INFOH |
| 2 | `fd9a` | `0x0266` | 11 | `fd9d` | `0x0263` DIAL partial |
| 3 | `fda4` | `0x025c` FSK OWN | 12 | `fd8f` | `0x0271` V.22FC |
| 4 | `fd9e` | `0x0262` | 13 | `026b` | `0x026b` |
| 5 | `026f` | `0x026f` HV.34 | 14 | `026a` | `0x026a` V.90 DPCM |
| 6 | `fda1` | **`0x025f` V.8** | 15 | `fd9c` | `0x0264` FSKFAX partial |
| 7 | `fda0` | `0x0260` INFO | 16 | `fd9b` | `0x0265` FAX partial |
| 8 | `0261` | `0x0261` V.34 | 17 | `fd9e` | `0x0262` |

Negative entries are requested indirectly: they fall through `0x068F` to the
fixed pair (type `0x000D`, download `0x0270` = the SIG overlay), which is what
the harness observes the task asking for while DIAL is the active page.

This is the DSP-side half of the page switch `docs/dial_v8_call.md` describes.
It also supersedes that document's open question about how the V.8 page number
maps to an overlay: the mapping is this table, and it lives in TIKRNL, not in
DIAL.

## What is still not reached

- `0x08F0` (the DIAL **state dispatcher**) and the `DM(0x3FB2)` action-vector
  call at `0x06E8` are on the far side of the `DM(0x3FC1) & 0x0100` test at
  `0x06D5`, which currently always routes back to the bootpage check at
  `0x0686`. Only the line handler at `0x08F1` runs each frame.
- `0x0686` then re-registers the continuation and yields through the patched
  PM `0x000A`, so the loop does not sustain itself without a host serving the
  overlay request in DM `0x31A9`/`0x31AA`.
- The kernel's own foreground still never dispatches the task on its own: with
  SPORT0 strobed the ISR (PM `0x0072`) advances the request queue
  (DM `0x2E44`/`0x2E45` now move, which they did not before the task was
  loaded) and the foreground reaches its dispatcher at `0x02A1` → `0x01B2` →
  `0x00D8`, but the service list at DM `0x2F28` (`0x0F00`) is empty, so
  `CALL (I4)` at `0x02A4` never fires. Populating that list is the channel
  assignment, i.e. the MIPS side — see `docs/dial_sport_drive.md` and the
  `--mainloop` path in `tools/eicon_mips_shim.py`. This harness bypasses it by
  calling the task's frame entry directly, exactly as the host would if it
  drove the task itself.
