# The Conexant HSF USB modem's DAA / line control interface

`hsf_fxo.c` drives the 0572:1300 as a line-side codec: bulk OUT 0x01 and bulk IN
0x81 carry raw codec samples with no framing, and everything else -- hook state,
tone generation, pulse dialling, session start and stop -- is the single vendor
request `CD2_CONTROL_SCRIPT`.

The transport half was already working. This note records the control half,
recovered from Conexant's closed `hsfusbcd2-i386.O` (shipped in
`hsfmodem-7.80.02.06oem/modules/imported/`) rather than from a bus capture.

## The request framing, and the field that was wrong

`hsfusbcd2210_` (.text 0x2c40) is the script pump. It ends in a call to the GPL
`OsUsbMakeControlRequest(hUsbOsHal, hCntrlReq, userContext, Request, Value,
Index, pBuf, nBytes)` -- whose prototype is in `osusb.c`, so the eight pushes at
.text 0x2ee9..0x2f17 name themselves:

    bmRequestType  0x41   vendor | interface | OUT
    bRequest       2      CD2_CONTROL_SCRIPT
    wValue         0xFF01, or 0xFF02 for script id 8 alone (.text 0x2ea7)
    wIndex         1 to load, 3 to delete
    body           the assembled script

**`wValue` is the field this project had wrong**: `hsf_fxo_script_load()` sent 0,
which no body can survive. `wIndex 1` was right all along, and the earlier
finding that "empty bodies stall at every index" is consistent with both.

## The templates are not the wire bodies

`hsfusbcd250_` (.data+0xe0) is a 35-entry table of 24-byte descriptors:

    +0  const void *template     (a relocation; empty slots have none)
    +4  uint16      template length
    +6  uint16      patch count
    +8  uint16      patch offsets[8]     -- offsets INTO THE TEMPLATE

The driver patches the template in place and then runs it through
`hsfusbcd290_` (.text 0x6f20) before sending. That assembler is why no guessed
or verbatim body was ever accepted, and it does three things:

* **Fixed-width verbatim copy** for most opcodes, at a width the jump table at
  .rodata 0x480 selects (1, 2, 3, 4, 5 or 6 bytes), plus one variable-length
  form (0x1a) whose third byte is a payload count.
* **Opcode remapping**: 0x54/0x58/0x5c/0x60/0x64/0x68 become `op - 0x51`, and
  their operand is biased by **+0x18** (.text 0x7031).
* **Label resolution**: opcodes 0x19 and 0x1b-0x22 carry a label number, not an
  address. The assembler keeps a 64-entry label table, writes 0xff for a
  forward reference and threads the unresolved sites through the destination
  buffer as a linked list; opcode **0x4f defines a label and emits nothing**,
  back-patching the whole chain (.text 0x70e3 / 0x718b).

So an assembled body is shorter than its template -- every 0x4f disappears --
and its branch operands are absolute offsets into the body. That is exactly why
a template sent as-is is rejected.

`tools/hsf_scripts.py` reimplements the assembler and generates
`hsf_scripts.h`. All 34 non-empty templates assemble without error, and the
generated table carries each script's patch offsets **already translated through
the assembly**; no patch offset lands on a remapped operand, so applying a patch
is a plain byte store (the tool checks this and warns if it ever stops being
true).

## Which script is which

`hsfusbcd2176_` (.text 0x3070) enqueues `(ctx, script_id, wIndex, patch[8])`.
Across all 19 call sites only ids **1-10 and 12** are ever enqueued, and wIndex
is 1 everywhere except two sites that pass 3 for script 8.

Identified:

| id | evidence |
|----|----------|
| 3 | off-hook. `hsfusbcd2185_` (.text 0x47b0) dispatches on `DEVMGR_DAA_RELAY_CODE` and routes 2, 3 and 8-11 -- every off-hook and pulse-dial state -- here |
| 4 | on-hook. The same dispatch routes 4-7, the on-hook states, here |
| 8 | tone/cadence. `hsfusbcd2187_` (.text 0x32f0) dispatches 21 signal ids onto it, and sends it with wIndex 3 (delete) to stop one |
| 7 | pulse dialling. `hsfusbcd2220_` patches three bytes: two context values divided by 1000 (milliseconds) and one literal, matching the template's 3 patch offsets |
| 9, 5 | session bring-up. `hsfusbcd2165_` (.text 0x4480), the function that zeroes the whole per-call state block, runs 9, then 5, then 9 |
| 6 | session end. `hsfusbcd2195_` / `hsfusbcd2201_` set the stopping flag first |
| 2, 10, 12 | parametrised; 2 takes one 16-bit value, 10 takes two, 12 takes a byte |

Note what is **not** in any script: a DAA relay word. The off-hook script
contains no 0xA6. The firmware holds the relay table and the scripts select from
it, which is why `hsf_smart_relays[]` is still worth having and still is not
something we transmit.

## Open, and how to close it

**Nothing here is confirmed against hardware.** The device did not answer
`CD2_GET_INFROMATION` at the end of the session that produced this note and
wants a physical replug, which is the documented recovery for a wedged EP0.

The one question a single hardware session answers is **which script ungates the
bulk pipes** -- the pipes NAK until the codec is started, and 9/5/6 are the
candidates. `hsf_fxo_probe --script N` sends one script with a
`CD2_GET_INFROMATION` either side, so a script that wedges the device is
reported as such rather than being blamed on the next one. Then
`--start-codec --stream 5` and `--hook off`.
