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

## Verified against the device (2026-09-01)

All of it holds. Every script id the driver enqueues is **accepted by the real
part**, and each one reports its own completion code back on the interrupt
endpoint -- `09 01` for script 9, `05 01` for script 5, `0a 01` for script 10,
`03 01` for the off-hook script, `02 01` for script 2. That is the `27 nn`
opcode every template ends with, arriving from the device: the assembler's
output executes, and `wValue = 0xFF01` is right.

One identification gained hardware support. Feeding the bulk OUT pipe while each
script runs in isolation, **only scripts 6 and 9 open it** (1120 bytes accepted,
against 0 for 1, 2, 3, 4, 5, 7, 10 and 12) -- the two the disassembly called
session bring-up and session end.

### Two defects of ours had to be fixed to see any of this

**The CD2 bootloader answers EP0 for only about three seconds.** Polling every
20 ms across a replug: the device appears, answers ~120 consecutive
`GET_INFROMATION` reporting family 01 (bootloader, wants firmware), and then
goes silent for good. The vendor driver uploads firmware on match, within
milliseconds; a probe started by hand is minutes late and meets a device that
has simply finished waiting. **This is the whole of the "wedged EP0" folklore in
`hsf_fxo.c`** -- it was never wedged, and the recovery written for it could not
have helped.

What hid it is worth keeping: libusb's darwin backend serves `GET_DESCRIPTOR`
for device *and* config out of IOKit's cache, so both keep succeeding after the
device has stopped answering anything at all. A liveness check built on them
reports a healthy device indefinitely. Only requests that reach the wire tell
the truth -- a string descriptor read is the cheapest honest one.

`hsf_fxo_probe --wait N` polls for the window **on a live EP0** (presence is not
the signal: the device sits on the bus long after it stops answering) and acts
inside it. Once firmware is running the device stays up indefinitely, so this
matters only for the upload.

**And two instruments were lying.** `hsf_fxo_stop()` freed transfers libusb
still owned -- a use-after-free surfacing inside `libusb_exit()` -- so every
probe run ended in SIGSEGV and lost whatever stdout was buffered; runs
alternated between working and printing nothing, which reads as flaky hardware.
And `rx_errors` counted `LIBUSB_TRANSFER_CANCELLED`, which is our own shutdown,
so an RX ring that was armed and correctly waiting reported "31 of 32 transfers
failed". Both are fixed; `--stream` now also prints the first real RX status.

## The patch bytes, and what the Linux driver actually does

**Scripts 2 and 10 take no parameters.** Their table entries have `npatch = 0`,
so although `hsfusbcd2200_` fills a patch buffer with one 16-bit value and
`hsfusbcd2202_` with two, the driver discards both. Only 7 (three bytes) and
11/12 (two) are really parametrised, which rules out the "unpatched templates"
theory for 2 and 10 without needing the hardware.

**Script 11 is enqueued as well** -- twelve scripts, not eleven. It is easy to
miss: `hsfusbcd2261_` pushes `$0xb` and jumps into script 12's argument tail
rather than repeating the four pushes, so it does not match a search for the
call pattern. That function sets **both** patch bytes to the same value, 1 or 2
(0x12/0x13/0x14/0x17 on the device variant selected by `ctx+0x68`), and picks
11 or 12 on a flag. Their templates are near-identical and their opcodes recur
in the hook scripts -- 12 is `2f xx 32 xx`, which is exactly the pair inside the
off-hook script 3, and 11 is `31 xx 2f xx`, whose `0x31` opens the on-hook
script 4. So 11/12 are the parametrised forms of the same relay operations.

Tested on the device with `--patch 1,1` and `--patch 2,2`: both accepted, both
reporting their own completion (`0b 01`, `0c 01`). No codec.

**The completion notification's second byte is a status.** Everything above
returns `nn 01`; script 8, the tone/signal script, returns **`08 80`** -- bit 7
set, and it produces no notification at all unless script 9 has run first. That
fits a script that needs a signal id: `hsfusbcd2187_` dispatches 21 of them but
the enqueue passes no patches, so the id must reach the firmware another way.

**The Linux driver does nothing special for the data path.** `cnxthwusb_probe`
finds the endpoints, allocates URBs, downloads firmware and registers the TTY --
there is no codec start at attach. The engine arms RX through
`hsfusbcd2267_`, which fetches a buffer and a length, **clamps it to 0x100** and
calls `OsUsbMakeDataReceiveRequest`; that is byte for byte what this transport
already does, 256-byte bulk IN transfers included. So the host side is not the
gate and never was.

## Open: what starts the codec

The bulk pipes still carry no samples. What is measured:

* RX is armed and waiting -- first status `CANCELLED`, i.e. our own teardown,
  with zero genuine errors. The device simply sends nothing, and the driver
  arms it exactly as we do.
* TX is accepted only after script 6 or 9, and then stops at a fixed **1120
  bytes** (1248 at 64-byte packets, 1216 at 32) however long the run. That is a
  ~1.2 kB FIFO filling once and never draining, so the codec is not clocking.
  Packet size is not the gate.
* None of the twelve scripts changes it, patched or not, on-hook or off-hook,
  alone or in the driver's own order.

What is left is the state the scripts *read*. The relay values live in the
firmware, not in the scripts (the off-hook script contains no 0xA6), so
something loads them -- most likely the country/config path, whose template
(26, seven patch bytes, 158 bytes and by far the largest) is not in the enqueued
set at all. Finding what sends it is the next thread; `hsfusbcd2165_`'s
surroundings and the `ctx+0x68` device-variant flag are where to start.
