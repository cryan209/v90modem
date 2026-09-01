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
**Every id 0-33 is enqueued** (34 is the queue's end sentinel), and wIndex is 1
everywhere except two sites that pass 3 for script 8.

Do not enumerate the ids by the four-push call pattern -- it gives the wrong
answer twice. Sites that share an argument tail push only the id and jump into
another site's `call`, which hides script 11 (`hsfusbcd2261_`) and, more
importantly, the entire signal range: **`hsfusbcd2187_`'s jump table at
`.rodata 0x2d0` maps 21 signal ids onto scripts -- signal 0 to script 0, and
signal n to script n+13 for n = 1..20** -- twenty arms, each pushing
`(patch buffer, wIndex 1, script id)` and jumping to the shared tail at 0x332e.
So scripts 0 and 14-33 are the tone/cadence set, and they take patch bytes from
`hsfusbcd2187_`'s own caller. **Template 26, the 158-byte one with seven patch
bytes, is signal id 13.** Signal ids 0x15/0x16 instead drive script 8, with
wIndex 3 (delete) to stop whatever signal is running.

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

**What the Linux driver does with the codec: nothing.** This was worth checking
properly rather than inferring from the GPL half, and the closed driver is
unambiguous.

`cnxthwusb_probe` finds the endpoints, allocates URBs, downloads firmware and
registers the TTY -- no codec start at attach. The data pump is started from
inside **`hsfusbcd2196_`, which is the NOTIFICATION HANDLER**: it checks
`msg[0] == 0xc1`, takes `msg[1]` as 1 or 2, copies `msg[8..]` into the context
and dispatches on the first payload byte -- the completing script id -- through
a 34-entry table at `.rodata 0x324`. The **`data[0] == 5`** case falls through
into the pump start. So it is the completion of **script 5** that tells the host
to stream.

And that start does no hardware configuration whatsoever. In full: zero a block
of counters, set two flags (`ctx->0x218 = 1`, `ctx->0x21c = 1`, both trivial
setters), prime **4** TX buffers, arm **16** RX URBs via `hsfusbcd2267_`, which
fetches a buffer and a length and **clamps it to 0x100**. No sample rate, no
format, no enable, no alternate setting -- the device has none to select, which
its own configuration descriptor confirms (two interfaces, one alt setting each,
64-byte bulk pair on IF0 and a 64-byte interrupt on IF1). Notification ids 14-33
share a handler that stores the payload and calls `OsEventSet`, i.e. async
events the engine waits on, not requests for more scripts.

The probe now mirrors that order -- pipes armed before any script is sent, as
the driver's ring always is -- because sending a script with no URBs posted and
starting the ring afterwards is a different experiment from the driver's. It
changes nothing: `9,5,9` and script 5 alone both still yield zero RX.

## The engine/driver data interface: two shared rings, no framing

Reversing far enough to answer the framing question did not need the 880 kB of
stripped `hsfengine` text -- the driver side names the whole interface.

`hsfusbcd2167_` (.text 0x6430) is the open call. Its fourth and fifth arguments
are descriptors carrying a buffer pointer at +4, and it stores them as

    ctx+0x894   RX ring base
    ctx+0x870   TX ring base
    ctx+0x934   ring size
    ctx+0x876   TX chunk = 0x80 (128)
    ctx+0x874   RX chunk = 0x40 (64)

`hsfusbcd2273_` is the TX length accessor: it computes
`write_ptr(0x8a2) - read_ptr(0x8a4)`, clamps to the chunk size, advances the read
pointer with wraparound against the ring size, and returns a byte count; its
partner returns `0x870 + offset`. The RX side is the same shape around
`0x894`. `hsfusbcd2269_` then hands that pointer and length straight to
`OsUsbMakeDataTransmitRequest`, and `hsfusbcd2267_` to
`OsUsbMakeDataReceiveRequest`, each clamped to 0x100.

**So the engine and the driver share two plain circular sample buffers, and the
driver ships slices of them as bulk URBs. There is no header, no framing and no
in-band configuration -- there is nowhere to put one.** That kills the first of
the two candidates above, from the driver side alone.

Matching the driver's exact granularity (64-byte RX transfers, 128-byte TX)
changes nothing: still zero RX.

There is one more control path in the driver -- `hsfusbcd2188_` CRCs a host
buffer with CRC-16-CCITT (poly 0x1021) and writes it in 64-byte blocks via
`CD2_WRITE_EEPROM`, driven by `hsfusbcd2168_`. **It is irrelevant to this part,
and the GPL source says so twice over.** `cnxthwusb_common.c:250` sets
`UpdateEEPROM` only inside `#if TARGET_HCF_FAMILY`, and only when a download
pipe at endpoint **0x03** is present with 64-byte packets; this device is HSF,
not HCF, and its configuration descriptor has no endpoint 0x03 at all (0x01 and
0x81 on IF0, 0x82 on IF1). So `cbUsbEEPROM_Restore` (`osusb.c:354`) can never
fire here. And the configuration itself does not live on the device: `NVM_Read`
in `osnvm.c` reads host-side dynamic parameters and falls back to the
compiled-in `g_DefaultCountryCode` / `g_FactoryProfile`, never touching the
device EEPROM. Our own probe reading it back as all zero is the expected state,
not a missing provisioning step.

## The notification endpoint, and ring detection working on a live line

The interrupt endpoint carries two distinct things, and telling them apart is
what made the DAA legible:

    bmRequestType 0xc1, bNotification 0x02 always
    wValue 1   a script completed: data[0] = script id, data[1] = status
               (0x01 succeeded, 0x80 did not)
    wValue 2   an asynchronous device event: data[0] = event code,
               data[1] carries a state bit in 0x80
    wIndex     a millisecond timestamp

**Event code 8 is RING, and the timestamps prove it rather than assuming it.**
With the modem on-hook on a live ATA extension and a call placed to it, 278
events arrived carrying only `0x0800` and `0x0880` -- one bit toggling. Their
`wIndex` spacing alternates **14 ms / 25 ms**, a 39 ms period = **25.6 Hz**,
which is the DAA reporting each half cycle of the 25 Hz ring voltage; and the
bursts are separated by **~190 ms (x5)** and **~2000 ms (x3)**, the
400/200/400/2000 double-ring cadence. So `0x80` in data[1] is the instantaneous
ring polarity, not a ring-start/stop flag.

That settles something this investigation had been quietly assuming in the other
direction: **the DAA works, the line is live, and the device is not dead.** The
silent codec is a specific fault, not a broken part.

The earlier guess in `hsf_fxo.h` that these would be the CDC PSTN codes
(RING_DETECT 0x09, AUX_JACK_HOOK_STATE 0x08) was wrong; they are Conexant's own.

### Hook control is UNKNOWN, and the LED is not a usable instrument

An earlier version of this section claimed, from LED observations, that script 4
(on-hook) works while script 3 (off-hook) is inert, and that scripts 11/12
carrying a relay value are the real hook control. **That is withdrawn.** It was
built on three by-eye observations with no control, and it does not survive
testing:

* a clean A/B -- script 4 baseline, then scripts 9,5,12 with relay 0xA6, then
  script 4, then scripts 9,5,3, each held 10-12 s -- lit the LED on **neither**
  arm;
* the obvious confound, that every LED-on observation coincided with
  `--feed --stream`, was tested directly (15 s quiet / 15 s streaming / 15 s
  quiet, no hook script at all) and the LED **stayed off**, so it is not a data
  light either;
* a four-way bisection of everything that differed between the original sweep
  and the failed A/B -- script 11 with 0xA6, script 11 with relay *code* 3,
  script 12 with relay code 3, and script 3 with streaming -- lit **none** of
  them.

So the LED-on state has not been reproduced in seven controlled attempts, and
the original observation cannot be attributed to any script. The timeline also
rules out the reading that produced the withdrawn claim: the relay-patch sweep
ran *after* the LED was reported on, so it cannot have caused it.

**Hook control is therefore unverified in both directions.** Script 3 and
script 4 are identified from the `DEVMGR_DAA_RELAY_CODE` dispatch in
`hsfusbcd2185_`, which is solid disassembly, and both are accepted by the device
with status 0x01 -- but nothing has confirmed that a relay actually moves. The
firmware accepts *any* operand for scripts 11/12 with status 0x01, so the
completion byte cannot distinguish a valid relay value from a rejected one, and
there is no other instrument in software.

The next instrument should be one that does not depend on reading an LED: the
ATA's own port state, or Asterisk seeing a seizure on the extension the modem is
wired to. A confirmed off-hook there also gives dial tone, which is the clean
test of the audio path.

**Lesson, and it is the same one this project's notes carry a dozen times: an
uncontrolled observation that fits a story is not a measurement.** The confound
here (streaming vs not) was present from the first observation and went
unexamined until the story it supported was contradicted.


## Open: what starts the codec

Eliminated, each by measurement rather than inference:

* the transport (the driver arms RX exactly as we do, same 0x100 clamp);
* the driver's codec setup (there is none -- no rate, no format, no enable);
* alternate settings (the device has one per interface);
* ordering (pipes are now armed before any script, as the driver's ring is);
* transfer granularity (64/128 as well as 160/256);
* in-band framing on the data pipes (two raw shared rings, nowhere for a header);
* all twelve scripts, patched and unpatched, on-hook and off-hook, alone and in
  the driver's own order.

* the tone/cadence scripts. Template 26 is **not** an unsent configuration
  block, which is what the last round guessed: it is signal id 13, one of the
  twenty `hsfusbcd2187_` sends. All of 0 and 14-33 were tried on the device on
  top of the session scripts; they load and run (15, 27, 32 and 33 report their
  own completion within the window, the rest run past it as a tone would) and
  none produces a sample.

What is left is firmware state the scripts read rather than carry. The relay
values are the proof it exists: the off-hook script contains no 0xA6, so the
firmware holds that table and something must load it. With the script space now
fully mapped and every one of the 34 exercised, that loader is not a script --
and the country/profile data the vendor stack holds host-side (`hsf.cty`,
`osnvm.c`) is consumed by the ENGINE rather than sent to the device, so it is
not a loader either. What loads the firmware's relay table is genuinely not
identified, and no path in this driver is a candidate for it.


## Inside the device: the firmware is 8051, and it has been opened up

The 7399-byte `ROM_IMAGE` the bootloader wants is **8051 code loaded at address
0**. File offset 0x28 proves it -- a textbook startup: `MOV R0,#7fh / CLR A /
MOV @R0,A / DJNZ R0` clearing internal RAM, then `MOV SP,#6ah`, then
`LJMP 0c9ah`. `tools/hsf_firmware.py` (with `tools/d8051.py`) disassembles it.

Two structures matter.

**A 105-entry LJMP dispatch table at 0x62.** 105 is 0x69, exactly the size of
the script opcode space, so this is very likely the interpreter's dispatch.
Targets at 0xd7xx-0xd9xx are on-chip mask ROM we do not have; **46 of the 105
land inside the uploaded image** and can be read. *Not settled:* several of
those 46 land mid-instruction under linear disassembly, so either the
interpreter enters shared code at computed points or this table is not the
script dispatch. Do not build on it without resolving that.

**SFR 0xF4 / 0xF5 are an indexed control register file** -- write a register
number to F4, then read or modify F5. This is the chip's internal control
plane, it is where a codec enable must live, and `--regs` maps it: **107 access
sites across 40 register indices**. Register 1 is the busiest and the only one
whose bits are set and cleared individually from several places
(`ORL #04h/#10h/#80h`, `ANL #7fh/#fbh`).

### A routine shaped exactly like the codec start

At 0x0c26, reached from dispatch entries 0x54/0x58/0x5c/0x60/0x64/0x68:

    MOV F4,#01h        ; select control register 1
    MOV R7,F5          ; read it
    JB  ACC.4, skip    ; already started?
    LCALL 1d4fh        ; init
    LCALL 1b11h        ; init
    MOV F4,#01h
    ORL F5,#10h        ; set bit 4 -- the "started" latch
    ...
    SETB AFh           ; SETB EA: global interrupt enable

An idempotent start guarded by a latch bit, ending in enabling interrupts. No
script we ship contains those opcodes, so this path has never run.

**Tried and it did not work.** `hsf_fxo_probe --raw <hex>` sends a hand-built
wire body; `5400 2740 2701 28 36` and the 0x58 and 0x68 equivalents are all
accepted by the device with the session scripts loaded, and produce no samples.
So either those wire opcodes do not actually reach 0x0c26 -- which would mean
the 0x62 table is not the script dispatch, and the mid-instruction targets say
that is possible -- or the enable needs more than this one call. Resolving which
is the next step, and the way in is the mask-ROM interpreter: the wire opcodes
our scripts *do* use dispatch there, so dumping 0xd000+ off the device would
settle the opcode map outright.


## Toward a mask-ROM dump: a read channel exists, a read opcode has not been found

**The EEPROM is not code memory.** `CD2_READ_EEPROM` swept over
wValue 0x0000-0xf000 returns 32 zero bytes at address 0 and 0xFF everywhere
above -- a real serial EEPROM, programmed at the bottom and erased above. The
mask ROM is not aliased there.

**Script opcode 0x26 appends a byte to the completion notification**, which is
an exfiltration channel. Proven directly: the hand-built body
`27 AA | 26 | 27 01 | 28 | 36` comes back as `aa 00 01` -- the two `27`
operands with a byte the `26` appended between them. It also explains the
odd 3- and 4-byte payloads seen earlier from the shipped scripts (script 32 is
`52 00 | 27 20 | 26 | 27 01 | 28 | 36` and reports `20 04 01`).

**Opcode 0x0b was thought to write a control register. It does not.** Shipped
scripts contain `0b 1c a0` and `0b 1d a0`, and the firmware register map has
exactly `MOV f4h,#1ch / MOV f5h,#a0h` and the same for 0x1d, which looked
conclusive. Tested with read-back either side, it is not: `0b 1f a6` and
`0b 1f a5` are both accepted and leave the register at 0x98, and `0b 1e 55`
wedges the device. The correspondence was pattern-matching, not proof. What
0x0b actually does is unknown, and there is no confirmed write primitive.

**The matching read is opcode 0x03, and it works.** With `0x26` as the sink, one opcode
that loads the result byte from a control register would make the register file
readable, and the same primitive pointed at code memory would dump the ROM.
Startup does `MOV f4h,#2dh / MOV f5h,#c0h`, so register 0x2d reading back 0xC0
is a ready-made oracle. Thirty-three candidate opcodes were tried as
`<op> 2d | 26 | 27 01 | 28 | 36` and **none returned 0xC0**; four (0x00, 0x02,
0x0c, 0x0e) completed with the appended byte still 0x00, and the rest did not
complete at all, which most likely means their operand length differs from the
template-language width and the script was malformed.

**Sweeping the remaining opcodes needs someone at the machine: opcode 0x0f
wedges the device**, and recovery is a physical replug (the firmware is
volatile, so a replug is a clean reset -- but it cannot be done in software).
That is the only thing standing between here and a full opcode map, and a full
opcode map is what turns the mask ROM from unreadable into a dump.


## The control register file is readable from the host

`03 <reg>` loads a control register into the script result byte; `26` appends
that byte to the completion notification. Found by sweeping the opcode space
against the firmware's own oracle -- startup does `MOV f4h,#2dh / MOV f5h,#c0h`,
so register 0x2D must read 0xC0, and **0x03 was the only opcode that returned
it**. `tools/hsf_opcode_sweep.py` does the sweep (resumable, because some
opcodes wedge the device and recovery is a physical replug);
`tools/hsf_regdump.py` does the reading.

Verified rather than assumed: reading 0x2D, then 0x01, then 0x2D again returns
`c0`, `90`, `c0` -- reproducible and register-dependent, which a constant or a
fluke would not be. Three further cross-checks against the firmware agree:
reg 0x2D = 0xc0, regs 0x1C/0x1D = 0xa0 and reg 0x03 = 0xad are all values the
firmware is seen writing.

The live control plane on a device with firmware running and no scripts sent:

    00: 01 90 00 ad 40 00 67 00 00 00 00 00 ff 00 ff 00
    10: 00 00 00 00 00 00 00 00 00 00 00 00 a0 a0 00 2c
    20: 00 00 ff 11 80 00 80 00 04 04 08 10 02 c0 40 00

**Register 1 reads 0x90, so bit 4 -- the "started" latch tested and set by the
enable routine at 0x0c26 -- is ALREADY SET.** That kills the theory in the
previous section that the codec had never been started: it has, before we send
anything. The routine at 0x0c26 is still a start sequence, but reaching it is
not what the silent codec is waiting for.

Two practical limits, both measured: the completion payload caps at about six
bytes, so reads must be batched four at a time (sixteen per script silently
returns five); and the completion can arrive after a short stream window, so a
miss is a timing artefact rather than a failed read and must be retried.


## The scripts made observable: what each one actually touches

With `03 <reg>` reading and `0b <reg> <val>` writing, the register file can be
dumped either side of a script, which finally makes the scripts observable
instead of inferred. Differential dump of 0x00-0x2F, one script at a time:

| script | control registers changed |
|--------|---------------------------|
| 9      | none |
| 5      | `1e 00->44`, `1f 2c->a8`, `21 00->08` |
| **3 (off-hook)** | **none** |
| **4 (on-hook)**  | **none** |

Two registers change on every step and are not effects of the scripts:
**`reg07` increments by one per script**, so it is a script-execution counter --
which independently proves the scripts run -- and `reg06` changes on every read,
so it is free-running (a timer).

**Scripts 3 and 4 touch no hardware register at all.** They are accepted, they
execute (the counter moves), they report status 0x01, and they change nothing.
So the doubt raised earlier in this document -- withdrawn once for resting on an
LED -- is now confirmed on a real instrument: **this driver has never taken the
line off-hook.** The `DEVMGR_DAA_RELAY_CODE` dispatch in `hsfusbcd2185_` that
identifies 3 and 4 as the hook scripts is good disassembly, but on this device
those scripts are inert.

The registers script 5 *does* write are suggestive: 0x1E, 0x1F, 0x20 and 0x21
are exactly the ones the firmware manipulates bit by bit (`ANL #0fh/#7eh/#f0h/
#f7h`, `ORL`, individual reads), and 0x1F lands on 0xA8 -- inside the
`hsf_smart_relays` range 0xA0-0xBD. That makes 0x1F the leading candidate for
the relay register, and `0b 1f a6` would be the seize.

**Redone properly, and it fails.** `hsf_regdump.py` now tags every script with a
leading `27 5a` and accepts only completions whose payload starts with that
marker, which removes the stale-completion problem that made the first attempt
worthless. With that: `0b 1f a6` and `0b 1f a5` are accepted and change nothing
(`before=98 after=98`), and `0b 1e 55` wedges the device. Register 0x1F also
reads 0x2c, 0xa8 and 0x98 at different times without being written, so it is
hardware-driven.

So the host can READ the whole control plane but has no proven way to write it,
and the relay register -- if 0x1F is even it -- is not settable this way. Script
5 does change 0x1E/0x1F/0x21, so the firmware can set them; what the host cannot
yet do is ask it to.


## The opcode sweep, finished

All 102 well-formed opcodes tested against the register-0x2D-reads-0xC0 oracle
(`tools/hsf_opcode_sweep.py`, resumable, marker-matched). **`0x03` is the only
opcode in the entire 105-opcode space that reads a control register.** No write
primitive exists that this oracle can see.

The sweep ran to completion with no wedges once the worklist used the real
per-opcode operand widths from the assembler's jump table. Sending `<op> 2d` to
an opcode taking zero or three operands desyncs the interpreter's parse, and a
desynced parse is what had been wedging the device -- each wedge costing a
physical replug. Opcode 0x0b wedges at every operand count tried and is skipped
outright.

The payloads also pin the execution model: `27 nn` **sets** a single result
byte and `26` **appends** it. Opcodes returning `5a5a` left the marker value
in place; those returning `5a0001` zeroed it. So report code, register read and
append all share one accumulator.
