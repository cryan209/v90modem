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
| 9, 5 | session bring-up. `hsfusbcd2165_` (.text 0x4480) sends 9, waits up to 1400 ms for its event, then sends 5. A second 9 is only the timeout retry path |
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
script runs in isolation, **only scripts 6 and 9 open it** -- the two the
disassembly called session bring-up and session end.

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

The original probe still differed from the driver in two decisive ways. It
misread a conditional branch in `hsfusbcd2165_` as an unconditional sequence
and sent `9,5,9`; the second 9 actually belongs only to the 1400 ms timeout
retry. It also filled the TX pipe by polling from the main loop. The driver
primes four 128-byte writes and submits the next slice immediately from every
TX completion callback. The probe now does exactly that and waits for each
script completion before advancing.

**That starts the codec transport.** On the attached 0572:1300, the normal
script 9 completion followed by script 5 produced, in a 20-second capture,
841,472 RX bytes in 13,148 packets with zero USB errors. Every packet was 64
bytes. Independent 10- and 5-second runs produced 421,376 and 211,520 bytes,
again continuously and without errors. Script 6 stops the session and a new
9/5 start works again.

The bytes are not G.711 codewords. Read provisionally as four-byte frames, the
stream contains two signed 16-bit little-endian slots and runs at about 10.5 to
10.6 thousand frames/s over these short wall-clock measurements, plausibly the
device's 10.6667 kHz internal clock. The exact slot meanings and clock still
need a controlled line signal; do not build an audio converter around that
provisional interpretation yet.

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

The driver's exact granularity is 64-byte RX transfers and 128-byte TX
transfers. Continuous completion-driven TX refill, rather than granularity by
itself, proved essential to making that transport run.

### What the HSF engine does on TX completion

The completion path is a producer/consumer handshake, not a request to replay
the completed USB buffer.  `hsfusbcd2186_` first raises the engine event
`STATUS_TX_DATA_AVAILABLE` (`0x10024`) and converts the completed byte count
into a count of free 128-byte ring chunks.  It then enters `hsfusbcd2212_`.
For all newly free chunks that routine calls the engine's registered producer
callback with the exact replacement byte count, and only after that callback
has advanced the ring write side does it call `hsfusbcd2269_` to submit the
newly produced slices.  Thus the closed engine is paced by completed chunks:

    TX URB complete -> advertise free chunks -> engine fills those bytes
                    -> driver submits the filled 128-byte ring slices

The corresponding engine event handler is `hsfengine565_`.  Its `0x10024`
branch calls `hsfengine660_(..., 2, 1)`, which merely sets the TX-available
condition under a critical section, and then falls through to the same session
pump (`hsfengine824_`) used after an RX event.  Audio generation therefore
happens in the engine session pump, outside the USB completion callback; the
callback only accounts capacity and wakes it.

The modem waveform generators themselves write contiguous little-endian
signed-16 samples (for example `hsfengine4712_` stores one generated `AX` per
sample and `hsfengine4716_` writes 16-bit silence).  Consequently the two
alternating 16-bit words visible on bulk RX cannot yet be labelled “TX slot”
and “RX slot”.  Any zero-word insertion or device-specific packing lies below
the waveform generator, in the sample-register/DMA data element.  Feeding a
tone into only the first or second apparent word position tests two wire-format
hypotheses; it does not reproduce an engine channel selection.

### THE DAA NEVER GOES OFF-HOOK -- and the off-hook script lies (2026-09-02)

With a live line and dial tone confirmed present on it, off-hook produces no
dial tone in the receive stream.  The reason is visible in the register file
now that it can be read:

    on-hook            01=90 02=00 03=ad 04=40
    after script 3     01=90 02=00 03=ad 04=40     <-- IDENTICAL

**`0x03` reads `0xad`, and 0xad is the low byte of `0x80AD` =
`ONHOOK_PHONEOFFLINE_CALLID` in `hsf.cty`'s SMART_RELAYS table** (already
decoded in `hsf_fxo.h`).  An ON-HOOK relay word.  The off-hook script is
accepted, completes, reports its `03 01` on the interrupt endpoint -- and
**changes nothing**.  The firmware map agrees that these are the relay
registers: `MOV f5h,#adh` sites exist on registers 0x01, 0x02 and 0x03.

So the DAA sits on-hook for every experiment in this document, which is why
there is no dial tone, no audio in either direction, and -- very plausibly -- no
transmit: **the entire "TX does not drain" investigation was conducted on a
modem that never picked up the phone.**

Writing the relay word directly is not sufficient either.  `0b 03 b6`
(`OFFHOOK_PHONETOLINE`) is accepted and **sticks** (0x03 reads 0xb6 afterwards
and holds across a session start), but the line is still not seized: no dial
tone, RMS 1.7 against 2.5.  The high byte does not take -- `0b 02 80` leaves
0x02 at 0x00 -- so either 0x03 is a shadow rather than the pin drive, or the
relay needs something beyond these two registers.

**This is where the work should resume**, and it is a far better-localised
question than the transmit stall: what actually drives the DAA relay, given
that the script the driver uses for off-hook demonstrably does not touch the
register holding the relay word.

### THE RX STREAM IS NOT AUDIO, AND "RX WORKS" WAS NEVER TESTED (2026-09-02)

Every health check in this investigation counts bytes.  None checked that the
bytes are audio, so the whole session's framing -- "receive works, transmit is
the hurdle" -- rested on an assumption nobody had tested.  It is false.

Captured off-hook while transmitting DTMF and analysed (`tools/hsf_rx_tone.py`,
Goertzel over dial tone and the transmitted digit):

    42480 frames, DC 694.7, RMS 2.5
    350 Hz 0.60x floor   440 Hz 0.94x   697 Hz 0.95x   1336 Hz 1.62x

**No dial tone off-hook, and no trace of our own transmitted digit.**  Both
16-bit slots carry the same flat value: slot 0 spans 682-708 with **22 distinct
values in 42480 frames**, slot 1 684-702 with 19.  That is an ADC reading
nothing -- a DC level with a few LSBs of noise -- not a line.

And the DC value identifies two registers.  **695 = 0x02B7**, exactly the range
`0x18`/`0x1a` were seen oscillating in (0x02b2..0x02b7).  They are not DMA
pointers, which is why they never wrapped: **they are the current ADC sample**.

One more thing the captures settle, recorded in the probe as a quirk and never
followed up: **RX only flows while OUT is being fed.**  The two captures
without `--feed` returned **rx = 0**; the one with it returned 169920 bytes.  A
device that will not produce IN without OUT is clocking both directions
together.

**Consequence: "RX works, TX does not" is withdrawn.**  Neither direction
carries audio.  The digital interface clocks and the codec's analogue side is
idle, so the TX FIFO not draining is plausibly that same condition rather than
a transmit-specific fault -- which is consistent with every transmit-specific
hypothesis in this document coming back null.  Every result above that used RX
byte count as "healthy" is measuring the transport, not the modem.

### Opcode 0x0b IS a register write -- the note saying otherwise was wrong

These notes record "Opcode 0x0b was thought to write a control register.  It
does not", on the evidence that `0b 1f a5` is accepted and leaves register 0x1f
at 0x98.  **That conclusion is withdrawn.**  Measured with the 0x03 read as the
oracle:

    reg 0x24: before=80  write 0x55 accepted  after=55
    reg 0x24: before=55  write 0x80 accepted  after=80

Unbiased, and it sticks.  What misled the earlier test is that **writability is
per register and per bit**, and 0x1f is one of the partial ones:

    reg 0x1c: before=20  write 0xa0  -> 20    bit 7 REFUSED at idle
    reg 0x1e: before=00  write 0xa0  -> a0    fully writable
    reg 0x1f: before=3c  write 0xa0  -> 1c    partially writable
    reg 0x06: before=f7  write 0xa0  -> 34    dynamic, hardware-driven

`0x1c` refusing bit 7 at idle is consistent with 0xa0 being a hardware-owned
"channel running" state rather than something the host sets, which is what the
idle/streaming dump suggested.

The firmware side agrees.  At 0x0a03/0x0a18/0x0a27/0x0a32 there is one handler
family sharing a shape -- register index from R5, value from R4 -- providing
**read, write, OR-set and AND-clear**:

    A = R5 + 0x40 ; MOV f4h,A      then  MOV R1,f5h      (read)
                                         MOV f5h,R4      (write)
                                         ORL f5h,A       (set bits)
                                         A = R4 ^ 0xff ; ANL f5h,A  (clear bits)

Each ends `MOV A,#02h/#03h; RET`, which is the operand width the driver's own
table carries.  **They are not in the 0x62 dispatch table and nothing in the
image references them**, so they are reached by computed dispatch, most likely
from the mask ROM at 0xd7xx -- more evidence that the 0x62 table is not the
script dispatch.  Note the `+ 0x40` bias in the handler, which the measured
writes show is NOT present at the script level, so something compensates it
before R5.

Also new: `MOV f6h,R1` / `MOV A,f7h` appear beside the F4/F5 accesses at 0x15d3
and 0x17f4, so **SFR F6/F7 is a SECOND indexed file** that these notes had not
recorded.

**Two cautions, both learned the hard way.**  Register writes **persist across
runs** -- an idle `0x1e = 0xa0` set here was still there on the next session --
so a sweep pollutes its own later arms unless each write is restored.  And
writing **0xff to 0x1e during a live stream wedged the part** (RX fell to 60160
mid-run and every later run failed until a replug), so 0xff is too blunt a
first probe on a register whose bits are unknown; use the OR-set and AND-clear
forms one bit at a time.

### The control register file, dumped IDLE and STREAMING (2026-09-02)

`tools/hsf_regdump.py --live` now brings the session up and opens the stream
before sending the read script (`--raw-post` in the probe), so an idle dump and
a streaming one are separate measurements rather than the same one twice.  The
oracle holds in both (`0x2d = 0xc0`).

    idle  00: 05 90 00 ad 40 00 59 02 00 00 00 00 ff 00 ff 00
          10: 00 00 00 00 00 00 00 00 00 00 00 02 20 20 00 3c
          20: 00 08 ff 11 80 00 80 00 04 04 08 10 02 c0 40 00
    live  00: 05 90 00 ad 40 00 d8 00 00 00 00 00 ff 00 ff 00
          10: 00 00 00 00 00 00 00 00 b4 02 b7 02 a0 a0 44 b8
          20: 00 08 ff 11 80 00 80 00 04 04 08 10 02 c0 40 00

Everything from 0x20 to 0x60 is identical; **the whole of the streaming state
is 0x06-0x07 and 0x18-0x1f**:

    0x06/0x07   0x0259 -> 0x00d8
    0x18/0x19   0x0000 -> 0x02b4     moves between reads (0x02b2..0x02b5)
    0x1a/0x1b   0x0200 -> 0x02b7     moves between reads (0x02b3..0x02b7)
    0x1c        0x20   -> 0xa0       static while streaming
    0x1d        0x20   -> 0xa0       static
    0x1e        0x00   -> 0x44       static
    0x1f        0x3c   -> 0xb8       static

`0x1c` and `0x1d` are exactly the registers the shipped scripts write with
`0b 1c a0` / `0b 1d a0`, and the difference from idle is **bit 7**, so 0xa0 is
plausibly "channel running" -- set on BOTH, which is notable given only one
direction works.

The two 16-bit pairs at 0x18 and 0x1a are the only things that move, and both
do.  **They do not wrap**: sampled repeatedly they sit inside 0x02b2..0x02b7, a
span of six, where a pointer cycling a buffer sampled at arbitrary instants
would be spread across its whole range.  So they are not free-running DMA
pointers over the ring; a small oscillation around a fixed value is more
consistent with a level, a difference, or a pointer that is being held.

**Not yet interpreted, and the obvious next move is blocked**: writing a
control register would settle what 0x1c/0x1d and the 0x18/0x1a pair mean, and
`0b` -- the opcode that looked like a register write -- is already recorded here
as NOT one (`0b 1f a5` is accepted and leaves the register at 0x98).  So the
file is readable and not writable by any known opcode, and finding a write
channel is what this line of attack needs next.

### The "DMA record encoder" does not exist as described (2026-09-02)

The entry below hypothesised that the engine fills the shared ring with
*records* from its DMA/sample-register layer, and that the missing TX operation
was that record encoder.  Read directly, it is not.

`hsfengine1049_` is the FIFO writer, and `hsfengine1047_(obj, stride, offset)`
configures it.  With the ring base at `obj->0x1c`, the write index at `0xc`,
stride at `0x20` and offset at `0x24`, it writes **raw 16-bit samples**:

    stride 2:  ring16[(idx + i) * 2 + offset] = src16[i]
    stride 3:  the same sample written to THREE consecutive slots
    stride 1:  a straight copy

No headers, no class byte, no record framing of any kind.  `hsfengine1784_`
creates both FIFOs and configures **both** with **stride 2, offset 1**.

Two corrections to what is written below.  The claimed packing
`wire[2i+0] = 0; wire[2i+1] = tx_sample` is wrong in its first half: the
untouched slot is **left as it was**, never zeroed -- only the offset slot is
written.  And the record grammar of `hsfengine2244_` belongs to the DMA FIFO
unit (`dmafifo_c.c`), which is the DMA-based hardware families' path; nothing
establishes that the USB path routes through it at all, since the cd2 driver
hands the engine a buffer pointer in the `ctx+0x20` callback and the engine
writes into it in place.

**None of it can explain the stall, and that is the point worth keeping.**  The
device accepts 19 blocks and then stops acknowledging OUT forever; its FIFO
fills once and never empties.  No sample layout, lane choice, frame size or
feed rate can produce that symptom -- a consumer reading the wrong slot would
still *consume*.  Format hypotheses have been tested repeatedly against a
symptom that format cannot cause, this session included.

**Correction after tracing beyond the shared rings:** “no framing” above is
true only of the USB driver's view.  The driver does not add a header, but the
engine fills the shared ring with records produced by its DMA/sample-register
layer; the ring is not the waveform generator's `int16_t[]` itself.
`hsfengine2244_` is the receive-side record parser.  It consumes a first byte,
selects the record class with `byte0 & 3`, consumes a second byte, and derives
an index from `byte1 >> 1` plus a flag from `byte1 & 1` before dispatching the
payload to `hsfengine1172_`, `hsfengine1175_`, or `hsfengine1166_`.  The latter
writes the decoded payload into the 4096-byte FIFO at object `+0x64`; the raw
USB FIFO is the separate 4096-byte FIFO at `+0x3c`.  In the other direction,
the engine must perform the inverse record construction before the USB driver
can ship the bytes.

This also explains the live TX result.  Raw signed-16 DTMF, duplicated into
both apparent word positions or placed in either position alone, is accepted
only until the device's roughly 2.4 kB input FIFO fills.  Adding a second
four-block kick exactly at script 5 completion changed nothing: a five-second
run still stopped at 2432 TX bytes while RX delivered 211712 bytes without an
error.  The missing operation is therefore not another USB prime and not a
choice between two PCM slots; it is the engine's TX DMA-record encoder (and
possibly its initial sample-register records).  Do not feed bare PCM to bulk
OUT and treat FIFO acceptance as transmitted audio.

The tempting direct inverse of that grammar is now disproved.  Prefixing each
128-byte OUT block with class 0 plus a rolling seven-bit index stopped after
1280 bytes; class 2 stopped after 1152 bytes and also stopped continuous RX.
Bare PCM had reached 2432 bytes.  Both engine record classes are therefore on
the engine's internal side of the sample-register layer, not the USB wire side.
The probe's experimental encoder was removed rather than leaving a known-wrong
protocol selectable.

Nor does this parser settle the earlier slot question.  The live USB capture
still aligns consistently as `[nonzero signed-16, zero signed-16]` four-byte
units, but those are the output of `samplereg_c.c`; they are not the records
consumed by `hsfengine2244_`.  The exact inverse must be taken from the
sample-register packer, not inferred by applying the DMA FIFO parser directly
to endpoint bytes.

### The sample-register lane is settled

The actual packer is the `hsfengine1780_` / `hsfengine1049_` path, adjacent to
the `DMA FIFO INIT FAILED` reference that identifies this compilation unit.
`hsfengine1780_` clears its staging samples and calls `hsfengine1049_` with the
generated signed-16 buffer.  The DMA FIFO was configured by `hsfengine1047_`
with a stride of **2** and lane offset **1**, so `hsfengine1049_` performs the
equivalent of:

    wire[2*i + 0] = 0;
    wire[2*i + 1] = tx_sample[i];

The receive sibling, `hsfengine1787_` / `hsfengine1043_`, copies one selected
lane into the engine's contiguous receive buffer and can swap adjacent lanes
for a hardware variant.  Thus the first apparent word is not TX on this path;
TX is the second signed-16 word.

That packing alone is still insufficient to make the firmware consume OUT
continuously.  A live lane-1 test with eight initial 128-byte blocks of exact
zero before DTMF again stopped at **2432 bytes**, identical to unprimed bare
PCM, while RX delivered 211712 bytes in five seconds without error.  Therefore
“prime” is not merely silent audio in the correct lane.  The remaining delta
is the DMA FIFO's producer/consumer state transition or its DCP initialization,
not sample layout.

### The stream-open scripts, and the one surface never tried

The transmit stall is now bounded from the driver side.  `hsfusbcd2269_` is the
only caller of `OsUsbMakeDataTransmitRequest` and it is a plain bulk submit of
at most 0x100 bytes taken from a host ring whose TX granularity (`ctx+0x876`)
is **0x80** against the RX granularity (`ctx+0x874`) of **0x40** -- so 128-byte
OUT blocks and 64-byte IN packets are the driver's own sizes, and the probe
already matches both.  `hsfusbcd2196_` primes **four** TX requests and
**sixteen** RX requests from the notification handler, which the probe also
matches.  The two calls immediately before that priming, `hsfusbcd2248_` and
`hsfusbcd2245_`, are bare setters of `ctx+0x21c` and `ctx+0x218`; there is no
control request there.  **Nothing the host does to the USB pipes is missing.**

What is missing is on the script side, and it is the one surface these notes
already named as untested: the patch bytes.  `hsfusbcd2261_` (.text 0x37f0) is
a start/stop pair around a single path code.  It zeroes an 8-byte patch buffer,
writes one code into bytes 0 **and** 1, and enqueues **script 12** when its
second argument is non-zero and **script 11** when it is zero.  The code comes
from the first argument and from the hardware variant at `ctx+0x68`: on the
ordinary part 0 -> 0x02 and 1 -> 0x01, with 2 and 3 doing nothing at all, and
on the `0x68 == 1` variant 0 -> 0x14, 1 -> 0x13, 2 -> 0x12, 3 -> 0x17.

The call sites are what make this the transmit lead rather than one more
script.  `hsfusbcd2167_` is the **stream-open** entry -- it is the function that
writes the 0x80/0x40 granularities at .text 0x6480 -- and on the way in it
sends **script 8 with wIndex 1** (`hsfusbcd2247_`) and then `2261_(3, 1)`, a
START.  `hsfusbcd2265_`, the stop counterpart, sends `2261_(3, 0)` and
`2261_(1, 0)`, two STOPs.  Off-hook (`hsfusbcd2180_`) is followed by
`2261_(2, 1)` and on-hook (`hsfusbcd2166_`, `hsfusbcd2185_`) by `2261_(2, 0)`.
So a session that has run 9 and 5 has brought the *codec* up -- which is what
the continuous RX shows -- and has not yet opened the *stream*, which is where
both the granularities and the start code are set.

`--post-script ID[,ID...]` with `--post-patch B[,B...]` sends scripts after the
session is up and the TX ring is primed, because the order relative to the
prime is not known and should be swept rather than guessed:

    ./hsf_fxo_probe --start-codec --feed --dtmf 5 --stream 5 \
        --post-script 8,12 --post-patch 0x17,0x17

The prediction it makes is specific: if this is the gate, TX stops being a
~2.4 kB one-shot FIFO fill and starts completing continuously, at the same
128-byte granularity, for the whole run.

### What the driver actually does at stream open -- script 1, which we never sent

Extracting every `hsfusbcd2176_` call site with its script id gives the whole
sequence in one place, and it contains a script this probe had never sent.

    hsfusbcd2165_   session start:  9 (wIndex 1), 5 (wIndex 1), 9 again on a
                                    1400 ms timeout -- the retry, not a third step
    hsfusbcd2167_   stream open:    hsfusbcd2241_ -> SCRIPT 1 (wIndex 1)
                                    hsfusbcd2247_ -> script 8 (wIndex 1)
                                    then granularities 0x80 TX / 0x40 RX,
                                    then hsfusbcd2196_'s 4 TX / 16 RX prime
    hsfusbcd2265_   stream stop:    2261_(3,0), 2261_(1,0) -> script 11
    hsfusbcd2195_/2201_  session end: 6, and 4
    hsfusbcd2180_/2166_/2185_  hook: 3 off, 4 on
    hsfusbcd2220_   pulse dial:     7
    hsfusbcd2187_/2250_  signals:   8 (wIndex 3 to delete)
    hsfusbcd2200_ / hsfusbcd2202_:  2, and 10 (wIndex 1, patch carrying 0x8000)

**Script 1 is a QUERY, and it is the first thing stream open does.**
`hsfusbcd2241_` clears an event, sends it, waits **1400 ms**, retries once on
timeout, then calls `hsfusbcd2239_(ctx, 4)` -- a FOUR byte read, where every
other script's completion is two -- and sets `ctx+0x911` into `ctx+0x930` and
**`ctx+0x68` from (byte `0x912` == 1)**.  That is the mode this document
previously took for a hardware variant: it is the device's own answer to script
1.

Live, this part replies **`01 00 00 01`**, so byte `0x912` is 0 and **the mode
is 0** -- measured now rather than assumed, which retrospectively justifies the
mode-0 reading of `hsfusbcd2261_` (args 2 and 3 inert, stop = script 11 with
0x01).

Two defects of ours fell out of sending it.  The completion parser took byte 1
as the status, which is right for a two-byte reply and reports script 1's
`01 00 00 01` as a **failure**; the status is the last byte.  And the probe
armed the notification pipe and both data rings together at attach, where the
driver has notify up from attach but primes the bulk rings only at stream open
-- so script 1 went into an already-running stream and **collapsed RX to 2366
bytes**.  With the rings deferred (`hsf_fxo_defer_rx()` / `hsf_fxo_arm_rx()`,
`--stream-open`) the driver's order runs intact: RX 168832 against a control's
169792.

**It does not unblock TX.**  In the full order transmit reaches 2944 bytes and
stops, against 2432 for the control -- still a FIFO that fills once, with 650
submits refused.  Note 2944 exceeds the 2496-2527 capacity bound measured
earlier, so that bound describes mid-sequence runs and is not a constant of the
part either.

**Scripts 2 and 10 are register writes, and both are now sent with the driver's
own parameters.  Neither moves TX.**

`hsfusbcd2200_(ctx, flags, value, addr)` builds a 16-bit big-endian word
`(addr & 0x1fff) | value | 0x2000` into patch[0..1] and sends script 2 -- but
**only when flags bit 0 is set**, so call sites passing 2 emit nothing at all.
`hsfusbcd2202_(ctx, idx)` sends script 10 with two words, `| 0x6000` and
`| 0xa000`, whose bodies come from `.rodata` tables at 0x1f0 and 0x1e0 indexed
by `idx`; the code clears exactly the bits those tables set (`dh &= ~0x06`,
`ah &= ~0x10`), so idx 0-7 is a 2-bit field at 0x0600 plus a 1-bit flag at
0x1000.

The important find is **`hsfusbcd2227_` -> `hsfusbcd2252_`, a codec register
programming sequence that stream open runs and nothing here had ever
performed**.  Nine calls, of which the three with flags 2 send nothing, leaving
six words -- with the ctx+0x8d0..0x8dc shadows zero, as they are on a fresh
session: **0x2004, 0xA208, 0xF000, 0xA228, 0x6000, 0x25B4** (two have a second
form, 0xA028 and 0x25B5, taken when ctx+0x8d4 is zero and ctx+0x1c8 byte 0 bit
0 is clear -- `--stream-open-alt`).

Measured, all on a healthy part: the six writes are accepted (notifications 4
-> 12) and TX reads **2560**, the alt form **2432**, the control **2432**.  The
script 10 sweep over all eight indices gives 2432-2688 with RX unchanged.
**Nothing shifts the transmit path.**

**The shadows are not zero, and taking them as zero got three of the six words
wrong.**  `hsfusbcd2169_`, which stream open calls immediately after script 8,
sets `ctx+0x8d0 = 0x40`, `0x8d2 = 0x800` and `0x8d4 = 0x200`
**unconditionally**; the rest derive from the config struct at `ctx+0x1c8` and
from `ctx+0x58`:

    0x8d6 = 0 / 0x200 / 0x400 / 0x600   from (cfg[0] >> 1) & 7
    0x8da = 0x1000 if (cfg[0] & 0x10) or ctx+0x58, else 0
    0x8dc = 0 / 0x800 / 0x1000 / 0x1800 from (cfg[0] >> 5) & 7
    0x8e4 = 0 if cfg[0] & 1, else 0x80 if cfg[1] & 1, else 0xC0

(`0x8d8` is set as well but only feeds a flags-2 call, which sends nothing.)
Note `0x8e4`'s default is **0xC0, not 0**.  Corrected, the all-zero-config
sequence is **0x2004, 0xA208, 0xF200, 0xAAE8, 0x6040, 0x25B4** -- against the
0x2004, 0xA208, 0xF000, 0xA228, 0x6000, 0x25B4 first sent.

**Still nothing.**  The corrected words give tx 2560, and sweeping each unknown
independently (`--regs d6,da,dc,e4`, twelve combinations covering every value
of all four fields) gives **2432 or 2560 in every one**, RX unchanged at
169600-169920.

So the driver-visible sequence is now reproduced end to end **with the correct
operands** -- session 9/5, stream open's script 1 query and script 8,
`hsfusbcd2169_`'s shadows, the six codec register writes, the 0x80/0x40
granularities, the 4 TX / 16 RX prime, RX-credited pacing, the script 11 stop
and the script 6 session end -- **and TX still fills its FIFO once and stops.**

**The ring itself holds no gate, and the accounting says TX is half the RX byte
rate in a different frame format.**  `hsfusbcd2273_` takes the contiguous run
between `ctx+0x8a4` (submit) and `ctx+0x8a2` (completed), and `0x8a2` is
advanced by `hsfusbcd2186_` on TX completion -- so the ring is shared memory the
engine writes in place during the callback at `ctx+0x20`, and the driver never
copies transmit data at all.  Nothing there gates the device.

The accounting is more interesting.  `hsfusbcd2184_` divides accumulated
**received** bytes by `ctx+0x876` = 128 -- the TX granularity, not the RX one --
into `ctx+0x8ac`, and `hsfusbcd2212_` hands the engine
`ctx+0x874 * units = 64 * (rx_bytes / 128) = rx_bytes / 2` as **both** the
receive and the transmit length.  So the driver asks for **half** the received
byte count, which matches the frame layout already on record here -- a received
frame is four bytes carrying one 16-bit sample and one zero slot
(`5b 01 00 00`) -- and means the transmit side is a **plain 16-bit mono stream
at 2 bytes per frame**, not the 4-byte two-slot frames this probe was sending
at 1:1 with RX.  Fixed (`--tx-quad` restores the old form).

**No effect: tx 2432 mono against 2560 quad.**  In hindsight it could not have
had one -- the FIFO never drains, so feeding it at half the rate in a different
layout only fills it more slowly.  It is in because it is what the driver does,
not because it helps.

That is now a strong negative result rather than a gap: **nothing the cd2
driver does to this device starts its OUT consumer.**  Whatever does is either
in the engine's own interaction with the part (the engine is what fills the TX
ring, and it is closed), in the content or framing of the OUT data itself, or
in a firmware-side condition no host software drives at all.

### TX drained once, unreproduced in 16 runs (2026-09-02)

With `hsfusbcd2265_`'s stream stop added to the teardown -- script 11 ahead of
script 6 -- one run transmitted **102400 bytes**
against the 2432-byte ceiling every previous run in this investigation has hit.
It was very nearly symmetric with receive (**101888** RX), which is what
RX-credited feeding predicts once the device is actually consuming, so the
codec drained the OUT FIFO for the whole four seconds.

**It did not reproduce, and on the evidence it is more likely a symptom than a
success.**  A fresh firmware load followed by **sixteen** identical runs gives
tx 2432 every time, RX steady at 169500-169700, and no degradation at all.  The
one drain came instead at roughly the twelfth run of an older load, in the six-
run sequence whose runs 5 and 6 were dead -- i.e. **immediately before that part
failed** -- and in that run **RX had fallen to 25.5 kB/s from the usual 42.4**.
A device that starts consuming OUT correctly should not also start producing
less; a device sliding into the state where it produces nothing might do
exactly that.  So "the OUT consumer can run" is NOT established, and the
earlier wording claiming it is has been withdrawn.  What can be said is that
there exists a state in which the FIFO empties, and that state is adjacent to
the failure.

A methodological note, since it nearly went into the record as a result: the
first reproduction attempt printed `<-- DRAINED` against all eight runs.  That
was an awk string comparison -- `gsub()` had made the field a string, so
`"2432" > "10000"` is true lexically.  **Force the numeric compare** (`$8+0`)
in any harness that flags a threshold.

### 2432 is not a constant of the device: the FIFO is 2496-2527 bytes

The transmit stall was described for a long time by the byte count at which it
happens, which invited the wrong question.  Swept over the TX block size on a
healthy part (three health runs in front of it, RX ~169500 in every arm), the
**total** accepted holds while the block COUNT scales inversely -- 32 B x 78,
64 x 39, 128 x 19, 192 x 13, 256 x 9, i.e. 2496, 2496, 2432, 2496 and 2304
bytes.  That is a byte capacity, not a transfer or packet limit, and each arm
bounds it: intersecting all five gives

    2496 <= device OUT FIFO <= 2527 bytes

which is 624-632 four-byte frames, or **58.5-59.2 ms** at the ~10.667 kHz
device clock -- a sensible audio buffer depth -- and note that 2.5 KiB minus one
64-byte USB packet is exactly 2496.  So **2432 is simply the largest multiple of
a 128-byte block that fits in that FIFO**, and quoting it as the signature was
quoting our own block size.  The FIFO fills once and never drains.

Correction to an earlier claim in these notes: "on a healthy part TX is 2432
exactly, every time" was overstated.  Mid-sequence runs at a 128-byte block are
consistent, but the FIRST run after a firmware load is not -- 2816, 2688, 1408
and 1408 have all been seen -- and that variation is unexplained.  Use
mid-sequence runs when the number matters.

### The host side is now fully bounded, and the stall is device-side

Following the driver's pump rather than sampling it settles how TX is meant to
be driven.  **Both** completions land in the same function: `hsfusbcd2184_`
(RX done) credits the received byte count, divides by `0x876` = 128 and adds
the quotient to `ctx+0x8ac`; `hsfusbcd2186_` (TX done) does the same for the
transmitted count into `ctx+0x8a6`; both tail-jump to `hsfusbcd2212_`.  And
`hsfusbcd2212_` **returns immediately when `ctx+0x8ac` is zero** -- no RX units
pending, no work -- otherwise submitting exactly one RX and one TX request and
decrementing both counters.  So TX is credited by RX, 128 bytes out per 128
bytes in, which is what a synchronous codec wants; TX completions re-enter the
pump but are not themselves a refill trigger.

This probe had been free-running a four-deep TX pipeline off its own
completions with no reference to RX at all.  That is now fixed (`--tx-free-run`
restores it) because it is what the driver does -- but **it is measured neutral
on the stall**: RX-paced 211520 RX / **tx 2432**, free-run 211392 RX /
**tx 2432**, back to back on a healthy part.

What that leaves is unambiguous, and the counter that shows it is new
(`tx submits refused (ring full)`): on a five-second run the device accepts
**19 blocks -- 2432 bytes -- and then never acknowledges another**, all 32
outstanding transfers stay un-acknowledged with an infinite timeout,
`tx_err 0`, and **817 submits are refused because the ring never drains**,
while RX continues at full rate throughout.  Nothing is erroring.  The device
simply stops consuming bulk OUT, and its input FIFO never empties.

**Off-hook is refuted too, and this time on a healthy part.**  These notes
already said "off-hook does not change it", but that was measured before the
missing session end was found, so it was almost certainly taken on a degraded
device.  Re-run clean, back to back with an on-hook control: on-hook **rx
211584 / tx 2432 / 817 refused**, off-hook **rx 211584 / tx 2432 / 817
refused** -- identical to the byte, with the off-hook script accepted and its
completion visible as a fourth notification.  The firmware's OUT consumer is
not gated on the DAA state.

So all four host-side candidates are now refuted by measurement rather than by
argument: the **pipes** (sizes, prime depths and ordering all match
`hsfusbcd2196_`), the **script layer** (all fifteen arms, tx 2432 without
exception), and the **pacing** (both disciplines, tx 2432), and the **hook state** (identical
to the byte).  What starts the
firmware's OUT consumer is a device-side question, and the register file and
the 8051 image are where it has to be answered.

**THE PROBE WAS KILLING THE PART, AND IT WAS ONE MISSING SCRIPT.**  Script 6
is `hsfusbcd2195_`/`hsfusbcd2201_`'s session end, and nothing here had ever
sent it: every run opened a session with 9 and 5 and abandoned it.  The cost
was not visible in any per-run signal except one.  A freshly loaded part served
**one** streaming run and then produced no audio at all -- while EP0 kept
answering, scripts 9 and 5 kept completing and reporting on the interrupt
endpoint, and TX kept filling its FIFO.  Measured: baseline 127232 RX bytes,
next run 320, next 0, and no recovery without a replug and a reload.  With
script 6 sent before the host-side teardown (so the notification ring is still
posted and its completion can be seen), four consecutive runs hold **127424,
127552, 127360, 127488**.  It is now sent by default; `--no-end-session`
reproduces the old behaviour deliberately.

Two things this invalidates upward.  **Any multi-run measurement taken before
this is suspect** -- it was a decay against run order with the setting along
for the ride, which is exactly how the `CD2_RESET` wValue sweep produced a
clean-looking monotone out of nothing.  And **the wobbling TX counts were the
same artefact**: 1152, 1408 and 2688 all came from degraded or
abandoned-session runs.  On a healthy part TX is **2432 exactly, every time**,
so the constant is real and is the signature to read.

**The script layer is REFUTED, on a healthy part, across the whole table.**
All fifteen arms of `tools/hsf_tx_gate_sweep.sh` -- six codes (0x02, 0x01,
0x14, 0x13, 0x12, 0x17) in both orderings, plus script 12 alone for three of
them -- give **tx = 2432 without exception**, with RX between 127424 and 133760
throughout, so the part was healthy for every one.  Neither `hsfusbcd2261_`'s
start code, nor the script-8 stream-open pairing, nor the ordering relative to
the session bring-up moves the transmit path at all.  By the test set out when
this lead was opened, the gate is **not in the script layer**, and the
firmware's own OUT consumer is next.

**Live, code 0x17 sent after the session: REFUTED.**  `--post-script 8,12
--post-patch 0x17,0x17` leaves TX at **2432 bytes**, the same figure bare PCM
reaches, while RX delivers 218942 bytes in five seconds with no error.  Script
12 is not being ignored -- it completes on the interrupt endpoint (`data=0c01`)
exactly as 9 and 5 do -- so this is a negative result about the code and the
ordering, not about whether the script runs.  `tools/hsf_tx_gate_sweep.sh`
walks the rest of `hsfusbcd2261_`'s table (0x02, 0x01, 0x14, 0x13, 0x12, 0x17)
in both orderings, with 0x17-after-session as its control; **a run that merely
completes proves nothing here, because the refuted arm completes.**  If TX
still stops at 2432 for every one, the gate is not in the script layer and the
firmware's own consumer is next.

One thing that run corrects in passing: the RX packet size is **not** a fixed
property of the device.  These notes record continuous **64-byte** RX packets;
the same probe on the same part now reports 855 packets of **256** bytes in
five seconds, at 43788 B/s against the earlier 42073 B/s.  Same stream, same
rate, different aggregation -- so read the byte rate, not the packet length.

### The loader is already faithful; the unexercised surface is CD2_RESET

Checked line by line against `OsUsbFWDownload` (`modules/osusb.c:448`), the
loader here matches the vendor's: `SET_CONFIGURATION 0` before the upload and
`SET_CONFIGURATION 1` after (both with their returns ignored, as the driver
does), the notification-pipe drain that discards bogus CDC ACM notifications,
the 50 ms sleep the driver's own comment calls load-bearing ("without this,
downloads occasionally fail"), 64-byte blocks with wValue = total size and
wIndex = running offset, and the ten 100 ms polls afterwards.  The only
divergence is cosmetic -- `CD2_GET_INFROMATION`'s wIndex is the retry counter
in both, and the device answers on the first attempt either way.  **There is no
missing step in the loader.**

What is missing is two whole requests.  Conexant's own header names seven
(`modules/imported/include/usbhalos.h:551`, `CD2REQUESTTYPE`) and the shipped
driver issues **five**: every control transfer in `hsfusbcd2-i386.O` is request
0, 1, 2, 3 or 4.  **`CD2_RESET` (5) and `CD2_WAKEONRING` (6) are never sent at
all.**

One near-miss is worth recording so it is not rediscovered: `hsfusbcd2210_`
calls `hsfusbcd2120_(5)` on the path where the notify pipe is unarmed, which
reads exactly like a reset being issued before the notify request is re-armed.
It is not.  `hsfusbcd2120_` is a two-instruction tail jump to `OsSleep`, so
that 5 is five milliseconds.

The framing of both is therefore unknown -- direction, wValue and wIndex are
guesses, and `hsf_fxo_reset()` uses the shape every other OUT vendor request in
this part uses (device recipient, no data stage).  `--reset` sends it,
`--wake-on-ring` sends the other, and `--reset-value`/`--reset-index` sweep the
fields.  The probe then closes, re-opens and reports what came back, which is
the actual measurement: **family 01 means the bootloader window is open again
without a replug.**

That is the reason to care.  The CD2 bootloader answers EP0 for only about
three seconds after it enumerates, so every firmware experiment is currently
timed by hand against a physical replug.  `libusb_reset_device()` is not an
alternative and has already been tried -- the darwin backend times out
re-enumerating, libusb 1.0.29 crashes on the stale handle, and the device
leaves the bus entirely.  A STALL from `--reset` is a real answer (the device
actively rejecting the opcode at that framing) and is reported as distinct from
an I/O error; the worst case is the replug the workflow already needs.

**Live, firmware running: `CD2_RESET` is ACCEPTED and `CD2_WAKEONRING` is
not.**  Device-recipient OUT with both fields zero, on family 03, request 5
completes (`ACCEPTED (0)`) while request 6 is rejected at all four framings
(device/interface x OUT/IN, STALL each time).  The firmware discriminates, so
acceptance is evidence of a handler rather than of an ignored opcode -- and the
same request 5 at the same framing is STALLED by the **bootloader**, which is
the cleanest control available: one opcode, one framing, two firmwares, two
answers.  What it does is unknown; the family stays 03, so it is not a reset of
the part.

**The wValue sweep that followed is VOID and its numbers must not be quoted.**
It appeared to show wValue 0 giving 127488 RX bytes, 1 giving 64, and >= 2
giving zero -- a clean-looking monotone.  Alternating a wValue = 0 arm between
the others broke it (0 read 0, 0, 127488, 0), and the decisive control was
running with **no `--reset` at all**: four consecutive runs, `rx = 0` every
time.  **The device had degraded into a state where the codec does not start,
and stayed there**, so the sweep was measuring that decay against run order,
not the parameter.  EP0 still answers, scripts 9 and 5 still complete and still
report on the interrupt endpoint, and TX still fills its FIFO -- only the audio
stream is gone, which is why nothing in the per-run output flagged it.

Two procedural consequences for anything driven from this probe:

* **This part degrades across runs and does not recover on its own.**  A
  multi-run sweep must reload firmware between arms, or it measures run order.
  Only a replug reaches the bootloader window, so a sweep of N settings costs N
  replugs -- which is the cost `CD2_RESET` was hoped to remove.
* **Every arm needs a verified-good baseline immediately before it.**  "Scripts
  accepted, device alive, TX accepted" was all true on a part producing no
  audio at all; the only honest health check is RX bytes actually arriving.

**Live, on the BOOTLOADER, device-recipient OUT, wValue and wIndex zero:
STALL.**  The device
actively rejects `CD2_RESET` at that framing.  That is a real answer but a
narrow one, and the run does not yet settle the question, for two reasons.

The first is that the part was **already in the bootloader** when it was sent
(`info = 01 02 01 03 00`), so what was tested is whether the *bootloader*
implements request 5 -- not whether the running firmware does, which is the
case that matters.  Retest with family 03.

The second is a defect in the probe, now fixed, and it is the kind that
manufactures a result: the re-open check reported "the window is open, no
replug needed" on family 01 **alone**, without comparing against the family
before the request.  A device that was already in the bootloader and completely
untouched therefore read as a success.  It now compares, and says
"STILL the bootloader it already was -- proves nothing" for exactly that case;
only a 03 -> 01 transition is reported as a reset.  (The follow-up run makes
the point independently: the window closed on its own about three seconds
later, as it always does, so nothing had re-enumerated.)

The framing is also no longer assumed.  `CD2_CONTROL_SCRIPT` is an
INTERFACE-recipient request in this part while `CD2_UPLOAD_FIRMWARE` is a
DEVICE-recipient one, so the recipient is not a constant and cannot be guessed
for an opcode nobody sends.  `--reset` now sweeps device/interface x OUT/IN and
stops at the first framing that is not rejected; `--reset-rt 0..3` pins one.

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


## Open: hook control and exact sample framing

The transport-start question is closed. The remaining audio-path work is to
identify the two 16-bit slots with a controlled tone, measure the clock over a
longer interval, and verify which script/profile combination physically seizes
the line. Sending script 3 after start did not yield RX in the first trial, so
off-hook must not yet be treated as a single context-free script operation.

### Exact driver off-hook sequence (2026-09-02)

`hsfusbcd2185_` settles the question. For relay codes 2, 3 and 8-11 it stores
the engine-supplied relay word at `ctx+0x8c4`, but never reads or transmits that
word. It calls `hsfusbcd2250_`, which queues the complete script 8 body with
`wIndex=3`, then queues script 3 and clears `ctx+0x8c0`. Thus the country
profile's `0x80B6`/`0x80A6` value is bookkeeping on this USB variant, not a
missing firmware upload.

`hsf_fxo_script_set_hook()` now reproduces that sequence, including the odd
Linux behavior where the script-8 delete normally completes with EPIPE but the
driver proceeds to script 3. Tested live after a clean firmware reload: script
9, script 5 and script 3 all completed with status 1, the data pump delivered
631,552 bytes in 2,467 256-byte packets over 15 seconds with no USB errors, and
the receive values remained the same DC-like 0x0157/0x0000 pair. Therefore no
unreproduced engine profile or relay argument remains in the host driver call
path. Either the accepted script does not seize this particular DAA, or these
four-byte units are not yet being interpreted at their true sample boundary;
an independently observed ATA seizure is the next discriminator.

The hook discriminator is now settled. The attached line is FXS 1 on a
Grandstream HT802 (extension 6004). Its own authenticated status page showed
**On Hook** before the test and **Off Hook** after script 3, so script 3 really
does operate the relay and seize the ATA. A capture begun with the same 9/5/3
run delivered 506,942 bytes over 12 seconds with no USB errors and contained
only the DC-like two-slot values. **That does not yet grade the audio path:**
this ATA is on another network, is not connected to Asterisk, and reports the
port Not Registered, so there is no established reason it should supply dial
tone. The next audio test needs a known analogue stimulus or a registered/live
FXS source; silence from this isolated ATA is not evidence of broken codec
routing.


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


## Running our own code on the device

**The firmware image is CRC-protected, and that is why two patched images were
refused.** CRC-16-CCITT, poly 0x1021, init 0xFFFF, over everything but the last
two bytes: on the stock image that is **0x05b9**, which is exactly the trailing
two bytes. `CD2_UPLOAD_FIRMWARE` returns EIO for any image whose trailer does
not match. The first refusal was blamed on the image growing by 14 bytes, and
that was wrong -- a size-neutral patch was refused too, which is what forced
looking for a checksum.

With the CRC recomputed, **a modified image loads and runs**: the device goes
bootloader (family 01) -> HSF (family 03) and answers normally afterwards. That
is arbitrary code execution on the controller, and it is the prerequisite for
dumping the mask ROM, since nothing in the shipped opcode set reads code memory
and the F4/F5 register file is not code memory either (indices 0x40-0x5f and
0x60-0x7f read identical -- the file mirrors, it does not extend).

`tools/hsf_patch_fw.py` patches and fixes the CRC; `hsf_fxo_probe --rom PATH
--bootloader` loads a chosen image, where `--bootloader` makes `--wait` keep
waiting until the device actually wants firmware -- without it `--load` silently
no-ops on an already-running device, which wasted a whole experiment.

**The result-byte hunt does not work, and live hardware now rules out the
table assumption.** The stub was placed over opcode 0x23's supposed handler at
0x02a9. A minimal four-byte replacement (`MOV 51h,#c3h / RET`) was loaded with
a valid CRC and the device returned to family 03, but executing a script with
opcode 0x23 produced no script completion and only an unrelated empty CDC
notification (`a1 20`). A larger MOVC reader failed the same way. Therefore
0x02a9 is live firmware code but not opcode 0x23's replaceable handler; the
105-entry structure matching the opcode count is not sufficient evidence that
it is the script dispatch. Patching it corrupts another path.

The next trustworthy instrument is a capture of the real Linux vendor stack
starting a call. That observes the engine-to-driver initialization without
guessing at the controller's mask-ROM interpreter.

**Script 11's own contribution to stability is unproven.**  Script 6 alone took
the part from dying on run 2 to four clean runs, and script 6 plus script 11
has now held sixteen -- but the six-run sequence that first included script 11
degraded at run 5, so the two settings have not been separated and the run
count before failure varies by load.  What is solid is that a session must be
ended; how much of `hsfusbcd2265_`'s stop pair matters is not measured.
