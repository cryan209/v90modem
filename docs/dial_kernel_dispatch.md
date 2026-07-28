# The kernel dispatching TIKRNL itself: DM 0x2F28 and the command ring

`docs/dial_under_tikrnl.md` drives the task by calling its frame entry
(PM `0x06BB`) directly. This is the other half: the kernel's own foreground
calling the task, off SPORT0, with the host doing nothing but queue one
command. Harness: `tools/dial_kernel_dispatch.py`.

`docs/dial_sport_drive.md` left this blocked on "the service list at DM
`0x2F28` is empty". It was, but not for the reason recorded there.

## First: DM 0x2F28 is not 0x0F00

The kernel's foreground writes five pointers the moment it wakes:

```
02ad  I0 = $2F27
02ae  DM(I0,M1) = $2F21     ; task registration block
02af  DM(I0,M1) = $2F00     ; host -> DSP command ring descriptor
02b0  DM(I0,M1) = $2F0E     ; DSP -> host descriptor + doorbell
02b1  DM(I0,M1) = $2F42
02b2  DM(I0,M1) = $2F4E
```

These read as `0x0F21`/`0x0F00`/… until `tools/adsp2181_dis.py` was corrected:
the immediate on a `DM(I,M) = <imm>` write is 16 bits, and the old decode
masked it to 12. That one nibble is the whole difference between a pointer
into unpopulated scratch — which is what "the list is permanently empty" was
built on — and a pointer into structures the kernel image already carries.

Nothing has to be invented for DM 0x2F28. It has to be *allowed to happen*:
the foreground only runs when the SPORT0 ISR has queued a sample
(DM `0x2E44` != DM `0x2E45`), which takes three receive slots from reset while
the ISR walks its DM `0x2E50` timeslot countdown.

## The command ring

PM `0x00D8` walks the descriptor at DM `0x2F00`:

| Offset | Meaning |
|---|---|
| +0 | scratch (read, discarded) |
| +1, +2 | carried into AX1 / AY0 for the refill path |
| +3 | byte count, **signed** |
| +4 | read pointer |
| +5 | ring base — `0x2800` |
| +6 | ring size — `0x0300` words |

The sign of +3 is the byte phase: `0` is empty; `> 0` means the next byte is
the low half of the word at +4; `< 0` means the high half. Each read flips the
sign and decrements the magnitude, and the negative case also advances +4,
wrapping at base+size (PM `0x00EA-0x00F0`). Two 768-word rings sit back to
back below the kernel's timeslot buffer: `0x2800-0x2AFF` inbound and
`0x2B00-0x2DFF` outbound, ending exactly at DM `0x2E00`.

PM `0x01B2` reads two bytes and assembles them little-endian; PM `0x02A1`
loads the result into I4 and calls it:

```
02a1  CALL $01B2
02a2  AR = AR + 0
02a3  I4 = SR0
02a4  IF EQ CALL (I4)
```

So a host command *is* a PM address to call — one 16-bit word per command.
Planting `0x06BB` in DM `0x2800` with count 2 makes the kernel call TIKRNL's
frame entry; that was the first confirmation the format was right.

## Bootstrapping: the task claims the dispatch slot

Queue TIKRNL's task entry, PM `0x0672`. Its init reaches kernel service
`0x0017` (PM `0x0281`), which builds a `CALL <vector>` instruction word at
PM `0x0294`:

```
0294  SR1 = $1C00                        ; CALL opcode
0295  SR0 = $0F00
0296  SR = LSHIFT AR (HI, OR) BY -4      ; shift the vector in
0297  SR = LSHIFT SR0 (LO, OR) BY -8
0298  PX = SR0
```

and patches it over two words named by the registration block at DM `0x2F21`:

```
DM 2f24 = 0x02b9   ->  PM 02b9: CALL $02A1               ->  CALL $06FC
DM 2f25 = 0x00b5   ->  PM 00b5: AR = SR0 + 0, SR0 = AR   ->  CALL $08F6
```

PM `0x02B9` is the foreground's per-sample dispatch and PM `0x00B5` is inside
the SPORT0 ISR. PM `0x08F6` is `JUMP $0582`, which lands in TIKRNL's own copy
of the block it overwrites (PM `0x0580-0x05EB`), where `0x0582` is
`AR = SR0 + 0, SR0 = AR` — the very instruction it displaced. The kernel keeps
the original word (`0x2A7EEA`) in the un-patch path at PM `0x0290-0x0292`, so
this is a general "claim an inline op" mechanism, not a TIKRNL special case.

The sample travels in SR1: the foreground reads it out of the queue at
PM `0x02B8`, calls the task, and writes SR1 back at PM `0x02BC`. TIKRNL's
continuation scales it in at PM `0x06FC` (`SE = DM(0x31B7)`) and out at
PM `0x07E3-0x07E5` (`MY0 = DM(0x31B6)`). PM `0x06FC` is the per-sample data
pump, not a per-frame entry.

**This was previously recorded as patching PM `0x0000` and PM `0x000A`.** That
was an artifact: with the pointer block still zero, PM `0x0283` indexes DM
`0x0002` instead of DM `0x2F23` and the patch lands on the kernel's vector
table. Initialise DM `0x2F27` first and it lands where the card puts it.

## The doorbell: how the task asks the host for something

`JUMP $000A` with a bit in AR reaches PM `0x01DE`:

```
01de  I0 = DM($2F29)      ; 0x2F0E
01df  M3 = 9
01e0  MODIFY(I0, M3)      ; -> 0x2F17
01e1  AY0 = DM(I0,M0)
01e2  AR = AR XOR AY0
01e3  DM(I0,M0) = AR
01e4  FLAG_OUT(8f)
01e5  FLAG_OUT(cf)
```

so DM `0x2F17` is a service-request bitmask the DSP toggles, followed by a host
interrupt. Bit *i* names slot *i* of the task's entry table at DM `0x31BA`:

| AR | Bit | Entry | Meaning |
|---:|---:|---|---|
| 1 | 0 | DM `0x31BA` = `0x06BB` | run my frame entry |
| 2 | 1 | DM `0x31BB` = `0x06D8` | the overlay I asked for is loaded |

TIKRNL raises bit 0 at PM `0x06B9-0x06BA` and bit 1 at PM `0x069D-0x069E`,
right after publishing the download in DM `0x31A9`/`0x31AA`. That is the same
request `docs/dial_under_tikrnl.md` serves by calling `DM(0x31BB)` directly;
here it arrives as a doorbell.

## Result

600 SPORT0 receive slots, 2100 Hz μ-law, host action limited to one queued
command:

```
[card] before the foreground runs: DM 2f27..2f2b = 0000 0000 0000 0000 0000
[card] after 3 SPORT0 slots:       DM 2f27..2f2b = 2f21 2f00 2f0e 2f42 2f4e
[card] command ring descriptor DM 2f00: count=0000 ptr=2800 base=2800 size=0300
[host] queued the task entry PM 0672 as a host command
[card] sample 0: the kernel dispatched PM 0672 from the ring
       PM 02b9 = 1c6fcf   PM 00b5 = 1c8f6f
[card] kernel ring dispatch PM 02a1: 1   task continuation PM 06fc: 599
```

One command in, and the kernel calls the task on every one of the remaining
599 samples with no host involvement — PM `0x02A1` never runs again, because
the task owns the slot that used to call it. That is the SPORT0-driven,
kernel-dispatched path `docs/dial_sport_drive.md` concluded was unreachable
without the MIPS side.

## Recovered: how the host hands the resume back

The task parks after publishing the SIG overlay request (DM `0x31A9` =
`0x000D`, DM `0x31AA` = `0x0270`) and raising doorbell bit 1. Merely queuing
`DM(0x31BB)` = `0x06D8` in the byte command ring cannot work: claiming PM
`0x02B9` took PM `0x02A1` out of the loop, and a pending download keeps the
sample continuation away from the task's own command fetch:

```
0705  ASTAT = DM($31A9)
0706  IF EQ JUMP $07D2      ; request outstanding -> pass audio, do nothing else
```

The completion instead uses the slot TIKRNL registered for precisely this
purpose. After downloading the requested image, the host temporarily changes
PM `0x02B9` from `CALL 0x06FC` to `CALL DM(0x31BB)` (`CALL 0x06D8`) for one
SPORT0 dispatch, then restores `CALL 0x06FC`. This is the same instruction
encoding kernel service `0x0017` builds at PM `0x0294-0x0298`; the real host
can perform the one-word program-memory change over IDMA.

The one-shot call reaches:

```
06d8  ASTAT = DM($31A9)
...
06dd  DM($31A9) = M0
06e2  CALL $08F0
06e8  CALL (I4)
```

so the request is acknowledged, the freshly downloaded page is entered, and
the next SPORT0 slot is back on the ordinary `0x06FC` sample continuation.
`KernelDispatch.resume()` models this handback.

With the ADDSP guide §5.4.1 calling setup programmed, the recovered live
sequence is:

```
SIG (0x0270) -> DIAL (0x0262) -> DIAL partial (0x0263) -> V.8 (0x025f)
```

Verification:

```bash
python3 tools/dial_kernel_dispatch.py \
    --dial-v8 --freq 2100 --samples 100
```

The run reports `resident=0x025f`, four `0x06D8` completion entries and V.8
entered through the shared PM `0x08F0` overlay interface.

## SPORT register and assigned-line requirements

The Eicon kernel legally performs `RX1 = SR0` at PM `0x00AE` and
`SR0 = TX1` at PM `0x00B8`. The ADSP-218x Instruction Set Reference §4-117
allows RX0/RX1/TX0/TX1 on either side of a register-to-register MOVE. The
original emulator inherited directional-only register handlers, discarded
the RX1 write and returned zero for the TX1 read. The core now models all
four SPORT data-register latches; external receive events load RXn and TXn
writes still invoke the wire callback.

Two values normally supplied by the MIPS channel-assignment database are also
required after the V.8 download:

- DM `0x32F0 = 0x0004`: TIKRNL loads this word into ASTAT and tests AV to
  select pointer-mode RX/TX buffers at `0x3F0F` and `0x3FB4`.
- DM `0x3F08` bit 5: marks the line descriptor assigned. With it clear,
  V.8's PM `0x2000` line entry interprets the word as a page selector and
  requests page 6 again instead of leaving its processing vectors active.

With both present, the kernel-dispatch verification reaches V.8 PM `0x08F1`
and runs the V.8 action vector at PM `0x204A` from the `0x06FC` sample
continuation.

## ANSam response check

`--stimulus ansam` generates the calling-side test input specified by
ITU-T V.8 (11/2000) §7.2: a 2100 Hz carrier, a 15 Hz sinusoidal amplitude
envelope ranging from 0.8 to 1.2 of average amplitude, and 180-degree phase
reversals every 450 ms. A two-second run is:

```bash
python3 tools/dial_kernel_dispatch.py \
    --dial-v8 --stimulus ansam --samples 16000
```

The transmit report is deliberately measured only after V.8 activation.
The current result is **no V.8 response**: of 15,992 post-activation SPORT0
transmit words, 15,937 were zero, 54 were μ-law silence (`0x00FF`), and one
was `0x0400`. A single isolated control-looking word is not an output
waveform. Thus overlay residency, PM `0x08F1`, and PM `0x204A` execution are
confirmed, but ANSam detection/CM transmission is not.
