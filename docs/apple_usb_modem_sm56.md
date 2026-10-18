# The Apple USB Modem (A1082) as an analogue side

`05ac:1401`, USB vendor string `Motorola, Inc.`, product `Apple USB Modem`.  A
**Motorola SM56 softmodem**: the device is a codec plus a line interface and
nothing else, so V.8, V.34, V.90, the AT interpreter and the DAA control all
ran on the host.  Its macOS driver was withdrawn years ago, which is why the
part enumerates and then does nothing.

Interest here is as an **analogue side over a real 2-wire line**, the role
`hsf_fxo_probe.c` / `hsf_v90_coupler` plays for the Conexant 0572:1300 in
`docs/hsf_usb_daa.md`.  The audio transport is solved and needs no work; the
line control is not.

## GUI entry point

`make gui-apple` launches an on-hook console with device pickers, hook/sense
readback, on-hook raw audio monitoring, ring watching and S0 auto-answer.
Raw codec audio is available before engine startup and during dialling;
engine eye and constellation remain separate. Optional RX/TX captures include
rate metadata and can be replayed without USB. Ring watching also keeps the
on-hook caller-ID receiver active; live CID now passes on both attached modems.
Diagnostics report line status/current, an uncalibrated voltage estimate, raw DC/RMS,
near-rail samples and transport counters. Profile 0 and hardware choices persist
across the fresh backend processes used for each call.

Full controls, capture locations and hardware-free checks are documented in
`docs/gui_mode.md`. The transport, resamplers and training algorithms are
unchanged (V.90 §5 analogue role; V.34 §10.1 training). Ring burst counting and
on-hook monitoring require live hardware validation; this integration makes
no new CONNECT or interoperability claim.

## What the descriptors say

One configuration, six interfaces, bus powered at 50 mA:

```
if 0  CDC ACM control (2/3/0)      ep 0x81 interrupt IN, 11 B
if 1  vendor (255/2/255)           no endpoints          <- command channel
if 2  Audio Control (1/1/0)        no endpoints
if 3  AudioStreaming (1/2) alt 1   ep 0x82 ISOCHRONOUS IN,  24 B/frame
if 4  AudioStreaming (1/2) alt 1   ep 0x03 ISOCHRONOUS OUT, 24 B/frame
if 5  DFU runtime (254/1/1)        wTransferSize 1024, bcdDFU 1.10
```

**There is no bulk endpoint and no CDC data-class interface.**  The CDC union
descriptor names interfaces 1-4 as slaves of interface 0, but with no data
interface there is no serial pipe, so nothing on the device could ever have
carried AT traffic or a data stream.  That is the structural proof that the
datapump is host-side, and it also means `cdc-acm` could never have worked
regardless of the subclass — the 2008 Linux threads on this device blamed
class 2/subclass 3 vs 2/2, which is true but not the reason.

The audio format descriptor is the other half of the proof.  Both streams are
PCM, mono, 16-bit, `bSamFreqType = 7`:

**7200, 8000, 8229, 8400, 9000, 9600, 10286 Hz**

That is 8 kHz plus **every V.34 symbol rate times three** — 2400·3, 2743·3,
2800·3, 3000·3, 3200·3, 3429·3.  A device with its own datapump has no reason
to publish a rate menu at all, let alone one that is exactly the host's T/3
grid per symbol rate.  9600 Hz is the rate this tree's own T/3 upstream
receiver already runs at for 3200 baud.

## State on a modern macOS, and the one non-obvious step

The device sits at **`bConfigurationValue = 0`**.  Nothing claims it, so
nothing sets the configuration, so *every* request stalls — including standard
`GET_INTERFACE` — and `ioreg` shows the device with **no `IOUSBHostInterface`
children at all**.  This reads exactly like broken hardware and is not.

After `libusb_set_configuration(h, 1)`:

- standard requests answer;
- `usbaudiod` immediately claims interfaces 2, 3 and 4, and the modem appears
  to CoreAudio as `Apple USB Modem`, uid
  `AppleUSBAudioEngine:Motorola, Inc.:Apple USB Modem:000000:3,4`, 1 channel
  in and 1 out, with all seven rates offered;
- libusb can therefore no longer claim 3 or 4 (`LIBUSB_ERROR_ACCESS`), though
  interfaces 0, 1 and 5 stay available.

So **the codec transport needs no bring-up sequence** — no equivalent of the
Conexant part's script 5/9, no firmware push.  Capture and playback are
ordinary CoreAudio.

## The codec is railed until something powers the analogue path

Capturing the input through CoreAudio at each rate yields **one distinct
sample value for the whole capture**:

```
rate     frames   distinct values   constant
7200      7168    1                 -30069
8000      7680    1                 -31703
9600     28672    1                 -32768   (2.99 s)
10286    10240    1                 -32768
```

RMS about the mean is exactly zero, so there is no noise floor to quote.  The
transport is fine: frame counts track the nominal rate, and the driver reports
the input stream's physical format as `9600 Hz lpcm, signed int, 16 bits,
1 ch`, which is what was requested.  Asking the audio unit for float32 instead
gives a constant −1.0, so two independent paths agree the data really is a
constant at negative full scale.

**The constant changes with sample rate, and that is the informative part.**  A
driver fill pattern or a substituted buffer would be rate-independent.  A
DC-saturated analogue front end read through a decimation chain whose DC gain
differs per rate produces a slightly different clipped constant at each rate,
railing outright at the higher ones.  So there is a real ADC running, with a
real filter chain, and its input stuck at a rail.

**The rail was the unpowered analogue path, and one register bit clears it.**
The previous reading of these captures said the rail was equally explained by
there being no line in the jack, and that the two could not be separated
without one.  That is now settled without a line, because the separation is a
command rather than a connection: with **register 5 bit 3 set** the same
capture at the same rate stops railing and delivers a real signal.  Measured
back to back on one device with nothing in the tel jack, the bit as the only
variable:

```
reg 5 = 0x00   min/max -32768 / -32768   ONE distinct value
reg 5 = 0x08   min/max    570 /    738   DC +657 counts (2.01% FS)
                                         RMS excl. DC 21.2 = -63.8 dBFS
                                         about 4.4 effective bits of noise
```

Repeatable in both directions (set, clear, set), so it is the analogue path's
power switch and not an artefact of when the capture ran.  **-63.8 dBFS is the
codec's own floor on an open line** - it is not a measurement of a bearer, and
a line in the jack will be worse.  What it does establish is that the front
end, its filter chain and the isochronous transport all work, which is the
whole of what the analogue side needs from this part.

## Driver provenance

macOS, introduced in 10.4.3, both 32-bit and both long gone:

| file | location on 10.6 |
|---|---|
| `MotorolaSM56KUSB.kext` | `/System/Library/Extensions/IOSerialFamily.kext/Contents/PlugIns/` |
| `SM56KUSBAudioFamily.kext` | `/System/Library/Extensions/` |

Workable on Lion only by copying both from Snow Leopard *and* booting a 32-bit
kernel; dead from Mountain Lion, which has no 32-bit kernel.  Nothing remains
on a current system — `IOSerialFamily.kext` no longer has a `PlugIns`
directory.

**The Windows driver Boot Camp shipped is the better target and is
obtainable.**  In Windows the device is `Motorola SM56 USB Data Fax Modem`,
installed by `MotorolaSetup.exe` from the Boot Camp Drivers folder.  The
10.7.5 Boot Camp support software on the Internet Archive
(`archive.org/details/motorola-setup`) offers that file individually at
`WindowsSupport/Drivers/MotorolaSetup.exe`, 1,840,991 bytes,
sha256 `f22dc4057e41f28c9f5698054fb512f9032ca6a3ed5352847fc1586123414483` —
no need for the 732 MB set or the 989 MB Snow Leopard Boot Camp ISO
(`archive.org/details/bootcamp3`).

It is a **solid** RAR SFX.  Homebrew's 7-zip has no RAR codec
("Unsupported Method", and it creates the tree as zero-byte files, which looks
like success); libarchive refuses with "RAR solid archive support
unavailable".  `unar` opens it.

`usmserial.inf` binds `USB\VID_05AC&PID_1401` — this exact device — plus
`190D:0001` and `190D:012C`, and is headed `Copyright Motorola, Inc.
2004-2007`, which settles the occasional claim that the part is Conexant.

## Driver architecture

```
USmSerial.sys   919,680 B   lower PnP filter; modem + datapump + AT
                            pdb d:\prj-modem\6.12.08\uni_drvr\sys\i386\
                            creates \Device\USMSERIAL, \Device\USmSrl
                            opens  \Device\MdbPass
utlamot.sys      25,984 B   upper filter, "Base" group; ALL USB I/O
                            pdb d:\prj-modem\6.12.08\uni_drvr\usb\...
                            imports USBD.SYS, 6x IOCTL_INTERNAL_USB_SUBMIT_URB
sm56.dll        202,016 B   user space
usm56hlpr.exe   660,768 B   user-space helper
usm56.reg       317,373 B   REGEDIT4 text, per-country DAA data
```

`USmSerial.sys` imports no USBD function and contains no `0x220003`; all USB
traffic is `utlamot.sys`.  **That is the file to read: 26 KB, not 900 KB.**
Ghidra headless decompiles it into 87 functions in under a minute.

## The command protocol

`utlamot.sys` builds an intermediate request descriptor and marshals it into a
`_URB_CONTROL_VENDOR_OR_CLASS_REQUEST`:

```
desc+0x00 URB Function -> urb+0x02      desc+0x0e wValue -> urb+0x4a
desc+0x08 TransferFlags-> urb+0x14      desc+0x10 wIndex -> urb+0x4c
desc+0x0c ReqTypeBits  -> urb+0x48      desc+0x12 Length -> urb+0x18
desc+0x0d bRequest     -> urb+0x49      desc+0x16 buffer -> urb+0x1c
                                        urb+0x00 = 0x50
```

Every request uses `URB_FUNCTION_CLASS_INTERFACE` (0x1b), i.e. a **class
request to an interface**, never a vendor request.  Two families:

**Family A — encapsulated command/response, `wIndex = 1` (interface 1).**
`bRequest 0x00` = CDC `SEND_ENCAPSULATED_COMMAND` (OUT),
`bRequest 0x01` = `GET_ENCAPSULATED_RESPONSE` (IN, 2 bytes).
Payloads are 3 bytes, once 9.  Complete set built by the dispatcher
`FUN_0001316a`, with the private IOCTL from `USmSerial.sys` that selects each:

| IOCTL | bRequest | wValue | wIndex | len | payload |
|---|---|---|---|---|---|
| `0x2200b4` | 0x00 | 0 | 1 | 9 | `02 05 FE 20 <lo> <hi> 01 05 01` |
| `0x2200b8` | 0x00 | 0 | 1 | 3 | `D0 00 00` |
| `0x2200c4` | 0x00 | 0 | 1 | 3 | `D0 00 00` |
| `0x2200c4` | 0x00 | 0 | 1 | 3 | `90 00 00` |
| `0x220088` | 0x00 | 0 | 1 | 3 | `00 <lo> <hi>` |
| `0x22008c` | 0x00 | 0 | 1 | 3 | `80 <arg> 00` |

**Family B — parameter in `wValue`, `wIndex = 0` (interface 0), no data.**

| IOCTL | bRequest | wValue | wIndex | len |
|---|---|---|---|---|
| `0x2200a4` | 0x14 | 16-bit arg | 0 | 0 |
| `0x2200a0` | 0x13 | 16-bit arg | 0 | 0 |
| `0x22008c` | 0x11 | 16-bit arg | 0 | 0 |

A separate function (`FUN_00012f46`) issues `GET_ENCAPSULATED_RESPONSE`
(`bRequest 0x01`, `wValue 0`, `wIndex 1`, 2 bytes) in three variants selected
by a mode byte at `devext+0x672` (values 1, 4, 6).

Observed opcodes in byte 0 of a command: `0x00`, `0x02`, `0x80`, `0x90`,
`0xD0`.  Observed lengths: 2, 3 (six sites), 9 (one).

**Verified live:** interface 1 accepts `SEND_ENCAPSULATED_COMMAND` and
`wIndex = 0` stalls it; `GET_ENCAPSULATED_RESPONSE` at `wIndex = 1` answers
zero-length when nothing is queued.  A full IN-direction sweep of
`bRequest 0x00-0xFF` across vendor/interface, vendor/device and class/interface
at `wValue` 0, 0xFF01, 0xFF02, 0x0100, 0x0001 produced exactly one non-stall
reply — that `GET_ENCAPSULATED_RESPONSE` — which matches the driver using
class requests and nothing else.  The device stayed alive throughout.

## The device is a register file, and that is the line interface

Tracing those IOCTLs back into `USmSerial.sys` identifies two of them as a
register read/write pair, and **that pair is how the driver does everything to
the line**.  The callers are thin wrappers:

| helper | IOCTL | what it puts on the wire |
|---|---|---|
| `ReadReg(idx)` @`0x9ca6b` | `0x22008c` | `80 <idx> 00`, then `GET_ENCAPSULATED_RESPONSE` |
| `WriteReg(idx,val)` @`0x9ca88` | `0x220088` | `00 <idx> <val>`, silent |

Both are `SEND_ENCAPSULATED_COMMAND`, `wValue = 0`, `wIndex = 1`, 3 bytes.  The
table above listed those two bodies as `00 <lo> <hi>` and `80 <arg> 00` with
the fields unnamed; they are index and value, and the `0x80` form is a read.

**Verified against the device.**  Every index tried but 0 answers, repeatably
and in order:

```
reg 0x01 = 0x01   reg 0x05 = 0x00   reg 0x11 = 0x0c
reg 0x02 = 0x83   reg 0x0a = 0x00   reg 0x1f = 0x20
0x03 0x04 0x0f 0x10 0x1a 0x1e = 0x00      0x00 answers nothing
```

Writes take and read back (`reg 5: 0x00 -> 0x08 -> 0x00`), and the device stays
alive throughout.  `apple_usb_modem_probe --regs`, `--read`, `--write`.

### Hook control

There is a pair of functions at `0x9dabe` / `0x9db43`, each branching on the
product ID - which is why one driver covers this part and the `190D:*` ones in
the same INF:

```
9dace: cmp dword [esi+0x3b3], 0x1401    <- this device
9dadc: push 5 ; call ReadReg
9dae3: or   eax, 0x8
9dae7: push 5 ; call WriteReg           <- set bit 3
9db31: push 2 ; call SendCmd(0x22009c)  <- the other PIDs, a different scheme
```

and the mirror at `0x9db43` with `and eax, 0xfffffff7`.  **That pair is the
receive path, NOT the hook** -- see the live results below; bit 3 is the on-hook
monitor and bit **0** is the loop closure.  The `190D:*` branch's
`SendCmd(0x22009c)` reaches register 5 too, by the other route described below.

**How they are reached, because two obvious searches both come up empty.**
Neither function has a direct caller and neither has a relocation pointing at
it.  The pointers are installed at RUN TIME:

```
9d66d: mov dword ptr [0xe675c], 0x9dabe
9d677: mov dword ptr [0xe6760], 0x9db43
```

two adjacent slots in a ~20-entry ops table at `0xe6750`-`0xe67b0`, filled by an
init routine, and called through a pair of idempotent wrappers at `0x3539b` /
`0x353c3` that guard on a state byte at `ctx+0x5305`.  **A pointer written by
`mov imm32` has no relocation and no call site, so both a caller search and a
`.reloc` scan report nothing** -- grep the disassembly for the GLOBAL's address
instead.  An earlier version of this file claimed a 10-byte-stride pointer
table at file offset `0x8d673`; that is **WRONG and withdrawn** -- those four
dword matches are x87 instruction bytes in the datapump (`fst`, `fadd`,
`fsub`), which is what a byte search for a code address will find in a 900 KB
binary.  Search for a stored pointer by its relocation or not at all.

**And with a line connected, bit 3 is NOT the hook.**  See below; the codec A/B
still holds, so it is the analogue receive path's power switch, but it does not
seize the line.

### The country blobs reach it the same way

At `0x9dd96` there is a table at RVA `0xce218` of 8-byte entries -
`{u32 profile, u8 r10, u8 r1a, u8 r1f, u8 r1e}`, `-1` terminated, default
`00 C0 00 00` - walked for the current profile and written as four `WriteReg`
calls to registers **0x10, 0x1a, 0x1f, 0x1e**.

**`HardwareInitBB` is the KEY into that table, not its contents.**  An earlier
version of this file said its four bytes were the four register values; they
are a single byte plus padding - **NZ 0x3e, AU 0x04, UK 0x5e, US 0x60** - and
NZ's row is `r10=0xa0 r1a=0xc0 r1f=0x00 r1e=0x00`.  Written to the device and
read back verbatim (`r1f` reads 0x20 out of reset and accepts 0x00).  The four
registers were right; where the values come from was not.

Note the address trap: objdump prints `.text` addresses as RVAs but an absolute
data operand as a VA, and this image's base is `0x10000` - so the code's
`[0xde218]` is RVA `0xce218`, and `0xde218` is in no section at all.

That also disposes of the objection that none of the blobs is 3 or 9 bytes long
and so cannot be carried by these commands: the channel does arbitrary
single-register writes, and a blob is delivered as a run of them.  The three
larger blobs (32, 500, 229 bytes) are still unplaced.

## With a line connected: off-hook, and the dial tone proves it

**The first attempt at this was worthless because the pair was not actually
connected**, and everything below supersedes it.  With a real pair into a Cisco
VG224 FXS port:

**Register 5 bit 0 (0x01) is the hook.**  Setting it seizes the line, and the
ground truth is the tone the exchange returns, resolved over a 4 s capture at
9600 Hz:

```
350.0 Hz   33.2% of power    0.0 dB
440.0 Hz   30.9% of power   -0.3 dB      RMS -15.9 dBFS
 50.0 Hz    0.0%          -53.5 dB
```

350 + 440 Hz at equal level is North American dial tone (this VG224 is on US
tones), and **the mains hum that dominates the on-hook capture is now 53 dB
down** -- loop current closes the loop and drops the line impedance, which is
exactly the corroboration a real seizure should come with.  Clearing bit 0
releases it.

**Register 5 bit 3 (0x08) is the receive path / on-hook monitor.**  The codec
A/B that first found it still holds, but it does not seize anything: on-hook
with a line it gives hum at -38 dBFS and no dial tone, and with no line the
codec's own -63.8 dBFS floor.  Mapped against dial tone, register 5 reads:

| reg 5 | line sense 0x1d | capture |
|---|---|---|
| 0x00 | 217 | railed at -32768 |
| 0x01 | 250 | **-16.0 dBFS, 99.8% in 300-600 Hz, dial tone** |
| 0x08 | 217 | -38.1 dBFS, 90% below 300 Hz, hum only |
| 0x09 | 250 | dial tone, as 0x01 |

**With no line, bit 0 rails the input** -- there is nothing to draw current from
-- which is exactly why the no-line session read it as a reset or override.  The
two earlier readings were each self-consistent and both incomplete.

### Register 0x1d is an analogue line sense, and it is the instrument to use

Not a bit field: **0x00 with no pair connected, 0xd8-0xda on-hook with a pair.**
One control request against a whole CoreAudio capture, so use it in preference
to listening, and **0x00 is the device telling you the pair is not connected** --
the check that would have saved the first session.

**Do not read more than that into it.**  Off-hook it has been seen at 0x06,
0x13, 0xfa and 0xfb depending on what the line was doing, so it is an analogue
reading (level or loop voltage), not a state code; the tool interprets only the
zero.  An earlier three-band classifier here was fitted to two observations and
called an established call "unexpected".

It is NOT a hook mirror.  It was first seen to differ between an on-hook and an
off-hook scan and that was coincidence -- polled undisturbed it wanders
0xd8/0xd9/0xda, and the two scans caught different samples of the same jitter.
One observation of a difference is not a measurement of a correlation.

### The `wIndex = 0` family is another route to register 5

`bRequest 0x11` to interface 0, parameter in `wValue`, no data -- the family this
project had never sent live -- **writes register 5**: `wValue 1` leaves it at
0x01, `wValue 2` reads back 0x00 because bit 1 is not implemented, and anything
from 3 up stalls.  That is also what the driver's `190D:*` hook path sends
(`SendCmd(2)` / `SendCmd(0)` through IOCTL `0x22009c`), so the two branches reach
the same register by different transports rather than being different schemes.
It does reset the rest of register 5, so it and a read-modify-write of the
register do not compose -- pick one.

### What does nothing, so it need not be re-tried

The driver only ever writes **eight** registers -- 5, 0x0a, 0x0f, 0x10, 0x11,
0x1a, 0x1e, 0x1f -- and with the line sense as the instrument, all but register
5 leave the line untouched: 0x0f swept 0-7 (a 3-bit field, so the likeliest
shape for a relay code), 0x11 across every writable bit (0, 1 and 4 are read
only), the 0x10/0x1a/0x1f/0x1e country profile, and `0x0a = 1`, which mutes to
exact digital silence rather than the rail -- so a mute and an unpowered path
are distinguishable.  The driver's `10 00 00` and `10 00 08` session commands
are accepted and change nothing on the line.

**And the 9-byte `0x2200b4` form is dead code**: the constant appears in
`utlamot.sys` and in nothing else -- not `USmSerial.sys`, not `sm56.dll`, not
`usm56hlpr.exe` -- so nothing in the shipped stack ever sends it, and it is not
how this part's DAA is programmed.

**Other opcodes remain unnamed.**  `0x2200c4`'s first sub-form emits
`10 00 <n&0x0f>` and is what `USmSerial.sys` calls with 0 and 8 around session
setup and teardown; `0x2200b4` emits the one 9-byte body,
`02 05 FE 20 <lo> <hi> 01 05 01`, with `bRequest 0x00` and a mode flag of 2,
which looks like a windowed write; `0x90` and `0xd0` appear once each.  The
`wIndex = 0` requests (`bRequest` 0x11/0x13/0x14) have still never been sent
live.

## usm56.reg: the per-country DAA data

REGEDIT4 text, 112 countries under
`HKLM\Software\Motorola\USMSERIAL\CurrentVersion\<dialling code>`.  New Zealand
is `64`, Australia `61`, UK `44`, US `1`.  Each carries four blobs with a dword
checksum each:

```
CountryInitBB    32 bytes    DialInitBB   500 bytes
LimitInitBB     229 bytes    HardwareInitBB 4 bytes
```

These are the ring thresholds, pulse-dial timing and off-hook current limits —
the regulatory configuration the command channel exists to deliver.
`HardwareInitBB`'s four bytes are the four DAA registers written at country
selection; the other three blobs are not placed, and the 9-byte form acting as
a windowed write is still a guess.

The seven sample rates appear **nowhere** in the INF or the `.reg`.  The
`9600`/`8400`/`7200` hits in the INF are `CONNECT 9600` response strings, i.e.
DTE rates.  Rate selection is in the binary.

## Transmit, and a call placed over the line

The output stream is the other half of the same CoreAudio device, so the
transmit path is a render callback on **element 0** of the same HAL unit that
element 1 captures with -- `apple_usb_modem_audio tone` and `dial` run both at
once, which is the point: what proves a digit reached the line is the far end's
reaction, and that arrives on the receive side while transmission is still
going.  (Watch the scopes: the capture format is set on `kAudioUnitScope_Output`
of element 1 and the transmit format on `kAudioUnitScope_Input` of element 0 --
opposite scopes on different elements.)

**On-hook, a transmitted tone shows up nowhere, and that proves nothing.**  The
loop is open, so there is no circuit; this test cannot distinguish a dead
transmit path from an open line.  Do it off-hook, where the dial tone is a
built-in reference:

```
1000 Hz transmitted at amplitude 0.15, off-hook, 9600 Hz:
  350 Hz  30.10% of power      dial tone
  440 Hz  32.36% of power      dial tone
 1000 Hz   0.89% of power      -15.6 dB rel -- OUR OWN TRANSMIT
```

so the transmit reaches the line and returns through the 2-wire hybrid about
15.6 dB down, which is the ordinary trans-hybrid loss.

**DTMF works: the exchange mutes the dial tone on the first digit.**  Q.23
pairs, 100 ms on / 100 ms off, per 50 ms window:

```
 time      rms   dial tone   DTMF '1'
 0.20     3433      210.6       90.8
 0.30     4871       73.4       76.3
 0.40     4976       88.9      698.6     <- our digit
 0.50      615       25.1      374.0
 0.60       13        1.4        0.3     <- dial tone gone, still off-hook
```

and it never returns, while the hook stays off for the remaining six seconds.
That is the exchange accepting the digit, and it is the ground truth for the
transmit path in the same way the dial tone was for the hook.

**A full call completes.**  Dialling `8416`, the RasFinder's extension, at
100/120 ms: dial tone, digits out from 0.6 to 1.5 s, PBX audio, then at
**6.60 s onward 2100 Hz at amplitude 6595 with the peak bin exactly 2100 Hz** --
the answering modem's ANS/ANSam.  So this part can seize a line, dial through
the PBX and reach a far-end modem, which is everything the analogue side needs
below the datapump.

`APPLE_MODEM_TX_AMP` sets the per-tone amplitude (default 0.15, a pair landing
near -16.5 dBFS, comparable to the dial tone arriving at -15.9);
`APPLE_MODEM_DTMF_MS` the on/off times.

## Coupling it to the engine

`apple_usb_modem_coupler` is the role `hsf_v90_coupler` plays for the Conexant
part: seize the line, DTMF an extension, wait for the far end's answer tone, run
the engine over the result, DTE on a PTY.  Line control is USB (the register
file above), the bearer is CoreAudio, and it links the whole engine minus
`sip_modem.o` -- the same object set the HSF coupler does.

```
apple_usb_modem_coupler --dial 8416 [--pty-link /tmp/applemodem]
apple_usb_modem_coupler --rx-replay tap.s16 [--pty-link ...]
apple_usb_modem_coupler --hook on|off
```

### The rate is 9600, not 8000

The rate list is 8000 plus every V.34 symbol rate times three, because the
SM56's host datapump ran its receiver on a T/3 grid.  The engine wants two
grids -- 8000 for `me_rx_audio()` and 16000 (T/2) for `me_rx_v90a_16k()` -- and
of the seven offered rates **only two reach both by a small exact ratio**:

| device rate | to 8000 | to 16000 |
|---|---|---|
| 7200 | 10/9 | 20/9 |
| **8000** | **1/1** | **2/1** |
| 8229 | 8000/8229 | 16000/8229 |
| 8400 | 20/21 | 40/21 |
| 9000 | 8/9 | 16/9 |
| **9600** | **5/6** | **5/3** |
| 10286 | 4000/5143 | 8000/5143 |

**The difference between the two is the whole argument.**  `8000 -> 16000` is
x2, i.e. **upsampling**: it invents the T/2 samples by interpolation instead of
measuring them, from a stream that is already critically sampled -- V.34 at 3429
baud occupies up to 3673 Hz, leaving 8000 just **327 Hz** of Nyquist margin.
9600 leaves **1127 Hz**, and `9600 -> 16000` carries genuine information to
4800 Hz, covering the whole DS0 band.  So **8000 is a dead end for the V.90
analogue role**, whose downstream is recoverable only from T/2 samples, while
9600 reaches both grids *and* is T/3 at 3200 baud exactly.  Default 9600.

**And exact rational resampling is also what keeps the HSF path's defect out of
here.**  That coupler decimates 16 kHz by two, so it must CHOOSE which of two
sample sets to keep, and swept as a fractional delay only 1/10, 2/10 and 9/10 of
phases reached Phase 4 across three recorded calls, with 0.0 -- the obvious
value -- failing on all three (`docs/hsf_analogue_v90_coupler.md`).  A 5/6
polyphase discards nothing and has no free parameter; the constant group delay
it adds is not a choice.  There is no `HSF_RX_DELAY` equivalent because there is
no equivalent decision.

`--rate 8000` still works and skips the receive resampler, which is one filter
fewer if all you want is V.34; it cannot feed the T/2 path.

**The resamplers are measured, not asserted** (`--selftest`, no device or line
needed): a tone through each path, fitted for amplitude and phase, with the
residual reported as SNDR.  All three are flat to **0.3%** with **52.7-84 dB**
SNDR over 300-3673 Hz, i.e. across the whole V.34 band including its worst
symbol rate.

**The DC blocker is not optional.**  This device sits at about +650 counts
on-hook, and on the HSF part a standing 908-count offset made SpanDSP's
ANS/ANSam detector reject a 5000-count tone outright.  One pole at 40 Hz
(-0.08 dB at 300 Hz) runs on every sample entering the modem.

**Answer detection is a 2100 Hz Goertzel, not a post-dial timer**, because a
timer ran V.8 into ringback on the HSF path and its 10 s timeout expired as the
far end answered.  Ringback is 400/440/480 Hz and ANS/ANSam is 2100 Hz.

### What is verified, and what is not

The line was disconnected when this was written, so **no call has been placed
through the engine by this program.**  What is verified:

- **Both rates run the same code path** -- `--rx-replay` on the live 8416
  recording at its native 9600 through both resamplers, and on the 8000
  resampling of it with the receive resampler bypassed: answer detected and the
  engine started in both.
- **It refuses to dial with no pair connected.**  `line_hook()` reads register
  0x1d after seizing and bails on 0x00, so a disconnected pair is reported
  rather than producing a session's worth of uninformative measurements -- which
  is exactly what happened before that check existed.
- **The offline path runs end to end.**  The live 8416 call was recorded at
  9600 Hz and resampled to 8000 (exactly 5/6), and `--rx-replay` on it fires the
  answer detector at **6.69 s** against the **6.60 s** the live call showed,
  starts the engine, and SpanDSP reports **`V.8 answer tone: ANSam/`** -- so the
  tone survives the DC blocker and is recognised, which is the specific thing
  the HSF offset broke.  The recording ends 0.46 s later, so V.8 cannot
  complete; that is the fixture's length, not a failure.
- The DC blocker leaves **-0.14 counts** of the fixture's -288, with the 2100 Hz
  amplitude unchanged (1731 -> 1758).

A recorded tap is the line as it arrived, so the replay runs the answer detector
too -- a replay that skipped to the engine could not reproduce a call that failed
in answer detection.

## Traps, each of which cost time here

1. **A device at configuration 0 answers nothing.**  Set the configuration
   before concluding anything about a stalling control endpoint.  Note the
   ordering hazard: all six interfaces claim successfully *before* the
   configuration is set and interfaces 2-4 stop claiming *after*, because
   setting it is what lets `usbaudiod` attach.
2. **libusb's darwin backend serves device and config descriptors from the
   IOKit cache**, so both keep succeeding after a device has stopped answering.
   Liveness needs a request that reaches the wire; a string descriptor is the
   cheapest.  (Already recorded in `docs/hsf_usb_daa.md`; reconfirmed.)
3. **Denied microphone access yields digital silence**, which is
   indistinguishable from a pristine noise floor.  Check the TCC authorization
   status *and* treat an exactly-all-zero buffer as "not a measurement".
   Granting permission mid-run leaves that run with no data — re-run.
4. **A constant that varies with sample rate is not a fill pattern.**  That is
   what separated "the device is railed" from "our capture is broken".
5. **Initialise min/max from the first sample.**  Starting max at 0 on
   all-negative data reports `max = 0` and invents a zero sample.
6. **`AT\r` was accepted by the command channel and did nothing.**  It is three
   bytes, which is a legal command length, so the write succeeded; `'A'` is
   `0x41`, not an opcode.  Acceptance of a control write is not evidence the
   payload was understood — and a softmodem has no AT interpreter on the
   device in the first place.
7. **Do not assume an unnamed callee is a helper.**  `0x14eca`, which appeared
   to compute command bytes, is `memcpy`.
8. **A queued response does not survive closing the handle.**  Sending the read
   from one process and the `GET` from the next reports "nothing queued" for
   every register, which reads exactly like a device that does not answer.  The
   register file looked dead for one round because of this.
9. **A 2-byte command is stalled.**  Only 3- and 9-byte bodies are accepted, so
   the read is `80 <idx> 00`; the earlier note that observed lengths were
   "2, 3, 9" counted `GET_ENCAPSULATED_RESPONSE`'s 2-byte IN as a command.
10. **A 16-tap-per-phase polyphase prototype is too short here, and it fails
   quietly.**  `--selftest` read gain **0.82** at 3600 Hz on the 5/6 receive path
   and **16.2 dB** SNDR on the 6/5 transmit path, where the `8000 -> 9600` image
   at 4400 Hz sits only 400 Hz into the stopband.  48 taps fixes both.  Either
   would have presented inside the modem as a level or a noise problem, nowhere
   near its cause -- which is the argument for the resampler having a
   measurement of its own rather than being assumed correct.
11. **A replay that duplicates a shortened version of the live path can pass for
   the WRONG REASON.**  The first `--rx-replay` did its own DC blocking and
   answer detection and fed the file straight to `me_rx_audio()`, so at a device
   rate of 9600 it handed 9600 Hz samples to an 8000 Hz entry point -- and it
   "detected the answer tone at 6.66 s against the live call's 6.60 s" only
   because the detector was mis-tuned by the same 6/5, looking for 1750 Hz.  Two
   errors cancelling.  Replay now goes through `engine_feed()`, the one path.
12. **Read register 0x1d before believing any line measurement.**  A whole
   session was spent on a pair that was not connected, concluding that bit 3
   was not the hook and that the hook was unidentified -- both wrong, from
   measurements that were internally consistent.  0x1d reads 0x00 when there is
   no pair, which is the device saying so.
13. **zsh does not word-split an unquoted `$var`** (it does split `$(...)`).
   `--read $R` with 59 indices in `R` therefore read ONE register whose index
   parsed out of `strtoul("01 02 03 ...")`, printed one line and exited 0 --
   which read as "the device stops answering after the first read" and briefly
   became a finding about the hardware.  Use `${=R}` or inline the command
   substitution.
14. **An IOCTL number does not name a request.**  `utlamot.sys` carries the
   IOCTL through a work queue into `FUN_0001316a`, which is where `bRequest`,
   `wValue`, `wIndex` and the length are chosen, and one IOCTL can have two
   sub-forms keyed on the body's first byte (`0x2200c4` does).  Reading the
   caller's payload and assuming it reaches the wire unchanged is what left the
   first table's fields unnamed.

## Open

- **A datapump on it.**  Seizing, dialling and reaching a far-end modem all
  work, so what is left is pointing the engine at this device the way
  `hsf_v90_coupler` does at the Conexant part -- receive sampling phase and the
  DC offset being the two things that cost that path a session
  (`docs/hsf_analogue_v90_coupler.md`).
- **Transmit level calibration.**  0.15 per tone was chosen to sit near the
  arriving dial tone and is not referred to dBm0.
- **Ring detection**, which needs an inbound call rather than a seizure.
- **Pulse dialling**, bit 0 toggled to the country profile's timing, untried.
- **What the other non-zero registers mean.**  0x01, 0x02, 0x08, 0x09, 0x0b,
  0x0c, 0x0d, 0x0e, 0x11, 0x13, 0x16-0x19 and 0x1b all read non-zero and none
  of them moves with the hook.
- What the remaining opcodes do: `0x10|n`, the 9-byte `0x02` form, `0x90`,
  `0xd0`.
- How the 32, 500 and 229-byte country blobs reach the device.  Runs of
  register writes is the obvious guess, and the register file is now readable,
  so a diff of it across a country change would show it.
- Whether the DFU interface expects a firmware push.
- The `wIndex = 0` requests (`bRequest` 0x11/0x13/0x14), untested live.
- Whether `0x220070` (tested by the dispatcher but reaching no request block
  above) does something else.

## Two modems at once: a conference bridge as an analogue bearer (2026-10-01)

Two of these parts on two FXS ports, both dialled into conference bridge 2280,
give a real analogue path between two hosts-under-our-control with no modem
protocol in it at all -- so anything measured is the bearer.
`tools/apple_modem_pair_test.sh <out-dir>` runs it and
`tools/apple_modem_tone_report.py <out-dir>` reports.

**Two device selections, and they are not the same handle.** The hook is a USB
control request and the codec is a CoreAudio device, so the probe now takes
`APPLE_MODEM_ADDR=bus:addr` (it prints every candidate) and the audio tool
`APPLE_MODEM_AUDIO_UID=<substring of the UID>`; `--descriptors` and `list`
enumerate them. **The two lists cannot be zipped together** -- the UIDs here
are `...:000000:3,4` and `...:1143000:3,4` and only the second carries the USB
location, so the mapping is established empirically: go off-hook on one USB
address and capture on each UID, and the modem still on-hook returns a stream
railed at -32768 (documented above) while the other returns dial tone. Here
USB `1:6` is UID `1143000` and `1:7` is `000000`.

**`find_device()` used to stop at the first match and `list` therefore showed
one modem while `system_profiler` showed two**; it now lists all and still
takes the first.

**A dial that is not accepted looks exactly like a dial that is.** The tool
reports "transmitted 10560 of 10560 scripted samples" either way. The only
evidence a leg joined is the **dial tone being gone**, so `join()` captures a
second afterwards and measures the 400 Hz line. It matters: **one of these two
FXS ports rejects DTMF at the 0.15 default and needs 0.35** (the other takes
0.15 every time), which cost this session four runs that reported success and
sat on dial tone. `join()` retries 0.15 / 0.25 / 0.35.

### What the path measures

Levels are dBFS at the codec; transmit amplitude 0.15 per tone is -16.5 dBFS.

- **Frequency response is flat**: A->B -21.4 / -21.0 / -20.9 / -20.9 dBFS at
  300 / 1000 / 2000 / 3000 Hz, B->A -23.3 / -23.0 / -22.8 / -22.8. So 0.5 dB
  of ripple over 300-3000 Hz, with a fixed 1.9 dB asymmetry between the
  directions. End-to-end loss is ~4.6 dB one way.
- **Frequency is exact**: 300.0, 1000.2, 1999.8, 3000.0 Hz recovered.
- **No AGC and no compression**: transmit -34, -26, -16.5 and -9.1 dBFS come
  back at a constant 4.6-5.2 dB loss, i.e. the path is linear over a 25 dB
  range. A 50 ms window shows the tone reaching full level in one window with
  no ramp, so nothing is adapting.
- **Distortion 0.12-0.21% THD** at three of the four levels. One capture read
  9.9% -- and it is not a level effect, because the *louder* row either side of
  it is clean; its spurs are a comb at exact multiples of 400 Hz, so something
  400 Hz-related was on the bridge at the time. Retest before quoting a
  distortion figure from a single capture.
- **The bridge does not mix a talker's own audio back**, and it does not send
  comfort noise: with both legs silent the receive floor is -71 / -73 dBFS.
- **Under double talk the path stays linear**: with A on 1000 Hz and B on
  1400 Hz simultaneously, the intermodulation products (400, 600, 2400 Hz) are
  all at -80 dBFS or below, 58 dB under the wanted tone. An earlier run showed
  400 Hz and 2400 Hz at 6-7% of the total power and **that was the same 400 Hz
  contamination, not the bridge** -- a repeat with the same two tones is clean.
- **The two hybrids differ a lot and repeatably**: each modem hears its own
  tone during double talk at 44 dB below the far tone on A and **18 dB on B**.
  B's 2-wire hybrid returns far more, which is what a modem's echo canceller
  would have to deal with on that port.

Delay is NOT measured: the transmit and capture processes have no common
clock and the bridge does not return a talker's own audio, so there is no
reference to time against.

**The receive path clips on dial tone.** Off-hook with no call, the 400 Hz
dial tone arrives at -2.5 dBFS with 13% of samples at full scale. Signals at
the levels above are nowhere near it, but nothing in this tree sets a receive
gain and the headroom against the exchange's own tones is about 2 dB.

## PBX test extensions, and what they settle (2026-10-01)

`9099` echo, `9333` DTMF read-back, `9222` voice read-back, `9666`/`9667`/`9668`
test tones. `tools/apple_modem_line_check.sh <bus:addr> <uid>` is the one that
came out of this and is worth running before anything else.

**Delay: 269 ms of network, and a raw reading would have been 122 ms wrong.**
The audio tool transmits and captures through one HAL unit on one device, so a
capture is sample-aligned with the transmit script and a delay IS measurable
against 9099. The capture of a 300 ms burst contains **two** returns:

    tone leaves at              200.0 ms  (the script's own 200 ms lead)
    near-end hybrid returns at  322.6 ms  at -55.0 dBFS
    the echo test returns at    591.8 ms  at -20.2 dBFS

The first is our own 2-wire hybrid and cannot have travelled anywhere, so the
122.6 ms in front of it is **this host's own loop latency** -- CoreAudio's
output buffering, the USB isochronous path, and the capture side again. The
network round trip is therefore 591.8 - 322.6 = **269 ms**, not the 392 ms the
burst's arrival says. Repeatable to +/-2 ms over three bursts. Read both
returns or the number is the host's buffering plus the network.

**Different PBX applications do not pass level alike.** Echo() returns the
burst 3.7 dB down for the whole round trip; ConfBridge costs 4.6 dB in ONE
direction. So a level measured through one application says nothing about the
other.

**9666 is a stepped tone reference and 9667 a slow sweep**, both at constant
source level, so they measure the receive path on their own -- no reliance on
our own transmitter. 9666 steps 400, 500, 700, 1000, 1500, 2000, 2500, 3000,
3400, 3800 Hz, 3 s each (a 300 Hz step precedes them), then the call ends.
Modem A's receive path against it, relative to 1000 Hz:

    400   +0.01     1500  +0.02     3000  -0.03
    500   +0.06     2000  -0.07     3400  -0.39
    700   +0.03     2500  -0.12     3800  -7.96

-- flat to **0.12 dB from 400 to 3000 Hz**, with 3800 Hz outside the band as
expected. 9667 rises about 20 Hz/s (70 -> 150 Hz over four seconds), so
reaching the voiceband takes minutes; its apparent level rise is the line's
own low-frequency roll-off, not the source. 9668 never connected here.

**Echo return loss in a call, single talk, held 30 s**: A 39.7 dB and B
30.8 dB, both steady to 0.5 dB with **no convergence trend**, so nothing
visibly adapts on this signal over that span. B's in-call noise floor is
-41 to -44 dBFS against A's -59, i.e. **15-18 dB worse**.

### B's port collapses about a second after it goes off-hook

This is what made B's dialling intermittent all session, and it is invisible to
every other instrument here. Transmitting a 1000 Hz tone at -9.1 dBFS into the
dial tone and measuring what comes straight back, against the delay between
the off-hook and the tone:

    tone at +0s after seizure   ERL 26.6 dB      (healthy)
    +1s, +2s, +5s, +10s         ERL 2.0-2.1 dB   (collapsed)

and on repeated seizures B reads 1.7-2.1 dB six times running where A reads
**24.0, 24.0, 24.1 dB**. So B returns essentially all of its own transmit a
second after seizure. The exchange then does not detect its DTMF -- our dial
script sends the first digit well over a second after the hook command -- which
is exactly the symptom: B joined 2 of 12 attempted calls and A missed none.

**It is not an open pair**: the dial tone is still there at the same level in
the collapsed state, and `--read 1d` still reports "pair present". Register 5
and the line sense are **identical** before and after the collapse on both
modems, so nothing in the DAA's register file sees it -- the hybrid balance
goes while the loop stays up. Suspect B's cable, jack or DAA termination.

This also reframes the double-talk figures in the section above (own-tone echo
44 dB down on A and 18 dB on B): that is the same fault, not a fixed property
of the two hybrids.

## Which line is which, and a direct call between them (2026-10-01)

The two modems are extensions **6004** and **6005**, and `9333` answers with
the calling line's own number in DTMF -- the one read-back that needs no
speech understanding, so it identifies a line objectively.
`tools/apple_modem_dtmf_decode.py <capture.s16>` decodes it. Modem **A**
(USB `1:6`, UID `1143000`) reads back **6004**; modem **B** (USB `1:7`, UID
`000000`) is **6005**, established by A dialling 6005 and B ringing.

**Capture the read-back with no gap after the dial.** 9333 answers, reads the
number and hangs up inside about ten seconds, so a capture started after the
usual settle lands on dial tone and reports nothing -- the first attempt here
did exactly that. `dial` captures 7.09 s of its own; start the next capture
immediately and the digits straddle the two files ("6" at the end of one,
"004" at the start of the next).

Its digits arrive at **-12 dBFS per tone, 290-300 ms, twist +0.1 dB**, which
is the reference for what this exchange considers a well-formed digit. Ours
go out at -16.5 dBFS for 100 ms by default.

### A direct call, and what differs from the bridge

A dials 6005, B rings, B answers by going off-hook. Nothing else is needed --
**answering requires no DTMF**, which is why B can take a call on a line too
poor to dial one.

- **The ring is NOT in the audio path.** With B on-hook and `--monitor on`,
  its capture is -62 dBFS noise for the whole ring; there is nothing to detect.
- **It IS on the interrupt endpoint, as CDC RING_DETECT.** Interface 0's
  endpoint 0x81 delivers `a1 09 00 00 00 00 00 00` throughout the ring and
  **nothing at all with no call** (a 16 s control run returns zero
  notifications), with gaps at the cadence's silent periods. That is the
  missing piece for an answer role: watch endpoint 0x81 for `0xa1 0x09`, then
  set register 5 bit 0.
- **Level is the same as the bridge**: A->B -21.8 dBFS direct against -21.0
  through 2280, B->A -23.7 against -23.0. So ConfBridge costs about 0.7 dB,
  not the 0.9 dB the Echo() comparison suggested, and **the 1.9 dB direction
  asymmetry is in the two lines, not in the bridge** -- it is the same on both
  paths.
- **Flat**: A->B reads -22.0 / -21.8 / -21.7 / -21.6 / -22.3 dBFS at 300 /
  1000 / 2000 / 3000 / 3400 Hz, i.e. 0.7 dB across the band including 3400.
- **The far end's echo is cancelled and ours is not.** A 300 ms burst from A
  returns ONCE, at 322.6 ms -- our own hybrid, at the same host latency as
  every other measurement here. B's hybrid was returning nearly all of its
  receive at the time (2-10 dB return loss), so its reflection of A's tone
  should have reached A at about -30 dBFS, well above A's own -39.3 dBFS
  echo, and there is **no step at all** where it is due (~592 ms). So at least
  12-20 dB of far-end echo suppression is active in the gateway. It does not
  touch our own near-end echo, which is the expected asymmetry: a gateway
  canceller removes our line's reflection before sending it onward, and never
  sees our own transmit reflected into our own receive.

**Consequence for the echo test:** 9099's 269 ms round trip is measurable only
because that echo is an application deliberately returning audio. The echo a
canceller would remove is removed, so a direct call gives no delay reading at
all by this method.

## The engine on both modems: a modem call over two real analogue lines (2026-10-01)

`apple_usb_modem_coupler` now selects its device and can answer, so both
modems can run the engine at once and call each other:

    ME_MODE=v34 APPLE_MODEM_ADDR=1:7 APPLE_MODEM_AUDIO_UID=000000 \
        ./apple_usb_modem_coupler --answer --ring-wait 45 --hold 60 &
    sleep 3
    ME_MODE=v34 APPLE_MODEM_TX_AMP=0.35 APPLE_MODEM_ADDR=1:6 \
        APPLE_MODEM_AUDIO_UID=1143000 ./apple_usb_modem_coupler --dial 6005 --hold 60

**`--answer` waits for CDC RING_DETECT and goes off-hook**, then calls
`me_answer()` and starts the engine on the first block -- there is no answer
tone to wait for, because this is the end that sends one. Answering needs no
DTMF, which is what lets the bad line take a call it cannot place.

**`ME_MODE=v34`, not the default.** V.90 needs a digital modem on a DS0 and
both ends here are analogue, so there is no V.90 call to be had between them;
the engine says so itself (`ME_V90_ROLE=analogue ignored in v34 mode`).

**Two engines on one host were writing one pair of PCM dumps.**
`/tmp/v34_tx.raw` and `/tmp/v34_rx.raw` are opened once per process from a
fixed path, so a two-instance run left two taps that are of neither end.
`ME_DUMP_DIR` now scopes them; set it per process or every tap from this rig
is worthless.

**The coupler's default DTMF is too quiet for this exchange.** At the built-in
0.15 the digits were not detected, the far line never rang, and the log shows
only "engine never started" -- which reads like a coupler fault and is the
exchange rejecting the dial. `APPLE_MODEM_TX_AMP=0.35` is what works here.

### How far it gets

Five attempts, same binaries and settings:

- **V.8 completes and selects V.34 on both ends in every run where the call
  connected** -- `V.8 call negotiation successful`, roles right (caller and
  answerer), both entering TRAINING.
- **Best case both ends reach the Phase 4 MP exchange** (`rx=PHASE4_MP tx=MP`
  on both), then each reports the other's retrain tone, restarts Phase 2 and
  falls back to V.22bis.
- The others stalled earlier -- one in Phase 3, two in Phase 2 around
  INFO1/INFOMARKSa, one never rang.

So the whole startup runs over a real analogue path: V.8, Phase 2's tone
choreography and INFO0/INFO1 exchange, Phase 3's S, S-bar, TRN and J, and
Phase 4's S, TRN and MP. **No data mode yet.**

**Read all of that against a bearer known to be faulty**: 6005's echo return
loss collapses to about 2 dB a second after it goes off-hook (previous
section), so its hybrid returns nearly everything it receives. That is a
plausible reason for an intermittent full-duplex startup and it is NOT
established here -- an attempt to measure the echo in the training taps read
a correlation of 0.92 at every lag tried, which is the low-entropy-reference
trap this document records elsewhere, so it says nothing. Fix the line, then
re-run before concluding anything about the stack.

## V.91 between the two modems: it negotiates, and it cannot train (2026-10-01)

Asked for directly, ignoring that both ends are analogue. Both sides with
`ME_V8_ADVERTISE_V91=1`:

**`ME_V8_ADVERTISE_V91` is ignored on the analogue-role branch.**
`prepare_v8_parms()` sets `pcm_modem_availability` in three branches and only
the two non-analogue ones consult that variable, so the coupler's own
`setenv("ME_V90_ROLE", "analogue", 0)` silently suppresses the offer.
`ME_V90_ROLE=digital` is needed on both ends to reach it.

With that, **V.8 negotiates V.91 on both ends** -- `pcm=0x6`, "V.91 and
V.90/V.92 digital available", `V8 selected V91` -- and both enter
`TRAINING: mod=V91` with a 2260-symbol startup at ceiling drn 28. That is the
first time the live V.91 path has run against any real bearer.

**It then fails at the first step, in both directions.** A new receive stage
trace (`V91 rx stage: ...`, added here because the V.91 receiver had none at
all and reported only "SCR fill exhausted waiting for peer DIL") shows **no
transition whatever: the receiver never leaves `HUNT_EZ`.** Not one codeword
of the peer's startup is recognised, so nothing about DIL, CP or rate
selection is being reached or tested.

That is structural, not a defect, and two measurements say so.

**There is no common clock, and V.91 has no timing recovery.** On a DS0 both
ends take the network's 8 kHz; here each modem's codec free-runs. Measured on
a 30 s 1000 Hz tone across a direct 6004 -> 6005 call, fitting the phase over
2 s blocks:

    control, A hearing its OWN tone through its own hybrid   +1.2 ppm, 15 deg residual
    B hearing A's tone, two independent codecs              +30.1 ppm, 102 deg residual

The control is what makes the second number mean anything -- one clock reads
essentially zero, as it must. And the 102 degree residual says the two are not
even related by a constant offset; a single frequency does not describe it.
At 30 ppm the codeword alignment slips about every four seconds, and V.91
carries nothing to track it with.

**And the path is not codeword-transparent anyway**: ~4.6 dB of loss each way
(measured above) and two D/A-A/D conversions between the two codecs, so a
transmitted codeword does not arrive as itself even when the sample lines up.

**What it would take.** V.91 needs a bit-transparent 64 kbit/s path with a
shared clock -- what the SIP/G.711 bearer in `sip_v90_modem` already is. These
two modems can exercise V.8's V.91 negotiation and the transmit side's startup
construction, and nothing past that; the receiver has no bearer to lock to.

## V.92 between the two Apple modems (2026-10-01)

Run with the roles the Recommendation requires rather than the symmetric
pairing V.91 used, because V.90/V.92 9.2 puts the **analogue** modem on the
**calling** side: 6004 (the good hybrid, 16.4 dB echo return loss) dials as
the analogue modem, and 6005 answers as the digital modem. 6005 is the right
end to answer on even though its hybrid collapses to ~2 dB a second after
seizure, because answering needs no DTMF.

    APPLE_MODEM_ADDR=1:7 APPLE_MODEM_AUDIO_UID=000000 \
    ME_MODE=v92 ME_V90_ROLE=digital ME_DUMP_DIR=<d>/ans \
      ./apple_usb_modem_coupler --answer --ring-wait 60 --hold 70 &
    sleep 3
    APPLE_MODEM_ADDR=1:6 APPLE_MODEM_AUDIO_UID=1143000 APPLE_MODEM_TX_AMP=0.35 \
    ME_MODE=v92 ME_V90_ROLE=analogue ME_DUMP_DIR=<d>/call \
      ./apple_usb_modem_coupler --dial 6005 --hold 70

`ME_V90_ROLE=digital` is needed on the answering end because the coupler
`setenv`s `analogue` with overwrite 0 -- it *is* the analogue modem, wired to a
2-wire line -- and an explicit value still wins. `ME_DUMP_DIR` must be set per
process or the two engines write one pair of PCM dumps.

**V.8 and the roles are right, and the V.92 INFO0 exchange completes
mutually.** The caller takes the analogue role (`U_INFO=78`) and reads the
peer as `PCM=V.90/V.92 digital available`; the answerer reads us as
`PCM=V.90/V.92 analogue available` and logs

    V.92 INFO0a flags: raw26_27=0x1, capability(bit26)=1, short-phase2(bit27)=0
    V.92 INFO0 confirmed mutually (INFO0d bit27=1, INFO0a bit26=1); selecting long Phase 2/3

First time the two roles in this tree have confirmed V.92 to each other, and
the first time over a real analogue bearer rather than G.711.

**`ME_V92_PCM_UPSTREAM` is what separates "V.92 negotiated" from "V.92
selected", and without it we demote ourselves.** With the default off, the
mutual INFO0 above is immediately followed by

    V.92 INFO0 was mutual but the peer answered a V.90 INFO1a
    (upstream_code=4, not 6/8000); demoting to V.90

and the peer answering a V.90-form INFO1a is **us**: `prepare_info1a()` in
`v34tx.c` selects Table 18's PCM upstream only when
`v92_pcm_upstream_capable` is set, which `v92_pcm_upstream_advertised()` gates
on that environment variable. The engine's note recording this behaviour of
d-modem applies to our own analogue role as well; it is a documented opt-in,
not a defect. With `ME_V92_PCM_UPSTREAM=1` on both ends the contract is
complete in both directions:

    (digital)  V.92 strict RX event: valid INFO1a U_INFO=78 MD=0 upstream_code=6 downstream_code=6
    (digital)  V.92 native Phase 4 RX enabled: 4-point TRN2u, L_U=8000
    (analogue) V.92 analogue Table 18 selected: linear PCM, U_INFO=78 RTD=0

`v92_analogue_phase3.c` then runs its first five stages live for the first
time -- `V92A_SILENCE`, `V92A_RU`, `V92A_RU_BAR`, `V92A_TRN1U`, `V92A_JA` --
and fails at the next one, `Sd-bar timeout (9.5.2.2.1)`, taking the 9.5.2
retrain and then sitting in Phase 2 at `rx=TONE_B` for the rest of the call.

**It cannot get past there, and the reason is the absence of a DIGITAL side --
not the analogue line.**  Say that carefully, because the first version of this
section got it wrong.  V.90/V.92's design case *is* an analogue local loop: the
digital modem sits on a DS0 and places codewords into timeslots on the
network's 8 kHz clock, exactly one codec D/As them toward the subscriber, and
the analogue modem recovers both the codewords and that clock from the line --
which is what 9.3's Sd and TRN1d are *for*.  Codewords surviving one D/A onto a
2-wire pair is the premise of the Recommendation, not an obstacle to it.

What this rig has instead is an analogue modem at **both** ends.  The end
running `ME_V90_ROLE=digital` has no DS0: its codewords are D/A'd by its own
free-running codec, cross the loop (and the PBX), and are A/D'd again at the
far modem.  So there is an extra conversion pair that a real V.92 call does not
have, and -- the harder half -- no network clock anywhere in the path for the
analogue end's timing recovery to lock to.  The V.91 section above measures
that directly: +30.1 ppm between the two codecs with a 102 degree residual,
against +1.2 ppm for the one-clock control.  **So what is refuted here is
modem-to-modem V.92, and `ME_V90_ROLE=digital` on a modem is a fiction the
engine will accept and the protocol will not.**

The run still reaches the codeword wall rather than stopping earlier, and the
V.90 run shows it inside one log: the answerer's own log says it transmitted
the whole downstream --

    V90 Phase 3: Sd complete (64 reps), starting S-bar-d
    V90 Phase 3: S-bar-d complete, starting TRN1d

-- while the caller's own log scores **every one of its 24 Sd acquisition
windows at exactly 0.000**, `no Sd in this window (held-out score 0.000)`, and
never reports a fit.  Zero, not a marginal score.  9.3.2's deadline then passes
in Ja, the analogue side takes its 9.5.2.1 retrain, and 60 s later training
times out and falls back to V.22bis.  That bounds the rig; it says nothing
about whether the receiver could lock a downstream that came from a real
digital modem.

**The rig that DOES test the analogue role is the SIP one, and it already
exists**: the coupler as the analogue modem on its loop, dialling
`sip_v90_modem` which answers on SIP as the digital modem over G.711.  There
the VG224's D/A toward the loop is the central-office codec and there is
exactly one of them, which is the real topology --
`docs/hsf_analogue_v90_coupler.md` is the same arrangement with the HSF part.
The open item there is our own codeword receiver, which is validated only
against `reference-data/artifacts/eicon-digital-downstream/`, a byte-exact G.711 capture, and
so does not normalise the gain, sampling phase and band-limiting a line
imposes before it slices.  That is a receiver gap and the thing to fix; it is
not a property of analogue lines.

**Not run from this Mac.**  Its default route is a VPN tunnel (`utun10`), so
`sip_v90_modem` here cannot register (403) and the bearer would cross the
tunnel even if it could -- and the 2026-09-30 finding already established that
this host's network path breaks V.8 outright, with the fix being to run the SIP
end on tower.  The analogue end has to stay on the Mac because the modem is
plugged into it, so the digital end belongs on tower, where Asterisk is, and
the audio path between them is then PBX-internal.

Artifacts: `reference-data/artifacts/apple-v92-r2` (V.92 negotiated, demoted to V.90) and
`reference-data/artifacts/apple-v92-r3` (V.92 selected both directions), each with its
`run.sh`.

**Two method notes from this session, both of which read as findings first.**
`apple_usb_modem_audio` does **not** seize the line -- `apple_usb_modem_probe
--hook on` is a separate step -- so a `dial` capture taken without it shows a
flat DC offset and no dial tone, which reads exactly like a dead line; off-hook
the same dial shows 400 Hz dial tone, muting on the first digit, and 400/450 Hz
ringback. And `--notify` is **not** a control for ring detection unless you
know nothing is ringing: an unanswered call leaves the PBX ringing the far
extension for a long time, so a "no call in progress" run taken fifteen seconds
after a dial test reported RING_DETECT and briefly looked like the endpoint
reporting rings spuriously. A minimal libusb A/B then showed the device quiet,
which is what withdrew it.

### V.92 against our own digital side over SIP -- the real topology (2026-10-01)

The section above could only refute modem-to-modem V.92.  This is the
arrangement the Recommendation is written for: the Apple modem is the ANALOGUE
modem on its 2-wire loop and dials `sip_v90_modem`, which answers on SIP as the
DIGITAL modem over G.711.  Exactly one codec sits in the path -- the VG224's
D/A toward the loop -- which is where a central-office codec sits in a real
V.92 call.  `reference-data/artifacts/apple-v92-sip-r4` (V.92) and `apple-v90-sip-r6` (V.90),
each with its `run.sh`.

**The digital end must be on tower, not this Mac.**  Register as **6000/6000**:
6001 now answers 403 Forbidden from both hosts and over IPv4 as well as the
IPv6 the name resolves to first, although the dialplan still routes it (dialling
6001 from an FXS port returns ringback).  This Mac's default route is a VPN
tunnel (`utun10`), so it cannot register at all and would carry the bearer over
the tunnel if it could; the 2026-09-30 finding already has this host's network
path breaking V.8 outright.  The modem has to stay on the Mac, so the digital
end goes in tower's `v90modem-sip` container and the audio path between them is
PBX-internal.  **Leave ~45 s between calls**: dial sooner and the exchange does
not ring, the coupler reports `engine never started`, and the previous call's
tap is all silence -- which reads exactly like a transmit failure.

**What happens: V.8, Phase 2 and the whole V.92 contract complete, and then
NEITHER side transmits Phase 3.**  V.92 is selected in both directions
(`valid INFO1a U_INFO=78 MD=0 upstream_code=6 downstream_code=6`, the digital
side arming `4-point TRN2u, L_U=8000` and the analogue side selecting
`Table 18 ... linear PCM`), the analogue startup runs SILENCE/Ru/R-bar-u/TRN1u/Ja,
and it then fails `Sd-bar timeout (9.5.2.2.1)` at +10.6 s, retrains per 9.5.2
and sits in Phase 2 at `rx=TONE_B` until the 60 s training timeout drops it to
V.22bis.  Reproducible to 50 ms across runs (+10560, +10613 ms).

**The analogue receiver is NOT at fault, and its own transmit tap is what says
so -- the fifth time in this project.**  `raw_v90_tx=453920` moves on the
digital side, which reads as a transmitted downstream and is not one:
µ-law-expanded, its own `live-tx.g711` is **exactly zero from t=11 s to t=67 s**,
56 seconds of digital silence (0xFF) spanning the whole of Phase 3.  Under
`ME_MODE=v90` the same path transmits continuously instead -- RMS ~1280 for the
rest of the call -- but at **1200 Hz with 0.0% at 1333 Hz**, i.e. the Phase 2 CC
carrier, not Sd.  So **in both modes the digital side never puts Sd on the
wire**, and the analogue side's `no Sd in this window (held-out score 0.000)` on
every window is correct behaviour rather than a detector failure.

**The cause is one step further up: Ja is never accepted, so 9.3.1.3's Sd never
starts.**  9.3.1.3 has the digital modem transmit Sd only after receiving Ja.
In the V.90 run the digital side logs `V.90 Ja capture: first bits at t=11.180s
(52 bits)` and then no `Ja detected, starting Sd` and no `Sd complete`; in the
V.92 run it logs **no Ja activity at all**.  (Between the two Apple modems the
500 ms energy-gap fallback did fire and that side did send `Sd complete (64
reps)`, so the Ja path works when the gap appears -- over this bearer it does
not.)  **That is the blocker, and it is upstream of the codeword question
entirely**: nothing yet tests whether our codeword receiver can lock a
downstream off a line, because no downstream has been transmitted.

**Device-level taps now exist, and the decisive reading needed them.**
`--rx-replay` could consume a recording and nothing could make one.  The
engine's own `v34_rx.raw` cannot answer this: it is written only while the V.34
receiver is fed, so on a V.90/V.92 call it has a **hole exactly over Phase 3**
-- its `RX dump mark` stage marks jump from Phase 2 straight to the
post-retrain Phase 2 -- and it is post-filter besides.  Reading it as a
continuous record of the line produced a confident, wrong story about a 54 s
pure 2400 Hz tone; **those marks are in the file's own log for this reason, so
map position to stage with them before interpreting it.**
`APPLE_MODEM_RX_TAP` and `APPLE_MODEM_TX_TAP` record the raw device streams at
`--rate`, the receive one being exactly what `--rx-replay` expects.  With them
the analogue side's line reads: ANSam at 2100 Hz RMS ~5050, the digital side's
Phase 2 CC at **1200 Hz RMS ~6000** arriving loud, then from t=19 s **RMS ~450
with no spectral content** while our own transmit continues at ~5950.

**One trap in the Sd measurement itself: DTMF's 1336 Hz column tone sits on top
of Sd's 1333 Hz line** (8000/6).  Both taps peak at 1336 Hz during dialling and
the receive tap reads 73.5% there, which is our own digits, not Sd.  Only the
post-dial region counts.

### The two Phase 3 blockers, separated -- and the V.90 one is half gone (2026-10-01)

The section above left "Ja is never accepted" as one blocker common to both
modes.  It is two different faults that happen to present the same way, and
the V.90 half is no longer true as written.

**V.92: the Ja machinery is called and the strict Phase-3 receiver never gets
that far.**  "No Ja activity at all" was a reading of the V.34 Ja search, which
on a V.92 call is the wrong instrument by design: V.92's Phase 3 upstream is
linear PCM (`v92_analogue_phase3.c` transmits Ru/uR/TRN1u/Ja as 16 kHz linear,
not V.34 symbols), so `phase3_ja_capture_hyp[]` is legitimately empty and the
`Ja search input ... longest_hyp_len=0` line in the server log is correct.  The
receiver that matters is `v92_p3_rx.c`, armed in the same breath
(`V.92 Phase 3 raw receiver armed at G.711 sample 81440`) -- **and it then logs
nothing whatever for the rest of the call, because every failure path inside it
is a rehunt rather than a failure.**  A call that acquires Ru and rejects TRN1u
is, in the server log, indistinguishable from one on which no upstream arrived.

New `tools/v92_p3_probe.c` (`make v92_p3_probe`) replays a recorded
`live-rx.g711` through that same receiver and prints every state change.  On
`reference-data/artifacts/apple-v92-sip-r4/server/live-rx.g711` armed at the sample the engine
named, it reproduces the live outcome exactly:

```
sample   85008 (  10.626s) state=ru1
sample   85272 (  10.659s) state=ur1
sample   85323 (  10.665s) state=trn1u
sample   85578 (  10.697s) state=ru1_hunt
final state=ru1_hunt rejects=2 last=trn1u_ones_low m0=48 m1=-1 ja_ok=0
```

So **Ru and uR are acquired off a real analogue loop** -- the bearer carries
them, and the received Ru is textbook: at t=10.62 the digital side's codewords
expand to `-1087 -623 +439 +1087 +623 -439` repeating, a clean period-6 line at
8000/6 = 1333 Hz with a stable phase.  **TRN1u is where it dies, and the reason
is arithmetic rather than a threshold.**  TRN1u (8.5.7) is GPA-scrambled ones
selecting ±L_U at the *full* 8 kHz symbol rate; the receiver recovers it by
slicing the sign of each received codeword, which gives **48% ones** against a
75% gate.  Swept offline over the received tap -- fractional sampling phase in
tenths of a sample crossed with ±3000 ppm of symbol-rate offset, 1500 symbols
per point -- **the best any sign slicer can reach on this signal is 67.4%**
(at +250 ppm, phase 0.8), still under the gate.  It is not DC either: the mean
over the TRN1u era is 0.8 counts.  **The signal is also not band-limited away**
-- measured over a 256-sample window at t=10.70, the energy is 19% below 1 kHz,
31% 1-2 kHz, 37% 2-3 kHz, 6% 3-3.4 kHz and 7% above 3.4 kHz, i.e. essentially
flat to Nyquist -- so what defeats the slicer is ISI, not a missing band.

That is the mirror image of the V.90 analogue-role gap already recorded here:
**the digital side's V.92 Phase 3 receiver has no equalizer and no timing
recovery, and it needs both.**  Ru survives only because a 1333 Hz line's sign
pattern is robust to both; nothing at the full symbol rate is.  Note what this
does NOT say: it is not a statement about the bearer, which carries Ru cleanly
and TRN1u broadband, and it is not a level problem, which is why no gain change
is worth trying.

**V.90: Sd IS now transmitted, and the previous section's "silent in both
directions" is superseded.**  `reference-data/artifacts/apple-v90-sip-r7` repeats r6 with
`V90_JA_BIT_DUMP` set and the digital side's own receive tap collected.  Same
binary, same peer, opposite outcome: the 9.3.1.3 500 ms heuristic fallback
fired at 11.1 s, `[V90] Phase 3: analogue Ja detected, starting Sd` and
`Sd complete (64 reps)` both appear, and **the digital side's own transmit tap
carries it** -- from sample 89120 (11.140 s) the µ-law stream expands to
`+1919 0 +1919 -1919 0 -1919` repeating, 8.4.4's four slots at level W with
signs ++-- and two zero slots, for exactly 384 symbols, then S-bar-d, then
TRN1d at RMS 943 for the 2.5 s the current default asks for.  Measure Sd with
the 1333 Hz bin and the *first non-zero sample*, not with a fixed time: the
bin reads 0.333 over the first 384 symbols and 0.005 over TRN1d, which is how
the two are told apart.

**What decided r6 from r7 is where the 9.3.1.3 allowance is measured from, and
the default anchor is not an instant in the protocol.**  The clock starts at
the first *suppressed* heuristic attempt, and the three heuristic sources only
consult that gate once they already have a Ja candidate -- so on a call where
none of them produces one, the clock never starts and the bound never applies.
In r6 nothing consulted it until the far end's own retrain silence at 13.34 s,
200 ms after the analogue side had already given up.  New
`ME_V90_JA_FALLBACK_ANCHOR=ja` measures it from the first captured Ja bit
instead, which is what the clause says ("after RECEIVING Ja ... may wait for up
to 500 ms").  **It is deliberately not the default**: over the eleven RasFinder
calls in `artifacts/` whose descriptor parsed, the gap from the first Ja bits
to the parse is 2.12-2.83 s and never under 2.1 s, so a 500 ms release there
would start Sd -- and so stop the peer's Ja per 9.3.2.4 -- before the descriptor
arrived, on every call that currently works.  The right anchor is a property of
how long the peer holds Ja, not of the clause.

**The remaining V.90 blocker is the analogue side not seeing Sd**, which is the
gap this file and `docs/hsf_analogue_v90_coupler.md` already name: the analogue
Phase 3 receiver slices codewords it expects to arrive as themselves and
normalises neither gain, sampling phase nor band-limiting.  It reports
`no Sd in this window (held-out score 0.000)` on every window of r7 while the
pattern above is provably on the wire from the other end.

**Also checked and NOT the problem, so do not re-derive:** the Ja gate's
suppression latches (`g_v90_ja_first_suppressed_ms`,
`g_v90_ja_suppress_logged[]`) are file-scope and are reset in
`v90_dil_capture_reset()`, whose call sites are per-call or per-retrain, so the
process-lifetime `static` bug that bit `v90_retire_phase2_cc_notch()` is not
present here.  And the analogue side's 1500 ms Ja deadline (9.3.2.4,
`ME_V90_ANALOGUE_JA_SD_BAR_MS`) is the clause value; it is shorter than the
2.1-2.8 s our own descriptor parse takes, so the two halves of this project
deadlock against each other by construction when the fallback does not fire --
which is the argument for fixing the parse rather than lengthening the deadline.

### Why the analogue side cannot see Sd: two thirds of it is at Nyquist (2026-10-01)

The entry above left "the analogue side reports `held-out score 0.000` on every
window while Sd is provably on the wire" as the remaining V.90 blocker.  It has
an exact cause, and the cause is in §8.4.4 rather than in the line, the level
or any threshold.

**First, that 0.000 was never a measurement.**  `v90a_sd_fit()` evaluated both
T/2 parities through `fit_one()` and `continue`d whenever the training-half
check failed -- discarding the score it had just computed, so a rejected window
reported the caller's initialiser.  Every one of the thousands of
`held-out score 0.000` lines in every analogue-role capture in `artifacts/`
means "no fit was accepted" and nothing more.  Fixed; the accept decision is
unchanged, and the log now also carries the 1333 Hz line fraction beside it.
With real numbers the same recording reads **0.396 at the Sd burst** and about
0.333 everywhere else, which is a very different picture from a flat zero.

**The arithmetic.**  Sd is `+W 0 +W -W 0 -W`, period six at the 8 kHz symbol
rate.  Its DFT over one period is |X| = 2 at k = 1 (**1333.33 Hz**) and |X| = 4
at k = 3, which is **4000 Hz -- Nyquist** -- and zero everywhere else.  By
Parseval that is **one third of Sd's energy on the 1333 Hz line and two thirds
at exactly half the sampling rate.**  A 2-wire loop has a null there, no linear
equaliser can restore what the channel removed, and so the best a
least-squares fit to the reference can possibly explain is the surviving third
-- which is the 0.333 plateau, measured.

**The synthetic control isolates the channel, one variable.**  The same Sd
sequence, byte-exact as the SIP G.711 bearer delivers it: **1.000, accepted**.
Low-passed at 3.4 kHz and nothing else changed: **0.456, rejected**.  The real
thing off the Apple modem's loop: **0.396**.  `FIT_SCORE_MIN` is 0.80.  So this
detector cannot fire on any analogue bearer, and the figure in its own comment
("Sd through a dispersive channel at any sampling phase reaches 0.97+") was
measured on a byte-exact DS0, where there is no channel.

**What survives is a better detector than the fit ever was**, because the
component that survives is also the distinctive one.  Over the Phase 3 era of
`reference-data/artifacts/apple-v90-sip-r7`, in 32 ms windows on the 16 kHz T/2 grid: the Sd
burst reads **0.994-1.000** of the block energy on the 1333 Hz line, and the
other **335** windows -- TRN1d, Jd, silence -- read at most **0.0377**.  1333.33
Hz on a 16 kHz grid is exactly one cycle per 12 samples, so it is a plain
period-12 DFT bin with no window and no leakage to correct.  `v90a_sd_line()`
measures it and `v90a_sd_line_taps()` builds what it can justify: a unit main
tap on the stronger T/2 parity, scaled so the received line comes out at the
amplitude §8.4.4's own fundamental has when W is 1 (two thirds).  It is
consulted only after the waveform fit has declined, so a byte-exact DS0 never
reaches it.  `ME_V90_ANALOGUE_SD_LINE=0` disables it,
`ME_V90_ANALOGUE_SD_LINE_MIN` moves the 0.50 gate.

Replayed through the engine, one variable (`apple_usb_modem_coupler
--rx-replay` on r7's own receive tap): **with it, `Sd acquired on its 1333 Hz
line (line fraction 0.539, amplitude 3627, T/2 parity 1)` at the exact Sd
onset; with `ME_V90_ANALOGUE_SD_LINE=0`, no acquisition of any kind for the
whole call.**  `v90_analogue_rx_test` and `vpcm_loopback_test --all-tests`
both still pass, which is the byte-exact path saying it is unaffected.

**THE NEXT STEP IS NOW EXACT, AND IT IS NOT A TUNING PROBLEM EITHER.**  Over a
band-limited channel Sd's slot structure is not merely attenuated -- **it is
inverted**.  Reconstructing `+1 0 +1 -1 0 -1` from its 1333 Hz bin alone gives
`0.333  0.667  0.333  -0.333  -0.667  -0.333`: **the slots that should be ZERO
carry the MOST energy.**  Measured on r7's own Sd burst, mean per slot at the
best phase: **1874  4647  2376  -2205  -4961  -2767** -- the analytic
prediction, to the ratio.  `v90a_sd_put()` scores `(w - z)/w` times the sign
agreement, so on this bearer its first factor is structurally negative and it
can never acquire the slot phase, at any threshold.  Which means:

1. §9.3.2.4's Sd-to-S-bar-d transition has to be taken from the **180 degree
   reversal of the 1333 Hz line**, which is independent of the channel's
   unknown phase shift at that frequency (the line's absolute phase is not,
   which is why it cannot give slot alignment).
2. The equaliser has to be trained on §8.4.5's **TRN1d by CMA** -- TRN1d is
   scrambled ones on one level, i.e. constant modulus, which is what CMA is
   for.  The neutral taps above exist to give it a starting gain and parity,
   not to equalise anything.
3. **Slot and frame alignment have to come from Jd**, since neither Sd nor
   TRN1d can supply it through this channel.

None of that is a threshold change, and none of it is reachable from the
byte-exact loopback, which is why the SIP bearer has never needed it.

### The loop carries the V.90 downstream perfectly well -- Sd is the only casualty (2026-10-01)

The entry above could be read as "an analogue loop cannot carry a V.90
downstream".  It does not say that, and the measurement says the opposite.
Bound every linear receiver the way `tools/v34_channel_bound.py` does, but
with real ground truth for once: the digital side's own `live-tx.g711` is the
exact DS0 it sent, so a least-squares fit from the analogue side's received
tap to those transmitted symbols, scored on held-out data, is an upper bound
on what any linear receiver could recover.  Anchored on the Sd onset, which
both taps carry (TX sample 89120, RX tap 19.600 s), T/2 spaced, lag swept
+/-260 samples in half-sample steps:

| signal | 41 taps | 81 | 161 | 321 |
|---|---|---|---|---|
| **Sd** (R²) | 0.371 | 0.360 | 0.367 | 0.392 |
| **TRN1d** (R²) | 0.823 | 0.829 | 0.848 | **0.860** |
| **TRN1d sign error** | 1.60% | 1.93% | 0.93% | **0.80%** |

**Sd is capped near one third however long the equaliser, which is the
Nyquist arithmetic again and arrives here from a completely independent
direction -- and §8.4.5's TRN1d, the very next signal, fits at 0.86 with 0.8%
sign errors.**  Sign-sliced with no equaliser at all, the same TRN1d
descrambles (GPC, taps 18 and 23) to only **58.5%** ones over a sweep of
sampling phase and +/-3000 ppm; equalised it is 99.2% correct.  So the ~40
points between those two figures are exactly what an equaliser is worth here,
and the loop is not the problem: **§8.4.4's Sd is the one signal in V.90
Phase 3 whose energy is mostly at Nyquist, and it is the only one this bearer
destroys.**

That also settles the design question the previous entry left open.  Training
the equaliser on TRN1d by CMA is not a hope, it is a thing the channel
measurably supports; and not trying to equalise on Sd is not a compromise,
because there is nothing there to equalise on.

**Two method traps, both of which gave the wrong answer first and both already
on record for `v34_channel_bound.py`.**  A lag sweep that is too narrow reads
as a dead channel: at +/-30 samples TRN1d fitted at **R² = 0.002 and 47.3%
sign error, i.e. chance**, and the same data at +/-260 gives 0.86 -- and the
tell was that every reported optimum sat on the edge of the sweep.  And a
window that is too long reads as a dead channel for a different reason: at
12000 symbols (1.5 s) the two free-running codecs drift apart within the
window and the fit degrades to 0.32 and below, swinging with tap count, where
3000 symbols is stable across 41 to 321 taps.  **Check that the winning lag is
interior and that the answer is stable in tap count before believing either a
good bound or a bad one.**

Echo was ruled out before any of this and is not a factor on this port: over
the 0.9 s in which the digital side is silent before Sd, our own transmit runs
at RMS 3132 and the receive tap reads 226, and a 96-tap fit from our transmit
tap to our receive tap over the TRN1d era explains nothing (held-out
R² = -0.010).  **That 226 is the line's noise floor, NOT the echo** -- see the
V.91 entry below, where a wide enough delay sweep finds the hybrid return at
121 ms and puts the true return loss at 27.2 dB.  Reading the ratio of the two
RMS figures as an echo measurement, as an earlier version of this paragraph
did, measures whichever of the two is larger.

**And it holds for the whole downstream, not just the first window.**  The same
fit stepped across the digital side's Phase 3 in 3000-symbol windows, 161 T/2
taps, each window fitted and scored on its own halves:

```
  tx+   432  TRN1d   R2 0.848   0.93%      tx+ 12432  TRN1d   R2 0.813   2.80%
  tx+  3432  TRN1d   R2 0.814   2.33%      tx+ 15432  TRN1d   R2 0.814   2.07%
  tx+  6432  TRN1d   R2 0.819   1.93%      tx+ 21432  Jd      R2 0.881   0.00%
  tx+  9432  TRN1d   R2 0.817   1.87%      tx+ 24432  Jd      R2 0.846   0.00%
```

TRN1d holds **0.9-2.8% sign error over 2.3 s**, and at 2% a ~70-bit Table 13
frame is clean about a quarter of the time, which is ample given how many Jd
frames the digital side sends.  **Do not quote the two 0.00% Jd windows as a
result**: Jd repeats the same frame, so it is a low-entropy reference and an
alignment score against it can be met at a wrong lag -- which is exactly what
their lags of -217 and -258 say, against TRN1d's consistent +48 to +101.  The
scrambled TRN1d windows are the trustworthy ones for the same reason they are
the ones that cannot be faked.

### How cleanly PCM survives v90modem -> SIP -> VG224 -> loop -> Apple modem (2026-10-01)

The bound above was a yes/no on recoverability.  This is the quantity a V.90
constellation is actually chosen from: **how finely can the analogue end
resolve a G.711 level after the whole path?**  Same method and the same
ground truth -- the digital side's `live-tx.g711` against the Apple modem's
device receive tap, held out, on `reference-data/artifacts/apple-v90-sip-r7` -- scored as
residual sigma in G.711 linear units rather than as R².

**THE CLOCK OFFSET IS THE FIRST THING TO TAKE OUT, AND IT IS WORTH 7 dB.**  The
two ends free-run: the DS0 clock arrives through SIP and the VG224, the Apple
codec has its own crystal, and the offset measures **-160 ppm**.  A fixed
filter cannot absorb a drifting sampling instant, so the fit degrades with
window length -- 11.1 dB over 800 symbols, 9.7 over 1600, 7.5 over 3000 --
which reads exactly like a noisy line and is not one.  Let the fit track rate
as well as lag and the same 3000-symbol window goes **8.3 -> 15.4 dB**.  Sweep
rate, or keep the window under ~100 ms, or the number is your own.

With rate tracked, 161 T/2 taps:

| | sigma (G.711 units) | SNR | levels at 6 sigma | bits/symbol | downstream |
|---|---|---|---|---|---|
| measured, 81 taps | 188 | 14.0 dB | 57 | 5.8 | ~46.6 kbit/s |
| measured, 161 taps | **159** | **15.4 dB** | **67** | **6.1** | **~48.6 kbit/s** |
| quiet-line additive floor | 53 | 25.0 dB | 203 | 7.7 | ~61 kbit/s |

and the binary training signals come through outright: **0 sign errors in 1500
held-out TRN1d symbols, a 5.9 sigma margin.**

**Read the two rows against each other, because they say different things.**
The additive floor is assumption-free -- the 0.9 s in which the digital side
transmits digital silence before Sd measures RMS 245 against the TRN1d era's
4365, i.e. **25 dB** -- and on its own it would carry more than V.90's 56 kbit/s
ceiling.  So **the line's noise is not what limits this path.**  What limits
the 15.4 dB is distortion, residual timing, and the measurement chain itself:
a fixed filter with no timing loop, fed through a Python resampler at an
assumed 9600 Hz.  The 7 dB that appeared the moment rate was tracked is the
measure of how much of that is mine, so **15.4 dB is a floor on the path, not
a ceiling.**

**One caveat that cuts the other way, and it is large.**  All of this is
measured on TRN1d, which the digital modem sends at RMS 943 = **-24.6 dBm0**,
while a real V.90 downstream constellation runs up to Table 15's -13 dBm0 --
**11.6 dB hotter.**  If the residual is additive the constellation gains all
of that; if it is proportional distortion it gains none.  Nothing here
separates those, and separating them needs a downstream carrying more than
one magnitude, which this capture does not contain because the call never
reached DIL.

**So, as a single answer: the path demonstrably resolves ~67 G.711 levels,
6.1 bits/symbol, about 48 kbit/s of downstream, with the binary training
signals error-free -- and both the additive floor (25 dB) and the level
headroom (11.6 dB) say the real figure is higher than that.**  What stops the
call is not the bearer's cleanliness; it is §8.4.4's Sd, two entries above.

### The clock offset measured exactly, and what it hid (2026-10-01)

The entry above treated "residual timing" as part of an unresolved floor.  It
is not unresolved -- the offset can be measured to about 1 ppm, and once it is
pinned the remaining residual turns out to be something else entirely.

**Measure it on a tone whose frequency is exactly known.**  The digital side's
Phase 2 CC carrier is 1200 Hz generated from the DS0 clock, and it sits on the
Apple modem's receive tap as a pure tone.  Mix it down, block-average, fit the
unwrapped phase slope:

```
  digital Phase 2 CC   15.50-16.15 s   1200.1961 Hz = +163.4 ppm  (phase residual 0.000 rad)
  digital Phase 2 CC   16.62-17.00 s   1200.1936 Hz = +161.3 ppm  (phase residual 0.002 rad)
  ANSam                 9.50-13.70 s   2099.2313 Hz = -366.1 ppm  (phase residual 0.884 rad)
```

A phase residual of 0.000-0.002 rad over 0.65 s is a frequency good to about
**0.4 ppm**, and the two stretches agree to 2.1 ppm.  **The equaliser fit,
which knows nothing about any of this, independently lands on -161 ppm against
the tones' -163** (the sign flips because one is the received tone running
fast and the other is the resampling step that corrects it).  So the offset is
real, it is the hardware, and it is **163 ppm** -- five times the +26 ppm the
HSF codec measured and the +30 ppm between the two Apple modems.  The ANSam
row is the control for the method and fails exactly as this file already
warns: 15 Hz AM and 450 ms phase reversals, phase residual 0.884 rad, answer
off by 500 ppm.  **Use a plain carrier, never ANSam, and quote the phase
residual beside the frequency or the number means nothing.**

**WHAT THE TIMING ERROR WAS HIDING: with it pinned, decision feedback is worth
11 dB, not the 1 dB measured before.**

```
  161 T/2 taps, linear                    SNR 15.50 dB   sigma 158
  161 T/2 taps + 8 ideal feedback taps    SNR 26.36 dB   sigma  45
  161 T/2 taps + 32 ideal feedback taps   SNR 27.01 dB   sigma  42
  321 T/2 taps + 8 ideal feedback taps    SNR 26.38 dB   sigma  45
```

The earlier "ideal DFE buys 1 dB" was taken at the untracked-timing operating
point, where the timing error swamped the ISI it was trying to measure.  **Fix
the first-order error before testing for a second one** -- that reading was
wrong and is withdrawn.

So the linear residual is **ISI, not noise**: a linear equaliser cannot invert
the band-edge roll-off and decision feedback can, which is precisely why V.90
precodes.  And at sigma 45 the path has arrived at its own additive floor --
the assumption-free quiet-line measurement is **25.0 dB, sigma 53** -- so two
independent methods agree that **this bearer is noise-limited at about 26 dB
once ISI is handled.**  Translated:

| | sigma | levels at 6 sigma | bits/symbol | downstream |
|---|---|---|---|---|
| linear equaliser | 158 | 68 | 6.1 | 48.7 kbit/s |
| **+ precoding / ideal DFE** | **45** | **238** | **7.9** | **63.2 kbit/s** |
| quiet-line additive floor | 53 | 202 | 7.7 | 61.3 kbit/s |

**i.e. the path carries full-rate V.90 downstream with margin -- 7.9
bits/symbol against the Recommendation's own 56 kbit/s ceiling -- and still
with the 11.6 dB of transmit headroom unspent, since all of this is measured
on TRN1d at -24.6 dBm0.**

**The control that says the DFE gain is real** and not the feedback filter
predicting TRN1d from its own past: TRN1d's signs satisfy `s[n] = s[n-18] ^
s[n-23]` through GPC, so a predictor would need taps at lag 18 and 23 -- and
8 feedback taps, which cannot reach them, already give the whole 11 dB, while
32 taps, which can, add only 0.6 dB.  The gain is ISI cancellation.

**AND THE ACTIONABLE CONSEQUENCE, which is a receiver gap not a measurement
one: 163 ppm walks the sampling instant by one 8 kHz sample every 6135
symbols, i.e. every 0.77 s** -- 3.3 symbols across the 20004T TRN1d this
project now sends by default.  The V.90 analogue-role receiver fits its
equaliser on Sd and then runs CMA; there is no symbol-timing tracking loop on
that path at all, so nothing follows that walk.  That is why the offline fit
degraded with window length (11.1 dB at 800 symbols, 7.5 at 3000) and it is
the same thing a live receiver would suffer.  **So the answer to "can we be
more exact about timing" is: offline, to about 1 ppm, and it is now pinned;
live, not at all yet, because the analogue path has no loop to be exact with.**

### V.91 over this bearer, and what duplex actually costs (2026-10-01)

Two separate questions, and the measurements put the weight in a different
place from where intuition puts it.

**V.91 ONE WAY: no, and not by a margin.**  V.91 is bit-transparent 64 kbit/s
-- it must recover *every* G.711 codeword exactly -- and the µ-law ladder's
steps are 2 linear units at the bottom of chord 0, rising to 256 at the top of
chord 7.  Against the measured sigma of **45** (precoded) a 6-sigma decision
needs 270 units between adjacent codes, so **not one adjacent pair is
separable anywhere on the ladder, not even in chord 7.**  Resolving chord 0
needs sigma below 0.33 units, which against a ~4000-RMS PCM stream is about
**82 dB** of SNR; this path gives 26.  It is **~56 dB short**, in either
direction, and no amount of transmit headroom, echo cancellation or
one-way-ness closes that.

That is not a property of this rig, it is what V.91 is: bit transparency
requires a *digital* path, and a single D/A plus A/D converts that requirement
into an analogue SNR demand no loop can meet.  This is the same conclusion the
two-Apple-modem V.91 attempt reached from the clock side, now from the
amplitude side.

**What IS available one way is reduced-rate PCM, i.e. V.90's downstream** --
7.9 bits/symbol precoded, measured two entries above.  If "V.91 at a reduced
DRN" is what is meant then yes, that works and it is the same constellation
problem V.90 already solves; what does not survive the reduction is the thing
that makes V.91 V.91.

**DUPLEX: real, but small here -- about 1.7 dB -- and the earlier echo figure
in this file was wrong.**  Over the 0.85 s in which the digital side is
provably silent and we are transmitting Ja, our TX runs at RMS 3142 and the
receive tap reads 243.  Quoting the ratio of those as "-22.2 dB of echo" is
what an earlier entry did and it is wrong: it measures whichever of echo and
noise is larger.  Separate them with a wide delay sweep and an ideal canceller:

```
  |corr| peaks at lag 1160 samples = 120.8 ms   (and weakly at 379 ms)
  ideal canceller: ERLE 1.6 dB, residual rms 201
  received 243  =  echo 137  +  floor 201
  TRUE hybrid return loss = 27.2 dB
```

**120.8 ms is this host's own CoreAudio and USB buffering**, the 122.6 ms this
file already measured between transmitting and hearing yourself -- so that
peak is the hybrid, delayed by the host, and a sweep that stops short of it
(mine first stopped at 900 samples) finds nothing and reports the noise floor
as the answer.  The weak 379 ms peak is consistent with the 269 ms network
round trip plus the same host latency.

Against a downstream arriving at RMS 4365, the uncancelled echo sits **30.1 dB
down** and the noise floor **26.7 dB** down, so adding the echo to the floor
costs **1.7 dB**, about 0.3 bits/symbol.  **Two things scale that, and they
matter more than the duplex question itself:** the echo is proportional to
transmit level, and this was measured at `APPLE_MODEM_TX_AMP=0.35`, so at full
level it would be ~9 dB hotter and the penalty ~6.7 dB; and **this is modem A.
Modem B's hybrid collapses to ~2 dB return loss a second after it goes
off-hook**, where the echo would be far above the far-end signal and duplex
would not work at all without a canceller that converges.

**So the ordering is: duplex costs ~2 dB on this port and ~7 dB at full
transmit level, which is a constellation step or two; V.91's bit transparency
is 56 dB away and is not a margin question at all.**


### Sd line recovery implemented and independently graded (2026-10-01)

The V.90 analogue **16 kHz frontend** now implements the three steps above.
It first tries the existing supervised Sd fit. Only if that fails and the
1333 Hz line passes `ME_V90_ANALOGUE_SD_LINE_MIN` does it select measured-line
recovery. `ME_V90_ANALOGUE_SD_LINE=0` disables the fallback; exact codewords,
the successful waveform-fit path, plain V.34 and V.92 do not select it.
`ME_V90A_SLOT_LEVELS=0` also keeps it out of the exact-codeword receiver.

* **Transition:** a coherent two-cycle sliding DFT tracks the 1333 Hz line.
  A sustained opposite phase, with line energy still present, proves the
  180-degree reversal (V.90 §8.4.4, §9.3.2.4). Its magnitude minimum dates
  the reversal in the receive stream. This event ends Ja. A level drop or
  silence cannot substitute for a reversal. After the specified 48T of
  S-bar-d, measured from that reversal, the frontend enters TRN1d.
* **Equalizer:** neutral line-derived gain/parity taps remain frozen during
  Sd/S-bar-d; §8.4.5's TRN1d starts CMA at `V90A_FSE_MU_CMA`. The line path
  does not use the experimental DD mode. It keeps CMA through Jd, whose
  transport also uses the same two levels, and freezes it before DIL. The
  2040T confirmation is reported, but a marginal early count does not discard
  this acquisition while CMA is still converging. Table 13's structure and
  CRC decide whether Jd was recovered.
* **Alignment:** Jd is probed at every symbol. Its first frame may inherit raw
  TRN1d scrambler history; later frames inherit *differential* Jd history
  (§5.3, §8.4.2). Both histories are tried on the line path, including raw
  polarity ambiguity. This matters when noise spoils the first frame.
  A validated 72T boundary establishes the bit/frame and six-slot grid and
  publishes Table 13's constellation fields before upstream S starts.

A separate sample-accounting defect prevented this path from running:
`apple_usb_modem_coupler`'s 9600-to-16000 resampler calls the engine with one
or two samples. The equalized frontend processed only complete pairs and
**discarded the one-sample calls**, i.e. one sample in five on that schedule.
It now feeds every sample and lets the FSE retain T/2 parity across calls.
The acquisition window also consumes every caller sample, including any tail
when a window is rejected. No resampler, buffer or codeword conversion was
added to the DS0 bearer.

The preserved `reference-data/artifacts/apple-v90-sip-r7` RX and TX taps were graded by
`tools/apple_v90_sd_recovery_verify.py`, which needs NumPy. This is a local
capture target; missing captures cause an error rather than a skip:

```sh
make v90-apple-line-test V90_LINE_PYTHON=.venv/bin/python
ME_MODE=v90 ME_V90_ROLE=analogue VPCM_ME_VERBOSE=1 \
  ./apple_usb_modem_coupler --rx-replay \
  reference-data/artifacts/apple-v90-sip-r7/call/rx-tap.s16 --pty-link /tmp/v90-line-replay
```

The independent frontend replay crops RX to 19.4–23.8 s, after Phase 2:
**dialling tones can also have a 1333 Hz line**, so hunting the entire audio
file without the engine's Phase 3 gate is invalid. The causal DC blocker and
5/3 interpolation reproduce the coupler's frontend; the digital TX tap is
used only by the grader, never as equalizer training input.

1. **Reversal:** the tracker dates it at crop sample **4205**; an independent
   single-cycle coherent projection crosses at **4207**, 0.125 ms apart.
   Both are receive-tap positions, not transmitter timestamps.
2. **CMA:** sixteen independent 1000-symbol TRN1d windows, starting at 2500T,
   recover **16000 actual TX signs with mean 0.78% error, worst 1.80%**.
   Unequalized signs on those same windows average **26.21%**. The lag sweep
   is ±260 symbols and every trained optimum is interior, at -9; each short
   window is graded independently to avoid treating free-running clocks as
   synchronized. Polarity is allowed to be arbitrary for this channel grade.
3. **Jd:** **eight CRC-valid frames**, every one bit-identical to the first
   TX Jd independently differential/GPC-decoded in Python. Their receive
   endpoints agree on the 72T/six-slot grid. This comparison validates fields
   and framing; repeated Jd is not used as an equalizer quality measurement.

The complete engine replay also reaches those eight Jd frames, starts
upstream S, recognizes J'd and enters DIL. With the fallback disabled, r7
never acquires Sd. The preserved r6 replay, which never completed the
requisite exchange, publishes no line-recovery transitions.

Default offline regressions cover all twelve fundamental phases, reversal
instant and 48T duration, attenuation/silence rejection, and later foreign
Jd frames after deliberately corrupting the first one, in both polarities.
`v90_analogue_sd_test`, `v90_analogue_rx_test`, `v92_startup_test` and
`vpcm_loopback_test --all-tests` pass. `make test` stops at the unrelated
plain V.34 **3000/9600 µ-law** row (`trained=0/0`): that binary links only
`v34_duplex_test.o` and SpanDSP, none of the modified analogue receiver files.

This is a Phase 3 recovery result, **not a new live CONNECT claim**. Absolute
DIL scaling is still conditional on the peer honoring requested U_INFO: CMA
fixes its output modulus but cannot identify the transmitted absolute Ucode.
The instrument labels that reference as requested rather than measured.
Long-run clock tracking, late TRN1d margin, DIL and Phase 4 through this loop
remain separate work; the good 16000-symbol channel grade does not claim that
every symbol of the entire recording was recovered.

## Through the Grandstream ATA against `sip_v90_modem` (2026-10-15)

The modem now hangs off the Grandstream ATA's line 2 (**6005**), not the VG224,
and dials the digital end, which ran from a HEAD build on vmhost CT 117
(`/root/apple-head`, ext **6002**) because tower was unreachable.  Recipe:
`reference-data/artifacts/apple-pve-20261015/run.sh <name> <mode> [hold]`; it dials `6002#`
with `APPLE_MODEM_DTMF_MS=150/150` and collects both ends' logs and taps.

### Rig: every `endpoint-modem` endpoint was missing from Asterisk

The ATA could not register (8576 rejected REGISTERs for 6005 that day, "No
matching endpoint found" then "Failed to authenticate"), and **this ATA removes
loop battery while unregistered**, so the modem read line sense 0x00, "no pair",
and went on reading it on-hook.  Asterisk had restarted on 2026-10-12 22:34
and parsed `allow=!all,clearmode,ulaw` (the `endpoint-modem` template) before
`codec_clearmode.so` had registered its format, so **every endpoint built on
that template failed to load**: 6000/6001 (the long-standing "6000 does not
authenticate"), 6004/6005, and all of the VG224's 84xx.  `module reload
res_pjsip.so` with the codec already loaded fixed it, and both ATA lines
registered within seconds.  A restart will break it again unless the load order
changes.  The first symptom is a 400 Hz tone with ±20 Hz sidebands after
dialling, which is the ATA saying it cannot route.

### What the line is

- **Round trip ≈ 0.5 s**: the digital side's V.34 echo canceller fits a bulk
  delay of 533 ms; ICMP from Asterisk to the ATA is 36–50 ms, so most of it is
  the ATA's jitter buffer.
- **The ATA drops nothing downstream.**  Counting 20 ms holes (a frame under
  25% of its neighbours) in all four taps, every hole the Apple modem received
  was already in the server's own transmit.  Upstream, the V.90/V.92 calls
  show 4–5 holes at the server against 2 sent: at most 2–3 per minute.
- Echo is negligible: on V.22bis each side's own band is 0.5% of what it
  receives.

### Mode matrix, one call each, 60 s hold

| mode | furthest point |
|---|---|
| V.22bis | V.8 → training; both ends restart in a coupled ~1.16 s cycle |
| V.32bis | V.8 → start-up; nothing logged for 55 s on either side |
| V.34 | Phase 4 MP both ends at 3200 baud (caller projected N=13); caller "Training failed" 3.6 s into MP; the retrain stalls in Phase 3 |
| V.90 | **Phase 3 complete on both ends**; digital enters Phase 4 (Ri); analogue stuck awaiting J'd |
| V.92 | same as V.90 (V.92 INFO0 mutual, but no PCM upstream offered, so it runs as V.90) |

**V.22bis.**  SpanDSP restarts the transmitter as well as the receiver on a
carrier drop once the receiver is past symbol acquisition
(`v22bis_rx.c:919`), so one drop at either end starts a loop: the answerer goes
back to 75 ms of silence and USB1 (still on air 15 s in; the 2250 Hz line is
dibit 11 at 600 baud), the caller loses carrier and restarts, and its
transmitter bursts 0.6 s on and 0.66 s off.  `engine_pair_test` connects V.22bis
with no delay; the 0.5 s round trip is the obvious difference and is
untested.  The `Training rx: RMS=0.0` line on the answer side is wrong: its
own G.711 tap carries the caller at RMS ~1200 throughout.

**V.34 retrain.**  After the first MP failure, both ends' INFO1 report
`max N by baud ... all 0`, and the digital side says `no legal measured duplex
symbol-rate pair`: the retrain has no line-probing result to choose from, so it
cannot succeed as built.

**V.90/V.92: first live Sd → TRN1d → Jd on the analogue side.**  The line
recovery from the section above works live: `Sd acquired on its 1333 Hz line
(line fraction 0.568)`, S-bar-d reversal found, TRN1d CMA, and **a Jd frame with
0 bad bits at TRN1d 20084T** (the digital side sent 20004T).  Repeated on the
V.92 call (fraction 0.662, Jd at 20083T).  The digital side detects the
analogue S after 5964 Jd symbols, sends J'd and enters Phase 4.

**The remaining blocker is still Ja, upstream.**  The digital side never decodes
a CRC-valid Ja descriptor (`parsed=0` throughout) and starts Sd only on the
energy-gap fallback 500 ms after the gap.  With no descriptor it sends **no
DIL**: J'd is followed by Ri.  The analogue side requested a 120-segment DIL,
and it leaves Jd only on J'd's 12 zeros or on the DIL's level change.  It saw
neither, and sat in "S (awaiting J'd)" for 47 s, until the digital side's 20 s
Ja deadline conceded to V.34 and retrained.  Next: rerun V.90 with
`V90_JA_BIT_DUMP` and the server's `v34_rx.raw` to replay the Ja decode
offline, and find out why the analogue side's J'd bit walk missed the 12 zeros
(it logged only the first Jd frame, so a slip after it is possible).

### Ja fails on our analogue transmitter's clock: ~195 ppm, and V.34 allows 100 (2026-10-15)

`reference-data/artifacts/apple-pve-20261015/v90-ja1` repeats the V.90 call with
`V90_JA_BIT_DUMP` and the server's `v34_rx.raw`.  Same outcome: the Apple side
recovers Sd → TRN1d → Jd (this time it also sees J'd and sits in DIL receive),
and the server never parses Ja.

**The live dump cannot answer this, and it looks as if it can.**
`V90_JA_BIT_DUMP` is rewritten on every pass until the call ends, and each
hypothesis is a 16384-bit ring (~2.5 s at 6400 bit/s), so at the end of a 40 s
call it holds the last 2.5 s, not Ja.  Its bits are white, but so are the
engine pair's, whose Ja parses.  Use the one-shot dump on a replay:
`ME_V90_JA_DUMP_EARLY=1 ME_V90_JA_DUMP_PREFIX=... ME_V90_JA_DUMP_MIN_BITS=8000
./v90_engine_replay live-rx.g711 ulaw --fast --from 0`.  The replay reproduces
the live capture start exactly (first Ja bits at 11.160 s, 48 bits).

**Ground truth is free, because the sender is ours.**  `v90_dil_preset_load(
V90_DIL_PRESET_MEASUREMENT)` + `v90_build_dil_descriptor_bits()` regenerates
the 1260 transmitted bits, and the analogue side's TX stage log dates TRN
(640 ms) and Ja (533 ms: 9.812-10.345 s, cut short when it saw Sd).  Scoring
each 128-bit window of every hypothesis against the descriptor (any rotation,
either polarity):

- **engine pair (positive control):** hypotheses 8 and 11 match solidly from
  bit ~3600; aligned BER **0.00**.
- **Apple call:** the **same hypotheses 8/11** match in only two isolated
  windows inside the Ja span; aligned BER **0.07-0.25 per 256 bits**
  (0.18 typical), then 0.48 when Ja stops.

So the digital side demodulates Ja on the right hypothesis with ~18% bit
errors, and the parser is exonerated.

**The upstream is drifting, not noisy.**  A least-squares FIR from the Apple
side's own TX tap (9600 → 8000) to the server's receive tap peaks at
**17-22 dB in the middle of a 0.5 s window and falls to ~6 dB at both
edges**, symmetrically: the timebase is sliding under the fit.  Tracking the
TX-to-RX offset through Phase 2-3 gives a smooth slope from 7.4901 to 7.4883 s
over 9.25 s: **-195 ppm** between the Apple codec and the ATA's 8 kHz (163 ppm
against the VG224, above).  (Steps of exactly 0.3 s in V.8/Phase 2 are
correlation aliases on periodic signals, not jitter-buffer jumps.)  Across
TRN + Ja that is ~0.75 of a 3200-baud symbol.

**Single-variable test, offline:** the server's `live-rx.g711` FFT-resampled
by -195 ppm, re-encoded to µ-law (the round trip changes only -0 to +0) and
replayed:

| tap | Ja |
|---|---|
| as recorded (0 ppm control) | no descriptor; concedes after 20 s |
| **-195 ppm** | **`parsed Ja DIL descriptor (N=120 LSP=12 LTP=11)`**, the descriptor sent, between 12.00 and 12.24 s, inside the Ja window |
| +195 ppm (wrong sign) | no descriptor |

**The fault is our analogue transmitter, and the spec says so:** V.34 §5.2
fixes the symbol rate at S ± 0.01%, and this codec puts our Phase 3 symbols
~195 ppm off.  The SM56's host datapump must have loop-timed its transmitter
(V.34 INFO0 bits 26:27 = 1, "synchronized to receive timing"), and
`apple_usb_modem_coupler` free-runs it.  The fix belongs in the coupler: a
receive-timing-derived correction to both rational resamplers (the 1200 Hz
Phase 2 CC already measures the offset to ~1 ppm), not a wider timing range in
the digital receiver.  That is a resampler ratio, not buffering: no samples are
spliced or dropped.  Every upstream stage after Ja (CP, MP, data) inherits the
same offset, so expect this to be the next gate for V.34 MP too.

### Loop timing in the coupler: Ja parses on every attempt (2026-10-15)

`apple_usb_modem_coupler` now loop-times its transmitter (V.34 INFO0 bits
26:27, "synchronized to receive timing"), which is what the section above
calls for.

**Measurement.**  The far end's ANSam, on the raw device stream before any
resampler, mixed to baseband in 20 ms blocks and **squared**, so V.25's 450 ms
phase reversals vanish; the unwrapped phase is fitted for frequency.  From
answer detection it takes 2 s of tone (or what there is, minimum 0.8 s),
gated at 0.2 rad residual and ±1000 ppm.  Live it reads **-164.8 to -165.6
ppm at 0.003-0.06 rad** on every call; unsquared the same fit gives the
-364 ppm / 0.88 rad that made ANSam look unusable before.

**Correction.**  All three resamplers became one variable-ratio polyphase:
240 phases (divisible by 3, 5 and 6, so at 0 ppm every output lands exactly on
a phase and the old exact-rational behaviour is unchanged), 48 taps per phase
as before, adjacent phases interpolated off nominal, and an integer position
accumulator.  Receive ratios are scaled by (1+d), transmit by 1/(1+d), so one
device sample in still owes one device sample out: nothing is spliced.
`--selftest` now also runs every path at ±200 ppm, for 3 s so the position
wraps several times: gain 0.998-1.003, SNDR 52.7-78.9 dB, the nominal figures.

**Verified at the far end.**  Our Phase 2 2400 Hz tone, measured on the
digital side's receive tap: **2399.604 Hz (-164.9 ppm) before, 2400.0000 Hz
±0.1 ppm after**, residual 0.001-0.003 rad.

**The trap it exposed: the transmit ring ran at exactly zero margin.**  The
first loop-timed calls slipped the upstream by one device sample (a clean
90 degree step at 2400 Hz) every 0.64 s: 32-33 underrun runs, ~300 samples
per call.  The rates were balanced (stage ratios +153 and -151 ppm); the cause
is granularity.  The engine produces 96-sample blocks and CoreAudio takes
512-frame cycles, so the ring's fill swings by up to a block with where block
edges fall against cycle edges, and the startup underruns (the engine starts
part way through a cycle) left its low point at exactly empty.  At exact 5/6
and 6/5 that alignment never moves; loop timing moves it one engine sample per
rx-resampler wrap (0.63 s at 165 ppm), and each crossing dipped below empty.
**`--rx-replay` hid it, because it drained in 80-sample chunks**;
`APPLE_REPLAY_CHUNK=512` reproduces the live count exactly (32 runs, 286
samples).  Zeros pre-filled into the ring, or owed to the output side, were
both eaten by the startup underruns.  The fix is a pre-roll: after engine
start the output plays silence, without reading the ring, until the ring holds
a whole callback plus 192 samples, then drains.  Zero underruns at replay
chunks 80/512/1024 and on every live call since.  New exit line: underrun
runs and samples (dated), device in/out frame counts (equal on every call:
input and output share the clock) and the ring's minimum fill per 0.5 s.

**Live, V.90, Apple on ATA 6005 → HEAD `sip_v90_modem` on CT 117, final build,
alternated arms:**

| loop timing | connected calls | Ja parsed | server accepted CPt |
|---|---|---|---|
| on | 3 (lt6, ab-on1, ab-on3) | **9 / 9 attempts** (start + 2 retrains each) | 5 |
| off | 4 (nolt4, ab-off1, ab-off2, ab-off3) | **0** | 0 |

(Before the change, 0 of 2 more; ab-on2 and lt5 never reached the server, a
dial failure, not an arm.)  With Ja parsed, the digital side sends the DIL
the Apple side asked for (120 segments, LSP=12, LTP=11), the Apple side
receives it and enters Phase 4 (CPt at drn 16, CP at drn 4 = 32000 up), and
the server accepts that CPt and sends TRN2d (D=24, K=18) and MP.

**Next blocker: the analogue side's Phase 4 receiver never finds Ri.**  It
logs `hunting Ri` from Phase 4 start until its §9.4.2 deadline (~11 s) and
retrains, on every attempt; the downstream TRN2d/MP the server sends are never
acquired.  Downstream timing is now on the far end's clock too (the receive
resamplers are corrected), so this is the analogue Phase 4 receiver itself.

### Ri was never acquired because §8.6.4's R is not one level on a line (2026-10-15)

**The wire is right.**  The digital side's own tap (`server/live-tx.g711`,
v90-lt6) holds exactly §8.6.4's Ri: `31 31 31 b1 b1 b1`, Ucode 78 (= U_INFO),
`+++−−−`.  It runs for 11.6 s on the first attempt, where the server never
accepted our CPt.  On the second it runs 763 ms, and R̄i is 24T with the run of
six at the junction (§8.6.4, §9.4.1.2), then TRN2d from sample 294106.

**The analogue receiver saw it all along.**  `V90A_RX_CODEWORD_DUMP` on
`--rx-replay` of the same call shows the sliced stream holding the
`+++−−−` sign pattern on 100% of symbols for 10 s.  It never sits at one
level, though.  Per slot of each run of three it reads **Ucode ~64 / ~82 /
~68** (linear 527 / 1151 / 623 around a sent 943), fixed by slot for the whole
of Ri.  That is the physics from the Sd section above.  `+++−−−` times (−1)^n
sums to 2 per period, so its 4000 Hz component has a third of the level.  No
loop passes 4000 Hz, and without that component the run reads 2/3, 4/3, 2/3.
The Ri hunt asked for twelve symbols at one level, within `level_tolerance`
(0.35), so it could never fire.  That is the `hunting Ri` from Phase 4 start to
the §9.4.2 deadline on every attempt.

**Two more traps behind it, both found by replay:**

- Once Ri was held (6194T), a single symbol off level dropped it four symbols
  before R̄i.  The equalised channel's tails pull the levels on *both* sides
  of the junction's run of six (−50 and +75 against slot levels 67 and 64).
- In R̄i, `ucode != r_ucode` (the DS0's "level changed, so TRN2d has begun")
  is true on almost every symbol on a line.  It would have started TRN2d 11
  symbols early, off the §5.4 frame grid for the rest of the call.

**Fix (`v90_analogue_phase4.c`, level-tolerant mode only; the DS0 path keeps
its exact tests):**

- The hunt compares each slot with the same slot one period earlier.  An LTI
  channel cannot change the fact that R repeats every six symbols.  It also
  records the six slot levels.
- R is dropped only after a whole period of R-signed symbols off their slot's
  level, and reversed symbols are judged by sign alone.  The sign carries
  §9.4.2.2's transition; the level only rejects a false R.
- R̄i is counted to exactly 24T.

**Verified offline against ground truth.**  On v90-lt6 the receiver now
acquires Ri, then R̄i, then TRN2d on both attempts where the server sent R̄i
(attempt 1 holds Ri until the deadline, which is correct: the server stayed in
Ri).  TRN2d's first symbol lands at cw 161090 and 280964.  Aligning the
server's tap to the received stream by sign correlation (93% agreement) puts it
at **exactly those symbols**, so the frame grid is right.  v90-ab-on1 and
v90-ab-on3 behave the same way on every attempt where the server sent R̄i.
`make test-fast` 546/546.

**Live, four calls after the change** (alternating nothing; the change only
acts once a Phase 4 receiver exists):

| call | Ja parsed by server | Phase 4 on the analogue side |
|---|---|---|
| v90-ri1, v90-ri3 | no (conceded after 20 s) | never reached.  The server goes to Ri without sending the DIL, and we wait in DIL receive |
| v90-ri2 | -- | V.8 failed on the analogue side (rig flake) |
| v90-ri4 | 3 / 3 attempts | attempts 1 and 3: server never accepted CPt and stayed in Ri; **we held Ri** until the deadline.  Attempt 2: server accepted CPt; **we acquired Ri (6418T), R̄i and TRN2d live**, switched to sending CP, and the server accepted that CP (drn 4) 60+ times while sending MP.  MP never decoded: see below |

Before the change, 0 of 9 attempts ever left `hunting Ri`.  Ja yield (2 of 4
calls) is rig variance in Phase 3's upstream, unchanged by this.  Two
digital-side facts showed up on the way and are separate follow-ups: the
server proceeds to Ri with no DIL when Ja fails, and on roughly half the
attempts it never accepts a CPt that the analogue side keeps sending.

**Next blocker: TRN2d decodes to nothing, and the cause is level accuracy, not
timing.**  Over TRN2d's first 4000 symbols the sliced codewords match the sent
ones **5%** of the time, with signs right 94% of the time.  A least-squares fit
of received against sent linear levels gives gain **0.79** (2 dB low, so every
Ucode reads 1-8 low).  SNR is **7.9 dB** with one tap and **14.6 dB** even with
±8 taps of residual ISI.  The equaliser was frozen at the DIL in the
sign-slicing (`Sd line`) mode and never calibrated, and CPt demapping needs
codewords that are exact.  TRN2d is known data (§8.6.5: scrambled ones through
the CPt mapping, all state zeroed at its first frame, which now lands on the
right symbol).  So the obvious next step is data-aided training and gain
calibration of the analogue equaliser over TRN2d, under the 4000 Hz null this
section and the Sd section both describe.

### TRN2d: our own echo, then the 4000 Hz null, then error propagation (2026-10-15)

The follow-up to the Ri fix: once TRN2d started on the right symbol, only 5%
of its codewords came out exact.  Four findings, in the order they were peeled
off.  Offline throughout, graded against the digital side's own tap
(`server/live-tx.g711`) with least-squares fits on the equaliser's T/2 input
(`V90A_T2_DUMP`, new) as the bound on what the samples support.

**1. The line is clean; our own transmit was the noise.**  The same fit that
reaches only 20 dB in Phase 4 reaches **59.7 dB on TRN1d**, where the Apple
side is silent.  The line's floor (pre-answer silence) is RMS 2-3 against 1020
for TRN2d.  The impairment is our own upstream coming back through the hybrid
~118 ms later (the USB round trip: 1133, 1142 and 1165 device samples on three
calls), at **-16.7 dB against the downstream** in Phase 4.  Nothing cancelled
it.  The echo is linear but long: on a window where only the echo is present
(V.34 Phase 2's L1/L2, 38.65 s of v90-lt6), 64 taps remove 14 dB and 384 taps
remove **31 dB**, down to the line floor.  The same fit holds 30 dB on the next
attempt's probing 20 s later.

Fitting it under Phase 4's double talk does not work.  An LS fit's excess
error is taps/samples times the interference, so a 7 dB reduction was all a
double-talk fit managed.  **`near_echo.c`** (new, wired into the coupler in the
device domain, where both directions share one clock) therefore accumulates
only echo-only 50 ms blocks: received at least 20 dB below transmitted.  That
passes the -27 dB echo and rejects a downstream.  It fits an exact
covariance-method LS of 768 taps (delays 94-174 ms).  The solve
(~75 M operations) runs on a worker in a live call, and every fit is installed
at a fixed sample count, so a live call and its `--rx-replay` cancel
identically.  The unit test (`near_echo_test`) reaches 47.9 dB ERLE under
continuous double talk after a 2 s echo-only stretch, and checks the worker and
inline paths are bit-identical.  In replay
(`APPLE_MODEM_EC_REF=<call>/tx-tap.s16`) the decision-feedback bound on Phase
4's MP goes from **20.0 dB to 47.6 dB**, and slot-constellation SER from 10%
to 0.  On by default in the analogue PCM role (`APPLE_MODEM_ECHO_CANCEL=0/1`
overrides).

**2. A two-wire loop needs decision feedback.**  With the echo gone, the best
linear T/2 equaliser still reads 13.8 dB (16.9 at 256 taps): the 4000 Hz null
again (see the Sd and Ri sections), which a linear filter can only invert by
amplifying noise.  32 T/2 + 16 feedback taps reach 49.7 dB on TRN1d and 41-45
dB on TRN2d/MP.  `v90_analogue_fse` gained feedback taps
(`v90a_fse_set_feedback`, 16 in the engine, `ME_V90A_DFE_TAPS`).  Getting
there took three corrections, each found with a harness running the real FSE
on the dumped T/2 stream:

- NLMS-DD from the CMA solution never moved off the CMA's ~8 dB: the null
  makes the problem ill-conditioned exactly where feedback is needed.  So
  TRN1d is solved by least squares instead (`v90a_fse_train_ls`), from TRN1d
  4000T.
- One pass learns the CMA's 2.8% wrong decisions and stops at 22.5 dB.  Three
  passes of 3000, each deciding with the last pass's taps, reach 48.9 dB.
- The solve's diagonal loading was relative to the mean diagonal.  The forward
  regressors are raw counts (the Sd fit freezes the AGC at 1) and the feedback
  ones are near 1, so the loading pinned every feedback tap to zero.  The
  engine read 12.6 dB, exactly a forward-only equaliser.  It is now relative to
  each element.

**3. Feedback needs coarse decisions, so the DIL and Ri need care.**  A
feedback loop fed decisions that merely track its output is a linear inverse
of the null, and that is unstable.  The DIL's 121-Ucode ladder is that fine
near zero: its quiet opening grew a Nyquist oscillation x1.35 a symbol to the
top of the ladder within 30 symbols, and the DIL never completed.  Three
changes in the engine:

- In the DIL the decision fed back is the codeword this side asked for.  We
  authored the descriptor (§8.4.1), `v90_analogue_rx` pins its first symbol,
  and the polarity is the one resolved on TRN1d.
- Ri/R̄i are sliced onto U_INFO alone (§8.6.4).
- Phase 4 slices onto the CPt/CP points for the next frame slot, through the
  previously unused `v90_analogue_phase4_slicer_ucodes()`.

There is also a runaway guard in the FSE: an output beyond any G.711 level
clears the feedback history.  With these the DIL measures properly for the
first time.  Constellation selection reports real noise-limited choices
(Mi 12-17 per slot, 36-41 kbit/s at a 3-sigma margin) instead of "none at a
3-sigma margin" and a 0-sigma fallback.

The taps are then **held** after the TRN1d solve.  Tracking at MU_TRACK left
Ri at 36.9 dB and no attempt decoded; holding gives 43.7-46.6 dB.  Ri is
periodic, and decision-directed feedback on a periodic sequence learns to
PREDICT it (d[k-6] = d[k]) instead of the channel; ME_V90A_TRACK_MU=<step>
restores tracking for experiments.

Replay needs the call's own constellation now.  Every change upstream of the
DIL changes the CPt this side sends, so a replay's Phase 4 no longer matches
the recording.  `V90A_CP_DUMP` (live) / `V90A_CP_LOAD` (replay) pin it.  Replay
with `APPLE_REPLAY_CHUNK=512`: 80-sample chunks change the outcome against the
live call, and 512 reproduces it exactly.

**Result.**  Live (v90-dfe1..3) the digital side now accepts our CPt and CP,
sends TRN2d and MP, and goes on to **B1d** (and upstream data mode) on most
attempts: the first complete Phase 4 from the server's side on this rig.
v90-dfe3 replayed with its live CP:

| | Ri | TRN2d 0-224 | TRN2d after |
|---|---|---|---|
| attempt 2 | 46.6 dB, exact 100% | 37 dB, **exact 100% for 1319 symbols** | one error, then a cascade (41% exact); MP 86% |

The receiver demaps **TRN2d 4572T to 5923 ones and finds an MP frame**,
logging `MP/Ed`, the furthest the analogue role has got on a real line.

**And live, v90-dfe4 (this build):** on attempts 1 and 2 of 3, the Apple
side logs `MP/Ed (TRN2d 4014T/19997 ones, MP 1 frames, 0 demap failures)`.
That is TRN2d demapped and an MP frame received live, while the server
accepted our CP and went on to B1d.  Both attempts then reached the §9.4.2
deadline before B1d decoded, and retrained.

**Next blocker: decision-feedback error propagation.**  The trained feedback
filter's dominant tap is b2 = 1.93 (|root| ~ 1.39, which is the x1.35/symbol
growth seen in the DIL), at every feedback length tried.  The null leaves a
lag-2 postcursor twice the main cursor.  So one wrong decision perturbs symbol
k+2 by about twice the decision distance and forces the next one wrong: no DFE
on this channel survives a single error.  Attempt 1 of the same replay fails
at TRN2d symbol 12, on a sign decision between Ucodes ±5 (0.022 apart).  The
remedies are transmitter-side shaping of the 4000 Hz energy (§5.4.5 spectral
shaping, requested in CP) or a reduced-state sequence detector over the
postcursor.  Excluding the smallest Ucodes from CP would make the first error
rarer, but not survivable.

### Phase 4 to data mode: a 1 - D^2 detector instead of decision feedback (2026-10-15)

The previous section ended on decision feedback that cascades on one error
(b2 = 1.93).  Three findings and a different receiver.

**The cascade is the channel, not a bad decision delay.**  Every decision delay
gives an unstable feedback filter (b max 1.4-2.3, all unstable); the best SNRs
have the largest lag-2 tap.  Fitting the line's T/2 response from TRN1d against
the server's tap (residual sigma 11.9, the floor) shows why.  It is **flat from
300 to 3400 Hz with a null at DC (-33 dB, -9 dB at 100 Hz) and another at
4000 Hz (-61 dB)**.  Feedback has to regenerate both missing bands from past
decisions.

**§5.4.5 spectral shaping is not enough on its own.**  The shaper (implemented
on our digital side in v90.c and validated live against SmartLink) can be
pointed at both nulls by requesting a1 = -1, a2 = +1 in CP.  The analogue side
never set these; with all four zero, F = 1 and the metric Σx² does not depend
on the signs.  Simulated through the measured channel with the server's real
TRN2d constellations: at Sr = 3 the 4000 Hz share falls from 1.5% to 0.06%,
but a forward-only equaliser only rises from 11.8 to 20.8 dB, and a DFE still
cascades.

**But "flat with nulls at DC and Nyquist" is partial response class IV, 1 - D².**
Equalise to z[k] = s[k] - s[k-2] instead of to the symbols, and no null has to
be inverted.  Forward-only taps reach 36.5 dB at 64 taps and 41.9 dB at 128,
against 12-16 dB to the symbols, 1 - D or 1 + D.  Then detect with a Viterbi
over the known memory instead of feeding decisions back.  1 - D² is two
interleaved 1 - D channels, each a trellis whose state is the previous symbol
on that interleave.  Offline, trained on TRN1d, run over 8000 symbols of
TRN2d/MP: **zero symbol errors** at a traceback of 16 per interleave (25
errors at 8: a 1 - D trellis has near-invisible constant-offset error events).

**`v90_analogue_pr4.c`** (new; `v90_analogue_pr4_test`): a 64-tap T/2 equaliser
to 1 - D², trained by least squares on 5000 symbols of TRN1d against the main
path's decisions.  It picks the best decision delay (one Cholesky, a solve per
delay), then runs a two-interleave Viterbi with traceback 24.  The engine uses
it from Phase 4's first symbol; Phase 3 and the DIL are untouched.  The engine
asks for each symbol's candidates through `v90_analogue_phase4_ucodes_ahead()`:
U_INFO in Ri, the R̄i count then CPt, CPt ∪ CP in MP, CP from B1d.  Two
details carried the rest:

- The detector decides ~60 symbols after the Phase 4 receiver would need to
  know them.  So on every receiver stage change it **re-decides its whole
  undecided window** with fresh sets, from the last decided symbol as a known
  state (`v90a_pr4_resync`).  Before this, TRN2d's first ~45 symbols were
  decided against "Ri or anything".
- In MP the set must include CP.  Ed (CPt, §8.6.2) is followed 12 symbols
  later by B1d (CP, §8.6.1), which the detector reaches before the receiver
  sees Ed.  With CPt only, B1d's CP-only points had no candidate.  The 1 - D²
  path through them corrupted Ed behind them, so Ed was never recognised and CP
  never came in: a deadlock (v90-dfe4 attempt 1, replayed).

**Replayed (v90-dfe4 with its live CP):** both attempts go TRN2d → MP → Ed →
**B1d 48/48 frames with 0 bit errors → data**.  Codewords are 100% exact
through MP, Ed and B1d on attempt 2.

**Live (v90-pr4-1, this build):** attempt 2 reaches **data mode on both ends**.
- The Apple side logs `B1d 48/48 frames with 0 bit errors` and `data`.
- The server logs `startup complete (upstream V.34 31200 bps, downstream PCM
  40000 bps)` and decodes upstream data (idle ones).
- Attempt 3 also reached B1d.  Attempt 1 hit the deadline after a poor TRN2d
  (10 ones).

v90-pr4-2 repeated it: `B1d 48/48 frames with 0 bit errors` and `data` on the
Apple side, while the server reported `startup complete (upstream V.34 31200
bps, downstream PCM 41333 bps)`.  `ME_V90_ANALOGUE_UPSTREAM_MAX_N=10` was set
to try a 24000 upstream but did not take effect: the server still negotiated
31200, so that call says nothing about a lower upstream rate.

**What ended both calls is the upstream.**  The server's V.90 upstream receiver at
31200 ran ~35% symbol errors, lost frame phase and then carrier.  It tried a
§9.6 rate renegotiation, got no answer from us, and retrained.  V.42 never
formed: the Apple side fell back to V.14, because our LAPM frames did not
survive the upstream.  That is the upstream workstream (see the V.90 upstream
notes), and the analogue side does not yet answer a §9.6 Rd while in data mode.

### The upstream, §9.6 renegotiation, and V.42 LAPM held (2026-10-15)

The previous section ended with data mode live but the call dying on the
upstream, an unanswered §9.6 Rd, and no LAPM.  Five findings, four of them
ours; recordings `reference-data/artifacts/apple-pve-20261015/v90-rr-{1..5}`.

**Why the upstream is poor: the line's response is longer than the server's
equaliser.**  Fitted from what we sent (`call/tx-tap.s16`, resampled 9600 →
8000 and drift-corrected per window) to what the server received
(`server/live-rx.g711`), v90-pr4-1's two data stretches:

- Our transmitter is clean: −20.4 dBFS RMS, peak −10.7 dBFS, no clipping.  It
  arrives at −21.8 dBm0.  The idle line at the server is RMS ~6, 35 dB down.
- Echo of the server's own downstream in the upstream is −38 dB.  There is no
  echo problem on the digital side.
- The wire is good *if the receiver spans it*.  A linear equaliser from the
  received stream back to our transmitted waveform reaches, in band:

  | span | 1 ms | 2.2 ms | 4 ms | 6 ms | 9 ms | 12 ms | 18 ms |
  |---|---|---|---|---|---|---|---|
  | SNR | 15.8 | 18.0 | 23.0 | 25.8 | 27.8 | 29.0 | 30.3 dB |

  The server's V.90 upstream FSE is `V34_V90_T3_FSE_TAPS` = 21 taps at T/3:
  **7 symbols, 2.2 ms, the 18 dB column**.  31200 needs ~30 dB, which matches the
  observed ~35% symbol errors.  The loop's DC null (the same one the 1 − D²
  detector exists for downstream) gives a long tail that 7 symbols cannot
  cover.  MP is Type 0, so the analogue side is not asked to precode either.
  Lengthening the FSE in `v90_upstream_replay` moved B1's out-of-sample
  distance 0.551 → 0.469 → 0.458 (21/41/63 taps), but whole-call replay error
  stayed near white in every arm (replay does not reproduce the live upstream
  here: 55-60% against 35% live).  So that change is untested and was not made.

**`ME_V90_ANALOGUE_UPSTREAM_MAX_N` did take effect; our server ignored it.**
v90-pr4-2's CP carried mask 0x01FF and the server's MP answered max drn 10
(24000), mask 0x01FF.  The analogue side transmitted at 24000 (`enter DATA ...
upstream=24000`).  But the server prepared its receiver at
`v34_get_current_bit_rate()` = 31200, the rate V.34 trained at, ignoring the
rate its own MP announced.  §9.4.2.4 has the analogue modem transmit at the
highest rate enabled in both, at or below MP bits 24:27.  That call was a rate
mismatch, not a measurement of 24000.  Fixed: the receiver is prepared at
`v90_mp_upstream_max_bps()` when that is lower (`CP/MP mask limits the
upstream 31200 -> 26400 bps`).

**The analogue side never saw R̄d.**  It acquired Rd, but in the RR stages the
1 − D² detector's candidates were the union of U_INFO, CPt and CP.  It decided
every Rd frame as ±78 (U_INFO) against a true ±68/67.  `r_frame_polarity()`
needs the exact highest Ucode, so no frame counted, R̄d never registered, and
the server's §9.6.1 timeout retrained.  Now `ucodes_ahead()` offers what §8.6.4
says can arrive: Rd/R̄d on CP's highest point per slot, Rt/R̄t on CPt's, then
Ucode-0 silence before Rt.

**Then it never saw MP: wrong constellation for renegotiation training.**  The
analogue side mapped the renegotiation's TRN2d/MP/Ed on CP's points.  §8.6
carries the data mode's *spectral shaping* into a renegotiation, "along with
the K previously derived from CPt", and §8.6.5 passes TRN2d's constellation in
CPt.  The server maps on CPt, correctly.  A merged frame (`rr_training`: CPt's
constellations, CP's Sr/ld/a1..b2, D = K(CPt) + 6 − Sr(CP)) fixed it.  In the
v90-rr-1 replay, MP decodes and the analogue side sends CP' and E/B1.

**Then Ed: the downstream equaliser drifts.**  The 1 − D² taps were fitted on
TRN1d and then held.  Aligned against the server's transmit tap (sign agreement
≥ 99.5%), codeword error in v90-rr-3 rose from 0-1% at B1d to 5-9% twenty
seconds later, and 27% further on.  A renegotiation's two-frame Ed then has
about even odds.  `v90_analogue_pr4.c` now tracks: decision-directed NLMS
against s[k] − s[k−2] from the Viterbi's traced-back decisions, only on steps
with ≥ 8 candidates.  Adapting on Ri's single-level square wave took the
trained 41 dB to 24 within a second.  At `ME_V90A_PR4_MU` 1, 2 or 5
(thousandths; default 2) v90-rr-3 replays at 0-0.4% to the end of the call, and
the renegotiation completes: B1d 48/48, 0 bit errors, then data.

**The server now learns the upstream rate.**  Before a renegotiation for
upstream loss, if the receiver settled at ≥ 0.10 decision error (healthy
upstreams settle near 0.025; this rig: 0.20-0.31 at 31200), the renegotiated
MP offers one rate lower, or two at ≥ 0.20.  The cap persists for the call,
including retrains (`g_v90_upstream_learnt_cap`; `ME_V90_UPSTREAM_STEP=0`
disables it).  Live, 31200 settled at 0.200-0.262 and 26400 at 0.071-0.112.

**LAPM: T400 runs out before our server can hear the ODP.**  The server's
upstream delivers its first data bit ~1.1 s after data entry (B1 acquisition
from the capture ring: v90-rr-4, +32.90 → +34.04 s).  The analogue side's V.42
T400 is 750 ms (V.42 §9.1.1) from its own data entry, so it fell back to V.14
before the server, which did detect the ODP, could answer.  With
`ME_V42_T400_MS=3000` on the analogue side, LAPM forms.

**Live (Apple ATA 6005 → CT 117 6002, this build):**

| call | renegotiation, analogue side | upstream | LAPM |
|---|---|---|---|
| v90-rr-1 (Rd fix only) | S/S̄/CP sent, MP never decoded → Ed deadline | 31200 → 26400 offered | no |
| v90-rr-2 | not reached: Phase 4 deadlines (22-ones TRN2d) | — | — |
| v90-rr-3 (+ CPt training) | MP decoded, Ed missed (5-9% codeword error) | 31200 (0.200) → 26400 (0.071) | no (T400) |
| v90-rr-4 (+ tracking) | **completes: B1d 48/48 → data** | 31200 (0.242) → 26400 (0.109) | no (T400) |
| v90-rr-5 (+ T400 3 s, 90 s hold) | **completes** | 31200 (0.262) → 26400 (0.112) | **connected at +28 s, held to hang-up** |

v90-rr-5 is the first call on this rig where V.42 LAPM (with V.42bis
negotiated) came up and held.  Data held for the rest of the call through a
completed renegotiation.  An upstream discontinuity 1 s before the 90 s
hang-up started renegotiation 2, offering 24000.

**Still open:**
- The server's T/3 FSE span on this loop.  It is the reason 31200 (and
  marginally 26400) decodes poorly; a longer FSE or precoding (MP Type 1) is
  the real fix.
- The server's 1.1 s upstream acquisition latency.  Until it falls below T400
  the analogue side needs `ME_V42_T400_MS`; the default stays at V.42's 750 ms.
- `--rx-replay` segfaults at V.14 fallback without `--pty-link`; pass one.
- Startup's Phase 4 is still a coin flip on some attempts (v90-rr-2).

### The upstream floor is noise, not equaliser span (2026-10-15)

The previous section blamed the server's 21-tap T/3 FSE.  Measured at the
symbol level, that is wrong.  Truth symbols come from demodulating our own
`call/tx-tap.s16`.  It runs on the Apple codec clock, −164 ppm against the
server, so it has to be resampled *and* carrier-corrected onto the server's
timebase first.  These symbols were fitted to `server/live-rx.g711` through
the same front end as `v34rx.c` (6/5, analytic, 1828.57 Hz).  v90-rr-5,
26400, 50-56 s:

| T/3 taps (span) | 21 (2.2 ms) | 41 (4.3) | 63 (6.6) | 99 (10.3) |
|---|---|---|---|---|
| LS from B1-length 128 symbols, out of sample | 0.223 | 0.205 | 0.214 | 0.275 |
| RLS tracking (λ 0.9995) | 0.21 | 0.175 | 0.171 | 0.165 |

Figures are distance to the lattice, the server's own metric.  Span is worth
≈1 dB at most.  The residual is white: a 3-tap noise predictor gains 0.3 dB,
so V.34 precoding (MP Type 1, §9.6) would not help either.  The waveform bound
(18 dB at 2.2 ms) was measuring the RRC band edges, which carry no symbol
information.

The waveform fit (15 ms span) reaches 31.4 dB, with a flat residual of
32-33 dB across 300-3300 Hz.  Echo of the server's downstream is −53 dB.  The
floor has two parts:
- μ-law quantisation, 36.9 dB.  It is fixed whatever the level.
- Analogue-side noise, 32.8 dB at the received −21.6 dBm0.  The analogue
  engine transmits at the tree's −14 dBm0 nominal (`ME_V34_TX_DBM0`), and
  V.90 §3.4 leaves that level to the user.

**Live, up-lvl9-1 (analogue side `ME_V34_TX_DBM0=-9`, server unchanged):**
- The upstream arrives 5.0 dB hotter (RMS 330 → 588).
- The waveform SNR rises 31.4 → 34.5 dB.  The analogue-noise part went
  32.8 → 37.7 dB, so that noise is additive.
- At 26400 the server settled at 0.078, and its running error over the whole
  stretch fell 0.128 → 0.093.  At 31200 it settled at 0.238, still unusable.
- LAPM connected.  A peer retrain followed at +75 s (not investigated).

μ-law (37 dB) is now the binding floor.  At most ~2.5 dB more is available
from level (−6 dBm0, peaks then near DAC full scale).  At 34.5 dB the server
sits ~1.7 dB above the symbol bound, which is the only place a longer FSE
could still pay.  Below 0.05 at 26400 needs both the level and that
receiver gap closed; 31200 is not reachable over μ-law on this loop.
Scripts: the session scratchpad's `symbound.py`, `adapt.py`, `resid.py`
and `echo.py`; they have not been moved into `tools/`.

## Passive diagnostic visibility (2026-10-16)

The Apple GUI now distinguishes constant rail, digital silence, constant input
and varying audio directly from its bounded raw sample window. Both GUIs show
AC RMS in dBFS and min/max alongside DC and cumulative near-rail counts;
telemetry also includes window zero/rail counts and total samples. Engine-only
TX underrun runs and samples are now visible separately from idle empty TX.
These observations use the existing passive taps (V.90 §5 analogue role); they
do not alter transport or training (V.34 §10.1). Synthetic selftests cover
rail, zero and constant-DC input across history wraparound. See `docs/gui_mode.md`
for units and limitations. Hardware interop remains untested by these checks.

## Caller ID and electrical status recovered and tested (2026-10-16)

Both modems were connected but at USB configuration 0. USB discovery inside
Codex's sandbox returned an empty list; outside the sandbox both were present.
Configuring each exposed both CoreAudio devices. Current mapping, established
by live incoming calls (USB addresses are valid only until replug):

| USB | CoreAudio UID substring | lab extension | on-hook raw sense |
|---|---|---|---|
| `0:13` | `145430` | 6004 | `0x32` (+50 signed) |
| `0:14` | `:000000:` | 6005 | `0x32` (+50 signed) |

### The original USB driver does have a digital-line guard

The Boot Camp `MotorolaSetup.exe` linked above was downloaded again and its
SHA-256 matched the recorded `f22dc405…4483`. Extracted x86 `USmSerial.sys`
has SHA-256 `5b79a2d94bca4a7e4de085602fc4b43a691b7f7d5b2ecc938461e8a632f03c12`.
Its bundled Readme documents the "Loop current too high" warning specifically
in this USB distribution. Decompilation supplies the actual decisions:

- `ReadReg` at `0x9ca6b` reads through the existing command channel.
- `0x9d15e` reads register `0x0c`: bit 6 must be set, bits 4:0 must be nonzero;
  code 31 reports `0x8002` and calls the on-hook routine. Other nonzero codes
  are accepted. An invalid link/zero current has a separate retry/failure path.
- The USB dispatcher also handles notification `a1 29`, `wValue=9` by calling
  on-hook and reporting `0x8002`.
- `0x1609c` maps `0x8002` to result `0x55`; the result table contains
  `LINE OVERLOAD DETECTED`.

Register `0x11=0x0c` also has the compatible DAA's OPE hardware-protection bit
(bit 3) set. The readout now exposes this enable bit. Si3056 §5.22 documents
the resulting OPD indication (register 19 bit 0), which also trips the coupler
guard. Receive overload (register 19 bit 2) alone does not. No OPE, country or
gain bits are rewritten by this change.

The coupler now checks this overload decision during its 500 ms seizure
settling interval and polls during active calls. It also recognizes the USB
notification even when a register read consumes it. On overload it clears the
hook bit, stops the call and returns failure. Hardware-free checks cover code
31, failed reads, invalid FDT, on-hook status and the notification framing.
No real overload was induced; these tests do not certify protection timing.

### Line voltage/current: useful estimates, not calibration

The compatible Silicon Labs DAA layout is documented in the manufacturer
[Si3056/Si3018/19/10 data sheet](https://www.skyworksinc.com/-/media/Skyworks/SL/documents/public/data-sheets/si3056.pdf),
registers 11, 12, 19 and 29, and in the
[Si3050/Si3011/18/19 data sheet](https://www.skyworksinc.com/-/media/SkyWorks/SL/documents/public/data-sheets/Si3050-11-18-19.pdf),
§5.8 and §5.12. The driver's use of the current/link fields corroborates that
part of the layout independently. Both Apple modems read revision register
`0x0b=0x14`, status `0x0c=0x40` on-hook, and auxiliary `0x13=0x02`.

Register `0x1d` is now exposed both raw and signed. The earlier VG224 values
`0xd8`–`0xda` are −40 to −38 signed, not a mysterious unsigned level near 217.
The current ATA reads +50 on-hook and +7 off-hook; these fit a 1 V/code LVS
interpretation, including polarity. **This is an inference, not a calibrated
Apple voltage measurement.** In particular the family data sheets restrict
some LVS functionality by line-side variant; a shared register layout and a
plausible reading do not establish the exact Apple silicon/BOM. Voltage is
explicitly labelled inferred/uncalibrated and cannot identify a digital line.
The 3.3 mA/code current scale is also an approximation from the compatible DAA
layout. Raw current code 31, rather than an estimated mA threshold, drives the
guard recovered from the driver.

`apple_usb_modem_probe --line-status` and both GUIs expose these readings,
current code, link validity, receive overload and off-hook dropout. An unreadable
register is not converted into a voltage or alarm. Register 19 bit 2 indicates
receive input overload, which is distinct from loop-current overload. Its bit
1 is normally set on-hook and is only presented as dropout while off-hook.

### Live CID works while remaining on-hook

Ring watching now starts silent-TX/on-hook RX **before** ringing and keeps RX
running between bursts. A private resampler feeds the vendored SpanDSP Bell
202 and V.23 ADSI receivers at 8 kHz; it changes neither raw samples nor the
engine resamplers. Checksum-valid SDMF/MDMF supplies number, name, private/
out-of-area reasons and received date/time. Structural bounds are checked
before parsing fields. V.253 §9.2.3.1.3 and Table 13 govern formatted AT reports;
V.23 §2 supplies the V.23 carrier pair. No DTMF or call-waiting CID is claimed.

An asynchronous data-interface entry point preserves S0's ring count: the old
`di_set_caller_id` calls `at_reset_call_info`, which resets rings as well as
fields. Pre-ring CID waits for the first RING; inter-ring CID reports when it
arrives with `AT+VCID=1`. Packet date/time replaces the previous synthesized
host timestamp. Decoder repeats are suppressed within a ring sequence.
The `decoder` label names the accepting receiver: Bell 202 and V.23 passbands
overlap, so this is not an independently measured modulation classification.

Bounded 12-second PBX calls, caller `6002 / Apple CID Test`, no answering:

| called extension | receiving modem | result | evidence |
|---|---|---|---|
| 6005 | `0:14` | checksum-valid number, name, date/time; AT fields after RING | `reference-data/artifacts/apple-cid-20261016/modem-1/` |
| 6004 | `0:13` | same; four RING indications, CID reported once | `reference-data/artifacts/apple-cid-20261016-r2/modem-0/` |

Both raw recordings decode again through `--cid-replay`. Neither test seized
its line. Both devices' manual/watch ownership and shutdown were also checked:
turning off the manual monitor does not stop a watch; stopping the final user
stops audio and clears the monitor bit.

**Answer transition, CLI:** `--answer --answer-rings 2 --ring-wait 20 --hold 1`
on `0:13` received `6002 / Apple Answer Test`, reported `RING 1/2`, decoded CID,
then reported `RING 2/2` and went off-hook. Active status was `0x49`, current
code 9 (approximately 29.7 mA), raw sense `0x07` (approximately +7 V). The
one-second engine run had zero render errors, zero TX underruns and exactly
135168 input/output device frames, including on-hook monitoring. It then
returned on-hook with sense `0x32`. Evidence:
`reference-data/artifacts/apple-cid-answer-20261016/`. This verifies the monitoring/answer
transition, not a complete modem connection.

The CLI's default remains first-ring answer; `--answer-rings 2` (or GUI S0=2)
leaves time for inter-ring CID. The CLI wait timeout now uses elapsed monotonic
time, rather than treating each USB completion as half a second. The latter
would time out early under frequent ring notifications.

`make gui-apple-test` runs `tests/apple.list`: bounded raw-tap/status checks,
independent FSK waveforms at 8000/9600 Hz for both carrier pairs and both
message formats, privacy, bad checksums, malformed TLVs and DC silence, plus
the existing GUI lifecycle checks. `console_test` covers received date/time,
pre-ring/inter-ring delivery, disabled reporting, no fabricated CID and S0/S1
ring-count preservation. Logs use the normal isolated test runner.


## Timestamped events and internal loopback (2026-10-16)

The coupler now records ring/CID, hook requests and readbacks, voltage/current
changes, polarity changes, line flags and protection, overload trips, monitor
ownership, engine phase/signal changes, render errors and TX gaps as structured
events. Each has UTC milliseconds, monotonic time, process ID, sequence and
elapsed milliseconds. `--line-events FILE` / `APPLE_MODEM_LINE_EVENTS` appends
JSONL; normal GUI launches choose a persistent file automatically. Rotation
keeps a 2 MiB current file and one backup. Memory retains 128 events and GUI
telemetry carries the last 16. CID events contain the received caller fields.

The GUI **Hardware self-test** and CLI `--loopback-test digital` exercise
Si3056 §5.30 digital loopback (DDL, register 10 bit 0), on-hook. They pause
monitoring, save and verify the affected registers, send a known integer sample
pattern, compare returned samples, restore and verify registers, and resume
monitoring. GUI raw recording must be stopped first. This checks the host USB /
digital sample path; it does not establish analogue line performance or modem
interoperability (V.90 §5). The former interpretation of register 0x0a as a
mute was incomplete: DDL returns the transmitted pattern, including silence.

Initial immediate-start tests had 2–16 mismatching samples, with some data
repeated from 250 ms earlier during audio startup. The test now allows one
second of silence before the pattern and compares a full steady-state second;
it still requires exact equality, rather than relaxing an error threshold.
Both `0:13` and `0:14` passed at 9600 Hz: 9600 samples, zero sample/bit errors,
correlation 1, gain 1 and RMSE 0. Observed round-trip delay was about 115–121 ms.
The 8000 Hz test on `0:13` also passed exactly over 8000 samples (145.5 ms).
All tests verified register restoration. Evidence is under
`reference-data/artifacts/apple-loopback-20261016/`, including a real GUI action which resumed
ring watching afterwards. The optional `startup` mode is unsupported on this
firmware: register 6's PDL bit did not latch; the test stopped and restored the
registers without claiming a pass.

### Pulse dialling

The compatible DAA supports host-controlled pulse dialling at nominal 10 pulses
per second (Si3056 §5.17), using hook switching and the prescribed calibration
handling. At the initial diagnostic commit the coupler generated only DTMF, even though
the parser accepted P/T modifiers. The implementation below supersedes that
limitation; the attached exchange/ATA must also accept pulses.

Manufacturer reference: [Si3056 data sheet, §§5.17 and 5.30](https://www.skyworksinc.com/-/media/Skyworks/SL/documents/public/data-sheets/si3056.pdf).

A bounded incoming call to `6004` with caller `6002 / Apple Event Test` verified
both ring-burst events, checksum-valid CID before second-ring answer, hook
readbacks, voltage/current transitions and engine startup. The call ran the
engine for one second and returned on-hook; no overload event occurred.
Evidence and a reproduction script are in
`reference-data/artifacts/apple-events-answer-20261016/`. This remains a monitoring/answer
transition check rather than a trained modem connection.


## Physical pulse dialling and PCM loopback measurements (2026-10-16)

### Physical dial plans

`ATDP6005`, `ATP` followed by `ATD6005`, stored numbers and `ATDL` now preserve
the initial pulse/tone default and every in-string P/T switch. An optional
physical-plan callback leaves the SIP callback's normalized number unchanged.
Unsupported physical modifiers are rejected before seizure rather than silently
removed. The Apple backend validates the complete plan, then performs USB hook
writes on its control thread, never an audio callback. CLI equivalent:
`--dial P6005`; an initial T (or no modifier) selects the normal DTMF path.
Mixed pulse/tone plans are also executed in order. Pulse-only transmission is
silent; mixed plans render DTMF only during their tone steps.

V.250 §§6.3.1.3–4 and 6.3.3–4 define the mode selection. The host schedule uses
10 pps, 60 ms break / 40 ms make, 600 ms additional interdigit pause; zero is
10 pulses. Si3056 §5.17 requires RCALD during makes. ADC calibration is held
and the 8 ms off-hook counter is selected (§§5.11/5.21), then all three settings
are restored and checked on success, error, or ATH/SIGINT. Configuration
readbacks exclude read-only/sticky status bits. A late schedule aborts the
attempt instead of stretching subsequent pulses.

An important correction to seizure control came from live testing: §5.22
requires OPE to be cleared **before** an off-hook transition, then re-armed
25 ms afterwards. Leaving it continuously enabled produced OPD during normal
make transients. Normal seizure and pulse makes now follow this sequence;
current saturation and USB overload notifications still stop the attempt.
The early failed attempts remain in `reference-data/artifacts/apple-pulse-dial-20261016/`.

`reference-data/artifacts/apple-pulse-dial-20261016-r4/` records a complete CLI `P6005` attempt:
31 pulses / 62 hook edges, observed break intervals 58–61 ms, intra-digit make
intervals 39–42 ms, maximum scheduler lateness 1 ms, configuration restored.
The destination did not ring. This verifies host-command timing, **not** physical
loop-current pulse shape or exchange acceptance. The HT802 has a separate
Enable Pulse Dialing option, default No, and a pulse-standard selector; the
current ATA setting has not been inspected or changed. Manufacturer reference:
[HT80x administration guide](https://documentation.grandstream.com/knowledge-base/ht80x-administration-guide/).

`reference-data/artifacts/apple-atdp-20261016/` verifies the real split PTY's `ATDP6005`
dispatch: the complete attempt produced 62 edges and restored settings; an ATH
after the first edge stopped the attempt and restored settings. Both returned
on-hook. `console_test` covers default ATP, mixed P/T, stored expansion, redial,
backend rejection and unchanged legacy callback numbers. The diagnostic
self-test covers zero's ten pulses and invalid physical dial plans.

### What survives the hardware loopback

The CLI now accepts `--loopback-test digital|startup|isolation|analog` with
`--loopback-pattern prbs|ulaw|alaw`. G.711 tests require `--rate 8000`: the
pattern is independent G.711 §3 decoded levels at 8000 symbols/s, including
both signs and the full alphabet. Mu-law's two zero codes represent one level,
so it has 255 distinct linear levels; A-law has 256. This is an uncoded full
alphabet PCM probe, **not** a V.90 negotiation or an achievable-rate claim.

The GUI button remains an on-hook digital test. CLI isolation/analog tests
briefly seize the line, run the test, release the line, and restore the saved
registers. Si3056 §5.30 defines IDL (register 1 bit 1) and analog HBE-off
(register 2 bit 1); the latter needs 12 dB TX headroom. Both line-side modes
use quarter-amplitude samples to leave headroom. A correlated line-side
return only establishes the path: its raw sample bit differences are not a
modulation BER. The separate report performs symbol decisions.

`tools/apple_pcm_loopback_report.py` trains a fixed 65-tap linear equalizer
on 4096 samples and scores 6544 separate held-out samples. No held-out targets
fit gain, DC or equalizer taps. It reports nearest G.711 level decisions,
canonicalizing mu-law zero, residual RMS/SNR and codeword bit errors. Synthetic
checks recover a delayed half-gain signal with a DC offset exactly and reject
an independent-noise control. It also reports candidate transport delays from
three-sample pattern matches, useful for spotting packet mixing.

| path | modem 0:13 | modem 0:14 | evidence |
|---|---|---|---|
| digital, A-law and mu-law | 8000/8000 exact samples in each run | same on repeat | `reference-data/artifacts/apple-pcm-loopback-20261016-r2/` |
| isolation, mu-law, quarter TX | 21.48 dB held-out SNR; 88.13% symbol errors | 21.48 dB; 88.33% | same |
| analog, quiet call, quarter TX | 16.59 dB; 92.33% symbol errors | 16.66 dB; 92.25% | `reference-data/artifacts/apple-pcm-loopback-20261016-quiet/` |

The isolation return includes about 3.3–3.5k counts of DC and filter ISI; even
after this equalizer it does not preserve arbitrary full-alphabet 64 kbit/s
PCM. The analog result includes the actual line-side path. Initial analog
attempts simply seized the FXS port, so its loud dial tone swamped the probe;
those are not codec-fidelity measurements. The quiet runs used bounded PBX
Wait calls from 6002 to 6004/6005, answered by the loopback test. All tests
restored registers and returned on-hook. These equalizer-specific error rates
are not upper bounds on what a trained receiver or reduced constellation can
recover; V.90 §5 uses a different end-to-end path and negotiated constellations.

One initial **digital A-law** run on `0:14` was a real failure: 2992/8000 sample
errors. Its error pattern repeats every eight samples: five samples have lag
1113, three have lag 1193 (an extra 80 samples / 10 ms). This is packet/sample
mixing rather than a scalar gain or law conversion. The raw taps are retained
under `reference-data/artifacts/apple-pcm-loopback-20261016/0-14-digital-alaw/`; all four digital
law/device combinations passed on the repeat. The intermittent fault remains
unresolved; an exact self-test failure must not be hidden by an equalizer or
by dropping/splicing samples.


## V.34 through the real loopback (2026-10-16)

`tools/apple_v34_loopback.py` now joins two independent vendored V.34 instances
through a live Apple waveform leg. It runs full V.34 §§10.1/11 startup,
negotiation, training and independent PRBS payload comparison, not seeded
mapper symbols. The harness adds no shared INFO/MP, equalizer or bit state.
The selected direction crosses CoreAudio TX → DAA loopback → CoreAudio RX;
the opposite direction remains the ordinary software G.711 bearer. This is a
one-leg hardware experiment, not two-line/foreign-modem interoperability.

The inherited socket carries only 8000 Hz s16le samples. A fixed 1536-sample
silent prefix reserves HAL burst headroom and is explicit channel delay.
Captured device samples pace both software endpoints. Atomic publication
keeps complete samples visible across callback/control threads, even if the
socket splits a sample's bytes. The audio callbacks do not wait on sockets.
The host fails on a TX gap instead of repeating/dropping samples to repair it;
stream buffers and runtime are bounded. Line-side modes retain the quarter
TX scale and full register/hook restoration. Analog uses a quiet incoming PBX
Wait call so the ATA's dial tone does not swamp the waveform.

Initial results, caller-to-answer through hardware unless marked reverse:

| device/path | baud / bit/s | recovered bits caller / answer | errors | result |
|---|---|---|---|---|
| 0:13 digital | 2400 / 9600 | 18240 / 16032 | 0 / 0 | pass |
| 0:14 digital | 2400 / 19200 | 20736 / 16320 | 0 / 0 | pass |
| 0:13 isolation | 2400 / 9600 | 18240 / 16128 | 0 / 0 | pass |
| 0:14 isolation | 2400 / 9600 | 18432 / 16160 | 0 / 0 | pass |
| 0:14 isolation | 2400 / 19200 | 970752 / 0 | 0 / unscorable | fail, 60 s |
| same, repeat | 2400 / 19200 | 971136 / 0 | 0 / unscorable | fail, 60 s |
| 0:13 analog | 2400 / 19200 | 20736 / 16256 | 0 / 0 | pass |
| 0:14 analog, reverse | 2400 / 19200 | 16192 / 18112 | 0 / 0 | pass |
| 0:13 analog | 3200 / 28800 | 26568 / 16200 | 0 / 0 | pass |
| 0:13 analog | 3429 / 31200 | 27227 / 16088 | 0 / 0 | pass |
| 0:13 analog | 3429 / 33600 | 28772 / 16464 | 0 / 158 | fail |

Both endpoints reported training in every row, but training alone is
insufficient: the two isolation failures never found the receiving payload
sequence. Their zero error counters must **not** be read as zero BER. At
33600 the hardware leg synchronized but had 158 bit errors (about 0.96% of
reported recovered bits). The highest clean analog row observed here is
31200. These short single-run passes are not long-duration reliability results
or a proven capacity ceiling. The isolation failure remains unexplained.

All rows had zero host stream gap samples, restored registers and returned
on-hook. Software controls passed at each tested rate; the 9600 control also
passed with 1536 samples of delay in both directions. Raw taps, line events,
modem logs, JSON summaries and control logs live in
`reference-data/artifacts/apple-v34-loopback-20261016/`. The failed rows are retained beside
the passes. This demonstrates why the preceding full-alphabet 64 kbit/s PCM
probe does not establish V.34 performance: a trained bandlimited modem can
recover data through that same analog path.

Reproduction (use a fresh output directory):

```sh
make apple_usb_modem_coupler v34_duplex_test
python3 tools/apple_v34_loopback.py --address 0:13 --uid 145430 \
  --mode digital --out artifacts/v34-digital-check
python3 tools/apple_v34_loopback.py --address 0:13 --uid 145430 \
  --mode analog --baud 3429 --bps 31200 \
  --quiet-pbx root@asterisk.net.cryan.nz --extension 6004 \
  --out artifacts/v34-analog-check
```

`--direction answer` routes the answerer's waveform through hardware instead.
The wrapper passes only when the peer succeeds, both receivers recover at
least 16000 bits with zero errors, transport succeeds, and hook/restoration
checks pass. Stream transport completion alone is not a modem pass.
`make gui-apple-test` now includes hardware-free fragmented socket exchanges
in both directions, a deliberately truncated channel which must fail, and
invalid descriptor rejection before opening USB.

## Separating loopback impairments (2026-10-16)

`tools/apple_loopback_spectrum.py` compares the saved full-alphabet probes
above, with Welch spectra, transfer response/coherence, and independent
held-out forward/inverse models. It does not change the live audio path.
Si3056 §5.30 defines the loopback boundaries; these measurements therefore
characterize those local paths, not an end-to-end SIP line or a foreign modem.
The isolation and analog probes use the same quarter-scale transmitted
samples. Digital uses full scale; the gain plots divide by the actual saved
TX samples, while the absolute power plots retain that level difference.

Measurements on the two devices (0:13 / 0:14):

| diagnostic | isolation | analog, quiet call |
|---|---|---|
| gain at 1 kHz | -1.02 / -1.02 dB | -1.72 / -1.49 dB |
| gain at 3.4 kHz | -2.99 / -2.99 dB | -3.64 / -3.42 dB |
| gain at 3.8 kHz | -19.60 / -19.60 dB | -20.27 / -20.03 dB |
| gain at 3.9 kHz | -27.93 / -27.91 dB | -29.00 / -28.76 dB |
| median coherence, 300–3400 Hz | 0.999999 / 0.999999 | 0.999995 / 0.999991 |
| 257-tap forward prediction SNR | 71.94 / 71.89 dB | 34.32 / 33.75 dB |
| 257-tap inverse reconstruction SNR | 51.67 / 51.24 dB | 21.81 / 21.92 dB |
| inverse plus 8 oracle feedback taps | 70.39 / 70.34 dB | 57.13 / 50.68 dB |

The shared sharp upper-band roll-off is already present in isolation mode.
Adding the analog path changes the 1 kHz gain by only 0.5–0.7 dB and retains
almost the same response shape. This points to filtering inside the common
loopback path as the principal full-band PCM impairment. Inversion amplifies
errors where that filter has removed signal; the inverse score must not be
called the additive line SNR. The much better oracle scores demonstrate
recoverable intersymbol interference under ideal previous decisions, but
are neither measured modem BER nor an achievable-rate prediction. The
oracle receives true previous symbols and has no decision error propagation.
In particular these numbers do not establish the cause of the isolation
19200 failure or the analog 33600 bit errors documented above.

Quadratic/cubic forward models do not improve held-out scores: isolation
71.53 / 71.48 dB and analog 34.23 / 33.70 dB. There is no evidence here that
this particular static polynomial distortion model explains the residual.
That does not exclude clipping at higher drive, dynamic nonlinearity, clock
drift, narrowband interference or time-varying effects. A level sweep and
longer independent captures are needed to separate them. Current captures
also cannot estimate remote echo or the relative clocks of two devices:
each local loopback shares its own audio clock.

Both digital controls have unity gain/coherence and exact raw sample returns.
Their fitted ~139 dB score is numerical regularization error, not a hardware
noise-floor measurement. In contrast, the retained packet fault has median
coherence 0.516 and only 3.10 dB forward prediction despite a superficially
flat response. Treat that as transport corruption, not a telephone filter.

The models use 6000 training samples, a 500-sample gap and 4944 held-out
samples; the centered input windows span ±128 samples. Neither the training
nor target windows overlap across the gap. The tool exports measured FIR
coefficients and a residual RMS for diagnostic replay. A white-noise replay
using that RMS is only an approximation: it does not establish that the
residual is white, independent, Gaussian or stationary.

Reproduction requires NumPy, SciPy and Matplotlib:

```sh
MPLCONFIGDIR=/tmp/apple-mpl .venv/bin/python tools/apple_loopback_spectrum.py \
  --self-check --out artifacts/apple-loopback-spectrum-check
```

The synthetic controls check a known FIR with DC and independent noise,
held-out inverse/ideal feedback fits, and an unrelated-signal negative
control. The retained plots, model files and JSON measurements are under
`reference-data/artifacts/apple-loopback-spectrum-20261016-verified/`.

### Independent quiet noise and response shape (2026-10-16)

`tools/apple_line_characterize.py` now measures quiet noise independently of
any equalizer, in two RX windows (2000–7000 and 22000–23500). It checks the
aligned TX is zero throughout both windows and an extra ±256 samples around
each. The post-probe window begins more than 800 samples after the received
probe ends. Welch power is integrated over nominal 300–3400 Hz, with 7.8125 Hz
bins. Each segment has its mean removed; these are AC, in-band measurements.

| quantity | analog 0:13 | analog 0:14 |
|---|---|---|
| quiet in-band RMS, before / after | 1.80 / 1.87 counts | 4.02 / 4.03 counts |
| active in-band RMS | 1779 counts | 1826 counts |
| quiet-referenced in-band SNR | **59.73 dB** | **53.13 dB** |
| 513-tap forward active residual, in-band RMS | 2.17 counts | 4.63 counts |
| active in-band signal / residual | **58.28 dB** | **51.92 dB** |

The independent quiet measurement and active residual agree within 1.5 dB.
This supports roughly 60 / 53 dB additive in-band SNR at this probe level on
these local analog loopbacks. It does not establish noise independence at
other transmit levels or the SNR of the remote SIP bearer. The active residual
includes model error, so it remains a signal-to-residual measurement.

**The earlier ~22 dB inverse-EQ score is not the line's noise SNR.** Increasing
the forward FIR from 65 to 257 to 513 taps improves analog broadband
prediction from about 24 to 34 to 43 dB. In-band, the 513-tap model reaches
58 / 52 dB. Much of the remaining broadband error is below the modem band;
short filters and a fixed DC estimate had included long-memory/low-frequency
response in what looked like noise. There is no need to assume an ideal DFE
to demonstrate the much lower in-band residual. Isolation controls show
73 / 77 dB quiet-referenced SNR and about 72 dB active residual SNR.

The training-only 513-tap FIR gives a response estimate without the Welch
transfer ratio's window smoothing. Train uses 5800 samples, test 4944; even
the longest input windows have a 188-sample gap. Response is normalized to
1 kHz. Both devices and both line-side modes agree closely:

| frequency | analog relative gain | ideal inverse magnitude boost |
|---|---|---|
| 1 kHz | 0 dB | 0 dB |
| 2 kHz | -0.37 dB | +0.37 dB |
| 3 kHz | -0.34 dB | +0.34 dB |
| 3.2 kHz | -0.16 dB | +0.16 dB |
| 3.4 kHz | -1.95 dB | +1.95 dB |
| 3.5 kHz | -4.21 dB | +4.21 dB |
| 3.6 kHz | -7.65 dB | +7.65 dB |
| 3.8 kHz | -18.7 dB | +18.7 dB |
| 3.9 kHz | -27.3 dB | +27.3 dB |

The analog -3/-6/-10/-20 dB crossings are approximately
3453/3558/3653/3817 Hz. Passband phase is close to linear: after removing a
best-fit delay over 300–3400 Hz, its 5th–95th percentile departure is roughly
-1.7 to +1.1 degrees. Delay relative to the chosen integer alignment is
61 microseconds in analog and 34 microseconds in isolation; this is not the
bulk USB latency.

Here "ideal response" means a flat magnitude and linear phase over the
chosen usable band. The CSV exports the measured complex-response summary
and inverse magnitude required to approach that target. Through 3.2 kHz it
needs little magnitude correction; 3.4 kHz needs about 2 dB. Attempting to
flatten the last few hundred Hz toward Nyquist instead amplifies noise by
19–27 dB. The inverse curve is a diagnostic, not a proposed live filter or
an optimum modulation spectrum. The local loopback only identifies the
combined TX/RX response; it cannot separate their individual filters.

Reproduce with NumPy/SciPy/Matplotlib:

```sh
MPLCONFIGDIR=/tmp/apple-mpl .venv/bin/python tools/apple_line_characterize.py \
  --self-check --out artifacts/apple-line-characterize-check
```

Synthetic checks verify integrated white-noise band power and the centered
FIR's complex-response convention. Results, response/inverse CSVs and the
visually inspected plot are in `reference-data/artifacts/apple-line-characterize-20261016/`.
No new hardware call or audio-path change was needed for this analysis.

### Native sample rate really extends the passband (2026-10-16)

The 3.45 kHz cutoff above is **an 8000 Hz mode result**, not the modem's
maximum analog bandwidth. New physical tests on both devices establish that
its common loopback filter moves with the native sample rate:

| native samples/s | analog -3 dB, 0:13 / 0:14 | analog -10 dB | analog -20 dB | gain at 4 kHz relative to 1 kHz |
|---|---|---|---|---|
| 8000 | 3452 / 3451 Hz | 3653 / 3653 Hz | 3817 / 3816 Hz | Nyquist; not reported |
| 9600 | 4176 / 4175 Hz | 4415 / 4414 Hz | 4610 / 4610 Hz | -0.68 / -0.69 dB |
| 10286 | 4442 / 4440 Hz | 4699 / 4698 Hz | 4911 / 4910 Hz | -0.10 / -0.11 dB |

Both isolation loops give essentially the same edge as their analog loops
(within about 5 Hz at -3 dB). The device is not restricted to a fixed
3.4 kHz passband. At 10286 Hz, 4.2 kHz is only about 0.4 dB down on 0:13;
4.4 kHz is 2.3 dB down. These are combined local TX/RX responses, not a
measurement of the remote ATA/SIP path. They do not establish a new CONNECT
rate or justify changing the engine's supported 8000/9600 Hz grids.

Implementation is deliberately confined to standalone loopback diagnostics:
Si3056 §5.30 probes now also accept `--rate 10286` with `--loopback-pattern
prbs`. G.711 alphabet probes and peer-stream tests still require 8000 Hz.
The PRBS is generated directly on the selected native grid; callbacks capture
and render it directly, with no engine resampler. Each run logs the HAL
nominal-rate readback and records `rate_hz` in its result. No training tables
or DSP constants changed.

`tools/apple_native_rate_probe.py` performs digital, isolation and quiet-call
analog tests on each modem at each rate. The uniform signed PRBS is lower
power than the previous full-alphabet G.711 probe: the analog 300–3400 Hz
quiet-referenced SNRs are about 53/52/52 dB on 0:13 and 47/46/45 dB on 0:14
at 8000/9600/10286 Hz respectively. These must not be compared directly with
the earlier 60/53 dB without accounting for the changed probe power. Native
rate changes the filter bandwidth as well as noise integration and signal
power in the fixed 300–3400 Hz band.

`tools/apple_native_rate_report.py` fits 513 taps using 0.6 seconds of
training, then independently scores a later interval starting at 0.75 seconds.
Input windows do not overlap between train and test. It validates the recorded
HAL readback, exports normalized response CSVs, plots the comparison and
excludes failed probes explicitly.

There were 21 attempts: 19 completed passes (18 distinct device/rate/mode
combinations plus one repeated digital control), and two retained failures:

* 0:13, 9600 Hz digital: 3772/9600 raw sample errors. Three-sample exact
  matches identify two delays, 1178 and 1098 samples, separated by 80 samples.
  Its repeat returned 9600/9600 exact. This is further evidence of the existing
  intermittent transport fault, not noise or a filter defect.
* 0:14, 10286 Hz isolation: test register readback failed before audio start;
  registers were restored. The repeat passed without relaxing any checks.

Every attempt restored registers and verified on-hook afterwards; completed
runs reported zero stream gaps. Both devices finished at 9600 Hz. All four
`make gui-apple-test` checks passed (the GUI check needed localhost socket
permission outside the sandbox); the direct diagnostic self-test also passed.

Evidence roots are `reference-data/artifacts/apple-native-rates-20261016/`,
`reference-data/artifacts/apple-native-rates-20261016-repeat13/`,
`reference-data/artifacts/apple-native-rates-20261016-device14/` and
`reference-data/artifacts/apple-native-rates-20261016-repeat14/`. The first contains the
combined summary and visually inspected `native-response.png`.

```sh
python3 tools/apple_native_rate_probe.py --quiet-pbx root@asterisk.net.cryan.nz \
  --out artifacts/apple-native-rate-check
# Include all evidence roots when a stopped sequence required a fresh repeat.
MPLCONFIGDIR=/tmp/apple-mpl .venv/bin/python tools/apple_native_rate_report.py \
  artifacts/apple-native-rate-check
```

## DAA identification: Si3018-compatible revision F (2026-10-16)

The complete register dumps are retained under
`reference-data/artifacts/apple-registers-20261016/`. Comparing only with the older Si3016
sheet gave misleading reserved-bit conflicts and a conditional domestic-only
interpretation. The newer manufacturer Si3056/Si3018/19/10 sheet, revision
1.06, resolves the principal identification fields:

* Register 11 (0x0b), page 56: LSID occupies bits 7:4; 0001 identifies Si3018.
  Both devices read 0x14: Si3018-compatible line-side ID, system-side revision
  code 4 (D under §5.32/Table 22). This identifies the interface revision,
  not conclusively the physical system-side part under Motorola's USB bridge.
* Register 13 (0x0d), page 57: REVB occupies bits 5:2; 0x18 gives code 6,
  revision F. Bit 6 is fixed zero, not the old domestic/international flag.
  The Si3018 is a global line-side device (§5.1.2). Withdraw the earlier
  conditional domestic-only interpretation.
* Register 1 bit 0 is SB (16-bit serial mode), register 2 bit 7 is INTE;
  registers 8/9 are PLL N/M dividers. Thus 0x01/0x83/0x02/0x0f are explained
  without interpreting reserved bits or an invalid sample-rate code.
* Register 14's 0x02 is ring polarity, not Si3016 safety-check-in-progress.
  Registers 22–24 are ring validation; 0x96/0x2d/0x19 match reset defaults.

Reference: https://www.skyworksinc.com/-/media/Skyworks/SL/documents/public/data-sheets/si3056.pdf
The Si3050 family is a weaker match: it specifies register 13 bit 6 fixed
one, versus our zero. The Si306x revision 0.9 manufacturer sheet (registers
11/13, pp.37–38) specifies different line IDs and bit 6 fixed one as well.

Read-only confirmation on both devices repeated the same ID/revision values.
Best supported identification is **Si3018-compatible line-side DAA, revision
F, with a Si3056-style system-side register interface, revision D**. A literal
Si3056 BOM is not proven by register compatibility alone.

The match is not complete: register 7 remains zero despite the last native
USB audio setting being 9600 Hz; register 19 bit 3 reads one despite being
reserved-zero; register 27 was nonzero despite being reserved; and voltage
register 29 / threshold register 44 return plausible nonzero values although
the Si3056 sheet restricts those features to Si3019. These are unresolved
USB-interface/variant differences, not grounds to relabel the part Si3019
against its explicit LSID. Keep voltage estimates uncalibrated, do not infer
new gain/clock settings from this identification, and do not repeat reads of
register 27 (the newer sheet says not to read or write it).

The public Si3056 map lists higher SRC modes, but that does not establish
support through this USB implementation. 10286 samples/s remains the highest
advertised and hardware-verified Apple USB audio rate.

## SNR, dynamic range and gain sweep (2026-10-16)

The older Si3016 sheet's 82–84 dB DR figures are full-scale-referenced,
300–3400 Hz measurements under specified termination/current conditions,
not the SNR of an arbitrarily quiet waveform. The closer Si3056/Si3018/19/10
sheet advertises 80 dB paths; Table 4 notes 5–6 define full-scale-referenced
noise with harmonics excluded, using a 1 kHz tone near 10300 samples/s.
These are individual path specifications, not a guarantee for our combined
local loopback. Source:
https://www.skyworksinc.com/-/media/Skyworks/SL/documents/public/data-sheets/si3056.pdf

New physical experiments use 1000 Hz at native 10286 samples/s, analog
HBE-off (§5.30), and the required 12 dB transmit headroom. The new diagnostic
`--loopback-pattern tone` generates a native sine; `--loopback-tone-level`
sets its pre-headroom amplitude in (0,1]. `--loopback-rx-gain` selects temporary
register-15 ARX gain of 0/3/6/9/12 dB. All five test registers including gain,
and the original hook state, are restored and read back for every attempt.
Auto-calibration remains enabled; no full-scale, country termination or
protection controls were altered. Engine data paths/defaults are unchanged.

### Results

| analog local loop measurement | 0:13 | 0:14 |
|---|---|---|
| strongest test tone, 0 dB ARX: received RMS | 4745 counts | 4871 counts |
| quiet 300–3400 Hz noise RMS | 1.86 counts | 4.00 counts |
| active noise RMS, fitted harmonics excluded | 1.86 counts | 4.38 counts |
| active tone SNR | 68.11 dB | 60.92 dB |
| tone / noise plus in-band harmonics (SNDR) | 68.06 dB | 60.90 dB |
| quiet noise below a full-scale digital sine | 81.90 dB | 75.27 dB |
| active noise below a full-scale digital sine | 81.89 dB | 74.46 dB |

The digital reference is explicitly 32767/sqrt(2) RMS counts:
`20 log10((32767/sqrt(2))/noise_rms)`. Thus the earlier ~60/53 dB probe SNR
was never evidence that the codec's dynamic range was limited to 60/53 dB.
The ~82/75 dB values are digital-full-scale-referenced noise headroom, not
an independent calibration of the chip's analog full-scale input, and do not
prove 82 dB clean operation at full-scale signal amplitude.

**Level helps SNR; receive gain does not measurably improve it here.** At
0 dB ARX, tone levels .25/.5/1 give SNDR 55.94/62.02/68.06 dB on 0:13.
0:14's clean/repeated corresponding points give 49.30/55.07/60.90 dB.
Four times the transmitted amplitude produces approximately 12 dB better
SNR without observed digital rail samples. This is a controlled tone result,
not permission to raise real-call transmit power without dBm calibration.

At the strongest test tone, 0:13 ARX 0/3/6/9/12 dB gives SNDR
68.06/68.06/68.10/67.97/68.11 dB. Received signal and noise both rise by
about 12 dB at ARX=12, so there is no useful improvement in input-referred
noise. 0:14 gives about 61 dB at 0/3/6/12; its repeated 9 dB point is worse
at 58.10 dB. Keep 0 dB receive gain as the baseline; the better-performing
0:13 is about 7 dB cleaner in this setup. No gain/default was left changed.

### Anomalies and verification

All 17 bounded hardware attempts completed with register restoration,
on-hook confirmation and no measured RX rail samples. That transport/path
pass is not a signal-fidelity pass: two 0:14 captures were severely distorted
(SNDR 4.89 and 5.08 dB, coherent in-band harmonic ratio approximately -5.7 dB).
They were the low-level/0 dB and strongest-level/9 dB points. The corresponding
transmit taps were clean (harmonic ratios -82.6 and -94.6 dB), so the measured
corruption occurs after our recorded transmit tap. Its origin within USB,
DAA or the loop is unresolved; a tone cannot distinguish delay mixing from
analog nonlinearity. Neither point showed digital rail clipping. Repeats
improved to 49.30 and 58.10 dB respectively; retain both originals rather
than claiming a uniformly clean device or labeling the fault definitively.
0:13's strongest 0 dB control repeated at 67.93 dB SNDR.

`tools/apple_snr_report.py` fits the tone frequency, DC, linear drift and
five coherent harmonics on a steady 0.85 s interval. Noise uses Welch power
in nominal 300–3400 Hz after removing those components; in-band distortion
adds the second/third harmonic powers. Quiet noise is measured independently
before tone onset. The tone's periodic correlation cannot determine unique
transport delay, so analysis windows do not use the loopback analyzer's lag.
The plots retain the distorted captures as separate marked points and join
only the other measurements. No result is a modulation BER or trained rate.

Synthetic checks recover a known non-bin-centered tone, second harmonic and
white-noise band power. Invalid level/gain inputs are rejected before opening
USB. `make gui-apple-test`: all four tests passed. Evidence is in
`reference-data/artifacts/apple-snr-sweep-20261016/` and its `-repeat13`, `-repeat14` and
`-repeat14-gain9` siblings, with the combined JSON and inspected plot in the
first directory.

```sh
python3 tools/apple_snr_sweep.py --quiet-pbx root@asterisk.net.cryan.nz \
  --out artifacts/apple-snr-check
MPLCONFIGDIR=/tmp/apple-mpl .venv/bin/python tools/apple_snr_report.py \
  --self-check artifacts/apple-snr-check
```

## Fresh physical call to PBX 9667 (2026-10-16)

The current dialplan is Answer, Wait(1), Playback(sweep), Hangup. The actual
`/var/lib/asterisk/sounds/sweep.wav` retrieved for comparison is mono 16-bit
8000 Hz, 160000 samples / 20 seconds. Its sweep is logarithmic, about
`f(t) proportional to exp(0.2649*t)`, not the minutes-long approximately
linear sweep inferred from the older short capture. The current reference
therefore supersedes that description of 9667.

Called 9667 physically with modem 0:13 / audio UID 145430. The standalone
audio instrument sent the four DTMF digits at native 10286 Hz and recorded
the dial/answer interval separately from the following 26-second receive
capture. There is an unmeasured audio restart gap between those files; they
are not spliced or treated as one sample-continuous recording. No modem
engine training or local DAA loopback was enabled. Receive gain remained
zero. The line was released in a finally block and on-hook verified.

`tools/apple_9667_report.py` compares short-window received RMS against the
actual source WAV at corresponding sweep frequencies, normalizing to 1 kHz.
The recording includes approximately the last 17 seconds of sweep followed
by a call-progress tone. An abrupt downward frequency jump bounds the sweep;
post-call audio is excluded from the frequency-response estimate. Finite
windows and the end transition limit the highest-frequency measurements;
there is no quoted result beyond the approximately 3.73 kHz analyzed range.

| received frequency | response relative to 1 kHz |
|---|---|
| 50 Hz | -22.18 dB |
| 100 Hz | -7.93 dB |
| 200 Hz | -1.19 dB |
| 300 Hz | -0.37 dB |
| 1 kHz | 0 dB |
| 2 kHz | -0.11 dB |
| 3 kHz | -0.21 dB |
| 3.4 kHz | -0.51 dB |
| 3.6 kHz | -2.62 dB |

The upper -3 dB crossing is approximately 3.63 kHz for this complete
PBX/ATA/line/Apple receive path. It is narrower than the 4.44 kHz combined
local-loopback result at 10286 Hz: the PBX reference is itself an 8 kHz
signal, and this is a different path. No component-specific filter or
extension beyond the DS0 band can be inferred from this call alone.

At 1 kHz the receive level is about 20213 RMS counts, approximately -1.19 dB
relative to a full-scale digital sine, versus 4745 RMS counts for the earlier
strongest internal analog-loop tone. No analyzed sweep window hits the
32760-count near-rail threshold. The whole receive file *does* contain rail
samples (5.93%): these belong to the call-progress tone after the sweep,
not a clipped sweep. The instrument's whole-file RMS label "noise floor"
is inappropriate for a recording full of tones; it is not a live-call SNR.
The source-to-received digital level at 1 kHz is +1.85 dB, which describes
two digital taps with different reference scales and is not a calibrated
analog gain or dBm measurement. Both audio runs reported zero render errors.

Evidence: `reference-data/artifacts/apple-9667-20261016/` contains source WAV, raw dial and
receive taps, final hook status, call log, response CSV/JSON and visually
inspected `call-response.png`. This was a received test sweep, not CONNECT
or payload evidence, and it does not demonstrate a change in live-call SNR.

### How clean the received 9668 DIL is (2026-10-16)

The seven complete playbacks are consistent and clean after correct source
alignment and accurate offline reference interpolation. The first exploratory
model appeared to split them into four good and three bad returns; that was
an analysis error, not established hardware corruption. A wide alignment
search selects a neighbouring pulse for the three low-scoring cases, and
band-limited interpolation removes another significant measurement floor.
Do not record the discarded exploratory fit scores as measured line defects.

`tools/apple_dil_quality.py` fits a forward channel from the actual mu-law
source to the native RX recording. It analyzes only 300–3400 Hz, using an
identical offline bandpass on source and RX. The original coarse matches are
supervised whole-sequence recognition; finer alignment, rate and FIR fitting
use training samples only. Source interpolation uses a band-limited 64x
reference rather than cubic interpolation near Nyquist. Neither the live
path nor the saved receive samples are modified.

Analysis excludes approximately the first 150 ms and uses native relative
indices 1500–15499. Within each 3000-sample cycle, 1000 samples train, 500
are guarded, the next 1000 score, and the remaining 500 are guarded. There
are 5000 training and 4500 scoring samples (including a final partial scoring
window); PSD scores use only the four complete contiguous 1000-sample score
windows, never a spliced waveform. The strongest initial DIL onset is not
covered by this held-out residual figure.

| playback | 65-tap in-band signal/residual | residual RMS, native RX counts |
|---|---|---|
| 1 | 55.52 dB | 2.23 |
| 2 | 55.71 dB | 2.26 |
| 3 | 55.51 dB | 2.31 |
| 4 | 55.84 dB | 2.23 |
| 5 | 55.60 dB | 2.21 |
| 6 | 55.32 dB | 2.28 |
| 7 | 55.47 dB | 2.32 |

Held-out signal RMS is 1331–1379 native counts. Independent quiet windows
2.15–2.45 s after each coarse start measure 1.60–1.67 counts RMS in-band.
The active residual is therefore only about 2.5–3 dB above the quiet floor,
including reference/interpolation/channel-model error. At a full-scale
*digital sine* reference those numbers correspond to roughly 80 dB active
noise/residual headroom and 83 dB quiet noise headroom; these are not a
calibrated analog full-scale specification or the SNR of a louder signal.

The timing search selects a nominal reference-index adjustment of +190 ppm
on every playback. This is relative to the nominal 10286 Hz analysis grid,
not independently calibrated physical oscillator error. Alignment optima
are interior to the search. Increasing the FIR from 65 to 129 taps changes
scores by less than 0.1 dB. An unrelated-reference control scores below
0 dB on every playback, rejecting a trivial fit to the envelope. The raw
17-tap scores of 14–17 dB are dominated by insufficient channel modeling,
not the additive noise floor.

Conclusion: the analyzed DIL portions are repeatable and close to the
measured noise floor once filtering/timing are accounted for. This is a
supervised waveform benchmark for V.90 §8.4.1 source material, not an
unassisted DIL decoder, G.711 codeword BER, negotiated constellation margin
or CONNECT demonstration. `quality.json`, `quality.log` and the inspected
`dil-quality.png` are beside the original 9668 evidence.


### 2026-10-16: RasFinder 6311–6313 blocked before dialling

Attempted each of 6311, 6312 and 6313 using the currently attached Apple
USB modem at USB 1:2, audio UID ending `000000:3,4`, native 9600 Hz,
`AT+MS=V34,0`. This is an analogue-to-analogue test through the RasFinder
AudioCodes ports. The earlier USB 0:13 / UID 145430 unit was absent; its
initial launch failed before seizure. Do not attribute these results to
that earlier, cleaner unit.

All three current-unit attempts aborted during line seizure, before DTMF
or audio training. Register 0x13 reported OPD (0x01), with register 0x0c
0x45 or 0x46. The guard released the line and on-hook readback succeeded.
These are local seizure failures, not RasFinder negotiation failures.
No CONNECT, peer training, or received data was observed.

An on-hook digital PRBS loopback at 9600 Hz then passed with zero sample
and bit errors over 9600 compared samples (unity gain, zero RMSE). One
further 6311 attempt after this audio initialization still tripped OPD
about 81 ms after the hook request (0x0c=0x46, 0x13=0x01). Further seizure
attempts were stopped; overload protection was retained. A clean digital
loopback does not validate the analogue line interface or attached line.

Evidence: `reference-data/artifacts/apple-rasfinder-20261016/`, including immutable coupler
snapshot and build provenance, per-port events/logs/results, on-hook
self-test, and `6311-warm/`. The current physical line/DAA seizure fault
must be resolved before this run can assess any of the three RasFinder
ports.


### Seizure follow-up: protection re-arm timing (2026-10-16)

The initial failures above do not establish a defective modem or line.
Readback confirmed OPE clears (register 17 0x0c -> 0x04); termination
register 26 was 0x00 (ILIM=0, nominal 160 mA overload threshold), not the
60 mA setting. The fault also occurred after keeping native audio running
on-hook, and after selecting the 8 ms off-hook counter. Both experiments
retained overload protection; the counter was restored to 0x20.

One 800-ohm DC termination seizure passed at current code 4 (~13.2 mA),
but the subsequent normal-termination standalone hook test also passed,
while an 800-ohm call attempt failed. Thus the first apparent termination
correlation was not reproducible and is not a fix. Register 26 was restored
to 0x00. Low current measured after OPD cannot exclude an earlier transient
because hardware protection changes the termination when it fires.

The useful controlled comparison adds an OH register readback immediately
after the hook write, before the existing 25 ms OPE re-arm wait. Alternated
call-path seizure tests with normal termination gave:

* Existing sequence: 3/3 OPD trips before audio.
* OH readback before the settling interval: 3/3 successful seizures, no OPD.

These short tests used a 50 ms audio hold, within the initial 300 ms silence
so no destination digits were transmitted. All returned on-hook. The
comparison holds the diagnostic OPE-clear verification constant between
arms; only the new arm includes the post-hook readback. This supports a
USB/DAA transition/re-arm timing problem rather than a sustained line
overcurrent. It does not independently measure the physical switch timing
or distinguish synchronization from the extra readback delay.

The coupler now verifies that OPE is clear before seizure, confirms OH
after the write, and then starts the Si3056 §5.22 25 ms settling interval.
Failed readbacks abort and restore protection; the overload guard remains
active after re-arming. No current-limit threshold is raised. The hardware
comparison, build logs, and subsequent call evidence are under the same
artifact root (`seizure-comparison.json`, `seizure-*-*.jsonl`, `readback.log`).
The coupler builds and its diagnostic self-test passes.


With that readback sequence, fresh strict-V.34 calls at native 9600 Hz
passed seizure on **all three** requested ports with normal termination,
OPE enabled, current code 7 (~23.1 mA), and no OPD/overload event:

| destination | observed result | CONNECT/data |
|---|---|---|
| 6311 | Answer tone, V.8 succeeds, V.34 reaches `PHASE3_WAIT_S` / TX `J`; later NO CARRIER | none |
| 6312 | Answer tone, V.34 reaches `PHASE3_WAIT_S` / TX `FIRST_S`; later NO CARRIER | none |
| 6313 | Answer tone, V.8 call negotiation fails (status 4) | none |

PBX snapshots identify the current Apple modem source as AudioCodes
extension 6314; the earlier HT802 topology should not be assumed for this
run. PBX snapshots for 6311/6312 show ulaw. The 6313 active snapshot was
empty, so the observed answer tone/V.8 samples alone do not establish its
PBX bridge details. All three attempts ended on-hook. Evidence is in
`6311-fixed/`, `6312-fixed/`, `6313-fixed/` and `fixed-summary.json`.
These are successful line seizure/dial/answer tests, not successful data
connections. The remaining V.8/V.34 interoperability problem is separate
from the reproduced and corrected protection re-arm startup issue.


### Direct pair with reported 5 V feed (2026-10-17)

Both attached Apple modems enumerated at USB `0:3` and `1:2`, initially
unconfigured. Setting configuration 1 exposed both CoreAudio endpoints.
Each was briefly seized separately using the coupler's normal protected
hook sequence. Both read register 5 = 0x09, register 12 = 0x40, register
19 = 0x02, and line sense = 0: current code zero and dropout asserted,
without an overload indication (Si3056 §5.8, §5.22). Thus neither trial
established operating loop current. Idle sense varied, and its voltage
scale is uncalibrated; these readings do not independently establish the
supply voltage, continuity, or a failed DAA. No peer audio or training was
attempted. Both were restored on-hook with monitoring off and hardware
protection enabled. Final readbacks and observations are saved locally in
`reference-data/artifacts/apple-direct-line-20261017/`. Check physical feed and wiring
under off-hook load before interpreting a modem negotiation failure.

Follow-up after the wiring was clarified: green is continuous between the
modems; red is interrupted by the USB supply (one modem to negative,
positive to the other modem). This is a series loop, so the individual
seizure results above cannot assess the feed: the other on-hook modem
opens the circuit. With BOTH hooks asserted, both modems report register
12 = 0x41 (current code 1, approximately 3.3 mA), register 19 = 0x00
(no dropout), and protection enabled, without overload. This supports
continuity but not adequate operating current; Si3056/Si3018/19/10
Table 4 specifies a minimum operating loop current of at least 10 mA
(configuration dependent). The reported 5 V feed is therefore a likely
limitation, not a demonstrated working modem bearer. No audio/training
test was performed at this undercurrent. Both hooks and monitors were
released afterward. Evidence: `both-offhook.json` in the same artifact
directory. A DC feed alone supplies neither dial tone nor ringing.

After the user changed the feed (following discussion of 12 V), the first
simultaneous-hook check showed sense about 6 on each DAA, current codes
2/3 and receive overload. A subsequent seizure for the audio test read
current codes 7/6 (nominal 23.1/19.8 mA), sense 7/6, no dropout, receive
overload or overcurrent, with hardware protection enabled. These are
coarse per-device current readings on a series circuit, not independent
precision measurements. The actual supply voltage was not metered; the
reason for the change between trials is not established.

A low-level 1 kHz tone (CoreAudio TX amplitude 0.05, 9600 Hz native)
passed in both directions. The peer captures contain strong tone windows
with amplitudes about 1382 and 1349 native counts, compared with near-zero
1 kHz bins outside the bursts; local returns were only tens of counts.
Both audio tools returned success, with fresh captures in
`reference-data/artifacts/apple-direct-line-20261017/audio-new-feed/`. This demonstrates
bidirectional physical audio and adequate indicated current during that
trial, not V.34 training, CONNECT or user data. Both hooks and monitors
were released afterward.


### Direct powered pair: quality and intermittent distortion (2026-10-17)

Fresh bidirectional tests at native 9600 Hz, with the changed feed above,
measure the complete USB/audio/DAA/powered-pair path. The sampled
300–3400 Hz response ripple is 0.53 dB in either direction. Clean 1 kHz
gains are -1.48/-1.69 dB. Independent quiet voice-band noise is about
32 counts RMS per receiver, with common tones near 104/311 Hz, 50 Hz hum
and higher-frequency contamination. Its source has not been isolated.

At transmitted sine amplitude 0.15, voice-band SNDR is about 39 dB in
both directions, with 0.016–0.032% 1 kHz THD on the moderate controls.
Strong amplitude 0.35 gives 46 dB SNDR when clean, but reverse-direction
trials are NOT consistently clean: 2/3 show severe waveform distortion
(about 53/76% THD and 5.7/1.5 dB SNDR); the third is clean. The other
direction is clean in 2/2 strong-level trials. No int16 rail clipping or
AudioUnit render errors explain the failed captures. This is not enough
evidence to attribute the fault to a peer, the physical line, the supply,
or the USB/audio path. The tool generates a known sine but does not
retain a tap of each actual submitted output buffer.

Simultaneous 1 kHz/1.4 kHz transmission initially has poor residual
voice-band ratios (4.0/27.3 dB) after removing the two expected tones.
A moderate-level repeat is clean at 38.9/39.0 dB; a lower-level repeat
is 29.4/29.4 dB, matching the one-way noise floor. Thus the bad duplex
trial is intermittent too and must not be discarded. Clean moderate
duplex own-echo levels are 27–29 dB below the far tones.

Evidence and reproducible analysis: local
`reference-data/artifacts/apple-direct-quality-20261017/report.md`, `summary.json`,
`duplex-summary.json`, `line-quality.png`, raw captures and per-run logs.
There are 34 one-way bursts plus three simultaneous-tone trials.
Measurements use independent quiet windows, fitted tone frequencies and
interior burst windows; the fitter passes a synthetic offset/noise/THD
control. Both devices finish on-hook with monitoring off and hardware
protection enabled (Si3056 §5.8, §5.22). This is an analogue-path quality
measurement, not V.34/V.90 training, CONNECT or a BER/data-rate result.


### V.90/V.91 over the direct powered pair (2026-10-17)

Rebuilt the coupler against the current engine (the prior binary predated
recent V.91 work). Added `--answer-now` for the ringless direct-pair case
(V.250 §6.3.5); this bypasses only ring waiting, keeping the protected hook
sequence. Both hooks are initially asserted to complete the series circuit.
USB/audio mapping from IORegistry: 0:3 is the modem at 00110000, audio UID
000000; 1:2 is at 01130000, audio UID 1130000.

Two completed 45-second V.90 trials at native 9600 Hz exchange the physical
digital-answerer/analogue-caller roles. Both negotiate V.90 and reach
Phase 4: the digital end accepts a CRC-valid CPt and transmits TRN2d/MP.
`crc=0` in that log is the CRC remainder, not a failure flag. The analogue
receiver enters TRN2d but fails its B1d/constellation deadline and retrains
(V.90 §9.4.2, §9.5.2). No CONNECT or user data in either orientation.
Earlier Ja acquisition uses the existing heuristic fallback rather than
a CRC-valid descriptor, so do not score every earlier detector event as
independently established interoperability. The analogue-side logs also
report no constellation at their 3-sigma margin before forming the offer.

V.91 at 9600 Hz fails in V.8, exercising no V.91 startup. A 30-second
retry at native 8000 Hz removes the resamplers: BOTH ends select V.91
and enter startup, but neither reports Ez/INFO acquisition, and both
escape to V.34 after no INFO' within 1.5 s (V.91 §8.3). No V.91 rate,
constellation, CONNECT or data is established. This identifies the
current implementation's first failure, not a proof that every possible
analogue PCM recovery method must fail.

The native V.91 own TX taps contain the intended Ucode-66 magnitudes
(±2236) for 1.495 s, with both signs present. All eight processes exit
normally with zero TX ring underruns and balanced device frame counts;
no TX tap clips at int16 rails. One swapped V.90 RX capture rails only
in its last 0.62 s as the peer releases the series loop first. Both hooks
and monitors finish cleared, protection enabled. No DSP constants,
training thresholds or G.711 bearer logic were altered for these trials.

Full evidence, tested executable snapshot/hash, launches, logs, device
TX/RX taps, engine dumps, manifests, summary and report are local under
`reference-data/artifacts/apple-direct-training-20261017/`. Coupler build, diagnostic
self-test, resampler self-test and conflicting dial/answer argument
check pass.

### Direct-pair V.90 calibration and delayed upstream E (2026-10-17)

Follow-up evidence is under `reference-data/artifacts/apple-v90-followup-20261017/`.
Use `ME_V90_JA_HEURISTIC_FALLBACK_MS=0` for this own-engine pair: the
default energy-gap heuristic advanced before decoding Ja, leaving
`dil_requested=false`. The digital end then sent Ri where the analogue
receiver expected its requested measurement DIL. Waiting for the CRC-valid
descriptor recovers the exact 1260-bit N=120/LSP=12/LTP=11 frame and sends
the real DIL. This is a peer-specific diagnostic setting; the default was
not changed because older SmartLink interop depends on the heuristic.

The waveform receiver also latched the first unconverged TRN1d magnitude
as an absolute Ucode (e.g. 65), although the digital TX tap carried the
negotiated U_INFO=78 (V.90 8.4.5). CMA normalizes modulus and cannot establish
that absolute level. Anchor the waveform path to negotiated U_INFO only
after TRN1d is observed; retain measured codewords on a byte-exact DS0.
On the same zero-DIL recording/CP, independent TX/RX codeword comparison
improves from 4.354% to 99.374% exact TRN2d codewords. This replay establishes
the calibration effect only up to the first divergent transmission.

The completed live `strict-reference-1` trial subsequently receives all
48/48 B1d frames with zero bit errors/demap failures and enters analogue
DATA, 33333 downstream / 31200 upstream. It is **not a working bidirectional
V.90 connection**: the digital receiver fails upstream B1, and later falls
back to V.22bis. A historical CONNECT at each DTE therefore falsely joins
two different modes. The bench runner now requires both V.90 startup flags
before sending payload. No payload delivery has been established.

Capping the upstream to N=4 in `strict-reference-9600up-1` is verified in
both CP/MP selection and handover at 9600 bit/s; it does not fix acquisition.
The own native TX tap acquires B1 offline at 100.0% fit. The same hardware
RX recording acquires at 99.8% when replayed with a timely E handover, but
fails with the live handover: the CP-bitstream E detector fires more than
six seconds after B1 (V.90 9.4.2.4-.5). The 8192-sample T/3 history retains
only 0.85 seconds, so its nominal 120000-sample search-back cannot find B1.
Restoring 131072 samples retains 13.65 seconds without changing playout or
sample accounting; the two complex observation rings occupy 2 MiB together.
With a fixed late handover/history, the retained RX fixture now acquires at
100.0% fit. `reference-data/artifacts/apple-direct-v90-b1/README.md` records its provenance.
The first two live history-build retries fail earlier in Phase 3, so they
do not establish live validation of that B1 fix.

The optional zero-DIL fallback also no longer offers mu-law levels 1..8,
whose spacing is 8 counts against the measured ~32-count voice-band noise.
It offers Ucodes 32,36,...60 in both laws, with at least 128 counts spacing;
every point individually meets Table 15's strictest 2540 amplitude limit.
Its eight-point product, CP/CPt K and rates remain unchanged. Both laws and
Sr=0..3 pass the power/spacing checks; this change alone did not complete
live training. It is separate from the normal measurement-DIL success.

#### Native 8 kHz waveform path and the linear TX handover

The direct-pair level measurements below refute the earlier blanket claim
that native 8000 Hz cannot support PCM recovery: a 16-level 8 ksymbol/s
alphabet decodes without errors in both laws/directions through the real
hardware using the existing PR4 detector. Added opt-in
`APPLE_MODEM_8K_PCM=1`: interpolate the native analogue RX waveform 2/1 for
the existing T/2 receiver, retaining direct 8 kHz engine RX/TX. This creates
no bandwidth above 4 kHz and does not resample a network G.711 bearer.
The normal 9600 Hz path and default 8000 Hz V.34-only path remain available.
The experiment does not add the 9600-Hz frontend's loop-clock correction
to native 8 kHz; these direct peers measured sub-ppm relative offsets.

The same echo-canceller window is scaled in samples for the 8 kHz
experiment: 750-sample first delay, 640 taps, 4000-sample solve interval.
Own TX/local RX correlations measured 143/149 ms echo peaks at 8 kHz,
versus 120/124 ms at 9600 Hz, inside the existing 94–174 ms coverage.
With the recorded live TX echo reference and pinned CP, the new replay
recovers a CRC-valid MP where the uncancelled 8 kHz live run stalled in
TRN2d. This does not judge later replay protocol actions after TX diverges.

The next live 8 kHz trial receives all B1d frames and acquires upstream B1
at 100.0% fit, but still times out: `me_tx_audio()` called only
`v90_phase3_tx()`, leaving the controller in DATA and the engine in TRAINING.
It never entered the engine's data stack or real data mapper. The linear
frontend now uses the same codeword generator as the byte-exact path and
expands its result only for physical waveform output. This preserves the
exact B1d-to-data boundary (V.90 9.4.1.5), including the remainder of the
output block, instead of emitting idle until the 60-second timeout.
`ENGINE_PEER_LINEAR_TX=1` exercises that API through the deterministic
whole-engine peer; both G.711 laws pass CONNECT and DTE payload regression.

Completed live `strict-history-8k-linear-v14-1` reaches V.90 DATA at both
ends: 28000 downstream, 9600 upstream. All four upstream marker lines
arrive intact; two of four downstream lines arrive intact and later
downstream output is corrupted. This is the first verified bidirectional
V.90 payload on the direct pair, **not reliable bulk transfer**. The
analogue B1d itself has 39 bit errors and 11 demap failures. No B1 acceptance
threshold was loosened. Both processes exit normally, without TX ring
underruns; device frame counts balance at 719872 in/out on the answerer
and 720384 in/out on the caller.

The first corrected native-9600 trial, `strict-linear-9600-lapm-1`, reaches
32000 downstream / 9600 upstream, establishes V.42 LAPM and V.42bis at both
ends, and delivers all four marker lines intact in both directions. A bulk
repeat exposes a separate timing failure: E is detected at +102078 ms,
after V.42 detection has already timed out and fallen back to V.14.

For the linear RX frontend, a CRC-valid accepted data-mode CP' now starts
the independent buffered B1 search (V.90 9.4.2.4-.5), without claiming an E
event. The early entry suppresses the optional blind/non-reset fallback:
the complete B1 template's >=95% fit and out-of-sample validation remain
mandatory before bits are published. Byte-exact DS0 reception retains its
existing E-triggered handover/fallback, and in-DATA rate renegotiation is
outside this startup recovery. The fixture also tests that early entry
does not change the receiver's event slot. Whole-engine tests exercise
both linear RX/TX APIs through `ENGINE_PEER_LINEAR_RX=1` and
`ENGINE_PEER_LINEAR_TX=1` in both laws.

Completed final-build `strict-cpprime-9600-lapm-bulk-2` confirms that path
live: CP' starts B1 search at +16923 ms, the upstream B1 fits at 100.0%,
both ends establish LAPM/V.42bis, and each sends a separately generated
4096-byte printable body with framing (4185 bytes total per direction).
The complete received blocks match byte-for-byte in both directions,
with no extra payload bytes (the caller also records the normal teardown
NO CARRIER result). Both engines remain in DATA until normal disconnect.
The 90-second call exits normally;
both device counts balance at 863744 in/out, with zero TX ring underruns.
Both hooks/monitors finish cleared and hardware protection enabled.
This proves a working 32000/9600 V.90 data connection in this configuration;
it is not a long soak or proof of every startup succeeding. An earlier
CP'-search hardware retry failed in Phase 3 before exercising the change.
The upstream N=4 cap remains part of this verified bench configuration.

### Exhaustive native PCM level/rate measurements (2026-10-17)

`apple_usb_modem_audio list` now enumerates each CoreAudio stream's
available physical formats. Both modems offer signed 16-bit mono input
and output at exactly 7200, 8000, 8229, 8400, 9000, 9600 and 10286 Hz.
No other native bit depth is advertised; host float conversion would not
establish a different physical converter resolution.

Added native `play RATE INPUT.s16 CAPTURE.s16`, which emits the supplied
int16 samples without amplitude scaling, companding, live resampling or
engine processing. `tools/apple_pcm_matrix.py` generates and captures the
bench stimuli, and refuses to replace different reference waveforms when
resuming. Capture manifests retain the generated waveform hash, playback
logs, USB status/protection checks and final hook/monitor releases.

The exhaustive matrix contains 14 direction/rate captures. Each transmits
every signed int16 value twice, in independent random orders; every mu-law
and A-law codeword 64 times at 8 ksymbol/s; quiet; and 1 kHz tones from 64
to 32000 counts peak. A DC staircase would not identify this AC-coupled
path, so the model includes channel memory. A separate 14-capture matrix
tests a fixed 16-level, roughly 2100-count-RMS alphabet, with zero source
construction clipping. A two-direction 8229 Hz repeat checks an outlier.

Measured voice-band quiet noise is 26.6–29.7 received counts RMS. Full-scale
1 kHz SNDR is 49.9–51.1 dB, with nearly constant gain across the amplitude
sweep. Random full-range PCM nevertheless hits clipping plateaus:
negative extrema are -30069, -31703, -32157, -32489, then -32768 at the
three highest native rates. Hundreds of samples can land on the identical
below-full-scale extrema. Counting int16 rails alone misses that clipping.
The measurements describe the combined DAC/line/ADC path, not an isolated
converter or an independently metered supply.

Offline clock registration uses separate high-level tones and a 129-tap
windowed-sinc interpolator; raw hardware extrema and registration overshoot
are counted separately. A 129-tap forward channel fit is validated on
different samples; voice-band validation SNDR is 37.2–39.8 dB. The raw
int16 arrays retain all 65536 TX values, raw/registered RX, and predictions.
There are only two observations per individual int16 code, so those alone
are not per-code noise distributions. Per-G.711-code estimate/noise tables
use many more observations. The true-predecessor PR4 estimate is explicitly
an oracle bound, separate from actual trellis decisions.

`make apple_pcm_pr4` builds the offline known-reference PR4 grader, using
the production detector, with 128 T/2 forward taps (the live frontend uses
64). It trains on 5000 known symbols, starts decoding
at symbol 8000, and scores only unseen later symbols with a fixed known
alphabet. The fixed 16-level probes decode zero errors over about 8312
held-out symbols in each law/direction at 8000, 8400, 9000, 9600 and
10286 Hz. 7200 fails badly. One 8229 B-to-A capture degrades to 14–15 dB fit;
its separate repeat has zero errors in all four law/direction combinations.
Do not attribute that intermittent failure to an invariant rate limit.

The full alphabet is not reliable. At non-8000 rates its offline waveform
construction itself clips 24–72 resampled native values, recorded in the
manifest, so its errors cannot all be blamed on hardware. The smaller
constellation probes do not have that construction clipping. All bench
captures so far are one-way with the opposite transmitter silent; live
full-duplex echo and startup remain additional requirements.

A further 8000/9600 Hz matrix in both directions sweeps 8,16,24,32,48,64,96
and 128 signed levels in each law. Greedy spacing maximises the minimum
separation subject to uniform RMS <=2540 (V.90 Table 15); it is a test
alphabet, not the six-slot CP mapper's power-shaping calculation. Default
24-step traceback first shows errors around 32 signed levels. Offline
256-step traceback recovers the 32-level mu-law sets without errors in
both directions at both rates; one 8000 A-to-B A-law set has 15/7848 errors,
the other 32-level cases have none. 48+ levels still fail heavily with that
longer traceback. These observations bracket this detector's tested
capability, not a proof of Shannon capacity or a negotiated V.90 rate.
No live traceback or DSP constants were changed for the size experiment.

Local evidence/report/plots: `reference-data/artifacts/apple-pcm-matrix-20261017/`,
`reference-data/artifacts/apple-pcm-subset-20261017/`,
`reference-data/artifacts/apple-pcm-subset-repeat-20261017/` and
`reference-data/artifacts/apple-pcm-sweep-20261017/`. Reproduce with numpy/scipy using
`tools/apple_pcm_matrix.py capture OUTPUT [--profile subset|sweep]`, then
`analyze OUTPUT`; select the rate list with `--rates`. The large captures
remain gitignored; the fixed delayed-E acquisition fixture is committed.

## Receive equaliser geometry experiments (2026-10-17)

The physical analogue path now accepts bounded diagnostic overrides
`ME_V90A_PR4_TAPS` (4..512, default 64) and `ME_V90A_PR4_DEPTH`
(1..256, default 24 per interleave). Invalid values restore the respective
compiled default. These change the local partial-response receiver, not PCM
codewords in the SIP bearer or the V.90 §5.4.3 constellation mapping.
Training still uses §8.4.5 TRN1d; depth adds approximately `depth/4` ms of
traceback latency at 8000 symbols/s, plus the fitted decision delay. The
existing `ME_V90A_PR4_MU` controls tracking in thousandths (default 2).

`tools/apple_pcm_pr4.c` additionally accepts `[traceback [taps
[tracking-milli]]]`, retaining the previous defaults 24/128/0. It reports
geometry and adaptation count alongside the held-out error counts.
The recorded-input sweep is `reference-data/artifacts/apple-rx-eq-20261017/sweep.json`:
8000/9600 Hz, both directions, both laws, 32/48 signed levels, 64/128/192
T/2 taps, depth 24/64/128, tracking 0/.002. Each run trains on 5000 known
symbols and scores later symbols against the immutable transmit reference;
the candidate set is the complete alphabet, never the next true symbol.
Offline clock registration and known multilevel training make this a stronger
measurement than live two-point TRN1d acquisition.

For the 32-level alphabet, with tracking .002:

| T/2 taps | Traceback/interleave | Errors / scored symbols | Error rate |
|---:|---:|---:|---:|
| 64 | 24 | 785 / 66581 | 1.179% |
| 128 | 24 | 417 / 66510 | 0.627% |
| 128 | 64 | 20 / 65870 | 0.0304% |
| 128 | 128 | 15 / 64846 | 0.0231% |

Longer tracebacks emit fewer tail symbols, so denominators differ. At 128/64,
seven of eight cases had zero errors; the remaining 20 were 8000 Hz A-to-B
A-law. 192 taps offered little additional training fit and worse aggregate
32-level errors than 128. Tracking .002 did not improve this short,
clock-registered dataset over held taps; that does not justify freezing live
tracking, which previously prevented long-call drift. The 48-level cases
still had roughly half their symbols wrong even with longer filters and
history. These measurements support additional receive margin, not a claim
that denser constellations or a higher negotiated bit rate are usable.

A paired replay of the same baseline live input (first 22 s, recorded own-TX
reference for echo cancellation) trained at 41.8 dB with 64 taps and 42.0 dB
with 128 taps. Increasing depth did not change that fit, as expected. Only
this pre-Phase-4 training comparison is evidence from the engine replay:
changed detector delay changes later responses, so its later decoding cannot
be scored as though the peer had heard those responses. The separate
known-reference benchmark above emits no responses and can score the entire
held-out recorded sequence.

### Live equaliser comparisons

All calls used one binary (SHA-256
`c9558e2c9cfac1c24edf30f482343fe9853893758057ce49e22eada573014ef6`),
native 9600 Hz, strict Ja, upstream N=4, normal V.42 LAPM/V.42bis and
90-second holds. Each successful trial sent an independently generated
4096-byte body plus markers in each direction and required the complete
opposite payload byte-for-byte, not merely CONNECT or matching markers.
They ran in A1/B1/A2/B2/C1 order under
`reference-data/artifacts/apple-v90-followup-20261017/rx-eq-*`.

| Trial | Taps / depth | Completed outcome | TRN1d fit |
|---|---|---|---|
| A1 | 64 / 24 | 33333/9600, exact payload both ways | 41.8 dB |
| B1 | 128 / 24 | V.8 negotiated no usable modulation; PR4 never started | — |
| A2 | 64 / 24 | 33333/9600, exact payload both ways | 41.8 dB |
| B2 | 128 / 24 | 32000/9600, exact payload both ways after three pre-PR4 training retries | 42.0 dB |
| C1 | 128 / 64 | No data connection; Phase 4 failures/retrains with poor training fits | 21.9, 18.5, 16.7 dB |

A1/A2/B2 had exact bidirectional payloads, no render errors and no TX ring
underruns. All five trials finished before scoring; both hooks and monitors
were released afterward and hardware protection remained enabled. The
combined 128/64 setting is **not a validated live improvement**. Its poor
fits occur before Phase 4 traceback starts, so failure is not proof that the
longer history itself is harmful. Likewise B1 failed before PR4 even started.
The alternating baseline/filter sample is only two calls per arm, with
2/2 versus 1/2 complete data calls: it cannot establish a reliability gain.
The default remains 64 taps/depth 24; the overrides are experimental.

The next useful RX work is to identify and prevent bad TRN1d training fits
(reference decisions, clock acquisition and residual echo), then repeat
alternated live tests of the longer traceback on clean training. More taps
alone add little fit on these captures. The recorded-input 39-fold reduction
in 32-level errors is evidence of potential receive margin, not achieved
live throughput or long-call reliability.

Validation: expanded slot-varying PR4 synthetic test covers 16/24 frozen,
128/24 tracking and 128/64 tracking, all with zero codeword errors; five
analogue/recorded-upstream regressions and two whole-engine linear-I/O
regressions passed, as did the coupler resampler self-test. Detailed output
is in `reference-data/artifacts/apple-rx-eq-20261017/`; no PCM bearer processing or ITU
constants were changed (V.90 §§5.4.3, 8.4.5).

## Training quality and B1 search latency (2026-10-17)

The failed `rx-eq-C1-t128-d64` recording does not support simply replacing
hard training decisions with an ideal TRN1d reference. Independently checked
against V.90 §§5.3/8.4.5's zero-initialised scrambled ones, the digital TX
engine's first 18000 checked TRN1d signs are exact; its native TX tap fits a
linear interpolated reference above 60 dB on held-out samples. The analogue
side's raw RX instead contains large disturbances at approximately 48 native
samples (5 ms) spacing, with occasional changes in cadence. Many affect two
adjacent samples. A known-sign PR4 fit on one bad interval was only 15.4 dB,
versus 21.5 dB when trained against the FSE decisions: the latter partly
learns the disturbances. Forcing the ideal reference would hide that fact,
not establish a clean receive channel. No training-pattern substitution was
installed.

A new native-9600, fixed-16-level capture in both directions was clean:
37.3–37.6 dB PR4 fits, zero held-out errors in both laws. Thus the disturbance
is intermittent. Its source is not established. The new callback diagnostics
report input/output timestamp discontinuities, signed gap-frame totals,
callbacks exceeding their block duration, and maximum wall duration; they
never fill a gap or splice samples. The initial three live rate trials
reported no timestamp gaps and no over-budget callbacks. These observations
cannot exclude faults hidden below CoreAudio.

### Native 9000 Hz and sample accounting

The physical coupler now accepts native `--rate 9000`: receive ratios 8/9 to
8 kHz and 16/9 to T/2, transmit 9/8. This retains the existing continuous
clock correction and scales the echo-canceller geometry by the native rate.
Unlike 9600, 9000 has an integer count of samples per USB millisecond; this is
an experimental comparison, not a proven cure for the intermittent noise.
The hardware's native format is still mono signed 16-bit PCM. The default
stays 9600 Hz and the byte-exact SIP G.711 bearer is unchanged.

The new TX ratio exposed truncation in the resampler initialisation:
`M*240/L` was rounded to an integer *before* conversion to fixed-point.
It is now calculated in fixed-point, preserving the fractional phase units.
The existing 9600 and opt-in 8000 ratios are numerically unchanged. Expanded
self-tests check tone gain/residual, output sample counts, and ±200 ppm for
all three new paths; all pass. Callback continuity accounting also has a
hardware-free check.

### Remove the extra half-second wait after CP′

Clean training uncovered a separate problem. In two native-9000 trials the
128-tap/depth-64 receiver reached B1d and DATA with ~42 dB TRN1d fits, but
V.42 failed to establish. The digital B1 search always waited for an extra
4800 T/3 samples (500 ms) beyond its handover anchor, even when B1 was already
in the captured history. Validated upstream data then appeared around 580 ms
after CP′ handover. V.42 §7.2.1.2's originator was already sending ODP while
our answerer waited; that delay consumed much of §9.1.1's default 750 ms T400.

`v34_v90_begin_b1_search()` now places the first coarse window's end at the
buffered input (with the existing small-history lower bound), rather than
500 ms into the future. This applies to the CRC-valid CP′ entry used by the
linear physical frontend (V.90 §§9.4.2.4–9.4.2.5). The existing forward retries
remain available when B1 really is later. E-triggered entry, complete-template
>=95% fit, out-of-sample validation and event semantics are unchanged. No E
event or carrier is fabricated. `ME_V90_T3_CPPRIME_LOOKAHEAD=4800` restores the
old window for controlled comparisons; default 0 searches buffered input first.

On the preserved `apple-direct-v90-b1` fixture, identical replay input with
the old/new setting publishes the first data bit within 600/100 ms of forced
late CP′ handover (20 ms block-end measurement). The early-CP′ fixture still
acquires, within 400 ms; the ordinary late-E entry still acquires, within
640 ms. A new fast-tier test requires acquisition and publication within
200 ms in the late-CP′ case and retains the assertion that CP′ does not
manufacture E. Eight targeted analogue/upstream/linear-I/O regressions pass.

### Pre-comparison code review (2026-10-18)

The CP′ entry retains its startup/linear-frontend qualification and disables
blind acquisition. The remaining T/2 symbols in the callback cannot enter
the T/3 data decoder: `V34_RX_STAGE_DATA` already requires `v90_t3_in_emit`.
The raw block was captured before CP decoding, so the next callback must
continue from the next physical sample, without feeding that block twice.
The shifted CP′ anchor is a search-window control, not an observed E time;
short history still clamps it at zero and can retain the forward wait.
Retries are bounded and the existing final-retry passable-window policy is
unchanged; the early search does not guarantee a high-quality live carrier.

One validation hole was closed: a complete B1 candidate with fewer than
128 available post-B1 symbols previously skipped the out-of-sample check.
The search now retains that candidate and waits for the existing minimum
validation span, without spending retries or changing thresholds
(V.90 §9.4.2.5). No line samples are inserted, removed or replayed by this wait.

The upstream replay harness also discarded partial blocks at handover and
EOF. It now feeds every sample, with partial-block first-bit timestamps.
Two additional fast-tier cases use handovers one 8 kHz sample past a block
boundary, covering both retained and short history. The physical coupler now
counts and reports TX ring overruns in headless and replay runs, including
every unqueued output sample; previously only the GUI exposed a drop count.
This is observation only: overflow/underrun behavior is unchanged.

Final targeted validation: 11 tests passed (five fixed Apple replay cases,
analogue TX/RX/PR4, V.PCM loopback, and both-law whole-engine linear I/O),
plus the coupler resampler/continuity self-test. Restoring lookahead=4800
still acquires but publishes at 600 ms and fails the 200 ms latency check.
The new default passes that check. These are offline results, not a new
hardware comparison.

For the next tower-wired comparison, alternate native 9600/9000 while holding
the CP′ lookahead, filter/traceback, rate caps and payload constant. Compare
completed calls using exact bidirectional payloads and CP′-to-first-upstream
data latency, alongside training quality, render errors, timestamp gaps,
over-budget callbacks, device frame balance, ring underruns and overruns.
Then compare lookahead 0/4800 at one fixed native rate as a separate variable.
Native 9000 changes only the codec conversion; negotiated 3200-baud upstream
still runs internally at 9600 T/3 samples/s (V.90 §5.2, V.34 §5.2).
The forward constant 4800 therefore means 500 ms at 3200 baud and about
533 ms at 3000 baud, regardless of the physical codec setting.

Remaining risks: acquisition work, synchronous tap writes and GUI locks
still run inside audio callbacks; a slow callback or a full/empty TX ring
can spoil a hardware comparison. Timestamp continuity does not establish
USB/codec sample integrity. The intermittent raw-waveform disturbance remains
unexplained; do not infer that native 9000 cures it from clean training alone.

### Direct-pair comparison stopped by USB control failure (2026-10-18)

Tested reviewed build `d129422d` against the two physically connected Apple
modems, newly enumerated at USB 0:2/UID 000000 (digital answerer) and
1:3/UID 1130000 (analogue caller). Both initially needed `--configure`.
Kept strict Ja descriptors, upstream N=4, CP′ lookahead 0 and default
equalizer/traceback, with 60-second holds and planned 4096-byte payloads
per direction. This local direct-USB test does not traverse SIP or Wi-Fi.

The completed native-9600 call reaches both V.90 startup flags at
32000 downstream/9600 upstream. Analogue B1d is 48/48 frames with no bit
errors or demap failures. The digital decoder misses CP′, however, and
uses E; first upstream data is traced 603 ms after digital DATA entry.
The caller falls back to V.14 while the answerer enters LAPM establishment.
Both DTEs never jointly CONNECT, so the runner correctly sends no payload.
This exercises the existing E path, not the early CP′ search
(V.90 §§9.4.2.4–9.4.2.5). Both devices count 576000 samples in and out,
with no timestamp gaps, over-budget callbacks, render errors, TX underruns
or TX overruns. This is training success, not a working data connection.

The following native-9000 call times out at the caller's V.8 stage and
does not complete V.90 startup. No payload is sent. Logged audio callbacks
have balanced per-device frame counts and no continuity/ring errors, but
the answerer's USB register reads become unresponsive near teardown.
It still enumerates with configuration 1; all line-register reads then
return -1, and protected hook-off/monitor-off commands fail. Its final hook
and protection state cannot be verified. The peer is confirmed on-hook,
monitor off, with hardware protection enabled. One standard USB reset of
the unresponsive unit times out; physical USB reconnection is required
before further calls. This does not identify native 9000 as the cause.

A repeat baseline was inadvertently launched during the prior runner's
cleanup and aborted before launching either modem process; it is excluded.
No B2 call was made. Wait for the entire runner, including cleanup, to exit
before the next trial. Blocking USB reads also extend the nominal hold:
the 9000 runner takes 87.6 seconds including teardown. A zero process exit
does not prove a successful hook release; inspect cleanup readbacks.

Retained executable/hash, runner, per-side logs/events/audio taps, launch
environments, manifests and report:
`reference-data/artifacts/apple-direct-review-20261018/`. The planned
alternating comparison remains incomplete; no payload reliability or rate
advantage is established by these two calls.

### Reconnected direct pair: live early-CP′ recovery (2026-10-18)

After the user reconnected both USB units, register reads recovered. The
mapping is now digital answerer 0:2/UID 000000 and analogue caller
1:2/UID 1130000. Reconfigured both audio interfaces and ran four sequential
60-second calls, waiting for each runner's complete cleanup. The executable
is unchanged from `d129422d`; strict Ja descriptors, upstream N=4, lookahead 0,
default equalizer/traceback and 4096-byte bodies per direction stay fixed.

| Run | Native codec rate | Completed result | CP′ search to first upstream bit |
|---|---|---|---|
| A1 | 9600 Hz | Training stalls before DATA; no payload queued | Not exercised |
| B1 | 9000 Hz | Exact bidirectional payload, 33333 downstream / 9600 upstream | 73 ms |
| A2 | 9600 Hz | Exact bidirectional payload, 33333 downstream / 9600 upstream; E entry | Not exercised |
| B2 | 9000 Hz | Exact bidirectional payload, 32000 downstream / 9600 upstream | 113 ms |

B1 and B2 decode CRC-valid CP′ and exercise the reviewed buffered B1 search
(V.90 §§9.4.2.4–9.4.2.5), rather than waiting for E on the CP demodulator.
Both ends establish LAPM and negotiate V.42bis in all three successful
calls. Each direction's independent 4096-byte printable body plus framing
(4161 bytes total) arrives byte-for-byte, with no extra user-data bytes:
the DTE captures contain only CONNECT, the exact peer block and teardown
NO CARRIER. The two 9000-Hz analogue B1d decodes each report 48/48 frames,
3 bit errors and zero demap failures; the successful 9600-Hz B1d has zero
bit errors. No acceptance thresholds were loosened.

Every side of every call reports zero render errors, timestamp gaps,
over-budget callbacks, TX ring underruns and TX ring overruns. Device
counts balance at 576000 in/out per 9600-Hz leg and 539648 in/out per
9000-Hz leg. All final readbacks confirm hook/monitor cleared and hardware
protection enabled; all eight processes exit normally. The USB control
failure does not recur in this batch, but its cause is still unknown.

This establishes live bidirectional payload and early-search operation
with the reviewed native-9000 path. Observed success is 1/2 at 9600 and
2/2 at 9000; two calls per arm cannot establish a reliability improvement.
The successful E-triggered 9600 call also shows that missing CP′ does not
always prevent LAPM. These are minute-long calls with a small payload,
not soak/full-upstream-rate evidence; the N=4 cap remains part of the setup.
Native 9000 does not select 3000 baud: these calls retain the negotiated
3200-baud/9600-Hz internal T/3 grid (V.90 §5.2, V.34 §5.2).

Retained executable/provenance, raw taps, manifests, events, launch settings,
independent DTE grading (`analyze.py`, `summary.json`) and report:
`reference-data/artifacts/apple-direct-resume-20261018/`. No source-code or
default-setting change was needed after this live comparison.
