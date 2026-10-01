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
against `artifacts/eicon-digital-downstream/`, a byte-exact G.711 capture, and
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

Artifacts: `artifacts/apple-v92-r2` (V.92 negotiated, demoted to V.90) and
`artifacts/apple-v92-r3` (V.92 selected both directions), each with its
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
V.92 call.  `artifacts/apple-v92-sip-r4` (V.92) and `apple-v90-sip-r6` (V.90),
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
`artifacts/apple-v92-sip-r4/server/live-rx.g711` armed at the sample the engine
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
directions" is superseded.**  `artifacts/apple-v90-sip-r7` repeats r6 with
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
`artifacts/apple-v90-sip-r7`, in 32 ms windows on the 16 kHz T/2 grid: the Sd
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
at RMS 3132 and the receive tap reads 226 -- **-22.9 dB** -- and a 96-tap fit
from our transmit tap to our receive tap over the TRN1d era explains nothing
(held-out R² = -0.010).

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
