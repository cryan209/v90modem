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

and the mirror at `0x9db43` with `and eax, 0xfffffff7`.  So on `05ac:1401` the
line is seized by a **read-modify-write of bit 3 of register 5**, not by a DAA
relay word and not by the `wIndex = 0` requests.  Neither function has a direct
caller; they are reached through a 10-byte-stride pointer table at file offset
`0x8d673`, the driver's hardware abstraction layer.

**Which direction is which is a reading, not a proof.**  The setting side also
frees the pending buffers and arms a ~10 ms timer with callback `0x9d1e3`, and
off-hook is the transition that needs a settle delay; the clearing side does
nothing but the write.  The codec A/B above agrees - setting the bit is what
brings the analogue front end to life - but with no line connected neither loop
current nor a dial tone has been observed, so "bit 3 set = off-hook" rests on
the settle timer and the codec, not on a seized line.

### The country blobs reach it the same way

`usm56.reg`'s 4-byte `HardwareInitBB` is no longer a mystery.  At `0x9dd96`
there is a table at `0xde218` of 8-byte entries - `{u32 country_id, u8 r10,
u8 r1a, u8 r1f, u8 r1e}`, `-1` terminated, default `00 C0 00` - walked for the
current country and then written as four `WriteReg` calls to registers
**0x10, 0x1a, 0x1f, 0x1e**.  Four bytes, four registers.

That also disposes of the objection that none of the blobs is 3 or 9 bytes long
and so cannot be carried by these commands: the channel does arbitrary
single-register writes, and a blob is delivered as a run of them.  The three
larger blobs (32, 500, 229 bytes) are still unplaced.

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
10. **An IOCTL number does not name a request.**  `utlamot.sys` carries the
   IOCTL through a work queue into `FUN_0001316a`, which is where `bRequest`,
   `wValue`, `wIndex` and the length are chosen, and one IOCTL can have two
   sub-forms keyed on the body's first byte (`0x2200c4` does).  Reading the
   caller's payload and assuming it reaches the wire unchanged is what left the
   first table's fields unnamed.

## Open

- **A line in the jack.**  Everything below the codec is now reachable, and
  nothing further can be settled without loop current: whether bit 3 really
  seizes the line, what registers 1, 2, 0x11 and 0x1f mean (they are the only
  ones with non-zero contents, so ring and loop sense are likely among them),
  and what the codec's floor is on a real bearer.
- What the remaining opcodes do: `0x10|n`, the 9-byte `0x02` form, `0x90`,
  `0xd0`.
- How the 32, 500 and 229-byte country blobs reach the device.  Runs of
  register writes is the obvious guess, and the register file is now readable,
  so a diff of it across a country change would show it.
- Whether the DFU interface expects a firmware push.
- The `wIndex = 0` requests (`bRequest` 0x11/0x13/0x14), untested live.
- Whether `0x220070` (tested by the dispatcher but reaching no request block
  above) does something else.
