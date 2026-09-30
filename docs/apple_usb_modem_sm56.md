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

**These captures were taken with NOTHING PLUGGED INTO THE TEL JACK, and that
bounds what they can say.**  With no line there is no loop current, so a DAA
line-sense output sitting at a rail is just what a disconnected DAA looks like.
The rail is therefore NOT evidence that a bring-up command is needed: "the
analogue path is unpowered until told otherwise" and "there is no line" predict
the same thing, and this setup cannot separate them.  Nor is a noise floor a
meaningful thing to ask for here - with no line the best it could describe is
the codec's own floor, and only once the path is powered at all.  Repeat these
measurements with a line connected before drawing anything from them.

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

**Not established: what any opcode means.**  Naming them requires tracing the
IOCTL call sites in `USmSerial.sys`; all eight constants above appear there,
once each, so the callers are findable.  The `wIndex = 0` requests (0x11,
0x13, 0x14) have never been sent live — the sweep only tried `wIndex = 1`.

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
the regulatory configuration the command channel presumably exists to deliver.
**None of them is 3 or 9 bytes**, so they are not carried by the commands
tabulated above; the 9-byte form acting as a windowed write is a guess, not a
finding.

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

## Open

- What the opcodes do.  Trace the eight IOCTLs into `USmSerial.sys`.
- How the country blobs reach the device.
- Whether the DFU interface expects a firmware push.
- Whether anything must be sent before the codec stops railing.  UNTESTABLE as
  measured: the captures above had no line connected, so the rail has a trivial
  explanation and the question needs a line in the jack first.
- The `wIndex = 0` requests (`bRequest` 0x11/0x13/0x14), untested live.
- Whether `0x220070` (tested by the dispatcher but reaching no request block
  above) does something else.
