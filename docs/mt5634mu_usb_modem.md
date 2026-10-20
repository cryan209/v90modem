# MultiTech MT5634MU USB modem (rig)

A real analogue peer from a fourth chip family (Lucent), next to the CX93001
and the Intel serial modem on ext 8411.

| | |
|---|---|
| USB | `06e0:f103`, product "MultiModemUSB", USB 1.0 full speed |
| `ATI3` | `LT V.90 1.0 MT5634MU USB Data/Fax Modem Version 8.18j` |
| `AT+GMR` | `U052099f` |
| Modes | V.90 data, `+FCLASS=0,1,2`; no `AT+MS`, `+GCAP` or `+GCI` |
| Line | Has dial tone. Dialling 6312 (RasFinder) connects V.34 + LAPM + V.42bis. 6311 gave `NO CARRIER` on 2026-10-19 |

## No serial port on macOS: use the bridge

Configuration 2 claims to be CDC-ACM, but the firmware stalls every CDC class
request and has no EP 0x86. Forcing it makes `/dev/cu.usbmodem*` appear,
but the replies arrive on EP 0x84, where AppleUSBACMControl swallows them, so
the tty looks dead. Configuration 1 is vendor-specific and works. Interface 1
has bulk-OUT EP 0x02 (16-byte packets) for host-to-modem bytes and
interrupt-IN EP 0x84 (63-byte packets) for modem-to-host bytes. Each IN packet
is one status byte followed by payload.

`tools/mt5634mu_bridge.c` seizes the device, selects configuration 1, and
exposes it as a PTY:

```bash
make mt5634mu_bridge
```

```bash
./mt5634mu_bridge --pty-link /tmp/mt5634mu
```

```bash
minicom -D /tmp/mt5634mu
```

- It prints the PTY path on stdout and logs to stderr. `-v` logs pipe setup
  and DTE open/close, and `-v -v` hex-dumps every transfer.
- **Hangup.** A PTY has no DTR line, so a DTE's DTR drop cannot be seen. When
  the last process closes the slave and there is carrier, the bridge sends a
  1 s guard, `+++`, another 1 s guard, then `ATH\r`, which behaves like
  `&D2`. Verified on a live call: carrier dropped 2.4 s after the close.
  `--no-hangup-on-close` turns this off. Output that the modem sends while no
  DTE has the port open is dropped.
- Speed settings on the PTY are ignored. The USB link has no baud rate, and the
  modem flow-controls the host by NAKing bulk-OUT.
- `tools/cx_at.py` needs pyserial, which this Mac's Python does not have.
  Install it in a venv to use that script with the bridge.
- `ATI11` and `AT&V` open a "Press any key" pager, and the pager eats the next
  command.

Verified 2026-10-19: `AT` returns `OK` and `ATI3` returns the string above. In
a 30 s loop, 3441 back-to-back `ATI3` commands returned 3441 byte-exact
replies. The bridge's byte counters matched the DTE side in both directions,
with no dropped or duplicated bytes.

## Bridge implementation limits

- macOS-only IOKit/CoreFoundation tool, built explicitly with
  `make mt5634mu_bridge`; not part of the hardware-free test tier.
  `make clean` removes the executable.
- Interrupt-IN uses blocking `ReadPipe` on a dedicated thread:
  `ReadPipeTO` returns `kIOReturnBadArgument` (`e00002c2`) for this pipe.
  Shutdown aborts the pipe before joining the reader.
- Carrier polling runs every 125 ms and accepts only an exactly two-byte
  reply repeated twice, because EP0 can return stale data around hangup.
  Unknown carrier is treated conservatively: close still attempts hangup.
- The escape guards are 1100 ms each. Hangup assumes the modem still uses
  compatible escape-character/guard-time settings; it is not a hardware DTR
  implementation. It triggers on a detected slave open-to-closed transition,
  not merely on changing a DTE's termios modem-control settings. Do not rely
  on it to cancel dialling when the carrier poll already reports off.
- The bridge retries a timed-out bulk-OUT transfer after clearing the stall.
  The AT stress result above does not establish byte-exact recovery from
  partial USB transfer timeouts, unplug/replug, or sustained binary DATA.
- Byte counters are logged on exit (there is no implemented `--stats`
  option). The receive counter counts bytes delivered to the PTY, excluding
  packet status bytes and output discarded while no DTE is open.
- Stopping the bridge closes USB but does not explicitly send `ATH` on
  process exit; close the DTE first if a connected call must be hung up.

These results establish AT transport and observed V.34 calls to the
RasFinder, not V.90/K56flex interoperability with our digital modem.

## Modem status and vendor control requests

The status byte at the start of each interrupt-IN packet is **not** modem
status. It stayed `0xc0` through off-hook, dialling, three V.34 calls to the
RasFinder on 6312 (CONNECT 33600/26400/21600, LAPM, V.42bis, login prompt)
and hangup. The bridge still logs any change it sees.

The modem's state is exposed as vendor control requests on EP0, sent with
wIndex 1 (interface 1): type `0xc1` for IN and `0x41` for OUT. They work while
the bridge holds the device, so a second process can read them alongside a
call. These were found by probing on 2026-10-19 and are all that responded
(IN 0-63 with wValue 0-3; OUT 0-31 zero-length).

| Request | Dir | What it is |
|---|---|---|
| `0x00` | OUT, wValue 0/1/2 (3+ stall) | **Data-path gate, kept in NVRAM.** At 0 the modem accepts one 16-byte write, never processes it, then NAKs bulk-OUT forever. This survives USB reset, re-enumeration and unplugging. Writing 1 drains the stuck bytes and restores AT. The bridge writes 1 at startup. |
| `0x1a` | IN, wLength 4 | 2 bytes. Bit 0 = **no carrier**: `01 00` idle, `00 00` from ~60 ms before `CONNECT` until the line drops. It does not follow `&C` and stays `01` off-hook while dialling. wLength 2 times out (firmware quirk). The bridge polls it and logs `carrier ON/off`. |
| `0x1b` | IN, up to 254 | The live S-register file, S0..S253 (`ATS10=21` shows up straight away). Recipient and wValue are ignored. |
| `0x0f` | IN, 100 | A Windows `COMMPROP` block: wPacketLength 100, MaxTxQueue 3584, MaxRxQueue 2448, MaxBaud 115200, ProvSubType 6 (`PST_MODEM`), ProvCapabilities 0x13f. |
| `0x0e` | IN, 6 | A Windows `SERIAL_CHARS` block (`00 00 00 00 11 13`, so XON 0x11 and XOFF 0x13). |
| `0x14` | IN, 16 | 16 zero bytes, which is the size of `SERIAL_HANDFLOW`. It never changed. |
| `0x02` | IN, 2 | Echoes the setup packet's wValue. It carries no state. |
| others | OUT, zero-length | Accepted with no visible effect: 1, 3, 5, 6, 7, 9, 10, 11, 13, 17 (wValue 0 only), 18, 28. Stalled: 2, 4, 8, 12, 14-16, 19-27, 29-31. Some of the stalled ones probably expect a data stage. |

So the firmware looks like a Windows serial-port model carried over USB
(`COMMPROP`, `SERIAL_CHARS`, handflow), but its request numbers are not the
Windows IOCTL function codes. No DSR, CTS or RI source has been found. Ringing
the modem from another line would show whether RI appears in `0x1a`.

**Do not sweep OUT requests on this modem.** A blind zero-length sweep
cleared `0x00`, stored it, and left the modem deaf even after a power cycle. It
also seems to have saved the active profile (S22, S45, S56 and S57 survived
the replug). Recovery was OUT `0x00` wValue 1. Any bytes sent before that
are executed once the path reopens; here that included a queued
`AT&F&W`, so the stored profile is now factory defaults.

`ATI11` (two pages of a pager) gives last-call diagnostics: modulation, rates,
LAPM/V.42bis, levels, echo, retrains, and the termination cause (0 after a
normal call, 3 after a 62 s `NO CARRIER` on 6311).
