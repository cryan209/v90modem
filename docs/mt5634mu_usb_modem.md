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

## Repeatable single-call diagnostic

With an idle bridge running and no other DTE holding its slave open:

```bash
python3 tools/mt5634mu_diag.py --number 6311 --out artifacts/mt5634mu-6311-a
python3 tools/mt5634mu_diag.py --number 6312 --out artifacts/mt5634mu-6312-a
```

Use a new output directory for each call. The stdlib-only driver records
monotonic timestamps and exact TX/RX bytes in `transcript.jsonl`, and identity,
active `AT&V` profile, dial result/time and complete `ATI11` in `summary.json`.
It sends `ATM0` before dialling (V.250 (07/2003) 6.3.14, Table 12) to keep the
speaker muted. It sets verbose results/echo but does not reset the modem,
write NVRAM, change modulation/error-control policy, or issue USB requests.
Both pagers are advanced with one space per prompt, including prompts split
across reads. A missing final `OK` stops the run before another command.

The default dial deadline is 80 s; after CONNECT it listens for 5 s without
sending login data, then explicitly escapes/hangs up before collecting ATI11.
On timeout it requests dial cancellation with CR and requires confirmation
before ATH (V.250 5.6.1/6.3.1). Timeout, USB failure or interruption is an
inconclusive run; inspect the modem before redialling. Do not depend on PTY
close to cancel training. Keep the bridge stderr log alongside the results.
Use the same profile and alternate the line order on repeat comparisons;
allow the ATA/line to recover between calls. A `NO DIALTONE` attempt does not
grade either RasFinder port's training.

`python3 tools/mt5634mu_diag_test.py` checks fragmented pager/result parsing,
failed-call diagnostic preservation, CONNECT cleanup, timeout cancellation,
and the BUSY-then-voicemail CDR case below.
It is in `tests/fast.list`; these tests qualify the driver, not hardware.

For a saved headerless Asterisk `cdr-csv/Master.csv` snapshot, this command
does **not** open the PTY or dial:

```bash
python3 tools/mt5634mu_diag.py --number 6311 --cdr-csv /path/to/Master.csv \
    --out artifacts/mt5634mu-route-6311
```

It selects the latest recorded 6004-to-number call and groups its legs by
uniqueid. `voicemail_after_busy` distinguishes the failed modem leg from an
`ANSWERED` voicemail leg; `modem_route_answered` only establishes a PBX answer,
not physical training. Use a fresh snapshot after the call has ended, and
correlate its timestamps/ID with the AT transcript, especially with concurrent
callers. The parser assumes the standard 18-column Master.csv layout.

## 21 October 2026: reproduced failure is BUSY followed by voicemail

The physical MT5634MU originated from the user-confirmed line 6004; 6311,
6312 and 6313 are separate ports on the same RasFinder. No DSP, G.711, gateway
gain/echo settings, NVRAM or USB protocol behavior was changed. `ATM0` was
acknowledged, and every complete call's AT&V confirms Speaker Control Off.

| Attempt | DTE result / dial time | Last-call / PBX evidence |
|---|---|---|
| 6311-c | NO CARRIER / 52.683 s | ATI11 rates/modulation/protocol NA, frames/retrains 0, termination 2. CDR `1792543928.1895`: Dial BUSY immediately, then VoiceMail ANSWERED for 50 s. |
| 6312-a | NO DIALTONE / 14.093 s | No training result; excluded from the port comparison. |
| 6312-b | CONNECT 33600 V42bis / 31.906 s | ATI11 V.34 33600/33600, LAPM, V42bis, 0 retrains/errors, round trip 358 ms; termination 0 after our ATH. CDR `1792544047.1899`: modem Dial leg ANSWERED, 35 s. |
| 6311-d | NO CARRIER / 56.420 s | Same NA/zero/termination-2 diagnostic. CDR `1792544136.1902`: immediate Dial BUSY, then 50 s VoiceMail ANSWERED. Live channel snapshot also shows VoiceMail, not AppDial. |

The repeated 6311 failures therefore occur before a modem-to-modem handshake:
the destination route is busy and the caller reaches voicemail. The long
wait is not evidence of failed V.34 equalization, LAPM or a USB byte-path
defect. The active profiles match across both lines, and 6312 establishes a
physical/error-control link through the same bridge. Its five-second passive
DATA window contained no login banner; this does not qualify payload transfer.

This identifies the immediate cause of **these calls**, not why the 6311 port
reports busy, nor the cause of the 19 October termination-3 attempt. An earlier
attempt here lost the USB device (`e00002ed`, device not responding) about
3 s after dialling and is excluded. Its corresponding CDR shows a 0-second
answered 6311 leg; a stuck port after that interruption is a hypothesis, not
established evidence. Check the 6311 gateway/line's hook/busy state before
another training investigation; registration/OPTIONS availability alone does
not establish line availability. Do not change DSP or disable dial-tone
detection to conceal these outcomes.

Retained local evidence: `reference-data/artifacts/mt5634mu-20261021/`
(exact AT transcripts, summaries, bridge logs/counters, filtered CDRs and PBX
route/channel snapshots; `manifest.json` hashes the files). No packet capture
was taken: automatic approval review rejected the proposed gateway-wide
SIP/RTP capture because it could include other calls' sensitive payloads;
the diagnosis uses modem diagnostics and PBX call metadata instead.

Validation: bridge builds with `make mt5634mu_bridge`; the fast-tier diagnostic
row passes all seven hardware-free tests. The live calls above establish the
observed routing and V.34/LAPM results only, not V.90 interoperation.

### Follow-up: 6314

At the user's request, 6004 dialled 6314 with the same active profile and
`ATM0` confirmed. Result: `NO CARRIER` after 54.820 s. ATI11 shows modulation,
rates and protocol NA, zero frames/retrains, termination cause 2. Asterisk
CDR `1792544500.1911` shows a **20-second NO ANSWER** on the 6314 Dial leg,
followed by 30 seconds in VoiceMail; the live channel snapshot confirms
VoiceMail. Unlike 6311's immediate BUSY, this route rings without answering.
No modem handshake or LAPM connection was established.

Evidence is retained in the same directory under `6314-a/`, `route-6314/`,
`bridge3.log`, `cdr-6314.csv` and `pbx-6314-live.txt`. The diagnostic now
accepts all four gateway extensions 6311–6314. The seven hardware-free tests
still pass; no modem or gateway settings other than the documented volatile
speaker/result settings were changed.
