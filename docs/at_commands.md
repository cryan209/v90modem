# AT command reference

The DTE interface on the PTY is the supported way to configure the modem. The
interpreter is SpanDSP's (`spandsp-master/src/at_interpreter.c`) with this
tree's additions in `data_interface.c`, `at_ms.c` (`+MS`), `v250_ctl.c` and
`at_test.c` (`+T*` diagnostics), and `fax_class2.c` (T.32). The modem prints
the same reference itself: `AT$` lists the basic commands and the way to every
other page (`ATD$`, `AT&$`, `AT+$`, `AT+MS$`, `ATI$`, `ATS$`). Those pages come
from `at_help.c`, and a drift test checks them against the interpreter, so
**`at_help.c` is the source**: when the two disagree, fix this note.

Many Hayes commands are accepted but change nothing, because the line is a SIP
call and the DTE port is a pty with no rate, DCD, DTR or break. Those rows say
so. A command the modem does not recognise answers ERROR (V.250 5.6).

Related notes: [v250_command_conformance_audit.md](v250_command_conformance_audit.md)
(how each command was checked), [at_stored_profiles.md](at_stored_profiles.md)
(`&W`, `&Y`, `--profile`), [fax_class_at.md](fax_class_at.md) (every `+F`
command), [v54_diagnostics.md](v54_diagnostics.md) (`+T*`),
[carrier_loss.md](carrier_loss.md) (S10), [mnp.md](mnp.md) (`+ES` and MNP),
[v92_quick_connect.md](v92_quick_connect.md) (`+PQC`, `+PSS`). `+MS` and the
`ATI` pages are tabulated in the [top-level readme](../readme.md).

## Common tasks

| To | Send |
|---|---|
| See what the modem supports, live | `AT$`, then `AT+MS$`, `ATS$` and the other pages it names |
| Dial an extension | `ATD6001` (the number goes to `--sip-server`) |
| Answer by hand instead of after two rings | `ATS0=0`, then `ATA` on RING |
| Offer V.34 at most on the next call | `AT+MS=V34` |
| Offer V.92 with PCM upstream | `AT+MS=V92`, `AT+PIG=0` |
| Require V.42 error control | `AT&M5` |
| Turn compression off | `AT&K0` |
| See the rate and protocol before CONNECT | `AT+MR=1;+ER=1;+DR=1` |
| Read why the last call ended, and its rates | `ATI6` (and `ATI11` for training detail) |
| Keep settings across restarts | start with `--profile file`, then `AT&W0` and `AT&Y0` |
| Go back to factory settings | `AT&F` |

## Consoles

`--pty-link` is the classic combined console: commands and data on one port,
`+++` (S2 character, S12 guard time) to escape to online command state, `ATO`
to return. `--control-link` and `--data-link` together give a control console
that always speaks AT (RING, CONNECT, NO CARRIER and mid-call commands) and a
data console carrying payload only; `ATO` on the control console prints the
data PTY's path. With `--lines N`, line *i* uses `<link>.i`
([multi_line.md](multi_line.md)).

## Basic commands (`AT$`)

| Command | Effect here |
|---|---|
| `A` | Answer the ringing call (S0 rings answer by itself) |
| `Dn` | Dial *n* (see below) |
| `E0`/`E1` | Command echo off/on |
| `H0` | Hang up, or end a dial/answer still in progress |
| `H1` | Off-hook command state (no effect on the SIP line) |
| `In` | Identification and diagnostics pages (below) |
| `Y11` | Line spectrum: RX/TX level per 150 Hz band |
| `L0-3`, `M0-3` | Speaker volume/mode: accepted, there is no speaker |
| `O0` | Return to online data (NO CARRIER if there is no call) |
| `P`, `T` | Pulse/tone dial default (SIP sends the digits as they are) |
| `Q0`/`Q1` | Result codes on/off |
| `V0`/`V1` | Numeric/verbose result codes |
| `X0-4` | X0 bare CONNECT; X1 and above CONNECT with the rate; X3/X4 BUSY; X2/X4 NO DIALTONE |
| `Zn` | Reset: factory settings, then profile *n* (0-1) if `&Wn` stored it |
| `Sr=n`, `Sr?` | Set or read S-register *r* |
| `+++` | Escape to online command state |
| `A/` | Repeat the last command line (no Enter needed) |

## Dialling (`ATD$`)

The number is sent to `--sip-server` as `sip:n@server`. Any key aborts a dial
in progress (OK), and S7 bounds it (NO CARRIER). A busy far end gives BUSY
(X3/X4); a network refusal gives NO DIALTONE (X2/X4). `+MS` and `+FCLASS`
choose what the call runs.

| Modifier | Effect |
|---|---|
| `0-9 * #` | Digits, sent as the SIP user part |
| `A-D` | DTMF A-D (tone dialling only) |
| `+` | International prefix, kept in the number |
| `,` | Pause (S8): dropped, a SIP call has no pause |
| `T` / `P` | Tone dialling (default) / pulse dialling (also drops A-D, `*`, `#`) |
| space, `-` | Ignored |
| `W`, `!` | Accepted and ignored |
| `@` | NO ANSWER if the far end does not answer; class 1 fax: no CNG |
| `;` | Class 1 fax: stay in command state; otherwise ignored |
| `S=n`, `Sn` | Dial stored number *n* (`+ASTO` / `&Zn`); the rest of the line is ignored |
| `L` | Redial the last number; `ATDL?` shows it |

Stored numbers last until the modem exits unless `--profile` names a file to
keep them in.

## Ampersand and vendor commands (`AT&$`)

| Command | Effect here |
|---|---|
| `&A0-3` | CONNECT suffix: 0 none, 1 `/ARQ`, 2 adds `/V34`, 3 adds `/LAPM/V42BIS` |
| `&B0-2` | Serial rate fixed/variable: accepted, a pty has no rate |
| `&C0`/`&C1` | DCD behaviour: stored, a pty has no DCD line |
| `&D0-2` | DTR behaviour: stored, a pty has no DTR line |
| `&F0`/`&F1` | Factory defaults (not the `&W` profile); `&F2` is ERROR |
| `&H0`/`&H1` | Transmit flow control none/CTS (`+IFC` DTE-by-DCE) |
| `&I0` | Software flow control off; XON/XOFF (`&I1-5`) is ERROR |
| `&K0-3` | Compression: `&K0` none, `&K1-3` V.42bis (`+DS`, `+DS44`) |
| `&M0`/`&M4`/`&M5` | Error control: none / V.42 or buffered / required (`+ES`) |
| `&R1`/`&R2` | Receive flow control: ignore RTS / RTS (`+IFC` DCE-by-DTE) |
| `&V` | View the active configuration and stored profiles |
| `&Wn` | Store the active configuration as profile *n* (0-1) |
| `&Yn` | Profile restored at power-on (0-1; kept in the `--profile` file) |
| `&Zn=s` | Store number *s* in slot *n* (0-9); `&Zn?` shows it |
| `\N0`/`\N2-4` | Rockwell error control: 0 none, 2 required, 3 auto, 4 LAPM |
| `%C0-2` | Rockwell compression: 0 none, 1/2 V.42bis (MNP 5 is ERROR) |

## Extended commands (`AT+$`)

Each takes the V.250 `=`, `?` and `=?` forms.

| Command | Effect here |
|---|---|
| `+GMI`, `+GMM`, `+GMR` | Manufacturer, model, revision |
| `+GCAP` | Capabilities: `+FCLASS`, `+MS`, `+ES`, `+DS` |
| `+ASTO=n,s` | Store dial string *s* in slot *n* (0-9); `ATDS=n` dials it |
| `+MS` | Modulation for the next call (`AT+MS$`; table in the readme) |
| `+MR=0/1` | Report `+MCR`/`+MRR` before CONNECT |
| `+MSC=0/1` | V.34 seamless rate change (V.34 11.6) on/off |
| `+IPR=n` | Fixed DTE rate; 0 = whatever the DTE sets |
| `+ICF=f,p` | Character framing: 8N1 (0 or 3) only |
| `+IFC=a,b` | Flow control: 0 none, 2 pty back-pressure |
| `+ILRR=0/1` | Report `+ILRR: <DTE rate>` before CONNECT |
| `+ES=o,f,a` | Error control: originator request, fallback, answerer mode (V.250 Table 20; MNP in [mnp.md](mnp.md)) |
| `+ER=0/1` | Report `+ER` (LAPM, ALT, NONE) before CONNECT |
| `+EWIND=t,r` | V.42 window *k* offered per direction (1-15) |
| `+EFRAM=t,r` | V.42 frame size N401 offered (1-128 octets) |
| `+ETBM=0,r,t` | Buffers at call end: TX discarded, RX delivered |
| `+EFCS=0` | 16-bit FCS (32-bit is not offered) |
| `+EB=0,0,0` | Break handling: none (a pty carries no break) |
| `+DS=d,n,s,l` | V.42bis: direction, required, dictionary size, string length |
| `+DS44=d,n,...` | V.44 offer (with V.42bis) |
| `+DR=0/1` | Report `+DR` before CONNECT |
| `+TLDL=0/1` | Local digital loop of the DTE data, during a call |
| `+TTER=t,l,n,p` | Bit/block error test on that loop |
| `+TNUM?` | Error counts of the last test |
| `+TSELF=1`, `+TRES?` | Partial self-test, and its result (0 none, 1 pass, 2 fail) |
| `+TMODE?` | Test mode: point to point (0) only |
| `+TRDL?`, `+TRDLS?`, `+TALS?` | Read as 0. Remote and analogue loops are not wired to the engine, so setting them is ERROR rather than an unconfirmed OK (V.250 6.7.2.14) |
| `+VCID=0/1` | Caller ID (DATE, TIME, NMBR, NAME from the SIP caller) after the first RING |
| `+FCLASS=n` | 0 data, 1/1.0 T.31 fax, 2.0 T.32 fax (2.1 adds V.34 fax) |
| `+FTS`/`+FRS`, `+FTM`/`+FRM`, `+FTH`/`+FRH` | Class 1: silence, image modulation, HDLC |
| `+FDT`/`+FDR`, `+FIS`/`+FCC`, `+FLI`/`+FPI`, `+FNR`/`+FBU`, `+FCT`/`+FIE`, ... | Class 2.0: pages, session parameters, IDs, reports, timeouts ([fax_class_at.md](fax_class_at.md)) |
| `+PMH=0/1` | V.92 modem-on-hold: 0 enabled, 1 disabled (factory) |
| `+PMHT=n` | Grant modem-on-hold with V.92 T1 code *n* (1-13); 0 denies |
| `+PMHR` | Ask the far end for modem-on-hold; `+PMHR: n` follows (ERROR if not armed) |
| `+PIG=0/1` | V.92 PCM upstream: 0 offered, 1 not (factory) |
| `+PCW=1/2` | A second call arriving mid-call: 1 hang up for it, 2 busy (SIP 486) |
| `+PQC=0-3`, `+PSS` | Quick Connect: 0 short Phase 1 and 2, 1 Phase 1 only, 2 Phase 2 only, 3 off (factory); `+PSS=2` forces the full startup |

`+MA` and `+PMHF` are not implemented (ERROR: there is no line to flash).

## Identification and diagnostics (`ATI$`)

`ATI0` product, `ATI3` software version (`git describe`), `ATI4` current
settings, `ATI6` link diagnostics, `ATI7` product configuration, `ATI11`
extended link diagnostics, `ATY11` line level per 150 Hz band. What each page
shows is tabulated in the readme. `ATI6`, `ATI11` and `ATY11` keep the last
call's state after NO CARRIER; during a call, read them on the control console
or after a guarded `+++`.

## S-registers (`ATS$`)

Only these registers exist; any other answers ERROR. Factory values are
SpanDSP's profile, except S0 and S7, which `--auto-answer` and
`--connect-timeout` set.

| Reg | Factory | Function |
|---|---|---|
| S0 | 2 | Rings before answering (0 = `ATA` only) |
| S1 | 0 | Rings counted on this call |
| S2 | 43 (`+`) | Escape character (above 127 disables escape) |
| S3 | 13 (CR) | Command line terminator |
| S4 | 10 (LF) | Response formatting character |
| S5 | 8 (BS) | Command line editing character |
| S6 | 1 | Pause before blind dialling, s (stored only) |
| S7 | 120 | Connection timeout, s (1-255): NO CARRIER if CONNECT is not reached |
| S8 | 5 | Comma pause, s (stored; SIP has no pause) |
| S10 | 14 | Carrier-loss disconnect delay, 1/10 s (V.22bis; 255 = never) |
| S12 | 50 | Escape guard time, 1/50 s (0 = none) |
