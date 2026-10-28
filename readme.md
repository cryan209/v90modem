# SIP V.90 Modem

A softmodem that answers and places modem calls over SIP. The RTP payload of a
G.711 call (PCMU or PCMA) carries exactly what a telephone network's digital
DS0 channel carries, so this program can be the **digital side of a V.90/V.92
call**: an ordinary analogue modem dials in through an FXS port, and we train,
send PCM codewords downstream, and bridge the data to a PTY.

It also runs V.34, V.32bis, V.22bis, V.21, V.23 and Bell 103/212A, the V.90
analogue role, V.91, clear channel (V.110/V.120), x2, K56flex, and fax (T.31
class 1, T.32 class 2.0, V.34 half-duplex). DSP is native where the standards
need it, with vendored SpanDSP and PJSIP for the rest.

**Documentation:** [user guide](docs/usage.md) · [docs index](docs/README.md) ·
[AT command reference](docs/at_commands.md) · [tools and make targets](tools/README.md) ·
[environment variables](docs/env_knobs.md)

## Quick start

Install the [build dependencies](#building) first. From the repository root:

```bash
make sip_v90_modem
# Terminal 1: keep the modem running
./sip_v90_modem --sip-server pbx.example.net \
                --username modem --password 'your-password' \
                --pty-link /tmp/v90modem
```

In a second terminal, open the command/data port (picocom is optional):

```bash
picocom --baud 115200 --flow n /tmp/v90modem
```

Type `AT` then Enter; expect `OK`. `ATD6001` dials extension 6001 through your
PBX. Incoming calls are answered after two rings (`ATS0=0` selects manual
`ATA`). After `CONNECT`, the PTY carries data rather than commands. To hang up,
wait one second, type `+++` **without Enter**, wait another second for `OK`,
then send `ATH`. Exit picocom with Ctrl-A, Ctrl-X; stop the modem with Ctrl-C
in terminal 1.

The PTY is not a login or PPP server: an application must handle the payload.
See the [user guide](docs/usage.md) for a full first-call walkthrough, split
control/data ports, saving settings and troubleshooting. `AT$` lists AT help;
`./sip_v90_modem --help` lists launch options. Password arguments may be visible
in shell history and process listings; use a dedicated test account.

To explore without a SIP account or hardware:

```bash
./sip_v90_modem --gui-loopback
# Browser alternative: ./sip_v90_modem --gui-web --loopback
```

Local loopback demonstrates the modem but does not validate a real phone path.

## Status (26 October 2026)

Data connections work, but this is an interoperability project, not a
qualified replacement for every hardware modem. A loopback pass or a CONNECT
message alone does not show hardware compatibility, so each row says what the
evidence is.

| Area | Evidence | Still open |
|---|---|---|
| **V.90 analogue caller** | Eicon Diva hardware: 56000 down / 31200 up, both laws, LAPM/V.42bis; 26-minute transfers, PPP and HTTP | Other peers and networks |
| **V.90 digital answerer** (the main goal) | Byte-exact PCM downstream; reaches data mode live against the RasFinder's analogue modems | Upstream can still lose sync mid-call; sustained transfers across chipsets |
| **V.92** | slmodemd dialling us: 36000 up (V.92 6.4.2 precoder) / 53333 down, clean for 170 s; Quick Connect in engine tests | A hardware V.92 peer; Quick Connect against a real modem |
| **V.34** | Eicon hardware both laws; 35-minute soaks to extension 7900 (400/400 lines) and PPP to 7910 (900/900 pings) | Sustained transfers as answerer; an occasional retrain against 7910 |
| **V.32bis, V.22bis, V.21, V.23, Bell 103/212A** | Engine-pair payload tests; V.22bis rides out short carrier drops (found against a CX93001) | Broad hardware coverage |
| **Error control** | V.42/LAPM, V.42bis, V.44; MNP 2-4 connected on 8 of 9 live calls | MNP 5 compression is not implemented |
| **Fax** | Eicon pages both ways, both laws, up to 28800 bit/s | 33600 (class 2.1) on hardware |
| **V.91, clear channel, V.110/V.120** | Two of our own endpoints over SIP | A foreign digital peer and a byte-exact bearer |
| **x2** | Server and analogue client, with LAPM, against original Courier firmware in an emulator | Real hardware |
| **K56flex** | Server: duplex data against a native HSF client; V.8bis with a real server (Eicon 7910) | Hardware data connection; a client for real servers |

Headline result: on 8 October, two unbroken V.90 calls to the Eicon card ran
about 26.5 minutes each and checked **16,177,152 payload bytes** with no
sequence, CRC or payload errors. Both laws passed PPP (30/30 pings) and exact
1 MiB HTTP transfers. Measured application rates on that LAN path, with a
40 ms jitter buffer, were 6.44 kB/s binary and 6.21 kB/s HTTP.

Where the evidence lives: [hardware procedure and results](docs/v90_hardware_interop.md),
[Eicon soaks](docs/eicon_soak_ppp.md), [RasFinder](docs/v90_rasfinder_phase4.md),
[V.92](docs/v92_full_startup.md), [MNP](docs/mnp.md),
[fax](docs/eicon_v34_fax_20261009.md), [x2](docs/x2_implementation.md),
[K56flex](docs/k56flex_implementation.md), and the open list in the
[gap analysis](docs/gap_analysis_2026-10.md).

## What the line must provide

V.90 downstream works only if the codewords we send reach the far end's D/A
converter unchanged. Everything between us and the FXS port must therefore:

- carry G.711 (PCMU or PCMA, the right law) with **no transcoding**;
- have no VAD or comfort noise, no echo cancellation, no gain or noise
  suppression;
- use a minimal, fixed jitter buffer (ours defaults to 200 ms, `ME_JB_MS`);
- end at an FXS port with a real D/A converter.

`make g711-path-test` probes a PBX path for byte-exact G.711. It is a live
network test, not a self-contained check: it needs an Answer()+Echo() extension
and account details passed through `G711_TEST_ARGS`. See the
[tool reference](tools/README.md#make-targets-outside-make-test).

## Configuration

The AT commands on the PTY are the supported interface: `AT+MS` chooses the
modulation for the next call, `AT+ES`/`AT+DS` error control and compression,
`ATS0` auto-answer. All are in the [AT command reference](docs/at_commands.md).

Start-up options set the power-on state:

| Option | Effect |
|---|---|
| `--sip-server`, `--username`, `--password` | Registrar and account; dialled numbers go to `sip:n@server`. Omit the server for peer-to-peer listening |
| `--no-register` | Keep outbound authentication without registering a new Contact; not the usual incoming-call setup |
| `--local-port`, `--rtp-port`, `--bind-addr` | Local SIP port, RTP port, address |
| `--pty-link path` | One console for commands and data (`+++` / `ATO`) |
| `--control-link path --data-link path` | A command console that always speaks AT, and a data-only port |
| `--mode name` | Power-on modulation offer (default `v90`): `v21`, `b103`, `b212`, `v23c`, `v22`, `v22-1200`, `v32`, `v32bis`, `v34`, `v90`, `v92`, `v91`, `k56`, `x2`, `x2-symm`, `clear`, `clear56`, `v120`, `v120-56`, `v110` |
| `--auto-answer rings`, `--connect-timeout s` | Factory S0 (default 2) and S7 (default 120 s) |
| `--profile file` | Keep `AT&W` profiles and stored numbers across restarts ([details](docs/at_stored_profiles.md)) |
| `--lines N [--recycle]` | N lines, one worker process each ([multi-line](docs/multi_line.md)) |
| `--gui`, `--gui-web`, `--gui-loopback` | Live signal monitor (below) |
| `--verbose` | Detailed engine log |

`--mode v90` also offers V.34, V.32bis and V.22bis as fallbacks, and V.32bis Annex A
automode meets a modem that has no V.8. `ME_MODE` takes the same names as
`--mode`. Most other `ME_*`, `V34_*` and `V90_*` environment variables are
diagnostics or experiments, not settings; [env_knobs.md](docs/env_knobs.md)
lists the few that are settings.

## How it fits together

```
 analogue modem ── FXS gateway ── SIP / RTP (G.711, untouched) ──┐
                                                                 │
 ┌───────────────────────────── sip_v90_modem ───────────────────▼────┐
 │  sip_modem.c        PJSIP: registration, calls, G.711 passthrough  │
 │  clock_recovery.c   fixed jitter buffer, exact 8000 Hz accounting  │
 │  modem_engine.c     call state, V.8/V.8bis, choice of datapump:    │
 │                       V.90/V.92 (vpcm_*, v90*, v92*), V.34 and     │
 │                       older (SpanDSP), x2, K56flex, clear channel, │
 │                       fax                                          │
 │  data_stack.c       V.14, V.42/LAPM, V.42bis, V.44; MNP (mnp.c)    │
 │  data_interface.c   AT interpreter and the PTY consoles            │
 └────────────────────────────────────────────────────────────────────┘
```

Each source file's header comment states its role and the spec clauses it
implements. `vpcm_*.c` is the layer V.90 and V.92 share. Not every decoder in
the tree is on the live path: `SRCS` in the `makefile` is the list that is.

## Building

`make` builds `sip_v90_modem` and the test binaries. SpanDSP
(`spandsp-master/`) and PJSIP (`pjproject/`) are vendored and always used; system
copies are not substitutes. Both are rebuilt automatically when the host OS or
architecture changes.

- **macOS:** install Apple's command-line tools (`xcode-select --install`) and
  Homebrew, then:

  ```bash
  brew install autoconf automake libtool pkg-config libtiff jpeg-turbo openssl@3 picocom
  make
  ```

  The makefile finds Homebrew itself. The vendored PJSIP build selects its own
  architecture suffix; no `ARCH_SUFFIX` override is needed.
- **Debian/Ubuntu:**

  ```bash
  sudo apt install build-essential autoconf automake libtool pkg-config \
                   libasound2-dev libssl-dev libopus-dev libtiff-dev \
                   libjpeg-dev uuid-dev python3 picocom \
                   libavformat-dev libavcodec-dev libswscale-dev \
                   libavutil-dev libv4l-dev
  make
  ```

`picocom` is only a terminal client; another serial terminal can be used.
For just the application, use `make sip_v90_modem`. Python 3 runs the GUI
launcher and test tools. See the [user guide](docs/usage.md#2-build) for build
checks and first-use steps.

Python analysis tools use a repo-local virtualenv:

```bash
python3 -m venv .venv
./.venv/bin/python -m pip install -r requirements-tools.txt
```

## Testing

| Command | What it runs |
|---|---|
| `make test-fast` | The quick tier ([tests/fast.list](tests/fast.list)), about 45 s |
| `make test-slow` | Long matrices ([tests/slow.list](tests/slow.list)) |
| `make test` | Both fast and slow tiers |
| `make pcm-loopback-test`, `make pcm-matrix` | V.90/V.92 PCM datapump, startup and recovery checks; all-profile matrix ([notes](docs/pcm_loopback_testing.md)) |
| `make v56-test`, `make v56-sweep` | V.34 modem pairs through synthetic noise, loss, delay and echo ([notes](docs/v56_loopback.md)) |
| `make v34-tone-matrix` | Score recorded modem tones for INFO0a, INFO1a and Phase 3 |
| `make x2-test x2-session-test x2-sym-test` | x2 components and sessions |

The runner is parallel, gives each test its own temporary directory, and keeps
the log of every failure in `test-logs/`. These are offline measurements; they
say nothing about hardware interoperability. Other targets are listed in
[tools/README.md](tools/README.md#make-targets-outside-make-test).

## Diagnostics

**Live GUI.** `--gui` opens a native macOS console, `--gui-web` the same in a
local browser. It shows RX/TX waveforms, constellations, eye diagrams, the
datapump and DS0 bytes, the AT console and training transitions, all held in
memory. `--gui-loopback` runs a local echo pair with V.21, V.22/V.32/V.34,
V.90/V.91 and V.92 profiles. See [GUI mode](docs/gui_mode.md).

**After a call.** `ATI6` shows the link (rates, protocol, octets, line level,
why it ended), `ATI11` the training detail, `ATY11` the line spectrum.

**Raw G.711 taps.** Set `VPCM_G711_TAP_DIR` to an existing directory to record
the bearer as `live-rx.g711` and `live-tx.g711`:

```bash
mkdir -p /tmp/v90-g711
VPCM_G711_TAP_DIR=/tmp/v90-g711 ./sip_v90_modem ...
```

**A recorded hardware run.** `tools/v90_hardware_interop.py` runs one bounded
call and keeps the log, both taps, hashes, the build revision and a parsed
timeline under `reference-data/artifacts/v90-hardware/` (passwords are
redacted; `--dry-run` checks the command):

```bash
./tools/v90_hardware_interop.py --label "Courier-V-Everything" --duration 180 -- \
  --sip-server asterisk.example --username 6001 --password 'secret' \
  --pty-link /tmp/v90modem
```

**Eicon soaks.** `tools/eicon_soak_test.py` runs timed download, upload and
simultaneous transfers, or PPP with pings and HTTP, against the Eicon answerer.
Run it from the wired Tower LAN ([setup](docs/eicon_soak_ppp.md)).

**Without G.711 passthrough.** If the local pjproject lacks passthrough codecs,
the modem falls back to a linear PCM bridge (the log says `falling back to
linear PCM bridge`). That is good enough for softmodem debugging, not for V.90.
`./scripts/softmodem_debug.sh` builds and launches in that mode
(`--log-file`, `--no-build`, `--skip-preflight`).

## License

GPL-2.0 (SpanDSP is LGPL; linmodem heritage is GPL).
