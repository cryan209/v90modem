# SIP V.90 Modem

A standalone SIP/G.711 softmodem built on vendored PJSIP and SpanDSP,
with native modem DSP, an AT command interface, and PTY/TCP data connections.
Its primary goal is the **digital answerer** in a V.90/V.92 connection:
an analogue modem dials in over a transparent G.711 bearer. It also implements
an analogue caller, lower-speed modem families, digital bearers and fax.

## Implementation status (26 October 2026)

This is an active interoperability project with working data connections,
not a fully qualified replacement for every hardware modem. Completion varies
by role and peer; a loopback pass or CONNECT message alone does not establish
hardware compatibility. The strongest sustained hardware evidence currently
comes from **our analogue V.90 caller against an Eicon Diva digital answerer**.
The [documentation index](docs/README.md) lists every topic note.

| Area | Implemented and demonstrated | Qualification still needed |
|---|---|---|
| V.90 analogue caller | Eicon hardware: 56000 downstream / 31200 upstream, LAPM/V.42bis, both PCMU and PCMA; checked sustained transfers, PPP and HTTP | Broader peer/network coverage; emulator training results have separate limitations |
| V.90 digital answerer (primary goal) | Native training, byte-exact PCM downstream, V.34 upstream and directed PCM/startup regressions | Repeatable analogue-client/FXS hardware qualification and sustained bidirectional transfers across chipsets |
| V.92 | PCM-upstream datapump, Phase 3/4 codecs and procedure tests; slmodemd dialling us over SIP links at V.92 32000 up / 56000 down with payload both ways (24 of 30 calls clean for a 150 s hold); Quick Connect: short Phase 1 (digital answerer and analogue caller, over V.8 and V.8bis) and short Phase 2 (`AT+PQC`, engine-level tests) | A hardware V.92 peer (the Eicon card answers a V.92 offer as V.90), remaining timing/trellis/recovery integration, Quick Connect against a real modem, and sustained payload qualification |
| V.34 | Full-duplex data and impaired-line tests; Eicon hardware checked echo in both laws at 31200 upstream / 21600 downstream; as caller, ~35-minute echo soaks against Eicon extension 7900 (V.34 and V.90, 400/400 lines) and PPP to 7910 (V.34 21600-24000, 900/900 pings) | Native V.34 sustained-transfer coverage comparable to the V.90 soaks; an occasional Phase 3 stall against 7910 still costs one retrain |
| V.32/V.32bis, V.22/V.22bis, V.21, V.23, Bell 103/212A | Datapumps, engine-pair payload tests and legacy automode; V.22bis holds the trained datapump through a short carrier drop (S10), found against a CX93001 | Broad hardware compatibility coverage |
| Error control | V.42/LAPM, V.42bis, V.44; MNP 2-4 (V.42 Annex A) when detection finds no LAPM peer: 8 of 9 live calls to the RasFinder's MultiTech modems connected with MNP | MNP 5 compression is not implemented |
| V.91, clear channel, V.110/V.120 | Digital bearer implementations and local/engine-pair tests | Foreign digital-peer qualification; a byte-exact compatible bearer is required |
| Fax | T.31/T.32 interfaces and V.34 half-duplex; Eicon pages verified in both directions/laws up to 28800 bit/s | Class 2.1 33600 has local SIP/RTP coverage; hardware qualification at that rate and broader peers remain |
| x2 | Server and analogue client; LAPM qualified against emulated peers in both roles (original Courier firmware as client, Ie030002 as server); symmetric engine-pair support | Real hardware and sustained qualification; at the higher rate indices the emulated Courier corrupts payload that our transmit tap decodes byte-exact |
| K56flex | Server: raw duplex connection against a native HSF client over PCMU (64 KiB exact each way in two minutes); first V.8bis exchanges with real hardware (Eicon 7910) | Hardware interoperability, in-data renegotiation, and a client for real servers (K56flex training against the 7910 has not yet completed) |
| Application | SIP calling/registration, AT control, PTY/TCP, diagnostics, GUI and supervised multi-line workers | Each line uses a separate worker process; interoperability depends on the bearer and selected mode |

On 8 October, two uninterrupted Eicon V.90 calls lasted about 26 minutes
37 seconds each and checked **16,177,152 payload bytes** with zero sequence,
CRC or exact-payload errors. Both laws also passed PPP with 30/30 ping replies
and exact 1 MiB HTTP downloads/uploads. A separate short PCMU LAN probe with
40 ms fixed jitter buffering measured 6.44 kB/s binary download; its HTTP
check measured 6.21 kB/s. These are measured application rates on that path,
not universal throughput guarantees or digital-answerer validation.

Evidence and limitations: [hardware interoperability](docs/v90_hardware_interop.md),
[Eicon soak/PPP/HTTP](docs/eicon_soak_ppp.md),
[PCM regression boundaries](docs/pcm_loopback_testing.md),
[V.92 remaining work](docs/v92_phase4_implementation.md),
[fax hardware results](docs/eicon_v34_fax_20261009.md),
[x2 status](docs/x2_implementation.md) and
[K56flex status](docs/k56flex_implementation.md).

## Build and test coverage

Use the vendored `spandsp-master/` and `pjproject/` trees. `make` builds the
application and test binaries; `make test-fast` runs the quick tier,
`make test-slow` the longer matrices, and `make test` both. The parallel runner
isolates each test's temporary files and retains failure logs in `test-logs/`.
The test definitions are [fast.list](tests/fast.list) and
[slow.list](tests/slow.list). Offline tests measure protocol/datapump behavior;
the hardware evidence above is a separate qualification layer.

## Live GUI

Run `./sip_v90_modem --gui` with your usual SIP options for the native macOS
console, or use `--gui-web` for a local browser. It shows live RX/TX waveforms,
QAM/TCM constellations, adjustable line-derived eyes, separate datapump/DS0 byte
views, AT control, serial payload, and training transitions. Histories are
bounded in memory; no diagnostic recordings accumulate on disk.
Use `./sip_v90_modem --gui-loopback` for a self-contained local echo pair,
with 300-baud V.21, V.22/V.32/V.34, V.90/V.91, V.92 PCM upstream
profiles and a 256-byte test pattern.
See [GUI mode](docs/gui_mode.md) for launch options, tap locations and plot limits.

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                   SIP V.90 Modem Server                 │
│                                                         │
│  ┌──────────┐   ┌────────────┐   ┌──────────────────┐  │
│  │  PJSIP   │   │   Clock    │   │    spandsp       │  │
│  │  SIP UA   │──▶│  Recovery  │──▶│  V.90 Digital    │  │
│  │  G.711   │   │  & Jitter  │   │  Modem Engine    │  │
│  │  μ-law   │◀──│  Buffer    │◀──│  (V.8+V.90+V.34) │  │
│  └──────────┘   └────────────┘   └────────┬─────────┘  │
│                                           │             │
│                                   ┌───────▼─────────┐  │
│                                   │  Data Interface  │  │
│                                   │  PTY / TCP / AT  │  │
│                                   └─────────────────┘  │
└─────────────────────────────────────────────────────────┘
          │ SIP/RTP (G.711 μ-law)
          ▼
┌─────────────────┐        PSTN / SIP        ┌──────────────┐
│  SIP Provider /  │◀───────────────────────▶│  Analog V.90  │
│  FXS Gateway     │                          │  Client Modem │
└─────────────────┘                          └──────────────┘
```

## How V.90 Works Over SIP

V.90 downstream (56k) relies on the server being on the digital side of the network,
sending carefully chosen PCM codewords. With SIP using G.711 μ-law (PCMU), the RTP
payload IS the μ-law PCM stream — exactly the same encoding as a T1 DS0 channel.

The key insight: if the SIP path is transparent G.711 with no transcoding, the
digital modem can select specific μ-law codeword levels that the analog client modem
can resolve after a single D/A conversion at the far-end ATA/FXS port.

### Requirements for V.90 to work:
- G.711 PCMU or PCMA with the correct law selected, no transcoding anywhere in the path
- No voice activity detection (VAD) or comfort noise generation (CNG)
- No echo cancellation on the SIP path
- Minimal, fixed jitter buffering
- The far-end analog modem connects through an FXS port with a real D/A converter

## Components

### `sip_modem.c` — Main application
- Initializes PJSIP stack and spandsp modem engine
- Registers with SIP provider, listens for incoming calls
- Can also originate outbound calls

### `modem_engine.c` — call state and modem engine
- Coordinates native V.PCM DSP and vendored SpanDSP, including digital and analogue roles
- Handles V.8 negotiation → V.90 training → data mode
- Falls back to V.34 if V.90 training fails

### `clock_recovery.c` — RTP-to-synchronous bridge
- Accepts RTP packets (variable timing, 20ms frames = 160 samples)
- Outputs steady 8000 Hz sample stream to spandsp
- Preserves 8000 Hz sample accounting; sample insertion/deletion or resampling breaks PCM modem operation

### `data_stack.c` / `mnp.c` — error control and compression
- V.14 async framing, V.42/LAPM with V.42bis or V.44, and MNP (V.42 Annex A)
- Selected by `AT+ES`/`AT+DS`; see [docs/v42_reliability.md](docs/v42_reliability.md) and [docs/mnp.md](docs/mnp.md)

### `data_interface.c` — User data I/O
- Creates PTY pair (virtual serial port)
- Optionally listens on TCP socket
- Supports AT command interface for modem control

## Building

```bash
# Ubuntu/Debian build deps for the bundled libraries
sudo apt install build-essential autoconf automake libtool pkg-config \
                 libasound2-dev libssl-dev libopus-dev libtiff-dev \
                 libavformat-dev libavcodec-dev libswscale-dev \
                 libavutil-dev libv4l-dev

# Build
make

# Run
./sip_v90_modem --sip-server your-provider.com \
                --username modem \
                --password yourpassword \
                --pty-link /tmp/v90modem
```

Select the highest modem family offered in V.8 with
`--mode v21|b103|b212|v23c|v22|v22-1200|v32|v32bis|v34|v90|v92|v91|k56|x2|x2-symm|clear|clear56|v120|v120-56|v110`. The default is `v90`; lower fallback
modes remain advertised (V.32bis and V.22bis; `ME_V8_ADVERTISE_V32=0` drops the
V.32 bit). `v32bis` offers V.32bis + V.22bis, `v32` the same capped at
V.32's 9600/4800. A pre-V.8 modem is met through V.32bis Annex A automode
(AA/AC/USB1, see `docs/v32bis_compliance_plan.md`); `ME_V8=0` makes this end
one. For a plain V.34 interoperability run, configure the
peer for V.34 and start this endpoint with:

```bash
./sip_v90_modem --mode v34 --sip-server your-provider.com \
                --username modem --password yourpassword \
                --pty-link /tmp/v90modem
```

`ME_MODE` takes the same names. The older `ME_V92_ENABLE=1` remains supported
when `ME_MODE` is unset. Unknown command-line arguments are an error (usage,
exit status 2).

Either of those sets the power-on default. A DTE can change the offer for
later calls with V.250's `AT+MS` on the PTY; the call in progress is never
touched, and `ATZ`/`AT&F` restore the default:

| `AT+MS=`   | next call offers                                         | automode 0 (`,0`) |
|------------|----------------------------------------------------------|-------------------|
| `V22`, `V22B` | V.22/V.22bis                                          | same              |
| `V32B` (`V32BIS`) | V.32bis + V.22bis                                 | V.32bis alone     |
| `V32`      | as V32B, rates capped at 9600/4800                      | V.32 alone        |
| `HST`, `V32TERBO` (`TERBO`), `VFC` (`V.FC`) | V.32bis + V.22bis (no datapump for these; V.250 fallback) | ERROR |
| `V34` (`V34+`, `V34B`, `V34BIS`) | V.34 + V.22bis                     | V.34 alone        |
| `K56` (`56`, `56K`, `K56FLEX`) | K56flex V.8bis, then V.8 offering V.90/V.34/V.22 (no K56flex data mode) | ERROR |
| `V90`      | V.90 + V.34 + V.22bis                                    | V.90 + V.34       |
| `V92`      | as V90, with V.92                                        | V.92 + V.34       |
| `V91`      | as V90, with V.91 in V.8's PCM availability             | V.91 + V.34       |
| `X2`       | x2, asymmetric (V.34 upstream): server when answering, analogue client when calling | same |
| `X2S` (`X2SYMM`) | x2 digital symmetric (PCM both ways) | same |
| `CLEAR` (`CLEARMODE`, `64K`) | no V.8: the DS0 is the bit pipe, V.14 (or LAPM) on it; max rate <=56000 selects restricted 56k | same |
| `V120`     | no V.8: V.120 UI frames on the DS0; max rate <=56000 selects 56k | same |
| `V21`, `B103` | 300-bit/s FSK | same |
| `B212` | Bell 212A at 1200 bit/s; legacy Bell 103 fallback | same |
| `V23C` | V.23 1200/75 bit/s, with direction selected by rate bounds | same |
| `V110` | V.110 rate adaptation on a compatible digital bearer | same |
| `X75` | recognised, no datapump here | ERROR (both) |

`AT+MS?` reads back e.g. `+MS: V34,1,0,0,0,0`; `AT+MS=?` lists the carriers;
`AT+MS$` prints Courier-style help -- the syntax, every carrier with its
aliases, accepted automodes, maximum rate and what the next call will offer,
and the current setting.
The rate subparameters (`<carrier>,<automode>,<min>,<max>` or V.250's
`...,<min_tx>,<max_tx>,<min_rx>,<max_rx>`) may not exceed the carrier's
maximum (14400 for V32B, 33600 for V34, 56000 for V90/V92, 60000 for K56 --
the shipped K56flex firmware tables run to 58000/60000 -- and 64000 for V91
and X2/X2S, whose digital symmetric mode carries 64000 both ways); they are stored and reported, not enforced -- rates come from training (`V32`'s 9600 cap is the one exception, since V.32 has no higher rate).
`ME_K56FLEX` and `ME_V8_ADVERTISE_V91`, when set, override `AT+MS`.

### On-line help and ATI pages

Courier-style `$` help lists what this modem actually does with each command,
including the ones it accepts and ignores (a pty has no DCD or DTR line, a SIP
call has no speaker or dial pause):

| Command | Page |
|---|---|
| `AT$`  | basic commands, and how to reach every other page |
| `ATD$` | dial modifiers (commas are dropped: a SIP call has no pause) |
| `AT&$` | ampersand commands |
| `AT+$` | extended commands, with the ones not yet applied named |
| `AT+MS$` | modulations (above) |
| `ATI$` | the ATI pages |
| `ATS$` | S-registers with their current values |

| ATI | Shows |
|---|---|
| `ATI0` / `ATI3` | product / build version (`git describe`) |
| `ATI4` | current settings: E/Q/V/X/&C/&D, S-registers, +FCLASS, +MS, +MR/+ER/+DR/+ES/+DS, console |
| `ATI6` | link diagnostics, live during a call (the engine pushes twice a second) and kept after it: direction, state (data / retraining), carrier, rates, error control, compression, octets each way, duration, line level each way, disconnect reason (also for a call that failed before data mode) |
| `ATI7` | product configuration: line, modulations, protocols, fax classes, diagnostics |
| `ATI11` | engine detail, live: mode and offer, role, V.92, G.711 law, framing, fallback, retrains, and for V.34/V.90 symbol rates, carriers, bit rate, round-trip delay, B1 SNR |
| `ATY11` | line spectrum after the Courier's frequency/level table: RX and TX level in dBm0 per 150 Hz band, 150-3900 Hz, over the last second, with a bar graph and the totals |

ATI6, ATI11 and ATY11 can be read after NO CARRIER (they keep the call's last
state and last second of audio); mid-call, use the control port, or a guarded
`+++` on the combined port. A command the modem does not recognise answers
ERROR (V.250 5.6).

CLEAR and V120 need the bearer byte-exact end to end and both ends set alike
-- there is no negotiation, as on ISDN. Two instances of this server reach
data with them over SIP; see `docs/clear_channel_v120.md`.

### macOS notes

- Install dependencies (Homebrew), then build with `make`.
- `spandsp-master/` is always used for SpanDSP; no system `spandsp` package is required.
- The Makefile auto-detects Homebrew where possible and will reconfigure vendored deps if you move the tree between macOS and Linux.
- If your pjproject install uses a different architecture/version suffix, override:
  - `make ARCH_SUFFIX=<your-suffix>`
  - e.g. `make ARCH_SUFFIX=arm64-apple-darwin23.0.0`

### Using bundled pjproject

- The top-level `make` now prefers the in-repo `pjproject/` tree by default.
- The bundled `spandsp-master/` and local `pjproject/` builds are host-specific, and `make` will automatically rebuild them when the host OS/arch changes.
- Use the vendored libraries for supported builds; system copies are not substitutes.

## Offline Tone Regression

- `vpcm_decode` can now probe V.34/V.90 Phase 2 from captured WAVs with `--v34`.
- For the stereo tone sets in `gough-lui-v34-modem-sounds/` and
  `gough-lui-v90-v92-modem-sounds/`, channel quality matters:
  some files only recover `INFO1a` on one side.
- To batch-score the tone corpus for `INFO0a`, `INFO1a`, and Phase 3 recovery, run:

```bash
make v34-tone-matrix
```

- `make v34-tone-matrix` now defaults to `gough-lui-v34-modem-sounds/` when present,
  and falls back to `gough-lui-v90-v92-modem-sounds/`. You can still pass a directory
  explicitly.

## V.56 Impaired-Line Loopback

`make v56-test` runs strict offline V.34 modem-pair regressions using the
V.56ter 511-bit pattern, both G.711 laws, and synthetic noise, attenuation,
delay and echo. `make v56bis-filter-test` validates 18 V.56bis AD/EDD
filters; `make v56bis-sweep` measures modem startup and BER through them.
`make v56-sweep` measures a million bits per direction across
noise/delay profiles, retaining JSON results and logs for failed cases too.
These are synthetic line measurements, not full V.56bis network coverage.
See [the harness guide](docs/v56_loopback.md) for custom sweeps and limitations.

## AT Diagnostic Loops

`AT+TLDL` enables a local DTE digital loop during an established data call.
`AT+TTER` runs finite pattern tests on that software loop; `AT+TNUM?` reports
retained bit/block errors. `make at_test_test` verifies both PTY modes.
`AT+TSELF=1` performs a limited safe host-memory/controller check.
Remote/analogue loop actions and full self-tests return ERROR until their
actual procedures are implemented. See [AT diagnostics](docs/v54_diagnostics.md).

## V.90/V.92 PCM Loopback

`make pcm-loopback-test` runs a strict directed PCM datapump matrix plus
startup and recovery procedure checks. `make pcm-matrix` retains diagnostic
results across all V.90 downstream and V.92 PCM-upstream profiles, including
high-rate failures. The downstream DS0 remains byte-exact; optional upstream
noise is applied before the simulated network A/D. See
[PCM testing procedures](docs/pcm_loopback_testing.md) for coverage, measured
limitations and reproducible sweeps.

## Python Analysis Tools

- The offline demod / Ja-analysis scripts under `tools/` use a small Python stack.
- Install or refresh the repo-local virtualenv with:

```bash
python3 -m venv .venv
./.venv/bin/python -m pip install -r requirements-tools.txt
```

- Run the Phase 3 demod tool with the virtualenv interpreter so it picks up
  `numpy` and `scipy`:

```bash
./.venv/bin/python tools/v34_phase3_demod.py --help
```

## Usage

### Live raw G.711 taps

When PJMEDIA passthrough is available, the live modem engine now receives and
transmits PCMU/PCMA octets directly. To capture the engine-side bearer, point
`VPCM_G711_TAP_DIR` at an existing directory before starting the modem:

```bash
mkdir -p /tmp/v90-g711
VPCM_G711_TAP_DIR=/tmp/v90-g711 ./sip_v90_modem ...
```

This writes `live-rx.g711` and `live-tx.g711`. Modem diagnostic snapshots also
report total RX/TX octets and split outgoing octets between raw V.90 generation
and the linear compatibility path. During Phase 4 they also report `cp_bits`,
`cp_valid`, and `cp_rejected` for the strict analogue-side CPt receiver.
Accepted CPt drives negotiated `Sr=0/1/2/3` Ri/TRN2d/MP/Ed training, while the
later CP/CP-prime selects the independent 48-frame B1d and connected-data
mapper. Both mappers implement the mandatory zero- and one-frame lookahead
algorithms. The live raw-G.711 path consumes the negotiated D bits per
six-codeword frame. Optional two-/three-frame lookahead is deliberately
rejected until implemented.

### Softmodem Debug Mode (no PJMEDIA passthrough)

If your local `pjproject` build does not include PJMEDIA passthrough codecs,
`sip_v90_modem` now falls back to a linear PCM bridge automatically instead of
exiting. This mode is intended for interoperability/debugging with softmodems.

- On startup, look for:
  - `G.711 passthrough enabled` (preferred), or
  - `falling back to linear PCM bridge for softmodem debugging`
- Keep your SIP call on G.711 (`PCMU/PCMA`) and disable VAD/echo cancellation
  on your PBX/ATA path as usual.

Quick launcher:

```bash
./scripts/softmodem_debug.sh --sip-server your-provider.com --username 6001 --password 6001
```

Local/no-register launcher:

```bash
./scripts/softmodem_debug.sh
```

Useful flags:

- `--pty-link /tmp/v90modem` (classic combined console: commands and data on one port, `+++` / `ATO`)
- `--control-link /tmp/v90ctl --data-link /tmp/v90data` (control console that always speaks AT, plus a payload-only data port; `ATO` on the control port prints the data PTY's path; not combinable with `--pty-link`)
- `--local-port 5060` (local SIP UDP port)
- `--log-file /tmp/v90modem.log` (capture runtime logs)
- `--no-build` (skip rebuild)
- `--skip-preflight` (bypass local UDP bind check)

Once running, connect to the PTY with minicom or any serial terminal:
```bash
minicom -D /tmp/v90modem
```

Or use the TCP data port:
```bash
nc localhost 5800
```

### Reproducible hardware interoperability run

Use the bounded runner to test an analogue modem/FXS path while preserving the
modem log, raw RX/TX G.711 taps, hashes, build revision, and parsed timeline:

```bash
./tools/v90_hardware_interop.py \
  --label "Courier-V-Everything" \
  --duration 180 \
  -- \
  --sip-server asterisk.example \
  --username 6001 \
  --password 'secret' \
  --pty-link /tmp/v90modem
```

Each run is stored under `reference-data/artifacts/v90-hardware/` with `manifest.json` and
`summary.json`. Password arguments are redacted from the manifest. Use
`--dry-run` to verify the command without starting the modem or writing files.

### Eicon sustained transfers and PPP

`tools/eicon_soak_test.py` runs a 610-second binary download, 610-second
upload and 310-second simultaneous test against the Eicon BRI answerer.
Both endpoints verify incompressible payloads, byte counts and receive duration.
`--test ppp` starts an explicit host-terminated PPP link and checks IPCP
and 30 pings. Add `--http-bytes 1048576` for checked HTTP download and
POST upload to the PPP peer. Run from the Tower LAN, with `/dev/ppp` and
`NET_ADMIN` for PPP inside Docker. On the wired Tower LAN,
`--jitter-buffer-ms 40` improved checked binary download to 6.44 kB/s
and HTTP to 6.21 kB/s. See [deployment and evidence](docs/eicon_soak_ppp.md).

### V.92 Phase 4

Native SUVd/CPd and CPu/CPu-prime handling, PCM-upstream datapump components
and recovery procedure tests are implemented. Full foreign-peer PCM-upstream
startup and sustained data are still under development; see
[V.92 implementation status](docs/v92_phase4_implementation.md).
Quick Connect (short Phase 1 and 2) is described in
[docs/v92_quick_connect.md](docs/v92_quick_connect.md).

## License

GPL-2.0 (due to spandsp LGPL and linmodem GPL heritage)

## Experimental x2

`--mode x2` (`AT+MS=X2`) selects the experimental asymmetric x2
implementation: the digital server when answering, the analogue client when
calling. `--mode x2-symm` (`AT+MS=X2S`) selects digital symmetric x2.
Against original Courier firmware running in an emulator, the server carries
PTY payload both ways, with V.42 LAPM (`CONNECT 53333/ARQ/x2/LAPM`), honours
`AT+MS` rate bounds in both directions and answers in-data fall-forward
renegotiation. The client dials both our own server and the native Ie030002
server. A LAPM soak at index 3 (40000 down, 31200 up) delivered 15750 of
15750 bytes over 132 s; at higher indices the emulated Courier corrupts payload
that our transmit tap decodes byte-exact.
Symmetric x2 has engine-pair coverage. None of this qualifies sustained or
real-hardware x2 interoperability. The x2 tests are in `tests/fast.list`
(`x2_test`, `x2_session_test`, `x2_client_test`, `x2_sym_test`). See
[x2 implementation and evidence](docs/x2_implementation.md).
