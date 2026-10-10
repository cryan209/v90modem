# SIP V.90 Modem

A standalone SIP-based V.90 digital modem server built on PJSIP and spandsp.

This acts as the **digital side** of a V.90 connection. An analog V.90 client modem
dials in via a SIP provider (or directly via an ATA/FXS gateway), and this software
answers the call, performs V.90 digital modem negotiation over the G.711 μ-law RTP
stream, and bridges the resulting data connection to a PTY (virtual serial port) or
TCP socket.

## Live GUI

Run `./sip_v90_modem --gui` with your usual SIP options for the native macOS
console, or use `--gui-web` for a local browser. It shows live RX/TX waveforms,
QAM/TCM constellations, adjustable line-derived eyes, separate datapump/DS0 byte
views, AT control, serial payload, and training transitions. Histories are
bounded in memory; no diagnostic recordings accumulate on disk.
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
- G.711 μ-law (PCMU) codec, no transcoding anywhere in the path
- No voice activity detection (VAD) or comfort noise generation (CNG)
- No echo cancellation on the SIP path
- Minimal, fixed jitter buffering
- The far-end analog modem connects through an FXS port with a real D/A converter

## Components

### `sip_modem.c` — Main application
- Initializes PJSIP stack and spandsp modem engine
- Registers with SIP provider, listens for incoming calls
- Can also originate outbound calls

### `modem_engine.c` — spandsp V.90 modem wrapper
- Configures spandsp for V.90 digital (server) mode
- Handles V.8 negotiation → V.90 training → data mode
- Falls back to V.34 if V.90 training fails

### `clock_recovery.c` — RTP-to-synchronous bridge
- Accepts RTP packets (variable timing, 20ms frames = 160 samples)
- Outputs steady 8000 Hz sample stream to spandsp
- Handles clock drift compensation

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
`--mode v22|v32|v32bis|v34|v90|v92|v91|k56|x2|clear|clear56|v120|v120-56`. The default is `v90`; lower fallback
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
| `X2`       | x2, asymmetric (V.34 upstream; symmetric not implemented) | same             |
| `CLEAR` (`CLEARMODE`, `64K`) | no V.8: the DS0 is the bit pipe, V.14 (or LAPM) on it; max rate <=56000 selects restricted 56k | same |
| `V120`     | no V.8: V.120 UI frames on the DS0; max rate <=56000 selects 56k | same |
| `B103`, `B212`, `V110`, `X75` | recognised, no datapump here | ERROR (both) |

`AT+MS?` reads back e.g. `+MS: V34,1,0,0,0,0`; `AT+MS=?` lists the carriers;
`AT+MS$` prints Courier-style help -- the syntax, every carrier with its
aliases, accepted automodes, maximum rate and what the next call will offer,
and the current setting.
The rate subparameters (`<carrier>,<automode>,<min>,<max>` or V.250's
`...,<min_tx>,<max_tx>,<min_rx>,<max_rx>`) may not exceed the carrier's
maximum (14400 for V32B, 33600 for V34, 56000 for V90/V92, 60000 for K56 --
the shipped K56flex firmware tables run to 58000/60000 -- and 64000 for V91
and X2, whose digital symmetric mode carries 64000 both ways; only the
asymmetric x2 session exists here); they are stored and reported, not enforced -- rates come from training (`V32`'s 9600 cap is the one exception, since V.32 has no higher rate).
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
- If needed, disable this and use system/Homebrew pjproject with:
  - `make USE_LOCAL_PJPROJECT=0`

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

Each run is stored under `artifacts/v90-hardware/` with `manifest.json` and
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

Strict Table 31 SUVd and mandatory-part Table 30 CPd codecs are implemented,
and the Phase 4 analyzer now reports progression through SUVd, CPd,
acknowledgement, Ed, B1d, and DATA. Full CPd optional parts and live CPu-driven
exchange remain under development; see `docs/v92_phase4_implementation.md`.

## License

GPL-2.0 (due to spandsp LGPL and linmodem GPL heritage)

## Experimental x2

`--mode x2` selects the initial PCMU digital-answerer implementation. Capability
exchange and A/B/J/C/D/E training are wired into the engine, including the
Courier's received J and S-bar gates. The payload mapper is verified against
original DSP vectors. The supported short MP is decoded from the line and selects original-firmware
data banks; mapped startup then activates downstream payload transmission.
Upstream E/B1 data handoff and bidirectional verification remain; this mode
does not yet complete an x2 connection.
Run `make x2-test x2-session-test`. See `docs/x2_implementation.md` for
supported profiles, capture verification and firmware-vector reproduction.
