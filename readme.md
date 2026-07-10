# SIP V.90 Modem

A standalone SIP-based V.90 digital modem server built on PJSIP and spandsp.

This acts as the **digital side** of a V.90 connection. An analog V.90 client modem
dials in via a SIP provider (or directly via an ATA/FXS gateway), and this software
answers the call, performs V.90 digital modem negotiation over the G.711 μ-law RTP
stream, and bridges the resulting data connection to a PTY (virtual serial port) or
TCP socket.

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

- `--pty-link /tmp/v90modem` (serial endpoint symlink)
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
  --pty /tmp/v90modem
```

Each run is stored under `artifacts/v90-hardware/` with `manifest.json` and
`summary.json`. Password arguments are redacted from the manifest. Use
`--dry-run` to verify the command without starting the modem or writing files.

### V.92 Phase 4

Strict Table 31 SUVd and mandatory-part Table 30 CPd codecs are implemented,
and the Phase 4 analyzer now reports progression through SUVd, CPd,
acknowledgement, Ed, B1d, and DATA. Full CPd optional parts and live CPu-driven
exchange remain under development; see `docs/v92_phase4_implementation.md`.

## License

GPL-2.0 (due to spandsp LGPL and linmodem GPL heritage)
