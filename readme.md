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
# Linux dependencies
sudo apt install libpjproject-dev libspandsp-dev

# Build
make

# Run
./sip_v90_modem --sip-uri sip:modem@your-provider.com \
                --sip-password yourpassword \
                --pty /tmp/v90modem
```

### macOS notes

- Install dependencies (Homebrew), then build with `make`.
- The Makefile auto-detects macOS and uses Homebrew-style pjproject naming.
- If your pjproject install uses a different architecture/version suffix, override:
  - `make ARCH_SUFFIX=<your-suffix>`
  - e.g. `make ARCH_SUFFIX=arm64-apple-darwin23.0.0`

### Using bundled pjproject

- The top-level `make` now prefers the in-repo `pjproject/` tree by default.
- If needed, disable this and use system/Homebrew pjproject with:
  - `make USE_LOCAL_PJPROJECT=0`

## Usage

Once running, connect to the PTY with minicom or any serial terminal:
```bash
minicom -D /tmp/v90modem
```

Or use the TCP data port:
```bash
nc localhost 5800
```

## License

GPL-2.0 (due to spandsp LGPL and linmodem GPL heritage)
