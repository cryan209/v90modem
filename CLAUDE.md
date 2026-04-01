# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a SIP-based V.90 digital modem server. It acts as the **digital side** of a V.90 connection — an analog V.90 client modem dials in via SIP, this software answers, performs V.90 handshake over G.711 μ-law RTP, and bridges the data connection to a PTY (virtual serial port).

**Key V.90 constraint:** The SIP path must carry G.711 PCMU with no transcoding, no VAD/CNG, and no echo cancellation — the RTP payload IS the DS0 PCM stream that the far-end D/A converter sees.

## Build Commands

```bash
# Build
make

# Clean and rebuild
make clean && make

# Run
./sip_v90_modem --sip-server asterisk.net.cryan.nz --username 6001 --password 6001 --pty /tmp/v90modem

# Connect to PTY
minicom -D /tmp/v90modem
```

No automated test suite exists. Testing is done manually with a real analog modem.

## Platform Notes (macOS Apple Silicon)

Dependencies are hardcoded to Homebrew paths:
- `PJPROJ_DIR = /opt/homebrew/Cellar/pjproject/2.16`
- SpanDSP 3.0 is compiled from source in `spandsp-master/` (not installed system-wide)
- `ARCH_SUFFIX = aarch64-apple-darwin24.6.0` must match your macOS version

To find the correct `ARCH_SUFFIX` for your system:
```bash
ls /opt/homebrew/lib/libpj-*.a | sed 's/.*libpj-//' | sed 's/\.a//'
```

## Architecture

Four modules, each with a `.c/.h` pair:

### `sip_modem.c` — Main application & SIP UA
- Initializes PJSIP (pjsua API), registers the SIP account
- Creates a **custom `pjmedia_port`** that bridges PJSIP's conference bridge to the modem engine:
  - `modem_put_frame()` — RTP audio in → `me_rx_audio()`
  - `modem_get_frame()` — `me_tx_audio()` → RTP audio out
- Forces G.711 PCMU and PCMA codec; disables all others (speex, iLBC, GSM, G.722, opus)
- Must not transcode G.711 to another codec. Will passthrough codec using PJMED_PASSTHROUGH_CODEC
- Tells modem engine what G.711 codec is in use
- Main loop polls `me_get_state()` every 10 ms; drives ring cadence, auto-answer (after 2 rings), outbound dialing, and hangup

### `modem_engine.c` — V.90 state machine & codec
State machine: `ME_IDLE → ME_DIALING → ME_V8 → ME_TRAINING → ME_DATA → ME_HANGUP`
- **G.711 Decoder/Encoder** - Must not transcode G.711 for V.90, V.91, V.92 PCM modem codes.
- **V.8 negotiation** — uses SpanDSP `v8_state_t`; selects V.90, V.34, or V.22bis fallback
- **V.90 downstream encoder** (ITU-T V.90 §5) — custom implementation:
  - Scrambler (x^23 + x^5 + 1 polynomial)
  - Maps data bytes → 7-bit magnitude + 1-bit sign → Ucode → PCM codeword
  - Differential sign coding per §5.4.5.1
  - Uses `v90_ucode_to_alaw[128]` table; μ-law uses formula `0xFF - ucode`
- **V.22bis** (SpanDSP) — used as upstream placeholder and V.34 fallback
- **V.34** (SpanDSP) — present in library; not yet wired into upstream path
- Thread-safe: state protected by `pthread_mutex_t g_state_mtx`; ring buffers (16 KB each) use separate mutexes

### `clock_recovery.c` — DPLL jitter compensation
- PI controller (Kp=0.01, Ki=0.001) that tracks RTP timestamp vs. wall-clock
- Emits slip signals (+1 insert sample / -1 drop sample) to keep modem clock locked
- Critical: modem signals are extremely sensitive to sample-rate errors

### `data_interface.c` — PTY & AT command interface
- Creates PTY master/slave pair via `openpty()`; symlinks slave to `/tmp/modem0`
- Uses SpanDSP `at_state_t` for Hayes AT command parsing (ATD, ATA, ATH, +++, etc.)
- Two modes: command mode (parses AT commands) and data mode (passes raw bytes to ring buffer)
- `pty_reader_thread()` runs a 50 ms select() loop

## Key Implementation Status

- **V.90 training (Phases 1–4, §8–9)** — stubbed with a fixed silence/tone period; full implementation needed for real interoperability
- **V.34 upstream** — SpanDSP V.22bis (2400 bps) is a working placeholder; V.34 upstream encoder not yet wired up
- **Mi negotiation** — hardcoded to Mi=128 (7 magnitude bits per symbol); should be negotiated during training

## Dependencies

| Library | Version | Source |
|---------|---------|--------|
| pjproject (PJSIP) | 2.16 | Homebrew |
| spandsp | 3.0 | Local build in `spandsp-master/` |
| OpenSSL | system | Homebrew |
| libtiff | system | Homebrew (pulled in by spandsp) |

ITU-T reference standards are in `ITU Docs/` (V.90, V.8, V.34, V.22bis, V.42, G.711, etc.).
