# Browser modem demo

`web/modem-lab/dist/` is a buildless interactive browser demo. It contains
JavaScript V.21/Bell 103 FSK and the real C modem engine compiled to WebAssembly
for V.32 at 9600 bit/s and V.90. It has no remote service dependency.

## Run

The generated WASM and loader are checked in so no compiler is needed to try it:

```
python3 -m http.server 8765 --bind 127.0.0.1 --directory web/modem-lab/dist
```

Open http://127.0.0.1:8765/. Enter up to 240 UTF-8 bytes, select a mode, and
press **Dial & send**. The call is generated first in a worker, then played
back with sample-timed events and received text. **Stop** cancels generation
or playback. Advanced playback and WAV exports put caller left, answerer right.
FSK exports are mono. Speaker output can be resampled by the browser; it never
feeds back into either modem or the virtual bearer.

For a single offline file, run `python3 tools/package_modem_demo.py` and open
`artifacts/modem-lab.html`. It embeds the worker, DSP, WASM, style, and interface;
there are no external scripts, fonts, audio files, or network requests.

## Signal paths

* **V.21 / Bell 103:** continuous-phase JS transmitter, 300 bit/s, 8N1 UTF-8
  bytes. V.21 channel 1 uses 980 Hz mark / 1180 Hz space (V.21 clause 2).
  Bell 103 originating tones are 1270 / 1070 Hz. A distinct sample-only
  receiver detects falling mark/space transitions, recovers frame timing,
  and fits the two sinusoids without knowing transmitter oscillator phase.
  The receiver gets neither the source text nor transmitter bit boundaries.
  Noise is repeatable uniform noise at clean, 24, 12, or 0 dB SNR. Start-up
  is an idle carrier; this does not implement V.25 call establishment.
* **V.32 / V.90:** two fresh isolated instances of the same WASM module,
  one calling and one answering. Each consumes the previous opposite TX
  frame before generating 160 G.711 codewords. The DS0 remains byte-exact
  (V.90 clause 5). V.32bis is rate-limited to the V.32 9600 bit/s mode
  (V.32bis clause 6 negotiation); V.90 selects the caller's analogue role.
  V.42 LAPM and the existing default data-stack negotiation carry the bytes.
  A two-byte little-endian application length precedes the message. The
  answerer constructs its `ACK: ` reply from the **received** message.
  The UI checks that both decoded messages match. Application length
  framing bounds messages separately from the underlying line-bit stream.
* **Plots:** waveform and spectrum use the generated TX audio. The advanced
  constellation uses actual equalized RX symbol reports from the answerer,
  through the existing passive `line_monitor` tap, at 200 ms intervals.
  It is auto-scaled, and a snapshot includes up to 256 symbols.

## Rebuild and verify

Install the official Emscripten SDK 4.0.16, then:

```
EMCC=/path/to/emsdk/upstream/emscripten/emcc python3 tools/build_modem_wasm.py
node web/modem-lab/test.mjs
python3 tools/package_modem_demo.py
```

The builder uses only vendored sources, scalar WASM, and a separate portable
header configuration. It stores objects under `artifacts/wasm-build/`, checks
source/header hashes before reuse, and never changes native objects or native
SpanDSP configuration. `dist/build-info.json` records compiler and source hashes.
The browser adapter is intentionally a browser-only translation unit, included
by this dedicated builder rather than native `SRCS` or native test object lists.
The included `tiffio.h` is an empty include shim: no fax/image implementations
are linked, and the public modem headers do not need TIFF declarations.

The adapter replaces SIP/PTY/fax presentation only. It does not replace any
DSP or synthesize CONNECT. Browser worker threads drive the engines serially;
internal optional POSIX background diagnostic workers cannot start, so the
existing inline Ja scan fallback is used. This is not a port of the full
SIP server, hardware audio driver, fax service, or every modulation.

The automated checks exercise both FSK tone pairs over clean/noisy signals,
unknown sample offsets, ASCII, NUL, UTF-8, and 240-byte messages. The WASM
checks train V.32 and V.90 with short ASCII, UTF-8, and 240-byte payloads,
verify messages in both directions, assert negotiated rates, and require
measured constellation samples. On 2026-10-16 V.32 negotiated 9600/9600 and
V.90 31200 upstream / 56000 downstream in these software-only checks.

A longer V.90 raw V.14 burst exposed a corrupted tail during initial browser
port checks; short bursts passed. The demo uses V.42 LAPM and its full test
matrix passes. No DSP constants or native receive paths were changed to make
the demo pass. This finding does not diagnose whether the raw V.14 limitation
is native behavior or WASM-specific, and it is not hardware evidence.

SpanDSP's LGPL license and source/build pointers are in `dist/COPYING.spandsp`
and `dist/NOTICE.txt`.
