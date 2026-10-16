# Browser modem demo

`web/modem-lab/dist/` is a buildless interactive browser demo. It contains
JavaScript V.21/Bell 103 FSK and the real C modem engine compiled to WebAssembly
for V.32 at 9600 bit/s, V.34, and V.90. It has no remote service dependency.

## Run

The generated WASM and loader are checked in so no compiler is needed to try it:

```
python3 -m http.server 8765 --bind 127.0.0.1 --directory web/modem-lab/dist
```

Open http://127.0.0.1:8765/. Enter up to 240 UTF-8 bytes, select a mode, and
press **Dial & send**. The call is generated first, then played
back with sample-timed events and received text. **Stop** cancels generation
or playback. Advanced playback and WAV exports put caller left, answerer right.
FSK exports are mono. Speaker output can be resampled by the browser; it never
feeds back into either modem or the virtual bearer.

For a single offline file, run `python3 tools/package_modem_demo.py` and open
`artifacts/modem-lab.html` or `web/modem-lab/dist/standalone.html`. The packager
refreshes both copies together. Each embeds the worker, DSP, WASM, style, and interface;
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
* **V.32 / V.34 / V.90:** two fresh isolated instances of the same WASM module,
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
node web/modem-lab/test-offline.mjs
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

## Negotiation inspector

The expandable inspector has **Simple decode**, **Verbose decode / engine
log**, and **Raw octets & bitstreams** views, with observing-endpoint and
protocol-stage filters. It follows playback by default. Its independent
inspector-time slider lets you revisit an earlier exchange without replaying
the call. All timestamps use the simulation's 20 ms frame clock, not the
engine's wall-clock `[TRACE]` prefix. TX log messages can describe queued
protocol material; data-layer HDLC timestamps mark completion at the tap.

The simple view retains actual V.8 CM/JM/CJ menu octets and peer capability
summaries, INFO0d/INFO0a role evidence, INFO1 channel selections, the parsed
Ja DIL descriptor (N/LSP/LTP), DIL transmission and measurement reports,
and V.42 detection/negotiation. The `a` and `d` role interpretation here
applies to V.90; call direction alone does not establish the PCM role.
Fields are taken from engine decodes, not reconstructed from the selected UI
mode. Engine labels are retained in raw/verbose views, including an existing
Tx INFO1d raw-frame label used for the analogue INFO1a block.

Four complete raw data-layer bitstreams (caller/answerer × TX/RX) are captured
using existing `DS_TX_BIT_DUMP` / `DS_RX_BIT_DUMP` hooks in each module's MEMFS.
The browser adapter flushes them and measures cumulative bit counts per frame.
The JavaScript inspector searches flags, removes HDLC bit stuffing, groups
LSB-first octets, and checks the reflected CRC-16 residual `0xf0b8` (V.42
clauses 7–8). Simple decode includes CRC-valid XID, SABME and UA records. XID
fields follow V.42 Table 11: optional procedures, TX/RX N401 in **bits**, TX/RX
window k, and private V.42bis P0/P1/P2. The observer identity and TX/RX direction
are preserved: a received peer menu is not this modem's own advertised offer.

Raw coverage is explicitly bounded. V.8 octets are decoded menu bytes; their
LSB-first rendering omits FSK preambles/start/stop bits. Packed INFO1 bytes
retain the engine's layout. There is no complete demodulated bitstream for
every startup phase or for Ja. DIL is a PCM training waveform, not application
bits. Exact G.711 codewords for both complete TX streams are also captured,
so the DS0 during Ja/DIL can be inspected at any simulation time. Display
windows show the most recent 512 data-layer bits and 64 G.711 codewords;
**Capture JSON** exports the complete captured streams, logs and decoded frames.
Verbose display shows at most 600 matching log lines at a time; its complete
capture is capped at 100,000 lines and an explicit omission count is shown if
reached. Current regression calls do not reach that cap.

The browser-only adapter enables verbose/FLOW logs for offline inspection.
This does **not** change the rule against FLOW logging on live calls. Native
engine/DSP sources are unchanged by the inspector. Tests require real CRC-valid
XID/SABME/UA frames in both advanced modes, check decoded XID parameters, flip
a captured bit to prove CRC rejection, verify timeline filtering and exact
DS0 capture lengths, and continue to verify payloads and constellation taps.

## Direct-file execution

Some browser/WebView implementations reject module workers created from a
`file://` page's opaque origin. The standalone package therefore selects an
inline runner for `file://`, using the identical embedded WASM and engine-pair
code. It yields between simulated one-second batches; Stop cancels at the next
yield and the engine instances are destroyed. HTTP(S) keeps the worker path.
No server, external imports, or WASM fetch is required for the local-file path.

`test-offline.mjs` extracts and executes the generated HTML's actual inline
module with a minimal DOM harness and `location.protocol='file:'`. Worker
construction deliberately throws. Both V.32 and V.90 must still exchange real
UTF-8 payloads and acknowledgements, and cooperative cancellation is checked.
This regression complements the normal DSP/inspector suite; it does not pretend
to exercise every browser's rendering or security policy.

## V.34 and startup selection

V.34 uses the same real engine pair and clean G.711 bearer, rather than audio
recordings. Automatic selection uses the 3429-symbol/s training profile and reaches 33600 bit/s on the clean synthetic path. Explicit lower ceilings remain available.
The V.34 ceiling selector also offers 19200, 24000, and Automatic; the higher
offers can trigger real retrains, retained in the events and inspector. The
ceiling limits the offer; the actual connection rate is still negotiated.

The startup controls apply to both virtual modems. **V.8 negotiation** is on by
default. It can be disabled for V.32, selecting the engine's V.25/legacy startup.
V.34 and V.90 require V.8 in this demo, so its checkbox stays enabled in value
and locked for those modes. **V.8bis before V.8** enables the existing
`ME_V8BIS` pre-negotiation on V.32, V.34 and V.90; it is available only with V.8.
FSK preset playback does not implement these startup protocols, so both controls
are disabled there. Controls are locked while generating/playing a call.

V.8bis completion and mode-selection messages are included in simple decode
and verbose logs. The result/JSON capture records the requested startup options.
The generic engine state is labelled V.25, V.8bis or V.8 according to the actual
startup option and observed V.8bis-completion log, rather than always saying V.8.

Regression coverage includes long UTF-8 payloads through V.34 with V.8 and
V.8bis, legacy V.32 without V.8, V.32 with V.8bis, and V.90 with V.8bis. It
requires actual V.8bis completion at both ends and no CM/JM menus in legacy
mode, and rejects unsupported V.8-off V.34/V.90 requests. The generated-file
harness also runs V.34/V.8bis without constructing a Worker.

### V.34 default offer ceiling (2026-10-16)

With an unrestricted offer the Phase-4 TRN estimate requested 24000 bit/s in
one direction, but the final B1 check measured 31.7 dB and assessed only 21600.
The engine correctly requested a V.34 §11.5 retrain. That attempt then timed
out in TRN (63% lock, below the existing 65% threshold), producing another
retrain before payload transfer. The demo now defaults to an explicit 21600
bit/s ceiling via the existing `ME_V34_BPS` setting; this avoids both restarts
without changing a DSP constant or the G.711 bearer. Automatic remains available
to reproduce the original behavior. Long UTF-8 messages pass without retrains
at the default ceiling both with and without V.8bis, including direct-file runs.
This is a demo profile choice, not a fix to the underlying rate-estimation or
retraining-acquisition discrepancy. See `docs/v34_data_mode_rates.md`.

### V.34 3429-symbol/s browser profile (2026-10-16)

The browser adapter now requests the existing 3429-symbol/s path and defaults
to a 24000 bit/s offer ceiling. Normal Phase-4 TRN rate selection, B1 checks
and §11.5 retraining remain enabled (V.34 §11.4–11.5). A deterministic
12000-byte varied ASCII transfer and its 12005-byte acknowledgement passed
at 24000/24000 without retrains or invalid captured HDLC frames. The previous
3200-symbol/s automatic profile finished at 21600/21600 after two retrains.
This is a browser profile improvement; it does not change the native default
or establish performance on a physical telephone line.

A diagnostic-only 3429/33600 run also passed that payload, but required
bypassing conservative training-rate checks. That bypass was removed and is
not shipped. Slowing Phase-4 equalizer adaptation reduced measured quality
and was also discarded. Higher automatic rates still need receiver/rate
estimator work; a CONNECT indication alone is insufficient validation.

### Paired B1/DATA conditioning below 31200 (2026-10-16)

The bundled engine includes the native V.34 receiver fix enabling its existing
complete B1 equalizer fit and matching DATA timing recovery at 3429 symbols/s
from 26400 bit/s upward. Previously both were gated to 31200/33600. The
noise/timing A/B matrix improved from 111/160 to 123/160 passing forced-rate
rows, with no regressions; million-bit qualification passed all 16 rows at
26400/30 dB and 28800/36 dB injected noise. See `docs/v34_data_mode_rates.md`
for the rates, delays, laws, failed alternative arms and reproduction tool.

This fixes receiver behaviour, not the separately conservative empirical
TRN/B1 rate policy. The default browser ceiling remains 24000. The bundle
was rebuilt from committed sources in an isolated staging directory so
parallel, uncommitted engine work was not included, then passed the browser
regressions before offline packaging.

### Automatic V.34 reaches 33600 (2026-10-16)

Automatic is now the browser default. The TRN and B1 checks share the corrected
receiver-specific 3429-symbol/s calibration instead of separately applying the
old single-channel `(SNR+13)/6` model. Neither check is bypassed. Explicit
19200/21600/24000 ceilings remain available.

The clean full-engine browser pair reaches 33600/33600 with V.8 and with
V.8bis+V.8. Regression tests require both the rate and the independently
received 12000-byte varied payload plus acknowledgement, with no retrains
or invalid captured HDLC frames. Direct-file fallback also requires actual
33600/33600 transfer. V.32 and V.90 retain their previous rates.

Native automatic-selection tests exercise a wider noise/timing matrix; see
`docs/v34_data_mode_rates.md` for the calibration, limits and remaining noisy
MP/E startup failures. This replaces the temporary conservative browser
ceiling described in earlier dated entries.

### Refresh shipped and offline engines (2026-10-17)

Reproduced an actual stale-bundle mismatch: the UI offered Automatic while
its embedded engine still applied the old training-rate formula and connected
at 24000/24000. Rebuilt the WASM from the shared training-rate calibration
and the V.34 §7/§10.1.3.9 MP acquisition fix. Browser-engine tests now pass
33600/33600 with V.8 and V.8bis, including 12000 varied bytes in both
directions with no retrains or invalid captured HDLC frames. The direct-file
runner also passes actual 33600/33600 transfer. These are software tests,
not physical modem interoperability measurements.

The packager now refreshes both existing offline entry points together,
`artifacts/modem-lab.html` and `web/modem-lab/dist/standalone.html`; the
offline regression requires their contents to match. An HTML file already
open in a browser needs reloading to use the refreshed embedded engine.
