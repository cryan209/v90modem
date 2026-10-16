# Browser modem demo

`web/modem-lab/dist/` is a buildless interactive browser demo. It contains
JavaScript V.21/Bell 103 FSK and the real C modem engine compiled to WebAssembly
for V.22/V.22bis, V.23, Bell 212A, V.32/V.32bis, V.34, V.90/V.91/V.92.
It has no remote service dependency.

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
node web/modem-lab/test-reliability.mjs
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

## x2 and K56flex downstream PCM demos (2026-10-17)

The x2 and K56flex buttons run the existing C payload mapper and its inverse
in separate WebAssembly instances. They are explicitly **downstream payload
only**, using a fixed shared profile. They do not initialize the full modem
engine, emit CONNECT, simulate line training or upstream replies, or fall back
to V.90. See `docs/x2_implementation.md` and `docs/k56flex_implementation.md`
for the native implementations' startup and interoperability limits.

- x2 uses recovered short-record allocation 15, B=37 and MD=5, GPC source
  scrambling: 42 source bits per six 8-kHz PCM codewords, or 56000 bit/s.
  This is actual mapper bit accounting, not the historical nominal CONNECT
  display index (x2 Draft 0.33 §§10/11/15/16/21/22).
- K56flex uses the shipped mu-law 56000 profile, report field zero: 56 bits
  per eight PCM codewords (K56flex Draft 0.23 §§7.1–7.12).

A two-byte little-endian application length precedes UTF-8 text. Encoders
zero-pad the final mapping frame. Only transmitted codewords and the public
fixed profile cross to the receiver; decoding never receives the input text.
The waveform expands the emitted mu-law codewords for playback, with a
300-ms silence lead-in and 500-ms tail. The receiver consumes the byte-exact
codewords, not speaker audio. The server occupies the left audio channel;
the right channel is silent. These paths have no QAM constellation or LAPM
capture, so the lab disables those controls and exposes the profile and raw
codewords in the signal inspector and JSON export instead.

Browser regressions check ASCII, NULs, varied characters, UTF-8, 240-byte and
12000-byte messages, cancellation, payload recovery and sample accounting
for both modes. Damaged-codeword tests require rejection or changed decoded
text, and malformed frame lengths must reject. Offline regressions execute
both modes through the packaged HTML's file runner and check startup/QAM
controls are disabled. Existing V.32/V.34/V.90 regressions remain in place.

## V.92, line experiments and transfer statistics (2026-10-17)

V.92 now runs two full engines with PCM upstream enabled through the existing
V.250 `+PIG=0` setting. The startup selector offers full startup (`+PQC=3`),
Quick Connect (`+PQC=0`), and a two-call comparison. The analogue caller and
digital answerer negotiate actual V.92 48000/56000 on the clean synthetic
line. Tests require the engine's reported carrier to be V92, complete
bidirectional payloads, and the short-Phase-1 / short-Phase-2 events for
Quick Connect (V.92 §§6, 9.2, 9.4; V.250 §6.8.7).

The browser DTE adapter now discards bytes delivered before CONNECT, matching
the native data-port gate. V.92 initially delivered pre-CONNECT async noise
and LAPM startup bytes to the application buffer; its valid payload followed
that prefix, so the old browser length framing never recovered. Dropping
host bytes while the port is closed fixes the browser boundary, without
altering bearer samples, training, or LAPM bit capture. The reply buffer is
allocated from the actual decoded reply length, not from the expected input.

The comparison creates independent full and short calls with identical
channel settings. It is not persistent reuse of a measured physical-line
profile. Both calls must verify their data before a saving is reported.
Without V.8bis, connection time measured at both DTE CONNECT events was
14.02 s full versus 12.54 s Quick Connect (1.48 s saved). With V.8bis the
12000-byte qualification saved 6.52 s. The suite checks 12000-byte varied
payloads in both directions in both startup configurations, with no invalid
captured HDLC frames.

### Channel placement and limits

The new controls keep native media and the DS0 byte-exact (V.90 §5):

- Additional one-way propagation delay, 0–200 ms, is a codeword FIFO in each
  direction. It preserves every octet and the 160-sample callback cadence;
  its leading idle codewords are the line's initial state. This is additional
  to the existing one-frame browser pair pipeline. V.32/V.34/V.90/V.92 support it.
- V.32/V.34 noise, attenuation and clock drift run **after** the network D/A,
  on the synthetic linear waveform just before `me_rx_audio()`. A hook guarded
  by `__EMSCRIPTEN__` calls the browser adapter; native builds do not contain
  the hook. Original transmitted/received DS0 codewords are not re-encoded.
- Noise is repeatable, with independent endpoint seeds 1234/1235 by default.
  Light/moderate/heavy/severe map to 40/30/24/16 dB relative to a fixed
  2500-linear-unit RMS reference. These labels are not measured channel SNR.
  Attenuation is 0–24 dB in the UI; the noise floor is applied after loss.
- The waveform-only clock experiment supports ±100 ppm. A causal 65-tap
  windowed-sinc reader uses a 128-sample lookback, adding 16 ms when enabled;
  the caller still provides and consumes exactly 160 samples per callback.
  The clean synchronous path bypasses that filter. These are channel-model
  choices, not changes to ITU modem DSP constants.
- V.90/V.92 expose propagation delay only. Their byte-exact PCM receivers
  would bypass a linear-only noise model, so the other controls are disabled
  rather than claiming to impair those receivers. x2/K56flex payload demos
  retain their fixed codeword path.

With analogue impairments enabled, waveform playback/WAV show the received
linear signals, labelled accordingly. Captured G.711 remains the original
transmitted DS0. A moderate-noise V.34 row passes at 26400/26400 with a
retrain request; 6 dB attenuation and +10 ppm rows pass at 33600/33600.
A 16-dB noise-reference plus 24-dB-loss row does not connect and reports no
verified bytes or throughput. These are synthetic experiments, not hardware
qualification or a universal rate-versus-SNR model.

### What the statistics measure

Connection time is the sample-clock time when both host ports have CONNECT,
including LAPM setup. Verified bytes count the original UTF-8 message and
its returned `ACK: ` message, excluding the two-byte length headers. Useful
throughput is those verified bits divided by time from connection to complete
bidirectional verification: an aggregate application goodput, including
framing, acknowledgements, compression and round-trip effects. It is not a
physical line rate or a bit-error estimate. Failed transfers report zero
verified bytes and no useful throughput.

Retrain requests count captured `[ME]` request log entries across endpoints;
CRC-invalid frames count the passive HDLC decoder's rejected candidates.
These are observable events, not inferred modem failures or a BER. PCM and
FSK demos mark inapplicable metrics explicitly. Statistics and comparison
results follow the playback clock and do not expose future verification.
Regression tests check timing arithmetic, byte counts, disabled controls,
codeword FIFO continuity, line-setting validation, payloads under impairment,
and the generated file runner's V.92 comparison and statistics timeline.

## V.90 directional CONNECT report at 80 ms delay (2026-10-17)

Reproduced `Connected · caller TX 56000 / RX 56000` with 80 ms additional
one-way delay, with V.8bis off and on. Both payloads passed, but the displayed
upload rate was false. The digital modem's MP/upstream decoder and subsequent
analogue transmit handover both selected 31200 bit/s.

At this timing, the analogue receiver finishes downstream B1d before its
transmitter finishes upstream B1. `me_v90_analogue_rx_codewords_locked()`
therefore latched upstream=0/downstream=56000 at DATA entry. At the later DTE
CONNECT, the report callback kept that incomplete latch, and the browser's
scalar-CONNECT fallback turned the missing upload into another 56000.

V.250 §6.4.3 reporting now reads the completed analogue modem negotiation at
report time (V.90 §9.4.2), rather than the early DATA-entry latch. The browser
requires explicit directional PCM reports; it does not substitute the scalar
CONNECT value or clamp an impossible upload to a plausible rate. V.90/V.92
caller TX and RX must match the opposite endpoint's RX and TX; V.90 upload
must also remain within its 33600-bit/s QAM limit. Invalid or missing reports
stop the lab with a diagnostic instead of displaying invented speeds.

Connection events include both endpoints' structured rate observations and
label the analogue caller's upload/download. The 80-ms regressions require
31200/56000 at every connection event and 56000/31200 at the digital endpoint,
with independently verified payloads and V.8bis both off and on. The packaged
file runner checks the same delayed case and the displayed statistics labels.
The existing 0/1/20/200-ms cases reproduced correct rates before this change;
no training constants, negotiated offers or bearer samples changed for the fix.


## Sustained reliability comparisons (2026-10-17)

The Reliability test panel runs V.34, V.90 and V.92 together, or the currently
selected modem alone. Choose 1–5 fresh calls per mode and 30 or 60 seconds of
traffic after both endpoints report CONNECT. V.92 Full/Quick startup is
selected separately from the ordinary demonstration's two-call comparison;
every reliability call starts fresh and has no saved physical-line profile.
Calls rotate their mode order between rounds, use identical line settings,
and use the same repeatable binary payload seeds for each mode in a round.
All-mode comparisons permit codeword-preserving delay; waveform noise, loss
and clock drift are available for V.34-only runs because those experiments do
not affect the PCM receivers. Unsupported combinations are rejected before
starting, rather than silently changing the channel between modes.

Each direction sends a separate varied binary stream. An independent
receiver-side generator checks every decoded byte; the receiver never reads
the transmitter's buffer. Received data is drained every 160-sample step so
streams can exceed the adapter's fixed receive buffer. At most one second of
bytes is outstanding per direction (bounded to 512–8192 bytes). After the
chosen interval, up to 15 simulated seconds allow queued bytes to drain.
Training times out at 60 seconds. A pass requires both streams to match in
full, the entire traffic interval, no observed disconnect, and the requested
modulation rather than a fallback.

Only completed calls contribute to the table. Stop leaves completed results
available for JSON export and does not grade the interrupted call. Export
includes settings, per-call byte counts, directional negotiated rates,
connection times, mismatch counts, retrain requests, disconnects and failure
reasons. Connected counts include calls that later failed. Verified bytes
are received bytes for calls with no byte mismatches, including valid partial
transfers; they do not imply a passing call. Total goodput averages passing
calls' upload plus download application bits received during the selected
traffic interval, excluding the final drain. It is distinct from each
negotiated physical line rate and from a BER. Retrain counts are observed
engine request log entries, not proof that retraining completed.

Long runs retain summaries, not audio, symbol or per-bit captures. A browser
adapter flag disables those captures without changing modem negotiation or
sample accounting. Clock-drift experiments use 256 samples (32 ms) of causal
lookback for these runs, versus the ordinary demo's 128 samples (16 ms), to
cover the maximum 135-second training/traffic/drain budget at ±100 ppm.
The clean path bypasses the clock filter. G.711 codewords and the 160-sample
callback cadence remain unchanged (V.90 §5).

The reliability regression runs two 30-second calls per mode, verifies over
50,000 bytes in each direction on every call, and checks round ordering,
summary arithmetic and directional rates. A separate 60-second V.92 Full
startup run at 80 ms delay verifies over 600,000 bytes. Additional cases
exercise noisy V.34, failed training and cancellation after a completed call.
The generated offline HTML is tested with the actual inline runner, all three
modes, displayed table values, export availability and impairment gating.
These are synthetic software checks, not hardware interoperability results.


## Additional engine modulations (2026-10-17)

The lab also exposes these existing native engine paths through isolated WASM
instances. These are complete simulated calls with independently decoded
messages and acknowledgements, rather than fixed PCM payload demonstrations.

| Lab mode | Engine offer | Caller TX / RX on the clean test line | Startup / data |
| --- | --- | --- | --- |
| V.22 | `v22-1200` | 1200 / 1200 bit/s | V.8 or V.25; LAPM |
| V.22bis | `v22` | 2400 / 2400 bit/s | V.8 or V.25; LAPM |
| V.32bis | `v32bis` | 14400 / 14400 bit/s | V.8 or V.25; LAPM |
| Bell 212A | `b212` | 1200 / 1200 bit/s | Bell answer tone; LAPM |
| V.23 mode 2 | `v23c` | 75 / 1200 bit/s | Constant-carrier FSK; V.14 asynchronous characters |
| V.91 | `v91` | 52000 / 52000 bit/s | V.8; symmetric digital PCM; LAPM |

V.22 uses the existing 1200-bit/s path (V.22bis §6.3.1); V.22bis uses 2400.
V.32 remains capped at 9600 while the separate V.32bis preset permits 14400
(V.32bis §6). Bell 212A uses the engine's 2225-Hz answer-tone procedure, as
covered in `docs/bell212a.md`. Bell 212A and V.23 disable V.8/V.8bis controls.
The V.22 and V.32 families permit both V.8 and legacy startup. V.91 requires
V.8 in this lab and uses ordinary non-transparent mode; the measured 52000
rate is reported, rather than advertising a fabricated 64000 connection.

V.23's caller uploads on the 75-bit/s backward channel and receives on the
1200-bit/s forward channel (V.23 §§2/4), matching the engine's videotex
arrangement. Reports must agree with the opposite endpoint and retain the
asymmetry. The 240-byte message limit also applies here, so allow time for
the slow upload. Its statistics mark HDLC CRC counts inapplicable because
this preset uses asynchronous characters without LAPM. The received byte
comparison still checks the message and acknowledgement exactly.

Analogue channel experiments also support V.22/V.22bis/V.32bis/Bell 212A/V.23,
on the same post-D/A waveform hook as V.32/V.34. V.91 keeps exact G.711
codewords and the shared sample clock (V.91 §§6.1/6.2, V.90 §5); only
propagation delay is exposed. Playback is an audible rendering of those
codewords, not an analogue interoperation claim. V.23 and V.91 select the
spectrum display because they have no analogue QAM constellation to show.
The Reliability panel continues to compare V.34/V.90/V.92.

Regression coverage verifies UTF-8 and varied 240-byte payloads in both
directions for all six additions, exact carrier/rate reports at every
connection event, V.25 startup for the V.22/V.32 families, legacy control
gating, and the generated offline runner's controls and displayed rates.
These checks exercise software pairs, not physical modem interoperability.

Additional channel probes pass for V.22/V.22bis/V.32bis/V.23 with light noise
and 20 ms delay, Bell 212A with light noise and no added delay, and V.91 with
80 ms delay. Bell 212A with 20 ms added delay fails LAPM establishment and
reports no CONNECT or verified bytes, with or without noise. The lab calls
out this limitation; it does not disable error checking or change training
constants to manufacture a pass. The datapump-only delayed tests in
`docs/bell212a.md` do not cover this engine/LAPM failure.
