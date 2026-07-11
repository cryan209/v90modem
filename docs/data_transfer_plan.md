# Modem Data Transfer Plan: V.14, V.42/V.42bis, and DTE Handoff

## Purpose

Turn a completed physical-layer handshake into a reliable, usable modem data
service. The target is a byte-exact path between a local DTE and a remote
analog modem over V.22bis, V.34, V.90, or V.92, with the link-layer behavior a
real modem expects:

- buffered asynchronous operation using V.14 when error correction is off
- V.42 LAPM negotiation, framing, retransmission, and flow control
- optional V.42bis compression after LAPM is established
- classic inline AT/data, split control/data, and direct process handoff
- deterministic tests below the hardware-interoperability layer

This plan is deliberately independent of the datapump. The same DTE and link
stack must work regardless of whether the synchronous bit stream comes from
V.22bis, V.34, V.90/V.92, or a future V.150.1 modem-relay transport.

## Definition of done

The data-transfer work is complete when all of the following are true:

1. A negotiated V.22bis, V.34, or V.90/V.92 call does not enter DTE online
   data mode until its selected data protocol is ready.
2. With error correction disabled, arbitrary binary payloads pass through the
   V.14 path byte-for-byte, including rate adaptation and legal stop-bit
   deletion.
3. With LAPM preferred, two endpoints establish V.42, exchange payloads in
   both directions, recover from corrupt/lost frames, and disconnect cleanly.
4. With LAPM required, a non-V.42 peer causes a deterministic failure rather
   than silently falling back.
5. V.42bis is enabled only when negotiated and improves compressible-data
   throughput without corrupting or significantly penalizing random data.
6. DTE backpressure never silently drops bytes. It either propagates through
   LAPM RNR, pauses the local source, or ends the call with an explicit error.
7. Mode A and Mode B work from the command line; Mode C can hand a connected
   stream to `pppd`, `login`, or another configured child.
8. Automated unit and in-process link tests pass, followed by at least one
   byte-exact transfer and one PPP session with a real analog modem.

## Terminology and byte direction

The words *upstream* and *downstream* are overloaded in V.90, so data-stack
code and logs should use unambiguous names:

- **DTE TX / line TX**: local application -> modem -> remote modem
- **line RX / DTE RX**: remote modem -> modem -> local application
- **calling party / answering party**: LAPM role, fixed for the life of a call

All public data-stack APIs should use these names. Existing engine ring names
can be migrated separately to avoid mixing a risky rename with protocol work.

## Current repository baseline (2026-07-11)

| Area | Current state | Gap |
|---|---|---|
| `modem_engine.c` | V.22bis, V.34, and live V.90 use the common V.14 stack; V.90 preserves its non-byte-aligned mapper reservoir | Add bounded queues/backpressure and validate each live path with real hardware |
| `data_stack.[ch]` | Implements raw diagnostics and 8N1 V.14 framing, fractional stop-bit deletion, underspeed mark insertion, and packed-byte helpers | Add bounded queues/backpressure, then the LAPM mode |
| `data_stack_test.c` | Normal build/test target covering round trips, long-run rate ratios, V.90-style 29-bit frames, idle mark, deleted stop bits, invalid input, reset, packed boundaries, and bursts | Add queue/overflow cases when bounded queues land |
| `data_interface.[ch]` | Mode A, guarded `+++`, `ATO`, and a Mode B two-PTY scaffold exist | Still singleton-based; split mode has no CLI wiring; partial writes, close semantics, and policy commands need hardening |
| `sip_modem.c` | Opens only the classic `--pty-link` interface | No split-port or exec configuration |
| SpanDSP `v42.c` | LAPM detection, HDLC, XID, windowing, retransmission, bit callbacks, a configurable timer rate, and public detection statuses exist | XID result exposure, busy/release/retry qualification, and live integration remain |
| `v42_link_test.c` | Tests detection, byte-exact duplex transfer, T400 at 2400/9600/28800/33600 bit/s, corrupted-frame retransmission, RNR/RR, DISC/UA, and sustained-outage retry exhaustion | Add public negotiated-XID result checks when that API lands |
| SpanDSP `v42bis.c` | Compressor/decompressor with dynamic/always/never modes exists | Must be driven by negotiated XID values, not fixed sample defaults |
| V.92 startup | Tracks the peer LAPM P bit and whether ODP/ADP may be bypassed | Result must be carried into the live data-stack start policy |

The common V.14 seam now covers the implemented V.22bis, V.34, and live V.90
paths. LAPM is being qualified in isolation before it is selected by live
calls.

## Layer model

```text
 DTE/process side                                      line/SIP side
 ----------------                                     -------------
 PTY, socket, or child process
     <-> AT command and result-code service
     <-> online byte stream
             <-> optional V.42bis compression
             <-> V.42 LAPM ----------------------+
          or <-> V.14 async rate adaptation ------+-> synchronous bits
          or <-> raw diagnostic framing ----------+       <-> datapump
                                                          <-> G.711 RTP
```

Two HDLC-shaped protocols must remain separate:

1. **V.42 LAPM** is a modem-to-modem link layer. It is negotiated by the
   modems and is invisible to the DTE.
2. **PPP async HDLC** is ordinary DTE payload. The modem must not parse or
   alter it, except for optional initial sniffing used to choose an exec
   handoff target.

The normal online path contains either LAPM or V.14, never V.14 inside LAPM.
LAPM accepts and delivers octets; V.14 adapts asynchronous characters to the
datapump's synchronous bit stream only when LAPM is not selected.

## Scope and non-goals

### In scope

- 8N1 V.14 buffered mode and rate adaptation
- V.42 ODP/ADP detection when required
- ODP/ADP bypass when both V.92 P bits validly advertise LAPM
- LAPM establishment, data, flow control, recovery, and release
- V.42bis negotiation and compression
- Mode A classic PTY, Mode B split PTYs, and Mode C exec handoff
- result codes and the minimum AT policy controls needed to select the above
- deterministic fault-injection and integration tests

### Deferred

- MNP2-4/MNP5 fallback; buffered V.14 is the initial non-LAPM fallback
- V.44, because SpanDSP has no implementation and LAPM must be stable first
- V.80 synchronous DTE access and TCP DTE listeners
- V.150.1 SPRT/SSE modem relay
- fax modes, which use a different stack
- general serial formats beyond 8N1 until the core path interoperates

## DTE presentation modes

### Mode A: classic inline serial (default)

One PTY carries AT commands while offline and payload while online. A guarded
`+++` enters online command mode; `ATO` returns to data mode without
renegotiating the line.

Required behavior:

- do not send `CONNECT` until V.14 is selected or LAPM is established
- preserve bytes withheld during a failed `+++` candidate
- use monotonic time for both pre- and post-sequence guard periods
- keep the physical carrier and link state alive in online command mode
- define `ATH`, DTE close, and process shutdown behavior in every call state
- report `NO CARRIER` exactly once per ended call

PTYs do not carry real DCD, DTR, CTS, or RTS. `&C`/`&D` behavior can only be
emulated using open/close events and result codes; clients must use `CLOCAL`.

### Mode B: split control and data ports

The existing two-PTY scaffold becomes a supported CLI mode:

```text
--control-pty /tmp/modem-control --data-pty /tmp/modem-data
```

- The control PTY always accepts AT commands and receives unsolicited results.
- The data PTY transports payload only while the data protocol is ready.
- `+++` has no special meaning on the data PTY.
- Bytes written before connect are rejected or held behind a bounded,
  explicitly reported queue; the initial implementation should reject them.
- Closing the data PTY does not implicitly destroy the control interface.

PTY-only Mode B is the first cut. A TCP listener is a later transport plugged
into the same port abstraction, not part of LAPM work.

### Mode C: direct exec/ISP handoff

After the link protocol is ready, start a configured child on a fresh PTY pair
and bridge its byte stream to the data stack:

```text
--exec -- /usr/sbin/pppd nodetach local noauth 192.0.2.1:192.0.2.2
--exec -- /usr/bin/login
```

Use an argv vector after `--`, not a shell command string. This avoids quoting
ambiguity and command injection. Define child environment, uid/gid, working
directory, stderr destination, termination grace period, and restart policy.
The default is one child per successful call, no automatic restart during that
call, SIGHUP on carrier loss, then SIGTERM/SIGKILL after bounded grace periods.

AUTOPPP sniffing is optional and comes after explicit `--exec` works. Sniffed
bytes must be replayed unchanged to the selected child.

## Target component boundaries

### 1. `dte_port`: transport only

Remove PTY-specific and global state from the online byte path. A transport
object owns file descriptors/processes and reports readiness; it does not know
about LAPM, compression, or modem symbols.

```c
typedef struct dte_port dte_port_t;

typedef struct {
    ssize_t (*read)(dte_port_t *p, uint8_t *buf, size_t cap);
    ssize_t (*write)(dte_port_t *p, const uint8_t *buf, size_t len);
    int     (*set_online)(dte_port_t *p, bool online);
    void    (*close)(dte_port_t *p);
} dte_port_ops_t;
```

Contract:

- nonblocking reads return `0` for no data and negative errno-style failures
- writes may be partial and the caller retains the unwritten suffix
- `set_online(false)` stops payload without necessarily closing the port
- close/hangup events are delivered once through a separate owner callback
- Mode A may share one underlying PTY with the AT service; Mode B and C do not

The AT interpreter and call controller own command mode, result codes, dialing,
answering, and hangup. `dte_port` owns byte transport only.

### 2. `data_stack`: per-call link protocol

Replace the framing-only structure with a per-call object that has explicit
configuration, state, and events:

```c
typedef enum {
    DS_POLICY_BUFFERED_ONLY,
    DS_POLICY_LAPM_PREFERRED,
    DS_POLICY_LAPM_REQUIRED
} ds_ec_policy_t;

typedef enum {
    DS_DOWN,
    DS_DETECTING,
    DS_ESTABLISHING,
    DS_ONLINE_V14,
    DS_ONLINE_LAPM,
    DS_DISCONNECTING,
    DS_FAILED
} ds_state_t;

typedef struct {
    bool calling_party;
    int line_tx_bit_rate;
    int line_rx_bit_rate;
    int dte_bit_rate;
    ds_ec_policy_t ec_policy;
    bool peer_lapm_known;
    bool peer_lapm;
    bool bypass_odp_adp;
    bool allow_v42bis;
} ds_config_t;
```

Required operations:

- initialize once per call with fixed calling/answering role
- start only after the datapump supplies stable data-bit clocks
- pull one line-TX bit and accept one line-RX bit
- optionally fill/consume packed LSB-first bits for the V.90 mapper
- accept DTE TX bytes and deliver DTE RX bytes through bounded queues
- expose state/protocol/rates/counters without exposing SpanDSP internals
- request orderly link release and force-reset on carrier loss

The legacy raw framing mode remains test/debug-only and must require an
explicit option. It must never be negotiated with a real modem.

### 3. Datapump adapter

Every datapump uses the same semantic callbacks:

```c
int  me_line_tx_get_bit(void *ctx);       /* always returns a clocked 0/1 */
void me_line_rx_put_bit(void *ctx, int bit_or_status);
```

- V.22bis and V.34 call the bit API directly.
- V.90's mapper consumes packed bits derived from the same bit API; partial
  six-symbol frames retain their exact bit position across RTP pulls.
- Negative SpanDSP signal-status values are handled by the datapump adapter
  and are never passed into V.14 or LAPM as data bits.
- The link stack is reset on retrain. Whether a fast retrain preserves LAPM
  is a later, explicit feature; the safe first behavior is disconnect/restart.

### 4. Call controller

The SIP/main-loop controller owns call lifecycle. The proposed sequence is:

```text
SIP connected
  -> V.8 and physical-layer training
  -> datapump data clock stable
  -> data_stack start
       -> V.14 immediately ready, or
       -> V.42 detect/bypass -> LAPM establish -> optional compression
  -> DTE online + CONNECT result
  -> payload transfer
  -> LAPM release when possible
  -> carrier teardown + NO CARRIER
```

S0 should be the single owner of auto-answer policy. The SIP layer reports
rings and obeys the AT/profile decision; it must not maintain an independent
hard-coded two-ring policy.

## Protocol selection and negotiation

### Policy inputs

The effective per-call policy is frozen before the data stack starts:

| Configuration | Peer capability | Result |
|---|---|---|
| buffered only | any | enter V.14; do not send ODP |
| LAPM preferred | both V.92 P bits = 1 | bypass ODP/ADP and establish LAPM |
| LAPM preferred | capability unknown | run ODP/ADP; fall back to V.14 on the defined unsupported outcome |
| LAPM preferred | peer explicitly says no LAPM | enter V.14 |
| LAPM required | confirmed or detected LAPM | establish LAPM |
| LAPM required | unsupported/timeout | fail the connection and report no carrier/error |

V.8/V.92 capability is evidence, not a second link state machine. Copy the
final verdict into `ds_config_t`, then let `data_stack` own all subsequent
protocol state.

### SpanDSP V.42 integration work

Do not wire `v42_tx_bit()` straight into the engine and assume it is complete.
First isolate it behind a small adapter and verify or patch these points:

1. **Timer rate:** `v42_init()` currently sets `tx_bit_rate = 28800`, while
   T400/T401/T403 advance from transmitted bit calls. Add a supported setter or
   explicit clock/tick API and configure the actual line-TX bit rate whenever
   it changes. V.22bis at 2400 bit/s must not use 28.8 kbit/s timers.
2. **Stable status API:** the callback currently exposes a mixture of public
   `SIG_STATUS_*` values and internal LAPM states. Add named public events for
   detection-supported, detection-unsupported, link-connected,
   link-disconnected, and link-error; do not make application code compare
   private numeric state values.
3. **Detection fallback:** verify caller and answerer T400 paths separately.
   A preferred-policy timeout selects V.14; a required-policy timeout fails.
4. **Start/restart/release:** test `v42_start`, `v42_restart`, `v42_stop`, DISC,
   and carrier-loss cleanup for leaks, duplicate events, and stale frames.
5. **XID application:** verify N401, window `k`, compression direction P0,
   dictionary P1, and maximum string P2 are parsed, bounded, applied in the
   correct direction, and observable through a public negotiated-result API.
6. **Queue callback semantics:** confirm zero-length/no-data, partial payload,
   and maximum-frame behavior. The adapter must never treat an empty DTE queue
   as end-of-carrier.
7. **HDLC fault behavior:** validate FCS rejection, aborts, flag sharing, REJ,
   RNR/RR, retry exhaustion, and sequence-number wraparound with tests.

Keep these changes as small, documented patches in the vendored SpanDSP tree;
do not reach into `spandsp/private/v42.h` from modem application code.

### V.14 requirements

The current 8N1 encoder/decoder is a useful base but is not full rate
adaptation. The completed V.14 path must:

- generate start bit, eight LSB-first data bits, and stop mark
- maintain continuous mark while idle
- use a rational/fixed-point rate accumulator between DTE character rate and
  the line data-bit clock
- delete only the permitted stop bits when the DTE rate exceeds available
  line capacity; never delete start or data bits
- accept legal deleted stop bits on receive without losing character phase
- count framing errors separately from deleted stop bits
- define recovery after illegal/missing bits and after retrain/reset
- support independent TX and RX line rates, required by PCM modem asymmetry

The configured DTE rate is a virtual modem setting, not something reliably
inferred from a PTY's termios speed. Start with an explicit/default DTE rate
and report both DTE and line rates.

### V.42bis placement

```text
DTE TX -> compressor -> LAPM I-frame payload -> line
line -> LAPM accepted I-frame -> decompressor -> DTE RX
```

Compression starts only after LAPM XID completes. Initialize compressor and
decompressor with the negotiated directional parameters, use dynamic mode by
default, flush only at defined protocol boundaries, and reset dictionaries on
new calls or negotiated reset. A compression error is a link error; never emit
partially decoded data as if valid.

## Buffering, flow control, and overload

Silent truncation is unacceptable for modem data. Replace boolean/void queue
callbacks with byte counts and explicit full/error results.

Suggested initial bounds:

- DTE TX queue: 64 KiB
- DTE RX queue: 64 KiB
- LAPM payload staging: bounded by negotiated N401 and window `k`
- exec/PTY pending-write queue: 64 KiB

Exact sizes may be tuned, but behavior must be fixed:

- **LAPM RX pressure:** assert local busy with RNR before the DTE RX queue is
  exhausted; send RR after it drains below a low-water mark.
- **LAPM TX pressure:** stop reading the DTE when the local TX queue is full.
- **V.14 pressure:** PTYs provide no in-band modem flow control. Stop reading
  locally where possible; if already-demodulated RX data cannot be retained,
  log an overrun and terminate rather than silently corrupting the stream.
- **Partial PTY writes:** retain the unwritten suffix and retry on writable
  readiness; treat EAGAIN as backpressure, not data loss.
- **Shutdown:** define whether queued bytes drain. Orderly local hangup may
  drain for a bounded interval; carrier loss discards queues immediately and
  records counters.

Queue depths, high-water events, RNR duration, retransmissions, FCS failures,
and discarded bytes must be visible in diagnostics.

## AT configuration and reporting

Implement one canonical internal profile, then map compatible command aliases
onto it. Do not let `AT\N`, `AT+ES`, and vendor-style commands maintain
independent settings.

Minimum first-cut controls:

- `AT+ES` or `AT\N`: buffered-only, LAPM-preferred, LAPM-required
- `AT+DS` or `AT%C`: compression disabled/enabled
- `ATS0=n`: auto-answer ring count; `S0=0` disables auto-answer
- `ATO`: leave online command mode
- `ATH`: orderly link release followed by call teardown
- `AT&F`/`ATZ`: restore/reset the complete data profile consistently

Result reporting needs a structured source of truth, even if the first output
is traditional text:

```text
CONNECT 56000
PROTOCOL: LAPM
COMPRESSION: V.42BIS
LINE: V.90 56000/31200
```

Respect quiet/numeric/verbose modes. `CONNECT` should represent DTE readiness,
not merely SIP answer or carrier detection. Store a machine-readable call
result containing modulation, line TX/RX rates, DTE rate, protocol,
compression, and termination reason.

## Threading and ownership

- Datapump and data-stack bit functions run in the RTP/sample-clock domain.
- PTY/socket/child I/O runs in a DTE I/O thread or event loop.
- Bounded queues are the only cross-thread payload boundary.
- The main/call-controller thread owns state transitions and user-visible
  events. RTP callbacks enqueue state events rather than invoking AT/SIP
  teardown recursively from inside SpanDSP.
- LAPM timers advance from a deterministic data-bit clock, not wall time.
- Each call has a generation ID. Deferred events from an old call are ignored
  after teardown or redial.
- No callback may hold a queue mutex while calling SpanDSP, writing a PTY, or
  acquiring the global engine-state mutex.

This also removes the current risk of callbacks tearing down an active modem
context from inside that context's receive function.

## Implementation sequence

Each milestone must land with its tests and leave the default path usable.

### Milestone 0: make the current scaffold a green baseline

Status: **in progress**. The normal test target and framing/reset coverage are
complete; the scripted raw V.22bis baseline and call diagnostics remain.

- Add `data_stack.o` to the relevant build only when it is used.
- Add `data_stack_test` to Makefile build/check targets now.
- Add reset, malformed stop-bit, packed-boundary, and queue-empty tests.
- Script a raw legacy V.22bis loopback baseline and record current behavior.
- Add protocol/rate/counter fields to a read-only call diagnostics structure.

Exit gate: clean build plus existing tests and `data_stack_test`; no runtime
behavior change.

### Milestone 1: integrate complete V.14

Status: **implementation complete; hardware validation pending**. Rate
adaptation is wired through V.22bis, V.34, and live V.90 data mode.

- Extend `data_stack` with TX rate adaptation and explicit RX framing errors.
- Replace V.22bis and V.34 raw-byte accumulators with the common bit seam.
- Feed V.90 packed input through the same stack without breaking six-symbol
  frame boundaries.
- Reset framing on call start, retrain, and carrier loss.
- Make V.14 the default real-peer path while raw remains an opt-in diagnostic.

Exit gate: bidirectional binary transfers over in-process V.22bis and the
available V.34/V.90 data paths are byte-exact at multiple DTE/line rate ratios.

### Milestone 2: qualify the SpanDSP LAPM core in isolation

Status: **in progress**. The bit-pipe harness, configurable line-rate timers,
public detection statuses, multirate T400 checks, duplex transfer,
corrupted-frame recovery, RNR/RR, DISC/UA, and sustained-outage retry
exhaustion are complete. Public negotiated-XID results remain.

- Build a two-ended bit-pipe test around two `v42_state_t` instances.
- Add the timer-rate and stable-status/config APIs described above.
- Cover caller/answerer detection, bypass, XID, data, DISC, and forced loss.
- Inject bit flips, deleted bit ranges, duplicated ranges, and long stalls.
- Assert delivered bytes, retransmission counts, state sequence, and timeout
  measured in line bits.

Exit gate: deterministic LAPM tests pass at 2400, 9600, 28800, and 33600 bit/s
with sanitizers enabled.

### Milestone 3: integrate V.42 policy and flow control

- Add `DS_POLICY_*` configuration and the link state machine.
- Carry V.8/V.92 LAPM and ODP/ADP-bypass verdicts into each call.
- Delay DTE `CONNECT` until LAPM is established or V.14 fallback is selected.
- Connect queue watermarks to RNR/RR and local DTE read throttling.
- Implement orderly release and explicit retry-exhaustion behavior.
- Add protocol result reporting.

Exit gate: in-process full-call tests select required/preferred/disabled modes,
survive error bursts without payload corruption, and fail as configured when
LAPM is unavailable. Then establish LAPM with one real analog modem.

### Milestone 4: add negotiated V.42bis

- Surface negotiated P0/P1/P2 from LAPM XID.
- Insert compressor/decompressor callbacks on the correct directional paths.
- Add disabled, dynamic, and peer-refused cases.
- Add dictionary reset, flush, incompressible-data, and corruption tests.
- Add compression result reporting and byte/coded-byte counters.

Exit gate: text achieves a repeatable throughput gain; seeded random data is
byte-exact and does not show an unacceptable throughput regression.

### Milestone 5: finish DTE modes and AT behavior

- Refactor the existing PTY code behind `dte_port` without changing Mode A.
- Wire `--control-pty`/`--data-pty` to the existing split-mode scaffold.
- Correct partial writes, close handling, `+++` edge cases, `ATO`, and results.
- Add the canonical EC/compression profile and command aliases.
- Move auto-answer policy to S0 and remove the independent SIP ring count.

Exit gate: an automated PTY integration test dials, transfers data, enters and
leaves online command mode, hangs up, and repeats without stale bytes or events.
The same transfer works through split ports with payload containing `+++`.

### Milestone 6: implement exec handoff

- Add argv-safe `--exec -- ...` parsing and per-call child lifecycle.
- Bridge partial nonblocking reads/writes through `dte_port` queues.
- Drop privileges and define environment/logging/timeouts.
- Add optional replay-safe AUTOPPP detection only after explicit exec works.

Exit gate: a child `cat`/test program passes binary loopback; a login program is
interactive; a real analog caller establishes a PPP session terminated by
host `pppd`.

### Milestone 7: hardware and impairment hardening

- Test multiple real modem chipsets and both call directions where supported.
- Exercise PCMU first, then PCMA if V.90 mapping is validated for it.
- Repeat at V.22bis, V.34, V.90, and V.92 negotiated variants.
- Add RTP loss/reorder/jitter tests at the G.711 link layer and distinguish
  physical retrain failures from LAPM recovery failures.
- Record negotiated parameters and artifacts for every failed interop case.

Exit gate: the interoperability matrix below has no unexplained corruption;
known unsupported cases fail cleanly and are documented.

## Test plan

### Unit tests

- V.14 encode/decode for all byte values and randomized streams
- rate-accumulator ratios, long-run drift, idle gaps, and reset boundaries
- legal deleted stop bits and illegal framing patterns
- queue wrap, full/empty, partial reads/writes, and watermarks
- AT profile parsing and result formatting
- V.42bis reference round trips for compressible and seeded random corpora

### Link-layer bit-pipe tests

Run two data stacks without audio DSP. Fault injection must be reproducible by
seed and recorded as line-bit offsets:

- single and burst bit flips
- complete frame loss
- duplicated and reordered test chunks (to emulate a faulty lower harness)
- stalls long enough to trigger T401/T403
- receiver busy intervals and recovery
- sequence-number wraparound and retry exhaustion
- disconnect during queued data and during XID

LAPM guarantees byte ordering and uniqueness, not recovery from indefinite
outage. Tests must distinguish successful recovery from the expected explicit
link failure.

### Full in-process calls

Extend the existing `vpcm_call_pair`/`vpcm_link` harness rather than scripting
two external SIP processes. Each case records:

- modulation and directional line rates
- advertised and selected EC/compression policy
- state transition timestamps in samples/bits
- payload hashes and lengths in both directions
- retransmissions, FCS failures, RNR duration, queue high-water marks
- final disconnect reason

### PTY/process integration

Use a small test client that opens PTYs in raw/CLOCAL mode. Cover Mode A escape
guard timing, Mode B simultaneous control and binary payload, child exit, DTE
close, redial, and shutdown. Do not depend on `minicom` for automated checks.

### Real-modem matrix

| Case | EC | Compression | Payload/fault | Required result |
|---|---|---|---|---|
| V.22bis baseline | off | off | binary file | byte-exact V.14 |
| V.34 baseline | off | off | binary file | byte-exact V.14 |
| V.90/V.92 baseline | off | off | both directions | byte-exact V.14 |
| LAPM normal | preferred | off | binary file | LAPM, byte-exact |
| LAPM impairment | required | off | controlled RTP loss | recover or explicit failure, never silent corruption |
| V.42bis text | required | dynamic | compressible corpus | byte-exact and faster than LAPM-only |
| V.42bis random | required | dynamic | seeded random corpus | byte-exact, bounded overhead |
| PPP | preferred | negotiated | LCP/IP traffic | stable IP session and clean teardown |

For each physical modem record vendor/model, firmware, AT profile, negotiated
modulation/rates, codec, SIP/RTP settings, and a timestamped modem trace.

## Observability

Add one concise state-transition log and one final per-call summary. Avoid a
line per bit/frame in normal mode.

Required transition events:

- datapump data clock ready/lost
- V.42 detection started, supported, unsupported, or bypassed
- XID requested/completed with negotiated parameters
- LAPM connected, busy/unbusy, recovery, disconnected, or retry exhausted
- compression enabled/disabled and reset
- DTE online/offline, escape, close, and child exit

Final counters:

- DTE TX/RX bytes
- LAPM information bytes/frames, retransmissions, rejects, FCS errors
- V.42bis input/output bytes by direction
- queue maxima, RNR count/duration, overrun/discard counts
- call end reason and last protocol state

Sensitive payload bytes must not be logged by default. Hex dumps require an
explicit debug option and should be bounded.

## Failure and teardown rules

| Event | Required action |
|---|---|
| V.42 unsupported, preferred | select V.14 and then issue `CONNECT` |
| V.42 unsupported, required | fail link setup; do not expose online data |
| LAPM retry exhaustion | mark link error, stop DTE payload, tear down carrier |
| V.42bis decode failure | link error; never forward uncertain output |
| DTE TX queue full | stop local reads until low-water |
| DTE RX queue cannot drain under LAPM | RNR, then timeout/fail if persistent |
| DTE RX overrun under V.14 | report overrun and disconnect |
| physical carrier loss | force-reset stack, discard queues, one `NO CARRIER` |
| local `ATH` | attempt bounded LAPM DISC, then SIP hangup |
| child exits | orderly hangup unless configured to keep carrier for another handoff |
| application shutdown | stop new calls, bounded release, close children and PTYs |

## Expected file-level work

- `data_stack.[ch]`: per-call V.14/V.42/V.42bis state and queues
- `data_stack_test.c`: framing/rate/queue unit tests
- new `v42_link_test.c`: isolated two-ended LAPM and fault injection
- `modem_engine.[ch]`: common bit seam, call-result events, no raw DTE framing
- `data_interface.[ch]`: AT service plus `dte_port` implementations
- `sip_modem.c`: CLI, call-policy ownership, deferred event handling
- `vpcm_call_pair` / `vpcm_link`: full-call data and impairment scenarios
- vendored SpanDSP V.42 public header/source: minimal timer, event, and
  negotiated-parameter APIs with corresponding tests
- `Makefile`: all new tests in normal `make check`/test targets
- `README`: PTY CLOCAL limitation, CLI modes, AT profile, and PPP example

## V.150/V.152 boundary

The present SIP media path is V.152-style voice-band data: transparent G.711
transport with transcoding, VAD/CNG, and echo cancellation disabled. No new
data-stack layer is required for that label.

V.150.1 is different: a gateway terminates the physical modem and transports
data/state using SPRT/SSE. It would replace the local datapump/bit adapter, not
the DTE, LAPM policy, compression, or handoff layers. Keeping the boundary at
the synchronous line-bit interface preserves that future option without
adding V.150.1 to this project phase.

## Decisions fixed by this plan

- PTY-only split mode before adding TCP.
- Explicit argv-safe exec before AUTOPPP sniffing.
- S0 owns auto-answer; SIP only reports and executes call policy.
- V.14 is the fallback when LAPM is preferred but unavailable.
- LAPM-required never silently falls back.
- V.42bis follows negotiated XID values; hard-coded parameters are test-only.
- Raw byte framing is diagnostic-only.
- Queue overflow is explicit and must never silently corrupt payload.

## Remaining product choices

These do not block Milestones 0-3:

- the default virtual DTE rate and exact extended result-code dialect
- whether child programs run under a configured service account or inherit the
  daemon account in the first Mode C release
- whether a future TCP data port is outbound, listening, or supports both
- which additional serial formats (7E1, 8E1, 8N2) follow the 8N1 milestone

## References

- `docs/v90_live_g711_plan.md`
- `docs/vpcm-refactor-plan.md`
- `docs/offline_v34_decoder_plan.md`
- `ITU Docs/T-REC-V.42-200203-I!!PDF-E.pdf`
- local V.42bis, V.44, V.80, V.150.0, V.150.1, and V.152 recommendations
- vendored SpanDSP `v42.c`, `v42bis.c`, `async.c`, `hdlc.c`, and tests
