# AT-controlled modem diagnostics

The modem now implements a **local digital interface loop** and a finite
bit/block-error test controller through V.250 (07/2003) §6.7.2 commands.
This is a subset of the diagnostic interface, not full V.54 conformance.
It applies to the DTE interface independently of the modulation in use.

## Local digital loop

V.250 §6.7.2.13 describes `+TLDL` as returning the local DTE's characters to
that DTE. It is not the far-end receiver-to-transmitter loop requested by
`+TRDL`, and it does not send the local transmitter's audio into its receiver.
V.54 (11/1988) §3 distinguishes the DTE/interface loop, the remote digital
loop (loop 2) and the local analogue loop (loop 3).

Start the loop only after a data connection has reached CONNECT, in **Online
Command State**. In the combined PTY presentation, use guarded `+++` to leave
data mode. In the split presentation, the control PTY already remains in
command state and the data PTY is the looped interface.

```text
AT+TLDL=1
OK
AT+TLDL?
+TLDL: 1
OK
```

On the combined port, issue `ATO` and write payload bytes; they return on the
same PTY. On split ports, write to/read from the data PTY. All 256 byte values
are preserved, subject to the combined port's normal guarded escape handling.
Loop bytes have a separate queue and never enter the modem's transmit-data
queue. Incoming peer data is clamped from the DTE during the loop. The
existing bearer/datapump/error-control connection continues to run.

Use guarded `+++` again on the combined port, then `AT+TLDL=0` to stop.
The stop discards pending loop output and restores the ordinary DTE/line
routes. Disconnect, changing to a fax class, `ATZ` and `AT&F` also stop it.
A loop start/stop requires an active data call in Online Command State;
queries and capability tests are available while idle.

## Finite error-rate tests

With the local loop active, issue:

```text
AT+TTER=3,511,100,1
OK
AT+TTER?
+TTER: 3,511,100,1
OK
AT+TNUM?
+TNUM: 0,0
OK
```

The exact remaining block count depends on how long the test has run.
`+TTER=<type>,<block_length>,<blocks>,<pattern>` follows V.250 §6.7.2.11:

| Argument | Supported values |
|---|---|
| type | 1 bit errors, 2 block errors, 3 both; `+TTER=0` stops |
| block length | 1..65535 bits |
| blocks | 1..65535 |
| pattern | 1: 511-bit, 2: 2047-bit, 3: all ones, 4: alternating |

The 63-bit pattern (code 0) is unsupported and returns ERROR. `+TTER=?`
advertises the implemented subset. The nine-stage and eleven-stage generators
are SpanDSP's O.153_9 and O.152_11 respectively; the two ends of the loop keep
independent transmit/receive pattern positions.

The test stays in Command State and finishes after the requested number of
complete blocks. Type becomes 0 and remaining blocks becomes 0. A block is
counted as errored once even if it contains multiple incorrect bits; an
unfinished block is not a complete errored block. `+TTER=0` retains completed
measurements. Disconnect and loop termination retain the last error totals;
a new test or `ATZ`/`AT&F` clears them.

`+TNUM?` reports the current/last bit and block errors (§6.7.2.12). A count not
requested by the test type is reported as zero. Internal totals are 64-bit;
the public values saturate at 65535 instead of wrapping. The Recommendation's
example uses a `+TTER:` prefix for `+TNUM?`; this implementation uses `+TNUM:`
consistently with the queried parameter's name.

**This test clocks the software local interface bit loop.** It does not cross
the PTY slave, codec, modulation datapump, SIP path or remote modem. Its clock
is paced by the reported CONNECT rate in the PTY reader; the progress cadence
is approximate and machine suspension does not manufacture hours of results.
A zero result here establishes only that the installed software bit loop
returned its test pattern. For measurements through the native datapumps,
use [V.56 loopback](v56_loopback.md) or [PCM loopback](pcm_loopback_testing.md).
During `+TTER`, DTE payload is ignored rather than echoed; normal local echo
resumes once the test completes or is stopped.

## Unsupported diagnostics report honestly

`+TMODE?` reports point-to-point (0); `+TMODE=?` advertises only `(0)`.
Multipoint/tandem mode (1) is rejected.

`+TRDL=1` is rejected: there is no implemented V.54 peer request/confirmation
path. V.250 §6.7.2.14 forbids reporting OK without that confirmation.
`+TRDL?` and `+TRDLS?` report inactive (0). Local analogue-loop actions
(`+TAL`) are rejected, and `+TALS?` reports inactive. Address selection,
front-panel/circuit enable controls and local digital-loop status `+TDLS`
are not implemented and return ERROR.

`+TSELF=1` implements §6.7.2.16's safe partial check for this software DCE.
It allocates 4 KiB of scratch working memory and verifies six byte patterns,
checks a known host-arithmetic result, and exercises the local test controller
with both clean and deliberately corrupted bit streams. Volatile accesses
ensure the memory/arithmetic checks execute rather than being folded away by
the compiler. Allocation/check failure gives a self-test result of 2.

The partial check uses separate state, leaves a live loop/BER test intact,
and does not reset a datapump or interrupt a call. It is a cursory host
CPU/working-memory/controller check, not an exhaustive RAM, analogue hardware,
codec or modem-interoperability test. `+TSELF=?` advertises only `(1)`;
intrusive full self-test `+TSELF=0` remains unsupported and returns ERROR.
`+TRES?` reports the actual last partial-check result (0 not run, 1 pass,
2 fail), preserved until reset. It never substitutes a BER count for that
result. Hayes/Courier `AT&T` aliases are not implemented.

## Verification

```sh
make at_test_test
./at_test_test
```

The test exercises the real T.31 AT interpreter on both combined and split
PTYs, using a simulated established carrier. It verifies binary local echo,
no forwarding to the peer, remote-data clamping, restored normal traffic,
guarded escape/ATO, finite BERT completion, partial self-test/result retention,
disconnect/reset and unsupported commands. Core tests inject multiple errors into two distinct blocks, check
retained/saturated counts and prove grading stops at the finite target.
This is automated software coverage, not a hardware or remote V.54 test.
