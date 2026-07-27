# V.8 overlay running standalone + TX capture attempt

## V.8 extracted and running

Extracted the V.8 bootpage (overlay `0x025f V8.F34`, page 6) to
`artifacts/eicon-dsp/v8/`. It is a full overlay that replaces DIAL (953 PM /
613 DM conflicts with DIAL -- it overwrites the DIAL code region, as expected
for a page swap). Same structure as DIAL:
- PM `0x08f0` entry (flag-conditional dispatch)
- `0x1FF4` state dispatcher (reads `0x3FB0` = bootpage_nr, sets vectors)
- `0x2000` line handler (reads RX from `0x3F08`/`0x3F09` into SR0/SR1)

Harness: `tools/v8_standalone_capture.py`. It loads V.8 directly (bypassing
the DIAL→V.8 transition, which needs the kernel task dispatcher), programs
GEN_SETUP for answer mode, feeds μ-law into `0x3F08`/`0x3F09`, and calls V.8's
handlers each frame.

## V.8 runs and is active

V.8's state machine runs: `bootpage_nr` oscillates between `0x000c` (active)
and `0x0000`/`0x0010` (LL page), and it requests pages `0x0001` (V.22) and
`0x000C` (AT-online) -- the post-negotiation transitions. TrnProgress stays
0 (no peer to negotiate with), as expected for a one-sided run.

## TX capture: partial success, real TX needs the kernel bridge

The DM-line capture (`0x3F08`/`0x3F09` after V.8 runs) shows the **input
echo**, not V.8's generated TX:
- silence in → DC out
- 440 Hz in → 440 Hz out
- 2100 Hz in → 2100 Hz out

V.8 reads `0x3F08`/`0x3F09` as RX (into SR0/SR1) but does NOT write its TX
back there. It writes processed RX to its internal buffers (`0x3994`,
`0x3999`) and the TX modulation is computed in its MAC kernels.

The actual transmitted modem signal goes out through **SPORT0 TX0**, which
is written by the **kernel's SPORT0 TX ISR** reading from a DM buffer via the
channel-table DAG walk -- the same kernel bridge that moves `0x2E52` ↔
`0x3F08`/`0x3F09`. Hooking the SPORT0 TX callback captured **0 TX0 writes**
because V.8 never writes TX0 directly; the kernel does, and we bypass the
kernel.

So: **V.8 executes and processes the line, but capturing its transmitted
modem signal requires the kernel SPORT0 TX path** (the channel-table
assignment / `dsp_assign` bridge). This is the same gap that blocks the
DIAL→V.8 transition: the kernel task dispatcher must be hooked to a task
descriptor for either the page transition or the TX sample bridge to occur.

## What works vs. what's blocked

| Capability | Status |
|---|---|
| DIAL runs standalone, processes μ-law, tone-sensitive state machine | ✓ |
| V.8 runs standalone, state machine active, requests next pages | ✓ |
| DIAL→V.8 page transition (NORM bit → load V.8) | ✗ (needs kernel task dispatch) |
| V.8 TX modem signal capture | ✗ (needs kernel SPORT0 TX bridge) |
| DIAL/V.8 RX processing (line in → state changes) | ✓ |

## The shared blocker

Both the DIAL→V.8 transition and V.8 TX capture are blocked by the same
thing: the **kernel task dispatcher** (foreground `CALL (I4)` at PM
`0x02a4`) needs a task descriptor linked in the channel table
(`0x2E40+` via `0x2E44`/`0x2E45`), which the MIPS `dsp_assign` sets up. The
queue protocol at `0x2F00`/`0x2F08`/`0x2F09` normalises any invalid queue
back to empty, so it can't be faked with a simple poke.

To complete the modem path without MIPS, the remaining work is either:
1. **Reverse the channel-descriptor format** the kernel dispatcher reads
   (the I4 vector, DAG context, state fields at `0x2E40+`) and plant a
   minimal one that vectors into DIAL/V.8's task loop -- OR
2. **Model the host supervisor** (guide §5.4.2: poll TrnProgress, load
   pages) AND wire the kernel SPORT0 TX ISR to capture TX0.

The kernel ISR at PM `0x0072` writes `DM($2E52) = RX0` (RX) but the TX
half (`TX0 = DM(...)`) reads via the channel-table DAG walk, so even running
the kernel ISR needs the channel-table hookup to produce TX.

## Harnesses

- `tools/dial_v8_supervisor.py` -- host-supervisor model (DIAL + V.8 load
  trigger); blocked by the NORM handler being reachable only via the kernel
  task loop.
- `tools/v8_standalone_capture.py` -- V.8 direct run + TX capture attempt
  (DM-line and SPORT0 TX0 paths).
