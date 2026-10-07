# Protocol audit using BinModem's specification digests

Date: 2026-10-07. This is a focused source audit, not a hardware interoperability result.

## Sources and coverage

The local BinModem checkout is at
`/Users/scottcryan/Documents/Codex/2026-10-07/https-github-com-casualarclamp-binmodem-https/work/BinModem`.
Its `docs/design/v92/spec-phase4-procedures.md` and
`spec-phase4-signals-digital.md` supplied the requirement checklist. Suspected
procedure defects were checked against our original V.92 (11/2000), clauses
9.6.1.1.3 and 9.6.2.1.3-.4, in `ITU Docs/`.

Reviewed native digital and analogue Phase 4 control transmission, retry
decisions, acknowledgement handling, the shared downstream Ed detector,
SUVd framing/CRC handling and the enclosing analogue timeout. Read the
V.34 half-duplex digest and our existing HDX audit for context; that comparison
is not a new complete HDX audit. V.92 Phases 1-3, quick connect, modem on hold,
renegotiation/FPE, and the V.32 constellation tables remain outside this pass.

## Finding 1 — P2: CP recovery is reset after each retransmission

Locations: `v90.c:4488-4501`, `v90.c:5858-5863`,
`v92_analogue_phase4.c:156-158`, `v92_analogue_phase4.c:183-202`.

V.92 9.6.1.1.3 and 9.6.2.1.3 require repeated CPd/CPu sequences when no
acknowledgement is received through the complete peer sequence after the
initial CP end plus 100 ms and RTD. That switches the transmitter from the
initial single-CP-plus-SUV exchange to repeated CP sequences.

The digital endpoint instead clears `v92_cpd_retry`, overwrites
`v92_cpd_end`, and unconditionally queues SUVd after every CPd, including
a retry. The analogue endpoint clears `cpu_retry` when queuing CPu,
overwrites `cpu_end` at its completion, and normally returns to SUVu.
Consequently both restart the initial waiting procedure after each retry.

Concrete source trace: initial CP -> SUV until expiry -> retry CP -> SUV
until a new expiry. If subsequent incoming control messages are corrupt or
absent, no new receive callback decision reasserts retry, and the endpoint
can remain on SUV indefinitely until the enclosing timeout. Repeated CPs
are particularly needed when the receiver still lacks the parameters.

Suggested correction: keep a persistent CP-repeat state once the initial
deadline decision fires; transmit CPs at each complete-message boundary
until termination, keeping the first deadline anchor separate. Preserve
the distinct clause 9.11 cleardown behavior.

Needed regression: suppress acknowledgement beyond the initial expiry,
verify consecutive CP messages, then corrupt/drop further incoming control
messages and verify CP transmission continues. Exercise both roles and laws.

## Finding 2 — P2: analogue completion requires an acknowledgement even when Ed arrives

Locations: `v92_analogue_phase4.c:100`, `v92_analogue_phase4.c:197-199`,
`v92_analogue_phase4.c:260-264`; shared detector
`v90_analogue_phase4.c:662-689`.

V.92 9.6.2.1.4 permits completion after sending an acknowledged CPu/SUVu
and receiving either an acknowledged CPd/SUVd **or Ed**. This matters when
the peer received our acknowledged message and moved on, but its own final
acknowledged control message was damaged on the way back.

The analogue transmitter's E2u transition requires `remote_ack`.
That flag is updated only by decoded CPd/SUVd acknowledge bits. The RX
function handles the DATA event but never turns `V90A4_RX_EVENT_ED` into
completion evidence. Furthermore the control callback returns true, arming
the shared Ed detector, only when `cpd_seen && remote_ack`; a valid
unacknowledged CPd cannot arm it. The alternative permitted by the spec
therefore cannot rescue this exchange.

Concrete trigger: receive a valid CPd with ack=0; send an acknowledged
CPu/SUVu; lose the peer's later acknowledged CPd/SUVd; receive its Ed/B1d.
Our endpoint continues control transmission rather than completing the
current message and sending E2u.

Suggested correction: separate downstream Ed acquisition from receipt of
a control acknowledgement, retain the complete-message/frame-boundary
guard against interpreting control-body zeroes as Ed, and latch Ed as the
alternative completion condition once CPd parameters are available.

Needed regression: a foreign-peer script that intentionally omits the
final acknowledged control sequence and sends Ed after accepting our
acknowledgement. Also check that zeros within CPd/SUVd do not trigger Ed.

## Finding 3 — P2: reserved bits incorrectly invalidate control messages

Follow-up audit: `v92_phase4_decode.c:221,558` and
`v92_cp_rx.c:307,385,465` make `reserved_ok` a prerequisite for decoding
CPd (base and full), CPt/CPu, CPus and SUVu.

V.92 Tables 23, 24, 27 and 30 explicitly distinguish the transmitter's
requirement to send zero from the receiver's requirement not to interpret
those reserved bits. Our original specification confirms this for CPt/CPu
bits 26:30, CPus bits 26:32, SUVu bits 19:25 and CPd bits 30:32, as well as
the reserved fields in CPd's optional parts. A diagnostic deviation should
not make an otherwise CRC-valid message invalid. The SUVd decoder already
implements the correct distinction.

A temporary executable linked against our current decoder objects encoded
baseline CPd, SUVu and CPus messages, set one reserved bit, and recomputed
the information-only CRC independently using polynomial 0x8408. Results:

| Message | Baseline accepted | Modified CRC valid | Modified accepted |
|---|---|---|---|
| CPd, bit 30 | yes | yes | no |
| SUVu, bit 19 | yes | yes | no |
| CPus, bit 26 | yes | yes | no |

The CPd probe also confirmed `parameters_ok = true`. CPt/CPu and base-CPd
share the source-level rejection pattern but were not separately exercised
by this probe. A peer setting one of these bits causes a valid control
exchange to be discarded, potentially provoking retries or a startup
timeout. No real peer exhibiting this deviation was measured.

Suggested correction: retain `reserved_ok` for diagnostics, remove it from
acceptance conditions only for fields explicitly designated as ignored by
the receiving modem. Keep rejecting undefined sequence identifiers and
unsupported parameter selections. Add CRC-valid reserved-bit variants for
each message type, including CPd optional-section reserved fields.

## Checks that did not produce a new finding

- SUVd uses 17 sync ones, zero start/fill bits, bit 33 acknowledgement,
  mapping-frame padding and the information-only CRC. Reserved bits are
  reported diagnostically without rejecting a CRC-valid SUVd, consistent
  with the existing Amendment 1 handling.
- The analogue transmitter sends 12000T of initial TRN2u unless SUVd
  arrives first, extends E2u by the negotiated bit, resets upstream state
  before B1u, and sends 48 twelve-symbol B1u frames.
- The enclosing analogue controller already checks the B1d deadline
  against its INFO1a-end origin (`v92_analogue_phase3.c:255-258`);
  absence of that deadline in the leaf Phase 4 object is not a finding.
- Existing HDX documentation already records the mandatory recovery and
  INFOh work, and explicitly identifies remaining optional/untested paths.
  Those limitations were not recast as newly discovered bugs.

## Validation and limitations

`make v92_startup_test` and `./v92_startup_test` completed successfully.
The findings above are source-confirmed control-flow discrepancies;
the proposed fault-injection regressions have not been implemented or run.
Passing the startup suite does not disprove either finding and establishes
no hardware interoperability claim. No protocol implementation was changed.
Existing user changes in `fax_class2.c` and `fax_class2_test.c` were preserved.

## V.90 follow-up: control reception and renegotiation

Read the existing `docs/v90_phase3_4_spec_audit.md` and
`docs/v90_analogue_role.md` before inspecting the implementation. Checked
original V.90 (09/98) Table 14 and clauses 9.6.1.2.3-.6 and 9.7, including
rendered PDF pages 31, 46 and 47. The following are additional findings;
previously recorded DIL/S-transition issues are not counted again.

### Finding 4 — P2: the digital modem rejects peer cleardown CP

`v90_cp_rx.c:24` requires drn >= 1. V.90 Table 14 and 9.7 explicitly use
drn = 0 in CP to request cleardown, including during rate renegotiation.
A CRC-valid CP with that value never reaches the engine's frame handler.
The engine therefore cannot recognize the peer's protocol disconnect
through this receiver, and can instead wait for renegotiation recovery.

Allowing drn=0 through the parser alone is insufficient:
`v90_set_phase4_cp()` routes data-mode CP through ordinary mapper setup
or parameter equality, and `v90_configure_data_mapper()` requires drn>=1
(`v90.c:1960`). There is no corresponding V.90 CP-cleardown branch in
that acceptance path. A fix needs an explicit cleardown event before
normal constellation negotiation; 9.7 says to ignore the constellation
fields for a drn=0 CP. The existing analogue MP-cleardown handling and
V.92 CPu-cleardown handling do not cover the V.90 digital endpoint.

### Finding 5 — P2: CPs echo-reconditioning requests are rejected

`v90_cp_rx.c:27` requires bit 30 to be zero. Table 14 uses bit 30 for a
silence request during renegotiation. Clauses 9.6.1.2.3-.6 require the
digital modem to acknowledge CPs, finish MP', transmit Ed then Ucode-0
silence while preserving the mapping-frame grid, and later transmit
Rt/Rt-bar after receiving a CP with bit 30 clear.

A valid CPs never reaches the engine handler. The digital V.90 state
machine also has no `silence_request` handling for this procedure, so
removing the parser check alone would still produce the ordinary Ed/B1d
sequence. This is a missing procedure, not just an overstrict parser.
Our analogue endpoint already offers CPs through
`ME_V90_ANALOGUE_RATE_RENEGOTIATE_SILENCE`; it cannot interoperate with
this digital receive path for that exchange.

### Finding 6 — P2: V.90 reserved fields are also interpreted as validity conditions

`vpcm_cp.c:468` rejects nonzero Table 14 reserved bits 25:29 and 129:135.
`v90_cp_rx.c:26` independently rejects reserved bit 18. Table 14 says
these fields are not interpreted by the digital modem.

The analogue MP receiver repeats this pattern in
`v90_analogue_phase4.c:272-305`, checking Table 16 reserved bits and
reserved words before accepting the CRC. That is source inspection;
the executable probe below exercised the digital CP path only.

Retain deviations as diagnostics, but distinguish ignored reserved fields
from invalid identifiers, start bits, CRC failures and unsupported parameters.

### Executable V.90 probe

A temporary executable linked `v90_cp_rx.o`, `vpcm_cp.o` and the existing
SpanDSP library. It supplied one-set, 4-point-modulated control messages
directly to the strict bitstream receiver. Every listed message had a
valid CRC; reserved-bit variants had independently recomputed CRCs.

| Input | Shared CP decoder accepts | Strict receiver invokes handler |
|---|---|---|
| Normal CP | yes | yes |
| CP, reserved bit 25 set | no | no |
| CP, reserved bit 18 set | yes | no |
| Cleardown CP, drn=0 | yes | no |
| CPs, bit 30 set | yes | no |

These results reproduce the parser defects independently of DSP and
network conditions. Full renegotiation/hangup integration and live
hardware were not tested. Needed regressions: scripted peer cleardown
while negotiating, CPs/CPs' -> Ed/silence -> CP -> Rt/Rt-bar with frame
alignment assertions, and reserved-bit variants for CP and MP.
No protocol implementation was changed during this audit.

## V.90 follow-up: mapper limits and upstream offers

### Finding 7 — P2: legal shaped CPt profiles below drn=4 are rejected

`v90.c:1885` imposes drn>=4 before deriving the actual training K and S.
That lower bound is correct only when Sr=0. For CPt, D=drn+8,
S=6-Sr and K=D-S=drn+2+Sr. V.90 8.6.5/Table 17 permits K=6..24
with S=3..6, including these smallest profiles:

| drn | Sr | S | K | Mapper accepts |
|---|---|---|---|---|
| 4 | 0 | 6 | 6 | yes |
| 3 | 1 | 5 | 6 | no |
| 2 | 2 | 4 | 6 | no |
| 1 | 3 | 3 | 6 | no |

A temporary executable called `v90_set_phase4_cp()` on fresh V.90 pumps,
with a valid one-set constellation, sufficient modulus product, common
upstream capability mask and lookahead 1 for shaped profiles. All four
passed `vpcm_cp_validate()`; only the unshaped profile was accepted. The
lower-bound guard rejects the others before any shaper work occurs.

An analogue peer choosing a robust low-rate training profile can therefore
have its CRC-valid CPt accepted by the bitstream parser but refused by the
mapper, leaving Ri running. This concerns training rates, not a proposed
extension of downstream data-mode rates below 28000 bit/s.

Suggested correction: validate drn's field range, then validate the derived
K/S pair against Table 17. Preserve drn=0's separate cleardown treatment.
Needed regression: all four rows above through actual mapped TRN2d/MP
reception, both laws, with additional boundary rows at K=24.

### Finding 8 — P2: an empty upstream rate intersection restores disallowed rates

`v90.c:1438-1443`, `v90_capped_upstream_mask()`, returns the peer's original
mask when intersection with the local upstream ceiling/floor is empty.
The fallback is explicit in the existing code and log, but defeats the
API's configured limits and the purpose stated in its header: MP must offer
only rates the receiver can accept.

An executable probe set `v90_set_upstream_rate_limit(s,24000)` and supplied
a CPt offering only 33600 upstream. Mapper setup succeeded and the MP
copied with `v90_copy_phase4_mp_bits()` advertised maximum drn=14, i.e.
33600 bit/s. The console explicitly logged that it was echoing the offer
uncapped. Thus a missing common rate becomes an apparently successful
negotiation outside the configured ceiling.

The same branch also restores rates below a configured minimum; the engine
passes `g_lim[LIM_MIN_RX]` to `v90_set_upstream_rate_floor()` at
`modem_engine.c:13226`. That minimum case was source-inspected, not separately
executed. The engine additionally caps its V.34 Phase 2 ceiling, but that
does not make advertising an unsupported rate in MP correct.

Suggested correction: keep an empty intersection empty and propagate a
negotiation failure or the applicable recovery/cleardown action. Do not
substitute a rate unsupported by the peer, either. Needed regressions:
empty intersections caused separately by ceiling and floor, plus a nonempty
intersection that verifies every MP mask bit and its selected maximum.

Validation: the mapper probe linked the existing V.92 startup dependency
objects (excluding its main test object) and SpanDSP. Source clauses were
checked against the rendered V.90 Table 17 (PDF page 36). No implementation
changes or hardware calls were made.

## V.90 follow-up: decoder bounds and MP semantics

### Finding 9 — P2: truncated CP buffers cause out-of-bounds reads

`vpcm_cp.c:364` permits every positive `nbits`, but
`vpcm_cp_decode_diag()` reads header positions through bit 135 before
checking the computed frame length. It reads the caller's original buffer,
not the zero-initialized diagnostic copy. For example, with a one-byte
allocation and nbits=1, the sync check can fail at bit 0 but execution still
reads bit 17 in the start-bit checks (`vpcm_cp.c:380`).

Executable reproduction: placed a single zero byte at the end of an mmap
page, protected the next page with PROT_NONE, and called the current decoder
with nbits=1. A SIGSEGV/SIGBUS handler printed
`OUT-OF-BOUNDS READ TRAPPED` and exited with status 77. Thus this is a
reproduced memory access defect, not merely a permissive acceptance check.
An AddressSanitizer attempt was abandoned before obtaining diagnostics;
the successful reproduction used ordinary existing decoder objects and a
guard page instead.

The streaming receiver normally calls the helper only after collecting its
target frame size, so this does not establish remote exploitability through
SIP. Direct callers, truncated captures and future decode integrations are
at risk. `vpcm_cp_decode_bits()` delegates to this helper and inherits the
same problem.

Suggested correction: reject inputs shorter than the fixed header/minimum
frame before any indexed reads, then check the derived complete length
before reading variable sections. Needed regression: every truncated
length up to the shortest CP frame, with a buffer of exactly that size.

### Finding 10 — P2: CRC-valid MP frames are accepted without parameter validation

`v90_analogue_phase4.c:266-306` checks MP framing, reserved-zero fields
and CRC, but not the selected maximum-rate or trellis values. Its caller at
lines 382-397 marks the frame valid and emits MP/MP_PRIME events.

Table 16 permits maximum drn=2..14, with drn=0 reserved for cleardown;
drn=1 and drn=15 are undefined. Trellis selection 3 is reserved, while
0,1,2 select the 16/32/64-state encoders. These are parameter selections,
distinct from the reserved fields receivers are instructed to ignore.

A temporary executable included the current Phase 4 receiver source so it
could call the exact static `mp_structure_ok()` used by the live decoder.
It independently constructed Type-0 CRCs and obtained:

| MP parameters | Validator accepts |
|---|---|
| drn=2, trellis=0 | yes |
| drn=1, trellis=0 | yes |
| drn=2, trellis=3 | yes |

`apply_phase4_events()` in `v90_analogue_phase3.c:218-222` immediately
forwards these MP events into the CP acknowledgement/E state machine.
Only afterward does upstream rate selection or data-transmitter setup
encounter an unusable profile. `v34_v90_arm_tx_data()` itself stores the
requested trellis without validating it; `get_external_baud()` later clears
the armed flag before attempting `v34_v90_begin_tx_data()`. A failed begin
falls back to the external symbol source rather than propagating a new
controller failure. Consequently acknowledging the malformed MP can move
the handshake toward E without a usable upstream data configuration.

Suggested correction: validate the selected MP parameters before publishing
MP-valid events; require a usable common rate before acknowledging a
non-cleardown offer, and validate the deferred transmitter configuration
before reporting that it has been armed. Keep valid drn=0 cleardown separate.
Needed regressions: CRC-valid undefined drn and trellis selections, empty
rate intersections, and deferred handoff failures. Full wire-level malformed
MP injection was not run; the executable tested the actual acceptance gate.

These follow-up probes changed no protocol code and made no hardware calls.

## Resolution (2026-10-07, same day)

Findings 1-10 are addressed except as noted. `make v92_startup_test` and
`v90_analogue_rx_test` pass (linked against SpanDSP only; the pjproject-linked
suites, `vpcm_loopback_test` among them, were NOT run in this environment, and
`modem_engine.c`'s one hunk was not compiled here). No hardware interop claim.

- **1** digital (`v90.c`, sticky `v92_cpd_repeat`) and analogue
  (`v92_analogue_phase4.c`, `cpu_repeat`): after the 9.6.x.1.3 timeout the
  transmitter repeats CP at each message boundary until the exchange is
  acknowledged both ways. The 9.11 cleardown paths deliberately do not set it.
  No fault-injection regression yet.
- **2** analogue: the Ed detector is armed on any complete CPd once its
  parameters are known, and a received Ed completes the exchange like an
  acknowledged CPd/SUVd. **Limit:** an Ed after a DAMAGED final control message
  is still missed, because the damaged message's non-zero frames disarm the
  detector (8.8.2 guard); covering it needs message-length-based re-arming.
- **3, 6** reserved bits no longer gate validity in `v92_cp_rx.c`,
  `v92_phase4_decode.c`, `vpcm_cp.c`, `v90_cp_rx.c` (incl. bit 18) or the
  analogue MP receiver; they stay diagnostics. Undefined MP drn (1, 15) and
  trellis 3 are now rejected (finding 10). Tests: `v92_startup_test`.
- **4** a data-mode CP with drn = 0 reaches the engine, which hangs up
  (`Remote (V.90 9.7 cleardown)`) before any mapper setup.
- **5 NOT fixed:** CPs (bit 30) still stops at the strict parser. Passing it on
  without the 9.6.1.2.3-.6 silence procedure would turn it into an ordinary
  Ed/B1d, which is worse than dropping it.
- **7** `drn >= 4` is now `drn >= 1`; the K 6..24 check already follows.
- **8** an empty upstream-rate intersection stays empty and fails MP build.
- **9** `vpcm_cp_decode_diag()` rejects `nbits < 136` before reading the header.
