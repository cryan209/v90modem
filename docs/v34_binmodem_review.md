# V.34 comparison with BinModem

Reviewed 2026-10-07. Reference checkout:
`/Users/scottcryan/Documents/Codex/2026-10-07/https-github-com-casualarclamp-binmodem-https/work/BinModem`.

This is a focused source comparison of duplex startup, MP handling, recovery,
and receiver acquisition. It is not a complete V.34 compliance audit or a
cross-implementation waveform test. Read alongside `v34_spec_gap.md`,
`v34_plain_phase2_call_role.md` and `v34_hdx_conformance_audit.md`.

## Findings in our implementation

### P2: legal cleardown MPs cannot pass the semantic gate (known gap, confirmed)

`spandsp-master/src/v34rx.c:3608` rejects either directional rate below 1.
V.34 11.7.1.1 explicitly requests zero in both directions for cleardown;
11.7.2.3 requires an acknowledged exchange before disconnecting. Consequently
a CRC-valid peer cleardown MP cannot become an accepted MP at this gate.
The existing gap audit already records missing duplex cleardown; this
comparison identifies its concrete receive-side blocker rather than a new gap.

BinModem `training.rs:1411` recognizes both zero fields during renegotiation
and `training.rs:1696` has a termination decision. Ours needs a separate
cleardown path before normal rate negotiation, plus MP/MP-prime completion and
an engine carrier-loss event. Merely accepting zero in normal rate selection
is insufficient. Test initiation from either role with a scripted foreign
peer, including loss of the first acknowledgement.

### P2: a Type-0 preamble is checked against the previous frame's bit 19

`spandsp-master/src/v34rx.c:10478-10484` calls `mp_seed_frame_prefix()` and
then tests `mp_frame_bits[19]` before collecting the new frame. That helper
(`v34rx.c:2155`) writes only positions 0 through 18. Position 19 therefore
belongs to an earlier frame, not the new preamble. If a previous collected
frame had bit 19 set, this path skips subsequent Type-0 preambles even when
their own bit 19 is zero. The initial hypothesis-lock path does not have
this check, so this finding is specifically about repeated-frame collection;
it does not establish that every receiver path will stall permanently.

Moreover, V.34 Table 20 says bit 19 is not interpreted by the receiver.
Our later semantic gate already tolerates it (`v34rx.c:3601`), while this
earlier check contradicts that policy. BinModem `mp.rs` checks frame start
bits and CRC without using bit 19 as a validity condition. Remove the stale
pre-collection check and preserve observed reserved bits for the CRC.
Regression: collect a frame with bit 19 set, then a valid ordinary Type-0
MP/MP-prime; verify the second frame is collected regardless of old storage.
This is source-confirmed; waveform fault injection was not run.

### P2: the second caller Tone B reversal remains timer-driven by default (known)

`spandsp-master/src/v34tx.c:3684` defaults the reversal-wait option off;
the condition at `v34tx.c:4075` then reverses after 100 bauds without requiring
the peer's subsequent Tone A reversal. This violates the normal ordering in
11.2.1.1.6 and has existing RasFinder evidence in the source and investigation
notes. The conformant option exposes a receiver acquisition failure, so
switching the default alone is not a validated correction.

BinModem's `phase2.rs` explicitly models tone/reversal stages. The practical
order for ours is to reproduce the timing-sensitive 21600-bit/s acquisition,
fix it, then enable the peer-driven reversal and exercise the clause's timeout
recovery separately. This review did not repeat the historical live A/B.

## Receiver mechanisms worth evaluating

BinModem `receiver.rs:883` solves a T/2 equalizer against known PP/TRN symbols
at several candidate alignments, estimates residual carrier rotation, and
refits with that rotation. Its receiver also retains sample history and loop
state for rewind/resynchronization after a discontinuity. These are concrete
comparison experiments, not proof its receiver is better on our bearer.

Our `v34_pp_fit.c` already provides a known-PP fitting experiment, but searches
of the production SpanDSP V.34 sources found no calls into that helper.
Compare fits on preserved failing and passing captures using the actual TX
symbols and held-out residuals before integrating an acquisition replacement.
Do not grade only distance to the receiver's own chosen point: the repository
already documents why that can favor a wrong constellation decision.
Keep V.90 and HDX scope explicit, preserve DS0 byte identity and sample counts,
and avoid treating a PCM downstream stream as ordinary analogue QAM.

## BinModem is not a normative oracle

Its `mp.rs:trellis_of()` maps reserved trellis code 3 to 16 states; our
semantic gate rejects it. That difference is not evidence to relax ours.
Also, BinModem `training.rs:1696` lets a responding cleardown complete after
sending an acknowledged frame and receiving any peer MP, whereas 11.7.2.3
requires receiving the initiator's MP-prime. Inspect and test that condition
before borrowing its cleardown implementation. These are reference-side
limitations, not findings against this repository.

## Validation

`make v34_mp_test v34_duplex_test` succeeded with SpanDSP reporting no pending
build work. `v34_mp_test` passed. Duplex 2400 baud/9600 bit/s passed both
PCMU and PCMA with both endpoints trained and zero payload errors in both
directions (more than 16000 received bits per direction). These tests do not
exercise cleardown, the stale reserved bit, or the timing-sensitive 21600 case.

No protocol source was changed, no live call was placed, and no BinModem
interoperability claim is made. Existing fax edits and the separate untracked
BinModem protocol audit were preserved.

## Follow-up: completion and deadlines

### P2: E cannot replace a lost peer MP-prime

V.34 11.4.1.1.3 and 11.4.1.2.3 allow completion after sending MP-prime
and receiving either peer MP-prime or E. BinModem `training.rs:1655`
expresses this as `far_acknowledged || far_e`.

Ours instead requires `mp_remote_ack_seen` in both the transmitter's
completion condition (`v34tx.c:8462`) and the E detector
(`v34rx.c:10424`). `mp_prime_may_end()` contains an E alternative, but
its caller's outer acknowledgement condition defeats that alternative.

Trigger: we receive ordinary MP and send MP-prime; the peer receives our
MP-prime and sends E, while its own preceding MP-prime was damaged. We
continue MP-prime and cannot use the valid E to complete. Preserve guards
against MP-body ones mimicking E, but use complete-message boundaries and
local MP-prime transmission state rather than making remote acknowledgement
mandatory. Needed regression: lose peer MP-prime, retain its E/B1, and
assert completion from both roles; also inject long runs of ones in MP bodies.
This is a source finding, not a waveform reproduction.

### P2: startup E recovery is replaced by an MP-only watchdog

V.34 11.4.2.1.2 requires the caller to retrain if E has not arrived after
J-prime within 2500 ms + 2 RTD (30 s with the peer's CME bit). Clause
11.4.2.2.2 gives the answerer 2500 ms + 3 RTD from its S-bar, or 30 s
with CME. BinModem's training state machine sets role/CME-aware deadlines.

Our `PHASE4_MP_TIMEOUT_BAUDS` is 20000 (`v34rx.c:543`), and the check
at `v34rx.c:10272` runs only while `mp_seen == 0`. At 2400 baud that
threshold is 8.33 seconds even within the MP stage, not the comment's
approximately two seconds. Once any MP is accepted, this watchdog stops
checking even if MP-prime/E never arrives. The engine's general training
timeout is 60 seconds (`modem_engine.c:3012,10764`); that is not the
clause-specific retrain procedure. The separate data-mode renegotiation
timer does not cover initial startup.

Needed regression: accept one MP, suppress subsequent MP-prime/E, and
assert retrain against the role-specific transmitted-signal origin for
several RTDs and both CME values. Also test complete absence of MP.
These branches were source-inspected; no timeout injection was run.

### P2: the answerer's Phase 4 TRN upper bound does not bound transmission

`phase4_trn_max_bauds()` computes a 2000-ms guard (`v34tx.c:7577`).
At `v34tx.c:7820` exceeding it deliberately continues TRN until the
receiver publishes PHASE4_TRN_READY; the optional independent transmit cap
defaults to zero. Thus a missing readiness event can extend TRN indefinitely
within the outer training timeout. V.34 11.4.1.2.2 requires at least 512T
but no longer than 2000 ms + RTD. A guard that only logs cannot enforce it.

This is a separate transmit-sequence defect from the missing E deadline:
one can occur before MP starts, while the other must remain effective
through MP/E. Test a delayed or absent receiver-ready event and verify
the answerer leaves TRN within the allowed bound, with an explicit recovery
decision if the receive side remains unusable. Do not turn this into an
unqualified forced-success handoff. No fault-injection run was made.

### Additional timing discrepancy: renegotiation uses a fixed wall-clock timeout

`modem_engine.c:5012,5034,5044,5282` implements a default 4000-ms timeout
from the API request using `trace_now_ms()`. Clause 11.6.2.1 specifies
2500 ms + 2 RTD, or 30 s for CME, measured after the transmitted S-to-S-bar
transition. Local INFO0 not advertising CME does not establish the peer's
CME value. A future correction should use transmitted sample positions,
the applicable INFO0 CME value and measured RTD; treat the environment
override as an explicit diagnostic deviation. This was not delay/CME tested.

## Follow-up: constellation and INFO1 parameter authority

### P2: duplex J's 16-point request does not configure TRN/MP/E transmission

V.34 10.1.3.8 and 10.1.3.9 select four- or sixteen-point TRN/MP from J;
10.1.3.2 makes E use that selection too. BinModem `training.rs` transfers
the received J size to its transmitter, and its MP/E source uses that size.

Our J detector records `rx.phase3_j_trn16` (`v34rx_phase3.c:600`).
The duplex transmitter tests/logs that field but does not copy its selection
into the constellation state. Phase-3 and Phase-4 TRN instead use
`tx.infoh.trn16` (`v34tx.c:7172,7736`), the half-duplex INFOh field.
`get_mp_or_mph_baud()` always consumes two bits and returns a four-point
symbol (`v34tx.c:8438` and the end of that function). `get_e_baud()`
likewise always emits two bits per symbol and ends after ten symbols
(`v34tx.c:8740`). A 16-point E needs five symbols for its twenty bits.

A temporary executable included the production `v34tx.c` to invoke the
actual static generators, linking the remaining SpanDSP library. With
`rx.phase3_j_trn16=1` and even `tx.infoh.trn16=1`, one MP symbol advanced
the real transmit bit pointer by 2 rather than 4. E's source remains the
two-bit, ten-symbol implementation. Changing J alone did not change the
first Phase-4 TRN point; the stronger evidence for the missing authority is
the source's exclusive selection through INFOh, not that single-point check.

This breaks a foreign duplex modem asking us for sixteen-point training;
forcing INFOh's flag alone would still leave MP/E wrong. Keep the two
directions' J selections separate. Preserve 11.6's explicit four-point
renegotiation requirement rather than carrying an old sixteen-point startup
selection into rate renegotiation. Needed regression: independently request
each constellation from each end, then grade TRN, MP/MP-prime and E through
the transmitted waveform and verify B1/payload alignment in both laws.
The probe was generator-level, not full interoperation.

### P2: undefined INFO1a parameters are published as a completed negotiation

The plain-V.34 branch of `process_rx_info1a()` (`v34rx.c:4795-4863`)
parses both three-bit symbol rates, pre-emphasis and projected data rate
without validating their selections. It unconditionally marks INFO1a
received and returns success. The outer receiver publishes INFO1_OK
(`v34rx.c:6058-6075`). An invalid baud code merely bypasses the retuning
condition, leaving the old configured rate in use; an invalid pre-emphasis
index later bypasses filter installation (`v34tx.c:6880`), effectively
substituting a flat filter.

Executable probe: included the production `v34rx.c`, initialized a plain
calling V.34 instance at 2400/9600, and called its exact parser with an
all-one information payload. Result: `rc=0 received=1 a2c=7 c2a=7
preemp=15 rx_baud=0`. Thus both undefined symbol-rate codes and an undefined
filter selection were accepted while RX stayed on its 2400-baud index.
This directly exercises the parser after the outer framing/CRC stage; no
CRC-valid malformed waveform was transmitted.

V.34 Table 16 defines symbol-rate indices 0..5, filters 0..10 and projected
rates 0..14. BinModem `info.rs:366-367` requires `SymbolRate::from_index()`
to succeed before returning an INFO1a; its general pre-emphasis validation
was not established here and should not be assumed correct.

Validate the complete candidate before mutating live parameters or setting
the received flag. Also verify consistency with the local INFO1c offer and
INFO0 capability/asymmetry constraints, rather than accepting a defined but
unoffered rate. The latter consistency cases were not separately probed.
Preserve the distinct V.90 INFO1a layouts where code 6 has a defined role.
Needed regression: all undefined indices, a legal but unoffered selection,
and a valid selection following a rejected frame, with no state changes or
INFO1_OK event on rejection.

Both temporary probes compiled successfully with the checkout's configured
SpanDSP headers and static library. No production protocol implementation
was edited in this follow-up.

## Resolution (2026-10-07)

Fixed, each with a test: **the stale bit-19 pre-collection check** (removed;
Table 20 bit 19 is not interpreted and the helper never wrote that position),
and **undefined INFO1a selections** (symbol-rate codes above 5, pre-emphasis
above 10, projected rate above 14 now reject the whole frame, restore the
previous values, set no `info1a_received` and publish no INFO1_OK; plain V.34
only, the V.90 layouts are untouched). `v34_info1a_validate_test` is new and in
`make test`; `v34_mp_test`, `v34_data_test` and all 46 `v34_duplex_test` rows
in the makefile pass. The bit-19 change has no dedicated regression (no
waveform-level injection exists for it).

### Second pass (same day): the rest, except one

- **E in place of a lost peer MP'** (11.4.1.1.3/11.4.1.2.3). Two defects, not one.
  The receiver ran `mp_unlock_after_reject()` on every CRC failure and, after
  three, rotated the decode mode -- so after one damaged MP' it had thrown away
  a lock that had just decoded a good MP, and was no longer looking for E at
  all. After a frame has been accepted a later reject now keeps the lock. E is
  then accepted without the peer's MP' only if our own complete MP' has gone
  out and the 20 ones end on the MP frame grid (`mp_bits_since_frame`, a whole
  number of frames plus 20 after the last decoded one, at least one frame in
  between), which is what keeps an MP body's ones from reading as E. The
  transmitter completes on E the same way. Test: `V34_TEST_LOST_MP_PRIME=1`
  makes one modem damage every MP' it sends; without the change the call never
  trains (60 s), with it both ends train with zero errors (`E after 372 bits`).
  Two rows in `make test`.
- **E deadline** (11.4.1.1/11.4.2): once an MP has been accepted, no E within
  2500 ms + 3 RTD (30 s with the peer's CME bit, new `v34_get_far_cme()`) raises
  TRAINING_FAILED. Measured from the first accepted MP, which is later than the
  clause's origins, and RTD is a 250 ms allowance (`ME_V34_RTD_MS`). Not
  exercised by a test of its own; the 50 duplex rows show it does not fire on a
  healthy call.
- **Answerer TRN bound** (11.4.1.2.2): TRN now stops at 2000 ms + a 250 ms RTD
  allowance (`ME_V34_TRN_RTD_MS`, 0 disables) and MP starts anyway, logged.
  Same caveat: no test forces the bound.
- **11.6 timeout**: counted in received audio from the request, 2500 + 2 x 250 +
  150 ms (S and S-bar), 30 s on CME; `ME_V34_RENEG_TIMEOUT_MS` still replaces it
  as a labelled deviation. The old 4000 ms wall-clock default is gone.
- **Duplex J's 16-point request**: the Phase 4 TRN, MP, MP' and E of a duplex
  modem now use the constellation of the J it received (10.1.3.3); 16-point MP
  is four scrambled bits per symbol with Q selecting the point and I
  differentially encoded, E is five symbols, and 11.6 stays four-point.
  `v34_phase4_16pt_test` calls the real generators. **Generator level only: our
  receiver has no 16-point Phase 4 path, so there is no loopback for it** and it
  has met no foreign modem. We still always ask for four-point ourselves.
- **Cleardown** (11.7): `v34_start_cleardown()` is the 11.6 start with an MP
  requesting zero in both directions; an MP with both rates zero is accepted
  inside a renegotiation and not otherwise; the responder is the ordinary 11.6
  responder; both ends finish when MP' has been sent and received
  (`v34_cleardown_complete()`), after which the engine releases the call.
  `V34_DUPLEX_CLEARDOWN` rows (ulaw 2400, alaw 3200) pass. **Deviations:** we
  still send TRN between S-bar and MP (11.7.1.1 has none), and the engine only
  RESPONDS to a peer's cleardown -- nothing in the engine initiates one (ATH
  still drops the SIP call), and no foreign modem has been tried.

- **Second Tone B reversal** (11.2.1.1.6): now waits for the answer modem's
  reversal by default (`ME_V34_SECOND_B_WAIT_REVERSAL=0` restores the timer).
  The failures that kept it off were not 21600 acquisition: the answer modem
  took the call modem's Tone B resuming as the reversal and opened its L1/L2
  window up to 0.7 s early, so INFO1a was built from noise. It now ignores that
  step until its own post-L2 reversal has been sent. 49 of 50 duplex rows pass;
  3000/28800 at 20 dB echo is a coin flip either way and is pinned to
  `V34_DUPLEX_DELAY=3` in the makefile. Not live-verified.
