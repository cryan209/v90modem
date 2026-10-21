# Carrier loss in data mode (S10)

V.250 6.3.12: S10 is "the amount of time, in tenths of a second, that the DCE
will remain connected to the line (off-hook) after the DCE has indicated the
absence of received line signal. If the received line signal is once again
detected before the time specified in S10 expires, the DCE remains connected
to the line and the call continues." Defined values are 1-254. V.250 gives no
default and does not define 255. We follow Hayes practice: factory 14 (1.4 s,
the value in `at_interpreter.c`'s profile, which used to say 0), and 255
means never disconnect on carrier loss.

## Why

A real CX93001 dialling us at V.22bis 2400 under MNP (`docs/mnp.md`, "CX93001
dialling in") drops its signal to line noise for about 30 ms, 8-10 s into
every call, then comes back at full level. Two faults of ours turned that
into a hang-up:

1. `v22bis_put_bit_cb` hung up on the first `SIG_STATUS_CARRIER_DOWN` after
   training.
2. SpanDSP's `v22bis_rx()` answered any carrier drop after symbol
   acquisition with `v22bis_restart()`, which restarts **both** directions.
   Our transmitter went back to the start of the handshake, and the trained
   equaliser, carrier and timing loops were thrown away.

## V.22bis (fixed)

- `v22bis_rx.c`: in `V22BIS_RX_TRAINING_STAGE_NORMAL_OPERATION`, a drop below
  the carrier-off threshold sets `rx.carrier_held` and reports CARRIER_DOWN.
  While the carrier is held, each sample runs `hold_sample()`: the T/2
  sampling counter and the carrier DDS run on at their trained rates, so the
  receiver resumes on the same symbol grid, but nothing goes to the
  equaliser, Gardner or carrier loops and no bits are delivered. Each held
  T/2 instant still puts a zero into the equaliser delay line and flips
  `baud_phase`, as `process_half_baud()` would; see "Parity" below. The AGC is
  already locked after acquisition. The transmitter is not touched. No
  sample is added to or removed from the stream. When the power comes back
  above carrier-on, CARRIER_UP is reported and the same trained receiver
  carries on. The V.22bis descrambler (1 + x^-14 + x^-17) is
  self-synchronising, so a few bits are lost after the drop and error
  control resends the frame they cut. Drops during training still restart
  the datapump, as before.
- `modem_engine.c`: CARRIER_DOWN after training starts an S10 countdown in
  received samples (`v22bis_carrier_hold_tick`, read through
  `di_carrier_loss_delay()`). CARRIER_UP before it runs out logs
  "carrier back after N ms" and the call continues. If it runs out, the
  engine logs "(S10), hanging up" and hangs up. If a second
  TRAINING_SUCCEEDED arrives in data, the engine logs "retrained in data":
  that would mean the datapump restarted instead of holding.
- Circuit 109 (DCD) to the DTE is not dropped during the hold.

Tests (`tests/fast.list`, `engine_pair_test --gap-ms`, which silences the
caller's signal into the answerer for N ms mid-payload, sample-accurate, one
sample in for one sample out):

| gap | grade |
|---|---|
| 40 ms, MNP | held without a retrain, payload intact both ways, caller resent LTs (5) recovered by MNP |
| 60.75 ms, MNP | the same; this length holds an odd number of T/2 instants |
| 2 s, MNP | first NO CARRIER 1.42 s after the gap began (S10 = 14) |

Carrier detect needs about 30 ms of silence to drop. The power meter is
exponential, and falling to the carrier-off level takes about 8 of its 4 ms
time constants. So a 20-30 ms gap never trips the hold: the receiver runs
through it on zeros, which leaves the equaliser taps unchanged because the
input is zero, and error control resends what was cut. Live 20 ms RTP
losses behave the same way.

### Parity (found live, 2026-10-21)

`process_half_baud()` takes T/2 samples and makes a decision only on
alternate ones (`baud_phase`). The first version of `hold_sample()` kept
the T/2 counter running but skipped those insertions. After a hold of an
odd number of T/2 instants, the receiver decided at the eye crossing for
the rest of the call: decision error 0.04 -> 0.77, constellation magnitude
3.0 -> 1.8, and the decision-directed loops wandered off. The first
loopback rows passed by luck, because their frame-granular gaps always held
an even number. A sample-by-sample sweep from 60.000 to 60.875 ms fails at
60.625-60.875 with the old hold and passes everywhere with the fixed one.

Checked by hand as well: `ATS10=5` with a 2 s gap hangs up after 0.52 s.
`ATS10=255` rides out a 3 s gap and the payload still arrives intact. LAPM
rides out a 40 ms gap too.

Negative control: with the datapump hold disabled and the engine still
holding, the 40 ms loopback **still delivers the payload**. SpanDSP's full
restart retrains against our own engine in about 1.6 s. Only the
"retrained in data" log tells the two apart, so the test checks for it. The
live CX does not survive that restart.

## Live, CX93001 (+ES=4,4, AT+MS=V22B) into the CT 117 server

- Call 1: V.8 failed twice (the CX's CM unreadable, `modulations=none`).
  We trained at 1200 without V.8 and the CX never reported CONNECT. Not a
  data-mode result.
- Call 2 (`CONNECT 2400`, CX `&V1`: PROTOCOL MNP): the CX's own analogue
  drop (~30 ms, RMS 5) at 17.98 s was ridden out; the old code hung up
  there. At 21.26 s, 3 lost RTP packets (seq 24329-24331) reached us as 60 ms
  of exact digital zero. The hold engaged, and the parity bug above left our
  receiver white for the rest of the call: 6 of 20 lines arrived, and 928
  octets were unsent when the CX's scheduled hang-up came. Replaying the tap
  through `v90_engine_replay --sync --echo-ref` reproduced the call to the
  millisecond. With the parity fix the replay's receiver stays at decision
  error 0.04 for the whole call. The CX's symbol timing drifts smoothly
  (about 0.9 samples/s) with no jump at the gap, so the signal itself was
  always decodable.
- Calls 3-4, with the parity fix deployed: both failed like call 1. V.8
  failed, we trained at 1200, and the CX never reported CONNECT. Replaying
  call 4 with the original `v22bis_rx.c` trains at 1200 at the same
  millisecond, so this handshake fault predates the hold. On those links
  the hold rode out the drops (10-60 ms) that the CX made while it waited.
  The parity fix is still unverified live in an MNP data call: 1 of the 4
  calls reached CONNECT 2400.

## Why calls 1, 3 and 4 trained at 1200 (fixed 2026-10-21)

The fault was ours: our answerer missed the CX's S1. The CX sent the same
handshake on all four calls. It detected our USB1 at 12.45 s, then 611 ms
later (6.3.1.1.1 b)) sent S1, the unscrambled double dibit 00/11 (+90 and
+270 degrees, Table 1/V.22bis), for 100 ms at 13.14-13.25 s. Then it sent
scrambled ones, and went to 16-point 600 ms after that (6.3.1.1.1 d)).
Demodulating its tap with phase differences shows S1 cleanly on every call.
Our transmit tap is right for what we decided: call 2 sends S1 back at
13.25 s, while calls 1, 3 and 4 hold USB1 to the 270 ms commit (6.3.1.2.2 b))
and then send scrambled ones at 1200. The CX does not fall back to 1200 as
6.3.1.1.1 c) says it should. It stays at 2400, so the two ends never meet.

SpanDSP's answerer spends the first 40 symbols of S1 on symbol acquisition,
which leaves about 25 to find S1 in before the equalizer has trained. It
read S1 by slicing each point to a quadrant and differencing the quadrants.
On this line S1's two points arrive only about 70 degrees apart (dispersion
the untrained equalizer has not removed). The 1200 bit/s slicer region for
point 00 is -26.6..63.4 degrees, around (3,1) at 18.4 degrees. On call 2 the
points sat at 67 and -3 degrees, either side of the boundary, and decoded as
00 11. On calls 1, 3 and 4 they sat at 53..63 and -9..-19 degrees, both in
one region, and decoded as 01 01 01. The decision-directed carrier loop and
equalizer then pulled them together (72 down to 22 degrees apart in 20
symbols), because both had the same target. Timing was identical on all
four calls (total correction -64/80 of a half-baud). Only the carrier phase
against the slicer differed, so whether a call connected was a coin flip
on that phase.

Fix (`v22bis_rx.c`, `SCRAMBLED_ONES_AT_1200`): the S1 search takes each
dibit from the angle between consecutive equalizer outputs, rounded to the
nearest Table 1 phase change, so it no longer depends on where the points
fall against the slicer. While an S1 run is in progress, the carrier loop
and equalizer do not adapt, so the two points stay apart. Scrambled ones
resume training when S1 ends. Each half alone fails every case below, so
both are needed. `v22bis_s1_test` (fast tier) checks:

- The four calls' receive taps (12.9-14.9 s, in
  `reference-data/artifacts/cx93001-v22bis-s1`) into a fresh answerer: old
  code 1200/2400/1200/1200, exactly as live; fixed, 2400 on all four.
- A SpanDSP caller into the answerer over `{1, 0.45, -0.2, 0.1}` at every
  delay from 0 to 14 samples. Each sample turns the carrier 54 degrees
  against the symbol timing. Old code: both ends at 1200 at every delay,
  and one echo tap of 0.2 is enough to cause it. A clean channel passes
  either way, which is why no loopback test caught this. Fixed: 2400 at
  every delay.

`v90_engine_replay --sync --echo-ref` of calls 1, 3 and 4 now trains at
2400 at 14060 ms, after 32 descrambled ones at 2400 from the CX
(6.3.1.1.2 e)). That shows the CX really was at 2400. A replay cannot show
what the CX would do after our S1, because that is our first divergent
transmission.

**Live re-check, 2026-10-21 (later): 4 of 4.** The CX (`AT\N2`,
`AT+MS=V22B`, `tools/cx_hold_dial.py ... 2904 35`) dialled the CT 117
answerer (`tools/soak/answer_ec_call.sh`, 5b12ad73's `v22bis_rx.c`) four
times. Every call trained at 2400 at 13760-13801 ms, just after the CX's
S1, where the old code had committed to 1200. The CX reported `CONNECT
2400`, `PROTOCOL LAPM`, `COMPRESSION V42Bis`. 20 of 20 lines arrived each
way on every call. Three calls ended with a LAPM disconnect at the CX's
scheduled `+++ATH`. On the fourth, the SIP hang-up arrived before the CX's
DISC frame, after all the data had arrived. Before the fix, the same setup
connected on 1 call of 4. Evidence: `/root/s1-live1`..`4` on CT 117.

V.8 was not the problem. The CX never sent a CM: in both V.8 windows its
V.21 low channel stays at the line-noise floor (about 63 dB, against 131 dB
once it transmits), and the RMS 64 the log reports is our own ANSam echo at
2100 Hz. With `AT+MS=V22B` it is a plain V.25 caller that stays silent
through the answer tone. "modulations=none" is the summary of a CM that
never arrived, not an unreadable one. The two V.8 attempts cost 11.9 s
before USB1, and the CX waits that long.

### Live re-check, 2026-10-22: another acquisition eye

Built isolated main `5decc31b` on wired vmhost CT 117, using its vendored
SpanDSP and pjproject, after `make clean`. The Mac's unrelated V.34
working-tree changes were not deployed. G.711 remained PCMU (RTP payload
0), passthrough codecs enabled, RX clock-recovery slips disabled; no DSP
constants, resampling, gain or sample accounting were changed. The CX
identified as `CX93001-EIS_V0.2013-V92` on
`/dev/cu.usbmodem246802461`, calling 2904 through ATA L2/6005. Both ends
used the same procedure as the four earlier successful calls:
`answer_ec_call.sh ... v22 "ATE0;AT&A3;AT+ER=1;ATS0=1"`, and
`cx_hold_dial.py ... 2904 35 'AT\N2' 'AT+MS=V22B'`.

Main did not connect reliably: the first call trained our TX at 1200 and
the CX never reported CONNECT; the second connected at 2400/LAPM/V.42bis
and delivered all 20 exact numbered lines each way. Both had continuous
RTP sequence numbers and 160-sample timestamp steps. S1 was present on
the first call, as it was in the original failing recordings. Our own TX
held USB1 and committed to 1200, so this was our acquisition failure.

The first call adds a case absent from the four original fixtures.
Gardner's acquisition correction alternated -16/0, ending at 0 instead
of the original fixture's -64 (units are 1/12 sample). At the end of
acquisition, consecutive decision outputs differed by about 1-12 degrees,
versus 70-72 degrees on original fixture 1. The decision eye was near
S1's crossing; the other T/2 eye still resolved its alternating 00/11.
The phase-change detector correctly returned 01 at the decision eye,
so the previous slicer-phase fix could not find this S1.

The initial candidate searched the other eye independently and inhibited
carrier/equalizer adaptation while either eye saw an S1 run. One call
connected at 2400/LAPM/V.42bis, with 20/20 exact lines each way; a single
missing 20 ms RTP packet was held without a retrain. The next call sent
our S1 and trained TX at 2400, but never acquired receive-side scrambled
ones. V.42 detection expired, our DTE fell back to V.14, and the CX never
reported CONNECT. A TX TRAINING_SUCCEEDED event alone is not receive
training or a hardware connection.

The revised candidate also selects the T/2 eye that found S1 for the
remaining acquisition. It changes which existing half-baud samples are
used for decisions during training; it inserts/removes no input samples
and changes no timing-loop constants. New state resets in
`v22bis_rx_restart()`. Comments cite V.22bis Table 1/2.5.2.2 and
6.3.1.1.2 b)-e): respond to S1, then acquire the 2400-bit/s scrambled ones
before declaring receive readiness.

`v22bis_s1_test` now retains the first failing call as `call5-rx.ulaw`
(12.9-14.9 s) and the TX-only candidate failure as `call6-rx.ulaw`
(12.9-15.5 s). It checks rates on all six, actual delivered receive bits
on original call 2 and new call 6, and both directions at 1200 and 2400
across all 15 dispersive-channel delays (both sides must deliver at least
256 bits as well as report the right rate). All pass locally. Negative
control: detecting on both eyes without selecting the detected eye still
reports TX 2400 on call 6 but fails its receive-bit assertion. The other
recordings diverge at our first changed transmission, so receive readiness
after that point is not asserted from those replays.

After another clean build, the revised candidate passed all five focused
rig test rows: S1, the three MNP gap/S10 cases, and `bell212a_test`.
The final build then completed **three consecutive fresh calls** (calls
5-7), all CX `CONNECT 2400`, last TX/RX 2400/2400, LAPM and V.42bis.
Each delivered exactly 20 numbered lines, in order and once each, in both
directions (1160 upstream bytes, 1280 downstream bytes). Training was at
14040, 14060 and 14060 ms; every call ended with the scheduled hang-up,
after the payload arrived. No call retrained in data.

RX RTP losses were 11, 1 and 0 packets respectively; TX had none. Call 5
had two four-packet gaps and one three-packet gap (80/80/60 ms), with
logged carrier holds of 60/60/10 ms; LAPM delivered the exact payload.
There were no reordered/duplicate packets or timestamp steps unexplained
by sequence loss on any final call. These are LAPM calls, not a fresh
MNP-only carrier-hold qualification.

The final calls' pre-response S1 replays all selected the original eye.
Thus the new eye-selection path is covered by the retained failing
hardware waveform and receive-readiness regression, but did not recur in
these three final live calls. This small call set is not a controlled
acquisition A/B or a statistical pass-rate estimate.
Evidence and SHA-256 manifest are retained locally in
`reference-data/artifacts/cx93001-s1-recheck-20261022/`.

The recorded successful calls trained at 13760-13801 ms, while today's
successful calls trained at 14040-14060 ms. Comparing our own TX taps
shows the same S1/scrambled-ones sequence with an approximately 300 ms
call-timeline offset (the original first successful TX and today's main
success match 99.4% of codewords over the aligned 0.8 s handshake window).
Today's failed main call has no corresponding S1 burst in our TX. Two
V.8 attempts still end without a CM before USB1, consistent with the
previous recordings; the negotiated rate on genuine CONNECTs is unchanged.

## Other datapumps (surveyed 2026-10-21, not changed)

Measured with `engine_pair_test --gap-ms` (V.42 LAPM, `AT+DS=0` so the
payload is still in flight) or read from the code:

- **V.21 / Bell 103 / V.23** (`v21_put_bit`): hang up on the first
  CARRIER_DOWN in data. Same fault as V.22bis had, but there is no trained
  state to keep, so the fix is only the engine's S10 timer. Read from code.
- **V.32bis** (`v32bis.c` over `v17rx.c`): the engine ignores the status, but
  `v17_rx()` calls `v17_rx_restart()` on any carrier drop, which throws away
  the trained receiver and waits for a fresh training sequence the peer
  never sends. Measured: a 30 ms gap into the answerer and the call never
  delivers again, ending at V.42 N400 (T401 retries exhausted). It is a
  hang-up on the first drop with a delay added, and S10 is not applied. A
  fix needs the same hold in `v17rx.c` (or in `v32bis.c` around it), with
  care for the clause 7 retrain watch, which must still see a real retrain
  tone.
- **V.34** (`v34rx.c`): never reports CARRIER_DOWN. Measured: a 30 ms gap
  passes, and a 3 s gap is recovered by the engine's §11 retrain ("V.34
  retrain complete; resuming the existing data link"). It never hangs up on
  carrier loss, so S10 is not applied in this direction either: a dead line
  is ended only by the retrain caps or V.42.
- **V.90 / V.92** (same `v34rx.c` path and status callback): no CARRIER_DOWN
  in data either. Retrain-on-loss is off by default for V.90
  (`ME_V90_RETRAIN_ON_LOSS=0`, `docs/retrain_and_resync.md`). Not measured
  here; the engine pair has no V.90 digital/analogue gap case yet.
- x2, K56flex and clear channel were not surveyed.
