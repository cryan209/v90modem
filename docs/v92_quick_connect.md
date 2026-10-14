# V.92 Quick Connect: short Phase 1 and short Phase 2

Reference: `ITU Docs/T-REC-V.92-200011-I!!PDF-E.pdf`, 8.2.5 (TONEq), 8.3.1
(ANSpcm), 8.3.6 (QTS), Tables 2-5, 9.2.4 (answer modem is digital modem),
9.2.5 (ODP/ADP bypass); V.250 6.8.7/6.8.8 (`+PQC`, `+PSS`).

`+PQC` (V.250 6.8.7) selects what is on: **0** short Phase 1 and 2, **1**
short Phase 1, **2** short Phase 2, **3** neither (the factory value, because
no foreign peer has been qualified). `+PSS=2` forces the full startup. Both
need V.92 on offer (`AT+MS=V92`).

## Short Phase 1: what is implemented

The **digital modem answering a V.8 call** (clause 9.2.4.1), with `+PQC` 0 or
1. It needs the digital V.90/V.92 role and a G.711 bearer.

The chain, in `v92_qc.c` (a pure codeword-in/codeword-out module) and driven
from `modem_engine.c`:

1. V.8 decodes the caller's QC1a while ANSam is still playing. spandsp's
   `v8.c` reports a new `V8_STATUS_V92_QUICK_CONNECT` the moment that frame
   completes (QC1a is followed by CM, but the 9.2.4.1 response does not wait
   for it), gated by the new `v8_parms.v92_quick_connect`.
2. The engine stops feeding V.8 and `v92_qc_t` owns the line:
   QCA1d (Table 5: 70 bits, V.21(H), P = our LAPM offer, LM = ANSpcm level),
   75 ms silence, QTS (128 x 6 symbols at the UQTS the caller's WXYZ selected,
   Table 2), QTS\ (8 x 6), then ANSpcm (Tables 6-10, -12 dBm0) until TONEq.
3. TONEq (980 Hz, 50 ms of 10 ms blocks holding at least 70 % of their energy
   in the 980 Hz bin and at least about -38 dBm0) stops ANSpcm; 75 ms of
   silence follows (600 codewords exactly: the module returns the short
   count, so the silence is not rounded up to an RTP frame), and the engine
   enters Phase 2 as if V.8 had just reported V.90/V.34 (the V.8 result
   handler is called with a synthetic result carrying the caller's QC octet,
   so INFO0d still advertises V.92).
4. No TONEq within 2 s of the end of QCA1d (9.2.4.3): the engine restarts V.8
   (ANSam). A second QC1a on the same call is not answered.

Frame alignment (8.3.6): the first QTS codeword is frame interval 0 and the
digital modem keeps that alignment. `v90_set_tx_frame_phase()` tells the Phase
3 generator which interval each codeword is in, and Sd (whose first symbol is
+W) waits for interval 0.

`make` builds `v92_qc_test` (each signal against the decoders already
validated on captures: strict QCA1d on V.21(H), `v92_detect_qts_sequence`,
`v92_anspcm_table_decode`, TONEq detection, the 2 s limit, Sd alignment) and
`v92_qc_engine_test`, which runs the whole answering engine against a scripted
analogue modem in both laws, with TONEq, without TONEq, and with `+PQC=3`
(the QC1a must be ignored). Both are in `tests/fast.list`.

## Short Phase 1, analogue caller (9.2.1.1, 9.2.1.3)

With `+PQC` 0 or 1 and the analogue role (`ME_V90_ROLE=analogue`), V.8's
caller sends QC1a (Table 2: analogue, QC, P = our LAPM offer, WXYZ = 0101, so
UQTS 70) ahead of CM. When QCA1d (Table 5) comes back, V.8 reports it and the
engine stops CM and goes silent (`v92_qca_t`): once ANSpcm has been heard for
1 s it sends TONEq (980 Hz) for as long as ANSpcm lasts; when ANSpcm has gone
(three 10 ms blocks without it; its phase reversals smear one or two), 75 ms of
silence, then Phase 2 as the analogue role starts it after V.8, with the
received-audio history kept for the digital modem's early INFO0d. No ANSpcm
within 6 s of QCA1d returns to V.8.

The one trap: a digital answerer that has Quick Connect off still answers a
QC1a with the older V.8-embedded QCA octet, ahead of its JM. V.8 therefore
reports QCA1d only when the carrier drops after it (Table 5 ends in ten ONEs
and then silence, 9.2.4.1); a JM preamble following cancels it.

Between two of our engines the whole of Quick Connect (short Phase 1 and 2)
now runs: `engine_pair_test ... --both-at AT+PQC=0` reaches V.92 data mode in
both laws and in V.90 data mode.

## QC2 / QCA2: Quick Connect started by V.8bis (9.2.1.2, 9.2.4.2)

With `ME_V8BIS=1` and `+PQC` 0 or 1 the same sequences start from V.8bis. The
digital modem's CRe is answered by the analogue modem's **QC2a**, a V.8bis-framed
message on V.21(H) (Table 3: first octet 0x2D = type 1011 and revision 0100 in
transmission order, then WXYZ, P, QC, analogue); the digital modem stops CRe and
answers with **QCA2d** on V.21(L) (Table 12: LM, P, QCA, digital), 75 ms of
silence, QTS, QTS\, ANSpcm, and from there it is the sequence above.

- `v8bis_modem` (`cfg.qc2`) hands a type-1011 frame to the engine as an event
  instead of to the V.8bis state machine, and holds a CRe that finds a station in
  the Initial State instead of answering it. `v8bis_modem_send_frame()` puts a
  frame in place of whatever is queued; `v8bis_modem_release_cre()` gives the held
  CRe to the state machine after all.
- The analogue modem waits 1 s after QC2a for QCA2d. Without it the CRe is
  released and V.8bis goes on (the `+PQC=3` answerer case), so a caller with
  Quick Connect on costs a V.8bis call against such an answerer one second.
- The digital modem resumes at the silence after QCA2d, less the idle samples
  that block already held (`v8bis_modem_tx_idle_tail`), so the 75 ms is exact.
- If no TONEq arrives in the 2 s after QCA2d the digital modem sends ANSam and V.8
  (9.2.4.1), as for QCA1d.

An ANSam heard first still sends the caller to the QC1a route; QCA2a (an analogue
answerer) and the digital modem taking the analogue role are not handled.

## Both modems analogue (9.2.1.1, 9.2.1.2, 9.2.3; Figures 7 and 8)

`ME_V90_ROLE=analogue` on both ends, `+PQC` 0 or 1. There is no PCM leg, so the
short Phase 1 ends in an ordinary V.34 Phase 2 and the only tone the two need is
ANSam in place of ANSpcm.

- **Answerer** (`v92_qc_start_analogue_answer`): QC1a (or, over V.8bis, QC2a) is
  answered by QCA1a (Table 4) or QCA2a (Table 5), then V.8 sends ANSam after its
  own 75 ms of silence (`v8_parms.ansam_no_delay` drops the 750 ms hold that
  otherwise precedes ANSam). While ANSam plays the TONEq detector listens
  (`V92_QC_WATCH`); a CM goes to V.8 as usual (9.2.3.4). TONEq ends ANSam, then 75
  ms of silence and V.34 Phase 2.
- **Caller** (`v92_qca_start_ansam`): QCA1a/QCA2a makes it stop CM, go silent, and
  send TONEq once ANSam has been heard (200 ms after QCA1a, 1 s after QCA2a, 9.2.1.1
  and 9.2.1.2); when ANSam is gone, 75 ms of silence and V.34 Phase 2.
- The result handler had assumed an analogue caller always meets a digital answerer.
  It now takes the V.34 path when the peer offers only the analogue role, whether
  or not Quick Connect is on; without that the two analogue modems did not
  connect at all.

Not covered: a digital caller (QC1d, QC2d), which this engine never is.

## A digital modem calling (9.2.2; 9.2.3.1 and 9.2.3.3 for the answerer)

The digital caller already sent QC1d with CM whenever V.92 was on offer (octet
0x45). With `+PQC` 0 or 1 it now acts on the reply:

- **QCA1a** (from V.8; Table 4, whose W0XYZ is the UQTS for our QTS) makes it stop CM
  and, after 75 ms of silence, send QTS, QTS\ and ANSpcm until TONEq comes, then 75
  ms of silence and Phase 2 as the V.90 digital role. Over V.8bis, a CRe is answered
  by QC2d (Table 11) and QCA2a (Table 5) plays the same part.
- The analogue answerer answers QC1d / QC2d with QCA1a / QCA2a (WXYZ 0101, UQTS 70) and
  then, silent, waits up to 2 s for ANSpcm for 1 s (9.2.3.3), sends TONEq until it
  stops, 75 ms of silence, Phase 2. A digital answerer that hears QC1d / QC2d now does
  the same ("may take on the role of the analogue modem", 9.2.4.1/.2): `g_qc_role_swap`
  marks the call, the QCA1a/QCA2a reply and the ANSpcm wait are the analogue answerer's,
  and the analogue role is borrowed for the rest of the call (`g_v90a_role_borrowed`, restored
  at the next call), so Phase 2 onward is the analogue modem's. Two default-role engines
  connect after it (V.92 56000/48000).

An analogue-role engine ANSWERING a digital caller runs the real Phase 2 and Phase 3/4:
`g_v90a_answering` makes `start_v34_training` initialise the borrowed SpanDSP engine as the
analogue ("calling") side whatever the call direction (docs/v90_analogue_role.md). The pair tests
for these cases are ordinary CONNECT tests, short Phase 2 included.

## Short Phase 2 (9.4), both roles

With `+PQC` 0 or 2 each modem's INFO0 carries the V.92 capability bit and the
short-Phase-2 request (INFO0d bits 27 and 26, INFO0a bits 26 and 27). When both
sequences carry both bits, and the INFO0 exchange was error-free, the Phase 2
ranging is replaced by one Tone transaction (`v34tx.c`, `v92_short_*`):

- digital: INFO0d, Tone B; once Tone A is detected and Tone B has run 50 ms, a
  Tone B phase reversal, 10 ms more Tone B, silence (9.4.1.1.3);
- analogue: Tone A; on detecting that reversal, 40 ms later its own Tone A
  reversal, 10 ms more Tone A, then INFO1a (9.4.2.1.3/.4);
- digital: on the Tone A reversal, round trip delay = the interval minus 40 ms
  (9.4.1.1.4), silence, then INFO1a and Phase 3.

There is no probing, no L1/L2 and no INFO1d. Two consequences are handled
explicitly: the analogue modem has no INFO1d row to take the upstream carrier
from, so INFO1a's Table 19 bit 33 carries it (the digital side applies it, and
the engine's V.90 INFO1a check accepts raw 32:33 = 2 only in this case); and
the analogue modem picks symbol rate 3200 by default and PCM upstream
(Table 18) when it is configured for it, since the digital modem has said nothing
about that.

Measured on two engines (`engine_pair_test`, analogue role against digital):
Phase 2 from the first INFO0 to the start of Phase 3 takes 0.46 s against 1.78 s
for the full procedure, in V.92 PCM-upstream mode (48000/56000), V.90 data mode
and A-law.

Timers and recovery (9.4.1.2.2/.3): if the Tone A reversal or INFO1a has not
arrived 2500 ms after the digital modem's reversal, or Tone A has not appeared,
the digital modem sends Tone B and the full procedure continues; the analogue
modem gives up on the digital modem's reversal after 5 s and does the same. Either
way the full Phase 2 is then kept for the rest of the call. `V92_SHORT_TEST_NO_REPLY=1`
makes the analogue modem ignore the reversal so that path runs end to end
(`tests/fast.list`). A retrain omits INFO0 and never starts a short Phase 2.

INFO0 recovery (9.4.1.2.1, 9.4.2.2.1): when the digital modem repeats INFO0d (bit 28 set
after a correct INFO0a) or the analogue modem repeats INFO0a, both then "proceed
according to 9.4.1.1.3" -- the short procedure carries on. `v34tx.c` calls
`v92_short_begin` at the points where the recovery hands back to Tone B (the
acknowledged-exchange exit and the retry-guard exit), when the armed flag and the
received INFO0 bits 26/27 still agree. `V92_SHORT_TEST_INFO0_RECOVERY=1` makes the
digital modem take the recovery once (`tests/fast.list`). Before this the recovery
quietly left the call on the full Phase 2 on the digital side, which the analogue
side then waited out.

Not done: nothing here has met a foreign V.92 modem.

## Qualification against slmodemd (Proxmox rig, 2026-10-13)

First contact with a foreign V.92 modem. Rig: Proxmox host, CT 100 (`slmodem`) runs the
**stock** D-Modem (`slmodemd` + `d-modem`, Asterisk account 6003), CT 117 (`v90modem-dev`)
runs our `sip_v90_modem` (account 6002); driver `tools/soak/qc_slm_pve.sh` (run on the
Proxmox host, `pct exec` into both). Limits of this rig: it is NOT the tower's build -- no
sinc resampler (slmodemd's internal RcFixed 9.6k<->8k instead, which the tower notes say
guts V.90 downstream), so no data mode; the repo's `rig/d-modem/d-modem.c` speaks a
different socket protocol from this slmodemd and cannot replace it.

slmodemd dials us (`AT+MS=92`), us digital, µ-law:

| `+PQC` | result |
|---|---|
| 3 (factory) | slm's QC1a (WXYZ 9, LAPM, ANSpcm -15 dBm0) is ignored, ordinary V.8, V.90 Phase 2; ends in retrains / NO CARRIER (rig resampler) |
| 1, 0 | **short Phase 1 completes against real slmodemd**: we answer QC1a with QCA1d, QTS (UQTS 78) and ANSpcm; slm detects them and sends **TONEq** (9.2.4.3 "TONEq detected", 75 ms silence, Phase 2) |

Phase 2 afterwards: slm's INFO0a carries V.92 capability (bit 26) but **not** the short-Phase-2
request (bit 27), so both ends correctly select the full Phase 2 (`+PQC=0` degrades to the same
as `+PQC=1`, no "abandoned" fallback needed). The call then behaves like the `+PQC=3` baseline:
INFO1a is rejected ("upstream_code=0, downstream_code=6, v92_contract=0") or a Phase 3 retrain
follows, both caused by the rig's resampler, not by Quick Connect.

Not qualified: our analogue caller against slmodemd answering (`ATA`): slm is an analogue-only
modem (`digital_side=no`) so it can never answer QC1a with QCA1d, and the runs did not get that
far -- the PBX answered the call itself after slm's REGISTER drew a 403 (baseline `+PQC=3`
failed identically). Short Phase 2, ODP/ADP bypass and data mode against slmodemd need the
tower's resampler build of slmodemd/d-modem, which is not on this rig.

### Attempt at the tower's sinc-resampler build (same day)

Rebuilt on CT 100 as `/root/D-Modem-rig`: `git worktree` of cryan209/D-Modem at `cf5ea9e6` (the
commit before slmodemd gained its internal RcFixed resampler, `fd78cab9`) with the original
`dsplibs.o`, `make -C slmodemd` (gcc -m32), plus the repo's `rig/d-modem/d-modem.c` (6/5 sinc,
`SIP_LOGIN`, two positional args) built against the main tree's pjsip.install.
`qc_slm_pve.sh` uses it by default (`SLM_BUILD=rig`; `SLM_BUILD=stock` for the old pair).

Result before the fix below: no data mode, and Phase 2 never confirmed INFO0: our digital side
never decoded slm's INFO0a (rx went INFO0 -> TONE_A) while slm decoded ours -- also on the commit
before any Quick Connect work (`44164407^`), so not a regression.

**Cause (found 2026-10-13): the rig's slmodemd frames its audio to d-modem as
`struct { s16 buf[192]; int volume; }`** -- a 4-byte trailer after every 20 ms -- and the repo's
`rig/d-modem/d-modem.c` read only the 192 samples, so the trailer was consumed as two more
samples each frame: a 190/192 time stretch. Measured on the wire: slm's Tone A at **2377.75 Hz**
(2400.0 on the stock build, whose pair speaks a different protocol), TONEq at 975 Hz, the
9600 Hz tap (`/tmp/dm_from_dsp_9600.raw`) at 2372.5 Hz; INFO0a's 600-baud DPSK drifted half a
symbol over its ~60 symbols, CRC `0x195f`. (Tone-level tells: a sinc re-resampling of the 9600
tap did not help -- the error is the time base, not the interpolator.) `d-modem` now takes
`DM_SLM_FRAME_TRAILER=4` (set by `qc_slm_pve.sh`) and skips the trailer.

With it, on the rebuilt rig: INFO0a decodes, "INFO0 confirmed mutually", INFO1d/INFO1a exchange,
Phase 3 starts and our Ja descriptor parses, for `+PQC` 3, 1 and 0 (Quick Connect Phase 1 as
above before it). slm's first INFO1a selects a 2400 baud V.34 upstream, which our strict check
(3000/3200 only) declines, we retrain through Phase 2 and the second INFO1a (3200) is accepted.
**That hangup was my own test script.** The numbered-line payload began typing into our serial
port while the call was still in handshake, and `PAYLOAD=` (empty) was being read as the default
(`${PAYLOAD:-1}`), so the "no payload" runs had it too; a DTE character aborts a call that has
not connected (fixed: `${PAYLOAD-1}`, `PAY_DELAY` must exceed the connect time, ~55-70 s).

Final matrix (rig build, `AT+MS=92`, slm dials us, µ-law, digital side ours, no payload typed
before connect):

| `+PQC` | Phase 1 | Phase 2 | result |
|---|---|---|---|
| 3 | slm's QC1a ignored (ordinary V.8) | full | `CONNECT 54666` |
| 1 | **short Phase 1 completes** (QCA1d/QTS/ANSpcm -> slm's TONEq) | full (slm never asks for short) | `CONNECT 54666` |
| 0 | as 1 | full | `CONNECT 54666` |

All through one Phase 2 retrain (slm's first INFO1a picks 2400 baud, which we decline per the
V.90 rules; the second picks 3200). slm reports `V92 capabilities: local=0 ... selected=90`: its
V.92 path is off without the PCM-upstream patch (`rig/slmodemd/v92-pcm-upstream.patch`), so these
are V.90 data modes and the ODP/ADP bypass (V.92 only) correctly does not engage. With `\N0` on
both DTEs (`SLM_NPARM='\N0' OURS_INIT='AT+PIG=0;AT\N0'`) both ends print CONNECT (54666 / 54667) and
**11928 of 11949 numbered lines ours->slm arrive (99.8 %)** over the PCM downstream. The V.34
upstream slm->ours delivered no lines (0 D-lines; the DTE saw garbled bytes): the known V.90
upstream-receive problem (docs/v90_upstream_*, memory notes), unrelated to Quick Connect.

Qualification summary: Quick Connect short Phase 1 interoperates with slmodemd's QC1a/TONEq
handling, does no harm to the rest of the call, and is the same through data mode as `+PQC=3`.
Not exercised against slmodemd: short Phase 2 (slm never requests it), the V.92 ODP/ADP bypass
(slm falls to V.90), the analogue-caller and digital-caller QC roles (slm is analogue-only and
only ever calls QC1a).

### Correction: `+PQC=3` was answering QC1a with QCA1d (2026-10-13, evening)

Every `+MS=92` call above logs `V8: QCA1d: Got Good QCA1d` at slmodemd, `+PQC=3` included
(and so does the 2026-10-11 build, `9c818516`). As the digital answerer our V.8 octet is
`0x47` (QCA bit set), and `conditionally_send_v92()` sent it whenever the peer had sent QC1a,
whatever `+PQC` said. That is a QCA1d with no QTS/ANSpcm behind it: 9.2.4.1 ties QCA1d to
"silence for 75 +- 5 ms and then QTS, QTS\ and ANSpcm", and 9.2.1.1 has the caller stop CM
and wait for them. slmodemd accepted it, ran its QC line verification against nothing and
set its quick-connect flag. So the `+PQC=3` row in the tables above did NOT run "ordinary
V.8". v8.c now withholds QCA when Quick Connect is off (regression:
`test_v8_v92_qc_exchange_over_analog_g711`); live, slmodemd logs
`quick connect indication from phase1 = 0`.

Consequence for V.92 against slmodemd: its DSP enables V.92 only for a Quick Connect session
(`initial Session Type = 2`, `localV92- 1`); without QC it starts Session Type 1, reports
`V92 capabilities: local=0` and selects V.90. Every V.92 PCM-upstream result against
slmodemd so far (the tower's 2026-10-11 data calls included) got V.92 through the unsolicited
QCA1d. The conformant route is real Quick Connect (`+PQC=0/1`).

### V.92 PCM upstream on the Proxmox rig: deterministic three-attempt failure

`slmodemd_map` (PCM-upstream patch + V92EchoCanceller bypass, `rig/slmodemd/README.md`),
`SLM_V92_PCM_UPSTREAM=1 DM_V92EC_BYPASS=1`, ours `ME_V92_PCM_UPSTREAM=1 AT+PIG=0`,
`AT+MS=92,0,300,56000`. Identical with the unsolicited QCA (`+PQC=3`, before the fix) and with
genuine Quick Connect (`+PQC=1`, after it); slmodemd's log is byte-for-byte the same size
call to call:

1. Startup V.92 Phase 3: slmodemd enters `waitForJd` ~0.16 s after our Jd starts and never
   logs `Jd Detected`; retrains after 3.1 s.
2. Retrain 1: `Jd Detected` 0.4 s after `waitForJd`; Phase 4 rejected,
   `ph4MeanErrorEnergyBeforeToAfterUpdateRatio` +2.531 (+2.228 with real QC).
3. Retrain 2: we withdraw the PCM offer, V.90-form data at 54667/31200; slmodemd's data EE
   climbs 20 -> 60 within 0.8 s and it drops to V.34 (`drop to V34 requested`, call ends).
   In plain V.90 calls the same first-data climb happens, and they survive it only because
   slmodemd still has a retrain left.

Ruled out, each by a run:
- Our Jd: CRC-valid, matches Table 21 (bits 41:46 zero, bit 47 = 0), and identical in the
  missed and the detected attempt.
- The path: `tools/analyze_dmodem_pcm_path.py` puts every Sd at slmodemd's DSP with
  correlation 1.000 and a constant 45 ms delay.
- TRN1d length: 12000, 16008, 24000 and 30000T all miss in attempt 1.
- Our build: `9c818516` (2026-10-11) behaves identically, so this is not a regression.
- slmodemd's startup choice of 2400 baud: dropping INFO1d rows (2743/2800 were offered at
  startup only, because `v34_restart` clears the far INFO0a baud capabilities) did not move
  it; slmodemd's choice comes from its own probe analysis. `SLM_CODECTYPE=4` (AonCyberLabs'
  value; this fork says AD1803 = 12) makes it pick 3200, but attempt 1 still misses Jd.
- The DSP blob: `dsplibs.o` here is AonCyberLabs' (`e64b4a7`).
- `trn1Sigma` (which differs between the attempts) is a double printed with `%d`
  (`fstpl` at `.text` 0x4342d), so it is noise in the log line, not an overflow.

- The wrapper's ioctl answers: `SLM_CODECTYPE=4 SLM_IODELAY=0` together (AonCyberLabs' values)
  with genuine QC (`both_pqc1`): startup still misses Jd; slmodemd picks 3200 baud at startup;
  the retrain-1 Phase 4 ratio drops to +1.858 (the tower's failing retrain logged +1.825) and
  is still rejected; same V.90 drop.

Open: why slmodemd's FIRST V.92 Phase 3 misses an intact Jd. The `V90Phase3Demodulator` state machine (`getV90Decision`, `.text` 0x23830) has `Wait For Jd` /
`Wait For V92Jd` states whose log lines are below `-d9`.

### Root cause of the startup "Jd miss": our Su-bar detector (2026-10-13, night)

`rig/slmodemd/phase3_decision_hook.c` interposes on slmodemd's per-symbol Phase-3 state
machines (`getV90Decision` / `getV92Decision`, one call site each) and logs every state
transition. slmodemd did NOT miss Jd: in attempt 1 it entered Wait-For-V92Jd (state 7) and
detected our Jd 131 symbols later (7 -> 8); state 8 waits for **Jp** and timed out after
24803 symbols (`JdPhaseDemod TimeOut`, 8 -> 23). Its `Jd Detected` line prints only at a
later stage, which is what made it look like a miss. We never sent Jp because our
`v92_su.c` never reported Su-bar.

9.5.2.1.7 sends Su-bar for only 24.5T (about four of the detector's six-symbol blocks) and
it lands anywhere on the detector's block grid. The detector required a block to differ
from the previous one by under 3%, and with the line's ISI none of the reversed blocks did
on most alignments: replaying the live tap, attempt 1's Su-bar is missed at 5 of 6
alignments and attempt 2's at 2 of 6. So success depended on where the burst fell. The
detector now also evaluates the same bin with a sliding window on the absolute clock and
takes six consecutive reversed windows as Su-bar (`v92_su_test`: both live slices at all
alignments, synthetic 24.5T Su-bar for 12 phases x 2 laws, no false Su-bar without a
reversal; the old detector misses 7 cases).

Live (`sufix1`, +PQC=1): slmodemd detects Jd and gets Jp in attempt 1, Phase 3 completes,
its TRN2 check passes (ratio +1.171, `disable linear mapping study`), it designs 69-point
constellations and the SUVd/SUVu exchange runs. Next blocker, ours: the CPd design
refuses the upstream, `CPd profile cannot arm B1u receiver (noise sigma 193.27 DS0, rx rms
1252)` (the tower measured ~27), no CPd goes out, and slmodemd ends on `Phase4 TimeOut`.

### Upstream sigma 193 was the rig: d-modem's linear 9600 -> 8000 interpolation

d-modem taps slmodemd's transmit before (`dm_from_dsp_9600.raw`) and after
(`dm_from_dsp.raw`) its 9600 -> 8000 conversion. Against an ideal resampling of the first,
the linear interpolation leaves an error 12.8 dB below the signal throughout the V.92
segments (time-varying across its five phases, so no equaliser removes it). Our TRN2u error
was 0.154 LU, just under that floor. `SLM_CODECTYPE=4 SLM_IODELAY=0` changed nothing
(sigma 192.3). `rig/d-modem/d-modem.c` now has `DM_UP_RESAMPLER=sinc`, a 65-tap Hann sinc
decimator (192 in / 160 out per frame, constant 33-sample delay), and `qc_slm_pve.sh` uses
it by default.

With it (`upsinc1`, +PQC=1): TRN2u error **0.019 LU, sigma 26.6 DS0** (the tower's figure),
CPd designed for **32000 bps** (20 points), and slmodemd reaches **V.92 data mode with PCM
upstream**, `Link: DP is V.92, rate: rx 54667, tx 32000`, `CONNECT 54667`. Not yet usable:
- our B1u never locks: after 10 s the best alignment holds 3 of 16 frames, the equalised
  data sits 0.10 LU from the nearest level (142.7 DS0 rms), and the data-mode equaliser
  re-solve was refused 5 of 6 times (residual above twice the first); we never print
  CONNECT, no lines move either way, and slmodemd hangs up.
- slmodemd's Phase 4 verdict on attempt 1 varies between calls (ratio 1.171 passed in
  `sufix1`, 2.504 failed here), so attempt 1 does not always reach data.

### B1u in data mode: our decision feedback, measured against slmodemd's own symbols (2026-10-14)

`rig/slmodemd/phase4_symbol_hook.c` records slmodemd's transmitted upstream symbols
(`V92Phase4Modulator::generateSymbol` for Phase 4, `V92Transmitter::process` for data -- its
`written` argument is the exact count; hooking `V92BitsToSymbol::process` instead reads stale
buffer). slmodemd transmits our CPd exactly: 100% of 145644 data symbols on +-1.3733 x point,
rms 0.98 LU. The bearer is clean (our tap = mu-law of d-modem's output, one fixed lag, error
<= 1.4 quantisation steps), and a least-squares FIR fitted against the true symbols reads
0.030 LU on TRN2u and 0.033 LU on data. So the path is linear and decodable; the fault is ours.

Against the true symbols, our equaliser reads 0.018 LU through TRN2u, SUVu and CPu, then at
the CPd acknowledgement (data slicer) its output jumps to three times the transmitted level
within ~200 symbols. The cause is the 8-tap decision feedback trained on four-level TRN2u:
on the 20 dense data levels its wrong decisions recirculate (slmodemd's transmit FIR is a
high-pass whose inverse leans on the feedback). Fixes:
- `v92_p3_eq` also solves the feed-forward-only fit from each least-squares window (the
  top-left block of the same normal equations) and `v92_p3_eq_use_ff()` switches to it with
  feedback off; the engine does so at the data slicer.
- `me_v92_p4_eq_mode_locked()` no longer releases the data slicer's hold on every block.

Replay of `p4sym3` (CPd unchanged): data 0.227 -> 0.050 LU from the nearest level, rms
distance 142.7 -> 72.3 DS0, data-mode re-solves 1 accepted / 5 refused -> 6 / 0, best B1u
candidate 3 -> 6 of 16 frames. **Still no lock**: 0.050 LU against a 0.079 half-spacing is
~10% symbol errors. TRN2u-trained linear taps give 0.049 on this data while data-trained ones
give 0.033, and decision-directed re-solves cannot get there through those errors. Next:
train on B1u data-aided (known scrambled ones through the CPd mapper), or design the CPd with
margin for 0.05 LU.

Tools: `v90_engine_replay --at "AT..;AT.."` plays the live DTE's configuration before the call
(the `+PQC`/`+PIG` a V.92 replay needs; `--fast` reproduces the live numbers exactly);
`v92_slm_chain_test` has the live 20-point profile (locks in the model) and
`CHAIN_DMODEM_UP=sinc65` for the new d-modem decimator.

### B1u data-aided training: B1u locks, V.92 startup completes (798af366, 2026-10-14)

V.92 8.7.1 starts B1u from zero scrambler, encoder, precoder and prefilter memories, so the
CPd alone fixes all 576 B1u symbols; our encoder regenerates slmodemd's (hooked) B1u exactly
(rms difference 0.38 units of ~4000, min-|x| member). The data equaliser now keeps each
symbol's input window from the slicer on, finds B1u by correlation after 1400 symbols, solves
the feed-forward taps against it, re-equalises the capture and restarts the B1u receiver on
it. It also samples whole input samples from the slicer on: at the TRN2u fractional offset
the 16-tap interpolator left 0.12 LU against B1u that no taps removed; on whole samples the
B1u solve reads 0.036 LU.

Live (`b1u1`, +PQC=1, slmodemd_map): **B1u locked by decoding (all ones), V.92 startup
complete, both DTEs CONNECT (54666 / 54667), slmodemd `Link: DP is V.92, rx 54667, tx 32000`,
and 22797 numbered lines downstream reached slmodemd's DTE.** The upstream does not hold in
data: 5 lines reached our DTE, rejected frames 201 at 10 s -> 8537 at 70 s (~18%), data error
~72 DS0 (0.05 LU). After B1u the 2 s re-solves are decision-directed on the slicer again and
settle back at TRN2u quality. Next: data-aided re-solves in data mode from the trellis
decoder's output re-encoded (the decoded bits through our encoder give the transmitted
symbols wherever the decode is right), instead of the hard slicer.

### Data re-solves from the trellis decoder (372f0979)

The Viterbi survivor's points of each decoded frame (G x point; no precoder or prefilter
coefficients here, so what was transmitted) become the least-squares targets for the input
windows that produced them; the hard slicer no longer trains the data equaliser. Window 4000
symbols (0.5 s). No wrong targets reach it (|equaliser - target| > 0.3 LU: 0 of 120000 rows).

Live (`aided1`): rejected upstream frames at 10 / 20 / 30 s of data 48 / 272 / 888 (B1u-only
training: 201 / 450 / 1150), data distance 58.8 DS0 at 10 s but 72 DS0 from ~30 s on, the same
plateau as before; 12 lines upstream, 24168 downstream. So the first seconds are fixed and
something over 10-30 s is not: a 0.5 s T-spaced re-solve with correct targets cannot follow
it. Two leads: the aided-row error is not uniform across the 12-symbol frame (0.055 LU at
positions 0-3, 0.044 at 8-11, on the replay), and the degradation starts before any payload
(idle ones until 60 s), so it is not data-dependent.

### Ground truth over a full call: slmodemd's transmit timing offset; error-free V.92 both ways

`phase4_symbol_hook.c` over a 96 s data call (`gt90`), sliding least-squares fits of our
received tap against slmodemd's transmitted symbols: one fixed lag for the whole call (no
slips), but the best achievable linear fit per 0.5 s went 0.033 LU at the start -> 0.12-0.14
LU at 40-60 s -> 0.035 at 95 s, and shorter windows (250 symbols) did not help. So from about
10 s on our receiver was at the limit of what any linear equaliser could get: the received
upstream itself stopped fitting a fixed channel. The residual was 81% in 3800-4000 Hz at 45 s
(29% at 5 s), varying from one 10 ms block to the next, kurtosis ~3 (not glitches).

A hook on `ResamplerTimingOffset::setTimingOffset` (12 call sites;
`V92Modulator::progress` calls it every block) shows slmodemd applying its receiver's timing
estimate, ~+0.57 ppm drifting to +0.556, to its 8000 -> 9600 transmit resampler. On the SIP rig
both modems share one clock, so the true offset is 0; the resulting slow transmit-phase drift
(~0.45 samples over the call) makes that resampler's error near Nyquist grow and fade.
`rig/slmodemd/slm_tx_timing_freeze.c` (linked into `slmodemd_map`; `qc_slm_pve.sh` sets
`SLM_FREEZE_TX_TIMING=0` by default) holds the offset at 0:

| | offset tracked | offset 0 |
|---|---|---|
| data distance | 58 -> 72 DS0 | 35 DS0, steady |
| rejected upstream frames, 90 s | 12594 | 55 |
| B1u solve residual | 0.036 LU | 0.022 LU |
| upstream lines (`\N0`) | 8 | 16425 (DTE overrun loses the rest: 500 lines/s > 32000 bps) |

**With LAPM (`\N3` both ends, `lapm1`): V.92, PCM upstream 32000 / downstream 54666, ODP/ADP
bypassed (9.2.5), V.42bis negotiated; 18037 upstream lines and 21100 downstream lines, each a
contiguous span with no gaps -- error-free in both directions.** First complete V.92 data
transfer against slmodemd on this rig.

This is a property of slmodemd on a shared-clock rig, not something our receiver can undo
(the best linear fit degrades, not just ours). A real V.92 analogue modem does slave its
upstream timing to the network clock; whether our receiver needs timing tracking for a peer
whose estimate wanders is open.

### Repeatability: 5/5 error-free (2026-10-14, `rep1`..`rep5`)

Same settings as `lapm1` (+PQC=1, `\N3` both ends, `slmodemd_map` with `SLM_FREEZE_TX_TIMING=0`,
130 s hold, payload from 60 s), five calls back to back:

| call | link | up lines (gaps) | down lines (gaps) | rejected upstream frames |
|---|---|---|---|---|
| rep1 | V.92 54667 / 32000, V.42bis | 26884 (0) | 22450 (0) | 39 in 90 s |
| rep2 | same | 28000 (0) | 27950 (0) | 31 in 80 s |
| rep3 | same | 25950 (0) | 28250 (0) | 35 in 80 s |
| rep4 | same | 27900 (0) | 27850 (0) | 24 in 80 s |
| rep5 | same | 28250 (0) | 28200 (0) | 8 in 80 s |

Every call takes exactly one retrain, identically: slmodemd's first Phase 4 check reads
`ph4MeanErrorEnergyBeforeToAfterUpdateRatio` +2.504 and retrains; the second reads +2.310 and
passes into data. That ~15 s first-attempt retrain is the remaining startup cost.

### The first-attempt Phase 4 retrain (2026-10-14)

`rig/slmodemd/phase4_eval_hook.c` logs every `V90ConnectionEvaluator::evaluatePhase4(ratio)`
call (`slmodemd_eval`). The decision path (`.text` 0x3fad0): while the evaluator's flag at
+0xb0 is set, a `ph4MeanErrorEnergyBeforeToAfterUpdateRatio` above **`V90Parameters`+0x438 =
1.500** initiates a retrain ("due to meanErrBefToAftUpdateRatio"), at most +0x45c = 2 times. The
retrain clears the flag, so **the ratio is checked once per call**: attempt 2 (2.31) is not
judged at all. Both attempts are above the threshold; slmodemd simply stops looking.

The ratio follows the rig's upstream path, deterministically per configuration:

| upstream 9600 -> 8000 in d-modem | added delay | first-attempt ratio | retrain |
|---|---|---|---|
| linear interpolation | 0 | 1.172 | no (`eval_lin`; the upstream data then fails, sigma ~190) |
| sinc, half-length 8 (`DM_UP_SINC_HALF=8`) | ~9 | 1.697 | yes (`h8`) |
| linear + 33-sample delay (`DM_UP_DELAY=33`) | 33 | 1.939 | yes |
| sinc, half-length 32 (default) | 33 | 2.505 | yes |

Our TRN2d length barely matters (2040T 2.481, 3996T 2.505, 8004T 2.668), and our Phase 4
timeline (CPt, Ri, 24T barred Ri, TRN2d) is the same in every row, so the effect is not in what
we transmit. Open: how slmodemd's upstream path (both its delay and its frequency response)
reaches its downstream equaliser's before/after-update error -- the next step is a hook on
`V90Equalizer::calcMeanErrorStatistics` to see which windows the two energies (+0x84, +0x13c)
cover. Workable now: accept the one retrain (~15 s), or the linear path for startup-only tests.

#### The two error windows (`phase4_errstats_hook.c`)

`V90Equalizer::calcMeanErrorStatistics` averages the equaliser's error-energy history (float
array at +0x98, 45 entries here). `V90Demapper::linearMappingStudy` raises two events during
TRN2d (+0x1ea4, +0x1ea6); at the first the mean is saved as "before" (+0x13c) and the history
cleared, at the second the new mean is "after". The update between them is slmodemd replacing
its downstream level map (from Phase 3's `findPadGain`) with levels measured on TRN2d.

| upstream | before | after | ratio | `findPadGain` best fit error |
|---|---|---|---|---|
| sinc (default) | 22.8 | 9.1 | 2.505 | 10512 |
| linear | 12.9 | 11.0 | 1.172 | 424 |

The "before" window is a flat ~10-unit offset over all 45 entries (same bumps at the same
entries in both arms), not a slower convergence, and it vanishes at the update. The sinc arm's
"after" is the better of the two. So with the sinc upstream slmodemd's Phase 3 pad-gain fit
of our DIL is 25x worse, its initial level map is off, and the measured-level update fixes it;
the evaluator reads that improvement as "not trained enough".

Our transmitted downstream is bit-identical between the arms up to TRN1d; after Jd it is the
same content shifted in time (Jd length follows our Su-bar detection). d-modem shares no state
between its directions. Our Jp always sends a zero phase extension (Table 22 bits 18:33;
9.5.1.1.6-8 has the digital modem measure it from Su), and slmodemd has
`adjustUinfoToPhaseOffset()`, but a fixed fraction does not fix the fit (0.25: ratio 3.10,
fit 7338; 0.5: 2.26, 13684; 0.75: no Phase 4 on attempt 1), so that is not it as stated.
Open: what in slmodemd's Phase 3 DIL analysis follows its upstream path.

#### It is Jp's content, not the upstream path (2026-10-14, afternoon)

The upstream path was only a proxy. Its delay moves our Su-bar detection across a Jd
boundary: every sinc call ran Jd for 4536 symbols, every linear call 4392. Two diagnostics
in `v90.c` separate the variables (both off by default, neither spec behaviour):
`ME_V92_JD_EXTRA_REPS=N` sends N more 72-bit Jd repetitions after Su-bar, and
`ME_V92_JD_SKIP_REPS=N` runs the Jd generator N repetitions without sending them, so Jp
starts from the same scrambler and differential-encoder state with no shift in time. The
Jd->Jp log line now prints that state. Linear upstream throughout (`jdab.sh` on the
Proxmox host); every number repeats exactly (rep 0 twice: 424, 424):

| Jd reps | Jd | sent? | pad-gain fit | Phase 4 ratio |
|---|---|---|---|---|
| +0 | 4392 | -- | 424 | 1.172 |
| +1 | 4464 | sent | 1968 | 1.361 |
| +2 | 4536 | sent | 16894 | 1.985 |
| +3 | 4608 | sent | 6871 | 1.947 |
| +4 | 4680 | sent | 1628 | 1.227 |
| +5 | 4752 | sent | 45996 | 2.923 |
| +6 | 4824 | sent | 26011 | 3.146 |
| +7 | 4896 | sent | 2542 | 1.445 |
| +8 | 4968 | sent | 3143 | 1.470 |
| +2 | 4392 | skipped | 7621 | 1.875 |
| +5 | 4392 | skipped | 28939 | 2.868 |
| +6 | 4392 | skipped | 12961 | 3.247 |

Skipping the repetitions reproduces sending them, so **slmodemd's DIL analysis depends on
the scrambled content of our Jp/Jp'**, pseudo-randomly in the scrambler state, not on when
it arrives or on the upstream. The DIL itself is identical in every arm (absolute SP signs,
V.90 8.4.1, as we send it), and so is the DIL-to-CPt time (4540 ms), so slmodemd does not
cut its study short. Jp's sign balance does not predict it (+5 is positive and worst, +8
negative and good), nor does the differential sign at Jp start.

What goes wrong is visible in slmodemd's `linearMapping` (its measured DIL levels). In a
good state every level is 4.000x the mu-law value (residual 2.5 after a gain+offset fit).
In a bad one the error has one sign for the whole call (+5/+6 high, +2 low), grows with
the level (Ucode 115: +403), and steps at particular DIL segments (Ucode 64 -> 63: +87 ->
+28); residual 20-28. That is an equaliser whose response was left wrong by Jp and keeps
drifting through the DIL (its error energy climbs 6 -> 13 there, against a flat 8.5 when
good).

A line-like DC block in d-modem's downstream (first-order 100 Hz) did not help: the good
state got worse (424 -> 9987), the content dependence stayed (skip 5: 132063, skip 2:
7275), and Jd stretched to 5544. Reverted, so slmodemd needs the DC the digital path
passes.

The lever we own is Jd length: 9.5.1.1.8 ends Jd only "once the digital modem has
determined the proper phase adjustment", in whole repetitions. But which states are good
is a property of slmodemd's equaliser, and nothing we can compute predicts it yet. Next:
dump slmodemd's equaliser taps at DIL start (a hook like `phase4_errstats_hook.c`) for a
good and a bad state and compare their responses; if Jp's content excites a direction its
adaptation leaves unconstrained, that says what a robust Jp/Jd ending would be.

#### slmodemd's equaliser through Jp and the DIL (`eq_tap_hook.c`)

Taps dumped every 400 symbols (`slmodemd_eq`), Jd skip 0 (good) against 2, 5, 6 (bad), linear
upstream; the hooked calls reproduce the unhooked fits exactly (424, 7621, 28939, 12961).
The equaliser is 300 T/2 linear taps plus 12 symbol-spaced DFE taps.

- Every run is identical up to slmodemd's "Jd Detected". During Jp the bad runs pick up a
  small in-band difference (|taps - good| 0.003-0.004 at Jd, 0.007-0.017 at DIL start),
  rising with frequency toward 4 kHz, and nothing above the data band.
- The DIL then amplifies it instead of removing it: 0.024 (skip 2), 0.066-0.068 (skips 5, 6)
  by Phase 4. Each run also drifts against its own DIL-start taps, the good one included
  (0.021; bad 0.031-0.056).
- The DFE carries the drift. Its first tap sits at about -0.66 in every run; through the DIL,
  skip 5's goes -0.62 -> -0.41 while the linear DC gain stays at 1.016-1.018, so the combined
  low-frequency gain lin/(1 - sum DFE) climbs 0.58 -> 0.90 (good 0.57 -> 0.77, skips 2 and 6
  0.58 -> 0.59-0.60). The DIL's long runs of one level make the linear and DFE taps nearly
  collinear, and decision-directed adaptation wanders along that ridge.

So the mechanism is slmodemd's equaliser adapting on the DIL it requested itself (N=144,
LSP=LTP=120), poorly conditioned, from a starting point Jp's content perturbs. It is not a
defect in what we send. A first DFE tap of -0.66 with a flat linear response (1.015 from 150
to 3950 Hz) is itself odd for this nearly ideal digital channel and suggests a fixed filter
in slmodemd's receive front end; that is not established. Options: accept the one retrain
(about 15 s); pick a Jd length (spec-legal, 9.5.1.1.8) from a table measured for this peer,
which is fitting slmodemd and not a fix; or find what puts -0.66 in the DFE.

#### What puts -0.66 in slmodemd's DFE: the 4 kHz band edge of the rig's downstream

Captured slmodemd's equaliser input (`EQ_DUMP_INPUT=1`) on a good call and fitted it against
our transmitted TRN1d:

- The channel into its equaliser is clean: per T/2 phase a plain fractional delay of a
  band-limited signal (phase 0 = sinc(n - 0.23), phase 1 = sinc(n + 0.27)).
- Its own linear taps alone already give x[n]: correlation 0.9998-1.0000, every ISI term
  under 1%, residual 0.4% of rms. So the 12-tap "DFE" has no ISI to cancel; it acts on past
  errors as a noise predictor. The residual has lag-1 autocorrelation -0.67/-0.76 and 71-85%
  of its power in 3-4 kHz, and its optimal one-step predictor's first tap is -0.658 against
  the dumped -0.666.
- Not slmodemd's pre-filter: `V90PreFilter` (`prefilter_hook.c`) is the identity through
  Phase 4 here.
- d-modem's 8000 -> 9600 downstream (257-tap Hann sinc, cutoff at exactly 4000 Hz) is -32 dB
  re signal against an ideal band-limited resampling, 99.4% of the error in 3.8-4.2 kHz: its
  transition band straddles the input Nyquist.

The fix that suggested itself does not work. d-modem gains `DM_RS_TAPS` (any odd length at the
same cutoff, on a ring-buffer pipeline), `DM_RS_EXTRA_DELAY` and `DM_RS_WINDOW=kaiser`; all off
by default. Skip 0 unless noted, linear upstream:

| downstream kernel | pipeline | slmodemd | TRN1d error energy | pad fit | ratio | DFE 1, 2 |
|---|---|---|---|---|---|---|
| 259 Hann (= default response) | 160 | V.92 | 22 | 3591 | 1.27 | -0.65, -0.08 |
| 259 Hann + 3 frames | 640 | V.92 | 20 | 736 | 1.29 | -0.67, -0.10 |
| 513 Hann | 320 | V.92 | 38 | 17598 | 2.84 | -0.76, -0.25 |
| 1025 Hann | 640 | falls back to V.34 at Jd | 550-780 | -- | -- | -- |
| 2049 Hann | 1120 | retrains at Ja end (rtd 5656) | -- | -- | -- | -- |
| cutoff 3800/3900/3950 (257) | 160 | falls back to V.34 | -- | -- | -- | -- |

Offline the longer kernels are much closer to ideal (1025: -40 dB, 2049: -55 dB), yet slmodemd
does worse with them: a sharper edge is harder for its 150-symbol equaliser, so more is left
for the noise predictor; at 1025 its TRN1d does not converge. Delay alone changes nothing (259
+ 3 frames). Any extra roll-off at 4 kHz trips `V90SpectralVerifier` (the call that logs "No
special conditions" when it passes) and slmodemd falls back to V.34. (Pad fits here also move
with the Jd state, which the pipeline delay changes, so compare error energy and taps, not fits.)

So the -0.66 is slmodemd's receiver coping with a downstream that is flat right up to a hard
edge at 4 kHz, which is what an ideal digital path delivers. Nothing in our transmitter is
involved.

A SOFTER edge, with the -6 dB point still at 4000 Hz, is the direction that works; the spectral
verifier passes at both lengths:

| downstream kernel | DFE 1 | Phase 3 error energy | ratio, skip 0 | ratio, skip 5 |
|---|---|---|---|---|
| 257/259 Hann (default) | -0.65 | 22 | 1.17-1.27 | 2.87 |
| 129 Hann | -0.39 | 17.7 | 1.186 | 1.476 |
| 65 Hann | -0.24 | 13.0 | 1.461 | 1.491 |

The Jd-state lottery shrinks from 1.17-3.25 to 1.19-1.49, under slmodemd's 1.5 limit at both
states tried, but with thin margins. Open: more Jd states, the sinc upstream, and whether the
first-attempt retrain goes away on full calls; and whether a real line, which rolls off from
about 3.4 kHz, behaves like the soft edge.
