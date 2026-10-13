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
