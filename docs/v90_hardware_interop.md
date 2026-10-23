# V.90 Hardware Interoperability Procedure

## Required bearer

- analogue modem connected to an FXS gateway
- SIP server, gateway, and modem server on the same LAN for the first tests
- PCMU or PCMA with no transcoding
- VAD/CNG, PLC, AGC, noise suppression, and gateway echo cancellation disabled
- raw RTP capture when the gateway or PBX can provide it

## One bounded attempt

```bash
./tools/v90_hardware_interop.py \
  --label "modem-chipset_gateway-model_pcmu" \
  --duration 180 \
  -- \
  --sip-server asterisk.example \
  --username 6001 \
  --password 'secret' \
  --pty-link /tmp/v90modem
```

During the run, place one call from the analogue modem. After connection, send
a deterministic payload through the PTY. Keep the generated directory intact;
`manifest.json` hashes the log and raw G.711 taps, while `summary.json` records
the first failed protocol layer when DATA is not reached.

## Evidence to record

- analogue modem manufacturer, model, firmware, and chipset
- FXS gateway model, firmware, port settings, and PCM law
- PBX version and codec configuration
- negotiated downstream and upstream rates shown by the analogue modem
- whether CPt, CP, MP, MP-prime, E, B1d, and DATA were observed
- first failure point from `summary.json`
- RTP loss, reordering, duplicate packets, and timestamp discontinuities
- downstream and upstream byte/bit error counts from the deterministic payload
- for USRobotics Courier tests, run the post-call diagnostic bundle before
  resetting the analogue modem:
  `./.venv/bin/python tools/cx_at.py --dev /dev/cu.usbserial-21210 usrdiag`
- for a Courier failure that reaches V.90 Phase 3/4 and then retrains, also run
  the verbose hidden-Y diagnostic bundle once:
  `./.venv/bin/python tools/cx_at.py --dev /dev/cu.usbserial-21210 usrdeepdiag`
- when the Courier itself is originating the analogue call, use its hidden Y4
  call-progress dial form for one repro run:
  `./.venv/bin/python tools/cx_at.py --dev /dev/cu.usbserial-21210 usry4dial 6001 --wait 120`

## Progression

1. V.8 and V.90 selection.
2. CRC-valid INFO0a/INFO1a.
3. S/Jd/DIL Phase 3 completion.
4. Strict CPt and TRN2d.
5. Data-mode CP, MP/MP-prime, E, and B1d.
6. Downstream unframed payload transfer.
7. Upstream V.34 payload transfer.
8. Simultaneous full-duplex transfer.
9. Retrain and rate renegotiation.

Do not promote a combination to interoperable until two consecutive fresh
calls reach the same negotiated rate and transfer the deterministic payload
without unexplained errors.

## Results matrix

| Modem/chipset | Gateway | Law | V.8 | Phase 3 | CPt/CP | E/B1d | DATA | Rates | Evidence |
|---|---|---:|---:|---:|---:|---:|---:|---|---|
| _pending_ | _pending_ | - | - | - | - | - | - | - | - |

### Wired RasFinder matrix, 20 October 2026

The outbound digital-role calls on wired vmhost CT 117 are recorded in
[`rasfinder_modulation_tests_20261020.md`](rasfinder_modulation_tests_20261020.md).
Across extensions 6311–6313, three V.90 offers produced one V.34 fallback,
one white-decoding V.90 DATA entry followed by retrain, and one V.90
48000/21600 LAPM login held about 61 s. All three direct V.34 calls reached
24000/LAPM/login; all three V.32bis calls timed out training. V.22bis reached
2400/LAPM/login on two lines; the third completed physical training and
V.42 detection but not LAPM.

This is a 12-call current-state snapshot, not a controlled acquisition A/B,
checked bulk-payload result, or two-fresh-call qualification. The source was
`3d21a8ff` plus archived working-tree changes, **not a clean main build**.
Use the snapshot manifest and binary hash in that note when reproducing it.
The soak script's default extension is still historical 8416: explicitly
set `RF_EXT=6311`, `6312` or `6313` for these routes rather than copying its
default. Keep account, SIP/RTP ports and PTY isolated per concurrent call.

### Conexant CX93001 direct Apple waveform leg, 18 October 2026

USB `0572:1349`, firmware `CX93001-EIS_V0.2013-V92`, originates into our
digital-role software through the remaining physical Apple USB modem on
the powered direct pair. V.8 selects V.90, but received INFO1a selects V.34
(V.90 Tables 10/11, §9.2); no CP′/B1 or V.90 DATA is established. Trials
cover native 9000/9600 Hz and hardware upstream ceilings 9600/33600 bit/s.
The uncapped call retrains from 21600/21600 to 16800/16800 and transfers
4096-byte bodies with exact framing both ways, without LAPM/compression.
There are extra bytes before the payloads; a preceding lower-rate call
has six corrupted downstream bytes. This is not an interoperability pass.
See `apple_usb_modem_sm56.md`, “Foreign USB hardware peer”, and retained
`reference-data/artifacts/apple-cx-direct-20261018/` for diagnostics and grading.

### Conexant CX93001 declines V.90: our Phase 2 sequencing, 21 October 2026

The CX93001 (`AT\N2`, `AT+MS=V90,1,300,33600,300,56000`) dials our
answerer at **2904**, originating as **6314 (AudioCodes port 4)** in the
21–23 October V.90 captures. The CX is attached to the dev Mac as
`/dev/cu.usbmodem246802461`. The older MNP calls used Grandstream line
6005; the current SIP caller identity is verified in each server log.
The answerer runs on vmhost CT 117
(`tools/soak/answer_ec_call.sh ... v90`, `tools/cx_hold_dial.py`). It had
declined V.90 at INFO1a since 23 July. It sent a V.34 INFO1a (Table 11),
which describes our transmit direction as 2400 baud with projected N=1.
That is the reading of a probe it could not measure. A poor downstream
measurement rules out PCM.

**Cause: ours.** On the taps, the CX sends L1 at 7.20-7.36 s and L2 until
7.84 s. We reversed Tone B at 7.62 s and started our L1 at 7.64 s, while
it was still sending L2. §9.2.1.1.6 lets the digital modem reverse only
after "detecting Tone A and the subsequent Tone A phase reversal". Under
§9.2.2.1.6 the CX sends that Tone A, reverses it and goes silent to wait
for our reversal, but our reversal had already passed (Tone A 7.84 s,
reversal 7.89 s). Our L1/L2 had arrived before it was listening.

Why we reversed early: `v34_rx()` advances `rx.sample_time` by each
demodulator's return value. Every demodulator except SILENCE returns 0,
the SpanDSP convention for "consumed the block", so the receive clock
stayed at 0 for the whole call. `tone_ab_hop_time` held only the in-block
offset of the second Tone A reversal. `v90_tone_a_reversal_recovery_due()`
compares it with the transmit clock, so the §9.2.1.2.4 deadline (900 ms +
RTD) read as expired when `SECOND_B` started. The replay logs "recovery
deadline reached at SECOND_B after 2 bauds" (tx_time 9440, rx_time 0,
hop 140). This came in on 23 July (6b30ac32), when the deadline replaced a
fixed 333 ms stage timer that happened to cover the CX's 300 ms. That is
the July regression window: the CX had accepted V.90 on 19 July.

The fix counts the rest of the block in `v34_rx()`. Which samples go to
which demodulator, and the return value, are unchanged. In replay,
`SECOND_B` now waits 193 bauds for the CX's real reversal (fast-tier
fixture `reference-data/artifacts/cx93001-v90-phase2`). On the wire, the
CX's Tone A reversal now comes first, then our reversal, then our L1/L2.
Our L1/L2 itself was always conformant: 25 equal tones, the four
§10.1.3.5 omissions, and L1 6 dB above L2.

**Live, after the fix: the CX selects V.90 on 8 of 11 calls** (0 of 2
earlier today, and none since July):

| Declared max power (INFO0d 33:37) | Calls | V.90 (UINFO) | Declined (first INFO1a, our direction) |
|---|---|---|---|
| default −6 dBm0 | 5 | 3 (95) | 2 (2400 baud N=5; 2400 baud N=0) |
| −7 / −9 / −12 / −13 dBm0 | 1 each | 4 (91 / 85 / 79 / 75) | — |
| −11 dBm0 | 1 | — | 1 (2800 baud N=7) |

So the maximum-power declaration is not the switch. The CX honours it
(UINFO falls as the ceiling falls, §8.2.3.2), and the declines are
scattered. Every decline has a poor first-probe reading. Two of the three
ran Phase 2 with the same stage timing as accepted calls. The third went
through INFO0 recovery. Our RTP transmit timing during L1/L2 is clean on
every call (largest gap 21.7 ms). The retrain Phase 2 of one declined call
read our probe as 3200 baud N=13, but still chose V.34.

**Next layer.** On the −13 dBm0 call we reached V.90 DATA at 46666/31200.
Our T/3 upstream B1 then gave up ("the upstream has no carrier"). The CX
never reported CONNECT and retrained at 67 s.

Also noted, not changed: the round-trip delay estimate is only measured on
the V.92 short Phase 2 path. Every V.90/V.34 Phase 2 deadline uses RTD = 0.
INFO0d bits 29:32 declare −13 dBm0 while the tap shows Phase 2 at about
−10 dBm0 (L2 −10.2, Tone B −9.9). `v90_info0d_nominal_power_code()` is meant
to declare the actual level. Why it does not is not yet investigated.

### Conexant CX93001 upstream: CP framing, B1 location and rate, 21 October 2026

"B1 giving up; the upstream has no carrier" on the CX was three separate
faults. The first two were ours and are fixed. The third is a rate policy.

**1. We discarded every CP frame the CX sent (fixed).** The CX's CP′
frames were on the line, valid, from 20.53 s, one every 181 ms. We
accepted the first one at 23.37 s. On the calls where that took longer
than about 2 s, the CX retrained. To establish this I demodulated the CX's
upstream independently, outside the receiver: blind CMA, 4-point
decisions, GPA descrambler, Table 14 framing. Its frames carry the CRC at
273:288 (remainder 0), then **one** zero fill bit, and the next frame's
sync starts at bit 290. Table 14 asks for three fill bits. Our framer
rejected every such frame on its fill bits (`structure` reject). The two
ones it had read as fill also left the next sync two ones short, so that
frame was missed as well. Fill bits now gate nothing (`vpcm_cp.c`, like
the reserved bits). Ones read as fill go back to the sync counter
(`v90_cp_rx.c`). `vpcm_loopback_test`'s new short-fill case accepts 0 of
6 back-to-back frames on the old code (3 syncs, 3 structure rejects,
exactly the live signature) and 6 of 6 now. Live, the renegotiation's CP
now decodes on every frame (`source=spandsp`, every 60-80 ms). In
replay, CP′ is accepted at 20.88 s instead of 23.16 s.

**2. The 4-point locator trained in the wrong place (fixed).**
`v90_t3_blind_locate()` assumed E is recognised a few dozen symbols after
the 4-point CP′/E ends, and trained on the 600 symbols ending 60 before
E. On the CX, E is recognised about 380 symbols after that, so the window
was mostly B1 and data. It trained to 0.27 and returned −1 without
logging. It now steps back one window per packet, up to 1 s, until a
window trains cleanly. Offline it finds the CX's CP′/E at a 0.0013
decision error (28.8 dB), ending exactly where the template search put
B1.

**3. 31200 is more than this upstream carries (open).** We always offer
the 3200-baud maximum in MP. Nothing measures the upstream before MP.
At 28.8 dB a 31200 constellation (power ≈ 725, about 1000 points) is
noise-limited. The template fits 60% (95% is needed), and the data
decodes white (distance to grid 0.68) at every rate the receiver
assumes. The same line with `ME_V90_UPSTREAM_MAX_BPS=24000`, live
(`cxfix24000-b1`):

- **B1 template fit 100%**, B1-era distance to grid 0.077, data symbol
  error 0.12-0.19.
- V.42 LAPM connected. The CX reported **`CONNECT 50667`**: its first
  V.90 connection.

The cap alone is not the fix. Of three 24000 calls, one is above. One was
declined at INFO1a. On the third (`cxfix24000-b3`) the template again fit
only 60%. The step-back found no 4-point CP′/E either: with CP′ now
accepted early, the T/3 capture was armed only 1280 samples before E, so
the CX's CP′/E was never in the receiver's history. The three 31200 calls
reached DATA twice. Neither acquired B1.

It did not last. The upstream symbol clock is about −127 ppm from ours,
and the decision error crept from 0.12 to 0.34 over 3 s. That set off two
§9.6 renegotiations (to 21600, then 19200). The CX retrained during the
second, and the call fell back to V.34. Holding that timing in data mode
is the next layer after B1.

The CX also retrains exactly 2.0 s after our TRN2d starts if it has not
recognised our MP (`cxfixnone-2/-3`, `cxup-21600`). It recognised MP on 2
of 5 Phase 4 attempts. That gate is downstream, and the B1 work does not
touch it.

**Measured rate, and the data path behind it (22 October 2026).** Five
changes, each checked offline against the recordings and then live:

1. **The MP's upstream offer is measured (`v90_upstream_measure_locked()`).**
   The T/3 capture now starts when CPt is accepted, not at CP′. During our
   500 ms TRN2d the CX sends CPt/SCR/CP, all 4-point (our Jd bit 47 is 0).
   The T/3 equaliser is trained on the newest 600 symbols, and the SNR
   caps the offer. Calibration, replayed with the real echo reference:
   SmartLink 36.9–37.3 dB, RasFinder 35.4–36.4 dB (both carry 31200),
   CX93001 28.1–28.6 dB. The rule: 31200 needs 35.0 dB, and each
   2400 bit/s less needs 2.0 dB less, so the CX is offered 21600 and the
   others are not capped. `ME_V90_UPSTREAM_NO_SNR_CAP=1` turns it off.
   Live at 21600, B1 is acquired at a 99.9% template fit, the upstream
   settles at 0.044, LAPM comes up and lines cross both ways
   (`cxfull-2`). An attempt to use the T/2 receiver's Phase 3 TRN for this
   was discarded: it read 4.6 dB on six unrelated calls.
2. **The timing loop starts at the measured clock offset.** The baud-line
   phase of the T/3 history before B1 gives −44 to −46 ppm on the CX,
   matching what the loop converged to. Before this, the loop took ~2.5 s
   to pull in from zero.
3. **The fractional timing actuator is a windowed sinc, not linear
   interpolation.** At 0.21 of the T/3 rate, linear interpolation passes
   0.79 at a fraction of 0.5 and 1 at a fraction of 0. A −45 ppm offset
   sweeps the fraction through a whole sample every 2.1 s, so the
   equaliser chased a gain change at the band edge. On the CX's 21600
   recording the radial error grew 0.17 → 0.37, with the gain wandering
   +2% then −4%, while the input level held to 0.15 dB. With the 8-tap
   sinc, the gain holds to ±0.2% and the error stays at 0.05.
4. **A repeated, identical B1 window settles at once.** The out-of-sample
   check read 0.00206 against a 0.002 "good" line (passable by a factor
   of two) on all seven windows of `cxfull-2`. Each window was the same
   B1 at the same place. The call spent six retries, 3 s, before
   settling, and each look was a 25–44 ms media callback.
5. **(Step-back, CP framing: see above.)**

**What still breaks the CX upstream, measured.**

- **Our callback stalls move the gateway's playout, and with it our
  echo.** In `cxfull-2` our RTP sends stalled 53–64 ms every 0.54 s from
  23.9 s to 26.6 s. Over the same period the echo on the wire moved in
  5 ms steps: 1882 → 1922 → 2042 → 2082 → 2122 samples. The line echo
  canceller's tracker followed each move, but too late, and the upstream
  broke every ~6.4 s, each time just after a move. Change 4 removes most
  of those stalls. One template fit is still a 25–44 ms callback.
  Holding the tracker (`ME_LEC_NO_TRACK=1`) is worse, because the Phase 3
  fit is already stale by data mode.
- **The CX recognises our MP on about 40% of Phase 4 attempts.** It
  retrains exactly 2.0 s after our TRN2d starts. At 8000T TRN2d, 2 of 3
  attempts got past MP. Too few calls to decide.
- **A false E can fire when the CX never acknowledges.** Accepting its
  plain CP early arms our E watch. If the CX then degrades (it is about
  to retrain), the watch fires and we go to DATA with no B1 to find
  (`cxsnr-3`, `cxsettle-2`).

## Eicon, 8 October 2026: clean LAN results and native V.34 corrections

Using our analogue caller against Eicon endpoints 7910 (PCMU) and 7900
(PCMA), the current-source LAN build passes V.90 checked echo: 10 and 30
201-byte lines respectively, at 56000 downstream / 31200 upstream.
The simultaneous PCMU capture has 4257 upstream RTP packets with no loss
and 536875 continuous exact decoded samples at the card input; no card CRC
errors. Earlier Mac failures show actual packet loss after entering the
cryanfw WireGuard tunnel, verified by captures at both ends.

Native V.34 on the clean LAN exposes two additional defects: exact
Table-18 J(16) was forced to J(4), and the selected 16-point Phase 4
constellation added 10 dB because its energy was not normalized. Corrected
per V.34 10.1.3/10.1.3.3/10.1.3.8/10.1.3.9, native V.34 now passes ten
201-byte echoes in each law at 31200 upstream / 21600 downstream, with
Eicon aborted/CRC 0/0 on both calls. Details and artifact paths are in
`eicon_rtp_causality_20261008.md` and `v34_tx_eicon_audit.md`.

### Menu ZMODEM upload and download

`reference-data/artifacts/eicon-zmodem-20261008/` tests both transfers on one Tower LAN
V.90 PCMU call using the corrected isolated build. Menu option 2/Z receives
65536 binary bytes (including every byte value) with an exact SHA-256 match,
in 20.61 s measured by the card menu, approximately 3179 bytes/s. Option 3/Z
then sends 65536 bytes of the menu's repetitive text payload; the downloaded
file matches the generator exactly by SHA-256 and length, completing in
2.82 s locally. The download payload is compressible under V.42bis, so that
elapsed time is not an uncompressed line-rate benchmark. Both lrzsz commands
exit 0 and the menu reports Transfer complete. Upload test files are
verified and then discarded by the menu's existing implementation.

`tools/eicon_zmodem_test.py` repeats the checks with exclusive binary-safe
PTY ownership by sz/rz, preserving the ZMODEM headers after textual prompts.
Run it inside the LAN modem build; it uses the isolated 5078/14600 ports.

## Phase 2 transmit contracts, 22 October 2026

The transmitter corrections address three independent wire defects:

- **INFO0d nominal power (V.90 Table 7, bits 29:32).** The declaration was
  fixed at -13 dBm0 even when the selected Phase 2 level was -10 dBm0.
  It now follows the existing transmit gain calibration, rounded and bounded
  to Table 7's -6..-21 dBm0 range. This changes signalling, not waveform gain.
- **INFO0d final fill (Table 7, bits 58:61).** The stage initializer could
  reset the differential point before the last fill bit was emitted. Save
  that baud's point before changing stage; contiguous recovery repeats retain
  its differential history. No sample or baud is added.
- **INFO1d repetition (8.2.3.1 and Table 9).** The generic INFO1c path added
  eight bits and rounded to a byte, making each digital message 120 bits
  instead of 109. Digital INFO1d now sends four contiguous 109-bit messages,
  preceded by one arbitrary-phase point for the group. V.92 Table 17 uses
  the same message length. Ordinary V.34 framing is unchanged.

`v34_phase4_16pt_test` checks the production generators' differential output:
CRC, final fill, exact frame boundaries and recovery repeats, in both PCM
laws. Power checks select -6, -10, -14 and -21 dBm0 and independently measure
Tone B and L2 after G.711 encode/decode; each is within 0.25 dB of the
declaration. These checks fail against the pre-change transmitter: stale
power, final fill and INFO1d length each have an independent failing check.

The review also found undefined signed left shifts in the Q7 precoder
projection (V.34 9.6.2/Table 11). Multiplication by 128 retains the same
scale for negative constellation coordinates. The complete generator test
passes UBSan after that correction; its directly included transmitter is
instrumented, while the linked SpanDSP archive is not.

Build verification exposed a separate stale-object trap: `make clean`
deleted all dependency files but omitted some test objects, including
`v34_hdx_test.o`. The old half-duplex test produced 102 failures; compiling
the same source afresh restored the nonlinear-primary case with zero payload
errors. Clean now removes all root and tools object files before the
existing explicit cleanup. Validation used a clean application build and a
clean vendored SpanDSP build.

Final validation: `make -j6` builds the application and tests; the runner
with `-j 6 tests/fast.list tests/slow.list` completes **615 passed, 0 failed**
in 223 seconds after integrating the newer upstream receiver corrections.
Logs: `test-logs/dsp-review-integrated/`. The equivalent cleanup fix already
present upstream is retained. The two multiline SIP
tests require localhost socket permission, which the initial sandboxed run
lacked. The K56flex RX rollback and V.FC/USB bridge work already pending in
this working tree are separate from these transmitter corrections.

These are transmitter conformance results, not hardware qualification or
evidence that any particular peer's training failure is fixed. A new live
comparison must check our transmitted tap and complete checked payload
traffic before claiming an interoperability improvement.

## CX93001 Ja recognised by template (23 October 2026)

**Symptom.** Six of six CX93001 V.90 calls on CT 117 selected V.90, then the CX
retrained 3.4 s after Phase 3 began and finally declined at INFO1a; the same
build had reached data mode on 21-22 October. A build of 191312a0 behaved the
same, so the line, not a code change, had moved.

**Cause.** V.90 9.3.2.4: the analogue modem sends TRN then Ja and retrains if no
Sd-to-Sd-bar transition arrives within 1500 ms of the start of Ja. The CX's
TRN lasts ~2 s and its Ja descriptor is the same in every call (N=119 LSP=124
LTP=126, identical across `cxfull-2` and `cxsettle-2`). On the failing calls the
Phase 3 equaliser had been conditioned on a late PP start (`onset 92 ago`, PP
residual 0.62 against 0.26, TRN lock 63% against 98%), so the Ja bits carried
too many errors for any frame's CRC to pass -- yet the frame IS captured: on
hypothesis 8 it agrees with the known frame at 77% over 1800 bits (noise: 50%).
We waited for a CRC-valid descriptor, the CX got no Sd, and it retrained. The
500 ms bound of 9.3.1.3 never started because that clock keys on a heuristic
Ja candidate, and the CX has no TRN-to-Ja gap to offer one.

**Fix.** `v90_ja_template_scan()` compares each hypothesis' newest Ja bits with
a 384-bit stretch of the CX's frame body; 70% agreement is 8 sigma from
chance, and 9.3.1.3 makes Sd follow the receipt of Ja, not a valid CRC. On a
match the engine answers with the CX's own descriptor (`v90_dil_load_cx93001`).
`ME_V90_JA_TEMPLATE=0` disables it. `v34_v90_copy_phase3_ja_bits_from()` copies
only the bits that arrived since the last look.

**Result.** Offline, the failing tap now recognises Ja at 281/384 (fixture
`reference-data/artifacts/cx93001-ja-crc-fails-20261023`, in `fast.list`).
Live, 3 of 4 calls recognised Ja and sent Sd/TRN1d/Jd; one completed Phase 3 and
reached Ri and Phase 4 CP before the CX retrained (the known MP-recognition
gap), and one lost the CX's S after Jd. Arming Phase 3 tracking after the TRN
refinement was tried and changed the Ja agreement by 0.8 points; not kept.

**Jd S-wait widened (same day).** With Ja recognised, the CX's S after Jd arrived
21324-32364 Jd symbols in (2 of 5 calls) -- it uses 9.3.2.7's whole 5000 ms,
counted from its own detection of Sd, and we then need the S's 128T and the line
watch. The resync budget (5100 ms - TRN1d + 4000 symbols = 24796) had expired
on calls where the S was found a few frames later. `v90_jd_s_wait_symbols()` now
allows 12000 symbols of round trip and detection (`ME_V90_JD_RTD_ALLOWANCE`).
Live (CT 117, 5 calls): 2 reached CPt/TRN2d/MP, and one completed the whole
startup -- CX recognised MP (data-mode CP drn=15), MP' and E exchanged, "DATA
upstream=12000 downstream=46666" -- then failed at T/3 4-point training
(decision error 0.0134) and B1 acquisition (fit 56-63%). The 4-point SNR before
MP read 19.4 dB that day against 28.5 on 22 October, so the offer fell to
12000 bps. The other 3 calls never got an S (the CX retrained during Jd).

**T/3 4-point training gate (23 October, not yet live-validated).** The CX's
4-point CP'/E signal measured 19.4 dB on the one call that reached data mode;
`v90_t3_blind_acquire()` trained it to a decision error of 0.0134 and rejected it
against a fixed 0.01 (-20 dB) -- below the line's own noise floor (error ~ 1/SNR).
The gate is now 0.03; an unconverged filter reads 0.05 or more, which
`v90_t3_blind_locate()` already rejects. The following 8-call run never reached
this stage (Ja recognised on 5/5 but the CX sent no S after Jd), so the effect is
unmeasured. S arrival is not explained by our TX stalls (checked against
rtp-tx.csv gaps after Sd on 15 calls).

**Why the CX sometimes never sends S after Jd (23 October, measured).** The CX
goes silent 0.60 s after our Sd starts and resumes *exactly* 4.10 s later on every
one of 16 calls (rx tap, `lag.py`): then it sends S if it has decoded Jd, or its
Tone A retrain if not. That instant does not move with TRN1d length (8004T and
20004T alike), RTP jitter (no correlation with >25 ms TX gaps, no seq/ts
discontinuities), or the pre-Sd wait. The CX's own `AT&V1` after a failed call shows
Rx LEVEL 215 / Line QUALITY 255 / EQM 0 (never measured) and Highest Rx State 62, and
on the one success level 10, quality 64, state 67, EQM 00E5: on failures it never
completes receive-side training on TRN1d/Jd, so the channel it hears is the
suspect, not our Jd timing. A 500 ms pre-Sd wait (`ME_V90_SD_DELAY_MS`, new,
default unchanged) completed Phase 3 on 3/6 calls against 2/6 for 0 ms -- no
evidence either way. Success over the day ran 2/5, 3/5, 0/8, 1/6, 1/10 on the same
build: the rig itself drifts.

## False E from an unqualified streaming decoder (23 October 2026)

A new wired CX93001 call reproduced the previously noted false-E path.
With `ME_V90_TRN2D_SYMBOLS=8000` (7998 transmitted symbols after frame
alignment), `cx-work-20261023-trn8k1` accepted CP from the batch decoder,
then reported upstream E at TRACE +21143 ms and entered DATA. Its streaming
framer had accepted **zero** frames (`cp_valid=0`). Independent DQPSK/GPA
recovery of the RX tap finds CRC-valid **unacknowledged CP** continuously
from RX-file 20.740313 s through 23.868750 s: no E/B1 transition occurred
there. B1 searches on that continuing CP scored 17–20% and stalled media
callbacks for 21–48 ms. These were searches for a B1 the peer had not sent.

The engine now qualifies its streaming E watch only after that same stream
has delivered an accepted CRC-valid CPt/CP. Rejecting a frame and changing
its hypothesis revokes that qualification and clears the ones counter;
new calls/retrains reset both. A batch CP still prepares the upstream rate,
but cannot validate another demodulator's bits. V.90 9.4.1.4 explicitly
allows E instead of a decoded CP prime, so the qualified E alternative is
retained. Sections 9.4.1.6 and 9.4.2.4–.5 require actual E before B1.

When the batch decoder recovers CP prime and the streaming path remains
unqualified, the existing independently validated B1 search is now also
available on a G.711 frontend. It reports no E and releases no bits before
its full B1/out-of-sample checks pass. This preserves A-law engine-pair
payload delivery when the live CP hypothesis loses lock; requiring a live
CRC without that recovery regressed two existing A-law rows.

Regression: paired first-24-second taps in
`reference-data/artifacts/cx93001-unqualified-e-20261023/`, now in
`tests/fast.list`. Pre-change synchronous echo-reference replay falsely
reports E at +22100 ms. Corrected replay accepts batch CP, then streaming
CP, with no E/DATA in the prefix. This is a negative handover regression;
it cannot qualify subsequent peer behavior after our first divergent TX.

**Our own TX checked first.** On the preceding default-TRN2d baseline,
independent spec decoding recovers all 11322 TRN2d bits as ones and validates
336/336 MP CRCs. On the failing 8000T call, the independent waveform
generator reproduces **all 7998 magnitudes and all 7998 signs exactly**
from the peer's CPt (Ucodes 97/85/42/41, Sr=1, ld=1, a1=1, other
coefficients zero). That clears the generated sequence, not the waveform
the CX ultimately hears through its analogue path.

**Live candidate limits.** Two candidate calls used the same 8000T setting.
The first retrained 2 s after TRN2d, before data CP; the second recovered
repeated CRC-valid CP on the streaming path, remained in MP/MP prime with
no false E/DATA, and followed the peer's retrain at +29204 ms. Both fell
back to V.34/LAPM. Neither establishes sustained V.90 payload or a startup
pass-rate improvement. The unresolved gate is still the CX recognizing our
MP and returning CP prime/E. Do not change the default TRN2d duration on
this evidence.

Baseline executable SHA-256:
`c45298f20df66df783a7e8c3bd2bd200d14d4a5c8195aa857d38e6b6bf5dd627`;
candidate:
`19218ec1bc18939758864a4902fa61a478b4d08e73d023bb41229d0275dac65c`.
Both use CT 117 `/root/mnp-live`, with the pre-existing deferred line-echo
fit and V.34 working-tree changes. The candidate changes only the engine's
E qualification and CP-prime recovery from that baseline.

Validation: full fast/slow run **620 passed**, with only the two localhost
SIP tests denied socket access by the sandbox; both pass when rerun with
localhost access (**622 tests passing across the runs**). Logs:
`test-logs/cx-e-qualified/` and `test-logs/cx-e-qualified-sockets/`.

Retained full call evidence, independent decode output, source diff, test
summaries and SHA-256 manifest: `reference-data/artifacts/cx-work-20261023/`.
The completed baseline and both candidate fallback calls each delivered
all 20 numbered lines exactly once, in order, in both directions; these
are V.34 payload results. The false-E V.90 call delivered none.

## RTP path checked at both PBX legs (23 October 2026)

The earlier route correction to Grandstream/6005 was itself wrong: it used
an older MNP note. `cxfull-2`, `cxsettle-2` and all the new V.90 server logs
identify the current caller as **6314, AudioCodes 1.4**. The CX dials **2904**.
The MP-114 registers at **10.69.70.221**; the PBX is **192.168.88.122** and
CT 117 is **192.168.88.230**. The host also has a direct wired VLAN-70
address, **10.69.70.16**. No network/gateway/PBX configuration changed.

Two further calls used the same candidate binary and 8000T TRN2d setting.
Call 1 failed at MP and fell back to V.34/LAPM. Call 2 reached two V.90
Phase-4 attempts; the first retrained ~2 s after TRN2d, the second kept
sending unacknowledged CP while we stayed in MP/MP prime, then retrained.
It never connected. Neither call qualifies V.90 payload.

**Engine to wire, call 1.** Host Ethernet capture contains 4711 TX packets,
753760 payload codewords, byte-identical to the complete TX tap, with no
sequence/timestamp gaps. The reverse 4709-packet wire stream matches the
RX tap over 752000 aligned codewords with zero mismatches, at a constant
1760-sample tap offset. That offset includes jitter-buffer/startup delay;
it is not a physical propagation-delay measurement. Host TX's largest
packet interval was 54.36 ms; RX's was 28.41 ms.

**Both PBX legs, call 2.** All 2599 downstream packets, 415840 codewords,
pass from CT 117 through the PBX toward the gateway in identical order,
with identical SSRC/sequence/timestamp and payload. The gateway-egress
payload is also byte-identical to our entire engine TX tap. All 2597
upstream packets, 415520 codewords, are identical across the reverse PBX
bridge. There are no captured sequence/timestamp gaps. This clears
transcoding, byte replacement and packet omission at the captured PBX
relay; it does not prove delivery to the gateway or its D/A playout.

**Gateway RTCP reports a downstream concern.** Its cumulative-loss counter
is 0 near the first packet, 2 at +4.79 s, 4 at +10.31 s, 5 at +25.77 s,
6 at +33.11 s, and 7 at +37.69 s (relative to the first RTP packet in the
PBX capture), remaining 7 through +48.37 s. The two senders' other reception
reports show no loss. This puts the reported loss beyond the captured PBX
egress. RTCP alone cannot distinguish network loss from gateway receiver
behavior, locate each missing packet, or prove it caused MP rejection.
The PBX-egress largest TX gap was 46.02 ms; the second MP interval includes
that gap. Captures reported zero kernel drops.

Reproduce the packet-identity, payload and RTCP checks with:

```sh
python3 tools/rtp_path_check.py \
  reference-data/artifacts/cx-wire-20261023/call2/pbx.pcap \
  --source 192.168.88.230 --relay 192.168.88.122 --peer 10.69.70.221 \
  --tx-tap reference-data/artifacts/cx-wire-20261023/call2/live-tx.g711
```

The diagnostic requires a relay that preserves RTP identity. It reports
changed bytes, missing/added packets and reordered/duplicate identities;
an empty capture is never a positive transparency result. Its fast-tier
negative controls include a changed mu-law negative-zero codeword, loss,
reordering, duplication, sequence wrap, a changed timestamp, RTP extension
and padding, capture truncation, compound RTCP and signed cumulative loss.
They use the RFC 3550 6.4.1/.2 reception-block layout, cross-checked against
the vendored `pjmedia/rtcp.h`.

**Direct wired isolation was blocked before training.** The identical
binary runs on vmhost with CT 117's existing loader/libraries, bound to
10.69.70.16, and can directly INVITE the gateway without registering or
changing infrastructure. Both CX auto-answer and explicit first-RING ATA
attempts fail to complete SIP answer setup. The second capture proves why:
the gateway sends final `200 OK` and BYE to **192.168.88.122:5060**, despite
Via/rport and Contact naming **10.69.70.16:5098**. The PBX answers BYE with
`481 Call/Transaction Does Not Exist`. The direct caller sees only ringing
and one-way early RTP. These calls cannot compare V.90 audio or packet-loss
performance. Both ended on-hook, CX S0 restored to 0, all test servers and
capture processes stopped.

Raw pcaps, matched taps, transcripts, reproduction scripts, JSON reports
and hash manifest: `reference-data/artifacts/cx-wire-20261023/`. This is
retained local evidence, not an added hardware qualification. The remaining
software timing concern is measurable Ja-search work: call 1 logged three
21–22 ms RX callbacks, of which 14.7–16.1 ms was the DIL/Ja search. Reducing
that work without changing which template windows match is a concrete next
step; it does not by itself resolve or explain the RTCP loss reports.

## Ja template comparison cost reduced without changing matches (23 October 2026)

The byte-per-bit template loop dominated an instrumented replay of the
21–22 ms live callbacks noted above. Replace eight scalar comparisons
with an unaligned-safe `memcpy`/XOR/popcount over eight 0/1 bytes. The
capture API and template both produce only 0/1, so each differing bit
contributes exactly one set bit. Byte order is immaterial. Rejected windows
may stop up to seven comparisons later, but accepted mismatch counts and
which first window is accepted are unchanged. The 384-bit window,
115-error budget, hypothesis order, scan cursor and V.90 9.3.1.3 timing
policy are unchanged; no signal constants or sample accounting change.

Instrumented offline replay of `cx-wire-20261023/call1`, 213 template calls
and 530 surrounding DIL/Ja search calls on each arm:

| Host | Template total before / after | Whole search total before / after |
|---|---|---|
| Mac ARM64 | 177.552 / 9.809 ms (18.1x) | 209.529 / 39.882 ms |
| Wired Linux x86-64 | 215.813 / 17.898 ms (12.1x) | 252.811 / 54.796 ms |

On Linux the largest template invocation was 2.600 / 0.221 ms; the largest
whole-search invocation was 3.283 / 1.301 ms. These are one controlled
replay per host/arm, not a live real-time guarantee. Temporary scope timers
were identical on both arms, accumulated counts/total/max without per-call
printing, and are absent from the delivered binary. The Linux candidate
was rebuilt after profiling and its SHA-256 matches the uninstrumented
candidate saved before profiling.

Three independent engine replay inputs (noisy Ja, false-E regression,
second wire-captured call) retain the same hypothesis, score, capture-bit
position and sample-clock recognition time. Their complete generated TX
taps are byte-identical over **94880 + 192000 + 415840 = 702720** codewords.
This checks implementation equivalence on fixed inputs; it does not claim
hardware behavior after a divergent transmission.

`v90_ja_match_test` exercises every error count at all eight alignments,
including the 115/116 boundary and non-word-aligned lengths. The Linux
ASan/UBSan build passes. Mac ASan stalled in shadow-memory initialization
before main, so no Mac ASan result is claimed. Full fast/slow validation:
**622 passed**, with only the two localhost socket rows denied by the
sandbox; both pass on their required-access rerun (**624 rows passing
across the runs**). Logs: `test-logs/cx-ja-cost/` and
`test-logs/cx-ja-cost-sockets/`.

Four live calls were alternated **A1/B1/A2/B2**, baseline/candidate, same
8000T TRN2d, `AT\N2`, `ATS7=60`, V.90 +MS bounds and a 15-second payload
hold after CONNECT. All calls recognized Ja. Neither arm had a
Ja-attributed RX callback over 20 ms in this batch, so it cannot establish
a live stall-rate or acquisition pass-rate improvement. None qualified
sustained V.90 data:

- A1: V.90 MP retrains, then V.34/LAPM at 4800.
- B1: valid batch CP prime, V.90 50666/21600 DATA announcement, but upstream
  B1 never acquires; the CX never CONNECTs. Its CP-prime search explicitly
  disables the four-point/blind fallback. The best template fits 61% and
  repeated searches make 21–56 ms callbacks. Independent raw demodulation
  confirms CP ends near RX-file 20.59 s; later symbols cease being 4-point.
  This is an acquisition lead, not proof that enabling blind fallback will
  recover data. Our own DTE later prints `CONNECT 50666/V90` after the
  error-control timeout falls back to V.14, despite the missing upstream
  lock. That is a separate CONNECT-gating defect to fix.
- A2: a streaming CP validates; E is accepted at +30900 ms, just 60 ms
  before the peer-retrain event. No V.90 link. This still needs a check
  against the raw retrain tone before treating the E as real.
- B2: V.90 retrains, then V.34/MNP at 21600; the short payload hold is not
  a sustained-link qualification.

Baseline Linux binary SHA-256:
`19218ec1bc18939758864a4902fa61a478b4d08e73d023bb41229d0275dac65c`;
candidate:
`ac4346df6274a4a2825518c9338f92d60a576e3def4457a2426687931d7396e4`.
Both retain the pre-existing deferred echo-fit/V.34 working-tree changes;
the comparison changes only this matcher. All live test processes stopped.
Evidence, profile sources/logs, replay equivalence, matched taps and manifest:
`reference-data/artifacts/cx-ja-cost-20261023/`.

## DTE CONNECT gated on upstream B1 (23 October 2026)

`cx-ja-B1` exposes an actual DTE defect, not only an optimistic engine
log: after seven failed B1 windows, the data stack's optional +ES fallback
reports `DS_LINK_FALLBACK` and the DTE receives **CONNECT 50666**, despite
no upstream carrier. The fallback callback previously checked only
whether a CONNECT had already been reported. V.90 9.4.1.6 requires B1
reception before turning on circuit 109; optional error control does not
relax that requirement.

Native V.90 CONNECT paths now require acquired, non-lost upstream carrier
and completed downstream startup. The data stack remains running so that
acquisition can deliver ODP/LAPM/MNP normally. If framing/link readiness
arrives first, the RX callback reports the deferred CONNECT when upstream
acquisition succeeds. This changes the DTE result, not the sample stream,
mapper or error-control negotiation. The analogue role, V.92's existing
PCM-upstream lock gate and established-call retrain reporting are retained.

A paired 45-second prefix is tracked as
`reference-data/artifacts/cx93001-connect-without-b1-20261023/`.
`tools/replay_dte_check.py` reads the real PTY under default V1/Q0 settings;
it does not mistake an engine DATA announcement for a CONNECT. Unmodified
replay emits one `CONNECT 50666`; the corrected replay emits none, with
B1-give-up, +ES fallback and CONNECT-deferral log evidence all present.
Both mu-law and A-law V.14 engine-pair rows now require the initial
CONNECT-deferral log and still deliver checked payload in both directions,
so a deferred connection is not simply suppressed forever.

Validation: all 42 focused V.90/V.92 engine-pair rows pass. The full
fast/slow run passes 623 rows; the only two initial failures are sandbox
localhost socket denials, and both pass with socket access (**625 rows
passing across the runs**). Logs: `test-logs/cx-connect-gate/` and
`test-logs/cx-connect-gate-sockets/`. Before/after PTY replay logs and probe
evidence are retained in `reference-data/artifacts/cx-connect-gate-20261023/`.
No new live acquisition success is claimed.

The same capture's standalone upstream probe can set the acquisition flag
via the E/4-point route where CP-prime search fails, but that probe has no
live echo cancellation and no decoded-FCS assertion. It is evidence for
investigating the CP-prime/blind policy, not permission to enable a weaker
acquisition path or claim working payload. That remains open.

The forced-E probe was subsequently checked at its decoded-bit output:
`V90_REPLAY_QUIET=1 V90_REPLAY_BITS_OUT=/tmp/cx-e-probe.bits
./v90_upstream_replay
reference-data/artifacts/cx93001-connect-without-b1-20261023/live-rx.g711
ulaw 3200 21600 20.8 2.95`. It publishes 526,716 bits and sets acquired,
but `tools/hdlc_bit_check.py` finds **zero valid complete frames**, 1,008 bad
complete frames and 1,051 aborted segments. This is a negative diagnostic,
not a validated alternative to the CP′ search: forced E and absent echo
cancellation both limit the experiment. Do not enable blind acquisition on
the strength of its acquired flag. The replay's optional ASCII bit output
allows independent CRC checking; `V90_REPLAY_QUIET` suppresses forced-mode
FLOW logging without changing the receiver.

### Nonzero B1 trellis-state probe (23 October 2026)

The same 45-second capture was tested with temporary offline builds, leaving
production sources and binaries unchanged. Sweeping 9600..31200 in 2400-bit/s
steps at 3000 and 3200 baud finds the strongest raw-template fit only at the
negotiated 3200/21600 (58.4%; other supported combinations at most 30.3%).
That does not support changing the negotiated rate to rescue acquisition.

Sweeping initial encoder states 0..15 instead finds **97.7%** at state 4,
GPA/16-state trellis, against 58.4% for the default zero state. The raw
standalone probe still rejects its out-of-sample lattice distance (0.736).
With the whole engine and recorded TX echo reference, state 4 fits **99.9%**,
passes the existing validation after a repeated window (distance 0.227),
and acquires B1. This is evidence that the zero-state template misses this
captured B1; it does not establish why the peer differs from the reset in
V.34 10.1.3.1. The plain V.34 receiver already searches nonzero states for
a different captured peer; the V.90 T/3 path currently does not.

The acquired engine emits clean all-ones windows at 0.1 and 0.3 seconds,
then deteriorates by 0.5 seconds. Its decoded bits have **zero valid HDLC
frames**, 680 bad complete frames and 752 aborted segments. Two independent
negative controls do not recover frames: freezing echo-delay tracking gives
0/1016/987 (valid/bad/aborted), and disabling V0 phase acquisition gives
0/684/747 with the same early deterioration. Neither control is a proposed
production change. Constant ones alone remain insufficient payload proof;
replay cannot validate the peer's reaction to divergent transmission.

Temporary sources, sweep logs, compressed decoded bits and checksums are
retained under `reference-data/artifacts/cx-b1-state-20261023/`. The next
acquisition change should search states with bounded work, preserve the
complete-frame and out-of-sample gates, and then be graded on live framed
payload rather than its acquisition flag. No fixed state-4 exception is
justified, and sustained data reception remains unresolved.

### Rejected B1 windows preserve prepared parameters (23 October 2026)

A bounded state-refinement candidate exposed a separate state-management
bug: loading a window's best template wrote its trellis and scrambler into
the durable receiver settings **before** held-out validation. A rejected
noise window could therefore replace the prepared 16-state code with the
64-state code for later searches. Those assignments now occur only at the
validated acquisition commit point (V.34 10.1.3.1); fitting and validation
thresholds are unchanged. The standalone replay checks that an unacquired
run preserves both prepared parameters. Against the paired 45-second CX
fixture that assertion fails with the previous receiver and passes with
the corrected receiver; the negative capture is now a quick-tier row.

The general state-refinement candidate is **not shipped**. It finds state 4
at 99.9% without a special-case override, but the actual-PTY regression
then reports CONNECT followed by garbage. Acquisition alone is therefore
insufficient to accept this candidate. Its source, decoded bits and PTY
log are retained as `candidate-*` in the same local evidence directory.
The original no-CONNECT regression remains required. This is a controlled
negative result, not successful hardware or sustained-data qualification.

Validation: clean rebuild; full existing suite 623 passing, with only two
sandbox socket-bind failures; both socket rows pass with access. The ten
focused replay rows, including the newly added negative row, also pass:
**626 distinct test rows passing across the runs**. Logs are under
`test-logs/cx-rejected-hypothesis{,-focused,-sockets}/`. The regression's
previous-receiver negative control exits 1 with `rejected B1 changed
prepared trellis or measured scrambler`.

### Tracking controls on the acquired capture (23 October 2026)

Isolated builds based on `5f8b263e`, with the captured state-4 template
override only, reproduce the early deterioration. Disabling each loop
separately still gives zero valid HDLC frames: default 680 bad/752 aborted,
no equalizer DD adaptation 778/814, no timing recovery 971/1011, no carrier
tracking 37/52. The smaller last counts do not indicate success; that arm
publishes fewer bits. No diagnostic override is shipped.

For the default run, mean squared lattice distance over consecutive
320-symbol windows is 0.051, 0.046, 0.132, 0.220, 0.680. With equalizer
adaptation disabled it is 0.054, 0.049, 0.138, 0.225, 0.676: adaptation is
not the cause of this early loss. Local timing-offset sweeps at 1600 and
3200 data symbols do not recover a clean lattice (best 0.600 and 0.618).
Later default-run windows return to small lattice distances but still
decode unstructured bits; a clean lattice alone is not frame recovery.

Generated TX taps pin the replay's limits: production and the state-4 probe
are byte-identical until sample 310240 (**38.78 seconds**), well after the
early deterioration. Both first differ from the recorded live TX at sample
42720 (**5.34 seconds**). Thus these are controlled receiver comparisons on
a fixed recording, not evidence of how the hardware would respond to the
candidate. Retained sources, logs, symbol and bit dumps, TX taps and
checksums: `reference-data/artifacts/cx-tracking-20261023/`. A new live
exchange is still needed to qualify any acquisition change.

### Wired rig prepared; CX93001 disconnected (23 October 2026)

CT117's idle `/root/mnp-live` build now includes the CONNECT gate and
rejected-window parameter fix. Its existing private-header/line-EC worktree
matches this Mac's pre-existing worktree; the two updated source hashes
were verified after transfer. Original sources and executable were saved
under `/root/cx-current-20261023-56624aea/`. The new executable and saved
`sip.current` have SHA-256
`da34f7b5c83dc8d61affe21b702133c478a088f70297510e1ad7ccaf5b84c0e5`.
The paired 45-second actual-PTY regression passes on Linux with zero
CONNECT results, B1 give-up and CONNECT-deferral evidence. No live server
is left running. Build and replay logs are retained under
`reference-data/artifacts/cx-rig-ready-20261023/`.

A new physical call could not start: `/dev/cu.usbmodem246802461` is absent
and pyserial's port inventory has no CX93001. The unrelated FTDI serial
port was not opened. Reconnecting the CX93001 is required for that test;
this preparation is not hardware qualification.

`tools/cx_hold_dial.py` also now cleans up failed and interrupted dial
attempts, rather than exiting before ATH and AT&V1. Guarded escape is
followed by a command terminator (a failed dial may already be in command
mode), hangup and the last-call report; serial and log handles are closed.
The quick-tier helper test exercises NO CARRIER, BUSY, NO DIALTONE,
timeout, interruption and normal CONNECT without touching hardware.
Payload grading is still required separately; a successful dial exit code
does not qualify sustained data.
