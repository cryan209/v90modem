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
answerer as **6314 (AudioCodes port 4)**. The answerer runs on vmhost CT 117
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
