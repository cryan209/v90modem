# D-Modem's interop peers

D-Modem (github.com/D-Modem/D-Modem; d-modem.github.io/D-Modem/interop-details/)
tests its own modem offline against other vendors' modem code: SmartLink, Agere,
SM56, PCTel, HSF, Broadcom, ESS and Intel 537EP soft modems, each run on
emulated hardware (ICH/AC'97, SmartDAA, BCM4212, ES2838, 537EP, and a small
Windows kernel for the WDM drivers), plus a whole emulated USRobotics Courier.
`half_v90modem` puts our engine among those peers, so these are the first
offline calls of ours against HSF, ESS, Agere, SM56, PCTel, Broadcom and Intel
code. Like every offline test, they say nothing about the same chips on real
hardware, but they are the vendors' own DSP code.

## How it fits

Each D-Modem peer is a program (`half_<name>`, `dsp/test/modem_half.hpp`) that
reads its line's audio from fd 3, writes its own to fd 4 (16-bit samples) and
prints `created <name>: <pty>`. Its DTE is on that pty. `interop_call` runs two
of them with `netsim` (D-Modem's G.711 network) in between and acts as both
DTEs: it dials, answers, then passes data both ways and checks it.

`half_v90modem` (`half_v90modem.c`, `make half_v90modem`) is the engine as such
a program. It is the digital modem, an 8000 Hz end, and `netsim` passes it the
decoded codewords unchanged. Converting codeword to linear and back is exact in
both directions: D-Modem's `g711.h` and SpanDSP use the same ±32124 μ-law and
±32256 A-law scales. So the engine sees the same octets an RTP payload would
carry. The exception is μ-law's −0 (0x7f), which reaches `netsim` as +0 (0xff);
both are 0 V at the far D/A. The call starts when the DTE sends ATD or ATA
(`me_on_sip_connected()`, as a SIP connect does) and ends when the engine hangs
up. Samples are lockstep, n written for every n read, and the peer owns the
clock. The law comes from `NET`'s third field (`V90MODEM_LAW` overrides it), and
`V90MODEM_VERBOSE=1` turns on the engine's `[ME]` log.

## Running it

The peers wrap i386 vendor objects, so the interop programs build only for
32-bit x86 (`-m32`), on Linux with `g++-multilib`, not on the arm64 Mac. On
CT 128 (`v90modem-buildtest`, vmhost) the build is:

```bash
git clone --depth 1 https://github.com/D-Modem/D-Modem.git && cd D-Modem
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4 --target interop_call netsim half_smartlink half_hsf half_ess half_agere half_sm56 half_pctel half_broadcom half_intel half_courier
```

The configure step lists which vendor objects it found (all of them, as of
D-Modem eceac3b). Then `make half_v90modem` in our tree, and a call is:

```bash
cd D-Modem/build/dsp/test
NET=9600:8000:ulaw SMARTLINK_V92FULL=1 STREAM=10 PROTO_B="ATX3;AT+MS=V92;AT+PIG=0;AT+PQC=0" LOG_DIR=/tmp/c1 NETSIM=$PWD/netsim ./interop_call ./half_smartlink /path/to/half_v90modem "ATX3;AT+MS=92"
```

The peer is modemA, the caller. We are modemB, the answerer, with our own AT
commands in `PROTO_B`. `tools/dmodem_matrix.sh <outdir> [row regex]` runs the
rows below three at a time. Each row uses the line settings of D-Modem's own row
for that peer (`dsp/test/interop_test.cpp`): `NET=9600:8000` for SmartLink and
Intel, and `NET_LOOP=0.45 NET=8000:8000` for the rest.

Our side needs these settings to do V.92 PCM upstream:
- `AT+MS=V92`. Our parser rejects `AT+MS=92`.
- `AT+PIG=0`.
- `AT+PQC=0`. Without Quick Connect, SmartLink reports `V92 capabilities:
  local=0, remote=1, selected=90` and trains V.90.
- `SMARTLINK_V92FULL=1` on the SmartLink side, which sets the full-V.92 bit in
  the library's runtime.

The failing rows and a SmartLink control gave the same verdicts unpaced (the
default) and with `MODEM_REALTIME=1`; the passing rows were run paced.

## Results (2026-11-24, a6737e79 + half_v90modem, D-Modem eceac3b; ESS V.90 updated after the Jd S gate fix)

STREAM=10, one call per row:

| Peer | Row | Result |
|---|---|---|
| SmartLink | V.92 PCM up, μ-law | Pass. 56000 down / 34666 up. Precoded CPd (p1 over 8 taps up to 0.57, the 9600 Hz codec filter's ISI); B1u locked by decoding, residual 0.020 LU |
| SmartLink | V.92 PCM up, A-law | Pass. 56000 / 34666, precoded, B1u locked |
| SmartLink | V.90 | Pass. 56000 / 31200 |
| PCTel | V.90 | Pass. 52000 / 31200 |
| SM56 | V.90 | Pass. 46666-50666 / 31200, LAPM + V.42bis. 10 of 11 over NET_DELAY 1-11 ms, with or without the L2 hold (below); the one failing delay differs per build and retrains after B1d, so a single matrix run can show it |
| Intel 537EP | V.92 offer | Pass. It has no CPu, so V.90 at 52000 / 31200 (as D-Modem says) |
| Intel 537EP | V.90 | Pass. 52000 / 31200 |
| ESS | V.92 PCM up, μ-law | Pass, 8 of 8, 37333 up, B1u locked first time since the Phase 3 retime (`docs/v92_pcm_upstream_findings.md`, "ESS μ-law's first B1u miss"). Before: 29333-30666 on the retrain's unprecoded CPd, then 37333 with the first B1u missing in 2 of 3. In about 6 of 40 calls ESS misses our first CPd and sends Tone A (0 of 48 against `half_open8`). The retrain usually recovers it; one call took two retrains and then a plain V.34 CONNECT at a negative SNR (`docs/v92_pcm_upstream_findings.md`, "The 90 s ESS μ-law call") |
| ESS | V.92 PCM up, A-law | Pass, 8 of 8 (and 3 of 3 before the retime), B1u locked at 37333 first time, 48000-52000 down. Before: about 30000 up and 3 of 6 calls. Our Jp had always asked for no Su extension, so the network A/D sampled ESS's pulse half a symbol off its peak (`docs/v92_pcm_upstream_findings.md`, "The receiver gap was the A/D's sampling phase"). ESS renegotiates 1-3 times a call, as it does against D-Modem's own modem. In about 1 call in 8, ESS's own transmit clock slews about −180 ppm through its first TRN1u. That also happens against `half_open8` at some line delays. Our receiver tracks it, and those calls lock B1u as usual. Until our data-mode frequency correction, a renegotiation's TRN2u came out at up to 153 DS0 and could cost a call its upstream. Now it is 57-61, the same as startup (`docs/v92_pcm_upstream_findings.md`, "A renegotiation's TRN2u was our held frequency walking") |
| ESS | V.90 | Pass, 8 of 8 over line delays, 45333-50666 down. Before: fail, because we rejected ESS's real S during our Jd as our own line (0.125-0.148 of our transmit RMS), so ESS retrained twice and fell to plain V.34, where we CONNECTed on a link nobody decodes (`docs/v90_phase3_4_spec_audit.md` row 12). ESS's plain V.34 fails against `half_open8` too |
| HSF | V.90, V.92 (both laws) | Fail. Since the INFO1d re-send fix (below), V.90 gets a V.90 INFO1a (3200 up), our Jd is answered with S, and HSF retrains in the second DIL cycle without sending the §9.3.2.10 S. Its V.92 rows decline PCM in INFO1a (U_INFO 2, 2400 baud). Against `half_open8` it picks a 3429-baud upstream, which we do not offer |
| Agere | V.90, V.92 | Pass, 5 of 5 per row since the CP framer fix (below), first attempt with no retrains: V.90 53333 down / 28800 up, V.92 (V.90 Phase 4, no PCM upstream) 53333-54666. Before: plain V.34 at 31200 after about 6 retrains, first because we rejected its CPt, then because we missed its single CP and CP' |
| Broadcom | V.92 PCM up | Pass since the L2 hold (below): 50666 down, PCM upstream, B1u locked, first attempt. Before: declined PCM in its first INFO1a |
| Broadcom | V.90 | Pass, 5 of 5 since the CP framer fix (below): 49333, first attempt. Before: plain V.34 at 31200 after 7 retrains. Its INFO1a chooses V.90 with a 3200-baud upstream, as Agere's does. Against `half_open8` it picks a 3429-baud upstream, which we do not offer (V.90 6.2 makes 3429 optional; INFO0d bit 40 = 0) |
| Courier | V.8 default | Fail. V.8 selects V.90, Phase 3, then NO CARRIER |

D-Modem's own modem passes the same rows. For example, Agere V.90 against their
`half_open8` passes on CT 128, and SmartLink V.92 against it connects at
56000/38666 with a 64-tap prefilter where we get 34666. So these failures are
ours to explain. Every failing row is a `NET_LOOP=0.45 NET=8000:8000` line,
except the Courier, which has the same line. PCTel and SM56 pass on that line,
so the line alone does not explain it. The sections below cover Phase 2
(INFO1d re-send, L2 length) and ESS V.92 (`docs/v92_pcm_upstream_findings.md`,
"The ESS modem itself"). There, D-Modem's own modem also fails with a
precoder and no prefilter. Unprecoded, it gets 29333. Ours now falls back to
an unprecoded CPd with decision feedback in data mode once a precoded B1u
fails, and answers the analogue modem's §9.8 Ru. That passes ESS μ-law, and
on the same build the SmartLink V.92 (precoded, both laws), SmartLink V.90,
PCTel, SM56 and Intel rows still pass.

## Phase 2: our INFO1d re-send talked over INFO1a (2026-11-25)

The HSF, Agere and Broadcom rows never got past Phase 2 on their first
attempt. After INFO1d we re-sent it as soon as the line had been silent for
120 bauds (200 ms). §9.2.1.2.6 gives the analogue modem 700 ms plus a round
trip from the end of INFO1d, and re-sends only on INFOMARKSa. These peers stop
Tone A once they have INFO1d and start INFO1a about 210 ms later (D-Modem's
own modem against Agere: INFO1d ends at 5965 ms, INFO1a decoded by 6174 ms).
So our second INFO1d went out on top of their INFO1a, we never decoded one,
and the peer retrained; the retrain's INFO1a then chose V.34. Waiting for the
deadline before a silence-triggered re-send (`v34tx.c`,
`get_v90_wait_info1a_baud()`) gives a V.90 INFO1a from HSF and Agere first
time. All three still fail later, as the rows above say. The rows that passed
still pass (SmartLink V.92 and V.90, ESS V.92 both laws, ESS V.90 7 of 8 over
line delays, PCTel, SM56, Intel), and `make test-fast` passes.

Two things are now visible that were hidden behind it:
- With D-Modem's own modem, HSF, Agere and Broadcom all choose a 3429-baud
  upstream. We offer 3000 and 3200 only, as V.90 6.2 allows, so a 3429 upstream
  receiver is the likely way to Broadcom.
- Agere retrains during our Jd, and HSF during our DIL. Agere's is explained
  in the next section; HSF's is not yet.

## Phase 2: L2 ends at Tone A, not at 300 ms (2026-11-25)

Agere retrained during our Jd although our Sd, S̄d, TRN1d and Jd were
sample-identical to `half_open8`'s (signs, magnitudes 7676/3772 and frame
bits; only TRN1d's length differed). The cause was in Phase 2. As the V.90
digital modem we sent L2 for a fixed 15 periods (300 ms, `l2_cycles()`) and
then went silent to wait for Tone A. §9.2.1.1.7 sends INFO1d "after the
digital modem detects Tone A and has received the local echo of L2 for a
period of time not to exceed 550 ms plus a round-trip delay", and §9.2.2.1.8
lets the analogue modem receive L2 for up to 500 ms. So L2 runs until Tone A.
D-Modem's modem does that (580 ms of L2 against Agere).

Agere's own status log showed what it made of our 300 ms (open8 in brackets):
- Rx signal level −69.5 dBm (−9.6), measured about 430 ms after it saw L2
  start, i.e. silence.
- In-band SNR 1.75 dB (36.6), and a "frequency offset" of 18.75-37.5 Hz (0.000).
- `PCM phase 2 detected RBS`, which open8 never gets.
- Then in Phase 3 its PCM timing loop ran to +81.6 ppm on our TRN1d (−0.06
  against open8), it raised exception 40009326, and it retrained. Against
  open8 it logs `Jd detected: ffff 403f` instead.

Fix (`v34tx.c`, `v90_extended_l2_arm()` and `v90_extended_l2_step()`): 300 ms
is now the minimum. If Tone A has not come by then, L2 continues, with the
receiver listening for Tone A under it (L1/L2 omit 2400 Hz so that it can be
heard). INFO1d follows the first Tone A event or 40 ms block with a 2400 Hz
share of at least 0.15. At 550 ms plus a round trip of L2 we fall back to the
silent wait, whose §9.2.1.2.5 deadline still counts from the start of L2.
`ME_V90_L2_UNTIL_TONE_A=0` restores the fixed L2. Two smaller changes go with
it. A Tone A reversal left in the event slot from §9.2.2.1.6 is cleared when
our L1 starts, because it was sending INFO1d at the end of L2 as if Tone A had
come. And the silent wait no longer treats 50 ms of silence as a Tone A whose
edge it missed.

Result on CT 128: Agere's held-L2 attempts measure a clean line (RBS detector
SNR 80 dB, no RBS), Agere gets through PCM Phase 3 and Phase 4 is now the
blocker (row above). Broadcom V.92 connects first time with a PCM upstream.
The rows that passed still pass, and `make test-fast` passes after one
fixture change: the analogue-role renegotiation test's drn sits on a knife
edge (codec-flip sigma 8.06 vs 7.93) and lands on 15 instead of 11 when
INFO1d moves by 40 ms, so it now accepts either.

Still open:
- Agere's first Phase 2 attempt still reads our L2 as silence, even held to
  540 ms; the attempts after a retrain do not. Our Tone B reversal comes
  41.5-42.1 ms after Agere's Tone A reversal in every attempt (§9.2.1.1.6 asks
  40 ± 1), good and bad alike, so that is not the difference.
- Phase 4 against Agere: we never decoded its CPt. Solved since; see
  "Phase 4: Agere's CPt has a shaping zero past unity" below.
- After a retrain our INFO1d probe can read the peer's L2 as unusable
  (SNR −8.9 dB) and offer no rates at all; the next INFO1a is then invalid.

### Phase 4: Agere's CPt has a shaping zero past unity (2026-11-25)

Agere logged `CPt sent`, then `R detected`, and retrained 5 s later; our
strict CP receiver logged nothing. Its CPt is clean. An independent
4-point demodulator on our receive tap (1920 Hz, 3200 baud, blind equalizer)
reads it at 33 dB SNR, and the frame is CRC-valid (remainder 0). Its fields:
CPt, drn 18, rate mask 0x1fff, Sr 1, ld 1, TRN1d gain 0x2000, one
constellation, and spectral shaping a1 = 0x40 (+1), a2 = 0xc0 (−1),
b1 = 0x3f (63/64), b2 = 0xbf (−65/64). These match Agere's own log line
`CPt sent 2048 7ffe 2000 c040 bf3f`.

Our receiver demodulated it correctly and then rejected every copy in
`vpcm_cp_validate()`. §5.4.5.6 says each coefficient has |·| ≤ 1, and b2 is
one LSB past that. Agere puts each zero of the shaping filter one LSB beyond
its pole. The digital modem uses these four values only in the sign-selection
metric F(z) = (1 − b1z⁻¹)(1 − b2z⁻¹) / ((1 − a1z⁻¹)(1 − a2z⁻¹)). There b1 and
b2 are feed-forward zeros, so any Q1.6 value keeps w[n] finite; only a1 and
a2 (feedback) diverge past unity. The validator now bounds only a1 and a2.
With the strict check relaxed, the unchanged live receiver
(`v90_cp_live_probe` on the tap cut 1.5 s after the Phase 4 hint) recovers the
same frame. `vpcm_loopback_test` carries Agere's 290 received bits as a
fixture.

Agere also ends each frame after one fill bit (290 bits, not Table 14's 292),
as the CX93001 does. Fill already gates nothing (c2377f6d). The batch
receiver's repeat-voting still assumes a 292-bit period. It found these
frames anyway through its single-frame path.

Result on CT 128: the CPt is accepted in every attempt.
- Agere V.90: 53333 down / 28800 up in 5 of 6 runs (`Rx Data Mode Active`,
  LAPM, first attempt, 33 s to CONNECT).
- Agere V.92: 53333 in 2 of 4 runs.
- Broadcom V.90: 49333 in 1 of 4 runs.
- Broadcom V.92: unchanged, 50666 with a PCM upstream.

The other runs failed one step later, and that was ours too (next section).

### Phase 4: a false sync at the end of SCR cost us the CP (2026-11-25)

Agere sends SCR while it trains on our TRN2d (§9.4.2.2), then ONE CP. It has
already heard our MP, so it follows with ONE CP' (§9.4.2.3-.4), then E and
B1. Its log on a failing attempt shows `CP sent`, `MP detected`, `CP' sent`,
`Tx Data Mode Active`, and an exception 1 s later. It never logged
`MP' detected`, because we never sent MP'. On the raw tap, the independent
demodulator reads CP (drn 20) and CP' at 31-35 dB with valid CRCs, and no
timing step at the SCR-to-CP boundary.

SCR descrambles to ones, and Agere ends it with a few zeros before the CP's
sync. A run of ones followed by a zero is a CP sync to `v90_cp_rx`, so the
framer started a false frame 41 bits before the real one.
- It took the false frame's length from noise in bits 103:128. When that
  came out long (up to 1788 bits), the framer swallowed the CP and the CP'
  whole.
- When a noise DFI was out of range, the framer dropped the false frame at
  bit 129, in time for the CP' to get through. That is why some runs passed.
- Either way it REJECTED the false frame, and every rejection calls
  `v34_reject_v90_phase4_hypothesis()`. That unlocks the CP tap's bit mapping
  and restarts its preamble search, just before the real CP. The tap's bits
  (`V90_CP_BIT_DUMP`) then came out garbled for about 300 bits from that
  point, while the independent demodulator on the same samples read them
  clean.

Fix (`v90_cp_rx.c`):
- Table 14 puts a zero start bit at every multiple of 17, from bit 17 to the
  CRC's start bit (272+δ). A one there ends the frame at once: a real sync's
  ones land on one within 17 bits.
- The framer then hunts again through the bits it had collected, so the real
  sync among them is found.
- An abandoned frame counts as superseded (`superseded_syncs`), not
  rejected, when that hunt completes a sync. Only a frame that is not
  followed by a sync is still a rejection that unlocks the hypothesis. A
  wrong bit order can produce sync-like ones, and its recovery is unchanged.

`vpcm_loopback_test` replays 1010 bits of Agere's SCR end, CP and CP' as our
tap recorded them. The old framer accepts nothing from them; the new one
accepts the CP'.

Result on CT 128:
- Agere V.90, Agere V.92 and Broadcom V.90: 12 of 12 calls connect PCM on
  the first attempt with no retrains (53333 / 53333-54666 / 49333), and data
  is correct both ways.
- The full matrix is at 14 of 18 rows. Only the three HSF rows and Courier
  fail, as before.

An earlier note here read Agere's Phase 3 TRN as "white" (0.638 from the
grid) and its Ja as decoded "only through the hypothesis-8 recovery".
Hypothesis 8 is the ordinary differential map (`v34_rx_map_phase4_raw_bits`),
and Agere's upstream at 3200 baud is a 33 dB signal that we demodulate
correctly. That reading was wrong.

### Agere's own status log

`half_agere`'s `AGERE_DEBUG=1` prints only codec writes, but the library
formats every status event through its exported `StatusParser()`. On CT 128,
`/root/agdbg` builds `half_agdbg`, a copy of the peer that prints them:
`objcopy --weaken-symbol=CodecStatusHandler --add-symbol
AgOrigCSH=.text:0xaf760,function,global` on `agrsm_core.o`, plus a
`CodecStatusHandler()` that calls `StatusParser()`, prints the text as
`AGST <seconds> ...` and calls the original. It is a local diagnostic build,
not a change to D-Modem. Run it as `./half_agdbg` in place of `./half_agere`.
Its log gives Agere's view of every phase: L1/L2 level, SNR, frequency and
timing offsets, the RBS verdict, RTD, `S detected`, `Jd detected`, CPt sent,
R detected and exception codes.

