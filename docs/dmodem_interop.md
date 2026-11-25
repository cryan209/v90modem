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
| HSF | V.90, V.92 (both laws) | Fail. V.90 now gets its own DIL (N=147) since the Ja template header check (below), reaches Phase 4, sends CPt and SCR, and retrains without sending CP. That is the training constellation our −6 dBm0 INFO0d lets it choose; at −12 dBm0 it connects in 3 of 4 calls (below). Its Phase 2 used to fail (no CRC-valid INFO1a) in about half the runs, because our Tone B reversal could miss the wire; fixed (below). Its V.92 rows decline PCM in INFO1a (U_INFO 2, 2400 baud). Against `half_open8` it picks a 3429-baud upstream, which we do not offer |
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

### Phase 3: we answered the HSF's Ja with the CX93001's descriptor (2026-11-25)

The HSF V.90 row got its Jd answered with S, then retrained in our second
DIL cycle. We had sent the DIL for N=119, LSP=124, LTP=126: the CX93001's
descriptor, which `v90_ja_template_scan()` substitutes when a 384-bit stretch
of Ja agrees with the CX's frame at 70% or more. The HSF's Ja matched at
271-273/384, before a whole frame of it had arrived. Its own descriptor is
N=147, LSP=126, LTP=126 (CRC-valid, the same in every call). With
`ME_V90_JA_TEMPLATE=0` we decode it, send that DIL, and the HSF ends it with
S/S-bar as §9.3.2.10 says.

The template's "8 sigma from chance" assumed random bits. Table 12 frames
are not. Aligned at frame bit 18, the HSF's Ja agrees with the CX's at
71% over bits 18:401, 77-78% over 146:529 (the SP/TP patterns and early
training Ucodes are near-common), and only 52-58% past bit 1000. What tells
the two apart is the header. The HSF's bits 18:50 arrive clean: all 11
fixed-zero bits (26:34, 42, 50) read zero, and N/LSP-1/LTP-1 differ from
the CX's in 5 of 22 field bits.

Fix: `v90_ja_header_names_other_descriptor()` (`v90_ja_match.h`). A template
match whose header is well formed and differs from the template's in 3 or more
field bits is another peer's Ja, and we wait for its CRC-valid descriptor. A
noisy header (any fixed-zero bit set) gives no verdict, so the CX's poorly
equalized Ja is still recognised. The CX fixture
(`reference-data/artifacts/cx93001-ja-crc-fails-20261023`) still matches,
384/384. On CT 128 the HSF now logs "header names another descriptor", parses
N=147 and gets the right DIL in every call.

### Phase 4: the HSF fails on the training constellation our INFO0d lets it ask for (2026-11-25)

With the right DIL, the HSF sends 5 CPt frames, sees our Ri-to-R̄i (§9.4.2.2),
sends SCR for about 2.8 s and retrains. It never sends CP. Against
`half_open8` it sends CP 3.5 s after CPt and connects. That holds with a
3200-baud upstream too (`DSP_V34_INFO0=7b,a` withholds 3429, giving
52000/31200), so its 3200 path is not the cause.

Everything we send is the same as what `half_open8` sends. Comparing the two
calls' downstreams from `NET_WAV`:
- Sd, S̄d and the start of TRN1d are byte-identical. `half_open8`'s TRN1d is
  about 8000T, ours 20004T, and 8004T changes nothing.
- Jd is identical (same rate mask, ld 1, CRC), as are J'd and the position of
  the last Jd frame before DIL.
- Both DILs equal our §8.4.1 generator for HSF's descriptor through a full
  cycle.
- Ri and R̄i are identical. Our TRN2d equals `half_open8`'s for all 2166 of
  its symbols, which also confirms our §5.4.5 shaper for a1 = 63/64.
- MP is CRC-valid in both. Ours offers 28800, 16-state, minimum shaping.
  Offering 31200 (`ME_V90_UPSTREAM_NO_SNR_CAP=1`) changes nothing.
- TRN2d length (2166T, 16000T), Ri length (4000T-16000T) and holding Jd 50 ms
  past S also change nothing.

What differs is the CPt the HSF sends: Ucodes 97, 88, 77, 52 to us, and
88, 81, 68, 45 to `half_open8`, in every call. The cause is INFO0d bits 33:37
(Table 7): we declare a −6 dBm0 maximum at the codec output, while
`half_open8` declares −12 dBm0 at the terminals. With
`ME_V90_MAX_TX_DBM0_CODE=23` (−12 dBm0) the HSF asks for 88, 81, 68, 45 and
connects V.90 in 3 of 4 calls, with data correct both ways (the fourth failed
in Phase 2). So the HSF chooses a training constellation it then cannot train
on when it is offered −6 dBm0. We have not established whether that is the
HSF, or the codec model of D-Modem's harness.

The default stays at −6 dBm0 for now. On the full matrix at −12 dBm0, every
other row still passes, but most connect lower:
- SmartLink V.90: 54667 (was 56000).
- Agere: 52000 (was 53333).
- Intel: 50666 (was 52000).
- SM56: 45333.
- ESS and Broadcom: unchanged.

The −6 dBm0 value was chosen for the RasFinder (v34tx.c, bits 33:37). With
`ME_V90_CP_POWER_ENFORCE` off (the default), a quieter declaration would not
reject anyone's CPt.

### Phase 2: our Tone B reversal could miss the wire (2026-11-25)

The HSF rows often failed in Phase 2: no CRC-valid INFO1a, then a retrain.
Six sequential HSF V.90 calls on 221960e2 got INFO1a on the first Phase 2 in
1 of 6, and in 2 of 6 never. In a failing call the HSF is silent through all
of our L1/L2 and our INFO1d. It raises Tone A about 0.35 s after INFO1d has
ended, then holds it while we wait for INFO1a.

That is what §9.2.2.1.7 predicts when the analogue modem has not seen our Tone
B phase reversal. It conditions its receiver for L1/L2 only after detecting
that reversal. The recording shows the reversal was never sent:
- Failing call: the HSF's Tone A reversal is at 7.1311 s. Our Tone B runs
  unreversed until 7.1725 s and goes straight into L1.
- Passing call: our reversal is at 7.1500 s, 42 ms after the HSF's. About
  22 ms of reversed Tone B follows before L1, against the 10 ms §9.2.1.1.6
  asks for.

Both calls log the same transmitter timing (SECOND_B_WAIT 38.3 ms,
SECOND_NOT_B 8.3 ms). The cause is `tx_cc_modulation()`. It keeps
generating the control channel to the end of the `v34_tx()` block (20 ms
here), even after a baud handler has switched the modulator to PCM L1/L2. So
L1 starts at the block boundary:
- If the reversal baud falls early in a block, 10–30 ms of reversed tone goes
  out.
- If it falls within the nine-symbol shaper's delay (about 6.7 ms) of the
  block's end, none does.

The K56flex Tone A reversal (V.34 §11.2.1.2.5) already had the fix: keep the
CC output for (9−1)/2 bauds, then switch to L1 at that exact sample. The V.90
digital modem's SECOND_NOT_B now takes the same path, into PCM L1/L2. On the
wire that was 8.2 ms of reversed Tone B, then L1; the follow-up below makes it
10 ms.

After the fix, on CT 128:
- Sequential HSF V.90 calls get INFO1a on the first Phase 2 in 9 of 9.
- On the full matrix, all three HSF rows do, with three calls in parallel.
- The HSF rows still fail at Phase 4 on the −6 dBm0 constellation (above).
- The matrix stays at 14 of 18, data exact on every connecting row. Rates
  are within run-to-run spread: Intel 49333, SM56 50666, Agere 53333.

Follow-up, same day: the plain V.34 call modem's Tone B reversal
(§11.2.1.1.6) ran through the same SECOND_NOT_B stage and handed over at the
block boundary too. It now takes the same tail. The tail also gained one baud:
- SECOND_NOT_B counts the reversal baud as its first and hands over as its
  sixth starts, so only five reversed bauds came before the shaper's delay.
- Measured to the sample on an engine-pair V.34 tap, the reversed tone was
  8.2 ms with the original tail and is 9.8 ms with the extra baud. The old
  block-end handover gave 13.3 ms on that alignment.
- The reversal instant is unchanged (7.2665 s in both builds).

Verification:
- HSF V.90: 6 of 6 first-Phase-2 INFO1a. Matrix: 14 of 18 again, data exact.
- V.34 duplex symbol-rate matrix (2400-3429, both laws): 12 of 12, error-free.
- The `test-fast` row at 10 dB injected noise (`V34_DUPLEX_NOISE_DB=10`) turned
  out to be a coin flip. There the caller decodes INFO1a only at some delays,
  and any few-ms change in Phase 2 timing moves which ones. Over delays 0-11 it
  passes 4/12 at 58655519, 9/12 with the 8.2 ms tail, and 3/12 with 10 ms. It
  now runs at delay 5, which passes on all three. Why INFO1a was lost at
  10 dB is the next section.
- No usable offline third-party peer for our V.34 *call* role:
  - Against `half_open8` our B1 never correlates at any symbol rate (receive
    SNR about −10 dB) on either build, so both calls fall into retrain loops.
    The old build's CONNECT 19200 carries garbage.
  - `half_smartlink` called with `PROTO_B=ATX3` never completes V.8.

### The INFO receiver missed reversals in noise (2026-11-25)

The 10 dB failures were ours: our caller lost INFO1a to a missed reversal.
The bits it received in INFO1a's window show it:
- At delay 5 it gets fill and sync `1111 01110010` and decodes INFO1a.
- At delay 0 it gets `1111 01110000`. One '1' (a 180° phase change, V.34
  §10.1.2.3.1) never registered, the flywheel supplied a '0', and the sync
  never matched.

`info_rx()` declares a '1' when the phase jumps more than 90° between two
samples. The RRC-shaped turn takes several samples, and in noise its 180° can
split into steps that never exceed 90° in any two. The control channel's
in-band SNR there is far above what DPSK needs, so the loss is the detector's.

The jump is now measured across three samples. The other options failed:

| Variant | 10 dB sweep, delays 0-11 | Other tests |
|---|---|---|
| Two samples (as before) | 3/12–9/12, varying with Phase 2 timing | all pass |
| One-baud phase check at the flywheel's zero decision | 10/12 | broke the `engine_pair` §11.5 retrain: false INFO0 sync codes in the peer's tones |
| Four samples | 10/12 | broke `v56_loopback_test` at 40 dB with a −1 dB echo: echo wobble corrupted a clean INFO0 |
| Three samples (now) | 10/12, INFO1a decoded at all 12 | all pass |

The three-sample results in full:
- 10 dB sweep: delays 2 and 8 still fail, later, at MP.
- `v56_loopback_test` echo case over delays 40–140 at 40 and 25 dB SNR:
  12/12, the same as the old receiver.
- `test-fast` 775/775. V.34 duplex matrix 12/12.
- HSF V.90 first-Phase-2 INFO1a 6/6.
- D-Modem matrix 14 of 18, data exact on every connecting row.

The four-sample build's ESS V.90 data failure did not recur on three samples.
That one run cannot say whether it was the receiver.

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

