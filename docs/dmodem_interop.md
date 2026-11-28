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

## Full matrix run, live rows included (2026-11-27, d91b4de3)

`LIVE=1 SIP_V90MODEM=<checkout>/sip_v90_modem tools/dmodem_matrix.sh` on CT
128 (wired), with half_v90modem and sip_v90_modem built from d91b4de3 and
D-Modem eceac3b. STREAM=10, one call per row. "Data" is the bytes each side
received intact. This is one call per row, so a pass is not a rate. The
cumulative per-peer history is in the next section.

**Result: 23 of 25 rows passed.** Both failures are HSF V.92, already known
(below). The four live rows passed on their first run through the matrix
script.

| Row | Caller's CONNECT | Data A / B | Notes |
|---|---|---|---|
| `sl_v92_ulaw` | 56000 | 6239/6239, 6613/6613 | B1u locked by decoding, CPd precoder |
| `sl_v92_alaw` | 56000 | 5872/5872, 6224/6224 | B1u locked by decoding, CPd precoder |
| `sl_v90_ulaw` | 56000 | 6973/6973, 7391/7391 | |
| `hsf_v92_ulaw` | none | none | **Fail** at 53 s: Phase 4, we wait for a valid CPt from the HSF. This is its own -6 dBm0 failure, and -12 dBm0 connects (see HSF below) |
| `hsf_v92_alaw` | none | none | **Fail**, the same way, at 53 s |
| `hsf_v90_ulaw` | 115200 (DTE) | 6239/6239, 6613/6613 | V.34 fallback after the HSF declines V.90 |
| `ess_v92_ulaw` | 50666 | 6973/6973, 7391/7391 | B1u locked by decoding |
| `ess_v92_alaw` | 49333 | 6606/6606, 7002/7002 | B1u locked by decoding |
| `ess_v90_ulaw` | 49333 | 6973/6973, 7391/7391 | |
| `agere_v92_ulaw` | 53333 V42bis | 6606/6606, 7002/7002 | V.90 Phase 4, no PCM upstream |
| `agere_v90_ulaw` | 53333 V42bis | 6239/6239, 6613/6613 | |
| `sm56_v90_ulaw` | 50666/LAPM/V42BIS | 6606/6606, 7002/7002 | |
| `pctel_v90_ulaw` | 115200 (DTE) | 6606/6606, 7002/7002 | |
| `broadcom_v92_ulaw` | 50666/LAPM/V42BIS | 6973/6973, 7391/7391 | B1u locked by decoding, CPd precoder |
| `broadcom_v90_ulaw` | 49333/LAPM/V42BIS | 6973/6973, 7391/7391 | |
| `intel_v92_ulaw` | 49333 | 6973/6973, 7391/7391 | no CPu, so V.90 |
| `intel_v90_ulaw` | 49333 | 6973/6973, 7391/7391 | |
| `courier_v32b_ulaw` | 14400/ARQ | 7340/7340, 7780/7780 | |
| `courier_v22b_ulaw` | 2400/ARQ | 7340/7340, 7780/7780 | |
| `we_courier_v32b_ulaw` | 14400 (ours) | 7340/7340, 7780/7780 | |
| `we_courier_v22b_ulaw` | 2400 (ours) | 7340/7340, 7780/7780 | |
| `live_courier_v32b` | 14400/ARQ after 11.3 s | PPP: 48 frames good, 0 bad | v90ras on d7adb775 + f8758e39. LCP and IPCP open, 20/20 echoes, 20/20 pings |
| `live_courier_v22b` | 2400/ARQ after 8.7 s | PPP: 54 frames good, 0 bad | the same |
| `livewe_courier_v32b` | 14400 (ours) | 40/40 lines each way | no retrain, no reception loss |
| `livewe_courier_v22b` | 2400 (ours) | 40/40 lines each way | the one NO CARRIER is the hang-up after the last line |

**Update, the same day:** the two HSF V.92 rows pass since the CPd deadline
(section below, "V.92 against the HSF: retrain when no CPd can be
designed"): V.34 26400, data exact, 8 of 8 over `NET_DELAY` 1-4. The offline
matrix rerun with it is 19 of 21: `hsf_v92_ulaw` missed in that run on the
older two-retrain path (fixed before the 8 of 8), and `we_courier_v22b_ulaw`
connected at 1200 with garbage once, then passed 3 of 3 at 2400 on reruns.

The DTE rates (115200) are what those peers' harnesses print, not line rates.
The previous full matrix was 15 of 18: these two HSF V.92 failures plus a
Courier row. Since then the matrix has gained the two "we call the Courier"
netsim rows and the four live rows.

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
| HSF | V.90 | Pass since the fallback MP fix (below): the HSF declines V.90 after two retrains (its own −6 dBm0 failure), and our V.34 fallback connects 26400 down / 24000 up at the first attempt, 4 of 4 over `NET_DELAY`, data exact (0 of 4 before) |
| HSF | V.90, V.92 (both laws) | Pass since the CPd deadline (below, 2026-11-27): 8 of 8 over `NET_DELAY` 1-4, both laws, V.34 26400 after one retrain of ours, data exact. Not PCM: at our −6 dBm0 ceiling the HSF's TRN2u carries no upstream level set, so we retrain without the PCM upstream offer and the HSF falls back to V.34 (0 of 8 before: SUVd for ~50 s until it hung up). V.92 fails at Phase 4 on the HSF's own −6 dBm0 failure. At −12 dBm0 it connects 52000/36000 PCM up, 4 of 4, since the Phase 3 fixes (below: Ru2's phase after MD, Su's level); 0 of 4 before. The rest of this cell is the history of both rows. V.90 now gets its own DIL (N=147) since the Ja template header check (below), reaches Phase 4, sends CPt and SCR, and retrains without sending CP. That is the training constellation our −6 dBm0 INFO0d lets it choose; at −12 dBm0 it connects in 3 of 4 calls (below). D-Modem's own modem fails identically at −6 dBm0 but falls back to plain V.34, which ours failed against the HSF at 3200 baud (0/4 vs 4/4) until the fallback MP fix (below, 2026-11-26). Our plain V.34 against it is now 1/8 by default and 7/8 with `V34_DUPLEX_SPEC_S=1 V34_P4_KEEP_BAUD_PHASE=1`, which wait on a fix to our own call-modem Phase 4 receiver (below). Its Phase 2 used to fail (no CRC-valid INFO1a) in about half the runs, because our Tone B reversal could miss the wire; fixed (below). Its V.92 rows decline PCM in INFO1a (U_INFO 2, 2400 baud). Against `half_open8` it picks a 3429-baud upstream, which we do not offer |
| Agere | V.90, V.92 | Pass, 5 of 5 per row since the CP framer fix (below), first attempt with no retrains: V.90 53333 down / 28800 up, V.92 (V.90 Phase 4, no PCM upstream) 53333-54666. Before: plain V.34 at 31200 after about 6 retrains, first because we rejected its CPt, then because we missed its single CP and CP' |
| Broadcom | V.92 PCM up | Pass since the L2 hold (below): 50666 down, PCM upstream, B1u locked, first attempt. Before: declined PCM in its first INFO1a |
| Broadcom | V.90 | Pass, 5 of 5 since the CP framer fix (below): 49333, first attempt. Before: plain V.34 at 31200 after 7 retrains. Its INFO1a chooses V.90 with a 3200-baud upstream, as Agere's does. Against `half_open8` it picks a 3429-baud upstream, which we do not offer (V.90 6.2 makes 3429 optional; INFO0d bit 40 = 0) |
| Courier | V.8 default | Not a valid target, dropped from `dmodem_matrix.sh`: D-Modem's own `half_open8` fails it the same way (V.90 Phase 3, the Courier's Tone A, a V.34 retrain that never finishes, NO CARRIER at 63 s), and D-Modem's source lists the emulated Courier's V.34 as not yet working even between two Couriers. See "The emulated Courier" below |
| Courier | V.32bis, Courier calls | Pass, 32 of 32 since the S fit at both T/2 phases (below), 14400, data exact. 0 before the CA reversal fix, then about 1 in 12 to 1 in 4 lost the S |
| Courier | V.22bis, Courier calls | Pass, 2400/ARQ, data exact |
| Courier | V.32bis, we call | Pass, 32 of 32 at 14400, data exact (`we_courier_v32b_ulaw`); 0 before the R3 and loss-retrain fixes below |
| Courier | V.22bis, we call | Pass, 24 of 24 at 2400, data exact (`we_courier_v22b_ulaw`); every call fell to 1200 before the S1 fix below, then 4 of 40 before the other-eye S1 search |
| Courier, live | V.32bis / V.22bis, Courier calls v90ras | Pass, 5 of 5 each: 14400/ARQ and 2400/ARQ through Asterisk into v90ras's pppd; every call LCP + IPCP open and 20 of 20 LCP echoes and pings intact, no retrain. See "The emulated Courier on a live SIP call" below |
| Courier, live | V.32bis / V.22bis, we call | Pass, 5 of 5 each: CONNECT 14400 and CONNECT 2400, 40 of 40 lines exact both ways. 1 call in 16 had a V.32bis loss retrain in data, which completed. The cause was our data-mode carrier loop, which kept its acquisition gains; fixed (see below) |

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
  (2026-11-28: no longer a failure. All three pass against us on a 3200
  upstream: Broadcom 5 of 5, Agere and Broadcom V.90 12 of 12, HSF on its
  V.34 fallback. A 3429 upstream would be a rate feature, 33600 up, and not
  an interop fix. It needs INFO0d bit 40 (`v34tx.c`, now a literal 0), INFO1d's
  3429 row, and the digital upstream path, which is fixed at 3000/3200 in at
  least 13 places: `v90_selected_upstream_baud_locked()`, the B1/CP-live/T/3
  rate code in `modem_engine.c`, `v90_p3_ja_grade()` and
  `v90_upstream_replay`. Only `p3_demod.c` has 3429 tables. Setting bit 40 also
  changes what every live V.90 caller may pick. The Intel/Conexant V.92 picks
  3429 even unoffered (`serial-x0-2052`), so it needs a live A/B, not only this
  matrix.)
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
  Not reproduced on 2026-11-28 (at 5933dc6f). Three forced retrains against
  Agere V.90 (`ME_V90_RETRAIN_AFTER_MS=3000`) re-offered 3000 = 12 and
  3200 = 13 and reconnected at 53333, 3 of 3. Only 2743 and 2800 dropped
  from 11 to 0 after the retrain.
- Agere's first-attempt L2: not reproduced on 2026-11-28 either. Agere V.90
  and V.92 trained at the first attempt in 10 of 10 calls (no retrain, one
  INFO1a), Broadcom in 4 of 4.

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

### The −6 dBm0 failure is the HSF's; its V.34 fallback fails on us (2026-11-26)

The open question above is settled, and the default stays at −6 dBm0.
Each arm below is 4 calls at `NET_DELAY` 1-4, μ-law, on 1496fe03.

The cause is not clipping or level in the harness. A diagnostic copy of
`netsim` (`/root/netdiag/netsim_diag` on CT 128) scales the loop path toward
the HSF (`NET_LOOP_GAIN_DB`) and counts samples past 16 bits:
- The loop output peaks at 20696 with 0 samples over 16 bits in every call.
- With 6 dB of extra loss, our −6 dBm0 arm still fails 0/4. In that arm the
  HSF's CPt is unchanged (97, 88, 77, 52) and its levels arrive below those
  of the −12 dBm0 set. The −12 dBm0 arm connects V.90 at 0 dB (3/4) and at
  −6 dB (4/4).

D-Modem's own modem fails the same way when it declares what we declare. A
diagnostic `half_open8` (`/root/netdiag/half_open8_pwr`) takes INFO0d bits
33:37 and bit 38 from `DIAG_V90_MAX_POWER` / `DIAG_V90_B38`:
- **Code 11 (−6 dBm0), at the codec output or at the terminals:** the HSF
  asks it for CPt 97, 88, 77, 52 too. It never sends CP, retrains twice
  (Tone A), then connects plain V.34 at 33600. That holds in 8 of 8 calls
  and is never V.90.
- **Code 23 (−12 dBm0, its default):** V.90 on the first attempt, 4 of 4.

So the HSF chooses, from a −6 dBm0 ceiling, a training constellation it
cannot train on, whoever the digital modem is. Changing the default would
cost every other row 1.3-2.7k to suit one peer.

What IS ours is the fallback. open8 survives the HSF's two retrains because
its plain V.34 works against the HSF; ours does not. HSF caller with
`AT+MS=V34`:
- **Us:** 0/4. Phase 3 completes and our Phase 4 TRN reads 30-34 dB, then
  the HSF sends Tone A, retrains and gives up (NO CARRIER).
- **open8:** 4/4 at 33600 (3429 baud).
- **open8 with `DSP_V34_INFO0=7b,a` (no 3429):** 4/4 at 31200. So 3429 is
  not the cause, since our calls run at 3200 baud.

The HSF rows now fail on our plain V.34 at 3200 baud, not on the power
declaration. That is the next thing to chase on this peer.

### Plain V.34 against the HSF: three defects of ours, one fixed (2026-11-26)

These are calls with the HSF as caller (`AT+MS=V34`), us answering, μ-law,
`NET_LOOP=0.45`, at `NET_DELAY` 1-4 ms. The HSF's own `AT#UD` after a failed
call gives key 60 = 0x3D, which is V.58 clearCause 61, trainingFailed. Key
0x33 = 5 is the count of retrains.

**Update (2026-11-26, later): fixes 2 and 3 are now the defaults.** The
loopback failures that kept them opt-in (below) no longer reproduce at
`V34_DUPLEX_DELAY=23`. With both on, the fast tier passes 783/783 and the
slow tier passes too; the two slow failures seen under load passed alone, or
failed the same without the knobs. The same HSF A/B on 3dc003df, 8 calls an
arm alternated over `NET_DELAY` 1-4:
- new defaults: 7/8, 28800-31200 after our B1 step-down;
- `V34_DUPLEX_SPEC_S=0 V34_P4_KEEP_BAUD_PHASE=0`: 2/8.

The full matrix on the new defaults is unchanged: 15 of 18 that run, plus
`ess_v90_ulaw`, which lost data (7002/7391) in that run. Rerun alone, it
fails 1 of 3 on the new defaults and 1 of 3 on 3dc003df (the old ones), with
received octets short or extra both ways. That is an intermittent
data-integrity defect on the ESS V.90 row, independent of this change. It is
fixed: "ESS V.90 carries its scrambler and trellis state into B1" below.

The failing new-arm call is the Phase 2 INFO1c loss below. Our caller's
Phase 4 receiver was not the problem it looked like. It finds no S/S̄ at
most delays with either S (old S: at 1 of 5 delays), and §11.4.1.1.1 is
the reason: the call modem detects S then S̄ *before* its J′, so its
Phase 4 receiver normally starts in TRN and takes the "no junction seen"
path. Our caller sends J′ on S alone (a tone-energy detector), not on S
then S̄; that is still a deviation from the clause.

Results over 8 calls an arm, alternated (before the update):
- **Defaults (fix 1 only):** 1/8. That was 0/4 on 1496fe03.
- **`V34_DUPLEX_SPEC_S=1 V34_P4_KEEP_BAUD_PHASE=1` (fixes 2 and 3 as
  well):** 7/8, data exact both ways. The HSF's MP asked 33600 of us on all 7
  calls that reached MP, and the eighth died in Phase 2 (below).

Fixes 2 and 3 are opt-in because our own call-modem receiver depends on the
defects they remove (below).

**1. We ignored the HSF's MD (V.34 §11.3.1.2.4). Fixed, default.** Its
INFO1c asks for 20 × 35 ms of MD. After the caller's first S-to-S̄ the
answerer must transmit silence, wait out the MD, then detect the second
S-to-S̄ and train on the PP that follows. We kept sending J through the MD
and armed only a guard field that nothing read. When we did see the second S,
we went straight to Phase 4, ahead of the caller's PP, TRN and J. So we sent
4-point Phase 4 TRN and MP to a modem whose J asked for 16-point, and we read
its Phase 3 TRN as Phase 4 TRN (52% ones). Now
`v34_answer_phase3_md_wait_init()` sends silence for the MD, re-arms both S
detectors, and waits up to 1.5 s for the second reversal before training on
PP/TRN and waiting for J.

**2. We jumped our symbol clock at Phase 4. `V34_P4_KEEP_BAUD_PHASE=1`.**
`phase4_wait_init()` zeroes `tx.baud_phase` and the shaper, which is meant
for a V.90 path that hands TX back from another modulator. In plain V.34 the
V.34 modulator has been running since Phase 3, and its clock is the one the
HSF's timing recovery locked to on our PP and TRN. The reset moves our
symbols by whatever fraction of a symbol has elapsed. That fraction depends
on the line delay, so the HSF's verdict did too: 4800 at 2-3 ms, often good
at 1 ms. With spec S on, over 8 calls, the HSF's MP asked of us:
- clock reset: 33600 or 28800 on 3, 4800 on 4, and 1 call died in Phase 2;
  4 connected.
- clock kept: 33600 on 8; 7 connected.

**3. Our duplex Phase 4 S and S̄ are not V.34's (§10.1.3.7, §11.4.1.2.1).
`V34_DUPLEX_SPEC_S=1`.** This is the generator defect that the half-duplex
path fixed on 2026-10-09 for the Diva card (`get_hdx_s_not_s_baud()`):
- S alternates 0° and 90°, not 45° and 135° (point 0 of Figure 5 and its
  90° rotation).
- S runs 127T.
- S̄ ends by rotating back to an S point, so TRN begins 17T after the
  reversal, not 16T.

D-Modem's modem sends exactly the specified sequence (`v34_trn4` is our
`training_constellation_4`). With the clock kept, the HSF asked:
- spec S: 33600 on 8 of 8;
- old points with only the counts corrected: 4800 on 5 of 8. So it is the
  orientation the HSF needs; it takes TRN's phase from S.
- old S: 31200 or 4800 (4/8 connected).

The same spec sequence in Phase 3 (S, S̄ before PP) did not help (6/8). It
also broke our own answerer's Phase 3 receive (TRN at 2.9 dB), so it is not
in.

**Why 2 and 3 are opt-in.** Both break rows of our own pair in `test-fast`:
- The kept clock: `v34_duplex_test 3429` at `V34_DUPLEX_DELAY=23` never
  decodes MP. The FDM engine pair (fast.list 802), whose TRN SNR sits on the
  32 dB edge for 33600, reads 31200.
- Spec S: `v34_duplex_test 3200` at delay 23 reads the answerer's TRN at
  2.9 dB.

Both delay-23 failures are our caller joining Phase 4 late. It takes the
"no junction seen; taking TRN as begun" path and then cannot acquire TRN.
So our call-modem Phase 4 receiver acquires only against our own
answerer's reset clock and 0/90° S. A real answerer gives it neither.
Fixing that receiver is the next step, and then 2 and 3 become the defaults.

The receiver's S-to-S̄ countdown (`phase4_s_bar_left = 16`) is also ours.
The repeated +90° step lands on the first S̄ symbol, so 15 remain under
§11.4.1.2.1. The 16 matches only our own stray S symbol. Changed alone, it
passed our loopback but cost the FDM pair its 33600, so it is not in.

What it was not, measured on the way:
- **Our 16-point TRN/MP mapping and scrambler.** They match D-Modem's.
- **Our level.** At −18 and −22 dBm0, 2/4 and 1/4 connected, against 0/4 at
  −14.
- **The Phase 4 S delay.**
- **The MP differential encoder seed.** §10.1.3.9 (1996) seeds it from the
  last TRN symbol, and ours starts from 0. That only rotates the first
  symbol, which differential decoding absorbs. It is still a spec deviation.

Still open on this peer:
- **Our B1 §11.6 renegotiation.** Fixed 2026-11-28 (section "Our §11.6
  opened before the HSF was listening", below). It was the timing, not the
  rate policy. B1's under-reading, which stepped calls down to 21600, is
  fixed too (section "B1 read the taps it arrived on", below): no step-down
  retrains in 24 calls. Still open: an 11.6 whose MP we do not lock (1 in
  about 10).
- **Phase 2.** Fixed 2026-11-28 (section "INFO1c came inside our L1/L2
  window", below). Offline, about 1 call in 8 never decoded the HSF's INFO1c.
  Live, every call missed it. The cause upstream of both that miss and the
  Phase 3 retrain that followed was our 27 ms INFO0a preamble (fixed, "Our
  27 ms preamble was Tone A before INFO0a"). Live HSF plain V.34 is 5 of 5.
- **The HSF matrix rows.** `hsf_v90_ulaw` passes since the fallback MP fix
  (matrix 15 of 18, 2026-11-26; see "The fallback's MP receiver rotated
  away" below). The two HSF V.92 rows never reach the fallback. (This bullet
  used to say the fallback should run the plain answerer. It should not: it
  is the V.34 call modem.)

### The V.90-declined fallback is the V.34 call modem; J′ was 16T (2026-11-26)

**Correction.** The bullet above once read the fallback's missing Phase 4
S/S̄ and MD wait as a defect. They are correct. V.90 §9.2.1.1.8: when INFO1a
selects V.34, the digital modem continues with V.34 Phase 3 "assuming the
role of a call modem", and the analogue modem becomes the answer modem
(§9.2.2.1.9). The call modem's Phase 4 is §11.4.1.1.1: after S-to-S̄ it stops
J, sends one J′ and then TRN. It sends no Phase 4 S, and §11.3.1.2.4's MD
belongs to the answerer. D-Modem's own digital fallback takes the same role.

**Fixed: our J′ lasted 16T.** J′ is one 16-bit pattern (0x899F, Table 19) at
two bits a symbol (§10.1.3.4 via §10.1.3.3), so 8T. `get_trn_baud()` counted
16T: J′ and then 8T more before TRN, so every call-modem Phase 4 TRN of ours
(plain caller and the fallback) started 8T late. Real callers already send
8T, and our answerer connects with them live. In loopback, the 3429/33600 pass
rates over `V34_DUPLEX_DELAY` 0-47 are unchanged at 140/144 (10 dB/2400,
36 dB/28800 and 38 dB/31200 rows); the 38 dB fast row moved from delay 0 to
delay 1, because delay 0 is now one of its four failing delays. The FDM
engine pair connects 33600 both ways (the answer side was 31200).
`V34_J_DASHED_BAUDS=16` replays taps recorded before the fix
(`rf-mt5634-v34-3429-swp`).
The D-Modem matrix with the fix is 13 of 18: the three HSF rows, Courier and
`ess_v90_ulaw`. ESS V.90 never reaches J′ (no `J_DASHED` in any of 7 runs).
It passed 0/3 with the fix and 2/3 with `V34_J_DASHED_BAUDS=16`. Every failure,
the old-J′ one included, ends identically: our own upstream-error §9.6
renegotiation, then 1.4 s with no line signal. That is run-to-run variance in a
path this change does not touch.

**Our answerer cannot decode the caller's J or J′.** In Phase 4 its J′
detector scores 1024 descrambler hypotheses against a periodic J′. During
the caller's J (§11.4.1.2.1 arms it at least 128T before J′), no hypothesis
matches J above 24-27 of 32 bits at 36 dB SNR, which is the noise level; TRN
itself starts at 55-71% ones and converges later. Its "J′" is therefore an
event timed by noise: it fires within 3 bits of J′, which J itself (4 bits
from J′) also meets. Tighter gates scored worse (strict dJ′ ≤ 2: 139/144;
plus the preceding 16 bits reading as J: 136/144). So did locking on J
first, which never locked and fell back to a 1024-bit floor (133/144):
detecting late starts TRN scoring late. The default detector is unchanged. Why its
Phase 4 symbols do not decode J: the answerer resets its receiver under the
running J, and the decode differenced twice. The opt-in fix,
`V34_P4_ANSWER_J_FRAMING=1`, is in [v34_phase4_j_dashed.md](v34_phase4_j_dashed.md).

### The fallback's MP receiver rotated away from the HSF's MP (2026-11-26)

**Fixed.** The HSF's V.90-declined fallback now connects: `hsf_v90_ulaw`
connects 4 of 4 over `NET_DELAY` 1/2.5/4/6.3 ms, against 0 of 4 on the
same delays without the fix, data exact both ways. Each call ran at 26400
down / 24000 up on the first fallback attempt, with B1 at 28.9-30.7 dB.

It was our MP receiver, not the HSF and not our transmitter. The method
was to compare the fallback with plain V.34 against the HSF, with us as the
caller (`DIAL=1 ./interop_call half_v90modem ./half_hsf`,
`ME_V34_SPAN_FLOW_LOG=1`, which is safe offline). With `ME_V34_BAUD=3200`
the plain call's INFO1a matches the fallback's field for field (3200 baud,
high carrier both ways, N=10, no power reduction) and it connects 26400,
2 of 2. Up to MP the two calls match: same Phase 3/4 stage timeline, Phase 4
TRN locked at 100% ones on GPA (tap 4), MP bit-identical.

They differ in the MP search. The HSF sends TRN for close to the full
2000 ms that §11.4.1.2.2 allows, so its MP comes 1460-1836 bauds after
our receiver starts looking. At 1200 bauds with no lock,
`mp_phase4_rotate_retry_mode()` moves to the next descrambler setting.
`mp_phase4_apply_retry_mode()` holds a plain duplex call on the defaults
(differential, GPA, b0,b1). That is the guard
`s->duplex && !s->v90_mode`, and the fallback keeps `v90_mode`. So the
fallback went to b1,b0 at about 1200 bauds and to GPC (tap 17) by 2400,
and it was off the defaults when the HSF's MP went past. Then retry mode 4
(absolute domain) locked TRN's ones as an MP preamble. A boundary
brute-force made a CRC-valid "MP1" from it with 5 start-bit errors,
reserved bit 19 set and a rate missing from its own mask. We sent MP′ and E
to a modem still in TRN, and failed 11.4's E timeout. The plain calls
locked the HSF's MP at 16-18/18 in the default setting each time.

The guard now also covers `v90_v34_fallback`, so the fallback, which is
the V.34 11.4 exchange (V.90 §9.2.1.1.8), keeps the V.34 defaults. V.90's
own CP search keeps the rotation. The earlier reading here ("the HSF cannot
read our MP", 13/18 in the differential domain) was the HSF still in TRN.

Still a defect, separately: the MP1 acceptance above should not have taken
a frame with 5 start-bit errors and a reserved bit set. It is reachable
only when the search has left the defaults.

The HSF V.92 rows never reach the fallback. Their Phase 3 failures were
ours, and are fixed in the next section.

### V.92 Phase 3 against the HSF: Ru2's phase and Su's level (2026-11-26)

**Fixed in Phase 3. The rows still fail at Phase 4, on the HSF's own −6 dBm0
failure.** With `ME_V90_MAX_TX_DBM0_CODE=23` (−12 dBm0) both HSF V.92 rows
now connect, 4 of 4: μ-law and A-law at `NET_DELAY` 1 and 2.5 ms,
52000 down / 36000 up with PCM upstream, data exact both ways. Without the
fix (be7665c2) the same arm fails 0 of 4, and no call gets past Ru2. At the
default −6 dBm0 they now reach Phase 4 and fail there. The full matrix on the
fix is 16 of 18: the two HSF V.92 rows fail, and `courier_v34` fails as before.

There were two receiver defects of ours, one after the other:
- **Ru2 never locked (`v92_p3_rx.c`).** §9.5.1.1.1 has the analogue modem
  send Ru, Ū, MD, then Ru again. The HSF's MD is 700 ms (MD=20, 5600
  samples), and its second Ru comes in one sample later than the first. It
  is on period-6 pattern phase 5, where Ru1 was on phase 4. MD is
  manufacturer-defined, and nothing in the clause keeps Ru's phase across
  it. Our MD wait scored only Ru1's hypothesis, so it timed out every call.
  It now takes Ru2 on whichever of the 12 phase/polarity hypotheses has run
  longest since MD ended, and derives uR from that one. Each call then
  passes the TRN1u gate (100%, 28.9-30.1 dB), and Ja is accepted (N=147).
- **S̄u was never seen (`v92_su.c`).** The Su detector required a six-slot
  bin energy of 900²×9 = 7.29M. The HSF's Su reads 7.20-7.24M and its 24.5T
  S̄u reads 6.56-7.68M. So the §9.5.2.1.7 reversal was missed, Jp was never
  sent, and the HSF retrained after 3.6 s of Su. The floor is now 6 dB lower
  (450²×9). The periodicity and polarity tests are what reject other
  signals, not the level. After this fix, the following all complete:
  - Jp (0.5T)
  - DIL
  - the second TRN1u
  - CPt and Ri
  - TRN2d
  - SUVu, accepted

At −6 dBm0 the HSF then has the CPt it always asks for there: ucodes 97,
88, 77, 52 (TRN2d at 5708 RMS against 4000 for `half_open8`'s
−12 dBm0 set). Its upstream stops being decodable during TRN2u/SUVu:
- The TRN2u noise is 49 DS0 for the first 3500 symbols. Then it comes in
  bursts of about 4000 symbols at 400-500 DS0, which is no better than
  chance, with clean stretches back at about 50 between them.
- The bursts are there with our equaliser's taps held and without its
  Phase 4 least-squares re-solve, so they are not our adaptation.
- The raw tap alternates between two spectra (lag-2/lag-4 autocorrelation
  −0.15/−0.18 against +0.05/0.00), and our timing loop reads the HSF's clock
  walking by up to −200 ppm. Against `half_open8` the HSF's upstream keeps
  one spectrum all through TRN2u and data (`NET_WAV` capture).

The noise figure therefore reads 600-860 DS0 when SUVu arrives.
`v90_build_v92_cpd_frame()` designs no constellation from that. We send
SUVd until the HSF gives up and sends a tone, which is the same failure as
its V.90 Phase 4 at −6 dBm0 (above). The default stays at −6 dBm0, as
decided there.

Our transmitter was checked first. The TRN2d/SUVd tap is steady (8
codewords, 5600-5900 RMS). Its points are exactly those of the HSF's CPt
(one constellation, no codec set, trn1d_gain 0.9988). This direction of
`netsim` adds no echo.

### B1 read the taps it arrived on, not the taps DATA runs on (2026-11-28)

**Fixed (`v34_rx_condition_b1_equalizer()`, `v34rx.c`).** Against the HSF
at 3429 baud our B1 rate check stepped calls down from 31200 to 21600, and
the HSF retrained on the 11.6 (2 of 2 earlier, 4 of 24 in the A/B below).
The line carried those rates. B1 was measuring the wrong equalizer.

B1's SNR was the normalized correlation of B1 with its template, through
the taps MP and E left. On the dense bauds DATA does not start on those taps.
B1 conditioning (§11.4.1.1.5/§11.4.1.2.5) replaces them with a 21-tap
least-squares fit over B1 itself. On one HSF line, with `NET_DELAY` 1-4:

| Reading | Range |
|---|---|
| B1 correlation (old) | 22.4-33.7 dB |
| Phase 4 TRN refit | 33.1-36.4 dB |
| B1 through the fitted taps (new) | 35.7-38.0 dB |
| Data mode at 31200 | 36.3-37.0 dB |

The low tail of the correlation set the rate.

B1's SNR is now the residual of that fit: B1 through the fitted taps against
the same targets. It is scaled by L/(L - T) (120 symbols less the fit's edges,
21 complex taps) for the noise the taps were not fitted to. It is rated on
the refit table with no slack, since it is the same estimator. In loopback
(`v34_duplex_test`, 3429 and 3200 baud, injected noise 28-40 dB, delays 0/23)
it reads 0-1.9 dB above the old correlation, never above the injected
noise, and within 0.7 dB of data mode where data mode reported.
`ME_V34_B1_FIT_SNR=0` restores the correlation.

The fit reads about 37 dB on this line, which is this receiver's own
saturation. There one B1 asked a fall-forward to 33600. Every 33600 on this
line was lost within about a second: 5 B1 fall-forwards, plus 2 calls that
trained there and were renegotiated down. So a fall-forward driven by B1 now
stops at 31200 at 3429 baud. The data-mode fall-forward can still go higher.
3200 baud keeps its top rate (31200): `engine_pair_test`'s clean line reads
38.2 dB on B1 and confirms 31200 at 37.5 dB.

**A/B (D-Modem rig, plain V.34, HSF calling, `NET_DELAY` 1-4, 24 calls per
arm, arms alternated):**

| | Fitted B1 | Old correlation |
|---|---|---|
| Connected, data exact | 22 | 21 |
| Step-down retrains | 0 | 4 (two to 21600) |
| B1 fall-forwards to 31200 confirmed | 7 of 8 | none asked |

The fitted arm mostly ends at 31200. The correlation arm mostly ends at
28800 or 26400 after its step-downs.

The other failures:
- **Phase 2 INFO1c miss** (fixed below, "INFO1c came inside our L1/L2 window"): 1 fitted call and 2 correlation calls.
- **An 11.6 whose MP we never locked:** 1 call in each arm. In the fitted
  arm the HSF answered our S and sent 4800 bauds of TRN. Our Phase 4
  receiver locked its MP only at baud 1999, read a reserved bit wrong, got
  no E, and the §11.5 retrain that followed never completed. That is our
  11.6 MP receiver, open.

### Our §11.6 opened before the HSF was listening (2026-11-28)

**Fixed (`v34_own_reneg_held_locked()`, `modem_engine.c`).** The HSF
retrained on every §11.6 we opened in data mode: the B1 step-down, and the
B1 fall-forward after the retrain that followed. That was 6 of 8 plain-V.34
calls with the HSF calling (`/root/netdiag/v34_final.sh` on CT 128,
`NET_DELAY` 1-4, two calls per delay). It also cost the 57 s `hsf_v92_alaw`
call above.

It was not our signal:
- **D-Modem's own modem.** It opens a §11.6 5 s into data
  (`DSP_V34_RENEG=5000:8`). The HSF answers it with its own S about 100 ms
  later, 2 of 2, renegotiated 33600/19200, data exact.
- **Our S on the wire** (`WAV=` recordings). 128T, about 37 ms at 3429 baud,
  the same spectrum as open8's and the same level relative to data. The HSF
  never answers it with an S.
- **Our probe at 5 s** (`ME_V34_RENEG_AFTER_MS=5000`, B1 actions off). 4 of
  4 renegotiate with no retrain.

It was the timing. The B1 rate check and the B1 fall-forward both run the
instant our receiver enters data mode. On these calls that is also the
instant our transmitter finishes B1: flow log, `Tx - B1 complete` and our
`transmitting S` at the same `tx_t` (7.734 s, 7.747 s). So our S followed
B1 with no data in between. 150-200 ms later we read "far end abandoned
(60 ms silence)", and then its retrain. V.34 §11.6 allows a renegotiation
"at any time during data mode". The HSF only arms its S detector after a
few hundred ms of our data. Probes at 100 and 200 ms were retrained on 5 of
6 calls. Probes at 300, 500, 1000, 2000 ms and 5 s were answered on 18 of
18 calls.

Our §11.6s now wait until our transmitter has sent 500 ms of data after B1
(`v34_get_tx_data_ms()`, `ME_V34_RENEG_MIN_DATA_MS`, 0 restores the old
timing). The hold is counted on the transmit side, which is what the far
end hears, so line delay does not change it. Only the B1 rate check and the
fall-forward are held. Renegotiations for a receiver that has stopped
decoding are not.

**Result (24 calls):** 23 connect with data exact both ways. The other one
is the known Phase 2 INFO1c miss (below). We opened 19 §11.6s of our own:
- All 15 to 26400 or 28800 completed with no retrain. That covers 13 B1
  step-downs, plus the 2 fall-forwards up from 21600.
- Both step-downs to 21600 at 3429 baud got a retrain from the HSF. Unlike
  before, the HSF answered our S and sent TRN, and it retrained during the
  MP exchange. The retrain settled at 21600, and the fall-forward then took
  both calls to 28800 (one confirmed at 36.8 dB). The underlying issue was
  B1 under-reading this line. Fixed: see "B1 read the taps it arrived on,
  not the taps DATA runs on" above.

The offline matrix with the hold is 21 of 21, and `test-fast` passes.

Live on v90ras (deployed a03e6087), the HSF called in plain V.34 over SIP.
Data started at 26400 at +21.0 s. Our fall-forward to 31200 went out at
+32.0 s and was confirmed at 37.2 dB, with no retrain, and PPP passed
20/20. A second call missed INFO1c in Phase 2 and stalled in INFOMARKSa.

Method note: with `ME_V34_RETRAIN_ON_LOSS=0` the probe's MP is not capped
by B1, so the HSF sends 33600 on a line that carries less and our data goes
white. That is the setup, not a receiver fault. Grade the timing by
"restart Phase2" against "reneg probe" instead.

### INFO1c came inside our L1/L2 window (2026-11-28)

**Fixed (bb4be57b, `v34rx.c`; answerer, plain V.34 only).** Every live
plain-V.34 call from the HSF to v90ras stalled in Phase 2: 8 of 8 across
a03e6087 and c2191054. Offline it was about 1 call in 8.

What happened on the live taps:
- Our `POST_L2_WAIT_TONE_B` timed out with no Tone B, and the HSF missed our
  reversal. (Falling through here is our known deviation from §11.2.2.2.3.)
- The HSF ran its own §11.2.2.1.4 recovery and probed late. L1 came at
  +9.90 s, then an L2 of about 3 blocks, then INFO1c from +10.14 to +10.32 s.
  §11.2.1.1.7 lets the call modem end L2 as soon as it has Tone A and its own
  echo, so a short L2 is legal.
- Our receiver framed L1/L2 as a fixed 21 × 20 ms window, which swallowed
  INFO1c. The §11.2.2.2.4 deadline (2000 ms + 2 RTD) then sent INFOMARKSa at
  +10.40 s. INFOMARKSa replaces the Tone A the HSF was waiting for, so both
  ends deadlocked.

The fix:
- In the answerer, the INFO search runs during L1/L2 and accepts INFO1c
  there. A frame there could only be INFO1c.
- The probe measurement stops at the end of L2. That is when 1200 Hz, which
  is empty in the probe (§11.2.1.2.6) but carries INFO1c and Tone B, rises
  above half the mean probe-tone magnitude.
- Any block whose empty bins (900/1200/1800 Hz) sit more than 12 dB above
  L1's median is skipped.
- These gates are answerer only. In the caller our own Tone B echo tripped
  them, which moved INFO1c's 3429 max N from 12 to 14 on the Apple coupler
  test.

Probe SNR measured on the live taps, replayed:

| Tap | Before | After |
|---|---|---|
| Failing taps, with INFO1c framed | -6.7 dB (0 bps) | 46.4 dB (31200) |
| Same, with the end gate only | 19 dB | |
| Call 384470 (L1→L2 step block included at an 18 dB gate) | 29.6 dB | 44.7 dB |
| Passing call 384644 | 42.0 dB | 42.0 dB, nothing skipped |

INFO1a goes out after 1158 of the 1200 bauds the deadline allows. That
leaves 70 ms, and it is deterministic. Offline (CT 128): HSF plain V.34 24
of 24 (12 at the default baud, 12 at `ME_V34_BAUD=3200`), and the full
matrix 21 of 21. The late probe never happens on the rig, so the new path
did not fire there.

**Live after deploy (5 calls).** Phase 2 completed on all 5 (before: 0 of
8). Each call took INFO1c at +10.34 s, sent INFO1a, and ran Phase 3.

**Next failure: the HSF retrained during our Phase 3 J.** Fixed the same
day; it was ours (next section). The note here once called it "the older
peer Phase 3 equaliser gap". It was not.

### Our 27 ms preamble was Tone A before INFO0a (2026-11-28)

**Fixed: one point before INFO0a from every answer modem**
(`transmission_preamble_init()`, V.34 §10.1.2.3.1: "Each INFO sequence is
preceded by a point at an arbitrary carrier phase").

Live plain V.34 from the HSF had two Phase 2 problems: the INFO1c miss in
the previous section, and the HSF retraining during our Phase 3 J. Both
came from one event, visible on the receive tap of every failing call:
- SpanDSP sent 16 bauds (27 ms) of unmodulated carrier before INFO0a. From
  the answer modem that carrier is Tone A arriving ahead of INFO0a.
- That is the §11.2.2.1.1 trigger ("detects Tone A before receiving
  INFO0a"), so the HSF repeated INFO0c. The tap at 1200 Hz, decoded as DPSK,
  shows two INFO0c frames straight after 65 ms of Tone B, the second with
  bit 28 set (it had our INFO0a by then). Then the HSF went silent.
- Our Tone B detector counts a reversal on the first "1" after 20 steady
  bits. The repeat's fill ones ("1111") satisfied it. On 404130 the
  "reversal" came 7 ms after our own first reversal. The real answer cannot
  come sooner than 40 ms plus the round trip (§11.2.1.1.3), and the round
  trip here is about 58 ms. On 404111 it came before we had reversed at
  all, and we skipped FIRST_A.
- So our second reversal and L1/L2 went out before the HSF had answered our
  first reversal. The HSF framed our probe wrongly.
- Its INFO1c then gave 4800 bps at 2400 baud and 0 bps at every other
  symbol rate, so INFO1a picked 2400 baud (21600 projected). Phase 3 at
  2400 is where it retrained. When the HSF's recovery probe came late, that
  was also the INFO1c-inside-L1/L2 case.

Offline (netsim) the HSF hears INFO0a promptly enough not to take the
recovery, which is why only the live calls showed it. The V.90 digital
answerer already sent one point. That was found on 2026-11-16 against the
Courier, which took V.90's §9.2.2.2.1 recovery on 5 of 12 calls with
16 bauds and 0 of 12 with one. The plain V.34 answerer had been left at 16
"until measured".

**Live A/B on v90ras (HSF plain V.34 to 2950, 5 calls per arm):**

| | Preamble | Result |
|---|---|---|
| bb4be57b | 16 bauds | 0 of 5. Each repeated INFO0c, went to 2400 baud, and retrained in Phase 3 |
| bb4be57b + `ME_V34_INFO0_PREAMBLE_BAUDS=1` | 1 baud | 5 of 5 connected, PPP 20/20 echoes, 20/20 pings, 0 bad frames |

In the 1-baud arm every call showed:
- one "reversal 1 in tone B", 120 ms after our reversal;
- no FIRST_A skip, no INFO1c inside L1/L2, and no restart;
- INFO1a at 3200 baud, 31200 both ways.

The default now gives every duplex answer modem one point. Before, only
V.90's digital role had it. Half-duplex (fax, §12.2) keeps 16 bauds: with
one, `fax_class2_test --engine-class21` (slow.list:40/41) stopped
negotiating V.34 against our own receiver.

**Offline (CT 128):**
- HSF plain V.34: 24 of 24.
- Full matrix: 20 of 21.
  - The miss was `hsf_v92_ulaw`. An alternated A/B against the committed
    binary over the two HSF V.92 rows gave 10 of 12 against 9 of 12. Both
    rows fail about one call in five either way, so that flake predates
    this change.
  - Do not A/B this with `ME_V34_INFO0_PREAMBLE_BAUDS=16` as the old arm.
    That knob overrides every role, so it puts the V.90 digital role back
    to 16 too.

Still open from this:
- **The caller role still sends 16 bauds.** That is Tone B before INFO0c,
  which can trigger the answerer's §11.2.2.2.1 the same way. Unmeasured.
- ~~The Tone B reversal detector still takes INFO fill ones as a
  reversal; no §11.2.2.2.1 outside K56flex.~~ Fixed, see the next section.
- **Correction: this change broke our own call modem.** Every
  `engine_pair_test` V.34 and K56flex row fell back (38 suite lines): our
  caller heard none of INFO0a. The suite runs before committing did not
  show it. Cause and fix in the next section. Real callers were not
  affected (live 5/5, matrix 20/21).

### Repeated INFO0c, the Tone B reversal, and our early CJ (2026-11-28)

**1. Tone B reversal confirmed by the bits after it** (v34rx.c, plain duplex
answer modem only; V.90 and K56flex keep the old detector). A "1" after 20
steady bits is held as a candidate. Two ones in the next three bits (signal
present) is an INFO frame's fill "1111" (§10.1.2.3.1), not §11.2.1.1.3's
reversal, and is dropped. Three steady bits, or the signal dropping (the
call modem goes silent 10 ms after its reversal), confirm it.
`tone_ab_hop_time` keeps the time of the phase step, and the transmitter
starts §11.2.1.2.5's 40 ms count that many bauds in
(`tone_b_reversal_age()`), so our second reversal is still 40 ms after
the peer's.

**2. §11.2.2.2.1 for the plain V.34 answer modem.** The receiver keeps
framing INFO0c in TONE_B while the first INFO0c's INFO0_OK sits in the
event slot. Before, the search was off then, so a repeat was never decoded.
A repeated INFO0c with bit 28 clear makes us repeat INFO0a with bit 28 set;
one with bit 28 set, or Tone B after it, sends us back to Tone A and
§11.2.1.2.3 (`k56_repeated_info0c_reranging()`, now also for plain V.34,
and a new INFO0_RETRY exit).

Replaying `line0-404130` and `line0-404111` (receive taps, `--from 0`,
`ME_V34_INFO0_PREAMBLE_BAUDS=16` to match what we sent live):
- **Before:** "reversal 1 in tone B" 20 ms after our own reversal, then
  the Phase 3 retrain, as live.
- **After:** "phase step opened an INFO frame", then the repeat decoded
  (bit 28 clear), INFO0a repeated, the second repeat decoded with bit 28
  set, and Tone A again. After our repeated INFO0a the replay diverges from
  what was sent, so it cannot judge the rest.

**3. Our answerer started INFO0a 33 ms early; our caller was deaf for it.**
Found because 07de727e failed 38 suite lines (above). SpanDSP's V.8
answerer accepts CJ on two of its three zero octets, stops JM there, and
the engine's 75 ms silence (§11.2.1.2.1) runs from that point. So INFO0a
began 42 ms after CJ had really ended. Our call modem only starts V.34,
and its receiver, after its own V.8 §8.2.2 silence. That breaks §11.2.1.1.1,
which conditions the receiver *during* that silence. The 27 ms preamble
had covered the gap.

Fix (v8.c): still accept two octets, but hold JM until the third octet's
33 ms have passed, or the third zero octet or a carrier drop ends CJ. This
moves INFO0a on every call we answer 33 ms later, to where V.8 puts it.
~~Our caller's deafness during its silence is still there; it now has the
same margin a conformant answerer gives it. Not fixed here.~~ Fixed the
same day, below ("Our caller listens during its V.8 silence").

**Results:**
- `make test`: everything passes except slow.list:67, which fails on HEAD
  too. Before the V.8 fix the same 38 lines failed on HEAD and on the
  detector change alone.
- D-Modem matrix (CT 128): 21 of 21, the first clean sweep. A run before
  the V.8 fix was 18 of 21; an alternated A/B of the three failing rows was
  9 of 9 with the change against 8 of 9 on 07de727e, so those were flakes.
- HSF plain V.34 (`v34_info1c.sh`): 23 of 24. The miss reached DATA, then
  failed a §11.6 renegotiation (no E) and timed out in the §11.5 retrain.
  Its Phase 2 was clean.
- Loopback `V34_DUPLEX_DELAY` 0-47 (`v34_duplex_test`, which bypasses
  V.8 and the engine, so it tests changes 1 and 2 only): 3429/28800 at
  36 dB and 3429/31200 at 38 dB are 48/48 each, the same as HEAD.

### Our caller listens during its V.8 silence (2026-11-28)

V.34 §11.2.1.1.1 has the call modem condition its receiver for Tone A and
INFO0a during V.8's post-CJ silence. Our V.34 receiver only exists after
it: V.8 §8.2.2 reports success 75 ms after CJ ends. A conformant answerer
starts INFO0a 75 ms after it *detects* CJ (§8.2.3), so the margin was the
round trip alone, and at a short round trip INFO0a's first bits arrived
as our receiver started.

The V.90 originate path already replayed the quiet stretch after JM into
the new Phase 2 receiver (`me_v90_prepare_info0a_preroll()`, from
0cdc64b1). `start_v34_training()` now does that for every call we
originate: plain V.34 and K56flex as well as V.90. In plain V.34 every
Phase 2 timestamp the caller compares is on the receive clock, so the
replay's offset against the transmit clock cancels. The only mixed-clock
comparison, `v90_tone_a_reversal_recovery_due()`, is V.90-only and
already ran with the replay.

**Results:**
- Our caller against our own answerer, with the 3cc6b3c5 JM hold taken
  out again so INFO0a comes 33 ms early: 34 of 240 `engine_pair_test`
  lines fail without the replay, 0 of 240 with it.
- D-Modem matrix (CT 128): 20 of 21. The miss is `hsf_v92_ulaw`, where the
  HSF calls us and the replay never runs. An alternated A/B of that row
  gave 2 of 3 on each arm: a flake.
- Live, from CT 117 to v90ras (2950) through Asterisk, `ME_MODE=v34`: 2 of
  2 CONNECT 31200 with v90ras's LCP arriving. The replay was skipped on
  both ("no post-JM gap"), as it should be. The quiet stretch we hear is
  75 ms less the round trip, so when it is under the 20 ms the search
  needs, INFO0a also arrives that much after our receiver starts.
- The RasFinder no longer answers on 6311 (receive silent) or 6312 (V.8
  times out), so there is no third-party V.34 answerer for this live.

### Our call modem: one-point preamble and §11.2.2.1.1 recovery (2026-11-28)

**Preamble.** The call modem sent 16 bauds (27 ms) of carrier before
INFO0c, which is Tone B before INFO0c: the answer modem's §11.2.2.2.1
trigger, the mirror of what 07de727e removed from our answerer. Every
duplex role now sends the single point of §10.1.2.3.1. Half-duplex (fax)
keeps 16, for the reason given at 07de727e.

**§11.2.2.1.1 was missing.** Our call modem had no way to notice a missed
INFO0a. Its receiver had no Tone A detector during the INFO0 search, so
after a lost INFO0a it sent Tone B for the rest of the call while the
answer modem waited in Tone A for its reversal. Measured with the JM hold
and the preroll both removed: 34 of 240 `engine_pair_test` lines failed
that way. Now, for the plain duplex call modem:
- The receiver counts Tone A (20 zero bits on a real carrier, outside a
  frame) before INFO0a, and the transmitter repeats INFO0c.
- INFO0c carries bit 28 = 1 once INFO0a is in (the clause's NOTE). It
  leaves the repeat for Tone B on INFO0a with bit 28 set, or on Tone A
  with INFO0a in.
- Waiting for the reversal, a repeated INFO0a gets INFO0c again only while
  the answer modem has *never* shown bit 28 = 1. A Broadcom answerer that
  had acknowledged us still sent one more INFO0a with bit 28 clear, and
  answering it with INFO0c lost 3 of 6 calls: our INFO1c then reported 0
  bps at every symbol rate. With the guard, 6 of 6.
- The first Tone A reversal is confirmed by the next three bits, like the
  answer modem's Tone B reversal: two ones mean a repeated INFO0a's fill.
  The 40 ms to our reversal still runs from the reversal itself.

**Our K56flex answerer only half-answered a repeated INFO0c.** With bit 28
clear it went straight back to Tone A instead of repeating INFO0a, so a
call modem that had missed INFO0a never got one. It now repeats INFO0a,
as the plain answerer has done since a4a5fe2c. The MT5634MU repeat that
branch was written for had bit 28 set and still takes the Tone A path.

`V34_TEST_DROP_INFO0A=N` makes the plain call modem discard its first N
good INFO0a frames. It corrupts two bits, because the single-bit CRC
repair otherwise put a dropped frame back by flipping a bit the CRC does
not cover.

**Results:**
- JM hold and preroll removed, recovery only: 240 of 240 `engine_pair_test`
  lines (34 failed without it).
- `V34_TEST_DROP_INFO0A=1` against our answerer: plain V.34 33600 and
  K56flex 52000 both connect through the recovery (now in fast.list).
- D-Modem, our call modem against vendor answerers (trial rows
  `we_*_v34_ulaw`), with the first INFO0a dropped: HSF 3/3, Agere 3/3,
  Broadcom 6/6, each through the INFO0 retry. The Intel and SmartLink rows
  never offered V.34 with the AT strings tried, and ESS reaches 19200 but
  corrupts data with or without this change, so those rows are not kept.
- D-Modem matrix on the final build: the 21 original rows 21/21, and the
  three new `we_*_v34_ulaw` rows (now in `dmodem_matrix.sh`) 3/3.
- Live, CT 117 to v90ras (2950) through Asterisk, `ME_MODE=v34`: 3 of 3
  CONNECT 31200 with LCP, the third with `V34_TEST_DROP_INFO0A=1`, which
  went through the INFO0 retry.

### ESS as V.34 answer modem: B1 from the scrambler carried on from E (2026-11-28)

`we_ess_v34_ulaw` (our plain V.34 call modem, the ESS answering) reached
data mode and corrupted it: CONNECT 19200 on our side and 14400 on the
ESS's, 85359 octets received for 7340 sent. Our receiver never decoded the
ESS. B1 correlated 0.19-0.28 at every offset, trellis state and
switching-pattern phase, at every symbol rate we retrained to
(3429, 3200, 3000, 2800). The data mode then read white (distance to grid
0.66) at 24-36 dB receive SNR, and the §11.6 renegotiations that followed
did the same. The ESS received our data.

V.34 §10.1.3.1 resets the scrambler (and trellis encoder, differential
encoder and precoder) before B1. The ESS does not: its scrambler runs on
from E, the same thing its V.90 analogue modem does into the upstream B1
(21da3dfa). The descrambler is self-synchronising, so at the end of E our
register is the last 23 scrambled bits received, which is the far end's
scrambler register. `v34_rx_b1_search()` now builds its templates from that
register once, when no reset-state template reaches 0.9 (plain V.34 only;
V.90 keeps `v90_t3_carried_b1()`). From the carried state the ESS's B1
correlates **1.000** at offset 0, often from trellis state 4 (it does not
reset the trellis encoder either), against at most 0.259 from the reset.

**Results:**
- `we_ess_v34_ulaw`, alternated against 815fd52a: 2 of 2 CONNECT 33600
  with data exact both ways, against 0 of 2 (CONNECT 19200, corrupt).
  Every B1 in those calls, the startup's and each renegotiation's, fitted
  at 1.000.
- D-Modem matrix with `we_ess_v34_ulaw` added to it: 25 of 25. The carried
  state was tried only in the ESS row (3 B1s, each 1.000); every other
  row's B1 fitted from the reset state, so their path is unchanged.
- `make test`: 814 of 816. slow.list:67 fails on HEAD too. slow.list:24
  (K56flex against the native HSF) lost upstream octets on one pair: 32000
  here, 42000 in an earlier run today, with training complete and downstream
  exact. That test acquires B1 on the T/3 path, so this search never runs
  in it (no "B1 found" line), and the pairs passed 6 of 6 on rerun. Open:
  an intermittent upstream data miss there, not yet investigated.

### Our CI ran into the answerer's ANSam (2026-11-28)

`we_intel_v34_ulaw` and `we_sl_v34_ulaw` are our plain V.34 call modem
against Intel's and SmartLink's answer modems, at 9600 Hz with no loop, as
D-Modem's own rows run them (`NET=8000:9600`, `INTEL_RATE=9600`; the peers'
AT strings are D-Modem's, `AT+MS=V34` and `AT+MS=34`). Neither connected.
SmartLink logged "Time Out Waiting For CM", and we never decoded a JM
("modulations=none"). Intel answered, but our receiver never decoded its
INFO0a.

Both were our CI. V.8 8.1.1 has the calling modem send CI until it detects
the answer tone. Our ANSam detector needs 550 ms of steady 2100 Hz (or three
450 ms reversal cycles) before it says so, and our CI bursts went on through
that time: the V.25 4.3 hold ended at 3.0 s, and four bursts went out on top
of the answerer's ANSam. With `ME_V8_NO_CI=1` SmartLink connected at 33600,
data exact. It is the same failure the 4.3 hold already avoids for slmodemd,
whose ANSam started under our second burst.

Fix (`v8.c`, `V8_CI_OFF`): no new CI burst starts while the detector's notch
test says 2100 Hz dominates the line (`tone_on`), for at most 2 s, which
returns a tone that never resolves to CI.

**Results:**
- `we_intel_v34_ulaw` and `we_sl_v34_ulaw`: CONNECT 33600, data exact both
  ways (0 of 2 before).
- D-Modem matrix: 25 of 27, with both new rows passing. The misses were
  `hsf_v92_alaw` and `ess_v90_ulaw`, where we are the answer modem, so this
  change cannot reach them. Both are fixed in the next two sections.

### V.92: our CPd went unacknowledged forever (2026-11-28)

`hsf_v92_alaw` hung for 50 s. One of the two HSF V.92 rows also failed in
each of two earlier matrix runs that day. The CPd deadline (below,
2026-11-27) covers a TRN2u that no CPd can be designed from. In this call
the CPd was designed and sent. The HSF sent 22 SUVu with ack = 0, then
nothing we framed, at a steady RMS of 3469. We sent SUVd until it hung up.
§9.6.1.1.3 repeats the CPd only on a received unacknowledged SUVu/CPu, and
none came.

Fix (`modem_engine.c`, `v92_cpd_deadline_expired_locked()`;
`v90_v92_awaiting_cpd_ack()` in `v90.c`): the deadline (5 s plus 5 round
trips) also bounds a CPd that is out and unacknowledged. It counts from
when either wait begins, so repeated CPds and unacknowledged SUVu do not
reset it. It ends in the same §9.5.1.1 retrain without the PCM upstream
offer. HSF V.92: 6 of 6 over three runs (V.34 26400). The new path fired in
one of them and recovered.

### V.90: our Rd waited for a pull that started on a data frame (2026-11-28)

`ess_v90_ulaw` connected and then lost data: 4096 of 6606 octets
downstream. The call then retrained down to 16800, where it read white. ESS opened a §9.6
renegotiation (S of 40 ms, 128T, at 229/1829/3429 Hz) and we logged "Rate
renegotiation requested; starting at the next data frame boundary". No Rd
followed for 4.5 s, until ESS retrained. In a passing run of the same row
the Rd went out at once.

§9.6 starts Rd only on a data frame boundary. The engine checked for one
only at the start of a transmit pull, and only V.92 first ran the mapper on
to the next boundary. A V.90 pull rarely ends on a 6-symbol frame. With
pull sizes whose phase never brought a boundary round, Rd never went out.
Now V.90 runs the mapper to the boundary too, then starts Rd
(`generate_v90_raw_codewords_locked()`). ESS V.90 and V.92 μ-law: 12 of 12
over six runs, and every request sent its Rd (7 requests, 7 Rd).

**D-Modem matrix with all three fixes: 27 of 27**, data exact on every row.
That is the first full sweep that includes the Intel and SmartLink V.34 rows.

**K56flex against the native HSF (slow.list:24), the miss noted above:**
`test_k56flex_rates.py --peer native-hsf` passed 45 of 45 over three runs at
`-j3`. It has shown only under the full suite's load. Still open.

Solved the same day, and it was the test. Under 8 busy-loop processes it failed
1 pair in 30 (58000 ceiling row). The upstream there was 8206 bytes against
8192 sent: the 8192 payload octets, exact, then `\r\nNO CARRIER\r\n`. The carrier
had held the whole 40 s. `k56flex_hsf_pair.py` drained the PTY one last time
only after it had closed the audio socket, which is its own hangup. Under load
the engine's NO CARRIER for that hangup got in ahead of the drain. The last
drain now runs before the socket closes. Under the same load: 45 of 45 over
three runs.

### Our answer modem never re-reversed Tone A (11.2.2.2.2) (2026-11-28)

HSF calling us in plain V.34 (`/root/netdiag/v34_final.sh` on CT 128,
`NET_LOOP=0.45`, delays 1-4): 18 of 20 calls connected, then 39 of 40. Every
failure ended the same way. Our §11.6 got no E: either the data went white
at 33600 and we stepped down, or B1 fell forward to 31200. The §11.5 retrain
that followed then stalled with our transmitter in FIRST_NOT_A. We had sent
the Tone A phase reversal of 11.2.1.2.3, the HSF's Tone B never reversed in
answer, and we waited 89 s until the call was dropped. That was 3 of 3 such
retrains.

§11.2.2.2.2: "If, in 11.2.1.2.4, the answer modem does not detect the Tone B
phase reversal within 2000 ms, the answer modem shall condition its receiver
to detect Tone B and then proceed according to 11.2.1.2.3." So it sends the
reversal again. We already did this for the V.90 analogue modem (V.90
9.2.2.2.2), but not for the plain V.34 answer modem. Now both do (`v34tx.c`,
FIRST_NOT_A). The log line is WARNING level, "reversing Tone A again
(V.34 11.2.2.2.2)".

**Results:**
- The same 40 calls: 40 of 40. Two of them retrained, one after our §11.6
  got no E. In that one FIRST_NOT_A lasted about 2.5 s (the 2000 ms plus a
  round trip), then the HSF reversed and the call reached data. Before, all
  three such retrains stalled.
- Raising the 3429-baud 33600 threshold by the 0.5 dB margin (31.5 to
  32.0 dB) was tried and reverted. The two white-at-33600 calls read
  31.7 dB, but our own clean loopback (`engine_pair_test --fdm-slot 5`,
  fast.list:806) reads 31.4-31.8 dB too and is clean at 33600, so the
  threshold cannot tell them apart. The B1-style refit reads 34.5-35.4 dB
  (31200) in both.
- D-Modem matrix: 26 of 27. The miss was `hsf_v92_ulaw`, the known HSF
  V.92 fallback flake: it retrained twice on the V.90 path, which this
  change does not touch, and the new rule never fired. `make test`: 815 of
  816; slow.list:67 fails on HEAD too.
- Still open: why our §11.6 after a fall-forward or step-down gets no E
  from the HSF (about 1 call in 20). With `ME_V34_SPAN_FLOW_LOG=1` it did
  not reproduce in 28 calls; the logging changes the timing. Found the next
  day, below.

### Our §11.6 receiver took the HSF's TRN for "no TRN" (2026-11-28)

The ~1 in 20 §11.6 with no E from the HSF. Reproduced with
`ME_V34_RENEG_AFTER_MS=600` (an 11.6 600 ms into data, every call) from
`/root/netdiag/v34_rn20.sh` on CT 128, 20 calls at `NET_DELAY` 1 and 2.

The cause was our receiver. 11.6.1.1.1 and 11.6.1.2.2 make TRN optional, so
`v34rx_phase4_trn.c` decides at 512 bauds into PHASE4_TRN whether the peer sent
any. If it decides "no", it starts the MP search on the symbols it has. The
test was the TRN ones-lock score below 80% on symbols cleaner than 20 dB. That
score is cumulative from the first scored symbol, so a receiver that took a
few hundred symbols to converge carries them in it. On 3 of the 20 calls the
HSF's real TRN read 78-79% overall while the recent window read 100% (the new
WARNING line, below). We left TRN for the MP search there on an equalizer half
trained. As initiator we send no TRN of our own (`ME_V34_RENEG_INIT_TRN_MS`
defaults to 0), so the HSF's TRN is the only training we get. One of the 3
then never decoded the HSF's MP: no E, then a §11.5 retrain.

The fix: the "no TRN" verdict also needs the recent window
(`phase4_trn_recent_score`) below 80%. A peer that really sends no TRN still
gets it: MP reads 50-64% in both scores (duplex responder after an initiator
with no TRN: 66% cumulative, 50% recent).

- Before, the same 20 calls: 3 false "no TRN" verdicts, 1 no E, 1 §11.5
  retrain. After: 0, 0, 0, and 20 of 20 connected.
- The verdict is now a WARNING line, "11.6 with no TRN (ones-lock N%, recent
  N%, on N dB symbols at baud N)", so a live log shows it.
- `v34_duplex_test` takes `V34_DUPLEX_RENEG_BY=answer`, for an 11.6 opened by
  the answer modem (v90ras's case). Three such rows are in fast.list. The
  false verdict does not reproduce in loopback: our receiver converges on our
  own TRN inside the first window.
- `make test-fast` 790 of 790. The 11.6 rows, the cleardown rows, the
  no-TRN test hook rows and the V.34 fall-forward pair rows: 33 of 33.

### V.92 against the HSF: retrain when no CPd can be designed (2026-11-27)

**Both HSF V.92 rows pass, by falling back to V.34.** `hsf_v92_ulaw` and
`hsf_v92_alaw` connect 26400 down / 24000 up with data exact both ways, 8 of 8
over `NET_DELAY` 1-4 ms (about 34 s to CONNECT, one call 57 s: below). Before,
0 of 8: every call failed at 53 s.

At our −6 dBm0 ceiling the HSF's TRN2u is chance (above: sigma 600-860 DS0),
so `v90_build_v92_cpd_frame()` designs nothing. §9.6.1.1.2 has us answer each
SUVu with one CPd, and nothing in the clause ends the wait when there is none
to send. So we sent SUVd for about 50 s, receiving the HSF's steady SUVu
(3469 RMS), until it hung up without a retrain of its own. The peer sends
nothing new to design on, so the wait could never end well. §9.5.1.1 lets
either modem retrain at any time.

The fix (`v92_cpd_deadline_expired_locked()`, `modem_engine.c`) starts a
§9.5.1.1 retrain when no CPd has been designable for 5 s after SUVu, plus 5
round trips (`ME_V92_CPD_DEADLINE_MS`, 0 disables). It shares the B1u
deadline's per-call cap of two. That retrain also withdraws the PCM upstream
offer (INFO1d bit 70) at once, without waiting for a second failed PCM
attempt. On the first version, which kept the offer, one call of the matrix
spent its second attempt on the same dead TRN2u and missed its CONNECT
window. On each of the 8 calls the deadline fired exactly once. The HSF then
declines PCM in its next INFO1a, or, in one call (A-law, 2 ms), tries V.90,
fails it on its own −6 dBm0 constellation, Tone A-retrains and then declines.
That call took 57 s, because the HSF also retrained on our §11.6 B1
renegotiation in V.34 (28800 to 26400), a known plain-V.34 item (above).

A retrain cannot lower the power ceiling. §9.5.1.1-.2 omit INFO0, so INFO0d's
bits 33:37 stay what the call declared. A retrain can only give the peer
another try, or its fallback.

None of the other 19 offline rows fired the deadline.

### ESS V.90 carries its scrambler and trellis state into B1 (2026-11-26)

**Fixed in our upstream B1 search (spandsp `v34rx.c`, `v90_t3_carried_b1()`).**
`ess_v90_ulaw` lost octets in about 1 call of 3, short or extra, on every
build. The ESS's B1 matched none of our templates: coarse 5-9% and fit
17-23%, which is chance. Every other peer fits 100%. So every ESS B1 was
placed blind from the 4-point CP′/E (`v90_t3_blind_acquire()`), with the
"+9 symbols" gap measured on the MT5634. The ESS renegotiates (§9.6) 1-4
times a call, and each time B1 was placed blind again. Those placements
sometimes decoded white (49% ones) until LAPM's N400 ran out, and the call
lost its tail.

V.34 §10.1.3.1 resets the scrambler, trellis encoder, differential encoder
and precoder before B1. The ESS resets none of the first two:

- **Scrambler.** It runs on from CP′/E. The 4-point CP′/E is differentially
  coded (§10.1.3.3: I_n = Z_n − Z_{n−1} mod 4, rotated clockwise). So the
  quadrant step between two of our decisions is the pair of scrambled bits,
  whatever the carrier phase. The last twelve symbols give the 23-bit
  register (clause 7). A template seeded with it fits the ESS's B1 100.0%
  (coarse 93-99.8%), starting the symbol after the last 4-point symbol.
- **Trellis encoder.** At start-up it is in state 0. After a renegotiation
  it carries state 4 on from the data mode before. The right scrambler
  state with trellis state 0 scores 47-58% coarse; with state 4 it scores
  93.7-99.8%, and the fit is 100.0%.

The search now reads that state, as if the 4-point signal ended anywhere
from three symbols before to eight after where it was found, in either
sense of rotation. It keeps the best coarse score (a wrong state scores
5-9%), tries the 16 trellis states when that score is under 90%, and fits
the winner. The spec's own reset state is one of the candidates. If the
fit is 0.95 or better, B1 commits through the normal template path,
including the out-of-sample check. Otherwise the blind placement goes on as
before. `ME_V90_T3_NO_CARRIED_B1=1` turns it off.

A second defect showed once B1 was exact. After a renegotiation the search
is anchored on CP′ and looks for the end of the 4-point signal in whatever
has arrived. The ESS is still repeating CP (ack=0) there until our MP
reaches it. The 4-point placement found no end, marked itself tried, and
every later window ran only the templates, which cannot fit this peer. The
§9.6 renegotiation then timed out ("no E within 56000 symbols") and the
call retrained, with CP′/E and B1 sitting in the ring. A 4-point signal
that runs off the end of what has arrived now counts as not yet ended
(`V90_T3_BLIND_OPEN_END`), and the next window looks again. That retry
needs the reset candidate. Once the 4-point placement succeeds, the
CP′-anchored search skips the templates. Without the reset candidate, our
own analogue role's fall-forward (`engine_pair_test` with
`ME_V90_UPSTREAM_TEST_FIRST_BPS=19200`) got a blind B1 after its
renegotiation, and its V.42 failed 3 of 3. Before the retry, a template had
found that B1 in a later window.

Results: `ess_v90_ulaw`, arms alternated against 52ccd4d0, 8 calls each.
- New build: 8 of 8. 17 of 19 B1s fitted exactly (fit=100.0%), and no §9.6
  timeout or retrain.
- 52ccd4d0: 7 of 8. All 20 B1s were blind (fit=0.0%), with 4 §9.6
  timeouts or retrains.

Over the four A/Bs the old build passed 15 of 20. The build with the
trellis search has passed 14 of 14 (including 6 runs of an earlier gate).

Our switch to MNP in one earlier failing call came from this too. A blind
B1 on the wrong frame phase decoded junk for ten sweep steps, and an "LR"
arrived after V.42 detection had already succeeded. The ESS then really did
complete MNP, and later received its own data back.

That "echo" never crossed the line. It is the ESS in command state, echoing
its DTE's characters (E1) after its own MNP link has failed. Four failing
`ess_v90_ulaw` runs on CT 128 (`fb/carry1`, `fb/abess_spec_8`,
`fb/ess_p3fix_1`, `dmi/frmat`) all show the same thing when the ESS's received
stream (`gotA`) is matched against both directions' data:
- `gotA` first holds our data in order: 3858, 3858, 2618 and 6239 octets.
- After that it holds the ESS's own transmit stream (`wantB`), unchanged and
  complete to the last octet. It starts mid-stream, at octet 1903, 1903, 1964
  and 4476.
- A single `\r\nOK\r\n` sits inside that copy, 810-1005 octets in. At the
  DTE's pace of 389 octets per 0.5 s, that is 1.0-1.3 s after the copy
  starts.
- `gotA` never shows NO CARRIER, so the ESS kept its carrier.

`interop_call` sends no `+++` with `CALLS=1`, and the data contains no `AT`.
We never send LN (mnp.c answers one with LNA and nothing more), and no LD
went out before our own N400 ran out about 12 s later. In every one of the
four, B1 after the §9.6 renegotiation was blind (fit 0.0%, the frame-phase
sweep ran through all 112 steps), so the upstream decoded white. We received
1042-3515 octets, all from before the renegotiation, and acknowledged
nothing after it. The ESS gave up its MNP link, went offline, and its
command-mode echo was graded as received data. So this is the same defect
as the blind B1, now fixed above, not a separate data-path problem. MNP
itself still connects in passing calls on the carried-state build
(`fb/abess_carried_*`).

### The emulated Courier (2026-11-27)

`courier_v34_ulaw` (both sides `ATX3`, so V.8 picks V.90) fails identically with
D-Modem's `half_open8` in our place: Phase 3 completes (open8 logs S after
163 Jd), the Courier sends Tone A, the V.34 retrain never finishes, and
`interop_call` gives up at 62.7 s. D-Modem's `interop_test.cpp` lists V.34 as
not yet working between two emulated Couriers and has no V.90 row for it. Its
Courier rows against `half_open8` are HST, V.32bis, V.22bis and V.FC; we can
run V.32bis and V.22bis. The Courier's S registers pick the modulation:
`ATS34=128S56=224S58=33` for V.32bis, `ATS27=4S56=224S58=33&N3` for V.22bis.
When we call, `interop_call` needs `DIAL=1`, because we answer a bare `ATD`
with ERROR.

**V.32bis: our CA carried no phase reversal.** With the Courier calling, every
call hung at 6.2's tone dialogue: the Courier sent AA (a clean 1800.0 Hz) for
60 s, and we sent AC/CA. Our trace showed AA detected and CA begun at symbol
163. `half_open8`, which sends a plain answer tone, gets AA first and passes.

`v32bis_tx_enter_phase()` set a phase's first symbol to the current index even
when the receive path entered it. The symbol it prepared went out on the next
symbol, and the next refill counted from two, so tone phases entered there
repeated their first state: AC went A, A, C, … and CA went C, C, A, …. After an
odd AC count the line carried A, C, C, C, A, C. That is one stray symbol, not
the reversal 6.1 has the call modem wait for. After an even count the
reversal came one symbol after the counter's start. The phase now starts on
the symbol it is first emitted as, and the AC to CA switch also waits for an
even count of AC symbols sent (6.2). On CT 128 with the Courier calling, 9 of
12 calls connect at 14400 with data exact, over `NET_DELAY` 1-11. The tone
dialogue completes in all 12. The other three never frame the Courier's R2
after its TRN, and then ignore its 7.1 retrain AA until the timeout.

**V.32bis: S found late, at the eye crossing.** The three calls that missed R2
all detected the Courier's S at rx symbol 226, against 77-78 in every passing
call. The Courier sends 6.1's NT probe S and its conditioning S back to back,
one S of about 350T (145 ms on the line, starting as our R1 begins). Our S
acquisition fits 64 symbols to alternating A and B through one complex gain.
In a failing call that fit's residual sat at 0.47-0.50 for 100 symbols while
the S was on the line at steady power. Then the power rose from 20 to 32 and
the residual fell to 0.13 by symbol 224. That is the signature of sampling at
the eye crossing: A and B average to a constant that fits one gain half way,
and the Godard timing loop took about 150 symbols to walk to the eye centre.
6.2 then holds the receiver for MT (124-130T) after detecting S. By the end
of the hold the S was over (residual near 1.0 from symbol 400), so we never
trained and never framed R2.

The S search now also fits the FSE output at the other T/2 phase, which the
V.17 receiver takes between bauds while V.32bis asks for it. If that window
fits and beats the current phase, the receiver processes the next T/2 output
as its baud, moving the sampling point half a symbol, and acquires there.
Arms alternated over `NET_DELAY` 1-8 against fdd95bdb:
- Old build: 31 of 32. The one failure detected S at 226.
- New build: 32 of 32. It moved its phase in 7 calls, and no S came late.
- The final build, with the restart below: 24 of 24.

**V.32bis: a stalled start-up now answers the far end's restart.** After its
R2 went unanswered, the Courier sent AA for 50 s (V.32bis 7.1's retrain tone),
and we sat in our S search. The clause 7 retrain watch ran only once start-up
was complete. The answer modem now also runs it during start-up, once its
tone phase is over, with two guards:
- The tone must measure 0.9 of a pure tone, not 0.7. The Courier's S carries
  0.46 of its power at 1800 Hz, which measures 0.68.
- It must come within 6 dB of the AA heard in the tone phase. In
  `v32bis_duplex_test`'s clause 7 retrains, the call modem's CC dies away as a
  pure 1800 Hz at rms 95, against about 3000 normally, and the unguarded watch
  restarted on that every time.

The call modem does not run this watch through its tone phase. That phase
ends at its second reversal, and the answer modem's AC rightly runs on until
it sees AA stop, past 128T on a long round trip. The RasFinder replay's call
modem took that AC for a restart. Since the next section, the call modem runs
the watch once a rate signal has arrived. `V32BIS_TEST_DROP_FIRST_R=1` (a test hook) ignores the
first R2 to force the stall. With it, two of two calls retrain on the
Courier's AA and connect at 14400 with data exact in 17 s.

### The emulated Courier answering our call (2026-11-27)

`dmodem_matrix.sh` now runs both directions: `courier_v32b_ulaw` and
`courier_v22b_ulaw` have the Courier call, and the `we_` rows have us call
(`DIAL=1`). Four defects were ours.

**V.22bis: we cut our own S1 short.** 6.3.1.1.1 b) has the call modem send S1
for 100 +-3 ms; the Courier then sends its S1 and goes to 2400. We ended the
unscrambled-ones stage on the first symbol that departed from the pattern.
Our own S1 starts while the Courier's ones are still arriving, so one wrong
phase decision ended the stage, and that switched our transmitter to timed
scrambled ones early: our S1 went out 75 ms long. The Courier refused it and
the call came up at 1200, then lost carrier. The stage now ends on 3
departures in the last 8 symbols, and an S1 already being sent completes
before the timed scrambled ones. With that, 16 of 16 calls trained at 2400.

**V.22bis: the Courier's S1 on our eye crossing.** A further run of 40 calls
lost 4 to 1200. In each, our decision samples of the Courier's S1 were 7-20
degrees apart for its whole 60 symbols, not the 80-90 of a passing call, so
the phase-change detector saw a run of 0-degree steps. The Courier's S1
follows its unscrambled ones, a pure tone with no timing in it, so the
sampling phase at S1's start is chance. The answer modem already searched the
other T/2 eye for S1 (the CX93001's case); the call modem now does too,
except while its own eye is in an S1 run. Replaying the receive side of one
call offline reproduces the fault (1200 before, 2400 after, the other eye's
run 61 long); it is the regression fixture
`reference-data/courier-v22bis-caller-s1-eye`. After the fix: 24 of 24 at
2400.

**V.32bis: a multi-rate R3 left us without E.** 6.2 has R3 name the data rate
to use. The Courier's R3 is its R1 again (0x9ff0), every rate both offer. We
took the rate only from a one-rate R3, so we never sent E. The Courier waited,
then retrained with AC. The call modem now picks the fastest rate both sides
offer, as D-Modem's call modem does against Agere's R3.

**V.32bis: the call modem answers a 7.2 retrain during start-up.** With E
missing, the Courier's AC held us 80 s in our S search, which took the AC for
S. The call modem now runs the 7.1 retrain watch during start-up, but only
after a rate signal has arrived, when no AC is left to come except a retrain's.
That avoids the RasFinder replay's long-round-trip AC above. Two guards keep it
off S and off silence:
- The 1800 Hz carrier must be absent, under 0.35 of the 600/3000 Hz lines.
  S alternates A and B a quarter turn apart, so half its power is at 1800 Hz,
  while AC alternates opposite states and has none. In loopback, a clean S
  passed the purity test, and the call modem restarted on the answer modem's
  second S.
- The signal must be above rms 50. The purity test is a ratio, so in
  `v32bis_duplex_test`'s 31 dB hybrid row the rms 3.5 residual, heard while
  the answer modem was silent after R1, read as AC.

**V.32bis: data that goes white now retrains (7.1/7.2).** Once connected, 14
of 108 calls with us calling went white 4-5 s into data and stayed white,
until LAPM gave up (T401 x N400) with the Courier still transmitting. A
retrain at 7 s (`ME_V32BIS_RETRAIN_AFTER_MS`) recovered all 16 tried. 7.1 and
7.2 let either modem retrain on "unsatisfactory signal reception". The data
receiver tracks its decision error against the nearest point, about 0.58 rms
when the decisions are white and 0.16-0.36 on a healthy 14400 call. Above
0.45 rms (power 0.2) for 2 s (4800T, D-Modem's call-modem hold), it retrains,
at most 3 times a call. Results:
- We call: 32 of 32, with 5 calls rescued by a retrain.
- Courier calls (V.32bis and V.22bis), and we call on V.22bis: 8 of 8 each,
  with no false retrains.
- Control: D-Modem's own caller against the Courier passed 6 of 6.

### The emulated Courier on a live SIP call (2026-11-27)

`tools/soak/half_sip_call.py` puts a D-Modem harness (`half_courier` here) on
a real SIP/RTP call from CT 128, with no netsim:
- It feeds the harness's fd 3/4 from a PCMU stream on a local 20 ms clock.
  The only conversion is the standard u-law tables: no gain, no resampling.
- It drives the harness's DTE as `interop_call` does: ATZ, the S-register
  string, then `ATD` once the call is answered (`ATA` with `--answer`).
- Calling, it INVITEs as 2903 (the u-law test account). Answering, it
  registers as 2902 and takes one call.

**The Courier calls v90ras (2950).** Each v90ras line answers into pppd, and
CT 128 has no pppd, so the bridge checks the data at the PPP layer itself:
- every frame's FCS;
- LCP, with the server's multilink options refused, so the link runs plain;
- IPCP, taking 10.254.200.2;
- 20 LCP echoes, then 20 ICMP pings to 10.254.200.1, each with a random
  100-byte payload.

The calls alternated, 5 per modulation: all 10 passed. V.32bis connected at
14400/ARQ in 11.3 s; v90ras logged its start-up complete at 14400 (MT=249).
V.22bis connected at 2400/ARQ in 8.7 s. None retrained, and there were no
bad frames.

One earlier V.32bis call did retrain, during the pings, and then completed:
the Courier sent AA after 120 ms of silence on its line. That silence was the
bridge's own fault. v90ras's RTP log shows its sending evenly paced (largest
gap 20.2 ms), while what it received from the bridge had one 54 ms gap. A
late tick catching up skipped reading the RTP socket and fed the Courier
silence. Fixed: every tick now polls the socket once.

**We call the Courier.** `half_sip_call.py --answer` registers the Courier
as 2902. `tools/soak/wecall_live.sh` runs our modem on CT 117 as 2903 (PCMU,
SIP port 5100) and dials it. After CONNECT each side sends 40 numbered lines,
and each counts the other's exact lines. Launch the bridge with the ssh
command itself in the background: `pct exec` waits for every process it
started, `nohup ... &` included, so a backgrounded bridge inside it held the
caller back until the bridge had unregistered, and Asterisk refused every call
(603, cause 16).

The calls alternated, 5 per modulation, on 669bf72c: all 10 passed. V.32bis
gave CONNECT 14400 (Courier 14400/ARQ after 12.6 s), V.22bis CONNECT 2400
(Courier 2400/ARQ after 11.3 s), 40 of 40 lines both ways, no retrain.

Over all 16 we-call calls, one V.32bis call lost reception about 23.7 s in.
Our modem sent AA (V.32bis 7.1), retrained and finished with every line. The
bearer was clean: our largest received RTP gap was 22.7 ms.

That call's tap reproduces offline, every time: `v90_engine_replay
live-rx.g711 ulaw --fast --dial --sync --at "ATX3;AT+MS=V32B"` loses
reception at 21.8 s (decision error 0.328 for 4818 symbols).

**The cause was our data-mode carrier loop, not the symbol timing.** It took
two wrong turns to find, and both are recorded so they aren't repeated:
- Holding the timing (`V32BIS_TIMING_HOLD=1`) decoded that tap, so the
  Godard timing loop looked guilty. Two ways of quietening the detector each
  held the tap but lost tracking under a clock offset, in netsim A/Bs on CT
  128 (NET_PPM 0/+-100/+-200, arms alternated, 40 calls each):
  - fine trigger 300: 71 loss retrains against 8;
  - a 16-baud average: 45 against 11.
- A decision-directed (Mueller-Muller) timing loop then held the tap and
  cut netsim losses from 21 to 6 in 40. It went live as 0f1be7f3 and was
  rolled back within the hour: 2 of 5 live we-call calls went bad. dd4 lost
  reception twice and got 11 of 40 lines; dd2 got 39 of 40 lines.

The replays of those taps settled it. Pass or fail moved with every
perturbation: the timing loop, holding the timing, holding the carrier, and
turning the echo canceller off each flipped some tap. That is a marginal eye
being knocked over by something else. On dd4 the timing barely moved (within
+-7 steps of 1/640 T over 16800 bauds). Meanwhile the carrier frequency
estimate swung 1800.18 -> 1799.12 Hz in 1200 bauds, and the decision error
rose 0.03 -> 0.18 with it. This path is digital end to end, so the real offset
is fixed and the swing is our loop ringing.

V.32bis trains through its own symbol sink, so V.17's trainer, which narrows
the loop to its tracking gains once trained, never ran. Data kept the
acquisition gains `v17_rx_restart()` sets (i 5000, p 40000). Data now uses
V.17's tracking gains (i 100, p 500000). Results:
- **All three live taps (e1, dd2, dd4) decode** at data-eye rms
  0.155-0.160, against 0.20-0.36 or lost. The result is flat over i 100-1000
  and p 200k-500k.
- **Netsim, alternated, 15 calls per arm and row:** we call, 0 loss
  retrains against 10 (6 calls); the Courier calls, 0 against 0. A
  decision-directed timing loop on top of the fix also gave 0, so it was not
  kept.
- **Frequency offset**, taps shifted offline: at +1 and -3 Hz the new gains
  decode and the old ones lose reception.
  - At +-2, +-4 and +-7 Hz our call modem never completes start-up, under
    either gain set: it stops after "incoming S". V.32bis 2.1 asks for
    +-7 Hz, so this is a separate, pre-existing gap in our S/TRN training.

`reference-data/courier-v32bis-data-carrier-ring` holds e1 and dd4 as
fast-tier fixtures: no reception loss and data-eye rms below 0.22. Under the
old gains e1 loses reception and dd4 runs at 0.296. `V32BIS_DATA_CARRIER=0:0`
restores the old gains.

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

### One bit error picked the wrong Phase 4 TRN (2026-11-25)

The remaining 10 dB failures at delays 2 and 8 were also ours. In both, one
receiver read Phase 4 TRN at 4.5–4.8 dB with 66–68% ones, while the other
direction read 11.6–11.9 dB, and MP never decoded.

What it was not:
- **The Phase 4 sampling instant.** Offsets of 0 to 0.75 T/2 changed nothing,
  and neither did suppressing the T/2 flip or keeping the Phase 3 front end.
- **The noise in that direction.** Without any noise on that path, the
  answerer still read 4.8 dB.

The far end's transmitter was the cause. Demodulated offline from the
transmit side, before the channel, our caller's Phase 3 TRN and J read 24 dB,
and its Phase 4 TRN read 7.6 dB with 4th-power coherence 0.36. That
coherence is what a 16-point constellation gives.

J says which TRN constellation the other end will use (V.34 §10.1.3.8), and
both ends use it in Phase 4 (§11.4). Table 18's J(4) = 0x8990 and
J(16) = 0x89B0 differ in one bit, and J repeats every 16 bits. Our J
confirmation needs eight near-exact hits at most four bits apart. Those
eight windows all lie in one J period, so one error on that bit made all
eight read a clean J(16) (dmin 0):
- At delay 2, our caller read J(16).
- At delay 8, our answerer read J(16).

Each then sent 16-point TRN to a receiver expecting 4-point.

The fix (`phase3_j_vote()`) takes the TRN size from the majority of every
near-exact window from one hypothesis and phase:
- **Before the 6000-bit gate.** J has usually been arriving for half a
  second by then. At delay 2 the vote was J(4) 1035 against J(16) 5.
- **After confirmation.** J keeps arriving until the far end hears our S,
  and nothing reads the TRN size before Phase 4.

When J is confirmed is unchanged. An earlier version delayed confirmation
until the vote spanned 96 bits. That moved our S 12 ms later against the
MT5634 3429-baud replay, and the replay diverged (B1 at +17 instead of +8).

Results:
- **10 dB row** (`V34_DUPLEX_NOISE_SEED` 1–8 × delays 0–11): 5/96 failures,
  against 8/96 before. The three wrong-J runs now pass. The other five are
  the same runs before and after. Three of them miss E after MP (§11.4),
  and two never accept any MP; see the next section.
- **J confirmation:** at the same bit count as before at every delay
  checked.
- **`test-fast`:** 777/777, including new rows at delays 2 and 8.

### One noise hit lost E (2026-11-25)

Three of the five remaining 10 dB failures were E (V.34 §11.4.1.1.3 and
§11.4.1.2.3). E is 20 ones, sent once, straight after the last MP'. The
receiver needed all 20 bits to be ones. At 10 dB, one or two symbol errors in
E lost it, and by then the far end had gone on to B1 and data. Its MP' frames
had stopped, so nothing else could end Phase 4. E as received:
- `11111111100111111111` (seed 3, delay 0, our caller).
- `11111111111111111101` (seed 5, delay 0, our answerer).
- `11111111111111110011` (seed 7, delay 10, our answerer).

On the MP frame grid, after a decoded MP', only two things can start: E or
the next repeated MP'. Table 20 fixes MP's first 20 bits: 17 ones, start 0,
type, and reserved 0. They differ from E in two of the ten 4-point symbols.
The receiver now takes E on the grid when the 20 bits are nearer E than that
prefix:
- Counted in symbols, not bits: a noise hit is one symbol and often flips
  both its bits. Counted in bits, an MP0' prefix with its (18,19) symbol hit
  (`0xFFFFB`) was 1 bit from E and 2 from MP, and was taken for E (seed 4,
  delay 5, B1 then −10.7 dB). In symbols it is 1 and 1, and a tie stays with
  the frame parser.
- At most 2 symbol errors, and 2 need a 2-symbol margin.
- Only after the peer's MP' has been decoded. Before it, the peer may still
  be repeating plain MP. One such frame, 3 symbols from its prefix and 2
  from E, was taken for E (seed 2, delay 10).

The B1 search after E looks only forward, so the decision cannot wait for
the bits that follow. Off the grid, and without a decoded MP', E still needs
all 20 ones.

Results:
- **10 dB row** (`V34_DUPLEX_NOISE_SEED` 1–8 × delays 0–11): 2/96 failures,
  against 5/96 before. The three on-grid E's are the only inexact matches
  in the 96 runs.
- **Still failing:** seed 2 delay 10 and seed 7 delay 3. In both, our caller
  never accepts any MP, so it never sends MP'. See the next section.
- **`test-fast`:** 781/781, including rows for seeds 3/0, 5/0, 7/10 and the
  false-E case 4/5.

### Phase 4 had no eye flip left (2026-11-25)

In the last two 10 dB failures, both ends read J(4) correctly (votes 1049–10
and 798–30), and the answerer sent 4-point Phase 4 TRN. Our caller read that
TRN at 51–55% ones and 3.2 dB for all 4800 bauds, so MP never decoded. A
passing neighbour (seed 7, delay 4) read 97–100% ones 0.2 s into TRN.

The front-end reset that conditions Phase 4 puts the symbol instant back to
`baud_half = 0`, whatever Phase 3 chose. The T/2 eye chooser has to find the
eye again, but its per-call cap (`V34_EYE_MAX_FLIPS`, 4) carried over from
Phase 3:
- Seed 7 delay 3: the caller's fourth flip fell on the far end's Phase 4 S,
  leaving none for TRN.
- Seed 2 delay 10: the caller flipped at S and again early in TRN, on eye
  sums carried over from before the reset.

The V.90 CP stage already gets two flips of its own and a fresh window for
the same reason (`v34_force_v90_phase4_cp_rx()`). The plain V.34 front-end
reset now does the same: it reserves two flips and clears the eye sums.

Results:
- **10 dB row** (`V34_DUPLEX_NOISE_SEED` 1–8 × delays 0–11): 0/96 failures,
  against 2/96 before. Both runs read TRN at 11.6–11.7 dB.
- **`test-fast`:** 783/783 with rows for both seeds.

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

