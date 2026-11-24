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
| SM56 | V.90 | Pass. 50666 / 31200, LAPM + V.42bis |
| Intel 537EP | V.92 offer | Pass. It has no CPu, so V.90 at 52000 / 31200 (as D-Modem says) |
| Intel 537EP | V.90 | Pass. 52000 / 31200 |
| ESS | V.92 PCM up, μ-law | Pass, 8 of 8, 37333 up, B1u locked first time since the Phase 3 retime (`docs/v92_pcm_upstream_findings.md`, "ESS μ-law's first B1u miss"). Before: 29333-30666 on the retrain's unprecoded CPd, then 37333 with the first B1u missing in 2 of 3. In about 6 of 40 calls ESS misses our first CPd and sends Tone A (0 of 48 against `half_open8`). The retrain usually recovers it; one call took two retrains and then a plain V.34 CONNECT at a negative SNR (`docs/v92_pcm_upstream_findings.md`, "The 90 s ESS μ-law call") |
| ESS | V.92 PCM up, A-law | Pass, 8 of 8 (and 3 of 3 before the retime), B1u locked at 37333 first time, 48000-52000 down. Before: about 30000 up and 3 of 6 calls. Our Jp had always asked for no Su extension, so the network A/D sampled ESS's pulse half a symbol off its peak (`docs/v92_pcm_upstream_findings.md`, "The receiver gap was the A/D's sampling phase"). ESS renegotiates 1-3 times a call, as it does against D-Modem's own modem. In about 1 call in 8, ESS's own transmit clock slews about −180 ppm through its first TRN1u. That also happens against `half_open8` at some line delays. Our receiver tracks it, and those calls lock B1u as usual. Until our data-mode frequency correction, a renegotiation's TRN2u came out at up to 153 DS0 and could cost a call its upstream. Now it is 57-61, the same as startup (`docs/v92_pcm_upstream_findings.md`, "A renegotiation's TRN2u was our held frequency walking") |
| ESS | V.90 | Pass, 8 of 8 over line delays, 45333-50666 down. Before: fail, because we rejected ESS's real S during our Jd as our own line (0.125-0.148 of our transmit RMS), so ESS retrained twice and fell to plain V.34, where we CONNECTed on a link nobody decodes (`docs/v90_phase3_4_spec_audit.md` row 12). ESS's plain V.34 fails against `half_open8` too |
| HSF | V.90, V.92 (both laws) | Fail. Since the INFO1d re-send fix (below), V.90 gets a V.90 INFO1a (3200 up), our Jd is answered with S, and HSF retrains in the second DIL cycle without sending the §9.3.2.10 S. Its V.92 rows decline PCM in INFO1a (U_INFO 2, 2400 baud). Against `half_open8` it picks a 3429-baud upstream, which we do not offer |
| Agere | V.90, V.92 | Fail. Since the INFO1d re-send fix, V.90 gets a V.90 INFO1a (3200 up), then Agere retrains during our Jd, whatever the TRN1d length (2496T, 8004T and 20004T alike) |
| Broadcom | V.90, V.92 | Fail. Declines PCM in its first INFO1a (V.34, 3000 baud, U_INFO 1). Against `half_open8` it picks V.90 with a 3429-baud upstream, which we do not offer (V.90 6.2 makes 3429 optional; INFO0d bit 40 = 0) |
| Courier | V.8 default | Fail. V.8 selects V.90, Phase 3, then NO CARRIER |

D-Modem's own modem passes the same rows. For example, Agere V.90 against their
`half_open8` passes on CT 128, and SmartLink V.92 against it connects at
56000/38666 with a 64-tap prefilter where we get 34666. So these failures are
ours to explain. Every failing row is a `NET_LOOP=0.45 NET=8000:8000` line,
except the Courier, which has the same line. PCTel and SM56 pass on that line,
so the line alone does not explain it. None of the failures has been
investigated yet, apart from ESS V.92 (`docs/v92_pcm_upstream_findings.md`,
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
- Agere retrains during our Jd, and HSF during our DIL. Neither is explained
  yet; Agere's `AGERE_DEBUG=1` log shows only codec register writes.

