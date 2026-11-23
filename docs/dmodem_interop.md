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

## Results (2026-11-24, a6737e79 + half_v90modem, D-Modem eceac3b)

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
| ESS | V.92 PCM up, μ-law and A-law | Fail. Phase 3 completes and our CPd goes out precoded, but B1u is not locked 5 s after the CPd acknowledgement. We report CONNECT, ESS answers ATD with OK, and the data is garbage |
| ESS | V.90 | Fail. We CONNECT at 28800, ESS answers OK, garbage |
| HSF | V.90, V.92 (both laws) | Fail. Reaches Phase 3 (our first S), then the peer gives NO CARRIER |
| Agere | V.90, V.92 | Fail. Same as HSF |
| Broadcom | V.90, V.92 | Fail. Same as HSF |
| Courier | V.8 default | Fail. V.8 selects V.90, Phase 3, then NO CARRIER |

D-Modem's own modem passes the same rows. For example, Agere V.90 against their
`half_open8` passes on CT 128, and SmartLink V.92 against it connects at
56000/38666 with a 64-tap prefilter where we get 34666. So these failures are
ours to explain. Every failing row is a `NET_LOOP=0.45 NET=8000:8000` line,
except the Courier, which has the same line. PCTel and SM56 pass on that line,
so the line alone does not explain it. None of the failures has been
investigated yet. The ESS V.92 row is the one to start with: its precoder
departs from §6.4.2 (`docs/v92_pcm_upstream_findings.md`, ESS quirk section)
and this is the first time our receiver has met it.
