# A working V.90 digital modem's downstream

Transmit-side DS0 µ-law captures from the Eicon Diva Server PRI card's own
shipped V.90 firmware, running under emulation in `../../../modem-dsp-emu`. The
peer is a USR Courier V.Everything, and **both calls connected**:

| file | reported by the Courier |
|---|---|
| `call1-connect-32000.ulaw` | `CONNECT 32000/ARQ/V90/LAPM` |
| `call3-connect-42666.ulaw` | `CONNECT 42666/ARQ/V90/LAPM` |

These are the only downstream streams in this repository that **we did not
generate**. That is their entire value: our receive path is analogue-side code
(Sd, S̄d, TRN1d, Jd, DIL are what an analogue modem receives from a digital one),
and until these existed it had only ever been tested against our own
transmitter, which cannot detect a wrong-but-self-consistent convention. See
[`../../docs/eicon_downstream_comparison.md`](../../docs/eicon_downstream_comparison.md).

Raw G.711 µ-law codewords, 8000 Hz, no header. 216160 and 190560 samples
(27.020 s / 23.820 s).

```text
sha256  a14e74e7b201fd21e9ed0fc6873d4b6fe070c2502dc27a6b721f3c931866b585  call1-connect-32000.ulaw
sha256  72cab5a62ed3b1e4a6c05b55f8e4c53b1302eb9cdce7efae1744ecf4ec5e62a8  call3-connect-42666.ulaw
```

## Provenance

Sliced from `modem-dsp-emu/artifacts/interop/courier-2185n-dil/selector.ulaw`,
which concatenates three calls. Boundaries are the endpoint log's per-call TX
sample totals — 216160 / 239200 / 190560, summing to that file's 645920 bytes
exactly:

```bash
cd ../modem-dsp-emu/artifacts/interop/courier-2185n-dil
dd if=selector.ulaw of=call1-connect-32000.ulaw bs=1 count=216160 status=none
dd if=selector.ulaw of=call3-connect-42666.ulaw bs=1 skip=455360 count=190560 status=none
```

The middle call is deliberately omitted: it returned `NO CARRIER`
(`0 payload / 21188 mark fill`), so it is not evidence of a readable downstream.

The firmware is Eicon's. These captures are used as an oracle — observable
behaviour of a second implementation — and no code is derived from it.

## Use

```bash
make eicon-rx-test
```
