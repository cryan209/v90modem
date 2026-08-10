# Comparing our downstream against a working V.90 digital modem

## Why this comparison is possible

`../modem-dsp-emu` runs the Eicon Diva Server PRI card's own shipped V.90
firmware under emulation. Its captures include a USR Courier that **connects**:

```text
artifacts/interop/courier-2185n-dil/selector-call1.modem.log
  CONNECT 32000/ARQ/V90/LAPM
artifacts/interop/courier-2185n-dil/selector-call3.modem.log
  CONNECT 42666/ARQ/V90/LAPM
```

That gives us the one thing archaeology on our own captures cannot: the same
peer model, one digital side it accepts and one (ours) it does not. The card's
transmit stream is a true DS0 µ-law capture, so codewords are recoverable
exactly, with no analogue-recording caveat.

The two projects share no code. Nothing here is copied from the card; the
firmware is Eicon's. It is used as an **oracle** — a second implementation whose
observable behaviour disambiguates the Recommendation — not as a source.

The fixtures and their provenance are in
[`../artifacts/eicon-digital-downstream/`](../artifacts/eicon-digital-downstream/README.md).

## The headline

**The card transmits 30000T of TRN1d — 3750 ms. We transmit 2496T — 312 ms.**
Twelve times less. Everything below is either the evidence for that or a
hypothesis it replaced.

## Finding 1 — TRN1d is 3750 ms, and it is spec-exact GPC

§8.4.5 defines TRN1d as the PCM codeword whose Ucode is U_INFO, with signs from
applying binary ones to the §5.3 scrambler, **initialized to zero**. §5.3 makes
that scrambler V.34's GPC.

Generating that sequence and sliding it over the whole capture locks
**bit-for-bit**, on both calls independently:

| | call 1 | call 3 |
|---|---|---|
| GPC lock | **100.0%** | **100.0%** |
| starts | 8420.9 ms | 8460.4 ms |
| runs | **30005T (3750.6 ms)** | **30005T (3750.6 ms)** |
| magnitudes over the run | `{48}` | `{48}` |
| U_INFO | 48 | 48 |

100% over a 512-symbol window and then a 30005-symbol unbroken run is not a
statistical artefact. (30000T is 5000 six-symbol frames exactly; the trailing 5
is where the *next* signal happens to keep matching before diverging.)

Against §9.3.1.4 — TRN1d minimum 2040T, and Jd must start within 4000 ms of
TRN1d starting — the card uses **3750 of the 4000 ms budget**, leaving 250 ms.
We use 312 ms and leave 3688 ms unused:

| | TRN1d | share of the §9.3.1.5 budget |
|---|---|---|
| §9.3.1.4 minimum | 2040T (255 ms) | 6% |
| **ours** (`v90_trn1d_len()`, `v90.c:99`) | 2496T (312 ms) | 8% |
| **the card** | **30000T (3750 ms)** | **94%** |

The comment at `v90.c:85` reasons toward exactly this and stops short:

> §9.3.1.4 makes TRN1d a *minimum* of 2040T … so sending the bare minimum
> leaves ~3.7 s of headroom unused. §9.3.2.5 has the analogue modem condition
> its equaliser on the *first 2040T*, so transmitting exactly 2040T gives a peer
> that arms even slightly late a short training sequence and no way to recover.

That reasoning was right, and the correction applied (2040 → 2496) was about 1%
of the available headroom. The card spends 94% of it.

This fits the observed failure without further assumption. Per the note at
`v90.c:91`, the Courier detects our Sd→S̄d and goes silent per §9.3.2.4
(correctly), then **never decodes Jd**, never sends S, and retrains at its
§9.3.2.7 deadline — which is what a peer whose equaliser never converged would
do.

### The knob exists but cannot reach the card's value

`ME_V90_TRN1D_SYMBOLS` already exists. Its guard is `parsed <= 16000`
(`v90.c:121`), i.e. 2000 ms, on the stated grounds that "much above ~2 s starts
eating the Jd window §9.3.2.7 entitles the peer to use." The working
implementation uses 3750 ms, so that ceiling is contradicted by the oracle and
is what blocks the experiment.

**Next step:** raise the clamp to admit the card's value and dial the Courier at
`ME_V90_TRN1D_SYMBOLS=30000`. This is a live test, not an offline one — it needs
hardware, and it is the single highest-value call to make.

## The full downstream timeline, and what else is comparable

Segmenting both sides by (Ucode-0 density, distinct magnitudes, sign
periodicity) gives the card's whole Phase 3, and ours beside it:

| stage | card (connects) | ours (fails) | comparable? |
|---|---|---|---|
| pre-Phase 3 silence | 3000 ms | 2900 ms | yes — match |
| Sd | **384T** | 384T | yes — **match** |
| S̄d | **48T** | 48T | yes — **match** |
| TRN1d (const-mag, GPC signs) | **30000T (3750 ms)** | 2496T (312 ms) | yes — **12× short** |
| sparse, one non-zero per 6, descending level | 1950 ms | 100 ms | **no** |
| const-magnitude periodic (`+++−−−`) | 500 ms | 1000 ms | **no** |
| multi-level (data) | 12200 ms | 11400 ms | **no** |

**Only the first four rows mean anything.** §9.3.1.3 has the digital modem send
Sd, S̄d and TRN1d unconditionally after Ja, so those are transmitted into the
void and are directly comparable. **Jd then continues until the analogue
modem's S arrives** — and on our call it never does: our own decode shows
`Jd zone … 9744.4 ms`, us sitting in Jd waiting. Every stage after TRN1d is
therefore gated on a peer response we do not get, and its duration is a
*consequence* of the failure, not a candidate cause.

Reading those last rows as defects would be the trap that cost `modem-dsp-emu`
three leads in one session (handoff §2: "do not compare two ends without
checking both are in the same phase"). They are recorded here so nobody
re-measures them and gets excited.

So: **TRN1d is the only defensible transmit-side fix these captures support.**

### Sd and S̄d are confirmed correct

Measured on the card, both calls: Sd is `{+W, +0, +W, −W, −0, −W}` with
W = Ucode **64**, running **384T**, followed by **48T** of S̄d. §8.4.4 defines
W = Ucode(16 + U_INFO), and the card's TRN1d runs at U_INFO = 48 — so
16 + 48 = 64 checks out, and the card is internally consistent and spec-exact.

`V90_SD_REPS 64` and `V90_SD_BAR_REPS 8` are right. Do not touch them.

### The remaining value in these captures is on the receive side

Decoding the card's Jd, J'd, DIL and Phase 4 is *not* confounded — a
reference decode is a reference decode regardless of what our transmitter did.
The sparse descending-level stage (Ucode 76 falling to 2, one non-zero per six
symbols, 1950 ms) is unidentified and is the obvious next target: `--dil-scan`
reports `default_125x12=no impairment=9` for the card, so whatever it is, it is
not the default DIL our encoder emits.

## Finding 2 — our decoder mislabels the card's TRN1d as "Sd"

`vpcm_decode --v90` reports, on the card's stream:

```text
[14261.1 ms] V.90 Sd (sign-pattern): 758 reps (4548 symbols, 568.5 ms)
```

**This is not Sd.** §8.4.4 makes Sd `{+W, +0, +W, −W, −0, −W}` — one third of
its symbols are Ucode 0. Measured over that exact span: 4547 of 4548 symbols are
Ucode 48 and **one** is Ucode 0. A constant-magnitude signal cannot be Sd.

The `--dil-scan` output labels the same span a "codeword-exact DIL run" with
"training Ucodes: 1 unique (first: 48)", so two different reports disagree about
one region and neither is right.

This is a real receive-path defect — an Sd detector that fires on a
constant-magnitude signal will mis-anchor everything downstream of it — and it
is what `make eicon-rx-test` pins.

## Withdrawn — do not re-derive

- ~~"The card transmits 4548T of Sd where §9.3.1.3 says 384T."~~ **Wrong.** That
  region is TRN1d (Finding 1), and 4548T was only the part of it our detector
  happened to bracket. The card's Sd was never measured here. Nothing in this
  capture shows the card violating §9.3.1.3, and `V90_SD_REPS 64` is not
  implicated. The SmartLink justification at `v90.c:75` stands unchallenged.
- ~~"A scrambler convention mismatch — seed, tap or polarity — explains why we
  cannot read the card's TRN1d."~~ **Disproved.** GPC, zero-initialized, matches
  100% (Finding 1). There is no convention error. Our "0 TRN1d symbols" is a
  search-and-labelling failure (Finding 2), not a disagreement about the
  scrambler. The GPA/GPC distinction CLAUDE.md warns about is not in play.
- ~~"Alignment the descrambler infers from our own S̄d length."~~ Withdrawn:
  `V90_SD_BAR_REPS` is referenced only by the transmitter; no receive path reads
  it.
- ~~"Long Sd predicts success."~~ Never survived its own census — the card sent
  the same long signal on call 2, which returned `NO CARRIER`. Withdrawn on its
  own terms even before Finding 1 removed the premise.

Four hypotheses, all killed by measurement. `modem-dsp-emu`'s handoff §0.5 is
worth importing as a habit: its measurements reproduce, its interpretations
repeatedly have not. Finding 1 is a measurement, verified twice. Treat the
causal story attached to it as still unproven until a live call says otherwise.

## Why our own tests could never have found this

Our receive path — `p3_demod`, `v90_dil_rx`, `v90_cp_rx`, and the
Sd/TRN1d/Jd/DIL decoders in `vpcm_decode` — is *analogue-side* code: those are
the signals an analogue modem receives from a digital one. Until these fixtures
existed it had only ever been fed by our own transmitter. In
`vpcm_loopback_test.c` that is `v90_dil_generate_codewords`, `v90_tx_codewords`,
`v90_tx_data_frame_codewords` and `v90_generate_trn2d_codewords` — all ours.

A wrong-but-self-consistent assumption is invisible to that harness by
construction: both ends agree, the suite goes green, the peer still fails.
CLAUDE.md's "passing them says nothing about hardware interop" is the weaker
form; the stronger form is that loopback cannot *in principle* catch it.

Finding 2 is an instance. Our Sd detector fires on a constant-magnitude signal;
our own Sd is never constant-magnitude, so the false positive can only be
triggered by someone else's transmitter.

## The fixture

`make eicon-rx-test` (`tools/eicon_rx_conformance.py`):

```text
[FAIL] call1-connect-32000.ulaw
         control: codeword-exact DIL run 14261.1-14829.6 ms
         control: Sd recovered, 758 reps
         TRN1d: 0 symbols descrambled (need >=48)
[FAIL] call3-connect-42666.ulaw
         control: codeword-exact DIL run 14317.9-14882.6 ms
         control: Sd recovered, 753 reps
         TRN1d: 0 symbols descrambled (need >=48)
```

Read that "control: Sd recovered" line with Finding 2 in mind — it is the
mislabel, not a pass. The control is doing its job in the narrow sense (codeword
recovery works, so the zero is a real failure) but its Sd arm is asserting
something false, and should be tightened to require Ucode-0 density near 1/3.

Not in `make test`: it encodes a known-open defect, and a suite that is red by
default stops being read. `--expect-failure` inverts the exit status, so the
target is green while the defect stands and goes loud if the control breaks or
the defect is fixed.

## Reproducing Finding 1

No build needed — it is a property of the bytes:

```python
import pathlib
d = pathlib.Path("artifacts/eicon-digital-downstream/call1-connect-32000.ulaw").read_bytes()
signs = [((b ^ 0xFF) >> 7) & 1 for b in d]
mags  = [(b ^ 0xFF) & 0x7F for b in d]

def gpc_ones(n, taps=(18, 23)):          # §5.3 -> V.34 GPC, zero-initialized
    s = []
    for i in range(n):
        v = 1
        for t in taps:
            v ^= s[i - t] if i - t >= 0 else 0
        s.append(v)
    return s

off = 67367                              # 8420.9 ms
run = 0
for i, r in enumerate(gpc_ones(len(d) - off)):
    if signs[off + i] != (r ^ 1):
        break
    run += 1
print(run, sorted(set(mags[off:off + run])))   # -> 30005 [48]
```
