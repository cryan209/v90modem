# Non-standard modulations: what exists, what we lack

Written 2026-10-12. Sources: our `at_ms.c` carrier table, the Courier
firmware notes (`courier_firmware_analysis.md`), and the strings and loader
stubs of the Cisco MICA portware image (`MicaEmu/mica-portware-map.bin`).

## What the two vendor images say they implement

| Source | Modulations listed |
|---|---|
| Courier V.Everything (`SV25.XMD`, `ATI7`/`ATC8`) | HST, V.32bis, V.32terbo, V.FC, V.34+, x2, V.90 |
| MICA portware banner and `%15S` help | V.90, K56flex 1.1, V.34+ (with or without terbo), V.32terbo, V.22bis, V.22, V.21, V.23, Bell 103, Bell 212, V.110, X2, MNP 2-5, V.42, V.42bis, fax class 2/2.0, SS7/COT |

MICA `%15S` selector (portware help text): 0 V.34+ automode with terbo, 1 V.34+
without terbo, 2 V.32terbo automode, 3 V.34, 5 V.32, 6 V.90 automode, 7 V.22,
8 V.21, 9 V.23, 10 Bell 212, 11 Bell 103. `%15S2`/`%15S3` toggle V.90 / K56flex
in V.8bis. `%19S2` carries an X2 string.

## Where we stand

| Modulation | State here |
|---|---|
| K56flex | V.8bis, payload core, training stream, client; upstream gates unrecovered (`k56flex_implementation.md`) |
| x2 | session, symbol and MP receive; qualified against a Courier (`x2_implementation.md`) |
| V.32bis, V.22(bis), V.21, V.23, Bell 103/212A, V.110, V.120 | wired |
| **V.32terbo** | **absent.** `AT+MS=V32TERBO` is accepted and falls back to V.32bis |
| **HST** (USR) | **absent.** same fallback |
| **V.FC** (Rockwell V.Fast Class) | **absent.** same fallback |
| **MNP 2-5** | **absent.** MICA and the Courier both carry it; matters for pre-V.42 peers |
| V.34 with terbo | not applicable to the datapump; it is a V.8/V.34 negotiation switch in MICA (`%15S0` vs `%15S1`) |

So against both vendor images the missing ones are **V.32terbo, HST, V.FC and
MNP**. x2 and K56flex exist but are incomplete. The three proprietary
datapumps are the real gap; MNP is a protocol layer, not a modulation.

## MICA overlay map (loader stubs in the C53 program)

The DSP overlays are chosen by small loader stubs between program `2BBA` and
`2C1D`; each picks an overlay pair, the second being the processor-role
companion, by testing a bit of `@05`/`DM 6F`:

| Stub | Overlay pair | Caller | Identified |
|---|---|---|---|
| `2BBA` | 7C / 7D | none found | 1-word files, empty |
| `2BC9` | 90 / 91 (10,020 B, largest) | `CBAC` | **no** |
| `2BD8` | 8A / 8B (6,800 B) | `07F8` | **no** (contains 1180, 600, 1200: V.21/V.22 constants?) |
| `2BE7` | 7E / 7F (5,798 B) | `C8BB` | **non-V.8 answer path**: loads when ANSam times out for S29 = 2-4 |
| `2BF6` | 8C / 8D | `0C7F` | K56flex PCM module |
| `2C05` | 80 / 81 if DM `2C` bit 10, else 8E / 8F | `1FB6`, `98F5`, `9DEB` | 8E/8F V.32bis; 80/81 **no** |

Overlays 7B (3.2 KB) and 85 have no stub in this range and are not yet
attributed; 88 is the K56flex carrier-rotation module (`mica_qam_firmware.md`).

Identification by constant scanning is **inconclusive** so far. A search of
the large overlays for rate words (19200, 16800 and so on) found only
false positives: `4B00` is a `bit 11,@00` opcode, not a rate. Carrier
constants (1920 in 7E, 3000 in 80, 1180/600 in 8A and 90) are weak hints,
not identifications. Do not read 7E as V.32terbo from this note.

## Live overlay probe (2026-10-12)

Setting: the "Modulation Standards" parameter is **S29** (`ATS29=n`; `%29M`
in the help strings). 0 V.34+ with terbo, 1 V.34+ no terbo, 2 V.32terbo,
3 V.32bis, 4 V.22bis, 5 K56flex 1.1, 6 V.90 (default). Probe: MicaEmu
`Module` with the linked C53, `ATS29=n`, `configure_call` + `start_answer`,
`core.set_coverage(True)`, then `first_exec()` of each loader stub. No peer
was attached, so this shows only what the answerer loads before negotiation.
**Caveat:** this used `start_answer` and mono framing, which the corrected
harness below replaces. The loader result (8E/8F first, 7E/7F at the ANSam
timeout for S29 = 2-4) did survive the correction; the timings did not.

| S29 | Stubs hit in 30M instructions |
|---|---|
| 0, 1, 2, 3, 4, 5, 6 | `2C05` only, taking the 8E/8F branch (cycle 21.50M for 2-4 and 6, 21.52M for 1 and 5) |
| 2 and 3, run 250M | additionally `2BE7` (7E/7F) at cycle 154,087,137, identical for both |

Conclusions, and their limits:

- **No terbo-specific overlay is selected by S29.** V.32terbo (2) and V.32bis
  (3) load the same overlays at the same cycles. Any terbo behaviour must live
  inside 8E/8F (rate mask or R-word table) or in a later stage.
- Every setting starts the answer in 8E/8F, so 8E is the generic answer
  overlay, not only V.32bis; the earlier "8E is V.32bis" wording is true of
  its transmitter and gates, not of how it is selected.
- 7E/7F loads at the same late cycle for both, which points to an
  answer-timeout/fallback path rather than a modulation. Not verified.
- Untested: a calling peer. With a V.32 AA/AC or V.8 CM tone fed in, the
  negotiated stage may load others. 90/91, 8A/8B and 80/81 never loaded here.

### Why the earlier injected-caller runs saw nothing (corrected)

The first caller injections (`inj.py`, `v8inj.py`, `tone.py`) were wrong, and
their conclusions ("the answerer never advances", "mirrored tone phases") are
withdrawn. Three harness errors, all visible in `mica_line.cpp`:

1. **Two lanes.** One C53 carries two channels as alternating G.711 octets.
   Channel 0 is the even lane of `g711_tx()`; the receive queue must get each
   sample duplicated into both lanes. Reading the interleaved stream as mono
   produced the mirrored spectra and a doubled time base.
2. **Call start.** The answer is `host.dsp_command(0, 0xA0, [0])` after
   `configure_call`, as `mica_peer_probe.py` does, not `start_answer`.
3. **V.8bis.** S53 (V.8bis) holds the first seconds of the answer; the
   answerer's first 2 s are silence either way.

`tools/mica_overlay_probe/call3.py` is the corrected harness.

### What the answerer actually does (S29, correct framing, 2026-10-12)

ANSam (2100 Hz, AM) starts at 2.0 s. Then it depends on S29:

| S29 | Answerer | Evidence |
|---|---|---|
| 0, 1 (V.34+ automode, with / without terbo), 5 (K56flex), 6 (V.90) | V.8: JM at ~4.1-4.4 s, modulations `0xA16` = V.21, V.22, V.23, V.32/V.32bis, V.34 | SpanDSP V.8 caller offering every modulation |
| 2 (V.32terbo), 3 (V.32bis), 4 (V.22bis) | **no V.8**: ANSam times out at 6.0 s, overlay 7E/7F loads (the non-V.8 answer path), V.8 caller fails at 8.6-9.4 s | same caller; loader trace |

Plain V.32bis caller (no V.8, SpanDSP, AC from 0 s or 3 s): start-up
completes against S29 = 0, 1, 2, 3, 5 and 6 at 14400, and not against S29 = 4.
The channel handler reaches `35d70` at about 10.5 s (about 9.5 s when the caller
starts at 0 s).

### V.32terbo is a V.32bis R-word extension (new)

`V32BIS_TRACE=1` prints the 16-bit words the caller decodes. MICA's opening
rate signal differs:

| S29 | R word from MICA |
|---|---|
| 2 (V.32terbo) | `0xFFF0` |
| 0, 1, 3 | `0x9FF0` |

Bits `0x2000` and `0x4000` are the difference: two extra bits in the rate
word, beyond V.32bis Table 5, which has five rates (4800, 7200, 9600, 12000,
14400; mask `0x1660`). SpanDSP ignores them (its decoder masks to the five
known bits). Two extra bits fit the two extra terbo rates, 16800 and 19200,
but that mapping is inferred, not read from a document.

DSP configuration agrees. Snapshots of DSP data memory 1.5M instructions after
`0xA0` (`cfg.py`) show S29 = 2 matching S29 = 0 (the "with terbo" setting)
and S29 = 1/3/4/6 matching each other across a block of words:

- `E6D0`: `0x00FF` (terbo, and `0x3FFF` for 0) against `0x003F` (V.32bis); two
  more bits set, which would fit the same two extra rates;
- `8D45`, `8D49`, `8E0E..8E4B` (including tables at `8E24..8E38`),
  `EC04..EC13` (a repeating four-word table), `EC80`, `EC84`, `D9FB`/`ED1D`,
  `DA63`, `E6CD`: terbo values for 0 and 2, standard values for the rest.

What is not known: the data-mode constellations and trellis for 16800 and
19200, the exact meaning of the two R bits, and how the caller echoes them.
SpanDSP cannot be the peer for that, because it will not offer those bits.
The `EC04..EC13` and `8E24..8E38` tables are the first place to look for the
terbo mapper, since they are the words that change.

### Echoing the terbo bits (patched caller, 2026-10-12)

`v32bis_set_rate_signal_extension(s, bits)` (vendored `v32bis.c`, default 0,
so the engine is unchanged; `v32bis_*` tests pass) ORs B13/B14 into the rate
signals the caller sends in the clause 6 dialogue. Against MICA, caller starts
at 3 s, `EXT` is the bits sent in R2:

| S29 | EXT | MICA R1 | R3 | Start-up |
|---|---|---|---|---|
| 2 | 0 | `0xFFF0` | `0x9990` (14400), E `0x999F` | completes |
| 2 | `0x2000`, `0x4000` or `0x6000` | `0xFFF0` | **none decodable** | does not complete |
| 3 | `0x6000` | `0x9FF0` | `0x9990` | completes, bits ignored |

- MICA keys on the **presence** of a terbo bit: each of B13 and B14 alone gives
  the same result as both. At S29 = 3 the bits make no difference, so this is a
  terbo-mode behaviour, not a general V.32bis one.
- After our R2, MICA at S29 = 2 leaves the standard sequence. Its signal is
  identical to the standard run until about 8.5 s, then 1.5 percent lower
  power in 20-28 s (rms 3292 against 3343, the same for all three EXT values),
  and the caller never frames an R3. SpanDSP can only frame the standard
  sequence, so what MICA sends after R2 is **not decoded**. I do not know
  whether it is a 16800/19200 R3, a longer training, or an error path.
- (Superseded by the demodulation below: what MICA sends is R3 `0xC990`.)
- So the handshake gate is found (terbo needs a terbo bit in the caller's
  R2) but the terbo continuation is not read. Next: capture MICA's lane-0
  audio for that call (`call3.py` writes `mono_<S29>_<EXT>.bin` as G.711) and
  analyse it by hand with a 4-point symbol demodulator at 2400 baud, 1800 Hz,
  starting around 8 s, to see what replaces R3.

### What replaces R3: `0xC990` (demodulated, 2026-10-12)

`tools/mica_overlay_probe/demod/` is a small pure-Python V.32bis demodulator
for MICA's lane-0 audio: mix down from 1800 Hz, 61-tap low-pass, symbol
timing from the 2400 Hz line in |z|^2 (sampling offset 0.75), 4th-power
carrier tracking kept continuous across blocks, then SpanDSP's own startup
word decoding (differential decoder, 23-bit self-synchronising descrambler,
tap 4 for the answerer). Validated on the standard run: S (`0xAAAA` x 21),
S-bar (`0x3333` x 29), **TRN matches the scrambled-ones sequence for 4400
consecutive symbols with no errors**, then `0x9990` (R3 selecting 14400)
exactly as SpanDSP's trace reports. Pitfalls found on the way: the state
numbering needs the mirrored constellation order (`[0,2,3,1]`; the obvious
order matches only 50 percent of TRN), and the per-block 4th-power
derotation must be unwrapped against the previous block or differential
decoding breaks at every block edge.

Timeline from the recordings (mono time, S29 = 2, caller started at 3 s):

| t | Standard run | Echoed bit (`0x6000`) |
|---|---|---|
| 3.5-7.0 s | first sequence: S, S-bar, TRN1, R1 (`0x9FF0`; `0xFFF0` at S29 = 2) | same |
| 7.0-8.0 s | MICA silent, waiting for our R2 | same |
| 8.4-8.6 s | second sequence: S, S-bar | same |
| 8.61-10.46 s | TRN2 (4400+ symbols, exact scrambled ones) | same, identical audio |
| 10.456 s | R3 `0x9990` x22, E `0x999F`, then dense data | R3 = **`0xC990` x502** if the caller never answers (4016 symbols, 1.67 s) |

`0xC990 ^ 0x9990 = 0x5000`: **the 14400 bit (B12) is cleared and B14 is set.**
So in terbo mode R3 names the extended rate by B14 instead of B12. Offering
B13 alone, B14 alone or both (`EXT` = `0x2000`, `0x4000`, `0x6000`) gave the
same MICA behaviour, and its power after 20 s was identical, so MICA picked
the same rate each time; with only B13 offered it still answered B14, which
means the answerer chose its own best terbo rate rather than echoing ours.
The most natural reading is B14 = 19200 and B13 = 16800 (higher rate on the
higher bit), but **that mapping is inferred, not observed**: nothing here
has measured the data rate.

(Corrected: an earlier version of this table put R1 and R3 in one sequence; the
caller's trace shows R1, a silence of about a second while MICA waits for R2,
then a second S/S-bar/TRN of 4464 symbols before R3. The audio agrees.)

MICA repeats R3 until it sees E and, getting none (SpanDSP does not accept
`0xC990` as a valid rate signal, so the caller never sends E), times out
after about 1.67 s. The caller side therefore stalls at R3, not at R1/R2: the
gate (a terbo bit in R2) and the answer (`0xC990`) are both understood now.

### Answering with E `0xC99F` (probe caller, 2026-10-12)

`v32bis_set_terbo_probe()` (default off) makes the caller accept an R3 that
names its rate by B13/B14 only, in the TRN-to-R framing stage and the E
stage, and answer with E = that word | `0xF`. The caller's receiver also
traces when it drops back to S search (`V32BIS_TRACE=1`: "rx power ... back to
S search"); that is the normal silence after R1, seen in the standard run too.

Against S29 = 2 with `EXT=0x6000`:

- the caller frames `0xC990` after 4464 symbols of TRN2, exactly as it frames
  `0x9990` in the standard run, and sends **E `0xC99F`**;
- MICA answers with its own **E `0xC99F`** at the same position as the
  standard `0x999F`. So MICA accepted the terbo rate selection and left the
  rate negotiation: E-for-E mirrors the standard dialogue;
- after E, MICA runs at 2400 baud like the standard run, and its audio is the
  standard run's until 10.456 s. The symbol-power kurtosis (E|s|^4 over
  (E|s|^2)^2 of the demodulated symbols, best sampling instant) is **1.363**
  for the standard 14400 run (what a 128-point cross constellation gives) and
  **1.41-1.42** for the terbo run: a different constellation, not noise at the
  same density. Uniform square QAM tops out near 1.40, so a value above that
  also says the points are not uniformly spread (non-uniform amplitudes, or
  our equalizer-less demodulator's ISI adding to it).

Not established: the data rate. SpanDSP has no constellation for it, so the
caller's data-mode receiver does not lock (`done` stays 0) and no payload is
read. The next measurement is an equalized, decision-directed constellation
of MICA's data signal, from which the number of points follows.

### Counting MICA's terbo constellation (equalized, 2026-10-12)

Method (`tools/mica_overlay_probe/demod/`, pure Python): resample the 1800 Hz
baseband to T/2 with a windowed sinc (the first cubic interpolator left too
much noise), train a 32-tap fractionally spaced LMS equalizer with a carrier
PLL on the **known** second TRN (4400 symbols, scrambled ones; mirror=True
state order; after aligning on a correlation peak 3 half-samples in, the
training MSE is 0.0002), keep those weights through the data that follows,
then refine them decision-directed against a lattice. Lattice parameters come
from a 2-D Fourier scan over spacing and rotation, which does not need the
carrier tracked first (the 4th-power tracker is unreliable on dense
constellations).

Control (the standard 14400 run, mono time 11.5-17.5 s):

| | |
|---|---|
| lattice spacing d | 0.2256, rotation 12-16 deg to the TRN frame, odd lattice (offset 0.5 d) |
| lattice score | 0.94 (1.0 = perfect); rms distance to nearest site 0.08 d |
| occupied sites | **128**, the same at every threshold from 0.1 to 0.5 of the mean |
| rows and columns | **12 and 12**, exactly the V.32bis 128-point cross |
| ranked-occupancy knee | at rank 126-128 |
| 6P/d^2 + 1 | 124 (3 percent under the true 128) |

Terbo run (S29 = 2, caller echoing B13/B14, after E `0xC99F`), same method:

| | |
|---|---|
| symbol rate, carrier | 2400 baud, 1800 Hz (unchanged) |
| lattice spacing d | **0.105-0.110**, same orientation (12 deg), same odd offset; the secondary peak at 0.075 and 57 deg is the lattice diagonal d/sqrt(2), not a finer lattice |
| mean power | 1.00 against 1.05 for the standard run (same equalizer scale) |
| 6P/d^2 + 1 | **525-545** (about 560 after the control's 3 percent) |
| rows / columns | about 28 blurred, edge counts taper over about 8 rows |
| sites above half the typical count | 513 (577 at 0.3, 603 at 0.1) |
| knee | none: occupancy falls smoothly from 41 to 9 between rank 0 and 580 |

So the data constellation has **roughly four times as many points as the
standard 128, most consistent with 512** (9 bits per symbol). 256 is excluded
(it would need d = 0.153) and so is 1024 (d = 0.077). With V.32bis's one
trellis bit that is 8 data bits per symbol: **8 x 2400 = 19200 bit/s**, which
fits B14 = 19200 and gives B13 = 16800 (7 data bits, 256 points) by elimination
of the neighbours. If MICA's terbo mode used no trellis, 9 data bits would be
21600; nothing here distinguishes the two.

What this does not show:

- **No cell-exact count.** The terbo clusters never resolve: rms distance to
  the nearest site stays at 0.3 d. Borrowing the equalizer that converged on
  the standard run changed nothing, so it is not equalizer error. A single
  uniform lattice may be the wrong model (warping, shaping or precoding would
  broaden every peak); I tried a radius-split spacing fit, but it is invalid
  for truncated sets and gave nothing.
- The estimate is a power/spacing one, good to about 5-10 percent, not a count.
- B13/B14 to 16800/19200 and the trellis question remain inferred.

### The terbo constellation is warped, not shaped (2026-10-12)

Why the earlier terbo clusters never resolved: the points do not sit on a
uniform lattice. A windowed local fit (Hann-weighted 1-D lattice spacing along
lattice rows and columns, `demod/warp2.py`) gives, for the same equalized
data:

| | standard 14400 (control) | terbo run |
|---|---|---|
| spacing at the centre | 0.2253 | 0.1011-0.1058 |
| spacing at \|x\| = 0.5 | 0.2253 | 0.1067 |
| spacing at \|x\| = 0.7 | 0.2253-0.2257 | 0.1094-0.1122 |
| spacing at \|x\| = 0.9-1.05 | 0.2253-0.2262 | 0.1145-0.1185 (clipped by the fit range) |
| local lattice score | 0.92-0.94 everywhere | 0.90 near the centre falling to 0.24 at the rim |

So the control is uniform to +-0.4 percent and the terbo spacing grows by
10-17 percent towards the edge, symmetrically on both axes. It is anisotropic:
the tangential spacing near r = 0.8 is 0.1055 against about 0.115-0.118
radially, which is what a radial map `r -> g(r)` of a uniform lattice gives.
Locally the lattice is crisp (score 0.9 on a single row), so the blur in the
global fits was the warp itself.

Fitting `g(r) = r (1 + a r^2 + b r^4)` (in the equalizer's unit-power scale)
by maximising the lattice score of the unwarped points (`demod/unwarp.py`,
`unwarp2.py`): the score goes **0.28 (no warp) -> 0.62** at a = 0.08-0.085,
b = 0, and the pre-warp lattice spacing is d0 = 0.1006. After unwarping:

- the ranked site occupancy now has a **knee at rank 510-525** (counts 21,
  19, 16, 13, 7, 1 at ranks 450, 480, 500, 510, 520, 540); before it only
  tapered;
- sites at or above 0.25, 0.35 and 0.50 of the mean count (27.3 for 14000
  symbols over 512 sites): **525, 517, 509**;
- the region is a **circle**: the farthest site in each of twelve 30-degree
  sectors is 12.7-12.8 cells from the centre (pi x 12.75^2 = 511), 26 rows
  and 26 columns, row lengths 5, 10, 14, 18, 20, 20, 22, 22, 24, 24, 26, 26,
  25, 25, 26, 26, 24, 24, 22, 22, 19, 20, 17, 13, 10, 5 (509 in all);
- **no probability shaping**: mean count per site in five radial bands is
  26.3, 27.1, 27.1, 28.1, 25.1 (expected 27.3 for 512 equiprobable points),
  and the control is flat as well (111.0 to 107.8). A shell-mapped
  (Gaussian-like) distribution would fall with radius; this does not.

So the data constellation is **about 512 equiprobable lattice points in a
circular region, with a radial warp that stretches the outer points by 10-17
percent**. 512 points is 9 bits per symbol, and the earlier 19200 bit/s
reading (8 data bits plus one trellis bit at 2400 baud) now rests on a count
(509-525 sites) and not only on a power estimate.

Limits: the cluster spread after unwarping is still 0.24 of the spacing, so
this is a threshold count, not a resolved one; the warp is a two-parameter
polynomial fitted by score, not a recovered formula (V.34's nonlinear
encoding is a similar idea, but nothing here identifies the function); and
trellis, rate-to-bits mapping and the B13 = 16800 reading are still inferred.

### Against V.34's nonlinear encoding (2026-10-12)

V.34's nonlinear encoder (clause 9, implemented in `v34tx.c` and exercised by
`v34_data_test.c`/`v34_phase4_16pt_test.c`) maps each point x to
`y = x * phi(zeta)`, `zeta = 0.3125 |x|^2 / E`, `phi = 1 + zeta/6 + zeta^2/120`
(the series of sinh(sqrt zeta)/sqrt zeta), with E the average energy of the
unwarped constellation. Tested as a one-parameter family `zeta = c r^2`
against the terbo data (`demod/v34nl*.py`, `v34curve.py`):

**The shape fits.** Measured spacing ratio to the centre (unclipped, Hann-
windowed local fit along central rows; the control gives 1.000 +-0.004 at every
radius) against the V.34 family at its best c = 0.48:

| radius (equalizer units) | 0.30 | 0.60 | 0.90 | 1.15 |
|---|---|---|---|---|
| measured, radial | 1.020 | 1.089 | 1.178 | 1.275 |
| V.34 form, c = 0.48 | 1.021 | 1.083 | 1.178 | 1.278 |
| V.34 form, c = 0.377 (the V.34 value, below) | 1.017 | 1.066 | 1.144 | 1.226 |

Four points, one parameter, agreement to 0.5 percent. The lattice-score fit
agrees: the best one-parameter V.34 family scores 0.614 (c = 0.46-0.54) against
0.616 for the free polynomial. The tangential spacing, measured more roughly,
is 1.015, 1.015 and 1.104 at 0.3, 0.6 and 1.15 against 1.007, 1.027 and 1.091
predicted at c = 0.48.

**The strength does not match V.34's constant.** V.34 fixes `c = 0.3125 / E`.
Measured on the unwarped constellation, E = 0.83 (a 512-point disc of radius
12.75 cells x 0.1010 = 1.288 has E = R^2/2 = 0.829; the unwarped data agree),
so V.34 predicts **c = 0.377** and a rim zeta of 0.625 for any equiprobable
disc. The fits want **c = 0.48 (0.46-0.50)**, rim zeta about 0.80, i.e. an
effective constant `c x E` of **0.40 +-0.03** instead of 0.3125, 27 percent
stronger. At c = 0.377 the radial misfit is 0.0044 in squared ratio against
0.0002 at the optimum, about twenty times worse, and the errors are 3-5
percent where the estimator is good to about 0.5 percent. The lattice score
alone cannot make this call (0.594 against 0.614, about 1.5 sigma), the
curves can.

So: **the same nonlinear-encoding family as V.34, with a larger constant
(about 0.40, perhaps 2/5)**. Not excluded: a bias in the local-spacing
estimator on a chirped lattice that the flat control cannot reveal, or an
equalizer scale error (it would shift r and c together); and the 0.40 is one
measurement from one call. The next test is a second recording, and the
V.34 field that signals the encoder in MP (Table 23's bit 31) has no terbo
counterpart that I know of.

### Second recording (2026-10-12): the warp constant replicates

A fresh terbo call, recorded with `EXT=0x4000 PROBE=1 call3.py 2 0 45 0 4.5`:
caller started at 4.5 s instead of 3 s, **B14 only** instead of B13+B14, so
the timing (TRN2 at 10.105 s instead of 8.607 s), the echoed word and the
random payload all differ. The caller framed R3 `0xC990`, answered E
`0xC99F`, and MICA answered `0xC99F`, exactly as before. `demod/gen.py` now
finds TRN2 and the lattice rotation itself (it reproduces the first
recording: TRN at 8.607 s, MSE 0.00017). Second recording: TRN match 300 of
300, training MSE 0.00017, lattice score 0.43 before refinement.

Radial spacing ratio to the centre, same windowed fit as before
(`demod/curve2.py`; four estimates per radius, x and y, both signs):

| radius | 0.30 | 0.60 | 0.90 | 1.15 | best V.34-family c |
|---|---|---|---|---|---|
| recording 1, original analysis | 1.021 | 1.084 | 1.178 | 1.272 | 0.47 (0.46-0.49) |
| recording 1, generalised pipeline | 1.025 | 1.089 | 1.184 | 1.279 | 0.49 (0.47-0.50) |
| recording 2 | 1.022 | 1.084 | 1.182 | 1.279 | 0.48 (0.47-0.50) |

Three fits of c from two different calls agree to 0.02, against V.34's
prediction of 0.377. The scale-independent product kappa = c x E (E is the
mean energy of the unwarped points, so an equalizer scale error cancels):

| | c | E (unwarped) | kappa = c x E |
|---|---|---|---|
| recording 1 | 0.49 | 0.826 | 0.405 |
| recording 2 | 0.48 | 0.827 | 0.397 |
| V.34 | | | 0.3125 |

So the effective constant is **0.40 +-0.01, 27 percent above V.34's 0.3125**,
and it is not an equalizer-scale artefact (kappa is invariant to it). That
disposes of the second explanation I left open; a bias in the local-spacing
estimator on a chirped lattice is still not excluded, but the estimator
reproduces a flat control and the same curve in two independent calls.

The constellation checks replicate too, and recording 2 is cleaner:

| | recording 1 | recording 2 |
|---|---|---|
| lattice score after unwarp | 0.62 | 0.77 |
| sites >= 0.25 / 0.35 / 0.5 of the mean count (27.3) | 525 / 517 / 509 | **512 / 512 / 512** |
| sites >= 0.65 | 494 | 499 |
| rows kept | 26 | 26 |
| farthest site per 30-degree sector (cells) | 12.7-12.8 | 12.7-12.7 |
| mean count per site, five radial bands | 26.3 27.1 27.1 28.0 25.8 | 26.3 27.1 27.1 28.2 26.8 |

A plateau of exactly 512 over a threshold range from 0.25 to 0.5 is a direct
count, not an estimate: **the terbo data constellation has 512 points**,
equiprobable, in a circular region of radius 12.75 cells, radially warped by
V.34's nonlinear-encoding function with kappa about 0.40.


### The terbo data is trellis-coded: 8 data bits + 1 code bit (2026-10-13)

Test (`demod/trellis*.py`, run on both recordings): unwarp with c = 0.48/0.49,
slice the 14000 equalized symbols to integer lattice sites (0.35 / 0.77
percent fall off the 512-site set), label each by its coset of an index-N
sublattice of Z^2, and measure block entropy of the label sequence
(Miller-Madow corrected).

- **No code at depth 2, a code at depth 3-4.** Every coset label is uniform and
  memoryless to three symbols (index 4: 2.000 bit, index 8: 2.997-3.000 of
  3.000, all 15 index-8 sublattices tried). Only the standard Ungerboeck
  partition Z^2 > 2RZ^2 (index 8, `L=<(2,2),(0,4)>`) drops at the fourth
  symbol: H(4th | previous 3) = **2.20 bit** (recording 2) and 2.52 (recording
  1, noisier), against 3.00 for the other partitions. A synthetic 8-state
  systematic code run through the same estimator gives 2.00, so the method is
  sensitive; a 0.35 percent slicing error rate accounts for the excess over 2.
- **What is constrained is the checkerboard parity** p = (i+j) mod 2 (the
  Z^2 > RZ^2 split). Given the previous three labels the next label lies in
  a set of about four labels (439 of 512 contexts, recording 2), and the
  supports seen most often are exactly the two parity classes {0,2,5,7} and
  {1,3,4,6} (438 of the 2570 four-label contexts show a clean one). p given the previous
  1 or 2 labels: 49 / 47 percent error (chance); given 3: **4.4 percent**
  (recording 1: 13.6), given 6: 0.2 (0.5). It is a function of the past,
  with a memory of three symbols.
- **Which bits matter.** Reducing each lagged label to a coarser class shows p
  depends on: the parity of the label 3 back; (a, b0) of the label 1 back; and
  (p, b1) of the label 2 back, where a = i mod 2 and b = (j - 2 floor(i/2))
  mod 4 = b0 + 2 b1. That is five bits of the past, 1 + 2 + 2, not a simple
  GF(2) sum in this basis (best linear fit only 73 percent: the labels form
  Z2 x Z4, so the differential encoding is mod 4).

So the 512-point constellation carries **one redundant (code) bit per symbol:
8 data bits x 2400 baud = 19200 bit/s**, and the B14 = 19200 reading no longer
rests on an assumed trellis. The 21600 alternative (no code) is excluded.
B13 = 16800 (7 data bits, 256 points + code bit) is still inferred; no
recording has used it.

Not established: the code's exact form (state count, polynomials, the
differential encoding of the data bits). The five-bit dependence is
consistent with an Ungerboeck-style coder on top of a mod-4 differential
encoder like V.32bis's, but I have not fitted one; V.32bis's own coder with
its 128-point lattice is the obvious first candidate to adapt. The scrambler
is not accounted for either (it sits before the coder, so it does not affect
this test).


### The coder is V.32bis's 8-state coder (2026-10-13)

Fitting (`demod/minimize.py`, `replay_rec1.py`, `cmp_map.py`). Not found by
guessing: a first fit of the spandsp table to the labels at chance (50 percent)
because it assumed a particular output bit and a fixed label-to-dibit map.
Found by inference instead:

1. Label each sliced symbol with Ungerboeck bits (z0, z1, z2) for the chain
   Z^2 > RZ^2 > 2Z^2 > 2RZ^2 (z0 is the checkerboard parity).
2. The next z0 is a deterministic function of the previous three full labels
   (128 histories; 620 of 13997 symbols disagree with the majority, the slicing
   noise).
3. Moore-minimise that function over the 8 possible next labels: **exactly 8
   states**, consistent for all 128 histories. Output z0 is a function of the
   state alone; the other two label bits (z1, z2) drive the transition, four
   ways from every state.
4. Compare with the V.17 / V.32bis coder in `spandsp-master/src/v32bis.c`
   (`convolutional[8][4]`, `s' = conv[s][d]`, Y0 = bit 2 of s' = bit 0 of s).
   The recovered machine is **isomorphic to it**: four isomorphisms exist
   (they differ only by the unobservable initial state and Y0 polarity), and
   they require the dibit d to be assigned to the lattice cosets with a
   different permutation in the Y0 = 0 and Y0 = 1 classes. With a single
   permutation for both classes there is no isomorphism, and the graph does not
   match with output bits 1 or 2 either.
5. Replaying `conv` directly on the sliced labels predicts Y0 correctly for
   **94.6 percent of 14000 symbols (recording 2) and 86.2 percent (recording 1,
   noisier slicing, after a 90-degree lattice rotation)**, against 51 percent
   for the same labels in shuffled order. The shortfall is the model's recovery
   after a mis-sliced symbol (memory three), not a different code: the 128
   history rows have no inconsistent entry.

So the terbo trellis is V.32bis's 8-state rate-2/3-style convolutional coder
operating on a 512-point lattice, with 8 data bits + 1 code bit = 19200 bit/s
(B14).

What differs from V.32bis, and what is not shown:

- V.32bis's own subset labelling is not periodic on any index-8 sublattice (its
  (d, Y0) labels are not a function of any coset: `demod/cmp_map.py`), whereas
  MICA's lattice is labelled by cosets of 2RZ^2. So the point-to-label map is not
  the V.32bis one, even though the coder is. The per-class dibit permutations
  are determined only up to the four isomorphisms.
- The differential encoder in front of the coder (V.32bis Table 3) and the
  scrambler are not yet checked; the replay uses d as read off the labels, so
  it tests the convolutional part only. Undoing the differential encoder is the
  next step, and the first test that the descrambled bit stream is sensible.
- Initial state, and the mapping of the remaining uncoded bits (the other 6 of
  8 data bits) to the position inside a coset, are unexamined.

## Next steps, in order of what could actually be lifted

1. Disassemble the entry of each unidentified pair and find its scrambler
   taps and carrier/baud setup (the method that identified 8E: its 8990 word,
   8880/888F headers and 5/23, 18/23 descramblers). 7E/7F, 80/81, 8A/8B and
   90/91 are the candidates for V.32terbo, V.22bis and V.34. That settles
   whether MICA holds a V.32terbo datapump we could lift.
2. Done (see above). V.32terbo is real in MICA: an extended R word and a
   changed block of DSP configuration, selected by S29 = 2.
3. Next: make the caller offer the extra R bits (a patched copy of the vendored
   `v32bis.c` rate-signal code) and see whether MICA, at S29 = 2, then answers
   16800/19200, and what its data-mode signal looks like. Only then decide on a V.32terbo datapump. The rate signalling (R words
   above 14400) is not in any ITU document here, so without MICA or Courier
   code we cannot write it from a spec.
4. HST and V.FC have no overlay in MICA's list; the Courier DSP image is the
   only source and its base address is unresolved
   (`courier_firmware_analysis.md`, "Datapump is present").
5. MNP 2-5 is specified publicly and can be written independently; it is the
   one gap here that needs no reverse engineering.
