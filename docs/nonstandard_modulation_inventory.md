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
| 8.4-8.6 s | S, S-bar | same |
| 8.61-10.48 s | TRN, then R1 | same, identical audio |
| 10.456 s | audio diverges (our R2 reaches MICA) | |
| ~10.5 s | R3 `0x9990` x16, E, then dense data | R3 = **`0xC990` x502** (4016 symbols, 1.67 s) |
| ~12.1 s | | MICA gives up on R3 and goes to a dense signal |

`0xC990 ^ 0x9990 = 0x5000`: **the 14400 bit (B12) is cleared and B14 is set.**
So in terbo mode R3 names the extended rate by B14 instead of B12. Offering
B13 alone, B14 alone or both (`EXT` = `0x2000`, `0x4000`, `0x6000`) gave the
same MICA behaviour, and its power after 20 s was identical, so MICA picked
the same rate each time; with only B13 offered it still answered B14, which
means the answerer chose its own best terbo rate rather than echoing ours.
The most natural reading is B14 = 19200 and B13 = 16800 (higher rate on the
higher bit), but **that mapping is inferred, not observed**: nothing here
has measured the data rate.

MICA repeats R3 until it sees E and, getting none (SpanDSP does not accept
`0xC990` as a valid rate signal, so the caller never sends E), times out
after about 1.67 s. The caller side therefore stalls at R3, not at R1/R2: the
gate (a terbo bit in R2) and the answer (`0xC990`) are both understood now.

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
