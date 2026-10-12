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
| `2BE7` | 7E / 7F (5,798 B) | `C8BB` | **no** |
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

### Injected callers (same day)

A spandsp calling modem was fed to the answerer through the linked C53's G.711
queue (`tools/mica_overlay_probe/`), ATS29=2 against ATS29=3, 500-800M
instructions (about 50-75 s of line time) each:

1. a V.32bis caller sending AC from the start (`inj.py`);
2. a V.8 caller offering only V.32/V.32bis (`v8inj.py`).

Result: **the two settings are indistinguishable.** Loader sequence, cycle
counts (8E/8F at 18.50M, 7E/7F at 151,088,145) and the channel handler
timeline (`34870`, `351e0`, `353a0`, `359b0`, `37f80`) are identical. Neither
caller connected: the V.8 handler never fired and the V.32bis caller never
completed start-up.

What this does not show: that the answerer cannot do V.32. The answerer is
silent for 4 s and then steps through several tone phases that I did not
identify (the spectra are mirrored about 2 kHz, which I do not understand and
may be a decoding or sampling artefact). I could not confirm it ever hears the
injected audio or that the line model supports an answer-side
negotiation. Treat this as "no terbo-specific difference found", not "no
terbo".

## Next steps, in order of what could actually be lifted

1. Disassemble the entry of each unidentified pair and find its scrambler
   taps and carrier/baud setup (the method that identified 8E: its 8990 word,
   8880/888F headers and 5/23, 18/23 descramblers). 7E/7F, 80/81, 8A/8B and
   90/91 are the candidates for V.32terbo, V.22bis and V.34. That settles
   whether MICA holds a V.32terbo datapump we could lift.
2. Done with and without injected callers (see above): no difference between
   S29=2 and 3. Before repeating, show that the answerer hears injected
   audio at all (a known tone into the receiver, observed in DSP state), and
   explain the 4 s of silence and the mirrored tone phases.
3. Only then decide on a V.32terbo datapump. The rate signalling (R words
   above 14400) is not in any ITU document here, so without MICA or Courier
   code we cannot write it from a spec.
4. HST and V.FC have no overlay in MICA's list; the Courier DSP image is the
   only source and its base address is unresolved
   (`courier_firmware_analysis.md`, "Datapump is present").
5. MNP 2-5 is specified publicly and can be written independently; it is the
   one gap here that needs no reverse engineering.
