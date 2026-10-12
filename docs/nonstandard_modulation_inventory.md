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

## Next steps, in order of what could actually be lifted

1. Disassemble the entry of each unidentified pair and find its scrambler
   taps and carrier/baud setup (the method that identified 8E: its 8990 word,
   8880/888F headers and 5/23, 18/23 descramblers). 7E/7F, 80/81, 8A/8B and
   90/91 are the candidates for V.32terbo, V.22bis and V.34. That settles
   whether MICA holds a V.32terbo datapump we could lift.
2. Run MicaEmu with `%15S2` set (V.32terbo automode) and watch which loader
   stub fires; the stub identifies the overlay without disassembly.
3. Only then decide on a V.32terbo datapump. The rate signalling (R words
   above 14400) is not in any ITU document here, so without MICA or Courier
   code we cannot write it from a spec.
4. HST and V.FC have no overlay in MICA's list; the Courier DSP image is the
   only source and its base address is unresolved
   (`courier_firmware_analysis.md`, "Datapump is present").
5. MNP 2-5 is specified publicly and can be written independently; it is the
   one gap here that needs no reverse engineering.
