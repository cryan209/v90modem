# Courier V.Everything Firmware Analysis — `SV25.XMD`

Analysis of the USRobotics Courier firmware image we've been testing against, to
support the V.90 interop effort (esp. the "Courier retrains to Tone A right after
our DIL, never sends CPt" blocker — see `courier-retrains-after-dil` memory).

## File facts

| | |
|---|---|
| File | `docs/SV25.XMD` |
| Size | 524,416 bytes = `0x80` header + `0x80000` (512 KB) image |
| Date | 1998-03-13 |
| MD5 | `872b9696eca7156ba22f1c874601b874` |
| Product | **USRobotics Courier V.Everything** (banner at file `0x263B7`) |
| Type | **SV = Supervisor** image, SDL/XMODEM-downloadable (`.XMD`) |

## Processors (confirmed by the modem's own `ATUSR` easter egg)

```
INT80188 Modem Functions      -> supervisor CPU = Intel 80188 (16-bit x86)
TMS32025 DSP Functions         -> datapump DSP  = TI TMS320C25 (16-bit fixed-point)
Copyright (c) 1988..1992 USRobotics
```

So `SV25.XMD` (minus the 0x80 header) is **exactly 0x80000 = 512 KB = the full
flash** — it contains BOTH the 80188 supervisor AND the TMS320C25 datapump.

## Image map

| file range | ~size | contents |
|---|---|---|
| `0x00000–0x00080` | 128 B | header: signature/checksum + `NHCFG` tag + `0x4A` fill |
| `0x00000–0x29800` | ~166 KB | **Intel 80188 supervisor** (x86-16 byte code) + string/resource tables |
| `0x08000–0x0C000` | ~16 KB | word-structured pockets (DSP tables / small overlay) |
| `0x29800–0x44000` | ~106 KB | **TMS320C25 DSP datapump** (16-bit words: code + tables + overlays) |
| `0x44000–0x7C000` | ~224 KB | `0x00` fill (unused) |
| `0x7C000–0x7E000` | ~8 KB | DSP coefficient table (int32, ~-774 default taps) |

Boundary is sharp: even/odd byte-distribution divergence (a word-vs-byte-code
discriminator) is ~0.4 in the 80188 region and jumps to **0.68–0.79** at exactly
`0x29800`. The C25 words are **big-endian** (high byte first). The DSP `B`/`CALL`
(`FF80`/`FE80`) targets are consistent 16-bit code addresses; several exceed the
region size, so the datapump is built as **multiple overlays** the supervisor
swaps in per phase (V.34 startup / V.90 / command). **The DIL-analysis and
constellation-design code — the logic that decides to send CP vs. abort to
retrain — is TMS320C25 code in `0x29800–0x44000`.**

## Format — NOT encrypted, NOT compressed

Initial entropy (~7.8) and poor gzip ratio (~83%) *looked* like compression, but
that's a red herring: the payload is **plaintext, uncompressed** code. The 80188
half is 16-bit x86; the high entropy comes from dense 5-byte far-calls
(`9A seg:off`) interleaved with tables, not a transform. Disassembly heuristics
(branch coherence) can't tell x86 code from data — 16-bit x86 is so dense that
even random bytes score as "coherent" — so region typing relied on the even/odd
word-structure metric above, not on disassembly.

Proof — disassembly at the banner-print site (file `0x263AF`):

```
0263af: 9a 41 9e 00 80   lcall 0x8000:0x9e41     ; far call into bank at seg 0x8000
0263b4: e8 6f 17         call  0x27b26           ; "print inline ASCIIZ" routine
0263b7: "USRobotics Courier V.Everything",0       ; inline string (call-then-db idiom)
0263de: a0 29 01         mov al,[0x129]          ; prints a 3-byte version as
0263e1: 9a d6 9d 00 80   lcall 0x8000:0x9dd6     ;   [0x129]:[0x128]:[0x127]
0263e8: 9a 13 16 00 80   lcall 0x8000:0x1613     ; print-char routine
```

### Layout
- `0x00000–0x00080` — 8-byte signature/checksum (`b7 f0 b2 fd d8 ec bc bd`) + `NHCFG` tag + `0x4A` fill
- `0x00080–0x44000` — active code + data + DSP tables (bank-switched x86)
- `0x44000–0x7C000` — `0x00` fill (unused)
- `0x7C000–0x7E000` — DSP coefficient table (see below)
- tail — mostly `0xFF`

### Quirks
- **Bank switching**: a 512 KB ROM can't fit one 64 KB segment. Near calls (`E8`)
  stay in-bank and disassemble linearly; far calls (`9A 8000:xxxx`) page into
  another bank. The bank→file mapping is **not yet solved** (naive `seg*16` and
  `+0x80` header offsets both fail), so cross-bank call resolution needs work.
- **String obfuscation**: some UI strings are stored **case-toggled** (XOR `0x20`
  on letters only) — e.g. credits at `0x100` read `jOHN wIERSEMA`. Others (the
  banner, diagnostics) are stored normal-case. To read reliably, per-run pick
  whichever of {identity, case-toggled} has more lowercase letters.

## Extracted content (confirmed)

### V.90 downstream rate table (file `0x019B02`) — the far-end DS0 rate grid
```
32 → 48000 bps   36 → 53333 bps
33 → 49333 bps   37 → 54666 bps
34 → 50666 bps   38 → 56000 bps
35 → 52000 bps   39 → 57333 bps
```
1333 bps grid — the exact rate codes the Courier maps for V.90 downstream.

### V.34 trellis code set (file `0x018D84`)
`8S-2D`, `16S-4D`, `32S-2D`, `64S-4D` trellis codes.

### Link-diagnostic vocabulary — the ATI6/ATI11-style readouts (file `~0x01CC00`)
```
V.90 Status                 (file 0x01C38D)
Far Echo Loss ... dB
Roundtrip Delay ... msec
Timing Offset ... ppm
Carrier Offset ... ppm
RX Upshifts / RX Downshifts
Constant Carrier / Carrier Usage
```
**This is the most immediately useful find** — see "How this helps the blocker".

### DSP coefficient table (file `0x7C080`)
Little-endian int32, mostly `-774` repeated, then `-781, -1087, -915, -901, ...` —
fixed-point constants loaded into the datapump DSP (default/init taps).

### Other
- Country/config list: Sweden, New Zealand, Belgium, Denmark, Austria, International, …
- "Lowest Speed Limit", "SDL Xmodem file transfer — (Y)es (N)o (T)est", reset/dial UI
- Developer credits (`0x100`): John Wiersema, Glenn Bushey, Kevin Lacey,
  Brinn Gilbert, Doug Blatt, Art Johnson, Pete Jankus, Bill Pierce, Ken Starkey, Joe F…
- Version is printed at runtime from RAM `[0x127..0x129]` as an X:Y:Z triplet
  (not a literal string in-image; set during init).

## Architecture summary

- **Supervisor**: Intel 80188 (16-bit x86). Uncompressed, plaintext, disassemblable
  as x86-16 — handles AT/S-registers, protocol sequencing, status/diagnostic
  displays, and DSP overlay loading. Occupies `0x0–0x29800`.
- **DSP datapump**: TI TMS320C25, big-endian 16-bit words, at `0x29800–0x44000`
  (~106 KB) plus tables at `0x08000–0x0C000` and `0x7C000`. Confirmed executable
  (sane `B`/`CALL` targets), organized as multiple overlays. **This is where DIL /
  constellation / retrain logic lives.** capstone has no C2x support — needs a
  TMS320C2x disassembler (hand-rolled Python, or IDA's TMS320C2x processor module,
  or a Ghidra C2x SLEIGH module).

## How this helps the "retrain after DIL" blocker

The retrain-vs-send-CP decision is split:
- **Protocol sequencing / status** → supervisor (disassemblable, reachable).
- **DIL analysis + constellation design** (the part likely *rejecting* our DIL)
  → DSP datapump (harder to reach).

**Immediate, no-further-RE win:** the diagnostic vocabulary above means the
Courier reports its *own* measurement of our downstream signal. During a live
attempt, `ATI6` / `ATI11` (and the "V.90 Status" display) expose **Timing Offset
(ppm)**, **Carrier Offset (ppm)**, **Roundtrip Delay**, **Far Echo Loss**, and
**Up/Downshift** counts right up to the moment it bails. That distinguishes:
- a **timing/carrier-offset** problem → points back at our clock recovery / DPLL
  (`clock_recovery.c`) and RTP sample-rate accuracy, vs.
- a genuine **constellation/impairment** rejection of our DIL.

## Container format (SDL/XMODEM download)

`524416 = 4097 × 128` — exactly 4097 XMODEM blocks. Block 0 (`0x00–0x80`) is the
SDL download header (checksum bytes + `NHCFG` tag + `0x4A` fill). The remaining
4096 blocks are the 512 KB flash image. **flash offset = file offset − 0x80.**
(Sent to the modem with `ATXMODEM`.)

## 80188 memory architecture — the blocker for reading resident/overlay code

Tooling built and working:
- **`c25dis.py`** (scratchpad) — faithful port of MAME's TMS320C25 disassembler
  (Tony La Porta table). Big-endian words. Verified correct.
- x86-16 **recursive-descent** from the confirmed banner code — follows only real
  control flow (avoids the "x86 decodes anything" trap).

What the recursive descent established:
- The banner code (flash `0x26337`) runs **from flash** and makes far calls to a
  **resident code segment `0x8000`** (26 distinct target offsets `0x0A2C…0xF42C`,
  ~47 call sites). There is **no per-call bank-select `OUT`** before them.
- `segment 0x8000` does **not** map linearly to flash `0x0`, `0x10000`, … (every
  base tried yields garbage at the known routine offsets). Combined with the spec
  ("512K Flash, **64K RAM**"), the model is: **segment `0x8000` = the 64 KB RAM**,
  into which resident code is copied from flash by the C-runtime **boot code** —
  which is assembled from multiple flash sources, not one contiguous copy.
- Hardware interface seen in real code: immediate `IN` from ports **`0x19`, `0x3E`**;
  `OUT` via `DX` at flash `0x26390/0x2D690`. Candidate DSP/UART/mailbox ports.

**Why we're blocked:** to disassemble the resident routines (`8000:xxxx`) or the
DSP overlay loader, we need the flash→RAM/overlay address mapping. That comes from
the boot code, but the 80188 reset vector is **not** at file `0x80070` (flash
`0x7FFF0`, top-mapped) as expected — it's not a standard `EA` far jump there. So
the flash is bank-switched / stored in non-linear order at coarse granularity, and
recovering the map needs the boot/bank-config code, which heuristics can't locate
(x86 decodes arbitrary bytes, so code vs. data can't be told apart statistically).

## Datapump is present & readable — but needs human-driven RE

Confirmed by decoding real code: the DSP region contains genuine V.34/V.90
datapump code, e.g. at file `0x040F98`:
```
rptk B5h ; mac $4450,*0+,AR3     ; 182-tap MAC, coeffs @prog 0x4450
rptk B5h ; mac $5450,*0+,AR3     ; another 182-tap filter @0x5450
mac  $2756 ; blkp $CD2F ; macd $37C4 ; macd $28C4 ...
```
That's a **bank of 182-tap correlators / matched filters** — the equalizer /
DIL-correlation machinery. The `mac` coefficient tables (e.g. file `0x320A0` =
prog `0x4450`) hold 182 large-amplitude signed values — consistent with **DIL /
probing reference sequences**, a strong candidate for the DIL-correlation input.

**What automation can't do here** (verified, don't re-attempt):
- Pin the DSP load/base address. `file = 0x29800 + 2·prog` is only a *hypothesis*:
  it gave locally-plausible hits, but `prog 0x0000` isn't a reset branch, and
  recursive-descent produces identical floods under correct and wrong bases
  (~46K insns, 7.9% invalid, 2 rets either way). C25 decodes arbitrary bytes, so
  code-vs-data and base cannot be resolved statistically.
- Isolate the specific DIL/constellation-design routine. Needs a person who can
  recognise routine boundaries, coefficient tables, and algorithm shape.

**Recommendation:** load `SV25.XMD` into **IDA** (has a TMS320C2x processor
module) or Ghidra, region `0x29800–0x44000`, big-endian words, and drive it
manually — seed from the FIR bank at `0x040F98` and the reference tables at prog
`0x4450/0x5450`. Everything above (region, endianness, verified code site,
candidate DIL tables, working `c25dis.py`) is the head-start. Alternatively, a
live DSP dump from the modem sidesteps the base/overlay problem entirely.

## Possible next steps

1. **Live RAM/state dump from the physical modem** (highest leverage): if the
   Courier exposes any memory-peek / debug-monitor capability, dumping segment
   `0x8000` (RAM) gives the resident code directly, and dumping DSP program RAM
   during a V.90 handshake gives the *active* overlay — sidestepping the whole
   static bank-mapping problem. Worth probing what debug AT commands this firmware
   supports (there are undocumented USR ones).
2. **Interactive disassembler + human loop**: load the flash into IDA/Ghidra (IDA
   has a TMS320C2x module) and resolve the boot/bank config manually, starting from
   the verified banner anchor and the port `0x19/0x3E` I/O. This is patient,
   multi-session RE — not automatable here because of the decode-anything problem.
3. **Mine the DSP data tables** (tractable now): the datapump region is
   data-dominated — extract the constellation/interleaver map (`flash 0x3B780`) and
   coefficient tables, which describe the constellation machinery without needing
   code RE.
4. **Correlate live** (needs a connected call): `ATI6`/`ATI11` Timing/Carrier
   Offset + echo vs. our DIL timing, to fork clock-recovery vs. constellation
   rejection.

## Repro notes

Analysis tooling: Python + `capstone` (x86-16). macOS blocks reads under
`~/Downloads` (TCC) — the file had to be copied into the repo first.
