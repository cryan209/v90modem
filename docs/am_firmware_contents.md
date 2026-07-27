# What's in the te_dmlt.am firmware?

## Short answer: V.90, not V.92

The `te_dmlt.am` firmware is the **analog-card modem firmware**, build
**122-11, Protocol 6.03(V24)** (per its version string
`TE_DMLT, Build 122-11, Protocol 6.03(V24) 111-1 [F#03FF]`). It's newer
than the PRI `te_dmlt.pm` (which pairs with `dspdload.bin` build 117-926)
but it is **V.90-class, not V.92**.

### Evidence

- Firmware debug strings reference `V.90 fallback`, `V.90 recovery`,
  `V.34 fallback`, `V.34FAX`, `V.8 incompatibility` — V.90/V.34 era.
- The string `[%s,%s] V.90A not supported` indicates V.90A (APCM, the
  analog-side V.90) awareness but not V.92.
- **No V.92 / V.PCM / PCM-upstream / Modem-on-Hold / V.44 / V.91 strings**
  anywhere in the firmware. A V.92 implementation would reference these
  prominently (PCM upstream, MOH, V.44 compression).
- The DSP combifile (`dspdload.bin` build 117-926) that supplies the DSP
  overlays has **V.90 DPCM (0x026a) and V.90 APCM (0x026b) overlays** but
  no V.92 overlay. The full download list (164 entries) contains zero
  V.92/V.PCM/V.44/V.91 entries.

## What .am is for

The `.am` (vs `.pm`) is the **analog-line variant** of the same modem
firmware family:

| File | Card | Size | Build | DSP combifile |
|---|---|---|---|---|
| `te_dmlt.pm` | PRI 30M (T1/E1, 30 DSPs) | 0.98 MB | 117-926 | `dspdload.bin` |
| `te_dmlt.am` | Analog (1-8 DSPs) | 2.2 MB | 122-11 | `dspdload.bin` (same) |

The `.am` is bigger because the analog card runs the full modem stack
(ATA/AT-command interpreter, V.42/V.42bis, fax T.30, voice) on the MIPS,
whereas the PRI card offloads more to the host. Both use the same ADSP-2181
DSP and the same `dspdload.bin` overlay set.

## The analog overlay set (from dspdload.bin)

The analog card uses the `.ANA` overlay variants (single-channel codec,
not T1/E1 multichannel):

| ID | Overlay | Role |
|---|---|---|
| 0x000d | DIVA Server Analog Kernel | single-DSP kernel (1 channel) |
| 0x0258 | TIKRNL81.ANA Task | task kernel (analog) |
| 0x0262 | DIAL/FSK/FAX.ANA | DIAL bootpage (analog) |
| 0x025f | V8.ANA Overlay | V.8 negotiation |
| 0x0261 | V34.ANA Overlay | V.34 |
| 0x026a | V.90 DPCM Overlay | V.90 digital-side (PCM downstream) |
| 0x026b | V90.ANA APCM Overlay | V.90 analog-side (PCM upstream) |
| 0x0260 | INFO.ANA | V.34 phase 2 info |
| 0x026e | INFOH.ANA | info helper |
| 0x026f | HV34.ANA | hybrid V.34 |
| 0x0266 | V22V32.ANA LEC | V.22/V.32 with echo cancellation |
| 0x0271/0x0275 | V22FC/V22bisFC.ANA | V.22 fast-connect |
| 0x0273 | V29FC.ANA | V.29 fast-connect (fax) |

The bootpage sequence is the same as documented in `addspv90guide.pdf`:
STARTUP(9) → DIAL(0) → V.8(6) → INFO(7) → V.34(8)/V.90(13/14).

## V.90 vs V.92 — the distinction

- **V.90** (1998): 56 kbit/s downstream (PCM), 33.6 kbit/s upstream
  (analog). The `DPCM` overlay = digital-side (downstream PCM),
  `APCM` = analog-side (upstream analog). This is what `.am` implements.
- **V.92** (2000): adds PCM upstream (up to 48 kbit/s), Modem-on-Hold,
  quick connect, V.44 compression. The Eicon/Dialogic firmware here
  predates V.92 — build 122-11 / protocol 6.03 is late-1990s V.90 era.

The project's own V.92 work (`v92_*.c`, `docs/v90_mi_negotiation.md`) is a
*new* implementation; the Eicon firmware is a reference V.90 only.

## Implication for the emulator path

Using `te_dmlt.am` + the Analog kernel (0x000d) + CardType=92 is still the
right path to get `dsp_assign` running with single-DSP geometry — it just
won't give us V.92 to compare against. It gives us a complete, shipping
V.90 (DPCM + APCM) implementation to validate our V.90 against, running on
the same ADSP-2181 the project targets.
