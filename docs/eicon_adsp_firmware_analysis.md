# Eicon/Dialogic ADSP V.34/V.90 firmware extraction

Initial extraction notes, 2026-07-26.

## Why this firmware matters

The files under `docs/firmware/` are Eicon/Dialogic DIVA DSP download
combifiles. They contain separately named, relocatable modem overlays rather
than one unidentified flat image. Two versions have been verified:

| Combifile | Build | Downloads | Relevant overlays |
|---|---:|---:|---|
| `dspdvmdm.bin` | 103-492 | 78 | V.34, V.90 DPCM, V.90 APCM |
| `dspdload.bin` | 117-926 | 164 | V.34, V.90 DPCM, V.90 APCM |

The architecture is ADSP-218x, not the Courier's TMS320C25. Evidence:

- `docs/addspv90guide.pdf` describes the matching Telindus V.90 package on the
  ADSP-218x family and its V.34/V.90 overlay organization.
- firmware task names include `TIKRNL81`, consistent with the ADSP-2181 target;
- program-memory records contain 24-bit ADSP instructions in 32-bit download
  containers;
- the public Eicon `dsp_defs.h` identifies the exact combifile structures and
  the `DSP_DOWNLOAD_FLAG_2181` flag.

This is useful as an independent shipping V.34/V.90 implementation. It does
not directly reveal the Courier's decisions, but its Phase 3 receiver,
DIL-analysis and V.90 sequencing can be compared with our implementation and
with the Courier firmware.

## Extractor

`tools/eicon_dsp_extract.py` parses the complete combifile directory and every
nested download. By default it extracts the three generic V.34/V.90 overlays:

```bash
./tools/eicon_dsp_extract.py docs/firmware/dspdvmdm.bin \
  -o artifacts/eicon-dsp/build-103-492

./tools/eicon_dsp_extract.py docs/firmware/dspdload.bin \
  --match 'V\.90 APCM Overlay' -o artifacts/eicon-dsp/build-117-926
```

List all contained downloads without writing images:

```bash
./tools/eicon_dsp_extract.py docs/firmware/dspdload.bin --list
```

Select other variants, such as the analogue-card overlays, with `--match`:

```bash
./tools/eicon_dsp_extract.py docs/firmware/dspdload.bin \
  --match 'V34\.ANA|V90\.ANA' -o artifacts/eicon-dsp/ana
```

Each selected download produces:

- `download.bin` — exact nested download record;
- `dm.bin` — complete 16-bit DM address space, little-endian;
- `pm.bin` — complete PM address space, packed little-endian 24-bit words;
- `dm.words` / `pm.words` — only populated addresses, suitable for conversion
  to another disassembler format;
- `metadata.json` — file offsets, hashes, segment relocation, block maps and
  resolved relocation records.

The binary images fill unpopulated addresses with zero. Use the `.words` maps
or block metadata when loaded zeroes must be distinguished from gaps.

## Verified extraction results

Build 103-492:

| Overlay | ID | File offset | Record bytes | DM words | PM words |
|---|---:|---:|---:|---:|---:|
| V.34 | `0x0261` | `0x5ce96` | 60,394 | 9,248 | 10,345 |
| V.90 DPCM | `0x026a` | `0x8c9f0` | 57,535 | 8,112 | 10,199 |
| V.90 APCM | `0x026b` | `0x9aab2` | 57,239 | 8,674 | 9,824 |

Build 117-926:

| Overlay | ID | File offset | Record bytes | DM words | PM words |
|---|---:|---:|---:|---:|---:|
| V.34 | `0x0261` | `0x139282` | 61,594 | 9,320 | 10,605 |
| V.90 DPCM | `0x026a` | `0x1de5ac` | 58,515 | 8,098 | 10,443 |
| V.90 APCM | `0x026b` | `0x1eca42` | 57,419 | 8,692 | 9,852 |

All declared section sizes, block counts, and the final combifile length match
exactly for both files. All four relocation forms and DWORD PM byte packing are
now recovered from the shipping MIPS protocol loader, as described below.
Across build 117-926 the extractor resolves 4,265 type-0 and 40,802 type-2
fixups; types 1 and 3 are supported although these combifiles do not use them.

## Container references

The parser follows the structures published in the historical Linux Eicon
driver (`drivers/isdn/hardware/eicon/dsp_defs.h`) and its userspace loader
(`divactrl/load/common/dsp_file.c`):

- 48-byte combifile and nested-file magic fields;
- little-endian `t_dsp_combifile_header` and `t_dsp_file_header`;
- memory-block, segment, symbol and data-block tables;
- fixed segments 0-3 and relocatable segments beginning at 4;
- DM words and DWORD-container PM words;
- Eicon relocation byte masks.

This container is distinct from the simpler IDMA/BDMA boot-page format in
section 6.1 of `addspv90guide.pdf`, although both ultimately describe ADSP-218x
PM and DM loads.

## Standalone ADSP-2181 execution prototype

`tools/adsp2181emu/` now contains a standalone ADSP-2181 interpreter adapted
from MAME's BSD-licensed ADSP-21xx core. It has separate 16K-word PM/DM spaces,
the ADSP register banks, DAGs, ALU/MAC/shifter, loops, SPORT callbacks and the
full 24-bit instruction dispatcher. It intentionally has no Eicon peripheral
model yet.

Build it with:

```bash
make -C tools/adsp2181emu
```

A first real execution test uses the bootable primary-rate kernel:

```bash
./tools/eicon_dsp_extract.py docs/firmware/dspdload.bin \
  --match '^DIVA Server PRI 30M Kernel' -o /tmp/eicon-kernel

./tools/adsp2181emu/eicon_adsp_run \
  /tmp/eicon-kernel/0009-diva-server-pri-30m-kernel/pm.bin \
  /tmp/eicon-kernel/0009-diva-server-pri-30m-kernel/dm.bin 100000
```

The corrected interpreter run executes reset PM `0x0000` (`0x18580f`), jumps
to the kernel entry at `0x0580`, initializes the ADSP and reaches its IDLE at
PM `0x02a9` after about 100 instructions. The earlier apparent `0x0f1858`
SPORT path was caused by treating DWORD PM as an ordinary little-endian
integer; it was byte-rotated and is not a valid result.

Set `ADSP_TRACE` to trace the first N instructions:

```bash
ADSP_TRACE=64 ./tools/adsp2181emu/eicon_adsp_run <pm.bin> <dm.bin> 1000
```

`ADSP_RX0`/`ADSP_RX1` and `ADSP_TX0`/`ADSP_TX1` attach little-endian signed
16-bit SPORT streams. `ADSP_TRACE_SPORT=N` logs the first N transfers, and
`ADSP_START_PC` is available for entry-point experiments.

### Recovered MIPS relocation loader

The PRI protocol image `docs/firmware/te_dmlt.pm` is little-endian MIPS. Its
routine at file offsets `0x75e04..0x75f30` reads each DWORD PM container and
performs relocation. The input layout is:

```text
host word 0: instruction[23:8]
host word 1 low byte: instruction[7:0]
host word 1 high byte: relocation type | segment number
```

In pseudocode, after reconstructing the 24-bit instruction as `v` and looking
up the allocated segment base `b`, the four cases are:

```text
type 0: PM v += b << 8; DM word += b
type 1: v += b            # low PM data part
type 2: v += b << 4       # standard ADSP command
type 3: permute(v); v += b << 2; inverse_permute(v)
```

The exact type-3 masks are implemented in `tools/eicon_dsp_extract.py` from
MIPS offsets `0x75eb0..0x75efc`. As a semantic check, V.90 DPCM PM `0x1900`
resolves from its container form to instruction `0x872f71`; its direct DM
operand is `0x32f7`, inside referenced segment 4 (`0x32f0..0x32f7`).

The extracted kernel, `TIKRNL81.F34`, and V.90 DPCM images can be composed in
load order with `tools/eicon_adsp_bundle.py`. There are 117 differing overlaps:
108 words where TIKRNL replaces the kernel task-loader window and nine where
DPCM replaces TIKRNL state/code. Preloading all three before reset is not a
valid boot sequence: the TIKRNL replacement removes the boot kernel entry.
The boot kernel must first run to IDLE, then receive task descriptors through
the Eicon host interface and perform the staged loads itself.

The emulator now includes the ADSP-2181 interrupt controller, IDMA PM/DM
transfer protocol, PMOVLAY/DMOVLAY registers and two external 8K PM/DM banks.
The corrected register map removes the false reset-time writes to invalid
registers: group-1 registers 14 and 15 are PMOVLAY and DMOVLAY.

The resident PRI kernel enables only mask bit `0x020`, the SPORT0 receive
interrupt, and idles at `0x02a9`. Pulsing it vectors through PM `0x0014`; this
is the 8 kHz PRI sample/TDM path, not a task-download mailbox. With no line
interface model it emits G.711 idle code `0x00ff` and passes words along SPORT1,
which is consistent with the physical DSP chain on a 30-channel card.

A staged-load experiment now performs the real sequence more closely:

1. boot the PRI kernel to IDLE;
2. write TIKRNL81.F34 and V.90 DPCM populated PM/DM words without resetting;
3. CALL TIKRNL's zero-sized exported label at PM `0x0672` with a valid return
   stack entry;
4. return cleanly to kernel IDLE and clock SPORT0 receive interrupts.

The TIKRNL initializer clears/configures its PM window and populates its data
structures (including DM `0x3184..` and the task tables at `0x31c8..`). It does
not activate a modem channel by itself. Calling V.90 symbol 0 as code was ruled
out: it is a one-word fixed data symbol at DM `0x3602`, not an entry point.
The remaining boundary is the MIPS-side `dsp_assign` control transaction that
selects a channel and fills the TIKRNL task/database parameters. That protocol
runs in `te_dmlt.pm`; it is distinct from the sample SPORT and from the generic
IDMA modem interface in `addspv90guide.pdf`.

`--card-type` can now apply the combifile directory's usage masks. Card type 23
(the older DIVA Server PRI 30M profile) maps to file set 5 and selects the
DPCM-capable digital-side task family:

```bash
./tools/eicon_dsp_extract.py docs/firmware/dspdload.bin \
  --card-type 23 --list
```

## Next reverse-engineering step

Load `pm.bin` as little-endian packed 24-bit ADSP-218x program memory and use
`pm.words` to mark populated ranges. Load `dm.bin` separately as little-endian
16-bit data memory. Start with a build-to-build diff:

1. align the same overlay IDs (`0x0261`, `0x026a`, `0x026b`);
2. identify unchanged PM routines and DM tables;
3. locate tables shared by V.34 and both V.90 overlays;
4. search the V.90 DPCM/APCM differences for Phase 3 and DIL-specific control;
5. use the V.34 training sequences and known ITU constants as anchors before
   assigning function names.

A disassembler must explicitly support ADSP-218x 24-bit instructions. Ordinary
x86/ARM `objdump` cannot decode these images.
