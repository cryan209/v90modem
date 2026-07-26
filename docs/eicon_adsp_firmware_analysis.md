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
  -o artifacts/eicon-dsp/build-117-926
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
exactly for both files. The newer V.34 and DPCM overlays contain three Eicon
type-2 relocations each. The extractor resolves their ADSP standard-command
14-bit direct addresses against relocatable segment 4. The selected APCM and
older overlays need no non-zero relocation fixups.

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
