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
The MIPS-side assignment boundary is now located. In `te_dmlt.pm` build
107-79, file-backed runtime objects use address bias `0x80011000`. Resolving the
trace-format references identifies generic `dsp_assign` at file offsets
`0x79cc4..0x7b978` and PRI `dsp30_assign` at `0x9775c..0x97dcc`.
`tools/eicon_dsp_assign.py` verifies these xrefs directly from the protocol
image.

The two routines have different roles. `dsp30_assign` pairs PRI transparent or
voice tasks such as download IDs `0x0065/0x0066` and `0x01f9/0x01fa`. Generic
`dsp_assign` selects download ID `0x0258`, `TIKRNL81.F34`, for a modem service;
it is therefore the relevant path even on the 30-DSP E1 card. The routine
allocates a per-DSP resource, copies the assignment parameter blob, and sets up
an asynchronous command state machine. It does **not** synchronously write a
flat modem database before returning.

For TIKRNL, the `0x0258` branch resolves symbol-table entries 13 and 14. In
build 117-926 these are fixed DM symbols:

| Symbol | DM address | Words | Role |
|---:|---:|---:|---|
| 13 | `0x3310` | 21 | MIPS-to-TIKRNL command/database mailbox |
| 14 | `0x3338` | 27 | TIKRNL-to-MIPS mailbox |

The mailbox transport is now recovered in more detail. The request parser is
at MIPS file offsets `0x78138..0x78388` and the sender is at
`0x786a4..0x78cc0`. The symbol-13 structure starts with control fields and owns
a 16-word circular command ring:

| Symbol-13 offset | Initial value | Meaning |
|---:|---:|---|
| `+0` | `0x0000` | command selector/doorbell written on commit |
| `+5` | `0x3327` | MIPS producer pointer |
| `+6` | `0x3327` | DSP consumer/shadow pointer |
| `+7` | `0x0010` | ring length in words |
| `+8` | `0x3327` | ring start |

For each commit the sender writes command words into DM
`0x3327..0x3336`, wrapping at 16 words, then writes the new producer pointer to
DM `0x3315`. It writes the request's selector to DM `0x3310` and the request's
control word, with bit `0x0020` cleared, to DM `0x3338`. These are the exact
post-initializer writes the emulator must reproduce.

The upper request is a byte record whose byte 2 selects one of five parser
forms. Form 0 selects a command script using bytes 3 and 4; form 1 queues a raw
byte payload; forms 2 and 3 update the `0x3338` control and `0x3310` selector
respectively; form 4 updates a separate state word. Script selection is
`script[mode][code]`, with two modes and codes below 75. The pointer table is at
MIPS file offset `0xeb248`. Script records consist of a word count, a
mask/opcode word and the remaining argument words; zero terminates and a
negative word branches within the script. `tools/eicon_dsp_assign.py` now dumps
that complete pointer map.

A second path carries the initial database itself. The helper at MIPS file
offsets `0x7bbf8..0x7bd0c` appends one database record to a host buffer. It
resolves the requested download symbol/location, emits that 16-bit DM address
little-endian, and appends arguments according to a compact format string:
`b` emits one byte and `w` emits one little-endian word. The strings adjacent
to `DSP_DRV.C` include formats such as `wbbww`, `bbbbb`, `bbbbww`, `bbb`, and
`bw`.

The commit helper at `0x7cd14..0x7cf18` writes the completed buffer into a
second ADSP circular ring. Its exact sequence is:

1. read and validate the ring producer against its start and length;
2. write a header containing the payload length and resource flags;
3. write the selected database/task identifier;
4. bulk-write the generated address/value records, splitting at ring wrap;
5. publish the updated producer pointer last.

The modem-specific switch-on branch is selected when the assigned task ID is
`0x0258`. It builds the TIKRNL/F34 setup block, appends additional records with
the format helper, and commits it through this ring. For example, one recovered
call selects database location `0x16` or `0x1c` and emits a `bbb` record with
three one-byte values. This is the database-write path originally being sought;
the symbol-13 command mailbox is the later runtime control path.

Thus `dsp_assign` itself allocates and loads the resource, switch-on commits the
initial database ring, and subsequent asynchronous requests use the symbol-13
command mailbox. Reproducing only one of those stages cannot activate a modem
channel. The emulator accepts `ADSP_POST_DM_WORDS` specifically for captured or
reconstructed post-initializer writes. SPORT0/SPORT1 remain only the PCM
highway.

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

## Script sender and command-ring semantics (2026-07-27)

Disassembly of the build 107-79 mailbox sender (te_dmlt.pm file
`0x786a4..0x78cc0`, capstone MIPS) recovered the exact command commit:

- The script interpreter walks 16-bit records: word 0 is the record length
  (including itself and the mask word), word 1 is a bit mask, and one argument
  word follows per set mask bit. A record with bit 15 set is a relative
  branch; a zero word terminates the script. Arguments whose mask bit is 3 or
  6 are shifted right by one unless a global flag is set. A NULL script
  pointer selects the shared empty script at file `0xeaf2c`.
- Record words are written to a 16-word ring in **PM data space** at PM
  `0x3327..0x3336`: the sender forms host-port address `ring_pos + 0x4000`,
  and the host-port helper (runtime `0x80082950`) treats addresses with bit
  `0x4000` set as 24-bit PM writes (data word then a zero pad byte), lower
  addresses as single 16-bit DM writes.
- The commit then writes the producer pointer to DM `0x3315`, the command
  selector to DM `0x3310`, and the control word with bit `0x20` cleared to DM
  `0x3338`.

The emulator's IDMA model previously had the bit-`0x4000` PM/DM select
inverted; this is fixed and covered by `adsp2181_core_test`.

Dynamic probing with the staged TIKRNL image (watchpoints +
`ADSP_TRACE_HOST`) shows:

- The resident kernel's SPORT0 RX ISR at PM `0x0072` is a per-timeslot TDM
  state machine: DM `0x2e44/0x2e45` hold channel-table pointers (`0x2e00`),
  DM `0x2e50` is a per-channel substate countdown, DM `0x2e52` holds the
  current PCM code (`0x00ff` idle). A staged but unassigned TIKRNL never runs:
  no IRQ (0/1/2/6) doorbells the command mailbox, and the DSP never reads DM
  `0x3310`.
- Channel activation therefore requires the `dsp_assign` initial database
  writes (the database-ring commit path at MIPS file `0x7cd14..0x7cf18`),
  which must hook a channel-table entry to the modem task before command
  scripts mean anything.

## Next reverse-engineering step

Reconstruct the complete `dsp_assign` write sequence for a modem (task
`0x0258`) assignment: disassemble `dsp_assign` (file `0x79cc4..0x7b978`), the
database record builder (`0x7bbf8..0x7bd0c`, format chars `b`/`w`) and the
ring commit (`0x7cd14..0x7cf18`), and emit the resulting DM/PM word map for
the emulator's `ADSP_POST_DM_WORDS`/`ADSP_HOST_SCRIPT` replay. Success
criterion: the SPORT0 ISR's channel-table walk reaches TIKRNL channel state
(the `0x2e00` table entry for the assigned timeslot points into TIKRNL
structures) and DM `0x3310` command selectors start being consumed.

## Database ring commit and kernel task dispatch (2026-07-27, session 2)

The database ring commit helper (MIPS file `0x7cd14..0x7cf18`) is fully
decoded. Its ring descriptor (MIPS-side struct) has: `+0x0c` producer-pointer
DM address, `+0x10` PM flag (clear: payload words are written to PM via the
`+0x4000` host-address convention), `+0x12` ring start, `+0x14` ring length.
Each commit writes: header word `(payload_bytes + 2) | resource_flags`, then
the database/task identifier word, then the byte payload (packed two bytes
per word, wrap-split), and publishes the producer last. The record builder
(file `0x7bbf8..0x7bd0c`) emits `[DM addr lo, DM addr hi, value bytes...]`;
its address operand resolves either a download symbol (8-byte table entries,
address at `+4`) or a database location id, and its format string emits one
byte (`b`) or one little-endian word (`w`) per character from a varargs list.

The `0x0258` tail of `dsp_assign` (runtime `0x8008c6e8..0x8008c978`) only
initializes the per-channel mailbox/request struct: `+0x140/0x144` = DSP host
register block, `+0x148` = TIKRNL symbol 13 (`0x3310`), `+0x14a` = symbol 14
(`0x3338`), plus timeout defaults (`0x80`) and zeroed state. All actual DSP
writes happen later through the asynchronous request/script machinery.

On the DSP side, watchpoint+trace analysis of the resident kernel shows the
foreground idle loop at PM `0x02a8..0x02ac` reads the channel-table pointers
DM `0x2e44/0x2e45` (-> `0x2e00`) and there is an indirect `CALL (I4)` at PM
`0x02a4` — the kernel's task dispatcher (further indirect calls through I4 at
PM `0x01d9/0x01ed/0x0266`, indirect jumps at `0x02fe/0x0522..`). DM
`0x2e00..0x2e3f` is the 64-entry G.711 timeslot buffer (`0x00ff` idle);
channel descriptors start at `0x2e40`; a second pointer/link table lives at
`0x2f00..0x2f2b`. DM `0x3fe0..0x3fff` is the ADSP-2181 system-register page
(`0x3ff9 = 0x8000` and `0x3ffa = 0` are read by the ISR every frame but are
not a host command doorbell — poking them changes nothing).

A standalone ADSP-218x disassembler now exists (MAME `2100dasm.cpp`,
BSD-3-Clause, fetched from the mamedev mirror) and is wired up as
`/tmp/dasm`; it decodes ALU/MAC/control flow correctly but mislabels the
direct DM read/write opcodes (`10dd ddaa ..` form, address in bits 4..17),
so watchpoints remain the ground truth for those.

Caller-scan of the protocol image shows the modem database setup is the dense
`db_record_append`/`db_ring_commit` cluster at file `0x8892c..0x8a50c`
(roughly a dozen append+commit pairs), with `request_parser` called from file
`0x92e50` and `0x9f05c`. Next step: disassemble that cluster to recover the
exact switch-on database contents (ring target, record list, values) for a
modem answer assignment, then replay through `ADSP_HOST_SCRIPT` and verify
the kernel's `CALL (I4)` dispatcher reaches TIKRNL channel code.

## Host doorbell and kernel command queue (2026-07-27, session 3)

The host doorbell is **IRQE** (enum 6, priority 5, imask bit `0x0100`,
vector `0x18`). The vector contains a bare RTI: its only purpose is to wake
the kernel foreground from IDLE. After an IRQE wake the foreground leaves its
`0x02a8..0x02ac` idle loop and runs the queue processor at `0x02ad..`:

1. rebuilds the free-list links at DM `0x2f27..0x2f2b` (-> `0x2f21`, `0x2f00`,
   `0x2f0e`, `0x2f42`, `0x2f4e` — five message entries);
2. reads the queue head/tail DM `0x2f08/0x2f09` (equal = empty);
3. calls the service dispatcher (`0x01c1` -> `0x02a1` -> `0x01b2` ->
   `0x00d8`) which walks the per-frame descriptor at DM `0x2f00`:
   field `+0` flags/command, `+1` = `0x2800`, `+4` = DM data pointer
   (dereferenced), `+0x0c/+0x0d` state; DM `0x2e78` is cleared when a queued
   entry was present.

Harness notes: level-sensitive IRQs must be held asserted across two
`adsp2181_run` slices because `check_irqs` runs at run-entry and the SPORT0
ISR masks IRQ1/IRQE (priority 5/7) while active. The kernel restores imask
from the status stack on RTI, so `ADSP_FORCE_IMASK` is re-applied every host
word. IRQ2's vector (`0x0004`) is a parked IDLE — it is not the doorbell.

Open: the exact queue-entry semantics for task download / channel assign.
Queue pushes with head!=tail are consumed silently (pointers normalized back
to `0x2f00`), and the per-frame descriptor at `0x2f00` is processed every
8 kHz frame regardless. The next step is decoding the queue processor at PM
`0x02ad..0x02c0` and the service routine at `0x00d8..0x0109` statically, then
replaying a task-download queue entry for TIKRNL so the boot kernel performs
the staged load itself (the real boot sequence), instead of pre-staging.

## MIPS shim: real firmware routines drive the emulator (2026-07-27, session 4)

`tools/eicon_mips_shim.py` runs the actual te_dmlt.pm routines under Unicorn
(physical kseg0 mappings — this unicorn build has unreliable guest data
accesses for pages first written after execution starts, and kuseg pages) and
connects their host-port calls to the ADSP-2181 emulator via ctypes
(`libadsp2181.dylib`). host_write (`0x80082950`) and host_read (`0x80082920`)
are hooked; `adsp2181_host_write/read` implement the exact IDMA semantics.

The command-script sender (`0x800896a4`) takes `a0` = request struct:
`+0/+4` host-reg pointers, `+8` symbol-13 address (`0x3310`), `+0xa`
symbol-14 (`0x3338`), `+0xc` active flag, `+0x10` script code (<75),
`+0x11` script mode (<2), `+0x12` command selector, `+0x14` script pc,
`+0x1c` request form (0=script, 1=single word, >=2=raw byte payload at
`+0x1e`), `+0x3e` control word. Script table index is `mode*75 + code`
(earlier "79" was wrong) into the pointer table at `0x800FC248`.

Verified end-to-end: with code 66 (mode 0) the sender writes the ring records
`a001 0708 | a00d 0a28 4333 0286 | e007 004b` to PM `0x3327..`, advances the
producer to `0x3331`, writes selector `0x0001` to DM `0x3310`, and clears the
control word — matching the statically recovered script (including the
mask-bit-3 `>>1` argument rule: `0x050c -> 0x0286`).

The DSP does not consume the command yet (consumer stays `0x3327`): channel
activation/doorbell on the DSP side is still required (kernel queue vector +
IRQE, or TIKRNL's own consumer hooked into the frame loop).

Also: the kernel queue handler at `0x01c1` performs `CALL (I4)` on queued
entries — queue entries carry function vectors. The per-frame descriptor at
DM `0x2f00` (fields: `+0` flags, `+1 = 0x2800`, `+4` DM data pointer) is
serviced every 8 kHz frame by the routine at `0x00d8`.
