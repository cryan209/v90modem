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

`--match` alone is ambiguous where a download id ships in several variants —
`0x0270` is `SIGLK`, `SIG` and `SIG.ANA`; `0x0262` is plain, `.F34` and `.ANA`.
`--card-type` resolves it the way the driver does, by the usage mask. Card type
56 is file set 12, the set the PRI 30M kernel, TIKRNL81.F34 and
DIAL/FSK/FAX.F34 all belong to, so it selects exactly the `.F34` variants:

```bash
./tools/eicon_dsp_extract.py docs/firmware/dspdload.bin \
  --card-type 56 --match Overlay -o artifacts/eicon-dsp/overlays
```

That is the overlay set `tools/dial_tikrnl_drive.py` serves page switches from.

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

## Session 5: parser path live, kernel scheduler model complete (2026-07-27)

- The shim now drives the full top-level path: byte request
  `[len, ?, form, code, mode]` -> request_parser (`0x80089138`) ->
  script_sender, reproducing the script-66 PM-ring commit through the real
  firmware code path (not just the sender in isolation).
- Kernel scheduler model: five service slots whose entry pointers live at
  DM `0x2f27..0x2f2b` (entries at `0x2f21/0x2f00/0x2f0e/0x2f42/0x2f4e`).
  IRQ/service handlers each own a slot (SPORT0 TX -> `2f27`, IRQ2 -> `2f28`,
  IRQL1/2 -> `2f29`, timer -> `2f2a`, `2f2b` spare) and CALL the slot's
  vector. The foreground per-frame service uses slot `2f28` (entry `0x2f00`,
  vector field `+1`, currently the inactive `0x2800`).
- TIKRNL's vector table (DM `0x31bb`) entries are wrappers that CALL the
  kernel's `0x01b2/0x00d8` service routine and then dispatch via SR0 — the
  kernel and task kernels share the scheduler.
- TIKRNL init (entry `0x672`) calls `0x0637/0x184d/0x064a`; `0x184d` exports
  service vectors `0x05ab/0x05b1` (or `0x05b7/0x05be`) into DM `0x3307+`.
- Service-driver table in te_dmlt.pm at file `0xeaec4`: {release `0x8008c978`,
  `0x80096980`, `0x80098310`, `0x80098614`, `0x80099734`, `0x800a6820`,
  `0x8009fae8`, `0x800a6874`, `0x800a687c`, `0x800a68c0`, `0x800a318c`} —
  the modem service entry points (assign is reached via this table, not by
  direct `jal`).
- Harness: `ADSP_STAGE_ENTRY2_AT` (call after word N), `ADSP_TRACE_AT_WORD`,
  hex-safe entry parsing. `tools/adsp2181_dis.py` decodes the full
  ADSP-2181 kernel+TIKRNL images reliably.

Next: run the modem service assign entry (table slot 1, `0x80096980`) in the
shim with a synthesized TIKRNL download struct (segments/symbols from
metadata.json), which performs the switch-on database commit; then feed the
E1 timeslot stream on SPORT0 (a call can also be signalled from the E1 side:
the kernel ISR walks timeslots and CAS/signalling arrives in-band).

## Session 6: service-assign entry runs live in the shim (2026-07-27)

`tools/eicon_mips_shim.py` gained an `--assign` mode that calls the real
service-assign entry `0x80096980` (file `0x85980`, service-driver table slot
1) under Unicorn with a synthesized TIKRNL download/task struct, so the
firmware's own code performs the switch-on database commit through the
hooked host port — not a hand-replayed write sequence.

Reverse-engineering required to make the routine run:

- **Correct `$gp`.** The image entry (file `0x4774`/`0x4764c`) sets
  `gp = 0x8010.0000 - 0x5c4b = 0x800fa3b5`. The shim's previous hardcoded
  `0x80108000` was wrong; it left the trace-printf pointer at `gp+0x1a7b`
  (`0x800fbe30`) NULL, so the first `jalr $v0` in the assign trace path
  faulted to address 0. The real pointer is file-backed and equals
  `0x80083180`.
- **Trace-printf redirection.** The real printf (`0x80083180`) writes to the
  hardware trace buffer at `0xa0005d20` (uncached kseg1). The shim overwrites
  the pointer at `0x800fbe30` with the no-op stub address so trace calls
  return immediately.
- **Three MIPS memory segments** the shim now maps: the code image
  (`0x11000`–`0x111000`, file-backed), the `.data`/`.bss` segment
  (`0x80200000`, physical `0x200000`, zero — the lookup tables at
  `0x80272c90` etc. are *not* in `te_dmlt.pm`, which ends at `0x100230`),
  and the runtime stack/heap segment (`0x80300000`, physical `0x300000`, zero
  — `sp = 0x80338700` and the database-record buffers at `0x80331c12` live
  here). An auto-map hook covers neighbouring pages and a low kuseg guard
  page so NULL-ish dereferences surface as zero instead of stopping the run.
- **Synthesized struct.** `0x80096980` takes `a0` = an assign request whose
  `+0` -> base (s2), `+4` -> resource (s0), `+8` -> existing mailbox (0 for
  fresh), `+0x18` = channel byte. `s0+4` -> a download descriptor with id
  `0x0258` at `+0`; `s0+0x40` = task id halfword. `s2+0xc` -> a channel
  context whose `+0x24` -> a descriptor; `s2+0x10` -> the host register
  block (data port `+0`, address port `+0x80`). The per-channel state
  `s1 = s2+0x200` owns the command mailbox (`+0x24`, active flag `+0x10`)
  and the database-ring descriptor (`+0x0c` producer DM offset, `+0x10` PM
  flag, `+0x12` ring base, `+0x14` ring length).

The dispatch for task `0x0258` (none of `0x213/0x1f5/0x1ff/0x227/0x2bd` match)
runs `0x80093d14` then `0x80090e58`. `0x80093d14` is a synchronous
command/handshake: it calls `0x80086af8` (DSP wait) and, on success,
`0x80093ba4` -> `0x8008cacc` (send a command word via `host_write`). For a
fresh assign the mailbox active flag makes `0x80086af8` return nonzero so
`0x80090e58` (the db record-append + ring-commit body) runs. `0x80090e58`
calls `db_record_append` (`0x8008cbf8`, 7x) and finishes at `0x80093b50`
with `db_ring_commit` (`0x8008dd14`).

Verified: `--assign` produces host_write transactions through the real
firmware path — the ring header to the PM ring and the producer-pointer
publish to DM — i.e. the switch-on database commit is live. The DSP does
not yet consume the command, because the synthesized database ring targets
DM `0x0001` rather than the real TIKRNL symbol-13 ring at `0x3327`; the
remaining work is feeding the correct ring descriptor (real TIKRNL
database-ring DM address and segment/symbol relocation table from
`metadata.json`) so the kernel's channel-table walk hooks the modem task.

## Session 7: Linux driver source + PR_RAM request queue (2026-07-27)

The `divas4linux` driver source (in `/tmp/divas4linux-master`) provides the
complete host-side architecture, confirming the reverse-engineered model:

- **`kernel/pr_pc.h`**: `struct pr_ram` — the shared-RAM request queue.
  `NextReq`/`NextRc`/`NextInd` are word offsets into `B[]` (the buffer area
  at +0x20).  `ReqInput`/`ReqOutput` are byte counters.  `REQ`/`RC`/`IND`
  structures form linked lists via their `next` field.
- **`kernel/di.c` `pr_out()`**: the host writes a `REQ` at `B[NextReq]`,
  advances `NextReq = REQ->next`, increments `ReqInput`.  The MIPS reads
  from `B[read_offset]` (gp+0x5e99), advances via `REQ->next`, increments
  `ReqOutput`.
- **`kernel/mi_pc.h`**: shared RAM at physical `0x1000`, protocol at
  `0x11000`, boot structure (`struct mp_load`) at `0x0`.
- **`kernel/mdm_msg.h`**: complete modem CAI byte layout (hardware type
  `0x11` = modem async, V.8 negotiation, modulation masks, speeds).
- **`kernel/message.c` `add_modem_b23()`**: CAPI→IDI modem call path.
- **`kernel/s_pri.c`**: PRI card init, DSP detect (`dsp_addr_port` at
  `+0x80`, `dsp_data_port` at `+0x00` — confirms the IDMA hook).

The shim's `--mainloop` mode now:
1. Maps shared RAM (physical `0x0`–`0x11000`).
2. Fixes the auto-map hook for kseg0/kseg1 (translates `0x8xxx`/`0xaxxx` to
   physical via `& 0x1fffffff`).
3. Calls the real firmware entry (`0x80082f90`) which stores the PR_RAM
   pointer and runs basic init.
4. Calls the post-wait init functions (`0x80083d10`, `0x8002a534`).
5. Hooks the DSP register region (physical `0x380000`+, computed from
   `DSPInfo=0x80`) with `_dsp_read`/`_dsp_write` routing to the ADSP IDMA
   interface.
6. Writes a modem `ASSIGN` request to the PR_RAM queue and runs the main
   loop (`0x80027970`).

**Result**: the MIPS main loop runs and reads the ASSIGN request from PR_RAM.
The firmware entry produces IDMA writes to the DSP (PM code download at
`0x3e8+`).  However, the init's DSP presence check (`lhu $s2, ($s1)` at
`0x80380000`) returns 0 because the ADSP's DM[0] is 0, so the firmware skips
DSP resource registration (gp+0x5eb9 stays 0).  Without registered DSPs, the
ASSIGN can't allocate a channel and produces no host_writes.

**Remaining work** (well-defined, not exploratory):
1. Model the DSP presence check: the firmware writes `0xFF` to DM[0x3f] via
   the addr/data ports and reads it back.  The `_dsp_read`/`_dsp_write` hooks
   must correctly route this through `idma_addr_write`/`idma_data_write`/
   `idma_data_read` so the write-back-read returns `0xFF`.
2. Load the combifile (`dspdload.bin`) into shared RAM at `DspCodeBaseAddr`
   (computed from the protocol image's end address) so the firmware can
   download DSP code from it.
3. Once the init detects DSPs and registers them (gp+0x5eb9 != 0), the
   main loop will process the ASSIGN request, calling `dsp_assign` and
   downloading the V.90 overlay internally.
## Session 8: linked call assignment and bearer activation

The network-layer `0xe6` rejection was a missing call-parent link, not a bad
modem LLC/DLC. In the Linux driver's `message.c`, the first
`nl_req_ncci(..., ASSIGN, 0)` is sent with global `NL_ID`; `send_req()`
prepends `CAI, 1, plci->Sig.Id` to the parameters. The shim previously sent
only LLI/LLC/DLC, leaving the firmware no signalling entity (PLCI) to attach
the network entity to.

`modem_nl_assign_payload()` now accepts the assigned signalling ID and emits
that CAI prefix. The native PR_RAM sequence consequently succeeds:

```
[sig] RC 0xef (ASSIGN_OK) Id=0x02 Ref=0x0000
[nl]  RC 0xef (ASSIGN_OK) Id=0x03 Ref=0x0001
```

The shim also writes the REQ `Reference` field explicitly, can submit the
network-layer `N_CONNECT`, and drains the PR_RAM indication chain. With
`--connect`, firmware accepts bearer activation and produces:

```
[call] RC  0xff (OK) Id=0x03 Ch=0x02 Ref=0x0001
[call] IND 0x03      Id=0x03 Ch=0x02   # N_CONNECT_ACK
[call] IND 0x04      Id=0x03 Ch=0x02   # N_DISC
```

The initial experiment disconnected because it activated NL without first
answering the parent signalling call. `--simulate-b-channel` now models the
answered incoming sequence: linked SIG+NL assignment, `CALL_RES` on SIG, then
NL activation. Both entities return `IND 0x03`, and no `N_DISC` appears after
512 main-loop iterations; the harness reports the simulated B-channel
`ACTIVE`.

RING and CID therefore belong to signalling before modem activation, as
expected. The next boundary is DSP resource startup: the held B-channel
currently produces no post-boot IDMA writes, so the switch-on database has
not yet initialized TIKRNL/DIAL before NORM/V.8.

Tracing the two modem-service entry points makes the missing state precise:
neither service assign `0x80096980` nor switch-on `0x80090e58` executes.
The `ASSIGN DSIG_ID` result (`Id=0x02`) is the global/listener signalling
entity. A real incoming call first produces `CALL_IND` and allocates a
per-call PLCI; `connect_res()` attaches `add_b1()`'s modem CAI to `CALL_RES`
on that PLCI. Sending `CALL_RES` to the listener can return `OK` and keep NL
from immediately disconnecting, but it does not allocate a modem DSP.

The simulator now reports this honestly as
`SIGNALLING ACTIVE, DSP UNASSIGNED`. The next implementation step is to inject
the network-side incoming-call event through the signalling handler (creating
the per-call entity), then issue linked NL ASSIGN and CAI-bearing CALL_RES
against that new entity.

## Session 9: native signalling trace and direct service-assign proof

`tools/eicon_mips_shim.py` now has `--trace-calls`, which records MIPS
`jal`/`jalr` targets by harness phase. The trace normalizes Unicorn's physical
PCs back to the protocol image's `0x800...` runtime addresses, so the output
can be compared directly with disassembly and earlier recovered entry points.

The native PR_RAM path is reproducible with:

```bash
.venv/bin/python tools/eicon_mips_shim.py \
  --kernel artifacts/eicon-dsp/build-117-926/kernel/0009-diva-server-pri-30m-kernel \
  --tikrnl artifacts/eicon-dsp/build-117-926/tikrnl/0258-tikrnl81.f34-task \
  --mainloop --simulate-b-channel --call-steps 2 \
  --trace-calls --trace-call-limit 120
```

Result: DSP resource registration is healthy (`gp+0x5eb9=0x0060`, 30 DSPs
answer the `0xa5a5` boot handshake), SIG and NL assignment both return
`ASSIGN_OK`, and `CALL_RES`/`N_CONNECT` both return/indicate success, but the
modem DSP path remains unentered:

```text
[call] simulated B-channel: SIGNALLING ACTIVE, DSP UNASSIGNED
[mainloop] modem DSP path: service_assign=0 switch_on=0
```

The phase trace pins down the boundary:

| Phase | Distinctive firmware calls | Meaning |
|---|---|---|
| `sig-assign` | `0x800c99e4` x4 | SIG ASSIGN copies/normalizes the listen/register parameter block. |
| `call-res` | `0x800c9470` x3 | CALL_RES runs signalling IE parsing/serialization, not modem service assignment. |
| all native phases | no `0x80096980`, no `0x80090e58` | The listener entity never becomes a per-call PLCI in the synthetic sequence. |

Disassembly around `0x800c9470` shows the IE walker/copy helpers and calls
into `0x800c99e4`; it is useful for reconstructing signalling payload format,
but it is downstream of the missing network-originated incoming-call event.
The viable clean route is therefore still to inject the incoming SETUP/CALL_IND
event before `CALL_RES`, so the firmware allocates a per-call PLCI instead of
answering the listener entity.

The direct allocator route is also live. This command:

```bash
.venv/bin/python tools/eicon_mips_shim.py \
  --kernel artifacts/eicon-dsp/build-117-926/kernel/0009-diva-server-pri-30m-kernel \
  --tikrnl artifacts/eicon-dsp/build-117-926/tikrnl/0258-tikrnl81.f34-task \
  --assign --words 40
```

calls the real service-driver entry `0x80096980` and produces the switch-on
database record through firmware host writes:

```text
[assign] returned v0=0x80804100 host_writes=17
[assign] TIKRNL command ring DM3327..3336:
001d 0000 0000 0000 00ff 0002 0000 0000
0102 0008 0000 0200 0008 0000 1e00 0000
[assign] host writes:
7327=001d ... 7315=3337
```

So the current hard fact is: **DSP switch-on works when invoked directly;
native CALL_RES is missing the per-call PLCI creation event.** The next code
step is to use the `0x800c94xx`/`0x800c99xx` signalling helpers as format
oracles while locating the upstream network ingress that emits `CALL_IND`
(`0x02`) to PR_RAM.

## Session 10: fake ingress state in the MIPS shim

`tools/eicon_mips_shim.py --simulate-b-channel` now explicitly arms incoming
signalling before answer:

1. SIG ASSIGN returns `Id=0x02`.
2. NL ASSIGN returns `Id=0x03`.
3. `INDICATE_REQ`/listen is posted to the assigned SIG id and returns `OK`.
4. A synthetic incoming-call object is linked into the listening SIG entity.

The entity table used by the firmware dispatcher is at `0x80299928`, with the
entity count in `gp+0x5eb9`. After SIG/NL assignment, the active listener is
slot 0:

```text
[entities] 00: ptr=0x801004e0 ... +14=00000002 +18=800164b8
```

With fake ingress enabled, the shim links a synthetic call object at
`SIG+0x1c` before issuing `CALL_RES`:

```text
[listen] RC 0xff (OK) Id=0x02 Ch=0x00 Ref=0x0000
[ingress] synthetic call object 0x80807000 linked to entity slot 0 obj=0x801004e0
[entities] 00: ptr=0x801004e0 ... +1c=80807000 +24=00000001
```

This proves the harness can fake the firmware-owned ingress state rather than
only pushing host PR_RAM requests. It still does not enter modem service
assignment:

```text
[mainloop] modem DSP path: service_assign=0 switch_on=0
```

So the remaining gap is no longer "how do we fake an ingress at all"; it is
which additional per-call fields the CALL_RES/resource-selection path expects
besides the minimal linkage written by the recovered `0x800172a8` allocation
branch. The branch writes `call+0x2f=1`, `call+0x28=sig`,
`sig+0x24=1`, `sig+0x12a=1`, `sig+0x1c=call`, and clears bit `0x10000` in
`sig+0x20`; later service selection likely depends on the call object's parsed
BC/LLC/CIP fields.

## Session 11: ingress field seeding

The fake ingress path now seeds the fields the recovered incoming setup parser
uses before answer:

| Offset | Seed | Meaning inferred from parser |
|---|---:|---|
| `sig+0x24` | `2` | pending incoming-call state after initial allocation |
| `sig+0x365` | `04 90 90 a3 00` | BC: 3.1 kHz audio / 64 kbit/s / A-law |
| `sig+0x37d` | `04 88 90 21 00` | LLC-style low-layer information |
| `sig+0x395` | `01 80` | channel/additional-info placeholder |
| `sig+0x51f` | `ff` | previous/invalid channel marker |
| `sig+0x520` | `11` | selected modem async resource byte |

`CALL_RES` now also uses the old IDI modem answer payload instead of the
26-byte SIG ASSIGN CAI:

```text
CAI len=6: 11 09 00 00 20 00
```

The run confirms these fields are present in the firmware object before
`CALL_RES`, but the path still stops before `service_assign`:

```text
[entities] 00 ... +1c=80807000 +24=00000002
[entities] 00: sig+340..52f=...049090a3...04889021...ff11...
[mainloop] modem DSP path: service_assign=0 switch_on=0
```

This means the blocker has moved again: the firmware is no longer missing
only obvious parsed BC/LLC/channel fields. The remaining condition is likely
ownership/allocator metadata on the per-call object that the real
`0x800785c4` allocation path creates and the synthetic `0x80807000` object
does not yet reproduce.

## Session 12: PRI/E1 signalling DSP lead

`docs/ADSP-21MOD870.PDF` and `docs/addspv90guide.pdf` are a useful correction
to the call-ingress model. The ADSP-21mod870 reference design is not just a
host plus isolated modem datapumps: its network-access diagram has
line-interface/call-control blocks for `T1,E1,PRI,xDSL,ATM`, and its modem
software guide says that T1/E1 operation programs SPORT0 in multichannel
mode, with DB setup locations for the SPORT0 control registers and `V34SLOT`
selecting the TDM slot used by modem operation.

The actual build-117-926 combifile for card type 23 matches that architecture.
The staged image contains separate PRI line/signalling downloads before the
modem task:

| ID | Download |
|---:|---|
| `0x0007` | DIVA Server PRI 2M TX Kernel |
| `0x0008` | DIVA Server PRI 2M RX Kernel |
| `0x000b` | DIVA Server PRI 2M TX SIG Kernel |
| `0x000c` | DIVA Server PRI 2M RX SIG Kernel |
| `0x0208` | SIG.MDM Task |
| `0x0209` | SIGPRTX Task |
| `0x020a` | SIGPRRX Task |
| `0x0258` | TIKRNL81.F34 Task |
| `0x0270` | SIG Overlay |
| `0x025f` | V8.F34 Overlay |
| `0x026a` | V.90 DPCM Overlay |

`tools/eicon_mips_shim.py` now prints those IDs in its DSP staging summary so
each run shows whether the line/SIG layer is present. A fresh
`--simulate-b-channel` run still answers the synthetic PLCI through SIG/NL but
never reaches DSP assignment:

```text
[mainloop] DSP code staged ... (64 downloads, card type 23 -> file set 5)
           id=0x000b ... DIVA Server PRI 2M TX SIG Kernel ...
           id=0x000c ... DIVA Server PRI 2M RX SIG Kernel ...
           id=0x0208 ... SIG.MDM Task ...
           id=0x0209 ... SIGPRTX Task ...
           id=0x020a ... SIGPRRX Task ...
           id=0x0258 ... TIKRNL81.F34 Task ...
           id=0x0270 ... SIG Overlay ...
[call] simulated B-channel: SIGNALLING ACTIVE, DSP UNASSIGNED
[mainloop] modem DSP path: service_assign=0 switch_on=0
```

That result changes the most likely next route. The fake MIPS object proves
we can satisfy visible PR_RAM request/response state, but it does not reproduce
the internal call-control ownership chain. The better target is now the
PRI/SIG DSP ingress side: either instantiate the SIG.MDM/SIGPRTX/SIGPRRX path
far enough that it emits the normal incoming-call indication into the MIPS
PLCI allocator, or recover exactly what metadata that path passes to
`0x800172a8`/`0x800785c4` and synthesize that object rather than the current
minimal shell.

## Session 13: SIG task registration recovered

`tools/eicon_sig_path_probe.py` now extracts the ADSP-side registration points
for the PRI/SIG path. The probe loads one kernel plus one SIG task, runs the
task's download entry, and diffs PM after the task calls the kernel's service
registration routine.

The task entries and registration results are:

| Task | Kernel | Entry | Registered patch |
|---|---|---:|---|
| `0x0208` SIG.MDM | PRI 30M kernel `0x0009` | `PM 0x0980` | `PM 0x02b9: CALL 0x02a1 -> CALL 0x0999` |
| `0x0209` SIGPRTX | PRI 2M TX SIG kernel `0x000b` | `PM 0x3900` | `PM 0x0032 -> CALL 0x3914` |
| `0x020a` SIGPRRX | PRI 2M RX SIG kernel `0x000c` | `PM 0x3900` | `PM 0x0032 -> CALL 0x390d` |

Probe output for `SIG.MDM`:

```text
[probe] task=0208-sig.mdm-task entry=0x0980
[probe] patch slots before: PM02b9=1c2a1f PM00b5=2a7eea
[probe] patch slots after:  PM02b9=1c999f PM00b5=2a7eea
[probe] PM changes: 1
  PM02b9: 1c2a1f -> 1c999f
```

Probe output for the PRI 2M SIG tasks:

```text
[probe] task=0209-sigprtx-task entry=0x3900
[probe] PM changes: 1
  PM0032: 0d0c7e -> 1f914f

[probe] task=020a-sigprrx-task entry=0x3900
[probe] PM changes: 1
  PM0032: 0d0c7e -> 1f90df
```

`SIG.MDM` is therefore not a vague architectural hunch any more: it installs a
foreground callback at `PM 0x0999`. That callback processes the task's private
state and eventually reaches the DSP-to-host doorbell helper at `PM 0x13a2`.
The helper saves temporary registers at `DM 0x05e2..0x05e4` and calls kernel
service `PM 0x000a` at `PM 0x13d2`, the same DSP-to-host doorbell path used by
TIKRNL. Its queue/format state is centred on:

```text
DM 05de = 0000
DM 05df = 00ab
DM 05e0 = 05f5
DM 05e1 = 0601
DM 05e5..0612 = nibble/order and format tables
DM 0660..06ef = SIG.MDM runtime state block
```

The immediate next target is to drive `SIG.MDM`'s `0x0999` foreground callback
with a populated `DM 0x05de..0x05e4` queue until it toggles the DSP-to-host
service bit at `DM 0x2f17`. Once that event shape is recovered, the MIPS shim
can either deliver the real DSP-side indication into PR_RAM or synthesize the
corresponding allocator metadata at the MIPS call-ingress boundary.

## Session 14: forced modem DSP assignment during fake call

There is now a deliberately simpler path in `tools/eicon_mips_shim.py`:
`--force-modem-dsp-assign`. `--simulate-b-channel` enables it by default.
After `CALL_RES` and `N_CONNECT`, the shim stages a direct PRI-kernel+TIKRNL
core, runs the recovered MIPS modem `SERVICE_ASSIGN` entry (`0x80096980`), and
pumps the TIKRNL command path long enough to observe the real switch-on
database commit.

This bypasses native PRI/SIG call ingress selection; it is a practical shim
affordance for "the bearer is connected, tell the modem DSP to handle it."

Successful run:

```text
[force] staging direct TIKRNL core for modem DSP assignment
[assign] calling 0x80096980 ... ch=1 mb13=0x7310 mb14=0x7338
[assign] returned v0=0x80804100 host_writes=17
[assign] TIKRNL command ring DM3327..3336:
001d 0000 0000 0000 00ff 0002 0000 0000
0102 0008 0000 0200 0008 0000 1e00 0000
[call] simulated B-channel: ACTIVE (modem DSP assigned)
[mainloop] modem DSP path: service_assign=1 switch_on=1
```

So the architectural problem is split cleanly:

1. The harness can now force genuine modem service assignment at the connected
   call boundary.
2. The still-open faithful path is to replace the forced direct TIKRNL core
   with either native PRI/SIG ingress metadata or a real firmware-selected DSP
   resource, then route the bearer PCM into that assigned task.

## Session 15: raw G.711 RX probe into forced TIKRNL

`tools/eicon_mips_shim.py` also has a first RX-side G.711 probe:
`--g711-probe-samples N --g711-probe-code BYTE`. After forced assignment it
writes the raw octet into TIKRNL's line words (`DM 0x3f08`/`0x3f09`) and runs
the task frame entry. If TIKRNL requests an overlay, the probe loads the
extracted image by download ID, sets `BOOTFINISHED`, and resumes the task's
completion entry.

This proves the assigned core can hear raw G.711 codewords and advance through
the task/page machinery:

```text
[g711] served requested overlay 0x0270 from 0270-sig-overlay
[g711] sample 0000: ... 31A9=0001 31AA=0262
[g711] served requested overlay 0x0262 from 0262-dial-fsk-fax.f34-overlay
[g711] sample 0001: ... 3FB0=000b 3FB2=17bb 3FB3=1706 ... 31AA=0263
[g711] served requested overlay 0x0271 from 0271-v.22fc-overlay
[g711] sample 0002: ... 3FB0=0001 3FB2=1582 3FB3=15dd ... 31AA=0266
[g711] served requested overlay 0x0266 from 0266-v.22-v.32-lec-overlay
[g711] fed 16 raw G.711 octets 0xff; line-state changes=5
```

So the current boundary is:

- RX into the forced modem DSP core: working enough to trigger SIG/DIAL/page
  transitions from raw codewords.
- TX back out as a B-channel G.711 stream: still open. Prior V.8 capture work
  shows the generated transmit signal is not written back to `0x3f08/0x3f09`;
  it goes through the kernel SPORT0 TX/channel-table bridge or a task TX
  buffer that still needs to be wired into this forced-call path.

## Session 16: forcing SPORT0 TX from the assigned core

`tools/eicon_mips_shim.py` now has two TX helpers for the forced-call path:

- `--tx-source-scan` pokes candidate DM words with a marker and checks whether
  SPORT0 TX0 emits it.
- `--force-tx-samples N --force-tx-code BYTE` preloads the recovered source
  and captures the resulting SPORT0 TX0 words.

The key correction was to drive the kernel's RX-side TDM interrupt, not only
the explicit SPORT0 TX interrupt. The resident ISR writes TX0 during the
SPORT0_RX timeslot walk. The source scan found the practical output latch:

```text
[txscan] marker 0x0055 source hits: rx:DM2e52->0055
```

Forcing that latch proves byte/codeword-level outbound control:

```text
[force-tx] source DM2e52=0x0055: captured=16 forced=16
top=0055:16 first16=0055 0055 0055 0055 0055 0055 0055 0055 ...
```

So we can force G.711 TX now by preloading `DM 0x2e52` before each
SPORT0_RX-driven TDM slot. This is not yet the modem page's generated TX; it
is the kernel TDM output latch. The next recovery step is to connect the
task-side TX buffer (`DM 0x3fb4` pointer mode, expected target around
`DM 0x2b01`/`0x3f09`) to this latch, or identify where the page writes its
generated sample before the ISR emits `DM 0x2e52`.

## Session 17: tone-driven RX and live TX pointer bridge

`--g711-probe-samples` can now drive the forced modem DSP with synthesized
u-law stimuli:

- `--g711-probe-stimulus constant` preserves the old raw-byte probe.
- `--g711-probe-stimulus tone --g711-probe-freq 2100` feeds a stable tone.
- `--g711-probe-stimulus ansam` feeds a V.8-style 2100 Hz ANSam carrier with
  15 Hz amplitude modulation and 450 ms phase reversals.

The probe can also test a live TX bridge:

```text
--bridge-task-tx
```

This follows the firmware's current `DM 0x3fb4` pointer, copies
`DM[DM 0x3fb4]` into the recovered kernel TDM output latch `DM 0x2e52`, and
then strobes the RX-driven TDM ISR so SPORT0 TX0 emits that value.

With ANSam, the forced core enters a different overlay chain than flat
silence/idle: after `0x0270` and `0x0262`, it requests and serves
`0x0263-dial.f34-partial-overlay`, then moves to `0x0271-v.22fc-overlay`.
The TX pointer also changes from the old pointer-mode buffer to page-owned
addresses:

```text
sample 0002: ... 31AA=0263 3FB4=2277 TXPTR=0000
sample 0004: ... 31AA=0271 3FB4=3764 TXPTR=0000
```

A 512-sample 2100 Hz tone run proves RX tone drive and separates the two TX
effects:

```text
[g711] fed 512 tone 2100Hz amp=20000; line-state changes=486
[g711] bridged task TX DM[3FB4]->DM2e52:
  words=512 unique=4 non_idle=3 top=0000:509,10cd:1,0080:1,fc58:1
[g711] SPORT0 TX0 bridged captures:
  words=512 unique=4 non_idle=3 top=0000:509,10cd:1,0080:1,fc58:1
[g711] SPORT0 TX0 natural captures:
  words=512 unique=126 non_idle=179 top=0000:300,00ff:33,...
```

Interpretation:

- RX can now be driven with real G.711 tone waveforms, not just a flat byte.
- The kernel/TDM side naturally emits varying TX0 words while a tone is being
  received, but that is separate from the explicit task-pointer bridge.
- The live task TX pointer bridge is only seeing three startup non-idle words;
  after the page settles at `DM 0x3fb4 = 0x3764`, `DM[0x3764]` stays zero in
  this forced path. The next target is therefore the page initialization or
  action vector that arms sustained transmit generation, not the SPORT0 latch.

## Session 18: DM 0x3764 TX producer recovered

Static tracing of the extracted `0x0271` V.22FC overlay corrects the earlier
interpretation of `DM 0x3764`. It is not a persistent G.711 buffer and the
`0xfc58` found there in `dm.words` is not an idle code. The overlay initially
uses `DM 0x3680..0x37cb` as boot data; its loader at `PM 0x1dc5` copies that
material into PM and clears the runtime region. The same address range is then
reused as line-adapter state.

The overlay publishes its receive and transmit sample locations during init:

```text
PM 1dc0: AR = 3763
PM 1dc1: DM(3f0f) = AR       ; RX sample pointer
PM 1dc2: AR = 3764
PM 1dc3: DM(3fb4) = AR       ; TX sample pointer
```

The kernel task dereferences the TX pointer once per frame:

```text
PM 076a: I4 = DM(3fb3)
PM 076b: CALL (I4)           ; V.22FC PM 1d06 TX action
...
PM 07bb: I0 = DM(3fb4)
PM 07bc: SR1 = DM(I0,M0)     ; fetch DM 3764
```

`PM 0x1d06` is the line-side sample-rate adapter. Its final path at `0x1d46`
produces `DM 0x3764` from a 20-word circular queue:

| DM | Role |
|---:|---|
| `0x3761` | queued TX sample count |
| `0x3764` | current signed linear TX sample, one word per 8 kHz frame |
| `0x3765` | producer/write pointer, initialized to `0x36e0` |
| `0x3768` | consumer/read pointer, initialized to `0x36e0` |
| `0x36e0..0x36f3` | 20-word circular TX queue |

At `PM 0x1d46`, a nonzero queue count is decremented, one signed sample is read
through `I0/L0=0x14`, the read pointer is saved, and the sample is written to
`DM 0x3764`. An empty queue writes zero. The producer is `PM 0x1d69`; when the
phase/count test at `PM 0x1d1e` fires it synthesizes a block into the circular
queue and increments `DM 0x3761`. The V.22FC page initializer sets
`DM 0x3f67 = 6`, which is the block size used by the adjacent line adapter.

Therefore the sustained zero has a precise meaning: the V.22FC modem engine is
idle or is feeding zero-valued source samples, rather than the TX pointer or
SPORT bridge being broken. Also, `DM 0x3764` is **linear 16-bit PCM**, not a raw
G.711 octet. Copying it directly to the TDM latch is useful as a plumbing probe
but bypasses the TIKRNL post-processing beginning at `PM 0x07db`; natural TX
capture must remain the correctness path.

The forced G.711 probe now reports the `0x3764` adapter independently: number
of nonzero output frames, queue-count range, final read/write pointers, and the
maximum number of nonzero words observed in the 20-word queue. This separates
three failure cases on the next run:

1. queue count always zero: the producer is not being scheduled;
2. queue count advances but the ring remains zero: modem TX source is idle;
3. `DM 0x3764` varies but SPORT0 TX does not: fault is after the page adapter.

The first instrumented 512-frame, 2100 Hz run reports:

```text
[g711] V.22FC page TX adapter DM3764:
  frames=508 nonzero=0 queue-count=0..8
  write=DM36ea read=DM36e6 ring-nonzero-max=0/20 top=0000:508
```

This resolves the three-way test as case 2. The producer is definitely being
scheduled: the queue count reaches eight and both circular pointers advance.
However, no nonzero word ever appears in the queue, so `PM 0x1d46` correctly
emits zero on every steady-state frame. The next target is upstream of the
line adapter: trace the V.22FC engine's source block around `DM 0x3fa7` and its
`PM 0x3cba` action while `DM 0x3fb0 = 0x000c`, and determine which control or
call-progress event transitions that engine from idle to answer-tone TX.

## Session 19: BRI experiment and a simpler direct driver

The BRI suggestion was tested against download `0x0006`, `DIVA Server BRI 2M
Kernel`, selected by card type 60/file set 9. This file set uses the exact same
`TIKRNL81.F34`, V.8, V.34 and V.90 overlays as the working PRI set, which made
the kernel look like a promising drop-in replacement.

It is not ABI-compatible at the resident-kernel boundary. Its service jump
table is shifted (for example service slots `0x0001..0x001e` target different
resident routines), its SPORT layout differs, and the current PRI task
registration/resume assumptions do not hold. In the forced probe it repeatedly
requests `0x0270` SIG and never advances to DIAL. Moving to BRI therefore means
recovering a second set of kernel vectors and interrupt plumbing; it does not
address the zero-valued modem source inside the shared F34 task.

The useful simplification is instead to remove MIPS/PRI **call control**, while
retaining the already-understood `0x0009` kernel as a small compatibility
substrate. `tools/dial_tikrnl_drive.py` now accepts:

```text
--role idle|answer|calling
```

`answer` and `calling` write the ADDSP §5.4.1 data-pump database directly:
`GEN_SETUP0`, role-specific `GEN_SETUP1` (`0x0484` answer, `0x048c` calling),
`GEN_SETUP2`, `INFO0_SETUP`, `WSTATUS`, `Norm_H` and `Norm_L`. This path has one
emulated ADSP, no Unicorn, no MIPS protocol image, no CAPI/IDI entities, no
synthetic call object and no PRI timeslot assignment. The Linux Eicon driver's
`message.c` remains the format oracle for those modem B1 parameters, while the
ADDSP guide defines their DSP database representation.

A direct answer-side smoke test now reaches V.8 immediately:

```bash
python3 tools/dial_tikrnl_drive.py --role answer --freq 0 --frames 512
```

```text
role=answer
page switches: SIG -> V.22FC -> V8.F34
bootpage_nr 0006:512
GEN_SETUP1=0484 WSTATUS=2000 Norm_H=0001 Norm_L=0100
```

The matching calling-side run remains on V.22FC (`bootpage 0x000c`), proving
that the role bit is live rather than an inert poke. This direct harness is the
preferred bring-up path. BRI kernel emulation can be deferred unless actual
BRI hardware timing becomes a goal.

The apparent remaining switch-on requirement was then disproved. The direct
harness was only running TIKRNL's `PM 0x06c1` page/RX half. On hardware the
kernel separately invokes the callback TIKRNL registered at `PM 0x06fc` once
per SPORT sample. That continuation calls the secondary page action through
`DM 0x3fb3`, consumes the signed-linear sample through `DM 0x3fb4`, and runs
the TX post-processing. Without it, the answer-side V.8 page was selected but
its transmitter never ran.

`Card.frame()` now invokes both halves. A 1.5-second direct answer run with
silence on RX produces real modem TX without any MIPS switch-on command:

```bash
python3 tools/dial_tikrnl_drive.py \
  --role answer --freq 0 --frames 12000 \
  --tx-out artifacts/eicon-dsp/direct-answer-tx.s16
```

```text
DM[3FB4] signed-linear TX:
  pointer=3764 nonzero=7733/12000 first-nonzero=4267
```

The transmitter starts at sample 4267 (533.4 ms). FFT of the first 4096 active
samples peaks at 2099.6 Hz, with signed amplitude approximately
`-1677..+1820`: this is the expected V.8 answer carrier generated by the
shipping firmware. The direct output file is raw 8 kHz signed 16-bit
little-endian PCM. This is now the uncomplicated modem-driving path originally
wanted: one ADSP core, direct documented database writes, real overlay
switching, both TIKRNL sample callbacks, and captured generated TX.

## Session 20: firmware G.711 encoder called

Download `0x02bf`, `G.711 Overlay`, is real and contains A-law/µ-law conversion
code at PM `0x0913..0x0972`. It belongs to the voice-kernel family, however,
and loads over PM `0x08f0..0x0975`; loading it beside V.8 would overwrite the
active modem overlay interface.

The important discovery is that TIKRNL already carries the same conversion
algorithm as a resident utility. Its signed-linear-to-G.711 entry is
`PM 0x1810..0x182f`. The modem overlays leave this range untouched. The
PRI/E1 kernel selects the A-law parameter table through `DM 0x3309 = 0x35b7`.
The routine takes the signed sample in AR and returns a bit-reversed serial
codeword in the low byte of SR1. Reversing that octet produces conventional
G.711/RTP byte order (`0xab` from the DSP becomes A-law silence `0xd5`).

The emulator now exposes diagnostic AR/SR accessors, and the direct harness
can call the shipping encoder after collecting the modem samples:

```bash
python3 tools/dial_tikrnl_drive.py \
  --role answer --freq 0 --frames 5000 \
  --tx-out artifacts/eicon-dsp/direct-answer-tx.s16 \
  --g711-out artifacts/eicon-dsp/direct-answer-tx.alaw
```

```text
called TIKRNL PM 1810; wrote 5000 A-law octets
first16=d5 d5 d5 d5 d5 d5 d5 d5 d5 d5 d5 d5 d5 d5 d5 d5
```

The 5000-octet result has 113 distinct codewords. Decoding it with the local
independent G.711 implementation differs from the source linear PCM by at most
39 counts (mean absolute error 8.46), confirming the firmware routine and bit
order. Thus the direct harness now emits the actual firmware-companded DS0
stream; no software approximation and no destructive `0x02bf` page load are
needed.

## Session 21: direct SIP/RTP endpoint

`tools/eicon_adsp_sip.py` turns the direct TIKRNL harness into a callable SIP
endpoint. It intentionally implements only UDP INVITE/ACK/BYE/OPTIONS and
PCMA/8000, avoiding all card signalling and host-driver call objects. Incoming
RTP A-law octets are written byte-exact to TIKRNL's `DM 0x3f08` line interface.
For every octet the harness executes one `PM 0x06c1` frame pass and one
`PM 0x06fc` continuation. The resulting signed-linear sample at the pointer
in `DM 0x3fb4` is passed through the shipping G.711 routine at `PM 0x1810` on
a second emulated ADSP core, then sent as RTP payload type 8.

The media scheduler advances in exact 160-sample/20-ms quanta. It has no
resampler, transcoder, PLC, VAD, comfort noise, echo cancellation or gain
processing. A missing inbound sample is A-law silence; late scheduler wakeups
execute every elapsed sample quantum rather than changing sample accounting.

```bash
python3 tools/eicon_adsp_sip.py \
  --bind 0.0.0.0 --advertise 192.0.2.10 \
  --sip-port 5060 --rtp-port 4000 --law pcma \
  --capture-prefix artifacts/eicon-dsp/sip-answer
```

A local loop test completed INVITE/200, received three 172-byte packets (12
bytes RTP plus 160 A-law octets), and completed BYE/200. The initial media was
firmware-generated A-law silence (`d5`). The optimized `Card.frame_fast()`
runs 5000 modem samples in approximately 0.11 seconds on the development
machine, leaving ample margin for an 8-kHz real-time call.

`--capture-prefix` records every outbound packet in a raw-IP PCAP, appends its
payload byte-exact to a raw A-law file, and independently decodes it to an
8-kHz mono WAV for listening. A 60-second local SIP call with continuous
inbound A-law silence produced:

```text
3001 RTP packets / 480160 samples / 60.02 seconds
RTP timestamp step: 160
active packets: 231, one run from packet 26 through 256
active interval: 0.520 through 5.140 seconds
active RMS: 981.4 counts, range -1696..+1824
FFT peak: 2098.6 Hz
```

The answer modem therefore waits about 520 ms, emits its approximately 2100 Hz
answer signal continuously for 4.62 seconds, then returns to A-law silence
when the caller supplies no modem signal. It remains stable and silent for the
rest of the minute; there are no extra tones, page-switch loops, RTP timestamp
discontinuities, or emulator stalls. The retained files are:

```text
artifacts/eicon-dsp/sip-answer-60s.rtp.pcap
artifacts/eicon-dsp/sip-answer-60s.alaw
artifacts/eicon-dsp/sip-answer-60s.wav
```

The WAV can be played directly with `afplay` on macOS or any ordinary audio
player. The PCAP uses `LINKTYPE_RAW` and contains synthesized IPv4/UDP headers
plus the exact RTP packets sent on the socket.

A subsequent call used this repository's `sip_v90_modem` as a genuine call
modem rather than a silence generator. `ATD1` placed a peer-to-peer SIP call;
the endpoints negotiated raw PCMU/8000 and exchanged 488 packets before the
caller's V.8 timeout. Capture now records both directions (`.ulaw`/`.wav` for
ADSP TX and `.rx.ulaw`/`.rx.wav` for peer TX) in one bidirectional PCAP.

The caller detected the ADSP's ANSam and transmitted real V.8 CM at the
expected 980/1170-Hz DPSK frequencies. On replay, the Eicon firmware remained
in bootpage 6 (V.8) until sample 41271 (5.159 s), then selected bootpage 1
(`0x0266`, V.22/V.32 LEC) and bootpage 3 (`0x025c`, FSK OWN). This proves the
receive RTP reaches and controls the genuine firmware state machine. The
remaining failure is now protocol-level: the Eicon side emits no JM response
after that page transition, so the project caller times out V.8 after about
9.7 seconds. Captures are retained as:

```text
artifacts/eicon-dsp/sip-project-caller-pcmu.rtp.pcap
artifacts/eicon-dsp/sip-project-caller-pcmu.wav
artifacts/eicon-dsp/sip-project-caller-pcmu.rx.wav
```

PCMU is now the endpoint default because TIKRNL's `DM 0x3f08` modem interface
is µ-law. `--law pcma` remains available for E1 experiments and selects the
firmware A-law encoder table; it must not be used as an implicit transcoder.

## Session 22: PDF setup correction and physical Courier call

The ADDSP V.90 User's Guide v5.3 §5.3.1 and §5.4.1 Tables 12-15 exposed a
major direct-harness setup error. `Info0_setup`, `Norm_H`, and `Norm_L` are at
write-database offsets `0x07`, `0x28`, and `0x29`; the harness had written them
to `0x03`, `0x0f`, and `0x10`. The latter two are P2SD and the low-level
dialler range, not modulation masks. The corrected initialization now writes:

```text
GEN_SETUP0    +00 = 00c4
GEN_SETUP1    +01 = 0484             answer, 2-wire, internal clock, norm
GEN_SETUP2    +02 = 0030
V8_SETUP      +04 = 6000             V90_DPCM + digital network
INFO0_SETUP   +07 = f0fd
TD / TA       +08/+09 = 0006/0006
TX tune       +0a = 00ff
DCD off/hyst  +0b/+0c = 0030/0000
WSTATUS       +0e = 2000             change_wdb
NORM_H        +28 = 0001             V.8
NORM_L        +29 = 8100             V.90 + V.34
SPEED masks   +2a/+2b = 001f/ff00
```

With those values, the repository's software call modem changed from V.8
failure to a successful V.8 result (`status=2`) and entered V.34 training.
That confirms the PDF-defined offsets were operationally significant.

The SIP endpoint now also implements SIP REGISTER with MD5 digest auth. It
registered extension 6001 directly with the test Asterisk and accepted a real
call from the physical USRobotics Courier on `/dev/cu.usbserial-21210`. The
45-second call exchanged 2232 outbound RTP packets / 357120 samples and saved
both directions:

```text
artifacts/eicon-dsp/sip-courier-pcmu.rtp.pcap
artifacts/eicon-dsp/sip-courier-pcmu.ulaw
artifacts/eicon-dsp/sip-courier-pcmu.wav
artifacts/eicon-dsp/sip-courier-pcmu.rx.ulaw
artifacts/eicon-dsp/sip-courier-pcmu.rx.wav
```

This first Courier call did not train. The Eicon changed from V.8 page 6 to
page 25 / `V.OWN` (`0x026d`) almost immediately and emitted no answer tone.
The Courier reported no carrier and `ATI6` showed a keypress abort. This is a
better reference peer than the software modem, but it reveals another startup
or capability-selection issue rather than validating the full path. The SIP
endpoint now logs live bootpage changes with sample timestamps so the next
Courier attempt can distinguish actual live timing from offline replay.

The reason the first Courier call was inaudible was then isolated in its RX
capture: the FXS path delivered a near-full-scale off-hook transient at about
100 ms (`-32124..+32124`). That made DIAL select `V.OWN` at 160 ms, well before
the normal transmitter begins ANSam at about 533 ms, leaving TX silent. The
SIP endpoint now discards the first 1000 ms of received bearer audio while
still consuming every RTP sample, modelling the bearer-seizure settling time
without shifting its clock.

A guarded Courier call was audibly active for 19.8 seconds. The audible
intervals correlate exactly with page changes: the first ANSam run is
`0.520..2.040 s`, and it cuts out when the firmware transitions from page 6
V.8 to page 16 (`0x0265`, FAX.F34 Partial). It does **not** request page 7 /
`0x0260` INFO during this call. It returns to V.8 at 6.300 s and emits several
more 2100-Hz bursts before selecting page 2 / `0x0267` V.32 Partial at
17.420 s and emitting a 3000-Hz signal. The cut-out is therefore genuine
firmware behavior, but it is a wrong low-level/fax path rather than the
expected V.34/V.90 INFO phase. The captured TX had RMS 1411, range
`-6908..+8316`, and 89956 non-zero decoded samples:

```text
artifacts/eicon-dsp/sip-courier-pcmu-guarded2.rtp.pcap
artifacts/eicon-dsp/sip-courier-pcmu-guarded2.wav
artifacts/eicon-dsp/sip-courier-pcmu-guarded2.rx.wav
```

A second PDF pass found the V.90-specific setup at write-database offsets
`0x79..0x7f`. Merely setting `V8_SETUP.V90_DPCM` and `NORM_L.V90` is not
enough: both V.90 speed masks default to zero. The harness now additionally
sets `SPEED_SEL_V90_H=0x003f`, `SPEED_SEL_V90_L=0xffff`, and explicit maximum
V.34/V.90 rates. `INFO0D_SETUP=0x03b7` advertises lookahead 3, 3429-baud
upstream support, µ-law PCM, codec-output power measurement, and -12 dBm0
maximum power. PCMA mode sets its PCM-coding bit 6 dynamically.

Two immediate physical retries registered successfully but received no SIP
INVITE from the FXS/Asterisk path, despite the Courier producing its local Y4
call-progress dump. Their unique crash-safe captures correctly contain only
headers, so they are not modem results. The settings remain in place for the
next call that reaches the endpoint.

The guide's §2.2 ordinary host flow sets `WSTATUS.BOOTFINISHED` (`0x1000`)
after a download and then lets the kernel redispatch the task. Testing that bit
in the direct harness changed post-V.8 `TrnProgress` to `0x0040/0x0044`, but
physical calls regressed into repeated low-level/FAX pages and never reached
INFO. The direct harness already resumes through TIKRNL's registered
post-download entry `DM(0x31bb)`; doing that *and* setting BOOTFINISHED signals
completion twice. The extra bit was therefore reverted. This distinction is
specific to the direct-resume harness, not a contradiction of the documented
normal kernel flow.

The DSP's own diagnostic outputs are now retained once per RTP packet in
`PREFIX.adsp.csv`: live and event-latched `TrnProgress`, `Rstatus_ch`,
`Rstatus`, change flags, bootpage/overlay, and all three eye-pattern words.
`PREFIX.adsp-dm.bin` originally snapshotted the 128 DSP-owned read-database
words every 20 ms. Format `EADSPDM2` now retains the complete 256-word
memory-mapped interface at `DM 0x3EE0..0x3FDF`: all 128 host-written setup
words followed by all 128 DSP-written status words. This preserves activation
strobes and selected setup alongside rate formats, `ErrorMessage`, detector
levels, reserved live state, and undocumented diagnostic values. Each record
is a little-endian `uint64` sample number followed by 256 `uint16` words.
The fields come from guide §5.3.2 and §6.6 (`EYESAMPLE_0` for V.8,
`EYESAMPLE_2` for INFO). This distinguishes a host timeout from a DSP state
stall on the next physical call.

The retained `sip-courier-live-20260728-181057.log` confirms the audible
"INFO then timeout" report. Three Courier calls cleanly selected page 7 /
`0x0260` INFO at 3.24-3.52 s. Their DSP-owned `TrnProgress` advanced through
`0x22/0x24`, `0x26`, and `0x28`, then stopped at `0x28` until the calls ended.
This is materially different from the later FAX/V.OWN attempts and proves the
Courier reached V.34/V.90 phase 2. A later A/B test showed that additionally
setting `WSTATUS.BOOTFINISHED` prevents this direct-resume path from reaching
INFO, so the INFO stall has a different cause.

The complete read-database snapshots exposed the larger missing flow: event
flags and their latched `RSTATUS_CH_dbs`, `RSTATUS_dbs`, and
`TRNPROGRESS_dbs` copies remain identically zero while the live E0-E2 words
change. Per §5.3.2 these mirrors are populated during the Host-Kernel RX_2400
communication cycle. The direct SIP harness calls TIKRNL entries as ordinary
subroutines and never runs that kernel/SPORT host cycle. It also differs from
the recovered MIPS flow, which sets BOOTFINISHED, restores assigned PCM buffer
pointers after non-V.8 overlays, and resumes the registered completion through
the kernel foreground slot. Therefore further setup-bit guessing cannot make
the direct-call path faithful; the SIP media loop needs to use the existing
`KernelDispatch` SPORT/doorbell path while retaining direct database call
activation.

A second concrete layering omission was found by comparing `Card.boot()` with
that kernel-driven harness. The `.F34` images are partial overlays: before
DIAL, the real flow layers V.OWN (`0x026d`) and FSK OWN (`0x025c`) underneath
it. DIAL calls shared routines beyond its own image, notably PM `0x244c` and
`0x2c4f`. The SIP direct harness loaded DIAL alone, leaving those targets as
cleared task memory or stale content and making classification vary among
INFO, FAX, and V.OWN across otherwise similar calls. `Card.boot()` now loads
V.OWN, FSK OWN, then DIAL in the recovered order. The direct-resume harness
must not also set the ordinary host/kernel BOOTFINISHED acknowledgement: doing
both completes a download twice. A fresh physical capture is required to
determine whether the remaining Host-Kernel RX_2400 cycle is still necessary
after fixing the base-image layer.

## Recovered: V.8's indirect page-7 handoff

The V.8 overlay does select INFO, but there is deliberately no immediate
`DM(0x3fb0) = 7` at the V.8 decision point.  The handoff has three stages:

1. PM `0x3ba1..0x3bfb` classifies the V.8 result.  On the normal-modem path,
   `DM(0x3fc4) & 0x0100` branches at `0x3ba7` to `0x3bc8`, which loads `AR=7`
   and stores it in the pending-page word `DM(0x0491)` at `0x3bfb`.  The
   alternate entry at `0x3c18..0x3c27` can also store 7 there.
2. A completion callback sets `DM(0x075b)`.  PM `0x372c..0x3763` then counts
   25 callback invocations in `DM(0x06b3)` before copying the pending page
   from `DM(0x0491)` to `bootpage_nr` (`DM(0x3fb0)`) at PM `0x3761` and
   setting the page-change strobe in `DM(0x3fc1)`.
3. TIKRNL sees that strobe, indexes its bootpage table at `DM(0x31d5)`, and
   requests download `0x0260`; the host only serves and acknowledges that
   request.  INFO is therefore not a direct ADSP `CALL` from V.8.

Replay of the first successful Courier capture confirms the exact path.  At
sample 27332, `DM(0x3fc4)=0xa100` selects page 7 into `DM(0x0491)`.  At sample
27487 the completion flag becomes non-zero.  At sample 27572 the counter moves
from 24 to 25, PM `0x3761` copies 7 into `DM(0x3fb0)`, and TIKRNL requests the
INFO overlay.  In a failed low-level/FAX call, `DM(0x0491)` never becomes 7;
V.8 classifies the input first and changes directly to page 16.  Thus a
missing page-7 transition is upstream of the overlay loader: either V.8 did
not set pending page 7 (inspect `0x0491`/`0x3fc4`) or its delayed completion
callback did not run (inspect `0x075b`/`0x06b3`).

## Replay result: page 7 loads; INFO receive acquisition stalls

Replaying the first successful Courier RX capture through the current layered
`Card` path proves the complete request and load path.  At sample 27573
(3.447 s) TIKRNL requested page 7, the harness served `0x0260`, and the
resident image became INFO.  `TrnProgress` then advanced
`0x20 -> 0x22 -> 0x24 -> 0x26 -> 0x28`; it remained at `0x28` through sample
80000.  The SIP endpoint now logs the served request explicitly rather than
only reporting the bootpage at the next 160-sample packet boundary.

INFO transmission is active after the switch.  The replayed DSP output is
non-zero continuously after `TrnProgress=0x28`, and the offline Phase 1/2
decoder recovers a V.90 `INFO0d` from it.  The failure is on INFO receive
acquisition: at the stall, the control-channel parser remains at its initial
PM state (`DM(0x16bd)=0x3520`) and its completion flag `DM(0x0686)` remains
zero.  PM `0x3574..0x358d` sets that flag only after the receive parser accepts
its bit sequence, while the state-`0x28` condition at PM `0x33a3..0x33cb`
continues testing it.  The peer capture contains control-channel energy and a
Tone-A candidate, but the independent decoder also fails to recover a valid
INFO0 frame from that interval.  Therefore the next investigation is the
INFO page's control-channel RX input/carrier/framing path, not page mapping,
overlay loading, or INFO transmission.

A subsequent live call exposed a separate direct-harness drive bug on page 16.
The same still-asserted request was treated as a new download up to eight times
per sample, repeatedly resetting the resident partial overlay and flooding the
synchronous log enough to make media processing appear stalled.  `Card.frame`
and `Card.frame_fast` now resume a request for the already-resident download
without reloading it or recording a duplicate page switch.  Replays retain the
single page-7/`0x0260` transition and no longer produce thousands of destructive
page-16 reloads.

The next physical calls confirmed a distinct policy gap: after exchanging V.8
signals the peer went quiet waiting for Phase 2, while the firmware selected
page 16 or V.32 rather than setting pending page 7.  An opt-in
`--force-info-after-v8` diagnostic now replaces the first post-V.8 low-level
fallback (after 1.5 s) with page 7/`0x0260`; natural page-7 requests are
untouched.  Replay of the first affected call changes the path from page 16 to
INFO and advances `TrnProgress` through `0x20/0x22/0x24/0x26/0x28` to `0x2a`.
This validates the host-policy hypothesis strongly enough for a live A/B call,
but the override remains diagnostic rather than a claim about the shipping
supervisor's exact acceptance gate.

The first forced live A/B calls show that the INFO microstate cadence itself is
not anomalously fast: `0x20 -> 0x2a` takes about 140-160 ms, comparable to the
natural page-7 captures.  The uncertain timing is the V.8-to-INFO seam.  In the
second call page 7 loaded at 2.055 s, while the peer-side recording contains a
CRC-valid INFO0c candidate beginning at 1.889 s, so waiting for the DSP's
fallback request can actually enter INFO too late and miss the peer's first
control-channel frame.

The open-source `divas4linux` driver does not schedule page 7.  Its
`kernel/message.c:add_b1()` builds a modem CAI and hands the call to the closed
MIPS protocol firmware.  That CAI includes call direction, digital-modem use,
modulation masks, exact answer-tone duration, answer-tone delay, carrier-wait
time and carrier-loss time.  The ADDSP guide §5.4.2 confirms that the MIPS
supervisor starts 40 s abort and 15 s training timers when the DSP requests
V.8, monitors published `TrnProgress`, and initiates retrain/disconnect policy;
it does not advance INFO microstates directly.  Our direct path bypasses both
the CAI-to-database call setup and the Host-Kernel RX_2400 publication cycle,
so the next faithful fix is to recover those MIPS-derived timing/setup writes,
not add sleeps between INFO states.

The natural V.8 completion gate is now pinned down.  At the protocol level it
is answer-side CJ reception after the modem has sent JM.  In the successful
firmware replay, that event sets `TrnProgress=0x0009` and dispatches PM
`0x3ba1`; with `DM(0x3eaa)&0x0060 == 0` and `DM(0x3fc4)&0x0100 != 0`, PM
`0x3bc8/0x3bfb` stores pending page 7 in `DM(0x0491)`.  A later transmitter
completion calls PM `0x3b95`, setting `DM(0x075b)=1`; PM `0x372c..0x3761`
counts 25 callbacks in `DM(0x06b3)` before publishing page 7.  The forced
calls never produce this gate naturally: they remain at `TrnProgress=0x0004`
and `DM(0x0491)=0` until the diagnostic override.  New captures retain all
five gate words and label forced page-7 requests explicitly.

## Blocker isolated: direct INFO RX is disconnected

A sample-by-sample replay of the first successful Courier call separates the
remaining failure from INFO framing.  In the direct `Card.frame_fast()` path,
page 7 loads at sample 27572 and publishes `DM(0x3f0f)=0x3763`, but
`DM(0x3763)` becomes zero and stays zero while the incoming `DM(0x3f08)`
codeword continues changing.  Consequently the INFO parser never leaves
`DM(0x16bd)=0x3520`, `DM(0x0686)` stays zero, and `TrnProgress` stops at
`0x0028`.  V.8 had its own active G.711 line adapter at the same pointer;
INFO depends on the assigned kernel/SPORT path that the direct subroutine
harness bypasses.

Replaying the identical RX bytes through `LiveKernelModem` proves the point.
The kernel path advances through `0x20/0x22/0x24/0x26/0x28` and then reaches
`0x2a/0x2b` at 3.642 s, corresponding to acquisition of the peer's Tone A.
It cannot progress further on a fixed replay because the recorded Courier
cannot react to the newly generated response.  Both paths transmit a
decodable INFO0d at 3.590 s, but the direct path incorrectly advertises A-law
in INFO0d bit 39 on a PCMU call; the kernel path advertises µ-law.  Thus the
next meaningful hardware test is `tools/eicon_adsp_sip.py --kernel-dispatch`.
A fresh call, rather than another replay, is required to determine whether the
Courier acknowledges that corrected INFO0d and advances beyond `0x2b`.

## INFO `0x37` terminal FFT corruption

Execution watchpoints on the INFO analysis sequencer (`PM 0x36ed`) show the
normal transform actions entering `PM 0x376e`, `0x3771`, and `0x3774` with
`DM(0x16c5..0x16c7)` progressing from `0x0080/0x0002/0x0004` to the terminal
`0x0001/0x0100/0x0200`.  At the `0x37` failure boundary, repeated analysis-result publication has
already advanced the linear pointer `DM(0x15f3)` beyond its 20-word buffer at
`0x0ddd..0x0df0`.  PM `0x323e..0x3244` appends two words per analysis and,
because detector completion never occurs, eventually writes through
`DM(0x0e4c)`, which holds the second `PM 0x373a` reset action in the active
analysis sequence.  It replaces that action with `0xffed`.  The sequencer
therefore reaches its second `PM 0x376e` transform without resetting the
terminal `0x0001/0x0100/0x0200` parameters.  The next stage shifts the span
to zero and doubles count/stride to `0x0200/0x0400`.  The indirect butterfly
stores at PM `0x3792/0x3794` then escape the `0x1110` work buffer and overwrite
the INFO control workspace.  The first consequential overwrite is
`DM(0x16b6)=0xffec`; PM
`0x217d..0x217f` subsequently copies that invalid variant selector to the
shared boot-page word.  The later `DI_control=0xfd00`, `BaudInfo=0x3000`, and
status values are downstream corruption, not host requests.

An independent emulator defect was also found in opcode class `0x10` (shift
with internal register move): the core executed the shift before sampling the
parallel move source.  INFO PM `0x25fc` shifts a new value into SR while moving
the preceding `SR1` accumulator to AR, so the old ordering forced AR to read
the newly cleared SR1.  The core now samples the move source first, with a
regression test using the firmware's exact `0x1013af` opcode.  The captured
candidate still fails earlier validation at PM `0x25e9..0x25f1`, so this fix
is necessary instruction semantics but is not by itself the `0x37` cure.

This is evidence of a missing or incorrectly timed emulator path rather than
a valid firmware transition.  The stale classifier/event value
`DM(0x198e)=0x06a6` is present at every watched transform entry.  Do not hide
the problem with an FFT bounds check: trace why the sequencer skips its reset
or why the detector completion fails to stop/reconfigure that sequence.

## The control-channel framer is not the `0x37` fault

The detector completion that never occurs is `DM(0x0686)`, published by the
INFO page's control-channel framer.  That framer has now been recovered and
exercised in isolation, and it works: the fault is upstream of it.

Two stages sit between the line and `DM(0x0686)`:

- **PM `0x34f0`, the demodulator.**  A 16-word circular sample history at
  `DM(0x16bb)` (`L0 = 0x10`) is correlated against the 16-tap reference at
  `DM(0x1554)`.  PM `0x350b` takes `|MR1|`; over `DM(0x164f)` it raises the
  energy flag `DM(0x0685)`, and over the immediate `0x0578` it becomes the
  one-bit decision published in `DM(0x060f)` at PM `0x3515`.  The magnitude
  itself lives only in `AR`/`AX1` and is never stored to DM.
- **PM `0x3520` and PM `0x25ab`, two framers.**  PM `0x3515` runs framer A
  through `DM(0x16bd)` and then falls into framer B (the `JUMP $25A0` at PM
  `0x351f`), once per demodulated sample.

Both framers keep 16 lanes in a circular buffer, one per sample phase of a
16x oversampled bit, advancing one lane per call:

|  | framer A | framer B |
|---|---|---|
| state | `DM(0x16bd)`, hunt `0x3520` | `DM(0x19cf)`, hunt `0x25ab` |
| lanes | `DM(0x0620..0x062f)` | `DM(0x1990..0x199f)` |
| bit planes | `DM(0x068c..)` | `DM(0x19d0..)` |
| call count | `DM(0x068a)` | `DM(0x19cd)` |
| payload | `DM(0x1651)`: `0x0110`/`0x01e0` (17/30 bits), by `DM(0x3f94)` bit 1 | fixed `0x0080` (8 bits) |
| success | `DM(0x0686) = 1` | `DM(0x198e)` event, `DM(0x198f)` octet |

A lane hunts an 11-bit window equal to `0x0772` — one fill bit followed by
the V.34 INFO synchronization code `0x372`, the same constant as
`V34_INFO_SYNC_CODE` in `v34_info_decode.h` — five times, then accumulates
CRC-16 (reflected `0x8408`, preset `0xffff`) over the payload while the bit
planes collect every lane's decision.  The received CRC is shifted in against
each lane's own register, so the lane whose residue is zero is the one that
sampled on the correct phase.  PM `0x3568`/`0x25e9` is that zero scan — the
validation the previous session saw fail — and PM `0x3574`/`0x25f5`
transposes the bit planes to recover the winning lane's payload.

`tools/info_cc_framer_probe.py` drives PM `0x3515` directly with ideal
decisions (each bit repeated across all 16 lanes) after running the
firmware's own initializers, PM `0x359a` and PM `0x3f7f`.  Framer A locks
sync, accepts its 17-bit payload, validates the CRC and sets
`DM(0x0686) = 1`.  So the framer, the 16-lane phase search and the emulated
instruction semantics along that path — including the opcode-class `0x10`
fix above — are all correct.  Framer B behaves identically when fed its own
8-bit message (it recovers `DM(0x198f) = 0x30` and publishes
`DM(0x198e) = 1`, the event the `_inject_l1l2_completion` gate fakes), but
note that the INFO page initializer at PM `0x3f4c` deliberately parks framer
B at the disabled handler `0x25f3`; PM `0x2602` is what installs it, and
nothing in the resident image references `0x2602` directly — it is reached
only through the PM action table at `0x2ee6..0x2eee`.

The remaining candidate is therefore the decision itself.  `DM(0x060f)` is a
hard threshold on a correlation magnitude against fixed constants
(`DM(0x164f)`, `0x0578`) that assume the real card's signal levels; nothing
downstream can recover if that bit is stuck or noisy.  The `[EXEC]`
watchpoint line now carries `ax1`/`ar`/`mr1` for this reason, and
`tools/eicon_adsp_sip.py --watch-exec 0x3515` logs the magnitude per sample
on a live call.  The next measurement is that magnitude against `0x0578`
over the INFO window: a magnitude that never crosses, or never stops
crossing, is a level/scaling fault in the emulated RX path, not a framing
one.  Note this repository's standing μ-law level gotcha (0 dBm0 is RMS
16017, not 4004) when interpreting it.

## Live against slmodemd: no `0x37` stall, no FFT corruption

Two calls from the tower rig's SmartLink softmodem (`slmodemd_trnref` behind
d-modem, dialling `ATD6001` into `tools/eicon_adsp_sip.py --kernel-dispatch`
registered as 6001) settle the framer question live and move the frontier.
The second call ran without `--init-info-detector-at-24`; both are
byte-identical in outcome, so that diagnostic is not load-bearing against
this peer.  Traces: `artifacts/eicon-live/run01.adsp.csv`, `run02.adsp.csv`.

What works, none of it forced:

- V.8 completes naturally (`TrnProgress 0x0004 -> 0x0003 -> 0x0009`) and the
  DSP requests page 7 on its own at ~5.57 s.  `--force-info-after-v8` is not
  needed against this peer.
- The peer decodes our INFO0a: `V34INFO, rxinfo0 0xbf,0x84,0x07,0x68,0x32`,
  logged by slmodemd as `rxinfo0a`.
- **The control-channel framer runs and validates.**  `DM(0x0686)` is set in
  749/754 of the 1028 captured 20 ms windows, and `DM(0x16bd)` cycles
  `0x3520 -> 0x3546 -> 0x3561` throughout.  The "detector completion never
  occurs" symptom is Courier-specific; the framer analysis above holds live.
- **No FFT corruption.**  `DM(0x15f3)` advances only `0x0ddd -> 0x0de9`,
  inside its 20-word buffer; `DM(0x0e4c)` holds `0x373a`/`0x376e`, the real
  reset and transform actions, never `0xffed`; `DM(0x16b6)` never leaves
  `0x0000`.  Zero anomalies in either run.
- INFO advances `0x20 -> 0x24 -> 0x26 -> 0x28 -> 0x2e -> 0x30 -> 0x32 ->
  0x34 -> 0x36 -> 0x37` in ~1.3 s.  It does not stall at `0x37`.

Where it actually fails: after ~120 ms in `0x37` the sequencer leaves for
state `0x0010` and stays there for the rest of the call.  The exit is a
normal, understood transition, not a fault in itself.  PM `0x3335` is the
INFO sequencer: it counts `DM(0x1647)` down, then calls the pre-condition
`DM(0x169a)` and up to four condition handlers `DM(0x1696..0x1699)`, taking
the first that returns LE and loading the matching next state from
`DM(0x1692..0x1695)`.  In state `0x37` those handlers are `0x33c4`,
`0x33c2`, `0x33c2` and `0x2476`; `0x33c2` is `AR = 0 + 1`, a stub that never
fires, `0x2476` tests framer B's event `DM(0x198e)` against 1, and `0x33c4`
falls through `0x33a3` to `AR = DM(0x0686) XOR 1` — it fires when framer A
completes.  The capture shows `DM(0x0686)` going to 1 in the window before
the transition, so `0x37` ends because a genuine CRC-valid control frame
arrived, exactly as designed.  `DM(0x1647)` still had `0x0a2f` left, so this
is not a timeout.

The real defect is what `0x0010` does.  `DM(0x3fb4)`'s sample is zero for the
whole `0x34..0x37` window — correct, the digital modem is listening for the
analogue modem's L1/L2 — and resumes at the `0x0010` transition.  The peer
decodes that resumed transmission as a **second INFO0a**
(`0xbf,0x84,0x87,0x68,0x29`, differing from the first in bit `0x80` of octet
2), returns to `TX_PHASE1_ANS`, and gets nothing further; it reports
`vpcm: Link Error` 13 s later.  So instead of proceeding from the INFO0
exchange to line probing and INFO1, the page drops back and repeats INFO0a.

Note that framer B never publishes: `DM(0x198e)` and `DM(0x198f)` stay zero
for the entire call while its state cycles `0x25ab -> 0x25c7 -> 0x25e2`.
That is consistent rather than alarming — framer B's payload is fixed at 8
bits where framer A takes the 17 that `DM(0x1651)` selects, so the two are
alternative message formats and this peer only sends the longer one.

Next: identify what state `0x0010` is meant to be and why `0x37`'s framer-A
exit targets it.  `DM(0x1695)`/`DM(0x1692)` hold the candidate next states at
that point, and PM `0x331e`/`0x334d` are where the winning one is committed;
capture those four words across the transition rather than inferring the
mapping from `TrnProgress` alone.

## The `0x37` exit: candidates captured, and the missing event

`tools/eicon_info_replay.py` replays a `.rx.ulaw` capture through a fresh
`LiveKernelModem`.  Our transmission cannot affect an already-recorded RX, so
the replay reproduces `run02`'s state path sample for sample and any DM word
can be instrumented without dialling the rig.

### The candidate table across the transition

`DM(0x1692..0x1695)` and `DM(0x1696..0x1699)` through the window:

| time | state | next0/test0 | next1/test1 | next2/test2 | next3/test3 |
|---|---|---|---|---|---|
| 6.1559 | `0x34` | `0a9d`/`33c4` | `0836`/`3384` | `0b69`/`33c2` | `0b69`/`33c2` |
| 6.5571 | `0x36` | `0a9d`/`33c4` | `0836`/`33c2` | `0b69`/`33c2` | `098f`/`3384` |
| 6.5671 | `0x36` | `0a9d`/`33c4` | `0836`/`33c2` | `0b69`/`33c2` | `1736`/`2476` |
| 6.6508 | `0x37` | `0a9d`/`33c4` | `0914`/`33c2` | `08d5`/`33c2` | `1736`/`2476` |
| 6.7371 | `0x10` | `0a9d`/`33c2` | `0a9d`/`339b` | `08d5`/`33c2` | `1736`/`2476` |

`0x33c2` is `AR = 0 + 1`, a stub that never fires, so state `0x37` has exactly
two live exits: `0x33c4` (framer A completed, `DM(0x0686) == 1`) to `0x0a9d`,
and `0x2476` (`DM(0x198e) == 1`) to `0x1736`.  Note `0x36` arms a timer exit
first and re-arms 10 ms later to the `DM(0x198e)` test — the sequencer is
deliberately set up to wait for that event across `0x36`/`0x37`.

`0x0a9d` is not a mis-set candidate: it is the intended framer-A successor,
and it is the state-`0x0010` script.  So the transition is the firmware taking
its documented fallback because the branch it is actually waiting for never
becomes true.  `DM(0x1647)` still held `0x0a15`, so nothing timed out.

### What is missing while we are silent

Transmit activity per state, from the same replay:

```
  6.1559s  0x0034    3210 samples    2.0% non-zero TX
  6.5571s  0x0036     534 samples    0.0% non-zero TX
  6.6239s  0x0037     906 samples    0.0% non-zero TX
  6.7371s  0x0010  109783 samples   99.4% non-zero TX
```

The 580 ms of silence is correct — the digital modem is listening.  Nothing is
missing from our transmit path.  What is missing is the input event.

The peer is presenting exactly what should raise it.  Complex demodulation of
the captured RX at 2400 Hz shows a steady Tone A from ~5.5 s at magnitude
~1738, with 180-degree phase reversals starting at 6.59 s: a burst at
6.59-6.74 s and another at 6.97-7.36 s.  The first burst coincides with our
`0x36` (6.557 s) and `0x37` (6.624 s) window almost exactly.

Injecting the event confirms the diagnosis.  With `DM(0x198e) = 1` written on
first reaching `0x37`, the sequencer takes `0x2476` instead:

```
  6.6239s  0x0037       5 samples    0.0% non-zero TX
  6.6245s  0x00a0      10 samples    0.0% non-zero TX
  6.6258s  0x00a2    6790 samples   99.3% non-zero TX
  7.4745s  0x00ab     111 samples   28.8% non-zero TX
```

State `0x00a2` transmits for 848 ms — the phase-1 response we currently never
send.  The `0xa0`/`0xa2`/`0xab` family had never been reached before.  Past
that point the replay is open loop and says nothing about what the call would
have done.

### Why the event is never published

`DM(0x198e)` has five writers in the INFO image and four of them clear it.
The only one that sets a value is PM `0x2470..0x2474`, the match arm of the
classifier PM `0x2461`: it stores `I6 - 0x1986`, the index of the matched
message code in the 8-entry table PM `0x2410` builds at `DM(0x1986)`
(`0x30`, `0x50 | DM(0x3f4b) & 0x0f`, `0x70`, `0x90`, `0xb0`, `0xd0`, `0x40`,
`0x60`).  Event 1 is therefore "the `0x50` message was received".

That classifier is reachable from framer B's success path (PM `0x2600`) and
from framer A's (PM `0x3587`) — but PM `0x357e` compares `DM(0x1651)` against
the length the INFO mode word selects, which is the value PM `0x3f7f` just
wrote there, so that test always takes the equal branch and PM `0x3583` only
lets the classify path run when `DM(0x1651) == 0x0080`.  With `DM(0x1651) =
0x0110` on this call, framer A can never publish an event.  Framer B has the
fixed `0x0080` length and is the intended publisher — and the INFO page
initializer at PM `0x3f4c` parks it at the disabled handler `0x25f3`, with
only PM `0x2602` installing it, which nothing in the resident image calls.

Installing it is necessary but not sufficient: `run01` ran with
`--init-info-detector-at-24`, which does call PM `0x2602`, and framer B cycled
`0x25ab -> 0x25c7 -> 0x25e2` all call without ever validating.  This peer is
transmitting a phase-reversed tone in that window, not an 8-bit control-channel
message, so there is nothing for framer B to decode.

Open question, and the next thing to settle: what is supposed to raise event 1
against a tone-only peer.  Either the probing/tone classifier reaches PM
`0x2470` by a path not yet found, or `DM(0x3f4b)` — tested for bits `0x10` and
`0x80` by the neighbouring condition handlers PM `0x2495`, `0x249a`, `0x24a9`
and folded into the table entry itself — is the tone-detector's output and the
gap is upstream of the classifier.  Resolve that before adding any injection
to the live path.

## What raises event 1: not the MIPS, and not a tone

The previous section left open what publishes `DM(0x198e) = 1` for a peer that
looks tone-only in the `0x36`/`0x37` window.  Three findings close it.

### The MIPS supervisor cannot be the source

`DM(0x198e)` has exactly one writer that stores a non-zero value: PM
`0x2470..0x2474`, the match arm of the classifier PM `0x2461`.  PM `0x2461` is
entered from PM `0x2600` (framer B's success path) and PM `0x3587` (framer
A's), and from nowhere else — no PM data word anywhere in the loaded image
holds `0x2461 << 8` or `0x2470 << 8`, so no script table can dispatch either.
The event is therefore raised inside the DSP, by a framer, or not at all.

That matches the host's documented role.  `DM(0x198e)` is overlay-private
scratch, outside the host-visible database window `0x3ee0..0x3fdf` that
`.adsp-dm.bin` snapshots, and the ADDSP guide's line follow-up only monitors
`TrnProgress`, runs the training/response timers and decides retrain policy.
The guide even anticipates this exact transition: "It is also possible that,
because of some recovery mechanisms in the training, the TrnProgress is
smaller than LastStatus", handled by counting `RetrainAutofallbackcount` and
forcing a fresh retrain past 10.  So `0x37 -> 0x0010` is a recovery the host
design expects to see occasionally — the defect is that we take it every time,
not that we take it.

### The peer is not tone-only there

The 2400 Hz carrier's phase reversals are its DPSK transmission, not tone
timing: slmodemd's own log shows `txstate TONE_AB=>TX_DPSK` at 667.464 and
back at 668.263, which maps to our 6.34-7.14 s — the reversal bursts measured
at 6.59-6.74 s and 6.97-7.36 s fall inside it.  Reading those bursts as Tone A
reversals in the previous section was wrong.

And the receiver decodes it.  Reconstructing framer A's bit planes
(`DM(0x068c..)`) per lane at each `DM(0x0686)` 0->1 edge gives a real message,
twice, with the same payload:

```
  5.568s trn=0028   lanes 1-13:  1 1111 1111 0000 1000
  6.710s trn=0037   lanes 2-13:  1 1111 1111 0000 1000
```

Twelve of the sixteen sample-phase lanes agree exactly, and the firmware's own
zero-scan accepted one, so the received CRC matched the computed `0x9bf1`.
The demodulator, the 16-lane phase search and the framer all work on real
signal — this is a genuine 17-bit control-channel message, not a false lock.

### Why the event branch is structurally dead

PM `0x3583` only lets framer A reach the classifier when `DM(0x1651) ==
0x0080`.  `DM(0x1651)` has a single writer, PM `0x3f84`, fed by PM `0x3f7f`,
which stores `0x0110` or `0x01e0` depending on `DM(0x3f94)` bit 1.  **It can
never hold `0x0080`**, so framer A can never publish an event in this build.

Framer B is hard-wired to the `0x0080` length (PM `0x25c5`) and is therefore
the only possible publisher — and the INFO page initializer PM `0x3f4c` parks
it at the disabled handler `0x25f3`, with only PM `0x2602` installing it and
nothing in the resident image calling that.  `run01` did call it via
`--init-info-detector-at-24`; framer B then cycled `0x25ab -> 0x25c7 ->
0x25e2` all call without validating, because the peer is sending 17-bit
messages in that window, not 8-bit ones.

So the `0x37` state offers two exits and only one of them is live for this
traffic: "17-bit message received" to `0x0a9d`/state `0x0010`, and "8-bit
message 1 received" to `0x1736`, which needs a message class the peer never
sends here.  The question is no longer what raises event 1 — it is why the
page is configured for 17-bit messages at a point in the handshake where its
own state graph expects the 8-bit class, i.e. what should have set
`DM(0x3f94)`/`DM(0x1651)` differently, or which earlier state should have run
the PM `0x2ee6..0x2eee` action block (`0x2410` table build, `0x2602` framer B
install, and the `DM(0x3f4b)` flag actions) that nothing in our run ever
enters.  `DM(0x3f4b)` stays `0x0000` for the whole call, which is consistent
with that block never running.

## The action block gates the transmitter, and running it unblocks the peer

### DM(0x3f94) and DM(0x1651) are already correct

`DM(0x3f94)` is set by the V.8 overlay, not the INFO page: PM `0x38a1`/`0x38a2`
store `0x0009` when the V.8 result has bit `0x0008`, PM `0x38a6`/`0x38a7`
store `0x0006` for bit `0x0004`, with `0x0008`/`0x0000` at PM `0x382e`/`0x385e`.
The INFO overlay's only writer, PM `0x3db5`, merely clears bit 1 when
`DM(0x3f93) & 0x0010` is zero.  Our calls get `0x0009`, so PM `0x330c` selects
INFO variant 8 (`DM(0x16b6) = 8`) and PM `0x3f7f` derives `DM(0x1651) =
0x0110`.  That is the V.90 path and the 17-bit message class that goes with
it; the `0x01e0` alternative belongs to mode `0x0006`.  Nothing is
misconfigured here, and the previous section's framing of this as a
misconfiguration was wrong — the 8-bit class simply is not this variant's.

### What the action block actually does

PM `0x2ee6..0x2eee` is a dispatch table indexed by action code, executed by the
script interpreter PM `0x2148` through the pointer `DM(0x1667)`:

| code | entry | effect |
|---|---|---|
| 0 | `0x2410` | build the message table at `DM(0x1986)`; `DM(0x16af) = 1` |
| 1 | `0x2602` | install framer B, call `0x2410`, then `DM(0x16af) = 0` |
| 2 | `0x242b` | clear `DM(0x3f4b)` bit `0x80` |
| 3 | `0x2430` | disable framer B; transmit message 0 |
| 4-8 | `0x243d`, `0x2441`, `0x243f`, `0x2434`, `0x243b` | transmit message 3, 5, 4, 1, 2 |

The transmit arms all reach PM `0x2446`, which builds an outgoing frame at
`DM(0x16a5)` — `0x0010`, then `0x0f72` (fill bits plus the sync `0x372`), the
message octet, and a CRC from PM `0x3aa4` (CRC-16-CCITT, `0x1021`, MSB first;
the framers use the reflected `0x8408` form of the same polynomial).  So this
block is the 8-bit control-channel transmitter, and action 7 sends message 1 —
the very code event 1 waits to receive.

`DM(0x16af)` is not a mute.  PM `0x3b0e..0x3b13` decrements it every sample and
reloads it with 4 on reaching zero, clocking the next bit out of the message
buffer: it is the transmit bit-clock divider.  Setting it to 0 — which is
exactly PM `0x2602`'s last instruction, PM `0x2609` — makes the countdown run
away for 65535 samples, so the modulator's carrier keeps running unmodulated
instead of going idle.

### Replay: the carrier is 1200 Hz

Dispatching action 1 at state `0x34` in replay turns the `0x34..0x37` window
from silent into a continuous, clean **1200 Hz** tone at rms 2048.  Writing
`DM(0x16af) = 0` directly does the same, which isolates the divider as the
cause.  1200 Hz is the V.34 answer-modem Tone B, and the peer is transmitting
2400 Hz Tone A across the same window.  Whether the shipping firmware really
produces Tone B by stalling this divider is not proven by that alone — the
peer is the arbiter.

### Live: the peer runs line probing for the first time

`tools/eicon_adsp_sip.py --kernel-dispatch --info-action 0x34:1`
(`artifacts/eicon-live/run03.adsp.csv`).  Our transmit envelope loses its gap:
baseline `run02` is 3%/0%/0%/0%/0%/19% active over 6.3-6.8 s, `run03` is 100%
throughout.  State `0x37` then holds for 1.44 s instead of 113 ms and does not
fall back to `0x0010`.

slmodemd's own state machine goes far past anything previously seen:

```
  357.865  microstate TX_PHASE2_ANS=>TX_L1      <- line probing L1
  358.025  microstate TX_L1=>TX_L2              <- line probing L2
  358.245  microstate TX_L2=>TX_PHASE3_ANS
  358.325  microstate TX_PHASE3_ANS=>RX_PHASE2_ANS   (goes silent, waits for us)
  359.785  rxstate RX_DPSK=>RX_L1               <- detects an L1 from us
  360.225  rxstate RX_L1=>RX_DPSK
  361.425  txstate TONE_AB=>SILENCERETRAIN
  364.585  vpcm: Link Error
```

Every earlier call ended in the `DET_INFO -> TX_PHASE1_ANS` loop with the peer
repeating INFO0 until it gave up.  This is the first time it has transmitted
L1 and L2 or detected anything from us in the probing phase, which confirms
the causal claim: the `0x34..0x37` silence was what blocked V.34 Phase 2, and
putting energy there unblocks the peer.

It still fails ~3 s later.  The peer detects our "L1" at 359.785, which maps
to our 8.03 s — the moment our own `TrnProgress` resets to `0x0000` — so what
it detected was most likely the stalled carrier or the reset transient rather
than a real L1, and it retrains when no INFO1 follows.  Two things to settle
next, in order: what the firmware's genuine Tone B and L1/L2 transmit path is
(action 1 is a diagnostic that stalls a clock, not that path), and why our
side resets to `0x0000` at 8.02 s instead of proceeding from `0x37`.

## The real Tone B path, and the reset is the FFT overrun

### Tone B is state `0x00a2`, reached only through event 1

The transmit source is a per-state field, `DM(0x166b)`, dispatched at PM
`0x3b0c` (`I4 = DM(0x166b); CALL (I4)`).  It has no writer in the overlay: the
state-script executor PM `0x336a` loads it as part of the 25-word state record,
alongside `DM(0x1667)`, `DM(0x1668)` and the rest of `DM(0x1665..0x167x)`.

Replaying `run02` with the event injected at `0x37` shows the sources and what
each transmits:

| state | `DM(0x166b)` | `DM(0x16a6)` | TX |
|---|---|---|---|
| `0x34`, `0x36`, `0x37` | `0x3b29` | `0xbbc0` | silent |
| `0x00a0` | `0x3b30` | `0x4440` | — |
| `0x00a2` | `0x3b30` | `0x4440` | **1200 Hz, rms 1998, 848 ms** |
| `0x00e0` | `0x3b29` | `0x4440` | 2100 Hz |

PM `0x3b29` is `SI = 0` — the receive states feed the modulator zeros, so their
silence is deliberate and correct, not a fault.  PM `0x3b30` is the message
buffer readout (`DM(0x16a5)` against the end marker `DM(0x16b3)`).  State
`0x00a2` selects it and puts out a clean 1200 Hz carrier for 848 ms: that is
the genuine V.34 answer-modem Tone B, produced by the state's own script with
no clock manipulation, and it is entered from `0x37` only through condition PM
`0x2476` (event 1) to candidate `DM(0x1695) = 0x1736`.

So `--info-action 0x34:1` produced 1200 Hz by a completely different mechanism
— stalling the bit-clock divider so the carrier leaks — and is not the real
path.  The real path needs event 1.

### The 8.02 s reset is the Courier's FFT overrun

Watchpoints put the corrupting stores at PM `0x3793` and `0x3795`: the
indirect butterfly this document identified in the first `0x37` investigation.
The analysis-result pointer `DM(0x15f3)` advances two words per analysis every
~26.7 ms from `0x0ddd`, and its buffer ends at `0x0df0`:

```
  run02 (baseline)     0x0ddd -> 0x0de9, still inside the buffer
  run03 (action 1)     0x0ddd -> 0x0df0 at ~6.62 s, then straight through:
                       0x0df1 ... 0x0e4b ... 0x0e4d at 7.8489 s
```

`DM(0x0e4c)` holds the sequence's second `PM 0x373a` reset action; it flips
from `373a` to `0000` at exactly 7.8489 s.  2.6 ms later the transform runs
without that reset, the butterfly escapes its `0x1110` work buffer, and PM
`0x3793`/`0x3795` overwrite `DM(0x1652)`, `DM(0x166b)` and `DM(0x1679)` — the
sequencer's own working set.  The garbage next-state and condition addresses
then walk `TrnProgress` through nonsense in a few hundred microseconds and
land on `0x0000`.  That is the "reset" at 8.02 s.

This corrects the earlier claim that slmodemd calls show no FFT corruption.
They show none only because they leave `0x37` after 113 ms and the pointer
never reaches `0x0df0`.  The corruption is not peer-specific — it is
dwell-time specific, and it bites about 1.2 s into the receive window.  Any
fix that legitimately keeps us in `0x34..0x37` long enough to do the real work
will hit it.

### One bug, two symptoms

Both open threads reduce to the same missing event.  The classifier PM `0x2461`
publishes `DM(0x198e)` and is also what completes the analysis; because it
never runs, (a) state `0x37` never advances to Tone B at `0x00a2`, and (b) the
analysis never stops appending, so the result buffer overruns into the
sequencer's action list.  Raising event 1 needs framer B — the fixed `0x0080`
length instance — to decode the 8-bit control-channel message, and framer B is
parked disabled at `0x25f3` by PM `0x3f4c` with only the unreferenced PM
`0x2602` installing it.

Next: find what dispatches the PM `0x2ee6..0x2eee` action table through the
script pointer `DM(0x1667)` in a real call.  PM `0x2148` is the interpreter and
PM `0x2169..0x2175` walk `DM(0x1667)` within 8-entry blocks based at `0x2be0`,
`0x2be8` and `0x2bf0`, so recovering who seeds that pointer for the INFO page
is the concrete remaining step — not another injection.

## The state-record format, and what actually seeds `DM(0x1667)`

### How a state record is applied

PM `0x336a` (installed as `DM(0x169f)` by PM `0x32dd`/`0x34c9`; PM `0x3376` is a
packed alternative) decodes the state script as `(offset, value-lo, value-hi)`
triples, writing each value to `DM(0x1642 + offset)` and stopping when the
offset equals `MR1 = 0x19`.  So a record can only reach `DM(0x1642..0x165a)` —
the raw fields, plus the candidate/condition indices at `DM(0x1653..0x165b)`
that PM `0x3329..0x3332` translate through the tables at `0x133e`/`0x131e` into
`DM(0x1692..0x1695)` and `DM(0x1696..0x169a)`.

PM `0x3435` then diffs each raw field against a shadow copy in
`DM(0x1688..0x1690)` and calls a per-field handler on change.  Two handler
shapes:

- PM `0x3480`, a **bitmask dispatcher**: for each set bit of the field, fetch
  an action address from a PM table and `CALL` it.  Used by PM `0x345e`
  (`0x2e9a`, 11 entries), `0x3463` (`0x2ea5`, 16), `0x3468` (`0x2eb5`, 13),
  `0x346d` (`0x2ec2`, 16), `0x3472` (`0x2ed2`, 16), `0x3477` (`0x2ee6`, 9) and
  `0x347c` (`0x2ee2`, 4).
- PM `0x34ae`, a **2-bit-slot pointer loader**: each slot indexes a PM
  sub-table and the fetched pointer is stored to consecutive DM words.  PM
  `0x349e` fills `DM(0x166a..0x166c)` (the transmit source `DM(0x166b)` among
  them) from the bases at PM `0x2bd5..0x2bd7`; PM `0x34a4` fills
  `DM(0x166e..0x166f)`; PM `0x34a9` fills `DM(0x1681..0x1683)`.

### `DM(0x1667)` is seeded, and it is not the problem

PM `0x2169`, `0x216b` and `0x216d` load `AY1` with `0x2be8`, `0x2be0` and
`0x2bf0` and fall into PM `0x216e`, which computes
`((DM(0x1667) - 0x2be0) & 7) + AY1` — switch script block, keep the phase
within it.  Those three are entries 10, 9 and 8 of the PM `0x2ea5` table, so
they are selected by bits in the state-record field **`DM(0x1644)`**.  The INFO
page initializer's block clear at PM `0x3f6a..0x3f6f` zeroes `DM(0x1667)`
first; PM `0x2149`/`0x214b` then walk it and PM `0x2165`/`0x2168` rewind it.

Watching it live confirms all of that works.  `DM(0x1644) = 0x0401` throughout
INFO selects block `0x2be8`, and `DM(0x1667)` walks `0x2be8..0x2bec` normally
across every state.  **The script pointer was never unseeded.**

### The action table is dispatched from a different field, and is downstream

PM `0x2ee6..0x2eee` is not reached through `DM(0x1667)` at all.  It is handler
PM `0x3477`'s table, dispatched as a bitmask from the state-record field
**`DM(0x164c)`** — bit N runs action N.

`DM(0x164c)` is `0x0000` for every INFO state `0x20..0x37`, and that is
correct, not a gap.  On the event-1 branch it is set by the states that follow:

```
  6.625  state 0x00a0   DM(0x164c)=0x0001   -> action 0, PM 0x2410, build the
                                               message table at DM(0x1986)
  6.626  state 0x00a2   DM(0x164c)=0x0080   -> action 7, PM 0x2434, transmit
                                               message 1, TX source 0x3b30
```

So the previous section's chicken-and-egg framing was wrong.  Installing
framer B and transmitting the 8-bit message are the **response** to event 1,
not its prerequisite: on receiving message 1 at `0x37` the page moves to
`0x00a0`, builds the message table, and at `0x00a2` sends its own message 1
back on the 1200 Hz Tone B carrier.  Nothing about the action block is
missing — the states that dispatch it are simply downstream of the transition
we never take.

That leaves exactly one thing unexplained, and it is now sharply posed: PM
`0x2470` is the only writer of a non-zero `DM(0x198e)` and it runs only from
the classifier PM `0x2461`, which only the 8-bit framer can reach.  So a real
card receiving message 1 at state `0x37` must be running framer B at that
point — yet PM `0x3f4c` parks it at `0x25f3` and the only installer, action 1,
is dispatched by a state we only reach afterwards.  Either another page or an
earlier INFO state sets `DM(0x164c)` bit 1 before `0x37` in a real call, or
framer B is installed by a path outside this overlay.  Dump `DM(0x164c)` and
`DM(0x19cf)` across every state of a call that gets further than ours — that
comparison, not more static reading, is what will settle it.

## INFOH is not it; the INFO page has two state chains and we take the wrong one

### INFOH checked and ruled out

INFOH (download `0x026e`) loads cleanly — "INFOH.F34 Overlay Version 1.00 Build
117-926" — and the bootpage table `DM(0x31d5)` maps **bootpage 10** to it
(entries are negative: `0xfd92` is `-0x026e`, `0xfda0` is `-0x0260` for page 7,
`0xfda1` is `-0x025f` for V.8).  So the firmware can request it.

It is not a variant of the INFO page, though.  Every region that matters
differs: the classifier/action block `0x2410..0x24ff`, the PM `0x2602` entry,
the framers `0x3510..0x35ff`, the PM action table `0x2ee0..0x2eef`, and the
record handlers `0x3435..0x34bf` all hash differently between the two
overlays, and INFOH brings its own DM image, so the INFO state records do not
exist in it.  `DM(0x164c)` reads `0x0006` after loading INFOH, but that is
overlay data sitting at that address, not a live state field.  INFOH is not
the missing piece.

### The missing piece is a state chain inside INFO

`tools/info_state_records.py` decodes every record and the state-vector table
at `DM(0x133e)`.  Across all 0x40 entries, exactly one record dispatches
action 1:

```
  index 03  state 0024 @07e5  DM(0x164c)=0002  ->  PM 0x2602 INSTALL FRAMER B
  index 2a  state 00a0 @1736  DM(0x164c)=0001  ->  PM 0x2410 build message table
  index 30  state 00a2 @175d  DM(0x164c)=0080  ->  PM 0x2434 transmit message 1
  index 32  state 00a7 @1796  DM(0x164c)=0101  ->  PM 0x2410, PM 0x243b (message 2)
```

Framer B is installed by the record for **state `0x0024`** at `DM 0x07e5`, which
belongs to the `0x07xx`/`0x08xx` chain (`07a0` state `0x20`, `07e5` state
`0x24`, `07f7` `0x26`, `080f` `0x28`, `082d` `0x2b`, `0836` `0x2c`, `084e`
`0x2e`, `08d5` `0x37` ...; records also simply run on into the next address,
so a chain of consecutive records is one state sequence).

Our calls never touch it.  PM `0x32dd` enters INFO with vector `0x0b87` and PM
`0x3317` with `0x0b69`, both in the `0x0bxx` chain, and the live trace walks
`0b18 -> 0b27 -> 0b36 -> 0b4b -> 0b60` before crossing into `0869 -> 087b ->
088d -> 0899 -> 08b1 -> 08e7`.  The `0x0bxx` record for state `0x24` leaves
`DM(0x164c)` at zero, so framer B is never installed — which is why the
`--init-info-detector-at-24` diagnostic worked: it was calling PM `0x2602` at
exactly the state whose real record would have done it, by luck.

### Correction: `DM(0x1651)` does reach `0x0080`

The record at `0x1736` (state `0x00a0`) sets `DM(0x1651) = 0x0080`, confirmed
live — it goes `0x0110 -> 0x0080` at the moment that state is entered.  The
earlier claim that `0x0080` was unreachable because PM `0x3f84` is the only
writer was wrong: it was a scan of direct-addressed writes only, and the
record loader writes `DM(0x1642 + offset)` through a DAG store.  So framer A
is itself reconfigured to the 8-bit message class at `0x00a0` and becomes the
event publisher there; framer B is not the only candidate after all.

### The remaining question

What selects the `0x07xx` chain over the `0x0bxx` one.  Both are complete INFO
sequences over the same states; only the `0x07xx` one arms the 8-bit control
channel at `0x24`.  Entry into it is via candidate index `0x02` or `0x14` in
the vector table, so the next step is to find which record's candidate field
carries those indices and what condition selects it — and whether the choice
keys off `DM(0x16b6)`/`DM(0x3f94)`, the INFO variant, since PM `0x3317` seeds
the `0x0bxx` entry while explicitly setting `DM(0x16b6) = 8`.

## What selects the `0x07xx` chain: `GEN_setup1` bit 7 and the reserved word `DM(0x3f8a)`

The chain entry is chosen by PM `0x34b5`, which is dispatched as bit 7 of the
PM `0x2eb5` action table:

```
  34b5: AR = 0x07a0 ; AX1 = 0x07a0
  34b7: AF = DM(0x3f8a) XOR 0x5678
  34ba: IF EQ JUMP 0x34c3                  ; magic present -> keep 0x07a0
  34bb: AF = DM(0x3ee0) AND 0x0040
  34be: IF EQ JUMP 0x34c1
  34bf:   AR = 0x0ae8 ; AX1 = 0x0ac4       ; override the entry vector
  34c1: DM(0x3f91) = DM(0x3ee7)
  34c3: CALL 0x34cb                        ; AF = DM(0x3f94) AND 0x0008
  34c4: IF NE JUMP 0x34c8
  34c5:   MR0 = AX1 ; I4 = 0x3376 ; JUMP 0x331e   ; packed record loader
  34c8:   MR0 = AR  ; I4 = 0x336a ; JUMP 0x331e   ; triple record loader
```

The field that dispatches it is `DM(0x167e)`, loaded verbatim from
`DM(0x3ee1)` at PM `0x3efe` — database offset `0x01`, **`GEN_setup1`**,
"operation mode parameters" in the ADDSP guide.  Our calls have
`GEN_setup1 = 0x0484` (bits 2, 7, 10), so bit 7 does fire PM `0x34b5`; the
`0x0bxx` alternative is bit 9 (PM `0x3315`) and is not set.

So the entry vector is decided by two words the host owns:

| `DM(0x3f8a)` | `DM(0x3ee0) & 0x0040` | `DM(0x3f94) & 8` | entry vector |
|---|---|---|---|
| `0x5678` | — | set | `0x07a0` |
| `0x5678` | — | clear | `0x07a0` |
| other | set | set | `0x0ae8` |
| other | set | clear | `0x0ac4` |

`DM(0x3f8a)` is database offset `0xaa`, inside the guide's elided
`A8..B7` reserved range.  The DSP only ever reads-and-clears it (PM `0x33bd`
is a condition handler testing for the same `0x5678`; PM `0x33be` clears it),
so it is a host-written token.  `tools/dial_kernel_dispatch.py` writes
`DM(0x3ee0) = 0x00c4` and never writes `DM(0x3f8a)`, so the override fires and
we get `0x0ae8` — the chain whose state `0x24` record leaves `DM(0x164c)` at
zero.

### Live with the token set

`--db-word 0x3f8a:0x5678` (`artifacts/eicon-live/run04`).  The page takes the
other chain — `0x24 -> 0x2c -> 0x2e -> 0x30 -> 0x32 -> 0x36 -> 0x37`, skipping
`0x22`, `0x28` and `0x34` — and **the firmware installs framer B itself**:
`DM(0x19cf)` cycles `0x25ab -> 0x25c7 -> 0x25e2` for the whole call, the first
time that has happened without the `--init-info-detector-at-24` diagnostic.
`0x37` then exits to `0x0026` rather than `0x0010`.

It is not a fix.  Framer B never validates a frame, `DM(0x198e)` stays `0`,
and event 1 still never fires.  The peer does worse than `run03`: one
`rxinfo0`, no L1/L2 probing, `vpcm: Link Error` after 14 s.  The new chain
skips state `0x34`, so the transmit window that unblocked probing in `run03`
is gone too.

Treat `0x5678` as an undocumented mode token, not a recovered setting: it is a
magic value in a reserved word, it changes the state graph, and nothing yet
shows a real MIPS supervisor writes it.  What it does establish is that the
`0x07xx` chain is reachable by configuration alone, and that framer B running
is necessary but not sufficient — this peer never sends the 8-bit message it
waits for.  The next question is what that message is on the wire, and whether
a V.90 call is supposed to see one at all, since `DM(0x3f94) = 9` selects
V.90 and the 8-bit class may belong to the `0x0006` (non-V.90) mode.

## No: the 8-bit class belongs to the V.90 decoding, not mode `0x0006`

The two record loaders read the *same* DM bytes differently.  PM `0x336a`
takes the offset from `w1 & 0xff` and the value from the low bytes of `w2`/`w3`;
PM `0x3376` takes the offset from `w1 >> 8` and the value from the high bytes.
The record blocks are dual-encoded — both decodings yield a plausible state
sequence over the same addresses — and PM `0x34c4` picks the loader from
`DM(0x3f94) & 0x0008`: set (mode `0x0009`, V.90) uses PM `0x336a`, clear (mode
`0x0006`) uses PM `0x3376`.

Decoding the token chain at `0x07a0` both ways:

```
  triple  (mode 9, V.90)   0020  0022  0024 actions=0002  0026  0028  002a  002b  002c
  packed  (mode 6)         0020  0022  0024 actions=0000  0026  0028  002a  002c  002e
```

**The framer-B install is present only in the triple decoding** — the V.90
one.  Under mode `0x0006` the same bytes at state `0x24` decode to
`actions = 0000`.  And the non-token entries never arm it under either
decoding:

```
  0x0ac4 packed (mode 6's own loader)   0020 0022 0024 0026 0028 002a 002c 002d  -- all actions=0000
  0x0ae8 triple (mode 9)                0020 0022 0024 0026 0028 002a 002b       -- all actions=0000
```

So the answer is no.  The 8-bit control channel is a **V.90** feature: it is
armed by the mode-9 decoding of the token chain and by nothing else.  That is
consistent with `run04`, where mode `0x0009` plus `DM(0x3f8a) = 0x5678` made
the firmware install framer B on its own.  Mode `0x0006`'s message class is
the 30-bit one PM `0x3f7f` selects (`DM(0x1651) = 0x01e0`), not 8-bit.

This also settles the previous section's closing speculation, which guessed the
opposite: the `0x0bxx` chain we ran originally is not "the right V.90 path" —
it is the V.90 path *without* the token, and the token chain is the V.90 path
with the 8-bit control channel armed.

What remains is why framer B, once genuinely installed and running for a whole
call (`run04`: `DM(0x19cf)` cycling `0x25ab/0x25c7/0x25e2` throughout), never
validates a frame.  Either this peer does not transmit the 8-bit message at
all, or it is transmitted in a form the magnitude-thresholded bit decision at
PM `0x3515` cannot recover.  Distinguishing those two needs the message's
on-wire form, and the transmitter is available to answer it: PM `0x2446` builds
the frame and `--info-action` can fire actions 3..8 to emit each message code.
Capturing our own transmission of message 1 and measuring it gives the exact
waveform framer B is waiting to receive, offline and without a peer.

## Stepping back: our own stack already documents this exact failure

The simple thing was in this repository the whole time.  `modem_engine.c:4856`,
written from live work against this same SmartLink peer:

> the peer gave up on Phase 3/4 and initiated a retrain: 70 ms silence then
> Tone A, waiting for our Tone B (V.90 §9.5.2.1) ... nothing ever answers Tone
> A, and the SmartLink peer declares a link error after ~3.1 s of unanswered
> Tone A (observed live 2026-07-22)

That is precisely the emulated card's symptom: slmodemd transmits Tone A,
nothing answers it, and it link-errors.  `run03` is the same statement from
the other side — putting 1200 Hz into the `0x34..0x37` window made the peer
advance to `TX_L1`/`TX_L2` immediately.  So the failure is simply **we never
answer Tone A with Tone B**, and the 8-bit control-channel work of the last
several sections was chasing the machinery around that, not the fault itself.

The earlier session's `_inject_l1l2_completion` comment — "the emulated INFO
probing classifier never publishes event 1, although the waveform is present"
— was right about the symptom.  Treating its Tone A reading as unfounded and
going after the message classifier instead was my error; both descriptions
converge on the same missing detection.

### Ruled out cheaply this turn

- **Host event acknowledgement.**  `tools/dial_kernel_dispatch.py:270-286`
  does service `DM(0x3fc1)` and the change bits; `change_flags` and `wstatus`
  stay `0x0000` across the call and `dbs_trnprogress` tracks every transition,
  so the host side of the interface is being consumed.
- **Receive level.**  Replaying `run02` with the µ-law stream scaled by 0.5x,
  2x and 4x produces a bit-identical state path and `DM(0x198e)` never leaves
  its stale `0x06a6` at any gain.  Not a threshold or scaling problem.
- **Skipping Tone A acquisition.**  States `0x2a`/`0x2b` last about two samples
  by construction, not because this peer causes an early exit: the record at
  `0x0b60` sets `DM(0x1650) = 1`, so the pre-condition PM `0x3391` fires on the
  second sample, and its only live condition targets state `0x2e` anyway.  The
  earlier note calling `0x2a`/`0x2b` "acquisition of the peer's Tone A" does
  not survive reading the record.
- **Any other writer of the event word.**  A watchpoint on `DM(0x198e)` across
  a whole call logs no write at all.  PM `0x2470` really is the only writer and
  it never runs, so the Tone A path does not reach it either.

### Where that leaves it

Two readings remain and they are worth separating before more work.  Either
this firmware answers Tone A from a detector we have not found — in which case
it is not `DM(0x198e)`, since nothing writes that word — or the INFO page
genuinely expects an 8-bit control-channel message here that slmodemd never
sends, and this build's Phase 2 is not the plain V.34 exchange the peer
implements.

The `run03` result is the practical lever either way: energy in the
`0x34..0x37` window is what the peer needs, and it advances immediately when
it gets it.  A response driven by an actual Tone A detector — rather than the
bit-clock stall `run03` used — is the next thing worth building, and
`tools/eicon_info_replay.py` can prototype it offline against the captured
peer audio before spending live calls.

## The tone detector: found, armed, counting — and two profiles are dead code

The INFO page's tone detector is FFT based.  PM `0x372d` fills the working
buffers, PM `0x36ed` runs the transform (256-point: span/count/stride walk
`0x80/2/4 -> 0x20/8/0x10 -> 8/0x20/0x40 -> 1/0x100/0x200`), and results are
appended to the 20-word buffer at `DM(0x0ddd)` through the pointer
`DM(0x15f3)`.  A detector is armed by writing a bin index to `DM(0x16f2)` and
a threshold to `DM(0x16f3)`, with `DM(0x16f0)`, `DM(0x16f1)` and `DM(0x06e7)`
as further parameters.

Four arming profiles exist in the image:

| routine | bin `DM(0x16f2)` | threshold `DM(0x16f3)` | reachable |
|---|---|---|---|
| PM `0x365c` | `0x0003` | `0x2aaa` | yes — armed on entering state `0x36` |
| PM `0x36b9` | `0x0005` | `0x1999` | yes |
| PM `0x3716` | `0x0008` | `0x1000` | **no reference anywhere** |
| PM `0x3722` | `0x0012` | `0x071c` | **no reference anywhere** |

PM `0x3716` and PM `0x3722` have no branch, no PM table slot and no DM vector
pointing at them — the same signature PM `0x2602` had before its action-table
slot was found.  PM `0x3722`'s bin 18 with the lowest threshold of the four is
the profile one would expect for acquiring a distant modem's Tone A.

The detection counter is `DM(0x06e6)`, incremented by PM `0x3702`/`0x370c`,
which are bits 2 and 1 of the PM `0x2ed2` table — handler PM `0x3472`, state
record field `DM(0x164b)`.  State `0x37` sets `DM(0x164b) = 0x1002`, so bit 1
dispatches PM `0x370c`, and live `DM(0x06e6)` counts `0,1,2,3,4` across state
`0x37`, one per ~26.7 ms analysis, before we leave at 4.  The condition
handlers PM `0x33ae..0x33bc` compare `DM(0x06e6)` against 3, 5, 6, `0x0e` and
`0x18`, so the state graph is waiting on a detection count.

So the detector is armed, the transform runs, and the count advances — what
never happens is a result that turns into `DM(0x198e)`.

Arming the dead profiles by hand does not help, and the reason is instructive:
calling PM `0x3722` or PM `0x3716` at state `0x34` or `0x36` leaves the state
path and `DM(0x198e)` bit-identical, because state `0x36`'s own record runs PM
`0x365c` on entry and immediately re-arms bin 3 over the top.  Any real use of
those profiles has to come from a state record that selects them, not from an
injection.

That points away from the MIPS/driver hypothesis rather than toward it: these
arming routines are dispatched from PM tables driven by state-record fields,
not from database words, so a different host-side database setup cannot reach
them directly.  What can is a state chain we do not run — the same conclusion
the `DM(0x164c)` work reached from the other side.  The concrete next step is
to search the record blocks for a field value whose PM `0x2ed2`/`0x2ec2` bits
resolve to `0x3716`/`0x3722`, which would name the state that arms the Tone A
profile; `tools/info_state_records.py` already decodes the records, so this is
a table lookup rather than more disassembly.

## The Tone A detector is armed at state `0x0c41`, which we never reach

Correcting the previous section: PM `0x3716` and PM `0x3722` are *not*
unreferenced.  The earlier scan looked for branches, PM table slots and DM
vectors, and missed immediate-store opcodes.  Two routines install them into
the analysis action lists:

```
  36ae: I4 = 0x0e53 ; DM(I4,M5) = 0x3716 ; DM(I4,M5) = 0x3700
  36e3: I4 = 0x0e4c ; DM(I4,M5) = 0x3722 ; DM(I4,M5) = 0x3700
```

`DM(0x0e4c)` and `DM(0x0e53)` are the analysis action slots — lists of PM
addresses terminated by `0x3700` — that the FFT sequencer executes.  (This is
also why the result-buffer overrun documented earlier is so destructive: the
20-word buffer at `DM(0x0ddd..0x0df0)` runs directly into the detector
programs.)

The installers are all bits of the PM `0x2ed2` table, dispatched from the
state-record field `DM(0x164b)`:

| bit | routine | profile |
|---|---|---|
| 4 | `0x36a7` | multi-action sequence |
| 6 | `0x36ae` | bin 8, threshold `0x1000` |
| 7 | `0x365c` | bin 3, threshold `0x2aaa` |
| 8 | `0x36dd` | — |
| 9 | `0x36e7` | — |
| **10** | **`0x36e3`** | **bin 18, threshold `0x071c`** |
| 11 | `0x36b9` | bin 5, threshold `0x1999` |

Scanning all 125 reachable records for `DM(0x164b)` gives the answer:

```
  @08b1 state=0a36  DM(0x164b)=0080  -> 365c (bin 3)      <- what our calls arm
  @08ff state=0b37  DM(0x164b)=0010  -> 36a7
  @0914 state=0c37  DM(0x164b)=0040  -> 36ae (bin 8)
  @09b3 state=0040  DM(0x164b)=0800  -> 36b9 (bin 5)
  @0a07 state=0b41  DM(0x164b)=0100  -> 36dd
  @0a28 state=0c41  DM(0x164b)=0400  -> 36e3 (bin 18)     <- the Tone A profile
  @0a3a state=0d41  DM(0x164b)=0200  -> 36e7
  @1b32/1b3e/1b59   DM(0x164b)=ff1f  -> arms every detector
```

The states form two parallel families: `0x37` with sub-states `0a37`, `0b37`,
`0c37`, `0d37`, and `0x41` with `0a41`, `0b41`, `0c41`, `0d41`.  Our calls
reach `0x37`/`0x0a37` and stop — never `0x0b37`, never `0x0c37`, and never the
`0x40`/`0x41`/`0x42` family at all.

Live exec watchpoints confirm it: PM `0x365c` runs on entering state `0x36`
and PM `0x36a7` at `0x37`, so `DM(0x0e4c)` only ever holds `0x376e`/`0x373a`
and `DM(0x0e53)` `0x3700`/`0x325c`.  **PM `0x36e3` never executes**, so
`0x3722` is never installed and the bin-18 detector never runs.  PM `0x36ae`
executes exactly once, early, during DIAL — never during INFO.

This also explains why arming the profiles by hand did nothing: state `0x36`'s
record re-runs PM `0x365c` on entry and overwrites the slot.

So the answer to "where is the Tone A detector" is: it exists, it is
configured with the lowest threshold of the four profiles as expected for
acquiring a distant tone, and it is armed by the record for state `0x0c41` —
a state in a family our INFO sequence never enters.

## The `0x41` family is the no-message path, not downstream of Tone B

Decoding complete records (the `0x19` terminator consumes a full three-word
triple) resolves the `0x41` decision at `0x09ec`:

```
  @09d7  state 0041
  @09ec  state 0a41
           default successor -> @0a07 state 0b41, pretest[12] (count == 5)
           slot 0: next[23] -> @0a28 state 0c41, test[13] -> PM 33b6 (count == 6)
           slot 1: next[24] -> @0a3a state 0d41, test[14] -> PM 33b8 (count == 24)
           slot 2: next[25] -> @09d7 state 0041, test[01] -> PM 33ca (always)
  @0a07  state 0b41
  @0a28  state 0c41                         <- arms bin 18 / Tone A
  @0a3a  state 0d41
  @0a4f  state 0042
```

Thus `0x0a41` takes its sequential `0x0b41` successor at detector-count 5,
selects `0x0c41` at 6, selects `0x0d41` at 24, and otherwise loops through
root state `0x41`.  At 6, `0x0c41`'s `DM(0x164b) = 0x0400` dispatches PM
`0x36e3` and installs the bin-18 Tone A profile.  This is a real, internally
connected detector path; the apparent absence of a vector pointing at
`0x0a41` is because the sequencer stores the address immediately after each
record as its default successor.  The `0x19` terminator's two-byte payload is
also live: it becomes `DM(0x165b)`/the default-successor pre-condition, which
is why the updated decoder prints it as `pretest` rather than discarding it.

The preceding default-successor records are:

```
  @098f  state 003c, event-1 candidate -> @1736 state 00a0
  @09a7  state 003e
  @09b3  state 0040
  @09c8  state 0a40
  @09d7  state 0041
```

State `0x3c` is the fork.  Its slot 3 is condition PM `0x2476`
(`DM(0x198e) == 1`) to vector index `0x2f`, record `0x1736`; that message
branch proceeds immediately to state `0x00a2` and transmits Tone B.  If the
message event does not fire, the inherited countdown pre-condition takes the
sequential path through `0x3e`/`0x40` into the `0x41` detector family.  No
record in the `0x00a0`/`0x00a2` message chain has a candidate back to vector
indices `0x23..0x25`, and the only candidate for root `0x41` is its own
`0x0a41` loop.

So the Tone A detector is **not after the Tone B response**.  It is on the
alternative no-message/default-successor chain.  Nor is it a direct candidate
of the `0x37` record: several intermediate receive states precede it.  In the
live call, framer A wins at `0x37` and sends the page to state `0x10` before
that chain can be reached, while the missing event-1 exit is what would send
it to the genuine Tone B path.  The `0x37` exit choice therefore remains the
single blocking question; the `0x41` lookup does not reveal a missing
post-`0x00a2` transition.

`tools/info_state_records.py --records 0x09d7:0x0a5b` now walks consecutive
records on their true boundaries and resolves each candidate and condition
index, so the result is reproducible without hand-splitting the table.

## The real V90D load path is INFO1a mode 6 -> bootpage 14

The desired destination is not selected by a magic TrnProgress value or by a
MIPS policy decision.  The DSP carries the V.90/V.34 decision all the way to
TIKRNL's ordinary overlay loader.

INFO initializer PM `0x3304..0x3310` computes the page to use when INFO
completes:

```
  AX0 = 0x000e
  AR = DM(0x3fbb) & 0x0070             ; BaudInfo, INFO1a bits 37:39
  if AR == 0x0060: DM(0x16b6) = AX0    ; value 6 -> page 14 / V90D
  else:
      AX0 = 0x000d
      if !(DM(0x3f94) & 2): AX0 = 8
      DM(0x16b6) = AX0                  ; non-6 result -> another data pump
```

That test is the firmware implementation of ITU-T V.90 §9.2.1.1.8: after
sending INFO1d and receiving INFO1a, the digital modem proceeds to V.90 Phase
3 only when INFO1a bits 37:39 encode integer 6; values 0 through 5 continue as
a V.34 call modem.

INFO's completion routine PM `0x2176..0x217f` then performs the actual DSP-side
page selection:

```
  DM(0x3fc1) |= 0x0100                  ; publish page-change status
  DM(0x3fb0) = DM(0x16b6)               ; bootpage_nr = 14 for V.90
```

From there the already-recovered normal loader path applies unchanged:

1. TIKRNL PM `0x0686..0x0694` indexes its table at `DM(0x31d5)` with
   `DM(0x3fb0) = 14`.
2. Table entry 14 is positive `0x026a`, the V.90 DPCM overlay, so TIKRNL
   publishes `DM(0x31aa) = 0x026a`, registers its post-download continuation,
   and yields through kernel service PM `0x000a`.
3. The host downloads overlay `0x026a`, sets `WSTATUS.BOOTFINISHED`, and
   dispatches the registered completion entry.
4. TIKRNL resumes into the V90D program for V.90 Phase 3.

This corrects the harness's synthetic `_load_v90d()` path, which watched for
TrnProgress `0x003b`, wrote bootpage 14 itself, and directly downloaded the
overlay.  There is no `0x003b` state in the decoded INFO record families, and
that shortcut bypassed the protocol decision we need to test.  It has been
removed: `KernelDispatch.service()` already serves the genuine page-14
request when INFO publishes one.

Therefore the path to pursue is:

```
INFO0 exchange -> Tone B/reversal -> L1/L2 -> INFO1d -> valid INFO1a
  -> INFO1a bits 37:39 == 6
  -> DM(0x3fbb) rate field 0x60
  -> DM(0x16b6) = DM(0x3fb0) = 14
  -> TIKRNL requests 0x026a V90D
```

Our call currently fails back at the `0x37` INFO0/Tone-B decision, long before
INFO1a can set that rate field.  Reaching the `0x41` Tone A detector is not the
goal by itself; the goal is to take the complete V.90 Phase 2 chain through a
CRC-valid INFO1a whose mode field is 6, then let the DSP request V90D naturally.

## Host-bit audit: V90D is enabled; the remaining selector is received INFO1a

Auditing `program_initial_setup()` and `program_v8_call()` against ADDSP V.90
User's Guide v5.3 §5.3.1 and §5.4.1 Tables 12, 13 and 15 gives:

| write DB | value | relevant meaning |
|---|---:|---|
| `GEN_SETUP0 +00` | `00c4` | extended training, PSTN, normal equalizer |
| `GEN_SETUP1 +01` | `0484` | answer, two-wire, internal clock, NORM operation |
| `GEN_SETUP2 +02` | `0030` | signal-quality step-up and fallback |
| `V8_SETUP +04` | `6000` | **V90_DPCM** and **digital network** |
| `INFO0_SETUP +07` | `f0fd` | V.34 Phase-2 capabilities and 3429 support |
| `delaycorrection +24` | `000c` | Eicon build's supplementary-buffer calibration |
| `NORM_H/L +28/+29` | `0001/8100` | V.8, V.90, and V.34 fallback |
| `SPEED_SEL_H/L +2a/+2b` | `001f/ff00` | V.34 fallback rates through 33600 |
| `V34SLOT +78` | channel | assigned TDM slot, now explicitly programmed |
| `SPEED_SEL_V90_H/L +79/+7a` | `003f/ffff` | every defined V.90 rate, 28000–56000 |
| `INFO0D_SETUP +7b` | `03b7` | lookahead 3, 3429 upstream, µ-law, codec measurement, −12 dBm0 |
| maxima `+7c..+7f` | `000e/0015/000e/0015` | V.34 33600 and V.90 56000, TX and RX |

These are all the documented host-controlled capability fields relevant to a
digital-side V.90 call.  One omission was found: the guide says `V34SLOT`
must be initialized before modem operation on a TDM interface.  It previously
worked only by relying on the reset default of slot zero.  The harness now
writes the selected channel explicitly; this matters for non-zero PRI slots
but does not explain the channel-zero captures.

Two write-database words change after DIAL consumes them, but neither removes
V.90 capability:

- DIAL's line handler changes `GEN_SETUP2` from `0x0030` to `0x0070`, enabling
  its tone detector while retaining the requested `0x30` step-up/fallback
  bits.
- PM `0x0bb7..0x0bc8` masks the ordinary V.34 `SPEED_SEL_L` fallback word from
  `0xff00` to `0x0100` for the selected recommendation.  The separate V.90
  masks at `+0x79/+0x7a` remain `0x003f/0xffff`.

`LiveKernelModem.configure_modem()` now validates the surviving V90D fields
after DIAL activation and fails immediately if a later change clears one.
The fact that live calls reach INFO with `DM(0x3f94) = 0x0009` independently
confirms that V.8 selected the V.90 INFO variant, rather than rejecting our
host capability setup.

So: **yes, the documented host bits needed to negotiate V90D are set**.  They
cannot directly force page 14.  The final selector is deliberately peer-owned
by V.90 §9.2.1.1.8: the firmware must finish INFO1 and decode INFO1a bits
37:39 as 6.  Our present failure occurs before that field exists, at the
`0x37` INFO0/Tone-B branch, and is not evidence of another missing V90D enable
bit.

### Other PDF requirements, separated from the current blocker

A wider pass over the guide found four integration requirements that are not
V90D-enable bits:

1. **`delaycorrection` (`+0x24`) must be platform calibrated.**  The guide
   says to measure `roundtripcntr` in a calibrated back-to-back session and
   write the compensation when INFO is initialized.  We had not written the
   word, so it happened to retain build 117-926's DM-image value `0x000c`.
   The harness now writes and validates `0x000c` explicitly, preserving
   existing behaviour while making the assumption visible.  Replaying
   `run02` with `0x0000` shifts the state-`0x30` exit by 5 ms but leaves the
   complete path and the `0x37 -> 0x10` failure unchanged.  A real calibrated
   loop is still required before claiming exact ranging timing.
2. **SPORT0 setup words `+0x70..+0x74` must be initialized before the startup
   page.**  They contain the physical SPORT0 control and 32-channel RX/TX
   enable masks.  The emulator does not execute a physical SPORT: its
   `sport0_tdm_frame` callback presents the selected slot directly, so writing
   guessed hardware-register images would make the model less faithful, not
   more.  These words are a hardware-integration obligation, not an omitted
   INFO capability bit.  `V34SLOT +0x78`, which the DSP does consume for the
   logical modem channel, is now set.
3. **`DATACONFIG +0x77` defaults to SCC1 data.**  Selecting value 1 in its
   two-bit data-pump field (word `0x4000`) chooses the documented IDMA TX/RX
   buffers.  This matters after V90D reaches DATASTATE and the PTY bridge is
   attached; it does not participate in V.8, INFO, or the page-14 decision.
   The experimental endpoint currently has no completed V90D data-interface
   cycle, so changing it during training would conceal that later work.
4. **Runtime boot service must finish within 75 ms.**  Guide §2.3 imposes the
   general page-load deadline (70 ms specifically for INFO -> V.34).  The
   emulator downloads and acknowledges a requested overlay synchronously in
   the same 8 kHz frame, so it meets the logical requirement without dropping
   or manufacturing bearer samples.

The guide also requires separate initial and training `change_wdb`
communication cycles and a `BOOTFINISHED` acknowledgement after each runtime
page load.  Both are already implemented.  The reconstructed 2400-Hz event
publication cycle is host-supervisor plumbing rather than another capability
setting.
