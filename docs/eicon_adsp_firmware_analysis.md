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
