# PRI vs BRI firmware: where the DSP count actually matters

## Summary

The `.pm` (PRI, 30-DSP) image was thought to be blocked on a 30-way DSP
*presence* detect, making the `.2qm` (BRI, 2-DSP) image look like the way
forward. Re-reading the firmware shows that was only half right, and the
first blocker was something else entirely:

1. The card init's per-DSP **constructor** does no probing at all — it
   builds 30 DSP objects unconditionally.
2. The first thing that actually failed was a missing **DSP code image** in
   card RAM, which has nothing to do with DSP count. Fixed; see below.
3. The 30-way handshake is real, but it lives in the **validator**
   (`0x80082130`) that runs after the constructors, and it is a
   command/response boot handshake, not the host driver's IDMA signature
   probe.

Switching to `.2qm` still costs a full entry-point re-derivation (see the
code-model note at the end) and would not have avoided (2) at all.

## What the init actually does

`0x80082f90` (the image entry):

```
80082fb0: lui   $s1, 0xa001
80082fb4: lw    $s1, 0x106c($s1)   # protocol image + 0x6c
...
8008300c: lhu   $s2, ($s1)         # download count
80083020: addiu $s1, $s1, 4        # -> t_dsp_portable_desc[], stride 0x30
```

`+0x6c` is `OFFS_DSP_CODE_BASE_ADDR` (`kernel/mi_pc.h:205`) and the 0x30
stride is `sizeof(t_dsp_portable_desc)` (`kernel/dsp_defs.h:190`, 10 words +
7 dwords). So `$s2` is the **DSP download-table count**, not a presence
word. The host driver stages that table in `pri_telindus_load`
(`kernel/s_pri.c`): `sharedRam[0] = download_count`, table immediately
after.

Then `0x80081de0` (card init, called at `0x800830d0`):

- builds a fixed table of 30 DSP register bases (`0xbc000800`, `+8` …
  `0xbc001070`, plus the two on-board at `0xbc000008/0x20`);
- loops `slti $v0, $s0, 0x1e` (`0x80082090`) calling the per-DSP
  constructor `0x80085394` 30 times — unconditionally, no bitmask, no bail;
- calls the validator `0x80082130` at `0x800820bc`;
- returns nonzero on failure, and the entry then **hangs** at the
  `0x800830ec` self-loop.

`0x80085394` makes exactly two calls (a trace printf and `0x8002a800`) and
never touches the host port, so there is no presence probe in construction.
It only records the code-table count/pointer at object `+0/+4` and searches
the table for download id `0x190` (FAX05 — not in the PRI-30M set).

The host driver's `dsp_check_presence` with the `0x5a5a`/`0xa5a5` signature
(`kernel/io.c:1229`, `s_pri.c:238`) is **host-side only**. Its result
(`a->dsp_mask`) is never written to the card — it feeds a `DBG_FTL` when
fewer than 2 respond, and `/proc` reporting.

## Fixed: DSP code image staging

`tools/eicon_dsp_stage.py` builds the image the firmware expects and
`tools/eicon_mips_shim.py --mainloop` now stages it before the entry runs.
Layout mirrors `pri_telindus_load` + `dsp_read_file`
(`divactrl/load/common/dsp_file.c`):

```
+0x0000  dword  download_count
+0x0004  t_dsp_portable_desc[128]          (128 * 0x30)
+0x1804  section data, dword-aligned, in dsp_read_file order
```

Each descriptor's seven pointer fields hold the card address of that
section, or 0 when empty (`dsp_card_load_portable`). Which downloads are
required is decided by the combifile itself: its directory maps a
`card_type_number` to a file set, and each download carries a usage-mask bit.

Base address is the protocol image's own `OFFS_PROTOCOL_END_ADDR`
(`0x80338700` for `te_dmlt.pm`), dword-aligned — the same value the image
entry uses as its initial `sp`, so the stack grows down away from it.

For card type 23 (`CARDTYPE_DIVASRV_P_30M_PCI` -> file set 5) that is 64
downloads / 848,580 bytes, including everything the modem path needs:

```
0x0258 TIKRNL81.F34   0x0261 V.34 Overlay   0x026a V.90 DPCM Overlay
0x025f V8.F34         0x0262/0x0263 DIAL.F34
```

Verified in the shim by probing the entry's registers:

| | `$s1` (DspCodeBaseAddr) | `$s2` (count) |
|---|---|---|
| before | `0x805a0000` (computed fallback) | `0x0000` |
| after | `0x80338700` | `0x0040` (64) |

Every download's section sizes sum exactly to its record length, which is a
strong check that the layout matches the shipping loader.

## Next blocker: the 30-way boot handshake in `0x80082130`

Card init still returns 1 (failure), unchanged by the staging — the entry
would hang at `0x800830ec`; `run_mainloop` currently bulldozes past it and
calls the post-init functions directly, which is why the symptom is a quiet
`host_writes=0` rather than a hang.

`0x80082130` walks the 30 DSP objects twice: first calling `0x800a62cc` on
each (`slti $v0, $s1, 0x1e` at `0x800821a0`), then pairing them two at a
time (`addiu $s1, $s1, 2`) through `0x800a77e0` and `0x800a7940`. Index
0x1c (28) is special-cased against flags at `gp+0x5e97` / `0x802a b895`.

`0x800a77e0` is the real per-DSP boot handshake:

```
800a7848: jal   0x80082950        # host_write(reg_block, addr, 0x5a5a)
800a784c: addiu $a2, $zero, 0x5a5a
800a7868: jal   0x800a6298        # post command
800a7878: jal   0x80086af8        # wait, timeout 0xffff
800a78c0: jal   0x800a6204
800a78c4: ori   $s3, $zero, 0xa5a5 # expect 0xa5a5 back
```

So the pattern resembles `dsp_check_presence`, but it goes through the
firmware's command/wait machinery rather than raw IDMA, and it needs the DSP
side to actually run and answer.

### Resolved: the DSPs answer it

`0x800a77e0` is not just a probe — it is a full kernel download followed by
an alive check. It writes `0x5a5a` to the download's symbol 0, streams the
DSP kernel in through `0x80086af8`, releases the core, then polls that word
for `0xa5a5`. Getting the emulated DSPs to answer took four fixes:

**1. The IDMA destination-type bit was inverted.** Bit 14 of the IDMA
address selects 16-bit data memory when set and 24-bit program memory when
clear — see the comment in `adsp2181_idma_data_write` for the three
independent proofs in the firmware and combifile. The emulator had it the
other way round, which is also why a single PM write previously needed a
commit-on-address-change workaround to make the DSP presence check pass.
`symbol_host_address` in the shim now applies the same rule as the
firmware's resolver, and the harness call sites that hand a bare DM address
to the host port OR in `0x4000`.

**2. The shim hooked the wrong register region.** The card init computes its
own DSP register bases (`0xbc000800 + row + index*8`, kseg1 for physical
`0x1c000000`); the shim's hook was still at `0x380000` from the earlier
hand-synthesized assign path, so the bulk transfer helper's writes went to
a scratch page and its read-back verify failed.

**3. The DSP must be held for the download.** ADSP-2181 IDMA boot
(BMODE=1, MMAP=0): "Program execution is held off until on-chip program
memory location 0 is written to." The Eicon download streams from PM
`0x0001` up and releases the core with a final write to PM 0.
`adsp2181_set_idma_boot_hold` models this; a core left running executes its
own half-replaced image and corrupts the transfer, which the download's own
read-back verify catches. Any later PM write re-arms the hold, since that
means a new code download is starting; data-memory writes do not, so
mailboxes and command rings still reach a running DSP.

**4. One emulated core per DSP.** All 30 register blocks previously aliased
onto a single ADSP, so each download landed in the previous DSP's running
image. `MipsShim.core_for` now creates a core per register block (~350 KB
each, so 30 is free), each held in IDMA boot mode until its own download
completes.

With those in place `--mainloop` reports:

```
[dsp] 31 cores: 29 answered the boot handshake with 0xa5a5, 2 still held (no download)
```

Every DSP the firmware downloads to boots from its own downloaded kernel and
writes `0xa5a500` to PM `0x3fff` — the exact word `0x800a78d0` polls for.
The transfers are validated by the firmware itself: all 36 DM blocks and the
PM blocks pass its read-back verify (`0x80082a38` for DM, `0x80082b8c` for
the 24-bit PM form).

### The SIGSEGV: an over-wide register hook, not the emulator

Running the DSPs during init (`--dsp-pump N`) used to crash. It was not a
memory-safety bug in the emulator, and AddressSanitizer would not have found
it — under lldb the fault is a NULL dereference inside `libunicorn`'s code
generator (`temp_load`), reached only *after* the MIPS had already faulted.
Two shim bugs were behind it:

**The DSP register hook was too wide.** It covered `0x1c000000..0x1c002000`,
but the DSP blocks stop at `0xbc0010f0` (two on-board DSPs at `+0x08`/`+0x20`,
module rows at `+0x800..+0x870` and `+0x1000..+0x1070`, each with its address
port `0x80` above). The card object's `+0x80` holds a *control* register
block at `0xbc001800`, and `0x80082dc0` writes a byte to
`[obj+0x80] + (a1 << 4)` — `0xbc0019b0` for `a1 = 0x1b`. The hook swallowed
that, read offset `0xb0` as an IDMA address-port write, and spawned a phantom
DSP core (31 cores became 46). The firmware then read back nonsense, computed
a bad pointer, and took a MIPS CPU exception; Unicorn crashed afterwards.
The hook now ends at `0x1c001100`.

**The auto-map hook could not map.** `_unmapped` called `mem_map` directly,
which throws when the page overlaps one of the larger fixed mappings, so an
auto-mappable access surfaced as `UC_ERR_WRITE_UNMAPPED`. It now goes through
`ensure_mapped`, which consults the live region list and maps at Unicorn's
4K granularity (a 64K unit can straddle a region edge and `mem_map` rejects
any overlap).

With both fixed, `--dsp-pump 256` runs clean:

```
[mainloop] running firmware entry (basic init)...      <- no fault
[mainloop] after init: gp+0x5e81=0x0000 gp+0x5eb9=0x0060
[dsp] 31 cores: 30 answered the boot handshake with 0xa5a5, 1 still held
[mainloop] ASSIGN posted: Sig=0x4447 NextReq=0x03e0 -> 0x0500 ReqInput 32->33
```

That is the whole init sequence working: the firmware entry completes with no
faults, the DSPs answer the handshake *in line* during the validator's poll,
96 resources are registered (`gp+0x5eb9`, previously 0), and the firmware
publishes its PR_RAM signature `0x4447` — the card has booted and is ready
for host requests.

The request queue is now set up by the firmware during boot, so
`run_mainloop` posts the modem ASSIGN afterwards, following `pr_out()`
(`kernel/di.c`): fill the REQ at `B[NextReq]`, advance `NextReq` to
`REQ->next`, and bump `ReqInput` from the firmware's `ReqOutput` so the
counters start level.

The DSP count never became a blocker: 30 cores are cheap, and the validator
handshakes them one at a time.

### Resolved: the main loop now consumes the request

Two bugs, both in what the shim was writing rather than in the firmware.

**The queue counters.** The main loop treats `(ReqOutput - ReqInput) & 0xff
== 0x20` as *empty* (`0x80027ae4`) and `0x20 -` that difference as the free
slot count (`0x80027a94`), so `ReqOutput` leads `ReqInput` by 32 when idle.
The firmware initialises `ReqOutput` to 32 with `ReqInput` at 0, and the host
only ever increments `ReqInput` — one per posted request, as `pr_out()` does.
Trying to "sync" the two first left the difference at `0xff` and the loop
never saw a request.

**The entity id.** `NL_ID` is **0x20** (`kernel/pc.h`; `DSIG_ID` 0x00,
`BLLC_ID` 0x60, `TASK_ID` 0x80, `MAN_ID` 0xe0) — the shim was sending 0x01.
That is not a harmless mislabel. The main loop matches a request against the
registered entities by `entity+0x14 == translate(ReqId)`, where
`0x80029ed4` indexes a byte table at `0x80121370` by `ReqId * 2`. Entry 0 is
`0x1f` and the rest are 0, so `translate(0x01)` returned 0 and matched the
first *free* entity (94 of the 96 registered entities have id 0). The request
was then handed to that entity's protocol handler (`0x80016564`), which
compares the raw `ReqId` against its own id, found `0x01 != 0x00`, and
returned without doing anything — never reaching the assign path at
`0x80027c4c` and never acknowledging.

With both fixed the request is consumed and acknowledged on the first
iteration:

```
[mainloop] ASSIGN posted at B[0x03e0]: Sig=0x4447 NextReq->0x0500 ReqInput 0->1 ReqOutput=32
[mainloop] iter 0: v0=0x00000001 ReqIn=1 ReqOut=33 Sig=0x4447
```

`ReqOutput` 32 -> 33 is the firmware's own acknowledgement, written by
`0x80029f88` (`REQ->Reference` stamped, read offset `gp+0x5e99` advanced to
`REQ->next`, `ReqOutput++`).

`--dsp-pump` now defaults to 256: the DSPs have to run in line with the MIPS
for the validator's handshake to complete within one call, and the IDMA boot
hold makes that safe.

### The ASSIGN payload, checked against the driver

The hand-built CAI was wrong in three ways, and `add_modem_b23()` turned out
not to be where a CAI comes from at all.

**Framing.** An IDI request payload is a list of `{code, length, data}`
triples with a single zero code byte terminating it — `add_ie()`
(`kernel/message.c`) writes a `0` after each parameter and backs over it when
the next is appended. The shim was writing a bare 26-byte blob with no code,
no length and no terminator.

**Wrong entity.** `add_modem_b23()` builds **LLI/LLC/DLC**, not a CAI, and
those go on the *network-layer* ASSIGN (`nl_req_ncci(plci, ASSIGN, 0)`). The
CAI is built by `add_b1()` and rides on the *signalling* ASSIGN
(`sig_req(plci, ASSIGN, DSIG_ID)`). The driver sends the signalling ASSIGN
first.

**Length.** `add_b1()` sets `cai[0] = 26` for a modem B1 protocol; the shim
had 25.

The CAI *content* was largely right. `cai[1] = 0x11` is correct — `add_b1()`'s
`resource[] = {5,9,13,12,16,39,9,17,17,18}` maps B1 protocol 7/8
(`B1_MODEM_ALL_NEGOTIATE` / `B1_MODEM_ASYNC`) to 17 = `DSP_CAI_HARDWARE_MODEM_ASYNC`
— and the Tx/Rx speed words were already at the right offsets (`cai[15]` /
`cai[19]`, with the minima at `cai[13]` / `cai[17]`).

`idi_parameters()`, `modem_cai()`, `modem_sig_assign_payload()` and
`modem_nl_assign_payload()` now build both, selected by `--entity`:

```
sig: 10 1a <26-byte CAI> 2d 06 "Capi20" 00
nl:  19 01 01 · 7c 02 09 04 · 20 09 0004 03 01 07 07 0000 43 · 00
```

The NL payload decodes as MaxDataLength 1024, Addr A 3, Addr B 1, modulo 7,
window 7, no XID, and negotiation flags `0x43`
(`DISABLE_V42_V42BIS | DISABLE_MNP_MNP5 | DISABLE_SDLC`) — the plain
`B2_TRANSPARENT` branch of `add_modem_b23()`. LLC `{9, 4}` is V42_IN with L3
transparent, i.e. the answering side.

### Next: still no return code

Correct framing did not by itself produce one, so the payload was not the
blocker. Both entities behave the same: the request is consumed and
acknowledged on the first iteration (`ReqOutput` 32 -> 33) but `RcOutput`
stays 0 over 200 iterations, and there are still no host writes.

`RcOutput` is genuinely card-owned — `pr_rc()` in `kernel/di.c` reads it as
the count of pending RCs, walks the chain from `NextRc`, and clears it — so 0
means no RC has been produced rather than one being missed. An ASSIGN must be
answered with an `ASSIGN_RC` (0xe0..0xef) carrying the assigned local id.

Worth checking next: `NextRc` is 0 while `NextReq` (0x03e0) and `NextInd`
(0x27e0) are firmware-initialised, so it is not clear whether B[0] is a
genuine RC chain head or an uninitialised pointer. If the host is supposed to
build the RC/IND chains, that would explain an entity that accepts a request
but can never answer it.

## Why `.2qm` is still expensive

The `.2qm` firmware (build 122-11) uses a different code model than `.pm`:

- `.pm` sets a global `$gp = 0x800fa3b5` via `lui gp, 0x8010; addiu gp, gp,
  -0x5c4b` at file `0x474`/`0x64c`;
- `.2qm` has **zero `lui gp` instructions** — position-independent or
  per-function gp. `lui a0` values cluster around `0x8028`/`0x8029`.

Re-deriving `MIPS_ENTRY`, `MIPS_POST_INIT1/2`, `MIPS_MAINLOOP`, the `$gp`
base and the trace-printf pointer is a real reverse-engineering task.
`.2qm` and `.am` are the same build and share structure, so entry points
found once would apply to both.

## Version note

`te_dmlt.pm` identifies as build 107-79 (`TE_DMLT, Build 107-79, Protocol
6.03(V11) 104-102`), while `dspdload.bin` is build 117-926 and
`dspdvmdm.bin` is 103-492. No combifile in `docs/firmware/` matches the
protocol build exactly. The container format is versioned separately
(`format_version_bcd`) and the firmware accepted the 117-926 table, but a
mismatch is worth ruling out if a download is later rejected by id.
