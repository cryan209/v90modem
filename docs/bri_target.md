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

Two ways forward, in order of preference:

1. **Make the emulated ADSP answer the handshake.** The shim's
   `host_write`/`host_read` hooks are function-level (`0x80082950` /
   `0x80082920`) and ignore `a0` — the register block — so all 30 DSPs
   already alias onto the single emulated ADSP. If one boot handshake
   completes, all 30 iterations complete. This is the honest path and it is
   what a real single-DSP call would exercise.
2. **Short-circuit the validator** by forcing `0x80082130` to return 0.
   Quick, but it skips whatever per-DSP state the validator establishes
   (`+0x108`/`+0x110` counters on the card object, the `0x1acb`/`0x1acf`
   gp-relative masks cleared per DSP index) and that state is likely read
   later during assign.

The DSP count is only reachable as a blocker at step 1, and only if the
single emulated DSP cannot be made to answer 30 times.

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
