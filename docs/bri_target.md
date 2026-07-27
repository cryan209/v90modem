# Re-deriving .2qm entry points: status

## Attempted: .2qm (BRI build 122-11) entry-point re-derivation

Goal: switch the shim from `.pm` (PRI, 30-DSP detect) to `.2qm` (BRI,
2-DSP detect) so `dsp_assign` runs with single-DSP-ish geometry.

### Result: blocked by a different code model

The `.2qm` firmware (build 122-11) uses a **different code model** than
`.pm` (build 117-926):

- `.pm` sets a global `$gp = 0x800fa3b5` via `lui gp, 0x8010; addiu gp,
  gp, -0x5c4b` at file 0x474/0x64c (the boot entry).
- `.2qm` has **zero `lui gp` instructions**. It uses position-independent
  / per-function gp or a different ABI. The `lui a0` values cluster around
  0x8028/0x8029 (the .data segment), not a global gp.

Re-deriving the entry points (MIPS_ENTRY, MIPS_POST_INIT1/2, MIPS_MAINLOOP,
the $gp base, the trace-printf pointer) for this code model is a real
reverse-engineering task — not a quick port from `.pm`. The `.2qm` and
`.am` are the same build (122-11) and share structure, so entry points
would apply to both, but they need to be found fresh.

## Alternative: make .pm's 30-DSP detect pass

The `.pm` PRI firmware's DSP detect expects 30 DSPs at register addresses
`0x380000 + row_offset[row] + dsp_index*8` (rows at 0, 0x800, 0x840,
0x1000, 0x1040). The shim's DSP hook covers 0x380000-0x390000 (all 30
DSPs) but routes them all to **one shared IDMA interface** — so DSP 1's
presence check sees DSP 0's leftover idma_cache, breaking the alternating
0x5a5a/0xa5a5 test that `dsp_check_presence` performs.

Two ways to make the PRI detect pass:
1. **Per-DSP IDMA cache**: give each of the 30 DSP addresses its own
   idma_addr/idma_cache pair in the hook, so each presence check is
   isolated. Only DSP 0 backs a real ADSP; DSPs 1-29 are fake (presence
   check passes but they never run).
2. **Fake the bitmask**: intercept the detect's read of the DSP-present
   bitmask and return "all 30 present", skipping the per-DSP presence
   check entirely.

Option 1 is cleaner (the detect runs naturally) but needs the hook to
track per-DSP IDMA state. Option 2 is a hack but quick.

## Emulator status (unchanged, both fixes solid)

1. OOB stack-pop underflow — fixed, verified.
2. IDMA PM write commit — fixed, presence check round-trips 0x5a5a.
3. Lockstep ADSP pump (IRQE every 256 MIPS instructions) — in place.
4. DSP-register hook (0x380000-0x390000) — covers all 30 DSP addresses,
   but routes to one shared IDMA (the multi-DSP detect blocker).

## Recommendation

The `.pm` path is closer to working (entry points known, just the multi-
DSP detect to fix). The per-DSP IDMA cache (option 1) is the smaller
change: extend the `_dsp_read`/`_dsp_write` hooks to key on the DSP
address (which DSP), give each its own idma_addr/idma_cache, and only
DSP 0 routes to the real ADSP. Then all 30 presence checks pass, the
firmware registers 30 "DSPs" (only DSP 0 real), and `dsp_assign`
proceeds.
