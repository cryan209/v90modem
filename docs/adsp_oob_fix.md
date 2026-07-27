# Boot-acknowledge + DSP detect: mainloop progress

## IDMA presence-check fix verified in mainloop

With the IDMA fix (`docs/adsp_oob_fix.md`), the `--mainloop` path now
reaches the DSP detect: the firmware writes 0x5a5a to PM[0] via IDMA and
reads it back successfully. Previously this returned 0 and the firmware
skipped DSP registration entirely.

## Lockstep ADSP pump added

Added an ADSP pump to the MIPS code hook (`_hook` in `eicon_mips_shim.py`):
every 256 MIPS instructions, strobe IRQE (irq 6) and run the ADSP ~2000
cycles. This keeps the DSP running in lockstep with the MIPS so it can
boot, acknowledge, and process commands the MIPS downloads. Without it the
MIPS init hangs polling for a DSP boot-ack that never comes.

## DSP detect reaches but bails before registration

With both fixes, the init now completes (no hang) and **reaches the DSP-
detect loop** at MIPS runtime 0x800294AC-0x80029504 (17 trace hits). But
it stops just before 0x800295A8 (`sh $a0, 0x5eb9($gp)` — the registration
write). The detect loop bails early because it's the **PRI 30M card's
multi-DSP detect**: it expects 30 DSPs in 4 rows of 7-8, and checks a
specific bitmask geometry. The emulator models one DSP, so the per-DSP
detect for DSPs 1-29 fails and the loop exits without registering even
DSP 0.

The firmware image `te_dmlt.pm` is PRI-specific (CardType=12,
CARD_MAEP PRI 30M). Its DSP detect is hardwired to the 30-DSP PRI geometry.

## The right path: analog firmware + Analog kernel

The combifile has the **DIVA Server Analog Kernel** (id 0x000d, single
DSP) and the repo has `docs/firmware/te_dmlt.am` (analog modem firmware,
2.2MB). The Analog kernel's DSP detect expects exactly one DSP — matching
the emulator. Switching to:

- DSP image: `000d-diva-server-analog-kernel` (single-channel)
- MIPS firmware: `docs/firmware/te_dmlt.am`
- CardType: 92 (CARDTYPE_DIVASRV_ANALOG_2PORT)

would make the DSP detect pass with one DSP, registering it (gp+0x5eb9 != 0),
and the main loop would then process the ASSIGN request with the real
`dsp_assign` — planting the command-ring consumer pointer and hooking the
channel table naturally.

## Remaining work for the analog pivot

The `.am` firmware is a different image (2.2MB vs 0.98MB) with different
code at the same offsets. Re-deriving the MIPS entry points
(MIPS_ENTRY 0x80082f90, MIPS_POST_INIT1/2, MIPS_MAINLOOP 0x80027970,
the trace-printf pointer at gp+0x1a7b, the $gp base) for `.am` is needed.
The DSP-register-region hook (physical 0x380000) and the IDMA routing
should be reusable as-is.

## Summary of emulator fixes

1. **OOB stack-pop underflow** (`2100ops.inc`) — guarded the array reads
   in pc_stack_pop, pc_stack_pop_val, cntr_stack_pop, stat_stack_pop.
2. **IDMA PM write commit** (`adsp2181_core.c`) — `idma_addr_write` now
   commits a pending PM write as `cache<<8` before changing address,
   matching real ADSP-2181 hardware. The Eicon presence check (single
   data write + re-address + read-back) now round-trips 0x5a5a correctly.

Both fixes are concrete emulator bugs (not guest-state issues) and are
verified by direct IDMA round-trip tests and the mainloop reaching DSP
detect.
