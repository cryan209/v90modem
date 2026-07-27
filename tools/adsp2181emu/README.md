# Standalone ADSP-2181 execution harness

This is an early system-emulation harness for the Eicon/Dialogic modem DSP
firmware. `adsp2181_core.c` and `2100ops.inc` are standalone adaptations of
MAME's ADSP-21xx CPU core (BSD-3-Clause, copyright Aaron Giles). The command
wrapper is GPL-2.0-or-later, matching this repository.

Build:

```bash
make -C tools/adsp2181emu
```

Run an extracted addressed image:

```bash
tools/adsp2181emu/eicon_adsp_run PM.bin DM.bin [cycles]
```

`ADSP_TRACE=N` traces the first N instructions. `ADSP_RX0`/`ADSP_RX1` and
`ADSP_TX0`/`ADSP_TX1` attach raw little-endian signed 16-bit SPORT files;
`ADSP_TRACE_SPORT=N` logs initial transfers. The core models ADSP-2181
interrupts, IDMA and both PMOVLAY/DMOVLAY banks. The Eicon MIPS-side channel
assignment protocol and real-time sample clock are not modeled yet.

The first validated target is the bootable `DIVA Server PRI 30M Kernel`. It
executes reset instruction `0x18580f`, enters at PM `0x0580`, initializes and
reaches IDLE at PM `0x02a9`. Exact Eicon type-2 relocation is implemented.
TIKRNL/V.90 can be staged after boot with `ADSP_STAGE_PM_WORDS` and
`ADSP_STAGE_DM_WORDS`; `ADSP_STAGE_ENTRY=0x672` runs the TIKRNL initializer
with a valid return stack.

`ADSP_POST_DM_WORDS` applies an addressed DM word map after all staged entry
points have run. This is the correct seam for replaying MIPS `dsp_assign`
writes: applying them with `ADSP_STAGE_DM_WORDS` would let the TIKRNL
initializer overwrite them. `tools/eicon_dsp_assign.py` locates the stripped
assignment routines and the TIKRNL command/database mailboxes.

The MIPS shim (`tools/eicon_mips_shim.py --assign`) drives the real
service-assign entry (0x80096980) under Unicorn and routes both the
single-word host-port helper and the bulk-write IDMA helper (memory-mapped
writes to the host register block at +0x80/+0x00) to the ADSP emulator's
IDMA interface, so the switch-on database commit reaches the DSP through
the firmware's own code path.

## Host-interface model and instrumentation

The IDMA model now follows the ground-truth semantics recovered from the
te_dmlt.pm host-port helper (runtime `0x80082950`): IDMA addresses with bit
`0x4000` set select 24-bit PM (two data writes per word), addresses below
`0x4000` select 16-bit DM. `adsp2181_host_write()` implements the MIPS
helper's single-word transaction (PM words land as `value<<8`).

The MIPS script sender commits modem commands as: script record words into a
16-word PM ring at PM `0x3327..0x3336` (host addresses `0x7327..`), then the
producer pointer to DM `0x3315`, the command selector to DM `0x3310`, and the
control word with bit `0x20` cleared to DM `0x3338`.

Harness additions:

- `ADSP_WATCH_DM=a,b,c` / `ADSP_WATCH_PM=a,b,c` log DSP reads/writes of the
  listed addresses with PC and cycle.
- `ADSP_WATCH_ALL=1` logs all DM/PM data accesses during the host loop.
- `ADSP_TRACE_HOST=N` traces the first N instructions executed inside the
  host loop (including ISR bodies).
- `ADSP_HOST_SCRIPT=path` replays host writes, one `<word-index> <addr>
  <value>` hex triplet per line, through `adsp2181_host_write`.
- `ADSP_HOST_POLL=a,b,c` prints host-visible mailbox changes after each word.
- `ADSP_HOST_IRQ=n` pulses IRQ `n` alongside each SPORT0 RX strobe.

Current status: with TIKRNL staged and initialized, the SPORT0 RX ISR remains
the kernel's per-timeslot TDM state machine (DM `0x2e44/0x2e45` channel-table
pointers, `0x2e50` substate countdown, `0x2e52` current PCM code, idle code
`0x00ff`). No IRQ doorbell (0/1/2/6) makes the DSP read the TIKRNL command
mailbox; activation requires replaying the `dsp_assign` initial database
writes that hook the channel table to the modem task.
