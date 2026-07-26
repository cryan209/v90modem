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
reaches IDLE at PM `0x02a9`. Exact Eicon type-2 relocation is implemented. TIKRNL/V.90 can be staged after boot
with `ADSP_STAGE_PM_WORDS` and `ADSP_STAGE_DM_WORDS`; `ADSP_STAGE_ENTRY=0x672`
runs the TIKRNL initializer with a valid return stack. Activating a modem
channel still requires the MIPS firmware's `dsp_assign` database transaction.
