#!/usr/bin/env python3
"""Replay a capture through the V.90 data pump and watch its global countdown.

**Use this, not `eicon_info_replay.py`, for anything on overlay `0x026a`.**
The two drive different harnesses over the same firmware:

    eicon_info_replay.py   LiveKernelModem      (dial_kernel_dispatch)
    this tool              create_native_mips_modem()  (eicon_mips_shim)

Live captures come from `eicon_adsp_sip.py --native-mips`, i.e. the second one.
On the INFO page the two agree closely enough to be confused for each other; on
page 14 they do not, and the kernel-dispatch harness parks in TrnProgress
`0x0060` where the native one walks `0x0060 -> 0x0062 -> ...` exactly as the
live card does.  Session 50 lost an afternoon to that.

What it prints: the data pump's two record layers (as decoded by
`tools/v90_dpcm_state_records.py`) on every change, plus each seed and expiry
of the global countdown `DM(0x20e0)` that condition index `0x02` tests.

    PM 0x2c7d   MY0 = PM(0x200c + DM(0x20e3)) -- the symbol-rate scale table
                0x200c..0x2011 = 0.2400 0.2743 0.2800 0.3000 0.3200 0.3429,
                the V.34 baud family over 10000.  Index 4 is 3200 baud, and
                the countdown is measured ticking at 3200 Hz.
    PM 0x2c65   AR = 0x7530 ; scale -> 9600 ticks = 3.000 s
    PM 0x2c6b   AR = 0x4e20 ; scale ; double -> 12800 ticks = 4.000 s
    PM 0x2c78   AR = MR1 + DM(0x3fcb) ; DM(0x20e0) = AR
    PM 0x2cb4   DM(0x3fcb) = DM(0x3fc9) * 10/3     -- the addend
    PM 0x2f7d   AY0 = DM(0x20e0) ; AR = AY0 - 1 ; DM(0x20e0) = AR, clamped at 0

The 0x2c65 path falls through PM 0x2c78 into PM 0x2c68 (`AR = AR + AY0`) and
stores again, so it adds `DM(0x3fcb)` *twice*.  `DM(0x3fc9)` is not the data
pump's at all: the resident INFO page maintains it at PM 0x3caf/0x3cb4, and
whatever it has reached at the handoff lands in every deadline this page sets.

Needs the MIPS emulator, so run it under the venv that has `unicorn`, and
build the ADSP core first -- `libadsp2181.dylib` is gitignored and the
top-level makefile does not build it:

    make -C tools/adsp2181emu
    /tmp/eicon-venv/bin/python tools/v90_dpcm_replay.py CAPTURE.rx.ulaw
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from eicon_mips_shim import create_native_mips_modem

SAMPLE_RATE = 8000
KERNEL = Path('artifacts/eicon-dsp/build-117-926/kernel/'
              '0009-diva-server-pri-30m-kernel')
TIKRNL = Path('artifacts/eicon-dsp/build-117-926/tikrnl/'
              '0258-tikrnl81.f34-task')

# The data pump's working set.  Layers and offsets from
# tools/v90_dpcm_state_records.py; DM(0x20e0..0x20e3) from the seeders above.
WORDS = {
    'trn': 0x3FC2,      # published TrnProgress = outer state AND 0x00ff
    'count': 0x20E0,    # global countdown, condition index 0x02
    'rate': 0x20E3,     # index into the PM 0x200c symbol-rate scale table
    'addend': 0x3FCB,   # DM(0x3fc9) * 10/3, added to every seed
    'optr': 0x120F, 'ostate': 0x1FF7, 'odwell': 0x1FF6,
    'iptr': 0x204A, 'istate': 0x2008, 'idwell': 0x2007,
}
# The countdown moves every tick, so key on the record pointers and states
# only -- otherwise this prints one line per sample for six seconds.
KEY = ('trn', 'optr', 'ostate', 'iptr', 'istate')
DM_COUNT = 0x20E0
DM_ELAPSED = 0x3FC9
DM_ADDEND = 0x3FCB


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('capture', type=Path, help='a .rx.ulaw capture (PCMU)')
    ap.add_argument('--from', dest='start', type=float, default=0.0)
    ap.add_argument('--to', dest='end', type=float, default=25.0,
                    help='stop here; the replay runs far slower than real time')
    ap.add_argument('--tx-prbs', action='store_true',
                    help='answer V90D TX requests with deterministic PRBS data')
    ap.add_argument('--native-bearer-activation', action='store_true',
                    help='use lower-PRI event 03 task attachment before ADDSP answer setup')
    args = ap.parse_args()

    data = args.capture.read_bytes()
    card = create_native_mips_modem(KERNEL, TIKRNL, 'pcmu',
                                    force_info_after_v8=True,
                                    tx_prbs=args.tx_prbs,
                                    native_bearer_activation=args.native_bearer_activation)
    dm = card.dm
    print('[replay] native-MIPS harness ready', flush=True)

    previous = None
    seeded = None
    live = total = page14_live = page14_total = 0
    first_page14_tx = None
    for index, code in enumerate(data):
        seconds = index / SAMPLE_RATE
        if seconds > args.end:
            break
        sample = card.frame_fast(code, index)
        if seconds < args.start:
            continue

        count = dm[DM_COUNT]
        if seeded is None or count > seeded[1]:
            # The only way the countdown rises is a seed: PM 0x2f7d only ever
            # decrements it, and clamps at zero.
            seeded = (seconds, count)
            if count:
                print(f'{seconds:8.4f}  countdown seeded {count:#06x} = {count}'
                      f' ticks, addend DM(0x3fcb)={dm[DM_ADDEND]} from'
                      f' DM(0x3fc9)={dm[DM_ELAPSED]}', flush=True)
        elif count == 0 and seeded[1]:
            held = seconds - seeded[0]
            print(f'{seconds:8.4f}  countdown expired after {held:.4f} s '
                  f'({seeded[1] / held:.0f} ticks/s)', flush=True)
            seeded = (seconds, 0)

        key = tuple(dm[WORDS[name]] for name in KEY)
        if key != previous:
            fields = ' '.join(f'{n}={dm[a]:04x}' for n, a in WORDS.items())
            print(f'{seconds:8.4f}  {fields}', flush=True)
            previous = key

        total += 1
        live += 1 if sample else 0
        if card.resident == 0x026A:
            page14_total += 1
            page14_live += 1 if sample else 0
            if sample and first_page14_tx is None:
                first_page14_tx = (seconds, sample, dm[WORDS['ostate']])
                print(f'{seconds:8.4f}  first V90D TX sample={sample} '
                      f'outer_state={first_page14_tx[2]:04x}', flush=True)

    print(f'TX over the replayed window: {100.0 * live / max(1, total):.1f}% '
          f'non-zero of {total} samples; page 14: '
          f'{100.0 * page14_live / max(1, page14_total):.1f}% non-zero of '
          f'{page14_total}; TX datagrams '
          f'{card.tx_accepted}/{card.tx_requests} accepted/requested')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
