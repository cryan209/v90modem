#!/usr/bin/env python3
"""Collect executed ADSP-2181 instructions from a native-MIPS INFO replay.

This is coverage, not a correctness oracle.  It reduces the opcode audit to
firmware instructions actually reached on a captured call, while the INFO
image is still resident so each PM address resolves to the word that ran.
The ADSP-2181 manual remains authoritative for semantics and ASTAT effects.

    make -C tools/adsp2181emu
    /tmp/eicon-venv/bin/python tools/adsp_opcode_audit.py \
        artifacts/eicon-native-tower/run14.rx.ulaw --to 9.5 \
        --out /tmp/info-opcodes.tsv
"""
from __future__ import annotations

import argparse
import collections
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from adsp2181_dis import disas
from eicon_mips_shim import ADSP, create_native_mips_modem

SAMPLE_RATE = 8000
KERNEL = Path('artifacts/eicon-dsp/build-117-926/kernel/'
              '0009-diva-server-pri-30m-kernel')
TIKRNL = Path('artifacts/eicon-dsp/build-117-926/tikrnl/'
              '0258-tikrnl81.f34-task')


def family(op: int) -> str:
    top = op >> 16
    if op == 0:
        return 'nop'
    if top == 0x02:
        return 'idle/flag'
    if top in (0x0A, 0x0B) or 0x18 <= top <= 0x1F:
        return 'control-flow'
    if 0x14 <= top <= 0x17:
        return 'hardware-loop'
    if top in (0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13):
        return 'shifter'
    if 0x20 <= top <= 0x27 or 0x50 <= top <= 0x7F:
        return 'alu/mac'
    if 0x28 <= top <= 0x2F:
        return 'alu-constant'
    if 0x30 <= top <= 0x4F:
        return 'immediate/move'
    if 0x80 <= top <= 0xFF:
        return 'memory'
    return 'system/move'


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('capture', type=Path, help='captured PCMU receive stream')
    ap.add_argument('--to', type=float, default=9.5,
                    help='replay stop time (default: 9.5)')
    ap.add_argument('--page', type=lambda x: int(x, 0), default=0x0260,
                    help='clear coverage when this overlay loads (default: 0x0260)')
    ap.add_argument('--tx-prbs', action='store_true',
                    help='supply deterministic V90D transmit datagrams')
    ap.add_argument('--watch-exec', default='',
                    help='comma-separated PM addresses to log during replay')
    ap.add_argument('--out', type=Path, default=Path('/tmp/info-opcodes.tsv'))
    args = ap.parse_args()

    data = args.capture.read_bytes()
    card = create_native_mips_modem(KERNEL, TIKRNL, 'pcmu',
                                    force_info_after_v8=True,
                                    tx_prbs=args.tx_prbs)
    ADSP.adsp2181_coverage_clear(card.cpu)
    for text in args.watch_exec.split(','):
        if text.strip():
            ADSP.adsp2181_watch_exec(card.cpu, int(text, 0), 1)
    limit = min(len(data), int(args.to * SAMPLE_RATE) + 1)
    page_start = None
    for index, code in enumerate(data[:limit]):
        before = card.resident
        card.frame_fast(code, index)
        if page_start is None and before != args.page and card.resident == args.page:
            # Discard earlier-page coverage. PM is movable and those addresses
            # no longer contain the instructions that ran before this load.
            ADSP.adsp2181_coverage_clear(card.cpu)
            page_start = index + 1

    if card.resident != args.page:
        raise SystemExit(f'page 0x{args.page:04x} is not resident at '
                         f'{args.to:.3f}s: overlay=0x{card.resident:04x}')

    rows = []
    families: collections.Counter[str] = collections.Counter()
    top_bytes: collections.Counter[int] = collections.Counter()
    for pc in range(0x4000):
        count = ADSP.adsp2181_coverage_count(card.cpu, pc)
        if not count:
            continue
        op = ADSP.adsp2181_read_pm(card.cpu, pc) & 0xFFFFFF
        name = family(op)
        rows.append((pc, op, count, name, disas(op)))
        families[name] += count
        top_bytes[op >> 16] += count

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open('w') as out:
        out.write('pc\topcode\texecutions\tfamily\tdisassembly\n')
        for pc, op, count, name, text in rows:
            out.write(f'{pc:04x}\t{op:06x}\t{count}\t{name}\t{text}\n')

    if page_start is None:
        raise SystemExit(f'page 0x{args.page:04x} was never loaded')
    print(f'[coverage] samples={limit - page_start} page_start={page_start} '
          f'resident=0x{card.resident:04x} unique_PCs={len(rows)} '
          f'total_instructions={sum(families.values())}')
    for name, count in families.most_common():
        print(f'  {name:18s} {count:12d}')
    print('[coverage] executed opcode top bytes:')
    print('  ' + ' '.join(f'{top:02x}:{count}'
                           for top, count in sorted(top_bytes.items())))
    print(f'[coverage] wrote {args.out}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
