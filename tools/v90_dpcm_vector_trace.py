#!/usr/bin/env python3
"""Trace the V90D transmit sample vector and find what fills it.

Companion to `tools/v90_dpcm_replay.py`, same harness
(`create_native_mips_modem()`), narrower question: run33 reaches outer state
`0x0080` and transmits, but the wire carries one nonzero sample every six
(`0,0,3388,0,0,0` repeating), i.e. the serializer publishes a six-sample vector
of which only one slot is ever populated.  The dataflow under test (session 54,
`docs/eicon_adsp_firmware_analysis.md`):

    PM 0x3db9..0x3dc6   DM(0x10ae..0x10b3) -> DM(0x3fa7..0x3fac)
                        three samples followed by their negatives
    PM 0x2ee9           dispatch through DM(0x2039); transmit mode selects
                        PM 0x2eed..0x2ef2, which walks the vector through
                        DM(0x20de), one signed-linear sample per frame
    PM 0x24de..0x24e3   handler set derived from inner field DM(0x2001)

What it found, and what each mode is for:

- default: samples those words every page-14 frame and reports per-slot
  occupancy.  The source vector holds a persistent mapping frame; the
  destination block reads zero at every frame boundary.
- `--watch ADDR --watch-from S [--watch-to S]` and `--watch-exec PM ADDR`
  turn on the core's DM-write / PM-execution watches for a short window, so
  writer PCs and execution history land on stderr.  Far too loud to leave on:
  keep the window under a few hundred milliseconds of audio.  These identified
  PM 0x2a52 (`CALL (I4)`, AX0 = 0x3fa7) as the generator dispatch and PM
  0x06ca..0x06cd, in the resident-kernel frame tail, as the per-frame clear.
- `--count PM ADDR` counts executions from outer state 0x0080: the generator
  dispatch runs 0.167/frame (the 1333 Hz mapping frame) while the clear runs
  6.000/frame, i.e. all six words are zeroed every 8 kHz frame.
- `--hold-block` re-copies the source vector before each frame, which is what
  the generator would have left behind had the block survived.  It takes the
  published output from 1 nonzero sample in 6 to 4 in 6 -- the source's own
  occupancy -- confirming the clear, not the generator, loses the samples.
- `--dump-tx` writes DM(0x3fb4) read as a *value*.  Nothing writes the generic
  adapter output word DM(0x3764) during V90D transmit, so the pointer
  indirection `frame_fast` applies has nothing to dereference on this page:
  PM 0x1a1e leaves the sample itself in DM(0x3fb4).

Runs against the emulator, so use the venv with `unicorn` and build the core
first (`libadsp2181.dylib` is gitignored):

    make -C tools/adsp2181emu
    /tmp/eicon-venv/bin/python tools/v90_dpcm_vector_trace.py \\
        artifacts/eicon-native-tower/run33.rx.ulaw --to 13.2

`EICON_V90D_BULK_ADAPTER=1` keeps the 0x1900..0x19c8 echo bulk-delay adapter
live instead of the diagnostic that RTS-es it out.  With the adapter live the
outer state machine does not reach 0x0080 at all, so there is no transmit era
to trace.
"""
from __future__ import annotations

import argparse
import collections
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import eicon_mips_shim as SHIM
from eicon_mips_shim import ADSP, create_native_mips_modem

SAMPLE_RATE = 8000
KERNEL = Path('artifacts/eicon-dsp/build-117-926/kernel/'
              '0009-diva-server-pri-30m-kernel')
TIKRNL = Path('artifacts/eicon-dsp/build-117-926/tikrnl/'
              '0258-tikrnl81.f34-task')

VECTOR = tuple(range(0x10AE, 0x10B4))     # the six-sample source vector
PUBLISHED = tuple(range(0x3FA7, 0x3FAD))  # its copy, three samples + negatives
DM_SERIAL = 0x20DE      # serializer cursor/output word
DM_MODE = 0x2039        # PM 0x2ee9 dispatch selector
DM_HANDLER = 0x2001     # inner field the handler set derives from
DM_OSTATE = 0x1FF7
DM_ISTATE = 0x2008
DM_TXPTR = 0x3FB4       # generic transmit pointer; frame_fast reads DM[DM[.]]


def signed(value: int) -> int:
    return value - 0x10000 if value & 0x8000 else value


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('capture', type=Path, help='a .rx.ulaw capture (PCMU)')
    ap.add_argument('--from', dest='start', type=float, default=0.0)
    ap.add_argument('--to', dest='end', type=float, default=25.0)
    ap.add_argument('--tx-prbs', action='store_true', default=True,
                    help='answer V90D TX requests with PRBS data (default)')
    ap.add_argument('--no-tx-prbs', dest='tx_prbs', action='store_false')
    ap.add_argument('--native-bearer-activation', action='store_true',
                    default=True, help='lower-PRI event 03 attachment (default)')
    ap.add_argument('--dump-tx', type=Path, default=None,
                    help='write DM(0x3fb4) read as a signed sample (not as a '
                         'pointer) for every page-14 frame, int16 LE')
    ap.add_argument('--dump-pm', type=Path, default=None,
                    help='once page 14 is resident, write PM 0x0000..0x3fff as '
                         '3-byte little-endian words (tools/adsp2181emu/dasm '
                         'input) and stop')
    ap.add_argument('--hold-block', action='store_true',
                    help='diagnostic: restore DM(0x3fa7..0x3fac) after every '
                         'page-14 frame that leaves it zeroed, so the six-word '
                         'mapping frame survives the resident-kernel clear at '
                         'PM 0x06ca..0x06cd for all six serializer reads')
    ap.add_argument('--count', type=lambda t: int(t, 0), action='append',
                    default=[], help='PM address to count executions of; '
                                     'coverage is cleared at outer state 0x0080')
    ap.add_argument('--changes', action='store_true',
                    help='print every change of the traced words, not a summary')
    ap.add_argument('--watch', type=lambda t: int(t, 0), action='append',
                    default=[], help='DM address to watch for writes (repeatable)')
    ap.add_argument('--watch-exec', type=lambda t: int(t, 0), action='append',
                    default=[], help='PM address to watch for execution (repeatable)')
    ap.add_argument('--watch-from', type=float, default=None)
    ap.add_argument('--watch-to', type=float, default=None)
    args = ap.parse_args()

    data = args.capture.read_bytes()
    card = create_native_mips_modem(KERNEL, TIKRNL, 'pcmu',
                                   force_info_after_v8=True,
                                   tx_prbs=args.tx_prbs,
                                   native_bearer_activation=args.native_bearer_activation)
    dm = card.dm
    print(f'[trace] native-MIPS harness ready; bulk adapter '
          f'{"disabled" if SHIM.V90D_BULK_ADAPTER_DISABLED else "LIVE"}',
          flush=True)

    dump = args.dump_tx.open('wb') if args.dump_tx else None
    watch_on = False
    counted_from = None
    held = None
    previous = None
    page14 = 0
    nonzero_slot = collections.Counter()
    slot_values = collections.defaultdict(collections.Counter)
    serial_values = collections.Counter()
    mode_values = collections.Counter()
    handler_values = collections.Counter()
    txptr_values = collections.Counter()
    published_nonzero = collections.Counter()
    out_values = collections.Counter()
    first_state = {}

    for index, code in enumerate(data):
        seconds = index / SAMPLE_RATE
        if seconds > args.end:
            break

        if args.watch or args.watch_exec:
            want = (args.watch_from is None or seconds >= args.watch_from) and \
                   (args.watch_to is None or seconds <= args.watch_to)
            if want != watch_on:
                for addr in args.watch:
                    ADSP.adsp2181_watch_dm(card.cpu, addr, 1 if want else 0)
                for addr in args.watch_exec:
                    ADSP.adsp2181_watch_exec(card.cpu, addr, 1 if want else 0)
                print(f'{seconds:8.4f}  watch {"on" if want else "off"} for '
                      f'dm={[f"{a:#06x}" for a in args.watch]} '
                      f'pm={[f"{a:#06x}" for a in args.watch_exec]}', flush=True)
                watch_on = want

        if (args.hold_block and held is not None and any(held)
                and card.resident == 0x026A and dm[DM_OSTATE] >= 0x0080):
            # The generator refills DM(0x3fa7..0x3fac) once per 1333 Hz mapping
            # frame, but the resident-kernel tail at PM 0x06ca..0x06cd zeroes
            # all six words every 8 kHz frame, so five of six serializer reads
            # find zero. Re-copy the persistent source vector before each frame,
            # which is what the generator would leave behind if the block
            # survived, and see whether the six samples reach the wire.
            for addr, value in zip(PUBLISHED, held):
                dm[addr] = value

        sample = card.frame_fast(code, index)
        if card.resident != 0x026A:
            continue

        if args.dump_pm is not None:
            words = bytearray()
            for addr in range(0x4000):
                word = ADSP.adsp2181_read_pm(card.cpu, addr) & 0xFFFFFF
                words += bytes((word & 0xFF, (word >> 8) & 0xFF, word >> 16))
            args.dump_pm.write_bytes(bytes(words))
            print(f'{seconds:8.4f}  wrote {args.dump_pm} '
                  f'(pmovlay={ADSP.adsp2181_pmovlay(card.cpu)}, '
                  f'ostate={dm[DM_OSTATE]:04x})', flush=True)
            return 0

        page14 += 1
        if dump is not None:
            dump.write(struct.pack('<h', signed(dm[DM_TXPTR])))
        held = [dm[a] for a in VECTOR]
        state = dm[DM_OSTATE]
        if args.count and state >= 0x0080 and counted_from is None:
            ADSP.adsp2181_coverage_clear(card.cpu)
            counted_from = page14
            print(f'{seconds:8.4f}  coverage cleared at ostate={state:04x}',
                  flush=True)
        first_state.setdefault(state, seconds)
        vector = [dm[a] for a in VECTOR]
        for slot, value in enumerate(vector):
            if value:
                nonzero_slot[slot] += 1
                slot_values[slot][value] += 1
        for slot, addr in enumerate(PUBLISHED):
            if dm[addr]:
                published_nonzero[slot] += 1
        serial_values[dm[DM_SERIAL]] += 1
        mode_values[dm[DM_MODE]] += 1
        handler_values[dm[DM_HANDLER]] += 1
        txptr_values[dm[DM_TXPTR] & 0x3FFF] += 1
        out_values[sample] += 1

        if args.changes:
            key = (state, dm[DM_ISTATE], dm[DM_MODE], dm[DM_HANDLER],
                   dm[DM_TXPTR] & 0x3FFF, tuple(vector),
                   tuple(dm[a] for a in PUBLISHED), dm[DM_SERIAL])
            if key != previous:
                print(f'{seconds:8.4f}  ostate={state:04x} '
                      f'istate={dm[DM_ISTATE]:04x} '
                      f'mode={dm[DM_MODE]:04x} handler={dm[DM_HANDLER]:04x} '
                      f'txptr={dm[DM_TXPTR] & 0x3FFF:04x} '
                      f'serial={dm[DM_SERIAL]:04x} out={sample} '
                      f'vec={"/".join(f"{v:04x}" for v in vector)} '
                      f'pub={"/".join(f"{dm[a]:04x}" for a in PUBLISHED)}',
                      flush=True)
                previous = key

    print(f'\n[trace] page-14 frames: {page14}')
    print('[trace] outer states reached: ' +
          ', '.join(f'{s:04x}@{t:.3f}' for s, t in sorted(first_state.items())))
    if not page14:
        return 0
    print(f'[trace] source vector DM({VECTOR[0]:#06x}..{VECTOR[-1]:#06x}):')
    for slot in range(len(VECTOR)):
        top = slot_values[slot].most_common(4)
        print(f'    slot {slot} DM{VECTOR[slot]:04x}: nonzero '
              f'{nonzero_slot[slot]:6d}/{page14} '
              f'({100.0 * nonzero_slot[slot] / page14:5.1f}%)  '
              + ' '.join(f'{v:04x}={signed(v)}x{n}' for v, n in top))
    print(f'[trace] published copy DM({PUBLISHED[0]:#06x}..{PUBLISHED[-1]:#06x})'
          ' nonzero: ' + ' '.join(f'{s}:{published_nonzero[s]}'
                                  for s in range(len(PUBLISHED))))
    print(f'[trace] DM({DM_SERIAL:#06x}) serializer: ' +
          ' '.join(f'{v:04x}x{n}' for v, n in serial_values.most_common(6)))
    print(f'[trace] DM({DM_MODE:#06x}) dispatch: ' +
          ' '.join(f'{v:04x}x{n}' for v, n in mode_values.most_common(6)))
    print(f'[trace] DM({DM_HANDLER:#06x}) handler source: ' +
          ' '.join(f'{v:04x}x{n}' for v, n in handler_values.most_common(6)))
    print(f'[trace] DM({DM_TXPTR:#06x}) transmit pointer: ' +
          ' '.join(f'{v:04x}x{n}' for v, n in txptr_values.most_common(6)))
    live = sum(n for v, n in out_values.items() if v)
    print(f'[trace] published output: {live}/{page14} nonzero '
          f'({100.0 * live / page14:.1f}%); top ' +
          ' '.join(f'{v}x{n}' for v, n in out_values.most_common(6)))
    print(f'[trace] TX datagrams {card.tx_accepted}/{card.tx_requests} '
          'accepted/requested')
    if args.count and counted_from is not None:
        frames = page14 - counted_from
        print(f'[trace] execution counts over {frames} frames from ostate 0x0080:')
        for pc in args.count:
            n = ADSP.adsp2181_coverage_count(card.cpu, pc)
            print(f'    PM {pc:04x}: {n:8d}  = {n / max(1, frames):7.3f} per frame')
    if dump is not None:
        dump.close()
        print(f'[trace] wrote {args.dump_tx}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
