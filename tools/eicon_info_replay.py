#!/usr/bin/env python3
"""Replay a captured RX G.711 stream through the emulated card, offline.

`tools/eicon_adsp_sip.py --capture-prefix P` writes the peer's received
stream to `P.rx.ulaw`.  Feeding that back through a fresh `LiveKernelModem`
reproduces a live call's state path exactly -- our transmission does not
affect an already-recorded RX -- so any DM word can be instrumented without
dialling the rig again.  The replay is open loop: once an injection changes
what we would have transmitted, the recorded peer cannot respond, so the
path past that point shows what the firmware does, not what the call would
have done.

Two views:

  --states   print the INFO sequencer's control words whenever they change.
             PM 0x3335 counts DM(0x1647) down, calls the pre-condition
             DM(0x169a), then up to four condition handlers DM(0x1696..99),
             takes the first that returns LE and loads the matching next
             state from DM(0x1692..95).
  --tx       summarise transmit activity per TrnProgress state, which is how
             a "we went silent here" window is identified.

  --inject-event STATE
             write DM(0x198e) = 1 the first time TrnProgress reaches STATE.
             DM(0x198e) is the INFO event word, published only by PM 0x2470
             as the index of a matched message code in the table PM 0x2410
             builds at DM(0x1986).  Injecting 1 at state 0x37 takes the
             sequencer's test3 branch (PM 0x2476) instead of the framer-A
             fallback (PM 0x33c4).

Usage:
    python3 tools/eicon_info_replay.py CAPTURE.rx.ulaw --tx
    python3 tools/eicon_info_replay.py CAPTURE.rx.ulaw --states --from 6.1 --to 6.8
    python3 tools/eicon_info_replay.py CAPTURE.rx.ulaw --tx --inject-event 0x37
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from dial_kernel_dispatch import LiveKernelModem

SAMPLE_RATE = 8000
DM_TRNPROGRESS = 0x3FC2
DM_EVENT = 0x198E

# The INFO sequencer's working set, PM 0x3335.
SEQUENCER = {
    'trn': 0x3FC2, 'internal': 0x1652, 'vector': 0x1679, 'entry': 0x169F,
    'next0': 0x1692, 'next1': 0x1693, 'next2': 0x1694, 'next3': 0x1695,
    'test0': 0x1696, 'test1': 0x1697, 'test2': 0x1698, 'test3': 0x1699,
    'pre': 0x169A,
}
# Shown alongside, but excluded from the change key: these move every sample.
EXTRA = {'timer': 0x1647, 'doneA': 0x0686, 'eventB': 0x198E}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('capture', type=Path, help='a .rx.ulaw capture (PCMU)')
    ap.add_argument('--states', action='store_true',
                    help='print sequencer control words on every change')
    ap.add_argument('--tx', action='store_true',
                    help='summarise TX activity per TrnProgress state')
    ap.add_argument('--inject-event', type=lambda v: int(v, 0), metavar='STATE',
                    help='set DM(0x198e) = 1 on first reaching this TrnProgress')
    ap.add_argument('--from', dest='start', type=float, default=0.0)
    ap.add_argument('--to', dest='end', type=float, default=1e9)
    args = ap.parse_args()

    if not args.states and not args.tx:
        args.tx = True

    data = args.capture.read_bytes()
    modem = LiveKernelModem()
    modem.boot()
    modem.configure_modem('answer', 'pcmu')
    dm = modem.card.dm

    previous = None
    injected = False
    activity: dict[int, list[int]] = {}
    order: list[int] = []

    for index, code in enumerate(data):
        seconds = index / SAMPLE_RATE
        if seconds > args.end and not args.tx:
            break
        if (args.inject_event is not None and not injected
                and dm[DM_TRNPROGRESS] == args.inject_event):
            dm[DM_EVENT] = 1
            injected = True
            print(f'{seconds:8.4f}  injected DM(0x198e) = 1 at TrnProgress '
                  f'0x{args.inject_event:04x}')

        sample = modem.frame_fast(code, index)

        state = dm[DM_TRNPROGRESS]
        if state not in activity:
            activity[state] = [0, 0]
            order.append((seconds, state))
        activity[state][0] += 1
        if sample:
            activity[state][1] += 1

        if args.states and args.start <= seconds <= args.end:
            key = tuple(dm[address] for address in SEQUENCER.values())
            if key != previous:
                fields = ' '.join(f'{name}={dm[address]:04x}'
                                  for name, address in SEQUENCER.items())
                extra = ' '.join(f'{name}={dm[address]:04x}'
                                 for name, address in EXTRA.items())
                print(f'{seconds:8.4f}  {fields} | {extra}')
                previous = key

    if args.inject_event is not None and not injected:
        print(f'TrnProgress never reached 0x{args.inject_event:04x}',
              file=sys.stderr)

    if args.tx:
        print('\nTX activity per TrnProgress state, in order of first entry:')
        for seconds, state in order:
            total, live = activity[state]
            print(f'  {seconds:8.4f}s  0x{state:04x}  {total:7d} samples  '
                  f'{100.0 * live / total:5.1f}% non-zero TX')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
