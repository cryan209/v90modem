#!/usr/bin/env python3
"""Decode the INFO overlay's state records and its state-vector table.

The INFO page is table driven.  PM 0x336a (installed as DM(0x169f)) reads a
state record from DM as `(w1, w2, w3)` triples, writing
`(w2 & 0xff) | ((w3 & 0xff) << 8)` to `DM(0x1642 + (w1 & 0xff))` and stopping
when the offset equals 0x19.  A record therefore sets only DM(0x1642..0x165a):
the raw per-state fields, plus the candidate/condition indices at
DM(0x1653..0x165b) that PM 0x3329..0x3332 translate through DM(0x133e) and
DM(0x131e) into DM(0x1692..0x1695) and DM(0x1696..0x169a).

Two of those raw fields drive everything this tool is for:

  DM(0x164c)  a bitmask dispatched by PM 0x3477 against the PM action table
              at 0x2ee6..0x2eee -- bit N runs action N (0 = PM 0x2410 build
              the message table, 1 = PM 0x2602 install framer B, 3..8 =
              transmit an 8-bit control-channel message).
  DM(0x1651)  framer A's payload length.  PM 0x3583 only lets framer A reach
              the classifier PM 0x2461 -- the sole publisher of the event
              word DM(0x198e) -- when this is 0x0080.

DM(0x133e) is the state-vector table: a candidate index in a record is looked
up there to get the next record's DM address.  Records also simply run on
into the next one, so a chain of consecutive addresses is one state sequence.

Usage:
    python3 tools/info_state_records.py            # table + records with actions
    python3 tools/info_state_records.py --all      # every table entry
    python3 tools/info_state_records.py --record 0x1736
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from dial_kernel_dispatch import LiveKernelModem

INFO_DOWNLOAD = 0x0260
RECORD_BASE = 0x1642
RECORD_END = 0x19
VECTOR_TABLE = 0x133E
VECTOR_TABLE_LEN = 0x40

FIELDS = {0x02: 'script(0x1644)', 0x05: 'timer(0x1647)', 0x0A: 'actions(0x164c)',
          0x0F: 'length(0x1651)', 0x10: 'state(0x1652)',
          0x11: 'next0', 0x12: 'next1', 0x13: 'next2', 0x14: 'next3',
          0x15: 'test0', 0x16: 'test1', 0x17: 'test2', 0x18: 'test3'}

ACTIONS = {0: 'PM 0x2410 build message table',
           1: 'PM 0x2602 INSTALL FRAMER B',
           2: 'PM 0x242b clear DM(0x3f4b) bit 0x80',
           3: 'PM 0x2430 transmit message 0',
           4: 'PM 0x243d transmit message 3',
           5: 'PM 0x2441 transmit message 5',
           6: 'PM 0x243f transmit message 4',
           7: 'PM 0x2434 transmit message 1',
           8: 'PM 0x243b transmit message 2'}


def decode(dm, address: int, limit: int = 48) -> dict[int, int]:
    """Replay PM 0x336a over one record."""
    out: dict[int, int] = {}
    for _ in range(limit):
        offset = dm[address] & 0xFF
        if offset == RECORD_END:
            break
        out[offset] = (dm[address + 1] & 0xFF) | ((dm[address + 2] & 0xFF) << 8)
        address += 3
    return out


def describe(record: dict[int, int]) -> str:
    return ' '.join(f'{FIELDS.get(offset, hex(RECORD_BASE + offset))}={value:04x}'
                    for offset, value in sorted(record.items()))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--all', action='store_true',
                    help='print every state-vector table entry')
    ap.add_argument('--record', type=lambda v: int(v, 0),
                    help='decode a single record at this DM address')
    args = ap.parse_args()

    modem = LiveKernelModem()
    modem.boot()
    modem.configure_modem('answer', 'pcmu')
    description = modem.card.download_overlay(INFO_DOWNLOAD)
    if description is None:
        print('INFO overlay 0x0260 is unavailable', file=sys.stderr)
        return 1
    dm = modem.card.dm
    print(description)

    if args.record is not None:
        print(f'record @{args.record:04x}: {describe(decode(dm, args.record))}')
        return 0

    print(f'\nstate-vector table DM({VECTOR_TABLE:#06x}):')
    dispatchers = []
    for index in range(VECTOR_TABLE_LEN):
        address = dm[VECTOR_TABLE + index]
        record = decode(dm, address) if 0x0600 <= address <= 0x1F00 else {}
        state = record.get(0x10)
        actions = record.get(0x0A, 0)
        length = record.get(0x0F)
        if not args.all and state is None and not actions:
            continue
        print(f'  {index:02x} -> {address:04x}  '
              f'state={"----" if state is None else format(state, "04x")}  '
              f'actions={actions:04x}  '
              f'length={"----" if length is None else format(length, "04x")}')
        if actions:
            dispatchers.append((index, address, state, actions))

    print('\nrecords that dispatch actions:')
    for index, address, state, actions in dispatchers:
        names = [ACTIONS[bit] for bit in range(9) if actions >> bit & 1 and bit in ACTIONS]
        label = '----' if state is None else format(state, '04x')
        print(f'  index {index:02x}  state {label} @{address:04x}  '
              f'DM(0x164c)={actions:04x}')
        for name in names:
            print(f'      {name}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
