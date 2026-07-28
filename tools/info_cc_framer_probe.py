#!/usr/bin/env python3
"""Drive the INFO overlay's control-channel framers with ideal bit decisions.

The INFO page (download 0x0260) recovers V.34/V.90 phase-2 control-channel
messages in two stages:

  PM 0x34f0   demodulator.  A 16-word circular sample history (DM 0x16bb,
              L0 = 0x10) is correlated against the 16-tap reference at
              DM 0x1554; |MR1| over DM(0x164f) raises the energy flag
              DM(0x0685), and |MR1| over 0x0578 becomes the one-bit decision
              published in DM(0x060f) at PM 0x3515.
  PM 0x3520+  framer A, and PM 0x25ab+ (entered by the JUMP at PM 0x351f)
              framer B.  Both run once per demodulated sample.

Each framer keeps 16 lanes in a circular buffer, one per sample phase of a
16x oversampled bit, and advances one lane per call:

              framer A                    framer B
  state       DM(0x16bd)  (hunt 0x3520)   DM(0x19cf)  (hunt 0x25ab)
  lanes       DM(0x0620..0x062f)          DM(0x1990..0x199f)
  bit planes  DM(0x068c..)                DM(0x19d0..)
  plane ptr   DM(0x0689)                  DM(0x19cc)
  call count  DM(0x068a)                  DM(0x19cd)
  payload     DM(0x1651): 0x0110 or       fixed 0x0080 (8 bits)
              0x01e0 (17 or 30 bits),
              selected by DM(0x3f94) bit 1
  success     DM(0x0686) = 1              DM(0x198e) = event, DM(0x198f)
                                          = the recovered octet

A lane hunts an 11-bit window equal to 0x0772 -- one fill bit followed by the
V.34 INFO synchronisation code 0x372 -- five times, then accumulates CRC-16
(reflected 0x8408, preset 0xffff) over the payload while the bit planes
collect every lane's decision.  The received CRC is then shifted in against
each lane's register, so the lane whose residue is zero is the one that was
sampling on the correct phase; PM 0x3568/0x25e9 scan for it and PM 0x3574/
0x25f5 transpose the bit planes to recover that lane's payload.

This harness bypasses the demodulator and feeds the framers ideal decisions
(each bit repeated over all 16 lanes), which is what an ideal 16x oversampled
control channel looks like at PM 0x3515.  It therefore separates the framer
and the emulated instruction semantics from the demodulator's level-dependent
decision: if a frame validates here but a live call never sets DM(0x0686),
the fault is upstream in DM(0x060f), not in the framer.

Usage:
    python3 tools/info_cc_framer_probe.py [--payload-bits N] [--verbose]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from dial_kernel_dispatch import ADSP, KERNEL_IDLE, LiveKernelModem

INFO_DOWNLOAD = 0x0260
LANES = 16
SYNC_CODE = 0x372          # V34_INFO_SYNC_CODE, matched as 0x0772 with a fill bit
SYNC_BITS = 10
CRC_BITS = 16

PM_FRAMER_A_RESET = 0x359A   # state = 0x3520, lanes = 0x0620, counters cleared
PM_FRAMER_A_LENGTH = 0x3F7F  # DM(0x1651) = 0x0110 / 0x01e0 from DM(0x3f94)
PM_FRAMER_B_INSTALL = 0x2602 # state = 0x25ab, lanes = 0x1990, message table
PM_PUBLISH_BIT = 0x3515      # DM(0x060f) = AR, then run both framers

DM_A_STATE = 0x16BD
DM_A_LANES = 0x16BC
DM_A_COUNT = 0x068A
DM_A_DONE = 0x0686
DM_A_LENGTH = 0x1651
DM_B_STATE = 0x19CF
DM_B_EVENT = 0x198E
DM_B_OCTET = 0x198F
DM_INFO_MODE = 0x3F94


def crc_bit(crc: int, bit: int) -> int:
    """One bit of the CRC-16 the framers compute at PM 0x354e / 0x25cf."""
    if (crc ^ bit) & 1:
        return ((crc >> 1) ^ 0x8408) & 0xFFFF
    return (crc >> 1) & 0xFFFF


def build_frame(payload: list[int], lead_fill: int = 12, tail_fill: int = 8) -> list[int]:
    """Fill, sync, payload, then the CRC LSB-first and uncomplemented.

    PM 0x3561/0x25e2 shifts each received CRC bit in against the register's
    own LSB, so a lane's residue is zero only for that transmission order.
    """
    bits = [1] * lead_fill
    bits += [(SYNC_CODE >> (SYNC_BITS - 1 - i)) & 1 for i in range(SYNC_BITS)]
    crc = 0xFFFF
    for bit in payload:
        crc = crc_bit(crc, bit)
    bits += payload
    bits += [(crc >> i) & 1 for i in range(CRC_BITS)]
    return bits + [1] * tail_fill


def probe(modem, payload: list[int], verbose: bool) -> tuple[int, int, int]:
    """Feed one ideal frame and return (A.done, B.event, B.octet)."""
    dm, cpu = modem.card.dm, modem.card.cpu

    # The page initializer at PM 0x3f4c runs these; call them directly so the
    # probe does not depend on reaching the live INFO entry seam.  It also
    # parks framer B at the disabled handler 0x25f3, so PM 0x2602 is what
    # installs it.
    for entry in (PM_FRAMER_A_RESET, PM_FRAMER_A_LENGTH, PM_FRAMER_B_INSTALL):
        ADSP.adsp2181_call(cpu, entry, KERNEL_IDLE)
        ADSP.adsp2181_run(cpu, 300_000)
        if not ADSP.adsp2181_idle(cpu):
            raise RuntimeError(f'PM {entry:#06x} did not return to IDLE')

    previous = None
    for index, bit in enumerate(build_frame(payload)):
        for _ in range(LANES):
            ADSP.adsp2181_set_ar(cpu, bit)
            ADSP.adsp2181_call(cpu, PM_PUBLISH_BIT, KERNEL_IDLE)
            ADSP.adsp2181_run(cpu, 400_000)
            if not ADSP.adsp2181_idle(cpu):
                raise RuntimeError(f'bit {index}: framer did not return to IDLE')
        state = (dm[DM_A_STATE], dm[DM_A_DONE], dm[DM_B_STATE], dm[DM_B_EVENT])
        if verbose and state != previous:
            print(f'  bit {index:3d} in={bit}  A.state={state[0]:04x} '
                  f'A.done={state[1]:04x} A.count={dm[DM_A_COUNT]:04x} | '
                  f'B.state={state[2]:04x} B.event={state[3]:04x}')
            previous = state
    return dm[DM_A_DONE], dm[DM_B_EVENT], dm[DM_B_OCTET]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--payload-bits', type=int,
                    help='probe one payload length instead of both framers')
    ap.add_argument('--verbose', action='store_true',
                    help='print every framer state change')
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

    # PM 0x3f7f has to run before DM(0x1651) means anything.
    ADSP.adsp2181_call(modem.card.cpu, PM_FRAMER_A_LENGTH, KERNEL_IDLE)
    ADSP.adsp2181_run(modem.card.cpu, 300_000)
    a_bits = dm[DM_A_LENGTH] // LANES
    print(f'DM(0x1651)={dm[DM_A_LENGTH]:#06x} -> framer A takes {a_bits} '
          f'payload bits (INFO_mode DM(0x3f94)={dm[DM_INFO_MODE]:04x}); '
          f'framer B takes 8')

    if args.payload_bits:
        cases = [(args.payload_bits,
                  [(index * 5 + 1) & 1 for index in range(args.payload_bits)])]
    else:
        # 0x30 is the first entry of the message table PM 0x2410 builds at
        # DM(0x1986); framer B publishes an event only for a listed high nibble.
        cases = [(a_bits, [(index * 5 + 1) & 1 for index in range(a_bits)]),
                 (8, [(0x30 >> (7 - index)) & 1 for index in range(8)])]

    failures = 0
    for payload_bits, payload in cases:
        print(f'\n{payload_bits}-bit payload')
        done, event, octet = probe(modem, payload, args.verbose)
        print(f'  framer A: DM(0x0686)={done:04x}   '
              f'framer B: DM(0x198e)={event:04x} DM(0x198f)={octet:04x}')
        if payload_bits == a_bits and done != 1:
            print('  FAIL: framer A did not publish its completion flag')
            failures += 1
        if payload_bits == 8 and event == 0:
            print('  FAIL: framer B did not publish an event')
            failures += 1

    if failures:
        return 1
    print('\nPASS: an ideal control channel completes both framers')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
