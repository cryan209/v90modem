#!/usr/bin/env python3
"""Let the PRI kernel dispatch the TIKRNL task itself, off SPORT0.

`tools/dial_tikrnl_drive.py` calls the task's frame entry (PM 0x06BB) by hand.
This one never does: it boots the kernel, hands it one host command, and from
then on the kernel's own foreground calls the task on every PCM sample.

## What DM 0x2F28 actually is

The kernel foreground wakes from IDLE when the SPORT0 ISR has queued a sample
(DM 0x2E44 != DM 0x2E45), and its first act is to write five pointers:

    PM 02ad  I0 = $2F27
    PM 02ae  DM(I0,M1) = $2F21     ; task registration block
    PM 02af  DM(I0,M1) = $2F00     ; host -> DSP command ring descriptor
    PM 02b0  DM(I0,M1) = $2F0E     ; DSP -> host descriptor + doorbell
    PM 02b1  DM(I0,M1) = $2F42
    PM 02b2  DM(I0,M1) = $2F4E

so DM 0x2F28 is not a value to invent -- it points at a ring descriptor the
kernel image already carries.  (The five constants read as 0x0F21/0x0F00/...
until the disassembler's immediate field was widened to the 16 bits the core
decodes; DM 0x0F00 is unpopulated scratch, which is why the list looked
permanently empty.)

The descriptor at DM 0x2F00, as PM 0x00D8 walks it:

    +0  scratch          +4  read pointer
    +1  -> AX1           +5  ring base   (0x2800)
    +2  -> AY0           +6  ring size   (0x0300 words)
    +3  byte count, signed: 0 = empty, > 0 = the next byte is the low half of
        the word at +4, < 0 = the high half.  Each read flips the sign and
        decrements the magnitude; the negative case also advances +4, wrapping
        at base+size.

PM 0x01B2 reads two bytes and assembles them little-endian; PM 0x02A1 loads
that into I4 and calls it.  The host command language is literally a stream of
PM addresses to call.  So one 16-bit word in the ring is one command.

## Bootstrapping the task

Push TIKRNL's task entry (PM 0x0672) as the first command.  Its init calls
kernel service 0x0017 (PM 0x0281), which builds a `CALL <vector>` instruction
and patches it over two words named by the registration block at DM 0x2F21:

    DM 2f24 = 0x02b9 -> PM 02b9: CALL $02A1        -> CALL $06FC
    DM 2f25 = 0x00b5 -> PM 00b5: AR = SR0 + 0, ... -> CALL $08F6

PM 0x02B9 is the foreground's per-sample dispatch and PM 0x00B5 is inside the
SPORT0 ISR, so after that one command the kernel runs the task itself on every
sample, with the codeword in SR1 on the way in and the task's reply in SR1 on
the way out (PM 0x02B8 / 0x02BC).  PM 0x08F6 lands on TIKRNL's own copy of the
instruction it displaced -- the task overwrites the kernel's PM 0x0580-0x05EB
block, and its 0x0582 is `AR = SR0 + 0, SR0 = AR` again.

The task asks for service the other way by toggling bits in DM 0x2F17
(= DM(0x2F29) + 9) and raising FLAG_OUT -- PM 0x01DE, reached by `JUMP $000A`
with the bit in AR.  Bit i means "call my entry table slot i", DM 0x31BA + i:
slot 0 = 0x06BB (frame entry), slot 1 = 0x06D8 (an overlay download landed).

Usage:
    python3 tools/dial_kernel_dispatch.py --freq 2100 --samples 600
"""
from __future__ import annotations

import argparse
import collections
import ctypes
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from dial_tikrnl_drive import (ADSP, DIAL_ID, KERNEL, TASK_ENTRY, TIKRNL,
                               Card, linear_to_mulaw)

for _name, _args in [('set_callbacks', [ctypes.c_void_p] * 4),
                     ('set_irq', [ctypes.c_void_p, ctypes.c_int, ctypes.c_int]),
                     ('imask', [ctypes.c_void_p])]:
    getattr(ADSP, 'adsp2181_' + _name).argtypes = _args
ADSP.adsp2181_imask.restype = ctypes.c_uint16

RX_CB = ctypes.CFUNCTYPE(ctypes.c_int32, ctypes.c_void_p, ctypes.c_int)
TX_CB = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_int, ctypes.c_int32)
TIM_CB = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_int)

SPORT0_RX = 3

# Kernel scheduler.
DM_QUEUE_HEAD = 0x2E44       # SPORT0 ISR write pointer into the sample queue
DM_QUEUE_TAIL = 0x2E45       # foreground read pointer
DM_RING_PTRS = 0x2F27        # the five descriptor pointers, written at PM 0x02AD
DM_CMD_DESC = 0x2F28         # -> the host -> DSP command ring descriptor
DM_DOORBELL = 0x2F17         # DM(DM(0x2F29) + 9): DSP -> host service bits
PM_FOREGROUND_SLOT = 0x02B9  # CALL $02A1, or the task's continuation
PM_ISR_SLOT = 0x00B5         # the ISR word a task may claim
PM_DISPATCH = 0x02A1         # read one command from the ring and call it

# TIKRNL.
DM_ENTRIES = 0x31BA          # entry table: slot i answers doorbell bit i
DM_DOWNLOAD_FLAG = 0x31A9
DM_DOWNLOAD_REQ = 0x31AA


class KernelDispatch:
    """Kernel + task, driven only through SPORT0 and the command ring."""

    def __init__(self, log: bool = False):
        self.card = Card(serve=False)
        self.log = log
        self.sample = 0xFF
        self.tx: list[int] = []
        self.doorbell: collections.Counter = collections.Counter()
        self.commands: list[int] = []
        self.started = False
        # Keep the ctypes trampolines alive for the life of the object.
        self._cbs = (RX_CB(self._rx), TX_CB(self._tx), TIM_CB(self._timer))

    # --- SPORT0 -----------------------------------------------------------
    def _rx(self, cpu, port):
        return self.sample if port == 0 else 0

    def _tx(self, cpu, port, value):
        if port == 0:
            self.tx.append(value & 0xFF)

    def _timer(self, cpu, enabled):
        pass

    # --- the host side of the command ring --------------------------------
    def push(self, vector: int) -> bool:
        """Queue one command: a PM address for the foreground to call."""
        dm = self.card.dm
        desc = dm[DM_CMD_DESC]
        if not desc or dm[desc + 3] != 0:
            return False          # not drained; one command at a time
        dm[dm[desc + 4]] = vector & 0xFFFF
        dm[desc + 3] = 2          # two bytes, low half of the word first
        self.commands.append(vector)
        return True

    def boot(self) -> None:
        card = self.card
        card._download(KERNEL)
        ADSP.adsp2181_run(card.cpu, 5000)
        if not ADSP.adsp2181_idle(card.cpu):
            raise RuntimeError('kernel did not reach its idle loop')
        ADSP.adsp2181_set_callbacks(card.cpu, *self._cbs)
        # The host downloads the task image, then commands the kernel to run
        # its entry.  Nothing else is called by hand from here on.
        card._download(TIKRNL)
        self.pending_entry = TASK_ENTRY

    def strobe(self, budget: int = 300000) -> collections.Counter:
        """One SPORT0 receive slot: interrupt, then run to IDLE."""
        card = self.card
        hist: collections.Counter = collections.Counter()
        ADSP.adsp2181_set_irq(card.cpu, SPORT0_RX, 1)
        ADSP.adsp2181_set_irq(card.cpu, SPORT0_RX, 0)
        for _ in range(budget):
            hist[ADSP.adsp2181_pc(card.cpu)] += 1
            ADSP.adsp2181_run(card.cpu, 1)
            if ADSP.adsp2181_idle(card.cpu):
                break
        return hist

    def service(self, index: int) -> None:
        """The host half: answer the doorbell, serve overlay downloads."""
        dm = self.card.dm
        bits = dm[DM_DOORBELL]
        if bits:
            dm[DM_DOORBELL] = 0
            for bit in range(16):
                if bits & (1 << bit):
                    self.doorbell[bit] += 1
                    entry = dm[DM_ENTRIES + bit]
                    if entry and self.log:
                        print(f'  sample {index}: doorbell bit {bit} -> '
                              f'entry {entry:04x}')
                    if entry:
                        self.push(entry)
        wanted = dm[DM_DOWNLOAD_REQ]
        if (dm[DM_DOWNLOAD_FLAG] and wanted != self.card.resident
                and wanted in self.card.overlays):
            description = self.card.download_overlay(wanted)
            if self.log:
                print(f'  sample {index}: served 0x{wanted:04x} {description}')


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--samples', type=int, default=600,
                    help='SPORT0 receive slots to strobe')
    ap.add_argument('--freq', type=int, default=2100,
                    help='line tone in Hz (0 = μ-law silence)')
    ap.add_argument('--amp', type=int, default=20000)
    ap.add_argument('--log', action='store_true')
    args = ap.parse_args()

    card_driver = KernelDispatch(log=args.log)
    card_driver.boot()
    card = card_driver.card
    dm, pm = card.dm, card.pm
    print(f'[card] kernel booted to IDLE, TIKRNL image downloaded; '
          f'imask=0x{ADSP.adsp2181_imask(card.cpu):03x}')
    print(f'[card] before the foreground runs: DM 2f27..2f2b = '
          + ' '.join(f'{dm[a]:04x}' for a in range(DM_RING_PTRS, DM_RING_PTRS + 5)))

    # The pointer block is empty until the foreground wakes, and the foreground
    # only wakes once the SPORT0 ISR has queued a sample -- which takes a few
    # receive slots, because the ISR walks the DM 0x2E50 timeslot countdown
    # first.  It is the kernel, not this harness, that writes DM 0x2F28.
    for primed in range(1, 33):
        card_driver.strobe()
        if dm[DM_CMD_DESC]:
            break
    else:
        raise SystemExit('the foreground never initialised DM 0x2F28')
    print(f'[card] after {primed} SPORT0 slots:     DM 2f27..2f2b = '
          + ' '.join(f'{dm[a]:04x}' for a in range(DM_RING_PTRS, DM_RING_PTRS + 5))
          + '  (written by PM 0x02AD, not by us)')
    desc = dm[DM_CMD_DESC]
    print(f'[card] command ring descriptor DM {desc:04x}: count={dm[desc + 3]:04x} '
          f'ptr={dm[desc + 4]:04x} base={dm[desc + 5]:04x} size={dm[desc + 6]:04x}')

    if args.freq:
        tone = [linear_to_mulaw(int(args.amp * math.sin(2 * math.pi * args.freq * i / 8000)))
                for i in range(8000)]
    else:
        tone = [0xFF] * 8000

    # The one and only thing the host hands the kernel by hand.
    card_driver.push(TASK_ENTRY)
    print(f'[host] queued the task entry PM {TASK_ENTRY:04x} as a host command')

    totals: collections.Counter = collections.Counter()
    registered_at = None
    for index in range(args.samples):
        card_driver.sample = tone[index % len(tone)]
        totals.update(card_driver.strobe())
        if registered_at is None and totals.get(TASK_ENTRY):
            registered_at = index
            print(f'[card] sample {index}: the kernel dispatched PM '
                  f'{TASK_ENTRY:04x} from the ring')
            print(f'       DM 2f27..2f2b = '
                  + ' '.join(f'{dm[a]:04x}'
                             for a in range(DM_RING_PTRS, DM_RING_PTRS + 5)))
            print(f'       PM {PM_FOREGROUND_SLOT:04x} = '
                  f'{pm[PM_FOREGROUND_SLOT] & 0xFFFFFF:06x}   '
                  f'PM {PM_ISR_SLOT:04x} = {pm[PM_ISR_SLOT] & 0xFFFFFF:06x}')
            # The task init clears PM 0x0900 upward, so the boot page can only
            # be downloaded now -- same ordering the host driver uses.
            card.download_overlay(DIAL_ID)
            print(f'       DIAL downloaded (0x{DIAL_ID:04x})')
            continue
        if registered_at is not None:
            card_driver.service(index)

    print(f'[card] {args.samples} SPORT0 slots; queue DM 2e44={dm[DM_QUEUE_HEAD]:04x} '
          f'2e45={dm[DM_QUEUE_TAIL]:04x}')
    print(f'[card] host commands issued: {len(card_driver.commands)} '
          + ' '.join(f'{c:04x}' for c in card_driver.commands[:8]))
    print(f'[card] kernel ring dispatch PM {PM_DISPATCH:04x}: '
          f'{totals.get(PM_DISPATCH, 0)}   task continuation PM 06fc: '
          f'{totals.get(0x06FC, 0)}')
    print('[card] task entries reached: '
          + ' '.join(f'{a:04x}={totals.get(a, 0)}'
                     for a in (TASK_ENTRY, 0x06BB, 0x06D8, 0x08F0, 0x08F1)))
    print('[card] DIAL: line handler 1bbd=%d state dispatcher 1b9c=%d'
          % (totals.get(0x1BBD, 0), totals.get(0x1B9C, 0)))
    print('[card] doorbell bits: '
          + (' '.join(f'bit{b}={n}' for b, n in sorted(card_driver.doorbell.items()))
             or 'none'))
    if card.served:
        print('[card] overlays served: '
              + ' '.join(f'0x{k:04x} x{v}' for k, v in card.served.most_common()))
    print(f'[card] DM 3F08={dm[0x3F08]:04x} 3F09={dm[0x3F09]:04x} '
          f'3FB0={dm[0x3FB0]:04x} 3FC1={dm[0x3FC1]:04x} '
          f'31A9={dm[DM_DOWNLOAD_FLAG]:04x} 31AA={dm[DM_DOWNLOAD_REQ]:04x}')

    undelivered = len(card_driver.commands) - totals.get(PM_DISPATCH, 0)
    if undelivered > 0:
        print(f'[card] {undelivered} host command(s) still in the ring: claiming '
              f'the foreground slot took PM {PM_DISPATCH:04x} out of the loop, and '
              'the only other reader is the task\'s own frame head PM 06bb, which '
              'a pending download stops it reaching (PM 0x0705-0x0706 parks on '
              'DM 0x31A9).  How the host hands a resume back in this mode is '
              'still open.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
