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

The completion cannot be put in the ordinary command ring: TIKRNL owns the
per-sample call site while the request is outstanding, so PM 0x02A1 is not
reachable.  The host instead lends that call site to the registered completion
for one SPORT0 slot, then restores TIKRNL's sample continuation.  This is the
same CALL encoding kernel service 0x0017 uses when it initially claims the
slot, and models the IDMA program-memory write available to the real host.

Usage:
    python3 tools/dial_kernel_dispatch.py --dial-v8 --stimulus ansam --samples 16000
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
                     ('watch_dm', [ctypes.c_void_p, ctypes.c_uint16,
                                   ctypes.c_int]),
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
PM_SAMPLE_CONTINUATION = 0x06FC

# TIKRNL.
DM_ENTRIES = 0x31BA          # entry table: slot i answers doorbell bit i
DM_DOWNLOAD_FLAG = 0x31A9
DM_DOWNLOAD_REQ = 0x31AA

# Data-pump write/read databases (ADDSP V.90 guide §5.4.1/§5.4.2).
DM_DB = 0x3EE0
DM_WSTATUS = 0x3EEE
DM_TRNPROG = 0x3FAD
DM_BOOTPAGE = 0x3FB0
V8_DOWNLOAD = 0x025F
FSK_OWN_DOWNLOAD = 0x025C
V_OWN_DOWNLOAD = 0x026D
DM_COUPLED_BUFFER_MODE = 0x32F0
DM_LINE_DESCRIPTOR = 0x3F08
DM_RX_BUFFER_POINTER = 0x3F0F
DM_TX_BUFFER_POINTER = 0x3FB4
DM_RX_BUFFER = 0x2B00
DM_TX_BUFFER = 0x2B01
SAMPLE_RATE = 8000


def make_stimulus(kind: str, samples: int, freq: int, amp: int) -> list[int]:
    """Build G.711 input, including normative V.8 ANSam (§7.2).

    ANSam is a 2100 Hz carrier with a 15 Hz sinusoidal envelope ranging from
    0.8 to 1.2 of average amplitude and 180-degree reversals every 450 ms.
    """
    if kind == 'silence' or not freq:
        return [0xFF] * samples
    result = []
    phase_offset = 0.0
    reversal_samples = int(0.450 * SAMPLE_RATE)
    for i in range(samples):
        if kind == 'ansam' and i and i % reversal_samples == 0:
            phase_offset += math.pi
        envelope = (1.0 + 0.2 * math.sin(2 * math.pi * 15 * i / SAMPLE_RATE)
                    if kind == 'ansam' else 1.0)
        sample = int(amp * envelope
                     * math.sin(2 * math.pi * freq * i / SAMPLE_RATE
                                + phase_offset))
        sample = max(-32768, min(32767, sample))
        result.append(linear_to_mulaw(sample))
    return result


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
            self.tx.append(value & 0xFFFF)

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

    def service(self, index: int) -> collections.Counter:
        """The host half: answer the doorbell, serve overlay downloads."""
        dm = self.card.dm
        hist: collections.Counter = collections.Counter()
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
        wanted = dm[DM_DOWNLOAD_REQ]
        if (dm[DM_DOWNLOAD_FLAG] and wanted != self.card.resident
                and wanted in self.card.overlays):
            description = self.card.download_overlay(wanted)
            # Most overlays need the reconstructed channel pointers restored,
            # but V.8 supplies its own per-line PCM words (0x3763/0x3764).
            if wanted != V8_DOWNLOAD:
                self.assign_pcm_buffers()
            # ADDSP V.90 guide §5.4.1: acknowledge that the requested boot
            # page is resident before dispatching the task's completion.
            dm[DM_WSTATUS] = 0x1000        # BOOTFINISHED; ACTIVATE was a strobe
            if self.log:
                print(f'  sample {index}: served 0x{wanted:04x} {description}')
        if bits & 0x0002:
            hist.update(self.resume(DM_ENTRIES + 1, index))
        return hist

    def assign_pcm_buffers(self) -> None:
        """Restore the one-line PCM pointers overlays may overwrite."""
        dm = self.card.dm
        dm[DM_RX_BUFFER_POINTER] = DM_RX_BUFFER
        dm[DM_TX_BUFFER_POINTER] = DM_TX_BUFFER
        dm[DM_COUPLED_BUFFER_MODE] = 0x0004

    @staticmethod
    def _call_word(target: int) -> int:
        """Encode ADSP-2181 CALL target, as kernel PM 0x0294-0x0298 does."""
        return 0x1C000F | ((target & 0x3FFF) << 4)

    def resume(self, entry_slot: int, index: int) -> collections.Counter:
        """Hand one SPORT0 dispatch to a registered TIKRNL entry.

        TIKRNL has patched PM 0x02B9 to CALL 0x06FC, so a completion queued in
        the byte ring would never reach PM 0x02A1.  The host can write program
        memory over IDMA; use that to install the registered completion for
        exactly one sample and restore the continuation immediately after it
        returns to the kernel.
        """
        entry = self.card.dm[entry_slot]
        if not entry:
            return collections.Counter()
        saved = self.card.pm[PM_FOREGROUND_SLOT]
        self.card.pm[PM_FOREGROUND_SLOT] = self._call_word(entry)
        try:
            hist = self.strobe()
        finally:
            self.card.pm[PM_FOREGROUND_SLOT] = saved
        if self.log:
            print(f'  sample {index}: resumed entry {entry:04x} through '
                  f'PM {PM_FOREGROUND_SLOT:04x}')
        return hist

    def program_v8_call(self, calling: bool) -> None:
        """Run the documented DIAL training setup (guide §5.4.1)."""
        dm = self.card.dm
        # One-time data-pump initialization, ADDSP V.90 guide §5.4.1
        # Tables 12-13. The real driver applies this before its per-call
        # training script; leaving these at overlay residue prevents V.8's
        # answer-delay/generator state from being armed.
        dm[DM_DB + 0x00] = 0x00C4
        dm[DM_DB + 0x07] = 0xF0FD
        dm[DM_DB + 0x08] = 0x0006
        dm[DM_DB + 0x09] = 0x0006
        dm[DM_DB + 0x0A] = 0x00FF
        dm[DM_DB + 0x0B] = 0x0030
        dm[DM_DB + 0x0C] = 0x0000
        dm[DM_DB + 0x2A] = 0x001F
        dm[DM_DB + 0x2B] = 0xFF00
        dm[DM_DB + 0x2C] = 0x0003
        dm[DM_DB + 0x2D] = 0x0003
        # ADDSP V.90 guide §5.4.1, Tables 14-16:
        # 0x048c is calling, 0x0484 is answering; 0x068c is analog loop test.
        dm[DM_DB + 0x01] = 0x048C if calling else 0x0484
        dm[DM_DB + 0x02] = 0x0030
        dm[DM_DB + 0x03] = 0xF0FD
        dm[DM_WSTATUS] = 0x2000
        dm[DM_DB + 0x0F] = 0x0001
        dm[DM_DB + 0x10] = 0x0100
        # Reconstruct the assigned one-word RX/TX buffers. TIKRNL dereferences
        # these pointers in AV/pointer mode; the modem pages themselves use
        # the line words at 0x3F08/0x3F09.
        self.assign_pcm_buffers()

    def load_v8(self, index: int) -> collections.Counter:
        """Perform the host-supervisor DIAL -> V.8 decision (§5.4.2.1)."""
        dm = self.card.dm
        dm[DM_BOOTPAGE] = 6
        dm[DM_DOWNLOAD_FLAG] = 1
        dm[DM_DOWNLOAD_REQ] = V8_DOWNLOAD
        description = self.card.download_overlay(V8_DOWNLOAD)
        if description is None:
            raise RuntimeError('V.8 overlay 0x025f is unavailable')
        dm[DM_WSTATUS] = 0x1000       # BOOTFINISHED
        hist = self.resume(DM_ENTRIES + 1, index)
        if self.log:
            print(f'  sample {index}: supervisor loaded V.8 ({description})')
        return hist


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--samples', type=int, default=600,
                    help='SPORT0 receive slots to strobe')
    ap.add_argument('--freq', type=int, default=2100,
                    help='line tone in Hz (0 = μ-law silence)')
    ap.add_argument('--amp', type=int, default=20000)
    ap.add_argument('--stimulus', choices=('tone', 'ansam', 'silence'),
                    default='tone',
                    help='receive stimulus; ansam follows ITU-T V.8 §7.2')
    ap.add_argument('--log', action='store_true')
    ap.add_argument('--dial-v8', action='store_true',
                    help='program DIAL calling mode and perform the documented '
                         'host-supervisor transition to V.8')
    ap.add_argument('--answering', action='store_true',
                    help='use answer-mode GEN_SETUP1 (normally transmit ANSam)')
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

    tone = make_stimulus(args.stimulus, args.samples, args.freq, args.amp)

    # The one and only thing the host hands the kernel by hand.
    card_driver.push(TASK_ENTRY)
    print(f'[host] queued the task entry PM {TASK_ENTRY:04x} as a host command')

    totals: collections.Counter = collections.Counter()
    registered_at = None
    v8_loaded = False
    v8_tx_start = None
    norm_was_set = False
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
            # The ADDSP page is layered. DIAL deliberately calls PM beyond
            # the end of its own image (notably 0x244C and 0x2C4F); those
            # routines remain from V.own and FSK own, loaded underneath it.
            # Starting from DIAL alone leaves those addresses unpopulated and
            # skips/crashes the full NORM initialization path.
            for base_id in (V_OWN_DOWNLOAD, FSK_OWN_DOWNLOAD):
                base_entry = card.overlays.get(base_id)
                if base_entry is None:
                    raise RuntimeError(f'missing base overlay 0x{base_id:04x}')
                card._download(base_entry[0])
            card.download_overlay(DIAL_ID)
            print(f'       DIAL downloaded (0x{DIAL_ID:04x})')
            if args.dial_v8:
                card_driver.program_v8_call(calling=not args.answering)
                norm_was_set = True
                print(f'[host] DIAL {"answering" if args.answering else "calling"} '
                      'setup programmed '
                      f'(GEN_SETUP1={dm[DM_DB + 1]:04x}, V.8/V.34 enabled)')
                totals.update(card._run(0x08F1, 200_000))
                # Channel activation enters DIAL's NORM setup at 0x13CC
                # before the supervisor permits page 6. Run that entry only
                # after the underlying "own" images are resident.
                totals.update(card._run(0x13CC, 1_000_000))
                print('[host] DIAL NORM initialization entry 0x13cc completed')
            continue
        if registered_at is not None:
            totals.update(card_driver.service(index))
            if args.dial_v8 and card.resident == V8_DOWNLOAD and not v8_loaded:
                v8_loaded = True
                if args.log:
                    for address in (0x3764, 0x3995, 0x3996, 0x3F4C, 0x3FC1,
                                    0x3FC2):
                        ADSP.adsp2181_watch_dm(card.cpu, address, 1)
                # The reconstructed single line must remain selected by the
                # kernel's line dispatcher after the completion returns.
                dm[DM_LINE_DESCRIPTOR] |= 0x0020
                totals.update(card._run(dm[DM_ENTRIES], 200000))
                v8_tx_start = len(card_driver.tx)
                print(f'[host] sample {index}: TIKRNL selected V.8; '
                      'overlay 0x025f loaded, line assigned and resumed')
            if (args.dial_v8 and norm_was_set and not v8_loaded
                    and not (dm[DM_DB + 1] & 0x0080)):
                totals.update(card_driver.load_v8(index))
                v8_loaded = True
                v8_tx_start = len(card_driver.tx)
                print(f'[host] sample {index}: DIAL consumed NORM; '
                      'V.8 overlay 0x025f loaded and resumed')

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
    if args.dial_v8:
        print('[card] V.8 transition: loaded=%s resident=0x%04x '
              'entry08f0=%d entry08f1=%d action204a=%d'
              % (v8_loaded, card.resident, totals.get(0x08F0, 0),
                 totals.get(0x08F1, 0), totals.get(0x204A, 0)))
    print('[card] doorbell bits: '
          + (' '.join(f'bit{b}={n}' for b, n in sorted(card_driver.doorbell.items()))
             or 'none'))
    if card.served:
        print('[card] overlays served: '
              + ' '.join(f'0x{k:04x} x{v}' for k, v in card.served.most_common()))
    print(f'[card] DM 3F08={dm[0x3F08]:04x} 3F09={dm[0x3F09]:04x} '
          f'3FB0={dm[0x3FB0]:04x} 3FC1={dm[0x3FC1]:04x} '
          f'3FC2={dm[0x3FC2]:04x} 3995={dm[0x3995]:04x} '
          f'3996={dm[0x3996]:04x} '
          f'31A9={dm[DM_DOWNLOAD_FLAG]:04x} 31AA={dm[DM_DOWNLOAD_REQ]:04x}')
    print(f'[card] PCM buffers: RXptr={dm[DM_RX_BUFFER_POINTER]:04x} '
          f'RX={dm[DM_RX_BUFFER]:04x} TXptr={dm[DM_TX_BUFFER_POINTER]:04x} '
          f'TX={dm[DM_TX_BUFFER]:04x}')
    if args.dial_v8:
        post_v8 = (card_driver.tx[v8_tx_start:]
                   if v8_tx_start is not None else [])
        counts = collections.Counter(post_v8)
        response_words = sum(value not in (0x0000, 0x00ff, 0x0400)
                             for value in post_v8)
        print(f'[card] SPORT0 TX after V.8 activation: words={len(post_v8)} '
              f'unique={len(set(post_v8))} '
              f'response={response_words} '
              f'top={",".join(f"{value:04x}:{count}" for value, count in counts.most_common(8))} '
              f'first16={" ".join(f"{value:04x}" for value in post_v8[:16])}')

    undelivered = len(card_driver.commands) - totals.get(PM_DISPATCH, 0)
    if undelivered > 0:
        print(f'[card] ERROR: {undelivered} ordinary host command(s) remain in '
              'the ring; overlay completions must use the registered one-shot '
              'resume path, not this ring')
    if args.dial_v8 and (not v8_loaded or not totals.get(0x08F0)):
        print('[card] ERROR: DIAL did not enter the V.8 overlay through PM 08f0')
        return 1
    if args.dial_v8 and response_words == 0:
        print('[card] ERROR: V.8 ran but emitted no response on the B-channel')
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
