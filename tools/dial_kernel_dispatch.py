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

from dial_tikrnl_drive import (ADSP, DIAL_ID, KERNEL, KERNEL_IDLE,
                               TASK_ENTRY, TIKRNL, Card, linear_to_mulaw)

for _name, _args in [('set_callbacks', [ctypes.c_void_p] * 4),
                     ('set_irq', [ctypes.c_void_p, ctypes.c_int, ctypes.c_int]),
                     ('sport0_tdm_frame', [ctypes.c_void_p, ctypes.c_int,
                                           ctypes.c_int, ctypes.c_uint16,
                                           ctypes.c_uint16, ctypes.c_int]),
                     ('watch_dm', [ctypes.c_void_p, ctypes.c_uint16,
                                   ctypes.c_int]),
                     ('imask', [ctypes.c_void_p])]:
    getattr(ADSP, 'adsp2181_' + _name).argtypes = _args
ADSP.adsp2181_imask.restype = ctypes.c_uint16
ADSP.adsp2181_sport0_tdm_frame.restype = ctypes.c_uint16

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
DM_READ_DB = 0x3F60
DM_EVENT_STRUCT_PTR = DM_DB + 0x90  # guide §6.3: 8 words + eventcounter
DM_CHANGE_BITS = DM_DB + 0xC1
DM_RSTATUS_CH_DBS = DM_DB + 0xC3
DM_RSTATUS_DBS = DM_DB + 0xC4
DM_TRNPROG_DBS = DM_DB + 0xC5
DM_WSTATUS = 0x3EEE
DM_DI_CONTROL = 0x3FAD
DM_LIVE_TRNPROG = 0x3FC2
DM_BOOTPAGE = 0x3FB0
V8_DOWNLOAD = 0x025F
FSK_OWN_DOWNLOAD = 0x025C
V_OWN_DOWNLOAD = 0x026D
DM_COUPLED_BUFFER_MODE = 0x32F0
DM_LINE_RX = 0x3F08
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
        self.events: collections.Counter = collections.Counter()
        self.host_event_counter: int | None = None
        self.published_rstatus_ch = 0
        self.published_rstatus = 0
        self.published_trnprogress = 0
        self.rx2400_phase = 0
        self.rx2400_last: tuple[int, int, int] | None = None
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

    def strobe_fast(self, budget: int = 300000) -> None:
        """One SPORT0 slot without per-instruction Python tracing."""
        card = self.card
        ADSP.adsp2181_set_irq(card.cpu, SPORT0_RX, 1)
        ADSP.adsp2181_set_irq(card.cpu, SPORT0_RX, 0)
        ADSP.adsp2181_run(card.cpu, budget)
        if not ADSP.adsp2181_idle(card.cpu):
            raise RuntimeError('kernel SPORT0 dispatch did not return to IDLE')

    def tdm_frame(self, active_word: int, active_slot: int = 0,
                  dispatch_slot: int | None = None, idle_word: int = 0x00ff,
                  budget: int = 300000) -> int:
        """Drive one 8 kHz PRI frame, dispatching this task on one timeslot."""
        if dispatch_slot is None:
            dispatch_slot = active_slot
        return ADSP.adsp2181_sport0_tdm_frame(
            self.card.cpu, active_slot, dispatch_slot, active_word & 0xffff,
            idle_word & 0xffff, budget)

    def produce_rx2400(self) -> None:
        """Reconstruct the missing Host-Kernel RX_2400 publication cycle.

        The closed host schedules this at 2400 Hz.  TIKRNL's live status words
        exist in the emulation, but its MIPS-facing cycle is absent, leaving
        the documented `_dbs` mirrors and event ring empty.  Guide §5.3.2
        defines this operation completely: copy changed status, set C1 bits
        F/E/D for this cycle, and publish event codes 1/2/3 (§5.5).
        """
        self.rx2400_phase += 2400
        if self.rx2400_phase < SAMPLE_RATE:
            return
        self.rx2400_phase -= SAMPLE_RATE
        dm = self.card.dm
        current = (dm[0x3FC0], dm[0x3FC1], dm[0x3FC2])
        previous = self.rx2400_last
        self.rx2400_last = current
        pointer = dm[DM_EVENT_STRUCT_PTR] & 0x3fff
        change = dm[DM_CHANGE_BITS] & 0x000f
        if previous is None:
            previous = tuple(value ^ 0xffff for value in current)
        publications = []
        for value, old, mirror, flag, event in zip(
                current, previous,
                (DM_RSTATUS_CH_DBS, DM_RSTATUS_DBS, DM_TRNPROG_DBS),
                (0x8000, 0x4000, 0x2000), (1, 2, 3)):
            if value != old:
                dm[mirror] = value
                change |= flag
                publications.append(event)
        dm[DM_CHANGE_BITS] = change
        if not publications or not pointer or pointer + 8 >= 0x4000:
            return
        counter = dm[pointer + 8]
        for event in publications:
            dm[pointer + (counter & 7)] = event
            counter = (counter + 1) & 0xffff
        dm[pointer + 8] = counter

    def poll_events(self, index: int) -> None:
        """Consume the documented DSP-to-host event ring.

        ADDSP guide §§5.1.3.1, 5.5 and 6.3 specify an eight-word ring followed
        by a monotonically wrapping event counter.  The host keeps its own
        counter; reading an event requires no DSP acknowledgement.
        """
        dm = self.card.dm
        pointer = dm[DM_EVENT_STRUCT_PTR] & 0x3fff
        if not pointer or pointer + 8 >= 0x4000:
            return
        dsp_counter = dm[pointer + 8]
        if self.host_event_counter is None:
            # Calls start with a fresh event structure.  If firmware residue
            # says otherwise, retain the newest eight events as required by
            # the guide rather than interpreting overwritten ring entries.
            self.host_event_counter = max(0, dsp_counter - 8)
        pending = (dsp_counter - self.host_event_counter) & 0xffff
        if pending > 8:
            if self.log:
                print(f'  sample {index}: lost {pending - 8} DSP events')
            self.host_event_counter = (dsp_counter - 8) & 0xffff
            pending = 8
        for _ in range(pending):
            event = dm[pointer + (self.host_event_counter & 7)]
            self.host_event_counter = (self.host_event_counter + 1) & 0xffff
            self.events[event] += 1
            if event == 0x0001:
                self.published_rstatus_ch = dm[DM_RSTATUS_CH_DBS]
            elif event == 0x0002:
                self.published_rstatus = dm[DM_RSTATUS_DBS]
            elif event == 0x0003:
                self.published_trnprogress = dm[DM_TRNPROG_DBS]
            if self.log and event in (0x0001, 0x0002, 0x0003):
                print(f'  sample {index}: data-pump event {event:04x}; '
                      f'change={dm[DM_CHANGE_BITS]:04x} '
                      f'status={self.published_rstatus_ch:04x}/'
                      f'{self.published_rstatus:04x} '
                      f'trn={self.published_trnprogress:04x}')

    def service(self, index: int, fast: bool = False) -> collections.Counter:
        """The host half: consume events, answer doorbells and serve downloads."""
        dm = self.card.dm
        hist: collections.Counter = collections.Counter()
        self.produce_rx2400()
        self.poll_events(index)
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
            hist.update(self.resume(DM_ENTRIES + 1, index, fast=fast))
            # BOOTFINISHED is a one-communication-cycle acknowledgement, not
            # persistent modem configuration. The MIPS host clears it after
            # dispatching the registered completion.
            dm[DM_WSTATUS] &= ~0x1000
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

    def resume(self, entry_slot: int, index: int,
               fast: bool = False) -> collections.Counter:
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
            if fast:
                ADSP.adsp2181_call(self.card.cpu, entry, KERNEL_IDLE)
                ADSP.adsp2181_run(self.card.cpu, 300_000)
                if not ADSP.adsp2181_idle(self.card.cpu):
                    raise RuntimeError('overlay completion did not return to IDLE')
                hist = collections.Counter()
            else:
                hist = self.strobe()
        finally:
            self.card.pm[PM_FOREGROUND_SLOT] = saved
        if self.log:
            print(f'  sample {index}: resumed entry {entry:04x} through '
                  f'PM {PM_FOREGROUND_SLOT:04x}')
        return hist

    def program_initial_setup(self, delay_correction: int = 0x000C) -> None:
        """Write the power-up setup, ADDSP guide §5.4.1 Table 12."""
        dm = self.card.dm
        dm[DM_DB + 0x00] = 0x00C4
        dm[DM_DB + 0x01] = 0x0040
        dm[DM_DB + 0x02] = 0x0000
        dm[DM_DB + 0x03] = 0x0000  # DISP_setup, guide §5.3.1 default
        dm[DM_DB + 0x07] = 0xF0FD  # INFO0_setup
        dm[DM_DB + 0x08] = 0x0006
        dm[DM_DB + 0x09] = 0x0006
        dm[DM_DB + 0x0A] = 0x00FF
        dm[DM_DB + 0x0B] = 0x0030
        dm[DM_DB + 0x0C] = 0x0000
        # Guide §5.3.1 requires platform calibration of supplementary kernel
        # buffering.  Build 117-926's Eicon image defaults to 12; write it
        # explicitly instead of inheriting whichever overlay occupied +0x24.
        dm[DM_DB + 0x24] = delay_correction
        # Do not write generic guide SPORT0 setup words +0x70..+0x74 here.
        # Eicon's PRI kernel 0x0009 owns DM(0x3ff6..0x3ffa) through its private
        # channel descriptors; neither it nor TIKRNL reads DB +0x70..+0x74.
        # adsp2181_sport0_tdm_frame models that descriptor selection directly.
        dm[DM_DB + 0x2C] = 0x0003
        dm[DM_DB + 0x2D] = 0x0003
        dm[DM_WSTATUS] = 0x2000

    def program_v8_call(self, calling: bool) -> None:
        """Write Tables 13 and 15 after Table 12 has been consumed."""
        dm = self.card.dm
        dm[DM_DB + 0x2A] = 0x001F
        dm[DM_DB + 0x2B] = 0xFF00
        # ADDSP V.90 guide §5.4.1 Tables 14-15.  This must be a second write
        # communication cycle; combining it with Table 12 prevents the DSP
        # from ever observing initial GEN_SETUP1/2 = 0x0040/0x0000.
        dm[DM_DB + 0x01] = 0x048C if calling else 0x0484
        dm[DM_DB + 0x02] = 0x0030
        # Digital-side V.90 call identity. Without these CAI-equivalent
        # fields V.8 completes correctly but clears its PCM capability bit and
        # deliberately selects page 1/V.22.
        dm[DM_DB + 0x04] = 0x6000  # V90_DPCM + digital network (§5.3.1)
        dm[DM_DB + 0x28] = 0x0001  # negotiate through V.8
        dm[DM_DB + 0x29] = 0x8100  # V.90 + V.34 modulation mask
        dm[DM_DB + 0x79] = 0x003F
        dm[DM_DB + 0x7A] = 0xFFFF
        dm[DM_DB + 0x7B] = 0x03B7
        dm[DM_DB + 0x7C] = 0x000E
        dm[DM_DB + 0x7D] = 0x0015
        dm[DM_DB + 0x7E] = 0x000E
        dm[DM_DB + 0x7F] = 0x0015
        # Do not write generic V34SLOT (+0x78).  Eicon's PRI kernel routes
        # TIKRNL through its private channel descriptor; A/B replay confirms
        # that +0x78 is inert in this image, like SPORT setup +0x70..+0x74.
        dm[DM_WSTATUS] = 0x2000
        dm[DM_DB + 0x0F] = 0x0001
        dm[DM_DB + 0x10] = 0x0100
        # Reconstruct the assigned one-word RX/TX buffers. TIKRNL dereferences
        # these pointers in AV/pointer mode; the modem pages themselves use
        # the line words at 0x3F08/0x3F09.
        self.assign_pcm_buffers()

    def load_v8(self, index: int, fast: bool = False) -> collections.Counter:
        """Perform the host-supervisor DIAL -> V.8 decision (§5.4.2.1)."""
        dm = self.card.dm
        dm[DM_BOOTPAGE] = 6
        dm[DM_DOWNLOAD_FLAG] = 1
        dm[DM_DOWNLOAD_REQ] = V8_DOWNLOAD
        description = self.card.download_overlay(V8_DOWNLOAD)
        if description is None:
            raise RuntimeError('V.8 overlay 0x025f is unavailable')
        dm[DM_WSTATUS] = 0x1000       # BOOTFINISHED
        hist = self.resume(DM_ENTRIES + 1, index, fast=fast)
        dm[DM_WSTATUS] &= ~0x1000
        if self.log:
            print(f'  sample {index}: supervisor loaded V.8 ({description})')
        return hist


class LiveKernelModem:
    """Card-compatible live modem using the real SPORT0 kernel dispatcher."""

    # The INFO overlay's action table, PM 0x2ee6..0x2eee, indexed by action
    # code.  Nothing in the resident image dispatches it; PM 0x2148 executes
    # such tables through the script pointer DM(0x1667).  Action 1 (PM 0x2602)
    # installs framer B and clears the transmit bit-clock divider DM(0x16af),
    # which leaves the 1200 Hz modulator carrier running unmodulated instead
    # of idle; actions 3..8 build an 8-bit control-channel message for
    # transmission through PM 0x2446.
    INFO_ACTIONS = {0: 0x2410, 1: 0x2602, 2: 0x242B, 3: 0x2430, 4: 0x243D,
                    5: 0x2441, 6: 0x243F, 7: 0x2434, 8: 0x243B}

    def __init__(self, channel: int = 0, enable_l1l2_gate: bool = False,
                 init_info_detector_at_24: bool = False,
                 info_actions: dict[int, int] | None = None,
                 delay_correction: int = 0x000C):
        self.driver = KernelDispatch()
        self.card = self.driver.card
        self.dm = self.card.dm
        self.pm = self.card.pm
        self.overlays = self.card.overlays
        self.switches = self.card.switches
        self.forced_info_samples = self.card.forced_info_samples
        self.resident = 0
        self.v8_loaded = False
        self.l1l2_forced_samples: list[int] = []
        self._tone_a_window: collections.deque[int] = collections.deque(maxlen=160)
        self._tone_a_checks = 0
        self._tone_a_injected = False
        self.enable_l1l2_gate = enable_l1l2_gate
        self.init_info_detector_at_24 = init_info_detector_at_24
        # {TrnProgress: action code}, each fired once on first reaching it.
        self.info_actions = dict(info_actions or {})
        self._info_actions_fired: set[int] = set()
        self.info_action_samples: list[tuple[int, int, int]] = []
        self._info_detector_last_trn = -1
        if not 0 <= channel < 32:
            raise ValueError('PRI channel must be in range 0..31')
        self.channel = channel
        if not 0 <= delay_correction <= 0xFFFF:
            raise ValueError('delay correction must be a 16-bit word')
        self.delay_correction = delay_correction

    def boot(self) -> None:
        driver, card, dm = self.driver, self.card, self.dm
        driver.boot()
        for _ in range(32):
            driver.strobe_fast()
            if dm[DM_CMD_DESC]:
                break
        else:
            raise RuntimeError('kernel foreground did not initialise command ring')
        if not driver.push(TASK_ENTRY):
            raise RuntimeError('could not queue TIKRNL task entry')
        for _ in range(32):
            driver.strobe_fast()
            if dm[DM_ENTRIES] and self.pm[PM_FOREGROUND_SLOT] != driver._call_word(PM_DISPATCH):
                break
        else:
            raise RuntimeError('kernel did not register TIKRNL')
        for base_id in (V_OWN_DOWNLOAD, FSK_OWN_DOWNLOAD):
            entry = card.overlays.get(base_id)
            if entry is None:
                raise RuntimeError(f'missing base overlay 0x{base_id:04x}')
            card._download(entry[0])
        card.download_overlay(DIAL_ID)
        self.resident = card.resident

    def _validate_v90d_configuration(self) -> None:
        """Verify the documented host inputs that make V90D negotiable."""
        dm = self.dm
        expected = {
            0x00: 0x00C4,  # extended training, PSTN, normal equaliser
            0x01: 0x0484,  # answer, 2-wire, internal clock, NORM
            0x04: 0x6000,  # V90_DPCM and digital network
            0x07: 0xF0FD,  # V.34 INFO0 capabilities
            0x24: self.delay_correction,
            0x28: 0x0001,  # V.8
            0x29: 0x8100,  # V.90 with V.34 fallback
            0x2A: 0x001F,  # V.34 high-rate mask
            0x79: 0x003F,  # every defined V.90 rate through 56 kbit/s
            0x7A: 0xFFFF,
            0x7B: 0x03B7,  # lookahead 3, 3429 upstream, PCMU, -12 dBm0
            0x7C: 0x000E,  # V.34 TX maximum 33600
            0x7D: 0x0015,  # V.90 TX maximum 56000
            0x7E: 0x000E,  # V.34 RX maximum 33600
            0x7F: 0x0015,  # V.90 RX maximum 56000
        }
        bad = [(offset, dm[DM_DB + offset], value)
               for offset, value in expected.items()
               if dm[DM_DB + offset] != value]
        # DIAL enables its line tone detector after consuming host GEN_SETUP2;
        # bit 6 is firmware-owned at this point, while 0x30 must survive.
        if dm[DM_DB + 0x02] & 0x003F != 0x0030:
            bad.append((0x02, dm[DM_DB + 0x02], 0x0030))
        if bad:
            detail = ', '.join(f'+{offset:02x}={actual:04x} (want {wanted:04x})'
                               for offset, actual, wanted in bad)
            raise RuntimeError(f'V90D setup did not survive DIAL activation: {detail}')

    def configure_modem(self, role: str, law: str = 'pcmu') -> None:
        if law != 'pcmu':
            raise NotImplementedError('kernel-dispatch live mode currently supports PCMU')
        if role != 'answer':
            raise NotImplementedError('kernel-dispatch live mode currently supports answer mode')
        # The MIPS channel assignment normally selects the SPORT companding
        # table. TIKRNL defaults to A-law; select the RTP bearer law before
        # DIAL snapshots it into INFO0D_setup bit 6.
        self.card.configure_g711_law(law)
        # Kernel service 0x001e identifies the assigned channel's companding
        # descriptor: 0x3c27 is µ-law, 0x3c07 is A-law. The MIPS assignment
        # normally writes this before TIKRNL configures its per-line adapter.
        self.dm[0x2F22] = 0x3C27 if law == 'pcmu' else 0x3C07
        # The guide requires two distinct Host-Kernel write cycles: power-up
        # Table 12 first, then recommendation/answer-mode Tables 13 and 15.
        # Drive a few genuine SPORT frames after each activation so TIKRNL
        # consumes change_wdb before any later values replace the first set.
        self.driver.program_initial_setup(self.delay_correction)
        for index in range(-16, -8):
            self.driver.tdm_frame(0x00ff, self.channel)
            self.driver.service(index, fast=True)
        if self.dm[DM_WSTATUS] & 0x2000:
            raise RuntimeError('DSP did not consume initial write database')
        self.driver.program_v8_call(calling=False)
        # These are DIAL-overlay setup entries and must run before the second
        # cycle is allowed to request/load V.8; calling them after that partial
        # overlay replacement corrupts the completion path.
        self.card._run(0x08F1, 200_000)
        self.card._run(0x13CC, 1_000_000)
        for index in range(-8, 0):
            self.driver.tdm_frame(0x00ff, self.channel)
            self.driver.service(index, fast=True)
        if self.dm[DM_WSTATUS] & 0x2000:
            raise RuntimeError('DSP did not consume answer-mode write database')
        self._validate_v90d_configuration()

    @staticmethod
    def _decode_mulaw(code: int) -> int:
        value = (~code) & 0xff
        sample = (((value & 0x0f) << 3) + 0x84) << ((value >> 4) & 7)
        sample -= 0x84
        return -sample if value & 0x80 else sample

    def _inject_l1l2_completion(self, code: int, index: int) -> None:
        """Publish INFO's missing post-L2 event on confirmed peer Tone A.

        V.90 §9.2.1.1.5 says Tone A follows the analogue modem's L1/L2.
        The emulated INFO probing classifier never publishes event 1, although
        the waveform is present.  Use a narrowband 2400-Hz gate only while the
        genuine firmware is in state 0x37; all subsequent sequencing remains
        firmware generated.  Four overlapping 20-ms confirmations reject the
        preceding multitone L2 signal.
        """
        dm = self.dm
        if not self.enable_l1l2_gate:
            return
        if dm[DM_LIVE_TRNPROG] != 0x0024:
            self._tone_a_window.clear()
            self._tone_a_checks = 0
            self._tone_a_injected = False
            return
        if self._tone_a_injected:
            return
        self._tone_a_window.append(self._decode_mulaw(code))
        if len(self._tone_a_window) < 160 or index % 40:
            return
        samples = list(self._tone_a_window)
        mean = sum(samples) / len(samples)
        total = sum((sample - mean) ** 2 for sample in samples)
        if total < 160 * 500 * 500:
            self._tone_a_checks = 0
            return
        coefficient = 2.0 * math.cos(2.0 * math.pi * 2400.0 / SAMPLE_RATE)
        q1 = q2 = 0.0
        for sample in samples:
            q0 = sample - mean + coefficient * q1 - q2
            q2, q1 = q1, q0
        tone_power = q1 * q1 + q2 * q2 - coefficient * q1 * q2
        ratio = 2.0 * tone_power / (len(samples) * total)
        self._tone_a_checks = self._tone_a_checks + 1 if ratio >= 0.70 else 0
        if self._tone_a_checks >= 4:
            dm[0x198E] = 1
            self._tone_a_injected = True
            self.l1l2_forced_samples.append(index)

    def frame_fast(self, code: int, index: int) -> int:
        driver, card, dm = self.driver, self.card, self.dm
        before = card.resident
        self._inject_l1l2_completion(code, index)
        driver.tdm_frame(code, self.channel)
        # Diagnostic reconstruction: PM 0x2602 installs the detector parser
        # and event table after the FFT workspace is finished with DM 0x1986.
        # Calling it at overlay load is too early: the FFT overwrites the table.
        # The natural variant-8 path currently fails to invoke it at the 0x37
        # seam, so keep this explicit and opt-in until that branch is recovered.
        trn_progress = dm[DM_LIVE_TRNPROG]
        # Diagnostic: dispatch an INFO action-table entry the first time the
        # page reaches a chosen state.  The natural dispatcher for this table
        # has not been recovered, so this stays explicit and opt-in.
        action = self.info_actions.get(trn_progress)
        if action is not None and trn_progress not in self._info_actions_fired:
            self._info_actions_fired.add(trn_progress)
            entry = self.INFO_ACTIONS[action]
            ADSP.adsp2181_call(card.cpu, entry, KERNEL_IDLE)
            ADSP.adsp2181_run(card.cpu, 400_000)
            if not ADSP.adsp2181_idle(card.cpu):
                raise RuntimeError(f'INFO action {action} (PM {entry:#06x}) '
                                   'did not return to IDLE')
            self.info_action_samples.append((index, trn_progress, action))
        if (self.init_info_detector_at_24
                and card.resident == 0x0260 and trn_progress == 0x0024
                and self._info_detector_last_trn != 0x0024):
            ADSP.adsp2181_call(card.cpu, 0x2602, KERNEL_IDLE)
            ADSP.adsp2181_run(card.cpu, 300_000)
            if not ADSP.adsp2181_idle(card.cpu):
                raise RuntimeError('INFO detector initialization did not return to IDLE')
        self._info_detector_last_trn = trn_progress
        # Do not force the INFO -> V90D handoff from TrnProgress.  Per V.90
        # §9.2.1.1.8, INFO1a bits 37:39 select Phase 3 only when they encode 6.
        # INFO turns that result into bootpage 14 itself; driver.service() then
        # serves TIKRNL's ordinary page-14 request for overlay 0x026a.
        driver.service(index, fast=True)
        if card.resident == V8_DOWNLOAD and not self.v8_loaded:
            self.v8_loaded = True
            dm[DM_LINE_RX] |= 0x0020
            self.card._run(dm[DM_ENTRIES], 200_000)
        elif (not self.v8_loaded and not (dm[DM_DB + 1] & 0x0080)):
            driver.load_v8(index, fast=True)
            self.v8_loaded = True
        if card.resident != before:
            card.switches.append((index, dm[DM_BOOTPAGE], card.resident))
        self.resident = card.resident
        # The PRI timeslot-to-SPORT TX bridge still needs the MIPS channel
        # assignment descriptor.  TIKRNL nevertheless publishes the exact
        # signed-linear sample through its assigned one-word TX buffer, just
        # as the direct harness reads it after the continuation.
        pointer = dm[DM_TX_BUFFER_POINTER] & 0x3fff
        value = dm[pointer] if pointer else 0
        return value - 0x10000 if value & 0x8000 else value


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
                dm[DM_LINE_RX] |= 0x0020
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
