#!/usr/bin/env python3
"""Run DIAL under TIKRNL, the way the card actually dispatches it.

`tools/dial_standalone_drive.py` proves DIAL processes audio, but it calls
DIAL's internals (PM 0x1B9C / 0x1BBD) as subroutines, with the DIAL overlay
layered straight onto the PRI kernel.  That is not how the card runs it.

The real chain is kernel -> task -> overlay:

  * The PRI 30M kernel (download 0x0009) owns PM 0x0000-0x05EB and the
    interrupt/service vector table.  Slots 0x0001-0x0003, 0x000A-0x000B,
    0x000E, 0x0015, 0x0017, 0x0019, 0x001E are kernel service entries; the
    genuine interrupt vectors in between are `RTI`.
  * TIKRNL81.F34 (download 0x0258) is the modem *task*.  Its entry is
    download symbol 0 = PM 0x0672.  Running it initialises the task, then
    registers a per-frame continuation through kernel service 0x0017, which
    patches PM 0x000A to `CALL 0x06FC` and PM 0x0000 to `CALL 0x08F6`.
  * DIAL/FSK/FAX.F34 (download 0x0262) is an *overlay* on the task.  Its
    download replaces the two stub words TIKRNL keeps at PM 0x08F0/0x08F1
    (`RTS`, `RTS`) with `JUMP 0x1B9C` / `JUMP 0x1BBD`.

TIKRNL's per-frame loop (PM 0x06BB-0x06EE) is what calls the overlay:

    06bb  CALL $0002        ; kernel queue service, host command dispatch
    06c0  CALL (I4)         ; dispatch the queued command
    06c2  CALL $064A        ; frame housekeeping / status publish
    06c7  AR = DM($3F08)    ; the line/RX register
    06d0  DM($3F05) = $FFFF
    06d1  CALL $08F1        ; -> DIAL line handler (0x1BBD)
    06e2  CALL $08F0        ; -> DIAL state dispatcher (0x1B9C)
    06e7  I4 = DM($3FB2)    ; -> DIAL's own action vector
    06e8  CALL (I4)

Two things this harness gets right that the standalone one does not:

1.  **Download order.**  TIKRNL's init (PM 0x0637) clears PM 0x0900-0x1DFF —
    the whole overlay region.  Layering DIAL before the task init silently
    erases most of DIAL.  The overlay has to be downloaded *after* the task
    has initialised, which is also the order the host driver uses.
2.  **Entry point.**  DIAL is entered by TIKRNL through the PM 0x08F0/0x08F1
    stubs, not called directly, so the overlay interface (and anything the
    task does to the data-pump database before and after the call) is
    exercised.

Also recovered here: the bootpage table at DM 0x31D5.  TIKRNL indexes it with
bootpage_nr (DM 0x3FB0) to get the overlay download id to ask the host for,
and it matches the ADDSP V.90 guide's Table 1 page numbering exactly
(0 = DIAL 0x0262, 6 = V.8 0x025F, 7 = INFO 0x0260, 8 = V.34 0x0261,
14 = V.90D 0x026A).  `--bootpage-table` prints it.

**Serving the page switch.**  When TIKRNL wants an overlay it publishes the
type in DM 0x31A9 and the download id in DM 0x31AA, then yields by jumping to
the kernel service slot PM 0x000A with AR = 2.  AR is an index into the task
entry table the init registered at DM 0x31BA:

    DM 31ba = 0x06BB   AR = 1: ordinary per-frame entry
    DM 31bb = 0x06D8   AR = 2: "the overlay you asked for is loaded"

`Card.frame()` plays the host side of that handshake: it downloads the
requested image and re-enters the task at DM(0x31BB), repeating until the task
stops asking.  0x06D8 is what runs the half of the frame loop the request path
skips -- the SIG stub at PM 0x1900, then the strobe clear, the DIAL state
dispatcher at PM 0x08F0 and the action vector at DM 0x3FB2.  Without a host
serving the download the task never gets there.

Overlay images come from the card-type 56 (PRI 30M / .F34) download set:

    python3 tools/eicon_dsp_extract.py docs/firmware/dspdload.bin \
        --card-type 56 --match Overlay -o artifacts/eicon-dsp/overlays

The harness can also activate calling or answering operation by writing the
ADDSP §5.4.1 database directly.  It invokes both halves of the real sample
schedule: the page/RX entry and TIKRNL's registered PM 0x06FC continuation.
This produces genuine modem TX without MIPS, IDI, call objects or timeslots:

Usage:
    python3 tools/dial_tikrnl_drive.py --freq 2100 --frames 200
    python3 tools/dial_tikrnl_drive.py --role answer --freq 0 --frames 12000 \\
        --tx-out /tmp/eicon-answer.s16 --g711-out /tmp/eicon-answer.alaw
"""
from __future__ import annotations

import argparse
import collections
import ctypes
import json
import math
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
ADSP = ctypes.CDLL(str(REPO / 'tools/adsp2181emu/libadsp2181.dylib'))
ADSP.adsp2181_create.restype = ctypes.c_void_p
for _name, _args in [('reset', [ctypes.c_void_p]), ('pm', [ctypes.c_void_p]),
                     ('dm', [ctypes.c_void_p]),
                     ('run', [ctypes.c_void_p, ctypes.c_int]),
                     ('pc', [ctypes.c_void_p]), ('idle', [ctypes.c_void_p]),
                     ('set_pc', [ctypes.c_void_p, ctypes.c_uint16]),
                     ('call', [ctypes.c_void_p, ctypes.c_uint16, ctypes.c_uint16]),
                     ('set_ar', [ctypes.c_void_p, ctypes.c_uint16]),
                     ('watch_dm', [ctypes.c_void_p, ctypes.c_uint16, ctypes.c_int]),
                     ('watch_pm', [ctypes.c_void_p, ctypes.c_uint16, ctypes.c_int]),
                     ('watch_exec', [ctypes.c_void_p, ctypes.c_uint16, ctypes.c_int])]:
    getattr(ADSP, 'adsp2181_' + _name).argtypes = _args
ADSP.adsp2181_pm.restype = ctypes.POINTER(ctypes.c_uint32)
ADSP.adsp2181_dm.restype = ctypes.POINTER(ctypes.c_uint16)
ADSP.adsp2181_pc.restype = ctypes.c_uint16
ADSP.adsp2181_idle.restype = ctypes.c_int
ADSP.adsp2181_pmovlay.argtypes = [ctypes.c_void_p]
ADSP.adsp2181_pmovlay.restype = ctypes.c_uint16
ADSP.adsp2181_dmovlay.argtypes = [ctypes.c_void_p]
ADSP.adsp2181_dmovlay.restype = ctypes.c_uint16
ADSP.adsp2181_read_pm.argtypes = [ctypes.c_void_p, ctypes.c_uint16]
ADSP.adsp2181_read_pm.restype = ctypes.c_uint32
ADSP.adsp2181_sr0.argtypes = [ctypes.c_void_p]
ADSP.adsp2181_sr0.restype = ctypes.c_uint16
ADSP.adsp2181_sr1.argtypes = [ctypes.c_void_p]
ADSP.adsp2181_sr1.restype = ctypes.c_uint16

KERNEL = 'artifacts/eicon-dsp/build-117-926/kernel/0009-diva-server-pri-30m-kernel'
TIKRNL = 'artifacts/eicon-dsp/build-117-926/tikrnl/0258-tikrnl81.f34-task'
OVERLAYS = 'artifacts/eicon-dsp/overlays'   # card-type 56 (PRI 30M / .F34) set
DIAL_ID = 0x0262                            # the bootpage the card starts on
V_OWN_ID = 0x026D                           # base routines under partial pages
FSK_OWN_ID = 0x025C                         # base routines under DIAL/FSK/FAX

# TIKRNL entry points (download 0x0258).
TASK_ENTRY = 0x0672      # download symbol 0: init + register with the kernel
FRAME_ENTRY = 0x06BB     # per-frame loop head: command dispatch -> overlay
FRAME_ENTRY_NO_HOST = 0x06C1  # same loop, past the host-command fetch/dispatch
SAMPLE_CONTINUATION = 0x06FC  # registered kernel callback: RX/TX second half
G711_ENCODE_ENTRY = 0x1810    # TIKRNL's resident signed-linear -> G.711 routine
KERNEL_IDLE = 0x02A8     # return address for a task call

# Task entry table registered with the kernel at init (PM 0x069C, SR0=0x31BA).
# The task selects one by loading AR and jumping to the kernel service slot
# PM 0x000A: AR = 1 -> DM 0x31BA = 0x06BB, AR = 2 -> DM 0x31BB = 0x06D8.
RESUME_ENTRIES = 0x31BA
RESUME_DOWNLOAD = 0x31BB # AR = 2: resume after the host served a download

# Overlay interface (segments shared by TIKRNL and every bootpage overlay).
OVL_STATE_STUB = 0x08F0  # -> DIAL 0x1B9C, the state dispatcher
OVL_LINE_STUB = 0x08F1   # -> DIAL 0x1BBD, the line/RX handler
SIG_STUBS = (0x1900, 0x1901, 0x1902)  # the SIG overlay's three stubs

# PM 0x069E is the `JUMP $000A` the request path takes after publishing the
# download in DM 0x31A9/0x31AA -- the marker that the task yielded for an
# overlay rather than finishing the frame.
PM_DOWNLOAD_YIELD = 0x069E

def sport_rx_word(code: int, law: str = 'pcmu') -> int:
    """Expand a DS0 octet as the T1/E1 SPORT compander does.

    ADDSP V.90 User's Guide §3.3: the SPORT companding hardware hands the
    page a signed linear sample, not the logarithmic G.711 code.  INFO's
    correlators are meaningless in the compressed domain, and the transmit
    side already agrees -- DM(0x3764) is signed linear.  Nothing here
    resamples or changes gain, so the DS0 stream stays byte-exact.
    """
    code &= 0xFF
    if law == 'pcma':
        value = code ^ 0x55
        sample = (value & 0x0F) << 4
        segment = (value & 0x70) >> 4
        if segment == 0:
            sample += 8
        elif segment == 1:
            sample += 0x108
        else:
            sample = (sample + 0x108) << (segment - 1)
        return (sample if value & 0x80 else -sample) & 0xFFFF
    value = (~code) & 0xFF
    sample = (((value & 0x0F) << 3) + 0x84) << ((value >> 4) & 7)
    sample -= 0x84
    return (-sample if value & 0x80 else sample) & 0xFFFF


# Data-pump database (ADDSP V.90 guide §5.3).
DM_LINE_RX = 0x3F08
DM_LINE_TX = 0x3F09
DM_BOOTPAGE = 0x3FB0     # bootpage_nr / DIAL state selector
DM_VEC_A = 0x3FB2        # DIAL primary action vector
DM_VEC_B = 0x3FB3        # DIAL secondary action vector
DM_TX_POINTER = 0x3FB4   # pointer to current signed-linear TX sample
DM_STATUS = 0x3FC1
DM_DB = 0x3EE0          # ADDSP data-pump database base (§5.4.1)

# TIKRNL private state.
DM_BOOTPAGE_TABLE = 0x31D5   # bootpage number -> overlay download id
DM_DOWNLOAD_REQ = 0x31AA     # download id TIKRNL is asking the host for
DM_DOWNLOAD_FLAG = 0x31A9

# The kernel's five ring-descriptor pointers.  Its foreground writes them at
# PM 0x02AD-0x02B2 the first time it wakes, but this harness calls the task
# directly and never lets the foreground run, so they have to be planted --
# see tools/dial_kernel_dispatch.py, which lets the kernel write them itself.
# They matter here because the task's registration (kernel service 0x0017)
# reads the block at DM 0x2F21 to find the two words it patches; with the
# pointers zero it indexes DM 0x0002 instead and patches PM 0x0000/0x000A.
RING_POINTERS = {0x2F27: 0x2F21,   # task registration block
                 0x2F28: 0x2F00,   # host -> DSP command ring descriptor
                 0x2F29: 0x2F0E,   # DSP -> host descriptor + doorbell
                 0x2F2A: 0x2F42,
                 0x2F2B: 0x2F4E}
PM_FOREGROUND_SLOT = 0x02B9   # kernel foreground: CALL $02A1 -> the task
PM_ISR_SLOT = 0x00B5          # SPORT0 ISR: an inline op the task may claim

# PM 0x00D8 walks the command ring as `I0 = DM(0x2F28)`, and PM 0x06C0 calls
# whatever handler address the walk produced.  DM 0x2F03 (the byte count) stays
# zero until a host command is queued, and an unwritten DM 0x2F28 walks DM
# 0x0000 instead -- harmless while that is unpopulated, which is why the
# pre-serving harness never noticed, but the V.8, V.22FC and DIAL-partial
# overlays all load DM from 0x0000, so the first page switch turns overlay
# coefficients into dispatch addresses.  See FRAME_ENTRY_NO_HOST.
DM_QUEUE_HEAD = 0x2F28


def read_words(path: Path) -> dict[int, int]:
    out = {}
    for line in path.read_text().splitlines():
        fields = line.split()
        if len(fields) == 2:
            out[int(fields[0], 16)] = int(fields[1], 16)
    return out


def reverse_octet(value: int) -> int:
    value &= 0xFF
    value = ((value & 0x55) << 1) | ((value >> 1) & 0x55)
    value = ((value & 0x33) << 2) | ((value >> 2) & 0x33)
    return ((value << 4) | (value >> 4)) & 0xFF


def linear_to_mulaw(sample: int) -> int:
    sample = max(-32768, min(32767, sample))
    sign = 0x80 if sample < 0 else 0
    if sample < 0:
        sample = -sample - 1
    sample = min(sample + 0x84, 0x7FFF)
    segment = 0
    top = sample >> 5
    while top and segment < 8:
        top >>= 1
        segment += 1
    if segment >= 8:
        return (sign | 0x7F) ^ 0xFF
    return (sign | (segment << 4) | ((sample >> (segment + 3)) & 0xF)) ^ 0xFF


class Card:
    """Kernel + TIKRNL task + DIAL overlay on one emulated ADSP-2181."""

    def __init__(self, log: bool = False, serve: bool = True,
                 max_downloads: int = 8, host_dispatch: bool = False,
                 force_info_after_v8: bool = False):
        self.log = log
        self.serve = serve
        self.max_downloads = max_downloads
        self.force_info_after_v8 = force_info_after_v8
        # PM 0x06BB-0x06C0 fetches and dispatches a host command.  With no
        # channel assigned there is nothing to fetch, and the walk aliases an
        # overlay's DM 0x0000, so the default entry starts just past it.
        self.entry = FRAME_ENTRY if host_dispatch else FRAME_ENTRY_NO_HOST
        self.cpu = ADSP.adsp2181_create()
        ADSP.adsp2181_reset(self.cpu)
        self.pm = ADSP.adsp2181_pm(self.cpu)
        self.dm = ADSP.adsp2181_dm(self.cpu)
        self.pm_loaded: set[int] = set()
        self.overlays = self._index_overlays()
        self.resident = 0                                  # download id on the card
        self.served: collections.Counter = collections.Counter()
        self.unserved: collections.Counter = collections.Counter()
        self.switches: list[tuple[int, int, int]] = []     # frame, bootpage, id
        self.forced_info_samples: list[int] = []

    @staticmethod
    def _index_overlays() -> dict[int, tuple[Path, str]]:
        """Map download id -> extracted image, from the card's overlay set."""
        root = REPO / OVERLAYS
        if not root.is_dir():
            raise SystemExit(
                f'{OVERLAYS} is missing.  Extract the card-type 56 overlay set:\n'
                '  python3 tools/eicon_dsp_extract.py docs/firmware/dspdload.bin \\\n'
                '      --card-type 56 --match Overlay -o ' + OVERLAYS)
        index: dict[int, tuple[Path, str]] = {}
        for entry in sorted(root.iterdir()):
            meta = entry / 'metadata.json'
            if meta.is_file():
                data = json.loads(meta.read_text())
                index.setdefault(data['download_id'],
                                 (entry, data['description']))
        return index

    def _download(self, directory: Path | str) -> None:
        base = REPO / directory
        for addr, value in read_words(base / 'pm.words').items():
            self.pm[addr] = value
            self.pm_loaded.add(addr)
        for addr, value in read_words(base / 'dm.words').items():
            self.dm[addr] = value

    def download_overlay(self, download_id: int) -> str | None:
        """Serve one overlay download the way the host driver does."""
        entry = self.overlays.get(download_id)
        if entry is None:
            self.unserved[download_id] += 1
            return None
        path, description = entry
        self._download(path)
        # Do not set WSTATUS.BOOTFINISHED here.  This direct harness resumes
        # TIKRNL explicitly through DM(0x31BB); adding the ordinary host/kernel
        # acknowledgement as well completes the download twice.  In particular
        # FAX/partial pages then leave the page-change strobe asserted and are
        # destructively reloaded several times per sample.
        self.resident = download_id
        self.served[download_id] += 1
        return description

    def boot(self) -> None:
        """Kernel download + reset, then the task, then the overlay."""
        self._download(KERNEL)
        ADSP.adsp2181_run(self.cpu, 5000)   # reset vector -> kernel init -> IDLE
        if not ADSP.adsp2181_idle(self.cpu):
            raise RuntimeError('kernel did not reach its idle loop')

        # Stand in for the foreground pass this harness never lets happen.
        for addr, value in RING_POINTERS.items():
            self.dm[addr] = value

        self._download(TIKRNL)
        ADSP.adsp2181_call(self.cpu, TASK_ENTRY, KERNEL_IDLE)
        for _ in range(1_000_000):
            ADSP.adsp2181_run(self.cpu, 1)
            if ADSP.adsp2181_idle(self.cpu):
                break
        else:
            raise RuntimeError('TIKRNL task entry did not return to the kernel')

        # The task registered itself: the kernel's foreground dispatch now
        # calls TIKRNL's per-sample continuation, and the SPORT0 ISR word the
        # task claimed calls its own copy of the instruction it displaced.
        self.foreground_slot = self.pm[PM_FOREGROUND_SLOT]
        self.isr_slot = self.pm[PM_ISR_SLOT]

        # Only now is it safe to download overlays: task init cleared PM
        # 0x0900-0x1DFF. The .F34 images are layered partial overlays. DIAL
        # calls shared routines beyond its own image (notably PM 0x244c and
        # 0x2c4f), supplied by V.OWN and FSK OWN in the real host flow. Loading
        # DIAL alone leaves those calls unpopulated and causes unstable V.8
        # classification/INFO transitions.
        for base_id in (V_OWN_ID, FSK_OWN_ID):
            entry = self.overlays.get(base_id)
            if entry is None:
                raise SystemExit(f'no extracted base image 0x{base_id:04x}')
            self._download(entry[0])
        if self.download_overlay(DIAL_ID) is None:
            raise SystemExit(f'no extracted image for download 0x{DIAL_ID:04x}')
        self.served.clear()   # the boot page is not a page *switch*

    def configure_g711_law(self, law: str) -> None:
        """Select TIKRNL's resident encoder parameter table."""
        self.dm[0x3309] = 0x35BE if law == 'pcmu' else 0x35B7

    def encode_g711(self, samples: list[int]) -> bytes:
        """Call TIKRNL's resident G.711 encoder at PM 0x1810.

        The PRI/E1 kernel selects the A-law parameter table at DM 0x3309.
        PM 0x1810 accepts signed linear PCM in AR and returns the serial-wire
        bit order in SR1.  Reverse each returned octet to conventional G.711
        file/RTP bit order.  Run this after modem framing: the subroutine uses
        core DAG registers that hardware SPORT companding would not disturb.
        """
        encoded = bytearray()
        for sample in samples:
            ADSP.adsp2181_set_ar(self.cpu, sample & 0xFFFF)
            ADSP.adsp2181_call(self.cpu, G711_ENCODE_ENTRY, KERNEL_IDLE)
            ADSP.adsp2181_run(self.cpu, 1000)
            encoded.append(reverse_octet(ADSP.adsp2181_sr1(self.cpu)))
        return bytes(encoded)

    def configure_modem(self, role: str, law: str = 'pcmu') -> None:
        """Activate the data pump directly, without MIPS/IDI call control.

        These are the ADDSP §5.4.1 database writes also made by the Linux
        driver's modem B1 assignment path.  GEN_SETUP1 bit 3 distinguishes
        calling (0x048c) from answering (0x0484) operation.
        """
        if role == 'idle':
            return
        # Tables 12-15 plus V.90-specific §5.3.1 fields. These values are all
        # resident before the final change_wdb strobe is consumed by DIAL.
        writes = {
            DM_DB + 0x00: 0x00C4,
            DM_DB + 0x01: 0x048C if role == 'calling' else 0x0484,
            DM_DB + 0x02: 0x0030,
            DM_DB + 0x04: 0x6000,                     # V90_DPCM + digital network
            DM_DB + 0x07: 0xF0FD,
            DM_DB + 0x08: 0x0006, DM_DB + 0x09: 0x0006,
            DM_DB + 0x0A: 0x00FF, DM_DB + 0x0B: 0x0030,
            DM_DB + 0x0C: 0x0000,
            DM_DB + 0x28: 0x0001,                     # V.8
            DM_DB + 0x29: 0x8100,                     # V.90 + V.34
            DM_DB + 0x2A: 0x001F, DM_DB + 0x2B: 0xFF00,
            DM_DB + 0x2C: 0x0003, DM_DB + 0x2D: 0x0003,
            DM_DB + 0x79: 0x003F, DM_DB + 0x7A: 0xFFFF,
            DM_DB + 0x7B: 0x03B7 | (0x0040 if law == 'pcma' else 0),
            DM_DB + 0x7C: 0x000E, DM_DB + 0x7D: 0x0015,
            DM_DB + 0x7E: 0x000E, DM_DB + 0x7F: 0x0015,
            DM_DB + 0x0E: 0x2000,
        }
        for address, value in writes.items():
            self.dm[address] = value

    def _run(self, entry: int, budget: int) -> collections.Counter:
        """Run the task from one entry point until it yields to the kernel."""
        hist: collections.Counter = collections.Counter()
        ADSP.adsp2181_call(self.cpu, entry, KERNEL_IDLE)
        for _ in range(budget):
            pc = ADSP.adsp2181_pc(self.cpu)
            hist[pc] += 1
            if pc not in self.pm_loaded:
                hint = ''
                if self.entry == FRAME_ENTRY and self.served:
                    hint = (' -- with an overlay resident and DM 0x2F28 still '
                            'unassigned, the host-command walk dispatches '
                            'overlay DM as code; drop --host-dispatch')
                raise RuntimeError(f'ran into unpopulated PM at 0x{pc:04x}{hint}')
            ADSP.adsp2181_run(self.cpu, 1)
            if ADSP.adsp2181_idle(self.cpu) or ADSP.adsp2181_pc(self.cpu) == KERNEL_IDLE:
                break
        return hist

    def _maybe_force_info(self, wanted: int, index: int) -> int:
        """Diagnostic host policy: replace a post-V.8 fallback with INFO.

        Shipping V.8 normally writes pending page 7 itself. This option tests
        whether a peer that goes quiet after V.8 is waiting for the host side
        to start V.34/V.90 Phase 2 rather than accepting the DSP's low-level
        fallback. It is intentionally opt-in and does not alter natural page-7
        requests.
        """
        page = self.dm[DM_BOOTPAGE]
        if (self.force_info_after_v8 and index >= 12000
                and self.resident == 0x025F and page not in (6, 7)
                and wanted != 0x0260):
            self.dm[DM_BOOTPAGE] = 7
            self.dm[DM_DOWNLOAD_REQ] = 0x0260
            if not self.forced_info_samples or self.forced_info_samples[-1] != index:
                self.forced_info_samples.append(index)
            return 0x0260
        return wanted

    def frame(self, rx_code: int, index: int = 0,
              budget: int = 20000) -> collections.Counter:
        """One 8 kHz frame: present a line sample, run TIKRNL's frame loop.

        The frame is not one pass through the task.  Whenever DIAL raises the
        page-change strobe (DM 0x3FC1 bit 8) the request path at PM 0x0686
        publishes an overlay download and yields; the host serves it and
        re-enters at DM(0x31BB) = PM 0x06D8.  Playing that host role here is
        what carries the frame past the request into the state dispatcher.
        """
        self.dm[DM_LINE_RX] = rx_code & 0xFFFF
        hist: collections.Counter = collections.Counter()
        entry = self.entry
        for _ in range(self.max_downloads + 1):
            this_pass = self._run(entry, budget)
            hist.update(this_pass)
            if not self.serve or not this_pass.get(PM_DOWNLOAD_YIELD):
                break
            wanted = self._maybe_force_info(self.dm[DM_DOWNLOAD_REQ], index)
            if wanted == self.resident:
                # The direct completion path can leave the request strobe set
                # briefly. Resume it, but do not destructively reload the
                # already-resident partial image or report a new page switch.
                entry = self.dm[RESUME_DOWNLOAD]
                continue
            description = self.download_overlay(wanted)
            if description is None:
                if self.log:
                    print(f'  frame {index}: no image for download '
                          f'0x{wanted:04x}, cannot serve')
                break
            self.switches.append((index, self.dm[DM_BOOTPAGE], wanted))
            if self.log:
                print(f'  frame {index}: bootpage {self.dm[DM_BOOTPAGE]:04x} '
                      f'-> served 0x{wanted:04x} {description}')
            entry = self.dm[RESUME_DOWNLOAD]

        # The real kernel invokes the continuation TIKRNL registered at init
        # once per SPORT sample.  It calls DM(3FB3), consumes DM(3FB4), and
        # runs the task's TX post-processing.  Calling only the 0x06c1 half
        # drives page/RX state but silently leaves every transmitter idle.
        hist.update(self._run(SAMPLE_CONTINUATION, budget))
        return hist

    def frame_fast(self, rx_code: int, index: int = 0,
                   budget: int = 20000) -> int:
        """Production version of :meth:`frame` without instruction tracing.

        ``adsp2181_run`` executes the whole pass in C.  The page-change strobe
        remains set while TIKRNL is yielded to the host, so it is sufficient
        to detect downloads without the Python per-instruction PC histogram.
        Returns the current signed-linear transmit sample.
        """
        self.dm[DM_LINE_RX] = rx_code & 0xFFFF
        entry = self.entry
        for _ in range(self.max_downloads + 1):
            ADSP.adsp2181_call(self.cpu, entry, KERNEL_IDLE)
            ADSP.adsp2181_run(self.cpu, budget)
            if not self.serve or not (self.dm[DM_STATUS] & 0x0100):
                break
            wanted = self._maybe_force_info(self.dm[DM_DOWNLOAD_REQ], index)
            if wanted == self.resident:
                # Complete a still-asserted request without resetting the
                # state held in the already-resident partial overlay.
                entry = self.dm[RESUME_DOWNLOAD]
                continue
            description = self.download_overlay(wanted)
            if description is None:
                break
            self.switches.append((index, self.dm[DM_BOOTPAGE], wanted))
            entry = self.dm[RESUME_DOWNLOAD]

        ADSP.adsp2181_call(self.cpu, SAMPLE_CONTINUATION, KERNEL_IDLE)
        ADSP.adsp2181_run(self.cpu, budget)
        pointer = self.dm[DM_TX_POINTER] & 0x3FFF
        value = self.dm[pointer] if pointer else 0
        return value - 0x10000 if value & 0x8000 else value


def print_bootpage_table(card: Card) -> None:
    names = {0x025C: 'FSK OWN', 0x025F: 'V.8', 0x0260: 'INFO', 0x0261: 'V.34',
             0x0262: 'DIAL/FSK/FAX', 0x0263: 'DIAL partial', 0x0266: 'V.22/V.32 LEC',
             0x0268: '?', 0x0269: '?', 0x026A: 'V.90 DPCM', 0x026B: '?',
             0x026E: 'INFOH', 0x026F: 'HV.34', 0x0271: 'V.22FC'}
    print('bootpage table (DM 0x31D5, indexed by bootpage_nr / DM 0x3FB0):')
    for page in range(18):
        raw = card.dm[DM_BOOTPAGE_TABLE + page]
        download = raw if raw < 0x8000 else 0x10000 - raw
        sign = '+' if raw < 0x8000 else '-'
        print(f'  page {page:2d}: {raw:04x} {sign} download 0x{download:04x} '
              f'{names.get(download, "")}')


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--frames', type=int, default=200)
    ap.add_argument('--freq', type=int, default=2100,
                    help='line tone in Hz (0 = μ-law silence)')
    ap.add_argument('--amp', type=int, default=20000)
    ap.add_argument('--role', choices=('idle', 'answer', 'calling'),
                    default='idle',
                    help='directly activate the modem database in this role; '
                         'bypasses MIPS, IDI, signalling and bearer assignment')
    ap.add_argument('--tx-out', type=Path,
                    help='write DM[3FB4] signed-linear TX samples as s16le')
    ap.add_argument('--g711-out', type=Path,
                    help='call TIKRNL PM 0x1810 and write raw A-law octets')
    ap.add_argument('--bootpage-table', action='store_true',
                    help='dump the recovered bootpage -> overlay id table')
    ap.add_argument('--no-serve-overlays', action='store_true',
                    help='leave the download requests unanswered, as before: '
                         'the task then never gets past PM 0x0686')
    ap.add_argument('--max-downloads', type=int, default=8,
                    help='cap on overlay downloads served within one frame')
    ap.add_argument('--host-dispatch', action='store_true',
                    help='enter at PM 0x06BB and run the host-command fetch '
                         'and dispatch; needs a MIPS-assigned queue, and '
                         'without one it dispatches overlay DM as code')
    ap.add_argument('--log', action='store_true')
    args = ap.parse_args()

    card = Card(log=args.log, serve=not args.no_serve_overlays,
                max_downloads=args.max_downloads,
                host_dispatch=args.host_dispatch)
    card.boot()
    card.configure_modem(args.role)
    print(f'[card] kernel + TIKRNL + DIAL up; role={args.role}; the task claimed '
          f'PM {PM_FOREGROUND_SLOT:04x}={card.foreground_slot & 0xFFFFFF:06x} '
          f'(foreground dispatch) and '
          f'PM {PM_ISR_SLOT:04x}={card.isr_slot & 0xFFFFFF:06x} (SPORT0 ISR)')
    print(f'[card] overlay stubs after the DIAL download: '
          f'PM 08f0={card.pm[OVL_STATE_STUB]:06x} '
          f'08f1={card.pm[OVL_LINE_STUB]:06x} (TIKRNL ships both as 0a000f = RTS)')
    print(f'[card] frame entry PM {card.entry:04x}; host-command queue '
          f'DM 2f28={card.dm[DM_QUEUE_HEAD]:04x} '
          + ('(assigned)' if card.dm[DM_QUEUE_HEAD] else
             '(unassigned -- the MIPS-side channel assignment)'))
    print(f'[card] task entry table DM {RESUME_ENTRIES:04x}: '
          f'AR=1 -> {card.dm[RESUME_ENTRIES]:04x}  '
          f'AR=2 -> {card.dm[RESUME_DOWNLOAD]:04x} (post-download resume)')
    print(f'[card] overlay images indexed: {len(card.overlays)} from {OVERLAYS}')

    if args.bootpage_table:
        print_bootpage_table(card)

    if args.freq:
        tone = [linear_to_mulaw(int(args.amp * math.sin(2 * math.pi * args.freq * i / 8000)))
                for i in range(8000)]
    else:
        tone = [0xFF] * 8000

    totals: collections.Counter = collections.Counter()
    states: collections.Counter = collections.Counter()
    changes = 0
    prev = None
    tx = []
    tx_linear = []
    for f in range(args.frames):
        hist = card.frame(tone[f % len(tone)], index=f)
        totals.update(hist)
        state = card.dm[DM_BOOTPAGE]
        states[state] += 1
        tx.append(card.dm[DM_LINE_TX])
        tx_pointer = card.dm[DM_TX_POINTER] & 0x3FFF
        tx_linear.append(card.dm[tx_pointer] if tx_pointer else 0)
        now = (card.dm[DM_LINE_RX], card.dm[DM_LINE_TX], state,
               card.dm[DM_VEC_A], card.dm[DM_VEC_B], card.dm[DM_STATUS])
        if now != prev:
            changes += 1
            if args.log or changes <= 12:
                print(f'  frame {f:4d}: 3F08={now[0]:04x} 3F09={now[1]:04x} '
                      f'3FB0={now[2]:04x} 3FB2={now[3]:04x} 3FB3={now[4]:04x} '
                      f'3FC1={now[5]:04x}')
            prev = now

    print(f'[card] {args.frames} frames, {changes} data-pump register changes')
    print('[card] DIAL entered via the overlay stubs: '
          f'08f1(line)={totals.get(OVL_LINE_STUB, 0)} '
          f'08f0(state)={totals.get(OVL_STATE_STUB, 0)} '
          f'1bbd={totals.get(0x1BBD, 0)} 1b9c={totals.get(0x1B9C, 0)}')
    print('[card] DIAL DSP work: action dispatch 1da7='
          f'{totals.get(0x1DA7, 0)} line-signal handler 1bce={totals.get(0x1BCE, 0)}')
    print('[card] SIG stubs (PM 1900-1902): '
          + ' '.join(f'{a:04x}:{totals.get(a, 0)}' for a in SIG_STUBS))
    print(f'[card] action vector call (PM 06e8, via DM 3FB2): '
          f'{totals.get(0x06E8, 0)}')
    print('[card] bootpage_nr (DM 3FB0) histogram: '
          + ' '.join(f'{s:04x}:{c}' for s, c in states.most_common()))

    # PM 0x0686-0x0694: index DM 0x31D5 with bootpage_nr, then publish the
    # wanted overlay in DM 0x31AA with a type in DM 0x31A9.  A positive table
    # entry is requested directly; the negative entries (DIAL among them) fall
    # through to the fixed pair (type 0x000D, download 0x0270 = the SIG
    # overlay) until SIG is resident, and are then requested directly too.
    if card.switches:
        print(f'[card] page switches served: {len(card.switches)}')
        for wanted, count in card.served.most_common():
            name = card.overlays[wanted][1].split(' Version')[0]
            pages = sorted({p for _, p, w in card.switches if w == wanted})
            print(f'  0x{wanted:04x} {name:28s} x{count:<4d} '
                  'from page ' + ','.join(f'{p:04x}' for p in pages))
        chain = ' -> '.join(f'{p:04x}:{w:04x}' for _, p, w in card.switches[:12])
        print(f'  first switches (page:download): {chain}'
              + (' ...' if len(card.switches) > 12 else ''))
    for wanted, count in sorted(card.unserved.items()):
        print(f'[card] UNSERVED download 0x{wanted:04x} x{count}: '
              'no extracted image, the frame stops at the request')
    download = card.dm[DM_DOWNLOAD_REQ]
    if download:
        print(f'[card] last TIKRNL overlay request: DM 31AA=0x{download:04x} '
              f'type DM 31A9=0x{card.dm[DM_DOWNLOAD_FLAG]:04x}; '
              f'resident overlay 0x{card.resident:04x}')
    # DM 0x3F09 is the second line register; TIKRNL ORs 0x1000 into it at PM
    # 0x06CE before handing the frame to the overlay, so print the whole word
    # rather than pretending the low byte is a bare μ-law codeword.
    print('[card] DM 3F09 (line register, post-TIKRNL) first 16: '
          + ' '.join(f'{v:04x}' for v in tx[:16]))
    first_nonzero = next((i for i, value in enumerate(tx_linear) if value), None)
    print(f'[card] DM[3FB4] signed-linear TX: pointer={card.dm[DM_TX_POINTER]:04x} '
          f'nonzero={sum(value != 0 for value in tx_linear)}/{len(tx_linear)} '
          f'first-nonzero={first_nonzero if first_nonzero is not None else "none"} '
          'first16=' + ' '.join(f'{v:04x}' for v in tx_linear[:16]))
    if args.tx_out:
        args.tx_out.parent.mkdir(parents=True, exist_ok=True)
        args.tx_out.write_bytes(b''.join(
            int(value).to_bytes(2, 'little') for value in tx_linear))
        print(f'[card] wrote {len(tx_linear)} signed-linear samples to {args.tx_out}')
    if args.g711_out:
        g711 = card.encode_g711(tx_linear)
        args.g711_out.parent.mkdir(parents=True, exist_ok=True)
        args.g711_out.write_bytes(g711)
        print(f'[card] called TIKRNL PM {G711_ENCODE_ENTRY:04x}; wrote '
              f'{len(g711)} A-law octets to {args.g711_out}; '
              f'first16={" ".join(f"{value:02x}" for value in g711[:16])}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
