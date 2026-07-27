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

Usage:
    python3 tools/dial_tikrnl_drive.py --freq 2100 --frames 200
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
                     ('call', [ctypes.c_void_p, ctypes.c_uint16, ctypes.c_uint16])]:
    getattr(ADSP, 'adsp2181_' + _name).argtypes = _args
ADSP.adsp2181_pm.restype = ctypes.POINTER(ctypes.c_uint32)
ADSP.adsp2181_dm.restype = ctypes.POINTER(ctypes.c_uint16)
ADSP.adsp2181_pc.restype = ctypes.c_uint16
ADSP.adsp2181_idle.restype = ctypes.c_int

KERNEL = 'artifacts/eicon-dsp/build-117-926/kernel/0009-diva-server-pri-30m-kernel'
TIKRNL = 'artifacts/eicon-dsp/build-117-926/tikrnl/0258-tikrnl81.f34-task'
OVERLAYS = 'artifacts/eicon-dsp/overlays'   # card-type 56 (PRI 30M / .F34) set
DIAL_ID = 0x0262                            # the bootpage the card starts on

# TIKRNL entry points (download 0x0258).
TASK_ENTRY = 0x0672      # download symbol 0: init + register with the kernel
FRAME_ENTRY = 0x06BB     # per-frame loop head: command dispatch -> overlay
FRAME_ENTRY_NO_HOST = 0x06C1  # same loop, past the host-command fetch/dispatch
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

# Data-pump database (ADDSP V.90 guide §5.3).
DM_LINE_RX = 0x3F08
DM_LINE_TX = 0x3F09
DM_BOOTPAGE = 0x3FB0     # bootpage_nr / DIAL state selector
DM_VEC_A = 0x3FB2        # DIAL primary action vector
DM_VEC_B = 0x3FB3        # DIAL secondary action vector
DM_STATUS = 0x3FC1

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
                 max_downloads: int = 8, host_dispatch: bool = False):
        self.log = log
        self.serve = serve
        self.max_downloads = max_downloads
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

        # Only now is it safe to download the overlay: the task init cleared
        # PM 0x0900-0x1DFF on its way through.
        if self.download_overlay(DIAL_ID) is None:
            raise SystemExit(f'no extracted image for download 0x{DIAL_ID:04x}')
        self.served.clear()   # the boot page is not a page *switch*

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
            wanted = self.dm[DM_DOWNLOAD_REQ]
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
        return hist


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
    print(f'[card] kernel + TIKRNL + DIAL up; the task claimed '
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
    for f in range(args.frames):
        hist = card.frame(tone[f % len(tone)], index=f)
        totals.update(hist)
        state = card.dm[DM_BOOTPAGE]
        states[state] += 1
        tx.append(card.dm[DM_LINE_TX])
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
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
