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

  --overlay  trace the `0x0c37` profile-installer seam, where the analysis
             result pointer fails to reset and overruns into the detector
             action lists.  State `0x0c37`'s record field DM(0x164b)=0x0040
             should dispatch PM 0x36ae, which stores 0x3716 into the action
             slot DM(0x0e53); PM 0x3716 in turn calls PM 0x3231 to rewind the
             20-word result buffer DM(0x0ddd..0x0df0) through its pointer
             DM(0x15f3).

             All three installer addresses are above PM 0x2000, so each one
             names a *different* instruction on each PMOVLAY page.  Exec
             watchpoints therefore log pmovlay and the fetched word at every
             execution ([EXEC] lines on stderr), and the DM view prints the
             seam's words whenever any of them changes, with the installer
             entry points disassembled as currently resolved.  If the words
             at 0x36ae do not disassemble as the documented
             `I4 = 0x0e53 ; DM(I4,M5) = 0x3716` store, the wrong overlay page
             is mapped and the fault is ours, not the firmware's.

Usage:
    python3 tools/eicon_info_replay.py CAPTURE.rx.ulaw --tx
    python3 tools/eicon_info_replay.py CAPTURE.rx.ulaw --states --from 6.1 --to 6.8
    python3 tools/eicon_info_replay.py CAPTURE.rx.ulaw --tx --inject-event 0x37
    python3 tools/eicon_info_replay.py CAPTURE.rx.ulaw --overlay 2> EXEC.log
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from adsp2181_dis import disas
# ADSP comes through dial_tikrnl_drive, as dial_kernel_dispatch takes it:
# importing eicon_mips_shim for it would drag in the MIPS emulator too.
from dial_tikrnl_drive import ADSP
from dial_kernel_dispatch import LiveKernelModem

SAMPLE_RATE = 8000
DM_TRNPROGRESS = 0x3FC2
DM_EVENT = 0x198E

# The `0x0c37` profile-installer seam.  DM(0x15f3) walks the 20-word analysis
# result buffer DM(0x0ddd..0x0df0), whose end abuts the detector action slots
# DM(0x0e4c) / DM(0x0e53) -- which is why failing to rewind it is destructive.
DM_ANALYSIS_COUNT = 0x06E6
DM_RECORD_DISPATCH = 0x164B
DM_RESULT_POINTER = 0x15F3
DM_ACTION_SLOT_A = 0x0E4C
DM_ACTION_SLOT_B = 0x0E53
RESULT_BUFFER = (0x0DDD, 0x0DF0)

OVERLAY_SEAM = {
    'trn': DM_TRNPROGRESS, 'internal': 0x1652,
    'count': DM_ANALYSIS_COUNT, 'dispatch': DM_RECORD_DISPATCH,
    'resultp': DM_RESULT_POINTER,
    'slot4c': DM_ACTION_SLOT_A, 'slot53': DM_ACTION_SLOT_B,
}
# PM 0x36ae installs 0x3716; 0x3716 calls 0x3231, which rewinds the pointer.
INSTALLER_CHAIN = (0x36AE, 0x3716, 0x3231)

# The sequencer loop, PM 0x3335, as the resident INFO page disassembles:
#
#   3335  AY0 = DM($1647) ; AR = AY0 - 1 ; IF LT AR = 0 ; DM($1647) = AR
#   3339  I4 = DM($169A) ; CALL (I4) ; IF LE JUMP $334E    <- pre-condition
#   333c  MR0 = DM($1692) ; I4 = DM($1696) ; CALL ; IF LE JUMP $334D
#   3340  MR0 = DM($1693) ; I4 = DM($1697) ; CALL ; IF LE JUMP $334D
#   3344  MR0 = DM($1694) ; I4 = DM($1698) ; CALL ; IF LE JUMP $334D
#   3348  MR0 = DM($1695) ; I4 = DM($1699) ; CALL ; IF LE JUMP $334D
#   334c  RTS                                              <- nothing matched
#   334d  DM($1679) = MR0                                  <- record selected
#   334e  I4 = DM($1679) ; MR1 = $0019 ; I6 = DM($169F) ; CALL (I6)
#   3353  DM($3FC2) = DM($1652) AND $00FF                  <- TrnProgress
#   3357  translate DM(0x1653..56) through 0x133e -> DM(0x1692..95)
#   335c  translate DM(0x1657..5b) through 0x131e -> DM(0x1696..9a)
#   3361  CALL $3435   <- diff the raw fields, dispatch the changed handlers
#   3362  JUMP $3339
#
# PM 0x169f is the record applier (PM 0x336a), which walks (offset, lo, hi)
# triples writing DM(0x1642 + offset).  DM(0x164b) is offset 9, so it has
# exactly one writer: whichever record DM(0x1679) was pointed at.
PM_CONDITION_TEST = {0x333D: 0, 0x3341: 1, 0x3345: 2, 0x3349: 3}
PM_RECORD_SELECTED = 0x334D
PM_RECORD_APPLY = 0x334E
SEQUENCER_TRACE = tuple(PM_CONDITION_TEST) + (PM_RECORD_SELECTED,
                                              PM_RECORD_APPLY)

# The candidate records and their condition handlers, as translated into place
# by PM 0x3357/0x335c on the previous pass.
SEQUENCER_STATE = {
    'trn': DM_TRNPROGRESS, 'internal': 0x1652, 'count': DM_ANALYSIS_COUNT,
    'record': 0x1679, 'dispatch': DM_RECORD_DISPATCH,
    'next0': 0x1692, 'next1': 0x1693, 'next2': 0x1694, 'next3': 0x1695,
    'test0': 0x1696, 'test1': 0x1697, 'test2': 0x1698, 'test3': 0x1699,
    'pre': 0x169A,
}

# The INFO sequencer's working set, PM 0x3335.
SEQUENCER = {
    'trn': 0x3FC2, 'internal': 0x1652, 'vector': 0x1679, 'entry': 0x169F,
    'next0': 0x1692, 'next1': 0x1693, 'next2': 0x1694, 'next3': 0x1695,
    'test0': 0x1696, 'test1': 0x1697, 'test2': 0x1698, 'test3': 0x1699,
    'pre': 0x169A,
}
# Shown alongside, but excluded from the change key: these move every sample.
EXTRA = {'timer': 0x1647, 'doneA': 0x0686, 'eventB': 0x198E}
# The same exclusion for the sequencer view: PM 0x3335 decrements the dwell
# timer on every pass, so keying on it prints one line per sample.
# dwell is DM(0x1650), the counter the pre-condition PM 0x3391 decrements; the
# chain only advances to the next record when it reaches 0.
SEQUENCER_EXTRA = {'timer': 0x1647, 'dwell': 0x1650}


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
    ap.add_argument('--overlay', action='store_true',
                    help='trace the 0x0c37 installer seam and its PMOVLAY page')
    ap.add_argument('--sequencer', action='store_true',
                    help='trace PM 0x3335 record selection: the candidates, '
                         'their conditions, and the record DM(0x164b) came from')
    ap.add_argument('--sport-companding', action='store_true',
                    help='expand the DS0 octet at SPORT0 as the native path '
                         'does (ADDSP V.90 guide §3.3); without it this replay '
                         'runs the pre-Session-44 compressed domain')
    ap.add_argument('--from', dest='start', type=float, default=0.0)
    ap.add_argument('--to', dest='end', type=float, default=1e9)
    args = ap.parse_args()

    if not args.states and not args.tx and not args.overlay and not args.sequencer:
        args.tx = True

    data = args.capture.read_bytes()
    modem = LiveKernelModem(expand_sport=args.sport_companding)
    modem.boot()
    modem.configure_modem('answer', 'pcmu')
    dm = modem.card.dm
    cpu = modem.card.cpu

    if args.overlay:
        for address in INSTALLER_CHAIN:
            ADSP.adsp2181_watch_exec(cpu, address, 1)
        print(f'watching PM {"/".join(f"0x{a:04x}" for a in INSTALLER_CHAIN)}'
              f' -- [EXEC] lines carry pmovlay and the fetched word, on stderr')

    if args.sequencer:
        for address in SEQUENCER_TRACE:
            ADSP.adsp2181_watch_exec(cpu, address, 1)
        print('watching PM ' + '/'.join(f'0x{a:04x}' for a in SEQUENCER_TRACE)
              + ' -- on stderr, mr0 at 0x334d is the selected record; the last'
                ' 0x333d/41/45/49 before it names the condition that matched')

    previous = None
    overlay_previous = None
    sequencer_previous = None
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

        if args.sequencer and args.start <= seconds <= args.end:
            key = tuple(dm[address] for address in SEQUENCER_STATE.values())
            if key != sequencer_previous:
                fields = ' '.join(f'{name}={dm[address]:04x}'
                                  for name, address in SEQUENCER_STATE.items())
                extra = ' '.join(f'{name}={dm[address]:04x}'
                                 for name, address in SEQUENCER_EXTRA.items())
                print(f'{seconds:8.4f}  {fields} | {extra}')
                sequencer_previous = key

        if args.overlay and args.start <= seconds <= args.end:
            key = tuple(dm[address] for address in OVERLAY_SEAM.values())
            if key != overlay_previous:
                fields = ' '.join(f'{name}={dm[address]:04x}'
                                  for name, address in OVERLAY_SEAM.items())
                pointer = dm[DM_RESULT_POINTER]
                low, high = RESULT_BUFFER
                where = ('below' if pointer < low else
                         'in' if pointer <= high else 'OVERRUN')
                pmovlay = ADSP.adsp2181_pmovlay(cpu)
                print(f'{seconds:8.4f}  {fields} | buffer {where} '
                      f'| pmovlay={pmovlay}')
                if overlay_previous is None or where == 'OVERRUN':
                    # Only worth re-dumping when the page could have changed
                    # under us; the words are read back through PMOVLAY.
                    for address in INSTALLER_CHAIN:
                        words = [ADSP.adsp2181_read_pm(cpu, address + offset)
                                 for offset in range(3)]
                        text = '  '.join(f'{w:06x} {disas(w)}' for w in words)
                        print(f'          PM {address:04x} (ov {pmovlay}): {text}')
                overlay_previous = key

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
