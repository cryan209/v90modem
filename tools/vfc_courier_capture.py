#!/usr/bin/env python3
"""Read-only native V.FC signature capture, launched by probe_vfc_firmware.py.

Only observes a firmware PC and RAM. No carrier, decoder result, sample or
guest state is supplied by this hook. The native capture retains 64 rows.
"""
import json
import os
from pathlib import Path
import sys

sys.path.insert(0, os.environ['VFC_CAPTURE_EMULATOR'])
mode = sys.argv.pop(1)
script = str(Path(__file__).resolve())
if mode == 'cli':
    from courier_emu import cli
    worker_command, link_side = cli._worker_command, cli._link_side

    def worker(*args, **kwargs):
        command = worker_command(*args, **kwargs)
        assert command[1:3] == ['-m', 'courier_emu.worker']
        return [command[0], script, 'worker', *command[3:]]

    def side(*args, **kwargs):
        command = link_side(*args, **kwargs)
        assert command[1:3] == ['-m', 'courier_emu']
        return [command[0], script, 'cli', *command[3:]]

    cli._worker_command, cli._link_side = worker, side
    raise SystemExit(cli.main())
if mode != 'worker':
    raise SystemExit('expected cli or worker')

from courier_emu.dsp import NativeC5x
addresses = [0x320, *range(0x330, 0x334), 0x326, 0x3a6, 0x3af,
             0x379, 0x3d0, 0x3d4, 0x3da, 0x3e2, 0x3c8, 0x3ca,
             0x6f, 0x3d8, 0x3d9, 0x3d1, 0x3d2]
pc = int(os.environ['VFC_CAPTURE_PC'], 16)
prefix = Path(sys.argv[sys.argv.index('--line-record') + 1])
destination = prefix.with_name(prefix.name + '-signature.json')
records = []
original_init, original_close = NativeC5x.__init__, NativeC5x.close


def initialize(self, *args, **kwargs):
    original_init(self, *args, **kwargs)
    self.set_pc_capture(pc, addresses)
    self._vfc_capture_armed = True


def close(self):
    if not self.closed and getattr(self, '_vfc_capture_armed', False):
        generation = len(records)
        prefix.with_name(prefix.name + f'-core{generation}-data.bin').write_bytes(
            b''.join(self.data(a).to_bytes(2, 'little') for a in range(65536)))
        prefix.with_name(prefix.name + f'-core{generation}-program.bin').write_bytes(
            b''.join(self.program(a).to_bytes(2, 'little') for a in range(65536)))
        records.append(self.pc_captures())
        destination.write_text(json.dumps({
            'pc': pc, 'addresses': addresses,
            'prefix': ['instructions', 'cycles', 'acc', 'dp', 'carry'],
            'capacity_per_core': 64, 'cores': records,
            'scope': 'read-only native firmware PC/RAM capture',
        }, indent=2) + '\n')
    original_close(self)


NativeC5x.__init__, NativeC5x.close = initialize, close
from courier_emu.worker import main
raise SystemExit(main())
