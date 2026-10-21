#!/usr/bin/env python3
"""Try a V.FC call between two original analogue Courier ROMs.

Both native CONNECT reports must explicitly say /VFC. V.32bis fallback,
carrier-only reports and substring payload checks never qualify V.FC here.
The emulator models the analogue bearer; our production audio is not touched.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--emulator', type=Path, required=True)
    parser.add_argument('--python', type=Path, required=True)
    parser.add_argument('--rom', type=Path, required=True)
    parser.add_argument('--library', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--rate', type=int, choices=range(14400, 28801, 2400),
                        default=28800)
    parser.add_argument('--instructions', type=int, default=300000000)
    v8_options = parser.add_mutually_exclusive_group()
    v8_options.add_argument('--enable-v8', action='store_true',
                           help='set S54=0 on both peers for a negotiation comparison')
    v8_options.add_argument('--disable-v8', action='store_true',
                           help='set S54=192 on both peers for proprietary startup')
    v8_options.add_argument('--inherit-v8', action='store_true',
                           help='retain NVRAM S54 for the original negative control')
    parser.add_argument('--automode', action='store_true',
                        help='use &N0 rather than a fixed rate (still requires /VFC)')
    parser.add_argument('--trace-selection', action='store_true',
                        help='observe firmware V.FC eligibility and overlay-request paths')
    parser.add_argument('--capture-signature', choices=('rx', 'tx'),
                        help='read-only native capture of signature dibits and decoder state')
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    emulator = args.emulator.resolve(strict=True)
    rom = args.rom.resolve(strict=True)
    library = args.library.resolve(strict=True)
    markers = {'a': 'VFC-CALLER-0123456789', 'b': 'VFC-ANSWER-9876543210'}
    # Courier &N8=14400, &N14=28800. S56 bit 6 disables V.34 while
    # leaving V.FC bit 7 clear. S58=33 disables x2 and V.90. No &W.
    # V.8 takes priority over the proprietary ROCK exchange, even when V.34
    # itself is disabled. Default to genuine V.FC startup, not V.32 fallback.
    v8 = '' if args.inherit_v8 else 'S54=0' if args.enable_v8 else 'S54=192'
    speed = 0 if args.automode else 8 + (args.rate-14400)//2400
    setting = f'ATX1S27=1S39=8{v8}S56=64S58=33&N{speed}&M0&B0&A3'
    with tempfile.TemporaryDirectory(prefix='vfc-peer-') as temporary:
        # Preserve a venv's executable symlink: resolving it would silently
        # select the base interpreter and a different set of dependencies.
        command = [str(args.python.absolute()), '-m', 'courier_emu', 'link', str(rom),
                   '--with-dsp', '--nvram-fixture', 'idsdl403', '--board-id', '7',
                   '--tick-ms', '5',
                   '--socket', str(Path(temporary) / 'line.sock'),
                   '--a-dip-preset', 'default', '--b-dip-preset', 'default',
                   '--a-at', setting + 'DT5551234', '--b-at', setting + 'A',
                   '--answer-on-ring', '--instructions', str(args.instructions),
                   '--audio-dir', str(output), '--a-send', markers['a'],
                   '--b-send', markers['b'], '--summary']
        if args.trace_selection:
            for address in ('04c4', '04c5', '04c6', '04dd', '04f7', '04f8',
                            '0a7e', '0179', '017b', '0d28'):
                command.extend(('--peek', address))
            # Supervisor per-instruction PC hooks disable its fast execution
            # path. Native DSP tracing and end-state peeks keep this bounded.
            for address in ('0026', '006f', '086f', '087f', '039a', '039b', '03cd'):
                command.extend(('--dsp-peek', address))
            command.extend(('--dsp-trace-range', 'a300:a440'))
        env = {**os.environ, 'PYTHONDONTWRITEBYTECODE': '1',
               'COURIER_C5X_LIBRARY': str(library), 'COURIER_LINE_FRAME_MS': '20'}
        if args.capture_signature:
            command[1:3] = [str(Path(__file__).with_name('vfc_courier_capture.py').resolve()), 'cli']
            env['VFC_CAPTURE_EMULATOR'] = str(emulator)
            env['VFC_CAPTURE_PC'] = 'aa44' if args.capture_signature == 'rx' else 'a719'
        manifest = {'command': command, 'rom_sha256': hashlib.sha256(rom.read_bytes()).hexdigest(),
                    'library_sha256': hashlib.sha256(library.read_bytes()).hexdigest(),
                    'settings': setting, 'capture_signature': args.capture_signature,
                    'capture_pc': env.get('VFC_CAPTURE_PC'),
                    'scope': 'native firmware pair, no production V.FC datapump'}
        (output / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        with (output / 'result.json').open('w') as out, (output / 'stderr.log').open('w') as err:
            process = subprocess.Popen(command, cwd=emulator, env=env, stdout=out,
                                       stderr=err, start_new_session=True)
            try:
                returncode = process.wait(timeout=180)
            except BaseException:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                raise
    result = json.loads((output / 'result.json').read_text()) if returncode == 0 else {}
    checks, terminals = {}, {}
    for role, opposite in (('a', 'b'), ('b', 'a')):
        side = result.get(role, {})
        # Hayes formatter is ASCII; payload preserves the 7E1 parity bit.
        raw = bytes.fromhex(side.get('serial_hex', ''))
        text = bytes(b & 127 for b in raw).decode('ascii')
        terminals[role] = text
        checks[role + '_vfc'] = bool(re.search(r'\r\nCONNECT +\d+/VFC(?:/[^\r\n]*)?\r\n', text, re.I))
        checks[role + '_marker'] = markers[opposite] in text
        checks[role + '_execution'] = not (side.get('error') or side.get('dsp_bridge', {}).get('error'))
    summary = {'passed': returncode == 0 and all(checks.values()),
               'checks': checks, 'terminals': terminals, 'returncode': returncode,
               'scope': 'initial native V.FC carrier/marker evidence only; no long traffic qualification'}
    (output / 'verification.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
