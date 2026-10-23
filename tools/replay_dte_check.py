#!/usr/bin/env python3
"""Assert the actual verbose DTE CONNECT result during an engine replay.

Reads the PTY, not the engine's DATA-state log. Use the default V1/Q0 DTE
configuration; command arguments after -- are the replay invocation.
"""
import argparse
import errno
import os
from pathlib import Path
import re
import select
import subprocess
import tempfile
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--expect', choices=('connected', 'unconnected'), required=True)
    p.add_argument('--require-log', action='append', default=[])
    p.add_argument('--log', type=Path)
    p.add_argument('command', nargs=argparse.REMAINDER)
    args = p.parse_args()
    cmd = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not cmd or '--pty-link' in cmd:
        p.error('supply a replay command after -- without --pty-link')
    with tempfile.TemporaryDirectory(prefix='replay-dte-') as scratch:
        link = Path(scratch)/'dte'
        log = args.log or Path(os.environ.get('ME_DUMP_DIR', scratch))/'replay-dte-engine.log'
        fd = None
        output = bytearray()
        deadline = time.monotonic()+120
        with log.open('wb') as f:
            proc = subprocess.Popen(cmd+['--pty-link', str(link)], stdout=f, stderr=subprocess.STDOUT)
            try:
                while proc.poll() is None:
                    if time.monotonic() > deadline:
                        raise TimeoutError('replay exceeded 120 seconds')
                    if fd is None and link.exists():
                        fd = os.open(link, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
                    if fd is None:
                        time.sleep(.005)
                        continue
                    ready, _, _ = select.select([fd], [], [], .02)
                    if ready:
                        try:
                            output.extend(os.read(fd, 65536))
                        except OSError as ex:
                            if ex.errno not in (errno.EIO, errno.EAGAIN):
                                raise
                            time.sleep(.005)
                # Drain bytes already queued when the master closed.
                if fd is not None:
                    while True:
                        try:
                            b = os.read(fd, 65536)
                        except OSError as ex:
                            if ex.errno in (errno.EIO, errno.EAGAIN):
                                break
                            raise
                        if not b:
                            break
                        output.extend(b)
            finally:
                if proc.poll() is None:
                    proc.kill()
                proc.wait()
                if fd is not None:
                    os.close(fd)
        if fd is None:
            raise RuntimeError('never opened the replay DTE; no observation')
        text = log.read_text(errors='replace')
        connects = re.findall(rb'(?:^|[\r\n])CONNECT(?: [^\r\n]*)?(?=[\r\n]|$)', output)
        print('DTE:', output.decode(errors='replace').strip())
        print('replay DTE CONNECT results:', len(connects))
        if proc.returncode or any(s not in text for s in args.require_log):
            print('replay failed or required log evidence missing:', log)
            return 1
        if bool(connects) != (args.expect == 'connected'):
            print('unexpected DTE CONNECT state:', log)
            return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
