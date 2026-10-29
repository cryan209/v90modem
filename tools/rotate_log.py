#!/usr/bin/env python3
"""Capture stdin in bounded rotating logs (no audio or modem-path changes).

Example: ./sip_v90_modem ... 2>&1 | python3 tools/rotate_log.py modem.log
The named file is newest; .1 is the preceding segment. Each defaults to 4 MiB.
This bounds only captured stdout/stderr, not files opened by the modem itself.
"""

import argparse
import os
import sys


class RotatingLog:
    """Binary writer retaining at most two segments, each max_bytes long."""

    def __init__(self, path, max_bytes=4 * 1024 * 1024):
        if max_bytes < 1:
            raise ValueError("max_bytes must be positive")
        self.path = os.fspath(path)
        self.max_bytes = max_bytes
        self.size = 0
        self.file = None

    def __enter__(self):
        # A new capture must not retain a segment from an unrelated old run.
        try:
            os.unlink(self.path + ".1")
        except FileNotFoundError:
            pass
        self.file = open(self.path, "wb", buffering=0)
        return self

    def write(self, data):
        data = memoryview(data)
        while data:
            if self.size == self.max_bytes:
                self.file.close()
                os.replace(self.path, self.path + ".1")
                self.file = open(self.path, "wb", buffering=0)
                self.size = 0
            n = self.file.write(data[:self.max_bytes - self.size])
            self.size += n
            data = data[n:]

    def __exit__(self, *exc):
        if self.file is not None:
            self.file.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("path", help="newest log segment (overwritten on startup)")
    ap.add_argument("--segment-mib", type=float, default=4,
                    help="size of each of the two retained segments (default 4 MiB)")
    args = ap.parse_args()
    if not 0 < args.segment_mib < float("inf"):
        ap.error("--segment-mib must be positive and finite")
    limit = int(args.segment_mib * 1024 * 1024)
    if limit < 1:
        ap.error("--segment-mib must be at least one byte")
    with RotatingLog(args.path, limit) as out:
        while True:
            data = os.read(sys.stdin.fileno(), 65536)
            if not data:
                break
            out.write(data)
    return 0


if __name__ == "__main__":
    sys.exit(main())
