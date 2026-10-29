#!/usr/bin/env python3
"""Run the test lists in parallel and report every failure, not just the first.

Usage: tools/run_tests.py [-j N] [--group NAME] [--filter REGEX] [--timeout S] LIST...

A list file holds one shell command per line, run from the repository root;
blank lines and lines starting with '#' are ignored, and '@group a b' tags
the rows that follow for --group.  Each command runs in
its own scratch directory (TMPDIR and ME_DUMP_DIR point there), so tests that
write scratch files or engine dumps cannot collide.  Output goes to
test-logs/<n>.log (newest) and <n>.log.1 (previous), each capped at 4 MiB
by default; older output is discarded. The summary lists failures with their
logs and the slowest tests. test-logs/timings.tsv records every duration so a
slow test can be moved to slow.list.

Exit status is 1 if anything failed or timed out.
"""

import argparse
import os
import re
import select
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from concurrent.futures import ThreadPoolExecutor, as_completed

from rotate_log import RotatingLog

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def read_lists(paths, group=None):
    """'@group a b' tags the rows that follow, until the next '@group'."""
    tests = []
    for path in paths:
        groups = []
        with open(path) as f:
            for lineno, line in enumerate(f, 1):
                cmd = line.strip()
                if cmd.startswith("@group"):
                    groups = cmd.split()[1:]
                elif cmd and not cmd.startswith("#"):
                    if group is None or group in groups:
                        tests.append((os.path.basename(path), lineno, cmd))
    return tests


def run_one(index, test, logdir, timeout, log_segment_bytes=4 * 1024 * 1024):
    list_name, lineno, cmd = test
    scratch = tempfile.mkdtemp(prefix="v90t-")
    dump = os.path.join(scratch, "dump")
    os.mkdir(dump)
    env = dict(os.environ, TMPDIR=scratch, ME_DUMP_DIR=dump)
    log = os.path.join(logdir, "%04d.log" % index)
    start = time.monotonic()
    with RotatingLog(log, log_segment_bytes) as out:
        out.write(("# %s:%d\n# %s\n" % (list_name, lineno, cmd)).encode())
        # Drain output continuously, even after rotation. Stopping at the cap
        # would block the child's pipe and turn noisy tests into timeouts.
        # Own session, so a timeout can kill the engine peers a test forks too.
        proc = subprocess.Popen(["bash", "-c", cmd], cwd=ROOT, env=env,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                stdin=subprocess.DEVNULL, start_new_session=True)
        timed_out = False
        exited_at = None
        eof = False
        try:
            while True:
                now = time.monotonic()
                if proc.poll() is None and now - start >= timeout and not timed_out:
                    timed_out = True
                    try:
                        os.killpg(proc.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                if proc.poll() is not None:
                    if exited_at is None:
                        exited_at = now
                    # A background descendant may still hold the pipe open.
                    # Do not wait indefinitely after the test itself exits.
                    if eof or now - exited_at >= 1.0:
                        break
                if eof:
                    time.sleep(0.01)
                elif select.select([proc.stdout], [], [], 0.1)[0]:
                    data = os.read(proc.stdout.fileno(), 65536)
                    if data:
                        out.write(data)
                    else:
                        eof = True
            code = proc.wait()
        finally:
            proc.stdout.close()
            if proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                proc.wait()
        status = "TIMEOUT" if timed_out else ("PASS" if code == 0 else "FAIL")
        if timed_out:
            code = None
    elapsed = time.monotonic() - start
    if status == "PASS":
        shutil.rmtree(scratch, ignore_errors=True)
    return index, test, status, code, elapsed, log, scratch


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("lists", nargs="+")
    ap.add_argument("-j", "--jobs", type=int, default=os.cpu_count() or 4)
    ap.add_argument("--filter", help="only run commands matching this regex")
    ap.add_argument("--group", help="only run rows tagged with this @group")
    ap.add_argument("--timeout", type=float, default=900.0,
                    help="seconds before a test is killed (default 900)")
    ap.add_argument("--log-segment-mib", type=float, default=4,
                    help="size of each of two retained output segments per test (default 4 MiB)")
    ap.add_argument("--logdir", default=os.path.join(ROOT, "test-logs"))
    args = ap.parse_args()
    if not 0 < args.log_segment_mib < float("inf"):
        ap.error("--log-segment-mib must be positive and finite")
    log_segment_bytes = int(args.log_segment_mib * 1024 * 1024)
    if log_segment_bytes < 1:
        ap.error("--log-segment-mib must be at least one byte")

    tests = read_lists(args.lists, args.group)
    if args.filter:
        rx = re.compile(args.filter)
        tests = [t for t in tests if rx.search(t[2])]
    if not tests:
        print("no tests selected")
        return 1
    shutil.rmtree(args.logdir, ignore_errors=True)
    os.makedirs(args.logdir)

    tty = sys.stdout.isatty()
    wall = time.monotonic()
    results = []
    with ThreadPoolExecutor(max_workers=args.jobs) as pool:
        futures = [pool.submit(run_one, i, t, args.logdir, args.timeout,
                               log_segment_bytes)
                   for i, t in enumerate(tests)]
        for done, fut in enumerate(as_completed(futures), 1):
            r = fut.result()
            results.append(r)
            _, test, status, _, elapsed, _, _ = r
            if status != "PASS" or not tty:
                print("%-7s %6.1fs  %s" % (status, elapsed, test[2]), flush=True)
            else:
                print("\r[%d/%d] %-70.70s" % (done, len(tests), test[2]),
                      end="", flush=True)
    wall = time.monotonic() - wall
    if tty:
        print()

    results.sort(key=lambda r: r[0])
    with open(os.path.join(args.logdir, "timings.tsv"), "w") as f:
        for _, test, status, _, elapsed, _, _ in results:
            f.write("%.2f\t%s\t%s:%d\t%s\n" % (elapsed, status, test[0], test[1], test[2]))

    failed = [r for r in results if r[2] != "PASS"]
    print("\nlogs: newest .log, previous .log.1; up to %.2f MiB each"
          % args.log_segment_mib)
    print("\nslowest:")
    for _, test, _, _, elapsed, _, _ in sorted(results, key=lambda r: -r[4])[:8]:
        print("  %6.1fs  %s" % (elapsed, test[2]))
    print("\n%d passed, %d failed, %.0fs wall, -j %d"
          % (len(results) - len(failed), len(failed), wall, args.jobs))
    for _, test, status, code, elapsed, log, scratch in failed:
        print("  %s (%s) %s:%d  %s\n      log %s  scratch %s"
              % (status, "exit %s" % code if code is not None else "killed",
                 test[0], test[1], test[2], log, scratch))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
