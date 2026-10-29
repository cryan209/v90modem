#!/usr/bin/env python3
"""Delete disposable runs in artifacts/ and captures/ once they go stale.

Usage: tools/expire_artifacts.py [--days N] [--delete] [ROOT...]

A run is a top-level directory (or file) under each ROOT (default:
artifacts/ and captures/ in the repository).  It expires when nothing inside
it has been modified for --days (default 14), so a run still being written
is never touched.  Without --delete this only lists what would go.

Kept regardless of age:
  - a run containing a file named KEEP (put the reason in it), and
  - a run holding a git worktree or checkout (a `.git` entry anywhere in it).

Anything worth keeping past that belongs in reference-data/ with a row in
reference-data/index.tsv naming its consumer (docs/artifact_storage.md).
"""

import argparse
import os
import shutil
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_ROOTS = [os.path.join(REPO, "artifacts"), os.path.join(REPO, "captures")]


def scan(path):
    """Return (newest mtime, bytes, reason to keep or None) for one run."""
    if not os.path.isdir(path) or os.path.islink(path):
        st = os.lstat(path)
        return st.st_mtime, st.st_size, None
    newest, size, keep = os.lstat(path).st_mtime, 0, None
    for dirpath, dirnames, filenames in os.walk(path):
        if ".git" in dirnames or ".git" in filenames:
            keep = "git worktree at " + os.path.relpath(dirpath, path)
            dirnames[:] = [d for d in dirnames if d != ".git"]
        if dirpath == path and "KEEP" in filenames:
            keep = "KEEP"
        for name in filenames + dirnames:
            try:
                st = os.lstat(os.path.join(dirpath, name))
            except FileNotFoundError:
                continue
            newest = max(newest, st.st_mtime)
            if name in filenames:
                size += st.st_size
    return newest, size, keep


def mb(n):
    return f"{n / 1048576:8.1f} MB"


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--days", type=float, default=14)
    ap.add_argument("--delete", action="store_true",
                    help="actually delete (default: list only)")
    ap.add_argument("roots", nargs="*", default=DEFAULT_ROOTS)
    args = ap.parse_args()

    cutoff = time.time() - args.days * 86400
    expired = kept = 0
    for root in args.roots:
        if not os.path.isdir(root):
            continue
        for name in sorted(os.listdir(root)):
            path = os.path.join(root, name)
            newest, size, keep = scan(path)
            if newest >= cutoff:
                continue
            age = (time.time() - newest) / 86400
            rel = os.path.relpath(path, REPO)
            if keep:
                kept += size
                print(f"keep    {mb(size)} {age:5.0f}d {rel} ({keep})")
                continue
            expired += size
            print(f"{'delete' if args.delete else 'expired'} {mb(size)} "
                  f"{age:5.0f}d {rel}")
            if args.delete:
                if os.path.isdir(path) and not os.path.islink(path):
                    shutil.rmtree(path)
                else:
                    os.remove(path)
    verb = "deleted" if args.delete else "would delete (rerun with --delete)"
    print(f"{verb}: {mb(expired).strip()}; kept past {args.days:g} days: "
          f"{mb(kept).strip()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
