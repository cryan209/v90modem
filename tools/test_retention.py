#!/usr/bin/env python3
"""Tests for the retention tools: expire_artifacts.py and check_reference_data.py."""

import os
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import check_reference_data as crd

HERE = os.path.dirname(os.path.abspath(__file__))
OLD = time.time() - 30 * 86400


def make(path, old=True):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write("x")
    if old:
        os.utime(path, (OLD, OLD))


def age_dirs(root):
    for dirpath, dirnames, _ in os.walk(root, topdown=False):
        for d in dirnames:
            os.utime(os.path.join(dirpath, d), (OLD, OLD))


def test_expire():
    with tempfile.TemporaryDirectory() as tmp:
        root = os.path.join(tmp, "artifacts")
        make(os.path.join(root, "stale", "rx.g711"))
        make(os.path.join(root, "kept", "rx.g711"))
        make(os.path.join(root, "kept", "KEEP"))
        make(os.path.join(root, "tree", "wt", ".git"))
        make(os.path.join(root, "live", "old.log"))
        age_dirs(root)
        make(os.path.join(root, "live", "new.log"), old=False)

        run = lambda *a: subprocess.run(
            [sys.executable, os.path.join(HERE, "expire_artifacts.py"), *a, root],
            check=True, capture_output=True, text=True).stdout
        out = run()
        assert "expired" in out and "stale" in out, out
        assert os.path.isdir(os.path.join(root, "stale")), "dry run deleted"
        run("--delete")
        left = sorted(os.listdir(root))
        assert left == ["kept", "live", "tree"], left


def test_checker():
    known = {"artifacts/apple-v92-sip-r4", "artifacts/trn1d-long-ab-1"}
    assert crd.is_indexed("artifacts/apple-v92-sip-r4.", known)
    assert crd.is_indexed("artifacts/trn1d-long-ab-*", known)
    assert not crd.is_indexed("artifacts/missing", known)

    c = ["/* see artifacts/old-run */\n", "x = 1; // artifacts/old-run\n",
         "/* start\n", "   artifacts/old-run\n", "*/ f(\"artifacts/new-run\");\n"]
    reads = [m.group(1) for _, l in crd.code_lines("a.c", c)
             for m in crd.DISPOSABLE.finditer(l)]
    assert reads == ["artifacts/new-run"], reads

    py = ['"""Doc names artifacts/old-run.\n', '"""\n', "# artifacts/old-run\n",
          "p = 'artifacts/new-run'\n", "q = root / 'artifacts/other-repo'\n"]
    reads = [m.group(1) for _, l in crd.code_lines("a.py", py)
             for m in crd.DISPOSABLE.finditer(l)]
    assert reads == ["artifacts/new-run"], reads
    assert not crd.DISPOSABLE.search("reference-data/artifacts/x")
    assert not crd.DISPOSABLE.search("test-artifacts/x")


if __name__ == "__main__":
    test_expire()
    test_checker()
    print("ok")
