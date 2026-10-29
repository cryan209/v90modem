#!/usr/bin/env python3
"""Check that retained data is only kept because something reads it.

Usage: tools/check_reference_data.py [--docs]

Retention rule (docs/artifact_storage.md): `artifacts/` and `captures/` are
disposable and expire; only `reference-data/` is kept, and every dataset there
that tracked code, tests or tools name must have a row in
`reference-data/index.tsv` saying what consumes it.

Fails (exit 1) when tracked code, tests, tools or the makefile
  - name a `reference-data/{artifacts,captures}/<dataset>` absent from
    index.tsv, or
  - read from the disposable `artifacts/<run>` or `captures/<run>` trees,
    which `tools/expire_artifacts.py` deletes.
Docs and code comments may cite runs as provenance, but a citation never
obliges anyone to keep the run; `--docs` lists the ones in docs for review.
"""

import argparse
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
INDEX = "reference-data/index.tsv"

NAME = r"((?:artifacts|captures)/[A-Za-z0-9][A-Za-z0-9._@+-]*\*?)"
RETAINED = re.compile(r"reference-data/" + NAME)
# A disposable run named as a path: not part of reference-data/..., a longer
# word such as "test-artifacts/", or another checkout's tree joined on as
# root / 'artifacts/...'.
DISPOSABLE = re.compile(r"(?<![A-Za-z0-9_./-])(?<!/ ')(?<!/')(?<!/ \")(?<!/\")" + NAME)
C_BLOCK = re.compile(r"/\*.*?(\*/|\Z)", re.S)
C_LIKE = (".c", ".h", ".cc", ".cpp", ".js", ".mjs", ".ts")
HASH_LIKE = (".py", ".sh", ".bash", ".zsh", ".mak", ".tsv", ".list", ".toml",
             ".yaml", ".yml", ".cfg", ".conf")

# Tracked readers of disposable runs that are allowed to break when the run
# expires: rig one-offs and a GUI's default scratch home.
ALLOWED = {
    "tools/k56flex_upstream_rig/rerun_poke.py",
    "tools/modem_gui.py",
}

# Files whose job is to describe the layout, not to consume data.
EXEMPT = {
    ".gitignore",
    INDEX,
    "tools/check_reference_data.py",
    "tools/expire_artifacts.py",
    "tools/test_retention.py",
}


def tracked():
    out = subprocess.run(["git", "ls-files", "-z"], cwd=ROOT, check=True,
                         capture_output=True).stdout
    return [p for p in out.decode().split("\0") if p]


def code_lines(path, lines):
    """Yield (lineno, text) with comments and docstrings blanked: a run named
    in a comment is provenance, not a read."""
    if path.endswith(C_LIKE):
        text = C_BLOCK.sub(lambda m: "\n" * m.group().count("\n"), "".join(lines))
        for n, line in enumerate(text.split("\n"), 1):
            yield n, line.split("//", 1)[0]
    elif path.endswith(HASH_LIKE) or os.path.basename(path) in ("makefile", "Makefile"):
        doc = None
        for n, line in enumerate(lines, 1):
            if path.endswith(".py"):
                for q in ('"""', "'''"):
                    if doc in (None, q) and line.count(q) % 2:
                        doc = None if doc else q
                        line = ""
                        break
                if doc:
                    continue
            yield n, re.sub(r"(^|\s)#.*", "", line)
    else:
        yield from enumerate(lines, 1)


def is_doc(path):
    return (path.startswith("docs/") or path.endswith((".md", ".txt", ".html"))
            or os.path.basename(path) in ("CLAUDE.md", "AGENTS.md"))


def is_indexed(name, known):
    """A citation ending in '-' or '*' is a wildcard family of runs."""
    name = name.rstrip(".")
    if name.endswith(("*", "-")):
        prefix = name.rstrip("*")
        return any(k.startswith(prefix) for k in known)
    return name in known


def indexed():
    names = set()
    with open(os.path.join(ROOT, INDEX)) as f:
        for line in f:
            path = line.split("\t", 1)[0]
            if path.startswith("reference-data/"):
                parts = path[len("reference-data/"):].split("/")
                if len(parts) >= 2:
                    names.add("/".join(parts[:2]))
    return names


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--docs", action="store_true",
                    help="also list disposable runs cited by docs")
    args = ap.parse_args()

    known = indexed()
    errors, doc_cites = [], {}
    for path in tracked():
        if path in EXEMPT or path.startswith("reference-data/"):
            continue
        try:
            with open(os.path.join(ROOT, path), encoding="utf-8") as f:
                lines = f.readlines()
        except (UnicodeDecodeError, IsADirectoryError, FileNotFoundError):
            continue
        if not any("artifacts/" in l or "captures/" in l for l in lines):
            continue
        if is_doc(path):
            for line in lines:
                for m in DISPOSABLE.finditer(line):
                    doc_cites.setdefault(m.group(1).rstrip("."), path)
            continue
        for n, line in code_lines(path, lines):
            for m in RETAINED.finditer(line):
                if not is_indexed(m.group(1), known):
                    errors.append(f"{path}:{n}: reference-data/{m.group(1)} "
                                  f"has no row in {INDEX}")
            if path in ALLOWED:
                continue
            for m in DISPOSABLE.finditer(line):
                errors.append(f"{path}:{n}: reads disposable {m.group(1)}; "
                              f"move it to reference-data/ and index it")

    for e in errors:
        print(e)
    if args.docs:
        for run, path in sorted(doc_cites.items()):
            print(f"note: {path} cites {run} (provenance only, may expire)")
    if errors:
        print(f"{len(errors)} retention error(s); see docs/artifact_storage.md")
        return 1
    print(f"ok: {len(known)} indexed datasets, no unindexed or disposable reads")
    return 0


if __name__ == "__main__":
    sys.exit(main())
