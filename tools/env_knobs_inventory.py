#!/usr/bin/env python3
"""Regenerate docs/env_knobs.md: every environment variable the code reads.

Usage: tools/env_knobs_inventory.py > docs/env_knobs.md   (from the repo root)

Finds names passed as string literals to getenv(), parse_env_int(), env_or(),
V34_DIAG_GETENV() and v34_diag_flag().  A name read through any other path
is missed, so keep new knobs on one of those readers.  For each name it
records where it is read, the default when the call states one, the nearest
comment, and whether any test, script, current doc or the project history
mentions it.  SETTINGS below is the hand-kept list of real settings; the rest
is classified by name.
"""

import glob
import os
import re
import subprocess
import sys

# The knobs a user is expected to set.  Everything else is a diagnostic, a
# test hook, or a switch left over from an experiment.  Keep this short.
SETTINGS = [
    ("ME_MODE", "Power-on modulation offer (`--mode`); `AT+MS` overrides it per call."),
    ("ME_V90_ROLE", "`digital` (default) or `analogue` for V.90/V.92."),
    ("ME_V92_PCM_UPSTREAM", "Offer V.92 PCM upstream (Table 18)."),
    ("ME_DATA_FRAMING", "Force `v14` or `lapm`; normally `AT+ES` decides."),
    ("ME_DATA_COMPRESSION", "Force a compressor; normally `AT+DS`/`AT+DS44` decide."),
    ("ME_V42_T400_MS", "V.42 T400 (9.1.1)."),
    ("ME_JB_MS", "Fixed RX jitter buffer depth in ms (default 200; 0 = pjmedia adaptive)."),
    ("ME_V8", "0 replaces V.8 with V.25 ANS."),
    ("ME_V8BIS", "1 runs V.8bis before V.8."),
    ("ME_V8_ADVERTISE_V32", "0 withdraws the V.32 bit from V.8."),
    ("ME_V8_ADVERTISE_V91", "1 offers V.91 in V.8."),
    ("ME_V8_TX_POWER_DBM0", "V.8 transmit level."),
    ("ME_V34_TX_DBM0", "V.34 nominal transmit level before INFO1 power reduction (default -14)."),
    ("ME_V25_ANS_AA", "1 answers plain ANS with V.32bis AA (Annex A.2.1.3)."),
    ("ME_V22_LEGACY", "0 withdraws the V.8-less V.22bis (USB1) path."),
    ("ME_V22_GUARD", "V.22bis guard tone in Hz (0, 550 or 1800)."),
    ("ME_V34_BAUD", "V.34 start symbol rate."),
    ("ME_V34_BPS", "V.34 start bit rate ceiling (0 = the symbol rate's maximum)."),
    ("ME_V90_MAX_TX_DBM0_CODE", "INFO0d bits 33:37, the transmit power we can honour."),
    ("ME_V92_MH", "1 arms V.92 modem-on-hold (9.10)."),
    ("ME_K56FLEX", "1 runs the K56flex path (experimental)."),
    ("ME_CLEARMODE", "1 offers RFC 4040 CLEARMODE for CLEAR/V.110/V.120."),
    ("ME_V110_SYNC", "1 runs V.110 synchronous."),
    ("ME_V120_ACK", "1 runs V.120 acknowledged mode."),
    ("ME_V120_VERIFY", "1 runs V.120 XID link verification."),
    ("ME_V120_COMPRESS", "1 runs V.120 Annex C compression (needs ME_V120_ACK)."),
    ("SIP_FORCE_PCMU", "1 offers PCMU only."),
    ("ME_DUMP_DIR", "Directory for engine PCM dumps; set one per process."),
    ("ME_G711_CAPTURE", "Record the wire codewords both ways."),
    ("VPCM_ME_VERBOSE", "Engine log verbosity."),
]

# Switches kept deliberately although nothing in the tree sets them, with the
# reason, so the removal backlog does not list them.
KEPT = {
    "ME_V90_UPSTREAM_T2": "pending experiment: the V.90 upstream through the ordinary V.34 receiver has never been compared",
    "ME_V90_PHASE_SWEEP": "diagnostic mode: holds one upstream frame-phase candidate for grading",
    "ME_V92_CPD_GAIN_PER_LU": "interop: slmodemd's G x LU convention (stops its V.92 upstream railing)",
    "ME_V90_CP_BAUD_CODE": "offline CP search control (v90_cp_live.c)",
    "ME_V90_CP_BROAD_MAP": "offline CP search control (v90_cp_live.c)",
    "ME_V90_CP_CARRIER": "offline CP search control (v90_cp_live.c)",
    "ME_V90_CP_CARRIER_STEP": "offline CP search control (v90_cp_live.c)",
    "ME_V90_CP_DIRECT_CARRIER_STEP": "offline CP search control (v90_cp_live.c)",
    "ME_V90_CP_DISABLE_DIRECT": "offline CP search control (v90_cp_live.c)",
    "ME_V90_CP_ENABLE_ADAPTIVE_FALLBACK": "offline CP analysis: the slow adaptive receiver",
    "ME_V90_CP_FREEZE_SAMPLE": "offline CP search control (v90_cp_live.c)",
    "ME_V90_CP_MAP": "offline CP search control (v90_cp_live.c)",
    "ME_V90_CP_ORDER": "offline CP search control (v90_cp_live.c)",
    "ME_V90_CP_SEARCH_END": "offline CP search control (v90_cp_live.c)",
    "ME_V90_CP_SEARCH_START": "offline CP search control (v90_cp_live.c)",
    "ME_V90_CP_TIMING": "offline CP search control (v90_cp_live.c)",
    "ME_V90_SMARTLINK_DUMMY_CPT": "interop: the only way to enable v90_repair_smartlink_dummy_cpt() (tested in vpcm_loopback_test)",
    "ME_TRAINING_TIMEOUT_MS": "interop: lengthens the 60 s training cap for multi-retrain calls",
    "V8_GUARD_TONE_HZ": "national V.8 guard-tone option, off by default",
    "V8_GUARD_TONE_LEVEL": "national V.8 guard-tone option, off by default",
    "ME_V92_MH_INITIATE": "test hook: provokes V.92 9.10 modem-on-hold against a real peer",
    "ME_V92_MH_GRANT": "test hook: V.92 9.10 modem-on-hold grant/deny",
    "ME_V90A_8K_FEED": "open investigation: V.90 analogue role over a real loop",
    "ME_V90A_DIL_REQUIRE_PLAN": "open investigation: V.90 analogue role over a real loop",
    "ME_V90A_TRN1D_ADAPT": "open investigation: V.90 analogue role over a real loop",
    "ME_V90A_TRN1D_MU": "open investigation: V.90 analogue role over a real loop",
    "ME_K56FLEX_BLIND": "K56flex: area under active work, not touched in the knob clean-up",
    "ME_K56FLEX_RATE": "K56flex: area under active work, not touched in the knob clean-up",
    "ME_K56FLEX_ROLE": "K56flex: area under active work, not touched in the knob clean-up",
    "ME_X2_SYMMETRIC": "x2: alongside K56flex, not touched in the knob clean-up",
    "ME_V34_TX_PREEMP": "diagnostic: forces Phase 3 pre-emphasis to measure a peer against a flat spectrum",
    "ME_ANS_NOTCH_RATIO": "interop: lets ANSam through band noise on a poor line (a caller otherwise sends CI all call)",
    "ME_SOUNDER_RMS": "measurement: level of the channel sounder (ME_SOUNDER)",
    "V32BIS_TRN_SYMBOLS": "test hook: a far end that sends long TRN (slmodemd runs to ~8000)",
}

READERS = r"getenv|parse_env_int|V34_DIAG_GETENV|env_or|v34_diag_flag|parse_v8_answer_tone_env"
CALL = re.compile(r"\b(?:%s)\(\s*\"([A-Z][A-Z0-9]*_[A-Z0-9_]+)\"\s*(?:,\s*([^)]{0,40}))?" % READERS)
COMMENT = re.compile(r"(?:/\*|//|^\*)\s*(.+?)\s*(?:\*/)?$")


def live_sources():
    out = subprocess.run(["make", "-pn"], capture_output=True, text=True).stdout
    for line in out.splitlines():
        if re.match(r"^SRCS :?= ", line):
            return set(w for w in line.split() if w.endswith(".c"))
    return set()


def tracked(paths):
    """Only files in git: the doc describes the repository, not a working tree."""
    out = subprocess.run(["git", "ls-files"], capture_output=True, text=True).stdout
    known = set(out.split())
    return [p for p in paths if p in known]


def read_all(paths):
    texts = []
    for p in paths:
        try:
            with open(p, errors="replace") as f:
                texts.append(f.read())
        except OSError:
            pass
    return texts


def classify(name):
    if re.search(r"DUMP|_LOG|LOG_|TRACE|DEBUG|_TAP|TAP$|VERBOSE|CAPTURE|STATS|PRINT|_DIAG|DIAG_", name):
        return "diagnostic"
    if re.search(r"AFTER_MS|FORCE|INJECT|DISRUPT|_HOLD|HOLD$|TEST|_PROBE$|SIMULAT", name):
        return "test hook"
    return "switch"


def main():
    live = live_sources() | set(os.path.basename(p) for p in glob.glob("spandsp-master/src/*.c"))
    files = tracked(glob.glob("*.c") + glob.glob("spandsp-master/src/*.c") + glob.glob("tools/*.c")
                    + glob.glob("rig/**/*.c", recursive=True))
    info = {}
    for path in files:
        try:
            with open(path, errors="replace") as f:
                lines = f.read().split("\n")
        except OSError:
            continue
        for i, line in enumerate(lines):
            for m in CALL.finditer(line):
                d = info.setdefault(m.group(1), {"files": set(), "default": None, "ctx": ""})
                d["files"].add(os.path.basename(path))
                if m.group(2) and d["default"] is None:
                    d["default"] = m.group(2).strip()
                if not d["ctx"]:
                    for j in range(i, max(-1, i - 14), -1):
                        c = COMMENT.search(lines[j].strip())
                        if c and len(c.group(1)) > 12 and not c.group(1).startswith("-"):
                            d["ctx"] = c.group(1)[:100].replace("|", "/")
                            break

    tests = read_all(["makefile"] + glob.glob("tests/*.list") + glob.glob("*_test.c"))
    scripts = read_all([p for p in glob.glob("tools/**/*", recursive=True) + glob.glob("rig/**/*", recursive=True)
                        if os.path.isfile(p) and p.endswith((".sh", ".py"))
                        and not p.endswith("env_knobs_inventory.py")])
    docs = read_all([p for p in glob.glob("docs/*.md")
                     if not p.endswith(("project_history.md", "env_knobs.md"))] + ["readme.md"])
    history = read_all(["docs/project_history.md"])
    # A program that sets a variable for code it calls (vpcm_decode sweeping
    # VPCM_V90_PP_PHASE) is a user of it.
    setters = set()
    for text in read_all(files):
        setters.update(re.findall(r'\bsetenv\(\s*"([A-Z][A-Z0-9]*_[A-Z0-9_]+)"', text))

    def used(name, texts):
        rx = re.compile(r"\b%s\b" % re.escape(name))
        return any(rx.search(t) for t in texts)

    settings = dict(SETTINGS)
    rows = []
    for name, d in sorted(info.items()):
        rows.append({
            "name": name,
            "files": ", ".join(sorted(d["files"])),
            "live": any(f in live for f in d["files"]),
            "cat": "setting" if name in settings else "kept" if name in KEPT else classify(name),
            "default": d["default"] or "",
            "ctx": settings.get(name, KEPT.get(name, d["ctx"])),
            "refs": ("c" if name in setters else "")
                    + "".join(c for c, t in (("t", tests), ("s", scripts), ("d", docs), ("h", history))
                              if used(name, t)),
        })

    live_rows = [r for r in rows if r["live"]]
    off_rows = [r for r in rows if not r["live"]]
    w = sys.stdout.write
    w("# Environment variables\n\n")
    w("Generated by `tools/env_knobs_inventory.py`; regenerate it rather than editing the\n"
      "tables. %d names are read: %d on the live call path (`SRCS` and SpanDSP) and %d only\n"
      "by offline tools and tests.\n\n" % (len(rows), len(live_rows), len(off_rows)))
    w("The supported interface is the AT command set on the PTY (`AT+MS`, `AT+ES`, `AT+DS`, ...).\n"
      "Environment variables are, in decreasing order of legitimacy:\n\n"
      "- **settings**: the short list below;\n"
      "- **diagnostics**: dumps, traces and logs, safe to leave in;\n"
      "- **test hooks**: provoke an event (`*_AFTER_MS`, `*_FORCE*`) for a harness;\n"
      "- **switches**: everything else. Nearly all of these either restore the behaviour\n"
      "  before a measured fix (`=0 disables`) or enable an experiment that was measured\n"
      "  and left off. They are the removal backlog: hard-code the default that the\n"
      "  measurement chose and delete the other branch.\n\n"
      "`refs` says where a name is used outside the code that reads it: `c` code that\n"
      "sets it for a callee, `t` a test\n"
      "or the makefile, `s` a script, `d` a current doc, `h` `docs/project_history.md`\n"
      "(where the measurement behind most switches is recorded). A switch with no `c`,\n"
      "`t`, `s` or `d` is the first thing to remove.\n\n")

    w("## Settings\n\n| name | read in | meaning |\n|---|---|---|\n")
    for name, meaning in SETTINGS:
        r = next((r for r in rows if r["name"] == name), None)
        w("| `%s` | %s | %s |\n" % (name, r["files"] if r else "**not found**", meaning))

    for title, cat in (("Live-path switches", "switch"), ("Kept on purpose", "kept"),
                       ("Live-path diagnostics", "diagnostic"), ("Live-path test hooks", "test hook")):
        sel = [r for r in live_rows if r["cat"] == cat]
        unused = sum(1 for r in sel if not set(r["refs"]) & set("ctsd"))
        w("\n## %s (%d, %d with no test/script/doc reference)\n\n" % (title, len(sel), unused))
        w("| name | read in | default | refs | nearest comment |\n|---|---|---|---|---|\n")
        for r in sel:
            w("| `%s` | %s | %s | %s | %s |\n" % (r["name"], r["files"], r["default"].replace("|", "/"),
                                                  r["refs"] or "-", r["ctx"]))

    removed = []
    if os.path.exists("tools/env_knobs_removed.tsv"):
        with open("tools/env_knobs_removed.tsv") as f:
            removed = [l.rstrip("\n").split("\t") for l in f if l.strip() and not l.startswith("#")]
    w("\n## Removed (%d)\n\n" % len(removed))
    w("Switches that used to exist and are gone; the behaviour is fixed at what was\n"
      "their default.  Older docs and the project history still mention them.\n"
      "Kept in `tools/env_knobs_removed.tsv`.\n\n| name | removed by |\n|---|---|\n")
    for name, commit in removed:
        w("| `%s` | %s |\n" % (name, commit))

    w("\n## Offline tools and tests (%d)\n\n" % len(off_rows))
    w("Options of analysis tools and harnesses, not of the modem. These belong on\n"
      "the tools' command lines; listed here so nothing reads an environment silently.\n\n")
    w("| name | read in | refs |\n|---|---|---|\n")
    for r in off_rows:
        w("| `%s` | %s | %s |\n" % (r["name"], r["files"], r["refs"] or "-"))


if __name__ == "__main__":
    main()
