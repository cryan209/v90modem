#!/usr/bin/env python3
"""Regenerate tools/README.md: one line for every tool in tools/ and scripts/.

Usage: tools/tools_inventory.py > tools/README.md   (from the repo root)

Each line is the first sentence of the tool's own docstring or top comment,
so fix a bad line in the tool, or in OVERRIDES below when the tool has none.
Tools are placed by the first SECTIONS pattern their name matches; FAMILIES
collapses a run of near-identical tools into one row.  Only files git tracks
are listed.
"""

import ast
import re
import subprocess
import sys

START_HERE = [
    ("run_tests.py", "Runs `tests/*.list` in parallel (`make test-fast`); every failure keeps its log in `test-logs/`."),
    ("v90_hardware_interop.py", "One bounded live call against an analogue modem, with log, raw taps and a parsed timeline."),
    ("summarize_modem_trace.py", "Turns a verbose `sip_v90_modem` log into a short call timeline."),
    ("constellation.py", "Constellation and eye diagrams for a V.34/V.90 call, as an HTML page."),
    ("pty_dial.py", "Dials a number through the modem's PTY and logs the result."),
    ("rtp_path_check.py", "Checks from a pcap that both legs of an RTP relay carry G.711 untouched."),
    ("measure_timing_slips.py", "Counts whole-sample timing slips in a recorded G.711 tap."),
    ("v42_stats_summary.py", "Summarises `V42_STATS` output (frames, bad FCS, REJ, T401) from a server log."),
    ("mnp_frames.py", "Lists the MNP frames in a bit dump."),
    ("env_knobs_inventory.py", "Regenerates `docs/env_knobs.md`."),
    ("tools_inventory.py", "Regenerates this file."),
]

# Tools with no usable header, or whose header's first sentence misleads.
OVERRIDES = {
    "softmodem_debug.sh": "Build and launch the modem for softmodem debugging, with preflight checks.",
    "v34_tone_matrix.sh": "Score tone recordings for INFO0a, INFO1a and Phase 3 (`make v34-tone-matrix`).",
    "auto_ja_search.py": "Search for Ja DBPSK candidates.",
    "check_ja_dil_consistency.py": "Check a Ja descriptor against the DIL window seen.",
    "decode_ja_bits.py": "Decode Ja/TRN bits around an anchor.",
    "detect_trn1u_ja.py": "Find TRN1u and Ja in a capture.",
    "explore_v90_ja_demod.py": "Measure Ja demodulation quality from anchored I/Q symbols.",
    "plot_iq_constellation.py": "Draw an I/Q dump as an SVG constellation.",
    "recover_ja_descriptor.py": "Recover a Ja DIL descriptor by block-period consensus.",
    "refine_ja_lock.py": "Refine Ja lock using framing/CRC metrics.",
    "smartlink_jd_probe.c": "Probe slmodemd's Jd handling.",
    "test_summarize_modem_trace.py": "Unit tests for `summarize_modem_trace.py`.",
    "validate_ja_sequence.py": "Check a Ja bitstream's structure.",
    "visualize_dil_waveform.py": "Show a Ja descriptor and its DIL waveform in the terminal.",
    "vpcm_cp_bits_probe.c": "Decode CP frames from an ASCII bit file.",
    "reverse_test.py": "Set the Conexant modem to auto-answer, dial it from our PTY, and log both sides.",
    "usr_voice_at.py": "Voice-mode (V.253) AT driver for the USR 56K external modem.",
    "sip_clearmode_test.py": "Two of our instances over local SIP must negotiate RFC 4040 CLEARMODE when asked.",
    "wav_to_g711.c": "Split a stereo 8 kHz WAV and encode each channel as raw G.711.",
    "gen_pcm_test_signal.py": "Generate the reference signal for `voice_pcm_fidelity.py`.",
    "apple_modem_line_check.sh": "Health check of an Apple USB Modem's line: off-hook, tone into dial tone, measure the echo.",
    "apple_usb_modem_probe.c": "USB-side probe for the Apple USB Modem (SM56).",
    "hsf_reg_bit_sweep.sh": "Toggle the HSF streaming registers one bit at a time, watching for transmit to drain.",
    "hsf_tx_gate_sweep.sh": "Sweep the HSF driver's path codes for the one that lets bulk OUT drain.",
    "hsf_tx_reaches_line.sh": "Check whether our DTMF reaches the line through the HSF modem.",
    "desplice_matrix.sh": "Replay every recorded call as taped and with its splices undone; report clean time and longest hold.",
    "search_v90_ja.py": "Search a capture for V.90 8.3.1 Ja candidates.",
    "v90_cp_frame.py": "Parse V.90 CP frames (Table 14) out of a bit string.",
    "v92_p3_probe.c": "Replay a recorded receive tap through the strict V.92 Phase 3 receiver.",
    "v92_p3_rx_smoketest.c": "Smoke test of the V.92 Phase 3 receiver on a raw G.711 stream.",
    "v8_loop_test.c": "Closed-loop V.8, our caller against our answerer, with delay and near-end echo.",
    "concede_ab_summary.py": "Score a `tools/soak/v90_concede_ab.sh` run.",
    "trn1d_pool.py": "Pool TRN1d first-attempt results across runs, by the TRN1d length the log reports.",
    "rtp_jitter_scan.py": "Scan an RTP trace for loss, timestamp jumps and arrival jitter.",
}

# (glob-ish regex, row name, description): one row for the whole family.
FAMILIES = [
    (r"^mica_qam_.*_oracle\.py$", "mica_qam_*_oracle.py",
     "Oracles that run one stage of the MICA QAM firmware in the emulator and compare it with `mica_qam.c` (`docs/mica_qam_firmware.md`)."),
    (r"^auto_table1[5-7]_search\.py$", "auto_table15/16/17_search.py",
     "Search captures for the V.90 Table 15-17 parameter frames."),
    (r"^generate_k56flex_.*\.py$", "generate_k56flex_*.py",
     "Generate K56flex regression vectors and fixtures from the original firmware."),
    (r"^generate_x2_.*\.py$", "generate_x2_*.py",
     "Generate x2 regression vectors from original I-modem/Courier execution."),
    (r"^vfc_.*_vectors\.py$", "vfc_*_vectors.py",
     "Generate V.FC firmware, frame and signature vectors."),
    (r"^smartlink_.*_(wrap|probe)\.c$", "smartlink_*.c",
     "Wrappers that call slmodemd's own routines (scrambler, shaper, mapping, CP symbols, TRN2d) for comparison."),
    (r"^v32bis_tx_.*\.py$", "v32bis_tx_*.py",
     "V.32bis reference transmitter stages: symbols, baseband, passband."),
]

SECTIONS = [
    ("Running and checking calls", None, r"^(pty_dial|summarize_modem_trace|replay_dte_check|v42_stats|mnp_frames|hdlc_bit_check|eye_summary|constellation|modem_showcase|multi_line|sip_loop|sip_clearmode|v91_serial_pair|softmodem_debug)"),
    ("Hardware peers", "Eicon Diva card", r"^eicon"),
    ("Hardware peers", "Apple USB Modem (SM56)", r"^apple"),
    ("Hardware peers", "Conexant HSF USB modem", r"^(hsf|d8051|reverse_test)"),
    ("Hardware peers", "Other modems", r"^(cx_|test_cx|mt5634mu|usr_|voice_pcm|gen_pcm_test|dmodem|slm_|peer_phase4|analyze_dmodem|k56flex_hsf)"),
    ("SIP, RTP and the G.711 bearer", None, r"^(g711|rtp|test_rtp|desplice|measure_timing|inject_sample|echo_lag|slip_ab|wav_to_g711)"),
    ("V.90 and V.92", "Ja, DIL and Phase 3", r"(ja|dil|trn1|trn2|concede|auto_table|info_state|validate_ja|v92_trn1u|v92_p3|v92_fit)"),
    ("V.90 and V.92", "Other V.90/V.92 and V.PCM", r"^(v90|v92|vpcm|test_vpcm|test_v90|pcm_|noci|rf_|v91)"),
    ("V.34, V.8 and V.56", None, r"^(v34|cc_demod|check_info|dpsk|plot_iq|v8_|v56|fax_v8|v34_tone)"),
    ("V.32bis", None, r"^(v32bis|test_v32bis)"),
    ("K56flex, x2, V.FC and firmware", None, r"^(k56flex|test_k56flex|generate_k56|x2|test_x2|analyze_x2|probe_x2|generate_x2|vfc|probe_vfc|build_vfc|mica|mips_dis|pe_info|smartlink)"),
    ("V.42bis and V.44", None, r"^(v42bis|v44)"),
    ("GUI and browser demo", None, r"^(gui|modem_gui|build_modem_wasm|package_modem_demo)"),
    ("Test runner and repository upkeep", None, r"."),
]

SUBDIRS = [
    ("asterisk/", "Asterisk RFC 4040 CLEARMODE codec module (own README)."),
    ("eicon/", "Test answerer, stream test and HTTP server that run on the Eicon host (`docs/eicon_soak_ppp.md`)."),
    ("h245/", "H.245 ASN.1 extraction and PER generator for V.70 (`make h245-schema`)."),
    ("k56flex_upstream_rig/", "Offline K56flex upstream equalizer and ISI analysis."),
    ("linux-capture/", "Running the vendor HSF driver under Linux for a usbmon capture (own README)."),
    ("mica_overlay_probe/", "Driving the emulated Cisco MICA overlays (own README)."),
    ("soak/", "Live-call and soak scripts for the tower/vmhost rigs (`rig/README.md`)."),
    ("v32bis_datapump/, v32bis_ref/", "Python V.32bis reference model and its tests."),
    ("v56bis/", "V.56bis reference filter data."),
    ("dumps/", "Small bit dumps the Ja tools read."),
]

MAKE_TARGETS = [
    ("test-build", "Build the modem and every test binary; run nothing."),
    ("g711-path-test", "Live: does the SIP path carry G.711 byte-exact, and does a law-pinned endpoint refuse the other law? Needs a PBX with an Answer()+Echo() extension (`G711_TEST_ARGS`)."),
    ("sip-loop-test", "Two instances over local SIP: ringing, caller ID, S0/ATA, an unanswered dial. Uses UDP 5070/5080, RTP 41000/42000."),
    ("sip-clearmode-test", "Two instances negotiate RFC 4040 CLEARMODE."),
    ("v32bis-ref-test, v32bis-datapump-test", "Python V.32bis reference and datapump tests."),
    ("v34-hdx-high-rate-test", "V.34 half-duplex at 3200/3429 baud, 26400-33600, both laws; lists every failure."),
    ("gui-spectrum-test, gui-fft-test, gui-audio-test", "Offline checks of the native GUI (macOS)."),
    ("h245-schema, h245-golden", "Regenerate the H.245 code and golden vectors (`pdftotext`, `asn1tools`)."),
]


def first_sentence(text):
    text = " ".join(text.split())
    m = re.match(r"(.+?[.!?])(\s|$)", text)
    s = m.group(1) if m else text
    if len(s) > 130:
        for sep in ("; ", " -- ", ": ", ", "):
            i = s.find(sep, 40, 130)
            if i > 0:
                s = s[:i] + "."
                break
        else:
            s = s[:127].rsplit(" ", 1)[0] + "..."
    return s.replace("|", "\\|")


def header(path):
    src = open(path, errors="replace").read()
    if path.endswith(".py"):
        try:
            doc = ast.get_docstring(ast.parse(src))
            if doc:
                return doc.split("\n\n")[0]
        except SyntaxError:
            pass
    if path.endswith(".c"):
        m = re.search(r"/\*(.*?)\*/", src, re.S)
        if m:
            text = m.group(1).strip().split("\n\n")[0]
            text = " ".join(l.strip(" *") for l in text.splitlines())
            return re.sub(r"^\S+\.c\s*[-—]+\s*", "", text)
        return ""
    body = re.sub(r"^#!.*\n", "", src)
    m = re.match(r"((?:#.*\n)+)", body)
    return " ".join(l.lstrip("# ") for l in m.group(1).splitlines()) if m else ""


def main():
    files = subprocess.run(["git", "ls-files", "tools", "scripts"],
                           capture_output=True, text=True, check=True).stdout.split()
    files = [f for f in files if f.count("/") == 1 and re.search(r"\.(py|c|sh)$", f)]
    rows = {}
    family_done = set()
    for path in sorted(files):
        name = path.split("/", 1)[1]
        fam = next((f for f in FAMILIES if re.search(f[0], name)), None)
        if fam:
            if fam[1] in family_done:
                continue
            family_done.add(fam[1])
            cell, desc = f"`{fam[1]}`", fam[2]
            match_name = name
        else:
            desc = OVERRIDES.get(name) or first_sentence(header(path)) or "(no description)"
            link = path.split("/", 1)[1] if path.startswith("tools/") else "../" + path
            cell = f"[`{name}`]({link})"
            match_name = name
        sec = next(s for s in SECTIONS if re.search(s[2], match_name))
        rows.setdefault((sec[0], sec[1]), []).append((cell, desc))

    out = sys.stdout.write
    out("# Tools\n\n")
    out("Every script and helper program in `tools/` and `scripts/`, one line each,\n"
        "taken from the tool's own header. Generated by `tools/tools_inventory.py`;\n"
        "regenerate it rather than editing it. Run a tool with `--help` for its\n"
        "options. Python tools that need numpy or scipy run from the repo virtualenv\n"
        "(`./.venv/bin/python`, see `requirements-tools.txt`).\n\n"
        "Most of these were written for one investigation. The topic note in `docs/`\n"
        "([index](../docs/README.md)) says what each one showed.\n\n")
    out("## Start here\n\n| Tool | What it does |\n|---|---|\n")
    for name, desc in START_HERE:
        out(f"| [`{name}`]({name}) | {desc} |\n")
    out("\n## Contents\n\n")
    seen = []
    for sec in SECTIONS:
        if sec[0] not in seen and any(k[0] == sec[0] for k in rows):
            seen.append(sec[0])
            anchor = re.sub(r"[^a-z0-9 -]", "", sec[0].lower()).replace(" ", "-")
            out(f"- [{sec[0]}](#{anchor})\n")
    out("- [Subdirectories](#subdirectories)\n- [Make targets outside `make test`](#make-targets-outside-make-test)\n")
    last = None
    for sec in SECTIONS:
        key = (sec[0], sec[1])
        if key not in rows:
            continue
        if sec[0] != last:
            out(f"\n## {sec[0]}\n")
            last = sec[0]
        if sec[1]:
            out(f"\n### {sec[1]}\n")
        out("\n| Tool | What it does |\n|---|---|\n")
        for cell, desc in rows[key]:
            out(f"| {cell} | {desc} |\n")
    out("\n## Subdirectories\n\n| Directory | Contents |\n|---|---|\n")
    for d, desc in SUBDIRS:
        out(f"| `{d}` | {desc} |\n")
    out("\n## Make targets outside `make test`\n\n"
        "`make test-fast`, `make test-slow` and `make test` run `tests/*.list`. The\n"
        "readme and topic notes cover the matrix targets (`pcm-matrix`, `v56-sweep`,\n"
        "`v34-tone-matrix`, ...). These are the others:\n\n"
        "| Target | What it does |\n|---|---|\n")
    for t, desc in MAKE_TARGETS:
        cell = ", ".join(f"`{x.strip()}`" for x in t.split(","))
        out(f"| {cell} | {desc} |\n")


if __name__ == "__main__":
    main()
