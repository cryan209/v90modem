#!/usr/bin/env python3
"""Voice-mode PCM fidelity harness.

Uses a modem's AT+FCLASS=8 voice mode (AT+VSM=130/131 etc.) to push a known
PCM test signal through the real analog line + FXS gateway + asterisk + SIP
path, bypassing V.8/V.90 training entirely. This isolates "does the RTP/G.711
path preserve PCM sample fidelity" from the V.90 handshake issues tracked
elsewhere in this repo.

Two directions:
  modem-tx  modem plays the reference signal via AT+VTX; we capture it on
            our SIP side (sip_v90_modem's live-rx.g711 tap) and compare.
  our-tx    sip_v90_modem transmits the reference signal via
            ME_VOICE_TEST_TX_FILE while dialing out; the modem captures it
            via AT+VRX and we compare what it recorded.

Requires ME_VOICE_CAPTURE_HOLD=1 support in modem_engine.c (keeps the SIP
call open past V.8 timeout instead of hanging up, since a peer in voice mode
never sends V.8 CM) and ME_VOICE_TEST_TX_FILE support (replaces our own
V.8/ANSam TX with raw G.711 octets read from a file) -- both added
alongside this tool.
"""
from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from g711_codec import alaw_to_linear  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
BINARY = ROOT / "sip_v90_modem"
USR_VOICE_AT = ROOT / "tools" / "usr_voice_at.py"
PYTHON = ROOT / ".venv" / "bin" / "python"


def compare_alaw(reference_path: Path, captured_path: Path) -> dict:
    ref = np.frombuffer(reference_path.read_bytes(), dtype=np.uint8)
    cap = np.frombuffer(captured_path.read_bytes(), dtype=np.uint8)
    if len(ref) == 0 or len(cap) == 0:
        return {"error": f"empty input: ref={len(ref)} octets, cap={len(cap)} octets"}

    ref_lin = alaw_to_linear(ref).astype(np.float64)
    cap_lin = alaw_to_linear(cap).astype(np.float64)

    # Cross-correlate to find the best time alignment (capture is delayed
    # and may be truncated relative to the reference).
    corr = np.correlate(cap_lin, ref_lin, mode="full")
    lag = int(np.argmax(np.abs(corr)) - (len(ref_lin) - 1))

    if lag >= 0:
        ref_aligned = ref_lin[: len(cap_lin) - lag]
        cap_aligned = cap_lin[lag: lag + len(ref_aligned)]
        ref_codes = ref[: len(cap) - lag]
        cap_codes = cap[lag: lag + len(ref_codes)]
    else:
        cap_aligned = cap_lin[: len(ref_lin) + lag]
        ref_aligned = ref_lin[-lag: -lag + len(cap_aligned)]
        cap_codes = cap[: len(ref) + lag]
        ref_codes = ref[-lag: -lag + len(cap_codes)]

    n = min(len(ref_aligned), len(cap_aligned))
    ref_aligned, cap_aligned = ref_aligned[:n], cap_aligned[:n]
    ref_codes, cap_codes = ref_codes[:n], cap_codes[:n]

    if n == 0:
        return {"error": "no overlap after alignment", "lag_samples": lag}

    byte_exact_rate = float(np.mean(ref_codes == cap_codes)) if n else 0.0

    denom = float(np.dot(ref_aligned, ref_aligned))
    scale = float(np.dot(ref_aligned, cap_aligned) / denom) if denom > 0 else 0.0
    noise = cap_aligned - scale * ref_aligned
    signal_power = float(np.mean((scale * ref_aligned) ** 2))
    noise_power = float(np.mean(noise ** 2))
    snr_db = 10 * np.log10(signal_power / noise_power) if noise_power > 0 else float("inf")

    return {
        "reference_octets": int(len(ref)),
        "captured_octets": int(len(cap)),
        "aligned_octets": int(n),
        "lag_samples": lag,
        "gain_scale": scale,
        "byte_exact_match_rate": byte_exact_rate,
        "snr_db": snr_db,
    }


def start_modem(env_extra: dict, log_path: Path) -> subprocess.Popen:
    env = os.environ.copy()
    env.update(env_extra)
    log_f = open(log_path, "wb")
    proc = subprocess.Popen(
        [str(BINARY),
         "--sip-server", "asterisk.net.cryan.nz",
         "--username", "6001",
         "--password", "6001",
         "--pty", "/tmp/v90modem"],
        cwd=ROOT, env=env, stdout=log_f, stderr=subprocess.STDOUT,
    )
    proc._log_f = log_f  # keep reference alive
    return proc


def stop_modem(proc: subprocess.Popen, timeout: float = 10.0) -> None:
    if proc.poll() is not None:
        return
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5)


def pty_dial_and_hold(number: str, hold_seconds: float, pty_path: str = "/tmp/v90modem") -> None:
    import termios
    import tty

    deadline_open = time.time() + 15
    fd = None
    while time.time() < deadline_open:
        try:
            fd = os.open(pty_path, os.O_RDWR | os.O_NOCTTY)
            break
        except OSError:
            time.sleep(0.5)
    if fd is None:
        raise RuntimeError(f"could not open {pty_path}")
    tty.setraw(fd, termios.TCSANOW)
    os.set_blocking(fd, False)

    def send(s: str) -> None:
        os.write(fd, s.encode())

    def drain(duration: float) -> None:
        deadline = time.time() + duration
        while time.time() < deadline:
            try:
                os.read(fd, 4096)
            except BlockingIOError:
                time.sleep(0.05)

    send("AT\r")
    time.sleep(1)
    drain(1.0)
    send(f"ATD{number}\r")
    print(f"[voice_pcm_fidelity] PTY dialing {number}, holding {hold_seconds:.0f}s", flush=True)
    drain(hold_seconds)
    send("+++")
    time.sleep(1.5)
    send("ATH\r")
    time.sleep(1)
    drain(2.0)
    os.close(fd)


def run_modem_tx(opts) -> int:
    """Modem's AT+VTX -> our live-rx.g711 capture."""
    out_dir = opts.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    env = {
        "VPCM_G711_TAP_DIR": str(out_dir),
        "SIP_FORCE_PCMA": "1",
        "ME_VOICE_CAPTURE_HOLD": "1",
    }
    print(f"[voice_pcm_fidelity] starting sip_v90_modem, taps -> {out_dir}", flush=True)
    proc = start_modem(env, out_dir / "modem.log")
    try:
        time.sleep(4.0)  # let it register before the modem dials in
        at_proc = subprocess.run(
            [str(PYTHON), str(USR_VOICE_AT), "--dev", opts.dev, "--vsm", str(opts.vsm),
             "dial-vtx", opts.dial_number, str(opts.reference), "--settle", str(opts.dial_wait)],
            cwd=ROOT,
        )
        time.sleep(2.0)  # let trailing capture flush
    finally:
        stop_modem(proc)

    if at_proc.returncode != 0:
        print("[voice_pcm_fidelity] usr_voice_at dial-vtx failed; see above", flush=True)
        return 1

    captured = out_dir / "live-rx.g711"
    result = compare_alaw(opts.reference, captured)
    print(f"[voice_pcm_fidelity] modem-tx result: {result}", flush=True)
    (out_dir / "result.json").write_text(_json(result))
    return 0


def run_our_tx(opts) -> int:
    """Our ME_VOICE_TEST_TX_FILE -> modem's AT+VRX capture."""
    out_dir = opts.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    captured = out_dir / "modem_vrx_capture.alaw"
    env = {
        "VPCM_G711_TAP_DIR": str(out_dir),
        "SIP_FORCE_PCMA": "1",
        "ME_VOICE_CAPTURE_HOLD": "1",
        "ME_VOICE_TEST_TX_FILE": str(opts.reference),
    }
    # This modem gives no answer-supervision text for voice-mode auto-answer
    # either, so instead of waiting to detect "connected" we arm AT+VRX
    # almost immediately (a few seconds of leading silence is harmless --
    # cross-correlation alignment in compare_alaw() trims it) and just
    # record generously long enough to span setup + the whole held call.
    capture_duration = opts.dial_wait + 20.0

    at_thread_result = {}

    def run_vrx():
        r = subprocess.run(
            [str(PYTHON), str(USR_VOICE_AT), "--dev", opts.dev, "--vsm", str(opts.vsm),
             "answer-vrx", str(captured), "--duration", str(capture_duration), "--wait", "3", "--rings", "1"],
            cwd=ROOT,
        )
        at_thread_result["rc"] = r.returncode

    print(f"[voice_pcm_fidelity] starting sip_v90_modem, taps -> {out_dir}", flush=True)
    proc = start_modem(env, out_dir / "modem.log")
    try:
        time.sleep(4.0)
        vrx_thread = threading.Thread(target=run_vrx, daemon=True)
        vrx_thread.start()
        time.sleep(2.0)  # let the modem arm ATS0/answer before we dial
        pty_dial_and_hold(opts.dial_number, opts.dial_wait + 5)
        vrx_thread.join(timeout=capture_duration + 30)
    finally:
        stop_modem(proc)

    if at_thread_result.get("rc") != 0:
        print("[voice_pcm_fidelity] usr_voice_at answer-vrx failed; see above", flush=True)
        return 1

    result = compare_alaw(opts.reference, captured)
    print(f"[voice_pcm_fidelity] our-tx result: {result}", flush=True)
    (out_dir / "result.json").write_text(_json(result))
    return 0


def _json(d: dict) -> str:
    import json
    return json.dumps(d, indent=2, sort_keys=True) + "\n"


def cmd_compare(opts) -> int:
    result = compare_alaw(opts.reference, opts.captured)
    print(_json(result))
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="mode", required=True)

    common = dict(dev="/dev/cu.usbserial-21240", vsm=130)

    p = sub.add_parser("modem-tx", help="modem AT+VTX -> our RTP capture")
    p.add_argument("--reference", type=Path, default=ROOT / "artifacts/voice-pcm-fidelity/reference.alaw")
    p.add_argument("--out-dir", type=Path, default=ROOT / "artifacts/voice-pcm-fidelity/modem-tx")
    p.add_argument("--dev", default=common["dev"])
    p.add_argument("--vsm", type=int, default=common["vsm"])
    p.add_argument("--dial-number", default="6001", help="extension the modem dials to reach us")
    p.add_argument("--dial-wait", type=float, default=30.0)
    p.set_defaults(func=run_modem_tx)

    p = sub.add_parser("our-tx", help="our TX file -> modem AT+VRX capture")
    p.add_argument("--reference", type=Path, default=ROOT / "artifacts/voice-pcm-fidelity/reference.alaw")
    p.add_argument("--out-dir", type=Path, default=ROOT / "artifacts/voice-pcm-fidelity/our-tx")
    p.add_argument("--dev", default=common["dev"])
    p.add_argument("--vsm", type=int, default=common["vsm"])
    p.add_argument("--dial-number", default="6312", help="extension we dial to reach the modem")
    p.add_argument("--dial-wait", type=float, default=16.0, help="seconds to hold the call for the VRX capture")
    p.set_defaults(func=run_our_tx)

    p = sub.add_parser("compare", help="compare two raw A-law files (offline)")
    p.add_argument("reference", type=Path)
    p.add_argument("captured", type=Path)
    p.set_defaults(func=cmd_compare)

    opts = ap.parse_args()
    return opts.func(opts)


if __name__ == "__main__":
    raise SystemExit(main())
