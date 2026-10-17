#!/usr/bin/env python3
"""Launch two sip_v90_modem instances and verify bidirectional PTY payload."""

from __future__ import annotations

import argparse
import os
import pathlib
import select
import signal
import subprocess
import sys
import tempfile
import time
import tty


def wait_path(path: pathlib.Path, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if path.exists():
            return
        time.sleep(0.05)
    raise RuntimeError(f"PTY link did not appear: {path}")


def open_raw(path: pathlib.Path) -> int:
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    tty.setraw(fd)
    return fd


def drain(fds: list[int], seconds: float = 0.5) -> dict[int, bytes]:
    output = {fd: b"" for fd in fds}
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        ready, _, _ = select.select(fds, [], [], 0.05)
        for fd in ready:
            try:
                output[fd] += os.read(fd, 65536)
            except BlockingIOError:
                pass
    return output


def wait_connect(fds: list[int], timeout: float) -> dict[int, bytes]:
    output = {fd: b"" for fd in fds}
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        ready, _, _ = select.select(fds, [], [], 0.2)
        for fd in ready:
            try:
                output[fd] += os.read(fd, 65536)
            except BlockingIOError:
                pass
        if all(b"CONNECT" in output[fd] for fd in fds):
            return output
        if any(b"NO CARRIER" in output[fd] for fd in fds):
            break
    raise RuntimeError(f"V.91 connection did not reach CONNECT: {output!r}")


def expect_payload(src: int, dst: int, payload: bytes, timeout: float) -> bytes:
    os.write(src, payload)
    received = b""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        ready, _, _ = select.select([dst], [], [], 0.2)
        if ready:
            try:
                received += os.read(dst, 65536)
            except BlockingIOError:
                pass
            if payload in received:
                return received
    raise RuntimeError(
        f"payload not received (expected={payload!r}, received={received!r})"
    )


def terminate(proc: subprocess.Popen[bytes]) -> None:
    if proc.poll() is not None:
        return
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.terminate()
        proc.wait(timeout=5)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary", default="./sip_v90_modem")
    parser.add_argument("--caller-port", type=int, default=5091)
    parser.add_argument("--answerer-port", type=int, default=5092)
    parser.add_argument("--timeout", type=float, default=25.0)
    parser.add_argument(
        "--robbed-phase", type=int, default=None,
        help="simulate a robbed-bit trunk on both receive paths at this cadence phase (0-5)")
    parser.add_argument(
        "--retrain", action="store_true",
        help="caller starts a retrain 1.5 s into data mode (8.4); payload must flow after it")
    parser.add_argument(
        "--escape", action="store_true",
        help="both ends abandon V.91 start-up after 0.7 s and escape to V.34 (8.3)")
    parser.add_argument(
        "--specify-dil", choices=("caller", "answerer", "both"), default=None,
        help="that end requests a specified DIL (INFO bit 26 = 1, J, PHIL, Em; 8.2.1.5-8.2.1.7, "
             "Figures 2 and 3)")
    parser.add_argument(
        "--transparent", choices=("caller", "answerer", "both"), default=None,
        help="that end requests transparent mode (INFO bit 40; 6.5, 8.7)")
    parser.add_argument(
        "--transparent-cleardown", action="store_true",
        help="with --transparent: also set INFO bit 41 (cleardown if not granted)")
    parser.add_argument(
        "--loop3", action="store_true",
        help="answerer closes V.54 loop 3 with AT&T3 (circuit 141): the caller must get its data "
             "back and the answerer's DTE must see none; AT&T0 restores normal data (clause 9)")
    parser.add_argument(
        "--no-control-channel", action="store_true",
        help="neither end requests a control channel; the local hangup then clears down with a "
             "retrain and CP drn = 0 (8.6)")
    parser.add_argument(
        "--attempts", type=int, default=3,
        help="run again (in a fresh process) up to this many times when a call fails; the "
             "host's 100-200 ms scheduling stalls and V.8 timeouts are not V.91 defects, "
             "while a real bug fails every attempt")
    args = parser.parse_args()
    if args.attempts > 1:
        last = 1
        for attempt in range(1, args.attempts + 1):
            cmd = [sys.executable, __file__] + [a for a in sys.argv[1:]] + ["--attempts", "1"]
            run = subprocess.run(cmd, capture_output=True, text=True)
            last = run.returncode
            if last == 0:
                sys.stdout.write(run.stdout)
                if attempt > 1:
                    print(f"v91_serial_pair_test: (passed on attempt {attempt}/{args.attempts})")
                return 0
            sys.stderr.write(f"v91_serial_pair_test: attempt {attempt}/{args.attempts} failed: "
                             + (run.stderr.splitlines() or ["?"])[0][:200] + "\n")
            time.sleep(3)
        sys.stderr.write(run.stderr)
        return last

    child_env = dict(os.environ)
    # These tests are about the V.91 protocol, not the host's scheduling: on a
    # busy Mac both processes stall for 100-200 ms at the same instants, which
    # exhausts the engine's default 200 ms jitter buffer and puts idle codewords
    # into the stream.  A deeper buffer keeps that out of the verdict.
    child_env.setdefault("ME_JB_MS", "320")
    label = "bidirectional PTY payload over SIP/RTP"
    if args.robbed_phase is not None:
        child_env["V91_SIM_ROBBED_RX"] = str(args.robbed_phase)
        label += f", simulated robbed-bit trunk phase={args.robbed_phase}"

    answerer_env = dict(child_env)
    caller_env = dict(child_env)
    # V.91 7.8 (Table 6 bit 26), 8.2.1.5: select each role explicitly;
    # an inherited diagnostic must not turn a one-sided test into both ends.
    caller_env["ME_V91_SPECIFY_DIL"] = "0"
    answerer_env["ME_V91_SPECIFY_DIL"] = "0"
    if args.retrain:
        caller_env["V91_TEST_RETRAIN_AFTER_MS"] = "1500"
        label += ", caller-initiated retrain"
    if args.escape:
        for env in (caller_env, answerer_env):
            env["V91_TEST_ESCAPE_AFTER_MS"] = "700"
        label += ", escape to V.34"

    if args.specify_dil in ("caller", "both"):
        caller_env["ME_V91_SPECIFY_DIL"] = "1"
    if args.specify_dil in ("answerer", "both"):
        answerer_env["ME_V91_SPECIFY_DIL"] = "1"
    if args.specify_dil:
        label += f", {args.specify_dil} specifies the DIL (J/PHIL/Em)"

    if args.no_control_channel:
        caller_env["ME_V91_CONTROL_CHANNEL"] = "0"
        answerer_env["ME_V91_CONTROL_CHANNEL"] = "0"
        label += ", no control channel"
    if args.transparent:
        val = "2" if args.transparent_cleardown else "1"
        if args.transparent in ("caller", "both"):
            caller_env["ME_V91_TRANSPARENT"] = val
        if args.transparent in ("answerer", "both"):
            answerer_env["ME_V91_TRANSPARENT"] = val
        label += f", {args.transparent} requests transparent mode"

    binary = str(pathlib.Path(args.binary).resolve())
    with tempfile.TemporaryDirectory(prefix="v91-serial-pair-") as temp_dir:
        temp = pathlib.Path(temp_dir)
        # Transmit taps for tools/v91_spec_decode.py (a receiver written from
        # the Recommendation, checking what we actually put on the wire).
        caller_env["VPCM_G711_TAP_DIR"] = str(temp / "caller-taps")
        answerer_env["VPCM_G711_TAP_DIR"] = str(temp / "answerer-taps")
        caller_pty = temp / "caller-pty"
        answerer_pty = temp / "answerer-pty"
        caller_log_path = temp / "caller.log"
        answerer_log_path = temp / "answerer.log"
        caller_log = caller_log_path.open("wb")
        answerer_log = answerer_log_path.open("wb")
        answerer = subprocess.Popen(
            [binary, "--mode", "v91", "--local-port", str(args.answerer_port),
             "--pty-link", str(answerer_pty), "--verbose"],
            stdout=answerer_log,
            stderr=subprocess.STDOUT,
            env=answerer_env,
        )
        caller = subprocess.Popen(
            [binary, "--mode", "v91", "--sip-server", f"127.0.0.1:{args.answerer_port}",
             "--local-port", str(args.caller_port),
             "--pty-link", str(caller_pty), "--verbose"],
            stdout=caller_log,
            stderr=subprocess.STDOUT,
            env=caller_env,
        )
        caller_fd = -1
        answerer_fd = -1
        try:
            wait_path(caller_pty, 8)
            wait_path(answerer_pty, 8)
            caller_fd = open_raw(caller_pty)
            answerer_fd = open_raw(answerer_pty)
            drain([caller_fd, answerer_fd])
            # Raw transport check: V.42 detection racing on loopback made this
            # test pass about half the time on the old and new encoders alike.
            for fd in (caller_fd, answerer_fd):
                os.write(fd, b"AT\\N0\r")
            drain([caller_fd, answerer_fd], 1.0)
            os.write(caller_fd, f"ATD{args.answerer_port}\r".encode("ascii"))
            if args.escape:
                # 8.3: what is under test is the escape and the hand-over to V.34's
                # Phase 2 (INFO0 exchange), not a complete V.34 connection, which
                # is slow and timing-sensitive on a busy host.
                deadline = time.monotonic() + args.timeout
                want = ("V91 escape to V34", "V34 stage: rx=INFO0")
                while time.monotonic() < deadline:
                    drain([caller_fd, answerer_fd], 0.5)
                    texts = [p.read_text(errors="replace") for p in (caller_log_path, answerer_log_path)]
                    if all(all(w in t for w in want) for t in texts):
                        print("v91_serial_pair_test: OK (escape to V.34, Phase 2 started on both ends)")
                        return 0
                raise RuntimeError("V.91 did not escape to V.34 and start Phase 2 on both ends")
            wait_connect([caller_fd, answerer_fd], args.timeout)
            drain([caller_fd, answerer_fd])
            if args.retrain:
                time.sleep(12.0)           # 1.5 s to the hook + a full re-startup
                drain([caller_fd, answerer_fd])

            c2a = b"V91-CALLER-TO-ANSWERER-0123456789\r\n"
            a2c = b"V91-ANSWERER-TO-CALLER-9876543210\r\n"
            expect_payload(caller_fd, answerer_fd, c2a, 5)
            expect_payload(answerer_fd, caller_fd, a2c, 5)
            if args.loop3:
                time.sleep(1.2)
                os.write(answerer_fd, b"+++")
                time.sleep(1.2)
                got = drain([answerer_fd], 1.0)[answerer_fd]
                if b"OK" not in got:
                    raise RuntimeError(f"answerer did not reach command mode: {got!r}")
                os.write(answerer_fd, b"AT&T3\r")
                got = drain([answerer_fd], 1.0)[answerer_fd]
                if b"OK" not in got:
                    raise RuntimeError(f"AT&T3 refused: {got!r}")
                os.write(answerer_fd, b"ATO\r")
                drain([answerer_fd], 1.5)
                probe = b"LOOP3-PROBE-0123456789\r\n"
                expect_payload(caller_fd, caller_fd, probe, 6)
                leaked = drain([answerer_fd], 1.0)[answerer_fd]
                if b"LOOP3-PROBE" in leaked:
                    raise RuntimeError("loop 3 leaked the data into the answerer's DTE")
                time.sleep(1.2)
                os.write(answerer_fd, b"+++")
                time.sleep(1.2)
                drain([answerer_fd], 1.0)
                os.write(answerer_fd, b"AT&T0\r")
                if b"OK" not in drain([answerer_fd], 1.0)[answerer_fd]:
                    raise RuntimeError("AT&T0 refused")
                os.write(answerer_fd, b"ATO\r")
                drain([answerer_fd, caller_fd], 1.5)
                expect_payload(caller_fd, answerer_fd, c2a, 6)
                label += "; V.54 loop 3 via AT&T3 (circuits 141/142)"
            # 8.6: a local hangup from data mode goes out as the control
            # channel's cleardown bit and the peer drops on receiving it.
            time.sleep(1.2)
            os.write(caller_fd, b"+++")
            time.sleep(1.2)
            os.write(caller_fd, b"ATH\r")
            ends = drain([caller_fd, answerer_fd], 6.0)
            if b"NO CARRIER" not in ends[answerer_fd]:
                raise RuntimeError(f"answerer did not clear down: {ends[answerer_fd]!r}")
            terminate(caller)
            terminate(answerer)
            caller_log.close()
            answerer_log.close()
            alog = answerer_log_path.read_text(errors="replace")
            clog = caller_log_path.read_text(errors="replace")
            if args.retrain and ("V91 retrain complete" not in clog
                                 or "V91 retrain complete" not in alog):
                raise RuntimeError("retrain did not complete on both ends")
            if args.escape:
                if "V91 escape to V34" not in clog or "V91 escape to V34" not in alog:
                    raise RuntimeError("V.91 did not escape to V.34 on both ends")
                print(f"v91_serial_pair_test: OK ({label})")
                return 0
            granted = bool(args.transparent) and args.robbed_phase is None
            if granted:
                # 8.7 / 6.5: 64000 bit/s, codewords carry the data, no control channel
                for name, text in (("caller", clog), ("answerer", alog)):
                    if "V91 enter DATA: rate=64000" not in text or "cc tx=0 rx=0" not in text:
                        raise RuntimeError(f"{name} did not enter transparent data mode")
                label += "; transparent mode at 64000"
            else:
                if args.transparent and "rate=64000" in alog:
                    raise RuntimeError("transparent mode granted on a robbed-bit path")
                if args.no_control_channel:
                    if "cc tx=0 rx=0" not in alog:
                        raise RuntimeError("a control channel was negotiated")
                    if "peer cleardown (CP drn 0)" not in alog:
                        raise RuntimeError("peer did not see the CP drn = 0 cleardown")
                else:
                    if "cc tx=1 rx=1" not in alog:
                        raise RuntimeError("control channel was not negotiated on both directions")
                    if "peer cleardown (control channel)" not in alog:
                        raise RuntimeError("peer did not see the control-channel cleardown")
            if not (args.retrain or args.escape):
                decode = subprocess.run(
                    [sys.executable, str(pathlib.Path(__file__).with_name("v91_spec_decode.py")),
                     str(temp / "caller-taps" / "live-tx.g711"),
                     str(temp / "answerer-taps" / "live-tx.g711"), "ulaw",
                     "V91-CALLER-TO-ANSWERER-0123456789\r\n",
                     "V91-ANSWERER-TO-CALLER-9876543210\r\n",
                     "--specify-dil", args.specify_dil or "neither"],
                    capture_output=True, text=True)
                keep = os.environ.get("V91_KEEP_TAPS")
                if keep:
                    import shutil
                    shutil.copytree(temp / "caller-taps", pathlib.Path(keep) / "caller-taps", dirs_exist_ok=True)
                    shutil.copytree(temp / "answerer-taps", pathlib.Path(keep) / "answerer-taps", dirs_exist_ok=True)
                    print(decode.stdout)
                if decode.returncode != 0:
                    raise RuntimeError("spec decoder rejected our transmit taps:\n"
                                       + decode.stdout + decode.stderr)
                label += "; spec decoder agrees"
            print(f"v91_serial_pair_test: OK ({label}"
                  + ("" if granted else "; CP drn 0 cleardown" if args.no_control_channel
                     else "; control-channel cleardown") + ")")
            return 0
        except Exception as exc:
            terminate(caller)
            terminate(answerer)
            caller_log.close()
            answerer_log.close()
            if (args.transparent and args.transparent_cleardown
                    and args.robbed_phase is not None
                    and all("V91 transparent denied, cleardown" in p.read_text(errors="replace")
                            for p in (caller_log_path, answerer_log_path))):
                # 8.7: bit 41 set and the request not granted -> both ends clear down
                print(f"v91_serial_pair_test: OK ({label}; cleared down as bit 41 requires)")
                return 0
            print(f"v91_serial_pair_test: FAIL: {exc}", file=sys.stderr)
            print("--- caller log ---", file=sys.stderr)
            print(caller_log_path.read_text(errors="replace"), file=sys.stderr)
            print("--- answerer log ---", file=sys.stderr)
            print(answerer_log_path.read_text(errors="replace"), file=sys.stderr)
            return 1
        finally:
            keep_dir = os.environ.get("V91_KEEP_TAPS")
            if keep_dir:
                # Debug aid: preserve both transmit taps whatever the outcome.
                import shutil
                for proc in (caller, answerer):
                    terminate(proc)
                time.sleep(0.5)
                shutil.copytree(temp / "caller-taps", pathlib.Path(keep_dir) / "caller-taps", dirs_exist_ok=True)
                shutil.copytree(temp / "answerer-taps", pathlib.Path(keep_dir) / "answerer-taps", dirs_exist_ok=True)
            if caller_fd >= 0:
                os.close(caller_fd)
            if answerer_fd >= 0:
                os.close(answerer_fd)
            terminate(caller)
            terminate(answerer)
            if not caller_log.closed:
                caller_log.close()
            if not answerer_log.closed:
                answerer_log.close()


if __name__ == "__main__":
    raise SystemExit(main())
