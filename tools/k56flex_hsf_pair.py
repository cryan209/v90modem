#!/usr/bin/env python3
"""Bounded whole-engine K56flex call against cruft's native legacy HSF client.

Direct byte-exact PCMU socket, no SIP/PBX or forced gates. The peer uses its
normal Flex-only V.8bis -> V.8 -> K56 startup. Carrier observations alone
are not success: --payload-bytes/--require-payload checks actual duplex DTE.
"""
import argparse
import errno
import hashlib
import json
import os
import re
from pathlib import Path
import socket
import struct
import subprocess
import sys
import tempfile
import time
import tty

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def drain(fd, output):
    while True:
        try:
            data = os.read(fd, 4096)
        except OSError as exc:
            if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK, errno.EIO):
                return
            raise
        if not data:
            return
        output.extend(data)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--cruft", type=Path, default=ROOT.parent / "cruft")
    ap.add_argument("--seconds", type=int, default=35)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--engine", type=Path, default=ROOT / "audio_sock_modem")
    ap.add_argument("--max-upstream-rate", type=int, default=9600,
                    help="native client's upstream ceiling (bit/s, multiple of 2400)")
    ap.add_argument("--dac-taps", type=int, default=129,
                    help="analogue peer reconstruction FIR length (DS0 remains unchanged)")
    ap.add_argument("--require-startup", action="store_true",
                    help="fail unless native K56 reaches Phase 3 through real V.8/INFO1")
    ap.add_argument("--require-audio-gates", action="store_true",
                    help="require audio-derived S/S-bar/report gates and native Phase 4 (not CONNECT)")
    ap.add_argument("--payload-bytes", type=int, default=0,
                    help="check this many raw 8N1 host bytes in each direction after carrier")
    ap.add_argument("--require-payload", action="store_true")
    args = ap.parse_args()
    if not 1 <= args.seconds <= 120:
        ap.error("--seconds must be 1..120")
    if not 9 <= args.dac_taps <= 513 or args.dac_taps % 4 != 1:
        ap.error("--dac-taps must be 4n+1 in 9..513")
    if not 0 <= args.payload_bytes <= 65536 or (args.require_payload and not args.payload_bytes):
        ap.error("--payload-bytes must be 0..65536, and nonzero with --require-payload")
    cruft = args.cruft.resolve()
    sys.path.insert(0, str(cruft))
    from native.hsf.k56_client import K56Client, PCMUStream
    from native.hsf.pcm import LIBRARY
    from native.hsf.serial_bits import encode_8n1, Decoder8N1

    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    executable = args.engine.resolve()
    env = dict(os.environ, ME_K56FLEX="probe", ME_MODE="k56", ME_V8="1",
               ME_MEDIA_CLOCK="1", VPCM_G711_TAP_DIR=str(out))
    client = K56Client(modulation="flex", max_upstream_rate=args.max_upstream_rate, dac_taps=args.dac_taps)
    phase2_events = []
    symbol_events = []
    probe_clock_events = []
    last_probe_clock = None
    last_phase2_state = None
    last_symbol_state = None
    carrier_seen, carrier_active, carrier_interruptions = False, False, 0

    def observe(peer):
        nonlocal last_phase2_state, last_symbol_state, carrier_seen, carrier_active, carrier_interruptions, last_probe_clock
        if peer.pump is None:
            return
        carrier = peer.pump.env.datapump_carrier == 3 and bool(peer.pump.state()["data"])
        if carrier_active and not carrier:
            carrier_interruptions += 1
        carrier_active = carrier
        carrier_seen |= carrier
        # Read-only original K56 diagnostics. 1203a1df branches to state
        # 0x28/1203b470 (DEMY) on a low-power rejection. Only 1203a3a4
        # changes RX+66c0 to the actual symbol path after INFO1/70 ms.
        pump = peer.pump
        cpu = pump.cpu
        rx = pump.object + 0x2338
        config = cpu._read32(rx + 0x4b0)
        # Original 1203c700 fits phase change across nine probe tones.
        # RX+a94 is its probe-analysis object; 2474/2478 accumulate timing
        # slope/count, copied by 1201f... into the PCM resampler. Read only.
        probe = rx + 0xa94
        probe_clock = (cpu._read32(probe + 0x2a4), cpu._read32(probe + 0x2478))
        if probe_clock != last_probe_clock and 1 <= probe_clock[1] <= 12:
            last_probe_clock = probe_clock
            floating_probe = lambda offset: struct.unpack("<f", cpu._read32(probe + offset).to_bytes(4, "little"))[0]
            probe_clock_events.append({"block": peer.block, "part": probe_clock[0],
                "count": probe_clock[1], "stage": cpu._read32(probe + 0x2a0),
                "frequency_offset": floating_probe(0x2470),
                "timing_sum": floating_probe(0x2474),
                "tone_phase_differences": [floating_probe(0x243c + 4*i) for i in range(9)]})
        # Original K56 symbol state machine (12022710 onward). Read only;
        # these diagnostics never feed a gate, decision, or waveform.
        symbol_state = (cpu._read32(rx + 0x94f4), cpu._read32(rx + 0x97d8))
        if (cpu._read32(rx + 0x66c0) or last_symbol_state is not None) and (
                symbol_state != last_symbol_state or peer.block % 16 == 0
                or cpu._read32(rx + 0x7f2c) > 16000
                or symbol_state[1] == 12
                or (symbol_state[1] == 7 and cpu._read32(rx + 0x7f2c) > 8000)):
            last_symbol_state = symbol_state
            floating_at = lambda address: struct.unpack("<f", cpu._read32(address).to_bytes(4, "little"))[0]
            signed_at = lambda address: struct.unpack("<i", cpu._read32(address).to_bytes(4, "little"))[0]
            dfe = cpu._read32(rx + 0x7370)
            dfe_taps = [signed_at(dfe + 4*i)*floating_at(0x1200621c) for i in range(16)]
            dfe_reference = floating_at(0x12006210) - sum(dfe_taps)*floating_at(0x12006220)
            symbol_events.append({
                "block": peer.block, "handler": hex(symbol_state[0]),
                "state": symbol_state[1], "clock": cpu._read32(rx + 0x8060),
                "counter": cpu._read32(rx + 0x7f2c),
                "receiver": cpu._read32(rx + 0x7f28),
                "loop6_variance": sum((x - dfe_reference)**2 for x in dfe_taps),
                "loop6_threshold": floating_at(0x12006224),
                "dfe_taps": dfe_taps,
                "training_generator": [cpu._read32(rx + 0x97e0 + 4*i) for i in range(10)],
                "training_level_table": [struct.unpack("<h", bytes(cpu._read8(0x12009a54 + 2*i + j) for j in range(2)))[0] for i in range(12)],
                "pad_level_table": [struct.unpack("<h", bytes(cpu._read8(0x12009b34 + 2*i + j) for j in range(2)))[0] for i in range(8)],
                "training_mode": cpu._read32(cpu._read32(rx + 0x94d8) + 0xde8),
                "probe_drift": floating_at(rx + 0x7d74),
                "resampler_rate": signed_at(rx + 0x7164),
                "resampler_correction": signed_at(rx + 0x809c),
                "r_detector": [cpu._read32(rx + 0x9788 + 4*i) for i in range(13)],
                "equalized_sample": struct.unpack("<h", bytes(cpu._read8(rx + 0x8090 + j) for j in range(2)))[0],
                "sliced_sample": struct.unpack("<h", bytes(cpu._read8(rx + 0x7d70 + j) for j in range(2)))[0],
                "pad_statistics": [[cpu._read32(rx + 0x9580 + group + 4*i) for i in range(6)]
                                   for group in (0x154, 0x16c, 0x184, 0x19c, 0x1b4)],
                "pad_decisions": [[signed_at(rx + 0x9580 + 20*i + 4*j) for j in range(5)] for i in range(6)],
                "policy": peer.pump.link_diagnostics(),
            })
        state = (cpu._read32(rx + 0x28), cpu._read32(rx + 0x66c0))
        if state == last_phase2_state:
            return
        last_phase2_state = state
        floating = lambda address: struct.unpack("<f", cpu._read32(address).to_bytes(4, "little"))[0]
        limits = struct.unpack("<hh", cpu._read32(config + 0x14).to_bytes(4, "little"))
        phase2_events.append({"block": peer.block, "rx_state": state[0],
                              "symbol_path": state[1],
                              "handler": hex(cpu._read32(rx + 0x14)),
                              "flex_classifier": hex(cpu._read32(cpu._read32(rx) + 0x84)),
                              "flex_symbol_receiver": cpu._read32(rx + 0x7f28),
                              # 1201efa0 compares these original INFO1 fields:
                              # enable bit 0, server law bit 2/client law bit 3.
                              "server_k56_flags": cpu._read32(cpu._read32(rx + 0x4ac) + 0x30) & 0x3ff,
                              "client_k56_flags": cpu._read32(cpu._read32(rx + 0x4a4) + 0x42) & 0x3ff,
                              # 1203a231 passes RX+a94 as this to 1203d7a0.
                              "measured_power": floating(rx + 0xa94 + 0x24b4),
                              "power_limits": list(limits),
                              "power_threshold": sum(limits)})

    stream = PCMUStream(client, observe)
    upstream, downstream, dte, client_received = bytearray(), bytearray(), bytearray(), bytearray()
    client_serial, serial_decoder = bytearray(), Decoder8N1()
    expected_up = hashlib.shake_256(b"v90modem Flex upstream payload").digest(args.payload_bytes)
    expected_down = hashlib.shake_256(b"v90modem Flex downstream payload").digest(args.payload_bytes)
    packed_up = encode_8n1(expected_up)
    sent_up = sent_down = 0
    carrier_frame = None
    fd, frames, end_reason = None, 0, "duration limit"
    start_wall = time.monotonic()
    final_state = None
    proc = None
    try:
        # Keep socket paths short enough for macOS sockaddr_un. All paths
        # are unique; there are no fixed scratch-file names across runs.
        with tempfile.TemporaryDirectory(prefix="k56hsf-") as scratch:
            sock_path = Path(scratch) / "audio"
            pty_path = Path(scratch) / "dte"
            with (out / "server.log").open("wb") as log:
                proc = subprocess.Popen(
                    [str(executable), "--listen", str(sock_path),
                     "--pty-link", str(pty_path), "--verbose"],
                    cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
                try:
                    deadline = time.monotonic() + 15
                    while not (sock_path.exists() and pty_path.exists()):
                        if proc.poll() is not None:
                            raise RuntimeError("engine exited during startup; see server.log")
                        if time.monotonic() > deadline:
                            raise RuntimeError("engine socket/PTY startup timed out")
                        time.sleep(0.02)
                    fd = os.open(pty_path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
                    tty.setraw(fd)
                    # The native peer's host is raw 8N1 with no V.42, so ask
                    # for none (V.250 +ES=1, V.14 only). Otherwise our V.42
                    # 8.10 detection pattern (ODP, DC1 even/odd parity)
                    # reaches its host ahead of the payload.
                    os.write(fd, b"AT+ES=1\r")
                    time.sleep(0.1)
                    os.write(fd, b"ATD1\r")
                    # Let the DTE thread accept ATD before connecting. The
                    # socket log is checked below: incoming/answer role is
                    # not a valid substitute for the intended digital caller.
                    time.sleep(0.2)
                    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
                        sock.settimeout(15)
                        sock.connect(str(sock_path))
                        tx = b"\xff" * 160
                        for _ in range(args.seconds * 50):
                            if time.monotonic() - start_wall > 180:
                                raise RuntimeError("wall-clock limit")
                            try:
                                sock.sendall(tx)
                                incoming = bytearray()
                                while len(incoming) < 160:
                                    data = sock.recv(160 - len(incoming))
                                    if not data:
                                        break
                                    incoming.extend(data)
                            except (BrokenPipeError, ConnectionResetError):
                                end_reason = "engine closed audio socket"
                                break
                            if len(incoming) != 160:
                                end_reason = "engine closed audio socket"
                                break
                            upstream.extend(tx)
                            downstream.extend(incoming)
                            tx = stream.exchange(bytes(incoming))
                            if client.pump:
                                # Drain the local host queue, including idle
                                # packed marks. This never controls the peer.
                                received = client.pump.read()
                                client_received.extend(received)
                                client_serial.extend(serial_decoder.feed(received))
                            frames += 1
                            drain(fd, dte)
                            if (args.payload_bytes and client.pump
                                    and client.pump.env.datapump_carrier == 3
                                    and client.pump.state()["data"] and b"CONNECT" in dte):
                                if carrier_frame is None:
                                    carrier_frame = frames
                                # A host-side two-second idle guard. This
                                # schedules DTE data, never training/gates.
                                if frames >= carrier_frame + 100:
                                    if sent_up < len(packed_up):
                                        sent_up += client.pump.write(packed_up[sent_up:])
                                    if sent_down < len(expected_down):
                                        try:
                                            sent_down += os.write(fd, expected_down[sent_down:])
                                        except BlockingIOError:
                                            pass
                            if client.failure:
                                end_reason = client.failure
                                break
                    drain(fd, dte)
                finally:
                    if fd is not None:
                        os.close(fd)
                        fd = None
                    proc.terminate()
                    try:
                        proc.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait()
        final_state = client.pump.state() if client.pump else None
        (out / "client-tx.g711").write_bytes(upstream)
        (out / "server-tx.g711").write_bytes(downstream)
        (out / "dte.bin").write_bytes(dte)
        (out / "client-received-packed.bin").write_bytes(client_received)
        (out / "client-received-serial.bin").write_bytes(client_serial)
        (out / "client-events.json").write_text(json.dumps(client.events, indent=2) + "\n")
        (out / "client-phase2.json").write_text(json.dumps(phase2_events, indent=2) + "\n")
        (out / "client-symbols.json").write_text(json.dumps(symbol_events, indent=2) + "\n")
        (out / "client-probe-clock.json").write_text(json.dumps(probe_clock_events, indent=2) + "\n")
        server_log = (out / "server.log").read_text(errors="replace")
        if "outgoing (completes ATD)" not in server_log:
            raise RuntimeError("wrong engine role; see server.log")
        notices = [notice for event in client.events
                   for notice in event.get("notifications", [])]
        phase3_seen = any(e["symbol_path"] == 1 and e["rx_state"] != 0x28
                          for e in phase2_events)
        flex_phase3_seen = any(e["symbol_path"] == 1 and e["rx_state"] != 0x28
                               and e["flex_symbol_receiver"] == 1
                               for e in phase2_events)
        phase2_handoff = (client.flex_agreed and client.v8_modulations == 1
                          and client.selected_modulation == "k56" and flex_phase3_seen
                          and "K56flex Phase 2 complete; starting PCM probes" in server_log)
        native_phase4 = any(13 <= e["state"] <= 20 and e["receiver"] == 1
                            for e in symbol_events)
        sbar_from_audio = "K56flex upstream S/S-bar from audio" in server_log
        report_from_audio = "K56flex upstream report from audio" in server_log
        end_from_audio = "K56flex upstream 899F from audio" in server_log
        connect = re.search(rb"\r\nCONNECT[^\r\n]*\r\n", dte)
        server_serial = bytes(dte[connect.end():]) if connect else b""
        payload_ok = bool(args.payload_bytes and connect and final_state and final_state["data"]
                          and client.pump.env.datapump_carrier == 3
                          and carrier_seen and carrier_interruptions == 0
                          and sent_up == len(packed_up) and sent_down == len(expected_down)
                          and server_serial == expected_up and client_serial == expected_down
                          and serial_decoder.framing_errors == 0)
        (out / "server-received-serial.bin").write_bytes(server_serial)
        report = {
            "transport": "lockstep 8 kHz PCMU, whole audio_sock_modem engine",
            "peer": "cruft native legacy HSF, Flex-only (no V.90/V.34 fallback)",
            "dac_taps": args.dac_taps,
            "audio_seconds": frames / 50, "end_reason": end_reason,
            "wall_seconds": time.monotonic() - start_wall,
            "flex_agreed": client.flex_agreed,
            "v8_started": client.v8 is not None,
            "v8_modulations": client.v8_modulations,
            "selected_modulation": client.selected_modulation,
            "phase2_handoff": phase2_handoff,
            "client_phase3_seen": phase3_seen,
            "client_flex_phase3_seen": flex_phase3_seen,
            "client_phase2_rejected": any(e["rx_state"] == 0x28 for e in phase2_events),
            "client_phase4_seen": native_phase4,
            "server_sbar_from_audio": sbar_from_audio,
            "server_report_from_audio": report_from_audio,
            "server_jtag_from_audio": end_from_audio,
            "final_client_state": final_state, "failure": client.failure,
            "client_carrier": client.pump.env.datapump_carrier if client.pump else 0,
            "client_rates": client.pump.negotiated_rates() if client.pump else None,
            "client_carrier_interruptions": carrier_interruptions,
            "client_received_packed_bytes": len(client_received),
            "payload_bytes_requested": args.payload_bytes,
            "upstream_serial_bytes": len(server_serial),
            "downstream_serial_bytes": len(client_serial),
            "client_framing_errors": serial_decoder.framing_errors,
            "upstream_payload_exact": bool(args.payload_bytes and server_serial == expected_up),
            "downstream_payload_exact": bool(args.payload_bytes and client_serial == expected_down),
            "client_tx_training_completed": any(n[0] == 1000 for n in notices),
            "client_rx_training_completed": any(n[0] == 1001 for n in notices),
            "server_dte_connect": b"CONNECT" in dte,
            "verified_connection": payload_ok,
            "limits": ("Checked raw duplex payload over local PCMU; no hardware qualification."
                       if payload_ok else "No checked duplex payload qualification."),
            "inputs": {str(executable): digest(executable), str(LIBRARY): digest(LIBRARY),
                       str(Path(__file__).resolve()): digest(Path(__file__))},
        }
        (out / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2))
        if args.require_startup and not phase2_handoff:
            raise SystemExit("K56flex V.8/Phase-2 startup requirement failed")
        if args.require_audio_gates and not (phase2_handoff and native_phase4
                                             and sbar_from_audio and report_from_audio):
            raise SystemExit("K56flex audio-gate/Phase-4 requirement failed")
        if args.require_payload and not payload_ok:
            raise SystemExit("K56flex checked duplex payload requirement failed")
    finally:
        client.close()
        if proc is not None and proc.poll() is None:
            proc.kill()
            proc.wait()


if __name__ == "__main__":
    main()
