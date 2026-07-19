#!/usr/bin/env python3
"""Voice-mode (V.253 FCLASS=8) AT driver for the USR 56K FAX EXT on
/dev/cu.usbserial-21240, used by tools/voice_pcm_fidelity.py.

Confirmed live against this modem (2026-07-19): AT+FCLASS=? doesn't
advertise "8" but AT+FCLASS=8 is accepted; AT+VSM=? lists:
  128 8-BIT LINEAR, 129 16-BIT LINEAR, 130 8-BIT ALAW, 131 8-BIT ULAW
(8000 Hz only for A-law/u-law). Data during AT+VTX/AT+VRX is framed per
V.253 10.1: 0x10 (DLE) bytes in the stream are escaped as 0x10 0x10.

The DTE-initiated end-of-data codes are NOT symmetric on this modem:
- Ending our own +VTX playback (host->modem): DLE-ETX (0x10 0x03) works
  cleanly -- confirmed, AT returns a clean OK right after.
- Ending an active +VRX capture (modem->host): DLE-ETX does *not* stop it;
  the modem keeps streaming (mostly silence/near-zero codewords, plus
  occasional in-band DLE event codes like DLE-'R' for ring, DLE-'b' for
  busy) indefinitely, wedging the port until every later AT command reads
  back as garbled binary. DLE-EOT (0x10 0x04) is what actually terminates
  it (confirmed live by recovering a wedged modem with it). Use DLE-EOT to
  end +VRX; if a session ever gets stuck anyway, recovery is DLE-EOT sent
  standalone with ~1s of quiet before/after, then a plain "AT\\r" probe.
"""
from __future__ import annotations

import argparse
import sys
import time

import serial

DLE = 0x10
ETX = 0x03
EOT = 0x04


def open_port(dev: str) -> serial.Serial:
    return serial.Serial(dev, 115200, timeout=0.1, rtscts=False, dsrdtr=False)


def send(port: serial.Serial, cmd: str, wait: float = 2.0, quiet: bool = False) -> str:
    port.reset_input_buffer()
    port.write((cmd + "\r").encode())
    port.flush()
    deadline = time.time() + wait
    buf = b""
    while time.time() < deadline:
        chunk = port.read(256)
        if chunk:
            buf += chunk
            text = buf.decode(errors="replace")
            if any(t in text for t in ("OK", "ERROR", "CONNECT")):
                time.sleep(0.1)
                buf += port.read(1024)
                break
    text = buf.decode(errors="replace")
    if not quiet:
        print(f">>> {cmd}", flush=True)
        print(text.strip() or "(no response)", flush=True)
    return text


def dle_stuff(data: bytes) -> bytes:
    return data.replace(bytes([DLE]), bytes([DLE, DLE]))


def dle_unstuff_stream(port: serial.Serial, out_path: str, duration: float) -> int:
    """Read from port until DLE-ETX or duration elapses; de-stuff DLE bytes;
    write payload to out_path. Returns bytes written."""
    deadline = time.time() + duration
    pending_dle = False
    written = 0
    with open(out_path, "wb") as f:
        while time.time() < deadline:
            chunk = port.read(4096)
            if not chunk:
                continue
            out = bytearray()
            for b in chunk:
                if pending_dle:
                    pending_dle = False
                    if b == ETX:
                        f.write(out)
                        return written + len(out)
                    if b == DLE:
                        out.append(DLE)
                    # else: not a valid V.253 escape, drop silently
                elif b == DLE:
                    pending_dle = True
                else:
                    out.append(b)
            if out:
                f.write(out)
                written += len(out)
    return written


def wait_for(port: serial.Serial, tokens: tuple[str, ...], timeout: float) -> str:
    deadline = time.time() + timeout
    buf = b""
    while time.time() < deadline:
        chunk = port.read(256)
        if chunk:
            buf += chunk
            text = buf.decode(errors="replace")
            if any(t in text for t in tokens):
                return text
    return buf.decode(errors="replace")


def setup_voice(port: serial.Serial, vsm: int) -> None:
    send(port, "ATZ", wait=3.0)
    send(port, "ATE1V1Q0", wait=2.0)
    send(port, "AT+FCLASS=8", wait=2.0)
    send(port, f"AT+VSM={vsm}", wait=2.0)


def cmd_probe(opts) -> int:
    port = open_port(opts.dev)
    try:
        send(port, "AT", wait=1.0)
        send(port, "ATI3", wait=2.0)
        send(port, "AT+FCLASS=8", wait=2.0)
        send(port, "AT+VSM=?", wait=2.0)
        send(port, "AT+FCLASS=0", wait=2.0)
    finally:
        port.close()
    return 0


def cmd_dial_vtx(opts) -> int:
    port = open_port(opts.dev)
    try:
        setup_voice(port, opts.vsm)
        payload = open(opts.file, "rb").read()
        print(f"[usr_voice_at] dialing {opts.number}, will VTX {len(payload)} octets on connect", flush=True)
        port.reset_input_buffer()
        port.write((f"ATDT{opts.number}\r").encode())
        port.flush()
        # Voice-mode ATD on this modem has no loop-current/polarity answer
        # supervision, so it never reports CONNECT/NO CARRIER for the dial
        # itself (confirmed live: ATDT returns a bare "OK" then goes quiet).
        # Settle for the far end's known ring/auto-answer cadence instead,
        # then rely on AT+VTX's own CONNECT response as the real signal.
        result = wait_for(port, ("NO CARRIER", "NO DIALTONE", "BUSY", "ERROR"), opts.settle)
        print(f"[usr_voice_at] post-dial settle: {result.strip()!r}", flush=True)
        if any(t in result for t in ("NO CARRIER", "NO DIALTONE", "BUSY", "ERROR")):
            return 1

        vtx_result = send(port, "AT+VTX", wait=2.0)
        if "CONNECT" not in vtx_result:
            print("[usr_voice_at] AT+VTX did not return CONNECT; aborting", flush=True)
            return 1
        framed = dle_stuff(payload) + bytes([DLE, ETX])
        t0 = time.time()
        # write in chunks so we don't overrun the modem's UART buffer
        CHUNK = 480
        for i in range(0, len(framed), CHUNK):
            port.write(framed[i:i + CHUNK])
            port.flush()
            time.sleep(len(framed[i:i + CHUNK]) / 8000.0 * 0.9)
        print(f"[usr_voice_at] VTX stream sent in {time.time() - t0:.1f}s", flush=True)
        time.sleep(0.5)
        send(port, "AT", wait=1.0)  # confirm we're back in command mode
        send(port, "ATH", wait=2.0)
        send(port, "AT+FCLASS=0", wait=2.0)
    finally:
        port.close()
    return 0


def cmd_answer_vrx(opts) -> int:
    port = open_port(opts.dev)
    try:
        setup_voice(port, opts.vsm)
        send(port, f"ATS0={opts.rings}", wait=2.0)
        # Like dial-vtx, this modem gives no reliable CONNECT/VCON text for
        # voice-mode auto-answer either (confirmed live: buffer just fills
        # with ring/noise bytes). Settle for the known ring/auto-answer
        # cadence, drain whatever accumulated, then blindly arm AT+VRX.
        print(f"[usr_voice_at] settling {opts.wait:.0f}s for incoming call auto-answer...", flush=True)
        drained = wait_for(port, ("NO CARRIER", "ERROR"), opts.wait)
        print(f"[usr_voice_at] settle buffer: {drained.strip()!r}", flush=True)
        if any(t in drained for t in ("NO CARRIER", "ERROR")):
            return 1

        send(port, "AT+VRX", wait=1.0, quiet=True)
        print(f"[usr_voice_at] AT+VRX armed, capturing for {opts.duration:.0f}s -> {opts.out}", flush=True)
        n = dle_unstuff_stream(port, opts.out, opts.duration)
        print(f"[usr_voice_at] captured {n} de-stuffed octets", flush=True)

        # DLE-ETX (the correct end-of-data code for our own +VTX playback)
        # does NOT stop an active +VRX capture on this modem -- confirmed
        # live, it just keeps streaming and wedges the port for every later
        # AT command. DLE-EOT (0x10 0x04) is what actually ends it. Retry a
        # couple of times and verify with a plain "AT" probe before giving
        # up, since leaving this modem in that state breaks every later run.
        recovered = False
        for attempt in range(3):
            port.reset_input_buffer()
            port.write(bytes([DLE, EOT]))
            port.flush()
            time.sleep(1.0)
            port.reset_input_buffer()
            port.write(b"AT\r")
            port.flush()
            time.sleep(1.0)
            probe = port.read(200)
            if probe.strip() in (b"AT\r\r\nOK", b"AT\r\nOK", b"OK"):
                recovered = True
                break
            print(f"[usr_voice_at] DLE-EOT attempt {attempt + 1} left port dirty: {probe!r}", flush=True)
        if not recovered:
            print("[usr_voice_at] WARNING: could not confirm clean command-mode recovery after +VRX; "
                  "modem may still be streaming. Manual recovery may be needed.", flush=True)

        send(port, "ATH", wait=2.0)
        send(port, f"ATS0={0}", wait=2.0)
        send(port, "AT+FCLASS=0", wait=2.0)
    finally:
        port.close()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dev", default="/dev/cu.usbserial-21240")
    ap.add_argument("--vsm", type=int, default=130, help="130=8-bit A-law, 131=8-bit u-law (this modem)")
    sub = ap.add_subparsers(dest="mode", required=True)

    p = sub.add_parser("probe")
    p.set_defaults(func=cmd_probe)

    p = sub.add_parser("dial-vtx", help="dial out and transmit a raw G.711 file in voice mode")
    p.add_argument("number")
    p.add_argument("file")
    p.add_argument("--settle", type=float, default=14.0,
                    help="seconds to wait after ATDT for the far end's ring/auto-answer cadence "
                         "(this modem has no answer supervision on voice-mode dial)")
    p.set_defaults(func=cmd_dial_vtx)

    p = sub.add_parser("answer-vrx", help="wait for an incoming call and capture voice-mode RX to a file")
    p.add_argument("out")
    p.add_argument("--duration", type=float, default=20.0)
    p.add_argument("--wait", type=float, default=3.0,
                    help="settle seconds before arming AT+VRX (no answer-supervision text to wait "
                         "for on this modem, so --duration should be generous instead)")
    p.add_argument("--rings", type=int, default=1)
    p.set_defaults(func=cmd_answer_vrx)

    opts = ap.parse_args()
    return opts.func(opts)


if __name__ == "__main__":
    raise SystemExit(main())
