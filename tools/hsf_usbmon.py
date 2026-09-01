#!/usr/bin/env python3
"""Decode a usbmon capture of the Conexant HSF USB modem (0572:1300).

Turns a raw dump into the one thing we could not derive from source or from
probing the device: the CD2_CONTROL_SCRIPT bodies, and which one is off-hook.

We already know a great deal, and this uses all of it:
  - the seven vendor requests, and the full framing of CD2_UPLOAD_FIRMWARE,
    which lets the tool VERIFY ITSELF against traffic we understand before
    anything it says about the unknown parts is worth believing
  - the SmartDAA relay words from hsf.cty, so a script body carrying 0x80A6
    (off-hook) or 0x80AD (on-hook monitor) is recognised on sight
  - that the firmware ORs 0x80 in itself (ROM_IMAGE 0A88), so the LOW BYTE
    alone (A6, AD, B5...) is just as likely to be what travels

  python3 tools/hsf_usbmon.py capture/*.mon
"""
import argparse
import binascii
import sys
from collections import Counter, defaultdict

VID, PID = 0x0572, 0x1300

CD2 = {0: "GET_INFROMATION", 1: "UPLOAD_FIRMWARE", 2: "CONTROL_SCRIPT",
       3: "READ_EEPROM", 4: "WRITE_EEPROM", 5: "RESET", 6: "WAKEONRING"}

# hsf.cty SMART_RELAYS, indexed by DEVMGR_DAA_RELAY_CODE (configtypes.h:407)
RELAY = {
    0xFFFF: "GPIO_RELAY_MASK", 0x80B5: "GPIO_DEFAULT / ONHOOK_PHONETOLINE_NOCALLID",
    0x80B6: "OFFHOOK_PHONETOLINE", 0x80A6: "OFFHOOK_PHONEOFFLINE  <<< SEIZE",
    0x80BD: "ONHOOK_PHONETOLINE_CALLID", 0x80AD: "ONHOOK_PHONEOFFLINE_CALLID  <<< LISTEN",
    0x80A5: "ONHOOK_PHONEOFFLINE_NOCALLID", 0x80A4: "PULSE_MAKE / PULSESETUP / PULSECLEAR",
}

hx = lambda b: binascii.hexlify(b).decode()


def parse(path):
    """usbmon text format:
         tag ts type pipe [s rt req val idx len | status] datalen [= words...]
    """
    out = []
    with open(path, errors="replace") as f:
        for line in f:
            t = line.split()
            if len(t) < 6:
                continue
            tag, ts, ev, pipe = t[0], t[1], t[2], t[3]
            if ":" not in pipe:
                continue
            kind, rest = pipe[0], pipe[1:]
            try:
                _dirn, bus, dev, ep = rest.split(":")
            except ValueError:
                continue
            i = 4
            setup = None
            if t[i] == "s":                       # control setup packet
                try:
                    setup = (int(t[i+1], 16), int(t[i+2], 16), int(t[i+3], 16),
                             int(t[i+4], 16), int(t[i+5], 16))
                except (IndexError, ValueError):
                    continue
                i += 6
            else:
                i += 1                            # status
            try:
                dlen = int(t[i]); i += 1
            except (IndexError, ValueError):
                continue
            data = b""
            if i < len(t) and t[i] == "=":
                data = binascii.unhexlify("".join(t[i+1:]).replace("_", ""))
            out.append(dict(tag=tag, ts=int(ts), ev=ev, kind=kind, ep=int(ep),
                            setup=setup, dlen=dlen, data=data, pipe=pipe))
    return out


def relay_hits(body):
    """Look for the known relay words, both byte orders, and the low bytes on
    their own -- the firmware ORs 0x80 in itself, so the wire may carry A6."""
    hits = []
    for w, name in RELAY.items():
        for pat, order in ((bytes([w & 0xFF, w >> 8]), "LE"),
                           (bytes([w >> 8, w & 0xFF]), "BE")):
            off = body.find(pat)
            while off != -1:
                hits.append((off, order, w, name))
                off = body.find(pat, off + 1)
    lows = {0xA4: "PULSE*", 0xA5: "ONHOOK_NOCALLID", 0xA6: "OFFHOOK  <<<",
            0xAD: "ONHOOK_CALLID  <<<", 0xB5: "DEFAULT", 0xB6: "OFFHOOK_PHONETOLINE",
            0xBD: "ONHOOK_CALLID_PHONETOLINE"}
    for off, b in enumerate(body):
        if b in lows:
            hits.append((off, "low byte", b, lows[b]))
    return hits


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="+")
    ap.add_argument("--all", action="store_true", help="dump every vendor request")
    args = ap.parse_args()

    for path in args.files:
        recs = parse(path)
        if not recs:
            print(f"\n=== {path}: no usbmon records parsed ===")
            continue
        print(f"\n=== {path}  ({len(recs)} records) ===")

        vendor, scripts, notes = [], [], []
        bulk_in = bulk_out = 0
        fw_blocks = 0

        for r in recs:
            s = r["setup"]
            if s:
                rt, req, val, idx, ln = s
                if (rt & 0x60) == 0x40:                       # vendor
                    vendor.append((r, rt, req, val, idx, ln))
                    if req == 1:
                        fw_blocks += 1
                    elif req == 2 and r["data"]:
                        scripts.append((r, rt, val, idx, r["data"]))
            elif r["kind"] == "B":
                if r["ep"] & 0x80 or "Bi" in r["pipe"]:
                    bulk_in += len(r["data"]) or max(r["dlen"], 0)
                else:
                    bulk_out += len(r["data"]) or max(r["dlen"], 0)
            elif r["kind"] == "I" and r["ev"] == "C" and r["data"]:
                notes.append(r)

        # --- self-check: do we recognise the traffic we already understand?
        counts = Counter(req for _, _, req, _, _, _ in vendor)
        print("  vendor requests seen:")
        for req, n in sorted(counts.items()):
            print(f"    {req} {CD2.get(req, '?'):<16s} x{n}")
        if fw_blocks:
            expect = -(-7399 // 64)
            print(f"  firmware upload: {fw_blocks} blocks "
                  f"({'matches' if fw_blocks == expect else f'expected {expect}'} "
                  f"-- parser sanity check)")
        if not counts:
            print("  NO vendor traffic -- wrong bus, or the driver never bound.")

        # --- the actual prize
        if scripts:
            print(f"\n  CD2_CONTROL_SCRIPT bodies ({len(scripts)}):")
            for r, rt, val, idx, body in scripts:
                print(f"    t={r['ts']} rt=0x{rt:02x} wValue={val} "
                      f"wIndex={idx} len={len(body)}")
                for o in range(0, len(body), 16):
                    print(f"      {o:04x}  {hx(body[o:o+16])}")
                for off, order, w, name in relay_hits(body):
                    shown = f"{w:04X}" if w > 0xFF else f"{w:02X}"
                    print(f"      ^ offset {off}: {shown} ({order}) = {name}")
        else:
            print("\n  no CONTROL_SCRIPT bodies in this file")

        if notes:
            print(f"\n  notifications ({len(notes)}):")
            seen = Counter()
            for r in notes:
                d = r["data"]
                seen[hx(d)] += 1
            for payload, n in seen.most_common():
                b = binascii.unhexlify(payload)
                code = b[1] if len(b) > 1 else None
                names = {0x00: "NETWORK_CONNECTION", 0x01: "RESPONSE_AVAILABLE",
                         0x08: "AUX_JACK_HOOK_STATE", 0x09: "RING_DETECT  <<<",
                         0x20: "SERIAL_STATE", 0x28: "CALL_STATE_CHANGE",
                         0x29: "LINE_STATE_CHANGE"}
                print(f"    x{n:<4d} {payload}"
                      + (f"   code 0x{code:02x} {names.get(code, '')}" if code is not None else ""))

        if bulk_in or bulk_out:
            print(f"\n  bulk: in {bulk_in} bytes, out {bulk_out} bytes")

        if args.all and vendor:
            print("\n  all vendor requests:")
            for r, rt, req, val, idx, ln in vendor:
                print(f"    t={r['ts']} rt=0x{rt:02x} {CD2.get(req,'?')} "
                      f"wValue={val} wIndex={idx} len={ln} {hx(r['data'])[:64]}")

    print("\nWhat to look for: a CONTROL_SCRIPT body present in the off-hook")
    print("capture and absent from the on-hook one, or differing in one byte")
    print("between them.  Diff 02-offhook against 03-onhook first.")


if __name__ == "__main__":
    main()
