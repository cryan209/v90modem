#!/usr/bin/env python3
"""Decode a usbmon *pcap* capture of the Conexant HSF USB modem (0572:1300).

The usbmon TEXT interface truncates payloads at 32 bytes, which loses both the
tail of a CD2_CONTROL_SCRIPT body and all of the bulk audio.  tcpdump on the
usbmon interface keeps the lot; this reads that (DLT_USB_LINUX_MMAPPED, 220).

  python3 tools/hsf_pcap.py capture.pcap --scripts
  python3 tools/hsf_pcap.py capture.pcap --bulk-out out.raw
"""
import argparse
import struct
import sys

DLT_USB_LINUX_MMAPPED = 220
XFER = {0: "ISO", 1: "INT", 2: "CTRL", 3: "BULK"}


def records(path):
    d = open(path, "rb").read()
    magic, vmaj, vmin, tz, sf, snap, net = struct.unpack("<IHHiIII", d[:24])
    if net != DLT_USB_LINUX_MMAPPED:
        sys.exit("not a usbmon mmapped capture (linktype %d)" % net)
    off = 24
    while off + 16 <= len(d):
        ts, tus, caplen, origlen = struct.unpack("<IIII", d[off:off + 16])
        off += 16
        pkt = d[off:off + caplen]
        off += caplen
        if len(pkt) < 64:
            continue
        # struct usbmon_packet (64 bytes)
        (urb_id, ev_type, xfer_type, epnum, devnum, busnum, flag_setup,
         flag_data, sec, usec, status, length, len_cap) = struct.unpack(
            "<QBBBBHccqiIII", pkt[:40])
        setup = pkt[40:48]
        data = pkt[64:64 + len_cap]
        yield dict(ts=ts + tus / 1e6, ev=chr(ev_type), xfer=xfer_type,
                   ep=epnum, dev=devnum, bus=busnum,
                   setup=setup if flag_setup == b"\x00" else None,
                   status=status, data=data)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("pcap")
    ap.add_argument("--scripts", action="store_true",
                    help="dump CD2_CONTROL_SCRIPT bodies in full")
    ap.add_argument("--bulk-out", metavar="PATH",
                    help="concatenate bulk-OUT payloads to a file")
    ap.add_argument("--bulk-in", metavar="PATH",
                    help="concatenate bulk-IN payloads to a file")
    ap.add_argument("--summary", action="store_true")
    a = ap.parse_args()

    counts, t0 = {}, None
    bo = bi = 0
    fo = open(a.bulk_out, "wb") if a.bulk_out else None
    fi = open(a.bulk_in, "wb") if a.bulk_in else None
    for r in records(a.pcap):
        if t0 is None:
            t0 = r["ts"]
        key = "%s ep%02x %s" % (XFER.get(r["xfer"], "?"), r["ep"], r["ev"])
        counts[key] = counts.get(key, 0) + 1
        if r["xfer"] == 2 and r["setup"] and r["ev"] == "S":
            bmr, breq, wval, widx, wlen = struct.unpack("<BBHHH", r["setup"])
            if a.scripts and breq == 0x02:
                print("t=%.6f rt=0x%02x req=%d wValue=0x%04x wIndex=%d len=%d"
                      % (r["ts"] - t0, bmr, breq, wval, widx, wlen))
                print("    " + r["data"].hex())
        if r["xfer"] == 3 and r["ev"] == "C" and r["data"]:
            if r["ep"] & 0x80:
                bi += len(r["data"])
                if fi:
                    fi.write(r["data"])
            else:
                bo += len(r["data"])
                if fo:
                    fo.write(r["data"])
        elif r["xfer"] == 3 and r["ev"] == "S" and r["data"] and not (r["ep"] & 0x80):
            bo += len(r["data"])
            if fo:
                fo.write(r["data"])
    if a.summary or not (a.scripts or fo or fi):
        for k in sorted(counts):
            print("%-16s %d" % (k, counts[k]))
        print("bulk OUT bytes: %d   bulk IN bytes: %d" % (bo, bi))
    for f in (fo, fi):
        if f:
            f.close()


if __name__ == "__main__":
    main()
