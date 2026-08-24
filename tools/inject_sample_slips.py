#!/usr/bin/env python3
"""Inject clock-recovery sample slips into a recorded G.711 tap.

modem_passthrough_get_frame() in sip_modem.c applies me_cr_get_adjustment()
to every pulled 160-codeword frame: adj > 0 duplicates the frame's last
codeword and feeds 161, adj < 0 feeds only the first 159.  The RX tap is
written inside me_rx_g711(), i.e. AFTER that edit, so a recording already
contains whatever slips the live call injected -- and a replay of it can
never show what they cost.  This reproduces them on a recording that has
none, so the cost can be measured on the desk.

  inject_sample_slips.py in.g711 out.g711 --at SEC[,SEC...] [--mode dup|del|alt]
  inject_sample_slips.py in.g711 out.g711 --start SEC --interval SEC --count N
"""
import argparse

FRAME = 160
RATE = 8000


def main():
    p = argparse.ArgumentParser()
    p.add_argument("src")
    p.add_argument("dst")
    p.add_argument("--at", help="comma-separated seconds")
    p.add_argument("--start", type=float, default=0.0)
    p.add_argument("--interval", type=float, default=10.0)
    p.add_argument("--count", type=int, default=0)
    p.add_argument("--mode", choices=("dup", "del", "alt"), default="alt")
    p.add_argument("--lose", help="comma-separated seconds: drop a whole "
                                  "160-codeword packet, as losing it does today")
    p.add_argument("--conceal", help="comma-separated seconds: replace a whole "
                                     "packet with fill, preserving stream length")
    p.add_argument("--fill", default="0xff",
                   help="concealment codeword (0xff u-law silence, 0xd5 A-law)")
    a = p.parse_args()

    if a.at:
        times = [float(t) for t in a.at.split(",")]
    else:
        times = [a.start + i * a.interval for i in range(a.count)]

    data = bytearray(open(a.src, "rb").read())

    if a.lose or a.conceal:
        fill = int(a.fill, 0)
        pkts = sorted({int(float(t) * RATE) // FRAME
                       for t in (a.lose or a.conceal).split(",")}, reverse=True)
        for fi in pkts:
            lo, hi = fi * FRAME, (fi + 1) * FRAME
            if hi > len(data):
                continue
            if a.lose:
                del data[lo:hi]
            else:
                data[lo:hi] = bytes([fill]) * FRAME
        open(a.dst, "wb").write(bytes(data))
        print("%s %d packet(s) at %s"
              % ("lost" if a.lose else "concealed", len(pkts),
                 ", ".join("%.2fs" % (f * FRAME / RATE) for f in sorted(pkts))))
        return
    # Frame index of each slip, descending so earlier edits keep their offsets.
    frames = sorted({int(t * RATE) // FRAME for t in times}, reverse=True)

    out = data
    for n, fi in enumerate(frames):
        end = (fi + 1) * FRAME
        if end > len(out):
            continue
        mode = a.mode
        if mode == "alt":
            # Alternate, as a DPLL hunting around zero error does.
            mode = "dup" if (len(frames) - 1 - n) % 2 == 0 else "del"
        if mode == "dup":
            out[end:end] = bytes([out[end - 1]])
        else:
            del out[end - 1:end]

    open(a.dst, "wb").write(bytes(out))
    print("injected %d slips (%s) at %s"
          % (len(frames), a.mode,
             ", ".join("%.2fs" % (f * FRAME / RATE) for f in sorted(frames))))


if __name__ == "__main__":
    main()
