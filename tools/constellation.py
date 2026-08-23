#!/usr/bin/env python3
"""Constellation and eye diagnostics for a V.34 / V.90 call, as an HTML page.

The receiver reports its distance from the lattice as one number per window.
That number says a call is failing but not what it is failing *as*: a rotation,
a scale, a collapsed eye and a wrong sampling instant all read the same.  A
constellation does say, at a glance, and it is the instrument a QAM link is
normally judged by.

Reads the symbol dumps the modem already writes:

  V34_DATA_FRAME_DUMP=<p>   -> <p>.answer / <p>.caller, int16 pairs in Q9.7,
                               the RECEIVED (equalized) data-mode symbols
  V34_DATA_TX_DUMP=<p>      -> same format, the symbols we TRANSMITTED
  ME_V90_UPSTREAM_SYM_DUMP  -> text "index re im", the V.90 upstream receiver

and, for the power figures, the G.711 taps VPCM_G711_TAP_DIR leaves behind
(live-rx.g711 / live-tx.g711).

  tools/constellation.py <artifact-dir> [-o out.html] [--symbols N] [--from N]

Stdlib only on purpose: this repo's python3 has no numpy, and a diagnostic
that cannot be run on the machine that made the recording is no diagnostic.
"""
import argparse
import math
import os
import struct
import sys

# 0 dBm0 in the linear units spandsp's G.711 decoders produce.  The overload
# point of both laws is +3.14 dBm0, and ulaw_to_linear()/alaw_to_linear() peak
# at 32124 and 32256, so a 0 dBm0 sine has that amplitude 3.14 dB down, and an
# RMS a further 3.01 dB below the amplitude.  Computed from the tables rather
# than quoted: the "0 dBm0 = RMS 4004" figure belongs to the 8159-full-scale
# convention and is 12 dB out here.
#: Most symbols a scatter is allowed to carry.  Beyond this it saturates and
#: only the page weight grows.
MAX_PLOT = 12000

DBM0_RMS = {"ulaw": 32124/(10**(3.14/20))/math.sqrt(2.0),
            "alaw": 32256/(10**(3.14/20))/math.sqrt(2.0)}


def ulaw2lin(b):
    b = ~b & 0xFF
    s, e, m = b & 0x80, (b >> 4) & 7, b & 0xF
    v = (((m << 1) + 33) << e) - 33
    return -v if s else v


def alaw2lin(b):
    b ^= 0x55
    s, e, m = b & 0x80, (b >> 4) & 7, b & 0xF
    v = (m << 4) + 8 if e == 0 else ((m << 4) + 0x108) << (e - 1)
    return -v if s else v


def read_pairs(path, start, count):
    """int16 pairs in Q9.7 -> list of (re, im)."""
    try:
        raw = open(path, "rb").read()
    except OSError:
        return []
    n = len(raw)//4
    start = max(0, min(start, n))
    end = n if count <= 0 else min(n, start + count)
    out = []
    for i in range(start, end):
        re, im = struct.unpack_from("<hh", raw, i*4)
        out.append((re/128.0, im/128.0))
    return out


def read_text_syms(path, start, count):
    out = []
    try:
        f = open(path)
    except OSError:
        return []
    with f:
        for i, line in enumerate(f):
            if i < start:
                continue
            if count > 0 and len(out) >= count:
                break
            p = line.split()
            if len(p) >= 3:
                try:
                    out.append((float(p[1]), float(p[2])))
                except ValueError:
                    pass
    return out


def tap_rms(path, law):
    """RMS of a G.711 tap, ignoring the digital silence either side of a call."""
    try:
        raw = open(path, "rb").read()
    except OSError:
        return None
    dec = ulaw2lin if law == "ulaw" else alaw2lin
    # Average over the loudest half of one-second blocks: a tap is mostly
    # handshake and silence, and the figure wanted is the level while the call
    # is up, not over the whole file.
    blocks = []
    for a in range(0, len(raw) - 8000, 8000):
        acc = 0
        for b in raw[a:a + 8000:7]:      # every 7th sample is plenty for RMS
            v = dec(b)
            acc += v*v
        blocks.append(acc/len(range(0, 8000, 7)))
    if not blocks:
        return None
    blocks.sort()
    top = blocks[len(blocks)//2:]
    return math.sqrt(sum(top)/len(top))


def lattice_error(syms):
    """Mean square distance to the nearest odd-integer point, and mean power.

    9.x puts every V.34 constellation point on odd integers whatever the rate,
    so this is the receiver's own metric and needs no knowledge of which
    constellation is in use.  2/3 is the value for symbols bearing no relation
    to the lattice at all (uniform over a cell of side 2), which is the number
    to compare against: not zero.
    """
    if not syms:
        return None, None
    err = pwr = 0.0
    for re, im in syms:
        dre = re - (2.0*math.floor((re - 1)/2.0) + 1.0)
        dim = im - (2.0*math.floor((im - 1)/2.0) + 1.0)
        # fold to nearest, not to the cell below
        if dre > 1.0:
            dre -= 2.0
        if dim > 1.0:
            dim -= 2.0
        err += dre*dre + dim*dim
        pwr += re*re + im*im
    return err/len(syms), pwr/len(syms)



def trace_svg(syms, baud, width=880, height=170):
    """CINR against time, one point per window.

    A constellation is the whole call superimposed, so a call that is clean
    for thirty seconds and white for seventy looks merely noisy in it.  This
    is the view that separates the two, and it is the one that names when the
    eye shut -- which is the question every collapse in this receiver has
    turned on.
    """
    win = 2048
    if len(syms) < win*2:
        return ""
    pts = []
    for a in range(0, len(syms) - win, win):
        e, pw = lattice_error(syms[a:a + win])
        if not e or not pw or e <= 0:
            continue
        pts.append((a/float(baud), 10.0*math.log10(pw/e)))
    if len(pts) < 2:
        return ""
    padl, padr, padt, padb = 42, 12, 12, 24
    iw = width - padl - padr
    ih = height - padt - padb
    tmax = max(t for t, _ in pts) or 1.0
    lo, hi = 0.0, 42.0

    def X(t):
        return padl + t/tmax*iw

    def Y(v):
        v = max(lo, min(hi, v))
        return padt + (hi - v)/(hi - lo)*ih

    out = ['<svg viewBox="0 0 %d %d" class="trace">' % (width, height)]
    for g in (0, 10, 20, 30, 40):
        out.append('<line class="grid" x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f"/>'
                   % (padl, Y(g), padl + iw, Y(g)))
        out.append('<text class="tick ry" x="%.1f" y="%.1f">%d</text>'
                   % (padl - 6, Y(g) + 3, g))
    # The band below which the output is white: distance 0.667 against this
    # call's own mean symbol power.
    pw_all = lattice_error(syms)[1] or 1.0
    white_db = 10.0*math.log10(pw_all/(2.0/3.0))
    out.append('<line class="white-line" x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f"/>'
               % (padl, Y(white_db), padl + iw, Y(white_db)))
    out.append('<text class="wlbl" x="%.1f" y="%.1f">white</text>'
               % (padl + iw - 4, Y(white_db) - 4))
    d = ["M%.1f %.1f" % (X(pts[0][0]), Y(pts[0][1]))]
    for t, v in pts[1:]:
        d.append("L%.1f %.1f" % (X(t), Y(v)))
    out.append('<path class="tr" d="%s"/>' % "".join(d))
    for lab in (0.0, tmax/2.0, tmax):
        out.append('<text class="tick" x="%.1f" y="%.1f">%ds</text>'
                   % (X(lab), height - 6, int(round(lab))))
    out.append('<text class="axlbl" x="%.1f" y="%.1f">CINR dB</text>'
               % (4, padt + 8))
    out.append("</svg>")
    return "".join(out)

class Panel:
    def __init__(self, role, label, syms, rms, law, note=""):
        self.role, self.label, self.syms, self.note = role, label, syms, note
        self.err, self.pwr = lattice_error(syms)
        self.cinr = (10.0*math.log10(self.pwr/self.err)
                     if self.err and self.pwr and self.err > 0 else None)
        self.dbm0 = (20.0*math.log10(rms/DBM0_RMS[law])
                     if rms else None)
        self.white = self.err is not None and self.err > 0.55

    def verdict(self):
        if not self.syms:
            return "no symbols recorded"
        if self.white:
            return "OUTPUT IS WHITE — symbols bear no relation to the lattice"
        if self.err > 0.25:
            return "eye open but marginal"
        return "eye open"


def svg(panel, size=360):
    if not panel.syms:
        return '<div class="empty">no symbols</div>'
    lim = 0.0
    for re, im in panel.syms:
        lim = max(lim, abs(re), abs(im))
    lim = max(4.0, math.ceil(lim/4.0)*4.0)
    padl, padr = 26, 40
    padt, padb = 14, 28
    inner = size - padl - padr

    def X(v):
        return padl + (v + lim)/(2*lim)*inner

    def Y(v):
        return padt + (lim - v)/(2*lim)*inner

    h = padt + inner + padb
    parts = ['<svg viewBox="0 0 %d %d" class="con">' % (size, h)]
    step = max(4, int(lim//4))
    t = -int(lim)
    while t <= lim:
        if t:
            parts.append('<line class="grid" x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f"/>'
                         % (X(t), padt, X(t), padt + inner))
            parts.append('<line class="grid" x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f"/>'
                         % (padl, Y(t), padl + inner, Y(t)))
            parts.append('<text class="tick" x="%.1f" y="%.1f">%d</text>'
                         % (X(t), padt + inner + 15, t))
            parts.append('<text class="tick ry" x="%.1f" y="%.1f">%d</text>'
                         % (padl + inner + 6, Y(t) + 3, t))
        t += step
    parts.append('<line class="axis" x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f"/>'
                 % (X(0), padt, X(0), padt + inner))
    parts.append('<line class="axis" x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f"/>'
                 % (padl, Y(0), padl + inner, Y(0)))
    # One path of zero-length segments with round caps, rather than one
    # <circle> per symbol: 20000 circles is a 780 KB page and this is 90 KB.
    # Plot a bounded, evenly spread subset -- the CINR figures and the trace
    # are computed over every symbol, but a scatter saturates long before
    # 200000 points and the page does not have to carry them.
    plot = panel.syms
    if len(plot) > MAX_PLOT:
        stride = len(plot)//MAX_PLOT + 1
        plot = plot[::stride]
    r = 1.7 if len(plot) > 8000 else 2.4
    d = []
    for re, im in plot:
        d.append("M%.1f %.1fh0" % (X(re), Y(im)))
    parts.append('<path class="pts %s" stroke-width="%.1f" d="%s"/>'
                 % ("bad" if panel.white else "ok", r, "".join(d)))
    parts.append("</svg>")
    return "".join(parts)


def fmt(v, unit, nd=0):
    return "—" if v is None else ("%.*f %s" % (nd, v, unit))


def render(panels, title, subtitle, trace="", trace_note=""):
    cells = []
    for p in panels:
        cells.append("""
      <section class="panel">
        <div class="hdr"><span class="role">{role}</span><span class="lbl">{label}</span></div>
        <dl>
          <div><dt>CINR</dt><dd>{cinr}</dd></div>
          <div><dt>POWER</dt><dd>{pwr}</dd></div>
          <div><dt>SYMBOLS</dt><dd>{n}</dd></div>
          <div><dt>TO LATTICE</dt><dd>{err}</dd></div>
        </dl>
        {svg}
        <p class="verdict {cls}">{verdict}</p>
        {note}
      </section>""".format(
            role=p.role, label=p.label,
            cinr=fmt(p.cinr, "dB", 1), pwr=fmt(p.dbm0, "dBm0", 1),
            n="{:,}".format(len(p.syms)),
            err="—" if p.err is None else "%.4f" % p.err,
            svg=svg(p),
            cls="bad" if p.white else ("warn" if (p.err or 0) > 0.25 else "ok"),
            verdict=p.verdict(),
            note='<p class="note">%s</p>' % p.note if p.note else ""))

    return """<title>{title}</title>
<style>
:root{{
  --bg:#f4f6f8; --card:#fff; --ink:#1b2733; --dim:#63727f; --rule:#dfe5ea;
  --pt:#8b9bab; --ok:#1a7f4b; --warn:#9a6a00; --bad:#b3261e; --accent:#0a6ebd;
}}
@media (prefers-color-scheme:dark){{:root:not([data-theme="light"]){{
  --bg:#12171c; --card:#1a2027; --ink:#e6edf3; --dim:#9aa7b2; --rule:#2b343d;
  --pt:#7f8f9f; --ok:#4cc38a; --warn:#d9a441; --bad:#f2726a; --accent:#58a6ff;
}}}}
:root[data-theme="dark"]{{
  --bg:#12171c; --card:#1a2027; --ink:#e6edf3; --dim:#9aa7b2; --rule:#2b343d;
  --pt:#7f8f9f; --ok:#4cc38a; --warn:#d9a441; --bad:#f2726a; --accent:#58a6ff;
}}
*{{box-sizing:border-box}}
body{{margin:0;padding:28px;background:var(--bg);color:var(--ink);
 font:14px/1.5 -apple-system,BlinkMacSystemFont,"Segoe UI",Helvetica,Arial,sans-serif}}
h1{{font-size:15px;font-weight:600;letter-spacing:.14em;text-transform:uppercase;
 margin:0 0 4px;color:var(--ink)}}
.sub{{color:var(--dim);margin:0 0 22px;font-size:13px}}
.wrap{{max-width:900px;margin:0 auto}}
.grid2{{display:grid;grid-template-columns:repeat(auto-fit,minmax(330px,1fr));gap:18px}}
.panel{{background:var(--card);border:1px solid var(--rule);border-radius:10px;padding:18px}}
.hdr{{display:flex;justify-content:space-between;align-items:baseline;
 border-bottom:1px solid var(--rule);padding-bottom:10px;margin-bottom:12px}}
.role{{font-size:11px;letter-spacing:.12em;text-transform:uppercase;color:var(--dim)}}
.lbl{{font-weight:600}}
dl{{margin:0 0 12px;display:grid;grid-template-columns:1fr 1fr;gap:6px 14px}}
dl>div{{display:flex;justify-content:space-between;gap:10px;
 border-bottom:1px dotted var(--rule);padding-bottom:4px}}
dt{{color:var(--dim);font-size:11px;letter-spacing:.09em;text-transform:uppercase}}
dd{{margin:0;font-variant-numeric:tabular-nums;font-weight:600}}
.con{{width:100%;height:auto;display:block}}
.grid{{stroke:var(--rule);stroke-width:1}}
.axis{{stroke:var(--dim);stroke-width:1;opacity:.6}}
.tick{{fill:var(--dim);font-size:9px;text-anchor:middle;font-family:inherit}}
.tick.ry{{text-anchor:start}}
.pts{{fill:none;stroke:var(--pt);stroke-linecap:round;opacity:.45}}
.pts.bad{{stroke:var(--bad);opacity:.3}}
.verdict{{margin:10px 0 0;font-size:12px;font-weight:600}}
.verdict.ok{{color:var(--ok)}} .verdict.warn{{color:var(--warn)}} .verdict.bad{{color:var(--bad)}}
.note{{margin:6px 0 0;font-size:12px;color:var(--dim)}}
.empty{{color:var(--dim);padding:40px 0;text-align:center}}
.wide{{margin-bottom:18px}}
.trace{{width:100%;height:auto;display:block}}
.tr{{fill:none;stroke:var(--accent);stroke-width:1.4}}
.white-line{{stroke:var(--bad);stroke-width:1;stroke-dasharray:4 3;opacity:.8}}
.wlbl{{fill:var(--bad);font-size:9px;text-anchor:end}}
.axlbl{{fill:var(--dim);font-size:9px}}
.legend{{margin-top:20px;font-size:12px;color:var(--dim);background:var(--card);
 border:1px solid var(--rule);border-radius:10px;padding:14px 18px}}
.legend b{{color:var(--ink)}}
</style>
<div class="wrap">
<h1>{title}</h1>
<p class="sub">{subtitle}</p>
{trace}<div class="grid2">{cells}</div>
<div class="legend">
<b>CINR</b> is carrier-to-interference-plus-noise measured against the lattice
itself: mean symbol power over mean square distance to the nearest odd-integer
point. It needs no knowledge of which constellation is in use, because ITU-T
V.34 clause 9 puts every point of every rate on odd integers.
<b>TO LATTICE</b> is that distance on its own — <b>0.667 is the value for
symbols bearing no relation to the lattice</b>, so a reading near it means the
output is white, not merely noisy. <b>POWER</b> is the G.711 tap level referred
to 0 dBm0, which for these decoders is RMS {ref:.0f}.
</div>
</div>""".format(title=title, subtitle=subtitle, cells="".join(cells),
                 trace=('<section class="panel wide"><div class="hdr">'
                        '<span class="role">Eye over the call</span>'
                        '<span class="lbl">%s</span></div>%s</section>'
                        % (trace_note, trace)) if trace else "",
                 ref=DBM0_RMS["ulaw"])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir", help="artifact directory from a soak or rate call")
    ap.add_argument("-o", "--out", default=None)
    ap.add_argument("--symbols", type=int, default=20000)
    ap.add_argument("--from", dest="start", type=int, default=0)
    ap.add_argument("--alaw", action="store_true")
    ap.add_argument("--title", default=None)
    a = ap.parse_args()
    law = "alaw" if a.alaw else "ulaw"
    d = a.dir

    def p(*n):
        return os.path.join(d, *n)

    panels = []
    rx = read_pairs(p("frames.answer"), a.start, a.symbols)
    src = "V34_DATA_FRAME_DUMP"
    if not rx:
        for cand in ("sym.txt", "upstream_sym.txt"):
            rx = read_text_syms(p(cand), a.start, a.symbols)
            if rx:
                src = "ME_V90_UPSTREAM_SYM_DUMP"
                break
    tx = read_pairs(p("frames_tx.answer"), a.start, a.symbols)

    rx_rms = tap_rms(p("live-rx.g711"), law)
    tx_rms = tap_rms(p("live-tx.g711"), law)

    panels.append(Panel("Local — receive", "what the peer sent us", rx,
                        rx_rms, law, "source: %s" % src))
    if tx:
        panels.append(Panel("Local — transmit", "what we sent", tx,
                            tx_rms, law, "source: V34_DATA_TX_DUMP"))
    else:
        panels.append(Panel("Local — transmit", "what we sent", [], tx_rms, law,
                            "no V34_DATA_TX_DUMP in this recording; the power "
                            "figure is measured from the transmit tap."))

    out = a.out or p("constellation.html")
    title = a.title or "Constellation diagnostics"
    sub = "%s — %s symbols from index %s" % (
        os.path.basename(os.path.normpath(d)),
        "{:,}".format(len(rx)), "{:,}".format(a.start))
    baud = 3200.0
    tr = trace_svg(rx, baud, )
    open(out, "w").write(render(
        panels, title, sub, tr,
        "one point per 2048 symbols, at %d baud" % baud))
    print(out)
    for pn in panels:
        print("  %-18s CINR %s  power %s  to-lattice %s  %s"
              % (pn.role, fmt(pn.cinr, "dB", 1), fmt(pn.dbm0, "dBm0", 1),
                 "—" if pn.err is None else "%.4f" % pn.err, pn.verdict()))


if __name__ == "__main__":
    main()
