#!/usr/bin/env python3
"""Regenerate V.90 TRN2d from the Recommendation text alone and compare it,
sign by sign, with what our transmitter actually put on the wire.

tools/v90_trn2d_spec_decode.py proves our TRN2d carries the right DATA, but
it removes the spectral shaper's choice without checking it -- and 5.4.5's
sign-inversion rules are invisible to a data decode.  8.6.5 makes TRN2d fully
deterministic (scrambler, differential encoder and shape filter memory all
zero at its start, metric defined in 5.4.5.6), so an analogue modem can
regenerate it exactly and train on it data-aided.  If our shaper's choices
differ from the Recommendation's, the data still decodes but the WAVEFORM is
not the one the far end expects.  This checks the waveform.

Written from V.90 (09/98) alone; imports nothing from the tree:
  5.3    GPC scrambler, d[n] = in[n] ^ d[n-18] ^ d[n-23], zero state (8.6.5)
  5.4.2  first S bits of each D-bit frame are sign inputs s0.., then b0..bK-1
  5.4.3  R0 = sum b_i 2^i; K_i = R_i mod M_i, interval 0 first
  5.4.4  labels descending: label 0 = largest Ucode in C_i
  5.4.5.2 Tables 3 and 4 (Sr = 1), then t_j(k) = p'_j(k) ^ t_j-1(k)
  5.4.5.5 Figure 2 trellis: state 0 allows A (-> 0) and B (-> 1),
         state 1 allows C (-> 0) and D (-> 1) [--trellis alt swaps C/D];
         rule chosen to minimise w through the end of frame j+ld
  5.4.5.6 y = x - b1 x[-1] + a1 y[-1]; v = y - b2 y[-1] + a2 v[-1]; w += v^2,
         x proportional to the linear value of the TRANSMITTED PCM code
  5.4.6  sign bit 1 = positive

  v90_trn2d_spec_generate.py <live-tx.g711> <ucode,ucode,..> <sr> <ld>
        <a1> <a2> <b1> <b2> [--trn N] [--start SAMPLE] [--uinfo U]
        [--trellis std|alt] [--metric tx|codec:<ucodes>]

Only Sr = 1 with one constellation for all six intervals is handled (that is
what the RasFinder's CPt asks for).  The TRN2d start is found as the sample
after Ri's four barred repetitions unless --start is given.
"""
import argparse
import itertools


def ulaw_mag(u):
    """G.711 mu-law magnitude for Ucode u (V.90 Table 1, up to scale)."""
    return ((2 * (u & 15) + 33) << (u >> 4)) - 33


def find_trn2d_start(ucode, sign, uinfo):
    """First sample after R-bar-i: Ri is U_INFO with +++--- signs, and 8.6.4's
    R-bar-i is four repetitions of ---+++."""
    n = len(ucode)
    pat = [1, 1, 1, 0, 0, 0]
    bar = [0, 0, 0, 1, 1, 1]
    i = 0
    while i + 6 * 40 < n:
        if all(ucode[i + k] == uinfo for k in range(6 * 8)) and \
           all(sign[i + 6 * r + k] == pat[k] for r in range(8) for k in range(6)):
            j = i
            while j + 6 <= n and all(ucode[j + k] == uinfo for k in range(6)) \
                    and [sign[j + k] for k in range(6)] == pat:
                j += 6
            if all(ucode[j + 6 * r + k] == uinfo and sign[j + 6 * r + k] == bar[k]
                   for r in range(4) for k in range(6)):
                return j + 24, (j - i) // 6
            i = j + 1
        else:
            i += 1
    return None, 0


class Shaper:
    """5.4.5.5/5.4.5.6 with the filter state carried explicitly."""

    RULES = {0: (('A', 0), ('B', 1)), 1: (('C', 0), ('D', 1))}

    def __init__(self, a1, a2, b1, b2, alt=False):
        self.a1, self.a2, self.b1, self.b2 = a1, a2, b1, b2
        if alt:
            self.RULES = {0: (('A', 0), ('B', 1)), 1: (('D', 0), ('C', 1))}
        self.state = 0
        self.f = (0.0, 0.0, 0.0)          # x[n-1], y[n-1], v[n-1]

    @staticmethod
    def apply(rule, t):
        if rule == 'A':
            return list(t)
        if rule == 'B':
            return [b ^ 1 for b in t]
        if rule == 'C':
            return [b ^ (1 if k % 2 == 0 else 0) for k, b in enumerate(t)]
        return [b ^ (1 if k % 2 == 1 else 0) for k, b in enumerate(t)]

    def run(self, f, signs, mags):
        x1, y1, v1 = f
        w = 0.0
        for s, m in zip(signs, mags):
            x = m if s else -m
            y = x - self.b1 * x1 + self.a1 * y1
            v = y - self.b2 * y1 + self.a2 * v1
            w += v * v
            x1, y1, v1 = x, y, v
        return w, (x1, y1, v1)

    def choose(self, frames_t, frames_mag):
        """frames_t/mag: frame j and the ld following frames.  Returns the
        signs for frame j, the rule, and whether the minimum was a tie."""
        best = []
        def walk(depth, state, f, w, first):
            if depth == len(frames_t):
                best.append((w, first))
                return
            for rule, nxt in self.RULES[state]:
                sg = self.apply(rule, frames_t[depth])
                dw, nf = self.run(f, sg, frames_mag[depth])
                walk(depth + 1, nxt, nf, w + dw, first or (rule, nxt, sg))
        walk(0, self.state, self.f, 0.0, None)
        best.sort(key=lambda e: e[0])
        w0, (rule, nxt, sg) = best[0]
        tie = len(best) > 1 and abs(best[1][0] - w0) <= 1e-9 * max(1.0, w0) \
            and best[1][1][0] != rule
        self.state = nxt
        _, self.f = self.run(self.f, sg, frames_mag[0])
        return sg, rule, tie


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('tap')
    ap.add_argument('mask')
    ap.add_argument('sr', type=int)
    ap.add_argument('ld', type=int)
    ap.add_argument('a1', type=float)
    ap.add_argument('a2', type=float)
    ap.add_argument('b1', type=float)
    ap.add_argument('b2', type=float)
    ap.add_argument('--trn', type=int, default=3996)
    ap.add_argument('--start', type=int)
    ap.add_argument('--uinfo', type=int, default=78)
    ap.add_argument('--trellis', choices=('std', 'alt'), default='std')
    ap.add_argument('--metric', default='tx')
    a = ap.parse_args()
    if a.sr != 1:
        raise SystemExit('only Sr = 1 is handled')

    raw = open(a.tap, 'rb').read()
    ucode = [0x7F - (c & 0x7F) for c in raw]
    sign = [1 if (c & 0x80) else 0 for c in raw]
    mask = sorted((int(u) for u in a.mask.split(',')), reverse=True)
    m = len(mask)
    k_bits = (m.bit_length() - 1) * 6
    if 1 << (k_bits // 6) != m:
        raise SystemExit('power-of-two constellation expected')
    s_bits = 6 - a.sr
    d_bits = k_bits + s_bits
    metric_mask = mask
    if a.metric.startswith('codec:'):
        metric_mask = sorted((int(u) for u in a.metric[6:].split(',')), reverse=True)

    start = a.start
    if start is None:
        start, ri = find_trn2d_start(ucode, sign, a.uinfo)
        if start is None:
            raise SystemExit('Ri / R-bar-i not found; pass --start')
        print('Ri %d reps, TRN2d starts at tap sample %d (%.4f s)' % (ri, start, start / 8000))
    frames = a.trn // 6

    # 5.3 scrambler over binary ones, zero state
    nbits = (frames + a.ld + 1) * d_bits
    d = []
    for n in range(nbits):
        b = 1 ^ (d[n - 18] if n >= 18 else 0) ^ (d[n - 23] if n >= 23 else 0)
        d.append(b)

    # 5.4.2-5.4.4 magnitudes, and the initial shaping signs t_j
    mags, ts, labels = [], [], []
    pprev5 = 0
    tprev = [0] * 6
    for j in range(frames + a.ld + 1):
        fr = d[j * d_bits:(j + 1) * d_bits]
        s = fr[:s_bits]
        b = fr[s_bits:]
        r = sum(bit << i for i, bit in enumerate(b))
        lab = []
        for i in range(6):
            lab.append(r % m)
            r //= m
        labels.append(lab)
        mags.append([ulaw_mag(metric_mask[k]) for k in lab])
        p = [0] + s                                   # Table 3, Sr = 1
        pp = [0] * 6                                  # Table 4
        pp[1] = p[1] ^ pprev5
        pp[2] = p[2]
        pp[3] = p[3] ^ pp[1]
        pp[4] = p[4]
        pp[5] = p[5] ^ pp[3]
        pprev5 = pp[5]
        t = [pp[k] ^ tprev[k] for k in range(6)]
        tprev = t
        ts.append(t)

    sh = Shaper(a.a1, a.a2, a.b1, a.b2, alt=(a.trellis == 'alt'))
    mag_err = sign_err = ties = 0
    first_mag = first_sign = None
    rules = {}
    for j in range(frames):
        sg, rule, tie = sh.choose(ts[j:j + a.ld + 1], mags[j:j + a.ld + 1])
        rules[rule] = rules.get(rule, 0) + 1
        ties += tie
        base = start + 6 * j
        for i in range(6):
            want_u = mask[labels[j][i]]
            if ucode[base + i] != want_u:
                mag_err += 1
                if first_mag is None:
                    first_mag = (j, i, ucode[base + i], want_u)
            if sign[base + i] != sg[i]:
                sign_err += 1
                if first_sign is None:
                    first_sign = (j, i, rule, tie)
    total = frames * 6
    print('TRN2d %d frames (%d symbols), D=%d K=%d S=%d, trellis %s, metric %s'
          % (frames, total, d_bits, k_bits, s_bits, a.trellis, a.metric))
    print('magnitudes: %d of %d differ%s' % (mag_err, total,
          '' if first_mag is None else '; first at frame %d interval %d: tap U%d, spec U%d' % first_mag))
    print('signs:      %d of %d differ%s' % (sign_err, total,
          '' if first_sign is None else '; first at frame %d interval %d (spec rule %s%s)'
          % (first_sign[0], first_sign[1], first_sign[2], ', a TIE' if first_sign[3] else '')))
    print('spec rules chosen: %s; metric ties: %d' % (rules, ties))


if __name__ == '__main__':
    main()
