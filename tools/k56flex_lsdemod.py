#!/usr/bin/env python3
"""Least-squares symbol demodulator for recorded V.34-family upstreams.

Recovers the complex symbols of a recorded upstream (a K56flex client's S,
PP, TRN, report, J', TRN2, MP) on our own modulator's symbol grid, so another
client's upstream can be compared with ours symbol by symbol, or re-modulated
with some of its symbols replaced and replayed into MicaEmu's mica_trace.

The model is linear: our modulator (k56flex_modulate, SpanDSP's V.34
transmitter, as k56flex_client_splice uses it) is driven with unit impulses
to get the waveform of one symbol for each phase within a 3- or 4-symbol
period; every other symbol is a shift of those by whole periods, rotated by
the carrier's advance over a period.  demod() solves for the symbols of a
window; synth() rebuilds the waveform of a symbol vector.  The recording must
use the same symbol rate, carrier, power reduction and pre-emphasis; a
different modulator shows up as a fixed lag and rotation, found by fitting a
known stretch (PP or TRN).  Needs numpy and the k56flex_modulate binary.

    import k56flex_lsdemod as L
    m = L.Model(baud=4, high=0, pr=5)        # V.34 Table 1 code, INFO1a values
    x = L.ulaw_file('call.ulaw')[int(27.40*8000):]
    syms, resid = m.demod(x, 1000, 1400)     # symbols 1000..1399 of the grid
                                             # whose symbol 0 is at sample 0
Used for the 26 October MICA work in docs/k56flex_implementation.md.
"""
import os
import subprocess
import tempfile

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODULATOR = os.path.join(ROOT, 'k56flex_modulate')


def ulaw_file(path):
    """mu-law octets to linear samples (G.711)."""
    b = np.frombuffer(open(path, 'rb').read(), dtype=np.uint8)
    x = (~b) & 0xff
    e = (x >> 4) & 7
    v = (((x & 15).astype(np.int32) << 3) + 0x84 << e) - 0x84
    return np.where(x & 0x80, -v, v).astype(float)


def ulaw_encode(v):
    """Linear samples to mu-law octets (G.711)."""
    v = np.clip(np.round(v), -32124, 32124).astype(int)
    sign = (v < 0).astype(int)*0x80
    m = np.minimum(np.abs(v) + 0x84, 0x7fff)
    e = np.zeros_like(m)
    for ee in range(7, 0, -1):
        e = np.where((e == 0) & ((m & (0x4000 >> (7 - ee))) != 0), ee, e)
    return (~(sign | (e << 4) | ((m >> (e + 3)) & 15)) & 0xff).astype(np.uint8)


def _fractional_delay(v, f):
    n = len(v) + 64
    spec = np.fft.rfft(np.concatenate([np.zeros(32), v, np.zeros(32)]), n)
    w = np.fft.rfftfreq(n)
    return np.fft.irfft(spec*np.exp(-2j*np.pi*w*f), n)[32:32 + len(v)]


class Model:
    def __init__(self, baud=4, high=0, pr=0, pe=0):
        # Symbols per period and samples per period: 3000 baud is 3 symbols
        # in 8 samples, 3200 baud 4 in 10.
        self.P, self.L = {3: (3, 8), 4: (4, 10)}[baud]
        self.args = [str(baud), str(high), str(pr), str(pe)]
        self.k0 = 600
        with tempfile.TemporaryDirectory() as tmp:
            basis = {}
            for p in range(self.P):
                for comp, val in ((0, 1), (1, 1j)):
                    s = np.zeros(self.k0 + self.P + 10, complex)
                    s[self.k0 + p] = val
                    basis[(p, comp)] = self._modulate(s, 4000, tmp)
            s = np.zeros(self.k0 + self.P + 10, complex)
            s[self.k0 + self.P] = 1
            shifted = self._modulate(s, 4000, tmp)
        r, i = basis[(0, 0)], basis[(0, 1)]
        a = np.stack([r[:-self.L], i[:-self.L]], 1)
        (c, d), *_ = np.linalg.lstsq(a, shifted[self.L:], rcond=None)
        self.theta = np.arctan2(d, c)          # carrier advance per period
        nz = np.nonzero(sum(np.abs(v) for v in basis.values()) > 1e-3)[0]
        self.lo, self.hi = nz[0], nz[-1]
        self.kernels = {k: v[self.lo:self.hi + 1] for k, v in basis.items()}
        self._delayed = {}

    def _modulate(self, syms, n, tmp):
        f, o = os.path.join(tmp, 's.f32'), os.path.join(tmp, 'o.s16')
        np.asarray(syms, dtype=np.complex64).view(np.float32).tofile(f)
        subprocess.run([MODULATOR, f, o] + self.args + [str(n)], check=True)
        return np.fromfile(o, dtype=np.int16).astype(float)

    def matrix(self, k0, k1, lag=0.0):
        """Model matrix for symbols k0..k1-1, waveform delayed by lag samples."""
        whole = int(np.floor(lag))
        frac = lag - whole
        if frac not in self._delayed:
            self._delayed[frac] = {k: _fractional_delay(v, frac) for k, v in self.kernels.items()}
        kern = self._delayed[frac]
        starts = [self.lo + (k//self.P - self.k0//self.P)*self.L for k in range(k0, k1)]
        t0 = min(starts)
        a = np.zeros((max(starts) + (self.hi - self.lo) + 1 - t0, 2*(k1 - k0)))
        for j, k in enumerate(range(k0, k1)):
            period, phase = divmod(k, self.P)
            rot = np.exp(1j*self.theta*(period - self.k0//self.P))
            r, i = kern[(phase, 0)], kern[(phase, 1)]
            s = starts[j] - t0
            a[s:s + len(r), 2*j] = rot.real*r + rot.imag*i
            a[s:s + len(r), 2*j + 1] = -rot.imag*r + rot.real*i
        return a, t0 + whole

    def demod(self, x, k0, k1, lag=0.0):
        """Symbols k0..k1-1 of x and the relative residual (window edges included)."""
        a, t0 = self.matrix(k0, k1, lag)
        y = x[t0:t0 + a.shape[0]]
        sol, *_ = np.linalg.lstsq(a, y, rcond=None)
        return sol[0::2] + 1j*sol[1::2], np.linalg.norm(a @ sol - y)/np.linalg.norm(y)

    def synth(self, syms, k0, lag=0.0):
        """Waveform of syms placed at grid symbols k0.., and its first sample index."""
        a, t0 = self.matrix(k0, k0 + len(syms), lag)
        v = np.zeros(2*len(syms))
        v[0::2], v[1::2] = np.real(syms), np.imag(syms)
        return a @ v, t0
