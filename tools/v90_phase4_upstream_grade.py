#!/usr/bin/env python3
"""Grade the peer's upstream Phase 4 signal (CPt / SCR / CP) OUTSIDE our receiver.

Our own Phase 4 receiver's dibits go white immediately after the far-end CPt,
so anything read through it measures that defect rather than the peer.  This
demodulates a recorded RX tap directly:

  analytic signal -> mix to baseband at V.34 5.1's fc = S*d/e -> exact
  band-limited resample to T/2 -> 41-tap fractionally-spaced equalizer adapted
  blind by CMA (every Phase 4 upstream signal here is constant modulus, so CMA
  is legitimate throughout) -> differential 4-point decode per 10.1.3.3 ->
  the analogue modem's GPA descrambler.

SCR is scrambled binary ones, so a correctly demodulated SCR descrambles to
ONES.  CP and CPt are structured data and do not.

TRAPS, both of which produced wrong answers before being caught:

  * A CONSTANT dibit descrambles to ones under EVERY tap pair, because
    1^1^1 = 1.  A pure tone therefore scores a perfect "100% ones" and looks
    exactly like a flawless SCR.  Always print the dibit histogram: real data
    is 25/25/25/25, a tone is 100/0/0/0.  This is the same trap the V.34 Phase
    4 TRN ones-lock carries (docs/v34_plain_phase2_call_role.md).
  * Distance-to-the-90-degree-grid reads ~26 deg for a signal with no relation
    to the constellation at all -- that is uniform over +/-45 deg, not "noisy".
    Quote the tail (fraction beyond 22.5 deg) with it.

  v90_phase4_upstream_grade.py <live-rx.g711> <t0> <t1> [baud] [carrier]

With no window, sweeps the whole tap in 0.5 s blocks and reports RMS, spectral
centroid and RMS bandwidth, which is how the Phase 4 window is located
(3200 baud reads ~945 Hz of RMS bandwidth; a flat +/-1600 Hz band gives 924).
"""
import sys, math
import numpy as np

FS = 8000.0

def ulaw_lin(raw):
    u = (~raw) & 0xFF
    t = ((((u & 0x0F).astype(np.int32) << 1) + 33) << ((u & 0x70) >> 4)) - 33
    return np.where((u & 0x80) != 0, -t, t).astype(np.float64) * 4.0

def analytic(v):
    X = np.fft.fft(v); n = len(v); h = np.zeros(n); h[0] = 1
    if n % 2 == 0: h[n//2] = 1; h[1:n//2] = 2
    else:          h[1:(n+1)//2] = 2
    return np.fft.ifft(X * h)

def fft_resample(z, num):
    n = len(z); Z = np.fft.fft(z); h = num // 2
    return np.fft.ifft(np.concatenate([Z[:h], Z[n-(num-h):]])) * (num / n)

def demod(x, t0, t1, baud, fc, ntaps=41, mu=3e-3, skip=1200):
    seg = x[int(t0*FS):int(t1*FS)]
    z = analytic(seg); n = np.arange(len(z))
    z = z * np.exp(-2j*math.pi*fc*n/FS)
    u = fft_resample(z, int(round(len(z)*(2*baud)/FS)))
    u = u / (np.sqrt(np.mean(np.abs(u)**2)) + 1e-12)
    w = np.zeros(ntaps, dtype=complex); w[ntaps//2] = 1.0
    out = []
    for k in range((len(u)-ntaps)//2):
        v = u[k*2:k*2+ntaps][::-1]
        y = np.dot(w, v); out.append(y)
        w -= mu * (y*(abs(y)**2 - 1.0)) * np.conj(v)
    return np.array(out)[skip:]

def survey(x):
    N = 4000
    print(" t(s)   RMS   centroid(Hz)  rmsbw(Hz)")
    for s in range(0, len(x)-N, N):
        b = x[s:s+N]; r = math.sqrt(np.mean(b*b))
        if r < 50:
            print(" %5.1f %6.0f   (quiet)" % (s/FS, r)); continue
        X = np.abs(np.fft.rfft(b*np.hanning(N)))**2
        f = np.fft.rfftfreq(N, 1/FS); m = (f > 100) & (f < 3800)
        P = X[m]; ff = f[m]
        c = np.sum(ff*P)/np.sum(P)
        bw = math.sqrt(np.sum(((ff-c)**2)*P)/np.sum(P))
        print(" %5.1f %6.0f   %7.1f    %6.1f" % (s/FS, r, c, bw))

def grade(x, t0, t1, baud, fc):
    s = demod(x, t0, t1, baud, fc)
    d = s[1:] * np.conj(s[:-1]); a = np.angle(d)
    q = np.rint(a/(math.pi/2)).astype(int) % 4
    err = np.degrees(a - np.rint(a/(math.pi/2))*(math.pi/2))
    h = np.bincount(q, minlength=4)/len(q)*100
    sd = math.sqrt(np.mean(err**2))
    print("window %.2f-%.2f s, baud %.0f, fc %.3f Hz, %d symbols"
          % (t0, t1, baud, fc, len(s)))
    print("  |z| dispersion   %.3f" % (np.std(np.abs(s))/np.mean(np.abs(s))))
    print("  dist to 90deg grid %.2f deg (26 deg = uniform, i.e. NO 4-point signal)" % sd)
    print("  implied SNR      %.1f dB" % (10*math.log10(1/(sd*sd/ (180/math.pi)**2 ))))
    print("  tail  >11.25deg %.2f%%   >22.5deg %.2f%%   (>45deg would be a dibit error)"
          % (100*np.mean(abs(err) > 11.25), 100*np.mean(abs(err) > 22.5)))
    print("  dibit histogram  %.1f %.1f %.1f %.1f   <-- 25 each = data, 100/0/0/0 = A TONE"
          % tuple(h))
    best = []
    for rot in range(4):
        qq = (q + rot) % 4
        for order in (0, 1):
            b = np.empty(2*len(qq), dtype=np.int8)
            if order == 0: b[0::2] = qq & 1;      b[1::2] = (qq >> 1) & 1
            else:          b[0::2] = (qq >> 1)&1; b[1::2] = qq & 1
            for t_1 in range(1, 32):
                for t_2 in range(t_1+1, 33):
                    o = b[t_2:] ^ b[t_2-t_1:-t_1] ^ b[:-t_2]
                    best.append((o.mean(), t_1, t_2, rot, order))
    best.sort(reverse=True)
    print("  descrambler sweep (out[n]=in[n]^in[n-t1]^in[n-t2]); GPA is (5,23):")
    for v, t_1, t_2, r, o in best[:4]:
        tag = "  <- GPA" if (t_1, t_2) == (5, 23) else ""
        print("     t=(%2d,%2d) rot%d ord%d -> %5.1f%% ones%s" % (t_1, t_2, r, o, 100*v, tag))

def main():
    x = ulaw_lin(np.frombuffer(open(sys.argv[1], 'rb').read(), dtype=np.uint8))
    print("%s: %d samples, %.1f s" % (sys.argv[1], len(x), len(x)/FS))
    if len(sys.argv) < 4:
        survey(x); return
    t0 = float(sys.argv[2]); t1 = float(sys.argv[3])
    baud = float(sys.argv[4]) if len(sys.argv) > 4 else 3200.0
    fc = float(sys.argv[5]) if len(sys.argv) > 5 else baud*8/14
    grade(x, t0, t1, baud, fc)

main()
