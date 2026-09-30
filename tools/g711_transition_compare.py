#!/usr/bin/env python3
"""Compare raw PCMU tap power/spectra and held-out TX echo across a transition.

No live receiver state, demodulator, AGC or equalizer is used. Positive offset
means rx[n] corresponds to tx[n-offset]. Physical RTT and file offset differ
by unknown tap origin skew. NumPy only. Outputs JSON and CSV for reproduction.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from numpy.lib.stride_tricks import sliding_window_view
from v90_phase4_upstream_grade import FS, ulaw_lin


def spectrum(x, size=512):
    frames = sliding_window_view(x, size)[::size//2]
    window = np.hanning(size)
    p = np.mean(abs(np.fft.rfft(frames * window, axis=1))**2, axis=0)
    p *= 2 / (FS * np.sum(window**2))
    p[[0, -1]] *= .5
    return np.fft.rfftfreq(size, 1/FS), p


def metrics(x):
    f, p = spectrum(x)
    centroid = np.sum(f*p)/np.sum(p)
    return dict(rms=float(np.sqrt(np.mean(x*x))),
                power=float(np.mean(x*x)), centroid_hz=float(centroid),
                rms_bandwidth_hz=float(np.sqrt(np.sum((f-centroid)**2*p)/sum(p))),
                band_power={f'{lo}-{hi}': float(sum(p[(f>=lo)&(f<hi)])*FS/512)
                            for lo, hi in [(0,600),(600,1200),(1200,2400),(2400,3400),(3400,4001)]})


def matrix(tx, a, b, offset, taps):
    half = taps//2
    lo = a-offset-half
    if lo < 0 or b-offset+half > len(tx):
        raise ValueError('aligned TX window outside recording')
    return sliding_window_view(tx, taps)[lo:lo+b-a]


def fit(rx, tx, a, b, offset, taps):
    X = matrix(tx,a,b,offset,taps)
    mid = (b-a)//2
    h = np.linalg.lstsq(X[:mid],rx[a:a+mid],rcond=1e-7)[0]
    residual = rx[a:b]-X@h
    removal = lambda sl: float(1-np.mean(residual[sl]**2)/np.mean(rx[a:b][sl]**2))
    return dict(train_removed=removal(slice(0,mid)), heldout_removed=removal(slice(mid,None))), h


def search(rx, tx, a, b, low, high):
    # Select delay on training half only; validation never selects the delay.
    y=rx[a:(a+b)//2]; y=y-y.mean()
    scores=[]
    for lag in range(low,high+1):
        x=tx[a-lag:a-lag+len(y)]; x=x-x.mean()
        scores.append(float(x@y/np.sqrt((x@x)*(y@y))))
    i=int(np.argmax(np.abs(scores)))
    return low+i, scores[i]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('rx'); p.add_argument('tx')
    p.add_argument('--before',nargs=2,type=float,default=[23.05,23.25])
    p.add_argument('--after',nargs=2,type=float,default=[23.5,25.2])
    p.add_argument('--rtt-ms',type=float,default=187)
    p.add_argument('--offset-search-ms',nargs=2,type=float,default=[80,220])
    p.add_argument('--taps',type=int,default=129)
    p.add_argument('--timeline',nargs=2,type=float,default=[23.0,23.7])
    p.add_argument('--output',required=True)
    args=p.parse_args()
    rx=ulaw_lin(np.fromfile(args.rx,dtype=np.uint8)); tx=ulaw_lin(np.fromfile(args.tx,dtype=np.uint8))
    if not 0 <= args.timeline[0] < args.timeline[1] <= len(rx)/FS: p.error('timeline outside RX')
    if args.taps<1 or args.taps%2!=1: p.error('taps must be positive and odd')
    windows={k:tuple(round(v*FS) for v in w) for k,w in [('before',args.before),('after',args.after)]}
    for a,b in windows.values():
        if not 0<=a<b<=len(rx) or b-a<max(1024,8*args.taps): p.error('window too short or outside RX')
    a,b=windows['after']; low,high=[round(v*FS/1000) for v in args.offset_search_ms]
    if not 0 <= low <= high < a or b-low > len(tx): p.error('offset search outside TX')
    lag,rho=search(rx,tx,a,b,low,high)
    rtt=round(args.rtt_ms*FS/1000)
    report=dict(rx=args.rx,tx=args.tx,sample_rate=FS,physical_rtt_ms=args.rtt_ms,
                selected_file_offset_samples=lag,selected_file_offset_ms=lag/FS*1000,
                implied_origin_skew_ms=lag/FS*1000-args.rtt_ms,training_correlation=rho,windows={})
    spectra=[]
    for name,(a,b) in windows.items():
        data=metrics(rx[a:b]); data['rx_seconds']=[a/FS,b/FS]
        data['aligned_tx_seconds']=[(a-lag)/FS,(b-lag)/FS]
        data['tx']=metrics(tx[a-lag:b-lag])
        data['echo']={}
        for label,delay in [('selected',lag),('rtt_as_file_offset',rtt),('wrong_delay_control',lag+4000)]:
            score,h=fit(rx,tx,a,b,delay,args.taps)
            data['echo'][label]=score
            if label=='selected':
                X=matrix(tx,a,b,lag,args.taps); residual=rx[a:b]-X@h
                data['residual']=metrics(residual)
                mid=(b-a)//2
                data['heldout_raw']=metrics(rx[a+mid:b])
                data['heldout_residual']=metrics(residual[mid:])
                f,raw=spectrum(rx[a:b]); _,clean=spectrum(residual); _,reference=spectrum(tx[a-lag:b-lag])
                spectra.append((name,f,raw,clean,reference))
        report['windows'][name]=data
    report['rx_power_change_db']=float(10*np.log10(report['windows']['after']['power']/report['windows']['before']['power']))
    base=Path(args.output); base.parent.mkdir(parents=True,exist_ok=True)
    base.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n')
    with base.with_suffix('.csv').open('w') as out:
        out.write('window,frequency_hz,rx_psd,residual_psd,aligned_tx_psd\n')
        for name,f,raw,clean,reference in spectra:
            for row in zip(f,raw,clean,reference): out.write(name+','+','.join(map(str,row))+'\n')
    # Short blocks locate the change without any protocol-stage counters.
    with base.with_name(base.name+'-timeline').with_suffix('.csv').open('w') as out:
        out.write('rx_seconds,rx_rms,aligned_tx_rms\n')
        for a in range(round(args.timeline[0]*FS),round(args.timeline[1]*FS),160):
            out.write(f'{a/FS:.3f},{np.sqrt(np.mean(rx[a:a+160]**2)):.6f},{np.sqrt(np.mean(tx[a-lag:a-lag+160]**2)):.6f}\n')
    print(json.dumps(report,indent=2))

if __name__=='__main__': main()
