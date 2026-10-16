#!/usr/bin/env python3
"""Compare training-only FIR responses of native-rate Si3056 §5.30 probes."""
import argparse,json
from pathlib import Path
import numpy as np
from scipy import signal
from apple_loopback_spectrum import fit
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def analyze(d):
    meta=json.loads((d/'result.json').read_text());fs=meta['rate_hz'];lag=meta['lag_samples']
    assert meta['restored']==1 and meta['exit_code']==0
    assert f'read back {fs} Hz' in (d/'log.txt').read_text()
    tx=np.fromfile(d/'tx.s16',dtype='<i2').astype(float)
    rx=np.fromfile(d/'rx.s16',dtype='<i2').astype(float)
    ix=np.arange(fs+300,int(2.45*fs)-300)
    train=np.arange(int(.60*fs));test=np.arange(int(.75*fs),len(ix))
    half=256
    assert ix[test[0]]-half > ix[train[-1]]+half
    assert ix[-1]+lag < len(rx)
    features=tx[ix[:,None]+np.arange(-half,half+1)]
    score,pred,coef,bias=fit(features,rx[ix+lag],train,test)
    hz,h=signal.freqz(coef[::-1],worN=8192,fs=fs)
    h*=np.exp(2j*np.pi*hz/fs*half)
    gain=20*np.log10(np.maximum(abs(h),1e-15));gain-=np.interp(1000,hz,gain)
    cutoffs={}
    for db in [3,6,10,20]:
        hits=np.flatnonzero((hz>1000)&(gain<=-db))
        cutoffs[str(db)]=float(hz[hits[0]]) if len(hits) else None
    f,power=signal.welch(rx[ix[test]+lag],fs,nperseg=1024)
    _,residual=signal.welch((rx[ix+lag]-pred)[test],fs,nperseg=1024)
    quiet=rx[int(.3*fs):int(.8*fs)]
    assert np.all(tx[int(.3*fs)-lag-256:int(.8*fs)-lag+256]==0)
    _,noise=signal.welch(quiet,fs,nperseg=1024)
    band=(f>=300)&(f<=3400)
    ratio=lambda p:float(10*np.log10(np.sum(power[band])/np.sum(p[band]))) if np.sum(p[band])>0 else None
    result=dict(directory=str(d),device=meta['device'],mode=meta['mode'],rate_hz=fs,
        cutoff_hz=cutoffs,forward=score,band_noise_snr_db=ratio(noise),band_residual_snr_db=ratio(residual),
        relative_gain_db={str(v):float(np.interp(v,hz,gain)) if v<fs/2 else None for v in [3000,3400,3600,3800,4000,4200,4400,4600,4800]},
        raw_sample_errors=meta['sample_errors'],lag_ms=meta['lag_ms'])
    np.savetxt(d/'response.csv',np.column_stack([hz,gain]),delimiter=',',header='frequency_hz,gain_relative_to_1khz_db',comments='')
    return result,hz,gain


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path,nargs='+');args=p.parse_args()
    fig,axes=plt.subplots(1,2,figsize=(13,5),layout='constrained');rows=[]
    for d in sorted(d for root in args.directory for d in root.iterdir()):
        if not (d/'result.json').exists():continue
        meta=json.loads((d/'result.json').read_text())
        if meta['exit_code'] != 0:
            print('Excluded failed probe:',d,flush=True);continue
        row,hz,gain=analyze(d);rows.append(row)
        if row['mode']=='digital':continue
        ax=axes[0 if row['device']=='0:13' else 1]
        ax.plot(hz,gain,color={8000:'#c44e52',9600:'#4c72b0',10286:'#55a868'}[row['rate_hz']],label=f"{row['rate_hz']} Hz {row['mode']}",ls='--' if row['mode']=='isolation' else '-')
    for addr,ax in zip(['0:13','0:14'],axes):
        ax.set(title='Modem '+addr,xlim=(2500,5100),ylim=(-40,2),xlabel='Frequency (Hz)',ylabel='Gain relative to 1 kHz (dB)')
        ax.axhline(-3,color='gray',lw=.7);ax.grid(alpha=.2);ax.legend(fontsize=9)
    fig.suptitle('Measured native-rate loopbacks: does the bandwidth scale?')
    fig.savefig(args.directory[0]/'native-response.png',dpi=160)
    (args.directory[0]/'summary.json').write_text(json.dumps(rows,indent=2)+'\n')
    for row in rows:print(row['device'],row['rate_hz'],row['mode'],row['cutoff_hz'],flush=True)


if __name__=='__main__':main()
