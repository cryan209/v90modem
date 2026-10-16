#!/usr/bin/env python3
"""Supervised, band-limited channel-fit diagnostic for captured 9668 playback.

ITU-T V.90 §8.4.1 DIL playback is known source material, not negotiated data.
Fit a forward channel on training windows; score separate received samples.
No source-level decisions, constellation negotiation, or BER are claimed.
"""
import argparse,json
from pathlib import Path
import numpy as np
from scipy import signal,ndimage
from g711_codec import ulaw_to_linear
from apple_loopback_spectrum import fit
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
FS=10286


def band_power(x):
    n=min(1024,len(x));f,p=signal.welch(x,FS,nperseg=n,noverlap=n//2)
    return float(p[(f>=300)&(f<=3400)].sum()*(f[1]-f[0]))


def analyze(d):
    source=ulaw_to_linear(np.frombuffer((d/'usr-dil.ulaw').read_bytes(),dtype=np.uint8)).astype(float)
    raw=np.fromfile(d/'rx.s16',dtype='<i2').astype(float)
    reference=signal.resample_poly(source,5143,4000)
    sos=signal.butter(5,[300,3400],fs=FS,btype='bandpass',output='sos')
    reference=signal.sosfiltfilt(sos,np.pad(reference,(512,512)))
    received=signal.sosfiltfilt(sos,raw)
    # Interpolation itself must be more accurate than the measured residual.
    # Band-limited 64x reference, then linear interpolation, avoids the much
    # larger phase-dependent error of cubic interpolation near Nyquist.
    reference=signal.resample_poly(reference,64,1,window=('kaiser',9))
    ix=np.arange(1500,15500)
    # Separate windows across the decaying source's amplitude range. 500
    # sample guards exceed the longest 129-tap feature span.
    phase=np.arange(len(ix))%3000
    train=np.flatnonzero(phase<1000)
    test=np.flatnonzero((phase>=1500)&(phase<2500))
    def features(lag,ppm,taps):
        pos=(ix[:,None]+lag+np.arange(-(taps//2),taps//2+1))*(1+ppm*1e-6)+512
        return ndimage.map_coordinates(reference,[64*pos.ravel()],order=1,prefilter=False,mode='constant').reshape(pos.shape)
    rows=[];illustrations=[]
    for j,b in enumerate(json.loads((d/'summary.json').read_text())['bursts']):
        start=round(b['start_s']*FS);target=received[start+ix]
        best=None
        # Timing and filter choices use training error only. Broad alignment
        # is essential because a pulse train has misleading correlation lobes.
        for lag in range(-400,401,10):
            _,pred,_,_=fit(features(lag,190,33),target,train,test)
            error=float(np.mean((target[train]-pred[train])**2))
            if best is None or error<best[0]:best=(error,lag)
        assert abs(best[1])<400, 'Alignment optimum at search edge'
        lag=best[1];best=None
        for ppm in range(175,206):
            _,pred,_,_=fit(features(lag,ppm,65),target,train,test)
            error=float(np.mean((target[train]-pred[train])**2))
            if best is None or error<best[0]:best=(error,ppm)
        assert best[1] not in (175,205), 'Rate optimum at search edge'
        ppm=best[1];ladder={}
        for taps in [17,33,65,129]:
            score,pred,_,_=fit(features(lag,ppm,taps),target,train,test)
            # PSD is computed on each contiguous held-out window separately;
            # disjoint blocks are never spliced into a synthetic audio stream.
            blocks=[]
            for cycle in range(5):
                block=np.flatnonzero((np.arange(len(ix))//3000==cycle)&(phase>=1500)&(phase<2500))
                if len(block)>=1000:blocks.append(block)
            rp=np.mean([band_power((target-pred)[block]) for block in blocks])
            sp=np.mean([band_power(target[block]) for block in blocks])
            ladder[str(taps)]=dict(score,band_residual_rms=float(np.sqrt(rp)),band_signal_rms=float(np.sqrt(sp)),band_signal_residual_db=float(10*np.log10(sp/rp)))
            if taps==65:chosen=pred.copy()
        # Negative control: unrelated reference with the same dimensions.
        rng=np.random.default_rng(9668+j)
        unrelated=rng.normal(size=(len(ix),65))
        control,_,_,_=fit(unrelated,target,train,test)
        quiet=raw[start+int(2.15*FS):start+int(2.45*FS)]
        row=dict(quiet_band_noise_rms=float(np.sqrt(band_power(quiet))),playback=j+1,coarse_start_s=b['start_s'],source_offset_samples=lag,reference_rate_adjustment_ppm=ppm,
            train_samples=len(train),test_samples=len(test),native_window_samples=[1500,15500],
            timing_selected_on='training only',tap_ladder=ladder,unrelated_reference_snr_db=control['snr'])
        rows.append(row);illustrations.append((ix/FS,target,chosen))
        print(j+1,lag,ppm,ladder['65'],flush=True)
    (d/'quality.json').write_text(json.dumps(rows,indent=2)+'\n')
    fig,axes=plt.subplots(1,2,figsize=(12,4.7),layout='constrained')
    for r in rows:
        axes[0].plot([int(t) for t in r['tap_ladder']],[v['band_signal_residual_db'] for v in r['tap_ladder'].values()],marker='o',label=str(r['playback']))
    axes[0].set(title='Independent held-out channel-fit score',xlabel='Forward FIR taps',ylabel='300–3400 Hz signal / residual (dB)');axes[0].legend(title='Playback',fontsize=8);axes[0].grid(alpha=.2)
    for j in [0,1]:
        t,y,pred=illustrations[j];axes[1].plot(t,y-pred,lw=.5,alpha=.7,label=f'Playback {j+1}')
    axes[1].set(title='Residual after timing and 65-tap channel fit',xlabel='Time relative to coarse source match (s)',ylabel='Native RX sample counts',ylim=(-15,15));axes[1].legend();axes[1].grid(alpha=.2)
    fig.suptitle('9668 DIL: pulse alignment and accurate reference interpolation matter')
    fig.savefig(d/'dil-quality.png',dpi=160)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path);a=p.parse_args();analyze(a.directory)
