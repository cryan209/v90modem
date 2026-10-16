#!/usr/bin/env python3
"""Separate quiet noise, active noise and harmonic distortion in 1 kHz probes.

Digital-full-scale reference is 32767/sqrt(2) sample units. It is not an
independently calibrated TIP/RING analogue full-scale measurement.
"""
import argparse,json
from pathlib import Path
import numpy as np
from scipy import signal,optimize
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
FS_RMS=32767/np.sqrt(2)


def band_power(x,fs):
    f,p=signal.welch(x,fs,nperseg=1024,noverlap=512)
    return float(np.sum(p[(f>=300)&(f<=3400)])*(f[1]-f[0]))


def tone_fit(y,fs):
    t=np.arange(len(y))/fs
    def design(freq,harmonics):
        return np.column_stack([np.ones(len(y)),t-t.mean()]+[
            fun(2*np.pi*freq*k*t) for k in range(1,harmonics+1) for fun in [np.sin,np.cos]])
    def error(freq):
        a=design(freq,1);w=np.linalg.lstsq(a,y,rcond=None)[0]
        return np.mean((y-a@w)**2)
    freq=optimize.minimize_scalar(error,bounds=(999,1001),method='bounded',options={'xatol':1e-8}).x
    a=design(freq,5);w=np.linalg.lstsq(a,y,rcond=None)[0]
    harmonics=np.array([np.sum(w[2+2*k:4+2*k]**2)/2 for k in range(5)])
    # Reference DC/ramp are excluded from both noise and distortion.
    residual=y-a@w
    return freq,harmonics,residual


def analyze(d):
    r=json.loads((d/'result.json').read_text());fs=r['rate_hz']
    if r['exit_code']!=0:return None
    assert r['restored']==1 and r['pattern']=='tone'
    y=np.fromfile(d/'rx.s16',dtype='<i2').astype(float)
    x=np.fromfile(d/'tx.s16',dtype='<i2').astype(float)
    # Windows sit safely within steady tone and pre-probe silence even though
    # a periodic tone cannot provide a unique transport-delay estimate.
    active=y[int(1.45*fs):int(2.3*fs)]
    quiet=y[int(.35*fs):int(.8*fs)]
    assert np.all(x[int(.35*fs):int(.8*fs)]==0)
    frequency,h,residual=tone_fit(active,fs)
    tx_frequency,tx_h,tx_residual=tone_fit(x[int(1.45*fs):int(2.3*fs)],fs)
    noise=band_power(residual,fs);qn=band_power(quiet,fs)
    distortion=sum(h[k-1] for k in [2,3] if k*frequency<=3400)
    db=lambda ratio:float(10*np.log10(max(ratio,1e-30)))
    return dict(device=r['device'],level=r['tone_level'],rx_gain_db=r['rx_gain_db'],directory=str(d),distorted=bool(distortion/h[0]>.0001 or noise/max(qn,1e-20)>25),
        tx_frequency_hz=float(tx_frequency),tx_signal_rms=float(np.sqrt(tx_h[0])),
        tx_thd_band_db=db(np.sum(tx_h[1:3])/tx_h[0]),
        frequency_hz=float(frequency),signal_rms=float(np.sqrt(h[0])),quiet_noise_rms=float(np.sqrt(qn)),
        active_noise_rms=float(np.sqrt(noise)),quiet_snr_db=db(h[0]/qn),active_snr_db=db(h[0]/noise),
        sndr_db=db(h[0]/(noise+distortion)),thd_band_db=db(distortion/h[0]),
        quiet_digital_fs_dynamic_range_db=db(FS_RMS**2/qn),active_digital_fs_dynamic_range_db=db(FS_RMS**2/noise),
        rx_peak=float(np.max(np.abs(active))),rail_fraction=float(np.mean(np.abs(active)>=32760)))


def self_check():
    fs=10286;rng=np.random.default_rng(84);t=np.arange(fs)/fs
    y=1000*np.sin(2*np.pi*1000.13*t)+1*np.sin(2*np.pi*2000.26*t)+rng.normal(size=fs)+17
    f,h,res=tone_fit(y,fs)
    assert abs(f-1000.13)<.002 and abs(np.sqrt(h[0])-1000/np.sqrt(2))<.1
    assert abs(np.sqrt(h[1])-.707)<.05
    assert abs(band_power(res,fs)-(3400-300)/(fs/2))<.06
    print('Synthetic tone/noise/harmonic checks passed.')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path,nargs='*');p.add_argument('--self-check',action='store_true');a=p.parse_args()
    if a.self_check:
        self_check()
        if not a.directory:return
    if not a.directory:p.error('directory is required')
    rows=[v for d in sorted(d for root in a.directory for d in root.iterdir()) if (d/'result.json').exists() and (v:=analyze(d))]
    (a.directory[0]/'summary.json').write_text(json.dumps(rows,indent=2)+'\n')
    fig,axes=plt.subplots(1,2,figsize=(11,4.7),layout='constrained')
    for device,color in [('0:13','#4c72b0'),('0:14','#c44e52')]:
        selected=sorted([r for r in rows if r['device']==device and r['level']==1 and not r['distorted']],key=lambda r:r['rx_gain_db'])
        axes[0].plot([r['rx_gain_db'] for r in selected],[r['active_snr_db'] for r in selected],marker='o',color=color,label=device+' noise only')
        axes[0].plot([r['rx_gain_db'] for r in selected],[r['sndr_db'] for r in selected],marker='x',ls='--',color=color,label=device+' noise + harmonics')
        selected=sorted([r for r in rows if r['device']==device and r['rx_gain_db']==0 and not r['distorted']],key=lambda r:r['level'])
        axes[1].plot([20*np.log10(r['level']) for r in selected],[r['sndr_db'] for r in selected],marker='o',color=color,label=device)
    for r in rows:
        if r['distorted']:
            axes[0 if r['level']==1 else 1].scatter(r['rx_gain_db'] if r['level']==1 else 20*np.log10(r['level']),r['sndr_db'],marker='x',color='gray',label='Distorted capture: '+r['device'])
    axes[0].set(title='Does receive gain help? Fixed transmitted tone',xlabel='Temporary receive gain (dB)',ylabel='Measured in-band ratio (dB)')
    axes[1].set(title='Does a stronger probe help? Receive gain 0 dB',xlabel='Tone level relative to maximum test level (dB)',ylabel='Signal / noise + harmonics (dB)')
    for ax in axes:ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.suptitle('Apple analog loopback: 1 kHz at 10286 samples/s; 300–3400 Hz measurements')
    fig.savefig(a.directory[0]/'snr-sweep.png',dpi=160)
    for r in rows:print(r['device'],r['level'],r['rx_gain_db'],'SNR',round(r['active_snr_db'],2),'SNDR',round(r['sndr_db'],2),'quiet DR',round(r['quiet_digital_fs_dynamic_range_db'],2),flush=True)


if __name__=='__main__':main()
