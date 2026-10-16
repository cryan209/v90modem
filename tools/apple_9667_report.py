#!/usr/bin/env python3
"""Compare a native-rate 9667 capture with the PBX's actual sweep WAV.

A sweep measures receive-path response. It is not a quiet-noise measurement
or a modem BER. Post-sweep call-progress tones are excluded from the response.
"""
import argparse,json,wave
from pathlib import Path
import numpy as np
from scipy import signal
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def windows(x,fs):
    n=int(.12*fs);rows=[]
    for center in np.arange(.2,len(x)/fs-.2,.05):
        v=x[int(center*fs)-n//2:int(center*fs)+n//2].astype(float);v-=v.mean()
        f=np.fft.rfftfreq(32768,1/fs);power=abs(np.fft.rfft(v*np.hanning(len(v)),n=32768))**2
        ix=np.argmax(power)
        rows.append((center,f[ix],np.sqrt(np.mean(v*v)),np.max(abs(v)),np.mean(abs(x[int(center*fs)-n//2:int(center*fs)+n//2])>=32760)))
    return np.array(rows)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path);a=p.parse_args();d=a.directory;fs=10286
    with wave.open(str(d/'source.wav')) as w:
        source_fs=w.getframerate();source=np.frombuffer(w.readframes(w.getnframes()),dtype='<i2').astype(float)
    rx=np.fromfile(d/'rx.s16',dtype='<i2').astype(float)
    sw=windows(source,source_fs);rw=windows(rx,fs)
    # An abrupt return from the high sweep to a low call-progress tone marks
    # its end. Do not score the busy/reorder tone as part of the sweep.
    end=next(i for i in range(1,len(rw)) if rw[i-1,1]>2000 and rw[i,1]<.7*rw[i-1,1])
    rw=rw[:end-1]
    # A log-frequency fit smooths finite-window FFT frequency estimates.
    band=(rw[:,1]>300)&(rw[:,1]<2800)
    slope,intercept=np.polyfit(rw[band,0],np.log(rw[band,1]),1)
    freq=np.exp(slope*rw[:,0]+intercept)
    source_amp=np.interp(freq,sw[:,1],sw[:,2])
    relative=20*np.log10(rw[:,2]/source_amp)
    ref=float(np.interp(1000,freq,relative));relative-=ref
    points={str(f):float(np.interp(f,freq,relative)) if freq[0]<=f<=freq[-1] else None for f in [50,100,200,300,400,500,1000,2000,3000,3400,3600,3800,3900]}
    high=(freq>1000)&(relative<=-3)
    summary=dict(rate_hz=fs,source_duration_s=len(source)/source_fs,capture_duration_s=len(rx)/fs,
        analyzed_sweep_end_s=float(rw[-1,0]),max_analyzed_frequency_hz=float(freq[-1]),
        log_sweep_exponent_per_s=float(slope),gain_db_at_1khz_relative_to_source=ref,
        rx_rms_at_1khz=float(np.interp(1000,freq,rw[:,2])),relative_gain_db=points,
        high_3db_hz=float(freq[np.flatnonzero(high)[0]]) if np.any(high) else None,
        sweep_window_rail_fraction_max=float(np.max(rw[:,4])),
        whole_capture_rail_fraction=float(np.mean(abs(rx)>=32760)))
    np.savetxt(d/'response.csv',np.column_stack([rw[:,0],freq,rw[:,2],relative]),delimiter=',',header='capture_time_s,frequency_hz,rx_rms_counts,gain_relative_1khz_db',comments='')
    (d/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    fig,axes=plt.subplots(2,1,figsize=(11,7),layout='constrained')
    f,t,z=signal.stft(rx/32768,fs,nperseg=1024,noverlap=768)
    axes[0].pcolormesh(t,f,20*np.log10(np.maximum(abs(z),1e-7)),shading='auto',vmin=-80,vmax=-5,cmap='magma')
    axes[0].axvline(rw[-1,0],color='cyan',ls='--',label='Analyzed sweep ends')
    axes[0].set(ylim=(0,5100),xlabel='Time in receive capture (s)',ylabel='Frequency (Hz)',title='9667 received audio: rising sweep, then call-progress tone');axes[0].legend()
    axes[1].plot(freq,relative,color='#4c72b0');axes[1].axhline(-3,color='gray',ls='--')
    axes[1].set(xlim=(40,4000),ylim=(-35,3),xlabel='Frequency (Hz)',ylabel='Gain relative to 1 kHz (dB)',title='Receive-path response relative to the PBX source WAV');axes[1].grid(alpha=.2)
    fig.suptitle('Live Apple modem 0:13 call to 9667 — native 10286 samples/s, receive gain 0 dB')
    fig.savefig(d/'call-response.png',dpi=160)
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
