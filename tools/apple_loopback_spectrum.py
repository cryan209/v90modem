#!/usr/bin/env python3
"""FFT/coherence and held-out channel models for saved Apple loopback probes.

Offline diagnostics only. Forward models train on one block, score a separate
block, and include DC. Inverse EQ and oracle DFE are benchmarks, not changes
to modem DSP. The oracle uses past true symbols and has no error propagation.
"""
import argparse,json
from pathlib import Path
import numpy as np
from scipy import signal
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

FS=8000
ROOT=Path(__file__).resolve().parents[1]
CASES={
 'digital-13':('artifacts/apple-pcm-loopback-20261016-r2/0-13-digital-ulaw','digital'),
 'isolation-13':('artifacts/apple-pcm-loopback-20261016-r2/0-13-isolation-ulaw','isolation'),
 'analog-13':('artifacts/apple-pcm-loopback-20261016-quiet/0-13-analog-ulaw','analog'),
 'digital-14':('artifacts/apple-pcm-loopback-20261016-r2/0-14-digital-ulaw','digital'),
 'isolation-14':('artifacts/apple-pcm-loopback-20261016-r2/0-14-isolation-ulaw','isolation'),
 'analog-14':('artifacts/apple-pcm-loopback-20261016-quiet/0-14-analog-ulaw','analog'),
 'packet-fault-14':('artifacts/apple-pcm-loopback-20261016/0-14-digital-alaw','packet fault'),
}
COLORS={'digital':'#2166ac','isolation':'#d88710','analog':'#12836c','packet fault':'#c23d51'}


def db(x):return 10*np.log10(np.maximum(x,1e-24))
def snr(expected,predicted):
    power=np.var(expected);mse=np.mean((expected-predicted)**2)
    return float(10*np.log10(max(power,1e-24)/max(mse,1e-24)))


def fit(features,target,train,test,ridge=1e-7):
    mean=features[train].mean(axis=0);scale=features[train].std(axis=0)
    scale=np.maximum(scale,1e-6)
    x=np.column_stack([(features-mean)/scale,np.ones(len(target))]);a=x[train]
    reg=np.eye(a.shape[1])*ridge*len(train);reg[-1,-1]=0
    w=np.linalg.solve(a.T@a+reg,a.T@target[train])
    y=x@w;coeff=w[:-1]/scale;bias=w[-1]-mean@coeff
    return dict(snr=snr(target[test],y[test]),rmse=float(np.sqrt(np.mean((target[test]-y[test])**2)))),y,coeff,bias


def analyze(directory,name,kind,out):
    meta=json.loads((directory/'result.json').read_text());lag=meta['lag_samples']
    tx=np.fromfile(directory/'tx.s16',dtype='<i2').astype(float);rx=np.fromfile(directory/'rx.s16',dtype='<i2').astype(float)
    # All windows fall within the PRBS; a 500-sample gap separates train/test.
    indices=np.arange(8256,19700);train=np.arange(0,6000);test=np.arange(6500,len(indices))
    x=tx[indices];y=rx[indices+lag]
    offsets=np.arange(-128,129)
    xf=tx[indices[:,None]+offsets];yf=rx[indices[:,None]+lag+offsets]
    forward,fp,coeff,bias=fit(xf,y,train,test)
    # Hammerstein polynomial controls: static quadratic/cubic before FIR.
    polynomial=np.column_stack([xf,(xf/8192)**2,(xf/8192)**3])
    nonlinear,npred,_,_=fit(polynomial,y,train,test,ridge=1e-6)
    inverse,ip,_,_=fit(yf,x,train,test)
    oracle=np.column_stack([yf,tx[indices[:,None]-np.arange(1,9)]])
    dfe,dp,_,_=fit(oracle,x,train,test)
    # Tap ladder uses fixed train/test, not increasingly overlapping windows.
    ladder={}
    for taps in [17,65,129,257]:
        k=taps//2
        v,_,_,_=fit(yf[:,128-k:129+k],x,train,test)
        ladder[str(taps)]=v
    f,pxx=signal.welch(x-x.mean(),FS,nperseg=1024,noverlap=768)
    _,pyy=signal.welch(y-y.mean(),FS,nperseg=1024,noverlap=768)
    _,cross=signal.csd(x-x.mean(),y-y.mean(),FS,nperseg=1024,noverlap=768)
    h=cross/np.maximum(pxx,1e-24);coherence=np.abs(cross)**2/np.maximum(pxx*pyy,1e-24)
    phase=np.unwrap(np.angle(h));group_delay=-np.gradient(phase,2*np.pi*f)*1000
    _,residual=signal.welch((y-fp)[test],FS,nperseg=512,noverlap=256)
    _,baseline=signal.welch(y[test]-np.mean(y[test]),FS,nperseg=512,noverlap=256)
    summary=dict(name=name,kind=kind,lag_samples=lag,train_samples=len(train),test_samples=len(test),
        dc_rx=float(y.mean()),rx_peak=float(np.max(np.abs(y))),
        rx_rail_fraction=float(np.mean(np.abs(y)>=32760)),forward_fir=forward,forward_cubic=nonlinear,inverse_fir=inverse,oracle_dfe=dfe,inverse_tap_ladder=ladder,
        gain_db_at_hz={str(hz):float(20*np.log10(max(abs(h[np.argmin(abs(f-hz))]),1e-20))) for hz in [100,300,1000,2000,3000,3400,3600,3800,3900]},
        coherence_median_300_3400=float(np.median(coherence[(f>=300)&(f<=3400)])),
        group_delay_ms_p5_p95_300_3400=[float(v) for v in np.percentile(group_delay[(f>=300)&(f<=3400)],[5,95])])
    # h_rev is a causal version of the centered fit. Its 128-sample shift is
    # subtracted from bulk delay by the model harness; all coefficients are
    # measured diagnostic values, not modem training constants.
    np.savez(out/(name+'-model.npz'),fir=coeff[::-1],bias=bias,lag=lag,center=128,
        tx_scale=meta.get('tx_scale',1 if kind=='digital' else .25),noise_rms=forward['rmse'],
        source=str(directory))
    return summary,dict(f=f,pxx=pxx,pyy=pyy,h=h,coherence=coherence,gd=group_delay,residual=residual,baseline=baseline)


def plots(items,out):
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':11,'axes.spines.top':False,'axes.spines.right':False})
    fig,ax=plt.subplots(2,2,figsize=(13,8.2),layout='constrained')
    for name in ['digital-13','isolation-13','analog-13']:
        summary,v=items[name];kind=summary['kind'];color=COLORS[kind];f=v['f']
        ax[0,0].plot(f,db(v['pxx']/32768**2),color=color,alpha=.5,ls='--')
        ax[0,0].plot(f,db(v['pyy']/32768**2),color=color,label=kind)
        ax[0,1].plot(f,20*np.log10(np.maximum(abs(v['h']),1e-12)),color=color,label=kind)
        ax[1,0].plot(f,v['coherence'],color=color,label=kind)
        ax[1,1].plot(f,v['gd'],color=color,label=kind)
    ax[0,0].set(title='FFT power spectra — dashed TX, solid RX',ylabel='dBFS/Hz',ylim=(-110,-20))
    ax[0,1].set(title='Measured RX / TX response',ylabel='Gain (dB)',ylim=(-65,10))
    ax[1,0].set(title='Coherence — repeatable linear relationship',ylabel='Magnitude-squared coherence',ylim=(0,1.03))
    ax[1,1].set(title='Filter group delay, bulk USB delay removed',ylabel='Delay (ms)',ylim=(-1,6))
    for a in ax.flat:a.set(xlim=(80,4000),xlabel='Frequency (Hz)');a.grid(alpha=.2)
    ax[0,0].legend(loc='lower left')
    fig.suptitle('Apple USB modem: digital, isolation and analogue loopbacks\nMeasured from the same known PCM alphabet; analogue recorded on a quiet call',fontsize=15)
    fig.savefig(out/'frequency-response.png',dpi=170);fig.savefig(out/'frequency-response.pdf');plt.close(fig)
    fig,ax=plt.subplots(1,3,figsize=(14,4.7),layout='constrained')
    labels=['digital','isolation','analog'];xx=np.arange(3)
    for i,name in enumerate(['digital-13','isolation-13','analog-13']):
        s,v=items[name];color=COLORS[s['kind']]
        ladder=s['inverse_tap_ladder'];ax[0].plot([int(k) for k in ladder],[min(90,v['snr']) for v in ladder.values()],marker='o',color=color,label=s['kind'])
        ax[1].bar(i-.18,min(90,s['forward_fir']['snr']),width=.35,color=color)
        ax[1].bar(i+.18,min(90,s['forward_cubic']['snr']),width=.35,color=color,alpha=.4)
        ax[2].bar(i-.18,min(90,s['inverse_fir']['snr']),width=.35,color=color)
        ax[2].bar(i+.18,min(90,s['oracle_dfe']['snr']),width=.35,color=color,alpha=.4)
    ax[0].set(title='Linear receive EQ: tap-length sweep',xlabel='Number of taps',ylabel='Held-out SNR (dB)',ylim=(0,95));ax[0].legend()
    ax[1].set(title='Can we model the channel?\nSolid: linear FIR · pale: + quadratic/cubic',xticks=xx,xticklabels=labels,ylabel='Held-out forward prediction SNR (dB)',ylim=(0,95))
    ax[2].set(title='Can we undo the channel?\nSolid: linear EQ · pale: oracle DFE',xticks=xx,xticklabels=labels,ylabel='Held-out reconstruction SNR (dB)',ylim=(0,95))
    for a in ax:a.grid(axis='y',alpha=.2)
    fig.suptitle('Models trained on 6,000 samples; scored on a separate 4,944\nDigital scores capped at 90 dB. Oracle DFE uses true past symbols: a bound, not a working receiver.',fontsize=13)
    fig.savefig(out/'models-and-eq.png',dpi=170);plt.close(fig)
    fig,ax=plt.subplots(1,2,figsize=(12,4.5),layout='constrained')
    for name in ['digital-14','packet-fault-14']:
        s,v=items[name];c=COLORS[s['kind']]
        ax[0].plot(v['f'],20*np.log10(np.maximum(abs(v['h']),1e-12)),color=c,label=s['kind'])
        ax[1].plot(v['f'],v['coherence'],color=c,label=s['kind'])
    ax[0].set(title='A packet fault can resemble a frequency response',ylabel='RX / TX gain (dB)',ylim=(-25,5))
    ax[1].set(title='But coherence exposes the lost consistency',ylabel='Magnitude-squared coherence',ylim=(0,1.03))
    for a in ax:a.set(xlim=(80,4000),xlabel='Frequency (Hz)');a.grid(alpha=.2);a.legend()
    fig.suptitle('Second modem: exact digital repeat versus retained 3-of-8 sample-delay fault',fontsize=14)
    fig.savefig(out/'packet-fault.png',dpi=170);plt.close(fig)


def self_check():
    """Independent random controls for alignment, held-out fitting and DFE."""
    rng=np.random.default_rng(914)
    x=rng.normal(size=14000)
    # A two-tap channel, independent additive noise, and substantial DC.
    y=x+0.6*np.roll(x,1)+rng.normal(scale=.01,size=len(x))+3
    ix=np.arange(32,13968)
    train=np.arange(6000);test=np.arange(6500,len(ix))
    xf=x[ix[:,None]+np.arange(-8,9)]
    yf=y[ix[:,None]+np.arange(-8,9)]
    forward,_,_,bias=fit(xf,y[ix],train,test)
    inverse,_,_,_=fit(yf,x[ix],train,test)
    oracle,_,_,_=fit(np.column_stack([yf,x[ix-1]]),x[ix],train,test)
    noise,_,_,_=fit(xf,rng.normal(size=len(ix)),train,test)
    assert forward['snr']>39 and abs(bias-3)<.01
    assert inverse['snr']>35 and oracle['snr']>38
    assert noise['snr']<.2
    print('Synthetic controls passed: known FIR/DC/noise, inverse, oracle, unrelated signal.')


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path);p.add_argument('--self-check',action='store_true');args=p.parse_args()
    if args.self_check:
        self_check()
        if args.out is None:return
    if args.out is None:p.error('--out is required unless using --self-check')
    args.out.mkdir(parents=True,exist_ok=True)
    items={}
    for name,(path,kind) in CASES.items():
        items[name]=analyze(ROOT/path,name,kind,args.out)
        s=items[name][0];print(name,'forward',round(s['forward_fir']['snr'],2),'cubic',round(s['forward_cubic']['snr'],2),'inverse',round(s['inverse_fir']['snr'],2),'DFE',round(s['oracle_dfe']['snr'],2),flush=True)
    (args.out/'summary.json').write_text(json.dumps([v[0] for v in items.values()],indent=2)+'\n');plots(items,args.out)

if __name__=='__main__':main()
