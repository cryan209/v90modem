#include "x2_mp_rx.h"
#include "x2_short_banks.h"
#include <math.h>
#include <string.h>
#define PI 3.14159265358979323846
void x2_mp_rx_init(x2_mp_rx_t *r,x2_mp_received_t received,void *context)
{
    if(!r)return;
    memset(r,0,sizeof(*r));r->received=received;r->context=context;
    for(unsigned i=0;i<127;++i) {
        double x=(double)i-63,fc=1700.0/8000;
        r->taps[i]=(x==0?1:sin(2*PI*fc*x)/(2*PI*fc*x))
                    *(0.54-0.46*cos(2*PI*i/126))*2*fc;
    }
    for(unsigned i=0;i<10;++i) {
        r->next_symbol[i]=i*0.25;
        for(unsigned j=0;j<2;++j)x2_scrambler_init(&r->hypotheses[i][j].descrambler,5,0);
    }
}
void x2_mp_rx_bit(x2_mp_rx_t *r,unsigned timing,unsigned phase,unsigned bit)
{
    if(!r||timing>=10||phase>=2||bit>1)return;
    x2_mp_rx_hypothesis_t *h=&r->hypotheses[timing][phase];
    if(!h->position) {
        if(bit) {
            if(h->ones<20)++h->ones;
            /* Draft 0.33 upstream continuation: the same hypothesis must
             * have two agreeing CRC-valid MPs. MP sync is only 17 ones;
             * bits inside a protected record never count toward E. */
            if(h->ones==20 && h->repeats>=2 && !r->e_detected) {
                r->e_detected=1;r->e_timing=timing;r->e_phase=phase;
                r->e_sample=r->samples;
            }
        }
        else {
            if(h->ones>=17){memset(h->frame,1,17);h->frame[17]=0;h->position=18;}
            h->ones=0;
        }
        return;
    }
    h->frame[h->position++]=(uint8_t)bit;
    if(h->position==104) {
        x2_mp_t mp;h->position=0;
        if(x2_mp_decode(h->frame,&mp)) {++r->rejected_frames;h->repeats=0;return;}
        ++r->valid_frames;
        /* Ignore the acknowledgement bit for repeat qualification: the
         * acknowledged final frame must retain all negotiated parameters. */
        if(h->repeats && ((h->previous_mp.words[0]^mp.words[0])&0x7fff)==0
           && !memcmp(h->previous_mp.words+1,mp.words+1,6))++h->repeats;
        else h->repeats=1;
        h->previous_mp=mp;
        if(h->repeats>=2 && r->received)r->received(r->context,&mp);
    }
}
static void symbol(x2_mp_rx_t *r,unsigned timing,double re,double im)
{
    double power=re*re+im*im,rr=re*re-im*im,ii=2*re*im;
    const double alpha=1.0/512;
    r->power[timing]+=alpha*(power-r->power[timing]);
    r->fourth_re[timing]+=alpha*(rr*rr-ii*ii-r->fourth_re[timing]);
    r->fourth_im[timing]+=alpha*(2*rr*ii-r->fourth_im[timing]);
    if(r->samples<1000)return;
    double reference=sqrt(r->power[timing]);
    if(reference<100)return;
    for(unsigned p=0;p<2;++p) {
        double phase=atan2(r->fourth_im[timing],r->fourth_re[timing])/4+p*PI/4;
        double x=re*cos(phase)+im*sin(phase),y=im*cos(phase)-re*sin(phase);
        double angle=atan2(y,x)*180/PI;if(angle<0)angle+=360;
        double radius=sqrt(power);unsigned orbit;double base;
        if(radius<0.671*reference){orbit=0;base=0;}
        else if(radius>1.164*reference){orbit=3;base=0;}
        else if(fmod(angle,90)<45){orbit=1;base=26.565051177;}
        else {orbit=2;base=63.434948823;}
        static const unsigned offset[]={0,1,2,2};
        unsigned rotation=(unsigned)((int)lround((angle-base)/90)+8)%4;
        unsigned z=(4-rotation+offset[orbit])%4;
        x2_mp_rx_hypothesis_t *h=&r->hypotheses[timing][p];
        if(r->four_point) {
            /* Points at 45 degrees; the same clockwise differential
             * convention as the 16-point quadrant bits, LSB first, GPA.
             * Decodes the captured in-data MP 0378/1ffe/0000/0500 and its
             * acknowledged 8378 (I-modem pair, 50.27/50.33 s). */
            unsigned quadrant=(unsigned)((int)lround((angle-45)/90)+8)%4;
            unsigned z4=(4-quadrant)%4;
            unsigned dibit=(z4+4-h->previous_rotation)%4;h->previous_rotation=z4;
            for(unsigned k=0;k<2;++k)x2_mp_rx_bit(r,timing,p,
                x2_descramble_bit(&h->descrambler,(dibit>>k)&1));
            continue;
        }
        unsigned nibble=(orbit<<2)|((z+4-h->previous_rotation)%4);h->previous_rotation=z;
        /* Courier C20F/AF63: differential I bits followed by point Q bits,
         * LSB first, through GPA. CRC owns acceptance, not constellation fit. */
        for(unsigned k=0;k<4;++k)x2_mp_rx_bit(r,timing,p,
            x2_descramble_bit(&h->descrambler,(nibble>>k)&1));
    }
}
void x2_mp_rx_audio(x2_mp_rx_t *r,const int16_t *samples,size_t count)
{
    if(!r||!samples)return;
    for(size_t n=0;n<count;++n,++r->samples) {
        double phase=2*PI*1920*(double)(r->samples%25)/8000,re=0,im=0;
        r->ring_re[r->position]=samples[n]*cos(phase);
        r->ring_im[r->position]=-samples[n]*sin(phase);
        for(unsigned j=0;j<127;++j) {
            unsigned i=(r->position+127-j)%127;
            re+=r->ring_re[i]*r->taps[j];im+=r->ring_im[i]*r->taps[j];
        }
        r->position=(r->position+1)%127;
        for(unsigned h=0;h<10;++h) {
            if(r->next_symbol[h]<=(double)r->samples) {
                double f=(double)r->samples-r->next_symbol[h];
                symbol(r,h,re*(1-f)+r->previous_re*f,im*(1-f)+r->previous_im*f);
                r->next_symbol[h]+=2.5;
            }
        }
        r->previous_re=re;r->previous_im=im;
    }
}
int x2_short_record_config(const x2_mp_t *mp,unsigned local_mask,x2_pcm_config_t *config)
{
    unsigned ceiling,eligible,index=0,md;
    if(!mp||!config||local_mask>0x7fff || (mp->words[0]&~0x83fc)
       ||mp->words[2] ||(mp->words[3]&255))return -1;
    md=mp->words[3]>>8;if(md>6)return -1;
    ceiling=(mp->words[0]>>2)&15;
    eligible=local_mask&((1u<<ceiling)-1);
    while(eligible){++index;eligible>>=1;}
    if(!index)return -1;
    x2_pcm_config_t c=x2_short_profiles[index-1];c.independent_signs=(uint8_t)md;
    if(x2_pcm_validate(&c))return -1;
    *config=c;return (int)index;
}
