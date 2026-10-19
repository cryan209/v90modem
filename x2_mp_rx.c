#include "x2_mp_rx.h"
#include "x2_short_banks.h"
#include <math.h>
#include <string.h>
#define PI 3.14159265358979323846
static int positive_ulaw(unsigned code)
{
    unsigned c = (~code) & 127;
    return (int)((((c & 15) << 3) + 132) << (c >> 4)) - 132;
}
/* Ie030002 D229/D238, CB3D: first nine-entry training alphabet, B=19,
 * MD=5. The allocation banks used after training are a separate constructor.
 */
void x2_training_config(x2_pcm_config_t *c)
{
    static const uint8_t codes[] = {0xa5,0xa7,0xad,0xaf,0xb7,0xbd,0xc5,0xcf,0xe5};
    unsigned i,j;
    memset(c,0,sizeof(*c)); c->amplitude_bits=19; c->independent_signs=5;
    for (i=0;i<128;++i) c->levels[i]=(int16_t)(positive_ulaw(128+i)/2);
    for (i=0;i<6;++i) {
        c->sizes[i]=9;
        for(j=0;j<9;++j)c->banks[i][j]=(uint16_t)(codes[j]*257u);
    }
}
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
            /* Only on the hypothesis tracking the MP best. Every MP begins
             * 17 ones, 0, then W1's two low zeros, so one symbol error on
             * bits 16..19 reads as 20 ones: with W1 034c (N1=3) a timing
             * hypothesis rejecting 44% of its frames declared E 300 ms
             * early (client pair, 19 Oct). An ACKed MP is not required:
             * the captured Courier's E follows 0344, its ACK set while the
             * last frame was already on the line. */
            unsigned best=0;
            for(unsigned t=0;t<10;++t)for(unsigned p=0;p<2;++p)
                if(r->hypotheses[t][p].repeats>best)best=r->hypotheses[t][p].repeats;
            if(h->ones==20 && h->repeats>=2 && h->repeats>=best && !r->e_detected) {
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
    /* atan2()/4 jumps by 90 degrees when the fourth-power mean crosses 180
     * degrees -- where the V.34 16-point grid's mean sits at zero carrier
     * offset -- and each jump is a differential symbol error on every
     * hypothesis. Track it continuously instead; differential decoding
     * absorbs the constant 90-degree ambiguity. */
    double estimate=atan2(r->fourth_im[timing],r->fourth_re[timing])/4;
    if(r->phase_valid[timing]) {
        double step=estimate-r->phase[timing];
        estimate=r->phase[timing]+step-floor(step/(PI/2)+0.5)*(PI/2);
    }
    r->phase[timing]=estimate;r->phase_valid[timing]=1;
    for(unsigned p=0;p<2;++p) {
        double phase=estimate+p*PI/4;
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
    return x2_short_profile(index,md,config) ? -1 : (int)index;
}
int x2_short_profile(unsigned index,unsigned md,x2_pcm_config_t *config)
{
    if(!config||!index||index>15||md>6)return -1;
    x2_pcm_config_t c=x2_short_profiles[index-1];c.independent_signs=(uint8_t)md;
    if(x2_pcm_validate(&c))return -1;
    *config=c;return 0;
}
unsigned x2_short_record_rate(unsigned index)
{
    if(!index||index>15)return 0;
    const x2_pcm_config_t *c=&x2_short_profiles[index-1];
    return (c->amplitude_bits+c->independent_signs)*8000u/6u;
}
uint16_t x2_short_record_mask(int min_bps,int max_bps)
{
    uint16_t mask=0;
    for(unsigned index=1;index<=15;++index) {
        int rate=(int)x2_short_record_rate(index);
        if((min_bps<=0||rate>=min_bps)&&(max_bps<=0||rate<=max_bps))mask|=(uint16_t)(1u<<(index-1));
    }
    return mask;
}
