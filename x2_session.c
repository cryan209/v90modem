#include "x2_session.h"
#include <math.h>
#include <string.h>
#define PI 3.14159265358979323846
static uint16_t crc_step(uint16_t crc, unsigned bit)
{
    return (uint16_t)((crc >> 1) ^ (((crc ^ bit) & 1) ? 0x8408 : 0));
}
int x2_info_encode(uint32_t body, unsigned n, uint8_t *bits)
{
    unsigned i; uint16_t crc = 0xffff;
    if (!bits || (n != 7 && n != 17) || body >= (1u << n)) return -1;
    for (i = 0; i < 12; ++i) bits[i] = (0x4ef >> i) & 1;
    for (i = 0; i < n; ++i) { bits[12+i] = (body >> i) & 1; crc = crc_step(crc, bits[12+i]); }
    for (i = 0; i < 16; ++i) bits[12+n+i] = (crc >> i) & 1;
    for (i = 0; i < 4; ++i) bits[28+n+i] = 1;
    return (int)(32+n);
}
int x2_info_decode(const uint8_t *bits, unsigned n, uint32_t *body)
{
    uint32_t value = 0; unsigned i; uint16_t crc = 0xffff, received = 0;
    if (!bits || !body || (n != 7 && n != 17)) return -1;
    for (i = 0; i < n+16; ++i) if (bits[i] > 1) return -1;
    for (i = 0; i < n; ++i) { value |= (uint32_t)bits[i] << i; crc = crc_step(crc,bits[i]); }
    for (i = 0; i < 16; ++i) received |= (uint16_t)(bits[n+i] << i);
    /* Both CRC transmission conventions are found by the independent capture
     * decoder. Body semantics are still checked before selecting x2. */
    if (received != crc && received != (uint16_t)(crc ^ 0xffff)) return -1;
    *body = value; return 0;
}
int x2_marker_parse(unsigned marker, unsigned answering, unsigned *index, unsigned *high)
{
    unsigned first, second, own;
    if (marker > 127 || answering > 1 || !index || !high) return -1;
    first = (marker >> 1) & 7; second = (marker >> 4) & 7;
    own = answering ? first : second;
    if (own != 6) return -1;
    *index = answering ? second : first; *high = marker & 1; return 0;
}
static uint8_t linear_ulaw(int sample)
{
    int sign = sample < 0 ? 0x80 : 0, exponent = 7, mask = 0x4000;
    if (sample < 0) sample = -sample;
    if (sample > 32635) sample = 32635;
    sample += 132;
    while (exponent > 0 && !(sample & mask)) { --exponent; mask >>= 1; }
    return (uint8_t)~(sign | (exponent << 4) | ((sample >> (exponent+3)) & 15));
}
static int positive_ulaw(unsigned code)
{
    unsigned c = (~code) & 127;
    return (int)((((c & 15) << 3) + 132) << (c >> 4)) - 132;
}
static int ones(void *unused) { (void)unused; return 1; }
/* Ie030002 D229/D238, CB3D: first nine-entry training alphabet, B=19,
 * MD=5. The allocation banks used after training are a separate constructor.
 */
static void training_config(x2_pcm_config_t *c)
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
int x2_session_init(x2_session_t *s)
{
    unsigned i;
    if (!s) return -1;
    memset(s,0,sizeof(*s)); s->stage=X2_INFO0;
    /* Captured I-modem INFO0 body 11111111101111000, first bit first.
     * This is V.34's 17-bit INFO0, not V.90's extended INFO0d. */
    x2_info_encode(0x3dff,17,s->info_bits);
    /* V.34 10.1.2.4/Table 17, 24 periods in 160 ms. The captured x2
       digital-server probe (5.01..5.17 s) has RMS 2450, not ordinary
       analogue L1's 6 dB boost. Preserve that PCM-server level here. */
    static const unsigned frequencies[]={150,300,450,600,750,1050,1350,1500,1650,1950,2100,2250,2550,2700,2850,3000,3150,3300,3450,3600,3750};
    static const int signs[]={1,-1,1,1,1,1,1,1,-1,1,1,-1,1,-1,1,-1,-1,-1,-1,1,1};
    for(unsigned n=0;n<160;++n) {
        double x=0;
        for(unsigned j=0;j<21;++j)x+=signs[j]*cos(2*PI*frequencies[j]*n/8000.0);
        s->probe[n]=(int16_t)lrint(750*x);
    }
    for(i=0;i<40;++i)s->info_rx[i].clock=i*200;
    x2_scrambler_init(&s->training_scrambler,18,0);
    return 0;
}
static void mp_received(void *context,const x2_mp_t *mp)
{
    x2_session_receive_mp(context,mp);
}
void x2_session_set_payload_source(x2_session_t *s,x2_get_bit_func_t get_bit,void *context)
{
    if(s){s->payload_source=get_bit;s->payload_context=context;}
}
void x2_session_receive_mp(x2_session_t *s,const x2_mp_t *mp)
{
    x2_pcm_config_t c;unsigned eligible,n=0;
    if(!s||!mp||s->stage<X2_TRAIN_C||s->stage>X2_RECORD_TX)return;
    int index=x2_short_record_config(mp,0x7fff,&c);
    if(index<0){s->stage=X2_FAILED;return;}
    /* W2 is the V.34 upstream capability mask, independent of PCM N1.
     * Highest allowed N no greater than N2; bit zero represents N=1. */
    eligible=mp->words[1]&((1u<<((mp->words[0]>>6)&15))-1);
    while(eligible){++n;eligible>>=1;}
    if(!n || n>14){s->stage=X2_FAILED;return;}
    s->peer_mp=*mp;s->data_config=c;s->selected_index=(unsigned)index;
    s->upstream_rate_n=n;s->mp_valid=1;
}
static int record_bit(void *context)
{
    x2_session_t *s=context;
    unsigned bit=s->record_bits[s->record_position++];
    if(s->record_position==sizeof(s->record_bits))s->record_position=0;
    return (int)bit;
}
static int payload_bit(void *context)
{
    x2_session_t *s=context;
    if(s->stage!=X2_PAYLOAD||!s->payload_source)return 1;
    return s->payload_source(s->payload_context);
}
static void stage(x2_session_t *s, x2_session_stage_t next)
{
    s->stage=next;s->stage_samples=0;
    if(next==X2_INFO0) {
        s->a_reversals=s->b_reversed=s->b_present=s->b_stable=0;
        s->b_samples=s->b_position=s->b_crossing_valid=0;
    }
    if(next==X2_TRAIN_C)x2_mp_rx_init(&s->mp_rx,mp_received,s);
    if(next==X2_DATA_STARTUP) {
        /* Ie030002 A8F1/C940 (039F bit 7 clear): six final training
         * samples, CFF4 bank selection, 0FF0 mapped samples, then CC02.
         * No C998 reset occurs: retain source history, parity and monitor. */
        s->training_mapper.config=s->data_config;
        s->training_mapper.get_bit=payload_bit;s->training_mapper.user_data=s;
    }
    if(next==X2_TRAIN_B)x2_scrambler_init(&s->training_scrambler,18,0);
    if(next==X2_TRAIN_E) {
        x2_pcm_config_t c;training_config(&c);
        if(x2_pcm_tx_init(&s->training_mapper,&c,18,ones,NULL))s->stage=X2_FAILED;
    }
    if(next==X2_RECORD_TX) {
        /* x2 Draft 0.33 sections 6, 20 and 21; Ie030002 CBB8/B54C
         * transmit a THREE-word record in the retained training banks,
         * unlike the Courier's four-word upstream MP. CBB5's data-bank
         * rebuild is not executed in the successful native startup.
         * C9F8 resets source/parity before the 54-sample alignment word.
         * The 85 protected/framing bits occupy four 24-bit mapper frames;
         * B58B supplies zero fill until the next record starts. Retain the
         * native ACK/expanded/64-state flags; N2 is the upstream rate,
         * independently from the PCM index. Courier B06E tests bit 13 for
         * nonlinear encoding: keep it clear because the upstream receiver
         * uses a linear constellation (V.34 9.7). */
        uint16_t words[3]={(uint16_t)(0xd000|(s->selected_index<<2)|(s->upstream_rate_n<<6)),
                           s->peer_mp.words[1],0};
        uint16_t crc=0xffff;unsigned p=0;
        memset(s->record_bits,0,sizeof(s->record_bits));
        for(unsigned i=0;i<17;++i)s->record_bits[p++]=1;
        ++p;
        for(unsigned j=0;j<3;++j) {
            for(unsigned i=0;i<16;++i) {
                unsigned bit=(words[j]>>i)&1;
                s->record_bits[p++]=(uint8_t)bit;crc=crc_step(crc,bit);
            }
            ++p;
        }
        for(unsigned i=0;i<16;++i)s->record_bits[p++]=(uint8_t)((crc>>i)&1);
        s->record_position=0;
        x2_pcm_config_t c;training_config(&c);
        if(x2_pcm_tx_init(&s->training_mapper,&c,18,record_bit,s))s->stage=X2_FAILED;
    }
}
static void info_bit(x2_session_t *s,x2_info_hypothesis_t *h,unsigned bit)
{
    if(!h->count) {
        h->sync=((h->sync<<1)|bit)&255;
        if(h->sync==0x72)h->count=1;
        return;
    }
    h->bits[h->count-1]=(uint8_t)bit;
    unsigned length=h->count++;
    uint32_t body;
    if(s->peer_info_valid && length==23 && !x2_info_decode(h->bits,7,&body)) {
        unsigned index,high;
        if(!x2_marker_parse(body,1,&index,&high)) {
            h->count=0;
            if(index!=4 || !high){stage(s,X2_FAILED);return;}
            s->marker=(uint8_t)body;s->marker_valid=1;++s->accepted_frames;
            stage(s,X2_UPSTREAM_WAIT);
            return;
        }
    }
    if(length!=33)return;
    h->count=0;
    if(x2_info_decode(h->bits,17,&body)){++s->rejected_frames;return;}
    if(!(body&0x40) || (body&0x1800))return;
    unsigned repeated=s->peer_info_valid && s->stage==X2_TONE_A && !s->a_reversals;
    s->peer_capabilities=(uint16_t)body;s->peer_info_valid=1;++s->accepted_frames;
    /* V.34 Table 14 bit 28 and Courier 8E8B/9120/9141: during error
     * recovery the peer repeats 17-bit INFO0 with ACK, even after the
     * first INFO0 was accepted. V.34 11.2.2.2.1 also requires recovery
     * on repeated INFO0c before the tone exchange, even without its ACK.
     * Do not reinterpret either frame as a 7-bit marker. */
    if(((body&0x10000) || repeated) && !s->info_bits[28]) {
        x2_info_encode(0x13dff,17,s->info_bits);
        s->info_position=s->info_clock=0;
        stage(s,X2_INFO0);
    }
    for(unsigned i=0;i<40;++i)s->info_rx[i].count=0;
}
/* Draft 0.33 clause 6: the Courier restarts with V.34 S/S-bar while
 * digital J repeats. Inspect the line, independent of a stale TRN equalizer.
 * At 3200/high the alternating S waveform has lines at 320/1920/3520 Hz.
 * Require sustained coherent S, then its polarity reversal into S-bar.
 * A time limit or S onset alone must not release the J acknowledgement.
 * The 40-sample window adds at most one window of detection latency. */
static void upstream_s_sample(x2_session_t *s, int16_t sample)
{
    static const unsigned frequencies[] = {320,1920,3520};
    double re[3]={0}, im[3]={0}, power[3], energy=0, total=0;
    unsigned j,k; int stable=1, reversed=1;
    s->s_window[s->s_position++]=sample;s->s_position%=40;
    if(++s->s_samples<40 || s->s_samples%5)return;
    for(j=0;j<40;++j) {
        double x=s->s_window[(s->s_position+j)%40];
        uint64_t time=s->rx_samples+1-40+j;
        energy+=x*x;
        for(k=0;k<3;++k) {
            double phase=2*PI*frequencies[k]*(double)(time%200)/8000;
            re[k]+=x*cos(phase);im[k]-=x*sin(phase);
        }
    }
    for(k=0;k<3;++k){power[k]=re[k]*re[k]+im[k]*im[k];total+=power[k];}
    if(energy<40*10000.0 || total<0.75*energy*20
       || power[0]<0.08*total || power[1]<0.08*total || power[2]<0.08*total) {
        /* Preserve the last coherent reference across the reversal's short
         * cancellation window, but abandon it after 10 ms without S. */
        if(++s->s_missing>16)s->s_stable=0;
        return;
    }
    s->s_missing=0;
    for(k=0;k<3;++k) {
        double reference=s->s_reference_re[k]*s->s_reference_re[k]
                        +s->s_reference_im[k]*s->s_reference_im[k];
        double dot=re[k]*s->s_reference_re[k]+im[k]*s->s_reference_im[k];
        double norm=sqrt(power[k]*reference);
        if(dot<0.85*norm)stable=0;
        if(dot>-0.80*norm || norm==0)reversed=0;
    }
    if(s->s_stable>=32 && reversed) {x2_session_upstream_s_bar(s);return;}
    if(!stable)s->s_stable=0;
    ++s->s_stable;
    for(k=0;k<3;++k){s->s_reference_re[k]=re[k];s->s_reference_im[k]=im[k];}
}
/* V.34 10.1.2 and 11.2.1.2.3-.5. A 5 ms coherent window
 * distinguishes sustained Tone B from INFO0 phase modulation. Retain the
 * pre-reversal phase through cancellation, and timestamp its zero crossing
 * at the window centre so detector confirmation does not extend the 40 ms.
 * Courier x2 follows this with the short channel probe, then its
 * directional marker in place of ordinary INFO1 (native 9083). */
static void phase2_b_sample(x2_session_t *s,int16_t sample,uint64_t tx_time)
{
    double re=0,im=0,energy=0;
    unsigned j;
    s->b_window[s->b_position++]=sample;
    s->b_position%=40;
    if(s->b_samples<40){++s->b_samples;return;}
    for(j=0;j<40;++j) {
        double x=s->b_window[(s->b_position+j)%40];
        double phase=2*PI*1200*(double)((s->rx_samples+1-40+j)%20)/8000;
        re+=x*cos(phase);im-=x*sin(phase);energy+=x*x;
    }
    double power=re*re+im*im;
    double reference=s->b_reference_re*s->b_reference_re+s->b_reference_im*s->b_reference_im;
    double dot=re*s->b_reference_re+im*s->b_reference_im;
    if(s->a_reversals==1 && s->b_present && !s->b_reversed) {
        if(!s->b_crossing_valid && dot<0) {
            s->b_crossing_sample=s->rx_samples-20;
            s->b_crossing_valid=1;
        }
        if(reference>0 && dot < -0.8*reference && energy>100000) {
            uint64_t age=s->rx_samples-s->b_crossing_sample;
            s->second_a_tx_sample=tx_time+(age<320?320-age:0);
            s->b_reversed=1;
        }
        return;
    }
    if(energy<100000 || 2*power<0.90*40*energy) {
        s->b_stable=0;return;
    }
    if(reference>0 && dot<0.90*sqrt(power*reference))s->b_stable=0;
    s->b_reference_re=re;s->b_reference_im=im;
    if(++s->b_stable>=160)s->b_present=1;
}
void x2_session_rx(x2_session_t *s,const int16_t *samples,size_t count)
{
    size_t k;unsigned i;
    if(!s||!samples)return;
    if(s->stage>=X2_TRAIN_C && s->stage<=X2_PAYLOAD && !s->mp_rx.e_detected)
        x2_mp_rx_audio(&s->mp_rx,samples,count);
    for(k=0;k<count;++k,++s->rx_samples) {
        double phase=2*PI*1200*(double)(s->rx_samples%20)/8000;
        double re=samples[k]*cos(phase),im=-samples[k]*sin(phase);
        if(s->stage==X2_TONE_A && s->peer_info_valid)
            phase2_b_sample(s,samples[k],s->tx_samples+k);
        if(s->stage==X2_J && !s->s_bar_seen)upstream_s_sample(s,samples[k]);
        if(s->marker_valid||s->stage==X2_FAILED)continue;
        for(i=0;i<40;++i) {
            x2_info_hypothesis_t *h=&s->info_rx[i];
            h->re+=re;h->im+=im;h->clock+=600;
            if(h->clock>=8000) {
                double dot=h->re*h->previous_re+h->im*h->previous_im;
                /* Empty signal has no differential phase information. */
                if(h->re*h->re+h->im*h->im>10000 &&
                   h->previous_re*h->previous_re+h->previous_im*h->previous_im>10000)
                    info_bit(s,h,dot<0);
                else {h->count=0;h->sync=0;}
                h->previous_re=h->re;h->previous_im=h->im;
                h->re=h->im=0;h->clock-=8000;
                if(s->marker_valid)break;
            }
        }
    }
}
void x2_session_upstream_j(x2_session_t *s)
{
    if(s && s->stage==X2_UPSTREAM_WAIT) {s->upstream_ready=1;stage(s,X2_ZERO);}
}
void x2_session_upstream_s_bar(x2_session_t *s)
{
    if(s && s->stage==X2_J)s->s_bar_seen=1;
}
size_t x2_session_tx(x2_session_t *s,uint8_t *octets,size_t count)
{
    size_t k;
    if(!s||!octets)return 0;
    for(k=0;k<count;++k,++s->tx_samples) {
        unsigned n=s->stage_samples;uint8_t code=0x7f;
        x2_session_stage_t old_stage=s->stage;
        if(n>80000 && s->stage!=X2_FAILED && s->stage!=X2_PAYLOAD){stage(s,X2_FAILED);old_stage=s->stage;n=0;}
        switch(s->stage) {
        case X2_INFO0:
        case X2_TONE_A: {
            if(s->stage==X2_TONE_A) {
                if(!s->a_reversals && n>=400 && s->peer_info_valid && s->b_present) {
                    s->sign^=1;s->a_reversals=1;
                    s->b_crossing_valid=0;
                }
                if(s->a_reversals==1 && s->b_reversed && s->tx_samples>=s->second_a_tx_sample) {
                    s->sign^=1;s->a_reversals=2;
                }
            }
            if(s->stage==X2_INFO0 && s->info_clock<600 && s->info_bits[s->info_position])s->sign^=1;
            code=linear_ulaw((int)((s->sign?-3000:3000)*cos(2*PI*2400*(double)(s->tx_samples%10)/8000)));
            if(s->stage==X2_INFO0) {
                s->info_clock+=600;
                if(s->info_clock>=8000) {
                    s->info_clock-=8000;
                    if(++s->info_position==49) {
                        s->info_position=0;
                        stage(s,X2_TONE_A);
                    }
                }
            } else if(s->stage==X2_TONE_A) {
                if(s->a_reversals==2 && s->tx_samples>=s->second_a_tx_sample+79)
                    stage(s,X2_PROBE);
            }
            break;
        }
        /* Courier/I-modem asymmetric wire trace: tone A ends before the
         * caller's marker (5.01 s vs 5.27 s). Holding A in MARKER_WAIT
         * prevents the real caller from leaving Phase 2. */
        case X2_PROBE:
            code=linear_ulaw(s->probe[n%160]);
            if(n==1279)stage(s,X2_MARKER_WAIT);
            break;
        case X2_MARKER_WAIT:
        case X2_UPSTREAM_WAIT:break;
        case X2_ZERO:if(n==19)stage(s,X2_PATTERN_A);break;
        case X2_PATTERN_A: {
            unsigned cycle=n/11,bit=n%11,levels=cycle&1?~0x712u:0x712u;
            code=((levels>>bit)&1)?0xbd:0xab;
            if((0x5b8>>bit)&1)code^=0x80;
            if(n>=352)code^=0x80;
            if(n==362)stage(s,X2_TRAIN_B);
            break;
        }
        case X2_TRAIN_B:
            code=(uint8_t)(0xb1^(x2_scramble_bit(&s->training_scrambler,1)<<7));
            if(n==20003)stage(s,X2_J);
            break;
        case X2_J:
        case X2_J_ACK: {
            unsigned word=s->stage==X2_J?0x9b0:0x99f;
            code=(uint8_t)(0xb1^(x2_scramble_bit(&s->training_scrambler,(word>>(n%12))&1)<<7));
            if(s->stage==X2_J && s->s_bar_seen && n%12==11)stage(s,X2_J_ACK);
            else if(s->stage==X2_J_ACK && n==11)stage(s,X2_TRAIN_C);
            break;
        }
        case X2_TRAIN_C:
            code=(uint8_t)(0xc1^(x2_scramble_bit(&s->training_scrambler,1)<<7));
            if(n==5999)stage(s,X2_TRAIN_D);
            break;
        case X2_TRAIN_D: {
            static const uint8_t table[]={0x9e,0x28,0xa1,0x21,0xa8,0x1e};
            code=table[(n+n/6)%6];
            if(n==1151)stage(s,X2_TRAIN_E);
            break;
        }
        case X2_TRAIN_E:
            x2_pcm_tx_g711(&s->training_mapper,&code,1);
            if(n==10001)stage(s,X2_RECORD_WAIT);
            break;
        case X2_RECORD_WAIT:
            if(s->mp_valid) {
                stage(s,X2_RECORD_ALIGN);
                code=0xa5;s->stage_samples=1;
            } else x2_pcm_tx_g711(&s->training_mapper,&code,1);
            break;
        case X2_RECORD_ALIGN:
            /* C936/C939/C9F8: eight +++--- groups, then one inverted
             * group. All six current training banks start with A5. */
            code=(uint8_t)(0xa5^((n%6>=3 ? 1u:0u)<<7));
            if(n>=48)code^=0x80;
            if(n==53)stage(s,X2_RECORD_TX);
            break;
        case X2_RECORD_TX:
            x2_pcm_tx_g711(&s->training_mapper,&code,1);
            /* Ie030002 A891/A8F1: received E releases the final training
             * and data-bank handoff. Finish the protected record first. */
            if(s->mp_rx.e_detected && s->record_position==0
               && s->training_mapper.output_position==6) {
                s->training_mapper.get_bit=ones;s->training_mapper.user_data=NULL;
                stage(s,X2_FINAL_TRAINING);
            }
            break;
        case X2_FINAL_TRAINING:
            x2_pcm_tx_g711(&s->training_mapper,&code,1);
            if(n==5)stage(s,X2_DATA_STARTUP);
            break;
        case X2_DATA_STARTUP:
        case X2_PAYLOAD:
            if(x2_pcm_tx_g711(&s->training_mapper,&code,1)!=1)return k;
            if(s->stage==X2_DATA_STARTUP && n==4079)stage(s,X2_PAYLOAD);
            break;
        case X2_FAILED:break;
        }
        octets[k]=code;
        /* A transition resets the next stage's counter to zero. */
        if(s->stage==old_stage)++s->stage_samples;
    }
    return count;
}
const char *x2_session_stage_name(x2_session_stage_t s)
{
    static const char *names[]={"INFO0","TONE_A","PROBE","MARKER_WAIT","UPSTREAM_WAIT","ZERO","PATTERN_A","TRAIN_B","J","J_ACK","TRAIN_C","TRAIN_D","TRAIN_E","RECORD_WAIT","RECORD_ALIGN","RECORD_TX","FINAL_TRAINING","DATA_STARTUP","PAYLOAD","FAILED"};
    return (unsigned)s<sizeof(names)/sizeof(names[0])?names[s]:"INVALID";
}
