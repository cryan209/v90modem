#include "x2_session.h"
#include "x2_training_capture.h"
#include "x2_short_vectors.h"
#include "x2_mp_capture.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#define PI 3.14159265358979323846
static int ulaw_linear(unsigned code)
{
    unsigned c=(~code)&255;
    int x=(int)((((c&15)<<3)+132)<<((c>>4)&7))-132;
    return c&128?-x:x;
}
static void codec_test(void)
{
    uint8_t bits[49];uint32_t body;unsigned i,m,role;
    assert(x2_info_encode(0x3dff,17,bits)==49);
    assert(x2_info_decode(bits+12,17,&body)==0 && body==0x3dff);
    for(i=12;i<45;++i){bits[i]^=1;assert(x2_info_decode(bits+12,17,&body)==-1);bits[i]^=1;}
    for(m=0;m<128;++m)for(role=0;role<2;++role){
        unsigned index=99,high=99,own=role?(m>>1)&7:(m>>4)&7;
        int rc=x2_marker_parse(m,role,&index,&high);
        assert((rc==0)==(own==6));
        if(rc==0)assert(index==(role?(m>>4)&7:(m>>1)&7) && high==(m&1));
    }
}
static void rx_frame(x2_session_t *s,unsigned body,unsigned n,unsigned delay)
{
    uint8_t bits[49];unsigned k,clock=0,position=0,sign=0;
    int16_t samples[1024];unsigned count=(unsigned)x2_info_encode(body,n,bits);
    for(k=0;k<delay;++k)samples[k]=0;
    x2_session_rx(s,samples,delay);
    for(k=0;position<count;++k){
        if(clock<600 && bits[position])sign^=1;
        samples[k]=(int16_t)((sign?-3000:3000)*cos(2*PI*1200*k/8000));
        clock+=600;if(clock>=8000){clock-=8000;++position;}
    }
    x2_session_rx(s,samples,k);
}
/* Feed a real coherent Tone B and its phase reversal, rather than advancing
   the transmitter by a timer. Phase-2 receiver state must survive chunk seams. */
static void phase2_exchange(x2_session_t *s,unsigned block)
{
    int16_t rx[160];uint8_t tx[160];unsigned pos=0;
    uint64_t start=s->tx_samples;
    while(pos<1200) {
        unsigned n=1200-pos;if(n>block)n=block;
        for(unsigned j=0;j<n;++j) {
            unsigned k=pos+j;
            rx[j]=k>=800?0:(int16_t)((k>=720?-3000:3000)*cos(2*PI*1200*(double)((s->rx_samples+j)%20)/8000));
        }
        x2_session_rx(s,rx,n);x2_session_tx(s,tx,n);pos+=n;
    }
    assert(s->a_reversals==2 && s->b_reversed);
    assert(s->second_a_tx_sample>=start+1032 && s->second_a_tx_sample<=start+1048);
    assert(s->stage==X2_PROBE);
    x2_session_tx(s,tx,1); /* Probe does not terminate on a guessed peer ACK. */
    for(unsigned i=0;i<8;++i)x2_session_tx(s,tx,160);
    assert(s->stage==X2_MARKER_WAIT);
}
static void info_recovery_test(void)
{
    x2_session_t s;uint8_t pcm[2048];
    x2_session_init(&s);
    rx_frame(&s,0x21ff,17,7);
    assert(s.peer_info_valid);
    x2_session_tx(&s,pcm,2048);
    assert(s.stage==X2_TONE_A);
    rx_frame(&s,0x121ff,17,9);
    assert(s.stage==X2_INFO0 && s.info_bits[28]);
    x2_session_tx(&s,pcm,654);
    assert(s.stage==X2_TONE_A);
    phase2_exchange(&s,1);
    assert(s.stage==X2_MARKER_WAIT);
    x2_session_tx(&s,pcm,160);
    for(unsigned i=0;i<160;++i)assert(pcm[i]==0x7f);
    rx_frame(&s,0x4d,7,11);
    assert(s.marker_valid && s.stage==X2_UPSTREAM_WAIT);
}
static void phase2_test(void)
{
    {
        x2_session_t s;uint8_t pcm[654];
        x2_session_init(&s);rx_frame(&s,0x25ff,17,7);
        x2_session_tx(&s,pcm,654);assert(s.stage==X2_TONE_A);
        rx_frame(&s,0x25ff,17,9);
        assert(s.stage==X2_INFO0 && s.info_bits[28]);
    }
    const unsigned blocks[]={1,17,160};
    for(unsigned b=0;b<3;++b) {
        x2_session_t s;uint8_t pcm[654];int16_t silence[160]={0};
        x2_session_init(&s);rx_frame(&s,0x21ff,17,7);
        x2_session_tx(&s,pcm,654);assert(s.stage==X2_TONE_A);
        for(unsigned i=0;i<20;++i){x2_session_rx(&s,silence,160);x2_session_tx(&s,pcm,160);}
        assert(s.stage==X2_TONE_A && s.a_reversals==0);
        /* Reset the tone duration so all chunk tests have the same clock origin. */
        s.stage_samples=0;
        phase2_exchange(&s,blocks[b]);
    }
}
static void training_test(void)
{
    x2_session_t s;uint8_t octets[2048];unsigned count=0;
    assert(x2_session_init(&s)==0);
    rx_frame(&s,0x21ff,17,7);
    assert(s.peer_info_valid && s.peer_capabilities==0x21ff);
    rx_frame(&s,0x4d,7,9);
    assert(s.marker_valid && s.stage==X2_UPSTREAM_WAIT);
    x2_session_tx(&s,octets,160);assert(s.stage==X2_UPSTREAM_WAIT);
    x2_session_upstream_j(&s);assert(s.stage==X2_ZERO);
    x2_session_tx(&s,octets,20);assert(s.stage==X2_PATTERN_A);
    x2_session_tx(&s,octets,363);assert(s.stage==X2_TRAIN_B);
    /* Independent capture-verified Barker expression. */
    for(unsigned i=0;i<363;++i){
        unsigned k=i%11,w=(i/11)&1?~0x712u:0x712u;
        unsigned expected=((w>>k)&1)?0xbd:0xab;
        if((0x5b8>>k)&1)expected^=0x80;if(i>=352)expected^=0x80;
        assert(octets[i]==expected);
    }
    while(count<20004){unsigned n=20004-count;if(n>2048)n=2048;x2_session_tx(&s,octets,n);count+=n;}
    assert(s.stage==X2_J);
    x2_session_tx(&s,octets,1200);assert(s.stage==X2_J); /* Hold, never a guessed timer. */
    x2_session_upstream_s_bar(&s);
    x2_session_tx(&s,octets,12);assert(s.stage==X2_J_ACK);
    x2_session_tx(&s,octets,12);assert(s.stage==X2_TRAIN_C);
    for(count=0;count<6000;){unsigned n=6000-count;if(n>2048)n=2048;x2_session_tx(&s,octets,n);count+=n;}
    assert(s.stage==X2_TRAIN_D);
    x2_session_tx(&s,octets,1152);assert(s.stage==X2_TRAIN_E);
    {const uint8_t table[]={0x9e,0x28,0xa1,0x21,0xa8,0x1e};for(unsigned i=0;i<1152;++i)assert(octets[i]==table[(i+i/6)%6]);}
    for(count=0;count<10002;){unsigned n=10002-count;if(n>2048)n=2048;x2_session_tx(&s,octets,n);count+=n;}
    assert(s.stage==X2_RECORD_WAIT);
    /* CRC-valid marker with an unsupported upstream carrier cannot advance. */
    x2_session_init(&s);rx_frame(&s,0x21ff,17,0);rx_frame(&s,0x4c,7,9);
    assert(s.stage==X2_FAILED);
}
static void s_bar_test(void)
{
    const unsigned blocks[]={1,17,160};
    for(unsigned b=0;b<3;++b) {
        x2_session_t s;int16_t linear[160];size_t pos=0;
        x2_session_init(&s);s.stage=X2_J;s.marker_valid=1;
        s.rx_samples=80800;
        while(pos<sizeof(x2_courier_s_bar)) {
            size_t n=blocks[b];if(n>sizeof(x2_courier_s_bar)-pos)n=sizeof(x2_courier_s_bar)-pos;
            for(size_t i=0;i<n;++i)linear[i]=(int16_t)ulaw_linear(x2_courier_s_bar[pos+i]);
            x2_session_rx(&s,linear,n);pos+=n;
            if(s.s_bar_seen)break;
        }
        assert(s.s_bar_seen && s.rx_samples>=81980 && s.rx_samples<=82160);
    }
    /* A reversed carrier alone and sustained S without S-bar cannot release J. */
    for(unsigned mode=0;mode<2;++mode) {
        x2_session_t s;int16_t samples[160];x2_session_init(&s);s.stage=X2_J;s.marker_valid=1;
        for(unsigned n=0;n<1600;n+=160) {
            for(unsigned i=0;i<160;++i) {
                unsigned t=n+i;double value=1800*cos(2*PI*1920*t/8000);
                if(mode)value+=1200*cos(2*PI*320*t/8000)+1200*cos(2*PI*3520*t/8000);
                if(!mode && t>=800)value=-value;
                samples[i]=(int16_t)value;
            }
            x2_session_rx(&s,samples,160);
        }
        assert(!s.s_bar_seen);
    }
}
static unsigned mp_notifications,source_calls;
static x2_mp_t notified_mp;
static void record_received(void *unused,const x2_mp_t *mp)
{
    (void)unused;++mp_notifications;notified_mp=*mp;
}
static int source_bit(void *unused)
{
    (void)unused;return (source_calls++*13+7)&1;
}
static void record_test(void)
{
    x2_mp_t mp={{0x344,0x3fe,0,0x500},0};x2_mp_rx_t rx;uint8_t bits[104];
    x2_mp_encode(&mp,bits);x2_mp_rx_init(&rx,record_received,NULL);mp_notifications=0;
    for(unsigned j=0;j<104;++j)x2_mp_rx_bit(&rx,0,0,bits[j]);
    assert(mp_notifications==0);
    for(unsigned j=0;j<104;++j)x2_mp_rx_bit(&rx,0,0,bits[j]);
    assert(mp_notifications==1 && !memcmp(notified_mp.words,mp.words,8));
    /* Every protected single-bit corruption is rejected; the previous
     * accepted working parameters cannot be overwritten by staging bits. */
    for(unsigned i=0;i<102;++i) {
        x2_mp_rx_init(&rx,record_received,NULL);mp_notifications=0;bits[i]^=1;
        for(unsigned frame=0;frame<3;++frame)for(unsigned j=0;j<104;++j)x2_mp_rx_bit(&rx,0,0,bits[j]);
        assert(mp_notifications==0);bits[i]^=1;
        for(unsigned frame=0;frame<3;++frame)for(unsigned j=0;j<104;++j)x2_mp_rx_bit(&rx,0,0,bits[j]);
        assert(mp_notifications>0);
    }
    x2_pcm_config_t c,sentinel;memset(&sentinel,0x55,sizeof(sentinel));
    for(unsigned ceiling=1;ceiling<=15;++ceiling)for(unsigned bit=0;bit<15;++bit) {
        mp.words[0]=(uint16_t)((ceiling<<2)|(13<<6));c=sentinel;
        int rc=x2_short_record_config(&mp,1u<<bit,&c);
        assert(rc==(bit<ceiling?(int)bit+1:-1));
        if(rc<0)assert(!memcmp(&c,&sentinel,sizeof(c)));
    }
    for(unsigned v=0;v<sizeof(x2_short_vectors)/sizeof(x2_short_vectors[0]);++v) {
        const struct x2_short_vector *e=&x2_short_vectors[v];x2_pcm_state_t state={0};uint8_t octets[6];
        mp.words[0]=(uint16_t)((e->index<<2)|(13<<6));mp.words[3]=(uint16_t)(e->md<<8);
        assert(x2_short_record_config(&mp,0x7fff,&c)==(int)e->index);
        assert(x2_pcm_encode(&c,&state,e->bits,0,octets)==0);
        assert(!memcmp(octets,e->octets,6));
    }
    mp.words[0]=0x344;mp.words[3]=0x700;c=sentinel;
    assert(x2_short_record_config(&mp,0x7fff,&c)==-1 && !memcmp(&c,&sentinel,sizeof(c)));
    mp.words[3]=0x501;assert(x2_short_record_config(&mp,0x7fff,&c)==-1);
    mp.words[3]=0x500;mp.words[2]=1;assert(x2_short_record_config(&mp,0x7fff,&c)==-1);
    /* Line decoding, including filtering/acquisition, is tested independently
     * from the strict bit framer with real recorded MP samples. */
    const unsigned blocks[]={1,17,160};
    for(unsigned b=0;b<3;++b) {
        x2_mp_rx_init(&rx,record_received,NULL);mp_notifications=0;
        int16_t linear[160];size_t pos=0;
        while(pos<sizeof(x2_mp_capture)) {
            size_t n=blocks[b];if(n>sizeof(x2_mp_capture)-pos)n=sizeof(x2_mp_capture)-pos;
            for(size_t j=0;j<n;++j)linear[j]=(int16_t)ulaw_linear(x2_mp_capture[pos+j]);
            x2_mp_rx_audio(&rx,linear,n);pos+=n;
        }
        assert(rx.e_detected && rx.e_sample==11434);
        assert(mp_notifications>20 && notified_mp.words[0]==0x344
            && notified_mp.words[1]==0x3fe && !notified_mp.words[2] && notified_mp.words[3]==0x500);
    }
}
static void upstream_e_test(const char *path)
{
    x2_mp_rx_t rx;x2_mp_t mp={{0x344,0x3fe,0,0x500},0};uint8_t bits[104];
    assert(x2_mp_encode(&mp,bits)==0);
    x2_mp_rx_init(&rx,NULL,NULL);
    for(unsigned i=0;i<40;++i)x2_mp_rx_bit(&rx,0,0,1);
    assert(!rx.e_detected); /* No validated parameters. */
    for(unsigned frame=0;frame<3;++frame)for(unsigned i=0;i<104;++i)x2_mp_rx_bit(&rx,0,0,bits[i]);
    assert(!rx.e_detected); /* MP's sync cannot become E. */
    for(unsigned i=0;i<40;++i)x2_mp_rx_bit(&rx,1,0,1);
    assert(!rx.e_detected); /* Different timing hypothesis. */
    for(unsigned i=0;i<19;++i)x2_mp_rx_bit(&rx,0,0,1);
    assert(!rx.e_detected);
    x2_mp_rx_bit(&rx,0,0,0); /* Interrupted E restarts the counter. */
    /* Complete the staged false prefix with zeros, then requalify MP. */
    for(unsigned i=0;i<104;++i)x2_mp_rx_bit(&rx,0,0,0);
    for(unsigned frame=0;frame<3;++frame)for(unsigned i=0;i<104;++i)x2_mp_rx_bit(&rx,0,0,bits[i]);
    for(unsigned i=0;i<19;++i)x2_mp_rx_bit(&rx,0,0,1);
    assert(!rx.e_detected);x2_mp_rx_bit(&rx,0,0,1);assert(rx.e_detected);
    if(path) {
        const unsigned blocks[]={1,17,160};uint64_t first=0;
        for(unsigned b=0;b<3;++b) {
            FILE *f=fopen(path,"rb");uint8_t raw[160];int16_t linear[160];size_t n;
            assert(f);x2_mp_rx_init(&rx,NULL,NULL);
            while((n=fread(raw,1,blocks[b],f))!=0) {
                for(size_t i=0;i<n;++i)linear[i]=(int16_t)ulaw_linear(raw[i]);
                x2_mp_rx_audio(&rx,linear,n);
                if(rx.e_detected)break;
            }
            fclose(f);
            printf("upstream E: detected=%u sample=%llu timing=%u phase=%u block=%u\n",
                   rx.e_detected,(unsigned long long)rx.e_sample,rx.e_timing,rx.e_phase,blocks[b]);
            assert(rx.e_detected);
            if(b)assert(rx.e_sample==first);else first=rx.e_sample;
        }
    }
}
static void session_output(x2_session_t *s,uint8_t *out,unsigned count,unsigned block)
{
    uint8_t scratch[160];
    for(unsigned n=0;n<count;) {
        unsigned k=count-n;if(k>block)k=block;
        assert(x2_session_tx(s,out?out+n:scratch,k)==k);n+=k;
    }
}
static void source_activation_test(void)
{
    const unsigned blocks[]={1,17,160};
    /* Native CBB8/B54C framing: D284/7FFF/0000, CRC FB0C, 11 pad
     * zeros. The downstream record is three words, not upstream MP's four. */
    static const uint8_t record[]={0xff,0xff,0x11,0x4a,0xfb,0xff,3,0,0x80,0x61,0x1f,0};
    for(unsigned b=0;b<3;++b) {
        x2_session_t s;x2_mp_t mp={{0x344,0x3fe,0,0x500},0};uint8_t octets[160],records[72];
        x2_session_init(&s);s.marker_valid=1;
        /* Reach E through its real initializer, avoiding seeded mapper internals. */
        s.stage=X2_TRAIN_D;s.stage_samples=1151;x2_session_tx(&s,octets,1);
        x2_session_set_payload_source(&s,source_bit,NULL);source_calls=0;
        session_output(&s,NULL,10002,blocks[b]);
        assert(s.stage==X2_RECORD_WAIT && !source_calls);
        session_output(&s,octets,120,blocks[b]);
        assert(s.stage==X2_RECORD_WAIT && !source_calls);
        /* Waiting for MP must continue training, without inserting silence. */
        for(unsigned i=0;i<120;++i)assert(octets[i]!=0x7f);
        x2_session_receive_mp(&s,&mp);
        assert(s.mp_valid && s.selected_index==1 && s.upstream_rate_n==10);
        assert(s.peer_mp.words[1]==0x03fe && s.downstream_rate_mask==0x7fff);
        assert(s.data_config.banks[0][1]==0xa8a8); /* Data alphabet, not E's A7. */
        session_output(&s,octets,54,blocks[b]);
        assert(s.stage==X2_RECORD_TX && !source_calls);
        for(unsigned i=0;i<54;++i)
            assert(octets[i]==(uint8_t)(0xa5^((i%6>=3?0x80:0)^(i>=48?0x80:0))));
        session_output(&s,records,sizeof(records),blocks[b]);
        assert(s.stage==X2_RECORD_TX && !source_calls); /* MP alone cannot release it. */
        x2_pcm_state_t mapper={0};x2_scrambler_t scrambler;
        x2_scrambler_init(&scrambler,18,0);
        for(unsigned r=0;r<3;++r) {
            uint8_t decoded[12]={0};
            for(unsigned frame=0;frame<4;++frame) {
                uint64_t bits;
                assert(!x2_pcm_decode(&s.training_mapper.config,&mapper,
                                      records+r*24+frame*6,&bits));
                for(unsigned i=0;i<24;++i) {
                    unsigned p=frame*24+i;
                    decoded[p/8]|=(uint8_t)(x2_descramble_bit(&scrambler,(bits>>i)&1)<<(p%8));
                }
            }
            assert(!memcmp(decoded,record,sizeof(record)));
        }
        /* E is qualified by the same timing hypothesis and CRC-valid MP;
         * release at a complete record even when E arrives mid-frame. */
        x2_mp_rx_init(&s.mp_rx,NULL,NULL);uint8_t bits[104];x2_mp_encode(&mp,bits);
        for(unsigned r=0;r<3;++r)for(unsigned i=0;i<104;++i)x2_mp_rx_bit(&s.mp_rx,0,0,bits[i]);
        session_output(&s,NULL,7,blocks[b]);
        for(unsigned i=0;i<19;++i)x2_mp_rx_bit(&s.mp_rx,0,0,1);
        assert(!s.mp_rx.e_detected && s.stage==X2_RECORD_TX);
        x2_mp_rx_bit(&s.mp_rx,0,0,1);assert(s.mp_rx.e_detected);
        session_output(&s,NULL,16,blocks[b]);assert(s.stage==X2_RECORD_TX);
        session_output(&s,NULL,1,blocks[b]);assert(s.stage==X2_FINAL_TRAINING);
        session_output(&s,NULL,6,blocks[b]);assert(s.stage==X2_DATA_STARTUP && !source_calls);
        session_output(&s,NULL,4080,blocks[b]);assert(s.stage==X2_PAYLOAD && !source_calls);
        uint32_t history=s.training_mapper.scrambler.history;
        session_output(&s,NULL,6,blocks[b]);assert(source_calls==24);
        assert(history!=s.training_mapper.scrambler.history);
        /* Draft 0.33 section 20: host limit intersects W2 and N2, rather
         * than confusing the upstream rate with the PCM downstream index. */
        s.stage=X2_RECORD_WAIT;s.upstream_rate_mask=3;
        x2_session_receive_mp(&s,&mp);
        assert(s.mp_valid && s.upstream_rate_n==2 && s.selected_index==1);
        s.upstream_rate_mask=0;
        x2_session_receive_mp(&s,&mp);assert(s.stage==X2_FAILED);
        /* Malformed MP cannot overwrite accepted parameters. */
        s.stage=X2_RECORD_WAIT;mp.words[2]=1;
        x2_session_receive_mp(&s,&mp);assert(s.stage==X2_FAILED);
    }
}
static void capture_test(const char *path)
{
    FILE *f=fopen(path,"rb");x2_session_t s;uint8_t raw[160];int16_t linear[160];size_t count;
    assert(f);x2_session_init(&s);
    while((count=fread(raw,1,sizeof(raw),f))!=0){
        for(size_t i=0;i<count;++i)linear[i]=(int16_t)ulaw_linear(raw[i]);
        x2_session_rx(&s,linear,count);
        if(s.marker_valid)break;
    }
    fclose(f);
    printf("capture: INFO0=%04x marker=%02x at RX sample %llu\n",s.peer_capabilities,s.marker,(unsigned long long)s.rx_samples);
    assert(s.peer_info_valid && s.marker==0x4d && s.stage==X2_UPSTREAM_WAIT);
}
int main(int argc,char **argv)
{
    codec_test();info_recovery_test();phase2_test();training_test();s_bar_test();record_test();source_activation_test();upstream_e_test(argc>1?argv[1]:NULL);if(argc>1)capture_test(argv[1]);
    puts("x2 session: INFO codecs, marker semantics, received gates, 105 I-modem frames, recorded MP/E and payload activation passed");return 0;
}
