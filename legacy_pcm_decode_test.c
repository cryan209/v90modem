#include "legacy_pcm_decode.h"
#include "x2_session.h"
#include "x2_mp_capture.h"
#include "k56flex_client.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct { unsigned info, digital, marker, mp, e, probe, param; int e_sample, rate; } results_t;
static void event(void *user, int sample, const char *protocol, const char *summary, const char *detail)
{
    results_t *r = user;
    assert(sample >= 0);
    if (!strcmp(protocol, "x2")) {
        if (!strcmp(summary, "Peer INFO0 decoded")) { r->info++; assert(strstr(detail,"crc=valid")); }
        if (!strcmp(summary, "Directional marker decoded")) { r->marker++; assert(strstr(detail,"marker=4d")); }
        if (!strcmp(summary, "MP qualified")) { r->mp++; assert(strstr(detail,"0344/03fe/0000/0500")); }
        if (!strcmp(summary, "Upstream E detected")) { r->e++; r->e_sample = sample; }
    }
    if (!strcmp(protocol,"Diagnostic") && !strcmp(summary,"x2-compatible digital INFO0")) { r->digital++; assert(strstr(detail,"body=13dff")); }
    if (!strcmp(protocol,"K56flex")) {
        if (!strcmp(summary,"P1 probe acquired")) r->probe++;
        if (!strcmp(summary,"Parameter record decoded")) {
            const char *rate = strstr(detail,"record_rate_bps=");
            assert(strstr(detail,"checksum=valid") && rate);
            r->param++; r->rate=atoi(rate+16);
        }
    }
}
static int16_t ulaw(uint8_t octet)
{
    unsigned u=(unsigned)(~octet)&255;
    int t=(((u&15)<<3)+132)<<((u>>4)&7);
    return (int16_t)((u&128)?132-t:t-132);
}
static unsigned info_frame(int16_t *out, unsigned body, unsigned n, unsigned start, unsigned carrier)
{
    uint8_t bits[49]; unsigned clock=0, position=0, sign=0, k=0;
    unsigned count=(unsigned)x2_info_encode(body,n,bits);
    while(position<count) {
        if(clock<600 && bits[position])sign^=1;
        out[k]=(int16_t)((sign?-3000:3000)*cos(2*3.14159265358979323846*carrier*(start+k)/8000));
        k++;clock+=600;if(clock>=8000){clock-=8000;position++;}
    }
    return k;
}
static void x2_tests(void)
{
    results_t r={0};
    int16_t *audio=calloc(160+sizeof(x2_mp_capture),sizeof(*audio));
    assert(audio);
    for(size_t n=0;n<sizeof(x2_mp_capture);n++)audio[160+n]=ulaw(x2_mp_capture[n]);
    assert(!legacy_pcm_decode_x2(audio,160+sizeof(x2_mp_capture),event,&r));
    assert(r.mp==1 && r.e==1 && r.e_sample==160+11434);
    free(audio);
    int16_t info[4096]={0};unsigned pos=160;
    pos+=info_frame(info+pos,0x21ff,17,pos,1200);pos+=160;
    pos+=info_frame(info+pos,0x4d,7,pos,1200);pos+=160;
    memset(&r,0,sizeof(r));
    assert(!legacy_pcm_decode_x2(info,pos,event,&r));
    assert(r.info==1 && r.marker==1 && !r.mp && !r.e);
    /* The same INFO0 appears on ordinary V.34 calls: without a marker
     * it must remain diagnostic, even with a valid CRC. */
    memset(info,0,sizeof(info));pos=160;
    pos+=info_frame(info+pos,0x21ff,17,pos,1200);pos+=160;
    memset(&r,0,sizeof(r));
    assert(!legacy_pcm_decode_x2(info,pos,event,&r));
    assert(!r.info && !r.marker && !r.mp && !r.e);
    memset(info,0,sizeof(info));pos=160;
    pos+=info_frame(info+pos,0x13dff,17,pos,2400);pos+=160;
    pos+=info_frame(info+pos,0x13dff,17,pos,2400);pos+=160;
    memset(&r,0,sizeof(r));
    assert(!legacy_pcm_decode_x2(info,pos,event,&r));
    assert(r.digital==1 && !r.marker && !r.mp && !r.e);
    puts("PASS: x2 INFO0/marker and captured qualified MP/E with offset");
}
static void status(void *user,unsigned bits){k56flex_train_status(user,bits);}
static void report(void *user,unsigned bits){k56flex_train_set_report(user,bits);}
static void flex_test(k56flex_law_t law,int rate)
{
    k56flex_train_t server;k56flex_train_cfg_t tc={0};k56flex_client_cfg_t cc={0};
    k56flex_client_t *peer;results_t r={0};
    int16_t *audio=calloc(64000,sizeof(*audio));unsigned pos=800,guard=0;
    tc.law=law;tc.rate_bps=rate;tc.training_word=0xffff;
    tc.param.extra=5;tc.param.control=0x123;tc.param.u=1;
    assert(!k56flex_train_init(&server,&tc));
    cc.law=law;cc.report_word=0x8880;cc.gate_b_pairs=2;
    cc.ctl.status=status;cc.ctl.report=report;cc.ctl.user=&server;
    peer=k56flex_client_new(&cc);assert(peer&&audio);
    /* Generate a dialogue with the decision-level peer, then throw away
     * its state. The offline receiver gets ONLY the recorded linear audio. */
    while(k56flex_train_phase(&server)<K56T_PRIME && guard++<12000) {
        uint8_t buf[6];size_t n=k56flex_train_g711(&server,buf,6);
        assert(n==6 && pos+n<=64000);
        for(size_t j=0;j<n;j++)audio[pos++]=(int16_t)k56flex_level_from_g711(law,buf[j]);
        k56flex_client_rx(peer,buf,n);assert(!peer->failed);
    }
    assert(k56flex_train_phase(&server)==K56T_PRIME);
    k56flex_client_free(peer);
    assert(!legacy_pcm_decode_flex(audio,pos,law==K56FLEX_LAW_A,event,&r));
    printf("flex %s %d: probes=%u params=%u decoded_rate=%d\n",law?"alaw":"ulaw",rate,r.probe,r.param,r.rate);
    assert(r.probe==1 && r.param==1 && r.rate==rate);
    free(audio);
}
int main(void)
{
    int16_t quiet[8000]={0};results_t r={0};
    assert(!legacy_pcm_decode_x2(quiet,8000,event,&r));
    assert(!legacy_pcm_decode_flex(quiet,8000,0,event,&r));
    assert(!r.info&&!r.digital&&!r.marker&&!r.mp&&!r.e&&!r.probe&&!r.param);
    x2_tests();flex_test(K56FLEX_LAW_MU,56000);flex_test(K56FLEX_LAW_A,32000);
    puts("legacy_pcm_decode_test: OK");return 0;
}
