/* Offline PCM datapump measurements. V.90 §§8.6/9.4 downstream and
 * V.92 §§6.4/8.7.1 PCM upstream. No processing of live RTP codewords.
 * These are directed component tests, not a full V.92 call simulation. */
#include <spandsp.h>
#include "v90.h"
#include "v90_analogue_phase4.h"
#include "v92_upstream_rx.h"
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    int v92, alaw, drn, sr, bits, seed, chunk, delay, corrupt;
    double noise_rms;
} options_t;
typedef struct {
    const uint8_t *expected;
    uint64_t bits, errors;
    int target;
} checker_t;

static uint8_t next_bit(uint32_t *state)
{
    *state = *state*1664525U + 1013904223U;
    return (uint8_t)(*state >> 31);
}
static void check_bit(checker_t *c, int bit)
{
    if (c->bits < (uint64_t)c->target) {
        c->errors += bit != c->expected[c->bits];
        c->bits++;
    }
}
static void check_byte(void *ctx, uint8_t byte)
{
    checker_t *c = ctx;
    for (int i=0; i<8; i++) check_bit(c, (byte >> i)&1);
}
static void r_pattern(uint8_t *out, int reps, v90_law_t law, int reverse)
{
    for (int i=0; i<reps*6; i++)
        out[i] = v90_codeword_compose(law, 40, ((i%6)<3) != reverse);
}
/* V.90 Table 16 Type 0, acknowledged MP; CRC excludes each start bit. */
static int mp_bits(uint8_t *bits)
{
    memset(bits,0,102);
    memset(bits,1,17);
    for (int i=0;i<4;i++) bits[24+i]=(13>>i)&1;
    bits[33]=1;
    memset(bits+36,1,13);
    uint16_t crc=0xffff;
    for (int start=17;start<68;start+=17)
        for (int i=start+1;i<=start+16;i++) crc=crc_itu16_bits(bits[i],1,crc);
    for (int i=0;i<16;i++) bits[69+i]=(crc>>i)&1;
    return 86;
}

/* Exercise the production data-frame entry point continuously across B1d
 * and payload; the receiver owns a separate scrambler and frame state. */
static int data_codewords(v90_law_t law, const vpcm_cp_frame_t *cpt,
                         const vpcm_cp_frame_t *cp, const uint8_t *bits,
                         int frames, uint8_t *wire)
{
    int total_bits=frames*(cp->drn+20), bytes=(total_bits+7)/8;
    uint8_t *packed=calloc((size_t)bytes,1);
    v90_state_t *tx=v90_init_data_pump(law);
    if (!packed || !tx) {free(packed);v90_free(tx);return 0;}
    for(int i=0;i<total_bits;i++)packed[i/8]|=(uint8_t)(bits[i]<<(i%8));
    int ok=v90_set_phase4_cp(tx,cpt) && v90_set_phase4_cp(tx,cp);
    v90_reset_data_mode(tx);
    ok=ok && v90_data_bits_per_frame(tx)==cp->drn+20;
    int consumed=0;
    for(int frame=0;ok && frame<frames;frame++) {
        int used=0;
        ok=v90_tx_data_frame_codewords(tx,wire+frame*6,packed+consumed,
                    bytes-consumed,&used,false)==6;
        consumed+=used;
    }
    ok=ok && consumed==bytes;
    free(packed);v90_free(tx);return ok;
}

static int downstream(const options_t *o, checker_t *check, int *startup,
                      uint64_t *rejects, int *b1_errors)
{
    v90_law_t law=o->alaw?V90_LAW_ALAW:V90_LAW_ULAW;
    vpcm_cp_frame_t cpt, cp;
    vpcm_cp_init(&cpt);
    cpt.v90_compatibility=false;
    cpt.codec_alaw=o->alaw;
    cpt.drn=12;
    cpt.shaping_redundancy=o->sr;
    cpt.constellation_count=1;
    cpt.upstream_rate_mask=0x1fff;
    vpcm_cp_enable_all_ucodes(cpt.masks[0]);
    cp=cpt; cp.v90_compatibility=true; cp.drn=o->drn;
    int d=cp.drn+20, td=cpt.drn+8;
    uint8_t mp[102];
    int mn=mp_bits(mp), tf=400+(mn+td-1)/td+2;
    int df=(o->bits+d-1)/d;
    int count=o->delay+216+tf*6+(48+df)*6;
    uint8_t *wire=calloc((size_t)count,1);
    uint8_t *training=calloc((size_t)tf*td,1);
    uint8_t *payload=malloc((size_t)(48+df)*d);
    if (!wire || !training || !payload) {
        free(wire);free(training);free(payload);return 2;
    }
    memset(wire,o->alaw?0xd5:0xff,(size_t)o->delay);
    r_pattern(wire+o->delay,32,law,0);
    r_pattern(wire+o->delay+192,4,law,1);
    memset(training,1,(size_t)400*td);
    memcpy(training+400*td,mp,(size_t)mn);
    memset(payload,1,(size_t)48*d);
    memcpy(payload+48*d,check->expected,(size_t)o->bits);
    memset(payload+48*d+o->bits,0,(size_t)(df*d-o->bits));
    if (o->corrupt>=count) {free(wire);free(training);free(payload);return 2;}
    v90_shaped_rx_state_t zero={0};
    int ok=v90_generate_phase4_codewords(law,&cpt,&zero,training,tf,
                    wire+o->delay+216,tf*6)==tf*6
        && data_codewords(law,&cpt,&cp,payload,48+df,wire+o->delay+216+tf*6);
    v90_analogue_phase4_config_t cfg={.law=law,.u_info=48,.cpt=cpt,.cp=cp};
    v90_analogue_phase4_t *rx=ok?v90_analogue_phase4_init(&cfg):NULL;
    if (!rx) { free(wire);free(training);free(payload);return 2; }
    /* Deliberate wire corruption is a negative-test control, not a channel model. */
    if (o->corrupt>=0 && o->corrupt<count) wire[o->corrupt]^=0x80;
    uint8_t bits[8192];
    unsigned events=0;
    for (int pos=0;pos<count;) {
        int n=count-pos; if (n>o->chunk)n=o->chunk;
        events|=v90_analogue_phase4_put(rx,wire+pos,n);
        int got;
        while ((got=v90_analogue_phase4_get_data_bits(rx,bits,sizeof(bits)))>0)
            for (int i=0;i<got;i++) check_bit(check,bits[i]);
        pos+=n;
    }
    *startup=(events&V90A4_RX_EVENT_DATA)!=0
        && v90_analogue_phase4_mp_frames(rx)>0
        && v90_analogue_phase4_b1d_frames(rx)==48;
    *b1_errors=v90_analogue_phase4_b1d_bit_errors(rx);
    *rejects=(uint64_t)v90_analogue_phase4_demap_failures(rx);
    v90_analogue_phase4_free(rx);
    free(wire);free(training);free(payload);
    return 0;
}

/* Synthetic Table 30 profile: full 64-point set and power-of-two moduli.
 * Tests §6.4 coding and §8.7.1 acquisition, not line-profile negotiation. */
static void upstream_profile(v92_cpd_frame_t *cpd, int drn)
{
    memset(cpd,0,sizeof(*cpd));
    cpd->modulus_present=true; cpd->constellations_present=true;
    cpd->selected_upstream_drn=(uint8_t)drn;
    cpd->trellis_select=0;
    cpd->gain_q0_16=0xffff;
    int k=v92_upstream_bits_per_frame((uint8_t)drn);
    for (int i=0;i<12;i++) cpd->moduli[i]=(uint8_t)(1U<<(k/12+(i<k%12)));
    cpd->set_sizes[0]=64;
    for (int i=0;i<64;i++) cpd->points[0][i]=(uint16_t)(2*i+1);
}
static int upstream(const options_t *o, checker_t *check, int *startup,
                    uint64_t *rejects, uint64_t *clips, double *correlation)
{
    v92_cpd_frame_t cpd;
    upstream_profile(&cpd,o->drn);
    int k=v92_upstream_bits_per_frame((uint8_t)o->drn);
    /* Complete bytes are delivered; pad the last frame, grade exactly --bits. */
    int frames=(o->bits+7+k-1)/k;
    int count=o->delay+(V92_B1U_FRAMES+frames)*12+V92_UPSTREAM_EQ_TAPS;
    if (o->corrupt>=count)return 2;
    int16_t *samples=calloc((size_t)count,sizeof(*samples));
    v92_upstream_rx_t *rx=calloc(1,sizeof(*rx));
    if (!samples || !rx) {free(samples);free(rx);return 2;}
    v92_upstream_wave_tx_t tx;
    v92_upstream_wave_tx_init(&tx);
    awgn_state_t *noise=o->noise_rms>0?awgn_init_dbov(NULL,o->seed,20*log10(o->noise_rms/32768.0)):NULL;
    if (o->noise_rms>0 && !noise) {free(samples);free(rx);return 2;}
    uint8_t bits[72]; double wave[12];
    int pos=o->delay;
    for (int f=0;f<V92_B1U_FRAMES+frames;f++) {
        for (int i=0;i<k;i++) {
            int index=(f-V92_B1U_FRAMES)*k+i;
            bits[i]=f<V92_B1U_FRAMES?1:index<o->bits?check->expected[index]:0;
        }
        if (!v92_upstream_wave_encode_frame(&tx,&cpd,bits,k,wave)) {
            if(noise)awgn_free(noise);free(samples);free(rx);return 2;
        }
        for (int i=0;i<12;i++) {
            /* The analog TX waveform is sampled once by the simulated network
             * A/D. G.711 bytes are then decoded by the digital receiver. */
            double value=120.0*wave[i]+(noise?awgn(noise):0);
            if (value>32767) {value=32767;(*clips)++;}
            if (value< -32768) {value= -32768;(*clips)++;}
            int16_t linear=(int16_t)lrint(value);
            uint8_t cw=o->alaw?linear_to_alaw(linear):linear_to_ulaw(linear);
            if (pos==o->corrupt) cw^=0x80;
            samples[pos++]=o->alaw?alaw_to_linear(cw):ulaw_to_linear(cw);
        }
    }
    if (noise)awgn_free(noise);
    if (!v92_upstream_b1_rx_init(rx,&cpd,check_byte,check)) {free(samples);free(rx);return 2;}
    for (pos=0;pos<count;) {
        int n=count-pos;if(n>o->chunk)n=o->chunk;
        v92_upstream_b1_rx_feed(rx,samples+pos,n);pos+=n;
    }
    *startup=rx->locked && rx->equalizer_trained;
    *rejects=rx->rejected_frames;
    *correlation=rx->correlation;
    free(samples);free(rx);return 0;
}
static int integer(const char *s, int low, int high, int *out)
{
    char *end;errno=0;long n=strtol(s,&end,10);
    if(errno || end==s || *end || n<low || n>high)return 0;
    *out=(int)n;return 1;
}
int main(int argc,char **argv)
{
    options_t o={.drn=9,.bits=16000,.seed=1,.chunk=37,.corrupt=-1};
    for(int i=1;i<argc;i++) {
        const char *key=argv[i];
        if(!strcmp(key,"--help")) {
            puts("Usage: pcm_ber_test --mode v90-downstream|v92-upstream [options]\n"
                 "  --law ulaw|alaw --drn N --bits N --seed N --chunk N --delay N\n"
                 "  --sr 0..3 (V.90) --noise-rms N (V.92 analog A/D input)\n"
                 "  --corrupt-sample N (negative control; flips a G.711 sign bit)\n"
                 "JSON stdout. Exit 0: exact payload and startup; 1: measurement failure; 2: invalid setup.");return 0;
        }
        if(++i==argc)return 2;
        const char *v=argv[i];
        if(!strcmp(key,"--mode")) {
            if(!strcmp(v,"v90-downstream"))o.v92=0;
            else if(!strcmp(v,"v92-upstream"))o.v92=1;
            else return 2;
        } else if(!strcmp(key,"--law")) {
            if(strcmp(v,"ulaw") && strcmp(v,"alaw"))return 2;
            o.alaw=!strcmp(v,"alaw");
        } else if(!strcmp(key,"--drn")) {if(!integer(v,1,22,&o.drn))return 2;}
        else if(!strcmp(key,"--bits")) {if(!integer(v,1,10000000,&o.bits))return 2;}
        else if(!strcmp(key,"--seed")) {if(!integer(v,1,1000000,&o.seed))return 2;}
        else if(!strcmp(key,"--chunk")) {if(!integer(v,1,160,&o.chunk))return 2;}
        else if(!strcmp(key,"--delay")) {if(!integer(v,0,8192,&o.delay))return 2;}
        else if(!strcmp(key,"--sr")) {if(!integer(v,0,3,&o.sr))return 2;}
        else if(!strcmp(key,"--corrupt-sample")) {if(!integer(v,0,20000000,&o.corrupt))return 2;}
        else if(!strcmp(key,"--noise-rms")) {
            char *end;errno=0;o.noise_rms=strtod(v,&end);
            if(errno || end==v || *end || !isfinite(o.noise_rms) || o.noise_rms<0 || o.noise_rms>10000)return 2;
        } else return 2;
    }
    if((o.v92 && (o.drn>19 || o.sr)) || (!o.v92 && o.noise_rms))return 2;
    uint8_t *expected=malloc((size_t)o.bits);
    if(!expected)return 2;
    uint32_t state=(uint32_t)o.seed;
    for(int i=0;i<o.bits;i++)expected[i]=next_bit(&state);
    checker_t check={.expected=expected,.target=o.bits};
    int startup=0,b1_errors=0;uint64_t rejects=0,clips=0;double corr=0;
    int error=o.v92?upstream(&o,&check,&startup,&rejects,&clips,&corr)
                   :downstream(&o,&check,&startup,&rejects,&b1_errors);
    const char *status=error?"harness_error":!startup?"no_acquisition":check.bits!=(uint64_t)o.bits?"short_payload"
                      :check.errors || rejects || b1_errors?"errors":"pass";
    printf("{\"full_call\":false,\"rate_source\":\"configured_profile\",\"scope\":\"%s\",",
           o.v92?"b1u-and-pcm-data":"phase4-and-pcm-data");
    printf("\"status\":\"%s\",\"mode\":\"%s\",\"law\":\"%s\",\"drn\":%d,\"rate_bps\":%.6f,"
           "\"seed\":%d,\"chunk_samples\":%d,\"delay_samples\":%d,\"sr\":%d,\"noise_rms\":%.6f,"
           "\"corrupt_sample\":%d,\"target_bits\":%d,\"checked_bits\":%llu,\"bit_errors\":%llu,"
           "\"startup_complete\":%s,\"rejected_frames\":%llu,\"b1_bit_errors\":%d,\"clips\":%llu,\"b1_correlation\":%.9f}\n",
           status,o.v92?"v92-upstream":"v90-downstream",o.alaw?"alaw":"ulaw",o.drn,
           (o.drn+(o.v92?17:20))*8000.0/6.0,o.seed,o.chunk,o.delay,o.sr,o.noise_rms,o.corrupt,
           o.bits,(unsigned long long)check.bits,(unsigned long long)check.errors,startup?"true":"false",
           (unsigned long long)rejects,b1_errors,(unsigned long long)clips,corr);
    free(expected);return error?2:strcmp(status,"pass")?1:0;
}
