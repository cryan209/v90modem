/* x2 analogue client against our own x2 server session.
 *
 * Downstream: the server's x2_session_tx octets drive x2_client_rx_codewords
 * from stretch B to payload, at several PCM ceilings; the client must find
 * the server J, the record alignment, the record (index and upstream N), the
 * final training frame and then the server's payload bits exactly.
 *
 * Upstream: the client's Phase 4 symbols (16-point TRN, MP, MP', E) through
 * SpanDSP's V.34 modulator must give the server's x2_mp_rx a CRC-valid MP
 * equal to the client's and then E (Draft 0.33 clauses 20, 21). */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <spandsp.h>
#include "x2_session.h"

#define CHECK(c) do { if(!(c)){fprintf(stderr,"%s:%d: %s\n",__FILE__,__LINE__,#c);exit(1);} } while(0)

typedef struct { uint32_t lfsr; uint8_t *bits; size_t count, max; } source_t;
static int source_bit(void *context)
{
    source_t *s=context;
    unsigned bit=(s->lfsr^(s->lfsr>>2)^(s->lfsr>>3)^(s->lfsr>>5))&1;
    s->lfsr=(s->lfsr>>1)|(bit<<15);
    if(s->count<s->max)s->bits[s->count++]=(uint8_t)bit;
    return (int)bit;
}
typedef struct { uint8_t *bits; size_t count, max; } sink_t;
static void sink_bit(void *context, unsigned bit)
{
    sink_t *s=context;
    if(s->count<s->max)s->bits[s->count++]=(uint8_t)bit;
}

static void downstream(uint16_t pcm_mask, unsigned expected_index, unsigned expected_n, unsigned no_align)
{
    static x2_session_t server;
    static x2_client_t client;
    source_t source={0xace1,malloc(1<<20),0,1<<20};
    sink_t sink={malloc(1<<20),0,1<<20};
    CHECK(!x2_session_init(&server));
    CHECK(!x2_client_init(&client,pcm_mask,0x1ffe,5));
    x2_client_set_put_bit(&client,sink_bit,&sink);
    x2_session_set_payload_source(&server,source_bit,&source);
    /* The server has accepted our marker and heard our Phase 3 J. */
    server.stage=X2_UPSTREAM_WAIT;
    x2_session_upstream_j(&server);
    unsigned mp_given=0,payload_samples=0;
    for(unsigned n=0;n<200000 && payload_samples<48000;++n) {
        uint8_t code;
        x2_session_stage_t before=server.stage;
        CHECK(x2_session_tx(&server,&code,1)==1);
        /* Ie030002 sends no alignment word: withhold ours, and the client
         * must find the record's frame phase itself. */
        if(!(no_align && before==X2_RECORD_ALIGN))x2_client_rx_codewords(&client,&code,1);
        /* Our S-to-S-bar answer to its J word. */
        if(client.j_seen && server.stage==X2_J)x2_session_upstream_s_bar(&server);
        if(!mp_given && server.stage==X2_TRAIN_C){x2_session_receive_mp(&server,&client.mp);mp_given=1;}
        /* Our acknowledged MP and E, as the server's MP receiver sees them. */
        if(client.record_valid && !client.e_sent){client.e_sent=1;server.mp_rx.e_detected=1;}
        if(server.stage==X2_PAYLOAD)++payload_samples;
        CHECK(server.stage!=X2_FAILED && client.dn!=X2C_DN_FAILED);
    }
    /* Finish the frame whose bits the server has already taken. */
    while(server.training_mapper.output_position!=6) {
        uint8_t code;
        CHECK(x2_session_tx(&server,&code,1)==1);
        x2_client_rx_codewords(&client,&code,1);
    }
    CHECK(client.j_seen && client.record_valid && client.final_seen && client.dn==X2C_DN_DATA);
    CHECK(client.selected_index==expected_index && server.selected_index==expected_index);
    CHECK(client.upstream_n==expected_n && server.upstream_rate_n==expected_n);
    CHECK(client.decode_errors==0);
    CHECK(x2_client_downstream_rate(&client)==x2_pcm_frame_bits(&server.data_config)*8000u/6u);
    /* Everything before the server's payload is scrambled ones (its
     * DATA_STARTUP); then exactly the server's payload bits. */
    CHECK(source.count>1000 && sink.count>=source.count);
    size_t offset=sink.count-source.count;
    for(size_t i=0;i<offset;++i)CHECK(sink.bits[i]==1);
    for(size_t i=0;i<source.count;++i)CHECK(sink.bits[offset+i]==source.bits[i]);
    printf("  index %2u (%u bit/s down, upstream N=%u)%s: %zu payload bits exact after %zu ones\n",
           expected_index,x2_client_downstream_rate(&client),expected_n,
           no_align?", no alignment word":"",source.count,offset);
    free(source.bits);free(sink.bits);
}

static int ones_bit(void *unused) { (void)unused; return 1; }
static x2_mp_t received_mp;
static unsigned received_count;
static void mp_received(void *context, const x2_mp_t *mp)
{
    (void)context;received_mp=*mp;++received_count;
}
static void upstream(void)
{
    static x2_client_t client;
    static x2_mp_rx_t rx;
    CHECK(!x2_client_init(&client,0x7fff,0x1ffe,5));
    CHECK(client.mp.words[0]==((15u<<2)|(13u<<6)) && client.mp.words[1]==0x1ffe && client.mp.words[3]==0x0500);
    v34_state_t *v34=v34_init(NULL,3200,31200,true,true,ones_bit,NULL,NULL,NULL);
    CHECK(v34);
    v34_tx_power(v34,-10.0f);
    /* Straight into Phase 4: S, S-bar, then 16-point TRN and MP. */
    client.up=X2C_UP_S2;client.j_seen=1;
    CHECK(!v34_tx_start_external_symbols(v34,4,1,x2_client_get_symbol,&client));
    x2_mp_rx_init(&rx,mp_received,NULL);
    int16_t audio[160];
    unsigned acked=0;
    for(unsigned block=0;block<2000 && !rx.e_detected;++block) {
        int n=v34_tx(v34,audio,160);
        for(int i=n;i<160;++i)audio[i]=0;
        /* Through the G.711 bearer, as the server receives it. */
        for(int i=0;i<160;++i)audio[i]=ulaw_to_linear(linear_to_ulaw(audio[i]));
        x2_mp_rx_audio(&rx,audio,160);
        if(received_count && !client.record_valid) {
            CHECK(!memcmp(received_mp.words,client.mp.words,sizeof(received_mp.words)));
            client.record_valid=1;client.ack_pending=1;client.upstream_n=13;
            /* The engine's handover: B1/data from the symbol after E. */
            CHECK(!v34_v90_arm_tx_data(v34,13,0,0,0,NULL,x2_client_data_due));
        }
        if(received_count && (received_mp.words[0]&0x8000))acked=1;
    }
    CHECK(received_count>=2 && acked && rx.e_detected);
    CHECK(client.up==X2C_UP_DATA && client.e_sent);
    for(unsigned block=0;block<4;++block)v34_tx(v34,audio,160);
    CHECK(v34_get_tx_data_mode(v34));
    printf("  upstream: MP %04x/%04x/%04x/%04x, acknowledged, E detected (%u valid frames)\n",
           received_mp.words[0]&0x7fff,received_mp.words[1],received_mp.words[2],received_mp.words[3],
           rx.valid_frames);
    v34_free(v34);
}

/* An in-data rate change opened by the client (x2_client_start_rate_change),
 * against our server session in lockstep: our upstream through SpanDSP's
 * modulator and the G.711 bearer into x2_session_rx (its S detector, the
 * 4-point MP receiver and E), its octets into the client. Both sides must
 * land on the new index, the alignment word must not reach the DTE, and
 * the payload must be exact across the change apart from the server's
 * DATA_STARTUP ones. */
static void rate_change(uint16_t pcm_mask, unsigned start_index, unsigned new_index)
{
    static x2_session_t server;
    static x2_client_t client;
    source_t source={0x1d0f,malloc(1<<21),0,1<<21};
    sink_t sink={malloc(1<<21),0,1<<21};
    CHECK(!x2_session_init(&server));
    CHECK(!x2_client_init(&client,pcm_mask,0x1ffe,5));
    x2_client_set_put_bit(&client,sink_bit,&sink);
    x2_session_set_payload_source(&server,source_bit,&source);
    server.stage=X2_UPSTREAM_WAIT;server.marker_valid=1;
    x2_session_upstream_j(&server);
    v34_state_t *v34=v34_init(NULL,3200,31200,true,true,ones_bit,NULL,NULL,NULL);
    CHECK(v34);
    v34_tx_power(v34,-10.0f);
    client.up=X2C_UP_DATA;
    CHECK(!v34_tx_start_external_symbols(v34,4,1,x2_client_get_symbol,&client));
    unsigned mp_given=0,payload_blocks=0,changed=0,armed=0,after=0;
    for(unsigned block=0;block<4000 && after<200;++block) {
        uint8_t codes[160];int16_t audio[160];
        CHECK(x2_session_tx(&server,codes,160)==160);
        x2_client_rx_codewords(&client,codes,160);
        if(client.j_seen && server.stage==X2_J)x2_session_upstream_s_bar(&server);
        if(!mp_given && server.stage==X2_TRAIN_C){x2_session_receive_mp(&server,&client.mp);mp_given=1;}
        if(!changed && client.record_valid && !client.e_sent){client.e_sent=1;server.mp_rx.e_detected=1;}
        int n=v34_tx(v34,audio,160);
        for(int i=n<0?0:n;i<160;++i)audio[i]=0;
        for(int i=0;i<160;++i)audio[i]=ulaw_to_linear(linear_to_ulaw(audio[i]));
        x2_session_rx(&server,audio,160);
        if(server.stage==X2_PAYLOAD && !changed && ++payload_blocks==100) {
            CHECK(client.dn==X2C_DN_DATA && client.selected_index==start_index);
            CHECK(!x2_client_start_rate_change(&client,new_index));
            CHECK(!v34_v90_resume_external_symbols(v34,x2_client_get_symbol,&client));
            changed=1;
        }
        if(changed && client.record_valid && !armed) {
            CHECK(!v34_v90_arm_tx_data(v34,(int)client.upstream_n,0,0,0,NULL,x2_client_data_due));
            armed=1;
        }
        if(changed && !client.rr && server.stage==X2_PAYLOAD && !x2_session_renegotiating(&server))++after;
        CHECK(server.stage!=X2_FAILED && client.dn!=X2C_DN_FAILED);
    }
    CHECK(changed && after>=200);
    CHECK(client.rate_changes==1 && client.final_seen && client.dn==X2C_DN_DATA);
    CHECK(client.selected_index==new_index && server.selected_index==new_index);
    CHECK(client.upstream_n==server.upstream_rate_n);
    CHECK(server.renegotiations==1 && v34_get_tx_data_mode(v34));
    while(server.training_mapper.output_position!=6) {
        uint8_t code;
        CHECK(x2_session_tx(&server,&code,1)==1);
        x2_client_rx_codewords(&client,&code,1);
    }
    /* sink = ones, the payload up to the change, ones (DATA_STARTUP), the rest. */
    CHECK(sink.count>=source.count);
    size_t lead=sink.count-source.count,i=0;
    size_t run=0;
    while(run<lead && sink.bits[run]==1)++run;
    /* The leading ones and the inserted ones are both all ones: find the
     * split by matching the payload from the front. */
    size_t extra=lead;
    size_t start=0;
    for(start=0;start<=lead;++start) {
        for(i=0;i<source.count && sink.bits[start+i]==source.bits[i];++i);
        if(i>=64)break;
    }
    CHECK(start<=lead);
    extra=lead-start;
    for(size_t j=0;j<extra;++j)CHECK(sink.bits[start+i+j]==1);
    for(size_t j=i;j<source.count;++j)CHECK(sink.bits[start+extra+j]==source.bits[j]);
    /* The DATA_STARTUP ones are dropped: nothing is inserted. */
    CHECK(extra==0);
    printf("  rate change %u -> %u (%u bit/s down, upstream N=%u): %zu payload bits exact, %zu before the change, %zu ones inserted\n",
           start_index,new_index,x2_client_downstream_rate(&client),client.upstream_n,source.count,i,extra);
    v34_free(v34);free(source.bits);free(sink.bits);
}

/* V.90 9.5.1.1: the server's retrain (70 ms of silence, then Tone B) is
 * recognised within 40 ms of the tone -- synthetic, and the original
 * Ie030002's own (reference-data/x2-imodem-retrain) -- while x2 data,
 * Tone A (2400 Hz, the analogue side's) and a quiet tone are not. */
static void feed_tone(x2_client_t *client, double f, double a, unsigned max, unsigned *n)
{
    CHECK(!x2_client_init(client,0x7fff,0x1ffe,5));
    x2_short_profile(15,5,&client->data);client->dn=X2C_DN_DATA;
    for(*n=0;*n<max && !client->tone_b;++*n) {
        uint8_t code=linear_to_ulaw((int)lrint(a*cos(2*M_PI*f*(*n)/8000)));
        x2_client_rx_codewords(client,&code,1);
    }
}
static void tone_b(void)
{
    static x2_client_t client;
    unsigned n;
    feed_tone(&client,1200,3000,800,&n);CHECK(client.tone_b && n<=400);
    feed_tone(&client,2400,3000,800,&n);CHECK(!client.tone_b);
    feed_tone(&client,1200,150,800,&n);CHECK(!client.tone_b);
    /* The I-modem's: 71 ms of silence from sample 88, then Tone B. */
    FILE *f=fopen("reference-data/x2-imodem-retrain/silence-tone-b-ulaw.g711","rb");
    CHECK(f);
    uint8_t buf[4000];
    CHECK(fread(buf,1,sizeof(buf),f)==sizeof(buf));
    fclose(f);
    CHECK(!x2_client_init(&client,0x7fff,0x1ffe,5));
    x2_short_profile(15,5,&client.data);client.dn=X2C_DN_DATA;
    for(n=0;n<sizeof(buf) && !client.tone_b;++n)x2_client_rx_codewords(&client,&buf[n],1);
    CHECK(client.tone_b && n>=88+568 && n<=88+568+400);
    /* A minute's worth of data codewords from the index-15 banks. */
    CHECK(!x2_client_init(&client,0x7fff,0x1ffe,5));
    x2_short_profile(15,5,&client.data);client.dn=X2C_DN_DATA;
    uint32_t lfsr=0xace1;
    for(unsigned k=0;k<480000;++k) {
        lfsr=lfsr*1103515245u+12345u;
        unsigned p=k%6,size=client.data.sizes[p];
        uint8_t code=(uint8_t)(client.data.banks[p][(lfsr>>16)%size]&0xff);
        if((lfsr>>8)&1)code^=0x80;
        x2_client_rx_codewords(&client,&code,1);
    }
    CHECK(!client.tone_b);
    printf("  retrain in data: Tone B recognised, synthetic and Ie030002's own; Tone A, a quiet tone and 60 s of data not\n");
}

int main(void)
{
    downstream(0x0001,1,13,0);
    downstream(0x003f,6,13,0);
    downstream(0x0fff,12,13,0);
    downstream(0x7fff,15,13,0);
    downstream(0x0001,1,13,1);
    downstream(0x7fff,15,13,1);
    upstream();
    rate_change(0x7fff,15,6);
    rate_change(0x7fff,15,1);
    rate_change(0x003f,6,6);
    tone_b();
    printf("x2 client: downstream record/final/payload at four ceilings, upstream MP/E and in-data rate changes passed\n");
    return 0;
}
