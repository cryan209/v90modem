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
    /* V.90 9.5.2.1's trigger: clean data never sets dn_lost; silence
       (0xff, outside every data bank) does, after exactly
       X2_CLIENT_LOSS_FRAMES frames. */
    CHECK(!client.dn_lost && client.bad_frames==0);
    unsigned frames=0;
    for(;frames<X2_CLIENT_LOSS_FRAMES+10 && !client.dn_lost;++frames) {
        uint8_t quiet[6]={0xff,0xff,0xff,0xff,0xff,0xff};
        x2_client_rx_codewords(&client,quiet,6);
    }
    CHECK(client.dn_lost && frames==X2_CLIENT_LOSS_FRAMES);
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

/* The client's answer to a server retrain (x2_session_client_retrain), as
 * Ie030002 accepted it: against a scripted digital side (Tone B; its
 * reversal 40 ms after our first; 10 ms more, then silence; its 160 ms
 * probe), our side must send 70 ms of silence, Tone A at 2400 Hz, its first
 * reversal no sooner than 50 ms into the tone, the second 40 ms after the
 * peer's, then stay silent through the probe and send the marker 69 on
 * 2400 Hz 60-100 ms after it, and come back to X2_CLIENT as the caller. */
static double tone_power(const int16_t *x, unsigned n, double f)
{
    double re=0,im=0,e=0;
    for(unsigned k=0;k<n;++k){re+=x[k]*cos(2*M_PI*f*k/8000);im-=x[k]*sin(2*M_PI*f*k/8000);e+=(double)x[k]*x[k];}
    return e>0 ? (re*re+im*im)/(n/2.0)/e : 0;
}
static void retrain_answer(void)
{
    static x2_session_t s, probe_source;
    CHECK(!x2_session_init_client(&s,0x7fff,0x1fff,5));
    CHECK(!x2_session_init(&probe_source));
    CHECK(x2_session_client_retrain(&s)==-1);   /* only from X2_CLIENT */
    s.stage=X2_CLIENT;
    CHECK(!x2_session_client_retrain(&s) && s.stage==X2_TONE_A && s.retrains==1);
    uint8_t expect_marker[41]={0};
    expect_marker[0]=expect_marker[1]=1;
    CHECK(x2_info_encode(0x69,7,expect_marker+2)==39 && !memcmp(expect_marker,s.marker_bits,41));
    int16_t tx[8000*4];
    unsigned n, first_rev=0, second_rev=0, b_rev_at=0, tone_a_start=0, marker_start=0, marker_end=0, probe_end=0;
    unsigned prev_rev=0;
    double b_sign=1;
    for(n=0;n<8000*4 && s.stage!=X2_CLIENT;++n) {
        uint8_t code;
        x2_session_stage_t before=s.stage;
        CHECK(x2_session_tx(&s,&code,1)==1);
        tx[n]=(int16_t)ulaw_to_linear(code);
        if(!tone_a_start && tx[n])tone_a_start=n;
        if(s.a_reversals!=prev_rev){
            if(s.a_reversals==1)first_rev=n; else second_rev=n;
            prev_rev=s.a_reversals;
            if(s.a_reversals==1)b_rev_at=n+320;
        }
        if(before!=X2_CLIENT_MARKER && s.stage==X2_CLIENT_MARKER)marker_start=n;
        /* The digital side: Tone B until 10 ms after its reversal, then
           silence; its probe 20 ms after that. */
        double x=0;
        if(!b_rev_at || n<b_rev_at+80) {
            if(b_rev_at && n==b_rev_at)b_sign=-1;
            x=b_sign*3000*cos(2*M_PI*1200*n/8000);
        }
        else if(n>=b_rev_at+80+160 && n<b_rev_at+80+160+1280)x=probe_source.probe[(n-b_rev_at)%160];
        if(b_rev_at && n==b_rev_at+80+160+1280)probe_end=n;
        int16_t rx=(int16_t)ulaw_to_linear(linear_to_ulaw((int)lrint(x)));
        x2_session_rx(&s,&rx,1);
        CHECK(s.stage!=X2_FAILED);
    }
    marker_end=n;
    CHECK(s.stage==X2_CLIENT && !s.answering);
    for(unsigned k=0;k<560;++k)CHECK(tx[k]==0);
    CHECK(tone_a_start>=560 && tone_power(tx+tone_a_start,160,2400)>0.9);
    CHECK(first_rev>=tone_a_start+400);
    CHECK(second_rev>=b_rev_at+300 && second_rev<=b_rev_at+360);
    /* x2_session's quiet test: 60 ms after the smoothed level falls (as at
       startup, where the I-modem takes it). */
    CHECK(probe_end && marker_start>=probe_end+480 && marker_start<=probe_end+800);
    /* DPSK at 600 bit/s spreads round its carrier: compare the bands. */
    double hi=0,lo=0;
    for(double f=2100;f<=2700;f+=50)hi+=tone_power(tx+marker_start+40,320,f);
    for(double f=900;f<=1500;f+=50)lo+=tone_power(tx+marker_start+40,320,f);
    CHECK(hi>20*lo);
    printf("  server retrain answered: silence 70 ms, Tone A, reversals %u ms into it and %u ms after its, marker 69 on 2400 Hz %u ms after its probe\n",
           (first_rev-tone_a_start)/8,(second_rev-b_rev_at)/8,(marker_start-probe_end)/8);
    (void)marker_end;
}

/* Our server's retrain (x2_session_server_retrain) against our client's
 * answer, both sessions in lockstep over a byte-exact line: 70 ms of
 * silence then Tone B on 1200 Hz; the client answers 70 ms after hearing
 * it; the server reverses 40 ms after the client's first reversal, ends
 * Tone B 10 ms later and stays silent until the client's second reversal;
 * its probe 14 ms after that (Ie030002's timing, artifacts/x2-rt-answer3);
 * then the marker 69 on 2400 Hz is accepted and Phase 3 is next. Also: Tone A
 * in data is answered while the call has retrains left (9.5.1.2). */
static void server_retrain(void)
{
    static x2_session_t srv, cli;
    CHECK(!x2_session_init(&srv));
    CHECK(!x2_session_init_client(&cli,0x7fff,0x1fff,5));
    CHECK(x2_session_server_retrain(&srv)==-1);   /* only in data */
    srv.stage=X2_PAYLOAD;cli.stage=X2_CLIENT;
    CHECK(!x2_session_server_retrain(&srv) && srv.stage==X2_TONE_A && srv.retrains==1 && !srv.answering);
    unsigned n, tone_b_start=0, srv_rev=0, cli_rev1=0, cli_rev2=0, probe_start=0, b_end=0;
    unsigned prev_cli=0, quiet=0;
    int16_t to_cli=0, to_srv=0;
    for(n=0;n<8000*4 && srv.stage!=X2_UPSTREAM_WAIT;++n) {
        uint8_t a,b;
        unsigned srv_before=srv.a_reversals;
        x2_session_stage_t srv_stage=srv.stage;
        CHECK(x2_session_tx(&srv,&a,1)==1 && x2_session_tx(&cli,&b,1)==1);
        to_cli=(int16_t)ulaw_to_linear(a);to_srv=(int16_t)ulaw_to_linear(b);
        if(!tone_b_start && to_cli)tone_b_start=n;
        if(srv.a_reversals!=srv_before)srv_rev=n;
        /* Tone B passes through zero every fifth sample: its end is the
           start of 5 ms of silence. */
        quiet=to_cli ? 0 : quiet+1;
        if(srv_rev && !b_end && n>srv_rev && quiet==40)b_end=n-39;
        if(srv_stage!=X2_PROBE && srv.stage==X2_PROBE)probe_start=n;
        if(cli.a_reversals!=prev_cli){if(cli.a_reversals==1)cli_rev1=n;else cli_rev2=n;prev_cli=cli.a_reversals;}
        /* The client hears Tone B as x2_client.c's tone_b would, 50 ms in. */
        if(cli.stage==X2_CLIENT && !cli.retrains && tone_b_start && n>=tone_b_start+400)
            CHECK(!x2_session_client_retrain(&cli));
        x2_session_rx(&cli,&to_cli,1);x2_session_rx(&srv,&to_srv,1);
        CHECK(srv.stage!=X2_FAILED && cli.stage!=X2_FAILED);
    }
    CHECK(srv.stage==X2_UPSTREAM_WAIT && srv.marker_valid && srv.marker==0x69);
    CHECK(tone_b_start==560);
    CHECK(srv_rev>=cli_rev1+300 && srv_rev<=cli_rev1+360);
    CHECK(b_end>=srv_rev+60 && b_end<=srv_rev+100);
    CHECK(cli_rev2>=srv_rev+300 && cli_rev2<=srv_rev+360);
    CHECK(probe_start>=cli_rev2+80 && probe_start<=cli_rev2+160);
    for(unsigned k=0;k<200 && cli.stage!=X2_CLIENT;++k) {
        uint8_t b;x2_session_tx(&cli,&b,1);
    }
    CHECK(cli.stage==X2_CLIENT && !cli.answering);
    printf("  server retrain against our client: Tone B at 70 ms, its reversal %u ms after the client's, probe %u ms after the second, marker 69 accepted at %u ms\n",
           (srv_rev-cli_rev1)/8,(probe_start-cli_rev2)/8,n/8);
    /* 9.5.1.2: Tone A in data starts the same retrain while one is left. */
    for(unsigned left=0;left<2;++left) {
        CHECK(!x2_session_init(&srv));
        srv.stage=X2_PAYLOAD;srv.max_retrains=left;
        int16_t rx[160];uint8_t tx[160];
        for(unsigned t=0;t<4000 && srv.stage==X2_PAYLOAD;t+=160) {
            for(unsigned k=0;k<160;++k)
                rx[k]=(int16_t)lrint(1660*(sqrt(0.8)*cos(2*M_PI*2400*(t+k)/8000)+sqrt(0.2)*cos(2*M_PI*1800*(t+k)/8000)));
            x2_session_rx(&srv,rx,160);
        }
        if(left)CHECK(srv.stage==X2_TONE_A && srv.retrains==1 && srv.peer_info_valid);
        else CHECK(srv.stage==X2_FAILED && srv.failure==X2_FAILURE_PEER_RETRAIN);
        if(left) {
            /* Silence for 70 ms, then Tone B, though Tone A is already up. */
            CHECK(x2_session_tx(&srv,tx,160)==160);
            for(unsigned k=0;k<160;++k)CHECK(ulaw_to_linear(tx[k])==0);
            /* No second retrain from Tone A once the call has used its one. */
            CHECK(x2_session_server_retrain(&srv)==-1);
        }
    }
    printf("  Tone A in data: answered with a retrain while one is left, else the call ends\n");
}

/* A-law (Ie030002 DSP command 6b): the A-law server's every octet outside
 * the Phase 2 tones is exactly the µ-law server's ^ 0x2a, through B, J, C,
 * D, E, the record and data; and an A-law client decodes it bit-exact. */
static void alaw_downstream(void)
{
    static x2_session_t mu, al;
    static x2_client_t client;
    source_t s1={0xace1,malloc(1<<20),0,1<<20}, s2={0xace1,malloc(1<<20),0,1<<20};
    sink_t sink={malloc(1<<20),0,1<<20};
    CHECK(!x2_session_init(&mu) && !x2_session_init(&al));
    x2_session_set_alaw(&al);
    CHECK(al.alaw && (al.info_body&0x200) && !(mu.info_body&0x200));
    CHECK(!x2_client_init(&client,0x7fff,0x1ffe,5));
    x2_client_set_alaw(&client);
    x2_client_set_put_bit(&client,sink_bit,&sink);
    x2_session_set_payload_source(&mu,source_bit,&s1);
    x2_session_set_payload_source(&al,source_bit,&s2);
    mu.stage=al.stage=X2_UPSTREAM_WAIT;
    x2_session_upstream_j(&mu);x2_session_upstream_j(&al);
    unsigned n,mp_given=0,payload=0,compared=0;
    for(n=0;n<200000 && payload<48000;++n) {
        uint8_t a,b;
        CHECK(x2_session_tx(&mu,&a,1)==1 && x2_session_tx(&al,&b,1)==1);
        CHECK(mu.stage==al.stage);
        CHECK(b==(uint8_t)(a^X2_ALAW_XOR));++compared;
        x2_client_rx_codewords(&client,&b,1);
        if(client.j_seen && mu.stage==X2_J){x2_session_upstream_s_bar(&mu);x2_session_upstream_s_bar(&al);}
        if(!mp_given && mu.stage==X2_TRAIN_C){x2_session_receive_mp(&mu,&client.mp);x2_session_receive_mp(&al,&client.mp);mp_given=1;}
        if(client.record_valid && !client.e_sent){client.e_sent=1;mu.mp_rx.e_detected=al.mp_rx.e_detected=1;}
        if(al.stage==X2_PAYLOAD)++payload;
        CHECK(al.stage!=X2_FAILED && client.dn!=X2C_DN_FAILED);
    }
    while(al.training_mapper.output_position!=6) {
        uint8_t b;CHECK(x2_session_tx(&al,&b,1)==1);x2_client_rx_codewords(&client,&b,1);
    }
    CHECK(client.dn==X2C_DN_DATA && client.decode_errors==0 && client.selected_index==15);
    CHECK(s2.count>1000 && sink.count>=s2.count);
    size_t offset=sink.count-s2.count;
    for(size_t i=0;i<s2.count;++i)CHECK(sink.bits[offset+i]==s2.bits[i]);
    /* The Phase 2 probe is coded in A-law, not XORed: its level matches. */
    static x2_session_t pm, pa;
    CHECK(!x2_session_init(&pm) && !x2_session_init(&pa));x2_session_set_alaw(&pa);
    pm.stage=pa.stage=X2_PROBE;
    for(unsigned k=0;k<160;++k) {
        uint8_t a,b;x2_session_tx(&pm,&a,1);x2_session_tx(&pa,&b,1);
        int x=ulaw_to_linear(a),y=alaw_to_linear(b);
        CHECK(abs(x-y)<=abs(x)/16+16);
    }
    printf("  A-law server: %u octets, all µ-law ^ 0x2a (zero, A, B, J, C, D, E, record, data); A-law client %zu payload bits exact; probe coded in A-law\n",
           compared,s2.count);
    free(s1.bits);free(s2.bits);free(sink.bits);
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
    retrain_answer();
    server_retrain();
    alaw_downstream();
    printf("x2 client: downstream record/final/payload at four ceilings, upstream MP/E and in-data rate changes passed\n");
    return 0;
}
