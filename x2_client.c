/* x2 analogue client after the marker; see x2_client.h for the sequence.
 * Independent implementation of recovered behavior; no vendor source. */
#include "x2_client.h"
#include "x2_mp_rx.h"
#include <string.h>
#include <math.h>

/* V.34 10.1.3.3 4-point training points and 10.1.3.9's 16-point mapping
 * (rows 2*Q2+Q1, columns Z), scaled to unit power -- the tables
 * v90_analogue_tx.c sends, so both analogue transmitters agree. */
static const float c4[4][2] = {
    {-0.7071068f, -0.7071068f}, {-0.7071068f, 0.7071068f},
    { 0.7071068f,  0.7071068f}, { 0.7071068f, -0.7071068f}
};
#define P1 0.3162278f
#define P3 0.9486833f
static const float c16[4][4][2] = {
    {{-P1, -P1}, {-P1,  P1}, { P1,  P1}, { P1, -P1}},
    {{ P3, -P1}, {-P1, -P3}, {-P3,  P1}, { P1,  P3}},
    {{-P1,  P3}, { P3,  P1}, { P1, -P3}, {-P3, -P1}},
    {{ P3,  P3}, { P3, -P3}, {-P3, -P3}, {-P3,  P3}}
};
#undef P1
#undef P3
/* V.34 10.1.3.6 PP: one 48-symbol period (same values as v90_analogue_tx.c). */
static const float pp[48][2] = {
    { 1.0000000f, 0.0000000f}, { 1.0000000f, 0.0000000f}, { 1.0000000f, 0.0000000f}, { 1.0000000f, 0.0000000f},
    {-0.5000000f, 0.8660254f}, {-0.8660254f, 0.5000000f}, {-1.0000000f, 0.0000000f}, {-0.8660254f,-0.5000000f},
    { 1.0000000f, 0.0000000f}, { 0.5000000f, 0.8660254f}, {-0.5000000f, 0.8660254f}, {-1.0000000f, 0.0000000f},
    { 1.0000000f, 0.0000000f}, { 0.0000000f, 1.0000000f}, {-1.0000000f, 0.0000000f}, {-0.0000000f,-1.0000000f},
    {-0.5000000f, 0.8660254f}, {-0.5000000f,-0.8660254f}, { 1.0000000f,-0.0000000f}, {-0.5000000f, 0.8660254f},
    { 1.0000000f, 0.0000000f}, {-0.8660254f, 0.5000000f}, { 0.5000000f,-0.8660254f}, { 0.0000000f, 1.0000000f},
    { 1.0000000f, 0.0000000f}, {-1.0000000f, 0.0000000f}, { 1.0000000f,-0.0000000f}, {-1.0000000f, 0.0000000f},
    {-0.5000000f, 0.8660254f}, { 0.8660254f,-0.5000000f}, {-1.0000000f, 0.0000000f}, { 0.8660254f, 0.5000000f},
    { 1.0000000f, 0.0000000f}, {-0.5000000f,-0.8660254f}, {-0.5000000f, 0.8660254f}, { 1.0000000f,-0.0000000f},
    { 1.0000000f, 0.0000000f}, {-0.0000000f,-1.0000000f}, {-1.0000000f, 0.0000000f}, { 0.0000000f, 1.0000000f},
    {-0.5000000f, 0.8660254f}, { 0.5000000f, 0.8660254f}, { 1.0000000f,-0.0000000f}, { 0.5000000f,-0.8660254f},
    { 1.0000000f, 0.0000000f}, { 0.8660254f,-0.5000000f}, { 0.5000000f,-0.8660254f}, {-0.0000000f,-1.0000000f}
};

/* Durations in 3200-baud symbols. The Courier's (Draft 0.33 3.4 and the
 * courier-emu bearer analysis): 60 ms after the marker, about 6400T of
 * Phase 3 TRN, J on a timer for 55 repetitions, and about 1.4 s of 16-point
 * TRN before its first MP. */
#define SILENCE_SYMBOLS 192
#define TRN_SYMBOLS     6400
#define J_SYMBOLS       440
#define TRN16_SYMBOLS   4400
#define ACK_FRAMES      2
#define E_BITS          20
/* In-data rate change bounds, in downstream codewords after our S: the
 * server's alignment word within 1.5 s (the Courier's own wait), the final
 * frame within 3 s (x2_session.c X2_RENEG_DEADLINE). */
#define RR_ALIGN_DEADLINE 12000
#define RR_DEADLINE       24000
/* The server's DATA_STARTUP: 4080 samples of ones in the new banks. */
#define DATA_STARTUP_CODEWORDS 4080
/* V.34 Table 18: J for a 16-point Phase 4 constellation, LSB first. */
#define J_WORD          0x89b0u
/* Server J (Draft 0.33 3.5): the 12-bit script word 09b0 behind GPC. */
#define SERVER_J        0x9b0u

static unsigned bit_length(unsigned x)
{
    unsigned n=0;
    while(x){++n;x>>=1;}
    return n;
}
static uint16_t crc_step(uint16_t crc, unsigned bit)
{
    return (uint16_t)((crc >> 1) ^ (((crc ^ bit) & 1) ? 0x8408 : 0));
}
static unsigned scramble(x2_client_t *c, unsigned bit)
{
    /* GPA, V.34 equation 7-2: the analogue modem's x^-5 tap (Draft 0.33 3.4:
     * the Courier's TRN descrambles with 1 + x^-5 + x^-23, not GPC). */
    unsigned out=(bit^(c->scramble>>4)^(c->scramble>>22))&1;
    c->scramble=(c->scramble<<1)|out;
    return out;
}
static void build_mp(x2_client_t *c, unsigned ack)
{
    x2_mp_t mp=c->mp;
    if(ack)mp.words[0]|=0x8000;
    x2_mp_encode(&mp,c->mp_bits);
}
int x2_client_init(x2_client_t *c, uint16_t pcm_mask, uint16_t upstream_mask, unsigned md)
{
    if(!c || !(pcm_mask&0x7fff) || !(upstream_mask&0x1fff) || md>6)return -1;
    memset(c,0,sizeof(*c));
    c->pcm_mask=pcm_mask&0x7fff;c->md=md;
    /* Clause 21: N1 is our PCM ceiling, N2 the highest V.34 N we transmit,
     * W2 the V.34 mask, W4's high byte the independent sign count. */
    unsigned n1=bit_length(c->pcm_mask),n2=bit_length(upstream_mask&0x1fff);
    c->mp.words[0]=(uint16_t)((n1<<2)|(n2<<6));
    c->mp.words[1]=(uint16_t)(upstream_mask&0x1fff);
    c->mp.words[3]=(uint16_t)(md<<8);
    build_mp(c,0);
    x2_training_config(&c->training);
    c->up=X2C_UP_SILENCE;c->dn=X2C_DN_WAIT_B;
    return 0;
}
void x2_client_set_put_bit(x2_client_t *c, x2_client_put_bit_t put_bit, void *context)
{
    if(c){c->put_bit=put_bit;c->put_context=context;}
}
static void enter(x2_client_t *c, x2_client_up_t next)
{
    c->up=next;c->up_symbols=0;
}
/* V.34 10.1.3.7: S alternates point 0 with point 0 rotated by 90 degrees;
 * S-bar is the pair rotated by 180 degrees (reversed after the last symbol). */
static void s_symbol(x2_client_t *c, float *re, float *im)
{
    *re=c4[c->s_index][0];*im=c4[c->s_index][1];c->s_index^=3;
}
/* V.34 10.1.3.3: two scrambled bits, differentially encoded. */
static void j_symbol(x2_client_t *c, unsigned b0, unsigned b1, float *re, float *im)
{
    unsigned in=scramble(c,b0)|(scramble(c,b1)<<1);
    c->diff=(c->diff+in)&3;
    *re=c4[c->diff][0];*im=c4[c->diff][1];
}
/* V.34 10.1.3.9 16-point: I1 I2 differential, Q1 Q2 the point, in time
 * order -- the mapping x2_mp_rx.c decodes the Courier's MP with. */
static void sixteen_symbol(x2_client_t *c, const unsigned bits[4], unsigned differential,
                           float *re, float *im)
{
    unsigned in=scramble(c,bits[0])|(scramble(c,bits[1])<<1);
    unsigned q=scramble(c,bits[2])|(scramble(c,bits[3])<<1);
    c->diff=differential ? (c->diff+in)&3 : in;
    *re=c16[q][c->diff][0];*im=c16[q][c->diff][1];
}
void x2_client_get_symbol(void *user_data, float *re, float *im)
{
    x2_client_t *c=user_data;
    *re=*im=0;
    if(!c)return;
    ++c->up_symbols;++c->up_total;
    switch(c->up) {
    case X2C_UP_SILENCE:
        if(c->up_symbols>=SILENCE_SYMBOLS)enter(c,X2C_UP_S);
        return;
    case X2C_UP_S:
    case X2C_UP_S2:
        s_symbol(c,re,im);
        if(c->up_symbols==128){c->s_index^=2;enter(c,c->up==X2C_UP_S?X2C_UP_S_BAR:X2C_UP_S_BAR2);}
        return;
    case X2C_UP_S_BAR:
    case X2C_UP_S_BAR2:
        s_symbol(c,re,im);
        if(c->up_symbols==16) {
            c->s_index^=2;
            /* V.34 10.1.3.8: the scrambler restarts from zero for TRN. */
            c->scramble=0;
            enter(c,c->up==X2C_UP_S_BAR?X2C_UP_PP:X2C_UP_TRN16);
        }
        return;
    case X2C_UP_PP:
        *re=pp[(c->up_symbols-1)%48][0];*im=pp[(c->up_symbols-1)%48][1];
        if(c->up_symbols==288)enter(c,X2C_UP_TRN);
        return;
    case X2C_UP_TRN: {
        /* Scrambled ones, direct mapping; J's differential encoder starts
         * from the last TRN symbol (V.34 10.1.3.3). */
        unsigned in=scramble(c,1)|(scramble(c,1)<<1);
        c->diff=in;*re=c4[in][0];*im=c4[in][1];
        if(c->up_symbols>=TRN_SYMBOLS){c->j_pos=0;enter(c,X2C_UP_J);}
        return;
    }
    case X2C_UP_J: {
        unsigned b0=(J_WORD>>c->j_pos)&1,b1=(J_WORD>>(c->j_pos+1))&1;
        c->j_pos=(c->j_pos+2)&15;
        j_symbol(c,b0,b1,re,im);
        /* The Courier stops J on a timer (@66), not on a received S: the
         * server sends no S. Silence until the server's J word. */
        if(c->up_symbols>=J_SYMBOLS)enter(c,c->j_seen?X2C_UP_S2:X2C_UP_WAIT_J);
        return;
    }
    case X2C_UP_WAIT_J:
        if(c->j_seen)enter(c,X2C_UP_S2);
        return;
    case X2C_UP_TRN16: {
        static const unsigned ones[4]={1,1,1,1};
        sixteen_symbol(c,ones,0,re,im);
        if(c->up_symbols>=TRN16_SYMBOLS){c->mp_pos=0;enter(c,X2C_UP_MP);}
        return;
    }
    case X2C_UP_MP: {
        unsigned bits[4];
        for(unsigned k=0;k<4;++k)bits[k]=c->mp_bits[c->mp_pos+k];
        sixteen_symbol(c,bits,1,re,im);
        c->mp_pos+=4;
        if(c->mp_pos<X2_MP_BITS)return;
        /* A frame boundary (104 = 26 symbols): acknowledge the server's
         * record from the next frame on, as the Courier's 8344 does. */
        c->mp_pos=0;
        if(c->ack_frames)++c->ack_frames;
        if(c->ack_pending && !c->ack_frames){build_mp(c,1);c->ack_frames=1;}
        else if(c->ack_frames>ACK_FRAMES)enter(c,X2C_UP_E);
        return;
    }
    case X2C_UP_E: {
        static const unsigned ones[4]={1,1,1,1};
        sixteen_symbol(c,ones,1,re,im);
        if(c->up_symbols*4>=E_BITS){c->e_sent=1;enter(c,X2C_UP_DATA);}
        return;
    }
    case X2C_UP_DATA:
        return;
    /* In-data rate change: S and S-bar as at Phase 4, then the MP in the
     * 4-point constellation (two bits a symbol, differential, scrambled --
     * the Courier's 0378/8378, which x2_mp_rx's four_point mode decodes),
     * acknowledged once the server's record carries its acknowledgement. */
    case X2C_UP_RR_S:
        s_symbol(c,re,im);
        if(c->up_symbols==128){c->s_index^=2;enter(c,X2C_UP_RR_S_BAR);}
        return;
    case X2C_UP_RR_S_BAR:
        s_symbol(c,re,im);
        if(c->up_symbols==16){c->s_index^=2;c->scramble=0;c->mp_pos=0;enter(c,X2C_UP_RR_MP);}
        return;
    case X2C_UP_RR_MP:
        j_symbol(c,c->mp_bits[c->mp_pos],c->mp_bits[c->mp_pos+1],re,im);
        c->mp_pos+=2;
        if(c->mp_pos<X2_MP_BITS)return;
        c->mp_pos=0;
        if(c->ack_frames)++c->ack_frames;
        if(c->ack_pending && !c->ack_frames){build_mp(c,1);c->ack_frames=1;}
        else if(c->ack_frames>ACK_FRAMES)enter(c,X2C_UP_RR_E);
        return;
    case X2C_UP_RR_E:
        j_symbol(c,1,1,re,im);
        if(c->up_symbols*2>=E_BITS){c->e_sent=1;enter(c,X2C_UP_DATA);}
        return;
    }
}
bool x2_client_can_change_rate(const x2_client_t *c)
{
    return c && !c->rr && c->up==X2C_UP_DATA && c->dn==X2C_DN_DATA
        && c->dn_samples-c->data_sample>=DATA_STARTUP_CODEWORDS;
}
int x2_client_start_rate_change(x2_client_t *c, unsigned index)
{
    if(!x2_client_can_change_rate(c) || !index || index>15)return -1;
    /* N1: the highest index we accept at or below the one asked for. */
    unsigned n1=bit_length(c->pcm_mask&((1u<<index)-1));
    if(!n1)return -1;
    c->mp.words[0]=(uint16_t)((c->mp.words[0]&~0x3cu)|(n1<<2));
    build_mp(c,0);
    c->rr=1;++c->rate_changes;c->rr_samples=0;
    c->record_valid=c->ack_pending=c->ack_frames=c->e_sent=c->final_seen=0;
    c->s_index=0;c->ring_count=0;c->hold_first=c->hold_frames=0;
    enter(c,X2C_UP_RR_S);
    return 0;
}
bool x2_client_data_due(void *user_data)
{
    const x2_client_t *c=user_data;
    return c && c->up==X2C_UP_DATA;
}
/* Clause 18/20: the server's three-word record. Bits 2..5 and 6..9 are the
 * server's ceilings -- Ie030002 sends F37C (PCM 15, V.34 N 13) whatever our
 * MP asked, and the Courier then ran 24000 up, the top of its own W2 03fe.
 * So the downstream index is the highest the record's PCM mask allows under
 * both ceilings, and the upstream N the highest in our W2 under both. */
static void accept_record(x2_client_t *c)
{
    unsigned ceiling=(c->record[0]>>2)&15,n1=(c->mp.words[0]>>2)&15;
    if(ceiling>n1)ceiling=n1;
    unsigned index=bit_length(c->record[1]&0x7fffu&((1u<<ceiling)-1));
    unsigned n_ceiling=(c->record[0]>>6)&15,n2=(c->mp.words[0]>>6)&15;
    if(n_ceiling>n2)n_ceiling=n2;
    unsigned n=bit_length(c->mp.words[1]&((1u<<n_ceiling)-1));
    if(!index || !(c->pcm_mask&(1u<<(index-1))) || !n
       || x2_short_profile(index,c->md,&c->data)) {
        c->dn=X2C_DN_FAILED;return;
    }
    c->selected_index=index;c->upstream_n=n;c->record_valid=1;c->ack_pending=1;
}
/* The rate change's record: the same ceilings as at startup, under the N1
 * of the MP that opened the change; the new banks wait for the final frame. */
static void rr_accept(x2_client_t *c, const uint16_t record[3])
{
    unsigned ceiling=(record[0]>>2)&15,n1=(c->mp.words[0]>>2)&15;
    if(ceiling>n1)ceiling=n1;
    unsigned index=bit_length(record[1]&0x7fffu&((1u<<ceiling)-1));
    unsigned n_ceiling=(record[0]>>6)&15,n2=(c->mp.words[0]>>6)&15;
    if(n_ceiling>n2)n_ceiling=n2;
    unsigned n=bit_length(c->mp.words[1]&((1u<<n_ceiling)-1));
    if(!index || !(c->pcm_mask&(1u<<(index-1))) || !n
       || x2_short_profile(index,c->md,&c->rr_next)) {
        c->dn=X2C_DN_FAILED;return;
    }
    memcpy(c->record,record,sizeof(c->record));
    c->selected_index=index;c->upstream_n=n;c->record_valid=1;c->ack_pending=1;
}
/* d->bits is the index of this bit in the decoder's descrambled stream. */
static void record_bit(x2_client_decoder_t *d, unsigned bit)
{
    if(!d->rpos) {
        if(bit){if(d->rones<17)++d->rones;return;}
        if(d->rones>=17)d->rpos=1;
        d->rones=0;return;
    }
    d->rbits[d->rpos-1]=(uint8_t)bit;
    if(++d->rpos<=67)return;
    d->rpos=0;
    uint16_t crc=0xffff,received=0,words[3]={0,0,0};
    for(unsigned w=0;w<3;++w) {
        if(d->rbits[17*w+16])return;
        for(unsigned i=0;i<16;++i) {
            unsigned b=d->rbits[17*w+i];
            words[w]|=(uint16_t)(b<<i);crc=crc_step(crc,b);
        }
    }
    for(unsigned i=0;i<16;++i)received|=(uint16_t)(d->rbits[51+i]<<i);
    if(received!=crc)return;
    /* Sync start: 17 ones, the zero, 67 bits, this one is the last. */
    d->record_start=d->bits-84;
    /* Two identical CRC-valid records: the server repeats it until our E. */
    if(d->record_repeats && !memcmp(words,d->record,sizeof(words)))++d->record_repeats;
    else d->record_repeats=1;
    memcpy(d->record,words,sizeof(words));
    if(d->record_repeats>=2)d->record_valid=1;
}
/* Is the newest 36 descrambled bits the server's 12-bit J word, in any
 * phase? history bit i is the bit received i bits ago. */
static unsigned server_j(uint64_t history)
{
    for(unsigned r=0;r<12;++r) {
        unsigned i;
        for(i=0;i<36;++i)
            if(((history>>i)&1)!=((SERVER_J>>((r+36-i)%12))&1))break;
        if(i==36)return 1;
    }
    return 0;
}
static int align_matches(const x2_client_t *c)
{
    /* x2_session.c RECORD_ALIGN at startup: eight +++--- groups then one
     * inverted group, on the training banks' lowest code A5. Ie030002 sends
     * none: it switches the E mapper's source to the record in place. */
    for(unsigned n=0;n<54;++n) {
        uint8_t expected=(uint8_t)(0xa5^((n%6>=3?1u:0u)<<7)^((n>=48?1u:0u)<<7));
        if(c->ring[(c->ring_pos+n)%54]!=expected)return 0;
    }
    return 1;
}
/* Decode one frame. Returns 1 if its source bits were all ones, 0 if not,
 * -1 if a codeword is outside the banks. */
static int decode_frame(x2_client_t *c, x2_client_decoder_t *d, const x2_pcm_config_t *config,
                        unsigned deliver)
{
    uint64_t bits;
    x2_pcm_state_t parity=d->parity;
    if(x2_pcm_decode(config,&parity,d->frame,&bits)){++d->errors;return -1;}
    d->parity=parity;
    unsigned n=x2_pcm_frame_bits(config),all_ones=1;
    for(unsigned i=0;i<n;++i,++d->bits) {
        unsigned bit=x2_descramble_bit(&d->descrambler,(unsigned)((bits>>i)&1));
        if(!bit)all_ones=0;
        if(deliver==3) {}
        else if(deliver==2) {
            unsigned slot=(c->hold_first+c->hold_frames-1)%9;
            if(c->hold_count[slot]<sizeof(c->hold_bits[slot]))c->hold_bits[slot][c->hold_count[slot]++]=(uint8_t)bit;
        }
        else if(deliver) {
            ++c->data_bits;
            if(c->put_bit)c->put_bit(c->put_context,bit);
        }
        else record_bit(d,bit);
    }
    return (int)all_ones;
}
static void decoder_reset(x2_client_decoder_t *d, unsigned skip)
{
    memset(d,0,sizeof(*d));d->skip=skip;
    x2_scrambler_init(&d->descrambler,18,0);
}
/* Feed one octet; returns 1 when a frame completes. */
static int decoder_octet(x2_client_decoder_t *d, uint8_t code)
{
    if(d->skip){--d->skip;return 0;}
    d->frame[d->frame_pos++]=code;
    if(d->frame_pos<6)return 0;
    d->frame_pos=0;return 1;
}
static void main_frame(x2_client_t *c)
{
    x2_client_decoder_t *d=&c->main;
    uint64_t start=d->bits;
    if(c->dn==X2C_DN_DATA) {
        /* After a rate change, the server's DATA_STARTUP ones (680 frames)
         * are not data: it stopped taking source bits mid-character, so
         * delivering them would break a V.14 character. Dropped, the
         * stream continues exactly where it stopped. */
        unsigned drop=c->drop_frames ? (--c->drop_frames,3u) : 1u;
        if(decode_frame(c,d,&c->data,drop)<0)++c->decode_errors;
        return;
    }
    int ones=decode_frame(c,d,&c->training,0);
    if(ones<0){++c->decode_errors;return;}
    /* After our E the server finishes its record (96 bits, four frames),
     * sends one frame of ones on the record boundary and switches to the
     * data banks on the next frame, keeping parity and scrambler history
     * (x2_session.c FINAL_TRAINING, Ie030002 A891/A8F1). A record's first
     * frame is never all ones: it holds the zero after the 17-one sync. */
    if(c->e_sent && ones==1 && start>=d->record_start && (start-d->record_start)%96==0) {
        c->final_seen=1;c->dn=X2C_DN_DATA;c->data_sample=c->dn_samples;
    }
}
/* While a rate change is open, the newest nine data frames are held back:
 * the server's 54-octet alignment word starts on a frame boundary, so when
 * it is recognised it is exactly those frames, and none of it reaches the
 * DTE. */
static void hold_deliver_oldest(x2_client_t *c)
{
    unsigned slot=c->hold_first;
    for(unsigned i=0;i<c->hold_count[slot];++i) {
        ++c->data_bits;
        if(c->put_bit)c->put_bit(c->put_context,c->hold_bits[slot][i]);
    }
    c->hold_first=(slot+1)%9;--c->hold_frames;
}
/* x2_session.c align_code(): each position's lowest code of the banks in
 * use; Ie030002 sends A5 at index 1, which is the same rule there. */
static int rr_align_matches(const x2_client_t *c)
{
    for(unsigned rule=0;rule<2;++rule) {
        unsigned n;
        for(n=0;n<54;++n) {
            unsigned p=n%6;
            uint8_t base=rule ? 0xa5 : (uint8_t)((c->data.banks[p][0]&0xff)|0x80);
            uint8_t expected=(uint8_t)(base^((p>=3?1u:0u)<<7)^((n>=48?1u:0u)<<7));
            if(c->ring[(c->ring_pos+n)%54]!=expected)break;
        }
        if(n==54)return 1;
    }
    return 0;
}
static void rr_data_frame(x2_client_t *c)
{
    if(c->hold_frames==9)hold_deliver_oldest(c);
    c->hold_count[(c->hold_first+c->hold_frames)%9]=0;++c->hold_frames;
    (void)decode_frame(c,&c->main,&c->data,2);
    if(c->ring_count<54 || !rr_align_matches(c))return;
    /* x2_session.c RECORD_TX: the record on a fresh mapper (zero scrambler
     * and parity), in the index-1 allocation at the current MD -- what the
     * Courier acknowledged from index 6 -- or, as far as anything shows,
     * the current banks: decode both and follow whichever carries it. */
    c->hold_frames=0;
    c->rr_config[1]=c->data;
    if(x2_short_profile(1,c->data.independent_signs,&c->rr_config[0]))c->rr_config[0]=c->data;
    for(unsigned k=0;k<2;++k)decoder_reset(&c->rr_dec[k],0);
    c->rr_decoder=2;c->dn=X2C_DN_RR_RECORD;
}
static void rr_record_frame(x2_client_t *c, unsigned k)
{
    x2_client_decoder_t *d=&c->rr_dec[k];
    uint64_t start=d->bits;
    int ones=decode_frame(c,d,&c->rr_config[k],0);
    if(ones<0)return;
    if(c->rr_decoder==2 && d->record_valid)c->rr_decoder=k;
    if(c->rr_decoder!=k)return;
    /* The server sets bit 15 once our MP is in (Ie030002 733c, then f33c). */
    if(!c->record_valid && d->record_repeats>=2 && (d->record[0]&0x8000))rr_accept(c,d->record);
    /* One frame of ones where the next record would start, then the new
     * banks, scrambler and parity carried on, as at startup. Not gated on
     * our E: Ie030002 ends its record about 45 ms after acknowledging, on
     * our acknowledged MP, before our E is out (x2-client-imodem-rr8,
     * 28 Oct). A record always starts 17 ones and a zero, so only the final
     * frame is all ones on a record boundary. */
    if(c->record_valid && ones==1 && start>=d->record_start && (start-d->record_start)%96==0) {
        c->final_seen=1;c->main=*d;c->data=c->rr_next;c->dn=X2C_DN_DATA;c->rr=0;
        c->data_sample=c->dn_samples;c->drop_frames=DATA_STARTUP_CODEWORDS/6;
    }
}
static int ulaw_linear(uint8_t code)
{
    unsigned c=~code&0xff;
    int v=(int)((((c&15)<<3)+132)<<((c>>4)&7))-132;
    return (c&0x80) ? -v : v;
}
/* Tone B in data: the server has gone to a retrain. V.90 9.5.1.1 has a
 * digital modem initiate one with 70 ms of silence and then Tone B
 * (1200 Hz), waiting for the analogue modem's Tone A -- what Ie030002 does
 * after 500 ms without our upstream (reference-data/x2-imodem-retrain;
 * the Courier's own retrain is the analogue side's Tone A, 9.5.2.1). Eight
 * 40-sample blocks, 40 ms, with 0.6 of the power at 1200 Hz and an RMS of
 * at least 250; x2 data is white over codewords. */
static void tone_b_sample(x2_client_t *c, uint8_t code)
{
    c->tone_window[c->tone_pos++]=(int16_t)ulaw_linear(code);
    if(c->tone_pos<40)return;
    c->tone_pos=0;
    double energy=0,re=0,im=0;
    for(unsigned n=0;n<40;++n) {
        double x=c->tone_window[n],ph=2*3.14159265358979323846*1200*n/8000;
        re+=x*cos(ph);im-=x*sin(ph);energy+=x*x;
    }
    if(energy>=40*250.0*250.0 && re*re+im*im>=0.6*20*energy) {
        if(++c->tone_b_blocks>=8)c->tone_b=1;
    }
    else c->tone_b_blocks=0;
}
void x2_client_rx_codewords(x2_client_t *c, const uint8_t *codewords, size_t count)
{
    if(!c||!codewords)return;
    for(size_t k=0;k<count;++k,++c->dn_samples) {
        uint8_t code=codewords[k];
        if(c->dn==X2C_DN_DATA || c->dn==X2C_DN_RR_RECORD)tone_b_sample(c,code);
        if(c->rr && c->dn!=X2C_DN_FAILED
           && (++c->rr_samples>RR_DEADLINE || (c->dn==X2C_DN_DATA && c->rr_samples>RR_ALIGN_DEADLINE)))
            c->dn=X2C_DN_FAILED;
        switch(c->dn) {
        case X2C_DN_WAIT_B:
            /* Stretch B: two levels, b1/31, one GPC bit per octet. */
            c->run=(code==0xb1||code==0x31)?c->run+1:0;
            if(c->run>=48){c->dn=X2C_DN_B;c->b_bits=0;x2_scrambler_init(&c->descrambler,18,0);}
            break;
        case X2C_DN_B:
            if(code!=0xb1 && code!=0x31)break;
            c->history=(c->history<<1)|x2_descramble_bit(&c->descrambler,((code^0xb1)>>7)&1);
            if(++c->b_bits>=60 && server_j(c->history)) {
                c->j_seen=1;c->dn=X2C_DN_TRAIN;c->ring_count=0;
                for(unsigned p=0;p<6;++p)decoder_reset(&c->phase[p],p);
            }
            break;
        case X2C_DN_TRAIN:
            c->ring[c->ring_pos]=code;c->ring_pos=(c->ring_pos+1)%54;
            if(++c->ring_count>=54 && align_matches(c)) {
                /* Our server: x2_pcm_tx_init restarts the mapper after the
                 * word with zero scrambler history and parity. */
                decoder_reset(&c->main,0);c->dn=X2C_DN_RECORD;
                break;
            }
            for(unsigned p=0;p<6;++p) {
                x2_client_decoder_t *d=&c->phase[p];
                if(!decoder_octet(d,code))continue;
                decode_frame(c,d,&c->training,0);
                if(d->record_valid) {
                    c->main=*d;c->record_phase=p;c->dn=X2C_DN_RECORD;
                    memcpy(c->record,d->record,sizeof(c->record));accept_record(c);
                    break;
                }
            }
            break;
        case X2C_DN_RECORD:
            if(!decoder_octet(&c->main,code))break;
            main_frame(c);
            if(c->main.record_valid && !c->record_valid) {
                memcpy(c->record,c->main.record,sizeof(c->record));accept_record(c);
            }
            break;
        case X2C_DN_DATA:
            if(c->rr) {
                c->ring[c->ring_pos]=code;c->ring_pos=(c->ring_pos+1)%54;
                if(c->ring_count<54)++c->ring_count;
                if(decoder_octet(&c->main,code))rr_data_frame(c);
            }
            else if(decoder_octet(&c->main,code))main_frame(c);
            break;
        case X2C_DN_RR_RECORD:
            for(unsigned j=0;j<2;++j)
                if((c->rr_decoder==2 || c->rr_decoder==j) && decoder_octet(&c->rr_dec[j],code))
                    rr_record_frame(c,j);
            break;
        case X2C_DN_FAILED:
            break;
        }
    }
}
unsigned x2_client_downstream_rate(const x2_client_t *c)
{
    return c && c->record_valid ? x2_pcm_frame_bits(&c->data)*8000u/6u : 0;
}
const char *x2_client_up_name(x2_client_up_t s)
{
    static const char *names[]={"SILENCE","S","S_BAR","PP","TRN","J","WAIT_J","S2","S_BAR2","TRN16","MP","E","DATA","RR_S","RR_S_BAR","RR_MP","RR_E"};
    return (unsigned)s<sizeof(names)/sizeof(names[0])?names[s]:"INVALID";
}
const char *x2_client_dn_name(x2_client_dn_t s)
{
    static const char *names[]={"WAIT_B","B","TRAIN","RECORD","DATA","FAILED","RR_RECORD"};
    return (unsigned)s<sizeof(names)/sizeof(names[0])?names[s]:"INVALID";
}
