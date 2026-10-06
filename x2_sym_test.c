#include "x2_sym.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "x2_sym_vectors.h"
static void startup(unsigned block, unsigned impairment)
{
    x2_sym_startup_t answer, call;
    uint8_t a[2011], c[2011], idle[17];
    size_t n = 0;
    assert(!x2_sym_startup_init(&answer, 1));
    assert(!x2_sym_startup_init(&call, 0));
    assert(x2_sym_startup_tx(&call, idle, sizeof(idle)) == sizeof(idle));
    for (unsigned i=0;i<sizeof(idle);++i) assert(idle[i]==0xff);
    assert(call.tx_position==0);
    while (n < 2010) n += x2_sym_startup_tx(&answer,a+n,block < 2010-n ? block : 2010-n);
    assert(x2_sym_startup_tx(&answer,idle,1)==0);
    for (unsigned i=0;i<2010;++i) {
        unsigned expected=i<1747?0x7e:i<1754?0:((i-1754)&1)?255-(i-1754)/2:(i-1754)/2;
        assert(a[i]==expected);
        if (impairment && i%6==impairment-1) a[i]^=1;
    }
    a[2010]=0x81;
    for (n=0;n<2010;) {
        size_t got=x2_sym_startup_rx(&call,a+n,block < 2011-n ? block : 2011-n);
        assert(got); n+=got;
    }
    assert(n==2010 && call.rx_stage==X2_SYM_ACQUIRED && call.tx_enabled);
    assert(impairment ? call.error_map && !(call.error_map & (call.error_map-1)) : !call.error_map);
    assert(x2_sym_startup_tx(&call,c,sizeof(c))==2010);
    assert(x2_sym_startup_rx(&answer,c,2010)==2010);
    assert(answer.rx_stage==X2_SYM_ACQUIRED);
}
static void rejection(void)
{
    x2_sym_startup_t s;
    uint8_t run[7], zero[7]={0}, ramp[256];
    memset(run,0x7e,7);
    for(unsigned i=0;i<256;++i)ramp[i]=(uint8_t)((i&1)?255-i/2:i/2);
    assert(!x2_sym_startup_init(&s,0));
    x2_sym_startup_rx(&s,run,7);x2_sym_startup_rx(&s,zero,7);
    ramp[37]^=2;x2_sym_startup_rx(&s,ramp,256);
    assert(s.rx_stage!=X2_SYM_ACQUIRED && !s.tx_enabled && s.restarts);
    ramp[37]^=2;x2_sym_startup_rx(&s,run,7);x2_sym_startup_rx(&s,zero,7);
    x2_sym_startup_rx(&s,ramp,256);assert(s.rx_stage==X2_SYM_ACQUIRED);
}
static void data(void)
{
    for(unsigned width=7;width<=8;++width)for(unsigned scramble=0;scramble<2;++scramble)
    for(unsigned reverse=0;reverse<2;++reverse) {
        x2_sym_data_t a,b;
        assert(!x2_sym_data_init(&a,width,scramble,reverse));
        assert(!x2_sym_data_init(&b,width,scramble,reverse));
        for(unsigned i=0;i<8192;++i) {
            uint8_t x=(uint8_t)(i*71+i/256),y=(uint8_t)(i*13+i/64);
            assert(x2_sym_data_rx(&b,x2_sym_data_tx(&a,x))==(x & ((1u<<width)-1)));
            assert(x2_sym_data_rx(&a,x2_sym_data_tx(&b,y))==(y & ((1u<<width)-1)));
        }
    }
    for(unsigned i=0;i<sizeof(sym_vectors)/sizeof(sym_vectors[0]);++i) {
        x2_sym_data_t s;
        assert(!x2_sym_data_init(&s,8,1,1));
        s.tx.history=sym_vectors[i].before;
        unsigned wire=x2_sym_data_tx(&s,(uint8_t)sym_vectors[i].source);
        assert(s.tx.history==sym_vectors[i].after);
        /* The final native history contains all eight emitted bits. */
        unsigned bits=sym_vectors[i].after>>15, reversed=0;
        for(unsigned j=0;j<8;++j)reversed|=((bits>>j)&1)<<(7-j);
        assert(wire==reversed);
    }
}
static void capability(void)
{
    for(unsigned errors=0;errors<128;++errors) {
        x2_sym_cap_t local,peer,decoded;
        uint8_t wire[11]; unsigned map=999,mask=999,scramble=999;
        assert(!x2_sym_cap_build(&local,0,0x41,1));
        assert(!x2_sym_cap_build(&peer,errors,0x41,1));
        assert(!x2_sym_cap_encode(&peer,wire));
        assert(!x2_sym_cap_decode(wire,&decoded));
        assert(!memcmp(&decoded,&peer,sizeof(peer)));
        assert(x2_sym_cap_merge(&local,&peer,&map,&mask,&scramble)==(errors?56000:64000));
        assert(map==errors && mask==(errors?1:0x41) && scramble==!errors);
        for(unsigned p=2;p<11;++p)for(unsigned bit=1;bit<(p<7?7:5);++bit) {
            x2_sym_cap_t unchanged=local;
            wire[p]^=(uint8_t)(1u<<bit);
            assert(x2_sym_cap_decode(wire,&unchanged)==-1);
            assert(!memcmp(&unchanged,&local,sizeof(local)));
            wire[p]^=(uint8_t)(1u<<bit);
        }
    }
    x2_sym_cap_t a,b;unsigned errors=123,mask=123,scramble=123;
    assert(!x2_sym_cap_build(&a,0,0,0));assert(!x2_sym_cap_build(&b,0,1,0));
    /* No low bits in common and no reported eight-bit eligibility. */
    a.words[3]=1;b.words[3]=1;
    assert(!x2_sym_cap_merge(&a,&b,&errors,&mask,&scramble));
    assert(errors==123 && mask==123 && scramble==123);
}
static void roles(void)
{
    /* CRC-decoded bodies from native asymmetric and symmetric calls. */
    assert(x2_info_role_select(0x3dff,0x21ff)==X2_ROLE_HOST);
    assert(x2_info_role_select(0x21ff,0x3dff)==X2_ROLE_CLIENT);
    assert(x2_info_role_select(0x3dff,0x3dff)==X2_ROLE_SYMMETRIC);
    for(unsigned l=0;l<4;++l)for(unsigned p=0;p<4;++p) {
        uint32_t local=0x40|(l<<11),peer=0x40|(p<<11);
        x2_role_t expected=(l&p&1)?X2_ROLE_SYMMETRIC:
            ((l^p)&2)?((l&2)?X2_ROLE_HOST:X2_ROLE_CLIENT):X2_ROLE_NONE;
        assert(x2_info_role_select(local,peer)==expected);
        assert(x2_info_role_select(local,peer&~0x40u)==X2_ROLE_NONE);
    }
    assert(x2_info_role_select(0x20000,0x3dff)==X2_ROLE_NONE);
    assert(x2_info_role_select(0x3dff,0x20000)==X2_ROLE_NONE);
}
typedef struct { unsigned tx,rx,other,errors; } link_bits_t;
static unsigned pattern(unsigned position,unsigned salt)
{
    unsigned x=position+salt;x^=x<<13;x^=x>>17;x^=x<<5;return (x>>7)&1;
}
static int link_get(void *ctx)
{
    link_bits_t *b=ctx;return (int)pattern(b->tx++,b->other^0x12345);
}
static void link_put(void *ctx,int bit)
{
    link_bits_t *b=ctx;
    b->errors+=(unsigned)bit!=pattern(b->rx++,b->other);
}
static void dialogue(unsigned block)
{
    x2_sym_link_t a,c;uint8_t ab[160],cb[160];
    link_bits_t aa={0,0,0x789ab,0},cc={0,0,0x789ab^0x12345,0};
    memset(ab,0xff,block);memset(cb,0xff,block);
    assert(!x2_sym_link_init(&a,1,link_get,link_put,&aa));
    assert(!x2_sym_link_init(&c,0,link_get,link_put,&cc));
    for(unsigned n=0;n<12000;n+=block) {
        x2_sym_link_rx(&a,cb,block);x2_sym_link_rx(&c,ab,block);
        x2_sym_link_tx(&a,ab,block);x2_sym_link_tx(&c,cb,block);
    }
    assert(a.rx_data && a.tx_data && c.rx_data && c.tx_data);
    assert(a.rate==64000 && c.rate==64000);
    assert(aa.rx>30000 && cc.rx>30000 && !aa.errors && !cc.errors);
}
int main(void)
{
    unsigned blocks[]={1,17,160};
    for(unsigned b=0;b<3;++b)for(unsigned phase=0;phase<=6;++phase)startup(blocks[b],phase);
    rejection();data();capability();roles();
    for(unsigned b=0;b<3;++b)dialogue(blocks[b]);
    puts("x2 symmetric: both startup roles, six impairment phases, restart, 56/64k duplex transforms and native GPC transitions passed");
    return 0;
}
