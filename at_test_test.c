/* Actual V.250 diagnostic commands through T.31 and both PTY presentations.
 * Core tests separately inject bit errors: a local software loop otherwise
 * cannot establish that its BERT checker would notice a fault. */
#include "at_test.h"
#include "data_interface.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>

static int failures;
#define CHECK(ok,what) do { if(!(ok)) {fprintf(stderr,"FAIL: %s\n",what);failures++;} } while(0)
static int cmd(at_test_t *s,const char *command,int online,char *response)
{ return at_test_command(s,command,online,response,256); }
static void core_tests(void)
{
    at_test_t s;char response[256];
    at_test_reset(&s);
    CHECK(cmd(&s,"+TLDL=1",0,response)<0,"loop rejected without carrier");
    CHECK(cmd(&s,"+TTER=3,511,2,1",1,response)<0,"BER requires a loop");
    CHECK(cmd(&s,"+TLDL=1",1,response)==0,"start local loop");
    for(int pattern=1;pattern<=4;pattern++) {
        char command[80];
        snprintf(command,sizeof(command),"+TTER=3,511,3,%d",pattern);
        CHECK(cmd(&s,command,1,response)==0,"start supported pattern");
        int ones=0;
        if(pattern==1) {
            for(int i=0;i<511;i++)ones+=s.pattern[i];
            CHECK(ones==256,"511 pattern population");
        }
        if(pattern==3)CHECK(s.pattern[0]==1,"all-ones pattern");
        if(pattern==4)CHECK((s.pattern[0]^s.pattern[1])==1,"alternating pattern");
        for(int i=0;i<1533;i++) {
            int bit=at_test_get_bit(&s);
            at_test_put_bit(&s,bit^(i==10 || i==20 || i==520));
        }
        CHECK(s.type==0 && s.checked_bits==1533 && s.blocks_left==0,"finite block completion");
        CHECK(s.bit_errors==3 && s.block_errors==2,"distinct bit and block counts");
        CHECK(cmd(&s,"+TNUM?",0,response)==0 && !strcmp(response,"+TNUM: 3,2"),"retained counts");
        at_test_put_bit(&s,1);CHECK(s.checked_bits==1533,"no grading past target");
    }
    CHECK(cmd(&s,"+TTER=1,65535,65535,2",1,response)==0,"maximum blocks avoid overflow");
    at_test_clock_local(&s,17);
    CHECK(s.checked_bits==17 && s.type==1,"test remains in progress");
    CHECK(cmd(&s,"+TSELF=1",1,response)==0 && s.self_result==1,"self-test during a live BERT");
    CHECK(s.checked_bits==17 && s.type==1 && s.blocks_left==65535
        && s.tx_pos==17 && s.rx_pos==17,"safe check preserves live pattern state");
    CHECK(cmd(&s,"+TTER=0",1,response)==0 && s.checked_bits==17,"manual stop retains progress");
    CHECK(cmd(&s,"+TTER=2,10,1,4",1,response)==0,"block-only test");
    for(int i=0;i<10;i++)at_test_put_bit(&s,at_test_get_bit(&s)^(i==3));
    CHECK(cmd(&s,"+TNUM?",1,response)==0 && !strcmp(response,"+TNUM: 0,1"),"unavailable bit count is zero");
    s.last_type=3;s.bit_errors=70000;s.block_errors=65536;
    CHECK(cmd(&s,"+TNUM?",1,response)==0 && !strcmp(response,"+TNUM: 65535,65535"),"public counts saturate");
    const char *bad[]={"+TTER=3,0,1,1","+TTER=3,1,0,1","+TTER=3,1,1,0",
        "+TTER=3,1,1,1,2","+TTER=3,,1,1","+TTER=3,65536,1,1","+TLDL=1x",
        "+TLDL=","+TNUM=1","+TMODE=1","+TRDL=1","+TAL=1,0","+TSELF=0"};
    for(size_t i=0;i<sizeof(bad)/sizeof(bad[0]);i++)CHECK(cmd(&s,bad[i],1,response)<0,bad[i]);
    CHECK(cmd(&s,"+TSELF=1",0,response)==0 && s.self_result==1,"safe partial self-test");
    CHECK(s.bit_errors==70000 && s.block_errors==65536,"self-test preserves error counters");
    at_test_disconnect(&s);
    CHECK(!s.local_loop && !s.type,"disconnect stops diagnostics");
    CHECK(s.bit_errors==70000,"disconnect preserves last result");
    at_test_reset(&s);CHECK(s.bit_errors==0 && !s.local_loop,"power-on reset clears results");
}

static int fd=-1;
static int read_bytes(int f,uint8_t *out,int capacity,int wait_ms)
{
    int used=0;
    while(wait_ms>0 && used<capacity) {
        struct pollfd p={.fd=f,.events=POLLIN};
        int step=wait_ms>10?10:wait_ms;
        int ready=poll(&p,1,step);wait_ms-=step;
        if(ready>0) {int n=(int)read(f,out+used,(size_t)(capacity-used));if(n>0)used+=n;}
    }
    return used;
}
static void command(const char *text,const char *want)
{
    char line[180],response[2048];
    int n=snprintf(line,sizeof(line),"%s\r",text);
    CHECK(write(fd,line,(size_t)n)==n,"command write");
    n=read_bytes(fd,(uint8_t *)response,sizeof(response)-1,100);response[n]='\0';
    if(!strstr(response,want)) {
        fprintf(stderr,"FAIL: %s expected [%s], got [%s]\n",text,want,response);failures++;
    }
}
static void raw(int f)
{
    struct termios tio;
    CHECK(tcgetattr(f,&tio)==0,"read termios");cfmakeraw(&tio);
    CHECK(tcsetattr(f,TCSANOW,&tio)==0,"raw PTY");
}
static int dummy_set(const char *mode,bool automode) {(void)mode;(void)automode;return 0;}
static void dummy_get(char *mode,size_t size,bool *automode) {snprintf(mode,size,"v90");*automode=true;}
static void dummy_reset(void) {}

static void pty_tests(int split)
{
    char ctrl[100],data[100];
    snprintf(ctrl,sizeof(ctrl),"/tmp/at_test_ctrl_%ld",(long)getpid());
    snprintf(data,sizeof(data),"/tmp/at_test_data_%ld",(long)getpid());
    int opened=split?di_open_split(ctrl,data):di_open(ctrl);
    CHECK(opened==0,"open diagnostic PTY");if(opened<0)return;
    fd=open(ctrl,O_RDWR|O_NOCTTY|O_NONBLOCK);
    CHECK(fd>=0,"open control slave");if(fd<0){di_close();return;}
    raw(fd);
    int datafd=split?open(data,O_RDWR|O_NOCTTY|O_NONBLOCK):fd;
    CHECK(datafd>=0,"open data slave");if(datafd<0){close(fd);di_close();return;}
    raw(datafd);
    command("ATE0","OK");
    command("AT+TLDL=1","ERROR");
    command("AT+TLDL?","+TLDL: 0");
    command("AT+TTER=?","(0-3),(1-65535),(1-65535),(1-4)");
    command("AT+TNUM?","+TNUM: 0,0");
    command("AT+TSELF=0","ERROR");
    command("AT+TSELF=?","+TSELF: (1)");
    command("AT+TSELF=1","OK");
    command("AT+TRES?","+TRES: 1");
    command("AT+TRDL=1","ERROR");
    di_on_connected(9600);
    uint8_t drain[256];read_bytes(fd,drain,sizeof(drain),60);
    if(!split) {
        usleep(1100000);CHECK(write(fd,"+++",3)==3,"escape write");
        read_bytes(fd,drain,sizeof(drain),1150);
    }
    command("AT+TLDL=1+TLDL?","+TLDL: 1");
    command("AT+TTER=3,511,2,1","OK");
    usleep(150000);
    command("AT+TTER?","+TTER: 0,511,0,1");
    command("AT+TNUM?","+TNUM: 0,0");
    command("AT+TRES?","+TRES: 1");
    command("AT+TMODE=1","ERROR");
    command("AT+TRDL=1","ERROR");
    command("AT+TAL=1,0","ERROR");
    command("AT+TTER=3,511,65535,2","OK");
    command("AT+TTER=0","OK");
    if(!split)command("ATO","CONNECT");
    uint8_t input[256],output[256],peer[256];
    for(int i=0;i<256;i++)input[i]=(uint8_t)i;
    CHECK(write(datafd,input,sizeof(input))==(int)sizeof(input),"binary local loop write");
    int n=read_bytes(datafd,output,sizeof(output),500);
    CHECK(n==256 && !memcmp(input,output,256),"all 256 byte values loop exactly");
    CHECK(di_read_data(peer,sizeof(peer))==0,"test bytes cannot reach the line");
    CHECK(di_write_data((const uint8_t *)"REMOTE",6)==6,"line receive is consumed during loop");
    CHECK(read_bytes(datafd,output,sizeof(output),60)==0,"remote data is clamped from DTE");
    if(!split) {
        usleep(1100000);CHECK(write(fd,"+++",3)==3,"escape from local loop");
        read_bytes(fd,drain,sizeof(drain),1150);
    }
    command("AT+TLDL=0","OK");
    command("AT+TLDL?","+TLDL: 0");
    if(!split)command("ATO","CONNECT");
    CHECK(write(datafd,"NORMAL",6)==6,"normal DTE payload write");
    usleep(80000);
    CHECK(di_read_data(peer,sizeof(peer))==6 && !memcmp(peer,"NORMAL",6),"normal line route restored");
    CHECK(di_write_data((const uint8_t *)"BACK",4)==4,"normal line receive");
    CHECK(read_bytes(datafd,output,sizeof(output),100)==4 && !memcmp(output,"BACK",4),"normal DTE receive restored");
    if(split) {
        command("AT+TLDL=1","OK");
        command("AT+TTER=3,65535,65535,1","OK");
    }
    di_on_disconnected();read_bytes(fd,drain,sizeof(drain),60);
    command("AT+TLDL?","+TLDL: 0");
    command("AT+TTER?","+TTER: 0");
    command("ATZ","OK");command("ATE0","OK");
    command("AT+TTER?","+TTER: 0,0,0,0");
    command("AT+TRES?","+TRES: 0");
    if(split)close(datafd);close(fd);fd=-1;di_close();
}
int main(void)
{
    core_tests();
    di_set_modulation_ops(dummy_set,dummy_get,dummy_reset);
    pty_tests(1);pty_tests(0);
    printf("V.250 diagnostic tests: %s (%d failures)\n",failures?"FAIL":"PASS",failures);
    return failures?1:0;
}
