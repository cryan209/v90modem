/* Production engine/PTY integration for the fax page fixtures.
 * The software T.30 terminal acts only as the Class 1 DTE. Both physical
 * modem endpoints are unmodified v90_engine_peer processes, clocked by DS0.
 * No CONNECT, channel-ready or training state is injected into either engine.
 */
typedef struct {
    pid_t pid;
    int input, output, pty;
    int ready, attached, binary, escape, length, mode, connected, deferred_fdr;
    uint8_t frame[300];
    uint8_t audio[160];
    char path[128], log[128], text[4096];
    int text_len;
    hdlc_rx_state_t outgoing;
    fax_state_t *backend;
    int local_backend;
} engine_fax_side_t;
static engine_fax_side_t engine_sides[2];
static int engine_running, engine_sequence;
static void engine_poll_side(int k);

static void engine_io_fail(const char *operation)
{
    perror(operation);
    exit(1);
}
static void engine_write(int fd, const void *data, size_t len)
{
    const uint8_t *p = data;
    while (len) {
        ssize_t n = write(fd,p,len);
        if (n < 0 && (errno == EINTR || errno == EAGAIN)) { usleep(1000); continue; }
        if (n <= 0) engine_io_fail("fax engine write");
        p += n; len -= n;
    }
}
static void engine_read(int fd, void *data, size_t len)
{
    uint8_t *p = data;
    while (len) {
        ssize_t n = read(fd,p,len);
        if (n < 0 && errno == EINTR) continue;
        if (n < 0 && errno == EAGAIN) {
            /* A Class 2 reader can be writing a whole page while holding
             * its fax mutex. Drain PTYs while awaiting media, so neither
             * side of that real backpressure can deadlock the test driver. */
            if (engine_running) for (int k=0;k<2;k++) engine_poll_side(k);
            usleep(1000);
            continue;
        }
        if (n <= 0) engine_io_fail("fax engine read");
        p += n; len -= n;
    }
}
static void engine_backend_bit(engine_fax_side_t *s, int bit)
{
    if (s->local_backend) fc2_v34hdx_put_bit(bit);
    else fax_v34hdx_put_bit(s->backend,bit);
}
static void engine_backend_channel(engine_fax_side_t *s, int mode)
{
    if (!s->attached || mode == s->mode) return;
    if (getenv("FAX_TEST_LOG")) fprintf(stderr,"ENGINE CHANNEL side=%ld mode=%d\n",s-engine_sides,mode);
    if (mode == V34_HALF_DUPLEX_PRIMARY_CHANNEL) {
        /* The DTE sees the modem's channel indication, not physical marks.
         * Reproduce that front-end event for the software T.30 DTE. */
        for (int i=0;i<40;i++) engine_backend_bit(s,1);
    }
    if (s->local_backend) fc2_v34hdx_set_channel(mode);
    else fax_v34hdx_set_channel(s->backend,mode);
    s->mode = mode;
}
static void engine_serial_byte(engine_fax_side_t *s, int c)
{
    if (!s->binary) {
        if (c != 0x10) return;
        s->binary = 1;
    }
    if (!s->escape && c == 0x10) { s->escape=1; return; }
    if (s->escape) {
        s->escape=0;
        if (c == 0x6B || c == 0x6D) {
            engine_backend_channel(s,c == 0x6B ? V34_HALF_DUPLEX_PRIMARY_CHANNEL : V34_HALF_DUPLEX_CONTROL_CHANNEL);
            s->length=0; return;
        }
        if (c == 3 || c == 7) {
            if (getenv("FAX_TEST_LOG")) fprintf(stderr,"ENGINE RECEIVE side=%ld mode=%d fcf=%02x len=%d ok=%d\n",s-engine_sides,s->mode,s->length>2?s->frame[2]:0,s->length,c==3);
            if (s->length >= 2 && s->attached) {
                hdlc_tx_state_t *tx = hdlc_tx_init(NULL,false,2,false,NULL,NULL);
                hdlc_tx_flags(tx,3);
                hdlc_tx_frame(tx,s->frame,s->length-2);
                if (c == 7) hdlc_tx_corrupt_frame(tx);
                for (int i=0;i<s->length*10+80;i++) engine_backend_bit(s,hdlc_tx_get_bit(tx));
                hdlc_tx_free(tx);
            }
            s->length=0; return;
        }
        if (c == 0x51) c=0x11;
        else if (c == 0x53) c=0x13;
        else if (c != 0x10) return;
    }
    if (s->length < (int)sizeof(s->frame)) s->frame[s->length++]=c;
    else engine_io_fail("fax serial frame overflow");
}
static void engine_poll_side(int k)
{
    engine_fax_side_t *s = &engine_sides[k];
    uint8_t data[4096];
    ssize_t n;
    while ((n=read(s->pty,data,sizeof(data))) > 0) {
        if (s->text_len+n < (int)sizeof(s->text)-1) {
            memcpy(s->text+s->text_len,data,n); s->text_len+=n; s->text[s->text_len]=0;
        }
        if (strstr(s->text,"CONNECT")) s->ready=1;
        if (strstr(s->text,"+FCO")) s->connected=1;
        if (k == 0 && external_class21) dce_write(data,n,NULL);
        else if (s->ready) for (int i=0;i<n;i++) engine_serial_byte(s,data[i]);
        if (s->connected && s->deferred_fdr) {
            s->deferred_fdr=0;
            engine_write(s->pty,"AT+FDR\r",7);
        }
    }
}
static void engine_command(int k, const char *line, int wait)
{
    engine_fax_side_t *s=&engine_sides[k];
    s->text_len=0; s->text[0]=0;
    engine_write(s->pty,line,strlen(line)); engine_write(s->pty,"\r",1);
    if (wait) {
        for (int i=0;i<500 && !strstr(s->text,"OK") && !strstr(s->text,"ERROR");i++) {
            usleep(1000); engine_poll_side(k);
        }
        check(strstr(s->text,"OK") != NULL,line);
    }
}
static void engine_start(void)
{
    if (engine_running) return;
    signal(SIGPIPE,SIG_IGN);
    engine_sequence++;
    memset(engine_sides,0,sizeof(engine_sides));
    for (int k=0;k<2;k++) {
        engine_fax_side_t *s=&engine_sides[k];
        int in[2],out[2];
        if (pipe(in) || pipe(out)) engine_io_fail("fax pipe");
        snprintf(s->path,sizeof(s->path),"/tmp/fax_engine_%d_%d_%d",getpid(),engine_sequence,k);
        snprintf(s->log,sizeof(s->log),"%s.log",s->path);
        s->pid=fork();
        if (s->pid < 0) engine_io_fail("fax fork");
        if (!s->pid) {
            int log=open(s->log,O_WRONLY|O_CREAT|O_TRUNC,0600);
            if (log < 0) _exit(2);
            dup2(in[0],0); dup2(out[1],1); dup2(log,2);
            close(in[1]); close(out[0]);
            setenv("ME_MEDIA_CLOCK","1",1);
            const char *args[6]; int n=0;
            args[n++]="./v90_engine_peer"; args[n++]=s->path;
            if ((k == 0) == !!external_source) args[n++]="--call";
            if (external_alaw) args[n++]="--alaw";
            args[n]=NULL; execv(args[0],(char *const *)args); _exit(2);
        }
        close(in[0]); close(out[1]); s->input=in[1]; s->output=out[0];
        fcntl(s->input,F_SETFD,FD_CLOEXEC); fcntl(s->output,F_SETFD,FD_CLOEXEC);
        fcntl(s->output,F_SETFL,O_NONBLOCK);
        s->pty=-1;
        for (int i=0;i<500 && s->pty < 0;i++) {
            s->pty=open(s->path,O_RDWR|O_NOCTTY|O_NONBLOCK);
            if (s->pty < 0) usleep(10000);
        }
        if (s->pty < 0) engine_io_fail("fax PTY");
        fcntl(s->pty,F_SETFD,FD_CLOEXEC);
        memset(s->audio,external_alaw?0xD5:0xFF,sizeof(s->audio));
        engine_command(k,k == 0 && external_class21 ? "AT+FCLASS=2.1" : "AT+FCLASS=1.0",1);
        if (k != 0 || !external_class21) engine_command(k,"AT+F34=4,1,1",1);
        s->local_backend=k == 0;
        s->mode=V34_HALF_DUPLEX_CONTROL_CHANNEL;
    }
    engine_running=1;
}
static int engine_at(const char *line)
{
    engine_start();
    /* A Class 2 DTE waits for +FCO before requesting receive data. The
     * direct fixture's on_connected call is deliberately not injected. */
    if (!strcmp(line,"AT+FDR") && !engine_sides[0].connected) {
        engine_sides[0].deferred_fdr=1;
        return 1;
    }
    engine_command(0,line,!strncmp(line,"AT+F",4) && strncmp(line,"AT+FDT",6) && strncmp(line,"AT+FDR",6));
    if (!strncmp(line,"ATD",3) || !strcmp(line,"ATA")) dial_seen=1;
    return 1;
}
static void engine_send_frame(void *user, const uint8_t *msg, int len, int ok)
{
    engine_fax_side_t *s=user;
    uint8_t out[600]; int n=0;
    if (len <= 0 || !ok) return;
    if (getenv("FAX_TEST_LOG")) fprintf(stderr,"ENGINE SEND side=%ld mode=%d fcf=%02x len=%d\n",s-engine_sides,s->mode,msg[2],len);
    for (int i=0;i<len;i++) {
        int c=msg[i];
        if (c == 0x10 || c == 0x11 || c == 0x13) {
            out[n++]=0x10; out[n++]=c == 0x11 ? 0x51 : c == 0x13 ? 0x53 : c;
        } else out[n++]=c;
    }
    out[n++]=0x10; out[n++]=3;
    engine_write(s->pty,out,n);
}
static void engine_drive_backend(engine_fax_side_t *s)
{
    if (!s->attached) return;
    int wanted=s->local_backend ? fc2_v34hdx_get_mode() : fax_v34hdx_get_mode(s->backend);
    if (wanted != s->mode) {
        int source=s->local_backend ? external_source : !external_source;
        if (source) {
            uint8_t cmd[2]={0x10,wanted == V34_HALF_DUPLEX_PRIMARY_CHANNEL ? 0x6B : 0x6D};
            engine_write(s->pty,cmd,2);
            engine_backend_channel(s,wanted);
            hdlc_rx_init(&s->outgoing,false,true,2,engine_send_frame,s);
        }
    }
    int bits=external_block*(s->mode == V34_HALF_DUPLEX_PRIMARY_CHANNEL ? 9600 : 1200)/8000;
    for (int i=0;i<bits;i++) {
        int bit=s->local_backend ? fc2_v34hdx_get_bit() : fax_v34hdx_get_bit(s->backend);
        hdlc_rx_put_bit(&s->outgoing,bit);
    }
    if (s->local_backend) fc2_v34hdx_advance(external_block);
    else fax_v34hdx_advance(s->backend,external_block);
}
static void pump_engine_fax(fax_state_t *peer)
{
    engine_start();
    for (int k=0;k<2;k++) {
        engine_fax_side_t *s=&engine_sides[k];
        engine_poll_side(k);
        if (s->ready && !s->attached && (k || !external_class21)) {
            s->backend=peer;
            int r=k ? fax_v34hdx_start_control(peer,9600) : fc2_v34hdx_start_control(9600);
            check(r == 0,"software T.30 DTE attaches after production engine CONNECT");
            s->attached=1;
            hdlc_rx_init(&s->outgoing,false,true,2,engine_send_frame,s);
        }
        if (k || !external_class21) engine_drive_backend(s);
    }
    uint8_t next[2][160];
    for (int k=0;k<2;k++) {
        uint8_t header[2]={external_block&255,external_block>>8};
        engine_write(engine_sides[k].input,header,2);
        engine_write(engine_sides[k].input,engine_sides[1-k].audio,external_block);
        engine_read(engine_sides[k].output,next[k],external_block);
    }
    for (int k=0;k<2;k++) memcpy(engine_sides[k].audio,next[k],external_block);
    usleep(1000); /* PTY reader threads must run between bearer ticks. */
    for (int k=0;k<2;k++) engine_poll_side(k);
}
static void engine_reset(void)
{
    if (!engine_running) return;
    for (int k=0;k<2;k++) close(engine_sides[k].input);
    for (int k=0;k<2;k++) {
        engine_fax_side_t *s=&engine_sides[k]; int status;
        close(s->output); close(s->pty); waitpid(s->pid,&status,0); unlink(s->path);
        check(WIFEXITED(status) && WEXITSTATUS(status) == 0,"production engine exits cleanly");
        FILE *f=fopen(s->log,"r"); char text[65536]; size_t n=f ? fread(text,1,sizeof(text)-1,f) : 0;
        if (f) fclose(f); text[n]=0;
        check(strstr(text,"T.30 Annex F attached to V.34 control channel") != NULL,
              "production engine negotiated V.34 and attached its fax transport");
        printf("       engine log: %s\n",s->log);
    }
    engine_running=0;
}
static void test_fc2_select(int selected)
{
    if (engine_fax_test && !selected) engine_reset();
    if (engine_fax_test && selected) engine_start();
    fc2_select(selected);
}
static void test_fc2_dte_bytes(const uint8_t *data, int len)
{
    if (engine_fax_test && external_class21) { engine_write(engine_sides[0].pty,data,len); return; }
    fc2_dte_bytes(data,len);
}
static void test_fc2_on_connected(void)
{
    if (!(engine_fax_test && external_class21)) fc2_on_connected();
}
static void test_fc2_on_disconnected(void)
{
    if (!(engine_fax_test && external_class21)) fc2_on_disconnected();
}
static void test_fc2_poll(void)
{
    if (engine_fax_test && external_class21) { usleep(5000); engine_poll_side(0); }
    else fc2_poll();
}
