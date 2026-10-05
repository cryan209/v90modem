# V.250 command conformance audit

Date: 2026-10-05. Source: ITU-T V.250 (07/2003),
`ITU Docs/T-REC-V.250-200307-I!!PDF-E.pdf`.
Scope: modulation selection/reporting, error control, compression, relevant
local-port controls, and V.92 controls. This is not a complete certification of
all V.250 clauses, basic Hayes commands, V.58 tests, or fax/voice extensions.
No runtime implementation was changed by this audit.

## Method and confidence

Trace: PTY -> SpanDSP AT interpreter -> `data_interface.c` -> modem engine /
`data_stack.c`. A standalone probe invokes the same AT interpreter directly,
with output capture and a modem-control callback that prints every invocation.
The interpreter, clear-channel code, and +MS parser were compiled directly
from current source; supporting functions came from the local SpanDSP archive.
Repeating with the archive's interpreter produced identical output, ruling
out a stale interpreter object as the explanation for these observations.

These probes demonstrate parsing, replies, discarded/stored state, and the
absence of relevant callbacks. Engine-policy conclusions below come from
source tracing, not a newly placed call. No hardware or SIP call was made.

V.250 5.4.4.2 requires implemented parameters to store valid settings; 5.4.4.3
requires reads to report those settings. Clause 6 makes commands associated
with implemented optional facilities required. Accepting a setting with OK
while discarding it does not meet that contract. Supporting a smaller honest
subset is preferable to test responses advertising unusable settings.

## Findings and priority

| ID | Priority / clauses | Observed implementation | Required change |
|---|---|---|---|
| AT-1 | High; 6.4.1 | `handle_plus_ms()` keeps all rates in `ms_cur` for readback, but passes only mapped mode and automode to the engine. `=V34,0,0,9600` maps to unrestricted `v34`; `=V32B,0,0,4800` maps to unrestricted `v32bis`. `=V120,0,0,1200` selects 56k despite the requested maximum. | Carry an immutable next-call configuration containing directional min/max bounds into offers and negotiated rate selection. Fail a connection outside the bounds. Reject unsupported settings until enforced. The V.22/V.22bis 1200 cap and CLEAR/V120 56k selector are the only current +MS-derived caps; the V32 mode's fixed 9600 cap is separate. |
| AT-2 | High; 6.4.3 | `+MR=1` returns OK; `+MR?` returns 0. Handler passes a NULL storage target. No +MCR/+MRR report path exists in the engine/DI integration inspected. | Store reporting enable and emit actual settled modulation and directional rates before +ER/+DR and CONNECT. This parameter/reporting is mandatory for conforming V-series data modems. |
| AT-3 | High; 6.5.1 | `+ES=3,3,5` returns OK; read returns `0,0,0`. All parser targets are NULL. Framing/detection is instead selected from `ME_DATA_FRAMING` and engine policy. | Implement the supported V.42 detection/fallback policies, including required-LAPM failure and disabled-error-control modes. Reject sync, alternative-protocol, or other modes until implemented. |
| AT-4 | High; 6.6.1, 6.6.2 | `+DS=0` returns OK without disabling engine compression; read returns only one zero. Standard `+DS=3,0,1024,32` returns ERROR because the handler parses one numeric value with maximum 1. `+DS44` is a skip-only TODO. Engine compression is chosen by `ME_DATA_COMPRESSION`. | Connect four-field V.42bis and nine-field V.44 policy to `ds_init_v42_ex()` / `ds_init_v44()` and negotiation outcomes. Enforce require-compression policy and supported dictionary/string/history limits. Clause 6.6.1 is mandatory when V.42bis is implemented; 6.6.2 when V.44 is implemented. |
| AT-5 | High; 6.5.5, 6.6.3 | `+ER=1` and `+DR=1` return OK, then read as 0. NULL storage targets; no corresponding report emissions found. | Emit negotiated error-control/compression reports, in order after modulation reports and before CONNECT. Derive them from settled protocol state, not requested settings. |
| AT-6 | High; 6.8.2-6.8.8 | `+PMH=1`, `+PIG=1`, `+PQC=3`, `+PSS=2` return OK without applying policy. `+PMH?` produces no information. Bare `+PMHR` and `+PMHF` return OK while idle. | Connect supported controls to the existing PCM-upstream, short-startup, and MOH machinery. In particular PMHR must return ERROR while idle or MOH-disabled; PMHF must return ERROR unless on hold. Translate PMHT's Table 33 values, rather than assuming environment-variable encodings are the public command API. These requirements are conditional on implementing V.92. |
| AT-7 | Medium; 6.5.6 | `+ETBM=2,2,20` returns OK; read is only `0,0`. Handler parses two NULL targets and does not implement/store the timer. | Implement all three fields and termination drain/discard/deadline policy. Test queued TX and RD data independently. The command is mandatory when V.42 or buffered mode is implemented. |
| AT-8 | Medium; 6.4.8 | `+MSC=1` returns OK; read returns 0; NULL storage target. | Connect seamless-rate-change policy if supported; otherwise reject it. Optional capability, not an obligation to implement the feature. |
| AT-9 | Medium; 6.5.4, 6.5.7, 6.5.8 | `+EFCS=2` is discarded and reads 0. `+EFRAM=256,128` is discarded and reads `0,0`. `+EWIND=7,9` persists in interpreter fields only; no downstream uses of those fields were found. EWIND stores value1 as rx and value2 as tx, opposite the spec's transmit/receive ordering. | Restrict FCS to implemented choices; connect frame/window limits to V.42 negotiation, correctly ordered, with value2 omitted/zero inheritance. EFCS/EWIND/EFRAM optional status does not permit pretending to apply them. |
| AT-10 | Medium; 6.2.10-6.2.13 | IPR, ICF, IFC persist in interpreter fields, but DI does not use them for line framing, virtual rate adaptation, or flow control. `+IPR=9600` succeeds despite a test response advertising 115200 only. `+ILRR=1` returns OK without reporting. | Define PTY semantics and the supported virtual DTE profile. Implement software flow control and serial framing only when supported, or advertise/reject honestly. Do not emulate physical pins that the PTY lacks. |
| AT-11 | Lower; 6.4.2 | `+MA?` returns bare OK with no list. The handler is a skip-only TODO. | Optional feature: reject unsupported forms or implement an actual fallback list and +MS interaction. |
| AT-12 | Medium; 6.1.9 | `+GCAP` reports only `+FCLASS`. +MS works, but associated +MR does not; +ES/+DS families are incomplete. | Generate capabilities from complete supported command families. Do not simply add +MS/+ES/+DS tokens to claim facilities before their required controls/reporting work. |

Other V.18 and V.58 test handlers contain stubs; they were not functionally
audited here. V.251 +A8 controls are a separate audit, not V.250 findings.

## Representative probe transcript

```text
AT+MR=1             -> OK
AT+MR?              -> +MR:0; OK
AT+ES=3,3,5         -> OK
AT+ES?              -> +ES:0,0,0; OK
AT+DS=0             -> OK
AT+DS?              -> +DS:0; OK
AT+DS=3,0,1024,32   -> ERROR
AT+EFCS=2           -> OK
AT+EFCS?            -> +EFCS:0; OK
AT+ETBM=2,2,20      -> OK
AT+ETBM?            -> +ETBM:0,0; OK
AT+EWIND=7,9        -> OK
AT+EWIND?           -> +EWIND:7,9; OK
AT+IPR=9600         -> OK
AT+IPR?             -> +IPR:9600; OK
AT+PMH?             -> OK (no information response)
AT+PMHR             -> OK (idle; expected ERROR)
AT+PMHF             -> OK (not on hold; expected ERROR)
```

## Implementation sequence and acceptance checks

1. Introduce a next-call control configuration with validated supported
   settings, reset semantics, and the existing leaf-lock discipline.
   Do not acquire the engine's state lock while holding `t31_mtx`.
2. Make write/read/test responses and unsupported forms honest. Validate
   every field before committing changes; a rejected compound setting must
   retain previous values (5.4.4.2).
3. Enforce +MS bounds and +ES policy in paired-engine calls. Demonstrate both
   negotiated directions, exact-bound rejection, required-LAPM failure,
   and disabled-error-control operation. Environment overrides must have
   explicit precedence; they must not silently defeat accepted AT settings.
4. Wire compression configuration and negotiated +MCR/+MRR/+ER/+DR reports.
   Assert transcript ordering before CONNECT and actual wire negotiation,
   not just successful readback. Test V.42bis and V.44 separately.
5. Wire supported V.92 controls with idle/disabled/on-hold state tests.
   Leave unimplemented optional features rejected until their behaviour is
   demonstrable. Add buffered termination and local-port features separately.

The existing `at_ms_test` checks parsing and modulation offers but includes
rate-setting readback without proving rate negotiation. Those checks can pass
while AT-1 remains. Likewise, storing EWIND/IPR/IFC fields is not evidence that
the engine or PTY obeys them.

## Standalone reproduction

Save the following as `/tmp/itu_control_audit.c`, then compile from the repo
root. This runs no SIP calls and uses no hardware. On macOS with Homebrew:

```sh
cc -DHAVE_CONFIG_H -I. -Ispandsp-master/src -Ispandsp-master \
  $(pkg-config --cflags libtiff-4) /tmp/itu_control_audit.c \
  clear_channel.c at_ms.c spandsp-master/src/at_interpreter.c \
  spandsp-master/src/.libs/libspandsp.a -lm -o /tmp/itu_control_audit
/tmp/itu_control_audit
```

The V.120 portion feeds independently HDLC-encoded frame bits through the
public DS0 path, rather than directly invoking the private frame callback.
The +MS portion tests the real parser/mode mapping; its modem-control stub
does not substitute for PTY/engine integration.

```c
#include <stdio.h>
#include <string.h>
#include <spandsp.h>
#include "clear_channel.h"
#include "at_ms.h"
static int tx(void *u, const uint8_t *b,size_t n){(void)u;for(size_t i=0;i<n;i++)if(b[i]!='\r'&&b[i]!='\n')putchar(b[i]);else putchar('|');return (int)n;}
static int ctl(void *u,int op,const char *num){(void)u;printf("[control %d %s]",op,num?num:"");return 0;}
static int pull(void *u){(void)u;return -1;}
static void push(void *u,uint8_t b){(void)u;printf("%02x ",b);}
static void frame(const char *name,const uint8_t *p,int n){
 clear_channel_t rx;cc_init_v120(&rx,0,1,pull,push,NULL);
 hdlc_tx_state_t *h=hdlc_tx_init(NULL,0,1,0,NULL,NULL);hdlc_tx_flags(h,2);hdlc_tx_frame(h,p,n);
 uint8_t line[400];for(int i=0;i<400;i++){line[i]=0;for(int b=0;b<8;b++){int v=hdlc_tx_get_bit(h);line[i]|=(v<0?1:v&1)<<(7-b);}}
 printf("%s delivered=",name);cc_rx(&rx,line,400);printf(" bytes=%llu bad=%llu unsupported=%llu\n",(unsigned long long)rx.rx_data_bytes,(unsigned long long)rx.rx_bad_frames,(unsigned long long)rx.rx_unsupported);cc_release(&rx);hdlc_tx_free(h);
}
int main(void){
 at_state_t *a=at_init(NULL,tx,NULL,ctl,NULL);
 const char *cmds[]={"ATE0","AT+MR=1","AT+MR?","AT+ER=1","AT+ER?","AT+DR=1","AT+DR?","AT+ES=3,3,5","AT+ES?","AT+DS=0","AT+DS?","AT+DS=3,0,1024,32","AT+MSC=1","AT+MSC?","AT+EFCS=2","AT+EFCS?","AT+ETBM=2,2","AT+ETBM?","AT+ETBM=2,2,20","AT+EWIND=7,9","AT+EWIND?","AT+EFRAM=256,128","AT+EFRAM?","AT+IPR=9600","AT+IPR?","AT+ICF=5,1","AT+ICF?","AT+IFC=1,1","AT+IFC?","AT+PMH=1","AT+PMH?","AT+PMHR","AT+PMHF","AT+PIG=1","AT+PQC=3","AT+PSS=2","AT+MA?","AT+ILRR=1","AT+GCAP"};
 for(unsigned i=0;i<sizeof(cmds)/sizeof(*cmds);i++){char s[100];snprintf(s,sizeof(s),"%s\r",cmds[i]);printf("\n%s -> ",cmds[i]);at_interpreter(a,s,(int)strlen(s));}puts("");at_free(a);
 clear_channel_t c;uint8_t hdr[4];cc_init_v120(&c,0,0,pull,push,NULL);cc_v120_frame_header(&c,hdr);printf("answer UI header %02x %02x %02x %02x\n",hdr[0],hdr[1],hdr[2],hdr[3]);cc_release(&c);
 frame("valid",(const uint8_t *)"\x08\x01\x03\x83""AB",6);
 frame("invalid CS E=0 then payload",(const uint8_t *)"\x08\x01\x03\x03\x00\x80""AB",8);
 frame("missing CS",(const uint8_t *)"\x08\x01\x03\x03",4);
 frame("other LLI",(const uint8_t *)"\x08\x03\x03\x83""AB",6);
 frame("segmented async",(const uint8_t *)"\x08\x01\x03\x82""AB",6);
 at_ms_settings_t ms;const char *v[]={"=V34,0,0,9600","=V32B,0,0,4800","=V120,0,0,1200","=V34,0,9600,9600", "=,0", "=V34"};
 for(unsigned i=0;i<sizeof(v)/sizeof(*v);i++){int r=at_ms_parse(v[i],&ms);printf("MS %s op=%d mode=%s\n",v[i],r,r==AT_MS_SET?at_ms_settings_to_mode(&ms):"-");}
 return 0;
}
```
