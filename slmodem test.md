--end msg--
10:11:12.319      inv0x5ce2574d39d8  ....SDP negotiation done: Success
10:11:12.319          pjsua_media.c  .....Call 0: updating media..
10:11:12.319          pjsua_media.c  ......Call 0: stream #0 (audio) unchanged.
10:11:12.319          pjsua_media.c  ......Audio updated, stream #0: PCMA (sendrecv)
on_call_media_state: callback
<072.703684> Found 2100
<072.703703> call: process: msg 0 --> 16
<072.703706> slamr0: 61632: change dp: --> 8...
<072.703710> v8: create: caller 1, automode 0, dp id 92.
<072.703723> V8: Create called, V8 version 23/09/03 .
<072.703726> ############################################################
<072.703728> V8: local configuration : 
<072.703729> 	Side = Caller
<072.703732> 	Operation Mode = 0
<072.703735> 	Modulations - V90=1, V34=1, V34HD=0, V32=1, V22=0, V17=0, V29=0, V27=0, V23=0, V21=0
<072.703739> 	Call Functions - Data=1, CallRxFax=0, CallTxFax=0, V.80=0
<072.703742> 	Protocol - LAPM V.42
<072.703744> 	v8bisIndication - 0
<072.703747> 	timeouts - signal detect 12 sec, message detect 7 sec
<072.703749> 	quickConnectEnabled - 1
<072.703751> 	lapmIndication - 1
<072.703753> 	ucodeForQts - 9
<072.703757> 	ansPcmLevel - 0
<072.703759> ############################################################
<072.703762> V8: Initial CM message length is 9 octets
<072.703765> call: delete...
<072.703768> Dialer was aborted.
<072.703770> CALLPROG_Delete is entered
<072.703772> cadence_delete with CADENCE_DIAL_OBJ is invoked
<072.703777> cadence_delete with CADENCE_OBJ is invoked
<072.703780> CALLPROG_Delete is exited
<072.711888> V8: State changed from V8_INIT to V8_ORG_WAITING_FOR_ANSAM
<072.711907> v8: status (6) V8_ORG_WAITING_FOR_ANSAM
<072.772185> V8: State changed from V8_ORG_WAITING_FOR_ANSAM to V8_ORG_ANSAM_DETECTED_WAITING_TE
<072.772203> v8: status (7) V8_ORG_ANSAM_DETECTED_WAITING_TE
<073.432352> ANSAM phase reversals detected delay = 450
<073.772185> V8 ANSAM Detected (CM ready)
<073.772209> V8: State changed from V8_ORG_ANSAM_DETECTED_WAITING_TE to V8_ORG_SEND_QC
<073.772212> v8: status (14) V8_ORG_SEND_QC
<073.972303> V8: State changed from V8_ORG_SEND_QC to V8_ORG_SEND_CM
<073.972321> v8: status (8) V8_ORG_SEND_CM
<077.151675> V8: on CALLER: remote call function is: 107
<077.151690> V8: call function DATA indication...
<077.151692> V8: Got Call Function Match!
<077.171808> V8: State changed from V8_ORG_SEND_CM to V8_ORG_SEND_CJ
<077.171823> v8: status (10) V8_ORG_SEND_CJ
<077.291399> V8: State changed from V8_ORG_SEND_CJ to V8_OK
<077.291414> v8: process: OK.
<077.291416> V8Report: remote V90: mod - 1, digital connection - 1, pcmIndication - 1
<077.291418> V8Report: v90:1, v34:1, v34hd:0, V32:1, V22:0, V17:0, V29:0, V27:0, V23:0, V21:0
<077.291421> main: socket_ioctl: cmd 10, arg 0...
<077.291423> main: socket_ioctl: returning c0
<077.291424> v8: Link established. Idle timer 864.
<077.291426> v8: status (13) V8_OK
<077.291428> slamr0: 105792: change dp: --> 90...
<077.291431> vpcm: create: dp 90, caller 1, frag 48 (size 53848).
<077.291471> VPCMXF_Create: side is Analog, maxDataBuffer - 48
<077.291552> V90Modem Construction (as Analog Modem)
<077.291563> $!$ 6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A1F3?????
<077.291567> $!$ 6@4A3C5?:A6@4A3955:766473955:7667;7A9>?:668@8<55>866776;7>==877;5>55?=:;998<9>>F:D473955:766473955:76@4A3C5?:A6@4A1F3?????
<077.291569> $!$ 6@4A3C5?:A6@4A3955:766477?9D?9669;7A9::7;;9:7>55>F:<476<7A:78:7:6955>>;88F8>:5:7:E8E7E:>:766473955:76@4A3C5?:A6@4A1F3?????
<077.291570> *********************************************************
<077.291572> V90Modem Version: 2.98  (25-Mar-04)
<077.291573> *********************************************************
<077.291575> $!$ 9<5@497B>F::8<7F55==:;998<9>>F:D475=9:?::9997B:5?;:?8F7G6?8D4@????
<077.291578> $!$ 8C8F7=9>>=:?8<7=55=8;;8@7<9@:7898F7G9C><:99;39:<>@;:8?7H::?;666D7>9B>F;89@395@:79:997:9>>E667;7B9B><664B3978>F:D9:8=9:>C66777H:<><;82D1C????
<077.291580> $!$ 898F7F:5>F:D8<7G:9?:7@475?9A>F;88<7::9:C66685=7>:C66685<79:C666E7>:<:7886C5E3B8A????
<077.291581> *********************************************************
<077.291584> V90Parameters: upStream min rate : 4800 upStream max rate : 4800  Rate mask :1
<077.291588> $!$ 9<5@4985>8;8887F9:?;:;998<6?:78:8<7;::>>664D39:9><:C976987:77C47493B8A????
<077.291590> $!$ 9<5@4985>8;8887F9:?;:;998<6?:78:8<7;::>>664D39:9><:C9769:7>F:88<396B:7762D1C????
<077.291592> $!$ 9<5@4985>8;8887F9:?;:;998<6?:78:8<7;::>>664D39:9><:C975<9D>E:D8<7<:9>@:E8E6=:>?7:;474F55;74C2A????
<077.291595> $!$ 9<5@4985>8;8887F9:?;:;998<6?:78:8<7;::>>664D39:9?9:D597=8D>D:;887G8D><;8997H:7=F;99;7=8D><;<887E::>8;:8@7H9C=F:;8E7:97>C:;474F55;84C2A????
<077.291599> V90PreFilter: HardwareCodecType: Panther_AD1803
<077.291916> $!$ 9<5@4987><;9887F:5>C:;994C55?:;:888=9::77C475?87<F9@6<5G3B8A????
<077.291987> V90ConnectionEvaluator reset called !
<077.291990> $!$ 664A3C5?:A6@4A3C5?:A6@4A397:>::>8F3987?9:D475F9:>::>887G9>?::C473C5?:A6@4A3C5?:A6@4A3C558A????
<077.291995> V90Phase3Demodulator: Reset called, sessionFlag = 1 !
<077.291999> V90Phase3Demodulator: initial state set to WaitForSd
<077.292006> $!$ 9<5@497:?8;;887E9>?A:;994C55<:;88<7::9><::473F55<D8C7?399B>F::8<399>?:668;7B:8>8:88C7>99:E4C2A????
<077.292009> $!$ 9<5@497:?8;;887E9>?A:;994C55?9:;9:7>:98D4@????
<077.292012> $!$ 9<5@497:?8;;887E9>?A:;994C55<C8;475;9:?;:7474F55:D764E4965;776577>5B;8762D1C????
<077.292014> $!$ 9<5@497:?8;;887E9>?A:;994C55<;8<6<3977><;:88396B:76C573G65;77657499::D7=2D1C????
<077.292032> V92Modem Construction (as Analog Modem)
<077.292036> $!$ 6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A1F3?????
<077.292040> $!$ 6@4A3C5?:A6@4A3955:766473955:7667;7A9>?:668@8<55>866776;7>==877;5>55?=:;998<9>>F:D473955:766473955:76@4A3C5?:A6@4A1F3?????
<077.292043> $!$ 6@4A3C5?:A6@4A3955:766477?9D?9669;7A9::7;;9:7>55>F:<476<7A:78:7:6955>>;88F8>:5:7:E8E7E:>:766473955:76@4A3C5?:A6@4A1F3?????
<077.292046> *********************************************************
<077.292047> V92Modem Version: 1.1  (9-Apr-01)
<077.292049> *********************************************************
<077.292051> $!$ 9<5@4;7B>F::8<7F55==:;998<9>>F:D475=9:?::9997B:5?;:?8F7G6?8D4@????
<077.292055> $!$ 8C8<7F9D?9;?477<9A><:78E8>:5?:664B3999>@:B477=9:?::9997B:5?;:E993998?9:79:7A55>=:?9?1F3?????
<077.292058> $!$ 6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A1F3?????
<077.292119> V92Modulator constraction
<077.292495> $!$ 9<5@4;85>?:79:7>68<D:E8;8>9A>8;:8F8;6?:79:795G66?<669:8=96?;:;477E9:>E:=9;7A55?::;9;39:9>F66594965;77:2D1C????
<077.292499> V92Modulator reset
<077.292503> V92EchoCanceller: constraction
<077.292505> $!$ 9<5@4;7:>::>8F5<96>E:98<7E9A><;85A399:>::>8F5=9:>C:79@39::?7::888=9:>;669;7H6?:77>5;493?????
<077.292508> $!$ 9<5@4;7:>::>8F5<96>E:98<7E9A><;85A399:>::>8F5?9>>C;:8<8;7A><:D474F55;87>571F3?????
<077.292512> $!$ 9<5@4;7:>::>8F5<96>E:98<7E9A><;85A399:>::>8F5;9:?;:7474F55:D764E4965;77657493B8A????
<077.292515> $!$ 9<5@4;7:>::>8F5<96>E:98<7E9A><;85A399:>::>8F5;9:?;:76;7>98>8;?474F55:D764E4965;77657493B8A????
<077.292521> vpcm: VPCM rate limits: 300-56000
<077.292523> main: socket_ioctl: cmd 10, arg 0...
<077.292525> main: socket_ioctl: returning c0
<077.292526> vpcm: Delays: HW 196, DMA 148
<077.292528> vpcm: initial dp V.90, session type 1.
<077.292530> VPcmV34Create: quick connect indication from phase1 = 0
<077.292531> VPcmV34Create: Uqts index is 9
<077.292535> VPcmV34Create: ANSpcm level index is 0
<077.292538> VPcmV34Create, initial Session Type = 1
<077.292540> RX at 630F9DC0
<077.292546> On Create: Setting desired TX MD (0 mSec)!
<077.292551> V90Parameters: upStream min rate : 4800 upStream max rate : 33600  Rate mask :1fff
<077.292554> $!$ 9<5@4985>8;8887F9:?;:;998<6?:78:8<7;::>>664D39:9><:C976987:77C47493B8A????
<077.292557> $!$ 9<5@4985>8;8887F9:?;:;998<6?:78:8<7;::>>664D39:9><:C9769:7>F:88<396B:7762D1C????
<077.292559> $!$ 9<5@4985>8;8887F9:?;:;998<6?:78:8<7;::>>664D39:9><:C975<9D>E:D8<7<:9>@:E8E6=:>?7:;474F55;74C2A????
<077.292562> $!$ 9<5@4985>8;8887F9:?;:;998<6?:78:8<7;::>>664D39:9?9:D597=8D>D:;887G8D><;8997H:7=F;99;7=8D><;<887E::>8;:8@7H9C=F:;8E7:97>C:;474F55;84C2A????
<077.292563> V90_V34_Main: reinitializing parameters.
<077.292570> V90ConnectionEvaluator reset called !
<077.292572> $!$ 664A3C5?:A6@4A3C5?:A6@4A397:>::>8F3987?9:D475F9:>::>887G9>?::C473C5?:A6@4A3C5?:A6@4A3C558A????
<077.292578> V90Phase3Demodulator: clearVerificationStatus called
<077.292581> V90_V34_Main: external reset called.
<077.292585> $!$ 9<777<9B==795;6<9:?;9:9?6<98>8:B8<4C55?;;>478<98>8:B8<39:8><;:478=9D:77;5>4B688D4@????
<077.292588> V34SetupModulator: baudrate 2400, carrier 1800, preemp 0, V90=0. fullReset=0
<077.292593> V34SetupDemodulator: baudrate 2400, carrier 1800
<077.292596> V34SetupModulator: baudrate 600, carrier 2400, preemp 0, V90=0. fullReset=1
<077.292602> V34FLO: Echo running in Original Integer...
<077.292604> V34FLO: Echo running in Original Integer...
<077.292608> V34HSHAKE: txstate NOSTATE0=>SILENCEINFO(rx NOSTATE0, mst NOSTATE0, [1]0, [2]0)
<077.292611> V34HSHAKE: rxstate NOSTATE0=>RX_DPSK(tx SILENCEINFO, mst NOSTATE0, [1]0, [2]0)
<077.292614> V34HSHAKE: microstate NOSTATE0=>DET_SYNC(tx SILENCEINFO, rx RX_DPSK, [1]0, [2]0)
<077.292617> V90, setINFO0dBits
<077.292620> V90ConstellationDesigner: set min rate to 300
<077.292622> V90ConstellationDesigner: set max rate to 56000
<077.292625> VPcmV34Main: minLevel given is 0 , minSigLevel set to 101
<077.292627> V34 filtdelay set to 83 (params initial delay = 196)
<077.292630> V34FEC, V34dmadelay set to 1404, (ext delay=148)
<077.292633> $!$ 9<5@4;7:>::>8F5<96>E:98<7E9A><;85A399:>::>8F5=9:>C:79@39::?7::888=9:>;669;7H6?:7785<4;3?????
<077.292635> VPcmFlo: From Stream - Entrance Filter forced disabled...
<077.292638> VPcmFlo: YES! entrance filter applied = 0
<077.292641> v8: delete...
<077.351796> V34HSHAKE: txstate SILENCEINFO=>TX_DPSK(rx RX_DPSK, mst DET_SYNC, [1]10, [2]0)
<077.351812> setINFO0aBits - setting info0a for V.PCM
<077.432266> V34HSHAKE: txstate TX_DPSK=>TONE_AB(rx RX_DPSK, mst DET_SYNC, [1]0, [2]0)
<077.831652> VPcmV34Main: Masking CAS detection after 4800 in train...
<078.851940> V34HSHAKE: txstate TONE_AB=>TX_DPSK(rx RX_DPSK, mst DET_SYNC, [1]0, [2]0)
<078.851961> errorrecovery is initialized in DET_SYNC
10:11:29.008           pjsua_core.c  .RX 437 bytes Request msg OPTIONS/cseq=17052 (rdata0x7fb57c0017d8) from UDP 192.168.88.122:5060:
OPTIONS sip:6000@192.168.88.123:33946;ob SIP/2.0
Via: SIP/2.0/UDP 192.168.88.122:5060;rport;branch=z9hG4bKPjef8ee01a-8808-4f82-9411-6d427893db96
From: <sip:6000@192.168.88.122>;tag=200bb936-b645-4706-9f24-423ae52dae79
To: <sip:6000@192.168.88.123;ob>
Contact: <sip:6000@192.168.88.122:5060>
Call-ID: 595a53bd-f286-4e3d-8aa4-18816b915554
CSeq: 17052 OPTIONS
Max-Forwards: 70
User-Agent: Asterisk PBX 22.5.1
Content-Length:  0


--end msg--
10:11:29.008           pjsua_core.c  .TX 813 bytes Response msg 200/OPTIONS/cseq=17052 (tdta0x7fb57c025998) to UDP 192.168.88.122:5060:
SIP/2.0 200 OK
Via: SIP/2.0/UDP 192.168.88.122:5060;rport=5060;received=192.168.88.122;branch=z9hG4bKPjef8ee01a-8808-4f82-9411-6d427893db96
Call-ID: 595a53bd-f286-4e3d-8aa4-18816b915554
From: <sip:6000@192.168.88.122>;tag=200bb936-b645-4706-9f24-423ae52dae79
To: <sip:6000@192.168.88.123;ob>;tag=z9hG4bKPjef8ee01a-8808-4f82-9411-6d427893db96
CSeq: 17052 OPTIONS
Allow: PRACK, INVITE, ACK, BYE, CANCEL, UPDATE, INFO, SUBSCRIBE, NOTIFY, REFER, MESSAGE, OPTIONS
Accept: application/sdp, application/pidf+xml, application/xpidf+xml, application/dialog-info+xml, application/simple-message-summary, message/sipfrag;version=2.0, application/im-iscomposing+xml, text/plain
Supported: replaces, 100rel, timer, norefersub, trickle-ice
Allow-Events: presence, dialog, message-summary, refer
Content-Length:  0


--end msg--
<092.351851> vpcm: train timeout!
<092.351867> vpcm: Link Error
<092.351870> slamr0: modem_update_status: 4
<092.351872> slamr0: --> FINISH.
<092.351874> slamr0: modem_hup...2
<092.351876> slamr0: modem set state: 2 --> 9...
<092.351878> slamr0: new state: DP_DISC
<092.351907> vpcm: train timeout!
<092.351936> vpcm: train timeout!
<092.351965> vpcm: train timeout!
<092.351967> slamr0: modem_stop..
<092.351969> main: hangup...
<092.351970> main: socket:sipinfo:hookstate: 0 
<092.351973> main: return data to child process...
<092.351975> slamr0: modem set hook: 1 --> 0...
<092.351977> main: socket_ioctl: cmd 2, arg 0...
<092.351979> main: socket_ioctl: returning 0
<092.351981> main: socket_ioctl: cmd 8, arg 0...
<092.351985> main: adjust volume frame needed<092.351986> main: socket_ioctl: returning 0
<092.351990> vpcm: delete...
<092.351997> $!$ 9<5@4979><:C8F7=::>C:79;7H:7:7:E8E39:8><;99:7B9D>E9:8<8;9B>@:D888=9>>F:D5A3989>@:C8@7G9<:7:E8=7?:8><;:475G7D=;669:7::;><::478=9D:7;88<7@9>?:;:998B55:?:?9:5=96?;:77:8=96?;:;474F55;76B477B:8<<:?884?55;D66573B3B8A????
<092.352000> V90Phase3Demodulator: clearVerificationStatus called
<092.352004> V92EchoCanceller Destruction
<092.352007> V92Modem Destruction
<092.352017> V90Modem Destruction
<092.352022> $!$ 9<5@4979><:C8F7=::>C:79;7H:7:7:E8E39:8><;99:7B9D>E9:8<8;9B>@:D888=9>>F:D5A3989>@:C8@7G9<:7:E8=7?:8><;:475G7D=;669:7::;><::478=9D:7;88<7@9>?:;:998B55:?:?9:5=96?;:77:8=96?;:;474F55;76B477B:8<<:?884?55;D66573B3B8A????
<092.352024> V90Phase3Demodulator: clearVerificationStatus called
<092.352035> slamr0: modem set state: 9 --> 1...
<092.352038> slamr0: new state: MODEM_IDLE
<092.352040> slamr0: modem report result: 3 (NO CARRIER)
