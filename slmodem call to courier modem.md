on_call_state: callback
10:34:38.316              d-modem.c  ...Call 0 state=CONFIRMED
10:34:38.334           conference.c !.Added port 1 (sip:6311@192.168.88.122), port count=2
10:34:38.334           conference.c  .Added port 2 (dmodem), port count=3
10:34:38.334           conference.c  .Port 1 (sip:6311@192.168.88.122) transmitting to port 2 (dmodem)
10:34:38.334           conference.c  .Port 2 (dmodem) transmitting to port 1 (sip:6311@192.168.88.122)
10:34:38.334           conference.c  .Added port 3 (scomb-rev), port count=4
10:34:38.334           conference.c  .Port 1 (sip:6311@192.168.88.122) transmitting to port 3 (scomb-rev)
10:34:38.334           conference.c  .Added port 4 (scomb-rev), port count=5
10:34:38.334           conference.c  .Port 2 (dmodem) transmitting to port 4 (scomb-rev)
10:34:38.334     strm0x78f66c02b328  Resetting jitter buffer in stream playback start
dmodem_get_frame: volume frame recieved
<479.897272> DCR: initial DC Evaluation done, DC level 0, enabled
<480.296813> slamr0: 19200: change dp: --> 8...
<480.296828> v8: create: caller 0, automode 0, dp id 92.
<480.296843> V8: Create called, V8 version 23/09/03 .
<480.296846> ############################################################
<480.296848> V8: local configuration : 
<480.296850> 	Side = Answer
<480.296852> 	Operation Mode = 0
<480.296854> 	Modulations - V90=0, V34=1, V34HD=0, V32=1, V22=0, V17=0, V29=0, V27=0, V23=0, V21=0
<480.296857> 	Call Functions - Data=1, CallRxFax=0, CallTxFax=0, V.80=0
<480.296860> 	Protocol - LAPM V.42
<480.296862> 	v8bisIndication - 0
<480.296864> 	timeouts - signal detect 12 sec, message detect 7 sec
<480.296866> 	quickConnectEnabled - 0
<480.296868> 	lapmIndication - 1
<480.296870> 	ucodeForQts - 9
<480.296872> 	ansPcmLevel - 0
<480.296875> ############################################################
<480.296879> V8: Initial JM message length is 8 octets
<480.317090> V8: State changed from V8_INIT to V8_ANS_SEND_ANSAM
<480.317104> v8: status (1) V8_ANS_SEND_ANSAM
<482.237548> V8: on ANSWER: remote call function is: 107
<482.237565> V8: call function DATA indication...
<482.237568> V8: on ANSWER: remote V90: mod - 1, digital connection - 0, pcmIndication - 2 , local - 0
<482.237570> V8: Final JM message length is 8 octets
<482.237573> V8: State changed from V8_ANS_SEND_ANSAM to V8_ANS_SEND_JM
<482.237576> v8: status (3) V8_ANS_SEND_JM
<483.297149> V8: State changed from V8_ANS_SEND_JM to V8_OK
<483.297165> v8: process: OK.
<483.297167> V8Report: remote V90: mod - 1, digital connection - 0, pcmIndication - 2
<483.297169> V8Report: v90:0, v34:1, v34hd:0, V32:1, V22:0, V17:0, V29:0, V27:0, V23:0, V21:0
<483.297172> main: socket_ioctl: cmd 10, arg 0...
<483.297175> main: socket_ioctl: returning c0
<483.297176> v8: Link established. Idle timer 864.
<483.297178> v8: status (13) V8_OK
<483.297180> slamr0: 47808: change dp: --> 34...
<483.297183> vpcm: create: dp 34, caller 0, frag 48 (size 53848).
<483.297230> VPCMXF_Create: side is Analog, maxDataBuffer - 48
<483.297301> V90Modem Construction (as Analog Modem)
<483.297314> $!$ 6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A1F3?????
<483.297317> $!$ 6@4A3C5?:A6@4A3955:766473955:7667;7A9>?:668@8<55>866776;7>==877;5>55?=:;998<9>>F:D473955:766473955:76@4A3C5?:A6@4A1F3?????
<483.297319> $!$ 6@4A3C5?:A6@4A3955:766477?9D?9669;7A9::7;;9:7>55>F:<476<7A:78:7:6955>>;88F8>:5:7:E8E7E:>:766473955:76@4A3C5?:A6@4A1F3?????
<483.297321> *********************************************************
<483.297322> V90Modem Version: 2.98  (25-Mar-04)
<483.297324> *********************************************************
<483.297339> $!$ 9<5@497B>F::8<7F55==:;998<9>>F:D475=9:?::9997B:5?;:?8F7G6?8D4@????
<483.297341> $!$ 8C8F7=9>>=:?8<7=55=8;;8@7<9@:7898F7G9C><:99;39:<>@;:8?7H::?;666D7>9B>F;89@395@:79:997:9>>E667;7B9B><664B3978>F:D9:8=9:>C66777H:<><;82D1C????
<483.297344> $!$ 898F7F:5>F:D8<7G:9?:7@475?9A>F;88<7::9:C66685=7>:C66685<79:C666E7>:<:7886C5E3B8A????
<483.297346> *********************************************************
<483.297349> V90Parameters: upStream min rate : 4800 upStream max rate : 4800  Rate mask :1
<483.297354> $!$ 9<5@4985>8;8887F9:?;:;998<6?:78:8<7;::>>664D39:9><:C976987:77C47493B8A????
<483.297357> $!$ 9<5@4985>8;8887F9:?;:;998<6?:78:8<7;::>>664D39:9><:C9769:7>F:88<396B:7762D1C????
<483.297360> $!$ 9<5@4985>8;8887F9:?;:;998<6?:78:8<7;::>>664D39:9><:C975<9D>E:D8<7<:9>@:E8E6=:>?7:;474F55;74C2A????
<483.297363> $!$ 9<5@4985>8;8887F9:?;:;998<6?:78:8<7;::>>664D39:9?9:D597=8D>D:;887G8D><;8997H:7=F;99;7=8D><;<887E::>8;:8@7H9C=F:;8E7:97>C:;474F55;84C2A????
<483.297368> V90PreFilter: HardwareCodecType: Panther_AD1803
<483.297671> $!$ 9<5@4987><;9887F:5>C:;994C55?:;:888=9::77C475?87<F9@6<5G3B8A????
<483.297739> V90ConnectionEvaluator reset called !
<483.297742> $!$ 664A3C5?:A6@4A3C5?:A6@4A397:>::>8F3987?9:D475F9:>::>887G9>?::C473C5?:A6@4A3C5?:A6@4A3C558A????
<483.297748> V90Phase3Demodulator: Reset called, sessionFlag = 1 !
<483.297753> V90Phase3Demodulator: initial state set to WaitForSd
<483.297761> $!$ 9<5@497:?8;;887E9>?A:;994C55<:;88<7::9><::473F55<D8C7?399B>F::8<399>?:668;7B:8>8:88C7>99:E4C2A????
<483.297764> $!$ 9<5@497:?8;;887E9>?A:;994C55?9:;9:7>:98D4@????
<483.297768> $!$ 9<5@497:?8;;887E9>?A:;994C55<C8;475;9:?;:7474F55:D764E4965;776577>5B;8762D1C????
<483.297771> $!$ 9<5@497:?8;;887E9>?A:;994C55<;8<6<3977><;:88396B:76C573G65;77657499::D7=2D1C????
<483.297793> V92Modem Construction (as Analog Modem)
<483.297797> $!$ 6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A1F3?????
<483.297800> $!$ 6@4A3C5?:A6@4A3955:766473955:7667;7A9>?:668@8<55>866776;7>==877;5>55?=:;998<9>>F:D473955:766473955:76@4A3C5?:A6@4A1F3?????
<483.297803> $!$ 6@4A3C5?:A6@4A3955:766477?9D?9669;7A9::7;;9:7>55>F:<476<7A:78:7:6955>>;88F8>:5:7:E8E7E:>:766473955:76@4A3C5?:A6@4A1F3?????
<483.297806> *********************************************************
<483.297808> V92Modem Version: 1.1  (9-Apr-01)
<483.297811> *********************************************************
<483.297813> $!$ 9<5@4;7B>F::8<7F55==:;998<9>>F:D475=9:?::9997B:5?;:?8F7G6?8D4@????
<483.297816> $!$ 8C8<7F9D?9;?477<9A><:78E8>:5?:664B3999>@:B477=9:?::9997B:5?;:E993998?9:79:7A55>=:?9?1F3?????
<483.297819> $!$ 6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A3C5?:A6@4A1F3?????
<483.297827> V92Modulator constraction
<483.298224> $!$ 9<5@4;85>?:79:7>68<D:E8;8>9A>8;:8F8;6?:79:795G66?<669:8=96?;:;477E9:>E:=9;7A55?::;9;39:9>F66594965;77:2D1C????
<483.298229> V92Modulator reset
<483.298233> V92EchoCanceller: constraction
<483.298237> $!$ 9<5@4;7:>::>8F5<96>E:98<7E9A><;85A399:>::>8F5=9:>C:79@39::?7::888=9:>;669;7H6?:77>5;493?????
<483.298241> $!$ 9<5@4;7:>::>8F5<96>E:98<7E9A><;85A399:>::>8F5?9>>C;:8<8;7A><:D474F55;87>571F3?????
<483.298251> $!$ 9<5@4;7:>::>8F5<96>E:98<7E9A><;85A399:>::>8F5;9:?;:7474F55:D764E4965;77657493B8A????
<483.298253> $!$ 9<5@4;7:>::>8F5<96>E:98<7E9A><;85A399:>::>8F5;9:?;:76;7>98>8;?474F55:D764E4965;77657493B8A????
<483.298263> vpcm: VPCM rate limits: 300-56000
<483.298264> main: socket_ioctl: cmd 10, arg 0...
<483.298269> main: socket_ioctl: returning c0
<483.298272> vpcm: Delays: HW 196, DMA 148
<483.298280> vpcm: initial dp V.34, session type 0.
<483.298282> VPcmV34Create: quick connect indication from phase1 = 0
<483.298285> VPcmV34Create: Uqts index is 9
<483.298287> VPcmV34Create: ANSpcm level index is 0
<483.298289> VPcmV34Create, initial Session Type = 0
<483.298291> RX at 5EB34460
<483.298296> On Create: Setting desired TX MD (0 mSec)!
<483.298300> $!$ 9<777<9B==795;6<9:?;9:9?6<98>8:B8<4C55?;;>478<98>8:B8<39:8><;:478=9D:77;5>4B688D4@????
<483.298303> V34SetupModulator: baudrate 2400, carrier 1800, preemp 0, V90=0. fullReset=0
<483.298307> V34SetupDemodulator: baudrate 2400, carrier 1800
<483.298309> V34SetupModulator: baudrate 600, carrier 2400, preemp 0, V90=0. fullReset=1
<483.298314> V34FLO: Echo running in Original Integer...
<483.298316> V34FLO: Echo running in Original Integer...
<483.298319> V34HSHAKE: txstate NOSTATE0=>SILENCEINFO(rx NOSTATE0, mst NOSTATE0, [1]0, [2]0)
<483.298323> V34HSHAKE: rxstate NOSTATE0=>RX_DPSK(tx SILENCEINFO, mst NOSTATE0, [1]0, [2]0)
<483.298325> V34HSHAKE: microstate NOSTATE0=>DET_SYNC(tx SILENCEINFO, rx RX_DPSK, [1]0, [2]0)
<483.298328> VPcmV34Main: minLevel given is 0 , minSigLevel set to 101
<483.298331> V34 filtdelay set to 83 (params initial delay = 196)
<483.298333> V34FEC, V34dmadelay set to 1404, (ext delay=148)
<483.298336> $!$ 9<5@4;7:>::>8F5<96>E:98<7E9A><;85A399:>::>8F5=9:>C:79@39::?7::888=9:>;669;7H6?:7785<4;3?????
<483.298339> VPcmFlo: From Stream - Entrance Filter forced disabled...
<483.298341> VPcmFlo: YES! entrance filter applied = 0
<483.298344> v8: delete...
<483.357469> V34HSHAKE: txstate SILENCEINFO=>TX_DPSK(rx RX_DPSK, mst DET_SYNC, [1]10, [2]0)
<483.357484> setINFO0aBits - setting info0a for V.34
<483.436943> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx TX_DPSK, rx RX_DPSK, [1]0, [2]0)
<483.436958> DET_SYNC is received in RX_DPSK,rx->gain=0x600
<483.457046> V34HSHAKE: txstate TX_DPSK=>TONE_AB(rx RX_DPSK, mst DET_INFO, [1]0, [2]12)
<483.477215> V34INFO, rxinfo0 0xff,0x84,0x47,0xec,0x0,0x0,0x0,0x0,0x0,0x0
<483.477226> V34HSHAKE: microstate DET_INFO=>TX_PHASE1_ANS(tx TONE_AB, rx RX_DPSK, [1]0, [2]0)
<483.477229> V34INFO, info0 received in DET_INFO
<483.657166> V34HSHAKE: microstate TX_PHASE1_ANS=>RX_PHASE1_ANS(tx TONE_AB, rx RX_DPSK, [1]0, [2]0)
<483.857218> VPcmV34Main: Masking CAS detection after 4800 in train...
<483.877362> On RX_PHASE1_ANS: is short=0, bulkDelay=1480, filtDelay=83
<483.877377> V34 bulk delay estimation 1480 (FAR=1)
<483.877379> V34HSHAKE: microstate RX_PHASE1_ANS=>TX_PHASE2_ANS(tx TONE_AB, rx RX_DPSK, [1]0, [2]83)
10:34:43.877           pjsua_core.c  .RX 437 bytes Request msg OPTIONS/cseq=13996 (rdata0x78f66c001bd8) from UDP 192.168.88.122:5060:
OPTIONS sip:6000@192.168.88.123:44702;ob SIP/2.0
Via: SIP/2.0/UDP 192.168.88.122:5060;rport;branch=z9hG4bKPjb8162cf9-f57d-4b5c-af75-039871e784d0
From: <sip:6000@192.168.88.122>;tag=f9ef5f77-4023-4bf4-90e4-7aeb93c2ad1d
To: <sip:6000@192.168.88.123;ob>
Contact: <sip:6000@192.168.88.122:5060>
Call-ID: 847f9093-d364-4b4d-975f-d8c6bcbf40da
CSeq: 13996 OPTIONS
Max-Forwards: 70
User-Agent: Asterisk PBX 22.5.1
Content-Length:  0


--end msg--
10:34:43.877           pjsua_core.c  .TX 813 bytes Response msg 200/OPTIONS/cseq=13996 (tdta0x78f66c00a698) to UDP 192.168.88.122:5060:
SIP/2.0 200 OK
Via: SIP/2.0/UDP 192.168.88.122:5060;rport=5060;received=192.168.88.122;branch=z9hG4bKPjb8162cf9-f57d-4b5c-af75-039871e784d0
Call-ID: 847f9093-d364-4b4d-975f-d8c6bcbf40da
From: <sip:6000@192.168.88.122>;tag=f9ef5f77-4023-4bf4-90e4-7aeb93c2ad1d
To: <sip:6000@192.168.88.123;ob>;tag=z9hG4bKPjb8162cf9-f57d-4b5c-af75-039871e784d0
CSeq: 13996 OPTIONS
Allow: PRACK, INVITE, ACK, BYE, CANCEL, UPDATE, INFO, SUBSCRIBE, NOTIFY, REFER, MESSAGE, OPTIONS
Accept: application/sdp, application/pidf+xml, application/xpidf+xml, application/dialog-info+xml, application/simple-message-summary, message/sipfrag;version=2.0, application/im-iscomposing+xml, text/plain
Supported: replaces, 100rel, timer, norefersub, trickle-ice
Allow-Events: presence, dialog, message-summary, refer
Content-Length:  0


--end msg--
<483.897391> V34HSHAKE: microstate TX_PHASE2_ANS=>TX_L1(tx TONE_AB, rx RX_DPSK, [1]0, [2]0)
<483.897490> V34HSHAKE: txstate TONE_AB=>TX_L1(rx RX_DPSK, mst TX_L1, [1]0, [2]42)
<483.897493> V34HSHAKE: rxstate RX_DPSK=>DET_AB(tx TX_L1, mst TX_L1, [1]0, [2]42)
<483.897495> V34RETRAIN, starting DET_AB, rx->rxflgs = 0x1a04,rx->gain=0x2fe
<484.057284> V34HSHAKE: microstate TX_L1=>TX_L2(tx TX_L1, rx DET_AB, [1]1536, [2]42)
<484.437259> V34RETRAIN, End of L2,rx->gain=0x2fe,count1=3636
<484.437274> V34HSHAKE: txstate TX_L1=>TONE_AB(rx DET_AB, mst TX_L2, [1]3636, [2]42)
<484.437277> V34HSHAKE: microstate TX_L2=>TX_PHASE3_ANS(tx TONE_AB, rx DET_AB, [1]3636, [2]42)
<484.437280> V34HSHAKE: rxstate DET_AB=>RX_DPSK(tx TONE_AB, mst TX_PHASE3_ANS, [1]3636, [2]42)
<484.516702> V34HSHAKE: txstate TONE_AB=>SILENCE(rx RX_DPSK, mst TX_PHASE3_ANS, [1]0, [2]162)
<484.516718> V34HSHAKE: microstate TX_PHASE3_ANS=>RX_PHASE2_ANS(tx SILENCE, rx RX_DPSK, [1]0, [2]162)
<484.716824> V34SetupDemodulator: baudrate 2400, carrier 1800
<484.716838> V34HSHAKE: rxstate RX_DPSK=>RX_L1(tx SILENCE, mst RX_PHASE2_ANS, [1]0, [2]20)
<484.736934> V34AGC, rx->gain = 0x151, at the beginning of RX_L1
<485.177174> V34SetupDemodulator: baudrate 2400, carrier 0
<485.177191> V34HSHAKE: rxstate RX_L1=>RX_DPSK(tx SILENCE, mst RX_PHASE2_ANS, [1]0, [2]1089)
<485.177195> V34HSHAKE: txstate SILENCE=>TONE_AB(rx RX_DPSK, mst RX_PHASE2_ANS, [1]0, [2]1089)
<485.177197> V34HSHAKE: microstate RX_PHASE2_ANS=>DET_SYNC(tx TONE_AB, rx RX_DPSK, [1]0, [2]1089)
<485.397121> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx TONE_AB, rx RX_DPSK, [1]0, [2]0)
<485.397141> DET_SYNC is received in RX_DPSK,rx->gain=0x2fe
<485.614190> V34HSHAKE: txstate TONE_AB=>TX_DPSK(rx RX_DPSK, mst DET_INFO, [1]0, [2]0)
<485.614209> RX MDLENGTH = 0
<485.614212> V34PROBE, snr_L1=25030 , snr_L2=14613 , L2toL1ratio=598  (all not in dB)
<485.614214> V34PROBE, dBcnt=0 , powerReductionReq=0 , gain=766
<485.614215> Sensitive RX Power Reduction mechanism disabled!
<485.614217> V34PROBE, not asking for power reduction
<485.614218> V34PROBE, rx->gain=766 ,(obj->rxinfo0.data[1]&0x80)=128
<485.614219> V34PROBE,0=5,0,0,0,0,5=12,0,12,0,0,10=0,12,0,0,0,12,0,0,0,0,20=0,0,22=1,2,24=4
<485.614223> VpcmV34Main: max V34 baud rate index = 0
<485.614224> V34PREEMPHASIS, - index is 6, baudrate= 3429
<485.614226> V34PROBESELECT, in ANSWER, txbaudrate = 3429,rxbaudrate = 3429,
<485.614227> V34PROBE, rxinfo1c 0x0,0x0,0x8,0x14,0x6,0x7,0x0,0x61,0x20,0x6
<485.614243> V34HSHAKE: microstate DET_INFO=>INFODONE(tx TX_DPSK, rx RX_DPSK, [1]0, [2]0)
<485.614245> V34PROBE, txinfo1a 0x0,0x1,0xa6,0xd0,0x0,0x0,0x0,0x0,0x0,0x0
<485.678988> V34HSHAKE: txstate TX_DPSK=>TONE_AB(rx RX_DPSK, mst INFODONE, [1]0, [2]0)
<485.679033> V34HSHAKE: txstate TONE_AB=>SILENCE(rx RX_DPSK, mst INFODONE, [1]0, [2]16)
<485.679049> $!$ 9<777<9B==795;5F96>@:D5A39:5>?:79:7>68>?:78C7?79?<;68C7>:=<C:;8E7@:9>?665D396:;?7:5839:8?@:C897H9A?:664?7;96?<::474<69;97?4@1F3?????
<485.679051> VPcmV34Main: Wait (before P2 COMPLETE)...
<485.697063> VPcmV34Main: Indicating First P2 COMPLETE... (after 96)
<485.697075> vpcm: New status 1
<485.697077> vpcm: Phase II completed !!!
<485.697098> On PHASE2_COMPLETE: added Silence = 0, p2DelayCntr = 144
<485.697099> VPcmV34Main: Wait (after P2 COMPLETE)...
<485.697119> VPcmV34Main: Wait (after P2 COMPLETE)...
<485.697139> VPcmV34Main: Wait (after P2 COMPLETE)...
<485.717362> VPcmV34Main: Wait (after P2 COMPLETE)...
<485.717400> VPcmV34Main: Wait (after P2 COMPLETE)...
<485.717420> VPcmV34Main: Wait (after P2 COMPLETE)...
<485.717441> VPcmV34Main: Wait (after P2 COMPLETE)...
<485.737316> VPcmV34Main: Wait (after P2 COMPLETE)...
<485.737351> VPcmV34Main: Wait (after P2 COMPLETE)...
<485.737372> VPcmV34Main: Wait (after P2 COMPLETE)...
<485.757432> V34Hshak: Setting up transmitter for phase3...
<485.757452> $!$ 9<777<9B==795;5F96>@:D5A3979?<:;478=9D:7:<8@7G96>C666D7B9C=;9>776;55;D66573E55?::;9;8=9>>E:=477>98>?:E5A3999><:9888B55?:;:888;:9:77C474;65;7764C3999><:9888B55>=:78:8=55;D665:4;6<;87;4C3997><;:88396B:77<2D1C????
<485.757457> $!$ 9<777<9B==795;5F96>@:D5A397<><;:475F9>>E:?8D7:9A:7;68F8@9:?966997>99?<:99;7B9D>E664D39:7><;:9<8;9C>@:D8>3965:76>8:7?9<:7:<8C7:9<:7;98<8=55?;:E474:5>8D4@????
<485.757459> V34TXSCALE, power reduction requested by remote modem is 0 dB
<485.757460> V34TXSCALE, txscale before is 5793, reduced txscale is 0 dB,final txscale is 5793
<485.757463> V34SetupModulator: baudrate 3429, carrier 1959, preemp 0, V90=0. fullReset=1
<485.757467> V34HSHAKE: rxstate RX_DPSK=>WAIT(tx SILENCE, mst INFODONE, [1]0, [2]30)
<485.757469> V34HSHAKE: txstate SILENCE=>SSEG(rx WAIT, mst INFODONE, [1]0, [2]30)
<485.757471> V34FLO: Echo running in Original Integer...
<485.757473> V34FLO: Echo running in Original Integer...
<485.757512> vpcm: New status 2
<485.777696> V34HSHAKE: txstate SSEG=>SBARSEG(rx WAIT, mst INFODONE, [1]0, [2]30)
<485.797704> V34HSHAKE: txstate SBARSEG=>PPSEG(rx WAIT, mst INFODONE, [1]0, [2]30)
<485.882297> V34HSHAKE: txstate PPSEG=>TRNSEG4(rx WAIT, mst INFODONE, [1]1624, [2]30)
<485.882359> V34Hshak: echo start wait time would be: NEC 409 symbols, FEC 1277 symbols...
<485.917537> VPcmV34Main: Echo adapt start reported...
<485.917832> updateAlphaNE: updated 0 => -4063
<485.977794> V34FEC - start FEC adaptation
<485.996909> updateAlphaFE: updated 0 => -1697
<486.637255> VPcmV34Main: Echo adapt middle reported...
<487.521246> V34HSHAKE: txstate TRNSEG4=>JTXMIT(rx WAIT, mst INFODONE, [1]1624, [2]409)
<487.521263> V34Hshak: On J TX start, would freeze EC after bulk delay (740 samples, bulk=1480)
<487.521266> V34HSHAKE: rxstate WAIT=>RECEIVE(tx JTXMIT, mst INFODONE, [1]1624, [2]740)
<487.521269> V34SetupDemodulator: baudrate 3429, carrier 1959
<487.521272> V34AGC, setup receiver gain = 0x2a2
<487.521274> V34HSHAKE: microstate INFODONE=>DET_SYNC(tx JTXMIT, rx RECEIVE, [1]1624, [2]740)
<487.717679> V34Hshak: on JTXMIT - time to freeze echo...
<487.717695> V34HSHAK: Freeze EC
<487.717697> ==== Near Echo Canceller report ======
<487.717699> ?======= Coefficients[1..144]=========
<487.717701> ?58 -184 -250 -52 35 13
<487.717703> ?-245 -332 -39 9 1 -105
<487.717704> ?-298 -41 155 84 -93 -267
<487.717706> ?64 274 16 -110 -133 99
<487.717708> ?275 -1 -111 -68 64 270
<487.717709> ?79 -115 -101 18 290 127
<487.717712> ?-164 -44 64 176 88 -201
<487.717713> ?-72 71 61 86 -109 -103
<487.717715> ?60 8 27 -56 -120 30
<487.717716> ?-20 -47 -6 -69 15 -11
<487.717718> ?-94 -11 -13 51 30 -162
<487.717719> ?-46 77 7 -15 -73 0
<487.717720> ?98 31 10 -40 29 153
<487.717722> ?23 -39 3 103 203 -31
<487.717723> ?-109 115 175 121 -77 -110
<487.717725> ?112 85 25 -36 -118 63
<487.717726> ?58 -103 -129 -158 13 75
<487.717728> ?-130 -153 -112 39 138 -115
<487.717729> ?-185 -33 60 100 -78 -155
<487.717730> ?34 137 99 -106 -204 69
<487.717732> ?242 78 -127 -112 118 214
<487.717733> ?25 -145 -119 39 169 30
<487.717735> ?-184 -122 77 215 68 -207
<487.717736> ?-70 184 219 148 -94 -92
<487.717737> ==== Far Echo Canceller report ======
<487.717739> ?======= Coefficients[1..144]=========
<487.717740> ?53 26 -20 -10 -26 3
<487.717741> ?21 -41 -57 -3 8 -4
<487.717743> ?-9 -36 -1 44 3 -12
<487.717745> ?-9 -10 27 17 -14 -5
<487.717746> ?0 3 22 21 -16 7
<487.717747> ?32 -26 -12 10 -36 -2
<487.717749> ?-2 -31 10 -14 -40 -44
<487.717750> ?-29 35 -21 -75 -54 -58
<487.717752> ?34 19 -84 -26 14 67
<487.717753> ?58 -42 30 47 78 148
<487.717755> ?-34 -18 128 74 117 29
<487.717756> ?-82 97 80 -59 -2 -75
<487.717758> ?-170 59 12 -359 20 493
<487.717760> ?59 -252 -207 -207 -24 -27
<487.717761> ?-112 -59 -106 21 86 -46
<487.717763> ?-24 -27 63 150 -14 -37
<487.717764> ?33 117 171 -11 -29 69
<487.717766> ?74 143 23 -80 24 55
<487.717768> ?93 -5 -133 -12 37 16
<487.717769> ?-45 -163 -62 16 -35 -45
<487.717771> ?-108 -78 20 12 -21 -47
<487.717772> ?-1 63 49 14 -32 17
<487.717774> ?93 46 2 -21 11 92
<487.717775> ?54 -10 -26 -12 62 48
<487.717961> S detected,rxflgs= 0x1304,rxgain=0x2a2
<487.736817> V34AGC, abcddetect gain = 0x3b7, AGC frozen
<487.756911> S-S1 is detected,rxsymcnt= 124,pllcnt= 2,gain= 0x3b7
<487.756929> V34HSHAKE: txstate JTXMIT=>XMIT0(rx RECEIVE, mst DET_SYNC, [1]7, [2]0)
<487.837668> TimingV34: Timing Offset [ppm] = 248
<487.957840> TimingV34: Timing Offset [ppm] = 44
<488.016923> V34EQU, equerr = 31239, preerr = 25113,
<488.077367> TimingV34: Timing Offset [ppm] = -13
<488.217015> TimingV34: Timing Offset [ppm] = 28
<488.369600> V34EQU, equerr = 197, preerr = 178,
<488.372031> TimingV34: Timing Offset [ppm] = -19
<488.457548> TimingV34: Timing Offset [ppm] = -18
<488.577632> TimingV34: Timing Offset [ppm] = -10
<488.617791> V34EQU, equerr = 100, preerr = 105,
<488.717331> TimingV34: Timing Offset [ppm] = 2
<488.737523> V34HSHAKE: txstate XMIT0=>SSEG(rx RECEIVE, mst DET_SYNC, [1]0, [2]0)
<488.737561> vpcm: New status 3
<488.777761> V34HSHAKE: txstate SSEG=>SBARSEG(rx RECEIVE, mst DET_SYNC, [1]0, [2]0)
<488.777841> V34HSHAKE: txstate SBARSEG=>TRNSEG4A(rx RECEIVE, mst DET_SYNC, [1]0, [2]0)
<488.842590> TimingV34: Timing Offset [ppm] = -8
<488.916860> V34EQU, equerr = 1124, preerr = 952,
<488.957076> TimingV34: Timing Offset [ppm] = -7
<489.076997> TimingV34: Timing Offset [ppm] = -12
<489.217673> TimingV34: Timing Offset [ppm] = -5
<489.217762> V34EQU, equerr = 2342, preerr = 1159,
<489.337818> TimingV34: Timing Offset [ppm] = -13
<489.458196> TimingV34: Timing Offset [ppm] = -9
<489.518130> V34EQU, equerr = 1790, preerr = 873,
<489.577513> TimingV34: Timing Offset [ppm] = -8
<489.717573> TimingV34: Timing Offset [ppm] = -3
<489.817162> V34EQU, equerr = 1469, preerr = 686,
<489.837289> TimingV34: Timing Offset [ppm] = -10
<489.957126> TimingV34: Timing Offset [ppm] = -2
<490.077253> TimingV34: Timing Offset [ppm] = -12
<490.118052> V34EQU, equerr = 1705, preerr = 950,
<490.217654> TimingV34: Timing Offset [ppm] = 9
<490.337550> TimingV34: Timing Offset [ppm] = -7
<490.507493> V34EQU, equerr = 1345, preerr = 682,
<490.512492> TimingV34: Timing Offset [ppm] = -3
<490.515265> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx TRNSEG4A, rx RECEIVE, [1]0, [2]0)
<490.517993> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<490.518003> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx TRNSEG4A, rx RECEIVE, [1]0, [2]7)
<490.518165> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx TRNSEG4A, rx RECEIVE, [1]0, [2]7)
<490.537147> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<490.537165> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx TRNSEG4A, rx RECEIVE, [1]0, [2]10)
<490.537200> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx TRNSEG4A, rx RECEIVE, [1]0, [2]10)
<490.557222> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<490.557240> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx TRNSEG4A, rx RECEIVE, [1]0, [2]10)
<490.557277> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx TRNSEG4A, rx RECEIVE, [1]0, [2]10)
<490.577434> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<490.577451> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx TRNSEG4A, rx RECEIVE, [1]0, [2]10)
<490.577458> TimingV34: Timing Offset [ppm] = -4
<490.577488> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx TRNSEG4A, rx RECEIVE, [1]0, [2]10)
<490.617584> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<490.617601> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx TRNSEG4A, rx RECEIVE, [1]0, [2]10)
<490.617636> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx TRNSEG4A, rx RECEIVE, [1]0, [2]10)
<490.637817> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<490.637832> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx TRNSEG4A, rx RECEIVE, [1]0, [2]10)
<490.637865> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx TRNSEG4A, rx RECEIVE, [1]0, [2]10)
<490.656914> V34DATARATE, precoefs=0, 0, 0, 0, 0, 0
<490.656930> V34DATARATE,threshold for data rate 14 = 50
<490.656933> V34DATARATE,threshold for data rate 13 = 84
<490.656935> V34DATARATE,threshold for data rate 12 = 131
<490.656936> V34DATARATE,threshold for data rate 11 = 205
<490.656937> V34DATARATE,threshold for data rate 10 = 366
<490.656939> V34DATARATE,threshold for data rate 9 = 572
<490.656940> V34DATARATE,threshold for data rate 8 = 937
<490.656941> V34DATARATE,threshold for data rate 7 = 1539
<490.656943> V34DATARATE, ethresh data rate = 7,ethreh=1345,rate2 = 0x89b0,data=1539
<490.656945> V34DATARATE, equerr = 1345,preerr=682
<490.657537> V34DATARATE, automatic: 16800, min 0, max 33600
<490.657541> V34DATARATE, Final choice data rate = 7, retrainThresh = 3078, renegDownthresh = 2055, renegUpthresh = 937
<490.657550> $!$ :E8E399<><;:477F96??669<89:8?;;88<7:9B:7;8888=9::C668F7G55?9:;8>8>9A>8;8475B88=76B478;9:?;;;997G9>>E:=474<68;=76571F3?????
<490.657553> V34HSHAKE: txstate TRNSEG4A=>XMITMP(rx RECEIVE, mst DET_INFO, [1]0, [2]5)
<490.657555> V34DATARATE, txmp bits 0xb9c2,0xfffd,0x0,0x0,0x0
<490.657811> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<490.657817> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<490.676889> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<490.676906> V34MP, Starting txmit MP again(1), rxflgs=0x614,txflags=0x2705
<490.677071> V34MP, MP detected, starting MP' txmit<490.677073> V34MP, Starting txmit MP again(0), rxflgs=0x614,txflags=0x2705
<490.697203> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<490.697902> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<490.697945> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<490.697953> V34MP, Starting txmit MP again(1), rxflgs=0x614,txflags=0x2705
<490.717196> TimingV34: Timing Offset [ppm] = -8
<490.717229> V34MP, MP detected, starting MP' txmit<490.717231> V34MP, Starting txmit MP again(2), rxflgs=0x614,txflags=0x2705
<490.717236> V34EQU, equerr = 1527, preerr = 745,
<490.771676> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<490.771696> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<490.771732> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<490.771741> V34MP, MP detected, starting MP' txmit<490.771742> V34MP, Starting txmit MP again(3), rxflgs=0x614,txflags=0x2705
<490.774555> V34MP, MP detected, starting MP' txmit<490.774564> V34MP, Starting txmit MP again(4), rxflgs=0x614,txflags=0x2705
<490.776801> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<490.776810> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<490.776845> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<490.776853> V34MP, MP detected, starting MP' txmit<490.776855> V34MP, Starting txmit MP again(5), rxflgs=0x614,txflags=0x2705
<490.777017> V34MP, MP detected, starting MP' txmit<490.777019> V34MP, Starting txmit MP again(6), rxflgs=0x614,txflags=0x2705
<490.779236> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<490.779243> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<490.779282> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<490.779290> V34MP, MP detected, starting MP' txmit<490.779291> V34MP, Starting txmit MP again(7), rxflgs=0x614,txflags=0x2705
<490.797560> V34MP, MP detected, starting MP' txmit<490.797569> V34MP, Starting txmit MP again(8), rxflgs=0x614,txflags=0x2705
<490.797771> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<490.797776> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<490.797815> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<490.797856> V34MP, MP detected, starting MP' txmit<490.797858> V34MP, Starting txmit MP again(9), rxflgs=0x614,txflags=0x2705
<490.817775> V34MP, MP detected, starting MP' txmit<490.817794> V34MP, Starting txmit MP again(10), rxflgs=0x614,txflags=0x2705
<490.837025> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<490.837042> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<490.837079> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<490.837090> V34MP, MP detected, starting MP' txmit<490.837092> V34MP, Starting txmit MP again(11), rxflgs=0x614,txflags=0x2705
<490.837096> TimingV34: Timing Offset [ppm] = -3
<490.837255> V34MP, MP detected, starting MP' txmit<490.837258> V34MP, Starting txmit MP again(12), rxflgs=0x614,txflags=0x2705
<490.857224> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<490.857240> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<490.857275> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<490.857287> V34MP, MP detected, starting MP' txmit<490.857289> V34MP, Starting txmit MP again(13), rxflgs=0x614,txflags=0x2705
<490.877418> V34MP, MP detected, starting MP' txmit<490.877434> V34MP, Starting txmit MP again(14), rxflgs=0x614,txflags=0x2705
<490.877929> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<490.877938> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<490.897376> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<490.897419> V34MP, MP detected, starting MP' txmit<490.897422> V34MP, Starting txmit MP again(15), rxflgs=0x614,txflags=0x2705
<490.897585> V34MP, MP detected, starting MP' txmit<490.897588> V34MP, Starting txmit MP again(16), rxflgs=0x614,txflags=0x2705
<490.917589> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<490.917602> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<490.917637> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<490.917644> V34MP, MP detected, starting MP' txmit<490.917646> V34MP, Starting txmit MP again(17), rxflgs=0x614,txflags=0x2705
<490.937962> V34MP, MP detected, starting MP' txmit<490.937979> V34MP, Starting txmit MP again(18), rxflgs=0x614,txflags=0x2705
<490.939461> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<490.939525> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<490.939562> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<490.939570> V34MP, MP detected, starting MP' txmit<490.939571> V34MP, Starting txmit MP again(19), rxflgs=0x614,txflags=0x2705
<490.964165> V34MP, MP detected, starting MP' txmit<490.964186> V34MP, Starting txmit MP again(20), rxflgs=0x614,txflags=0x2705
<490.964212> TimingV34: Timing Offset [ppm] = -8
<490.964564> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<490.964571> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<490.976848> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<490.976864> V34MP, MP detected, starting MP' txmit<490.976867> V34MP, Starting txmit MP again(21), rxflgs=0x614,txflags=0x2705
<490.977029> V34MP, MP detected, starting MP' txmit<490.977031> V34MP, Starting txmit MP again(22), rxflgs=0x614,txflags=0x2705
<490.997124> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<490.997141> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<490.997178> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<490.997184> V34MP, MP detected, starting MP' txmit<490.997186> V34MP, Starting txmit MP again(23), rxflgs=0x614,txflags=0x2705
<491.017156> V34EQU, equerr = 1460, preerr = 0,
<491.017207> V34MP, MP detected, starting MP' txmit<491.017210> V34MP, Starting txmit MP again(24), rxflgs=0x614,txflags=0x2705
<491.017434> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<491.017440> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<491.017475> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<491.017482> V34MP, MP detected, starting MP' txmit<491.017485> V34MP, Starting txmit MP again(25), rxflgs=0x614,txflags=0x2705
<491.038702> V34MP, MP detected, starting MP' txmit<491.038721> V34MP, Starting txmit MP again(26), rxflgs=0x614,txflags=0x2705
<491.057620> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<491.057637> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<491.057672> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<491.057683> V34MP, MP detected, starting MP' txmit<491.057686> V34MP, Starting txmit MP again(27), rxflgs=0x614,txflags=0x2705
<491.057963> V34MP, MP detected, starting MP' txmit<491.057967> V34MP, Starting txmit MP again(28), rxflgs=0x614,txflags=0x2705
<491.076877> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<491.076894> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<491.083588> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<491.083614> V34MP, MP detected, starting MP' txmit<491.083616> V34MP, Starting txmit MP again(29), rxflgs=0x614,txflags=0x2705
<491.083657> TimingV34: Timing Offset [ppm] = -5
<491.097066> V34MP, MP detected, starting MP' txmit<491.097077> V34MP, Starting txmit MP again(30), rxflgs=0x614,txflags=0x2705
<491.097279> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<491.097285> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<491.097319> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<491.117032> V34MP, MP detected, starting MP' txmit<491.117042> V34MP, Starting txmit MP again(31), rxflgs=0x614,txflags=0x2705
<491.117212> V34MP, MP detected, starting MP' txmit<491.117214> V34MP, Starting txmit MP again(32), rxflgs=0x614,txflags=0x2705
<491.137233> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<491.137251> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<491.137292> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<491.137299> V34MP, MP detected, starting MP' txmit<491.137301> V34MP, Starting txmit MP again(33), rxflgs=0x614,txflags=0x2705
<491.159414> V34MP, MP detected, starting MP' txmit<491.159435> V34MP, Starting txmit MP again(34), rxflgs=0x614,txflags=0x2705
<491.159656> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<491.159661> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<491.159695> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<491.159703> V34MP, MP detected, starting MP' txmit<491.159705> V34MP, Starting txmit MP again(35), rxflgs=0x614,txflags=0x2705
<491.177591> V34MP, MP detected, starting MP' txmit<491.177603> V34MP, Starting txmit MP again(36), rxflgs=0x614,txflags=0x2705
<491.177732> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<491.177736> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<491.197751> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<491.197767> V34MP, MP detected, starting MP' txmit<491.197770> V34MP, Starting txmit MP again(37), rxflgs=0x614,txflags=0x2705
<491.198491> V34MP, MP detected, starting MP' txmit<491.198501> V34MP, Starting txmit MP again(38), rxflgs=0x614,txflags=0x2705
<491.216832> TimingV34: Timing Offset [ppm] = -1
<491.217481> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<491.217488> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<491.217558> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<491.217567> V34MP, MP detected, starting MP' txmit<491.217599> V34MP, Starting txmit MP again(39), rxflgs=0x614,txflags=0x2705
<491.236870> V34MP, MP detected, starting MP' txmit<491.236883> V34MP, Starting txmit MP again(40), rxflgs=0x614,txflags=0x2705
<491.237161> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<491.237168> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<491.237202> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<491.237209> V34MP, MP detected, starting MP' txmit<491.237212> V34MP, Starting txmit MP again(41), rxflgs=0x614,txflags=0x2705
<491.301179> V34MP, MP detected, starting MP' txmit<491.301194> V34MP, Starting txmit MP again(42), rxflgs=0x614,txflags=0x2705
<491.303418> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<491.303425> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<491.303460> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<491.303466> V34MP, MP detected, starting MP' txmit<491.303468> V34MP, Starting txmit MP again(43), rxflgs=0x614,txflags=0x2705
<491.303676> V34MP, MP detected, starting MP' txmit<491.303680> V34MP, Starting txmit MP again(44), rxflgs=0x614,txflags=0x2705
<491.305909> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<491.305916> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<491.305950> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<491.305958> V34MP, MP detected, starting MP' txmit<491.305959> V34MP, Starting txmit MP again(45), rxflgs=0x614,txflags=0x2705
<491.317616> V34EQU, equerr = 1490, preerr = 0,
<491.317705> V34MP, MP detected, starting MP' txmit<491.317707> V34MP, Starting txmit MP again(46), rxflgs=0x614,txflags=0x2705
<491.318530> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<491.318538> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<491.318572> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<491.318579> V34MP, MP detected, starting MP' txmit<491.318581> V34MP, Starting txmit MP again(47), rxflgs=0x614,txflags=0x2705
<491.370184> TimingV34: Timing Offset [ppm] = -7
<491.370290> V34MP, MP detected, starting MP' txmit<491.370294> V34MP, Starting txmit MP again(48), rxflgs=0x614,txflags=0x2705
<491.393976> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<491.393991> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<491.394029> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<491.394038> V34MP, MP detected, starting MP' txmit<491.394041> V34MP, Starting txmit MP again(49), rxflgs=0x614,txflags=0x2705
<491.396299> V34MP, MP detected, starting MP' txmit<491.396306> V34MP, Starting txmit MP again(50), rxflgs=0x614,txflags=0x2705
<491.396482> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<491.396487> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<491.396520> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<491.396527> V34MP, MP detected, starting MP' txmit<491.396529> V34MP, Starting txmit MP again(51), rxflgs=0x614,txflags=0x2705
<491.398836> V34MP, MP detected, starting MP' txmit<491.398845> V34MP, Starting txmit MP again(52), rxflgs=0x614,txflags=0x2705
<491.399031> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<491.399036> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<491.417218> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<491.417237> V34MP, MP detected, starting MP' txmit<491.417239> V34MP, Starting txmit MP again(53), rxflgs=0x614,txflags=0x2705
<491.417400> V34MP, MP detected, starting MP' txmit<491.417403> V34MP, Starting txmit MP again(54), rxflgs=0x614,txflags=0x2705
<491.437433> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<491.437451> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<491.437484> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<491.437491> V34MP, MP detected, starting MP' txmit<491.437493> V34MP, Starting txmit MP again(55), rxflgs=0x614,txflags=0x2705
<491.457514> V34MP, MP detected, starting MP' txmit<491.457529> V34MP, Starting txmit MP again(56), rxflgs=0x614,txflags=0x2705
<491.457629> TimingV34: Timing Offset [ppm] = -6
<491.457762> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<491.457768> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<491.457810> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<491.457928> V34MP, MP detected, starting MP' txmit<491.457932> V34MP, Starting txmit MP again(57), rxflgs=0x614,txflags=0x2705
<491.477716> V34MP, MP detected, starting MP' txmit<491.477730> V34MP, Starting txmit MP again(58), rxflgs=0x614,txflags=0x2705
<491.496706> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<491.496718> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<491.496752> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<491.496759> V34MP, MP detected, starting MP' txmit<491.496761> V34MP, Starting txmit MP again(59), rxflgs=0x614,txflags=0x2705
<491.496927> V34MP, MP detected, starting MP' txmit<491.496930> V34MP, Starting txmit MP again(60), rxflgs=0x614,txflags=0x2705
<491.516907> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<491.516923> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<491.516958> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<491.516965> V34MP, MP detected, starting MP' txmit<491.516967> V34MP, Starting txmit MP again(61), rxflgs=0x614,txflags=0x2705
<491.536983> V34MP, MP detected, starting MP' txmit<491.537000> V34MP, Starting txmit MP again(62), rxflgs=0x614,txflags=0x2705
<491.537122> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<491.537126> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<491.537158> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<491.537164> V34MP, MP detected, starting MP' txmit<491.537165> V34MP, Starting txmit MP again(63), rxflgs=0x614,txflags=0x2705
<491.557157> V34MP, MP detected, starting MP' txmit<491.557169> V34MP, Starting txmit MP again(64), rxflgs=0x614,txflags=0x2705
<491.577157> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<491.577170> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<491.577206> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<491.577212> V34MP, MP detected, starting MP' txmit<491.577214> V34MP, Starting txmit MP again(65), rxflgs=0x614,txflags=0x2705
<491.577318> TimingV34: Timing Offset [ppm] = -7
<491.577376> V34MP, MP detected, starting MP' txmit<491.577378> V34MP, Starting txmit MP again(66), rxflgs=0x614,txflags=0x2705
<491.597293> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<491.597302> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<491.597334> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<491.597341> V34MP, MP detected, starting MP' txmit<491.597342> V34MP, Starting txmit MP again(67), rxflgs=0x614,txflags=0x2705
<491.597401> V34EQU, equerr = 1421, preerr = 0,
<491.617353> V34MP, MP detected, starting MP' txmit<491.617361> V34MP, Starting txmit MP again(68), rxflgs=0x614,txflags=0x2705
<491.617485> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<491.617488> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<491.637412> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<491.637437> V34MP, MP detected, starting MP' txmit<491.637439> V34MP, Starting txmit MP again(69), rxflgs=0x614,txflags=0x2705
<491.637683> V34MP, MP detected, starting MP' txmit<491.637688> V34MP, Starting txmit MP again(70), rxflgs=0x614,txflags=0x2705
<491.657573> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<491.657586> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<491.657620> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<491.657627> V34MP, MP detected, starting MP' txmit<491.657628> V34MP, Starting txmit MP again(71), rxflgs=0x614,txflags=0x2705
<491.677622> V34MP, MP detected, starting MP' txmit<491.677633> V34MP, Starting txmit MP again(72), rxflgs=0x614,txflags=0x2705
<491.677765> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<491.677769> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<491.677813> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<491.677820> V34MP, MP detected, starting MP' txmit<491.677826> V34MP, Starting txmit MP again(73), rxflgs=0x614,txflags=0x2705
<491.696809> V34MP, MP detected, starting MP' txmit<491.696819> V34MP, Starting txmit MP again(74), rxflgs=0x614,txflags=0x2705
<491.697006> TimingV34: Timing Offset [ppm] = -7
<491.716832> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<491.716848> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<491.716884> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<491.716892> V34MP, MP detected, starting MP' txmit<491.716895> V34MP, Starting txmit MP again(75), rxflgs=0x614,txflags=0x2705
<491.717060> V34MP, MP detected, starting MP' txmit<491.717064> V34MP, Starting txmit MP again(76), rxflgs=0x614,txflags=0x2705
<491.737065> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<491.737080> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<491.737114> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<491.737121> V34MP, MP detected, starting MP' txmit<491.737136> V34MP, Starting txmit MP again(77), rxflgs=0x614,txflags=0x2705
<491.757117> V34MP, MP detected, starting MP' txmit<491.757130> V34MP, Starting txmit MP again(78), rxflgs=0x614,txflags=0x2705
<491.757349> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<491.757354> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<491.757388> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<491.757395> V34MP, MP detected, starting MP' txmit<491.757396> V34MP, Starting txmit MP again(79), rxflgs=0x614,txflags=0x2705
<491.777311> V34MP, MP detected, starting MP' txmit<491.777324> V34MP, Starting txmit MP again(80), rxflgs=0x614,txflags=0x2705
<491.797321> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<491.797334> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<491.797368> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<491.797378> V34MP, MP detected, starting MP' txmit<491.797380> V34MP, Starting txmit MP again(81), rxflgs=0x614,txflags=0x2705
<491.797540> V34MP, MP detected, starting MP' txmit<491.797542> V34MP, Starting txmit MP again(82), rxflgs=0x614,txflags=0x2705
<491.817506> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<491.817518> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<491.817622> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<491.817630> V34MP, MP detected, starting MP' txmit<491.817632> V34MP, Starting txmit MP again(83), rxflgs=0x614,txflags=0x2705
<491.837557> TimingV34: Timing Offset [ppm] = -8
<491.837594> V34MP, MP detected, starting MP' txmit<491.837596> V34MP, Starting txmit MP again(84), rxflgs=0x614,txflags=0x2705
<491.837783> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<491.837792> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<491.837852> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<491.857594> V34MP, MP detected, starting MP' txmit<491.857602> V34MP, Starting txmit MP again(85), rxflgs=0x614,txflags=0x2705
<491.857763> V34MP, MP detected, starting MP' txmit<491.857765> V34MP, Starting txmit MP again(86), rxflgs=0x614,txflags=0x2705
<491.876832> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<491.876850> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<491.876886> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<491.876894> V34MP, MP detected, starting MP' txmit<491.876896> V34MP, Starting txmit MP again(87), rxflgs=0x614,txflags=0x2705
<491.896842> V34MP, MP detected, starting MP' txmit<491.896856> V34MP, Starting txmit MP again(88), rxflgs=0x614,txflags=0x2705
<491.897057> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<491.897063> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<491.897096> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<491.897104> V34MP, MP detected, starting MP' txmit<491.897141> V34MP, Starting txmit MP again(89), rxflgs=0x614,txflags=0x2705
<491.897170> V34EQU, equerr = 1478, preerr = 0,
<491.917060> V34MP, MP detected, starting MP' txmit<491.917074> V34MP, Starting txmit MP again(90), rxflgs=0x614,txflags=0x2705
<491.917282> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<491.917287> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]10)
<491.937104> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<491.937124> V34MP, MP detected, starting MP' txmit<491.937126> V34MP, Starting txmit MP again(91), rxflgs=0x614,txflags=0x2705
<491.937287> V34MP, MP detected, starting MP' txmit<491.937289> V34MP, Starting txmit MP again(92), rxflgs=0x614,txflags=0x2705
<491.957292> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]144, [2]6)
<491.957417> TimingV34: Timing Offset [ppm] = -3
<491.957459> V34MP, MP detected, starting MP' txmit<491.957463> V34MP, Starting txmit MP again(93), rxflgs=0x614,txflags=0x2705
<491.977358> V34MP, MP detected, starting MP' txmit<491.977370> V34MP, Starting txmit MP again(94), rxflgs=0x614,txflags=0x2705
<491.977537> V34MP, MP detected, starting MP' txmit<491.977539> V34MP, Starting txmit MP again(95), rxflgs=0x614,txflags=0x2705
<491.997438> V34RETRAIN, retrain request detected, rtncount = 0 
<491.997457> $!$ 9<777<9B==795;6<9:?;9:9?6<98>8:B8<4C55?;;>478<98>8:B8<39:8><;:478=9D:77;5>4B688D4@????
<491.997460> V34SetupModulator: baudrate 2400, carrier 1800, preemp 0, V90=0. fullReset=0
<491.997464> V34SetupDemodulator: baudrate 2400, carrier 1800
<491.997466> V34SetupModulator: baudrate 600, carrier 2400, preemp 0, V90=0. fullReset=1
<491.997470> V34FLO: Echo running in Original Integer...
<491.997472> V34FLO: Echo running in Original Integer...
<491.997474> V34HSHAKE: txstate XMITMP=>SILENCERETRAIN(rx RECEIVE, mst DET_SYNC, [1]0, [2]0)
<491.997477> V34HSHAKE: rxstate RECEIVE=>WAIT(tx SILENCERETRAIN, mst DET_SYNC, [1]0, [2]0)
<491.997479> V34RETRAIN, going into retrain in Handshake
<491.997487> vpcm: New status 0
<491.997489> vpcm: Re-starting phase II
<492.056809> V34HSHAKE: microstate DET_SYNC=>TX_PHASE1_ANS(tx SILENCERETRAIN, rx WAIT, [1]180, [2]0)
<492.056827> V34HSHAKE: rxstate WAIT=>RX_DPSK(tx SILENCERETRAIN, mst TX_PHASE1_ANS, [1]180, [2]0)
<492.056830> V34HSHAKE: txstate SILENCERETRAIN=>TONE_AB(rx RX_DPSK, mst TX_PHASE1_ANS, [1]180, [2]0)
<492.056833> V34RETRAIN, SILENCERETRAIN finished, rx->rxflgs,=0x1a04,rx->gain=0x2a2,gainestimate=0x2a2
<492.236686> V34HSHAKE: microstate TX_PHASE1_ANS=>RX_PHASE1_ANS(tx TONE_AB, rx RX_DPSK, [1]0, [2]0)
<492.497193> VPcmV34Main: Masking CAS detection after 4800 in train...
<492.517303> On RX_PHASE1_ANS: is short=0, bulkDelay=1956, filtDelay=83
<492.517319> V34 bulk delay estimation 1956 (FAR=1)
<492.517322> V34HSHAKE: microstate RX_PHASE1_ANS=>TX_PHASE2_ANS(tx TONE_AB, rx RX_DPSK, [1]0, [2]83)
<492.517349> V34HSHAKE: microstate TX_PHASE2_ANS=>TX_L1(tx TONE_AB, rx RX_DPSK, [1]0, [2]0)
<492.537441> V34HSHAKE: txstate TONE_AB=>TX_L1(rx RX_DPSK, mst TX_L1, [1]0, [2]42)
<492.537457> V34HSHAKE: rxstate RX_DPSK=>DET_AB(tx TX_L1, mst TX_L1, [1]0, [2]42)
<492.537459> V34RETRAIN, starting DET_AB, rx->rxflgs = 0x1a04,rx->gain=0x2a2
<492.697257> V34HSHAKE: microstate TX_L1=>TX_L2(tx TX_L1, rx DET_AB, [1]1536, [2]42)
<493.117521> V34RETRAIN, End of L2,rx->gain=0x2f3,count1=4120
<493.117537> V34HSHAKE: txstate TX_L1=>TONE_AB(rx DET_AB, mst TX_L2, [1]4120, [2]42)
<493.117540> V34HSHAKE: microstate TX_L2=>TX_PHASE3_ANS(tx TONE_AB, rx DET_AB, [1]4120, [2]42)
<493.117543> V34HSHAKE: rxstate DET_AB=>RX_DPSK(tx TONE_AB, mst TX_PHASE3_ANS, [1]4120, [2]42)
<493.196987> V34HSHAKE: txstate TONE_AB=>SILENCE(rx RX_DPSK, mst TX_PHASE3_ANS, [1]0, [2]162)
<493.197003> V34HSHAKE: microstate TX_PHASE3_ANS=>RX_PHASE2_ANS(tx SILENCE, rx RX_DPSK, [1]0, [2]162)
<493.457543> V34SetupDemodulator: baudrate 2400, carrier 1800
<493.457562> V34HSHAKE: rxstate RX_DPSK=>RX_L1(tx SILENCE, mst RX_PHASE2_ANS, [1]0, [2]20)
<493.477622> V34AGC, rx->gain = 0x14e, at the beginning of RX_L1
<493.896848> V34SetupDemodulator: baudrate 2400, carrier 0
<493.896865> V34HSHAKE: rxstate RX_L1=>RX_DPSK(tx SILENCE, mst RX_PHASE2_ANS, [1]0, [2]1089)
<493.896869> V34HSHAKE: txstate SILENCE=>TONE_AB(rx RX_DPSK, mst RX_PHASE2_ANS, [1]0, [2]1089)
<493.896872> V34HSHAKE: microstate RX_PHASE2_ANS=>DET_SYNC(tx TONE_AB, rx RX_DPSK, [1]0, [2]1089)
<494.197419> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx TONE_AB, rx RX_DPSK, [1]0, [2]0)
<494.197436> DET_SYNC is received in RX_DPSK,rx->gain=0x2a2
<494.337255> V34HSHAKE: txstate TONE_AB=>TX_DPSK(rx RX_DPSK, mst DET_INFO, [1]0, [2]0)
<494.337273> RX MDLENGTH = 0
<494.337276> V34PROBE, snr_L1=25074 , snr_L2=11148 , L2toL1ratio=455  (all not in dB)
<494.337278> V34PROBE, dBcnt=0 , powerReductionReq=0 , gain=674
<494.337280> Sensitive RX Power Reduction mechanism disabled!
<494.337282> V34PROBE, not asking for power reduction
<494.337284> V34PROBE, rx->gain=674 ,(obj->rxinfo0.data[1]&0x80)=128
<494.337286> V34PROBE,0=5,0,0,0,0,5=12,0,12,0,0,10=0,12,0,0,0,12,0,0,0,0,20=0,0,22=1,2,24=4
<494.337290> VpcmV34Main: max V34 baud rate index = 0
<494.337292> V34PREEMPHASIS, - index is 6, baudrate= 3429
<494.337293> V34PROBESELECT, in ANSWER, txbaudrate = 3429,rxbaudrate = 3429,
<494.337295> V34PROBE, rxinfo1c 0xc0,0x0,0x8,0x14,0x6,0x7,0x0,0x61,0x20,0x0
<494.337299> V34HSHAKE: microstate DET_INFO=>INFODONE(tx TX_DPSK, rx RX_DPSK, [1]0, [2]0)
<494.337301> V34PROBE, txinfo1a 0x0,0x1,0xa6,0xd0,0x0,0x0,0x0,0x0,0x0,0x0
<494.456876> V34HSHAKE: txstate TX_DPSK=>TONE_AB(rx RX_DPSK, mst INFODONE, [1]0, [2]0)
<494.478911> V34HSHAKE: txstate TONE_AB=>SILENCE(rx RX_DPSK, mst INFODONE, [1]0, [2]16)
<494.478942> $!$ 9<777<9B==795;5F96>@:D5A39:5>?:79:7>68>?:78C7?79?<;68C7>:=<C:;8E7@:9>?665D396<;8775?39:8?@:C897H9A?:664?7;96?<::474<69;97?4@1F3?????
<494.478945> VPcmV34Main: Wait (before P2 COMPLETE)...
<494.478965> VPcmV34Main: Indicating First P2 COMPLETE... (after 96)
<494.478968> vpcm: New status 1
<494.478970> vpcm: Phase II completed !!!
<494.478989> On PHASE2_COMPLETE: added Silence = 0, p2DelayCntr = 144
<494.478991> VPcmV34Main: Wait (after P2 COMPLETE)...
<494.497049> VPcmV34Main: Wait (after P2 COMPLETE)...
<494.497076> VPcmV34Main: Wait (after P2 COMPLETE)...
<494.497097> VPcmV34Main: Wait (after P2 COMPLETE)...
<494.497117> VPcmV34Main: Wait (after P2 COMPLETE)...
<494.517152> VPcmV34Main: Wait (after P2 COMPLETE)...
<494.517182> VPcmV34Main: Wait (after P2 COMPLETE)...
<494.517203> VPcmV34Main: Wait (after P2 COMPLETE)...
<494.517223> VPcmV34Main: Wait (after P2 COMPLETE)...
<494.537276> VPcmV34Main: Wait (after P2 COMPLETE)...
<494.537311> V34Hshak: Setting up transmitter for phase3...
<494.537318> $!$ 9<777<9B==795;5F96>@:D5A3979?<:;478=9D:7:<8@7G96>C666D7B9C=;9>776;55;D66573E55?::;9;8=9>>E:=477>98>?:E5A3999><:9888B55?:;:888;:9:77C474;65;7764C3999><:9888B55>=:78:8=55;D665:4;6<;87;4C3997><;:88396B:77<2D1C????
<494.537322> $!$ 9<777<9B==795;5F96>@:D5A397<><;:475F9>>E:?8D7:9A:7;68F8@9:?966997>99?<:99;7B9D>E664D39:7><;:9<8;9C>@:D8>3965:76>8:7?9<:7:<8C7:9<:7;98<8=55?;:E474:5>8D4@????
<494.537324> V34TXSCALE, power reduction requested by remote modem is 3 dB
<494.537326> V34TXSCALE, txscale before is 5793, reduced txscale is 3 dB,final txscale is 4099
<494.537328> V34SetupModulator: baudrate 3429, carrier 1959, preemp 0, V90=0. fullReset=1
<494.537332> V34HSHAKE: rxstate RX_DPSK=>WAIT(tx SILENCE, mst INFODONE, [1]0, [2]30)
<494.537334> V34HSHAKE: txstate SILENCE=>SSEG(rx WAIT, mst INFODONE, [1]0, [2]30)
<494.537337> V34FLO: Echo running in Original Integer...
<494.537339> V34FLO: Echo running in Original Integer...
<494.537377> vpcm: New status 2
<494.577510> V34HSHAKE: txstate SSEG=>SBARSEG(rx WAIT, mst INFODONE, [1]0, [2]30)
<494.577564> V34HSHAKE: txstate SBARSEG=>PPSEG(rx WAIT, mst INFODONE, [1]0, [2]30)
<494.657042> V34HSHAKE: txstate PPSEG=>TRNSEG4(rx WAIT, mst INFODONE, [1]2100, [2]30)
<494.657057> V34Hshak: echo start wait time would be: NEC 409 symbols, FEC 1447 symbols...
<494.697208> VPcmV34Main: Echo adapt start reported...
<494.717348> updateAlphaNE: updated 0 => -4063
<494.777650> V34FEC - start FEC adaptation
<494.796688> updateAlphaFE: updated 0 => -662
<495.616880> VPcmV34Main: Echo adapt middle reported...
<496.576903> V34NEC - stop NEC adaptation
<496.657356> V34HSHAKE: txstate TRNSEG4=>JTXMIT(rx WAIT, mst INFODONE, [1]2100, [2]409)
<496.657373> V34Hshak: On J TX start, would freeze EC after bulk delay (978 samples, bulk=1956)
<496.657376> V34HSHAKE: rxstate WAIT=>RECEIVE(tx JTXMIT, mst INFODONE, [1]2100, [2]978)
<496.657380> V34SetupDemodulator: baudrate 3429, carrier 1959
<496.657383> V34AGC, setup receiver gain = 0x29c
<496.657386> V34HSHAKE: microstate INFODONE=>DET_SYNC(tx JTXMIT, rx RECEIVE, [1]2100, [2]978)
<496.917778> S detected,rxflgs= 0x1304,rxgain=0x29c
<496.936955> V34Hshak: on JTXMIT - time to freeze echo...
<496.936972> V34HSHAK: Freeze EC
<496.936976> ==== Near Echo Canceller report ======
<496.936977> ?======= Coefficients[1..144]=========
<496.936980> ?1 -3 2 -1 5 0
<496.936983> ?2 -1 -1 0 -3 0
<496.936984> ?-3 -3 1 -2 -1 -1
<496.936987> ?1 -1 -1 2 -1 0
<496.936989> ?2 -2 -2 -1 -1 -1
<496.936992> ?-1 -1 -3 0 0 -2
<496.936994> ?0 0 1 1 1 1
<496.936997> ?-1 0 -2 -1 1 0
<496.937000> ?1 2 0 -1 -1 1
<496.937003> ?0 0 -1 -1 1 0
<496.937005> ?-2 -2 -1 1 0 0
<496.937007> ?0 -1 -1 -1 -1 -1
<496.937009> ?-2 1 1 -1 -1 -1
<496.937012> ?-1 1 0 -3 -3 -2
<496.937015> ?-2 0 -1 -2 -1 -1
<496.937017> ?0 0 -1 0 1 0
<496.937019> ?-1 0 -1 -2 1 -1
<496.937022> ?0 1 -1 0 1 0
<496.937024> ?0 -1 0 -2 -1 1
<496.937026> ?-1 0 0 -1 1 2
<496.937030> ?0 -2 0 1 -1 -1
<496.937033> ?0 1 0 0 1 -1
<496.937037> ?0 1 0 0 0 0
<496.937039> ?-1 -1 -2 0 0 -2
<496.937040> ==== Far Echo Canceller report ======
<496.937043> ?======= Coefficients[1..144]=========
<496.937044> ?13 89 29 -58 -43 -20
<496.937047> ?37 -3 -97 -64 -27 3
<496.937049> ?-12 -92 -72 -22 6 -4
<496.937052> ?-77 -55 9 30 17 -60
<496.937055> ?-30 61 54 22 -31 0
<496.937057> ?90 55 17 -5 5 89
<496.937059> ?56 -5 0 11 73 43
<496.937061> ?-28 -3 6 34 20 -50
<496.937065> ?-28 -6 -3 -25 -80 -46
<496.937068> ?-19 -24 -46 -91 -28 -14
<496.937071> ?-24 5 -93 -21 63 -68
<496.937074> ?28 -4 -148 117 86 -157
<496.937077> ?131 349 159 -38 -76 -14
<496.937079> ?7 46 56 -25 -15 6
<496.937083> ?18 33 -50 -52 -11 -19
<496.937085> ?5 -52 -73 -20 -27 -7
<496.937086> ?-40 -72 3 0 -12 -20
<496.937089> ?-49 27 33 -15 3 -11
<496.937090> ?31 49 -11 11 16 25
<496.937093> ?49 -3 10 27 12 40
<496.937095> ?4 -6 27 4 9 -6
<496.937098> ?-22 11 -12 -24 -23 -35
<496.937099> ?-6 -16 -35 -24 -24 -4
<496.937102> ?-11 -28 -16 -7 19 5
<496.937114> V34AGC, abcddetect gain = 0x3ad, AGC frozen
<496.956964> S-S1 is detected,rxsymcnt= 126,pllcnt= 2,gain= 0x3ad
<496.956983> V34HSHAKE: txstate JTXMIT=>XMIT0(rx RECEIVE, mst DET_SYNC, [1]3, [2]0)
<497.057379> TimingV34: Timing Offset [ppm] = 111
<497.177121> TimingV34: Timing Offset [ppm] = 26
<497.237445> V34EQU, equerr = 29181, preerr = 23198,
<497.296869> TimingV34: Timing Offset [ppm] = -13
<497.417556> TimingV34: Timing Offset [ppm] = 39
<497.537057> V34EQU, equerr = 160, preerr = 157,
<497.557104> TimingV34: Timing Offset [ppm] = -22
<497.676755> TimingV34: Timing Offset [ppm] = -28
<497.797409> TimingV34: Timing Offset [ppm] = -7
<497.837565> V34EQU, equerr = 97, preerr = 101,
<497.917097> TimingV34: Timing Offset [ppm] = 2
<497.937141> V34HSHAKE: txstate XMIT0=>SSEG(rx RECEIVE, mst DET_SYNC, [1]0, [2]0)
<497.937176> vpcm: New status 3
<497.977319> V34HSHAKE: txstate SSEG=>SBARSEG(rx RECEIVE, mst DET_SYNC, [1]0, [2]0)
<497.977458> V34HSHAKE: txstate SBARSEG=>TRNSEG4A(rx RECEIVE, mst DET_SYNC, [1]0, [2]0)
<498.057179> TimingV34: Timing Offset [ppm] = -4
<498.137653> V34EQU, equerr = 92, preerr = 100,
<498.176885> TimingV34: Timing Offset [ppm] = -4
<498.297658> TimingV34: Timing Offset [ppm] = 1
<498.417305> TimingV34: Timing Offset [ppm] = -14
<498.437249> V34EQU, equerr = 751, preerr = 292,
<498.566155> TimingV34: Timing Offset [ppm] = -8
<498.677721> TimingV34: Timing Offset [ppm] = -10
<498.737037> V34EQU, equerr = 273, preerr = 149,
<498.797914> TimingV34: Timing Offset [ppm] = -9
<498.917338> TimingV34: Timing Offset [ppm] = -10
<499.017511> V34EQU, equerr = 290, preerr = 141,
<499.037133> TimingV34: Timing Offset [ppm] = -10
<499.176851> TimingV34: Timing Offset [ppm] = -1
<499.297228> TimingV34: Timing Offset [ppm] = 6
<499.317437> V34EQU, equerr = 10227, preerr = 11203,
<499.417071> TimingV34: Timing Offset [ppm] = 41
<499.538193> TimingV34: Timing Offset [ppm] = 39
<499.617618> V34EQU, equerr = 631, preerr = 457,
<499.677001> TimingV34: Timing Offset [ppm] = 34
<499.757659> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx TRNSEG4A, rx RECEIVE, [1]0, [2]0)
<499.797682> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<499.797698> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx TRNSEG4A, rx RECEIVE, [1]0, [2]7)
<499.797734> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx TRNSEG4A, rx RECEIVE, [1]0, [2]7)
<499.797760> TimingV34: Timing Offset [ppm] = 8
<499.817073> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<499.817088> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx TRNSEG4A, rx RECEIVE, [1]0, [2]10)
<499.817256> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx TRNSEG4A, rx RECEIVE, [1]0, [2]10)
<499.837439> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<499.837456> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx TRNSEG4A, rx RECEIVE, [1]0, [2]10)
<499.857185> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx TRNSEG4A, rx RECEIVE, [1]0, [2]10)
<499.877669> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<499.877687> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx TRNSEG4A, rx RECEIVE, [1]0, [2]10)
<499.879916> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx TRNSEG4A, rx RECEIVE, [1]0, [2]10)
<499.897582> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<499.897597> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx TRNSEG4A, rx RECEIVE, [1]0, [2]10)
<499.897697> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx TRNSEG4A, rx RECEIVE, [1]0, [2]10)
<499.897749> V34DATARATE, precoefs=0, 0, 0, 0, 0, 0
<499.897753> V34DATARATE,threshold for data rate 14 = 50
<499.897755> V34DATARATE,threshold for data rate 13 = 84
<499.897823> V34DATARATE,threshold for data rate 12 = 131
<499.897827> V34DATARATE,threshold for data rate 11 = 205
<499.897829> V34DATARATE,threshold for data rate 10 = 366
<499.897831> V34DATARATE,threshold for data rate 9 = 572
<499.897834> V34DATARATE,threshold for data rate 8 = 937
<499.897836> V34DATARATE, ethresh data rate = 8,ethreh=631,rate2 = 0x8990,data=937
<499.897840> V34DATARATE, equerr = 631,preerr=457
<499.897842> V34DATARATE, automatic: 19200, min 0, max 33600
<499.897845> V34DATARATE, Final choice data rate = 8, retrainThresh = 1874, renegDownthresh = 1238, renegUpthresh = 572
<499.897850> $!$ :E8E399<><;:477F96??669<89:8?;;88<7:9B:7;8888=9::C668F7G55?9:;8>8>9A>8;8475B88=76B478;9:?;;;997G9>>E:=474<68;=76571F3?????
<499.897853> V34HSHAKE: txstate TRNSEG4A=>XMITMP(rx RECEIVE, mst DET_INFO, [1]0, [2]0)
<499.897856> V34DATARATE, txmp bits 0x85c2,0xfffd,0x0,0x0,0x0
<499.917877> TimingV34: Timing Offset [ppm] = 10
<499.918095> V34EQU, equerr = 394, preerr = 283,
<499.937812> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<499.937833> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]162, [2]10)
<499.937871> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<499.937886> V34MP, Starting txmit MP again(1), rxflgs=0x614,txflags=0x2705
<499.956863> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<499.956880> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]162, [2]10)
<499.956972> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<499.956990> V34MP, MP detected, starting MP' txmit<499.956993> V34MP, Starting txmit MP again(0), rxflgs=0x614,txflags=0x2705
<499.977012> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<499.977028> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]162, [2]10)
<499.977559> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<499.977618> V34MP, Starting txmit MP again(1), rxflgs=0x614,txflags=0x2705
<500.097868> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<500.097887> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]162, [2]10)
<500.097923> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<500.097939> V34MP, MP detected, starting MP' txmit<500.097943> V34MP, Starting txmit MP again(2), rxflgs=0x614,txflags=0x2705
<500.100323> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<500.100331> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]162, [2]10)
<500.100408> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<500.100454> V34MP, MP detected, starting MP' txmit<500.100458> V34MP, Starting txmit MP again(3), rxflgs=0x614,txflags=0x2705
<500.100533> TimingV34: Timing Offset [ppm] = 10
<500.102935> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<500.102943> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]162, [2]10)
<500.105157> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<500.105179> V34MP, MP detected, starting MP' txmit<500.105181> V34MP, Starting txmit MP again(4), rxflgs=0x614,txflags=0x2705
<500.107564> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<500.107571> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]162, [2]10)
<500.107604> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<500.107618> V34MP, MP detected, starting MP' txmit<500.107621> V34MP, Starting txmit MP again(5), rxflgs=0x614,txflags=0x2705
<500.117426> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<500.117440> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]162, [2]10)
<500.117554> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<500.117609> V34MP, MP detected, starting MP' txmit<500.117613> V34MP, Starting txmit MP again(6), rxflgs=0x614,txflags=0x2705
<500.157541> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<500.157558> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]162, [2]10)
<500.157593> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<500.157609> V34MP, MP detected, starting MP' txmit<500.157612> V34MP, Starting txmit MP again(7), rxflgs=0x614,txflags=0x2705
<500.177696> TimingV34: Timing Offset [ppm] = 9
<500.177860> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<500.177867> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]162, [2]10)
<500.177903> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<500.177916> V34MP, MP detected, starting MP' txmit<500.177918> V34MP, Starting txmit MP again(8), rxflgs=0x614,txflags=0x2705
<500.196980> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<500.196997> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]162, [2]10)
<500.197031> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<500.197133> V34MP, MP detected, starting MP' txmit<500.197138> V34MP, Starting txmit MP again(9), rxflgs=0x614,txflags=0x2705
<500.217094> V34EQU, equerr = 463, preerr = 0,
<500.237179> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<500.237199> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]162, [2]10)
<500.237235> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<500.237250> V34MP, MP detected, starting MP' txmit<500.237253> V34MP, Starting txmit MP again(10), rxflgs=0x614,txflags=0x2705
<500.257346> V34MP -MP sequence,33b9,bffe,0,0,0,0,0,0,0,2143
<500.257363> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]162, [2]10)
<500.257475> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]180, [2]10)
<500.257521> V34MP, MP detected, starting MP' txmit<500.257525> V34MP, Starting txmit MP again(11), rxflgs=0x614,txflags=0x2705
<500.277558> V34MP -MP1 sequence,b3b9,bffe,0,0,0,0,0,0,0,664b
<500.277576> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]162, [2]9)
<500.297455> V34MP - E sequence detected,rxflgs= 0x69c
<500.297467> V34DATARATE, preliminary txbitrate 33600,rxbitrate 19200
<500.297470> V34DATARATE, finally txbitrate 33600,rxbitrate 19200
<500.297472> V34DATARATE, for tx data rate - 14, PTC - 48, setting nofTxBits to 16
<500.297487> V34HSHAKE: txstate XMITMP=>EXMIT(rx RECEIVE, mst DET_SYNC, [1]188, [2]0)
<500.297529> V34HSHAKE: txstate EXMIT=>DATAXMIT(rx RECEIVE, mst DET_SYNC, [1]8, [2]0)
<500.297531> V34MP- E transmit completed
<500.297584> TimingV34: Timing Offset [ppm] = 9
<500.337370> vpcm: New status 4
<500.337388> vpcm: Link: DP is V.34, rate: rx 19200, tx 33600
<500.337392> slamr0: modem_update_status: 256
<500.337394> slamr0: --> DP LINK
<500.337396> slamr0: modem set state: 2 --> 4...
<500.337398> slamr0: new state: EC_ESTAB
<500.357324> vpcm: No symbols Requested!!!
<500.357409> vpcm: No symbols Requested!!!
<500.357476> vpcm: No symbols Requested!!!
<500.357544> vpcm: No symbols Requested!!!
<500.377424> vpcm: No symbols Requested!!!
<500.377587> vpcm: No symbols Requested!!!
<500.377632> V34DATA, getting into data mode from Handshake, Tx bit rate - 33600, Rx bit Rate - 19200
<500.776986> slamr0: rx pattern: 0x88.
<500.777002> slamr0: rx pattern: 0x89.
<500.777005> slamr0: rx pattern: 0x88.
<500.777006> slamr0: rx pattern: 0x89.
<500.777007> slamr0: rx pattern: 0x88.
<500.777075> slamr0: rx pattern: 0x89.
<500.777076> slamr0: rx pattern: 0x88.
<500.777077> slamr0: rx pattern: 0x89.
<500.777079> slamr0: rx pattern: 0x88.
<500.777143> slamr0: rx pattern: 0x89.
<500.777146> slamr0: rx pattern: 0x88.
<500.777148> slamr0: rx pattern: 0x89.
<500.777150> slamr0: rx pattern: 0x88.
<500.777152> slamr0: rx pattern: 0x89.
<500.777218> slamr0: rx pattern: 0x88.
<500.777220> slamr0: rx pattern: 0x89.
<500.777222> slamr0: rx pattern: 0x88.
<500.777224> slamr0: rx pattern: 0x89.
<500.777225> slamr0: rx pattern: 0x88.
<500.777227> slamr0: rx pattern: 0x89.
<500.777229> slamr0: rx pattern: 0x88.
<500.796943> slamr0: detector finished.
<500.796961> slamr0: modem_update_status: 1024
<500.796964> slamr0: --> PACK LINK
<500.796966> slamr0: hdlc_start...
<500.977205> V34EQU, equerr = 461, preerr = 0,
<501.357181> slamr0: modem update config...
<501.357197> slamr0: ec = 1 (1)
<501.357199> slamr0: ec_detector = 1 (1)
<501.357202> slamr0: ec_tx_win_size = 15 (15)
<501.357205> slamr0: ec_rx_win_size = 15 (15)
<501.357206> slamr0: ec_tx_info_size = 128 (128)
<501.357299> slamr0: ec_rx_info_size = 128 (128)
<501.357303> slamr0: comp = 3 (3)
<501.357305> slamr0: comp_dict_size = 2048 (2048)
<501.357307> slamr0: comp_max_string = 32 (34)
<501.577357> V34EQU, equerr = 423, preerr = 0,
<502.177516> V34EQU, equerr = 429, preerr = 0,
<502.776692> V34EQU, equerr = 410, preerr = 0,
<502.817005> slamr0: modem update config...
<502.817020> slamr0: ec = 1 (1)
<502.817022> slamr0: ec_detector = 1 (1)
<502.817024> slamr0: ec_tx_win_size = 15 (15)
<502.817025> slamr0: ec_rx_win_size = 15 (15)
<502.817027> slamr0: ec_tx_info_size = 128 (128)
<502.817028> slamr0: ec_rx_info_size = 128 (128)
<502.817030> slamr0: comp = 3 (3)
<502.817031> slamr0: comp_dict_size = 2048 (2048)
<502.817032> slamr0: comp_max_string = 32 (32)
<503.297682> TimingV34: Timing Offset [ppm] = 0
<503.357820> V34EQU, equerr = 419, preerr = 0,
<503.987512> V34EQU, equerr = 489, preerr = 0,
<504.077153> slamr0: hdlc frame integrity error.
<504.077235> slamr0: hdlc frame integrity error.
<504.097359> slamr0: hdlc frame integrity error.
<504.097376> slamr0: hdlc frame integrity error.
<504.117237> slamr0: hdlc frame integrity error.
<504.197744> slamr0: hdlc frame integrity error.
<504.237268> slamr0: hdlc frame integrity error.
<504.258052> slamr0: hdlc frame integrity error.
<504.277196> slamr0: hdlc frame integrity error.
<504.397005> slamr0: hdlc frame integrity error.
<504.397093> slamr0: hdlc frame integrity error.
<504.457420> slamr0: hdlc frame integrity error.
<504.477485> slamr0: hdlc frame integrity error.
<504.537212> slamr0: hdlc frame fcs error.
<504.557121> V34EQU, equerr = 2563, preerr = 0,
<504.577547> slamr0: hdlc frame integrity error.
<504.617399> slamr0: hdlc frame integrity error.
<504.657729> slamr0: hdlc frame integrity error.
<504.658060> slamr0: hdlc frame fcs error.
<504.677732> slamr0: hdlc frame integrity error.
<504.717168> slamr0: hdlc frame fcs error.
<504.737340> slamr0: hdlc frame integrity error.
<504.777757> slamr0: hdlc frame integrity error.
<504.797672> slamr0: hdlc frame integrity error.
<504.837852> slamr0: hdlc frame integrity error.
<504.837934> slamr0: hdlc frame integrity error.
<504.903074> slamr0: hdlc frame integrity error.
<504.917205> slamr0: hdlc frame integrity error.
<505.041107> slamr0: hdlc frame integrity error.
<505.045947> slamr0: hdlc frame fcs error.
<505.045956> slamr0: hdlc frame integrity error.
<505.046024> slamr0: hdlc frame integrity error.
<505.057621> $!$ 9<777<9B==795;6<9:?;9:9?6<98>8:B8<4C55?;;>478<98>8:B8<39:8><;:478=9D:77;5>4B688D4@????
<505.057635> V34SetupModulator: baudrate 2400, carrier 1800, preemp 0, V90=0. fullReset=0
<505.057641> V34SetupDemodulator: baudrate 2400, carrier 1800
<505.057643> V34SetupModulator: baudrate 600, carrier 2400, preemp 0, V90=0. fullReset=1
<505.057747> V34FLO: Echo running in Original Integer...
<505.057752> V34FLO: Echo running in Original Integer...
<505.057754> V34HSHAKE: txstate DATAXMIT=>SILENCERETRAIN(rx RECEIVE, mst DET_SYNC, [1]0, [2]0)
<505.057757> V34HSHAKE: rxstate RECEIVE=>WAIT(tx SILENCERETRAIN, mst DET_SYNC, [1]0, [2]0)
<505.057760> V34TIMING,Retrain started
<505.057765> vpcm: New status 7
<505.096901> vpcm: New status 0
<505.096919> vpcm: Re-starting phase II
<505.137154> V34HSHAKE: microstate DET_SYNC=>TX_PHASE1_ANS(tx SILENCERETRAIN, rx WAIT, [1]180, [2]0)
<505.137172> V34HSHAKE: rxstate WAIT=>RX_DPSK(tx SILENCERETRAIN, mst TX_PHASE1_ANS, [1]180, [2]0)
<505.137175> V34HSHAKE: txstate SILENCERETRAIN=>TONE_AB(rx RX_DPSK, mst TX_PHASE1_ANS, [1]180, [2]0)
<505.137177> V34RETRAIN, SILENCERETRAIN finished, rx->rxflgs,=0x1a04,rx->gain=0x29c,gainestimate=0x29c
<505.397380> TX_PHASE_ANS: count3 = 0 
<505.397396> TX_PHASE_ANS: count3 = 1 
<505.397400> TX_PHASE_ANS: count3 = 2 
<505.397403> TX_PHASE_ANS: count3 = 3 
<505.557370> VPcmV34Main: Masking CAS detection after 4800 in train...
<505.577429> V34HSHAKE: microstate TX_PHASE1_ANS=>RX_PHASE1_ANS(tx TONE_AB, rx RX_DPSK, [1]0, [2]0)
<505.955377> On RX_PHASE1_ANS: is short=0, bulkDelay=2396, filtDelay=83
<505.955395> V34 bulk delay estimation 2396 (FAR=1)
<505.955397> V34HSHAKE: microstate RX_PHASE1_ANS=>TX_PHASE2_ANS(tx TONE_AB, rx RX_DPSK, [1]0, [2]83)
<505.955424> V34HSHAKE: microstate TX_PHASE2_ANS=>TX_L1(tx TONE_AB, rx RX_DPSK, [1]0, [2]0)
<505.957630> V34HSHAKE: txstate TONE_AB=>TX_L1(rx RX_DPSK, mst TX_L1, [1]0, [2]42)
<505.957645> V34HSHAKE: rxstate RX_DPSK=>DET_AB(tx TX_L1, mst TX_L1, [1]0, [2]42)
<505.957648> V34RETRAIN, starting DET_AB, rx->rxflgs = 0x1a04,rx->gain=0x29c
<506.077659> V34HSHAKE: microstate TX_L1=>TX_L2(tx TX_L1, rx DET_AB, [1]1536, [2]42)
<506.556933> V34RETRAIN, End of L2,rx->gain=0x348,count1=4556
<506.556948> V34HSHAKE: txstate TX_L1=>TONE_AB(rx DET_AB, mst TX_L2, [1]4556, [2]42)
<506.556952> V34HSHAKE: microstate TX_L2=>TX_PHASE3_ANS(tx TONE_AB, rx DET_AB, [1]4556, [2]42)
<506.556955> V34HSHAKE: rxstate DET_AB=>RX_DPSK(tx TONE_AB, mst TX_PHASE3_ANS, [1]4556, [2]42)
<506.638045> V34HSHAKE: txstate TONE_AB=>SILENCE(rx RX_DPSK, mst TX_PHASE3_ANS, [1]0, [2]162)
<506.638063> V34HSHAKE: microstate TX_PHASE3_ANS=>RX_PHASE2_ANS(tx SILENCE, rx RX_DPSK, [1]0, [2]162)
<506.936958> V34SetupDemodulator: baudrate 2400, carrier 1800
<506.936978> V34HSHAKE: rxstate RX_DPSK=>RX_L1(tx SILENCE, mst RX_PHASE2_ANS, [1]0, [2]20)
<506.957035> V34AGC, rx->gain = 0x173, at the beginning of RX_L1
<507.377926> V34SetupDemodulator: baudrate 2400, carrier 0
<507.378288> V34HSHAKE: rxstate RX_L1=>RX_DPSK(tx SILENCE, mst RX_PHASE2_ANS, [1]0, [2]1089)
<507.378292> V34HSHAKE: txstate SILENCE=>TONE_AB(rx RX_DPSK, mst RX_PHASE2_ANS, [1]0, [2]1089)
<507.378294> V34HSHAKE: microstate RX_PHASE2_ANS=>DET_SYNC(tx TONE_AB, rx RX_DPSK, [1]0, [2]1089)
<507.717641> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx TONE_AB, rx RX_DPSK, [1]0, [2]0)
<507.717660> DET_SYNC is received in RX_DPSK,rx->gain=0x29c
<507.857436> V34HSHAKE: txstate TONE_AB=>TX_DPSK(rx RX_DPSK, mst DET_INFO, [1]0, [2]0)
<507.858798> RX MDLENGTH = 0
<507.858802> V34PROBE, snr_L1=25177 , snr_L2=9096 , L2toL1ratio=370  (all not in dB)
<507.858804> V34PROBE, dBcnt=0 , powerReductionReq=0 , gain=668
<507.858806> Sensitive RX Power Reduction mechanism disabled!
<507.858808> V34PROBE, not asking for power reduction
<507.858809> V34PROBE, rx->gain=668 ,(obj->rxinfo0.data[1]&0x80)=128
<507.858811> V34PROBE,0=4,0,0,0,0,5=12,0,12,0,0,10=0,12,0,0,0,12,0,0,0,0,20=0,0,22=0,2,24=4
<507.858815> VpcmV34Main: max V34 baud rate index = 0
<507.858816> V34PREEMPHASIS, - index is 6, baudrate= 3429
<507.858818> V34PROBESELECT, in ANSWER, txbaudrate = 3429,rxbaudrate = 3429,
<507.858820> V34PROBE, rxinfo1c 0xc0,0x0,0x8,0x14,0x6,0x7,0x0,0x61,0x20,0x0
<507.858823> V34HSHAKE: microstate DET_INFO=>INFODONE(tx TX_DPSK, rx RX_DPSK, [1]0, [2]0)
<507.858826> V34PROBE, txinfo1a 0x0,0x1,0xa6,0xd0,0x0,0x0,0x0,0x0,0x0,0x0
<507.996898> V34HSHAKE: txstate TX_DPSK=>TONE_AB(rx RX_DPSK, mst INFODONE, [1]0, [2]0)
<507.996948> V34HSHAKE: txstate TONE_AB=>SILENCE(rx RX_DPSK, mst INFODONE, [1]0, [2]16)
<507.996963> $!$ 9<777<9B==795;5F96>@:D5A39:5>?:79:7>68>?:78C7?79?<;68C7>:=<C:;8E7@:9>?665D396<;97>5?39:8?@:C897H9A?:664?7;96?<::474<69;97?4@1F3?????
<507.996966> VPcmV34Main: Wait (before P2 COMPLETE)...
<507.996987> VPcmV34Main: Indicating First P2 COMPLETE... (after 96)
<507.996989> vpcm: New status 1
<507.996991> vpcm: Phase II completed !!!
<507.997011> On PHASE2_COMPLETE: added Silence = 0, p2DelayCntr = 144
<507.997013> VPcmV34Main: Wait (after P2 COMPLETE)...
<508.017053> VPcmV34Main: Wait (after P2 COMPLETE)...
<508.017090> VPcmV34Main: Wait (after P2 COMPLETE)...
<508.017113> VPcmV34Main: Wait (after P2 COMPLETE)...
<508.017133> VPcmV34Main: Wait (after P2 COMPLETE)...
<508.037174> VPcmV34Main: Wait (after P2 COMPLETE)...
<508.037209> VPcmV34Main: Wait (after P2 COMPLETE)...
<508.037229> VPcmV34Main: Wait (after P2 COMPLETE)...
<508.037250> VPcmV34Main: Wait (after P2 COMPLETE)...
<508.057320> VPcmV34Main: Wait (after P2 COMPLETE)...
<508.057358> V34Hshak: Setting up transmitter for phase3...
<508.057366> $!$ 9<777<9B==795;5F96>@:D5A3979?<:;478=9D:7:<8@7G96>C666D7B9C=;9>776;55;D66573E55?::;9;8=9>>E:=477>98>?:E5A3999><:9888B55?:;:888;:9:77C474;65;7764C3999><:9888B55>=:78:8=55;D665:4;6<;87;4C3997><;:88396B:77<2D1C????
<508.057371> $!$ 9<777<9B==795;5F96>@:D5A397<><;:475F9>>E:?8D7:9A:7;68F8@9:?966997>99?<:99;7B9D>E664D39:7><;:9<8;9C>@:D8>3965:76>8:7?9<:7:<8C7:9<:7;98<8=55?;:E474:5>8D4@????
<508.057374> V34TXSCALE, power reduction requested by remote modem is 3 dB
<508.057376> V34TXSCALE, txscale before is 5793, reduced txscale is 3 dB,final txscale is 4099
<508.057380> V34SetupModulator: baudrate 3429, carrier 1959, preemp 0, V90=0. fullReset=1
<508.057385> V34HSHAKE: rxstate RX_DPSK=>WAIT(tx SILENCE, mst INFODONE, [1]0, [2]30)
<508.057388> V34HSHAKE: txstate SILENCE=>SSEG(rx WAIT, mst INFODONE, [1]0, [2]30)
<508.057392> V34FLO: Echo running in Original Integer...
<508.057394> V34FLO: Echo running in Original Integer...
<508.057432> vpcm: New status 2
<508.097607> V34HSHAKE: txstate SSEG=>SBARSEG(rx WAIT, mst INFODONE, [1]0, [2]30)
<508.097660> V34HSHAKE: txstate SBARSEG=>PPSEG(rx WAIT, mst INFODONE, [1]0, [2]30)
<508.177249> V34HSHAKE: txstate PPSEG=>TRNSEG4(rx WAIT, mst INFODONE, [1]2540, [2]30)
<508.177264> V34Hshak: echo start wait time would be: NEC 409 symbols, FEC 1604 symbols...
<508.217369> VPcmV34Main: Echo adapt start reported...
<508.237464> updateAlphaNE: updated 0 => -4063
<508.391517> V34FEC - start FEC adaptation
<509.157872> VPcmV34Main: Echo adapt middle reported...
<510.096905> V34NEC - stop NEC adaptation
<510.217834> V34HSHAKE: txstate TRNSEG4=>JTXMIT(rx WAIT, mst INFODONE, [1]2540, [2]409)
<510.217859> V34Hshak: On J TX start, would freeze EC after bulk delay (1198 samples, bulk=2396)
<510.217862> V34HSHAKE: rxstate WAIT=>RECEIVE(tx JTXMIT, mst INFODONE, [1]2540, [2]1198)
<510.217866> V34SetupDemodulator: baudrate 3429, carrier 1959
<510.217869> V34AGC, setup receiver gain = 0x2e6
<510.217871> V34HSHAKE: microstate INFODONE=>DET_SYNC(tx JTXMIT, rx RECEIVE, [1]2540, [2]1198)
<510.537485> S detected,rxflgs= 0x1304,rxgain=0x2e6
<510.557580> V34AGC, abcddetect gain = 0x3a4, AGC frozen
<510.577695> S-S1 is detected,rxsymcnt= 127,pllcnt= 2,gain= 0x3a4
<510.577709> V34HSHAKE: txstate JTXMIT=>XMIT0(rx RECEIVE, mst DET_SYNC, [1]5, [2]1)
<510.657645> TimingV34: Timing Offset [ppm] = 59
<510.796980> TimingV34: Timing Offset [ppm] = 4
<510.858943> V34EQU, equerr = 31259, preerr = 24403,
<510.917521> TimingV34: Timing Offset [ppm] = -10
<511.036871> TimingV34: Timing Offset [ppm] = 31
<511.157419> V34EQU, equerr = 179, preerr = 170,
<511.157731> TimingV34: Timing Offset [ppm] = -14
<511.296725> TimingV34: Timing Offset [ppm] = -24
<511.421294> TimingV34: Timing Offset [ppm] = -12
<511.437893> V34EQU, equerr = 102, preerr = 105,
<511.537371> TimingV34: Timing Offset [ppm] = 4
<511.577679> V34HSHAKE: txstate XMIT0=>SSEG(rx RECEIVE, mst DET_SYNC, [1]0, [2]1)
<511.577853> vpcm: New status 3
<511.616925> V34HSHAKE: txstate SSEG=>SBARSEG(rx RECEIVE, mst DET_SYNC, [1]0, [2]1)
<511.617003> V34HSHAKE: txstate SBARSEG=>TRNSEG4A(rx RECEIVE, mst DET_SYNC, [1]0, [2]1)
<511.657329> TimingV34: Timing Offset [ppm] = -9
<511.737721> V34EQU, equerr = 94, preerr = 100,
<511.796812> TimingV34: Timing Offset [ppm] = -5
<511.917585> TimingV34: Timing Offset [ppm] = -3
<512.037496> TimingV34: Timing Offset [ppm] = -8
<512.037612> V34EQU, equerr = 119, preerr = 118,
<512.157294> TimingV34: Timing Offset [ppm] = -16
<512.368169> TimingV34: Timing Offset [ppm] = -7
<512.375548> V34EQU, equerr = 141, preerr = 149,
<512.417525> TimingV34: Timing Offset [ppm] = -4
<512.537653> TimingV34: Timing Offset [ppm] = -15
<512.637218> V34EQU, equerr = 131, preerr = 139,
<512.657325> TimingV34: Timing Offset [ppm] = -5
<512.777067> TimingV34: Timing Offset [ppm] = -8
<512.916847> TimingV34: Timing Offset [ppm] = -7
<512.937383> V34EQU, equerr = 133, preerr = 145,
<513.037783> TimingV34: Timing Offset [ppm] = -8
<513.157522> TimingV34: Timing Offset [ppm] = 3
<513.238158> V34EQU, equerr = 142, preerr = 156,
<513.283625> TimingV34: Timing Offset [ppm] = -5
<513.416861> TimingV34: Timing Offset [ppm] = -7
<513.416926> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx TRNSEG4A, rx RECEIVE, [1]0, [2]1)
<513.437098> V34MP -MP sequence,30f9,bffe,0,0,0,0,0,0,0,d35d
<513.437116> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx TRNSEG4A, rx RECEIVE, [1]0, [2]8)
<513.437155> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx TRNSEG4A, rx RECEIVE, [1]0, [2]8)
<513.457281> V34MP -MP sequence,30f9,bffe,0,0,0,0,0,0,0,d35d
<513.457292> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx TRNSEG4A, rx RECEIVE, [1]0, [2]10)
<513.477582> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx TRNSEG4A, rx RECEIVE, [1]0, [2]10)
<513.497404> V34MP -MP sequence,30f9,bffe,0,0,0,0,0,0,0,d35d
<513.497422> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx TRNSEG4A, rx RECEIVE, [1]0, [2]10)
<513.497464> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx TRNSEG4A, rx RECEIVE, [1]0, [2]10)
<513.517761> V34MP -MP sequence,30f9,bffe,0,0,0,0,0,0,0,d35d
<513.517775> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx TRNSEG4A, rx RECEIVE, [1]0, [2]10)
<513.517819> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx TRNSEG4A, rx RECEIVE, [1]0, [2]10)
<513.537817> TimingV34: Timing Offset [ppm] = -5
<513.537866> V34EQU, equerr = 135, preerr = 142,
<513.556862> V34MP -MP sequence,30f9,bffe,0,0,0,0,0,0,0,d35d
<513.556883> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx TRNSEG4A, rx RECEIVE, [1]0, [2]10)
<513.557038> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx TRNSEG4A, rx RECEIVE, [1]0, [2]10)
<513.577350> V34MP -MP sequence,30f9,bffe,0,0,0,0,0,0,0,d35d
<513.577367> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx TRNSEG4A, rx RECEIVE, [1]0, [2]10)
<513.577407> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx TRNSEG4A, rx RECEIVE, [1]0, [2]10)
<513.597151> V34DATARATE, precoefs=0, 0, 0, 0, 0, 0
<513.597168> V34DATARATE,threshold for data rate 14 = 50
<513.597171> V34DATARATE,threshold for data rate 13 = 84
<513.597173> V34DATARATE,threshold for data rate 12 = 131
<513.597174> V34DATARATE,threshold for data rate 11 = 205
<513.597175> V34DATARATE, ethresh data rate = 11,ethreh=135,rate2 = 0x89b0,data=205
<513.597177> V34DATARATE, equerr = 135,preerr=142
<513.597179> V34DATARATE, automatic: 26400, min 0, max 33600
<513.597181> V34DATARATE, Final choice data rate = 11, retrainThresh = 410, renegDownthresh = 285, renegUpthresh = 131
<513.597186> $!$ :E8E399<><;:477F96??669<89:8?;;88<7:9B:7;8888=9::C668F7G55?9:;8>8>9A>8;8475B88=76B478;9:?;;;997G9>>E:=474<68;=76571F3?????
<513.597188> V34HSHAKE: txstate TRNSEG4A=>XMITMP(rx RECEIVE, mst DET_INFO, [1]0, [2]5)
<513.597190> V34DATARATE, txmp bits 0xb5c2,0xfffd,0x0,0x0,0x0
<513.597340> V34MP -MP sequence,30f9,bffe,0,0,0,0,0,0,0,d35d
<513.597345> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]140, [2]10)
<513.597385> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]176, [2]10)
<513.597401> V34MP, Starting txmit MP again(1), rxflgs=0x614,txflags=0x2701
<513.617329> V34MP, MP detected, starting MP' txmit<513.617338> V34MP, Starting txmit MP again(0), rxflgs=0x614,txflags=0x2701
<513.637586> V34MP -MP sequence,30f9,bffe,0,0,0,0,0,0,0,d35d
<513.637602> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]140, [2]10)
<513.637733> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]176, [2]10)
<513.637816> V34MP, Starting txmit MP again(1), rxflgs=0x614,txflags=0x2701
<513.657796> V34MP, MP detected, starting MP' txmit<513.657813> V34MP, Starting txmit MP again(2), rxflgs=0x614,txflags=0x2701
<513.701592> V34MP -MP sequence,30f9,bffe,0,0,0,0,0,0,0,d35d
<513.701612> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]140, [2]10)
<513.701651> TimingV34: Timing Offset [ppm] = -5
<513.701660> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]176, [2]10)
<513.701982> V34MP, MP detected, starting MP' txmit<513.701990> V34MP, Starting txmit MP again(3), rxflgs=0x614,txflags=0x2701
<513.704349> V34MP, MP detected, starting MP' txmit<513.704357> V34MP, Starting txmit MP again(4), rxflgs=0x614,txflags=0x2701
<513.704608> V34MP -MP sequence,30f9,bffe,0,0,0,0,0,0,0,d35d
<513.704615> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]140, [2]10)
<513.707101> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]176, [2]10)
<513.707186> V34MP, MP detected, starting MP' txmit<513.707191> V34MP, Starting txmit MP again(5), rxflgs=0x614,txflags=0x2701
<513.707455> V34MP, MP detected, starting MP' txmit<513.707460> V34MP, Starting txmit MP again(6), rxflgs=0x614,txflags=0x2701
<513.717557> V34MP -MP sequence,30f9,bffe,0,0,0,0,0,0,0,d35d
<513.717575> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]140, [2]10)
<513.718196> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]176, [2]10)
<513.718435> V34MP, MP detected, starting MP' txmit<513.718439> V34MP, Starting txmit MP again(7), rxflgs=0x614,txflags=0x2701
<513.738133> V34MP, MP detected, starting MP' txmit<513.738150> V34MP, Starting txmit MP again(8), rxflgs=0x614,txflags=0x2701
<513.738296> V34MP -MP sequence,30f9,bffe,0,0,0,0,0,0,0,d35d
<513.738300> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]140, [2]10)
<513.738341> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]176, [2]10)
<513.738355> V34MP, MP detected, starting MP' txmit<513.738357> V34MP, Starting txmit MP again(9), rxflgs=0x614,txflags=0x2701
<513.848963> V34MP, MP detected, starting MP' txmit<513.848979> V34MP, Starting txmit MP again(10), rxflgs=0x614,txflags=0x2701
<513.849347> V34MP -MP sequence,30f9,bffe,0,0,0,0,0,0,0,d35d
<513.849356> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]140, [2]10)
<513.851846> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]176, [2]10)
<513.852066> V34MP, MP detected, starting MP' txmit<513.852073> V34MP, Starting txmit MP again(11), rxflgs=0x614,txflags=0x2701
<513.852278> V34MP, MP detected, starting MP' txmit<513.852282> V34MP, Starting txmit MP again(12), rxflgs=0x614,txflags=0x2701
<513.852289> TimingV34: Timing Offset [ppm] = -3
<513.854587> V34MP -MP sequence,30f9,bffe,0,0,0,0,0,0,0,d35d
<513.854599> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]140, [2]10)
<513.855305> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]176, [2]10)
<513.855354> V34MP, MP detected, starting MP' txmit<513.855357> V34MP, Starting txmit MP again(13), rxflgs=0x614,txflags=0x2701
<513.857728> V34MP, MP detected, starting MP' txmit<513.857738> V34MP, Starting txmit MP again(14), rxflgs=0x614,txflags=0x2701
<513.858451> V34MP -MP sequence,30f9,bffe,0,0,0,0,0,0,0,d35d
<513.858460> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]140, [2]10)
10:35:13.885           pjsua_core.c  .RX 437 bytes Request msg OPTIONS/cseq=23361 (rdata0x78f66c001bd8) from UDP 192.168.88.122:5060:
OPTIONS sip:6000@192.168.88.123:44702;ob SIP/2.0
Via: SIP/2.0/UDP 192.168.88.122:5060;rport;branch=z9hG4bKPj269323c2-0692-4f2e-a492-f3155a7c76b8
From: <sip:6000@192.168.88.122>;tag=17c5e6c5-4dfd-4d14-8d34-24db244db764
To: <sip:6000@192.168.88.123;ob>
Contact: <sip:6000@192.168.88.122:5060>
Call-ID: 4cada772-4833-4b3e-bc27-4955b4368e16
CSeq: 23361 OPTIONS
Max-Forwards: 70
User-Agent: Asterisk PBX 22.5.1
Content-Length:  0


--end msg--
10:35:13.885           pjsua_core.c  .TX 813 bytes Response msg 200/OPTIONS/cseq=23361 (tdta0x78f66c0238e8) to UDP 192.168.88.122:5060:
SIP/2.0 200 OK
Via: SIP/2.0/UDP 192.168.88.122:5060;rport=5060;received=192.168.88.122;branch=z9hG4bKPj269323c2-0692-4f2e-a492-f3155a7c76b8
Call-ID: 4cada772-4833-4b3e-bc27-4955b4368e16
From: <sip:6000@192.168.88.122>;tag=17c5e6c5-4dfd-4d14-8d34-24db244db764
To: <sip:6000@192.168.88.123;ob>;tag=z9hG4bKPj269323c2-0692-4f2e-a492-f3155a7c76b8
CSeq: 23361 OPTIONS
Allow: PRACK, INVITE, ACK, BYE, CANCEL, UPDATE, INFO, SUBSCRIBE, NOTIFY, REFER, MESSAGE, OPTIONS
Accept: application/sdp, application/pidf+xml, application/xpidf+xml, application/dialog-info+xml, application/simple-message-summary, message/sipfrag;version=2.0, application/im-iscomposing+xml, text/plain
Supported: replaces, 100rel, timer, norefersub, trickle-ice
Allow-Events: presence, dialog, message-summary, refer
Content-Length:  0


--end msg--
<513.885447> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]176, [2]10)
<513.885473> V34MP, MP detected, starting MP' txmit<513.885477> V34MP, Starting txmit MP again(15), rxflgs=0x614,txflags=0x2701
<513.887822> V34EQU, equerr = 132, preerr = 26,
<513.887877> V34MP, MP detected, starting MP' txmit<513.887881> V34MP, Starting txmit MP again(16), rxflgs=0x614,txflags=0x2701
<513.890396> V34MP -MP sequence,30f9,bffe,0,0,0,0,0,0,0,d35d
<513.890407> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]140, [2]10)
<513.890508> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]176, [2]10)
<513.890562> V34MP, MP detected, starting MP' txmit<513.890566> V34MP, Starting txmit MP again(17), rxflgs=0x614,txflags=0x2701
<513.890803> V34MP, MP detected, starting MP' txmit<513.890807> V34MP, Starting txmit MP again(18), rxflgs=0x614,txflags=0x2701
<513.893099> V34MP -MP sequence,30f9,bffe,0,0,0,0,0,0,0,d35d
<513.893109> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]140, [2]10)
<513.893154> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]176, [2]10)
<513.893169> V34MP, MP detected, starting MP' txmit<513.893172> V34MP, Starting txmit MP again(19), rxflgs=0x614,txflags=0x2701
<513.897641> V34MP, MP detected, starting MP' txmit<513.897650> V34MP, Starting txmit MP again(20), rxflgs=0x614,txflags=0x2701
<513.897892> V34MP -MP sequence,30f9,bffe,0,0,0,0,0,0,0,d35d
<513.897897> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]140, [2]10)
<513.898000> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]172, [2]10)
<513.917783> V34MP, MP detected, starting MP' txmit<513.917806> V34MP, Starting txmit MP again(21), rxflgs=0x614,txflags=0x2701
<513.917926> TimingV34: Timing Offset [ppm] = -8
<513.918096> V34MP, MP detected, starting MP' txmit<513.918099> V34MP, Starting txmit MP again(22), rxflgs=0x614,txflags=0x2701
<513.936905> V34MP -MP sequence,30f9,bffe,0,0,0,0,0,0,0,d35d
<513.937286> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]136, [2]10)
<513.937609> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]176, [2]10)
<513.937799> V34MP, MP detected, starting MP' txmit<513.937806> V34MP, Starting txmit MP again(23), rxflgs=0x614,txflags=0x2701
<513.957163> V34MP, MP detected, starting MP' txmit<513.957176> V34MP, Starting txmit MP again(24), rxflgs=0x614,txflags=0x2701
<513.957326> V34MP -MP sequence,30f9,bffe,0,0,0,0,0,0,0,d35d
<513.957330> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]140, [2]10)
<513.957507> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]176, [2]10)
<513.957557> V34MP, MP detected, starting MP' txmit<513.957561> V34MP, Starting txmit MP again(25), rxflgs=0x614,txflags=0x2701
<513.977225> V34MP, MP detected, starting MP' txmit<513.977235> V34MP, Starting txmit MP again(26), rxflgs=0x614,txflags=0x2701
<513.977499> V34MP -MP sequence,30f9,bffe,0,0,0,0,0,0,0,d35d
<513.977505> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]140, [2]10)
<513.997273> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]176, [2]10)
<513.997805> V34MP, MP detected, starting MP' txmit<513.997814> V34MP, Starting txmit MP again(27), rxflgs=0x614,txflags=0x2701
<513.998181> V34MP, MP detected, starting MP' txmit<513.998186> V34MP, Starting txmit MP again(28), rxflgs=0x614,txflags=0x2701
<514.017503> V34MP -MP sequence,30f9,bffe,0,0,0,0,0,0,0,d35d
<514.017521> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]140, [2]10)
<514.017653> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]176, [2]10)
<514.017704> V34MP, MP detected, starting MP' txmit<514.017709> V34MP, Starting txmit MP again(29), rxflgs=0x614,txflags=0x2701
<514.037585> V34MP, MP detected, starting MP' txmit<514.037598> V34MP, Starting txmit MP again(30), rxflgs=0x614,txflags=0x2701
<514.037774> TimingV34: Timing Offset [ppm] = -10
<514.037916> V34MP -MP sequence,30f9,bffe,0,0,0,0,0,0,0,d35d
<514.037921> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]140, [2]10)
<514.037994> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]172, [2]10)
<514.038039> V34MP, MP detected, starting MP' txmit<514.038042> V34MP, Starting txmit MP again(31), rxflgs=0x614,txflags=0x2701
<514.057821> V34MP, MP detected, starting MP' txmit<514.057838> V34MP, Starting txmit MP again(32), rxflgs=0x614,txflags=0x2701
<514.076773> V34MP -MP sequence,30f9,bffe,0,0,0,0,0,0,0,d35d
<514.076793> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]136, [2]10)
<514.076839> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]176, [2]10)
<514.076856> V34MP, MP detected, starting MP' txmit<514.076859> V34MP, Starting txmit MP again(33), rxflgs=0x614,txflags=0x2701
<514.078087> V34MP, MP detected, starting MP' txmit<514.078093> V34MP, Starting txmit MP again(34), rxflgs=0x614,txflags=0x2701
<514.097010> V34MP -MP sequence,30f9,bffe,0,0,0,0,0,0,0,d35d
<514.097020> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]140, [2]10)
<514.097063> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]176, [2]10)
<514.097143> V34MP, MP detected, starting MP' txmit<514.097147> V34MP, Starting txmit MP again(35), rxflgs=0x614,txflags=0x2701
<514.117033> V34MP, MP detected, starting MP' txmit<514.117044> V34MP, Starting txmit MP again(36), rxflgs=0x614,txflags=0x2701
<514.117268> V34MP -MP sequence,30f9,bffe,0,0,0,0,0,0,0,d35d
<514.117274> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]140, [2]10)
<514.117354> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]176, [2]10)
<514.117402> V34MP, MP detected, starting MP' txmit<514.117406> V34MP, Starting txmit MP again(37), rxflgs=0x614,txflags=0x2701
<514.138014> V34EQU, equerr = 130, preerr = 0,
<514.138115> V34MP, MP detected, starting MP' txmit<514.138118> V34MP, Starting txmit MP again(38), rxflgs=0x614,txflags=0x2701
<514.256853> V34MP -MP sequence,30f9,bffe,0,0,0,0,0,0,0,d35d
<514.256872> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]140, [2]10)
<514.256916> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]176, [2]10)
<514.256931> V34MP, MP detected, starting MP' txmit<514.256934> V34MP, Starting txmit MP again(39), rxflgs=0x614,txflags=0x2701
<514.256999> TimingV34: Timing Offset [ppm] = -4
<514.259237> V34MP, MP detected, starting MP' txmit<514.259249> V34MP, Starting txmit MP again(40), rxflgs=0x614,txflags=0x2701
<514.259488> V34MP -MP sequence,30f9,bffe,0,0,0,0,0,0,0,d35d
<514.259493> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]140, [2]10)
<514.259592> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]172, [2]10)
<514.259637> V34MP, MP detected, starting MP' txmit<514.259641> V34MP, Starting txmit MP again(41), rxflgs=0x614,txflags=0x2701
<514.262210> V34MP, MP detected, starting MP' txmit<514.262224> V34MP, Starting txmit MP again(42), rxflgs=0x614,txflags=0x2701
<514.262534> V34MP -MP sequence,30f9,bffe,0,0,0,0,0,0,0,d35d
<514.262542> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]136, [2]10)
<514.265070> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]176, [2]10)
<514.265258> V34MP, MP detected, starting MP' txmit<514.265264> V34MP, Starting txmit MP again(43), rxflgs=0x614,txflags=0x2701
<514.265460> V34MP, MP detected, starting MP' txmit<514.265462> V34MP, Starting txmit MP again(44), rxflgs=0x614,txflags=0x2701
<514.267722> V34MP -MP sequence,30f9,bffe,0,0,0,0,0,0,0,d35d
<514.267737> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]140, [2]10)
<514.267876> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]176, [2]10)
<514.267923> V34MP, MP detected, starting MP' txmit<514.267926> V34MP, Starting txmit MP again(45), rxflgs=0x614,txflags=0x2701
<514.270509> V34MP, MP detected, starting MP' txmit<514.270516> V34MP, Starting txmit MP again(46), rxflgs=0x614,txflags=0x2701
<514.270661> V34MP -MP sequence,30f9,bffe,0,0,0,0,0,0,0,d35d
<514.270665> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]140, [2]10)
<514.270706> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]176, [2]10)
<514.270721> V34MP, MP detected, starting MP' txmit<514.270723> V34MP, Starting txmit MP again(47), rxflgs=0x614,txflags=0x2701
<514.277637> V34MP, MP detected, starting MP' txmit<514.277645> V34MP, Starting txmit MP again(48), rxflgs=0x614,txflags=0x2701
<514.278505> TimingV34: Timing Offset [ppm] = -13
<514.297560> V34MP -MP sequence,30f9,bffe,0,0,0,0,0,0,0,d35d
<514.297570> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]140, [2]10)
<514.297674> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]176, [2]10)
<514.297720> V34MP, MP detected, starting MP' txmit<514.297725> V34MP, Starting txmit MP again(49), rxflgs=0x614,txflags=0x2701
<514.298241> V34MP, MP detected, starting MP' txmit<514.298245> V34MP, Starting txmit MP again(50), rxflgs=0x614,txflags=0x2701
<514.317837> V34MP -MP sequence,30f9,bffe,0,0,0,0,0,0,0,d35d
<514.317850> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]140, [2]10)
<514.317889> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]172, [2]10)
<514.318034> V34MP, MP detected, starting MP' txmit<514.318039> V34MP, Starting txmit MP again(51), rxflgs=0x614,txflags=0x2701
<514.337050> V34MP, MP detected, starting MP' txmit<514.337066> V34MP, Starting txmit MP again(52), rxflgs=0x614,txflags=0x2701
<514.337296> V34MP -MP sequence,30f9,bffe,0,0,0,0,0,0,0,d35d
<514.337301> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]136, [2]10)
<514.337342> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]176, [2]10)
<514.337391> V34MP, MP detected, starting MP' txmit<514.337395> V34MP, Starting txmit MP again(53), rxflgs=0x614,txflags=0x2701
<514.357232> V34MP, MP detected, starting MP' txmit<514.357241> V34MP, Starting txmit MP again(54), rxflgs=0x614,txflags=0x2701
<514.377269> V34MP -MP sequence,30f9,bffe,0,0,0,0,0,0,0,d35d
<514.377282> V34HSHAKE: microstate DET_INFO=>DET_SYNC(tx XMITMP, rx RECEIVE, [1]140, [2]10)
<514.377322> V34HSHAKE: microstate DET_SYNC=>DET_INFO(tx XMITMP, rx RECEIVE, [1]176, [2]10)
<514.377334> V34MP, MP detected, starting MP' txmit<514.377336> V34MP, Starting txmit MP again(55), rxflgs=0x614,txflags=0x2701
<514.397397> V34MP, MP detected, starting MP' txmit<514.397413> V34MP, Starting txmit MP again(56), rxflgs=0x614,txflags=0x2701
<514.397647> V34MP -MP sequence,30f9,bffe,0,0,0,0,0,0,0,d35d