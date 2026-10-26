# Tools

One line per tool in `tools/` and `scripts/`, taken from its own header
(docstring or top comment); run a tool with `--help` for its options. Python
tools that need numpy/scipy run from the repo virtualenv (`./.venv/bin/python`,
see `requirements-tools.txt`). Most are investigation tools from one topic: the
note for that topic in `docs/` (indexed in `docs/README.md`) says how its results
were used. Regenerate the table after adding a tool rather than letting it drift.

## Subdirectories

| Directory | Contents |
|---|---|
| `asterisk/` | Asterisk RFC 4040 CLEARMODE codec module (its own README) |
| `eicon/` | Eicon BRI test answerer, binary stream test and HTTP server run on eicon420 (`docs/eicon_soak_ppp.md`) |
| `h245/` | H.245 ASN.1 extraction and PER code generator for V.75/V.70 (`make h245-schema`, `make h245-golden`) |
| `k56flex_upstream_rig/` | Offline K56flex upstream equalizer/ISI analysis |
| `linux-capture/` | Running the vendor HSF driver under Linux for a usbmon capture (its own README) |
| `mica_overlay_probe/` | Driving the emulated Cisco MICA overlays (its own README) |
| `soak/` | Live-call and soak scripts for the tower/vmhost rigs (`rig/README.md`) |
| `v32bis_datapump/`, `v32bis_ref/` | Python V.32bis reference model and datapump tests (`make v32bis-ref-test`, `make v32bis-datapump-test`) |
| `v56bis/` | V.56bis reference filter data |
| `dumps/` | Small retained bit dumps used by the Ja tools |

## Make targets outside `make test`

`make test-fast`, `make test-slow` and `make test` run `tests/*.list`; the
readme and the topic notes describe the matrix targets (`pcm-matrix`,
`v56-sweep`, `v34-tone-matrix`, ...). These targets are not covered elsewhere:

| Target | What it does |
|---|---|
| `test-build` | Build `sip_v90_modem` and every test binary without running anything |
| `g711-path-test` | Live check that the SIP path carries G.711 byte-exact (`g711_path_exactness.py`), then that a law-pinned endpoint refuses the other law (488). Needs a PBX with an Answer()+Echo() extension; `G711_TEST_ARGS` overrides the account |
| `sip-loop-test` | Two `sip_v90_modem` instances over real SIP on 127.0.0.1: ringing, caller ID, S0/ATA answering, an unanswered dial. Binds UDP 5070/5080 and RTP 41000/42000 |
| `sip-clearmode-test` | Two of our endpoints negotiate RFC 4040 CLEARMODE (`docs/clear_channel_v120.md`) |
| `v32bis-ref-test`, `v32bis-datapump-test` | Python V.32bis reference model and datapump unit tests |
| `v34-hdx-high-rate-test` | Diagnostic matrix of V.34 half-duplex at 3200/3429 baud, 26400-33600, both laws; reports every failure |
| `gui-spectrum-test`, `gui-fft-test`, `gui-audio-test` | Offline checks of the native GUI's spectrum plot, display FFT and audio clock (macOS) |
| `h245-schema`, `h245-golden` | Regenerate the H.245 PER code and golden vectors for V.70 (`docs/v70_dsvd.md`; needs `pdftotext` and `asn1tools`) |

## Test runner and unit tests

| Tool | What it does |
|---|---|
| `tools/apple_cid_test.py` | Independent on-hook FSK fixtures through the actual Apple CID receive path. |
| `tools/apple_clock_reference_test.py` | Hardware-free ANSam reference ownership and explicit ppm override checks. |
| `tools/apple_v34_stream_test.py` | Hardware-free V.34 socket seam checks, including fragmented sample bytes. |
| `tools/eicon_dil_termination_test.py` | Replay a foreign DIL terminator; no claim of live V.90 interoperation. |
| `tools/eicon_soak_test.py` | Timed download, upload, simultaneous binary tests, or PPP over an Eicon call. |
| `tools/eicon_zmodem_test.py` | Test both menu ZMODEM directions over a dedicated Eicon modem call. |
| `tools/mt5634mu_diag_test.py` | Hardware-free checks of fragmented pager/result handling and call cleanup. |
| `tools/multi_line_test.py` | Multi-line server test: `sip_v90_modem --lines N` over local SIP. |
| `tools/reverse_test.py` | Reverse-direction test: set Conexant to auto-answer, then dial 6311 from the v90modem PTY and log both sides with timestamps. |
| `tools/run_tests.py` | Run the test lists in parallel and report every failure, not just the first. |
| `tools/sip_clearmode_test.py` | Two sip_v90_modem instances over SIP on 127.0.0.1 in V.120: the SDP must offer and select RFC 4040 CLEARMODE/8000 when ME_CLEARMODE=1, PCMU otherwise, and ... |
| `tools/sip_loop_test.py` | Two sip_v90_modem instances calling each other over SIP on 127.0.0.1. |
| `tools/test_cx_hold_dial.py` | Exercise failed-dial cleanup without opening hardware or waiting 90 s. |
| `tools/test_k56flex_probe_timeout.py` | Engine-level timer regression, not K56flex interop evidence. |
| `tools/test_rtp_path_check.py` | Negative controls for byte transparency and RTCP loss attribution. |
| `tools/test_summarize_modem_trace.py` | Unit tests for `summarize_modem_trace.py`. |
| `tools/test_v32bis_compare_spandsp.py` | Tests for the local SpanDSP V.32bis comparison harness. |
| `tools/test_v32bis_rasfinder_acquisition.py` | V.32bis 5.2.1/6.1: recover S and R1 from three foreign-modem calls. |
| `tools/test_v32bis_spec_policy.py` | Focused tests for the V.32bis startup-state policy helpers. |
| `tools/test_v32bis_tcm.py` | Unit tests for v32bis_tcm.py. |
| `tools/test_v32bis_wav_harness.py` | Smoke tests for the V.32bis WAV interop harness. |
| `tools/test_v90_early_loss.py` | V.90 9.5.1.1/9.6: detect sustained loss even before baseline learning. |
| `tools/test_vpcm_record_recover.py` | Framing checks for the recording recovery tool, independent of DSP fits. |
| `tools/v91_serial_pair_test.py` | Launch two sip_v90_modem instances and verify bidirectional PTY payload. |

## Hardware peers and rigs

| Tool | What it does |
|---|---|
| `tools/apple_9667_report.py` | Compare a native-rate 9667 capture with the PBX's actual sweep WAV. |
| `tools/apple_9668_report.py` | Recognize repeated 9668 playback; raw correlation is not a decoder BER. |
| `tools/apple_dil_quality.py` | Supervised, band-limited channel-fit diagnostic for captured 9668 playback. |
| `tools/apple_line_characterize.py` | Offline noise and response diagnostics for retained Si3056 §5.30 probes. |
| `tools/apple_loopback_spectrum.py` | FFT/coherence and held-out channel models for saved Apple loopback probes. |
| `tools/apple_modem_dtmf_decode.py` | Decode DTMF from a raw s16 capture made by apple_usb_modem_audio. |
| `tools/apple_modem_line_check.sh` | One-command health check for an Apple USB Modem's analogue line: go off-hook, transmit a tone into the dial tone, and measure how much of it comes straight ... |
| `tools/apple_modem_pair_test.sh` | Two Apple USB Modems (05ac:1401) into a conference bridge, and a one-way tone test across it. |
| `tools/apple_modem_tone_report.py` | Report on a tools/apple_modem_pair_test.sh output directory. |
| `tools/apple_native_rate_probe.py` | Bounded standalone Si3056 §5.30 native-rate probes, no engine resampling. |
| `tools/apple_native_rate_report.py` | Compare training-only FIR responses of native-rate Si3056 §5.30 probes. |
| `tools/apple_pcm_loopback_report.py` | Measure G.711 levels in captured Apple hardware loopbacks (G.711 §3). |
| `tools/apple_pcm_matrix.py` | Native Apple USB modem bench: exhaustive PCM excitation and RX modelling. |
| `tools/apple_pcm_pr4.c` | Offline, known-reference PR4 limit measurement for apple_pcm_matrix.py. |
| `tools/apple_snr_report.py` | Separate quiet noise, active noise and harmonic distortion in 1 kHz probes. |
| `tools/apple_snr_sweep.py` | Controlled local analogue-loop tone/gain sweep; Si3056 §§5.29–5.30. |
| `tools/apple_usb_modem_probe.c` | apple_usb_modem_probe -- USB-side probe for the Apple USB Modem (A1082), USB 05ac:1401, a Motorola SM56 softmodem. |
| `tools/apple_v34_loopback.py` | V.34 §§10.1/11 training and PRBS payload through one Apple hardware leg. |
| `tools/apple_v90_sd_recovery_verify.py` | Grade the preserved Apple r7 analogue Phase 3, independently by stage. |
| `tools/cx_at.py` | Minimal AT driver for the Conexant CX93010 USB modem. |
| `tools/cx_hold_dial.py` | Dial from an AT modem (the CX93001), hold the call and send numbered lines. |
| `tools/dmodem_at.py` | Drive the remote d-modem/slmodemd rig's AT interface over SSH+socat. |
| `tools/eicon_call_batch.py` | Bounded Eicon modem calls with exact echo checks and per-call evidence. |
| `tools/eicon_compare_received_audio.py` | Compare Eicon Audio1 paired PCMU samples with a local tap at a verified offset. |
| `tools/eicon_ppp_direct.py` | PPP straight from CONNECT on an Eicon endpoint that answers with LCP. |
| `tools/eicon_rtp_pcap_check.py` | Read Ethernet or raw IPv4 RTP from classic pcap; preserve payloads and packet gaps. |
| `tools/eicon_rx_conformance.py` | Receive-path conformance against a downstream we did not generate. |
| `tools/hsf_capture.sh` | Capture the Conexant HSF USB modem's wire traffic, one action per file. |
| `tools/hsf_extract_rom.py` | Extract ROM_IMAGE from Conexant's c2firmware.h into a flat binary. |
| `tools/hsf_find_result.py` | Locate the script interpreter's result byte in the controller's internal RAM. |
| `tools/hsf_firmware.py` | Take apart the HSF USB modem's controller firmware (hsf_rom_image.bin). |
| `tools/hsf_load_patched.py` | Load a patched firmware image, insisting the device is in BOOTLOADER state. |
| `tools/hsf_opcode_sweep.py` | Sweep the HSF script opcode space looking for a register-read primitive. |
| `tools/hsf_patch_fw.py` | Patch the HSF controller firmware to run our own 8051 code on the device. |
| `tools/hsf_pcap.py` | Decode a usbmon *pcap* capture of the Conexant HSF USB modem (0572:1300). |
| `tools/hsf_probe_response.py` | Measure the HSF coupler's receive channel response from a recorded call. |
| `tools/hsf_pty_traffic.py` | Pass numbered lines both ways across two modem PTYs and grade the result. |
| `tools/hsf_reg_bit_sweep.sh` | Sweep the streaming control registers one bit at a time, during a live stream, watching whether transmit starts draining. |
| `tools/hsf_regdump.py` | Read the HSF controller's internal control register file from the host. |
| `tools/hsf_rx_tone.py` | Is the HSF RX stream real line audio, and does our TX reach the line? |
| `tools/hsf_scripts.py` | Extract the CD2_CONTROL_SCRIPT templates from hsfusbcd2-i386.O. |
| `tools/hsf_seqdiff.py` | Read two usbmon pcaps side by side as SEQUENCES OF SCRIPT IDS. |
| `tools/hsf_tx_gate_sweep.sh` | Sweep the hsfusbcd2261_ path codes (and the script-8/12 ordering) looking for the one that makes bulk OUT drain instead of filling once and stopping. |
| `tools/hsf_tx_reaches_line.sh` | Does our DTMF reach the LINE? |
| `tools/hsf_usbmon.py` | Decode a usbmon capture of the Conexant HSF USB modem (0572:1300). |
| `tools/hsf_v34_call.sh` | One HSF-analogue call into the digital side, with a DTE traffic grader on both PTYs. |
| `tools/mnp_frames.py` | List the MNP (V.42 Annex A) frames in a DS_TX_BIT_DUMP / DS_RX_BIT_DUMP. |
| `tools/mt5634mu_diag.py` | One bounded MT5634MU call through its bridge; stdlib only, no reset/NVRAM. |
| `tools/pty_dial.py` | Dial a number via the sip_v90_modem PTY (AT interface) and log the result. |
| `tools/slm_local_pair.py` | One call between slmodemd (SmartLink soft modem) and audio_sock_modem, locally. |
| `tools/slm_slm_pair.py` | One call between TWO slmodemd instances, as a calibration for slm_local_pair.py. |
| `tools/usr_voice_at.py` | Voice-mode (V.253 FCLASS=8) AT driver for the USR 56K FAX EXT on /dev/cu.usbserial-21240, used by tools/voice_pcm_fidelity.py. |

## SIP, RTP and G.711 bearer

| Tool | What it does |
|---|---|
| `tools/desplice_call.py` | Undo the clock-recovery splices a recorded call injected into its own tap. |
| `tools/desplice_matrix.sh` | Replay every recorded call twice -- as taped, and with its own clock-recovery splices undone -- and report clean TIME and the longest unbroken hold. |
| `tools/echo_lag_track.py` | Track the bulk delay of our own echo through a call, from the DS0 taps. |
| `tools/g711_codec.py` | Standalone G.711 A-law/mu-law codec (numpy, table-based). |
| `tools/g711_path_exactness.py` | G.711 byte-exactness probe for the Asterisk path. |
| `tools/g711_transition_compare.py` | Compare raw PCMU tap power/spectra and held-out TX echo across a transition. |
| `tools/measure_timing_slips.py` | Census the whole-sample timing slips in a recorded G.711 tap. |
| `tools/rtp_jitter_scan.py` | Scan an rtp-rx.csv/rtp-tx.csv trace for loss, timestamp discontinuities and arrival jitter, optionally reporting the neighbourhood of given instants. |
| `tools/rtp_path_check.py` | Check both legs of a G.711 RTP relay from one Ethernet/raw-IP pcap. |

## V.90, V.92 and V.PCM

| Tool | What it does |
|---|---|
| `tools/auto_ja_search.py` | Auto-search Ja DBPSK candidates. |
| `tools/auto_table15_search.py` | Auto-search V.92 Table 15 (INFO0d) in a G.711 u-law capture. |
| `tools/auto_table16_search.py` | Auto-search Table 16 (INFO0a) frames in a G.711 u-law capture by: 1) Finding the guard/data region where 1800 Hz and 2400 Hz are both present. |
| `tools/auto_table17_search.py` | Auto-search V.92 Table 17 (INFO1d) in a G.711 u-law capture. |
| `tools/check_ja_dil_consistency.py` | Check a Ja descriptor against the observed DIL window. |
| `tools/concede_ab_summary.py` | Score a tools/soak/v90_concede_ab.sh directory. |
| `tools/decode_ja_bits.py` | Decode Ja/TRN bitstreams around an anchor. |
| `tools/detect_trn1u_ja.py` | Locate TRN1u and Ja in a capture. |
| `tools/explore_v90_ja_demod.py` | Explore V.90 Ja demodulation quality from anchored I/Q symbols. |
| `tools/info_state_records.py` | Decode the INFO overlay's state records and its state-vector table. |
| `tools/ja_concede_cost.py` | What conceding on Ja actually costs, measured on the recorded corpus. |
| `tools/ja_deadline_cost.py` | Price a mid-attempt deadline trigger for the Ja concession. |
| `tools/pcm_ber_regression.py` | Check PCM measurement failure detection and callback-boundary invariance. |
| `tools/pcm_sweep.py` | Retain directed V.90 downstream/V.92 PCM upstream datapump measurements. |
| `tools/recover_ja_descriptor.py` | Recover a Ja DIL descriptor by block-period consensus. |
| `tools/refine_ja_lock.py` | Refine Ja lock using framing/CRC nearness metrics. |
| `tools/search_v90_ja.py` | V.90 upstream symbol-rate/carrier set (§6.2/V.90). |
| `tools/smartlink_cp_symbol_wrap.c` | Link-time diagnostic wrapper for SmartLink's closed-source V.90 CP symbol reader. |
| `tools/smartlink_jd_probe.c` | Probe slmodemd's Jd handling (SmartLink wrapper). |
| `tools/smartlink_linear_mapping_wrap.c` | Read-only link-time diagnostic hook for SmartLink's analogue-side V.90 linear-mapping study. |
| `tools/smartlink_scrambler_probe.c` | Offline oracle for SmartLink's byte-at-a-time GPC scrambler. |
| `tools/smartlink_shaper_probe.c` | Offline oracle for SmartLink's V.90 spectral shaper. |
| `tools/smartlink_trn2d_reference_wrap.c` | Read-only link-time diagnostic hook for SmartLink's analogue-side V.90 Phase 4 receiver. |
| `tools/trn1d_ab_summary.py` | Score a tools/soak/v90_trn1d_ab.sh directory. |
| `tools/trn1d_pool.py` | Pool TRN1d first-attempt results across runs, grouped by the TRN1d length the log actually reports rather than by directory name. |
| `tools/trn2d_call_batch.sh` | Batch V.90 capture calls against the tower rig; stop on first TRN2REF. |
| `tools/trn2d_pairs_analyze.py` | Analyze SmartLink TRN2d [received, reference] int16 pair capture. |
| `tools/v90_cp_frame.py` | --------------------------------------------------------------------------- A standalone Table-14 CP/CPt framer, for reading a recovered Phase 4 bit stream ... |
| `tools/v90_cp_live_probe.c` | Offline probe for the strict live V.90 CP fallback. |
| `tools/v90_dil_from_audio.py` | Turn an audio clip into a V.90 DIL training-Ucode ladder. |
| `tools/v90_downstream_segment.py` | Reference segmentation of a V.90 digital-side downstream DS0 capture. |
| `tools/v90_hardware_interop.py` | Run one bounded real-modem V.90 attempt and preserve reproducible evidence. |
| `tools/v90_jd_decode.py` | Decode the Jd frame from a V.90 downstream DS0 capture. |
| `tools/v90_long_soak_summary.py` | Score a tools/soak/v90_long_soak.sh directory. |
| `tools/v90_pad_gain.py` | Replicate the analogue modem's pad-gain search against our downstream PCM. |
| `tools/v90_phase2_timeline.py` | Label a V.90 Phase 2 capture: what was on the line, and what stage we were in. |
| `tools/v90_phase3_segment.py` | Segment a V.90 downstream capture into Phase 3 stages. |
| `tools/v90_phase4_capture_check.py` | Locate CRC-valid V.90 CP frames and measure TX echo on recorded DS0 taps. |
| `tools/v90_phase4_upstream_grade.py` | Grade the peer's upstream Phase 4 signal (CPt / SCR / CP) OUTSIDE our receiver. |
| `tools/v90_rd_verify.py` | Check OUR OWN transmitted Rd against V.90 §9.6.1.1, out of a live TX tap. |
| `tools/v90_trn2d_spec_decode.py` | Decode OUR OWN V.90 Phase 4 downstream (TRN2d then MP) off a transmit tap, with a receiver written from the Recommendation text alone. |
| `tools/v90_trn2d_spec_generate.py` | Regenerate V.90 TRN2d from the Recommendation text alone and compare it, sign by sign, with what our transmitter actually put on the wire. |
| `tools/v90_upstream_bench.sh` | Score the V.90 upstream receiver over a directory of recorded calls. |
| `tools/v90_upstream_score.py` | Clean time and longest hold from a v90_upstream_replay log. |
| `tools/v92_fit_line_channel.py` | Fit the V.92 upstream loop channel from a real call, for v92_line_channel. |
| `tools/v92_p3_probe.c` | v92_p3_probe - replay a recorded digital-side G.711 receive tap through the V.92 strict Phase-3 receiver (v92_p3_rx.c) and print every state change. |
| `tools/v92_p3_rx_smoketest.c` | Fallback for non-seekable streams: discard bytes manually. |
| `tools/v92_trn1u_bound.py` | How well could ANY linear receiver recover V.92 TRN1u from this recording? |
| `tools/v92_trn2u_replay.c` | Replay a raw G.711 capture through the native V.92 TRN2u/CPu receiver. |
| `tools/validate_ja_sequence.py` | Validate Ja bitstream structure. |
| `tools/visualize_dil_waveform.py` | Show a Ja descriptor and its synthesized DIL waveform in the terminal. |
| `tools/vpcm_cp_bits_probe.c` | Decode CP frames from an ASCII bit file with the `vpcm_cp` codec. |
| `tools/vpcm_record_probe.c` | Offline analogue recording recovery helper. |
| `tools/vpcm_record_recover.py` | Experimental CP-constrained recovery from an 8 kHz analogue recording. |

## V.34 and V.8

| Tool | What it does |
|---|---|
| `scripts/v34_tone_matrix.sh` | Score a directory of tone recordings for INFO0a, INFO1a and Phase 3 recovery (`make v34-tone-matrix`). |
| `tools/check_info_tables.py` | Check decoded bitstreams against INFO field layouts used by this repo. |
| `tools/dpsk_info_probe.py` | Probe likely INFO sequences in a G.711 u-law capture using a simple 2400 Hz DBPSK demodulator. |
| `tools/plot_iq_constellation.py` | Render an I/Q dump as an SVG constellation. |
| `tools/rf_ansam_latency.py` | How long the far end holds ANSam before answering — its CM-detection latency. |
| `tools/rf_phase4_summary.py` | Score a RasFinder TRN2d sweep on what the peer does inside Phase 4. |
| `tools/v34_channel_bound.py` | How good could ANY linear receiver be on this recording? |
| `tools/v34_encoder_oracle.py` | Independent V.34 raw-symbol oracle for 3200/3429 baud, expanded, 16-state. |
| `tools/v34_hdx_training_phase.py` | Measure a V.34 PP-preceded burst's S/S-bar and following symbols against its PP. |
| `tools/v34_mp_offline.py` | Decide the V.34 Phase 4 MP interpretation offline, against the frame CRC. |
| `tools/v34_mp_tap_check.py` | Independently decode caller 4-point V.34 MP from a G.711 tap. |
| `tools/v34_mp_timeline.py` | List every V.34 Phase 4 MP / MP' frame on a G.711 tap, with its time. |
| `tools/v34_phase2_timeline.py` | Segment a captured G.711 stream into V.34 Phase 2 events. |
| `tools/v34_phase3_demod.py` | V.90 / V.34 / V.92 Phase 3 upstream demodulator (analogue modem TX). |
| `tools/v34_phase3_verify.py` | Check a captured Phase 3 transmission against V.34 10.1.3.6/10.1.3.7 itself. |
| `tools/v34_primary_waveform_decode.py` | Independent 3429/4800 V.34 HDX PCMU waveform -> ECM page decoder. |
| `tools/v34_rate_sweep.py` | Measure forced V.34 rates against training estimates and actual PRBS errors. |
| `tools/v34_trn_ground_truth.py` | Score a V.34 receiver's Phase 4 TRN decode against the transmitter's own symbols. |
| `tools/v56_sweep.py` | Run reproducible synthetic-line modem BER measurements; retain every failure. |
| `tools/v56bis_filters.py` | Design and independently measure V.56bis Table A.10/A.11 channel FIRs. |
| `tools/v8_loop_test.c` | Build: cc -O1 -Ispandsp-master/src $(pkg-config --cflags libtiff-4) tools/v8_loop_test.c spandsp-master/src/.libs/libspandsp.a $(pkg-config --libs ... |
| `tools/v8_timeline_audit.py` | V.8 timeline audit of a call's two G.711 taps against V.8 (11/2000) clause 8. |

## V.32bis

| Tool | What it does |
|---|---|
| `tools/v32bis_channel_sweep.py` | Sweep combined V.32bis startup channel impairments and report recovery. |
| `tools/v32bis_compare_spandsp.py` | Compare the local Python V.32bis reference against the bundled SpanDSP tree. |
| `tools/v32bis_datapump_startup.py` | Run the runtime-oriented V.32bis datapump startup skeleton. |
| `tools/v32bis_ref_report.py` | Run and report a V.32bis logical startup scenario. |
| `tools/v32bis_rx_roundtrip.py` | Run a clean TX->RX round trip for V.32bis startup symbols. |
| `tools/v32bis_tcm.py` | V.32bis Trellis-Coded Modulation — TX encoder and RX Viterbi decoder. |
| `tools/v32bis_tx_baseband.py` | Emit a V.32bis startup trace as pulse-shaped complex baseband samples. |
| `tools/v32bis_tx_passband.py` | Emit a V.32bis startup trace as real passband samples. |
| `tools/v32bis_tx_symbols.py` | Emit a V.32bis startup trace as exact complex symbols. |
| `tools/v32bis_wav_harness.py` | Generate and analyze simple V.32bis WAV fixtures. |

## K56flex, x2, V.FC, MICA and firmware

| Tool | What it does |
|---|---|
| `tools/test_x2_host_replay.py` | Grade known foreign payload through the complete x2 engine on a saved RX tap. |
| `tools/analyze_x2_courier_output.py` | Locate Courier 403 DSP-to-supervisor loss in an 832a PC capture. |
| `tools/generate_x2_short_banks.py` | Generate x2 short-record banks and frame vectors from original I-modem execution. |
| `tools/generate_x2_sym_vectors.py` | Export original I-modem symmetric source/history captures, no program words. |
| `tools/generate_x2_vectors.py` | Run original QF060003 instructions to generate portable x2 regression vectors. |
| `tools/probe_x2_client_imodem.py` | Our x2 analogue client dials the original Ie030002 I-modem as an x2 server. |
| `tools/probe_x2_host_courier.py` | Fresh x2 host feedback against the original analog Courier emulator. |
| `tools/probe_x2_symmetric_imodem.py` | Clock an original Courier endpoint against the actual v90modem engine. |
| `tools/generate_k56flex_pcm_fixtures.py` | Generate research PCM fixtures, NOT hardware recordings. |
| `tools/generate_k56flex_probe_vectors.py` | Generate k56flex_probe_tables.h and k56flex_probe_vectors.h. |
| `tools/generate_k56flex_upstream_vectors.py` | Retain upstream point signs from MicaEmu's independently checked transmitter. |
| `tools/generate_k56flex_vectors.py` | Generate k56flex_tables.h and k56flex_firmware_vectors.h. |
| `tools/k56flex_hsf_pair.py` | Bounded whole-engine K56flex call against cruft's native legacy HSF client. |
| `tools/k56flex_hsf_probe_check.py` | Compare captured server PCMU with the recovered legacy HSF TRN1 references. |
| `tools/k56flex_hsf_sip.py` | The native legacy HSF K56flex client on a real SIP call. |
| `tools/k56flex_record_probe.c` | Offline probe references and joint block decisions for the record-recovery harness. |
| `tools/k56flex_record_recover.py` | Recover repeated K56flex parameter records from an analogue 8 kHz WAV. |
| `tools/k56flex_upstream_lattice.py` | Measure MICA's K56flex upstream trellis input against the transmitted points. |
| `tools/mica_qam_adapt_oracle.py` | Compare production FIR coefficient sweeps with original MICA 6910/6B54. |
| `tools/mica_qam_block_oracle.py` | Compare production block cadence with original MICA 6C50. |
| `tools/mica_qam_capture_copy_oracle.py` | Verify original 1A8C ring-to-capture copying and cursor direction. |
| `tools/mica_qam_coordinate_oracle.py` | Compare production feedback slicer with original MICA coordinates fixture. |
| `tools/mica_qam_extended_bits_oracle.py` | Verify original BE68 alternate word-history producer. |
| `tools/mica_qam_fir_oracle.py` | Compare the production complex rotor with original MICA 6910 instructions. |
| `tools/mica_qam_gate_oracle.py` | Verify MICA K56flex first-gate control flow and B31D word selection. |
| `tools/mica_qam_init_oracle.py` | Compare production timing initialization with original MICA 533F. |
| `tools/mica_qam_input_setup_oracle.py` | Execute original 6BF4 input-FIR setup and inspect its expanded descriptor. |
| `tools/mica_qam_phase_oracle.py` | Compare production phase bookkeeping with original MICA 542D. |
| `tools/mica_qam_predictor_adapt_oracle.py` | Verify original DAB7 6CF8 adaptation setup, sweeps and updated forward output. |
| `tools/mica_qam_predictor_angle_oracle.py` | Original 6D4E correlation angle, including 6831/6890, versus production C. |
| `tools/mica_qam_predictor_block_oracle.py` | Verify joined three-pair 6C50 processing against the production block. |
| `tools/mica_qam_predictor_control_oracle.py` | Complete original 4AF3 mode-15 predictor controller versus production C. |
| `tools/mica_qam_predictor_correlation_oracle.py` | Original mode-15 4B59..4B86 complex correlation versus production C. |
| `tools/mica_qam_predictor_fir_oracle.py` | Verify original DAB7 setup and forward FIR addressing/scaling. |
| `tools/mica_qam_predictor_normal_oracle.py` | Complete original 4AF3 non-15 predictor controller versus production C. |
| `tools/mica_qam_predictor_phase_oracle.py` | Original 4BB1 predictor phase/phasor update versus production C. |
| `tools/mica_qam_predictor_run_oracle.py` | Verify consecutive 6C50 calls and multi-block cadence against production C. |
| `tools/mica_qam_resampler_oracle.py` | Compare production resampling with original MICA 51CE/4580/5219. |
| `tools/mica_qam_rotation_oracle.py` | Compare the production complex rotor with original MICA 5A54 instructions. |
| `tools/mica_qam_source_emit_oracle.py` | Verify original 9320 carrier presets and full 4A6C phase-cycle source writes. |
| `tools/mica_qam_source_pair_oracle.py` | Verify original 4A6C writes complex source pairs into the DAB7 ring. |
| `tools/mica_qam_startup_bits_oracle.py` | Verify original 8F6F/8F74 startup word-history producers. |
| `tools/mica_qam_startup_count_oracle.py` | Verify original 8274..827E startup sample-count prefix at SPM=0. |
| `tools/mica_qam_startup_fir_oracle.py` | Verify original 829E..82A3 FIR instruction sequence at SPM=0/OVM=0. |
| `tools/mica_qam_startup_output_oracle.py` | Verify original 9367 startup output writer at SPM=0. |
| `tools/mica_qam_startup_phase_oracle.py` | Verify original 828B..8299 phase and coefficient-bank selection. |
| `tools/mica_qam_startup_pipeline_oracle.py` | Verify original 90FF extraction, startup mapper and 4A6C together at SPM=0. |
| `tools/mica_qam_startup_rotate_oracle.py` | Verify overlay 88 original 43D1 startup carrier rotation. |
| `tools/mica_qam_startup_rotate_output_oracle.py` | Verify original 43D1 rotation joined to 9367 output writer. |
| `tools/mica_qam_startup_samples_oracle.py` | Verify whole original 8270 through a permuted DM bank-pointer table at SPM=0. |
| `tools/mica_qam_startup_source_oracle.py` | Verify startup nibble mapping joined to original 4A6C at SPM=0. |
| `tools/mica_qam_startup_symbol_oracle.py` | Verify original 97A2/97A8 -> 97C3 startup symbol mapping at SPM=0. |
| `tools/mica_qam_startup_take_oracle.py` | Verify bounded original 90FF symbol extraction with all three producers. |
| `tools/mica_qam_timing_oracle.py` | Compare production timing controller with original MICA 45B8/460E/535B. |
| `tools/mica_qam_v32bis_tx_oracle.py` | Verify mica_v32bis_tx against the original module setup and symbol path. |
| `tools/mica_qam_word_rx_oracle.py` | Recover hard-decision response acquisition using original MICA instructions. |
| `tools/mips_dis.py` | Disassemble a file range of the Eicon MIPS protocol image (te_dmlt.pm). |
| `tools/pe_info.py` | Minimal PE/COFF reader for the Windows driver work. |
| `tools/probe_vfc_firmware.py` | Try a V.FC call between two original analogue Courier ROMs. |
| `tools/vfc_courier_capture.py` | Read-only native V.FC signature capture, launched by probe_vfc_firmware.py. |
| `tools/vfc_firmware_vectors.py` | Extract V.FC mapper tables and execute original Courier mapper test vectors. |
| `tools/vfc_frame_vectors.py` | Execute Courier c1bf frame planning and the c88a schedule before bit packing. |
| `tools/vfc_signature_vectors.py` | Execute original V.FC signature TX/RX bit routines for independent fixtures. |
| `tools/x2_compare_upstream_symbols.py` | Compare native Courier mapper points with our waveform receiver output. |
| `tools/x2_courier_capture.py` | Native mapper/DAC capture for probe_x2_host_courier.py. |

## GUI

| Tool | What it does |
|---|---|
| `tools/gui_apple_test.py` | Hardware-free Apple GUI check: on-hook idle, AT control and fresh sessions. |
| `tools/gui_ata_test.py` | Local SIP registrar/auth/listener lifecycle check; no physical ATA required. |
| `tools/gui_audio_clock_test.py` | Verify native audio sample-clock accounting with an offline audio engine. |
| `tools/gui_fft_test.py` | Compare the native display FFT against its original direct DFT. |
| `tools/gui_loopback_test.py` | Exercise the self-contained GUI echo pair and profile changes over SIP. |
| `tools/gui_smoke_test.py` | Real localhost SIP GUI check (requires permission to bind local sockets). |
| `tools/gui_spectrum_test.py` | Check fresh-sample histories, averaging, phase timing and native layout. |
| `tools/modem_gui.py` | Local live modem console. |

## Other

| Tool | What it does |
|---|---|
| `scripts/softmodem_debug.sh` | Build and launch sip_v90_modem for softmodem debugging, with preflight checks (see readme "Softmodem Debug Mode"). |
| `tools/analyze_dmodem_pcm_path.py` | Compare a live G.711 TX tap with the PCM consumed by d-modem. |
| `tools/build_modem_wasm.py` | Build the browser engine from vendored sources, without touching native objects. |
| `tools/cc_demod.py` | Demodulate a V.34 control channel (10.2.4) out of a recorded G.711 tap. |
| `tools/constellation.py` | Constellation and eye diagnostics for a V.34 / V.90 call, as an HTML page. |
| `tools/d8051.py` | 8051 disassembler, complete enough for the HSF script interpreter. |
| `tools/env_knobs_inventory.py` | Regenerate docs/env_knobs.md: every environment variable the code reads. |
| `tools/eye_summary.py` | One line of eye measurements per call, for a rate/time matrix. |
| `tools/fax_v8_probe.sh` | V.34 fax interop, stage 1: what does a real fax machine offer in its V.8 CM? |
| `tools/gen_pcm_test_signal.py` | Generate a known reference PCM test signal for the voice-mode fidelity harness (tools/voice_pcm_fidelity.py): a slow 300-3000 Hz sweep plus a fixed 1 kHz ... |
| `tools/hdlc_bit_check.py` | Check complete HDLC frames in a DS_TX_BIT_DUMP ASCII bit stream. |
| `tools/inject_sample_slips.py` | Inject clock-recovery sample slips into a recorded G.711 tap. |
| `tools/modem_showcase.py` | Record real offline engine handshakes as 8 kHz stereo listening WAVs. |
| `tools/noci_ab_summary.py` | Score an ME_V8_NO_CI A/B: per call, what V.8 did at each end. |
| `tools/package_modem_demo.py` | Package the browser modem as one offline HTML file, including its WASM. |
| `tools/peer_phase4_report.py` | Read the SmartLink peer's own log for the two numbers that decide a V.90 call. |
| `tools/replay_dte_check.py` | Assert the actual verbose DTE CONNECT result during an engine replay. |
| `tools/slip_ab_summary.py` | Score a v90_slip_ab.sh run: clean time, longest hold, payload, splices. |
| `tools/summarize_modem_trace.py` | Summarize live sip_v90_modem verbose logs into a compact modem timeline. |
| `tools/v42_stats_summary.py` | Summarise V42_STATS output from a server.log. |
| `tools/v42bis_interop.py` | Cross-check SpanDSP against modem-dsp-emu's independent V.42bis codec. |
| `tools/v44_interop.py` | Check the C V.44 port against modem-dsp-emu's independent Python codec. |
| `tools/v91_spec_decode.py` | Decode OUR OWN V.91 transmit taps with a receiver written from the text. |
| `tools/voice_pcm_fidelity.py` | Voice-mode PCM fidelity harness. |
| `tools/wav_to_g711.c` | wav_to_g711.c — Split a stereo 8kHz 16-bit PCM WAV into L/R channels and encode each to raw G.711 A-law or μ-law using SpanDSP. |
