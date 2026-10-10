CC = gcc

UNAME_S := $(shell uname -s)
UNAME_M := $(shell uname -m)
HOST_TAG := $(UNAME_S)-$(UNAME_M)
BREW_PREFIX := $(shell command -v brew >/dev/null 2>&1 && brew --prefix 2>/dev/null)
HOMEBREW_PREFIX := $(if $(BREW_PREFIX),$(BREW_PREFIX),/opt/homebrew)
AUTO_ARCH_SUFFIX := $(patsubst $(HOMEBREW_PREFIX)/lib/libpj-%.a,%,$(firstword $(wildcard $(HOMEBREW_PREFIX)/lib/libpj-*.a)))
USE_LOCAL_PJPROJECT ?= 1
PJ_LOCAL_ROOT ?= pjproject
PJ_LOCAL_MAKEFILE := $(PJ_LOCAL_ROOT)/Makefile
PJ_HOST_STAMP := $(PJ_LOCAL_ROOT)/.build-host
PJ_CONFIG_SITE := pj_config_site.h
PJ_LOCAL_CONFIG_SITE := $(PJ_LOCAL_ROOT)/pjlib/include/pj/config_site.h

# Local SpanDSP 3.0.0 build (with V.34 support)
SPANDSP_ROOT = spandsp-master
SPANDSP_DIR  = $(SPANDSP_ROOT)/src
SPANDSP_LIB  = $(SPANDSP_DIR)/.libs/libspandsp.a
SPANDSP_MAKE = $(SPANDSP_ROOT)/Makefile
SPANDSP_HOST_STAMP := $(SPANDSP_ROOT)/.build-host

# Shared defaults
PJ_CFLAGS   ?=
PJ_LIBS     ?=
TIFF_LDFLAGS := $(shell pkg-config --libs libtiff-4 2>/dev/null || echo "-L$(HOMEBREW_PREFIX)/lib -ltiff")
JPEG_CFLAGS := $(shell pkg-config --cflags libjpeg 2>/dev/null || echo "-I$(HOMEBREW_PREFIX)/opt/jpeg-turbo/include")
JPEG_LDFLAGS := $(shell pkg-config --libs libjpeg 2>/dev/null || echo "-L$(HOMEBREW_PREFIX)/opt/jpeg-turbo/lib -ljpeg")
# Homebrew keeps OpenSSL keg-only, so it is not on the default library path
# and a bare -lssl fails to link ("ld: library 'ssl' not found").  Ask
# pkg-config, then fall back to the keg.
OPENSSL_LDFLAGS := $(shell pkg-config --libs openssl 2>/dev/null || echo "-L$(HOMEBREW_PREFIX)/opt/openssl@3/lib -lssl -lcrypto")
# Linux's libuuid is separate from libc (macOS has it in libSystem, where
# pkg-config finds nothing and this stays empty); pjlib needs it.
UUID_LDFLAGS := $(shell pkg-config --libs uuid 2>/dev/null)
SYSTEM_LIBS ?= $(TIFF_LDFLAGS) $(JPEG_LDFLAGS) $(OPENSSL_LDFLAGS) $(UUID_LDFLAGS) -lm -lpthread
PJ_BUILD_PREREQ ?=

ifneq ($(and $(filter 1,$(USE_LOCAL_PJPROJECT)),$(wildcard $(PJ_LOCAL_MAKEFILE))),)
  PJ_BUILD_PREREQ := pjproject
  PJ_LOCAL_PJSUA  = $(firstword $(wildcard $(PJ_LOCAL_ROOT)/pjsip/lib/libpjsua-*.a))
  PJ_LOCAL_SUFFIX = $(patsubst $(PJ_LOCAL_ROOT)/pjsip/lib/libpjsua-%.a,%,$(PJ_LOCAL_PJSUA))
  PJ_CFLAGS += -I$(PJ_LOCAL_ROOT)/pjlib/include \
               -I$(PJ_LOCAL_ROOT)/pjlib-util/include \
               -I$(PJ_LOCAL_ROOT)/pjnath/include \
               -I$(PJ_LOCAL_ROOT)/pjmedia/include \
               -I$(PJ_LOCAL_ROOT)/pjsip/include
  PJ_LIBS += -L$(PJ_LOCAL_ROOT)/pjlib/lib \
             -L$(PJ_LOCAL_ROOT)/pjlib-util/lib \
             -L$(PJ_LOCAL_ROOT)/pjnath/lib \
             -L$(PJ_LOCAL_ROOT)/pjmedia/lib \
             -L$(PJ_LOCAL_ROOT)/pjsip/lib \
             -L$(PJ_LOCAL_ROOT)/third_party/lib \
             -lpjsua-$(PJ_LOCAL_SUFFIX) \
             -lpjsip-ua-$(PJ_LOCAL_SUFFIX) \
             -lpjsip-simple-$(PJ_LOCAL_SUFFIX) \
             -lpjsip-$(PJ_LOCAL_SUFFIX) \
             -lpjmedia-codec-$(PJ_LOCAL_SUFFIX) \
             -lpjmedia-videodev-$(PJ_LOCAL_SUFFIX) \
             -lpjmedia-audiodev-$(PJ_LOCAL_SUFFIX) \
             -lpjmedia-$(PJ_LOCAL_SUFFIX) \
             -lpjnath-$(PJ_LOCAL_SUFFIX) \
             -lpjlib-util-$(PJ_LOCAL_SUFFIX) \
             -lsrtp-$(PJ_LOCAL_SUFFIX) \
             -lresample-$(PJ_LOCAL_SUFFIX) \
             -lgsmcodec-$(PJ_LOCAL_SUFFIX) \
             -lspeex-$(PJ_LOCAL_SUFFIX) \
             -lilbccodec-$(PJ_LOCAL_SUFFIX) \
             -lg7221codec-$(PJ_LOCAL_SUFFIX) \
             -lyuv-$(PJ_LOCAL_SUFFIX) \
             -lwebrtc-$(PJ_LOCAL_SUFFIX) \
             -lpj-$(PJ_LOCAL_SUFFIX)
  ifeq ($(UNAME_S),Darwin)
    SYSTEM_LIBS += -framework CoreAudio \
                   -framework CoreServices \
                   -framework AudioUnit \
                   -framework AudioToolbox \
                   -framework Foundation \
                   -framework AppKit \
                   -framework AVFoundation \
                   -framework CoreGraphics \
                   -framework CoreVideo \
                   -framework CoreMedia
  else ifeq ($(UNAME_S),Linux)
    SYSTEM_LIBS += -lutil -lasound -lavformat -lavcodec -lswscale -lavutil -lv4l2 -lstdc++ -lopus
  endif
else
  ifeq ($(UNAME_S),Darwin)
    # pjproject 2.16 on Apple Silicon macOS
    PJPROJ_DIR  ?= $(HOMEBREW_PREFIX)/Cellar/pjproject/2.16
    ARCH_SUFFIX ?= $(if $(AUTO_ARCH_SUFFIX),$(AUTO_ARCH_SUFFIX),aarch64-apple-darwin24.6.0)

    PJ_CFLAGS += -I$(PJPROJ_DIR)/include -I$(HOMEBREW_PREFIX)/include
    PJ_LIBS   += -L$(PJPROJ_DIR)/lib \
                 -lpjsua-$(ARCH_SUFFIX) \
                 -lpjsip-ua-$(ARCH_SUFFIX) \
                 -lpjsip-simple-$(ARCH_SUFFIX) \
                 -lpjsip-$(ARCH_SUFFIX) \
                 -lpjmedia-codec-$(ARCH_SUFFIX) \
                 -lpjmedia-audiodev-$(ARCH_SUFFIX) \
                 -lpjmedia-$(ARCH_SUFFIX) \
                 -lpjnath-$(ARCH_SUFFIX) \
                 -lpjlib-util-$(ARCH_SUFFIX) \
                 -lpj-$(ARCH_SUFFIX) \
                 -lsrtp-$(ARCH_SUFFIX) \
                 -lresample-$(ARCH_SUFFIX) \
                 -lgsmcodec-$(ARCH_SUFFIX) \
                 -lspeex-$(ARCH_SUFFIX) \
                 -lilbccodec-$(ARCH_SUFFIX) \
                 -lg7221codec-$(ARCH_SUFFIX) \
                 -lwebrtc-$(ARCH_SUFFIX)
    SYSTEM_LIBS += -L$(HOMEBREW_PREFIX)/lib \
                   -framework CoreAudio \
                   -framework CoreServices \
                   -framework AudioUnit \
                   -framework AudioToolbox \
                   -framework Foundation \
                   -framework AppKit \
                   -framework AVFoundation \
                   -framework CoreGraphics \
                   -framework CoreVideo \
                   -framework CoreMedia
  else ifeq ($(UNAME_S),Linux)
    PJ_CFLAGS += $(shell pkg-config --cflags libpjproject 2>/dev/null)
    PJ_LIBS   += $(shell pkg-config --libs libpjproject 2>/dev/null)
    ifeq ($(strip $(PJ_LIBS)),)
      PJ_LIBS += -lpjsua -lpjsip-ua -lpjsip-simple -lpjsip \
                 -lpjmedia-codec -lpjmedia-audiodev -lpjmedia \
                 -lpjnath -lpjlib-util -lpj -lsrtp -lresample \
                 -lgsmcodec -lspeex -lilbccodec -lg7221codec -lwebrtc
    endif
    SYSTEM_LIBS += -lutil
  endif
endif

TIFF_CFLAGS := $(shell pkg-config --cflags libtiff-4 2>/dev/null || echo "-I$(HOMEBREW_PREFIX)/include")

CFLAGS = -Wall -Wextra -O2 -g \
         -MMD -MP \
         -I. -I$(SPANDSP_DIR) -I$(SPANDSP_DIR)/.. -Iport \
         $(PJ_CFLAGS) $(TIFF_CFLAGS) \
         -DPJ_AUTOCONF=1 -DPJ_IS_BIG_ENDIAN=0 -DPJ_IS_LITTLE_ENDIAN=1

# To build against a non-default system pjproject on macOS, update ARCH_SUFFIX, e.g.:
#   ARCH_SUFFIX = arm64-apple-darwin23.0.0
# Or run: ls $(HOMEBREW_PREFIX)/lib/libpj-*.a | sed 's/.*libpj-//' | sed 's/\.a//'

LDFLAGS = $(PJ_LIBS) $(SPANDSP_LIB) $(SYSTEM_LIBS)

# libusb, for the Conexant HSF USB modem line interface (hsf_fxo.c).  Only the
# hsf_fxo_probe target needs it; the SIP modem does not link against it.
LIBUSB_CFLAGS := $(shell pkg-config --cflags libusb-1.0 2>/dev/null || echo "-I$(HOMEBREW_PREFIX)/include/libusb-1.0")
LIBUSB_LIBS   := $(shell pkg-config --libs libusb-1.0 2>/dev/null || echo "-L$(HOMEBREW_PREFIX)/lib -lusb-1.0")

SRCS   = v8bis_modem.c v8bis_fsm.c v8bis_ie.c v8bis_msg.c v8bis_tones.c profile_file.c line_monitor.c at_help.c v250_ctl.c at_test.c legacy_pcm_decode.c k56flex_client.c k56flex_rxfe.c at_ms.c clear_channel.c v25_automode.c k56flex_train.c k56flex_probe.c k56flex_v8bis.c k56flex.c x2.c x2_sym.c v34_line_ec.c v92_mh.c v92_mh_line.c v92_rn.c v92_rsig.c v92_tone_a.c v92_analogue_audio.c v92_analogue_phase4.c v92_analogue_phase3.c v92_su.c sip_modem.c modem_engine.c v90_analogue_linear.c v90_analogue_fse.c v90_analogue_sd.c v34_pp_fit.c v90_sounder.c clock_recovery.c data_interface.c fax_class2.c data_stack.c v44.c v90.c v90_cp_rx.c v90_cp_live.c v90_analogue_tx.c v90_analogue_rx.c v90_analogue_phase3.c v90_analogue_phase4.c v90_dil_measure.c v90_dil_presets.c p3_demod.c v91.c vpcm_cp.c vpcm_g711_stream.c vpcm_call.c vpcm_call_pair.c vpcm_link.c vpcm_v91_session.c v92_phase3_decode.c v92_phase3_ru.c v92_ja_decode.c v92_p3_rx.c v92_p3_eq.c v92_phase4_decode.c v92_cp_rx.c v92_trn2u.c v92_upstream_data.c v92_upstream_rx.c x2_session.c x2_mp_rx.c
OBJS   = $(SRCS:.c=.o)
TARGET = sip_v90_modem
TEST_TARGETS = line_monitor_test v92_b1u_lock_test v250_ctl_test at_test_test pcm_ber_test v56_loopback_test legacy_pcm_decode_test at_ms_test console_test clear_channel_test k56flex_client_test k56flex_train_test k56flex_probe_test k56flex_v8bis_test k56flex_test mica_qam_test x2_test v42bis_test v44_test v92_startup_test port_cp_stream_test port_data_rx_test port_v34_fixed_test port_v34_fixed_lms_test port_v34_fixed_solve_test vpcm_loopback_test vpcm_decode vpcm_encode v92_trn2u_replay data_stack_test v42_link_test v42_throughput_test v34_phase2_decode_test v34_mp_test v34_info1a_validate_test v34_phase4_16pt_test v34_data_test v34_gardner_test fax_class_test fax_class2_test fax_v34_preamble_test v90_upstream_replay v90_engine_replay v34_duplex_test v34_fdm_test v32bis_spandsp_test v32bis_duplex_test v32bis_engine_pair_test engine_pair_test v90_engine_peer v92_proc_eval_test v90_analogue_tx_test v90_analogue_rx_test v90_reneg_silence_test v90_analogue_sd_test v34_pp_fit_test v34_hdx_test v92_p3_rx_line_test v92_mh_test v92_mh_line_test v92_mh_retrain_test v92_rn_test v92_rsig_test v92_tone_a_test x2_session_test x2_sym_test x2_b1_test v8bis_test v8bis_fsm_test v8bis_modem_test v76_test v75_test v70_test v70_v34_test
TEST_OBJS = v92_su.o vpcm_loopback_test.o v90.o v90_cp_rx.o v90_dil_rx.o v90_dil_measure.o v90_dil_presets.o v90_analogue_tx.o v90_analogue_rx.o v90_analogue_phase3.o v90_analogue_phase4.o v91.o vpcm_cp.o vpcm_g711_stream.o vpcm_call.o vpcm_call_pair.o vpcm_link.o vpcm_v90_session.o vpcm_v91_session.o vpcm_v91_loopback.o v92_phase3_decode.o v92_phase3_ru.o v92_phase4_decode.o v92_ja_decode.o v92_p3_rx.o v92_p3_eq.o v92_cp_rx.o v92_trn2u.o v92_upstream_data.o v92_upstream_rx.o p3_demod.o
DECODE_OBJS = legacy_pcm_decode.o x2_session.o x2_sym.o x2_mp_rx.o x2.o k56flex_client.o k56flex_rxfe.o k56flex_train.o k56flex_probe.o k56flex.o vpcm_decode.o v90_dil_measure.o v90_dil_presets.o v34_phase2_decode.o v34_info_decode.o v8bis_decode.o v92_short_phase1_decode.o v92_short_phase2_decode.o v92_phase3_decode.o v92_phase3_ru.o v92_phase4_decode.o v92_ja_decode.o v92_p3_rx.o v92_p3_eq.o v92_anspcm_decode.o p3_demod.o v90.o v90_cp_rx.o v91.o vpcm_cp.o v21_fsk_demod.o phase12_decode.o call_init_tone_probe.o v90_dil_rx.o
ENCODE_OBJS = vpcm_encode.o v90.o v91.o vpcm_cp.o v92_phase4_decode.o v90_dil_measure.o v90_dil_presets.o
V92_STARTUP_TEST_OBJS = v92_line_channel.o v90_analogue_fse.o v90_analogue_sd.o v92_analogue_audio.o v92_analogue_phase4.o v92_upstream_data.o v92_upstream_rx.o v92_analogue_phase3.o v92_su.o v90_analogue_rx.o v90_analogue_linear.o v90_analogue_phase4.o p3_demod.o v92_startup_test.o v92_trn2u.o v92_cp_rx.o v92_p3_rx.o v92_p3_eq.o v92_ja_decode.o v90.o v90_cp_rx.o v90_dil_measure.o v90_dil_presets.o v91.o vpcm_cp.o v92_phase4_decode.o
V92_REPLAY_OBJS = tools/v92_trn2u_replay.o v92_trn2u.o v92_cp_rx.o vpcm_cp.o
DATA_STACK_TEST_OBJS = data_stack_test.o data_stack.o v44.o
V44_TEST_OBJS = v44_test.o v44.o
V42_LINK_TEST_OBJS = v42_link_test.o
V42_THROUGHPUT_TEST_OBJS = v42_throughput_test.o
# V.70 DSVD: V.76 multiplex, V.75 control entity, V.70 terminal.  Not in SRCS: nothing
# in the engine links them yet (docs/v70_dsvd.md says what is missing).
V70_STACK_OBJS = v76.o per.o h245_schema.o v75_h245.o v75.o v70.o
V8BIS_TEST_OBJS = v8bis_test.o v8bis_tones.o v8bis_msg.o v8bis_ie.o
V8BIS_FSM_TEST_OBJS = v8bis_fsm_test.o v8bis_fsm.o v8bis_tones.o v8bis_msg.o v8bis_ie.o
V8BIS_MODEM_TEST_OBJS = v8bis_modem_test.o v8bis_modem.o v8bis_fsm.o v8bis_tones.o v8bis_msg.o v8bis_ie.o
V76_TEST_OBJS = v76_test.o v76.o
V75_TEST_OBJS = v75_test.o v75.o v75_h245.o per.o h245_schema.o v76.o
V70_TEST_OBJS = v70_test.o v70.o v75.o v75_h245.o per.o h245_schema.o v76.o
V70_V34_TEST_OBJS = v70_v34_test.o v70.o v75.o v75_h245.o per.o h245_schema.o v76.o
V34_PHASE2_DECODE_TEST_OBJS = v34_phase2_decode_test.o v34_phase2_decode.o
V34_MP_TEST_OBJS = v34_mp_test.o
V34_DATA_TEST_OBJS = v34_data_test.o
V34_GARDNER_TEST_OBJS = v34_gardner_test.o
FAX_CLASS_TEST_OBJS = fax_class_test.o data_interface.o fax_class2.o at_ms.o at_test.o v250_ctl.o at_help.o line_monitor.o profile_file.o
V250_CTL_TEST_OBJS = v250_ctl_test.o v250_ctl.o
LINE_MONITOR_TEST_OBJS = line_monitor_test.o line_monitor.o
# The two DTE console arrangements (classic combined, control + data).
CONSOLE_TEST_OBJS = console_test.o data_interface.o fax_class2.o at_ms.o at_test.o v250_ctl.o at_help.o line_monitor.o profile_file.o
# AT+MS through the real engine and PTY, plus at_ms.c on its own.
AT_TEST_TEST_OBJS = at_test_test.o data_interface.o fax_class2.o at_ms.o at_test.o v250_ctl.o at_help.o line_monitor.o profile_file.o
AT_MS_TEST_OBJS = at_ms_test.o $(filter-out sip_modem.o,$(OBJS))
# Clear channel / V.120: the module back to back, and the engine looped on
# itself through the PTY.
CLEAR_CHANNEL_TEST_OBJS = clear_channel_test.o $(filter-out sip_modem.o,$(OBJS))
FAX_CLASS2_TEST_OBJS = fax_class2_test.o fax_class2.o
V90_UPSTREAM_REPLAY_OBJS = v90_upstream_replay.o
HSF_FXO_PROBE_OBJS = hsf_fxo_probe.o hsf_fxo.o
APPLE_USB_MODEM_PROBE_OBJS = tools/apple_usb_modem_probe.o
HSF_V90_COUPLER_OBJS = hsf_v90_coupler.o hsf_fxo.o $(filter-out sip_modem.o,$(OBJS))
# ESP32 port, layer 2: the streamed V.90 CP decode standing alone, with the
# Table 14 framer it feeds.  No V.34 receiver.
PORT_CP_STREAM_TEST_OBJS = port/cp_stream_test.o port/cp_stream.o v90_cp_rx.o vpcm_cp.o
# ESP32 port, layer 3: the V.90 upstream DATA feed-forward decode core.
PORT_DATA_RX_TEST_OBJS = port/data_rx_test.o port/data_rx.o
# Fixed-point datapath.  `make fixed` builds it, `make float` (or plain `make`)
# goes back.  Not a default: on an ESP32-S3 the FPU makes float both faster and
# smaller, and this is for the FPU-less parts (ESP32-C3/C6, RV32IMC).
#
# The two modes CANNOT share objects -- V34_FIXED_POINT changes the layout of
# v34_rx_state_t -- and this makefile has no header dependencies, so a stale
# object linked across a mode switch gives impossible values at runtime rather
# than a link error.  BUILD_MODE_STAMP records which mode the tree is in and
# the targets clean when it changes; do not defeat it.
BUILD_MODE_STAMP = .build-mode
FIXED_CPPFLAGS = -DV34_FIXED_POINT -I$(CURDIR)/port

# ESP32 port: fixed-point kernels, checked against float on real probe data.
PORT_V34_FIXED_TEST_OBJS = port/v34_fixed_test.o
PORT_V34_FIXED_LMS_TEST_OBJS = port/v34_fixed_lms_test.o
PORT_V34_FIXED_SOLVE_TEST_OBJS = port/v34_fixed_solve_test.o
# The whole engine with the SIP front end swapped for a file reader, so a
# recorded call can be run through V.8 and Phases 2-4 exactly as the media
# thread runs it.  Everything $(TARGET) links except sip_modem.o, which is
# the part being replaced.
V90_ENGINE_REPLAY_OBJS = v90_engine_replay.o $(filter-out sip_modem.o,$(OBJS))
# Not a test: replays a recorded digital-side G.711 receive tap through the
# V.92 strict Phase-3 receiver, which live only ever rehunts and so reports
# nothing when it rejects.  Same objects that receiver needs in the engine.
V92_P3_RX_LINE_TEST_OBJS = v92_p3_rx_line_test.o $(filter-out v92_startup_test.o,$(V92_STARTUP_TEST_OBJS))
V92_P3_PROBE_OBJS = v92_p3_probe.o v92_p3_rx.o v92_p3_eq.o v92_ja_decode.o p3_demod.o v90.o v90_cp_rx.o v90_dil_measure.o v90_dil_presets.o v91.o vpcm_cp.o v92_phase4_decode.o v92_trn2u.o v92_cp_rx.o
V34_DUPLEX_TEST_OBJS = v34_duplex_test.o v34_line_ec.o
V92_B1U_LOCK_TEST_OBJS = v92_b1u_lock_test.o v92_upstream_rx.o v92_upstream_data.o
PCM_BER_TEST_OBJS = pcm_ber_test.o $(filter-out vpcm_loopback_test.o,$(TEST_OBJS))
V56_LOOPBACK_TEST_OBJS = v56_loopback_test.o v34_line_ec.o
V34_HDX_TEST_OBJS = v34_hdx_test.o
V32BIS_SPANDSP_TEST_OBJS = v32bis_spandsp_test.o
V32BIS_DUPLEX_TEST_OBJS = v32bis_duplex_test.o
V34_PP_FIT_TEST_OBJS = v34_pp_fit_test.o v34_pp_fit.o v90_analogue_tx.o v90_analogue_phase4.o v90_dil_measure.o v90.o v90_cp_rx.o v90_dil_presets.o v91.o vpcm_cp.o v92_phase4_decode.o
V90_ANALOGUE_TX_TEST_OBJS = v90_analogue_tx_test.o v90_analogue_tx.o v90_analogue_phase4.o v90_dil_measure.o v90.o v90_cp_rx.o v90_dil_presets.o v91.o vpcm_cp.o v92_phase4_decode.o
# Includes v90.c itself to drive the transmit state machine.
V90_RENEG_SILENCE_TEST_OBJS = v90_reneg_silence_test.o v90_cp_rx.o v90_dil_measure.o v90_dil_presets.o v91.o vpcm_cp.o v92_phase4_decode.o
V90_ANALOGUE_SD_TEST_OBJS = v90_analogue_sd_test.o v90_analogue_sd.o v90_analogue_fse.o
V90_ANALOGUE_RX_TEST_OBJS = v90_analogue_rx_test.o v90_analogue_rx.o v90_analogue_linear.o v90_analogue_fse.o v90_analogue_sd.o v90_sounder.o v90_analogue_phase3.o v90_analogue_phase4.o v90_analogue_tx.o v90_dil_measure.o v90.o v90_cp_rx.o v90_dil_presets.o v91.o vpcm_cp.o v92_phase4_decode.o
# v92_proc_eval_test.c includes phase12_decode.c directly (its evaluator is
# static), so it links phase12_decode.o's dependencies but not the .o itself.
V92_PROC_EVAL_TEST_OBJS = v92_proc_eval_test.o v34_info_decode.o v8bis_decode.o v92_short_phase1_decode.o v92_short_phase2_decode.o v92_anspcm_decode.o v92_cp_rx.o v92_phase4_decode.o v90.o v90_cp_rx.o v91.o vpcm_cp.o v21_fsk_demod.o call_init_tone_probe.o v90_dil_measure.o v90_dil_presets.o

USE_V34_STUBS ?= 0
ifeq ($(USE_V34_STUBS),1)
SRCS += v34_stubs.c
TEST_OBJS += v34_stubs.o
endif

.PHONY: all test test-fast test-slow test-build clean distclean fixed float fixed-compare spandsp pjproject v34-tone-matrix v92-loop-rx-test v34-duplex-test v34-matrix-test v32bis-ref-test v32bis-datapump-test v32bis-test v91-serial-pair-test eicon-rx-test g711-path-test FORCE

all: $(TARGET) $(TEST_TARGETS)

# The regression suite is tests/fast.list (seconds per row) and tests/slow.list
# (real-time engine fax runs, minutes per row), run in parallel by
# tools/run_tests.py with every failure reported.  TEST_ARGS passes options
# through, e.g. TEST_ARGS="-j 4" or TEST_ARGS="--filter v34_duplex".
RUN_TESTS = python3 tools/run_tests.py $(TEST_ARGS)
TEST_LISTS = tests/fast.list tests/slow.list

test-build: $(TEST_TARGETS)

test: test-build
	$(RUN_TESTS) $(TEST_LISTS)

test-fast: test-build
	$(RUN_TESTS) tests/fast.list

test-slow: test-build
	$(RUN_TESTS) tests/slow.list

# Local preserved analogue capture, deliberately outside make test: artifacts/
# are gitignored. Missing evidence is a hard error in the verifier.
V90_LINE_PYTHON ?= python3
.PHONY: v90-apple-line-test
v90-apple-line-test: v90_analogue_rx_test
	$(V90_LINE_PYTHON) tools/apple_v90_sd_recovery_verify.py

v91-serial-pair-test: $(TARGET)
	python3 tools/v91_serial_pair_test.py --binary ./$(TARGET)
	python3 tools/v91_serial_pair_test.py --binary ./$(TARGET) --robbed-phase 2

# Receive-path conformance against a downstream we did not generate.  Encodes a
# known-open defect (docs/eicon_downstream_comparison.md, Finding 4: DIL
# recovery cannot read a real one-pass DIL), so it is deliberately NOT part of
# `test` -- a suite that is red by default stops being read.  --expect-failure
# inverts the exit status: green while the defect stands, loud when the Phase 3
# chain regresses or the defect is fixed.
eicon-rx-test: vpcm_decode
	python3 tools/eicon_rx_conformance.py --binary ./vpcm_decode --expect-failure

# The V.92 Phase 3 upstream receiver against an analogue modem on a real 2-wire
# loop (artifacts/v92-loop-upstream/) and a modelled one.  Part of `test` since
# docs/v92_p3_rx_line_plan.md step 6 made Ja decode; kept as its own target too.
v92-loop-rx-test: v92_p3_rx_line_test
	./v92_p3_rx_line_test

# Measures whether the SIP path delivers G.711 byte-exactly, which CLAUDE.md's
# first constraint requires and which no offline test can check: the RTP payload
# *is* the DS0 stream, so a transcode, an audiohook, or an adaptive jitter
# buffer anywhere between here and the far end silently breaks Phase 3.  Needs a
# live PBX and an Answer()+Echo() extension (see the module docstring), so it is
# deliberately NOT part of `test`.  Second invocation is the counterfactual: a
# law-pinned endpoint must *refuse* the other law rather than transcode it.
# Override G711_TEST_ARGS for a different registrar, extension or account.
G711_TEST_ARGS ?=
g711-path-test:
	python3 tools/g711_path_exactness.py $(G711_TEST_ARGS)
	python3 tools/g711_path_exactness.py --law alaw --expect-488 $(G711_TEST_ARGS)

v32bis-ref-test:
	python3 -m unittest discover -s tools/v32bis_ref -t .
	python3 -m unittest tools/test_v32bis_compare_spandsp.py \
		tools/test_v32bis_spec_policy.py tools/test_v32bis_tcm.py \
		tools/test_v32bis_wav_harness.py

v32bis-datapump-test:
	python3 -m unittest discover -s tools/v32bis_datapump -t .

.PHONY: v32bis-reneg-test
v32bis-reneg-test: v32bis_duplex_test
	./v32bis_duplex_test --reneg-only

v32bis-test: v32bis_spandsp_test v32bis_duplex_test v32bis-ref-test v32bis-datapump-test
	./v32bis_spandsp_test
	./v32bis_duplex_test

# AppKit is optional; ordinary/headless modem builds remain C-only.
.PHONY: gui gui-smoke-test gui-loopback-test gui-audio-test
gui-smoke-test: $(TARGET)
	python3 tools/gui_smoke_test.py

gui-audio-test:
	python3 tools/gui_audio_clock_test.py

gui-loopback-test: $(TARGET)
	python3 tools/gui_loopback_test.py

gui: $(TARGET) modem_gui_native

modem_gui_native: tools/modem_gui_native.swift
	@cache=$$(mktemp -d); trap 'rm -rf "$$cache"' EXIT; swiftc -module-cache-path "$$cache" -O $< -o $@

$(TARGET): $(OBJS) spandsp $(PJ_BUILD_PREREQ)
	$(CC) $(OBJS) -o $@ $(LDFLAGS)

# Needs the Conexant HSF USB modem attached, so it is not in TEST_TARGETS and
# does not run under `make test`.
hsf_fxo_probe: $(HSF_FXO_PROBE_OBJS)
	$(CC) $(HSF_FXO_PROBE_OBJS) -o $@ $(LIBUSB_LIBS) -lpthread -lm

hsf_v90_coupler: $(HSF_V90_COUPLER_OBJS) spandsp $(PJ_BUILD_PREREQ)
	$(CC) $(HSF_V90_COUPLER_OBJS) -o $@ $(LDFLAGS) $(LIBUSB_LIBS)

hsf_fxo.o hsf_fxo_probe.o hsf_v90_coupler.o: CFLAGS += $(LIBUSB_CFLAGS)
# hsf_v90_coupler.c is a two-line wrapper that #includes hsf_fxo_probe.c.
# This makefile has no header dependencies, so without this line an edit to
# the probe silently leaves the coupler linked against the previous build.
hsf_v90_coupler.o: hsf_fxo_probe.c hsf_fxo.h modem_engine.h data_interface.h

# The Apple USB Modem (A1082, USB 05ac:1401), a Motorola SM56 softmodem: a
# candidate analogue side over a real 2-wire line.  Both need the device
# attached, so neither is in TEST_TARGETS nor runs under `make test`.
# See docs/apple_usb_modem_sm56.md.
apple_usb_modem_probe: $(APPLE_USB_MODEM_PROBE_OBJS)
	$(CC) $(APPLE_USB_MODEM_PROBE_OBJS) -o $@ $(LIBUSB_LIBS)

tools/apple_usb_modem_probe.o: CFLAGS += $(LIBUSB_CFLAGS)

# CoreAudio/AVFoundation, not libusb: once the configuration is set, usbaudiod
# owns the codec and it is reached as an ordinary audio device.
apple_usb_modem_audio: tools/apple_usb_modem_audio.m
	$(CC) $(CFLAGS) -fobjc-arc $< -o $@ \
	    -framework AVFoundation -framework AudioToolbox \
	    -framework CoreAudio -framework CoreFoundation -framework Foundation

# The coupler runs the engine over the Apple part's line: libusb for the hook
# and the register file, CoreAudio for the bearer, and the whole engine minus
# sip_modem.o -- the same object set hsf_v90_coupler links.
APPLE_USB_MODEM_COUPLER_OBJS = tools/apple_usb_modem_coupler.o \
    $(filter-out sip_modem.o,$(OBJS))

apple_usb_modem_coupler: $(APPLE_USB_MODEM_COUPLER_OBJS) spandsp $(PJ_BUILD_PREREQ)
	$(CC) $(APPLE_USB_MODEM_COUPLER_OBJS) -o $@ $(LDFLAGS) $(LIBUSB_LIBS) \
	    -framework AVFoundation -framework AudioToolbox \
	    -framework CoreAudio -framework CoreFoundation -framework Foundation

tools/apple_usb_modem_coupler.o: CFLAGS += $(LIBUSB_CFLAGS)
tools/apple_usb_modem_coupler.o: tools/apple_usb_modem_coupler.m \
    modem_engine.h data_interface.h
	$(CC) $(CFLAGS) $(LIBUSB_CFLAGS) -c $< -o $@

vpcm_loopback_test: $(TEST_OBJS) spandsp $(PJ_BUILD_PREREQ)
	$(CC) $(TEST_OBJS) -o $@ $(LDFLAGS)

vpcm_decode: $(DECODE_OBJS) spandsp $(PJ_BUILD_PREREQ)
	$(CC) $(DECODE_OBJS) -o $@ $(LDFLAGS)

vpcm_encode: $(ENCODE_OBJS) spandsp $(PJ_BUILD_PREREQ)
	$(CC) $(ENCODE_OBJS) -o $@ $(LDFLAGS)

v92_startup_test: $(V92_STARTUP_TEST_OBJS) spandsp $(PJ_BUILD_PREREQ)
	$(CC) $(V92_STARTUP_TEST_OBJS) -o $@ $(LDFLAGS)

v92_trn2u_replay: $(V92_REPLAY_OBJS) spandsp $(PJ_BUILD_PREREQ)
	$(CC) $(V92_REPLAY_OBJS) -o $@ $(LDFLAGS)

port_v34_fixed_solve_test: $(PORT_V34_FIXED_SOLVE_TEST_OBJS)
	$(CC) $(PORT_V34_FIXED_SOLVE_TEST_OBJS) -o $@ -lm

port_v34_fixed_lms_test: $(PORT_V34_FIXED_LMS_TEST_OBJS)
	$(CC) $(PORT_V34_FIXED_LMS_TEST_OBJS) -o $@ -lm

port_v34_fixed_test: $(PORT_V34_FIXED_TEST_OBJS)
	$(CC) $(PORT_V34_FIXED_TEST_OBJS) -o $@ -lm

port_data_rx_test: $(PORT_DATA_RX_TEST_OBJS)
	$(CC) $(PORT_DATA_RX_TEST_OBJS) -o $@ -lm

port_cp_stream_test: $(PORT_CP_STREAM_TEST_OBJS) spandsp
	$(CC) $(PORT_CP_STREAM_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

v44_test: $(V44_TEST_OBJS)
	$(CC) $(V44_TEST_OBJS) -o $@

data_stack_test: $(DATA_STACK_TEST_OBJS) spandsp
	$(CC) $(DATA_STACK_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

fax_class_test: $(FAX_CLASS_TEST_OBJS) spandsp
	$(CC) $(FAX_CLASS_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

line_monitor_test: $(LINE_MONITOR_TEST_OBJS) spandsp
	$(CC) $(LINE_MONITOR_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

v250_ctl_test: $(V250_CTL_TEST_OBJS)
	$(CC) $(V250_CTL_TEST_OBJS) -o $@

console_test: $(CONSOLE_TEST_OBJS) spandsp
	$(CC) $(CONSOLE_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

fax_class2_test: $(FAX_CLASS2_TEST_OBJS) spandsp
	$(CC) $(FAX_CLASS2_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

fax_v34_preamble_test: fax_v34_preamble_test.c $(SPANDSP_ROOT)/src/fax.c spandsp
	$(CC) -DHAVE_CONFIG_H -I$(SPANDSP_DIR) -I$(SPANDSP_ROOT) -msse2 -std=gnu99 -ffast-math -w $(TIFF_CFLAGS) $(JPEG_CFLAGS) fax_v34_preamble_test.c -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

v42_link_test: $(V42_LINK_TEST_OBJS) spandsp
	$(CC) $(V42_LINK_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

v42_throughput_test: $(V42_THROUGHPUT_TEST_OBJS) spandsp
	$(CC) $(V42_THROUGHPUT_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

# V.8bis stage 1: tones, framing and the information field codec.  Not in SRCS yet:
# the engine does not use them until stage 3 (docs/v8bis_engine_scope.md).
v8bis_test: $(V8BIS_TEST_OBJS) spandsp
	$(CC) $(V8BIS_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

v8bis_fsm_test: $(V8BIS_FSM_TEST_OBJS)
	$(CC) $(V8BIS_FSM_TEST_OBJS) -o $@ $(SYSTEM_LIBS)

v8bis_modem_test: $(V8BIS_MODEM_TEST_OBJS) spandsp
	$(CC) $(V8BIS_MODEM_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

v76_test: $(V76_TEST_OBJS)
	$(CC) $(V76_TEST_OBJS) -o $@ $(SYSTEM_LIBS)

v75_test: $(V75_TEST_OBJS)
	$(CC) $(V75_TEST_OBJS) -o $@ $(SYSTEM_LIBS)

v70_test: $(V70_TEST_OBJS)
	$(CC) $(V70_TEST_OBJS) -o $@ $(SYSTEM_LIBS)

v70_v34_test: $(V70_V34_TEST_OBJS) spandsp
	$(CC) $(V70_V34_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

# H.245 PER schema and its independent check.  These need pdftotext and
# `pip install asn1tools` (H245_PYTHON), so they are not part of `make test`; the
# golden vectors in v75_h245_golden.h carry the result into it.
H245_PYTHON ?= python3
H245_PDF = ITU Docs/T-REC-H.245-202203-I!!PDF-E.pdf
H245_BUILD ?= /tmp/h245-per

.PHONY: h245-schema h245-golden h245-per-oracle v70-g729a-test
h245-schema:
	mkdir -p $(H245_BUILD)
	$(H245_PYTHON) tools/h245/extract_asn1.py "$(H245_PDF)" $(H245_BUILD)/h245.asn
	$(H245_PYTHON) tools/h245/per_gen.py $(H245_BUILD)/h245.asn h245_schema.c h245_schema.h

h245-golden: h245-schema
	$(H245_PYTHON) tools/h245/golden.py $(H245_BUILD)/h245.asn > v75_h245_golden.h

h245-per-oracle: h245-schema per.o h245_schema.o
	$(CC) -Wall -O2 tools/h245/per_tool.c per.o h245_schema.o -o $(H245_BUILD)/per_tool
	for seed in 1 2 3; do $(H245_PYTHON) tools/h245/per_oracle.py $(H245_BUILD)/h245.asn $(H245_BUILD)/per_tool 4000 $$seed || exit 1; done

# G.729 Annex A over DSVD, bit-exact against the ITU test vectors.  The ITU source is
# copyrighted ("All rights reserved", no open terms) and is NOT in this tree: extract
# `ITU Docs/T-REC-G.729-201206-I!!SOFT-ZST-E.zip` yourself and
#   make v70-g729a-test G729A_SRC=<dir>/Software/G729_Release3/g729AnnexA/c_code
G729A_SRC ?=
G729A_BUILD ?= /tmp/g729a-build
v70-g729a-test: $(V70_STACK_OBJS)
	@test -n "$(G729A_SRC)" || { echo "set G729A_SRC to the extracted g729AnnexA/c_code"; exit 2; }
	mkdir -p $(G729A_BUILD)
	for f in $(G729A_SRC)/*.C; do b=`basename $$f .C`; case $$b in CODER|DECODER) continue;; esac; \
	  $(CC) -w -O2 -D__unix__=1 -x c -c $$f -o $(G729A_BUILD)/$$b.o || exit 1; done
	$(CC) -Wall -Wextra -O2 -D__unix__=1 -I$(G729A_SRC) -c v70_g729a.c -o $(G729A_BUILD)/v70_g729a.o
	$(CC) -Wall -Wextra -O2 -c v70_g729a_test.c -o $(G729A_BUILD)/v70_g729a_test.o
	$(CC) $(G729A_BUILD)/v70_g729a_test.o $(G729A_BUILD)/v70_g729a.o `ls $(G729A_BUILD)/[A-Z]*.o` $(V70_STACK_OBJS) -o $(G729A_BUILD)/v70_g729a_test -lm
	for v in ALGTHM SPEECH PITCH LSP; do $(G729A_BUILD)/v70_g729a_test $(G729A_SRC)/../test_vectors $$v || exit 1; done

v34_phase2_decode_test: $(V34_PHASE2_DECODE_TEST_OBJS)
	$(CC) $(V34_PHASE2_DECODE_TEST_OBJS) -o $@ -lm

# The full V.34 symbol-rate matrix.  Not in `make test`: only the rows listed
# in that target complete today, and docs/v34_spec_gap.md item 5 tracks the
# rest.  This target reports every row rather than stopping at the first
# failure, so it is the one to run when working on the remaining rates.
v34-matrix-test: v34_duplex_test
	@for b in 2400 2743 2800 3000 3200 3429; do \
	  for law in ulaw alaw; do \
	    ./v34_duplex_test $$b 9600 $$law 2>/dev/null | grep "V.34 duplex" || true; \
	  done; \
	done

v34-duplex-test: v34_duplex_test
	./v34_duplex_test 2400 9600 ulaw
	./v34_duplex_test 2400 9600 alaw
	./v34_duplex_test 2743 9600 ulaw
	./v34_duplex_test 2743 9600 alaw
	./v34_duplex_test 2800 9600 ulaw
	./v34_duplex_test 2800 9600 alaw
	./v34_duplex_test 3000 9600 alaw
	./v34_duplex_test 3200 9600 ulaw
	./v34_duplex_test 3200 9600 alaw
	./v34_duplex_test 3429 9600 alaw

v34_duplex_test: $(V34_DUPLEX_TEST_OBJS) spandsp
	$(CC) $(V34_DUPLEX_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

# N unmodified 8 kHz V.34 pairs frequency-multiplexed over one 48/96 kHz
# linear channel (docs/v34_fdm.md).  Exploratory; not part of make test.
v34_fdm_test: v34_fdm_test.o fdm_bank.o spandsp
	$(CC) v34_fdm_test.o fdm_bank.o -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

# Two sip_v90_modem instances over real SIP on 127.0.0.1 (no registrar):
# ringing, caller ID, S0 and ATA answering, a dial nobody answers.  Binds UDP
# ports 5070/5080 and RTP 41000/42000, so it is not part of make test.
.PHONY: sip-clearmode-test
sip-clearmode-test: $(TARGET)
	python3 tools/sip_clearmode_test.py

.PHONY: sip-loop-test
sip-loop-test: $(TARGET)
	python3 tools/sip_loop_test.py

.PHONY: v56-test v56-sweep v56bis-filter-test v56bis-sweep
.PHONY: pcm-loopback-test pcm-data-test pcm-matrix pcm-procedure-test
pcm-loopback-test: pcm-data-test pcm-procedure-test

pcm-data-test: pcm_ber_test
	$(RUN_TESTS) --group pcm-data-test $(TEST_LISTS)

# Diagnostic: retains high-rate V.92 failures rather than declaring them clean.
pcm-matrix: pcm_ber_test
	python3 tools/pcm_sweep.py --all-rates

pcm-procedure-test: vpcm_loopback_test v92_proc_eval_test v92_mh_test v92_mh_line_test v92_mh_retrain_test v92_rn_test v92_rsig_test v92_tone_a_test
	./vpcm_loopback_test --pcm-procedure-tests
	./v92_proc_eval_test
	./v92_mh_test
	./v92_mh_line_test
	./v92_mh_retrain_test
	./v92_rn_test
	./v92_rsig_test
	./v92_tone_a_test

# Diagnostic, not in `make test`: our V.92 upstream receiver against a model
# of slmodemd's transmit chain (EQ_TAPS=63 locks it).
v92_slm_chain_test: v92_slm_chain_test.o v92_p3_eq.o v92_upstream_rx.o v92_upstream_data.o
	$(CC) $^ -o $@ -lm

v92_b1u_lock_test: $(V92_B1U_LOCK_TEST_OBJS)
	$(CC) $(V92_B1U_LOCK_TEST_OBJS) -o $@ -lm

pcm_ber_test: $(PCM_BER_TEST_OBJS) spandsp $(PJ_BUILD_PREREQ)
	$(CC) $(PCM_BER_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(PJ_LIBS) $(SYSTEM_LIBS)

v56_loopback_test: $(V56_LOOPBACK_TEST_OBJS) spandsp
	$(CC) $(V56_LOOPBACK_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

# Fast regression; the long, one-million-bit sweep is explicitly requested.
v56-test: v56_loopback_test
	$(RUN_TESTS) --group v56-test $(TEST_LISTS)

# Same calibrated offline line and strict 511-bit checker across modulation families.
.PHONY: modulation-loopback-test
modulation-loopback-test: v56_loopback_test
	python3 tools/v56_sweep.py --modulation v32bis --snr off,40 --delays 0 --bits 16000 --seconds 30 --require-clean
	python3 tools/v56_sweep.py --modulation v22bis --snr off,40 --delays 0 --bits 16000 --seconds 30 --require-clean

v56-sweep: v56_loopback_test
	python3 tools/v56_sweep.py

v56bis-filter-test:
	python3 tools/v56bis_filters.py --check

# Diagnostic channel matrix; failure rows are retained, not regression gates.
v56bis-sweep: v56_loopback_test v56bis-filter-test
	python3 tools/v56_sweep.py --cases 2400:9600 --snr off --delays 0 --bits 16000 --seconds 30 --channels 1:1,1:2,1:3,5:1,5:2,5:3,6:1,6:2,6:3,7:1,7:2,7:3,8:1,8:2,8:3,9:1,9:2,9:3

.PHONY: v34-hdx-primary-test
# 12.5 resynchronization followed by at least 8000 error-free primary bits.
# All symbol rates and laws; unequal ceilings also check the MPh-to-mapper seam.
# V.34 Table 22: recipient selects a carrier different from the source default.
# Sample-exact delayed G.711 channel with deterministic analogue noise.
.PHONY: v34-hdx-channel-test
v34-hdx-channel-test: v34_hdx_test
	$(RUN_TESTS) --group v34-hdx-channel-test $(TEST_LISTS)

.PHONY: v34-hdx-infoh-test
v34-hdx-infoh-test: v34_hdx_test
	$(RUN_TESTS) --group v34-hdx-infoh-test $(TEST_LISTS)

v34-hdx-primary-test: v34_hdx_test
	$(RUN_TESTS) --group v34-hdx-primary-test $(TEST_LISTS)

# Clause 12.5.3/12.6: primary -> control -> a second primary interval.
.PHONY: v34-hdx-turnaround-test v34-hdx-recovery-test
v34-hdx-turnaround-test: v34_hdx_test
	$(RUN_TESTS) --group v34-hdx-turnaround-test $(TEST_LISTS)

# 12.6.2.3: recipient requests a lower ceiling; both peers exchange MPh.
.PHONY: v34-hdx-parameters-test
v34-hdx-parameters-test: v34_hdx_test
	$(RUN_TESTS) --group v34-hdx-parameters-test $(TEST_LISTS)

# 12.7: one peer initiates; the other detects its tone without a host request.
# 12.8: either peer or both peers may initiate the control retrain.
.PHONY: v34-hdx-retrain-test
v34-hdx-retrain-test: v34_hdx_test
	$(RUN_TESTS) --group v34-hdx-retrain-test $(TEST_LISTS)

# 12.4.3.1: foreign recipient returns to Tone A rather than sending PPh.
.PHONY: v34-hdx-startup-recovery-test
v34-hdx-startup-recovery-test: v34_hdx_test
	$(RUN_TESTS) --group v34-hdx-startup-recovery-test $(TEST_LISTS)

# Four-second loss of one control direction forces the three-second recovery.
v34-hdx-recovery-test: v34_hdx_test
	$(RUN_TESTS) --group v34-hdx-recovery-test $(TEST_LISTS)

# Diagnostic gate for the dense profiles still under development. Every
# failure remains a failure; run the whole matrix before returning status.
.PHONY: v34-hdx-high-rate-test
v34-hdx-high-rate-test: v34_hdx_test
	@failed=0; for baud in 3200 3429; do \
	  for rate in 26400 28800 31200 33600; do \
	    if test $$baud = 3200 && test $$rate = 33600; then continue; fi; \
	    for law in ulaw alaw; do \
	      V34_HDX_PRIMARY=1 ./v34_hdx_test $$baud $$rate $$law 8 || failed=1; \
	    done; \
	  done; \
	done; exit $$failed

v34_hdx_test: $(V34_HDX_TEST_OBJS) spandsp
	$(CC) $(V34_HDX_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

# Includes the production v34rx.c to call its static INFO1a parser, so it is
# built without -DNDEBUG (the asserts are the test) and links only the library.
v34_info1a_validate_test: v34_info1a_validate_test.c $(SPANDSP_ROOT)/src/v34rx.c spandsp
	$(CC) -DHAVE_CONFIG_H -I$(SPANDSP_DIR) -I$(SPANDSP_ROOT) -msse2 -std=gnu99 -ffast-math -w $(JPEG_CFLAGS) v34_info1a_validate_test.c -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

v34_phase4_16pt_test: v34_phase4_16pt_test.c $(SPANDSP_ROOT)/src/v34tx.c spandsp
	$(CC) -DHAVE_CONFIG_H -I$(SPANDSP_DIR) -I$(SPANDSP_ROOT) -msse2 -std=gnu99 -ffast-math -w $(JPEG_CFLAGS) v34_phase4_16pt_test.c -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

v34_mp_test: $(V34_MP_TEST_OBJS) spandsp
	$(CC) $(V34_MP_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

v34_data_test: $(V34_DATA_TEST_OBJS) spandsp
	$(CC) $(V34_DATA_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

# The timing loop is header-only and free of spandsp types, so this one needs
# nothing but libm -- which is the point: it can be run against a synthetic
# signal whose true sampling instant is known.
v34_gardner_test: $(V34_GARDNER_TEST_OBJS)
	$(CC) $(V34_GARDNER_TEST_OBJS) -o $@ -lm

v32bis_spandsp_test: $(V32BIS_SPANDSP_TEST_OBJS) spandsp
	$(CC) $(V32BIS_SPANDSP_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

v32bis_duplex_test: $(V32BIS_DUPLEX_TEST_OBJS) spandsp
	$(CC) $(V32BIS_DUPLEX_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

# Not a test: a tool for replaying a recorded call into the upstream
# receiver, so faults that only show after tens of seconds can be bisected
# without waiting on the rig.
v90_upstream_replay: $(V90_UPSTREAM_REPLAY_OBJS) spandsp
	$(CC) $(V90_UPSTREAM_REPLAY_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

# Two whole engines in two processes, clocked against each other, carrying a
# V.32bis call from V.8/V.25 to DTE data (v32bis_engine_pair_test.c).
V32BIS_ENGINE_PAIR_TEST_OBJS = v32bis_engine_pair_test.o $(filter-out sip_modem.o,$(OBJS))
v32bis_engine_pair_test: $(V32BIS_ENGINE_PAIR_TEST_OBJS) spandsp $(PJ_BUILD_PREREQ)
	$(CC) $(V32BIS_ENGINE_PAIR_TEST_OBJS) -o $@ $(LDFLAGS)

v90_engine_replay: $(V90_ENGINE_REPLAY_OBJS) spandsp $(PJ_BUILD_PREREQ)
	$(CC) $(V90_ENGINE_REPLAY_OBJS) -o $@ $(LDFLAGS)

at_test_test: $(AT_TEST_TEST_OBJS) spandsp
	$(CC) $(AT_TEST_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

at_ms_test: $(AT_MS_TEST_OBJS) spandsp $(PJ_BUILD_PREREQ)
	$(CC) $(AT_MS_TEST_OBJS) -o $@ $(LDFLAGS)

clear_channel_test: $(CLEAR_CHANNEL_TEST_OBJS) spandsp $(PJ_BUILD_PREREQ)
	$(CC) $(CLEAR_CHANNEL_TEST_OBJS) -o $@ $(LDFLAGS)

v92_p3_rx_line_test: $(V92_P3_RX_LINE_TEST_OBJS) spandsp $(PJ_BUILD_PREREQ)
	$(CC) $(V92_P3_RX_LINE_TEST_OBJS) -o $@ $(LDFLAGS)

v92_p3_probe: $(V92_P3_PROBE_OBJS) spandsp
	$(CC) $(V92_P3_PROBE_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

v92_p3_probe.o: tools/v92_p3_probe.c v92_p3_rx.h v92_ja_decode.h v90.h
	$(CC) $(CFLAGS) -c tools/v92_p3_probe.c -o $@

v34_pp_fit_test: $(V34_PP_FIT_TEST_OBJS) spandsp
	$(CC) $(V34_PP_FIT_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

v90_analogue_tx_test: $(V90_ANALOGUE_TX_TEST_OBJS) spandsp
	$(CC) $(V90_ANALOGUE_TX_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

v90_analogue_sd_test: $(V90_ANALOGUE_SD_TEST_OBJS)
	$(CC) $(V90_ANALOGUE_SD_TEST_OBJS) -o $@ -lm

v90_reneg_silence_test: $(V90_RENEG_SILENCE_TEST_OBJS) spandsp
	$(CC) $(V90_RENEG_SILENCE_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

v90_analogue_rx_test: $(V90_ANALOGUE_RX_TEST_OBJS) spandsp
	$(CC) $(V90_ANALOGUE_RX_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

v92_proc_eval_test.o: phase12_decode.c phase12_decode.h

v92_proc_eval_test: $(V92_PROC_EVAL_TEST_OBJS) spandsp
	$(CC) $(V92_PROC_EVAL_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

# V.92 9.10 modem-on-hold codec and transaction controller.
V92_MH_TEST_OBJS = v92_mh_test.o v92_mh.o
v92_mh_test: $(V92_MH_TEST_OBJS) spandsp
	$(CC) $(V92_MH_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)
V92_MH_LINE_TEST_OBJS = v92_mh_line_test.o v92_mh_line.o v92_mh.o
V92_TONE_A_TEST_OBJS = v92_tone_a_test.o v92_tone_a.o v92_trn2u.o v92_cp_rx.o v92_upstream_data.o v92_phase4_decode.o v91.o vpcm_cp.o v90.o v90_cp_rx.o v90_dil_measure.o v90_dil_presets.o
v92_tone_a_test: $(V92_TONE_A_TEST_OBJS) spandsp
	$(CC) $(V92_TONE_A_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)
V92_RSIG_TEST_OBJS = v92_rsig_test.o v92_rsig.o v91.o vpcm_cp.o v90.o v90_cp_rx.o v90_dil_measure.o v90_dil_presets.o v92_phase4_decode.o
v92_rsig_test: $(V92_RSIG_TEST_OBJS) spandsp
	$(CC) $(V92_RSIG_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)
V92_RN_TEST_OBJS = v92_rn_test.o v92_rn.o
v92_rn_test: $(V92_RN_TEST_OBJS)
	$(CC) $(V92_RN_TEST_OBJS) -o $@
V92_MH_RETRAIN_TEST_OBJS = v92_mh_retrain_test.o v92_mh_line.o v92_mh.o
v92_mh_retrain_test: $(V92_MH_RETRAIN_TEST_OBJS) spandsp
	$(CC) $(V92_MH_RETRAIN_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)
v92_mh_line_test: $(V92_MH_LINE_TEST_OBJS) spandsp
	$(CC) $(V92_MH_LINE_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

$(SPANDSP_LIB): FORCE
	@set -e; \
	current_host="$(HOST_TAG)"; \
	previous_host=""; \
	if [ -f "$(SPANDSP_HOST_STAMP)" ]; then \
		previous_host="$$(cat "$(SPANDSP_HOST_STAMP)")"; \
	fi; \
	if [ "$$previous_host" != "$$current_host" ]; then \
		echo "Preparing SpanDSP for $$current_host"; \
		if [ -f "$(SPANDSP_ROOT)/Makefile" ]; then \
			$(MAKE) -C "$(SPANDSP_ROOT)" distclean >/dev/null 2>&1 || true; \
		fi; \
		rm -f "$(SPANDSP_HOST_STAMP)"; \
	fi; \
	if [ -f "$(SPANDSP_ROOT)/config.status" ] && \
	   ! grep -q '^#define SPANDSP_SUPPORT_V32BIS 1' "$(SPANDSP_DIR)/spandsp.h"; then \
		echo "Reconfiguring SpanDSP to enable V.32bis support..."; \
		$(MAKE) -C "$(SPANDSP_ROOT)" distclean >/dev/null 2>&1 || true; \
	fi; \
	if [ ! -f "$(SPANDSP_ROOT)/config.status" ]; then \
		echo "Configuring SpanDSP with V.34 and V.32bis support for $$current_host..."; \
		(cd "$(SPANDSP_ROOT)" && \
		 CFLAGS="$(TIFF_CFLAGS) $(JPEG_CFLAGS)" \
		 LDFLAGS="$(TIFF_LDFLAGS) $(JPEG_LDFLAGS)" \
		 ./configure --enable-v34 --enable-v32bis); \
	fi; \
	$(MAKE) -C "$(SPANDSP_ROOT)"; \
	printf '%s\n' "$$current_host" > "$(SPANDSP_HOST_STAMP)"

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Keep object layouts synchronized with the headers that define them.  This is
# especially important for the V.92 startup controllers, whose private state is
# shared across several translation units and otherwise fails as an apparent
# wire-protocol error after a header-only change.
-include $(wildcard *.d tools/*.d)

sip_modem.o:      sip_modem.c      modem_engine.h data_interface.h
modem_engine.o:   modem_engine.c   v8bis_modem.h v8bis_fsm.h modem_engine.h v25_automode.h v92_mh.h v92_mh_line.h data_stack.h clock_recovery.h v90.h v90_cp_rx.h v91.h v92_p3_rx.h v92_cp_rx.h v92_trn2u.h v92_upstream_rx.h
clock_recovery.o: clock_recovery.c clock_recovery.h
# ATI3/ATI7 name the build.  Rewritten only when git describe changes, so it
# does not force a rebuild of data_interface.o on every make.
.PHONY: FORCE
build_version.h: FORCE
	@v=$$(git describe --always --dirty 2>/dev/null || echo unknown); \
	printf '#define V90MODEM_VERSION "%s"\n' "$$v" > build_version.h.tmp; \
	if cmp -s build_version.h.tmp build_version.h; then rm -f build_version.h.tmp; \
	else mv build_version.h.tmp build_version.h; fi

data_interface.o: data_interface.c data_interface.h modem_engine.h at_ms.h v250_ctl.h at_help.h build_version.h profile_file.h
at_ms.o:          at_ms.c          at_ms.h
clear_channel.o:  clear_channel.c  clear_channel.h
data_stack.o:     data_stack.c     data_stack.h $(SPANDSP_DIR)/spandsp/v42.h
v90.o:            v90.c            v90.h
v91.o:            v91.c            v91.h v90.h vpcm_cp.h
vpcm_cp.o:        vpcm_cp.c        vpcm_cp.h
v90_cp_rx.o:      v90_cp_rx.c      v90_cp_rx.h vpcm_cp.h
v90_dil_rx.o:     v90_dil_rx.c     v90_dil_rx.h v90.h
vpcm_g711_stream.o: vpcm_g711_stream.c vpcm_g711_stream.h v91.h
vpcm_call.o:      vpcm_call.c      vpcm_call.h vpcm_g711_stream.h vpcm_v91_session.h v91.h
vpcm_call_pair.o: vpcm_call_pair.c vpcm_call_pair.h vpcm_call.h vpcm_g711_stream.h v91.h
vpcm_link.o:      vpcm_link.c      vpcm_link.h vpcm_call.h vpcm_g711_stream.h v91.h
vpcm_v90_session.o: vpcm_v90_session.c vpcm_v90_session.h v90.h v91.h vpcm_cp.h
vpcm_v91_session.o: vpcm_v91_session.c vpcm_v91_session.h v91.h
vpcm_v91_loopback.o: vpcm_v91_loopback.c vpcm_v91_loopback.h vpcm_call_pair.h vpcm_v91_session.h v91.h
ifeq ($(USE_V34_STUBS),1)
v34_stubs.o:      v34_stubs.c
endif
v8bis_decode.o:   v8bis_decode.c   v8bis_decode.h
v92_short_phase1_decode.o: v92_short_phase1_decode.c v92_short_phase1_decode.h v91.h
v92_anspcm_decode.o:       v92_anspcm_decode.c       v92_anspcm_decode.h       v91.h
v92_short_phase2_decode.o: v92_short_phase2_decode.c v92_short_phase2_decode.h
v92_phase3_decode.o: v92_phase3_decode.c v92_phase3_decode.h v92_phase3_ru.h
v92_phase3_ru.o: v92_phase3_ru.c v92_phase3_ru.h v92_phase3_decode.h
v92_phase4_decode.o: v92_phase4_decode.c v92_phase4_decode.h
v92_cp_rx.o:      v92_cp_rx.c      v92_cp_rx.h vpcm_cp.h
v92_trn2u.o:      v92_trn2u.c      v92_trn2u.h v92_cp_rx.h
v92_upstream_data.o: v92_upstream_data.c v92_upstream_data.h
v92_upstream_rx.o: v92_upstream_rx.c v92_upstream_rx.h v92_upstream_data.h
tools/v92_trn2u_replay.o: tools/v92_trn2u_replay.c v92_trn2u.h v92_cp_rx.h
v92_ja_decode.o:  v92_ja_decode.c  v92_ja_decode.h v90.h
v92_p3_rx.o:      v92_p3_rx.c      v92_p3_rx.h v92_p3_eq.h v92_ja_decode.h v90.h
v92_p3_eq.o:      v92_p3_eq.c      v92_p3_eq.h
p3_demod.o:       p3_demod.c       p3_demod.h
v34_phase2_decode.o: v34_phase2_decode.c v34_phase2_decode.h v90.h v91.h
v34_phase2_decode_test.o: v34_phase2_decode_test.c v34_phase2_decode.h
v34_mp_test.o: v34_mp_test.c $(SPANDSP_DIR)/spandsp/v34.h
v34_data_test.o: v34_data_test.c $(SPANDSP_DIR)/spandsp/v34.h
v34_gardner_test.o: v34_gardner_test.c $(SPANDSP_DIR)/v34_gardner.h
v90_upstream_replay.o: v90_upstream_replay.c $(SPANDSP_DIR)/spandsp/v34.h
v90_engine_replay.o: v90_engine_replay.c modem_engine.h
v34_duplex_test.o: v34_duplex_test.c $(SPANDSP_DIR)/spandsp/v34.h
v34_fdm_test.o: v34_fdm_test.c fdm_bank.h $(SPANDSP_DIR)/spandsp/v34.h
fdm_bank.o: fdm_bank.c fdm_bank.h
v32bis_duplex_test.o: v32bis_duplex_test.c $(SPANDSP_DIR)/spandsp/v32bis.h spandsp
v32bis_spandsp_test.o: v32bis_spandsp_test.c $(SPANDSP_DIR)/spandsp/v32bis.h spandsp
v34_info_decode.o: v34_info_decode.c v34_info_decode.h v90.h
v21_fsk_demod.o:  v21_fsk_demod.c  v21_fsk_demod.h
phase12_decode.o: phase12_decode.c phase12_decode.h v21_fsk_demod.h v34_info_decode.h v90.h v8bis_decode.h
call_init_tone_probe.o: call_init_tone_probe.c call_init_tone_probe.h
vpcm_decode.o:    vpcm_decode.c    v34_info_decode.h v34_phase2_decode.h v90.h v91.h vpcm_cp.h v8bis_decode.h v92_short_phase1_decode.h v92_short_phase2_decode.h v92_phase3_decode.h v92_ja_decode.h p3_demod.h phase12_decode.h
vpcm_encode.o:    vpcm_encode.c    v90.h v91.h vpcm_cp.h
v90_cp_live.o:    v90_cp_live.c    v90_cp_live.h p3_demod.h
data_stack_test.o: data_stack_test.c data_stack.h
vpcm_loopback_test.o: vpcm_loopback_test.c v91.h vpcm_cp.h vpcm_call.h vpcm_call_pair.h vpcm_link.h vpcm_v90_session.h v92_cp_rx.h v92_trn2u.h v92_upstream_data.h v92_upstream_rx.h

spandsp: $(SPANDSP_LIB)

pjproject:
	@set -e; \
	if ! cmp -s "$(PJ_CONFIG_SITE)" "$(PJ_LOCAL_CONFIG_SITE)"; then \
		cp "$(PJ_CONFIG_SITE)" "$(PJ_LOCAL_CONFIG_SITE)"; \
	fi; \
	current_host="$(HOST_TAG)"; \
	previous_host=""; \
	if [ -f "$(PJ_HOST_STAMP)" ]; then \
		previous_host="$$(cat "$(PJ_HOST_STAMP)")"; \
	fi; \
	if [ "$$previous_host" != "$$current_host" ]; then \
		echo "Preparing local pjproject for $$current_host"; \
		if [ -f "$(PJ_LOCAL_ROOT)/build.mak" ]; then \
			$(MAKE) -C "$(PJ_LOCAL_ROOT)" distclean >/dev/null 2>&1 || true; \
		fi; \
		rm -f "$(PJ_HOST_STAMP)"; \
	fi; \
	if [ ! -f "$(PJ_LOCAL_ROOT)/build.mak" ]; then \
		echo "Configuring local pjproject in $(PJ_LOCAL_ROOT)"; \
		(cd "$(PJ_LOCAL_ROOT)" && ./aconfigure); \
	fi; \
	$(MAKE) -C "$(PJ_LOCAL_ROOT)" lib; \
	printf '%s\n' "$$current_host" > "$(PJ_HOST_STAMP)"

# Switch the tree to the integer datapath.  Rebuilds spandsp too, because
# v34rx.c and the private header both change under the flag.
fixed:
	@if [ "`cat $(BUILD_MODE_STAMP) 2>/dev/null`" != "fixed" ]; then \
		echo "switching build mode -> fixed (full rebuild)"; \
		$(MAKE) clean >/dev/null; \
		rm -f $(SPANDSP_DIR)/*.o $(SPANDSP_DIR)/.libs/*.o $(SPANDSP_DIR)/*.lo; \
	fi
	@echo fixed > $(BUILD_MODE_STAMP)
	$(MAKE) -C $(SPANDSP_DIR) libspandsp.la CPPFLAGS="$(FIXED_CPPFLAGS)"
	$(MAKE) $(TARGET) v90_engine_replay v90_upstream_replay \
		CFLAGS="$(CFLAGS) -DV34_FIXED_POINT"
	@echo "built with V34_FIXED_POINT; \`make float\` to go back"

# Back to the floating-point datapath (the default everything else assumes).
float:
	@if [ "`cat $(BUILD_MODE_STAMP) 2>/dev/null`" = "fixed" ]; then \
		echo "switching build mode -> float (full rebuild)"; \
		$(MAKE) clean >/dev/null; \
		rm -f $(SPANDSP_DIR)/*.o $(SPANDSP_DIR)/.libs/*.o $(SPANDSP_DIR)/*.lo; \
		$(MAKE) -C $(SPANDSP_DIR) libspandsp.la; \
	fi
	@echo float > $(BUILD_MODE_STAMP)
	$(MAKE) all

# Compare the two on a recording.  This is the check that matters for the
# fixed path: the arithmetic differs, so the logs cannot be identical -- what
# has to match is the acquisition structure and the decode quality.
fixed-compare:
	@test -n "$(REC)" || { echo "usage: make fixed-compare REC=artifacts/.../live-rx.g711"; exit 2; }
	$(MAKE) float >/dev/null
	./v90_engine_replay $(REC) ulaw --fast > .fixcmp-float.log 2>&1 || true
	$(MAKE) fixed >/dev/null
	./v90_engine_replay $(REC) ulaw --fast > .fixcmp-fixed.log 2>&1 || true
	@for f in float fixed; do \
		printf "  %-6s B1=%-2s E=%-2s DATA=%-2s Ja=%-2s median sym err %s shell bad %s%%\n" $$f \
		  "`grep -c 'B1 acquired' .fixcmp-$$f.log`" \
		  "`grep -c 'upstream E detected' .fixcmp-$$f.log`" \
		  "`grep -c 'enter DATA after B1' .fixcmp-$$f.log`" \
		  "`grep -c 'Ja descriptor recovered' .fixcmp-$$f.log`" \
		  "`grep -o 'sym err [0-9.]*' .fixcmp-$$f.log | awk '{print $$3}' | sort -g | awk '{a[NR]=$$1} END{print (NR? a[int(NR/2)+1] : \"n/a\")}'`" \
		  "`grep -o 'shell bad [0-9]*%' .fixcmp-$$f.log | grep -o '[0-9]*' | sort -n | awk '{a[NR]=$$1} END{print (NR? a[int(NR/2)+1] : \"n/a\")}'`"; \
	done
	@# A recording on which neither arm acquires B1 produces no symbol-error
	@# figures at all, and the two arms then "agree" on nothing.  Say so, loudly:
	@# a comparison harness that reports n/a is worse than one that fails.
	@for f in float fixed; do \
		if [ "`grep -c 'B1 acquired' .fixcmp-$$f.log`" = "0" ]; then \
			echo "  !! $$f never acquired B1 on this recording"; \
			echo "     (`grep -o 'B1 giving up[^;]*' .fixcmp-$$f.log | head -1`)"; \
			vacuous=1; \
		fi; \
	done; \
	if [ -n "$$vacuous" ]; then \
		echo "  !! the comparison above is VACUOUS -- the arms agree because"; \
		echo "     neither decoded anything.  Pick a recording that acquires."; \
		exit 1; \
	fi

clean:
	rm -f modem_gui_native
	rm -rf "Modem GUI.app"
	rm -f audio_sock_modem audio_sock_modem.o slm_bridge v90_engine_peer v90_engine_peer.o engine_pair_test engine_pair_test.o k56flex_client_test.o k56flex_train_test.o k56flex_probe_test.o k56flex_v8bis_test.o k56flex_test.o mica_qam_test.o mica_qam.o x2_test.o x2_sym_test.o v42bis_test.o $(OBJS) $(TARGET) $(TEST_OBJS) $(DECODE_OBJS) $(LEGACY_PCM_DECODE_TEST_OBJS) $(V92_REPLAY_OBJS) $(V92_STARTUP_TEST_OBJS) $(V92_P3_RX_LINE_TEST_OBJS) $(DATA_STACK_TEST_OBJS) $(V44_TEST_OBJS) $(V42_LINK_TEST_OBJS) $(V42_THROUGHPUT_TEST_OBJS) $(FAX_CLASS_TEST_OBJS) $(FAX_CLASS2_TEST_OBJS) $(V34_PHASE2_DECODE_TEST_OBJS) $(V34_MP_TEST_OBJS) $(V34_DATA_TEST_OBJS) $(V34_DUPLEX_TEST_OBJS) $(V56_LOOPBACK_TEST_OBJS) $(AT_TEST_TEST_OBJS) $(PCM_BER_TEST_OBJS) $(V90_ANALOGUE_TX_TEST_OBJS) $(V90_ANALOGUE_RX_TEST_OBJS) $(V92_MH_TEST_OBJS) $(V92_MH_LINE_TEST_OBJS) $(V92_MH_RETRAIN_TEST_OBJS) $(V92_RN_TEST_OBJS) $(V92_RSIG_TEST_OBJS) $(V92_TONE_A_TEST_OBJS) $(TEST_TARGETS) v34_duplex_test *.d tools/*.d $(APPLE_USB_MODEM_PROBE_OBJS) apple_usb_modem_probe apple_usb_modem_audio \
	    tools/apple_usb_modem_coupler.o apple_usb_modem_coupler
	rm -f $(V8BIS_TEST_OBJS) $(V8BIS_FSM_TEST_OBJS) $(V8BIS_MODEM_TEST_OBJS) $(V70_STACK_OBJS) per.o h245_schema.o v76_test.o v75_test.o v70_test.o v70_v34_test.o

distclean: clean
	rm -f "$(SPANDSP_HOST_STAMP)" "$(PJ_HOST_STAMP)" $(BUILD_MODE_STAMP)
	@if [ -f "$(SPANDSP_ROOT)/Makefile" ]; then \
		$(MAKE) -C "$(SPANDSP_ROOT)" distclean >/dev/null 2>&1 || true; \
	fi
	@if [ -f "$(PJ_LOCAL_ROOT)/build.mak" ]; then \
		$(MAKE) -C "$(PJ_LOCAL_ROOT)" distclean >/dev/null 2>&1 || true; \
	fi

v34-tone-matrix: vpcm_decode
	bash scripts/v34_tone_matrix.sh

FORCE:

v90_analogue_linear.o: v90_analogue_linear.c v90_analogue_linear.h v90.h

v34_pp_fit.o: v34_pp_fit.c v34_pp_fit.h
v34_pp_fit_test.o: v34_pp_fit_test.c v34_pp_fit.h v90_analogue_tx.h
v90_analogue_sd.o: v90_analogue_sd.c v90_analogue_sd.h
v90_analogue_sd_test.o: v90_analogue_sd_test.c v90_analogue_sd.h v90_analogue_fse.h
v90_analogue_fse.o: v90_analogue_fse.c v90_analogue_fse.h

v90_sounder.o: v90_sounder.c v90_sounder.h v90.h

# V.42bis lifecycle and bounds regressions.
v42bis_test: v42bis_test.o $(SPANDSP_LIB)
	$(CC) v42bis_test.o -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

# x2 Draft 0.33 clauses 10/11/21/22; no external library dependency.
X2_TEST_OBJS = x2_test.o x2.o
x2_test: $(X2_TEST_OBJS)
	$(CC) $(X2_TEST_OBJS) -o $@
.PHONY: x2-test
x2-test: x2_test
	./x2_test

# K56flex Draft 0.23 clauses 4.4-4.6, 4.12, 7.1-7.12; no external library dependency.
# Regenerate tables/vectors: tools/generate_k56flex_vectors.py ../MicaEmu .
K56FLEX_TEST_OBJS = k56flex_test.o k56flex.o
k56flex_test: $(K56FLEX_TEST_OBJS)
	$(CC) $(K56FLEX_TEST_OBJS) -o $@

# MICA resident QAM engine (test-only, not linked into the modem).
MICA_QAM_TEST_OBJS = mica_qam_test.o mica_qam.o
mica_qam_test: $(MICA_QAM_TEST_OBJS)
	$(CC) $(MICA_QAM_TEST_OBJS) -o $@
.PHONY: k56flex-test
k56flex-test: k56flex_test
	./k56flex_test

K56FLEX_V8BIS_TEST_OBJS = k56flex_v8bis_test.o k56flex_v8bis.o k56flex.o
k56flex_v8bis_test: $(K56FLEX_V8BIS_TEST_OBJS) spandsp
	$(CC) $(K56FLEX_V8BIS_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

X2_SESSION_TEST_OBJS = x2_session_test.o x2_session.o x2_sym.o x2.o x2_mp_rx.o
x2_session_test: $(X2_SESSION_TEST_OBJS)
	$(CC) $(X2_SESSION_TEST_OBJS) -o $@ -lm
.PHONY: x2-session-test
x2-session-test: x2_session_test
	./x2_session_test

K56FLEX_PROBE_TEST_OBJS = k56flex_probe_test.o k56flex_probe.o k56flex.o
k56flex_probe_test: $(K56FLEX_PROBE_TEST_OBJS)
	$(CC) $(K56FLEX_PROBE_TEST_OBJS) -o $@
K56FLEX_TRAIN_TEST_OBJS = k56flex_train_test.o k56flex_train.o k56flex_probe.o k56flex.o
k56flex_train_test: $(K56FLEX_TRAIN_TEST_OBJS)
	$(CC) $(K56FLEX_TRAIN_TEST_OBJS) -o $@

# Original Courier 403 PCMU regression (capture retained in courier-emu).
x2_b1_test: x2_b1_test.o spandsp
	$(CC) x2_b1_test.o $(SPANDSP_LIB) -o $@ -lm

x2-b1-test: x2_b1_test
	./x2_b1_test ../courier-emu/artifacts/x2-upstream-e-20261005/courier-upstream.g711

.PHONY: x2-b1-test

# A clocked, byte-exact PCMU peer for foreign emulator closed-loop tests.
# Two whole engines, each a v90_engine_peer process, against each other one
# 20 ms frame at a time (engine_pair_test.c).
engine_pair_test: engine_pair_test.o fdm_bank.o v90_engine_peer
	$(CC) engine_pair_test.o fdm_bank.o -o $@ -lm

# The engine on a raw G.711 audio socket instead of SIP (audio_sock_modem.c),
# and the slmodemd -e program that puts the SmartLink soft modem on that
# socket (rig/slm_bridge/slm_bridge.c; tools/slm_local_pair.sh drives both).
audio_sock_modem: audio_sock_modem.o $(filter-out sip_modem.o,$(OBJS)) spandsp $(PJ_BUILD_PREREQ)
	$(CC) audio_sock_modem.o $(filter-out sip_modem.o,$(OBJS)) -o $@ $(LDFLAGS)
slm_bridge: rig/slm_bridge/slm_bridge.c spandsp
	$(CC) $(CFLAGS) rig/slm_bridge/slm_bridge.c -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

v90_engine_peer: v90_engine_peer.o $(filter-out sip_modem.o,$(OBJS)) spandsp $(PJ_BUILD_PREREQ)
	$(CC) v90_engine_peer.o $(filter-out sip_modem.o,$(OBJS)) -o $@ $(LDFLAGS)
K56FLEX_CLIENT_TEST_OBJS = k56flex_client_test.o k56flex_client.o k56flex_rxfe.o k56flex_channel.o k56flex_train.o k56flex_probe.o k56flex.o
k56flex_client_test: $(K56FLEX_CLIENT_TEST_OBJS)
	$(CC) $(K56FLEX_CLIENT_TEST_OBJS) -o $@ -lm

LEGACY_PCM_DECODE_TEST_OBJS = legacy_pcm_decode_test.o legacy_pcm_decode.o x2_session.o x2_sym.o x2_mp_rx.o x2.o k56flex_client.o k56flex_rxfe.o k56flex_train.o k56flex_probe.o k56flex.o
legacy_pcm_decode_test: $(LEGACY_PCM_DECODE_TEST_OBJS)
	$(CC) $(LEGACY_PCM_DECODE_TEST_OBJS) -o $@ -lm

X2_SYM_TEST_OBJS = x2_sym_test.o x2_sym.o x2.o
x2_sym_test: $(X2_SYM_TEST_OBJS)
	$(CC) $(X2_SYM_TEST_OBJS) -o $@
.PHONY: x2-sym-test
x2-sym-test: x2_sym_test
	./x2_sym_test

# T.31 Annex B / T.32 Annex C page transfers over byte-exact G.711.
.PHONY: fax-v34-test
fax-v34-test: fax_class_test fax_class2_test engine_pair_test v90_engine_peer
	$(RUN_TESTS) --group fax-v34-test $(TEST_LISTS)

# Full pages through two production engines and their PTYs; no injected
# modem events. The far-end Class 1 DTE owns a software T.30 terminal.
.PHONY: fax-v34-engine-test
fax-v34-engine-test: fax_class2_test v90_engine_peer
	$(RUN_TESTS) --group fax-v34-engine-test $(TEST_LISTS)

