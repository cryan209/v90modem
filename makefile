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
SYSTEM_LIBS ?= $(TIFF_LDFLAGS) $(JPEG_LDFLAGS) $(OPENSSL_LDFLAGS) -lm -lpthread
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
         -I. -I$(SPANDSP_DIR) -I$(SPANDSP_DIR)/.. -Iport \
         $(PJ_CFLAGS) $(TIFF_CFLAGS) \
         -DPJ_AUTOCONF=1 -DPJ_IS_BIG_ENDIAN=0 -DPJ_IS_LITTLE_ENDIAN=1

# To build against a non-default system pjproject on macOS, update ARCH_SUFFIX, e.g.:
#   ARCH_SUFFIX = arm64-apple-darwin23.0.0
# Or run: ls $(HOMEBREW_PREFIX)/lib/libpj-*.a | sed 's/.*libpj-//' | sed 's/\.a//'

LDFLAGS = $(PJ_LIBS) $(SPANDSP_LIB) $(SYSTEM_LIBS)

SRCS   = sip_modem.c modem_engine.c clock_recovery.c data_interface.c data_stack.c v90.c v90_cp_rx.c v90_cp_live.c v90_analogue_tx.c v90_analogue_rx.c v90_analogue_phase3.c v90_analogue_phase4.c v90_dil_measure.c v90_dil_presets.c p3_demod.c v91.c vpcm_cp.c vpcm_g711_stream.c vpcm_call.c vpcm_call_pair.c vpcm_link.c vpcm_v91_session.c v92_phase3_decode.c v92_phase3_ru.c v92_ja_decode.c v92_p3_rx.c v92_phase4_decode.c v92_cp_rx.c v92_trn2u.c v92_upstream_data.c v92_upstream_rx.c
OBJS   = $(SRCS:.c=.o)
TARGET = sip_v90_modem
TEST_TARGETS = port_cp_stream_test port_data_rx_test port_v34_fixed_test port_v34_fixed_lms_test port_v34_fixed_solve_test vpcm_loopback_test vpcm_decode vpcm_encode v92_trn2u_replay data_stack_test v42_link_test v34_phase2_decode_test v34_mp_test v34_data_test v34_gardner_test fax_class_test v90_upstream_replay v90_engine_replay v34_duplex_test v32bis_spandsp_test v92_proc_eval_test v90_analogue_tx_test v90_analogue_rx_test
TEST_OBJS = vpcm_loopback_test.o v90.o v90_cp_rx.o v90_dil_rx.o v90_dil_measure.o v90_dil_presets.o v90_analogue_tx.o v90_analogue_rx.o v90_analogue_phase3.o v90_analogue_phase4.o v91.o vpcm_cp.o vpcm_g711_stream.o vpcm_call.o vpcm_call_pair.o vpcm_link.o vpcm_v90_session.o vpcm_v91_session.o vpcm_v91_loopback.o v92_phase3_decode.o v92_phase3_ru.o v92_phase4_decode.o v92_ja_decode.o v92_p3_rx.o v92_cp_rx.o v92_trn2u.o v92_upstream_data.o v92_upstream_rx.o p3_demod.o
DECODE_OBJS = vpcm_decode.o v90_dil_measure.o v90_dil_presets.o v34_phase2_decode.o v34_info_decode.o v8bis_decode.o v92_short_phase1_decode.o v92_short_phase2_decode.o v92_phase3_decode.o v92_phase3_ru.o v92_phase4_decode.o v92_ja_decode.o v92_p3_rx.o v92_anspcm_decode.o p3_demod.o v90.o v90_cp_rx.o v91.o vpcm_cp.o v21_fsk_demod.o phase12_decode.o call_init_tone_probe.o v90_dil_rx.o
ENCODE_OBJS = vpcm_encode.o v90.o v91.o vpcm_cp.o v92_phase4_decode.o v90_dil_measure.o v90_dil_presets.o
V92_REPLAY_OBJS = tools/v92_trn2u_replay.o v92_trn2u.o v92_cp_rx.o vpcm_cp.o
DATA_STACK_TEST_OBJS = data_stack_test.o data_stack.o
V42_LINK_TEST_OBJS = v42_link_test.o
V34_PHASE2_DECODE_TEST_OBJS = v34_phase2_decode_test.o v34_phase2_decode.o
V34_MP_TEST_OBJS = v34_mp_test.o
V34_DATA_TEST_OBJS = v34_data_test.o
V34_GARDNER_TEST_OBJS = v34_gardner_test.o
FAX_CLASS_TEST_OBJS = fax_class_test.o data_interface.o
V90_UPSTREAM_REPLAY_OBJS = v90_upstream_replay.o
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
V34_DUPLEX_TEST_OBJS = v34_duplex_test.o
V34_HDX_TEST_OBJS = v34_hdx_test.o
V32BIS_SPANDSP_TEST_OBJS = v32bis_spandsp_test.o
V90_ANALOGUE_TX_TEST_OBJS = v90_analogue_tx_test.o v90_analogue_tx.o v90_analogue_phase4.o v90_dil_measure.o v90.o v90_cp_rx.o v90_dil_presets.o v91.o vpcm_cp.o v92_phase4_decode.o
V90_ANALOGUE_RX_TEST_OBJS = v90_analogue_rx_test.o v90_analogue_rx.o v90_analogue_phase3.o v90_analogue_phase4.o v90_analogue_tx.o v90_dil_measure.o v90.o v90_cp_rx.o v90_dil_presets.o v91.o vpcm_cp.o v92_phase4_decode.o
# v92_proc_eval_test.c includes phase12_decode.c directly (its evaluator is
# static), so it links phase12_decode.o's dependencies but not the .o itself.
V92_PROC_EVAL_TEST_OBJS = v92_proc_eval_test.o v34_info_decode.o v8bis_decode.o v92_short_phase1_decode.o v92_short_phase2_decode.o v92_anspcm_decode.o v92_cp_rx.o v92_phase4_decode.o v90.o v90_cp_rx.o v91.o vpcm_cp.o v21_fsk_demod.o call_init_tone_probe.o v90_dil_measure.o v90_dil_presets.o

USE_V34_STUBS ?= 0
ifeq ($(USE_V34_STUBS),1)
SRCS += v34_stubs.c
TEST_OBJS += v34_stubs.o
endif

.PHONY: all test clean distclean fixed float fixed-compare spandsp pjproject v34-tone-matrix v34-duplex-test v34-matrix-test v32bis-ref-test v32bis-datapump-test v32bis-test v91-serial-pair-test eicon-rx-test g711-path-test FORCE

all: $(TARGET) $(TEST_TARGETS)

test: $(TEST_TARGETS)
	./data_stack_test
	./v42_link_test
	./v34_phase2_decode_test
	./v34_mp_test
	./v34_data_test
	./v34_gardner_test
	./fax_class_test
# The rows that recover payload without a single bit error.  Every rate that
# trains at all now does, in both laws; only 3429 does not train, so the two
# 3429 rows stay out.  `make v34-matrix-test` runs all twelve.
	./v34_duplex_test 2400 9600 ulaw
	./v34_duplex_test 2400 9600 alaw
	./v34_duplex_test 2743 9600 ulaw
	./v34_duplex_test 2743 9600 alaw
	./v34_duplex_test 2800 9600 ulaw
	./v34_duplex_test 2800 9600 alaw
	./v34_duplex_test 3000 9600 ulaw
	./v34_duplex_test 3000 9600 alaw
	./v34_duplex_test 3200 9600 ulaw
	./v34_duplex_test 3200 9600 alaw
# 21600 bps, where the data-mode carrier and equalizer work shows.  These rows
# all carried errors or recovered nothing before B1 was made to supply the
# residual carrier frequency and the equalizer was allowed to adapt in data
# mode.  2400 baud is deliberately absent: 21600 there is 9 bits per symbol,
# an 896-point constellation whose symbols sit RMS ~20 on a grid of spacing 2,
# and the bearer's own noise floor is marginal for it -- see
# `docs/v34_data_mode_rates.md`.
	./v34_duplex_test 2800 21600 ulaw
	./v34_duplex_test 3000 21600 ulaw
	./v34_duplex_test 3200 21600 ulaw
	./v34_duplex_test 3200 21600 alaw
	./v34_duplex_test 3429 21600 ulaw
	./v34_duplex_test 3429 21600 alaw
# V.34 11.6 rate renegotiation, the resynchronisation that does not cost a
# retrain.  Each row runs to data mode, has the CALLER initiate 11.6, leaves
# the answerer to detect its S and respond through the same public entry point
# the engine uses, and then requires 8000 bits of payload in BOTH directions
# with zero errors on the far side of it.  Every symbol rate that trains does
# this at 9600 in both laws.  21600 is deliberately absent: it renegotiates
# and comes back decoding -- B1 correlation 0.99, shell index 0-1% -- but some
# rows carry bit errors afterwards, and a longer TRN (which 11.6.1.1.1 permits
# up to 2000 ms) does not help, so the cause is not equalizer reconvergence
# and is not understood.  See docs/retrain_and_resync.md.
	V34_DUPLEX_RENEG=4000 ./v34_duplex_test 2400 9600 ulaw
	V34_DUPLEX_RENEG=4000 ./v34_duplex_test 2400 9600 alaw
	V34_DUPLEX_RENEG=4000 ./v34_duplex_test 2743 9600 ulaw
	V34_DUPLEX_RENEG=4000 ./v34_duplex_test 2743 9600 alaw
	V34_DUPLEX_RENEG=4000 ./v34_duplex_test 2800 9600 ulaw
	V34_DUPLEX_RENEG=4000 ./v34_duplex_test 2800 9600 alaw
	V34_DUPLEX_RENEG=4000 ./v34_duplex_test 3000 9600 ulaw
	V34_DUPLEX_RENEG=4000 ./v34_duplex_test 3000 9600 alaw
	V34_DUPLEX_RENEG=4000 ./v34_duplex_test 3200 9600 ulaw
	V34_DUPLEX_RENEG=4000 ./v34_duplex_test 3200 9600 alaw
	./v32bis_spandsp_test
	./v92_proc_eval_test
	./v90_analogue_tx_test
	./v90_analogue_rx_test
	./vpcm_loopback_test --all-tests
	# The coupled analogue<->digital Phase 2 harness.  It is the only in-tree
	# check that both halves of the §9.2.1.1/§9.2.2.1 tone choreography mesh,
	# and it costs ~0.1 s, so it runs by default rather than staying behind
	# --experimental-v90-info.
	./vpcm_loopback_test --session-only --experimental-v90-info

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

v32bis-test: v32bis_spandsp_test v32bis-ref-test v32bis-datapump-test
	./v32bis_spandsp_test

$(TARGET): $(OBJS) spandsp $(PJ_BUILD_PREREQ)
	$(CC) $(OBJS) -o $@ $(LDFLAGS)

vpcm_loopback_test: $(TEST_OBJS) spandsp $(PJ_BUILD_PREREQ)
	$(CC) $(TEST_OBJS) -o $@ $(LDFLAGS)

vpcm_decode: $(DECODE_OBJS) spandsp $(PJ_BUILD_PREREQ)
	$(CC) $(DECODE_OBJS) -o $@ $(LDFLAGS)

vpcm_encode: $(ENCODE_OBJS) spandsp $(PJ_BUILD_PREREQ)
	$(CC) $(ENCODE_OBJS) -o $@ $(LDFLAGS)

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

data_stack_test: $(DATA_STACK_TEST_OBJS) spandsp
	$(CC) $(DATA_STACK_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

fax_class_test: $(FAX_CLASS_TEST_OBJS) spandsp
	$(CC) $(FAX_CLASS_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

v42_link_test: $(V42_LINK_TEST_OBJS) spandsp
	$(CC) $(V42_LINK_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

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

v34_hdx_test: $(V34_HDX_TEST_OBJS) spandsp
	$(CC) $(V34_HDX_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

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

# Not a test: a tool for replaying a recorded call into the upstream
# receiver, so faults that only show after tens of seconds can be bisected
# without waiting on the rig.
v90_upstream_replay: $(V90_UPSTREAM_REPLAY_OBJS) spandsp
	$(CC) $(V90_UPSTREAM_REPLAY_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

v90_engine_replay: $(V90_ENGINE_REPLAY_OBJS) spandsp $(PJ_BUILD_PREREQ)
	$(CC) $(V90_ENGINE_REPLAY_OBJS) -o $@ $(LDFLAGS)

v90_analogue_tx_test: $(V90_ANALOGUE_TX_TEST_OBJS) spandsp
	$(CC) $(V90_ANALOGUE_TX_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

v90_analogue_rx_test: $(V90_ANALOGUE_RX_TEST_OBJS) spandsp
	$(CC) $(V90_ANALOGUE_RX_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

v92_proc_eval_test.o: phase12_decode.c phase12_decode.h

v92_proc_eval_test: $(V92_PROC_EVAL_TEST_OBJS) spandsp
	$(CC) $(V92_PROC_EVAL_TEST_OBJS) -o $@ $(SPANDSP_LIB) $(SYSTEM_LIBS)

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

sip_modem.o:      sip_modem.c      modem_engine.h data_interface.h
modem_engine.o:   modem_engine.c   modem_engine.h data_stack.h clock_recovery.h v90.h v90_cp_rx.h v91.h v92_p3_rx.h v92_cp_rx.h v92_trn2u.h v92_upstream_rx.h
clock_recovery.o: clock_recovery.c clock_recovery.h
data_interface.o: data_interface.c data_interface.h modem_engine.h
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
v92_p3_rx.o:      v92_p3_rx.c      v92_p3_rx.h v92_ja_decode.h v90.h p3_demod.h
p3_demod.o:       p3_demod.c       p3_demod.h
v34_phase2_decode.o: v34_phase2_decode.c v34_phase2_decode.h v90.h v91.h
v34_phase2_decode_test.o: v34_phase2_decode_test.c v34_phase2_decode.h
v34_mp_test.o: v34_mp_test.c $(SPANDSP_DIR)/spandsp/v34.h
v34_data_test.o: v34_data_test.c $(SPANDSP_DIR)/spandsp/v34.h
v34_gardner_test.o: v34_gardner_test.c $(SPANDSP_DIR)/v34_gardner.h
v90_upstream_replay.o: v90_upstream_replay.c $(SPANDSP_DIR)/spandsp/v34.h
v90_engine_replay.o: v90_engine_replay.c modem_engine.h
v34_duplex_test.o: v34_duplex_test.c $(SPANDSP_DIR)/spandsp/v34.h
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
	rm -f $(OBJS) $(TARGET) $(TEST_OBJS) $(DECODE_OBJS) $(V92_REPLAY_OBJS) $(DATA_STACK_TEST_OBJS) $(V42_LINK_TEST_OBJS) $(FAX_CLASS_TEST_OBJS) $(V34_PHASE2_DECODE_TEST_OBJS) $(V34_MP_TEST_OBJS) $(V34_DATA_TEST_OBJS) $(V34_DUPLEX_TEST_OBJS) $(V90_ANALOGUE_TX_TEST_OBJS) $(V90_ANALOGUE_RX_TEST_OBJS) $(TEST_TARGETS) v34_duplex_test

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
