CC = gcc

UNAME_S := $(shell uname -s)
USE_LOCAL_PJPROJECT ?= 1
PJ_LOCAL_ROOT ?= pjproject
PJ_LOCAL_MAKEFILE := $(PJ_LOCAL_ROOT)/Makefile

# Local SpanDSP 3.0.0 build (with V.34 support)
SPANDSP_ROOT = spandsp-master
SPANDSP_DIR  = $(SPANDSP_ROOT)/src
SPANDSP_LIB  = $(SPANDSP_DIR)/.libs/libspandsp.a
SPANDSP_MAKE = $(SPANDSP_ROOT)/Makefile

# Shared defaults
PJ_CFLAGS   ?=
PJ_LIBS     ?=
SYSTEM_LIBS ?= -ltiff -lssl -lcrypto -lm -lpthread
PJ_BUILD_PREREQ ?=

ifneq ($(and $(filter 1,$(USE_LOCAL_PJPROJECT)),$(wildcard $(PJ_LOCAL_MAKEFILE))),)
  PJ_BUILD_PREREQ := pjproject
  PJ_LOCAL_PJSUA  := $(firstword $(wildcard $(PJ_LOCAL_ROOT)/pjsip/lib/libpjsua-*.a))
  PJ_LOCAL_SUFFIX := $(patsubst $(PJ_LOCAL_ROOT)/pjsip/lib/libpjsua-%.a,%,$(PJ_LOCAL_PJSUA))
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
    PJPROJ_DIR  ?= /opt/homebrew/Cellar/pjproject/2.16
    ARCH_SUFFIX ?= aarch64-apple-darwin24.6.0

    PJ_CFLAGS += -I$(PJPROJ_DIR)/include -I/opt/homebrew/include
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
    SYSTEM_LIBS += -L/opt/homebrew/lib \
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

CFLAGS = -Wall -Wextra -O2 -g \
         -I$(SPANDSP_DIR) -I$(SPANDSP_DIR)/.. \
         $(PJ_CFLAGS) \
         -DPJ_AUTOCONF=1 -DPJ_IS_BIG_ENDIAN=0 -DPJ_IS_LITTLE_ENDIAN=1

# To build on a different macOS version or arch, update ARCH_SUFFIX, e.g.:
#   ARCH_SUFFIX = arm64-apple-darwin23.0.0
# Or run: ls /opt/homebrew/lib/libpj-*.a | sed 's/.*libpj-//' | sed 's/\.a//'

LDFLAGS = $(PJ_LIBS) $(SPANDSP_LIB) $(SYSTEM_LIBS)

SRCS   = sip_modem.c modem_engine.c clock_recovery.c data_interface.c v90.c v91.c vpcm_cp.c
OBJS   = $(SRCS:.c=.o)
TARGET = sip_v90_modem
TEST_TARGETS = vpcm_loopback_test
TEST_OBJS = vpcm_loopback_test.o v91.o vpcm_cp.o

USE_V34_STUBS ?= 0
ifeq ($(USE_V34_STUBS),1)
SRCS += v34_stubs.c
TEST_OBJS += v34_stubs.o
endif

.PHONY: all clean spandsp pjproject

all: $(TARGET) $(TEST_TARGETS)

$(TARGET): $(OBJS) $(SPANDSP_LIB) $(PJ_BUILD_PREREQ)
	$(CC) $(OBJS) -o $@ $(LDFLAGS)

vpcm_loopback_test: $(TEST_OBJS) $(SPANDSP_LIB) $(PJ_BUILD_PREREQ)
	$(CC) $(TEST_OBJS) -o $@ $(LDFLAGS)

$(SPANDSP_LIB):
	@if [ ! -f "$(SPANDSP_LIB)" ]; then \
		$(MAKE) -C $(SPANDSP_ROOT) ; \
	fi

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

sip_modem.o:      sip_modem.c      modem_engine.h clock_recovery.h data_interface.h
modem_engine.o:   modem_engine.c   modem_engine.h clock_recovery.h
clock_recovery.o: clock_recovery.c clock_recovery.h
data_interface.o: data_interface.c data_interface.h modem_engine.h
v90.o:            v90.c            v90.h
v91.o:            v91.c            v91.h vpcm_cp.h
vpcm_cp.o:        vpcm_cp.c        vpcm_cp.h
ifeq ($(USE_V34_STUBS),1)
v34_stubs.o:      v34_stubs.c
endif
vpcm_loopback_test.o: vpcm_loopback_test.c v91.h vpcm_cp.h

spandsp:
	$(MAKE) -C $(SPANDSP_ROOT)

pjproject:
	@if [ ! -f "$(PJ_LOCAL_ROOT)/build.mak" ]; then \
		echo "Configuring local pjproject in $(PJ_LOCAL_ROOT)"; \
		(cd "$(PJ_LOCAL_ROOT)" && ./aconfigure); \
	fi
	$(MAKE) -C $(PJ_LOCAL_ROOT) lib

clean:
	rm -f $(OBJS) $(TARGET) $(TEST_OBJS) $(TEST_TARGETS)
