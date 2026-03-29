CC = gcc

# pjproject 2.16 on Apple Silicon macOS
PJPROJ_DIR   = /opt/homebrew/Cellar/pjproject/2.16
ARCH_SUFFIX  = aarch64-apple-darwin24.6.0

# Local SpanDSP 3.0.0 build (with V.34 support)
SPANDSP_ROOT = spandsp-master
SPANDSP_DIR  = $(SPANDSP_ROOT)/src
SPANDSP_LIB  = $(SPANDSP_DIR)/.libs/libspandsp.a
SPANDSP_MAKE = $(SPANDSP_ROOT)/Makefile

CFLAGS = -Wall -Wextra -O2 -g \
         -I$(PJPROJ_DIR)/include \
         -I$(SPANDSP_DIR) -I$(SPANDSP_DIR)/.. \
         -I/opt/homebrew/include \
         -DPJ_AUTOCONF=1 -DPJ_IS_BIG_ENDIAN=0 -DPJ_IS_LITTLE_ENDIAN=1

# To build on a different macOS version or arch, update ARCH_SUFFIX, e.g.:
#   ARCH_SUFFIX = arm64-apple-darwin23.0.0
# Or run: ls /opt/homebrew/lib/libpj-*.a | sed 's/.*libpj-//' | sed 's/\.a//'

LDFLAGS = \
  -L$(PJPROJ_DIR)/lib \
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
  -lwebrtc-$(ARCH_SUFFIX) \
  $(SPANDSP_LIB) \
  -L/opt/homebrew/lib \
  -ltiff \
  -lssl -lcrypto \
  -lm -lpthread \
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

SRCS   = sip_modem.c modem_engine.c clock_recovery.c data_interface.c v90.c v91.c
OBJS   = $(SRCS:.c=.o)
TARGET = sip_v90_modem
TEST_TARGETS = vpcm_loopback_test
TEST_OBJS = vpcm_loopback_test.o v91.o

.PHONY: all clean spandsp

all: spandsp $(TARGET) $(TEST_TARGETS)

$(TARGET): $(OBJS) $(SPANDSP_LIB)
	$(CC) $(OBJS) -o $@ $(LDFLAGS)

vpcm_loopback_test: $(TEST_OBJS) $(SPANDSP_LIB)
	$(CC) $(TEST_OBJS) -o $@ $(LDFLAGS)

$(SPANDSP_LIB): $(SPANDSP_MAKE)
	$(MAKE) -C $(SPANDSP_ROOT)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

sip_modem.o:      sip_modem.c      modem_engine.h clock_recovery.h data_interface.h
modem_engine.o:   modem_engine.c   modem_engine.h clock_recovery.h
clock_recovery.o: clock_recovery.c clock_recovery.h
data_interface.o: data_interface.c data_interface.h modem_engine.h
v90.o:            v90.c            v90.h
v91.o:            v91.c            v91.h
vpcm_loopback_test.o: vpcm_loopback_test.c v91.h

spandsp:
	$(MAKE) -C $(SPANDSP_ROOT)

clean:
	rm -f $(OBJS) $(TARGET) $(TEST_OBJS) $(TEST_TARGETS)
