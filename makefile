CC = gcc

# pjproject 2.16 on Apple Silicon macOS
PJPROJ_DIR   = /opt/homebrew/Cellar/pjproject/2.16
ARCH_SUFFIX  = aarch64-apple-darwin24.6.0

# Local SpanDSP 3.0.0 build (with V.34 support)
SPANDSP_DIR  = spandsp-master/src

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
  $(SPANDSP_DIR)/.libs/libspandsp.a \
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

SRCS   = sip_modem.c modem_engine.c clock_recovery.c data_interface.c
OBJS   = $(SRCS:.c=.o)
TARGET = sip_v90_modem

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(OBJS) -o $@ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

sip_modem.o:      sip_modem.c      modem_engine.h clock_recovery.h data_interface.h
modem_engine.o:   modem_engine.c   modem_engine.h clock_recovery.h
clock_recovery.o: clock_recovery.c clock_recovery.h
data_interface.o: data_interface.c data_interface.h modem_engine.h

clean:
	rm -f $(OBJS) $(TARGET)
