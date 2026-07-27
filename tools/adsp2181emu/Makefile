CC ?= cc
CFLAGS ?= -O2 -g
CFLAGS += -std=c11 -Wall -Wextra -Wno-unused-parameter -Wno-sign-compare

all: eicon_adsp_run libadsp2181.dylib

eicon_adsp_run: eicon_adsp_run.o adsp2181_core.o
	$(CC) $(CFLAGS) $^ -o $@

# Shared library for the ctypes callers (tools/eicon_mips_shim.py).  Built
# from the same core sources, so an emulator change reaches the shim.
libadsp2181.dylib: adsp2181_core.c adsp2181_core.h 2100ops.inc
	$(CC) $(CFLAGS) -fPIC -dynamiclib adsp2181_core.c -o $@

# AddressSanitizer build of the same library, for debugging memory errors
# reached through the ctypes callers.  Point the shim at it with
# ADSP2181_LIB and preload the ASan runtime (see README).
ASAN_FLAGS = -fsanitize=address -fno-omit-frame-pointer -O1 -g
libadsp2181_asan.dylib: adsp2181_core.c adsp2181_core.h 2100ops.inc
	$(CC) -std=c11 $(ASAN_FLAGS) -fPIC -dynamiclib adsp2181_core.c -o $@

adsp2181_core_test: adsp2181_core_test.o adsp2181_core.o
	$(CC) $(CFLAGS) $^ -o $@

eicon_adsp_run.o: eicon_adsp_run.c adsp2181_core.h
adsp2181_core_test.o: adsp2181_core_test.c adsp2181_core.h
adsp2181_core.o: adsp2181_core.c adsp2181_core.h 2100ops.inc

test: adsp2181_core_test
	./adsp2181_core_test

clean:
	rm -f eicon_adsp_run adsp2181_core_test *.o

.PHONY: all test clean
