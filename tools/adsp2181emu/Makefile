CC ?= cc
CFLAGS ?= -O2 -g
CFLAGS += -std=c11 -Wall -Wextra -Wno-unused-parameter -Wno-sign-compare

all: eicon_adsp_run

eicon_adsp_run: eicon_adsp_run.o adsp2181_core.o
	$(CC) $(CFLAGS) $^ -o $@

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
