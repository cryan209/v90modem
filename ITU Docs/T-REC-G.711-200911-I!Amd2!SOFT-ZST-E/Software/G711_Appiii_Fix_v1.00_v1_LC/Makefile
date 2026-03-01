#
# Makefile for GNU gcc/g++ compiler
#
.SUFFIXES: .o .c .cpp .h

# Directory locations
SRC_DIR   = src/mainlib
BASOP_DIR = src/basicOp_stl2005_v2.2
TEST_DIR  = src/test_codec
OUT_DIR=out

SUBDIRS   = $(SRC_DIR) $(BASOP_DIR) $(TEST_DIR)

MAKE      =  make
CC        =  gcc
CXX       =  g++

all: $(SUBDIRS)

$(SUBDIRS)::
	$(MAKE) -C $@ "CC=$(CC)" "CXX=$(CXX)"

clean: 
	$(foreach DIR, $(SUBDIRS), $(MAKE) -C $(DIR) clean;)
	/bin/rm $(TEST_DIR)/*.exe
	/bin/rm -r $(OUT_DIR)
