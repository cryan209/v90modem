#
# Makefile for GNU gcc/g++ compiler
#
.SUFFIXES: .o .c .h

SRC_DIR       = ../mainlib
BASOP_DIR     = ../basicOp_stl2005_v2.2

CC	          = gcc
CFLAGS	      = -O2 -Wall -fno-builtin

CFLAGS        += -DWMOPS=1
CFLAGS        += -I$(SRC_DIR)/g711App
CFLAGS        += -I$(BASOP_DIR)

LD            = $(CC)
LDFLAGS       = -Wall -static

# Add options if needed.
LDFLAGS       +=

LIBS	      = $(SRC_DIR)/libg711App.a \
                $(BASOP_DIR)/libbasop.a

# List header files.
HDRS	      = $(SRC_DIR)/g711App/g711App.h

# List C source files.
SRCS          =

OBJS =   $(patsubst %.cpp,%.o,$(SRCS:%.c=%.o))

ENCODER       = encoder

ENCODER_OBJ   = $(ENCODER).o

DECODER       = decoder

DECODER_OBJ   = $(DECODER).o

TARGET        =	$(ENCODER) $(DECODER)

all:		  $(TARGET)

$(ENCODER): $(ENCODER_OBJ) $(OBJS) $(LIBS)
	$(LD) $(LDFLAGS) $(ENCODER_OBJ) $(OBJS) $(LIBS) -o $@

$(DECODER): $(DECODER_OBJ) $(OBJS) $(LIBS)
	$(LD) $(LDFLAGS) $(DECODER_OBJ) $(OBJS) $(LIBS) -o $@

$(ENCODER_OBJ): $(HDRS)
$(DECODER_OBJ): $(HDRS)
$(OBJS): $(HDRS)

clean:
	@rm -f $(TARGET) $(ENCODER_OBJ) $(DECODER_OBJ) $(OBJS) core *.ln

.c.o:
	$(CC) $(CFLAGS) -c $< -o $@
