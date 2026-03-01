#
# Makefile for GNU gcc/g++ compiler
#
.SUFFIXES: .o .c .h

CC	          = gcc
CFLAGS	      = -O2 -Wall -fno-builtin

CFLAGS        += -DWMOPS=1
CFLAGS        += -I../basicOp_stl2005_v2.2
CFLAGS        += -Iutil -Ig711App -Ilower_band -Iapp1_postfilter

# Do not change the library name.
LIBRARY	      = libg711App.a

# List common header files.
HDRS	      = util/dsputil.h

# List C source files.
SRCS	      = util/dsputil.c \
				util/mathtool.c \
				util/oper_32b.c \
				util/errexit.c \
				g711App/g711Appenc.c \
				g711App/g711Appdec.c \
				g711App/prehpf.c \
				g711App/softbit.c \
				lower_band/lowband_enc.c \
				lower_band/lowband_dec.c \
				lower_band/g711a.c \
				lower_band/g711mu.c \
				lower_band/fec_lowband.c \
				lower_band/lpctool.c \
				lower_band/autocorr_ns.c \
				app1_postfilter/post.c \
				app1_postfilter/post_anasyn.c \
				app1_postfilter/post_gainfct.c \
				app1_postfilter/post_rfft.c \
				util/table_mathtool.c \
				lower_band/table_lowband.c \
				app1_postfilter/table_post.c

OBJS =   $(SRCS:%.c=%.o)

all:		$(LIBRARY)

$(LIBRARY):	$(OBJS)
		@echo  $(LIBRARY)
		ar cru $(LIBRARY) $(OBJS)
		ranlib $(LIBRARY)

$(OBJS): $(HDRS)

clean:;		@rm -f $(LIBRARY) $(OBJS) core *.ln

.c.o:
	$(CC) $(CFLAGS) -c $< -o $@
