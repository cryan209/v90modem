#
# Makefile for Microsoft Visual C++: nmake -f makefile.cl
#
.SUFFIXES: .obj .c .h

CC=cl

INCLUDES= -I../basicOp_stl2005_v2.2 -Iutil -Ig711App -Ilower_band -Iapp1_postfilter

DFLAGS= -DWMOPS=1

CFLAGS= -nologo -G5 -GX -O2 -W3 $(INCLUDES) $(DFLAGS)

# Do not change the library name.
LIBRARY= g711App.lib

# List header files.
HDRS= util/dsputil.h

# List object files.
OBJS= util/dsputil.obj \
      util/mathtool.obj \
      util/oper_32b.obj \
      util/errexit.obj \
      g711App/g711Appenc.obj \
      g711App/g711Appdec.obj \
      g711App/prehpf.obj \
      g711App/softbit.obj \
      lower_band/lowband_enc.obj \
      lower_band/lowband_dec.obj \
      lower_band/g711a.obj \
      lower_band/g711mu.obj \
      lower_band/fec_lowband.obj \
      lower_band/lpctool.obj \
      lower_band/autocorr_ns.obj \
      app1_postfilter/post.obj \
      app1_postfilter/post_anasyn.obj \
      app1_postfilter/post_gainfct.obj \
      app1_postfilter/post_rfft.obj \
      util/table_mathtool.obj \
      lower_band/table_lowband.obj \
      app1_postfilter/table_post.obj

all: $(LIBRARY)

$(LIBRARY): $(OBJS)
	del $(LIBRARY)
	lib -nologo -OUT:$(LIBRARY) $(OBJS)

$(OBJS): $(HDRS)

clean:
	del /S *.obj *.exe

.c.obj:
	$(CC) -c $(CFLAGS) $*.c -Fo$*.obj
