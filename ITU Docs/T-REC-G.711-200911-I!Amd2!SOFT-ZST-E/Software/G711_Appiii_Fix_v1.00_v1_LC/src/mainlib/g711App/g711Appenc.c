/* 
   ITU-T G.711-Appendix III  ANSI-C Source Code
   Copyright (c) 2007-2009
   NTT, France Telecom, VoiceAge Corp., Huawei

   Version: 1.0
   Revision Date: Sep. 25, 2009
*/
/*
 *------------------------------------------------------------------------
 *  File: g711Appenc.c
 *  Function: G.711 encoder Toolbox
 *------------------------------------------------------------------------
 */

#include "g711App_common.h"
#include "prehpf.h"
#include "lowband.h"

#define OK  0
#define NG  1

/* High-pass filter cutoff definition */
#define FILT_NO_8KHZ_INPUT   5

typedef struct {
  int     ns;
  void*   pHpassFiltBuf; /* High-pass filter buffer */
  void*   SubEncoderL;   /* Work space for lower-band sub-encoder */
} G711AppEncoder_WORK;

/*----------------------------------------------------------------
  Function:
    G.711APP encoder constructor
  Return value:
    Pointer to work space
  ----------------------------------------------------------------*/
void *G711AppEncode_const(
  int law,              /* (i): G.711 law for core layer */
  int ns             /* (i): Noise Shaping mode       */
)
{
  G711AppEncoder_WORK *w=NULL;

  /* Static memory allocation */
  w = (void *)malloc( sizeof(G711AppEncoder_WORK) );
  if ( w == NULL )  return NULL;

  w->ns = ns;

  w->pHpassFiltBuf = highpass_1tap_iir_const();
  if ( w->pHpassFiltBuf == NULL )  error_exit( "HPF init error." );

  w->SubEncoderL = lowband_encode_const(law, ns);
  if ( w->SubEncoderL == NULL )  error_exit( "Lower band init error." );

  G711AppEncode_reset( (void *)w );

  return (void *)w;
}

/*----------------------------------------------------------------
  Function:
    G.711APP encoder destructor
  Return value:
    None
  ----------------------------------------------------------------*/
void G711AppEncode_dest(
  void*  p_work   /* (i): Work space */
)
{
  G711AppEncoder_WORK *w=(G711AppEncoder_WORK *)p_work;

  if ( w != NULL )
  {
    highpass_1tap_iir_dest( w->pHpassFiltBuf );
    lowband_encode_dest( w->SubEncoderL );   /* Lower band */

    free( w );
  }
}

/*----------------------------------------------------------------
  Function:
    G.711APP encoder reset
  Return value:
    OK
  ----------------------------------------------------------------*/
int  G711AppEncode_reset(
  void*  p_work   /* (i/o): Work space */
)
{
  G711AppEncoder_WORK *w=(G711AppEncoder_WORK *)p_work;

  if ( w != NULL )
  {
    highpass_1tap_iir_reset(w->pHpassFiltBuf);
    lowband_encode_reset( w->SubEncoderL );   /* Lower band */
  }
  return OK;
}

/*----------------------------------------------------------------
  Function:
    G.711APP encoder
  Return value:
    OK/NG
  ----------------------------------------------------------------*/
int  G711AppEncode(
  const short*    inwave,
  unsigned char*  bitstream,
  void*           p_work
) {
  Word16  SubSigLow[L_FRAME_NB];
  unsigned char  *bpt = bitstream;

  G711AppEncoder_WORK *w=(G711AppEncoder_WORK *)p_work;

  if (p_work == NULL) return NG;

  IF (w->ns)
  {
    /* High-pass filtering */
    highpass_1tap_iir( FILT_NO_8KHZ_INPUT,
                       L_FRAME_NB, (Word16 *)inwave,
                       SubSigLow, w->pHpassFiltBuf );
    /* ------------------------------------------------------------- */
    /* Narrow-band encoder                                           */
    /* ------------------------------------------------------------- */
    lowband_encode( SubSigLow, bpt, w->SubEncoderL );
  }
  ELSE
  {
    /* ------------------------------------------------------------- */
    /* Narrow-band encoder                                           */
    /* ------------------------------------------------------------- */
    lowband_encode( (Word16 *)inwave, bpt, w->SubEncoderL );
  }


  bpt += NBytesPerFrame0;

  return OK;
}
