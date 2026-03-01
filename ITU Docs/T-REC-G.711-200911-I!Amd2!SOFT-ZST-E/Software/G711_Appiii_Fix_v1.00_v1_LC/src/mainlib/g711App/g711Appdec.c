/* 
   ITU-T G.711-Appendix III  ANSI-C Source Code
   Copyright (c) 2007-2009
   NTT, France Telecom, VoiceAge Corp., Huawei

   Version: 1.0
   Revision Date: Sep. 25, 2009
*/
/*
 *------------------------------------------------------------------------
 *  File: g711Appdec.c
 *  Function: G.711 decoder Toolbox
 *------------------------------------------------------------------------
 */

#include "g711App_common.h"
#include "lowband.h"

#define OK  0
#define NG  1

#ifdef WMOPS
  extern short           Idng;
  extern short           Id;
#endif

typedef struct {
  int     ng;
  int     law;         /* Logarithmic law */
  int     gain_ns;     /* Noise shaping gain */
  void*   SubDecoderL; /* Work space for lower-band sub-decoder */
  void*   SubDecoderH; /* Work space for higher-band sub-decoder */
  void*   pQMFBuf;     /* QMF filter buffer */
} G711AppDecoder_WORK;

/*----------------------------------------------------------------
  Function:
    G.711APP decoder constructor
  Return value:
    Pointer to work space
  ----------------------------------------------------------------*/
void *G711AppDecode_const(
  int  law,   /* (i): G.711 law for core layer */
  int  ng,    /* (i): Noise Gate mode          */
  int  ferc,  /* (i): FERC mode                */
  int  pf     /* (i): PostFilter mode          */
)
{
  G711AppDecoder_WORK *w=NULL;

  /* Static memory allocation */
  w = (void *)malloc( sizeof(G711AppDecoder_WORK) );
  if ( w == NULL )  return NULL;

  w->ng  = ng;
  w->law = law;

  w->SubDecoderL = lowband_decode_const(law, ferc, pf);
  if ( w->SubDecoderL == NULL )  error_exit( "Lower band init error." );

  G711AppDecode_reset( (void *)w );

  return (void *)w;
}

/*----------------------------------------------------------------
  Function:
    G.711APP decoder destructor
  Return value:
    None
  ----------------------------------------------------------------*/
void G711AppDecode_dest(
  void*  p_work  /* (i): Work space */
)
{
  G711AppDecoder_WORK *w=(G711AppDecoder_WORK *)p_work;

  if ( w != NULL )
  {
    lowband_decode_dest( w->SubDecoderL );   /* Lower band */

    free( w );
  }
}

/*----------------------------------------------------------------
  Function:
    G.711APP decoder reset
  Return value:
    OK
  ----------------------------------------------------------------*/
int  G711AppDecode_reset(
  void*  p_work  /* (i/o): Work space */
)
{
  G711AppDecoder_WORK *w=(G711AppDecoder_WORK *)p_work;

  if ( w != NULL )
  {
    lowband_decode_reset( w->SubDecoderL );  /* Lower band */
    w->gain_ns = 32767;                      /* start with full gain */
  }
  return OK;
}

/*----------------------------------------------------------------
  Function:
    G.711APP decoder
  Return value:
    OK/NG
  ----------------------------------------------------------------*/
int  G711AppDecode(
  const unsigned char*  bitstream,   /* (i):   Input bitstream  */
  short*                outwave,     /* (o):   Output signal    */
  void*                 p_work,      /* (i/o): Work space       */
  int                   ploss_status /* (i):   Packet-loss flag */
) {
  const unsigned char  *bpt = bitstream;
  Word16  SubSigLow[L_FRAME_NB];
  Word16  gain = 32767;
  Word32  gain32;
  Word16  i, n;

  G711AppDecoder_WORK *w=(G711AppDecoder_WORK *)p_work;

  if (p_work == NULL) return NG;

  /* ------------------------------------------------------------- */
  /* Lower-band decoder                                            */
  /* Frame erasure concealment is integrated in the decoder.       */
  /* ------------------------------------------------------------- */
  lowband_decode( bpt, ploss_status, SubSigLow, &gain, w->SubDecoderL );
  bpt += NBytesPerFrame0;
  
  /* --------------------- */
  /* Band reconstructing   */
  /* --------------------- */
  mov16( L_FRAME_NB, SubSigLow, outwave );

  IF (w->ng)
  {
    n = L_FRAME_NB;
    gain32 = L_add(L_mult(gain,350),32768); /* inc gain to be able to converge to 1.0 */

    FOR (i=0; i<n; ++i)
    {
      w->gain_ns = mac_r(gain32,w->gain_ns,32418); /* 32418 = 32768 - 350 */

      IF (sub(w->gain_ns, 32767) < 0)
      {
        outwave[i] = mult_r(w->gain_ns,outwave[i]); move16();
      }
    }
  }

#ifdef WMOPS
  setCounter(Id);
#endif

  return OK;
}
