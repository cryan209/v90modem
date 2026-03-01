/* 
   ITU-T G.711-Appendix III  ANSI-C Source Code
   Copyright (c) 2007-2009
   NTT, France Telecom, VoiceAge Corp., Huawei

   Version: 1.0
   Revision Date: Sep. 25, 2009
*/
/*
 *------------------------------------------------------------------------
 *  File: lowband_dec.c
 *  Function: Lower-band decoder
 *------------------------------------------------------------------------
 */

#include "g711App_common.h"
#include "g711.h"
#include "fec_lowband.h"
#include "lowband.h"
#include "lpctool.h"

#ifdef WMOPS
  extern short           Idferc;
  extern short           Idng;
  extern short           Id;
#endif

typedef struct {
  Word16 ferc;
  Word16 pf;
  Word16 law;                    /* G.711 law */
  void*  pFECWork;               /* Lower band FEC work area */
  Word16 sigdec_past[L_WINDOW];  /* buffer for past G.711 decoded signal */
  Word16 mem_A[ORD_M+1];         /* A(z) of previous frame */
  Word16 mem_rc[ORD_M];          /* Reflection coefficients of previous frame */
  Word16 mem_wfilter[ORD_M];     /* buffer for the weighting filter */
  Word16 loss_cnt;               /* indicator of past lost frames (up to 2) */
  Word32 energy;                 /* Signal energy of previous frame */
} lowband_decode_work;

/* -------------------------------------------------------------------
  Function:
    Lower-band decoder constructor
  Return value:
    Pointer of work space
   -------------------------------------------------------------------*/
void    *lowband_decode_const(
  int    law,               /* (i): G.711 decoding law [G711ALAW/G711ULAW] */
  int    ferc,              /* (i): FERC mode                              */
  int    pf                 /* (i): PostFilter mode                        */
) {
  lowband_decode_work *work=NULL;

  if ( law == G711ULAW || law == G711ALAW ) {
    work = (lowband_decode_work *)malloc( sizeof(lowband_decode_work) );
    if ( work != NULL )
    {
      work->law = (Word16)law;
      work->ferc = (Word16)ferc;
      work->pf = (Word16)pf;

      /* FEC routines for 5ms frame */
      work->pFECWork = FEC_lowerband_const( work->ferc, work->pf );
      if ( work->pFECWork == NULL ) {
        lowband_decode_dest((void *)work);
        return NULL;
      }

      lowband_decode_reset( (void *)work );
    }
  }
  return (void *)work;
}

/* -------------------------------------------------------------------
  Function:
    Lower-band decoder destructor
  Return value:
    None
   -------------------------------------------------------------------*/
void    lowband_decode_dest(
  void *ptr   /* (i): Pointer to work space */
) {
  lowband_decode_work *work = (lowband_decode_work *)ptr;
  if (work != NULL)
  {
    FEC_lowerband_dest(work->pFECWork);
    free( work );
  }
}

/* -------------------------------------------------------------------
  Function:
    Lower-band decoder reset
  Return value:
    None
   -------------------------------------------------------------------*/
void    lowband_decode_reset(
  void *ptr   /* (i/o): Pointer to work space */
) {
  lowband_decode_work *work = (lowband_decode_work *)ptr;

  if (work != NULL) {
    work->mem_A[0] = 4096;  move16();
    zero16(ORD_M, &work->mem_A[1]);
    zero16(ORD_M, work->mem_rc);
    zero16(ORD_M, work->mem_wfilter);
    zero16(L_WINDOW, work->sigdec_past);
    work->loss_cnt = 0;  move16();
    work->energy = 0;    move16(); /* could be set to some high value */
    FEC_lowerband_reset(work->pFECWork);
  }
}


/* -------------------------------------------------------------------
  Function:
    Lower band decoder
  Return value:
    None
   -------------------------------------------------------------------*/
void    lowband_decode(
  const unsigned char code0[],   /* (i): Layer 0 bitstream (multiplexed)    */
  int                 loss_flag, /* (i): Frame erasure status flag          */
  Word16              sigout[],  /* (o): Output 5-ms signal                 */
  Word16              *gain,     /* (o): Target gain for noise gate         */
  void                *ptr       /* (i/o): Pointer to work space            */
) {
  Word16  i;
  Word16  tmp;
  Word16  offset;
  Word16  sigtmp;
  Word16  sigdec[L_FRAME_NB];
  Word16  exp[L_FRAME_NB];
  Word16  sign[L_FRAME_NB];
  Word16  excode_buf[ORD_M+L_FRAME_NB], *excode;
  Word32  lval;
  Word32  tmp32;

  lowband_decode_work *work = (lowband_decode_work *)ptr;

  Word16  *rc=work->mem_rc;        /* Reflection coefficients              */
  Word16  *A =work->mem_A;         /* A0(z) with spectral expansion        */

  /* G.711 decoding function */
  short   (*convertLog_Lin)(short, short*, short*);

  IF (loss_flag == 0)
  {
    excode = excode_buf+ORD_M;

    IF (sub(work->law, G711ALAW) == 0)
    {
      convertLog_Lin = convertALaw_Lin;            move16();
      offset = ALAW_OFFSET;                  move16();
    }
    ELSE    /* work->law == G711ULAW */
    {
      convertLog_Lin = convertMuLaw_Lin;             move16();
      offset = 0;                            move16();
    }

    FOR (i=0; i<L_FRAME_NB; i++)
    {
      /* Decoding of the shaped sample */
      sigtmp = (*convertLog_Lin) (code0[i], &exp[i], &sign[i]);
      sigdec[i] = sub(sigtmp, offset);
      move16();
    }

    /* Updating of the past synthesized buffer */
    mov16(L_FRAME_NB, work->sigdec_past + L_FRAME_NB, work->sigdec_past);
    mov16(L_FRAME_NB, sigdec, work->sigdec_past + L_FRAME_NB);
  }

  /* ---------------------------------------------------------- */
  /* Lower-band frame-erasure concealment (FERC)                */
  /* Sigout is 5-ms delayed from sigtmp.                        */
  /* ---------------------------------------------------------- */
#ifdef WMOPS
    setCounter(Idferc);
#endif
  FEC_lowerband( loss_flag, sigdec, sigout, work->pFECWork );
#ifdef WMOPS
    setCounter(Id);
#endif

  /* Updating of the lost frames counter */
  loss_flag = s_min((Word16)loss_flag, 1);      /* 1 or zero */
  tmp = add(work->loss_cnt, (Word16)loss_flag); /* Add 1 if Loss */
  tmp = s_min(tmp, 2);                          /* Do not Count above 2 */
  tmp = sub(tmp, 1);                            /* Decrement if no Loss */
  tmp = add(tmp, (Word16)loss_flag);            /* But Re-Increment Since there is Loss */
  work->loss_cnt = s_max(tmp, 0); move16();     /* Do not Decrement below 0 */

  /* ----------------------------- */
  /* Calculate Gain for Noise Gate */
  /* ----------------------------- */
#ifdef WMOPS
    setCounter(Idng);
#endif
  lval = 300; /* offset energy measure */
  move32();

  FOR (i=L_WINDOW/2; i<L_WINDOW; ++i) {
    tmp = sub(work->sigdec_past[i], mult_r(work->sigdec_past[i-1],24576));
    lval = L_mac(lval,tmp,tmp);
  }
  tmp32 = lval; move32();
  lval = L_add(work->energy,lval);
  work->energy = tmp32; move32();

  SqrtI31(lval,&lval);
  *gain = extract_h(L_shl(lval,9));
  *gain = s_max(*gain, 8192);

  return;
}

