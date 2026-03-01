/* 
   ITU-T G.711-Appendix III  ANSI-C Source Code
   Copyright (c) 2007-2009
   NTT, France Telecom, VoiceAge Corp., Huawei

   Version: 1.0
   Revision Date: Sep. 25, 2009
*/
/*
 *------------------------------------------------------------------------
 *  File: lowband_enc.c
 *  Function: Lower-band encoder
 *------------------------------------------------------------------------
 */

#include "g711App_common.h"
#include "g711.h"
#include "lowband.h"
#include "lpctool.h"

#ifdef WMOPS
  extern short           Idns;
  extern short           Id;
#endif

typedef struct {
  Word16 ns;
  Word16 law;                    /* G.711 law */
  Word16 sigdec_past[L_WINDOW];  /* buffer for past G.711 decoded signal */
  Word16 mem_A[ORD_M+1];         /* A(z) of previous frame */
  Word16 mem_rc[ORD_M];          /* Reflection coefficients of previous frame */
  Word16 mem_wfilter[ORD_M];     /* buffer for the weighting filter */
} lowband_encode_work;

/* -------------------------------------------------------------------
  Function:
    Lower-band encoder constructor
  Return value:
    Pointer to work space
   -------------------------------------------------------------------*/
void    *lowband_encode_const(
  int law,   /* (i): G.711 encoding law [G711ALAW/G711ULAW] */
  int ns     /* (i): Noise Shaping mode                     */
) {
  lowband_encode_work *work=NULL;

  if ( law == G711ULAW || law == G711ALAW ) {
    work = (lowband_encode_work *)malloc( sizeof(lowband_encode_work) );
    if ( work != NULL ) {
      work->law = (Word16)law;
      work->ns = (Word16)ns;
      lowband_encode_reset( (void *)work );
    }
  }
  return (void *)work;
}

/* -------------------------------------------------------------------
  Function:
    Lower-band encoder destructor
  Return value:
    None
   -------------------------------------------------------------------*/
void    lowband_encode_dest(
  void *ptr  /* (i): Pointer to work space */
) {
  lowband_encode_work *work = (lowband_encode_work *)ptr;
  if (work != NULL ) free( work );
}

/* -------------------------------------------------------------------
  Function:
    Lower-band encoder reset
  Return value:
    None
   -------------------------------------------------------------------*/
void    lowband_encode_reset(
  void *ptr  /* (i/o): Pointer to work space */
) {
  lowband_encode_work *work = (lowband_encode_work *)ptr;

  if (work != NULL) {
    work->mem_A[0] = 4096; move16();
    zero16(ORD_M, &work->mem_A[1]);
    zero16(ORD_M, work->mem_rc);
    zero16(ORD_M, work->mem_wfilter);
    zero16(L_WINDOW, work->sigdec_past);
  }
}

/* -------------------------------------------------------------------
  Function:
    Lower-band encoder
  Return value:
    None
   -------------------------------------------------------------------*/
void    lowband_encode(
  const Word16  sigin[],  /* (i): Input 5-ms signal                     */
  unsigned char code0[],  /* (o): Core-layer bitstream (multiplexed)    */
  void          *ptr      /* (i/o): Pointer to work space               */
) {
  Word16  i, j;
  Word16  offset;
  Word16  sigtmp;
  Word16  codtmp;
  Word16  sAlpha;
  Word16  *sigdec;
  Word16  norm;
  Word16  stable;
  Word16  excode[L_FRAME_NB];
  Word16  rh[ORD_M+1];             /* Autocorrelations of windowed signal  */
  Word16  rl[ORD_M+1];
  Word32  lval;

  lowband_encode_work *work = (lowband_encode_work *)ptr;

  Word16  *rc=work->mem_rc;        /* Reflection coefficients              */
  Word16  *A =work->mem_A;         /* A0(z) with bandwidth-expansion       */

  /* G.711 decoding function */
  short   (*convertLin_Log)(short, short*, short*, short*);
  short   exp[L_FRAME_NB];

  /* Pointer initialization */
  sigdec = work->sigdec_past + L_FRAME_NB;

  IF (work->ns)
  {
#ifdef WMOPS
    setCounter(Idns);
#endif

    /* LP analysis and filter weighting */
    norm = AutocorrNS(work->sigdec_past, rh, rl);
    Levinson(rh, rl, rc, &stable, ORD_M, A);

    IF (sub(norm, MAX_NORM) >= 0) {
      FOR (i=1; i<=ORD_M; ++i) {
        A[i] = shr(A[i],add(i,sub(norm,MAX_NORM)));
        move16();
      }
    }
    ELSE {
      sAlpha = negate(rc[0]); ;       /* rc[0] == -r[1]/r[0]   */
      IF (sub (sAlpha, -32256) < 0)   /* r[1]/r[0] < -0.984375 */
      {
        sAlpha = add (sAlpha, 32767);
        sAlpha = add (sAlpha, 1536);
        sAlpha = shl (sAlpha, 4);     /* alpha=16*(r[1]/r[0]+1+0.75/16) */
        Weight_a(A, A, mult_r(GAMMA1, sAlpha), ORD_M);
      }
      ELSE {
        Weight_a(A, A, GAMMA1, ORD_M);
      }
    }

    /* Update of the past signal */
    mov16(L_FRAME_NB, work->sigdec_past + L_FRAME_NB, work->sigdec_past);

    IF (work->law == G711ALAW)
    {
      convertLin_Log = convertLin_ALaw;
      offset = ALAW_OFFSET;
      move16();
    }
    ELSE  /* work->law == G711ULAW */
    {
      convertLin_Log = convertLin_MuLaw;
      offset = 0;
      move16();
    }

    FOR (i=0; i<L_FRAME_NB; i++)
    {
      /* Calculation of the shaped sample */
      lval = L_mult(A[0], sigin[i]);    /* put sigin in 32-bits in Q12 (A[0]=1.0) */
      lval = L_mac(lval, A[0], offset); /* put offset in Q12 and add to sigin */
      FOR (j=0; j<ORD_M; j++) {
        lval = L_mac(lval, A[j+1], work->mem_wfilter[j]);
      }
      sigtmp = extract_h(L_shl(lval, 3));

      test();test();test();
      IF (sub(work->law, G711ALAW) == 0               &&
          sub(norm, MAX_NORM) >= 0                    &&
          sub(sigtmp, ALAW_OFFSET-ALAW_DEADZONE) >= 0 &&
          sub(sigtmp, ALAW_OFFSET+ALAW_DEADZONE) <= 0)
      {
        codtmp = 0xD5; move16(); sigdec[i] = 8; move16(); /* reduce G711 cracles */
        exp[i] = 0; move16();

        excode[i] = 7; move16();
        IF (sub(sigtmp, 2) < 0)       { excode[i] = 0;   move16(); }
        ELSE IF (sub(sigtmp, 16) < 0) { excode[i] = shr(sigtmp, 1); move16(); }
      }
      ELSE
      {
        test();test();test();
        IF (sub(work->law, G711ULAW) == 0     &&
            sub(norm, MAX_NORM) >= 0          &&
            sub(sigtmp, -MULAW_DEADZONE) >= 0 &&
            sub(sigtmp, MULAW_DEADZONE) <= 0)
        {
          codtmp = 0xFF; move16(); sigdec[i] = 0; move16();
          exp[i] = 0; move16();

          excode[i] = 3 << 1; move16();
          IF (sub(sigtmp, -1) < 0)      { excode[i] = 0; move16(); } /* 0xFF (R1=0) with R2a ext. 0 and 1 will replace 0x7F with ext. 3 and 2 */
          ELSE IF (sigtmp < 0)          { excode[i] = 1 << 1; move16(); }
          ELSE IF (sub(sigtmp, 2) < 0 ) { excode[i] = 2 << 1; move16(); }
        }
        ELSE
        {
#ifdef WMOPS
    setCounter(Id);
#endif
          codtmp = (*convertLin_Log) (sigtmp, &excode[i], &sigdec[i], &exp[i]);
#ifdef WMOPS
    setCounter(Idns);
#endif
        }
      }
      
      code0[i] = (unsigned char)s_and (codtmp, 0x00FF);  move16();
      sigdec[i] = sub(sigdec[i], offset); move16();

      /* Update the noise-shaping filter memory */
      FOR (j=ORD_M-1; j>0; j--) {
        work->mem_wfilter[j] = work->mem_wfilter[j-1];   move16();
      }
      work->mem_wfilter[0] = sub(sigin[i], sigdec[i]);
      move16();
    }
#ifdef WMOPS
    setCounter(Id);
#endif

  }
  ELSE
  {
    IF (work->law == G711ALAW)
    {
      convertLin_Log = convertLin_ALaw;
    }
    ELSE  /* work->law == G711ULAW */
    {
      convertLin_Log = convertLin_MuLaw;
    }
    FOR (i=0; i<L_FRAME_NB; i++)
    {
      codtmp = (*convertLin_Log) (sigin[i], &excode[i], &sigdec[i], &exp[i]);
      code0[i] = (unsigned char)s_and (codtmp, 0x00FF);  move16();
    }

  }

}
