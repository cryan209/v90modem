/* 
   ITU-T G.711-Appendix III  ANSI-C Source Code
   Copyright (c) 2007-2009
   NTT, France Telecom, VoiceAge Corp., Huawei

   Version: 1.0
   Revision Date: Sep. 25, 2009
*/
/*
 *------------------------------------------------------------------------
 *  File: fec_lowband.h
 *  Function: Frame erasure concealment (FERC)
 *------------------------------------------------------------------------
 */

#include "g711App_common.h"
#include "oper_32b.h"
#include "lpctool.h"
#include "fec_lowband.h"
#include "post_const.h"
#include "post.h"
#include "post_anasyn.h"
#include "post_gainfct.h"

#define L_FRAME2      (2*L_FRAME_NB)
#define L_AHEAD       40    /* Output speech is 1-frame delay. */
#define L_BUFF        (L_AHEAD+L_FRAME_NB)
#define RSX_LEN       5

#ifdef WMOPS
  extern short           Idpf;
  extern short           Idferc;
#endif


/* Static memory allocated dynamically */
typedef struct _LBFEC_STATE
{
  Word16    prev_bfi;    /* bad frame indicator of previous frame */

  /* signal buffers */
  Word16    *mem_speech; /* lower-band speech buffer */
  Word16    *mem_exc;    /* past LPC residual */

  /* analysis results (signal class, LPC coefficients, pitch delay) */
  Word16    clas;        /* unvoiced, weakly voiced, voiced, transient */
  Word16    t0;          /* pitch delay */
  Word16    maxco;       /* correlation at pitch delay */

  /* variables for crossfade */
  Word16    count_crossfade; /* counter for cross-fading (number of samples) */

  /* variables for synthesis attenuation */
  Word16    count_att;   /* counter for lower-band attenuation (number of samples) */
  Word16    inc_att;     /* increment for counter update */
  Word16    fact1;       /* increment for attenuation factor, first part of the curve */
  Word16    fact2p;      /* additional increment for attenuation factor, 2nd part of the curve (attenuiation increment is fact1+fact2p) */
  Word16    fact3p;      /* additional increment for attenuation factor, 3rd part of the curve (attenuiation increment is fact1+fact2p+fact3p) */
  Word16    weight_lb;   /* attenuation factor */

  Word16    factrho;      
  
  /* coefficient of AR predictive filter A(z) */
  Word16    *a;          /* LPC coefficients */
  Word16    *mem_syn;    /* past synthesis */

  Word16    fecbuf[L_BUFF]; /* for low band delay*/
  Word16    *nooffsig;   /* no offset signal */
  Word16    ferc;
  Word16    pf;
  VAR_MEM   *var_mem;

} LBFEC_STATE;

/***********************************
 * declaration of FERC subroutines *
 ***********************************/

/* lower-band analysis (main subroutine: LBFEC_ana) */
static Word16  LBFEC_classif_modif(LBFEC_STATE * state);

static void    LBFEC_residu(LBFEC_STATE * state);
static void    LBFEC_DC_remove(LBFEC_STATE * state);

/* lower-band synthesis (main subroutine: LBFEC_syn) */
static void    LBFEC_syn(LBFEC_STATE * state, Word16 * syn, Word16 NumSamples);
static Word16  LBFEC_ltp_pred_1s(Word16* exc, Word16 t0, Word16 *jitter);
static void    LBFEC_ltp_syn(LBFEC_STATE* state, Word16* cur_exc, Word16* cur_syn, Word16 n, Word16 *jitter);
static void    LBFEC_syn_filt(Word16 m, Word16* a, Word16* x, Word16* y, Word16 n);
static void    LBFEC_attenuate(LBFEC_STATE * state, Word16 * cur_sig, Word16 * tabout, Word16 NumSamples, Word16 * ind, Word16 * weight);
static void    LBFEC_attenuate_lin(LBFEC_STATE * state, Word16 fact, Word16 * cur_sig, Word16 * tabout, Word16 NumSamples, Word16 * ind, Word16 * weight);

static void    LBFEC_calc_weight(Word16 *ind_weight, Word16 fact1, Word16 fact2p, Word16 fact3p, Word16 * weight);

static void    LBFEC_update_mem_exc(LBFEC_STATE * state, Word16 * cur_sig, Word16 NumSamples);

static Word16  LBFEC_pitch_ol(Word16 * signal, Word16 *maxco);
static void    LBFEC_lpc(LBFEC_STATE * state, Word16 * mem_speech);

static void    Plc_resynchro(Word16  *conceal, Word16  *sp, Word16  clas);
static Word32  Sum_vect_E(const Word16 *vec, const Word16 lvec);
static void    Resample_vector(Word16 *vect, short n);


/************************************
 * definition of main FERC routines *
 ************************************/

/* Calculate energy ratio */
Word16 get_E_ratio(  /* OUT: ratio */
  Word32  vec_E1,    /* IN: energy 1 */
  Word32  vec_E2     /* IN: energy 2 */
){
  Word16  ratio;
  Word32  L_enr1, L_enr2;
  Word32  L_tmp1, L_tmp2;
  Word16  exp1, exp2, exp3;
  Word16  high1, high2;

  /* Calculate Maximum Normalized Correlation */
  L_tmp1 = L_add(vec_E1, 1);
  L_tmp2 = L_add(vec_E2, 1);

  /* ----------- Calculate Energy Ratio Part ----------- */

  L_enr1 = L_min(L_tmp1, L_tmp2); /* Get Numer (must be the smallest for div_s) */
  L_enr2 = L_max(L_tmp1, L_tmp2); /* Get Denom (must be the  highest for div_s) */

  /* Get the Exponents */
  exp1 = norm_l(L_enr1);
  exp2 = norm_l(L_enr2);

  /* Calculate Total Exponent of L_enr1 * L_enr2 (will be used later) */
  exp3 = add(exp1, exp2);

  /* Normalize Numer & Denom */
  L_enr1 = L_shl(L_enr1, exp1);
  L_enr2 = L_shl(L_enr2, exp2);

  /* Get the Highs Parts (16 Bits Precision) */
  high1 = round(L_enr1);
  high2 = round(L_enr2);

  /* ---- Calculate Ratio of Min/Max ---- */

  /* Calculate 'Exp of Numer' minus 'Exp of Denom' (is always >= 0 because Numer < Denom) */
  exp1 = sub(exp1, exp2);

  /* Make sure Numer < Denom (exp2 set to -1 when L_enr2 < L_enr1) */
  exp2 = shr(extract_h(L_sub(L_enr2, L_enr1)), 15);
  L_tmp1 = L_shl(L_enr1, exp2);

  /* Divide & Apply Exponent (account for the Numer adjustment with exp2) */
  ratio = div_s(round(L_tmp1), high2);
  ratio = shr(ratio, add(exp1, exp2)); /* When exp2 = -1 Right Shift Count is Reduced by 1 */

  return ratio;
}


/* Constructor */
void    *FEC_lowerband_const( Word16 ferc, Word16 pf )
{
  LBFEC_STATE * state;

  /* allocate memory for FERC state */
  state = (LBFEC_STATE *)malloc(sizeof(LBFEC_STATE));
  if (state != NULL)
  {
    state->a = (Word16 *)calloc(ORD_LPC + 1, sizeof(Word16));
    state->mem_syn = (Word16 *)calloc(ORD_LPC, sizeof(Word16));
    /* signal buffers */
    state->mem_speech = (Word16 *)calloc(MEMSPEECH_LEN, sizeof(Word16));
    state->mem_exc = (Word16 *)calloc(MAXPIT2P1, sizeof(Word16));

    state->nooffsig = (Word16 *)calloc(MEMSPEECH_LEN, sizeof(Word16));

    state->pf = pf;
    state->ferc = ferc;
    state->var_mem = NULL;
    if ( pf != 0 ) {
      state->var_mem = (VAR_MEM *)malloc(sizeof(VAR_MEM));
    }
    FEC_lowerband_reset( (void *)state );
  }

  return (void *)state;
}

/* Destructor */
void    FEC_lowerband_dest( void *ptr_work )  /* OUT: state variables of FERC */
{
  LBFEC_STATE *state = (LBFEC_STATE *)ptr_work;
  if ( state != NULL )
  {
    free(state->mem_speech);
    free(state->mem_exc);
    free(state->a);
    free(state->mem_syn);
    free(state->nooffsig);
    if (state->var_mem != NULL) free(state->var_mem);
    free(state);
  }
}

/* Reset */
void    FEC_lowerband_reset( void *ptr_work )  /* OUT: state variables of FERC */
{
  Word16      i;
  LBFEC_STATE *state = (LBFEC_STATE *)ptr_work;

  /* bad frame indicator */
  state->prev_bfi = 0;  move16();

  /* LPC, pitch, signal classification parameters */
  FOR (i = 0; i < ORD_LPC; i++)
  {
    state->mem_syn[i] = (Word16) 0;    move16();
    state->a[i] = (Word16) 0;          move16();
  }
  state->a[ORD_LPC] = (Word16) 0;      move16();
  state->t0 = (Word16) 0;              move16();
  state->clas = LBFEC_WEAKLY_VOICED;   move16();

  FOR (i = 0; i < MEMSPEECH_LEN; i++)
  {
    state->mem_speech[i] = (Word16) 0; move16();
  }
  FOR (i = 0; i < MAXPIT2P1; i++)
  {
    state->mem_exc[i] = (Word16) 0;    move16();
  }

  /* cross-fading counter*/
  state->count_crossfade = CROSSFADELEN; move16(); /*init*/

  /* adaptive muting */
  state->count_att = (Word16) 0; move16();
  state->weight_lb = 32767;      move16();
  state->inc_att = 1;            move16();
  state->fact1 = FACT1_V;        move16();
  state->fact2p = FACT2P_V;      move16();
  state->fact3p = FACT3P_V;      move16();
  state->factrho = 0;            move16();

  zero16( L_BUFF, state->fecbuf );

  IF (state->pf != 0)
  {
    postProc_Init_anaSynth(&(state->var_mem->var_anaSynth));
    postProc_InitGainProcess(&(state->var_mem->var_gain));
  }
}

/*----------------------------------------------------------------------
 * LBFEC_conceal_p1(fec_state, xl, xh, output, decoder)
 * 1st part of extrapolation of 1st erased frame
 *
 * fec_state (i/o) : state variables of FERC
 *---------------------------------------------------------------------- */

void LBFEC_conceal_p1(void * fec_state) /* I/O: state variables of FERC */
{
  LBFEC_STATE * state = (LBFEC_STATE *) fec_state;

 /***********************
  * reset counter       *
  ***********************/

  state->count_crossfade = 0; move16(); /* reset counter for cross-fading */
  state->count_att = 0;       move16(); /* reset counter for attenuation in lower band */
  state->weight_lb = 32767;   move16(); /* reset attenuation weight to 1 (Q15) */

  LBFEC_DC_remove(state); /*remove DC, 50Hz high pass*/

 /**********************************
  * analyze buffer of past samples *
  * - LPC analysis
  * - pitch estimation
  **********************************/


  /* perform 6th order LPC analysis on past valid decoded signal */
  LBFEC_lpc(state, state->nooffsig);
  /* estimate (open-loop) pitch */
  /* attention, may shift noofsig, but only used after for zero crossing rate not 
  influenced by this shift (except for very small values)*/
  state->t0 = LBFEC_pitch_ol(state->nooffsig + MEMSPEECH_LEN - MAXPIT2, &state->maxco); move16();

  return;
}

/*----------------------------------------------------------------------
 * LBFEC_conceal_p2(fec_state, xl, xh, output, decoder)
 * 2nd part of extrapolation of 1st erased frame
 *
 * fec_state (i/o) : state variables of FERC
 * output (o) :      synthesized signal for erased frame
 *---------------------------------------------------------------------- */

void LBFEC_conceal_p2(
  void*   fec_state, /* I/O: state variables of FERC */
  Word16* output     /* OUT: synthesized signal for erased frame */
) {
  LBFEC_STATE * state = (LBFEC_STATE *) fec_state;

  Word32  L_tmp1, L_tmp2;

  /* compute LPC residual signal */
  LBFEC_residu(state);

  /* update memory for LPC
     during ereased period the state->mem_syn contains the non weighted
     synthetised speech memory. For the    first erased frame, it
     should contain the output speech.
     Saves the last ORD_LPC samples of the output signal in
     state->mem_syn    */

  mov16(ORD_LPC, &state->mem_speech[MAXPIT2P1], state->mem_syn);

  /* determine signal classification and modify residual in case of transient */
  state->clas = LBFEC_classif_modif(state); move16();

  L_tmp1 = L_add(Sum_vect_E(&state->mem_speech[MEMSPEECH_LEN - state->t0], state->t0), 1);
  L_tmp2 = L_add(Sum_vect_E(&state->mem_speech[MEMSPEECH_LEN - state->t0*2], state->t0), 1);

  test();
  IF ( L_sub(L_tmp1, L_tmp2) < 0 
      && L_sub(L_shr(L_tmp2, 10), L_mult(state->t0, 360)) > 0)
  {
    state->factrho = sub( 32767, sqrt_q15( get_E_ratio(L_tmp1, L_tmp2) ) );
    state->factrho = mult(state->factrho, div_s(1, state->t0));
  }
  ELSE
  {
    state->factrho = 0;   move16();
  }

 /*****************************
  * synthesize erased samples *
  *****************************/

  /* set increment for attenuation */
  IF (sub(state->clas, LBFEC_TRANSIENT) == 0)
  {
    /* attenuation in 15 ms */
    state->inc_att = 4;         move16(); /* time increment*/
    state->fact1 = FACT1_V_R;   move16(); /* basic increment */
    state->fact2p = FACT2P_V_R; move16(); /* additional increment for 2nd linear part of attenuation function */
    state->fact3p = FACT3P_V_R; move16(); /* additional increment for last linear part of attenuation function */
  }
  ELSE
  {
    /* attenuation in 60 ms */
    state->inc_att = 1;        move16(); /* time increment*/
    state->fact1 = FACT1_V;    move16(); /* basic increment */
    state->fact2p = FACT2P_V;  move16(); /* additional increment for 2nd linear part of attenuation function */
    state->fact3p = FACT3P_V;  move16(); /* additional increment for last linear part of attenuation function */
  }

  /* shift low band memory */
  mov16(MEMSPEECH_LEN_MFRAME,&state->mem_speech[L_FRAME_NB],state->mem_speech); /*shift low band*/

  /* synthesize lost frame (low band)*/
  LBFEC_syn(state, output, L_FRAME_NB); /* xl : mem_speech[len] */

  /* attenuate synthesis filter */
  state->a[1] = round(L_mult(state->a[1],GAMMA_AZ1)); move16();
  state->a[2] = round(L_mult(state->a[2],GAMMA_AZ2)); move16();
  state->a[3] = round(L_mult(state->a[3],GAMMA_AZ3)); move16();
  state->a[4] = round(L_mult(state->a[4],GAMMA_AZ4)); move16();
  state->a[5] = round(L_mult(state->a[5],GAMMA_AZ5)); move16();
  state->a[6] = round(L_mult(state->a[6],GAMMA_AZ6)); move16();

  /* attenuate outputs */
  /*first lost frame : low complexity linear attenuation*/
  LBFEC_attenuate_lin(state, state->fact1, output, output, L_FRAME_NB, &state->count_att, &state->weight_lb);

  return;
}

/*----------------------------------------------------------------------
 * LBFEC_conceal_p3(fec_state, xl, xh, output, decoder)
 * extrapolation of 2nd (and consecutively) erased frames
 *
 * fec_state (i/o) : state variables of FERC
 * output (o) :      synthesized signal for erased frame
 *---------------------------------------------------------------------- */

void LBFEC_conceal_p3(
  void *  fec_state,  /* I/O: state variables of FERC */
  Word16* output      /* OUT: synthesized signal for erased frame */
) {
  LBFEC_STATE * state = (LBFEC_STATE *) fec_state;

  /* synthesize lost frame (low band)*/
  LBFEC_syn(state, output, L_FRAME_NB);
  IF (sub(state->prev_bfi, 1) <= 0)
  { /*2nd lost frame : low complexity linear attenuatino*/
    LBFEC_attenuate_lin(state, state->fact1, output, output, L_FRAME_NB, &state->count_att, &state->weight_lb);
  }
  ELSE
  { /* attenuation, complete version*/
    LBFEC_attenuate(state, output, output, L_FRAME_NB, &state->count_att, &state->weight_lb);
  }
  return;
}


/**********************************
 * definition of FERC subroutines *
 **********************************/

/*-------------------------------------------------------------------------*
 * LBFEC_attenuate(state, in, out, n, count, weight)
 * linear muting with adaptive slope
 *
 * state (i/o) : FERC state variables
 * in    (i)   : input signal
 * out   (o)   : output signal = attenuated input signal
 * n     (i)   : number of samples
 * count (i/o) : counter
 * weight (i/o): muting factor
 *--------------------------------------------------------------------------*/

static void LBFEC_attenuate(
  LBFEC_STATE* state,      /* I/O: state variables of FERC */
  Word16* in,              /* IN: input signal */
  Word16* out,             /* OUT: attenuated signal */
  Word16  n,               /* IN: number of samples */
  Word16* count,           /* I/O: counter of attenuated samples */
  Word16* weight           /* I/O: attenuation factor */
) {
  Word16  i;

  IF (sub(state->prev_bfi, 5) == 0)
  {
    IF (sub(*weight, 30400) < 0)
    {
      test();
      IF (sub(state->clas,LBFEC_VOICED) == 0 
          || sub(state->clas,LBFEC_WEAKLY_VOICED) == 0 )
      {
        state->fact3p = s_max(sub(mult(*weight, 102), 20), 0);     move16();
      }
    }
  }

  FOR (i = 0; i < n; i++)
  {
    /* calculate attenuation factor and multiply */

    LBFEC_calc_weight(count, state->fact1, state->fact2p, state->fact3p, weight);

    out[i] = mult_r(*weight, in[i]);      move16();
    *count = add(*count, state->inc_att); move16();
  }
  return;
}

/*-------------------------------------------------------------------------*
 * LBFEC_attenuate_lin(state, fact, in, out, n, count, weight)
 * linear muting with fixed slope (low complexity)
 *
 * state (i/o) : FERC state variables
 * fact  (i/o) : muting parameter
 * in    (i)   : input signal
 * out   (o)   : output signal = attenuated input signal
 * n     (i)   : number of samples
 * count (i/o) : counter
 * weight (i/o): muting factor
 *--------------------------------------------------------------------------*/

static void LBFEC_attenuate_lin(
  LBFEC_STATE* state,       /* I/O: state variables of FERC */
  Word16  fact,             /* IN: attenuation increment */
  Word16* in,               /* IN: input signal */
  Word16* out,              /* OUT: attenuated signal */
  Word16  n,                /* IN: number of samples */
  Word16* count,            /* I/O: counter of attenuated samples */
  Word16* weight            /* I/O: attenuation factor */
) {
  Word16  i;

  IF (sub(state->factrho, fact) > 0)
  {
    fact = state->factrho; move16();
  }

  FOR (i = 0; i < n; i++)
  {
    /* calculate attenuation factor and multiply */
    *weight = s_max(sub(*weight, fact), 0);    move16();

    out[i] = mult_r(*weight, in[i]);           move16();
  }

  state->factrho = shr(state->factrho, 1);     move16();

  *count = add(*count, i_mult(state->inc_att, n)); move16();
  return;
}

/*-------------------------------------------------------------------------*
 * LBFEC_calc_weight(ind_weight,fact1, fact2, fact3, weight)
 *                     
 * calculate attenuation factor
 *--------------------------------------------------------------------------*/

static void LBFEC_calc_weight(
  Word16 *ind_weight, /* I/O: counter of attenuated samples */
  Word16 fact1,       /* IN: increment for attenuation factor, first slope */
  Word16 fact2p,      /* IN: additional increment for 2nd slope */
  Word16 fact3p,      /* IN: additional increment for 3rd slope */
  Word16 *weight      /* OUT: computed attenuation factor */
) {

  *weight = sub(*weight, fact1);    /* increment composant for all slopes */
  if (sub(*ind_weight, END_1ST_PART) >= 0)
  {
    *weight = sub(*weight, fact2p); /* additional increment for 2nd slope */
  }
  if (sub(*ind_weight, END_2ND_PART) >= 0)
  {
    *weight = sub(*weight, fact3p); /* additional increment for 3rd slope */
  }
  if (sub(*ind_weight, END_3RD_PART) >= 0)
  {
    *weight = 0; move16();          /* complete muting */
  }
  if (*weight < 0) /*for security*/
  {
    *weight = 0;    move16();        /* complete muting */
  }
  if (*weight == 0) /* when complete muting is achived*/
  {
    *ind_weight = END_3RD_PART; move16(); /*stop counter (to avoid overflow) */
  }
  return;
}

/*--------------------------------------------------------------------------*
 * Function LBFEC_update_mem_exc                                            *
 * Update of state->mem_exc and shifts the memory                           *
 * if state->t0 > L_FRAME_NB                                                *
 *--------------------------------------------------------------------------*/

static void LBFEC_update_mem_exc(
  LBFEC_STATE * state,      /* I/O: state variables of FERC */
  Word16 * exc,             /* I: excitation signal to memorise*/
  Word16  n                 /* length of excitaion signal (frame length) */
) {
  Word16  *ptr;
  Word16  temp;
  Word16  lag;

  /* shift ResMem, if t0 > l_frame */
  lag = add(state->t0, T0_SAVEPLUS); /*update temp samples*/
  temp = sub(lag, n);

  ptr = state->mem_exc + sub(MAXPIT2P1, lag);
  IF (temp > 0) /*shift needed*/
  {
    mov16(temp,&ptr[n],ptr);
    mov16(n, exc, &ptr[temp]);
  }
  ELSE
  {
    /* copy last "pitch cycle" of residual */
    mov16(lag, &exc[sub(n, lag)], ptr);
  }
  return;
}

/* 50 Hz higgh pass filter to remove DC component*/
static void LBFEC_DC_remove(LBFEC_STATE * state)  /* I/O: state variables of FEC */
{
  int     i;
  Word16  tmp1;

  /*offset removing for pitch estimation input*/
  state->nooffsig[0] = state->mem_speech[0]; move16();                        

  FOR (i = 1; i < MEMSPEECH_LEN; i++)
  {
    tmp1 = sub(mult_r(state->nooffsig[i-1], 31784), state->mem_speech[i-1]); /*31784 0.97 en Q15*/
    state->nooffsig[i] = add(state->mem_speech[i], tmp1); move16();
  }
}


/*----------------------------------------------------------------------
 * LBFEC_pitch_ol(signal, maxco)
 * open-loop pitch estimation
 *
 * signal      (i) : pointer to signal buffer (including signal memory)
 * maxco       (o) : maximal correlation
 *
 *---------------------------------------------------------------------- */

static Word16 LBFEC_pitch_ol(  /* OUT: repetition period (pitch lag) */
  Word16 *signal,    /* IN: pointer to signal buffer (including signal memory) */
  Word16 *maxco)     /* OUT: maximal correlation */
{  
  Word16  i, j, il, k; 
  Word16  ind, ind2;
  Word16  w_ds_sig[MAXPIT2_DS];
  Word32  corx_f, ener1_f, ener2_f;
  Word32  temp_f;
  Word16  valid = 0; /*not valid for the first lobe */
  Word16  start_ind, end_ind, beg_last_per, end_last_per;
  Word16  e1, e2, co, em, norm_e, ne1;
  Word32  ener1n, ener2n;
  Word16  *ptr1, *nooffsigptr;
  Word32  L_temp; 
  Word16  maxco_s8, stable;
  Word16  overfl_shft;
  Word16  ds_sig[MAXPIT2_DS];
  Word16  ai[3] = {4096,0,0};
  Word16  cor_h[3], cor_l[3], rc[3];
  Word16  *pt1, *pt2;
  Word16  zcr;

  nooffsigptr = signal; move16();

  /* downsample (filter and decimate) signal by factor 4*/
  ptr1 = ds_sig;
  FOR (i = FACT_M1; i < MAXPIT2; i += FACT)
  {
    temp_f = L_mult0(nooffsigptr[i], LBFEC_fir_lp[0]);
    FOR (k = 1; k < FEC_L_FIR_FILTER_LTP; k++)
    {
      temp_f = L_mac0(temp_f, nooffsigptr[sub(i, k)], LBFEC_fir_lp[k]);
    }
    *ptr1++ = round(temp_f); move16();
  }
  /*compute the autocorrelation function*/
  Autocorr(ds_sig, &LBFEC_lpc_win_80[HAMWINDLEN-MAXPIT2_DS], cor_h, cor_l, 2, MAXPIT2_DS);
  /* length = 72 samples, uses the end of the window of 80 samples*/

  /*60 Hz bandwidth expansion + 40 dB noise floor */
  Lag_window(cor_h, cor_l, LBFEC_lag_h, LBFEC_lag_l, 2);    
  /* Levinson to compute the 2nd ordre LPC filter*/
  Levinson(cor_h, cor_l, rc, &stable, 2, ai);

  /* weighting by 0.94*/
  ai[1] = round(L_mult(ai[1],GAMMA));  move16();
  ai[2] = round(L_mult(ai[2],GAMMA2)); move16();

  /* filtering to obtaine the weighted signal */
  w_ds_sig[0] = ds_sig[0]; move16();

  L_temp = L_mult(ai[1], ds_sig[0]);
  w_ds_sig[1] = add(ds_sig[1], round(L_shl(L_temp,3))); move16();

  FOR (i = 2; i < MAXPIT2_DS; i++)
  {
    L_temp = L_mult(ai[1], ds_sig[i - 1]);
    L_temp = L_mac(L_temp, ai[2], ds_sig[i - 2]);
    w_ds_sig[i] = add(ds_sig[i], round(L_shl(L_temp,3))); move16();
  }

  ind = MAXPIT_S2_DS; move16(); /*default value*/
  ind2 = 0; /* initialise counter for multiple pitch*/

  /*Test overflow on w_ds_sig*/
  overfl_shft = 0; move16();

  /* compute energy of signal in range [len/fac-1,(len-MAX_PIT)/fac-1] */
  ener1_f = 1; move32();
  FOR (j = MAXPIT2_DSM1; j >= MAXPIT_DSP1; j--)
  {
    ener1_f = L_mac0(ener1_f, w_ds_sig[j], w_ds_sig[j]);
  }

  /* compute exponent */
  ne1 = norm_l(ener1_f);

  /* compute maximal correlation (maxco) and pitch lag (ind) */
  *maxco = 0; move16();
  ener2_f = L_msu0(ener1_f, w_ds_sig[MAXPIT2_DSM1], w_ds_sig[MAXPIT2_DSM1]); /*update, part 1*/

  /* search first zero-crossing */
  pt1 = &w_ds_sig[MAXPIT2_DSM1];
  pt2 = pt1-1;
  zcr = 0; move16();
  j = 2;   move16();

  test();
  WHILE(zcr == 0 && sub(j, MAXPIT2_DSM1) < 0)
  {
    if (s_xor(*pt1, *pt2) < 0)
    {
      zcr = j; move16();
    }
    pt1--;
    pt2--;
    j = add(j, 1);
  }

  FOR (i = 1; i < MAXPIT_DS; i++) /* < to avoid out of range later*/
  {
    ind2 = add(ind2, 1); /* update counter for multiple pitch*/
    corx_f = 0; move32();

    FOR (j = MAXPIT2_DSM1; j >= MAXPIT_DSP1; j--)
    {
      corx_f = L_mac0(corx_f, w_ds_sig[j], w_ds_sig[j-i]);
    }
    ener2_f = L_mac0(ener2_f, w_ds_sig[MAXPIT_DSP1-i], w_ds_sig[MAXPIT_DSP1-i]); /*update, part 2*/
    norm_e = s_min(ne1, norm_l(ener2_f));
    ener1n = L_shl(ener1_f, norm_e);
    ener2n = L_shl(ener2_f, norm_e);
    corx_f = L_shl(corx_f, norm_e);
    e1 = round(ener1n);
    e2 = round(ener2n);
    if (L_sub(0x7fffffff, ener2_f) > 0) /* not saturated*/ /* BUG correction*/
    {
      ener2_f = L_msu0(ener2_f, w_ds_sig[MAXPIT2_DSM1-i], w_ds_sig[MAXPIT2_DSM1-i]);         /*update, part 1*/
    }
    co = round(corx_f);
    em = s_max(e1, e2);
    if (co > 0)    
    {
      co = div_s(co, em);
    }

    if (co < 0) 
    {
      valid = 1; move16(); 
      /* maximum correlation is only searched after the first positive lobe of autocorrelation function*/
    }
    if (sub(i, zcr) < 0) /*pitch must be grater then zcr (at least one zero crossing in a pitch period)*/
    {
      valid = 0; move16();
    }
    IF (sub(valid,1) == 0)
    {
      test();
      IF ((sub(ind2, ind) == 0) || (sub(ind2, shl(ind,1)) == 0)) /* double or triple of actual pitch */
      {
        if (sub(*maxco, 27850) > 0) /* 0.85 : high correlation, small chance that double pitch is OK*/
        {
          *maxco = 32767; move16(); /* the already found pitch value is kept*/
        }

        /*increase the already found maxco to avoid choosing the double pitch*/
        maxco_s8 = shr(*maxco, 3);
        if (sub(*maxco, 29126) < 0)/*to avoid overflow*/
        {
          *maxco = add(*maxco, maxco_s8); 
        }
      }

      test();
      IF ((sub(co, *maxco) > 0) && (sub(i, MINPIT_DS) >= 0))
      {
        *maxco = co;
        ind = i;  move16(); /*save the new candidate*/
        ind2 = 1; move16(); /* reset counter for multiple pitch*/
      }
    }
  }

  /* convert pitch to non decimated domain */    
  il = shl(ind, FACTLOG2);
  ind = il; move16();

  /* shift DC-removed signal to avoid overflow in correlation */
  if (L_sub(ener1_f, 0x01000000) > 0) /* maxcor will be computed on 4* points in non weighted domain --> overflow risq*/
  {
    overfl_shft = add(overfl_shft,1);
  }

  IF (overfl_shft > 0)
  {
    FOR (i = 1; i < MAXPIT2; i++)
    {
      nooffsigptr[i] = shr(nooffsigptr[i], overfl_shft); move16();
    }
  }

  /* refine pitch in non-decimated (8 kHz) domain by step of 1
     -> maximize correlation around estimated pitch lag (ind) */
  start_ind = sub(il, 2);
  start_ind = s_max(start_ind, MINPIT);
  end_ind = add(il, 2);
  beg_last_per = sub(MAXPIT2, il);
  end_last_per = sub(MAXPIT2, 1);
  j = sub(end_last_per, start_ind);
  ener1_f = L_mac0(1, nooffsigptr[end_last_per], nooffsigptr[end_last_per]); /*to avoid division by 0*/
  ener2_f = L_mac0(1, nooffsigptr[j], nooffsigptr[j]); /*to avoid division by 0*/
  FOR (j = sub(end_last_per, 1); j > beg_last_per; j--)
  {
    ener1_f = L_mac0(ener1_f, nooffsigptr[j], nooffsigptr[j]);
    ener2_f = L_mac0(ener2_f, nooffsigptr[j-start_ind], nooffsigptr[j-start_ind]);
  }
  ener1_f = L_mac0(ener1_f, nooffsigptr[j], nooffsigptr[j]); /*last point for ener1, for ener2 it is done in update 2*/
  /* compute exponent */
  ne1 = norm_l(ener1_f);
  /* compute maximal correlation (maxco) and pitch lag (ind) */
  *maxco = 0; move16();

  FOR (i = start_ind; i <= end_ind; i++)
  {
    corx_f = 0; move32();

    ener2_f = L_mac0(ener2_f, nooffsigptr[beg_last_per-i], nooffsigptr[beg_last_per-i]); /*update, part 2*/
    FOR (j = end_last_per; j >= beg_last_per; j--)
    {
      corx_f    = L_mac0(corx_f, nooffsigptr[j], nooffsigptr[j-i]);
    }
    norm_e = s_min(ne1, norm_l(ener2_f));
    ener1n = L_shl(ener1_f, norm_e);
    ener2n = L_shl(ener2_f, norm_e);
    corx_f = L_shl(corx_f, norm_e);
    e1 = round(ener1n);
    e2 = round(ener2n);
    co = round(corx_f);
    em = s_max(e1, e2);
    if (co > 0)    
    {
      co = div_s(co, em);
    }
    if (sub(co, *maxco) > 0)
    {
      ind = i; move16();
    }
    *maxco = s_max(co, *maxco);
    if (L_sub(0x7fffffff, ener2_f) > 0) /* not saturated*/ 
    {
      ener2_f = L_msu0(ener2_f, nooffsigptr[end_last_per-i], nooffsigptr[end_last_per-i]);         /*update, part 1*/
    }
  }
  IF (sub(*maxco, 8192) < 0) /*very weakly or unvoiced signal*/
  {
    if (sub(ind, 32) < 0)
    {
      ind = shl(ind,1); /*repetition period = 2 times pitch for very small pitch, at least 2 times MINPIT */
    }
  }

  return ind;
}


/*----------------------------------------------------------------------
 * LBFEC_classif_modif(maxco)
 * signal classification and conditional residual modification
 *
 *---------------------------------------------------------------------- */

static Word16 LBFEC_classif_modif(
  LBFEC_STATE* state   /* I/O: state variables of FERC */
)
{
  Word16  clas, Temp, tmp1, tmp2, tmp3, tmp4, i, maxres, absres, zcr;
  Word16  *pt1, *pt2;

  Word16  abs_mem_exc[MAXPIT+4];
  Word16  *mem_exc;
  Word16  maxpulse, pulseind=0;
  Word16  mincheck;
  Word16  end2nd;
  Word16  maxpulse2nd, pulseind2nd=0;
  Word16  absval;
  Word32  cumul, pulsecumul;
  Word16  signbit, signbit2nd;

  mem_exc = state->mem_exc; move16();

 /************************************************************************
  * select preliminary class => clas = UNVOICED, WEAKLY_VOICED or VOICED *
  * by default clas=WEAKLY_VOICED                                        *
  * classification criterio:                                             *
  * -normalized correlation from open-loop pitch                         *
  * -zero crossing rate                                                  *
  ************************************************************************/

  /* compute zero-crossing (from + to -) rate in last 5 ms */
  pt1 = &state->nooffsig[sub(MEMSPEECH_LEN, 80)];
  pt2 = pt1-1;
  zcr = 0; move16();

  FOR (i = 0; i< 80; i++)
  {
    Temp = 0; move16();

    if (*pt1 <= 0)
    {
      Temp = 1; move16();
    }
    if (*pt2 > 0)
    {
      Temp = add(Temp,1);
    }

    zcr = add(zcr, shr(Temp,1));
    pt1++;
    pt2++;
  }

  /* set default clas to weakly voiced*/
  clas = LBFEC_WEAKLY_VOICED; move16();

  /* detect voiced clas (cmaximum correlation > 0.7) */
  if (sub(state->maxco, 22936) > 0) /* 22936 in Q15 = 0.7 */
  {
    clas = LBFEC_VOICED; move16();
  }

  /* change class to unvoiced if zcr is high */
  IF (sub(zcr,20)>=0)
  {
    clas = LBFEC_UNVOICED; move16();
    /* change repetition period if unvoiced class (to avoid too short pitch lags) */
    if (sub(state->t0, 32) < 0)
    {
      state->t0 = shl(state->t0,1); /*2 times pitch for very small pitch, at least 2 times MINPIT */
    }
  }

 /**************************************************************************
  * detect transient => clas = TRANSIENT                                   *
  * + modify residual to limit amplitude for LTP                           *
  * (this is performed only if current class is WEAKLY VOICED to avoid     *
  *  perturbation of the residual for LTP in case of VOICED class)         *
  *  UNVOICED class is smoothed differently, see below                     *
  **************************************************************************/

  /* detect transient and limit amplitude of residual */
  Temp = 0;
  IF (sub(clas,5) == 0) /*LBFEC_WEAKLY_VOICED(5) */
  {
    tmp1 = sub(MAXPIT2P1, state->t0); /* tmp1 = start index of last "pitch cycle" */
    tmp2 = sub(tmp1, state->t0);  /* tmp2 = start index of last but one "pitch cycle" */
    tmp3 = sub(tmp2, 2);   /* neighbourhood in the previous period: +-2*/
    tmp4 = add(state->t0, 4); 
    /*pre_compute abs values */
    FOR (i = 0; i < tmp4; i++)
    {
      abs_mem_exc[i] = abs_s(mem_exc[tmp3]);
      tmp3++;  /*address*/
    }

    FOR (i = 0; i < state->t0; i++)
    {
      /*maxres : maximum residual amplitude around the same position (+-2) in the previous pitch period */
      maxres = s_max(s_max(abs_mem_exc[i],     abs_mem_exc[i+1]), 
                     s_max(abs_mem_exc[i+2],   abs_mem_exc[i+3]));
      maxres = s_max(abs_mem_exc[i+4], maxres);
      absres = abs_s(mem_exc[tmp1 + i]);

      /* if magnitude in last cycle > magnitude in last but one cycle */
      IF (sub(absres, maxres) > 0)
      {
        /* detect  and count transient (ratio >= 1/8) */
        tmp4 = shr(absres,3);
        if (sub(tmp4, maxres) >= 0) 
        {
          Temp = add(Temp, 1);
        }

        /* limit the amplitude of the repetition period (even if a transient is not detected...) */
        tmp4 = negate(maxres);
        if (mem_exc[tmp1 + i] < 0)
        {
          mem_exc[tmp1 + i] = tmp4; move16();
        }
        if (mem_exc[tmp1 + i] >= 0)
        {
          mem_exc[tmp1 + i] = maxres; move16();
        }
      }
    }
  }

  IF (sub(clas,1) == 0) /*in case of unvoiced class*/
  {
    Word32 mean;
    Word16 smean;

    mean = 0; move32();

    tmp1 = sub(MAXPIT2P1, 80); /* tmp1 = start index of last 5 ms, last period is smoothed */
    FOR (i = 0; i < 80; i++)
    {
      mean = L_mac0(mean, abs_s(mem_exc[tmp1 + i]), 1);
    }
    mean = L_shr(mean, 5);  /*80/32 = "mean" contains 2.5 * mean amplitude*/
    smean = extract_l(mean);

    tmp1 = sub(MAXPIT2P1, state->t0); /* tmp1 = start index of last "pitch cycle" */
    FOR (i = 0; i < state->t0; i++)
    {
      tmp4 = shr(mem_exc[tmp1 + i], 2);
      if (sub(abs_s(mem_exc[tmp1 + i]), smean) > 0) /* if current amplitude < 2.5 mean amplitude */
      {
        mem_exc[tmp1 + i] = tmp4;    move16(); /* current amplitude is divided by 4 */
      }
    }
  }
  IF (Temp>0) /*if at least one transient is detected */
  {
    clas = LBFEC_TRANSIENT;
    state->t0 = s_min(state->t0,40); move16(); /*max 5 ms pitch */
  }

 /*******************************************************************************
  * pitch tuning by searching last glotal pulses in case of VOICED class        *
  * checks if there is no 2 pulses in the last periode due to pitch decreasing  *
  *******************************************************************************/
  IF (sub(clas, LBFEC_VOICED) == 0)
  {
    mincheck = sub(state->t0,5); /*max pitch variation searched is +-5 */
    maxpulse = -1;    move16();
    maxpulse2nd = -1; move16();
    cumul = 0;        move32();
    pulsecumul = 0;   move32();

    pt1 = &mem_exc[MAXPIT2]; /*check the last period*/
    FOR (i = 0; i < state->t0; i++)
    {
      absval = abs_s(*pt1);
      if (sub(absval, maxpulse) > 0)
      {
        pulseind = i; move16(); /* index of the biggest pulse*/
      }
      maxpulse = s_max(absval, maxpulse); /* amplitude of the biggest pulse*/
      cumul = L_mac0(cumul, absval, 1); /* to compute the mean amplitude */
      pt1--;
    }
    pulsecumul = L_mult0(maxpulse, state->t0);
    signbit = s_and(mem_exc[sub(MAXPIT2, pulseind)],(Word16)0x8000); /*sign of the biggest pulse*/

    IF (L_sub(cumul, L_shr(pulsecumul,2)) < 0) /* if max amplitude > 4*mean amplitude --> real pulse*/
    {
      end2nd = sub(pulseind, mincheck);
      IF (end2nd >= 0) /*biggest pulse at the beggining of the last period*/
      {
        pt1 = &mem_exc[MAXPIT2]; /*end of excitation*/
        FOR (i = 0; i < end2nd; i++) /*search 2nd pulse at the end of the period*/
        {
          absval = abs_s(*pt1);
          if (sub(absval, maxpulse2nd) > 0) 
          {
            pulseind2nd = i; move16(); /* index of the 2nd biggest pulse*/
          }
          maxpulse2nd = s_max(absval, maxpulse2nd); /* amplitude of the 2nd biggest pulse*/
          pt1--;
        }
      }
      IF (sub(pulseind, 5) < 0) /*biggest pulse at the end of the last period*/
      {
        end2nd = sub(state->t0,5);
        end2nd = add(end2nd,pulseind);
        pt1 = &mem_exc[sub(MAXPIT2, end2nd)]; /*end of excitation*/
        FOR (i = end2nd; i < state->t0; i++) /*search 2nd pulse at the beggining of the period*/
        {
          absval = abs_s(*pt1);
          if (sub(absval, maxpulse2nd) > 0)
          {
            pulseind2nd = i; move16(); /* index of the 2nd biggest pulse*/
          }
          maxpulse2nd = s_max(absval, maxpulse2nd); /* amplitude of the 2nd biggest pulse*/
          pt1--;
        }
      }
      /* if the amplitude of the 2nd biggest pulse is at least the half of the biggest pulse*/
      IF (sub(maxpulse2nd, shr(maxpulse,1)) > 0)
      {
        signbit2nd = s_and(mem_exc[sub(MAXPIT2, pulseind2nd)],(Word16)0x8000); /*sign of the 2nd biggest pulse*/
        IF (s_xor(signbit, signbit2nd) == 0) /*if signes are  identical*/
        {
          /*the new pitch value is the distance betwwen the two pulses*/
          state->t0 = abs_s(sub(pulseind,pulseind2nd)); move16(); 
        }
      }
    }
  }

  return clas;
}


/*----------------------------------------------------------------------
 * LBFEC_syn_filt(m, a, x, y, n n)
 * LPC synthesis filter
 *
 * m (i) : LPC order
 * a (i) : LPC coefficients
 * x (i) : input buffer
 * y (o) : output buffer
 * n (i) : number of samples
 *---------------------------------------------------------------------- */

static void LBFEC_syn_filt(
  Word16  m,  /* IN:  LPC order*/
  Word16* a,  /* IN:  LPC coefficients (Q12) */
  Word16* x,  /* IN:  input buffer (Q15) */
  Word16* y,  /* OUT: output buffer (Q15) */
  Word16  n   /* IN:  number of samples*/
) {
  Word32  L_temp;
  Word16  j;

  L_temp = L_mult(a[0], *x);  /* Q28= Q12 * Q15 * 2 */
  FOR (j = 1; j <= m; j++)
  {
    L_temp = L_msu(L_temp, a[j], y[-j]); /* Q28= Q12 * Q15 * 2 */
  }
  *y = round(L_shl(L_temp, 3)); move16(); /*Q15*/

  return;
}


/*----------------------------------------------------------------------
 * LBFEC_ltp_pred_1s(cur_exc, t0, jitter)
 * one-step LTP prediction and jitter update
 *
 * exc     (i)   : excitation buffer (exc[...-1] correspond to past)
 * t0      (i)   : pitch lag
 * jitter  (i/o) : pitch lag jitter
 *---------------------------------------------------------------------- */

static Word16 LBFEC_ltp_pred_1s(
  Word16* exc,    /* IN:  excitation buffer */
  Word16  t0,     /* IN:  repetition period (pitch lag) */
  Word16* jitter  /* I/O: pitch lag jitter (+-1 or 0) */
) {
  Word16  i;

  i = sub(*jitter, t0);

  /* update jitter for next sample */
  *jitter = negate(*jitter);

  /* prediction =  exc[-t0+jitter] */
  return exc[i];
}


/*----------------------------------------------------------------------
 * LBFEC_ltp_syn(state, cur_exc, cur_syn, n, jitter)
 * LTP prediction followed by LPC synthesis filter
 *
 * state    (i/o) : FERC state variables
 * cur_exc  (i)   : pointer to current excitation sample (cur_exc[...-1] correspond to past)
 * cur_syn  (i/o) : pointer to current synthesis sample
 * n     (i)      : number of samples
 * jitter  (i/o)  : pitch lag jitter
 *---------------------------------------------------------------------- */

static void LBFEC_ltp_syn(
  LBFEC_STATE* state,      /* I/O: state variables of FERC */
  Word16* cur_exc,         /* IN:  excitation buffer */
  Word16* cur_syn,         /* I/O: synthetised signal buffer */
  Word16  n,               /* IN:  number of samples to synthetise */
  Word16* jitter           /* I/O: pitch lag jitter (+-1 or 0) */
) {
  Word16  i;

  FOR (i = 0; i < n; i++)
  {
    /* LTP prediction using exc[...-1] */
    *cur_exc = LBFEC_ltp_pred_1s(cur_exc, state->t0, jitter); move16();

    /* LPC synthesis filter (generate one sample) */
    LBFEC_syn_filt(ORD_LPC, state->a, cur_exc, cur_syn, 1);

    cur_exc++;
    cur_syn++;
  } 

  return;
}


/*----------------------------------------------------------------------
 * LBFEC_syn(state, syn, n)
 * extrapolate erased lower-band signal (FERC)
 *
 * state (i/o) : FERC state variables
 * syn   (o)   : synthesis
 * n     (i)   : number of samples
 *---------------------------------------------------------------------- */

static void LBFEC_syn(
  LBFEC_STATE * state,  /* I/O: state variables of FERC */
  Word16 * syn,         /* OUT: synthetised signal buffer */
  Word16  n             /* IN:  number of samples to synthetise */
) {
  Word16  buffer_syn[2*ORD_LPC]; /* synthesis buffer */
  Word16  buffer_exc[MAXPIT+T0_SAVEPLUS+L_FRAME_NB]; /* excitation buffer */
  Word16  *cur_syn;     /* pointer to current sample of synthesis */
  Word16  *cur_exc;     /* pointer to current sample of excition */
  Word16  *exc;         /* pointer to beginning of excitation in current frame */
  Word16  temp;
  Word16  jitter;

  cur_exc = &buffer_exc[add(state->t0, T0_SAVEPLUS)];
  cur_syn = &buffer_syn[ORD_LPC];

  exc = cur_exc;

  /* copy memory
   - past samples of synthesis (LPC order)            -> buffer_syn[0]
   - last "pitch cycle" of excitation (t0+t0SavePlus) -> buffer_exc[0]
  */
  mov16(ORD_LPC, state->mem_syn, buffer_syn); /* load memory part of synthetised signal  */

  temp = add(state->t0, T0_SAVEPLUS);
  mov16(temp, state->mem_exc + sub(MAXPIT2P1, temp), buffer_exc);/* load memory part of exitation signal  */

 /***************************************************
  * set pitch jitter according to clas information  *
  ***************************************************/


  jitter = s_and(state->clas, 1); /*0 (not activated) for VOICED class */  
  state->t0 = s_or(state->t0, jitter);    /* change even delay as jitter is more efficient for odd delays */

 /*****************************************************
  * generate signal by LTP prediction + LPC synthesis *
  *****************************************************/

  temp = sub(n, ORD_LPC);
  /* first samples [0...ORD_LPC-1] */
  LBFEC_ltp_syn(state, cur_exc, cur_syn, ORD_LPC, &jitter);
  mov16(ORD_LPC, cur_syn, syn);

  /* remaining samples [ORD_LPC...n-1] */
  LBFEC_ltp_syn(state, &cur_exc[ORD_LPC], &syn[ORD_LPC], temp, &jitter);

  /* update memory:
   - synthesis for next frame (last LPC-order samples)
   - excitation */
  mov16(ORD_LPC, &syn[temp], state->mem_syn);
  LBFEC_update_mem_exc(state, exc, n);

  return;
}


/*--------------------------------------------------------------------------*
 * Function LBFEC_lpc                                                       *
 * LPC analysis                                                             *
 *--------------------------------------------------------------------------*/

static void LBFEC_lpc(
  LBFEC_STATE * state,      /* I/O: state variables of FERC */
  Word16 * mem_speech       /* IN: past decoded speech */
) {
  Word16  tmp;
  Word16  cor_h[ORD_LPC + 1];
  Word16  cor_l[ORD_LPC + 1];
  Word16  rc[ORD_LPC + 1];

  /* compute the autocorrelation function */
  Autocorr(&mem_speech[MEMSPEECH_LEN - HAMWINDLEN], LBFEC_lpc_win_80, cor_h, cor_l,
      ORD_LPC, HAMWINDLEN);

  /* Lag windowing    */
  /*60 Hz bandwidth expansion + 40 dB noise floor */
  Lag_window(cor_h, cor_l, LBFEC_lag_h, LBFEC_lag_l, ORD_LPC);

  /* Levinson to compute the LPC filter*/
  Levinson(cor_h, cor_l, rc, &tmp, ORD_LPC, state->a);

  return;
}


/*-------------------------------------------------------------------------*
 * Function LBFEC_residu                                                *
 * compute the past excitation signal by incers LPC filtering               *
 *--------------------------------------------------------------------------*/

static void LBFEC_residu(LBFEC_STATE * state) /* I/O: state variables of FERC */
{
  Word32  L_temp;
  Word16  *ptr_sig, *ptr_res;
  Word16  i, j;

  ptr_res = state->mem_exc; /* residual to compute */
  ptr_sig = &state->mem_speech[MEMSPEECH_LEN - MAXPIT2P1];  /*past decoded signal */

  FOR (i = 0; i < MAXPIT2P1; i++)
  {
    L_temp = L_mult(ptr_sig[i], state->a[0]);
    FOR (j = 1; j <= ORD_LPC; j++)
    {
      L_temp = L_mac(L_temp, state->a[j], ptr_sig[i - j]);
    }
    L_temp = L_shl(L_temp, 3);    /* Q28 -> Q31 */
    ptr_res[i] = round(L_temp); move16();/*Q31 -> Q15 */
  }

  return;
}

/* Frame-erasure concealment procedure + dealy line for lower band */

void  FEC_lowerband(
  int     loss_flag,   /* IN: indicate lost frame */
  Word16  *sigin,      /* IN: input decoded signal (in case of valid frame)*/
  Word16  *sigout,     /* OUT: final lower band output signal */
  void    *ptr_work    /* I/O: state variables of FERC */
) {
  Word16  *spch, *spchnew;
  Word16  weight;
  Word16  i;
  Word16  crossfade_buf[CROSSFADELEN];
  Word16  anaflag;
  Word16  *spchnew_post;

  LBFEC_STATE *state = (LBFEC_STATE *)ptr_work;

  Word16  sp[L_FRAME_NB], rsx_buff[2*L_FRAME_NB];

  spch    = &state->fecbuf[L_BUFF-L_FRAME_NB-L_AHEAD]; /* future output to realise L_AHEAD delay*/
  spchnew = &state->fecbuf[L_BUFF-L_FRAME_NB];         /* for "current" frame*/

  spchnew_post = spchnew - L_FLT_DIV2;
  anaflag = 1;  move16();

  IF (state->ferc)
  {
    IF (sub(state->prev_bfi, 1) == 0) /*finish 1st erased frame concealment */
    {
      LBFEC_conceal_p2(ptr_work, spchnew); /* 2nd part of the 1st erased frame concealment */
      mov16((short)L_FRAME_NB, spchnew, &(state->mem_speech[MEMSPEECH_LEN_MFRAME])); /*save 5 ms*/ 

      anaflag = 0;  move16();
      /* call post-processing, effective when post-procesing is activated in command line*/
#ifdef WMOPS
    setCounter(Idpf);
#endif
      postProc_Processing(spchnew, spchnew_post, state->var_mem,
                          state->pf, anaflag,
                          &(state->mem_speech[MEMSPEECH_LEN_MFRAME-L_FLT_DIV2]));
#ifdef WMOPS
    setCounter(Idferc);
#endif
      mov16( L_BUFF-L_FRAME_NB, &state->fecbuf[L_FRAME_NB], &state->fecbuf[0] ); /*shift memory*/
    }
    IF (sub(loss_flag, NO_FRAME_LOSS) == 0)    /* Current frame not lost */
    {
      /* shift speech buffers */
      mov16(MEMSPEECH_LEN_MFRAME, &state->mem_speech[L_FRAME_NB],state->mem_speech); /*shift 5 ms*/
      i = 0; move16();

      IF (state->count_crossfade == 0) /* first good frame, crossfade is needed*/
      {
        /*FERC synthesize a new frame in parallel with the valid one */
        LBFEC_conceal_p3(ptr_work, crossfade_buf); 

        /* prepare the buffers */
        mov16(L_FRAME_NB, spch, rsx_buff);
        mov16(L_FRAME_NB, crossfade_buf, rsx_buff + L_FRAME_NB);
        mov16(L_FRAME_NB, sigin, sp);

        /* do the resynchro */
        Plc_resynchro(rsx_buff, sp, state->clas);

        /* copy back the resynchronized concealement */
        mov16(L_FRAME_NB, rsx_buff, spch);
        mov16(L_FRAME_NB, rsx_buff, &state->mem_speech[MEMSPEECH_LEN_MFRAME-L_FRAME_NB]);
        mov16(L_FRAME_NB, rsx_buff + L_FRAME_NB, crossfade_buf);

        weight = 819; move16(); /*32768/40*/

        FOR (i = 0; i < CROSSFADELEN; i++)
        {
          /* cross-fade samples with FERC synthesis (in lower band only) */
          spchnew[i] = add(mult(sigin[i], weight), mult(crossfade_buf[i], sub(32767, weight)));
          weight = add(weight, 819);
        }

        state->count_crossfade = CROSSFADELEN; move16();
      }
      FOR (; i < L_FRAME_NB; i++) /*no crossfade is needed */
      {
        /* simple copy of valid input */
        spchnew[i] = sigin[i]; move16();
      }
      mov16((short)L_FRAME_NB, spchnew, &(state->mem_speech[MEMSPEECH_LEN_MFRAME])); /*save 5 ms*/ 
      state->prev_bfi = 0; move16(); /* set previous bfi to good frame */
    }
    ELSE    /* Current frame lost */
    {
      IF (state->prev_bfi == 0) /*previous frame was valid : first lost frame */
      {
        /* call first part  of the 1st erased frame concealment (no output samples yet) */
        LBFEC_conceal_p1(ptr_work);
        state->prev_bfi = 1; move16(); /* set previous bfi to first bad frame*/
      }
      ELSE
      {
        mov16(MEMSPEECH_LEN_MFRAME,&state->mem_speech[L_FRAME_NB],state->mem_speech); /*shift*/
        /* FERC synthesize a new frame */
        LBFEC_conceal_p3(ptr_work, spchnew);

        state->prev_bfi = add(state->prev_bfi, 1); /* update bad frame counter*/    

        mov16((short)L_FRAME_NB, spchnew, &(state->mem_speech[MEMSPEECH_LEN_MFRAME])); /*save 5 ms*/ 
      }
    }

    IF (sub(state->prev_bfi, 1) != 0) /*1st erased frame concealment to finish if == 1*/
    {
      /* call post-processing, effective when post-procesing is activated in command line*/
#ifdef WMOPS
    setCounter(Idpf);
#endif
      postProc_Processing(spchnew, spchnew_post, state->var_mem,
                          state->pf, anaflag,
                          &(state->mem_speech[MEMSPEECH_LEN_MFRAME-L_FLT_DIV2]));
#ifdef WMOPS
    setCounter(Idferc);
#endif
    }

    /* Copy buffer data to output buffer */
    /* Output speech is L_AHEAD delayed. */
    /* final output is the last but one lower band frame to be synchronise with higher band */
    mov16( L_FRAME_NB, spch, sigout ); 

    /* Update wave buffer. */
    IF (sub(state->prev_bfi, 1) != 0) /*1st erased frame concealment to finish if == 1*/
    {
      mov16( L_BUFF-L_FRAME_NB, &state->fecbuf[L_FRAME_NB], &state->fecbuf[0] );
    }
  }
  ELSE
  {
    mov16(MEMSPEECH_LEN_MFRAME, &state->mem_speech[L_FRAME_NB],state->mem_speech); /*shift 5 ms*/
    IF (sub(loss_flag, NO_FRAME_LOSS) == 0)    /* Current frame not lost */
    {
      FOR (i=0; i < L_FRAME_NB; i++) /*no crossfade is needed */
      {
        /* simple copy of valid input */
        spchnew[i] = sigin[i]; move16();
      }
    }
    ELSE
    {
      FOR (i=0; i < L_FRAME_NB; i++) /*no crossfade is needed */
      {
        /* simple copy of valid input */
        spchnew[i] = 0; move16();
      }
    }
    mov16((short)L_FRAME_NB, spchnew, &(state->mem_speech[MEMSPEECH_LEN_MFRAME])); /*save 5 ms*/ 
#ifdef WMOPS
    setCounter(Idpf);
#endif
    postProc_Processing(spchnew, spchnew_post, state->var_mem,
                        state->pf, anaflag,
                        &(state->mem_speech[MEMSPEECH_LEN_MFRAME-L_FLT_DIV2]));
#ifdef WMOPS
    setCounter(Idferc);
#endif

    /* Copy buffer data to output buffer */
    /* Output speech is L_AHEAD delayed. */
    /* final output is the last but one lower band frame to be synchronise with higher band */
    IF (state->pf)
    {
      mov16( L_FRAME_NB, spchnew_post, sigout ); 
    }
    ELSE
    {
      mov16( L_FRAME_NB, spchnew, sigout ); 
    }
  }
}

/*-----------------------------------------------------------------*
 *   Funtion  fec_resynchro                                        *
 *            ~~~~~~~~~~~~~~~~                                     *
 *   - Resynchronization after FERC                                *
 *-----------------------------------------------------------------*/

static void Plc_resynchro(
  Word16 *conceal,  /* I/O: pointer to buffer with concealed speech (and extrapolated conc.) */
  Word16 *sp,       /* IN:  pointer to buffer with newly decoded speech */
  Word16  clas      /* IN:  last class */
)
{
  Word16  i, j, j_win;
  Word16  *pt_end, *pt1, *pt2;
  Word32  L_c, L_cmax;

  Word32  L_enr1, L_enr2;
  Word32  L_tmp1, L_tmp2;
  Word16  exp1, exp2, exp3;
  Word16  high1, high2;

  Word16  ratio;

  Word32  L_vec_E1, L_vec_E2;
  Word16  incr;
  Word16  coef;
  Word16  len;

  /* Initialize Pointer to the end of the Buffer */
  pt_end = conceal + L_FRAME_NB;

  /* Calculate Correlation between the Extra Frame and the Received Frame */
  /* - only in a limited range - */
  j_win = 0;    move16();
  L_cmax = 0L;  move32();
  FOR (j = -RSX_LEN; j <= RSX_LEN; j++)
  {
    pt1 = pt_end + j;
    pt2 = sp;
    L_c = L_mult(*pt1++, *pt2++);
    FOR (i = 1; i < L_FRAME_NB-RSX_LEN; i++)
    {
      L_c = L_mac(L_c, *pt1++, *pt2++);
    }
    if (L_sub(L_c, L_cmax) > 0)
    {
      j_win = j;    move16();
    }
    L_cmax = L_max(L_c, L_cmax);
  }

  /* Calculate Maximum Normalized Correlation */
  L_tmp1 = L_add(Sum_vect_E(pt_end + j_win, L_FRAME_NB - RSX_LEN), 1);
  L_tmp2 = L_add(Sum_vect_E(sp, L_FRAME_NB - RSX_LEN), 1);

  L_vec_E1 = L_tmp1;
  L_vec_E2 = L_tmp2;

  /* ----------- Calculate Energy Ratio Part ----------- */

  L_enr1 = L_min(L_tmp1, L_tmp2); /* Get Numer (must be the smallest for div_s) */
  L_enr2 = L_max(L_tmp1, L_tmp2); /* Get Denom (must be the  highest for div_s) */

  /* Get the Exponents */
  exp1 = norm_l(L_enr1);
  exp2 = norm_l(L_enr2);
  /* Calculate Total Exponent of L_enr1 * L_enr2 (will be used later) */
  exp3 = add(exp1, exp2);

  /* Normalize Numer & Denom */
  L_enr1 = L_shl(L_enr1, exp1);
  L_enr2 = L_shl(L_enr2, exp2);

  /* Get the Highs Parts (16 Bits Precision) */
  high1 = round(L_enr1);
  high2 = round(L_enr2);

  /* ---- Calculate Ratio of Min/Max ---- */

  /* Calculate 'Exp of Numer' minus 'Exp of Denom' (is always >= 0 because Numer < Denom)*/
  exp1 = sub(exp1, exp2);
  /* Make sure Numer < Denom (exp2 set to -1 when L_enr2 < L_enr1)*/
  exp2 = shr(extract_h(L_sub(L_enr2, L_enr1)), 15);
  L_tmp1 = L_shl(L_enr1, exp2);
  /* Divide & Apply Exponent (account for the Numer adjustment with exp2)*/
  ratio = div_s(round(L_tmp1), high2);
  ratio = shr(ratio, add(exp1, exp2)); /* When exp2 = -1 Right Shift Count is Reduced by 1 */

  IF (sub(clas, LBFEC_VOICED) == 0)
  {
    /* Min/Max > 0.5 */
    IF (sub(ratio, 16384) > 0)
    {
      /* ----------- Calculate 'L_cmax' / sqrt(L_enr1 x L_enr2) Part ----------- */

      /* Calculate Rounding difference for #1 & #2 */
      L_tmp1 = L_abs(L_mac(L_enr1, high1, -32768));
      L_tmp2 = L_abs(L_mac(L_enr2, high2, -32768));

      L_tmp2 = L_sub(L_tmp2, L_tmp1); /* Diff #2 - Diff #1 */
      if (L_tmp2  > 0) {
        /* Diff #2 worst than #1, then take 32 bits precision 'L_enr2' x high1 */
        L_tmp1 = L_mls(L_enr2, high1);
      }
      if (L_tmp2 <= 0) {
        /* Diff #1 worst than #2, then take 32 bits precision 'L_enr1' x high2 */
        L_tmp1 = L_mls(L_enr1, high2);
      }

      /* Normalize Product for Inverse Square Root */
      exp2 = norm_l(L_tmp1);
      L_tmp1 = L_shl(L_tmp1, exp2);
      /* Update Total Exponent of L_enr1 * L_enr2 Product */
      exp3 = add(exp3, exp2);
      /* if Odd we'll have to Shift once more Later */
      exp2 = s_and (exp3, 1);
      /* Do Inverse Square Root */
      Isqrt_n(&L_tmp1, &exp3);
      /* Update Exponent to Get Correct Result */
      exp3 = add(exp3, exp2);

      /* Get the Exponent of L_cmax */
      exp1 = norm_l(L_cmax);
      /* Normalize */
      L_cmax = L_shl(L_cmax, exp1);

      /* Do L_cmax x 1/sqrt(L_enr1 x L_enr2) */
      L_tmp2 = L_mls(L_tmp1, round(L_cmax));   
      /* Update Exponent to Get Correct Result */
      exp3 = add(exp3, exp1);
      /* Adjust (now in Q30) */
      L_tmp2 = L_shr(L_tmp2, exp3);

      /* L_cmax x 1/sqrt(L_enr1 x L_enr2) > 0.7 (in Q30) */
      IF (L_sub(L_tmp2, 751619277L) > 0)
      {
        /* do resynchronization, if required */
        Resample_vector(conceal, negate(j_win));
      }
    }
  }

  IF (L_sub(L_vec_E1, L_vec_E2) > 0)
  {
    len = add((Word16)L_FRAME_NB, (Word16)CROSSFADELEN);
    incr = sqrt_q15( sub(32767, ratio)); move16();
    incr = mult(incr, 409); /* 409 = 32767 / 40 * 2*/
    coef = sub(32767, incr);

    FOR (i = 0; i < len; i++)
    {
      conceal[i] = mult(conceal[i], coef);
      coef = sub(coef, incr);
    }
  }    
}

/*---------------------------------------------------------------------*
 * procedure   Sum_vect_E:                                     
 *             ~~~~~~~~~~                                    
 *  Find vector energy
 *---------------------------------------------------------------------*/

static Word32 Sum_vect_E( /* OUT:   return calculated vector energy */
  const Word16 *vec,      /* IN:   input vector                     */
  const Word16 lvec       /* IN:   length of input vector           */
)
{
  Word16  j;
  Word32  L_sum;

  L_sum = L_mult0(*vec, *vec);
  FOR (j = 1 ; j < lvec; j++)
  {
    L_sum = L_mac0(L_sum, vec[j], vec[j]);
  }
  return L_sum;
}

/*---------------------------------------------------------------------*
 * routine Resample_vector()                                           *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~                                           *
 * adds/removes samples from a vector, thereby changing its size       *
 *---------------------------------------------------------------------*/

static void Resample_vector(  
  Word16  *vect,      /* I/O: vector to be resampled */
  short   n           /* IN:   number of samples to be added/removed */
)
{
  Word16  new_sig[L_FRAME2];
  Word16  i, lim, i_pos;
  Word32  L_delta, L_pos;
  Word16  frac;
  Word32  L_tmp;
  Word16  length;

  if (n == 0) return;

  /* delta = (L_FRAME2 - n - 1) / (float)(L_FRAME2 - 1)*/
  L_delta = L_mls(27183337L, sub(L_FRAME2 - 1, n)); /* 27183337L for 1/79 in Q31*/

  /* L_pos will be accumulated in Q16 (High part will be integer part)*/

  /*pos = delta*/
  L_pos = L_delta;      move32();

  /* lim = n > 0 ? 0 : -n*/
  lim = negate(s_min(n, 0));

  new_sig[0] = vect[0]; move16();
  length = sub(L_FRAME2, lim);
  FOR (i = 1; i < length; i++)
  {
    /*i_pos = (short)pos*/
    i_pos = extract_h(L_pos);
    /*f = pos - i_pos*/
    frac = lshr(extract_l(L_pos), 1);

    /*new_sig[i] = (1-f) * vect[i_pos] + f * vect[i_pos+1]*/
    L_tmp = L_mult(frac, vect[i_pos+1]);
    L_tmp = L_msu(L_tmp, frac, vect[i_pos]);
    new_sig[i] = msu_r(L_tmp, -32768, vect[i_pos]);

    /*pos += delta*/
    L_pos = L_add(L_pos, L_delta);
  }

  mov16(L_FRAME2, new_sig, vect);
  zero16(lim, vect + length); 
}

/* returns the low band pitch lag*/
Word16  get_lb_pit(void * pFECWork)
{
  LBFEC_STATE * pFEC = (LBFEC_STATE *) pFECWork;
  return pFEC->t0;
}
