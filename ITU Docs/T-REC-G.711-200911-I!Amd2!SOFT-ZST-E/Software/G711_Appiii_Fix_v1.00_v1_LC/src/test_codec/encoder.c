/* 
   ITU-T G.711-Appendix III  ANSI-C Source Code
   Copyright (c) 2007-2009
   NTT, France Telecom, VoiceAge Corp., Huawei

   Version: 1.0
   Revision Date: Sep. 25, 2009
*/
/*
 *------------------------------------------------------------------------
 *  File: encoder.c
 *  Function: G.711 toolbox encoder test program
 *------------------------------------------------------------------------
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stl.h"
#include "g711App.h"
#include "softbit.h"

/***************************************************************************
 * usage()
 ***************************************************************************/
static void usage(char progname[])
{
  fprintf(stderr, "\n");
  fprintf(stderr, " Usage: %s [-options] <law> <infile> <codefile>\n", progname);
  fprintf(stderr, "\n");
  fprintf(stderr, " where:\n" );
  fprintf(stderr, "   law         is the desired G.711 law (A or u).\n");
  fprintf(stderr, "   infile      is the name of the input file to be encoded.\n");
  fprintf(stderr, "   codefile    is the name of the output bitstream file.\n");
  fprintf(stderr, "\n");
  fprintf(stderr, " Options:\n");
  fprintf(stderr, "   -ns         indicates that Noise Shaping tool is activated\n");
  fprintf(stderr, "   -hardbit    Output bitstream file is in multiplexed hardbit format.\n");
  fprintf(stderr, "   -quiet      quiet processing.\n");
  fprintf(stderr, "\n");
}

typedef struct {
  int  ns;
  int  quiet;
  int  format;
  int  law;
  char *input_fname;
  char *code_fname;
} ENCODER_PARAMS;

static void  get_commandlile_params(
  int            argc,
  char           *argv[],
  ENCODER_PARAMS *params
) {
  char  *progname=argv[0];

  if (argc < 4) {
      usage(progname);
      exit(1);
  }

  /* Default mode */
  params->quiet = 0;
  params->format = 0;    /* Default is G.192 softbit format */
  params->ns = 0;

  /* Search options */
  while (argc > 1 && argv[1][0] == '-')
  {
      if (strcmp(argv[1], "-ns") == 0)
      {
          /* set the mode */
          params->ns = 1;
          /* Move arg{c,v} over the option to the next argument */
          argc --;
          argv ++;
      }
      else if (strcmp(argv[1],"-quiet") == 0) {
          /* Set the quiet mode flag */
          params->quiet=1;
          /* Move arg{c,v} over the option to the next argument */
          argc --;
          argv ++;
      }
      else if (strcmp(argv[1], "-hardbit") == 0) {
          params->format = 1;    /* Hardbit output format */
          /* Move arg{c,v} over the option to the next argument */
          argc --;
          argv ++;
      }
      else if (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "-?") == 0) {
          /* Display help message */
          usage(progname);
          exit(1);
      }
      else {
          fprintf(stderr, "ERROR! Invalid option \"%s\"\n\n",argv[1]);
          usage(progname);
          exit(1);
      }
  }
  
  /* check law character */
  if (strcmp(argv[1], "u") == 0) {
      params->law = MODE_ULAW;
  }
  else if (strcmp(argv[1], "A") == 0) {
      params->law = MODE_ALAW;
  }
  else {
      fprintf(stderr, "Error in law character %s\n", argv[1]);
      fprintf(stderr, "Law character must be A or u!!\n");
      exit(-1);
  }

  /* Open input signal and output code files. */
  params->input_fname  = argv[2/*++iargv*/];
  params->code_fname   = argv[3/*++iargv*/];
}
#ifdef WMOPS
  short           Id = -1;
  short           Idns = -1;
#endif

/***************************************************************************
 * main()
 ***************************************************************************/
int
main(int argc, char *argv[])
{
  int             i;
  ENCODER_PARAMS  params;
  int             nsamplesIn;
  int             nbitsOut;
  int             nbytesOut;
  FILE            *fpin, *fpcode;

  void            *theEncoder=0;

  int             status;
  short           sbufIn[NSamplesPerFrame8k];
  unsigned short  sbufOut[G192_HeaderSize+TotalBitsPerFrame];
  unsigned char   cbufOut[TotalBytesPerFrame];

  /* Set parameters from argv[]. */
  get_commandlile_params( argc, argv, &params );

  nsamplesIn = NSamplesPerFrame8k;  /* Input sampling rate is 8 kHz */
  nbitsOut  = NBitsPerFrame0;
  nbytesOut = NBytesPerFrame0;

  /* Open input speech file. */
  fpin = fopen(params.input_fname, "rb");
  if (fpin == (FILE *)NULL) {
      fprintf(stderr, "file open error.\n");
      exit(1);
  }

  /* Open output bitstream. */
  fpcode = fopen(params.code_fname, "wb");
  if (fpcode == (FILE *)NULL) {
      fprintf(stderr, "file open error.\n");
      exit(1);
  }

  /* Instanciate an encoder. */
  theEncoder = G711AppEncode_const(params.law, params.ns);
  if (theEncoder == 0) {
      fprintf(stderr, "Encoder init error.\n");
      exit(1);
  }

  /* Reset (unnecessary if right after instantiation!). */
  G711AppEncode_reset( theEncoder );

#ifdef WMOPS
  setFrameRate(8000, NSamplesPerFrame8k);
  Id = getCounterId("Encoder");
  setCounter(Id);
  Init_WMOPS_counter();
  Idns = getCounterId("NoiseShaping");
  setCounter(Idns);
  Init_WMOPS_counter();
#endif

  while (1)
  {
#ifdef WMOPS
      setCounter(Id);
      fwc();
      Reset_WMOPS_counter();
      setCounter(Idns);
      fwc();
      Reset_WMOPS_counter();
      setCounter(Id);
#endif
      /* Initialize sbuf[]. */
      for (i=0; i<nsamplesIn; i++) sbufIn[i] = 0;

      /* Read input singal from fin. */
      if ( fread( sbufIn, sizeof(short), nsamplesIn, fpin ) == 0 )
          break;

      /* Encode. */
      status = G711AppEncode( sbufIn, cbufOut, theEncoder );

      if ( status ) {
          fprintf(stderr, "Encoder NG. Exiting.\n");
          exit(1);
      }

      if( params.format == 0 )    /* G.192 softbit output format */
      {
          /* Write main header */
          sbufOut[0] = G192_SYNCHEADER;
          sbufOut[idxG192_BitstreamLength] = nbitsOut;

          /* Convert from hardbit to softbit. */
          hardbit2softbit( nbytesOut, cbufOut, &sbufOut[G192_HeaderSize] );

          /* Write bitstream. */
          fwrite( sbufOut, sizeof(short), G192_HeaderSize+nbitsOut, fpcode );
      }
      else    /* Hardbit output format */
      {
          /* Write bitstream. */
          fwrite( cbufOut, sizeof(char), nbytesOut, fpcode );
      }
  }
#ifdef WMOPS
  setCounter(Id);
  fwc();
  WMOPS_output(0);
  setCounter(Idns);
  fwc();
  WMOPS_output(0);
#endif

  /* Close files. */
  fclose(fpin);
  fclose(fpcode);
  
  /* Delete the encoder. */
  G711AppEncode_dest( theEncoder );

  return 0;
}
