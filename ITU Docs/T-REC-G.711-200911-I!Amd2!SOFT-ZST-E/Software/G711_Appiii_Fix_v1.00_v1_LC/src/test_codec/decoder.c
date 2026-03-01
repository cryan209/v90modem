/* 
   ITU-T G.711-Appendix III  ANSI-C Source Code
   Copyright (c) 2007-2009
   NTT, France Telecom, VoiceAge Corp., Huawei

   Version: 1.0
   Revision Date: Sep. 25, 2009
*/
/*
 *------------------------------------------------------------------------
 *  File: decoder.c
 *  Function: G.711 toolbox decoder test program
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
  fprintf(stderr, " Usage: %s [-options] <law> <codefile> <outfile>\n", progname);
  fprintf(stderr, "\n");
  fprintf(stderr, " where:\n");
  fprintf(stderr, "   law         is the desired G.711 law (A or u).\n");
  fprintf(stderr, "   codefile    is the name of the input bitstream file.\n");
  fprintf(stderr, "   outfile     is the name of the decoded output file.\n");
  fprintf(stderr, "\n");
  fprintf(stderr, " Options:\n");
  fprintf(stderr, "   -ng         indicates that Noise Gate tool is activated\n");
  fprintf(stderr, "   -pf         indicates that PostFilter tool is activated\n");
  fprintf(stderr, "   -ferc       indicates that Frame ERasure Concealment tool is activated\n");
  fprintf(stderr, "   -hardbit    Input bitstream file is in multiplexed hardbit format.\n");
  fprintf(stderr, "   -quiet      quiet processing.\n");
  fprintf(stderr, "\n");
}

typedef struct {
  int  ng;
  int  ferc;
  int  pf;
  int  quiet;
  int  law;
  char *code_fname;
  char *output_fname;
  int  format;
} DECODER_PARAMS;

static void  get_commandlile_params(
  int            argc,
  char           *argv[],
  DECODER_PARAMS *params
) {
  char  *progname=argv[0];

  if (argc < 4) {
      usage(progname);
      exit(1);
  }

  /* Default mode */
  params->quiet = 0;
  params->format = 0;    /* Default is G.192 softbit format */
  params->ng = 0;
  params->ferc = 0;
  params->pf = 0;

  while (argc > 1 && argv[1][0] == '-')
  {
      if (strcmp(argv[1], "-quiet") == 0) {
          /* Set the quiet mode flag */
          params->quiet=1;

          /* Move arg{c,v} over the option to the next argument */
          argc --;
          argv ++;
      }
      else if (strcmp(argv[1], "-ng") == 0)
      {
          /* set the mode */
          params->ng = 1;
          /* Move arg{c,v} over the option to the next argument */
          argc --;
          argv ++;
      }
      else if (strcmp(argv[1], "-ferc") == 0)
      {
          /* set the mode */
          params->ferc = 1;
          /* Move arg{c,v} over the option to the next argument */
          argc --;
          argv ++;
      }
      else if (strcmp(argv[1], "-pf") == 0)
      {
          /* set the mode */
          params->pf = 1;
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
          fprintf(stderr, "ERROR! Invalid option \"%s\" in command line\n\n",argv[1]);
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
      fprintf(stderr, "Error in law character %s, ", argv[1]);
      fprintf(stderr, "law character must be A or u!!\n\n");
      usage(progname);
      exit(-1);
  }


  /* Open input code, output signal files. */
  params->code_fname   = argv[2];
  params->output_fname = argv[3];

}

#ifdef WMOPS
  short           Id = -1;
  short           Idng = -1;
  short           Idferc = -1;
  short           Idpf = -1;
#endif

/***************************************************************************
 * main()
 ***************************************************************************/
int
main(int argc, char *argv[])
{
  DECODER_PARAMS  params;
  int             nbitsIn;
  int             nbytesIn;
  int             nsamplesOut;
  FILE            *fpcode, *fpout;

  void            *theDecoder=0;

  int             status;
  short           sbufOut[NSamplesPerFrame8k];
  unsigned short  sbufIn[G192_HeaderSize+TotalBitsPerFrame];
  unsigned char   cbufIn[TotalBytesPerFrame];
  int             payloadsize;
  int             ploss_status=0;

  /* Set parameters from argv[]. */
  get_commandlile_params( argc, argv, &params );

  nbitsIn  = NBitsPerFrame0;
  nbytesIn = NBytesPerFrame0;
  nsamplesOut = NSamplesPerFrame8k;

  /* Open input bitstream */
  fpcode   = fopen(params.code_fname, "rb");
  if (fpcode == (FILE *)NULL) {
      fprintf(stderr, "file open error.\n");
      exit(1);
  }

  /* Open output speech file. */
  fpout  = fopen(params.output_fname, "wb");
  if (fpout == (FILE *)NULL) {
      fprintf(stderr, "file open error.\n");
      exit(1);
  }

  /* Instanciate an decoder. */
  theDecoder = G711AppDecode_const(params.law, params.ng, params.ferc, params.pf);
  if (theDecoder == 0) {
      fprintf(stderr, "Decoder init error.\n");
      exit(1);
  }

#ifdef WMOPS
  setFrameRate(8000, NSamplesPerFrame8k);
  Idng = getCounterId("NoiseGate");
  setCounter(Idng);
  Init_WMOPS_counter();
  Idferc = getCounterId("FERC");
  setCounter(Idferc);
  Init_WMOPS_counter();
  Idpf = getCounterId("PostFilter");
  setCounter(Idpf);
  Init_WMOPS_counter();
  Id = getCounterId("Decoder");
  setCounter(Id);
  Init_WMOPS_counter();
#endif

  /* Reset (unnecessary if right after instantiation!). */
  G711AppDecode_reset( theDecoder );

  while (1)
  {
#ifdef WMOPS
      setCounter(Id);
      fwc();
      Reset_WMOPS_counter();
      setCounter(Idng);
      fwc();
      Reset_WMOPS_counter();
      setCounter(Idferc);
      fwc();
      Reset_WMOPS_counter();
      setCounter(Idpf);
      fwc();
      Reset_WMOPS_counter();
      setCounter(Id);
#endif
      if( params.format == 0 )    /* G.192 softbit output format */
      {
          /* Read bitstream. */
          if (fread(sbufIn, sizeof(short), G192_HeaderSize+nbitsIn, fpcode) ==  0)
              break;

          /* Check FER and payload size */
          payloadsize = checksoftbit( sbufIn );

          ploss_status = 0; /* False: No FER */
          IF( payloadsize <= 0 )  /* Frame erasure */
          {
              ploss_status = 1; /* True: FER */
          }

          /* Convert from softbit to hardbit */
          softbit2hardbit( nbytesIn, &sbufIn[G192_HeaderSize], cbufIn );
      }
      else
      {
          /* Read bitstream. */
          if (fread(cbufIn, sizeof(char), nbytesIn, fpcode) ==  0)
              break;
          ploss_status = 0; /* False: No FER */
                            /* When FER is detected, set ploss_status=1 */
      }

      /* Decode. */
      status = G711AppDecode( cbufIn, sbufOut, theDecoder, ploss_status );

      if ( status ) {
          fprintf(stderr, "Decoder NG. Exiting.\n");
          exit(1);
      }

      /* Write output signal to fout. */
      fwrite(sbufOut, sizeof(short), nsamplesOut, fpout);
  }
#ifdef WMOPS
  setCounter(Id);
  fwc();
  WMOPS_output(0);
  setCounter(Idng);
  fwc();
  WMOPS_output(0);
  setCounter(Idferc);
  fwc();
  WMOPS_output(0);
  setCounter(Idpf);
  fwc();
  WMOPS_output(0);
#endif

  /* Close files. */
  fclose(fpcode);
  fclose(fpout);
  
  /* Delete the decoder. */
  G711AppDecode_dest( theDecoder );

  return 0;
}
