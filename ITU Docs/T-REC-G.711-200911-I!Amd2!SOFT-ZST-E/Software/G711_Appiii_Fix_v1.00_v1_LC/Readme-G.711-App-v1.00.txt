====================================================================================
   ITU-T G.711-Appendix III  ANSI-C Source Code
   Copyright (c) 2007-2008
   NTT, France Telecom, VoiceAge Corp., Huawei

   Version: 1.0
   Revision Date: September. 25, 2009
====================================================================================

These files represent the ITU-T G.711-Appendix Coder Fixed-Point Bit-Exact C simulation.
All code is written in ANSI-C.  The coder is implemented as two separate programs:

        encoder [-options] <law> <infile> <codefile>
        decoder [-options] <law> <codefile> <outfile> 
	
                            FILE FORMATS:
                            =============

The file format of the supplied binary data is 16-bit binary data which is
read and written in 16 bit little-endian words.
The data is therefore platform DEPENDENT.

The bitstream follows the ITU-T G.192 format. For every 5-ms input speech frame,
the bitstream contains the following data:

	Word16 SyncWord
	Word16 DataLen
	Word16 1st Databit
	Word16 2nd DataBit
	.
	.
	.
	Word16 Nth DataBit

Each bit is presented as follows: Bit 0 = 0x007f, Bit 1 = 0x0081.

The SyncWord from the encoder is always 0x6b21. The SyncWord 0x6b20, on decoder side, 
indicates that the current frame was received in error (frame erasure).

The DataLen parameter gives the number of speech data bits in the frame. 


			INSTALLING THE SOFTWARE
			=======================

Installing the software on the PC:

The package includes Makefile for gcc and makefile.cl for Visual C++. 
The makefiles can be used as follows:

Linux/Cygwin: make
Visual C++  : nmake -f makefile.cl

The codec has been successfully compiled on Linux/Cygwin using gcc
and Windows using Visual C++.


                       RUNNING THE SOFTWARE
                       ====================

The usage of the "encoder" program is as follows:

  encoder [-options] <law> <infile> <codefile>

  where:
    law        is the desired G.711 law (A or u).
    infile     is the name of the input file to be encoded.
    codefile   is the name of the bitstream file.

  Options:
    -ns        indicates that Noise Shaping tool is activated
    -quiet     quiet processing.
    -hardbit   outputs hardbit instead of G.192 softbit.


The usage of the "decoder" program is as follows:

  decoder [-options] <law> -mode <modenum> <codefile> <outfile>

  where:
    law        is the desired G.711 law (A or u)
    codefile   is the name of the bitstream file.
    outfile    is the name of the decoded output file.

  Options:
    -ng        indicates that Noise Gate tool is activated
    -pf        indicates that PostFilter tool is activated
    -ferc      indicates that Frame ERasure Concealment tool is activated
    -quiet     quiet processing.
    -hardbit   specifies hardbit instead of G.192 softbit.

If you run the software on Windows:

Cygwin    : make proc
Visual C++: nmake -f makefile.cl proc
