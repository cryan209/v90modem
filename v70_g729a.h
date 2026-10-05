/*
 * v70_g729a.h -- G.729 Annex A speech coder for V.70's voice channel (V.70 5.4),
 * 10 ms / 80 samples <-> 10 octets.
 *
 * This is an ADAPTER over the ITU-T G.729 reference ANSI-C source
 * (`ITU Docs/T-REC-G.729-201206-I!!SOFT-ZST-E.zip`, g729AnnexA/c_code).  That
 * source is "Copyright (c) AT&T, France Telecom, NTT, Universite de Sherbrooke.
 * All rights reserved" with no open-source terms, so it is deliberately NOT
 * copied into this tree: extract the package yourself and point G729A_SRC at
 * its g729AnnexA/c_code directory (see the `v70-g729a-test` make target).
 *
 * The 80 parameter bits go out MSB first, the order of RFC 3551's G.729
 * payload.  The reference code keeps its state in globals, so there is ONE
 * encoder and ONE decoder per process.
 */
#ifndef V70_G729A_H
#define V70_G729A_H

#include <stdint.h>

#define G729A_FRAME_SAMPLES 80
#define G729A_FRAME_OCTETS  10

void v70_g729a_init(void);
/* pcm: 80 16-bit samples at 8 kHz. */
void v70_g729a_encode(const int16_t pcm[G729A_FRAME_SAMPLES], uint8_t out[G729A_FRAME_OCTETS]);
/* erased: the frame was lost (the decoder then conceals from its state). */
void v70_g729a_decode(const uint8_t in[G729A_FRAME_OCTETS], int erased,
                      int16_t pcm[G729A_FRAME_SAMPLES]);

#endif
