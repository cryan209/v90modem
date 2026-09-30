/* ITU-T V.44 modem stream codec. Ported from modem-dsp-emu/tools/v44.py.
 * Encoding uses its append-only string-segment policy; decoding also accepts
 * peer string extensions. Packet/parameter modes are not advertised. */
#ifndef V44_H
#define V44_H
#include <stddef.h>
#include <stdint.h>
typedef struct v44_encoder_s v44_encoder_t;
typedef struct v44_decoder_s v44_decoder_t;
typedef void (*v44_output_fn)(void *, const uint8_t *, int);
v44_encoder_t *v44_encoder_init(int codewords, int max_string, int history,
                               v44_output_fn output, void *ctx);
v44_decoder_t *v44_decoder_init(int codewords, int max_string, int history,
                               v44_output_fn output, void *ctx);
int v44_encoder_feed(v44_encoder_t *, const uint8_t *, size_t);
int v44_encoder_flush(v44_encoder_t *);
int v44_decoder_feed(v44_decoder_t *, const uint8_t *, size_t);
void v44_encoder_reset(v44_encoder_t *);
void v44_decoder_reset(v44_decoder_t *);
void v44_encoder_free(v44_encoder_t *);
void v44_decoder_free(v44_decoder_t *);
#endif
