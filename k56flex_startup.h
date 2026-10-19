/* K56flex digital-side post-V.8bis startup (Draft 0.23 clause 4.1).
 * V.8bis 9.9 assigns the MS receiver the V.8 answering role and the sender
 * the calling role, independently of who dialled. Only use after an accepted
 * K56flex exchange and ACK. Digital Phase 2 remains answer-shaped.
 * The common V.34 Phase-2 prelude (10.1.2) is experimental here: completion
 * requires received CRC-valid INFO1 and local INFO1 TX completion. This
 * module never exposes V.34 DATA or synthesizes a K56flex peer gate.
 */
#ifndef K56FLEX_STARTUP_H
#define K56FLEX_STARTUP_H
#include <stdint.h>

typedef struct k56flex_startup_s k56flex_startup_t;
enum { K56S_V8, K56S_PHASE2, K56S_READY, K56S_FAILED };
k56flex_startup_t *k56flex_startup_new(int v8_calling_party, int verbose);
void k56flex_startup_free(k56flex_startup_t *s);
void k56flex_startup_rx(k56flex_startup_t *s, const int16_t *pcm, int count);
void k56flex_startup_tx(k56flex_startup_t *s, int16_t *pcm, int count);
int k56flex_startup_state(const k56flex_startup_t *s);
const char *k56flex_startup_failure(const k56flex_startup_t *s);
#endif
