#ifndef LEGACY_PCM_DECODE_H
#define LEGACY_PCM_DECODE_H
#include <stddef.h>
#include <stdint.h>
/* Offline 8 kHz receive evidence. Event positions are detection sample
 * offsets, unless the detail explicitly supplies a measured waveform anchor.
 * No transmit clock or peer gates are synthesized. */
typedef void (*legacy_pcm_event_fn)(void *user, int sample, const char *protocol,
                                  const char *summary, const char *detail);
int legacy_pcm_decode_x2(const int16_t *samples, size_t count,
                         legacy_pcm_event_fn event, void *user);
int legacy_pcm_decode_flex(const int16_t *samples, size_t count, int alaw,
                           legacy_pcm_event_fn event, void *user);
#endif
