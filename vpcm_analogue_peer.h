/* V.90 Table 7 bits 33:39 and 9.5.2: the digital modem announces its
 * PCM law/power in INFO0d; a retrain resumes after INFO0 and must retain them.
 * A physical analogue bearer has no local G.711 law. Its PCM receiver uses
 * the peer's law, independently of any virtual/SIP bearer codec. */
#ifndef VPCM_ANALOGUE_PEER_H
#define VPCM_ANALOGUE_PEER_H
#include <stdbool.h>

typedef struct {
    bool valid;
    bool alaw;
    unsigned max_power_code;
    bool codec_output;
} vpcm_analogue_peer_t;

static inline void vpcm_analogue_peer_reset(vpcm_analogue_peer_t *p)
{
    *p = (vpcm_analogue_peer_t){0};
}

static inline void vpcm_analogue_peer_note(vpcm_analogue_peer_t *p, bool valid,
                                          bool alaw, unsigned max_power_code,
                                          bool codec_output)
{
    if (valid)
        *p = (vpcm_analogue_peer_t){true, alaw, max_power_code & 31, codec_output};
}

static inline bool vpcm_analogue_peer_alaw(const vpcm_analogue_peer_t *p,
                                          bool physical_analogue, bool bearer_alaw)
{
    return physical_analogue && p->valid ? p->alaw : bearer_alaw;
}

static inline double vpcm_analogue_peer_max_power(const vpcm_analogue_peer_t *p)
{
    return p->valid ? -0.5*(p->max_power_code + 1) : 0.0;
}
#endif
