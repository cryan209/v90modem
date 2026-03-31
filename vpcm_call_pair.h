#ifndef VPCM_CALL_PAIR_H
#define VPCM_CALL_PAIR_H

#include "vpcm_call.h"

#include <stdbool.h>

typedef struct {
    vpcm_call_t *caller;
    vpcm_call_t *answerer;
    const char *label;
} vpcm_call_pair_t;

bool vpcm_call_pair_init(vpcm_call_pair_t *pair,
                         vpcm_call_t *caller,
                         vpcm_call_t *answerer,
                         const char *label);
bool vpcm_call_pair_attach_taps(vpcm_call_pair_t *pair, const char *dir);
void vpcm_call_pair_detach_taps(vpcm_call_pair_t *pair);
bool vpcm_call_pair_drive_to_run(vpcm_call_pair_t *pair,
                                 vpcm_call_run_mode_t run_mode);
bool vpcm_call_pair_drive_to_done(vpcm_call_pair_t *pair);
void vpcm_call_pair_set_v91_state(vpcm_call_pair_t *pair,
                                  vpcm_v91_modem_state_t state);

#endif
