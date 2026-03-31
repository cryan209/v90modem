#include "vpcm_call_pair.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

static void vpcm_call_pair_make_tap_name(char *dst, size_t dst_cap, const char *src)
{
    size_t i;
    size_t out = 0;

    if (!dst || dst_cap == 0)
        return;
    if (!src)
        src = "stream";
    for (i = 0; src[i] != '\0' && out + 1 < dst_cap; ++i) {
        unsigned char ch = (unsigned char) src[i];

        if (isalnum(ch))
            dst[out++] = (char) tolower(ch);
        else if (out > 0 && dst[out - 1] != '-')
            dst[out++] = '-';
    }
    while (out > 0 && dst[out - 1] == '-')
        out--;
    if (out == 0 && dst_cap > 1)
        dst[out++] = 'x';
    dst[out] = '\0';
}

bool vpcm_call_pair_init(vpcm_call_pair_t *pair,
                         vpcm_call_t *caller,
                         vpcm_call_t *answerer,
                         const char *label)
{
    if (!pair || !caller || !answerer)
        return false;
    pair->caller = caller;
    pair->answerer = answerer;
    pair->label = label;
    return true;
}

bool vpcm_call_pair_attach_taps(vpcm_call_pair_t *pair, const char *dir)
{
    char base[64];
    char caller_tx_path[512];
    char caller_rx_path[512];
    char answerer_tx_path[512];
    char answerer_rx_path[512];

    if (!pair || !pair->caller || !pair->answerer)
        return false;
    if (!dir || !*dir)
        return true;

    vpcm_call_pair_make_tap_name(base, sizeof(base), pair->label);
    if (snprintf(caller_tx_path, sizeof(caller_tx_path), "%s/%s-caller-tx.g711", dir, base)
            >= (int) sizeof(caller_tx_path)
        || snprintf(caller_rx_path, sizeof(caller_rx_path), "%s/%s-caller-rx.g711", dir, base)
            >= (int) sizeof(caller_rx_path)
        || snprintf(answerer_tx_path, sizeof(answerer_tx_path), "%s/%s-answerer-tx.g711", dir, base)
            >= (int) sizeof(answerer_tx_path)
        || snprintf(answerer_rx_path, sizeof(answerer_rx_path), "%s/%s-answerer-rx.g711", dir, base)
            >= (int) sizeof(answerer_rx_path)) {
        return false;
    }

    if (!vpcm_call_attach_stream_taps(pair->caller, caller_tx_path, caller_rx_path))
        return false;
    if (!vpcm_call_attach_stream_taps(pair->answerer, answerer_tx_path, answerer_rx_path)) {
        vpcm_call_detach_stream_taps(pair->caller);
        return false;
    }
    return true;
}

void vpcm_call_pair_detach_taps(vpcm_call_pair_t *pair)
{
    if (!pair)
        return;
    if (pair->caller)
        vpcm_call_detach_stream_taps(pair->caller);
    if (pair->answerer)
        vpcm_call_detach_stream_taps(pair->answerer);
}

bool vpcm_call_pair_drive_to_run(vpcm_call_pair_t *pair,
                                 vpcm_call_run_mode_t run_mode)
{
    if (!pair || !pair->caller || !pair->answerer)
        return false;

    vpcm_call_set_state(pair->caller, VPCM_CALL_WAIT_DIALTONE);
    vpcm_call_set_state(pair->answerer, VPCM_CALL_WAIT_RINGING);

    if (!vpcm_call_step_to_next_state(pair->caller) || !vpcm_call_step(pair->answerer))
        return false;
    if (!vpcm_call_step_to_next_state(pair->caller) || !vpcm_call_step(pair->answerer))
        return false;
    if (!vpcm_call_step(pair->caller) || !vpcm_call_step_to_next_state(pair->answerer))
        return false;
    if (!vpcm_call_step_to_run_mode(pair->caller, run_mode)
        || !vpcm_call_step_to_run_mode(pair->answerer, run_mode)) {
        return false;
    }
    return true;
}

bool vpcm_call_pair_drive_to_done(vpcm_call_pair_t *pair)
{
    if (!pair || !pair->caller || !pair->answerer)
        return false;

    vpcm_call_set_state(pair->caller, VPCM_CALL_HANGUP);
    vpcm_call_set_state(pair->answerer, VPCM_CALL_HANGUP);
    if (!vpcm_call_step_to_next_state(pair->caller)
        || !vpcm_call_step_to_next_state(pair->answerer)) {
        return false;
    }
    vpcm_call_pair_detach_taps(pair);
    return true;
}

void vpcm_call_pair_set_v91_state(vpcm_call_pair_t *pair,
                                  vpcm_v91_modem_state_t state)
{
    if (!pair || !pair->caller || !pair->answerer)
        return;
    vpcm_call_set_v91_state(pair->caller, state);
    vpcm_call_set_v91_state(pair->answerer, state);
}
