#include "vpcm_link.h"

size_t vpcm_link_transfer(vpcm_call_t *src_call,
                          vpcm_call_t *dst_call,
                          uint8_t *scratch,
                          size_t scratch_cap,
                          size_t octets,
                          vpcm_link_transform_fn transform,
                          void *user_data)
{
    size_t moved;

    if (!src_call || !dst_call || !scratch || scratch_cap == 0 || octets == 0)
        return 0;
    if (octets > scratch_cap)
        octets = scratch_cap;

    moved = vpcm_g711_stream_read(&src_call->tx_stream, scratch, octets);
    if (moved == 0)
        return 0;

    if (transform) {
        moved = transform(scratch, scratch_cap, scratch, moved, user_data);
        if (moved == 0)
            return 0;
    }

    moved = vpcm_g711_stream_write(&dst_call->rx_stream, scratch, moved);
    if (moved > 0) {
        vpcm_call_advance_tick(src_call);
        vpcm_call_advance_tick(dst_call);
    }
    return moved;
}
