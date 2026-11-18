/* One-way codeword hop between two vpcm_call endpoints: reads the source's
 * TX stream, applies an optional caller-supplied transform (the loopback
 * passes a plain copy or a robbed-bit LSB mask) and writes the
 * destination's RX stream.  Offline harness; no live-call caller. */
#ifndef VPCM_LINK_H
#define VPCM_LINK_H

#include "vpcm_call.h"

#include <stddef.h>
#include <stdint.h>

typedef size_t (*vpcm_link_transform_fn)(uint8_t *dst,
                                         size_t dst_cap,
                                         const uint8_t *src,
                                         size_t src_len,
                                         void *user_data);

size_t vpcm_link_transfer(vpcm_call_t *src_call,
                          vpcm_call_t *dst_call,
                          uint8_t *scratch,
                          size_t scratch_cap,
                          size_t octets,
                          vpcm_link_transform_fn transform,
                          void *user_data);

#endif
