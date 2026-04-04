#ifndef V34_INFO_DECODE_H
#define V34_INFO_DECODE_H

#include "v90.h"

#include <spandsp.h>
#include <spandsp/private/bitstream.h>
#include <spandsp/bit_operations.h>
#include <spandsp/crc.h>
#include <spandsp/v34.h>

#include <stdbool.h>
#include <stdint.h>

#define V34_INFO_SYNC_CODE 0x372
#define V34_INFO_MAX_BUF_BYTES 25

typedef struct {
    bool valid;
    int target_bits;
    int payload_bytes;
    uint8_t payload[V34_INFO_MAX_BUF_BYTES];
} v34_info_frame_t;

typedef struct {
    uint16_t bitstream;
    uint16_t crc;
    int bit_count;
    int target_bits;
    uint8_t info_buf[V34_INFO_MAX_BUF_BYTES];
} v34_info_collector_t;

void v34_info_collector_init(v34_info_collector_t *collector, int target_bits);
void v34_info_collector_reset(v34_info_collector_t *collector, int target_bits);
void v34_info_collector_load_snapshot(v34_info_collector_t *collector,
                                      uint16_t bitstream,
                                      uint16_t crc,
                                      int bit_count,
                                      int target_bits,
                                      const uint8_t *info_buf,
                                      int info_buf_len);
bool v34_info_frame_from_collector(v34_info_frame_t *frame,
                                   const v34_info_collector_t *collector);
bool v34_info_collector_push_bit(v34_info_collector_t *collector,
                                 int bit,
                                 uint8_t *frame_out,
                                 int frame_out_len);
bool v34_info_collector_push_bit_verbose(v34_info_collector_t *collector,
                                         int bit,
                                         uint8_t *frame_out,
                                         int frame_out_len,
                                         bool *frame_complete_out,
                                         uint16_t *crc_out);
bool v34_info_validate_frame_bytes(const uint8_t *buf, int target_bits);
bool v34_info_try_boundary_recovery(uint8_t *out,
                                    const uint8_t *in,
                                    int nbits,
                                    int *shift_out);
bool v34_info_try_local_slip_recovery(uint8_t *out,
                                      const uint8_t *in,
                                      int nbits,
                                      int *pivot_out,
                                      int *shift_out);

bool v34_map_received_info0a(v90_info0a_t *dst, const v34_v90_info0a_t *src);
bool v34_map_received_info1a(v90_info1a_t *dst, const v34_v90_info1a_t *src);

bool v34_info_parse_info0a_v90(const uint8_t *buf,
                               v34_v90_info0a_t *raw_out,
                               v90_info0a_t *mapped_out);
bool v34_info_parse_info1a_v90(const uint8_t *buf,
                               v34_v90_info1a_t *raw_out,
                               v90_info1a_t *mapped_out);
bool v34_info_parse_info1d_v90(const uint8_t *buf,
                               v34_v90_info1d_t *raw_out);
bool v34_info_parse_info0a_v90_frame(const v34_info_frame_t *frame,
                                     v34_v90_info0a_t *raw_out,
                                     v90_info0a_t *mapped_out);
bool v34_info_parse_info1a_v90_frame(const v34_info_frame_t *frame,
                                     v34_v90_info1a_t *raw_out,
                                     v90_info1a_t *mapped_out);
bool v34_info_parse_info1d_v90_frame(const v34_info_frame_t *frame,
                                     v34_v90_info1d_t *raw_out);

#endif
