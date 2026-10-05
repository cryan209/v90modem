/* V.250 (07/2003) 6.7.2 diagnostic command state.
 * Local digital loop is the DTE loop in 6.7.2.13, not a DSP/audio loop.
 * Threading: caller owns serialization; no engine or PTY calls here. */
#ifndef AT_TEST_H
#define AT_TEST_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

typedef struct {
    bool local_loop;
    int type, last_type, block_length, pattern_id, pattern_length;
    int self_result; /* V.250 6.7.2.17: 0 not run, 1 pass, 2 fail */
    uint32_t blocks_left, block_bits;
    uint64_t checked_bits, bit_errors, block_errors;
    bool block_bad;
    int tx_pos, rx_pos;
    uint8_t pattern[2047];
} at_test_t;

void at_test_reset(at_test_t *s);
/* Returns 0 for accepted command, -1 for ERROR; response may be empty.
 * online_command means an active data call in Online Command State. */
int at_test_command(at_test_t *s, const char *command, bool online_command,
                    char *response, size_t response_size);
int at_test_get_bit(at_test_t *s);
void at_test_put_bit(at_test_t *s, int bit);
/* Clock the installed local digital bit loop, with independent TX/RX state. */
void at_test_clock_local(at_test_t *s, uint64_t bits);
void at_test_disconnect(at_test_t *s);
#endif
