/* V.FC proprietary ROCK signature, Courier 403 a6db..a71b / aa18..aa59.
 * Bit layer only: input/output dibits are before scrambling/after descrambling.
 * The caller owns the all-ones acquisition, QPSK modem and burst scheduling.
 * Firmware addresses identify this protocol; there is no ITU V.FC clause.
 */
#ifndef VFC_SIGNATURE_H
#define VFC_SIGNATURE_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint64_t window;
    unsigned dibits;
    bool matched;
} vfc_signature_rx_t;

typedef struct {
    uint8_t code;
    unsigned response_count; /* firmware 03ca timer units; not milliseconds */
} vfc_signature_t;

/* ROCK then four copies of the code byte, least-significant dibit first.
 * Firmware a6f9 masks the transmitted code to four bits. Invalid arguments
 * leave output unchanged. This does not generate the all-ones preamble. */
bool vfc_signature_encode(unsigned code, uint8_t dibits[32]);
void vfc_signature_rx_init(vfc_signature_rx_t *s);

/* Starts at aa18, after the firmware's aa05 all-ones acquisition. V.8 has
 * priority when both 03a6 bit 0 and 03af bit 12 are set. Set v8_priority to
 * their conjunction. Successful recognition is latched until reset, as the
 * native decoder then leaves this state for a40b. Invalid dibits are ignored. */
bool vfc_signature_rx_put(vfc_signature_rx_t *s, unsigned dibit,
                          bool v8_priority, vfc_signature_t *result);
#endif
