/* MT5634MU bulk-OUT retry boundary, independent of IOKit for testing.
 *
 * EP 0x02 has 16-byte USB packets. WritePipeTO does not report how much of
 * a timed-out multi-packet write completed. Retrying 64 bytes after 48 were
 * accepted duplicates 48 DTE bytes (observed in a 256 KiB live upload).
 * Submit at most ONE packet per request. On NAK/timeout retain the endpoint
 * data toggle: USB 2.0 8.6 handles lost ACKs; clearing endpoint halt resets
 * that protection (9.4.5) and is not a remedy for ordinary backpressure.
 */
#ifndef MT5634MU_BULK_WRITE_H
#define MT5634MU_BULK_WRITE_H

#include <signal.h>
#include <stddef.h>
#include <stdint.h>

#define MT5634MU_OUT_PACKET 16u
#define MT5634MU_WRITE_OK 0
#define MT5634MU_WRITE_RETRY 1
#define MT5634MU_WRITE_ERROR (-1)

typedef int (*mt5634mu_packet_writer_t)(void *ctx, const uint8_t *data, size_t n);

static inline int mt5634mu_write_packets(const uint8_t *data, size_t n,
                                        unsigned endpoint_packet_size,
                                        mt5634mu_packet_writer_t write_packet,
                                        void *ctx, const volatile sig_atomic_t *quit)
{
    size_t packet = endpoint_packet_size;
    if (!packet || !write_packet || (!data && n))
        return MT5634MU_WRITE_ERROR;
    if (packet > MT5634MU_OUT_PACKET)
        packet = MT5634MU_OUT_PACKET;
    while (n) {
        size_t count = n < packet ? n : packet;
        int result;
        if (quit && *quit)
            return MT5634MU_WRITE_ERROR;
        result = write_packet(ctx, data, count);
        if (result == MT5634MU_WRITE_RETRY)
            continue;  /* same packet, no pointer advance or toggle reset */
        if (result != MT5634MU_WRITE_OK)
            return MT5634MU_WRITE_ERROR;
        data += count;
        n -= count;
    }
    return MT5634MU_WRITE_OK;
}
#endif
