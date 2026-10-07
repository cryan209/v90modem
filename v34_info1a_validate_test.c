/* V.34 Table 16: undefined INFO1a selections are not a negotiation.
 * Includes the production v34rx.c so the static parser can be called after
 * the framing/CRC stage.  A rejected frame must change nothing and must not
 * set info1a_received; a defined one must still be accepted afterwards. */
#include "spandsp-master/src/v34rx.c"

#include <assert.h>

static void pack(uint8_t buf[], int pr, int apr, int md, int hi, int pe,
                 int rate, int a2c, int c2a)
{
    /* Table 16 field order, packed the way process_rx_info1a() reads it. */
    bitstream_state_t bs;
    uint8_t *p = buf;
    int fields[][2] = {{pr,3},{apr,3},{md,7},{hi,1},{pe,4},{rate,4},
                       {a2c,3},{c2a,3},{0,10}};

    memset(buf, 0, 25);
    bitstream_init(&bs, true);
    for (unsigned f = 0; f < sizeof(fields)/sizeof(fields[0]); f++)
        bitstream_put(&bs, &p, fields[f][0], fields[f][1]);
}

static int try_frame(v34_rx_state_t *s, int pe, int rate, int a2c, int c2a)
{
    uint8_t buf[25];

    pack(buf, 0, 0, 0, 0, pe, rate, a2c, c2a);
    return process_rx_info1a(s, &s->info1a, buf);
}

int main(void)
{
    v34_state_t *v = v34_init(NULL, 2400, 9600, true, true, NULL, NULL, NULL, NULL);
    v34_rx_state_t *rx;
    info1a_t before;

    assert(v);
    rx = &v->rx;
    rx->v90_mode = false;
    rx->info1a_received = false;
    for (int i = 0; i <= 5; i++)
        rx->local_info1c_high_carrier[i] = false;

    before = rx->info1a;
    assert(try_frame(rx, 15, 15, 7, 7) < 0);            /* the all-ones frame */
    assert(!rx->info1a_received && memcmp(&before, &rx->info1a, sizeof(before)) == 0);
    assert(try_frame(rx, 3, 5, 6, 4) < 0);              /* a2c code 6 (V.90 PCM) in a plain frame */
    assert(try_frame(rx, 3, 5, 4, 7) < 0);              /* c2a undefined */
    assert(try_frame(rx, 11, 5, 4, 4) < 0);             /* pre-emphasis 11 */
    assert(try_frame(rx, 3, 15, 4, 4) < 0);             /* projected rate 15 */
    assert(!rx->info1a_received);
    assert(try_frame(rx, 3, 9, 4, 4) == 0);             /* a valid one still goes through */
    assert(rx->info1a_received && rx->info1a.baud_rate_a_to_c == 4);
    v34_free(v);
    puts("PASS: V.34 INFO1a undefined selections rejected without side effects");
    return 0;
}
