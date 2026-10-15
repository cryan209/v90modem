/* Apple SM56 DAA status decoding, shared by the probe and live coupler.
 * Original USmSerial.sys 6.12.08, ReadReg @0x9ca6b and guard @0x9d15e:
 * register 12 bit 6 = valid isolation link; bits 4:0 = current, 31 overload.
 * Si3056/Si3018/19 datasheet §5.8 and registers 11/12/19/29 describe
 * the compatible status layout. The Apple board's LVS scale is inferred,
 * not calibrated: expose raw signed sense alongside the estimate. Never
 * convert a missing USB reply (-1) into a voltage or an overload flag.
 */
#ifndef APPLE_MODEM_LINE_H
#define APPLE_MODEM_LINE_H
typedef struct {
    int hook, status, sense, revision, auxiliary;
} apple_modem_line_t;

static inline int apple_line_valid(const apple_modem_line_t *s)
{
    return s->status >= 0 && (s->status & 0x40) && s->hook >= 0;
}
static inline int apple_line_overcurrent(const apple_modem_line_t *s)
{
    /* Si3056 §5.22: OPD is the hardware power-overload indication.
     * OVL (bit 2) is receive overload and must not trip the hook guard. */
    return apple_line_valid(s) && (s->hook & 1) &&
        ((s->status & 31) == 31 || (s->auxiliary >= 0 && (s->auxiliary & 1)));
}
static inline int apple_line_signed_sense(int value)
{
    return value >= 128 ? value-256 : value;
}
#endif
