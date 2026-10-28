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
/* Si3056 §5.14 / register 5 bit 2: validated ring indication. Unlike
 * interrupt delivery, this does not depend on the interrupt-mask setup. */
static inline int apple_line_ringing(const apple_modem_line_t *s)
{
    return apple_line_valid(s) && !(s->hook & 1) && (s->hook & 4);
}

/* USmSerial.sys 6.12.08 @0x9d15e requires a valid isolation link and
 * nonzero loop current, rejecting overload code 31. Voltage sense (LVS,
 * Si3056 §5.8/register 29) is variant-dependent and is NOT cable detection. */
static inline int apple_line_seized(const apple_modem_line_t *s)
{
    return apple_line_valid(s) && (s->hook & 1) &&
           (s->status & 31) > 0 && !apple_line_overcurrent(s) &&
           s->auxiliary >= 0 && !(s->auxiliary & 2);
}

/* Final process teardown, not on-hook CID monitoring. Si3056 §5.22:
 * restore OPE after seizure failures as well as successful calls. Attempt
 * both writes even after an I/O error; only readback can confirm release.
 * Preserve unrelated bits, but use safe values when the initial read fails. */
static inline int apple_line_release(int (*read_reg)(unsigned char),
                                     int (*write_reg)(unsigned char, unsigned char))
{
    int hook = read_reg(5), protection = read_reg(17);
    int hw = write_reg(5, (unsigned char)(hook < 0 ? 0 : hook & ~9));
    int pw = write_reg(17, (unsigned char)(protection < 0 ? 12 : protection | 8));
    hook = read_reg(5);
    protection = read_reg(17);
    return hw >= 0 && pw >= 0 && hook >= 0 && !(hook & 9) &&
           protection >= 0 && (protection & 8) ? 0 : -1;
}

static inline int apple_line_signed_sense(int value)
{
    return value >= 128 ? value-256 : value;
}
#endif
