/* apple_usb_modem_probe -- USB-side probe for the Apple USB Modem (A1082),
 * USB 05ac:1401, a Motorola SM56 softmodem.
 *
 * Role: read the device's descriptors, put it into its one configuration, and
 * exercise the CDC class-interface command channel on interface 1 that the
 * withdrawn Motorola driver used.  The codec itself is NOT here -- once the
 * configuration is set, macOS's usbaudiod claims the audio interfaces and the
 * codec is reached through CoreAudio; see tools/apple_usb_modem_audio.m.
 *
 * Protocol, recovered from utlamot.sys (the USB half of the Boot Camp Windows
 * driver) and confirmed against the device.  All requests are
 * URB_FUNCTION_CLASS_INTERFACE, i.e. class requests to an interface:
 *
 *   wIndex 1 (interface 1), bRequest 0x00  SEND_ENCAPSULATED_COMMAND, 3 or 9 B
 *   wIndex 1 (interface 1), bRequest 0x01  GET_ENCAPSULATED_RESPONSE, 2 B
 *   wIndex 0 (interface 0), bRequest 0x11/0x13/0x14, argument in wValue, no data
 *
 * The device is a REGISTER FILE reached through the 3-byte encapsulated
 * commands, and that is the whole of the line interface:
 *
 *   write:  00 <index> <value>          (silent)
 *   read:   80 <index> 00               then GET_ENCAPSULATED_RESPONSE, 1 byte
 *
 * Register 5 bit 3 (0x08) powers the analogue RECEIVE path: with it clear the
 * codec's input is railed at -32768 at every sample rate, and with it set the
 * codec delivers a real signal.  Deterministic and repeatable in both
 * directions, with and without a line.
 *
 * IT IS NOT THE HOOK.  Against a VG224 FXS port -- a port that supplies loop
 * current and dial tone -- setting it produces mains hum and no dial tone, and
 * no register in 0x01-0x3b changes when it flips.  The hook relay is not
 * identified; do not read --hook as seizing the line.  Two other bits found on
 * the way: register 5 bit 0 re-rails the codec even with bit 3 set (a reset or
 * override), and register 0x0a = 1 gives exact digital SILENCE rather than the
 * rail (a mute -- the two are distinguishable).
 *
 * Registers 0x10, 0x1a, 0x1f, 0x1e carry the per-country DAA configuration.
 * usm56.reg's HardwareInitBB is the one-byte profile KEY into the driver's own
 * table (NZ 0x3e -> a0/c0/00/00), not the four values themselves.
 *
 * Other opcodes the driver emits and this tool does not model: 0x02 (a 9-byte
 * windowed form), 0x10|n, 0x90, 0xd0.  See docs/apple_usb_modem_sm56.md.
 *
 * TRAPS, each of which cost a session:
 *
 *  - The device sits at bConfigurationValue 0 until something sets it, and an
 *    unconfigured device stalls EVERY request including standard
 *    GET_INTERFACE, while ioreg shows it with no interface children at all.
 *    That reads exactly like dead hardware.  `--configure` first.
 *  - libusb's darwin backend serves device and config descriptors out of the
 *    IOKit cache, so those keep succeeding after the device has stopped
 *    answering anything.  Liveness here is a STRING descriptor, which has to
 *    reach the wire.  (Same trap as docs/hsf_usb_daa.md.)
 *  - A queued response does NOT survive closing the handle.  Sending the read
 *    from one process and the GET from the next reports "nothing queued" for
 *    every register, which reads exactly like a device that does not answer.
 *    Command and response must be one session -- hence --read/--regs rather
 *    than --send followed by --get.
 *  - A 2-byte command is STALLED.  Only 3- and 9-byte bodies are accepted, so
 *    the read is "80 <index> 00" and not "<index> 00".
 *
 * Build: make apple_usb_modem_probe
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <libusb.h>

#define VID 0x05ac
#define PID 0x1401

/* Interface numbers, per the device's own configuration descriptor. */
#define IF_ACM_CONTROL   0
#define IF_COMMAND       1

/* CDC class requests carried on IF_COMMAND. */
#define SEND_ENCAPSULATED_COMMAND  0x00
#define GET_ENCAPSULATED_RESPONSE  0x01

static libusb_device_handle *h;

/* Liveness: must reach the wire.  See the trap note above. */
static int alive(void)
{
    unsigned char s[64];
    return libusb_get_string_descriptor_ascii(h, 3, s, sizeof s) > 0;
}

static void hexdump(const char *tag, const unsigned char *b, int n)
{
    printf("%s (%d):", tag, n);
    for (int i = 0; i < n; i++) printf(" %02x", b[i]);
    printf("  |");
    for (int i = 0; i < n; i++) putchar((b[i] >= 32 && b[i] < 127) ? b[i] : '.');
    printf("|\n");
}

static const char *xfer_name(int attr)
{
    switch (attr & 3) {
    case LIBUSB_TRANSFER_TYPE_CONTROL:     return "control";
    case LIBUSB_TRANSFER_TYPE_ISOCHRONOUS: return "ISOCHRONOUS";
    case LIBUSB_TRANSFER_TYPE_BULK:        return "bulk";
    case LIBUSB_TRANSFER_TYPE_INTERRUPT:   return "interrupt";
    }
    return "?";
}

/* Print a USB-audio class AS_GENERAL/FORMAT_TYPE_I pair if present.  The rate
 * list is the whole reason this device is interesting: 8 kHz plus every V.34
 * symbol rate times three. */
static void dump_audio_format(const unsigned char *extra, int len)
{
    int i = 0;
    while (i + 2 < len) {
        int dlen = extra[i], dtype = extra[i + 1], sub = extra[i + 2];
        if (dtype == 0x24 && sub == 0x02 && i + 8 <= len) {   /* FORMAT_TYPE */
            int nch = extra[i + 4], sub_sz = extra[i + 5];
            int bits = extra[i + 6], nrates = extra[i + 7];
            printf("        format: %d ch, %d-byte subframe, %d bits, %d rates:",
                   nch, sub_sz, bits, nrates);
            for (int r = 0; r < nrates && i + 8 + r * 3 + 2 < len; r++) {
                const unsigned char *p = extra + i + 8 + r * 3;
                printf(" %u", (unsigned)(p[0] | (p[1] << 8) | (p[2] << 16)));
            }
            printf("\n");
        }
        if (dlen <= 0) break;
        i += dlen;
    }
}

static int cmd_descriptors(libusb_device *dev)
{
    struct libusb_device_descriptor dd;
    if (libusb_get_device_descriptor(dev, &dd) < 0) return 1;

    printf("device %04x:%04x  class=%u/%u/%u  bcdUSB=%04x bcdDevice=%04x\n",
           dd.idVendor, dd.idProduct, dd.bDeviceClass, dd.bDeviceSubClass,
           dd.bDeviceProtocol, dd.bcdUSB, dd.bcdDevice);
    if (dd.bDeviceClass == 0xff)
        printf("  (vendor-specific device class -- no in-box driver will bind)\n");

    for (int c = 0; c < dd.bNumConfigurations; c++) {
        struct libusb_config_descriptor *cfg;
        if (libusb_get_config_descriptor(dev, c, &cfg) < 0) continue;
        printf("  config %u: %u interfaces, maxpower %u mA\n",
               cfg->bConfigurationValue, cfg->bNumInterfaces, cfg->MaxPower * 2);
        for (int n = 0; n < cfg->bNumInterfaces; n++) {
            const struct libusb_interface *itf = &cfg->interface[n];
            for (int a = 0; a < itf->num_altsetting; a++) {
                const struct libusb_interface_descriptor *id = &itf->altsetting[a];
                printf("    if %u alt %u: class=%u/%u/%u, %u endpoint(s)%s\n",
                       id->bInterfaceNumber, id->bAlternateSetting,
                       id->bInterfaceClass, id->bInterfaceSubClass,
                       id->bInterfaceProtocol, id->bNumEndpoints,
                       id->bInterfaceNumber == IF_COMMAND ? "   <- command channel" : "");
                /* Only AudioStreaming (subclass 2) has FORMAT_TYPE; in
                 * AudioControl the same 0x24/0x02 pair is INPUT_TERMINAL. */
                if (id->bInterfaceClass == LIBUSB_CLASS_AUDIO &&
                    id->bInterfaceSubClass == 2 && id->extra_length)
                    dump_audio_format(id->extra, id->extra_length);
                for (int e = 0; e < id->bNumEndpoints; e++) {
                    const struct libusb_endpoint_descriptor *ed = &id->endpoint[e];
                    printf("      ep 0x%02x %-3s %-11s wMaxPacketSize=%u bInterval=%u\n",
                           ed->bEndpointAddress,
                           (ed->bEndpointAddress & 0x80) ? "IN" : "OUT",
                           xfer_name(ed->bmAttributes), ed->wMaxPacketSize,
                           ed->bInterval);
                }
            }
        }
        libusb_free_config_descriptor(cfg);
    }
    return 0;
}

static int configure(void)
{
    int cfg = -1, rc;
    if (libusb_get_configuration(h, &cfg) == 0)
        printf("bConfigurationValue = %d\n", cfg);
    if (cfg == 1) { printf("already configured\n"); return 0; }
    rc = libusb_set_configuration(h, 1);
    printf("set_configuration(1): %s\n", rc ? libusb_error_name(rc) : "ok");
    if (rc) return 1;
    libusb_get_configuration(h, &cfg);
    printf("bConfigurationValue = %d\n", cfg);
    printf("note: usbaudiod now owns interfaces 2-4; use the audio tool for the codec\n");
    return 0;
}

/* Read-only: IN-direction requests only, so nothing is written to the device.
 * A stall means "no such request", which is the normal answer. */
static int sweep(void)
{
    static const unsigned char types[] = { 0xC1, 0xC0, 0xA1 };
    static const char *tname[]   = { "vendor/if", "vendor/dev", "class/if" };
    static const unsigned short wvals[] = { 0x0000, 0xFF01, 0xFF02, 0x0100, 0x0001 };
    unsigned char buf[64];
    int hits = 0;

    libusb_claim_interface(h, IF_COMMAND);
    printf("alive before sweep: %s\n", alive() ? "yes" : "NO");
    for (unsigned t = 0; t < sizeof types; t++) {
        for (unsigned w = 0; w < sizeof wvals / sizeof *wvals; w++) {
            for (int req = 0; req <= 0xFF; req++) {
                int rc;
                memset(buf, 0, sizeof buf);
                rc = libusb_control_transfer(h, types[t], req, wvals[w],
                                             IF_COMMAND, buf, sizeof buf, 300);
                if (rc == LIBUSB_ERROR_PIPE || rc == LIBUSB_ERROR_TIMEOUT) continue;
                printf("%-11s bRequest=0x%02x wValue=0x%04x -> %d byte(s):",
                       tname[t], req, wvals[w], rc);
                for (int i = 0; i < rc && i < 32; i++) printf(" %02x", buf[i]);
                printf("\n");
                hits++;
                if (!alive()) {
                    printf("*** device stopped answering after %s bRequest=0x%02x\n",
                           tname[t], req);
                    goto done;
                }
            }
        }
    }
done:
    printf("non-stall replies: %d\nalive after sweep: %s\n",
           hits, alive() ? "yes" : "NO");
    libusb_release_interface(h, IF_COMMAND);
    return 0;
}

static int get_response(void)
{
    unsigned char buf[64];
    int rc;
    libusb_claim_interface(h, IF_COMMAND);
    memset(buf, 0, sizeof buf);
    rc = libusb_control_transfer(h, 0xA1, GET_ENCAPSULATED_RESPONSE, 0,
                                 IF_COMMAND, buf, 2, 1000);
    if (rc < 0) printf("GET_ENCAPSULATED_RESPONSE: %s\n", libusb_error_name(rc));
    else if (rc == 0) printf("GET_ENCAPSULATED_RESPONSE: zero-length (nothing queued)\n");
    else hexdump("GET_ENCAPSULATED_RESPONSE", buf, rc);
    libusb_release_interface(h, IF_COMMAND);
    return 0;
}

/* Sends a command.  This writes to a telephony device whose opcode semantics
 * are NOT known, so it is deliberately explicit rather than convenient. */
static int send_command(const char *hex)
{
    unsigned char buf[64];
    int n = 0, rc, xfer;
    const char *p = hex;

    while (*p && n < (int)sizeof buf) {
        char *end;
        long v;
        while (*p == ' ' || *p == ',') p++;
        if (!*p) break;
        v = strtol(p, &end, 16);
        if (end == p || v < 0 || v > 0xff) {
            fprintf(stderr, "bad hex byte at \"%s\"\n", p);
            return 1;
        }
        buf[n++] = (unsigned char)v;
        p = end;
    }
    if (n == 0) { fprintf(stderr, "no bytes given\n"); return 1; }
    if (n != 3 && n != 9)
        printf("warning: driver only ever sends 3- or 9-byte commands; sending %d\n", n);

    libusb_claim_interface(h, IF_COMMAND);
    hexdump("SEND_ENCAPSULATED_COMMAND", buf, n);
    rc = libusb_control_transfer(h, 0x21, SEND_ENCAPSULATED_COMMAND, 0,
                                 IF_COMMAND, buf, n, 1000);
    printf("  send: %s\n", rc < 0 ? libusb_error_name(rc) : "accepted");
    if (rc >= 0) {
        unsigned char notify[16];
        /* The ACM control interface raises RESPONSE_AVAILABLE when a reply is
         * queued.  Absence of one is not an error; most commands are silent. */
        if (libusb_claim_interface(h, IF_ACM_CONTROL) == 0 &&
            libusb_interrupt_transfer(h, 0x81, notify, sizeof notify, &xfer, 1500) == 0)
            hexdump("  notification", notify, xfer);
        else
            printf("  notification: none\n");
        get_response();
    }
    printf("alive: %s\n", alive() ? "yes" : "NO");
    return rc < 0;
}

static int notify_listen(double secs)
{
    unsigned char b[16];
    int rc, n, got = 0, blocks = (int)(secs / 0.5 + 0.5);

    libusb_claim_interface(h, IF_ACM_CONTROL);
    for (int t = 0; t < blocks; t++) {
        rc = libusb_interrupt_transfer(h, 0x81, b, sizeof b, &n, 500);
        if (rc == 0) { char tag[32]; snprintf(tag, sizeof tag, "t=%5.1fs", t * 0.5);
                       hexdump(tag, b, n); got++; }
    }
    printf("notifications in %.1f s: %d\n", secs, got);
    libusb_release_interface(h, IF_ACM_CONTROL);
    return 0;
}

/* One register read, command and response in a single session.  Returns the
 * value 0-255, or -1.  Index 0 answers nothing on this device; every other
 * index tried answers, repeatably and in order. */
static int reg_read(unsigned char idx)
{
    unsigned char cmd[3] = { 0x80, 0, 0x00 }, in[8], notify[16];
    int xfer = 0, rc;

    cmd[1] = idx;
    if (libusb_control_transfer(h, 0x21, SEND_ENCAPSULATED_COMMAND, 0,
                               IF_COMMAND, cmd, 3, 1000) != 3)
        return -1;
    /* RESPONSE_AVAILABLE, if the ACM interface is ours; not required. */
    libusb_interrupt_transfer(h, 0x81, notify, sizeof notify, &xfer, 300);
    for (int try = 0; try < 4; try++) {
        rc = libusb_control_transfer(h, 0xA1, GET_ENCAPSULATED_RESPONSE, 0,
                                     IF_COMMAND, in, sizeof in, 1000);
        if (rc > 0) return in[0];
        if (rc < 0) return -1;
        usleep(20000);
    }
    return -1;
}

static int reg_write(unsigned char idx, unsigned char val)
{
    unsigned char cmd[3] = { 0x00, idx, val };
    return libusb_control_transfer(h, 0x21, SEND_ENCAPSULATED_COMMAND, 0,
                                   IF_COMMAND, cmd, 3, 1000) == 3 ? 0 : -1;
}

static int claim_command_path(void)
{
    libusb_claim_interface(h, IF_ACM_CONTROL);   /* optional: notifications */
    return libusb_claim_interface(h, IF_COMMAND);
}

static int cmd_regs(int argc, char **argv)
{
    static const unsigned char dflt[] = {
        0x01, 0x02, 0x03, 0x04, 0x05, 0x0a, 0x0f, 0x10, 0x11, 0x1a, 0x1e, 0x1f
    };
    if (claim_command_path() < 0) { fprintf(stderr, "claim failed\n"); return 1; }
    if (argc > 2) {
        for (int i = 2; i < argc; i++) {
            unsigned long idx = strtoul(argv[i], NULL, 16);
            printf("reg 0x%02lx = ", idx);
            int v = reg_read((unsigned char)idx);
            if (v < 0) printf("no response\n"); else printf("0x%02x\n", v);
        }
    } else {
        for (unsigned i = 0; i < sizeof dflt; i++) {
            int v = reg_read(dflt[i]);
            printf("reg 0x%02x = ", dflt[i]);
            if (v < 0) printf("no response\n"); else printf("0x%02x\n", v);
        }
    }
    return 0;
}

/* Writes to the line interface of a telephony device.  No write found so far
 * seizes the line, but this is a DAA's register file: read the header before
 * sweeping one on a line you care about. */
static int cmd_write(const char *sidx, const char *sval)
{
    unsigned long idx = strtoul(sidx, NULL, 16), val = strtoul(sval, NULL, 16);
    int before, after;

    if (idx > 0xff || val > 0xff) { fprintf(stderr, "index/value out of range\n"); return 1; }
    if (claim_command_path() < 0) { fprintf(stderr, "claim failed\n"); return 1; }
    before = reg_read((unsigned char)idx);
    if (reg_write((unsigned char)idx, (unsigned char)val) < 0) {
        fprintf(stderr, "write rejected\n");
        return 1;
    }
    usleep(50000);
    after = reg_read((unsigned char)idx);
    printf("reg 0x%02lx: 0x%02x -> wrote 0x%02lx -> reads 0x%02x%s\n",
           idx, before & 0xff, val, after & 0xff,
           after == (int)val ? "" : "   (DID NOT TAKE)");
    printf("alive: %s\n", alive() ? "yes" : "NO");
    return 0;
}

static int cmd_hook(const char *what)
{
    int on = !strcmp(what, "on"), v;

    if (strcmp(what, "on") && strcmp(what, "off")) {
        fprintf(stderr, "--hook takes on or off\n");
        return 1;
    }
    if (claim_command_path() < 0) { fprintf(stderr, "claim failed\n"); return 1; }
    v = reg_read(5);
    if (v < 0) { fprintf(stderr, "cannot read register 5\n"); return 1; }
    v = on ? (v | 0x08) : (v & ~0x08);
    if (reg_write(5, (unsigned char)v) < 0) { fprintf(stderr, "write rejected\n"); return 1; }
    usleep(50000);
    printf("hook %s: register 5 = 0x%02x\n", what, reg_read(5) & 0xff);
    printf("now capture: ./apple_usb_modem_audio capture 9600 2\n");
    return 0;
}

static void usage(const char *argv0)
{
    fprintf(stderr,
        "usage: %s [--descriptors | --configure | --sweep | --get |\n"
        "           --send \"<hex bytes>\" | --notify [seconds] |\n"
        "           --regs [idx ...] | --read <idx> | --write <idx> <val> |\n"
        "           --hook on|off]\n"
        "\n"
        "  --descriptors  (default) dump the configuration, interfaces, endpoints\n"
        "                 and the audio rate list; does not claim anything\n"
        "  --configure    set bConfigurationValue 1 -- do this FIRST, or every\n"
        "                 request below stalls\n"
        "  --sweep        read-only IN-direction request sweep of interface 1\n"
        "  --get          one GET_ENCAPSULATED_RESPONSE\n"
        "  --send         one SEND_ENCAPSULATED_COMMAND; WRITES to the device,\n"
        "                 and the opcodes are not understood -- see the header\n"
        "  --notify       listen on the interface 0 interrupt endpoint\n"
        "  --regs         read the registers that are known to answer (or those given)\n"
        "  --read         read one register, hex index\n"
        "  --write        write one register and read it back; WRITES to the\n"
        "                 line interface\n"
        "  --hook         set or clear register 5 bit 3, which powers the analogue\n"
        "                 RECEIVE path; with it off the codec reads -32768.  This\n"
        "                 is NOT the hook -- it does not seize the line\n",
        argv0);
}

int main(int argc, char **argv)
{
    const char *mode = (argc > 1) ? argv[1] : "--descriptors";
    libusb_device **list;
    libusb_device *dev = NULL;
    ssize_t n;
    int rc = 1;

    if (!strcmp(mode, "-h") || !strcmp(mode, "--help")) { usage(argv[0]); return 0; }
    if (libusb_init(NULL) < 0) { fprintf(stderr, "libusb_init failed\n"); return 1; }

    n = libusb_get_device_list(NULL, &list);
    for (ssize_t i = 0; i < n; i++) {
        struct libusb_device_descriptor dd;
        if (libusb_get_device_descriptor(list[i], &dd) == 0 &&
            dd.idVendor == VID && dd.idProduct == PID) { dev = list[i]; break; }
    }
    if (!dev) {
        fprintf(stderr, "no %04x:%04x on the bus\n", VID, PID);
        libusb_free_device_list(list, 1);
        libusb_exit(NULL);
        return 1;
    }

    if (!strcmp(mode, "--descriptors")) {
        rc = cmd_descriptors(dev);
        libusb_free_device_list(list, 1);
        libusb_exit(NULL);
        return rc;
    }

    if (libusb_open(dev, &h) < 0) {
        fprintf(stderr, "open failed (in use, or no permission)\n");
        libusb_free_device_list(list, 1);
        libusb_exit(NULL);
        return 1;
    }
    libusb_free_device_list(list, 1);

    if (!strcmp(mode, "--configure")) rc = configure();
    else {
        int cfg = -1;
        libusb_get_configuration(h, &cfg);
        if (cfg != 1) {
            printf("device is at configuration %d; setting it first\n", cfg);
            configure();
        }
        if      (!strcmp(mode, "--sweep"))  rc = sweep();
        else if (!strcmp(mode, "--get"))    rc = get_response();
        else if (!strcmp(mode, "--send"))   rc = (argc > 2) ? send_command(argv[2])
                                                           : (usage(argv[0]), 1);
        else if (!strcmp(mode, "--notify")) rc = notify_listen((argc > 2) ? atof(argv[2]) : 6.0);
        else if (!strcmp(mode, "--regs"))   rc = cmd_regs(argc, argv);
        else if (!strcmp(mode, "--read"))   rc = (argc > 2) ? cmd_regs(argc, argv)
                                                           : (usage(argv[0]), 1);
        else if (!strcmp(mode, "--write"))  rc = (argc > 3) ? cmd_write(argv[2], argv[3])
                                                           : (usage(argv[0]), 1);
        else if (!strcmp(mode, "--hook"))   rc = (argc > 2) ? cmd_hook(argv[2])
                                                           : (usage(argv[0]), 1);
        else { usage(argv[0]); rc = 1; }
    }

    libusb_close(h);
    libusb_exit(NULL);
    return rc;
}
