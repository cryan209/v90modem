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
 * Opcode (payload byte 0) values the driver emits: 0x00 0x02 0x80 0x90 0xd0.
 * What they MEAN is not established -- see docs/apple_usb_modem_sm56.md.
 *
 * TRAPS, both of which cost a session:
 *
 *  - The device sits at bConfigurationValue 0 until something sets it, and an
 *    unconfigured device stalls EVERY request including standard
 *    GET_INTERFACE, while ioreg shows it with no interface children at all.
 *    That reads exactly like dead hardware.  `--configure` first.
 *  - libusb's darwin backend serves device and config descriptors out of the
 *    IOKit cache, so those keep succeeding after the device has stopped
 *    answering anything.  Liveness here is a STRING descriptor, which has to
 *    reach the wire.  (Same trap as docs/hsf_usb_daa.md.)
 *
 * Build: make apple_usb_modem_probe
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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

static void usage(const char *argv0)
{
    fprintf(stderr,
        "usage: %s [--descriptors | --configure | --sweep | --get |\n"
        "           --send \"<hex bytes>\" | --notify [seconds]]\n"
        "\n"
        "  --descriptors  (default) dump the configuration, interfaces, endpoints\n"
        "                 and the audio rate list; does not claim anything\n"
        "  --configure    set bConfigurationValue 1 -- do this FIRST, or every\n"
        "                 request below stalls\n"
        "  --sweep        read-only IN-direction request sweep of interface 1\n"
        "  --get          one GET_ENCAPSULATED_RESPONSE\n"
        "  --send         one SEND_ENCAPSULATED_COMMAND; WRITES to the device,\n"
        "                 and the opcodes are not understood -- see the header\n"
        "  --notify       listen on the interface 0 interrupt endpoint\n", argv0);
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
        else { usage(argv[0]); rc = 1; }
    }

    libusb_close(h);
    libusb_exit(NULL);
    return rc;
}
