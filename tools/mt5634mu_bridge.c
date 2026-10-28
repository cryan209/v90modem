/*
 * mt5634mu_bridge.c -- expose the MultiTech MT5634MU USB modem as a PTY.
 *
 * The MT5634MU ("MultiModemUSB", USB 06e0:f103, Lucent chipset, firmware
 * U052099f) gives macOS no usable serial port.  Its configuration 2 claims to
 * be CDC-ACM, but the firmware stalls every CDC class request, has no EP 0x86,
 * and sends its replies on EP 0x84, where AppleUSBACMControl swallows them as
 * notifications, so /dev/cu.usbmodem* looks dead.  Configuration 1 is
 * vendor-specific and works:
 *
 *   interface 1, bulk-OUT EP 0x02 (16-byte packets)   host -> modem bytes
 *   interface 1, interrupt-IN EP 0x84 (63-byte packets) modem -> host bytes,
 *       each packet = ONE status byte (0xc0 idle) + payload
 *
 * This bridge seizes the device, selects configuration 1, and copies bytes
 * between those pipes and a pseudo-terminal, so minicom, tools/cx_at.py and
 * the soak/dial scripts drive it like any serial modem.  The status byte is
 * stripped from the data; it stayed 0xc0 through a whole V.34 call, but any
 * change is logged bit by bit.
 *
 * Vendor control requests (type 0x41/0xc1, wIndex 1) found by probing, see
 * docs/mt5634mu_usb_modem.md:
 *   OUT 0x00 wValue 0/1/2  data-path gate, kept in NVRAM across power cycles.
 *                          0 makes bulk-OUT NAK forever, so the bridge always
 *                          writes 1 at startup.
 *   IN  0x1a wLength 4     2 bytes; bit 0 set = no carrier (independent of &C).
 *                          wLength 2 times out (firmware quirk).
 *   IN  0x1b               the live S-register file, S0..S253.
 * Carrier is polled with 0x1a and logged; the hangup on close is skipped when
 * there is no carrier.
 *
 * Hangup: a PTY carries no DTR, so a DTE's DTR drop is invisible here.  When
 * the last process closes the slave side, the bridge treats that as DTR off
 * and, if there is carrier, sends the escape (1 s guard, "+++", 1 s guard)
 * followed by "ATH\r", unless --no-hangup-on-close is given.
 *
 * Build:  make mt5634mu_bridge
 *   or:   cc -O2 -Wall -o mt5634mu_bridge tools/mt5634mu_bridge.c \
 *             -framework IOKit -framework CoreFoundation
 * Run:    ./mt5634mu_bridge --pty-link /tmp/mt5634mu
 *         minicom -D /tmp/mt5634mu
 *
 * The interrupt pipe is read with a blocking ReadPipe on its own thread:
 * ReadPipeTO returns kIOReturnBadArgument (e00002c2) on interrupt pipes.
 */
#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/IOCFPlugIn.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/usb/IOUSBLib.h>

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define MT_PRODUCT_NAME "MultiModemUSB"
#define MT_CONFIG 1
#define MT_EP_OUT 0x02
#define MT_EP_IN 0x84
#define MT_IN_PACKET 64
#define MT_OUT_CHUNK 64
#define MT_STATUS_IDLE 0xc0

typedef IOUSBDeviceInterface650 **usb_dev_t;
typedef IOUSBInterfaceInterface650 **usb_intf_t;

static usb_dev_t g_dev;
static usb_intf_t g_intf;
static UInt8 g_pipe_out, g_pipe_in;
static int g_master = -1;
static volatile sig_atomic_t g_quit;
/* Set while some process has the slave open.  Modem output that arrives with
 * no DTE (the OK to a hangup's ATH) is dropped, as a closed tty would. */
static volatile int g_dte_open;
static int g_verbose;
static const char *g_link;
static struct timeval g_t0;

/* Byte counters, reported on exit and by --stats. */
static uint64_t g_bytes_to_modem, g_bytes_from_modem, g_in_packets;

static void logmsg(const char *fmt, ...)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    double t = (double) (tv.tv_sec - g_t0.tv_sec)
        + (tv.tv_usec - g_t0.tv_usec) / 1e6;
    fprintf(stderr, "[mt5634mu %9.3f] ", t);
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
}

static void hexdump(const char *dir, const UInt8 *p, UInt32 n)
{
    char line[3 * MT_IN_PACKET + 1];
    UInt32 i, k = 0;
    for (i = 0; i < n && k + 4 < sizeof line; i++)
        k += (UInt32) snprintf(line + k, sizeof line - k, "%02x ", p[i]);
    line[k] = '\0';
    logmsg("%s %u: %s", dir, (unsigned) n, line);
}

static void on_signal(int sig)
{
    (void) sig;
    g_quit = 1;
}

/* ---- USB ---------------------------------------------------------------- */

static IOReturn open_device(void)
{
    io_service_t svc = IOServiceGetMatchingService(kIOMainPortDefault,
        IOServiceNameMatching(MT_PRODUCT_NAME));
    if (!svc) {
        logmsg("no USB device named \"%s\" (is the modem plugged in?)",
            MT_PRODUCT_NAME);
        return kIOReturnNotFound;
    }

    IOCFPlugInInterface **plug = NULL;
    SInt32 score;
    IOReturn kr = IOCreatePlugInInterfaceForService(svc,
        kIOUSBDeviceUserClientTypeID, kIOCFPlugInInterfaceID, &plug, &score);
    IOObjectRelease(svc);
    if (kr || !plug) {
        logmsg("device plug-in: %08x", kr);
        return kr ? kr : kIOReturnError;
    }
    (*plug)->QueryInterface(plug, CFUUIDGetUUIDBytes(kIOUSBDeviceInterfaceID650),
        (LPVOID *) &g_dev);
    IODestroyPlugInInterface(plug);
    if (!g_dev)
        return kIOReturnError;

    UInt16 vid = 0, pid = 0;
    (*g_dev)->GetDeviceVendor(g_dev, &vid);
    (*g_dev)->GetDeviceProduct(g_dev, &pid);
    if (vid != 0x06e0 || pid != 0xf103)
        logmsg("warning: \"%s\" is %04x:%04x, expected 06e0:f103",
            MT_PRODUCT_NAME, vid, pid);

    kr = (*g_dev)->USBDeviceOpenSeize(g_dev);
    if (kr) {
        logmsg("USBDeviceOpenSeize: %08x", kr);
        return kr;
    }
    /* Config 2 is the fake CDC-ACM descriptor; only config 1 works. */
    kr = (*g_dev)->SetConfiguration(g_dev, MT_CONFIG);
    if (kr) {
        logmsg("SetConfiguration(%d): %08x", MT_CONFIG, kr);
        return kr;
    }

    IOUSBFindInterfaceRequest req = {
        kIOUSBFindInterfaceDontCare, kIOUSBFindInterfaceDontCare,
        kIOUSBFindInterfaceDontCare, kIOUSBFindInterfaceDontCare
    };
    io_iterator_t it;
    kr = (*g_dev)->CreateInterfaceIterator(g_dev, &req, &it);
    if (kr)
        return kr;
    io_service_t isvc;
    while ((isvc = IOIteratorNext(it))) {
        IOCFPlugInInterface **ipl = NULL;
        usb_intf_t intf = NULL;
        kr = IOCreatePlugInInterfaceForService(isvc,
            kIOUSBInterfaceUserClientTypeID, kIOCFPlugInInterfaceID, &ipl, &score);
        IOObjectRelease(isvc);
        if (kr || !ipl)
            continue;
        (*ipl)->QueryInterface(ipl,
            CFUUIDGetUUIDBytes(kIOUSBInterfaceInterfaceID650), (LPVOID *) &intf);
        IODestroyPlugInInterface(ipl);
        if (!intf)
            continue;
        UInt8 num = 0xff;
        (*intf)->GetInterfaceNumber(intf, &num);
        if (num == 1 && !g_intf)
            g_intf = intf;
        else
            (*intf)->Release(intf);
    }
    IOObjectRelease(it);
    if (!g_intf) {
        logmsg("configuration %d has no interface 1", MT_CONFIG);
        return kIOReturnNotFound;
    }
    kr = (*g_intf)->USBInterfaceOpenSeize(g_intf);
    if (kr) {
        logmsg("USBInterfaceOpenSeize: %08x", kr);
        return kr;
    }

    UInt8 nep = 0;
    (*g_intf)->GetNumEndpoints(g_intf, &nep);
    for (UInt8 p = 1; p <= nep; p++) {
        UInt8 dir, num, type, interval;
        UInt16 mps;
        if ((*g_intf)->GetPipeProperties(g_intf, p, &dir, &num, &type, &mps,
                &interval))
            continue;
        UInt8 addr = (UInt8) (num | (dir == kUSBIn ? 0x80 : 0));
        if (g_verbose)
            logmsg("pipe %u: EP 0x%02x type %u mps %u", p, addr, type, mps);
        if (addr == MT_EP_OUT)
            g_pipe_out = p;
        else if (addr == MT_EP_IN)
            g_pipe_in = p;
    }
    if (!g_pipe_out || !g_pipe_in) {
        logmsg("interface 1 lacks EP 0x%02x / 0x%02x", MT_EP_OUT, MT_EP_IN);
        return kIOReturnNotFound;
    }
    return kIOReturnSuccess;
}

static void close_device(void)
{
    if (g_intf) {
        (*g_intf)->USBInterfaceClose(g_intf);
        (*g_intf)->Release(g_intf);
        g_intf = NULL;
    }
    if (g_dev) {
        (*g_dev)->USBDeviceClose(g_dev);
        (*g_dev)->Release(g_dev);
        g_dev = NULL;
    }
}

static int usb_write(const UInt8 *p, UInt32 n)
{
    while (n) {
        UInt32 k = n > MT_OUT_CHUNK ? MT_OUT_CHUNK : n;
        /* The modem NAKs while its buffer is full; that is the flow control. */
        IOReturn kr = (*g_intf)->WritePipeTO(g_intf, g_pipe_out, (void *) p, k,
            0, 5000);
        if (kr == kIOReturnTimeout || kr == kIOUSBTransactionTimeout) {
            logmsg("bulk-OUT timed out (modem not accepting data)");
            (*g_intf)->ClearPipeStallBothEnds(g_intf, g_pipe_out);
            continue;
        }
        if (kr) {
            logmsg("bulk-OUT: %08x", kr);
            return -1;
        }
        if (g_verbose > 1)
            hexdump("->", p, k);
        g_bytes_to_modem += k;
        p += k;
        n -= k;
    }
    return 0;
}

static void sleep_ms(int ms)
{
    struct timespec ts = { ms / 1000, (long) (ms % 1000) * 1000000L };
    while (nanosleep(&ts, &ts) && errno == EINTR && !g_quit)
        ;
}

/* ---- status byte -------------------------------------------------------- */

/* The meaning of the status byte is not documented.  Bits are logged by
 * position; names are only attached once watched across a live event (see
 * docs/mt5634mu_usb_modem.md). */
static UInt8 g_status = MT_STATUS_IDLE;

/* Carrier from IN request 0x1a: -1 unknown, 0 off, 1 on. */
static volatile int g_carrier = -1;

static IOReturn vendor_in(UInt8 req, UInt16 len, UInt8 *buf, UInt32 *done)
{
    IOUSBDevRequestTO q = { 0xc1, req, 0, 1, len, buf, 0, 300, 300 };
    IOReturn kr = (*g_dev)->DeviceRequestTO(g_dev, &q);
    *done = q.wLenDone;
    if (kr)
        (*g_dev)->USBDeviceAbortPipeZero(g_dev);
    return kr;
}

static IOReturn vendor_out(UInt8 req, UInt16 value)
{
    IOUSBDevRequestTO q = { 0x41, req, value, 1, 0, NULL, 0, 500, 500 };
    return (*g_dev)->DeviceRequestTO(g_dev, &q);
}

/* EP0 replies are occasionally stale around a hangup, so a reading only
 * counts when it is exactly two bytes and matches the one before it. */
static void *carrier_thread(void *arg)
{
    (void) arg;
    int prev = -1;
    while (!g_quit) {
        UInt8 b[4] = { 0 };
        UInt32 n = 0;
        int cur = -1;
        if (vendor_in(0x1a, sizeof b, b, &n) == kIOReturnSuccess && n == 2)
            cur = (b[0] & 1) ? 0 : 1;
        if (cur >= 0 && cur == prev && cur != g_carrier) {
            if (g_carrier >= 0 || cur == 1)
                logmsg("carrier %s", cur ? "ON" : "off");
            g_carrier = cur;
        }
        prev = cur;
        sleep_ms(125);
    }
    return NULL;
}

static void status_changed(UInt8 s)
{
    UInt8 diff = (UInt8) (s ^ g_status);
    char bits[8 * 6 + 1];
    int k = 0;
    for (int b = 7; b >= 0; b--)
        if (diff & (1u << b))
            k += snprintf(bits + k, sizeof bits - k, " b%d%c", b,
                (s >> b) & 1 ? '+' : '-');
    logmsg("status 0x%02x -> 0x%02x:%s", g_status, s, bits);
    g_status = s;
}

/* ---- modem -> PTY thread ------------------------------------------------ */

static void *reader_thread(void *arg)
{
    (void) arg;
    UInt8 buf[MT_IN_PACKET];
    while (!g_quit) {
        UInt32 len = sizeof buf;
        IOReturn kr = (*g_intf)->ReadPipe(g_intf, g_pipe_in, buf, &len);
        if (kr == kIOReturnAborted) {
            if (g_quit)
                break;
            (*g_intf)->ClearPipeStallBothEnds(g_intf, g_pipe_in);
            continue;
        }
        if (kr) {
            logmsg("interrupt-IN: %08x; giving up", kr);
            g_quit = 1;
            break;
        }
        if (len == 0)
            continue;
        g_in_packets++;
        if (g_verbose > 1)
            hexdump("<-", buf, len);
        if (buf[0] != g_status)
            status_changed(buf[0]);
        UInt32 off = 1;
        while (off < len && g_dte_open) {
            ssize_t w = write(g_master, buf + off, len - off);
            if (w > 0) {
                off += (UInt32) w;
                g_bytes_from_modem += (uint64_t) w;
            } else if (w < 0 && errno == EAGAIN) {
                /* DTE not reading; wait rather than drop. */
                struct pollfd pf = { g_master, POLLOUT, 0 };
                poll(&pf, 1, 100);
                if (g_quit)
                    break;
            } else {
                /* No slave open (EIO): nobody to deliver to. */
                break;
            }
        }
    }
    return NULL;
}

/* ---- PTY ---------------------------------------------------------------- */

static int open_pty(char *name, size_t namelen)
{
    int m = posix_openpt(O_RDWR | O_NOCTTY);
    if (m < 0 || grantpt(m) || unlockpt(m)) {
        perror("posix_openpt");
        return -1;
    }
    const char *sn = ptsname(m);
    if (!sn) {
        perror("ptsname");
        return -1;
    }
    snprintf(name, namelen, "%s", sn);

    /* Put the line discipline in raw mode, as a serial port would be after
     * the DTE configures it.  The termios state belongs to the pty pair and
     * outlives this temporary open of the slave. */
    int s = open(name, O_RDWR | O_NOCTTY);
    if (s >= 0) {
        struct termios t;
        if (tcgetattr(s, &t) == 0) {
            cfmakeraw(&t);
            cfsetspeed(&t, B115200);
            tcsetattr(s, TCSANOW, &t);
        }
        close(s);
    }
    fcntl(m, F_SETFL, fcntl(m, F_GETFL) | O_NONBLOCK);
    return m;
}

/* DTE closed the port: the PTY equivalent of DTR off (V.250 &D2). */
static void hangup_modem(void)
{
    if (g_carrier == 0) {
        logmsg("DTE closed the port (no carrier, nothing to hang up)");
        return;
    }
    logmsg("DTE closed the port: escape + ATH");
    sleep_ms(1100);
    if (g_quit || usb_write((const UInt8 *) "+++", 3))
        return;
    sleep_ms(1100);
    if (!g_quit)
        usb_write((const UInt8 *) "ATH\r", 4);
}

static void cleanup_link(void)
{
    if (g_link)
        unlink(g_link);
}

static void usage(const char *argv0)
{
    fprintf(stderr,
        "usage: %s [--pty-link PATH] [--no-hangup-on-close] [-v [-v]]\n"
        "  --pty-link PATH         symlink PATH to the slave PTY (e.g. /tmp/mt5634mu)\n"
        "  --no-hangup-on-close    don't send +++/ATH when the DTE closes the port\n"
        "  -v                      log pipe setup; -v -v hex-dumps every transfer\n",
        argv0);
}

int main(int argc, char **argv)
{
    int hangup_on_close = 1;
    static const struct option opts[] = {
        { "pty-link", required_argument, NULL, 'l' },
        { "no-hangup-on-close", no_argument, NULL, 'n' },
        { "verbose", no_argument, NULL, 'v' },
        { "help", no_argument, NULL, 'h' },
        { NULL, 0, NULL, 0 }
    };
    int c;
    while ((c = getopt_long(argc, argv, "l:nvh", opts, NULL)) != -1) {
        switch (c) {
        case 'l': g_link = optarg; break;
        case 'n': hangup_on_close = 0; break;
        case 'v': g_verbose++; break;
        default: usage(argv[0]); return c == 'h' ? 0 : 2;
        }
    }
    gettimeofday(&g_t0, NULL);

    if (open_device() != kIOReturnSuccess) {
        close_device();
        return 1;
    }

    /* A stored 0 here leaves the modem deaf after every power-up. */
    IOReturn gk = vendor_out(0x00, 1);
    if (gk)
        logmsg("data-path gate (OUT 0x00 = 1): %08x", gk);

    char slave[128];
    g_master = open_pty(slave, sizeof slave);
    if (g_master < 0) {
        close_device();
        return 1;
    }
    if (g_link) {
        unlink(g_link);
        if (symlink(slave, g_link)) {
            perror(g_link);
            g_link = NULL;
        }
    }
    printf("%s\n", g_link ? g_link : slave);
    fflush(stdout);
    logmsg("bridging %s%s%s <-> USB 06e0:f103 config %d IF1 (EP 0x%02x/0x%02x)",
        slave, g_link ? " via " : "", g_link ? g_link : "", MT_CONFIG,
        MT_EP_OUT, MT_EP_IN);

    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_signal;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGHUP, &sa, NULL);
    signal(SIGPIPE, SIG_IGN);

    pthread_t rd;
    pthread_create(&rd, NULL, reader_thread, NULL);
    pthread_t cd;
    pthread_create(&cd, NULL, carrier_thread, NULL);

    /* PTY -> modem.  POLLHUP on the master means no process has the slave
     * open; an open-to-closed transition is the DTE hanging up. */
    UInt8 buf[256];
    while (!g_quit) {
        struct pollfd pf = { g_master, POLLIN, 0 };
        int r = poll(&pf, 1, 200);
        if (r < 0) {
            if (errno == EINTR)
                continue;
            perror("poll");
            break;
        }
        int hup = r > 0 && (pf.revents & (POLLHUP | POLLERR));
        if (!hup && !g_dte_open) {
            g_dte_open = 1;
            if (g_verbose)
                logmsg("DTE opened the port");
        }
        if (r > 0 && (pf.revents & POLLIN)) {
            ssize_t n = read(g_master, buf, sizeof buf);
            if (n > 0) {
                if (usb_write(buf, (UInt32) n))
                    break;
                continue;
            }
        }
        if (hup) {
            if (g_dte_open) {
                g_dte_open = 0;
                if (hangup_on_close)
                    hangup_modem();
                else
                    logmsg("DTE closed the port");
            }
            /* No slave open: the master reports POLLHUP continuously. */
            sleep_ms(100);
        }
    }

    g_quit = 1;
    (*g_intf)->AbortPipe(g_intf, g_pipe_in);
    pthread_join(rd, NULL);
    pthread_join(cd, NULL);
    logmsg("exit: %llu bytes to modem, %llu from modem in %llu packets",
        (unsigned long long) g_bytes_to_modem,
        (unsigned long long) g_bytes_from_modem,
        (unsigned long long) g_in_packets);
    cleanup_link();
    close(g_master);
    close_device();
    return 0;
}
