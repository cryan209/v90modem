/*
 * hsf_fxo_probe.c -- bring up the Conexant HSF USB modem and report what it does.
 *
 * Replaces the throwaway Python probes used to work this device out.  Needs the
 * hardware (0572:1300) attached; it is deliberately NOT part of `make test`.
 *
 *   make hsf_fxo_probe
 *   ./tools/hsf_extract_rom.py <path>/c2firmware.h     # once, makes hsf_rom_image.bin
 *   ./hsf_fxo_probe                                    # info only
 *   ./hsf_fxo_probe --wait 60 --load --script 9        # THE ONE THAT WORKS
 *   ./hsf_fxo_probe --load                             # load firmware if needed
 *   ./hsf_fxo_probe --load --stream 5                  # then stream for 5s
 *   ./hsf_fxo_probe --load --script 9                  # send one script
 *   ./hsf_fxo_probe --load --start-codec --stream 5    # script 9 completion, then 5
 *   ./hsf_fxo_probe --load --start-codec --hook off --stream 5
 *
 * --wait is not a convenience.  The CD2 bootloader answers EP0 for only about
 * THREE SECONDS after it enumerates and then goes silent for good (measured
 * 2026-09-01: ~120 successful GET_INFROMATION at 25 ms intervals, then nothing).
 * The vendor driver uploads firmware on match, within milliseconds; a probe
 * started by hand is minutes late and finds a device that looks wedged and is
 * merely finished waiting.  --wait polls for the window and acts inside it, so
 * run it FIRST and replug the device while it waits.
 *
 * --script remains useful for exercising individual firmware operations.  The
 * normal stream start is --start-codec; it mirrors the closed Linux driver.
 */

#include "hsf_fxo.h"

#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

static unsigned long g_rx_packets;
static unsigned char g_first[64];
static size_t        g_first_len;
static unsigned long g_hist[256];
static unsigned long g_rx_len_hist[257];
static bool          g_feed_tx;
static FILE         *g_rx_file;
static bool          g_dtmf_tx;
static double        g_dtmf_phase_lo;
static double        g_dtmf_phase_hi;
static double        g_dtmf_freq_lo;
static double        g_dtmf_freq_hi;
static int           g_tx_slot = -1;
static unsigned      g_tx_prime_blocks;
static unsigned      g_tx_blocks;
static bool          g_tx_pace_rx = true;	/* credit TX from RX, as the driver does */
static size_t        g_tx_block = 128;	/* hsfusbcd2167_ sets ctx+0x876 = 0x80 */
static size_t        g_rx_credit;
static uint8_t       g_script1_reply[2];
static int           g_script1_mode;
static bool          g_script1_seen;

static void fill_tx(uint8_t *buf)
{
	memset(buf, 0, g_tx_block);
	g_tx_blocks++;
	if (g_tx_blocks <= g_tx_prime_blocks)
		return;
	if (!g_dtmf_tx)
		return;
	/* RX strongly suggests four-byte frames.  Put the same signed sample in
	 * both candidate 16-bit slots so either possible audio lane carries the
	 * digit without putting two different time instants into one frame. */
	int16_t *s = (int16_t *)buf;
	for (size_t i = 0; i < g_tx_block / 4; i++) {
		int16_t v = (int16_t)(3500.0 * (sin(g_dtmf_phase_lo) +
						 sin(g_dtmf_phase_hi)));
		if (g_tx_slot < 0 || g_tx_slot == 0)
			s[2*i] = v;
		if (g_tx_slot < 0 || g_tx_slot == 1)
			s[2*i + 1] = v;
		g_dtmf_phase_lo += 2.0 * M_PI * g_dtmf_freq_lo / 10666.666667;
		g_dtmf_phase_hi += 2.0 * M_PI * g_dtmf_freq_hi / 10666.666667;
		if (g_dtmf_phase_lo >= 2.0 * M_PI) g_dtmf_phase_lo -= 2.0 * M_PI;
		if (g_dtmf_phase_hi >= 2.0 * M_PI) g_dtmf_phase_hi -= 2.0 * M_PI;
	}
}

/*
 * The driver's pacing, which is not what this probe originally did.
 *
 * Both completions land in the same pump.  hsfusbcd2184_ (RX done) credits the
 * received byte count, divides by 0x876 = 128, and adds the quotient to
 * ctx+0x8ac; hsfusbcd2186_ (TX done) does the same for the transmitted count
 * into ctx+0x8a6; both then tail-jump to hsfusbcd2212_.  And hsfusbcd2212_
 * RETURNS IMMEDIATELY when ctx+0x8ac is zero -- no RX units pending, no work --
 * otherwise submitting exactly one RX and one TX and decrementing both.
 *
 * So TX is credited by RX: 128 bytes out for every 128 bytes in, which is what
 * a synchronous codec wants.  Feeding TX from its own completions alone, as
 * this probe did, free-runs a four-deep pipeline against a device that is
 * clocking at a fixed rate, and overruns it.
 */
static void tx_pump(struct hsf_dev *d)
{
	while (g_rx_credit >= g_tx_block) {
		uint8_t next[256];
		fill_tx(next);
		if (hsf_fxo_tx_submit(d, next, g_tx_block) < 0)
			break;
		g_rx_credit -= g_tx_block;
	}
}

static void on_tx_done(size_t len, void *user)
{
	struct hsf_dev *d = user;
	(void)len;
	if (!g_feed_tx)
		return;
	if (g_tx_pace_rx) {
		/* A TX completion also re-enters the pump, but the pump does
		 * nothing without RX credit -- so this is not a refill site. */
		tx_pump(d);
		return;
	}
	uint8_t next[256];
	fill_tx(next);
	(void)hsf_fxo_tx_submit(d, next, g_tx_block);
}

static void on_rx(const uint8_t *data, size_t len, void *user)
{
	(void)user;
	g_rx_packets++;
	if (len <= 256)
		g_rx_len_hist[len]++;
	if (g_rx_file)
		fwrite(data, 1, len, g_rx_file);
	for (size_t i = 0; i < len; i++)
		g_hist[data[i]]++;
	if (g_first_len < sizeof(g_first)) {
		size_t n = sizeof(g_first) - g_first_len;
		if (n > len)
			n = len;
		memcpy(g_first + g_first_len, data, n);
		g_first_len += n;
	}
	if (g_feed_tx && g_tx_pace_rx) {
		g_rx_credit += len;
		tx_pump(user);
	}
}

static unsigned long g_ring_events;
static pthread_mutex_t g_notify_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_notify_cond = PTHREAD_COND_INITIALIZER;
static unsigned        g_script_done[34];
static uint8_t         g_script_status[34];

static void on_notify(const struct hsf_notification *n, void *user)
{
	(void)user;
	if (n->wValue == 1 && n->data_len >= 2 && n->data[0] < 34) {
		/*
		 * Most scripts reply with two bytes, <id> <status>.  SCRIPT 1 IS
		 * DIFFERENT: it is hsfusbcd2241_'s stream-open query and replies
		 * with FOUR, and hsfusbcd2239_ is called with a length of 4 to
		 * read it.  Live this part answers 01 00 00 01, and the driver
		 * takes ctx+0x911 (byte 1) and ctx+0x912 (byte 2) as payload --
		 * ctx+0x68, the mode deciding whether scripts 11 and 12 do
		 * anything, is set from (byte 2 == 1) -- which leaves the LAST
		 * byte as the status.
		 *
		 * Reading byte 1 as the status for a four-byte reply therefore
		 * reports a successful script 1 as a failure, which is exactly
		 * what it did the first time it was sent in order.
		 */
		pthread_mutex_lock(&g_notify_lock);
		g_script_done[n->data[0]]++;
		g_script_status[n->data[0]] = n->data[n->data_len - 1];
		if (n->data[0] == 1 && n->data_len >= 4) {
			g_script1_reply[0] = n->data[1];
			g_script1_reply[1] = n->data[2];
			g_script1_mode = (n->data[2] == 1);
			g_script1_seen = true;
		}
		pthread_cond_broadcast(&g_notify_cond);
		pthread_mutex_unlock(&g_notify_lock);
	}
	/* Ring arrives as a half-cycle toggle at 25 Hz, so printing every one
	 * buries everything else -- count them and print the first. */
	if (n->wValue == 2 && n->data_len >= 1 && n->data[0] == HSF_EVENT_RING) {
		if (g_ring_events++)
			return;
		printf("  RING detected (event 0x%02x); further ring edges counted only\n",
		       n->data[0]);
		fflush(stdout);
		return;
	}
	printf("  NOTIFICATION bmRequestType=0x%02x code=0x%02x "
	       "wValue=0x%04x wIndex=%u wLength=%u",
	       n->bmRequestType, n->bNotification, n->wValue, n->wIndex, n->wLength);
	for (size_t i = 0; i < n->data_len; i++)
		printf("%s%02x", i ? "" : "  data=", n->data[i]);
	printf("\n");
	fflush(stdout);
}

static int wait_script(unsigned id, unsigned before, int timeout_ms)
{
	struct timespec until;
	clock_gettime(CLOCK_REALTIME, &until);
	until.tv_sec += timeout_ms / 1000;
	until.tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
	if (until.tv_nsec >= 1000000000L) {
		until.tv_sec++;
		until.tv_nsec -= 1000000000L;
	}

	pthread_mutex_lock(&g_notify_lock);
	while (id < 34 && g_script_done[id] == before) {
		int r = pthread_cond_timedwait(&g_notify_cond, &g_notify_lock, &until);
		if (r == ETIMEDOUT) {
			pthread_mutex_unlock(&g_notify_lock);
			return -ETIMEDOUT;
		}
	}
	int status = id < 34 ? g_script_status[id] : 0;
	pthread_mutex_unlock(&g_notify_lock);
	return status == 0x01 ? 0 : -EIO;
}

static unsigned script_count(unsigned id)
{
	pthread_mutex_lock(&g_notify_lock);
	unsigned n = id < 34 ? g_script_done[id] : 0;
	pthread_mutex_unlock(&g_notify_lock);
	return n;
}

int main(int argc, char **argv)
{
	bool do_load = false;
	bool do_start = false;
	int  stream_secs = 0;
	int  script_ids[16];
	int  n_scripts = 0;
	int  hook = -1;
	int  wait_secs = 0;
	const char *rom_path = NULL;
	const char *rx_path = NULL;
	bool need_bootloader = false;
	bool do_feed = false;
	int  do_reset = 0;		/* 1 = CD2_RESET, 2 = CD2_WAKEONRING */
	unsigned reset_value = 0, reset_index = 0;
	int  reset_rt = -1;		/* -1 sweeps all four framings */
	bool end_session = true;
	bool session_ended = false;
	bool stream_open = false;
	bool stream_open_alt = false;
	char dtmf = '\0';
	uint8_t patch[8];
	size_t  n_patch = 0;
	int     post_ids[16];
	int     n_post = 0;
	uint8_t post_patch[8];
	size_t  n_post_patch = 0;
	uint8_t raw[256];
	size_t  n_raw = 0;

	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i], "--load")) {
			do_load = true;
		} else if (!strcmp(argv[i], "--stream") && i + 1 < argc) {
			stream_secs = atoi(argv[++i]);
		} else if (!strcmp(argv[i], "--script") && i + 1 < argc) {
			/* Comma-separated, so a sequence runs inside one session
			 * rather than across opens. */
			for (char *p = argv[++i]; *p && n_scripts < 16; ) {
				script_ids[n_scripts++] = atoi(p);
				while (*p && *p != ',')
					p++;
				if (*p == ',')
					p++;
			}
		} else if (!strcmp(argv[i], "--start-codec")) {
			do_start = true;
		} else if (!strcmp(argv[i], "--bootloader")) {
			need_bootloader = true;
		} else if (!strcmp(argv[i], "--rom") && i + 1 < argc) {
			rom_path = argv[++i];
		} else if (!strcmp(argv[i], "--raw") && i + 1 < argc) {
			/* Hand-built wire body, for opcodes no shipped template
			 * contains.  The firmware's own dispatch table is the
			 * only guide to what these do. */
			const char *h = argv[++i];
			for (; h[0] && h[1] && n_raw < sizeof raw; h += 2) {
				unsigned v;
				if (sscanf(h, "%2x", &v) != 1)
					break;
				raw[n_raw++] = (uint8_t)v;
			}
		} else if (!strcmp(argv[i], "--post-script") && i + 1 < argc) {
			/* Scripts sent AFTER the session is up and the TX ring is
			 * primed, which is where hsfusbcd2167_ sends script 8 and
			 * then the 2261_ start pair.  Order matters here and is not
			 * known, so it is a separate flag rather than a fixed
			 * sequence. */
			for (char *p = argv[++i]; *p && n_post < 16; ) {
				post_ids[n_post++] = atoi(p);
				while (*p && *p != ',')
					p++;
				if (*p == ',')
					p++;
			}
		} else if (!strcmp(argv[i], "--post-patch") && i + 1 < argc) {
			for (char *p = argv[++i]; *p && n_post_patch < 8; ) {
				post_patch[n_post_patch++] = (uint8_t)strtoul(p, NULL, 0);
				while (*p && *p != ',')
					p++;
				if (*p == ',')
					p++;
			}
		} else if (!strcmp(argv[i], "--patch") && i + 1 < argc) {
			for (char *p = argv[++i]; *p && n_patch < 8; ) {
				patch[n_patch++] = (uint8_t)strtoul(p, NULL, 0);
				while (*p && *p != ',')
					p++;
				if (*p == ',')
					p++;
			}
		} else if (!strcmp(argv[i], "--stream-open")) {
			stream_open = true;
		} else if (!strcmp(argv[i], "--stream-open-alt")) {
			stream_open = true;
			stream_open_alt = true;
		} else if (!strcmp(argv[i], "--tx-block") && i + 1 < argc) {
			g_tx_block = (size_t)strtoul(argv[++i], NULL, 0);
			if (g_tx_block < 1 || g_tx_block > 256)
				g_tx_block = 128;
		} else if (!strcmp(argv[i], "--tx-free-run")) {
			/* The old behaviour: refill from TX completions alone. */
			g_tx_pace_rx = false;
		} else if (!strcmp(argv[i], "--no-end-session")) {
			/* Only to reproduce the degradation deliberately. */
			end_session = false;
		} else if (!strcmp(argv[i], "--reset")) {
			do_reset = 1;
		} else if (!strcmp(argv[i], "--wake-on-ring")) {
			do_reset = 2;
		} else if (!strcmp(argv[i], "--reset-value") && i + 1 < argc) {
			reset_value = (unsigned)strtoul(argv[++i], NULL, 0);
		} else if (!strcmp(argv[i], "--reset-rt") && i + 1 < argc) {
			reset_rt = atoi(argv[++i]);
		} else if (!strcmp(argv[i], "--reset-index") && i + 1 < argc) {
			reset_index = (unsigned)strtoul(argv[++i], NULL, 0);
		} else if (!strcmp(argv[i], "--feed")) {
			do_feed = true;
		} else if (!strcmp(argv[i], "--dtmf") && i + 1 < argc) {
			dtmf = argv[++i][0];
			do_feed = true;
		} else if (!strcmp(argv[i], "--tx-slot") && i + 1 < argc) {
			g_tx_slot = atoi(argv[++i]);
			if (g_tx_slot < 0 || g_tx_slot > 1) {
				fprintf(stderr, "--tx-slot must be 0 or 1\n");
				return 2;
			}
		} else if (!strcmp(argv[i], "--tx-prime-blocks") && i + 1 < argc) {
			g_tx_prime_blocks = (unsigned)strtoul(argv[++i], NULL, 0);
		} else if (!strcmp(argv[i], "--rx-out") && i + 1 < argc) {
			rx_path = argv[++i];
		} else if (!strcmp(argv[i], "--wait") && i + 1 < argc) {
			wait_secs = atoi(argv[++i]);
		} else if (!strcmp(argv[i], "--hook") && i + 1 < argc) {
			hook = !strcmp(argv[++i], "off");
		} else {
			fprintf(stderr, "usage: %s [--load] [--script ID] [--start-codec]"
				" [--script ID[,ID...]] [--patch B[,B...]]"
				" [--rom PATH] [--bootloader]"
				" [--hook on|off] [--wait SECONDS] [--feed] [--dtmf DIGIT]"
				" [--tx-slot 0|1] [--tx-prime-blocks N]"
				" [--post-script ID[,ID...]] [--post-patch B[,B...]]"
				" [--reset|--wake-on-ring] [--reset-value N] [--reset-index N]"
				" [--reset-rt 0|1|2|3] [--no-end-session] [--tx-free-run]"
				" [--stream-open]"
				" [--rx-out PATH]"
				" [--stream SECONDS]\n",
				argv[0]);
			return 2;
		}
	}

	struct hsf_dev *d = NULL;
	uint8_t info[5];
	if (dtmf) {
		static const char keys[] = "123A456B789C*0#D";
		static const double rows[] = {697, 770, 852, 941};
		static const double cols[] = {1209, 1336, 1477, 1633};
		const char *p = strchr(keys, dtmf);
		if (!p) {
			fprintf(stderr, "invalid DTMF digit: %c\n", dtmf);
			return 2;
		}
		size_t n = (size_t)(p - keys);
		g_dtmf_freq_lo = rows[n / 4];
		g_dtmf_freq_hi = cols[n % 4];
		g_dtmf_tx = true;
	}

	if (wait_secs > 0) {
		printf("waiting up to %ds for the bootloader window -- REPLUG THE DEVICE NOW\n",
		       wait_secs);
		fflush(stdout);
		/* Poll on a live EP0, not on mere presence: the device sits on the
		 * bus long after it has stopped answering, so presence is not the
		 * signal.  20 ms is comfortably inside a ~3 s window. */
		for (int t = 0; t < wait_secs * 50; t++) {
			d = hsf_fxo_open();
			if (d) {
				if (hsf_fxo_get_information(d, info) == 0) {
					/* --bootloader: keep waiting unless the
					 * device actually wants firmware.  Without
					 * this, --load silently no-ops on a device
					 * that is already running, which ran a
					 * whole patched-firmware experiment against
					 * the stock image. */
					if (!need_bootloader ||
					    info[2] == HSF_FAMILY_BOOTLOADER)
						break;
				}
				hsf_fxo_close(d);
				d = NULL;
			}
			usleep(20 * 1000);
		}
		if (!d) {
			fprintf(stderr, "the window never opened\n");
			return 1;
		}
	} else {
		d = hsf_fxo_open();
		if (!d) {
			fprintf(stderr, "no HSF modem (%04x:%04x) found, or it could not be opened\n",
				HSF_VID, HSF_PID);
			return 1;
		}
		if (hsf_fxo_get_information(d, info) < 0) {
			fprintf(stderr, "CD2_GET_INFROMATION failed.  The CD2 bootloader answers "
				"EP0 for only ~3s after it enumerates; try --wait N and replug.\n");
			hsf_fxo_close(d);
			return 1;
		}
	}
	const char *fam = info[2] == HSF_FAMILY_BOOTLOADER ? "bootloader, wants firmware"
			: info[2] == HSF_FAMILY_HCF        ? "HCF"
			: info[2] == HSF_FAMILY_HSF        ? "HSF, firmware running"
			                                   : "unknown";
	printf("info = %02x %02x %02x %02x %02x   (%s)\n",
	       info[0], info[1], info[2], info[3], info[4], fam);

	if (do_reset) {
		const char *name = do_reset == 1 ? "CD2_RESET" : "CD2_WAKEONRING";
		static const char *rtname[4] = {
			"device/OUT", "interface/OUT", "device/IN", "interface/IN"
		};
		uint8_t before = info[2];
		int changed = 0;

		/* Sweep the framing unless one was named.  A single STALL settles
		 * nothing: CD2_CONTROL_SCRIPT is interface-recipient in this part
		 * while CD2_UPLOAD_FIRMWARE is device-recipient, so the recipient
		 * cannot be assumed for an opcode nobody sends. */
		for (int rt = reset_rt >= 0 ? reset_rt : 0;
		     rt <= (reset_rt >= 0 ? reset_rt : 3); rt++) {
			int r = do_reset == 1
				? hsf_fxo_reset(d, (unsigned)rt, (uint16_t)reset_value,
						(uint16_t)reset_index)
				: hsf_fxo_wake_on_ring(d, (unsigned)rt, (uint16_t)reset_value,
						       (uint16_t)reset_index);
			/* libusb codes spelled out rather than including libusb.h:
			 * -9 LIBUSB_ERROR_PIPE, -4 LIBUSB_ERROR_NO_DEVICE.  A STALL
			 * is the device ACTIVELY REJECTING the opcode, which is a
			 * real answer and not the same as an I/O error. */
			printf("%s %-14s wValue=0x%04x wIndex=0x%04x: %s (%d)\n",
			       name, rtname[rt & 3], reset_value, reset_index,
			       r >= 0    ? "ACCEPTED"
			       : r == -9 ? "STALL (device rejected it)"
			       : r == -4 ? "device left the bus"
			                 : "error", r);
			if (r >= 0 || r == -4) {
				changed = 1;
				break;
			}
		}

		if (!changed) {
			printf("every framing tried was rejected;"
			       " %s is not implemented this way\n", name);
			hsf_fxo_close(d);
			return 1;
		}

		/* The handle may now describe a device that is gone.  Re-open and
		 * compare the family AGAINST WHAT IT WAS: reporting "the bootloader
		 * window is open" on family 01 alone claims a result for a device
		 * that was already in the bootloader and untouched, which is exactly
		 * what the first live run of this did. */
		hsf_fxo_close(d);
		d = NULL;
		for (int t = 0; t < 150; t++) {
			usleep(20 * 1000);
			d = hsf_fxo_open();
			if (!d)
				continue;
			uint8_t again[5];
			if (hsf_fxo_get_information(d, again) == 0) {
				printf("after %d ms: info = %02x %02x %02x %02x %02x   (%s)\n",
				       (t + 1) * 20, again[0], again[1], again[2],
				       again[3], again[4],
				       again[2] == before && before == HSF_FAMILY_BOOTLOADER
					       ? "STILL the bootloader it already was -- proves nothing"
				       : again[2] == before
					       ? "UNCHANGED -- the request did not reset it"
				       : again[2] == HSF_FAMILY_BOOTLOADER
					       ? "family 03 -> 01: IT RESET, and the window is open"
					              " without a replug"
					       : "family changed, but not to the bootloader");
				break;
			}
			hsf_fxo_close(d);
			d = NULL;
		}
		if (!d) {
			printf("the device did not come back within 3s"
			       " -- it needs a physical replug\n");
			return 1;
		}
	}

	if (do_load && info[2] == HSF_FAMILY_BOOTLOADER) {
		printf("loading firmware...\n");
		int r = hsf_fxo_load_firmware(d, rom_path);
		if (r < 0) {
			fprintf(stderr, "firmware load failed: %d%s\n", r,
				r == -ENOENT ? " (run tools/hsf_extract_rom.py first)" : "");
			hsf_fxo_close(d);
			return 1;
		}
		hsf_fxo_get_information(d, info);
		printf("info = %02x %02x %02x %02x %02x   (%s)\n",
		       info[0], info[1], info[2], info[3], info[4],
		       info[2] == HSF_FAMILY_HSF ? "HSF, firmware running" : "unexpected");
	}

	uint8_t eeprom[64];
	int er = hsf_fxo_read_eeprom(d, 0, eeprom, sizeof(eeprom));
	if (er > 0) {
		bool all_zero = true;
		for (int i = 0; i < er; i++)
			if (eeprom[i]) {
				all_zero = false;
				break;
			}
		printf("eeprom[0..%d]: %s\n", er - 1,
		       all_zero ? "all zero (config lives host-side, per osnvm.c)" : "non-zero");
	}

	/*
	 * Arm the pipes BEFORE sending any script.  The vendor driver keeps its
	 * RX ring posted continuously and starts the data pump from inside the
	 * notification handler (hsfusbcd2196_, the data[0]==5 case), i.e. within
	 * microseconds of the script completing.  Sending a script with no URBs
	 * posted and starting the ring afterwards is a different experiment from
	 * the one the driver runs, and it is the one this probe used to do.
	 */
	struct hsf_callbacks cb = {
		.rx_samples   = on_rx,
		.tx_done      = on_tx_done,
		.notification = on_notify,
		.user         = d,
	};
	if (rx_path) {
		g_rx_file = fopen(rx_path, "wb");
		if (!g_rx_file) {
			perror(rx_path);
			hsf_fxo_close(d);
			return 1;
		}
	}
	g_feed_tx = do_feed;
	if (stream_open)
		hsf_fxo_defer_rx(d, true);
	if (stream_secs > 0 && hsf_fxo_start(d, &cb) < 0) {
		fprintf(stderr, "could not start streaming\n");
		hsf_fxo_close(d);
		return 1;
	}

	/* Prime the controller before the session-start edge.  Although the
	 * closed driver's host-side pump calls its ring accessor from the script-5
	 * completion arm, this device only starts continuous RX when bulk OUT is
	 * already queued.  A post-completion-only prime was tested live and
	 * returned just 1598 RX bytes before the pipe stopped. */
	if (do_feed && stream_secs > 0) {
		uint8_t first[256];
		for (int i = 0; i < 4; i++) {
			fill_tx(first);
			if (hsf_fxo_tx_submit(d, first, g_tx_block) < 0)
				break;
		}
	}

	/* Bracket every script with GET_INFROMATION.  Gotcha 2 in hsf_fxo.c: a
	 * cached descriptor read proves nothing, and a wedged EP0 makes the next
	 * request lie about the one before it. */
	if (n_raw) {
		int r = hsf_fxo_script_load(d, 0xFF01, raw, n_raw);
		uint8_t after[5];
		int live = hsf_fxo_get_information(d, after);
		printf("raw script (%zu bytes): %s, device %s\n", n_raw,
		       r == 0 ? "accepted" : "REJECTED",
		       live < 0 ? "NOT RESPONDING" : "alive");
		usleep(50 * 1000);
	}

	for (int i = 0; i < n_scripts; i++) {
		int r = hsf_fxo_script_run(d, (unsigned)script_ids[i],
					   n_patch ? patch : NULL, n_patch);
		uint8_t after[5];
		int live = hsf_fxo_get_information(d, after);
		printf("script %d: load %s, device %s\n", script_ids[i],
		       r == 0 ? "accepted" : "rejected",
		       live < 0 ? "NOT RESPONDING" : "alive");
		usleep(50 * 1000);
	}

	if (do_start) {
		static const unsigned seq[] = {
			HSF_SCRIPT_SESSION_A, HSF_SCRIPT_SESSION_B
		};
		int r = 0;
		for (size_t i = 0; i < sizeof seq / sizeof seq[0]; i++) {
			unsigned before = script_count(seq[i]);
			r = hsf_fxo_script_run(d, seq[i], NULL, 0);
			if (r < 0)
				break;
			r = wait_script(seq[i], before, 2000);
			if (r < 0) {
				fprintf(stderr, "script %u completion timed out or failed\n", seq[i]);
				break;
			}
		}
		uint8_t after[5];
		int live = hsf_fxo_get_information(d, after);
		printf("start codec (script 9 completion, then script 5): %s, device %s\n",
		       r == 0 ? "accepted" : "rejected",
		       live < 0 ? "NOT RESPONDING" : "alive");
	}

	/*
	 * hsfusbcd2167_'s stream open, in its own order, which is NOT what this
	 * probe did.  It runs after the 9/5 session and before the ring is
	 * primed:
	 *
	 *   hsfusbcd2241_  script 1, wIndex 1, wait up to 1400 ms, retry once.
	 *                  This is a QUERY: the reply is four bytes rather than
	 *                  the usual two, hsfusbcd2239_(ctx, 4) reads it, and
	 *                  ctx+0x68 -- the mode that decides whether scripts 11
	 *                  and 12 do anything at all -- is set from reply byte
	 *                  0x912.  Live this part answers 01 00 00 01, so byte
	 *                  0x912 is 0 and the mode is 0.
	 *   hsfusbcd2247_  script 8, wIndex 1.
	 *   then the granularities (0x80 TX, 0x40 RX) and the 4 TX / 16 RX prime.
	 *
	 * Sending script 1 into an already-running stream instead collapses RX,
	 * so the order is the point of this flag.
	 */
	if (stream_open) {
		unsigned before1 = script_count(1);
		int r = hsf_fxo_script_run(d, 1, NULL, 0);
		if (r == 0)
			r = wait_script(1, before1, 1400);
		if (r < 0) {
			unsigned again = script_count(1);
			r = hsf_fxo_script_run(d, 1, NULL, 0);	/* the one retry */
			if (r == 0)
				r = wait_script(1, again, 1400);
		}
		printf("stream open (script 1): %s", 
		       r == 0 ? "completed" : "no completion in 2x1400ms");
		if (g_script1_seen)
			printf("  reply payload %02x %02x -> ctx+0x68 mode %d",
			       g_script1_reply[0], g_script1_reply[1], g_script1_mode);
		printf("\n");

		r = hsf_fxo_script_run(d, HSF_SCRIPT_SIGNAL, NULL, 0);
		printf("stream open (script 8): %s\n", r == 0 ? "sent" : "rejected");

		/*
		 * hsfusbcd2227_ -> hsfusbcd2252_, the codec register programming
		 * that stream open does after the ring setup and which nothing
		 * here had ever performed.  Each is hsfusbcd2200_(ctx, flags,
		 * value, addr), which sends script 2 ONLY when flags bit 0 is set
		 * -- so the three calls with flags 2 emit nothing -- and builds a
		 * 16-bit big-endian word (addr & 0x1fff) | value | 0x2000.
		 *
		 * The shadows at ctx+0x8d0..0x8dc are zero on a fresh session, so
		 * the words below are that sequence with zero shadows.  Two calls
		 * have a second form taken when ctx+0x8d4 is zero AND ctx+0x1c8's
		 * byte 0 bit 0 is clear (0xA028 for the fourth, 0x25B5 for the
		 * last); --stream-open-alt selects those.
		 */
		static const uint16_t regs[] = {
			0x2004, 0xA208, 0xF000, 0xA228, 0x6000, 0x25B4
		};
		static const uint16_t regs_alt[] = {
			0x2004, 0xA208, 0xF000, 0xA028, 0x6000, 0x25B5
		};
		const uint16_t *w = stream_open_alt ? regs_alt : regs;
		for (size_t k = 0; k < sizeof regs / sizeof regs[0]; k++) {
			uint8_t rp[8] = { (uint8_t)(w[k] >> 8), (uint8_t)(w[k] & 0xff) };
			int rr = hsf_fxo_script_run(d, 2, rp, sizeof rp);
			printf("  stream open reg write 0x%04x: %s\n", w[k],
			       rr == 0 ? "sent" : "rejected");
			usleep(5 * 1000);
		}

		/* Arm the data rings and prime AFTER the open, as the driver
		 * does from hsfusbcd2196_ rather than at attach. */
		hsf_fxo_arm_rx(d);
		if (do_feed) {
			uint8_t first[256];
			for (int i = 0; i < 4; i++) {
				fill_tx(first);
				if (hsf_fxo_tx_submit(d, first, g_tx_block) < 0)
					break;
			}
		}
	}

	if (hook >= 0) {
		int r = hsf_fxo_script_set_hook(d, hook != 0);
		uint8_t after[5];
		int live = hsf_fxo_get_information(d, after);
		printf("hook %s (script %d): %s, device %s\n",
		       hook ? "off" : "on", hook ? 3 : 4,
		       r == 0 ? "accepted" : "rejected",
		       live < 0 ? "NOT RESPONDING" : "alive");
	}

	for (int i = 0; i < n_post; i++) {
		int r = hsf_fxo_script_run(d, (unsigned)post_ids[i],
					   n_post_patch ? post_patch : NULL,
					   n_post_patch);
		uint8_t after[5];
		int live = hsf_fxo_get_information(d, after);
		printf("post script %d: load %s, device %s\n", post_ids[i],
		       r == 0 ? "accepted" : "rejected",
		       live < 0 ? "NOT RESPONDING" : "alive");
		usleep(50 * 1000);
	}

	if (stream_secs > 0) {
		printf("streaming for %ds%s...\n", stream_secs,
		       do_feed ? " (feeding signed-linear silence out)" : "");
		if (do_feed) {
			/* on_tx_done maintains the four-deep pipeline without a gap. */
			sleep((unsigned)stream_secs);
		} else {
			sleep((unsigned)stream_secs);
		}

		/*
		 * Script 6 is hsfusbcd2195_/hsfusbcd2201_'s session end, and until
		 * now nothing in this probe has ever sent it: every run opened a
		 * session with 9 and 5 and abandoned it.  That is the obvious
		 * suspect for a part that serves exactly one streaming run per
		 * firmware load and then produces no audio at all.
		 *
		 * Sent BEFORE hsf_fxo_stop() so the notification ring is still
		 * posted and the script's own completion code can be seen; the
		 * host-side teardown is ours and the device knows nothing about it.
		 */
		/* BEFORE the teardown, with the notification ring still posted.
		 * Sending it after hsf_fxo_stop() was tried and the part degraded
		 * on the very next run exactly as if it had never been sent, so
		 * the ordering is load-bearing and not merely convenient for
		 * observing the completion. */
		if (end_session) {
			/* hsfusbcd2265_ is the stream stop and runs BEFORE the
			 * session end: 2261_(3,0) then 2261_(1,0), which in the
			 * default mode 0 is one script 11 carrying 0x01 in both
			 * patch bytes. */
			uint8_t stop[8] = { HSF_SCRIPT_STOP_CODE, HSF_SCRIPT_STOP_CODE };
			unsigned before11 = script_count(HSF_SCRIPT_PATH_STOP);
			int rs = hsf_fxo_script_run(d, HSF_SCRIPT_PATH_STOP,
						    stop, sizeof stop);
			if (rs == 0)
				rs = wait_script(HSF_SCRIPT_PATH_STOP, before11, 2000);
			printf("stream stop (script 11): %s\n",
			       rs == 0 ? "completed" : "no completion within 2s");

			unsigned before6 = script_count(HSF_SCRIPT_SESSION_END);
			int r = hsf_fxo_script_run(d, HSF_SCRIPT_SESSION_END, NULL, 0);
			if (r == 0)
				r = wait_script(HSF_SCRIPT_SESSION_END, before6, 2000);
			printf("session end (script 6): %s\n",
			       r == 0 ? "completed" : "no completion within 2s");
			session_ended = true;
		}

		hsf_fxo_stop(d);

		struct hsf_stats s;
		hsf_fxo_stats(d, &s);
		static const char *st[] = { "COMPLETED", "ERROR", "TIMED_OUT",
					    "CANCELLED", "STALL", "NO_DEVICE", "OVERFLOW" };
		printf("rx %llu bytes in %lu packets, tx %llu, rx_err %llu, notifications %llu\n",
		       (unsigned long long)s.rx_bytes, g_rx_packets,
		       (unsigned long long)s.tx_bytes, (unsigned long long)s.rx_errors,
		       (unsigned long long)s.notifications);
		printf("rx packet lengths:");
		for (int i = 0; i <= 256; i++)
			if (g_rx_len_hist[i])
				printf(" %d:%lu", i, g_rx_len_hist[i]);
		printf("\n");
		if (g_ring_events)
			printf("ring edges: %lu\n", g_ring_events);
		printf("tx_err %llu, tx submits refused (ring full) %llu\n",
		       (unsigned long long)s.tx_errors,
		       (unsigned long long)s.underruns);
		if (s.tx_first_error) {
			int v = s.tx_first_error - 1;
			printf("first tx status: %s (%d)\n",
			       (v >= 0 && v < 7) ? st[v] : "?", v);
		}
		if (s.rx_first_error) {
			int v = s.rx_first_error - 1;
			printf("first rx status: %s (%d)\n",
			       (v >= 0 && v < 7) ? st[v] : "?", v);
		}

		if (s.rx_bytes) {
			double rate = (double)s.rx_bytes / stream_secs;
			printf("~%.0f bytes/s -> %.0f frames/s if two 16-bit slots\n",
			       rate, rate / 4.0);
			printf("first bytes:");
			for (size_t i = 0; i < g_first_len; i++)
				printf(" %02x", g_first[i]);
			printf("\n");
			/* This is only a byte-frequency diagnostic.  The ring is
			 * signed-linear samples, not G.711 codewords. */
			unsigned long best = 0;
			int mode = 0;
			for (int i = 0; i < 256; i++)
				if (g_hist[i] > best) {
					best = g_hist[i];
					mode = i;
				}
			printf("modal raw byte 0x%02x%s\n", mode,
			       mode == 0x00 ? " (linear zero/padding)" : "");
		} else {
			printf("no samples received\n");
		}
	}

	/*
	 * Script 6 is hsfusbcd2195_/hsfusbcd2201_'s session end, and a session
	 * that is opened and abandoned leaves the part producing no audio at all
	 * on every later run until it is replugged and reloaded -- while EP0,
	 * the scripts and their notifications all still look healthy.
	 *
	 * It therefore runs for ANY run that started a session, not just a
	 * streaming one.  It used to sit inside the streaming block, so
	 * `--start-codec` without `--stream` -- which is how you take a hook off
	 * or put it back -- silently killed the device it was tidying up.
	 */
	if (do_start && end_session && !session_ended) {
		/* No completion wait: by here the notification ring is down (torn
		 * down with the stream, or never started on a non-streaming run),
		 * so there is nothing to observe it on.  That it completes was
		 * confirmed once with the ring still posted; what matters per run
		 * is that it is SENT. */
		int r = hsf_fxo_script_run(d, HSF_SCRIPT_SESSION_END, NULL, 0);
		printf("session end (script 6): %s\n",
		       r == 0 ? "sent" : "rejected");
	}

	if (g_rx_file) {
		fclose(g_rx_file);
		g_rx_file = NULL;
	}
	hsf_fxo_close(d);
	return 0;
}
