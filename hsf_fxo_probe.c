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
 *   ./hsf_fxo_probe --load --start-codec --stream 5    # scripts 9,5,9 then stream
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
 * --script is the instrument for the open question, which is WHICH script
 * ungates the bulk pipes.  Send them one at a time: every attempt is bracketed
 * with GET_INFROMATION, so a script that wedges the device is reported as such
 * rather than being blamed on the next one.
 */

#include "hsf_fxo.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static unsigned long g_rx_packets;
static unsigned char g_first[64];
static size_t        g_first_len;
static unsigned long g_hist[256];

static void on_rx(const uint8_t *data, size_t len, void *user)
{
	(void)user;
	g_rx_packets++;
	for (size_t i = 0; i < len; i++)
		g_hist[data[i]]++;
	if (g_first_len < sizeof(g_first)) {
		size_t n = sizeof(g_first) - g_first_len;
		if (n > len)
			n = len;
		memcpy(g_first + g_first_len, data, n);
		g_first_len += n;
	}
}

static void on_notify(const struct hsf_notification *n, void *user)
{
	(void)user;
	printf("  NOTIFICATION bmRequestType=0x%02x code=0x%02x "
	       "wValue=0x%04x wIndex=%u wLength=%u",
	       n->bmRequestType, n->bNotification, n->wValue, n->wIndex, n->wLength);
	for (size_t i = 0; i < n->data_len; i++)
		printf("%s%02x", i ? "" : "  data=", n->data[i]);
	printf("\n");
	fflush(stdout);
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
	bool do_feed = false;

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
		} else if (!strcmp(argv[i], "--feed")) {
			do_feed = true;
		} else if (!strcmp(argv[i], "--wait") && i + 1 < argc) {
			wait_secs = atoi(argv[++i]);
		} else if (!strcmp(argv[i], "--hook") && i + 1 < argc) {
			hook = !strcmp(argv[++i], "off");
		} else {
			fprintf(stderr, "usage: %s [--load] [--script ID] [--start-codec]"
				" [--script ID[,ID...]]"
				" [--hook on|off] [--wait SECONDS] [--feed]"
				" [--stream SECONDS]\n",
				argv[0]);
			return 2;
		}
	}

	struct hsf_dev *d = NULL;
	uint8_t info[5];

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
				if (hsf_fxo_get_information(d, info) == 0)
					break;
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

	if (do_load && info[2] == HSF_FAMILY_BOOTLOADER) {
		printf("loading firmware...\n");
		int r = hsf_fxo_load_firmware(d, NULL);
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

	/* Bracket every script with GET_INFROMATION.  Gotcha 2 in hsf_fxo.c: a
	 * cached descriptor read proves nothing, and a wedged EP0 makes the next
	 * request lie about the one before it. */
	for (int i = 0; i < n_scripts; i++) {
		int r = hsf_fxo_script_run(d, (unsigned)script_ids[i], NULL, 0);
		uint8_t after[5];
		int live = hsf_fxo_get_information(d, after);
		printf("script %d: load %s, device %s\n", script_ids[i],
		       r == 0 ? "accepted" : "rejected",
		       live < 0 ? "NOT RESPONDING" : "alive");
		usleep(50 * 1000);
	}

	if (do_start) {
		int r = hsf_fxo_script_start_codec(d);
		uint8_t after[5];
		int live = hsf_fxo_get_information(d, after);
		printf("start codec (scripts 9,5,9): %s, device %s\n",
		       r == 0 ? "accepted" : "rejected",
		       live < 0 ? "NOT RESPONDING" : "alive");
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

	if (stream_secs > 0) {
		struct hsf_callbacks cb = {
			.rx_samples   = on_rx,
			.notification = on_notify,
		};
		if (hsf_fxo_start(d, &cb) < 0) {
			fprintf(stderr, "could not start streaming\n");
			hsf_fxo_close(d);
			return 1;
		}
		printf("streaming for %ds%s...\n", stream_secs,
		       do_feed ? " (feeding u-law silence out)" : "");
		if (do_feed) {
			/* A bulk codec has no isochronous clock, so the device may
			 * only produce RX while the host is giving it OUT data to
			 * play.  160 bytes per 20 ms is 8 kHz. */
			/* HSF_FEED_CHUNK lets the packet size be varied: the bulk
			 * endpoints are 64 B, so a 160 B buffer ends in a short
			 * packet, which some devices treat as end-of-stream. */
			const char *cs = getenv("HSF_FEED_CHUNK");
			size_t chunk = cs ? (size_t)atoi(cs) : 160;
			if (chunk == 0 || chunk > 256)
				chunk = 160;
			uint8_t silence[256];
			memset(silence, 0xff, sizeof(silence));
			for (int ms = 0; ms < stream_secs * 1000; ms += 20) {
				size_t want = 160;
				while (want) {
					size_t n = want < chunk ? want : chunk;
					if (hsf_fxo_tx_submit(d, silence, n) < 0)
						break;
					want -= n;
				}
				usleep(20 * 1000);
			}
		} else {
			sleep((unsigned)stream_secs);
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
		if (s.rx_first_error) {
			int v = s.rx_first_error - 1;
			printf("first rx status: %s (%d)\n",
			       (v >= 0 && v < 7) ? st[v] : "?", v);
		}

		if (s.rx_bytes) {
			double rate = (double)s.rx_bytes / stream_secs;
			printf("~%.0f bytes/s -> %.0f Hz if 8-bit, %.0f Hz if 16-bit\n",
			       rate, rate, rate / 2.0);
			printf("first bytes:");
			for (size_t i = 0; i < g_first_len; i++)
				printf(" %02x", g_first[i]);
			printf("\n");
			/* An on-hook line is silence, so the modal byte names the
			 * encoding: 0xff u-law, 0xd5 A-law, 0x00 linear. */
			unsigned long best = 0;
			int mode = 0;
			for (int i = 0; i < 256; i++)
				if (g_hist[i] > best) {
					best = g_hist[i];
					mode = i;
				}
			printf("modal byte 0x%02x (%s)\n", mode,
			       mode == 0xff ? "u-law silence" :
			       mode == 0xd5 ? "A-law silence" :
			       mode == 0x00 ? "linear zero"   : "?");
		} else {
			printf("no samples -- the codec is gated behind CD2_CONTROL_SCRIPT\n");
		}
	}

	hsf_fxo_close(d);
	return 0;
}
