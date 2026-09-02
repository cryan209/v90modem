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
/*
 * The device clocks the bulk stream at 16000 Hz -- measured on the vendor
 * driver's own traffic, from DTMF the driver generated for a dialled digit, so
 * the figure comes from spec-fixed tone frequencies rather than from a byte
 * rate (docs/hsf_usb_daa.md, "The vendor driver, captured on real hardware").
 * It was 21240.0 here, taken from a byte rate measured against a stream
 * nothing was pacing.  Synthesizing at 21240 and clocking out at 16000 lands a
 * 697 Hz row tone on 525 Hz, which no DTMF detector is listening for.
 * --tx-rate overrides it, so the two can be A/B'd on one binary.
 */
static double        g_tx_rate = 16000.0;
static double        g_dtmf_phase_lo;
static double        g_dtmf_phase_hi;
static double        g_dtmf_freq_lo;
static double        g_dtmf_freq_hi;
static int           g_tx_slot = -1;
static unsigned      g_tx_prime_blocks;
static unsigned      g_tx_blocks;
static bool          g_tx_pace_rx = true;	/* credit TX from RX, as the driver does */
static size_t        g_tx_block = 128;	/* hsfusbcd2167_ sets ctx+0x876 = 0x80 */
/*
 * The driver's own accounting says TX is HALF the received byte rate, and that
 * TX frames are 2 bytes rather than 4.  hsfusbcd2184_ divides accumulated
 * RECEIVED bytes by ctx+0x876 = 128 -- the TX granularity, not the RX one --
 * into ctx+0x8ac, and hsfusbcd2212_ then hands the engine
 * ctx+0x874 * units = 64 * (rx_bytes / 128) = rx_bytes / 2, as BOTH the
 * receive length and the transmit length.
 *
 * That matches the frame layout on the wire: a received frame is four bytes
 * carrying one 16-bit sample and one zero slot (5b 01 00 00), so the audio is
 * half the byte rate, and the transmit side is a plain 16-bit mono stream.
 * This probe had been sending 4-byte two-slot frames at 1:1 with RX bytes,
 * i.e. twice the rate in the wrong format.
 */
static bool          g_tx_mono = true;
static size_t        g_rx_credit;
static uint8_t       g_script1_reply[2];
static int           g_script1_mode;
static bool          g_script1_seen;

/*
 * DTMF dialling: a digit string sent as proper bursts rather than one tone held
 * forever.  A held tone is not a digit -- exchanges collect on the tone's
 * leading edge and need silence between digits -- which is why holding 697+1209
 * never made the exchange cut dial tone.  ITU-T Q.23: 40 ms minimum tone and
 * 40 ms minimum gap; 120/80 here is comfortably inside every collector.
 */
static const struct { char d; double lo, hi; } g_dtmf_tab[] = {
	{'1',697,1209},{'2',697,1336},{'3',697,1477},{'A',697,1633},
	{'4',770,1209},{'5',770,1336},{'6',770,1477},{'B',770,1633},
	{'7',852,1209},{'8',852,1336},{'9',852,1477},{'C',852,1633},
	{'*',941,1209},{'0',941,1336},{'#',941,1477},{'D',941,1633},
};
static const char  *g_dial;
static size_t       g_dial_pos;
static unsigned     g_dial_samples;      /* samples emitted in this phase */
static bool         g_dial_gap;
static unsigned     g_dial_tone_len  = 120 * 16;   /* 120 ms at 16 kHz */
static unsigned     g_dial_gap_len   =  80 * 16;
static unsigned     g_dial_delay     = 1000 * 16;  /* wait before dialling */
static unsigned     g_dial_waited;
static double       g_dial_amp = 8000.0;

/*
 * After the digits, emit a distinctive tone in bursts so an echo test has
 * something to correlate: 250 ms on, 750 ms off.  Dialling an echo service and
 * looking for this coming back is the one test that exercises transmit and
 * receive together, end to end, over a real call.
 */
static double   g_echo_tone;          /* Hz, 0 = off */
static double   g_echo_amp = 8000.0;
static double   g_echo_on_ms = 50.0;
static bool     g_echo_sweep;
static bool     g_pcm_code_test;
static uint16_t g_pcm_lfsr = 0xace1;
static int16_t  g_ulaw_sample;
static unsigned g_ulaw_repeat;

static int16_t ulaw_decode(uint8_t u)
{
	u = (uint8_t)~u;
	int t = ((u & 0x0f) << 3) + 0x84;
	t <<= (u & 0x70) >> 4;
	return (int16_t)((u & 0x80) ? (0x84 - t) : (t - 0x84));
}
/* --tx-out writes exactly what we hand the device, so the received stream can
 * be CROSS-CORRELATED against it.  A single-bin tone detector cannot separate
 * our own hybrid sidetone from a network echo, and is contaminated by whatever
 * cadenced tone the line happens to be playing; a correlation peak at a
 * non-zero lag is unambiguous. */
static FILE    *g_tx_file;
static unsigned g_echo_n;

/* Returns false once the string is finished. */
static bool dial_sample(int16_t *out)
{
	if (!g_dial || !*g_dial)
		return false;
	if (g_dial_waited < g_dial_delay) {
		g_dial_waited++;
		*out = 0;
		return true;
	}
	if (g_dial[g_dial_pos] == '\0') {
		if (g_pcm_code_test) {
			/* Let the echo service answer before replaying a real 8 kHz
			 * G.711/V.90 stream.  The HSF bulk pipe is 16 ksample/s, so
			 * each decoded codeword occupies two output samples. */
			if (g_echo_n++ < (unsigned)(5 * g_tx_rate)) {
				*out = 0;
				return true;
			}
			if (g_ulaw_repeat == 0) {
				/* Deterministic maximal-length PRBS selecting the
				 * negotiated V.90 Ucode 0..78 and sign. */
				unsigned lsb = g_pcm_lfsr & 1u;
				g_pcm_lfsr >>= 1;
				if (lsb)
					g_pcm_lfsr ^= 0xb400u;
				unsigned ucode = g_pcm_lfsr % 79u; /* local U_INFO=78 */
				unsigned sign = (g_pcm_lfsr >> 8) & 1u;
				int c = ((0xffu - ucode) & 0x7fu) | (sign ? 0x80u : 0u);
				g_ulaw_sample = c == EOF ? 0 : ulaw_decode((uint8_t)c);
				g_ulaw_repeat = 2;
			}
			*out = g_ulaw_sample;
			g_ulaw_repeat--;
			return true;
		}
		if (g_echo_tone <= 0 && !g_echo_sweep)
			return false;
		if (g_echo_sweep) {
			/* Repeating 100--4000 Hz stepped sweep: 250 ms tone and
			 * 250 ms silence per bin.  Repetition ensures the low bins
			 * are measured again after a slow echo service answers. */
			unsigned dwell = (unsigned)(g_tx_rate / 2.0);
			unsigned on = dwell / 2;
			unsigned step = (g_echo_n / dwell) % 40;
			unsigned ph = g_echo_n % dwell;
			double f = 100.0 * (step + 1);
			double t = (double)g_echo_n++ / g_tx_rate;
			*out = (ph < on) ? (int16_t)(g_echo_amp * sin(2*M_PI*f*t)) : 0;
			return true;
		}
		/* A burst SHORT relative to the network round trip, so an echo
		 * returns as a separate blip instead of overlapping the
		 * hybrid's local sidetone.  250 ms cannot distinguish them. */
		unsigned period = (unsigned)g_tx_rate;                    /* 1 s */
		unsigned on     = (unsigned)(g_tx_rate * g_echo_on_ms / 1000.0);
		unsigned ph = g_echo_n++ % period;
		double t = (double)g_echo_n / g_tx_rate;
		*out = (ph < on) ? (int16_t)(g_echo_amp * sin(2*M_PI*g_echo_tone*t)) : 0;
		return true;
	}
	if (g_dial_gap) {
		*out = 0;
		if (++g_dial_samples >= g_dial_gap_len) {
			g_dial_samples = 0;
			g_dial_gap = false;
			g_dial_pos++;
		}
		return true;
	}
	char c = g_dial[g_dial_pos];
	double lo = 0, hi = 0;
	for (size_t i = 0; i < sizeof g_dtmf_tab / sizeof g_dtmf_tab[0]; i++)
		if (g_dtmf_tab[i].d == c) { lo = g_dtmf_tab[i].lo; hi = g_dtmf_tab[i].hi; }
	if (lo == 0) { g_dial_pos++; return true; }
	double t = (double)g_dial_samples / g_tx_rate;
	*out = (int16_t)(g_dial_amp * (sin(2*M_PI*lo*t) + sin(2*M_PI*hi*t)) / 2.0);
	if (++g_dial_samples >= g_dial_tone_len) {
		g_dial_samples = 0;
		g_dial_gap = true;
	}
	return true;
}

static void fill_tx_inner(uint8_t *buf);

static void fill_tx(uint8_t *buf)
{
	fill_tx_inner(buf);
	if (g_tx_file)
		fwrite(buf, 1, g_tx_block, g_tx_file);
}

static void fill_tx_inner(uint8_t *buf)
{
	memset(buf, 0, g_tx_block);
	g_tx_blocks++;
	if (g_tx_blocks <= g_tx_prime_blocks)
		return;
	if (g_dial) {
		int16_t *ds = (int16_t *)buf;
		size_t nf = g_tx_block / 2;
		for (size_t i = 0; i < nf; i++) {
			int16_t sm = 0;
			dial_sample(&sm);
			ds[i] = sm;
		}
		return;
	}
	if (!g_dtmf_tx)
		return;
	/* RX strongly suggests four-byte frames.  Put the same signed sample in
	 * both candidate 16-bit slots so either possible audio lane carries the
	 * digit without putting two different time instants into one frame. */
	int16_t *s = (int16_t *)buf;
	size_t frames = g_tx_mono ? g_tx_block / 2 : g_tx_block / 4;
	for (size_t i = 0; i < frames; i++) {
		int16_t v = (int16_t)(3500.0 * (sin(g_dtmf_phase_lo) +
						 sin(g_dtmf_phase_hi)));
		if (g_tx_mono) {
			s[i] = v;
		} else {
			if (g_tx_slot < 0 || g_tx_slot == 0)
				s[2*i] = v;
			if (g_tx_slot < 0 || g_tx_slot == 1)
				s[2*i + 1] = v;
		}
		g_dtmf_phase_lo += 2.0 * M_PI * g_dtmf_freq_lo / g_tx_rate;
		g_dtmf_phase_hi += 2.0 * M_PI * g_dtmf_freq_hi / g_tx_rate;
		if (g_dtmf_phase_lo >= 2.0 * M_PI) g_dtmf_phase_lo -= 2.0 * M_PI;
		if (g_dtmf_phase_hi >= 2.0 * M_PI) g_dtmf_phase_hi -= 2.0 * M_PI;
	}
}

/*
 * Arming the data rings FROM SCRIPT 5'S COMPLETION, which is what the vendor
 * driver does and what this probe could not previously express.
 *
 * Read off usbmon (docs/hsf_usb_daa.md): the vendor's first bulk transfer is
 * submitted at the instant script 5's completion notification arrives, not
 * before the script sequence and not from a later poll in the main loop.  This
 * probe called hsf_fxo_start() -- which submits the whole RX ring -- before any
 * script had been sent, so the device saw bulk transfers against a session it
 * had not yet opened.
 *
 * The submit therefore has to happen inside the notification callback, on the
 * libusb event thread, which is exactly where the driver does it (its RX and TX
 * completion handlers tail-jump into the same pump).
 */
static bool           g_arm_on_session_b;
static volatile bool  g_armed;

static void arm_bulk_now(struct hsf_dev *d)
{
	if (g_armed)
		return;
	g_armed = true;
	hsf_fxo_arm_rx(d);
	if (!g_feed_tx)
		return;
	uint8_t first[256];
	for (int i = 0; i < 4; i++) {
		fill_tx(first);
		if (hsf_fxo_tx_submit(d, first, g_tx_block) < 0)
			break;
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
		/* hsfusbcd2212_ hands the engine 64 * (rx_bytes / 128).  A 128-byte
		 * unit is 64 samples, so that quantity is a SAMPLE count, not a
		 * byte count -- and the transmit side owes the same number of
		 * samples, i.e. the SAME number of bytes.  Reading it as bytes (an
		 * earlier change here) fed transmit at half rate. */
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
	struct hsf_dev *nd = user;
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

		/* The vendor's arming point.  Do it here, in the callback, not
		 * back in main: the driver submits its first bulk pair from
		 * this completion and the device evidently cares. */
		if (g_arm_on_session_b && nd &&
		    n->data[0] == HSF_SCRIPT_SESSION_B)
			arm_bulk_now(nd);
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

/*
 * The codec open: script 1 (the query), script 8, and the six script-2 register
 * writes.  Factored out because the ORDER of this relative to the 9/5 session
 * start is the whole question -- the vendor runs it BEFORE, this probe ran it
 * after.  See codec_open_prelude()'s two callers.
 */
/*
 * The vendor driver's CALL sequence, read off the wire in exact order
 * (docs/hsf_usb_daa.md).  This is NOT the init sequence, and the difference is
 * the point: at a call the driver sends NO script 1, writes SEVEN codec
 * registers beginning with 0x35b7, starts the session, goes off-hook, and then
 * writes an EIGHTH register 0xaac8 -- which is 0xaae8 with bit 0x20 cleared,
 * the same register with one bit toggled immediately after the DAA takes the
 * line.  Nothing this probe did had that trailing write.
 *
 *   2 x 0x35b7, 0x2004, 0xa208, 0xf200, 0xaae8, 0x6040, 0x35b4
 *   8 wIndex 1 / 9 / 5          -> bulk
 *   8 wIndex 3 / 3 (off-hook)
 *   2 x 0xaac8
 */
static int script2_reg(struct hsf_dev *d, uint16_t w)
{
	int r = hsf_fxo_script2_reg(d, w);
	printf("  call-seq reg 0x%04x: %s\n", w, r == 0 ? "sent" : "REJECTED");
	usleep(5 * 1000);
	return r;
}

static int codec_open_prelude(struct hsf_dev *d, unsigned reg_d6, unsigned reg_da,
			      unsigned reg_dc, unsigned reg_e4)
{
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
	/*
	 * hsfusbcd2169_ -- which stream open calls right after script 8 --
	 * initialises the shadows this sequence reads, and three of them
	 * are UNCONDITIONAL: ctx+0x8d0 = 0x40, 0x8d2 = 0x800,
	 * 0x8d4 = 0x200.  Taking them as zero (as this probe first did)
	 * gets three of the six words wrong.
	 *
	 * The rest come from the config struct at ctx+0x1c8 and from
	 * ctx+0x58, none of which is visible from here:
	 *   0x8d6 = 0/0x200/0x400/0x600 from (cfg[0] >> 1) & 7
	 *   0x8da = 0x1000 if (cfg[0] & 0x10) or ctx+0x58, else 0
	 *   0x8dc = 0/0x800/0x1000/0x1800 from (cfg[0] >> 5) & 7
	 *   0x8e4 = 0 if cfg[0] & 1, else 0x80 if cfg[1] & 1, else 0xC0
	 * (0x8d8 is set too but only feeds a flags-2 call, which sends
	 * nothing, so it does not matter here.)
	 *
	 * --regs d6,da,dc,e4 sweeps them; the default is the all-zero
	 * config, which is 0xC0 for 0x8e4 rather than 0.
	 */
	const uint16_t d0 = 0x40, d2 = 0x800, d4 = 0x200;
	uint16_t w4 = (uint16_t)((reg_e4 | 0x20 | d2 | reg_da) | 0x208);
	uint16_t regs[6] = {
		(uint16_t)((0x004 & 0x1fff) | 0x2000),
		(uint16_t)((0x208 & 0x1fff) | 0x8000 | 0x2000),
		(uint16_t)(((d4 | 0x1000) & 0x1fff) | 0xC000 | 0x2000),
		(uint16_t)((w4 & 0x1fff) | 0x8000 | 0x2000),
		(uint16_t)(((reg_d6 | d0) & 0x1fff) | 0x4000 | 0x2000),
		(uint16_t)(((reg_dc | 0x5b4) & 0x1fff) | 0x2000),
	};
	const uint16_t *w = regs;
	for (size_t k = 0; k < sizeof regs / sizeof regs[0]; k++) {
		uint8_t rp[8] = { (uint8_t)(w[k] >> 8), (uint8_t)(w[k] & 0xff) };
		int rr = hsf_fxo_script_run(d, 2, rp, sizeof rp);
		printf("  stream open reg write 0x%04x: %s\n", w[k],
		       rr == 0 ? "sent" : "rejected");
		usleep(5 * 1000);
	}

	return 0;
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
	bool vendor_seq = false;
	bool arm_on_5 = false;
	bool call_seq = false;
	bool need_bootloader = false;
	bool do_feed = false;
	int  do_reset = 0;		/* 1 = CD2_RESET, 2 = CD2_WAKEONRING */
	unsigned reset_value = 0, reset_index = 0;
	int  reset_rt = -1;		/* -1 sweeps all four framings */
	int  trickle_ms = 0;
	bool end_session = true;
	bool session_ended = false;
	bool stream_open = false;
	/*
	 * reg_dc = 0x1000 is not a guess: with these four values the six
	 * script-2 codec register words this probe emits are BYTE-IDENTICAL to
	 * the vendor driver's, captured on the wire --
	 * 2004 a208 f200 aae8 6040 35b4.  It was 0 here, which made the last
	 * word 0x25b4 and was the only one of the six that differed.
	 * (Correcting it does not by itself change what the device does.)
	 */
	unsigned reg_d6 = 0, reg_da = 0, reg_dc = 0x1000, reg_e4 = 0xC0;
	char dtmf = '\0';
	uint8_t patch[8];
	size_t  n_patch = 0;
	int     post_ids[16];
	int     n_post = 0;
	uint8_t post_patch[8];
	size_t  n_post_patch = 0;
	uint8_t raw[256];
	size_t  n_raw = 0;
	bool    raw_post = false;

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
		} else if (!strcmp(argv[i], "--arm-on-5")) {
			arm_on_5 = true;
		} else if (!strcmp(argv[i], "--dial") && i + 1 < argc) {
			g_dial = argv[++i];
			do_feed = true;
		} else if (!strcmp(argv[i], "--tx-out") && i + 1 < argc) {
			g_tx_file = fopen(argv[++i], "wb");
		} else if (!strcmp(argv[i], "--echo-on-ms") && i + 1 < argc) {
			g_echo_on_ms = atof(argv[++i]);
		} else if (!strcmp(argv[i], "--echo-tone") && i + 1 < argc) {
			g_echo_tone = atof(argv[++i]);
		} else if (!strcmp(argv[i], "--echo-sweep")) {
			g_echo_sweep = true;
		} else if (!strcmp(argv[i], "--pcm-code-test")) {
			g_pcm_code_test = true;
			do_feed = true;
		} else if (!strcmp(argv[i], "--dial-amp") && i + 1 < argc) {
			g_dial_amp = atof(argv[++i]);
		} else if (!strcmp(argv[i], "--call-seq")) {
			call_seq = true;
			do_start = true;
		} else if (!strcmp(argv[i], "--vendor-seq")) {
			vendor_seq = true;
			do_start = true;
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
		} else if (!strcmp(argv[i], "--raw-post")) {
			/* Send --raw AFTER the session and stream open, so a register
			 * read reflects a device that is actually streaming rather
			 * than an idle one. */
			raw_post = true;
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
		} else if (!strcmp(argv[i], "--regs") && i + 1 < argc) {
			char *q = argv[++i];
			unsigned v[4] = { 0, 0, 0, 0xC0 };
			for (int k = 0; k < 4 && *q; k++) {
				v[k] = (unsigned)strtoul(q, &q, 0);
				if (*q == ',')
					q++;
			}
			reg_d6 = v[0]; reg_da = v[1]; reg_dc = v[2]; reg_e4 = v[3];
		} else if (!strcmp(argv[i], "--stream-open-alt")) {
			stream_open = true;
		} else if (!strcmp(argv[i], "--tx-trickle") && i + 1 < argc) {
			/* Feed one block every N ms from the main loop instead of
			 * from RX credit.  Far below the codec rate, so the FIFO
			 * cannot be overrun: if the device consumes at all, the
			 * accepted total must exceed its ~2.5 kB capacity. */
			trickle_ms = atoi(argv[++i]);
		} else if (!strcmp(argv[i], "--tx-block") && i + 1 < argc) {
			g_tx_block = (size_t)strtoul(argv[++i], NULL, 0);
			if (g_tx_block < 1 || g_tx_block > 256)
				g_tx_block = 128;
		} else if (!strcmp(argv[i], "--tx-quad")) {
			/* The old 4-byte two-slot frames at 1:1 with RX bytes. */
			g_tx_mono = false;
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
		} else if (!strcmp(argv[i], "--tx-rate") && i + 1 < argc) {
			g_tx_rate = atof(argv[++i]);
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
	const char *dial_delay_ms = getenv("HSF_DIAL_DELAY_MS");
	if (dial_delay_ms)
		g_dial_delay = (unsigned)strtoul(dial_delay_ms, NULL, 0) * 16;
	const char *dial_tone_ms = getenv("HSF_DIAL_TONE_MS");
	if (dial_tone_ms)
		g_dial_tone_len = (unsigned)strtoul(dial_tone_ms, NULL, 0) * 16;
	const char *dial_gap_ms = getenv("HSF_DIAL_GAP_MS");
	if (dial_gap_ms)
		g_dial_gap_len = (unsigned)strtoul(dial_gap_ms, NULL, 0) * 16;

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
	/* The vendor driver never issues CD2_READ_EEPROM; --vendor-seq is a
	 * replay, so it does not either. */
	int er = (vendor_seq || call_seq) ? -1 :
		 hsf_fxo_read_eeprom(d, 0, eeprom, sizeof(eeprom));
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
	if (stream_open || vendor_seq || call_seq)
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
	 * returned just 1598 RX bytes before the pipe stopped.
	 *
	 * --vendor-seq skips it: that experiment predates both the correct
	 * script order and the correct codec register value (0x35b4, not
	 * 0x25b4), so it was measuring a session the device had not opened.
	 * Under --vendor-seq the prime happens in arm_bulk_now(), from script
	 * 5's completion, which is where the driver does it. */
	if (do_feed && stream_secs > 0 && !vendor_seq && !call_seq) {
		uint8_t first[256];
		/* The driver's own runtime log (osusb.c dbg, _DEBUG in osusb.c)
		 * shows it keeps exactly ONE transmit URB outstanding -- the same
		 * ule address every pass -- submitting one RX then one TX and
		 * re-submitting each from its completion.  "Primes four" came
		 * from the disassembly and is not what it does.  HSF_TX_PRIME
		 * makes the depth an experiment. */
		const char *tp = getenv("HSF_TX_PRIME");
		int nprime = tp ? atoi(tp) : 4;
		for (int i = 0; i < nprime; i++) {
			fill_tx(first);
			if (hsf_fxo_tx_submit(d, first, g_tx_block) < 0)
				break;
		}
	}

	/* Bracket every script with GET_INFROMATION.  Gotcha 2 in hsf_fxo.c: a
	 * cached descriptor read proves nothing, and a wedged EP0 makes the next
	 * request lie about the one before it. */
	if (n_raw && !raw_post) {
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
		/*
		 * The vendor driver, captured on real hardware (2026-09-02),
		 * issues HSF_SCRIPT_SIGNAL (8, wValue=0xFF02) immediately
		 * BEFORE the 9/5 pair -- completion 08 80 -- and only then does
		 * the first bulk transfer go out.  This probe never sent it.
		 * HSF_SIGNAL_FIRST makes that an experiment rather than an
		 * assumption; see docs/hsf_usb_daa.md.
		 */
		static const unsigned seq_plain[] = {
			HSF_SCRIPT_SESSION_A, HSF_SCRIPT_SESSION_B
		};
		static const unsigned seq_signal[] = {
			HSF_SCRIPT_SIGNAL, HSF_SCRIPT_SESSION_A, HSF_SCRIPT_SESSION_B
		};
		const bool sig_first = getenv("HSF_SIGNAL_FIRST") != NULL;
		const unsigned *seq = sig_first ? seq_signal : seq_plain;
		const size_t seq_n = sig_first ? 3 : 2;

		/*
		 * --vendor-seq replays the session bring-up EXACTLY as the
		 * vendor driver performs it, read off usbmon rather than out of
		 * the blob (docs/hsf_usb_daa.md).  The driver runs this whole
		 * unit, and the probe's own order was inverted: it sent 9 and 5
		 * FIRST and only then the script 1 / 8 / 2 prelude.
		 *
		 *   CD2_GET_INFROMATION
		 *   CLEAR_FEATURE(HALT) ep 0x82
		 *   script 1   wIndex 1      (the query; 4-byte reply)
		 *   script 8   wIndex 1
		 *   script 2   wIndex 1
		 *   script 8   wIndex 3      (delete)
		 *   script 8   wIndex 1
		 *   script 9   wIndex 1
		 *   script 5   wIndex 1      -> bulk begins on its completion
		 */
		if (call_seq) {
			/* The driver runs a full init pass (script 1 + the six codec
			 * registers) before any call.  HSF_CALL_INIT=1 reproduces
			 * that ahead of the call sequence. */
			/*
			 * The vendor's INIT, which --call-seq needs to have run
			 * once and which this probe could not previously do.  The
			 * capture shows it is not just scripts: the driver resets
			 * the USB port, re-reads the descriptors and issues
			 * SET_CONFIGURATION, and runs the whole unit TWICE, each
			 * cycle ending by closing the session (6) and going
			 * on-hook (4).  HSF_CALL_INIT=N runs N cycles.
			 */
			const char *ci = getenv("HSF_CALL_INIT");
			int cycles = ci ? atoi(ci) : 0;
			for (int cyc = 0; cyc < cycles; cyc++) {
				/* NO bus reset and NO SET_CONFIGURATION here: the
				 * COLD init capture has neither.  They appear in
				 * cap-full only because that was a SECOND driver
				 * load.  HSF_INIT_RESET=1 puts them back. */
				if (getenv("HSF_INIT_RESET")) {
					hsf_fxo_bus_reset(d);
					hsf_fxo_set_configuration(d);
				}
				uint8_t inf[5];
				hsf_fxo_get_information(d, inf);
				hsf_fxo_clear_notify_halt(d);
				codec_open_prelude(d, reg_d6, reg_da, reg_dc, reg_e4);
				hsf_fxo_script_run_index(d, HSF_SCRIPT_SIGNAL, 3, NULL, 0);
				hsf_fxo_script_run_index(d, HSF_SCRIPT_SIGNAL, 1, NULL, 0);
				unsigned i9 = script_count(HSF_SCRIPT_SESSION_A);
				if (hsf_fxo_script_run(d, HSF_SCRIPT_SESSION_A, NULL, 0) == 0)
					wait_script(HSF_SCRIPT_SESSION_A, i9, 1400);
				unsigned i5 = script_count(HSF_SCRIPT_SESSION_B);
				if (hsf_fxo_script_run(d, HSF_SCRIPT_SESSION_B, NULL, 0) == 0)
					wait_script(HSF_SCRIPT_SESSION_B, i5, 1400);
				unsigned i6 = script_count(HSF_SCRIPT_SESSION_END);
				if (hsf_fxo_script_run(d, HSF_SCRIPT_SESSION_END, NULL, 0) == 0)
					wait_script(HSF_SCRIPT_SESSION_END, i6, 1400);
				unsigned i4 = script_count(HSF_SCRIPT_ON_HOOK);
				if (hsf_fxo_script_run(d, HSF_SCRIPT_ON_HOOK, NULL, 0) == 0)
					wait_script(HSF_SCRIPT_ON_HOOK, i4, 1400);
				hsf_fxo_script_run_index(d, HSF_SCRIPT_SIGNAL, 1, NULL, 0);
				printf("  call-seq: init cycle %d done\n", cyc + 1);
			}
			/*
			 * Prime the transmit ring HERE -- after any init cycle
			 * and before the call registers.  This device only
			 * starts continuous receive when bulk OUT is already
			 * queued, but queueing it before the init (which is what
			 * main() used to do) puts transfers against a session the
			 * init then tears down with script 6.
			 */
			if (do_feed) {
				uint8_t pre[256];
				for (int i = 0; i < 4; i++) {
					fill_tx(pre);
					if (hsf_fxo_tx_submit(d, pre, g_tx_block) < 0)
						break;
				}
			}
			static const uint16_t regs_default[] = {
				0x35b7, 0x2004, 0xa208, 0xf200,
				0xaae8, 0x6040, 0x35b4
			};
			uint16_t regs[8];
			size_t nregs = sizeof regs_default / sizeof regs_default[0];
			memcpy(regs, regs_default, sizeof regs_default);
			/* HSF_CALL_REGS=w,w,... overrides the call's codec register
			 * words, so the gain field can be swept without a rebuild.
			 * Receive currently clips ~13% of samples, so one of these
			 * is 20-plus dB hot against the vendor's setting. */
			const char *cr2 = getenv("HSF_CALL_REGS");
			if (cr2) {
				nregs = 0;
				char buf[128];
				snprintf(buf, sizeof buf, "%s", cr2);
				for (char *tok = strtok(buf, ","); tok && nregs < 8;
				     tok = strtok(NULL, ","))
					regs[nregs++] = (uint16_t)strtoul(tok, NULL, 0);
			}
			for (size_t i = 0; i < nregs; i++)
				script2_reg(d, regs[i]);
			hsf_fxo_script_run_index(d, HSF_SCRIPT_SIGNAL, 1, NULL, 0);
			unsigned b9 = script_count(HSF_SCRIPT_SESSION_A);
			if (hsf_fxo_script_run(d, HSF_SCRIPT_SESSION_A, NULL, 0) == 0)
				wait_script(HSF_SCRIPT_SESSION_A, b9, 1400);
			unsigned b5 = script_count(HSF_SCRIPT_SESSION_B);
			if (hsf_fxo_script_run(d, HSF_SCRIPT_SESSION_B, NULL, 0) == 0)
				wait_script(HSF_SCRIPT_SESSION_B, b5, 1400);
			/* bulk runs from here, as it does for the driver */
			hsf_fxo_arm_rx(d);
			if (do_feed) {
				uint8_t first[256];
				for (int i = 0; i < 4; i++) {
					fill_tx(first);
					if (hsf_fxo_tx_submit(d, first, g_tx_block) < 0)
						break;
				}
			}
			g_armed = true;
			/* off-hook, then the trailing register write */
			/* HSF_NO_HOOK=1 runs the identical sequence but stays
			 * ON-HOOK -- the control that says whether what arrives
			 * on the receive stream is the line at all. */
			if (!getenv("HSF_NO_HOOK")) {
				hsf_fxo_script_run_index(d, HSF_SCRIPT_SIGNAL, 3, NULL, 0);
				unsigned b3 = script_count(HSF_SCRIPT_OFF_HOOK);
				if (hsf_fxo_script_run(d, HSF_SCRIPT_OFF_HOOK, NULL, 0) == 0)
					wait_script(HSF_SCRIPT_OFF_HOOK, b3, 1400);
				printf("  call-seq: off-hook sent\n");
			} else {
				printf("  call-seq: STAYING ON-HOOK (control)\n");
			}
			/* HSF_TRAIL_REG overrides the trailing post-off-hook write
			 * (0 skips it) so its effect can be isolated. */
			const char *tr = getenv("HSF_TRAIL_REG");
			unsigned trail = tr ? (unsigned)strtoul(tr, NULL, 0) : 0xaac8;
			if (trail)
				script2_reg(d, (uint16_t)trail);
			/* The vendor's capture sends a SECOND 0x35b7 about 2 s after
			 * off-hook, immediately followed by script 8 wIndex 3 and
			 * script 3 again.  Replaying only the register write did not
			 * test that observed transition.  HSF_TRAIL2_REG / _MS make
			 * the complete late sequence an experiment; 0 disables. */
			const char *t2 = getenv("HSF_TRAIL2_REG");
			if (t2) {
				const char *t2ms = getenv("HSF_TRAIL2_MS");
				usleep((useconds_t)(t2ms ? atoi(t2ms) : 2000) * 1000);
				script2_reg(d, (uint16_t)strtoul(t2, NULL, 0));
				hsf_fxo_script_run_index(d, HSF_SCRIPT_SIGNAL, 3, NULL, 0);
				if (!getenv("HSF_NO_HOOK")) {
					unsigned b3 = script_count(HSF_SCRIPT_OFF_HOOK);
					if (hsf_fxo_script_run(d, HSF_SCRIPT_OFF_HOOK,
					                           NULL, 0) == 0)
						wait_script(HSF_SCRIPT_OFF_HOOK, b3, 1400);
				}
			}
			/* TX is already running while the control plane is assembled, so
			 * fill_tx() has advanced (and, with the two-second late sequence,
			 * exhausted) the dial programme before the final line state exists.
			 * Start the actual dial delay at the completed call transition. */
			g_dial_pos = 0;
			g_dial_samples = 0;
			g_dial_gap = false;
			g_dial_waited = 0;
			goto codec_done;
		}
		if (vendor_seq) {
			/*
			 * The full replay, read off usbmon.  The driver runs a
			 * complete init cycle TWICE at load -- each one opening
			 * a session and then closing it again with script 6 and
			 * going on-hook with script 4 -- and the actual call is
			 * only script 2, 8, 9, 5.  So the codec register
			 * programming belongs to INIT, not to the call, and a
			 * probe that does it once as part of the call has the
			 * device in a state the driver never leaves it in.
			 */
			for (int cycle = 0; cycle < 2; cycle++) {
				uint8_t info[5];
				hsf_fxo_get_information(d, info);
				hsf_fxo_clear_notify_halt(d);
				codec_open_prelude(d, reg_d6, reg_da, reg_dc, reg_e4);
				hsf_fxo_script_run_index(d, HSF_SCRIPT_SIGNAL, 3, NULL, 0);
				hsf_fxo_script_run_index(d, HSF_SCRIPT_SIGNAL, 1, NULL, 0);
				unsigned c9 = script_count(HSF_SCRIPT_SESSION_A);
				if (hsf_fxo_script_run(d, HSF_SCRIPT_SESSION_A, NULL, 0) == 0)
					wait_script(HSF_SCRIPT_SESSION_A, c9, 1400);
				unsigned c5 = script_count(HSF_SCRIPT_SESSION_B);
				if (hsf_fxo_script_run(d, HSF_SCRIPT_SESSION_B, NULL, 0) == 0)
					wait_script(HSF_SCRIPT_SESSION_B, c5, 1400);
				unsigned c6 = script_count(HSF_SCRIPT_SESSION_END);
				if (hsf_fxo_script_run(d, HSF_SCRIPT_SESSION_END, NULL, 0) == 0)
					wait_script(HSF_SCRIPT_SESSION_END, c6, 1400);
				unsigned c4 = script_count(HSF_SCRIPT_ON_HOOK);
				if (hsf_fxo_script_run(d, HSF_SCRIPT_ON_HOOK, NULL, 0) == 0)
					wait_script(HSF_SCRIPT_ON_HOOK, c4, 1400);
				hsf_fxo_script_run_index(d, HSF_SCRIPT_SIGNAL, 1, NULL, 0);
				printf("vendor-seq: init cycle %d done\n", cycle + 1);
			}

			/* The call itself: script 2 (0x35b7), 8, 9, 5. */
			uint8_t p2[8] = { 0x35, 0xb7 };
			hsf_fxo_script_run(d, 2, p2, sizeof p2);
			hsf_fxo_script_run_index(d, HSF_SCRIPT_SIGNAL, 1, NULL, 0);
			unsigned b9 = script_count(HSF_SCRIPT_SESSION_A);
			int r9 = hsf_fxo_script_run(d, HSF_SCRIPT_SESSION_A, NULL, 0);
			int w9 = (r9 == 0) ? wait_script(HSF_SCRIPT_SESSION_A, b9, 1400) : -1;
			printf("vendor-seq: script 9 -> send %d, completion %d\n", r9, w9);

			/* From here the arming is the CALLBACK's job. */
			g_arm_on_session_b = true;
			unsigned b5 = script_count(HSF_SCRIPT_SESSION_B);
			int r5 = hsf_fxo_script_run(d, HSF_SCRIPT_SESSION_B, NULL, 0);
			int w5 = (r5 == 0) ? wait_script(HSF_SCRIPT_SESSION_B, b5, 1400) : -1;
			printf("vendor-seq: script 5 -> send %d, completion %d, "
			       "bulk armed from callback: %s\n",
			       r5, w5, g_armed ? "yes" : "NO");

			uint8_t after[5];
			printf("vendor-seq: device %s\n",
			       hsf_fxo_get_information(d, after) < 0 ? "NOT RESPONDING" : "alive");
			goto codec_done;
		}
		int r = 0;
		for (size_t i = 0; i < seq_n; i++) {
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
codec_done: ;
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
		codec_open_prelude(d, reg_d6, reg_da, reg_dc, reg_e4);

		/*
		 * Arm the data rings and prime AFTER the open, as the driver
		 * does from hsfusbcd2196_ rather than at attach.
		 *
		 * --arm-on-5 instead re-runs the 9/5 session start after the
		 * codec open and arms from SCRIPT 5'S COMPLETION CALLBACK, which
		 * is where the vendor driver submits its first bulk pair.  This
		 * is the one configuration that combines the two things known
		 * separately to matter: the register writes in the order that
		 * empirically yields receive, and the vendor's arming point.
		 */
		if (arm_on_5) {
			hsf_fxo_script_run_index(d, HSF_SCRIPT_SIGNAL, 1, NULL, 0);
			unsigned a9 = script_count(HSF_SCRIPT_SESSION_A);
			if (hsf_fxo_script_run(d, HSF_SCRIPT_SESSION_A, NULL, 0) == 0)
				wait_script(HSF_SCRIPT_SESSION_A, a9, 1400);
			g_arm_on_session_b = true;
			unsigned a5 = script_count(HSF_SCRIPT_SESSION_B);
			if (hsf_fxo_script_run(d, HSF_SCRIPT_SESSION_B, NULL, 0) == 0)
				wait_script(HSF_SCRIPT_SESSION_B, a5, 1400);
			printf("arm-on-5: bulk armed from callback: %s\n",
			       g_armed ? "yes" : "NO");
		} else {
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

	if (n_raw && raw_post) {
		int r = hsf_fxo_script_load(d, 0xFF01, raw, n_raw);
		printf("raw script (%zu bytes, after stream open): %s\n", n_raw,
		       r == 0 ? "accepted" : "REJECTED");
	}

	if (stream_secs > 0) {
		printf("streaming for %ds%s...\n", stream_secs,
		       do_feed ? " (feeding signed-linear silence out)" : "");
		if (trickle_ms > 0) {
			g_tx_pace_rx = false;	/* main loop owns the feed */
			int n = stream_secs * 1000 / trickle_ms;
			for (int k = 0; k < n; k++) {
				uint8_t b[256];
				fill_tx(b);
				(void)hsf_fxo_tx_submit(d, b, g_tx_block);
				usleep((useconds_t)trickle_ms * 1000);
			}
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

			/* hsfusbcd2195_/hsfusbcd2201_ closes the session with
			 * script 6 and then hsfusbcd2166_/hsfusbcd2185_ returns
			 * the DAA relay on-hook with script 4.  Omitting the latter
			 * leaves the ATA port seized between otherwise independent
			 * probe runs, so the next run has no dial tone to grade. */
			unsigned before4 = script_count(HSF_SCRIPT_ON_HOOK);
			int r4 = hsf_fxo_script_run(d, HSF_SCRIPT_ON_HOOK, NULL, 0);
			if (r4 == 0)
				r4 = wait_script(HSF_SCRIPT_ON_HOOK, before4, 2000);
			printf("on-hook (script 4): %s\n",
			       r4 == 0 ? "completed" : "no completion within 2s");
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
