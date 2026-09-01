/*
 * hsf_fxo.c -- Conexant HSF USB modem (0572:1300) line-side codec transport.
 *
 * Reimplements the GPL half of hsfmodem-7.80 (modules/osusb.c) over libusb.
 * Everything here has been exercised against the real device except the
 * script calls at the bottom, which are the remaining unknown.
 *
 * Two hard-won gotchas are load-bearing in this file:
 *
 *  1. A FAILED CONTROL TRANSFER WEDGES EP0.  Every request after it returns
 *     EPIPE until the pipe is cleared.  Without libusb_clear_halt(0) between
 *     attempts, stalls look like protocol answers and are not.  This cost
 *     three rounds of wrong conclusions.
 *
 *  2. libusb's darwin backend serves GET_DESCRIPTOR(DEVICE) from cache, so it
 *     succeeding proves nothing about the wire.  Use GET_INFROMATION as the
 *     liveness check, and bracket any probing with it.
 *
 * Do not call libusb_reset_device() on this part: it drops off the bus and
 * needs a physical replug.
 */

#include "hsf_fxo.h"

#include <errno.h>
#include <libusb.h>	/* include path comes from LIBUSB_CFLAGS in makefile */
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* osusb.h: MAX_{RECEIVE,SEND}_URB_BUFSIZE = MAX_BULK_DATA_PACKET*4, and
 * NUM_{RECEIVE,SEND}_URBS = 32 for the non-HCF (HSF) build. */
#define HSF_XFER_SIZE   256
#define HSF_NUM_RX      32
#define HSF_NUM_TX      32
#define HSF_NOTIFY_SIZE 64

#define EP_BULK_OUT 0x01
#define EP_BULK_IN  0x81
#define EP_NOTIFY   0x82

#define IF_DATA 0
#define IF_CTRL 1

/* osusb.c: sync path uses vendor|device, async path vendor|interface. */
#define RT_VENDOR_DEV_IN   (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_IN)
#define RT_VENDOR_DEV_OUT  (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_OUT)
#define RT_VENDOR_IF_OUT   (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE | LIBUSB_ENDPOINT_OUT)

#define CTRL_TIMEOUT_MS 1000
#define FW_BLOCK_SIZE   64	/* osusb.c FW_BLOCK_SIZE */

struct hsf_dev {
	libusb_context       *ctx;
	libusb_device_handle *h;

	struct libusb_transfer *rx[HSF_NUM_RX];
	struct libusb_transfer *tx[HSF_NUM_TX];
	uint8_t                 rx_buf[HSF_NUM_RX][HSF_XFER_SIZE];
	uint8_t                 tx_buf[HSF_NUM_TX][HSF_XFER_SIZE];
	bool                    tx_busy[HSF_NUM_TX];
	struct libusb_transfer *notify;
	uint8_t                 notify_buf[HSF_NOTIFY_SIZE];

	struct hsf_callbacks cb;
	struct hsf_stats     stats;

	pthread_mutex_t lock;
	pthread_t       thread;
	bool            running;
	bool            streaming;
};

/* ------------------------------------------------------------------ helpers */

/* Gotcha 1.  Cheap, and the difference between a readable device and a wedged
 * one, so it is called on every control failure rather than selectively. */
static void unwedge(struct hsf_dev *d)
{
	libusb_clear_halt(d->h, 0);
}

static int vendor_in(struct hsf_dev *d, uint8_t req, uint16_t val, uint16_t idx,
		     uint8_t *buf, uint16_t len)
{
	int r = libusb_control_transfer(d->h, RT_VENDOR_DEV_IN, req, val, idx,
					buf, len, CTRL_TIMEOUT_MS);
	if (r < 0)
		unwedge(d);
	return r;
}

static int vendor_out(struct hsf_dev *d, uint8_t rt, uint8_t req, uint16_t val,
		      uint16_t idx, const uint8_t *buf, uint16_t len)
{
	int r = libusb_control_transfer(d->h, rt, req, val, idx,
					(unsigned char *)buf, len, CTRL_TIMEOUT_MS);
	if (r < 0)
		unwedge(d);
	return r;
}

int hsf_fxo_get_information(struct hsf_dev *d, uint8_t info[5])
{
	/* osusb.c retries this up to 3 times with wIndex = attempt number. */
	for (int i = 0; i < 3; i++) {
		int r = vendor_in(d, CD2_GET_INFROMATION, 0, (uint16_t)i, info, 5);
		if (r == 5)
			return 0;
	}

	/*
	 * The device can end up in a state where descriptor reads still work but
	 * vendor requests do not answer at all (LIBUSB_ERROR_IO, IOKit
	 * kIOReturnNotResponding -- distinct from a STALL, which is an active
	 * rejection).  An explicit SET_CONFIGURATION 1 brings it back.  The
	 * likely cause is this driver's own SET_CONFIGURATION 0 during the
	 * firmware upload: osusb.c ignores the return of both, so a failed
	 * restore leaves the device unconfigured and silent.
	 */
	unsigned char cfg = 0;
	libusb_control_transfer(d->h, LIBUSB_ENDPOINT_IN, LIBUSB_REQUEST_GET_CONFIGURATION,
				0, 0, &cfg, 1, CTRL_TIMEOUT_MS);
	libusb_control_transfer(d->h, LIBUSB_ENDPOINT_OUT, LIBUSB_REQUEST_SET_CONFIGURATION,
				1, 0, NULL, 0, 5000);

	for (int i = 0; i < 3; i++) {
		int r = vendor_in(d, CD2_GET_INFROMATION, 0, (uint16_t)i, info, 5);
		if (r == 5)
			return 0;
	}
	return -EIO;
}

int hsf_fxo_read_eeprom(struct hsf_dev *d, uint16_t addr, uint8_t *buf, size_t len)
{
	int r = vendor_in(d, CD2_READ_EEPROM, addr, 0, buf, (uint16_t)len);
	return r < 0 ? -EIO : r;
}

/* ------------------------------------------------------------------ firmware */

static uint8_t *read_file(const char *path, size_t *out_len)
{
	FILE *f = fopen(path, "rb");
	if (!f)
		return NULL;
	if (fseek(f, 0, SEEK_END) != 0) {
		fclose(f);
		return NULL;
	}
	long n = ftell(f);
	rewind(f);
	if (n <= 0) {
		fclose(f);
		return NULL;
	}
	uint8_t *buf = malloc((size_t)n);
	if (buf && fread(buf, 1, (size_t)n, f) != (size_t)n) {
		free(buf);
		buf = NULL;
	}
	fclose(f);
	if (buf)
		*out_len = (size_t)n;
	return buf;
}

int hsf_fxo_load_firmware(struct hsf_dev *d, const char *rom_path)
{
	uint8_t info[5];

	if (hsf_fxo_get_information(d, info) < 0)
		return -EIO;
	if (info[2] == HSF_FAMILY_HSF)
		return 0;			/* already running */
	if (info[2] != HSF_FAMILY_BOOTLOADER)
		return -ENODEV;
	/* osusb.c switches on info[3]; only case 3 ("X4") maps to ROM_IMAGE. */
	if (info[3] != 3)
		return -ENOTSUP;

	size_t fw_len = 0;
	uint8_t *fw = read_file(rom_path ? rom_path : "hsf_rom_image.bin", &fw_len);
	if (!fw)
		return -ENOENT;

	/* osusb.c brackets the upload with SET_CONFIGURATION 0 / 1 and ignores
	 * the return of both, then sleeps 50 ms: "without this, downloads
	 * occasionally fail". */
	libusb_control_transfer(d->h, LIBUSB_ENDPOINT_OUT, LIBUSB_REQUEST_SET_CONFIGURATION,
				0, 0, NULL, 0, 5000);
	unsigned char drain[HSF_NOTIFY_SIZE];
	int got = 0;
	libusb_interrupt_transfer(d->h, EP_NOTIFY, drain, sizeof(drain), &got, 50);
	usleep(50 * 1000);

	int rc = 0;
	for (size_t off = 0; off < fw_len; ) {
		uint16_t n = (uint16_t)(fw_len - off > FW_BLOCK_SIZE ? FW_BLOCK_SIZE : fw_len - off);
		/* wValue = total size, wIndex = running offset. */
		int r = vendor_out(d, RT_VENDOR_DEV_OUT, CD2_UPLOAD_FIRMWARE,
				   (uint16_t)fw_len, (uint16_t)off, fw + off, n);
		if (r != n) {
			rc = -EIO;
			break;
		}
		off += (size_t)n;
	}
	free(fw);
	if (rc)
		return rc;

	/* Poll for the handover; in practice it reports on the first attempt. */
	rc = -EIO;
	for (int i = 0; i < 10; i++) {
		usleep(100 * 1000);
		if (hsf_fxo_get_information(d, info) == 0 && info[2] == HSF_FAMILY_HSF) {
			rc = 0;
			break;
		}
	}

	libusb_control_transfer(d->h, LIBUSB_ENDPOINT_OUT, LIBUSB_REQUEST_SET_CONFIGURATION,
				1, 0, NULL, 0, 5000);
	return rc;
}

/* ------------------------------------------------------------- transfer ring */

static void LIBUSB_CALL on_rx(struct libusb_transfer *t)
{
	struct hsf_dev *d = t->user_data;

	if (t->status == LIBUSB_TRANSFER_COMPLETED) {
		d->stats.rx_bytes += (uint64_t)t->actual_length;
		if (d->cb.rx_samples && t->actual_length > 0)
			d->cb.rx_samples(t->buffer, (size_t)t->actual_length, d->cb.user);
	} else if (t->status != LIBUSB_TRANSFER_TIMED_OUT) {
		d->stats.rx_errors++;
	}

	/* osusb.c re-arms from the upper layer on every completion; the RX ring
	 * has no other pacing, so re-submit unless we are shutting down. */
	if (d->streaming && libusb_submit_transfer(t) < 0)
		d->stats.rx_errors++;
}

static void LIBUSB_CALL on_tx(struct libusb_transfer *t)
{
	struct hsf_dev *d = t->user_data;

	pthread_mutex_lock(&d->lock);
	for (size_t i = 0; i < HSF_NUM_TX; i++) {
		if (d->tx[i] == t) {
			d->tx_busy[i] = false;
			break;
		}
	}
	pthread_mutex_unlock(&d->lock);

	if (t->status == LIBUSB_TRANSFER_COMPLETED) {
		d->stats.tx_bytes += (uint64_t)t->actual_length;
		if (d->cb.tx_done)
			d->cb.tx_done((size_t)t->actual_length, d->cb.user);
	} else {
		d->stats.tx_errors++;
	}
}

static void LIBUSB_CALL on_notify(struct libusb_transfer *t)
{
	struct hsf_dev *d = t->user_data;

	if (t->status == LIBUSB_TRANSFER_COMPLETED && t->actual_length >= 8) {
		/* osusb.c:851 casts the payload to a setup packet and byte-swaps
		 * wValue/wIndex/wLength; the codes themselves are unknown. */
		struct hsf_notification n;
		memset(&n, 0, sizeof(n));
		n.bmRequestType  = t->buffer[0];
		n.bNotification  = t->buffer[1];
		n.wValue         = (uint16_t)(t->buffer[2] | (t->buffer[3] << 8));
		n.wIndex         = (uint16_t)(t->buffer[4] | (t->buffer[5] << 8));
		n.wLength        = (uint16_t)(t->buffer[6] | (t->buffer[7] << 8));
		n.data_len = (size_t)t->actual_length - 8;
		if (n.data_len > sizeof(n.data))
			n.data_len = sizeof(n.data);
		memcpy(n.data, t->buffer + 8, n.data_len);

		d->stats.notifications++;
		if (d->cb.notification)
			d->cb.notification(&n, d->cb.user);
	}

	/* The notify URB is the one transfer osusb.c re-arms in its own
	 * completion routine (osusb.c:870) rather than from the upper layer. */
	if (d->running)
		libusb_submit_transfer(t);
}

static void *event_thread(void *arg)
{
	struct hsf_dev *d = arg;
	struct timeval tv = { .tv_sec = 0, .tv_usec = 100 * 1000 };

	while (d->running)
		libusb_handle_events_timeout_completed(d->ctx, &tv, NULL);
	return NULL;
}

/* ------------------------------------------------------------------ lifecycle */

struct hsf_dev *hsf_fxo_open(void)
{
	struct hsf_dev *d = calloc(1, sizeof(*d));
	if (!d)
		return NULL;

	if (libusb_init(&d->ctx) < 0) {
		free(d);
		return NULL;
	}

	/* Opens unprivileged on macOS even with AppleUSBCDCCompositeDevice
	 * attached -- libusb seizes it.  No kext or root needed. */
	d->h = libusb_open_device_with_vid_pid(d->ctx, HSF_VID, HSF_PID);
	if (!d->h) {
		libusb_exit(d->ctx);
		free(d);
		return NULL;
	}

	/* Both claims succeed; neither interface has a driver child. */
	libusb_claim_interface(d->h, IF_DATA);
	libusb_claim_interface(d->h, IF_CTRL);

	pthread_mutex_init(&d->lock, NULL);
	return d;
}

void hsf_fxo_close(struct hsf_dev *d)
{
	if (!d)
		return;
	hsf_fxo_stop(d);
	libusb_release_interface(d->h, IF_DATA);
	libusb_release_interface(d->h, IF_CTRL);
	libusb_close(d->h);
	libusb_exit(d->ctx);
	pthread_mutex_destroy(&d->lock);
	free(d);
}

int hsf_fxo_start(struct hsf_dev *d, const struct hsf_callbacks *cb)
{
	if (d->running)
		return -EBUSY;
	d->cb = *cb;
	d->running = true;
	d->streaming = true;

	d->notify = libusb_alloc_transfer(0);
	if (!d->notify)
		return -ENOMEM;
	libusb_fill_interrupt_transfer(d->notify, d->h, EP_NOTIFY, d->notify_buf,
				       HSF_NOTIFY_SIZE, on_notify, d, 0);
	libusb_submit_transfer(d->notify);

	for (size_t i = 0; i < HSF_NUM_RX; i++) {
		d->rx[i] = libusb_alloc_transfer(0);
		if (!d->rx[i])
			return -ENOMEM;
		libusb_fill_bulk_transfer(d->rx[i], d->h, EP_BULK_IN, d->rx_buf[i],
					  HSF_XFER_SIZE, on_rx, d, 0);
		libusb_submit_transfer(d->rx[i]);
	}
	for (size_t i = 0; i < HSF_NUM_TX; i++) {
		d->tx[i] = libusb_alloc_transfer(0);
		if (!d->tx[i])
			return -ENOMEM;
		d->tx_busy[i] = false;
	}

	if (pthread_create(&d->thread, NULL, event_thread, d) != 0) {
		d->running = false;
		return -EIO;
	}
	return 0;
}

void hsf_fxo_stop(struct hsf_dev *d)
{
	if (!d->running)
		return;
	d->streaming = false;
	d->running = false;

	for (size_t i = 0; i < HSF_NUM_RX; i++)
		if (d->rx[i])
			libusb_cancel_transfer(d->rx[i]);
	if (d->notify)
		libusb_cancel_transfer(d->notify);

	pthread_join(d->thread, NULL);

	for (size_t i = 0; i < HSF_NUM_RX; i++)
		if (d->rx[i]) {
			libusb_free_transfer(d->rx[i]);
			d->rx[i] = NULL;
		}
	for (size_t i = 0; i < HSF_NUM_TX; i++)
		if (d->tx[i]) {
			libusb_free_transfer(d->tx[i]);
			d->tx[i] = NULL;
		}
	if (d->notify) {
		libusb_free_transfer(d->notify);
		d->notify = NULL;
	}
}

int hsf_fxo_tx_submit(struct hsf_dev *d, const uint8_t *data, size_t len)
{
	if (!d->streaming)
		return -EINVAL;
	if (len > HSF_XFER_SIZE)
		return -EMSGSIZE;

	pthread_mutex_lock(&d->lock);
	size_t slot = HSF_NUM_TX;
	for (size_t i = 0; i < HSF_NUM_TX; i++) {
		if (!d->tx_busy[i]) {
			slot = i;
			d->tx_busy[i] = true;
			break;
		}
	}
	pthread_mutex_unlock(&d->lock);

	if (slot == HSF_NUM_TX) {
		/* Ring full.  Bulk carries no implicit clock, so this must be
		 * counted rather than swallowed -- a starved or backed-up TX
		 * path is a sample-accounting fault, not a hiccup. */
		d->stats.underruns++;
		return -EAGAIN;
	}

	memcpy(d->tx_buf[slot], data, len);
	libusb_fill_bulk_transfer(d->tx[slot], d->h, EP_BULK_OUT, d->tx_buf[slot],
				  (int)len, on_tx, d, 0);
	if (libusb_submit_transfer(d->tx[slot]) < 0) {
		pthread_mutex_lock(&d->lock);
		d->tx_busy[slot] = false;
		pthread_mutex_unlock(&d->lock);
		d->stats.tx_errors++;
		return -EIO;
	}
	return 0;
}

void hsf_fxo_stats(const struct hsf_dev *d, struct hsf_stats *out)
{
	*out = d->stats;
}

/* -------------------------------------------------------------- line control */

/* hsf.cty Profile\0000 SMART_RELAYS, little-endian 16-bit as written there. */
const uint16_t hsf_smart_relays[HSF_RELAY_COUNT] = {
	[HSF_RELAY_GPIO_MASK]                    = 0xFFFF,
	[HSF_RELAY_GPIO_DEFAULT]                 = 0x80B5,
	[HSF_RELAY_OFFHOOK_PHONETOLINE]          = 0x80B6,
	[HSF_RELAY_OFFHOOK_PHONEOFFLINE]         = 0x80A6,
	[HSF_RELAY_ONHOOK_PHONETOLINE_CALLID]    = 0x80BD,
	[HSF_RELAY_ONHOOK_PHONETOLINE_NOCALLID]  = 0x80B5,
	[HSF_RELAY_ONHOOK_PHONEOFFLINE_CALLID]   = 0x80AD,
	[HSF_RELAY_ONHOOK_PHONEOFFLINE_NOCALLID] = 0x80A5,
	[HSF_RELAY_OFFHOOK_PULSE_MAKE]           = 0x80A4,
	[HSF_RELAY_OFFHOOK_PULSE_BREAK]          = 0x80A5,
	[HSF_RELAY_OFFHOOK_PULSESETUP]           = 0x80A4,
	[HSF_RELAY_OFFHOOK_PULSECLEAR]           = 0x80A4,
};

int hsf_fxo_script_load(struct hsf_dev *d, const uint8_t *body, size_t len)
{
	if (len == 0)
		return -EINVAL;		/* osusb.c:1364 asserts against this */
	int r = vendor_out(d, RT_VENDOR_IF_OUT, CD2_CONTROL_SCRIPT,
			   0, 1, body, (uint16_t)len);
	return r < 0 ? -EIO : 0;
}

int hsf_fxo_script_delete(struct hsf_dev *d)
{
	/* wIndex 3.  A stall here is normal: osusb.c:766 declines to retry it
	 * precisely because the device answers a delete with EPIPE. */
	libusb_control_transfer(d->h, RT_VENDOR_IF_OUT, CD2_CONTROL_SCRIPT,
				0, 3, NULL, 0, CTRL_TIMEOUT_MS);
	unwedge(d);
	return 0;
}

int hsf_fxo_script_start_codec(struct hsf_dev *d)
{
	(void)d;
	return -ENOSYS;		/* body unknown -- see hsf_fxo.h */
}

int hsf_fxo_script_set_hook(struct hsf_dev *d, bool off_hook)
{
	(void)d;
	(void)off_hook;
	return -ENOSYS;		/* body unknown -- see hsf_fxo.h */
}
