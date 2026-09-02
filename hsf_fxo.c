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
 * Do not call libusb_reset_device() on this part.  Measured on libusb 1.0.29
 * against a device that had stopped answering EP0: the darwin backend reports
 * "timeout waiting for reenumerate", libusb then crashes on the stale handle,
 * and the device DISAPPEARS FROM THE BUS -- zero entries in ioreg and
 * system_profiler afterwards.  It needs a physical replug, so this is not a
 * recovery path, it is a way to spend one.
 */

#include "hsf_fxo.h"
#include "hsf_scripts.h"

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
#define RT_VENDOR_IF_IN    (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_INTERFACE | LIBUSB_ENDPOINT_IN)

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
	bool                    defer_rx;
	struct libusb_transfer *notify;
	uint8_t                 notify_buf[HSF_NOTIFY_SIZE];

	struct hsf_callbacks cb;
	struct hsf_stats     stats;

	pthread_mutex_t lock;
	pthread_t       thread;
	bool            running;
	bool            streaming;
	int             inflight;	/* transfers libusb still owns */
};

/* ------------------------------------------------------------------ helpers */

/*
 * A submitted transfer belongs to libusb until its callback runs, and freeing
 * one before that is undefined -- in practice a use-after-free inside
 * libusb_exit().  Everything that submits goes through here, and every callback
 * that declines to re-submit calls xfer_done(), so hsf_fxo_stop() can wait for
 * the ring to drain instead of guessing.
 */
static int xfer_submit(struct hsf_dev *d, struct libusb_transfer *t)
{
	pthread_mutex_lock(&d->lock);
	d->inflight++;
	pthread_mutex_unlock(&d->lock);
	int r = libusb_submit_transfer(t);
	if (r < 0) {
		pthread_mutex_lock(&d->lock);
		d->inflight--;
		pthread_mutex_unlock(&d->lock);
	}
	return r;
}

static void xfer_done(struct hsf_dev *d)
{
	pthread_mutex_lock(&d->lock);
	d->inflight--;
	pthread_mutex_unlock(&d->lock);
}

static int xfer_inflight(struct hsf_dev *d)
{
	pthread_mutex_lock(&d->lock);
	int n = d->inflight;
	pthread_mutex_unlock(&d->lock);
	return n;
}

/*
 * Gotcha 1.  Called on every control failure rather than selectively.
 *
 * NOTE (measured 2026-09-01, libusb 1.0.29): libusb_clear_halt(h, 0) DOES NOT
 * WORK ON macOS and never has.  The darwin backend resolves an endpoint address
 * to a pipeRef by searching the CLAIMED INTERFACES, and EP0 belongs to none of
 * them, so every call returns LIBUSB_ERROR_NOT_FOUND with
 *
 *     darwin_clear_halt: endpoint not found on any open interface
 *
 * on stderr.  This function was therefore a no-op on the one platform it was
 * written for, and gotcha 1's remedy has never actually been in force here --
 * the same shape of defect as the rest of this project's instrument bugs.
 *
 * The portable equivalent is the standard request libusb_clear_halt would have
 * sent, so send it directly.  It needs EP0 to be answering, which it is after a
 * STALL (an active rejection) and is not after the device has stopped
 * responding altogether; the return is reported so the two are distinguishable
 * instead of both looking like success.
 */
static int unwedge(struct hsf_dev *d)
{
	/* CLEAR_FEATURE(ENDPOINT_HALT=0) on endpoint 0, recipient endpoint. */
	int r = libusb_control_transfer(d->h, LIBUSB_RECIPIENT_ENDPOINT,
					LIBUSB_REQUEST_CLEAR_FEATURE, 0, 0,
					NULL, 0, CTRL_TIMEOUT_MS);
	return r < 0 ? r : 0;
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

	int last = 0;
	for (int i = 0; i < 3; i++) {
		last = vendor_in(d, CD2_GET_INFROMATION, 0, (uint16_t)i, info, 5);
		if (last == 5)
			return 0;
	}
	/* Report WHY.  "GET_INFROMATION failed" alone sent a session chasing the
	 * bootloader window when the real cause was another driver owning the
	 * interfaces -- LIBUSB_ERROR_NOT_FOUND / _BUSY / _ACCESS say so. */
	fprintf(stderr, "hsf_fxo: CD2_GET_INFROMATION -> %s (%d)\n",
		last < 0 ? libusb_error_name(last) : "short read", last);
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

static int reset_like(struct hsf_dev *d, uint8_t req, unsigned rt,
		      uint16_t wvalue, uint16_t windex)
{
	static const uint8_t rts[4] = {
		RT_VENDOR_DEV_OUT, RT_VENDOR_IF_OUT,
		RT_VENDOR_DEV_IN,  RT_VENDOR_IF_IN,
	};
	uint8_t in = (rt & 2) != 0;
	uint8_t buf[8];

	/* No unwedge() on failure: the point of these requests is to change the
	 * device's state, so a recovery that re-drives SET_CONFIGURATION would
	 * confuse "it did nothing" with "it worked and we undid it".  The raw
	 * libusb result is what the caller needs to see. */
	return libusb_control_transfer(d->h, rts[rt & 3], req, wvalue, windex,
				       in ? buf : NULL, in ? (uint16_t)sizeof buf : 0,
				       CTRL_TIMEOUT_MS);
}

int hsf_fxo_reset(struct hsf_dev *d, unsigned rt, uint16_t wvalue, uint16_t windex)
{
	return reset_like(d, CD2_RESET, rt, wvalue, windex);
}

int hsf_fxo_wake_on_ring(struct hsf_dev *d, unsigned rt, uint16_t wvalue, uint16_t windex)
{
	return reset_like(d, CD2_WAKEONRING, rt, wvalue, windex);
}

/* ------------------------------------------------------------- transfer ring */

static void LIBUSB_CALL on_rx(struct libusb_transfer *t)
{
	struct hsf_dev *d = t->user_data;

	if (t->status == LIBUSB_TRANSFER_COMPLETED) {
		d->stats.rx_bytes += (uint64_t)t->actual_length;
		if (d->cb.rx_samples && t->actual_length > 0)
			d->cb.rx_samples(t->buffer, (size_t)t->actual_length, d->cb.user);
	} else if (t->status != LIBUSB_TRANSFER_TIMED_OUT &&
		   t->status != LIBUSB_TRANSFER_CANCELLED) {
		/* CANCELLED is our own hsf_fxo_stop(), not a device fault.  Counting
		 * it read as "31 of 32 RX transfers failed" on a ring that was
		 * simply armed and waiting, which is a very different diagnosis. */
		d->stats.rx_errors++;
		/* Counting these told us nothing for a whole session: a stall, a
		 * NAK timeout and a dead pipe are different faults with different
		 * fixes.  Keep the first status seen. */
		if (!d->stats.rx_first_error)
			d->stats.rx_first_error = (int)t->status + 1;
	}

	/* osusb.c re-arms from the upper layer on every completion; the RX ring
	 * has no other pacing, so re-submit unless we are shutting down. */
	if (!d->streaming) {
		xfer_done(d);
		return;
	}
	if (libusb_submit_transfer(t) < 0) {
		d->stats.rx_errors++;
		xfer_done(d);
	}
}

static void LIBUSB_CALL on_tx(struct libusb_transfer *t)
{
	struct hsf_dev *d = t->user_data;

	if (t->status == LIBUSB_TRANSFER_COMPLETED) {
		d->stats.tx_bytes += (uint64_t)t->actual_length;
		/* Match osusb.c's UsbTransmitDataCompletionRoutine ordering: the
		 * upper-layer refill callback runs while this request is still
		 * reserved, then ChangeUrbListEntryState() releases it.  Releasing
		 * first lets the callback resubmit the transfer whose completion
		 * callback is still executing, which is not what the driver does. */
		if (d->cb.tx_done)
			d->cb.tx_done((size_t)t->actual_length, d->cb.user);
	} else if (t->status != LIBUSB_TRANSFER_CANCELLED) {
		d->stats.tx_errors++;
		/* Same lesson as the RX side: a stall, a NAK timeout and a
		 * cancelled transfer are different faults.  A STALL here means
		 * the device REJECTED what we sent and the pipe stays halted
		 * until cleared -- which looks exactly like "it stopped
		 * accepting data". */
		if (!d->stats.tx_first_error)
			d->stats.tx_first_error = (int)t->status + 1;
	}

	pthread_mutex_lock(&d->lock);
	for (size_t i = 0; i < HSF_NUM_TX; i++) {
		if (d->tx[i] == t) {
			d->tx_busy[i] = false;
			break;
		}
	}
	pthread_mutex_unlock(&d->lock);
	xfer_done(d);
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
	if (!d->running) {
		xfer_done(d);
		return;
	}
	if (libusb_submit_transfer(t) < 0)
		xfer_done(d);
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

	/*
	 * This used to read "both claims succeed; neither interface has a driver
	 * child" and DISCARD both return values.  That is false whenever a
	 * kernel driver is attached -- the vendor hsfusbcd2, or cdc_acm on a
	 * host that binds it -- and the failure is silent in the worst way: the
	 * probe runs to completion, every transfer is rejected by usbfs, and the
	 * only trace is a dmesg line ("did not claim interface 0 before use")
	 * that nothing here was reading.  A run in that state reports zero
	 * received bytes and no notifications, which is indistinguishable from
	 * a device that is ignoring us.
	 */
	/* libusb_set_auto_detach_kernel_driver() is 1.0.16+; the capture guest
	 * ships 1.0.11, so detach explicitly instead. */
	for (int ifn = 0; ifn < 2; ifn++) {
		int which = ifn ? IF_CTRL : IF_DATA;
		if (libusb_kernel_driver_active(d->h, which) == 1 &&
		    libusb_detach_kernel_driver(d->h, which) < 0)
			fprintf(stderr, "hsf_fxo: interface %d has a kernel driver "
					"and it could not be detached\n", which);
		int cr = libusb_claim_interface(d->h, which);
		if (cr < 0) {
			fprintf(stderr, "hsf_fxo: claim interface %d failed: %s "
					"-- every transfer will be refused\n",
				which, libusb_error_name(cr));
			libusb_close(d->h);
			libusb_exit(d->ctx);
			free(d);
			return NULL;
		}
	}

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
	xfer_submit(d, d->notify);

	/* The driver's own RX granularity is ctx+0x874 = 64, not the 0x100 clamp
	 * in hsfusbcd2267_; HSF_RX_SIZE lets that be varied. */
	const char *rs = getenv("HSF_RX_SIZE");
	int rx_size = rs ? atoi(rs) : HSF_XFER_SIZE;
	if (rx_size <= 0 || rx_size > HSF_XFER_SIZE)
		rx_size = HSF_XFER_SIZE;

	/* How many RX transfers are in flight at once.  This ring holds 32 and
	 * used to submit all of them; the vendor driver keeps exactly ONE bulk
	 * IN outstanding and re-submits it from the completion (hsfusbcd2212_
	 * submits one RX and one TX per pass).  HSF_RX_INFLIGHT makes the depth
	 * an experiment rather than an assumption. */
	const char *ri = getenv("HSF_RX_INFLIGHT");
	size_t inflight = ri ? (size_t)atoi(ri) : HSF_NUM_RX;
	if (inflight < 1 || inflight > HSF_NUM_RX)
		inflight = HSF_NUM_RX;

	for (size_t i = 0; i < HSF_NUM_RX; i++) {
		d->rx[i] = libusb_alloc_transfer(0);
		if (!d->rx[i])
			return -ENOMEM;
		libusb_fill_bulk_transfer(d->rx[i], d->h, EP_BULK_IN, d->rx_buf[i],
					  rx_size, on_rx, d, 0);
		if (!d->defer_rx && i < inflight)
			xfer_submit(d, d->rx[i]);
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

	/* Order matters.  Clearing these two flags first stops the callbacks
	 * re-arming, so a cancelled transfer stays cancelled. */
	d->streaming = false;
	d->running = false;

	for (size_t i = 0; i < HSF_NUM_RX; i++)
		if (d->rx[i])
			libusb_cancel_transfer(d->rx[i]);
	for (size_t i = 0; i < HSF_NUM_TX; i++)
		if (d->tx[i] && d->tx_busy[i])
			libusb_cancel_transfer(d->tx[i]);
	if (d->notify)
		libusb_cancel_transfer(d->notify);

	pthread_join(d->thread, NULL);

	/*
	 * The event thread exits on !running, which it may well have observed
	 * BEFORE the cancellations were reaped -- so pump events here until
	 * libusb has handed every transfer back.  Freeing one it still owns is
	 * a use-after-free that surfaces inside libusb_exit(), which is exactly
	 * what this used to do: every probe run ended in SIGSEGV, and the crash
	 * was silently truncating the output it was there to collect.
	 */
	struct timeval tv = { .tv_sec = 0, .tv_usec = 50 * 1000 };
	for (int i = 0; i < 40 && xfer_inflight(d) > 0; i++)
		libusb_handle_events_timeout_completed(d->ctx, &tv, NULL);

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

void hsf_fxo_defer_rx(struct hsf_dev *d, bool defer)
{
	d->defer_rx = defer;
}

int hsf_fxo_arm_rx(struct hsf_dev *d)
{
	if (!d->streaming)
		return -EINVAL;
	for (size_t i = 0; i < HSF_NUM_RX; i++)
		if (d->rx[i] && xfer_submit(d, d->rx[i]) < 0)
			return -EIO;
	d->defer_rx = false;
	return 0;
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
	if (xfer_submit(d, d->tx[slot]) < 0) {
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

static int script_load_index(struct hsf_dev *d, uint16_t wvalue,
			     uint16_t windex, const uint8_t *body, size_t len)
{
	if (len == 0)
		return -EINVAL;		/* osusb.c:1364 asserts against this */
	int r = vendor_out(d, RT_VENDOR_IF_OUT, CD2_CONTROL_SCRIPT,
			   wvalue, windex, body, (uint16_t)len);
	return r < 0 ? -EIO : 0;
}

int hsf_fxo_script_load(struct hsf_dev *d, uint16_t wvalue,
			const uint8_t *body, size_t len)
{
	return script_load_index(d, wvalue, 1, body, len);
}

int hsf_fxo_script_delete(struct hsf_dev *d)
{
	/* wIndex 3.  A stall here is normal: osusb.c:766 declines to retry it
	 * precisely because the device answers a delete with EPIPE. */
	libusb_control_transfer(d->h, RT_VENDOR_IF_OUT, CD2_CONTROL_SCRIPT,
				0xFF01, 3, NULL, 0, CTRL_TIMEOUT_MS);
	unwedge(d);
	return 0;
}

static int script_run_index(struct hsf_dev *d, unsigned id, uint16_t windex,
			    const uint8_t *patch, size_t npatch)
{
	if (id >= sizeof hsf_scripts / sizeof hsf_scripts[0])
		return -EINVAL;
	const struct hsf_script *s = &hsf_scripts[id];
	if (!s->body || !s->len)
		return -EINVAL;
	if (npatch > s->npatch)
		npatch = s->npatch;

	/* The driver patches the template and assembles afterwards; the offsets
	 * in hsf_scripts.h are already carried through that assembly, and none
	 * of them lands on an opcode the assembler rewrites, so a byte store at
	 * the translated offset is exactly what the vendor path produces. */
	uint8_t body[256];
	if (s->len > sizeof body)
		return -EMSGSIZE;
	memcpy(body, s->body, s->len);
	for (size_t i = 0; i < npatch; i++) {
		if (s->patch_off[i] >= s->len)
			return -EINVAL;
		body[s->patch_off[i]] = patch[i];
	}

	return script_load_index(d, s->wvalue, windex, body, s->len);
}

int hsf_fxo_script_run(struct hsf_dev *d, unsigned id,
		       const uint8_t *patch, size_t npatch)
{
	return script_run_index(d, id, 1, patch, npatch);
}

int hsf_fxo_script_run_index(struct hsf_dev *d, unsigned id, uint16_t windex,
			     const uint8_t *patch, size_t npatch)
{
	return script_run_index(d, id, windex, patch, npatch);
}

/*
 * The vendor driver clears a halt on the interrupt-IN endpoint at the top of
 * every session bring-up -- observed on the wire (docs/hsf_usb_daa.md), not
 * derived from the blob.  Note this is endpoint 0x82, which IS inside a
 * claimed interface, unlike the EP0 clear that cannot work on darwin.
 */
int hsf_fxo_clear_notify_halt(struct hsf_dev *d)
{
	return libusb_clear_halt(d->h, EP_NOTIFY);
}

/*
 * The two things the vendor driver's init does that nothing here ever did: a
 * USB port reset and a SET_CONFIGURATION.  Both are plainly in the capture --
 * the driver resets the port, re-reads the descriptors, sets configuration 1,
 * and does it TWICE before the call -- and both are ordinary standard requests
 * we simply never issued.  A reset invalidates the claims, so they are dropped
 * and retaken around it.
 */
/*
 * Script 2's patch offsets are 15 and 16, big-endian.  Taken from the vendor
 * driver's own traffic: every script-2 body it sends is the template with
 * exactly those two bytes replaced (f0 f1 -> 20 04, a2 08, f2 00, aa e8,
 * 60 40, 35 b4).
 *
 * tools/hsf_scripts.py reports npatch=0 for this script, so
 * hsf_fxo_script_run() clamps the patch count to zero and sends the RAW
 * TEMPLATE.  Every codec register write this probe ever made was therefore the
 * same no-op body -- six identical transfers that configured nothing -- which
 * is invisible unless the wire is compared byte for byte, because the script id
 * and length are right.
 */
#define HSF_SCRIPT2_PATCH_OFF 15

int hsf_fxo_script2_reg(struct hsf_dev *d, uint16_t word)
{
	/* the table is positional: hsf_scripts[2] IS script 2 */
	if (2 >= sizeof hsf_scripts / sizeof hsf_scripts[0])
		return -EINVAL;
	const struct hsf_script *s = &hsf_scripts[2];
	if (s->len < HSF_SCRIPT2_PATCH_OFF + 2)
		return -EINVAL;
	uint8_t body[256];
	if (s->len > sizeof body)
		return -EMSGSIZE;
	memcpy(body, s->body, s->len);
	body[HSF_SCRIPT2_PATCH_OFF]     = (uint8_t)(word >> 8);
	body[HSF_SCRIPT2_PATCH_OFF + 1] = (uint8_t)(word & 0xff);
	return hsf_fxo_script_load(d, s->wvalue, body, s->len);
}

int hsf_fxo_bus_reset(struct hsf_dev *d)
{
	libusb_release_interface(d->h, IF_DATA);
	libusb_release_interface(d->h, IF_CTRL);
	int r = libusb_reset_device(d->h);
	if (r < 0)
		return r;
	for (int ifn = 0; ifn < 2; ifn++) {
		int which = ifn ? IF_CTRL : IF_DATA;
		if (libusb_kernel_driver_active(d->h, which) == 1)
			libusb_detach_kernel_driver(d->h, which);
		int cr = libusb_claim_interface(d->h, which);
		if (cr < 0)
			return cr;
	}
	return 0;
}

int hsf_fxo_set_configuration(struct hsf_dev *d)
{
	/* SET_CONFIGURATION must not be issued while interfaces are claimed. */
	libusb_release_interface(d->h, IF_DATA);
	libusb_release_interface(d->h, IF_CTRL);
	int r = libusb_set_configuration(d->h, 1);
	for (int ifn = 0; ifn < 2; ifn++) {
		int which = ifn ? IF_CTRL : IF_DATA;
		if (libusb_kernel_driver_active(d->h, which) == 1)
			libusb_detach_kernel_driver(d->h, which);
		libusb_claim_interface(d->h, which);
	}
	return r;
}

int hsf_fxo_script_start_codec(struct hsf_dev *d)
{
	/* hsfusbcd2165_ sends 9 and waits up to 0x578/1400 ms.  On the normal
	 * completion path it then sends 5 and returns.  A second 9 is only the
	 * timeout retry; treating 9,5,9 as an unconditional sequence was a
	 * disassembly error and left the codec in the wrong state. */
	int r = hsf_fxo_script_run(d, HSF_SCRIPT_SESSION_A, NULL, 0);
	if (r < 0)
		return r;
	return hsf_fxo_script_run(d, HSF_SCRIPT_SESSION_B, NULL, 0);
}

int hsf_fxo_script_set_hook(struct hsf_dev *d, bool off_hook)
{
	/* hsfusbcd2185_ does not dispatch an off-hook relay code directly to
	 * script 3.  At .text 0x4870 it first calls hsfusbcd2250_, which queues
	 * the complete script-8 body with wIndex 3, and only then queues script
	 * 3.  Sending an empty delete request or script 3 alone is not the same
	 * firmware transition. */
	if (off_hook) {
		int r = script_run_index(d, HSF_SCRIPT_SIGNAL, 3, NULL, 0);
		/* The delete request normally completes with EPIPE.  osusb.c's
		 * control completion path explicitly declines to retry wIndex 3;
		 * hsfusbcd2185_ nevertheless queues script 3 immediately afterwards. */
		(void)r;
	}
	return hsf_fxo_script_run(d,
				  off_hook ? HSF_SCRIPT_OFF_HOOK : HSF_SCRIPT_ON_HOOK,
				  NULL, 0);
}
