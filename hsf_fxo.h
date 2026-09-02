/*
 * hsf_fxo.h -- Conexant HSF USB modem (0572:1300) as a line-side codec.
 *
 * This device is a codec plus SmartDAA on a USB bulk pipe: no DSP, no AT
 * interpreter, no opinions about the samples.  The host does everything, which
 * makes it the analogue-side equivalent of the DS0 the rest of this project is
 * built around -- and the reason constraint 1 in CLAUDE.md is satisfiable here
 * where a controller-based modem could never satisfy it.
 *
 * Device layout (read off the wire, 2026-09-01):
 *   IF0  CDC-Data,  bulk OUT 0x01 + bulk IN 0x81, 64 B packets, no alt settings
 *   IF1  CDC sub 1 (Direct Line Control Model), proto 0xff, interrupt IN 0x82
 *
 * IF1's DLCM subclass is cosmetic -- every CDC PSTN class request (SET_HOOK_STATE
 * included) stalls.  Line control is the vendor CD2_CONTROL_SCRIPT request; its
 * request framing and its script bodies are now recovered from Conexant's closed
 * hsfusbcd2 rather than captured, and live in hsf_scripts.h.  See the line
 * control section below.
 *
 * What IS known and implemented here comes from the GPL half of hsfmodem-7.80
 * (modules/osusb.c, modules/cnxthwusb_common.c and the imported headers).
 */

#ifndef HSF_FXO_H
#define HSF_FXO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define HSF_VID 0x0572
#define HSF_PID 0x1300

/* CD2 vendor requests -- usbhalos.h CD2REQUESTTYPE, in declaration order. */
enum hsf_cd2_request {
	CD2_GET_INFROMATION = 0,	/* sic: the typo is Conexant's */
	CD2_UPLOAD_FIRMWARE = 1,
	CD2_CONTROL_SCRIPT  = 2,
	CD2_READ_EEPROM     = 3,
	CD2_WRITE_EEPROM    = 4,
	CD2_RESET           = 5,
	CD2_WAKEONRING      = 6,
};

/* info[2] from CD2_GET_INFROMATION. */
enum hsf_family {
	HSF_FAMILY_BOOTLOADER = 1,	/* firmware not loaded; wants ROM_IMAGE */
	HSF_FAMILY_HCF        = 2,
	HSF_FAMILY_HSF        = 3,	/* firmware running */
};

/*
 * A device notification.  osusb.c:851 casts the interrupt payload straight to a
 * USB setup packet, so the header is bmRequestType/bRequest/wValue/wIndex/
 * wLength, little-endian, followed by wLength bytes.
 *
 * Measured on the device 2026-09-01, and NOT the CDC PSTN codes the earlier
 * guess here proposed:
 *
 *   bmRequestType 0xc1, bNotification 0x02 always.
 *   wValue 1  -- a script completed.  data[0] is the script id, data[1] a
 *                status: 0x01 succeeded, 0x80 did not (script 8 with no signal
 *                id selected is the one that reports 0x80).
 *   wValue 2  -- an asynchronous device event.  data[0] is the event code and
 *                data[1] carries a state bit in 0x80.
 *   wIndex    -- a millisecond timestamp, which is what makes the events
 *                readable as a waveform.
 *
 * hsfusbcd2196_ (.text 0x3cf0) is the driver's handler; it requires
 * bNotification 2, copies data[] to its context and dispatches on data[0]
 * through a 34-entry table at .rodata 0x324.  The data[0] == 5 case is what
 * starts the driver's data pump, so a script-5 completion is the "begin
 * streaming" signal.
 */

/*
 * Event code 8 is RING, and the timestamps prove it rather than assuming it.
 * With the modem on-hook on a live line and a call placed to it, 278 events
 * arrived carrying only 0x0800 and 0x0880 -- i.e. one bit toggling.  Their
 * wIndex spacing alternates 14 ms / 25 ms, a 39 ms period = 25.6 Hz, which is
 * the DAA reporting each half cycle of the 25 Hz ring voltage; and the bursts
 * are separated by ~190 ms (x5) and ~2000 ms (x3), which is the 400/200/400/2000
 * double-ring cadence.  So bit 0x80 of data[1] is the instantaneous ring
 * polarity, not a ring-start/ring-stop flag.
 */
#define HSF_EVENT_RING 0x08
struct hsf_notification {
	uint8_t  bmRequestType;
	uint8_t  bNotification;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
	uint8_t  data[56];
	size_t   data_len;
};

struct hsf_dev;

/*
 * Callbacks run on the libusb event thread.  Keep them short: the RX callback
 * is on the audio path, and the transport re-arms the transfer as soon as it
 * returns.  Neither may call back into hsf_fxo_* except hsf_fxo_tx_submit().
 */
struct hsf_callbacks {
	/* Raw codec samples.  No framing: osusb.c hands the URB payload straight
	 * to the upper layer with no header of any kind (OsUsbMakeDataReceiveRequest
	 * -> UsbReceiveDataCompletionRoutine -> cbUsbHalDataReceived). */
	void (*rx_samples)(const uint8_t *data, size_t len, void *user);
	void (*tx_done)(size_t len, void *user);
	void (*notification)(const struct hsf_notification *n, void *user);
	void *user;
};

/* Lifecycle ---------------------------------------------------------------- */

struct hsf_dev *hsf_fxo_open(void);
void            hsf_fxo_close(struct hsf_dev *d);

/*
 * Load ROM_IMAGE if the device is in bootloader state.  No-op (returns 0) if
 * firmware is already running.  rom_path defaults to "hsf_rom_image.bin", built
 * from the shipped c2firmware.h by tools/hsf_extract_rom.py.
 *
 * The firmware is volatile: it survives until the device is unplugged.
 */
int hsf_fxo_load_firmware(struct hsf_dev *d, const char *rom_path);

int hsf_fxo_get_information(struct hsf_dev *d, uint8_t info[5]);
int hsf_fxo_read_eeprom(struct hsf_dev *d, uint16_t addr, uint8_t *buf, size_t len);

/* Streaming ---------------------------------------------------------------- */

/*
 * Arm the RX ring and start the event thread.  32 transfers of 256 bytes each
 * way, matching NUM_RECEIVE_URBS / MAX_RECEIVE_URB_BUFSIZE in osusb.h.
 *
 * NOTE: until the codec is started (see hsf_fxo_script_start_codec) the device
 * NAKs the bulk pipes -- measured: bulk OUT times out rather than stalling, and
 * bulk IN yields nothing.  Streaming is gated on the DAA script, not on the
 * host posting transfers.
 */
int  hsf_fxo_start(struct hsf_dev *d, const struct hsf_callbacks *cb);
void hsf_fxo_stop(struct hsf_dev *d);

/* Queue one buffer to the codec.  Returns -EAGAIN if the TX ring is full. */
int hsf_fxo_tx_submit(struct hsf_dev *d, const uint8_t *data, size_t len);

/* Counters.  underruns is the one that matters: bulk gives no implicit clock,
 * so a starved TX ring must surface as a counted event rather than as missing
 * samples -- the same class of defect as the clock_recovery splice. */
struct hsf_stats {
	uint64_t rx_bytes, tx_bytes;
	uint64_t rx_errors, tx_errors;
	uint64_t underruns;		/* tx ring empty when a transfer completed */
	uint64_t notifications;
	int      rx_first_error;	/* libusb_transfer_status + 1, 0 if none */
	int      tx_first_error;
};
void hsf_fxo_stats(const struct hsf_dev *d, struct hsf_stats *out);

/* Line control ------------------------------------------------------------ */

/*
 * CD2_CONTROL_SCRIPT is the whole of line control, and the request framing is
 * now known.  hsfusbcd2210_ (.text 0x2c40) issues it as
 *
 *     bmRequestType 0x41 (vendor | interface | OUT)
 *     bRequest      2    (CD2_CONTROL_SCRIPT)
 *     wValue        0xFF01, or 0xFF02 for script 8 alone (.text 0x2ea7)
 *     wIndex        1 to load, 3 to delete (a delete stalls, and that is normal)
 *     body          the assembled script
 *
 * wValue was the missing piece: everything here previously sent 0, which no
 * body can survive.  wIndex 1 was right all along.
 *
 * The body is NOT the template held in the driver's table.  Templates are an
 * intermediate language; hsfusbcd290_ (.text 0x6f20) assembles them -- resolving
 * labels into absolute offsets and remapping one opcode range -- and the result
 * is what goes on the wire.  tools/hsf_scripts.py reimplements that assembler
 * and generates hsf_scripts.h, which is what this file sends.
 *
 * Scripts are enqueued by hsfusbcd2176_ (.text 0x3070) with a script id, the
 * wIndex, and up to eight patch bytes written into the template BEFORE assembly.
 * hsf_scripts.h carries the patch offsets already translated into the assembled
 * body (no offset lands on a remapped operand, so a patch is a plain byte
 * store).
 *
 * EVERY id 0-33 is enqueued -- id 34 is the queue's end sentinel.  Do not
 * enumerate them by the four-push call pattern: sites that share an argument
 * tail push only the id and jump into another site's call, which hides script
 * 11 (hsfusbcd2261_) and the whole 0/14-33 signal range (hsfusbcd2187_).
 */
enum hsf_script_id {
	/* Identified from the relay-code dispatch in hsfusbcd2185_ (.text 0x47b0),
	 * which routes DEVMGR_DAA_RELAY_CODE 2, 3 and 8-11 -- every off-hook and
	 * pulse-dial state -- to one script and 4-7, the on-hook states, to the
	 * other.  hsfusbcd2180_ and hsfusbcd2166_ are the direct entry points. */
	HSF_SCRIPT_OFF_HOOK    = 3,
	HSF_SCRIPT_ON_HOOK     = 4,

	/* Enqueued in order by hsfusbcd2165_ (.text 0x4480), the function that
	 * zeroes the whole per-call state block: 9, then 5, then 9 again.  This
	 * is the session bring-up, and so the best candidate for what ungates the
	 * bulk pipes.  NOT confirmed against hardware. */
	HSF_SCRIPT_SESSION_A   = 9,
	HSF_SCRIPT_SESSION_B   = 5,

	/* hsfusbcd2195_ / hsfusbcd2201_, which set the "stopping" flag first. */
	HSF_SCRIPT_SESSION_END = 6,

	/* hsfusbcd2187_ (.text 0x32f0) is the signal generator.  Its jump table at
	 * .rodata 0x2d0 maps 21 signal ids onto scripts -- signal 0 to script 0,
	 * and signal n to script n+13 for n = 1..20 -- so scripts 0 and 14-33 are
	 * the tone/cadence set, each taking patch bytes from that function's own
	 * caller.  Signal ids 0x15/0x16 instead drive script 8, with wIndex 3
	 * (delete) to stop whatever signal is running. */
	HSF_SCRIPT_SIGNAL      = 8,
	HSF_SCRIPT_SIGNAL_BASE = 13,	/* + signal id, for signal 1..20 */

	/* hsfusbcd2220_ (.text 0x6910) patches three bytes: two context values
	 * divided by 1000 (so milliseconds) and one literal.  Pulse dialling. */
	HSF_SCRIPT_PULSE       = 7,

	/* hsfusbcd2261_ (.text 0x37f0) is the ONLY producer of these two, and it
	 * is a start/stop pair around one path code: it builds an 8-byte patch of
	 * zeros, writes the code into bytes 0 AND 1, and then enqueues script 12
	 * when its second argument is non-zero and script 11 when it is zero.
	 *
	 * The code comes from the first argument, and from the hardware variant
	 * at ctx+0x68: on the ordinary part 0 -> 0x02 and 1 -> 0x01 (2 and 3 do
	 * nothing at all), and on the 0x68 == 1 variant 0 -> 0x14, 1 -> 0x13,
	 * 2 -> 0x12, 3 -> 0x17.
	 *
	 * Where they are sent is what makes them interesting for the transmit
	 * path.  hsfusbcd2167_ (.text 0x6430) is the stream-open entry -- it is
	 * the function that sets the TX granularity to 0x80 and the RX to 0x40 at
	 * .text 0x6480 -- and on the way in it sends script 8 with wIndex 1
	 * (hsfusbcd2247_) and then 2261_(3, 1), i.e. START.  hsfusbcd2265_, the
	 * stop counterpart, sends 2261_(3, 0) and 2261_(1, 0), i.e. STOP twice.
	 * Off-hook (hsfusbcd2180_) is followed by 2261_(2, 1) and on-hook
	 * (hsfusbcd2166_, hsfusbcd2185_) by 2261_(2, 0).
	 *
	 * NOT confirmed against hardware.  These are the scripts that have only
	 * ever been sent as unpatched templates, and the patch byte is the whole
	 * of their meaning. */
	HSF_SCRIPT_PATH_START  = 12,
	HSF_SCRIPT_PATH_STOP   = 11,

	/*
	 * ctx+0x68 is NOT a hardware variant, which these notes first assumed.
	 * hsfusbcd2308_ (.text 0x1d30) is a property handler whose jump table
	 * sets it to 0, 1, 2 or 3 from an upper-layer call, so it is a runtime
	 * MODE, and 0 is the default.
	 *
	 * In mode 0 the table above collapses: 2261_(0,*) -> code 0x02,
	 * 2261_(1,*) -> code 0x01, and 2261_(2,*) and 2261_(3,*) DO NOTHING AT
	 * ALL.  Two consequences.  The stream-open start, 2261_(3,1), is a
	 * no-op -- so the vendor never sends script 12 there in this
	 * configuration, which is worth knowing before reading anything into a
	 * sweep of its codes.  And hsfusbcd2265_'s stop pair, 2261_(3,0) then
	 * 2261_(1,0), reduces to a single real call: script 11 with both patch
	 * bytes 0x01.
	 */
	HSF_SCRIPT_STOP_CODE   = 0x01,
};

/*
 * The DAA relay control words ARE known, even though nothing in the assembled
 * scripts carries one -- the firmware holds the table and the scripts select
 * from it, which is why the off-hook script contains no 0xA6.
 *
 * configtypes.h:407 DEVMGR_DAA_RELAY_CODE indexes a 12-entry table; hsf.cty
 * ships the values per country and per DAA type, and hsfcadmus2smart.inf says
 * this part is "DAA Type: SMART", so SMART_RELAYS is our column.  cvtinf.pl
 * pads them from 16 to 32 bits (DAA_RELAY_TYPE), so the wire value is
 * 0x000080xx.
 *
 * Differencing the states that vary in one property decodes the low byte:
 *   bit 4  phone jack connected to line   (0xB* vs 0xA*)
 *   bit 3  caller ID / on-hook monitor    (0xBD/0xB5, 0xAD/0xA5)
 *   bit 1  off-hook          } complementary
 *   bit 0  on-hook           }
 *   bit 2  set in every state -- constant
 *
 * Note index 6: on-hook, phone isolated, monitor ENABLED.  That is the DAA
 * listening to the line without seizing it -- the caller-ID path -- which is
 * how ring and line-in-use are observed before going off-hook.
 */
enum hsf_daa_relay {
	HSF_RELAY_GPIO_MASK                   = 0,
	HSF_RELAY_GPIO_DEFAULT                = 1,
	HSF_RELAY_OFFHOOK_PHONETOLINE         = 2,
	HSF_RELAY_OFFHOOK_PHONEOFFLINE        = 3,	/* the normal modem seize */
	HSF_RELAY_ONHOOK_PHONETOLINE_CALLID   = 4,
	HSF_RELAY_ONHOOK_PHONETOLINE_NOCALLID = 5,
	HSF_RELAY_ONHOOK_PHONEOFFLINE_CALLID  = 6,	/* listen without seizing */
	HSF_RELAY_ONHOOK_PHONEOFFLINE_NOCALLID = 7,
	HSF_RELAY_OFFHOOK_PULSE_MAKE          = 8,
	HSF_RELAY_OFFHOOK_PULSE_BREAK         = 9,
	HSF_RELAY_OFFHOOK_PULSESETUP          = 10,
	HSF_RELAY_OFFHOOK_PULSECLEAR          = 11,
	HSF_RELAY_COUNT                       = 12,
};

/* SMART_RELAYS, hsf.cty Profile\0000.  Per-country profiles may differ; this is
 * the base profile and is the one to check a capture against first. */
extern const uint16_t hsf_smart_relays[HSF_RELAY_COUNT];

/* Load one assembled body.  wvalue is 0xFF01 for every script but 8. */
/*
 * CD2_RESET and CD2_WAKEONRING are named in Conexant's own header
 * (modules/imported/include/usbhalos.h:551, CD2REQUESTTYPE) and the shipped
 * driver issues NEITHER: every control transfer in hsfusbcd2-i386.O is
 * request 0, 1, 2, 3 or 4.  The one place a reset looked like it might live --
 * hsfusbcd2210_ calling hsfusbcd2120_(5) on the path where the notify pipe is
 * unarmed -- is not one: hsfusbcd2120_ tail-jumps to OsSleep, so that 5 is
 * five milliseconds.
 *
 * So the framing is unknown: direction, wValue and wIndex are all guesses, and
 * the shape here is the one every other OUT-direction vendor request in this
 * part uses (device recipient, no data stage).  Sweep rather than assume.
 *
 * The reason to care is the workflow.  The CD2 bootloader answers EP0 for only
 * about three seconds after it enumerates, so every firmware experiment needs a
 * physical replug timed by hand.  A working CD2_RESET would replace that.  Note
 * that libusb_reset_device() is NOT an alternative and has been tried: the
 * darwin backend times out re-enumerating, libusb 1.0.29 then crashes on the
 * stale handle, and the device leaves the bus entirely.
 *
 * Both invalidate the handle by construction if they do anything at all --
 * close and re-open afterwards.  Worst case is the replug the workflow already
 * requires.
 */
/*
 * The notification pipe and the data rings are armed together by
 * hsf_fxo_start(), which is NOT the driver's order: the notify pipe is up from
 * attach, while the bulk rings are only primed at stream open -- after
 * hsfusbcd2241_'s script 1 query and hsfusbcd2247_'s script 8.  Sending script
 * 1 into rings that are already running collapses receive.  Call
 * hsf_fxo_defer_rx() before hsf_fxo_start() to hold the RX ring back, then
 * hsf_fxo_arm_rx() once the stream-open scripts have run.
 */
void hsf_fxo_defer_rx(struct hsf_dev *d, bool defer);
int  hsf_fxo_arm_rx(struct hsf_dev *d);

int hsf_fxo_reset(struct hsf_dev *d, unsigned rt, uint16_t wvalue, uint16_t windex);
int hsf_fxo_wake_on_ring(struct hsf_dev *d, unsigned rt, uint16_t wvalue, uint16_t windex);

/* rt selector for the two above.  CD2_CONTROL_SCRIPT is an INTERFACE-recipient
 * request in this part while CD2_UPLOAD_FIRMWARE is a DEVICE-recipient one, so
 * the recipient is not a constant here and cannot be assumed for an opcode
 * nobody sends. */
enum hsf_reset_rt {
	HSF_RT_DEV_OUT = 0,
	HSF_RT_IF_OUT  = 1,
	HSF_RT_DEV_IN  = 2,
	HSF_RT_IF_IN   = 3,
};

int hsf_fxo_script_load(struct hsf_dev *d, uint16_t wvalue,
			const uint8_t *body, size_t len);
int hsf_fxo_script_delete(struct hsf_dev *d);

/*
 * Send the script with the given id from hsf_scripts.h, patching in up to
 * npatch bytes first.  Returns -EINVAL for an id with no template.
 */
int hsf_fxo_script_run(struct hsf_dev *d, unsigned id,
		       const uint8_t *patch, size_t npatch);

/* As hsf_fxo_script_run, but with an explicit wIndex.  The vendor uses wIndex
 * 3 on script 8 as a delete, and 1 everywhere else. */
int hsf_fxo_script_run_index(struct hsf_dev *d, unsigned id, uint16_t windex,
			     const uint8_t *patch, size_t npatch);

/* CLEAR_FEATURE(ENDPOINT_HALT) on the interrupt-IN endpoint 0x82. */
int hsf_fxo_clear_notify_halt(struct hsf_dev *d);

/* HSF_SCRIPT_SESSION_A then _B.  _A is retried only after a 1400 ms timeout. */
int hsf_fxo_script_start_codec(struct hsf_dev *d);
int hsf_fxo_script_set_hook(struct hsf_dev *d, bool off_hook);

#endif /* HSF_FXO_H */
