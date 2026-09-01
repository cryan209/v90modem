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
 * (modules/osusb.c, modules/cnxthwusb_common.c, modules/imported/include/*.h).
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
 * wLength, little-endian, followed by wLength bytes.  The notification CODES are
 * not known -- the CDC PSTN ones (RING_DETECT 0x09, AUX_JACK_HOOK_STATE 0x08)
 * are the obvious guess but are unconfirmed on this device.
 */
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
 * store).  Only ids 1-10 and 12 are ever enqueued by any of the 19 call sites.
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

	/* hsfusbcd2187_ (.text 0x32f0) dispatches 21 signal ids onto it, and sends
	 * it with wIndex 3 -- delete -- to stop one.  Tone/cadence generation. */
	HSF_SCRIPT_SIGNAL      = 8,

	/* hsfusbcd2220_ (.text 0x6910) patches three bytes: two context values
	 * divided by 1000 (so milliseconds) and one literal.  Pulse dialling. */
	HSF_SCRIPT_PULSE       = 7,
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
int hsf_fxo_script_load(struct hsf_dev *d, uint16_t wvalue,
			const uint8_t *body, size_t len);
int hsf_fxo_script_delete(struct hsf_dev *d);

/*
 * Send the script with the given id from hsf_scripts.h, patching in up to
 * npatch bytes first.  Returns -EINVAL for an id with no template.
 */
int hsf_fxo_script_run(struct hsf_dev *d, unsigned id,
		       const uint8_t *patch, size_t npatch);

/* HSF_SCRIPT_SESSION_A, _B, _A -- the hsfusbcd2165_ order.  Unverified. */
int hsf_fxo_script_start_codec(struct hsf_dev *d);
int hsf_fxo_script_set_hook(struct hsf_dev *d, bool off_hook);

#endif /* HSF_FXO_H */
