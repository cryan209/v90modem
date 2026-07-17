/* 
 * Copyright (C) 2021 Aon plc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA 
 */

#include <unistd.h>
#include <stdbool.h>
#include <time.h>
#include <stdio.h>
#include <signal.h>
#include <errno.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <string.h>

#include <pjsua-lib/pjsua.h>

#define SIGNATURE PJMEDIA_SIG_CLASS_PORT_AUD('D','M')

struct dmodem {
	pjmedia_port base;
	pj_timestamp timestamp;
	pj_sock_t sock;
};

static struct dmodem port;
static bool destroying = false;
static pj_pool_t *pool;

static void error_exit(const char *title, pj_status_t status) {
	pjsua_perror(__FILE__, title, status);
	if (!destroying) {
		destroying = true;
		pjsua_destroy();
		exit(1);
	}
}

static FILE *dm_tap_tx, *dm_tap_rx;

/* Net(8000) -> DSP(9600) rational 6/5 resampler.
 *
 * V.90 puts the modem detector's whole burden on exact 8 kHz symbol timing
 * and on near-Nyquist structure: Sd = {+W,+0,+W,-W,-0,-W} has most of its
 * energy at 4 kHz (the 8 kHz Nyquist).  Two naive resamplers both fail the
 * SmartLink Sd detector:
 *   - zero-order hold keeps the 4 kHz line but its floor() phase injects up
 *     to ~104 us of jitter onto a 125 us symbol, so the recovered pattern
 *     comes out as +W,0,+W,0,-W,-W (corrupted);
 *   - pjmedia's polyphase keeps timing but its ~3.4 kHz voice cutoff nukes
 *     the 4 kHz line to -53 dB.
 * A windowed-sinc polyphase interpolator with the cutoff placed right at
 * 4 kHz preserves both.  fc = 3950 Hz, 12 taps, six output phases (k*5 mod
 * 6).  A one-frame pipeline delay supplies the "future" samples the FIR
 * needs across the 20 ms frame boundary without edge glitches. */
#define RS_TAPS   12
#define RS_HALF   5           /* taps span offsets [-5 .. +6] */
#define RS_PHASES 6
static float rs_ker[RS_PHASES][RS_TAPS];
static int   rs_ker_ready;

static void rs_build_kernel(void) {
	const double fs_in = 8000.0, fc = 3950.0;
	const double wc = fc/(fs_in/2.0);   /* normalised cutoff (~0.9875) */
	int p, t;
	if (rs_ker_ready) return;
	for (p = 0; p < RS_PHASES; p++) {
		double f = (double)p/(double)RS_PHASES;   /* fractional output pos */
		double sum = 0.0;
		for (t = 0; t < RS_TAPS; t++) {
			double x = (double)(t - RS_HALF) - f;  /* input samples from output pt */
			double s, w, y = wc*x;
			if (fabs(y) < 1e-9) s = wc;
			else s = wc*sin(M_PI*y)/(M_PI*y);
			w = 0.5 - 0.5*cos(2.0*M_PI*t/(RS_TAPS-1));  /* Hann */
			rs_ker[p][t] = (float)(s*w);
			sum += rs_ker[p][t];
		}
		for (t = 0; t < RS_TAPS; t++)
			rs_ker[p][t] /= (float)sum;   /* unity DC gain */
	}
	rs_ker_ready = 1;
}

static pj_status_t dmodem_put_frame(pjmedia_port *this_port, pjmedia_frame *frame) {
	struct dmodem *sm = (struct dmodem *)this_port;
	/* pipeline: hist_tail[RS_TAPS] = tail of frame N-2, prev[] = frame N-1 */
	static pj_int16_t hist[RS_TAPS];
	static pj_int16_t prev[1024];
	static int prev_n = -1;      /* -1 until first frame seen */
	int len;

	if (frame->type == PJMEDIA_FRAME_TYPE_AUDIO) {
		const pj_int16_t *in = (const pj_int16_t *)frame->buf;
		int in_n = (int)(frame->size / 2);
		pj_int16_t out[1152];
		pj_int16_t work[RS_TAPS + 1024 + RS_TAPS];
		int out_n, k, i;

		rs_build_kernel();
		if (in_n > 1024)
			return PJ_ETOOBIG;

		if (prev_n < 0) {
			/* Prime the pipeline: emit one frame of silence (20 ms latency). */
			out_n = in_n * 6 / 5;
			memset(out, 0, (size_t)out_n * 2);
			memcpy(prev, in, (size_t)in_n * 2);
			memset(hist, 0, sizeof(hist));
			prev_n = in_n;
		} else {
			/* Render prev[] with hist before it and this frame's head after. */
			out_n = prev_n * 6 / 5;
			for (i = 0; i < RS_TAPS; i++)
				work[i] = hist[i];
			for (i = 0; i < prev_n; i++)
				work[RS_TAPS + i] = prev[i];
			for (i = 0; i < RS_TAPS; i++)
				work[RS_TAPS + prev_n + i] = (i < in_n) ? in[i] : prev[prev_n-1];
			for (k = 0; k < out_n; k++) {
				int num = k * 5;
				int ic = num / 6;           /* integer input index in prev */
				int ph = num % 6;           /* output phase */
				const float *h = rs_ker[ph];
				int base = RS_TAPS + ic - RS_HALF;   /* work index of tap 0 */
				float acc = 0.0f;
				int t;
				for (t = 0; t < RS_TAPS; t++)
					acc += (float)work[base + t] * h[t];
				if (acc > 32767.0f) acc = 32767.0f;
				else if (acc < -32768.0f) acc = -32768.0f;
				out[k] = (pj_int16_t)(acc >= 0 ? acc + 0.5f : acc - 0.5f);
			}
			/* advance pipeline */
			for (i = 0; i < RS_TAPS; i++)
				hist[i] = prev[prev_n - RS_TAPS + i];
			memcpy(prev, in, (size_t)in_n * 2);
			prev_n = in_n;
		}
		if (!dm_tap_rx) dm_tap_rx = fopen("/tmp/dm_to_dsp.raw","wb");
		if (dm_tap_rx) { fwrite(out,2,out_n,dm_tap_rx); fflush(dm_tap_rx); }
		if ((len=write(sm->sock, out, out_n*2)) != out_n*2) {
			error_exit("error writing frame",0);
		}
	}

	return PJ_SUCCESS;
}

/* DSP 9600 -> net 8000: the upstream is V.34-modulated, band-limited
 * well below 3.4 kHz, so simple linear interpolation is adequate. */
static pj_status_t dmodem_get_frame(pjmedia_port *this_port, pjmedia_frame *frame) {
	struct dmodem *sm = (struct dmodem *)this_port;
	pj_int16_t raw[1152];
	pj_int16_t *out;
	int out_n, in_n, k;
	frame->size = PJMEDIA_PIA_AVG_FSZ(&this_port->info); // MAX? what is

	int len;
	out_n = (int)(frame->size / 2);
	in_n = out_n * 6 / 5;
	if (in_n > (int)PJ_ARRAY_SIZE(raw))
		return PJ_ETOOBIG;
	if ((len=read(sm->sock, raw, in_n*2)) != in_n*2) {
		error_exit("error reading frame",0);
	}
	out = (pj_int16_t *)frame->buf;
	for (k = 0; k < out_n; k++) {
		int num = k * 6;
		int i = num / 5;
		int frac = num % 5;          /* source position k*1.2 = i + frac/5 */
		int a = raw[i];
		int b = (i + 1 < in_n) ? raw[i + 1] : raw[i];
		out[k] = (pj_int16_t)((a * (5 - frac) + b * frac) / 5);
	}
	len = out_n * 2;

	if (!dm_tap_tx) dm_tap_tx = fopen("/tmp/dm_from_dsp.raw","wb");
	if (dm_tap_tx) { fwrite(frame->buf,1,(size_t)len,dm_tap_tx); fflush(dm_tap_tx); }
	frame->timestamp.u64 = sm->timestamp.u64;
	frame->type = PJMEDIA_FRAME_TYPE_AUDIO;
	sm->timestamp.u64 += PJMEDIA_PIA_SPF(&this_port->info);

	return PJ_SUCCESS;
}

static pj_status_t dmodem_on_destroy(pjmedia_port *this_port) {
	printf("destroy\n");
	exit(-1);
}

/* Callback called by the library when call's state has changed */
static void on_call_state(pjsua_call_id call_id, pjsip_event *e) {
	pjsua_call_info ci;

	PJ_UNUSED_ARG(e);

	pjsua_call_get_info(call_id, &ci);
	PJ_LOG(3,(__FILE__, "Call %d state=%.*s", call_id,
				(int)ci.state_text.slen,
				ci.state_text.ptr));

	if (ci.state == PJSIP_INV_STATE_DISCONNECTED) {
		close(port.sock);
		if (!destroying) {
			destroying = true;
			pjsua_destroy();
			exit(0);
		}
	}
}

/* Callback called by the library when call's media state has changed */
static void on_call_media_state(pjsua_call_id call_id) {
	pjsua_call_info ci;
	pjsua_conf_port_id port_id;
	static int done=0;

	pjsua_call_get_info(call_id, &ci);

//	printf("media_status %d media_cnt %d ci.conf_slot %d aud.conf_slot %d\n",ci.media_status,ci.media_cnt,ci.conf_slot,ci.media[0].stream.aud.conf_slot);
	if (ci.media_status == PJSUA_CALL_MEDIA_ACTIVE) {
		if (!done) {
			/* The port itself now converts 8000<->9600 (ZOH down,
			   linear up); it faces the bridge at 8000 Hz directly. */
			pjsua_conf_add_port(pool, &port.base, &port_id);
			pjsua_conf_connect(ci.conf_slot, port_id);
			pjsua_conf_connect(port_id, ci.conf_slot);
			done = 1;
		}
	} else {
		done = 0;
	}
}


int main(int argc, char *argv[]) {
	pjsua_acc_id acc_id;
	pj_status_t status;

	if (argc != 3) {
		return -1;
	}

	signal(SIGPIPE,SIG_IGN);

	char *dialstr = argv[1];

	char *sip_user = getenv("SIP_LOGIN");
	if (!sip_user) {
		return -1;
	}
	char *sip_domain = strchr(sip_user,'@');
	if (!sip_domain) {
		return -1;
	}
	*sip_domain++ = '\0';
	char *sip_pass = strchr(sip_user,':');
	if (!sip_pass) {
		return -1;
	}
	*sip_pass++ = '\0';

	status = pjsua_create();
	if (status != PJ_SUCCESS) error_exit("Error in pjsua_create()", status);

	/* Init pjsua */
	{
		pjsua_config cfg;
		pjsua_logging_config log_cfg;
		pjsua_media_config med_cfg;

		pjsua_config_default(&cfg);
		cfg.cb.on_call_media_state = &on_call_media_state;
		cfg.cb.on_call_state = &on_call_state;

		pjsua_logging_config_default(&log_cfg);
		log_cfg.console_level = 4;

		pjsua_media_config_default(&med_cfg);
		med_cfg.no_vad = true;
		med_cfg.ec_tail_len = 0;
		/* Modem waveforms cannot tolerate the adaptive jitter buffer
		 * periodically discarding samples to chase a voice-oriented latency
		 * target.  The SIP hop is on the local LAN, so use a small fixed
		 * three-packet playout buffer and never discard a received frame. */
		med_cfg.jb_init = 60;
		med_cfg.jb_min_pre = 60;
		med_cfg.jb_max_pre = 60;
		med_cfg.jb_max = 200;
		med_cfg.jb_discard_algo = PJMEDIA_JB_DISCARD_NONE;
		med_cfg.clock_rate = 8000;
		med_cfg.snd_clock_rate = 8000;
		med_cfg.audio_frame_ptime = 20;

		status = pjsua_init(&cfg, &log_cfg, &med_cfg);
		if (status != PJ_SUCCESS) error_exit("Error in pjsua_init()", status);
	}

	pjsua_set_ec(0,0); // maybe?
	pjsua_set_null_snd_dev();
	
	/* g711 only */
	pjsua_codec_info codecs[32];
	unsigned count = sizeof(codecs)/sizeof(*codecs);
	pjsua_enum_codecs(codecs,&count);
	for (int i=0; i<count; i++) {
		int pri = 0;
		if (pj_strcmp2(&codecs[i].codec_id,"PCMU/8000/1") == 0) {
			/* u-law preferred: the SmartLink V.90 datapump is
			   hardwired for u-law PCM tables (US-market blob). */
			pri = 2;
		} else if (pj_strcmp2(&codecs[i].codec_id,"PCMA/8000/1") == 0) {
			pri = 0;
		}
		pjsua_codec_set_priority(&codecs[i].codec_id, pri);
//		printf("codec: %s %d\n",pj_strbuf(&codecs[i].codec_id),pri);
	}

	/* Add UDP transport. */
	{
		pjsua_transport_config cfg;

		pjsua_transport_config_default(&cfg);
		cfg.port = 5060;
		status = pjsua_transport_create(PJSIP_TRANSPORT_UDP, &cfg, NULL);
		if (status != PJ_SUCCESS) error_exit("Error creating transport", status);
	}

	pj_caching_pool cp;
	pj_caching_pool_init(&cp, NULL, 1024*1024);
	pool = pj_pool_create(&cp.factory, "pool1", 4000, 4000, NULL);

	pj_str_t name = pj_str("dmodem");
	
	memset(&port,0,sizeof(port));
	port.sock = atoi(argv[2]); // inherited from parent
	pjmedia_port_info_init(&port.base.info, &name, SIGNATURE, 8000, 1, 16, 160);
	port.base.put_frame = dmodem_put_frame;
	port.base.get_frame = dmodem_get_frame;
	port.base.on_destroy = dmodem_on_destroy;

	char buf[384];
	memset(buf,0,sizeof(buf));
	write(port.sock, buf, sizeof(buf));

	/* Initialization is done, now start pjsua */
	status = pjsua_start();
	if (status != PJ_SUCCESS) error_exit("Error starting pjsua", status);

	{
		pjsua_acc_config cfg;
		pjsua_acc_config_default(&cfg);
		snprintf(buf,sizeof(buf),"sip:%s@%s",sip_user,sip_domain);
		pj_strdup2(pool,&cfg.id,buf);
		snprintf(buf,sizeof(buf),"sip:%s",sip_domain);
		pj_strdup2(pool,&cfg.reg_uri,buf);
		cfg.register_on_acc_add = false;
		cfg.cred_count = 1;
		cfg.cred_info[0].realm = pj_str("*");
		cfg.cred_info[0].scheme = pj_str("digest");
		cfg.cred_info[0].username = pj_str(sip_user);
		cfg.cred_info[0].data_type = PJSIP_CRED_DATA_PLAIN_PASSWD;
		cfg.cred_info[0].data = pj_str(sip_pass);

		status = pjsua_acc_add(&cfg, PJ_TRUE, &acc_id);
		if (status != PJ_SUCCESS) error_exit("Error adding account", status);
	}

	snprintf(buf,sizeof(buf),"sip:%s@%s",dialstr,sip_domain);
	printf("calling %s\n",buf);
	pj_str_t uri = pj_str(buf);
	
	pjsua_call_id callid;
	status = pjsua_call_make_call(acc_id, &uri, 0, NULL, NULL, &callid);
	if (status != PJ_SUCCESS) error_exit("Error making call", status);

	struct timespec ts = {100, 0};
	while(1) {
		nanosleep(&ts,NULL);
	}

	return 0;
}
