/*
 * SpanDSP - a series of DSP components for telephony
 *
 * v34_local.h - ITU V.34 modem
 *
 * Written by Steve Underwood <steveu@coppice.org>
 *
 * Copyright (C) 2009 Steve Underwood
 *
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License version 2.1,
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

void log_info0(logging_state_t *log, bool tx, const v34_capabilities_t *cap, int info0_acknowledgement);
void log_info1c(logging_state_t *log, bool tx, const info1c_t *info1c);
void log_info1a(logging_state_t *log, bool tx, const info1a_t *info1a);
void log_infoh(logging_state_t *log, bool tx, const infoh_t *infoh);

void log_mp(logging_state_t *log, bool tx, const mp_t *mp);
void log_mph(logging_state_t *log, bool tx, const mph_t *mph);

int v34_rx_restart(v34_state_t *s, int baud_rate, int bit_rate, int high_carrier);
void v34_rx_set_primary_channel(v34_state_t *s, int baud_rate, int high_carrier);
void v34_set_working_parameters(v34_parameters_t *s, int baud_rate, int bit_rate, int expanded);

/* V.34 11.4: the measured receive SNR over the Phase-4 TRN segment, and the
   largest Table 16 rate index (2400*n bit/s) it will carry.  Returns 0 when
   no usable measurement exists, in which case the caller must not constrain
   the rate on its account. */
int v34_phase4_trn_measured_rate_n(v34_state_t *s, float *snr_db);

/*- End of file ------------------------------------------------------------*/
