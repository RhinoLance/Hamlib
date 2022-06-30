/*
 *  Hamlib Kenwood backend - Description of TM-V71(A) and variations (tmv71)
 *  Adapted from tmv71(G) code
 * 		Copyright (c) 2011 by Charles Suprin
 *  	Copyright (c) 2018 Mikael Nousiainen
 *		Copyright (c) 2022 Lance Conry
 *
 * 	FEATURES
 * 	A limiting feature of the TM-V71 and tmv71 is that when setting the
 *  frequency, it must be within the currently selected VFO's frequency range.
 *  e.g. If the current frequency is 146.50, it may only be changed to a value
 *  between 144.000 and 148.000.  To change to 440.000, the operator must 
 *  first change the frequency band of the VFO to UHF before the new 
 *  frequency can be set.
 * 
 *  This rig model addresses the above limitation by using memory channels
 *  rather than the VFO modes.  It defines two channels, 998 and 999, which it
 *  uses in place of VFOA and VFOB respectively.
 *  
 *	 LICENSE
 *   This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 *
 *   This library is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public
 *   License along with this library; if not, write to the Free Software
 *   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdlib.h>
#include <math.h>
#include <ctype.h>

#include "hamlib/rig.h"
#include "kenwood.h"
#include "th.h"
#include "tones.h"
#include "num_stdio.h"
#include "misc.h"

static int tmv71_open(RIG *rig);
static int tmv71_do_get_freq(RIG *rig, vfo_t vfo, freq_t *freq);
static int tmv71_do_set_freq(RIG *rig, vfo_t vfo, freq_t freq);
static int tmv71_get_freq(RIG *rig, vfo_t vfo, freq_t *freq);
static int tmv71_set_freq(RIG *rig, vfo_t vfo, freq_t freq);
static int tmv71_get_split_freq(RIG *rig, vfo_t vfo, freq_t *freq);
static int tmv71_set_split_freq(RIG *rig, vfo_t vfo, freq_t freq);
static int tmv71_set_vfo (RIG *rig, vfo_t vfo);
static int tmv71_get_vfo(RIG *rig, vfo_t *vfo);
static int tmv71_set_split_vfo (RIG *rig, vfo_t vfo, split_t split, vfo_t txvfo);
static int tmv71_get_split_vfo (RIG *rig, vfo_t vfo, split_t *split, vfo_t *txvfo);
static int tmv71_set_ts(RIG *rig, vfo_t vfo, shortfreq_t ts);
static int tmv71_get_ts(RIG *rig, vfo_t vfo, shortfreq_t *ts);
static int tmv71_set_ctcss_tone(RIG *rig, vfo_t vfo, tone_t tone);
static int tmv71_get_ctcss_tone(RIG *rig, vfo_t vfo, tone_t *tone);
static int tmv71_set_ctcss_sql(RIG *rig, vfo_t vfo, tone_t tone);
static int tmv71_get_ctcss_sql(RIG *rig, vfo_t vfo, tone_t *tone);
static int tmv71_set_mode(RIG *rig, vfo_t vfo, rmode_t mode, pbwidth_t width);
static int tmv71_get_mode(RIG *rig, vfo_t vfo, rmode_t *mode, pbwidth_t *width);
static int tmv71_get_mem(RIG *rig, vfo_t vfo, int *ch);
static int tmv71_set_mem(RIG *rig, vfo_t vfo, int ch);
static int tmv71_set_dcs_sql(RIG * rig, vfo_t vfo, tone_t code);
static int tmv71_get_dcs_sql(RIG * rig, vfo_t vfo, tone_t *code);
static int tmv71_set_channel(RIG *rig, vfo_t vfo, const channel_t *chan);
static int tmv71_get_channel(RIG *rig, vfo_t vfo, channel_t *chan, int read_only);
static int tmv71_set_ptt(RIG *rig, vfo_t vfo, ptt_t ptt);
static int tmv71_get_dcd(RIG *rig, vfo_t vfo, dcd_t *dcd);

/*
static int tmv71_set_rptr_shift(RIG *rig, vfo_t vfo, rptr_shift_t shift);
static int tmv71_get_rptr_shift(RIG *rig, vfo_t vfo, rptr_shift_t* shift);
static int tmv71_set_rptr_offs(RIG *rig, vfo_t vfo, shortfreq_t offset);
static int tmv71_get_rptr_offs(RIG *rig, vfo_t vfo, shortfreq_t* offset);
static int tmv71_vfo_op(RIG *rig, vfo_t vfo, vfo_op_t op);
static int tmv71_set_level(RIG *rig, vfo_t vfo, setting_t level, value_t val);
static int tmv71_get_level(RIG *rig, vfo_t vfo, setting_t level, value_t *val);
static int tmv71_get_func(RIG *rig, vfo_t vfo, setting_t func, int *status);
static int tmv71_set_func(RIG *rig, vfo_t vfo, setting_t func, int status);
static int tmv71_get_parm(RIG *rig, setting_t parm, value_t *val);
static int tmv71_set_parm(RIG *rig, setting_t parm, value_t val);
static int tmv71_get_ext_level(RIG *rig, vfo_t vfo, token_t token, value_t *val);
static int tmv71_set_ext_level(RIG *rig, vfo_t vfo, token_t token, value_t val);
*/

static int rig_pull_bc(RIG *rig, vfo_t *ctrl, vfo_t *ptt);

// Rig capabilities

#define tmv71_MODE_LIST (RIG_MODE_FM | RIG_MODE_FMN | RIG_MODE_AM)
#define tmv71_MODE_LIST_FM (RIG_MODE_FM | RIG_MODE_FMN)
#define tmv71_MODE_LIST_TX (RIG_MODE_FM | RIG_MODE_FMN)

#define tmv71_FUNC_GET (RIG_FUNC_TSQL | \
						RIG_FUNC_TONE | \
						RIG_FUNC_REV |  \
						RIG_FUNC_LOCK | \
						RIG_FUNC_ARO |  \
						RIG_FUNC_AIP |  \
						RIG_FUNC_RESUME)
#define tmv71_FUNC_SET (RIG_FUNC_TSQL |   \
						RIG_FUNC_TONE |   \
						RIG_FUNC_TBURST | \
						RIG_FUNC_REV |    \
						RIG_FUNC_LOCK |   \
						RIG_FUNC_ARO |    \
						RIG_FUNC_AIP |    \
						RIG_FUNC_RESUME)

#define tmv71_LEVEL_ALL (RIG_LEVEL_SQL | \
						 RIG_LEVEL_RFPOWER)

#define tmv71_PARMS (RIG_PARM_BACKLIGHT | \
					 RIG_PARM_BEEP |      \
					 RIG_PARM_APO)

#define tmv71_VFO_OP (RIG_OP_UP | RIG_OP_DOWN)

#define tmv71_CHANNEL_CAPS \
	TH_CHANNEL_CAPS,       \
		.flags = 1,        \
		.dcs_code = 1,     \
		.dcs_sql = 1,

#define tmv71_CHANNEL_CAPS_WO_LO \
	TH_CHANNEL_CAPS,             \
		.dcs_code = 1,           \
		.dcs_sql = 1,

#define TOKEN_BACKEND(t) (t)

#define TOK_LEVEL_EXT_DATA_BAND TOKEN_BACKEND(100)

// TM-D710 protocol definitions

#define tmv71_BAND_A 0
#define tmv71_BAND_B 1
#define tmv71_BAND_UNKNOWN 2
#define tmv71_BAND_A_CHANNEL 998
#define tmv71_BAND_B_CHANNEL 999

#define tmv71_BAND_MODE_VFO 0
#define tmv71_BAND_MODE_MEMORY 1
#define tmv71_BAND_MODE_CALL 2
#define tmv71_BAND_MODE_WX 3

#define tmv71_RF_POWER_MIN 0
#define tmv71_RF_POWER_MAX 2

#define tmv71_SQL_MIN 0
#define tmv71_SQL_MAX 0x1F

#define tmv71_MODE_FM 0
#define tmv71_MODE_NFM 1
#define tmv71_MODE_AM 2

#define tmv71_MODE_FM_PASSBAND 15000
#define tmv71_MODE_NFM_PASSBAND 5000
#define tmv71_MODE_AM_PASSBAND 4000

static struct kenwood_priv_caps tmv71_priv_caps = {
	.cmdtrm = EOM_TH, /* Command termination character */
	//.mode_table = tmv71_mode_table,
};

const struct rig_caps tmv71_caps = {
	.rig_model = RIG_MODEL_TMV71,
	.model_name = "TM-V71(A)",
	.mfg_name = "Kenwood",
	.version = BACKEND_VER ".1",
	.copyright = "LGPL",
	.status = RIG_STATUS_STABLE,
	.rig_type = RIG_TYPE_MOBILE | RIG_FLAG_APRS | RIG_FLAG_TNC,
	.ptt_type = RIG_PTT_RIG,
	.dcd_type = RIG_DCD_RIG,
	.port_type = RIG_PORT_SERIAL,
	.serial_rate_min = 9600,
	.serial_rate_max = 57600,
	.serial_data_bits = 8,
	.serial_stop_bits = 1,
	.serial_parity = RIG_PARITY_NONE,
	.serial_handshake = RIG_HANDSHAKE_NONE,
	.write_delay = 0,
	.post_write_delay = 0,
	.timeout = 1000,
	.retry = 3,

	.has_get_func = tmv71_FUNC_GET,
	.has_set_func = tmv71_FUNC_SET,
	.has_get_level = tmv71_LEVEL_ALL,
	.has_set_level = RIG_LEVEL_SET(tmv71_LEVEL_ALL),
	.has_get_parm = tmv71_PARMS,
	.has_set_parm = tmv71_PARMS,
	.level_gran = {},
	.parm_gran = {},
	.ctcss_list = kenwood42_ctcss_list,
	.dcs_list = common_dcs_list,
	.preamp = {
		RIG_DBLST_END,
	},
	.attenuator = {
		RIG_DBLST_END,
	},
	.max_rit = Hz(0),
	.max_xit = Hz(0),
	.max_ifshift = Hz(0),
	.vfo_ops = tmv71_VFO_OP,
	.scan_ops = RIG_SCAN_NONE,
	.targetable_vfo = RIG_TARGETABLE_FREQ,
	.transceive = RIG_TRN_OFF,
	.bank_qty = 0,
	.chan_desc_sz = 8,

	.chan_list = {
		{0, 199, RIG_MTYPE_MEM, {tmv71_CHANNEL_CAPS}},			/* normal MEM */
		{200, 219, RIG_MTYPE_EDGE, {tmv71_CHANNEL_CAPS}},		/* U/L MEM */
		{221, 222, RIG_MTYPE_CALL, {tmv71_CHANNEL_CAPS_WO_LO}}, /* Call 0/1 */
		RIG_CHAN_END,
	},
	/*
	 * TODO: Japan & TM-D700S, and Taiwan models
	 */
	.rx_range_list1 = {
		{MHz(118), MHz(470), tmv71_MODE_LIST, -1, -1, RIG_VFO_A | RIG_VFO_MEM},
		{MHz(136), MHz(174), tmv71_MODE_LIST_FM, -1, -1, RIG_VFO_A | RIG_VFO_B | RIG_VFO_MEM},
		{MHz(300), MHz(524), tmv71_MODE_LIST_FM, -1, -1, RIG_VFO_A | RIG_VFO_B | RIG_VFO_MEM},
		{MHz(800), MHz(1300), tmv71_MODE_LIST_FM, -1, -1, RIG_VFO_B | RIG_VFO_MEM},
		RIG_FRNG_END,
	}, /* rx range */
	.tx_range_list1 = {
		{MHz(144), MHz(146), tmv71_MODE_LIST_TX, W(5), W(50), RIG_VFO_A | RIG_VFO_B | RIG_VFO_MEM},
		{MHz(430), MHz(440), tmv71_MODE_LIST_TX, W(5), W(35), RIG_VFO_A | RIG_VFO_B | RIG_VFO_MEM},
		RIG_FRNG_END,
	}, /* tx range */

	.rx_range_list2 = {
		{MHz(118), MHz(470), tmv71_MODE_LIST, -1, -1, RIG_VFO_A | RIG_VFO_MEM},
		{MHz(136), MHz(174), tmv71_MODE_LIST_FM, -1, -1, RIG_VFO_A | RIG_VFO_B | RIG_VFO_MEM},
		{MHz(300), MHz(524), tmv71_MODE_LIST_FM, -1, -1, RIG_VFO_A | RIG_VFO_B | RIG_VFO_MEM},
		{MHz(800), MHz(1300), tmv71_MODE_LIST_FM, -1, -1, RIG_VFO_B | RIG_VFO_MEM}, /* TODO: cellular blocked */
		RIG_FRNG_END,
	}, /* rx range */
	.tx_range_list2 = {
		{MHz(144), MHz(148), tmv71_MODE_LIST_TX, W(5), W(50), RIG_VFO_A | RIG_VFO_B | RIG_VFO_MEM},
		{MHz(430), MHz(450), tmv71_MODE_LIST_TX, W(5), W(35), RIG_VFO_A | RIG_VFO_B | RIG_VFO_MEM},
		RIG_FRNG_END,
	}, /* tx range */

	.tuning_steps = {
		{tmv71_MODE_LIST, kHz(5)},
		{tmv71_MODE_LIST, kHz(6.25)},
		{tmv71_MODE_LIST, kHz(8.33)},
		{tmv71_MODE_LIST, kHz(10)},
		{tmv71_MODE_LIST, kHz(12.5)},
		{tmv71_MODE_LIST, kHz(15)},
		{tmv71_MODE_LIST, kHz(20)},
		{tmv71_MODE_LIST, kHz(25)},
		{tmv71_MODE_LIST, kHz(30)},
		{tmv71_MODE_LIST, kHz(50)},
		{tmv71_MODE_LIST, kHz(100)},
		RIG_TS_END,
	},
	/* mode/filter list, remember: order matters! */
	.filters = {
		{RIG_MODE_FM, kHz(15)},
		{RIG_MODE_FMN, kHz(5)},
		{RIG_MODE_AM, kHz(4)},
		RIG_FLT_END,
	},
	.priv = (void *)&tmv71_priv_caps,

	.rig_init = kenwood_init,
	.rig_open = tmv71_open,
	.rig_cleanup = kenwood_cleanup,
	.set_freq = tmv71_set_freq,
	.get_freq = tmv71_get_freq,
	.set_split_freq = tmv71_set_split_freq,
	.get_split_freq = tmv71_get_split_freq,
	.set_mode = tmv71_set_mode,
	.get_mode = tmv71_get_mode,
	.set_vfo = tmv71_set_vfo,
	.get_vfo = tmv71_get_vfo,
	.set_ts = tmv71_set_ts,
	.get_ts = tmv71_get_ts,
	.set_ctcss_tone = tmv71_set_ctcss_tone,
	.get_ctcss_tone = tmv71_get_ctcss_tone,
	.set_ctcss_sql = tmv71_set_ctcss_sql,
	.get_ctcss_sql = tmv71_get_ctcss_sql,
	.set_split_vfo = tmv71_set_split_vfo,
	.get_split_vfo = tmv71_get_split_vfo,
	.set_dcs_sql = tmv71_set_dcs_sql,
	.get_dcs_sql = tmv71_get_dcs_sql,
	.set_mem = tmv71_set_mem,
	.get_mem = tmv71_get_mem,
	.set_channel = tmv71_set_channel,
	.get_channel = tmv71_get_channel,

	/*
	.set_func = tmv71_set_func,
	.get_func = tmv71_get_func,
	.set_level = tmv71_set_level,
	.get_level = tmv71_get_level,
	.set_parm = tmv71_set_parm,
	.get_parm = tmv71_get_parm,
	//.get_info =  th_get_info,
	*/
	.get_dcd = tmv71_get_dcd,
	.set_ptt = tmv71_set_ptt,
	/*
	.vfo_op = tmv71_vfo_op,
	.scan   =  th_scan,
	.set_ext_level = tmv71_set_ext_level,
	.get_ext_level = tmv71_get_ext_level,

	.extlevels = tmv71_mem_ext_levels,


	.set_rptr_shift = tmv71_set_rptr_shift,
	.get_rptr_shift = tmv71_get_rptr_shift,
	.set_rptr_offs = tmv71_set_rptr_offs,
	.get_rptr_offs = tmv71_get_rptr_offs,
	*/

	.decode_event = th_decode_event,
};

/* enum of different tones */
enum tmv71_tone_type
{
	tx_tone,
	ctcss,
	dcs
};

/* enum of different tones */
enum tmv71_tx_rx
{
	tx,
	rx
};

struct
{
	int vfo_tx;
	int vfo_rx;
	int split_mode_active;
} tmv71_state;

/* structure for handling ME radio command */
struct tmv71_me
{
	int channel;	 // P1
	freq_t freq;	 // P2
	int step;		 // P3
	int shift;		 // P4
	int reverse;	 // P5
	int tone;		 // P6
	int ct;			 // P7
	int dcs;		 // P8
	int tone_freq;	 // P9
	int ct_freq;	 // P10
	int dcs_val;	 // P11
	int offset;		 // P12
	int mode;		 // P13
	freq_t tx_freq;	 // P14
	int tx_step; 	 // P15
	int lockout;	 // P16
};

/* structure for handling BC radio commands */
struct tmv71_bc
{
	int ctrl; // P1
	int ptt; // P2
};

/* structure for holding frequency and supporting step size */
struct tmv71_stepFreq
{
	long frequency;
	int step;
};

/* structure for holding frequency and supporting step size */
struct tmv71_toneDetail
{
	int tone_enabled;
	int tone_freq;
	int ctcss_enabled;
	int ctcss_freq;
	int dcs_enabled;
	int dcs_freq;
};

static int tmv71_open(RIG *rig)
{
	rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

	tmv71_state.vfo_tx = RIG_VFO_A;
	tmv71_state.vfo_tx = RIG_VFO_A;

	return 0;
}

static int tmv71_transform_band_to_vfo(int band){
	return band == tmv71_BAND_A ? RIG_VFO_A : RIG_VFO_B;
};

static int tmv71_transform_vfo_to_band(int vfo)
{
	if( vfo == RIG_VFO_A ) return tmv71_BAND_A;
	if (vfo == RIG_VFO_B) return tmv71_BAND_B;
	
	return tmv71_BAND_UNKNOWN;
};

int tmv71_transform_rptr_shift_from_hamlib(rptr_shift_t shift, int *tmv71_shift)
{
	switch (shift)
	{
	case RIG_RPT_SHIFT_NONE:
		*tmv71_shift = 0;
		break;
	case RIG_RPT_SHIFT_PLUS:
		*tmv71_shift = 1;
		break;
	case RIG_RPT_SHIFT_MINUS:
		*tmv71_shift = 2;
		break;
	default:
		rig_debug(RIG_DEBUG_ERR, "%s: Unexpected shift value '%d'\n", __func__, shift);
		return -RIG_EPROTO;
	}

	return RIG_OK;
}

int tmv71_transform_rptr_shift_to_hamlib(int radio_shift, rptr_shift_t *shift)
{
	switch (radio_shift)
	{
	case 0:
		*shift = RIG_RPT_SHIFT_NONE;
		break;
	case 1:
		*shift = RIG_RPT_SHIFT_PLUS;
		break;
	case 2:
		*shift = RIG_RPT_SHIFT_MINUS;
		break;
	default:
		rig_debug(RIG_DEBUG_ERR, "%s: Unexpected shift value '%d'\n", __func__, radio_shift);
		return -RIG_EPROTO;
	}

	return RIG_OK;
}

static int tmv71_trasform_mode_to_hamlib(int src_mode, rmode_t *target_mode, pbwidth_t *width)
{
	switch (src_mode)
	{
	case tmv71_MODE_FM:
		*target_mode = RIG_MODE_WFM;
		*width = tmv71_MODE_FM_PASSBAND;
		break;
	case tmv71_MODE_NFM:
		*target_mode = RIG_MODE_FM;
		*width = tmv71_MODE_NFM_PASSBAND;
		break;
	case tmv71_MODE_AM:
		*target_mode = RIG_MODE_AM;
		*width = tmv71_MODE_AM_PASSBAND;
		break;
	default:
		rig_debug(RIG_DEBUG_ERR, "%s: Illegal value from radio '%ld'\n", __func__, (long)src_mode);
		return -RIG_EINVAL;
	}

	return RIG_OK;
}

static int tmv71_trasform_mode_from_hamlib(rmode_t src_mode, int *target_mode)
{

	switch (src_mode)
	{
	case RIG_MODE_WFM:
		*target_mode = tmv71_MODE_FM;
		break;

	case RIG_MODE_FM:
	case RIG_MODE_FMN:
		*target_mode = tmv71_MODE_NFM;
		break;

	case RIG_MODE_AM:
		*target_mode = tmv71_MODE_AM;
		break;

	default:
		rig_debug(RIG_DEBUG_ERR, "%s: Illegal value from hamlib '%ld'\n", __func__, (long)src_mode);
		return -RIG_EINVAL;
	}

	return RIG_OK;
}

static int tmv71_vfo_to_channel(RIG *rig, vfo_t vfo, int *channel){
	rig_debug(RIG_DEBUG_TRACE, "%s: called with vfo: %d\n", __func__, vfo);

	rig_debug(RIG_DEBUG_TRACE, "%s: RIG_VFO_A: %d, RIG_VFO_B %d\n", __func__, RIG_VFO_A, RIG_VFO_B);

	vfo_t ctrl, ptt;
	int retval;

	switch( vfo ){
		case RIG_VFO_A:
			*channel = tmv71_BAND_A_CHANNEL;
			rig_debug(RIG_DEBUG_TRACE, "%s: set channel: %d from vfo: %d\n",
					  __func__, *channel, vfo);
			break;
		case RIG_VFO_B:
			*channel = tmv71_BAND_B_CHANNEL;
			rig_debug(RIG_DEBUG_TRACE, "%s: set channel: %d from vfo: %d\n",
					  __func__, *channel, vfo);
			break;
		default:
			
			retval = rig_pull_bc(rig, &ctrl, &ptt);

			if( retval != RIG_OK){
				*channel = tmv71_BAND_A_CHANNEL;
				rig_debug(RIG_DEBUG_WARN, "%s - Unable to resolve current VFO. \
					Falling back to RIG_VFO_A.\n", __func__);
			}
			else{
				int radio_vfo = ctrl;
				tmv71_vfo_to_channel(rig, radio_vfo, channel);
			}
			
	}
	
	return RIG_OK;
}

static struct tmv71_me tmv71_get_update_me(){
	struct tmv71_me me_new;
	me_new.freq = -1;
	me_new.step = -1;
	me_new.shift = -1;
	me_new.reverse = -1;
	me_new.tone = -1;
	me_new.ct = -1;
	me_new.dcs = -1;
	me_new.tone_freq = -1;
	me_new.ct_freq = -1;
	me_new.dcs_val = -1;
	me_new.offset = -1;
	me_new.mode = -1;
	me_new.tx_freq = -1;
	me_new.tx_step = -1;
	me_new.lockout = -1;

	return me_new;
}

/*
 *	Get the details of a memory channel
 */
int rig_pull_me(RIG * rig, int ch, struct tmv71_me *me_struct)
	{
		rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

		char cmdbuf[8];
		char buf[80];
		int retval;

		snprintf(cmdbuf, sizeof(cmdbuf), "ME %03d", ch);
		retval = kenwood_transaction(rig, cmdbuf, buf, sizeof(buf));
		if (retval != RIG_OK)
		{
			return retval;
		}

		retval = num_sscanf(buf, "ME %d,%" SCNfreq ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%" SCNfreq ",%d,%d",
			&me_struct->channel, &me_struct->freq,
			&me_struct->step, &me_struct->shift,
			&me_struct->reverse, &me_struct->tone,
			&me_struct->ct, &me_struct->dcs,
			&me_struct->tone_freq, &me_struct->ct_freq,
			&me_struct->dcs_val, &me_struct->offset,
			&me_struct->mode, &me_struct->tx_freq,
			&me_struct->tx_step, &me_struct->lockout);

		if (retval != 16)
		{
			rig_debug(RIG_DEBUG_ERR, "%s: Unexpected reply '%s'\n",
				 __func__, buf);
			return -RIG_ERJCTED;
		}

		return RIG_OK;
}

/*
 *	Set the details of a memory channel
 */
int rig_push_me(RIG *rig, struct tmv71_me *me_struct)
{
	char cmdbuf[80];
	char buf[80];

	rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

	snprintf(cmdbuf, sizeof(cmdbuf), "ME %03d,%010.0f,%1d,%1d,%1d,%1d,%1d,%1d,%02d,%02d,%03d,%08d,%1d,%010.0f,%1d,%1d",
			 me_struct->channel, me_struct->freq,
			 me_struct->step, me_struct->shift,
			 me_struct->reverse, me_struct->tone,
			 me_struct->ct, me_struct->dcs,
			 me_struct->tone_freq, me_struct->ct_freq,
			 me_struct->dcs_val, me_struct->offset,
			 me_struct->mode, me_struct->tx_freq,
			 me_struct->tx_step, me_struct->lockout);

	return kenwood_transaction(rig, cmdbuf, buf, sizeof(buf));
}

/*
 *	Get the band state of VFO/MEMORY
 */
int rig_pull_vm(RIG *rig, vfo_t vfo, int *mode)
{
	rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

	int band = tmv71_transform_vfo_to_band(vfo);

	char cmdbuf[80];
	char buf[80];

	snprintf(cmdbuf, sizeof(cmdbuf), "VM %1d", band);

	int retval;
	retval = kenwood_transaction(rig, cmdbuf, buf, sizeof(buf));
	if (retval != RIG_OK)
	{
		return retval;
	}

	retval = num_sscanf(buf, "VM %d,%d", &band, mode);

	if (retval != 2)
	{
		rig_debug(RIG_DEBUG_ERR, "%s: Unexpected reply '%s'\n", __func__, buf);
		return -RIG_ERJCTED;
	}

	return RIG_OK;
}

/*
 *	Set the band to VFO/MEMORY
 */
int rig_push_vm(RIG *rig, vfo_t vfo, int mode)
{
	rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

	int band = tmv71_transform_vfo_to_band(vfo);

	char cmdbuf[80];
	char buf[80];

	snprintf(cmdbuf, sizeof(cmdbuf), "VM %1d,%1d",
			 band, mode);

	return kenwood_transaction(rig, cmdbuf, buf, sizeof(buf));
}

/*
 *	Get the CTRL and PTT.
 */
int rig_pull_bc(RIG *rig, vfo_t *ctrl, vfo_t *ptt)
{
	rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

	char cmdbuf[80];
	char buf[80];

	snprintf(cmdbuf, sizeof(cmdbuf), "BC");

	int retval;
	retval = kenwood_transaction(rig, cmdbuf, buf, sizeof(buf));
	if (retval != RIG_OK)
	{
		rig_debug(RIG_DEBUG_TRACE, "%d: failed after kenwood_transaction\n", retval);
		return retval;
	}

	int band_ctrl, band_ptt;

	retval = num_sscanf(buf, "BC %d,%d", &band_ctrl, &band_ptt);

	*ctrl = tmv71_transform_band_to_vfo(band_ctrl);
	*ptt = tmv71_transform_band_to_vfo(band_ptt);

	if (retval != 2)
	{
		rig_debug(RIG_DEBUG_ERR, "%s: Unexpected reply '%s'\n", __func__, buf);
		return -RIG_ERJCTED;
	}

	return RIG_OK;
}

/*
 *	Set the CTRL and PTT.
 */
int rig_push_bc(RIG *rig, vfo_t ctrl, vfo_t ptt)
{
	int band_ctrl = tmv71_transform_vfo_to_band(ctrl);
	int band_ptt = tmv71_transform_vfo_to_band(ptt);
	

	rig_debug(RIG_DEBUG_TRACE, "%s - called to set VFOs(bands) CRTL to: %d(%d) and PTT to: %d(%d)\n",
			  __func__, ctrl, band_ctrl, ptt, band_ptt);

	char cmdbuf[80];
	char buf[80];

	snprintf(cmdbuf, sizeof(cmdbuf), "BC %1d,%1d",
			 band_ctrl, band_ptt);

	return kenwood_transaction(rig, cmdbuf, buf, sizeof(buf));
}

/*
 *	Get the current memory channel number set to the band
 */
int rig_pull_mr(RIG *rig, vfo_t vfo, int *channel)
{
	int band = tmv71_transform_vfo_to_band(vfo);

	rig_debug(RIG_DEBUG_TRACE, "%s - called for band %d\n", __func__, band);

	char cmdbuf[80];
	char buf[80];

	snprintf(cmdbuf, sizeof(cmdbuf), "MR %d", band);

	int retval;
	retval = kenwood_transaction(rig, cmdbuf, buf, sizeof(buf));
	if (retval != RIG_OK)
	{
		return retval;
	}

	rig_debug(RIG_DEBUG_TRACE, "%s - Successfully requested MR\n", __func__);
	
	retval = num_sscanf(buf, "MR %*d, %d",
						channel);

	if (retval != 1)
	{
		rig_debug(RIG_DEBUG_ERR, "%s: Unexpected reply '%s'\n", __func__, buf);
		return -RIG_ERJCTED;
	}

	return RIG_OK;
}

/*
 *	Set the band to a given memory channel
 */
int rig_push_mr(RIG *rig, vfo_t vfo, int channel)
{

	int band = tmv71_transform_vfo_to_band(vfo);

	rig_debug(RIG_DEBUG_TRACE, "%s - called for band: %d, channel %d\n", __func__, band, channel);

	char cmdbuf[80];
	char buf[80];

	snprintf(cmdbuf, sizeof(cmdbuf), "MR %d,%d",
			 band, channel);

	return kenwood_transaction(rig, cmdbuf, buf, sizeof(buf));
}

/*
*	Get the memory channel name
*/
int rig_pull_mn(RIG *rig, int channel, char *name)
{
	char cmdbuf[80];
	char buf[80];

	rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

	snprintf(cmdbuf, sizeof(cmdbuf), "MR %s", name);

	int retval;
	retval = kenwood_transaction(rig, cmdbuf, buf, sizeof(buf));
	if (retval != RIG_OK)
	{
		return retval;
	}

	retval = num_sscanf(buf, "MN %s", name);

	if (retval != 1)
	{
		rig_debug(RIG_DEBUG_ERR, "%s: Unexpected reply '%s'\n", __func__, buf);
		return -RIG_ERJCTED;
	}

	return RIG_OK;
}

/*
 *	Set the memory channel name
 */
int rig_push_mn(RIG *rig, int channel, char *name)
{
	char cmdbuf[80];
	char buf[80];

	rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

	snprintf(cmdbuf, sizeof(cmdbuf), "MN %d,%s",
			 channel, name);

	return kenwood_transaction(rig, cmdbuf, buf, sizeof(buf));
}

int rig_push_tx(RIG *rig)
{
	char cmdbuf[80];
	char buf[80];

	rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

	snprintf(cmdbuf, sizeof(cmdbuf), "TX");

	return kenwood_transaction(rig, cmdbuf, buf, sizeof(buf));
}

int rig_push_rx(RIG *rig)
{
	char cmdbuf[80];
	char buf[80];

	rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

	snprintf(cmdbuf, sizeof(cmdbuf), "RX");

	return kenwood_transaction(rig, cmdbuf, buf, sizeof(buf));
}

int rig_pull_by(RIG *rig, vfo_t vfo, dcd_t *dcd)
{
	char cmdbuf[80];
	char buf[80];

	rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

	snprintf(cmdbuf, sizeof(cmdbuf), "BY %d", vfo);

	int retval;
	retval = kenwood_transaction(rig, cmdbuf, buf, sizeof(buf));
	if (retval != RIG_OK)
	{
		return retval;
	}

	int dcdVal;
	retval = sscanf(buf, "BY %d", &dcdVal);
	
	switch (dcdVal)
	{
	case 0:
		*dcd = RIG_DCD_OFF;
		break;
	case 1:
		*dcd = RIG_DCD_ON;
		break;
	default:
		rig_debug(RIG_DEBUG_ERR, "%s: unexpected reply '%s', len=%ld\n", __func__, buf, (long)strlen(buf));
		return -RIG_ERJCTED;
	}

	return RIG_OK;
}

int tmv71_update_memory_channel(RIG *rig, int channel, struct tmv71_me *me_new)
{

	rig_debug(RIG_DEBUG_TRACE, "%s: called for channel %d\n",
			  __func__, channel);

	// Get the memory channel so that we have the current values
	struct tmv71_me me_current;
	int retval = rig_pull_me(rig, channel, &me_current);
	if (retval != RIG_OK)
	{
		return retval;
	}

	if( me_new->freq != -1 )		me_current.freq = me_new->freq;
	if (me_new->step != -1)			me_current.step = me_new->step;
	if (me_new->shift != -1)		me_current.shift = me_new->shift;
	if (me_new->reverse != -1)		me_current.reverse = me_new->reverse;
	if (me_new->tone != -1)			me_current.tone = me_new->tone;
	if (me_new->ct != -1)			me_current.ct = me_new->ct;
	if (me_new->dcs != -1)			me_current.dcs = me_new->dcs;
	if (me_new->tone_freq != -1)	me_current.tone_freq = me_new->tone_freq;
	if (me_new->ct_freq != -1)		me_current.ct_freq = me_new->ct_freq;
	if (me_new->dcs_val != -1)		me_current.dcs_val = me_new->dcs_val;
	if (me_new->offset != -1)		me_current.offset = me_new->offset;
	if (me_new->mode != -1)			me_current.mode = me_new->mode;
	if (me_new->tx_freq != -1)		me_current.tx_freq = me_new->tx_freq;
	if (me_new->tx_step != -1)		me_current.tx_step = me_new->tx_step;
	if (me_new->lockout != -1)		me_current.lockout = me_new->lockout;

	return rig_push_me(rig, &me_current);
}

/*
 * tmv71_resolve_vfo_name
 *
 * Common function for converting a vfo_t to a string.
 */
int tmv71_resolve_vfo_name(vfo_t vfo, char *name)
{
	switch(vfo){
		case RIG_VFO_A:
			strcpy(name, "RIG_VFO_A");
			break;
		case RIG_VFO_B:
			strcpy(name, "RIG_VFO_B");
			break;
		case RIG_VFO_CURR :
			strcpy(name, "RIG_VFO_CURR");
			break;
		default:
			strcpy(name, "UNRESOLVED_VFO");
	}

	return RIG_OK;
}

/*
 * tmv71_resolve_freq
 *
 * Common function for converting a frequency into the closes match which
 * the radio supports.
 */
struct tmv71_stepFreq tmv71_resolve_supported_freq(int freq)
{
	long freq5, freq625, resolvedFreq;
	int step;

	freq5 = round(freq / 5000) * 5000;
	freq625 = round(freq / 6250) * 6250;

	if (fabs(freq5 - freq) < fabs(freq625 - freq))
	{
		step = 0;
		resolvedFreq = freq5;
	}
	else
	{
		step = 1;
		resolvedFreq = freq625;
	}

	struct tmv71_stepFreq result;

	/* Step needs to be at least 10kHz on higher band, otherwise 5 kHz */
	result.step = resolvedFreq >= MHz(470) ? 4 : step;
	result.frequency = resolvedFreq >= MHz(470) ? (round(resolvedFreq / 10000) * 10000) : resolvedFreq;

	return result;
}

/**
 * \brief Provides a VFO for split operation
 * \param for_split_action If the Split VFO is required, 1, else 0
 * \param data  Data pointer to be passed to cfunc()
 *
 *  Depending on whether a Split VFO has been set, this method may either
 *  resolve to the provided VFO, or the already set Split VFO.
 * \internal
 *
 * \return always RIG_OK.
 */
int tmv71_resolve_vfo_for_split(RIG *rig, int for_split_action, vfo_t requested_vfo, vfo_t *resolved_vfo)
{

	char vfo_debug_name[14];
	tmv71_resolve_vfo_name(requested_vfo, vfo_debug_name);
	rig_debug(RIG_DEBUG_TRACE, "%s: called with split_action %d, requested_vfo %s\n",
			  __func__, for_split_action, vfo_debug_name);

	if (tmv71_state.split_mode_active == RIG_SPLIT_ON)
	{
		rig_debug(RIG_DEBUG_TRACE, "%s: satmode is enabled.  use tmv71_state.vfo_(t|r)x\n",
				  __func__);

		*resolved_vfo = (for_split_action)
							? tmv71_state.vfo_tx
							: tmv71_state.vfo_rx;
	}
	else{
		*resolved_vfo = requested_vfo;
	}

	tmv71_resolve_vfo_name(*resolved_vfo, vfo_debug_name);
	rig_debug(RIG_DEBUG_TRACE, "%s: resolved vfo to %s\n",
			  __func__, vfo_debug_name);

	return RIG_OK;
}

/*
 * tmv71_set_freq
 * Assumes rig!=NULL
 * Common function for getting the main and split frequency.
 */
int tmv71_do_set_freq(RIG *rig, vfo_t vfo, freq_t freq)
{
	int channel;
	tmv71_vfo_to_channel(rig, vfo, &channel);

	rig_debug(RIG_DEBUG_TRACE, "%s: called for channel %d with freq %f\n",
			  __func__, channel, freq);

	struct tmv71_stepFreq sf = tmv71_resolve_supported_freq(freq);

	struct tmv71_me me_struct = tmv71_get_update_me();
	me_struct.step = sf.step;
	me_struct.freq = sf.frequency;

	return tmv71_update_memory_channel(rig, channel, &me_struct);
}

/*
 * tmv71_do_get_freq
 * Assumes rig!=NULL, freq!=NULL
 * Common function for getting the main and split frequency.
 */
int tmv71_do_get_freq(RIG *rig, vfo_t vfo, freq_t *freq)
{
	int channel;
	tmv71_vfo_to_channel(rig, vfo, &channel);

	rig_debug(RIG_DEBUG_TRACE, "%s: called for channel: %d)\n", __func__, channel);

	struct tmv71_me me_struct;
	int retval = rig_pull_me(rig, channel, &me_struct);

	if (retval == RIG_OK)
	{
		*freq = me_struct.freq;
	}

	return retval;
}

/*
 * tmv71_set_freq
 * Assumes rig!=NULL, freq!=NULL
 */
int tmv71_set_freq(RIG *rig, vfo_t vfo, freq_t freq)
{
	rig_debug(RIG_DEBUG_TRACE, "%s: called for vfo: %d, at %f)\n", __func__, vfo, freq);

	rig_debug(RIG_DEBUG_TRACE, "%s: RIG_VFO_CURRENT: %d)\n", __func__, RIG_VFO_CURR);

	vfo_t split_vfo;
	tmv71_resolve_vfo_for_split(rig, 0, vfo, &split_vfo);

	return tmv71_do_set_freq(rig, split_vfo, freq);
}

/*
 * tmv71_get_freq
 * Assumes rig!=NULL, freq!=NULL
 */
int tmv71_get_freq(RIG *rig, vfo_t vfo, freq_t *freq)
{
	rig_debug(RIG_DEBUG_TRACE, "%s: called for vfo: %d)\n", __func__, vfo);

	vfo_t resolved_vfo;
	tmv71_resolve_vfo_for_split(rig, 0, vfo, &resolved_vfo);

	return tmv71_do_get_freq(rig, resolved_vfo, freq);
}

/*
 * tmv71_set_split_freq
 * Assumes rig!=NULL, freq!=NULL
 */
int tmv71_set_split_freq(RIG *rig, vfo_t vfo, freq_t freq)
{
	rig_debug(RIG_DEBUG_TRACE, "%s: called for vfo: %d, at %f)\n", __func__, vfo, freq);

	vfo_t resolved_vfo;
	tmv71_resolve_vfo_for_split(rig, 1, vfo, &resolved_vfo);

	return tmv71_do_set_freq(rig, resolved_vfo, freq);
}

/*
 * tmv71_get_split_freq
 * Assumes rig!=NULL, freq!=NULL
 */
int tmv71_get_split_freq(RIG *rig, vfo_t vfo, freq_t *freq)
{
	rig_debug(RIG_DEBUG_TRACE, "%s: called for vfo: %d)\n", __func__, vfo);

	vfo_t resolved_vfo;
	tmv71_resolve_vfo_for_split(rig, 1, vfo, &resolved_vfo);

	return tmv71_do_get_freq(rig, resolved_vfo, freq);
}

int tmv71_set_ptt(RIG *rig, vfo_t vfo, ptt_t ptt)
{
	rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

	if( ptt == RIG_PTT_ON ){
		rig_push_tx(rig);
	}
	else{
		rig_push_rx(rig);
	}

	return RIG_OK;
}

/*
 * tmv71_set_mode
 * Assumes rig!=NULL
 * Common function for setting the mode.
 */
int tmv71_set_mode(RIG *rig, vfo_t vfo, rmode_t mode, pbwidth_t width)
{

	vfo_t resolved_vfo;
	tmv71_resolve_vfo_for_split(rig, 0, vfo, &resolved_vfo);

	int channel;
	tmv71_vfo_to_channel(rig, resolved_vfo, &channel);

	rig_debug(RIG_DEBUG_TRACE, "%s: called for channel %d with: %lu\n",
			  __func__, channel, mode);

	struct tmv71_me me_struct = tmv71_get_update_me();

	tmv71_trasform_mode_from_hamlib( mode, &me_struct.mode);

	return tmv71_update_memory_channel(rig, channel, &me_struct);
}

/*
 * tmv71_get_mode
 * Assumes rig!=NULL
 */
int tmv71_get_mode(RIG *rig, vfo_t vfo, rmode_t *mode, pbwidth_t *width)
{
	rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

	vfo_t resolved_vfo;
	tmv71_resolve_vfo_for_split(rig, 0, vfo, &resolved_vfo);

	int channel;
	tmv71_vfo_to_channel(rig, resolved_vfo, &channel);

	struct tmv71_me me_struct;
	int retval = rig_pull_me(rig, channel, &me_struct);

	if (retval != RIG_OK)
	{
		return retval;
	}

	tmv71_trasform_mode_to_hamlib(me_struct.mode, mode, width);

	return retval;
}

/*
 * tmv71_set_mem
 * Assumes rig!=NULL
 * Common function for setting the channel.
 */
int tmv71_set_mem(RIG *rig, vfo_t vfo, int channel)
{
	rig_debug(RIG_DEBUG_TRACE, "%s: called for channel %d on vfo: %d\n",
			  __func__, channel, vfo);

	int retval = rig_push_mr(rig, vfo, channel);
	if (retval != RIG_OK)
	{
		return retval;
	}

	return RIG_OK;
}

/*
 * tmv71_get_mode
 * Assumes rig!=NULL
 */
int tmv71_get_mem(RIG *rig, vfo_t vfo, int *channel)
{
	rig_debug(RIG_DEBUG_TRACE, "%s: called for vfo: %d\n", __func__, vfo);

	int retval = rig_pull_mr(rig, vfo, channel);

	if (retval != RIG_OK)
	{
		return retval;
	}

	return RIG_OK;
}

/*
 * tmv71_set_ts
 * Assumes rig!=NULL
 * function for setting the tuning step.
 */
int tmv71_set_ts(RIG *rig, vfo_t vfo, shortfreq_t step)
{
	int channel;
	tmv71_vfo_to_channel(rig, vfo, &channel);

	rig_debug(RIG_DEBUG_TRACE, "%s: called for channel %d with: %ld\n",
			  __func__, channel, step);

	struct tmv71_me me_struct = tmv71_get_update_me();
	me_struct.step = step;

	return tmv71_update_memory_channel(rig, channel, &me_struct);
}

/*
 * tmv71_get_ts
 * Assumes rig!=NULL
 * function for getting the tuning step.
 */
int tmv71_get_ts(RIG *rig, vfo_t vfo, shortfreq_t *step)
{
	rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

	int channel;
	tmv71_vfo_to_channel(rig, vfo, &channel);

	struct tmv71_me me_struct;
	int retval = rig_pull_me(rig, channel, &me_struct);

	if (retval != RIG_OK)
	{
		return retval;
	}

	*step = me_struct.step;

	return retval;
}

void tmv71_find_tonelist(enum tmv71_tone_type type, tone_t *tone_list)
{
	switch (type)
	{
		case tx_tone:
			tone_list = kenwood42_ctcss_list;
			break;

		case ctcss:
			tone_list = kenwood42_ctcss_list;
			break;

		case dcs:
			tone_list = common_dcs_list;
			break;
	}

}

int tmv71_tone_to_code(enum tmv71_tone_type type, tone_t tone, int *code)
{

	tone_t tone_list;

	tmv71_find_tonelist(type, &tone_list);

	*code = -1;

	int length = sizeof(tone_list) / sizeof(int);

	for (int cI = 0; cI < length; cI++)
	{
		if (tone_list[&cI] == tone)
		{
			*code = cI;
			break;
		}
	}
	if (*code == -1)
	{
		rig_debug(RIG_DEBUG_ERR, "%s: Unsupported tone value '%d'\n", __func__, tone);
		return -RIG_EINVAL;
	}

	return 0;
}

int tmv71_code_to_tone(enum tmv71_tone_type type, tone_t code, tone_t *tone)
{
	tone_t tone_list;

	tmv71_find_tonelist(type, &tone_list);

	int length = sizeof(tone_list) / sizeof(int);

	if( length >= code ){
		rig_debug(RIG_DEBUG_ERR, "%s: Unsupported tone value '%d'\n", __func__, code);
		return -RIG_EINVAL;
	}

	*tone = tone_list[&code];

	return 0;
}

int tmv71_do_get_tone(RIG *rig, vfo_t vfo, enum tmv71_tone_type type, tone_t *tone)
{
	int channel;
	tmv71_vfo_to_channel(rig, vfo, &channel);

	struct tmv71_me me_struct;
	int retval = rig_pull_me(rig, channel, &me_struct);
	if(retval != RIG_OK){
		return retval;
	}

	int enabled = 0;
	int code = 0;
	enum tmv71_tone_type tone_type; 

	switch( type ){
		case tx_tone:
			enabled = me_struct.tone;
			code = me_struct.tone_freq;
			tone_type = ctcss;
			break;

		case ctcss:
			enabled = me_struct.ct;
			code = me_struct.ct_freq;
			tone_type = ctcss;
			break;

		case dcs:
			enabled = me_struct.dcs;
			code = me_struct.dcs_val;
			tone_type = dcs;
			break;
	}

	if( !enabled ){
		return 0;
	}

	tmv71_code_to_tone(tone_type, code, tone);

	return RIG_OK;
}

int tmv71_do_set_tone(RIG *rig, vfo_t vfo, enum tmv71_tone_type type, tone_t tone)
{
	struct tmv71_me me_struct = tmv71_get_update_me();

	//disable all tones first
	me_struct.tone = 0;
	me_struct.ct = 0;
	me_struct.dcs = 0;

	enum tmv71_tone_type tone_type;
	int tone_code;

	switch (type)
	{
	case tx_tone:
		me_struct.tone = 1;
		tone_code = me_struct.tone_freq;
		tone_type = ctcss;
		break;

	case ctcss:
		me_struct.ct = 1;
		tone_code = me_struct.ct_freq;
		tone_type = ctcss;
		break;

	default:
		me_struct.dcs = 1;
		tone_code = me_struct.dcs_val;
		tone_type = dcs;
		break;
	}

	tmv71_tone_to_code(tone_type, tone, &tone_code);

	int channel;
	tmv71_vfo_to_channel(rig, vfo, &channel);

	return tmv71_update_memory_channel(rig, channel, &me_struct);
}

/*
 * tmv71_set_ctcss
 * Assumes rig!=NULL
 * Function for setting the TX tone.
 */
int tmv71_set_ctcss_tone(RIG *rig, vfo_t vfo, tone_t tone)
{
	rig_debug(RIG_DEBUG_TRACE, "%s: called for channel %d with: %d\n",
			  __func__, vfo, tone);

	return tmv71_do_set_tone(rig, vfo, tx_tone, tone);
}

/*
 * tmv71_get_ctcss
 * Assumes rig!=NULL
 * function for getting the TX tone.
 */
int tmv71_get_ctcss_tone(RIG *rig, vfo_t vfo, tone_t *tone)
{
	rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

	tmv71_do_get_tone(rig, vfo, tx_tone, tone);

	return 0;
}

/*
 * tmv71_set_ctcss
 * Assumes rig!=NULL
 * Function for setting the ctcss tone squelch (TX & RX).
 */
int tmv71_set_ctcss_sql(RIG *rig, vfo_t vfo, tone_t tone)
{
	rig_debug(RIG_DEBUG_TRACE, "%s: called for channel %d with: %d\n",
			  __func__, vfo, tone);

	return tmv71_do_set_tone(rig, vfo, ctcss, tone);
}

/*
 * tmv71_get_ctcss
 * Assumes rig!=NULL
 * function for getting the ctcss tone squelch (TX & RX).
 */
int tmv71_get_ctcss_sql(RIG *rig, vfo_t vfo, tone_t *tone)
{
	rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

	tmv71_do_get_tone(rig, vfo, ctcss, tone);

	return 0;
}

/*
 * tmv71_set_ctcss
 * Assumes rig!=NULL
 * Function for setting the DCS tone squelch (TX & RX).
 */
int tmv71_set_dcs_sql(RIG *rig, vfo_t vfo, tone_t tone)
{
	rig_debug(RIG_DEBUG_TRACE, "%s: called for channel %d with: %d\n",
			  __func__, vfo, tone);

	return tmv71_do_set_tone(rig, vfo, dcs, tone);
}

/*
 * tmv71_get_ctcss
 * Assumes rig!=NULL
 * function for getting the DCS tone squelch (TX & RX).
 */
int tmv71_get_dcs_sql(RIG *rig, vfo_t vfo, tone_t *tone)
{
	rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

	tmv71_do_get_tone(rig, vfo, dcs, tone);

	return 0;
}

int tmv71_create_clean_memory_channel(RIG *rig, int channel)
{

	rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

	struct tmv71_me me_struct;

	me_struct.channel = channel;
	me_struct.freq = 146500000;
	me_struct.step = 0;
	me_struct.shift = 0;
	me_struct.reverse = 0;
	me_struct.tone = 0;
	me_struct.ct = 0;
	me_struct.dcs = 0;
	me_struct.tone_freq = 0;
	me_struct.ct_freq = 0;
	me_struct.dcs_val = 0;
	me_struct.offset = 0;
	me_struct.mode = 0;
	me_struct.tx_freq = 0;
	me_struct.tx_step = 0;
	me_struct.lockout = 0;

	int retval = rig_push_me(rig, &me_struct);
	if (retval != RIG_OK)
	{
		return retval;
	}

	return retval;
}

int tmv71_get_current_vfo(RIG *rig, vfo_t *ctrl, vfo_t *ptt){

	rig_debug(RIG_DEBUG_VERBOSE, "%s called\n", __func__);

	int retval = rig_pull_bc(rig, ctrl, ptt);
	if (retval != RIG_OK)
	{
		rig_debug(RIG_DEBUG_VERBOSE, "rig_pull_bc failed with %d\n", retval);
		return retval;
	}

	rig_debug(RIG_DEBUG_TRACE, "%s - returned ctrl: %d, ptt: %d\n", __func__, *ctrl, *ptt);
	
	return RIG_OK;
}

int tmv71_set_vfo(RIG *rig, vfo_t vfo)
{
	rig_debug(RIG_DEBUG_VERBOSE, "%s called with vfo of: %d\n", __func__, vfo);

	int channel, retval;
	vfo_t ctrl, ptt;

	channel = 0;
	ctrl = vfo;

	switch(vfo){
		case RIG_VFO_A:
		case RIG_VFO_VFO:
			channel = tmv71_BAND_A_CHANNEL;
			break;
		case RIG_VFO_B:
			channel = tmv71_BAND_B_CHANNEL;
			break;
		case RIG_VFO_MEM:
			
			// get current band
			retval = tmv71_get_current_vfo(rig, &ctrl, &ptt);
			if (retval != RIG_OK)
			{
				return retval;
			}

			break;
		default:
			rig_debug(RIG_DEBUG_ERR, "%s: Unsupported VFO %d\n", __func__, vfo);
			return -RIG_EVFO;
	}

	// We always operate in memory mode, so set the band to memory.
	retval = rig_push_vm(rig, ctrl, tmv71_BAND_MODE_MEMORY);
	if (retval != RIG_OK)
	{
		return retval;
	}

	// If we're using a psudo vfo, set the special memory channel;
	if( channel > 0 ){
		
		// check that the channel exists
		struct tmv71_me me_struct;
		retval = rig_pull_me(rig, channel, &me_struct);
		if (retval != RIG_OK)
		{
			rig_debug(RIG_DEBUG_VERBOSE, "%s - No psudo vfo.  \
					Creating channel: %d\n", __func__, ctrl);

			// no channel, let's create one.
			retval = tmv71_create_clean_memory_channel(rig, channel);
			if (retval != RIG_OK)
			{
				return retval;
			}
		}

		// set the channel
		retval = rig_push_mr(rig, ctrl, channel);
		if (retval != RIG_OK)
		{
			return retval;
		}
	}

	retval = rig_push_bc(rig, ctrl, ctrl);
	if (retval != RIG_OK)
	{
		return retval;
	}

	return RIG_OK;
}

int tmv71_get_vfo(RIG *rig, vfo_t *vfo){

	rig_debug(RIG_DEBUG_VERBOSE, "%s called\n", __func__);

	// get current band
	vfo_t band, ptt;
	int retval = tmv71_get_current_vfo(rig, &band, &ptt);
	if (retval != RIG_OK)
	{
		rig_debug(RIG_DEBUG_VERBOSE, "%s - tmv71_get_current_vfo failed with %d\n", __func__, retval);
		return retval;
	}

	rig_debug(RIG_DEBUG_VERBOSE, "%s - tmv71_get_current_vfo returned band: %d, ptt: %d\n", __func__, band, ptt);

	//get the momory channel to know if it's a psudo VFD
	int channel;
	retval = rig_pull_mr(rig, band, &channel);
	if (retval != RIG_OK)
	{
		return retval;
	}

	rig_debug(RIG_DEBUG_VERBOSE, "%s - Finding VFO with channel: %d\n", __func__, channel);

	switch (channel)
	{
	case tmv71_BAND_A_CHANNEL:
		*vfo = RIG_VFO_A;
		rig_debug(RIG_DEBUG_VERBOSE, "%s - matched RIG_VFO_A, vfo should be set to: %d\n", __func__, RIG_VFO_A);
		break;

	case tmv71_BAND_B_CHANNEL:
		*vfo = RIG_VFO_B;
		rig_debug(RIG_DEBUG_VERBOSE, "%s - matched RIG_VFO_B, vfo should be set to: %d\n", __func__, RIG_VFO_B);
		break;

	default:
		*vfo = RIG_VFO_MEM;
		rig_debug(RIG_DEBUG_VERBOSE, "%s - matched RIG_VFO_MEM, vfo should be set to: %d\n", __func__, RIG_VFO_MEM);
		}

	rig_debug(RIG_DEBUG_VERBOSE, "%s - mapped channel: %d to vfo: %d\n", __func__, channel, *vfo);
	rig_debug(RIG_DEBUG_VERBOSE, "%s - returning vfo of: %d\n", __func__, *vfo);

	return RIG_OK;
}

/*
 *	tmv71_set_split_vfo
 *
 *	This radio has two VFOs, and either one can be the TX/RX.  As such, this function does two things:
 *	- Sets PTT control on the specified VFO.
 *	- If "Split" is enabled, sets the TX_VFO and RX_VFO for use in Set_Freq and Set_Split_Freq
 *	
 *
 */
int tmv71_set_split_vfo(RIG *rig, vfo_t vfo, split_t split, vfo_t txvfo)
{
	char vfo_debug_name[14];
	tmv71_resolve_vfo_name(txvfo, vfo_debug_name);
	rig_debug(RIG_DEBUG_TRACE, "%s: called with split %d, vfo %s\n",
			  __func__, split, vfo_debug_name);

	int retval = rig_push_bc(rig, txvfo, txvfo);
	if( retval != RIG_OK)
	{
		return retval;
	}

	if (split == RIG_SPLIT_ON) {
		tmv71_state.vfo_tx = txvfo;
		tmv71_state.vfo_rx = txvfo == RIG_VFO_A ? RIG_VFO_B : RIG_VFO_A;
		tmv71_state.split_mode_active = RIG_SPLIT_ON;

		char vfo_debug_tx[14], vfo_debug_rx[14];
		tmv71_resolve_vfo_name(tmv71_state.vfo_tx, vfo_debug_tx);
		tmv71_resolve_vfo_name(tmv71_state.vfo_rx, vfo_debug_rx);
		rig_debug(RIG_DEBUG_TRACE, "%s: Set split VFOs TX: %s, RX: %s\n",
				  __func__, vfo_debug_tx, vfo_debug_rx);
	}
	else{
		tmv71_state.split_mode_active = RIG_SPLIT_OFF;
	}

	return RIG_OK;
}

int tmv71_get_split_vfo(RIG *rig, vfo_t vfo, split_t *split, vfo_t *txvfo)
{

	*txvfo = tmv71_state.vfo_tx;

	//The following is just sanity checking.

	vfo_t band, ptt;
	int retval = tmv71_get_current_vfo(rig, &band, &ptt);
	if (retval != RIG_OK)
	{
		return retval;
	}

	if (ptt != tmv71_state.vfo_tx)
	{
		//Hmmm, we have a problem.  The operator has manually switched the 
		//	TX vfo so we are in an inconsistent state.  As this is a GET call,
		//	we have a decision to make.  If we update the switch the split VFO
		//	to match the actual radio state, we introduce enexpected behaviour 
		//	to the code.  Better to write a warning to the log, and leave the 
		//	VFO's as set by the set_split_vfo call.

		rig_debug(RIG_DEBUG_WARN, "The PTT band has been manually changed \
				leaving the radio in an inconsistent state.  RigCtl will continue \
				to address %s as the TX band.\n",
			ptt == 0 ? "VFO A" : "VFO B");
	}

	return RIG_OK;
}

static int tmv71_find_tuning_step_index(RIG *rig, shortfreq_t ts, int *step_index)
{
	int k, stepind = -1;

	for (k = 0; rig->state.tuning_steps[k].ts != 0; ++k)
	{

		if ((rig->caps->tuning_steps[k].modes == RIG_MODE_NONE) && (rig->caps->tuning_steps[k].ts == 0))
			break;
		else if (rig->caps->tuning_steps[k].ts == ts)
		{
			stepind = k;
			break;
		}
	}
	if (stepind == -1)
	{
		rig_debug(RIG_DEBUG_ERR, "%s: Unsupported tuning step value '%ld'\n", __func__, ts);
		return -RIG_EINVAL;
	}

	*step_index = stepind;

	return RIG_OK;
}



int tmv71_set_channel(RIG *rig, vfo_t vfo, const channel_t *chan) {

	rig_debug(RIG_DEBUG_VERBOSE, "%s called\n", __func__);

	struct tmv71_me me_struct;

	me_struct.channel = chan->channel_num;
	me_struct.freq = chan->freq;
	me_struct.tx_freq = chan->tx_freq;

	int retval = tmv71_find_tuning_step_index(rig, chan->tuning_step, &me_struct.step);
	if (retval != RIG_OK)
	{
		return retval;
	}

	retval = tmv71_transform_rptr_shift_from_hamlib(chan->rptr_shift, &me_struct.shift);
	if (retval != RIG_OK)
	{
		return retval;
	}

	me_struct.offset = chan->rptr_offs;

	me_struct.reverse = (chan->funcs & RIG_FUNC_REV) ? 1 : 0;
	me_struct.tone = (chan->funcs & RIG_FUNC_TONE) ? 1 : 0;
	me_struct.ct = (chan->funcs & RIG_FUNC_TSQL) ? 1 : 0;

	//disable all tones
	me_struct.tone = 0;
	me_struct.ct = 0;
	me_struct.dcs = 0;

	if (me_struct.tone && chan->ctcss_tone != 0)
	{
		tmv71_tone_to_code(tx_tone, chan->ctcss_tone, &me_struct.tone_freq);
		me_struct.tone = 1;
	}

	if (me_struct.ct && chan->ctcss_sql != 0)
	{
		tmv71_tone_to_code(tx_tone, chan->ctcss_sql, &me_struct.ct_freq);
		me_struct.ct = 1;
	}

	if (chan->dcs_sql != 0)
	{
		tmv71_tone_to_code(tx_tone, chan->dcs_sql, &me_struct.dcs_val);
		me_struct.dcs = 1;
	}

	me_struct.lockout = (chan->flags & RIG_CHFLAG_SKIP) ? 1 : 0;

	retval = tmv71_trasform_mode_from_hamlib(chan->mode, &me_struct.mode);
	if (retval != RIG_OK)
	{
		return retval;
	}

	me_struct.tx_step = 0;

	retval = rig_push_me(rig, &me_struct);
	if (retval != RIG_OK)
	{
		return retval;
	}

	return rig_push_mn(rig, me_struct.channel, (char *)chan->channel_desc);
}

int tmv71_get_channel(RIG *rig, vfo_t vfo, channel_t *chan, int read_only)
{
	int retval;
	struct tmv71_me me_struct;

	rig_debug(RIG_DEBUG_VERBOSE, "%s called\n", __func__);

	if (!rig || !chan)
	{
		return -RIG_EINVAL;
	}

	retval = rig_pull_me(rig, chan->channel_num, &me_struct);
	if (retval != RIG_OK)
	{
		return retval;
	}

	chan->freq = me_struct.freq;
	chan->vfo = RIG_VFO_CURR;

	retval = tmv71_trasform_mode_to_hamlib(me_struct.mode, &chan->mode, &chan->width);
	if (retval != RIG_OK)
	{
		return retval;
	}

	chan->tuning_step = rig->caps->tuning_steps[me_struct.step].ts;

	chan->funcs = 0;

	if (me_struct.tone != 0)
	{
		chan->funcs |= RIG_FUNC_TONE;
	}
	if (me_struct.ct != 0)
	{
		chan->funcs |= RIG_FUNC_TSQL;
	}
	if (me_struct.reverse != 0)
	{
		chan->funcs |= RIG_FUNC_REV;
	}

	chan->ctcss_tone = rig->caps->ctcss_list[me_struct.tone_freq];
	chan->ctcss_sql = rig->caps->ctcss_list[me_struct.ct_freq];
	chan->dcs_code = 0;
	if (me_struct.dcs)
	{
		tone_t *dcs_list = common_dcs_list;
		chan->dcs_sql = dcs_list[me_struct.dcs_val];
	}
	else
	{
		chan->dcs_sql = 0;
	}

	retval = tmv71_transform_rptr_shift_to_hamlib(me_struct.shift, &chan->rptr_shift);
	if (retval != RIG_OK)
	{
		return retval;
	}

	chan->rptr_offs = me_struct.offset;

	retval = rig_pull_mn(rig, chan->channel_num, chan->channel_desc);
	if (retval != RIG_OK)
	{
		return retval;
	}

	chan->flags = RIG_CHFLAG_NONE;

	if (me_struct.lockout)
	{
		chan->flags |= RIG_CHFLAG_SKIP;
	}

	chan->tx_freq = me_struct.tx_freq;

	// Unsupported features
	chan->bank_num = 0;
	chan->ant = 0;
	chan->split = RIG_SPLIT_OFF;
	chan->tx_vfo = RIG_VFO_NONE;
	chan->tx_mode = RIG_MODE_NONE;
	chan->tx_width = 0;
	chan->rit = 0;
	chan->xit = 0;
	chan->scan_group = 0;
	// TODO: chan->levels
	chan->ext_levels = NULL;

	return RIG_OK;
}

int tmv71_get_dcd(RIG *rig, vfo_t vfo, dcd_t *dcd){

	return rig_pull_by(rig, vfo, dcd);
}