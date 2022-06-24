/*
 *  Hamlib Kenwood backend - TM-V71(A) description.
 *  Copyright (c) 2011 by Charles Suprin
 *  Copyright (c) 2018 Mikael Nousiainen
 *  Copyright (c) 2022 Lance Conry
 *
 *  Command set specification available in:
 *  - https://github.com/LA3QMA/TM-V71_TM-D710-Kenwood
 *  - http://kd7dvd.us/equipment/tm-d710a/manuals/control_commands.pdf
 *
 *  This codebase is derived from the TMD-710 codebase.
 * 
 *  Features
 *  -In VFO mode, the radio can only set the frequency within the selected
 *     frequency band of the current VFO.
 *  -This implementation differs from the tmd710.c in that it never uses VFO
 *    mode, but rather uses two memory channels (998 and 999) as psudo VFOs.
 *    This allows for a more functional implementation.
 *  -Channel memory slots 998 and 999 are used for VOFA (left side of radio)
 *    and VFOB (right side of radio) respectively.


 *  Features not yet implemented:
 *  - DTMF send
 *  - Tone burst frequency setting
 *  - Call channel settings
 *  - Change between dual-band/single-band mode
 *  - Several miscellaneous settings available via the MU menu command, which could be exposed via extparms/extlevels
 *
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
#include <time.h>

#include "hamlib/rig.h"
#include "kenwood.h"
#include "th.h"
#include "tones.h"
#include "num_stdio.h"
#include "misc.h"

/* structure for holding frequency and supporting step size */
struct StepFreq{
  long frequency;
  int step;
};

static int tmv71_open(RIG *rig);
static int tmv71_do_get_freq(RIG *rig, int channel, freq_t *freq);
static int tmv71_do_set_freq(RIG *rig, int channel, freq_t freq);
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
static int tmv71_set_rptr_shift(RIG *rig, vfo_t vfo, rptr_shift_t shift);
static int tmv71_get_rptr_shift(RIG *rig, vfo_t vfo, rptr_shift_t* shift);
static int tmv71_set_rptr_offs(RIG *rig, vfo_t vfo, shortfreq_t offset);
static int tmv71_get_rptr_offs(RIG *rig, vfo_t vfo, shortfreq_t* offset);
static int tmv71_get_mem(RIG *rig, vfo_t vfo, int *ch);
static int tmv71_set_mem(RIG *rig, vfo_t vfo, int ch);
static int tmv71_set_dcs_sql(RIG * rig, vfo_t vfo, tone_t code);
static int tmv71_get_dcs_sql(RIG * rig, vfo_t vfo, tone_t *code);
static int tmv71_set_channel(RIG *rig, vfo_t vfo, const channel_t *chan);
static int tmv71_get_channel(RIG *rig, vfo_t vfo, channel_t *chan, int read_only);
static int tmv71_set_ptt(RIG *rig, vfo_t vfo, ptt_t ptt);
static int tmv71_get_dcd(RIG *rig, vfo_t vfo, dcd_t *dcd);
static int tmv71_vfo_op(RIG *rig, vfo_t vfo, vfo_op_t op);
static int tmv71_set_level(RIG *rig, vfo_t vfo, setting_t level, value_t val);
static int tmv71_get_level(RIG *rig, vfo_t vfo, setting_t level, value_t *val);
static int tmv71_get_func(RIG *rig, vfo_t vfo, setting_t func, int *status);
static int tmv71_set_func(RIG *rig, vfo_t vfo, setting_t func, int status);
static int tmv71_get_parm(RIG *rig, setting_t parm, value_t *val);
static int tmv71_set_parm(RIG *rig, setting_t parm, value_t val);
static int tmv71_get_ext_level(RIG *rig, vfo_t vfo, token_t token, value_t *val);
static int tmv71_set_ext_level(RIG *rig, vfo_t vfo, token_t token, value_t val);
static struct StepFreq tmv71_resolve_supported_freq(int freq);
static int tmv71_create_clean_memory_channel(RIG *rig, int channel);

#define tmv71_MODES     (RIG_MODE_FM|RIG_MODE_FMN|RIG_MODE_AM)
#define tmv71_MODES_FM  (RIG_MODE_FM|RIG_MODE_FMN)
#define tmv71_MODES_TX  (RIG_MODE_FM|RIG_MODE_FMN)

#define tmv71_FUNC_GET (RIG_FUNC_TSQL|   \
                       RIG_FUNC_TONE|   \
                       RIG_FUNC_REV|    \
                       RIG_FUNC_LOCK|   \
                       RIG_FUNC_ARO|    \
                       RIG_FUNC_AIP|    \
                       RIG_FUNC_RESUME)
#define tmv71_FUNC_SET (RIG_FUNC_TSQL|   \
                       RIG_FUNC_TONE|   \
                       RIG_FUNC_TBURST| \
                       RIG_FUNC_REV|    \
                       RIG_FUNC_LOCK|   \
                       RIG_FUNC_ARO|    \
                       RIG_FUNC_AIP|    \
                       RIG_FUNC_RESUME)

#define tmv71_LEVEL_ALL (RIG_LEVEL_SQL| \
                        RIG_LEVEL_RFPOWER)

#define tmv71_PARMS  (RIG_PARM_BACKLIGHT|\
                        RIG_PARM_BEEP|\
                        RIG_PARM_APO)

#define tmv71_VFO_OP (RIG_OP_UP|RIG_OP_DOWN)

#define tmv71_CHANNEL_CAPS   \
            TH_CHANNEL_CAPS,\
            .flags=1,   \
            .dcs_code=1,    \
            .dcs_sql=1,

#define tmv71_CHANNEL_CAPS_WO_LO \
            TH_CHANNEL_CAPS,\
            .dcs_code=1,    \
            .dcs_sql=1,

#define TOKEN_BACKEND(t) (t)

#define TOK_LEVEL_EXT_DATA_BAND TOKEN_BACKEND(100)

// TM-D710 protocol definitions

#define tmv71_BAND_A 0
#define tmv71_BAND_B 1

#define tmv71_BAND_MODE_VFO 0
#define tmv71_BAND_MODE_MEMORY 1
#define tmv71_BAND_MODE_CALL 2
#define tmv71_BAND_MODE_WX 3

#define tmv71_RF_POWER_MIN 0
#define tmv71_RF_POWER_MAX 2

#define tmv71_SQL_MIN 0
#define tmv71_SQL_MAX 0x1F

// TM-D710 MU command value tables

#define tmv71_ANNOUNCE_OFF 0
#define tmv71_ANNOUNCE_AUTO 1
#define tmv71_ANNOUNCE_MANUAL 2

#define tmv71_LANGUAGE_ENGLISH 0
#define tmv71_LANGUAGE_JAPANESE 1

#define tmv71_SMETER_HANG_UP_TIME_OFF 0
#define tmv71_SMETER_HANG_UP_TIME_125 1
#define tmv71_SMETER_HANG_UP_TIME_250 2
#define tmv71_SMETER_HANG_UP_TIME_500 3

#define tmv71_MUTE_HANG_UP_TIME_OFF 0
#define tmv71_MUTE_HANG_UP_TIME_125 1
#define tmv71_MUTE_HANG_UP_TIME_250 2
#define tmv71_MUTE_HANG_UP_TIME_500 3
#define tmv71_MUTE_HANG_UP_TIME_750 4
#define tmv71_MUTE_HANG_UP_TIME_1000 5

#define tmv71_TIMEOUT_TIMER_3MIN 0
#define tmv71_TIMEOUT_TIMER_5MIN 1
#define tmv71_TIMEOUT_TIMER_10MIN 2

#define tmv71_RECALL_METHOD_ALL 0
#define tmv71_RECALL_METHOD_CURRENT 1

#define tmv71_ECHOLINK_SPEED_FAST 0
#define tmv71_ECHOLINK_SPEED_SLOW 1

#define tmv71_DTMF_SPEED_FAST 0
#define tmv71_DTMF_SPEED_SLOW 1

#define tmv71_DTMF_PAUSE_100 0
#define tmv71_DTMF_PAUSE_250 1
#define tmv71_DTMF_PAUSE_500 2
#define tmv71_DTMF_PAUSE_750 3
#define tmv71_DTMF_PAUSE_1000 4
#define tmv71_DTMF_PAUSE_1500 5
#define tmv71_DTMF_PAUSE_2000 6

#define tmv71_BACKLIGHT_COLOR_AMBER 0
#define tmv71_BACKLIGHT_COLOR_GREEN 1

#define tmv71_SCAN_RESUME_TIME 0
#define tmv71_SCAN_RESUME_CARRIER 1
#define tmv71_SCAN_RESUME_SEEK 2

#define tmv71_AUTO_POWER_OFF_OFF 0
#define tmv71_AUTO_POWER_OFF_30MIN 1
#define tmv71_AUTO_POWER_OFF_60MIN 2
#define tmv71_AUTO_POWER_OFF_90MIN 3
#define tmv71_AUTO_POWER_OFF_120MIN 4
#define tmv71_AUTO_POWER_OFF_180MIN 5

#define tmv71_EXT_DATA_BAND_A 0
#define tmv71_EXT_DATA_BAND_B 1
#define tmv71_EXT_DATA_BAND_TXA_RXB 2
#define tmv71_EXT_DATA_BAND_TXB_RXA 3

#define tmv71_EXT_DATA_SPEED_1200 0
#define tmv71_EXT_DATA_SPEED_9600 1

#define tmv71_SQC_SOURCE_OFF 0
#define tmv71_SQC_SOURCE_BUSY 1
#define tmv71_SQC_SOURCE_SQL 2
#define tmv71_SQC_SOURCE_TX 3
#define tmv71_SQC_SOURCE_BUSY_OR_TX 4
#define tmv71_SQC_SOURCE_SQL_OR_TX 5

#define tmv71_VFO_A_CHANNEL 998
#define tmv71_VFO_B_CHANNEL 999

static rmode_t tmv71_mode_table[KENWOOD_MODE_TABLE_MAX] = {
    [0] = RIG_MODE_FM,
    [1] = RIG_MODE_FMN,
    [2] = RIG_MODE_AM,
};

static struct kenwood_priv_caps  tmv71_priv_caps  = {
    .cmdtrm =  EOM_TH,   /* Command termination character */
    .mode_table = tmv71_mode_table,
};

/* Private TM-D710 extra levels definitions
 *
 * Token definitions for .cfgparams in rig_caps
 * See enum rig_conf_e and struct confparams in rig.h
 */
const struct confparams tmv71_mem_ext_levels[] = {
    { TOK_LEVEL_EXT_DATA_BAND, "EXTDATABAND", "External data band", "External data band",
        NULL, RIG_CONF_COMBO, { .c = { .combostr = { "A", "B", "TXA-RXB", "TXB-RXA", NULL } } }
    },
    { RIG_CONF_END, NULL, }
};

const struct rig_caps tmv71_caps = {
    .rig_model = RIG_MODEL_TMD710_MEM,
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
        {0, 199, RIG_MTYPE_MEM, {tmv71_CHANNEL_CAPS}},          /* normal MEM */
        {200, 219, RIG_MTYPE_EDGE, {tmv71_CHANNEL_CAPS}},       /* U/L MEM */
        {221, 222, RIG_MTYPE_CALL, {tmv71_CHANNEL_CAPS_WO_LO}}, /* Call 0/1 */
        RIG_CHAN_END,
    },
    /*
     * TODO: Japan & TM-D700S, and Taiwan models
     */
    .rx_range_list1 = {
        {MHz(118), MHz(470), tmv71_MODES, -1, -1, RIG_VFO_A | RIG_VFO_MEM},
        {MHz(136), MHz(174), tmv71_MODES_FM, -1, -1, RIG_VFO_A | RIG_VFO_B | RIG_VFO_MEM},
        {MHz(300), MHz(524), tmv71_MODES_FM, -1, -1, RIG_VFO_A | RIG_VFO_B | RIG_VFO_MEM},
        {MHz(800), MHz(1300), tmv71_MODES_FM, -1, -1, RIG_VFO_B | RIG_VFO_MEM},
        RIG_FRNG_END,
    }, /* rx range */
    .tx_range_list1 = {
        {MHz(144), MHz(146), tmv71_MODES_TX, W(5), W(50), RIG_VFO_A | RIG_VFO_B | RIG_VFO_MEM},
        {MHz(430), MHz(440), tmv71_MODES_TX, W(5), W(35), RIG_VFO_A | RIG_VFO_B | RIG_VFO_MEM},
        RIG_FRNG_END,
    }, /* tx range */

    .rx_range_list2 = {
        {MHz(118), MHz(470), tmv71_MODES, -1, -1, RIG_VFO_A | RIG_VFO_MEM},
        {MHz(136), MHz(174), tmv71_MODES_FM, -1, -1, RIG_VFO_A | RIG_VFO_B | RIG_VFO_MEM},
        {MHz(300), MHz(524), tmv71_MODES_FM, -1, -1, RIG_VFO_A | RIG_VFO_B | RIG_VFO_MEM},
        {MHz(800), MHz(1300), tmv71_MODES_FM, -1, -1, RIG_VFO_B | RIG_VFO_MEM}, /* TODO: cellular blocked */
        RIG_FRNG_END,
    }, /* rx range */
    .tx_range_list2 = {
        {MHz(144), MHz(148), tmv71_MODES_TX, W(5), W(50), RIG_VFO_A | RIG_VFO_B | RIG_VFO_MEM},
        {MHz(430), MHz(450), tmv71_MODES_TX, W(5), W(35), RIG_VFO_A | RIG_VFO_B | RIG_VFO_MEM},
        RIG_FRNG_END,
    }, /* tx range */

    .tuning_steps = {
        {tmv71_MODES, kHz(5)},
        {tmv71_MODES, kHz(6.25)},
        {tmv71_MODES, kHz(8.33)},
        {tmv71_MODES, kHz(10)},
        {tmv71_MODES, kHz(12.5)},
        {tmv71_MODES, kHz(15)},
        {tmv71_MODES, kHz(20)},
        {tmv71_MODES, kHz(25)},
        {tmv71_MODES, kHz(30)},
        {tmv71_MODES, kHz(50)},
        {tmv71_MODES, kHz(100)},
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
    //.set_trn =  th_set_trn,
    //.get_trn =  th_get_trn,

    .set_func = tmv71_set_func,
    .get_func = tmv71_get_func,
    .set_level = tmv71_set_level,
    .get_level = tmv71_get_level,
    .set_parm = tmv71_set_parm,
    .get_parm = tmv71_get_parm,
    //.get_info =  th_get_info,
    .get_dcd = tmv71_get_dcd,
    .set_ptt = tmv71_set_ptt,
    .vfo_op = tmv71_vfo_op,
    //.scan   =  th_scan,
    .set_ext_level = tmv71_set_ext_level,
    .get_ext_level = tmv71_get_ext_level,

    .extlevels = tmv71_mem_ext_levels,

    .set_rptr_shift = tmv71_set_rptr_shift,
    .get_rptr_shift = tmv71_get_rptr_shift,
    .set_rptr_offs = tmv71_set_rptr_offs,
    .get_rptr_offs = tmv71_get_rptr_offs,

    .decode_event = th_decode_event,
};

/* structure for handling FO radio command */
typedef struct {
  int vfo;       // P1
  freq_t freq;   // P2
  int step;      // P3
  int shift;     // P4
  int reverse;   // P5
  int tone;      // P6
  int ct;        // P7
  int dcs;       // P8
  int tone_freq; // P9
  int ct_freq;   // P10
  int dcs_val;   // P11
  int offset;    // P12
  int mode;      // P13
} tmv71_fo;

/* structure for handling ME radio command */
typedef struct {
  int channel;   // P1
  freq_t freq;   // P2
  int step;      // P3
  int shift;     // P4
  int reverse;   // P5
  int tone;      // P6
  int ct;        // P7
  int dcs;       // P8
  int tone_freq; // P9
  int ct_freq;   // P10
  int dcs_val;   // P11
  int offset;    // P12
  int mode;      // P13
  freq_t tx_freq;   // P14
  int p15_unknown;  // P15
  int lockout;   // P16
} tmv71_me;

/* structure for handling MU (menu) radio command */
typedef struct {
  int beep; // P1 0/1
  int beep_volume; // P2 (1-7)
  int ext_speaker_mode; // P3
  int announce; // P4
  int language; // P5
  int voice_volume; // P6 (0-7)
  int voice_speed; // P7 (0-4)
  int playback_repeat; // P8 0/1
  int playback_repeat_interval; // P9 (00-60)
  int continuous_recording; // P10 0/1
  int vhf_aip; // P11 0/1
  int uhf_aip; // P12 0/1
  int smeter_sql_hang_up_time; // P13
  int mute_hang_up_time; // P14
  int beat_shift; // P15 0/1
  int timeout_timer; // P16
  int recall_method; // P17
  int echolink_speed; // P18
  int dtmf_hold; // P19 0/1
  int dtmf_speed; // P20
  int dtmf_pause; // P21
  int dtmf_key_lock; // P22 0/1
  int auto_repeater_offset; // P23 0/1
  int tone_1750_tx_hold; // P24 0/1
  int p25_unknown; // TODO
  int brightness_level; // P26 (0-8)
  int auto_brightness; // P27 0/1
  int backlight_color; // P28
  int pf1_key; // P29
  int pf2_key; // P30
  int mic_pf1_key; // P31
  int mic_pf2_key; // P32
  int mic_pf3_key; // P33
  int mic_pf4_key; // P34
  int mic_key_lock; // P35 0/1
  int scan_resume; // P36
  int auto_power_off; // P37
  int ext_data_band; // P38
  int ext_data_speed; // P39
  int sqc_source; // P40
  int auto_pm_store; // P41 0/1
  int display_partition_bar; // P42 0/1
} tmv71_mu;

/* structure for handling VE radio command */
typedef struct
{
  vfo_t band;     // P1
  int mode;     // P2
} tmv71_vm;

static int tmv71_open(RIG *rig) {
	
	rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

	rig->state.tx_vfo = RIG_VFO_A;
	rig_debug(RIG_DEBUG_TRACE, "RIG_VFO_A: %d\trig->state.tx_vfo: %d\n", RIG_VFO_A, rig->state.tx_vfo);
  
  // Sleep the ensure the serial connection has been established beofore trying 
  // communicate with the radio.
  long msec = 1000;
  struct timespec ts;
  ts.tv_sec = msec / 1000;
  ts.tv_nsec = (msec % 1000) * 1000000;
  nanosleep(&ts, &ts);
  
  int retval = tmd710_setup(rig);
  if (retval != RIG_OK)
  {
    return retval;
  }

  return 0;
}

/*
All operations will be using memory channels, so perform the following init:

- Create (if required) the memory channels
- Put the both sides of the radio in memory mode and set channels.
- Assign PTT to the right side of the radio
*/
int tmd710_setup(RIG *rig) {

  rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

  /*
  // create two clean memory channels with clean defaults set.
  int retval998 = tmv71_set_clean_memory_channel(rig, tmv71_VFO_A_CHANNEL);
  int retval999 = tmv71_set_clean_memory_channel(rig, tmv71_VFO_B_CHANNEL);
  if (retval998 != RIG_OK || retval999 != RIG_OK)
  {
    return retval998 != RIG_OK ? retval998 : retval999;
  }

  // assign respective channels to each side of the radio
  int retvalA = tmv71_set_mem(rig, RIG_VFO_A, tmv71_VFO_A_CHANNEL);
  int retvalB = tmv71_set_mem(rig, RIG_VFO_B, tmv71_VFO_B_CHANNEL);
  if (retvalA != RIG_OK || retvalB != RIG_OK)
  {
    return retvalA != RIG_OK ? retvalA : retvalB;
  }
  */

  // set control to the right of the radio.
  /*
  int retValSet = tmv71_set_vfo(rig, RIG_VFO_B);
  if (retValSet != RIG_OK)
  {
    return retValSet;
  }
  */
 
  return 0;
}

int tmv71_set_vfo_channel(RIG *rig, vfo_t vfo, int channel){

  //Set the vfo to memory mode
  tmv71_vm vm_struct;
  vm_struct.band = vfo;
  vm_struct.mode = tmv71_BAND_MODE_MEMORY;

  int retval = tmv71_rigSet_vfoMode(rig, &vm_struct);
  if (retval != RIG_OK)
  {
    return retval;
  }

  //check that the channel exists
  tmv71_me me_struct;
  retval = tmv71_pull_me(rig, channel, &me_struct);
  if( retval != RIG_OK ){
    
    //no channel, let's create one.
    retval = tmv71_create_clean_memory_channel(rig, channel);
    if( retval != RIG_OK ){
      return retval;
    }
  }

  //set the channel
  retval = tmv71_rigSet_memoryChannel(rig, vfo, channel);

}

int tmv71_create_clean_memory_channel(RIG *rig, int channel)
{

  rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

  tmv71_me me_struct;

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
  me_struct.p15_unknown = 0;
  me_struct.lockout = 0;

  int retval = tmv71_push_me(rig, &me_struct);
  if (retval != RIG_OK)
  {
    return retval;
  }

  return retval;
}

static int tmv71_get_vfo_num(RIG *rig, int *vfonum, vfo_t *vfo) {
  char buf[10];
  int retval, ctrlnum, pttnum;

  rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

  retval = kenwood_transaction(rig, "BC", buf, sizeof(buf));
  if (retval != RIG_OK) {
    return retval;
  }

  retval = sscanf(buf, "BC %d,%d", &ctrlnum, &pttnum);
  if (retval != 2) {
    rig_debug(RIG_DEBUG_ERR, "Unable to parse '%s', expected 'BC c,p'\n", buf);
    return -RIG_EPROTO;
  }

  switch (ctrlnum) {
    case tmv71_BAND_A:
      if (vfo != NULL) {
        *vfo = RIG_VFO_A;
      }
      break;
    case tmv71_BAND_B:
      if (vfo != NULL) {
        *vfo = RIG_VFO_B;
      }
      break;
    default:
      rig_debug(RIG_DEBUG_ERR, "%s: Unexpected VFO value '%c'\n", __func__, buf[3]);
      return -RIG_EVFO;
  }

  if (vfonum != NULL) {
    *vfonum = ctrlnum;
  }

  return RIG_OK;
}

static int tmv71_get_vfo_and_mode(RIG *rig, vfo_t *vfo, int *vfomode)
{
  char cmdbuf[10], buf[10];
  int retval, vfonum, vfomodenum;

  rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

  /* Get VFO band */

  retval = tmv71_get_vfo_num(rig, &vfonum, vfo);
  if (retval != RIG_OK) {
    return retval;
  }

  /* Get mode of the VFO band */

  snprintf(cmdbuf, sizeof(cmdbuf), "VM %d", vfonum);

  retval = kenwood_safe_transaction(rig, cmdbuf, buf, 10, 6);
  if (retval != RIG_OK) {
    return retval;
  }

  retval = sscanf(buf, "VM %d,%d", &vfonum, &vfomodenum);
  if (retval != 2) {
    rig_debug(RIG_DEBUG_ERR, "Unable to parse '%s', expected 'VM c,m'\n", buf);
    return -RIG_EPROTO;
  }

  if (vfomode != NULL) {
    *vfomode = vfomodenum;
  }

  return RIG_OK;
}

static int tmv71_resolve_vfo(RIG *rig, vfo_t vfo, vfo_t *resolved_vfo, int *resolved_vfonum)
{
  switch (vfo) {
    case RIG_VFO_CURR:
      return tmv71_get_vfo_num(rig, resolved_vfonum, resolved_vfo);
    case RIG_VFO_A:
      if (resolved_vfo != NULL) {
        *resolved_vfo = RIG_VFO_A;
      }
      if (resolved_vfonum != NULL) {
        *resolved_vfonum = tmv71_BAND_A;
      }
      break;
    case RIG_VFO_B:
      if (resolved_vfo != NULL) {
        *resolved_vfo = RIG_VFO_B;
      }
      if (resolved_vfonum != NULL) {
        *resolved_vfonum = tmv71_BAND_B;
      }
      break;
    default:
      return -RIG_ENTARGET;
  }

  return RIG_OK;
}

static int tmv71_scan_me(char *buf, tmv71_me *me_struct) {
  int retval;

  retval = num_sscanf(buf, "ME %x,%"SCNfreq",%x,%x,%x,%x,%x,%x,%d,%d,%d,%d,%d,%"SCNfreq",%d,%d",
      &me_struct->channel, &me_struct->freq,
      &me_struct->step, &me_struct->shift,
      &me_struct->reverse, &me_struct->tone,
      &me_struct->ct, &me_struct->dcs,
      &me_struct->tone_freq, &me_struct->ct_freq,
      &me_struct->dcs_val, &me_struct->offset,
      &me_struct->mode, &me_struct->tx_freq,
      &me_struct->p15_unknown, &me_struct->lockout);

  if (retval != 16) {
    rig_debug(RIG_DEBUG_ERR, "%s: Unexpected reply '%s'\n", __func__, buf);
    return -RIG_ERJCTED;
  }

  return RIG_OK;
}

/*
 * The TM-D710(G) has a single command ME that queries and sets many values.
 * This function pulls that string from the radio given a memory channel.
 * Push/pull naming is used inside the backend rather than get/set.
 * There is one unknown field.
 */
int tmv71_pull_me(RIG *rig, int ch, tmv71_me *me_struct)
{
  char cmdbuf[8];
  char buf[80];
  int retval;

  rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

  snprintf(cmdbuf, sizeof(cmdbuf), "ME %03d", ch);
  retval = kenwood_transaction(rig, cmdbuf, buf, sizeof(buf));
  if (retval != RIG_OK) {
    return retval;
  }

  retval = tmv71_scan_me(buf, me_struct);
  if (retval != RIG_OK) {
    return retval;
  }

  return RIG_OK;
}

int tmv71_push_me(RIG *rig, tmv71_me *me_struct)
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
      me_struct->p15_unknown, me_struct->lockout);

  return kenwood_transaction(rig, cmdbuf, buf, sizeof(buf));
}

int tmv71_get_memory_name(RIG *rig, int ch, char *name)
{
  char cmdbuf[8];
  char buf[80];
  int retval;

  rig_debug(RIG_DEBUG_TRACE, "%s: called on channel %d\n", __func__, ch);

  snprintf(cmdbuf, sizeof(cmdbuf), "MN %03d", ch);
  retval = kenwood_transaction(rig, cmdbuf, buf, sizeof(buf));
  if (retval != RIG_OK) {
    return retval;
  }

  retval = num_sscanf(buf, "MN %d,%s", &ch, name);
  if( retval == 1) {
    name = "";
  }
  else if (retval != 2) {
    rig_debug(RIG_DEBUG_ERR, "%s: Unexpected reply '%s'\n", __func__, buf);
    return -RIG_ERJCTED;
  }

  return RIG_OK;
}

int tmv71_set_memory_name(RIG *rig, int ch, char *name)
{
  char cmdbuf[32];
  char buf[80];
  int retval;

  rig_debug(RIG_DEBUG_TRACE, "%s: called on channel %d with name %s\n", __func__, ch, name);

  snprintf(cmdbuf, sizeof(cmdbuf), "MN %03d,%s", ch, name);
  retval = kenwood_transaction(rig, cmdbuf, buf, sizeof(buf));
  if (retval != RIG_OK) {
    return retval;
  }

  return RIG_OK;
}

/*
 * The TM-D710(G) has a single command FO that queries and sets many values.
 * This function pulls that string from the radio given a VFO.
 * Push/pull language is used inside the backend rather than get/set.
 */
int tmv71_pull_fo(RIG *rig, vfo_t vfo, tmv71_fo *fo_struct)
{
  char cmdbuf[8];
  char buf[80];
  int vfonum;
  int retval;

  rig_debug(RIG_DEBUG_TRACE, "%s: called with VFO %08X\n", __func__, vfo);

  retval = tmv71_resolve_vfo(rig, vfo, NULL, &vfonum);
  if (retval != RIG_OK) {
    return retval;
  }

  snprintf(cmdbuf, sizeof(cmdbuf), "FO %1d", vfonum);
  retval = kenwood_safe_transaction(rig, cmdbuf, buf, sizeof(buf), 48);
  if (retval != RIG_OK) {
    return retval;
  }

  retval = num_sscanf(buf, "FO %x,%"SCNfreq",%x,%x,%x,%x,%x,%x,%d,%d,%d,%d,%d",
      &fo_struct->vfo, &fo_struct->freq,
      &fo_struct->step, &fo_struct->shift,
      &fo_struct->reverse, &fo_struct->tone,
      &fo_struct->ct, &fo_struct->dcs,
      &fo_struct->tone_freq, &fo_struct->ct_freq,
      &fo_struct->dcs_val, &fo_struct->offset,
      &fo_struct->mode);
  if (retval != 13) {
    rig_debug(RIG_DEBUG_ERR, "%s: Unexpected reply '%s'\n", __func__, buf);
    return -RIG_ERJCTED;
  }

  return RIG_OK;
}

int tmv71_push_fo(RIG *rig, vfo_t vfo, tmv71_fo *fo_struct)
{
  char cmdbuf[80];
  char buf[80];
  int retval;

  rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

  snprintf(cmdbuf, sizeof(cmdbuf), "FO %1d,%010.0f,%1d,%1d,%1d,%1d,%1d,%1d,%02d,%02d,%03d,%08d,%1d",
      fo_struct->vfo, fo_struct->freq,
      fo_struct->step, fo_struct->shift,
      fo_struct->reverse, fo_struct->tone,
      fo_struct->ct, fo_struct->dcs,
      fo_struct->tone_freq, fo_struct->ct_freq,
      fo_struct->dcs_val, fo_struct->offset,
      fo_struct->mode);

  retval = kenwood_safe_transaction(rig, cmdbuf, buf, sizeof(buf), 48);
  if (retval != RIG_OK) {
    return retval;
  }

  retval = num_sscanf(buf, "FO %x,%"SCNfreq",%x,%x,%x,%x,%x,%x,%d,%d,%d,%d,%d",
      &fo_struct->vfo, &fo_struct->freq,
      &fo_struct->step, &fo_struct->shift,
      &fo_struct->reverse, &fo_struct->tone,
      &fo_struct->ct, &fo_struct->dcs,
      &fo_struct->tone_freq, &fo_struct->ct_freq,
      &fo_struct->dcs_val, &fo_struct->offset,
      &fo_struct->mode);
  if (retval != 13) {
    rig_debug(RIG_DEBUG_ERR, "%s: Unexpected reply '%s'\n", __func__, buf);
    return -RIG_ERJCTED;
  }

  return RIG_OK;
}

int tmv71_scan_mu(char *buf, tmv71_mu *mu_struct) {
  int retval;

  retval = num_sscanf(buf,
      "MU %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
         "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,"
         "%d,%d,%d,%d,%d,%d,%d,%d,%X,%X,"
         "%X,%X,%X,%X,%d,%d,%d,%d,%d,%d,"
         "%d,%d",
      &mu_struct->beep,
      &mu_struct->beep_volume,
      &mu_struct->ext_speaker_mode,
      &mu_struct->announce,
      &mu_struct->language,
      &mu_struct->voice_volume,
      &mu_struct->voice_speed,
      &mu_struct->playback_repeat,
      &mu_struct->playback_repeat_interval,
      &mu_struct->continuous_recording,
      &mu_struct->vhf_aip,
      &mu_struct->uhf_aip,
      &mu_struct->smeter_sql_hang_up_time,
      &mu_struct->mute_hang_up_time,
      &mu_struct->beat_shift,
      &mu_struct->timeout_timer,
      &mu_struct->recall_method,
      &mu_struct->echolink_speed,
      &mu_struct->dtmf_hold,
      &mu_struct->dtmf_speed,
      &mu_struct->dtmf_pause,
      &mu_struct->dtmf_key_lock,
      &mu_struct->auto_repeater_offset,
      &mu_struct->tone_1750_tx_hold,
      &mu_struct->p25_unknown,
      &mu_struct->brightness_level,
      &mu_struct->auto_brightness,
      &mu_struct->backlight_color,
      &mu_struct->pf1_key,
      &mu_struct->pf2_key,
      &mu_struct->mic_pf1_key,
      &mu_struct->mic_pf2_key,
      &mu_struct->mic_pf3_key,
      &mu_struct->mic_pf4_key,
      &mu_struct->mic_key_lock,
      &mu_struct->scan_resume,
      &mu_struct->auto_power_off,
      &mu_struct->ext_data_band,
      &mu_struct->ext_data_speed,
      &mu_struct->sqc_source,
      &mu_struct->auto_pm_store,
      &mu_struct->display_partition_bar
  );

  if (retval != 42) {
    rig_debug(RIG_DEBUG_ERR, "%s: Unexpected reply '%s'\n", __func__, buf);
    return -RIG_ERJCTED;
  }

  return RIG_OK;
}

int tmv71_pull_mu(RIG *rig, tmv71_mu *mu_struct)
{
  char buf[128];
  int retval;

  rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

  retval = kenwood_transaction(rig, "MU", buf, sizeof(buf));
  if (retval != RIG_OK) {
    return retval;
  }

  retval = tmv71_scan_mu(buf, mu_struct);
  if (retval != RIG_OK) {
    return retval;
  }

  return RIG_OK;
}

int tmv71_push_mu(RIG *rig, tmv71_mu *mu_struct)
{
  char cmdbuf[128];
  char buf[128];
  int retval;

  rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

  // we re-use fo_struct->vfo for the channel#
  snprintf(cmdbuf, sizeof(cmdbuf),
      "MU %1d,%1d,%1d,%1d,%1d,%1d,%1d,%1d,%02d,%1d,"
          "%1d,%1d,%1d,%1d,%1d,%1d,%1d,%1d,%1d,%1d,"
          "%1d,%1d,%1d,%1d,%1d,%1d,%1d,%1d,%02X,%02X,"
          "%02X,%02X,%02X,%02X,%1d,%1d,%1d,%1d,%1d,%1d,"
          "%1d,%1d",
      mu_struct->beep,
      mu_struct->beep_volume,
      mu_struct->ext_speaker_mode,
      mu_struct->announce,
      mu_struct->language,
      mu_struct->voice_volume,
      mu_struct->voice_speed,
      mu_struct->playback_repeat,
      mu_struct->playback_repeat_interval,
      mu_struct->continuous_recording,
      mu_struct->vhf_aip,
      mu_struct->uhf_aip,
      mu_struct->smeter_sql_hang_up_time,
      mu_struct->mute_hang_up_time,
      mu_struct->beat_shift,
      mu_struct->timeout_timer,
      mu_struct->recall_method,
      mu_struct->echolink_speed,
      mu_struct->dtmf_hold,
      mu_struct->dtmf_speed,
      mu_struct->dtmf_pause,
      mu_struct->dtmf_key_lock,
      mu_struct->auto_repeater_offset,
      mu_struct->tone_1750_tx_hold,
      mu_struct->p25_unknown,
      mu_struct->brightness_level,
      mu_struct->auto_brightness,
      mu_struct->backlight_color,
      mu_struct->pf1_key,
      mu_struct->pf2_key,
      mu_struct->mic_pf1_key,
      mu_struct->mic_pf2_key,
      mu_struct->mic_pf3_key,
      mu_struct->mic_pf4_key,
      mu_struct->mic_key_lock,
      mu_struct->scan_resume,
      mu_struct->auto_power_off,
      mu_struct->ext_data_band,
      mu_struct->ext_data_speed,
      mu_struct->sqc_source,
      mu_struct->auto_pm_store,
      mu_struct->display_partition_bar
    );

  retval = kenwood_transaction(rig, cmdbuf, buf, sizeof(buf));
  if (retval != RIG_OK) {
    return retval;
  }

  retval = tmv71_scan_mu(buf, mu_struct);
  if (retval != RIG_OK) {
    return retval;
  }

  return RIG_OK;
}

/*
 * tmv71_resolve_freq
 * 
 * Common function for converting a frequency into the closes match which
 * the radio supports.
 */
struct StepFreq tmv71_resolve_supported_freq( int freq ){
  long freq5, freq625, resolvedFreq;
  int step;
  
  freq5 = round(freq / 5000) * 5000;
  freq625 = round(freq / 6250) * 6250;
  
  if (fabs(freq5 - freq) < fabs(freq625 - freq)) {
    step = 0;
    resolvedFreq = freq5;
  } else {
    step = 1;
    resolvedFreq = freq625;
  }

  struct StepFreq result;

  /* Step needs to be at least 10kHz on higher band, otherwise 5 kHz */
  result.step = resolvedFreq >= MHz(470) ? 4 : step;
  result.frequency = resolvedFreq >= MHz(470) ? (round(resolvedFreq / 10000) * 10000) : resolvedFreq;

  return result;
}

/*
 * tmv71_set_freq
 * Assumes rig!=NULL
 * Common function for getting the main and split frequency.
 */
int tmv71_do_set_freq(RIG *rig, int channel, freq_t freq)
{
  rig_debug(RIG_DEBUG_TRACE, "%s: called for channel %d with freq %f\n", 
    __func__, channel, freq);

  tmv71_me me_struct;
  int retval = tmv71_pull_me(rig, channel, &me_struct);
  if (retval != RIG_OK) {
    return retval;
  }

  struct StepFreq sf = tmv71_resolve_supported_freq(freq);

  me_struct.channel = channel;
  me_struct.step = sf.step;
  me_struct.freq = sf.frequency;
  me_struct.tx_freq = sf.frequency;

  return tmv71_push_me(rig, &me_struct);
}

/*
 * tmv71_do_get_freq
 * Assumes rig!=NULL, freq!=NULL
 * Common function for getting the main and split frequency.
 */
int tmv71_do_get_freq(RIG *rig, int channel, freq_t *freq)
{
  rig_debug(RIG_DEBUG_TRACE, "%s: called for channel: %d)\n", __func__, channel);

  tmv71_me me_struct;
  int retval = tmv71_pull_me(rig, channel, &me_struct);

  if (retval == RIG_OK) {
    *freq = me_struct.freq;
  }

  return retval;
}

/*
 * tmv71_set_freq
 * Assumes rig!=NULL, freq!=NULL
 */
int tmv71_set_freq(RIG *rig, vfo_t vfo, freq_t freq) {

	rig_debug(RIG_DEBUG_TRACE, "%s: called for vfo: %s(%d)\n", __func__, rig_strvfo(vfo), vfo);

	return tmv71_do_set_freq(rig, tmv71_VFO_A_CHANNEL, freq);
}

/*
 * tmv71_get_freq
 * Assumes rig!=NULL, freq!=NULL
 */
int tmv71_get_freq(RIG *rig, vfo_t vfo, freq_t *freq) {
	 
	rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

	return tmv71_do_get_freq(rig, tmv71_VFO_A_CHANNEL, freq);
}

/*
 * tmv71_set_split_freq
 * Assumes rig!=NULL, freq!=NULL
 */
int tmv71_set_split_freq(RIG *rig, vfo_t vfo, freq_t freq) {

	rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);
	
	return tmv71_do_set_freq(rig, tmv71_VFO_B_CHANNEL, freq);
}

/*
 * tmv71_get_split_freq
 * Assumes rig!=NULL, freq!=NULL
 */
int tmv71_get_split_freq(RIG *rig, vfo_t vfo, freq_t *freq) {
	 
	rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

	return tmv71_do_get_freq(rig, tmv71_VFO_B_CHANNEL, freq);
}

static int tmv71_find_ctcss_index(RIG *rig, tone_t tone, int *ctcss_index) {
  int k, stepind = -1;
  for (k = 0; k < 42; k++) {
    if (rig->caps->ctcss_list[k] == tone) {
      stepind = k;
      break;
    }
  }
  if (stepind == -1) {
    rig_debug(RIG_DEBUG_ERR, "%s: Unsupported tone value '%d'\n", __func__, tone);
    return -RIG_EINVAL;
  }

  *ctcss_index = stepind;

  return RIG_OK;
}

/*
 * tmv71_set_ctcss_tone
 * Assumes rig!=NULL, freq!=NULL
 */
static int tmv71_set_ctcss_tone(RIG *rig, vfo_t vfo, tone_t tone)
{
  int retval, stepind;
  tmv71_fo fo_struct;

  rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

  retval = tmv71_find_ctcss_index(rig, tone, &stepind);
  if (retval != RIG_OK) {
    return retval;
  }

  retval = tmv71_pull_fo(rig, vfo, &fo_struct);
  if (retval != RIG_OK) {
    return retval;
  }

  fo_struct.tone_freq = stepind;

  return tmv71_push_fo(rig, vfo, &fo_struct);
}

/*
 * tmv71_get_ctcss_tone
 * Assumes rig!=NULL, freq!=NULL
 */
int tmv71_get_ctcss_tone(RIG *rig, vfo_t vfo, tone_t *tone)
{
  tmv71_fo fo_struct;
  int retval;

  const struct rig_caps *caps;
  caps = rig->caps;

  rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

  retval = tmv71_pull_fo(rig, vfo, &fo_struct);

  if (retval == RIG_OK) {
    *tone = caps->ctcss_list[fo_struct.tone_freq];
  }

  return retval;
}

/*
 * tmv71_set_ctcss_sql
 * Assumes rig!=NULL, freq!=NULL
 */
static int tmv71_set_ctcss_sql(RIG *rig, vfo_t vfo, tone_t tone)
{
  int retval, stepind;
  tmv71_fo fo_struct;

  rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

  retval = tmv71_find_ctcss_index(rig, tone, &stepind);
  if (retval != RIG_OK) {
    return retval;
  }

  retval = tmv71_pull_fo(rig, vfo, &fo_struct);
  if (retval != RIG_OK) {
    return retval;
  }

  fo_struct.ct_freq = stepind;

  return tmv71_push_fo(rig, vfo, &fo_struct);
}

/*
 * tmv71_get_ctcss_sql
 * Assumes rig!=NULL, freq!=NULL
 */
int tmv71_get_ctcss_sql(RIG *rig, vfo_t vfo, tone_t *tone)
{
  tmv71_fo fo_struct;
  int retval;

  const struct rig_caps *caps;
  caps = rig->caps;

  rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

  retval = tmv71_pull_fo(rig, vfo, &fo_struct);

  if (retval == RIG_OK) {
    *tone = caps->ctcss_list[fo_struct.ct_freq];
  }

  return retval;
}

/*
 *      status: ok, no vfo checks
 */
int tmv71_get_dcs_sql(RIG *rig, vfo_t vfo, tone_t *code)
{
  int retval;
  tmv71_fo fo_struct;

  if (!rig || !code) {
    return -RIG_EINVAL;
  }

  retval = tmv71_pull_fo(rig, vfo, &fo_struct);
  if (retval != RIG_OK) {
    return retval;
  }

  if (fo_struct.dcs) {
    tone_t *dcs_list = common_dcs_list;
    *code = dcs_list[fo_struct.dcs_val];
  } else {
    *code = 0;
  }

  return RIG_OK;
}

static int tmv71_find_dcs_index(tone_t code, int *dcs_index) {
  int i = 0;

  // we only allow exact matches here
  tone_t *dcs_list = common_dcs_list;
  while (code != dcs_list[i]) {
    if (dcs_list[i] == 0) {
      return -RIG_EINVAL;
    }
    i++;
  }

  *dcs_index = i;

  return RIG_OK;
}

/*
 *      status: ok, no vfo checks
 */
int tmv71_set_dcs_sql(RIG *rig, vfo_t vfo, tone_t code)
{
  int retval, dcs_index, dcs_enable;
  tmv71_fo fo_struct;

  if (code == 0) {
    dcs_index = 0;
    dcs_enable = 0;
  } else {
    retval = tmv71_find_dcs_index(code, &dcs_index);
    if (retval != RIG_OK) {
      return retval;
    }
    dcs_enable = 1;
  }

  retval = tmv71_pull_fo(rig, vfo, &fo_struct);
  if (retval != RIG_OK) {
    return retval;
  }

  fo_struct.dcs = dcs_enable;
  fo_struct.dcs_val = dcs_index;

  return tmv71_push_fo(rig, vfo, &fo_struct);
}

static int tmv71_get_mode_hamlib_values(int tmv71_mode, rmode_t *mode, pbwidth_t *width) {
  switch (tmv71_mode) {
    case 0:
      *mode = RIG_MODE_FM;
      *width = 15000;
      break;
    case 1:
      *mode = RIG_MODE_FMN;
      *width = 5000;
      break;
    case 2:
      *mode = RIG_MODE_AM;
      *width = 4000;
      break;
    default:
      rig_debug(RIG_DEBUG_ERR, "%s: Illegal value from radio '%ld'\n", __func__, (long)tmv71_mode);
      return -RIG_EINVAL;
  }

  return RIG_OK;
}

/*
 * tmv71_get_mode
 * Assumes rig!=NULL, freq!=NULL
 */
int tmv71_get_mode(RIG *rig, vfo_t vfo, rmode_t *mode, pbwidth_t *width)
{
  tmv71_fo fo_struct;
  int retval;

  rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

  retval = tmv71_pull_fo(rig, vfo, &fo_struct);
  if (retval != RIG_OK) {
    return retval;
  }

  retval = tmv71_get_mode_hamlib_values(fo_struct.mode, mode, width);
  if (retval != RIG_OK) {
    return retval;
  }

  return RIG_OK;
}

static int tmv71_get_mode_tmv71_value(rmode_t mode, int *tmv71_mode) {
  if (mode == RIG_MODE_FM) {
    *tmv71_mode = 0;
  } else if (mode == RIG_MODE_FMN) {
    *tmv71_mode = 1;
  } else if (mode == RIG_MODE_AM) {
    *tmv71_mode = 2;
  } else {
    rig_debug(RIG_DEBUG_ERR, "%s: Illegal value from radio '%ld'\n", __func__, (long)mode);
    return -RIG_EINVAL;
  }

  return RIG_OK;
}

/*
 * tmv71_set_mode
 * Assumes rig!=NULL, freq!=NULL
 */
static int tmv71_set_mode(RIG *rig, vfo_t vfo, rmode_t mode, pbwidth_t width)
{
  int retval, tmv71_mode = RIG_MODE_NONE;
  tmv71_fo fo_struct;

  rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

  retval = tmv71_get_mode_tmv71_value(mode, &tmv71_mode);
  if (retval != RIG_OK) {
    return retval;
  }

  retval = tmv71_pull_fo(rig, vfo, &fo_struct);
  if (retval != RIG_OK) {
    return retval;
  }

  fo_struct.mode = tmv71_mode;

  return tmv71_push_fo(rig, vfo, &fo_struct);
}

static int tmv71_find_tuning_step_index(RIG *rig, shortfreq_t ts, int *step_index) {
  int k, stepind = -1;

  for (k = 0; rig->state.tuning_steps[k].ts != 0; ++k) {

    if ((rig->caps->tuning_steps[k].modes == RIG_MODE_NONE)
        && (rig->caps->tuning_steps[k].ts == 0))
      break;
    else if (rig->caps->tuning_steps[k].ts == ts) {
      stepind = k;
      break;
    }
  }
  if (stepind == -1) {
    rig_debug(RIG_DEBUG_ERR, "%s: Unsupported tuning step value '%ld'\n", __func__, ts);
    return -RIG_EINVAL;
  }

  *step_index = stepind;

  return RIG_OK;
}

/*
 * tmv71_set_ts
 * Assumes rig!=NULL, freq!=NULL
 */
static int tmv71_set_ts(RIG *rig, vfo_t vfo, shortfreq_t ts)
{
  int retval, stepind;
  tmv71_fo fo_struct;

  rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

  retval = tmv71_find_tuning_step_index(rig, ts, &stepind);
  if (retval != RIG_OK) {
    return retval;
  }

  retval = tmv71_pull_fo(rig, vfo, &fo_struct);
  if (retval != RIG_OK) {
    return retval;
  }

  fo_struct.step = stepind;

  return tmv71_push_fo(rig, vfo, &fo_struct);
}

/*
 * tmv71_get_ts
 * Assumes rig!=NULL, freq!=NULL
 */
static int tmv71_get_ts(RIG *rig, vfo_t vfo, shortfreq_t* ts)
{
  int retval;
  tmv71_fo fo_struct;
  rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

  retval = tmv71_pull_fo(rig, vfo, &fo_struct);

  if (retval == RIG_OK) {
    *ts = rig->caps->tuning_steps[fo_struct.step].ts;
  }

  return retval;
}

int tmv71_get_rptr_shift_tmv71_value(rptr_shift_t shift, int *tmv71_shift) {
  switch (shift) {
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

/*
 * tmv71_set_rptr_shift
 * Assumes rig!=NULL, freq!=NULL
 */
int tmv71_set_rptr_shift(RIG *rig, vfo_t vfo, rptr_shift_t shift)
{
  int retval;
  tmv71_fo fo_struct;

  rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

  retval = tmv71_pull_fo(rig, vfo, &fo_struct);
  if (retval != RIG_OK) {
    return retval;
  }

  retval = tmv71_get_rptr_shift_tmv71_value(shift, &fo_struct.shift);
  if (retval != RIG_OK) {
    return retval;
  }

  return tmv71_push_fo(rig, vfo, &fo_struct);
}

int tmv71_get_rptr_shift_hamlib_value(int tmv71_shift, rptr_shift_t *shift) {
  switch (tmv71_shift) {
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
      rig_debug(RIG_DEBUG_ERR, "%s: Unexpected shift value '%d'\n", __func__, tmv71_shift);
      return -RIG_EPROTO;
  }

  return RIG_OK;
}

/*
 * tmv71_get_rptr_shft
 * Assumes rig!=NULL, freq!=NULL
 */
int tmv71_get_rptr_shift(RIG *rig, vfo_t vfo, rptr_shift_t *shift)
{
  int retval;
  tmv71_fo fo_struct;

  rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

  retval = tmv71_pull_fo(rig, vfo, &fo_struct);
  if (retval != RIG_OK) {
    return retval;
  }

  retval = tmv71_get_rptr_shift_hamlib_value(fo_struct.shift, shift);
  if (retval != RIG_OK) {
    return retval;
  }

  return RIG_OK;
}

/*
 * tmv71_set_rptr_offs
 * Assumes rig!=NULL
 */
int tmv71_set_rptr_offs(RIG *rig, vfo_t vfo, shortfreq_t freq)
{
  int retval;
  tmv71_fo fo_struct;
  long freq5, freq625, freq_sent;

  rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

  retval = tmv71_pull_fo(rig, vfo, &fo_struct);
  if (retval != RIG_OK) {
    return retval;
  }

  freq5 = round(freq / 5000) * 5000;
  freq625 = round(freq / 6250) * 6250;
  if (abs((int) (freq5 - freq)) < abs((int) (freq625 - freq))) {
    freq_sent = freq5;
  } else {
    freq_sent = freq625;
  }
  /* Step needs to be at least 10kHz on higher band, otherwise 5 kHz */
  fo_struct.offset = freq_sent >= MHz(470) ? (round(freq_sent / 10000) * 10000) : freq_sent;

  return tmv71_push_fo(rig, vfo, &fo_struct);
}



/*
 * tmv71_get_rptr_offs
 * Assumes rig!=NULL, freq!=NULL
 */
int tmv71_get_rptr_offs(RIG *rig, vfo_t vfo, shortfreq_t *rptr_offs)
{
  tmv71_fo fo_struct;
  int retval;

  rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

  retval = tmv71_pull_fo(rig, vfo, &fo_struct);

  if (retval == RIG_OK)
    *rptr_offs = fo_struct.offset;


  return retval;
}

/*
 * tmv71_get_vfo
 * Assumes rig!=NULL
 */
int tmv71_get_vfo(RIG *rig, vfo_t *vfo)
{
  int vfomode;
  int retval;

  rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

  retval = tmv71_get_vfo_and_mode(rig, vfo, &vfomode);
  if (retval != RIG_OK) {
    return retval;
  }

  switch (vfomode) {
    case tmv71_BAND_MODE_VFO:
      break;
    case tmv71_BAND_MODE_MEMORY:
    case tmv71_BAND_MODE_CALL:
      *vfo = RIG_VFO_MEM;
      break;
    default:
      rig_debug(RIG_DEBUG_ERR, "%s: Unexpected VFO mode value '%c'\n", __func__, vfomode);
      return -RIG_EVFO;
  }

  return RIG_OK;
}

/*
 * tmv71_set_vfo
 *
 * Assumes rig!=NULL
 */
int tmv71_set_vfo(RIG *rig, vfo_t vfo)
{
  char vfobuf[16], ackbuf[16];
  int vfonum;
  int channel = -1;
  int retval;

  rig_debug(RIG_DEBUG_TRACE, "%s: called %s\n", __func__, rig_strvfo(vfo));

  switch (vfo) {
    case RIG_VFO_A:
    case RIG_VFO_VFO:
      vfonum = tmv71_BAND_A;
      channel = tmv71_VFO_A_CHANNEL;
      break;
    case RIG_VFO_B:
      vfonum = tmv71_BAND_B;
      channel = tmv71_VFO_B_CHANNEL;
      break;
    case RIG_VFO_MEM:
      //Rig is always in memory mode.
      break;
    default:
      rig_debug(RIG_DEBUG_ERR, "%s: Unsupported VFO %d\n", __func__, vfo);
      return -RIG_EVFO;
  }

  //Set to memory mode, with the psudo-VFO channel if required.
  snprintf(vfobuf, sizeof(vfobuf), "VM %1d,%1d", vfonum, tmv71_BAND_MODE_MEMORY);
  retval = kenwood_transaction(rig, vfobuf, ackbuf, sizeof(ackbuf));
  if (retval != RIG_OK) return retval;
  
  //Set PTT & Control to the VFO
  snprintf(vfobuf, sizeof(vfobuf), "BC %1d,%1d", vfonum, vfonum);
  retval = kenwood_transaction(rig, vfobuf, ackbuf, sizeof(ackbuf));
  if (retval != RIG_OK) return retval;

  if( channel < 0 ){
    return RIG_OK;
  }

  retval = tmv71_set_vfo_channel(rig, vfonum, channel);
  if (retval != RIG_OK) return retval;

  return RIG_OK;
}

/*
*	tmv71_set_split_vfo
*
*	This radio has two VFOs, and either one can be the TX/RX.  As such, this function does two things:
*	- Sets PTT control on the specified VFO.
*	- Sets the TX_VFO and RX_VFO for use in Set_Freq and Set_Split_Freq
*	- The value of split is ignored, as the radio is always in "split" mode.
*
*/
int tmv71_set_split_vfo (RIG *rig, vfo_t vfo, split_t split, vfo_t txvfo)
{
    char vfobuf[16], ackbuf[16];
    int retval;

    rig_debug(RIG_DEBUG_TRACE, "%s: called vfo: %s\ttxvfo: %s\n", __func__, rig_strvfo(vfo), rig_strvfo(txvfo));
	
	rig->state.tx_vfo = txvfo;

	int txVfoIndex = txvfo == RIG_VFO_A ? 0 : 1;

	sprintf(vfobuf, "BC %d,%d", txVfoIndex, txVfoIndex );
    retval = kenwood_transaction(rig, vfobuf, ackbuf, sizeof(ackbuf));
	if (retval != RIG_OK)
        return retval;

    
    return RIG_OK;
}

int tmv71_get_split_vfo (RIG *rig, vfo_t vfo, split_t *split, vfo_t *txvfo)
{
    char buf[10];
    int retval;

    rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

	/* Get VFO band */

	retval = kenwood_safe_transaction(rig, "BC", buf, 10, 6);
	if (retval != RIG_OK)
		return retval;

	switch (buf[5]) {
	   	case '0': *txvfo = RIG_VFO_A; break;
		case '1': *txvfo = RIG_VFO_B; break;
		default:
			rig_debug(RIG_DEBUG_ERR, "%s: Unexpected txVFO value '%c'\n", __func__, buf[5]);
			return -RIG_EPROTO;
	}

    rig->state.tx_vfo = *txvfo;

	return RIG_OK;
}

int tmv71_get_mem(RIG *rig, vfo_t vfo, int *ch)
{
  char cmd[16];
  char membuf[16];
  int retval;
  int vfonum;
  int n;

  rig_debug(RIG_DEBUG_VERBOSE, "%s called\n", __func__);

  if (!rig || !ch) {
    return -RIG_EINVAL;
  }

  if (RIG_VFO_CURR == vfo || RIG_VFO_VFO == vfo) {
    retval = tmv71_get_vfo_num(rig, &vfonum, NULL);
    if (retval != RIG_OK) {
      return retval;
    }
  }

  snprintf(cmd, sizeof(cmd), "MR %d", vfonum);
  retval = kenwood_safe_transaction(rig, cmd, membuf, sizeof(membuf), 8);
  if (retval != RIG_OK) {
    return retval;
  }

  n = sscanf(membuf, "MR %*d,%d", ch);
  if (n != 1) {
    rig_debug(RIG_DEBUG_ERR, "Unable to parse '%s', expected 'MR v,ccc'\n", membuf);
    return -RIG_EPROTO;
  }

  return RIG_OK;
}

int tmv71_set_mem(RIG *rig, vfo_t vfo, int ch)
{
  int retval, vfonum;
  char cmd[16];
  char membuf[16];

  rig_debug(RIG_DEBUG_VERBOSE, "%s called\n", __func__);

  if (!rig) {
    return -RIG_EINVAL;
  }

  if (RIG_VFO_CURR == vfo || RIG_VFO_VFO == vfo) {
    retval = tmv71_get_vfo_num(rig, &vfonum, NULL);
    if (retval != RIG_OK) {
      return retval;
    }
  }

  snprintf(cmd, sizeof(cmd), "MR %d,%03d", vfonum, ch);

  return kenwood_safe_transaction(rig, cmd, membuf, sizeof(membuf), 8);
}

int tmv71_get_channel(RIG *rig, vfo_t vfo, channel_t *chan, int read_only)
{
  int retval;
  tmv71_me me_struct;

  rig_debug(RIG_DEBUG_VERBOSE, "%s called\n", __func__);

  if (!rig || !chan) {
    return -RIG_EINVAL;
  }

  retval = tmv71_pull_me(rig, chan->channel_num, &me_struct);
  if (retval != RIG_OK) {
    return retval;
  }

  chan->freq = me_struct.freq;
  chan->vfo = RIG_VFO_CURR;

  retval = tmv71_get_mode_hamlib_values(me_struct.mode, &chan->mode, &chan->width);
  if (retval != RIG_OK) {
    return retval;
  }

  chan->tuning_step = rig->caps->tuning_steps[me_struct.step].ts;

  chan->funcs = 0;

  if (me_struct.tone != 0) {
    chan->funcs |= RIG_FUNC_TONE;
  }
  if (me_struct.ct != 0) {
    chan->funcs |= RIG_FUNC_TSQL;
  }
  if (me_struct.reverse != 0) {
    chan->funcs |= RIG_FUNC_REV;
  }

  chan->ctcss_tone = rig->caps->ctcss_list[me_struct.tone_freq];
  chan->ctcss_sql = rig->caps->ctcss_list[me_struct.ct_freq];
  chan->dcs_code = 0;
  if (me_struct.dcs) {
    tone_t *dcs_list = common_dcs_list;
    chan->dcs_sql = dcs_list[me_struct.dcs_val];
  } else {
    chan->dcs_sql = 0;
  }

  retval = tmv71_get_rptr_shift_hamlib_value(me_struct.shift, &chan->rptr_shift);
  if (retval != RIG_OK) {
    return retval;
  }

  chan->rptr_offs = me_struct.offset;

  retval = tmv71_get_memory_name(rig, chan->channel_num, chan->channel_desc);
  if (retval != RIG_OK) {
    return retval;
  }

  chan->flags = RIG_CHFLAG_NONE;

  if (me_struct.lockout) {
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

int tmv71_set_channel(RIG *rig, vfo_t vfo, const channel_t *chan)
{
  int retval;
  tmv71_me me_struct;

  rig_debug(RIG_DEBUG_VERBOSE, "%s called\n", __func__);

  if (!rig || !chan) {
    return -RIG_EINVAL;
  }

  me_struct.channel = chan->channel_num;
  me_struct.freq = chan->freq;
  me_struct.tx_freq = chan->tx_freq;

  retval = tmv71_find_tuning_step_index(rig, chan->tuning_step, &me_struct.step);
  if (retval != RIG_OK) {
    return retval;
  }

  retval = tmv71_get_rptr_shift_tmv71_value(chan->rptr_shift, &me_struct.shift);
  if (retval != RIG_OK) {
    return retval;
  }

  me_struct.offset = chan->rptr_offs;

  me_struct.reverse = (chan->funcs & RIG_FUNC_REV) ? 1 : 0;
  me_struct.tone = (chan->funcs & RIG_FUNC_TONE) ? 1 : 0;
  me_struct.ct = (chan->funcs & RIG_FUNC_TSQL) ? 1 : 0;

  if (!me_struct.tone && chan->ctcss_tone == 0) {
    me_struct.tone_freq = 0;
  } else {
    retval = tmv71_find_ctcss_index(rig, chan->ctcss_tone, &me_struct.tone_freq);
    if (retval != RIG_OK) {
      return retval;
    }
  }

  if (!me_struct.ct && chan->ctcss_sql == 0) {
    me_struct.ct_freq = 0;
  } else {
    retval = tmv71_find_ctcss_index(rig, chan->ctcss_sql, &me_struct.ct_freq);
    if (retval != RIG_OK) {
      return retval;
    }
  }

  if (chan->dcs_sql == 0) {
    me_struct.dcs = 0;
    me_struct.dcs_val = 0;
  } else {
    retval = tmv71_find_dcs_index(chan->dcs_sql, &me_struct.dcs_val);
    if (retval != RIG_OK) {
      return retval;
    }
    me_struct.dcs = 1;
  }

  me_struct.lockout = (chan->flags & RIG_CHFLAG_SKIP) ? 1 : 0;

  retval = tmv71_get_mode_tmv71_value(chan->mode, &me_struct.mode);
  if (retval != RIG_OK) {
    return retval;
  }

  me_struct.p15_unknown = 0;

  retval = tmv71_push_me(rig, &me_struct);
  if (retval != RIG_OK) {
    return retval;
  }

  return tmv71_set_memory_name(rig, me_struct.channel, (char *) chan->channel_desc);
}

int tmv71_set_ptt(RIG *rig, vfo_t vfo, ptt_t ptt)
{
  char ackbuf[32];

  rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

  return kenwood_transaction(rig, (ptt == RIG_PTT_ON) ? "TX" : "RX", ackbuf, sizeof(ackbuf));
}

int tmv71_get_dcd(RIG *rig, vfo_t vfo, dcd_t *dcd)
{
  char cmd[8], buf[8];
  int retval;
  int vfonum, dcd_val;

  retval = tmv71_resolve_vfo(rig, vfo, NULL, &vfonum);
  if (retval != RIG_OK) {
    return retval;
  }

  snprintf(cmd, sizeof(cmd), "BY %d", vfonum);

  retval = kenwood_safe_transaction(rig, cmd, buf, sizeof(buf), 6);
  if (retval != RIG_OK) {
    return retval;
  }

  retval = sscanf(buf, "BY %d,%d", &vfonum, &dcd_val);
  if (retval != 2) {
    rig_debug(RIG_DEBUG_ERR, "%s: unexpected reply '%s', len=%ld\n", __func__, buf, (long)strlen(buf));
    return -RIG_EPROTO;
  }

  switch (dcd_val) {
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

int tmv71_vfo_op(RIG *rig, vfo_t vfo, vfo_op_t op)
{
  char ackbuf[8];
  rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

  switch (op) {
    case RIG_OP_UP:
      return kenwood_transaction(rig, "UP", ackbuf, sizeof(ackbuf));
    case RIG_OP_DOWN:
      return kenwood_transaction(rig, "DW", ackbuf, sizeof(ackbuf));
    default:
      return -RIG_EINVAL;
  }
}

/*
 * tmv71_get_level
 * Assumes rig!=NULL, val!=NULL
 */
int tmv71_get_level(RIG *rig, vfo_t vfo, setting_t level, value_t *val)
{
  char buf[10], ackbuf[20];
  int retval, v, l;
  int vfonum;

  rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

  retval = tmv71_resolve_vfo(rig, vfo, NULL, &vfonum);
  if (retval != RIG_OK) {
    return retval;
  }

  switch (level) {
    case RIG_LEVEL_RFPOWER:
      snprintf(buf, sizeof(buf), "PC %d", vfonum);
      retval = kenwood_transaction(rig, buf, ackbuf, sizeof(ackbuf));
      if (retval != RIG_OK)
        return retval;

      retval = sscanf(ackbuf, "PC %d,%d", &v, &l);
      if (retval != 2 || l < 0 || l > 2) {
        rig_debug(RIG_DEBUG_ERR, "%s: Unexpected reply '%s'\n", __func__, ackbuf);
        return -RIG_ERJCTED;
      }

      /* range [0.0 ... 1.0] */
      val->f = (float) (l - tmv71_RF_POWER_MIN) / (float) (tmv71_RF_POWER_MAX - tmv71_RF_POWER_MIN);

      // RF power needs to be inverted
      val->f = 1.0f - val->f;
      break;

    case RIG_LEVEL_SQL:
      snprintf(buf, sizeof(buf), "SQ %d", vfonum);
      retval = kenwood_transaction(rig, buf, ackbuf, sizeof(ackbuf));
      if (retval != RIG_OK)
        return retval;

      retval = sscanf(ackbuf, "SQ %X", &l);
      if (retval != 1 || l < tmv71_SQL_MIN || l > tmv71_SQL_MAX) {
        rig_debug(RIG_DEBUG_ERR, "%s: Unexpected reply '%s'\n", __func__, ackbuf);
        return -RIG_ERJCTED;
      }

      /* range [0.0 ... 1.0] */
      val->f = (float) (l - tmv71_SQL_MIN) / (float) (tmv71_SQL_MAX - tmv71_SQL_MIN);
      break;

    default:
      rig_debug(RIG_DEBUG_ERR, "%s: Unsupported Level %ld\n", __func__, (long)level);
      return -RIG_EINVAL;
  }

  return RIG_OK;
}

int tmv71_set_level(RIG *rig, vfo_t vfo, setting_t level, value_t val)
{
  char buf[12], ackbuf[12];
  int vfonum;
  int retval;

  rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

  retval = tmv71_resolve_vfo(rig, vfo, NULL, &vfonum);
  if (retval != RIG_OK) {
    return retval;
  }

  switch (level) {
    case RIG_LEVEL_RFPOWER:
      // RF power needs to be inverted
      snprintf(buf, sizeof(buf), "PC %d,%d", vfonum,
          (int) ((1.0f - val.f) * (tmv71_RF_POWER_MAX - tmv71_RF_POWER_MIN) + tmv71_RF_POWER_MIN));

      return kenwood_transaction(rig, buf, ackbuf, sizeof(ackbuf));

    case RIG_LEVEL_SQL:
      snprintf(buf, sizeof(buf), "SQ %d,%02X", vfonum,
          (int) (val.f * (tmv71_SQL_MAX - tmv71_SQL_MIN) + tmv71_SQL_MIN));
      return kenwood_transaction(rig, buf, ackbuf, sizeof(ackbuf));

    default:
      rig_debug(RIG_DEBUG_ERR, "%s: Unsupported Level %ld\n", __func__, (long)level);
      return -RIG_EINVAL;
  }
}

static int tmv71_tburst(RIG *rig, int status)
{
  char ackbuf[8];
  return kenwood_transaction(rig, (status == 1) ? "TT" : "RX", ackbuf, sizeof(ackbuf));
}

/*
 * tmv71_get_kenwood_func
 * Assumes rig!=NULL, status!=NULL
 */
static int tmv71_get_kenwood_func(RIG *rig, const char *cmd, int *status)
{
  char buf[8];
  int retval, len, expected;

  rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

  len = strlen(cmd);
  expected = len + 2;

  retval = kenwood_safe_transaction(rig, cmd, buf, sizeof(buf), expected);
  if (retval != RIG_OK) {
    return retval;
  }

  if (status) {
    *status = (buf[len + 1] == '0') ? 0 : 1;
  }

  return RIG_OK;
};

/*
 * tmv71_set_kenwood_func
 * Assumes rig!=NULL, status!=NULL
 */
static int tmv71_set_kenwood_func(RIG *rig, const char *cmd, int status)
{
  char buf[16], ackbuf[16];

  rig_debug(RIG_DEBUG_TRACE, "%s: cmd = %s, status = %d\n", __func__, cmd, status);

  strncpy(buf, cmd, sizeof(buf) - 2);
  buf[sizeof(buf) - 1] = '\0';
  strncat(buf, status ? " 1" : " 0", sizeof(buf) - 1);

  return kenwood_transaction(rig, buf, ackbuf, sizeof(ackbuf));
}

/*
 * tmv71_get_func
 * Assumes rig!=NULL, status!=NULL
 */
int tmv71_get_func(RIG *rig, vfo_t vfo, setting_t func, int *status)
{
  int retval;
  int use_fo = 0, use_mu = 0;
  tmv71_fo fo_struct;
  tmv71_mu mu_struct;

  rig_debug(RIG_DEBUG_TRACE, "%s: called (0x%04lx)\n", __func__, (long)func);

  switch (func) {
    case RIG_FUNC_TONE:
    case RIG_FUNC_TSQL:
    case RIG_FUNC_REV:
      use_fo = 1;
      break;
    case RIG_FUNC_ARO:
    case RIG_FUNC_AIP:
    case RIG_FUNC_RESUME:
      use_mu = 1;
      break;
  }

  if (use_fo) {
    retval = tmv71_pull_fo(rig, vfo, &fo_struct);
    if (retval != RIG_OK) {
      return retval;
    }
  }

  if (use_mu) {
    retval = tmv71_pull_mu(rig, &mu_struct);
    if (retval != RIG_OK) {
      return retval;
    }
  }

  switch (func) {
    case RIG_FUNC_TONE:
      *status = (fo_struct.tone != 0) ? 1 : 0;
      break;
    case RIG_FUNC_TSQL:
      *status = (fo_struct.ct != 0) ? 1 : 0;
      break;
    case RIG_FUNC_REV:
      *status = (fo_struct.reverse != 0) ? 1 : 0;
      break;
    case RIG_FUNC_LOCK:
      return tmv71_get_kenwood_func(rig, "LK", status);
    case RIG_FUNC_ARO:
      *status = (mu_struct.auto_repeater_offset != 0) ? 1 : 0;
      break;
    case RIG_FUNC_AIP:
      *status = ((mu_struct.vhf_aip != 0) || (mu_struct.uhf_aip != 0)) ? 1 : 0;
      break;
    case RIG_FUNC_RESUME:
      *status = (mu_struct.scan_resume == tmv71_SCAN_RESUME_TIME) ? 1 : 0;
      break;
    default:
      rig_debug(RIG_DEBUG_ERR, "%s: Unsupported function %#lx\n", __func__, (long)func);
      return -RIG_EINVAL;
  }

  return RIG_OK;
}

/*
 * tmv71_set_func
 * Assumes rig!=NULL, status!=NULL
 */
int tmv71_set_func(RIG *rig, vfo_t vfo, setting_t func, int status)
{
  int retval;
  int use_fo = 0, use_mu = 0;
  tmv71_fo fo_struct;
  tmv71_mu mu_struct;

  rig_debug(RIG_DEBUG_TRACE, "%s: called (0x%04lx)\n", __func__, (long)func);

  switch (func) {
    case RIG_FUNC_TONE:
    case RIG_FUNC_TSQL:
    case RIG_FUNC_REV:
      use_fo = 1;
      break;
    case RIG_FUNC_ARO:
    case RIG_FUNC_AIP:
    case RIG_FUNC_RESUME:
      use_mu = 1;
      break;
  }

  if (use_fo) {
    retval = tmv71_pull_fo(rig, vfo, &fo_struct);
    if (retval != RIG_OK) {
      return retval;
    }
  }

  if (use_mu) {
    retval = tmv71_pull_mu(rig, &mu_struct);
    if (retval != RIG_OK) {
      return retval;
    }
  }

  switch (func) {
    case RIG_FUNC_TONE:
      fo_struct.tone = status ? 1 : 0;
      break;
    case RIG_FUNC_TSQL:
      fo_struct.ct = status ? 1 : 0;
      break;
    case RIG_FUNC_REV:
      fo_struct.reverse = status ? 1 : 0;
      break;
    case RIG_FUNC_ARO:
      mu_struct.auto_repeater_offset = status ? 1 : 0;
      break;
    case RIG_FUNC_AIP:
      mu_struct.vhf_aip = status ? 1 : 0;
      mu_struct.uhf_aip = status ? 1 : 0;
      break;
    case RIG_FUNC_RESUME:
      mu_struct.scan_resume = status ? tmv71_SCAN_RESUME_TIME : tmv71_SCAN_RESUME_CARRIER;
      break;
    case RIG_FUNC_LOCK:
      return tmv71_set_kenwood_func(rig, "LK", status);
    case RIG_FUNC_TBURST:
      return tmv71_tburst(rig, status);
    default:
      rig_debug(RIG_DEBUG_ERR, "%s: Unsupported function %#lx\n", __func__, (long)func);
      return -RIG_EINVAL;
  }

  if (use_fo) {
    return tmv71_push_fo(rig, vfo, &fo_struct);
  }

  if (use_mu) {
    return tmv71_push_mu(rig, &mu_struct);
  }

  return -RIG_EINVAL;
}

/*
 * tmv71_get_parm
 * Assumes rig!=NULL, status!=NULL
 */
int tmv71_get_parm(RIG *rig, setting_t parm, value_t *val)
{
  int retval;
  tmv71_mu mu_struct;

  rig_debug(RIG_DEBUG_TRACE, "%s: called (0x%04lx)\n", __func__, (long)parm);

  retval = tmv71_pull_mu(rig, &mu_struct);
  if (retval != RIG_OK) {
    return retval;
  }

  switch (parm) {
    case RIG_PARM_BEEP:
      val->i = mu_struct.beep ? 1 : 0;
      break;
    case RIG_PARM_APO:
      if (mu_struct.auto_power_off == tmv71_AUTO_POWER_OFF_180MIN) {
        val->i = 180;
      } else {
        val->i = mu_struct.auto_power_off * 30;
      }
      break;
    case RIG_PARM_BACKLIGHT:
      val->f = ((float) mu_struct.brightness_level) / 8.0f;
      break;
    default:
      rig_debug(RIG_DEBUG_ERR, "%s: Unsupported parm %#lx\n", __func__, (long)parm);
      return -RIG_EINVAL;
  }

  return RIG_OK;
}

int tmv71_set_parm(RIG *rig, setting_t parm, value_t val)
{
  int retval;
  tmv71_mu mu_struct;

  rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

  retval = tmv71_pull_mu(rig, &mu_struct);
  if (retval != RIG_OK) {
    return retval;
  }

  switch (parm) {
    case RIG_PARM_BEEP:
      mu_struct.beep = val.i ? 1 : 0;
      break;
    case RIG_PARM_BACKLIGHT:
      if (val.f < 0 || val.f > 1) {
        return -RIG_EINVAL;
      }

      mu_struct.brightness_level = (int) (val.f * 8);
      break;
    case RIG_PARM_APO: {
      int value = 0;

      if (val.i > 120) {
        value = tmv71_AUTO_POWER_OFF_180MIN;
      } else if (val.i > 90) {
        value = tmv71_AUTO_POWER_OFF_120MIN;
      } else if (val.i > 60) {
        value = tmv71_AUTO_POWER_OFF_90MIN;
      } else if (val.i > 30) {
        value = tmv71_AUTO_POWER_OFF_60MIN;
      } else if (val.i > 0) {
        value = tmv71_AUTO_POWER_OFF_30MIN;
      }

      mu_struct.auto_power_off = value;
      break;
    }
    default:
      rig_debug(RIG_DEBUG_ERR, "%s: Unsupported parm %#lx\n", __func__, (long)parm);
      return -RIG_EINVAL;
  }

  return tmv71_push_mu(rig, &mu_struct);
}

/*
 * tmv71_get_ext_level
 * Assumes rig!=NULL, rig->state.priv!=NULL, val!=NULL
 *
 */
int tmv71_get_ext_level(RIG *rig, vfo_t vfo, token_t token, value_t *val) {
  int retval;
  tmv71_mu mu_struct;

  rig_debug(RIG_DEBUG_VERBOSE, "%s called\n", __func__);

  retval = tmv71_pull_mu(rig, &mu_struct);
  if (retval != RIG_OK) {
    return retval;
  }

  switch (token) {
    case TOK_LEVEL_EXT_DATA_BAND:
      val->i = mu_struct.ext_data_band;
      break;
    default:
      rig_debug(RIG_DEBUG_ERR, "%s: Unsupported ext level %ld\n", __func__, (long)token);
      return -RIG_EINVAL;
  }

  return RIG_OK;
}

/*
 * tmv71_set_ext_level
 * Assumes rig!=NULL, rig->state.priv!=NULL
 *
 */
int tmv71_set_ext_level(RIG *rig, vfo_t vfo, token_t token, value_t val) {
  int retval;
  tmv71_mu mu_struct;

  rig_debug(RIG_DEBUG_VERBOSE, "%s called\n", __func__);

  retval = tmv71_pull_mu(rig, &mu_struct);
  if (retval != RIG_OK) {
    return retval;
  }

  switch (token) {
    case TOK_LEVEL_EXT_DATA_BAND: {
      int v = val.i;
      if (v != tmv71_EXT_DATA_BAND_A && v != tmv71_EXT_DATA_BAND_B
          && v != tmv71_EXT_DATA_BAND_TXA_RXB && v != tmv71_EXT_DATA_BAND_TXB_RXA) {
        return -RIG_EINVAL;
      }

      mu_struct.ext_data_band = v;
      break;
    }
    default:
      rig_debug(RIG_DEBUG_ERR, "%s: Unsupported ext level %ld\n", __func__, (long)token);
      return -RIG_EINVAL;
  }

  return tmv71_push_mu(rig, &mu_struct);
}

/*
Set Memory/VFO mode.
*/
int tmv71_rigSet_vfoMode(RIG *rig, tmv71_vm *vm_struct){

  rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

  char cmdbuf[80];
  char buf[80];

  snprintf(cmdbuf, sizeof(cmdbuf), "VM %1d,%1d",
           vm_struct->band, vm_struct->mode);

  return kenwood_transaction(rig, cmdbuf, buf, sizeof(buf));
}

/*
Set the channel memory
*/
int tmv71_rigSet_memoryChannel(RIG *rig, vfo_t vfo, int channel)
{

  rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

  char cmdbuf[80];
  char buf[80];

  snprintf(cmdbuf, sizeof(cmdbuf), "MR %1d,%1d",
            vfo, channel);

  return kenwood_transaction(rig, cmdbuf, buf, sizeof(buf));
}

/*
Get the channel memory
*/
int tmv71_rigGet_memoryChannel(RIG *rig, vfo_t vfo)
{

  rig_debug(RIG_DEBUG_TRACE, "%s: called\n", __func__);

  char cmdbuf[80];
  char buf[80];

  snprintf(cmdbuf, sizeof(cmdbuf), "MR %1d",
           vfo);

  return kenwood_transaction(rig, cmdbuf, buf, sizeof(buf));
}
