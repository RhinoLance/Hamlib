/*
 *  Hamlib Uniden backend - BC895 description
 *  Copyright (c) 2001-2008 by Stephane Fillod
 *
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


#include <stdlib.h>

#include <hamlib/rig.h>
#include "uniden.h"


#define BC895_MODES (RIG_MODE_AM|RIG_MODE_FM|RIG_MODE_WFM)

#define BC895_FUNC (RIG_FUNC_MUTE)

#define BC895_LEVEL_ALL (RIG_LEVEL_ATT|RIG_LEVEL_RAWSTR)

#define BC895_PARM_ALL (RIG_PARM_NONE)

#define BC895_VFO RIG_VFO_A

#define BC895_CHANNEL_CAPS \
        UNIDEN_CHANNEL_CAPS \
        .ctcss_sql=1, \
        .dcs_sql=1

/* The BC895 seems to max out at 32 while 12 seems to be about minimum. */
#define BC895_STR_CAL { 4, \
        { \
        {   0, -54 }, \
        {  12, -20 }, /* TBC */ \
        {  32,   4 }, /* TBC */ \
        { 255,  60 }, \
    } }


/*
 * bc895 rig capabilities.
 *
 * TODO: check this with manual or web site.
 */
struct rig_caps bc895_caps =
{
    RIG_MODEL(RIG_MODEL_BC895),
    .model_name = "BC895xlt",
    .mfg_name =  "Uniden",
    .version =  BACKEND_VER ".0",
    .copyright =  "LGPL",
    .status =  RIG_STATUS_ALPHA,
    .rig_type =  RIG_TYPE_TRUNKSCANNER,
    .ptt_type =  RIG_PTT_NONE,
    .dcd_type =  RIG_DCD_RIG,
    .port_type =  RIG_PORT_SERIAL,
    .serial_rate_min =  2400,
    .serial_rate_max =  9600,
    .serial_data_bits =  8,
    .serial_stop_bits =  1,
    .serial_parity =  RIG_PARITY_NONE,
    .serial_handshake =  RIG_HANDSHAKE_NONE,
    .write_delay =  0,
    .post_write_delay =  1,
    .timeout =  200,
    .retry =  3,

    .has_get_func =  BC895_FUNC,
    .has_set_func =  BC895_FUNC,
    .has_get_level =  BC895_LEVEL_ALL,
    .has_set_level =  RIG_LEVEL_SET(BC895_LEVEL_ALL),
    .has_get_parm =  BC895_PARM_ALL,
    .has_set_parm =  RIG_PARM_SET(BC895_PARM_ALL),
    .level_gran =  {},                 /* FIXME: granularity */
    .parm_gran =  {},
    .ctcss_list =  uniden_ctcss_list,
    .dcs_list =  uniden_dcs_list,
    .preamp =   { RIG_DBLST_END },
    .attenuator =   { RIG_DBLST_END },
    .max_rit =  Hz(0),
    .max_xit =  Hz(0),
    .max_ifshift =  Hz(0),
    .targetable_vfo =  0,
    .transceive =  RIG_TRN_OFF,
    .bank_qty =   10,   /* A..J */
    .chan_desc_sz =  0,
    .str_cal = BC895_STR_CAL,

    .chan_list =  {
        { 1, 300, RIG_MTYPE_MEM, {BC895_CHANNEL_CAPS} },
        RIG_CHAN_END,
    },

    .rx_range_list1 =  { RIG_FRNG_END, },    /* FIXME: enter region 1 setting */
    .tx_range_list1 =  { RIG_FRNG_END, },
    .rx_range_list2 =  {
        {MHz(29), MHz(956), BC895_MODES, -1, -1, BC895_VFO},
        RIG_FRNG_END,
    },
    .tx_range_list2 =  { RIG_FRNG_END, },
    .tuning_steps =  {
        {BC895_MODES, kHz(5)},
        {BC895_MODES, kHz(7.5)},
        {BC895_MODES, kHz(10)},
        {BC895_MODES, kHz(12.5)},
        {BC895_MODES, kHz(25)},
        {BC895_MODES, kHz(50)},
        RIG_TS_END,
        RIG_TS_END,
    },
    /* mode/filter list, remember: order matters! */
    .filters =  {
        {RIG_MODE_AM | RIG_MODE_FM, kHz(8)},
        {RIG_MODE_WFM, kHz(230)},
        RIG_FLT_END,
    },
    .priv =  NULL,

    .set_freq =  uniden_set_freq,
    .get_freq =  uniden_get_freq,
    .set_mode =  uniden_set_mode,
    .get_mode =  uniden_get_mode,
    .set_mem =  uniden_set_mem,
    .get_mem =  uniden_get_mem,
    .get_dcd =  uniden_get_dcd,
    .get_info =  uniden_get_info,
    .get_level = uniden_get_level,
    .set_level = uniden_set_level,
    .get_channel = uniden_get_channel,
    .set_channel = uniden_set_channel,

    .hamlib_check_rig_caps = HAMLIB_CHECK_RIG_CAPS
};

/*
 * Function definitions below
 */

