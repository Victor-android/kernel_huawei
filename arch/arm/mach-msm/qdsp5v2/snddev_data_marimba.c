/* Copyright (c) 2009-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/mfd/msm-adie-codec.h>
#include <linux/uaccess.h>
#include <mach/qdsp5v2/snddev_icodec.h>
#include <mach/qdsp5v2/marimba_profile.h>
#include <mach/qdsp5v2/aux_pcm.h>
#include <mach/qdsp5v2/snddev_ecodec.h>
#include <mach/qdsp5v2/audio_dev_ctl.h>
#include <mach/qdsp5v2/snddev_virtual.h>
#include <mach/board.h>
#include <asm/mach-types.h>
#include <mach/gpio.h>
#include <mach/qdsp5v2/snddev_mi2s.h>
#include <mach/qdsp5v2/mi2s.h>
#include <mach/qdsp5v2/audio_acdb_def.h>

/*< DTS2011110705477 yinzhaoyang 20111110 begin */
#include <asm-arm/huawei/smem_vendor_huawei.h>
#include "../smd_private.h"
/* DTS2011110705477 yinzhaoyang 20111110 end > */
/* define the value for BT_SCO */
#define BT_SCO_PCM_CTL_VAL (PCM_CTL__RPCM_WIDTH__LINEAR_V |\
				PCM_CTL__TPCM_WIDTH__LINEAR_V)
#define BT_SCO_DATA_FORMAT_PADDING (DATA_FORMAT_PADDING_INFO__RPCM_FORMAT_V |\
				DATA_FORMAT_PADDING_INFO__TPCM_FORMAT_V)
#define BT_SCO_AUX_CODEC_INTF   AUX_CODEC_INTF_CTL__PCMINTF_DATA_EN_V

#ifdef CONFIG_DEBUG_FS
static struct dentry *debugfs_hsed_config;
static void snddev_hsed_config_modify_setting(int type);
static void snddev_hsed_config_restore_setting(void);
#endif

static struct adie_codec_action_unit iearpiece_48KHz_osr256_actions[] =
	HANDSET_RX_48000_OSR_256;

static struct adie_codec_hwsetting_entry iearpiece_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = iearpiece_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(iearpiece_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile iearpiece_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = iearpiece_settings,
	.setting_sz = ARRAY_SIZE(iearpiece_settings),
};

static struct snddev_icodec_data snddev_iearpiece_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "handset_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_SPKR,
	.profile = &iearpiece_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
	.property = SIDE_TONE_MASK,
	.max_voice_rx_vol[VOC_NB_INDEX] = -200,
	.min_voice_rx_vol[VOC_NB_INDEX] = -1700,
	.max_voice_rx_vol[VOC_WB_INDEX] = -200,
	.min_voice_rx_vol[VOC_WB_INDEX] = -1700
};

static struct platform_device msm_iearpiece_device = {
	.name = "snddev_icodec",
	.id = 0,
	.dev = { .platform_data = &snddev_iearpiece_data },
};

/*< DTS2011010400519 dongchen 20110104 begin */
/* Add huawei devices */
#ifdef CONFIG_HUAWEI_KERNEL
/*< U8800 device begin */
static struct adie_codec_action_unit iearpiece_48KHz_osr256_actions_u8800[] =
	HANDSET_RX_48000_OSR_256_U8800;

static struct adie_codec_hwsetting_entry iearpiece_settings_u8800[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = iearpiece_48KHz_osr256_actions_u8800,
		.action_sz = ARRAY_SIZE(iearpiece_48KHz_osr256_actions_u8800),
	}
};

static struct adie_codec_dev_profile iearpiece_profile_u8800 = {
	.path_type = ADIE_CODEC_RX,
	.settings = iearpiece_settings_u8800,
	.setting_sz = ARRAY_SIZE(iearpiece_settings_u8800),
};

static struct snddev_icodec_data snddev_iearpiece_data_u8800 = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "handset_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_SPKR,
	.profile = &iearpiece_profile_u8800,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
	.property = SIDE_TONE_MASK,
	.max_voice_rx_vol[VOC_NB_INDEX] = -700,
	.min_voice_rx_vol[VOC_NB_INDEX] = -2200,
	.max_voice_rx_vol[VOC_WB_INDEX] = -700,
	.min_voice_rx_vol[VOC_WB_INDEX] = -2200
};

static struct platform_device msm_iearpiece_device_u8800 = {
	.name = "snddev_icodec",
	.id = 0,
	.dev = { .platform_data = &snddev_iearpiece_data_u8800 },
};
/* U8800 device end >*/

/*< U8820 device begin */
static struct adie_codec_action_unit iearpiece_48KHz_osr256_actions_u8820[] =
	HANDSET_RX_48000_OSR_256_U8820;

static struct adie_codec_hwsetting_entry iearpiece_settings_u8820[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = iearpiece_48KHz_osr256_actions_u8820,
		.action_sz = ARRAY_SIZE(iearpiece_48KHz_osr256_actions_u8820),
	}
};

static struct adie_codec_dev_profile iearpiece_profile_u8820 = {
	.path_type = ADIE_CODEC_RX,
	.settings = iearpiece_settings_u8820,
	.setting_sz = ARRAY_SIZE(iearpiece_settings_u8820),
};

static struct snddev_icodec_data snddev_iearpiece_data_u8820 = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "handset_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_SPKR,
	.profile = &iearpiece_profile_u8820,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
	.property = SIDE_TONE_MASK,
	.max_voice_rx_vol[VOC_NB_INDEX] = -200,
	.min_voice_rx_vol[VOC_NB_INDEX] = -1700,
	.max_voice_rx_vol[VOC_WB_INDEX] = -200,
	.min_voice_rx_vol[VOC_WB_INDEX] = -1700
};

static struct platform_device msm_iearpiece_device_u8820 = {
	.name = "snddev_icodec",
	.id = 0,
	.dev = { .platform_data = &snddev_iearpiece_data_u8820 },
};
/* U8820 device end >*/

/*< U8800-51 device begin */
static struct adie_codec_action_unit iearpiece_48KHz_osr256_actions_u8800_51[] =
	HANDSET_RX_48000_OSR_256_U8800_51;

static struct adie_codec_hwsetting_entry iearpiece_settings_u8800_51[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = iearpiece_48KHz_osr256_actions_u8800_51,
		.action_sz = ARRAY_SIZE(iearpiece_48KHz_osr256_actions_u8800_51),
	}
};

static struct adie_codec_dev_profile iearpiece_profile_u8800_51 = {
	.path_type = ADIE_CODEC_RX,
	.settings = iearpiece_settings_u8800_51,
	.setting_sz = ARRAY_SIZE(iearpiece_settings_u8800_51),
};

static struct snddev_icodec_data snddev_iearpiece_data_u8800_51 = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "handset_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_SPKR,
	.profile = &iearpiece_profile_u8800_51,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
	.property = SIDE_TONE_MASK,
	.max_voice_rx_vol[VOC_NB_INDEX] = -200,
	.min_voice_rx_vol[VOC_NB_INDEX] = -1700,
	.max_voice_rx_vol[VOC_WB_INDEX] = -200,
	.min_voice_rx_vol[VOC_WB_INDEX] = -1700
};

static struct platform_device msm_iearpiece_device_u8800_51 = {
	.name = "snddev_icodec",
	.id = 0,
	.dev = { .platform_data = &snddev_iearpiece_data_u8800_51 },
};
/* U8800-51 device end >*/

/*< DTS2011102903430 dongchen 20111029 begin */
/*< U8800-pro device begin */
static struct adie_codec_action_unit iearpiece_48KHz_osr256_actions_u8800_pro[] =
	HANDSET_RX_48000_OSR_256_U8800_PRO;

static struct adie_codec_hwsetting_entry iearpiece_settings_u8800_pro[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = iearpiece_48KHz_osr256_actions_u8800_pro,
		.action_sz = ARRAY_SIZE(iearpiece_48KHz_osr256_actions_u8800_pro),
	}
};

static struct adie_codec_dev_profile iearpiece_profile_u8800_pro = {
	.path_type = ADIE_CODEC_RX,
	.settings = iearpiece_settings_u8800_pro,
	.setting_sz = ARRAY_SIZE(iearpiece_settings_u8800_pro),
};

static struct snddev_icodec_data snddev_iearpiece_data_u8800_pro = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "handset_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_SPKR,
	.profile = &iearpiece_profile_u8800_pro,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
	.max_voice_rx_vol[VOC_NB_INDEX] = 600,
	.min_voice_rx_vol[VOC_NB_INDEX] = -900,
	.max_voice_rx_vol[VOC_WB_INDEX] = 600,
	.min_voice_rx_vol[VOC_WB_INDEX] = -900
};

static struct platform_device msm_iearpiece_device_u8800_pro = {
	.name = "snddev_icodec",
	.id = 0,
	.dev = { .platform_data = &snddev_iearpiece_data_u8800_pro },
};
/* U8800-pro device end >*/
/* DTS2011102903430 dongchen 20111029 end >*/

#endif
/* DTS2011010400519 dongchen 20110104 end >*/

/*< DTS2011021900121 dongchen 20110217 begin */
/* HAC handset */
static struct adie_codec_action_unit iearpiece_hac_48KHz_osr256_actions[] =
	HANDSET_HAC_RX_48000_OSR_256;

static struct adie_codec_hwsetting_entry iearpiece_hac_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = iearpiece_hac_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(iearpiece_hac_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile iearpiece_hac_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = iearpiece_hac_settings,
	.setting_sz = ARRAY_SIZE(iearpiece_hac_settings),
};

static struct snddev_icodec_data snddev_iearpiece_hac_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "handset_hac_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_HAC_SPKR,
	.profile = &iearpiece_hac_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
	.max_voice_rx_vol[VOC_NB_INDEX] = -200,
	.min_voice_rx_vol[VOC_NB_INDEX] = -1700,
	.max_voice_rx_vol[VOC_WB_INDEX] = -200,
	.min_voice_rx_vol[VOC_WB_INDEX] = -1700
};

static struct platform_device msm_iearpiece_hac_device = {
	.name = "snddev_icodec",
	.id = 35,
	.dev = { .platform_data = &snddev_iearpiece_hac_data },
};
/* DTS2011021900121 dongchen 20110217 end >*/

static struct adie_codec_action_unit imic_8KHz_osr256_actions[] =
	HANDSET_TX_8000_OSR_256;

static struct adie_codec_action_unit imic_16KHz_osr256_actions[] =
	HANDSET_TX_16000_OSR_256;

static struct adie_codec_action_unit imic_48KHz_osr256_actions[] =
	HANDSET_TX_48000_OSR_256;

static struct adie_codec_hwsetting_entry imic_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = imic_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(imic_8KHz_osr256_actions),
	},
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = imic_16KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(imic_16KHz_osr256_actions),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = imic_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(imic_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile imic_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = imic_settings,
	.setting_sz = ARRAY_SIZE(imic_settings),
};

static enum hsed_controller imic_pmctl_id[] = {PM_HSED_CONTROLLER_0};

static struct snddev_icodec_data snddev_imic_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_MIC,
	.profile = &imic_profile,
	.channel_mode = 1,
	.pmctl_id = imic_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(imic_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_imic_device = {
	.name = "snddev_icodec",
	.id = 1,
	.dev = { .platform_data = &snddev_imic_data },
};

/*< DTS2011010400519 dongchen 20110104 begin */
/* Add huawei devices */
#ifdef CONFIG_HUAWEI_KERNEL
/*< U8800 device begin */
static struct adie_codec_action_unit imic_8KHz_osr256_actions_u8800[] =
	HANDSET_TX_8000_OSR_256_U8800;

static struct adie_codec_action_unit imic_16KHz_osr256_actions_u8800[] =
	HANDSET_TX_16000_OSR_256_U8800;

static struct adie_codec_action_unit imic_48KHz_osr256_actions_u8800[] =
	HANDSET_TX_48000_OSR_256_U8800;

static struct adie_codec_hwsetting_entry imic_settings_u8800[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = imic_8KHz_osr256_actions_u8800,
		.action_sz = ARRAY_SIZE(imic_8KHz_osr256_actions_u8800),
	},
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = imic_16KHz_osr256_actions_u8800,
		.action_sz = ARRAY_SIZE(imic_16KHz_osr256_actions_u8800),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = imic_48KHz_osr256_actions_u8800,
		.action_sz = ARRAY_SIZE(imic_48KHz_osr256_actions_u8800),
	}
};

static struct adie_codec_dev_profile imic_profile_u8800 = {
	.path_type = ADIE_CODEC_TX,
	.settings = imic_settings_u8800,
	.setting_sz = ARRAY_SIZE(imic_settings_u8800),
};

static enum hsed_controller imic_pmctl_id_u8800[] = {PM_HSED_CONTROLLER_0};

static struct snddev_icodec_data snddev_imic_data_u8800 = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_MIC,
	.profile = &imic_profile_u8800,
	.channel_mode = 1,
	.pmctl_id = imic_pmctl_id_u8800,
	.pmctl_id_sz = ARRAY_SIZE(imic_pmctl_id_u8800),
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_imic_device_u8800 = {
	.name = "snddev_icodec",
	.id = 1,
	.dev = { .platform_data = &snddev_imic_data_u8800 },
};
/* U8800 device end >*/

/*< U8820 device begin */
static struct adie_codec_action_unit imic_8KHz_osr256_actions_u8820[] =
	HANDSET_TX_8000_OSR_256_U8820;

static struct adie_codec_action_unit imic_16KHz_osr256_actions_u8820[] =
	HANDSET_TX_16000_OSR_256_U8820;

static struct adie_codec_action_unit imic_48KHz_osr256_actions_u8820[] =
	HANDSET_TX_48000_OSR_256_U8820;

static struct adie_codec_hwsetting_entry imic_settings_u8820[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = imic_8KHz_osr256_actions_u8820,
		.action_sz = ARRAY_SIZE(imic_8KHz_osr256_actions_u8820),
	},
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = imic_16KHz_osr256_actions_u8820,
		.action_sz = ARRAY_SIZE(imic_16KHz_osr256_actions_u8820),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = imic_48KHz_osr256_actions_u8820,
		.action_sz = ARRAY_SIZE(imic_48KHz_osr256_actions_u8820),
	}
};

static struct adie_codec_dev_profile imic_profile_u8820 = {
	.path_type = ADIE_CODEC_TX,
	.settings = imic_settings_u8820,
	.setting_sz = ARRAY_SIZE(imic_settings_u8820),
};

static enum hsed_controller imic_pmctl_id_u8820[] = {PM_HSED_CONTROLLER_0};

static struct snddev_icodec_data snddev_imic_data_u8820 = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_MIC,
	.profile = &imic_profile_u8820,
	.channel_mode = 1,
	.pmctl_id = imic_pmctl_id_u8820,
	.pmctl_id_sz = ARRAY_SIZE(imic_pmctl_id_u8820),
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_imic_device_u8820 = {
	.name = "snddev_icodec",
	.id = 1,
	.dev = { .platform_data = &snddev_imic_data_u8820 },
};
/* U8820 device end >*/

/*< U8800-51 device begin */
static struct adie_codec_action_unit imic_8KHz_osr256_actions_u8800_51[] =
	HANDSET_TX_8000_OSR_256_U8800_51;

static struct adie_codec_action_unit imic_16KHz_osr256_actions_u8800_51[] =
	HANDSET_TX_16000_OSR_256_U8800_51;

static struct adie_codec_action_unit imic_48KHz_osr256_actions_u8800_51[] =
	HANDSET_TX_48000_OSR_256_U8800_51;

static struct adie_codec_hwsetting_entry imic_settings_u8800_51[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = imic_8KHz_osr256_actions_u8800_51,
		.action_sz = ARRAY_SIZE(imic_8KHz_osr256_actions_u8800_51),
	},
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = imic_16KHz_osr256_actions_u8800_51,
		.action_sz = ARRAY_SIZE(imic_16KHz_osr256_actions_u8800_51),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = imic_48KHz_osr256_actions_u8800_51,
		.action_sz = ARRAY_SIZE(imic_48KHz_osr256_actions_u8800_51),
	}
};

static struct adie_codec_dev_profile imic_profile_u8800_51 = {
	.path_type = ADIE_CODEC_TX,
	.settings = imic_settings_u8800_51,
	.setting_sz = ARRAY_SIZE(imic_settings_u8800_51),
};

static enum hsed_controller imic_pmctl_id_u8800_51[] = {PM_HSED_CONTROLLER_0};

static struct snddev_icodec_data snddev_imic_data_u8800_51 = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_MIC,
	.profile = &imic_profile_u8800_51,
	.channel_mode = 1,
	.pmctl_id = imic_pmctl_id_u8800_51,
	.pmctl_id_sz = ARRAY_SIZE(imic_pmctl_id_u8800_51),
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_imic_device_u8800_51 = {
	.name = "snddev_icodec",
	.id = 1,
	.dev = { .platform_data = &snddev_imic_data_u8800_51 },
};
/* U8800-51 device end >*/
#endif
/* DTS2011010400519 dongchen 20110104 end >*/
static struct adie_codec_action_unit ihs_stereo_rx_48KHz_osr256_actions[] =
	HEADSET_STEREO_RX_LEGACY_48000_OSR_256;

static struct adie_codec_hwsetting_entry ihs_stereo_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_stereo_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihs_stereo_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ihs_stereo_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ihs_stereo_rx_settings,
	.setting_sz = ARRAY_SIZE(ihs_stereo_rx_settings),
};

static struct snddev_icodec_data snddev_ihs_stereo_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_stereo_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_SPKR_STEREO,
	.profile = &ihs_stereo_rx_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
	.property = SIDE_TONE_MASK,
	.max_voice_rx_vol[VOC_NB_INDEX] = -700,
	.min_voice_rx_vol[VOC_NB_INDEX] = -2200,
	.max_voice_rx_vol[VOC_WB_INDEX] = -900,
	.min_voice_rx_vol[VOC_WB_INDEX] = -2400
};

static struct platform_device msm_ihs_stereo_rx_device = {
	.name = "snddev_icodec",
	.id = 2,
	.dev = { .platform_data = &snddev_ihs_stereo_rx_data },
};

/*< DTS2011010400519 dongchen 20110104 begin */
/* Add huawei devices */
#ifdef CONFIG_HUAWEI_KERNEL
/*< U8800 device begin */
static struct adie_codec_action_unit ihs_stereo_rx_48KHz_osr256_actions_u8800[] =
	HEADSET_STEREO_RX_CAPLESS_48000_OSR_256_U8800;

static struct adie_codec_hwsetting_entry ihs_stereo_rx_settings_u8800[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_stereo_rx_48KHz_osr256_actions_u8800,
		.action_sz = ARRAY_SIZE(ihs_stereo_rx_48KHz_osr256_actions_u8800),
	}
};

static struct adie_codec_dev_profile ihs_stereo_rx_profile_u8800 = {
	.path_type = ADIE_CODEC_RX,
	.settings = ihs_stereo_rx_settings_u8800,
	.setting_sz = ARRAY_SIZE(ihs_stereo_rx_settings_u8800),
};

static struct snddev_icodec_data snddev_ihs_stereo_rx_data_u8800 = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_stereo_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_SPKR_STEREO,
	.profile = &ihs_stereo_rx_profile_u8800,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_hsed_voltage_on,
	.pamp_off = msm_snddev_hsed_voltage_off,
	.max_voice_rx_vol[VOC_NB_INDEX] = -300, 
	.min_voice_rx_vol[VOC_NB_INDEX] = -1800, 
	.max_voice_rx_vol[VOC_WB_INDEX] = -500, 
	.min_voice_rx_vol[VOC_WB_INDEX] = -2000 
};

static struct platform_device msm_ihs_stereo_rx_device_u8800 = {
	.name = "snddev_icodec",
	.id = 2,
	.dev = { .platform_data = &snddev_ihs_stereo_rx_data_u8800 },
};
/* U8800 device end >*/

/*< U8820 device begin */
static struct adie_codec_action_unit ihs_stereo_rx_48KHz_osr256_actions_u8820[] =
	HEADSET_STEREO_RX_CAPLESS_48000_OSR_256_U8820;

static struct adie_codec_hwsetting_entry ihs_stereo_rx_settings_u8820[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_stereo_rx_48KHz_osr256_actions_u8820,
		.action_sz = ARRAY_SIZE(ihs_stereo_rx_48KHz_osr256_actions_u8820),
	}
};

static struct adie_codec_dev_profile ihs_stereo_rx_profile_u8820 = {
	.path_type = ADIE_CODEC_RX,
	.settings = ihs_stereo_rx_settings_u8820,
	.setting_sz = ARRAY_SIZE(ihs_stereo_rx_settings_u8820),
};

static struct snddev_icodec_data snddev_ihs_stereo_rx_data_u8820 = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_stereo_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_SPKR_STEREO,
	.profile = &ihs_stereo_rx_profile_u8820,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_hsed_voltage_on,
	.pamp_off = msm_snddev_hsed_voltage_off,
	.max_voice_rx_vol[VOC_NB_INDEX] = -300, 
	.min_voice_rx_vol[VOC_NB_INDEX] = -1800, 
	.max_voice_rx_vol[VOC_WB_INDEX] = -500, 
	.min_voice_rx_vol[VOC_WB_INDEX] = -2000 
};

static struct platform_device msm_ihs_stereo_rx_device_u8820 = {
	.name = "snddev_icodec",
	.id = 2,
	.dev = { .platform_data = &snddev_ihs_stereo_rx_data_u8820 },
};
/* U8820 device end >*/

/*< U8800-51 device begin */
static struct adie_codec_action_unit ihs_stereo_rx_48KHz_osr256_actions_u8800_51[] =
	HEADSET_STEREO_RX_CAPLESS_48000_OSR_256_U8800_51;

static struct adie_codec_hwsetting_entry ihs_stereo_rx_settings_u8800_51[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_stereo_rx_48KHz_osr256_actions_u8800_51,
		.action_sz = ARRAY_SIZE(ihs_stereo_rx_48KHz_osr256_actions_u8800_51),
	}
};

static struct adie_codec_dev_profile ihs_stereo_rx_profile_u8800_51 = {
	.path_type = ADIE_CODEC_RX,
	.settings = ihs_stereo_rx_settings_u8800_51,
	.setting_sz = ARRAY_SIZE(ihs_stereo_rx_settings_u8800_51),
};

static struct snddev_icodec_data snddev_ihs_stereo_rx_data_u8800_51 = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_stereo_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_SPKR_STEREO,
	.profile = &ihs_stereo_rx_profile_u8800_51,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_hsed_voltage_on,
	.pamp_off = msm_snddev_hsed_voltage_off,
	.max_voice_rx_vol[VOC_NB_INDEX] = -300, 
	.min_voice_rx_vol[VOC_NB_INDEX] = -1800, 
	.max_voice_rx_vol[VOC_WB_INDEX] = -500, 
	.min_voice_rx_vol[VOC_WB_INDEX] = -2000 
};

static struct platform_device msm_ihs_stereo_rx_device_u8800_51 = {
	.name = "snddev_icodec",
	.id = 2,
	.dev = { .platform_data = &snddev_ihs_stereo_rx_data_u8800_51 },
};
/* U8800-51 device end >*/

/*< DTS2011102903430 dongchen 20111029 begin */
/*< U8800-pro device begin */
static struct adie_codec_action_unit ihs_stereo_rx_48KHz_osr256_actions_u8800_pro[] =
	HEADSET_STEREO_RX_CAPLESS_48000_OSR_256_U8800_PRO;

static struct adie_codec_hwsetting_entry ihs_stereo_rx_settings_u8800_pro[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_stereo_rx_48KHz_osr256_actions_u8800_pro,
		.action_sz = ARRAY_SIZE(ihs_stereo_rx_48KHz_osr256_actions_u8800_pro),
	}
};

static struct adie_codec_dev_profile ihs_stereo_rx_profile_u8800_pro = {
	.path_type = ADIE_CODEC_RX,
	.settings = ihs_stereo_rx_settings_u8800_pro,
	.setting_sz = ARRAY_SIZE(ihs_stereo_rx_settings_u8800_pro),
};

static struct snddev_icodec_data snddev_ihs_stereo_rx_data_u8800_pro = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_stereo_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_SPKR_STEREO,
	.profile = &ihs_stereo_rx_profile_u8800_pro,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_hsed_voltage_on,
	.pamp_off = msm_snddev_hsed_voltage_off,
	.max_voice_rx_vol[VOC_NB_INDEX] = 0, 
	.min_voice_rx_vol[VOC_NB_INDEX] = -1500, 
	.max_voice_rx_vol[VOC_WB_INDEX] = -200, 
	.min_voice_rx_vol[VOC_WB_INDEX] = -1700 
};

static struct platform_device msm_ihs_stereo_rx_device_u8800_pro = {
	.name = "snddev_icodec",
	.id = 2,
	.dev = { .platform_data = &snddev_ihs_stereo_rx_data_u8800_pro },
};
/* U8800-pro device end >*/
/* DTS2011102903430 dongchen 20111029 end >*/

#endif
/* DTS2011010400519 dongchen 20110104 end >*/
/*< DTS2011010400519 dongchen 20110104 begin */
/* Add huawei devices */
#ifdef CONFIG_HUAWEI_KERNEL
/*< U8800 device begin */
static struct adie_codec_action_unit ihs_music_stereo_rx_48KHz_osr256_actions_u8800[] =
	HEADSET_MUSIC_STEREO_RX_CAPLESS_48000_OSR_256_U8800;

static struct adie_codec_hwsetting_entry ihs_music_stereo_rx_settings_u8800[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_music_stereo_rx_48KHz_osr256_actions_u8800,
		.action_sz = ARRAY_SIZE(ihs_music_stereo_rx_48KHz_osr256_actions_u8800),
	}
};

static struct adie_codec_dev_profile ihs_music_stereo_rx_profile_u8800 = {
	.path_type = ADIE_CODEC_RX,
	.settings = ihs_music_stereo_rx_settings_u8800,
	.setting_sz = ARRAY_SIZE(ihs_music_stereo_rx_settings_u8800),
};

static struct snddev_icodec_data snddev_ihs_music_stereo_rx_data_u8800 = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_music_stereo_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_SPKR_MUSIC_STEREO,
	.profile = &ihs_music_stereo_rx_profile_u8800,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_hsed_voltage_on,
	.pamp_off = msm_snddev_hsed_voltage_off,
	.property = SIDE_TONE_MASK,
	.max_voice_rx_vol[VOC_NB_INDEX] = -300, 
	.min_voice_rx_vol[VOC_NB_INDEX] = -1800, 
	.max_voice_rx_vol[VOC_WB_INDEX] = -500, 
	.min_voice_rx_vol[VOC_WB_INDEX] = -2000 
};

static struct platform_device msm_ihs_music_stereo_rx_device_u8800 = {
	.name = "snddev_icodec",
	.id = 34,
	.dev = { .platform_data = &snddev_ihs_music_stereo_rx_data_u8800 },
};
/* U8800 device end >*/

/*< U8820 device begin */
static struct adie_codec_action_unit ihs_music_stereo_rx_48KHz_osr256_actions_u8820[] =
	HEADSET_MUSIC_STEREO_RX_CAPLESS_48000_OSR_256_U8820;

static struct adie_codec_hwsetting_entry ihs_music_stereo_rx_settings_u8820[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_music_stereo_rx_48KHz_osr256_actions_u8820,
		.action_sz = ARRAY_SIZE(ihs_music_stereo_rx_48KHz_osr256_actions_u8820),
	}
};

static struct adie_codec_dev_profile ihs_music_stereo_rx_profile_u8820 = {
	.path_type = ADIE_CODEC_RX,
	.settings = ihs_music_stereo_rx_settings_u8820,
	.setting_sz = ARRAY_SIZE(ihs_music_stereo_rx_settings_u8820),
};

static struct snddev_icodec_data snddev_ihs_music_stereo_rx_data_u8820 = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_music_stereo_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_SPKR_MUSIC_STEREO,
	.profile = &ihs_music_stereo_rx_profile_u8820,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_hsed_voltage_on,
	.pamp_off = msm_snddev_hsed_voltage_off,
	.property = SIDE_TONE_MASK,
	.max_voice_rx_vol[VOC_NB_INDEX] = -300, 
	.min_voice_rx_vol[VOC_NB_INDEX] = -1800, 
	.max_voice_rx_vol[VOC_WB_INDEX] = -500, 
	.min_voice_rx_vol[VOC_WB_INDEX] = -2000 
};

static struct platform_device msm_ihs_music_stereo_rx_device_u8820 = {
	.name = "snddev_icodec",
	.id = 34,
	.dev = { .platform_data = &snddev_ihs_music_stereo_rx_data_u8820 },
};
/* U8820 device end >*/

/*< U8800-51 device begin */
static struct adie_codec_action_unit ihs_music_stereo_rx_48KHz_osr256_actions_u8800_51[] =
	HEADSET_MUSIC_STEREO_RX_CAPLESS_48000_OSR_256_U8800_51;

static struct adie_codec_hwsetting_entry ihs_music_stereo_rx_settings_u8800_51[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_music_stereo_rx_48KHz_osr256_actions_u8800_51,
		.action_sz = ARRAY_SIZE(ihs_music_stereo_rx_48KHz_osr256_actions_u8800_51),
	}
};

static struct adie_codec_dev_profile ihs_music_stereo_rx_profile_u8800_51 = {
	.path_type = ADIE_CODEC_RX,
	.settings = ihs_music_stereo_rx_settings_u8800_51,
	.setting_sz = ARRAY_SIZE(ihs_music_stereo_rx_settings_u8800_51),
};

static struct snddev_icodec_data snddev_ihs_music_stereo_rx_data_u8800_51 = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_music_stereo_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_SPKR_MUSIC_STEREO,
	.profile = &ihs_music_stereo_rx_profile_u8800_51,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_hsed_voltage_on,
	.pamp_off = msm_snddev_hsed_voltage_off,
	.property = SIDE_TONE_MASK,
	.max_voice_rx_vol[VOC_NB_INDEX] = -300, 
	.min_voice_rx_vol[VOC_NB_INDEX] = -1800, 
	.max_voice_rx_vol[VOC_WB_INDEX] = -500, 
	.min_voice_rx_vol[VOC_WB_INDEX] = -2000 
};

static struct platform_device msm_ihs_music_stereo_rx_device_u8800_51 = {
	.name = "snddev_icodec",
	.id = 34,
	.dev = { .platform_data = &snddev_ihs_music_stereo_rx_data_u8800_51 },
};
/* U8800-51 device end >*/
#endif
/* DTS2011010400519 dongchen 20110104 end >*/

static struct adie_codec_action_unit ihs_mono_rx_48KHz_osr256_actions[] =
	HEADSET_RX_LEGACY_48000_OSR_256;

static struct adie_codec_hwsetting_entry ihs_mono_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_mono_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihs_mono_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ihs_mono_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ihs_mono_rx_settings,
	.setting_sz = ARRAY_SIZE(ihs_mono_rx_settings),
};

static struct snddev_icodec_data snddev_ihs_mono_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_mono_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_SPKR_MONO,
	.profile = &ihs_mono_rx_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
	.property = SIDE_TONE_MASK,
	.max_voice_rx_vol[VOC_NB_INDEX] = -700,
	.min_voice_rx_vol[VOC_NB_INDEX] = -2200,
	.max_voice_rx_vol[VOC_WB_INDEX] = -900,
	.min_voice_rx_vol[VOC_WB_INDEX] = -2400,

};

static struct platform_device msm_ihs_mono_rx_device = {
	.name = "snddev_icodec",
	.id = 3,
	.dev = { .platform_data = &snddev_ihs_mono_rx_data },
};

static struct adie_codec_action_unit ihs_ffa_stereo_rx_48KHz_osr256_actions[] =
	HEADSET_STEREO_RX_CAPLESS_48000_OSR_256;

static struct adie_codec_hwsetting_entry ihs_ffa_stereo_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_ffa_stereo_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihs_ffa_stereo_rx_48KHz_osr256_actions),
	}
};

#ifdef CONFIG_DEBUG_FS
static struct adie_codec_action_unit
	ihs_ffa_stereo_rx_class_d_legacy_48KHz_osr256_actions[] =
	HEADSET_STEREO_RX_CLASS_D_LEGACY_48000_OSR_256;

static struct adie_codec_hwsetting_entry
	ihs_ffa_stereo_rx_class_d_legacy_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions =
		ihs_ffa_stereo_rx_class_d_legacy_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE
		(ihs_ffa_stereo_rx_class_d_legacy_48KHz_osr256_actions),
	}
};

static struct adie_codec_action_unit
	ihs_ffa_stereo_rx_class_ab_legacy_48KHz_osr256_actions[] =
	HEADSET_STEREO_RX_LEGACY_48000_OSR_256;

static struct adie_codec_hwsetting_entry
	ihs_ffa_stereo_rx_class_ab_legacy_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions =
		ihs_ffa_stereo_rx_class_ab_legacy_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE
		(ihs_ffa_stereo_rx_class_ab_legacy_48KHz_osr256_actions),
	}
};
#endif

static struct adie_codec_dev_profile ihs_ffa_stereo_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ihs_ffa_stereo_rx_settings,
	.setting_sz = ARRAY_SIZE(ihs_ffa_stereo_rx_settings),
};

static struct snddev_icodec_data snddev_ihs_ffa_stereo_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_stereo_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_SPKR_STEREO,
	.profile = &ihs_ffa_stereo_rx_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.voltage_on = msm_snddev_hsed_voltage_on,
	.voltage_off = msm_snddev_hsed_voltage_off,
	.max_voice_rx_vol[VOC_NB_INDEX] = -700,
	.min_voice_rx_vol[VOC_NB_INDEX] = -2200,
	.max_voice_rx_vol[VOC_WB_INDEX] = -900,
	.min_voice_rx_vol[VOC_WB_INDEX] = -2400,
};

static struct platform_device msm_ihs_ffa_stereo_rx_device = {
	.name = "snddev_icodec",
	.id = 4,
	.dev = { .platform_data = &snddev_ihs_ffa_stereo_rx_data },
};

static struct adie_codec_action_unit ihs_ffa_mono_rx_48KHz_osr256_actions[] =
	HEADSET_RX_CAPLESS_48000_OSR_256;

static struct adie_codec_hwsetting_entry ihs_ffa_mono_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_ffa_mono_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihs_ffa_mono_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ihs_ffa_mono_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ihs_ffa_mono_rx_settings,
	.setting_sz = ARRAY_SIZE(ihs_ffa_mono_rx_settings),
};

static struct snddev_icodec_data snddev_ihs_ffa_mono_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_mono_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_SPKR_MONO,
	.profile = &ihs_ffa_mono_rx_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_hsed_voltage_on,
	.pamp_off = msm_snddev_hsed_voltage_off,
	.max_voice_rx_vol[VOC_NB_INDEX] = -700,
	.min_voice_rx_vol[VOC_NB_INDEX] = -2200,
	.max_voice_rx_vol[VOC_WB_INDEX] = -900,
	.min_voice_rx_vol[VOC_WB_INDEX] = -2400,
};

static struct platform_device msm_ihs_ffa_mono_rx_device = {
	.name = "snddev_icodec",
	.id = 5,
	.dev = { .platform_data = &snddev_ihs_ffa_mono_rx_data },
};

static struct adie_codec_action_unit ihs_mono_tx_8KHz_osr256_actions[] =
	HEADSET_MONO_TX_8000_OSR_256;

static struct adie_codec_action_unit ihs_mono_tx_16KHz_osr256_actions[] =
	HEADSET_MONO_TX_16000_OSR_256;

static struct adie_codec_action_unit ihs_mono_tx_48KHz_osr256_actions[] =
	HEADSET_MONO_TX_48000_OSR_256;

static struct adie_codec_hwsetting_entry ihs_mono_tx_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ihs_mono_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihs_mono_tx_8KHz_osr256_actions),
	},
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = ihs_mono_tx_16KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihs_mono_tx_16KHz_osr256_actions),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_mono_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ihs_mono_tx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ihs_mono_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = ihs_mono_tx_settings,
	.setting_sz = ARRAY_SIZE(ihs_mono_tx_settings),
};

static struct snddev_icodec_data snddev_ihs_mono_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "headset_mono_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_MIC,
	.profile = &ihs_mono_tx_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};

static struct platform_device msm_ihs_mono_tx_device = {
	.name = "snddev_icodec",
	.id = 6,
	.dev = { .platform_data = &snddev_ihs_mono_tx_data },
};

/*< DTS2011010400519 dongchen 20110104 begin */
/* Add huawei devices */
#ifdef CONFIG_HUAWEI_KERNEL
/*< U8800 device begin */
static struct adie_codec_action_unit ihs_mono_tx_8KHz_osr256_actions_u8800[] =
	HEADSET_MONO_TX_8000_OSR_256_U8800;

static struct adie_codec_action_unit ihs_mono_tx_16KHz_osr256_actions_u8800[] =
	HEADSET_MONO_TX_16000_OSR_256_U8800;

static struct adie_codec_action_unit ihs_mono_tx_48KHz_osr256_actions_u8800[] =
	HEADSET_MONO_TX_48000_OSR_256_U8800;

static struct adie_codec_hwsetting_entry ihs_mono_tx_settings_u8800[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ihs_mono_tx_8KHz_osr256_actions_u8800,
		.action_sz = ARRAY_SIZE(ihs_mono_tx_8KHz_osr256_actions_u8800),
	},
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = ihs_mono_tx_16KHz_osr256_actions_u8800,
		.action_sz = ARRAY_SIZE(ihs_mono_tx_16KHz_osr256_actions_u8800),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_mono_tx_48KHz_osr256_actions_u8800,
		.action_sz = ARRAY_SIZE(ihs_mono_tx_48KHz_osr256_actions_u8800),
	}
};

static struct adie_codec_dev_profile ihs_mono_tx_profile_u8800 = {
	.path_type = ADIE_CODEC_TX,
	.settings = ihs_mono_tx_settings_u8800,
	.setting_sz = ARRAY_SIZE(ihs_mono_tx_settings_u8800),
};

static struct snddev_icodec_data snddev_ihs_mono_tx_data_u8800 = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "headset_mono_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_MIC,
	.profile = &ihs_mono_tx_profile_u8800,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};

static struct platform_device msm_ihs_mono_tx_device_u8800 = {
	.name = "snddev_icodec",
	.id = 6,
	.dev = { .platform_data = &snddev_ihs_mono_tx_data_u8800 },
};
/* U8800 device end >*/

/*< U8820 device begin */
static struct adie_codec_action_unit ihs_mono_tx_8KHz_osr256_actions_u8820[] =
	HEADSET_MONO_TX_8000_OSR_256_U8820;

static struct adie_codec_action_unit ihs_mono_tx_16KHz_osr256_actions_u8820[] =
	HEADSET_MONO_TX_16000_OSR_256_U8820;

static struct adie_codec_action_unit ihs_mono_tx_48KHz_osr256_actions_u8820[] =
	HEADSET_MONO_TX_48000_OSR_256_U8820;

static struct adie_codec_hwsetting_entry ihs_mono_tx_settings_u8820[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ihs_mono_tx_8KHz_osr256_actions_u8820,
		.action_sz = ARRAY_SIZE(ihs_mono_tx_8KHz_osr256_actions_u8820),
	},
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = ihs_mono_tx_16KHz_osr256_actions_u8820,
		.action_sz = ARRAY_SIZE(ihs_mono_tx_16KHz_osr256_actions_u8820),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_mono_tx_48KHz_osr256_actions_u8820,
		.action_sz = ARRAY_SIZE(ihs_mono_tx_48KHz_osr256_actions_u8820),
	}
};

static struct adie_codec_dev_profile ihs_mono_tx_profile_u8820 = {
	.path_type = ADIE_CODEC_TX,
	.settings = ihs_mono_tx_settings_u8820,
	.setting_sz = ARRAY_SIZE(ihs_mono_tx_settings_u8820),
};

static struct snddev_icodec_data snddev_ihs_mono_tx_data_u8820 = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "headset_mono_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_MIC,
	.profile = &ihs_mono_tx_profile_u8820,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};

static struct platform_device msm_ihs_mono_tx_device_u8820 = {
	.name = "snddev_icodec",
	.id = 6,
	.dev = { .platform_data = &snddev_ihs_mono_tx_data_u8820 },
};
/* U8820 device end >*/

/*< U8800-51 device begin */
static struct adie_codec_action_unit ihs_mono_tx_8KHz_osr256_actions_u8800_51[] =
	HEADSET_MONO_TX_8000_OSR_256_U8800_51;

static struct adie_codec_action_unit ihs_mono_tx_16KHz_osr256_actions_u8800_51[] =
	HEADSET_MONO_TX_16000_OSR_256_U8800_51;

static struct adie_codec_action_unit ihs_mono_tx_48KHz_osr256_actions_u8800_51[] =
	HEADSET_MONO_TX_48000_OSR_256_U8800_51;

static struct adie_codec_hwsetting_entry ihs_mono_tx_settings_u8800_51[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ihs_mono_tx_8KHz_osr256_actions_u8800_51,
		.action_sz = ARRAY_SIZE(ihs_mono_tx_8KHz_osr256_actions_u8800_51),
	},
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = ihs_mono_tx_16KHz_osr256_actions_u8800_51,
		.action_sz = ARRAY_SIZE(ihs_mono_tx_16KHz_osr256_actions_u8800_51),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_mono_tx_48KHz_osr256_actions_u8800_51,
		.action_sz = ARRAY_SIZE(ihs_mono_tx_48KHz_osr256_actions_u8800_51),
	}
};

static struct adie_codec_dev_profile ihs_mono_tx_profile_u8800_51 = {
	.path_type = ADIE_CODEC_TX,
	.settings = ihs_mono_tx_settings_u8800_51,
	.setting_sz = ARRAY_SIZE(ihs_mono_tx_settings_u8800_51),
};

static struct snddev_icodec_data snddev_ihs_mono_tx_data_u8800_51 = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "headset_mono_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_MIC,
	.profile = &ihs_mono_tx_profile_u8800_51,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};

static struct platform_device msm_ihs_mono_tx_device_u8800_51 = {
	.name = "snddev_icodec",
	.id = 6,
	.dev = { .platform_data = &snddev_ihs_mono_tx_data_u8800_51 },
};
/* U8800-51 device end >*/
#endif
/* DTS2011010400519 dongchen 20110104 end >*/

static struct adie_codec_action_unit ifmradio_handset_osr64_actions[] =
	FM_HANDSET_OSR_64;

static struct adie_codec_hwsetting_entry ifmradio_handset_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ifmradio_handset_osr64_actions,
		.action_sz = ARRAY_SIZE(ifmradio_handset_osr64_actions),
	}
};

static struct adie_codec_dev_profile ifmradio_handset_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ifmradio_handset_settings,
	.setting_sz = ARRAY_SIZE(ifmradio_handset_settings),
};

static struct snddev_icodec_data snddev_ifmradio_handset_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_FM),
	.name = "fmradio_handset_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_LP_FM_SPKR_PHONE_STEREO_RX,
	.profile = &ifmradio_handset_profile,
	.channel_mode = 1,
	.default_sample_rate = 8000,
	.pamp_on = NULL,
	.pamp_off = NULL,
	.dev_vol_type = SNDDEV_DEV_VOL_DIGITAL,
};

static struct platform_device msm_ifmradio_handset_device = {
	.name = "snddev_icodec",
	.id = 7,
	.dev = { .platform_data = &snddev_ifmradio_handset_data },
};


static struct adie_codec_action_unit ispeaker_rx_48KHz_osr256_actions[] =
   SPEAKER_STEREO_RX_48000_OSR_256;

static struct adie_codec_hwsetting_entry ispeaker_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispeaker_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispeaker_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ispeaker_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ispeaker_rx_settings,
	.setting_sz = ARRAY_SIZE(ispeaker_rx_settings),
};

static struct snddev_icodec_data snddev_ispeaker_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "speaker_stereo_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_STEREO,
	.profile = &ispeaker_rx_profile,
	.channel_mode = 2,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = &msm_snddev_poweramp_on,
	.pamp_off = &msm_snddev_poweramp_off,
	.max_voice_rx_vol[VOC_NB_INDEX] = 1000,
	.min_voice_rx_vol[VOC_NB_INDEX] = -500,
	.max_voice_rx_vol[VOC_WB_INDEX] = 1000,
	.min_voice_rx_vol[VOC_WB_INDEX] = -500,
};

static struct platform_device msm_ispeaker_rx_device = {
	.name = "snddev_icodec",
	.id = 8,
	.dev = { .platform_data = &snddev_ispeaker_rx_data },

};

/*< DTS2011010400519 dongchen 20110104 begin */
/* Add huawei devices */
#ifdef CONFIG_HUAWEI_KERNEL
/*< U8800 device begin */
static struct adie_codec_action_unit ispeaker_rx_48KHz_osr256_actions_u8800[] =
   SPEAKER_RX_48000_OSR_256_U8800;

static struct adie_codec_hwsetting_entry ispeaker_rx_settings_u8800[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispeaker_rx_48KHz_osr256_actions_u8800,
		.action_sz = ARRAY_SIZE(ispeaker_rx_48KHz_osr256_actions_u8800),
	}
};

static struct adie_codec_dev_profile ispeaker_rx_profile_u8800 = {
	.path_type = ADIE_CODEC_RX,
	.settings = ispeaker_rx_settings_u8800,
	.setting_sz = ARRAY_SIZE(ispeaker_rx_settings_u8800),
};

static struct snddev_icodec_data snddev_ispeaker_rx_data_u8800 = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "speaker_stereo_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_MONO, //7,
	.profile = &ispeaker_rx_profile_u8800,
	.channel_mode = 1,//single channel for u8800
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = &msm_snddev_poweramp_on,
	.pamp_off = &msm_snddev_poweramp_off,
	.max_voice_rx_vol[VOC_NB_INDEX] = 200, 
	.min_voice_rx_vol[VOC_NB_INDEX] = -1300, 
	.max_voice_rx_vol[VOC_WB_INDEX] = 200, 
	.min_voice_rx_vol[VOC_WB_INDEX] = -1300  
};

static struct platform_device msm_ispeaker_rx_device_u8800 = {
	.name = "snddev_icodec",
	.id = 8,
	.dev = { .platform_data = &snddev_ispeaker_rx_data_u8800 },
};
/* U8800 device end >*/

/*< U8820 device begin */
/*< DTS2011012602886 dongchen 20110126 begin */
static struct adie_codec_action_unit ispeaker_rx_48KHz_osr256_actions_u8820[] =
   SPEAKER_RX_48000_OSR_256_U8820;

static struct adie_codec_hwsetting_entry ispeaker_rx_settings_u8820[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispeaker_rx_48KHz_osr256_actions_u8820,
		.action_sz = ARRAY_SIZE(ispeaker_rx_48KHz_osr256_actions_u8820),
	}
};

static struct adie_codec_dev_profile ispeaker_rx_profile_u8820 = {
	.path_type = ADIE_CODEC_RX,
	.settings = ispeaker_rx_settings_u8820,
	.setting_sz = ARRAY_SIZE(ispeaker_rx_settings_u8820),
};

static struct snddev_icodec_data snddev_ispeaker_rx_data_u8820 = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "speaker_stereo_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_MONO, //7,
	.profile = &ispeaker_rx_profile_u8820,
	.channel_mode = 1,//single channel for u8800_51
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = &msm_snddev_poweramp_on,
	.pamp_off = &msm_snddev_poweramp_off,
	.max_voice_rx_vol[VOC_NB_INDEX] = 200, 
	.min_voice_rx_vol[VOC_NB_INDEX] = -1300, 
	.max_voice_rx_vol[VOC_WB_INDEX] = 200, 
	.min_voice_rx_vol[VOC_WB_INDEX] = -1300  
};

static struct platform_device msm_ispeaker_rx_device_u8820 = {
	.name = "snddev_icodec",
	.id = 8,
	.dev = { .platform_data = &snddev_ispeaker_rx_data_u8820 },
};
/* DTS2011012602886 dongchen 20110126 end >*/
/* U8820 device end >*/

/*< U8800-51 device begin */
static struct adie_codec_action_unit ispeaker_rx_48KHz_osr256_actions_u8800_51[] =
   SPEAKER_RX_48000_OSR_256_U8800_51;

static struct adie_codec_hwsetting_entry ispeaker_rx_settings_u8800_51[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispeaker_rx_48KHz_osr256_actions_u8800_51,
		.action_sz = ARRAY_SIZE(ispeaker_rx_48KHz_osr256_actions_u8800_51),
	}
};

static struct adie_codec_dev_profile ispeaker_rx_profile_u8800_51 = {
	.path_type = ADIE_CODEC_RX,
	.settings = ispeaker_rx_settings_u8800_51,
	.setting_sz = ARRAY_SIZE(ispeaker_rx_settings_u8800_51),
};

static struct snddev_icodec_data snddev_ispeaker_rx_data_u8800_51 = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "speaker_stereo_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_MONO, //7,
	.profile = &ispeaker_rx_profile_u8800_51,
	.channel_mode = 1,//single channel for u8800_51
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = &msm_snddev_poweramp_on,
	.pamp_off = &msm_snddev_poweramp_off,
	.max_voice_rx_vol[VOC_NB_INDEX] = 200, 
	.min_voice_rx_vol[VOC_NB_INDEX] = -1300, 
	.max_voice_rx_vol[VOC_WB_INDEX] = 200, 
	.min_voice_rx_vol[VOC_WB_INDEX] = -1300  
};

static struct platform_device msm_ispeaker_rx_device_u8800_51 = {
	.name = "snddev_icodec",
	.id = 8,
	.dev = { .platform_data = &snddev_ispeaker_rx_data_u8800_51 },
};
/* U8800-51 device end >*/

/*< DTS2011102903430 dongchen 20111029 begin */
/*< U8800-pro device begin */
static struct adie_codec_action_unit ispeaker_rx_48KHz_osr256_actions_u8800_pro[] =
   SPEAKER_RX_48000_OSR_256_U8800_PRO;

static struct adie_codec_hwsetting_entry ispeaker_rx_settings_u8800_pro[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispeaker_rx_48KHz_osr256_actions_u8800_pro,
		.action_sz = ARRAY_SIZE(ispeaker_rx_48KHz_osr256_actions_u8800_pro),
	}
};

static struct adie_codec_dev_profile ispeaker_rx_profile_u8800_pro = {
	.path_type = ADIE_CODEC_RX,
	.settings = ispeaker_rx_settings_u8800_pro,
	.setting_sz = ARRAY_SIZE(ispeaker_rx_settings_u8800_pro),
};

static struct snddev_icodec_data snddev_ispeaker_rx_data_u8800_pro = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "speaker_stereo_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_MONO, //7,
	.profile = &ispeaker_rx_profile_u8800_pro,
	.channel_mode = 1,//single channel for u8800_pro
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = &msm_snddev_poweramp_on,
	.pamp_off = &msm_snddev_poweramp_off,
	.max_voice_rx_vol[VOC_NB_INDEX] = 1200, 
	.min_voice_rx_vol[VOC_NB_INDEX] = -300, 
	.max_voice_rx_vol[VOC_WB_INDEX] = 1200, 
	.min_voice_rx_vol[VOC_WB_INDEX] = -300  
};

static struct platform_device msm_ispeaker_rx_device_u8800_pro = {
	.name = "snddev_icodec",
	.id = 8,
	.dev = { .platform_data = &snddev_ispeaker_rx_data_u8800_pro },
};
/* U8800-pro device end >*/
/* DTS2011102903430 dongchen 20111029 end >*/

#endif
/* DTS2011010400519 dongchen 20110104 end >*/
/*< DTS2011031005289 dongchen 20110329 begin */
/*< speaker ptt rx device begin */
static struct adie_codec_action_unit ispeaker_ptt_rx_48KHz_osr256_actions[] =
   SPEAKER_PTT_RX_48000_OSR_256;

static struct adie_codec_hwsetting_entry ispeaker_ptt_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispeaker_ptt_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispeaker_ptt_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ispeaker_ptt_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ispeaker_ptt_rx_settings,
	.setting_sz = ARRAY_SIZE(ispeaker_ptt_rx_settings),
};

static struct snddev_icodec_data snddev_ispeaker_ptt_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "speaker_ptt_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_PTT, 
	.profile = &ispeaker_ptt_rx_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = &msm_snddev_poweramp_on,
	.pamp_off = &msm_snddev_poweramp_off,
	.max_voice_rx_vol[VOC_NB_INDEX] = 200, 
	.min_voice_rx_vol[VOC_NB_INDEX] = -1300, 
	.max_voice_rx_vol[VOC_WB_INDEX] = 200, 
	.min_voice_rx_vol[VOC_WB_INDEX] = -1300  
};

static struct platform_device msm_ispeaker_ptt_rx_device = {
	.name = "snddev_icodec",
	.id = 252,
	.dev = { .platform_data = &snddev_ispeaker_ptt_rx_data },
};
/* speaker ptt rx device end >*/

/*< speaker ptt tx device begin */
static struct adie_codec_action_unit ispeaker_ptt_tx_8KHz_osr256_actions[] =
	SPEAKER_PTT_TX_8000_OSR_256;

static struct adie_codec_action_unit ispeaker_ptt_tx_48KHz_osr256_actions[] =
	SPEAKER_PTT_TX_48000_OSR_256;

static struct adie_codec_hwsetting_entry ispeaker_ptt_tx_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ispeaker_ptt_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispeaker_ptt_tx_8KHz_osr256_actions),
	},
	{ /* 8KHz profile is good for 16KHz */
		.freq_plan = 16000,
		.osr = 256,
		.actions = ispeaker_ptt_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispeaker_ptt_tx_8KHz_osr256_actions),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispeaker_ptt_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispeaker_ptt_tx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ispeaker_ptt_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = ispeaker_ptt_tx_settings,
	.setting_sz = ARRAY_SIZE(ispeaker_ptt_tx_settings),
};

static enum hsed_controller ispk_pmctl_ptt_id[] = {PM_HSED_CONTROLLER_0};

static struct snddev_icodec_data snddev_ispeaker_ptt_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "speaker_ptt_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_PTT_MIC,
	.profile = &ispeaker_ptt_tx_profile,
	.channel_mode = 1,
	.pmctl_id = ispk_pmctl_ptt_id,
	.pmctl_id_sz = ARRAY_SIZE(ispk_pmctl_ptt_id),
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};

static struct platform_device msm_ispeaker_ptt_tx_device = {
	.name = "snddev_icodec",
	.id = 253,
	.dev = { .platform_data = &snddev_ispeaker_ptt_tx_data },
};
/* speaker ptt tx device end >*/
/* DTS2011031005289 dongchen 20110329 end >*/
/*< DTS2011010400519 dongchen 20110104 begin */
/* Add huawei devices */
#ifdef CONFIG_HUAWEI_KERNEL
/*< U8800 device begin */
static struct adie_codec_action_unit ispeaker_music_mono_rx_48KHz_osr256_actions_u8800[] =
   SPEAKER_MUSIC_MONO_RX_48000_OSR_256_U8800;

static struct adie_codec_hwsetting_entry ispeaker_music_mono_rx_settings_u8800[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispeaker_music_mono_rx_48KHz_osr256_actions_u8800,
		.action_sz = ARRAY_SIZE(ispeaker_music_mono_rx_48KHz_osr256_actions_u8800),
	}
};

static struct adie_codec_dev_profile ispeaker_music_mono_rx_profile_u8800 = {
	.path_type = ADIE_CODEC_RX,
	.settings = ispeaker_music_mono_rx_settings_u8800,
	.setting_sz = ARRAY_SIZE(ispeaker_music_mono_rx_settings_u8800),
};

static struct snddev_icodec_data snddev_ispeaker_music_mono_rx_data_u8800 = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "speaker_music_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_MUSIC_MONO, // which value is 240
	.profile = &ispeaker_music_mono_rx_profile_u8800,
	.channel_mode = 1, //2,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = &msm_snddev_poweramp_4music_on, // only for music
	.pamp_off = &msm_snddev_poweramp_off,
	.max_voice_rx_vol[VOC_NB_INDEX] = 200, 
	.min_voice_rx_vol[VOC_NB_INDEX] = -1300, 
	.max_voice_rx_vol[VOC_WB_INDEX] = 200, 
	.min_voice_rx_vol[VOC_WB_INDEX] = -1300  
};

static struct platform_device msm_ispeaker_music_mono_rx_device_u8800 = {
	.name = "snddev_icodec",
	.id = 32,
	.dev = { .platform_data = &snddev_ispeaker_music_mono_rx_data_u8800 },

};
/* U8800 device end >*/

/*< U8820 device begin */
/*< DTS2011012602886 dongchen 20110126 begin */
static struct adie_codec_action_unit ispeaker_music_mono_rx_48KHz_osr256_actions_u8820[] =
   SPEAKER_MUSIC_MONO_RX_48000_OSR_256_U8820;

static struct adie_codec_hwsetting_entry ispeaker_music_mono_rx_settings_u8820[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispeaker_music_mono_rx_48KHz_osr256_actions_u8820,
		.action_sz = ARRAY_SIZE(ispeaker_music_mono_rx_48KHz_osr256_actions_u8820),
	}
};

static struct adie_codec_dev_profile ispeaker_music_mono_rx_profile_u8820 = {
	.path_type = ADIE_CODEC_RX,
	.settings = ispeaker_music_mono_rx_settings_u8820,
	.setting_sz = ARRAY_SIZE(ispeaker_music_mono_rx_settings_u8820),
};

static struct snddev_icodec_data snddev_ispeaker_music_mono_rx_data_u8820 = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "speaker_music_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_MUSIC_MONO, // which value is 240
	.profile = &ispeaker_music_mono_rx_profile_u8820,
	.channel_mode = 1, //2,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = &msm_snddev_poweramp_4music_on, // only for music
	.pamp_off = &msm_snddev_poweramp_off,
	.max_voice_rx_vol[VOC_NB_INDEX] = 200, 
	.min_voice_rx_vol[VOC_NB_INDEX] = -1300, 
	.max_voice_rx_vol[VOC_WB_INDEX] = 200, 
	.min_voice_rx_vol[VOC_WB_INDEX] = -1300  
};

static struct platform_device msm_ispeaker_music_mono_rx_device_u8820 = {
	.name = "snddev_icodec",
	.id = 32,
	.dev = { .platform_data = &snddev_ispeaker_music_mono_rx_data_u8820 },

};
/* DTS2011012602886 dongchen 20110126 end >*/
/* U8820 device end >*/

/*< U8800-51 device begin */
static struct adie_codec_action_unit ispeaker_music_mono_rx_48KHz_osr256_actions_u8800_51[] =
   SPEAKER_MUSIC_MONO_RX_48000_OSR_256_U8800_51;

static struct adie_codec_hwsetting_entry ispeaker_music_mono_rx_settings_u8800_51[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispeaker_music_mono_rx_48KHz_osr256_actions_u8800_51,
		.action_sz = ARRAY_SIZE(ispeaker_music_mono_rx_48KHz_osr256_actions_u8800_51),
	}
};

static struct adie_codec_dev_profile ispeaker_music_mono_rx_profile_u8800_51 = {
	.path_type = ADIE_CODEC_RX,
	.settings = ispeaker_music_mono_rx_settings_u8800_51,
	.setting_sz = ARRAY_SIZE(ispeaker_music_mono_rx_settings_u8800_51),
};

static struct snddev_icodec_data snddev_ispeaker_music_mono_rx_data_u8800_51 = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "speaker_music_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_MUSIC_MONO, // which value is 240
	.profile = &ispeaker_music_mono_rx_profile_u8800_51,
	.channel_mode = 1, //2,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = &msm_snddev_poweramp_4music_on, // only for music
	.pamp_off = &msm_snddev_poweramp_off,
	.max_voice_rx_vol[VOC_NB_INDEX] = 200, 
	.min_voice_rx_vol[VOC_NB_INDEX] = -1300, 
	.max_voice_rx_vol[VOC_WB_INDEX] = 200, 
	.min_voice_rx_vol[VOC_WB_INDEX] = -1300  
};

static struct platform_device msm_ispeaker_music_mono_rx_device_u8800_51 = {
	.name = "snddev_icodec",
	.id = 32,
	.dev = { .platform_data = &snddev_ispeaker_music_mono_rx_data_u8800_51 },

};
/* U8800-51 device end >*/
#endif
/* DTS2011010400519 dongchen 20110104 end >*/

static struct adie_codec_action_unit ifmradio_speaker_osr64_actions[] =
	FM_SPEAKER_OSR_64;

static struct adie_codec_hwsetting_entry ifmradio_speaker_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ifmradio_speaker_osr64_actions,
		.action_sz = ARRAY_SIZE(ifmradio_speaker_osr64_actions),
	}
};

static struct adie_codec_dev_profile ifmradio_speaker_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ifmradio_speaker_settings,
	.setting_sz = ARRAY_SIZE(ifmradio_speaker_settings),
};

static struct snddev_icodec_data snddev_ifmradio_speaker_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_FM),
	.name = "fmradio_speaker_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_LP_FM_SPKR_PHONE_STEREO_RX,
	.profile = &ifmradio_speaker_profile,
	.channel_mode = 1,
	.default_sample_rate = 8000,
	.pamp_on = &msm_snddev_poweramp_on,
	.pamp_off = &msm_snddev_poweramp_off,
	.dev_vol_type = SNDDEV_DEV_VOL_DIGITAL,
};

static struct platform_device msm_ifmradio_speaker_device = {
	.name = "snddev_icodec",
	.id = 9,
	.dev = { .platform_data = &snddev_ifmradio_speaker_data },
};

/*< DTS2010061001230 dongchen 20100610 begin */
//configure to capless mode due to SR.00329728 and our headset is class AB rather than class D.
#ifdef CONFIG_HUAWEI_KERNEL
static struct adie_codec_action_unit ifmradio_headset_osr64_actions[] =
	FM_HEADSET_CLASS_AB_STEREO_CAPLESS_OSR_64;
#else
static struct adie_codec_action_unit ifmradio_headset_osr64_actions[] =
	FM_HEADSET_STEREO_CLASS_D_LEGACY_OSR_64;
#endif
/* DTS2010061001230 SR.00329728 dongchen 20100610 end >*/

static struct adie_codec_hwsetting_entry ifmradio_headset_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ifmradio_headset_osr64_actions,
		.action_sz = ARRAY_SIZE(ifmradio_headset_osr64_actions),
	}
};

static struct adie_codec_dev_profile ifmradio_headset_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ifmradio_headset_settings,
	.setting_sz = ARRAY_SIZE(ifmradio_headset_settings),
};

/* < DTS2010061100256 dongchen 20100611 begin */
#ifdef CONFIG_HUAWEI_KERNEL
static struct snddev_icodec_data snddev_ifmradio_headset_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_FM),
	.name = "fmradio_headset_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_LP_FM_HEADSET_SPKR_STEREO_RX,
	.profile = &ifmradio_headset_profile,
	.channel_mode = 1,
	.default_sample_rate = 8000,
	.pamp_on = msm_snddev_hsed_voltage_on,
	.pamp_off = msm_snddev_hsed_voltage_off,
	.dev_vol_type = SNDDEV_DEV_VOL_DIGITAL,
};
#else
static struct snddev_icodec_data snddev_ifmradio_headset_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_FM),
	.name = "fmradio_headset_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_LP_FM_HEADSET_SPKR_STEREO_RX,
	.profile = &ifmradio_headset_profile,
	.channel_mode = 1,
	.default_sample_rate = 8000,
	.pamp_on = NULL,
	.pamp_off = NULL,
	.dev_vol_type = SNDDEV_DEV_VOL_DIGITAL,
};
#endif
/*DTS2010061100256 dongchen 20100611 end> */

static struct platform_device msm_ifmradio_headset_device = {
	.name = "snddev_icodec",
	.id = 10,
	.dev = { .platform_data = &snddev_ifmradio_headset_data },
};


static struct adie_codec_action_unit ifmradio_ffa_headset_osr64_actions[] =
	FM_HEADSET_CLASS_AB_STEREO_CAPLESS_OSR_64;

static struct adie_codec_hwsetting_entry ifmradio_ffa_headset_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ifmradio_ffa_headset_osr64_actions,
		.action_sz = ARRAY_SIZE(ifmradio_ffa_headset_osr64_actions),
	}
};

static struct adie_codec_dev_profile ifmradio_ffa_headset_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ifmradio_ffa_headset_settings,
	.setting_sz = ARRAY_SIZE(ifmradio_ffa_headset_settings),
};

static struct snddev_icodec_data snddev_ifmradio_ffa_headset_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_FM),
	.name = "fmradio_headset_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_LP_FM_HEADSET_SPKR_STEREO_RX,
	.profile = &ifmradio_ffa_headset_profile,
	.channel_mode = 1,
	.default_sample_rate = 8000,
	.pamp_on = msm_snddev_hsed_voltage_on,
	.pamp_off = msm_snddev_hsed_voltage_off,
	.dev_vol_type = SNDDEV_DEV_VOL_DIGITAL,
};

static struct platform_device msm_ifmradio_ffa_headset_device = {
	.name = "snddev_icodec",
	.id = 11,
	.dev = { .platform_data = &snddev_ifmradio_ffa_headset_data },
};

static struct snddev_ecodec_data snddev_bt_sco_earpiece_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "bt_sco_rx",
	.copp_id = 1,
	.acdb_id = ACDB_ID_BT_SCO_SPKR,
	.channel_mode = 1,
	.conf_pcm_ctl_val = BT_SCO_PCM_CTL_VAL,
	.conf_aux_codec_intf = BT_SCO_AUX_CODEC_INTF,
	.conf_data_format_padding_val = BT_SCO_DATA_FORMAT_PADDING,
	.max_voice_rx_vol[VOC_NB_INDEX] = 400,
	.min_voice_rx_vol[VOC_NB_INDEX] = -1100,
	.max_voice_rx_vol[VOC_WB_INDEX] = 400,
	.min_voice_rx_vol[VOC_WB_INDEX] = -1100,
};

static struct snddev_ecodec_data snddev_bt_sco_mic_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "bt_sco_tx",
	.copp_id = 1,
	.acdb_id = ACDB_ID_BT_SCO_MIC,
	.channel_mode = 1,
	.conf_pcm_ctl_val = BT_SCO_PCM_CTL_VAL,
	.conf_aux_codec_intf = BT_SCO_AUX_CODEC_INTF,
	.conf_data_format_padding_val = BT_SCO_DATA_FORMAT_PADDING,
};

struct platform_device msm_bt_sco_earpiece_device = {
	.name = "msm_snddev_ecodec",
	.id = 0,
	.dev = { .platform_data = &snddev_bt_sco_earpiece_data },
};

struct platform_device msm_bt_sco_mic_device = {
	.name = "msm_snddev_ecodec",
	.id = 1,
	.dev = { .platform_data = &snddev_bt_sco_mic_data },
};

/*< DTS2011010400519 dongchen 20110104 begin */
/* Add huawei devices */
#ifdef CONFIG_HUAWEI_KERNEL
/*< U8800 device begin */
static struct snddev_ecodec_data snddev_bt_sco_earpiece_data_u8800 = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "bt_sco_rx",
	.copp_id = 1,
	.acdb_id = ACDB_ID_BT_SCO_SPKR,
	.channel_mode = 1,
	.conf_pcm_ctl_val = BT_SCO_PCM_CTL_VAL,
	.conf_aux_codec_intf = BT_SCO_AUX_CODEC_INTF,
	.conf_data_format_padding_val = BT_SCO_DATA_FORMAT_PADDING,
	.max_voice_rx_vol[VOC_NB_INDEX] = 0, 
	.min_voice_rx_vol[VOC_NB_INDEX] = -1500, 
	.max_voice_rx_vol[VOC_WB_INDEX] = 0, 
	.min_voice_rx_vol[VOC_WB_INDEX] = -1500,
};

struct platform_device msm_bt_sco_earpiece_device_u8800 = {
	.name = "msm_snddev_ecodec",
	.id = 0,
	.dev = { .platform_data = &snddev_bt_sco_earpiece_data_u8800 },
};
/* U8800 device end >*/

/*< U8820 device begin */
static struct snddev_ecodec_data snddev_bt_sco_earpiece_data_u8820 = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "bt_sco_rx",
	.copp_id = 1,
	.acdb_id = ACDB_ID_BT_SCO_SPKR,
	.channel_mode = 1,
	.conf_pcm_ctl_val = BT_SCO_PCM_CTL_VAL,
	.conf_aux_codec_intf = BT_SCO_AUX_CODEC_INTF,
	.conf_data_format_padding_val = BT_SCO_DATA_FORMAT_PADDING,
	.max_voice_rx_vol[VOC_NB_INDEX] = 0, 
	.min_voice_rx_vol[VOC_NB_INDEX] = -1500, 
	.max_voice_rx_vol[VOC_WB_INDEX] = 0, 
	.min_voice_rx_vol[VOC_WB_INDEX] = -1500,
};

struct platform_device msm_bt_sco_earpiece_device_u8820 = {
	.name = "msm_snddev_ecodec",
	.id = 0,
	.dev = { .platform_data = &snddev_bt_sco_earpiece_data_u8820 },
};
/* U8820 device end >*/

/*< U8800-51 device begin */
static struct snddev_ecodec_data snddev_bt_sco_earpiece_data_u8800_51 = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "bt_sco_rx",
	.copp_id = 1,
	.acdb_id = ACDB_ID_BT_SCO_SPKR,
	.channel_mode = 1,
	.conf_pcm_ctl_val = BT_SCO_PCM_CTL_VAL,
	.conf_aux_codec_intf = BT_SCO_AUX_CODEC_INTF,
	.conf_data_format_padding_val = BT_SCO_DATA_FORMAT_PADDING,
	.max_voice_rx_vol[VOC_NB_INDEX] = 0, 
	.min_voice_rx_vol[VOC_NB_INDEX] = -1500, 
	.max_voice_rx_vol[VOC_WB_INDEX] = 0, 
	.min_voice_rx_vol[VOC_WB_INDEX] = -1500,
};

struct platform_device msm_bt_sco_earpiece_device_u8800_51 = {
	.name = "msm_snddev_ecodec",
	.id = 0,
	.dev = { .platform_data = &snddev_bt_sco_earpiece_data_u8800_51 },
};
/* U8800-51 device end >*/
#endif
/* DTS2011010400519 dongchen 20110104 end >*/

static struct adie_codec_action_unit idual_mic_endfire_8KHz_osr256_actions[] =
	MIC1_LEFT_LINE_IN_RIGHT_8000_OSR_256;

static struct adie_codec_hwsetting_entry idual_mic_endfire_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = idual_mic_endfire_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(idual_mic_endfire_8KHz_osr256_actions),
	}, /* 8KHz profile can be used for 16KHz */
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = idual_mic_endfire_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(idual_mic_endfire_8KHz_osr256_actions),
	}, /* 8KHz profile can be used for 48KHz */
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = idual_mic_endfire_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(idual_mic_endfire_8KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile idual_mic_endfire_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = idual_mic_endfire_settings,
	.setting_sz = ARRAY_SIZE(idual_mic_endfire_settings),
};

static enum hsed_controller idual_mic_endfire_pmctl_id[] = {
	PM_HSED_CONTROLLER_0, PM_HSED_CONTROLLER_2
};

static struct snddev_icodec_data snddev_idual_mic_endfire_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_dual_mic_endfire_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_MIC_ENDFIRE,
	.profile = &idual_mic_endfire_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pmctl_id = idual_mic_endfire_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(idual_mic_endfire_pmctl_id),
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_idual_mic_endfire_device = {
	.name = "snddev_icodec",
	.id = 12,
	.dev = { .platform_data = &snddev_idual_mic_endfire_data },
};


static struct snddev_icodec_data\
		snddev_idual_mic_endfire_real_stereo_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_dual_mic_endfire_tx_real_stereo",
	.copp_id = 0,
	.acdb_id = PSEUDO_ACDB_ID,
	.profile = &idual_mic_endfire_profile,
	.channel_mode = REAL_STEREO_CHANNEL_MODE,
	.default_sample_rate = 48000,
	.pmctl_id = idual_mic_endfire_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(idual_mic_endfire_pmctl_id),
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_real_stereo_tx_device = {
	.name = "snddev_icodec",
	.id = 26,
	.dev = { .platform_data =
			&snddev_idual_mic_endfire_real_stereo_data },
};

static struct adie_codec_action_unit idual_mic_bs_8KHz_osr256_actions[] =
	MIC1_LEFT_AUX_IN_RIGHT_8000_OSR_256;

static struct adie_codec_hwsetting_entry idual_mic_broadside_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = idual_mic_bs_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(idual_mic_bs_8KHz_osr256_actions),
	}, /* 8KHz profile can be used for 16KHz */
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = idual_mic_bs_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(idual_mic_bs_8KHz_osr256_actions),
	}, /* 8KHz profile can be used for 16KHz */
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = idual_mic_bs_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(idual_mic_bs_8KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile idual_mic_broadside_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = idual_mic_broadside_settings,
	.setting_sz = ARRAY_SIZE(idual_mic_broadside_settings),
};

static enum hsed_controller idual_mic_broadside_pmctl_id[] = {
	PM_HSED_CONTROLLER_0, PM_HSED_CONTROLLER_2
};

static struct snddev_icodec_data snddev_idual_mic_broadside_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_dual_mic_broadside_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_MIC_BROADSIDE,
	.profile = &idual_mic_broadside_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pmctl_id = idual_mic_broadside_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(idual_mic_broadside_pmctl_id),
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_idual_mic_broadside_device = {
	.name = "snddev_icodec",
	.id = 13,
	.dev = { .platform_data = &snddev_idual_mic_broadside_data },
};

/*< DTS2011010400519 dongchen 20110104 begin */
/* Add huawei devices */
#ifdef CONFIG_HUAWEI_KERNEL
/*< U8800 device begin */
static struct adie_codec_action_unit idual_mic_bs_8KHz_osr256_actions_u8800[] =
	MIC1_LEFT_AUX_IN_RIGHT_8000_OSR_256_U8800;

static struct adie_codec_hwsetting_entry idual_mic_broadside_settings_u8800[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = idual_mic_bs_8KHz_osr256_actions_u8800,
		.action_sz = ARRAY_SIZE(idual_mic_bs_8KHz_osr256_actions_u8800),
	}, /* 8KHz profile can be used for 16KHz */
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = idual_mic_bs_8KHz_osr256_actions_u8800,
		.action_sz = ARRAY_SIZE(idual_mic_bs_8KHz_osr256_actions_u8800),
	}, /* 8KHz profile can be used for 16KHz */
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = idual_mic_bs_8KHz_osr256_actions_u8800,
		.action_sz = ARRAY_SIZE(idual_mic_bs_8KHz_osr256_actions_u8800),
	}
};

static struct adie_codec_dev_profile idual_mic_broadside_profile_u8800 = {
	.path_type = ADIE_CODEC_TX,
	.settings = idual_mic_broadside_settings_u8800,
	.setting_sz = ARRAY_SIZE(idual_mic_broadside_settings_u8800),
};

static enum hsed_controller idual_mic_broadside_pmctl_id_u8800[] = {
	PM_HSED_CONTROLLER_0, PM_HSED_CONTROLLER_2
};

static struct snddev_icodec_data snddev_idual_mic_broadside_data_u8800 = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_dual_mic_broadside_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_MIC_BROADSIDE,
	.profile = &idual_mic_broadside_profile_u8800,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pmctl_id = idual_mic_broadside_pmctl_id_u8800,
	.pmctl_id_sz = ARRAY_SIZE(idual_mic_broadside_pmctl_id_u8800),
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_idual_mic_broadside_device_u8800 = {
	.name = "snddev_icodec",
	.id = 13,
	.dev = { .platform_data = &snddev_idual_mic_broadside_data_u8800 },
};
/* U8800 device end >*/

/*< U8820 device begin */
static struct adie_codec_action_unit idual_mic_bs_8KHz_osr256_actions_u8820[] =
	MIC1_LEFT_AUX_IN_RIGHT_8000_OSR_256_U8820;

static struct adie_codec_hwsetting_entry idual_mic_broadside_settings_u8820[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = idual_mic_bs_8KHz_osr256_actions_u8820,
		.action_sz = ARRAY_SIZE(idual_mic_bs_8KHz_osr256_actions_u8820),
	}, /* 8KHz profile can be used for 16KHz */
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = idual_mic_bs_8KHz_osr256_actions_u8820,
		.action_sz = ARRAY_SIZE(idual_mic_bs_8KHz_osr256_actions_u8820),
	}, /* 8KHz profile can be used for 16KHz */
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = idual_mic_bs_8KHz_osr256_actions_u8820,
		.action_sz = ARRAY_SIZE(idual_mic_bs_8KHz_osr256_actions_u8820),
	}
};

static struct adie_codec_dev_profile idual_mic_broadside_profile_u8820 = {
	.path_type = ADIE_CODEC_TX,
	.settings = idual_mic_broadside_settings_u8820,
	.setting_sz = ARRAY_SIZE(idual_mic_broadside_settings_u8820),
};

static enum hsed_controller idual_mic_broadside_pmctl_id_u8820[] = {
	PM_HSED_CONTROLLER_0, PM_HSED_CONTROLLER_2
};

static struct snddev_icodec_data snddev_idual_mic_broadside_data_u8820 = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_dual_mic_broadside_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_MIC_BROADSIDE,
	.profile = &idual_mic_broadside_profile_u8820,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pmctl_id = idual_mic_broadside_pmctl_id_u8820,
	.pmctl_id_sz = ARRAY_SIZE(idual_mic_broadside_pmctl_id_u8820),
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_idual_mic_broadside_device_u8820 = {
	.name = "snddev_icodec",
	.id = 13,
	.dev = { .platform_data = &snddev_idual_mic_broadside_data_u8820 },
};
/* U8820 device end >*/

/*< U8800-51 device begin */
static struct adie_codec_action_unit idual_mic_bs_8KHz_osr256_actions_u8800_51[] =
	MIC1_LEFT_AUX_IN_RIGHT_8000_OSR_256_U8800_51;

static struct adie_codec_hwsetting_entry idual_mic_broadside_settings_u8800_51[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = idual_mic_bs_8KHz_osr256_actions_u8800_51,
		.action_sz = ARRAY_SIZE(idual_mic_bs_8KHz_osr256_actions_u8800_51),
	}, /* 8KHz profile can be used for 16KHz */
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = idual_mic_bs_8KHz_osr256_actions_u8800_51,
		.action_sz = ARRAY_SIZE(idual_mic_bs_8KHz_osr256_actions_u8800_51),
	}, /* 8KHz profile can be used for 16KHz */
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = idual_mic_bs_8KHz_osr256_actions_u8800_51,
		.action_sz = ARRAY_SIZE(idual_mic_bs_8KHz_osr256_actions_u8800_51),
	}
};

static struct adie_codec_dev_profile idual_mic_broadside_profile_u8800_51 = {
	.path_type = ADIE_CODEC_TX,
	.settings = idual_mic_broadside_settings_u8800_51,
	.setting_sz = ARRAY_SIZE(idual_mic_broadside_settings_u8800_51),
};

static enum hsed_controller idual_mic_broadside_pmctl_id_u8800_51[] = {
	PM_HSED_CONTROLLER_0, PM_HSED_CONTROLLER_2
};

static struct snddev_icodec_data snddev_idual_mic_broadside_data_u8800_51 = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_dual_mic_broadside_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_MIC_BROADSIDE,
	.profile = &idual_mic_broadside_profile_u8800_51,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pmctl_id = idual_mic_broadside_pmctl_id_u8800_51,
	.pmctl_id_sz = ARRAY_SIZE(idual_mic_broadside_pmctl_id_u8800_51),
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_idual_mic_broadside_device_u8800_51 = {
	.name = "snddev_icodec",
	.id = 13,
	.dev = { .platform_data = &snddev_idual_mic_broadside_data_u8800_51 },
};
/* U8800-51 device end >*/

/*< DTS2011102903430 dongchen 20111029 begin */
/*< U8800 pro device begin */
static struct adie_codec_action_unit idual_mic_bs_8KHz_osr256_actions_u8800_pro[] =
	MIC1_LEFT_AUX_IN_RIGHT_8000_OSR_256_U8800_PRO;

static struct adie_codec_hwsetting_entry idual_mic_broadside_settings_u8800_pro[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = idual_mic_bs_8KHz_osr256_actions_u8800_pro,
		.action_sz = ARRAY_SIZE(idual_mic_bs_8KHz_osr256_actions_u8800_pro),
	}, /* 8KHz profile can be used for 16KHz */
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = idual_mic_bs_8KHz_osr256_actions_u8800_pro,
		.action_sz = ARRAY_SIZE(idual_mic_bs_8KHz_osr256_actions_u8800_pro),
	}, /* 8KHz profile can be used for 16KHz */
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = idual_mic_bs_8KHz_osr256_actions_u8800_pro,
		.action_sz = ARRAY_SIZE(idual_mic_bs_8KHz_osr256_actions_u8800_pro),
	}
};

static struct adie_codec_dev_profile idual_mic_broadside_profile_u8800_pro = {
	.path_type = ADIE_CODEC_TX,
	.settings = idual_mic_broadside_settings_u8800_pro,
	.setting_sz = ARRAY_SIZE(idual_mic_broadside_settings_u8800_pro),
};

static enum hsed_controller idual_mic_broadside_pmctl_id_u8800_pro[] = {
	PM_HSED_CONTROLLER_0, PM_HSED_CONTROLLER_2
};

static struct snddev_icodec_data snddev_idual_mic_broadside_data_u8800_pro = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_dual_mic_broadside_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_MIC_BROADSIDE,
	.profile = &idual_mic_broadside_profile_u8800_pro,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pmctl_id = idual_mic_broadside_pmctl_id_u8800_pro,
	.pmctl_id_sz = ARRAY_SIZE(idual_mic_broadside_pmctl_id_u8800_pro),
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_idual_mic_broadside_device_u8800_pro = {
	.name = "snddev_icodec",
	.id = 13,
	.dev = { .platform_data = &snddev_idual_mic_broadside_data_u8800_pro },
};
/* U8800 pro device end >*/
/* DTS2011102903430 dongchen 20111029 end >*/

/*< DTS2011110705477 yinzhaoyang 20111110 begin */
static struct adie_codec_action_unit idual_mic_bs_8KHz_osr256_actions_u8800_pro_es[] =
	MIC1_LEFT_AUX_IN_RIGHT_8000_OSR_256_U8800_PRO_ES;

static struct adie_codec_hwsetting_entry idual_mic_broadside_settings_u8800_pro_es[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = idual_mic_bs_8KHz_osr256_actions_u8800_pro_es,
		.action_sz = ARRAY_SIZE(idual_mic_bs_8KHz_osr256_actions_u8800_pro_es),
	}, /* 8KHz profile can be used for 16KHz */
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = idual_mic_bs_8KHz_osr256_actions_u8800_pro_es,
		.action_sz = ARRAY_SIZE(idual_mic_bs_8KHz_osr256_actions_u8800_pro_es),
	}, /* 8KHz profile can be used for 16KHz */
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = idual_mic_bs_8KHz_osr256_actions_u8800_pro_es,
		.action_sz = ARRAY_SIZE(idual_mic_bs_8KHz_osr256_actions_u8800_pro_es),
	}
};

static struct adie_codec_dev_profile idual_mic_broadside_profile_u8800_pro_es = {
	.path_type = ADIE_CODEC_TX,
	.settings = idual_mic_broadside_settings_u8800_pro_es,
	.setting_sz = ARRAY_SIZE(idual_mic_broadside_settings_u8800_pro_es),
};

static enum hsed_controller idual_mic_broadside_pmctl_id_u8800_pro_es[] = {
	PM_HSED_CONTROLLER_0, PM_HSED_CONTROLLER_2
};

static struct snddev_icodec_data snddev_idual_mic_broadside_data_u8800_pro_es = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_dual_mic_broadside_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_MIC_BROADSIDE,
	.profile = &idual_mic_broadside_profile_u8800_pro_es,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pmctl_id = idual_mic_broadside_pmctl_id_u8800_pro_es,
	.pmctl_id_sz = ARRAY_SIZE(idual_mic_broadside_pmctl_id_u8800_pro_es),
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_idual_mic_broadside_device_u8800_pro_es = {
	.name = "snddev_icodec",
	.id = 13,
	.dev = { .platform_data = &snddev_idual_mic_broadside_data_u8800_pro_es },
};
/* DTS2011110705477 yinzhaoyang 20111110 end > */
#endif
/* DTS2011010400519 dongchen 20110104 end >*/

static struct adie_codec_action_unit ispk_dual_mic_ef_8KHz_osr256_actions[] =
	SPEAKER_MIC1_LEFT_LINE_IN_RIGHT_8000_OSR_256;

static struct adie_codec_hwsetting_entry ispk_dual_mic_ef_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ispk_dual_mic_ef_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispk_dual_mic_ef_8KHz_osr256_actions),
	}, /* 8KHz profile can be used for 16Khz */
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = ispk_dual_mic_ef_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispk_dual_mic_ef_8KHz_osr256_actions),
	}, /* 8KHz profile can be used for 48KHz */
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispk_dual_mic_ef_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispk_dual_mic_ef_8KHz_osr256_actions),
	},
};

static struct adie_codec_dev_profile ispk_dual_mic_ef_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = ispk_dual_mic_ef_settings,
	.setting_sz = ARRAY_SIZE(ispk_dual_mic_ef_settings),
};

static struct snddev_icodec_data snddev_spk_idual_mic_endfire_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "speaker_dual_mic_endfire_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_MIC_ENDFIRE,
	.profile = &ispk_dual_mic_ef_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pmctl_id = idual_mic_endfire_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(idual_mic_endfire_pmctl_id),
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_spk_idual_mic_endfire_device = {
	.name = "snddev_icodec",
	.id = 14,
	.dev = { .platform_data = &snddev_spk_idual_mic_endfire_data },
};

static struct adie_codec_action_unit ispk_dual_mic_bs_8KHz_osr256_actions[] =
	SPEAKER_MIC1_LEFT_AUX_IN_RIGHT_8000_OSR_256;

static struct adie_codec_hwsetting_entry ispk_dual_mic_bs_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ispk_dual_mic_bs_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispk_dual_mic_bs_8KHz_osr256_actions),
	}, /* 8KHz profile can be used for 16Khz */
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = ispk_dual_mic_bs_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispk_dual_mic_bs_8KHz_osr256_actions),
	}, /* 8KHz profile can be used for 48KHz */
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispk_dual_mic_bs_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispk_dual_mic_bs_8KHz_osr256_actions),
	},
};

static struct adie_codec_dev_profile ispk_dual_mic_bs_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = ispk_dual_mic_bs_settings,
	.setting_sz = ARRAY_SIZE(ispk_dual_mic_bs_settings),
};
static struct snddev_icodec_data snddev_spk_idual_mic_broadside_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "speaker_dual_mic_broadside_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_MIC_BROADSIDE,
	.profile = &ispk_dual_mic_bs_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pmctl_id = idual_mic_broadside_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(idual_mic_broadside_pmctl_id),
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_spk_idual_mic_broadside_device = {
	.name = "snddev_icodec",
	.id = 15,
	.dev = { .platform_data = &snddev_spk_idual_mic_broadside_data },
};

static struct adie_codec_action_unit itty_hs_mono_tx_8KHz_osr256_actions[] =
	TTY_HEADSET_MONO_TX_8000_OSR_256;

static struct adie_codec_hwsetting_entry itty_hs_mono_tx_settings[] = {
	/* 8KHz, 16KHz, 48KHz TTY Tx devices can shared same set of actions */
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = itty_hs_mono_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(itty_hs_mono_tx_8KHz_osr256_actions),
	},
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = itty_hs_mono_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(itty_hs_mono_tx_8KHz_osr256_actions),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = itty_hs_mono_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(itty_hs_mono_tx_8KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile itty_hs_mono_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = itty_hs_mono_tx_settings,
	.setting_sz = ARRAY_SIZE(itty_hs_mono_tx_settings),
};

static struct snddev_icodec_data snddev_itty_hs_mono_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE | SNDDEV_CAP_TTY),
	.name = "tty_headset_mono_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_TTY_HEADSET_MIC,
	.profile = &itty_hs_mono_tx_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_itty_hs_mono_tx_device = {
	.name = "snddev_icodec",
	.id = 16,
	.dev = { .platform_data = &snddev_itty_hs_mono_tx_data },
};

static struct adie_codec_action_unit itty_hs_mono_rx_8KHz_osr256_actions[] =
	TTY_HEADSET_MONO_RX_CLASS_D_8000_OSR_256;

static struct adie_codec_action_unit itty_hs_mono_rx_16KHz_osr256_actions[] =
	TTY_HEADSET_MONO_RX_CLASS_D_16000_OSR_256;
/*< DTS2011012105219 yinzhaoyang modify 20110122 begin */
/* huawei TTY devices did not use Class D*/
#ifdef CONFIG_HUAWEI_KERNEL
static struct adie_codec_action_unit itty_hs_mono_rx_48KHz_osr256_actions[] =
	TTY_HEADSET_MONO_RX_CLASS_AB_48000_OSR_256;
#else
static struct adie_codec_action_unit itty_hs_mono_rx_48KHz_osr256_actions[] =
	TTY_HEADSET_MONO_RX_CLASS_D_48000_OSR_256;
#endif
/* DTS2011012105219 yinzhaoyang modify 20110122 end > */

static struct adie_codec_hwsetting_entry itty_hs_mono_rx_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = itty_hs_mono_rx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(itty_hs_mono_rx_8KHz_osr256_actions),
	},
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = itty_hs_mono_rx_16KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(itty_hs_mono_rx_16KHz_osr256_actions),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = itty_hs_mono_rx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(itty_hs_mono_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile itty_hs_mono_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = itty_hs_mono_rx_settings,
	.setting_sz = ARRAY_SIZE(itty_hs_mono_rx_settings),
};

static struct snddev_icodec_data snddev_itty_hs_mono_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE | SNDDEV_CAP_TTY),
	.name = "tty_headset_mono_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_TTY_HEADSET_SPKR,
	.profile = &itty_hs_mono_rx_profile,
	.channel_mode = 1,
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
	.max_voice_rx_vol[VOC_NB_INDEX] = 0,
	.min_voice_rx_vol[VOC_NB_INDEX] = 0,
	.max_voice_rx_vol[VOC_WB_INDEX] = 0,
	.min_voice_rx_vol[VOC_WB_INDEX] = 0,
};

static struct platform_device msm_itty_hs_mono_rx_device = {
	.name = "snddev_icodec",
	.id = 17,
	.dev = { .platform_data = &snddev_itty_hs_mono_rx_data },
};

static struct adie_codec_action_unit ispeaker_tx_8KHz_osr256_actions[] =
	SPEAKER_TX_8000_OSR_256;

static struct adie_codec_action_unit ispeaker_tx_48KHz_osr256_actions[] =
	SPEAKER_TX_48000_OSR_256;

static struct adie_codec_hwsetting_entry ispeaker_tx_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ispeaker_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispeaker_tx_8KHz_osr256_actions),
	},
	{ /* 8KHz profile is good for 16KHz */
		.freq_plan = 16000,
		.osr = 256,
		.actions = ispeaker_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispeaker_tx_8KHz_osr256_actions),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispeaker_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ispeaker_tx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ispeaker_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = ispeaker_tx_settings,
	.setting_sz = ARRAY_SIZE(ispeaker_tx_settings),
};

static enum hsed_controller ispk_pmctl_id[] = {PM_HSED_CONTROLLER_0};

static struct snddev_icodec_data snddev_ispeaker_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "speaker_mono_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_MIC,
	.profile = &ispeaker_tx_profile,
	.channel_mode = 1,
	.pmctl_id = ispk_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(ispk_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};

static struct platform_device msm_ispeaker_tx_device = {
	.name = "snddev_icodec",
	.id = 18,
	.dev = { .platform_data = &snddev_ispeaker_tx_data },
};

/*< DTS2011010400519 dongchen 20110104 begin */
/* Add huawei devices */
#ifdef CONFIG_HUAWEI_KERNEL
/*< U8800 device begin */
static struct adie_codec_action_unit ispeaker_tx_8KHz_osr256_actions_u8800[] =
	SPEAKER_TX_8000_OSR_256_U8800;

static struct adie_codec_action_unit ispeaker_tx_48KHz_osr256_actions_u8800[] =
	SPEAKER_TX_48000_OSR_256_U8800;

static struct adie_codec_hwsetting_entry ispeaker_tx_settings_u8800[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ispeaker_tx_8KHz_osr256_actions_u8800,
		.action_sz = ARRAY_SIZE(ispeaker_tx_8KHz_osr256_actions_u8800),
	},
	{ /* 8KHz profile is good for 16KHz */
		.freq_plan = 16000,
		.osr = 256,
		.actions = ispeaker_tx_8KHz_osr256_actions_u8800,
		.action_sz = ARRAY_SIZE(ispeaker_tx_8KHz_osr256_actions_u8800),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispeaker_tx_48KHz_osr256_actions_u8800,
		.action_sz = ARRAY_SIZE(ispeaker_tx_48KHz_osr256_actions_u8800),
	}
};

static struct adie_codec_dev_profile ispeaker_tx_profile_u8800 = {
	.path_type = ADIE_CODEC_TX,
	.settings = ispeaker_tx_settings_u8800,
	.setting_sz = ARRAY_SIZE(ispeaker_tx_settings_u8800),
};

static enum hsed_controller ispk_pmctl_id_u8800[] = {PM_HSED_CONTROLLER_0};

static struct snddev_icodec_data snddev_ispeaker_tx_data_u8800 = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "speaker_mono_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_MIC,
	.profile = &ispeaker_tx_profile_u8800,
	.channel_mode = 1,
	.pmctl_id = ispk_pmctl_id_u8800,
	.pmctl_id_sz = ARRAY_SIZE(ispk_pmctl_id_u8800),
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};

static struct platform_device msm_ispeaker_tx_device_u8800 = {
	.name = "snddev_icodec",
	.id = 18,
	.dev = { .platform_data = &snddev_ispeaker_tx_data_u8800 },
};
/* U8800 device end >*/

/*< U8820 device begin */
static struct adie_codec_action_unit ispeaker_tx_8KHz_osr256_actions_u8820[] =
	SPEAKER_TX_8000_OSR_256_U8820;

static struct adie_codec_action_unit ispeaker_tx_48KHz_osr256_actions_u8820[] =
	SPEAKER_TX_48000_OSR_256_U8820;

static struct adie_codec_hwsetting_entry ispeaker_tx_settings_u8820[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ispeaker_tx_8KHz_osr256_actions_u8820,
		.action_sz = ARRAY_SIZE(ispeaker_tx_8KHz_osr256_actions_u8820),
	},
	{ /* 8KHz profile is good for 16KHz */
		.freq_plan = 16000,
		.osr = 256,
		.actions = ispeaker_tx_8KHz_osr256_actions_u8820,
		.action_sz = ARRAY_SIZE(ispeaker_tx_8KHz_osr256_actions_u8820),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispeaker_tx_48KHz_osr256_actions_u8820,
		.action_sz = ARRAY_SIZE(ispeaker_tx_48KHz_osr256_actions_u8820),
	}
};

static struct adie_codec_dev_profile ispeaker_tx_profile_u8820 = {
	.path_type = ADIE_CODEC_TX,
	.settings = ispeaker_tx_settings_u8820,
	.setting_sz = ARRAY_SIZE(ispeaker_tx_settings_u8820),
};

static enum hsed_controller ispk_pmctl_id_u8820[] = {PM_HSED_CONTROLLER_0};

static struct snddev_icodec_data snddev_ispeaker_tx_data_u8820 = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "speaker_mono_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_MIC,
	.profile = &ispeaker_tx_profile_u8820,
	.channel_mode = 1,
	.pmctl_id = ispk_pmctl_id_u8820,
	.pmctl_id_sz = ARRAY_SIZE(ispk_pmctl_id_u8820),
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};

static struct platform_device msm_ispeaker_tx_device_u8820 = {
	.name = "snddev_icodec",
	.id = 18,
	.dev = { .platform_data = &snddev_ispeaker_tx_data_u8820 },
};
/* U8820 device end >*/

/*< U8800-51 device begin */
static struct adie_codec_action_unit ispeaker_tx_8KHz_osr256_actions_u8800_51[] =
	SPEAKER_TX_8000_OSR_256_U8800_51;

static struct adie_codec_action_unit ispeaker_tx_48KHz_osr256_actions_u8800_51[] =
	SPEAKER_TX_48000_OSR_256_U8800_51;

static struct adie_codec_hwsetting_entry ispeaker_tx_settings_u8800_51[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = ispeaker_tx_8KHz_osr256_actions_u8800_51,
		.action_sz = ARRAY_SIZE(ispeaker_tx_8KHz_osr256_actions_u8800_51),
	},
	{ /* 8KHz profile is good for 16KHz */
		.freq_plan = 16000,
		.osr = 256,
		.actions = ispeaker_tx_8KHz_osr256_actions_u8800_51,
		.action_sz = ARRAY_SIZE(ispeaker_tx_8KHz_osr256_actions_u8800_51),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ispeaker_tx_48KHz_osr256_actions_u8800_51,
		.action_sz = ARRAY_SIZE(ispeaker_tx_48KHz_osr256_actions_u8800_51),
	}
};

static struct adie_codec_dev_profile ispeaker_tx_profile_u8800_51 = {
	.path_type = ADIE_CODEC_TX,
	.settings = ispeaker_tx_settings_u8800_51,
	.setting_sz = ARRAY_SIZE(ispeaker_tx_settings_u8800_51),
};

static enum hsed_controller ispk_pmctl_id_u8800_51[] = {PM_HSED_CONTROLLER_0};

static struct snddev_icodec_data snddev_ispeaker_tx_data_u8800_51 = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "speaker_mono_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_MIC,
	.profile = &ispeaker_tx_profile_u8800_51,
	.channel_mode = 1,
	.pmctl_id = ispk_pmctl_id_u8800_51,
	.pmctl_id_sz = ARRAY_SIZE(ispk_pmctl_id_u8800_51),
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};

static struct platform_device msm_ispeaker_tx_device_u8800_51 = {
	.name = "snddev_icodec",
	.id = 18,
	.dev = { .platform_data = &snddev_ispeaker_tx_data_u8800_51 },
};
/* U8800-51 device end >*/
#endif
/* DTS2011010400519 dongchen 20110104 end >*/

static struct adie_codec_action_unit iearpiece_ffa_48KHz_osr256_actions[] =
	HANDSET_RX_48000_OSR_256_FFA;

static struct adie_codec_hwsetting_entry iearpiece_ffa_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = iearpiece_ffa_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(iearpiece_ffa_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile iearpiece_ffa_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = iearpiece_ffa_settings,
	.setting_sz = ARRAY_SIZE(iearpiece_ffa_settings),
};

static struct snddev_icodec_data snddev_iearpiece_ffa_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "handset_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_SPKR,
	.profile = &iearpiece_ffa_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
	.max_voice_rx_vol[VOC_NB_INDEX] = -700,
	.min_voice_rx_vol[VOC_NB_INDEX] = -2200,
	.max_voice_rx_vol[VOC_WB_INDEX] = -1400,
	.min_voice_rx_vol[VOC_WB_INDEX] = -2900,
};

static struct platform_device msm_iearpiece_ffa_device = {
	.name = "snddev_icodec",
	.id = 19,
	.dev = { .platform_data = &snddev_iearpiece_ffa_data },
};

static struct adie_codec_action_unit imic_ffa_8KHz_osr256_actions[] =
	HANDSET_TX_8000_OSR_256_FFA;

static struct adie_codec_action_unit imic_ffa_16KHz_osr256_actions[] =
	HANDSET_TX_16000_OSR_256_FFA;

static struct adie_codec_action_unit imic_ffa_48KHz_osr256_actions[] =
	HANDSET_TX_48000_OSR_256_FFA;

static struct adie_codec_hwsetting_entry imic_ffa_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = imic_ffa_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(imic_ffa_8KHz_osr256_actions),
	},
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = imic_ffa_16KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(imic_ffa_16KHz_osr256_actions),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = imic_ffa_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(imic_ffa_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile imic_ffa_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = imic_ffa_settings,
	.setting_sz = ARRAY_SIZE(imic_ffa_settings),
};

static struct snddev_icodec_data snddev_imic_ffa_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_MIC,
	.profile = &imic_ffa_profile,
	.channel_mode = 1,
	.pmctl_id = imic_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(imic_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
};

static struct platform_device msm_imic_ffa_device = {
	.name = "snddev_icodec",
	.id = 20,
	.dev = { .platform_data = &snddev_imic_ffa_data },
};


/*< DTS2010081201753 dongchen 20100926 begin */
#ifdef CONFIG_HUAWEI_KERNEL
static struct adie_codec_action_unit
	ihs_stereo_speaker_stereo_rx_48KHz_osr256_actions[] =
	HEADSET_MONO_SPEAKER_MONO_RX_CAPLESS_48000_OSR_256;
#else
static struct adie_codec_action_unit
	ihs_stereo_speaker_stereo_rx_48KHz_osr256_actions[] =
	HEADSET_STEREO_SPEAKER_STEREO_RX_CAPLESS_48000_OSR_256;
#endif
/* DTS2010081201753 dongchen 20100926 end >*/


static struct adie_codec_hwsetting_entry
	ihs_stereo_speaker_stereo_rx_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_stereo_speaker_stereo_rx_48KHz_osr256_actions,
		.action_sz =
		ARRAY_SIZE(ihs_stereo_speaker_stereo_rx_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ihs_stereo_speaker_stereo_rx_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ihs_stereo_speaker_stereo_rx_settings,
	.setting_sz = ARRAY_SIZE(ihs_stereo_speaker_stereo_rx_settings),
};

static struct snddev_icodec_data snddev_ihs_stereo_speaker_stereo_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_stereo_speaker_stereo_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_STEREO_PLUS_SPKR_STEREO_RX,
	.profile = &ihs_stereo_speaker_stereo_rx_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_poweramp_on,
	.pamp_off = msm_snddev_poweramp_off,
	.voltage_on = msm_snddev_hsed_voltage_on,
	.voltage_off = msm_snddev_hsed_voltage_off,
	.max_voice_rx_vol[VOC_NB_INDEX] = -500,
	.min_voice_rx_vol[VOC_NB_INDEX] = -2000,
	.max_voice_rx_vol[VOC_WB_INDEX] = -500,
	.min_voice_rx_vol[VOC_WB_INDEX] = -2000,
};

static struct platform_device msm_ihs_stereo_speaker_stereo_rx_device = {
	.name = "snddev_icodec",
	.id = 21,
	.dev = { .platform_data = &snddev_ihs_stereo_speaker_stereo_rx_data },
};

/*< DTS2011010400519 dongchen 20110104 begin */
/* Add huawei devices */
#ifdef CONFIG_HUAWEI_KERNEL
/*< U8800 device begin */
/* set mono capless headset and mono speaker for U8800 */
static struct adie_codec_action_unit
	ihs_stereo_speaker_stereo_rx_48KHz_osr256_actions_u8800[] =
	HEADSET_MONO_SPEAKER_MONO_RX_CAPLESS_48000_OSR_256_U8800;

static struct adie_codec_hwsetting_entry
	ihs_stereo_speaker_stereo_rx_settings_u8800[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_stereo_speaker_stereo_rx_48KHz_osr256_actions_u8800,
		.action_sz =
		ARRAY_SIZE(ihs_stereo_speaker_stereo_rx_48KHz_osr256_actions_u8800),
	}
};

static struct adie_codec_dev_profile ihs_stereo_speaker_stereo_rx_profile_u8800 = {
	.path_type = ADIE_CODEC_RX,
	.settings = ihs_stereo_speaker_stereo_rx_settings_u8800,
	.setting_sz = ARRAY_SIZE(ihs_stereo_speaker_stereo_rx_settings_u8800),
};

static struct snddev_icodec_data snddev_ihs_stereo_speaker_stereo_rx_data_u8800 = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_stereo_speaker_stereo_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_STEREO_PLUS_SPKR_STEREO_RX,
	.profile = &ihs_stereo_speaker_stereo_rx_profile_u8800,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_poweramp_on,
	.pamp_off = msm_snddev_poweramp_off,
	.voltage_on = msm_snddev_hsed_voltage_on,
	.voltage_off = msm_snddev_hsed_voltage_off,
	.max_voice_rx_vol[VOC_NB_INDEX] = -500,
	.min_voice_rx_vol[VOC_NB_INDEX] = -2000,
	.max_voice_rx_vol[VOC_WB_INDEX] = -500,
	.min_voice_rx_vol[VOC_WB_INDEX] = -2000,
};

static struct platform_device msm_ihs_stereo_speaker_stereo_rx_device_u8800 = {
	.name = "snddev_icodec",
	.id = 21,
	.dev = { .platform_data = &snddev_ihs_stereo_speaker_stereo_rx_data_u8800 },
};
/* U8800 device end >*/

/*< U8820 device begin */
/* set mono capless headset and mono speaker for U8820, maybe need change to stereo speaker afterwards */
static struct adie_codec_action_unit
	ihs_stereo_speaker_stereo_rx_48KHz_osr256_actions_u8820[] =
	HEADSET_MONO_SPEAKER_MONO_RX_CAPLESS_48000_OSR_256_U8820;

static struct adie_codec_hwsetting_entry
	ihs_stereo_speaker_stereo_rx_settings_u8820[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_stereo_speaker_stereo_rx_48KHz_osr256_actions_u8820,
		.action_sz =
		ARRAY_SIZE(ihs_stereo_speaker_stereo_rx_48KHz_osr256_actions_u8820),
	}
};

static struct adie_codec_dev_profile ihs_stereo_speaker_stereo_rx_profile_u8820 = {
	.path_type = ADIE_CODEC_RX,
	.settings = ihs_stereo_speaker_stereo_rx_settings_u8820,
	.setting_sz = ARRAY_SIZE(ihs_stereo_speaker_stereo_rx_settings_u8820),
};

static struct snddev_icodec_data snddev_ihs_stereo_speaker_stereo_rx_data_u8820 = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_stereo_speaker_stereo_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_STEREO_PLUS_SPKR_STEREO_RX,
	.profile = &ihs_stereo_speaker_stereo_rx_profile_u8820,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_poweramp_on,
	.pamp_off = msm_snddev_poweramp_off,
	.voltage_on = msm_snddev_hsed_voltage_on,
	.voltage_off = msm_snddev_hsed_voltage_off,
	.max_voice_rx_vol[VOC_NB_INDEX] = -500,
	.min_voice_rx_vol[VOC_NB_INDEX] = -2000,
	.max_voice_rx_vol[VOC_WB_INDEX] = -500,
	.min_voice_rx_vol[VOC_WB_INDEX] = -2000,
};

static struct platform_device msm_ihs_stereo_speaker_stereo_rx_device_u8820 = {
	.name = "snddev_icodec",
	.id = 21,
	.dev = { .platform_data = &snddev_ihs_stereo_speaker_stereo_rx_data_u8820 },
};
/* U8820 device end >*/

/*< U8800-51 device begin */
/* set mono capless headset and mono speaker for U8800-51 */
static struct adie_codec_action_unit
	ihs_stereo_speaker_stereo_rx_48KHz_osr256_actions_u8800_51[] =
	HEADSET_MONO_SPEAKER_MONO_RX_CAPLESS_48000_OSR_256_U8800_51;

static struct adie_codec_hwsetting_entry
	ihs_stereo_speaker_stereo_rx_settings_u8800_51[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ihs_stereo_speaker_stereo_rx_48KHz_osr256_actions_u8800_51,
		.action_sz =
		ARRAY_SIZE(ihs_stereo_speaker_stereo_rx_48KHz_osr256_actions_u8800_51),
	}
};

static struct adie_codec_dev_profile ihs_stereo_speaker_stereo_rx_profile_u8800_51 = {
	.path_type = ADIE_CODEC_RX,
	.settings = ihs_stereo_speaker_stereo_rx_settings_u8800_51,
	.setting_sz = ARRAY_SIZE(ihs_stereo_speaker_stereo_rx_settings_u8800_51),
};

/*< DTS2011102903430 dongchen 20111029 begin */
static struct snddev_icodec_data snddev_ihs_stereo_speaker_stereo_rx_data_u8800_51 = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "headset_stereo_speaker_stereo_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HEADSET_STEREO_PLUS_SPKR_MONO_RX,
	.profile = &ihs_stereo_speaker_stereo_rx_profile_u8800_51,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_poweramp_on,
	.pamp_off = msm_snddev_poweramp_off,
	.voltage_on = msm_snddev_hsed_voltage_on,
	.voltage_off = msm_snddev_hsed_voltage_off,
	.max_voice_rx_vol[VOC_NB_INDEX] = -500,
	.min_voice_rx_vol[VOC_NB_INDEX] = -2000,
	.max_voice_rx_vol[VOC_WB_INDEX] = -500,
	.min_voice_rx_vol[VOC_WB_INDEX] = -2000,
};
/* DTS2011102903430 dongchen 20111029 end >*/

static struct platform_device msm_ihs_stereo_speaker_stereo_rx_device_u8800_51 = {
	.name = "snddev_icodec",
	.id = 21,
	.dev = { .platform_data = &snddev_ihs_stereo_speaker_stereo_rx_data_u8800_51 },
};
/* U8800-51 device end >*/
#endif
/* DTS2011010400519 dongchen 20110104 end >*/


/* < DTS2010061100001  chenlei 20100527 begin*/
#ifdef CONFIG_HUAWEI_KERNEL
/* < DTS2010061100404  chenlei 20100611 begin*/
static enum hsed_controller handset_secondary_mic_tx_pmctl_id[] = {PM_HSED_CONTROLLER_0};
/* DTS2010061100404  chenlei 20100611 end >*/
static struct adie_codec_action_unit handset_secondary_mic_tx_8KHz_osr256_actions[] =
	HANDSET_SECONDARY_MIC_TX_8000_OSR_256;

static struct adie_codec_action_unit handset_secondary_mic_tx_16KHz_osr256_actions[] =
	HANDSET_SECONDARY_MIC_TX_16000_OSR_256;

/*< DTS2011030301679 dongchen 20110303 begin */
static struct adie_codec_action_unit handset_secondary_mic_tx_48KHz_osr256_actions[] =
	HANDSET_SECONDARY_MIC_TX_48000_OSR_256;
/* DTS2011030301679 dongchen 20110303 end >*/

/*< DTS2011030301679 dongchen 20110303 begin */
static struct adie_codec_hwsetting_entry handset_secondary_mic_tx_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = handset_secondary_mic_tx_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(handset_secondary_mic_tx_8KHz_osr256_actions),
	},
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = handset_secondary_mic_tx_16KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(handset_secondary_mic_tx_16KHz_osr256_actions),
	},
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = handset_secondary_mic_tx_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(handset_secondary_mic_tx_48KHz_osr256_actions),
	}
};
/* DTS2011030301679 dongchen 20110303 end >*/

static struct adie_codec_dev_profile handset_secondary_mic_tx_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = handset_secondary_mic_tx_settings,
	.setting_sz = ARRAY_SIZE(handset_secondary_mic_tx_settings),
};

/*< DTS2011030301679 dongchen 20110303 begin */
/* set default_sample_rate to 48000, keep the same as handset_tx device */
static struct snddev_icodec_data snddev_handset_secondary_mic_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_secondary_mic_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_HANDSET_MIC,
	.profile = &handset_secondary_mic_tx_profile,
	.channel_mode = 1,
	.pmctl_id = handset_secondary_mic_tx_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(handset_secondary_mic_tx_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = NULL,
	.pamp_off = NULL,
};
/* DTS2011030301679 dongchen 20110303 end >*/

static struct platform_device msm_handset_secondary_mic_tx_device = {
	.name = "snddev_icodec",
	.id = 30,
	.dev = { .platform_data = &snddev_handset_secondary_mic_tx_data },
};
#endif
/* DTS2010061100001  chenlei 20100527 end >*/

static struct snddev_mi2s_data snddev_mi2s_stereo_rx_data = {
	.capability = SNDDEV_CAP_RX ,
	.name = "hdmi_stereo_rx",
	.copp_id = 3,
	.acdb_id = ACDB_ID_HDMI,
	.channel_mode = 2,
	.sd_lines = MI2S_SD_0,
	.route = msm_snddev_tx_route_config,
	.deroute = msm_snddev_tx_route_deconfig,
	.default_sample_rate = 48000,
};

static struct platform_device msm_snddev_mi2s_stereo_rx_device = {
	.name = "snddev_mi2s",
	.id = 0,
	.dev = { .platform_data = &snddev_mi2s_stereo_rx_data },
};


static struct snddev_mi2s_data snddev_mi2s_fm_tx_data = {
	.capability = SNDDEV_CAP_TX ,
	.name = "fmradio_stereo_tx",
	.copp_id = 2,
	.acdb_id = ACDB_ID_FM_TX,
	.channel_mode = 2,
	.sd_lines = MI2S_SD_3,
	.route = NULL,
	.deroute = NULL,
	.default_sample_rate = 48000,
};

static struct platform_device  msm_snddev_mi2s_fm_tx_device = {
	.name = "snddev_mi2s",
	.id = 1,
	.dev = { .platform_data = &snddev_mi2s_fm_tx_data},
};

/*< DTS2011012604575 dongchen 20110127 begin */
#ifdef CONFIG_HUAWEI_KERNEL
/* ANALOG FM SPEAKER DEVICE */
static struct adie_codec_action_unit ifm_analog_speaker_48KHz_osr256_actions[] =
	FM_ANALOG_SPEAKER_48000_OSR_256; 

static struct adie_codec_hwsetting_entry ifm_analog_speaker_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ifm_analog_speaker_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ifm_analog_speaker_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ifm_analog_speaker_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ifm_analog_speaker_settings,
	.setting_sz = ARRAY_SIZE(ifm_analog_speaker_settings),
};

static struct snddev_icodec_data snddev_analog_fm_speaker_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "fmradio_analog_speaker",
	.copp_id = 0,
	.acdb_id = PSEUDO_ACDB_ID,
	.profile = &ifm_analog_speaker_profile,
	.channel_mode = 1,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = &msm_snddev_poweramp_on,
	.pamp_off = &msm_snddev_poweramp_off,
};

static struct platform_device  msm_snddev_analog_fm_speaker_device = {
	.name = "snddev_icodec",
	.id = 250,
	.dev = { .platform_data = &snddev_analog_fm_speaker_data},
};

/* ANALOG FM HEADSET DEVICE */
static struct adie_codec_action_unit ifm_analog_hs_48KHz_osr256_actions[] =
	FM_ANALOG_HEADSET_48000_OSR_256;

static struct adie_codec_hwsetting_entry ifm_analog_hs_settings[] = {
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = ifm_analog_hs_48KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(ifm_analog_hs_48KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile ifm_analog_hs_profile = {
	.path_type = ADIE_CODEC_RX,
	.settings = ifm_analog_hs_settings,
	.setting_sz = ARRAY_SIZE(ifm_analog_hs_settings),
};

static struct snddev_icodec_data snddev_analog_fm_hs_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "fmradio_analog_headset",
	.copp_id = 0,
	.acdb_id = PSEUDO_ACDB_ID,
	.profile = &ifm_analog_hs_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_hsed_voltage_on,
	.pamp_off = msm_snddev_hsed_voltage_off,
};

static struct platform_device msm_snddev_analog_fm_hs_device = {
	.name = "snddev_icodec",
	.id = 251,
	.dev = { .platform_data = &snddev_analog_fm_hs_data },
};
#endif //#ifdef CONFIG_HUAWEI_KERNEL
/* DTS2011012604575 dongchen 20110127 end >*/

static struct snddev_icodec_data snddev_fluid_imic_tx_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_MIC,
	.profile = &ispeaker_tx_profile,
	.channel_mode = 1,
	.pmctl_id = ispk_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(ispk_pmctl_id),
	.default_sample_rate = 48000,
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};

static struct platform_device msm_fluid_imic_tx_device = {
	.name = "snddev_icodec",
	.id = 22,
	.dev = { .platform_data = &snddev_fluid_imic_tx_data },
};

static struct snddev_icodec_data snddev_fluid_iearpiece_rx_data = {
	.capability = (SNDDEV_CAP_RX | SNDDEV_CAP_VOICE),
	.name = "handset_rx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_STEREO,
	.profile = &ispeaker_rx_profile,
	.channel_mode = 2,
	.pmctl_id = NULL,
	.pmctl_id_sz = 0,
	.default_sample_rate = 48000,
	.pamp_on = &msm_snddev_poweramp_on,
	.pamp_off = &msm_snddev_poweramp_off,
	.max_voice_rx_vol[VOC_NB_INDEX] = -500,
	.min_voice_rx_vol[VOC_NB_INDEX] = -1000,
	.max_voice_rx_vol[VOC_WB_INDEX] = -500,
	.min_voice_rx_vol[VOC_WB_INDEX] = -1000,
};

static struct platform_device msm_fluid_iearpeice_rx_device = {
	.name = "snddev_icodec",
	.id = 23,
	.dev = { .platform_data = &snddev_fluid_iearpiece_rx_data },
};

static struct adie_codec_action_unit fluid_idual_mic_ef_8KHz_osr256_actions[] =
	MIC1_LEFT_AUX_IN_RIGHT_8000_OSR_256;

static struct adie_codec_hwsetting_entry fluid_idual_mic_endfire_settings[] = {
	{
		.freq_plan = 8000,
		.osr = 256,
		.actions = fluid_idual_mic_ef_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(fluid_idual_mic_ef_8KHz_osr256_actions),
	}, /* 8KHz profile can be used for 16KHz */
	{
		.freq_plan = 16000,
		.osr = 256,
		.actions = fluid_idual_mic_ef_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(fluid_idual_mic_ef_8KHz_osr256_actions),
	}, /* 8KHz profile can also be used for 48KHz */
	{
		.freq_plan = 48000,
		.osr = 256,
		.actions = fluid_idual_mic_ef_8KHz_osr256_actions,
		.action_sz = ARRAY_SIZE(fluid_idual_mic_ef_8KHz_osr256_actions),
	}
};

static struct adie_codec_dev_profile fluid_idual_mic_endfire_profile = {
	.path_type = ADIE_CODEC_TX,
	.settings = fluid_idual_mic_endfire_settings,
	.setting_sz = ARRAY_SIZE(fluid_idual_mic_endfire_settings),
};

static enum hsed_controller fluid_idual_mic_endfire_pmctl_id[] = {
	PM_HSED_CONTROLLER_0, PM_HSED_CONTROLLER_2
};

static struct snddev_icodec_data snddev_fluid_idual_mic_endfire_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "handset_dual_mic_endfire_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_MIC_ENDFIRE,
	.profile = &fluid_idual_mic_endfire_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pmctl_id = fluid_idual_mic_endfire_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(fluid_idual_mic_endfire_pmctl_id),
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};

static struct platform_device msm_fluid_idual_mic_endfire_device = {
	.name = "snddev_icodec",
	.id = 24,
	.dev = { .platform_data = &snddev_fluid_idual_mic_endfire_data },
};

static struct snddev_icodec_data snddev_fluid_spk_idual_mic_endfire_data = {
	.capability = (SNDDEV_CAP_TX | SNDDEV_CAP_VOICE),
	.name = "speaker_dual_mic_endfire_tx",
	.copp_id = 0,
	.acdb_id = ACDB_ID_SPKR_PHONE_MIC_ENDFIRE,
	.profile = &fluid_idual_mic_endfire_profile,
	.channel_mode = 2,
	.default_sample_rate = 48000,
	.pmctl_id = fluid_idual_mic_endfire_pmctl_id,
	.pmctl_id_sz = ARRAY_SIZE(fluid_idual_mic_endfire_pmctl_id),
	.pamp_on = msm_snddev_tx_route_config,
	.pamp_off = msm_snddev_tx_route_deconfig,
};

static struct platform_device msm_fluid_spk_idual_mic_endfire_device = {
	.name = "snddev_icodec",
	.id = 25,
	.dev = { .platform_data = &snddev_fluid_spk_idual_mic_endfire_data },
};

static struct snddev_virtual_data snddev_a2dp_tx_data = {
	.capability = SNDDEV_CAP_TX,
	.name = "a2dp_tx",
	.copp_id = 5,
	.acdb_id = PSEUDO_ACDB_ID,
};

static struct snddev_virtual_data snddev_a2dp_rx_data = {
	.capability = SNDDEV_CAP_RX,
	.name = "a2dp_rx",
	.copp_id = 2,
	.acdb_id = PSEUDO_ACDB_ID,
};

static struct platform_device msm_a2dp_rx_device = {
	.name = "snddev_virtual",
	.id = 0,
	.dev = { .platform_data = &snddev_a2dp_rx_data },
};

static struct platform_device msm_a2dp_tx_device = {
	.name = "snddev_virtual",
	.id = 1,
	.dev = { .platform_data = &snddev_a2dp_tx_data },
};

static struct snddev_virtual_data snddev_uplink_rx_data = {
	.capability = SNDDEV_CAP_RX,
	.name = "uplink_rx",
	.copp_id = 5,
	.acdb_id = PSEUDO_ACDB_ID,
};

static struct platform_device msm_uplink_rx_device = {
	.name = "snddev_virtual",
	.id = 2,
	.dev = { .platform_data = &snddev_uplink_rx_data },
};

static struct platform_device *snd_devices_ffa[] __initdata = {
	&msm_iearpiece_ffa_device,
	&msm_imic_ffa_device,
	&msm_ifmradio_handset_device,
	&msm_ihs_ffa_stereo_rx_device,
	&msm_ihs_ffa_mono_rx_device,
	&msm_ihs_mono_tx_device,
	&msm_bt_sco_earpiece_device,
	&msm_bt_sco_mic_device,
	&msm_ispeaker_rx_device,
	&msm_ifmradio_speaker_device,
	&msm_ifmradio_ffa_headset_device,
	&msm_idual_mic_endfire_device,
	&msm_idual_mic_broadside_device,
	&msm_spk_idual_mic_endfire_device,
	&msm_spk_idual_mic_broadside_device,
	&msm_itty_hs_mono_tx_device,
	&msm_itty_hs_mono_rx_device,
	&msm_ispeaker_tx_device,
	&msm_ihs_stereo_speaker_stereo_rx_device,
	&msm_a2dp_rx_device,
	&msm_a2dp_tx_device,
	&msm_snddev_mi2s_stereo_rx_device,
	&msm_snddev_mi2s_fm_tx_device,
	&msm_uplink_rx_device,
	&msm_real_stereo_tx_device,
};

static struct platform_device *snd_devices_surf[] __initdata = {
	&msm_iearpiece_device,
	&msm_imic_device,
	&msm_ihs_stereo_rx_device,
	&msm_ihs_mono_rx_device,
	&msm_ihs_mono_tx_device,
	&msm_bt_sco_earpiece_device,
	&msm_bt_sco_mic_device,
	&msm_ifmradio_handset_device,
	&msm_ispeaker_rx_device,
	&msm_ifmradio_speaker_device,
	&msm_ifmradio_headset_device,
	&msm_itty_hs_mono_tx_device,
	&msm_itty_hs_mono_rx_device,
	&msm_ispeaker_tx_device,
	&msm_ihs_stereo_speaker_stereo_rx_device,
	&msm_a2dp_rx_device,
	&msm_a2dp_tx_device,
	&msm_snddev_mi2s_stereo_rx_device,
	&msm_snddev_mi2s_fm_tx_device,
	&msm_uplink_rx_device,
};

/*< DTS2011010400519 dongchen 20110104 begin */
/*<BU5D09852 lgq, sound device switch failure, 20100512 begin*/
#ifdef CONFIG_HUAWEI_KERNEL
static struct platform_device *snd_devices_u8800[] __initdata = {
	&msm_iearpiece_device_u8800,
	&msm_imic_device_u8800,
	&msm_ihs_stereo_rx_device_u8800,
	&msm_ihs_mono_tx_device_u8800,
	&msm_bt_sco_earpiece_device_u8800,
	&msm_bt_sco_mic_device,
	&msm_ifmradio_handset_device,
	&msm_ispeaker_rx_device_u8800,
	&msm_ifmradio_speaker_device,
	&msm_ifmradio_headset_device,
	&msm_itty_hs_mono_tx_device,
	&msm_itty_hs_mono_rx_device,
	&msm_ispeaker_tx_device_u8800,
	&msm_snddev_mi2s_fm_tx_device,
	/*< DTS2010072400226 dongchen 20100724 begin */
	/*<BU5D10256, lgq 20100519, hs mic enable begin */
	//delete &msm_idual_mic_endfire_device,
	&msm_idual_mic_broadside_device_u8800,
	//delete &msm_spk_idual_mic_endfire_device,
	&msm_spk_idual_mic_broadside_device,    
	/* BU5D10256, lgq 20100519, hs mic enable end> */
    /* DTS2010072400226 dongchen 20100724 end >*/
	/* < DTS2010061100001  chenlei 20100527 begin*/
	&msm_handset_secondary_mic_tx_device,              
    /* DTS2010061100001  chenlei 20100527 end >*/
    /*< DTS2010062500999 dongchen 20100625 begin */
	&msm_ihs_stereo_speaker_stereo_rx_device_u8800,
    /* DTS2010062500999 dongchen 20100625 end >*/
    /*< DTS2010120204486 dongchen 20101223 begin */
    &msm_ispeaker_music_mono_rx_device_u8800,
	&msm_snddev_mi2s_stereo_rx_device,
    /* DTS2010120204486 dongchen 20101223 end >*/
	/*< DTS2010122105598 dongchen 20101221 begin */
    &msm_ihs_music_stereo_rx_device_u8800,
    /* DTS2010122105598 dongchen 20101221 end >*/
};
#endif
/*BU5D09852 lgq, sound device switch failure, 20100512 end>*/
/* DTS2011010400519 dongchen 20110104 end >*/

/*< DTS2011010400519 dongchen 20110104 begin */
/*< DTS2010122004868 dongchen 20101220 begin */
#ifdef CONFIG_HUAWEI_KERNEL
static struct platform_device *snd_devices_u8820[] __initdata = {
	&msm_iearpiece_device_u8820,
	&msm_imic_device_u8820,
	&msm_ihs_stereo_rx_device_u8820,
	&msm_ihs_mono_tx_device_u8820,
	&msm_bt_sco_earpiece_device_u8820,
	&msm_bt_sco_mic_device,
	&msm_ifmradio_handset_device,
	/*< DTS2011012602886 dongchen 20110126 begin */
	&msm_ispeaker_rx_device_u8820,
	/* DTS2011012602886 dongchen 20110126 end >*/
	&msm_ifmradio_speaker_device,
	&msm_ifmradio_headset_device,
	&msm_itty_hs_mono_tx_device,
	&msm_itty_hs_mono_rx_device,
	&msm_ispeaker_tx_device_u8820,
	&msm_snddev_mi2s_fm_tx_device,
	&msm_idual_mic_broadside_device_u8820,
	&msm_spk_idual_mic_broadside_device,    
	&msm_handset_secondary_mic_tx_device,              
	&msm_ihs_stereo_speaker_stereo_rx_device_u8820,
	/*< DTS2010120204486 dongchen 20101223 begin */
    /*< DTS2011012602886 dongchen 20110126 begin */
    &msm_ispeaker_music_mono_rx_device_u8820,
    /* DTS2011012602886 dongchen 20110126 end >*/
	&msm_snddev_mi2s_stereo_rx_device,
    /* DTS2010120204486 dongchen 20101223 end >*/
	/*< DTS2010122105598 dongchen 20101221 begin */
    &msm_ihs_music_stereo_rx_device_u8820,
    /* DTS2010122105598 dongchen 20101221 end >*/
	/*< DTS2011021803548 dongchen 20110218 begin */
	&msm_snddev_analog_fm_speaker_device,
	&msm_snddev_analog_fm_hs_device,
	/* DTS2011021803548 dongchen 20110218 end >*/
    /*< DTS2011031005289 dongchen 20110329 begin */
    &msm_ispeaker_ptt_rx_device,
    &msm_ispeaker_ptt_tx_device,
    /* DTS2011031005289 dongchen 20110329 end >*/
};

static struct platform_device *snd_devices_u8800_51[] __initdata = {
	&msm_iearpiece_device_u8800_51,
	&msm_imic_device_u8800_51,
	&msm_ihs_stereo_rx_device_u8800_51,
	&msm_ihs_mono_tx_device_u8800_51,
	&msm_bt_sco_earpiece_device_u8800_51,
	&msm_bt_sco_mic_device,
	&msm_ifmradio_handset_device,
	&msm_ispeaker_rx_device_u8800_51,
	&msm_ifmradio_speaker_device,
	&msm_ifmradio_headset_device,
	&msm_itty_hs_mono_tx_device,
	&msm_itty_hs_mono_rx_device,
	&msm_ispeaker_tx_device_u8800_51,
	&msm_snddev_mi2s_fm_tx_device,
	&msm_idual_mic_broadside_device_u8800_51,
	&msm_spk_idual_mic_broadside_device,    
	&msm_handset_secondary_mic_tx_device,              
	&msm_ihs_stereo_speaker_stereo_rx_device_u8800_51,
	/*< DTS2010120204486 dongchen 20101223 begin */
    &msm_ispeaker_music_mono_rx_device_u8800_51,
	&msm_snddev_mi2s_stereo_rx_device,
    /* DTS2010120204486 dongchen 20101223 end >*/
	/*< DTS2010122105598 dongchen 20101221 begin */
    &msm_ihs_music_stereo_rx_device_u8800_51,
    /* DTS2010122105598 dongchen 20101221 end >*/
    /*< DTS2011012604575 dongchen 20110127 begin */
    &msm_snddev_analog_fm_speaker_device,
    &msm_snddev_analog_fm_hs_device,
    /* DTS2011012604575 dongchen 20110127 end >*/
    /*< DTS2011021900121 dongchen 20110217 begin */
    &msm_iearpiece_hac_device,
    /* DTS2011021900121 dongchen 20110217 end >*/
};

/*< DTS2011102903430 dongchen 20111029 begin */
static struct platform_device *snd_devices_u8800_pro[] __initdata = {
	&msm_iearpiece_device_u8800_pro,
	&msm_imic_device_u8800_51,
	&msm_ihs_stereo_rx_device_u8800_pro,
	&msm_ihs_mono_tx_device_u8800_51,
	&msm_bt_sco_earpiece_device_u8800_51,
	&msm_bt_sco_mic_device,
	&msm_ispeaker_rx_device_u8800_pro,
	&msm_itty_hs_mono_tx_device,
	&msm_itty_hs_mono_rx_device,
	&msm_ispeaker_tx_device_u8800_51,
	&msm_snddev_mi2s_fm_tx_device,
	&msm_idual_mic_broadside_device_u8800_pro,
	&msm_spk_idual_mic_broadside_device,    
	&msm_handset_secondary_mic_tx_device,              
	&msm_ihs_stereo_speaker_stereo_rx_device_u8800_51,
    &msm_ispeaker_music_mono_rx_device_u8800_51,
    &msm_ihs_music_stereo_rx_device_u8800_51,
    &msm_snddev_analog_fm_speaker_device,
    &msm_snddev_analog_fm_hs_device,
    &msm_iearpiece_hac_device,
};
/* DTS2011102903430 dongchen 20111029 end >*/

#endif
/* DTS2010122004868 dongchen 20101220 end >*/
/* DTS2011010400519 dongchen 20110104 end >*/
/*< DTS2011110705477 yinzhaoyang 20111110 begin */
static struct platform_device *snd_devices_u8800_pro_es[] __initdata = {
	&msm_iearpiece_device_u8800_pro,
	&msm_imic_device_u8800_51,
	&msm_ihs_stereo_rx_device_u8800_pro,
	&msm_ihs_mono_tx_device_u8800_51,
	&msm_bt_sco_earpiece_device_u8800_51,
	&msm_bt_sco_mic_device,
	&msm_ifmradio_handset_device,
	&msm_ispeaker_rx_device_u8800_pro,
	&msm_ifmradio_speaker_device,
	&msm_ifmradio_headset_device,
	&msm_itty_hs_mono_tx_device,
	&msm_itty_hs_mono_rx_device,
	&msm_ispeaker_tx_device_u8800_51,
	&msm_snddev_mi2s_fm_tx_device,
	&msm_idual_mic_broadside_device_u8800_pro_es,
	&msm_spk_idual_mic_broadside_device,    
	&msm_handset_secondary_mic_tx_device,              
	&msm_ihs_stereo_speaker_stereo_rx_device_u8800_51,
    &msm_ispeaker_music_mono_rx_device_u8800_51,
	&msm_snddev_mi2s_stereo_rx_device,
    &msm_ihs_music_stereo_rx_device_u8800_51,
    &msm_snddev_analog_fm_speaker_device,
    &msm_snddev_analog_fm_hs_device,
};
/* DTS2011110705477 yinzhaoyang 20111110 end > */


static struct platform_device *snd_devices_fluid[] __initdata = {
	&msm_ihs_stereo_rx_device,
	&msm_ihs_mono_rx_device,
	&msm_ihs_mono_tx_device,
	&msm_ispeaker_rx_device,
	&msm_ispeaker_tx_device,
	&msm_fluid_imic_tx_device,
	&msm_fluid_iearpeice_rx_device,
	&msm_fluid_idual_mic_endfire_device,
	&msm_fluid_spk_idual_mic_endfire_device,
	&msm_a2dp_rx_device,
	&msm_a2dp_tx_device,
	&msm_snddev_mi2s_stereo_rx_device,
	&msm_uplink_rx_device,
	&msm_ifmradio_speaker_device,
	&msm_ifmradio_headset_device,
};

#ifdef CONFIG_DEBUG_FS
static void snddev_hsed_config_modify_setting(int type)
{
	struct platform_device *device;
	struct snddev_icodec_data *icodec_data;

	device = &msm_ihs_ffa_stereo_rx_device;
	icodec_data = (struct snddev_icodec_data *)device->dev.platform_data;

	if (icodec_data) {
		if (type == 1) {
			icodec_data->voltage_on = NULL;
			icodec_data->voltage_off = NULL;
			icodec_data->profile->settings =
				ihs_ffa_stereo_rx_class_d_legacy_settings;
			icodec_data->profile->setting_sz =
			ARRAY_SIZE(ihs_ffa_stereo_rx_class_d_legacy_settings);
		} else if (type == 2) {
			icodec_data->voltage_on = NULL;
			icodec_data->voltage_off = NULL;
			icodec_data->profile->settings =
				ihs_ffa_stereo_rx_class_ab_legacy_settings;
			icodec_data->profile->setting_sz =
			ARRAY_SIZE(ihs_ffa_stereo_rx_class_ab_legacy_settings);
		}
	}
}

static void snddev_hsed_config_restore_setting(void)
{
	struct platform_device *device;
	struct snddev_icodec_data *icodec_data;

	device = &msm_ihs_ffa_stereo_rx_device;
	icodec_data = (struct snddev_icodec_data *)device->dev.platform_data;

	if (icodec_data) {
		icodec_data->voltage_on = msm_snddev_hsed_voltage_on;
		icodec_data->voltage_off = msm_snddev_hsed_voltage_off;
		icodec_data->profile->settings = ihs_ffa_stereo_rx_settings;
		icodec_data->profile->setting_sz =
			ARRAY_SIZE(ihs_ffa_stereo_rx_settings);
	}
}

static ssize_t snddev_hsed_config_debug_write(struct file *filp,
	const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	char *lb_str = filp->private_data;
	char cmd;

	if (get_user(cmd, ubuf))
		return -EFAULT;

	if (!strcmp(lb_str, "msm_hsed_config")) {
		switch (cmd) {
		case '0':
			snddev_hsed_config_restore_setting();
			break;

		case '1':
			snddev_hsed_config_modify_setting(1);
			break;

		case '2':
			snddev_hsed_config_modify_setting(2);
			break;

		default:
			break;
		}
	}
	return cnt;
}

static int snddev_hsed_config_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static const struct file_operations snddev_hsed_config_debug_fops = {
	.open = snddev_hsed_config_debug_open,
	.write = snddev_hsed_config_debug_write
};
#endif

void __ref msm_snddev_init(void)
{
    /*< DTS2011110705477 yinzhaoyang 20111110 begin */
    smem_huawei_vender *vender_para_ptr;
    const char *country_name = "es";
    /* DTS2011110705477 yinzhaoyang 20111110 end > */
    
	if (machine_is_msm7x30_ffa() || machine_is_msm8x55_ffa() ||
		machine_is_msm8x55_svlte_ffa()) {
		platform_add_devices(snd_devices_ffa,
		ARRAY_SIZE(snd_devices_ffa));
#ifdef CONFIG_DEBUG_FS
		debugfs_hsed_config = debugfs_create_file("msm_hsed_config",
					S_IFREG | S_IRUGO, NULL,
		(void *) "msm_hsed_config", &snddev_hsed_config_debug_fops);
#endif
	} else if (machine_is_msm7x30_surf() || machine_is_msm8x55_surf() ||
		machine_is_msm8x55_svlte_surf())
		platform_add_devices(snd_devices_surf,
		ARRAY_SIZE(snd_devices_surf));
	else if (machine_is_msm7x30_fluid())
		platform_add_devices(snd_devices_fluid,
		ARRAY_SIZE(snd_devices_fluid));
    /*<BU5D09852 lgq, sound device switch failure, 20100512 begin*/
    #ifdef CONFIG_HUAWEI_KERNEL
    /*< DTS2010092400487  lijianzhao 20100924 begin */
    /*< DTS2010122004868 dongchen 20101220 begin */
    /*< DTS2010112702297 wangquanli 201001125 begin */
    else if (machine_is_msm7x30_u8800())
    {
        platform_add_devices(snd_devices_u8800, ARRAY_SIZE(snd_devices_u8800));
    }
    else if (machine_is_msm7x30_u8820())
    {
        platform_add_devices(snd_devices_u8820, ARRAY_SIZE(snd_devices_u8820));
    }
    else if (machine_is_msm7x30_u8800_51())
    {
        platform_add_devices(snd_devices_u8800_51, ARRAY_SIZE(snd_devices_u8800_51));
    }
	/*< DTS2011030202729  liliang 20110302  begin */
    /*< DTS2011102903430 dongchen 20111029 begin */
    else if (machine_is_msm8255_u8800_pro())
    {
        /*< DTS2011110705477 yinzhaoyang 20111110 begin */
        vender_para_ptr = (smem_huawei_vender*)smem_alloc(SMEM_ID_VENDOR0, sizeof(smem_huawei_vender));
        if (!vender_para_ptr)
        {
            pr_info("%s: Can't find vender parameter\n", __func__);
            return;
        }
        pr_info("vendor:%s,country:%s\n", vender_para_ptr->vender_para.vender_name, vender_para_ptr->vender_para.country_name);

        /* choose audio parameter table according to the vender name */
        if(!memcmp(vender_para_ptr->vender_para.country_name, country_name, strlen(country_name)))
        {
            platform_add_devices(snd_devices_u8800_pro_es, ARRAY_SIZE(snd_devices_u8800_pro_es));
        }
        else
        {
            platform_add_devices(snd_devices_u8800_pro, ARRAY_SIZE(snd_devices_u8800_pro));
        }
        /* DTS2011110705477 yinzhaoyang 20111110 end > */
    }
    /* DTS2011102903430 dongchen 20111029 end >*/
	/* DTS2011030202729  liliang 20110302 end >*/ 	
    /*< DTS2011050601476 dongchen 20110506 begin */
    /* delete DTS2011041501614, snd_devices_u8860 is in snddev_data_timpani.c */
    /* DTS2011050601476 dongchen 20110506 end >*/
    /*< DTS2011051303410 dongchen 20110516 begin */
    /* delete DTS2011050700551, QTR8615 use snddev_data_timpani.c */
    /* DTS2011051303410 dongchen 20110516 end >*/
    /* DTS2010112702297 wangquanli 201001125 end >*/
	/* DTS2010122004868 dongchen 20101220 end >*/
    /* DTS2010092400487  lijianzhao 20100924 end >*/
    #endif
    /*BU5D09852 lgq, sound device switch failure, 20100512 end>*/
	else
		pr_err("%s: Unknown machine type\n", __func__);
}
