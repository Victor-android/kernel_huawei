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

#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/bootmem.h>
#include <linux/io.h>
#ifdef CONFIG_SPI_QSD
#include <linux/spi/spi.h>
#endif
#include <linux/mfd/pmic8058.h>
#include <linux/mfd/marimba.h>
#include <linux/i2c.h>
#include <linux/input.h>
/* <DTS2010072202961 hufeng 20100722 begin */
/* removed several lines */
/* DTS2010072202961 hufeng 20100722 end> */
#include <linux/ofn_atlab.h>
#include <linux/power_supply.h>
#include <linux/input/pmic8058-keypad.h>
#include <linux/i2c/isa1200.h>
#include <linux/pwm.h>
#include <linux/pmic8058-pwm.h>
#include <linux/i2c/tsc2007.h>
#include <linux/input/kp_flip_switch.h>
#include <linux/leds-pmic8058.h>
#include <linux/input/cy8c_ts.h>
#include <linux/msm_adc.h>
#include <linux/dma-mapping.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>

#include <mach/mpp.h>
#include <mach/board.h>
#include <mach/camera.h>
#include <mach/memory.h>
#include <mach/msm_iomap.h>
#include <mach/msm_hsusb.h>
#include <mach/rpc_hsusb.h>
#include <mach/msm_spi.h>
#include <mach/qdsp5v2/msm_lpa.h>
#include <mach/dma.h>
#include <linux/android_pmem.h>
#include <linux/input/msm_ts.h>
#include <mach/pmic.h>
#include <mach/rpc_pmapp.h>
#include <mach/qdsp5v2/aux_pcm.h>
#include <mach/qdsp5v2/mi2s.h>
#include <mach/qdsp5v2/audio_dev_ctl.h>
#include <mach/msm_battery.h>
#include <mach/rpc_server_handset.h>
#include <mach/msm_tsif.h>
#include <mach/socinfo.h>
#include <linux/cyttsp.h>
/*< DTS2011012004226 xuhui 20110120 begin */
//porting wifi driver for 7x30 platform
/* < DTS2010111804286  hanshirong 20101118 begin */
#ifdef CONFIG_HUAWEI_WIFI_SDCC
#include <linux/wifi_tiwlan.h>
#include <linux/skbuff.h>
#endif

/*<DTS2011042602009  fengwei 20110426 begin*/
#ifdef CONFIG_HUAWEI_KERNEL

#include <linux/touch_platform_config.h>

static char buf_virtualkey[500];
static ssize_t  buf_vkey_size=0;

atomic_t touch_detected_yet = ATOMIC_INIT(0); 
#define MSM_7x30_TOUCH_INT       148
#define MSM_7x30_RESET_PIN 		 85
struct vreg *vreg_gp4 = NULL;
#endif
/*DTS2011042602009 fengwei  20110426 end>*/

/* DTS2010111804286  hanshirong 20101118 end > */
/* DTS2011012004226 xuhui 20110120 end >*/
/* <DTS2010060900262 liyuping 20100805 begin */
#ifdef CONFIG_USB_AUTO_INSTALL
#include "../../../drivers/usb/gadget/usb_switch_huawei.h"
#include "../../../arch/arm/mach-msm/proc_comm.h"
#include "smd_private.h"

/* < DTS2011042602168 caomingxing 20110426 begin */
/*< DTS2011011904316 genghua 20110121 begin */
#ifdef CONFIG_HUAWEI_NFC_PN544
#include <linux/nfc/pn544.h>
#endif
/* DTS2011011904316 genghua 20110121 end >*/
/* DTS2011042602168 caomingxing 20110426 end > */

#define USB_SERIAL_LEN 20
/* keep the parameters transmitted from SMEM */
smem_huawei_vender usb_para_data;

/* keep the boot mode transfered from APPSBL */
//unsigned int usb_boot_mode = 0;

/* keep usb parameters transfered from modem */
app_usb_para usb_para_info;

/* all the pid used by mobile */
/* add new pid config for google */
/* new requirement: usb tethering */
usb_pid_stru usb_pid_array[]={
    {PID_ONLY_CDROM,     PID_NORMAL,     PID_UDISK, PID_AUTH,     PID_GOOGLE, PID_WLAN}, /* for COMMON products */
    {PID_ONLY_CDROM_TMO, PID_NORMAL_TMO, PID_UDISK, PID_AUTH_TMO, PID_GOOGLE, PID_WLAN}, /* for TMO products */
};

/* pointer to the member of usb_pid_array[], according to the current product */
usb_pid_stru *curr_usb_pid_ptr = &usb_pid_array[0];
#endif  
/* DTS2010060900262 liyuping 20100805 end > */

/*<BU5D09205 zhangtao 20100503 begin*/
#ifdef CONFIG_HUAWEI_FEATURE_AT42QT_TS
#include <linux/atmel_i2c_rmi.h>
#endif
/*BU5D09205 zhangtao 20100503 end>*/
/* < DTS2010061100003 luojianhong 201000612 begin*/
#include <linux/audio_amplifier.h>
#include <asm/mach/mmc.h>
#include <asm/mach/flash.h>
#include <mach/vreg.h>
#include "devices.h"
#include "timer.h"
#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android_composite.h>
#endif
#include "pm.h"
#include "spm.h"
#include <linux/msm_kgsl.h>
#include <mach/dal_axi.h>
#include <mach/msm_serial_hs.h>
#include <mach/msm_reqs.h>
#include <mach/qdsp5v2/mi2s.h>
#include <mach/qdsp5v2/audio_dev_ctl.h>

/*<BU5D07918, sibingsong 20100415 begin*/
#ifdef CONFIG_HUAWEI_FEATURE_VIBRATOR
#include "msm_vibrator.h"
#endif
/*BU5D07918, sibingsong 20100415 end>*/

/*< BU5D08419 haoqingtao 20100424 begin*/    
#include <linux/hardware_self_adapt.h>
/* BU5D08419 haoqingtao 20100424 end>*/    

/* <DTS2010081400556 shenjinming 20100814 begin */
/*< DTS2010071902252 shenjinming 20100719 begin */
#ifdef CONFIG_HUAWEI_EVALUATE_POWER_CONSUMPTION 
#include <mach/msm_battery.h>
#endif
/* DTS2010071902252 shenjinming 20100719 end >*/
/* DTS2010081400556 shenjinming 20100814 end> */
#include <mach/sdio_al.h>
#include "smd_private.h"
#include <linux/bma150.h>
/*< DTS2011093001643 lijianzhao 20110930 begin */
#include <linux/gpio_event.h>
#define GPIO_SLIDE_DETECT 42 //hall irq gpio
/* DTS2011093001643 lijianzhao 20110930 end >*/
/*DTS2011082405478 zhudengkui 20110913 begin */
//modify PMEM from 23M to 36M
#define MSM_PMEM_SF_SIZE	0x2400000
/* DTS2011082405478 zhudengkui 20110913 end>*/
#define MSM_FB_SIZE		0x500000
 /*< DTS2011062504321 songxiaoming 20110625 begin */
 /*<DTS2010102803387 penghai 20101028 begin*/
 /*<DTS2010090400203 penghai 20100904 begin*/
 /*<DTS2010082702166 penghai 20100827 begin*/
/*add dsp memory space for video*/
#define MSM_PMEM_ADSP_SIZE      0x2400000
/*DTS2010082702166 penghai 20100827 end>*/
/*DTS2010090400203 penghai 20100904 end>*/
/*DTS2010102803387 penghai 20101028 end>*/
/* DTS2011062504321 songxiaoming 20110625 end > */

#define MSM_FLUID_PMEM_ADSP_SIZE	0x2800000
#define PMEM_KERNEL_EBI1_SIZE   0x600000
#define MSM_PMEM_AUDIO_SIZE     0x200000

#define PMIC_GPIO_INT		27
#define PMIC_VREG_WLAN_LEVEL	2900

/* < DTS2011092803933  sunhonghui 20110928 begin */
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
#ifdef CONFIG_HUAWEI_KERNEL
/*interupt pin*/
#define PMIC_GPIO_SD_DET_PMICGPIO21_INTERUPT	21
#define PMIC_GPIO_SD_DET_PMICGPIO36_INTERUPT	36
#endif
#endif
/* DTS2011092803933  sunhonghui 20110928 end > */

/* DTS2011101201153  hujun 20111030 end > */
/*use pimc gpio 36 to enable the powre of SD*/
#ifndef CONFIG_HUAWEI_KERNEL
#define PMIC_GPIO_SDC4_EN_N	35  /* PMIC GPIO Number 36 */
#else
#define PMIC_GPIO_SDC4_EN_N	17  /* PMIC GPIO Number 18 */
#endif
/* DTS2011101201153  hujun 20111030 end > */
#define PMIC_GPIO_HDMI_5V_EN_V3 32  /* PMIC GPIO for V3 H/W */
#define PMIC_GPIO_HDMI_5V_EN_V2 39 /* PMIC GPIO for V2 H/W */

#define ADV7520_I2C_ADDR	0x39

#define FPGA_SDCC_STATUS       0x8E0001A8

#define FPGA_OPTNAV_GPIO_ADDR	0x8E000026
#define OPTNAV_I2C_SLAVE_ADDR	(0xB0 >> 1)
#define OPTNAV_IRQ		20
/*  DTS2010061100003 luojianhong 201000612 end > */
#define OPTNAV_CHIP_SELECT	19
/*<BU5D08118 zhangtao 20100419 begin*/
/*this is i2c pull-up power configs the i2c 
*pinname is gp13 and the voltage of the pin is 1800 mv */
#ifdef CONFIG_HUAWEI_KERNEL
	#define VREG_GP13_NAME	"gp13" 
	#define VREG_GP13_VOLTAGE_VALUE	1800
#endif
/*BU5D08118 zhangtao 20100419 end>*/
/*<BU5D09205 zhangtao 20100503 begin*/
#ifdef CONFIG_HUAWEI_FEATURE_AT42QT_TS
#define VCC_TS2V8 "gp4"
#define VCC_TS1V8 "gp7"
#endif
/*BU5D09205 zhangtao 20100503 end>*/

/* Macros assume PMIC GPIOs start at 0 */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)    (sys_gpio - NR_GPIO_IRQS)

#define PMIC_GPIO_FLASH_BOOST_ENABLE	15	/* PMIC GPIO Number 16 */
#define PMIC_GPIO_HAP_ENABLE   16  /* PMIC GPIO Number 17 */

#define PMIC_GPIO_WLAN_EXT_POR  22 /* PMIC GPIO NUMBER 23 */
/* < DTS2011102201769 zhangyu 20111025 begin */
#define PMIC_GPIO_TORCH_FLASH   23 /*  PMIC GPIO NUMBER 24   */
/* DTS2011102201769 zhangyu 20111025 end > */

#define BMA150_GPIO_INT 1

#define HAP_LVL_SHFT_MSM_GPIO 24

#define PMIC_GPIO_QUICKVX_CLK 37 /* PMIC GPIO 38 */

#define	PM_FLIP_MPP 5 /* PMIC MPP 06 */

/*< DTS2011012004291 xuhui 20110120 begin */
/*config res for  BCM4329 bt-wifi-fm in one */
#ifndef CONFIG_HUAWEI_KERNEL

/*< DTS2011021001070 xuhui 20110210 begin */
// 50 mSeconds for BCM4329 stable
/*< DTS2011021100645 xuhui 20110222 begin */
//delay 150ms, fix wifi-soft-ap turn on err, with and without wifi is on 
#define BCM4329_POWER_DELAY 150
/* DTS2011021100645 xuhui 20110222 end >*/
/* DTS2011021001070 xuhui 20110210 end >*/

/* BCM BT GPIOs config*/
#define GPIO_BT_UART_RTS   134 
#define GPIO_BT_UART_CTS   135
#define GPIO_BT_RX         136
#define GPIO_BT_TX         137

/*wake signals*/
#define GPIO_BT_WAKE_BT    143
#define GPIO_BT_WAKE_MSM   142

/*control signals*/
#define GPIO_BT_SHUTDOWN_N 161
#define GPIO_BT_RESET_N    163

/*gpio function*/
#define GPIO_BT_FUN_0        0
#define GPIO_BT_FUN_1        1 
#define GPIO_BT_ON           1
#define GPIO_BT_OFF          0

#endif
/* DTS2011012004291 xuhui 20110120 end >*/

/*< BU5D08419 haoqingtao 20100424 begin*/    

static unsigned int camera_id = 0;
static unsigned int lcd_id = 0;
static unsigned int ts_id = 0;
static unsigned int sub_board_id = 0;
/* <DTS2010071502918 shenjinming 20101008 begin */
#ifdef CONFIG_HUAWEI_POWER_DOWN_CHARGE
static unsigned int charge_flag = 0;
#endif
/* DTS2010071502918 shenjinming 20101008 end> */ 
/* < DTS2011042703449  liujinggang 20110427 begin */
compass_gs_position_type  get_compass_gs_position(void)
{
	compass_gs_position_type compass_gs_position=COMPASS_TOP_GS_TOP;
	
	/* < DTS2011043000257  liujinggang 20110503 begin */
	/* <DTS2011050700551 zhangbo 20110505 begin */
	/* <DTS2011062600102 sunhonghui 20110626 begin */
    /* <DTS2011071600361 liyuping 20110716 begin */
	/* < DTS2011090504241 liwei 20110905 begin */
	/* < DTS2011082302564 liwei 20110823 begin */
	/*<DTS2011091502092 liyuping 20110915 begin */
	/* < DTS2011102401822 liwei 20111024 begin */
	if((machine_is_msm7x30_u8800()) 
		|| (machine_is_msm7x30_u8820()) 
		|| (machine_is_msm7x30_u8800_51())
		|| (machine_is_msm8255_u8800_pro())
		|| (machine_is_msm8255_u8860()) 
		|| (machine_is_msm8255_c8860()) 
		|| (machine_is_msm8255_u8860lp())
		|| (machine_is_msm8255_u8860_92())            
	    || (machine_is_msm8255_u8667())
		|| (machine_is_msm8255_u8860_51()))
    /* DTS2011102401822 liwei 20111024 end > */
	/* DTS2011091502092 liyuping 20110915 end> */
	/* DTS2011082302564 liwei 20110823 end > */
	/* DTS2011071600361 liyuping 20110716 end> */
	/* DTS2011062600102 sunhonghui 20110626 end> */
	/* DTS2011050700551 zhangbo 20110505 end> */
	{
		compass_gs_position=COMPASS_BOTTOM_GS_BOTTOM;
	}
	/* <DTS2011091200073 zhangbo 20110912 begin */
	else if(machine_is_msm8255_u8680() 
	        || machine_is_msm8255_u8730())
	/* DTS2011091200073 zhangbo 20110912 end> */
	{
	    compass_gs_position=COMPASS_TOP_GS_TOP;
	}
	else
	{
		compass_gs_position=COMPASS_TOP_GS_TOP;
	}
	/* DTS2011090504241 liwei 20110905 end > */	
	/* DTS2011043000257  liujinggang 20110503 end > */
	return compass_gs_position;
}
/*return the string by compass position*/
char *get_compass_gs_position_name(void)
{
	compass_gs_position_type compass_gs_position=COMPASS_TOP_GS_TOP;
	char *position_name=NULL;

	compass_gs_position = get_compass_gs_position();

	switch(compass_gs_position)
	{
		case COMPASS_TOP_GS_TOP:
			 position_name = "COMPASS_TOP_GS_TOP";
			 break;
			 
		case COMPASS_TOP_GS_BOTTOM:
			 position_name = "COMPASS_TOP_GS_BOTTOM";
			 break;

		case COMPASS_BOTTOM_GS_TOP:
			 position_name = "COMPASS_BOTTOM_GS_TOP";
			 break;

		case COMPASS_BOTTOM_GS_BOTTOM:
			 position_name = "COMPASS_BOTTOM_GS_BOTTOM";
			 break;
			 
		case COMPASS_NONE_GS_BOTTOM:
			 position_name = "COMPASS_NONE_GS_BOTTOM";
			 break;

		default:
			 position_name = "COMPASS_TOP_GS_TOP";
			 break;
	}

	return position_name;
	
}
/* DTS2011042703449  liujinggang 20110427 end > */

static int pm8058_gpios_init(void)
{
	int rc;
	int pmic_gpio_hdmi_5v_en;

#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	struct pm8058_gpio sdcc_det = {
		.direction      = PM_GPIO_DIR_IN,
		.pull           = PM_GPIO_PULL_UP_1P5,
		.vin_sel        = 2,
		.function       = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol    = 0,
	};
#endif
	struct pm8058_gpio sdc4_en = {
		.direction      = PM_GPIO_DIR_OUT,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = PM_GPIO_VIN_L5,
		.function       = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol    = 0,
		.out_strength   = PM_GPIO_STRENGTH_LOW,
		.output_value   = 0,
	};

	struct pm8058_gpio haptics_enable = {
		.direction      = PM_GPIO_DIR_OUT,
		.pull           = PM_GPIO_PULL_NO,
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol    = 0,
		.vin_sel        = 2,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
	};

	struct pm8058_gpio flash_boost_enable = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = PM_GPIO_VIN_S3,
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_2,
	};

	/*< DTS2011091402372   yuguangcai 20110914 begin */
	/* < DTS2011102201769 zhangyu 20111025 begin */
	/* Init pm gpio23 with 0 like Qualcomm default config */


		struct pm8058_gpio gpio23 = {
			.direction      = PM_GPIO_DIR_OUT,
			.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
			.output_value   = 0,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = 2,
			.out_strength   = PM_GPIO_STRENGTH_LOW,
			.function       = PM_GPIO_FUNC_NORMAL,
		};

		struct pm8058_gpio gpio24 = {
			.direction      = PM_GPIO_DIR_OUT,
			.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
			.output_value   = 0,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = 2,
			.out_strength   = PM_GPIO_STRENGTH_LOW,
			.function       = PM_GPIO_FUNC_NORMAL,
		};

		/* Deassert GPIO#23 (source for Ext_POR on WLAN-Volans) */		
		rc = pm8058_gpio_config(PMIC_GPIO_WLAN_EXT_POR, &gpio23);
		if (rc) {
			pr_err("%s PMIC_GPIO_WLAN_EXT_POR config failed\n", __func__);
			return rc;
		}
		
		/* config pmic gpio 24 mean TORCH/FLASH */
		rc = pm8058_gpio_config(PMIC_GPIO_TORCH_FLASH, &gpio24);
		if (rc) {
			pr_err("%s PMIC_GPIO_WLAN_EXT_POR config failed\n", __func__);
			return rc;
		}

	/* DTS2011102201769 zhangyu 20111025 end > */

	/* DTS2011091402372   yuguangcai 20110914 end > */


	if (machine_is_msm8x55_svlte_surf() || machine_is_msm8x55_svlte_ffa() ||
						machine_is_msm7x30_fluid())
		pmic_gpio_hdmi_5v_en = PMIC_GPIO_HDMI_5V_EN_V2 ;
	else
		pmic_gpio_hdmi_5v_en = PMIC_GPIO_HDMI_5V_EN_V3 ;

	if (machine_is_msm7x30_fluid()) {
		rc = pm8058_gpio_config(PMIC_GPIO_HAP_ENABLE, &haptics_enable);
		if (rc) {
			pr_err("%s: PMIC GPIO %d write failed\n", __func__,
				(PMIC_GPIO_HAP_ENABLE + 1));
			return rc;
		}
		rc = pm8058_gpio_config(PMIC_GPIO_FLASH_BOOST_ENABLE,
			&flash_boost_enable);
		if (rc) {
			pr_err("%s: PMIC GPIO %d write failed\n", __func__,
				(PMIC_GPIO_FLASH_BOOST_ENABLE + 1));
			return rc;
		}
	}

/* < DTS2010092900628 luojianhong 201000929 begin*/
/* support U8820 version*/
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
/* < DTS2011092803933  sunhonghui 20110928 begin */
    /*set U8860 interupt flag*/
    sdcc_det.inv_int_pol = 1;
    /*set the pin to interupt function*/
#ifdef CONFIG_HUAWEI_KERNEL
    if(machine_is_msm8255_u8860_51())
    {
        rc = pm8058_gpio_config(PMIC_GPIO_SD_DET_PMICGPIO21_INTERUPT - 1, &sdcc_det);
    }
	/* < DTS2011102401822 liwei 20111024 begin */
    else if(machine_is_msm8255_u8680()
	    || machine_is_msm8255_u8667()
	    || machine_is_msm8255_u8730())
    /* DTS2011102401822 liwei 20111024 end > */
    {
        rc = pm8058_gpio_config(PMIC_GPIO_SD_DET_PMICGPIO36_INTERUPT - 1, &sdcc_det);
    }
    else
    {
        rc = 0;
    }
#else
    rc = pm8058_gpio_config(PMIC_GPIO_SD_DET_PMICGPIO36_INTERUPT - 1, &sdcc_det);
#endif
/* DTS2011092803933  sunhonghui 20110928 end > */
	
	if (rc) {
		pr_err("%s PMIC_GPIO_SD_DET config failed\n", __func__);
		return rc;
	}
#endif


	/*< DTS2011091402372   yuguangcai 20110914 begin */
    /*delete some lines*/
	/* DTS2011091402372   yuguangcai 20110914 end > */

	/*< DTS2010112702297 wangquanli 201001125 begin */
	/*< DTS2011030202729  liliang 20110302  begin */	
	/*< DTS2011042703705 zhangbo 20110422 begin */
	/* <DTS2011050700551 zhangbo 20110505 begin */
	/* < DTS2011061703204 caomingxing 20110617 begin */
	/* U8860/C8860 use PM_GPIO_18 in modem, it should not be reconfigured here */
	if ( machine_is_msm7x30_fluid() 
	|| (machine_is_msm7x30_u8800()) 
	|| (machine_is_msm7x30_u8820()) 
	|| (machine_is_msm7x30_u8800_51()) 
	|| (machine_is_msm8255_u8800_pro()))
	{
	/* DTS2011061703204 caomingxing 20110617 end > */
	/* DTS2011050700551 zhangbo 20110505 end> */
	/* DTS2011042703705 zhangbo 20110422 end >*/
	/* DTS2011030202729  liliang 20110302 end >*/
	/* DTS2010112702297 wangquanli 201001125 end >*/
		rc = pm8058_gpio_config(PMIC_GPIO_SDC4_EN_N, &sdc4_en);
		if (rc) {
			pr_err("%s PMIC_GPIO_SDC4_EN_N config failed\n",
								 __func__);
			return rc;
		}
		rc = gpio_request(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SDC4_EN_N),
				  "sdc4_en");
		if (rc) {
			pr_err("%s PMIC_GPIO_SDC4_EN_N gpio_request failed\n",
				__func__);
			return rc;
		}
		gpio_set_value_cansleep(
			PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SDC4_EN_N), 0);
	}
/*  DTS2010092900628 luojianhong 201000929 end > */
	return 0;
}
/* BU5D08419 haoqingtao 20100424 end>*/    
/*virtual key support */
static ssize_t tma300_vkeys_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
	__stringify(EV_KEY) ":" __stringify(KEY_BACK) ":50:842:80:100"
	":" __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":170:842:80:100"
	":" __stringify(EV_KEY) ":" __stringify(KEY_HOME) ":290:842:80:100"
	":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":410:842:80:100"
	"\n");
}

static struct kobj_attribute tma300_vkeys_attr = {
	.attr = {
		.mode = S_IRUGO,
	},
	.show = &tma300_vkeys_show,
};

static struct attribute *tma300_properties_attrs[] = {
	&tma300_vkeys_attr.attr,
	NULL
};

static struct attribute_group tma300_properties_attr_group = {
	.attrs = tma300_properties_attrs,
};

static struct kobject *properties_kobj;

#define CYTTSP_TS_GPIO_IRQ	150
static int cyttsp_platform_init(struct i2c_client *client)
{
	int rc = -EINVAL;
	struct vreg *vreg_ldo8, *vreg_ldo15;

	vreg_ldo8 = vreg_get(NULL, "gp7");

	if (!vreg_ldo8) {
		pr_err("%s: VREG L8 get failed\n", __func__);
		return rc;
	}

	rc = vreg_set_level(vreg_ldo8, 1800);
	if (rc) {
		pr_err("%s: VREG L8 set failed\n", __func__);
		goto l8_put;
	}

	rc = vreg_enable(vreg_ldo8);
	if (rc) {
		pr_err("%s: VREG L8 enable failed\n", __func__);
		goto l8_put;
	}

	vreg_ldo15 = vreg_get(NULL, "gp6");

	if (!vreg_ldo15) {
		pr_err("%s: VREG L15 get failed\n", __func__);
		goto l8_disable;
	}

	rc = vreg_set_level(vreg_ldo15, 3050);
	if (rc) {
		pr_err("%s: VREG L15 set failed\n", __func__);
		goto l8_disable;
	}

	rc = vreg_enable(vreg_ldo15);
	if (rc) {
		pr_err("%s: VREG L15 enable failed\n", __func__);
		goto l8_disable;
	}

	/* check this device active by reading first byte/register */
	rc = i2c_smbus_read_byte_data(client, 0x01);
	if (rc < 0) {
		pr_err("%s: i2c sanity check failed\n", __func__);
		goto l8_disable;
	}

	rc = gpio_tlmm_config(GPIO_CFG(CYTTSP_TS_GPIO_IRQ, 0, GPIO_CFG_INPUT,
					GPIO_CFG_PULL_UP, GPIO_CFG_6MA), GPIO_CFG_ENABLE);
	if (rc) {
		pr_err("%s: Could not configure gpio %d\n",
					 __func__, CYTTSP_TS_GPIO_IRQ);
		goto l8_disable;
	}

	rc = gpio_request(CYTTSP_TS_GPIO_IRQ, "ts_irq");
	if (rc) {
		pr_err("%s: unable to request gpio %d (%d)\n",
			__func__, CYTTSP_TS_GPIO_IRQ, rc);
		goto l8_disable;
	}

	/* virtual keys */
	tma300_vkeys_attr.attr.name = "virtualkeys.cyttsp-i2c";
	properties_kobj = kobject_create_and_add("board_properties",
				NULL);
	if (properties_kobj)
		rc = sysfs_create_group(properties_kobj,
			&tma300_properties_attr_group);
	if (!properties_kobj || rc)
		pr_err("%s: failed to create board_properties\n",
				__func__);

	return CY_OK;

l8_disable:
	vreg_disable(vreg_ldo8);
l8_put:
	vreg_put(vreg_ldo8);
	return rc;
}

static int cyttsp_platform_resume(struct i2c_client *client)
{
	/* add any special code to strobe a wakeup pin or chip reset */
	mdelay(10);

	return CY_OK;
}

static struct cyttsp_platform_data cyttsp_data = {
	.panel_maxx = 479,
	.panel_maxy = 799,
	.disp_maxx = 469,
	.disp_maxy = 799,
	.disp_minx = 10,
	.disp_miny = 0,
	.flags = 0,
	.gen = CY_GEN3,	/* or */
	.use_st = CY_USE_ST,
	.use_mt = CY_USE_MT,
	.use_hndshk = CY_SEND_HNDSHK,
	.use_trk_id = CY_USE_TRACKING_ID,
	.use_sleep = CY_USE_SLEEP,
	.use_gestures = CY_USE_GESTURES,
	/* activate up to 4 groups
	 * and set active distance
	 */
	.gest_set = CY_GEST_GRP1 | CY_GEST_GRP2 |
				CY_GEST_GRP3 | CY_GEST_GRP4 |
				CY_ACT_DIST,
	/* change act_intrvl to customize the Active power state
	 * scanning/processing refresh interval for Operating mode
	 */
	.act_intrvl = CY_ACT_INTRVL_DFLT,
	/* change tch_tmout to customize the touch timeout for the
	 * Active power state for Operating mode
	 */
	.tch_tmout = CY_TCH_TMOUT_DFLT,
	/* change lp_intrvl to customize the Low Power power state
	 * scanning/processing refresh interval for Operating mode
	 */
	.lp_intrvl = CY_LP_INTRVL_DFLT,
	.resume = cyttsp_platform_resume,
	.init = cyttsp_platform_init,
};

static int pm8058_pwm_config(struct pwm_device *pwm, int ch, int on)
{
	struct pm8058_gpio pwm_gpio_config = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = PM_GPIO_VIN_S3,
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_2,
	};
	int	rc = -EINVAL;
	int	id, mode, max_mA;

	id = mode = max_mA = 0;
	switch (ch) {
	case 0:
	case 1:
	case 2:
		if (on) {
			id = 24 + ch;
			rc = pm8058_gpio_config(id - 1, &pwm_gpio_config);
			if (rc)
				pr_err("%s: pm8058_gpio_config(%d): rc=%d\n",
				       __func__, id, rc);
		}
		break;

	case 3:
		id = PM_PWM_LED_KPD;
		mode = PM_PWM_CONF_DTEST3;
		max_mA = 200;
		break;

	case 4:
		id = PM_PWM_LED_0;
		mode = PM_PWM_CONF_PWM1;
		max_mA = 40;
		break;

	case 5:
		id = PM_PWM_LED_2;
		mode = PM_PWM_CONF_PWM2;
		max_mA = 40;
		break;

	case 6:
		id = PM_PWM_LED_FLASH;
		mode = PM_PWM_CONF_DTEST3;
		max_mA = 200;
		break;

	default:
		break;
	}

	if (ch >= 3 && ch <= 6) {
		if (!on) {
			mode = PM_PWM_CONF_NONE;
			max_mA = 0;
		}
		rc = pm8058_pwm_config_led(pwm, id, mode, max_mA);
		if (rc)
			pr_err("%s: pm8058_pwm_config_led(ch=%d): rc=%d\n",
			       __func__, ch, rc);
	}

	return rc;
}

static int pm8058_pwm_enable(struct pwm_device *pwm, int ch, int on)
{
	int	rc;

	switch (ch) {
	case 7:
		rc = pm8058_pwm_set_dtest(pwm, on);
		if (rc)
			pr_err("%s: pwm_set_dtest(%d): rc=%d\n",
			       __func__, on, rc);
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}

static const unsigned int fluid_keymap[] = {

/*<BU5D09283 luojianhong 20100506 begin*/
	KEY(0, 0, KEY_VOLUMEUP),
	KEY(0, 1, KEY_VOLUMEDOWN),
/* <DTS2010092702071 liugaofei 20100927 begin */
/* application have some problem,so set the mistake keyvalue*/
/*< DTS2010111701674 lijianzhao 20101117 begin */
/* KEY_CHAT is PTT, KEY_SOUND is handless */
	KEY(1, 0, KEY_CHAT),
	KEY(1, 1, KEY_SOUND),
/* DTS2010111701674 lijianzhao 20101117 end >*/
/* DTS2010092702071 liugaofei 20100927 end > */
/*<BU5D09283 luojianhong 20100506 end*/
};

/*< DTS2011042203570 n00111309 20110422 begin */
/* add for U8860, which do NOT use keypad row array .  */
static const unsigned int fluid_norow_keymap[] = {        
        KEY(0, 1, KEY_VOLUMEUP),
		KEY(1, 0, KEY_VOLUMEDOWN),		
};
/* DTS2011042203570 n00111309 20110422 end */

/* < BU5D08125 durui 20100419 begin */
static const unsigned int surf_keymap[] = {
    KEY(0, 0, KEY_VOLUMEUP),     //big_board  4 -- 4
    KEY(0, 1, KEY_VOLUMEDOWN),    //big_board  4 -- 3
    KEY(0, 2, KEY_UP),     //big_board  4 -- 2
    KEY(0, 3, KEY_RIGHT),  //big_board  4 -- 1

    KEY(1, 0, KEY_LEFT),  //big_board  3 -- 4
    KEY(1, 1, KEY_SEND), //big_board  3 -- 3
    KEY(1, 2, KEY_DOWN),     //big_board   3 -- 2
    KEY(1, 3, KEY_4),    //big_board   3 -- 1

    KEY(2, 0, KEY_6),        //big_board   2 -- 4
    KEY(2, 1, KEY_RIGHT),     //big_board   2 -- 3
    KEY(2, 2, KEY_ENTER),     //big_board   2 -- 2
    KEY(2, 3, KEY_LEFT),     //big_board   2 -- 1

    KEY(3, 0, KEY_HOME), //big_board   1 -- 4
    KEY(3, 1, KEY_BACK), //big_board  1 -- 3
    KEY(3, 2, KEY_UP),    //big_board   1 -- 2
    KEY(3, 3, KEY_MENU),   //big_board  1 -- 1
};
/* BU5D08125 durui 20100419 end > */
/*< DTS2011090104536 duanfei 20110901 begin */
/* add for U8680, which do NOT use keypad row array .  */
static const unsigned int U8680_keymap[] = {
    KEY(0, 0, KEY_VOLUMEUP),
    KEY(0, 1, KEY_VOLUMEDOWN),
    KEY(0, 2, KEY_CAMERA),
};

static struct matrix_keymap_data U8680_keymap_data = {
    .keymap_size    = ARRAY_SIZE(U8680_keymap),
    .keymap         = U8680_keymap,
};

static struct pmic8058_keypad_data U8680_keypad_data = {
	.input_name		= "fluid-keypad",
	.input_phys_device	= "fluid-keypad/input0",
	.num_rows		= 5,
	.num_cols		= 5,
	.rows_gpio_start	= 8,
	.cols_gpio_start	= 0,
	.debounce_ms		= {8, 10},
	.scan_delay_ms		= 32,
	.row_hold_ns		= 91500,
	.wakeup			= 1,
	.keymap_data            = &U8680_keymap_data,
};
/* DTS2011090104536 duanfei 20110901 end >*/
static struct resource resources_keypad[] = {
	{
		.start	= PM8058_KEYPAD_IRQ(PMIC8058_IRQ_BASE),
		.end	= PM8058_KEYPAD_IRQ(PMIC8058_IRQ_BASE),
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= PM8058_KEYSTUCK_IRQ(PMIC8058_IRQ_BASE),
		.end	= PM8058_KEYSTUCK_IRQ(PMIC8058_IRQ_BASE),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct matrix_keymap_data surf_keymap_data = {
        .keymap_size    = ARRAY_SIZE(surf_keymap),
        .keymap         = surf_keymap,
};


static struct pmic8058_keypad_data surf_keypad_data = {
	.input_name		= "surf_keypad",
	.input_phys_device	= "surf_keypad/input0",
	.num_rows		= 12,
	.num_cols		= 8,
	.rows_gpio_start	= 8,
	.cols_gpio_start	= 0,
	.debounce_ms		= {8, 10},
	.scan_delay_ms		= 32,
	.row_hold_ns		= 91500,
	.wakeup			= 1,
	.keymap_data            = &surf_keymap_data,
};

static struct matrix_keymap_data fluid_keymap_data = {
        .keymap_size    = ARRAY_SIZE(fluid_keymap),
        .keymap         = fluid_keymap,
};



static struct pmic8058_keypad_data fluid_keypad_data = {
	.input_name		= "fluid-keypad",
	.input_phys_device	= "fluid-keypad/input0",
	.num_rows		= 5,
	.num_cols		= 5,
	.rows_gpio_start	= 8,
	.cols_gpio_start	= 0,
	.debounce_ms		= {8, 10},
	.scan_delay_ms		= 32,
	.row_hold_ns		= 91500,
	.wakeup			= 1,
	.keymap_data            = &fluid_keymap_data,
};

/*< DTS2011042203570 n00111309 20110422 begin */
/* add for U8860, which do NOT use keypad row array .  */
static struct matrix_keymap_data fluid_norow_keymap_data = {
        .keymap_size    = ARRAY_SIZE(fluid_norow_keymap),
        .keymap         = fluid_norow_keymap,
};

static struct pmic8058_keypad_data fluid_norow_keypad_data = {
	.input_name		= "fluid-keypad",
	.input_phys_device	= "fluid-keypad/input0",
	.num_rows		= 5,
	.num_cols		= 5,
	.rows_gpio_start	= 8,
	.cols_gpio_start	= 0,
	.debounce_ms		= {8, 10},
	.scan_delay_ms		= 32,
	.row_hold_ns		= 91500,
	.wakeup			= 1,
	.keymap_data            = &fluid_norow_keymap_data,
};
/* DTS2011042203570 n00111309 20110422 end */

/*< DTS2011092601370 zhongjinrong 20110926 begin */
static const unsigned int u8730_keymap[] = {
    KEY(0, 0, KEY_VOLUMEUP), 
    KEY(0, 1, KEY_VOLUMEDOWN), 
    KEY(0, 2, KEY_CHAT),
    KEY(0, 3, KEY_SOUND), 
    KEY(0, 5, KEY_CAMERA),

    KEY(1, 0, KEY_S),
    KEY(1, 1, KEY_D),
    KEY(1, 2, KEY_G),
    KEY(1, 3, KEY_J),
    KEY(1, 4, KEY_L), 

	
    
    KEY(2, 0, KEY_W),
    KEY(2, 1, KEY_E),
    KEY(2, 2, KEY_T),
    KEY(2, 3, KEY_U),
    KEY(2, 4, KEY_O),


    KEY(3, 0, KEY_EMAIL),
    KEY(3, 1, KEY_COMMA ),
    KEY(3, 3, KEY_LEFT),
    KEY(3, 4, KEY_RIGHT),

    
    
    KEY(4, 0, KEY_LEFTSHIFT),
    KEY(4, 1, KEY_F),
    KEY(4, 2, KEY_SPACE),
    KEY(4, 3, KEY_DOT ),
    KEY(4, 4,KEY_BACKSPACE),

   
    KEY(5, 0, KEY_Q),
    KEY(5, 1, KEY_R),
    KEY(5, 2, KEY_Y),
    KEY(5, 3, KEY_I),
    KEY(5, 4, KEY_P),


   KEY(6, 0, KEY_Z),
    KEY(6, 1, KEY_QUESTION),
    KEY(6, 2, KEY_V),
    KEY(6, 3, KEY_M),
    KEY(6, 4, KEY_DOWN),

    
    
    KEY(7, 0, KEY_LEFTALT),
    KEY(7, 1, KEY_X),
    KEY(7, 2, KEY_B),
    KEY(7, 3, KEY_K),
    KEY(7, 4,KEY_ENTER),


    KEY(8, 0, KEY_A),
    KEY(8, 1, KEY_C),
    KEY(8, 2, KEY_H),
    KEY(8, 3, KEY_N),
    KEY(8, 4, KEY_UP),
};

static struct matrix_keymap_data u8730_keymap_data = {
        .keymap_size    = ARRAY_SIZE(u8730_keymap),
        .keymap         = u8730_keymap,
};


static struct pmic8058_keypad_data u8730_keypad_data = {
	.input_name		= "fluid-keypad",
	.input_phys_device	= "fluid-keypad/input0",
	.num_rows		= 10,
	.num_cols		= 6,
	.rows_gpio_start	= 8,
	.cols_gpio_start	= 0,
	.debounce_ms		= {8, 10},
	.scan_delay_ms		= 32,
	.row_hold_ns		= 91500,
	.wakeup			= 1,
	.keymap_data            = &u8730_keymap_data,
};

#ifdef CONFIG_HUAWEI_KEYBOARD_LEDS
static struct platform_device msm_device_pmic_keyboard_leds = {
	.name   = "pmic-keyboard-leds",
	.id = -1,
};
#endif
/* DTS2011092601370 zhongjinrong 20119026 end >*/
static struct pm8058_pwm_pdata pm8058_pwm_data = {
	.config		= pm8058_pwm_config,
	.enable		= pm8058_pwm_enable,
};

/* Put sub devices with fixed location first in sub_devices array */
#define	PM8058_SUBDEV_KPD	0
#define	PM8058_SUBDEV_LED	1

static struct pm8058_gpio_platform_data pm8058_gpio_data = {
	.gpio_base	= PM8058_GPIO_PM_TO_SYS(0),
	.irq_base	= PM8058_GPIO_IRQ(PMIC8058_IRQ_BASE, 0),
	.init		= pm8058_gpios_init,
};

static struct pm8058_gpio_platform_data pm8058_mpp_data = {
	.gpio_base	= PM8058_GPIO_PM_TO_SYS(PM8058_GPIOS),
	.irq_base	= PM8058_MPP_IRQ(PMIC8058_IRQ_BASE, 0),
};

/* < DTS2011030404089 zhangtao 20110314 begin */
/*changge the ffa leds node as keyboard-ffa-backlight*/
static struct pmic8058_led pmic8058_ffa_leds[] = {
	[0] = {
		.name		= "keyboard-ffa-backlight",
		.max_brightness = 15,
		.id		= PMIC8058_ID_LED_KB_LIGHT,
	},
};
/* DTS2011030404089 zhangtao 20110314 end > */

static struct pmic8058_leds_platform_data pm8058_ffa_leds_data = {
	.num_leds = ARRAY_SIZE(pmic8058_ffa_leds),
	.leds	= pmic8058_ffa_leds,
};

static struct pmic8058_led pmic8058_surf_leds[] = {
	[0] = {
		.name		= "keyboard-backlight",
		.max_brightness = 15,
		.id		= PMIC8058_ID_LED_KB_LIGHT,
	},
	[1] = {
		.name		= "voice:red",
		.max_brightness = 20,
		.id		= PMIC8058_ID_LED_0,
	},
	[2] = {
		.name		= "wlan:green",
		.max_brightness = 20,
		.id		= PMIC8058_ID_LED_2,
	},
};

static struct mfd_cell pm8058_subdevs[] = {
	{	.name = "pm8058-keypad",
		.id		= -1,
		.num_resources	= ARRAY_SIZE(resources_keypad),
		.resources	= resources_keypad,
	},
	{	.name = "pm8058-led",
		.id		= -1,
	},
	{	.name = "pm8058-gpio",
		.id		= -1,
		.platform_data	= &pm8058_gpio_data,
		.data_size	= sizeof(pm8058_gpio_data),
	},
	{	.name = "pm8058-mpp",
		.id		= -1,
		.platform_data	= &pm8058_mpp_data,
		.data_size	= sizeof(pm8058_mpp_data),
	},
	{	.name = "pm8058-pwm",
		.id		= -1,
		.platform_data	= &pm8058_pwm_data,
		.data_size	= sizeof(pm8058_pwm_data),
	},
	{	.name = "pm8058-nfc",
		.id		= -1,
	},
	{	.name = "pm8058-upl",
		.id		= -1,
	},
};

static struct pmic8058_leds_platform_data pm8058_surf_leds_data = {
	.num_leds = ARRAY_SIZE(pmic8058_surf_leds),
	.leds	= pmic8058_surf_leds,
};

static struct pmic8058_led pmic8058_fluid_leds[] = {
	[0] = {
		.name		= "keyboard-backlight",
		.max_brightness = 15,
		.id		= PMIC8058_ID_LED_KB_LIGHT,
	},
	[1] = {
		.name		= "flash:led_0",
		.max_brightness = 15,
		.id		= PMIC8058_ID_FLASH_LED_0,
	},
	[2] = {
		.name		= "flash:led_1",
		.max_brightness = 15,
		.id		= PMIC8058_ID_FLASH_LED_1,
	},
};

static struct pmic8058_leds_platform_data pm8058_fluid_leds_data = {
	.num_leds = ARRAY_SIZE(pmic8058_fluid_leds),
	.leds	= pmic8058_fluid_leds,
};

static struct pm8058_platform_data pm8058_7x30_data = {
	.irq_base = PMIC8058_IRQ_BASE,

	.num_subdevs = ARRAY_SIZE(pm8058_subdevs),
	.sub_devices = pm8058_subdevs,
	.irq_trigger_flags = IRQF_TRIGGER_LOW,
};
/* < DTS2010092501047 luojianhong 201000926 begin*/
/*add one more audio_amplifier_data for the right channel*/
static struct amplifier_platform_data right_audio_amplifier_data = {
    .amplifier_on = NULL,
    .amplifier_off = NULL,
    /*< DTS2010120204486 dongchen 20101223 begin */
    #ifdef CONFIG_HUAWEI_KERNEL
    .amplifier_4music_on = NULL,
    #endif
    /* DTS2010120204486 dongchen 20101223 end >*/
};
#ifdef CONFIG_HUAWEI_FEATURE_RIGHT_TPA2028D1_AMPLIFIER
static struct i2c_board_info msm_amplifier_boardinfo[]  = {
		{
		I2C_BOARD_INFO("tpa2028d1_r", 0x58),/*audio amplifier device*/
		.platform_data = &right_audio_amplifier_data,
	}
};
#endif
/*  DTS2010092501047 luojianhong 201000926 end > */

/*<DTS2011042602009 fengwei 20110426 begin*/
/*
 *the fucntion touch_power used to contrl the tp's power
 */
#ifdef CONFIG_HUAWEI_KERNEL
 /* < DTS2011042801270 yuezenglong 20110428 begin */
int power_switch(int pm)
{
    int rc_gp4 = 0;
    int value = IC_PM_VDD;
	if (IC_PM_ON == pm)
	{
        vreg_gp4 = vreg_get(NULL,"gp4");
        if (IS_ERR(vreg_gp4)) 
        {
		    pr_err("%s:gp4 power init get failed\n", __func__);
            goto err_power_fail;
        }
        rc_gp4=vreg_set_level(vreg_gp4, value);
        if(rc_gp4)
        {
            pr_err("%s:gp4 power init faild\n",__func__);
            goto err_power_fail;
        }
        rc_gp4=vreg_enable(vreg_gp4);
        if (rc_gp4) 
        {
		    pr_err("%s:gp4 power init failed \n", __func__);
	    }
        
        mdelay(50);     
  
	}
	else if(IC_PM_OFF == pm)
	{
		if(NULL != vreg_gp4)
		{
           rc_gp4 = vreg_disable(vreg_gp4);
           if (rc_gp4)
           {
               pr_err("%s:gp4 power disable failed \n", __func__);
           }
        }
	}
	else 
    {
       	rc_gp4 = -EPERM;
       	pr_err("%s:gp4 power switch not support yet!\n", __func__);	
    }
err_power_fail:
	return rc_gp4;
}
/* DTS2011042801270 yuezenglong 20110428 end > */
/*
 *use the touch_gpio_config_interrupt to config the gpio
 *which we used, but the gpio number can't exposure to user
 *so when the platform or the product changged please self self adapt
 */
 
int touch_gpio_config_interrupt(void)
{
	int gpio_config = 0;
    int ret = 0;
    gpio_config = GPIO_CFG(MSM_7x30_TOUCH_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA);
    ret = gpio_tlmm_config(gpio_config, GPIO_CFG_ENABLE);
    return ret;    
}
/*
 *the fucntion set_touch_probe_flag when the probe is detected use this function can set the flag ture
 */

void set_touch_probe_flag(int detected)/*we use this to detect the probe is detected*/
{
    if(detected >= 0)
    {
    	atomic_set(&touch_detected_yet, 1);
    }
    else
    {
    	atomic_set(&touch_detected_yet, 0);
    }
    return;
}

/*
 *the fucntion read_touch_probe_flag when the probe is ready to detect first we read the flag 
 *if the flag is set ture we will break the probe else we 
 *will run the probe fucntion
 */

int read_touch_probe_flag(void)
{
    int ret = 0;
    ret = atomic_read(&touch_detected_yet);
    return ret;
}

/*this function reset touch panel */
int touch_reset(void)
{
    int ret = 0;

	gpio_request(MSM_7x30_RESET_PIN,"TOUCH_RESET");
	ret = gpio_tlmm_config(GPIO_CFG(MSM_7x30_RESET_PIN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	ret = gpio_direction_output(MSM_7x30_RESET_PIN, 1);
	mdelay(5);
	ret = gpio_direction_output(MSM_7x30_RESET_PIN, 0);
	mdelay(10);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
	ret = gpio_direction_output(MSM_7x30_RESET_PIN, 1);
	mdelay(50);//must more than 10ms.

	return ret;
}

/*this function return reset gpio at 7x30 platform */
int get_touch_reset_pin(void)
{
	int ret = MSM_7x30_RESET_PIN;
	return ret;
}

/*this function get the tp  resolution*/
static int get_phone_version(struct tp_resolution_conversion *tp_resolution_type)
{
    int ret = 0;
    
    ret = touch_reset();
    if(ret)
    {
        printk(KERN_ERR "%s: reset failed \n", __func__);
        return -1;
    }
    /*<DTS2011051601005 fengwei 20110516 begin*/
    if (machine_is_msm7x30_u8820()
	  ||machine_is_msm7x30_u8800_51()
	  ||machine_is_msm8255_u8800_pro()
          ||machine_is_msm7x30_u8800())
    {
        tp_resolution_type->lcd_x = LCD_X_WVGA;
        tp_resolution_type->lcd_y = LCD_Y_WVGA;   
        tp_resolution_type->jisuan = LCD_JS_WVGA;
    }
    /*< DTS2011070901815  sunhonghui 20110709 begin */
    /* <DTS2011071600361 liyuping 20110716 begin */
	/* < DTS2011082302564 liwei 20110823 begin */
	/*<DTS2011091502092 liyuping 20110915 begin */
    /*< DTS2011091902006 duanfei 20110919 begin*/
    else if (machine_is_msm8255_u8860()
            ||(machine_is_msm8255_c8860())
            ||(machine_is_msm8255_u8860lp())
            ||(machine_is_msm8255_u8860_92())
            ||(machine_is_msm8255_u8860_51()))
    /* DTS2011091902006 duanfei 20110919 end >*/
    /* DTS2011091502092 liyuping 20110915 end> */
	/* DTS2011082302564 liwei 20110823 end > */
	/* DTS2011071600361 liyuping 20110716 end> */
    /*DTS2011070901815  sunhonghui 20110709 end >*/
    {
        tp_resolution_type->lcd_x = LCD_X_FWVGA;
        tp_resolution_type->lcd_y = LCD_Y_FWVGA;   
        tp_resolution_type->jisuan = LCD_JS_FWVGA;
    }
    /*< DTS2011091902006 duanfei 20110919 begin*/
    /* <DTS2011091200073 zhangbo 20110912 begin */
    /* < DTS2011102401822 liwei 20111024 begin */
    else if (machine_is_msm8255_u8680()
         || machine_is_msm8255_u8667()
	     || machine_is_msm8255_u8730())
    {
    	tp_resolution_type->lcd_x = LCD_X_WVGA;
        tp_resolution_type->lcd_y = LCD_Y_WVGA;   
        tp_resolution_type->jisuan = LCD_JS_WVGA_4INCHTP;
    }
    /* DTS2011102401822 liwei 20111024 end > */
    /* DTS2011091200073 zhangbo 20110912 end> */
    /* DTS2011091902006 duanfei 20110919 end >*/
    else
    {
        tp_resolution_type->lcd_x = LCD_X_WVGA;
        tp_resolution_type->lcd_y = LCD_Y_WVGA;   
        tp_resolution_type->jisuan = LCD_JS_WVGA;
    }    
    /*DTS2011051601005  fengwei 20110516 end>*/
  
    return 1;
}

/* <DTS2011050303216 wangjiongfeng 20110504 begin */
#ifdef CONFIG_HUAWEI_FEATURE_PROXIMITY_EVERLIGHT_APS_9900
int aps9900_gpio_config_interrupt(void)
{
    int gpio_config = 0;
    int ret = 0;
    
    gpio_config = GPIO_CFG(MSM_7X30_APS9900_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA);
    ret = gpio_tlmm_config(gpio_config, GPIO_CFG_ENABLE);
    return ret; 
}

static struct aps9900_hw_platform_data aps9900_hw_data = {
    .aps9900_power = power_switch,
    .aps9900_gpio_config_interrupt = aps9900_gpio_config_interrupt,
};
#endif
/* <DTS2011050303216 wangjiongfeng 20110504 end */
/* < DTS2011102806486  zhangmin 20111029  begin */
#ifdef CONFIG_HUAWEI_FEATURE_PROXIMITY_LITEON_LTR_558
int ltr558_gpio_config_interrupt(void)
{
    int gpio_config = 0;
    int ret = 0;
    gpio_config = GPIO_CFG(MSM_7X30_LTR558_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA);
    ret = gpio_tlmm_config(gpio_config, GPIO_CFG_ENABLE);
    return ret; 
}
static struct ltr558_hw_platform_data ltr558_hw_data = {
    .ltr558_power = power_switch,
    .ltr558_gpio_config_interrupt = ltr558_gpio_config_interrupt,
};
#endif
/* DTS2011102806486  zhangmin 20111029  end > */

/* < DTS2011042801270 yuezenglong 20110428 begin */
static struct touch_hw_platform_data touch_hw_data = {
    .touch_power = power_switch,
    .touch_gpio_config_interrupt = touch_gpio_config_interrupt,
    .set_touch_probe_flag = set_touch_probe_flag,
    .read_touch_probe_flag = read_touch_probe_flag,
    .touch_reset = touch_reset,
    .get_touch_reset_pin = get_touch_reset_pin,
    .get_phone_version = get_phone_version,
};
/* DTS2011042801270 yuezenglong 20110428 end > */
#endif

/*DTS2011042602009 fengwei 20110426 end>*/


static struct i2c_board_info pm8058_boardinfo[] __initdata = {
	{
		I2C_BOARD_INFO("pm8058-core", 0x55),
		.irq = MSM_GPIO_TO_INT(PMIC_GPIO_INT),
		.platform_data = &pm8058_7x30_data,
	},
};

static struct i2c_board_info cy8info[] __initdata = {
	{
		I2C_BOARD_INFO(CY_I2C_NAME, 0x24),
		.platform_data = &cyttsp_data,
#ifndef CY_USE_TIMER
		.irq = MSM_GPIO_TO_INT(CYTTSP_TS_GPIO_IRQ),
#endif /* CY_USE_TIMER */
	},
};

static struct i2c_board_info msm_camera_boardinfo[] __initdata = {
#ifdef CONFIG_MT9D112
	{
		I2C_BOARD_INFO("mt9d112", 0x78 >> 1),
	},
#endif
/*< DTS2011051400258   songxiaoming 20110514 begin */
#ifdef CONFIG_HUAWEI_SENSOR_HI701
	{
		I2C_BOARD_INFO("hi701", 0x60 >> 1),
	},
#endif
/* DTS2011051400258   songxiaoming 20110514 end > */

/*<BU5D08108, lijuan 00152865, 20100419 begin*/
#ifdef CONFIG_HUAWEI_SENSOR_OV7690
	{
		I2C_BOARD_INFO("ov7690", 0x42 >> 1),
	},
#endif 
/* BU5D08108, lijuan 00152865, 20100419 end> */

/*<BU5D10275  liyuping 00166065 20100529 begin*/

#ifdef CONFIG_HUAWEI_SENSOR_HIMAX0356
	{
		I2C_BOARD_INFO("himax0356", 0x68 >> 1),
	},
#endif 

/*BU5D10275  liyuping 00166065 20100529 end> */
/*<DTS2011042704563 penghai 20110427 begin*/
#ifdef CONFIG_HUAWEI_SENSOR_MT9D113
	{
		I2C_BOARD_INFO("mt9d113", 0x78 >> 1),
	},
#endif
/*DTS2011042704563 penghai 20110427 end>*/
/*<BU5D08116, lijuan 00152865, 20100419 begin*/
/* < DTS2011071802246 zhangyu 20110718 BEGIN */
#ifdef CONFIG_HUAWEI_SENSOR_S5K4E1GX
	{
		I2C_BOARD_INFO("s5k4e1gx", 0x6E >> 1),
	},
#endif 
/* DTS2011071802246 zhangyu 20110718 END > */ 
/* BU5D08116, lijuan 00152865, 20100419 end> */
/*<BU5D09497  lijuan 00152865  20100514 begin*/
#ifdef CONFIG_HUAWEI_SENSOR_OV5647_SUNNY
	{
		I2C_BOARD_INFO("ov5647_sunny", 0x6c >> 1),
	},
#endif 
/* BU5D09497  lijuan 00152865 20100514 end> */

/*<DTS2011042704563 penghai 20110427 begin*/
#ifdef CONFIG_HUAWEI_SENSOR_MT9E013
	{
		I2C_BOARD_INFO("mt9e013", 0x6C >> 2),
	},
#endif
/*DTS2011042704563 penghai 20110427 end>*/

#ifdef CONFIG_S5K3E2FX
	{
		I2C_BOARD_INFO("s5k3e2fx", 0x20 >> 1),
	},
#endif
#ifdef CONFIG_MT9P012
	{
		I2C_BOARD_INFO("mt9p012", 0x6C >> 1),
	},
#endif
#ifdef CONFIG_VX6953
	{
		I2C_BOARD_INFO("vx6953", 0x20),
	},
#endif

/*<DTS2011042704563 penghai 20110427 begin*/
/*deletes some lines*/
/*DTS2011042704563 penghai 20110427 end>*/
#ifdef CONFIG_SN12M0PZ
	{
		I2C_BOARD_INFO("sn12m0pz", 0x34 >> 1),
	},
#endif
#if defined(CONFIG_MT9T013) || defined(CONFIG_SENSORS_MT9T013)
	{
		I2C_BOARD_INFO("mt9t013", 0x6C),
	},
#endif
/* < DTS2011061801881 zhangyu 20110620 BEGIN */
#ifdef CONFIG_HUAWEI_SENSOR_MT9V114	
	{		
		I2C_BOARD_INFO("mt9v114_sunny", 0x7A >> 1),	
	},
#endif //CONFIG_HUAWEI_SENSOR_MT9V114
/* DTS2011061801881 zhangyu 20110620 END > */ 
/*< DTS2011072600848   songxiaoming 20110726 begin */
#ifdef CONFIG_HUAWEI_SENSOR_OV7736
	{
		I2C_BOARD_INFO("ov7736", 0x3C >> 1),//0x3C>>1,fake addr is 0x1E(0x78>>2),real add is 0x3c(0x78>>1)
	},
#endif
/* DTS2011072600848   songxiaoming 20110726 end > */
/* < DTS2011090701903 zhangyu 20110907 begin */
#ifdef CONFIG_HUAWEI_SENSOR_MT9P017
	{
		I2C_BOARD_INFO("mt9p017", 0x6D),//i2c real addr is 36.
	},
#endif
/* DTS2011090701903 zhangyu 20110907 end > */ 
/*<  DTS2011101000840   yuguangcai 20111010 begin */
#ifdef CONFIG_HUAWEI_SENSOR_S5K5CA
	{
		I2C_BOARD_INFO("s5k5ca", 0x5A >> 1),
	},
#endif
/* DTS2011101000840   yuguangcai 20111010 end > */
/* < DTS2011100802746 zhangyu 20111010 begin */
#ifdef CONFIG_HUAWEI_SENSOR_S5K4E1GX_P
	{
		I2C_BOARD_INFO("s5k4e1gx_p", 0x30 >> 1),
	},
#endif 
/* DTS2011100802746 zhangyu 20111010 end > */ 
/*< DTS2011102805854 yuguangcai 20111031 begin */
#ifdef CONFIG_HUAWEI_SENSOR_MT9T113
	{
		I2C_BOARD_INFO("mt9t113", 0x3C >> 2), 
	},
#endif
/* DTS2011102805854 yuguangcai 20111031 end > */

};

#ifdef CONFIG_MSM_CAMERA
#define	CAM_STNDBY	143
/*<DTS2011042704563 penghai 20110427 begin*/
static uint32_t camera_off_gpio_table[] = {
	/* parallel CAMERA interfaces */
/* < DTS2010072200238 luojianhong 201000722 begin*/
	GPIO_CFG(0,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* RST */
	GPIO_CFG(1, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VCM */
	GPIO_CFG(2,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT2 */
	GPIO_CFG(3,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT3 */
	GPIO_CFG(4,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT4 */
	GPIO_CFG(5,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT5 */
	GPIO_CFG(6,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT6 */
	GPIO_CFG(7,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT7 */
	GPIO_CFG(8,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT8 */
	GPIO_CFG(9,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT9 */
	GPIO_CFG(10, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT10 */
	GPIO_CFG(11, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT11 */
	GPIO_CFG(12, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* PCLK */
	GPIO_CFG(13, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VSYNC_IN */
	GPIO_CFG(15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* MCLK */
	#ifdef CONFIG_HUAWEI_CAMERA
    /*< DTS2011062900801   songxiaoming 20110629 begin */
    /*< DTS2011081004264 zhangyu 20110810 begin */
    /* Delete 2 lines */
	/* DTS2011081004264 zhangyu 20110810 end > */
	/* DTS2011062900801   songxiaoming 20110629 end > */
	GPIO_CFG(31, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), /* RESET FOR OV7690*/
	GPIO_CFG(52, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* pwd FOR mt9d113*/
	GPIO_CFG(55, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* RESET FOR 4E1*/	
	GPIO_CFG(56, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VCM FOR 4E1*/	
	GPIO_CFG(88, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* ov5647*/	
	#endif
/*  DTS2010072200238 luojianhong 201000722 end > */
};
static uint32_t camera_on_gpio_table[] = {
	/* parallel CAMERA interfaces */
	GPIO_CFG(0,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* RST */
	GPIO_CFG(1, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), /* VCM */
	GPIO_CFG(2,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT2 */
	GPIO_CFG(3,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT3 */
	GPIO_CFG(4,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT4 */
	GPIO_CFG(5,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT5 */
	GPIO_CFG(6,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT6 */
	GPIO_CFG(7,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT7 */
	GPIO_CFG(8,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT8 */
	GPIO_CFG(9,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT9 */
	GPIO_CFG(10, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT10 */
	GPIO_CFG(11, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT11 */
	GPIO_CFG(12, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* PCLK */
	GPIO_CFG(13, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VSYNC_IN */
	GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* MCLK */
	/*<BU5D09497  lijuan 00152865  20100514 begin*/
	#ifdef CONFIG_HUAWEI_CAMERA
	GPIO_CFG(16, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), /* SCL */
	GPIO_CFG(17, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), /* SDL */
	GPIO_CFG(31, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* RESET FOR OV7690*/
	GPIO_CFG(52, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* pwd FOR mt9d113*/
	GPIO_CFG(55, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* RESET FOR 4E1*/	
	GPIO_CFG(56, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* VCM FOR 4E1*/	
	GPIO_CFG(88, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* ov5647*/	
	#endif
	/* BU5D09497  lijuan 00152865  20100514 end> */
};
/*DTS2011042704563 penghai 20110427 end>*/

static uint32_t camera_off_gpio_fluid_table[] = {
	/* FLUID: CAM_VGA_RST_N */
	GPIO_CFG(31, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* FLUID: CAMIF_STANDBY */
	GPIO_CFG(CAM_STNDBY, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)
};

static uint32_t camera_on_gpio_fluid_table[] = {
	/* FLUID: CAM_VGA_RST_N */
	GPIO_CFG(31, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* FLUID: CAMIF_STANDBY */
	GPIO_CFG(CAM_STNDBY, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)
};

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

/*<BU5D08108, lijuan 00152865, 20100419 begin*/
#ifdef CONFIG_HUAWEI_CAMERA
/*< DTS2010062300810 lijianzhao 20100624 begin*/
 struct msm_camera_sensor_vreg sensor_vreg_array[] = {
    {
		.vreg_name   = "gp7",
		.mv	  = 2850,
		.always_on = 0,
	},    
    {
		.vreg_name   = "gp2",
		.mv	  = 1800,
		.always_on = 0,
	},

};
/*DTS2010062300810 lijianzhao 20100624 end >*/
static int32_t msm_camera_vreg_config_on(
       struct msm_camera_sensor_vreg *sensor_vreg,
       uint8_t vreg_num)
{
    struct vreg *vreg_handle;
    uint8_t temp_vreg_sum;
    int32_t rc;
    
    if(sensor_vreg_array == NULL)
    {
        return 0;
    }
    
    for(temp_vreg_sum = 0; temp_vreg_sum < vreg_num;temp_vreg_sum++)
    {
        vreg_handle = vreg_get(0, sensor_vreg[temp_vreg_sum].vreg_name);
    	if (!vreg_handle) {
    		printk(KERN_ERR "vreg_handle get failed\n");
    		return -EIO;
    	}
    	rc = vreg_set_level(vreg_handle, sensor_vreg[temp_vreg_sum].mv);
    	if (rc) {
    		printk(KERN_ERR "vreg_handle set level failed\n");
    		return -EIO;
    	}
    	rc = vreg_enable(vreg_handle);
    	if (rc) {
    		printk(KERN_ERR "vreg_handle enable failed\n");
    		return -EIO;
    	}
    }
    return 0;
}

static int32_t msm_camera_vreg_config_off(
       struct msm_camera_sensor_vreg *sensor_vreg,
       uint8_t vreg_num)
{
    struct vreg *vreg_handle;
    uint8_t temp_vreg_sum;
    int32_t rc;
    
    if(sensor_vreg == NULL)
    {
        return 0;
    }

    for(temp_vreg_sum = 0; temp_vreg_sum < vreg_num;temp_vreg_sum++)
    {
        if(sensor_vreg[temp_vreg_sum].always_on)
        {
            continue;
        }
        vreg_handle = vreg_get(0, sensor_vreg[temp_vreg_sum].vreg_name);
    	if (!vreg_handle) {
    		printk(KERN_ERR "vreg_handle get failed\n");
    		return -EIO;
    	}
    	rc = vreg_disable(vreg_handle);
    	if (rc) {
    		printk(KERN_ERR "vreg_handle disable failed\n");
    		return -EIO;
    	}
    }
    return 0;
}
#endif //CONFIG_HUAWEI_CAMERA
/* BU5D08108, lijuan 00152865, 20100419 end> */
static int config_camera_on_gpios(void)
{
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));

/*<DTS2011042704563 penghai 20110427 begin*/
/*deletes some lines*/
/*DTS2011042704563 penghai 20110427 end>*/
	if (machine_is_msm7x30_fluid()) {
		config_gpio_table(camera_on_gpio_fluid_table,
			ARRAY_SIZE(camera_on_gpio_fluid_table));
		/* FLUID: turn on 5V booster */
		gpio_set_value(
			PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_FLASH_BOOST_ENABLE), 1);
		/* FLUID: drive high to put secondary sensor to STANDBY */
		gpio_set_value(CAM_STNDBY, 1);
	}
	return 0;
}

static void config_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));

/*<DTS2011042704563 penghai 20110427 begin*/
/*deletes some lines*/
/*DTS2011042704563 penghai 20110427 end>*/

	if (machine_is_msm7x30_fluid()) {
		config_gpio_table(camera_off_gpio_fluid_table,
			ARRAY_SIZE(camera_off_gpio_fluid_table));
		/* FLUID: turn off 5V booster */
		gpio_set_value(
			PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_FLASH_BOOST_ENABLE), 0);
	}
}

struct resource msm_camera_resources[] = {
	{
		.start	= 0xA6000000,
		.end	= 0xA6000000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VFE,
		.end	= INT_VFE,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.flags  = IORESOURCE_DMA,
	}
};

struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.camifpadphy = 0xAB000000,
	.ioext.camifpadsz  = 0x00000400,
	.ioext.csiphy = 0xA6100000,
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = INT_CSI,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 122880000,
};

/*<DTS2010052800141 liqingshan 20100511 begin*/
/*description camera flash*/
/*<DTS2010101100312 lijuan 20101012 begin*/
/* < DTS2011080305465 zhangyu 20110803 begin */
/* Change the PWM freq 500Hz to 1500Hz */
/*raise the brightness of the first flash, it can help the AWB caculate of camera*/
static struct msm_camera_sensor_flash_src msm_flash_src_pwm = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_PWM,
	._fsrc.pwm_src.freq  = 1500,/*pwm freq*/
	._fsrc.pwm_src.max_load = 300,
	._fsrc.pwm_src.low_load = 100,/*low level*/
	._fsrc.pwm_src.high_load = 300,/*high level*/
	._fsrc.pwm_src.channel = 0,/*chanel id -> gpio num 24*/
};
/* DTS2011080305465 zhangyu 20110803 end > */ 
/*DTS2010101100312 lijuan 20101012 end>*/
/*DTS2010052800141 liqingshan 20100511 end>*/

#ifdef CONFIG_MT9D112
static struct msm_camera_sensor_flash_data flash_mt9d112 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src_pwm
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9d112_data = {
	.sensor_name    = "mt9d112",
	.sensor_reset   = 0,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_mt9d112,
	.csi_if         = 0
};

static struct platform_device msm_camera_sensor_mt9d112 = {
	.name      = "msm_camera_mt9d112",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9d112_data,
	},
};
#endif

/*< DTS2011051400258   songxiaoming 20110514 begin */
#ifdef CONFIG_HUAWEI_SENSOR_HI701
static struct msm_camera_sensor_flash_data flash_hi701 = {
	.flash_type = MSM_CAMERA_FLASH_NONE,
	.flash_src  = &msm_flash_src_pwm
};

static struct msm_camera_sensor_info msm_camera_sensor_hi701_data = {	
	.sensor_name	= "hi701",
	.sensor_reset   = 31,
	.sensor_vreg  = sensor_vreg_array,
    .vreg_num     = ARRAY_SIZE(sensor_vreg_array),
	.vreg_enable_func = msm_camera_vreg_config_on,
	.vreg_disable_func = msm_camera_vreg_config_off,
	.slave_sensor = 1,
	.sensor_pwd     = 52,
	.vcm_pwd        = 0,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.flash_data     = &flash_hi701,
	.num_resources  = ARRAY_SIZE(msm_camera_resources)
    //    .master_init_control_slave = sensor_master_init_control_slave,
};

static struct platform_device msm_camera_sensor_hi701 = {
	.name      = "msm_camera_hi701",
	.dev       = {
		.platform_data = &msm_camera_sensor_hi701_data,
	},
};
#endif
/* DTS2011051400258   songxiaoming 20110514 end > */

/*<  DTS2011101000840   yuguangcai 20111010 begin */
#ifdef CONFIG_HUAWEI_SENSOR_S5K5CA
static struct msm_camera_sensor_flash_data flash_s5k5ca = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src_pwm
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k5ca_data = {	
	.sensor_name	= "s5k5ca",
	.sensor_reset   = 88,
	.sensor_vreg  = sensor_vreg_array,
       .vreg_num     = ARRAY_SIZE(sensor_vreg_array),
	.vreg_enable_func = msm_camera_vreg_config_on,
	.vreg_disable_func = msm_camera_vreg_config_off,
	.slave_sensor = 0,
	.sensor_pwd     = 55,
	.vcm_pwd        = 56,
	.vcm_enable     = 1,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.flash_data     = &flash_s5k5ca,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.csi_if         = 1
    //    .master_init_control_slave = sensor_master_init_control_slave,
};

static struct platform_device msm_camera_sensor_s5k5ca = {
	.name      = "msm_camera_s5k5ca",
	.dev       = {
		.platform_data = &msm_camera_sensor_s5k5ca_data,
	},
};
#endif
/* DTS2011101000840   yuguangcai 20111010 end > */
/*< DTS2011102805854 yuguangcai 20111029 begin */
#ifdef CONFIG_HUAWEI_SENSOR_MT9T113
static struct msm_camera_sensor_flash_data flash_mt9t113 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src_pwm
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9t113_data = {	
	.sensor_name	= "mt9t113",
	.sensor_reset   = 88,
	.sensor_vreg  = sensor_vreg_array,
       .vreg_num     = ARRAY_SIZE(sensor_vreg_array),
	.vreg_enable_func = msm_camera_vreg_config_on,
	.vreg_disable_func = msm_camera_vreg_config_off,
	.slave_sensor = 0,
	.sensor_pwd     = 55,
	.vcm_pwd        = 56,
	.vcm_enable     = 1,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.flash_data     = &flash_mt9t113,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.csi_if         = 1
	//    .master_init_control_slave = sensor_master_init_control_slave,
};

static struct platform_device msm_camera_sensor_mt9t113 = {
	.name      = "msm_camera_mt9t113",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9t113_data,
	},
};
#endif
/* DTS2011102805854 yuguangcai 20111029 end > */
/*<BU5D08108, lijuan 00152865, 20100419 begin*/
#ifdef CONFIG_HUAWEI_SENSOR_OV7690
static struct msm_camera_sensor_flash_data flash_ov7690 = {
	.flash_type = MSM_CAMERA_FLASH_NONE,
	.flash_src  = &msm_flash_src_pwm
};

static struct msm_camera_sensor_info msm_camera_sensor_ov7690_data = {
	/*<DTS2011042704563 penghai 20110427 begin*/
	/*<DTS2011031804440 penghai 20110318 begin*/
	.sensor_name    = "ov7690",
	/* DTS2011031804440 penghai 20110318 end>*/	
	/*DTS2011042704563 penghai 20110427 end>*/
	.sensor_reset   = 31,
	.sensor_vreg  = sensor_vreg_array,
    	.vreg_num     = ARRAY_SIZE(sensor_vreg_array),
	.vreg_enable_func = msm_camera_vreg_config_on,
	.vreg_disable_func = msm_camera_vreg_config_off,
	/*<BU5D08116, lijuan 00152865, 20100419 begin*/
	.slave_sensor = 1,
	/*<BU5D08116, lijuan 00152865, 20100419 end*/
	.sensor_pwd     = 0,
	.vcm_pwd        = 0,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.flash_data     = &flash_ov7690,
	.num_resources  = ARRAY_SIZE(msm_camera_resources)
};

static struct platform_device msm_camera_sensor_ov7690 = {
	.name      = "msm_camera_ov7690",
	.dev       = {
		.platform_data = &msm_camera_sensor_ov7690_data,
	},
};
#endif
/* BU5D08108, lijuan 00152865, 20100419 end> */

/*<BU5D10275  liyuping 00166065 20100529 begin*/
#ifdef CONFIG_HUAWEI_SENSOR_HIMAX0356
static struct msm_camera_sensor_flash_data flash_himax0356 = {
	.flash_type = MSM_CAMERA_FLASH_NONE,
	.flash_src  = &msm_flash_src_pwm
};

static struct msm_camera_sensor_info msm_camera_sensor_himax0356_data = {
	/*<DTS2011042704563 penghai 20110427 begin*/
	/*<DTS2011031804440 penghai 20110318 begin*/
	.sensor_name    = "himax0356",
	/* DTS2011031804440 penghai 20110318 end>*/	
	/*DTS2011042704563 penghai 20110427 end>*/
	.sensor_reset   = 31,
	.sensor_vreg  = sensor_vreg_array,
    .vreg_num     = ARRAY_SIZE(sensor_vreg_array),
	.vreg_enable_func = msm_camera_vreg_config_on,
	.vreg_disable_func = msm_camera_vreg_config_off,
	.slave_sensor = 1,
	.sensor_pwd     = 0,
	.vcm_pwd        = 0,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.flash_data     = &flash_himax0356,
	.num_resources  = ARRAY_SIZE(msm_camera_resources)
};

static struct platform_device msm_camera_sensor_himax0356 = {
	.name      = "msm_camera_himax0356",
	.dev       = {
		.platform_data = &msm_camera_sensor_himax0356_data,
	},
};
#endif 


/*<DTS2011042704563 penghai 20110427 begin*/
#ifdef CONFIG_HUAWEI_SENSOR_MT9D113
static struct msm_camera_sensor_flash_data flash_mt9d113 = {
	.flash_type = MSM_CAMERA_FLASH_NONE,
	.flash_src  = &msm_flash_src_pwm
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9d113_data = {
	/*<DTS2011022300749 zhangyu 20110223 begin*/	
	.sensor_name	= "mt9d113",
	/*DTS2011022300749 zhangyu 20110223 end>*/
	.sensor_reset   = 31,
	.sensor_vreg  = sensor_vreg_array,
    .vreg_num     = ARRAY_SIZE(sensor_vreg_array),
	.vreg_enable_func = msm_camera_vreg_config_on,
	.vreg_disable_func = msm_camera_vreg_config_off,
	.slave_sensor = 1,
	.sensor_pwd     = 52,
	.vcm_pwd        = 0,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.flash_data     = &flash_mt9d113,
	.num_resources  = ARRAY_SIZE(msm_camera_resources)
    //    .master_init_control_slave = sensor_master_init_control_slave,
};

static struct platform_device msm_camera_sensor_mt9d113 = {
	.name      = "msm_camera_mt9d113",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9d113_data,
	},
};
#endif
/*DTS2011042704563 penghai 20110427 end>*/

/* < DTS2011061801881 zhangyu 20110620 BEGIN */
#ifdef CONFIG_HUAWEI_SENSOR_MT9V114
static struct msm_camera_sensor_flash_data flash_mt9v114 = {
	.flash_type = MSM_CAMERA_FLASH_NONE,
	.flash_src  = &msm_flash_src_pwm
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9v114_data = {
	.sensor_name	= "mt9v114_sunny",
	.sensor_reset   = 31,
	.sensor_vreg  = sensor_vreg_array,
    .vreg_num     = ARRAY_SIZE(sensor_vreg_array),
	.vreg_enable_func = msm_camera_vreg_config_on,
	.vreg_disable_func = msm_camera_vreg_config_off,
	.slave_sensor = 1,
	.sensor_pwd     = 52,
	.vcm_pwd        = 0,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.flash_data     = &flash_mt9v114,
	.num_resources  = ARRAY_SIZE(msm_camera_resources)
};

static struct platform_device msm_camera_sensor_mt9v114 = {
	.name      = "msm_camera_mt9v114_sunny",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9v114_data,
	},
};
#endif
/* DTS2011061801881 zhangyu 20110620 END > */ 

/*< DTS2011072600848   songxiaoming 20110726 begin */
#ifdef CONFIG_HUAWEI_SENSOR_OV7736
static struct msm_camera_sensor_flash_data flash_ov7736 = {
	.flash_type = MSM_CAMERA_FLASH_NONE,
	.flash_src  = &msm_flash_src_pwm
};

static struct msm_camera_sensor_info msm_camera_sensor_ov7736_data = {
	.sensor_name	= "ov7736",
	.sensor_reset   = 31,
	.sensor_vreg  = sensor_vreg_array,
    .vreg_num     = ARRAY_SIZE(sensor_vreg_array),
	.vreg_enable_func = msm_camera_vreg_config_on,
	.vreg_disable_func = msm_camera_vreg_config_off,
	.slave_sensor = 1,
	.sensor_pwd     = 52,
	.vcm_pwd        = 0,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.flash_data     = &flash_ov7736,
	.num_resources  = ARRAY_SIZE(msm_camera_resources)
    //    .master_init_control_slave = sensor_master_init_control_slave,
};

static struct platform_device msm_camera_sensor_ov7736 = {
	.name      = "msm_camera_ov7736",
	.dev       = {
		.platform_data = &msm_camera_sensor_ov7736_data,
	},
};
#endif
/* DTS2011072600848   songxiaoming 20110726 end > */

/* < DTS2010102600914 zhangtao 20101026 begin */
#ifdef CONFIG_HUAWEI_LEDS_PMIC
static struct platform_device msm_device_pmic_leds = {
	.name   = "pmic-leds",
	.id = -1,
};
#endif
/* DTS2010102600914 zhangtao 20101026 end > */


/*BU5D10275  liyuping  00166065 20100529 end> */

/*<BU5D10443, lijuan 00152865, 20100524 begin*/
#ifdef CONFIG_HUAWEI_SENSOR_S5K4E1GX
static struct msm_camera_sensor_flash_data flash_s5k4e1gx = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src_pwm
};

/* < DTS2011071802246 zhangyu 20110718 BEGIN */
static struct msm_camera_sensor_info msm_camera_sensor_s5k4e1gx_data = {
	/*<DTS2011042704563 penghai 20110427 begin*/
	/*<DTS2011031804440 penghai 20110318 begin*/
	.sensor_name    = "s5k4e1gx",
	/* DTS2011031804440 penghai 20110318 end>*/	
	/*DTS2011042704563 penghai 20110427 end>*/
	/*<  DTS2011101302587   yuguangcai 20111013 begin */
	.sensor_reset   = 88,
	/* DTS2011101302587   yuguangcai 20111013 end > */
	.sensor_vreg  = sensor_vreg_array,
	.vreg_num     = ARRAY_SIZE(sensor_vreg_array),
	.vreg_enable_func = msm_camera_vreg_config_on,
	.vreg_disable_func = msm_camera_vreg_config_off,
	.slave_sensor = 0,
	.sensor_pwd     = 0,
	.vcm_pwd        = 56,
	.vcm_enable     = 1,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.flash_data     = &flash_s5k4e1gx,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.csi_if         = 1
};
/* DTS2011071802246 zhangyu 20110718 END > */ 

static struct platform_device msm_camera_sensor_s5k4e1gx = {
	.name = "msm_camera_s5k4e1gx",
	.dev = {
		.platform_data = &msm_camera_sensor_s5k4e1gx_data,
	},
};
#endif
/* BU5D10443, lijuan 00152865, 20100524 end >*/

/* < DTS2011100802746 zhangyu 20111010 begin */
#ifdef CONFIG_HUAWEI_SENSOR_S5K4E1GX_P
static struct msm_camera_sensor_flash_data flash_s5k4e1gx_p = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src_pwm
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k4e1gx_p_data = {
	.sensor_name    = "s5k4e1gx_p",
	/*<  DTS2011101302587   yuguangcai 20111013 begin */
	/*< DTS2011110104318 zhangyu 20111103 begin */
	.sensor_reset   = 55,
	/* DTS2011110104318 zhangyu 20111103 end > */
	/* DTS2011101302587   yuguangcai 20111013 end > */
	.sensor_vreg  = sensor_vreg_array,
	.vreg_num     = ARRAY_SIZE(sensor_vreg_array),
	.vreg_enable_func = msm_camera_vreg_config_on,
	.vreg_disable_func = msm_camera_vreg_config_off,
	.slave_sensor = 0,
	.sensor_pwd     = 0,
	.vcm_pwd        = 56,
	.vcm_enable     = 1,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.flash_data     = &flash_s5k4e1gx_p,
	.num_resources  = ARRAY_SIZE(msm_camera_resources)
};

static struct platform_device msm_camera_sensor_s5k4e1gx_p = {
	.name = "msm_camera_s5k4e1gx_p",
	.dev = {
		.platform_data = &msm_camera_sensor_s5k4e1gx_p_data,
	},
};
#endif
/* DTS2011100802746 zhangyu 20111010 end > */ 

/* < DTS2011090701903 zhangyu 20110907 begin */
#ifdef CONFIG_HUAWEI_SENSOR_MT9P017
static struct msm_camera_sensor_flash_data flash_mt9p017 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src_pwm
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9p017_data = {
	.sensor_name    = "mt9p017",
	.sensor_reset   = 88,
	.sensor_pwd     = 55,
	.vcm_pwd        = 56,
	.vcm_enable     = 1,
	.pdata          = &msm_camera_device_data,
	.flash_data     = &flash_mt9p017,
	.sensor_vreg  = sensor_vreg_array,
	.vreg_num     = ARRAY_SIZE(sensor_vreg_array),
	.vreg_enable_func = msm_camera_vreg_config_on,
	.vreg_disable_func = msm_camera_vreg_config_off,
	.slave_sensor = 0, 	
	 .resource       = msm_camera_resources,        
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.csi_if         = 1
};

static struct platform_device msm_camera_sensor_mt9p017 = {
	.name      = "msm_camera_mt9p017",
	.id        = -1,
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9p017_data,
	},
};
#endif
/* DTS2011090701903 zhangyu 20110907 end > */ 

/*<BU5D09497  lijuan 00152865  20100514 begin*/
#ifdef CONFIG_HUAWEI_SENSOR_OV5647_SUNNY
static struct msm_camera_sensor_flash_data flash_ov5647_sunny = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src_pwm
};

static struct msm_camera_sensor_info msm_camera_sensor_ov5647_sunny_data = {
	/*<DTS2011042704563 penghai 20110427 begin*/
	/*<DTS2011031804440 penghai 20110318 begin*/
	.sensor_name    = "ov5647_sunny",
	/* DTS2011031804440 penghai 20110318 end>*/	
	/*DTS2011042704563 penghai 20110427 end>*/
	.sensor_reset   = 88,
	.sensor_vreg  = sensor_vreg_array,
	.vreg_num     = ARRAY_SIZE(sensor_vreg_array),
	.vreg_enable_func = msm_camera_vreg_config_on,
	.vreg_disable_func = msm_camera_vreg_config_off,
	.slave_sensor = 0,
	.sensor_pwd     = 55,
	.vcm_pwd        = 56,
	.vcm_enable     = 1,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.flash_data     = &flash_ov5647_sunny,
	.num_resources  = ARRAY_SIZE(msm_camera_resources)
};

static struct platform_device msm_camera_sensor_ov5647_sunny = {
	.name = "msm_camera_ov5647_sunny",
	.dev = {
		.platform_data = &msm_camera_sensor_ov5647_sunny_data,
	},
};
#endif
/* BU5D09497  lijuan 00152865  20100514 end> */

/*<DTS2011042704563 penghai 20110427 begin*/
#ifdef CONFIG_HUAWEI_SENSOR_MT9E013
static struct msm_camera_sensor_flash_data flash_mt9e013 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src_pwm
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9e013_data = {
	.sensor_name    = "mt9e013",
	.sensor_vreg  = sensor_vreg_array,
	.vreg_num     = ARRAY_SIZE(sensor_vreg_array),
	.vreg_enable_func = msm_camera_vreg_config_on,
	.vreg_disable_func = msm_camera_vreg_config_off,
	.slave_sensor   = 0,
	.sensor_reset   = 88,
	.sensor_pwd     = 55,
	.vcm_pwd        = 56,
	.vcm_enable     = 1,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_mt9e013,
	.csi_if         = 1
};

static struct platform_device msm_camera_sensor_mt9e013 = {
	.name      = "msm_camera_mt9e013",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9e013_data,
	},
};
#endif
/*DTS2011042704563 penghai 20110427 end>*/


#ifdef CONFIG_WEBCAM_OV9726
static struct msm_camera_sensor_flash_data flash_ov9726 = {
	.flash_type	= MSM_CAMERA_FLASH_LED,
	.flash_src	= &msm_flash_src_pwm
};
static struct msm_camera_sensor_info msm_camera_sensor_ov9726_data = {
	.sensor_name	= "ov9726",
	.sensor_reset	= 0,
	.sensor_pwd	= 85,
	.vcm_pwd	= 1,
	.vcm_enable	= 0,
	.pdata		= &msm_camera_device_data,
	.resource	= msm_camera_resources,
	.num_resources	= ARRAY_SIZE(msm_camera_resources),
	.flash_data	= &flash_ov9726,
	.csi_if		= 1
};
struct platform_device msm_camera_sensor_ov9726 = {
	.name	= "msm_camera_ov9726",
	.dev	= {
		.platform_data = &msm_camera_sensor_ov9726_data,
	},
};
#endif

#ifdef CONFIG_S5K3E2FX
static struct msm_camera_sensor_flash_data flash_s5k3e2fx = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src_pwm,
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k3e2fx_data = {
	.sensor_name    = "s5k3e2fx",
	.sensor_reset   = 0,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_s5k3e2fx,
	.csi_if         = 0
};

static struct platform_device msm_camera_sensor_s5k3e2fx = {
	.name      = "msm_camera_s5k3e2fx",
	.dev       = {
		.platform_data = &msm_camera_sensor_s5k3e2fx_data,
	},
};
#endif

#ifdef CONFIG_MT9P012
static struct msm_camera_sensor_flash_data flash_mt9p012 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src_pwm
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9p012_data = {
	.sensor_name    = "mt9p012",
	.sensor_reset   = 0,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable     = 1,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_mt9p012,
	.csi_if         = 0
};

static struct platform_device msm_camera_sensor_mt9p012 = {
	.name      = "msm_camera_mt9p012",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9p012_data,
	},
};
#endif

/*<DTS2011042704563 penghai 20110427 begin*/
/*deletes some lines*/
/*DTS2011042704563 penghai 20110427 end>*/
#ifdef CONFIG_VX6953
static struct msm_camera_sensor_flash_data flash_vx6953 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src_pwm
};
static struct msm_camera_sensor_info msm_camera_sensor_vx6953_data = {
	.sensor_name    = "vx6953",
	.sensor_reset   = 0,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable		= 0,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_vx6953,
	.csi_if         = 1
};
static struct platform_device msm_camera_sensor_vx6953 = {
	.name  	= "msm_camera_vx6953",
	.dev   	= {
		.platform_data = &msm_camera_sensor_vx6953_data,
	},
};
#endif

#ifdef CONFIG_SN12M0PZ
static struct msm_camera_sensor_flash_src msm_flash_src_current_driver = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_CURRENT_DRIVER,
	._fsrc.current_driver_src.low_current = 210,
	._fsrc.current_driver_src.high_current = 700,
	._fsrc.current_driver_src.driver_channel = &pm8058_fluid_leds_data,
};

static struct msm_camera_sensor_flash_data flash_sn12m0pz = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src_current_driver
};
static struct msm_camera_sensor_info msm_camera_sensor_sn12m0pz_data = {
	.sensor_name    = "sn12m0pz",
	.sensor_reset   = 0,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable     = 1,
	.pdata          = &msm_camera_device_data,
	.flash_data     = &flash_sn12m0pz,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.csi_if         = 0
};

static struct platform_device msm_camera_sensor_sn12m0pz = {
	.name      = "msm_camera_sn12m0pz",
	.dev       = {
		.platform_data = &msm_camera_sensor_sn12m0pz_data,
	},
};
#endif
/* < DTS2011052606009 jiaxianghong 20110527 begin */
/* <DTS2011032104626 shenjinming 20110321 begin */
/* driver for hw device detect */
static struct platform_device huawei_device_detect = {
	.name = "hw-dev-detect",
	.id		= -1,
};
/* <DTS2011032104626 shenjinming 20110321 end> */
/*< DTS2011093001643 lijianzhao 20110930 begin */
#ifdef CONFIG_HUAWEI_KERNEL
static struct gpio_event_direct_entry hw_slide_map[] = {
	{GPIO_SLIDE_DETECT ,SW_LID}
};

static struct gpio_event_input_info hw_slide_info = {
	.info.func = gpio_event_input_func,
	/*< DTS2011100905665 lijianzhao 20111009 begin */
	/* 0 indicate switch on ,1 indicate switch off . so match android code */
	.flags = GPIOEDF_ACTIVE_HIGH,
	/* DTS2011100905665 lijianzhao 20111009 end >*/
	.type = EV_SW,
	.keymap = hw_slide_map,
	.keymap_size = ARRAY_SIZE(hw_slide_map)
};

static struct gpio_event_info *hw_slide_event_info[] = {	
	&hw_slide_info.info,
};

static struct gpio_event_platform_data hw_slide_data = {
	.name = "hw_slide_detect",
	.info = hw_slide_event_info,
	.info_count = ARRAY_SIZE(hw_slide_event_info)
};

static struct platform_device hw_slide_device = {
	.name = GPIO_EVENT_DEV_NAME,
	.id = 0,
	.dev	= {
		.platform_data	= &hw_slide_data,
	},
};
/* add hall device */
static void __init add_slide_detect_device(void)
{
	int ret;
	/* U8730 support hall device only,at present */
	if(machine_is_msm8255_u8730())
	{
		ret = gpio_tlmm_config(GPIO_CFG(GPIO_SLIDE_DETECT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA)
						, GPIO_CFG_ENABLE);
		if (ret) 
		{
			pr_err("%s: gpio_tlmm_config(GPIO_%d)=%d\n",
				__func__, GPIO_SLIDE_DETECT, ret);
			return;
		}
		platform_device_register(&hw_slide_device);
	}
}
#endif
/* DTS2011093001643 lijianzhao 20110930 end >*/
#ifdef CONFIG_MT9T013
static struct msm_camera_sensor_flash_data flash_mt9t013 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src_pwm
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9t013_data = {
	.sensor_name    = "mt9t013",
	.sensor_reset   = 0,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_mt9t013,
	.csi_if         = 1
};

static struct platform_device msm_camera_sensor_mt9t013 = {
	.name      = "msm_camera_mt9t013",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9t013_data,
	},
};
#endif

#ifdef CONFIG_MSM_GEMINI
static struct resource msm_gemini_resources[] = {
	{
		.start  = 0xA3A00000,
		.end    = 0xA3A00000 + 0x0150 - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = INT_JPEG,
		.end    = INT_JPEG,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device msm_gemini_device = {
	.name           = "msm_gemini",
	.resource       = msm_gemini_resources,
	.num_resources  = ARRAY_SIZE(msm_gemini_resources),
};
#endif

#ifdef CONFIG_MSM_VPE
static struct resource msm_vpe_resources[] = {
	{
		.start	= 0xAD200000,
		.end	= 0xAD200000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VPE,
		.end	= INT_VPE,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device msm_vpe_device = {
       .name = "msm_vpe",
       .id   = 0,
       .num_resources = ARRAY_SIZE(msm_vpe_resources),
       .resource = msm_vpe_resources,
};
#endif

#endif /*CONFIG_MSM_CAMERA*/
/*<BU5D08178, sibingsong 20100416 begin*/ 
#ifdef CONFIG_HUAWEI_FEATURE_RGB_KEY_LIGHT
static struct platform_device rgb_leds_device = {
	.name   = "rgb-leds",
	.id     = 0,
};
#endif
/*BU5D08178, sibingsong 20100416 end>*/

/* < DTS2010111603970 zhangtao 20101116 begin */
/*init the PTT Light attr*/
#ifdef CONFIG_HUAWEI_FEATURE_PTT_KEY_LIGHT
static struct platform_device ptt_led_driver = {
	.name   = "ptt-led",
	.id     = 0,
};
#endif
/* DTS2010111603970 zhangtao 20101116 end > */
#ifdef CONFIG_MSM7KV2_AUDIO
/* <BU5D07963, audio params change, lgq 39739 0419 begin */
static uint32_t audio_pamp_gpio_config =
   GPIO_CFG(82, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA);
/* BU5D07963, audio params change, lgq 39739 0419 end > */

/*<DTS2011060201308 yanghaimin 20110602, begin*/
/* u8860 add hac gpio ctl */
static uint32_t audio_hac_gpio_config =
   GPIO_CFG(0xFF, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA);
static unsigned audio_hac_gpio = 0xFF;
/* DTS2011060201308 yanghaimin 20110602, end>*/

static uint32_t audio_fluid_icodec_tx_config =
  GPIO_CFG(85, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);

static int __init snddev_poweramp_gpio_init(void)
{
	int rc;

	pr_info("snddev_poweramp_gpio_init \n");
	rc = gpio_tlmm_config(audio_pamp_gpio_config, GPIO_CFG_ENABLE);
	if (rc) {
		printk(KERN_ERR
			"%s: gpio_tlmm_config(%#x)=%d\n",
			__func__, audio_pamp_gpio_config, rc);
	}
	return rc;
}

/*<DTS2011060201308 yanghaimin 20110602, begin*/
/* u8860 add hac gpio ctl */
static int __init snddev_hac_gpio_init(void)
{
    int rc;

    pr_info("snddev_hac_gpio_init \n");
    if ( (machine_is_msm7x30_u8800_51())) 
    {
        audio_hac_gpio = 33;
        audio_hac_gpio_config = GPIO_CFG(33, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA);
    /*< DTS2011070901815  sunhonghui 20110709 begin */
    /* <DTS2011071600361 liyuping 20110716 begin */
	/* < DTS2011082302564 liwei 20110823 begin */
	/*<DTS2011091502092 liyuping 20110915 begin */
    /* <DTS2011091200073 zhangbo 20110912 begin */
	/* < DTS2011102401822 liwei 20111024 begin */
    } else if (machine_is_msm8255_u8860() 
    			|| machine_is_msm8255_u8860lp()
    			|| machine_is_msm8255_u8860_92()
                || machine_is_msm8255_u8680()
                || machine_is_msm8255_u8667()
				|| machine_is_msm8255_u8860_51()
				|| machine_is_msm8255_u8730())
	/* DTS2011102401822 liwei 20111024 end > */
    /* DTS2011091200073 zhangbo 20110912 end> */
	/* DTS2011091502092 liyuping 20110915 end> */             
	/* DTS2011082302564 liwei 20110823 end > */
	/* DTS2011071600361 liyuping 20110716 end> */
    {
    /*DTS2011070901815  sunhonghui 20110709 end >*/
        audio_hac_gpio = 181;
        audio_hac_gpio_config = GPIO_CFG(181, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA);
    }

    /* no gpio ctl */
    if(0xFF == audio_hac_gpio){
        printk(KERN_ERR
            "%s: gpio_tlmm_config(%#x), gpio=%d\n",
            __func__, audio_hac_gpio_config, audio_hac_gpio);
        return 0;
    }
        
    /* config gpio */
    rc = gpio_tlmm_config(audio_hac_gpio_config, GPIO_CFG_ENABLE);
    if (rc) {
        printk(KERN_ERR
            "%s: gpio_tlmm_config(%#x)=%d\n",
            __func__, audio_hac_gpio_config, rc);
    }
    return rc;
}
/* DTS2011060201308 yanghaimin 20110602, end>*/
/*< BU5D08419 haoqingtao 20100424 begin*/    
void msm_snddev_tx_route_config(void)
{
	int rc;
    /*<DTS2011092905909 yinzhaoyang 20110929  begin*/
    struct vreg *   vreg_ldo13 = NULL;
    /* DTS2011092905909 yinzhaoyang 20110929  end >*/
	pr_debug("%s()\n", __func__);
/*< DTS2010092400487  lijianzhao 20100924 begin */
	/*< DTS2010112702297 wangquanli 201001125 begin */
	/*< DTS2011030202729  liliang 20110302  begin */	
	/*< DTS2011042703705 zhangbo 20110422 begin */
	/* <DTS2011050700551 zhangbo 20110505 begin */
    /*<DTS2011051601005 fengwei 20110516 begin*/
	if ( machine_is_msm7x30_fluid() 
	     || (machine_is_msm7x30_u8800()) 
	     || (machine_is_msm7x30_u8820()) 
	     || (machine_is_msm7x30_u8800_51()) 
	     || (machine_is_msm8255_u8800_pro()))  
	 {
     /*DTS2011051601005 fengwei 20110516 end>*/
	 /* DTS2011050700551 zhangbo 20110505 end> */
	/* DTS2011042703705 zhangbo 20110422 end >*/
	/* DTS2011030202729  liliang 20110302 end >*/
	/* DTS2010112702297 wangquanli 201001125 end >*/
		rc = gpio_tlmm_config(audio_fluid_icodec_tx_config,
		GPIO_CFG_ENABLE);
		if (rc) {
			printk(KERN_ERR
				"%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, audio_fluid_icodec_tx_config, rc);
		} else
			gpio_set_value(85, 0);
	}
/* DTS2010092400487  lijianzhao 20100924 end >*/

    /*<DTS2011092905909 yinzhaoyang 20110929  begin*/
    /*enable L13 for master and second MIC power */
    if (machine_is_msm8255_u8730())
    {
      vreg_ldo13 = vreg_get(NULL, "wlan");
      if (!vreg_ldo13) 
      {
          pr_err("%s: VREG L13 get failed\n", __func__);
	
      }
      else
      {
          pr_err("%s: VREG L13 get succ \n", __func__);
		  rc = vreg_set_level(vreg_ldo13, 2900);
          if (rc)
          {
              pr_err("%s: VREG L13 set failed\n", __func__);
          }
          else
          {
              pr_err("%s: VREG L13set succ \n", __func__);
          }

          rc = vreg_enable(vreg_ldo13);
          if (rc)
          {
              pr_err("%s: VREG L13 enable failed\n", __func__);
          }
          else
          {
              pr_err("%s: VREG L13 enable succ \n", __func__);
          }
      }	      
    }
    /* DTS2011092905909 yinzhaoyang 20110929  end >*/
}

void msm_snddev_tx_route_deconfig(void)
{
	int rc;
    /*<DTS2011092905909 yinzhaoyang 20110929  begin*/
    struct vreg *   vreg_ldo13 = NULL;
    /* DTS2011092905909 yinzhaoyang 20110929  end >*/
	pr_debug("%s()\n", __func__);
/*< DTS2010092400487  lijianzhao 20100924 begin */
	/*< DTS2010112702297 wangquanli 201001125 begin */
	/*< DTS2011030202729  liliang 20110302  begin */
	/*< DTS2011042703705 zhangbo 20110422 begin */
	/* <DTS2011050700551 zhangbo 20110505 begin */
    /*<DTS2011051601005 fengwei 20110516 begin*/
	if ( machine_is_msm7x30_fluid() 
	    || (machine_is_msm7x30_u8800()) 
	    || (machine_is_msm7x30_u8820()) 
	    || (machine_is_msm7x30_u8800_51()) 
	    || (machine_is_msm8255_u8800_pro()))  
	{
	/*DTS2011051601005  fengwei 20110516 end>*/
	/* DTS2011050700551 zhangbo 20110505 end> */
	/* DTS2011042703705 zhangbo 20110422 end >*/
	/* DTS2011030202729  liliang 20110302 end >*/
	/* DTS2010112702297 wangquanli 201001125 end >*/
		rc = gpio_tlmm_config(audio_fluid_icodec_tx_config,
		GPIO_CFG_DISABLE);
		if (rc) {
			printk(KERN_ERR
				"%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, audio_fluid_icodec_tx_config, rc);
		}
	}
/* DTS2010092400487  lijianzhao 20100924 end >*/
    /*<DTS2011092905909 yinzhaoyang 20110929  begin*/
    /*disable L13 for master and second MIC power */
    if (machine_is_msm8255_u8730())
    {
      vreg_ldo13 = vreg_get(NULL, "wlan");
      if (!vreg_ldo13) 
      {
          pr_err("%s: VREG L13 get failed\n", __func__);
	      
      }
      else
	  {
          pr_err("%s: VREG L13 get succ \n", __func__);
		  rc = vreg_disable(vreg_ldo13);
	      if (rc)
	      {
	         pr_err("%s: VREG L13 disable failed\n", __func__);
	      }
	      else
	      {
	         pr_err("%s: VREG L13 disable succ \n", __func__);
	      }
      }
    }
    /* DTS2011092905909 yinzhaoyang 20110929  end >*/
}
/* < DTS2010061100003 luojianhong 201000612 begin*/
static struct amplifier_platform_data audio_amplifier_data = {
    .amplifier_on = NULL,
    .amplifier_off = NULL,
    /*< DTS2010120204486 dongchen 20101223 begin */
    #ifdef CONFIG_HUAWEI_KERNEL
    .amplifier_4music_on = NULL,
    #endif
    /* DTS2010120204486 dongchen 20101223 end >*/
    
};
void msm_snddev_poweramp_on(void)
{
	gpio_set_value(82, 1);	/* enable spkr poweramp */
    if(audio_amplifier_data.amplifier_on)
        audio_amplifier_data.amplifier_on();
/* < DTS2010092501047 luojianhong 201000926 begin*/
    if(right_audio_amplifier_data.amplifier_on)
		right_audio_amplifier_data.amplifier_on();
/*  DTS2010092501047 luojianhong 201000926 end > */
	pr_info("%s: power on amplifier\n", __func__);

/* <DTS2010081400556 shenjinming 20100814 begin */
/*<	DTS2010071902252 shenjinming 20100719 begin	*/
#ifdef CONFIG_HUAWEI_EVALUATE_POWER_CONSUMPTION	
    /* start calculate speaker consume */
    huawei_rpc_current_consuem_notify(EVENT_SPEAKER_STATE, SPEAKER_ON_STATE );
#endif
/* DTS2010071902252	shenjinming	20100719 end >*/
/* DTS2010081400556 shenjinming 20100814 end> */
    
}

void msm_snddev_poweramp_off(void)
{
    if(audio_amplifier_data.amplifier_off)
        audio_amplifier_data.amplifier_off();
/* < DTS2010092501047 luojianhong 201000926 begin*/
	if(right_audio_amplifier_data.amplifier_off)
        right_audio_amplifier_data.amplifier_off();
/*  DTS2010092501047 luojianhong 201000926 end > */
	gpio_set_value(82, 0);	/* disable spkr poweramp */
	pr_info("%s: power off amplifier\n", __func__);
/* <DTS2010081400556 shenjinming 20100814 begin */
/*<	DTS2010071902252 shenjinming 20100719 begin	*/
#ifdef CONFIG_HUAWEI_EVALUATE_POWER_CONSUMPTION	
    /* stop calculate speaker consume */
    huawei_rpc_current_consuem_notify(EVENT_SPEAKER_STATE, SPEAKER_OFF_STATE );
#endif
/* DTS2010071902252	shenjinming	20100719 end >*/
/* DTS2010081400556 shenjinming 20100814 end> */

}
/*  DTS2010061100003 luojianhong 201000612 end > */

/*< DTS2010120204486 dongchen 20101223 begin */
#ifdef CONFIG_HUAWEI_KERNEL
void msm_snddev_poweramp_4music_on(void)
{
	gpio_set_value(82, 1);	/* enable spkr poweramp */
    if(audio_amplifier_data.amplifier_4music_on)
        audio_amplifier_data.amplifier_4music_on();
    if(right_audio_amplifier_data.amplifier_4music_on)
		right_audio_amplifier_data.amplifier_4music_on();
	pr_info("%s: power on amplifier for music\n", __func__);

#ifdef CONFIG_HUAWEI_EVALUATE_POWER_CONSUMPTION	
    /* start calculate speaker consume */
    huawei_rpc_current_consuem_notify(EVENT_SPEAKER_STATE, SPEAKER_ON_STATE );
#endif

}
#endif
/* DTS2010120204486 dongchen 20101223 end >*/

/*<DTS2011060201308 yanghaimin 20110602, begin*/
/* u8860 add hac gpio ctl */
#ifdef CONFIG_HUAWEI_KERNEL
void msm_snddev_hac_on(void)
{
    if(0xFF != audio_hac_gpio){
        gpio_set_value(audio_hac_gpio, 1);	/* enable hac parallel inductance */
        pr_info("%s: enable hac gpio %d\n", __func__, audio_hac_gpio);
    }
}

void msm_snddev_hac_off(void)
{
    if(0xFF != audio_hac_gpio){
        gpio_set_value(audio_hac_gpio, 0);	/* disable hac parallel inductance */
        pr_info("%s: disable hac gpio %d\n", __func__, audio_hac_gpio);
    }
}
#endif
/* DTS2011060201308 yanghaimin 20110602, end>*/

static struct vreg *snddev_vreg_ncp, *snddev_vreg_gp4;

void msm_snddev_hsed_voltage_on(void)
{
	int rc;

	snddev_vreg_gp4 = vreg_get(NULL, "gp4");
	if (IS_ERR(snddev_vreg_gp4)) {
		pr_err("%s: vreg_get(%s) failed (%ld)\n",
		__func__, "gp4", PTR_ERR(snddev_vreg_gp4));
		return;
	}
	rc = vreg_enable(snddev_vreg_gp4);
	if (rc)
		pr_err("%s: vreg_enable(gp4) failed (%d)\n", __func__, rc);

	snddev_vreg_ncp = vreg_get(NULL, "ncp");
	if (IS_ERR(snddev_vreg_ncp)) {
		pr_err("%s: vreg_get(%s) failed (%ld)\n",
		__func__, "ncp", PTR_ERR(snddev_vreg_ncp));
		return;
	}
	rc = vreg_enable(snddev_vreg_ncp);
	if (rc)
		pr_err("%s: vreg_enable(ncp) failed (%d)\n", __func__, rc);
}

void msm_snddev_hsed_voltage_off(void)
{
	int rc;

	if (IS_ERR(snddev_vreg_ncp)) {
		pr_err("%s: vreg_get(%s) failed (%ld)\n",
		__func__, "ncp", PTR_ERR(snddev_vreg_ncp));
		return;
	}
	rc = vreg_disable(snddev_vreg_ncp);
	if (rc)
		pr_err("%s: vreg_disable(ncp) failed (%d)\n", __func__, rc);
	vreg_put(snddev_vreg_ncp);

	if (IS_ERR(snddev_vreg_gp4)) {
		pr_err("%s: vreg_get(%s) failed (%ld)\n",
		__func__, "gp4", PTR_ERR(snddev_vreg_gp4));
		return;
	}
	rc = vreg_disable(snddev_vreg_gp4);
	if (rc)
		pr_err("%s: vreg_disable(gp4) failed (%d)\n", __func__, rc);

	vreg_put(snddev_vreg_gp4);

}

static unsigned aux_pcm_gpio_on[] = {
	GPIO_CFG(138, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_DOUT */
	GPIO_CFG(139, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_DIN  */
	GPIO_CFG(140, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_SYNC */
	GPIO_CFG(141, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_CLK  */
};

static int __init aux_pcm_gpio_init(void)
{
	int pin, rc;

	pr_info("aux_pcm_gpio_init \n");
	for (pin = 0; pin < ARRAY_SIZE(aux_pcm_gpio_on); pin++) {
		rc = gpio_tlmm_config(aux_pcm_gpio_on[pin],
					GPIO_CFG_ENABLE);
		if (rc) {
			printk(KERN_ERR
				"%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, aux_pcm_gpio_on[pin], rc);
		}
	}
	return rc;
}

static struct msm_gpio mi2s_clk_gpios[] = {
	{ GPIO_CFG(145, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_SCLK"},
	{ GPIO_CFG(144, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_WS"},
	{ GPIO_CFG(120, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_MCLK_A"},
};

static struct msm_gpio mi2s_rx_data_lines_gpios[] = {
	{ GPIO_CFG(121, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD0_A"},
	{ GPIO_CFG(122, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD1_A"},
	{ GPIO_CFG(123, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD2_A"},
	{ GPIO_CFG(146, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD3"},
};

static struct msm_gpio mi2s_tx_data_lines_gpios[] = {
	{ GPIO_CFG(146, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD3"},
};

int mi2s_config_clk_gpio(void)
{
	int rc = 0;

	rc = msm_gpios_request_enable(mi2s_clk_gpios,
			ARRAY_SIZE(mi2s_clk_gpios));
	if (rc) {
		pr_err("%s: enable mi2s clk gpios  failed\n",
					__func__);
		return rc;
	}
	return 0;
}

int  mi2s_unconfig_data_gpio(u32 direction, u8 sd_line_mask)
{
	int i, rc = 0;
	sd_line_mask &= MI2S_SD_LINE_MASK;

	switch (direction) {
	case DIR_TX:
		msm_gpios_disable_free(mi2s_tx_data_lines_gpios, 1);
		break;
	case DIR_RX:
		i = 0;
		while (sd_line_mask) {
			if (sd_line_mask & 0x1)
				msm_gpios_disable_free(
					mi2s_rx_data_lines_gpios + i , 1);
			sd_line_mask = sd_line_mask >> 1;
			i++;
		}
		break;
	default:
		pr_err("%s: Invaild direction  direction = %u\n",
						__func__, direction);
		rc = -EINVAL;
		break;
	}
	return rc;
}

int mi2s_config_data_gpio(u32 direction, u8 sd_line_mask)
{
	int i , rc = 0;
	u8 sd_config_done_mask = 0;

	sd_line_mask &= MI2S_SD_LINE_MASK;

	switch (direction) {
	case DIR_TX:
		if ((sd_line_mask & MI2S_SD_0) || (sd_line_mask & MI2S_SD_1) ||
		   (sd_line_mask & MI2S_SD_2) || !(sd_line_mask & MI2S_SD_3)) {
			pr_err("%s: can not use SD0 or SD1 or SD2 for TX"
				".only can use SD3. sd_line_mask = 0x%x\n",
				__func__ , sd_line_mask);
			rc = -EINVAL;
		} else {
			rc = msm_gpios_request_enable(mi2s_tx_data_lines_gpios,
							 1);
			if (rc)
				pr_err("%s: enable mi2s gpios for TX failed\n",
					   __func__);
		}
		break;
	case DIR_RX:
		i = 0;
		while (sd_line_mask && (rc == 0)) {
			if (sd_line_mask & 0x1) {
				rc = msm_gpios_request_enable(
					mi2s_rx_data_lines_gpios + i , 1);
				if (rc) {
					pr_err("%s: enable mi2s gpios for"
					 "RX failed.  SD line = %s\n",
					 __func__,
					 (mi2s_rx_data_lines_gpios + i)->label);
					mi2s_unconfig_data_gpio(DIR_RX,
						sd_config_done_mask);
				} else
					sd_config_done_mask |= (1 << i);
			}
			sd_line_mask = sd_line_mask >> 1;
			i++;
		}
		break;
	default:
		pr_err("%s: Invaild direction  direction = %u\n",
			__func__, direction);
		rc = -EINVAL;
		break;
	}
	return rc;
}

int mi2s_unconfig_clk_gpio(void)
{
	msm_gpios_disable_free(mi2s_clk_gpios, ARRAY_SIZE(mi2s_clk_gpios));
	return 0;
}

#endif /* CONFIG_MSM7KV2_AUDIO */

static int __init buses_init(void)
{
	if (gpio_tlmm_config(GPIO_CFG(PMIC_GPIO_INT, 1, GPIO_CFG_INPUT,
				  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
		pr_err("%s: gpio_tlmm_config (gpio=%d) failed\n",
		       __func__, PMIC_GPIO_INT);
/*< DTS2011042203570 n00111309 20110422 begin */
/* <DTS2010092702071 liugaofei 20100927 begin */
/* support U8820 keypad */
	/*< DTS2010112702297 wangquanli 201001125 begin */
	/*< DTS2011030202729  liliang 20110302  begin */		
	if (machine_is_msm7x30_fluid() || machine_is_msm7x30_u8800() || machine_is_msm7x30_u8820() || machine_is_msm7x30_u8800_51() || machine_is_msm8255_u8800_pro())  {
    /* DTS2011042203570 n00111309, do not add machine_is_msm8255_u8860() here.
	 * it will be handled seperately */
	/* DTS2011030202729  liliang 20110302 end >*/
	/* DTS2010112702297 wangquanli 201001125 end >*/
/* DTS2010092702071 liugaofei 20100927 end > */ 
		pm8058_7x30_data.sub_devices[PM8058_SUBDEV_KPD].platform_data
			= &fluid_keypad_data;
		pm8058_7x30_data.sub_devices[PM8058_SUBDEV_KPD].data_size
			= sizeof(fluid_keypad_data);
	 
	/* add for U8860, which do NOT use keypad row array .  */
	/* <DTS2011050700551 zhangbo 20110505 begin */
    /* <DTS2011062600102  sunhonghui 20110626 begin */	
    /* <DTS2011071600361 liyuping 20110716 begin */
	/* < DTS2011082302564 liwei 20110823 begin */
/*< DTS2011090104536 duanfei 20110901 begin */
	/*<DTS2011091502092 liyuping 20110915 begin */
    } else if ( machine_is_msm8255_u8860() 
			    || machine_is_msm8255_c8860() 
			    || machine_is_msm8255_u8860lp()
			    || machine_is_msm8255_u8860_92()
			    || machine_is_msm8255_u8860_51())
	/* DTS2011091502092 liyuping 20110915 end> */ 
/* DTS2011090104536 duanfei 20110901 end >*/
	/* DTS2011082302564 liwei 20110823 end > */
	/* DTS2011071600361 liyuping 20110716 end> */
    /* DTS2011062600102  sunhonghui 20110626 end> */
	/* DTS2011050700551 zhangbo 20110505 end> */
    {
        pm8058_7x30_data.sub_devices[PM8058_SUBDEV_KPD].platform_data
               = &fluid_norow_keypad_data;
        pm8058_7x30_data.sub_devices[PM8058_SUBDEV_KPD].data_size
               = sizeof(fluid_norow_keypad_data);
/*< DTS2011090104536 duanfei 20110901 begin */
    }
	/* < DTS2011102401822 liwei 20111024 begin */
    else if (machine_is_msm8255_u8680()
	    || machine_is_msm8255_u8667())
	/* DTS2011102401822 liwei 20111024 end > */
    {
    	pm8058_7x30_data.sub_devices[PM8058_SUBDEV_KPD].platform_data
			= &U8680_keypad_data;
		pm8058_7x30_data.sub_devices[PM8058_SUBDEV_KPD].data_size
			= sizeof(U8680_keypad_data);
    }
/*< DTS2011092601370 zhongjinrong 20110926 begin */
    else if(machine_is_msm8255_u8730())
    {
   	 	pm8058_7x30_data.sub_devices[PM8058_SUBDEV_KPD].platform_data
			= &u8730_keypad_data;
		pm8058_7x30_data.sub_devices[PM8058_SUBDEV_KPD].data_size
			= sizeof(u8730_keypad_data);
    }
/* DTS2011092601370 zhongjinrong 20119026 end >*/
    else {
/* DTS2011090104536 duanfei 20110901 end >*/
		pm8058_7x30_data.sub_devices[PM8058_SUBDEV_KPD].platform_data
			= &surf_keypad_data;
		pm8058_7x30_data.sub_devices[PM8058_SUBDEV_KPD].data_size
			= sizeof(surf_keypad_data);
	}
/* DTS2011042203570 n00111309 20110422 end */
	
	i2c_register_board_info(6 /* I2C_SSBI ID */, pm8058_boardinfo,
				ARRAY_SIZE(pm8058_boardinfo));

	return 0;
}

#define TIMPANI_RESET_GPIO	1

struct bahama_config_register{
	u8 reg;
	u8 value;
	u8 mask;
};

enum version{
	VER_1_0,
	VER_2_0,
	VER_UNSUPPORTED = 0xFF
};


static struct vreg *vreg_marimba_1;
static struct vreg *vreg_marimba_2;
static struct vreg *vreg_marimba_3;

static struct msm_gpio timpani_reset_gpio_cfg[] = {
{ GPIO_CFG(TIMPANI_RESET_GPIO, 0, GPIO_CFG_OUTPUT,
	GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "timpani_reset"} };

static u8 read_bahama_ver(void)
{
	int rc;
	struct marimba config = { .mod_id = SLAVE_ID_BAHAMA };
	u8 bahama_version;

	rc = marimba_read_bit_mask(&config, 0x00,  &bahama_version, 1, 0x1F);
	if (rc < 0) {
		printk(KERN_ERR
			 "%s: version read failed: %d\n",
			__func__, rc);
			return rc;
	} else {
		printk(KERN_INFO
		"%s: version read got: 0x%x\n",
		__func__, bahama_version);
	}

	switch (bahama_version) {
	case 0x08: /* varient of bahama v1 */
	case 0x10:
	case 0x00:
		return VER_1_0;
	case 0x09: /* variant of bahama v2 */
		return VER_2_0;
	default:
		return VER_UNSUPPORTED;
	}
}

static int config_timpani_reset(void)
{
	int rc;

	rc = msm_gpios_request_enable(timpani_reset_gpio_cfg,
				ARRAY_SIZE(timpani_reset_gpio_cfg));
	if (rc < 0) {
		printk(KERN_ERR
			"%s: msm_gpios_request_enable failed (%d)\n",
				__func__, rc);
	}
	return rc;
}

static unsigned int msm_timpani_setup_power(void)
{
	int rc;

	rc = config_timpani_reset();
	if (rc < 0)
		goto out;

	rc = vreg_enable(vreg_marimba_1);
	if (rc) {
		printk(KERN_ERR "%s: vreg_enable() = %d\n",
					__func__, rc);
		goto out;
	}
	rc = vreg_enable(vreg_marimba_2);
	if (rc) {
		printk(KERN_ERR "%s: vreg_enable() = %d\n",
					__func__, rc);
		goto fail_disable_vreg_marimba_1;
	}

	rc = gpio_direction_output(TIMPANI_RESET_GPIO, 1);
	if (rc < 0) {
		printk(KERN_ERR
			"%s: gpio_direction_output failed (%d)\n",
				__func__, rc);
		msm_gpios_free(timpani_reset_gpio_cfg,
				ARRAY_SIZE(timpani_reset_gpio_cfg));
		vreg_disable(vreg_marimba_2);
	} else
		goto out;


fail_disable_vreg_marimba_1:
	vreg_disable(vreg_marimba_1);

out:
	return rc;
};

static void msm_timpani_shutdown_power(void)
{
	int rc;

	rc = vreg_disable(vreg_marimba_1);
	if (rc) {
		printk(KERN_ERR "%s: return val: %d\n",
					__func__, rc);
	}
	rc = vreg_disable(vreg_marimba_2);
	if (rc) {
		printk(KERN_ERR "%s: return val: %d\n",
					__func__, rc);
	}

	rc = gpio_direction_output(TIMPANI_RESET_GPIO, 0);
	if (rc < 0) {
		printk(KERN_ERR
			"%s: gpio_direction_output failed (%d)\n",
				__func__, rc);
	}

	msm_gpios_free(timpani_reset_gpio_cfg,
				   ARRAY_SIZE(timpani_reset_gpio_cfg));
};

static unsigned int msm_bahama_core_config(int type)
{
	int rc = 0;

	if (type == BAHAMA_ID) {

		int i;
		struct marimba config = { .mod_id = SLAVE_ID_BAHAMA };

		const struct bahama_config_register v20_init[] = {
			/* reg, value, mask */
			{ 0xF4, 0x84, 0xFF }, /* AREG */
			{ 0xF0, 0x04, 0xFF } /* DREG */
		};

		if (read_bahama_ver() == VER_2_0) {
			for (i = 0; i < ARRAY_SIZE(v20_init); i++) {
				u8 value = v20_init[i].value;
				rc = marimba_write_bit_mask(&config,
					v20_init[i].reg,
					&value,
					sizeof(v20_init[i].value),
					v20_init[i].mask);
				if (rc < 0) {
					printk(KERN_ERR
						"%s: reg %d write failed: %d\n",
						__func__, v20_init[i].reg, rc);
					return rc;
				}
				printk(KERN_INFO "%s: reg 0x%02x value 0x%02x"
					" mask 0x%02x\n",
					__func__, v20_init[i].reg,
					v20_init[i].value, v20_init[i].mask);
			}
		}
	}
	printk(KERN_INFO "core type: %d\n", type);

	return rc;
}

static unsigned int msm_bahama_setup_power(void)
{
	int rc;

	rc = vreg_enable(vreg_marimba_3);
	if (rc) {
		printk(KERN_ERR "%s: vreg_enable() = %d\n",
				__func__, rc);
	}

	return rc;
};

static unsigned int msm_bahama_shutdown_power(int value)
{
	int rc = 0;

	if (value != BAHAMA_ID) {
		rc = vreg_disable(vreg_marimba_3);
		if (rc) {
			printk(KERN_ERR "%s: return val: %d\n",
					__func__, rc);
		}
	}

	return rc;
};

static struct msm_gpio marimba_svlte_config_clock[] = {
	{ GPIO_CFG(34, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		"MARIMBA_SVLTE_CLOCK_ENABLE" },
};

static unsigned int msm_marimba_gpio_config_svlte(int gpio_cfg_marimba)
{
	if (machine_is_msm8x55_svlte_surf() ||
		machine_is_msm8x55_svlte_ffa()) {
		if (gpio_cfg_marimba)
			gpio_set_value(GPIO_PIN
				(marimba_svlte_config_clock->gpio_cfg), 1);
		else
			gpio_set_value(GPIO_PIN
				(marimba_svlte_config_clock->gpio_cfg), 0);
	}

	return 0;
};

static unsigned int msm_marimba_setup_power(void)
{
	int rc;

	rc = vreg_enable(vreg_marimba_1);
	if (rc) {
		printk(KERN_ERR "%s: vreg_enable() = %d \n",
					__func__, rc);
		goto out;
	}
	rc = vreg_enable(vreg_marimba_2);
	if (rc) {
		printk(KERN_ERR "%s: vreg_enable() = %d \n",
					__func__, rc);
		goto out;
	}

	if (machine_is_msm8x55_svlte_surf() || machine_is_msm8x55_svlte_ffa()) {
		rc = msm_gpios_request_enable(marimba_svlte_config_clock,
				ARRAY_SIZE(marimba_svlte_config_clock));
		if (rc < 0) {
			printk(KERN_ERR
				"%s: msm_gpios_request_enable failed (%d)\n",
					__func__, rc);
			return rc;
		}

		rc = gpio_direction_output(GPIO_PIN
			(marimba_svlte_config_clock->gpio_cfg), 0);
		if (rc < 0) {
			printk(KERN_ERR
				"%s: gpio_direction_output failed (%d)\n",
					__func__, rc);
			return rc;
		}
	}

out:
	return rc;
};

static void msm_marimba_shutdown_power(void)
{
	int rc;

	rc = vreg_disable(vreg_marimba_1);
	if (rc) {
		printk(KERN_ERR "%s: return val: %d\n",
					__func__, rc);
	}
	rc = vreg_disable(vreg_marimba_2);
	if (rc) {
		printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
	}
};
/* < DTS2011030705328 sihongfang 20110311 begin */
/*bcm4329 do not use this function*/
#ifdef CONFIG_HUAWEI_KERNEL
/* DTS2011030705328 sihongfang 20110311 end > */
static int bahama_present(void)
{
	int id;
	switch (id = adie_get_detected_connectivity_type()) {
	case BAHAMA_ID:
		return 1;

	case MARIMBA_ID:
		return 0;

	case TIMPANI_ID:
	default:
	printk(KERN_ERR "%s: unexpected adie connectivity type: %d\n",
			__func__, id);
	return -ENODEV;
	}
}


struct vreg *fm_regulator;
static int fm_radio_setup(struct marimba_fm_platform_data *pdata)
{
	int rc;
	uint32_t irqcfg;
	const char *id = "FMPW";

	int bahama_not_marimba = bahama_present();

	if (bahama_not_marimba == -1) {
		printk(KERN_WARNING "%s: bahama_present: %d\n",
				__func__, bahama_not_marimba);
		return -ENODEV;
	}
	if (bahama_not_marimba)
		fm_regulator = vreg_get(NULL, "s3");
	else
		fm_regulator = vreg_get(NULL, "s2");

	if (IS_ERR(fm_regulator)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(fm_regulator));
		return -1;
	}
	if (!bahama_not_marimba) {

		rc = pmapp_vreg_level_vote(id, PMAPP_VREG_S2, 1300);

		if (rc < 0) {
			printk(KERN_ERR "%s: voltage level vote failed (%d)\n",
				__func__, rc);
			return rc;
		}
	}
	rc = vreg_enable(fm_regulator);
	if (rc) {
		printk(KERN_ERR "%s: vreg_enable() = %d\n",
					__func__, rc);
		return rc;
	}

	rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_DO,
					  PMAPP_CLOCK_VOTE_ON);
	if (rc < 0) {
		printk(KERN_ERR "%s: clock vote failed (%d)\n",
			__func__, rc);
		goto fm_clock_vote_fail;
	}
	/*Request the Clock Using GPIO34/AP2MDM_MRMBCK_EN in case
	of svlte*/
	if (machine_is_msm8x55_svlte_surf() ||
			machine_is_msm8x55_svlte_ffa())	{
		rc = marimba_gpio_config(1);
		if (rc < 0)
			printk(KERN_ERR "%s: clock enable for svlte : %d\n",
						__func__, rc);
	}
	irqcfg = GPIO_CFG(147, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
					GPIO_CFG_2MA);
	rc = gpio_tlmm_config(irqcfg, GPIO_CFG_ENABLE);
	if (rc) {
		printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, irqcfg, rc);
		rc = -EIO;
		goto fm_gpio_config_fail;

	}
	return 0;
fm_gpio_config_fail:
	pmapp_clock_vote(id, PMAPP_CLOCK_ID_DO,
				  PMAPP_CLOCK_VOTE_OFF);
fm_clock_vote_fail:
	vreg_disable(fm_regulator);
	return rc;

};

static void fm_radio_shutdown(struct marimba_fm_platform_data *pdata)
{
	int rc;
	const char *id = "FMPW";
	uint32_t irqcfg = GPIO_CFG(147, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP,
					GPIO_CFG_2MA);

	int bahama_not_marimba = bahama_present();
	if (bahama_not_marimba == -1) {
		printk(KERN_WARNING "%s: bahama_present: %d\n",
			__func__, bahama_not_marimba);
		return;
	}

	rc = gpio_tlmm_config(irqcfg, GPIO_CFG_ENABLE);
	if (rc) {
		printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, irqcfg, rc);
	}
	if (fm_regulator != NULL) {
		rc = vreg_disable(fm_regulator);

		if (rc) {
			printk(KERN_ERR "%s: return val: %d\n",
					__func__, rc);
		}
		fm_regulator = NULL;
	}
	rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_DO,
					  PMAPP_CLOCK_VOTE_OFF);
	if (rc < 0)
		printk(KERN_ERR "%s: clock_vote return val: %d\n",
						__func__, rc);

	/*Disable the Clock Using GPIO34/AP2MDM_MRMBCK_EN in case
	of svlte*/
	if (machine_is_msm8x55_svlte_surf() ||
			machine_is_msm8x55_svlte_ffa())	{
		rc = marimba_gpio_config(0);
		if (rc < 0)
			printk(KERN_ERR "%s: clock disable for svlte : %d\n",
						__func__, rc);
	}


	if (!bahama_not_marimba)	{
		rc = pmapp_vreg_level_vote(id, PMAPP_VREG_S2, 0);

		if (rc < 0)
			printk(KERN_ERR "%s: vreg level vote return val: %d\n",
						__func__, rc);
	}
}

static struct marimba_fm_platform_data marimba_fm_pdata = {
	.fm_setup =  fm_radio_setup,
	.fm_shutdown = fm_radio_shutdown,
	.irq = MSM_GPIO_TO_INT(147),
	.vreg_s2 = NULL,
	.vreg_xo_out = NULL,
};
/* < DTS2011030705328 sihongfang 20110311 begin */
#endif
/* DTS2011030705328 sihongfang 20110311 end > */

/* Slave id address for FM/CDC/QMEMBIST
 * Values can be programmed using Marimba slave id 0
 * should there be a conflict with other I2C devices
 * */
#define MARIMBA_SLAVE_ID_FM_ADDR	0x2A
#define MARIMBA_SLAVE_ID_CDC_ADDR	0x77
#define MARIMBA_SLAVE_ID_QMEMBIST_ADDR	0X66

#define BAHAMA_SLAVE_ID_FM_ADDR         0x2A
#define BAHAMA_SLAVE_ID_QMEMBIST_ADDR   0x7B

static const char *tsadc_id = "MADC";
static const char *vregs_tsadc_name[] = {
	"gp12",
	"s2",
};
static struct vreg *vregs_tsadc[ARRAY_SIZE(vregs_tsadc_name)];

static const char *vregs_timpani_tsadc_name[] = {
	"s3",
	"gp12",
	"gp16"
};
static struct vreg *vregs_timpani_tsadc[ARRAY_SIZE(vregs_timpani_tsadc_name)];

static int marimba_tsadc_power(int vreg_on)
{
	int i, rc = 0;
	int tsadc_adie_type = adie_get_detected_codec_type();

	if (tsadc_adie_type == TIMPANI_ID) {
		for (i = 0; i < ARRAY_SIZE(vregs_timpani_tsadc_name); i++) {
			if (!vregs_timpani_tsadc[i]) {
				pr_err("%s: vreg_get %s failed(%d)\n",
				__func__, vregs_timpani_tsadc_name[i], rc);
				goto vreg_fail;
			}

			rc = vreg_on ? vreg_enable(vregs_timpani_tsadc[i]) :
				  vreg_disable(vregs_timpani_tsadc[i]);
			if (rc < 0) {
				pr_err("%s: vreg %s %s failed(%d)\n",
					__func__, vregs_timpani_tsadc_name[i],
				       vreg_on ? "enable" : "disable", rc);
				goto vreg_fail;
			}
		}
		/* Vote for D0 and D1 buffer */
		rc = pmapp_clock_vote(tsadc_id, PMAPP_CLOCK_ID_D1,
			vreg_on ? PMAPP_CLOCK_VOTE_ON : PMAPP_CLOCK_VOTE_OFF);
		if (rc)	{
			pr_err("%s: unable to %svote for d1 clk\n",
				__func__, vreg_on ? "" : "de-");
			goto do_vote_fail;
		}
		rc = pmapp_clock_vote(tsadc_id, PMAPP_CLOCK_ID_DO,
			vreg_on ? PMAPP_CLOCK_VOTE_ON : PMAPP_CLOCK_VOTE_OFF);
		if (rc)	{
			pr_err("%s: unable to %svote for d1 clk\n",
				__func__, vreg_on ? "" : "de-");
			goto do_vote_fail;
		}
	} else if (tsadc_adie_type == MARIMBA_ID) {
		for (i = 0; i < ARRAY_SIZE(vregs_tsadc_name); i++) {
			if (!vregs_tsadc[i]) {
				pr_err("%s: vreg_get %s failed (%d)\n",
					__func__, vregs_tsadc_name[i], rc);
				goto vreg_fail;
			}

			rc = vreg_on ? vreg_enable(vregs_tsadc[i]) :
				  vreg_disable(vregs_tsadc[i]);
			if (rc < 0) {
				pr_err("%s: vreg %s %s failed (%d)\n",
					__func__, vregs_tsadc_name[i],
				       vreg_on ? "enable" : "disable", rc);
				goto vreg_fail;
			}
		}
		/* If marimba vote for DO buffer */
		rc = pmapp_clock_vote(tsadc_id, PMAPP_CLOCK_ID_DO,
			vreg_on ? PMAPP_CLOCK_VOTE_ON : PMAPP_CLOCK_VOTE_OFF);
		if (rc)	{
			pr_err("%s: unable to %svote for d0 clk\n",
				__func__, vreg_on ? "" : "de-");
			goto do_vote_fail;
		}
	} else {
		pr_err("%s:Adie %d not supported\n",
				__func__, tsadc_adie_type);
		return -ENODEV;
	}

	msleep(5); /* ensure power is stable */

	return 0;

do_vote_fail:
vreg_fail:
	while (i) {
		if (vreg_on) {
			if (tsadc_adie_type == TIMPANI_ID)
				vreg_disable(vregs_timpani_tsadc[--i]);
			else if (tsadc_adie_type == MARIMBA_ID)
				vreg_disable(vregs_tsadc[--i]);
		} else {
			if (tsadc_adie_type == TIMPANI_ID)
				vreg_enable(vregs_timpani_tsadc[--i]);
			else if (tsadc_adie_type == MARIMBA_ID)
				vreg_enable(vregs_tsadc[--i]);
		}
	}

	return rc;
}

static int marimba_tsadc_vote(int vote_on)
{
	int rc = 0;

	if (adie_get_detected_codec_type() == MARIMBA_ID) {
		int level = vote_on ? 1300 : 0;
		rc = pmapp_vreg_level_vote(tsadc_id, PMAPP_VREG_S2, level);
		if (rc < 0)
			pr_err("%s: vreg level %s failed (%d)\n",
			__func__, vote_on ? "on" : "off", rc);
	}

	return rc;
}

static int marimba_tsadc_init(void)
{
	int i, rc = 0;
	int tsadc_adie_type = adie_get_detected_codec_type();

	if (tsadc_adie_type == TIMPANI_ID) {
		for (i = 0; i < ARRAY_SIZE(vregs_timpani_tsadc_name); i++) {
			vregs_timpani_tsadc[i] = vreg_get(NULL,
						vregs_timpani_tsadc_name[i]);
			if (IS_ERR(vregs_timpani_tsadc[i])) {
				pr_err("%s: vreg get %s failed (%ld)\n",
				       __func__, vregs_timpani_tsadc_name[i],
				       PTR_ERR(vregs_timpani_tsadc[i]));
				rc = PTR_ERR(vregs_timpani_tsadc[i]);
				goto vreg_get_fail;
			}
		}
	} else if (tsadc_adie_type == MARIMBA_ID) {
		for (i = 0; i < ARRAY_SIZE(vregs_tsadc_name); i++) {
			vregs_tsadc[i] = vreg_get(NULL, vregs_tsadc_name[i]);
			if (IS_ERR(vregs_tsadc[i])) {
				pr_err("%s: vreg get %s failed (%ld)\n",
				       __func__, vregs_tsadc_name[i],
				       PTR_ERR(vregs_tsadc[i]));
				rc = PTR_ERR(vregs_tsadc[i]);
				goto vreg_get_fail;
			}
		}
	} else {
		pr_err("%s:Adie %d not supported\n",
				__func__, tsadc_adie_type);
		return -ENODEV;
	}

	return 0;

vreg_get_fail:
	while (i) {
		if (tsadc_adie_type == TIMPANI_ID)
			vreg_put(vregs_timpani_tsadc[--i]);
		else if (tsadc_adie_type == MARIMBA_ID)
			vreg_put(vregs_tsadc[--i]);
	}
	return rc;
}

static int marimba_tsadc_exit(void)
{
	int i, rc = 0;
	int tsadc_adie_type = adie_get_detected_codec_type();

	if (tsadc_adie_type == TIMPANI_ID) {
		for (i = 0; i < ARRAY_SIZE(vregs_timpani_tsadc_name); i++) {
			if (vregs_tsadc[i])
				vreg_put(vregs_timpani_tsadc[i]);
		}
	} else if (tsadc_adie_type == MARIMBA_ID) {
		for (i = 0; i < ARRAY_SIZE(vregs_tsadc_name); i++) {
			if (vregs_tsadc[i])
				vreg_put(vregs_tsadc[i]);
		}
		rc = pmapp_vreg_level_vote(tsadc_id, PMAPP_VREG_S2, 0);
		if (rc < 0)
			pr_err("%s: vreg level off failed (%d)\n",
						__func__, rc);
	} else {
		pr_err("%s:Adie %d not supported\n",
				__func__, tsadc_adie_type);
		rc = -ENODEV;
	}

	return rc;
}


static struct msm_ts_platform_data msm_ts_data = {
	.min_x          = 0,
	.max_x          = 4096,
	.min_y          = 0,
	.max_y          = 4096,
	.min_press      = 0,
	.max_press      = 255,
	.inv_x          = 4096,
	.inv_y          = 4096,
	.can_wakeup	= false,
};

static struct marimba_tsadc_platform_data marimba_tsadc_pdata = {
	.marimba_tsadc_power =  marimba_tsadc_power,
	.init		     =  marimba_tsadc_init,
	.exit		     =  marimba_tsadc_exit,
	.level_vote	     =  marimba_tsadc_vote,
	.tsadc_prechg_en = true,
	.can_wakeup	= false,
	.setup = {
		.pen_irq_en	=	true,
		.tsadc_en	=	true,
	},
	.params2 = {
		.input_clk_khz		=	2400,
		.sample_prd		=	TSADC_CLK_3,
	},
	.params3 = {
		.prechg_time_nsecs	=	6400,
		.stable_time_nsecs	=	6400,
		.tsadc_test_mode	=	0,
	},
	.tssc_data = &msm_ts_data,
};

static struct vreg *vreg_codec_s4;
static int msm_marimba_codec_power(int vreg_on)
{
	int rc = 0;

	if (!vreg_codec_s4) {

		vreg_codec_s4 = vreg_get(NULL, "s4");

		if (IS_ERR(vreg_codec_s4)) {
			printk(KERN_ERR "%s: vreg_get() failed (%ld)\n",
				__func__, PTR_ERR(vreg_codec_s4));
			rc = PTR_ERR(vreg_codec_s4);
			goto  vreg_codec_s4_fail;
		}
	}

	if (vreg_on) {
		rc = vreg_enable(vreg_codec_s4);
		if (rc)
			printk(KERN_ERR "%s: vreg_enable() = %d \n",
					__func__, rc);
		goto vreg_codec_s4_fail;
	} else {
		rc = vreg_disable(vreg_codec_s4);
		if (rc)
			printk(KERN_ERR "%s: vreg_disable() = %d \n",
					__func__, rc);
		goto vreg_codec_s4_fail;
	}

vreg_codec_s4_fail:
	return rc;
}

static struct marimba_codec_platform_data mariba_codec_pdata = {
	.marimba_codec_power =  msm_marimba_codec_power,
#ifdef CONFIG_MARIMBA_CODEC
	.snddev_profile_init = msm_snddev_init,
#endif
};

static struct marimba_platform_data marimba_pdata = {
	.slave_id[MARIMBA_SLAVE_ID_FM]       = MARIMBA_SLAVE_ID_FM_ADDR,
	.slave_id[MARIMBA_SLAVE_ID_CDC]	     = MARIMBA_SLAVE_ID_CDC_ADDR,
	.slave_id[MARIMBA_SLAVE_ID_QMEMBIST] = MARIMBA_SLAVE_ID_QMEMBIST_ADDR,
	.slave_id[SLAVE_ID_BAHAMA_FM]        = BAHAMA_SLAVE_ID_FM_ADDR,
	.slave_id[SLAVE_ID_BAHAMA_QMEMBIST]  = BAHAMA_SLAVE_ID_QMEMBIST_ADDR,
	.marimba_setup = msm_marimba_setup_power,
	.marimba_shutdown = msm_marimba_shutdown_power,
	.bahama_setup = msm_bahama_setup_power,
	.bahama_shutdown = msm_bahama_shutdown_power,
	.marimba_gpio_config = msm_marimba_gpio_config_svlte,
	.bahama_core_config = msm_bahama_core_config,
/* < DTS2011030705328 sihongfang 20110311 begin */
/*bcm4329 do not use this function*/
#ifdef CONFIG_HUAWEI_KERNEL
	.fm = &marimba_fm_pdata,
#endif
/* DTS2011030705328 sihongfang 20110311 end > */
	.codec = &mariba_codec_pdata,
};

static void __init msm7x30_init_marimba(void)
{
	int rc;

	vreg_marimba_1 = vreg_get(NULL, "s3");
	if (IS_ERR(vreg_marimba_1)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(vreg_marimba_1));
		return;
	}
	rc = vreg_set_level(vreg_marimba_1, 1800);

	vreg_marimba_2 = vreg_get(NULL, "gp16");
	if (IS_ERR(vreg_marimba_1)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(vreg_marimba_1));
		return;
	}
	rc = vreg_set_level(vreg_marimba_2, 1200);

	vreg_marimba_3 = vreg_get(NULL, "usb2");
	if (IS_ERR(vreg_marimba_3)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(vreg_marimba_3));
		return;
	}
	rc = vreg_set_level(vreg_marimba_3, 1800);
}

static struct marimba_codec_platform_data timpani_codec_pdata = {
	.marimba_codec_power =  msm_marimba_codec_power,
#ifdef CONFIG_TIMPANI_CODEC
	.snddev_profile_init = msm_snddev_init_timpani,
#endif
};

static struct marimba_platform_data timpani_pdata = {
	.slave_id[MARIMBA_SLAVE_ID_CDC]	= MARIMBA_SLAVE_ID_CDC_ADDR,
	.slave_id[MARIMBA_SLAVE_ID_QMEMBIST] = MARIMBA_SLAVE_ID_QMEMBIST_ADDR,
	.marimba_setup = msm_timpani_setup_power,
	.marimba_shutdown = msm_timpani_shutdown_power,
	.codec = &timpani_codec_pdata,
	.tsadc = &marimba_tsadc_pdata,
};

#define TIMPANI_I2C_SLAVE_ADDR	0xD

static struct i2c_board_info msm_i2c_gsbi7_timpani_info[] = {
	{
		I2C_BOARD_INFO("timpani", TIMPANI_I2C_SLAVE_ADDR),
		.platform_data = &timpani_pdata,
	},
};

#ifdef CONFIG_MSM7KV2_AUDIO
static struct resource msm_aictl_resources[] = {
	{
		.name = "aictl",
		.start = 0xa5000100,
		.end = 0xa5000100,
		.flags = IORESOURCE_MEM,
	}
};

static struct resource msm_mi2s_resources[] = {
	{
		.name = "hdmi",
		.start = 0xac900000,
		.end = 0xac900038,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "codec_rx",
		.start = 0xac940040,
		.end = 0xac940078,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "codec_tx",
		.start = 0xac980080,
		.end = 0xac9800B8,
		.flags = IORESOURCE_MEM,
	}

};

static struct msm_lpa_platform_data lpa_pdata = {
	.obuf_hlb_size = 0x2BFF8,
	.dsp_proc_id = 0,
	.app_proc_id = 2,
	.nosb_config = {
		.llb_min_addr = 0,
		.llb_max_addr = 0x3ff8,
		.sb_min_addr = 0,
		.sb_max_addr = 0,
	},
	.sb_config = {
		.llb_min_addr = 0,
		.llb_max_addr = 0x37f8,
		.sb_min_addr = 0x3800,
		.sb_max_addr = 0x3ff8,
	}
};

static struct resource msm_lpa_resources[] = {
	{
		.name = "lpa",
		.start = 0xa5000000,
		.end = 0xa50000a0,
		.flags = IORESOURCE_MEM,
	}
};

static struct resource msm_aux_pcm_resources[] = {

	{
		.name = "aux_codec_reg_addr",
		.start = 0xac9c00c0,
		.end = 0xac9c00c8,
		.flags = IORESOURCE_MEM,
	},
	{
		.name   = "aux_pcm_dout",
		.start  = 138,
		.end    = 138,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_din",
		.start  = 139,
		.end    = 139,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_syncout",
		.start  = 140,
		.end    = 140,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_clkin_a",
		.start  = 141,
		.end    = 141,
		.flags  = IORESOURCE_IO,
	},
};

static struct platform_device msm_aux_pcm_device = {
	.name   = "msm_aux_pcm",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_aux_pcm_resources),
	.resource       = msm_aux_pcm_resources,
};

struct platform_device msm_aictl_device = {
	.name = "audio_interct",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_aictl_resources),
	.resource = msm_aictl_resources,
};

struct platform_device msm_mi2s_device = {
	.name = "mi2s",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_mi2s_resources),
	.resource = msm_mi2s_resources,
};

struct platform_device msm_lpa_device = {
	.name = "lpa",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_lpa_resources),
	.resource = msm_lpa_resources,
	.dev		= {
		.platform_data = &lpa_pdata,
	},
};
#endif /* CONFIG_MSM7KV2_AUDIO */

#define DEC0_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC1_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
 #define DEC2_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
 #define DEC3_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC4_FORMAT (1<<MSM_ADSP_CODEC_MIDI)

static unsigned int dec_concurrency_table[] = {
	/* Audio LP */
	0,
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_MODE_LP)|
	(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 1 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),

	 /* Concurrency 2 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 3 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 4 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 5 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 6 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
};

#define DEC_INFO(name, queueid, decid, nr_codec) { .module_name = name, \
	.module_queueid = queueid, .module_decid = decid, \
	.nr_codec_support = nr_codec}

#define DEC_INSTANCE(max_instance_same, max_instance_diff) { \
	.max_instances_same_dec = max_instance_same, \
	.max_instances_diff_dec = max_instance_diff}

static struct msm_adspdec_info dec_info_list[] = {
	DEC_INFO("AUDPLAY4TASK", 17, 4, 1),  /* AudPlay4BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY3TASK", 16, 3, 11),  /* AudPlay3BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY2TASK", 15, 2, 11),  /* AudPlay2BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY1TASK", 14, 1, 11),  /* AudPlay1BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY0TASK", 13, 0, 11), /* AudPlay0BitStreamCtrlQueue */
};

static struct dec_instance_table dec_instance_list[][MSM_MAX_DEC_CNT] = {
	/* Non Turbo Mode */
	{
		DEC_INSTANCE(4, 3), /* WAV */
		DEC_INSTANCE(4, 3), /* ADPCM */
		DEC_INSTANCE(4, 2), /* MP3 */
		DEC_INSTANCE(0, 0), /* Real Audio */
		DEC_INSTANCE(4, 2), /* WMA */
		DEC_INSTANCE(3, 2), /* AAC */
		DEC_INSTANCE(0, 0), /* Reserved */
		DEC_INSTANCE(0, 0), /* MIDI */
		DEC_INSTANCE(4, 3), /* YADPCM */
		DEC_INSTANCE(4, 3), /* QCELP */
		DEC_INSTANCE(4, 3), /* AMRNB */
		DEC_INSTANCE(1, 1), /* AMRWB/WB+ */
		DEC_INSTANCE(4, 3), /* EVRC */
		DEC_INSTANCE(1, 1), /* WMAPRO */
	},
	/* Turbo Mode */
	{
		DEC_INSTANCE(4, 3), /* WAV */
		DEC_INSTANCE(4, 3), /* ADPCM */
		DEC_INSTANCE(4, 3), /* MP3 */
		DEC_INSTANCE(0, 0), /* Real Audio */
		DEC_INSTANCE(4, 3), /* WMA */
		DEC_INSTANCE(4, 3), /* AAC */
		DEC_INSTANCE(0, 0), /* Reserved */
		DEC_INSTANCE(0, 0), /* MIDI */
		DEC_INSTANCE(4, 3), /* YADPCM */
		DEC_INSTANCE(4, 3), /* QCELP */
		DEC_INSTANCE(4, 3), /* AMRNB */
		DEC_INSTANCE(2, 3), /* AMRWB/WB+ */
		DEC_INSTANCE(4, 3), /* EVRC */
		DEC_INSTANCE(1, 2), /* WMAPRO */
	},
};

static struct msm_adspdec_database msm_device_adspdec_database = {
	.num_dec = ARRAY_SIZE(dec_info_list),
	.num_concurrency_support = (ARRAY_SIZE(dec_concurrency_table) / \
					ARRAY_SIZE(dec_info_list)),
	.dec_concurrency_table = dec_concurrency_table,
	.dec_info_list = dec_info_list,
	.dec_instance_list = &dec_instance_list[0][0],
};

static struct platform_device msm_device_adspdec = {
	.name = "msm_adspdec",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_adspdec_database
	},
};

	/* <DTS2010072202961 hufeng 20100722 begin */
	/* removed several lines */
	/* DTS2010072202961 hufeng 20100722 end> */

#ifdef CONFIG_USB_FUNCTION
static struct usb_mass_storage_platform_data usb_mass_storage_pdata = {
	.nluns          = 0x02,
	.buf_size       = 16384,
	.vendor         = "GOOGLE",
	.product        = "Mass storage",
	.release        = 0xffff,
};

static struct platform_device mass_storage_device = {
	.name           = "usb_mass_storage",
	.id             = -1,
	.dev            = {
		.platform_data          = &usb_mass_storage_pdata,
	},
};
#endif
#ifdef CONFIG_USB_ANDROID
/*< updata QC2030 USB yanzhijun 20101105 begin */
#ifdef CONFIG_USB_AUTO_INSTALL
static char *usb_functions_hw_normal_adb[] = {
  "modem",
  "nmea",
  "usb_mass_storage",
  "adb",
  "diag",
};

static char *usb_functions_hw_normal[] = {
  "modem",
  "nmea",
  "usb_mass_storage",
};

static char *usb_functions_hw_ms[] = {
  "usb_mass_storage",
};

static char *usb_functions_google_ms[] = {
  "usb_mass_storage",
};

static char *usb_functions_google_ms_adb[] = {
  "usb_mass_storage",
  "adb",  
};

/*< DTS2011082402558 zhangyancun 20110824 begin */
/* add adb port for USB tether */
static char *usb_functions_rndis_adb[] = {
	"rndis", 
	"adb",	
};
/* DTS2011082402558 zhangyancun 20110824 end >*/
static char *usb_functions_rndis[] = {
  "rndis", 
};


static char *usb_functions_all[] = {  
#ifdef CONFIG_USB_ANDROID_RNDIS
    "rndis",
#endif
#ifdef CONFIG_USB_F_SERIAL
  "modem",
  "nmea",
#endif
  "usb_mass_storage",
  "adb",
#ifdef CONFIG_USB_ANDROID_DIAG
  "diag",
#endif
#ifdef CONFIG_USB_ANDROID_RMNET
  "rmnet",
#endif
#ifdef CONFIG_USB_ANDROID_ACM
  "acm",
#endif
};

static struct android_usb_product_hw usb_products[] = {
  {
    .adb_product_id = PID_UDISK,
    .adb_num_functions  = ARRAY_SIZE(usb_functions_hw_ms),
    .adb_functions  = usb_functions_hw_ms,
    .product_id = PID_UDISK,
    .num_functions  = ARRAY_SIZE(usb_functions_hw_ms),
    .functions  = usb_functions_hw_ms,
    /*< DTS2011062501797 zhangyancun 20110728 begin */
    /* two udisks to support internal sdcard (merge DTS2011031601341) */
    .nluns = 2,
    /* DTS2011062501797 zhangyancun 20110728 end >*/ 
		/* no cdrom */
        .cdrom_index=-1,
  },
  {
    .adb_product_id = PID_ONLY_CDROM,
    .adb_num_functions  = ARRAY_SIZE(usb_functions_hw_ms),
    .adb_functions  = usb_functions_hw_ms,
    .product_id = PID_ONLY_CDROM,
    .num_functions  = ARRAY_SIZE(usb_functions_hw_ms),
    .functions  = usb_functions_hw_ms,
    .nluns = 1,
        .cdrom_index=0,
  },
  {
    .adb_product_id = PID_AUTH,
    .adb_num_functions  = ARRAY_SIZE(usb_functions_hw_normal_adb),
    .adb_functions  = usb_functions_hw_normal_adb,
    .product_id = PID_AUTH,
    .num_functions  = ARRAY_SIZE(usb_functions_hw_normal_adb),
    .functions  = usb_functions_hw_normal_adb,
    .nluns = 1,
        .cdrom_index=0,
  },
  {
    .adb_product_id = PID_NORMAL,
    .adb_num_functions  = ARRAY_SIZE(usb_functions_hw_normal_adb),
    .adb_functions  = usb_functions_hw_normal_adb,
    .product_id = PID_NORMAL,
    .num_functions = ARRAY_SIZE(usb_functions_hw_normal),
    .functions  = usb_functions_hw_normal,
    .nluns = 1,
        .cdrom_index=0,
  },
  {
    .adb_product_id = PID_ONLY_CDROM_TMO,
    .adb_num_functions  = ARRAY_SIZE(usb_functions_hw_ms),
    .adb_functions  = usb_functions_hw_ms,
    .product_id = PID_ONLY_CDROM_TMO,
    .num_functions  = ARRAY_SIZE(usb_functions_hw_ms),
    .functions  = usb_functions_hw_ms,
    .nluns = 1,
        .cdrom_index=0,
  },
  {
    .adb_product_id = PID_AUTH_TMO,
    .adb_num_functions  = ARRAY_SIZE(usb_functions_hw_normal_adb),
    .adb_functions  = usb_functions_hw_normal_adb,
    .product_id = PID_AUTH_TMO,
    .num_functions  = ARRAY_SIZE(usb_functions_hw_normal_adb),
    .functions  = usb_functions_hw_normal_adb,
    .nluns = 1,
        .cdrom_index=0,
  },
  {
    .adb_product_id = PID_NORMAL_TMO,
    .adb_num_functions  = ARRAY_SIZE(usb_functions_hw_normal_adb),
    .adb_functions  = usb_functions_hw_normal_adb,
    .product_id = PID_NORMAL_TMO,
    .num_functions = ARRAY_SIZE(usb_functions_hw_normal),
    .functions  = usb_functions_hw_normal,
    .nluns = 1,
        .cdrom_index=0,
  },
  {
    .adb_product_id = PID_GOOGLE,
    .adb_num_functions  = ARRAY_SIZE(usb_functions_google_ms_adb),
    .adb_functions  = usb_functions_google_ms_adb,
    .product_id = PID_GOOGLE_MS,
    .num_functions = ARRAY_SIZE(usb_functions_google_ms),
    .functions  = usb_functions_google_ms,
    .nluns = 3,
        .cdrom_index=1,
  },
  {
    /*< DTS2011082402558 zhangyancun 20110824 begin */
    /* add adb port for USB tether */
    .adb_product_id = PID_WLAN_ADB,
    .adb_num_functions  = ARRAY_SIZE(usb_functions_rndis_adb),
    .adb_functions  = usb_functions_rndis_adb,
    /* DTS2011082402558 zhangyancun 20110824 end >*/
    .product_id = PID_WLAN,
    .num_functions  = ARRAY_SIZE(usb_functions_rndis),
    .functions  = usb_functions_rndis,
    .nluns = 1,
        .cdrom_index=-1,
  },
};


static char product_name[MAX_NAME_LEN];
static char vendor_name[MAX_NAME_LEN];
static char manufacturer_name[MAX_NAME_LEN];
#define MAX_LENS 3
static struct usb_mass_storage_platform_data mass_storage_pdata = {
  .nluns    = MAX_LENS,
  .vendor   = vendor_name,//"Qualcomm Incorporated",
  .product        = product_name,//"Mass storage",
  .release  = 0x0100,
};

static struct platform_device usb_mass_storage_device = {
  .name = "usb_mass_storage",
  .id = -1,
  .dev  = {
    .platform_data = &mass_storage_pdata,
  },
};

static struct usb_ether_platform_data rndis_pdata = {
  /* ethaddr is filled by board_serialno_setup */
  .vendorID = HUAWEI_VID,
  .vendorDescr  = manufacturer_name,//"Huawei Incorporated",
};

static struct platform_device rndis_device = {
  .name = "rndis",
  .id = -1,
  .dev  = {
    .platform_data = &rndis_pdata,
  },
};

static struct android_usb_platform_data android_usb_pdata = {
  .vendor_id  = HUAWEI_VID,
  .product_id = 0x9026,
  .version  = 0x0100,
  .product_name   = product_name,
  .manufacturer_name  = manufacturer_name,//"Huawei Incorporated",
  .num_products = ARRAY_SIZE(usb_products),
  .products = usb_products,
  .num_functions = ARRAY_SIZE(usb_functions_all),
  .functions = usb_functions_all,
  .serial_number = "1234567890ABCDEF",
};
#endif
/* updata QC2030 USB yanzhijun 20101105 end >*/


static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};

static int __init board_serialno_setup(char *serialno)
{
	int i;
	char *src = serialno;

	/* create a fake MAC address from our serial number.
	 * first byte is 0x02 to signify locally administered.
	 */
	rndis_pdata.ethaddr[0] = 0x02;
	for (i = 0; *src; i++) {
		/* XOR the USB serial across the remaining bytes */
		rndis_pdata.ethaddr[i % (ETH_ALEN - 1) + 1] ^= *src++;
	}

	android_usb_pdata.serial_number = serialno;
	return 1;
}
__setup("androidboot.serialno=", board_serialno_setup);

#endif

/* < DTS2010061100003 luojianhong 201000612 begin*/
/*remove the statements about m33c01*/

/*< DTS2010071700383 haoqingtao 20100716 begin*/
/* kernel29 -> kernel32 driver modify*/
/*  DTS2010061100003 luojianhong 201000612 end > */
/*<BU5D09205 zhangtao 20100503 begin*/
/* < DTS2010070200975 zhangtao 20100702 begin */
/* delete some lines*/
/* DTS2010070200975 zhangtao 20100702 end > */
/*<BU5D09397 lijuan 00152865, 20100506 begin*/
lcd_panel_type lcd_panel_probe(void)
{
    lcd_panel_type hw_lcd_panel = LCD_NONE;
/*< DTS2010092400487  lijianzhao 20100924 begin */
	/*< DTS2010112702297 wangquanli 201001125 begin */
	/*< DTS2011030202729  liliang 20110302  begin */
	if ((machine_is_msm7x30_u8800()) || (machine_is_msm7x30_u8820()) || (machine_is_msm7x30_u8800_51()) || (machine_is_msm8255_u8800_pro()))
	/* DTS2011030202729  liliang 20110302 end >*/
	/* DTS2010112702297 wangquanli 201001125 end >*/
	{
		switch (lcd_id)
		{
			case 0:  
				hw_lcd_panel = LCD_NT35582_BYD_WVGA;
			break;
		 	case 1:  
				hw_lcd_panel = LCD_NT35582_TRULY_WVGA;
			break;
			/*< DTS2010120703279 lijianzhao 20101207 begin */
			/* NT35510 alpha_si is MDDI type2 LCD */
			case 2:  
				hw_lcd_panel = LCD_NT35510_ALPHA_SI_WVGA;
			break;
			/* DTS2010120703279 lijianzhao 20101207 end >*/
/*< DTS2011060802925  jiaoshuangwei 20110608 begin */
			/* NT35510 alpha_si is MDDI type2 LCD */
			case 3:  
				hw_lcd_panel = LCD_NT35510_ALPHA_SI_WVGA_TYPE2;
			break;
/* DTS2011060802925 jiaoshuangwei 20110608 end >*/
			default : 
				hw_lcd_panel = LCD_NONE;
			break;					  
		}
	}
/*<DTS2011042601502  jiaoshuangwei 20110308 begin*/
	/* <DTS2011050700551 zhangbo 20110505 begin */
    /* <DTS2011062600102 sunhonghui 20110626 begin */
    /* <DTS2011071600361 liyuping 20110716 begin */
	/* < DTS2011082302564 liwei 20110823 begin */
/*< DTS2011090102706 jiaoshuangwei 20110901 begin */
/*<DTS2011091502092 liyuping 20110915 begin */
    else if (machine_is_msm8255_u8860() 
			|| machine_is_msm8255_c8860() 
			|| machine_is_msm8255_u8860lp()
			|| machine_is_msm8255_u8860_92()
			|| machine_is_msm8255_u8860_51())
/* DTS2011091502092 liyuping 20110915 end> */ 
/* DTS2011090102706 jiaoshuangwei 20110901 end >*/
	/* DTS2011082302564 liwei 20110823 end > */
	/* DTS2011071600361 liyuping 20110716 end> */
    /* DTS2011062600102 sunhonghui 20110626 end> */
	/* DTS2011050700551 zhangbo 20110505 end> */
	{
		switch (lcd_id)
		{
            case 0:
				hw_lcd_panel = LCD_NT35560_TOSHIBA_FWVGA;
				break;
			default : 
				hw_lcd_panel = LCD_NONE;
				break;					  
		}
	}
/*< DTS2011090102706 jiaoshuangwei 20110901 begin */
/* <DTS2011091200073 zhangbo 20110912 begin */
/* < DTS2011102401822 liwei 20111024 begin */
	else if( machine_is_msm8255_u8680()
        || machine_is_msm8255_u8667()
	    || (machine_is_msm8255_u8730()))
/* DTS2011102401822 liwei 20111024 end > */
/* DTS2011091200073 zhangbo 20110912 end> */
	{
		switch (lcd_id)
		{
            case 0:
				hw_lcd_panel = LCD_RSP61408_CHIMEI_WVGA;
				break;
/*< DTS2011100803621  jiaoshuangwei 20111008 begin */
			case 2:
				hw_lcd_panel = LCD_RSP61408_BYD_WVGA;
				break;
/* DTS2011100803621 jiaoshuangwei 20111008 end >*/
			default : 
				hw_lcd_panel = LCD_RSP61408_CHIMEI_WVGA;
				break;					  
		}
	}
/* DTS2011090102706 jiaoshuangwei 20110901 end >*/
/* DTS2011042601502  jiaoshuangwei 20110308 end>*/
/* DTS2010092400487  lijianzhao 20100924 end >*/
    /*< DTS2011091402488 hujun 20110914 begin */
    /*delet one line to avoid too much logs*/
    /*DTS2011091402488 hujun 20110914 end > */
    return hw_lcd_panel;

}
/*BU5D09397 lijuan 00152865, 20100506 end>*/


/* < BU5D10365 liujinggang 20100521 begin*/
char *get_lcd_panel_name(void)
{
    lcd_panel_type hw_lcd_panel = LCD_NONE;
    char *pname = NULL;
    
    hw_lcd_panel = lcd_panel_probe();
    
    switch (hw_lcd_panel)
    {
     case LCD_S6D74A0_SAMSUNG_HVGA:
            pname = "SAMSUNG S6D74A0";
            break;
            
        case LCD_ILI9325_INNOLUX_QVGA:
            pname = "INNOLUX ILI9325";
            break;

        case LCD_ILI9325_BYD_QVGA:
            pname = "BYD ILI9325";
            break;

        case LCD_ILI9325_WINTEK_QVGA:
            pname = "WINTEK ILI9325";
            break;

        case LCD_SPFD5408B_KGM_QVGA:
            pname = "KGM SPFD5408B";
            break;

        case LCD_HX8357A_BYD_QVGA:
            pname = "BYD HX8357A";
            break;

        case LCD_HX8368A_SEIKO_QVGA:
            pname = "SEIKO HX8368A";
            break;
			
        case LCD_HX8347D_TRULY_QVGA:
            pname = "TRULY HX8347D";
            break;
			
        case LCD_ILI9325C_WINTEK_QVGA:
            pname = "WINTEK ILI9325C";
            break;
						
        case LCD_NT35582_BYD_WVGA:
            pname = "BYD NT35582";
            break;
            
        case LCD_NT35582_TRULY_WVGA:
            pname = "TRULY NT35582";
            break;

/*<DTS2011042601502  jiaoshuangwei 20110308 begin*/
    	case LCD_NT35560_TOSHIBA_FWVGA:
		    pname = "TOSHIBA NT35560";
/* DTS2011042601502  jiaoshuangwei 20110308 end>*/
/*<DTS2011062801414 fengwei 20110628 begin*/
			break;
/*DTS2011062801414 fengwei 20110628 end>*/

		/*< DTS2010120703279 lijianzhao 20101207 begin */
    	case LCD_NT35510_ALPHA_SI_WVGA:
/*< DTS2011022503112 lijianzhao 20110225 begin */
		    pname = "TRULY NT35510";
/* DTS2011022503112 lijianzhao 20110225 end >*/ 
            break;
		/* DTS2010120703279 lijianzhao 20101207 end >*/
            
/*< DTS2011060802925 jiaoshuangwei 20110608 begin */
       /* set the lcd name is SUCCESS NT_35510 */
    	case LCD_NT35510_ALPHA_SI_WVGA_TYPE2:
		    pname = "SUCCESS NT35510";
            break;
/* DTS2011060802925 jiaoshuangwei 20110608 end >*/
/*< DTS2011090102706 jiaoshuangwei 20110901 begin */
		case LCD_RSP61408_CHIMEI_WVGA:
			pname = "CHIMEI RSP61408";
			break;
/* DTS2011090102706 jiaoshuangwei 20110901 end >*/
/*< DTS2011100803621  jiaoshuangwei 20111008 begin */
		case LCD_RSP61408_BYD_WVGA:
			pname = "BYD RSP61408";
			break;
/* DTS2011100803621 jiaoshuangwei 20111008 end >*/
        default:
            pname = "UNKNOWN LCD";
            break;
    }

    return pname;
}
/* < DTS2011042703449  liujinggang 20110427 begin */
static int gsensor_support_dummyaddr(void)
{
    int ret = -1;	/*default value means actual address*/

	if ( (machine_is_msm7x30_u8800()) 
		|| (machine_is_msm7x30_u8820()) 
		|| (machine_is_msm7x30_u8800_51()) 
		|| (machine_is_msm8255_u8800_pro()))
    {
		ret = (int)GS_ST303DLH;
    }
    return ret;
}
static int gsensor_support_dummyaddr_adi346(void)
{
    int ret = -1;	/*default value means actual address*/

    ret = (int)GS_ADI346;

    return ret;
}
static int gs_init_flag = 0;   /*gsensor is not initialized*/

/* < DTS2011042801270 yuezenglong 20110428 begin */
#ifdef CONFIG_HUAWEI_FEATURE_GYROSCOPE_L3G4200DH
static struct gyro_platform_data gy_l3g4200d_platform_data = {
    .gyro_power = power_switch,
    .axis_map_x = 0,     /*x map read data[axis_map_x] from i2c*/
    .axis_map_y = 1,
    .axis_map_z = 2,	
    .negate_x = 0,       /*negative x,y or z*/
    .negate_y =0,
    .negate_z = 0,
    .slave_addr = 0x68,  	/*i2c slave address*/
    .dev_id = 0x0F,             /*WHO AM I*/
};
#endif
/* DTS2011042801270 yuezenglong 20110428 end > */
#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ACCELEROMETER_MMA8452
static struct gs_platform_data gs_mma8452_platform_data = {
    .adapt_fn = NULL,
    .slave_addr = (0x38 >> 1),  /*i2c slave address*/
    .dev_id = 0x2A,    /*WHO AM I*/
    .init_flag = &gs_init_flag,
	.get_compass_gs_position=get_compass_gs_position,
};
#endif

/* < DTS2011043000257  liujinggang 20110503 begin */
/*add compass platform data and the func of power_switch*/
#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ACCELEROMETER_BOSCH_BMA250
static struct gs_platform_data gs_bma250_platform_data = {
    .adapt_fn = NULL,
    .slave_addr = (0x32 >> 1),  /*i2c slave address*/
    .dev_id = 0x03,    /*WHO AM I*/
    .init_flag = &gs_init_flag,
    .get_compass_gs_position=get_compass_gs_position,
    .gs_power = power_switch,
};
#endif

#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ST_LSM303DLH
static struct gs_platform_data st303_gs_platform_data = {
    .adapt_fn = gsensor_support_dummyaddr,
    .slave_addr = (0x32 >> 1),  /*i2c slave address*/
    .dev_id = 0,    /*WHO AM I*/
    .init_flag = &gs_init_flag,
    .get_compass_gs_position=NULL,
    .gs_power = power_switch,
};

static struct compass_platform_data st303_compass_platform_data = {
    .compass_power = power_switch,
};
#endif

#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_AK8975
static struct compass_platform_data akm8975_compass_platform_data = {
    .compass_power = power_switch,
};
#endif 
	
#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ACCELEROMETER_ST_LIS3XH
static struct gs_platform_data gs_st_lis3xh_platform_data = {
    .adapt_fn = NULL,
    .slave_addr = (0x30 >> 1),  /*i2c slave address*/
    .dev_id = 0,    /*WHO AM I*/
    .init_flag = &gs_init_flag,
    .get_compass_gs_position=get_compass_gs_position,
    .gs_power = power_switch,
};
#endif
#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ACCELEROMETER_ADI_ADXL346
static struct gs_platform_data gs_adi346_platform_data = {
    .adapt_fn = gsensor_support_dummyaddr_adi346,
    .slave_addr = (0xA6 >> 1),  /*i2c slave address*/
    .dev_id = 0,    /*WHO AM I*/
    .init_flag = &gs_init_flag,
    .get_compass_gs_position=get_compass_gs_position,
    .gs_power = power_switch,
};
#endif 
/* DTS2011043000257  liujinggang 20110503 end > */
/* DTS2011042703449  liujinggang 20110427 end > */

/* BU5D10365 liujinggang 20100521 end > */
/* < DTS2011042602168 caomingxing 20110426 begin */
#ifdef CONFIG_HUAWEI_KERNEL
/*< DTS2011011904316 genghua 20110121 begin */

#ifdef CONFIG_HUAWEI_NFC_PN544
/* this function is used to reset pn544 by controlling the ven pin */
static int pn544_ven_reset(void)
{
	int ret=0;
	int gpio_config=0;
	/* < DTS2011051001634 caomingxing 20110510 begin */
	ret = gpio_request(GPIO_NFC_VEN, "gpio 130 for NFC pn544");
	/* DTS2011051001634 caomingxing 20110510 end > */
	
	gpio_config = GPIO_CFG(GPIO_NFC_VEN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);
	ret = gpio_tlmm_config(gpio_config, GPIO_CFG_ENABLE);
	
	ret = gpio_direction_output(GPIO_NFC_VEN,0);

	/* < DTS2011051001634 caomingxing 20110510 begin */
	/* pull up first, then pull down for 10 ms, and enable last */
	gpio_set_value(GPIO_NFC_VEN, 1);
	mdelay(5);
	gpio_set_value(GPIO_NFC_VEN, 0);
	/* DTS2011051001634 caomingxing 20110510 end > */

	mdelay(10);
	gpio_set_value(GPIO_NFC_VEN, 1);
	mdelay(5);
	return 0;
}

static int pn544_interrupt_gpio_config(void)
{
	int ret=0;
	int gpio_config=0;
	gpio_config = GPIO_CFG(GPIO_NFC_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA);
	ret = gpio_tlmm_config(gpio_config, GPIO_CFG_ENABLE);
	/* < DTS2011051001634 caomingxing 20110510 begin */
	ret = gpio_request(GPIO_NFC_INT, "gpio 49 for NFC pn544");
	/* DTS2011051001634 caomingxing 20110510 end > */
	ret = gpio_direction_input(GPIO_NFC_INT);
	return 0;
}

/* < DTS2011091905981  sunyue 20110929 begin */
static int pn544_fw_download_pull_down(void)
{
	gpio_set_value(PN544_DOWNLOAD_GPIO, 1);
	mdelay(5);
	gpio_set_value(PN544_DOWNLOAD_GPIO, 0);
	mdelay(5);
	return 0;	
}

static int pn544_fw_download_pull_high(void)
{
	gpio_set_value(PN544_DOWNLOAD_GPIO, 0);
	mdelay(5);
	gpio_set_value(PN544_DOWNLOAD_GPIO, 1);
	mdelay(5);
	return 0;
}

static struct pn544_nfc_platform_data pn544_hw_data = 
{
	.pn544_ven_reset = pn544_ven_reset,
	.pn544_interrupt_gpio_config = pn544_interrupt_gpio_config,
	.pn544_fw_download_pull_down = pn544_fw_download_pull_down,
	.pn544_fw_download_pull_high = pn544_fw_download_pull_high,	
};
/* DTS2011091905981  sunyue 20110929 end > */

#endif
/* DTS2011011904316 genghua 20110121 end >*/
#endif
/* DTS2011042602168 caomingxing 20110426 end > */
static struct i2c_board_info msm_i2c_board_info[] = {
/* < DTS2010061100003 luojianhong 201000612 begin*/
#ifdef CONFIG_HUAWEI_FEATURE_TPA2028D1_AMPLIFIER
    {   
		I2C_BOARD_INFO("tpa2028d1", 0x58),  
        .platform_data = &audio_amplifier_data,
	},
#endif
/*  DTS2010061100003 luojianhong 201000612 end > */
    /* < DTS2011042801270 yuezenglong 20110428 begin */
	#ifdef CONFIG_HUAWEI_FEATURE_GYROSCOPE_L3G4200DH
    {   
		I2C_BOARD_INFO("l3g4200d", 0x68),  
		.platform_data = &gy_l3g4200d_platform_data,
    },
	#endif
	/* DTS2011042801270 yuezenglong 20110428 end > */
	/* < DTS2011042703449  liujinggang 20110427 begin */
	/* <BU5D07680 liujinggang 20100413 begin */
	#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ST_LSM303DLH
	{
		I2C_BOARD_INFO("st303_gs", 0x64 >> 1),         
		.platform_data = &st303_gs_platform_data,
		//.irq = MSM_GPIO_TO_INT() 
	},
	/* < DTS2011043000257  liujinggang 20110503 begin */
	{
		I2C_BOARD_INFO("st303_compass", 0x3c >> 1),/* actual i2c address is 0x3c    */    
		.platform_data = &st303_compass_platform_data,
		//.irq = MSM_GPIO_TO_INT() 
	},
	/* DTS2011043000257  liujinggang 20110503 end > */
	#endif
	/* BU5D07680 liujinggang 20100413 end> */
	#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ACCELEROMETER_MMA8452
    {
        I2C_BOARD_INFO("gs_mma8452", 0x38 >> 1),
        .platform_data = &gs_mma8452_platform_data,
        .irq = MSM_GPIO_TO_INT(19)    //MEMS_INT1
    },
	#endif	

	#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ACCELEROMETER_BOSCH_BMA250
    {
        I2C_BOARD_INFO("gs_bma250", 0x32 >> 1),
        .platform_data = &gs_bma250_platform_data,
        .irq = MSM_GPIO_TO_INT(19)    //MEMS_INT1
    },
	#endif	
	#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_AK8975
	/* < DTS2011043000257  liujinggang 20110503 begin */
    {
        I2C_BOARD_INFO("akm8975", 0x18 >> 1),//7 bit addr, no write bit
        .platform_data = &akm8975_compass_platform_data,
        .irq = MSM_GPIO_TO_INT(132)
    },
	/* DTS2011043000257  liujinggang 20110503 end > */
	#endif 
	#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ACCELEROMETER_ST_LIS3XH
    {
        I2C_BOARD_INFO("gs_st_lis3xh", 0x30 >> 1),
		.platform_data = &gs_st_lis3xh_platform_data,
        .irq = MSM_GPIO_TO_INT(19)    //MEMS_INT1
    },
	#endif
	#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ACCELEROMETER_ADI_ADXL346
    {
        I2C_BOARD_INFO("gs_adi346", 0xA6>>1),  /* actual address 0xA6, fake address 0xA8*/
		.platform_data = &gs_adi346_platform_data,
        .irq = MSM_GPIO_TO_INT(19)    //MEMS_INT1
    },
	#endif 
	/* DTS2011042703449  liujinggang 20110427 end > */
/* < BU5D08118 zhangtao 20100419 begin */	
#ifdef CONFIG_HUAWEI_FEATURE_PROXIMITY_EVERLIGHT_APS_12D
	{   
		I2C_BOARD_INFO("aps-12d", 0x88 >> 1),  
	},
#endif
/* < BU5D08118 zhangtao 20100419 end */
/* < DTS2011042604384  wangjiongfeng 20110427 begin */
#ifdef CONFIG_HUAWEI_FEATURE_PROXIMITY_EVERLIGHT_APS_9900
	{   
		I2C_BOARD_INFO("aps-9900", 0x39),
        /* <DTS2011050303216 wangjiongfeng 20110504 begin */
        .irq = MSM_GPIO_TO_INT(MSM_7X30_APS9900_INT),
        .platform_data = &aps9900_hw_data,
        /* <DTS2011050303216 wangjiongfeng 20110504 end */
	},
#endif
/* < DTS2011042604384  wangjiongfeng 20110427 end */
/* < DTS2011042602168 caomingxing 20110426 begin */
/* < DTS2011011904316 genghua 20110121 begin */
#ifdef CONFIG_HUAWEI_NFC_PN544
	{
		I2C_BOARD_INFO(PN544_DRIVER_NAME, PN544_I2C_ADDR),
		.irq = MSM_GPIO_TO_INT(GPIO_NFC_INT),
		.platform_data = &pn544_hw_data,
	},
#endif
/* DTS2011011904316 genghua 20110121 end >*/
/* DTS2011042602168 caomingxing 20110426 end > */
/* < DTS2010061200552 zhangtao 20100612 begin */	
#ifdef CONFIG_HUAWEI_FEATURE_RMI_TOUCH
	{   
            /*<DTS2011042602009 fengwei 20110426 begin*/
	        I2C_BOARD_INFO("Synaptics_rmi", 0x70),   // actual address 0x24, use fake address 0x70
            .platform_data = &touch_hw_data,
            .irq = MSM_GPIO_TO_INT(MSM_7x30_TOUCH_INT),
            .flags = true, //this flags is the switch of the muti_touch 
	        /*DTS2011042602009 fengwei 20110426 end>*/

				

	},
#endif
/* DTS2010061200552 zhangtao 20100612 end > */	
/*BU5D08118 zhangtao 20100419 end > */
/*<BU5D09205 zhangtao 20100503 begin*/
#ifdef CONFIG_HUAWEI_FEATURE_AT42QT_TS
	{   
		I2C_BOARD_INFO("atmel-rmi-ts", 0x4a),  
/* < DTS2010070200975 zhangtao 20100702 begin */
/* delete some lines*/
/* DTS2010070200975 zhangtao 20100702 end > */
        .irq = MSM_GPIO_TO_INT(ATMEL_RMI_TS_IRQ),
	},
#endif
/*BU5D09205 zhangtao 20100419 end > */
/* < DTS2011102806486  zhangmin 20111029  begin */
#ifdef CONFIG_HUAWEI_FEATURE_PROXIMITY_LITEON_LTR_558
	{   
		I2C_BOARD_INFO("ltr-558", 0x46 >> 1),
        /*they use same int_gpio*/
        .irq = MSM_GPIO_TO_INT(MSM_7X30_LTR558_INT),
        .platform_data = &ltr558_hw_data,
	},
#endif
/* DTS2011102806486  zhangmin 20111029  end > */
/* <DTS2010072202961 hufeng 20100722 begin */
/* removed several lines */
/* DTS2010072202961 hufeng 20100722 end> */
};

static struct i2c_board_info msm_marimba_board_info[] = {
	{
		I2C_BOARD_INFO("marimba", 0xc),
		.platform_data = &marimba_pdata,
	}
};

#ifdef CONFIG_USB_FUNCTION
static struct usb_function_map usb_functions_map[] = {
	{"diag", 0},
	{"adb", 1},
	{"modem", 2},
	{"nmea", 3},
	{"mass_storage", 4},
	{"ethernet", 5},
};

static struct usb_composition usb_func_composition[] = {
	{
		.product_id         = 0x9012,
		.functions	    = 0x5, /* 0101 */
	},

	{
		.product_id         = 0x9013,
		.functions	    = 0x15, /* 10101 */
	},

	{
		.product_id         = 0x9014,
		.functions	    = 0x30, /* 110000 */
	},

	{
		.product_id         = 0x9016,
		.functions	    = 0xD, /* 01101 */
	},

	{
		.product_id         = 0x9017,
		.functions	    = 0x1D, /* 11101 */
	},

	{
		.product_id         = 0xF000,
		.functions	    = 0x10, /* 10000 */
	},

	{
		.product_id         = 0xF009,
		.functions	    = 0x20, /* 100000 */
	},

	{
		.product_id         = 0x9018,
		.functions	    = 0x1F, /* 011111 */
	},

};
static struct msm_hsusb_platform_data msm_hsusb_pdata = {
	.version	= 0x0100,
	.phy_info	= USB_PHY_INTEGRATED | USB_PHY_MODEL_45NM,
	.vendor_id	= 0x12d1,
	.product_name	= "Qualcomm HSUSB Device",
	.serial_number	= "1234567890ABCDEF",
	.manufacturer_name
			= "Qualcomm Incorporated",
	.compositions	= usb_func_composition,
	.num_compositions
			= ARRAY_SIZE(usb_func_composition),
	.function_map	= usb_functions_map,
	.num_functions	= ARRAY_SIZE(usb_functions_map),
	.core_clk	= 1,
};
#endif

static struct msm_handset_platform_data hs_platform_data = {
	.hs_name = "7k_handset",
	.pwr_key_delay_ms = 500, /* 0 will disable end key */
};

static struct platform_device hs_device = {
	.name   = "msm-handset",
	.id     = -1,
	.dev    = {
		.platform_data = &hs_platform_data,
	},
};

static struct msm_pm_platform_data msm_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency = 8594,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].residency = 23740,

	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency = 4594,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].residency = 23740,

	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].supported = 1,
#ifdef CONFIG_MSM_STANDALONE_POWER_COLLAPSE
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].suspend_enabled = 0,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].idle_enabled = 1,
#else /*CONFIG_MSM_STANDALONE_POWER_COLLAPSE*/
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].suspend_enabled = 0,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].idle_enabled = 0,
#endif /*CONFIG_MSM_STANDALONE_POWER_COLLAPSE*/
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].latency = 500,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE].residency = 6000,

	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].supported = 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].suspend_enabled
		= 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].idle_enabled = 0,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency = 443,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].residency = 1098,

	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].supported = 1,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].latency = 2,
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].residency = 0,
};

static struct resource qsd_spi_resources[] = {
	{
		.name   = "spi_irq_in",
		.start	= INT_SPI_INPUT,
		.end	= INT_SPI_INPUT,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_out",
		.start	= INT_SPI_OUTPUT,
		.end	= INT_SPI_OUTPUT,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_err",
		.start	= INT_SPI_ERROR,
		.end	= INT_SPI_ERROR,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_base",
		.start	= 0xA8000000,
		.end	= 0xA8000000 + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "spidm_channels",
		.flags  = IORESOURCE_DMA,
	},
	{
		.name   = "spidm_crci",
		.flags  = IORESOURCE_DMA,
	},
};

#define AMDH0_BASE_PHYS		0xAC200000
#define ADMH0_GP_CTL		(ct_adm_base + 0x3D8)
static int msm_qsd_spi_dma_config(void)
{
	void __iomem *ct_adm_base = 0;
	u32 spi_mux = 0;
	int ret = 0;

	ct_adm_base = ioremap(AMDH0_BASE_PHYS, PAGE_SIZE);
	if (!ct_adm_base) {
		pr_err("%s: Could not remap %x\n", __func__, AMDH0_BASE_PHYS);
		return -ENOMEM;
	}

	spi_mux = (ioread32(ADMH0_GP_CTL) & (0x3 << 12)) >> 12;

	qsd_spi_resources[4].start  = DMOV_USB_CHAN;
	qsd_spi_resources[4].end    = DMOV_TSIF_CHAN;

	switch (spi_mux) {
	case (1):
		qsd_spi_resources[5].start  = DMOV_HSUART1_RX_CRCI;
		qsd_spi_resources[5].end    = DMOV_HSUART1_TX_CRCI;
		break;
	case (2):
		qsd_spi_resources[5].start  = DMOV_HSUART2_RX_CRCI;
		qsd_spi_resources[5].end    = DMOV_HSUART2_TX_CRCI;
		break;
	case (3):
		qsd_spi_resources[5].start  = DMOV_CE_OUT_CRCI;
		qsd_spi_resources[5].end    = DMOV_CE_IN_CRCI;
		break;
	default:
		ret = -ENOENT;
	}

	iounmap(ct_adm_base);

	return ret;
}

static struct platform_device qsd_device_spi = {
	.name		= "spi_qsd",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(qsd_spi_resources),
	.resource	= qsd_spi_resources,
};

static struct msm_gpio qsd_spi_gpio_config_data[] = {
	{ GPIO_CFG(45, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "spi_clk" },
	{ GPIO_CFG(46, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "spi_cs0" },
	{ GPIO_CFG(47, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "spi_mosi" },
	{ GPIO_CFG(48, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "spi_miso" },
};

static int msm_qsd_spi_gpio_config(void)
{
	return msm_gpios_request_enable(qsd_spi_gpio_config_data,
		ARRAY_SIZE(qsd_spi_gpio_config_data));
}

static void msm_qsd_spi_gpio_release(void)
{
	msm_gpios_disable_free(qsd_spi_gpio_config_data,
		ARRAY_SIZE(qsd_spi_gpio_config_data));
}

static struct msm_spi_platform_data qsd_spi_pdata = {
	.max_clock_speed = 26331429,
	.clk_name = "spi_clk",
	.pclk_name = "spi_pclk",
	.gpio_config  = msm_qsd_spi_gpio_config,
	.gpio_release = msm_qsd_spi_gpio_release,
	.dma_config = msm_qsd_spi_dma_config,
};

static void __init msm_qsd_spi_init(void)
{
	qsd_device_spi.dev.platform_data = &qsd_spi_pdata;
}

#ifdef CONFIG_USB_EHCI_MSM
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{
        int rc;
        static int vbus_is_on;
        struct pm8058_gpio usb_vbus = {
                .direction      = PM_GPIO_DIR_OUT,
                .pull           = PM_GPIO_PULL_NO,
                .output_buffer  = PM_GPIO_OUT_BUF_CMOS,
                .output_value   = 1,
                .vin_sel        = 2,
                .out_strength   = PM_GPIO_STRENGTH_MED,
                .function       = PM_GPIO_FUNC_NORMAL,
                .inv_int_pol    = 0,
        };

        /* If VBUS is already on (or off), do nothing. */
        if (unlikely(on == vbus_is_on))
                return;

        if (on) {
                rc = pm8058_gpio_config(36, &usb_vbus);
                if (rc) {
                        pr_err("%s PMIC GPIO 36 write failed\n", __func__);
                        return;
                }
	} else {
		gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(36), 0);
	}

        vbus_is_on = on;
}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
        .phy_info   = (USB_PHY_INTEGRATED | USB_PHY_MODEL_45NM),
        .vbus_power = msm_hsusb_vbus_power,
        .power_budget   = 180,
};
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
static int hsusb_rpc_connect(int connect)
{
	if (connect)
		return msm_hsusb_rpc_connect();
	else
		return msm_hsusb_rpc_close();
}
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
static struct vreg *vreg_3p3;
static int msm_hsusb_ldo_init(int init)
{
/* < DTS2011063004178 caomingxing 20110630 begin */
/* we change the voltage to 3400 instead of 3075,
 * for preventing usb port from being not recognized by PC sometimes.
 */
#ifdef CONFIG_HUAWEI_KERNEL
	int def_vol = 3400;
#else
	uint32_t version = 0;
	int def_vol = 3400;

	version = socinfo_get_version();

	if (SOCINFO_VERSION_MAJOR(version) >= 2 &&
			SOCINFO_VERSION_MINOR(version) >= 1) {
		def_vol = 3075;
		pr_debug("%s: default voltage:%d\n", __func__, def_vol);
	}
#endif
/* DTS2011063004178 caomingxing 20110630 end > */
	if (init) {
		vreg_3p3 = vreg_get(NULL, "usb");
		if (IS_ERR(vreg_3p3))
			return PTR_ERR(vreg_3p3);
		vreg_set_level(vreg_3p3, def_vol);
	} else
		vreg_put(vreg_3p3);

	return 0;
}

static int msm_hsusb_ldo_enable(int enable)
{
	static int ldo_status;

	if (!vreg_3p3 || IS_ERR(vreg_3p3))
		return -ENODEV;

	if (ldo_status == enable)
		return 0;

	ldo_status = enable;

	if (enable)
		return vreg_enable(vreg_3p3);

	return vreg_disable(vreg_3p3);
}

static int msm_hsusb_ldo_set_voltage(int mV)
{
	static int cur_voltage = 3400;

	if (!vreg_3p3 || IS_ERR(vreg_3p3))
		return -ENODEV;

	if (cur_voltage == mV)
		return 0;

	cur_voltage = mV;

	pr_debug("%s: (%d)\n", __func__, mV);

	return vreg_set_level(vreg_3p3, mV);
}
#endif

#ifndef CONFIG_USB_EHCI_MSM
static int msm_hsusb_pmic_notif_init(void (*callback)(int online), int init);
#endif
static struct msm_otg_platform_data msm_otg_pdata = {
	.rpc_connect	= hsusb_rpc_connect,

#ifndef CONFIG_USB_EHCI_MSM
	.pmic_vbus_notif_init         = msm_hsusb_pmic_notif_init,
#else
	.vbus_power = msm_hsusb_vbus_power,
#endif
	.core_clk		 = 1,
	.pemp_level		 = PRE_EMPHASIS_WITH_20_PERCENT,
	.cdr_autoreset		 = CDR_AUTO_RESET_DISABLE,
	.drv_ampl		 = HS_DRV_AMPLITUDE_DEFAULT,
	.se1_gating		 = SE1_GATING_DISABLE,
	.chg_vbus_draw		 = hsusb_chg_vbus_draw,
	.chg_connected		 = hsusb_chg_connected,
	.chg_init		 = hsusb_chg_init,
	.ldo_enable		 = msm_hsusb_ldo_enable,
	.ldo_init		 = msm_hsusb_ldo_init,
	.ldo_set_voltage	 = msm_hsusb_ldo_set_voltage,
};

#ifdef CONFIG_USB_GADGET
static struct msm_hsusb_gadget_platform_data msm_gadget_pdata = {
	.is_phy_status_timer_on = 1,
};
#endif
#ifndef CONFIG_USB_EHCI_MSM
typedef void (*notify_vbus_state) (int);
notify_vbus_state notify_vbus_state_func_ptr;
int vbus_on_irq;
static irqreturn_t pmic_vbus_on_irq(int irq, void *data)
{
	pr_info("%s: vbus notification from pmic\n", __func__);

	(*notify_vbus_state_func_ptr) (1);

	return IRQ_HANDLED;
}
static int msm_hsusb_pmic_notif_init(void (*callback)(int online), int init)
{
	int ret;

	if (init) {
		if (!callback)
			return -ENODEV;

		notify_vbus_state_func_ptr = callback;
		vbus_on_irq = platform_get_irq_byname(&msm_device_otg,
			"vbus_on");
		if (vbus_on_irq <= 0) {
			pr_err("%s: unable to get vbus on irq\n", __func__);
			return -ENODEV;
		}

		ret = request_any_context_irq(vbus_on_irq, pmic_vbus_on_irq,
			IRQF_TRIGGER_RISING, "msm_otg_vbus_on", NULL);
		if (ret < 0) {
			pr_info("%s: request_irq for vbus_on"
				"interrupt failed\n", __func__);
			return ret;
		}
		msm_otg_pdata.pmic_vbus_irq = vbus_on_irq;
		return 0;
	} else {
		free_irq(vbus_on_irq, 0);
		notify_vbus_state_func_ptr = NULL;
		return 0;
	}
}
#endif

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
	/*< DTS2011040201879   songxiaoming 20110505 begin */
/* <DTS2010120101602 penghai 20101231 begin */
	.cached = 1,
/* DTS2010120101602 penghai 20101231 end> */
    /* DTS2011040201879   songxiaoming 20110505 end > */
};

static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};


static struct msm_gpio dtv_panel_gpios[] = {
	{ GPIO_CFG(120, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "wca_mclk" },
	{ GPIO_CFG(121, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "wca_sd0" },
	{ GPIO_CFG(122, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "wca_sd1" },
	{ GPIO_CFG(123, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "wca_sd2" },
	{ GPIO_CFG(124, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "dtv_pclk" },
	{ GPIO_CFG(125, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_en" },
	{ GPIO_CFG(126, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_vsync" },
	{ GPIO_CFG(127, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_hsync" },
	{ GPIO_CFG(128, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_data0" },
	{ GPIO_CFG(129, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_data1" },
	{ GPIO_CFG(130, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_data2" },
	{ GPIO_CFG(131, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_data3" },
	{ GPIO_CFG(132, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_data4" },
	{ GPIO_CFG(160, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_data5" },
	{ GPIO_CFG(161, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_data6" },
	{ GPIO_CFG(162, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_data7" },
	{ GPIO_CFG(163, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_data8" },
	{ GPIO_CFG(164, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_data9" },
	{ GPIO_CFG(165, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat10" },
	{ GPIO_CFG(166, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat11" },
	{ GPIO_CFG(167, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat12" },
	{ GPIO_CFG(168, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat13" },
	{ GPIO_CFG(169, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat14" },
	{ GPIO_CFG(170, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat15" },
	{ GPIO_CFG(171, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat16" },
	{ GPIO_CFG(172, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat17" },
	{ GPIO_CFG(173, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat18" },
	{ GPIO_CFG(174, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat19" },
	{ GPIO_CFG(175, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat20" },
	{ GPIO_CFG(176, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat21" },
	{ GPIO_CFG(177, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat22" },
	{ GPIO_CFG(178, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat23" },
};


#ifdef HDMI_RESET
static unsigned dtv_reset_gpio =
	GPIO_CFG(37, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);
#endif
#if defined(CONFIG_FB_MSM_HDMI_ADV7520_PANEL) || defined(CONFIG_BOSCH_BMA150)
/* there is an i2c address conflict between adv7520 and bma150 sensor after
 * power up on fluid. As a solution, the default address of adv7520's packet
 * memory is changed as soon as possible
 */
static int __init fluid_i2c_address_fixup(void)
{
	unsigned char wBuff[16];
	unsigned char rBuff[16];
	struct i2c_msg msgs[3];
	int res;
	int rc = -EINVAL;
	struct vreg *vreg_ldo8;
	struct i2c_adapter *adapter;

	if (machine_is_msm7x30_fluid()) {
		adapter = i2c_get_adapter(0);
		if (!adapter) {
			pr_err("%s: invalid i2c adapter\n", __func__);
			return PTR_ERR(adapter);
		}

		/* turn on LDO8 */
		vreg_ldo8 = vreg_get(NULL, "gp7");
		if (!vreg_ldo8) {
			pr_err("%s: VREG L8 get failed\n", __func__);
			goto adapter_put;
		}

		rc = vreg_set_level(vreg_ldo8, 1800);
		if (rc) {
			pr_err("%s: VREG L8 set failed\n", __func__);
			goto ldo8_put;
		}

		rc = vreg_enable(vreg_ldo8);
		if (rc) {
			pr_err("%s: VREG L8 enable failed\n", __func__);
			goto ldo8_put;
		}

		/* change packet memory address to 0x74 */
		wBuff[0] = 0x45;
		wBuff[1] = 0x74;

		msgs[0].addr = ADV7520_I2C_ADDR;
		msgs[0].flags = 0;
		msgs[0].buf = (unsigned char *) wBuff;
		msgs[0].len = 2;

		res = i2c_transfer(adapter, msgs, 1);
		if (res != 1) {
			pr_err("%s: error writing adv7520\n", __func__);
			goto ldo8_disable;
		}

		/* powerdown adv7520 using bit 6 */
		/* i2c read first */
		wBuff[0] = 0x41;

		msgs[0].addr = ADV7520_I2C_ADDR;
		msgs[0].flags = 0;
		msgs[0].buf = (unsigned char *) wBuff;
		msgs[0].len = 1;

		msgs[1].addr = ADV7520_I2C_ADDR;
		msgs[1].flags = I2C_M_RD;
		msgs[1].buf = rBuff;
		msgs[1].len = 1;
		res = i2c_transfer(adapter, msgs, 2);
		if (res != 2) {
			pr_err("%s: error reading adv7520\n", __func__);
			goto ldo8_disable;
		}

		/* i2c write back */
		wBuff[0] = 0x41;
		wBuff[1] = rBuff[0] | 0x40;

		msgs[0].addr = ADV7520_I2C_ADDR;
		msgs[0].flags = 0;
		msgs[0].buf = (unsigned char *) wBuff;
		msgs[0].len = 2;

		res = i2c_transfer(adapter, msgs, 1);
		if (res != 1) {
			pr_err("%s: error writing adv7520\n", __func__);
			goto ldo8_disable;
		}

		/* for successful fixup, we release the i2c adapter */
		/* but leave ldo8 on so that the adv7520 is not repowered */
		i2c_put_adapter(adapter);
		pr_info("%s: fluid i2c address conflict resolved\n", __func__);
	}
	return 0;

ldo8_disable:
	vreg_disable(vreg_ldo8);
ldo8_put:
	vreg_put(vreg_ldo8);
adapter_put:
	i2c_put_adapter(adapter);
	return rc;
}
subsys_initcall_sync(fluid_i2c_address_fixup);
#endif


static int dtv_panel_power(int on)
{
	int flag_on = !!on;
	static int dtv_power_save_on;
	int rc;

	if (dtv_power_save_on == flag_on)
		return 0;

	dtv_power_save_on = flag_on;
	pr_info("%s: %d\n", __func__, on);

#ifdef HDMI_RESET
	if (on) {
		/* reset Toshiba WeGA chip -- toggle reset pin -- gpio_180 */
		rc = gpio_tlmm_config(dtv_reset_gpio, GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
				       __func__, dtv_reset_gpio, rc);
			return rc;
		}

		/* bring reset line low to hold reset*/
		gpio_set_value(37, 0);
	}
#endif

	if (on) {
		rc = msm_gpios_enable(dtv_panel_gpios,
				ARRAY_SIZE(dtv_panel_gpios));
		if (rc < 0) {
			printk(KERN_ERR "%s: gpio enable failed: %d\n",
				__func__, rc);
			return rc;
		}
	} else {
		rc = msm_gpios_disable(dtv_panel_gpios,
				ARRAY_SIZE(dtv_panel_gpios));
		if (rc < 0) {
			printk(KERN_ERR "%s: gpio disable failed: %d\n",
				__func__, rc);
			return rc;
		}
	}

	mdelay(5);		/* ensure power is stable */

#ifdef HDMI_RESET
	if (on) {
		gpio_set_value(37, 1);	/* bring reset line high */
		mdelay(10);		/* 10 msec before IO can be accessed */
	}
#endif

	return rc;
}

static struct lcdc_platform_data dtv_pdata = {
	.lcdc_power_save   = dtv_panel_power,
};

/* < DTS2011041200623 xuhui 20110412 begin */
/*disable QC's In Band Sleep mode with BCM4329 bluetooth chip*/
#ifdef CONFIG_HUAWEI_KERNEL
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
       .inject_rx_on_wakeup = 1,
       .rx_to_inject = 0xFD,
};
#endif
/* DTS2011041200623 xuhui 20110412 end > */

static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	}
};

static int msm_fb_detect_panel(const char *name)
{
	/* <DTS2010072202961 hufeng 20100722 begin */
	if (!strcmp(name, "mddi_orise"))
		return -EPERM;
	/* DTS2010072202961 hufeng 20100722 end> */

	return -ENODEV;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
	.mddi_prescan = 1,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_fb_resources),
	.resource       = msm_fb_resources,
	.dev    = {
		.platform_data = &msm_fb_pdata,
	}
};

static struct platform_device msm_migrate_pages_device = {
	.name   = "msm_migrate_pages",
	.id     = -1,
};

static struct android_pmem_platform_data android_pmem_kernel_ebi1_pdata = {
       .name = PMEM_KERNEL_EBI1_DATA_NAME,
	/* if no allocator_type, defaults to PMEM_ALLOCATORTYPE_BITMAP,
	* the only valid choice at this time. The board structure is
	* set to all zeros by the C runtime initialization and that is now
	* the enum value of PMEM_ALLOCATORTYPE_BITMAP, now forced to 0 in
	* include/linux/android_pmem.h.
	*/
       .cached = 0,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
       .name = "pmem_adsp",
       .allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
       .cached = 0,
};

static struct android_pmem_platform_data android_pmem_audio_pdata = {
       .name = "pmem_audio",
       .allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
       .cached = 0,
};

static struct platform_device android_pmem_kernel_ebi1_device = {
       .name = "android_pmem",
       .id = 1,
       .dev = { .platform_data = &android_pmem_kernel_ebi1_pdata },
};

static struct platform_device android_pmem_adsp_device = {
       .name = "android_pmem",
       .id = 2,
       .dev = { .platform_data = &android_pmem_adsp_pdata },
};

static struct platform_device android_pmem_audio_device = {
       .name = "android_pmem",
       .id = 4,
       .dev = { .platform_data = &android_pmem_audio_pdata },
};

static struct kgsl_platform_data kgsl_pdata = {
#ifdef CONFIG_MSM_NPA_SYSTEM_BUS
	/* NPA Flow IDs */
	.high_axi_3d = MSM_AXI_FLOW_3D_GPU_HIGH,
	.high_axi_2d = MSM_AXI_FLOW_2D_GPU_HIGH,
#else
	/* AXI rates in KHz */
	.high_axi_3d = 192000,
	.high_axi_2d = 192000,
#endif
	.max_grp2d_freq = 0,
	.min_grp2d_freq = 0,
	.set_grp2d_async = NULL, /* HW workaround, run Z180 SYNC @ 192 MHZ */
	.max_grp3d_freq = 245760000,
	.min_grp3d_freq = 192 * 1000*1000,
	.set_grp3d_async = set_grp3d_async,
	.imem_clk_name = "imem_clk",
	.grp3d_clk_name = "grp_clk",
	.grp3d_pclk_name = "grp_pclk",
#ifdef CONFIG_MSM_KGSL_2D
	.grp2d0_clk_name = "grp_2d_clk",
	.grp2d0_pclk_name = "grp_2d_pclk",
#else
	.grp2d0_clk_name = NULL,
#endif
	.idle_timeout_3d = HZ/20,
	.idle_timeout_2d = HZ/10,
	.nap_allowed = true,

#ifdef CONFIG_KGSL_PER_PROCESS_PAGE_TABLE
	.pt_va_size = SZ_32M,
	/* Maximum of 32 concurrent processes */
	.pt_max_count = 32,
#else
	.pt_va_size = SZ_128M,
	/* We only ever have one pagetable for everybody */
	.pt_max_count = 1,
#endif
};

static struct resource kgsl_resources[] = {
	{
		.name = "kgsl_reg_memory",
		.start = 0xA3500000, /* 3D GRP address */
		.end = 0xA351ffff,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "kgsl_yamato_irq",
		.start = INT_GRP_3D,
		.end = INT_GRP_3D,
		.flags = IORESOURCE_IRQ,
	},
	{
		.name = "kgsl_2d0_reg_memory",
		.start = 0xA3900000, /* Z180 base address */
		.end = 0xA3900FFF,
		.flags = IORESOURCE_MEM,
	},
	{
		.name  = "kgsl_2d0_irq",
		.start = INT_GRP_2D,
		.end = INT_GRP_2D,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device msm_device_kgsl = {
	.name = "kgsl",
	.id = -1,
	.num_resources = ARRAY_SIZE(kgsl_resources),
	.resource = kgsl_resources,
	.dev = {
		.platform_data = &kgsl_pdata,
	},
};

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)

#define QCE_SIZE		0x10000
#define QCE_0_BASE		0xA8400000

#define QCE_HW_KEY_SUPPORT	1

#define QCE_SHARE_CE_RESOURCE	0
#define QCE_CE_SHARED		0

static struct resource qce_resources[] = {
	[0] = {
		.start = QCE_0_BASE,
		.end = QCE_0_BASE + QCE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name = "crypto_channels",
		.start = DMOV_CE_IN_CHAN,
		.end = DMOV_CE_OUT_CHAN,
		.flags = IORESOURCE_DMA,
	},
	[2] = {
		.name = "crypto_crci_in",
		.start = DMOV_CE_IN_CRCI,
		.end = DMOV_CE_IN_CRCI,
		.flags = IORESOURCE_DMA,
	},
	[3] = {
		.name = "crypto_crci_out",
		.start = DMOV_CE_OUT_CRCI,
		.end = DMOV_CE_OUT_CRCI,
		.flags = IORESOURCE_DMA,
	},
	[4] = {
		.name = "crypto_crci_hash",
		.start = DMOV_CE_HASH_CRCI,
		.end = DMOV_CE_HASH_CRCI,
		.flags = IORESOURCE_DMA,
	},
};

#endif

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE)

static struct msm_ce_hw_support qcrypto_ce_hw_suppport = {
	.ce_shared = QCE_CE_SHARED,
	.shared_ce_resource = QCE_SHARE_CE_RESOURCE,
	.hw_key_support = QCE_HW_KEY_SUPPORT,
};

static struct platform_device qcrypto_device = {
	.name		= "qcrypto",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(qce_resources),
	.resource	= qce_resources,
	.dev		= {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.platform_data = &qcrypto_ce_hw_suppport,
	},
};
#endif

#if defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)

static struct msm_ce_hw_support qcedev_ce_hw_suppport = {
	.ce_shared = QCE_CE_SHARED,
	.shared_ce_resource = QCE_SHARE_CE_RESOURCE,
	.hw_key_support = QCE_HW_KEY_SUPPORT,
};
static struct platform_device qcedev_device = {
	.name		= "qce",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(qce_resources),
	.resource	= qce_resources,
	.dev		= {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.platform_data = &qcedev_ce_hw_suppport,
	},
};
#endif

/* <DTS2010072202961 hufeng 20100722 begin */
/* removed several lines */
/* DTS2010072202961 hufeng 20100722 end> */

/* < BU5D10320 lijianzhao 20100521 begin */
        /*delete some lines*/
/*In the modem side ,LCD have been power on*/
static int display_common_power(int on)
{
	int rc = 0;
    if (on) 
    {
        /*pmapp_display_clock_config(1);*/
    } else 
    {
        /*pmapp_display_clock_config(0);*/
    }
	return rc;
}
/* BU5D10320 lijianzhao 20100521 end > */
static int msm_fb_mddi_sel_clk(u32 *clk_rate)
{
	*clk_rate *= 2;
	return 0;
}

static int msm_fb_mddi_client_power(u32 client_id)
{
/*< DTS2011041700393 lijianzhao 20110417 begin */
/* modify for 4125 baseline */
#ifndef CONFIG_HUAWEI_KERNEL
	struct vreg *vreg_ldo20, *vreg_ldo16;
	int rc;

	printk(KERN_NOTICE "\n client_id = 0x%x", client_id);
	/* Check if it is Quicklogic client */
	if (client_id == 0xc5835800) {
		printk(KERN_NOTICE "\n Quicklogic MDDI client");
		other_mddi_client = 0;
		vreg_ldo16 = vreg_get(NULL, "gp10");

		if (IS_ERR(vreg_ldo16)) {
			rc = PTR_ERR(vreg_ldo16);
			pr_err("%s: gp10 vreg get failed (%d)\n",
				   __func__, rc);
			return rc;
		}

		rc = vreg_disable(vreg_ldo16);
		if (rc) {
			pr_err("%s: LDO16 vreg enable failed (%d)\n",
				   __func__, rc);
			return rc;
		}
	} else {
		printk(KERN_NOTICE "\n Non-Quicklogic MDDI client");
		quickvx_mddi_client = 0;
		gpio_set_value(97, 0);
		gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(
			PMIC_GPIO_QUICKVX_CLK), 0);

		vreg_ldo20 = vreg_get(NULL, "gp13");

		if (IS_ERR(vreg_ldo20)) {
			rc = PTR_ERR(vreg_ldo20);
			pr_err("%s: gp13 vreg get failed (%d)\n",
				   __func__, rc);
			return rc;
		}
		rc = vreg_set_level(vreg_ldo20, 1500);
		if (rc) {
			pr_err("%s: vreg LDO20 set level failed (%d)\n",
			       __func__, rc);
			return rc;
		}
	}
	#endif
/* DTS2011041700393 lijianzhao 20110417 end >*/	
	return 0;
}

static struct mddi_platform_data mddi_pdata = {
	.mddi_power_save = display_common_power,
	.mddi_sel_clk = msm_fb_mddi_sel_clk,
	.mddi_client_power = msm_fb_mddi_client_power,
};

int mdp_core_clk_rate_table[] = {
	122880000,
	122880000,
	122880000,
	192000000,
};

static struct msm_panel_common_pdata mdp_pdata = {
	.hw_revision_addr = 0xac001270,
	.gpio = 30,
	.mdp_core_clk_rate = 122880000,
	.mdp_core_clk_table = mdp_core_clk_rate_table,
	.num_mdp_clk = ARRAY_SIZE(mdp_core_clk_rate_table),
};

/* <DTS2010072202961 hufeng 20100722 begin */
/* removed several lines */
/* DTS2010072202961 hufeng 20100722 end> */
/*< DTS2010082601848 lijianzhao 20100826 begin */
static int atv_dac_power(int on)
{
	int rc = 0;
	struct vreg *vreg_s4, *vreg_ldo9;

	vreg_s4 = vreg_get(NULL, "s4");
	if (IS_ERR(vreg_s4)) {
		rc = PTR_ERR(vreg_s4);
		pr_err("%s: s4 vreg get failed (%d)\n",
			__func__, rc);
		return -1;
	}
	vreg_ldo9 = vreg_get(NULL, "gp1");
	if (IS_ERR(vreg_ldo9)) {
		rc = PTR_ERR(vreg_ldo9);
		pr_err("%s: ldo9 vreg get failed (%d)\n",
			__func__, rc);
		return rc;
	}

	if (on) {
		rc = vreg_enable(vreg_s4);
		if (rc) {
			pr_err("%s: s4 vreg enable failed (%d)\n",
				__func__, rc);
			return rc;
		}
		rc = vreg_enable(vreg_ldo9);
		if (rc) {
			pr_err("%s: ldo9 vreg enable failed (%d)\n",
				__func__, rc);
			return rc;
		}
	} else {
		rc = vreg_disable(vreg_ldo9);
		if (rc) {
			pr_err("%s: ldo9 vreg disable failed (%d)\n",
				   __func__, rc);
			return rc;
		}
		rc = vreg_disable(vreg_s4);
		if (rc) {
			pr_err("%s: s4 vreg disable failed (%d)\n",
				   __func__, rc);
			return rc;
		}
	}
	return rc;
}

static struct tvenc_platform_data atv_pdata = {
	.poll		 = 1,
	.pm_vid_en	 = atv_dac_power,
};

static void __init msm_fb_add_devices(void)
{
	msm_fb_register_device("mdp", &mdp_pdata);
	msm_fb_register_device("pmdh", &mddi_pdata);
	/* <DTS2010072202961 hufeng 20100722 begin */
	/* removed several lines */
	/* DTS2010072202961 hufeng 20100722 end> */
	msm_fb_register_device("dtv", &dtv_pdata);
	msm_fb_register_device("tvenc", &atv_pdata);
#ifdef CONFIG_FB_MSM_TVOUT
	msm_fb_register_device("tvout_device", NULL);
#endif
}
/* DTS2010082601848 lijianzhao 20100826 end >*/
/* DTS2010071700383 haoqingtao 20100716 end> */


/* <DTS2010072202961 hufeng 20100722 begin */
/* removed several lines */
/* DTS2010072202961 hufeng 20100722 end> */

/*< DTS2011012004291 xuhui 20110120 begin */
//don't delete QC's bt code
#ifdef CONFIG_HUAWEI_KERNEL
#if defined(CONFIG_MARIMBA_CORE) && \
   (defined(CONFIG_MSM_BT_POWER) || defined(CONFIG_MSM_BT_POWER_MODULE))
static struct platform_device msm_bt_power_device = {
	.name = "bt_power",
	.id     = -1
};

enum {
	BT_RFR,
	BT_CTS,
	BT_RX,
	BT_TX,
};

static struct msm_gpio bt_config_power_on[] = {
	{ GPIO_CFG(134, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
		"UART1DM_RFR" },
	{ GPIO_CFG(135, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
		"UART1DM_CTS" },
	{ GPIO_CFG(136, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
		"UART1DM_Rx" },
	{ GPIO_CFG(137, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
		"UART1DM_Tx" }
};

static struct msm_gpio bt_config_power_off[] = {
	{ GPIO_CFG(134, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,   GPIO_CFG_2MA),
		"UART1DM_RFR" },
	{ GPIO_CFG(135, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,   GPIO_CFG_2MA),
		"UART1DM_CTS" },
	{ GPIO_CFG(136, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,   GPIO_CFG_2MA),
		"UART1DM_Rx" },
	{ GPIO_CFG(137, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,   GPIO_CFG_2MA),
		"UART1DM_Tx" }
};

static const char *vregs_bt_marimba_name[] = {
	"s3",
	"s2",
	"gp16",
	"wlan"
};
static struct vreg *vregs_bt_marimba[ARRAY_SIZE(vregs_bt_marimba_name)];

static const char *vregs_bt_bahama_name[] = {
	"s3",
	"usb2",
	"s2",
	"wlan"
};
static struct vreg *vregs_bt_bahama[ARRAY_SIZE(vregs_bt_bahama_name)];

static u8 bahama_version;

static int marimba_bt(int on)
{
	int rc;
	int i;
	struct marimba config = { .mod_id = MARIMBA_SLAVE_ID_MARIMBA };

	struct marimba_config_register {
		u8 reg;
		u8 value;
		u8 mask;
	};

	struct marimba_variant_register {
		const size_t size;
		const struct marimba_config_register *set;
	};

	const struct marimba_config_register *p;

	u8 version;

	const struct marimba_config_register v10_bt_on[] = {
		{ 0xE5, 0x0B, 0x0F },
		{ 0x05, 0x02, 0x07 },
		{ 0x06, 0x88, 0xFF },
		{ 0xE7, 0x21, 0x21 },
		{ 0xE3, 0x38, 0xFF },
		{ 0xE4, 0x06, 0xFF },
	};

	const struct marimba_config_register v10_bt_off[] = {
		{ 0xE5, 0x0B, 0x0F },
		{ 0x05, 0x08, 0x0F },
		{ 0x06, 0x88, 0xFF },
		{ 0xE7, 0x00, 0x21 },
		{ 0xE3, 0x00, 0xFF },
		{ 0xE4, 0x00, 0xFF },
	};

	const struct marimba_config_register v201_bt_on[] = {
		{ 0x05, 0x08, 0x07 },
		{ 0x06, 0x88, 0xFF },
		{ 0xE7, 0x21, 0x21 },
		{ 0xE3, 0x38, 0xFF },
		{ 0xE4, 0x06, 0xFF },
	};

	const struct marimba_config_register v201_bt_off[] = {
		{ 0x05, 0x08, 0x07 },
		{ 0x06, 0x88, 0xFF },
		{ 0xE7, 0x00, 0x21 },
		{ 0xE3, 0x00, 0xFF },
		{ 0xE4, 0x00, 0xFF },
	};

	const struct marimba_config_register v210_bt_on[] = {
		{ 0xE9, 0x01, 0x01 },
		{ 0x06, 0x88, 0xFF },
		{ 0xE7, 0x21, 0x21 },
		{ 0xE3, 0x38, 0xFF },
		{ 0xE4, 0x06, 0xFF },
	};

	const struct marimba_config_register v210_bt_off[] = {
		{ 0x06, 0x88, 0xFF },
		{ 0xE7, 0x00, 0x21 },
		{ 0xE9, 0x00, 0x01 },
		{ 0xE3, 0x00, 0xFF },
		{ 0xE4, 0x00, 0xFF },
	};

	const struct marimba_variant_register bt_marimba[2][4] = {
		{
			{ ARRAY_SIZE(v10_bt_off), v10_bt_off },
			{ 0, NULL },
			{ ARRAY_SIZE(v201_bt_off), v201_bt_off },
			{ ARRAY_SIZE(v210_bt_off), v210_bt_off }
		},
		{
			{ ARRAY_SIZE(v10_bt_on), v10_bt_on },
			{ 0, NULL },
			{ ARRAY_SIZE(v201_bt_on), v201_bt_on },
			{ ARRAY_SIZE(v210_bt_on), v210_bt_on }
		}
	};

	on = on ? 1 : 0;

	rc = marimba_read_bit_mask(&config, 0x11,  &version, 1, 0x1F);
	if (rc < 0) {
		printk(KERN_ERR
			"%s: version read failed: %d\n",
			__func__, rc);
		return rc;
	}

	if ((version >= ARRAY_SIZE(bt_marimba[on])) ||
	    (bt_marimba[on][version].size == 0)) {
		printk(KERN_ERR
			"%s: unsupported version\n",
			__func__);
		return -EIO;
	}

	p = bt_marimba[on][version].set;

	printk(KERN_INFO "%s: found version %d\n", __func__, version);

	for (i = 0; i < bt_marimba[on][version].size; i++) {
		u8 value = (p+i)->value;
		rc = marimba_write_bit_mask(&config,
			(p+i)->reg,
			&value,
			sizeof((p+i)->value),
			(p+i)->mask);
		if (rc < 0) {
			printk(KERN_ERR
				"%s: reg %d write failed: %d\n",
				__func__, (p+i)->reg, rc);
			return rc;
		}
		printk(KERN_INFO "%s: reg 0x%02x value 0x%02x mask 0x%02x\n",
				__func__, (p+i)->reg,
				value, (p+i)->mask);
	}
	return 0;
}

static int bahama_bt(int on)
{
	int rc;
	int i;
	struct marimba config = { .mod_id = SLAVE_ID_BAHAMA };

	struct bahama_variant_register {
		const size_t size;
		const struct bahama_config_register *set;
	};

	const struct bahama_config_register *p;


	const struct bahama_config_register v10_bt_on[] = {
		{ 0xE9, 0x00, 0xFF },
		{ 0xF4, 0x80, 0xFF },
		{ 0xF0, 0x06, 0xFF },
		{ 0xE4, 0x00, 0xFF },
		{ 0xE5, 0x00, 0x0F },
#ifdef CONFIG_WLAN
		{ 0xE6, 0x38, 0x7F },
		{ 0xE7, 0x06, 0xFF },
#endif
		{ 0x11, 0x13, 0xFF },
		{ 0xE9, 0x21, 0xFF },
		{ 0x01, 0x0C, 0x1F },
		{ 0x01, 0x08, 0x1F },
	};

	const struct bahama_config_register v20_bt_on_fm_off[] = {
		{ 0x11, 0x0C, 0xFF },
		{ 0x13, 0x01, 0xFF },
		{ 0xF4, 0x80, 0xFF },
		{ 0xF0, 0x00, 0xFF },
		{ 0xE9, 0x00, 0xFF },
#ifdef CONFIG_WLAN
		{ 0x81, 0x00, 0xFF },
		{ 0x82, 0x00, 0xFF },
		{ 0xE6, 0x38, 0x7F },
		{ 0xE7, 0x06, 0xFF },
#endif
		{ 0xE9, 0x21, 0xFF }
	};

	const struct bahama_config_register v20_bt_on_fm_on[] = {
		{ 0x11, 0x0C, 0xFF },
		{ 0x13, 0x01, 0xFF },
		{ 0xF4, 0x86, 0xFF },
		{ 0xF0, 0x06, 0xFF },
		{ 0xE9, 0x00, 0xFF },
#ifdef CONFIG_WLAN
		{ 0x81, 0x00, 0xFF },
		{ 0x82, 0x00, 0xFF },
		{ 0xE6, 0x38, 0x7F },
		{ 0xE7, 0x06, 0xFF },
#endif
		{ 0xE9, 0x21, 0xFF }
	};

	const struct bahama_config_register v10_bt_off[] = {
		{ 0xE9, 0x00, 0xFF },
	};

	const struct bahama_config_register v20_bt_off_fm_off[] = {
		{ 0xF4, 0x84, 0xFF },
		{ 0xF0, 0x04, 0xFF },
		{ 0xE9, 0x00, 0xFF }
	};

	const struct bahama_config_register v20_bt_off_fm_on[] = {
		{ 0xF4, 0x86, 0xFF },
		{ 0xF0, 0x06, 0xFF },
		{ 0xE9, 0x00, 0xFF }
	};

	const struct bahama_variant_register bt_bahama[2][3] = {
		{
			{ ARRAY_SIZE(v10_bt_off), v10_bt_off },
			{ ARRAY_SIZE(v20_bt_off_fm_off), v20_bt_off_fm_off },
			{ ARRAY_SIZE(v20_bt_off_fm_on), v20_bt_off_fm_on }
		},
		{
			{ ARRAY_SIZE(v10_bt_on), v10_bt_on },
			{ ARRAY_SIZE(v20_bt_on_fm_off), v20_bt_on_fm_off },
			{ ARRAY_SIZE(v20_bt_on_fm_on), v20_bt_on_fm_on }
		}
	};

	u8 offset = 0; /* index into bahama configs */

	/* Init mutex to get/set FM/BT status respectively */
	mutex_init(&config.xfer_lock);

	on = on ? 1 : 0;

	bahama_version = read_bahama_ver();

	if (bahama_version == VER_UNSUPPORTED) {
		dev_err(&msm_bt_power_device.dev,
			"%s: unsupported version\n",
			__func__);
		return -EIO;
	}

	if (bahama_version == VER_2_0) {
		if (marimba_get_fm_status(&config))
			offset = 0x01;
	}

	p = bt_bahama[on][bahama_version + offset].set;

	dev_info(&msm_bt_power_device.dev,
		"%s: found version %d\n", __func__, bahama_version);

	for (i = 0; i < bt_bahama[on][bahama_version + offset].size; i++) {
		u8 value = (p+i)->value;
		rc = marimba_write_bit_mask(&config,
			(p+i)->reg,
			&value,
			sizeof((p+i)->value),
			(p+i)->mask);
		if (rc < 0) {
			dev_err(&msm_bt_power_device.dev,
				"%s: reg %d write failed: %d\n",
				__func__, (p+i)->reg, rc);
			return rc;
		}
		dev_info(&msm_bt_power_device.dev,
			"%s: reg 0x%02x write value 0x%02x mask 0x%02x\n",
				__func__, (p+i)->reg,
				value, (p+i)->mask);
	}
	/* Update BT status */
	if (on)
		marimba_set_bt_status(&config, true);
	else
		marimba_set_bt_status(&config, false);

	/* Destory mutex */
	mutex_destroy(&config.xfer_lock);

	if (bahama_version == VER_2_0 && on) { /* variant of bahama v2 */
		/* Disable s2 as bahama v2 uses internal LDO regulator */
		for (i = 0; i < ARRAY_SIZE(vregs_bt_bahama_name); i++) {
			if (!strcmp(vregs_bt_bahama_name[i], "s2")) {
				rc = vreg_disable(vregs_bt_bahama[i]);
				if (rc < 0) {
					printk(KERN_ERR
						"%s: vreg %s disable "
						"failed (%d)\n",
						__func__,
						vregs_bt_bahama_name[i], rc);
					return -EIO;
				}
				rc = pmapp_vreg_level_vote("BTPW",
								PMAPP_VREG_S2,
								0);
				if (rc < 0) {
					printk(KERN_ERR "%s: vreg "
						"level off failed (%d)\n",
						__func__, rc);
					return -EIO;
				}
				printk(KERN_INFO "%s: vreg disable & "
					"level off successful (%d)\n",
					__func__, rc);
			}
		}
	}

	return 0;
}

static int bluetooth_power_regulators(int on, int bahama_not_marimba)
{
	int i, rc;
	const char **vregs_name;
	struct vreg **vregs;
	int vregs_size;

	if (bahama_not_marimba) {
		vregs_name = vregs_bt_bahama_name;
		vregs = vregs_bt_bahama;
		vregs_size = ARRAY_SIZE(vregs_bt_bahama_name);
	} else {
		vregs_name = vregs_bt_marimba_name;
		vregs = vregs_bt_marimba;
		vregs_size = ARRAY_SIZE(vregs_bt_marimba_name);
	}

	for (i = 0; i < vregs_size; i++) {
		if (bahama_not_marimba && (bahama_version == VER_2_0) &&
			!on && !strcmp(vregs_bt_bahama_name[i], "s2"))
			continue;
		rc = on ? vreg_enable(vregs[i]) :
			  vreg_disable(vregs[i]);
		if (rc < 0) {
			printk(KERN_ERR "%s: vreg %s %s failed (%d)\n",
				__func__, vregs_name[i],
			       on ? "enable" : "disable", rc);
			return -EIO;
		}
	}
	return 0;
}

static int bluetooth_power(int on)
{
	int rc;
	const char *id = "BTPW";

	int bahama_not_marimba = bahama_present();

	if (bahama_not_marimba == -1) {
		printk(KERN_WARNING "%s: bahama_present: %d\n",
				__func__, bahama_not_marimba);
		return -ENODEV;
	}

	if (on) {
		rc = pmapp_vreg_level_vote(id, PMAPP_VREG_S2, 1300);
		if (rc < 0) {
			printk(KERN_ERR "%s: vreg level on failed (%d)\n",
				__func__, rc);
			return rc;
		}

		rc = bluetooth_power_regulators(on, bahama_not_marimba);
		if (rc < 0)
			return -EIO;

		rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_DO,
					  PMAPP_CLOCK_VOTE_ON);
		if (rc < 0)
			return -EIO;

		if (machine_is_msm8x55_svlte_surf() ||
				machine_is_msm8x55_svlte_ffa()) {
					rc = marimba_gpio_config(1);
					if (rc < 0)
						return -EIO;
		}

		rc = (bahama_not_marimba ? bahama_bt(on) : marimba_bt(on));
		if (rc < 0)
			return -EIO;

		msleep(10);

		rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_DO,
					  PMAPP_CLOCK_VOTE_PIN_CTRL);
		if (rc < 0)
			return -EIO;

		if (machine_is_msm8x55_svlte_surf() ||
				machine_is_msm8x55_svlte_ffa()) {
					rc = marimba_gpio_config(0);
					if (rc < 0)
						return -EIO;
		}

		rc = msm_gpios_enable(bt_config_power_on,
			ARRAY_SIZE(bt_config_power_on));

		if (rc < 0)
			return rc;

	} else {
		rc = msm_gpios_enable(bt_config_power_off,
					ARRAY_SIZE(bt_config_power_off));
		if (rc < 0)
			return rc;

		/* check for initial RFKILL block (power off) */
		if (platform_get_drvdata(&msm_bt_power_device) == NULL)
			goto out;

		rc = (bahama_not_marimba ? bahama_bt(on) : marimba_bt(on));
		if (rc < 0)
			return -EIO;

		rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_DO,
					  PMAPP_CLOCK_VOTE_OFF);
		if (rc < 0)
			return -EIO;

		rc = bluetooth_power_regulators(on, bahama_not_marimba);
		if (rc < 0)
			return -EIO;

		if (bahama_version == VER_1_0) {
			rc = pmapp_vreg_level_vote(id, PMAPP_VREG_S2, 0);
			if (rc < 0) {
				printk(KERN_ERR "%s: vreg level off failed "
				"(%d)\n", __func__, rc);
				return -EIO;
			}
		}
	}

out:
	printk(KERN_DEBUG "Bluetooth power switch: %d\n", on);

	return 0;
}

static void __init bt_power_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(vregs_bt_marimba_name); i++) {
		vregs_bt_marimba[i] = vreg_get(NULL, vregs_bt_marimba_name[i]);
		if (IS_ERR(vregs_bt_marimba[i])) {
			printk(KERN_ERR "%s: vreg get %s failed (%ld)\n",
			       __func__, vregs_bt_marimba_name[i],
			       PTR_ERR(vregs_bt_marimba[i]));
			return;
		}
	}

	for (i = 0; i < ARRAY_SIZE(vregs_bt_bahama_name); i++) {
		vregs_bt_bahama[i] = vreg_get(NULL, vregs_bt_bahama_name[i]);
		if (IS_ERR(vregs_bt_bahama[i])) {
			printk(KERN_ERR "%s: vreg get %s failed (%ld)\n",
			       __func__, vregs_bt_bahama_name[i],
			       PTR_ERR(vregs_bt_bahama[i]));
			return;
		}
	}

    msm_bt_power_device.dev.platform_data = &bluetooth_power;
}
#else
#define bt_power_init(x) do {} while (0)
#endif

#endif
/*don't do all bt ops here:*/
#ifndef CONFIG_HUAWEI_KERNEL
static struct platform_device msm_bt_power_device = {
    .name = "bt_power",
    .id     = -1
};

enum {
    BT_WAKE,
    BT_RFR,
    BT_CTS,
    BT_RX,
    BT_TX,
    BT_PCM_DOUT,
    BT_PCM_DIN,
    BT_PCM_SYNC,
    BT_PCM_CLK,
    BT_HOST_WAKE,
};
//xh start
/* config all msm bt gpio here!*/

static struct msm_gpio bt_config_bcm4329_power_on[] = {
    { GPIO_CFG(GPIO_BT_UART_RTS, GPIO_BT_FUN_1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
        "UART1DM_RFR" },
    { GPIO_CFG(GPIO_BT_UART_CTS, GPIO_BT_FUN_1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
        "UART1DM_CTS" },
    { GPIO_CFG(GPIO_BT_RX, GPIO_BT_FUN_1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
        "UART1DM_Rx" },
    { GPIO_CFG(GPIO_BT_TX, GPIO_BT_FUN_1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
        "UART1DM_Tx" },
    /*following 2 are the wakeup between 4329 and MSM*/
    { GPIO_CFG(GPIO_BT_WAKE_BT, GPIO_BT_FUN_0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,  GPIO_CFG_2MA ),
        "MSM_WAKE_BT"  },
    { GPIO_CFG(GPIO_BT_WAKE_MSM, GPIO_BT_FUN_0, GPIO_CFG_INPUT , GPIO_CFG_NO_PULL ,  GPIO_CFG_2MA ),	
        "BT_WAKE_MSM"  }
};

static struct msm_gpio bt_config_power_control[] = {  
    /*following 2 are bt on/off control*/
    { GPIO_CFG(GPIO_BT_SHUTDOWN_N, GPIO_BT_FUN_0, GPIO_CFG_OUTPUT , GPIO_CFG_NO_PULL ,  GPIO_CFG_2MA ), 
        "BT_REG_ON"  },
    { GPIO_CFG(GPIO_BT_RESET_N, GPIO_BT_FUN_0, GPIO_CFG_OUTPUT , GPIO_CFG_NO_PULL ,  GPIO_CFG_2MA ), 
        "BT_PWR_ON"  }
};

static struct msm_gpio bt_config_bcm4329_power_off[] = {
    { GPIO_CFG(GPIO_BT_UART_RTS, GPIO_BT_FUN_0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,   GPIO_CFG_2MA),
        "UART1DM_RFR" },
    { GPIO_CFG(GPIO_BT_UART_CTS, GPIO_BT_FUN_0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,   GPIO_CFG_2MA),
        "UART1DM_CTS" },
    { GPIO_CFG(GPIO_BT_RX, GPIO_BT_FUN_0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,   GPIO_CFG_2MA),
        "UART1DM_Rx" },
    { GPIO_CFG(GPIO_BT_TX, GPIO_BT_FUN_0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,   GPIO_CFG_2MA),
        "UART1DM_Tx" },
    /*following 2 are the wakeup between 4329 and MSM*/
    { GPIO_CFG(GPIO_BT_WAKE_BT, GPIO_BT_FUN_0, GPIO_CFG_INPUT , GPIO_CFG_PULL_DOWN ,  GPIO_CFG_2MA ),
        "MSM_WAKE_BT"  },
    { GPIO_CFG(GPIO_BT_WAKE_MSM, GPIO_BT_FUN_0, GPIO_CFG_INPUT , GPIO_CFG_PULL_DOWN ,  GPIO_CFG_2MA ),	
        "BT_WAKE_MSM"  }
 };

//xh end


/* configure all bt power here! */
static const char *vregs_bt_bcm4329_name[] = {
    "s3"
};

static struct vreg *vregs_bt_bcm4329[ARRAY_SIZE(vregs_bt_bcm4329_name)];

/* put power on for bt*/
static int bluetooth_bcm4329_power_regulators(int on)
{
    int i = 0;
    int rc = 0;

    for (i = 0; i < ARRAY_SIZE(vregs_bt_bcm4329_name); i++) {
        rc = on ? vreg_enable(vregs_bt_bcm4329[i]) :
            vreg_disable(vregs_bt_bcm4329[i]);
        if (rc < 0) {
        printk(KERN_ERR "%s: vreg %s %s failed (%d)\n",
            __func__, vregs_bt_bcm4329_name[i],
    			       on ? "enable" : "disable", rc);
        return -EIO;
        }
    }

    /*gpio power for bcm4329*/
    if(on)
    {

        rc = gpio_direction_output(GPIO_BT_SHUTDOWN_N, GPIO_BT_ON);  /*bt_reg_on off on :161 -->1*/
        if (rc) 
        {
            printk(KERN_ERR "%s:  bt power1 on fail (%d)\n",
                   __func__, rc);
            return -EIO;
        }
          
        mdelay(1);
        rc = gpio_direction_output(GPIO_BT_RESET_N, GPIO_BT_ON);  /*bt_pwr_on  on:163 -->1*/
        if (rc) 
        {
            printk(KERN_ERR "%s:  bt power2 off fail (%d)\n",
                   __func__, rc);
            return -EIO;
        }


    }
    else
    {
        rc = gpio_direction_output(GPIO_BT_RESET_N, GPIO_BT_OFF);  /*bt_pwr_on off:163 -->0*/
        if (rc) 
        {
            printk(KERN_ERR "%s:  bt power2 off fail (%d)\n",
                   __func__, rc);
            return -EIO;
        }
        mdelay(1);

        rc = gpio_direction_output(GPIO_BT_SHUTDOWN_N, GPIO_BT_OFF);  /*bt_reg_on :161 -->0*/
        if (rc) 
        {
            printk(KERN_ERR "%s:  bt power1 off fail (%d)\n",
                   __func__, rc);
            return -EIO;
        }
        mdelay(1);

    }		
    /*< DTS2011021001070 xuhui 20110210 begin */
    //delay 50 micseconds for BCM4329 power 
    mdelay(BCM4329_POWER_DELAY);
    /* DTS2011021001070 xuhui 20110210 end >*/
    return 0;
}


	

static int bluetooth_bcm4329_power(int on)
{
    int rc = 0;

   // const char *id = "BTPW";

    if (on)
    {
        rc = msm_gpios_enable(bt_config_bcm4329_power_on,
            ARRAY_SIZE(bt_config_bcm4329_power_on));
        if (rc < 0)
        {
            printk(KERN_ERR "%s: bcm4329_config_gpio on failed (%d)\n",
                __func__, rc);
            return rc;
        }


        /*step1: pimc: vote the power level */
        /*   now use s3 
        rc = pmapp_vreg_level_vote(id, PMAPP_VREG_S2, 1300);
        if (rc < 0) 
        {
            printk(KERN_ERR "%s: vreg level on failed (%d)\n",
            __func__, rc);
        return rc;
        }
        */
    
        /* pimc: put the power on */
        rc = bluetooth_bcm4329_power_regulators(on);
        if (rc < 0) 
        {
            printk(KERN_ERR "%s: bcm4329_power_regulators on failed (%d)\n",
                __func__, rc);
            return rc;
        }
        /*step2: pimc: get the work clk for bt*/
        /*  not share clock
        rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_DO,
            PMAPP_CLOCK_VOTE_ON);
        if (rc < 0)
        return -EIO;
        */
		
        /*step3:  check bt version, and init the corresponding chip */
       /* rc = bcm4329_bt(on);
        if (rc < 0)
        return -EIO;
                */

        /*step4: msm: config the msm  bt gpios*/
      /*  rc = msm_gpios_enable(bt_config_bcm4329_power_on,
            ARRAY_SIZE(bt_config_bcm4329_power_on));
        if (rc < 0)
        {
            printk(KERN_ERR "%s: bcm4329_config_gpio on failed (%d)\n",
                __func__, rc);
            return rc;
        }
        */

    }
    else
    {
        /*step5: msm: config the msm  bt gpios*/
        rc = msm_gpios_enable(bt_config_bcm4329_power_off,
            ARRAY_SIZE(bt_config_bcm4329_power_off));
        if (rc < 0)
        {
            printk(KERN_ERR "%s: bcm4329_config_gpio on failed (%d)\n",
                __func__, rc);
            return rc;
        }

        /* check for initial rfkill block (power off) */
        if (platform_get_drvdata(&msm_bt_power_device) == NULL)
        {
            printk(KERN_DEBUG "bluetooth rfkill block error : \n");
            goto out;
        }
      
        //deal with chipversion here

        /*step2: pimc: get the work clk off  for bt*/
        /*		
        rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_DO,
            PMAPP_CLOCK_VOTE_OFF);
        if (rc < 0)
        return -EIO;
        */
	
        /*step1: pimc: put the power on */

        rc = bluetooth_bcm4329_power_regulators(on);
        if (rc < 0) 
        {
            printk(KERN_ERR "%s: bcm4329_power_regulators off failed (%d)\n",
                __func__, rc);
            return rc;
        }
       
        /*
        rc = pmapp_vreg_level_vote(id, PMAPP_VREG_S2, 0);
        if (rc < 0) 
        {
            printk(KERN_INFO "%s: vreg level off failed (%d)\n",
                            __func__, rc);
        }
        */


    }	
out:
        printk(KERN_DEBUG "Bluetooth BCM4329 power switch: %d\n", on);

        return 0;
}


	
static void __init bt_bcm4329_power_init(void)
{
    /*step1: here will check the power, */
    int i = 0;
    int rc = -1;

    for (i = 0; i < ARRAY_SIZE(vregs_bt_bcm4329_name); i++)
    {
        vregs_bt_bcm4329[i] = vreg_get(NULL, vregs_bt_bcm4329_name[i]);
        if (IS_ERR(vregs_bt_bcm4329[i])) 
        {
            printk(KERN_ERR "%s: vreg get %s failed (%ld)\n",
                __func__, vregs_bt_bcm4329_name[i],
                PTR_ERR(vregs_bt_bcm4329[i]));
        return;
        }
    }
   
    //handle bt power control: becareful
    rc = msm_gpios_request_enable(bt_config_power_control,
                            ARRAY_SIZE(bt_config_power_control));
    if (rc < 0) {
            printk(KERN_ERR
                    "%s: bt power control request_enable failed (%d)\n",
                            __func__, rc);
            return;
    }
    
    rc = gpio_direction_output(GPIO_BT_RESET_N, GPIO_BT_OFF);  /*bt_pwr_on off:163 -->0*/
    if (rc) 
    {
        printk(KERN_ERR "%s:  bt power2 off fail (%d)\n",
               __func__, rc);
        return ;
    }
    mdelay(1);

    rc = gpio_direction_output(GPIO_BT_SHUTDOWN_N, GPIO_BT_OFF);  /*bt_reg_on :161 -->0*/
    if (rc) 
    {
        printk(KERN_ERR "%s:  bt power1 off fail (%d)\n",
               __func__, rc);
        return ;
    }
    mdelay(1);










    /*step2: config platform_data*/
    msm_bt_power_device.dev.platform_data = &bluetooth_bcm4329_power;
	
}

static struct resource bluesleep_resources[] = {
    {
    .name	= "gpio_host_wake",
    .start	= GPIO_BT_WAKE_MSM,
    .end	= GPIO_BT_WAKE_MSM,
    .flags	= IORESOURCE_IO,
    },
    {
    .name	= "gpio_ext_wake",
    .start	= GPIO_BT_WAKE_BT,
    .end	= GPIO_BT_WAKE_BT,
    .flags	= IORESOURCE_IO,
    },
    {
    .name	= "host_wake",
    .start	= MSM_GPIO_TO_INT(GPIO_BT_WAKE_MSM),
    .end	= MSM_GPIO_TO_INT(GPIO_BT_WAKE_MSM),
    .flags	= IORESOURCE_IRQ,
    },
};

static struct platform_device msm_bluesleep_device = {
    .name = "bluesleep",
    .id		= -1,
    .num_resources	= ARRAY_SIZE(bluesleep_resources),
    .resource	= bluesleep_resources,
};
#else
#define bt_bcm4329_power_init(x) do {} while (0)
#endif
/* DTS2011012004291 xuhui 20110120 end >*/
/*<BU5D08118 zhangtao 20100419 begin*/

/*when msm7x30 start the i2c pull up power is not config *
*set the gp13 right pull up*/

#ifdef CONFIG_HUAWEI_KERNEL
	static void __init i2c_power_init(void)
	{
		struct vreg *vreg_gp13=NULL;
		int rc;
	
		vreg_gp13 = vreg_get(NULL, VREG_GP13_NAME);
		rc = vreg_set_level(vreg_gp13, VREG_GP13_VOLTAGE_VALUE);
		if (rc) {
		printk("%s: vreg_gp13  vreg_set_level failed \n", __func__);
	}
		rc = vreg_enable(vreg_gp13);
		if (rc) {
		pr_err("%s: vreg_gp13    vreg_enable failed \n", __func__);
	}
		mdelay(5);
	
	}
#endif
/*BU5D08118 zhangtao 20100419 end>*/
static struct msm_psy_batt_pdata msm_psy_batt_data = {
	.voltage_min_design 	= 2800,
	.voltage_max_design	= 4300,
	.avail_chg_sources   	= AC_CHG | USB_CHG ,
	.batt_technology        = POWER_SUPPLY_TECHNOLOGY_LION,
};

static struct platform_device msm_batt_device = {
	.name 		    = "msm-battery",
	.id		    = -1,
	.dev.platform_data  = &msm_psy_batt_data,
};

static char *msm_adc_fluid_device_names[] = {
	"LTC_ADC1",
	"LTC_ADC2",
	"LTC_ADC3",
};

static char *msm_adc_surf_device_names[] = {
	"XO_ADC",
};

static struct msm_adc_platform_data msm_adc_pdata;

static struct platform_device msm_adc_device = {
	.name   = "msm_adc",
	.id = -1,
	.dev = {
		.platform_data = &msm_adc_pdata,
	},
};

#ifdef CONFIG_MSM_SDIO_AL
static struct msm_gpio mdm2ap_status = {
	GPIO_CFG(77, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	"mdm2ap_status"
};


static int configure_mdm2ap_status(int on)
{
	if (on)
		return msm_gpios_request_enable(&mdm2ap_status, 1);
	else {
		msm_gpios_disable_free(&mdm2ap_status, 1);
		return 0;
	}
}

static int get_mdm2ap_status(void)
{
	return gpio_get_value(GPIO_PIN(mdm2ap_status.gpio_cfg));
}

static struct sdio_al_platform_data sdio_al_pdata = {
	.config_mdm2ap_status = configure_mdm2ap_status,
	.get_mdm2ap_status = get_mdm2ap_status,
	.allow_sdioc_version_major_2 = 1,
	.peer_sdioc_version_minor = 0x0001,
	.peer_sdioc_version_major = 0x0003,
	.peer_sdioc_boot_version_minor = 0x0001,
	.peer_sdioc_boot_version_major = 0x0002,
};

struct platform_device msm_device_sdio_al = {
	.name = "msm_sdio_al",
	.id = -1,
	.dev		= {
		.platform_data	= &sdio_al_pdata,
	},
};

#endif /* CONFIG_MSM_SDIO_AL */

static struct platform_device *devices[] __initdata = {
#if defined(CONFIG_SERIAL_MSM) || defined(CONFIG_MSM_SERIAL_DEBUGGER)
	&msm_device_uart2,
#endif
	&msm_device_smd,
	&msm_device_dmov,
	/* <DTS2010072202961 hufeng 20100722 begin */
	/* removed several lines */
	/* DTS2010072202961 hufeng 20100722 end> */
#ifdef CONFIG_USB_FUNCTION
	&msm_device_hsusb_peripheral,
	&mass_storage_device,
#endif
#ifdef CONFIG_USB_MSM_OTG_72K
	&msm_device_otg,
#ifdef CONFIG_USB_GADGET
	&msm_device_gadget_peripheral,
#endif
#endif
#ifdef CONFIG_USB_ANDROID
	&usb_mass_storage_device,
	&rndis_device,
#ifdef CONFIG_USB_ANDROID_DIAG
	&usb_diag_device,
#endif
	&android_usb_device,
#endif
	&qsd_device_spi,
#ifdef CONFIG_I2C_SSBI
	&msm_device_ssbi6,
	&msm_device_ssbi7,
#endif
	&android_pmem_device,
	&msm_fb_device,
	&msm_migrate_pages_device,
	/* <DTS2010072202961 hufeng 20100722 begin */
	/* removed several lines */
	/* DTS2010072202961 hufeng 20100722 end> */
#ifdef CONFIG_MSM_ROTATOR
	&msm_rotator_device,
#endif
	/* <DTS2010072202961 hufeng 20100722 begin */
	/* removed several lines */
	/* DTS2010072202961 hufeng 20100722 end> */
	&android_pmem_kernel_ebi1_device,
	&android_pmem_adsp_device,
	&android_pmem_audio_device,
	&msm_device_i2c,
	&msm_device_i2c_2,
	&msm_device_uart_dm1,
	&hs_device,
#ifdef CONFIG_MSM7KV2_AUDIO
	&msm_aictl_device,
	&msm_mi2s_device,
	&msm_lpa_device,
	&msm_aux_pcm_device,
#endif
	&msm_device_adspdec,
	&qup_device_i2c,

/*< DTS2011012004291 xuhui 20110120 begin */
#ifdef CONFIG_HUAWEI_KERNEL
#if defined(CONFIG_MARIMBA_CORE) && \
   (defined(CONFIG_MSM_BT_POWER) || defined(CONFIG_MSM_BT_POWER_MODULE))
	&msm_bt_power_device,
#endif
#else
//we use bluesleep device as 7200 7x25
    &msm_bt_power_device,
    &msm_bluesleep_device,	

#endif
/* DTS2011012004291 xuhui 20110120 end >*/

	&msm_device_kgsl,
#ifdef CONFIG_MT9T013
	&msm_camera_sensor_mt9t013,
#endif
#ifdef CONFIG_MT9D112
	&msm_camera_sensor_mt9d112,
#endif

/*< DTS2011051400258   songxiaoming 20110514 begin */
#ifdef CONFIG_HUAWEI_SENSOR_HI701	
    &msm_camera_sensor_hi701,
#endif 
/* DTS2011051400258   songxiaoming 20110514 end > */
/*<  DTS2011101000840   yuguangcai 20111010 begin */
#ifdef CONFIG_HUAWEI_SENSOR_S5K5CA
	&msm_camera_sensor_s5k5ca,
#endif
/* DTS2011101000840   yuguangcai 20111010 end > */
/*< DTS2011102805854 yuguangcai 20111029 begin */
#ifdef CONFIG_HUAWEI_SENSOR_MT9T113
	&msm_camera_sensor_mt9t113,
#endif
/* DTS2011102805854 yuguangcai 20111029 end > */
/*< DTS2011072600848   songxiaoming 20110726 begin */
#ifdef CONFIG_HUAWEI_SENSOR_OV7736	
    &msm_camera_sensor_ov7736,
#endif 
/* DTS2011072600848   songxiaoming 20110726 end > */

/*<BU5D08108, lijuan 00152865, 20100419 begin*/
#ifdef CONFIG_HUAWEI_SENSOR_OV7690
	&msm_camera_sensor_ov7690,
#endif
/* BU5D08108, lijuan 00152865, 20100419 end> */


/*<BU5D10275  liyuping  20100529 begin*/
#ifdef CONFIG_HUAWEI_SENSOR_HIMAX0356	
    &msm_camera_sensor_himax0356,
#endif 
/*BU5D10275  liyuping 20100529 end> */

/*<DTS2011042704563 penghai 20110427 begin*/
#ifdef CONFIG_HUAWEI_SENSOR_MT9D113	
    &msm_camera_sensor_mt9d113,
#endif 
/*DTS2011042704563 penghai 20110427 end>*/

/* < DTS2011061801881 zhangyu 20110620 BEGIN */
#ifdef CONFIG_HUAWEI_SENSOR_MT9V114	
    &msm_camera_sensor_mt9v114,
#endif 
/* DTS2011061801881 zhangyu 20110620 END > */ 

/*<BU5D08116, lijuan 00152865, 20100419 begin*/
#ifdef CONFIG_HUAWEI_SENSOR_S5K4E1GX
	&msm_camera_sensor_s5k4e1gx,
#endif
/* < DTS2011090701903 zhangyu 20110907 begin */
/* < DTS2011100802746 zhangyu 20111010 begin */
#ifdef CONFIG_HUAWEI_SENSOR_S5K4E1GX_P
	&msm_camera_sensor_s5k4e1gx_p,
#endif
/* DTS2011100802746 zhangyu 20111010 end > */ 
#ifdef CONFIG_HUAWEI_SENSOR_MT9P017
	&msm_camera_sensor_mt9p017,
#endif
/* DTS2011090701903 zhangyu 20110907 end > */ 
/* BU5D08116, lijuan 00152865, 20100419 end> */
/*<BU5D09497  lijuan 00152865  20100514 begin*/
#ifdef CONFIG_HUAWEI_SENSOR_OV5647_SUNNY
	&msm_camera_sensor_ov5647_sunny,
#endif
/* BU5D09497  lijuan 00152865  20100514 end> */

/*<DTS2011042704563 penghai 20110427 begin*/
#ifdef CONFIG_HUAWEI_SENSOR_MT9E013
	&msm_camera_sensor_mt9e013,
#endif
/*DTS2011042704563 penghai 20110427 end>*/

#ifdef CONFIG_S5K3E2FX
	&msm_camera_sensor_s5k3e2fx,
#endif
#ifdef CONFIG_MT9P012
	&msm_camera_sensor_mt9p012,
/*<DTS2011042704563 penghai 20110427 begin*/
/*deletes some lines*/
/*DTS2011042704563 penghai 20110427 end>*/
#endif
#ifdef CONFIG_VX6953
	&msm_camera_sensor_vx6953,
#endif
#ifdef CONFIG_SN12M0PZ
	&msm_camera_sensor_sn12m0pz,
#endif
	&msm_device_vidc_720p,
#ifdef CONFIG_MSM_GEMINI
	&msm_gemini_device,
#endif
#ifdef CONFIG_MSM_VPE
	&msm_vpe_device,
#endif
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)
	&msm_device_tsif,
#endif
#ifdef CONFIG_MSM_SDIO_AL
	&msm_device_sdio_al,
#endif

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE)
	&qcrypto_device,
#endif

#if defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)
	&qcedev_device,
#endif
/*< DTS2011092601370 zhongjinrong 20110926 begin */
#ifdef CONFIG_HUAWEI_KEYBOARD_LEDS
	&msm_device_pmic_keyboard_leds,
#endif
/* DTS2011092601370 zhongjinrong 20119026 end >*/	

	&msm_batt_device,
	&msm_adc_device,
	&msm_ebi0_thermal,
	&msm_ebi1_thermal,
/*<BU5D08276 sibingsong 20100416 begin*/
#ifdef CONFIG_HUAWEI_FEATURE_RGB_KEY_LIGHT
	&rgb_leds_device,
#endif
/*BU5D08276 sibingsong 20100416 end>*/
/* < DTS2010102600914 zhangtao 20101026 begin */
#ifdef CONFIG_HUAWEI_LEDS_PMIC
    &msm_device_pmic_leds,
#endif
/* DTS2010102600914 zhangtao 20101026 end > */
/* < DTS2010111603970 zhangtao 20101116 begin */
#ifdef CONFIG_HUAWEI_FEATURE_PTT_KEY_LIGHT
/*init the light*/
    &ptt_led_driver,
#endif
/* DTS2010111603970 zhangtao 20101116 end > */

    /* <DTS2011032104626 shenjinming 20110321 begin */
    &huawei_device_detect,
    /* <DTS2011032104626 shenjinming 20110321 end> */

};

static struct msm_gpio msm_i2c_gpios_hw[] = {
	{ GPIO_CFG(70, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "i2c_scl" },
	{ GPIO_CFG(71, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "i2c_sda" },
};

static struct msm_gpio msm_i2c_gpios_io[] = {
	{ GPIO_CFG(70, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "i2c_scl" },
	{ GPIO_CFG(71, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "i2c_sda" },
};

static struct msm_gpio qup_i2c_gpios_io[] = {
	{ GPIO_CFG(16, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "qup_scl" },
	{ GPIO_CFG(17, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "qup_sda" },
};
static struct msm_gpio qup_i2c_gpios_hw[] = {
	{ GPIO_CFG(16, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "qup_scl" },
	{ GPIO_CFG(17, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "qup_sda" },
};

static void
msm_i2c_gpio_config(int adap_id, int config_type)
{
	struct msm_gpio *msm_i2c_table;

	/* Each adapter gets 2 lines from the table */
	if (adap_id > 0)
		return;
	if (config_type)
		msm_i2c_table = &msm_i2c_gpios_hw[adap_id*2];
	else
		msm_i2c_table = &msm_i2c_gpios_io[adap_id*2];
	msm_gpios_enable(msm_i2c_table, 2);
}
/*This needs to be enabled only for OEMS*/
#ifndef CONFIG_QUP_EXCLUSIVE_TO_CAMERA
static struct vreg *qup_vreg;
#endif
static void
qup_i2c_gpio_config(int adap_id, int config_type)
{
	int rc = 0;
	struct msm_gpio *qup_i2c_table;
	/* Each adapter gets 2 lines from the table */
	if (adap_id != 4)
		return;
	if (config_type)
		qup_i2c_table = qup_i2c_gpios_hw;
	else
		qup_i2c_table = qup_i2c_gpios_io;
	rc = msm_gpios_enable(qup_i2c_table, 2);
	if (rc < 0)
		printk(KERN_ERR "QUP GPIO enable failed: %d\n", rc);
	/*This needs to be enabled only for OEMS*/
#ifndef CONFIG_QUP_EXCLUSIVE_TO_CAMERA
	if (qup_vreg) {
		int rc = vreg_set_level(qup_vreg, 1800);
		if (rc) {
			pr_err("%s: vreg LVS1 set level failed (%d)\n",
			__func__, rc);
		}
		rc = vreg_enable(qup_vreg);
		if (rc) {
			pr_err("%s: vreg_enable() = %d \n",
			__func__, rc);
		}
	}
#endif
}

static struct msm_i2c_platform_data msm_i2c_pdata = {
	.clk_freq = 100000,
	.pri_clk = 70,
	.pri_dat = 71,
	.rmutex  = 1,
	.rsl_id = "D:I2C02000021",
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_init(void)
{
	if (msm_gpios_request(msm_i2c_gpios_hw, ARRAY_SIZE(msm_i2c_gpios_hw)))
		pr_err("failed to request I2C gpios\n");

	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
	
	 /*BU5D07679 zhangtao 20100413 begin*/
	#ifdef CONFIG_HUAWEI_KERNEL
	i2c_power_init();
	#endif
	/*BU5D07679 zhangtao 20100413 end*/	
}

static struct msm_i2c_platform_data msm_i2c_2_pdata = {
	.clk_freq = 100000,
	.rmutex  = 1,
	.rsl_id = "D:I2C02000022",
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_2_init(void)
{
	msm_device_i2c_2.dev.platform_data = &msm_i2c_2_pdata;
}

/* < DTS2011061801881 zhangyu 20110620 BEGIN */
static struct msm_i2c_platform_data qup_i2c_pdata = {
	.clk_freq = 100000,
	.pclk = "camif_pad_pclk",
	.msm_i2c_config_gpio = qup_i2c_gpio_config,
};
/* DTS2011061801881 zhangyu 20110620 END > */ 

static void __init qup_device_i2c_init(void)
{
	if (msm_gpios_request(qup_i2c_gpios_hw, ARRAY_SIZE(qup_i2c_gpios_hw)))
		pr_err("failed to request I2C gpios\n");

	qup_device_i2c.dev.platform_data = &qup_i2c_pdata;
	/*This needs to be enabled only for OEMS*/
#ifndef CONFIG_QUP_EXCLUSIVE_TO_CAMERA
	qup_vreg = vreg_get(NULL, "lvsw1");
	if (IS_ERR(qup_vreg)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(qup_vreg));
	}
#endif
}

#ifdef CONFIG_I2C_SSBI
static struct msm_ssbi_platform_data msm_i2c_ssbi6_pdata = {
	.rsl_id = "D:PMIC_SSBI",
	.controller_type = MSM_SBI_CTRL_SSBI2,
};

static struct msm_ssbi_platform_data msm_i2c_ssbi7_pdata = {
	.rsl_id = "D:CODEC_SSBI",
	.controller_type = MSM_SBI_CTRL_SSBI,
};
#endif

static struct msm_acpu_clock_platform_data msm7x30_clock_data = {
	.acpu_switch_time_us = 50,
	.vdd_switch_time_us = 62,
};

static void __init msm7x30_init_irq(void)
{
	msm_init_irq();
}

/* <DTS2010072202961 hufeng 20100722 begin */
/* removed several lines */
/* DTS2010072202961 hufeng 20100722 end> */

struct vreg *vreg_s3;
struct vreg *vreg_mmc;

#if (defined(CONFIG_MMC_MSM_SDC1_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC2_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC3_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC4_SUPPORT))

struct sdcc_gpio {
	struct msm_gpio *cfg_data;
	uint32_t size;
	struct msm_gpio *sleep_cfg_data;
};
#if defined(CONFIG_MMC_MSM_SDC1_SUPPORT)
static struct msm_gpio sdc1_lvlshft_cfg_data[] = {
	{GPIO_CFG(35, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA), "sdc1_lvlshft"},
};
#endif
static struct msm_gpio sdc1_cfg_data[] = {
	{GPIO_CFG(38, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "sdc1_clk"},
	{GPIO_CFG(39, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_cmd"},
	{GPIO_CFG(40, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_3"},
	{GPIO_CFG(41, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_2"},
	{GPIO_CFG(42, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_1"},
	{GPIO_CFG(43, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_0"},
};

static struct msm_gpio sdc2_cfg_data[] = {
	{GPIO_CFG(64, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "sdc2_clk"},
	{GPIO_CFG(65, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_cmd"},
	{GPIO_CFG(66, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_3"},
	{GPIO_CFG(67, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_2"},
	{GPIO_CFG(68, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_1"},
	{GPIO_CFG(69, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_0"},

#ifdef CONFIG_MMC_MSM_SDC2_8_BIT_SUPPORT
	{GPIO_CFG(115, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_4"},
	{GPIO_CFG(114, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_5"},
	{GPIO_CFG(113, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_6"},
	{GPIO_CFG(112, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_7"},
#endif
};

static struct msm_gpio sdc3_cfg_data[] = {
	{GPIO_CFG(110, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "sdc3_clk"},
	{GPIO_CFG(111, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_cmd"},
	{GPIO_CFG(116, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_3"},
	{GPIO_CFG(117, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_2"},
	{GPIO_CFG(118, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_1"},
	{GPIO_CFG(119, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_0"},
};

static struct msm_gpio sdc3_sleep_cfg_data[] = {
	{GPIO_CFG(110, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc3_clk"},
	{GPIO_CFG(111, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc3_cmd"},
	{GPIO_CFG(116, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc3_dat_3"},
	{GPIO_CFG(117, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc3_dat_2"},
	{GPIO_CFG(118, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc3_dat_1"},
	{GPIO_CFG(119, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc3_dat_0"},
};

static struct msm_gpio sdc4_cfg_data[] = {
	{GPIO_CFG(58, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "sdc4_clk"},
	{GPIO_CFG(59, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc4_cmd"},
	{GPIO_CFG(60, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc4_dat_3"},
	{GPIO_CFG(61, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc4_dat_2"},
	{GPIO_CFG(62, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc4_dat_1"},
	{GPIO_CFG(63, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc4_dat_0"},
};

static struct sdcc_gpio sdcc_cfg_data[] = {
	{
		.cfg_data = sdc1_cfg_data,
		.size = ARRAY_SIZE(sdc1_cfg_data),
		.sleep_cfg_data = NULL,
	},
	{
		.cfg_data = sdc2_cfg_data,
		.size = ARRAY_SIZE(sdc2_cfg_data),
		.sleep_cfg_data = NULL,
	},
	{
		.cfg_data = sdc3_cfg_data,
		.size = ARRAY_SIZE(sdc3_cfg_data),
		.sleep_cfg_data = sdc3_sleep_cfg_data,
	},
	{
		.cfg_data = sdc4_cfg_data,
		.size = ARRAY_SIZE(sdc4_cfg_data),
		.sleep_cfg_data = NULL,
	},
};

struct sdcc_vreg {
	struct vreg *vreg_data;
	unsigned level;
};

static struct sdcc_vreg sdcc_vreg_data[4];

static unsigned long vreg_sts, gpio_sts;

static uint32_t msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct sdcc_gpio *curr;

	curr = &sdcc_cfg_data[dev_id - 1];

	if (!(test_bit(dev_id, &gpio_sts)^enable))
		return rc;

	if (enable) {
		set_bit(dev_id, &gpio_sts);
		rc = msm_gpios_request_enable(curr->cfg_data, curr->size);
		if (rc)
			printk(KERN_ERR "%s: Failed to turn on GPIOs for slot %d\n",
				__func__,  dev_id);
	} else {
		clear_bit(dev_id, &gpio_sts);
		if (curr->sleep_cfg_data) {
			msm_gpios_enable(curr->sleep_cfg_data, curr->size);
			msm_gpios_free(curr->sleep_cfg_data, curr->size);
		} else {
			msm_gpios_disable_free(curr->cfg_data, curr->size);
		}
	}

	return rc;
}

static uint32_t msm_sdcc_setup_vreg(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct sdcc_vreg *curr;
	static int enabled_once[] = {0, 0, 0, 0};

	curr = &sdcc_vreg_data[dev_id - 1];

	if (!(test_bit(dev_id, &vreg_sts)^enable))
		return rc;

	if (!enable || enabled_once[dev_id - 1])
		return 0;

	if (enable) {
		set_bit(dev_id, &vreg_sts);
		rc = vreg_set_level(curr->vreg_data, curr->level);
		if (rc) {
			printk(KERN_ERR "%s: vreg_set_level() = %d \n",
					__func__, rc);
		}
		rc = vreg_enable(curr->vreg_data);
		if (rc) {
			printk(KERN_ERR "%s: vreg_enable() = %d \n",
					__func__, rc);
		}
		enabled_once[dev_id - 1] = 1;
	} else {
		clear_bit(dev_id, &vreg_sts);
		rc = vreg_disable(curr->vreg_data);
		if (rc) {
			printk(KERN_ERR "%s: vreg_disable() = %d \n",
					__func__, rc);
		}
	}
	return rc;
}

static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	int rc = 0;
	struct platform_device *pdev;

	pdev = container_of(dv, struct platform_device, dev);
	rc = msm_sdcc_setup_gpio(pdev->id, (vdd ? 1 : 0));
	if (rc)
		goto out;

	if (pdev->id == 4) /* S3 is always ON and cannot be disabled */
		rc = msm_sdcc_setup_vreg(pdev->id, (vdd ? 1 : 0));
out:
	return rc;
}

#if defined(CONFIG_MMC_MSM_SDC1_SUPPORT) && \
	defined(CONFIG_CSDIO_VENDOR_ID) && \
	defined(CONFIG_CSDIO_DEVICE_ID) && \
	(CONFIG_CSDIO_VENDOR_ID == 0x70 && CONFIG_CSDIO_DEVICE_ID == 0x1117)

#define MBP_ON  1
#define MBP_OFF 0

#define MBP_RESET_N \
	GPIO_CFG(44, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA)
#define MBP_INT0 \
	GPIO_CFG(46, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA)

#define MBP_MODE_CTRL_0 \
	GPIO_CFG(35, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)
#define MBP_MODE_CTRL_1 \
	GPIO_CFG(36, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)
#define MBP_MODE_CTRL_2 \
	GPIO_CFG(34, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)
#define TSIF_EN \
	GPIO_CFG(35, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN,	GPIO_CFG_2MA)
#define TSIF_DATA \
	GPIO_CFG(36, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN,	GPIO_CFG_2MA)
#define TSIF_CLK \
	GPIO_CFG(34, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)

static struct msm_gpio mbp_cfg_data[] = {
	{GPIO_CFG(44, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
		"mbp_reset"},
	{GPIO_CFG(85, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
		"mbp_io_voltage"},
};

static int mbp_config_gpios_pre_init(int enable)
{
	int rc = 0;

	if (enable) {
		rc = msm_gpios_request_enable(mbp_cfg_data,
			ARRAY_SIZE(mbp_cfg_data));
		if (rc) {
			printk(KERN_ERR
				"%s: Failed to turnon GPIOs for mbp chip(%d)\n",
				__func__, rc);
		}
	} else
		msm_gpios_disable_free(mbp_cfg_data, ARRAY_SIZE(mbp_cfg_data));
	return rc;
}

static int mbp_setup_rf_vregs(int state)
{
	struct vreg *vreg_rf = NULL;
	struct vreg *vreg_rf_switch	= NULL;
	int rc;

	vreg_rf = vreg_get(NULL, "s2");
	if (IS_ERR(vreg_rf)) {
		pr_err("%s: s2 vreg get failed (%ld)",
				__func__, PTR_ERR(vreg_rf));
		return -EFAULT;
	}
	vreg_rf_switch = vreg_get(NULL, "rf");
	if (IS_ERR(vreg_rf_switch)) {
		pr_err("%s: rf vreg get failed (%ld)",
				__func__, PTR_ERR(vreg_rf_switch));
		return -EFAULT;
	}

	if (state) {
		rc = vreg_set_level(vreg_rf, 1300);
		if (rc) {
			pr_err("%s: vreg s2 set level failed (%d)\n",
					__func__, rc);
			return rc;
		}

		rc = vreg_enable(vreg_rf);
		if (rc) {
			printk(KERN_ERR "%s: vreg_enable(s2) = %d\n",
					__func__, rc);
		}

		rc = vreg_set_level(vreg_rf_switch, 2600);
		if (rc) {
			pr_err("%s: vreg rf switch set level failed (%d)\n",
					__func__, rc);
			return rc;
		}
		rc = vreg_enable(vreg_rf_switch);
		if (rc) {
			printk(KERN_ERR "%s: vreg_enable(rf) = %d\n",
					__func__, rc);
		}
	} else {
		(void) vreg_disable(vreg_rf);
		(void) vreg_disable(vreg_rf_switch);
	}
	return 0;
}

static int mbp_setup_vregs(int state)
{
	struct vreg *vreg_analog = NULL;
	struct vreg *vreg_io = NULL;
	int rc;

	vreg_analog = vreg_get(NULL, "gp4");
	if (IS_ERR(vreg_analog)) {
		pr_err("%s: gp4 vreg get failed (%ld)",
				__func__, PTR_ERR(vreg_analog));
		return -EFAULT;
	}
	vreg_io = vreg_get(NULL, "s3");
	if (IS_ERR(vreg_io)) {
		pr_err("%s: s3 vreg get failed (%ld)",
				__func__, PTR_ERR(vreg_io));
		return -EFAULT;
	}
	if (state) {
		rc = vreg_set_level(vreg_analog, 2600);
		if (rc) {
			pr_err("%s: vreg_set_level failed (%d)",
					__func__, rc);
		}
		rc = vreg_enable(vreg_analog);
		if (rc) {
			pr_err("%s: analog vreg enable failed (%d)",
					__func__, rc);
		}
		rc = vreg_set_level(vreg_io, 1800);
		if (rc) {
			pr_err("%s: vreg_set_level failed (%d)",
					__func__, rc);
		}
		rc = vreg_enable(vreg_io);
		if (rc) {
			pr_err("%s: io vreg enable failed (%d)",
					__func__, rc);
		}
	} else {
		rc = vreg_disable(vreg_analog);
		if (rc) {
			pr_err("%s: analog vreg disable failed (%d)",
					__func__, rc);
		}
		rc = vreg_disable(vreg_io);
		if (rc) {
			pr_err("%s: io vreg disable failed (%d)",
					__func__, rc);
		}
	}
	return rc;
}

static int mbp_set_tcxo_en(int enable)
{
	int rc;
	const char *id = "UBMC";
	struct vreg *vreg_analog = NULL;

	rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_A1,
		enable ? PMAPP_CLOCK_VOTE_ON : PMAPP_CLOCK_VOTE_OFF);
	if (rc < 0) {
		printk(KERN_ERR "%s: unable to %svote for a1 clk\n",
			__func__, enable ? "" : "de-");
		return -EIO;
	}
	if (!enable) {
		vreg_analog = vreg_get(NULL, "gp4");
		if (IS_ERR(vreg_analog)) {
			pr_err("%s: gp4 vreg get failed (%ld)",
					__func__, PTR_ERR(vreg_analog));
			return -EFAULT;
		}

		(void) vreg_disable(vreg_analog);
	}
	return rc;
}

static void mbp_set_freeze_io(int state)
{
	if (state)
		gpio_set_value(85, 0);
	else
		gpio_set_value(85, 1);
}

static int mbp_set_core_voltage_en(int enable)
{
	int rc;
	struct vreg *vreg_core1p2 = NULL;

	vreg_core1p2 = vreg_get(NULL, "gp16");
	if (IS_ERR(vreg_core1p2)) {
		pr_err("%s: gp16 vreg get failed (%ld)",
				__func__, PTR_ERR(vreg_core1p2));
		return -EFAULT;
	}
	if (enable) {
		rc = vreg_set_level(vreg_core1p2, 1200);
		if (rc) {
			pr_err("%s: vreg_set_level failed (%d)",
					__func__, rc);
		}
		(void) vreg_enable(vreg_core1p2);

		return 80;
	} else {
		gpio_set_value(85, 1);
		return 0;
	}
	return rc;
}

static void mbp_set_reset(int state)
{
	if (state)
		gpio_set_value(GPIO_PIN(MBP_RESET_N), 0);
	else
		gpio_set_value(GPIO_PIN(MBP_RESET_N), 1);
}

static int mbp_config_interface_mode(int state)
{
	if (state) {
		gpio_tlmm_config(MBP_MODE_CTRL_0, GPIO_CFG_ENABLE);
		gpio_tlmm_config(MBP_MODE_CTRL_1, GPIO_CFG_ENABLE);
		gpio_tlmm_config(MBP_MODE_CTRL_2, GPIO_CFG_ENABLE);
		gpio_set_value(GPIO_PIN(MBP_MODE_CTRL_0), 0);
		gpio_set_value(GPIO_PIN(MBP_MODE_CTRL_1), 1);
		gpio_set_value(GPIO_PIN(MBP_MODE_CTRL_2), 0);
	} else {
		gpio_tlmm_config(MBP_MODE_CTRL_0, GPIO_CFG_DISABLE);
		gpio_tlmm_config(MBP_MODE_CTRL_1, GPIO_CFG_DISABLE);
		gpio_tlmm_config(MBP_MODE_CTRL_2, GPIO_CFG_DISABLE);
	}
	return 0;
}

static int mbp_setup_adc_vregs(int state)
{
	struct vreg *vreg_adc = NULL;
	int rc;

	vreg_adc = vreg_get(NULL, "s4");
	if (IS_ERR(vreg_adc)) {
		pr_err("%s: s4 vreg get failed (%ld)",
				__func__, PTR_ERR(vreg_adc));
		return -EFAULT;
	}
	if (state) {
		rc = vreg_set_level(vreg_adc, 2200);
		if (rc) {
			pr_err("%s: vreg_set_level failed (%d)",
					__func__, rc);
		}
		rc = vreg_enable(vreg_adc);
		if (rc) {
			pr_err("%s: enable vreg adc failed (%d)",
					__func__, rc);
		}
	} else {
		rc = vreg_disable(vreg_adc);
		if (rc) {
			pr_err("%s: disable vreg adc failed (%d)",
					__func__, rc);
		}
	}
	return rc;
}

static int mbp_power_up(void)
{
	int rc;

	rc = mbp_config_gpios_pre_init(MBP_ON);
	if (rc)
		goto exit;
	pr_debug("%s: mbp_config_gpios_pre_init() done\n", __func__);

	rc = mbp_setup_vregs(MBP_ON);
	if (rc)
		goto exit;
	pr_debug("%s: gp4 (2.6) and s3 (1.8) done\n", __func__);

	rc = mbp_set_tcxo_en(MBP_ON);
	if (rc)
		goto exit;
	pr_debug("%s: tcxo clock done\n", __func__);

	mbp_set_freeze_io(MBP_OFF);
	pr_debug("%s: set gpio 85 to 1 done\n", __func__);

	udelay(100);
	mbp_set_reset(MBP_ON);

	udelay(300);
	rc = mbp_config_interface_mode(MBP_ON);
	if (rc)
		goto exit;
	pr_debug("%s: mbp_config_interface_mode() done\n", __func__);

	udelay(100 + mbp_set_core_voltage_en(MBP_ON));
	pr_debug("%s: power gp16 1.2V done\n", __func__);

	mbp_set_freeze_io(MBP_ON);
	pr_debug("%s: set gpio 85 to 0 done\n", __func__);

	udelay(100);

	rc = mbp_setup_rf_vregs(MBP_ON);
	if (rc)
		goto exit;
	pr_debug("%s: s2 1.3V and rf 2.6V done\n", __func__);

	rc = mbp_setup_adc_vregs(MBP_ON);
	if (rc)
		goto exit;
	pr_debug("%s: s4 2.2V  done\n", __func__);

	udelay(200);

	mbp_set_reset(MBP_OFF);
	pr_debug("%s: close gpio 44 done\n", __func__);

	msleep(20);
exit:
	return rc;
}

static int mbp_power_down(void)
{
	int rc;
	struct vreg *vreg_adc = NULL;

	vreg_adc = vreg_get(NULL, "s4");
	if (IS_ERR(vreg_adc)) {
		pr_err("%s: s4 vreg get failed (%ld)",
				__func__, PTR_ERR(vreg_adc));
		return -EFAULT;
	}

	mbp_set_reset(MBP_ON);
	pr_debug("%s: mbp_set_reset(MBP_ON) done\n", __func__);

	udelay(100);

	rc = mbp_setup_adc_vregs(MBP_OFF);
	if (rc)
		goto exit;
	pr_debug("%s: vreg_disable(vreg_adc) done\n", __func__);

	udelay(5);

	rc = mbp_setup_rf_vregs(MBP_OFF);
	if (rc)
		goto exit;
	pr_debug("%s: mbp_setup_rf_vregs(MBP_OFF) done\n", __func__);

	udelay(5);

	mbp_set_freeze_io(MBP_OFF);
	pr_debug("%s: mbp_set_freeze_io(MBP_OFF) done\n", __func__);

	udelay(100);
	rc = mbp_set_core_voltage_en(MBP_OFF);
	if (rc)
		goto exit;
	pr_debug("%s: mbp_set_core_voltage_en(MBP_OFF) done\n", __func__);

	gpio_set_value(85, 1);

	rc = mbp_set_tcxo_en(MBP_OFF);
	if (rc)
		goto exit;
	pr_debug("%s: mbp_set_tcxo_en(MBP_OFF) done\n", __func__);

	rc = mbp_config_gpios_pre_init(MBP_OFF);
	if (rc)
		goto exit;
exit:
	return rc;
}

static void (*mbp_status_notify_cb)(int card_present, void *dev_id);
static void *mbp_status_notify_cb_devid;
static int mbp_power_status;
static int mbp_power_init_done;

static uint32_t mbp_setup_power(struct device *dv,
	unsigned int power_status)
{
	int rc = 0;
	struct platform_device *pdev;

	pdev = container_of(dv, struct platform_device, dev);

	if (power_status == mbp_power_status)
		goto exit;
	if (power_status) {
		pr_debug("turn on power of mbp slot");
		rc = mbp_power_up();
		mbp_power_status = 1;
	} else {
		pr_debug("turn off power of mbp slot");
		rc = mbp_power_down();
		mbp_power_status = 0;
	}
exit:
	return rc;
};

int mbp_register_status_notify(void (*callback)(int, void *),
	void *dev_id)
{
	mbp_status_notify_cb = callback;
	mbp_status_notify_cb_devid = dev_id;
	return 0;
}

static unsigned int mbp_status(struct device *dev)
{
	return mbp_power_status;
}

static uint32_t msm_sdcc_setup_power_mbp(struct device *dv, unsigned int vdd)
{
	struct platform_device *pdev;
	uint32_t rc = 0;

	pdev = container_of(dv, struct platform_device, dev);
	rc = msm_sdcc_setup_power(dv, vdd);
	if (rc) {
		pr_err("%s: Failed to setup power (%d)\n",
			__func__, rc);
		goto out;
	}
	if (!mbp_power_init_done) {
		mbp_setup_power(dv, 1);
		mbp_setup_power(dv, 0);
		mbp_power_init_done = 1;
	}
	if (vdd >= 0x8000) {
		rc = mbp_setup_power(dv, (0x8000 == vdd) ? 0 : 1);
		if (rc) {
			pr_err("%s: Failed to config mbp chip power (%d)\n",
				__func__, rc);
			goto out;
		}
		if (mbp_status_notify_cb) {
			mbp_status_notify_cb(mbp_power_status,
				mbp_status_notify_cb_devid);
		}
	}
out:
	/* should return 0 only */
	return 0;
}

#endif

#endif

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
static unsigned int msm7x30_sdcc_slot_status(struct device *dev)
{
/* < DTS2011092803933  sunhonghui 20110928 begin */
/*change for u8860 use the different pin to detect sd card*/
#ifdef CONFIG_HUAWEI_KERNEL
    if(machine_is_msm8255_u8860_51())
    {
        return (unsigned int)
            gpio_get_value_cansleep(
                PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SD_DET_PMICGPIO21_INTERUPT - 1));
    }
	/* < DTS2011102401822 liwei 20111024 begin */
    else if(machine_is_msm8255_u8680()
	    || machine_is_msm8255_u8667()
	    || machine_is_msm8255_u8730())
	/* DTS2011102401822 liwei 20111024 end > */
    {
        return (unsigned int)
            gpio_get_value_cansleep(
                PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SD_DET_PMICGPIO36_INTERUPT - 1));
    }
    else
    {
        return 0;
    }
#else
    return (unsigned int)
        gpio_get_value_cansleep(
            PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SD_DET_PMICGPIO36_INTERUPT - 1));
#endif
/* DTS2011092803933  sunhonghui 20110928 end > */
}
#endif

static void msm_sdcc_sdio_lpm_gpio(struct device *dv, unsigned int active)
{
	pr_debug("%s not implemented\n", __func__);
}

/* < BU5D09338 durui 20100506 begin*/
#ifndef CONFIG_HUAWEI_KERNEL
static int msm_sdcc_get_wpswitch(struct device *dv)
{
	void __iomem *wp_addr = 0;
	uint32_t ret = 0;
	struct platform_device *pdev;

	if (!(machine_is_msm7x30_surf()))
		return -1;
	pdev = container_of(dv, struct platform_device, dev);

	wp_addr = ioremap(FPGA_SDCC_STATUS, 4);
	if (!wp_addr) {
		pr_err("%s: Could not remap %x\n", __func__, FPGA_SDCC_STATUS);
		return -ENOMEM;
	}

	ret = (((readl(wp_addr) >> 4) >> (pdev->id-1)) & 0x01);
	pr_info("%s: WP Status for Slot %d = 0x%x \n", __func__,
							pdev->id, ret);
	iounmap(wp_addr);

	return ret;
}
#endif /*CONFIG_HUAWEI_KERNEL*/
/* BU5D09338 durui 20100506 end > */
#endif

#if defined(CONFIG_MMC_MSM_SDC1_SUPPORT)
#if defined(CONFIG_CSDIO_VENDOR_ID) && \
	defined(CONFIG_CSDIO_DEVICE_ID) && \
	(CONFIG_CSDIO_VENDOR_ID == 0x70 && CONFIG_CSDIO_DEVICE_ID == 0x1117)
static struct mmc_platform_data msm7x30_sdc1_data = {
	.ocr_mask	= MMC_VDD_165_195 | MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power_mbp,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.status	        = mbp_status,
	.register_status_notify = mbp_register_status_notify,
#ifdef CONFIG_MMC_MSM_SDC1_DUMMY52_REQUIRED
	.dummy52_required = 1,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 24576000,
	.nonremovable	= 0,
};
#else
static struct mmc_platform_data msm7x30_sdc1_data = {
	.ocr_mask	= MMC_VDD_165_195,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#ifdef CONFIG_MMC_MSM_SDC1_DUMMY52_REQUIRED
	.dummy52_required = 1,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 0,
};
#endif
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
/*< DTS2011090203887 hujun 20110902 begin */
/*merge from qcom SBA20*/
static struct mmc_platform_data msm7x30_sdc2_data = {
	.ocr_mask	= MMC_VDD_165_195 | MMC_VDD_27_28,
	.translate_vdd	= msm_sdcc_setup_power,
#ifdef CONFIG_MMC_MSM_SDC2_8_BIT_SUPPORT
	.mmc_bus_width  = MMC_CAP_8_BIT_DATA,
#else
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#endif
#ifdef CONFIG_MMC_MSM_SDC2_DUMMY52_REQUIRED
	.dummy52_required = 1,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 1,
};
/*DTS2011090203887 hujun 20110902 end >*/
#endif

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
static struct mmc_platform_data msm7x30_sdc3_data = {
	.ocr_mask	= MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#ifdef CONFIG_MMC_MSM_SDIO_SUPPORT
/* < DTS2011022102443 xuke 20110303 begin */
#ifdef CONFIG_HUAWEI_KERNEL
	.sdiowakeup_irq = MSM_GPIO_TO_INT(118),
#endif
/* DTS2011022102443 xuke 20110303 end > */
#endif
#ifdef CONFIG_MMC_MSM_SDC3_DUMMY52_REQUIRED
	.dummy52_required = 1,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 0,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
/* < DTS2011092803933  sunhonghui 20110928 begin */
/*this is used only by UE who's sdcard interupt pin is pmicgpio21*/
static struct mmc_platform_data msm7x30_sdc4_data = {
	.ocr_mask	= MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	.status      = msm7x30_sdcc_slot_status,
	.status_irq  = PM8058_GPIO_IRQ(PMIC8058_IRQ_BASE, PMIC_GPIO_SD_DET_PMICGPIO21_INTERUPT - 1),
	.irq_flags   = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
#endif
/* < BU5D09338 durui 20100506 begin*/
#ifndef CONFIG_HUAWEI_KERNEL
	.wpswitch    = msm_sdcc_get_wpswitch,
#endif
/* BU5D09338 durui 20100506 end > */
#ifdef CONFIG_MMC_MSM_SDC4_DUMMY52_REQUIRED
	.dummy52_required = 1,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
/*< DTS2010102201246 genghua 20101022 begin 
 * we modified the max value of the msmsdcc_fmax
 * to slow down the max value of the clock of the
 * externel SD card slot
 * to make more compatible for more SD card */  
#ifdef CONFIG_HUAWEI_KERNEL
	.msmsdcc_fmax	= 40960000,
#else
	.msmsdcc_fmax	= 49152000,
#endif
/* DTS2010102201246 genghua 20101022 end >*/
	.nonremovable	= 0,
};
/* DTS2011092803933  sunhonghui 20110928 end > */
#endif

/* < DTS2011092803933  sunhonghui 20110928 begin */
#ifdef CONFIG_HUAWEI_KERNEL
static struct mmc_platform_data msm7x30_sdc4_data_no_detect = {
	.ocr_mask	= MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#ifdef CONFIG_MMC_MSM_SDC4_DUMMY52_REQUIRED
	.dummy52_required = 1,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 40960000,
	.nonremovable	= 0,
};
#endif
/* DTS2011092803933  sunhonghui 20110928 end > */

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
static void msm_sdc1_lvlshft_enable(void)
{
	int rc;

	/* Enable LDO5, an input to the FET that powers slot 1 */
	rc = vreg_set_level(vreg_mmc, 2850);
	if (rc)
		printk(KERN_ERR "%s: vreg_set_level() = %d \n",	__func__, rc);

	rc = vreg_enable(vreg_mmc);
	if (rc)
		printk(KERN_ERR "%s: vreg_enable() = %d \n", __func__, rc);

	/* Enable GPIO 35, to turn on the FET that powers slot 1 */
	rc = msm_gpios_request_enable(sdc1_lvlshft_cfg_data,
				ARRAY_SIZE(sdc1_lvlshft_cfg_data));
	if (rc)
		printk(KERN_ERR "%s: Failed to enable GPIO 35\n", __func__);

	rc = gpio_direction_output(GPIO_PIN(sdc1_lvlshft_cfg_data[0].gpio_cfg),
				1);
	if (rc)
		printk(KERN_ERR "%s: Failed to turn on GPIO 35\n", __func__);
}
#endif

/* < DTS2010111804286  hanshirong 20101118 begin */
#ifdef CONFIG_HUAWEI_WIFI_SDCC

/* < DTS2011090203253 xuke 20110902 begin */
#define BCM_CHIP_4329						0
#define BCM_CHIP_4330						1
static int bcm_chip_type = -1;
/* DTS2011090203253 xuke 20110902 end > */

#define PREALLOC_WLAN_NUMBER_OF_SECTIONS	4
#define PREALLOC_WLAN_NUMBER_OF_BUFFERS		160
#define PREALLOC_WLAN_SECTION_HEADER		24

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 1024)

#define WLAN_SKB_BUF_NUM	16
/* < DTS2011090203253 xuke 20110902 begin */
#define WLAN_SLEEP_WAKE        40					/*40 is available in Asura / Phoenix*/
/* < DTS2011041800861 xuke 20110418 begin */
/* support wlan sleep wake on gpio 18*/
#define WLAN_SLEEP_WAKE_18		18					/*18 is available in U8860 / C8860*/
/* DTS2011041800861 xuke 20110418 end > */

#define WLAN_GPIO_FUNC_0         0
#define WLAN_GPIO_FUNC_1         1
#define WLAN_STAT_ON             1
#define WLAN_STAT_OFF            0

static struct msm_gpio wlan_wakes_msm[] = {
    { GPIO_CFG(WLAN_SLEEP_WAKE, WLAN_GPIO_FUNC_0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),"WLAN_WAKES_MSM"
    }    
};
/* < DTS2011041800861 xuke 20110418 begin */
static struct msm_gpio wlan_wakes_msm_18[] = {
    { GPIO_CFG(WLAN_SLEEP_WAKE_18, WLAN_GPIO_FUNC_0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),"WLAN_WAKES_MSM_18"
    }    
};
/* DTS2011041800861 xuke 20110418 end > */

/* for wifi power supply */
#define WLAN_REG 162								/*WLAN_REG is available in U8860 / C8860 / Asura / Phoenix*/
#define WLAN_PWR 164								/*WLAN_OWR is available in U8860 / C8860 , but we still request and use it in Ausra / Phoenix*/
/* DTS2011090203253 xuke 20110902 end > */

static struct msm_gpio wifi_config_init[] = {
    { GPIO_CFG(WLAN_REG, WLAN_GPIO_FUNC_0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
        "WL_REG_ON" }, 
    { GPIO_CFG(WLAN_PWR, WLAN_GPIO_FUNC_0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
        "WL_PWR_ON" }
};
static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

typedef struct wifi_mem_prealloc_struct {
	void *mem_ptr;
	unsigned long size;
} wifi_mem_prealloc_t;

static wifi_mem_prealloc_t wifi_mem_array[PREALLOC_WLAN_NUMBER_OF_SECTIONS] = {
	{ NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER) }
};

static void *bcm_wifi_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_NUMBER_OF_SECTIONS)
		return wlan_static_skb;
	if ((section < 0) || (section > PREALLOC_WLAN_NUMBER_OF_SECTIONS))
		return NULL;
	if (wifi_mem_array[section].size < size)
		return NULL;
	return wifi_mem_array[section].mem_ptr;
}

/* < DTS2011110105497 zhuwenying 20111117 begin */
static int bcm_wifi_set_power_4330(int enable)
{
	int ret = 0;

   	if (enable)
	{
			/* turn on wifi_vreg */
            ret = gpio_direction_output(WLAN_REG, WLAN_STAT_ON);
            if (ret < 0) {
            	printk(KERN_ERR "%s: turn on wlan_reg failed (%d)\n" , __func__, ret);
            	return -EIO;
            }
            mdelay(150);
            printk(KERN_ERR "%s: wifi power successed to pull up\n" , __func__ );
		
	}
    else { 
        	/* turn off wifi_vreg */
            ret = gpio_direction_output(WLAN_REG, WLAN_STAT_OFF);
            if (ret < 0) {
            	printk(KERN_ERR "%s: turn off wlan_reg failed (%d)\n" , __func__, ret);
            	return -EIO;
            }
            mdelay(1);
            printk(KERN_ERR "%s: wifi power successed to pull down\n",__func__ );
	}

	return ret;
}
/* DTS2011110105497 zhuwenying 20111117 end > */


static int bcm_wifi_set_power(int enable)
{
	int ret = 0;

   	if (enable)
	{
            //turn on wifi_vreg
            ret = gpio_direction_output(WLAN_REG, WLAN_STAT_ON);
            if (ret < 0) {
                    printk(KERN_ERR
                            "%s: turn on wlan_reg failed (%d)\n",
                                    __func__, ret);
                    return -EIO;
            }
            mdelay(1);
            
            //turn on wifi_reset
            ret = gpio_direction_output(WLAN_PWR, WLAN_STAT_ON);
            if (ret < 0) {
                    printk(KERN_ERR
                            "%s: enable wlan_reset failed (%d)\n",
                                    __func__, ret);
                    return -EIO;
            }
            mdelay(150);
            printk(KERN_ERR "%s: wifi power successed to pull up\n",__func__);
		
	}
        else
        { 
            //turn off wifi_reset
            ret = gpio_direction_output(WLAN_PWR, WLAN_STAT_OFF);
            if (ret < 0) {
                    printk(KERN_ERR
                            "%s: disable wlan_reset failed (%d)\n",
                                    __func__, ret);
                    return -EIO;
            }
            mdelay(1);

            //turn off wifi_vreg
            ret = gpio_direction_output(WLAN_REG, WLAN_STAT_OFF);
            if (ret < 0) {
                    printk(KERN_ERR
                            "%s: turn off wlan_reg failed (%d)\n",
                                    __func__, ret);
                    return -EIO;
            }
            mdelay(1);
            printk(KERN_ERR "%s: wifi power successed to pull down\n",__func__);
	}

	return ret;
}

int __init bcm_wifi_init_gpio_mem(void)
{
	int i = 0;
	int rc = 0;

	/* < DTS2011090203253 xuke 20110902 begin */
	if( bcm_chip_type == BCM_CHIP_4330 ) {
        rc = msm_gpios_request_enable(wlan_wakes_msm,
                                ARRAY_SIZE(wlan_wakes_msm));
        if (rc < 0) {
            printk(KERN_ERR
                        "%s: msm_gpios_request_enable failed (%d)\n",
                        __func__, rc);
            return -EIO;
        }
	}
	else if( bcm_chip_type == BCM_CHIP_4329 ) {
	/* < DTS2011041800861 xuke 20110418 begin */
		rc = msm_gpios_request_enable(wlan_wakes_msm_18,
                                ARRAY_SIZE(wlan_wakes_msm_18));
        if (rc < 0) {
            printk(KERN_ERR
                        "%s: wlan_wakes_msm_18 msm_gpios_request_enable failed (%d)\n",
                        __func__, rc);
            return -EIO;
        }
	/* DTS2011041800861 xuke 20110418 end > */
	}
	else {
		printk(KERN_ERR "%s: unkown bcm_chip_type = %d\n", __func__, bcm_chip_type );
	}
	/* DTS2011090203253 xuke 20110902 end > */

	rc = msm_gpios_request_enable(wifi_config_init,
					ARRAY_SIZE(wifi_config_init));
            if (rc < 0) {
                    printk(KERN_ERR
                            "%s: wifi_config_init msm_gpios_request_enable failed (%d)\n",
                                    __func__, rc);
                    return -EIO;
            }

        mdelay(5);
        //turn off wifi_vreg
        rc = gpio_direction_output(WLAN_REG, 0);
        if (rc < 0) {
                printk(KERN_ERR
                        "%s: turn off wlan_reg failed (%d)\n",
                                __func__, rc);
                return -EIO;
        }
        mdelay(5);
        
        //turn off wifi_reset
        rc = gpio_direction_output(WLAN_PWR, 0);
        if (rc < 0) {
                printk(KERN_ERR
                        "%s: disable wlan_reset failed (%d)\n",
                                __func__, rc);
                return -EIO;
        }
        mdelay(5);
       
	/* < DTS2011050604788 hanshirong 20110510 begin */
	printk("dev_alloc_skb malloc 32k buffer to avoid page allocation fail\n");
	for(i=0;( i < WLAN_SKB_BUF_NUM );i++) {
		if (i < (WLAN_SKB_BUF_NUM/2))
			wlan_static_skb[i] = dev_alloc_skb(4096); //malloc skb 4k buffer
		else
			wlan_static_skb[i] = dev_alloc_skb(32768); //malloc skb 32k buffer
	}
	/* DTS2011050604788 hanshirong 20110510 end > */
	for(i=0;( i < PREALLOC_WLAN_NUMBER_OF_SECTIONS );i++) {
		wifi_mem_array[i].mem_ptr = kmalloc(wifi_mem_array[i].size,
							GFP_KERNEL);
		if (wifi_mem_array[i].mem_ptr == NULL)
			return -ENOMEM;
	}
	
	printk("bcm_wifi_init_gpio_mem successfully \n");
	return 0;
}

/* < DTS2011090203253 xuke 20110902 begin */
int bcm_detect_chip_type( void )
{
	if((machine_is_msm8255_u8860()) 		
	|| (machine_is_msm8255_c8860()) 		
	|| (machine_is_msm8255_u8860lp())		
	|| (machine_is_msm8255_u8860_92())
/*<DTS2011100803656 liyuping 20111008 begin*/
//Add three products,all of them use BCM4329 chip.
	|| (machine_is_msm7x30_u8800_51())
	|| (machine_is_msm8255_u8800_pro())
	|| (machine_is_msm7x30_u8820())	
/* DTS2011100803656 liyuping 20111008 end >*/
/*<DTS2011091502092 liyuping 20110915 begin */
	|| (machine_is_msm8255_u8860_51()))
/* DTS2011091502092 liyuping 20110915 end> */ 
	{
		bcm_chip_type = BCM_CHIP_4329;
	}
	else {
		bcm_chip_type = BCM_CHIP_4330;
	}

	printk( KERN_INFO "%s : bcm_chip_type = %d\n" , __FUNCTION__ , bcm_chip_type );

	return 0;
}
/* DTS2011090203253 xuke 20110902 end > */

/* < DTS2011042103409 hanshirong 20110423 begin */
int bcm_set_carddetect(int detect)
{
	return (int)(sub_board_id&HW_VER_SUB_MASK);
}
/* DTS2011042103409 hanshirong 20110423 end > */
	
static struct wifi_platform_data bcm_wifi_control = {
	.mem_prealloc	= bcm_wifi_mem_prealloc,
	.set_power	=bcm_wifi_set_power,
/* < DTS2011042103409 hanshirong 20110423 begin */
	.set_carddetect = bcm_set_carddetect,
/* DTS2011042103409 hanshirong 20110423 end > */
/* < DTS2011052606009 jiaxianghong 20110527 end */
};

static struct platform_device bcm_wifi_device = {
        /* bcm4329_wlan device */
        .name           = "bcm4329_wlan",
        .id             = 1,
        .num_resources  = 0,
        .resource       = NULL,
        .dev            = {
                .platform_data = &bcm_wifi_control,
        },
};
/* < DTS2011110105497 zhuwenying 20111117 begin */

static struct wifi_platform_data bcm_wifi_control_4330 = {
	.mem_prealloc	= bcm_wifi_mem_prealloc,
	.set_power	=bcm_wifi_set_power_4330,
};

/* < DTS2011090203253 xuke 20110902 begin */
static struct platform_device bcm_wifi_device_4330 = {
        /* bcm4330_wlan device */
        .name           = "bcm4330_wlan",
        .id             = 1,
        .num_resources  = 0,
        .resource       = NULL,
        .dev            = {
                .platform_data = &bcm_wifi_control_4330,
        },
};
/* DTS2011090203253 xuke 20110902 end > */
#endif
/* DTS2011110105497 zhuwenying 20111117 end > */

/* DTS2010111804286  hanshirong 20101118 end > */

/*<DTS2011042602009 fengwei 20110426 begin*/
/* add virtual keys fucntion */
/* same product use same config for virtual key */
#ifdef CONFIG_HUAWEI_KERNEL
static ssize_t synaptics_virtual_keys_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
        memcpy( buf, buf_virtualkey, buf_vkey_size );
		return buf_vkey_size; 
}

static struct kobj_attribute synaptics_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.synaptics",
		.mode = S_IRUGO,
	},
	.show = &synaptics_virtual_keys_show,
};

static struct attribute *synaptics_properties_attrs[] = {
	&synaptics_virtual_keys_attr.attr,
	NULL
};

static struct attribute_group synaptics_properties_attr_group = {
	.attrs = synaptics_properties_attrs,
};

static void __init virtualkeys_init(void)
{
    struct kobject *properties_kobj;
    int ret;
    /*<DTS2011051601005  fengwei 20110516 begin*/	
    /*< DTS2011093000343 duanfei 20110930 begin */
    if (machine_is_msm8255_c8860())
    {
        if ((HW_VER_SUB_VA == get_hw_sub_board_id())
            ||(HW_VER_SUB_VB == get_hw_sub_board_id()))
        {
            buf_vkey_size = sprintf(buf_virtualkey,
        			    __stringify(EV_KEY) ":" __stringify(KEY_HOME)  ":50:930:112:80"
        		       ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":180:930:112:80"
        		       ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":300:930:112:80"
        		       ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":430:930:112:80"
        		       "\n");
        }
        else
        {
            buf_vkey_size = sprintf(buf_virtualkey,
        			    __stringify(EV_KEY) ":" __stringify(KEY_MENU)  ":50:930:112:80"
        		       ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":180:930:112:80"
        		       ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":300:930:112:80"
        		       ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":430:930:112:80"
        		       "\n");
        }
    }
    else if (machine_is_msm8255_u8860_51())
    {
        if (HW_VER_SUB_VA == get_hw_sub_board_id())
        {
            buf_vkey_size = sprintf(buf_virtualkey,
        			    __stringify(EV_KEY) ":" __stringify(KEY_HOME)  ":50:930:112:80"
        		       ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":180:930:112:80"
        		       ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":300:930:112:80"
        		       ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":430:930:112:80"
        		       "\n");
        }
        else
        {
            buf_vkey_size = sprintf(buf_virtualkey,
        			    __stringify(EV_KEY) ":" __stringify(KEY_MENU)  ":50:930:112:80"
        		       ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":180:930:112:80"
        		       ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":300:930:112:80"
        		       ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":430:930:112:80"
        		       "\n");
        }
    }
    else if (machine_is_msm8255_u8860lp())
    {
        if (HW_VER_SUB_VA == get_hw_sub_board_id())
        {
            buf_vkey_size = sprintf(buf_virtualkey,
        			    __stringify(EV_KEY) ":" __stringify(KEY_HOME)  ":50:930:112:80"
        		       ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":180:930:112:80"
        		       ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":300:930:112:80"
        		       ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":430:930:112:80"
        		       "\n");
        }
        else
        {
            buf_vkey_size = sprintf(buf_virtualkey,
        			    __stringify(EV_KEY) ":" __stringify(KEY_MENU)  ":50:930:112:80"
        		       ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":180:930:112:80"
        		       ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":300:930:112:80"
        		       ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":430:930:112:80"
        		       "\n");
        }
    }
    else if (machine_is_msm8255_u8860_92()
            ||machine_is_msm8255_u8860())
    {
        buf_vkey_size = sprintf(buf_virtualkey,
                        __stringify(EV_KEY) ":" __stringify(KEY_HOME)  ":50:930:112:80"
                      ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":180:930:112:80"
                      ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":300:930:112:80"
                      ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":430:930:112:80"
                      "\n");
    }
    /* DTS2011093000343 duanfei 20110930 end >*/
    else if ( machine_is_msm7x30_u8820())
    {
        buf_vkey_size = sprintf(buf_virtualkey,
        			__stringify(EV_KEY) ":" __stringify(KEY_MENU)  ":67:850:130:80"
        		   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":192:850:112:80"
        		   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":309:850:116:80"
        		   ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":424:850:110:80"
        		   "\n");
    }
    else if ( machine_is_msm7x30_u8800_51()
		   ||machine_is_msm8255_u8800_pro()
                   ||machine_is_msm7x30_u8800())
    {
        buf_vkey_size = sprintf(buf_virtualkey,
        			__stringify(EV_KEY) ":" __stringify(KEY_BACK)  ":67:850:130:80"
        		   ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":192:850:112:80"
        		   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":309:850:116:80"
        		   ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":424:850:110:80"
        		   "\n");
    }
    /*< DTS2011091902006 duanfei 20110919 begin*/
    /* <DTS2011091200073 zhangbo 20110912 begin */
    /* < DTS2011102401822 liwei 20111024 begin */
    else if (machine_is_msm8255_u8680()
        || machine_is_msm8255_u8667()
		|| machine_is_msm8255_u8730())
    /* DTS2011102401822 liwei 20111024 end > */
    /* DTS2011091200073 zhangbo 20110912 end> */
    {
    	buf_vkey_size = sprintf(buf_virtualkey,
        			__stringify(EV_KEY) ":" __stringify(KEY_HOME)        ":50:840:112:70"
        		   ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)         ":180:840:112:70"
        		   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)         ":300:840:112:70"
        		   ":" __stringify(EV_KEY) ":" __stringify(KEY_GENIUSBUTTON) ":430:840:112:70"  /*special key for a certain apk*/
        		   "\n");
    }
    /* DTS2011091902006 duanfei 20110919 end >*/
    /*< DTS2011093000343 duanfei 20110930 begin */
    else
    {
        buf_vkey_size = sprintf(buf_virtualkey,
        			    __stringify(EV_KEY) ":" __stringify(KEY_MENU)  ":50:930:112:80"
        		       ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":180:930:112:80"
        		       ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":300:930:112:80"
        		       ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":430:930:112:80"
        		       "\n");  
    }
    /* DTS2011093000343 duanfei 20110930 end >*/
    /*DTS2011051601005 fengwei 20110516 end>*/

   	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		ret = sysfs_create_group(properties_kobj,
					 &synaptics_properties_attr_group);
	if (!properties_kobj || ret)
		pr_err("failed to create board_properties\n");
}
#endif

/*DTS2011042602009 fengwei 20110426 end>*/


static void __init msm7x30_init_mmc(void)
{
	vreg_s3 = vreg_get(NULL, "s3");
	if (IS_ERR(vreg_s3)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_s3));
		return;
	}

	vreg_mmc = vreg_get(NULL, "mmc");
	if (IS_ERR(vreg_mmc)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_mmc));
		return;
	}

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
/*< DTS2010092400487  lijianzhao 20100924 begin */
	/*< DTS2010112702297 wangquanli 201001125 begin */
	/*< DTS2011030202729  liliang 20110302  begin */	
	/*< DTS2011042703705 zhangbo 20110422 begin */
	/* <DTS2011050700551 zhangbo 20110505 begin */
    /* <DTS2011062600102 sunhonghui 20110626 begin */
    /* <DTS2011071600361 liyuping 20110716 begin */
	/* < DTS2011082302564 liwei 20110823 begin */
	/* <DTS2011091200073 zhangbo 20110912 begin */
	/*<DTS2011091502092 liyuping 20110915 begin */
	/* < DTS2011102401822 liwei 20111024 begin */
	if ( machine_is_msm7x30_fluid() 
	|| (machine_is_msm7x30_u8800()) 
	|| (machine_is_msm7x30_u8820()) 
	|| (machine_is_msm7x30_u8800_51()) 
	|| (machine_is_msm8255_u8800_pro())
	|| (machine_is_msm8255_u8860())
	|| (machine_is_msm8255_c8860())
    || (machine_is_msm8255_u8860lp())
    || (machine_is_msm8255_u8860_92())
	|| (machine_is_msm8255_u8680())
    || machine_is_msm8255_u8667()
	|| (machine_is_msm8255_u8860_51())
	|| (machine_is_msm8255_u8730()))
    {
	/* DTS2011102401822 liwei 20111024 end > */
    /* DTS2011091502092 liyuping 20110915 end> */
  /* DTS2011091200073 zhangbo 20110912 end> */
	/* DTS2011082302564 liwei 20110823 end > */
	/* DTS2011071600361 liyuping 20110716 end> */
    /* DTS2011062600102 sunhonghui 20110626 end> */
	/* DTS2011050700551 zhangbo 20110505 end> */
	/* DTS2011042703705 zhangbo 20110422 end >*/
	/* DTS2011030202729  liliang 20110302 end >*/
	/* DTS2010112702297 wangquanli 201001125 end >*/
		msm7x30_sdc1_data.ocr_mask =  MMC_VDD_27_28 | MMC_VDD_28_29;
		msm_sdc1_lvlshft_enable();
	}
/* DTS2010092400487  lijianzhao 20100924 end >*/
	sdcc_vreg_data[0].vreg_data = vreg_s3;
	sdcc_vreg_data[0].level = 1800;
	msm_add_sdcc(1, &msm7x30_sdc1_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	if (machine_is_msm8x55_svlte_surf())
		msm7x30_sdc2_data.msmsdcc_fmax =  24576000;
	if (machine_is_msm8x55_svlte_surf() ||
			machine_is_msm8x55_svlte_ffa()) {
		msm7x30_sdc2_data.sdio_lpm_gpio_setup = msm_sdcc_sdio_lpm_gpio;
#ifdef CONFIG_MMC_MSM_SDIO_SUPPORT
		msm7x30_sdc2_data.sdiowakeup_irq = MSM_GPIO_TO_INT(68);
#ifdef CONFIG_MSM_SDIO_AL
		msm7x30_sdc2_data.is_sdio_al_client = 1;
#endif
#endif
	}
	sdcc_vreg_data[1].vreg_data = vreg_s3;
	sdcc_vreg_data[1].level = 1800;
	msm_add_sdcc(2, &msm7x30_sdc2_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
	sdcc_vreg_data[2].vreg_data = vreg_s3;
	sdcc_vreg_data[2].level = 1800;
	msm_sdcc_setup_gpio(3, 1);
	msm_add_sdcc(3, &msm7x30_sdc3_data);
/* < DTS2010111804286  hanshirong 20101118 begin */
#ifdef CONFIG_HUAWEI_WIFI_SDCC
/* < DTS2011090203253 xuke 20110902 begin */
	bcm_detect_chip_type();
	bcm_wifi_init_gpio_mem();
	platform_device_register(&bcm_wifi_device);
	platform_device_register(&bcm_wifi_device_4330);
/* DTS2011090203253 xuke 20110902 end > */
#endif
/* DTS2010111804286  hanshirong 20101118 end > */
#endif
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
	sdcc_vreg_data[3].vreg_data = vreg_mmc;
	sdcc_vreg_data[3].level = 2850;
	
    /* < DTS2011092803933  sunhonghui 20110928 begin */
    /*change for u8860lp use the different pin to detect sd card*/
#ifdef CONFIG_HUAWEI_KERNEL
    if(machine_is_msm8255_u8860_51())
    {
        msm_add_sdcc(4, &msm7x30_sdc4_data);
    }
	/* < DTS2011102401822 liwei 20111024 begin */
    else if(machine_is_msm8255_u8680()
	    || machine_is_msm8255_u8667()
	    || machine_is_msm8255_u8730())
    /* DTS2011102401822 liwei 20111024 end > */
    {
        msm7x30_sdc4_data.status_irq  = PM8058_GPIO_IRQ(PMIC8058_IRQ_BASE, PMIC_GPIO_SD_DET_PMICGPIO36_INTERUPT - 1),
        msm_add_sdcc(4, &msm7x30_sdc4_data);
    }
    else
    {
        msm_add_sdcc(4, &msm7x30_sdc4_data_no_detect);
    }
#else
    msm_add_sdcc(4, &msm7x30_sdc4_data);
#endif
    /* DTS2011092803933  sunhonghui 20110928 end > */
#endif

}

/* <DTS2010072202961 hufeng 20100722 begin */
/* removed several lines */
/* DTS2010072202961 hufeng 20100722 end> */

#ifdef CONFIG_SERIAL_MSM_CONSOLE
static struct msm_gpio uart2_config_data[] = {
	{ GPIO_CFG(49, 2, GPIO_CFG_OUTPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "UART2_RFR"},
	{ GPIO_CFG(50, 2, GPIO_CFG_INPUT,   GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "UART2_CTS"},
	{ GPIO_CFG(51, 2, GPIO_CFG_INPUT,   GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "UART2_Rx"},
	{ GPIO_CFG(52, 2, GPIO_CFG_OUTPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "UART2_Tx"},
};

static void msm7x30_init_uart2(void)
{
	msm_gpios_request_enable(uart2_config_data,
			ARRAY_SIZE(uart2_config_data));

}
#endif

/* TSIF begin */
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)

#define TSIF_B_SYNC      GPIO_CFG(37, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_B_DATA      GPIO_CFG(36, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_B_EN        GPIO_CFG(35, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_B_CLK       GPIO_CFG(34, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)

static const struct msm_gpio tsif_gpios[] = {
	{ .gpio_cfg = TSIF_B_CLK,  .label =  "tsif_clk", },
	{ .gpio_cfg = TSIF_B_EN,   .label =  "tsif_en", },
	{ .gpio_cfg = TSIF_B_DATA, .label =  "tsif_data", },
	{ .gpio_cfg = TSIF_B_SYNC, .label =  "tsif_sync", },
};

static struct msm_tsif_platform_data tsif_platform_data = {
	.num_gpios = ARRAY_SIZE(tsif_gpios),
	.gpios = tsif_gpios,
	.tsif_pclk = "tsif_pclk",
	.tsif_ref_clk = "tsif_ref_clk",
};
#endif /* defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE) */
/* TSIF end   */

static void __init pmic8058_leds_init(void)
{
	if (machine_is_msm7x30_surf()) {
		pm8058_7x30_data.sub_devices[PM8058_SUBDEV_LED].platform_data
			= &pm8058_surf_leds_data;
		pm8058_7x30_data.sub_devices[PM8058_SUBDEV_LED].data_size
			= sizeof(pm8058_surf_leds_data);
	} else if (!machine_is_msm7x30_fluid()) {
		pm8058_7x30_data.sub_devices[PM8058_SUBDEV_LED].platform_data
			= &pm8058_ffa_leds_data;
		pm8058_7x30_data.sub_devices[PM8058_SUBDEV_LED].data_size
			= sizeof(pm8058_ffa_leds_data);
	} else if (machine_is_msm7x30_fluid()) {
		pm8058_7x30_data.sub_devices[PM8058_SUBDEV_LED].platform_data
			= &pm8058_fluid_leds_data;
		pm8058_7x30_data.sub_devices[PM8058_SUBDEV_LED].data_size
			= sizeof(pm8058_fluid_leds_data);
	}
}

static struct msm_spm_platform_data msm_spm_data __initdata = {
	.reg_base_addr = MSM_SAW_BASE,

	.reg_init_values[MSM_SPM_REG_SAW_CFG] = 0x05,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_CTL] = 0x18,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY] = 0x00006666,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY] = 0xFF000666,

	.reg_init_values[MSM_SPM_REG_SAW_SLP_CLK_EN] = 0x01,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN] = 0x03,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN] = 0x00,

	.reg_init_values[MSM_SPM_REG_SAW_SLP_CLMP_EN] = 0x01,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_RST_EN] = 0x00,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_MPM_CFG] = 0x00,

	.awake_vlevel = 0xF2,
	.retention_vlevel = 0xE0,
	.collapse_vlevel = 0x72,
	.retention_mid_vlevel = 0xE0,
	.collapse_mid_vlevel = 0xE0,

	.vctl_timeout_us = 50,
};

#if defined(CONFIG_TOUCHSCREEN_TSC2007) || \
	defined(CONFIG_TOUCHSCREEN_TSC2007_MODULE)

#define TSC2007_TS_PEN_INT	20

static struct msm_gpio tsc2007_config_data[] = {
	{ GPIO_CFG(TSC2007_TS_PEN_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	"tsc2007_irq" },
};

static struct vreg *vreg_tsc_s3;
static struct vreg *vreg_tsc_s2;

static int tsc2007_init(void)
{
	int rc;

	vreg_tsc_s3 = vreg_get(NULL, "s3");
	if (IS_ERR(vreg_tsc_s3)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_tsc_s3));
		return -ENODEV;
	}

	rc = vreg_set_level(vreg_tsc_s3, 1800);
	if (rc) {
		pr_err("%s: vreg_set_level failed \n", __func__);
		goto fail_vreg_set_level;
	}

	rc = vreg_enable(vreg_tsc_s3);
	if (rc) {
		pr_err("%s: vreg_enable failed \n", __func__);
		goto fail_vreg_set_level;
	}

	vreg_tsc_s2 = vreg_get(NULL, "s2");
	if (IS_ERR(vreg_tsc_s2)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_tsc_s2));
		goto fail_vreg_get;
	}

	rc = vreg_set_level(vreg_tsc_s2, 1300);
	if (rc) {
		pr_err("%s: vreg_set_level failed \n", __func__);
		goto fail_vreg_s2_level;
	}

	rc = vreg_enable(vreg_tsc_s2);
	if (rc) {
		pr_err("%s: vreg_enable failed \n", __func__);
		goto fail_vreg_s2_level;
	}

	rc = msm_gpios_request_enable(tsc2007_config_data,
			ARRAY_SIZE(tsc2007_config_data));
	if (rc) {
		pr_err("%s: Unable to request gpios\n", __func__);
		goto fail_gpio_req;
	}

	return 0;

fail_gpio_req:
	vreg_disable(vreg_tsc_s2);
fail_vreg_s2_level:
	vreg_put(vreg_tsc_s2);
fail_vreg_get:
	vreg_disable(vreg_tsc_s3);
fail_vreg_set_level:
	vreg_put(vreg_tsc_s3);
	return rc;
}

static int tsc2007_get_pendown_state(void)
{
	int rc;

	rc = gpio_get_value(TSC2007_TS_PEN_INT);
	if (rc < 0) {
		pr_err("%s: MSM GPIO %d read failed\n", __func__,
						TSC2007_TS_PEN_INT);
		return rc;
	}

	return (rc == 0 ? 1 : 0);
}

static void tsc2007_exit(void)
{
	vreg_disable(vreg_tsc_s3);
	vreg_put(vreg_tsc_s3);
	vreg_disable(vreg_tsc_s2);
	vreg_put(vreg_tsc_s2);

	msm_gpios_disable_free(tsc2007_config_data,
		ARRAY_SIZE(tsc2007_config_data));
}

static int tsc2007_power_shutdown(bool enable)
{
	int rc;

	if (enable == false) {
		rc = vreg_enable(vreg_tsc_s2);
		if (rc) {
			pr_err("%s: vreg_enable failed\n", __func__);
			return rc;
		}
		rc = vreg_enable(vreg_tsc_s3);
		if (rc) {
			pr_err("%s: vreg_enable failed\n", __func__);
			vreg_disable(vreg_tsc_s2);
			return rc;
		}
		/* Voltage settling delay */
		msleep(20);
	} else {
		rc = vreg_disable(vreg_tsc_s2);
		if (rc) {
			pr_err("%s: vreg_disable failed\n", __func__);
			return rc;
		}
		rc = vreg_disable(vreg_tsc_s3);
		if (rc) {
			pr_err("%s: vreg_disable failed\n", __func__);
			vreg_enable(vreg_tsc_s2);
			return rc;
		}
	}

	return rc;
}

static struct tsc2007_platform_data tsc2007_ts_data = {
	.model = 2007,
	.x_plate_ohms = 300,
	.irq_flags    = IRQF_TRIGGER_LOW,
	.init_platform_hw = tsc2007_init,
	.exit_platform_hw = tsc2007_exit,
	.power_shutdown	  = tsc2007_power_shutdown,
	.invert_x	  = true,
	.invert_y	  = true,
	/* REVISIT: Temporary fix for reversed pressure */
	.invert_z1	  = true,
	.invert_z2	  = true,
	.get_pendown_state = tsc2007_get_pendown_state,
};

static struct i2c_board_info tsc_i2c_board_info[] = {
	{
		I2C_BOARD_INFO("tsc2007", 0x48),
		.irq		= MSM_GPIO_TO_INT(TSC2007_TS_PEN_INT),
		.platform_data = &tsc2007_ts_data,
	},
};
#endif

static const char *vregs_isa1200_name[] = {
	"gp7",
	"gp10",
};

static const int vregs_isa1200_val[] = {
	1800,
	2600,
};
static struct vreg *vregs_isa1200[ARRAY_SIZE(vregs_isa1200_name)];

static int isa1200_power(int vreg_on)
{
	int i, rc = 0;

	for (i = 0; i < ARRAY_SIZE(vregs_isa1200_name); i++) {
		if (!vregs_isa1200[i]) {
			pr_err("%s: vreg_get %s failed (%d)\n",
				__func__, vregs_isa1200_name[i], rc);
			goto vreg_fail;
		}

		rc = vreg_on ? vreg_enable(vregs_isa1200[i]) :
			  vreg_disable(vregs_isa1200[i]);
		if (rc < 0) {
			pr_err("%s: vreg %s %s failed (%d)\n",
				__func__, vregs_isa1200_name[i],
			       vreg_on ? "enable" : "disable", rc);
			goto vreg_fail;
		}
	}
	return 0;

vreg_fail:
	while (i)
		vreg_disable(vregs_isa1200[--i]);
	return rc;
}

static int isa1200_dev_setup(bool enable)
{
	int i, rc;

	if (enable == true) {
		for (i = 0; i < ARRAY_SIZE(vregs_isa1200_name); i++) {
			vregs_isa1200[i] = vreg_get(NULL,
						vregs_isa1200_name[i]);
			if (IS_ERR(vregs_isa1200[i])) {
				pr_err("%s: vreg get %s failed (%ld)\n",
					__func__, vregs_isa1200_name[i],
					PTR_ERR(vregs_isa1200[i]));
				rc = PTR_ERR(vregs_isa1200[i]);
				goto vreg_get_fail;
			}
			rc = vreg_set_level(vregs_isa1200[i],
					vregs_isa1200_val[i]);
			if (rc) {
				pr_err("%s: vreg_set_level() = %d\n",
					__func__, rc);
				goto vreg_get_fail;
			}
		}

		rc = gpio_tlmm_config(GPIO_CFG(HAP_LVL_SHFT_MSM_GPIO, 0,
				GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: Could not configure gpio %d\n",
					__func__, HAP_LVL_SHFT_MSM_GPIO);
			goto vreg_get_fail;
		}

		rc = gpio_request(HAP_LVL_SHFT_MSM_GPIO, "haptics_shft_lvl_oe");
		if (rc) {
			pr_err("%s: unable to request gpio %d (%d)\n",
					__func__, HAP_LVL_SHFT_MSM_GPIO, rc);
			goto vreg_get_fail;
		}

		gpio_set_value(HAP_LVL_SHFT_MSM_GPIO, 1);
	} else {
		for (i = 0; i < ARRAY_SIZE(vregs_isa1200_name); i++)
			vreg_put(vregs_isa1200[i]);

		gpio_free(HAP_LVL_SHFT_MSM_GPIO);
	}

	return 0;
vreg_get_fail:
	while (i)
		vreg_put(vregs_isa1200[--i]);
	return rc;
}
static struct isa1200_platform_data isa1200_1_pdata = {
	.name = "vibrator",
	.power_on = isa1200_power,
	.dev_setup = isa1200_dev_setup,
	.pwm_ch_id = 1, /*channel id*/
	/*gpio to enable haptic*/
	.hap_en_gpio = PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_HAP_ENABLE),
	.max_timeout = 15000,
	.mode_ctrl = PWM_GEN_MODE,
	.pwm_fd = {
		.pwm_div = 256,
	},
	.is_erm = false,
	.smart_en = true,
	.ext_clk_en = true,
	.chip_en = 1,
};

static struct i2c_board_info msm_isa1200_board_info[] = {
	{
		I2C_BOARD_INFO("isa1200_1", 0x90>>1),
		.platform_data = &isa1200_1_pdata,
	},
};

/*< DTS2011091402372   yuguangcai 20110914 begin */
/*Add new i2c information for flash tps61310*/
#ifdef CONFIG_HUAWEI_FEATURE_TPS61310
static struct i2c_board_info tps61310_board_info[] = {
	{
		I2C_BOARD_INFO("tps61310" , 0x33),
	},
};
#endif
/* DTS2011091402372   yuguangcai 20110914 end > */

static int kp_flip_mpp_config(void)
{
	return pm8058_mpp_config_digital_in(PM_FLIP_MPP,
		PM8058_MPP_DIG_LEVEL_S3, PM_MPP_DIN_TO_INT);
}

static struct flip_switch_pdata flip_switch_data = {
	.name = "kp_flip_switch",
	.flip_gpio = PM8058_GPIO_PM_TO_SYS(PM8058_GPIOS) + PM_FLIP_MPP,
	.left_key = KEY_OPEN,
	.right_key = KEY_CLOSE,
	.active_low = 0,
	.wakeup = 1,
	.flip_mpp_config = kp_flip_mpp_config,
};

static struct platform_device flip_switch_device = {
	.name   = "kp_flip_switch",
	.id	= -1,
	.dev    = {
		.platform_data = &flip_switch_data,
	}
};

static const char *vregs_tma300_name[] = {
	"gp6",
	"gp7",
};

static const int vregs_tma300_val[] = {
	3050,
	1800,
};
static struct vreg *vregs_tma300[ARRAY_SIZE(vregs_tma300_name)];

static int tma300_power(int vreg_on)
{
	int i, rc = -EINVAL;

	for (i = 0; i < ARRAY_SIZE(vregs_tma300_name); i++) {
		/* Never disable gp6 for fluid as lcd has a problem with it */
		if (!i && !vreg_on)
			continue;
		if (!vregs_tma300[i]) {
			printk(KERN_ERR "%s: vreg_get %s failed (%d)\n",
				__func__, vregs_tma300_name[i], rc);
			return rc;
		}

		rc = vreg_on ? vreg_enable(vregs_tma300[i]) :
			  vreg_disable(vregs_tma300[i]);
		if (rc < 0) {
			printk(KERN_ERR "%s: vreg %s %s failed (%d)\n",
				__func__, vregs_tma300_name[i],
			       vreg_on ? "enable" : "disable", rc);
			return rc;
		}
	}

	return 0;
}

#define TS_GPIO_IRQ 150

static int tma300_dev_setup(bool enable)
{
	int i, rc;

	if (enable) {
		/* get voltage sources */
		for (i = 0; i < ARRAY_SIZE(vregs_tma300_name); i++) {
			vregs_tma300[i] = vreg_get(NULL, vregs_tma300_name[i]);
			if (IS_ERR(vregs_tma300[i])) {
				pr_err("%s: vreg get %s failed (%ld)\n",
					__func__, vregs_tma300_name[i],
					PTR_ERR(vregs_tma300[i]));
				rc = PTR_ERR(vregs_tma300[i]);
				goto vreg_get_fail;
			}
			rc = vreg_set_level(vregs_tma300[i],
					vregs_tma300_val[i]);
			if (rc) {
				pr_err("%s: vreg_set_level() = %d\n",
					__func__, rc);
				i++;
				goto vreg_get_fail;
			}
		}

		/* enable interrupt gpio */
		rc = gpio_tlmm_config(GPIO_CFG(TS_GPIO_IRQ, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_UP, GPIO_CFG_6MA), GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: Could not configure gpio %d\n",
					__func__, TS_GPIO_IRQ);
			goto vreg_get_fail;
		}

		/* virtual keys */
		tma300_vkeys_attr.attr.name = "virtualkeys.msm_tma300_ts";
		properties_kobj = kobject_create_and_add("board_properties",
					NULL);
		if (!properties_kobj) {
			pr_err("%s: failed to create a kobject"
					"for board_properites\n", __func__);
			rc = -ENOMEM;
			goto vreg_get_fail;
		}
		rc = sysfs_create_group(properties_kobj,
				&tma300_properties_attr_group);
		if (rc) {
			pr_err("%s: failed to create a sysfs entry %s\n",
					__func__, tma300_vkeys_attr.attr.name);
			kobject_put(properties_kobj);
			goto vreg_get_fail;
		}
	} else {
		/* put voltage sources */
		for (i = 0; i < ARRAY_SIZE(vregs_tma300_name); i++)
			vreg_put(vregs_tma300[i]);
		/* destroy virtual keys */
		if (properties_kobj) {
			sysfs_remove_group(properties_kobj,
				&tma300_properties_attr_group);
			kobject_put(properties_kobj);
		}
	}
	return 0;

vreg_get_fail:
	while (i)
		vreg_put(vregs_tma300[--i]);
	return rc;
}

static struct cy8c_ts_platform_data cy8ctma300_pdata = {
	.power_on = tma300_power,
	.dev_setup = tma300_dev_setup,
	.ts_name = "msm_tma300_ts",
	.dis_min_x = 0,
	.dis_max_x = 479,
	.dis_min_y = 0,
	.dis_max_y = 799,
	.res_x	 = 479,
	.res_y	 = 1009,
	.min_tid = 1,
	.max_tid = 255,
	.min_touch = 0,
	.max_touch = 255,
	.min_width = 0,
	.max_width = 255,
	.invert_y = 1,
	.nfingers = 4,
	.irq_gpio = TS_GPIO_IRQ,
	.resout_gpio = -1,
};

static struct i2c_board_info cy8ctma300_board_info[] = {
	{
		I2C_BOARD_INFO("cy8ctma300", 0x2),
		.platform_data = &cy8ctma300_pdata,
	}
};
/* < DTS2010090300591 luojianhong 201000903 begin*/
#ifdef CONFIG_HUAWEI_U8800PP1_WIFI_LOW_CONSUME
/* < DTS2010090700733 luojianhong 201000908 begin*/
/*delete other powers except "wlan2"*/
void __init config_wifi_for_low_consume(void)
{
	
	int rc;
	struct vreg *vreg_wlan2 = NULL;
	vreg_wlan2 = vreg_get(NULL, "wlan2");
	if (IS_ERR(vreg_wlan2)) {
		printk(KERN_ERR "%s: vreg_get failed wlan2\n",
			__func__);
		return;
	}

	// Power up 2.5v Analog
	rc = vreg_set_level(vreg_wlan2, 2500);
	rc = vreg_enable(vreg_wlan2);
	
	msleep(10);

   //shut down this power after WIFI is initialised.   

	rc = vreg_disable(vreg_wlan2);
	if (rc) {
		printk(KERN_ERR "%s: vreg_disable failed vreg_wlan2\n",
			__func__);
	}
}
/*  DTS2010090700733 luojianhong 201000908 end > */
#endif
/*  DTS2010090300591 luojianhong 201000903 end > */

#ifdef CONFIG_USB_AUTO_INSTALL
/* provide a method to map pid_index to usb_pid, 
 * pid_index is kept in NV(4526). 
 * At power up, pid_index is read in modem and transfer to app in share memory.
 * pid_index can be modified through write file fixusb(msm_hsusb_store_fixusb).
*/
u16 pid_index_to_pid(u32 pid_index)
{
    u16 usb_pid = 0xFFFF;
    
    switch(pid_index)
    {
        case CDROM_INDEX:
        /*< DTS2011062501797 zhangyancun 20110728 begin */
        /* merge DTS2011050604512 */
        case SLATE_TEST_INDEX:
        /* DTS2011062501797 zhangyancun 20110728 end >*/ 
            usb_pid = curr_usb_pid_ptr->cdrom_pid;
            break;
        case NORM_INDEX:
            usb_pid = curr_usb_pid_ptr->norm_pid;
            break;
        case AUTH_INDEX:
            usb_pid = curr_usb_pid_ptr->auth_pid;
            break;
        case GOOGLE_INDEX:
            usb_pid = curr_usb_pid_ptr->google_pid;
            break;
        
        case GOOGLE_WLAN_INDEX:
            usb_pid = curr_usb_pid_ptr->wlan_pid;
        /* set the USB pid to multiport when the index is 0
           This is happened when the NV is not set or set 
           to zero 
        */
        case ORI_INDEX:
        default:
            usb_pid = curr_usb_pid_ptr->norm_pid;
            break;
    }

    USB_PR("lxy: %s, pid_index=%d, usb_pid=0x%x\n", __func__, pid_index, usb_pid);
    return usb_pid;
}

void set_usb_device_name(void)
{
  memset(manufacturer_name, 0, MAX_NAME_LEN);
  memset(product_name, 0, MAX_NAME_LEN);
  memset(vendor_name, 0, MAX_NAME_LEN);
  
  strcpy(manufacturer_name, "Huawei Incorporated");
  /*< DTS2011082402558 zhangyancun 20110824 begin */
  /* modify the product name of USB */
  strcpy(product_name, "Android Adapter");
  /* DTS2011082402558 zhangyancun 20110824 end >*/
}

/* add new pid config for google */
void set_usb_pid_sn(u32 pid_index)
{
    switch(pid_index)
    {
        /* new requirement: usb tethering */
        case GOOGLE_WLAN_INDEX:
            USB_PR("lxy: set pid=0x%x, sn=NULL\n", GOOGLE_WLAN_INDEX);
            android_set_product_id(PID_WLAN);
            set_usb_sn(NULL);
            break;
        case GOOGLE_INDEX:
            USB_PR("lxy: set pid=0x%x, sn=%s\n", PID_GOOGLE_MS, usb_para_data.usb_para.usb_serial);
            android_set_product_id(PID_GOOGLE_MS);
            set_usb_sn(usb_para_data.usb_para.usb_serial);
            break;
            
        case NORM_INDEX:
            USB_PR("lxy: set pid=0x%x, sn=%s\n", curr_usb_pid_ptr->norm_pid, USB_SN_STRING);
            android_set_product_id(curr_usb_pid_ptr->norm_pid);
            set_usb_sn(USB_SN_STRING);
            break;
        /*< DTS2011062501797 zhangyancun 20110728 begin */
        /* merge DTS2011050604512 */
        case SLATE_TEST_INDEX:
        /* DTS2011062501797 zhangyancun 20110728 end >*/ 
        case CDROM_INDEX:
            USB_PR("lxy: set pid=0x%x, sn=%s\n", curr_usb_pid_ptr->cdrom_pid, "");
            android_set_product_id(curr_usb_pid_ptr->cdrom_pid);
            set_usb_sn(NULL);
            break;
            
        case ORI_INDEX:
            USB_PR("lxy: set pid=0x%x, sn=%s\n", curr_usb_pid_ptr->norm_pid, "");
            android_set_product_id(curr_usb_pid_ptr->norm_pid);
            set_usb_sn(NULL);
            break;
            
        case AUTH_INDEX:
            USB_PR("lxy: set pid=0x%x, sn=%s\n", curr_usb_pid_ptr->auth_pid, "");
            android_set_product_id(curr_usb_pid_ptr->auth_pid);
            set_usb_sn(NULL);
            break;
            
        default:
            USB_PR("lxy: set pid=0x%x, sn=%s\n", curr_usb_pid_ptr->norm_pid, "");
            android_set_product_id(curr_usb_pid_ptr->norm_pid);
            set_usb_sn(NULL);
            break;
    }

}


/*  
 * Get usb parameter from share memory and set usb serial number accordingly.
 */
static void proc_usb_para(void)
{
    smem_huawei_vender *usb_para_ptr;
    char *vender_name="t-mobile";

    USB_PR("lxy: < %s\n", __func__);

    /* initialize */
    usb_para_info.usb_pid_index = 0;
    usb_para_info.usb_pid = PID_NORMAL;

    set_usb_device_name();
    
    /* now the smem_id_vendor0 smem id is a new struct */
    usb_para_ptr = (smem_huawei_vender*)smem_alloc(SMEM_ID_VENDOR0, sizeof(smem_huawei_vender));
    if (!usb_para_ptr)
    {
      USB_PR("lxy: %s: Can't find usb parameter\n", __func__);
        return;
    }

    USB_PR("lxy: vendor:%s,country:%s\n", usb_para_ptr->vender_para.vender_name, usb_para_ptr->vender_para.
country_name);

    memcpy(&usb_para_data, usb_para_ptr, sizeof(smem_huawei_vender));
    
    /* decide usb pid array according to the vender name */
    if(!memcmp(usb_para_ptr->vender_para.vender_name, vender_name, strlen(vender_name)))
    {
        curr_usb_pid_ptr = &usb_pid_array[1];
        USB_PR("lxy: USB setting is TMO\n");
    }
    else
    {
        curr_usb_pid_ptr = &usb_pid_array[0];
        USB_PR("lxy: USB setting is NORMAL\n");
    }

    USB_PR("lxy: smem usb_serial=%s, usb_pid_index=%d\n", usb_para_ptr->usb_para.usb_serial, usb_para_ptr->usb_para.
usb_pid_index);

    /* when manufacture, we need to use the diag. so if the usb_serial is null
       and the nv value is google index, we set the ports to normal.
    */
    if (0 == usb_para_data.usb_para.usb_serial[0] 
      && GOOGLE_INDEX == usb_para_ptr->usb_para.usb_pid_index)
    {
      USB_PR("%s usb serial number is null in google mode. so switch to original mode\n", __func__);
      usb_para_ptr->usb_para.usb_pid_index = ORI_INDEX;
    }

    usb_para_info.usb_pid_index = usb_para_ptr->usb_para.usb_pid_index;
    
    usb_para_info.usb_pid = pid_index_to_pid(usb_para_ptr->usb_para.usb_pid_index);

    /* add new pid config for google */
    set_usb_pid_sn(usb_para_info.usb_pid_index);
    
    /* < DTS2011092003743 liwei 20111006 begin */
    /* the vendor emobile in japan doesn't need cdrom driver */
    if ((0 == memcmp(usb_para_data.vender_para.vender_name, VENDOR_EMOBILE, strlen(VENDOR_EMOBILE)))
        && (0 == memcmp(usb_para_data.vender_para.country_name, COUNTRY_JAPAN, strlen(COUNTRY_JAPAN))))
    {
        u16 index = 0;
		
        /* find the google mode from the products array */ 
        for (; index < android_usb_pdata.num_products; index++)
        {
            if (PID_GOOGLE == android_usb_pdata.products[index].adb_product_id)
            {
                /* set the nluns to 2 from 3, del the cdrom */
                android_usb_pdata.products[index].nluns = 2;         /* 2 means two udisk */
                android_usb_pdata.products[index].cdrom_index = -1;  /* -1 means not support cdrom */

                USB_PR("%s %s%s doesn't need the cdrom. nluns=%d, cdrom_index=%d\n",
                       __func__, 
					   usb_para_data.vender_para.country_name, 
					   usb_para_data.vender_para.vender_name,
                       android_usb_pdata.products[index].nluns, 
					   android_usb_pdata.products[index].cdrom_index);
            }  
        }  
    }
    /* DTS2011092003743 liwei 20111006 end > */
	
    USB_PR("lxy: curr_usb_pid_ptr: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n", 
        curr_usb_pid_ptr->cdrom_pid, 
        curr_usb_pid_ptr->norm_pid, 
        curr_usb_pid_ptr->udisk_pid,
        curr_usb_pid_ptr->auth_pid,
        curr_usb_pid_ptr->google_pid);
    USB_PR("lxy: usb_para_info: usb_pid_index=%d, usb_pid = 0x%x>\n", 
        usb_para_info.usb_pid_index, 
        usb_para_info.usb_pid);
}
#endif  /* CONFIG_USB_AUTO_INSTALL */
/* BU5D00386 lixiangyu 20100105 end > */


static void __init msm7x30_init(void)
{
	int rc;
	unsigned smem_size;
	uint32_t usb_hub_gpio_cfg_value = GPIO_CFG(56,
						0,
						GPIO_CFG_OUTPUT,
						GPIO_CFG_NO_PULL,
						GPIO_CFG_2MA);
	uint32_t soc_version = 0;

	if (socinfo_init() < 0)
		printk(KERN_ERR "%s: socinfo_init() failed!\n",
		       __func__);

	soc_version = socinfo_get_version();

	msm_clock_init(msm_clocks_7x30, msm_num_clocks_7x30);
#ifdef CONFIG_SERIAL_MSM_CONSOLE
	msm7x30_init_uart2();
#endif
	msm_spm_init(&msm_spm_data, 1);
	msm_acpu_clock_init(&msm7x30_clock_data);
	/* <DTS2010072202961 hufeng 20100722 begin */
	/* removed several lines */
	/* DTS2010072202961 hufeng 20100722 end> */
#ifdef CONFIG_USB_FUNCTION
	msm_hsusb_pdata.swfi_latency =
		msm_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_hsusb_peripheral.dev.platform_data = &msm_hsusb_pdata;
#endif

#ifdef CONFIG_USB_AUTO_INSTALL
    proc_usb_para();
#endif  /* #ifdef CONFIG_USB_AUTO_INSTALL */

#ifdef CONFIG_USB_MSM_OTG_72K
	if (SOCINFO_VERSION_MAJOR(soc_version) >= 2 &&
			SOCINFO_VERSION_MINOR(soc_version) >= 1) {
		pr_debug("%s: SOC Version:2.(1 or more)\n", __func__);
		msm_otg_pdata.ldo_set_voltage = 0;
	}

	msm_device_otg.dev.platform_data = &msm_otg_pdata;
#ifdef CONFIG_USB_GADGET
	msm_otg_pdata.swfi_latency =
 	msm_pm_data
 	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;
#endif
#endif
/* < DTS2011041200623 xuhui 20110412 begin */
/*disable QC's In Band Sleep mode with BCM4329 bluetooth chip*/
#ifdef CONFIG_HUAWEI_KERNEL
	msm_uart_dm1_pdata.wakeup_irq = gpio_to_irq(136);
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
#endif
/* DTS2011041200623 xuhui 20110412 end > */
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)
	msm_device_tsif.dev.platform_data = &tsif_platform_data;
#endif
	if (machine_is_msm7x30_fluid()) {
		msm_adc_pdata.dev_names = msm_adc_fluid_device_names;
		msm_adc_pdata.num_adc = ARRAY_SIZE(msm_adc_fluid_device_names);
	} else {
		msm_adc_pdata.dev_names = msm_adc_surf_device_names;
		msm_adc_pdata.num_adc = ARRAY_SIZE(msm_adc_surf_device_names);
	}
#ifdef CONFIG_USB_ANDROID
#ifndef CONFIG_HUAWEI_KERNEL
	if (machine_is_msm8x55_svlte_surf() ||
		machine_is_msm8x55_svlte_ffa()) {
		android_usb_pdata.product_id = 0x9028;
		android_usb_pdata.num_products =
			ARRAY_SIZE(fusion_usb_products);
		android_usb_pdata.products = fusion_usb_products;
	}
#endif
#endif

	platform_add_devices(devices, ARRAY_SIZE(devices));
    /* <BU5D08126 duangan 2010-4-24 begin */
	#ifdef CONFIG_HUAWEI_FEATURE_OEMINFO
    rmt_oeminfo_add_device();
	#endif
    /* BU5D08126 duangan 2010-4-24 end> */

/* <DTS2010092002892 duangan 20100926 begin */
#ifdef CONFIG_HUAWEI_KERNEL
    hw_extern_sdcard_add_device();
#endif
/* DTS2010092002892 duangan 20100926 end> */
#ifdef CONFIG_USB_EHCI_MSM
	msm_add_host(0, &msm_usb_host_pdata);
#endif
	msm7x30_init_mmc();
	/* <DTS2010072202961 hufeng 20100722 begin */
	/* removed several lines */
	/* DTS2010072202961 hufeng 20100722 end> */
	msm_qsd_spi_init();


	msm_fb_add_devices();
	msm_pm_set_platform_data(msm_pm_data, ARRAY_SIZE(msm_pm_data));
	msm_device_i2c_init();
	msm_device_i2c_2_init();
	qup_device_i2c_init();
	buses_init();
	msm7x30_init_marimba();
/* BU5D08419 haoqingtao 20100424 end>*/    
#ifdef CONFIG_MSM7KV2_AUDIO
	snddev_poweramp_gpio_init();
	aux_pcm_gpio_init();
    /*<DTS2011060201308 yanghaimin 20110602, begin*/
    /* u8860 add hac gpio ctl */
    snddev_hac_gpio_init();
    /* DTS2011060201308 yanghaimin 20110602, end>*/
#endif
/*<BU5D07918, sibingsong 20100415 begin*/
#ifdef CONFIG_HUAWEI_FEATURE_VIBRATOR
	msm_init_pmic_vibrator();
#endif
/*BU5D07918, sibingsong 20100415 end>*/

	i2c_register_board_info(0, msm_i2c_board_info,
			ARRAY_SIZE(msm_i2c_board_info));

	if (!machine_is_msm8x55_svlte_ffa())
		marimba_pdata.tsadc = &marimba_tsadc_pdata;

	if (machine_is_msm7x30_fluid())
		i2c_register_board_info(0, cy8info,
					ARRAY_SIZE(cy8info));

	i2c_register_board_info(2, msm_marimba_board_info,
			ARRAY_SIZE(msm_marimba_board_info));

	i2c_register_board_info(2, msm_i2c_gsbi7_timpani_info,
			ARRAY_SIZE(msm_i2c_gsbi7_timpani_info));

	i2c_register_board_info(4 /* QUP ID */, msm_camera_boardinfo,
				ARRAY_SIZE(msm_camera_boardinfo));
/* < DTS2010092501047 luojianhong 201000926 begin*/
#ifdef CONFIG_HUAWEI_FEATURE_RIGHT_TPA2028D1_AMPLIFIER
	i2c_register_board_info(4 /* QUP ID */, msm_amplifier_boardinfo,
				ARRAY_SIZE(msm_amplifier_boardinfo));
#endif
/*  DTS2010092501047 luojianhong 201000926 end > */
/*< DTS2011012004291 xuhui 20110120 begin */
#ifndef CONFIG_HUAWEI_KERNEL
        bt_bcm4329_power_init();
#else
	bt_power_init();
#endif 
/* DTS2011012004291 xuhui 20110120 end >*/
#ifdef CONFIG_I2C_SSBI
	msm_device_ssbi6.dev.platform_data = &msm_i2c_ssbi6_pdata;
	msm_device_ssbi7.dev.platform_data = &msm_i2c_ssbi7_pdata;
#endif
	/*< DTS2010112702297 wangquanli 201001125 begin */
	/*< DTS2011030202729  liliang 20110302  begin */	
	if (machine_is_msm7x30_fluid() || (machine_is_msm7x30_u8800()) || (machine_is_msm7x30_u8800_51()) || (machine_is_msm8255_u8800_pro())) 
	/* DTS2011030202729  liliang 20110302 end >*/
	/* DTS2010112702297 wangquanli 201001125 end >*/
		i2c_register_board_info(0, msm_isa1200_board_info,
			ARRAY_SIZE(msm_isa1200_board_info));

/*< DTS2011091402372   yuguangcai 20110914 begin */
/*Register i2c information for flash tps61310*/
#ifdef CONFIG_HUAWEI_FEATURE_TPS61310
	i2c_register_board_info(0, tps61310_board_info,
			ARRAY_SIZE(tps61310_board_info));
#endif
/* DTS2011091402372   yuguangcai 20110914 end > */
#if defined(CONFIG_TOUCHSCREEN_TSC2007) || \
	defined(CONFIG_TOUCHSCREEN_TSC2007_MODULE)
	if (machine_is_msm8x55_svlte_ffa())
		i2c_register_board_info(2, tsc_i2c_board_info,
				ARRAY_SIZE(tsc_i2c_board_info));
#endif

	if (machine_is_msm7x30_surf())
		platform_device_register(&flip_switch_device);
	pmic8058_leds_init();

	if (machine_is_msm7x30_fluid()) {
		/* Initialize platform data for fluid v2 hardware */
		if (SOCINFO_VERSION_MAJOR(
				socinfo_get_platform_version()) == 2) {
			cy8ctma300_pdata.res_y = 920;
			cy8ctma300_pdata.invert_y = 0;
		}
		i2c_register_board_info(0, cy8ctma300_board_info,
			ARRAY_SIZE(cy8ctma300_board_info));
	}
	/* < DTS2010090300591 luojianhong 201000903 begin*/
	#ifdef CONFIG_HUAWEI_U8800PP1_WIFI_LOW_CONSUME
	config_wifi_for_low_consume();
	#endif
	/*  DTS2010090300591 luojianhong 201000903 end > */

	if (machine_is_msm8x55_svlte_surf() || machine_is_msm8x55_svlte_ffa()) {
		rc = gpio_tlmm_config(usb_hub_gpio_cfg_value, GPIO_CFG_ENABLE);
		if (rc)
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, usb_hub_gpio_cfg_value, rc);
	}

	boot_reason = *(unsigned int *)
		(smem_get_entry(SMEM_POWER_ON_STATUS_INFO, &smem_size));
	printk(KERN_NOTICE "Boot Reason = 0x%02x\n", boot_reason);
	/*<DTS2011042602009 fengwei 20110426 begin*/
#ifdef CONFIG_HUAWEI_KERNEL
       virtualkeys_init();
#endif
	/*DTS2011042602009 fengwei 20110426 end>*/
/*< DTS2011093001643 lijianzhao 20110930 begin */
#ifdef CONFIG_HUAWEI_KERNEL
	add_slide_detect_device();
#endif
/* DTS2011093001643 lijianzhao 20110930 end >*/
}

static unsigned pmem_sf_size = MSM_PMEM_SF_SIZE;
static int __init pmem_sf_size_setup(char *p)
{
	pmem_sf_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_sf_size", pmem_sf_size_setup);

static unsigned fb_size = MSM_FB_SIZE;
static int __init fb_size_setup(char *p)
{
	fb_size = memparse(p, NULL);
	return 0;
}
early_param("fb_size", fb_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static int __init pmem_adsp_size_setup(char *p)
{
	pmem_adsp_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_adsp_size", pmem_adsp_size_setup);

static unsigned fluid_pmem_adsp_size = MSM_FLUID_PMEM_ADSP_SIZE;
static int __init fluid_pmem_adsp_size_setup(char *p)
{
	fluid_pmem_adsp_size = memparse(p, NULL);
	return 0;
}
early_param("fluid_pmem_adsp_size", fluid_pmem_adsp_size_setup);

static unsigned pmem_audio_size = MSM_PMEM_AUDIO_SIZE;
static int __init pmem_audio_size_setup(char *p)
{
	pmem_audio_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_audio_size", pmem_audio_size_setup);

static unsigned pmem_kernel_ebi1_size = PMEM_KERNEL_EBI1_SIZE;
static int __init pmem_kernel_ebi1_size_setup(char *p)
{
	pmem_kernel_ebi1_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_kernel_ebi1_size", pmem_kernel_ebi1_size_setup);

static void __init msm7x30_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;
/*
   Request allocation of Hardware accessible PMEM regions
   at the beginning to make sure they are allocated in EBI-0.
   This will allow 7x30 with two mem banks enter the second
   mem bank into Self-Refresh State during Idle Power Collapse.

    The current HW accessible PMEM regions are
    1. Frame Buffer.
       LCDC HW can access msm_fb_resources during Idle-PC.

    2. Audio
       LPA HW can access android_pmem_audio_pdata during Idle-PC.
*/
	size = fb_size ? : MSM_FB_SIZE;
	addr = alloc_bootmem(size);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb\n",
		size, addr, __pa(addr));

	size = pmem_audio_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_audio_pdata.start = __pa(addr);
		android_pmem_audio_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for audio "
			"pmem arena\n", size, addr, __pa(addr));
	}

	size = pmem_kernel_ebi1_size;
	if (size) {
		addr = alloc_bootmem_aligned(size, 0x100000);
		android_pmem_kernel_ebi1_pdata.start = __pa(addr);
		android_pmem_kernel_ebi1_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for kernel"
			" ebi1 pmem arena\n", size, addr, __pa(addr));
	}

	size = pmem_sf_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_pdata.start = __pa(addr);
		android_pmem_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for sf "
			"pmem arena\n", size, addr, __pa(addr));
	}

	if machine_is_msm7x30_fluid()
		size = fluid_pmem_adsp_size;
	else
		size = pmem_adsp_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_adsp_pdata.start = __pa(addr);
		android_pmem_adsp_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for adsp "
			"pmem arena\n", size, addr, __pa(addr));
	}
}

static void __init msm7x30_map_io(void)
{
	msm_shared_ram_phys = 0x00100000;
	msm_map_msm7x30_io();
	msm7x30_allocate_memory_regions();
}

/*< BU5D08419 haoqingtao 20100424 begin*/    
#define ATAG_CAMERA_ID 0x4d534D74
/* setup calls mach->fixup, then parse_tags, parse_cmdline
 * We need to setup meminfo in mach->fixup, so this function
 * will need to traverse each tag to find smi tag.
 */
int __init parse_tag_camera_id(const struct tag *tags)
{
    int camera_id = 0, find = 0;

	struct tag *t = (struct tag *)tags;

	for (; t->hdr.size; t = tag_next(t)) {
		if (t->hdr.tag == ATAG_CAMERA_ID) {
			find = 1;
			break;
		}
	}
	if (find)
		camera_id = t->u.revision.rev;
	return camera_id;
}
__tagtable(ATAG_CAMERA_ID, parse_tag_camera_id);

#define ATAG_LCD_ID 0x4d534D73
int __init parse_tag_lcd_id(const struct tag *tags)
{
    int lcd_id = 0, find = 0;
	struct tag *t = (struct tag *)tags;

	for (; t->hdr.size; t = tag_next(t)) {
		if (t->hdr.tag == ATAG_LCD_ID) {
			find = 1;
			break;
		}
	}
	if (find)
		lcd_id = t->u.revision.rev;
	return lcd_id;

}
__tagtable(ATAG_LCD_ID, parse_tag_lcd_id);

#define ATAG_TS_ID 0x4d534D75
int __init parse_tag_ts_id(const struct tag *tags)
{
    int ts_id = 0, find = 0;
	struct tag *t = (struct tag *)tags;

	for (; t->hdr.size; t = tag_next(t)) {
		if (t->hdr.tag == ATAG_TS_ID) {
			find = 1;
			break;
		}
	}
	if (find)
		ts_id = t->u.revision.rev;
	return ts_id;

}
__tagtable(ATAG_TS_ID, parse_tag_ts_id);

/* <DTS2010071502918 shenjinming 20101008 begin */
#ifdef CONFIG_HUAWEI_POWER_DOWN_CHARGE
#define ATAG_CHARGE_FLAG  0x4d534D77
int __init parse_tag_charge_flag(const struct tag *tags)
{
    int charge_flag = 0, find = 0;
	struct tag *t = (struct tag *)tags;

	for (; t->hdr.size; t = tag_next(t)) {
		if (t->hdr.tag == ATAG_CHARGE_FLAG) {
			find = 1;
			break;
		}
	}
	if (find)
		charge_flag = t->u.revision.rev;
	return charge_flag;

}
__tagtable(ATAG_CHARGE_FLAG, parse_tag_charge_flag);
#endif
/* DTS2010071502918 shenjinming 20101008 end> */ 
#define ATAG_SUB_BOARD_ID 0x4d534D76
int __init parse_tag_sub_board_id(const struct tag *tags)
{
    int sub_board_id = 0, find = 0;
	struct tag *t = (struct tag *)tags;

	for (; t->hdr.size; t = tag_next(t)) {
		if (t->hdr.tag == ATAG_SUB_BOARD_ID) {
			find = 1;
			break;
		}
	}
	if (find)
		sub_board_id = t->u.revision.rev;
	return sub_board_id;

}
__tagtable(ATAG_SUB_BOARD_ID, parse_tag_sub_board_id);
static void __init msm7x30_fixup(struct machine_desc *desc,
                                 struct tag *tags,
                                 char **cmdline,
                                 struct meminfo *mi)
{
    camera_id = parse_tag_camera_id((const struct tag *)tags);
    printk("%s:camera_id=%d\n", __func__, camera_id);
        
    lcd_id = parse_tag_lcd_id((const struct tag *)tags);
    printk("%s:lcd_id=%d\n", __func__, lcd_id);

    ts_id = parse_tag_ts_id((const struct tag *)tags);
    printk("%s:ts_id=%d\n", __func__, ts_id);

    sub_board_id = parse_tag_sub_board_id((const struct tag *)tags);
    printk("%s:sub_board_id=%d\n", __func__, sub_board_id);

    /* <DTS2010071502918 shenjinming 20101008 begin */
#ifdef CONFIG_HUAWEI_POWER_DOWN_CHARGE    
    charge_flag = parse_tag_charge_flag((const struct tag *)tags);
    printk("%s:charge_flag=%d\n", __func__, charge_flag);
#endif
    /* DTS2010071502918 shenjinming 20101008 end> */ 
    
}

hw_ver_sub_type get_hw_sub_board_id(void)
{
    return (hw_ver_sub_type)(sub_board_id&HW_VER_SUB_MASK);
}

/* <DTS2010071502918 shenjinming 20101008 begin */
#ifdef CONFIG_HUAWEI_POWER_DOWN_CHARGE
unsigned int get_charge_flag(void)
{
    return charge_flag;
}
#endif
/* DTS2010071502918 shenjinming 20101008 end> */ 

MACHINE_START(MSM7X30_SURF, "QCT MSM7X30 SURF")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io  = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params = PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
MACHINE_END

MACHINE_START(MSM7X30_FFA, "QCT MSM7X30 FFA")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io  = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params = PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
MACHINE_END

MACHINE_START(MSM7X30_FLUID, "QCT MSM7X30 FLUID")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io  = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params = PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
MACHINE_END

MACHINE_START(MSM8X55_SURF, "QCT MSM8X55 SURF")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io  = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params = PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
MACHINE_END

MACHINE_START(MSM8X55_FFA, "QCT MSM8X55 FFA")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io  = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params = PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
MACHINE_END
MACHINE_START(MSM8X55_SVLTE_SURF, "QCT MSM8X55 SVLTE SURF")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io  = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params = PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
MACHINE_END
MACHINE_START(MSM8X55_SVLTE_FFA, "QCT MSM8X55 SVLTE FFA")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io  = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params = PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
MACHINE_END

MACHINE_START(MSM7X30_U8800, "HUAWEI U8800 BOARD")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io  = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params = PHYS_OFFSET + 0x100,
	.fixup = msm7x30_fixup,
	.map_io = msm7x30_map_io,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
MACHINE_END
/*< DTS2010092400487  lijianzhao 20100924 begin */
MACHINE_START(MSM7X30_U8820, "HUAWEI U8820 BOARD")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io  = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params = PHYS_OFFSET + 0x100,
	.fixup = msm7x30_fixup,
	.map_io = msm7x30_map_io,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
MACHINE_END
/*< DTS2010112702297 wangquanli 201001125 begin */
MACHINE_START(MSM7X30_U8800_51, "HUAWEI U8800-51 BOARD")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io  = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params = PHYS_OFFSET + 0x100,
	.fixup = msm7x30_fixup,
	.map_io = msm7x30_map_io,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
MACHINE_END
/* DTS2010112702297 wangquanli 201001125 end >*/
/*< DTS2011030202729  liliang 20110302  begin */
MACHINE_START(MSM8255_U8800_PRO, "HUAWEI U8800-PRO BOARD")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io  = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params = PHYS_OFFSET + 0x100,
	.fixup = msm7x30_fixup,
	.map_io = msm7x30_map_io,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
MACHINE_END
/* DTS2011030202729  liliang 20110302 end >*/
/* DTS2010092400487  lijianzhao 20100924 end >*/ 
/* BU5D08419 haoqingtao 20100424 end>*/    

/* <DTS2011041501614 duangan 2011-4-18 begin */
MACHINE_START(MSM8255_U8860, "HUAWEI U8860 BOARD")
#ifdef CONFIG_MSM_DEBUG_UART
    .phys_io  = MSM_DEBUG_UART_PHYS,
    .io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
    .boot_params = PHYS_OFFSET + 0x100,
    .fixup = msm7x30_fixup,
    .map_io = msm7x30_map_io,
    .init_irq = msm7x30_init_irq,
    .init_machine = msm7x30_init,
    .timer = &msm_timer,
MACHINE_END
/* DTS2011041501614 duangan 2011-4-18 end> */

/* <DTS2011050700551 zhangbo 20110505 begin */
MACHINE_START(MSM8255_C8860, "HUAWEI C8860 BOARD")
#ifdef CONFIG_MSM_DEBUG_UART
    .phys_io  = MSM_DEBUG_UART_PHYS,
    .io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
    .boot_params = PHYS_OFFSET + 0x100,
    .fixup = msm7x30_fixup,
    .map_io = msm7x30_map_io,
    .init_irq = msm7x30_init_irq,
    .init_machine = msm7x30_init,
    .timer = &msm_timer,
MACHINE_END
/* DTS2011050700551 zhangbo 20110505 end> */
/* <DTS2011062600102 sunhonghui 20110626 begin */
MACHINE_START(MSM8255_U8860LP, "HUAWEI U8860LP BOARD")
#ifdef CONFIG_MSM_DEBUG_UART
    .phys_io  = MSM_DEBUG_UART_PHYS,
    .io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
    .boot_params = PHYS_OFFSET + 0x100,
    .fixup = msm7x30_fixup,
    .map_io = msm7x30_map_io,
    .init_irq = msm7x30_init_irq,
    .init_machine = msm7x30_init,
    .timer = &msm_timer,
MACHINE_END
/* DTS2011062600102 sunhonghui 20110626 end> */
/* <DTS2011071600361 liyuping 20110716 begin */
MACHINE_START(MSM8255_U8860_92, "HUAWEI U8860_92 BOARD")
#ifdef CONFIG_MSM_DEBUG_UART
    .phys_io  = MSM_DEBUG_UART_PHYS,
    .io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
    .boot_params = PHYS_OFFSET + 0x100,
    .fixup = msm7x30_fixup,
    .map_io = msm7x30_map_io,
    .init_irq = msm7x30_init_irq,
    .init_machine = msm7x30_init,
    .timer = &msm_timer,
MACHINE_END
/* DTS2011071600361 liyuping 20110716 end> */
/* < DTS2011082302564 liwei 20110823 begin */
MACHINE_START(MSM8255_U8680, "HUAWEI U8680 BOARD")
#ifdef CONFIG_MSM_DEBUG_UART
    .phys_io  = MSM_DEBUG_UART_PHYS,
    .io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
    .boot_params = PHYS_OFFSET + 0x100,
    .fixup = msm7x30_fixup,
    .map_io = msm7x30_map_io,
    .init_irq = msm7x30_init_irq,
    .init_machine = msm7x30_init,
    .timer = &msm_timer,
MACHINE_END
/* DTS2011082302564 liwei 20110823 end > */
/* < DTS2011102401822 liwei 20111024 begin */
MACHINE_START(MSM8255_U8667, "HUAWEI U8667 BOARD")
#ifdef CONFIG_MSM_DEBUG_UART
    .phys_io  = MSM_DEBUG_UART_PHYS,
    .io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
    .boot_params = PHYS_OFFSET + 0x100,
    .fixup = msm7x30_fixup,
    .map_io = msm7x30_map_io,
    .init_irq = msm7x30_init_irq,
    .init_machine = msm7x30_init,
    .timer = &msm_timer,
MACHINE_END
/* DTS2011102401822 liwei 20111024 end > */
/* <DTS2011091200073 zhangbo 20110912 begin */
MACHINE_START(MSM8255_U8730, "HUAWEI U8730 BOARD")
#ifdef CONFIG_MSM_DEBUG_UART
    .phys_io  = MSM_DEBUG_UART_PHYS,
    .io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
    .boot_params = PHYS_OFFSET + 0x100,
    .fixup = msm7x30_fixup,
    .map_io = msm7x30_map_io,
    .init_irq = msm7x30_init_irq,
    .init_machine = msm7x30_init,
    .timer = &msm_timer,
MACHINE_END
/* DTS2011091200073 zhangbo 20110912 end> */
/*<DTS2011091502092 liyuping 20110915 begin */
MACHINE_START(MSM8255_U8860_51, "HUAWEI U8860_51 BOARD")
#ifdef CONFIG_MSM_DEBUG_UART
    .phys_io  = MSM_DEBUG_UART_PHYS,
    .io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
    .boot_params = PHYS_OFFSET + 0x100,
    .fixup = msm7x30_fixup,
    .map_io = msm7x30_map_io,
    .init_irq = msm7x30_init_irq,
    .init_machine = msm7x30_init,
    .timer = &msm_timer,
MACHINE_END
/* DTS2011091502092 liyuping 20110915 end> */ 


