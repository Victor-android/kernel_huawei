/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


#ifndef __MSM_BATTERY_H__
#define __MSM_BATTERY_H__

#define AC_CHG     0x00000001
#define USB_CHG    0x00000002

struct msm_psy_batt_pdata {
	u32 voltage_max_design;
	u32 voltage_min_design;
	u32 avail_chg_sources;
	u32 batt_technology;
	u32 (*calculate_capacity)(u32 voltage);
};

/* <DTS2010081400556 shenjinming 20100814 begin */
/*< DTS2010071902252 shenjinming 20100719 begin */
#ifdef CONFIG_HUAWEI_EVALUATE_POWER_CONSUMPTION 
typedef enum {   
   EVENT_LCD_STATE = 0, /*LCD*/
   EVENT_INS_CAMERA_STATE = 1, /*inside camera*/
   EVENT_OUTS_CAMERA_STATE = 2, /*outside camera*/
   EVENT_CAMERA_STATE = 3,     /*camera*/
   EVENT_WIFI_STATE = 4, /*WIFI status*/  
   EVENT_BT_STATE = 5,  /*blue tooth state*/      
   EVENT_FM_STATE = 6, /* FM */
   EVENT_CODEC_STATE = 7, /* AUDIO Speaker */  
   EVENT_CAMERA_FLASH_STATE = 8, /* Camera flash */  
 
   EVENT_KEYPAD_BACKLIGHT_STATE = 20,  /*KPD backlight*/
   EVENT_VIBRATOR_STATE,  /*vibrator state*/   
   EVENT_GSM850_GSM900_STATE,  /*GSM850\GSM900*/
   EVENT_GSM1800_GSM1900_STATE,  /*GSM1800\GSM1900*/
   EVENT_WCDMA_RF_STATE,  /*WCDMA*/
   EVENT_CDMA1X_RF_STATE,  /*WCDMA*/   
   EVENT_SPEAKER_STATE,  /*SPEAKER*/
   EVENT_CPU_STATE,/*CPU*/  
   EVENT_GPS_STATE,/*GPS*/      
   EVENT_HW_NONE = 0xFF
} device_current_consume_type;

#define DEVICE_POWER_STATE_OFF 0
#define DEVICE_POWER_STATE_ON 1
#define SPEAKER_ON_STATE  1
#define SPEAKER_OFF_STATE 0

/* notify modem sides to calculate consume */
int huawei_rpc_current_consuem_notify(device_current_consume_type device_event, __u32 device_state);
#endif
/* DTS2010071902252 shenjinming 20100719 end >*/
/* DTS2010081400556 shenjinming 20100814 end> */
#endif
