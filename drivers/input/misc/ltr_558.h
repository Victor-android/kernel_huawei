/* < DTS2011102806486  zhangmin 20111029  begin */
/* drivers/input/misc/ltr_558.c
 * Copyright (C) 2011 HUAWEI, Inc.
 * Author: zhangmin/195861 
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef _LTR558_H
#define _LTR558_H
/* LTR-558 Registers */
#define LTR558_ALS_CONTR 	        0x80
#define LTR558_PS_CONTR 	        0x81
#define LTR558_PS_LED 		        0x82
#define LTR558_PS_N_PULSES 	    0x83
#define LTR558_PS_MEAS_RATE 	    0x84
#define LTR558_ALS_MEAS_RATE 	    0x85
#define LTR558_PART_ID             0x86
#define LTR558_MANUFACTURER_ID 	0x87
#define LTR558_INTERRUPT 	        0x8F
#define LTR558_PS_THRES_UP_0 	    0x90
#define LTR558_PS_THRES_UP_1 	    0x91
#define LTR558_PS_THRES_LOW_0 	0x92
#define LTR558_PS_THRES_LOW_1 	0x93
#define LTR558_ALS_THRES_UP_0 	0x97
#define LTR558_ALS_THRES_UP_1 	0x98
#define LTR558_ALS_THRES_LOW_0	0x99
#define LTR558_ALS_THRES_LOW_1 	0x9A
#define LTR558_INTERRUPT_PERSIST  0x9E
/* 558's Read Only Registers */
#define LTR558_ALS_DATA_CH1_0	    0x88
#define LTR558_ALS_DATA_CH1_1 	0x89
#define LTR558_ALS_DATA_CH0_0 	0x8A
#define LTR558_ALS_DATA_CH0_1 	0x8B
#define LTR558_ALS_PS_STATUS 	    0x8C
#define LTR558_PS_DATA_0 	        0x8D
#define LTR558_PS_DATA_1 	        0x8E
/* Basic Operating Modes */
#define MODE_ALS_ON_Range1 	    0x0B
#define MODE_ALS_ON_Range2 	    0x03
#define MODE_ALS_StdBy 		    0x00
#define MODE_PS_ON_Gain1 	        0x03
#define MODE_PS_ON_Gain4 	        0x07
#define MODE_PS_ON_Gain8 	        0x0B
#define MODE_PS_ON_Gain16 	        0x0F
#define MODE_PS_StdBy 		        0x00

#define PS_RANGE1   1
#define PS_RANGE4   2
#define PS_RANGE8   3
#define PS_RANGE16  4
#define ALS_RANGE1_320 1
#define ALS_RANGE2_64K 2

#define PS_DATA_BIT     1
#define PS_INIT         2
#define ALS_DATA_BIT    4
#define DATA_STATE      5
#define ALS_INIT        8
#define INT_STATE       10
#define ALS_GAIN  25

/* IOCTLs for ltr558 device */
#define ECS_IOCTL_APP_SET_DELAY	    _IOW(0xA1, 0x18, short)
#define ECS_IOCTL_APP_GET_DELAY       _IOR(0xA1, 0x30, short)
#define ECS_IOCTL_APP_SET_LFLAG		_IOW(0xA1, 0x1C, short)
#define ECS_IOCTL_APP_GET_LFLAG		_IOR(0xA1, 0x1B, short)
#define ECS_IOCTL_APP_SET_PFLAG		_IOW(0xA1, 0x1E, short)
#define ECS_IOCTL_APP_GET_PFLAG		_IOR(0xA1, 0x1D, short)

/* Power On response time in ms */
#define PON_DELAY 400
#define WAKEUP_DELAY 10

#endif
/* DTS2011102806486  zhangmin 20111029  end > */
