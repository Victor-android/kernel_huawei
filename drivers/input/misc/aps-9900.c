/* < DTS2011042604384  wangjiongfeng 20110427 begin */
/* drivers/input/misc/aps-12d.c
 *
 * Copyright (C) 2010 HUAWEI, Inc.
 * Author: Benjamin Gao <gaohuajiang@huawei.com>
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

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
/* < DTS2011050303216 wangjiongfeng 20110504 begin */
#include <linux/hardware_self_adapt.h>
/* < DTS2011050303216 wangjiongfeng 20110504 end */
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <mach/gpio.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include "aps-9900.h"
#include <asm/mach-types.h>
#include <linux/hardware_self_adapt.h>
#include <mach/vreg.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
/* < DTS2011052803160 shenjinming 20110611 begin */
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <linux/hw_dev_dec.h>
#endif
/* DTS2011052803160 shenjinming 201106011 end > */

#undef PROXIMITY_DB
#ifdef PROXIMITY_DB
#define PROXIMITY_DEBUG(fmt, args...) printk(KERN_INFO fmt, ##args)
#else
#define PROXIMITY_DEBUG(fmt, args...)
#endif

static struct workqueue_struct *aps_wq;

struct aps_data {
    uint16_t addr;
    struct i2c_client *client;
    struct input_dev *input_dev;
    struct mutex  mlock;
    struct hrtimer timer;
    struct work_struct  work;
    /* < DTS2011050303216 wangjiongfeng 20110504 begin */
	/* delete user_irq */
    /* < DTS2011050303216 wangjiongfeng 20110504 end */
    int (*power)(int on);
};

/* < DTS2011081101306  liujinggang 20110811 begin */
/* add lsensor level*/
#define LSENSOR_MAX_LEVEL 7
/* < DTS2011091700448  liujinggang 20110917 begin */
/* adjust the values of lsensor level*/
/* < DTS2011093001897  liujinggang 20110930 begin */
/* adjust the values for U8860 pp2*/
/* < DTS2011101000667 zhangyu 20111015 begin */
/* adjust the values for U8860 */
static uint16_t lsensor_adc_table[LSENSOR_MAX_LEVEL] = {
	8, 25, 110, 400, 750, 1200, 3000
};
/* DTS2011101000667 zhangyu 20111015 end > */
/* DTS2011093001897  liujinggang 20110930 end > */
/* DTS2011091700448  liujinggang 20110917 end > */
/* DTS2011081101306  liujinggang 20110811 end > */
/* < DTS2011052003350 wangjiongfeng 20110520 begin */
/* add the macro of log */
static int aps9900_debug_mask;
module_param_named(aps9900_debug, aps9900_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

#define APS9900_DBG(x...) do {\
    if (aps9900_debug_mask) \
        printk(KERN_DEBUG x);\
    } while (0)
/* DTS2011052003350 wangjiongfeng 20110520 end >*/

struct aps_init_regdata{
      int reg;
      uint8_t data;
};
static struct aps_data  *this_aps_data;

/* < DTS2011050303216 wangjiongfeng 20110504 begin */
struct input_dev *sensor_9900_dev=NULL;
/* < DTS2011050303216 wangjiongfeng 20110504 end */
static int aps_9900_delay = 1000;     /*1s*/
/* < DTS2011050303216 wangjiongfeng 20110504 begin */
/* delete aps_9900_count */
/* < DTS2011050303216 wangjiongfeng 20110504 end */

static int aps_first_read = 1;
/* < DTS2011050303216 wangjiongfeng 20110504 begin */
static int aps_open_flag=0;
/* < DTS2011050303216 wangjiongfeng 20110504 end */
/* use this to make sure which device is open and make a wake lcok*/
/* < DTS2011052502345 wangjiongfeng 20110525 begin */
/* add the flag of min_proximity_value */
/* < DTS2011090706338  liujinggang 20110924 begin */
/*modify the value*/
static int min_proximity_value = 853;
/* DTS2011090706338  liujinggang 20110924 end > */
/* DTS2011052502345 wangjiongfeng 20110525 end >*/
static int light_device_minor = 0;
static int proximity_device_minor = 0;
static struct wake_lock proximity_wake_lock;
static atomic_t l_flag;
static atomic_t p_flag;
/* < DTS2011071500961  liujinggang 20110715 begin */
/*get value of proximity and light*/
static int proximity_data_value = 0;
static int light_data_value = 0;
/* DTS2011071500961  liujinggang 20110715 end > */
/* < DTS2011081902842  liujinggang 20110820 begin */
/*init the value of reg 0*/
static u8  reg0_value = 0x38; 
/* DTS2011081902842  liujinggang 20110820 end > */

#define CMD_BYTE      0x80
#define CMD_WORD      0xA0
#define CMD_SPECIAL   0xE0

#define APDS9900_ENABLE_REG  0x00
#define APDS9900_ATIME_REG   0x01
#define APDS9900_PTIME_REG   0x02
#define APDS9900_WTIME_REG   0x03
#define APDS9900_AILTL_REG   0x04
#define APDS9900_AILTH_REG   0x05
#define APDS9900_AIHTL_REG   0x06
#define APDS9900_AIHTH_REG   0x07
#define APDS9900_PILTL_REG   0x08
#define APDS9900_PILTH_REG   0x09
#define APDS9900_PIHTL_REG   0x0A
#define APDS9900_PIHTH_REG   0x0B
#define APDS9900_PERS_REG    0x0C
#define APDS9900_CONFIG_REG  0x0D
#define APDS9900_PPCOUNT_REG 0x0E
#define APDS9900_CONTROL_REG 0x0F
#define APDS9900_REV_REG      0x11
#define APDS9900_ID_REG       0x12
#define APDS9900_STATUS_REG  0x13
#define APDS9900_CDATAL_REG  0x14
#define APDS9900_CDATAH_REG  0x15
#define APDS9900_IRDATAL_REG 0x16
#define APDS9900_IRDATAH_REG 0x17
#define APDS9900_PDATAL_REG  0x18
#define APDS9900_PDATAH_REG  0x19

#define DETECTION_THRESHOLD	500

#define APDS9900_POWER_ON 1     /* set the APDS9900_ENABLE_REG's PON=1,Writing a 1 activates the APDS9900 */
#define APDS9900_POWER_OFF 0    /* set the APDS9900_ENABLE_REG's PON=1,Writing a 0 disables the APDS9900 */
/* < DTS2011070101164 yuezenglong 20110701 begin */
/*reconfig reg after resume*/
#define APDS9900_ENABLE 0x3F    /* set the APDS9900_ENABLE_REG's*/
/* DTS2011070101164 yuezenglong 20110701 end > */
/* < DTS2011050303216 wangjiongfeng 20110504 begin */
#define APDS9900_POWER_MASK (1<<0)
#define APDS9900_STATUS_PROXIMITY_BIT (1<<5)
#define APDS9900_STATUS_ALS_BIT (1<<4)
/* < DTS2011050303216 wangjiongfeng 20110504 end */
#define APDS9900_PEN_BIT_SHIFT 2
#define APDS9900_AEN_BIT_SHIFT 1

#define APDS_9901_ID  0x20 /* APDS-9901 */
#define APDS_9900_ID  0x29 /* APDS-9900 */
#define APDS_9900_REV_ID 0x01

/* < DTS2011081902842  liujinggang 20110820 begin */
/*define and init the value of reg 0*/
/* < DTS2011052502345 wangjiongfeng 20110525 begin */
#define APDS_9900_MAX_PPDATA 1023
/* < DTS2011090706338  liujinggang 20110924 begin */
/*modify the value*/
#define APDS_9900_PWINDOWS_VALUE 170
/* DTS2011090706338  liujinggang 20110924 end > */
#define APDS_9900_PWAVE_VALUE 250
/* DTS2011052502345 wangjiongfeng 20110525 end >*/
#define APDS9900_REG0_POWER_OFF	0xfe
#define APDS9900_REG0_AEN_OFF	0xfd
#define APDS9900_REG0_PEN_OFF	0xfb

static struct aps_init_regdata aps9900_init_regdata[]=
{
    {APDS9900_ENABLE_REG, 0x0},
    {APDS9900_ATIME_REG,   0xdb},
    {APDS9900_PTIME_REG,   0xff},
    /* < DTS2011090706338  liujinggang 20110924 begin */
    /*modify the value*/
    {APDS9900_WTIME_REG,  0xb6},
    /* DTS2011090706338  liujinggang 20110924 end > */
    /* < DTS2011052003350 wangjiongfeng 20110520 begin */
    /* modify the ppcount from 8 to 4 */
    {APDS9900_PPCOUNT_REG, 0x04},
    /* DTS2011052003350 wangjiongfeng 20110520 end >*/
    {APDS9900_CONTROL_REG, 0x20},
    {APDS9900_ENABLE_REG, 0x38},
    /* < DTS2011052502345 wangjiongfeng 20110525 begin */
    {APDS9900_PERS_REG, 0x12}
    /* DTS2011052502345 wangjiongfeng 20110525 end >*/
};
/* DTS2011081902842  liujinggang 20110820 end > */

/* Coefficients in open air: 
  * GA:Glass (or Lens) Attenuation Factor
  * DF:Device Factor
  * alsGain: ALS Gain
  * aTime: ALS Timing
  * ALSIT = 2.72ms * (256 ¨C ATIME) = 2.72ms * (256-0xDB) =  100ms
  */
static int aTime = 0xDB; 
static int alsGain = 1;
/* < DTS2011081101306  liujinggang 20110811 begin */
/* modify the parameter by FAE */
/*
static int ga=48;
static int coe_b=223;
static int coe_c=7;
static int coe_d=142;
*/
/* < DTS2011081702329  liujinggang 20110817 begin */
/* modify the parameter */
static int ga=515;
/* DTS2011081702329  liujinggang 20110817 end > */
static int coe_b=1948;
static int coe_c=613;
static int coe_d=1163;
/* DTS2011081101306  liujinggang 20110811 end > */
static int DF=52;

static int  set_9900_register(struct aps_data  *aps, u8 reg, u16 value, int flag)
{
    int ret;

    mutex_lock(&aps->mlock);
    /*  flag=1 means reading the world value */
    if (flag)
    {
        ret = i2c_smbus_write_word_data(aps->client, CMD_WORD | reg, value);
    }
    /*  flag=0 means reading the byte value */
    else
    {
        ret = i2c_smbus_write_byte_data(aps->client, CMD_BYTE | reg, (u8)value);
    }

    mutex_unlock(&aps->mlock);
    /* < DTS2011050303216 wangjiongfeng 20110504 begin */
	/* delete some lines */
	/* < DTS2011050303216 wangjiongfeng 20110504 end */

    return ret;
}
static int get_9900_register(struct aps_data  *aps, u8 reg, int flag)
{
    int ret;

    mutex_lock(&aps->mlock);
    if (flag)
    {
        ret = i2c_smbus_read_word_data(aps->client, CMD_WORD | reg);
    }
    else
    {
        ret = i2c_smbus_read_byte_data(aps->client, CMD_BYTE | reg);
    }
    mutex_unlock(&aps->mlock);
    return ret;
}

static int aps_9900_open(struct inode *inode, struct file *file)
{ 
    /* < DTS2011050303216 wangjiongfeng 20110504 begin */
    /* when the device is open use this if light open report -1 when proximity open then lock it*/
    if( light_device_minor == iminor(inode) ){
        PROXIMITY_DEBUG("%s:light sensor open\n", __func__);
        aps_first_read = 1;
    }

    if( proximity_device_minor == iminor(inode) ){
        PROXIMITY_DEBUG("%s:proximity_device_minor == iminor(inode)\n", __func__);
        wake_lock( &proximity_wake_lock);
        /* 0 is close, 1 is far */
        input_report_abs(this_aps_data->input_dev, ABS_DISTANCE, 1);
        input_sync(this_aps_data->input_dev);
    }

    if (!aps_open_flag)
    {
        u8 value_reg0;
        int ret;
        
		/* < DTS2011081902842  liujinggang 20110820 begin */
		/*set bit 0 of reg 0 ==1*/
		value_reg0 = reg0_value;
        /* if power on ,will not set PON=1 again */
        if (APDS9900_POWER_OFF == (value_reg0 & APDS9900_POWER_MASK))
        {
            value_reg0 = value_reg0 | APDS9900_POWER_ON;
            ret = set_9900_register(this_aps_data, APDS9900_ENABLE_REG, value_reg0 , 0);
            if (ret)
            {
                printk(KERN_ERR "%s:set_9900_register is error(%d)!", __func__, ret);
            }
		     else
		     {
				reg0_value = value_reg0;
			 }
        }
		/* DTS2011081902842  liujinggang 20110820 end > */
        if (this_aps_data->client->irq)   
        {
            enable_irq(this_aps_data->client->irq);
        }
    }
    aps_open_flag++;
    /* < DTS2011050303216 wangjiongfeng 20110504 end */
	return nonseekable_open(inode, file);
}

static int aps_9900_release(struct inode *inode, struct file *file)
{
    /* < DTS2011050303216 wangjiongfeng 20110504 begin */
    aps_open_flag--;
    aps_9900_delay = 1000;//1s
    
    /*when proximity is released then unlock it*/
    if( proximity_device_minor == iminor(inode) ){
        PROXIMITY_DEBUG("%s: proximity_device_minor == iminor(inode)\n", __func__);
        wake_unlock( &proximity_wake_lock);
    }
    if( light_device_minor == iminor(inode) ){
        PROXIMITY_DEBUG("%s: light_device_minor == iminor(inode)\n", __func__);
    }
    if (!aps_open_flag)
    {
        int value_reg0,ret;
        
		/* < DTS2011081902842  liujinggang 20110820 begin */
		/*set bit 0 of reg 0 ==0*/
		value_reg0 = reg0_value & APDS9900_REG0_POWER_OFF;
        ret = set_9900_register(this_aps_data, APDS9900_ENABLE_REG, value_reg0, 0);
        if (ret)
        {
            printk(KERN_ERR "%s:set_9900_register is error(%d)!", __func__, ret);
        }
		else
		{
			reg0_value = value_reg0;
		}
		/* DTS2011081902842  liujinggang 20110820 end > */
        if (this_aps_data->client->irq) 
        {
            disable_irq(this_aps_data->client->irq);
        }
    }
    /* < DTS2011050303216 wangjiongfeng 20110504 end */
    return 0;
}

static int
aps_9900_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
     unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    int flag;
    int value_reg0;
	/* < DTS2011050303216 wangjiongfeng 20110504 begin */
    int set_flag;
    int ret;
	/* < DTS2011050303216 wangjiongfeng 20110504 end */
	/* < DTS2011081902842  liujinggang 20110820 begin */
	/*delete one line */
	/* DTS2011081902842  liujinggang 20110820 end > */
    switch (cmd) 
    {
        case ECS_IOCTL_APP_SET_LFLAG:   /* app set  light sensor flag */
        {
            if (copy_from_user(&flag, argp, sizeof(flag)))
            {
                return -EFAULT;
            }
            atomic_set(&l_flag, flag);
			/* < DTS2011050303216 wangjiongfeng 20110504 begin */
            set_flag = atomic_read(&l_flag) ? 1 : 0;

			/* < DTS2011081902842  liujinggang 20110820 begin */
			/*set bit 1 of reg 0 by set_flag */
            /* set AEN,if l_flag=1 then enable ALS.if l_flag=0 then disable ALS */
			if (set_flag)
			{
				value_reg0 = reg0_value | (set_flag << APDS9900_AEN_BIT_SHIFT);
           		ret = set_9900_register(this_aps_data, APDS9900_ENABLE_REG, value_reg0, 0);
			}
			else
			{
				value_reg0 = reg0_value & APDS9900_REG0_AEN_OFF;
           		ret = set_9900_register(this_aps_data, APDS9900_ENABLE_REG, value_reg0, 0);
			}
            if (ret)
            {
                printk(KERN_ERR "%s:set ECS_IOCTL_APP_SET_LFLAG flag is error(%d)!", __func__, ret);
            }
			else
			{
				reg0_value = value_reg0;
			}
			/* DTS2011081902842  liujinggang 20110820 end > */
			/* < DTS2011050303216 wangjiongfeng 20110504 end */
            break;
        }
        case ECS_IOCTL_APP_GET_LFLAG:  /*app  get open light sensor flag*/
        {
            flag = atomic_read(&l_flag);
            if (copy_to_user(argp, &flag, sizeof(flag)))
            {
                return -EFAULT;
            }
            break;
        }
        case ECS_IOCTL_APP_SET_PFLAG:
        {
            if (copy_from_user(&flag, argp, sizeof(flag)))
            {
                return -EFAULT;
            }
            atomic_set(&p_flag, flag);
			/* < DTS2011050303216 wangjiongfeng 20110504 begin */
            set_flag = atomic_read(&p_flag) ? 1 : 0;
			/* < DTS2011081902842  liujinggang 20110820 begin */
			/*set bit 1 of reg 0 by set_flag */
            /* set PEN,if p_flag=1 then enable proximity.if p_flag=0 then disable proximity */
			if (set_flag)
			{
				value_reg0 = reg0_value | (set_flag << APDS9900_PEN_BIT_SHIFT);
          		ret = set_9900_register(this_aps_data, APDS9900_ENABLE_REG, value_reg0, 0);
			}
			else
			{
				value_reg0 = reg0_value & APDS9900_REG0_PEN_OFF;
           		ret = set_9900_register(this_aps_data, APDS9900_ENABLE_REG, value_reg0, 0);
			}
            if (ret)
            {
                printk(KERN_ERR "%s:set ECS_IOCTL_APP_SET_PFLAG flag is error(%d)!", __func__, ret);
            }	
			else
			{
				reg0_value = value_reg0;
			}
			/* DTS2011081902842  liujinggang 20110820 end > */
			/* < DTS2011050303216 wangjiongfeng 20110504 end */
            break;
        }
        case ECS_IOCTL_APP_GET_PFLAG:  /*get open acceleration sensor flag*/
        {
            flag = atomic_read(&p_flag);
            if (copy_to_user(argp, &flag, sizeof(flag)))
            {
                return -EFAULT;
            }
            break;
        }
        case ECS_IOCTL_APP_SET_DELAY:
        {
            if (copy_from_user(&flag, argp, sizeof(flag)))
            {
                return -EFAULT;
            }
            if(flag)
                aps_9900_delay = flag;
            else
                aps_9900_delay = 20;   /*20ms*/
            break;
        }
        case ECS_IOCTL_APP_GET_DELAY:
        {
            flag = aps_9900_delay;
            if (copy_to_user(argp, &flag, sizeof(flag)))
            {
                return -EFAULT;
            }
            break;
        }
		/* < DTS2011071500961  liujinggang 20110715 begin */
		/*get value of proximity and light*/
		case ECS_IOCTL_APP_GET_PDATA_VALVE:
        {
            flag = proximity_data_value;
            if (copy_to_user(argp, &flag, sizeof(flag)))
            {
                return -EFAULT;
            }
            break;
        }
		case ECS_IOCTL_APP_GET_LDATA_VALVE:
        {
            flag = light_data_value;
            if (copy_to_user(argp, &flag, sizeof(flag)))
            {
                return -EFAULT;
            }
            break;
        }
		/* DTS2011071500961  liujinggang 20110715 end > */

        default:
  	/* < DTS2011050303216 wangjiongfeng 20110504 begin */
        {
            break;
        }
		
    }
	/* < DTS2011050303216 wangjiongfeng 20110504 end */
    return 0;
}

static struct file_operations aps_9900_fops = {
    .owner = THIS_MODULE,
    .open = aps_9900_open,
    .release = aps_9900_release,
    .ioctl = aps_9900_ioctl,
};

static struct miscdevice light_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "light",
    .fops = &aps_9900_fops,
};

static struct miscdevice proximity_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "proximity",
    .fops = &aps_9900_fops,
};

static int luxcalculation(int cdata, int irdata)
{
    int luxValue=0;
    /* < DTS2011050303216 wangjiongfeng 20110504 begin */
	int first_half, sec_half;
	/* < DTS2011050303216 wangjiongfeng 20110504 end */
    int iac1=0;
    int iac2=0;
    int iac=0;

    /*
     *Lux Equation:
     *       IAC1 = CH0DATA -B * CH1DATA                (IAC:IR Adjusted Count)
     *       IAC2 = C * CH0DATA - D * CH1DATA
     *       IAC = Max (IAC1, IAC2, 0)
     *       LPC = GA * DF / (ALSIT * AGAIN)               (LPC:Lux per Count)
     *       Lux = IAC * LPC
     *Coefficients in open air:
     *       GA = 0.48
     *       B = 2.23
     *       C = 0.7
     *       D = 1.42
    */
	/* < DTS2011081101306  liujinggang 20110811 begin */
	/*adjust data by parameter*/
    iac1 = (int) (cdata - (coe_b*irdata/1000));
    iac2 = (int) ((coe_c*cdata/1000) - (coe_d*irdata/1000));
    
	/* < DTS2011050303216 wangjiongfeng 20110504 begin */
    if (iac1 > iac2)
    {
        iac = iac1;
    }
    else if (iac1 <= iac2)
    {
        iac = iac2;
    }
    else
    {
        iac = 0;
    }
	
	/* < DTS2011081702329  liujinggang 20110817 begin */
    first_half = iac*ga*DF/100;
	/* DTS2011081702329  liujinggang 20110817 end > */
    sec_half = ((272*(256-aTime))*alsGain)/100;
    luxValue = first_half/sec_half;
	 APS9900_DBG("first_half====%d  ,sec_half ===%d,iac===%d\n", first_half,sec_half,iac);
    /* < DTS2011050303216 wangjiongfeng 20110504 end */
	/* DTS2011081101306  liujinggang 20110811 end > */
    return luxValue;
}

static void aps_9900_work_func(struct work_struct *work)
{
    /* < DTS2011050303216 wangjiongfeng 20110504 begin */
    int pdata = 0;/* proximity data*/
    int cdata = 0;/* ch0 data  */
    int irdata = 0;/* ch1 data  */
    int cdata_high = 0, cdata_low = 0;
    int lux; 
    int status;
    int ret;
	/* < DTS2011081101306  liujinggang 20110811 begin */
    int als_level = 0;
    int i = 0;
	/* DTS2011081101306  liujinggang 20110811 end > */
    struct aps_data *aps = container_of(work, struct aps_data, work);

    status = get_9900_register(aps,APDS9900_STATUS_REG,0);
    /* proximity flag is open and the interrupt belongs to proximity */
    if (atomic_read(&p_flag) && (status & APDS9900_STATUS_PROXIMITY_BIT))
    {
        int pthreshold_h=0, pthreshold_l;
        /* read the proximity data  */
        pdata = get_9900_register(aps, APDS9900_PDATAL_REG, 1);
        /* < DTS2011052502345 wangjiongfeng 20110525 begin */
        /* add the arithmetic of setting the proximity thresholds automatically */
        if ((pdata + APDS_9900_PWAVE_VALUE) < min_proximity_value)
        {
            min_proximity_value = pdata + APDS_9900_PWAVE_VALUE;
            ret = set_9900_register(aps, APDS9900_PILTL_REG, min_proximity_value, 1);
            ret |= set_9900_register(aps, APDS9900_PIHTL_REG, (min_proximity_value + APDS_9900_PWINDOWS_VALUE), 1);
            if (ret)
            {
                printk(KERN_ERR "%s:set APDS9900_PILTL_REG register is error(%d)!", __func__, ret);
            }
            APS9900_DBG("%s:min_proximity_value=%d\n", __func__, min_proximity_value);
        }
        /* DTS2011052502345 wangjiongfeng 20110525 end >*/
        pthreshold_h = get_9900_register(aps, APDS9900_PIHTL_REG, 1);
        pthreshold_l = get_9900_register(aps, APDS9900_PILTL_REG, 1);
        /* < DTS2011052003350 wangjiongfeng 20110520 begin */
        /* add some logs */
        APS9900_DBG("%s:pdata=%d pthreshold_h=%d pthreshold_l=%d\n", __func__, pdata, pthreshold_h, pthreshold_l);
        /* DTS2011052003350 wangjiongfeng 20110520 end >*/
        /* clear proximity interrupt bit */
        ret = set_9900_register(aps, 0x65, 0, 0);
        if (ret)
        {
            printk(KERN_ERR "%s:set_9900_register is error(%d),clear failed!", __func__, ret);
        }
		/* < DTS2011071500961  liujinggang 20110715 begin */
		/*get value of proximity*/
         proximity_data_value = pdata;
		/* DTS2011071500961  liujinggang 20110715 end > */
        /* if more than the value of  proximity high threshold we set*/
        if (pdata >= pthreshold_h) {
            /* < DTS2011051603591 wangjiongfeng 20110504 begin */
			/* < DTS2011052003350 wangjiongfeng 20110520 begin */
            /* modify the low threshold register of proximity from 400 to 700 */
			/* < DTS2011052502345 wangjiongfeng 20110525 begin */
            ret = set_9900_register(aps, APDS9900_PILTL_REG, min_proximity_value, 1);
			/* DTS2011052502345 wangjiongfeng 20110525 end >*/
			/* DTS2011052003350 wangjiongfeng 20110520 end >*/
            /* < DTS2011051603591 wangjiongfeng 20110504 end */
            if (ret)
            {
                printk(KERN_ERR "%s:set APDS9900_PILTL_REG register is error(%d)!", __func__, ret);
            }
            input_report_abs(aps->input_dev, ABS_DISTANCE, 0);
            input_sync(aps->input_dev);
        }
        /* if less than the value of  proximity low threshold we set*/
        /* < DTS2011052502345 wangjiongfeng 20110525 begin */
        /* the condition of pdata==pthreshold_l is valid */
        else if (pdata <=  pthreshold_l)
        {
            ret = set_9900_register(aps, APDS9900_PILTL_REG, 0, 1);
            if (ret)
            {
                printk(KERN_ERR "%s:set APDS9900_PILTL_REGs register is error(%d)!", __func__, ret);
            }
            input_report_abs(aps->input_dev, ABS_DISTANCE, 1);
            input_sync(aps->input_dev);
        }
        /* DTS2011052502345 wangjiongfeng 20110525 end >*/
    }
    /* ALS flag is open and the interrupt belongs to ALS */
    if (atomic_read(&l_flag) && (status & APDS9900_STATUS_ALS_BIT)) 
    {
        /* read the CH0 data and CH1 data  */
        cdata = get_9900_register(aps, APDS9900_CDATAL_REG, 1);
        irdata = get_9900_register(aps, APDS9900_IRDATAL_REG, 1);
        /* set ALS high threshold = ch0(cdata) + 20%,low threshold = ch0(cdata) - 20% */
        cdata_high = (cdata *  600)/500;
        cdata_low = (cdata *  400)/500;
		/* < DTS2011081101306  liujinggang 20110811 begin */
		/* the max value of cdata_high == 0xffff */
		if(0xffff <= cdata_high )
		{
			cdata_high=0xffff;
		}
		/* DTS2011081101306  liujinggang 20110811 end > */
        /* clear als interrupt bit */
        ret = set_9900_register(aps, 0x66,0, 0);
        ret |= set_9900_register(aps, APDS9900_AILTL_REG, cdata_low, 1);
        ret |= set_9900_register(aps, APDS9900_AIHTL_REG, cdata_high, 1);
        if (ret)
        {
            printk(KERN_ERR "%s:set APDS9900_AILTL_REG register is error(%d)!", __func__, ret);
        }
        /* convert the raw pdata and irdata to the value in units of lux */
        lux = luxcalculation(cdata, irdata);
		/* < DTS2011081101306  liujinggang 20110811 begin */
		/*get level of light*/
        APS9900_DBG("%s:cdata=%d irdata=%d lux=%d\n",__func__, cdata, irdata, lux);
        if (lux >= 0) 
        {
			/* < DTS2011071500961  liujinggang 20110715 begin */
			/*get value of light*/
	         light_data_value = lux;
			/* DTS2011071500961  liujinggang 20110715 end > */
	        /* < DTS2011052003350 wangjiongfeng 20110520 begin */
	        /* lux=0 is valid */
	        als_level = LSENSOR_MAX_LEVEL - 1;
			for (i = 0; i < ARRAY_SIZE(lsensor_adc_table); i++){
				if (lux < lsensor_adc_table[i]){
					als_level = i;
					break;
				}
			}
	        APS9900_DBG("%s:cdata=%d irdata=%d lux=%d,als_level==%d\n",__func__, cdata, irdata, lux,als_level);
	
			if(aps_first_read)
			{
				aps_first_read = 0;
				input_report_abs(aps->input_dev, ABS_LIGHT, -1);
				input_sync(aps->input_dev);
			}
			else
			{
				input_report_abs(aps->input_dev, ABS_LIGHT, als_level);
				input_sync(aps->input_dev);
			}
        }
        /* if lux<0,we need to change the gain which we can set register 0x0f */
        else {
                printk("Need to change gain %2d \n", lux);
        }
        /* DTS2011052003350 wangjiongfeng 20110520 end >*/
	/* DTS2011081101306  liujinggang 20110811 end > */
    }   
    if ((status & APDS9900_STATUS_PROXIMITY_BIT)  && (!atomic_read(&p_flag)))
    {
        /* clear proximity interrupt bit */
        ret = set_9900_register(aps, 0x65, 0, 0);
        if (ret)
        {
            printk(KERN_ERR "%s:clear proximity interrupt bit failed(%d)!", __func__, ret);
        }
    }
    if ((status & APDS9900_STATUS_ALS_BIT) && (!atomic_read(&l_flag)))
    {
        /* clear als interrupt bit */
        ret = set_9900_register(aps, 0x66,0, 0);
        if (ret)
        {
            printk(KERN_ERR "%s:clear als interrupt bit failed(%d)!", __func__, ret);
        }
    }

    /* < DTS2011052502345 wangjiongfeng 20110525 begin */
    /* delete the condition */
    if (aps->client->irq)
    {
        enable_irq(aps->client->irq);
    }
    /* DTS2011052502345 wangjiongfeng 20110525 end >*/
	/* < DTS2011050303216 wangjiongfeng 20110504 end */
}
    /* < DTS2011050303216 wangjiongfeng 20110504 begin */
	/* delete some lines */
	/* < DTS2011050303216 wangjiongfeng 20110504 end */

static inline int aps_i2c_reg_init(struct aps_data *aps)
{
    int ret;
    int i;
    int revid;
    int id;

    for (i=0; i< ARRAY_SIZE(aps9900_init_regdata);i++)
    {
        ret = set_9900_register(aps, CMD_BYTE|aps9900_init_regdata[i].reg, aps9900_init_regdata[i].data, 0);
        if (ret < 0)
        {
            printk(KERN_ERR "Ret of init aps9900 regs(%d - %d) is %d\n", aps9900_init_regdata[i].reg, aps9900_init_regdata[i].data, get_9900_register(aps, aps9900_init_regdata[i].reg, 0));
            break;
        }
    }
    /* ID check ,if not equal, return -1 */
    revid = get_9900_register(aps, APDS9900_REV_REG, 0);
    id = get_9900_register(aps, APDS9900_ID_REG, 0);
    if ((APDS_9900_REV_ID != revid)  || (APDS_9900_ID != id)) 
    {
        printk(KERN_ERR "%s:The ID of checking failed!(revid=%.2x id=%.2x)", __func__, revid, id);
        ret = -1;
    }

    return ret;
}

irqreturn_t aps_irq_handler(int irq, void *dev_id)
{
    struct aps_data *aps = dev_id;

    disable_irq_nosync(aps->client->irq);
    queue_work(aps_wq, &aps->work);

    return IRQ_HANDLED;
}

/* < DTS2011050303216 wangjiongfeng 20110504 begin */
/* delete aps_timer_func function */
/* < DTS2011050303216 wangjiongfeng 20110504 end */
static int aps_9900_probe(
    struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret;
    struct aps_data *aps;
	/* < DTS2011050303216 wangjiongfeng 20110504 begin */
    struct aps9900_hw_platform_data *platform_data = NULL;

    if (client->dev.platform_data == NULL)
    {
        pr_err("%s:platform data is NULL. exiting.\n", __func__);
        ret = -ENODEV;
        goto err_exit;
    }

    platform_data = client->dev.platform_data;

    if (platform_data->aps9900_power)
    {
        ret = platform_data->aps9900_power(IC_PM_ON);
        if (ret < 0)
        {
            pr_err("%s:aps9900 power on error!\n", __func__);
            goto err_exit ;
        }
    }
	/* < DTS2011050303216 wangjiongfeng 20110504 end */
    mdelay(5);
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        PROXIMITY_DEBUG(KERN_ERR "aps_9900_probe: need I2C_FUNC_I2C\n");
        ret = -ENODEV;
        goto err_check_functionality_failed;
    }
	/* < DTS2011050303216 wangjiongfeng 20110504 begin */
	/* delete some lines */
	/* < DTS2011050303216 wangjiongfeng 20110504 end */

    aps = kzalloc(sizeof(*aps), GFP_KERNEL);
    if (aps == NULL) {
        ret = -ENOMEM;
        goto err_alloc_data_failed;
    }

    mutex_init(&aps->mlock);
    INIT_WORK(&aps->work, aps_9900_work_func);
    aps->client = client;
    i2c_set_clientdata(client, aps);

    PROXIMITY_DEBUG(KERN_INFO "ghj aps_9900_probe send command 2\n ");

    ret = aps_i2c_reg_init(aps);
    if (ret <0)
    {
        printk(KERN_ERR "aps_i2c_reg_init: Failed to init aps_i2c_reg_init!(%d)\n",ret);
        goto err_detect_failed;
    }
    /* DTS2011050303216 wangjiongfeng 20110504 begin */
    /* delete mdelay(12) */
    /* DTS2011050303216 wangjiongfeng 20110504 end */
    if (client->irq) 
    {
/* < DTS2011050303216 wangjiongfeng 20110504 begin */
        if (platform_data->aps9900_gpio_config_interrupt)
        {
            ret = platform_data->aps9900_gpio_config_interrupt();
            if (ret) 
            {
                PROXIMITY_DEBUG(KERN_ERR "gpio_tlmm_config error\n");
                goto err_gpio_config_failed;
            }
        }
        
        if (request_irq(client->irq, aps_irq_handler,IRQF_TRIGGER_LOW, client->name, aps) >= 0) 
        {
            PROXIMITY_DEBUG("Received IRQ!\n");
			/* < DTS2011070101164 yuezenglong 20110701 begin */
    	    disable_irq(aps->client->irq);
            #if 0
            if (set_irq_wake(client->irq, 1) < 0)
            {
                printk(KERN_ERR "failed to set IRQ wake\n");
            }
            #endif
    		/* DTS2011070101164 yuezenglong 20110701 end > */
        }
        else 
        {
            printk("Failed to request IRQ!\n");
        }
         /* set the threshold of proximity and ALS */
         ret = set_9900_register(aps, APDS9900_AILTL_REG, 0, 1);
         ret |= set_9900_register(aps, APDS9900_AIHTL_REG, 0, 1);
         ret |= set_9900_register(aps, APDS9900_PILTL_REG, 0, 1);
         /* < DTS2011052003350 wangjiongfeng 20110520 begin */
         /* modify the high proximity threshold from 500 to 800 */
         /* < DTS2011090706338  liujinggang 20110924 begin */
         /*modify the value*/
         ret |= set_9900_register(aps, APDS9900_PIHTL_REG, 0x3c0, 1);
         /* DTS2011090706338  liujinggang 20110924 end > */
         /* DTS2011052003350 wangjiongfeng 20110520 end >*/
		 /* < DTS2011052502345 wangjiongfeng 20110525 begin */
		 /* set the low thresthold of 1023 to make sure make an interrupt */
         /* < DTS2011090706338  liujinggang 20110924 begin */
         /*modify the value*/
         ret |= set_9900_register(aps, APDS9900_PILTL_REG, 0x3bf, 1);
         /* DTS2011090706338  liujinggang 20110924 end > */
		 /* DTS2011052502345 wangjiongfeng 20110525 end >*/
         if (ret)
         {
             printk(KERN_ERR "%s:set the threshold of proximity and ALS failed(%d)!", __func__, ret);
         }
		 
    }
    /* if not define irq,then error */
    else
    {
        printk(KERN_ERR "please set the irq num!\n");
        goto err_detect_failed;
    }
    if (sensor_9900_dev == NULL) 
    {
         aps->input_dev = input_allocate_device();
         if (aps->input_dev == NULL) {
         ret = -ENOMEM;
         PROXIMITY_DEBUG(KERN_ERR "aps_9900_probe: Failed to allocate input device\n");
         goto err_input_dev_alloc_failed;
         }
        aps->input_dev->name = "sensors_aps";
        aps->input_dev->id.bustype = BUS_I2C;
        input_set_drvdata(aps->input_dev, aps);
        
        ret = input_register_device(aps->input_dev);
        if (ret) {
            printk(KERN_ERR "aps_9900_probe: Unable to register %s input device\n", aps->input_dev->name);
            goto err_input_register_device_failed;
        }
        sensor_9900_dev = aps->input_dev;
    } else {
        printk(KERN_INFO "sensor_dev is not null+++++++++++++++++++++++\n");
        aps->input_dev = sensor_9900_dev;
    }
/* < DTS2011050303216 wangjiongfeng 20110504 end */
    set_bit(EV_ABS, aps->input_dev->evbit);
    input_set_abs_params(aps->input_dev, ABS_LIGHT, 0, 10240, 0, 0);
    input_set_abs_params(aps->input_dev, ABS_DISTANCE, 0, 1, 0, 0);

    ret = misc_register(&light_device);
    if (ret) {
        printk(KERN_ERR "aps_9900_probe: light_device register failed\n");
        goto err_light_misc_device_register_failed;
    }

    ret = misc_register(&proximity_device);
    if (ret) {
        printk(KERN_ERR "aps_9900_probe: proximity_device register failed\n");
        goto err_proximity_misc_device_register_failed;
    }

    if( light_device.minor != MISC_DYNAMIC_MINOR ){
        light_device_minor = light_device.minor;
    }


    if( proximity_device.minor != MISC_DYNAMIC_MINOR ){
        proximity_device_minor = proximity_device.minor ;
    }

    wake_lock_init(&proximity_wake_lock, WAKE_LOCK_SUSPEND, "proximity");

    aps_wq = create_singlethread_workqueue("aps_wq");

    if (!aps_wq) 
    {
        ret = -ENOMEM;
        goto err_create_workqueue_failed;
    }

    this_aps_data =aps;
	/* < DTS2011050303216 wangjiongfeng 20110504 begin */

    /* < DTS2011052803160 shenjinming 20110611 begin */
    #ifdef CONFIG_HUAWEI_HW_DEV_DCT
    /* detect current device successful, set the flag as present */
    set_hw_dev_flag(DEV_I2C_APS);
    #endif
    /* DTS2011052803160 shenjinming 201106011 end > */
   
    printk(KERN_INFO "aps_9900_probe: Start Proximity Sensor APS-9900\n");

    return 0;

err_create_workqueue_failed:
    misc_deregister(&proximity_device);
err_proximity_misc_device_register_failed:
    misc_deregister(&light_device);
err_light_misc_device_register_failed:
err_input_register_device_failed:
    input_free_device(aps->input_dev);
err_input_dev_alloc_failed:
err_gpio_config_failed:
err_detect_failed:
    kfree(aps);
err_alloc_data_failed:
err_check_functionality_failed:
    if(platform_data->aps9900_power)
    {
        platform_data->aps9900_power(IC_PM_OFF);
    }
err_exit:
    return ret;
	/* < DTS2011050303216 wangjiongfeng 20110504 end */
  
}
static int aps_9900_remove(struct i2c_client *client)
{
    struct aps_data *aps = i2c_get_clientdata(client);

    PROXIMITY_DEBUG("ghj aps_9900_remove enter\n ");
	/* < DTS2011050303216 wangjiongfeng 20110504 begin */
    if (aps->client->irq)
    {
        disable_irq(aps->client->irq);
    }
	/* < DTS2011070101164 yuezenglong 20110701 begin */
    free_irq(client->irq, aps);
	/* DTS2011070101164 yuezenglong 20110701 end > */
	/* < DTS2011050303216 wangjiongfeng 20110504 end */
    misc_deregister(&light_device);
    misc_deregister(&proximity_device);
    input_unregister_device(aps->input_dev);

    kfree(aps);
    return 0;
}

/* < DTS2011081902842  liujinggang 20110820 begin */
/*set  reg 0 */
static int aps_9900_suspend(struct i2c_client *client, pm_message_t mesg)
{
    int ret;
    struct aps_data *aps = i2c_get_clientdata(client);

    PROXIMITY_DEBUG("ghj aps_9900_suspend enter\n ");
	/* < DTS2011050303216 wangjiongfeng 20110504 begin */
    if (aps->client->irq)
    {
        disable_irq(aps->client->irq);
    }
	/* < DTS2011050303216 wangjiongfeng 20110504 end */
    ret = cancel_work_sync(&aps->work);

    /* set [PON] bit =0 ,meaning disables the oscillator */
	/* < DTS2011050303216 wangjiongfeng 20110504 begin */
	/* < DTS2011070101164 yuezenglong 20110701 begin */
	/*reconfig reg before supspend*/
    ret = set_9900_register(aps, APDS9900_ENABLE_REG,  APDS9900_POWER_OFF,0);
	/* DTS2011070101164 yuezenglong 20110701 end > */
    if (ret)
    {
        printk(KERN_ERR "%s:set APDS9900_ENABLE_REG register[PON=OFF] failed(%d)!", __func__, ret);
    }
	/* < DTS2011050303216 wangjiongfeng 20110504 end */
    return 0;
}

static int aps_9900_resume(struct i2c_client *client)
{
    /* < DTS2011050303216 wangjiongfeng 20110504 begin */
    int ret;
    struct aps_data *aps = i2c_get_clientdata(client);
    
    PROXIMITY_DEBUG("ghj aps_9900_resume enter\n ");
	/* < DTS2011070101164 yuezenglong 20110701 begin */
    /* Command 0 register: set [PON] bit =1 */
    ret = set_9900_register(aps, APDS9900_ENABLE_REG, reg0_value,0);
	/* DTS2011070101164 yuezenglong 20110701 end > */
    if (ret)
    {
        printk(KERN_ERR "%s:set APDS9900_ENABLE_REG register[PON=ON] failed(%d)!", __func__, ret);
    }
    if (aps->client->irq)
    {
        enable_irq(aps->client->irq);
    }
	/* < DTS2011050303216 wangjiongfeng 20110504 end */
    return 0;
}
/* DTS2011081902842  liujinggang 20110820 end > */

static const struct i2c_device_id aps_id[] = {
    { "aps-9900", 0 },
    { }
};

static struct i2c_driver aps_driver = {
    .probe      = aps_9900_probe,
    .remove     = aps_9900_remove,
    .suspend    = aps_9900_suspend,
    .resume     = aps_9900_resume,
    .id_table   = aps_id,
    .driver = {
        .name   ="aps-9900",
    },
};

static int __devinit aps_9900_init(void)
{
    return i2c_add_driver(&aps_driver);
}

static void __exit aps_9900_exit(void)
{
    i2c_del_driver(&aps_driver);
	/* < DTS2011050303216 wangjiongfeng 20110504 begin */
    if (aps_wq)
    {
        destroy_workqueue(aps_wq);
    }
	/* < DTS2011050303216 wangjiongfeng 20110504 end */
}

device_initcall_sync(aps_9900_init);
module_exit(aps_9900_exit);
MODULE_DESCRIPTION("Proximity Driver");
MODULE_LICENSE("GPL");
/* < DTS2011042604384  wangjiongfeng 20110427 end */
