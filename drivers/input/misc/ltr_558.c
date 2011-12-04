/* < DTS2011102806486  zhangmin 20111029  begin */
/* drivers/input/misc/ltr_558.c*/
/*
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

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/hardware_self_adapt.h>
#include <linux/wakelock.h>

#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <linux/hw_dev_dec.h>
#endif

#include "ltr_558.h"

#ifndef PROXIMITY_LTR_DEBUG
#define PROXIMITY_LTR_DEBUG(fmt, args...) printk(KERN_INFO fmt, ##args)
#else
#define PROXIMITY_LTR_DEBUG(fmt, args...)
#endif
/* add the macro of log */
static int ltr558_debug_mask;
module_param_named(ltr558_debug, ltr558_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

#define LTR558_DBG(x...) do {\
    if (ltr558_debug_mask) \
        printk(KERN_DEBUG x);\
    } while (0)

/* < DTS2011110704273  zhangmin 20111107 begin */
#define PART_ID 0x80
/* DTS2011110704273  zhangmin 20111107  end > */
#define LSENSOR_MAX_LEVEL 7
static uint16_t ltrsensor_adc_table[LSENSOR_MAX_LEVEL] = {
	80, 100, 340, 700, 1000, 2000, 6000
};
/*the HAL's table is 
    10, 225, 320, 640, 1280,2600, 10240
*/
#define LTR_558_MAX_PPDATA 2047

static int ltr_558_pwindows_value = 350;
static int ltr_558_pwave_value = 250;
static int min_proximity_value = 1400;
static struct wake_lock proximity_wake_lock;
static struct workqueue_struct *ltr_wq;
static int ps_gainrange;
static int als_gainrange;

static int ltr_first_read = 1;
static int ltr_open_flag=0;
static int ltr_558_delay = 1000; 
static int light_device_minor = 0;
static int proximity_device_minor = 0;
static atomic_t l_flag;
static atomic_t p_flag;
static u8 ps_reg0;
static u8 als_reg0;

struct ltr558_data {
    uint16_t addr;
    struct i2c_client *client;
    struct input_dev *input_dev;
    struct mutex mlock;
    struct hrtimer timer;
    struct work_struct  work;
    int (*power)(int on);
};
static struct ltr558_data *ltr_this_data =  NULL;
struct input_dev *sensor_ltr_dev=NULL;


static int get_ltr_register(struct ltr558_data *ltr , u8 reg)
{
	int read_data;
	/* < DTS2011110704273  zhangmin 20111107 begin */
    mutex_lock(&ltr->mlock);
	/* DTS2011110704273  zhangmin 20111107  end > */
    read_data = i2c_smbus_read_byte_data(ltr->client, reg);
    if (read_data < 0)
    {
        printk(KERN_ALERT "%s:Read LTR register error!\n",__func__);
    }
	/* < DTS2011110704273  zhangmin 20111107 begin */
    mutex_unlock(&ltr->mlock);
	/* DTS2011110704273  zhangmin 20111107  end > */
    return read_data;
}
static inline int set_ltr_register(struct ltr558_data *ltr,  u8 reg, u8 value)
{
	int write_error;
	/* < DTS2011110704273  zhangmin 20111107 begin */
    mutex_lock(&ltr->mlock);
	/* DTS2011110704273  zhangmin 20111107  end > */
    write_error = i2c_smbus_write_byte_data(ltr->client, reg, value);
    if (write_error < 0)
    {
        printk(KERN_ALERT "%s:Write LTR register error!\n",__func__);
		/* < DTS2011110704273  zhangmin 20111107 begin */
        mutex_unlock(&ltr->mlock);
		/* DTS2011110704273  zhangmin 20111107  end > */
        return write_error;   
    }
    else
    {
		/* < DTS2011110704273  zhangmin 20111107 begin */
        mutex_unlock(&ltr->mlock);
		/* DTS2011110704273  zhangmin 20111107  end > */
        return 0;
    }
}
static int ltr558_ps_enable(struct ltr558_data *ltr,int range_index)
{
    int ret= 0;
    int set_range;
    u8  value_reg;
    /*the register of LTR558_INTERRUPT must be set under the module of StandBy*/
    value_reg = get_ltr_register(ltr, LTR558_INTERRUPT);
    value_reg = ( value_reg | 0x01 );
    ret = set_ltr_register(ltr,LTR558_INTERRUPT, value_reg);
    if(ret < 0 )
    {
        printk(KERN_ALERT "%s:set LTR558 ps LTR558_INTERRUPT failed!\n",__func__);
    }
    value_reg = get_ltr_register(ltr,LTR558_PS_CONTR);
    switch (range_index) 
    {
        case PS_RANGE1:
            set_range = MODE_PS_ON_Gain1;
            break;
        case PS_RANGE4:
            set_range = MODE_PS_ON_Gain4;
            break;
        case PS_RANGE8:
            set_range = MODE_PS_ON_Gain8;
            break;
        case PS_RANGE16:
            set_range = MODE_PS_ON_Gain16;
            break;
        default:
            set_range = MODE_PS_ON_Gain16;
            break;
    }  
    set_range |= value_reg ;
    ret = set_ltr_register(ltr,LTR558_PS_CONTR, set_range);
    mdelay(10);
    return ret;
}
static int ltr558_als_enable(struct ltr558_data *ltr,int gainrange)
{
    int ret;
    int value_reg = 0;
    /*the register of LTR558_INTERRUPT must be write under the standby mode*/
    value_reg = get_ltr_register(ltr, LTR558_INTERRUPT);
    value_reg = ( value_reg | 0x02 );
    ret = set_ltr_register(ltr,LTR558_INTERRUPT, value_reg);

    
    if (gainrange == 1)
        ret = set_ltr_register(ltr,LTR558_ALS_CONTR, MODE_ALS_ON_Range1);
    else if (gainrange == 2)
        ret = set_ltr_register(ltr,LTR558_ALS_CONTR, MODE_ALS_ON_Range2);
    else
        ret = -1;
    mdelay(WAKEUP_DELAY);
    return ret;
}
static int ltr558_als_disable(struct ltr558_data *ltr)
{
    int ret = 0;   
    ret = set_ltr_register(ltr,LTR558_ALS_CONTR, MODE_ALS_StdBy);
    if(ret < 0 )
    {
        printk(KERN_ALERT "%s:LTR558 als diable failed!\n",__func__);
    }
    return ret;
}
static int ltr558_ps_disable(struct ltr558_data *ltr)
{
    int ret = 0; 
    ret |= set_ltr_register(ltr,LTR558_PS_CONTR, MODE_PS_StdBy); 
    if(ret < 0 )
    {
        printk(KERN_ALERT "%s:LTR558 ps diable failed!\n",__func__);
    }
    return ret;
}
static int ltr558_ps_read(struct ltr558_data *ltr)
{
    int psval_lo, psval_hi, psdata;
    psval_lo = get_ltr_register(ltr,LTR558_PS_DATA_0);
    if (psval_lo < 0)
    {
        psdata = psval_lo;
    }
    psval_hi = get_ltr_register(ltr,LTR558_PS_DATA_1);
    if (psval_hi < 0)
    {
        psdata = psval_hi;
    }
    psdata = ((psval_hi & 7)* 256) + psval_lo;
    return psdata;
}

static int ltr558_als_read(struct ltr558_data *ltr,int gainrange)
{
    int alsval_ch0_lo, alsval_ch0_hi, alsval_ch0;
    int alsval_ch1_lo, alsval_ch1_hi, alsval_ch1;
    int ratio = 0; 
    int ch0_x = 0;
    int ch1_x = 0;
    int lux = 0 ;
    alsval_ch1_lo = get_ltr_register(ltr,LTR558_ALS_DATA_CH1_0);
    alsval_ch1_hi = get_ltr_register(ltr,LTR558_ALS_DATA_CH1_1);
    alsval_ch1 = (alsval_ch1_hi * 256) + alsval_ch1_lo;
    
    alsval_ch0_lo = get_ltr_register(ltr,LTR558_ALS_DATA_CH0_0);
    alsval_ch0_hi = get_ltr_register(ltr,LTR558_ALS_DATA_CH0_1);
    alsval_ch0 = (alsval_ch0_hi * 256) + alsval_ch0_lo;

    ratio = alsval_ch1 * 100 / (alsval_ch0 + alsval_ch1);
    LTR558_DBG("ratio = %d,alsval_ch0 = %d,alsval_ch1 = %d\n",ratio,alsval_ch0,alsval_ch1);
    /*the parameter is provided by FAE*/
    if (ratio < 45)
    {
        ch0_x =  1774;
        ch1_x = -1106;
    }
    else if(ratio >= 45 && ratio < 64)
    {
        ch0_x = 3773;
        ch1_x = 1336;
    }
    else if ((ratio >= 64) && (ratio < 85))
    {
        ch0_x = 1690;
        ch1_x = 169 ;
    }
    else if(ratio >= 85)
    {
        lux = 0;
    }
    lux = (alsval_ch0*ch0_x - alsval_ch1*ch1_x) / ALS_GAIN;
    LTR558_DBG("luxdata = %d\n",lux);
    return lux ;
}

static int ltr558_open(struct inode *inode, struct file *file)
{
    /* when the device is open use this if light open report -1 when proximity open then lock it*/
    int ret;
    if( light_device_minor == iminor(inode) )
    {
        PROXIMITY_LTR_DEBUG("%s:ltr light sensor open\n", __func__);
        ltr_first_read = 1;
    }

    if( proximity_device_minor == iminor(inode) )
    {
        PROXIMITY_LTR_DEBUG("%s:ltr proximity_device_minor == iminor(inode)\n", __func__);
        wake_lock( &proximity_wake_lock);
        /* 0 is close, 1 is far */
        input_report_abs(ltr_this_data->input_dev, ABS_DISTANCE, 1);
        input_sync(ltr_this_data->input_dev);
    }

    if (!ltr_open_flag)
    {
        ret  = set_ltr_register(ltr_this_data,LTR558_PS_CONTR, MODE_PS_StdBy);
        ret |= set_ltr_register(ltr_this_data,LTR558_ALS_CONTR, MODE_ALS_StdBy);
        if(ret < 0)
        {
            printk(KERN_ERR "%s:set MODE_PS_StdBy failed ,ret = %d\n", __func__,ret);
        }
        if (ltr_this_data->client->irq)   
        {
            enable_irq(ltr_this_data->client->irq);
        }
    }
    ltr_open_flag++;
	return nonseekable_open(inode, file);
}
static int ltr558_release(struct inode *inode, struct file *file)
{
    ltr_open_flag--;
    if( proximity_device_minor == iminor(inode) )
    {
        PROXIMITY_LTR_DEBUG("%s: proximity_device_minor == iminor(inode)\n", __func__);
        wake_unlock( &proximity_wake_lock);
    }
    if( light_device_minor == iminor(inode) )
    {
        PROXIMITY_LTR_DEBUG("%s: light_device_minor == iminor(inode)\n", __func__);
    }
    if (!ltr_open_flag)
    {
        int ret;
		ret = set_ltr_register(ltr_this_data,LTR558_PS_CONTR, MODE_PS_StdBy);
        if(ret < 0)
        {
            printk(KERN_ERR "%s:set MODE_PS_StdBy failed ,ret = %d\n", __func__,ret);
        }
        if (ltr_this_data->client->irq) 
        {
            disable_irq(ltr_this_data->client->irq);
        }
    }
    return 0;
}
static int
ltr558_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
     unsigned long arg)
{
    int ret = 0;
    int flag;
    int set_flag;
    void __user *argp = (void __user *)arg;
    switch (cmd)
    {
        case ECS_IOCTL_APP_SET_LFLAG:
        {
            LTR558_DBG("into set lflag!!!\n");
            if (copy_from_user(&flag, argp, sizeof(flag)))
            {
                return -EFAULT;
            }
            atomic_set(&l_flag, flag);
            set_flag = atomic_read(&l_flag) ? 1 : 0;
			/*set bit 1 of reg 0 by set_flag */
			if (set_flag)
			{
				ret = ltr558_als_enable(ltr_this_data,als_gainrange);
                if (ret < 0)
                {
                    printk(KERN_ERR "%s:enable TLR558 als failed ,ret = %d\n", __func__,ret);
                }   
			}
			else
			{
                LTR558_DBG("%s:disable light sensor\n", __func__);
				ret = ltr558_als_disable(ltr_this_data);
                if (ret < 0)
                {
                    printk(KERN_ERR "%s:LTR set diable LFLAG is error(%d)!", __func__, ret);
                }
			}
            break;
        }
        case ECS_IOCTL_APP_GET_LFLAG:
        {
            flag = atomic_read(&l_flag);
            printk(KERN_ERR "%s:get light flag\n", __func__);
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
            printk(KERN_ERR "%s:into ps_set \n", __func__);
            atomic_set(&p_flag, flag);
            set_flag = atomic_read(&p_flag) ? 1 : 0;		
			if (set_flag)
			{
				ret = ltr558_ps_enable(ltr_this_data,ps_gainrange);
                if (ret < 0)
                {
                    printk(KERN_ERR "%s:enable TLR558 ps failed . error(%d)!\n", __func__,ret);
                }  
			}
			else
			{
				ret = ltr558_ps_disable(ltr_this_data);
                if (ret < 0)
                {
                    printk(KERN_ERR "%s:LTR set ECS_IOCTL_APP_SET_PFLAG flag is error(%d)!", __func__, ret);
                }

			}
            break;
        }
        case ECS_IOCTL_APP_GET_PFLAG:
        {
            flag = atomic_read(&p_flag);
            printk(KERN_ERR "%s:get ps flag\n", __func__);
            if (copy_to_user(argp, &flag, sizeof(flag)))
            {
                return -EFAULT;
            }
            break;
        }
        case ECS_IOCTL_APP_SET_DELAY:
        {
            printk(KERN_ERR "%s:set delay", __func__);
            if (copy_from_user(&flag, argp, sizeof(flag)))
            {
                return -EFAULT;
            }
            if(flag)
                ltr_558_delay = flag;
            else
                ltr_558_delay = 20;/*20ms*/
            break;
            
        }
        case ECS_IOCTL_APP_GET_DELAY:
        {
            
            flag = ltr_558_delay;
            if (copy_to_user(argp, &flag, sizeof(flag)))
            {
                return -EFAULT;
            }
            break;
            
        }
        default:
        {
            break;
        }
    }
    return ret;
    
}
static void ltr558_work_func(struct work_struct *work)
{
  
    int als_ps_status;
    int pdata = 0;
    int cdata = 0;
    int cdata_high,cdata_low ;
    int lux = 0;
    int als_level = 0;
    int interrupt, newdata;
    int pthreshold_h=0, pthreshold_l;
    int temp = 0;
    int ret ,i;
    struct ltr558_data *ltr_data = container_of(work, struct ltr558_data, work);
    als_ps_status = get_ltr_register(ltr_data,LTR558_ALS_PS_STATUS);
    LTR558_DBG(KERN_ERR "%s: first als_ps_status=0x%x,\n",__func__,als_ps_status);

    interrupt = als_ps_status & INT_STATE;
    newdata = als_ps_status & DATA_STATE;
    
    LTR558_DBG("get into work func .\n");
    if( PS_INIT == (interrupt & PS_INIT) )
    {
        if ( PS_DATA_BIT == (newdata & PS_DATA_BIT))
        {
            pdata = ltr558_ps_read(ltr_data);
        }
        LTR558_DBG("pdata = %d  \n",pdata);  
        LTR558_DBG("ps_contl = 0x%x  \n",get_ltr_register( ltr_data, LTR558_PS_CONTR));
        if ((pdata + ltr_558_pwave_value) < min_proximity_value)
        {
            LTR558_DBG("readjust the min_proximity_value\n");
            min_proximity_value = pdata + ltr_558_pwave_value;
            temp = min_proximity_value + ltr_558_pwindows_value;
            ret = set_ltr_register(ltr_data,LTR558_PS_THRES_UP_0, temp & 0xff);
            ret |= set_ltr_register(ltr_data,LTR558_PS_THRES_UP_1, temp >> 8);
            if (ret < 0)
            {
                printk(KERN_ERR "%s:set LRT558_PILTL_UP register is error(%d)!", __func__, ret);
            }
            temp = min_proximity_value;
            ret  = set_ltr_register(ltr_data,LTR558_PS_THRES_LOW_0,temp & 0xff);
            ret |= set_ltr_register(ltr_data,LTR558_PS_THRES_LOW_1,temp >> 8 );
            if (ret < 0)
            {
                printk(KERN_ERR "%s:set LRT558_PILTL_LOW register is error(%d)!", __func__, ret);
            }
            
        }
		/*read the high threhold and the low threhold*/
        pthreshold_h = get_ltr_register(ltr_data, LTR558_PS_THRES_UP_1);
        temp = get_ltr_register(ltr_data, LTR558_PS_THRES_UP_0);
        pthreshold_h = (pthreshold_h << 8) + temp;
        
        pthreshold_l = get_ltr_register(ltr_data, LTR558_PS_THRES_LOW_1);
        temp = get_ltr_register(ltr_data, LTR558_PS_THRES_LOW_0);
        pthreshold_l = (pthreshold_l << 8) + temp;
        LTR558_DBG(KERN_ERR "%s:before pthreshold_h = %d, pthreshold_l = %d\n",__func__,pthreshold_h,pthreshold_l);
   
        /* if more than the value of  proximity high threshold we set*/
        if (pdata >= pthreshold_h) 
        {
            /* stop the close init */
            temp = LTR_558_MAX_PPDATA;
            ret  = set_ltr_register(ltr_data,LTR558_PS_THRES_UP_0, temp & 0xff);
            ret |= set_ltr_register(ltr_data,LTR558_PS_THRES_UP_1, temp >> 8 );
            if (ret <0)
            {
                printk(KERN_ERR "%s:set LTR558_PILTL_UP register is error(%d)!", __func__, ret);
            }
            temp = min_proximity_value;
            ret  = set_ltr_register(ltr_data,LTR558_PS_THRES_LOW_0,temp & 0xff);
            ret |= set_ltr_register(ltr_data,LTR558_PS_THRES_LOW_1,temp >> 8 );
            if (ret < 0)
            {
                printk(KERN_ERR "%s:set LTR558_PILTL_LOW register is error(%d)!", __func__, ret);
            }
            input_report_abs(ltr_data->input_dev, ABS_DISTANCE, 0);
            input_sync(ltr_data->input_dev);
        }
        /* if less than the value of  proximity low threshold we set*/
        /* the condition of pdata == pthreshold_l is valid */
        else if (pdata <=  pthreshold_l)
        {
            /*stop the leave int*/
            temp = min_proximity_value + ltr_558_pwindows_value;
            ret  = set_ltr_register(ltr_data,LTR558_PS_THRES_UP_0, temp & 0xff);
            ret |= set_ltr_register(ltr_data,LTR558_PS_THRES_UP_1, temp >> 8);  
            ret |= set_ltr_register(ltr_data,LTR558_PS_THRES_LOW_0,0);
            ret |= set_ltr_register(ltr_data,LTR558_PS_THRES_LOW_1,0);
            if (ret < 0)
            {
                printk(KERN_ERR "%s:set LTR558_PILTL_LOW register is error(%d)!", __func__, ret);
            }
            input_report_abs(ltr_data->input_dev, ABS_DISTANCE, 1);
            input_sync(ltr_data->input_dev);
        }   
        pthreshold_h = get_ltr_register(ltr_data, LTR558_PS_THRES_UP_1);
        temp = get_ltr_register(ltr_data, LTR558_PS_THRES_UP_0);
        pthreshold_h = (pthreshold_h << 8) + temp;
        
        pthreshold_l = get_ltr_register(ltr_data, LTR558_PS_THRES_LOW_1);
        temp = get_ltr_register(ltr_data, LTR558_PS_THRES_LOW_0);
        pthreshold_l = (pthreshold_l << 8) + temp;
        LTR558_DBG(KERN_ERR "%s:after set ,pthreshold_h = %d, pthreshold_l = %d\n",__func__,pthreshold_h,pthreshold_l);
   
    }
    if( ALS_INIT == (interrupt & ALS_INIT) ) 
    {
        if ( ALS_DATA_BIT == (newdata & ALS_DATA_BIT))
        {
            lux = ltr558_als_read(ltr_data,als_gainrange);
        }
        cdata = lux ;
        /*40 is becaus ALS_GAIN*40 = 1000*/
        cdata_high = (cdata *  600)/500/40;
        cdata_low = (cdata *  400)/500/40;
        if(cdata_high == cdata_low)
            cdata_low = cdata_high + 1;
        LTR558_DBG(KERN_ERR "%s:cdata_high = %d,cdata_low = %d\n",__func__,cdata_high,cdata_low);
        ret = set_ltr_register(ltr_data,LTR558_ALS_THRES_UP_0, cdata_high & 0xff);
        ret |= set_ltr_register(ltr_data,LTR558_ALS_THRES_UP_1, cdata_high >> 8);
        ret |= set_ltr_register(ltr_data,LTR558_ALS_THRES_LOW_0, cdata_low & 0xff);
        ret |= set_ltr_register(ltr_data,LTR558_ALS_THRES_LOW_1, cdata_low >> 8);
        if (ret)
        {
            printk(KERN_ERR "%s:set ltr558 als_set register is error(%d)!", __func__, ret);
        }
    
   #if 0    
        pthreshold_h = get_ltr_register(ltr_data, LTR558_ALS_THRES_UP_1);
        temp = get_ltr_register(ltr_data, LTR558_ALS_THRES_UP_0);
        pthreshold_h = (pthreshold_h << 8) + temp;
        
        pthreshold_l = get_ltr_register(ltr_data, LTR558_ALS_THRES_LOW_1);
        temp = get_ltr_register(ltr_data, LTR558_ALS_THRES_LOW_0);
        pthreshold_l = (pthreshold_l << 8) + temp;
        printk(KERN_ERR "%s:ALS pthreshold_h = %d, pthreshold_l = %d\n",__func__,pthreshold_h,pthreshold_l);
   #endif     

        if (lux >= 0)
        {
	        /* lux=0 is valid */
	        als_level = LSENSOR_MAX_LEVEL - 1;
			for (i = 0; i < ARRAY_SIZE(ltrsensor_adc_table); i++)
            {
				if (lux < ltrsensor_adc_table[i])
                {
					als_level = i;
					break;
				}
			}
	        LTR558_DBG("%s:lux=%d,als_level=%d\n",__func__,lux,als_level);
            if(ltr_first_read)
            {
                ltr_first_read = 0;
                input_report_abs(ltr_data->input_dev, ABS_LIGHT, -1);
                input_sync(ltr_data->input_dev);
            }
            else
            {
				input_report_abs(ltr_data->input_dev, ABS_LIGHT, als_level);
                input_sync(ltr_data->input_dev);
            }
        }
        /* if lux<0,we need to change the gain which we can set register 0x0f */
        else 
        {
            printk("Need to change gain %2d \n", lux);
        }
    }
    if (ltr_data->client->irq)
    {
        enable_irq(ltr_data->client->irq);
    }
    als_ps_status = get_ltr_register(ltr_data,LTR558_ALS_PS_STATUS);
    LTR558_DBG("%s:als_ps_status=0x%x,\n",__func__,als_ps_status);
    LTR558_DBG("get out of work func.\n");
   
}

static struct file_operations ltr558_fops = {
    .owner = THIS_MODULE,
    .open = ltr558_open,
    .release = ltr558_release,
    .ioctl = ltr558_ioctl,
};
static struct miscdevice ltr558_ps_dev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "proximity",
    .fops = &ltr558_fops,
};

static struct miscdevice ltr558_als_dev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "light",
    .fops = &ltr558_fops,
};


irqreturn_t ltr_irq_handler(int irq, void *dev_id)
{
    struct ltr558_data *ltr = dev_id;
    disable_irq_nosync(ltr->client->irq);
    queue_work(ltr_wq, &ltr->work);
    return IRQ_HANDLED;
}


static int ltr558_probe(struct i2c_client *client, const struct i2c_device_id *id)
{ 
    int ret = 0;
    struct ltr558_data *ltr; 
	/* < DTS2011110704273  zhangmin 20111107 begin */
    int value = 0;
	/* DTS2011110704273  zhangmin 20111107  end > */
    struct ltr558_hw_platform_data *platform_data = NULL;
    
    if (client->dev.platform_data == NULL)
    {
        PROXIMITY_LTR_DEBUG("%s:ltr_platform data is NULL. exiting.\n", __func__);
        ret = -ENODEV;
        goto err_exit;
    }
    platform_data = client->dev.platform_data;
    
    if (platform_data->ltr558_power)
    {
        ret = platform_data->ltr558_power(IC_PM_ON);
        if (ret < 0)
        {
            pr_err("%s:ltr558 power on error!\n", __func__);
            goto err_exit ;
        }
    }
    //mdelay(5);
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        printk(KERN_ALERT "%s: LTR-558 functionality check failed.\n", __func__);
        ret = -ENODEV;
        goto err_check_functionality_failed;
    }
    ltr = kzalloc(sizeof(*ltr), GFP_KERNEL);
    if (ltr == NULL) 
    {
        printk(KERN_ALERT "%s: LTR-558ALS kzalloc failed.\n", __func__);
        ret = -ENOMEM;
        goto err_alloc_data_failed;
    }
    mutex_init(&ltr->mlock);
    INIT_WORK(&ltr->work, ltr558_work_func);
    ltr->client = client;
    i2c_set_clientdata(client, ltr);
    /* the delay time should be re-affirm with FAE*/
    //mdelay(PON_DELAY);
	/* < DTS2011110704273  zhangmin 20111107 begin */
	/*add part_id to avoid read error*/
    value = get_ltr_register(ltr,LTR558_PART_ID);
    if(value != PART_ID)
    {
        printk("Failed to get LTR558_PART_ID!\n");
        ret = -EIO;
        goto err_detect_failed;
    }
	/* DTS2011110704273  zhangmin 20111107  end > */
    
    if (client->irq) 
    {
        if (platform_data->ltr558_gpio_config_interrupt)
        {
            ret = platform_data->ltr558_gpio_config_interrupt();
            if (ret) 
            {
                printk(KERN_ERR "gpio_tlmm_config error\n");
                goto err_gpio_config_failed;
            }
        }  
        if (request_irq(client->irq, ltr_irq_handler,IRQF_TRIGGER_LOW, client->name, ltr) >= 0) 
        {
            PROXIMITY_LTR_DEBUG("Received IRQ!\n");
    	    disable_irq(ltr->client->irq);
        }
        else 
        {
            printk("Failed to request IRQ!\n");
        }
        ret  = set_ltr_register(ltr,LTR558_PS_THRES_UP_0, 0x6c);
        ret |= set_ltr_register(ltr,LTR558_PS_THRES_UP_1, 0x07);
        ret |= set_ltr_register(ltr,LTR558_PS_THRES_LOW_0, 0x6B);
        ret |= set_ltr_register(ltr,LTR558_PS_THRES_LOW_1, 0x07);
        if (ret < 0)
        {
            printk(KERN_ERR "%s:set the threshold of proximity  failed(%d)!", __func__, ret);
        }
		ret  = set_ltr_register(ltr,LTR558_ALS_THRES_UP_0, 0x01);
        ret |= set_ltr_register(ltr,LTR558_ALS_THRES_UP_1, 0x01);
        ret |= set_ltr_register(ltr,LTR558_ALS_THRES_LOW_0, 0x00);
        ret |= set_ltr_register(ltr,LTR558_ALS_THRES_LOW_1, 0x01);
        if (ret < 0)
        {
            printk(KERN_ERR "%s:set the threshold of ALS failed(%d)!", __func__, ret);
        }
        ps_gainrange = PS_RANGE16;
        als_gainrange = ALS_RANGE2_64K;
        ret   = set_ltr_register(ltr,LTR558_PS_LED, 0xE7);
        ret  |= set_ltr_register(ltr,LTR558_PS_CONTR, 0x0C);
        ret  |= set_ltr_register(ltr,LTR558_INTERRUPT_PERSIST, 0x01);
        ret  |= set_ltr_register(ltr,LTR558_ALS_MEAS_RATE,0x01);
        if (ret < 0)
        {
            printk(KERN_ERR "%s:set the threshold of ALS failed(%d)!", __func__, ret);
        }
    }
    /* if not define irq,then error */
    else
    {
        printk(KERN_ERR "please set the irq num!\n");
        goto err_detect_failed;
    }
    if (sensor_ltr_dev == NULL) 
    {
        ltr->input_dev = input_allocate_device();
        if (ltr->input_dev == NULL) 
        {
            ret = -ENOMEM;
            PROXIMITY_LTR_DEBUG(KERN_ERR "ltr_probe: Failed to allocate input device\n");
            goto err_input_dev_alloc_failed;
        }
        ltr->input_dev->name = "sensors_aps";
        ltr->input_dev->id.bustype = BUS_I2C;
        input_set_drvdata(ltr->input_dev, ltr);
        
        ret = input_register_device(ltr->input_dev);
        if (ret) 
        {
            printk(KERN_ERR "ltr_probe: Unable to register %s input device\n", ltr->input_dev->name);
            goto err_input_register_device_failed;
        }
        sensor_ltr_dev = ltr->input_dev;
    } 
    else 
    {
        printk(KERN_INFO "sensor_ltr_dev is not null\n");
        ltr->input_dev = sensor_ltr_dev;
    }
    set_bit(EV_ABS, ltr->input_dev->evbit);
    // shoule be adjust
    input_set_abs_params(ltr->input_dev, ABS_LIGHT, 0, 10240, 0, 0);//????
    input_set_abs_params(ltr->input_dev, ABS_DISTANCE, 0, 1, 0, 0);

    ret = misc_register(&ltr558_ps_dev);
    if (ret) 
    {
        printk(KERN_ALERT "%s: LTR-558ALS misc_register ps failed.\n", __func__);
        goto err_proximity_misc_device_register_failed;
    }
    ret = misc_register(&ltr558_als_dev);
    if (ret) 
    {
        printk(KERN_ALERT "%s: LTR-558ALS misc_register ps failed.\n", __func__);
        goto err_light_misc_device_register_failed;
    }
    if( ltr558_ps_dev.minor != MISC_DYNAMIC_MINOR ){
        proximity_device_minor = ltr558_ps_dev.minor ;
    }
    
    if( ltr558_als_dev.minor != MISC_DYNAMIC_MINOR ){
        light_device_minor = ltr558_als_dev.minor;
    }
    wake_lock_init(&proximity_wake_lock, WAKE_LOCK_SUSPEND, "proximity");
    ltr_wq = create_singlethread_workqueue("ltr_wq");
    if (!ltr_wq) 
    {
        ret = -ENOMEM;
        goto err_create_workqueue_failed;
    }
    ltr_this_data = ltr;
    #ifdef CONFIG_HUAWEI_HW_DEV_DCT
    /* detect current device successful, set the flag as present */
    set_hw_dev_flag(DEV_I2C_APS);
    #endif
    printk(KERN_INFO "ltr558_probe: Start Proximity Sensor ltr558\n");

    return 0;
    
err_create_workqueue_failed:
    misc_deregister(&ltr558_ps_dev);
err_proximity_misc_device_register_failed:
    misc_deregister(&ltr558_als_dev);
err_light_misc_device_register_failed:
err_input_register_device_failed:
    input_free_device(ltr->input_dev);
err_input_dev_alloc_failed:
err_gpio_config_failed:
err_detect_failed:
    kfree(ltr);
err_alloc_data_failed:
err_check_functionality_failed:
    /* < DTS2011110704273  zhangmin 20111107 begin */
	/*power off for ltr558*/
    if(platform_data->ltr558_power)
    {
        platform_data->ltr558_power(IC_PM_OFF);
    }
    /* DTS2011110704273  zhangmin 20111107  end > */
err_exit:
    return ret;

}
static int ltr558_remove(struct i2c_client *client)
{
    struct ltr558_data *ltr = i2c_get_clientdata(client);

    if (ltr->client->irq)
    {
        disable_irq(ltr->client->irq);
    }
    free_irq(client->irq, ltr);
    misc_deregister(&ltr558_als_dev);
    misc_deregister(&ltr558_ps_dev);
    input_unregister_device(ltr->input_dev);
    
    kfree(ltr);
    return 0;
}
static int ltr558_suspend(struct i2c_client *client, pm_message_t mesg)
{
    int ret;
    struct ltr558_data *ltr = i2c_get_clientdata(client);
    PROXIMITY_LTR_DEBUG(KERN_ERR "%s:ltr558_suspend enter\n ", __func__);
    if (ltr->client->irq)
    {
        disable_irq(ltr->client->irq);
    }
    ret  = cancel_work_sync(&ltr->work);
    ps_reg0  = get_ltr_register(ltr, LTR558_PS_CONTR);
    als_reg0 = get_ltr_register(ltr, LTR558_ALS_CONTR);
    ret  = ltr558_als_disable(ltr);
    ret |= ltr558_ps_disable(ltr);
    if (ret < 0)
        printk(KERN_ERR "%s:set LTR558_ENABLE_REG register[PON=OFF] failed(%d)!", __func__, ret);
    return 0;
}
static int ltr558_resume(struct i2c_client *client)
{
    int ret = 1;
    struct ltr558_data *ltr = i2c_get_clientdata(client);
    PROXIMITY_LTR_DEBUG("LTR558_resume enter\n ");
    ret  = set_ltr_register(ltr,LTR558_PS_CONTR, ps_reg0);
    ret |= set_ltr_register(ltr,LTR558_ALS_CONTR, als_reg0);
    mdelay(WAKEUP_DELAY);
    if (ltr->client->irq)
    {
        enable_irq(ltr->client->irq);
    }
    return 0;
}
static const struct i2c_device_id ltr558_id[] = {
    { "ltr-558", 0 },
    {}
};
static struct i2c_driver ltr558_driver = {
    .probe = ltr558_probe,
    .remove = ltr558_remove,
    .suspend = ltr558_suspend,
    .resume = ltr558_resume,
    .id_table = ltr558_id,
    .driver = {
		.name	="ltr-558",
	},
   
};
static int __init ltr_558_init(void)
{
    return i2c_add_driver(&ltr558_driver);
}
static void __exit ltr_558_exit(void)
{
    i2c_del_driver(&ltr558_driver);
}
/* < DTS2011110704273  zhangmin 20111107 begin */
device_initcall_sync(ltr_558_init);
module_exit(ltr_558_exit);
/* DTS2011110704273  zhangmin 20111107  end > */
MODULE_DESCRIPTION("Proximity Driver");
MODULE_LICENSE("GPL");
/* DTS2011102806486  zhangmin 20111029  end > */
