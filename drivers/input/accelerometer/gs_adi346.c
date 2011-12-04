/* < DTS2011010404642 wuzhihui 20110104 begin */
/* drivers/input/accelerometer/gs_adi346.c
 *
 * Copyright (C) 2010-2011  Huawei.
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
#include "linux/hardware_self_adapt.h"
#include <linux/slab.h>
#include <mach/vreg.h>

#include <linux/gs_adxl345.h>

/* <DTS2011021804534 shenjinming 20110218 begin */
#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <linux/hw_dev_dec.h>
#endif
/* DTS2011021804534 shenjinming 20110218 end> */

//#define ENABLE_GS_DEBUG
//#undef ENABLE_GS_DEBUG

#ifdef ENABLE_GS_DEBUG
#define GS_DEBUG(fmt, args...) printk(KERN_ERR fmt, ##args)
#else
#define GS_DEBUG(fmt, args...)
#endif

#define GS_POLLING   		1

#define ADI346_DRV_NAME		"gs_adi346"

/* DEVIDs */
#define ID_ADXL345	0xE5
#define ID_ADXL346	0xE6

#define DATA_READY	(1 << 7)
#define SINGLE_TAP	(1 << 6)
#define DOUBLE_TAP	(1 << 5)
#define ACTIVITY	(1 << 4)
#define INACTIVITY	(1 << 3)
#define FREE_FALL	(1 << 2)
#define WATERMARK	(1 << 1)
#define OVERRUN		(1 << 0)

/* ACT_INACT_CONTROL Bits */
#define ACT_ACDC	(1 << 7)
#define ACT_X_EN	(1 << 6)
#define ACT_Y_EN	(1 << 5)
#define ACT_Z_EN	(1 << 4)
#define INACT_ACDC	(1 << 3)
#define INACT_X_EN	(1 << 2)
#define INACT_Y_EN	(1 << 1)
#define INACT_Z_EN	(1 << 0)

/* TAP_AXES Bits */
#define SUPPRESS	(1 << 3)
#define TAP_X_EN	(1 << 2)
#define TAP_Y_EN	(1 << 1)
#define TAP_Z_EN	(1 << 0)

/* ACT_TAP_STATUS Bits */
#define ACT_X_SRC	(1 << 6)
#define ACT_Y_SRC	(1 << 5)
#define ACT_Z_SRC	(1 << 4)
#define ASLEEP		(1 << 3)
#define TAP_X_SRC	(1 << 2)
#define TAP_Y_SRC	(1 << 1)
#define TAP_Z_SRC	(1 << 0)

/* BW_RATE Bits */
#define LOW_POWER	(1 << 4)
#define RATE(x)		((x) & 0xF)

/* POWER_CTL Bits */
#define PCTL_LINK	(1 << 5)
#define PCTL_AUTO_SLEEP (1 << 4)
#define PCTL_MEASURE	(1 << 3)
#define PCTL_SLEEP	(1 << 2)
#define PCTL_WAKEUP(x)	((x) & 0x3)

/* DATA_FORMAT Bits */
#define SELF_TEST	(1 << 7)
#define SPI		(1 << 6)
#define INT_INVERT	(1 << 5)
#define FULL_RES	(1 << 3)
#define JUSTIFY		(1 << 2)
#define RANGE(x)	((x) & 0x3)
#define RANGE_PM_2g	0
#define RANGE_PM_4g	1
#define RANGE_PM_8g	2
#define RANGE_PM_16g	3





static unsigned model;

enum {
	MODE_2G = 0,
	MODE_4G,
	MODE_8G,
};

/* < DTS2011071300230  liujinggang 20110810 begin */
#define MG_PER_SAMPLE		720		/*HAL: 720=1g*/
//#define FILTER_SAMPLE_NUMBER	128
#define FILTER_SAMPLE_NUMBER	256		/* 256 = 1g */
#define	GPIO_INT1		19
#define GPIO_INT2		20
#define GS_ST_TIMRER		(1000)		/*1000ms*/
/* DTS2011071300230  liujinggang 20110810 end > */

#define ECS_IOCTL_READ_ACCEL_XYZ			_IOR(0xA1, 0x06, char[3])
#define ECS_IOCTL_APP_SET_DELAY 			_IOW(0xA1, 0x18, short)
#define ECS_IOCTL_APP_GET_DELAY 			_IOR(0xA1, 0x30, short)
#define ECS_IOCTL_APP_SET_AFLAG 			_IOW(0xA1, 0x13, short)
#define ECS_IOCTL_APP_GET_AFLAG				_IOR(0xA1, 0x14, short)
#define ECS_IOCTL_READ_DEVICEID				_IOR(0xA1, 0x31, char[20])

struct gs_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_irq;
	struct mutex  mlock;
	struct hrtimer timer;
	struct work_struct  work;	
	uint32_t flags;
	struct early_suspend early_suspend;
};


static struct gs_data  *this_gs_data;

static struct workqueue_struct *gs_wq;
static signed short st_sensor_data[3];
static char gs_device_id[] = ADI346_DRV_NAME;

struct input_dev *sensor_dev = NULL;

static int accel_delay = GS_ST_TIMRER;     /*1s*/

static atomic_t a_flag;

#ifdef CONFIG_HAS_EARLYSUSPEND
static void gs_early_suspend(struct early_suspend *h);
static void gs_late_resume(struct early_suspend *h);
#endif
/* < DTS2011072105664 xiangxu  20110722 begin */
static inline int reg_read(struct gs_data *gs , int reg);
static int adi346_debug_mask;
module_param_named(adi346_debug, adi346_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

#define ADI346_DBG(x...) do {\
    if (adi346_debug_mask) \
        printk(KERN_DEBUG x);\
    } while (0)
#define adi346_PRINT_PER_TIMES 100
unsigned int adi346_times = 0;

void adi346_print_debug(int start_reg,int end_reg)
{
	int reg, ret;

	for(reg = start_reg ; reg <= end_reg ; reg ++)
	{
		/* read reg value */
		ret = reg_read(this_gs_data,reg);
		/* print reg info */
		ADI346_DBG("adi346 reg 0x%x values 0x%x\n",reg,ret);
	}
}

/* DTS2011072105664 xiangxu  20110722 end > */
/* < DTS2011011905410   liujinggang 20110119 begin */
static compass_gs_position_type  compass_gs_position=COMPASS_TOP_GS_TOP;
/* DTS2011011905410   liujinggang 20110119 end > */
static int adxl34x_i2c_read_block(struct i2c_client *client,
				unsigned char reg, int count,
				void *buf)
{
	int ret;

	ret = i2c_master_send(client, &reg, 1);
	if (ret < 0)
		return ret;

	ret = i2c_master_recv(client, buf, count);
	if (ret < 0)
		return ret;

	if (ret != count)
		return -EIO;

	return 0;
}

/**************************************************************************************/
static inline int reg_read(struct gs_data *gs , int reg)
{
	int val;

	mutex_lock(&gs->mlock);

	val = i2c_smbus_read_byte_data(gs->client, reg);
	if (val < 0)
		printk(KERN_ERR "ADI346 chip i2c %s failed\n", __FUNCTION__);

	mutex_unlock(&gs->mlock);

	return val;
}
static inline int reg_write(struct gs_data *gs, int reg, uint8_t val)
{
	int ret;

	mutex_lock(&gs->mlock);
	ret = i2c_smbus_write_byte_data(gs->client, reg, val);
	if(ret < 0) {
		printk(KERN_ERR "ADI346 chip i2c %s failed\n", __FUNCTION__);
	}
	mutex_unlock(&gs->mlock);

	return ret;
}

/**************************************************************************************/

static int gs_data_to_compass(signed short accel_data [3])
{
	memset((void*)accel_data, 0, sizeof(accel_data));
	accel_data[0]=st_sensor_data[0];
	accel_data[1]=st_sensor_data[1];
	accel_data[2]=st_sensor_data[2];
	return 0;
}

/**************************************************************************************/

/* < DTS2011071300230  liujinggang 20110810 begin */
/*set register*/
static int gs_st_open(struct inode *inode, struct file *file)
{	
	/*gs active mode*/
	reg_write(this_gs_data, GS_ADI_REG_OFSX, 0); /* offset: 0g */
	reg_write(this_gs_data, GS_ADI_REG_OFSY, 0); /* offset: 0g */
	reg_write(this_gs_data, GS_ADI_REG_OFSZ, 0); /* offset: 0g */
	reg_write(this_gs_data,GS_ADI_REG_BW,0x0b);    /* Rate: 200Hz, IDD: 130uA */
	reg_write(this_gs_data,GS_ADI_REG_DATA_FORMAT,0x0B);/* Data Format: 16g right justified  256=1g*/
	reg_write(this_gs_data, GS_ADI_REG_POWER_CTL, 0x08);  

	if (this_gs_data->use_irq)
		enable_irq(this_gs_data->client->irq);
	else
		hrtimer_start(&this_gs_data->timer, ktime_set(0, 500000000), HRTIMER_MODE_REL);

	return nonseekable_open(inode, file);
}

static int gs_st_release(struct inode *inode, struct file *file)
{
	/*gs standby mode*/
	reg_write(this_gs_data, GS_ADI_REG_POWER_CTL, 0);
	

	if (this_gs_data->use_irq)
		disable_irq(this_gs_data->client->irq);
	else
		hrtimer_cancel(&this_gs_data->timer);

	accel_delay = GS_ST_TIMRER;	  

	return 0;
}
/* DTS2011071300230  liujinggang 20110810 end > */

static int
gs_st_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
	   unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	signed short accel_buf[3];
	short flag;
	
	switch (cmd) 
	{
		case ECS_IOCTL_APP_SET_AFLAG:     /*set open acceleration sensor flag*/
			if (copy_from_user(&flag, argp, sizeof(flag)))
				return -EFAULT;
			break;
				
		case ECS_IOCTL_APP_SET_DELAY:
			if (copy_from_user(&flag, argp, sizeof(flag)))
				return -EFAULT;
			break;
		
			default:
				break;
	}
	switch (cmd) 
	{
		case ECS_IOCTL_APP_SET_AFLAG:
			atomic_set(&a_flag, flag);
			break;
			
		case ECS_IOCTL_APP_GET_AFLAG:  /*get open acceleration sensor flag*/
			flag = atomic_read(&a_flag);
			break;
			
		case ECS_IOCTL_APP_SET_DELAY:
			if(flag)
				accel_delay = flag;
			else
				accel_delay = 10;   /*10ms*/
			break;
			
		case ECS_IOCTL_APP_GET_DELAY:
			flag = accel_delay;
			break;
			
		case ECS_IOCTL_READ_ACCEL_XYZ:
			gs_data_to_compass(accel_buf);
			break;
			
		default:
			break;
	}
	switch (cmd) 
	{
		case ECS_IOCTL_APP_GET_AFLAG:
			if (copy_to_user(argp, &flag, sizeof(flag)))
				return -EFAULT;
			break;

		case ECS_IOCTL_APP_GET_DELAY:
			if (copy_to_user(argp, &flag, sizeof(flag)))
				return -EFAULT;
			break;
			
		case ECS_IOCTL_READ_ACCEL_XYZ:
			if (copy_to_user(argp, &accel_buf, sizeof(accel_buf)))
				return -EFAULT;
			break;
			
		case ECS_IOCTL_READ_DEVICEID:
			if (copy_to_user(argp, gs_device_id, sizeof(gs_device_id)))
				return -EFAULT;
			break;
			
		default:
			break;
	}
	return 0;
}

static struct file_operations gs_st_fops = {
	.owner = THIS_MODULE,
	.open = gs_st_open,
	.release = gs_st_release,
	.ioctl = gs_st_ioctl,
};

static struct miscdevice gsensor_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "accel",
	.fops = &gs_st_fops,
};

static void gs_work_func(struct work_struct *work)
{
	/* < DTS2011071300230  liujinggang 20110810 begin */
	/*modify the data processing*/
	short buf[3];
	int x = 0, y = 0, z = 0;
	short x16 = 0, y16 = 0, z16 = 0;

	int sesc = accel_delay/1000;
	int nsesc = (accel_delay%1000)*1000000;	
	struct gs_data *gs = container_of(work, struct gs_data, work);
       
	adxl34x_i2c_read_block(gs->client, GS_ADI_REG_DATAX0, GS_ADI_REG_DATAZ1 - GS_ADI_REG_DATAX0 + 1, buf);

	x16 = buf[0];
	y16 = buf[1];
	z16 = buf[2];

	/* < DTS2011072105664  xiangxu 20110722 begin*/
	ADI346_DBG("Gs_adi346:A  x : %d y : %d z : %d \n", x16,y16,z16);
	/*  DTS2011072105664   xiangxu 20110722 end > */
	x = x16*MG_PER_SAMPLE/FILTER_SAMPLE_NUMBER;
	y = y16*MG_PER_SAMPLE/FILTER_SAMPLE_NUMBER;
	z = z16*MG_PER_SAMPLE/FILTER_SAMPLE_NUMBER;

	/* < DTS2011011905410   liujinggang 20110119 begin */
	/* < DTS2011030405439 weiheng 20110307 begin */
	/*report different values by machines*/
	if((compass_gs_position==COMPASS_TOP_GS_BOTTOM)||(compass_gs_position==COMPASS_BOTTOM_GS_BOTTOM)||(compass_gs_position==COMPASS_NONE_GS_BOTTOM))
	{
		//inverse
		y*=(-1);
	}
	else
	{
		/*
		if((compass_gs_position==0)||(compass_gs_position==2))
		*/
		//obverse
		z*=(-1);
	}
	/* DTS2011030405439 weiheng 20110307 end > */
	/* DTS2011011905410   liujinggang 20110119 end > */

	input_report_abs(gs->input_dev, ABS_X, y);
	input_report_abs(gs->input_dev, ABS_Y, x);			
	input_report_abs(gs->input_dev, ABS_Z, z);
	GS_DEBUG("Gs_adi346:A  x :0x%x y :0x%x z :0x%x \n", x,y,z);
	input_sync(gs->input_dev);

	/*
	* There is a transform formula between ABS_X, ABS_Y, ABS_Z
	* and Android_X, Android_Y, Android_Z.
	*                      -          -
	*                      			|  0 -1  0 |
	* [ABS_X ABS_Y ABS_Z]*   |  1  0  0 | = [Android_X, Android_Y, Android_Z]
	*                      			|  0  0 -1 |
	*                      -          -
	* compass uses Android_X, Andorid_Y, Android_Z
	*/
	memset((void*)st_sensor_data, 0, sizeof(st_sensor_data));
	st_sensor_data[0]= -y;
	st_sensor_data[1]= x;
	st_sensor_data[2]= -z;
		

	/* DTS2011071300230  liujinggang 20110810 end > */
	/* < DTS2011072105664 xiangxu 20110722 begin */
	ADI346_DBG("Gs_adi346:A  x : %d y : %d z : %d \n", x,y,z);
	if(adi346_debug_mask)
	{
		/* print reg info in such times */
		if(!(++adi346_times%adi346_PRINT_PER_TIMES))
		{
			/* count return to 0 */
			adi346_times = 0;
			adi346_print_debug(GS_ADI_REG_THRESH_TAP,GS_ADI_REG_ORIENT);
		}
	}
	/* DTS2011072105664 xiangxu 20110722 end > */
	if (gs->use_irq)
		enable_irq(gs->client->irq);
	else
		hrtimer_start(&gs->timer, ktime_set(sesc, nsesc), HRTIMER_MODE_REL);
	
}


static enum hrtimer_restart gs_timer_func(struct hrtimer *timer)
{
	struct gs_data *gs = container_of(timer, struct gs_data, timer);		
	queue_work(gs_wq, &gs->work);
	//hrtimer_start(&gs->timer, ktime_set(0, 512500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

#ifndef GS_POLLING 	
static irqreturn_t gs_irq_handler(int irq, void *dev_id)
{
	struct gs_data *gs = dev_id;
	disable_irq(gs->client->irq);
	queue_work(gs_wq, &gs->work);
	return IRQ_HANDLED;
}

static int gs_config_int_pin(void)
{
	int err;

	err = gpio_request(GPIO_INT1, "gpio_gs_int");
	if (err) 
	{
		printk(KERN_ERR "gpio_request failed for st gs int\n");
		return -1;
	}	

	err = gpio_configure(GPIO_INT1, GPIOF_INPUT | IRQF_TRIGGER_HIGH);
	if (err) 
	{
		gpio_free(GPIO_INT1);
		printk(KERN_ERR "gpio_config failed for gs int HIGH\n");
		return -1;
	}     

	return 0;
}

static void gs_free_int(void)
{
	gpio_free(GPIO_INT1);
}
#endif /*GS_POLLING*/

static int gs_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{	
	int ret = 0;
	struct gs_data *gs;
	unsigned char revid;
	/* < DTS2011011905410   liujinggang 20110119 begin */
	struct gs_platform_data *pdata = NULL;
 	/* < DTS2011043000257  liujinggang 20110503 begin */
	/*delete 20 lines*/
	/* DTS2011043000257  liujinggang 20110503 end > */
	    
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "gs_adi346_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	/* < DTS2011043000257  liujinggang 20110503 begin */
	/*turn on the power*/
	pdata = client->dev.platform_data;
	if (pdata){
		if(pdata->gs_power != NULL){
			ret = pdata->gs_power(IC_PM_ON);
			if(ret < 0 ){
				goto err_check_functionality_failed;
			}
		}
				
		if(pdata->adapt_fn != NULL){
			ret = pdata->adapt_fn();
			if(ret > 0){
				client->addr = pdata->slave_addr;//actual address
				printk(KERN_INFO "%s:change i2c addr to actrual address = %d\n", __FUNCTION__, pdata->slave_addr);
				if(client->addr == 0){
					printk(KERN_ERR "%s: bad i2c address = %d\n", __FUNCTION__, client->addr);
					ret = -EFAULT;
					goto err_power_failed;
				}
			}
		}
		
		if(pdata->get_compass_gs_position != NULL){
			compass_gs_position=pdata->get_compass_gs_position();
		}
			

		if(pdata->init_flag != NULL){
			if(*(pdata->init_flag)){
				printk(KERN_ERR "gs_adi346 probe failed, because the othe gsensor has been probed.\n");
				ret = -ENODEV;
				goto err_power_failed;
			}
		}
	}
	/* DTS2011011905410   liujinggang 20110119 end > */
	client->addr = 0x53;//8bit address is 0xA6

#ifndef   GS_POLLING 	
	ret = gs_config_int_pin();
	if(ret <0)
	{
		goto err_power_failed;
	}
#endif
	/* DTS2011043000257  liujinggang 20110503 end > */

	gs = kzalloc(sizeof(*gs), GFP_KERNEL);
	if (gs == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	mutex_init(&gs->mlock);

	INIT_WORK(&gs->work, gs_work_func);
	gs->client = client;
	i2c_set_clientdata(client, gs);

	revid=reg_read(gs,GS_ADI_REG_DEVID);

	printk(KERN_INFO "%s:     %d     revid===%d\n",__func__,__LINE__,revid);

	switch (revid) {
	case ID_ADXL346:
		model = 346;
		break;
	default:
		printk(KERN_ERR "Failed to probe \n" );
		goto err_detect_failed;
	}

	/* < DTS2011071300230  liujinggang 20110810 begin */
	/*modify the initializtion*/

	ret = reg_write(gs, GS_ADI_REG_OFSX, 0); /* offset: 0g */
	if ( ret <0 )
		goto err_detect_failed;

	ret = reg_write(gs, GS_ADI_REG_OFSY, 0); /* offset: 0g */
	if ( ret <0 )
		goto err_detect_failed;

	ret = reg_write(gs, GS_ADI_REG_OFSZ, 0); /* offset: 0g */
	if ( ret <0 )
		goto err_detect_failed;	

	ret = reg_write(gs,GS_ADI_REG_BW,0x0b);    /* Rate: 200Hz, IDD: 130uA */
	if ( ret <0 )
		goto err_detect_failed;

	ret = reg_write(gs,GS_ADI_REG_DATA_FORMAT,0x0B);/* Data Format: 16g right justified  256=1g*/
	if ( ret <0 )
		goto err_detect_failed;

	ret = reg_write(gs, GS_ADI_REG_POWER_CTL, 0x0);  
	if (ret < 0) {
		printk(KERN_ERR "%s:reset chip failed\n", __FUNCTION__);
		/* fail? */
		goto err_detect_failed;
	}
	/* DTS2011071300230  liujinggang 20110810 end > */

    /* <DTS2011021804534 shenjinming 20110218 begin */
    #ifdef CONFIG_HUAWEI_HW_DEV_DCT
    /* detect current device successful, set the flag as present */
    set_hw_dev_flag(DEV_I2C_G_SENSOR);
    #endif
    /* DTS2011021804534 shenjinming 20110218 end> */   

	if (sensor_dev == NULL)
	{
		gs->input_dev = input_allocate_device();
		if (gs->input_dev == NULL) {
			ret = -ENOMEM;
			printk(KERN_ERR "%s: Failed to allocate input device\n",__FUNCTION__);
			goto err_input_dev_alloc_failed;
		}
		
		gs->input_dev->name = "sensors";
		sensor_dev = gs->input_dev;
		
	}else{
		gs->input_dev = sensor_dev;
	}
	
	gs->input_dev->id.vendor = GS_ADI346;
	
	set_bit(EV_ABS,gs->input_dev->evbit);
	set_bit(ABS_X, gs->input_dev->absbit);
	set_bit(ABS_Y, gs->input_dev->absbit);
	set_bit(ABS_Z, gs->input_dev->absbit);
	set_bit(EV_SYN,gs->input_dev->evbit);


	gs->input_dev->id.bustype = BUS_I2C;
	
	input_set_drvdata(gs->input_dev, gs);
	
	ret = input_register_device(gs->input_dev);
	if (ret) {
		printk(KERN_ERR "gs_probe: Unable to register %s input device\n", gs->input_dev->name);
		goto err_input_register_device_failed;
	}
	
	ret = misc_register(&gsensor_device);
	if (ret) {
		printk(KERN_ERR "gs_probe: gsensor_device register failed\n");
		goto err_misc_device_register_failed;
	}

#ifndef   GS_POLLING 
	if (client->irq) {
		ret = request_irq(client->irq, gs_irq_handler, 0, client->name, gs);
		
		if (ret == 0)
			gs->use_irq = 1;
		else
			dev_err(&client->dev, "request_irq failed\n");
	}
#endif 

	

	if (!gs->use_irq) {
		hrtimer_init(&gs->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		gs->timer.function = gs_timer_func;
	}
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	gs->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	gs->early_suspend.suspend = gs_early_suspend;
	gs->early_suspend.resume = gs_late_resume;
	register_early_suspend(&gs->early_suspend);
#endif

	gs_wq = create_singlethread_workqueue("gs_wq");
	if (!gs_wq)
		return -ENOMEM;
	
	this_gs_data =gs;

	/* < DTS2011011905410   liujinggang 20110119 begin */
	if(pdata && pdata->init_flag)
		*(pdata->init_flag) = 1;
	/* DTS2011011905410   liujinggang 20110119 end > */

	printk(KERN_INFO "gs_probe: Start adi346  in %s mode\n", gs->use_irq ? "interrupt" : "polling");

	return 0;
	
err_misc_device_register_failed:
	misc_deregister(&gsensor_device);

err_input_register_device_failed:
	input_free_device(gs->input_dev);

err_input_dev_alloc_failed:
err_detect_failed:
	kfree(gs);
err_alloc_data_failed:
#ifndef   GS_POLLING 
	gs_free_int();
#endif
/* < DTS2011043000257  liujinggang 20110503 begin */
/*turn down the power*/	
err_power_failed:
	if(pdata->gs_power != NULL){
		pdata->gs_power(IC_PM_OFF);
	}
err_check_functionality_failed:

/* DTS2011043000257  liujinggang 20110503 end > */
	printk(KERN_INFO "gs_probe: faile  adi346  in  mode\n");

	return ret;
}

static int gs_remove(struct i2c_client *client)
{
	struct gs_data *gs = i2c_get_clientdata(client);
	unregister_early_suspend(&gs->early_suspend);
	if (gs->use_irq)
		free_irq(client->irq, gs);
	else
		hrtimer_cancel(&gs->timer);
	misc_deregister(&gsensor_device);
	input_unregister_device(gs->input_dev);
	kfree(gs);
	return 0;
}

/* < DTS2011071300230  liujinggang 20110810 begin */
/*set register*/
static int gs_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct gs_data *gs = i2c_get_clientdata(client);

	if (gs->use_irq)
		disable_irq(client->irq);
	else
		hrtimer_cancel(&gs->timer);
	ret = cancel_work_sync(&gs->work);
	if (ret && gs->use_irq) 
		enable_irq(client->irq);
	
	reg_write(this_gs_data, GS_ADI_REG_POWER_CTL, 0); /* power down */
	return 0;
}

static int gs_resume(struct i2c_client *client)
{
	struct gs_data *gs = i2c_get_clientdata(client);
	
	reg_write(this_gs_data, GS_ADI_REG_OFSX, 0); /* Tap Threshold: 2G */
	reg_write(this_gs_data, GS_ADI_REG_OFSY, 0); /* Tap Threshold: 2G */
	reg_write(this_gs_data, GS_ADI_REG_OFSZ, 0); /* Tap Threshold: 2G */
	reg_write(this_gs_data,GS_ADI_REG_BW,0x0b);    /* Rate: 200Hz, IDD: 130uA */
	 reg_write(this_gs_data,GS_ADI_REG_DATA_FORMAT,0x0B);/* Data Format: 16g right justified  256=1g*/
	reg_write(this_gs_data, GS_ADI_REG_POWER_CTL, 0x08);
	
	if (!gs->use_irq)
		hrtimer_start(&gs->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	else
		enable_irq(client->irq);


	return 0;
}
/* DTS2011071300230  liujinggang 20110810 end > */

#ifdef CONFIG_HAS_EARLYSUSPEND
static void gs_early_suspend(struct early_suspend *h)
{
	struct gs_data *gs;
	gs = container_of(h, struct gs_data, early_suspend);
	gs_suspend(gs->client, PMSG_SUSPEND);
}

static void gs_late_resume(struct early_suspend *h)
{
	struct gs_data *gs;
	gs = container_of(h, struct gs_data, early_suspend);
	gs_resume(gs->client);
}
#endif

static const struct i2c_device_id gs_id[] = {
	{ ADI346_DRV_NAME, 0 },
	{ }
};

static struct i2c_driver gs_driver = {
	.probe		=gs_probe,
	.remove		= gs_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= gs_suspend,
	.resume		= gs_resume,
#endif
	.id_table	= gs_id,
	.driver = {
		.name	=ADI346_DRV_NAME,
	},
};

static int __devinit gs_adi346_init(void)
{
	GS_DEBUG("gs_adi346_init\n");
	return i2c_add_driver(&gs_driver);
}

static void __exit gs_adi346_exit(void)
{
	i2c_del_driver(&gs_driver);
	if (gs_wq)
		destroy_workqueue(gs_wq);
}

module_init(gs_adi346_init);
module_exit(gs_adi346_exit);

MODULE_DESCRIPTION("gs_adi346 Driver");
MODULE_LICENSE("GPL");
/* DTS2011010404642 wuzhihui 20110104 end > */
