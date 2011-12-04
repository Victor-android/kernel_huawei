/* drivers/input/gsdev.c
 *
 * 
 *created by zlp 2009-2-18 11:10
 * 
 *
 */
/* < DTS2011042703449  liujinggang 20110427 begin */
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
/*BK4D00263, add for misc devices, dingxifeng KF14049, 2009-5-20 begin */
#include <mach/vreg.h>
/* DTS2011042703449  liujinggang 20110427 end > */

#include <linux/miscdevice.h>
#include <asm/uaccess.h>
/*BK4D00263, add for misc devices, dingxifeng KF14049, 2009-5-20 end */

/*BK4D00074, add  <gs_st.h> file, dingxifeng KF14049, 2009-4-1 begin */

#include <linux/gs_st.h>
/* <BU5D01928 zhangxiangdang 20100316 begin */
#include "linux/hardware_self_adapt.h"
/* BU5D01928 zhangxiangdang 20100316 end> */ 

#define GS_POLLING   1
/*BK4D00074, add  <gs_st.h> file, dingxifeng KF14049, 2009-4-1 end */

//delete by mzh
//#define gs_swap(x, y) do { typeof(x) z = x; x = y; y = z; } while (0)

static struct workqueue_struct *gs_wq;

/* BK4D04662, G-sensor & Compass share input dev, zhouzuohua 00145359 2009.09.05  start */
/* <BU5D01928 zhangxiangdang 20100316 begin */
extern struct input_dev *sensor_dev ;
/* BU5D01928 zhangxiangdang 20100316 end> */ 
/* BK4D04662, G-sensor & Compass share input dev, zhouzuohua 00145359 2009.09.05  end   */

struct gs_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_irq;
	
/*BK4D00074, add  mlock, dingxifeng KF14049, 2009-4-1 begin */
	
	struct mutex  mlock;
	
/*BK4D00074, add  mlock, dingxifeng KF14049, 2009-4-1 end */
	
	struct hrtimer timer;
	struct work_struct  work;	
	uint32_t flags;
	int (*power)(int on);
	struct early_suspend early_suspend;
};

/*BK4D00263, add for misc devices, dingxifeng KF14049, 2009-5-20 begin */

static struct gs_data  *this_gs_data;
/*BK4D00263, add for misc devices, dingxifeng KF14049, 2009-5-20 end */

/*BK4D01075, add  delay  variable, dingxifeng KF14049, 2009-6-10  begin*/

static int accel_delay = GS_ST_TIMRER;     /*1s*/

/*BK4D01075, add  delay  variable, dingxifeng KF14049, 2009-6-10  end*/
/*BK4D01898, add  acc_flag for control G-sensor, dingxifeng KF14049, 2009-7-2  begin*/

static atomic_t a_flag;
/*BK4D01898, add  acc_flag for control G-sensor, dingxifeng KF14049, 2009-7-2  end*/

#ifdef CONFIG_HAS_EARLYSUSPEND
static void gs_early_suspend(struct early_suspend *h);
static void gs_late_resume(struct early_suspend *h);
#endif

/**************************************************************************************/
/*BK4D00074, add accelerometer code, dingxifeng KF14049, 2009-4-1 begin */

static inline int reg_read(struct gs_data *gs , int reg)
{
	int val;

	mutex_lock(&gs->mlock);

	val = i2c_smbus_read_byte_data(gs->client, reg);
	if (val < 0)
		printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");

	mutex_unlock(&gs->mlock);

	return val;
}
static inline int reg_write(struct gs_data *gs, int reg, uint8_t val)
{
	int ret;

	mutex_lock(&gs->mlock);
	ret = i2c_smbus_write_byte_data(gs->client, reg, val);
	if(ret < 0) {
		printk(KERN_ERR "i2c_smbus_write_byte_data failed\n");
	}
	mutex_unlock(&gs->mlock);

	return ret;
}
/*BK4D02883, delet  #ifndef   GS_POLLING  ,dingxifeng KF14049, 2009-7-20   */



/*BK4D00074, add accelerometer code, dingxifeng KF14049, 2009-4-1 end */

/**************************************************************************************/


/*BK4D00068, add accelerometer code, dingxifeng KF14049, 2009-3-31 begin */

/*BK4D00610,  modify  calculate of Acceleration value , dingxifeng KF14049, 2009-06-01 begin*/
#define MG_PER_SAMPLE   720                       /*HAL: 720=1g*/                       
#define FILTER_SAMPLE_NUMBER  256           /*256LSB =1g*/  
/*BK4D00610,  modify  calculate of Acceleration value , dingxifeng KF14049, 2009-06-01 end */


/*BK4D00235, add  interface for compass, dingxifeng KF14049, 2009-5-13 begin */
static int sensor_data[4];
/* <BU5D01928 zhangxiangdang 20100316 begin */
/* < DTS2011042703449  liujinggang 20110427 begin */
/*adjust device name */
static char st_device_id[] = "st_35de";
/* DTS2011042703449  liujinggang 20110427 end > */
/* BU5D01928 zhangxiangdang 20100316 end> */ 

int gs_st_data_to_compass(int accel_data [3])
{
	memset(accel_data, 0, 3 );
/*BK4D00610,  modify  calculate of Acceleration value , dingxifeng KF14049, 2009-06-01 begin */
	accel_data[0]=sensor_data[0]/2;
	accel_data[1]=sensor_data[1]/2;
	accel_data[2]=sensor_data[2]/2;
/*BK4D00610,  modify  calculate of Acceleration value , dingxifeng KF14049, 2009-06-01 end */
	return 0;

}
/*BK4D00263, add  interface for compass, dingxifeng KF14049, 2009-5-20 begin */

/**************************************************************************************/

static int gs_st_open(struct inode *inode, struct file *file)
{			
       reg_read(this_gs_data, GS_ST_REG_STATUS ); /* read status */
	
	reg_write(this_gs_data, GS_ST_REG_CTRL1, GS_ST_CTRL1_PD|
								          GS_ST_CTRL1_Zen|
		                                                  GS_ST_CTRL1_Yen|
		                                                  GS_ST_CTRL1_Xen); 
	
	reg_write(this_gs_data, GS_ST_REG_CTRL3, GS_INTMODE_DATA_READY); 
	reg_read(this_gs_data, GS_ST_REG_OUT_X ); /* read X */
	reg_read(this_gs_data, GS_ST_REG_OUT_Y ); /* read Y */
	reg_read(this_gs_data, GS_ST_REG_OUT_Z ); /* read Z*/

	if (this_gs_data->use_irq)
		enable_irq(this_gs_data->client->irq);
	else
		 hrtimer_start(&this_gs_data->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	
		
	return nonseekable_open(inode, file);
}

static int gs_st_release(struct inode *inode, struct file *file)
{
	
	 reg_write(this_gs_data, GS_ST_REG_CTRL1, 0x00); 
	
	if (this_gs_data->use_irq)
		disable_irq(this_gs_data->client->irq);
	else
		hrtimer_cancel(&this_gs_data->timer);
	
/*BK4D01075, add  delay  variable, dingxifeng KF14049, 2009-6-10  begin*/
	
	   accel_delay = GS_ST_TIMRER;	  
	

	
/*BK4D01075, add  delay  variable, dingxifeng KF14049, 2009-6-10  end*/
		return 0;
}

static int
gs_st_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
	   unsigned long arg)
{
	
	int i;
	void __user *argp = (void __user *)arg;
	int accel_buf[3];
	
/*BK4D01075, add  set delay interface for app , dingxifeng KF14049, 2009-6-10  begin*/
	short flag;

	switch (cmd) 
	{
/*BK4D01898, add  acc_flag for control G-sensor, dingxifeng KF14049, 2009-7-2  begin*/

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
			
/*BK4D01637, the driver cant not use 0ms , dingxifeng KF14049, 2009-6-22 begin*/
			if(flag)
				accel_delay = flag;
			else
				accel_delay = 10;   /*10ms*/
/*BK4D01637, the driver cant not use 0ms , dingxifeng KF14049, 2009-6-22  end*/
			
			break;
			
		case ECS_IOCTL_APP_GET_DELAY:
			flag = accel_delay;
			break;
			
/*BK4D01075, add  set delay interface for app , dingxifeng KF14049, 2009-6-10  end*/
		case ECS_IOCTL_READ_ACCEL_XYZ:
			for(i=0;i<3;i++)
				gs_st_data_to_compass(accel_buf);
			break;
		default:
			break;
	}
	switch (cmd) 
	{
/*BK4D01075, add  set delay interface for app , dingxifeng KF14049, 2009-6-10  begin*/
		case ECS_IOCTL_APP_GET_AFLAG:
			if (copy_to_user(argp, &flag, sizeof(flag)))
				return -EFAULT;
			
			break;
/*BK4D01898, add  acc_flag for control G-sensor, dingxifeng KF14049, 2009-7-2  end*/

		case ECS_IOCTL_APP_GET_DELAY:
			if (copy_to_user(argp, &flag, sizeof(flag)))
			return -EFAULT;
			
			break;
			
/*BK4D01075, add  set delay interface for app , dingxifeng KF14049, 2009-6-10  end*/
		case ECS_IOCTL_READ_ACCEL_XYZ:
			if (copy_to_user(argp, &accel_buf, sizeof(accel_buf)))
				return -EFAULT;
			break;
/* <BU5D01928 zhangxiangdang 20100316 begin */
		case ECS_IOCTL_READ_DEVICEID:
			if (copy_to_user(argp, st_device_id, sizeof(st_device_id)))
				return -EFAULT;
			break;
/* BU5D01928 zhangxiangdang 20100316 end> */ 
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

/*BK4D01637, delet #if 0 BK4D00074, BK4D00068, dingxifeng KF14049, 2009-6-22  */
/*BK4D00263, add  interface for compass, dingxifeng KF14049, 2009-5-20 end */

static void gs_work_func(struct work_struct *work)
{
	int status;	
	int x,y,z;
	struct gs_data *gs = container_of(work, struct gs_data, work);
/*BK4D01637, start timer in work_func , dingxifeng KF14049, 2009-6-22  begin*/
	int	sesc = accel_delay/1000;
	int nsesc = (accel_delay%1000)*1000000;
/*BK4D01637, start timer in work_func , dingxifeng KF14049, 2009-6-22  end*/
/*BK4D00068, add accelerometer code, dingxifeng KF14049, 2009-3-31 begin */

 /*BK4D00074,  optimize code, dingxifeng KF14049, 2009-4-1 begin */
       
	 status = reg_read(gs, GS_ST_REG_STATUS ); /* read status */
	
	if(status & (1<<3))
	{
		 u8 u8x = reg_read(gs,GS_ST_REG_OUT_X);
		
		 u8  u8y=  reg_read(gs,GS_ST_REG_OUT_Y);
		
		 u8 u8z = reg_read(gs,GS_ST_REG_OUT_Z);
		 
 /*BK4D00263, add  interface for compass, dingxifeng KF14049, 2009-5-20 begin */
		
/*BK4D00074,  optimize code, dingxifeng KF14049, 2009-4-1 end */

#if 0
/*BK4D000141,  reverse the value of X axis 180  degree,for switching screen landscape and portrait  , dingxifeng KF14049, 2009-4-24 begin */

		x = MG_PER_SAMPLE *(s8)u8y  ;		
/*BK4D000141,  reverse the value of X axis 180  degree,for switching screen landscape and portrait, dingxifeng KF14049, 2009-4-24 end */

		y = MG_PER_SAMPLE * (s8)u8x ;			
		z = MG_PER_SAMPLE * (s8)u8z;
#endif	
		
/*BK4D00610,  modify  calculate of Acceleration value , dingxifeng KF14049, 2009-06-01 begin */
		if(u8x&0x80)/*负值*/
		{
			x= u8x-256; 		/*负数按照补码计算 */  
		}
		else
		{
			x = u8x;
		}
					
		if(u8y&0x80)/*负值*/
		{
			 y=u8y-256; 		/*负数按照补码计算 */  	 
		}
		else
		{
			y = u8y;
		}
				
		if(u8z&0x80)/*负值*/
		{
			 z=u8z-256; 		/*负数按照补码计算 */   
		}
		else
		{
			  z = u8z;
		}

		/*<BU5D04028 yuxuesong 20100301 begin*/
		/* change the x,y,z for u8300 because orientation of accelerometer of u8300 is different.*/
		//if(machine_is_msm7x25_u8300()) 
		{
			int x1,y1,z1;
			x1 = y * (-1);
			y1 = x * (-1);
			z1 = z * (-1);
			
			x = x1;
			y = y1;
			z = z1;
		}
		/* BU5D04028 yuxuesong 20100301 end>*/

		memset(sensor_data, 0, 3 );
		sensor_data[0]= 46*(s16)x/10;
		sensor_data[1]= 46*(s16)y/10 ;	
		sensor_data[2]= 46*(s16)z/10;
		/*(Decimal value/ 256) * 4.6 g,For (0g ~+2.3g)*/	
		x = (MG_PER_SAMPLE*46*(s16)x)/FILTER_SAMPLE_NUMBER/10;           
		y = (MG_PER_SAMPLE*46*(s16)y)/FILTER_SAMPLE_NUMBER/10;
		z = (MG_PER_SAMPLE*46*(s16)z)/FILTER_SAMPLE_NUMBER/10;
		x *=(-1);		

		input_report_abs(gs->input_dev, ABS_X, x);			
		input_report_abs(gs->input_dev, ABS_Y, y);			
		input_report_abs(gs->input_dev, ABS_Z, z);
		input_sync(gs->input_dev);



		
/*BK4D00610,  modify  calculate of Acceleration value , dingxifeng KF14049, 2009-06-01 end */
/*BK4D01637, start timer in work_func , dingxifeng KF14049, 2009-6-22  begin*/
/*BK4D00263, add  interface for compass, dingxifeng KF14049, 2009-5-20 end */


	}
	if (gs->use_irq)
		enable_irq(gs->client->irq);
	else
		hrtimer_start(&gs->timer, ktime_set(sesc, nsesc), HRTIMER_MODE_REL);
/*BK4D00068, add accelerometer code, dingxifeng KF14049, 2009-3-31 end */	

	
}


static enum hrtimer_restart gs_timer_func(struct hrtimer *timer)
{
	struct gs_data *gs = container_of(timer, struct gs_data, timer);		
	queue_work(gs_wq, &gs->work);
	return HRTIMER_NORESTART;
}
/*BK4D01637, start timer in work_func , dingxifeng KF14049, 2009-6-22  end*/

#ifndef   GS_POLLING 	

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
/*BK4D00068, modify accelerometer int pin, dingxifeng KF14049, 2009-3-31 begin */
/*BK4D00074,  optimize code, dingxifeng KF14049, 2009-4-1 begin */

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
/*BK4D00074,  optimize code, dingxifeng KF14049, 2009-4-1 end */

/*BK4D00068, modify accelerometer int pin, dingxifeng KF14049, 2009-3-31 end */
#endif
static int gs_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{	
      int ret;
      struct gs_data *gs;
 	/* < DTS2011042703449  liujinggang 20110427 begin */
      struct gs_platform_data *pdata;
	struct vreg *vreg_gp4=NULL;
	int rc;

	vreg_gp4 = vreg_get(NULL, "gp4");
    /* <DTS2011012600839 liliang 20110215 begin */
    /* set gp4 voltage as 2700mV for all */
    rc = vreg_set_level(vreg_gp4,VREG_GP4_VOLTAGE_VALUE_2700);
    /* <DTS2011012600839 liliang 20110215 end >*/
	if (rc) {
		printk("%s: vreg_gp4  vreg_set_level failed \n", __func__);
		return rc;
	}
	
	rc = vreg_enable(vreg_gp4);
	if (rc) {
		printk("%s: vreg_gp4    vreg_enable failed \n", __func__);
		return rc;
	}
	mdelay(5);
    
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "gs_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	pdata = client->dev.platform_data;
	if (pdata){
		if(pdata->adapt_fn != NULL){
			ret = pdata->adapt_fn();
			if(ret > 0){
				client->addr = pdata->slave_addr;//actual address
				printk(KERN_INFO "%s:change i2c addr to actrual address = %d\n", __FUNCTION__, pdata->slave_addr);
				if(client->addr == 0){
					printk(KERN_ERR "%s: bad i2c address = %d\n", __FUNCTION__, client->addr);
					ret = -EFAULT;
					goto err_check_functionality_failed;
				}
			}
		}

		if(pdata->init_flag != NULL){
			if(*(pdata->init_flag)){
				printk(KERN_ERR "gs_st probe failed, because the othe gsensor has been probed.\n");
				ret = -ENODEV;
				goto err_check_functionality_failed;
			}
		}
	}
	/* DTS2011042703449  liujinggang 20110427 end > */
	
#ifndef   GS_POLLING 	
	ret = gs_config_int_pin();
	if(ret <0)
	{
		goto err_check_functionality_failed;
	}
#endif

	gs = kzalloc(sizeof(*gs), GFP_KERNEL);
	if (gs == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
/*BK4D00074, add  mlock, dingxifeng KF14049, 2009-4-1 begin */	
	mutex_init(&gs->mlock);
/*BK4D00074, add  mlock, dingxifeng KF14049, 2009-4-1 end */

	INIT_WORK(&gs->work, gs_work_func);
	gs->client = client;
	i2c_set_clientdata(client, gs);

/*BK4D00068, remove accelerometer code , dingxifeng KF14049, 2009-3-31 begin */	
#if 0
	
    ret = i2c_smbus_write_byte_data(gs->client, 0x20, 0x47); /* device command = ctrl_reg1 */
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_write_byte_data failed\n");
		/* fail? */
		goto err_detect_failed;
	}
#endif	
/*BK4D00068, remove accelerometer code , dingxifeng KF14049, 2009-3-31 end */	

/*BK4D00074,  optimize code, dingxifeng KF14049, 2009-4-1 begin */

	ret = reg_write(gs, GS_ST_REG_CTRL2, 0x00); /* device command = ctrl_reg2 */
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_write_byte_data failed\n");
		/* fail? */
		goto err_detect_failed;
	}
/*BK4D00074,  optimize code, dingxifeng KF14049, 2009-4-1 end */

/*BK4D00068, remove accelerometer code , dingxifeng KF14049, 2009-3-31 begin */	
#if 0

	ret = i2c_smbus_write_byte_data(gs->client, 0x22, 0x04); /* device command = ctrl_reg3 */
	if (ret < 0) {
		printk(KERN_ERR "i2c_smbus_write_byte_data failed\n");
		/* fail? */
		goto err_detect_failed;
	}
#endif

	/* <BU5D01928 zhangxiangdang 20100316 begin */
	if (sensor_dev == NULL)
	{
		gs->input_dev = input_allocate_device();
		if (gs->input_dev == NULL) {
			ret = -ENOMEM;
			printk(KERN_ERR "gs_probe: Failed to allocate input device\n");
			goto err_input_dev_alloc_failed;
		}
		
		gs->input_dev->name = "sensors";
		sensor_dev = gs->input_dev;
		
	}else{

		gs->input_dev = sensor_dev;
	}
	
	gs->input_dev->id.vendor = GS_ST35DE;//for  akm8973 compass detect.
	/* BU5D01928 zhangxiangdang 20100316 end> */

	set_bit(EV_ABS,gs->input_dev->evbit);
	
	set_bit(ABS_X, gs->input_dev->absbit);
	set_bit(ABS_Y, gs->input_dev->absbit);
	set_bit(ABS_Z, gs->input_dev->absbit);
	
	set_bit(EV_SYN,gs->input_dev->evbit);


	gs->input_dev->id.bustype = BUS_I2C;
/*BK4D00263, delet input  interface for misc, dingxifeng KF14049, 2009-5-20 begin */
	//gs->input_dev->open = gs_st_input_open;
	//gs->input_dev->close = gs_st_input_close;
/*BK4D00263, delet input interface for misc, dingxifeng KF14049, 2009-5-20 end */
	
	input_set_drvdata(gs->input_dev, gs);
	
/*BK4D00068, add accelerometer code , dingxifeng KF14049, 2009-3-31 end */
	
	ret = input_register_device(gs->input_dev);
	if (ret) {
		printk(KERN_ERR "gs_probe: Unable to register %s input device\n", gs->input_dev->name);
		goto err_input_register_device_failed;
	}
	
/*BK4D00068, remove accelerometer code, dingxifeng KF14049, 2009-3-31 begin */
/*BK4D00074,  optimize code, dingxifeng KF14049, 2009-4-1 begin */
	ret = misc_register(&gsensor_device);
	if (ret) {
/*BK4D02883, modify printk mesg, dingxifeng KF14049, 2009-7-20 begin */
		printk(KERN_ERR "gs_probe: gsensor_device register failed\n");
/*BK4D02883, modify printk mesg, dingxifeng KF14049, 2009-7-20 end */

		goto err_misc_device_register_failed;
	}
/*BK4D00263, add for misc devices, dingxifeng KF14049, 2009-5-20 end */

#ifndef   GS_POLLING 
	if (client->irq) {
		ret = request_irq(client->irq, gs_irq_handler, 0, client->name, gs);
		
		if (ret == 0)
			gs->use_irq = 1;
		else
			dev_err(&client->dev, "request_irq failed\n");
	}
#endif 
/*BK4D00074,  optimize code, dingxifeng KF14049, 2009-4-1 end */

/*BK4D00068, remove accelerometer code, dingxifeng KF14049, 2009-3-31 end */

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
/*BK4D00235, modify creat workqueue position, dingxifeng KF14049, 2009-5-13 begin */

      gs_wq = create_singlethread_workqueue("gs_wq");
	if (!gs_wq)
		return -ENOMEM;
/*BK4D00263, add for misc devices, dingxifeng KF14049, 2009-5-20 begin */
	
	  this_gs_data =gs;
/*BK4D00263, add for misc devices, dingxifeng KF14049, 2009-5-20 end */

/*BK4D00235, modify creat workqueue position, dingxifeng KF14049, 2009-5-13 end */
/*BK4D02883, modify printk mesg, dingxifeng KF14049, 2009-7-20 begin */

	printk(KERN_INFO "gs_probe: Start LIS35DE  in %s mode\n", gs->use_irq ? "interrupt" : "polling");
/*BK4D02883, modify printk mesg, dingxifeng KF14049, 2009-7-20 end */
 	/* < DTS2011042703449  liujinggang 20110427 begin */
	if(pdata && pdata->init_flag)
		*(pdata->init_flag) = 1;
	/* DTS2011042703449  liujinggang 20110427 end > */
	return 0;
	
/*BK4D00263, add for misc devices, dingxifeng KF14049, 2009-5-20 begin */
err_misc_device_register_failed:
		misc_deregister(&gsensor_device);
		
/*BK4D00263, add for misc devices, dingxifeng KF14049, 2009-5-20 end */

err_input_register_device_failed:
	input_free_device(gs->input_dev);

err_input_dev_alloc_failed:
/* <BU5D01928 zhangxiangdang 20100316 begin */
err_detect_failed:
	kfree(gs);


err_alloc_data_failed:
#ifndef   GS_POLLING 
	gs_free_int();
#endif
/* BU5D01928 zhangxiangdang 20100316 end> */
err_check_functionality_failed:
 	/* < DTS2011042703449  liujinggang 20110427 begin */
	if (vreg_gp4!=NULL)
	{
		rc = vreg_disable(vreg_gp4);
		if (rc) {
			printk("%s: vreg_gp4    vreg_enable failed \n", __func__);
			return rc;
		}
	}
	/* DTS2011042703449  liujinggang 20110427 end > */
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
/*BK4D00263, add for misc devices, dingxifeng KF14049, 2009-5-20 begin */
	
	misc_deregister(&gsensor_device);
	
	input_unregister_device(gs->input_dev);

	
/*BK4D00263, add for misc devices, dingxifeng KF14049, 2009-5-20 end */
	kfree(gs);
	return 0;
}

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
/*BK4D00074,  optimize code, dingxifeng KF14049, 2009-4-1 begin */

	reg_write(gs, GS_ST_REG_CTRL3, 0); /* disable interrupt */
	
	reg_write(gs, GS_ST_REG_CTRL1, 0x00); /* deep sleep */
/*BK4D00074,  optimize code, dingxifeng KF14049, 2009-4-1 end */

	if (gs->power) {
		ret = gs->power(0);
		if (ret < 0)
			printk(KERN_ERR "gs_resume power off failed\n");
	}
	return 0;
}

static int gs_resume(struct i2c_client *client)
{
	struct gs_data *gs = i2c_get_clientdata(client);
	
/*BK4D000141,  modify resume interface, dingxifeng KF14049, 2009-4-24 begin */
	reg_write(gs, GS_ST_REG_CTRL1, GS_ST_CTRL1_PD|
						GS_ST_CTRL1_Zen|
						GS_ST_CTRL1_Yen|
						GS_ST_CTRL1_Xen); /* enable abs int */
/*BK4D000141,  modify value of X axis for application, dingxifeng KF14049, 2009-4-24 end*/
/*BK4D00074,  optimize code, dingxifeng KF14049, 2009-4-1 begin */
 /*BK4D02883, modify active mode for high  ,dingxifeng KF14049, 2009-7-20  begin */
	
       reg_write(gs, GS_ST_REG_CTRL3, GS_INTMODE_DATA_READY);/*active mode*/
    
/*BK4D02883, modify active mode for high  ,dingxifeng KF14049, 2009-7-20  end*/
	if (!gs->use_irq)
		hrtimer_start(&gs->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	else
/*BK4D000141,  modify value of X axis for application, dingxifeng KF14049, 2009-4-24 begin */
		enable_irq(client->irq);
	
/*BK4D000141,  modify value of X axis for application, dingxifeng KF14049, 2009-4-24 end */
	
/*BK4D00074,  optimize code, dingxifeng KF14049, 2009-4-1 end */

	return 0;
}

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
	{ /*GS_I2C_NAME*/"gs_st", 0 },
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
		.name	="gs_st",
	},
};

static int __devinit gs_init(void)
{
	return i2c_add_driver(&gs_driver);
}

static void __exit gs_exit(void)
{
	i2c_del_driver(&gs_driver);
	if (gs_wq)
		destroy_workqueue(gs_wq);
}

module_init(gs_init);
module_exit(gs_exit);

MODULE_DESCRIPTION("accessor  Driver");
MODULE_LICENSE("GPL");
