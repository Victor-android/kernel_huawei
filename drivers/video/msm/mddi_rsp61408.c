/*< DTS2011090102706 jiaoshuangwei 20110901 begin */
/* drivers\video\msm\mddi_rsp61408.c
 * rsp61408 LCD driver for 7x27 platform
 *
 * Copyright (C) 2010 HUAWEI Technology Co., ltd.
 * 
 * Date: 2011/09/01
 * By jiaoshuangwei
 * 
 */

#include "msm_fb.h"
#include "mddihost.h"
#include "mddihosti.h"
#include <linux/mfd/pmic8058.h>
#include <mach/gpio.h>
#include <mach/vreg.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/hardware_self_adapt.h>
#include <linux/pwm.h>
#include <mach/pmic.h>
#include "hw_backlight.h"
#include "hw_mddi_lcd.h"
#include "lcd_hw_debug.h"
/* <DTS2011102904584 qitongliang 20111109 begin */
#define PM_GPIO_24 24
#define PM_GPIO_HIGH_VALUE 1 
/* DTS2011102904584 qitongliang 20111109 end> */

struct sequence* rsp61408_wvga_init_table = NULL;
static lcd_panel_type lcd_panel_wvga = LCD_NONE;
/*<  DTS2011091905632 jiaoshuangwei 20110924 begin */
/*delete the initialize sequence */

/* <DTS2011102904584 qitongliang 20111109 begin */
static struct sequence rsp61408_wvga_write_cabc_brightness_table[]= 
{
	{0x000B9,TYPE_COMMAND,0},//B9H
	{0x00000,TYPE_PARAMETER,0},
	{0x00000,TYPE_PARAMETER,0},
	{MDDI_MULTI_WRITE_END,TYPE_COMMAND,0}, //the end flag,it don't sent to driver IC
};
/* DTS2011102904584 qitongliang 20111109 end> */

static const struct sequence rsp61408_wvga_standby_exit_table[]= 
{
	/*set the delay time 100ms*/
	
	{0x0011,TYPE_COMMAND,0},
	{0x0000,TYPE_PARAMETER,0},
/*< DTS2011102903310 qitongliang 20111029 begin */
	{0x00029,TYPE_COMMAND,150}, //29h
/* DTS2011102903310 qitongliang 20111029 end >*/	
	{0x00000,TYPE_PARAMETER,0},
	{0x0003A,TYPE_COMMAND,100},
	{0x00077,TYPE_PARAMETER,0},//11h
/*< DTS2011101302113 qitongliang 20111013 begin */
	/*open Vsync singal,when lcd sleep out*/
	{0x00035,TYPE_COMMAND,0},
	{0x00000,TYPE_PARAMETER,0},
	{MDDI_MULTI_WRITE_END,TYPE_COMMAND,0}, //the end flag,it don't sent to driver IC
/* DTS2011101302113  qitongliang 20111013 end >*/
};
/*  DTS2011091905632 jiaoshuangwei 20110924 end >*/ 
static const struct sequence rsp61408_wvga_standby_enter_table[]= 
{
	/*set the delay time 100ms*/
/*< DTS2011101302113 qitongliang 20111013 begin */
	/*close Vsync singal,when lcd sleep in*/
	{0x00034,TYPE_COMMAND,0},
	{0x00000,TYPE_PARAMETER,0},
/* DTS2011101302113  qitongliang 20111013 end >*/
	{0x00028,TYPE_COMMAND,0}, //29h
	{0x00000,TYPE_PARAMETER,0},
/*< DTS2011102903310 qitongliang 20111029 begin */
	{0x0010,TYPE_COMMAND,150},
/* DTS2011102903310 qitongliang 20111029 end >*/
	{0x0000,TYPE_PARAMETER,0},
	{MDDI_MULTI_WRITE_END,TYPE_COMMAND,100}, //the end flag,it don't sent to driver IC
};

/*< DTS2011093001847 qitongliang 20111110 begin */
/* gamma 2.2 */
static const struct sequence rsp61408_wvga_dynamic_gamma22_table[] = 
{          
    {0x000B0,TYPE_COMMAND,0},//B0
	{0x00004,TYPE_PARAMETER,0},
	
	{0x000B3,TYPE_COMMAND,0},//B3
	{0x00002,TYPE_PARAMETER,0},
	{0x00000,TYPE_PARAMETER,0},
    /*
    {0x000B6,TYPE_COMMAND,0},//B6 MIPI Control
    {0x00052,TYPE_PARAMETER,0},
    {0x00083,TYPE_PARAMETER,0},
    */
 	{0x000B7,TYPE_COMMAND,0},//B7 MDDI Control
	{0x00000,TYPE_PARAMETER,0},
	{0x00000,TYPE_PARAMETER,0},
	{0x00011,TYPE_PARAMETER,0},
	{0x00025,TYPE_PARAMETER,0},
	
	{0x000C1,TYPE_COMMAND,0},//C1 panel driving setting2 
	{0x00042,TYPE_PARAMETER,0},
	{0x00031,TYPE_PARAMETER,0},
	{0x00004,TYPE_PARAMETER,0},
	{0x00026,TYPE_PARAMETER,0},
	{0x00026,TYPE_PARAMETER,0},
	{0x00032,TYPE_PARAMETER,0},
	{0x00012,TYPE_PARAMETER,0},
	{0x00028,TYPE_PARAMETER,0},
	{0x0000E,TYPE_PARAMETER,0},
	{0x00014,TYPE_PARAMETER,0},
	{0x000A5,TYPE_PARAMETER,0},
	{0x0000F,TYPE_PARAMETER,0},
	{0x00058,TYPE_PARAMETER,0},
	{0x00021,TYPE_PARAMETER,0},
	{0x00001,TYPE_PARAMETER,0},
	
	{0x000C2,TYPE_COMMAND,0},//C2 display v-timing setting
	{0x00008,TYPE_PARAMETER,0},
	{0x00006,TYPE_PARAMETER,0},
	{0x00006,TYPE_PARAMETER,0},
	{0x00001,TYPE_PARAMETER,0},
	{0x00003,TYPE_PARAMETER,0},
	{0x00000,TYPE_PARAMETER,0},
	
	{0x000C8,TYPE_COMMAND,0},//C8  gamma 2.2
	{0x00002,TYPE_PARAMETER,0},//v255
	{0x00017,TYPE_PARAMETER,0},//v251
	{0x0001D,TYPE_PARAMETER,0},//v247
	{0x00029,TYPE_PARAMETER,0},//v240
	{0x00036,TYPE_PARAMETER,0},//v224 38
	{0x00050,TYPE_PARAMETER,0},//v176 52
	{0x00033,TYPE_PARAMETER,0},//v79
	{0x00021,TYPE_PARAMETER,0},//v31
	{0x00016,TYPE_PARAMETER,0},//v15
	{0x00011,TYPE_PARAMETER,0},//v8
	{0x00008,TYPE_PARAMETER,0},//v4
	{0x00002,TYPE_PARAMETER,0},//v0
	{0x00002,TYPE_PARAMETER,0},//v255
	{0x00017,TYPE_PARAMETER,0},//v251
	{0x0001D,TYPE_PARAMETER,0},//v247
	{0x00029,TYPE_PARAMETER,0},//v240
	{0x00036,TYPE_PARAMETER,0},//v224 38 37
	{0x00050,TYPE_PARAMETER,0},//v176 52
	{0x00033,TYPE_PARAMETER,0},//v79
	{0x00021,TYPE_PARAMETER,0},//v31
	{0x00016,TYPE_PARAMETER,0},//v15
	{0x00011,TYPE_PARAMETER,0},//v8
	{0x00008,TYPE_PARAMETER,0},//v4
	{0x00002,TYPE_PARAMETER,0},//v0
	
	{0x000C9,TYPE_COMMAND,0},//C9
	{0x00002,TYPE_PARAMETER,0},//v255
	{0x00017,TYPE_PARAMETER,0},//v251
	{0x0001D,TYPE_PARAMETER,0},//v247
	{0x00029,TYPE_PARAMETER,0},//v240
	{0x00036,TYPE_PARAMETER,0},//v224 38
	{0x00050,TYPE_PARAMETER,0},//v176 52
	{0x00033,TYPE_PARAMETER,0},//v79
	{0x00021,TYPE_PARAMETER,0},//v31
	{0x00016,TYPE_PARAMETER,0},//v15
	{0x00011,TYPE_PARAMETER,0},//v8
	{0x00008,TYPE_PARAMETER,0},//v4
	{0x00002,TYPE_PARAMETER,0},//v0
	{0x00002,TYPE_PARAMETER,0},//v255
	{0x00017,TYPE_PARAMETER,0},//v251
	{0x0001D,TYPE_PARAMETER,0},//v247
	{0x00029,TYPE_PARAMETER,0},//v240
	{0x00036,TYPE_PARAMETER,0},//v224 38 37
	{0x00050,TYPE_PARAMETER,0},//v176 52
	{0x00033,TYPE_PARAMETER,0},//v79
	{0x00021,TYPE_PARAMETER,0},//v31
	{0x00016,TYPE_PARAMETER,0},//v15
	{0x00011,TYPE_PARAMETER,0},//v8
	{0x00008,TYPE_PARAMETER,0},//v4
	{0x00002,TYPE_PARAMETER,0},//v0
	
	{0x000CA,TYPE_COMMAND,0},//CA
	{0x00002,TYPE_PARAMETER,0},//v255
	{0x00017,TYPE_PARAMETER,0},//v251
	{0x0001D,TYPE_PARAMETER,0},//v247
	{0x00029,TYPE_PARAMETER,0},//v240
	{0x00036,TYPE_PARAMETER,0},//v224 38
	{0x00050,TYPE_PARAMETER,0},//v176 52
	{0x00033,TYPE_PARAMETER,0},//v79
	{0x00021,TYPE_PARAMETER,0},//v31
	{0x00016,TYPE_PARAMETER,0},//v15
	{0x00011,TYPE_PARAMETER,0},//v8
	{0x00008,TYPE_PARAMETER,0},//v4
	{0x00002,TYPE_PARAMETER,0},//v0
	{0x00002,TYPE_PARAMETER,0},//v255
	{0x00017,TYPE_PARAMETER,0},//v251
	{0x0001D,TYPE_PARAMETER,0},//v247
	{0x00029,TYPE_PARAMETER,0},//v240
	{0x00036,TYPE_PARAMETER,0},//v224 38 37
	{0x00050,TYPE_PARAMETER,0},//v176 52
	{0x00033,TYPE_PARAMETER,0},//v79
	{0x00021,TYPE_PARAMETER,0},//v31
	{0x00016,TYPE_PARAMETER,0},//v15
	{0x00011,TYPE_PARAMETER,0},//v8
	{0x00008,TYPE_PARAMETER,0},//v4
	{0x00002,TYPE_PARAMETER,0},//v0
	{MDDI_MULTI_WRITE_END,TYPE_COMMAND,0}, //the end flag,it don't sent to driver IC
};
/* gamma1.9 */
static const struct sequence rsp61408_wvga_dynamic_gamma19_table[] = {};
/* gamma2.5 */
static const struct sequence rsp61408_wvga_dynamic_gamma25_table[] = 
{
/*there is 2.5 GAMA initialization sequence */
	{0x000B0,TYPE_COMMAND,0},//B0
	{0x00004,TYPE_PARAMETER,0},
	
	{0x000B3,TYPE_COMMAND,0},//B3
	{0x00002,TYPE_PARAMETER,0},
	{0x00000,TYPE_PARAMETER,0},
    /*
    {0x000B6,TYPE_COMMAND,0},//B6 MIPI Control
    {0x00052,TYPE_PARAMETER,0},
    {0x00083,TYPE_PARAMETER,0},
    */
	{0x000B7,TYPE_COMMAND,0},//B7 MDDI Control
	{0x00000,TYPE_PARAMETER,0},
	{0x00000,TYPE_PARAMETER,0},
	{0x00011,TYPE_PARAMETER,0},
	{0x00025,TYPE_PARAMETER,0},
	
	{0x000C1,TYPE_COMMAND,0},//C1 panel driving setting2 
	{0x00042,TYPE_PARAMETER,0},
	{0x00031,TYPE_PARAMETER,0},
	{0x00004,TYPE_PARAMETER,0},
	{0x00026,TYPE_PARAMETER,0},
	{0x00026,TYPE_PARAMETER,0},
	{0x00032,TYPE_PARAMETER,0},
	{0x00012,TYPE_PARAMETER,0},
	{0x00028,TYPE_PARAMETER,0},
	{0x0000E,TYPE_PARAMETER,0},
	{0x00014,TYPE_PARAMETER,0},
	{0x000A5,TYPE_PARAMETER,0},
	{0x0000F,TYPE_PARAMETER,0},
	{0x00058,TYPE_PARAMETER,0},
	{0x00021,TYPE_PARAMETER,0},
	{0x00001,TYPE_PARAMETER,0},
	
	{0x000C2,TYPE_COMMAND,0},//C2 display v-timing setting
	{0x00008,TYPE_PARAMETER,0},
	{0x00006,TYPE_PARAMETER,0},
	{0x00006,TYPE_PARAMETER,0},
	{0x00001,TYPE_PARAMETER,0},
	{0x00003,TYPE_PARAMETER,0},
	{0x00000,TYPE_PARAMETER,0},
	
	{0x000C8,TYPE_COMMAND,0},//C8  gamma 2.5
	{0x00002,TYPE_PARAMETER,0},//v255
	{0x00017,TYPE_PARAMETER,0},//v251
	{0x0001D,TYPE_PARAMETER,0},//v247
	{0x00029,TYPE_PARAMETER,0},//v240
	{0x00039,TYPE_PARAMETER,0},//v224 38
	{0x00054,TYPE_PARAMETER,0},//v176 52
	{0x00030,TYPE_PARAMETER,0},//v79
	{0x00020,TYPE_PARAMETER,0},//v31
	{0x00017,TYPE_PARAMETER,0},//v15
	{0x00013,TYPE_PARAMETER,0},//v8
	{0x00008,TYPE_PARAMETER,0},//v4
	{0x00002,TYPE_PARAMETER,0},//v0
	{0x00002,TYPE_PARAMETER,0},//v255
	{0x00017,TYPE_PARAMETER,0},//v251
	{0x0001D,TYPE_PARAMETER,0},//v247
	{0x00029,TYPE_PARAMETER,0},//v240
	{0x00039,TYPE_PARAMETER,0},//v224 38
	{0x00054,TYPE_PARAMETER,0},//v176 52
	{0x00030,TYPE_PARAMETER,0},//v79
	{0x00020,TYPE_PARAMETER,0},//v31
	{0x00017,TYPE_PARAMETER,0},//v15
	{0x00013,TYPE_PARAMETER,0},//v8
	{0x00008,TYPE_PARAMETER,0},//v4
	{0x00002,TYPE_PARAMETER,0},//v0
	
	{0x000C9,TYPE_COMMAND,0},//C9
	{0x00002,TYPE_PARAMETER,0},//v255
	{0x00017,TYPE_PARAMETER,0},//v251
	{0x0001D,TYPE_PARAMETER,0},//v247
	{0x00029,TYPE_PARAMETER,0},//v240
	{0x00039,TYPE_PARAMETER,0},//v224 38
	{0x00054,TYPE_PARAMETER,0},//v176 52
	{0x00030,TYPE_PARAMETER,0},//v79
	{0x00020,TYPE_PARAMETER,0},//v31
	{0x00017,TYPE_PARAMETER,0},//v15
	{0x00013,TYPE_PARAMETER,0},//v8
	{0x00008,TYPE_PARAMETER,0},//v4
	{0x00002,TYPE_PARAMETER,0},//v0
	{0x00002,TYPE_PARAMETER,0},//v255
	{0x00017,TYPE_PARAMETER,0},//v251
	{0x0001D,TYPE_PARAMETER,0},//v247
	{0x00029,TYPE_PARAMETER,0},//v240
	{0x00039,TYPE_PARAMETER,0},//v224 38
	{0x00054,TYPE_PARAMETER,0},//v176 52
	{0x00030,TYPE_PARAMETER,0},//v79
	{0x00020,TYPE_PARAMETER,0},//v31
	{0x00017,TYPE_PARAMETER,0},//v15
	{0x00013,TYPE_PARAMETER,0},//v8
	{0x00008,TYPE_PARAMETER,0},//v4
	{0x00002,TYPE_PARAMETER,0},//v0
	
	{0x000CA,TYPE_COMMAND,0},//CA
	{0x00002,TYPE_PARAMETER,0},//v255
	{0x00017,TYPE_PARAMETER,0},//v251
	{0x0001D,TYPE_PARAMETER,0},//v247
	{0x00029,TYPE_PARAMETER,0},//v240
	{0x00039,TYPE_PARAMETER,0},//v224 38
	{0x00054,TYPE_PARAMETER,0},//v176 52
	{0x00030,TYPE_PARAMETER,0},//v79
	{0x00020,TYPE_PARAMETER,0},//v31
	{0x00017,TYPE_PARAMETER,0},//v15
	{0x00013,TYPE_PARAMETER,0},//v8
	{0x00008,TYPE_PARAMETER,0},//v4
	{0x00002,TYPE_PARAMETER,0},//v0
	{0x00002,TYPE_PARAMETER,0},//v255
	{0x00017,TYPE_PARAMETER,0},//v251
	{0x0001D,TYPE_PARAMETER,0},//v247
	{0x00029,TYPE_PARAMETER,0},//v240
	{0x00039,TYPE_PARAMETER,0},//v224 38
	{0x00054,TYPE_PARAMETER,0},//v176 52
	{0x00030,TYPE_PARAMETER,0},//v79
	{0x00020,TYPE_PARAMETER,0},//v31
	{0x00017,TYPE_PARAMETER,0},//v15
	{0x00013,TYPE_PARAMETER,0},//v8
	{0x00008,TYPE_PARAMETER,0},//v4
	{0x00002,TYPE_PARAMETER,0},//v0
	{MDDI_MULTI_WRITE_END,TYPE_COMMAND,0}, //the end flag,it don't sent to driver IC
};
/* resolve the tear screen and display inversion problem for buddy*/
/* gamma 2.2 */
static const struct sequence rsp61408_tear_dynamic_gamma22_U8730_table[] = 
{
	{0x000C1,TYPE_COMMAND,0}, 
	{0x00043,TYPE_PARAMETER,0}, 
	{0x00031,TYPE_PARAMETER,0}, 
	{0x00000,TYPE_PARAMETER,0}, 
	{0x00026,TYPE_PARAMETER,0}, 
	{0x00026,TYPE_PARAMETER,0},
	{0x00032,TYPE_PARAMETER,0}, 
	{0x00012,TYPE_PARAMETER,0}, 
	{0x00028,TYPE_PARAMETER,0},
	{0x0000E,TYPE_PARAMETER,0}, 
	{0x00014,TYPE_PARAMETER,0},
	{0x000A5,TYPE_PARAMETER,0}, 
	{0x0000F,TYPE_PARAMETER,0}, 
	{0x00058,TYPE_PARAMETER,0}, 
	{0x00021,TYPE_PARAMETER,0}, 
	{0x00001,TYPE_PARAMETER,0},
	{MDDI_MULTI_WRITE_END,TYPE_COMMAND,0}, //the end flag,it don't sent to driver IC
};
/* resolve the tear screen and display inversion problem for buddy*/
/* gamma2.5 */
static const struct sequence rsp61408_tear_dynamic_gamma25_U8730_table[] = 
{
	{0x000C1,TYPE_COMMAND,0}, 
	{0x00043,TYPE_PARAMETER,0}, 
	{0x00031,TYPE_PARAMETER,0}, 
	{0x00000,TYPE_PARAMETER,0}, 
	{0x00026,TYPE_PARAMETER,0}, 
	{0x00026,TYPE_PARAMETER,0},
	{0x00032,TYPE_PARAMETER,0}, 
	{0x00012,TYPE_PARAMETER,0}, 
	{0x00028,TYPE_PARAMETER,0},
	{0x0000E,TYPE_PARAMETER,0}, 
	{0x00014,TYPE_PARAMETER,0},
	{0x000A5,TYPE_PARAMETER,0}, 
	{0x0000F,TYPE_PARAMETER,0}, 
	{0x00058,TYPE_PARAMETER,0}, 
	{0x00021,TYPE_PARAMETER,0}, 
	{0x00001,TYPE_PARAMETER,0},
	{MDDI_MULTI_WRITE_END,TYPE_COMMAND,0}, //the end flag,it don't sent to driver IC
};

/* add the function  to set different gama by different mode */
int rsp61408_set_dynamic_gamma(enum danymic_gamma_mode  gamma_mode)
{
    int ret = 0;
	
    if (LOW_LIGHT == gamma_mode)
    {
        printk(KERN_ERR "the dynamic_gamma_setting is wrong\n");
    }

    switch(gamma_mode)
    {
        case GAMMA25:
            ret = process_lcd_table((struct sequence*)&rsp61408_wvga_dynamic_gamma25_table,
                        ARRAY_SIZE(rsp61408_wvga_dynamic_gamma25_table), lcd_panel_wvga);
			
			/* resolve the tear screen and display inversion problem for buddy*/
			if (machine_is_msm8255_u8730())
			{
                ret = process_lcd_table((struct sequence*)&rsp61408_tear_dynamic_gamma25_U8730_table,
                        ARRAY_SIZE(rsp61408_tear_dynamic_gamma25_U8730_table), lcd_panel_wvga);

			}
			
            break ;
        case GAMMA22:
			 ret = process_lcd_table((struct sequence*)&rsp61408_wvga_dynamic_gamma22_table,
                        ARRAY_SIZE(rsp61408_wvga_dynamic_gamma22_table), lcd_panel_wvga);
      
			/* resolve the tear screen and display inversion problem for buddy*/
			if (machine_is_msm8255_u8730())
			{
                ret = process_lcd_table((struct sequence*)&rsp61408_tear_dynamic_gamma22_U8730_table,
                        ARRAY_SIZE(rsp61408_tear_dynamic_gamma22_U8730_table), lcd_panel_wvga);
			}
			
            break;
        case HIGH_LIGHT:
            ret = process_lcd_table((struct sequence*)&rsp61408_wvga_dynamic_gamma19_table,
                        ARRAY_SIZE(rsp61408_wvga_dynamic_gamma19_table), lcd_panel_wvga);
            break;
        default:
            ret= -1;
            break;
    }
	MDDI_LCD_DEBUG("%s: change gamma mode to %d\n",__func__,gamma_mode);
    return ret;
}
/* DTS2011093001847 qitongliang 20111110 end >*/

static int rsp61408_lcd_on(struct platform_device *pdev)
{
	boolean para_debug_flag = FALSE;
    uint32 para_num = 0;
	int ret = 0;
/*<  DTS2011091905632 jiaoshuangwei 20110924 begin */
	/*delete the lcd reset*/
/*  DTS2011091905632 jiaoshuangwei 20110924 end >*/ 
 /* open debug file and read the para */
	switch(lcd_panel_wvga)
	{
		case LCD_RSP61408_CHIMEI_WVGA:
/*< DTS2011100803621  jiaoshuangwei 20111008 begin */
		case LCD_RSP61408_BYD_WVGA:
/* DTS2011100803621 jiaoshuangwei 20111008 end >*/
			para_debug_flag = lcd_debug_malloc_get_para( "rsp61408_wvga_init_table", 
	    		(void**)&rsp61408_wvga_init_table,&para_num);
			break;
		default:
			break;
	}
	/* If exist the init file ,then init lcd with it for debug */
    if( (TRUE == para_debug_flag)&&(NULL != rsp61408_wvga_init_table))
    {
		ret = process_lcd_table(rsp61408_wvga_init_table, para_num, lcd_panel_wvga);
    }
/*<  DTS2011091905632 jiaoshuangwei 20110924 begin */
    else
    {
		/* Exit Standby Mode */
		ret = process_lcd_table((struct sequence*)&rsp61408_wvga_standby_exit_table, 
			ARRAY_SIZE(rsp61408_wvga_standby_exit_table), lcd_panel_wvga);
    }
/*  DTS2011091905632 jiaoshuangwei 20110924 end >*/
       
	/* Must malloc before,then you can call free */
	if((TRUE == para_debug_flag)&&(NULL != rsp61408_wvga_init_table))
	{
		lcd_debug_free_para((void *)rsp61408_wvga_init_table);
	}
	
    MDDI_LCD_DEBUG("%s: rsp61408_lcd exit sleep mode ,on_ret=%d\n",__func__,ret);
	
	return ret;
}

static int rsp61408_lcd_off(struct platform_device *pdev)
{
	int ret = 0;
	ret = process_lcd_table((struct sequence*)&rsp61408_wvga_standby_enter_table, 
    	      		ARRAY_SIZE(rsp61408_wvga_standby_enter_table), lcd_panel_wvga);
    MDDI_LCD_DEBUG("%s: rsp61408_lcd enter sleep mode ,off_ret=%d\n",__func__,ret);
	return ret;
}
/* <DTS2011102904584 qitongliang 20111109 begin */
int rsp61408_set_cabc_backlight(uint32 brightness)
{
	int ret = 0;
	uint32 bl_level = brightness;
	static boolean first_flag = TRUE;
	
	struct pm8058_gpio backlight_drv = 
	{
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 1,
		.pull           = PM_GPIO_PULL_NO,
		/*< DTS2011111404301 qitongliang 20111114 begin */
		/* LCD_PWM 1.8V */
		.vin_sel        = PM_GPIO_VIN_S3,
		/* DTS2011111404301 qitongliang 20111114 end >*/
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_NORMAL,
		.inv_int_pol 	= 1,
	};

	if (TRUE == first_flag)
	{
		first_flag = FALSE;
		
		ret = pm8058_gpio_config(PM_GPIO_24, &backlight_drv);
  	    if (ret) 
  	    {
  		    pr_err("%s PMIC_GPIO_WLAN_EXT_POR config failed\n", __func__);
  		    return ret;
  	    }
  	    pm8058_gpio_set_value(PM_GPIO_24, PM_GPIO_HIGH_VALUE);
	}
	
    
	rsp61408_wvga_write_cabc_brightness_table[2].reg = bl_level;
	ret = process_lcd_table((struct sequence*)&rsp61408_wvga_write_cabc_brightness_table,
                    ARRAY_SIZE(rsp61408_wvga_write_cabc_brightness_table), lcd_panel_wvga);
    return ret;
}
/* DTS2011102904584 qitongliang 20111109 end> */

static int __devinit rsp61408_probe(struct platform_device *pdev)
{
	msm_fb_add_device(pdev);
 	return 0;
}

static struct platform_driver this_driver = {
	.probe  = rsp61408_probe,
	.driver = {
		.name   = "mddi_rsp61408_wvga",
	},
};

static struct msm_fb_panel_data rsp61408_panel_data = {
	.on = rsp61408_lcd_on,
	.off = rsp61408_lcd_off,
	/* <DTS2011102904584 qitongliang 20111109 begin */
	.set_backlight = lcd_backlight_set,
	.set_cabc_brightness = rsp61408_set_cabc_backlight,
	/* DTS2011102904584 qitongliang 20111109 end> */
	/*< DTS2011093001847 qitongliang 20111110 begin */
    .set_dynamic_gamma = rsp61408_set_dynamic_gamma,
	/* DTS2011093001847 qitongliang 20111110 end >*/
};

static struct platform_device this_device = {
	.name   = "mddi_rsp61408_wvga",
	.id	= 0,
	.dev	= {
		.platform_data = &rsp61408_panel_data,
	}
};
static int __init rsp61408_init(void)
{
	int ret = 0;
	struct msm_panel_info *pinfo = NULL;
	bpp_type bpp = MDDI_OUT_24BPP;		
	mddi_type mddi_port_type = mddi_port_type_probe();

	lcd_panel_wvga=lcd_panel_probe();
/*< DTS2011100803621  jiaoshuangwei 20111008 begin */
	if((LCD_RSP61408_CHIMEI_WVGA != lcd_panel_wvga) && (LCD_RSP61408_BYD_WVGA != lcd_panel_wvga))
/* DTS2011100803621 jiaoshuangwei 20111008 end >*/
	{
		return 0;
	}

	MDDI_LCD_DEBUG("%s:start init %s\n",__func__,this_device.name);
	/* Select which bpp accroding MDDI port type */
	if(MDDI_TYPE1 == mddi_port_type)
	{
		bpp = MDDI_OUT_16BPP;
	}
	else if(MDDI_TYPE2 == mddi_port_type)
	{
		bpp = MDDI_OUT_24BPP;
	}
	else
	{
		bpp = MDDI_OUT_16BPP;
	}
	
	ret = platform_driver_register(&this_driver);
	if (!ret) 
	{
		pinfo = &rsp61408_panel_data.panel_info;
		pinfo->xres = 480;
		pinfo->yres = 800;
		pinfo->type = MDDI_PANEL;
		pinfo->pdest = DISPLAY_1;
		pinfo->mddi.vdopkt = MDDI_DEFAULT_PRIM_PIX_ATTR;
		pinfo->wait_cycle = 0;
		pinfo->bpp = (uint32)bpp;
		pinfo->fb_num = 2;
        pinfo->clk_rate = 192000000;
	    pinfo->clk_min = 192000000;
	    pinfo->clk_max = 192000000;
        pinfo->lcd.vsync_enable = TRUE;
        pinfo->lcd.refx100 = 5500;
		pinfo->lcd.v_back_porch = 0;
		pinfo->lcd.v_front_porch = 0;
		pinfo->lcd.v_pulse_width = 22;
		pinfo->lcd.hw_vsync_mode = TRUE;
		pinfo->lcd.vsync_notifier_period = 0;
		pinfo->bl_max = 255;

		ret = platform_device_register(&this_device);
		if (ret)
		{
			platform_driver_unregister(&this_driver);
		}
	}

	return ret;
}
module_init(rsp61408_init);
/* DTS2011090102706 jiaoshuangwei 20110901 end >*/
