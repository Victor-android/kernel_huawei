/*< DTS2011060802925 jiaoshuangwei 20110608 begin */
/* drivers\video\msm\mddi_nt35510_type2.c
 * NT35510 LCD driver for 7x30 platform
 *
 * Copyright (C) 2010 HUAWEI Technology Co., ltd.
 * 
 * Date: 2010/12/07
 * By lijianzhao
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
struct sequence* nt35510_wvga_init_table_type2 = NULL;
static lcd_panel_type lcd_panel_wvga = LCD_NONE;

static const struct sequence nt35510_wvga_standby_exit_table_type2[] = 
{
   /*set the delay time 100ms*/
	{0x1100,0,100}
};

static const struct sequence nt35510_wvga_standby_enter_table_type2[]= 
{
  /*set the delay time 100ms*/
	{0x1000,0,100}
};


static int nt35510_lcd_on_type2(struct platform_device *pdev)
{
	boolean para_debug_flag = FALSE;
    uint32 para_num = 0;
	int ret = 0;
    /* open debug file and read the para */
	switch(lcd_panel_wvga)
	{
		case LCD_NT35510_ALPHA_SI_WVGA_TYPE2:
			para_debug_flag = lcd_debug_malloc_get_para( "nt35510_alpha_si_wvga_init_table_type2", 
	    		(void**)&nt35510_wvga_init_table_type2,&para_num);
			break;
		default:
			break;
	}
	/* If exist the init file ,then init lcd with it for debug */
    if( (TRUE == para_debug_flag)&&(NULL != nt35510_wvga_init_table_type2))
    {
		ret = process_lcd_table(nt35510_wvga_init_table_type2, para_num, lcd_panel_wvga);
    }
    else
    {
		/* Exit Standby Mode */
		ret = process_lcd_table((struct sequence*)&nt35510_wvga_standby_exit_table_type2, 
			ARRAY_SIZE(nt35510_wvga_standby_exit_table_type2), lcd_panel_wvga);
    }
       
	/* Must malloc before,then you can call free */
	if((TRUE == para_debug_flag)&&(NULL != nt35510_wvga_init_table_type2))
	{
		lcd_debug_free_para((void *)nt35510_wvga_init_table_type2);
	}


    MDDI_LCD_DEBUG("%s: nt35510_lcd exit sleep mode ,on_ret=%d\n",__func__,ret);
	return ret;
}

static int nt35510_lcd_off_type2(struct platform_device *pdev)
{
	int ret = 0;
	ret = process_lcd_table((struct sequence*)&nt35510_wvga_standby_enter_table_type2, 
    	      		ARRAY_SIZE(nt35510_wvga_standby_enter_table_type2), lcd_panel_wvga);
    MDDI_LCD_DEBUG("%s: nt35510_lcd_type2 enter sleep mode ,off_ret=%d\n",__func__,ret);
	return ret;
}


static int __devinit nt35510_probe_type2(struct platform_device *pdev)
{
	msm_fb_add_device(pdev);
 	return 0;
}

static struct platform_driver this_driver = {
	.probe  = nt35510_probe_type2,
	.driver = {
		.name   = "mddi_nt35510_wvga_type2",
	},
};

static struct msm_fb_panel_data nt35510_panel_data_type2 = {
	.on = nt35510_lcd_on_type2,
	.off = nt35510_lcd_off_type2,
	.set_backlight = pwm_set_backlight,
};

static struct platform_device this_device = {
	.name   = "mddi_nt35510_wvga_type2",
	.id	= 0,
	.dev	= {
		.platform_data = &nt35510_panel_data_type2,
	}
};
static int __init nt35510_init_type2(void)
{
	int ret = 0;
	struct msm_panel_info *pinfo = NULL;

	bpp_type bpp = MDDI_OUT_16BPP;		
	mddi_type mddi_port_type = mddi_port_type_probe();

	lcd_panel_wvga=lcd_panel_probe();
	
	if(LCD_NT35510_ALPHA_SI_WVGA_TYPE2 != lcd_panel_wvga)
	{
		return 0;
	}
	MDDI_LCD_DEBUG("%s:------nt35510_init_type2------\n",__func__);
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
		pinfo = &nt35510_panel_data_type2.panel_info;
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
module_init(nt35510_init_type2);
/* DTS2011060802925 jiaoshuangwei 20110608 end >*/
