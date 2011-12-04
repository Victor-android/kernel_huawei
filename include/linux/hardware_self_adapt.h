#ifndef HARDWARE_SELF_ADAPT_H
#define HARDWARE_SELF_ADAPT_H

/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*

                    GPIO/BIO   S E R V I C E S

GENERAL DESCRIPTION

REFERENCES

EXTERNALIZED FUNCTIONS
  None.

INITIALIZATION AND SEQUENCING REQUIREMENTS

Copyright (c) 2009 by HUAWEI, Incorporated.  All Rights Reserved.
*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*/
/*===========================================================================

                      EDIT HISTORY FOR FILE

 This section contains comments describing changes made to this file.
  Notice that changes are listed in reverse chronological order.


when       who      what, where, why
-------------------------------------------------------------------------------
20091123    hqt      creat
=========================================================================== */
/* < DTS2011042801270 yuezenglong 20110428 begin */
#define IC_PM_ON   1
#define IC_PM_OFF  0
#define IC_PM_VDD 2700   /*set gp4 voltage as 2700mV for all*/
/* DTS2011042801270 yuezenglong 20110428 end > */

typedef enum
{
    LCD_S6D74A0_SAMSUNG_HVGA,
    LCD_ILI9325_INNOLUX_QVGA,
    LCD_ILI9325_BYD_QVGA,
    LCD_ILI9325_WINTEK_QVGA,
    LCD_SPFD5408B_KGM_QVGA,

    LCD_HX8357A_BYD_QVGA,
    LCD_HX8368A_SEIKO_QVGA,

    LCD_HX8347D_TRULY_QVGA,
    LCD_ILI9325C_WINTEK_QVGA,    
    
    LCD_HX8368A_TRULY_QVGA,
    /*<BU5D09397 lijuan 00152865, 20100506 begin*/
    LCD_NT35582_TRULY_WVGA = 128,
    LCD_NT35582_BYD_WVGA,
	/*< DTS2010120703279 lijianzhao 20101207 begin */
    LCD_NT35510_ALPHA_SI_WVGA,
	/* DTS2010120703279 lijianzhao 20101207 end >*/
   /*BU5D09397 lijuan 00152865, 20100506 end>*/
/*< DTS2011042601502  jiaoshuangwei 20110308 begin*/
     LCD_NT35560_TOSHIBA_FWVGA,
/* DTS2011042601502  jiaoshuangwei 20110308 end>*/
/*< DTS2011060802925 jiaoshuangwei 20110608 begin */
/*add the LCD_NT35510_ALPHA_SI_WVGA to  panel type list*/
    LCD_NT35510_ALPHA_SI_WVGA_TYPE2,
/* DTS2011060802925 jiaoshuangwei 20110608 end >*/
/*< DTS2011090102706 jiaoshuangwei 20110901 begin */
    LCD_RSP61408_CHIMEI_WVGA,
/* DTS2011090102706 jiaoshuangwei 20110901 end >*/
/*< DTS2011100803621  jiaoshuangwei 20111008 begin */
	LCD_RSP61408_BYD_WVGA,
/* DTS2011100803621 jiaoshuangwei 20111008 end >*/
	
    LCD_MAX_NUM,
    LCD_NONE =0xFF
}lcd_panel_type;

typedef enum
{
    HW_VER_SUB_VA            = 0x0,
    HW_VER_SUB_VB            = 0x1,
    HW_VER_SUB_VC            = 0x2,
    HW_VER_SUB_VD            = 0x3,
    HW_VER_SUB_VE            = 0x4,
    HW_VER_SUB_SURF          = 0xF,
    HW_VER_SUB_MAX           = 0xF
}hw_ver_sub_type;

/* < DTS2011042703449  liujinggang 20110427 begin */
typedef enum
{
	GS_ADIX345 	= 0x01,
	GS_ST35DE	= 0x02,
	GS_ST303DLH = 0X03,
	GS_MMA8452  = 0x04,
	GS_BMA250   = 0x05,
	GS_STLIS3XH	= 0x06,
	GS_ADI346  = 0x07,
}hw_gs_type;
typedef enum
{
	COMPASS_TOP_GS_TOP 			=0,
	COMPASS_TOP_GS_BOTTOM 		=1,
	COMPASS_BOTTOM_GS_TOP 		=2,
	COMPASS_BOTTOM_GS_BOTTOM	=3,
	COMPASS_NONE_GS_BOTTOM		=4,
}compass_gs_position_type;

/* < DTS2011043000257  liujinggang 20110503 begin */
/*add compass platform data and the func of power_switch*/
struct gs_platform_data {
	int (*adapt_fn)(void);	/* fucntion is suported in some product */
	int slave_addr;     /*I2C slave address*/
	int dev_id;         /*who am I*/
	int *init_flag;     /*Init*/
	compass_gs_position_type (*get_compass_gs_position)(void);
	int (*gs_power)(int on);
};

struct compass_platform_data {
	int (*compass_power)(int on);
};
/* DTS2011043000257  liujinggang 20110503 end > */

/* < DTS2011042801270 yuezenglong 20110428 begin */
struct gyro_platform_data {
	u8 fs_range;
	u8 axis_map_x;     /*x map read data[axis_map_x] from i2c*/
	u8 axis_map_y;
	u8 axis_map_z;
	
	u8 negate_x;       /*negative x,y or z*/
	u8 negate_y;
	u8 negate_z;
	int dev_id;        /*who am I*/
	int slave_addr;
	int (*gyro_power)(int on);
};
/* DTS2011042801270 yuezenglong 20110428 end > */	

#define HW_VER_MAIN_MASK (0xFFF0)
#define HW_VER_SUB_MASK  (0x000F)

lcd_panel_type lcd_panel_probe(void);
int board_use_tssc_touch(bool * use_touch_key);
int board_support_ofn(bool * ofn_support);

char *get_compass_gs_position_name(void);
/* DTS2011042703449  liujinggang 20110427 end > */
char *get_lcd_panel_name(void);

hw_ver_sub_type get_hw_sub_board_id(void);
/* <DTS2010071502918 shenjinming 20101008 begin */
#ifdef CONFIG_HUAWEI_POWER_DOWN_CHARGE
unsigned int get_charge_flag(void);
#endif
/* DTS2010071502918 shenjinming 20101008 end> */ 
/* <DTS2011050303216 wangjiongfeng 20110504 begin */
#ifdef CONFIG_HUAWEI_FEATURE_PROXIMITY_EVERLIGHT_APS_9900
#define MSM_7X30_APS9900_INT 89
struct aps9900_hw_platform_data {
    int (*aps9900_power)(int on);
    int (*aps9900_gpio_config_interrupt)(void);
};
#endif
/* <DTS2011050303216 wangjiongfeng 20110504 end */
/* < DTS2011102806486  zhangmin 20111029  begin */
#ifdef CONFIG_HUAWEI_FEATURE_PROXIMITY_LITEON_LTR_558
#define MSM_7X30_LTR558_INT 89
struct ltr558_hw_platform_data {
    int (*ltr558_power)(int on);
    int (*ltr558_gpio_config_interrupt)(void);
};
#endif
/* DTS2011102806486  zhangmin 20111029  end > */
#endif

