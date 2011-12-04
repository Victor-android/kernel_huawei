/*< DTS2010120703279 lijianzhao 20101207 begin */
/* drivers\video\msm\hw_backlight.h
 * backlight driver for 7x30 platform
 *
 * Copyright (C) 2010 HUAWEI Technology Co., ltd.
 * 
 * Date: 2010/12/07
 * By lijianzhao
 * 
 */
#ifndef HW_BACKLIGHT_H
#define HW_BACKLIGHT_H
void pwm_set_backlight (struct msm_fb_data_type * mfd);
/*< DTS2011070504600  sunhonghui 20110706 begin*/
void lcd_backlight_set(struct msm_fb_data_type * mfd);
/* DTS2011070504600  sunhonhui 20110706 end>*/
#endif
/* DTS2010120703279 lijianzhao 20101207 end >*/