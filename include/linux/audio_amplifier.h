/* < DTS2010061100003 luojianhong 201000612 begin*/
/*
 * include/linux/tpa2028d1_i2c.h - platform data structure for tpa2028d1
 *
 * Copyright (C) 2008 Google, Inc.
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
#ifndef _LINUX_AUDIO_AMPLIFIER_H
#define _LINUX_AUDIO_AMPLIFIER_H


struct amplifier_platform_data {
	void (*amplifier_on)(void);	
    void (*amplifier_off)(void);	
    /*< DTS2010120204486 dongchen 20101223 begin */
    #ifdef CONFIG_HUAWEI_KERNEL
    void (*amplifier_4music_on)(void);
    #endif
    /* DTS2010120204486 dongchen 20101223 end >*/
};

#endif /* _LINUX_AUDIO_AMPLIFIER_H */
/*  DTS2010061100003 luojianhong 201000612 end > */