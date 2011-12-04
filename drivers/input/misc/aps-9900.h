/* < DTS2011042604384  wangjiongfeng 20110427 begin */
/* drivers/input/misc/aps-9900.h
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

#ifndef _LINUX_APS_9900_H
#define _LINUX_APS_9900_H

#define ECS_IOCTL_APP_SET_DELAY	    _IOW(0xA1, 0x18, short)
#define ECS_IOCTL_APP_GET_DELAY       _IOR(0xA1, 0x30, short)
#define ECS_IOCTL_APP_SET_LFLAG		_IOW(0xA1, 0x1C, short)
#define ECS_IOCTL_APP_GET_LFLAG		_IOR(0xA1, 0x1B, short)
#define ECS_IOCTL_APP_SET_PFLAG		_IOW(0xA1, 0x1E, short)
#define ECS_IOCTL_APP_GET_PFLAG		_IOR(0xA1, 0x1D, short)
/* < DTS2011071500961  liujinggang 20110715 begin */
#define ECS_IOCTL_APP_GET_PDATA_VALVE	_IOR(0xA1, 0x32, short)
#define ECS_IOCTL_APP_GET_LDATA_VALVE	_IOR(0xA1, 0x33, short)
/* DTS2011071500961  liujinggang 20110715 end > */

#define VREG_GP4_NAME	"gp4"
#define VREG_GP4_VOLTAGE_VALUE	2700


#endif /* _LINUX_APS_9900_H */
/* < DTS2011042604384  wangjiongfeng 20110427 end */